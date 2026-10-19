// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2024 Oplus. All rights reserved
 */
#define pr_fmt(fmt) "aizerofs: " fmt

#include "aizerofs_shrink.h"
#include <linux/list.h>
#include <linux/shrinker.h>
#include <linux/spinlock.h>
#include <linux/swap.h>
#include <linux/sched/signal.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/idr.h>
#include <linux/rcupdate.h>
#include <linux/fsnotify.h>
#include <linux/namei.h>
#include <linux/timer.h>
#include "aizerofs_internal.h"

/*
 * The dma-buf cache is used specifically for large AI files to improve their read
 * and write performance.
 * 1. When the dma-buf is allocated for the first time, a dma-buf cache is created
 * for large AI files. When the dma-buf is allocated again, it is searched for
 * relevant pages from dma-buf cache(from front to back). If relevant pages exist,
 * add them directly to the sglist of dma-buf.
 * 2. When reading large AI files to dma-buf, I/O is not required if relevant pages
 * exist in the dma-buf cache.
 * 3. When the dma-buf is released, the page is added to the dma-buf cache.
 * 4. When memory is reclaimed, shrink the pages in the dma-buf cache from
 * the back to the front.
 *
 * dbuf_cache->pages description:
 * find --->
 * 0                            remained_pages-1        total_pages-1
 * |---------------------------|------------------------|
 *                           <--- shrink
 */

MODULE_IMPORT_NS(DMA_BUF);

/* Just to protect muliple rcu writers */
static DEFINE_SPINLOCK(dbuf_cache_lock);
LIST_HEAD(dbuf_cache_list);

static DEFINE_IDR(dbuf_cache_param_idr);
/* idr index must be protected */
static DEFINE_MUTEX(dbuf_cache_param_idr_mutex);

static struct aizerofs_dma_buf_cache *create_dbuf_cache(char *path,
						loff_t pos,
						loff_t size);

static struct aizerofs_dma_buf_cache *
	find_and_prep_destroy_dmabuf_cache_by_inode(struct inode *inode);

static struct aizerofs_dma_buf_cache *find_and_prep_destroy_dmabuf_cache_by_path(char *path);

static int enable_fsnotify = 1;
module_param(enable_fsnotify, int, 0644);
MODULE_PARM_DESC(enable_fsnotify, "enable fsnotify (1 or 0)");

#define SCENE_IMAGE_SEGMENTATION_OR_INPAINTING 1
#define AIZEROFS_SCENE_SHRINK_AVAILABLE_WATER_MARK ((SZ_2G) >> PAGE_SHIFT)
#define DEFAULT_TIMEOUT_TIMES_MS (10 * 1000)

struct timer_list aizerofs_scene_timer;

static int aizerofs_scene = 0;
static void aizerofs_scene_timeout_callback(struct timer_list *t)
{
	aizerofs_scene = 0;
	pr_debug("%s aizerofs_scene:%d!\n", __func__, aizerofs_scene);
}

int aizerofs_set_scene(unsigned long arg)
{
	del_timer(&aizerofs_scene_timer);

	aizerofs_scene = (int)arg;
	pr_debug("%s aizerofs_scene:%d!\n", __func__, aizerofs_scene);

	aizerofs_scene_timer.expires = jiffies + msecs_to_jiffies(DEFAULT_TIMEOUT_TIMES_MS);
	add_timer(&aizerofs_scene_timer);

	return 0;
}

/*
 * This list is used to hold the inode information
 * of the parent directory associated with fsnotify.
 */
static LIST_HEAD(dir_inode_list);
static DEFINE_MUTEX(dir_inode_mutex);
/*
 * Inode information of the parent directory
 * associated with fsnotify.
 *
 * NOTE: This is simply to prevent concurrent
 * association of mark to the parent inode.
 */
struct fsnotify_inode_data {
	struct list_head list;
	struct inode *parent_inode;
	struct fsnotify_mark *mark;
};

/************************** manage param data (due to gki restriction) *************************/
#define AIZEROFS_PATH_MAX 512
/* in/out data from user */
struct aizerofs_param_data {
	int idx; /* out */
	/* in */
	__u64 pos; /* unit: byte */
	__u64 bin_len;
	char bin_path[AIZEROFS_PATH_MAX];
};

typedef struct file *(*filp_open_t)(const char *filename, int flags, umode_t mode);
extern filp_open_t filp_open_dup;

int aizerofs_handle_get_param_idx(unsigned long arg)
{
	struct aizerofs_param_data *param_data;
	unsigned int in_size;
	int ret = 0;

	in_size = sizeof(struct aizerofs_param_data);

	param_data = kmalloc(in_size, GFP_KERNEL | __GFP_NOFAIL);
	if (!param_data)
		return -ENOMEM;

	if (copy_from_user(param_data, (void __user *)arg, in_size) != 0) {
		ret = -EFAULT;
		goto out_free_param_data;
	}

	if (unlikely(strlen(param_data->bin_path) >= AIZEROFS_PATH_MAX)) {
		pr_err("aizerofs: WARNING: func:%s path:%s len:%ld >= %d \n",
			__func__, param_data->bin_path, strlen(param_data->bin_path),
			(unsigned int)AIZEROFS_PATH_MAX);
		kfree(param_data);
		return -EINVAL;
	}

	/* get param_idx */
	mutex_lock(&dbuf_cache_param_idr_mutex);
	param_data->idx = idr_alloc(&dbuf_cache_param_idr, param_data, 0, 0, GFP_KERNEL | __GFP_NOFAIL);
	if (param_data->idx < 0) {
		mutex_unlock(&dbuf_cache_param_idr_mutex);
		ret = param_data->idx;
		goto out_free_param_data;
	}
	mutex_unlock(&dbuf_cache_param_idr_mutex);
	pr_debug("%s:%d idr_alloc param_data->idx:%d \n",
		__func__, __LINE__, param_data->idx);

	if (copy_to_user((void __user *)arg, param_data, in_size) != 0) {
		ret = -EFAULT;
		goto out_free_idr;
	}

	return 0;

out_free_idr:
	idr_remove(&dbuf_cache_param_idr, param_data->idx);
out_free_param_data:
	kfree(param_data);

	return ret;
}

static inline struct aizerofs_param_data *find_param_data_by_idx(int param_idx, bool del_idr)
{
	struct aizerofs_param_data *param_data;

	mutex_lock(&dbuf_cache_param_idr_mutex);
	param_data = (struct aizerofs_param_data *)idr_find(&dbuf_cache_param_idr, param_idx);
	if (!param_data) {
		pr_err("@ %s Warning: param_data(%d) dont exist! @\n", __func__, param_idx);
		return NULL;
	}
	if (del_idr)
		idr_remove(&dbuf_cache_param_idr, param_idx);
	mutex_unlock(&dbuf_cache_param_idr_mutex);

	return param_data;
}

int aizerofs_handle_put_param_idx(int param_idx)
{
	struct aizerofs_param_data *param_data;

	param_data = find_param_data_by_idx(param_idx, true);
	if (!param_data)
		return -ENODEV;

	/* release param_data */
	kfree(param_data);

	pr_debug("%s:%d  free idr and param_data, param_idx:%d! \n",
		__func__, __LINE__, param_idx);

	return 0;
}

/* only be used during system_heap_do_allocate */
static inline struct aizerofs_dma_buf_cache *find_dmabuf_cache_by_idx(int param_idx)
{
	struct aizerofs_param_data *param_data;

	param_data = find_param_data_by_idx(param_idx, false);
	if (!param_data)
		return NULL;

	return find_dmabuf_cache_by_path(param_data->bin_path, true);
}

static struct aizerofs_dma_buf_cache *create_dbuf_cache_by_idx(int param_idx)
{
	struct aizerofs_param_data *param_data;
	struct aizerofs_dma_buf_cache *dbuf_cache;

	param_data = find_param_data_by_idx(param_idx, false);
	if (!param_data)
		return NULL;
	dbuf_cache = create_dbuf_cache(param_data->bin_path, param_data->pos,
				 param_data->bin_len);

	return dbuf_cache;
}

/********************* fsnotify for FS_OPEN_WRITE *********************/
static LIST_HEAD(marks);
static DEFINE_SPINLOCK(marks_list_lock);

struct async_free_marks {
	struct list_head node;
	struct fsnotify_mark *mark;
};

static void free_marks(struct work_struct *work)
{
	struct async_free_marks *m = NULL;

again:
	m = NULL;
	spin_lock(&marks_list_lock);
	if (!list_empty(&marks)) {
		list_for_each_entry(m, &marks, node) {
			list_del(&m->node);
			break;
		}
	}
	spin_unlock(&marks_list_lock);
	if (!m)
		return;

	if (enable_fsnotify) {
		fsnotify_destroy_mark(m->mark, m->mark->group);
		fsnotify_put_group(m->mark->group);
		fsnotify_put_mark(m->mark);
	}

	pr_info("%s dropping marks:%lx", __func__,
		(unsigned long)m->mark);

	kfree(m);
	goto again;
}
static DECLARE_WORK(deferred_remove_marks, free_marks);

static void async_free_mark(struct fsnotify_mark *mark)
{
	struct async_free_marks *m = kzalloc(sizeof(*m), GFP_KERNEL | __GFP_NOFAIL);

	spin_lock(&marks_list_lock);
	m->mark = mark;
	list_add_tail(&m->node, &marks);
	schedule_work(&deferred_remove_marks);
	spin_unlock(&marks_list_lock);
}

static DEFINE_MUTEX(dbuf_cache_fsnotify_mutex);

/*
 * We drop dbuf cache and metadata when file is opened in write mode.
 */
static int aizerofs_fsnotify_handle_event(
		struct fsnotify_group *group,
		u32 mask, const void *data,
		int data_type, struct inode *dir,
		const struct qstr *file_name, u32 cookie,
		struct fsnotify_iter_info *iter_info)
{
	struct inode *inode;
	struct path *path;
	struct aizerofs_dma_buf_cache *dbuf_cache;
	bool is_dir;

	if (!(mask & (FS_OPEN_WRITE | FS_DELETE_SELF | FS_MOVE_SELF)))
		return 0;

	if (mask & FS_OPEN_WRITE) {
		path = (struct path *)data;
		AIZEROFS_BUG_ON(!path);
		inode = path->dentry->d_inode;
	} else {
		inode = (struct inode *)data;
	}
	AIZEROFS_BUG_ON(!inode);
	is_dir = S_ISDIR(inode->i_mode);

	pr_info("%s:%d  FS_OPEN_WRITE:%d FS_DELETE_SELF:%d FS_MOVE_SELF:%d inode:%lx\n",
		__func__, __LINE__, mask & FS_OPEN_WRITE ? 1 : 0,
		mask & FS_DELETE_SELF ? 1 : 0, mask & FS_MOVE_SELF ? 1 : 0,
		(unsigned long)inode);

	mutex_lock(&dbuf_cache_fsnotify_mutex);
retry:
	do {
		dbuf_cache = find_and_prep_destroy_dmabuf_cache_by_inode(inode);
		if (!dbuf_cache) {
			if (is_dir)
				break;

			pr_err("%s failed to find dbuf_cache or another destroying is ongoing\n", __func__);
			mutex_unlock(&dbuf_cache_fsnotify_mutex);
			return -ENOENT;
		}
		if (IS_ERR(dbuf_cache)) {
			/* wait for releasing */
			schedule_timeout_uninterruptible(HZ / 15);
			goto retry;
		}

		dmabuf_cache_shrink(0, PAGE_COUNTER_MAX, dbuf_cache);
		/* arrive here, remained pages should have been zero */
		AIZEROFS_BUG_ON(dbuf_cache->remained_pages);
		dmabuf_cache_destroy(dbuf_cache);
	} while (is_dir);
	mutex_unlock(&dbuf_cache_fsnotify_mutex);

	if (is_dir) {
		struct fsnotify_inode_data *inode_data;
		mutex_lock(&dir_inode_mutex);
		list_for_each_entry(inode_data, &dir_inode_list, list) {
			if (inode_data->parent_inode == inode) {
				/* destroy mark */
				pr_debug("%s:%d  put mark:%lx and group:%lx \n", __func__, __LINE__,
						(unsigned long)inode_data->mark, (unsigned long)inode_data->mark->group);
				async_free_mark(inode_data->mark);
				/* free inode_data */
				list_del(&inode_data->list);
				pr_debug("%s:%d free inode_data:%lx \n", __func__, __LINE__, (unsigned long)inode_data);
				kfree(inode_data);
				break;
			}
		}
		mutex_unlock(&dir_inode_mutex);
	}

	return 0;
}

static void aizerofs_free_mark(struct fsnotify_mark *mark)
{
	pr_debug("%s:%d kfree mark:%lx  refcnt:%d \n", __func__, __LINE__,
		(unsigned long)mark, refcount_read(&mark->refcnt));
	kfree(mark);
}

static const struct fsnotify_ops aizerofs_fsnotify_ops = {
	.handle_event = aizerofs_fsnotify_handle_event,
	.free_mark = aizerofs_free_mark,
};

static int aizerofs_register_fsnotify_mark(void *data,
				struct inode *inode, bool is_parent)
{
	int ret;
	struct fsnotify_group *group;
	struct fsnotify_mark *mark;
	struct fsnotify_inode_data *inode_data;
	struct aizerofs_dma_buf_cache *dbuf_cache;

	if (is_parent)
		inode_data = (struct fsnotify_inode_data *)data;
	else
		dbuf_cache = (struct aizerofs_dma_buf_cache *)data;

	mark = kzalloc(sizeof(struct fsnotify_mark), GFP_KERNEL | __GFP_NOFAIL);
	if (!mark) {
		pr_err("aizerofs: failed to alloc mark\n");
		return -ENOMEM;
	}
	pr_debug("%s:%d alloc mark:%lx  \n", __func__, __LINE__, (unsigned long)mark);

retry:
	group = fsnotify_alloc_group(&aizerofs_fsnotify_ops, 0);
	if (IS_ERR(group)) {
		schedule_timeout_uninterruptible(1);
		goto retry;
	}
	pr_debug("%s:%d alloc group:%lx  \n", __func__, __LINE__, (unsigned long)group);

	fsnotify_init_mark(mark, group);
	if (is_parent)
		mark->mask = FS_OPEN | FS_DELETE_SELF | FS_MOVE_SELF;
	else
		mark->mask = FS_OPEN | FS_OPEN_WRITE | FS_DELETE_SELF | FS_MOVE_SELF;
	ret = fsnotify_add_inode_mark(mark, inode, 0);
	if (ret < 0) {
		pr_err("aizerofs: failed to fsnotify_add_inode_mark\n");
		goto put_group;
	}

	if (is_parent)
		inode_data->mark = mark;
	else
		dbuf_cache->mark = mark;

	pr_info("%s:%d is_parent:%d mark:%lx %s:%lx \n",
		__func__, __LINE__, is_parent, (unsigned long)mark,
		is_parent ? "inode_data->mark" : "dbuf_cache->mark",
		is_parent ? (unsigned long)inode_data->mark :
		(unsigned long)dbuf_cache->mark);

	return 0;

put_group:
	fsnotify_put_group(group);
	fsnotify_put_mark(mark);

	return ret;
}

int dbuf_cache_drop_by_path(char *path)
{
	struct aizerofs_dma_buf_cache *dbuf_cache;

retry:
	dbuf_cache = find_and_prep_destroy_dmabuf_cache_by_path(path);
	if (!dbuf_cache) {
		pr_err_ratelimited("%s failed to find dbuf_cache or another destroying is ongoing for %s\n",
			__func__, path);
		return 0;
	}
	if (IS_ERR(dbuf_cache)) {
		/* wait for releasing */
		schedule_timeout_uninterruptible(HZ / 15);
		goto retry;
	}

	pr_info("%s for %s\n", __func__, path);
	dmabuf_cache_shrink(0, PAGE_COUNTER_MAX, dbuf_cache);
	/* arrive here, remained pages should have been zero */
	AIZEROFS_BUG_ON(dbuf_cache->remained_pages);
	dmabuf_cache_destroy(dbuf_cache);

	return 0;
}

int dbuf_cache_drop_all(void)
{
	struct aizerofs_dma_buf_cache *dbuf_cache;
	char path[AIZEROFS_PATH_MAX];

	/* drop caches and metadata till dbuf_cache_list is empty */
again:
	memset(path, 0, AIZEROFS_PATH_MAX);
	rcu_read_lock();
	list_for_each_entry_rcu(dbuf_cache, &dbuf_cache_list, list) {
		strncpy(path, dbuf_cache->bin_path, AIZEROFS_PATH_MAX - 1);
		break;
	}
	rcu_read_unlock();

	if (path[0] != 0) {
		dbuf_cache_drop_by_path(path);
		goto again;
	}

	return 0;
}

/********************* dma-buf cache operation set * *********************/
struct page *get_page_from_dbuf_cache(struct aizerofs_dma_buf_cache *dbuf_cache,
				unsigned long idx)
{
	if (dbuf_cache && dbuf_cache->remained_pages > 0 &&
	    idx < dbuf_cache->remained_pages)
		return dbuf_cache->pages[idx];

	return NULL;
}
EXPORT_SYMBOL_GPL(get_page_from_dbuf_cache);

inline struct aizerofs_dma_buf_cache *find_dmabuf_cache_by_path(char *path, bool allocate)
{
	struct aizerofs_dma_buf_cache *dbuf_cache = NULL;

	if (!path)
		return NULL;

	rcu_read_lock();
	list_for_each_entry_rcu(dbuf_cache, &dbuf_cache_list, list) {
		if (!strcmp(dbuf_cache->bin_path, path)) {
			if (!allocate) {
				spin_lock(&dbuf_cache->lock);
				if (dbuf_cache->is_destroying) {
					pr_err("ERROR!!!%s search %s during destoying\n", __func__, path);
					spin_unlock(&dbuf_cache->lock);
					dbuf_cache = NULL;
				}
				if (dbuf_cache) {
					spin_unlock(&dbuf_cache->lock);
					pr_info("%s find dbuf_cache for %s allocate:%d\n", __func__, path, allocate);
				}
				rcu_read_unlock();
				return dbuf_cache;
			}

			spin_lock(&dbuf_cache->lock);

			/* dbuf has not been released or dbuf_cache is destroying */
			if (dbuf_cache->flags & DBUF_CACHE_DISABLE_SHRINK) {
				pr_err("ERROR!!!%s new %s allocation before release\n", __func__, path);
				spin_unlock(&dbuf_cache->lock);
				rcu_read_unlock();
				return ERR_PTR(-EBUSY);
			}
			if (dbuf_cache->is_destroying) {
				pr_err("ERROR!!!%s new %s allocation during destoying\n", __func__, path);
				spin_unlock(&dbuf_cache->lock);
				rcu_read_unlock();
				return ERR_PTR(-EBUSY);
			}

			/* Okay, everything is fine */
			dbuf_cache->flags |= DBUF_CACHE_DISABLE_SHRINK;
			dbuf_cache->allocated_pages = 0;
			dbuf_cache->stop_io_worker = 0;
			dbuf_cache->read_pages = dbuf_cache->remained_pages;
			pr_info("%s find dbuf_cache for %s allocate:%d\n", __func__, path, allocate);
			spin_unlock(&dbuf_cache->lock);
			rcu_read_unlock();
			return dbuf_cache;
		}
	}

	rcu_read_unlock();
	return NULL;
}

static struct aizerofs_dma_buf_cache *find_and_prep_destroy_dmabuf_cache_by_path(char *path)
{
	struct aizerofs_dma_buf_cache *dbuf_cache = NULL;

	if (!path)
		return NULL;

	rcu_read_lock();
	list_for_each_entry_rcu(dbuf_cache, &dbuf_cache_list, list) {
		if (!strcmp(dbuf_cache->bin_path, path)) {
			spin_lock(&dbuf_cache->lock);
			if (dbuf_cache->flags & DBUF_CACHE_DISABLE_SHRINK) {
				pr_err_ratelimited("ERROR!!!%s destroy dbuf_cache %s before releasing\n",
					__func__, dbuf_cache->bin_path);
				dbuf_cache->flags |= DBUF_CACHE_DEFERRED_DESTROY;
				spin_unlock(&dbuf_cache->lock);
				continue;
			}

			if (dbuf_cache->is_destroying) {
				pr_err("ERROR!!!%s search %s during destoying\n", __func__, path);
				spin_unlock(&dbuf_cache->lock);
				dbuf_cache = NULL;
			} else {
				pr_info("%s for %s inode:%lx enter stage1\n", __func__, path,
					(unsigned long)dbuf_cache->inode);
				dbuf_cache->is_destroying = DESTROY_STAGE1;
			}
			if (dbuf_cache)
				spin_unlock(&dbuf_cache->lock);
			rcu_read_unlock();
			return dbuf_cache;
		}
	}
	rcu_read_unlock();

	return NULL;
}

static struct aizerofs_dma_buf_cache *find_and_prep_destroy_dmabuf_cache_by_inode(struct inode *inode)
{
	struct aizerofs_dma_buf_cache *dbuf_cache = NULL;

	if (!inode)
		return NULL;

	rcu_read_lock();
	list_for_each_entry_rcu(dbuf_cache, &dbuf_cache_list, list) {
		if (dbuf_cache->inode &&
		    (dbuf_cache->inode == inode ||
		     dbuf_cache->parent_inode == inode)) {
			spin_lock(&dbuf_cache->lock);
			if (dbuf_cache->flags & DBUF_CACHE_DISABLE_SHRINK) {
				pr_err_ratelimited("ERROR!!!%s destroy dbuf_cache %s before releasing\n",
					__func__, dbuf_cache->bin_path);
				dbuf_cache->flags |= DBUF_CACHE_DEFERRED_DESTROY;
				spin_unlock(&dbuf_cache->lock);
				continue;
			}

			if (dbuf_cache->is_destroying) {
				pr_err("ERROR!!!%s search %s during destoying\n", __func__, dbuf_cache->bin_path);
				spin_unlock(&dbuf_cache->lock);
				dbuf_cache = NULL;
			} else {
				pr_info("%s for %s inode:%lx enter stage1\n", __func__, dbuf_cache->bin_path,
					(unsigned long)dbuf_cache->inode);
				dbuf_cache->is_destroying = DESTROY_STAGE1;
			}
			if (dbuf_cache)
				spin_unlock(&dbuf_cache->lock);
			rcu_read_unlock();
			return dbuf_cache;
		}
	}
	rcu_read_unlock();

	return NULL;
}

inline struct aizerofs_dma_buf_cache *find_dmabuf_cache_by_dmabuf(struct dma_buf *dbuf)
{
	struct aizerofs_dma_buf_cache *dbuf_cache;

	rcu_read_lock();
	list_for_each_entry_rcu(dbuf_cache, &dbuf_cache_list, list) {
		if (dbuf_cache->dbuf == dbuf) {
			spin_lock(&dbuf_cache->lock);
			if (dbuf_cache->is_destroying == DESTROY_STAGE2) {
				pr_err("ERROR!!!%s search %s during destoying\n",
					__func__, dbuf_cache->bin_path);
				spin_unlock(&dbuf_cache->lock);
				rcu_read_unlock();
				return NULL;
			}
			spin_unlock(&dbuf_cache->lock);
			rcu_read_unlock();
			return dbuf_cache;
		}
	}
	rcu_read_unlock();

	return NULL;
}

#define BATCH_IO 32
#define BATCH_SIZE (BATCH_IO * PAGE_SIZE)

struct aizerofs_io {
	struct file *filp;
	loff_t pos;
	unsigned long len;
	struct page **pages;
	struct bio_vec bvec[BATCH_IO];
};

bool is_overlayfs(struct file *filp) {
	struct super_block *sb;

	if (!filp) {
		return false;
	}

	sb = filp->f_inode->i_sb;
	if (sb && sb->s_magic == OVERLAYFS_SUPER_MAGIC) {
		return true;
	}

	return false;
}

static int aizerofs_do_batch_io(struct aizerofs_io *io, struct aizerofs_dma_buf_cache *dbuf_cache)
{
	for (unsigned long len = 0; len < io->len; len += BATCH_SIZE) {
		struct bio_vec *bvec = io->bvec;
		struct iov_iter iter;
		struct kiocb kiocb;
		int nr = (io->len - len) >= BATCH_SIZE ?  BATCH_IO : (io->len - len) / PAGE_SIZE;
		int ret;

retry:
		for (int i = 0; i < nr; i++) {
			bvec[i].bv_page = io->pages[(io->pos + len)/ PAGE_SIZE + i];
			if (!bvec[i].bv_page)
				pr_err("!!!!!!!ERROR %s for %s pos: %ld allocated_pages %ld read_pages:%lx nr_reclaimed:%ld flags:%lx\n",
					__func__, dbuf_cache->bin_path,
					(unsigned long)((io->pos + len)/ PAGE_SIZE + i),
					dbuf_cache->allocated_pages,
					dbuf_cache->read_pages,
					dbuf_cache->nr_reclaimed,
					dbuf_cache->flags);
			bvec[i].bv_offset = 0;
			bvec[i].bv_len = PAGE_SIZE;
		}

		init_sync_kiocb(&kiocb, io->filp);
		kiocb.ki_pos = io->pos + len;
		iov_iter_bvec(&iter, ITER_DEST, bvec, nr, nr * PAGE_SIZE);

		ret = io->filp->f_op->read_iter(&kiocb, &iter);
		if (ret < 0 && ret != -EAGAIN) {
			pr_err("%s-%s io error at %llx ret:%d\n",
				__func__, current->comm, io->pos, ret);
			return ret;
		}
		if (ret == -EAGAIN) {
			schedule_timeout_uninterruptible(1);
			pr_err_ratelimited("%s IO retry @%llx for %s\n",
				__func__, io->pos, dbuf_cache->bin_path);
			goto retry;
		}
	}

	return 0;
}

static void io_work(struct work_struct *work)
{
	struct file *bin_file;
	unsigned long pos, offs;
	struct aizerofs_dma_buf_cache *dbuf_cache = container_of(work, struct aizerofs_dma_buf_cache,
			io_worker);
	int ret;
	const struct cred *cred;
	const struct cred *old_cred;

	/* everything is still in memory */
	if (dbuf_cache->remained_pages == dbuf_cache->total_pages) {
		pr_info("%s bypass async read: dbuf_cache:%lx remained_pages:%ld read_pages:%ld total_pages:%ld\n",
			__func__, (unsigned long)dbuf_cache, dbuf_cache->remained_pages, dbuf_cache->read_pages,
			dbuf_cache->total_pages);
		return;
	}

	/* Use READ_ONCE to ensure we get a consistent value */
	cred = READ_ONCE(dbuf_cache->cred);
	if (!cred) {
		pr_err("cred is NULL\n");
		return;
	}

	old_cred = override_creds(cred);
	pr_info("override_dbuf_cache_creds\n");

	bin_file = filp_open_dup(dbuf_cache->bin_path, O_RDONLY | O_DIRECT, 0);
	if (IS_ERR(bin_file)) {
		pr_err("%s failed to open %s: %ld\n", __func__, dbuf_cache->bin_path, PTR_ERR(bin_file));
		goto out_cred;
	}

	offs = dbuf_cache->pos / PAGE_SIZE;
	pos = offs + dbuf_cache->remained_pages;

	pr_info("%s start async read: dbuf_cache:%lx pos:%lx offs:%lx remained_pages:%ld read_pages:%ld total_pages:%ld allocated_pages:%ld\n",
		__func__, (unsigned long)dbuf_cache, pos, offs, dbuf_cache->remained_pages, dbuf_cache->read_pages,
		dbuf_cache->total_pages, dbuf_cache->allocated_pages);

	while (1) {
		if (dbuf_cache->allocated_pages >= dbuf_cache->remained_pages &&
		    dbuf_cache->allocated_pages  - (pos - offs) >= BATCH_IO) {
			struct aizerofs_io io = {
				.filp = bin_file,
				.pos = pos * PAGE_SIZE,
				.len = BATCH_SIZE,
				.pages = dbuf_cache->pages,
			};
			pr_debug("%s allocated:%lx pos:%lx len:%lx nr:%d\n", __func__, dbuf_cache->allocated_pages, pos, io.len, BATCH_IO);
			ret = aizerofs_do_batch_io(&io, dbuf_cache);
			if (ret) {
				pr_err("%s failed to read pos:%lx len:%lx\n", __func__, pos, io.len);
				break;
			}
			pos += BATCH_IO;
			dbuf_cache->read_pages += BATCH_IO;
		} else if (dbuf_cache->allocated_pages == dbuf_cache->total_pages &&
			   (pos - offs) < dbuf_cache->total_pages) {
			int nr = dbuf_cache->total_pages - (pos - offs);
			struct aizerofs_io io = {
				.filp = bin_file,
				.pos = pos * PAGE_SIZE,
				.len = nr * PAGE_SIZE,
				.pages = dbuf_cache->pages,
			};
			pr_debug("%s allocated:%lx pos:%lx len:%lx nr:%d\n", __func__, dbuf_cache->allocated_pages, pos, io.len, nr);
			ret = aizerofs_do_batch_io(&io, dbuf_cache);
			if (ret) {
				pr_err("%s failed to read pos:%lx len:%lx\n", __func__, pos, io.len);
				break;
			}
			pos += nr;
			dbuf_cache->read_pages += nr;
		} else {
			schedule_timeout_uninterruptible(2);
		}

		if (dbuf_cache->total_pages - (pos - offs) == 0 ||
		    dbuf_cache->stop_io_worker) {
			pr_info("%s end async read, remained_pages:%ld read_pages:%ld total_pages:%ld\n",
				__func__, dbuf_cache->remained_pages, dbuf_cache->read_pages,
				dbuf_cache->total_pages);
			break;
		}
	}
	filp_close(bin_file, NULL);
out_cred:
	revert_creds(old_cred);
}

static inline int register_fsnotify_for_parent_dir(
		struct aizerofs_dma_buf_cache *dbuf_cache, struct path *path)
{
	int ret;
	struct dentry *parent_dentry;
	struct inode *parent_inode;
	struct fsnotify_inode_data *inode_data;

	mutex_lock(&dir_inode_mutex);
	parent_dentry = path->dentry->d_parent;
	parent_inode = parent_dentry->d_inode;
	ihold(parent_inode);
	/* First we check if anyone has registered mark to parent_inode */
	if (!list_empty(&dir_inode_list)) {
		list_for_each_entry(inode_data, &dir_inode_list, list) {
			if (inode_data->parent_inode == parent_inode) {
				/* we don't set dbuf_cache->parent_mark! */
				spin_lock(&dbuf_cache->lock);
				dbuf_cache->parent_inode = parent_inode;
				spin_unlock(&dbuf_cache->lock);
				iput(parent_inode);
				mutex_unlock(&dir_inode_mutex);
				return 0;
			}
		}
	}
	inode_data = kzalloc(sizeof(struct fsnotify_inode_data), GFP_KERNEL | __GFP_NOFAIL);
	pr_debug("%s:%d  alloc inode_data:%lx \n", __func__, __LINE__, (unsigned long)inode_data);
	AIZEROFS_BUG_ON(!inode_data);
	inode_data->parent_inode = parent_inode;
	list_add_tail(&inode_data->list, &dir_inode_list);

	ret = aizerofs_register_fsnotify_mark((void *)inode_data, parent_inode, true);
	if (ret) {
		iput(parent_inode);
		pr_err("%s %d failed to register_fsnotify ret:%d\n",
				__func__, __LINE__, ret);
		return ret;
	}
	spin_lock(&dbuf_cache->lock);
	dbuf_cache->parent_inode = parent_inode;
	spin_unlock(&dbuf_cache->lock);
	iput(parent_inode);
	mutex_unlock(&dir_inode_mutex);
	return 0;
}

static int prepare_register_fsnotify(char *path, struct aizerofs_dma_buf_cache *dbuf_cache)
{
	struct path _path;
	struct inode *inode;
	int ret;

	ret = kern_path(path, LOOKUP_FOLLOW, &_path);
	if (ret) {
		pr_err("%s failed to open path:%s ret:%d\n", __func__, path, ret);
		return -EINVAL;
	}
	inode = d_inode(_path.dentry);
	ihold(inode);
	ret = aizerofs_register_fsnotify_mark((void *)dbuf_cache, inode, false);
	if (ret) {
		pr_err("%s %d failed to register_fsnotify:%s ret:%d\n",
			__func__, __LINE__, path, ret);
		goto put_path;
	}
	dbuf_cache->inode = inode;

	/* register fsnotify mark for parent dir */
	ret = register_fsnotify_for_parent_dir(dbuf_cache, &_path);
	if (ret)
		goto put_mark;

	path_put(&_path);
	iput(inode);

	return 0;

put_mark:
	fsnotify_destroy_mark(dbuf_cache->mark, dbuf_cache->mark->group);
	fsnotify_put_group(dbuf_cache->mark->group);
	fsnotify_put_mark(dbuf_cache->mark);

put_path:
	path_put(&_path);
	iput(inode);

	return ret;
}

static struct aizerofs_dma_buf_cache *create_dbuf_cache(char *path,
						loff_t pos,
						loff_t size)
{
	unsigned int nr_pages;
	struct aizerofs_dma_buf_cache *dbuf_cache;

	if (!path)
		return NULL;

	/* create new dbuf_cache */
	dbuf_cache = kzalloc(sizeof(*dbuf_cache), GFP_KERNEL | __GFP_NOFAIL);
	memset(dbuf_cache, 0, sizeof(*dbuf_cache));

	/* add data to dbuf_cache */
	nr_pages = DIV_ROUND_UP(size, PAGE_SIZE);
	dbuf_cache->pages = __vmalloc_array(nr_pages, sizeof(struct page *),
							GFP_KERNEL | __GFP_NOFAIL);
	dbuf_cache->pos = pos;
	dbuf_cache->total_pages = nr_pages;
	strncpy(dbuf_cache->bin_path, path, sizeof(dbuf_cache->bin_path) - 1);
	memset(dbuf_cache->pages, 0, nr_pages * sizeof(struct page *));
	spin_lock_init(&dbuf_cache->lock);
	/* forbit shrink..., until call handle_dbuf_cache_release */
	dbuf_cache->flags |= DBUF_CACHE_DISABLE_SHRINK;
	smp_mb();

	if (enable_fsnotify) {
		if (prepare_register_fsnotify(path, dbuf_cache)) {
			pr_err("%s failed to prepare_register_fsnotify path:%s \n",
				__func__, path);
			kvfree(dbuf_cache->pages);
			kfree(dbuf_cache);
			return NULL;
		}
	}

	spin_lock(&dbuf_cache_lock);
	list_add_tail_rcu(&dbuf_cache->list, &dbuf_cache_list);
	spin_unlock(&dbuf_cache_lock);

	pr_info("%s create dbuf_cache for %s total_pages:%ld pages:%lx [0]:%lx [nr_pages -1]:%lx\n",
		__func__, path, dbuf_cache->total_pages, (unsigned long)dbuf_cache->pages,
		(unsigned long)dbuf_cache->pages[0], (unsigned long)dbuf_cache->pages[nr_pages - 1]);

	return dbuf_cache;
}

/********************* public hook func used by xxx_heap, eg system_heap *********************/
struct system_heap_buffer {
	struct dma_heap *heap;
	struct list_head attachments;
	struct mutex lock;
	unsigned long len;
	struct sg_table sg_table;
	int vmap_cnt;
	void *vaddr;

	bool uncached;
};

bool handle_dbuf_cache_release(struct dma_buf *dmabuf)
{
	struct aizerofs_dma_buf_cache *dbuf_cache;

	if (!aizerofs_enabled())
		return false;

	dbuf_cache = find_dmabuf_cache_by_dmabuf(dmabuf);
	if (!dbuf_cache)
		return false;

	spin_lock(&dbuf_cache->lock);
	/*
	 * AI app crashes before Async_io has been asked to stop by
	 * direct pass-through
	 */
	if (!dbuf_cache->stop_io_worker) {
		int i;
		struct system_heap_buffer *buffer = dmabuf->priv;
		struct sg_table *table;
		struct scatterlist *sg;

		dbuf_cache->dbuf = NULL;
		/* We must wait for async io to complete! */
		dbuf_cache->stop_io_worker = 1;
		spin_unlock(&dbuf_cache->lock);
		flush_work(&dbuf_cache->io_worker);

		table = &buffer->sg_table;
		for_each_sgtable_sg(table, sg, i) {
			struct page *page = sg_page(sg);

			__free_pages(page, compound_order(page));
		}

		spin_lock(&dbuf_cache->lock);
		dbuf_cache->read_pages = 0;
		dbuf_cache->allocated_pages = 0;
		dbuf_cache->remained_pages = 0;
		memset(dbuf_cache->pages, 0, dbuf_cache->total_pages * sizeof(struct page *));
		/* for drop metadata */
		dbuf_cache->flags &= ~DBUF_CACHE_DISABLE_SHRINK;

		pr_info("%s:%d(non-stopped-io) for %s remained_pages:%ld total_pages:%ld allocated_pages:%ld read_pages:%ld stop_io_worker:%d\n",
				__func__, __LINE__, dbuf_cache->bin_path, dbuf_cache->remained_pages, dbuf_cache->total_pages,
				dbuf_cache->allocated_pages, dbuf_cache->read_pages, dbuf_cache->stop_io_worker);
		spin_unlock(&dbuf_cache->lock);

		atomic64_inc(&aizerofs_perf_stat.kill_stat[AIZEROFS_RELEASE_HIT_KILL_WITH_AIO]);
	} else {
		if (dbuf_cache->flags & DBUF_CACHE_THREAD_IO_ERR || dbuf_cache->flags & DBUF_CACHE_DEFERRED_DESTROY) {
			int i;
			struct system_heap_buffer *buffer = dmabuf->priv;
			struct sg_table *table;
			struct scatterlist *sg;

			if (dbuf_cache->flags & DBUF_CACHE_DEFERRED_DESTROY)
				dbuf_cache->is_destroying = DESTROY_STAGE1;

			spin_unlock(&dbuf_cache->lock);

			table = &buffer->sg_table;
			for_each_sgtable_sg(table, sg, i) {
				struct page *page = sg_page(sg);

				__free_pages(page, compound_order(page));
			}

			spin_lock(&dbuf_cache->lock);

			pr_info("%s:%d(destroy) for %s remained_pages:%ld total_pages:%ld allocated_pages:%ld read_pages:%ld stop_io_worker:%d\n",
				__func__, __LINE__, dbuf_cache->bin_path, dbuf_cache->remained_pages, dbuf_cache->total_pages,
				dbuf_cache->allocated_pages, dbuf_cache->read_pages, dbuf_cache->stop_io_worker);
			dbuf_cache->dbuf = NULL;
			dbuf_cache->read_pages = 0;
			dbuf_cache->allocated_pages = 0;
			dbuf_cache->remained_pages = 0;
			memset(dbuf_cache->pages, 0, dbuf_cache->total_pages * sizeof(struct page *));
			dbuf_cache->flags &= ~DBUF_CACHE_THREAD_IO_ERR;
			/* for drop metadata */
			dbuf_cache->flags &= ~DBUF_CACHE_DISABLE_SHRINK;
			spin_unlock(&dbuf_cache->lock);
			if (dbuf_cache->flags & DBUF_CACHE_DEFERRED_DESTROY) {
				mutex_lock(&dbuf_cache_fsnotify_mutex);
				dmabuf_cache_destroy(dbuf_cache);
				mutex_unlock(&dbuf_cache_fsnotify_mutex);
			}

			return true;
		}

		pr_info("%s:%d(normal) for %s remained_pages:%ld total_pages:%ld allocated_pages:%ld read_pages:%ld stop_io_worker:%d\n",
				__func__, __LINE__, dbuf_cache->bin_path, dbuf_cache->remained_pages, dbuf_cache->total_pages,
				dbuf_cache->allocated_pages, dbuf_cache->read_pages, dbuf_cache->stop_io_worker);
		dbuf_cache->remained_pages = dbuf_cache->total_pages;
		dbuf_cache->dbuf = NULL;
		dbuf_cache->flags &= ~DBUF_CACHE_DISABLE_SHRINK;
		/* we have only one NUMA node */
		mod_node_page_state(page_pgdat(dbuf_cache->pages[0]),
				NR_KERNEL_MISC_RECLAIMABLE, dbuf_cache->total_pages);
		spin_unlock(&dbuf_cache->lock);
	}

#if AIZEROFS_DISABLE_DCACHE
	dmabuf_cache_shrink(0, PAGE_COUNTER_MAX, dbuf_cache);
#endif
	return true;
}
EXPORT_SYMBOL_GPL(handle_dbuf_cache_release);

struct aizerofs_dma_buf_cache *find_or_create_dbuf_cache(unsigned long *len)
{
	struct aizerofs_dma_buf_cache *dbuf_cache = NULL;
	/* decode param_idx from len */
	unsigned long real_len = *len & (SZ_4G - 1);
	/* Node: (1 << 63) for identify encode! */
	unsigned int param_idx = (*len >> 32) & (~(1 << 31));

	if (*len & BIT(63)) {
		*len = real_len;

		if (!aizerofs_enabled()) {
			pr_err("!!!!FIXME: call dbuf_cache allocation after disabling aizerofs\n");
			aizerofs_handle_put_param_idx(param_idx);
			return ERR_PTR(-ENOMEM);
		}

		/* case: use dbuf cache */
		dbuf_cache = find_dmabuf_cache_by_idx(param_idx);
		if (!dbuf_cache) {
			/* first: dbuf_cache->dbuf set null */
			dbuf_cache = create_dbuf_cache_by_idx(param_idx);
			if (!dbuf_cache) {
				pr_err("FIXME: fail to create_dbuf_cache\n");
				aizerofs_handle_put_param_idx(param_idx);
				return ERR_PTR(-ENOMEM);
			}
			INIT_WORK(&dbuf_cache->io_worker, io_work);
		} else if (IS_ERR(dbuf_cache)) {
			aizerofs_handle_put_param_idx(param_idx);
			goto out;
		}

		aizerofs_handle_put_param_idx(param_idx);
		/* Flush any pending io_work before updating cred to avoid use-after-free */
		flush_work(&dbuf_cache->io_worker);
		spin_lock(&dbuf_cache->lock);
		if (dbuf_cache->cred)
			put_cred(dbuf_cache->cred);
		dbuf_cache->cred = prepare_kernel_cred(current);
		spin_unlock(&dbuf_cache->lock);
		schedule_work(&dbuf_cache->io_worker);
	}

out:
	return dbuf_cache;
}
EXPORT_SYMBOL_GPL(find_or_create_dbuf_cache);

void dbuf_cache_add_pages(struct aizerofs_dma_buf_cache *dbuf_cache,
				 struct page *page, unsigned long idx)
{
	int j;
	unsigned long nr;

	if (!dbuf_cache)
		return;

	/* All pages from sglist of dma-buf are added to dbuf_cache->pages */
	spin_lock(&dbuf_cache->lock);
	nr = compound_nr(page);
	if (!dbuf_cache->pages[idx]) {
		for (j = 0; j < nr; j++)
			dbuf_cache->pages[idx + j] = page + j;
	}
	dbuf_cache->allocated_pages += nr;
	spin_unlock(&dbuf_cache->lock);
}
EXPORT_SYMBOL_GPL(dbuf_cache_add_pages);

void dbuf_cache_init_dbuf(struct aizerofs_dma_buf_cache *dbuf_cache,
				 struct dma_buf *dbuf)
{
	if (!aizerofs_enabled() || !dbuf_cache)
		return;

	/* reset/record dbuf_cache->dbuf */
	spin_lock(&dbuf_cache->lock);
	dbuf_cache->dbuf = dbuf;
	mod_node_page_state(page_pgdat(dbuf_cache->pages[0]),
			NR_KERNEL_MISC_RECLAIMABLE, -dbuf_cache->remained_pages);
	spin_unlock(&dbuf_cache->lock);
}
EXPORT_SYMBOL_GPL(dbuf_cache_init_dbuf);


void dbuf_cache_terminate_io_worker(struct aizerofs_dma_buf_cache *dbuf_cache)
{
	if (!aizerofs_enabled() || !dbuf_cache)
		return;

	spin_lock(&dbuf_cache->lock);
	dbuf_cache->dbuf = NULL;
	if (!dbuf_cache->stop_io_worker) {
		/* We must wait for async io to complete! */
		dbuf_cache->stop_io_worker = 1;
		spin_unlock(&dbuf_cache->lock);
		flush_work(&dbuf_cache->io_worker);

		spin_lock(&dbuf_cache->lock);
		dbuf_cache->read_pages = 0;
		dbuf_cache->allocated_pages = 0;
		dbuf_cache->remained_pages = 0;
		/*
		 * NOTE: we need clear dbuf_cache->pages,
		 * check in dbuf_cache_add_pages.
		 */
		memset(dbuf_cache->pages, 0,
			dbuf_cache->total_pages * sizeof(struct page *));
		/* for drop metadata */
		dbuf_cache->flags &= ~DBUF_CACHE_DISABLE_SHRINK;
		spin_unlock(&dbuf_cache->lock);

		atomic64_inc(&aizerofs_perf_stat.kill_stat[AIZEROFS_ALLOC_HIT_KILL_WITH_AIO]);
	} else {
		dbuf_cache->allocated_pages = 0;
		dbuf_cache->flags &= ~DBUF_CACHE_DISABLE_SHRINK;
		spin_unlock(&dbuf_cache->lock);
		atomic64_inc(&aizerofs_perf_stat.kill_stat[AIZEROFS_ALLOC_HIT_KILL_WITHOUT_AIO]);
	}

	pr_info("%s:%d total_pages:%ld read_pages:%ld remained_pages:%ld stop_io_worker:%d \n",
		__func__, __LINE__, dbuf_cache->total_pages, dbuf_cache->read_pages,
		dbuf_cache->remained_pages, dbuf_cache->stop_io_worker);
}
EXPORT_SYMBOL_GPL(dbuf_cache_terminate_io_worker);

/********************* shrink func *********************/
void dmabuf_cache_destroy(struct aizerofs_dma_buf_cache *target_cache)
{
	struct aizerofs_dma_buf_cache *dbuf_cache, *sync_dbuf_cache = NULL;

again:
	spin_lock(&dbuf_cache_lock);
	if (target_cache) {
		spin_lock(&target_cache->lock);
		if (!(target_cache->flags & DBUF_CACHE_DISABLE_SHRINK)
		    && !target_cache->remained_pages &&
		    target_cache->is_destroying ==  DESTROY_STAGE1) {
			pr_info("%s for %s inode:%lx enter stage2\n", __func__, target_cache->bin_path,
				(unsigned long)target_cache->inode);
			target_cache->is_destroying = DESTROY_STAGE2;
			sync_dbuf_cache = target_cache;
			list_del_rcu(&target_cache->list);
		}
		spin_unlock(&target_cache->lock);
		goto unlock;
	}

	if (!list_empty(&dbuf_cache_list)) {
		list_for_each_entry_rcu(dbuf_cache, &dbuf_cache_list, list) {
			spin_lock(&dbuf_cache->lock);
			if (!(dbuf_cache->flags & DBUF_CACHE_DISABLE_SHRINK)
					&& !dbuf_cache->remained_pages &&
					dbuf_cache->is_destroying == DESTROY_STAGE1) {
				pr_info("%s for %s inode:%lx enter stage2\n", __func__, dbuf_cache->bin_path,
						(unsigned long)dbuf_cache->inode);
				sync_dbuf_cache = dbuf_cache;
				dbuf_cache->is_destroying = DESTROY_STAGE2;
				list_del_rcu(&dbuf_cache->list);
				spin_unlock(&dbuf_cache->lock);
				break;
			}
			spin_unlock(&dbuf_cache->lock);
		}
	}

unlock:
	spin_unlock(&dbuf_cache_lock);

	if (sync_dbuf_cache) {
		synchronize_rcu();
		pr_info("%s dropping metadata for %s marks:%lx \n", __func__,
			sync_dbuf_cache->bin_path, (unsigned long)sync_dbuf_cache->mark);
		async_free_mark(sync_dbuf_cache->mark);
		vfree(sync_dbuf_cache->pages);
		if (sync_dbuf_cache->cred)
			put_cred(sync_dbuf_cache->cred);
		kfree(sync_dbuf_cache);
		/* we are destroying all */
		if (!target_cache) {
			sync_dbuf_cache = NULL;
			goto again;
		}
	}
}

unsigned long dmabuf_cache_shrink(gfp_t gfp_mask, unsigned long nr_to_scan,
				 struct aizerofs_dma_buf_cache *target_cache)
{
	unsigned long nr_total = 0;
	struct page *page, *head;
	unsigned int order, nr;
	struct aizerofs_dma_buf_cache *dbuf_cache;
	int only_scan = 0;
	unsigned long last_valid_idx, idx;

	if (!nr_to_scan)
		only_scan = 1;

	rcu_read_lock();
	list_for_each_entry_rcu(dbuf_cache, &dbuf_cache_list, list) {
		if (only_scan) {
			if (!(READ_ONCE(dbuf_cache->flags) & DBUF_CACHE_DISABLE_SHRINK))
				nr_total += dbuf_cache->remained_pages;
		} else {
			if (target_cache && dbuf_cache != target_cache)
				continue;

			while (nr_total < nr_to_scan) {
				spin_lock(&dbuf_cache->lock);
				if ((READ_ONCE(dbuf_cache->flags) & DBUF_CACHE_DISABLE_SHRINK) ||
				     READ_ONCE(dbuf_cache->remained_pages) == 0 ||
				     READ_ONCE(dbuf_cache->is_destroying) == DESTROY_STAGE2) {
					spin_unlock(&dbuf_cache->lock);
					break;
				}

				last_valid_idx = dbuf_cache->remained_pages - 1;
				page = dbuf_cache->pages[last_valid_idx];
				AIZEROFS_BUG_ON(!page);
				head = compound_head(page);
				order = compound_order(head);
				nr = compound_nr(head);

				for (idx = last_valid_idx - nr + 1; idx <= last_valid_idx; idx++)
					dbuf_cache->pages[idx] = NULL;
				dbuf_cache->remained_pages -= nr;
				dbuf_cache->nr_reclaimed += nr;

				spin_unlock(&dbuf_cache->lock);

				mod_node_page_state(page_pgdat(head), NR_KERNEL_MISC_RECLAIMABLE, -nr);
				__free_pages(head, order);

				nr_total += nr;
			}
		}
	}
	rcu_read_unlock();

	return nr_total ? nr_total : SHRINK_STOP;
}

static unsigned long dmabuf_cache_shrink_count(struct shrinker *shrinker,
		struct shrink_control *sc)
{
	unsigned long ret;
	unsigned long available;

	/*Don't shrink when in aizerofs scene and available is engough.*/
	available = si_mem_available();
	if ((aizerofs_scene == SCENE_IMAGE_SEGMENTATION_OR_INPAINTING) &&
		available > AIZEROFS_SCENE_SHRINK_AVAILABLE_WATER_MARK)
		return 0;

	ret = dmabuf_cache_shrink(sc->gfp_mask, 0, NULL);
	return (ret == SHRINK_STOP) ? 0 : ret;
}

static unsigned long dmabuf_cache_shrink_scan(struct shrinker *shrinker,
		struct shrink_control *sc)
{
	if (sc->nr_to_scan == 0)
		return 0;
	return dmabuf_cache_shrink(sc->gfp_mask, sc->nr_to_scan, NULL);
}

struct shrinker dmabuf_cache_shrinker = {
	.count_objects = dmabuf_cache_shrink_count,
	.scan_objects = dmabuf_cache_shrink_scan,
	.seeks = DEFAULT_SEEKS,
	.batch = 0,
};

static int proc_dma_buf_cache_stat_show(struct seq_file *s, void *v)
{
	long i = 0, tot_size, valid_size, read_size;
	struct aizerofs_dma_buf_cache *dbuf_cache;

	seq_puts(s, "**************** Dma_buf Cache Statistics *****************\n");
	seq_puts(s, "aizerofs_kill_stat:\n");
	for (i = 0; i < NR_AIZEROFS_KILL_STAT_ITEMS; i++)
		seq_printf(s, " %s: %llu\n", aizerofs_kill_stat_text[i],
				atomic64_read(&aizerofs_perf_stat.kill_stat[i]));
	seq_puts(s, "\n");

	rcu_read_lock();
	list_for_each_entry_rcu(dbuf_cache, &dbuf_cache_list, list) {
		tot_size = dbuf_cache->total_pages << PAGE_SHIFT;
		valid_size = dbuf_cache->remained_pages << PAGE_SHIFT;
		read_size = dbuf_cache->read_pages << PAGE_SHIFT;
		seq_printf(s, "dbuf_cache:%ld bin_path:%s pos:%lld(byte)-%lld(page) dma_buf:%lx\n",
				i++, dbuf_cache->bin_path, dbuf_cache->pos, dbuf_cache->pos >> PAGE_SHIFT,
				(unsigned long)dbuf_cache->dbuf);
		seq_printf(s, "  STATE -> disable_shrink:%d reclaimed:%ld\n",
				dbuf_cache->flags & DBUF_CACHE_DISABLE_SHRINK ? 1 : 0, dbuf_cache->nr_reclaimed);
		seq_printf(s, "  SIZE -> tot:[%ld page (%ld M) (%ld G)] valid:[%ld page (%ld M) (%ld G)] read:[%ld page (%ld M) (%ld G)]\n\n",
				dbuf_cache->total_pages, tot_size / SZ_1M, tot_size / SZ_1G,
				dbuf_cache->remained_pages, valid_size / SZ_1M, valid_size / SZ_1G,
				dbuf_cache->read_pages, read_size / SZ_1M, read_size /  SZ_1G);
	}
	rcu_read_unlock();

	return 0;
}

void dmabuf_caches_destroy_all(void)
{
	struct aizerofs_dma_buf_cache *dbuf_cache;
	char path[AIZEROFS_PATH_MAX];

	if (aizerofs_enabled())
		return;

	/* drop caches and metadata till dbuf_cache_list is empty */
again:
	memset(path, 0, AIZEROFS_PATH_MAX);
	rcu_read_lock();
	list_for_each_entry_rcu(dbuf_cache, &dbuf_cache_list, list) {
		spin_lock(&dbuf_cache->lock);
		/*
		 * for dynamic switching off aizerofs, dma_buf's release() will
		 * destroy the releasing dbuf
		 */
		if (dbuf_cache->flags & DBUF_CACHE_DISABLE_SHRINK) {
			spin_unlock(&dbuf_cache->lock);
			continue;
		}
		spin_unlock(&dbuf_cache->lock);
		strncpy(path, dbuf_cache->bin_path, AIZEROFS_PATH_MAX - 1);
		break;
	}
	rcu_read_unlock();

	if (path[0] != 0) {
		dbuf_cache_drop_by_path(path);
		goto again;
	}
}
EXPORT_SYMBOL_GPL(dmabuf_caches_destroy_all);

int dmabuf_cache_init(void)
{
	int ret;
	struct proc_dir_entry *root_dir;

	timer_setup(&aizerofs_scene_timer, aizerofs_scene_timeout_callback, 0);

	ret = register_shrinker(&dmabuf_cache_shrinker, "dmabuf-cache-shrinker");
	if (ret) {
		pr_err("%s fail to register_shrinker for dmabuf_cache_shrinker\n", __func__);
	}
	pr_err("success to register_shrinker for dmabuf_cache_shrinker\n");

	root_dir = proc_mkdir("dma_buf_cache", NULL);
	if (!root_dir) {
		pr_err("%s fail to proc_mkdir for dmabuf-cache\n", __func__);
	}

	proc_create_single("dma_buf_cache_stat", 0400,
			root_dir, proc_dma_buf_cache_stat_show);

	return ret;
}
