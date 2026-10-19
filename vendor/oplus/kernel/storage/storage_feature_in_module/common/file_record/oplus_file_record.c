// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025-2026 Oplus. All rights reserved.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/async.h>
#include <trace/events/filemap.h>
#include <soc/oplus/boot/oplus_project.h>
#define PROC_NAME "io_file_record"
#define TASK_COMM_LEN 16
#define MAX_PROCESSES_ENTRY 32

typedef int (*tracing_is_on_t)(void);
static tracing_is_on_t tracing_is_on_dup = NULL;
#define FOR_EACH_INTEREST(i) \
	for (i = 0; i < sizeof(interests) / sizeof(struct tracepoints_table); \
	i++)

#define TRACING_MARK_BUF_SIZE 256

#define tracing_mark(fmt, args...) \
do { \
	char buf[TRACING_MARK_BUF_SIZE]; \
	snprintf(buf, TRACING_MARK_BUF_SIZE, "B|" fmt, ##args); \
	tracing_mark_write(buf); \
	snprintf(buf, TRACING_MARK_BUF_SIZE, "E|" fmt, ##args); \
	tracing_mark_write(buf); \
} while (0)

static spinlock_t node_lock;

struct tracepoints_table {
	const char *name;
	void *func;
	struct tracepoint *tp;
	bool init;
};

bool fuzzy_match(const char *pattern, const char *str) {
	if (pattern == NULL || str == NULL) {
		return false;
	}
	int pattern_len = strlen(pattern);
	int str_len = strlen(str);

	if (str_len < pattern_len) {
		return false;
	}

	return strncmp(str, pattern, pattern_len) == 0;
}

static noinline int tracing_mark_write(const char *buf)
{
	trace_printk(buf);
	return 0;
}

struct process_entry {
	const char *process_names;
	unsigned long repeat_count;
	unsigned long last_ino;
};

static struct process_entry process_table[MAX_PROCESSES_ENTRY];
static int current_index = 0;

struct process_entry *find_process_entry(const char *name) {
	int i;

	for (i = 0; i < current_index; i++) {
		if (process_table[i].process_names &&
			fuzzy_match(process_table[i].process_names, name)) {
				return &process_table[i];
		}
	}
	return NULL;
}

struct process_entry *add_process_entry(const char *name) {
	struct process_entry *entry;

	entry = find_process_entry(name);
	if (entry)
		return entry;

	if (current_index >= ARRAY_SIZE(process_table)) {
		printk(KERN_ERR "Process table is full, cannot add %s\n", name);
		return NULL;
	}

	entry = &process_table[current_index];
	entry->process_names =  kstrdup(name, GFP_KERNEL);
	entry->repeat_count = 1;
	entry->last_ino = 0;

	current_index++;
	return entry;
}

void del_process_entry(const char *name) {
	int i, j;

	for (i = 0; i < current_index; i++) {
		if (process_table[i].process_names &&
			strcmp(process_table[i].process_names, name) == 0) {
			kfree(process_table[i].process_names);
			for (j = i; j < current_index - 1; j++) {
				process_table[j] = process_table[j + 1];
			}

			current_index--;

			if (current_index < ARRAY_SIZE(process_table)) {
				process_table[current_index].process_names = NULL;
			}

			return;
		}
	}
}

static void init_record_process_names(void) {
	int i;
	struct process_entry *entry;

	const char *process_names[] = {
		"droid.ugc.aweme",
		".ugc.aweme.lite",
		"nmeng.pinduoduo",
		"com.tencent.mm",
		".smile.gifmaker",
		"kuaishou.nebula",
		"id.AlipayGphone",
		"m.taobao.taobao",
		"encent.mobileqq",
		"baidu.searchbox",
		"utonavi.minimap",
		"ngdong.app.mall",
		"com.xingin.xhs",
		"id.article.news",
		"utonavi.minimap",
		"ficationmanager",
		"ndroid.systemui",
		"om.oplus.camera",
		"loros.gallery3d",
		"oloros.launcher",
		"m.heytap.market",
		"ros.filemanager",
		"assistantscreen",
		"ndroid.settings",
		"ndroid.launcher",
		"[GT]ColdPool#",
		"aidu.input_oppo",
		"method.sogouoem",
	};

	for (i = 0; i < ARRAY_SIZE(process_names); i++) {
		entry = add_process_entry(process_names[i]);
		if (!entry)
			printk(KERN_ERR "Failed to add process: %s\n", process_names[i]);
	}
}


void clear_all_process_entries(void) {
	int i;

	for (i = 0; i < current_index; i++) {
		if (process_table[i].process_names) {
			kfree(process_table[i].process_names);
			process_table[i].process_names = NULL;
		}
	}

	current_index = 0;
}

static void put_dentry(void *data, async_cookie_t cookie)
{
	struct dentry *dentry = data;
	dput(dentry);
}

static void file_map_track_handler(void *ignore, struct folio *folio)
{
	struct address_space *mapping = folio->mapping;
	struct dentry *dentry;
	struct process_entry *entry;
	unsigned long flags;

	if (!tracing_is_on_dup()) {
		return;
	}

	spin_lock_irqsave(&node_lock, flags);
	entry = find_process_entry(current->comm);
	if (!entry) {
		spin_unlock_irqrestore(&node_lock, flags);
		return;
	}
	spin_unlock_irqrestore(&node_lock, flags);

	if(!mapping->host->i_ino)
		return;
	if (mapping->host->i_ino == entry->last_ino) {
		entry->repeat_count++;
		return;
	}
	entry->last_ino = mapping->host->i_ino;

	if (mapping->host->i_sb) {
		dentry = d_find_alias(mapping->host);
		if (!dentry)
			return;

		tracing_mark("%d|file_record %s order=%u repeat_count=%lu\n", current->tgid, dentry->d_name.name,
				folio_order(folio), entry->repeat_count);
		entry->repeat_count = 1;
		async_schedule(put_dentry, dentry);
	}
}

static struct tracepoints_table interests[] = {
	{
		.name = "mm_filemap_add_to_page_cache",
		.func = file_map_track_handler
	},
};

/*
 * Find the struct tracepoint* associated with a given tracepoint
 * name.
 */
static void lookup_tracepoints(struct tracepoint *tp, void *ignore)
{
	int i;

	FOR_EACH_INTEREST(i) {
		if (strcmp(interests[i].name, tp->name) == 0)
			interests[i].tp = tp;
	}
}

static void uninstall_tracepoints(void)
{
	int i;

	FOR_EACH_INTEREST(i) {
		if (interests[i].init) {
			tracepoint_probe_unregister(interests[i].tp,
						    interests[i].func,
						    NULL);
		}
	}
}

static bool install_tracepoints(void)
{
	int i;

	for_each_kernel_tracepoint(lookup_tracepoints, NULL);
	FOR_EACH_INTEREST(i) {
		if (interests[i].tp == NULL) {
			pr_err("%s : tracepoint %s not found\n",
				THIS_MODULE->name, interests[i].name);
			uninstall_tracepoints();
			return false;
		}

		tracepoint_probe_register(interests[i].tp,
					  interests[i].func,
					  NULL);
		interests[i].init = true;
	}

	return true;
}

static ssize_t file_record_proc_read(struct file *file, char __user *buf,
					size_t count, loff_t *ppos)
{
	char *buffer;
	int len = 0;
	int i;
	unsigned long flags;

	for (i = 0; i < current_index; i++) {
		if (process_table[i].process_names) {
			len += strlen(process_table[i].process_names) + 1;
		}
	}

	buffer = kmalloc(len + 1, GFP_KERNEL);
	if (!buffer) {
		return -ENOMEM;
	}

	buffer[0] = '\0';
	len = 0;

	spin_lock_irqsave(&node_lock, flags);

	for (i = 0; i < current_index; i++) {
		if (process_table[i].process_names) {
			len += sprintf(buffer + len, "%s\n", process_table[i].process_names);
		}
	}

	spin_unlock_irqrestore(&node_lock, flags);

	if (*ppos >= len) {
		kfree(buffer);
		return 0;
	}

	if (count > len - *ppos) {
		count = len - *ppos;
	}

	if (copy_to_user(buf, buffer + *ppos, count)) {
		kfree(buffer);
		return -EFAULT;
	}

	*ppos += count;
	kfree(buffer);
	return count;
}

static int handle_add_command(char *name, unsigned long flags)
{
	if (current_index >= MAX_PROCESSES_ENTRY) {
		return -ENOSPC;
	}

	struct process_entry *entry = add_process_entry(name);
	if (!entry) {
		printk(KERN_ERR "Failed to add process: %s\n", name);
		return -EFAULT;
	}
	return 0;
}

static int process_command(char *cmd, char *name, unsigned long flags)
{
	if (!cmd || !name) {
		return -EINVAL;
	}

	if (!strncmp(cmd, "-add", sizeof("-add"))) {
		return handle_add_command(name, flags);
	} else if (!strncmp(cmd, "-del", sizeof("-del"))) {
		del_process_entry(name);
	} else if (!strncmp(cmd, "-clear", sizeof("-clear"))) {
		clear_all_process_entries();
	} else {
		 return -EINVAL;
	}
	return 0;
}

static ssize_t file_record_proc_write(struct file *file, const char __user *buf,
                                      size_t count, loff_t *ppos)
{
	char *buffer, *orig;
	char *cmd;
	char *name;
	unsigned long flags;
	int ret = 0;

	if (count <= 0) {
		return -EINVAL;
	}

	buffer = kmalloc(count + 1, GFP_KERNEL);
	if (!buffer) {
		return -ENOMEM;
	}

	if (copy_from_user(buffer, buf, count)) {
		 kfree(buffer);
		return -EFAULT;
	}

	buffer[count] = '\0';
	orig = buffer;

	if (buffer[count-1] == '\n') {
		buffer[count-1] = '\0';
	}

	cmd = strsep(&buffer, " ");
	name = strsep(&buffer, " ");

	spin_lock_irqsave(&node_lock, flags);
	ret = process_command(cmd, name, flags);
	spin_unlock_irqrestore(&node_lock, flags);

	kfree(orig);
	return ret ?: count;
}

static void trace_symbol_init(void)
{
	int ret;
	struct kprobe tracing_is_on_kp = {
		.symbol_name = "tracing_is_on"
	};

	ret = register_kprobe(&tracing_is_on_kp);
	if (ret) {
		pr_err("get tracing_is_on_kp addr from kprobe failed! ret=%d\n", ret);
		return;
	}
	tracing_is_on_dup = (tracing_is_on_t)tracing_is_on_kp.addr;
	pr_info("suceesfully get tracing_is_on addr:0x%px\n", tracing_is_on_dup);
	unregister_kprobe(&tracing_is_on_kp);

	return;
}

static const struct proc_ops file_record_proc_fops = {
	.proc_write = file_record_proc_write,
	.proc_read = file_record_proc_read,
};

static void create_proc_node(void)
{
	struct proc_dir_entry *pentry;

	pentry = proc_create(PROC_NAME,
		(S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH), NULL,
			&file_record_proc_fops);

	if (!pentry)
		pr_err("%s: %s fail, name=%s", THIS_MODULE->name, __func__, PROC_NAME);
}

static int __init oplus_file_record_init(void)
{
	spin_lock_init(&node_lock);

	trace_symbol_init();

	init_record_process_names();

	if (install_tracepoints())
		create_proc_node();

	return 0;
}

static void __exit oplus_file_record_exit(void)
{
	unsigned long flags;

	spin_lock_irqsave(&node_lock, flags);

	clear_all_process_entries();

	spin_unlock_irqrestore(&node_lock, flags);
	remove_proc_entry(PROC_NAME, NULL);
	uninstall_tracepoints();
}

module_init(oplus_file_record_init);
module_exit(oplus_file_record_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("jiayingrui");
MODULE_DESCRIPTION("Used to record IO file paths to trace buf");
