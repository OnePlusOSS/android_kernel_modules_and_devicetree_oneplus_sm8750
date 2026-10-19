#include <linux/sort.h>
#include <linux/proc_fs.h>
#include <linux/sched/cputime.h>
#include <kernel/sched/sched.h>
#include <linux/threads.h>
#include "sa_common.h"
#include "sa_sysfs.h"
#include "sa_ddl.h"
#if IS_ENABLED(CONFIG_OPLUS_SCHED_GROUP_OPT)
#include "sa_group.h"
#endif

pid_t ddl_pid_rd = -1;

#define MSEC_TO_NSEC(val) (val * NSEC_PER_MSEC)
#define NSEC_TO_MSEC(val) (val / NSEC_PER_MSEC)

struct ddl_sinfo_data ddl_sdata[PID_MAX_DEFAULT];

static inline bool oplus_ddl_runnable(struct oplus_rq *orq)
{
	return orq->nr_ddl_preempted > 0;
}

u64 oplus_get_task_ddl(struct task_struct *task)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(task);

	if (IS_ERR_OR_NULL(ots))
		return 0;

	return READ_ONCE(ots->ddl);
}
EXPORT_SYMBOL_GPL(oplus_get_task_ddl);

static inline bool runnable_at_comp(struct rb_node *a_node, const struct rb_node *b_node)
{
	struct oplus_task_struct *ots_a = rb_entry(a_node, struct oplus_task_struct, ddl_node);
	struct oplus_task_struct *ots_b = rb_entry(b_node, struct oplus_task_struct, ddl_node);
	u64 ddl_a = ots_a->runnable_ts + MSEC_TO_NSEC(ots_a->ddl);
	u64 ddl_b = ots_b->runnable_ts + MSEC_TO_NSEC(ots_b->ddl);

	return (s64)(ddl_a - ddl_b) < 0;
}

void oplus_set_task_ddl(struct task_struct *task, u64 ddl)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(task);
	struct rq *rq;
	struct oplus_rq *orq;
	struct rq_flags flags;
	unsigned long irqflag;

	if (IS_ERR_OR_NULL(ots))
		return;

	if (ddl > MAX_DDL_LIMIT)
		return;

	if (!test_task_is_fair(task) || test_task_ux(task))
		return;

	rq = task_rq_lock(task, &flags);
	orq = get_oplus_rq(rq);

	spin_lock_irqsave(&orq->ddl_lock, irqflag);
	smp_mb__after_spinlock();
	WRITE_ONCE(ots->ddl, ddl);

	if (!ddl) {
		if (!oplus_rbnode_empty(&ots->ddl_node)) {
			if (test_and_clear_bit(OTS_STATE_DDL_ACTIVE_PREEMPTED, &ots->state))
				orq->nr_ddl_preempted--;
			rb_erase_cached(&ots->ddl_node, &orq->ddl_root);
			RB_CLEAR_NODE(&ots->ddl_node);
			put_task_struct(task);
		}
	} else if (task->se.on_rq && !task_on_cpu(rq, task)) {
		ots->runnable_ts = ots->enqueue_time;
		if (oplus_rbnode_empty(&ots->ddl_node)) {
			clear_bit(OTS_STATE_DDL_ACTIVE, &ots->state);
			clear_bit(OTS_STATE_DDL_ACTIVE_PREEMPTED, &ots->state);
			WRITE_ONCE(ots->ddl_active_ts, 0);
			get_task_struct(task);
			rb_add_cached(&ots->ddl_node, &orq->ddl_root, runnable_at_comp);
		} else {
			rb_erase_cached(&ots->ddl_node, &orq->ddl_root);
			rb_add_cached(&ots->ddl_node, &orq->ddl_root, runnable_at_comp);
		}
	}
	spin_unlock_irqrestore(&orq->ddl_lock, irqflag);
	task_rq_unlock(rq, task, &flags);
}
EXPORT_SYMBOL_GPL(oplus_set_task_ddl);

bool oplus_ddl_within_limit(struct rq *rq, struct task_struct *task)
{
	u64 now = rq_clock(rq);
	struct oplus_task_struct *ots = get_oplus_task_struct(task);
	u64 ddl_rthres = 0;

	if (!oplus_get_task_ddl(task) || !ots->ddl_active_ts)
		return false;

	if (!task->se.on_rq)
		return false;

#if IS_ENABLED(CONFIG_OPLUS_SCHED_GROUP_OPT)
	ddl_rthres = get_sg_ddl_rthres(task->sched_task_group);
#endif
	return now - ots->ddl_active_ts < MSEC_TO_NSEC(ddl_rthres);
}

void oplus_ddl_check_preempt(struct rq *rq, struct task_struct *p,
		struct task_struct *curr, bool *preempt, bool *nopreempt)
{
	u64 ddl_p = oplus_get_task_ddl(p);
	u64 ddl_curr = oplus_get_task_ddl(curr);

	if (!ddl_p && !ddl_curr)
		return;

	if (ddl_curr) {
		if (oplus_ddl_within_limit(rq, curr)) {
			*nopreempt = true;
			return;
		}
	}

	if (ddl_p) {
		struct oplus_task_struct *ots_p = get_oplus_task_struct(p);
		u64 now = rq_clock(rq);
		if (IS_ERR_OR_NULL(ots_p))
			return;

		if (now - ots_p->runnable_ts >= MSEC_TO_NSEC(ddl_p))
			*preempt = true;
	}
}

void oplus_ddl_preempt_tint(struct rq *rq, struct task_struct *prev)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(prev);
	u64 now = rq_clock(rq);

	if(!prev->se.on_rq || IS_ERR_OR_NULL(ots))
		return;

	if (ots->ddl_active_ts) {
		if (!oplus_ddl_within_limit(rq, prev)) {
			if (test_bit(OTS_STATE_DDL_ACTIVE_PREEMPTED, &ots->state))
				clear_bit(OTS_STATE_DDL_ACTIVE_PREEMPTED, &ots->state);
			return;
		}

		set_bit(OTS_STATE_DDL_ACTIVE_PREEMPTED, &ots->state);
		WRITE_ONCE(ots->ddl_active_ts, now - ots->ddl_active_ts);
	}
}

void oplus_task_ddl_tint(struct rq *rq, struct task_struct *next)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(next);
	u64 now = rq_clock(rq);

	if (!oplus_get_task_ddl(next))
		return;

	if (!test_bit(OTS_STATE_DDL_ACTIVE, &ots->state)
			&& !test_bit(OTS_STATE_DDL_ACTIVE_PREEMPTED, &ots->state))
		return;

	/*
	 * Clear active preempt bit when the task scheduled in, and ensure the
	 * preempted active ddl task running time within limit after several
	 * preemptions.
	 */
	if (unlikely(test_and_clear_bit(OTS_STATE_DDL_ACTIVE_PREEMPTED, &ots->state))) {
		WRITE_ONCE(ots->ddl_active_ts, now - ots->ddl_active_ts);
		return;
	}

	WRITE_ONCE(ots->ddl_active_ts, now);
}

void oplus_dequeue_ddl_node(struct rq *rq, struct task_struct *p)
{
	struct oplus_rq *orq = get_oplus_rq(rq);
	struct oplus_task_struct *ots = get_oplus_task_struct(p);
	unsigned long irqflag;

	if (!p || IS_ERR_OR_NULL(ots))
		return;

	spin_lock_irqsave(&orq->ddl_lock, irqflag);
	if (!oplus_rbnode_empty(&ots->ddl_node)) {
		/*
		 * Don't reset ddl status in dequeue path, set_next_entity
		 * also do the dequeue work, thus, we will use the active
		 * bit and active_ts in the following preemption check.
		 * reset ddl status in enqueue path because we only care about
		 * the runnable status.
		 */
		rb_erase_cached(&ots->ddl_node, &orq->ddl_root);
		RB_CLEAR_NODE(&ots->ddl_node);
		if (test_bit(OTS_STATE_DDL_ACTIVE_PREEMPTED, &ots->state))
			orq->nr_ddl_preempted--;
		put_task_struct(p);
	}
	spin_unlock_irqrestore(&orq->ddl_lock, irqflag);
}

void oplus_enqueue_ddl_node(struct rq *rq, struct task_struct *p)
{
	struct oplus_rq *orq = get_oplus_rq(rq);
	struct oplus_task_struct *ots = get_oplus_task_struct(p);
	u64 now = rq_clock(rq);
	unsigned long irqflag;

	if (!p || !oplus_get_task_ddl(p) || IS_ERR_OR_NULL(ots))
		return;

	/*
	 * Reset ddl status in enqueue path, but plz keep in mind that
	 * don't reset ddl_active_ts in preeempted path. ddl_active_ts
	 * is used to keep ddl running time in preempted path and grows
	 * automatically.
	 */
	clear_bit(OTS_STATE_DDL_ACTIVE, &ots->state);
	if (!p->se.on_rq) {
		if (!task_on_rq_migrating(p)) {
			ots->runnable_ts = now;
			clear_bit(OTS_STATE_DDL_ACTIVE_PREEMPTED, &ots->state);
		}
		WRITE_ONCE(ots->ddl_active_ts, 0);
	} else {
		struct cfs_rq *cfs_rq = task_cfs_rq(p);
		if (cfs_rq && cfs_rq->curr == &p->se) {
			ots->runnable_ts = now;
			if (ots->ddl_active_ts && !oplus_ddl_within_limit(rq, p))
				return;
			oplus_ddl_preempt_tint(rq, p);
		}
	}

	spin_lock_irqsave(&orq->ddl_lock, irqflag);
	smp_mb__after_spinlock();
	if (!oplus_rbnode_empty(&ots->ddl_node)) {
		WARN_ON_ONCE(1);
		spin_unlock_irqrestore(&orq->ddl_lock, irqflag);
		return;
	}

	get_task_struct(p);
	rb_add_cached(&ots->ddl_node, &orq->ddl_root, runnable_at_comp);
	if (test_bit(OTS_STATE_DDL_ACTIVE_PREEMPTED, &ots->state))
		orq->nr_ddl_preempted++;
	spin_unlock_irqrestore(&orq->ddl_lock, irqflag);
}

#define __node_2_ots(node) \
	rb_entry((node), struct oplus_task_struct, ddl_node)

static struct oplus_task_struct *pick_ddl_task(struct rq *rq, bool *active_preempted)
{
	struct oplus_rq *orq = get_oplus_rq(rq);
	struct rb_node *left = NULL, *active_left = NULL, *node = NULL;
	struct oplus_task_struct *ots = NULL;
	unsigned long irqflag;
	bool ddl_runnable;

	if (!orq)
		return NULL;

	spin_lock_irqsave(&orq->ddl_lock, irqflag);
	ddl_runnable = oplus_ddl_runnable(orq);
	left = rb_first_cached(&orq->ddl_root);
	if (!left)
		goto out_unlock;

	for (node = left; node; node = rb_next(node)) {
		ots = __node_2_ots(node);

		if (IS_ERR_OR_NULL(ots)) {
			WARN_ON_ONCE(1);
			goto out_leftmost;
		}

		if (!active_left)
			active_left = node;

		if (ddl_runnable) {
			if (test_bit(OTS_STATE_DDL_ACTIVE_PREEMPTED, &ots->state)) {
				*active_preempted = true;
				goto out_unlock;
			}
		} else
			break;
	}

out_leftmost:
	ots = active_left ? __node_2_ots(active_left) : NULL;

out_unlock:
	spin_unlock_irqrestore(&orq->ddl_lock, irqflag);
	return ots;
}

void update_ddl_hit_history(struct task_struct *p)
{
	if(p) {
		if (!strlen(ddl_sdata[p->pid].comm)
			|| strncmp(p->comm, ddl_sdata[p->pid].comm, strlen(p->comm))) {
			memset(&ddl_sdata[p->pid], 0, sizeof(struct ddl_sinfo_data));
			strscpy_pad(ddl_sdata[p->pid].comm, p->comm, TASK_COMM_LEN);
		}
		ddl_sdata[p->pid].hit++;
	}
}

void oplus_replace_next_task_ddl(struct rq *rq, struct task_struct **p,
	struct sched_entity **se, bool *repick, bool simple)
{
	bool active_preempted = false;
	u64 now = rq_clock(rq), runnable_time;
	struct oplus_task_struct *ots;

	ots = pick_ddl_task(rq, &active_preempted);
	if (IS_ERR_OR_NULL(ots))
		return;

	runnable_time = now - ots->runnable_ts;
	if (unlikely((s64)runnable_time <= 0) && !active_preempted)
		return;

	if (NSEC_TO_MSEC(runnable_time) >= oplus_get_task_ddl(ots->task) ||
			active_preempted) {
		*p = ots->task;
		*se = &ots->task->se;
		*repick = true;
		set_bit(OTS_STATE_DDL_ACTIVE, &ots->state);
		update_ddl_hit_history(*p);
	}
}

static ssize_t proc_ddl_task_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[BUFFER_SIZE_DDL];
	size_t len = 0;
	struct task_struct *task;

	rcu_read_lock();
	task = find_task_by_vpid(ddl_pid_rd);
	if (task)
		get_task_struct(task);
	rcu_read_unlock();

	if (task) {
		len = snprintf(buffer, sizeof(buffer), "%d ddl = %llu\n",
			ddl_pid_rd, oplus_get_task_ddl(task));
		put_task_struct(task);
	}

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
};

/*
 * Example:
 * adb shell "echo "p 1611 8" > proc/oplus_scheduler/sched_assist/ux_task"
 * 'p' means pid, '1611' is thread pid, '8' means set ux state as '2'
 *
 * adb shell "echo "r 1611" > proc/oplus_scheduler/sched_assist/ux_task"
 * "r" means we want to read thread "1611"'s ddl info
 */
static ssize_t proc_ddl_task_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[BUFFER_SIZE_DDL];
	char *str, *token;
	char opt_str[OPT_STR_MAX][13] = {"0", "0", "0"};
	int cnt = 0, pid, ddl, err;

	int uid = task_uid(current).val;
	/* only accept ux from system server or performance binder */
	if (SYSTEM_UID != uid && ROOT_UID != uid) {
		return -EFAULT;
	}

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';
	str = strstrip(buffer);
	while ((token = strsep(&str, " ")) && *token && (cnt < OPT_STR_MAX)) {
		strlcpy(opt_str[cnt], token, sizeof(opt_str[cnt]));
		cnt += 1;
	}

	if (cnt != OPT_STR_MAX) {
		if (cnt == (OPT_STR_MAX - 1) && !strncmp(opt_str[OPT_STR_TYPE], "r", 1)) {
			err = kstrtoint(strstrip(opt_str[OPT_STR_PID]), 10, &pid);
			if (err)
				return err;

			if (pid > 0 && pid <= PID_MAX_DEFAULT)
				ddl_pid_rd = pid;
		}

		return -EFAULT;
	}

	err = kstrtoint(strstrip(opt_str[OPT_STR_PID]), 10, &pid);
	if (err)
		return err;

	err = kstrtoint(strstrip(opt_str[OPT_STR_VAL]), 10, &ddl);
	if (err)
		return err;

	if (!strncmp(opt_str[OPT_STR_TYPE], "p", 1) && (ddl >= 0)) {
		struct task_struct *ddl_task = NULL;

		if (pid > 0 && pid <= PID_MAX_DEFAULT) {
			rcu_read_lock();
			ddl_task = find_task_by_vpid(pid);
			if (ddl_task)
				get_task_struct(ddl_task);
			rcu_read_unlock();

			if (ddl_task) {
				oplus_set_task_ddl(ddl_task, ddl);
				put_task_struct(ddl_task);
			}
		}
	}

	return count;
};

static long proc_ddl_task_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *__arg = (void __user *) arg;
	struct task_struct *ddl_task = NULL, *t = NULL;
	int ret = 0;
	struct ddl_ioctl_data data;
	pid_t pid;
	u64 ddl;

	if (_IOC_TYPE(cmd) != DDL_MAGIC ||
		_IOC_NR(cmd) >= DDL_CMD_MAX) {
		return -EINVAL;
	}

	if (copy_from_user(&data, (void __user *)__arg, sizeof(data))) {
		pr_err("Invalid address!!!");
		return -EFAULT;
	}

	pid = data.pid;
	ddl = data.ddl;

	rcu_read_lock();
	ddl_task = find_task_by_vpid(pid);
	if (!ddl_task || !pid_alive(ddl_task)) {
		rcu_read_unlock();
		return -ECHILD;
	}
	get_task_struct(ddl_task);
	rcu_read_unlock();

	switch (cmd) {
	case SET_THREAD_DDL:
		oplus_set_task_ddl(ddl_task, ddl);
		break;

	case SET_PROCESS_DDL:
		for_each_thread(ddl_task, t) {
			if (!t || !pid_alive(t))
				continue;
			get_task_struct(t);
			oplus_set_task_ddl(t, ddl);
			put_task_struct(t);
		}
		break;

	default:
		ret = -EINVAL;
		break;
	}

	put_task_struct(ddl_task);
	return ret;
}

#ifdef CONFIG_COMPAT
static long proc_ddl_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return proc_ddl_task_ioctl(file, cmd, (unsigned long)(compat_ptr(arg)));
}
#endif

static int ddl_sinfo_comp(const void *a, const void *b)
{
	const struct ddl_sinfo_data sa = *(const struct ddl_sinfo_data *)a;
	const struct ddl_sinfo_data sb = *(const struct ddl_sinfo_data *)b;
	return sb.hit - sa.hit;
}

static ssize_t proc_ddl_sinfo_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	size_t len = 0;
	unsigned int i = 0, num = 0;
	char buffer[BUFFER_SIZE_DDL];

	while (i < PID_MAX_DEFAULT && num <PID_MAX_DEFAULT) {
		while (num < PID_MAX_DEFAULT && ddl_sdata[num].hit)
			num++;
		while (i < PID_MAX_DEFAULT && !ddl_sdata[i].hit)
			i++;
		if (i < num) {
			i = num + 1;
			continue;
		}
		if (num < PID_MAX_DEFAULT && i < PID_MAX_DEFAULT) {
			ddl_sdata[num] = ddl_sdata[i];
			memset(&ddl_sdata[i], 0, sizeof(struct ddl_sinfo_data));
		}
		i++;
		num++;
	}

	num = min(num, (unsigned int)PID_MAX_DEFAULT - 1);
	sort(ddl_sdata, num, sizeof(struct ddl_sinfo_data), ddl_sinfo_comp, NULL);

	for (i = 0; i < NUM_DDL_HIT_ITEM; i++)
	{
		if (!ddl_sdata[i].hit)
			break;
		len += snprintf(buffer + len, sizeof(buffer) - len, "%s:%llu ",
			ddl_sdata[i].comm, ddl_sdata[i].hit);
		if (len >= MAX_GUARDS_SIZE) {
			len += snprintf(buffer + len, sizeof(buffer) - len, "...");
			break;
		}
	}
	buffer[len++] = '\n';
	memset(ddl_sdata, 0, sizeof(struct ddl_sinfo_data) * PID_MAX_DEFAULT);

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static const struct proc_ops ddl_task_fops = {
	.proc_read		= proc_ddl_task_read,
	.proc_write		= proc_ddl_task_write,
	.proc_ioctl		= proc_ddl_task_ioctl,
#ifdef CONFIG_COMPAT
	.proc_compat_ioctl = proc_ddl_compat_ioctl,
#endif
	.proc_lseek		= default_llseek,
};

static const struct proc_ops ddl_sinfo_fops = {
	.proc_read		= proc_ddl_sinfo_read,
	.proc_lseek		= default_llseek,
};

void oplus_sched_ddl_init(struct proc_dir_entry *pde)
{
	struct proc_dir_entry *proc_node;

	proc_node = proc_create("ddl_task", 0666, pde, &ddl_task_fops);
	if (!proc_node) {
		remove_proc_entry("ddl_task", pde);
		pr_err("failed to create proc node ddl_task\n");
	}

	proc_node = proc_create("ddl_sinfo", 0666, pde, &ddl_sinfo_fops);
	if (!proc_node) {
		remove_proc_entry("ddl_task", pde);
		pr_err("failed to create proc node ddl_task\n");
	}

}
