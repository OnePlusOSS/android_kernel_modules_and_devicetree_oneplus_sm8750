// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#include <linux/sched.h>
#include <linux/list.h>
#include <linux/jiffies.h>

#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include <linux/cpumask.h>
#include <linux/sched/topology.h>
#include <linux/sched/task.h>
#include <linux/string.h>
#include <linux/sched/cputime.h>
#include <kernel/sched/sched.h>
#include <fs/proc/internal.h>
#include <linux/signal.h>
#include <linux/cpufeature.h>
#include <linux/sched/clock.h>
#include <linux/thread_info.h>
#include <linux/threads.h>
#include <linux/profile.h>
#include <linux/kprobes.h>
#include <linux/cgroup.h>
#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
#include <linux/cpuhotplug.h>
#endif /* CONFIG_OPLUS_FEATURE_SCHED_SPREAD */

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CPU_JANKINFO)
#include "sa_jankinfo.h"
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOADBALANCE)
#include "sa_balance.h"
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
#include <../kernel/oplus_cpu/sched/frame_boost/frame_group.h>
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_DDL)
#include "sa_ddl.h"
#endif

#if IS_ENABLED(CONFIG_OPLUS_SCHED_TUNE)
#include <../kernel/oplus_cpu/sched/sched_tune/tune.h>
#endif

#if IS_ENABLED(CONFIG_OPLUS_SCHED_GROUP_OPT)
#include "sa_group.h"
#endif

#include "sched_assist.h"
#include "sa_common.h"
#include "sa_fair.h"
#include "sa_priority.h"

#ifdef CONFIG_OPLUS_CPU_AUDIO_PERF
#include "sa_audio.h"
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_PIPELINE)
#include "sa_pipeline.h"
#endif
#include "sa_exec.h"

#define CREATE_TRACE_POINTS
#include "cputime.h"
#undef CREATE_TRACE_POINTS

#define CREATE_TRACE_POINTS
#include "trace_sched_assist.h"

#define MS_TO_NS (1000000)
#define MAX_INHERIT_GRAN ((u64)(64 * MS_TO_NS))

#define INHERIT_UX_SEC_WIDTH				8
#define INHERIT_UX_MASK_BASE				0x00000000ff

#define inherit_ux_offset_of(type)			(type * INHERIT_UX_SEC_WIDTH)
#define inherit_ux_mask_of(type)			((u64)(INHERIT_UX_MASK_BASE) << (inherit_ux_offset_of(type)))

#define inherit_ux_get_bits(value, type)	((value & inherit_ux_mask_of(type)) >> inherit_ux_offset_of(type))
#define inherit_ux_value(type, value)		((u64)value << inherit_ux_offset_of(type))

#define SCHED_MAX_CPUSET 100ULL
#define SCHED_MAX_CPUCTL 100ULL
#define SCHED_MAX_CFS_R 100ULL
#define SCHED_MAX_RT_R 10ULL
#define SCHED_MAX_DDL_ACTIVE 10ULL
#define SCHED_MAX_DDL_TASK 10ULL
#define SCHED_MAX_AFFINITY_MASK 1000ULL
#define MAX_PID (32768)
#define CPUCTL_MULT_UNIT (SCHED_MAX_CPUSET)
#define CFS_R_MULT_UNIT (CPUCTL_MULT_UNIT * SCHED_MAX_CPUCTL)
#define RT_R_MULT_UNIT (CFS_R_MULT_UNIT * SCHED_MAX_CFS_R)
#define DDL_ACTIVE_MULT_UNIT (RT_R_MULT_UNIT * SCHED_MAX_RT_R)
#define DDL_TASK_MULT_UNIT (DDL_ACTIVE_MULT_UNIT * SCHED_MAX_DDL_ACTIVE)
#define AFFINITY_MASK_MULT_UNIT (DDL_TASK_MULT_UNIT * SCHED_MAX_DDL_TASK)
#define AFFINITY_SET_MULT_UNIT (AFFINITY_MASK_MULT_UNIT * SCHED_MAX_AFFINITY_MASK)

#define SCHED_MAX_UCLAMP_MIN 1000ULL
#define SCHED_MAX_UCLAMP_MAX 1000ULL
#define SCHED_MAX_STUNE 1000ULL
#define UCLAMP_MAX_MULT_UNIT (SCHED_MAX_UCLAMP_MIN)
#define STUNE_MULT_UNIT (UCLAMP_MAX_MULT_UNIT * SCHED_MAX_UCLAMP_MAX)

#ifdef CONFIG_OPLUS_SCHED_HALT_MASK_PRT
#define SCHED_PARTIAL_HALT_OFFSET 10000LL

cpumask_t cur_cpus_halt_mask = { CPU_BITS_NONE };
EXPORT_SYMBOL(cur_cpus_halt_mask);
cpumask_t cur_cpus_phalt_mask = { CPU_BITS_NONE };
EXPORT_SYMBOL(cur_cpus_phalt_mask);
DEFINE_PER_CPU(int[OPLUS_MAX_PAUSE_TYPE], oplus_cur_pause_client);
EXPORT_SYMBOL(oplus_cur_pause_client);
#endif /* CONFIG_OPLUS_SCHED_HALT_MASK_PRT */

#ifdef CONFIG_OPLUS_FEATURE_TICK_GRAN
DEFINE_PER_CPU(u64, retired_instrs);
DEFINE_PER_CPU(u64, nvcsw);
DEFINE_PER_CPU(u64, nivcsw);
#endif

/* debug print frequency limit */
static DEFINE_PER_CPU(int, prev_ux_state);
static DEFINE_PER_CPU(int, prev_sub_ux_state);
static DEFINE_PER_CPU(int, prev_ux_priority);
static DEFINE_PER_CPU(u64, prev_vruntime);
static DEFINE_PER_CPU(u64, prev_min_vruntime);
static DEFINE_PER_CPU(u64, prev_preset_vruntime);
static DEFINE_PER_CPU(int, prev_hwbinder_flag);
static DEFINE_PER_CPU(u64, prev_sched_info);
static DEFINE_PER_CPU(u64, prev_qs_info);

#if IS_ENABLED(CONFIG_SCHED_WALT)
#define WINDOW_SIZE (16000000)
#define scale_demand(d) ((d)/(WINDOW_SIZE >> SCHED_CAPACITY_SHIFT))
#endif

/* to avoid there is no schedtune_task_boost symbol , we need define a weak symbol here*/
__attribute__((weak)) int schedtune_task_boost(__attribute__((unused)) struct task_struct *p)
{
	return 0;
}

#ifdef CONFIG_OPLUS_FEATURE_TICK_GRAN
#define TICK_GRAN_NUM 3
#endif

#ifdef CONFIG_LOCKING_PROTECT
struct sched_assist_locking_ops *locking_ops __read_mostly;

void register_sched_assist_locking_ops(struct sched_assist_locking_ops *ops)
{
	if (!ops)
		return;

	if (cmpxchg(&locking_ops, NULL, ops))
		pr_warn("sched_assist_locking_ops has already been registered!\n");
}
EXPORT_SYMBOL_GPL(register_sched_assist_locking_ops);
#endif

#define TOPAPP 4
#define BGAPP  3

bool is_top(struct task_struct *p)
{
	struct cgroup_subsys_state *css;

	if (p == NULL)
		return false;

	rcu_read_lock();
	css = task_css(p, cpu_cgrp_id);
	if (!css) {
		rcu_read_unlock();
		return false;
	}
	rcu_read_unlock();

	return css->id == TOPAPP;
}

#ifdef CONFIG_OPLUS_FEATURE_INPUT_BOOST
bool is_webview(struct task_struct *p)
{
	unsigned long im_flag = IM_FLAG_NONE;

	if (!ta_task(p))
		return false;

	im_flag = oplus_get_im_flag(p);
	if (test_bit(IM_FLAG_WEBVIEW, &im_flag))
		return true;

	return false;
}
#endif

struct oplus_rq *get_oplus_rq(struct rq *rq)
{
	struct oplus_rq *orq = NULL;

	if (!rq)
		return NULL;

	orq = (struct oplus_rq *) READ_ONCE(rq->android_oem_data1[ORQ_IDX]);
	if (IS_ERR_OR_NULL(orq))
		return NULL;

	return orq;
}
EXPORT_SYMBOL_GPL(get_oplus_rq);

bool is_heavy_load_top_task(struct task_struct *p)
{
	if (!ta_task(p))
		return false;

	/* is UI main thread or RenderThread of TOP APP */
	if ((p->pid == p->tgid) || (!strncmp(p->comm, "RenderThread", 12)))
		return true;

	return false;
}

struct ux_sched_cputopo ux_sched_cputopo;
bool global_less_prime_cpu_arch;

static inline void sched_init_ux_cputopo(void)
{
	int i = 0;

	ux_sched_cputopo.cls_nr = 0;
	for (; i < OPLUS_NR_CPUS; ++i) {
		cpumask_clear(&ux_sched_cputopo.sched_cls[i].cpus);
		ux_sched_cputopo.sched_cls[i].capacity = ULONG_MAX;
	}
}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOADBALANCE)
void dump_oplus_cpu_array(void)
{
	struct ux_sched_cputopo *ux_cputopo = &ux_sched_cputopo;
	int cls_nr = ux_cputopo->cls_nr;
	int i, j;
	char buf[256];
	int count = 0;

	for (i = 0; i < 2*cls_nr; i++) {
		count += snprintf(buf + count, PAGE_SIZE - count, "OPLUS_CPU_ARRAY: order_idx=%d, [", i);
		for (j = 0; j < cls_nr; j++) {
			count += snprintf(buf + count, PAGE_SIZE - count, "%d%s",
				topology_cluster_id(cpumask_first(&ux_cputopo->oplus_cpu_array[i][j])),
				j == cls_nr-1 ? "":" -> ");
		}
		count += snprintf(buf + count, PAGE_SIZE - count, "]\n");
		pr_info("%s", buf);
		memset(buf, 0, sizeof(buf));
		count = 0;
	}
}
EXPORT_SYMBOL_GPL(dump_oplus_cpu_array);

static void build_oplus_cpu_array(void)
{
	struct ux_sched_cputopo *ux_cputopo = &ux_sched_cputopo;
	int cls_nr = ux_cputopo->cls_nr;
	int i;

	/* Construct cpu_array row by row */
	for (i = 0; i < cls_nr; i++) {
		int j, k = 1;

		/* Fill out first column with appropriate cpu arrays */
		cpumask_copy(&ux_cputopo->oplus_cpu_array[i][0],
				&ux_cputopo->sched_cls[i].cpus);

		/*
		 * k starts from column 1 because 0 is filled
		 * Fill clusters for the rest of the row,
		 * above i in ascending order
		 */
		for (j = i + 1; j < cls_nr; j++) {
			cpumask_copy(&ux_cputopo->oplus_cpu_array[i][k],
					&ux_cputopo->sched_cls[j].cpus);
			k++;
		}

		/*
		 * k starts from where we left off above.
		 * Fill clusters below i in descending order.
		 */
		for (j = i - 1; j >= 0; j--) {
			cpumask_copy(&ux_cputopo->oplus_cpu_array[i][k],
					&ux_cputopo->sched_cls[j].cpus);
			k++;
		}
	}

	for (i = cls_nr; i < 2*cls_nr; i++) {
		int j, k = 1;

		/* Fill out first column with appropriate cpu arrays */
		cpumask_copy(&ux_cputopo->oplus_cpu_array[i][0],
				&ux_cputopo->sched_cls[i-cls_nr].cpus);

		/*
		 * k starts from column 1 because 0 is filled
		 * Fill clusters for the rest of the row,
		 * above i in ascending order
		 */
		for (j = i - cls_nr - 1; j >= 0; j--) {
			cpumask_copy(&ux_cputopo->oplus_cpu_array[i][k],
					&ux_cputopo->sched_cls[j].cpus);
			k++;
		}

		/*
		 * k starts from where we left off above.
		 * Fill clusters below i in descending order.
		 */
		for (j = i + 1; j < 2*cls_nr; j++) {
			cpumask_copy(&ux_cputopo->oplus_cpu_array[i][k],
					&ux_cputopo->sched_cls[j-cls_nr].cpus);
			k++;
		}
	}
}
#endif

inline bool is_less_prime_cpu_arch(void)
{
	unsigned int sliver_cpus = 0;
	unsigned int total_cpus = 0;
	int i;
	bool ret = false;

	for (i = 0; i < ux_sched_cputopo.cls_nr; i++) {
		if (i == 0) {
			sliver_cpus = cpumask_weight(&ux_sched_cputopo.sched_cls[i].cpus);
		}
		total_cpus += cpumask_weight(&ux_sched_cputopo.sched_cls[i].cpus);
	}
	/* The number of small cpus at least two more than that of prime cpus */
	ret = sliver_cpus >= (total_cpus - sliver_cpus + 4);

	return ret;
}

void update_ux_sched_cputopo(void)
{
	unsigned long cpu_cap = 0;
	unsigned int cpu = 0;
	int i = 0, insert_idx = 0, cls_nr = 0;
	struct ux_sched_cluster sched_cls;
	int cls_id = -1, prev_cls_id = -1;

	/* reset prev cpu topo info */
	sched_init_ux_cputopo();

	/* update new cpu topo info */
	for_each_possible_cpu(cpu) {
		cpu_cap = arch_scale_cpu_capacity(cpu);
		cls_id = topology_cluster_id(cpu);

		/* add cpu with same cls_id into target sched_cls, mtk can update capacity in mtk_update_cpu_capacity */
		if (cls_id == prev_cls_id) {
			for (i = 0; i < ux_sched_cputopo.cls_nr; ++i) {
				if (cls_id == i) {
					cpumask_set_cpu(cpu, &ux_sched_cputopo.sched_cls[i].cpus);
					break;
				}
			}

			continue;
		}

		cpumask_clear(&sched_cls.cpus);
		cpumask_set_cpu(cpu, &sched_cls.cpus);
		sched_cls.capacity = cpu_cap;
		cls_nr = ux_sched_cputopo.cls_nr;

		if (!cls_nr) {
			ux_sched_cputopo.sched_cls[cls_nr] = sched_cls;
		} else {
			for (i = 0; i <= ux_sched_cputopo.cls_nr; ++i) {
				if (cls_id < i || ux_sched_cputopo.sched_cls[i].capacity == ULONG_MAX) {
					insert_idx = i;
					break;
				}
			}
			if (insert_idx == ux_sched_cputopo.cls_nr) {
				ux_sched_cputopo.sched_cls[insert_idx] = sched_cls;
			} else {
				for (; cls_nr > insert_idx; cls_nr--)
					ux_sched_cputopo.sched_cls[cls_nr] = ux_sched_cputopo.sched_cls[cls_nr-1];

				ux_sched_cputopo.sched_cls[insert_idx] = sched_cls;
			}
		}
		ux_sched_cputopo.cls_nr++;

		prev_cls_id = cls_id;
	}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOADBALANCE)
	build_oplus_cpu_array();
#endif

	global_less_prime_cpu_arch = is_less_prime_cpu_arch();
}
EXPORT_SYMBOL(update_ux_sched_cputopo);

bool task_is_runnable(struct task_struct *task)
{
	if (!task)
		return false;

	if (READ_ONCE((task)->__state) != TASK_RUNNING)
		return false;

	return (task->on_rq && !task->on_cpu);
}


bool is_min_cluster(int cpu)
{
	/*
	 * The package id value of the cpu in min_cluster is 0.
	 */
	return !topology_cluster_id(cpu);
}

bool is_max_cluster(int cpu)
{
	struct ux_sched_cputopo ux_cputopo = ux_sched_cputopo;
	int cls_nr = ux_cputopo.cls_nr;

	/*
	 * The package_id value of the cpu in mxn_cluster is the largest.
	 */
	return topology_cluster_id(cpu) == cls_nr-1;
}

bool is_mid_cluster(int cpu)
{
	return !(is_min_cluster(cpu) || is_max_cluster(cpu));
}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_PIPELINE)
extern bool oplus_pipeline_task_skip_ux_change(struct oplus_task_struct *ots, int *ux_state);
#endif

/* UX synchronization rules
 * 1. when task set ux first time, or alter ux state,
 *    ACQUIRE (rq->lock)         prevent task migrate between rq
 *    ACQUIRE (ux_list->lock)
 *    add task to list or change position in list
 *    RELEASE (ux_list->lock)
 *    RELEASE (rq->lock)

 * 2. when task ux -> 0, or dequeue from list
 *    ACQUIRE (rq->lock)         prevent task migrate between rq
 *    ACQUIRE (ux_list->lock)
 *    list_del_init(ux_entry)
 *    RELEASE (ux_list->lock)
 *    RELEASE (rq->lock)

 * 3. oplus_rbtree_empty(ux_list) is atomic, unnecessary to lock
 *    ux_list_first_entry(ux_list) isn't atomic, but is safe to get without lock

 * 4. oplus_rbtree_empty(ux_list) and then list_first_entry(ux_list)
 *    ACQUIRE (ux_list->lock)
 *    oplus_rbtree_empty(ux_list)
 *    list_first_entry(ux_list)
 *    RELEASE(ux_list->lock)

*/
void oplus_set_ux_state_lock(struct task_struct *t, int ux_state, int inherit_type, bool need_lock_rq)
{
	struct rq *rq;
	struct rq_flags flags;
	struct oplus_rq *orq;
	struct oplus_task_struct *ots;
	unsigned long irqflag;
	bool need_lock_pi = (inherit_type == INHERIT_UX_PIFUTEX) ? false : true;

	if (need_lock_rq && need_lock_pi) {
		rq = task_rq_lock(t, &flags);
	} else if (need_lock_rq && !need_lock_pi) {
		lockdep_assert_held(&t->pi_lock);
		rq = __task_rq_lock(t, &flags);
	} else {
		rq = task_rq(t);
	}

	if (!raw_spin_is_locked(&t->pi_lock)) {
		DEBUG_BUG_ON(1);
	}
	if (!raw_spin_is_locked(__rq_lockp(rq))) {
		DEBUG_BUG_ON(2);
	}

	ots = get_oplus_task_struct(t);

	if (IS_ERR_OR_NULL(ots))
		goto out;

	if (!(ots->ux_state & SCHED_ASSIST_UX_MASK) && (ots->sub_ux_state & SCHED_ASSIST_UX_MASK)) {
		/* ux_state is empty and sub_ux_state is not */
		DEBUG_BUG_ON(3);
	}

	if ((ots->ux_state & SCHED_ASSIST_UX_MASK) && (ots->sub_ux_state & SCHED_ASSIST_UX_MASK)) {
		/* ux_state is inherit or sub_ux_state is not*/
		DEBUG_BUG_ON(ots->ux_state & SA_TYPE_INHERIT);
		DEBUG_BUG_ON(!(ots->sub_ux_state & SA_TYPE_INHERIT));
	}

	if (inherit_type == -1) {
		DEBUG_BUG_ON(ux_state & SA_TYPE_INHERIT);
		if (ux_state & SCHED_ASSIST_UX_MASK) {
			if (ots->ux_state & SA_TYPE_INHERIT) {
				ots->sub_ux_state = ots->ux_state;
			}
		} else {
			if (ots->ux_state & SA_TYPE_INHERIT) {
				goto out;
			} else if (ots->sub_ux_state & SCHED_ASSIST_UX_MASK) {
				ux_state = ots->sub_ux_state;
				ots->sub_ux_state = 0;
			}
		}
	} else {
		if (ux_state & SCHED_ASSIST_UX_MASK) {
			DEBUG_BUG_ON(!(ux_state & SA_TYPE_INHERIT));
		}
		if (ux_state & SCHED_ASSIST_UX_MASK) {
			if ((ots->ux_state & SCHED_ASSIST_UX_MASK) && !(ots->ux_state & SA_TYPE_INHERIT)) {
				ots->sub_ux_state = ux_state;
				goto out;
			}
		} else {
			if (!(ots->ux_state & SA_TYPE_INHERIT)) {
				if (ots->sub_ux_state & SCHED_ASSIST_UX_MASK) {
					ots->sub_ux_state = ux_state;
				}
				goto out;
			}
		}
	}

	if (ux_state == ots->ux_state)
		goto out;
	/* just save ux state, if it is rt */
	if (unlikely(!test_task_is_fair(t))) {
		/* rt task set ux_state as well */
		ots->ux_state = ux_state;
		ots->ux_priority = ux_state_to_priority(ux_state);
		ots->ux_nice = ux_type_to_nice(ux_state);
		goto out;
	}

	if (inherit_type == INHERIT_UX_PIFUTEX)
		goto set;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_PIPELINE)
	if (oplus_pipeline_task_skip_ux_change(ots, &ux_state))
		goto out;
#endif

set:
	orq = get_oplus_rq(rq);
	/* BUG 6523080
	* 1. task T is migrating from rq1 -> rq2
	* 2. set task T ux state to 0 without locking rq1
		spin_lock(rq1 -> ux_list_lock)
		remove it from rq1
		ots->ux_priority = -1
		spin_unlock(rq1 -> ux_list_lock)
	* 3. task T mirgated to rq2 with ots->ux_priority = -1
		spin_lock(rq2 -> ux_list_lock)
		enqueue task T
		spin_unlock(rq2 -> ux_list_lock)
	* 4. brk access -> ux_prio_to_weight[ots->ux_priority]
	*/

	/* BUG 6523080
	* 0. task T run on rq1
	* 1. task T put into sleep with ux state = 2 (not in any ux list)
	* 2. task T awake, and putting into rq2
	* 3. task T set ux state to 0 (it will lock the rq1)
	* 4. now task T in rq2 with ux_state = 0, and ux_priority = -1
	*/

	/* BUG 6523080
	* 0. task T run on rq1
	* 1. set task T ux state to 0 without locking rq1
	* 2. task migrated to rq2
	* 2. task T tick on rq2 (may reorder the ux tree)
	* 3. now task T in rq2 with ux_state = 0, and ux_priority = -1
	*/
	spin_lock_irqsave(orq->ux_list_lock, irqflag);
	smp_mb__after_spinlock();
	ots->ux_state = ux_state;

	/* if ux is disabled, task only removed from list, not insert to list. */
	if (unlikely(!global_sched_assist_enabled)) {
		if (!oplus_rbnode_empty(&ots->ux_entry)) {
			update_ux_timeline_task_removal(orq, ots, &t->se, task_current(rq, t));
			put_task_struct(t);
		}
		ots->ux_priority = ux_state_to_priority(ux_state);
		ots->ux_nice = ux_type_to_nice(ux_state);
		spin_unlock_irqrestore(orq->ux_list_lock, irqflag);
		goto out;
	}

	if (!(ux_state & SCHED_ASSIST_UX_MASK)) {
		if (!oplus_rbnode_empty(&ots->ux_entry)) {
			bool is_curr;
			lockdep_assert_rq_held(rq);
			is_curr = task_current(rq, t);
			update_ux_timeline_task_removal(orq, ots, &t->se, is_curr);
			if (is_curr) {
				/* if current's ux turn to off, it is likely to resched for this rq */
				resched_curr(rq);
			}
			put_task_struct(t);
			/* make sure task is removed from the list before ux_priority set to invalid */
			/* smp_wmb(); */
		}
		ots->ux_priority = -1;
		ots->ux_nice = -1;
	} else if (task_on_rq_queued(t)) {
		int prio, nice;
		bool unlinked, is_fair;
		struct task_struct *curr;
		lockdep_assert_rq_held(rq);
		unlinked = oplus_rbnode_empty(&ots->ux_entry);
		if (unlinked) {
			get_task_struct(t);
			/* when obtain ux state first time after enqueued,
			  sum_exec_baseline reset to task's curr exec runtime
			  make sure this task gain ux bonus exec time. */
			/*if (!ots->total_exec) {
				ots->sum_exec_baseline = t->se.sum_exec_runtime;
			}*/
			prio = ux_state_to_priority(ux_state);
			nice = ux_type_to_nice(ux_state);
			initial_prio_nice_and_vruntime(orq, ots, prio, nice);
			insert_task_to_ux_timeline(ots, orq);
		} else {
			int next_type;
			pick_next_ux_exec(ots, 0, &next_type);
			if (next_type) {
				prio = ux_type_to_priority(ots, next_type);
				nice = ux_type_to_nice(next_type);
				update_ux_timeline_task_change(orq, ots, prio, nice);
			} else {
				/* task is already timeout for current ux_state */
				update_ux_timeline_task_removal(orq, ots, &t->se, task_current(rq, t));
				put_task_struct(t);
			}
		}
		rcu_read_lock();
		curr = rcu_dereference(rq->curr);
		is_fair = (curr == NULL) || test_task_is_fair(curr);
		rcu_read_unlock();
		if (is_fair && !task_current(rq, t) && (ots == ux_list_first_entry(&orq->ux_list))) {
			resched_curr(rq);
		}
		if (unlikely(global_debug_enabled & DEBUG_SYSTRACE)) {
			if (task_current(rq, t)) {
				ux_state_systrace_c(cpu_of(rq), t);
			}
		}
	} else {
		ots->ux_priority = ux_state_to_priority(ux_state);
		ots->ux_nice = ux_type_to_nice(ux_state);
	}
	spin_unlock_irqrestore(orq->ux_list_lock, irqflag);

out:
	/* ux_state is empty and sub_ux_state is not */
	DEBUG_BUG_ON((ots != NULL) && !(ots->ux_state & SCHED_ASSIST_UX_MASK) && (ots->sub_ux_state & SCHED_ASSIST_UX_MASK));

	if (need_lock_rq && need_lock_pi) {
		task_rq_unlock(rq, t, &flags);
	} else if (need_lock_rq && !need_lock_pi) {
		__task_rq_unlock(rq, &flags);
	}
	set_ux_task_dsq_id(t);
}
EXPORT_SYMBOL(oplus_set_ux_state_lock);

noinline int tracing_mark_write(const char *buf)
{
	trace_printk(buf);
	return 0;
}

int is_vip_mvp(struct task_struct *p)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(p);
	if (IS_ERR_OR_NULL(ots))
		return false;

	return atomic_read(&ots->is_vip_mvp);
}

void ux_state_systrace_c(unsigned int cpu, struct task_struct *p)
{
	int ux_state = (oplus_get_ux_state(p) & (SCHED_ASSIST_UX_MASK | SA_TYPE_INHERIT | SCHED_ASSIST_UX_PRIORITY_MASK));
	if (is_vip_mvp(p))
		ux_state |= SA_TYPE_MQ_VIP;

	if (per_cpu(prev_ux_state, cpu) != ux_state) {
		char buf[256];

		snprintf(buf, sizeof(buf), "C|9999|Cpu%d_ux_state|%d\n", cpu, ux_state);
		tracing_mark_write(buf);
		per_cpu(prev_ux_state, cpu) = ux_state;
	}
}

void ux_priority_systrace_c(unsigned int cpu, struct task_struct *t)
{
	struct rq *rq;
	struct oplus_rq *orq;
	struct oplus_task_struct *ots;
	int ux_priority;
	u64 value;

	if (NULL == t) {
		return;
	}
	ots = get_oplus_task_struct(t);
	if (IS_ERR_OR_NULL(ots)) {
		return;
	}

	ux_priority = ots->ux_priority * 10 + ots->ux_nice;
	if (per_cpu(prev_ux_priority, cpu) != ux_priority) {
		char buf[256];

		snprintf(buf, sizeof(buf), "C|9998|Cpu%d_ux_priority|%d\n", cpu, ux_priority);
		tracing_mark_write(buf);
		per_cpu(prev_ux_priority, cpu) = ux_priority;
	}

	value = ots->vruntime;
	if (per_cpu(prev_vruntime, cpu) != value) {
		char buf[256];

		snprintf(buf, sizeof(buf), "C|9998|Cpu%d_vruntime|%llu\n", cpu, value);
		tracing_mark_write(buf);
		per_cpu(prev_vruntime, cpu) = value;
	}

	rq = cpu_rq(cpu);
	orq = get_oplus_rq(rq);
	value = orq->min_vruntime;
	if (per_cpu(prev_min_vruntime, cpu) != value) {
		char buf[256];

		snprintf(buf, sizeof(buf), "C|9998|Cpu%d_min_vruntime|%llu\n", cpu, value);
		tracing_mark_write(buf);
		per_cpu(prev_min_vruntime, cpu) = value;
	}

	value = ots->preset_vruntime;
	if (per_cpu(prev_preset_vruntime, cpu) != value) {
		char buf[256];

		snprintf(buf, sizeof(buf), "C|9998|Cpu%d_preset_vtime|%llu\n", cpu, value);
		tracing_mark_write(buf);
		per_cpu(prev_preset_vruntime, cpu) = value;
	}

	if (per_cpu(prev_sub_ux_state, cpu) != ots->sub_ux_state) {
		char buf[256];

		snprintf(buf, sizeof(buf), "C|9998|Cpu%d_sub_ux_state|%d\n", cpu, ots->sub_ux_state);
		tracing_mark_write(buf);
		per_cpu(prev_sub_ux_state, cpu) = ots->sub_ux_state;
	}
}

void sched_info_systrace_c(unsigned int cpu, struct task_struct *p)
{
	struct rq *rq = cpu_rq(cpu);
	struct cfs_rq *cfs_rq = &rq->cfs;
	struct rt_rq *rt_rq = &rq->rt;
	unsigned int cfs_running = cfs_rq->h_nr_running;
	unsigned int rt_running = rt_rq->rt_nr_running;
	struct oplus_task_struct *ots = get_oplus_task_struct(p);
	int ddl_hint = ots && test_bit(OTS_STATE_DDL_ACTIVE, &ots->state) ? 1 : 0;
	int ddl_task = 0;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_DDL)
	ddl_task = ots && oplus_get_task_ddl(p) ? 1 : 0;
#endif
	u64 s_info = 0;
	char buf[256];
	struct css_set *cset;
	int cpu_cid, cpuset_cid;

	cset = task_css_set(p);
	cpu_cid = cset->subsys[cpu_cgrp_id] ? cset->subsys[cpu_cgrp_id]->id : 0;
	cpuset_cid = cset->subsys[cpuset_cgrp_id] ? cset->subsys[cpuset_cgrp_id]->id : 0;

	if (cpu_cid >= SCHED_MAX_CPUCTL)
		cpu_cid = 0;
	if (cpuset_cid >= SCHED_MAX_CPUSET)
		cpuset_cid = 0;

	cfs_running = min(cfs_running, (unsigned int)SCHED_MAX_CFS_R - 1);
	rt_running = min(rt_running, (unsigned int)SCHED_MAX_RT_R - 1);

	s_info += cpuset_cid;
	s_info += cpu_cid * CPUCTL_MULT_UNIT;
	s_info += cfs_running * CFS_R_MULT_UNIT;
	s_info += rt_running * RT_R_MULT_UNIT;
	s_info += ddl_hint * DDL_ACTIVE_MULT_UNIT;
	s_info += ddl_task * DDL_TASK_MULT_UNIT;
	s_info += ((u8)cpumask_bits(p->cpus_ptr)[0]) * AFFINITY_MASK_MULT_UNIT;
	if (cpumask_weight(p->cpus_ptr) < nr_cpu_ids) {
		if (ots && likely(test_bit(OTS_STATE_SET_AFFINITY, &ots->state))
			&& ots->affinity_pid > 0 && ots->affinity_pid < PID_MAX_LIMIT)
			s_info += ((u64)ots->affinity_pid) * AFFINITY_SET_MULT_UNIT;
	}

	if (per_cpu(prev_sched_info, cpu) != s_info) {
		snprintf(buf, sizeof(buf), "C|9999|Cpu%d_sched_info|%llu\n", cpu, s_info);
		tracing_mark_write(buf);
		per_cpu(prev_sched_info, cpu) = s_info;
	}
}

void qs_info_systrace_c(unsigned int cpu, struct task_struct *p)
{
	unsigned int uclamp_min = 0, uclamp_max = 0;
	unsigned int stune_boost = 0;
	u64 s_info = 0;
	char buf[256];

	uclamp_min = uclamp_eff_value(p, UCLAMP_MIN);
	uclamp_max = uclamp_eff_value(p, UCLAMP_MAX);
#if IS_ENABLED(CONFIG_OPLUS_SCHED_TUNE)
	stune_boost = schedtune_task_boost(p) + 100;
#endif

	uclamp_min = min(uclamp_min, (unsigned int)SCHED_MAX_UCLAMP_MIN - 1);
	uclamp_max = min(uclamp_max, (unsigned int)SCHED_MAX_UCLAMP_MAX - 1);
	stune_boost = min(stune_boost, (unsigned int)SCHED_MAX_STUNE - 1);

	s_info += uclamp_min;
	s_info += uclamp_max * UCLAMP_MAX_MULT_UNIT;
	s_info += stune_boost * STUNE_MULT_UNIT;

	if (per_cpu(prev_qs_info, cpu) != s_info) {
		snprintf(buf, sizeof(buf), "C|9999|Cpu%d_qs_info|%llu\n", cpu, s_info);
		tracing_mark_write(buf);
		per_cpu(prev_qs_info, cpu) = s_info;
	}
}

void fbg_state_systrace_c(unsigned int cpu, struct task_struct *p)
{
	char buf[256];
	struct oplus_task_struct *ots = get_oplus_task_struct(p);

	if (ots) {
		snprintf(buf, sizeof(buf), "C|10000|Cpu%d_fbg_state|%d\n", cpu, ots->fbg_state);
		tracing_mark_write(buf);
	}
}

void sa_scene_systrace_c(void)
{
	static int prev_ux_scene;
	int assist_scene = global_sched_assist_scene;
	if (prev_ux_scene != assist_scene) {
		char buf[64];

		snprintf(buf, sizeof(buf), "C|9999|Ux_Scene|%d\n", global_sched_assist_scene);
		tracing_mark_write(buf);
		prev_ux_scene = assist_scene;
	}
}

#ifdef CONFIG_OPLUS_SCHED_HALT_MASK_PRT
void sa_corectl_systrace_c(void)
{
	char buf[256];
	int cur_mask;
	u64 halt_info = 0;
	unsigned int cpu;
	int *cur_client_state;

	if (likely(!(global_debug_enabled & DEBUG_SYSTRACE))) {
		return;
	}

	cur_mask = cpumask_bits(&cur_cpus_halt_mask)[0];
	snprintf(buf, sizeof(buf), "C|9999|Cpu_Halt_Mask|%d\n", cur_mask);
	tracing_mark_write(buf);

	cur_mask = cpumask_bits(&cur_cpus_phalt_mask)[0];
	snprintf(buf, sizeof(buf), "C|9999|Cpu_Partial_Halt_Mask|%d\n", cur_mask);
	tracing_mark_write(buf);


	for_each_present_cpu(cpu) {
		cur_client_state = per_cpu_ptr(oplus_cur_pause_client, cpu);
		halt_info = cur_client_state[OPLUS_HALT];
		halt_info += cur_client_state[OPLUS_PARTIAL_HALT] * SCHED_PARTIAL_HALT_OFFSET;
		snprintf(buf, sizeof(buf), "C|9999|Cpu%d_Pause_Client|%llu\n", cpu, halt_info);
		tracing_mark_write(buf);
	}
}
EXPORT_SYMBOL(sa_corectl_systrace_c);
#endif /* CONFIG_OPLUS_SCHED_HALT_MASK_PRT */

void hwbinder_systrace_c(unsigned int cpu, int flag)
{
	if (per_cpu(prev_hwbinder_flag, cpu) != flag) {
		char buf[256];

		snprintf(buf, sizeof(buf), "C|9999|Cpu%d_hwbinder|%d\n", cpu, flag);
		tracing_mark_write(buf);
		per_cpu(prev_hwbinder_flag, cpu) = flag;
	}
}

void sched_assist_init_oplus_rq(void)
{
	int cpu;
	struct oplus_rq *orq;

	for_each_possible_cpu(cpu) {
		struct rq *rq = cpu_rq(cpu);

		if (!rq) {
			ux_err("failed to init oplus rq(%d)", cpu);
			continue;
		}
		orq = kzalloc_node(sizeof(struct oplus_rq), GFP_KERNEL, cpu_to_node(cpu));
		if (!orq) {
			pr_err("alloc oplus_rq%d failed %lu\n", cpu, sizeof(struct oplus_rq));
			continue;
		}

		orq->ux_list = RB_ROOT_CACHED;
		orq->exec_timeline = RB_ROOT_CACHED;
		orq->ux_list_lock = kmalloc(sizeof(spinlock_t), GFP_KERNEL);
		spin_lock_init(orq->ux_list_lock);
		orq->nr_running = 0;
		orq->min_vruntime = 0;
		orq->load_weight = 0;
		orq->skip_ots = NULL;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_DDL)
		orq->ddl_root = RB_ROOT_CACHED;
		spin_lock_init(&orq->ddl_lock);
#endif

#ifdef CONFIG_LOCKING_PROTECT
		INIT_LIST_HEAD(&orq->locking_thread_list);
		orq->locking_list_lock = kmalloc(sizeof(spinlock_t), GFP_KERNEL);
		spin_lock_init(orq->locking_list_lock);
		orq->rq_locking_task = 0;
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOADBALANCE)
		orq->lb.pid = INVALID_PID;
#endif

#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
		per_cpu(task_lb_count, cpu_of(rq)).ux_low = 0;
		per_cpu(task_lb_count, cpu_of(rq)).ux_high = 0;
		per_cpu(task_lb_count, cpu_of(rq)).top_low = 0;
		per_cpu(task_lb_count, cpu_of(rq)).top_high = 0;
		per_cpu(task_lb_count, cpu_of(rq)).foreground_low = 0;
		per_cpu(task_lb_count, cpu_of(rq)).foreground_high = 0;
		per_cpu(task_lb_count, cpu_of(rq)).background_low = 0;
		per_cpu(task_lb_count, cpu_of(rq)).background_high = 0;
#endif /* CONFIG_OPLUS_FEATURE_SCHED_SPREAD */

#ifdef CONFIG_OPLUS_FEATURE_TICK_GRAN
		orq->resched_timer = kmalloc(sizeof(struct hrtimer), GFP_KERNEL);
#endif
		smp_mb();
		WRITE_ONCE(rq->android_oem_data1[ORQ_IDX], (u64) orq);
	}

#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
	cpumask_clear(&nr_mask);
#endif /* CONFIG_OPLUS_FEATURE_SCHED_SPREAD */
}

bool is_task_util_over(struct task_struct *tsk, int threshold)
{
	bool sum_over = false;
	bool demand_over = false;

#if IS_ENABLED(CONFIG_SCHED_WALT)
	sum_over = scale_demand(task_wts_sum(tsk)) >= threshold;
	demand_over = oplus_task_util(tsk) >= threshold;
#else
	sum_over = tsk->se.avg.util_avg >= threshold;
#endif

	return sum_over || demand_over;
}

static inline bool oplus_is_min_capacity_cpu(int cpu)
{
	struct ux_sched_cputopo ux_cputopo = ux_sched_cputopo;
	int cls_nr = ux_cputopo.cls_nr - 1;
	int cls_id = topology_cluster_id(cpu);

	if (unlikely(cls_nr <= 0))
		return false;

	return cls_id <= 0;
}

bool oplus_task_misfit(struct task_struct *tsk, int cpu)
{
	if (is_task_util_over(tsk, BOOST_THRESHOLD_UNIT) && oplus_is_min_capacity_cpu(cpu))
		return true;

	return false;
}

unsigned int ux_task_exec_limit(struct task_struct *p)
{
	int ux_state = oplus_get_ux_state(p);
	unsigned int exec_limit = UX_EXEC_SLICE;

	if (global_lowend_plat_opt && (ux_state & SA_TYPE_HEAVY) && is_heavy_load_top_task(p)) {
		exec_limit *= 40;
		return exec_limit;
	}

	if (sched_assist_scene(SA_LAUNCH) && !(ux_state & SA_TYPE_INHERIT)) {
		exec_limit *= 30;
		return exec_limit;
	}

	exec_limit = ux_max_exec_time(ux_state);
	return exec_limit;
}
EXPORT_SYMBOL_GPL(ux_task_exec_limit);

void set_im_flag_with_bit(int im_flag, struct task_struct *task)
{
	struct oplus_task_struct *ots = NULL;

	if (im_flag < 0)
		return;

	ots = get_oplus_task_struct(task);
	if (IS_ERR_OR_NULL(ots))
		return;
	if (im_flag <= IM_FLAG_CLEAR) {
		set_bit(im_flag, &ots->im_flag);
	} else { /* im_flag > 64 means clear relative bimap which is im_flag - 64 */
		clear_bit((im_flag - IM_FLAG_CLEAR), &ots->im_flag);
	}
}



bool test_task_ux(struct task_struct *task)
{
	if (unlikely(!global_sched_assist_enabled))
		return false;

	if (!task)
		return false;

	if (!test_task_is_fair(task))
		return false;

	if (oplus_get_ux_state(task) & SCHED_ASSIST_UX_MASK) {
		struct oplus_task_struct *ots;
		unsigned int limit;

		ots = get_oplus_task_struct(task);
		if (IS_ERR_OR_NULL(ots))
			return false;

		limit = ux_task_exec_limit(task);
		if (ots->total_exec && (ots->total_exec > limit)) {
			if (unlikely(global_debug_enabled & DEBUG_FTRACE))
				trace_printk("task is not ux by limit, comm=%-12s pid=%d ux_state=%d total_exec=%llu limit=%u\n",
					task->comm, task->pid, ots->ux_state, ots->total_exec, limit);

			return false;
		}

		return true;
	}

	return false;
}
EXPORT_SYMBOL_GPL(test_task_ux);

int get_ux_state_type(struct task_struct *task)
{
	struct oplus_task_struct *ots;

	if (!task)
		return UX_STATE_INVALID;

	if (!test_task_is_fair(task))
		return UX_STATE_INVALID;

	ots = get_oplus_task_struct(task);

	if (IS_ERR_OR_NULL(ots))
		return UX_STATE_NONE;

	if ((ots->ux_state & SCHED_ASSIST_UX_MASK) && (ots->sub_ux_state & SCHED_ASSIST_UX_MASK))
		return UX_STATE_COMBINED;

	if (ots->ux_state & SCHED_ASSIST_UX_MASK) {
		return (ots->ux_state & SA_TYPE_INHERIT) ? UX_STATE_INHERIT : UX_STATE_STATIC;
	}

	return UX_STATE_NONE;
}
EXPORT_SYMBOL_GPL(get_ux_state_type);

bool is_multiple_ux(struct oplus_task_struct *ots)
{
	unsigned int ux_state_type = ots->ux_state & SCHED_ASSIST_UX_MASK;
	unsigned int sub_ux_state_type = ots->sub_ux_state & SCHED_ASSIST_UX_MASK;
	unsigned int ux_type = (ux_state_type | sub_ux_state_type);

	/* has more than 1 bit */
	return ux_type && (ux_type & ux_type - 1);
}

/*s64 __maybe_unused account_ux_runtime(struct rq *rq, struct task_struct *curr)
{
	struct oplus_rq *orq = get_oplus_rq(rq);
	struct oplus_task_struct *ots = get_oplus_task_struct(curr);
	s64 delta;
	unsigned int limit;

	if (IS_ERR_OR_NULL(ots))
		return;

	lockdep_assert_rq_held(rq);

	if (!(rq->clock_update_flags & RQCF_UPDATED))
		update_rq_clock(rq);

	delta = curr->se.sum_exec_runtime - ots->sum_exec_baseline;

	if (delta < 0)
		delta = 0;
	else
		delta += rq_clock_task(rq) - curr->se.exec_start;

	if (delta < CFS_SCHED_MIN_GRAN)
		return delta;

	ots->sum_exec_baseline += delta;
	ots->total_exec += delta;

	ots->vruntime += calc_delta_fair(delta, ots->ux_priority);

	limit = ux_task_exec_limit(curr);
	if (ots->total_exec > limit) {
		remove_ux_task(orq, ots);
		put_task_struct(curr);
	} else {
		// if ux slice has expired but total exectime not, just requeue without put/get task_struct
		remove_ux_task(orq, ots);
		insert_task_to_ux_timeline(ots, orq);
	}

	return delta;
}*/

static void enqueue_ux_thread(struct rq *rq, struct task_struct *p)
{
	struct oplus_rq *orq;
	struct oplus_task_struct *ots;
	unsigned long irqflag;

	if (unlikely(!global_sched_assist_enabled))
		return;

	oplus_set_enqueue_time(p, rq->clock);
#ifdef CONFIG_OPLUS_CPU_AUDIO_PERF
	oplus_sched_assist_audio_enqueue_hook(p);
#endif

	ots = get_oplus_task_struct(p);
	if (IS_ERR_OR_NULL(ots))
		return;

	if (!test_task_is_fair(p) || !oplus_rbnode_empty(&ots->ux_entry))
		return;

	orq = get_oplus_rq(rq);
	spin_lock_irqsave(orq->ux_list_lock, irqflag);
	smp_mb__after_spinlock();
	if (!oplus_rbnode_empty(&ots->ux_entry)) {
		DEBUG_BUG_ON(1);
		spin_unlock_irqrestore(orq->ux_list_lock, irqflag);
		return;
	}

	/* task's ux entry should be initialized with INIT_LIST_HEAD() */
	/*if (ots->ux_entry.__rb_parent_color == 0)
		RB_CLEAR_NODE(&ots->ux_entry);*/

	if (test_task_ux(p)) {
		get_task_struct(p);
		if (!ots->total_exec) {
			int ux_priority, ux_nice;
			/* ots->sum_exec_baseline = p->se.sum_exec_runtime; */
			ux_priority = ux_state_to_priority(ots->ux_state);
			ux_nice = ux_type_to_nice(ots->ux_state);
			initial_prio_nice_and_vruntime(orq, ots, ux_priority, ux_nice);
		} else {
			update_vruntime_task_attach(orq, ots);
		}
		insert_task_to_ux_timeline(ots, orq);
	}
	spin_unlock_irqrestore(orq->ux_list_lock, irqflag);
}

static void dequeue_ux_thread(struct rq *rq, struct task_struct *p)
{
	struct oplus_rq *orq;
	struct oplus_task_struct *ots;
	unsigned long irqflag;

	if (!rq || !p)
		return;

	oplus_set_enqueue_time(p, 0);
	ots = get_oplus_task_struct(p);
	if (IS_ERR_OR_NULL(ots))
		return;

	orq = get_oplus_rq(rq);
	spin_lock_irqsave(orq->ux_list_lock, irqflag);
	smp_mb__after_spinlock();
	if (!oplus_rbnode_empty(&ots->ux_entry)) {
		int ux_state_type;
		update_ux_timeline_task_removal(orq, ots, NULL, false);

		/* inherit ux can only keep it's ux state in MAX_INHERIT_GRAN */
		ux_state_type = get_ux_state_type(p);
		if ((ux_state_type == UX_STATE_INHERIT || ux_state_type == UX_STATE_COMBINED) &&
			(p->se.sum_exec_runtime - ots->inherit_ux_start > get_max_inherit_gran(p))) {
			atomic64_set(&ots->inherit_ux, 0);
			ots->ux_depth = 0;
			if (ots->ux_state & SA_TYPE_INHERIT) {
				ots->ux_state = 0;
			}
			if (ots->sub_ux_state & SA_TYPE_INHERIT) {
				ots->sub_ux_state = 0;
			}
			set_ux_task_dsq_id(p);
			if (unlikely(global_debug_enabled & DEBUG_FTRACE))
				trace_printk("dequeue and unset inherit ux task=%-12s pid=%d tgid=%d sum_exec_runtime=%llu inherit_start=%llu\n",
							p->comm, p->pid, p->tgid, p->se.sum_exec_runtime, ots->inherit_ux_start);
		}

		/* if (ots->ux_state & SA_TYPE_ONCE) {
			atomic64_set(&ots->inherit_ux, 0);
			ots->ux_depth = 0;
			ots->ux_state = 0;
			set_ux_task_dsq_id(p);
			if (unlikely(global_debug_enabled & DEBUG_FTRACE))
				trace_printk("dequeue and unset once ux task=%-12s pid=%d tgid=%d inherit_start=%llu\n",
					p->comm, p->pid, p->tgid, ots->inherit_ux_start);
		}*/
		put_task_struct(p);
	}

	if (p->__state != TASK_RUNNING) {
		ots->total_exec = 0;
		ots->vruntime = 0;
		ots->preset_vruntime = 0;
	} else {
		update_vruntime_task_detach(orq, ots);
	}
	spin_unlock_irqrestore(orq->ux_list_lock, irqflag);
}

void queue_ux_thread(struct rq *rq, struct task_struct *p, int enqueue)
{
	if (enqueue)
		enqueue_ux_thread(rq, p);
	else
		dequeue_ux_thread(rq, p);
}
EXPORT_SYMBOL(queue_ux_thread);

bool test_inherit_ux(struct task_struct *task, int type)
{
	u64 inherit_ux;

	if (!task)
		return false;

	inherit_ux = oplus_get_inherit_ux(task);
	return inherit_ux_get_bits(inherit_ux, type) > 0;
}
EXPORT_SYMBOL_GPL(test_inherit_ux);

inline void inherit_ux_inc(struct task_struct *task, int type)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(task);

	if (IS_ERR_OR_NULL(ots))
		return;

	atomic64_add(inherit_ux_value(type, 1), &ots->inherit_ux);
}

inline void inherit_ux_sub(struct task_struct *task, int type, int value)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(task);

	if (IS_ERR_OR_NULL(ots))
		return;

	atomic64_sub(inherit_ux_value(type, value), &ots->inherit_ux);
}

void inc_inherit_ux_refs(struct task_struct *task, int type)
{
	inherit_ux_inc(task, type);
}
EXPORT_SYMBOL_GPL(inc_inherit_ux_refs);

inline bool test_task_ux_depth(int ux_depth)
{
	return ux_depth < UX_DEPTH_MAX;
}

bool test_set_inherit_ux(struct task_struct *tsk)
{
	int ux_depth = oplus_get_ux_depth(tsk);

	return tsk && test_task_ux(tsk) && test_task_ux_depth(ux_depth);
}
EXPORT_SYMBOL_GPL(test_set_inherit_ux);

void set_inherit_ux(struct task_struct *task, int type, int depth, int inherit_val)
{
	struct oplus_task_struct *ots;
	if (!task || type >= INHERIT_UX_MAX)
		return;

	ots = get_oplus_task_struct(task);
	if (IS_ERR_OR_NULL(ots))
		return;
	/*
	 * For PIFUTEX, &task may inherit rt prio but would lose it soon after unlock,
	 * at this case, we should check fair_class with it's ->normal_prio but not ->prio.
	 * do this check in caller.
	 */
	if (type == INHERIT_UX_PIFUTEX)
		goto set;

	if (!test_task_is_fair(task))
		return;

set:
	inherit_ux_inc(task, type);
	oplus_set_ux_depth(task, depth + 1);

	oplus_set_ux_state_lock(task,
		(inherit_val & (SCHED_ASSIST_UX_PRIORITY_MASK | SCHED_ASSIST_UX_MASK)) | SA_TYPE_INHERIT, type, true);
	oplus_set_inherit_ux_start(task, jiffies_to_nsecs(jiffies));
	trace_inherit_ux_set(task, type, oplus_get_ux_state(task), oplus_get_inherit_ux(task), oplus_get_ux_depth(task));
}
EXPORT_SYMBOL_GPL(set_inherit_ux);

void reset_inherit_ux(struct task_struct *inherit_task, struct task_struct *ux_task, int reset_type)
{
	int reset_depth = 0;
	int reset_inherit = 0;
	int ux_state;

	if (!inherit_task || !ux_task || reset_type >= INHERIT_UX_MAX)
		return;

	reset_inherit = oplus_get_ux_state(ux_task);
	reset_depth = oplus_get_ux_depth(ux_task);

	if (!test_inherit_ux(inherit_task, reset_type) || !(reset_inherit & SA_TYPE_ANIMATOR))
		return;

	ux_state = (oplus_get_inherited_ux_state(inherit_task) & ~SCHED_ASSIST_UX_MASK) | reset_inherit | SA_TYPE_INHERIT;
	oplus_set_ux_depth(inherit_task, reset_depth + 1);
	oplus_set_ux_state_lock(inherit_task, ux_state, reset_type, true);
	trace_inherit_ux_reset(inherit_task, reset_type, oplus_get_ux_state(inherit_task),
		oplus_get_inherit_ux(inherit_task), oplus_get_ux_depth(inherit_task));
}
EXPORT_SYMBOL_GPL(reset_inherit_ux);

void unset_inherit_ux_value(struct task_struct *task, int type, int value)
{
	s64 inherit_ux;
	struct oplus_task_struct *ots;

	if (!task || type >= INHERIT_UX_MAX)
		return;

	inherit_ux_sub(task, type, value);
	inherit_ux = oplus_get_inherit_ux(task);

	if (inherit_ux > 0) {
		return;
	}

	if (inherit_ux < 0)
		oplus_set_inherit_ux(task, 0);

	ots = get_oplus_task_struct(task);
	if (IS_ERR_OR_NULL(ots))
		return;

	ots->ux_depth = 0;

	oplus_set_ux_state_lock(task, 0, type, true);
	trace_inherit_ux_unset(task, type, oplus_get_ux_state(task), oplus_get_inherit_ux(task), oplus_get_ux_depth(task));
}
EXPORT_SYMBOL_GPL(unset_inherit_ux_value);

void unset_inherit_ux(struct task_struct *task, int type)
{
	unset_inherit_ux_value(task, type, 1);
}
EXPORT_SYMBOL_GPL(unset_inherit_ux);

void clear_all_inherit_type(struct task_struct *p)
{
	struct oplus_task_struct *ots;

	ots = get_oplus_task_struct(p);
	if (IS_ERR_OR_NULL(ots))
		return;

	atomic64_set(&ots->inherit_ux, 0);
	ots->ux_depth = 0;
	oplus_set_ux_state_lock(p, 0, INHERIT_UX_MAX, true);
}

int get_max_inherit_gran(struct task_struct *p)
{
	if (global_lowend_plat_opt && test_inherit_ux(p, INHERIT_UX_BINDER))
		return MAX_INHERIT_GRAN * 2;

	return MAX_INHERIT_GRAN;
}

#ifndef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
bool im_mali(const char *comm)
{
	if (!strncmp(comm, "mali-", 5)) {
		const char *postfix = comm + 5;
		return !strcmp(postfix, "event-hand") ||
			!strcmp(postfix, "mem-purge") || !strcmp(postfix, "cpu-comman") ||
			!strcmp(postfix, "compiler") || !strcmp(postfix, "cmar-backe");
	}
	return false;
}
#else
inline bool im_mali(const char *comm) {
	return false;
}
#endif

void sched_assist_target_comm(struct task_struct *task, const char *comm)
{
#ifndef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
	/* mali-event-handle need to keep ux enduringly */
	if (im_mali(comm)) {
		oplus_set_ux_state_lock(task, SA_TYPE_LIGHT, -1, true);
	}
#endif
}

void adjust_rt_lowest_mask(struct task_struct *p, struct cpumask *local_cpu_mask, int ret, bool force_adjust)
{
	struct cpumask mask_backup;
	int next_cpu = -1;
	int keep_target_cpu = -1;
	int keep_backup_cpu = -1;
	int lowest_prio = INT_MIN;
	unsigned int drop_cpu;
	struct rq *rq;
	struct task_struct *task;
	struct oplus_rq *orq;
	struct oplus_task_struct *ots = NULL;
	unsigned long irqflag;
	unsigned long im_flag;

	if (!ret || !local_cpu_mask || cpumask_empty(local_cpu_mask))
		return;

	cpumask_copy(&mask_backup, local_cpu_mask);

	drop_cpu = cpumask_first(local_cpu_mask);
	while (drop_cpu < nr_cpu_ids) {
		int ux_task_state;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_PIPELINE)
		if (oplus_pipeline_rt_skip_prime_cpu(drop_cpu)) {
			cpumask_clear_cpu(drop_cpu, local_cpu_mask);
			drop_cpu = cpumask_next(drop_cpu, local_cpu_mask);
			continue;
		}
#endif

		/* unlocked access */
		rq = cpu_rq(drop_cpu);
		orq = get_oplus_rq(rq);
		task = rcu_dereference(rq->curr);

		if (!task || (task->flags & PF_EXITING)) {
			drop_cpu = cpumask_next(drop_cpu, local_cpu_mask);
			continue;
		}

		spin_lock_irqsave(orq->ux_list_lock, irqflag);
		if (!test_task_ux(task)) {
			if (oplus_rbtree_empty(&orq->ux_list)) {
				spin_unlock_irqrestore(orq->ux_list_lock, irqflag);
				drop_cpu = cpumask_next(drop_cpu, local_cpu_mask);
				continue;
			} else {
				ots =  ux_list_first_entry(&orq->ux_list);
				if (!IS_ERR_OR_NULL(ots)) {
					task = ots->task;
					/*
					 * if PF_EXITING, the task will be free later so that it doesn't need to be preempt-protect,
					 * In the meantime, it will result to panic when calling ots or task_struct after free them.
					 * */
					if (!task || (task->flags & PF_EXITING)) {
						spin_unlock_irqrestore(orq->ux_list_lock, irqflag);
						drop_cpu = cpumask_next(drop_cpu, local_cpu_mask);
						continue;
					}
				} else {
					spin_unlock_irqrestore(orq->ux_list_lock, irqflag);
					drop_cpu = cpumask_next(drop_cpu, local_cpu_mask);
					continue;
				}
			}
		}
		ux_task_state = oplus_get_ux_state(task);
		spin_unlock_irqrestore(orq->ux_list_lock, irqflag);

		im_flag = oplus_get_im_flag(p);

		/* avoid sf premmpt heavy ux tasks,such as ui, render... */
		if ((test_bit(IM_FLAG_SURFACEFLINGER, &im_flag) || test_bit(IM_FLAG_RENDERENGINE, &im_flag)) &&
			((ux_task_state & SA_TYPE_HEAVY) || (ux_task_state & SA_TYPE_LISTPICK))) {
			cpumask_clear_cpu(drop_cpu, local_cpu_mask);
			if (unlikely(global_debug_enabled & DEBUG_FTRACE))
				trace_printk("clear cpu from lowestmask, curr_heavy task=%-12s pid=%d drop_cpu=%d\n", task->comm, task->pid, drop_cpu);
		}

#ifdef CONFIG_OPLUS_SCHED_MT6895
		if (ux_task_state & SA_TYPE_HEAVY) {
#else
		if (sched_assist_scene(SA_LAUNCH) && (ux_task_state & SA_TYPE_HEAVY)) {
#endif
			cpumask_clear_cpu(drop_cpu, local_cpu_mask);
			if (unlikely(global_debug_enabled & DEBUG_FTRACE))
				trace_printk("clear cpu from lowestmask, curr_heavy task=%-12s pid=%d drop_cpu=%d\n", task->comm, task->pid, drop_cpu);
		}

		if (ux_task_state & SA_TYPE_ANIMATOR) {
			cpumask_clear_cpu(drop_cpu, local_cpu_mask);
			if (unlikely(global_debug_enabled & DEBUG_FTRACE))
				trace_printk("clear cpu from lowestmask, curr_anima task=%-12s pid=%d drop_cpu=%d\n", task->comm, task->pid, drop_cpu);
		}

#ifdef CONFIG_OPLUS_CPU_AUDIO_PERF
		if (oplus_sched_assist_audio_perf_check_exit_latency(p, drop_cpu))
			cpumask_clear_cpu(drop_cpu, local_cpu_mask);
#endif

		drop_cpu = cpumask_next(drop_cpu, local_cpu_mask);
	}

	if (likely(!cpumask_empty(local_cpu_mask)))
		return;

	if (unlikely(global_debug_enabled & DEBUG_FTRACE))
		trace_printk("lowest mask is empty, force is %d\n", force_adjust);

	/* We may get empty local_cpu_mask if we do unsuitable drop work */
	if (!force_adjust) {
		cpumask_copy(local_cpu_mask, &mask_backup);
		return;
	}

	next_cpu = cpumask_first(&mask_backup);
	while (next_cpu < nr_cpu_ids) {
		/* unlocked access */
		struct task_struct *task;
		bool is_target;
		int prio;

		rq = cpu_rq(next_cpu);
		task = rcu_dereference(rq->curr);

		/*
		* if PF_EXITING, the task will be free later so that it doesn't need to be preempt-protect,
		* In the meantime, it will result to panic when calling ots or task_struct after free them.
		* */
		if (!task || (task->flags & PF_EXITING)) {
			next_cpu = cpumask_next(next_cpu, &mask_backup);
			continue;
		}

		is_target = !(oplus_get_ux_state(task) & SA_TYPE_ANIMATOR);
		prio = task->prio;

		if (lowest_prio == INT_MIN) {
			if (is_target)
				keep_target_cpu = next_cpu;
			else
				keep_backup_cpu = next_cpu;

			lowest_prio = prio;
		} else if (is_target && prio > lowest_prio) {
			keep_target_cpu = next_cpu;
			lowest_prio = prio;
		} else if (!is_target && prio > lowest_prio) {
			keep_backup_cpu = next_cpu;
			lowest_prio = task->prio;
		}

		next_cpu = cpumask_next(next_cpu, &mask_backup);
	}

	if (keep_target_cpu != -1)
		cpumask_set_cpu(keep_target_cpu, local_cpu_mask);
	else if (keep_backup_cpu != -1)
		cpumask_set_cpu(keep_backup_cpu, local_cpu_mask);
}
EXPORT_SYMBOL(adjust_rt_lowest_mask);

bool sa_skip_rt_sync(struct rq *rq, struct task_struct *p, bool *sync)
{
	int cpu = cpu_of(rq);
	struct oplus_rq *orq = get_oplus_rq(rq);
	struct oplus_task_struct *ots;
	unsigned long irqflag;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_PIPELINE)
	if (oplus_pipeline_rt_skip_prime_cpu(cpu)) {
		if (*sync) {
			*sync = false;
			return true;
		}
	}
#endif

	spin_lock_irqsave(orq->ux_list_lock, irqflag);
	ots = ux_list_first_entry(&orq->ux_list);
	if (IS_ERR_OR_NULL(ots) || test_bit(IM_FLAG_CAMERA_HAL, &ots->im_flag)) {
		spin_unlock_irqrestore(orq->ux_list_lock, irqflag);
		return false;
	}
	spin_unlock_irqrestore(orq->ux_list_lock, irqflag);

	if (*sync) {
		*sync = false;
		if (unlikely(global_debug_enabled & DEBUG_FTRACE))
			trace_printk("comm=%-12s pid=%d cpu=%d\n", p->comm, p->pid, cpu);

		return true;
	}

	return false;
}
EXPORT_SYMBOL(sa_skip_rt_sync);

bool sa_rt_skip_ux_cpu(int cpu)
{
	struct rq *rq;
	struct oplus_rq *orq;
	struct task_struct *curr;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_PIPELINE)
	if (oplus_pipeline_rt_skip_prime_cpu(cpu))
		return true;
#endif

	rq = cpu_rq(cpu);
	orq = get_oplus_rq(rq);
	curr = rq->curr;

	/* skip running ux */
	if (curr && test_task_ux(curr))
		return true;

	/* skip runnable ux */
	if (!oplus_rbtree_empty(&orq->ux_list))
		return true;

	return false;
}
EXPORT_SYMBOL(sa_rt_skip_ux_cpu);

ssize_t oplus_show_cpus(const struct cpumask *mask, char *buf)
{
	ssize_t i = 0;
	unsigned int cpu;

	for_each_cpu(cpu, mask) {
		if (i)
			i += scnprintf(&buf[i], (PAGE_SIZE - i - 2), " ");
		i += scnprintf(&buf[i], (PAGE_SIZE - i - 2), "%u", cpu);
		if (i >= (PAGE_SIZE - 5))
			break;
	}
	i += sprintf(&buf[i], "\n");
	return i;
}

#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
void sa_spread_systrace_c(void)
{
	static int prev_ux_spread;
	int ux_spread = should_force_spread_tasks();

	if (prev_ux_spread != ux_spread) {
		char buf[32];
		snprintf(buf, sizeof(buf), "C|9999|Ux_Spread|%d\n", ux_spread);
		tracing_mark_write(buf);
		prev_ux_spread = ux_spread;
	}
}
#endif

/* register vender hook in kernel/sched/topology.c */
void android_vh_build_sched_domains_handler(void *unused, bool has_asym)
{
	update_ux_sched_cputopo();
}

/* register vender hook in  kernel/sched/rt.c */
void android_rvh_select_task_rq_rt_handler(void *unused,
			struct task_struct *p, int prev_cpu, int sd_flag, int wake_flags, int *new_cpu)
{
}

void android_rvh_find_lowest_rq_handler(void *unused,
			struct task_struct *p, struct cpumask *local_cpu_mask, int ret, int *best_cpu)
{
	adjust_rt_lowest_mask(p, local_cpu_mask, ret, true);

	if (!ret || !local_cpu_mask)
		return;

	if (cpumask_empty(local_cpu_mask))
		return;

	if (cpumask_test_cpu(task_cpu(p), local_cpu_mask))
		*best_cpu = task_cpu(p);
	else
		*best_cpu = cpumask_first(local_cpu_mask);
}

/* register vender hook in kernel/sched/core.c */
void android_rvh_sched_fork_handler(void *unused, struct task_struct *p)
{
}

void android_rvh_after_enqueue_task_handler(void *unused, struct rq *rq, struct task_struct *p, int flags)
{
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CPU_JANKINFO)
	jankinfo_android_rvh_enqueue_task_handler(unused, rq, p, flags);
#endif
	queue_ux_thread(rq, p, 1);
}
EXPORT_SYMBOL(android_rvh_after_enqueue_task_handler);

void android_rvh_dequeue_task_handler(void *unused, struct rq *rq, struct task_struct *p, int flags)
{
	queue_ux_thread(rq, p, 0);

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CPU_JANKINFO)
	if (!rt_task(p))
		return;
	if ((flags & DEQUEUE_SLEEP)) {
		struct oplus_task_struct *ots = get_oplus_task_struct(p);
		if (ots && p->__state & TASK_UNINTERRUPTIBLE)
			ots->block_start_time = sched_clock();
	}
#endif
}
EXPORT_SYMBOL(android_rvh_dequeue_task_handler);

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
android_rvh_schedule_handler_t fbg_android_rvh_schedule_callback;
EXPORT_SYMBOL(fbg_android_rvh_schedule_callback);
#endif

#ifdef CONFIG_OPLUS_FEATURE_TICK_GRAN
static DEFINE_PER_CPU(int, prev_tick_gran_state);
void tick_gran_state_systrace_c(unsigned int cpu, int tick_gran_state)
{
	if (per_cpu(prev_tick_gran_state, cpu) != tick_gran_state) {
		char buf[256];

		snprintf(buf, sizeof(buf), "C|9999|Cpu%d_tick_gran_state|%d\n",
				cpu, tick_gran_state);
		tracing_mark_write(buf);
		per_cpu(prev_tick_gran_state, cpu) = tick_gran_state;
	}
}

void sa_sched_switch_handler(void *unused, bool preempt, struct task_struct *prev, struct task_struct *next, unsigned int prev_state)
{
	if (unlikely(global_debug_enabled & DEBUG_SYSTRACE))
		nopreempt_state_systrace_c(smp_processor_id(), 0);

	if (unlikely(global_debug_enabled & DEBUG_AMU_INSTRUCTION)) {
		if (!preempt && prev_state)
			per_cpu(nvcsw, smp_processor_id())++;
		else
			per_cpu(nivcsw, smp_processor_id())++;
	}
}
#endif

void android_vh_account_process_tick_gran_handler(void *unused, struct task_struct *p, struct rq *rq, int user_tick, int *ticks)
{
#ifdef CONFIG_OPLUS_FEATURE_TICK_GRAN
	if (unlikely(global_debug_enabled & DEBUG_DYNAMIC_HZ))
		goto out;

	if (jiffies % TICK_GRAN_NUM) {
		if (unlikely(global_debug_enabled & DEBUG_SYSTRACE))
			tick_gran_state_systrace_c(smp_processor_id(), 1);
		*ticks = 0;
	} else {
		if (unlikely(global_debug_enabled & DEBUG_SYSTRACE))
			tick_gran_state_systrace_c(smp_processor_id(), 0);
		*ticks = TICK_GRAN_NUM;
	}

out:
#endif

	if (*ticks == 0)
		return;
	trace_android_vh_account_task_time(p, rq, user_tick, *ticks);
}
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_account_task_time);

void android_rvh_schedule_handler(void *unused, struct task_struct *prev, struct task_struct *next, struct rq *rq)
{
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CPU_JANKINFO)
	jankinfo_android_rvh_schedule_handler(unused, prev, next, rq);
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	if (fbg_android_rvh_schedule_callback)
		fbg_android_rvh_schedule_callback(prev, next, rq);
#endif

	if (unlikely(global_debug_enabled & DEBUG_SYSTRACE)) {
		ux_state_systrace_c(cpu_of(rq), next);
		if (likely(prev != next)) {
			sched_info_systrace_c(cpu_of(rq), next);
			qs_info_systrace_c(cpu_of(rq), next);
		}
	}

	if (unlikely(global_debug_enabled & DEBUG_KMSG) && likely(prev != next))
		fbg_state_systrace_c(cpu_of(rq), next);

#ifdef CONFIG_LOCKING_PROTECT
#ifndef CONFIG_LOCKING_ENTITY_ELIGIBLE
	LOCKING_CALL_OP(locking_tick_hit, prev, next);
#endif
	if (unlikely(global_debug_enabled & DEBUG_SYSTRACE) && likely(prev != next))
		LOCKING_CALL_OP(state_systrace_c, cpu_of(rq), next);
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_DDL)
	oplus_task_ddl_tint(rq, next);
#endif
}
EXPORT_SYMBOL(android_rvh_schedule_handler);

void android_vh_scheduler_tick_handler(void *unused, struct rq *rq)
{
#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
	update_rq_nr_imbalance(smp_processor_id());
#endif /* CONFIG_OPLUS_FEATURE_SCHED_SPREAD */

	if (unlikely(global_debug_enabled & DEBUG_SYSTRACE)) {
		if (cpu_of(rq) == 0) {
			sa_scene_systrace_c();
#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
			sa_spread_systrace_c();
#endif /* CONFIG_OPLUS_FEATURE_SCHED_SPREAD */
		}
#if (SA_DEBUG_ON >= 1)
		if (cpu_of(rq) == 1) {
			ux_priority_systrace_c(cpu_of(rq), rq->curr);
		}
#endif
	}

#ifdef CONFIG_OPLUS_FEATURE_TICK_GRAN
	if (unlikely(global_debug_enabled & DEBUG_SYSTRACE))
		tick_gran_state_systrace_c(smp_processor_id(), 2);

	if (unlikely(global_debug_enabled & DEBUG_AMU_INSTRUCTION))
		per_cpu(retired_instrs, smp_processor_id()) = read_sysreg_s(SYS_AMEVCNTR0_INST_RET_EL0);
#endif
}

static int boost_kill = 1;
module_param_named(boost_kill, boost_kill, uint, 0644);

static inline void do_boost_kill_task(struct task_struct *p)
{
	cpumask_var_t boost_mask;
	int is_32bit = test_ti_thread_flag(&p->thread_info, TIF_32BIT);

	set_user_nice(p, -20);
	if (is_32bit)
		cpumask_and(boost_mask, cpu_active_mask, system_32bit_el0_cpumask());
	else
		cpumask_copy(boost_mask, cpu_active_mask);
	if (!cpumask_empty(boost_mask)) {
		cpumask_copy(&p->cpus_mask, boost_mask);
		p->nr_cpus_allowed = cpumask_weight(boost_mask);
	}
}

void android_vh_exit_signal_handler(void *unused, struct task_struct *p)
{
	if (p == NULL)
		return;

	if (boost_kill && bg_task(p)) {
		do_boost_kill_task(p);
	}
}

static int process_exit_notifier(struct notifier_block *self,
			unsigned long cmd, void *v)
{
	struct task_struct *p = v;

	/* only boost background tasks */
	if (boost_kill && bg_task(p)) {
		rcu_read_lock();
		do_boost_kill_task(p);
		rcu_read_unlock();
	}

	return NOTIFY_OK;
}


struct notifier_block process_exit_notifier_block = {
	.notifier_call	= process_exit_notifier,
};

void android_vh_cgroup_set_task_handler(void *unused, int ret, struct task_struct *task)
{
}

void sched_setaffinity_tracking(struct task_struct *task, const struct cpumask *in_mask)
{
	struct oplus_task_struct *ots;
	struct task_struct *leader = NULL;
	pid_t affinity_pid = -1, affinity_tgid = -1;

	ots = get_oplus_task_struct(task);
	if (IS_ERR_OR_NULL(ots))
		return;

	if (cpumask_weight(in_mask) == nr_cpu_ids) {
		clear_bit(OTS_STATE_SET_AFFINITY, &ots->state);
		ots->affinity_pid = ots->affinity_tgid = -1;
	} else {
		if (strncmp(current->comm, "OomAdjuster", TASK_COMM_LEN) == 0)
			return;

		rcu_read_lock();
		affinity_pid = current->pid;
		leader = current->group_leader;
		if (leader && pid_alive(leader))
			affinity_tgid = leader->pid;
		rcu_read_unlock();

		ots->affinity_pid = affinity_pid;
		ots->affinity_tgid = affinity_tgid;
		set_bit(OTS_STATE_SET_AFFINITY, &ots->state);
	}
}

void android_rvh_set_cpus_allowed_comm_handler(void *unused, struct task_struct *task, const struct cpumask *new_mask)
{
	sched_setaffinity_tracking(task, new_mask);
}

/* TODO: [ALM:7849986] This code should be removed after 2024.09.27 */
void android_rvh_setscheduler_handler(void *unused, struct task_struct *p)
{
	if (unlikely(p->pid == p->tgid)) {
		struct oplus_task_struct *ots = get_oplus_task_struct(p);

		if (!IS_ERR_OR_NULL(ots) && test_bit(IM_FLAG_SURFACEFLINGER, &ots->im_flag)) {
			pr_err("set scheduler: curr=%-16s tgid=%d p=%-16s tgid=%d prio=%d C:(%ps<-%ps<-%ps<-%ps<-%ps)\n",
				current->comm, current->tgid, p->comm, p->tgid, p->prio,
				(void *)CALLER_ADDR2, (void *)CALLER_ADDR3, (void *)CALLER_ADDR4,
				(void *)CALLER_ADDR5, (void *)CALLER_ADDR6);
		}
	}
}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_BAN_APP_SET_AFFINITY)
void android_vh_sched_setaffinity_early_handler(void *unused, struct task_struct *task, const struct cpumask *new_mask, bool *skip)
{
	unsigned long im_flag = IM_FLAG_NONE;
	int curr_uid = current_uid().val;

	curr_uid = curr_uid % PER_USER_RANGE;
	if ((curr_uid < FIRST_APPLICATION_UID) || (curr_uid > LAST_APPLICATION_UID))
		return;

	if (task->pid == task->tgid) {
		im_flag = oplus_get_im_flag(task);
	} else {
		struct task_struct *main_task;

		rcu_read_lock();
		main_task = find_task_by_vpid(task->tgid);
		if (main_task)
			im_flag = oplus_get_im_flag(main_task);
		rcu_read_unlock();
	}

	if (test_bit(IM_FLAG_FORBID_SET_CPU_AFFINITY, &im_flag))
		*skip = 1;
}
#endif

#ifdef CONFIG_OPLUS_SCHED_GROUP_OPT

static inline s64 entity_key(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	return (s64)(se->vruntime - cfs_rq->min_vruntime);
}

u64 avg_vruntime(struct cfs_rq *cfs_rq)
{
	struct sched_entity *curr = cfs_rq->curr;
	s64 avg = cfs_rq->avg_vruntime;
	long load = cfs_rq->avg_load;

	if (curr && curr->on_rq) {
		unsigned long weight = scale_load_down(curr->load.weight);

		avg += entity_key(cfs_rq, curr) * weight;
		load += weight;
	}

	if (load) {
		/* sign flips effective floor / ceil */
		if (avg < 0)
			avg -= (load - 1);
		avg = div_s64(avg, load);
	}

	return cfs_rq->min_vruntime + avg;
}

static inline void update_load_set(struct load_weight *lw, unsigned long w)
{
	lw->weight = w;
	lw->inv_weight = 0;
}

#define WMULT_CONST	(~0U)
#define WMULT_SHIFT	32

static void __update_inv_weight(struct load_weight *lw)
{
	unsigned long w;

	if (likely(lw->inv_weight))
		return;

	w = scale_load_down(lw->weight);

	if (BITS_PER_LONG > 32 && unlikely(w >= WMULT_CONST))
		lw->inv_weight = 1;
	else if (unlikely(!w))
		lw->inv_weight = WMULT_CONST;
	else
		lw->inv_weight = WMULT_CONST / w;
}

/*
 * delta_exec * weight / lw.weight
 *   OR
 * (delta_exec * (weight * lw->inv_weight)) >> WMULT_SHIFT
 *
 * Either weight := NICE_0_LOAD and lw \e sched_prio_to_wmult[], in which case
 * we're guaranteed shift stays positive because inv_weight is guaranteed to
 * fit 32 bits, and NICE_0_LOAD gives another 10 bits; therefore shift >= 22.
 *
 * Or, weight =< lw.weight (because lw.weight is the runqueue weight), thus
 * weight/lw.weight <= 1, and therefore our shift will also be positive.
 */
static u64 __calc_delta(u64 delta_exec, unsigned long weight, struct load_weight *lw)
{
	u64 fact = scale_load_down(weight);
	u32 fact_hi = (u32)(fact >> 32);
	int shift = WMULT_SHIFT;
	int fs;

	__update_inv_weight(lw);

	if (unlikely(fact_hi)) {
		fs = fls(fact_hi);
		shift -= fs;
		fact >>= fs;
	}

	fact = mul_u32_u32(fact, lw->inv_weight);

	fact_hi = (u32)(fact >> 32);
	if (fact_hi) {
		fs = fls(fact_hi);
		shift -= fs;
		fact >>= fs;
	}

	return mul_u64_u32_shr(delta_exec, fact, shift);
}

/*
 * delta /= w
 */
static inline u64 calc_delta_fair(u64 delta, struct sched_entity *se)
{
	if (unlikely(se->load.weight != NICE_0_LOAD))
		delta = __calc_delta(delta, NICE_0_LOAD, &se->load);

	return delta;
}

static s64 entity_lag(u64 avruntime, struct sched_entity *se)
{
	s64 vlag, limit;

	vlag = avruntime - se->vruntime;
	limit = calc_delta_fair(max_t(u64, 2*se->slice, TICK_NSEC), se);

	return clamp(vlag, -limit, limit);
}

static void reweight_eevdf(struct sched_entity *se, u64 avruntime,
			   unsigned long weight)
{
	unsigned long old_weight = se->load.weight;
	s64 vlag, vslice;

	/*
	 * VRUNTIME
	 * ========
	 *
	 * COROLLARY #1: The virtual runtime of the entity needs to be
	 * adjusted if re-weight at !0-lag point.
	 *
	 * Proof: For contradiction assume this is not true, so we can
	 * re-weight without changing vruntime at !0-lag point.
	 *
	 *             Weight	VRuntime   Avg-VRuntime
	 *     before    w          v            V
	 *      after    w'         v'           V'
	 *
	 * Since lag needs to be preserved through re-weight:
	 *
	 *	lag = (V - v)*w = (V'- v')*w', where v = v'
	 *	==>	V' = (V - v)*w/w' + v		(1)
	 *
	 * Let W be the total weight of the entities before reweight,
	 * since V' is the new weighted average of entities:
	 *
	 *	V' = (WV + w'v - wv) / (W + w' - w)	(2)
	 *
	 * by using (1) & (2) we obtain:
	 *
	 *	(WV + w'v - wv) / (W + w' - w) = (V - v)*w/w' + v
	 *	==> (WV-Wv+Wv+w'v-wv)/(W+w'-w) = (V - v)*w/w' + v
	 *	==> (WV - Wv)/(W + w' - w) + v = (V - v)*w/w' + v
	 *	==>	(V - v)*W/(W + w' - w) = (V - v)*w/w' (3)
	 *
	 * Since we are doing at !0-lag point which means V != v, we
	 * can simplify (3):
	 *
	 *	==>	W / (W + w' - w) = w / w'
	 *	==>	Ww' = Ww + ww' - ww
	 *	==>	W * (w' - w) = w * (w' - w)
	 *	==>	W = w	(re-weight indicates w' != w)
	 *
	 * So the cfs_rq contains only one entity, hence vruntime of
	 * the entity @v should always equal to the cfs_rq's weighted
	 * average vruntime @V, which means we will always re-weight
	 * at 0-lag point, thus breach assumption. Proof completed.
	 *
	 *
	 * COROLLARY #2: Re-weight does NOT affect weighted average
	 * vruntime of all the entities.
	 *
	 * Proof: According to corollary #1, Eq. (1) should be:
	 *
	 *	(V - v)*w = (V' - v')*w'
	 *	==>    v' = V' - (V - v)*w/w'		(4)
	 *
	 * According to the weighted average formula, we have:
	 *
	 *	V' = (WV - wv + w'v') / (W - w + w')
	 *	   = (WV - wv + w'(V' - (V - v)w/w')) / (W - w + w')
	 *	   = (WV - wv + w'V' - Vw + wv) / (W - w + w')
	 *	   = (WV + w'V' - Vw) / (W - w + w')
	 *
	 *	==>  V'*(W - w + w') = WV + w'V' - Vw
	 *	==>	V' * (W - w) = (W - w) * V	(5)
	 *
	 * If the entity is the only one in the cfs_rq, then reweight
	 * always occurs at 0-lag point, so V won't change. Or else
	 * there are other entities, hence W != w, then Eq. (5) turns
	 * into V' = V. So V won't change in either case, proof done.
	 *
	 *
	 * So according to corollary #1 & #2, the effect of re-weight
	 * on vruntime should be:
	 *
	 *	v' = V' - (V - v) * w / w'		(4)
	 *	   = V  - (V - v) * w / w'
	 *	   = V  - vl * w / w'
	 *	   = V  - vl'
	 */
	if (avruntime != se->vruntime) {
		vlag = entity_lag(avruntime, se);
		vlag = div_s64(vlag * old_weight, weight);
		se->vruntime = avruntime - vlag;
	}

	/*
	 * DEADLINE
	 * ========
	 *
	 * When the weight changes, the virtual time slope changes and
	 * we should adjust the relative virtual deadline accordingly.
	 *
	 *	d' = v' + (d - v)*w/w'
	 *	   = V' - (V - v)*w/w' + (d - v)*w/w'
	 *	   = V  - (V - v)*w/w' + (d - v)*w/w'
	 *	   = V  + (d - V)*w/w'
	 */
	vslice = (s64)(se->deadline - avruntime);
	vslice = div_s64(vslice * old_weight, weight);
	se->deadline = avruntime + vslice;
}

void android_vh_reweight_entity_handler(void *unused, struct sched_entity *se)
{
	if (!(global_sched_group_enabled & 0x1))
		return;

	if (!entity_is_task(se)) {
		struct cfs_rq *gcfs_rq = group_cfs_rq(se);
		if (!gcfs_rq)
			return;

		struct task_group *tg = gcfs_rq->tg;
		struct css_tg_map *oplus_tg = get_oplus_tg_map(tg);
		if (!oplus_tg || !oplus_tg->sg_info || !oplus_tg->sg_info->dynamic_share)
			return;

		unsigned long group_weight = clamp(gcfs_rq->load.weight,
			se->load.weight, scale_load(MAX_SHARES));
		if (group_weight == se->load.weight)
			return;

		struct cfs_rq *cfs_rq = cfs_rq_of(se);
		if (se->on_rq) {
			u64 avruntime = avg_vruntime(cfs_rq);
			reweight_eevdf(se, avruntime, group_weight);
		} else {
				/*
				 * Because we keep se->vlag = V - v_i, while: lag_i = w_i*(V - v_i),
				 * we need to scale se->vlag when w_i changes.
				 */
				se->vlag = div_s64(se->vlag * se->load.weight, group_weight);
		}

		update_load_set(&se->load, group_weight);
	}
}
#endif

void set_ux_task_dsq_id(struct task_struct *task)
{
}
