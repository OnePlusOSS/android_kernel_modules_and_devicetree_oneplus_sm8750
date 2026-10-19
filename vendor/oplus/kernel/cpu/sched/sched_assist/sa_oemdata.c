// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Oplus. All rights reserved.
 */
#include <linux/slab.h>
#include <linux/sched/task.h>
#include <linux/minmax.h>
#include <linux/align.h>
#include <asm/cache.h>
#include <linux/topology.h>
#include <linux/vmalloc.h>
#include <asm/barrier.h>
#include <uapi/linux/sched/types.h>
#include <linux/sched/cputime.h>
#include <kernel/sched/sched.h>
#include "sa_oemdata.h"
#include "sched_assist.h"
#include "sa_common.h"

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOADBALANCE)
#include "sa_balance.h"
#endif

static struct kmem_cache *oplus_task_struct_cachep;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_POWERMODEL)
void (*ots_free_powermodel_task_state)(struct oplus_task_struct *ots) = NULL;
EXPORT_SYMBOL(ots_free_powermodel_task_state);
#endif

static inline void init_oplus_task_struct(struct oplus_task_struct *ots);

static inline struct oplus_task_struct *alloc_oplus_task_struct(struct task_struct *task)
{
	struct oplus_task_struct *ots = kmem_cache_alloc(oplus_task_struct_cachep, GFP_ATOMIC);
	if (IS_ERR_OR_NULL(ots))
		return NULL;

	init_oplus_task_struct(ots);
	ots->task = task;
	return ots;
}

static inline void free_oplus_task_struct(struct oplus_task_struct *ots)
{
	if (!ots)
		return;

	kmem_cache_free(oplus_task_struct_cachep, ots);
}

void android_vh_dup_task_struct_handler(void *unused,
		struct task_struct *tsk, struct task_struct *orig)
{
	struct oplus_task_struct *ots;
	struct oplus_task_struct *orig_ots;

	if (!tsk || !orig)
		return;
	/* The required space has been allocated */
	if (!IS_ERR_OR_NULL((void *)tsk->android_oem_data1[OTS_IDX]))
		return;

	ots = alloc_oplus_task_struct(tsk);
	if (IS_ERR_OR_NULL(ots))
		return;

	/* if thread fork from RenderThread, inherit its IM_FLAG_RENDER_THREAD */
	orig_ots = get_oplus_task_struct(orig);
	if (!IS_ERR_OR_NULL(orig_ots)) {
		if (test_bit(IM_FLAG_RENDER_THREAD, &orig_ots->im_flag) && !strcmp(orig->comm, "RenderThread")) {
			set_bit(IM_FLAG_RENDER_THREAD, &ots->im_flag);
		}
	}

	smp_mb();

	WRITE_ONCE(tsk->android_oem_data1[OTS_IDX], (u64) ots);

	sched_setaffinity_tracking(tsk, tsk->cpus_ptr);
}

void android_vh_free_task_handler(void *unused, struct task_struct *tsk)
{
	struct oplus_task_struct *ots = NULL;

	if (!tsk)
		return;

	ots = (struct oplus_task_struct *) READ_ONCE(tsk->android_oem_data1[OTS_IDX]);
	if (IS_ERR_OR_NULL(ots))
		return;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOADBALANCE)
	/*
	 * NOTE:
	 * When the task is destroyed, the task needs to be removed from the
	 * rt_boost linked list, otherwise it may cause a crash due to access
	 * to an illegal address.
	 */
	remove_rt_boost_task(tsk);
#endif

	WRITE_ONCE(tsk->android_oem_data1[OTS_IDX], 0);
	smp_mb();

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_POWERMODEL)
	if (ots_free_powermodel_task_state != NULL) {
		ots_free_powermodel_task_state(ots);
	}
#endif

	ots->im_flag = 0;

	free_oplus_task_struct(ots);
}

static int register_oemdata_hooks(void)
{
	int ret = 0;

	REGISTER_TRACE_VH(android_vh_dup_task_struct, android_vh_dup_task_struct_handler);
	REGISTER_TRACE_VH(android_vh_free_task, android_vh_free_task_handler);

	return ret;
}

static void unregister_oemdata_hooks(void)
{
	UNREGISTER_TRACE_VH(android_vh_dup_task_struct, android_vh_dup_task_struct_handler);
	UNREGISTER_TRACE_VH(android_vh_free_task, android_vh_free_task_handler);
}

/*
 * NOTE:
 * Initialize the oplus_task_struct here.
 */
static void init_oplus_task_struct(struct oplus_task_struct *ots)
{
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_POWERMODEL)
	int cpu;
#endif

	memset(ots, 0, sizeof(struct oplus_task_struct));

	RB_CLEAR_NODE(&ots->ux_entry);
	RB_CLEAR_NODE(&ots->exec_time_node);
	atomic64_set(&ots->inherit_ux, 0);
	ots->ux_priority = -1;
	ots->ux_nice = -1;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_ABNORMAL_FLAG)
	ots->abnormal_flag = 0;
#endif
#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
	ots->lb_state = 0;
	ots->ld_flag = 0;
#endif
	ots->target_process = -1;
	ots->update_running_start_time = false;
/*#if IS_ENABLED(CONFIG_OPLUS_LOCKING_STRATEGY)*/
	memset(&ots->lkinfo, 0, sizeof(struct locking_info));
	INIT_LIST_HEAD(&ots->lkinfo.node);
/*#endif*/
	INIT_LIST_HEAD(&ots->fbg_list);

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_DDL)
	RB_CLEAR_NODE(&ots->ddl_node);
#endif

#ifdef CONFIG_LOCKING_PROTECT
	INIT_LIST_HEAD(&ots->locking_entry);
	ots->locking_start_time = 0;
	ots->locking_depth = 0;
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOADBALANCE)
	/* for loadbalance */
	plist_node_init(&ots->rtb, MAX_IM_FLAG_PRIO);
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_PIPELINE)
	atomic_set(&ots->pipeline_cpu, -1);
	ots->is_immuned_thread = 0;
#endif

#if IS_ENABLED(CONFIG_ARM64_AMU_EXTN) && IS_ENABLED(CONFIG_OPLUS_FEATURE_CPU_JANKINFO)
	ots->amu_cycle = 0;
	ots->amu_instruct = 0;
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_POWERMODEL)
	for (cpu = 0; cpu < MAX_CPU_CNT; cpu++) {
		ots->powermodel_cpu_task_states[cpu] = NULL;
	}
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_QOS_SCHED)
	ots->qos_level = -1;
	ots->qos_recover_prio = -2;
	mutex_init(&ots->qs_mutex);
#endif

	raw_spin_lock_init(&ots->fbg_list_entry_lock);
	ots->preferred_cluster_id = -1;
	ots->fbg_depth = -1;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_CFBT)
	ots->cfbt_cur_group = -1;
	ots->cfbt_running = false;
#endif /* CONFIG_OPLUS_FEATURE_SCHED_CFBT */
}

static void alloc_ots_mem_for_all_threads(void)
{
	struct task_struct *p, *g;
	u32 iter_cpu;

	read_lock(&tasklist_lock);

	for_each_process_thread(g, p) {
		struct oplus_task_struct *ots = NULL;

		ots = (struct oplus_task_struct *) READ_ONCE(p->android_oem_data1[OTS_IDX]);
		if (IS_ERR_OR_NULL(ots)) {
			ots = alloc_oplus_task_struct(p);
			if (!IS_ERR_OR_NULL(ots)) {
				smp_mb();
				WRITE_ONCE(p->android_oem_data1[OTS_IDX], (u64) ots);
			}
		}
	}
	for_each_possible_cpu(iter_cpu) {
		struct oplus_task_struct *ots = NULL;

		p = cpu_rq(iter_cpu)->idle;
		ots = (struct oplus_task_struct *) READ_ONCE(p->android_oem_data1[OTS_IDX]);
		if (IS_ERR_OR_NULL(ots)) {
			ots = alloc_oplus_task_struct(p);
			if (!IS_ERR_OR_NULL(ots)) {
				smp_mb();
				WRITE_ONCE(p->android_oem_data1[OTS_IDX], (u64) ots);
			}
		}
	}
	read_unlock(&tasklist_lock);
}

int sa_oemdata_init(void)
{
	oplus_task_struct_cachep = kmem_cache_create("oplus_task_struct",
			sizeof(struct oplus_task_struct), 0,
			SLAB_PANIC | SLAB_ACCOUNT, NULL);

	if (!oplus_task_struct_cachep)
		return -ENOMEM;

	alloc_ots_mem_for_all_threads();

	register_oemdata_hooks();

	return 0;
}

void __maybe_unused sa_oemdata_deinit(void)
{
	unregister_oemdata_hooks();
	kmem_cache_destroy(oplus_task_struct_cachep);
}

