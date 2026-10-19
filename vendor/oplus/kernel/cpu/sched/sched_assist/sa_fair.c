// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */


#include "sched_assist.h"
#include "sa_common.h"
#include "sa_fair.h"
#include "sa_priority.h"
#include <linux/sched/cputime.h>
#include <kernel/sched/sched.h>
#include <linux/list.h>
#include <include/linux/sched.h>
#include <linux/cpuidle.h>
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
#include <../kernel/oplus_cpu/sched/frame_boost/frame_group.h>
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_VT_CAP)
#include "../eas_opt/oplus_cap.h"
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOADBALANCE)
#include "sa_balance.h"
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_PIPELINE)
#include "sa_pipeline.h"
#endif

#include "sa_group.h"

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_DDL)
#include "sa_ddl.h"
#endif

#include "trace_sched_assist.h"

#ifdef CONFIG_OPLUS_CPU_AUDIO_PERF
#include "sa_audio.h"
#endif
extern unsigned int sysctl_sched_latency;

#define MS_TO_NS (1000000)

#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
#define NR_IMBALANCE_THRESHOLD (24)
struct cpumask nr_mask;
DEFINE_PER_CPU(struct task_count_rq, task_lb_count);
EXPORT_PER_CPU_SYMBOL(task_lb_count);
#endif

#ifdef CONFIG_OPLUS_ADD_CORE_CTRL_MASK
struct cpumask *ux_cpu_halt_mask;
#endif

int oplus_idle_cpu(int cpu)
{
	struct rq *rq = cpu_rq(cpu);

	if (rq->curr != rq->idle)
		return 0;

	if (rq->nr_running)
		return 0;

#if IS_ENABLED(CONFIG_SMP)
	if (rq->ttwu_pending)
		return 0;
#endif

	return 1;
}

inline bool is_heavy_load_ux_task(struct task_struct *task)
{
	struct ux_sched_cputopo ux_cputopo = ux_sched_cputopo;
	int heavy_load_threshold;
	bool ret = false;

	struct oplus_task_struct *ots = get_oplus_task_struct(task);
	if (IS_ERR_OR_NULL(ots))
		return false;

	heavy_load_threshold = (ux_cputopo.sched_cls[0].capacity >> 1);
	ret = is_task_util_over(task, heavy_load_threshold);

	return ret;
}

inline bool is_high_priority_pipeline_task(struct task_struct *task)
{
	int ux_priority;

	struct oplus_task_struct *ots = get_oplus_task_struct(task);
	if (IS_ERR_OR_NULL(ots))
		return false;

	/* Audio thread is usually a light thread, not need boost to prime cpus */
	if (ots->ux_state & SA_TYPE_SWIFT)
		return false;

	ux_priority = ux_state_to_priority(ots->ux_state);
	/* Above of UX_PRIORITY_PIPELINE be regarded as high priority base on local testing*/
	if (ux_priority > (uint)(UX_PRIORITY_PIPELINE >> SCHED_ASSIST_UX_PRIORITY_SHIFT))
		return true;

	return false;
}

inline int get_task_cls_for_scene(struct task_struct *task)
{
	struct ux_sched_cputopo ux_cputopo = ux_sched_cputopo;
	int cls_max = ux_cputopo.cls_nr - 1;
	int cls_mid = cls_max - 1;
	unsigned long im_flag;

	/* only one cluster or init failed */
	if (unlikely(cls_max <= 0))
		return 0;

	/* for 2 clusters cpu, mid = max */
	if (cls_mid == 0) {
		if (global_silver_perf_core)
			cls_max = cls_mid;
		else
			cls_mid = cls_max;
	}

	if (global_lowend_plat_opt) {
		if (global_less_prime_cpu_arch) {
			if (is_high_priority_pipeline_task(task))
				return cls_mid;
			else
				return 0;
		}

		/* for launch scene, heavy ux task should not move to min capacity cluster */
		if (sched_assist_scene(SA_LAUNCH) && test_ux_type(task, SA_TYPE_HEAVY | SA_TYPE_ANIMATOR)
			&& is_heavy_load_ux_task(task))
			return test_ux_type(task, SA_TYPE_ANIMATOR) ? cls_mid : cls_max;

		/* TOP APP's UI&Renderthread boost to big CPU */
		if (test_ux_type(task, SA_TYPE_HEAVY) && is_heavy_load_top_task(task))
			return cls_mid;
	}

	/* for launch scene, heavy ux task should not move to min capacity cluster */
	if (sched_assist_scene(SA_LAUNCH) && test_ux_type(task, SA_TYPE_HEAVY | SA_TYPE_ANIMATOR))
		return test_ux_type(task, SA_TYPE_ANIMATOR) ? cls_mid : cls_max;

	/*if (global_lowend_plat_opt && test_ux_type(task, SA_TYPE_HEAVY) && is_heavy_load_top_task(task))
		return cls_mid;
	*/

	if (sched_assist_scene(SA_ANIM) && test_ux_type(task, SA_TYPE_ANIMATOR))
		return is_task_util_over(task, BOOST_THRESHOLD_UNIT) ? cls_mid : 0;

	if (sched_assist_scene(SA_LAUNCHER_SI))
		return is_task_util_over(task, BOOST_THRESHOLD_UNIT) ? cls_mid : 0;

	im_flag = oplus_get_im_flag(task);
	if (test_bit(IM_FLAG_CAMERA_HAL, &im_flag))
		return cls_mid;

	return 0;
}

/*
 * The margin used when comparing utilization with CPU capacity.
 *
 * (default: ~20%)
 */
#define fits_capacity(cap, max)	((cap) * 1280 < (max) * 1024)

#ifdef CONFIG_UCLAMP_TASK
static inline unsigned long uclamp_task_util(struct task_struct *p)
{
	return clamp(oplus_task_util(p),
		     uclamp_eff_value(p, UCLAMP_MIN),
		     uclamp_eff_value(p, UCLAMP_MAX));
}
#else
static inline unsigned long uclamp_task_util(struct task_struct *p)
{
	return oplus_task_util(p);
}
#endif

static inline bool task_fits_capacity(struct task_struct *p, long capacity)
{
	return fits_capacity(uclamp_task_util(p), capacity);
}

static inline bool task_fits_max(struct task_struct *p, int dst_cpu)
{
	unsigned long capacity = 0;

#ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
	capacity = capacity_orig_of(dst_cpu);
#else
	struct rq *rq = cpu_rq(dst_cpu);

	capacity = rq->cpu_capacity;
#endif

	return task_fits_capacity(p, capacity);
}

/* Todo:  @bug:7901603 This function needs to be put into is_ux_task_prefer_cpu_for_scene */
#ifdef CONFIG_ARCH_MEDIATEK
static inline bool ux_eas_skip_little_cluster(struct task_struct *p, int dst_cpu)
{
	int cls_id = topology_cluster_id(dst_cpu);
	int prefer_cls_id = get_task_cls_for_scene(p);

	if (cls_id == 0 && prefer_cls_id == 0)
		return task_fits_max(p, dst_cpu);

	return true;
}
#endif

static inline bool is_ux_task_prefer_cpu_for_scene(struct task_struct *task, unsigned int cpu)
{
	struct ux_sched_cputopo ux_cputopo = ux_sched_cputopo;
	int prefer_cls_id = ux_cputopo.cls_nr - 1;
#ifdef CONFIG_ARCH_MEDIATEK
	int cls_id = topology_cluster_id(cpu);
#endif

	/* only one cluster or init failed */
	if (unlikely(prefer_cls_id <= 0))
		return true;

	prefer_cls_id = get_task_cls_for_scene(task);
#ifdef CONFIG_ARCH_MEDIATEK
	/* mtk will modify cpu_capacity_orig, direct comparison of cap is not accurate*/
	return cls_id >= prefer_cls_id;
#else
	return arch_scale_cpu_capacity(cpu) >= ux_cputopo.sched_cls[prefer_cls_id].capacity;
#endif /* CONFIG_ARCH_MEDIATEK */
}

static inline bool skip_rt_and_ux(struct task_struct *p)
{
	return !(sched_assist_scene(SA_LAUNCH) && p->pid == p->tgid
		&& !test_ux_type(p, SA_TYPE_URGENT_MASK));
}

bool should_ux_task_skip_cpu(struct task_struct *task, unsigned int dst_cpu)
{
	struct oplus_rq *orq = NULL;
	int reason = -1;
	unsigned long im_flag;

	if (unlikely(!global_sched_assist_enabled))
		return false;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_PIPELINE)
	if (oplus_pipeline_task_skip_cpu(task, dst_cpu))
		return true;
#endif

	if (!test_task_ux(task))
		return false;

	if (!is_ux_task_prefer_cpu_for_scene(task, dst_cpu)) {
		reason = 0;
		goto skip;
	}

	if (skip_rt_and_ux(task)) {
		if (cpu_rq(dst_cpu)->rt.rt_nr_running) {
			reason = 1;
			goto skip;
		}

		/* camera hal thread only skip rt, because they are too much,
		 * if they skip each other, maybe easily jump to super big core. :(
		 */
		im_flag = oplus_get_im_flag(task);
		if (test_bit(IM_FLAG_CAMERA_HAL, &im_flag))
			return false;

		orq = get_oplus_rq(cpu_rq(dst_cpu));
		if (orq_has_ux_tasks(orq)) {
			reason = 2;
			goto skip;
		}
	}

	return false;

skip:
	if (unlikely(global_debug_enabled & DEBUG_FTRACE))
		trace_printk("ux task=%-12s pid=%d skip_cpu=%d reason=%d\n", task->comm, task->pid, dst_cpu, reason);

	return true;
}
EXPORT_SYMBOL(should_ux_task_skip_cpu);

static inline bool strict_ux_task(struct task_struct *task)
{
	return sched_assist_scene(SA_LAUNCH) && (task->pid == task->tgid)
		&& (task->tgid == save_top_app_tgid);
}

int get_topology_cluster_id(int cpu)
{
#ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
	struct ux_sched_cputopo ux_cputopo = ux_sched_cputopo;
	int cls_nr = ux_cputopo.cls_nr;
	int i = 0;

	for (i = 0; i < cls_nr; i++) {
		if (cpumask_test_cpu(cpu, &ux_cputopo.sched_cls[i].cpus))
			return i;
	}
#endif

	return topology_cluster_id(cpu);
}

static inline bool select_target_cpu_fastpath(struct task_struct *task, int target_cpu)
{
	struct rq *orig_rq = cpu_rq(target_cpu);
	struct oplus_rq *orig_orq = get_oplus_rq(orig_rq);

	bool latency_sensitive = false;

	if (global_lowend_plat_opt && global_less_prime_cpu_arch) {
		/* Audio thread prefer to idle cpu as latency_sensitive, so respect this choice */
		oplus_sched_assist_audio_latency_sensitive(task, &latency_sensitive);
		if ((latency_sensitive == true) && oplus_idle_cpu(target_cpu))
			return true;
		else
			return false;
	}

	if (test_task_ux(orig_rq->curr))
		return false;

	if (orq_has_ux_tasks(orig_orq))
		return false;

	if (orig_rq->rt.rt_nr_running)
		return false;

#ifdef CONFIG_ARCH_MEDIATEK
	if (!ux_eas_skip_little_cluster(task, target_cpu))
		return false;
#endif

	if (!is_ux_task_prefer_cpu_for_scene(task, target_cpu))
		return false;

	if (is_vip_mvp(orig_rq->curr))
		return false;

	return true;
}

#define lsub_positive(_ptr, _val) do {				\
	typeof(_ptr) ptr = (_ptr);				\
	*ptr -= min_t(typeof(*ptr), *ptr, _val);		\
} while (0)

static inline unsigned long task_util(struct task_struct *p)
{
	return READ_ONCE(p->se.avg.util_avg);
}

static inline unsigned long _task_util_est(struct task_struct *p)
{
	return READ_ONCE(p->se.avg.util_est) & ~UTIL_AVG_UNCHANGED;
}

static unsigned long
cpu_util(int cpu, struct task_struct *p, int dst_cpu, int boost)
{
	struct cfs_rq *cfs_rq = &cpu_rq(cpu)->cfs;
	unsigned long util = READ_ONCE(cfs_rq->avg.util_avg);
	unsigned long runnable;

	if (boost) {
		runnable = READ_ONCE(cfs_rq->avg.runnable_avg);
		util = max(util, runnable);
	}

	/*
	 * If @dst_cpu is -1 or @p migrates from @cpu to @dst_cpu remove its
	 * contribution. If @p migrates from another CPU to @cpu add its
	 * contribution. In all the other cases @cpu is not impacted by the
	 * migration so its util_avg is already correct.
	 */
	if (p && task_cpu(p) == cpu && dst_cpu != cpu)
		lsub_positive(&util, task_util(p));
	else if (p && task_cpu(p) != cpu && dst_cpu == cpu)
		util += task_util(p);

	if (sched_feat(UTIL_EST)) {
		unsigned long util_est;

		util_est = READ_ONCE(cfs_rq->avg.util_est);

		/*
		 * During wake-up @p isn't enqueued yet and doesn't contribute
		 * to any cpu_rq(cpu)->cfs.avg.util_est.enqueued.
		 * If @dst_cpu == @cpu add it to "simulate" cpu_util after @p
		 * has been enqueued.
		 *
		 * During exec (@dst_cpu = -1) @p is enqueued and does
		 * contribute to cpu_rq(cpu)->cfs.util_est.enqueued.
		 * Remove it to "simulate" cpu_util without @p's contribution.
		 *
		 * Despite the task_on_rq_queued(@p) check there is still a
		 * small window for a possible race when an exec
		 * select_task_rq_fair() races with LB's detach_task().
		 *
		 *   detach_task()
		 *     deactivate_task()
		 *       p->on_rq = TASK_ON_RQ_MIGRATING;
		 *       -------------------------------- A
		 *       dequeue_task()                    \
		 *         dequeue_task_fair()              + Race Time
		 *           util_est_dequeue()            /
		 *       -------------------------------- B
		 *
		 * The additional check "current == p" is required to further
		 * reduce the race window.
		 */
		if (p && unlikely(task_on_rq_queued(p) || current == p))
			lsub_positive(&util_est, _task_util_est(p));

		util = max(util, util_est);
	}

	return min(util, capacity_orig_of(cpu));
}

static unsigned long cpu_util_without(int cpu, struct task_struct *p)
{
	/* Task has no contribution or is new */
	if (cpu != task_cpu(p) || !READ_ONCE(p->se.avg.last_update_time))
		p = NULL;

	return cpu_util(cpu, p, -1, 0);
}

/* access capacity_orig/cpu_capacity value that aware sugov/walt freqency limiter
 * capacity_orig: qcom android_rvh_update_cpu_capacity
 * cpu_capacity: mtk mtk_update_cpu_capacity
 */
static inline unsigned long oplus_capacity_spare_of(int cpu, struct task_struct *p)
{
#ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
	return max_t(long, capacity_orig_of(cpu) - cpu_util_without(cpu, p), 0);
#else
	return max_t(long, cpu_rq(cpu)->cpu_capacity - cpu_util_without(cpu, p), 0);
#endif
}

static inline unsigned long cpu_util_cum(int cpu)
{
	struct cfs_rq *cfs_rq;
	unsigned int util = 0;

	cfs_rq = &cpu_rq(cpu)->cfs;
	util = READ_ONCE(cfs_rq->avg.util_avg);

	if (sched_feat(UTIL_EST))
		util = max(util, READ_ONCE(cfs_rq->avg.util_est));

	return min_t(unsigned long, util, capacity_orig_of(cpu));
}

static inline unsigned int get_idle_exit_latency(struct rq *rq)
{
	struct cpuidle_state *idle = idle_get_state(rq);

	if (idle)
		return idle->exit_latency;

	return 0; /* CPU is not idle */
}

bool set_ux_task_to_prefer_cpu(struct task_struct *task, int *orig_target_cpu)
{
	struct rq *rq = NULL;
	struct oplus_rq *orq = NULL;
	struct ux_sched_cputopo ux_cputopo = ux_sched_cputopo;
	int cls_nr = ux_cputopo.cls_nr - 1;
	int start_cls = -1;
	int cpu = 0;
	int direction = -1;
	int orig_cls_id = 0;
	cpumask_t search_cpus = CPU_MASK_NONE;
	int max_spare_cap_cpu = -1;
	int best_idle_cpu = -1;
	unsigned int min_exit_latency = UINT_MAX;
	unsigned long best_idle_cuml_util = ULONG_MAX;
	bool walk_next_cls = true;
	bool ux_cls_boost = false;
	unsigned int cpu_rq_ux_runnable_cnt = UINT_MAX;
	int least_nr_cpu = -1;
	int subopt_cpu = -1, vip_cpu = -1, max_subopt_cpu = -1;
	long spare_cap = 0, subopt_max_spare_cap = 0;
	long vip_max_spare_cap = -1, max_spare_cap = -1, rt_max_spare_cap = -1;

	if (unlikely(!global_sched_assist_enabled))
		return false;

	if (unlikely(cls_nr <= 0))
		return false;

	if (!test_task_ux(task))
		return false;

	/* 1. fastpath */
	if (*orig_target_cpu >= 0 && *orig_target_cpu < OPLUS_NR_CPUS) {
		orig_cls_id = get_topology_cluster_id(*orig_target_cpu);
		if (select_target_cpu_fastpath(task, *orig_target_cpu))
			return false;
	}

	start_cls = cls_nr = get_task_cls_for_scene(task);
	ux_cls_boost = start_cls > 0 ? true : false;
	/*
	 * Avoiding ux core selection can easily lead to small cores for tasks
	 * that would otherwise be on large cores
	 */
	if (!(global_lowend_plat_opt && global_less_prime_cpu_arch)) {
		if (start_cls < orig_cls_id) {
			start_cls = orig_cls_id;
			cls_nr = orig_cls_id;
		}
	}
	if (cls_nr != ux_cputopo.cls_nr - 1)
		direction = 1;

retry:
	cpumask_and(&search_cpus, task->cpus_ptr, cpu_active_mask);
#ifdef CONFIG_OPLUS_ADD_CORE_CTRL_MASK
	if (ux_cpu_halt_mask)
		cpumask_andnot(&search_cpus, &search_cpus, ux_cpu_halt_mask);
#endif /* CONFIG_OPLUS_ADD_CORE_CTRL_MASK */
	cpumask_and(&search_cpus, &search_cpus,
		&ux_cputopo.sched_cls[cls_nr].cpus);

	for_each_cpu(cpu, &search_cpus) {
		rq = cpu_rq(cpu);
		orq = get_oplus_rq(rq);

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_PIPELINE)
		if (oplus_pipeline_task_skip_cpu(task, cpu))
			continue;
#endif

		/* fit status to check if taks util fits cpu capacity */
		if (global_lowend_plat_opt && global_less_prime_cpu_arch) {
			if (cls_nr == 0 && ((oplus_task_util(task) > (ux_cputopo.sched_cls[1].capacity >> 1)) || ux_cls_boost))
				break;
		} else {
			if (cls_nr == 0 && (!task_fits_max(task, cpu) || ux_cls_boost))
				break;
		}

		/*
		 * Find an optimal backup IDLE CPU
		 * Looking for:
		 * - favoring shallowest idle states
		 * - CPU utilization
		 */
		if (available_idle_cpu(cpu)) {
			unsigned long new_util_cuml = 0;
			unsigned int idle_exit_latency = get_idle_exit_latency(rq);

			if (idle_exit_latency > min_exit_latency)
				continue;

			new_util_cuml = cpu_util_cum(cpu);
			if (idle_exit_latency == min_exit_latency && new_util_cuml > best_idle_cuml_util)
				continue;

			best_idle_cpu = cpu;
			min_exit_latency = idle_exit_latency;
			best_idle_cuml_util = new_util_cuml;
			continue;
		}

		/* If there is an idle cpu, then only the idle cpu is checked */
		if (best_idle_cpu != -1)
			continue;

		/*
		 * case: The system runs on a heavy load picking no cpu, and prevent
		 * EAS picking a small core, pick max_spare_cap cpu and first cluster
		 */
		spare_cap = oplus_capacity_spare_of(cpu, task);
		/* Avoid task always been choosed to prime+ cpu who have highest capacity on low-end platform */
		if (global_lowend_plat_opt && global_less_prime_cpu_arch) {
			if ((topology_physical_package_id(cpu) == start_cls) && (spare_cap > subopt_max_spare_cap)) {
				subopt_max_spare_cap = spare_cap;
				max_subopt_cpu = cpu;
			}
		} else {
			if (spare_cap > subopt_max_spare_cap) {
				subopt_max_spare_cap = spare_cap;
				max_subopt_cpu = cpu;
			}
		}

		/*
		 * Keep track of runnables for each CPU, if none of the
		 * CPUs have spare capacity then use CPU with less
		 * number of ux runnables.
		 */
		if (orq->nr_running < cpu_rq_ux_runnable_cnt) {
			cpu_rq_ux_runnable_cnt = orq->nr_running;
			least_nr_cpu = cpu;
		}

		/*
		 * strict_ux case: The system runs on a heavy load picking no cpu,
		 * and prevent EAS picking a small core, pick max_spare_cap cpu
		 * and first cluster
		 */
		if (walk_next_cls && strict_ux_task(task) && !global_silver_perf_core)
			subopt_cpu = cpu;

		/* If an ux thread running on this CPU, drop it! */
		if (oplus_get_ux_state(rq->curr) & SCHED_ASSIST_UX_MASK)
			continue;

		if (orq_has_ux_tasks(orq))
			continue;

		if (rq->curr->prio < MAX_RT_PRIO) {
			if (global_lowend_plat_opt && global_less_prime_cpu_arch) {
				continue;
			} else {
				if (spare_cap > rt_max_spare_cap) {
					rt_max_spare_cap = spare_cap;
					subopt_cpu = cpu;
				}
				continue;
			}
		}

		/* If there are rt threads in runnable state on this CPU, drop it! */
		if (rt_rq_is_runnable(&rq->rt))
			continue;

		/* Find an optimal backup vip CPU for max_spare_cap */
		if (is_vip_mvp(rq->curr)) {
			if (spare_cap > vip_max_spare_cap) {
				vip_max_spare_cap = spare_cap;
				vip_cpu = cpu;
			}
			continue;
		}

		/*
		 * Compute the maximum possible capacity we expect
		 * to have available on this CPU once the task is
		 * enqueued here.
		 */
		if (spare_cap > max_spare_cap) {
			max_spare_cap = spare_cap;
			max_spare_cap_cpu = cpu;
		}
	}

	/* 2. cpu select idle cpu -> max_spare_cap cpu */
	if (best_idle_cpu != -1) {
		trace_set_ux_task_to_prefer_cpu(task, "idle",
					*orig_target_cpu, best_idle_cpu,
					start_cls, cls_nr,
					&search_cpus);
			*orig_target_cpu = best_idle_cpu;
			return true;
	}

	if (max_spare_cap_cpu != -1) {
		trace_set_ux_task_to_prefer_cpu(task, "spare_cap",
						*orig_target_cpu, max_spare_cap_cpu,
						start_cls, cls_nr,
						&search_cpus);
		*orig_target_cpu = max_spare_cap_cpu;
		return true;
	}

	walk_next_cls = false;
	cls_nr = cls_nr + direction;
	if (global_lowend_plat_opt) {
		if (cls_nr >= 0 && cls_nr < ux_cputopo.cls_nr) {
			goto retry;
		} else if (cls_nr == ux_cputopo.cls_nr && start_cls != 0) {
			cls_nr = start_cls - 1;
			direction = -1;
			goto retry;
		}
	} else {
		if (cls_nr > 0 && cls_nr < ux_cputopo.cls_nr)
			goto retry;
	}

	/* 3 No cpu select, Preempt VIP threads, Priority: ux > VIP. */
	if (vip_cpu != -1) {
		trace_set_ux_task_to_prefer_cpu(task, "vip",
						*orig_target_cpu, vip_cpu,
						start_cls, cls_nr,
						&search_cpus);
		*orig_target_cpu = vip_cpu;
		return true;
	}

	/* 4 No cpu select, RT: max_spare_cap/strict_ux */
	if (subopt_cpu != -1) {
		trace_set_ux_task_to_prefer_cpu(task, "subopt",
						*orig_target_cpu, subopt_cpu,
						start_cls, cls_nr,
						&search_cpus);
		*orig_target_cpu = subopt_cpu;
		return true;
	}

	/* 5 No cpu select, cpu:max_spare_cap */
	if (max_subopt_cpu != -1) {
		trace_set_ux_task_to_prefer_cpu(task, "spare_sub",
						*orig_target_cpu, max_subopt_cpu,
						start_cls, cls_nr,
						&search_cpus);
		*orig_target_cpu = max_subopt_cpu;
		return true;
	}

	/* 6 No cpu select, Keep track of runnables for each CPU */
	if (least_nr_cpu != -1) {
		trace_set_ux_task_to_prefer_cpu(task, "nr_cpu",
						*orig_target_cpu, least_nr_cpu,
						start_cls, cls_nr,
						&search_cpus);
		*orig_target_cpu = least_nr_cpu;
		return true;
	}

	return false;
}
EXPORT_SYMBOL(set_ux_task_to_prefer_cpu);

bool should_ux_task_skip_eas(struct task_struct *p)
{
	return test_task_ux(p) && global_sched_assist_scene && !sched_assist_scene(SA_CAMERA);
}
EXPORT_SYMBOL(should_ux_task_skip_eas);

#ifdef CONFIG_FAIR_GROUP_SCHED
/* Walk up scheduling entities hierarchy */
#define for_each_sched_entity(se) \
		for (; se; se = se->parent)
#else
#define for_each_sched_entity(se) \
		for (; se; se = NULL)
#endif

extern void set_next_entity(struct cfs_rq *cfs_rq, struct sched_entity *se);
void oplus_replace_next_task_fair(struct rq *rq, struct task_struct **p, struct sched_entity **se, bool *repick, bool simple)
{
	struct oplus_rq *orq = get_oplus_rq(rq);
	struct rb_node *node;
	unsigned long irqflag;

	if (unlikely(!global_sched_assist_enabled))
		return;

	spin_lock_irqsave(orq->ux_list_lock, irqflag);
	smp_mb__after_spinlock();
	if (!orq_has_ux_tasks(orq)) {
		spin_unlock_irqrestore(orq->ux_list_lock, irqflag);
		return;
	}

	while ((node = rb_first_cached(&orq->ux_list)) != NULL) {
		struct oplus_task_struct *ots = rb_entry(node, struct oplus_task_struct, ux_entry);
		struct task_struct *candidate = ots_to_ts(ots);

		if (IS_ERR_OR_NULL(candidate))
			continue;

		if (unlikely(task_cpu(candidate) != rq->cpu)) {
			update_ux_timeline_task_removal(orq, ots, NULL, false);
			put_task_struct(candidate);
			DEBUG_BUG_ON(1);
			continue;
		}

		if (unlikely(!test_task_ux(candidate))) {
			update_ux_timeline_task_removal(orq, ots, NULL, false);
			put_task_struct(candidate);

			/*
			 * WARNING:
			 * Too many print logs may cause the following problems
			 * so WARN_ON here is not smart:
			 * a) this may affect standby power consumption;
			 * b) Too many logs may cause the device to crash because
			 *	  it currently holds rq->lock;
			 */
			/* WARN_ON(1); */
			continue;
		}

		/*
		* Avoid running the skip buddy, if running something else can
		* be done without getting too unfair.
		*/
		if (orq->skip_ots) {
			if (orq->skip_ots == ots) {
				struct rb_node *next_node = rb_next(node);
				if (next_node != NULL) {
					struct oplus_task_struct *second = rb_entry(next_node, struct oplus_task_struct, ux_entry);
					if (preempt_compare(second, ots, UX_YIELD_GRAN) < 1) {
						ots = second;
						candidate = ots_to_ts(ots);
						DEBUG_BUG_ON(candidate == NULL);
					}
				}
			}

			/* skipped task is repicked again, reset skip_ots */
			if (orq->skip_ots == ots) {
				orq->skip_ots = NULL;
			}
		}

		/*
		 * new task cpu must equals to this cpu, or is_same_group return null,
		 * it will cause stability issue in pick_next_task_fair()
		 */
		if (task_cpu(candidate) == cpu_of(rq)) {
			*p = candidate;
			*se = &candidate->se;
			*repick = true;
		} else
			pr_err("cpu%d replace ux task failed, ux_task cpu%d, \n", cpu_of(rq), task_cpu(candidate));

		break;
	}
	spin_unlock_irqrestore(orq->ux_list_lock, irqflag);
}

#ifdef CONFIG_OPLUS_FEATURE_TICK_GRAN
static DEFINE_PER_CPU(int, prev_nopreempt_state);
void nopreempt_state_systrace_c(unsigned int cpu, int nopreempt_state)
{
	if (per_cpu(prev_nopreempt_state, cpu) != nopreempt_state) {
		char buf[256];

		snprintf(buf, sizeof(buf), "C|9999|Cpu%d_nopreempt_state|%d\n",
				cpu, nopreempt_state);
		tracing_mark_write(buf);
		per_cpu(prev_nopreempt_state, cpu) = nopreempt_state;
	}
}
EXPORT_SYMBOL(nopreempt_state_systrace_c);

enum hrtimer_restart no_preempt_resched(struct hrtimer *timer)
{
	struct oplus_rq *orq = container_of(&timer, struct oplus_rq, resched_timer);
	struct rq *rq = cpu_rq(orq->cpu);
	struct rq_flags rf;

	rq_lock(rq, &rf);
	resched_curr(rq);
	rq_unlock(rq, &rf);

	if (unlikely(global_debug_enabled & DEBUG_SYSTRACE))
		nopreempt_state_systrace_c(orq->cpu, 0);

	return HRTIMER_NORESTART;
}

void resched_timer_init(void)
{
#ifdef CONFIG_TICK_GRAN_RESCHED_TIMER
	struct rq *rq;
	struct oplus_rq *orq;
	int i;

	for_each_possible_cpu(i) {
		rq = cpu_rq(i);
		orq = get_oplus_rq(rq);
		orq->cpu = i;
		hrtimer_init(orq->resched_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		orq->resched_timer->function = &no_preempt_resched;
	}
#endif
}
EXPORT_SYMBOL(resched_timer_init);
#endif

inline void oplus_check_preempt_wakeup(struct rq *rq, struct task_struct *p, bool *preempt, bool *nopreempt)
{
	struct task_struct *curr;
	struct oplus_rq *orq;
	struct oplus_task_struct *curr_ots;
	unsigned long irqflag;
	bool wake_ux;
	bool curr_ux;
#ifdef CONFIG_OPLUS_FEATURE_TICK_GRAN
	u64 ran;
#ifdef CONFIG_TICK_GRAN_RESCHED_TIMER
	u64 preempt_min_gran_ns = 1000000ULL;
	ktime_t ktime;
#endif
	int cpu = cpu_of(rq);
#endif

	/* this cpu is running in this function, no rcu primitives needed*/
	curr = rq->curr;
	curr_ots = get_oplus_task_struct(curr);
#ifdef CONFIG_LOCKING_PROTECT
	LOCKING_CALL_OP(check_preempt_wakeup, rq, p, preempt, nopreempt);
	if (*nopreempt == true)
		return;
#endif

	if (likely(!global_sched_assist_enabled))
		return;

	wake_ux = test_task_ux(p);
	curr_ux = test_task_ux(curr);

	if (!wake_ux && !curr_ux) {
#ifdef CONFIG_OPLUS_FEATURE_TICK_GRAN
		if (unlikely(global_debug_enabled & DEBUG_DYNAMIC_PREEMPT))
			return;

		ran = curr->se.sum_exec_runtime - curr->se.prev_sum_exec_runtime;
		if (ran <= 500000ULL && PRIO_TO_NICE((p)->static_prio) >= 0) {
			*nopreempt = true;
			if (unlikely(global_debug_enabled & DEBUG_SYSTRACE))
				nopreempt_state_systrace_c(cpu, 1);
#ifdef CONFIG_TICK_GRAN_RESCHED_TIMER
			if (!hrtimer_active(&orq->resched_timer)) {
				ktime = ns_to_ktime(preempt_min_gran_ns);
				hrtimer_start(orq->resched_timer, ktime, HRTIMER_MODE_REL);
			}
#endif
		}
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_DDL)
		if (global_sched_ddl_enabled)
			oplus_ddl_check_preempt(rq, p, curr, preempt, nopreempt);
#endif

		return;
	}

	/* ux can preempt un-ux */
	if (wake_ux && !curr_ux) {
		*preempt = true;
		return;
	}

	if (!wake_ux && curr_ux) {
		*nopreempt = true;
		return;
	}

	/* both of wake_task and curr_task are ux */
	orq = get_oplus_rq(rq);
	spin_lock_irqsave(orq->ux_list_lock, irqflag);
	smp_mb__after_spinlock();
	if (!IS_ERR_OR_NULL(curr_ots) && !oplus_rbnode_empty(&curr_ots->ux_entry)) {
		struct oplus_task_struct *first_ots = ux_list_first_entry(&orq->ux_list);
		DEBUG_BUG_ON(first_ots == NULL);
		/* account_ux_runtime(rq, curr); */
		if (first_ots != curr_ots && preempt_compare(curr_ots, first_ots, CFS_WAKEUP_GRAN) == 1) {
			*preempt = true;
		} else {
			*nopreempt = true;
		}
	}
	spin_unlock_irqrestore(orq->ux_list_lock, irqflag);
}
EXPORT_SYMBOL(oplus_check_preempt_wakeup);

#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
bool is_spread_task_enabled(void)
{
	return (global_sched_assist_enabled & FEATURE_SPREAD) && !sched_assist_scene(SA_CAMERA);
}
EXPORT_SYMBOL(is_spread_task_enabled);

void update_rq_nr_imbalance(int cpu)
{
	int total_nr = 0;
	int i = -1;
	int threshold = NR_IMBALANCE_THRESHOLD;

	/* Note: check without holding rq lock */
	for_each_cpu(i, cpu_active_mask) {
		total_nr += cpu_rq(i)->nr_running;
		if (oplus_idle_cpu(i))
			cpumask_clear_cpu(i, &nr_mask);
	}

	if (!oplus_idle_cpu(cpu) && (total_nr >= threshold))
		cpumask_set_cpu(cpu, &nr_mask);
	else
		cpumask_clear_cpu(cpu, &nr_mask);
}

bool should_force_spread_tasks(void)
{
	return !cpumask_empty(&nr_mask);
}
EXPORT_SYMBOL(should_force_spread_tasks);

int task_lb_sched_type(struct task_struct *tsk)
{
	if (test_task_ux(tsk))
		return SA_UX;
	else if (ta_task(tsk))
		return SA_TOP;
	else if (fg_task(tsk) || rootcg_task(tsk))
		return SA_FG;
	else if (bg_task(tsk))
		return SA_BG;

	return SA_INVALID;
}
EXPORT_SYMBOL(task_lb_sched_type);

void dec_task_lb(struct task_struct *tsk, struct rq *rq,
	int high_load, int task_type)
{
	int cpu = cpu_of(rq);

	if (high_load == SA_HIGH_LOAD) {
		switch (task_type) {
		case SA_UX:
			per_cpu(task_lb_count, cpu).ux_high--;
			break;
		case SA_TOP:
			per_cpu(task_lb_count, cpu).top_high--;
			break;
		case SA_FG:
			per_cpu(task_lb_count, cpu).foreground_high--;
			break;
		case SA_BG:
			per_cpu(task_lb_count, cpu).background_high--;
			break;
		}
	} else if (high_load == SA_LOW_LOAD) {
		switch (task_type) {
		case SA_UX:
			per_cpu(task_lb_count, cpu).ux_low--;
			break;
		case SA_TOP:
			per_cpu(task_lb_count, cpu).top_low--;
			break;
		case SA_FG:
			per_cpu(task_lb_count, cpu).foreground_low--;
			break;
		case SA_BG:
			per_cpu(task_lb_count, cpu).background_low--;
			break;
		}
	}
}
EXPORT_SYMBOL(dec_task_lb);

void inc_task_lb(struct task_struct *tsk, struct rq *rq,
	int high_load, int task_type)
{
	int cpu = cpu_of(rq);

	if (high_load == SA_HIGH_LOAD) {
		switch (task_type) {
		case SA_UX:
			per_cpu(task_lb_count, cpu).ux_high++;
			break;
		case SA_TOP:
			per_cpu(task_lb_count, cpu).top_high++;
			break;
		case SA_FG:
			per_cpu(task_lb_count, cpu).foreground_high++;
			break;
		case SA_BG:
			per_cpu(task_lb_count, cpu).background_high++;
			break;
		}
	} else if (high_load == SA_LOW_LOAD) {
		switch (task_type) {
		case SA_UX:
			per_cpu(task_lb_count, cpu).ux_low++;
			break;
		case SA_TOP:
			per_cpu(task_lb_count, cpu).top_low++;
			break;
		case SA_FG:
			per_cpu(task_lb_count, cpu).foreground_low++;
			break;
		case SA_BG:
			per_cpu(task_lb_count, cpu).background_low++;
			break;
		}
	}
}
EXPORT_SYMBOL(inc_task_lb);

void inc_ld_stats(struct task_struct *tsk, struct rq *rq)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(tsk);
	int curr_high_load;
	int curr_task_type;

	if (IS_ERR_OR_NULL(ots))
		return;

	curr_high_load = ots->lb_state & 0x1;
	curr_task_type = (ots->lb_state >> 1) & 0x7;

	inc_task_lb(tsk, rq, curr_high_load, curr_task_type);
	ots->ld_flag = -1;
}
EXPORT_SYMBOL(inc_ld_stats);

void dec_ld_stats(struct task_struct *tsk, struct rq *rq)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(tsk);
	int curr_high_load;
	int curr_task_type;

	if (IS_ERR_OR_NULL(ots))
		return;

	curr_high_load = ots->lb_state & 0x1;
	curr_task_type = (ots->lb_state >> 1) & 0x7;

	ots->ld_flag = 0;
	dec_task_lb(tsk, rq, curr_high_load, curr_task_type);
}
EXPORT_SYMBOL(dec_ld_stats);

#endif /* CONFIG_OPLUS_FEATURE_SCHED_SPREAD */

/* implement vender hook in driver/android/fair.c */
void android_rvh_place_entity_handler(void *unused, struct cfs_rq *cfs_rq, struct sched_entity *se, int initial, u64 *vruntime)
{
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_VT_CAP)
	struct task_struct *se_task = NULL;
	int cpu = cpu_of(rq_of(cfs_rq));
	unsigned int cluster_id = topology_physical_package_id(cpu);
	u64 adjust_time = 0;

	if (!sa_adjust_group_enable || oplus_cap_multiple[cluster_id] <= 100)
		return;

	if (!oplus_entity_is_task(se) || initial)
		return;

	se_task = task_of(se);
	if (test_task_ux(se_task))
		return;

	switch (get_grp_adinfo(se_task)) {
	case AD_TOP:
		adjust_time = (group_adjust.adjust_std_vtime_slice * group_adjust.group_param[AD_TOP].vtime_compensate * oplus_cap_multiple[cluster_id]);
		break;
	case AD_FG:
		adjust_time = (group_adjust.adjust_std_vtime_slice * group_adjust.group_param[AD_FG].vtime_compensate * oplus_cap_multiple[cluster_id]);
		break;
	case AD_BG:
		adjust_time = (group_adjust.adjust_std_vtime_slice * group_adjust.group_param[AD_BG].vtime_compensate * oplus_cap_multiple[cluster_id]);
		break;
	case AD_DF:
		adjust_time = (group_adjust.adjust_std_vtime_slice * group_adjust.group_param[AD_DF].vtime_compensate * oplus_cap_multiple[cluster_id]);
		break;
	default:
		break;
	}
	adjust_time = clamp_val(adjust_time, 0, se->vruntime);
	se->vruntime -= adjust_time;
	if (unlikely(eas_opt_debug_enable))
		trace_printk("[eas_opt]: common:%s, pid: %d, cpu: %d, group_id: %d, adjust_time: %llu, adjust_after_vtime: %llu\n",
				se_task->comm, se_task->pid, cpu, get_grp_adinfo(se_task), adjust_time, se->vruntime);
#endif
}

#ifdef OPLUS_UX_EEVDF_COMPATIBLE
void android_rvh_update_deadline_handler(void *unused, struct cfs_rq *cfs_rq, struct sched_entity *se, bool *skip_preempt) {
	if (entity_is_task(se)) {
		struct task_struct *curr =  task_of(se);
		struct oplus_task_struct *ots = get_oplus_task_struct(curr);

		if (!IS_ERR_OR_NULL(ots) && !oplus_rbnode_empty(&ots->ux_entry)) {
			/* skip eevdf preempt when ux task is running */
			*skip_preempt = true;
		}
	}
}
#endif

void android_rvh_enqueue_entity_handler(void *unused, struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	struct task_struct *p = entity_is_task(se) ? task_of(se) : NULL;
	struct rq *rq = rq_of(cfs_rq);
#ifdef OPLUS_UX_EEVDF_COMPATIBLE
	/* if enqueue back the current task */
	if ((cfs_rq->curr == se) && entity_is_task(se)) {
		struct task_struct *curr = task_of(se);
		struct oplus_task_struct *ots = get_oplus_task_struct(curr);
		if (!IS_ERR_OR_NULL(ots) && !oplus_rbnode_empty(&ots->ux_entry)) {
			exclude_ux_vruntime(se);
		}
	}
#endif

#ifdef CONFIG_LOCKING_PROTECT
	LOCKING_CALL_OP(enqueue_entity, rq, p);
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_DDL)
	oplus_enqueue_ddl_node(rq, p);
#endif
}


void android_rvh_dequeue_entity_handler(void *unused, struct cfs_rq *cfs, struct sched_entity *se)
{
	struct task_struct *p = entity_is_task(se) ? task_of(se) : NULL;
	struct rq *rq = rq_of(cfs);
#ifdef CONFIG_LOCKING_PROTECT
	LOCKING_CALL_OP(dequeue_entity, rq, p);
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_DDL)
	oplus_dequeue_ddl_node(rq, p);
#endif
}


void android_rvh_check_preempt_wakeup_handler(void *unused, struct rq *rq, struct task_struct *p, bool *preempt, bool *nopreempt,
	int wake_flags, struct sched_entity *se, struct sched_entity *pse, int next_buddy_marked)
{
	oplus_check_preempt_wakeup(rq, p, preempt, nopreempt);
}

#ifndef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
/*add hook for new task util init*/
void android_rvh_post_init_entity_util_avg_handler(void *unused, struct sched_entity *se)
{
}
#endif

void android_rvh_replace_next_task_fair_handler(void *unused,
		struct rq *rq, struct task_struct **p, struct sched_entity **se, bool *repick, bool simple, struct task_struct *prev)
{
	oplus_replace_next_task_fair(rq, p, se, repick, simple);
#ifdef CONFIG_LOCKING_PROTECT
	if (*repick != true)
		LOCKING_CALL_OP(replace_next_task_fair, rq, p, se, repick, simple);
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_DDL)
	if (*repick != true && global_sched_ddl_enabled)
		oplus_replace_next_task_ddl(rq, p, se, repick, simple);
#endif

	/*
	* NOTE:
	* Because the following code is not merged in kernel-5.15,
	* set_next_entity() will no longer be called to remove the
	* task from the red-black tree when pick_next_task_fair(),
	* so we remove the picked task here.
	*
	* https://android-review.googlesource.com/c/kernel/common/+/1667002
	*/
	if (simple && true == *repick) {
		for_each_sched_entity((*se)) {
			struct cfs_rq *cfs_rq = cfs_rq_of(*se);
			set_next_entity(cfs_rq, *se);
		}
	}
}
EXPORT_SYMBOL(android_rvh_replace_next_task_fair_handler);

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
migrate_task_callback_t fbg_migrate_task_callback;
EXPORT_SYMBOL(fbg_migrate_task_callback);
#endif

void android_rvh_can_migrate_task_handler(void *unused, struct task_struct *p, int dst_cpu, int *can_migrate)
{
	if (should_ux_task_skip_cpu(p, dst_cpu))
		*can_migrate = 0;

	/* have indicated that migration is rejected, no need more judgement */
	if (*can_migrate == 0)
		return;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	if (fbg_migrate_task_callback &&
		fbg_migrate_task_callback(p, task_cpu(p), dst_cpu))
		*can_migrate = 0;
#endif
}

void task_tpd_mask(struct task_struct *tsk, cpumask_t *request)
{
}
EXPORT_SYMBOL(task_tpd_mask);

bool task_tpd_check(struct task_struct *tsk, int dst_cpu)
{
	return true;
}
EXPORT_SYMBOL(task_tpd_check);

#ifdef CONFIG_OPLUS_ADD_CORE_CTRL_MASK
bool oplus_cpu_halted(unsigned int cpu)
{
	return ux_cpu_halt_mask && cpumask_test_cpu(cpu, ux_cpu_halt_mask);
}

void init_ux_halt_mask(struct cpumask *halt_mask)
{
	ux_cpu_halt_mask = halt_mask;
}
EXPORT_SYMBOL_GPL(init_ux_halt_mask);
#endif
