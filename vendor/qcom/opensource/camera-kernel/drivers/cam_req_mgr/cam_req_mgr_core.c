// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include "cam_req_mgr_interface.h"
#include "cam_req_mgr_util.h"
#include "cam_req_mgr_core.h"
#include "cam_req_mgr_workq.h"
#include "cam_req_mgr_debug.h"
#include "cam_trace.h"
#include "cam_debug_util.h"
#include "cam_req_mgr_dev.h"
#include "cam_req_mgr_debug.h"
#include "cam_common_util.h"
#include "cam_mem_mgr.h"
#include "cam_mem_mgr_api.h"
#include "cam_cpas_api.h"
#ifdef OPLUS_FEATURE_CAMERA_COMMON
#include <cam_kevent_fb_custom.h>
#endif

static struct cam_req_mgr_core_device *g_crm_core_dev;
static struct cam_req_mgr_core_link g_links[MAXIMUM_LINKS_CAPACITY];

static void __cam_req_mgr_reset_apply_data(struct cam_req_mgr_core_link *link)
{
	int pd;

	for (pd = 0; pd < CAM_PIPELINE_DELAY_MAX; pd++) {
		link->req.apply_data[pd].req_id = -1;
		link->req.prev_apply_data[pd].req_id = -1;
	}
}

static void __cam_req_mgr_reset_state_monitor_array(
	struct cam_req_mgr_core_link *link)
{
	int i = 0;
	struct cam_req_mgr_state_monitor *state_monitor;

	for (i = 0; i < MAX_REQ_STATE_MONITOR_NUM; i++) {
		state_monitor = &link->req.state_monitor[i];
		state_monitor->req_state = CAM_CRM_STATE_INVALID;
		state_monitor->req_id = -1;
		state_monitor->frame_id = -1;
		memset(&state_monitor->time_stamp, 0, sizeof(struct timespec64));
		memset(&state_monitor->name, 0, sizeof(state_monitor->name));
	}
}

void cam_req_mgr_core_link_reset(struct cam_req_mgr_core_link *link)
{
	int i = 0;

	spin_lock_bh(&link->req.reset_link_spin_lock);
	link->link_hdl = 0;
	link->num_devs = 0;
	link->max_delay = CAM_PIPELINE_DELAY_0;
	link->min_delay = CAM_PIPELINE_DELAY_2;
	link->workq = NULL;
	link->pd_mask = 0;
	link->l_dev = NULL;
	link->req.in_q = NULL;
	link->req.l_tbl = NULL;
	link->req.num_tbl = 0;
	link->watchdog = NULL;
	spin_lock_bh(&link->link_state_spin_lock);
	link->state = CAM_CRM_LINK_STATE_AVAILABLE;
	spin_unlock_bh(&link->link_state_spin_lock);
	link->parent = NULL;
	link->sync_link_sof_skip = false;
	link->open_req_cnt = 0;
	link->last_flush_id = -1;
	link->initial_sync_req = -1;
	link->dual_trigger = false;
	link->trigger_cnt[0][CAM_TRIGGER_POINT_SOF] = 0;
	link->trigger_cnt[0][CAM_TRIGGER_POINT_EOF] = 0;
	link->trigger_cnt[1][CAM_TRIGGER_POINT_SOF] = 0;
	link->trigger_cnt[1][CAM_TRIGGER_POINT_EOF] = 0;
	link->in_msync_mode = false;
	link->retry_cnt = 0;
	link->is_shutdown = false;
	link->initial_skip = true;
	link->sof_timestamp = 0;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	link->shift_timestamp = 0;
#endif
	link->prev_sof_timestamp = 0;
	link->skip_init_frame = false;
	link->num_sync_links = 0;
	link->last_sof_trigger_jiffies = 0;
	link->wq_congestion = false;
	link->try_for_internal_recovery = false;
	link->is_sending_req = false;
	link->resume_sync_curr_mask = 0;
	link->resume_sync_dev_mask = 0;
	atomic_set(&link->eof_event_cnt, 0);
	link->cont_empty_slots = 0;
	__cam_req_mgr_reset_apply_data(link);
	__cam_req_mgr_reset_state_monitor_array(link);

	for (i = 0; i < MAXIMUM_LINKS_PER_SESSION - 1; i++)
		link->sync_link[i] = NULL;
	spin_unlock_bh(&link->req.reset_link_spin_lock);

	mutex_lock(&link->lock);
	link->properties_mask = CAM_LINK_PROPERTY_NONE;
	mutex_unlock(&link->lock);
}

void cam_req_mgr_handle_core_shutdown(void)
{
	struct cam_req_mgr_core_session *session;
	struct cam_req_mgr_core_session *tsession;
	struct cam_req_mgr_session_info ses_info;

	if (!list_empty(&g_crm_core_dev->session_head)) {
		list_for_each_entry_safe(session, tsession,
			&g_crm_core_dev->session_head, entry) {
			ses_info.session_hdl =
				session->session_hdl;
			cam_req_mgr_destroy_session(&ses_info, true);
		}
	}
}

static int __cam_req_mgr_setup_payload(struct cam_req_mgr_core_workq *workq)
{
	int32_t                  i = 0;
	int                      rc = 0;
	struct crm_task_payload *task_data = NULL;

	task_data = CAM_MEM_ZALLOC_ARRAY(
		workq->task.num_task, sizeof(*task_data),
		GFP_KERNEL);
	if (!task_data) {
		rc = -ENOMEM;
	} else {
		for (i = 0; i < workq->task.num_task; i++)
			workq->task.pool[i].payload = &task_data[i];
	}

	return rc;
}

/**
 * __cam_req_mgr_find_pd_tbl()
 *
 * @brief    : Find pipeline delay based table pointer which matches delay
 * @tbl      : Pointer to list of request table
 * @delay    : Pipeline delay value to be searched for comparison
 *
 * @return   : pointer to request table for matching pipeline delay table.
 *
 */
static struct cam_req_mgr_req_tbl *__cam_req_mgr_find_pd_tbl(
	struct cam_req_mgr_req_tbl *tbl, int32_t delay)
{
	if (!tbl)
		return NULL;

	do {
		if (delay != tbl->pd)
			tbl = tbl->next;
		else
			return tbl;
	} while (tbl != NULL);

	return NULL;
}

/**
 * __cam_req_mgr_inc_idx()
 *
 * @brief    : Increment val passed by step size and rollover after max_val
 * @val      : value to be incremented
 * @step     : amount/step by which val is incremented
 * @max_val  : max val after which idx will roll over
 *
 */
static void __cam_req_mgr_inc_idx(int32_t *val, int32_t step, int32_t max_val)
{
	*val = (*val + step) % max_val;
}

/**
 * __cam_req_mgr_dec_idx()
 *
 * @brief    : Decrement val passed by step size and rollover after max_val
 * @val      : value to be decremented
 * @step     : amount/step by which val is decremented
 * @max_val  : after zero value will roll over to max val
 *
 */
static void __cam_req_mgr_dec_idx(int32_t *val, int32_t step, int32_t max_val)
{
	*val = *val - step;
	if (*val < 0)
		*val = max_val + (*val);
}

/**
 * __cam_req_mgr_inject_delay()
 *
 * @brief         : Check if any pd device is injecting delay
 * @tbl           : cam_req_mgr_req_tbl
 * @curr_idx      : slot idx
 * @trigger       : Trigger point
 *
 * @return   : 0 for success, negative for failure
 */
static int __cam_req_mgr_inject_delay(
	struct cam_req_mgr_req_tbl *tbl,
	int32_t curr_idx, uint32_t trigger)
{
	struct cam_req_mgr_tbl_slot *slot = NULL;
	int rc = 0;

	while (tbl) {
		slot = &tbl->slot[curr_idx];
		if ((trigger == CAM_TRIGGER_POINT_SOF) &&
			(slot->inject_delay_at_sof > 0)) {
			slot->inject_delay_at_sof--;
			CAM_DBG(CAM_CRM,
				"Delay injected by pd %d device at SOF, inject_delay_at_sof:%d",
				tbl->pd, slot->inject_delay_at_sof);
			rc = -EAGAIN;
		} else if ((trigger == CAM_TRIGGER_POINT_EOF) &&
			(slot->inject_delay_at_eof > 0)) {
			slot->inject_delay_at_eof--;
			CAM_DBG(CAM_CRM,
				"Delay injected by pd %d device at EOF, inject_delay_at_eof:%d",
				tbl->pd, slot->inject_delay_at_eof);
			rc = -EAGAIN;
		}
		__cam_req_mgr_dec_idx(&curr_idx, tbl->pd_delta,
			tbl->num_slots);
		tbl = tbl->next;
	}
	return rc;
}

/**
 * __cam_req_mgr_dev_handle_to_name()
 *
 * @brief    : Finds device name based on the device handle
 * @dev_hdl  : Device handle whose name is to be found
 * @link     : Link on which the device is connected
 * @return   : String containing the device name
 *
 */
static char *__cam_req_mgr_dev_handle_to_name(
	int32_t dev_hdl, struct cam_req_mgr_core_link *link)
{
	struct cam_req_mgr_connected_device *dev = NULL;
	int i = 0;

	for (i = 0; i < link->num_devs; i++) {
		dev = &link->l_dev[i];

		if (dev_hdl == dev->dev_hdl)
			return dev->dev_info.name;
	}

	return "Invalid dev_hdl";
}

/**
 * __cam_req_mgr_name_to_dev_handle()
 *
 * @brief      : Finds device handle based on the device name
 * @dev_name   : Device name whose dev handle is to be found
 * @link       : Link on which the device is connected
 * @return     : device handle
 *
 */
static int32_t __cam_req_mgr_name_to_dev_handle(
	char *dev_name, struct cam_req_mgr_core_link *link)
{
	struct cam_req_mgr_connected_device *dev = NULL;
	int i = 0;

	for (i = 0; i < link->num_devs; i++) {
		dev = &link->l_dev[i];
		CAM_DBG(CAM_CRM, "dev name %s, iterate dev name %s",
			dev_name, dev->dev_info.name);

		if (strcmp(dev_name, dev->dev_info.name) == 0)
			return dev->dev_hdl;
	}
	return -EINVAL;
}

/**
 * __cam_req_mgr_find_dev_name()
 *
 * @brief      : Find the dev name whose req is not ready
 * @link       : link info
 * @req_id     : req_id which is not ready
 * @pd         : pipeline delay
 * @masked_val : masked value holds the bit for all devices
 *               that don't have the req_id ready for a given
 *               pipeline delay
 * @pd         : pipeline delay
 *
 */
static void __cam_req_mgr_find_dev_name(
	struct cam_req_mgr_core_link *link,
	int64_t req_id, uint32_t pd, uint32_t masked_val)
{
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	char trace[64] = {0};
#endif
	int i = 0;
	struct cam_req_mgr_connected_device *dev = NULL;

	for (i = 0; i < link->num_devs; i++) {
		dev = &link->l_dev[i];
		if (dev->dev_info.p_delay == pd) {
			if (masked_val & BIT(dev->dev_bit))
				continue;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
			if (link->wq_congestion) {
				CAM_INFO_RATE_LIMIT(CAM_CRM,
					"WQ congestion, Skip Frame: req: %lld not ready on link: 0x%x for pd: %d dev: %s open_req count: %u",
					req_id, link->link_hdl, pd,
					dev->dev_info.name, link->open_req_cnt);
				snprintf(trace, sizeof(trace), "KMD %d_4 Skip Frame", link->link_hdl);
				trace_long(trace, req_id);
				trace_int(trace, 0);
				trace_begin_end("WQ congestion, Skip Frame: req: %lld not ready on link: 0x%x for pd: %d dev: %d_%d_%s open_req count: %d",
					req_id, link->link_hdl, pd,
					dev->dev_info.dev_id, dev->dev_info.dev_hdl, dev->dev_info.name, link->open_req_cnt);
			}
			else {
				CAM_INFO(CAM_CRM,
					"Skip Frame: req: %lld not ready on link: 0x%x for pd: %d dev: %s open_req count: %u",
					req_id, link->link_hdl, pd,
					dev->dev_info.name, link->open_req_cnt);
				snprintf(trace, sizeof(trace), "KMD %d_4 Skip Frame", link->link_hdl);
				trace_long(trace, req_id);
				trace_int(trace, 0);
				trace_begin_end("Skip Frame: req: %lld not ready on link: 0x%x for pd: %d dev: %d_%d_%s open_req count: %d",
					req_id, link->link_hdl, pd,
					dev->dev_info.dev_id, dev->dev_info.dev_hdl, dev->dev_info.name, link->open_req_cnt);
			}
#else
			if (link->wq_congestion)
				CAM_INFO_RATE_LIMIT(CAM_CRM,
					"WQ congestion, Skip Frame: req: %lld not ready on link: 0x%x for pd: %d dev: %s open_req count: %u",
					req_id, link->link_hdl, pd,
					dev->dev_info.name, link->open_req_cnt);
			else
				CAM_INFO(CAM_CRM,
					"Skip Frame: req: %lld not ready on link: 0x%x for pd: %d dev: %s open_req count: %u",
					req_id, link->link_hdl, pd,
					dev->dev_info.name, link->open_req_cnt);
#endif
		}
	}
}

static const char *__cam_req_mgr_operation_type_to_str(
	enum cam_req_mgr_req_state type)
{
	switch (type) {
	case CAM_CRM_REQ_READY:
		return "REQ_READY";
	case CAM_CRM_NOTIFY_TRIGGER:
		return "NOTIFY_TRIGGER";
	case CAM_CRM_PROCESS_TRIGGER:
		return "PROCESS_TRIGGER";
	case CAM_CRM_SEND_REQ:
		return "SEND_REQ";
	case CAM_CRM_NOTIFY_ERR:
		return "NOTIFY_ERR";
	case CAM_CRM_PROCESS_ERR:
		return "PROCESS_ERR";
	default:
		return "INVALID_TYPE";
	}
}

static void __cam_req_mgr_update_state_monitor_array(
	struct cam_req_mgr_core_link *link,
	struct cam_req_mgr_state_monitor *state)
{
	int idx = link->req.next_state_idx;
	char *dev_name = NULL;
	struct cam_req_mgr_state_monitor *state_monitor =
		&link->req.state_monitor[idx];

	dev_name = __cam_req_mgr_dev_handle_to_name(state->dev_hdl, link);

	spin_lock_bh(&link->req.monitor_slock);
	CAM_DBG(CAM_REQ,
		"Update: link_hdl %x dev %x dev_name %s req_id %lld frame_id %lld set to State: %s",
		link->link_hdl, state->dev_hdl,
		dev_name,
		state->req_id, state->frame_id,
		__cam_req_mgr_operation_type_to_str(state->req_state));

	state_monitor->req_state = state->req_state;
	state_monitor->req_id = state->req_id;
	state_monitor->dev_hdl = state->dev_hdl;
	state_monitor->frame_id = state->frame_id;
	scnprintf(state_monitor->name, sizeof(state_monitor->name), "%s", dev_name);
	ktime_get_clocktai_ts64(&state_monitor->time_stamp);

	__cam_req_mgr_inc_idx(&link->req.next_state_idx, 1, MAX_REQ_STATE_MONITOR_NUM);

	spin_unlock_bh(&link->req.monitor_slock);
}

/**
 * __cam_req_mgr_dump_state_monitor_array()
 *
 * @brief    : dump the state monitor array, dump from index next_wr_idx
 *             to print state information in time order.
 * @link     : link pointer
 *
 */
static void __cam_req_mgr_dump_state_monitor_array(
	struct cam_req_mgr_core_link *link)
{
	int i;
	int idx = link->req.next_state_idx;
	struct tm ts;
	struct timespec64 timespec;

	/*
	 * Get current time and print it immediately, wen need this log to cacluate the
	 * difference between log time and the time we get using ktime_get_clocktai_ts64,
	 * we need do like this because the time we get always has a 20-30ms difference from
	 * log time and do not have a good solution at present.
	 */
	ktime_get_clocktai_ts64(&timespec);
	time64_to_tm(timespec.tv_sec, 0, &ts);
	CAM_INFO(CAM_CRM, "Link %x start dump state monitor array, time: %d-%d %d:%d:%d.%lld",
		link->link_hdl, ts.tm_mon + 1, ts.tm_mday, ts.tm_hour,
		ts.tm_min, ts.tm_sec, timespec.tv_nsec / 1000000);

	CAM_INFO(CAM_CRM, "%16s  %6s  %10s  %8s  %10s",
		"state", "req id", "dev hdl", "frame id", "time stamp");
	spin_lock_bh(&link->req.monitor_slock);
	for (i = 0; i < MAX_REQ_STATE_MONITOR_NUM; i++) {
		if (link->req.state_monitor[idx].req_state != CAM_CRM_STATE_INVALID) {
			time64_to_tm(link->req.state_monitor[idx].time_stamp.tv_sec, 0, &ts);
			CAM_INFO(CAM_CRM, "%16s  %6d  %10x  %8d  %d-%d %d:%d:%d.%lld",
				__cam_req_mgr_operation_type_to_str(
				link->req.state_monitor[idx].req_state),
				link->req.state_monitor[idx].req_id,
				link->req.state_monitor[idx].dev_hdl,
				link->req.state_monitor[idx].frame_id,
				ts.tm_mon + 1, ts.tm_mday, ts.tm_hour,
				ts.tm_min, ts.tm_sec,
				link->req.state_monitor[idx].time_stamp.tv_nsec / 1000000);
		}

		__cam_req_mgr_inc_idx(&idx, 1, MAX_REQ_STATE_MONITOR_NUM);
	}
	spin_unlock_bh(&link->req.monitor_slock);
}

/**
 * __cam_req_mgr_notify_frame_skip()
 *
 * @brief : Notify all devices of frame skipping
 *
 */
static int __cam_req_mgr_notify_frame_skip(
	struct cam_req_mgr_core_link *link,
	uint32_t trigger)
{
	int                                  rc = 0, i, j, pd, idx;
	bool                                 found = false;
	struct cam_req_mgr_apply_request     frame_skip;
	struct cam_req_mgr_apply            *apply_data = NULL;
	struct cam_req_mgr_connected_device *dev = NULL;
	struct cam_req_mgr_tbl_slot         *slot = NULL;

	apply_data = link->req.prev_apply_data;

	if (link->max_delay < 0 || link->max_delay >= CAM_PIPELINE_DELAY_MAX) {
		CAM_ERR(CAM_CRM, "link->max_delay is out of bounds: %d",
			link->max_delay);
		return -EINVAL;
	}

	for (i = 0; i < link->num_devs; i++) {
		dev = &link->l_dev[i];
		pd = dev->dev_info.p_delay;
		if (pd >= CAM_PIPELINE_DELAY_MAX) {
			CAM_WARN(CAM_CRM, "pd %d greater than max",
				pd);
			continue;
		}

		found = false;
		idx = apply_data[pd].idx;
		slot = &dev->pd_tbl->slot[idx];

		for (j = 0; j < slot->ops.num_dev; j++) {
			if (dev->dev_hdl == slot->ops.dev_hdl[j]) {
				found = true;
				break;
			}
		}

		/*
		 * If apply_at_eof is enabled do not apply at SOF
		 * e.x. Flash device
		 */
		if ((trigger == CAM_TRIGGER_POINT_SOF) && found &&
			(slot->ops.apply_at_eof))
			continue;

		/*
		 * If apply_at_eof is not enabled ignore EOF
		 */
		if ((trigger == CAM_TRIGGER_POINT_EOF) && found &&
			(!slot->ops.apply_at_eof))
			continue;

		frame_skip.dev_hdl = dev->dev_hdl;
		frame_skip.link_hdl = link->link_hdl;
		frame_skip.request_id =
			apply_data[pd].req_id;
		frame_skip.trigger_point = trigger;
		frame_skip.report_if_bubble = 0;
		frame_skip.last_applied_max_pd_req =
			 link->req.prev_apply_data[link->max_delay].req_id;

		CAM_DBG(CAM_REQ,
			"Notify_frame_skip: link: 0x%x pd %d req_id %lld last_applied %lld",
			link->link_hdl, pd, apply_data[pd].req_id,
			link->req.prev_apply_data[link->max_delay].req_id);
		if ((dev->ops) && (dev->ops->notify_frame_skip))
			dev->ops->notify_frame_skip(&frame_skip);
	}

	return rc;
}

/**
 * __cam_req_mgr_send_evt()
 *
 * @brief      : Send event to all connected devices
 * @req_id     : Req ID
 * @type       : Event type
 * @error      : Error type
 * @link       : Link info
 *
 */
static int __cam_req_mgr_send_evt(
	uint64_t                       req_id,
	enum cam_req_mgr_link_evt_type type,
	enum cam_req_mgr_device_error  error,
	struct cam_req_mgr_core_link  *link)
{
	int i, rc = 0;
	struct cam_req_mgr_link_evt_data     evt_data = {0};
	struct cam_req_mgr_connected_device *device = NULL;

	CAM_DBG(CAM_CRM,
		"Notify event type: %d to all connected devices on link: 0x%x",
		type, link->link_hdl);

	/* Try for internal recovery */
	if (link->try_for_internal_recovery)
		evt_data.try_for_recovery = true;

	for (i = 0; i < link->num_devs; i++) {
		device = &link->l_dev[i];
		if (!device)
			continue;

		evt_data.dev_hdl = device->dev_hdl;
		evt_data.link_hdl = link->link_hdl;
		evt_data.req_id = req_id;
		evt_data.evt_type = type;
		if (type == CAM_REQ_MGR_LINK_EVT_UPDATE_PROPERTIES)
			evt_data.u.properties_mask = link->properties_mask;
		else
			evt_data.u.error = error;

		if (device->ops && device->ops->process_evt) {
			rc = device->ops->process_evt(&evt_data);
			if (rc) {
				CAM_ERR(CAM_CRM,
					"Failed to send evt %d on link 0x%x dev 0x%x req %lld",
					type, link->link_hdl, device->dev_hdl, req_id);
				break;
			}
		}
	}

	/* Updated if internal recovery succeeded */
	link->try_for_internal_recovery = evt_data.try_for_recovery;

	return rc;
}

/**
 * __cam_req_mgr_notify_error_on_link()
 *
 * @brief : Notify userspace on exceeding max retry
 *          attempts to apply same req
 * @link  : link on which the req could not be applied
 *
 */
static int __cam_req_mgr_notify_error_on_link(
	struct cam_req_mgr_core_link *link,
	struct cam_req_mgr_connected_device *dev)
{
	struct cam_req_mgr_core_session *session = NULL;
	struct cam_req_mgr_message       msg = {0};
	int rc = 0, pd;

	session = (struct cam_req_mgr_core_session *)link->parent;
	if (!session) {
		CAM_WARN(CAM_CRM, "session ptr NULL %x", link->link_hdl);
		return -EINVAL;
	}

	pd = dev->dev_info.p_delay;
	if (pd >= CAM_PIPELINE_DELAY_MAX) {
		CAM_ERR(CAM_CRM, "pd : %d is more than expected", pd);
		return -EINVAL;
	}

	if (link->min_delay >= CAM_PIPELINE_DELAY_MAX) {
		CAM_ERR(CAM_CRM, "min pd : %d is more than expected",
			link->min_delay);
		return -EINVAL;
	}

	/* Notify all devices in the link about the error */
	__cam_req_mgr_send_evt(link->req.apply_data[link->min_delay].req_id,
		CAM_REQ_MGR_LINK_EVT_STALLED, CRM_KMD_ERR_FATAL, link);

	/*
	 * Internal recovery succeeded - skip userland notification
	 * If recovery had failed subdevice will reset this flag
	 */
	if (link->try_for_internal_recovery) {
		CAM_INFO(CAM_CRM, "Internal recovery succeeded on link: 0x%x",
			link->link_hdl);
		return 0;
	}

	CAM_ERR_RATE_LIMIT(CAM_CRM,
		"Notifying userspace to trigger recovery on link 0x%x for session %d",
		link->link_hdl, session->session_hdl);

	__cam_req_mgr_dump_state_monitor_array(link);

	memset(&msg, 0, sizeof(msg));

	msg.session_hdl = session->session_hdl;
	msg.u.err_msg.error_type = CAM_REQ_MGR_ERROR_TYPE_RECOVERY;
	msg.u.err_msg.request_id =
		link->req.apply_data[pd].req_id;
	msg.u.err_msg.link_hdl   = link->link_hdl;
	msg.u.err_msg.resource_size = 0;
	msg.u.err_msg.error_code = CAM_REQ_MGR_LINK_STALLED_ERROR;

	CAM_DBG(CAM_CRM, "Failed for device: %s while applying request: %lld",
		dev->dev_info.name, link->req.apply_data[pd].req_id);

	rc = cam_req_mgr_notify_message(&msg,
		V4L_EVENT_CAM_REQ_MGR_ERROR,
		V4L_EVENT_CAM_REQ_MGR_EVENT);

	if (rc)
		CAM_ERR_RATE_LIMIT(CAM_CRM,
			"Error in notifying recovery for session %d link 0x%x rc %d",
			session->session_hdl, link->link_hdl, rc);

	return rc;
}

/**
 * __cam_req_mgr_traverse()
 *
 * @brief    : Traverse through pd tables, it will internally cover all linked
 *             pd tables. Each pd table visited will check if idx passed to its
 *             in ready state. If ready means all devices linked to the pd table
 *             have this request id packet ready. Then it calls subsequent pd
 *             tbl with new idx. New idx value takes into account the delta
 *             between current pd table and next one.
 * @traverse_data: contains all the info to traverse through pd tables
 *
 * @return: 0 for success, negative for failure
 *
 */
static int __cam_req_mgr_traverse(struct cam_req_mgr_traverse *traverse_data)
{
	int                          rc = 0;
	int32_t                      next_idx = traverse_data->idx;
	int32_t                      curr_idx = traverse_data->idx;
	struct cam_req_mgr_req_tbl  *tbl;
	struct cam_req_mgr_apply    *apply_data;
	struct cam_req_mgr_tbl_slot *slot = NULL;

	if (!traverse_data->tbl || !traverse_data->apply_data) {
		CAM_ERR(CAM_CRM, "NULL pointer %pK %pK",
			traverse_data->tbl, traverse_data->apply_data);
		traverse_data->result = 0;
		return -EINVAL;
	}

	tbl = traverse_data->tbl;
	apply_data = traverse_data->apply_data;
	slot = &tbl->slot[curr_idx];
	CAM_DBG(CAM_CRM,
		"Enter pd %d idx %d state %d skip %d status %d skip_idx %d",
		tbl->pd, curr_idx, tbl->slot[curr_idx].state,
		tbl->skip_traverse, traverse_data->in_q->slot[curr_idx].status,
		traverse_data->in_q->slot[curr_idx].skip_idx);

	/* Check if req is ready or in skip mode or pd tbl is in skip mode */
	if (tbl->slot[curr_idx].state == CRM_REQ_STATE_READY ||
		traverse_data->in_q->slot[curr_idx].skip_idx == 1 ||
		tbl->skip_traverse > 0) {
		if (tbl->next) {
			__cam_req_mgr_dec_idx(&next_idx, tbl->pd_delta,
				tbl->num_slots);
			traverse_data->idx = next_idx;
			traverse_data->tbl = tbl->next;
			rc = __cam_req_mgr_traverse(traverse_data);
		}
		if (rc >= 0) {
			SET_SUCCESS_BIT(traverse_data->result, tbl->pd);

			if (traverse_data->validate_only == false) {
				apply_data[tbl->pd].pd = tbl->pd;
				apply_data[tbl->pd].req_id =
					CRM_GET_REQ_ID(
					traverse_data->in_q, curr_idx);
				apply_data[tbl->pd].idx = curr_idx;

				CAM_DBG(CAM_CRM, "req_id: %lld with pd of %d",
				apply_data[tbl->pd].req_id,
				apply_data[tbl->pd].pd);
				/*
				 * If traverse is successful decrement
				 * traverse skip
				 */
				if (tbl->skip_traverse > 0) {
					apply_data[tbl->pd].req_id = -1;
					tbl->skip_traverse--;
				}
			}
		} else {
			/* linked pd table is not ready for this traverse yet */
			return rc;
		}
	} else {
		/* This pd table is not ready to proceed with asked idx */
		traverse_data->result_data.req_id =
			CRM_GET_REQ_ID(traverse_data->in_q, curr_idx);
		traverse_data->result_data.pd = tbl->pd;
		traverse_data->result_data.masked_value =
			(tbl->dev_mask & slot->req_ready_map);
		SET_FAILURE_BIT(traverse_data->result, tbl->pd);
		return -EAGAIN;
	}

	return 0;
}

/**
 * __cam_req_mgr_in_q_skip_idx()
 *
 * @brief    : Decrement val passed by step size and rollover after max_val
 * @in_q     : input queue pointer
 * @idx      : Sets skip_idx bit of the particular slot to true so when traverse
 *             happens for this idx, no req will be submitted for devices
 *             handling this idx.
 *
 */
static void __cam_req_mgr_in_q_skip_idx(struct cam_req_mgr_req_queue *in_q,
	int32_t idx)
{
	in_q->slot[idx].req_id = -1;
	in_q->slot[idx].skip_idx = 1;
	CAM_DBG(CAM_CRM, "SET IDX SKIP on slot= %d", idx);
}

/**
 * __cam_req_mgr_tbl_set_id()
 *
 * @brief    : Set unique id to table
 * @tbl      : pipeline based table which requires new id
 * @req      : pointer to request data wihch contains num_tables counter
 *
 */
static void __cam_req_mgr_tbl_set_id(struct cam_req_mgr_req_tbl *tbl,
	struct cam_req_mgr_req_data *req)
{
	if (!tbl)
		return;
	do {
		tbl->id = req->num_tbl++;
		CAM_DBG(CAM_CRM, "%d: pd %d skip_traverse %d delta %d",
			tbl->id, tbl->pd, tbl->skip_traverse,
			tbl->pd_delta);
		tbl = tbl->next;
	} while (tbl != NULL);
}

/**
 * __cam_req_mgr_tbl_set_all_skip_cnt()
 *
 * @brief    : Each pd table sets skip value based on delta between itself and
 *             max pd value. During initial streamon or bubble case this is
 *             used. That way each pd table skips required num of traverse and
 *             align themselve with req mgr connected devs.
 * @l_tbl    : iterates through list of pd tables and sets skip traverse
 *
 */
static void __cam_req_mgr_tbl_set_all_skip_cnt(
	struct cam_req_mgr_req_tbl **l_tbl)
{
	struct cam_req_mgr_req_tbl *tbl = *l_tbl;
	int32_t                     max_pd;

	if (!tbl)
		return;

	max_pd = tbl->pd;
	do {
		tbl->skip_traverse = max_pd - tbl->pd;
		CAM_DBG(CAM_CRM, "%d: pd %d skip_traverse %d delta %d",
			tbl->id, tbl->pd, tbl->skip_traverse,
			tbl->pd_delta);
		tbl = tbl->next;
	} while (tbl != NULL);
}

/**
 * __cam_req_mgr_find_slot_for_req()
 *
 * @brief    : Find idx from input queue at which req id is enqueued
 * @in_q     : input request queue pointer
 * @req_id   : request id which needs to be searched in input queue
 *
 * @return   : slot index where passed request id is stored, -1 for failure
 *
 */
static int32_t __cam_req_mgr_find_slot_for_req(
	struct cam_req_mgr_req_queue *in_q, int64_t req_id)
{
	int32_t                   idx, i;
	struct cam_req_mgr_slot  *slot;

	idx = in_q->rd_idx;
	for (i = 0; i < in_q->num_slots; i++) {
		slot = &in_q->slot[idx];
		if (slot->req_id == req_id) {
			CAM_DBG(CAM_CRM,
				"req: %lld found at idx: %d status: %d sync_mode: %d",
				req_id, idx, slot->status, slot->sync_mode);
			break;
		}
		__cam_req_mgr_dec_idx(&idx, 1, in_q->num_slots);
	}
	if (i >= in_q->num_slots)
		idx = -1;

	return idx;
}

/**
 * __cam_req_mgr_disconnect_req_on_sync_link()
 *
 * @brief    : Disconnect link and sync link
 * @link     : pointer to link
 * @slot     : poniter to slot
 *
 */
static void __cam_req_mgr_disconnect_req_on_sync_link(
	struct cam_req_mgr_core_link *link,
	struct cam_req_mgr_slot      *slot)
{
	int                           i, j;
	int                           sync_idx;
	struct cam_req_mgr_slot      *sync_slot;
	struct cam_req_mgr_req_queue *sync_in_q;
	struct cam_req_mgr_core_link *sync_link;

	for (i = 0; i < slot->num_sync_links; i++) {
		sync_link = cam_get_link_priv(slot->sync_link_hdls[i]);
		if (!sync_link)
			continue;

		sync_in_q = sync_link->req.in_q;
		sync_idx = __cam_req_mgr_find_slot_for_req(sync_in_q, slot->req_id);
		if (sync_idx < 0)
			continue;

		CAM_DBG(CAM_CRM,
			"Req: %llu on link: 0x%x flushed, update sync_link: 0x%x at slot: %d",
			slot->req_id, link->link_hdl, sync_link->link_hdl, sync_idx);

		sync_slot = &sync_in_q->slot[sync_idx];
		if (sync_slot->sync_mode == CAM_REQ_MGR_SYNC_MODE_SYNC) {
			sync_slot->sync_mode = CAM_REQ_MGR_SYNC_MODE_NO_SYNC;
			sync_slot->num_sync_links = 0;

			for (j = 0; j < MAXIMUM_LINKS_PER_SESSION - 1; j++) {
				if (sync_slot->sync_link_hdls[j] == link->link_hdl)
					sync_slot->sync_link_hdls[j] = 0;
			}
		}
	}
}

/**
 * __cam_req_mgr_flush_req_slot()
 *
 * @brief    : reset all the slots/pd tables when flush is
 *             invoked
 * @link     : link pointer
 *
 */
static void __cam_req_mgr_flush_req_slot(
	struct cam_req_mgr_core_link *link)
{
	int                           i;
	int                           idx;
	struct cam_req_mgr_slot      *slot;
	struct cam_req_mgr_req_tbl   *tbl;
	struct cam_req_mgr_req_queue *in_q = link->req.in_q;

	for (idx = 0; idx < in_q->num_slots; idx++) {
		slot = &in_q->slot[idx];
		tbl = link->req.l_tbl;
		CAM_DBG(CAM_CRM,
			"RESET idx: %d req_id: %lld slot->status: %d",
			idx, slot->req_id, slot->status);

		if ((slot->req_id > 0) && slot->num_sync_links)
			__cam_req_mgr_disconnect_req_on_sync_link(link, slot);

		/* Reset input queue slot */
		slot->req_id = -1;
		slot->bubble_times = 0;
		slot->internal_recovered = false;
		slot->skip_idx = 1;
		slot->recover = 0;
		slot->additional_timeout = 0;
		slot->sync_mode = CAM_REQ_MGR_SYNC_MODE_NO_SYNC;
		slot->status = CRM_SLOT_STATUS_NO_REQ;
		slot->num_sync_links = 0;
		for (i = 0; i < MAXIMUM_LINKS_PER_SESSION - 1; i++)
			slot->sync_link_hdls[i] = 0;

		/* Reset all pd table slot */
		while (tbl != NULL) {
			CAM_DBG(CAM_CRM, "pd: %d: idx %d state %d",
				tbl->pd, idx, tbl->slot[idx].state);
			tbl->slot[idx].req_ready_map = 0;
			tbl->slot[idx].req_apply_map = 0;
			tbl->slot[idx].state = CRM_REQ_STATE_EMPTY;
			tbl->slot[idx].ops.apply_at_eof = false;
			for (i = 0; i < MAX_DEV_FOR_SPECIAL_OPS; i++)
				tbl->slot[idx].ops.dev_hdl[i] = -1;
			tbl->slot[idx].ops.num_dev = 0;
			tbl = tbl->next;
		}
	}

	atomic_set(&link->eof_event_cnt, 0);
	in_q->wr_idx = 0;
	in_q->rd_idx = 0;

	link->trigger_cnt[0][CAM_TRIGGER_POINT_SOF] = 0;
	link->trigger_cnt[0][CAM_TRIGGER_POINT_EOF] = 0;
	link->trigger_cnt[1][CAM_TRIGGER_POINT_SOF] = 0;
	link->trigger_cnt[1][CAM_TRIGGER_POINT_EOF] = 0;
	link->cont_empty_slots = 0;
}

/**
 * __cam_req_mgr_reset_req_slot()
 *
 * @brief    : reset specified idx/slot in input queue as well as all pd tables
 * @link     : link pointer
 * @idx      : slot index which will be reset
 *
 */
static void __cam_req_mgr_reset_req_slot(struct cam_req_mgr_core_link *link,
	int32_t idx)
{
	int                           i;
	struct cam_req_mgr_slot      *slot;
	struct cam_req_mgr_req_tbl   *tbl = link->req.l_tbl;
	struct cam_req_mgr_req_queue *in_q = link->req.in_q;

	slot = &in_q->slot[idx];
	CAM_DBG(CAM_CRM, "RESET: idx: %d: slot->status %d", idx, slot->status);

	/* Check if CSL has already pushed new request*/
	if (slot->status == CRM_SLOT_STATUS_REQ_ADDED ||
		in_q->last_applied_idx == idx ||
		idx < 0)
		return;

	if ((slot->req_id > 0) && slot->num_sync_links)
		__cam_req_mgr_disconnect_req_on_sync_link(link, slot);

	/* Reset input queue slot */
	slot->req_id = -1;
	slot->bubble_times = 0;
	slot->internal_recovered = false;
	slot->skip_idx = 0;
	slot->recover = 0;
	slot->additional_timeout = 0;
	slot->recovery_counter = 0;
	slot->sync_mode = CAM_REQ_MGR_SYNC_MODE_NO_SYNC;
	slot->status = CRM_SLOT_STATUS_NO_REQ;
	slot->num_sync_links = 0;
	slot->frame_sync_shift = 0;
	for (i = 0; i < MAXIMUM_LINKS_PER_SESSION - 1; i++)
		slot->sync_link_hdls[i] = 0;

	/* Reset all pd table slot */
	while (tbl != NULL) {
		CAM_DBG(CAM_CRM, "pd: %d: idx %d state %d",
			tbl->pd, idx, tbl->slot[idx].state);
		tbl->slot[idx].req_ready_map = 0;
		tbl->slot[idx].req_apply_map = 0;
		tbl->slot[idx].state = CRM_REQ_STATE_EMPTY;
		tbl->slot[idx].inject_delay_at_sof = 0;
		tbl->slot[idx].inject_delay_at_eof = 0;
		tbl->slot[idx].ops.apply_at_eof = false;
		for (i = 0; i < MAX_DEV_FOR_SPECIAL_OPS; i++)
			tbl->slot[idx].ops.dev_hdl[i] = -1;
		tbl->slot[idx].ops.num_dev = 0;
		tbl = tbl->next;
	}
}

/**
 * __cam_req_mgr_validate_crm_wd_timer()
 *
 * @brief    : Validate/modify the wd timer based on associated
 *             timeout with the request
 * @link     : link pointer
 *
 */
static void __cam_req_mgr_validate_crm_wd_timer(
	struct cam_req_mgr_core_link *link)
{
	int idx = 0;
	int next_frame_timeout, current_frame_timeout, max_frame_timeout;
	int64_t current_req_id, next_req_id;
	struct cam_req_mgr_req_queue *in_q = link->req.in_q;

	if (link->skip_init_frame) {
		CAM_DBG(CAM_CRM,
			"skipping modifying wd timer for first frame after streamon");
		link->skip_init_frame = false;
		return;
	}

	idx = in_q->rd_idx;
	__cam_req_mgr_dec_idx(
		&idx, (link->max_delay - 1),
		in_q->num_slots);
	next_frame_timeout = in_q->slot[idx].additional_timeout;
	next_req_id = in_q->slot[idx].req_id;
	CAM_DBG(CAM_CRM,
		"rd_idx: %d idx: %d next_req_id: %lld next_frame_timeout: %d ms",
		in_q->rd_idx, idx, next_req_id, next_frame_timeout);

	idx = in_q->rd_idx;
	__cam_req_mgr_dec_idx(
		&idx, link->max_delay,
		in_q->num_slots);
	current_frame_timeout = in_q->slot[idx].additional_timeout;
	current_req_id = in_q->slot[idx].req_id;
	CAM_DBG(CAM_CRM,
		"rd_idx: %d idx: %d curr_req_id: %lld current_frame_timeout: %d ms",
		in_q->rd_idx, idx, current_req_id, current_frame_timeout);

	if ((current_req_id == -1) && (next_req_id == -1)) {
		CAM_DBG(CAM_CRM,
			"Skip modifying wd timer, continue with same timeout");
		return;
	}

	max_frame_timeout = (current_frame_timeout > next_frame_timeout) ?
		current_frame_timeout : next_frame_timeout;

	spin_lock_bh(&link->link_state_spin_lock);
	if (link->watchdog) {
		if ((max_frame_timeout + CAM_REQ_MGR_WATCHDOG_TIMEOUT) >
			link->watchdog->expires) {
			CAM_DBG(CAM_CRM,
				"Modifying wd timer expiry from %d ms to %d ms",
				link->watchdog->expires,
				(next_frame_timeout +
				 CAM_REQ_MGR_WATCHDOG_TIMEOUT));
			crm_timer_modify(link->watchdog,
				max_frame_timeout +
				CAM_REQ_MGR_WATCHDOG_TIMEOUT);
		}  else if (max_frame_timeout) {
			max_frame_timeout = CAM_REQ_MGR_WATCHDOG_TIMEOUT +
				CAM_REQ_MGR_COMPUTE_TIMEOUT(max_frame_timeout);
			CAM_DBG(CAM_CRM,
				"Reset wd timer to frame from %d ms to %d ms",
				link->watchdog->expires,
				max_frame_timeout);
			crm_timer_modify(link->watchdog,
				max_frame_timeout);
		} else if (!max_frame_timeout && (link->watchdog->expires >
			CAM_REQ_MGR_WATCHDOG_TIMEOUT)) {
			CAM_DBG(CAM_CRM,
				"Reset wd timer to default from %d ms to %d ms",
				link->watchdog->expires,
				CAM_REQ_MGR_WATCHDOG_TIMEOUT);
			crm_timer_modify(link->watchdog,
				CAM_REQ_MGR_WATCHDOG_TIMEOUT);
		}
	} else {
		CAM_WARN(CAM_CRM, "Watchdog timer exited already");
	}
	spin_unlock_bh(&link->link_state_spin_lock);
}

/**
 * __cam_req_mgr_check_for_lower_pd_devices()
 *
 * @brief    : Checks if there are any devices on the link having a lesser
 *             pd than the max pd of the link
 * @link     : Pointer to link which needs to be checked
 *
 * @return   : 0 if a lower pd device is found negative otherwise
 */
static int __cam_req_mgr_check_for_lower_pd_devices(
	struct cam_req_mgr_core_link	*link)
{
	int i = 0;
	struct cam_req_mgr_connected_device *dev = NULL;

	for (i = 0; i < link->num_devs; i++) {
		dev = &link->l_dev[i];
		if (dev->dev_info.p_delay < link->max_delay)
			return 0;
	}

	return -EAGAIN;
}

/**
 * __cam_req_mgr_move_to_next_req_slot()
 *
 * @brief    : While streaming if input queue does not contain any pending
 *             request, req mgr still needs to submit pending request ids to
 *             devices with lower pipeline delay value. But if there are
 *             continuous max_delay empty slots, we don't need to move to
 *             next slot since the last request is applied to all devices.
 * @link     : Link handle
 *
 * @return   : 0 for success, negative for failure
 */
static int __cam_req_mgr_move_to_next_req_slot(
	struct cam_req_mgr_core_link *link)
{
	int rc = 0;
	struct cam_req_mgr_req_queue *in_q = link->req.in_q;
	int32_t idx = in_q->rd_idx;
	struct cam_req_mgr_slot *slot;

	__cam_req_mgr_inc_idx(&idx, 1, in_q->num_slots);
	slot = &in_q->slot[idx];

	CAM_DBG(CAM_CRM, "idx: %d: slot->status %d", idx, slot->status);

	/*
	 * Some slot can't be reset due to irq congestion and
	 * and performance issue, we need to reset it when we
	 * want to move to this slot.
	 */
	if (slot->status == CRM_SLOT_STATUS_REQ_APPLIED) {
		CAM_WARN(CAM_CRM,
			"slot [idx: %d req: %lld last_applied_idx: %d] was not reset, reset it now",
			idx, in_q->slot[idx].req_id, in_q->last_applied_idx);
		if (in_q->last_applied_idx == idx) {
			CAM_WARN(CAM_CRM,
				"last_applied_idx: %d",
				in_q->last_applied_idx);
			in_q->last_applied_idx = -1;
		}
		__cam_req_mgr_reset_req_slot(link, idx);
	}

	/* Check if there is new req from CSL, if not complete req */
	if (slot->status == CRM_SLOT_STATUS_NO_REQ) {
		rc = __cam_req_mgr_check_for_lower_pd_devices(link);
		if (rc) {
			CAM_DBG(CAM_CRM, "No lower pd devices on link 0x%x",
				link->link_hdl);
			return rc;
		}

		if (link->cont_empty_slots++ >= link->max_delay) {
			CAM_DBG(CAM_CRM, "There are %d continuous empty slots on link 0x%x",
				link->cont_empty_slots, link->link_hdl);
			return -EAGAIN;
		}

		__cam_req_mgr_in_q_skip_idx(in_q, idx);
		slot->status = CRM_SLOT_STATUS_REQ_ADDED;
		if (in_q->wr_idx != idx)
			CAM_WARN(CAM_CRM,
				"CHECK here wr %d, rd %d", in_q->wr_idx, idx);
		else
			__cam_req_mgr_inc_idx(&in_q->wr_idx, 1, in_q->num_slots);
	} else
		link->cont_empty_slots = 0;

	__cam_req_mgr_inc_idx(&in_q->rd_idx, 1, in_q->num_slots);

	return rc;
}

/**
 * __cam_req_mgr_send_req()
 *
 * @brief              : send request id to be applied to each device connected on link
 * @link               : pointer to link whose input queue and req tbl are
 *                       traversed through
 * @in_q               : pointer to input request queue
 *
 * @return   : 0 for success, negative for failure
 *
 */
static int __cam_req_mgr_send_req(struct cam_req_mgr_core_link *link,
	struct cam_req_mgr_req_queue *in_q, uint32_t trigger,
	struct cam_req_mgr_connected_device **failed_dev)
{
	int                                  rc = 0, pd, i, j, idx;
	bool                                 found = false;
	int64_t                              req_applied_to_min_pd = -1;
	struct cam_req_mgr_connected_device *dev = NULL;
	struct cam_req_mgr_apply_request     apply_req;
	struct cam_req_mgr_link_evt_data     evt_data;
	struct cam_req_mgr_tbl_slot          *slot = NULL;
	struct cam_req_mgr_slot              *req_slot = NULL;
	struct cam_req_mgr_apply             *apply_data = NULL;
	struct cam_req_mgr_state_monitor     state;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	char trace[64] = {0};
#endif

	apply_req.link_hdl = link->link_hdl;
	apply_req.report_if_bubble = 0;
	apply_req.re_apply = false;
	if (link->retry_cnt > 0) {
		if (g_crm_core_dev->recovery_on_apply_fail)
			apply_req.re_apply = true;
	}

	if (link->state == CAM_CRM_LINK_STATE_ERR)
		apply_req.recovery = true;
	else
		apply_req.recovery = false;

	if (link->max_delay < 0 || link->max_delay >= CAM_PIPELINE_DELAY_MAX) {
		CAM_ERR(CAM_CRM, "link->max_delay is out of bounds: %d",
			link->max_delay);
		return -EINVAL;
	}

	apply_data = link->req.apply_data;
	req_slot = &in_q->slot[in_q->rd_idx];

	/*
	 * This For loop is to address the special operation requested
	 * by device
	 */
	for (i = 0; i < link->num_devs; i++) {
		dev = &link->l_dev[i];
		pd = dev->dev_info.p_delay;
		if (pd >= CAM_PIPELINE_DELAY_MAX) {
			CAM_WARN(CAM_CRM, "pd %d greater than max",
				pd);
			continue;
		}

		found = false;
		idx = apply_data[pd].idx;
		slot = &dev->pd_tbl->slot[idx];

		if (!slot->ops.num_dev || (slot->ops.num_dev > MAX_DEV_FOR_SPECIAL_OPS)) {
			CAM_DBG(CAM_CRM,
				"No special ops detected for slot %d dev %s num_dev %d",
				idx, dev->dev_info.name, slot->ops.num_dev);
			continue;
		}

		for (j = 0; j < slot->ops.num_dev; j++) {
			if (dev->dev_hdl == slot->ops.dev_hdl[j]) {
				found = true;
				break;
			}
		}

		if (!found) {
			CAM_DBG(CAM_CRM,
				"Dev_hdl : %d Not matched", dev->dev_hdl);
			continue;
		}

		if (slot->req_apply_map & BIT(dev->dev_bit))
			continue;

		if (apply_data[pd].skip_idx ||
			(apply_data[pd].req_id < 0)) {
			CAM_DBG(CAM_CRM,
				"dev %s skip %d req_id %lld",
				dev->dev_info.name,
				apply_data[pd].skip_idx,
				apply_data[pd].req_id);
			apply_req.dev_hdl = dev->dev_hdl;
			apply_req.request_id =
				link->req.prev_apply_data[pd].req_id;
			apply_req.trigger_point = trigger;
			apply_req.report_if_bubble = 0;
			apply_req.last_applied_max_pd_req =
				link->req.prev_apply_data[link->max_delay].req_id;
			if ((dev->ops) && (dev->ops->notify_frame_skip))
				dev->ops->notify_frame_skip(&apply_req);
			continue;
		}

		/* This one is to prevent EOF request to apply on SOF*/
		if ((trigger == CAM_TRIGGER_POINT_SOF) &&
			(slot->ops.apply_at_eof)) {
			CAM_DBG(CAM_CRM, "EOF event cannot be applied at SOF");
			break;
		}

		if ((trigger == CAM_TRIGGER_POINT_EOF) &&
			(!slot->ops.apply_at_eof)) {
			CAM_DBG(CAM_CRM, "NO EOF DATA FOR REQ: %llu",
				apply_data[pd].req_id);
			break;
		}

		apply_req.dev_hdl = dev->dev_hdl;
		apply_req.request_id =
			apply_data[pd].req_id;
		apply_req.trigger_point = trigger;
		if ((dev->ops) && (dev->ops->apply_req)) {
			rc = dev->ops->apply_req(&apply_req);
			if (rc) {
				*failed_dev = dev;
				__cam_req_mgr_notify_frame_skip(link,
					trigger);
				return rc;
			} else
				slot->req_apply_map |= BIT(dev->dev_bit);
		} else {
			CAM_DBG(CAM_REQ,
				"link_hdl: %x pd: %d req_id %lld ops %p apply_req %p",
				link->link_hdl, pd, apply_req.request_id,
				dev->ops, (dev->ops ? dev->ops->apply_req : NULL));
			break;
		}

		state.req_state = CAM_CRM_SEND_REQ;
		state.req_id = apply_req.request_id;
		state.dev_hdl = apply_req.dev_hdl;
		state.frame_id = -1;
		__cam_req_mgr_update_state_monitor_array(link, &state);

		CAM_DBG(CAM_REQ,
			"SEND: link_hdl %x dev %s pd %d req_id %lld",
			link->link_hdl, dev->dev_info.name,
			pd, apply_req.request_id);

		if ((trigger == CAM_TRIGGER_POINT_EOF) &&
			(slot->ops.apply_at_eof)) {
			slot->ops.apply_at_eof = false;
			if (atomic_read(&link->eof_event_cnt) > 0)
				atomic_dec(&link->eof_event_cnt);
			return 0;
		}
	}

	/* For regular send requests */
	for (i = 0; i < link->num_devs; i++) {
		dev = &link->l_dev[i];
		if (dev) {
			pd = dev->dev_info.p_delay;
			if (pd >= CAM_PIPELINE_DELAY_MAX) {
				CAM_WARN(CAM_CRM, "pd %d greater than max",
					pd);
				continue;
			}

			if (!(dev->dev_info.trigger & trigger))
				continue;

			if (apply_data[pd].skip_idx ||
				(apply_data[pd].req_id < 0)) {
				CAM_DBG(CAM_CRM,
					"dev %s skip %d req_id %lld",
					dev->dev_info.name,
					apply_data[pd].skip_idx,
					apply_data[pd].req_id);
				apply_req.dev_hdl = dev->dev_hdl;
				apply_req.request_id =
					link->req.prev_apply_data[pd].req_id;
				apply_req.trigger_point = trigger;
				apply_req.report_if_bubble = 0;
				apply_req.last_applied_max_pd_req =
					link->req.prev_apply_data[link->max_delay].req_id;
				apply_req.no_further_requests =
					(req_slot->req_id == -1) ? true : false;
				if ((dev->ops) && (dev->ops->notify_frame_skip))
					dev->ops->notify_frame_skip(&apply_req);
				continue;
			}

			found = false;

			apply_req.dev_hdl = dev->dev_hdl;
			apply_req.request_id =
				apply_data[pd].req_id;
			idx = apply_data[pd].idx;
			slot = &dev->pd_tbl->slot[idx];
			apply_req.report_if_bubble =
				in_q->slot[idx].recover;

			/*
			 * If it is dual trigger usecase, need to tell
			 * devices that the req is re-applied, then the
			 * devices need to skip applying if the req has
			 * been handled.
			 * e.x. ISP device
			 */
			if (link->retry_cnt > 0) {
				if (!apply_req.report_if_bubble &&
					link->dual_trigger)
					apply_req.re_apply = true;
			}

			if (slot->req_apply_map & BIT(dev->dev_bit))
				continue;

			for (j = 0; j < slot->ops.num_dev; j++) {
				if (dev->dev_hdl == slot->ops.dev_hdl[j]) {
					found = true;
					break;
				}
			}

			/*
			 * If apply_at_eof is enabled do not apply at SOF
			 * e.x. Flash device
			 */
			if ((trigger == CAM_TRIGGER_POINT_SOF) && found &&
				(slot->ops.apply_at_eof))
				continue;

			if ((trigger == CAM_TRIGGER_POINT_EOF) && found &&
				(!slot->ops.apply_at_eof))
				continue;

			apply_req.trigger_point = trigger;
			apply_req.last_applied_done_timestamp = 0;
			if ((dev->dev_info.dev_id == CAM_REQ_MGR_DEVICE_IFE) &&
				(in_q->slot[idx].mismatched_frame_mode == CRM_NOTIFY_MISMATCHED_FRMAE))
				apply_req.last_applied_done_timestamp = link->last_applied_done_timestamp;

			CAM_DBG(CAM_REQ,
				"SEND: link_hdl %x dev %s pd %d req_id %lld",
				link->link_hdl, dev->dev_info.name,
				pd, apply_req.request_id);
			if (dev->ops && dev->ops->apply_req) {
				rc = dev->ops->apply_req(&apply_req);
				if (rc < 0) {
					*failed_dev = dev;
					break;
				} else
					slot->req_apply_map |= BIT(dev->dev_bit);
			}

			if (dev->dev_info.dev_id == CAM_REQ_MGR_DEVICE_SENSOR) {
				link->last_applied_done_timestamp = apply_req.last_applied_done_timestamp;
				CAM_DBG(CAM_REQ,
					"Apply req:%lld done with last_applied_done_timestamp:0x%llx",
					apply_req.request_id, link->last_applied_done_timestamp);
			}

			state.req_state = CAM_CRM_SEND_REQ;
			state.req_id = apply_req.request_id;
			state.dev_hdl = apply_req.dev_hdl;
			state.frame_id = -1;
			__cam_req_mgr_update_state_monitor_array(link, &state);

			if (pd == link->min_delay)
				req_applied_to_min_pd = apply_req.request_id;

			trace_cam_req_mgr_apply_request(link, &apply_req, dev);
#ifdef OPLUS_FEATURE_CAMERA_COMMON
			snprintf(trace, sizeof(trace), "KMD %d_5 ApplyRequest %d_%s", apply_req.link_hdl, dev->dev_info.dev_id, dev->dev_info.name);
			trace_long(trace, apply_req.request_id);
			trace_int(trace, 0);
#endif
		}
	}

	if (rc < 0) {
		CAM_WARN_RATE_LIMIT(CAM_CRM, "APPLY FAILED pd %d req_id %lld",
			dev->dev_info.p_delay, apply_req.request_id);
		/* Apply req failed notify already applied devs */
		for (; i >= 0; i--) {
			dev = &link->l_dev[i];
			evt_data.evt_type = CAM_REQ_MGR_LINK_EVT_ERR;
			evt_data.dev_hdl = dev->dev_hdl;
			evt_data.link_hdl =  link->link_hdl;
			evt_data.req_id = apply_req.request_id;
			evt_data.u.error = CRM_KMD_ERR_BUBBLE;
			if (dev->ops && dev->ops->process_evt)
				dev->ops->process_evt(&evt_data);
		}
		__cam_req_mgr_notify_frame_skip(link, trigger);
	} else {
		memcpy(link->req.prev_apply_data, link->req.apply_data,
			CAM_PIPELINE_DELAY_MAX *
			sizeof(struct cam_req_mgr_apply));
		if (req_applied_to_min_pd > 0) {
			link->open_req_cnt--;
			CAM_DBG(CAM_REQ,
				"Open_reqs: %u after successfully applying req:%d",
				link->open_req_cnt, req_applied_to_min_pd);
		}
	}

	return rc;
}

/**
 * __cam_req_mgr_check_link_is_ready()
 *
 * @brief    : traverse through all request tables and see if all devices are
 *             ready to apply request settings.
 * @link     : pointer to link whose input queue and req tbl are
 *             traversed through
 * @idx      : index within input request queue
 * @validate_only : Whether to validate only and/or update settings
 *
 * @return   : 0 for success, negative for failure
 *
 */
static int __cam_req_mgr_check_link_is_ready(struct cam_req_mgr_core_link *link,
	int32_t idx, bool validate_only)
{
	int                            rc;
	struct cam_req_mgr_traverse    traverse_data;
	struct cam_req_mgr_req_queue  *in_q;
	struct cam_req_mgr_apply      *apply_data;

	in_q = link->req.in_q;

	apply_data = link->req.apply_data;

	if (validate_only == false) {
		memset(apply_data, 0,
		    sizeof(struct cam_req_mgr_apply) * CAM_PIPELINE_DELAY_MAX);
	}

	traverse_data.apply_data = apply_data;
	traverse_data.idx = idx;
	traverse_data.tbl = link->req.l_tbl;
	traverse_data.in_q = in_q;
	traverse_data.result = 0;
	traverse_data.result_data.masked_value = 0;
	traverse_data.result_data.pd = 0;
	traverse_data.result_data.req_id = 0;
	traverse_data.validate_only = validate_only;
	traverse_data.open_req_cnt = link->open_req_cnt;

	/*
	 * Some no-sync mode requests are processed after link config,
	 * then process the sync mode requests after no-sync mode requests
	 * are handled, the initial_skip should be false when processing
	 * the sync mode requests.
	 */
	if (link->initial_skip) {
		CAM_DBG(CAM_CRM,
			"Set initial_skip to false for link %x",
			link->link_hdl);
		link->initial_skip = false;
	}

	/*
	 *  Traverse through all pd tables, if result is success,
	 *  apply the settings
	 */
	rc = __cam_req_mgr_traverse(&traverse_data);
	CAM_DBG(CAM_CRM,
		"SOF: idx %d result %x pd_mask %x rc %d",
		idx, traverse_data.result, link->pd_mask, rc);

	if (!rc && traverse_data.result == link->pd_mask) {
		CAM_DBG(CAM_CRM,
			"READY: link_hdl= %x idx= %d, req_id= %lld :%lld :%lld",
			link->link_hdl, idx,
			apply_data[2].req_id,
			apply_data[1].req_id,
			apply_data[0].req_id);
	} else {
		rc = -EAGAIN;
		__cam_req_mgr_find_dev_name(link,
			traverse_data.result_data.req_id,
			traverse_data.result_data.pd,
			traverse_data.result_data.masked_value);
	}

	return rc;
}

/**
 * __cam_req_mgr_check_sync_for_mslave()
 *
 * @brief    : Processes requests during sync mode [master-slave]
 *             Here master corresponds to the link having a higher
 *             max_delay (pd) compared to the slave link.
 * @link     : Pointer to link whose input queue and req tbl are
 *             traversed through
 * @slot     : Pointer to the current slot being processed
 * @return   : 0 for success, negative for failure
 *
 */
static int __cam_req_mgr_check_sync_for_mslave(
	struct cam_req_mgr_core_link *link,
	struct cam_req_mgr_core_link *sync_link,
	struct cam_req_mgr_slot *slot)
{
	struct cam_req_mgr_slot      *sync_slot = NULL;
	int sync_slot_idx = 0, prev_idx, next_idx, rd_idx, sync_rd_idx, rc = 0;
	int64_t req_id = 0, sync_req_id = 0;
	int32_t sync_num_slots = 0;

	if (!sync_link  || !link) {
		CAM_ERR(CAM_CRM, "Sync link or link is null");
		return -EINVAL;
	}

	req_id = slot->req_id;

	if (!sync_link->req.in_q) {
		CAM_ERR(CAM_CRM, "Link hdl %x in_q is NULL",
			sync_link->link_hdl);
		return -EINVAL;
	}

	sync_num_slots = sync_link->req.in_q->num_slots;
	sync_rd_idx = sync_link->req.in_q->rd_idx;

	CAM_DBG(CAM_CRM,
		"link_hdl %x req %lld frame_skip_flag %d open_req_cnt:%u initial_sync_req [%lld,%lld] is_master:%d",
		link->link_hdl, req_id, link->sync_link_sof_skip,
		link->open_req_cnt, link->initial_sync_req,
		sync_link->initial_sync_req, link->is_master);

	if (sync_link->sync_link_sof_skip) {
		CAM_DBG(CAM_CRM,
			"No req applied on corresponding SOF on sync link: %x",
				sync_link->link_hdl);
		sync_link->sync_link_sof_skip = false;
		return -EAGAIN;
	}

	if (link->in_msync_mode &&
		sync_link->in_msync_mode &&
		(req_id - sync_link->req.in_q->slot[sync_rd_idx].req_id >
		link->max_delay - sync_link->max_delay)) {
		CAM_DBG(CAM_CRM,
			"Req: %lld on link:%x need to hold for link: %x req:%d",
			req_id,
			link->link_hdl,
			sync_link->link_hdl,
			sync_link->req.in_q->slot[sync_rd_idx].req_id);
		return -EINVAL;
	}

	if (link->is_master) {
		if (sync_link->initial_skip) {
			CAM_DBG(CAM_CRM,  "Link 0x%x [slave] not streamed on",
				sync_link->link_hdl);
			return -EAGAIN;
		}

		rc = __cam_req_mgr_check_link_is_ready(link, slot->idx, true);
		if (rc) {
			CAM_DBG(CAM_CRM,
				"Req: %lld [master] not ready on link: %x, rc=%d",
				req_id, link->link_hdl, rc);
			link->sync_link_sof_skip = true;
			return rc;
		}

		prev_idx = slot->idx;
		__cam_req_mgr_dec_idx(&prev_idx,
			(link->max_delay - sync_link->max_delay),
			link->req.in_q->num_slots);

		rd_idx = sync_link->req.in_q->rd_idx;
		sync_req_id = link->req.in_q->slot[prev_idx].req_id;
		if ((sync_link->initial_sync_req != -1) &&
			(sync_link->initial_sync_req <= sync_req_id)) {
			sync_slot_idx = __cam_req_mgr_find_slot_for_req(
				sync_link->req.in_q, sync_req_id);
			if (sync_slot_idx == -1) {
				CAM_DBG(CAM_CRM,
					"Prev Req: %lld [master] not found on link: %x [slave]",
					sync_req_id, sync_link->link_hdl);
				link->sync_link_sof_skip = true;
				return -EINVAL;
			}

			if ((sync_link->req.in_q->slot[sync_slot_idx].status !=
				CRM_SLOT_STATUS_REQ_APPLIED) &&
				(((sync_slot_idx - rd_idx + sync_num_slots) %
				sync_num_slots) >= 1) &&
				(sync_link->req.in_q->slot[rd_idx].status !=
				CRM_SLOT_STATUS_REQ_APPLIED)) {
				CAM_DBG(CAM_CRM,
					"Prev Req: %lld [master] not next on link: %x [slave]",
					sync_req_id,
					sync_link->link_hdl);
				return -EINVAL;
			}

			rc = __cam_req_mgr_check_link_is_ready(sync_link,
				sync_slot_idx, true);
			if (rc &&
				(sync_link->req.in_q->slot[sync_slot_idx].status
				!= CRM_SLOT_STATUS_REQ_APPLIED)) {
				CAM_DBG(CAM_CRM,
					"Req: %lld not ready on [slave] link: %x, rc=%d",
					sync_req_id, sync_link->link_hdl, rc);
				link->sync_link_sof_skip = true;
				return rc;
			}
		}
	} else {
		if (link->initial_skip)
			link->initial_skip = false;

		rc = __cam_req_mgr_check_link_is_ready(link, slot->idx, true);
		if (rc) {
			CAM_DBG(CAM_CRM,
				"Req: %lld [slave] not ready on link: %x, rc=%d",
				req_id, link->link_hdl, rc);
			link->sync_link_sof_skip = true;
			return rc;
		}

		next_idx = link->req.in_q->rd_idx;
		rd_idx = sync_link->req.in_q->rd_idx;
		__cam_req_mgr_inc_idx(&next_idx,
			(sync_link->max_delay - link->max_delay),
			link->req.in_q->num_slots);

		sync_req_id = link->req.in_q->slot[next_idx].req_id;

		if ((sync_link->initial_sync_req != -1) &&
			(sync_link->initial_sync_req <= sync_req_id)) {
			sync_slot_idx = __cam_req_mgr_find_slot_for_req(
				sync_link->req.in_q, sync_req_id);
			if (sync_slot_idx == -1) {
				CAM_DBG(CAM_CRM,
					"Next Req: %lld [slave] not found on link: %x [master]",
					sync_req_id, sync_link->link_hdl);
				link->sync_link_sof_skip = true;
				return -EINVAL;
			}

			if ((sync_link->req.in_q->slot[sync_slot_idx].status !=
				CRM_SLOT_STATUS_REQ_APPLIED) &&
				(((sync_slot_idx - rd_idx + sync_num_slots) %
				sync_num_slots) >= 1) &&
				(sync_link->req.in_q->slot[rd_idx].status !=
				CRM_SLOT_STATUS_REQ_APPLIED)) {
				CAM_DBG(CAM_CRM,
					"Next Req: %lld [slave] not next on link: %x [master]",
					sync_req_id, sync_link->link_hdl);
				return -EINVAL;
			}

			sync_slot = &sync_link->req.in_q->slot[sync_slot_idx];
			rc = __cam_req_mgr_check_link_is_ready(sync_link,
				sync_slot_idx, true);
			if (rc && (sync_slot->status !=
				CRM_SLOT_STATUS_REQ_APPLIED)) {
				CAM_DBG(CAM_CRM,
					"Next Req: %lld [slave] not ready on [master] link: %x, rc=%d",
					sync_req_id, sync_link->link_hdl, rc);
				link->sync_link_sof_skip = true;
				return rc;
			}
		}
	}

	CAM_DBG(CAM_REQ,
		"Req: %lld ready to apply on link: %x [validation successful]",
		req_id, link->link_hdl);

	return 0;
}


/**
 * __cam_req_mgr_check_sync_request_is_ready()
 *
 * @brief    : processes requests during sync mode
 * @link     : pointer to link whose input queue and req tbl are
 *             traversed through
 * @slot     : pointer to the current slot being processed
 * @return   : 0 for success, negative for failure
 *
 */
static int __cam_req_mgr_check_sync_req_is_ready(
	struct cam_req_mgr_core_link *link,
	struct cam_req_mgr_core_link *sync_link,
	struct cam_req_mgr_slot *slot,
	uint32_t trigger)
{
	struct cam_req_mgr_slot *sync_rd_slot = NULL;
	int64_t req_id = 0, sync_req_id = 0;
	int sync_slot_idx = 0, sync_rd_idx = 0, rc = 0;
	int32_t sync_num_slots = 0;
	uint64_t sync_frame_duration = 0;
	uint64_t sof_timestamp_delta = 0;
	uint64_t master_slave_diff = 0;
	uint64_t ref_timestamp, sync_ref_timestamp;
	bool ready = true, sync_ready = true;
	int slot_idx_diff = 0;

	if (!sync_link || !link) {
		CAM_ERR(CAM_CRM, "Sync link null");
		return -EINVAL;
	}

	req_id = slot->req_id;

	if (!sync_link->req.in_q) {
		CAM_ERR(CAM_CRM, "Link hdl %x in_q is NULL",
			sync_link->link_hdl);
		return -EINVAL;
	}

	sync_num_slots = sync_link->req.in_q->num_slots;
	sync_rd_idx    = sync_link->req.in_q->rd_idx;
	sync_rd_slot   = &sync_link->req.in_q->slot[sync_rd_idx];
	sync_req_id    = sync_rd_slot->req_id;

	CAM_DBG(CAM_REQ,
		"link_hdl %x sync link_hdl %x req %lld",
		link->link_hdl, sync_link->link_hdl, req_id);

	if ((sync_link->initial_skip) &&
		(req_id > sync_req_id)) {
		link->initial_skip = false;
		CAM_DBG(CAM_CRM,
			"sync link %x not streamed on",
			sync_link->link_hdl);
		return -EAGAIN;
	}
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	link->sof_timestamp += link->shift_timestamp;
#endif
	if (sync_link->prev_sof_timestamp)
		sync_frame_duration = sync_link->sof_timestamp -
			sync_link->prev_sof_timestamp;
	else
		sync_frame_duration = DEFAULT_FRAME_DURATION;

	ref_timestamp = link->sof_timestamp + slot->frame_sync_shift;
	sync_ref_timestamp = sync_link->sof_timestamp + sync_rd_slot->frame_sync_shift;

	sof_timestamp_delta =
		(ref_timestamp >= sync_ref_timestamp)
		? (ref_timestamp - sync_ref_timestamp)
		: (sync_ref_timestamp - ref_timestamp);

	CAM_DBG(CAM_CRM,
		"sync link %x last frame_duration is %d ns sof_timestamp_delta:%llu ns",
		sync_link->link_hdl, sync_frame_duration, sof_timestamp_delta);

	if (link->initial_skip) {
		link->initial_skip = false;

		if ((ref_timestamp > sync_ref_timestamp) &&
			(sync_ref_timestamp > 0) &&
			(ref_timestamp - sync_ref_timestamp) <
			(sync_frame_duration / 2)) {
			/*
			 * If this frame sync with the previous frame of sync
			 * link, then we need to skip this frame, since the
			 * previous frame of sync link is also skipped.
			 */
			CAM_DBG(CAM_CRM,
				"This frame sync with previous sync_link %x frame",
				sync_link->link_hdl);
			return -EAGAIN;
		} else if (ref_timestamp <= sync_ref_timestamp) {
			/*
			 * Sometimes, link receives the SOF event is eariler
			 * than sync link in IFE CSID side, but link's SOF
			 * event is processed later than sync link's, then
			 * we need to skip this SOF event since the sync
			 * link's SOF event is also skipped.
			 */
			CAM_DBG(CAM_CRM,
				"The previous frame of sync link is skipped");
			return -EAGAIN;
		}
	}

	if (sync_link->sync_link_sof_skip) {
		CAM_DBG(CAM_REQ,
			"No req applied on corresponding SOF on sync link: %x",
			sync_link->link_hdl);
		sync_link->sync_link_sof_skip = false;
		return -EAGAIN;
	}

	sync_slot_idx = __cam_req_mgr_find_slot_for_req(
		sync_link->req.in_q, req_id);
	if (sync_slot_idx == -1) {
		CAM_DBG(CAM_CRM, "Req: %lld not found on link: %x [other link]",
			req_id, sync_link->link_hdl);
		return -EAGAIN;
	}

	slot_idx_diff = (sync_slot_idx - sync_rd_idx + sync_num_slots) %
		sync_num_slots;
	if ((sync_link->req.in_q->slot[sync_slot_idx].status !=
		CRM_SLOT_STATUS_REQ_APPLIED) &&
		((slot_idx_diff > 1) ||
		((slot_idx_diff == 1) &&
		(sync_rd_slot->status !=
		CRM_SLOT_STATUS_REQ_APPLIED)))) {
		CAM_DBG(CAM_CRM,
			"Req: %lld [other link] not next req to be applied on link: %x",
			req_id, sync_link->link_hdl);
		return -EAGAIN;
	}

	rc = __cam_req_mgr_check_link_is_ready(link, slot->idx, true);
	if (rc) {
		CAM_DBG(CAM_CRM,
			"Req: %lld [My link] not ready on link: %x, rc=%d",
			req_id, link->link_hdl, rc);
		ready = false;
	}

	if (sync_link->req.in_q) {
		rc = __cam_req_mgr_check_link_is_ready(sync_link,
			sync_slot_idx, true);
		if (rc &&
			(sync_link->req.in_q->slot[sync_slot_idx].status !=
				CRM_SLOT_STATUS_REQ_APPLIED) &&
			(sync_link->req.in_q->slot[sync_slot_idx].status !=
				CRM_SLOT_STATUS_REQ_READY)) {
			CAM_DBG(CAM_CRM,
				"Req: %lld not ready on link: %x, rc=%d",
				req_id, sync_link->link_hdl, rc);
			sync_ready = false;
		}
	} else {
		CAM_ERR(CAM_CRM, "Link hdl %x in_q is NULL",
			sync_link->link_hdl);
		return -EINVAL;
	}

	/*
	 * If both of them are ready or not ready, then just
	 * skip this sof and don't skip sync link next SOF.
	 */
	if (sync_ready != ready) {
		CAM_DBG(CAM_CRM,
			"Req: %lld ready %d sync_ready %d, ignore sync link next SOF",
			req_id, ready, sync_ready);

		/*
		 * Only skip the frames if current frame sync with
		 * next frame of sync link.
		 */
		if (sof_timestamp_delta > sync_frame_duration / 2)
			link->sync_link_sof_skip = true;
		return -EINVAL;
	} else if (!ready) {
		CAM_DBG(CAM_CRM,
			"Req: %lld not ready on link: %x",
			req_id, link->link_hdl);
		return -EINVAL;
	}

	/*
	 * Do the self-correction when the frames are sync,
	 * we consider that the frames are synced if the
	 * difference of two SOF timestamp less than
	 * (sync_frame_duration / 5).
	 */
	master_slave_diff = sync_frame_duration;
	do_div(master_slave_diff, 5);
	if ((trigger == CAM_TRIGGER_POINT_SOF) &&
		(sync_ref_timestamp > 0) &&
		(sof_timestamp_delta < master_slave_diff) &&
		(sync_rd_slot->sync_mode == CAM_REQ_MGR_SYNC_MODE_SYNC) &&
		(req_id >= link->initial_sync_req) &&
		(req_id - link->initial_sync_req >=
		(INITIAL_IN_SYNC_REQ + link->max_delay))) {

		/*
		 * This means current frame should sync with next
		 * frame of sync link, then the request id of in
		 * rd slot of two links should be same.
		 */
		CAM_DBG(CAM_CRM,
			"link %x req_id %lld, sync_link %x req_id %lld",
			link->link_hdl, req_id,
			sync_link->link_hdl, sync_req_id);

		if (req_id > sync_req_id) {
			CAM_DBG(CAM_CRM,
				"link %x too quickly, skip this frame",
				link->link_hdl);
			return -EAGAIN;
		} else if (req_id < sync_req_id) {
			CAM_DBG(CAM_CRM,
				"sync link %x too quickly, skip next frame of sync link",
				sync_link->link_hdl);
			link->sync_link_sof_skip = true;
		} else if ((sync_link->req.in_q->slot[sync_slot_idx].status !=
			CRM_SLOT_STATUS_REQ_APPLIED) && !sync_link->is_sending_req) {
			CAM_DBG(CAM_CRM,
				"link %x other not applied", link->link_hdl);
			return -EAGAIN;
		}
	}
	CAM_DBG(CAM_REQ,
		"Req: %lld ready to apply on link: %x [validation successful]",
		req_id, link->link_hdl);

	return 0;
}

static int __cam_req_mgr_check_multi_sync_link_ready(
	struct cam_req_mgr_core_link *link,
	struct cam_req_mgr_core_link **sync_link,
	struct cam_req_mgr_slot *slot,
	int32_t num_sync_links,
	uint32_t trigger)
{
	int i, rc = 0;

	spin_lock_bh(&link->link_state_spin_lock);
	if (link->state == CAM_CRM_LINK_STATE_IDLE) {
		spin_unlock_bh(&link->link_state_spin_lock);
		CAM_ERR(CAM_CRM, "link hdl %x is in idle state",
				link->link_hdl);
		return -EINVAL;
	}
	spin_unlock_bh(&link->link_state_spin_lock);

	for (i = 0; i < num_sync_links; i++) {
		if (sync_link[i]) {
			if (slot->req_id <= sync_link[i]->last_flush_id) {
				CAM_DBG(CAM_CRM,
					"link:0x%x req:%lld has been flushed in sync link:0x%x, last_flush_id:%lld",
					link->link_hdl, slot->req_id,
					sync_link[i]->link_hdl,
					sync_link[i]->last_flush_id);

				rc = __cam_req_mgr_check_link_is_ready(link, slot->idx, true);
				if (!rc)
					continue;
				else {
					CAM_DBG(CAM_CRM, "link %x not ready", link->link_hdl);
					return -EINVAL;
				}
			}

			if (sync_link[i]->state ==
				CAM_CRM_LINK_STATE_IDLE) {
				CAM_ERR(CAM_CRM, "sync link hdl %x is idle",
					sync_link[i]->link_hdl);
				return -EINVAL;
			}
			if (link->max_delay == sync_link[i]->max_delay) {
				rc = __cam_req_mgr_check_sync_req_is_ready(
						link, sync_link[i],
						slot, trigger);
				if (rc < 0) {
					CAM_DBG(CAM_CRM, "link %x not ready",
						link->link_hdl);
					return rc;
				}
			} else if (link->max_delay >
					sync_link[i]->max_delay) {
				link->is_master = true;
				sync_link[i]->is_master = false;
				rc = __cam_req_mgr_check_sync_for_mslave(
					link, sync_link[i], slot);
				if (rc < 0) {
					CAM_DBG(CAM_CRM, "link%x not ready",
						link->link_hdl);
					return rc;
				}
			} else {
				link->is_master = false;
				sync_link[i]->is_master = true;
				rc = __cam_req_mgr_check_sync_for_mslave(
						link, sync_link[i], slot);
				if (rc < 0) {
					CAM_DBG(CAM_CRM, "link %x not ready",
						link->link_hdl);
					return rc;
				}
			}
		} else {
			CAM_ERR(CAM_REQ, "Sync link is null");
			return -EINVAL;
		}
	}

	rc = __cam_req_mgr_inject_delay(link->req.l_tbl,
		slot->idx, trigger);
	if (rc < 0) {
		CAM_DBG(CAM_CRM,
			"Req: %lld needs to inject delay at %s",
			slot->req_id,
			(trigger == CAM_TRIGGER_POINT_SOF) ? "SOF" : "EOF");
		return -EINVAL;
	}

	/*
	 *  At this point all validation is successfully done
	 *  and we can proceed to apply the given request.
	 *  Ideally the next call should return success.
	 */
	rc = __cam_req_mgr_check_link_is_ready(link, slot->idx, false);
	if (rc)
		CAM_WARN(CAM_CRM, "Unexpected return value rc: %d", rc);

	return 0;
}

/**
 * __cam_req_mgr_process_req()
 *
 * @brief              : processes read index in request queue and traverse through table
 * @link               : pointer to link whose input queue and req tbl are
 *                       traversed through
 *
 * @return   : 0 for success, negative for failure
 *
 */
static int __cam_req_mgr_process_req(struct cam_req_mgr_core_link *link,
	struct cam_req_mgr_trigger_notify *trigger_data)
{
	int                                  rc = 0, idx, pd, i;
	int                                  reset_step = 0;
	bool                                 is_applied = true;
	int32_t                              num_sync_links;
	uint32_t                             trigger = trigger_data->trigger;
	uint64_t                             wq_sched_timeout = 0;
	struct cam_req_mgr_slot             *slot = NULL;
	struct cam_req_mgr_tbl_slot         *tbl_slot = NULL;
	struct cam_req_mgr_req_queue        *in_q;
	struct cam_req_mgr_core_session     *session;
	struct cam_req_mgr_connected_device *dev = NULL;
	struct cam_req_mgr_core_link        *tmp_link = NULL;
	struct cam_req_mgr_apply            *apply_data = NULL;
	struct cam_req_mgr_core_link
		*sync_link[MAXIMUM_LINKS_PER_SESSION - 1];
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	char trace[64] = {0};
#endif
	session = (struct cam_req_mgr_core_session *)link->parent;
	if (!session) {
		CAM_WARN(CAM_CRM, "session ptr NULL %x", link->link_hdl);
		return -EINVAL;
	}

	mutex_lock(&session->lock);
	/*
	 * During session destroy/unlink the link state is updated and session
	 * mutex is released when flushing the workq. In case the wq is scheduled
	 * thereafter this API will then check the updated link state and exit
	 */
	spin_lock_bh(&link->link_state_spin_lock);
	if (link->state == CAM_CRM_LINK_STATE_IDLE) {
		spin_unlock_bh(&link->link_state_spin_lock);
		mutex_unlock(&session->lock);
		return -EPERM;
	}
	spin_unlock_bh(&link->link_state_spin_lock);

	in_q = link->req.in_q;
	/*
	 * Check if new read index,
	 * - if in pending  state, traverse again to complete
	 *    transaction of this read index.
	 * - if in applied_state, somthign wrong.
	 * - if in no_req state, no new req
	 */
	CAM_DBG(CAM_REQ,
		"%s Req[%lld] idx %d req_status %d link_hdl %x wd_timeout %d ms trigger:%d",
		((trigger == CAM_TRIGGER_POINT_SOF) ? "SOF" : "EOF"),
		in_q->slot[in_q->rd_idx].req_id, in_q->rd_idx,
		in_q->slot[in_q->rd_idx].status, link->link_hdl,
		in_q->slot[in_q->rd_idx].additional_timeout, trigger);

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	snprintf(trace, sizeof(trace), "KMD %d_3 Process Request %d", link->link_hdl, trigger);
	trace_long(trace, in_q->slot[in_q->rd_idx].req_id);
	trace_int(trace, 0);
	if (in_q->slot[in_q->rd_idx].req_id == -1) {
		memset(trace, 0, sizeof(trace));
		snprintf(trace, sizeof(trace), "KMD %d_4 Skip Frame", link->link_hdl);
		trace_long(trace, in_q->slot[in_q->rd_idx].req_id);
		trace_int(trace, 0);
		trace_begin_end("%s Req[%lld] idx %d req_status %d link_hdl %x wd_timeout %d ms trigger:%d",
			((trigger == CAM_TRIGGER_POINT_SOF) ? "SOF" : "EOF"),
			in_q->slot[in_q->rd_idx].req_id, in_q->rd_idx,
			in_q->slot[in_q->rd_idx].status, link->link_hdl,
			in_q->slot[in_q->rd_idx].additional_timeout, trigger);
	}
#endif
	slot = &in_q->slot[in_q->rd_idx];

	if ((trigger != CAM_TRIGGER_POINT_SOF) &&
		(trigger != CAM_TRIGGER_POINT_EOF))
		goto end;

	if ((trigger == CAM_TRIGGER_POINT_EOF) &&
		(!atomic_read(&link->eof_event_cnt))) {
		CAM_DBG(CAM_CRM, "Not any request to schedule at EOF");
		goto end;
	}

	if (slot->status == CRM_SLOT_STATUS_REQ_APPLIED)
		goto end;

	if (trigger == CAM_TRIGGER_POINT_SOF) {

		if (slot->status == CRM_SLOT_STATUS_NO_REQ) {
			CAM_DBG(CAM_CRM, "No Pending req");
			rc = 0;
			__cam_req_mgr_notify_frame_skip(link,
				trigger);
			goto end;
		}

		/*
		 * Update the timestamp in session lock protection
		 * to avoid timing issue.
		 */
		link->prev_sof_timestamp = link->sof_timestamp;
		link->sof_timestamp = trigger_data->sof_timestamp_val;

		/* Use half frame interval to detect the WQ congestion */
		wq_sched_timeout = CAM_REQ_MGR_HALF_FRAME_DURATION(link->sof_timestamp -
			link->prev_sof_timestamp) / CAM_COMMON_NS_PER_MS;

		/* Check for WQ congestion */
		if (jiffies_to_msecs(jiffies -
			link->last_sof_trigger_jiffies) <
			wq_sched_timeout)
			link->wq_congestion = true;
		else
			link->wq_congestion = false;

		/*
		 * Only update the jiffies for SOF trigger,
		 * since it is used to protect from
		 * applying fails in ISP which is triggered at SOF.
		 */
		if (trigger == CAM_TRIGGER_POINT_SOF)
			link->last_sof_trigger_jiffies = jiffies;

	}

	if (slot->status != CRM_SLOT_STATUS_REQ_READY) {
		if (slot->sync_mode == CAM_REQ_MGR_SYNC_MODE_SYNC) {
			/*
			 * If slot->num_sync_links is 0, it is sched_req_v1, otherwise
			 * it is sched_req_v2.
			 */
			if (slot->num_sync_links == 0) {
				num_sync_links = link->num_sync_links;
				for (i = 0; i < num_sync_links; i++)
					sync_link[i] = link->sync_link[i];
			} else {
				num_sync_links = slot->num_sync_links;
				for (i = 0; i < num_sync_links; i++)
					sync_link[i] = cam_get_link_priv(slot->sync_link_hdls[i]);
			}
			rc = __cam_req_mgr_check_multi_sync_link_ready(
				link, sync_link, slot, num_sync_links, trigger);
		} else {
			if (link->in_msync_mode) {
				CAM_DBG(CAM_CRM,
					"Settings master-slave non sync mode for link 0x%x",
					link->link_hdl);
				link->in_msync_mode = false;
				link->initial_sync_req = -1;
				for (i = 0; i < link->num_sync_links; i++) {
					if (link->sync_link[i]) {
						tmp_link = link->sync_link[i];
						tmp_link->initial_sync_req = -1;
						tmp_link->in_msync_mode = false;
					}
				}
			}

			/*
			 * Validate that if the req is ready to apply before
			 * checking the inject delay.
			 */
			rc = __cam_req_mgr_check_link_is_ready(link,
				slot->idx, true);

			if (!rc) {
				rc = __cam_req_mgr_inject_delay(link->req.l_tbl,
					slot->idx, trigger);
				if (rc < 0)
					CAM_DBG(CAM_CRM,
						"Req: %lld needs to inject delay at %s",
						slot->req_id,
						(trigger == CAM_TRIGGER_POINT_SOF) ? "SOF" : "EOF");

				if (!rc)
					rc = __cam_req_mgr_check_link_is_ready(link,
						slot->idx, false);
			}
		}

		if (rc < 0) {
			/*
			 * If traverse result is not success, then some devices
			 * are not ready with packet for the asked request id,
			 * hence try again in next sof
			 */
			slot->status = CRM_SLOT_STATUS_REQ_PENDING;
			spin_lock_bh(&link->link_state_spin_lock);
			if (link->state == CAM_CRM_LINK_STATE_ERR) {
				/*
				 * During error recovery all tables should be
				 * ready, don't expect to enter here.
				 * @TODO: gracefully handle if recovery fails.
				 */
				CAM_ERR_RATE_LIMIT(CAM_CRM,
					"FATAL recovery cant finish idx %d status %d",
					in_q->rd_idx,
					in_q->slot[in_q->rd_idx].status);
				rc = -EPERM;
			}
			spin_unlock_bh(&link->link_state_spin_lock);
			__cam_req_mgr_notify_frame_skip(link, trigger);
			__cam_req_mgr_validate_crm_wd_timer(link);
			goto end;
		} else {
			slot->status = CRM_SLOT_STATUS_REQ_READY;
			link->is_sending_req = true;
			CAM_DBG(CAM_REQ,
				"linx_hdl %x Req[%lld] idx %d ready to apply",
				link->link_hdl, in_q->slot[in_q->rd_idx].req_id,
				in_q->rd_idx);
		}
	} else {
		rc = __cam_req_mgr_inject_delay(link->req.l_tbl,
			slot->idx, trigger);
		if (rc < 0) {
			CAM_DBG(CAM_CRM,
				"linx_hdl %x Req: %lld needs to inject delay at %s",
				link->link_hdl, slot->req_id,
				(trigger == CAM_TRIGGER_POINT_SOF) ? "SOF" : "EOF");
			__cam_req_mgr_notify_frame_skip(link, trigger);
			__cam_req_mgr_validate_crm_wd_timer(link);
			goto end;
		}
	}
	mutex_unlock(&session->lock);

	rc = __cam_req_mgr_send_req(link, link->req.in_q, trigger, &dev);
	if (rc < 0) {
		/* Apply req failed retry at next sof */
		slot->status = CRM_SLOT_STATUS_REQ_PENDING;
		link->is_sending_req = false;

		if (!link->wq_congestion && dev) {
			if (rc != -EAGAIN)
				link->retry_cnt++;

			if (link->retry_cnt >= MAXIMUM_RETRY_ATTEMPTS) {
				CAM_DBG(CAM_CRM,
					"Max retry attempts (count %d) reached on link[0x%x] for req [%lld]",
					MAXIMUM_RETRY_ATTEMPTS, link->link_hdl,
					in_q->slot[in_q->rd_idx].req_id);

				cam_req_mgr_debug_delay_detect();
				trace_cam_delay_detect("CRM",
					"Max retry attempts reached",
					in_q->slot[in_q->rd_idx].req_id,
					CAM_DEFAULT_VALUE,
					link->link_hdl,
					CAM_DEFAULT_VALUE, rc);

				/*
				 * Try for internal recovery - primarily for IFE subdev
				 * if it's the first instance of stall
				 */
				if (!slot->recovery_counter)
					link->try_for_internal_recovery = true;

				__cam_req_mgr_notify_error_on_link(link, dev);

				/* Increment internal recovery counter */
				if (link->try_for_internal_recovery) {
					slot->recovery_counter++;
					link->try_for_internal_recovery = false;
				}

				link->retry_cnt = 0;
			}
		} else
			CAM_WARN_RATE_LIMIT(CAM_CRM,
				"workqueue congestion, last applied idx:%d rd idx:%d",
				in_q->last_applied_idx,
				in_q->rd_idx);
	} else {
		if (link->retry_cnt)
			link->retry_cnt = 0;

		/* Check for any long exposure settings */
		__cam_req_mgr_validate_crm_wd_timer(link);

		spin_lock_bh(&link->link_state_spin_lock);
		if (link->state == CAM_CRM_LINK_STATE_ERR) {
			CAM_WARN(CAM_CRM, "Err recovery done idx %d",
				in_q->rd_idx);
			link->state = CAM_CRM_LINK_STATE_READY;
		}
		spin_unlock_bh(&link->link_state_spin_lock);

		if (link->sync_link_sof_skip)
			link->sync_link_sof_skip = false;

		apply_data = link->req.apply_data;

		for (i = 0; i < link->num_devs; i++) {
			dev = &link->l_dev[i];
			pd = dev->dev_info.p_delay;

			if (pd >= CAM_PIPELINE_DELAY_MAX) {
				rc = -EINVAL;
				CAM_ERR(CAM_CRM, "link_hdl %x dev %s pd %d greater than max",
					link->link_hdl, dev->dev_info.name, pd);
				goto end;
			}

			idx = apply_data[pd].idx;
			tbl_slot = &dev->pd_tbl->slot[idx];

			if ((apply_data[pd].req_id != -1) &&
				(tbl_slot->req_apply_map != dev->pd_tbl->dev_mask)) {
				is_applied = false;
				break;
			}
		}

		if (is_applied) {
			slot->status = CRM_SLOT_STATUS_REQ_APPLIED;

			CAM_DBG(CAM_CRM, "req %d is applied on link %x success",
				slot->req_id,
				link->link_hdl);
			idx = in_q->rd_idx;
			reset_step = link->max_delay;

			for (i = 0; i < link->num_sync_links; i++) {
				if (link->sync_link[i]) {
					if ((link->in_msync_mode) &&
						(link->sync_link[i]->max_delay >
							reset_step))
						reset_step =
						link->sync_link[i]->max_delay;
				}
			}

			in_q->last_applied_idx = idx;

			__cam_req_mgr_dec_idx(
				&idx, reset_step + 1,
				in_q->num_slots);
			__cam_req_mgr_reset_req_slot(link, idx);
		}
		link->is_sending_req = false;
	}

	return rc;

end:
	mutex_unlock(&session->lock);
	return rc;
}

/**
 * __cam_req_mgr_add_tbl_to_link()
 *
 * @brief    : Add table to list under link sorted by pd decremeting order
 * @l_tbl    : list of pipeline delay tables.
 * @new_tbl  : new tbl which will be appended to above list as per its pd value
 *
 */
static void __cam_req_mgr_add_tbl_to_link(struct cam_req_mgr_req_tbl **l_tbl,
	struct cam_req_mgr_req_tbl *new_tbl)
{
	struct cam_req_mgr_req_tbl *tbl;

	if (!(*l_tbl) || (*l_tbl)->pd < new_tbl->pd) {
		new_tbl->next = *l_tbl;
		if (*l_tbl) {
			new_tbl->pd_delta =
				new_tbl->pd - (*l_tbl)->pd;
		}
		*l_tbl = new_tbl;
	} else {
		tbl = *l_tbl;

		/* Reach existing  tbl which has less pd value */
		while (tbl->next != NULL &&
			new_tbl->pd < tbl->next->pd) {
			tbl = tbl->next;
		}
		if (tbl->next != NULL) {
			new_tbl->pd_delta =
				new_tbl->pd - tbl->next->pd;
		} else {
			/* This is last table in linked list*/
			new_tbl->pd_delta = 0;
		}
		new_tbl->next = tbl->next;
		tbl->next = new_tbl;
		tbl->pd_delta = tbl->pd - new_tbl->pd;
	}
	CAM_DBG(CAM_CRM, "added pd %d tbl to link delta %d", new_tbl->pd,
		new_tbl->pd_delta);
}

/**
 * __cam_req_mgr_create_pd_tbl()
 *
 * @brief    : Creates new request table for new delay value
 * @delay    : New pd table allocated will have this delay value
 *
 * @return   : pointer to newly allocated table, NULL for failure
 *
 */
static struct cam_req_mgr_req_tbl *__cam_req_mgr_create_pd_tbl(int32_t delay)
{
	int i = 0;

	struct cam_req_mgr_req_tbl *tbl =
		CAM_MEM_ZALLOC(sizeof(struct cam_req_mgr_req_tbl), GFP_KERNEL);
	if (tbl != NULL) {
		tbl->num_slots = MAX_REQ_SLOTS;
		CAM_DBG(CAM_CRM, "pd= %d slots= %d", delay, tbl->num_slots);
		for (i = 0; i < MAX_REQ_SLOTS; i++) {
			tbl->slot[i].ops.apply_at_eof = false;
		}
	}

	return tbl;
}

/**
 * __cam_req_mgr_destroy_all_tbl()
 *
 * @brief   : This func will destroy all pipeline delay based req table structs
 * @l_tbl    : pointer to first table in list and it has max pd .
 *
 */
static void __cam_req_mgr_destroy_all_tbl(struct cam_req_mgr_req_tbl **l_tbl)
{
	struct cam_req_mgr_req_tbl  *tbl = *l_tbl, *temp;

	CAM_DBG(CAM_CRM, "*l_tbl %pK", tbl);
	while (tbl != NULL) {
		temp = tbl->next;
		CAM_MEM_FREE(tbl);
		tbl = temp;
	}
	*l_tbl = NULL;
}

/**
 * __cam_req_mgr_setup_in_q()
 *
 * @brief : Initialize req table data
 * @req   : request data pointer
 *
 * @return: 0 for success, negative for failure
 *
 */
static int  __cam_req_mgr_setup_in_q(struct cam_req_mgr_req_data *req)
{
	int                           i;
	struct cam_req_mgr_req_queue *in_q = req->in_q;

	if (!in_q) {
		CAM_ERR(CAM_CRM, "NULL in_q");
		return -EINVAL;
	}

	mutex_lock(&req->lock);
	in_q->num_slots = MAX_REQ_SLOTS;

	for (i = 0; i < in_q->num_slots; i++) {
		in_q->slot[i].idx = i;
		in_q->slot[i].req_id = -1;
		in_q->slot[i].skip_idx = 0;
		in_q->slot[i].status = CRM_SLOT_STATUS_NO_REQ;
	}

	in_q->wr_idx = 0;
	in_q->rd_idx = 0;
	mutex_unlock(&req->lock);

	return 0;
}

/**
 * __cam_req_mgr_reset_req_tbl()
 *
 * @brief : Initialize req table data
 * @req   : request queue pointer
 *
 * @return: 0 for success, negative for failure
 *
 */
static int __cam_req_mgr_reset_in_q(struct cam_req_mgr_req_data *req)
{
	struct cam_req_mgr_req_queue *in_q = req->in_q;

	if (!in_q) {
		CAM_ERR(CAM_CRM, "NULL in_q");
		return -EINVAL;
	}

	mutex_lock(&req->lock);
	memset(in_q->slot, 0,
		sizeof(struct cam_req_mgr_slot) * in_q->num_slots);
	in_q->num_slots = 0;

	in_q->wr_idx = 0;
	in_q->rd_idx = 0;
	mutex_unlock(&req->lock);

	return 0;
}

/**
 * __cam_req_mgr_process_sof_freeze()
 *
 * @brief : Apoptosis - Handles case when connected devices are not responding
 * @priv  : link information
 * @data  : task data
 *
 */
static int __cam_req_mgr_process_sof_freeze(void *priv, void *data)
{
	struct cam_req_mgr_core_link    *link = NULL;
	struct cam_req_mgr_req_queue    *in_q = NULL;
	struct cam_req_mgr_core_session *session = NULL;
	struct cam_req_mgr_message       msg = {0};
	int rc = 0;
	int64_t last_applied_req_id = -EINVAL;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	char fb_payload[PAYLOAD_LENGTH] = {0};
#endif

	if (!data || !priv) {
		CAM_ERR(CAM_CRM, "input args NULL %pK %pK", data, priv);
		return -EINVAL;
	}

	link = (struct cam_req_mgr_core_link *)priv;
	session = (struct cam_req_mgr_core_session *)link->parent;
	if (!session) {
		CAM_WARN(CAM_CRM, "session ptr NULL %x", link->link_hdl);
		return -EINVAL;
	}

	in_q = link->req.in_q;
	if (in_q) {
		mutex_lock(&link->req.lock);
		if (in_q->last_applied_idx >= 0)
			last_applied_req_id =
				in_q->slot[in_q->last_applied_idx].req_id;
		mutex_unlock(&link->req.lock);
	}

	spin_lock_bh(&link->link_state_spin_lock);
	if ((link->watchdog) && (link->watchdog->pause_timer)) {
		CAM_INFO(CAM_CRM,
			"link:%x watchdog paused, maybe stream on/off is delayed",
			link->link_hdl);
		spin_unlock_bh(&link->link_state_spin_lock);
		return rc;
	}
	spin_unlock_bh(&link->link_state_spin_lock);

	CAM_ERR(CAM_CRM,
		"SOF freeze for session: %d link: 0x%x max_pd: %d last_req_id:%d",
		session->session_hdl, link->link_hdl, link->max_delay,
		last_applied_req_id);
#ifdef OPLUS_FEATURE_CAMERA_COMMON
		KEVENT_FB_FRAME_ERROR(fb_payload, "SOF freeze", session->session_hdl);
#endif

	__cam_req_mgr_send_evt(0, CAM_REQ_MGR_LINK_EVT_SOF_FREEZE,
		CRM_KMD_ERR_FATAL, link);
	memset(&msg, 0, sizeof(msg));

	msg.session_hdl = session->session_hdl;
	msg.u.err_msg.error_type = CAM_REQ_MGR_ERROR_TYPE_SOF_FREEZE;
	msg.u.err_msg.request_id = 0;
	msg.u.err_msg.link_hdl   = link->link_hdl;
	msg.u.err_msg.error_code = CAM_REQ_MGR_ISP_UNREPORTED_ERROR;
	msg.u.err_msg.resource_size = 0;

	rc = cam_req_mgr_notify_message(&msg,
		V4L_EVENT_CAM_REQ_MGR_ERROR, V4L_EVENT_CAM_REQ_MGR_EVENT);

	if (rc)
		CAM_ERR(CAM_CRM,
			"Error notifying SOF freeze for session %d link 0x%x rc %d",
			session->session_hdl, link->link_hdl, rc);

	return rc;
}

/**
 * __cam_req_mgr_sof_freeze()
 *
 * @brief : Callback function for timer timeout indicating SOF freeze
 * @data  : timer pointer
 *
 */
static void __cam_req_mgr_sof_freeze(struct timer_list *timer_data)
{
	struct cam_req_mgr_timer     *timer =
		container_of(timer_data, struct cam_req_mgr_timer, sys_timer);
	struct crm_workq_task               *task = NULL;
	struct cam_req_mgr_core_link        *link = NULL;
	struct crm_task_payload             *task_data;

	if (!timer) {
		CAM_ERR(CAM_CRM, "NULL timer");
		return;
	}

	link = (struct cam_req_mgr_core_link *)timer->parent;

	task = cam_req_mgr_workq_get_task(link->workq);
	if (!task) {
		CAM_ERR(CAM_CRM, "No empty task");
		return;
	}

	task_data = (struct crm_task_payload *)task->payload;
	task_data->type = CRM_WORKQ_TASK_NOTIFY_FREEZE;
	task->process_cb = &__cam_req_mgr_process_sof_freeze;
	cam_req_mgr_workq_enqueue_task(task, link, CRM_TASK_PRIORITY_0);
}

/**
 * __cam_req_mgr_create_subdevs()
 *
 * @brief   : Create new crm  subdev to link with realtime devices
 * @l_dev   : list of subdevs internal to crm
 * @num_dev : num of subdevs to be created for link
 *
 * @return  : pointer to allocated list of devices
 */
static int __cam_req_mgr_create_subdevs(
	struct cam_req_mgr_connected_device **l_dev, int32_t num_dev)
{
	int rc = 0;
	*l_dev = CAM_MEM_ZALLOC(sizeof(struct cam_req_mgr_connected_device) *
		num_dev, GFP_KERNEL);
	if (!*l_dev)
		rc = -ENOMEM;

	return rc;
}

/**
 * __cam_req_mgr_destroy_subdev()
 *
 * @brief    : Cleans up the subdevs allocated by crm for link
 * @l_device : pointer to list of subdevs crm created
 *
 */
static void __cam_req_mgr_destroy_subdev(
	struct cam_req_mgr_connected_device **l_device)
{
	CAM_DBG(CAM_CRM, "*l_device %pK", *l_device);
	if (*(l_device) != NULL) {
		CAM_MEM_FREE(*(l_device));
		*l_device = NULL;
	}
}

/**
 * __cam_req_mgr_disconnect_link()
 *
 * @brief    : Unlinks all devices on the link
 * @link     : pointer to link
 *
 * @return   : returns if unlink for any device was success or failure
 */
static int __cam_req_mgr_disconnect_link(struct cam_req_mgr_core_link *link)
{
	int32_t                                 i = 0;
	struct cam_req_mgr_connected_device    *dev;
	struct cam_req_mgr_core_dev_link_setup  link_data;
	int                                     rc = 0;

	link_data.link_enable = 0;
	link_data.link_hdl = link->link_hdl;
	link_data.crm_cb = NULL;

	/* Using device ops unlink devices */
	for (i = 0; i < link->num_devs; i++) {
		dev = &link->l_dev[i];
		link_data.dev_hdl = dev->dev_hdl;
		if (dev->ops && dev->ops->link_setup) {
			rc = dev->ops->link_setup(&link_data);
			if (rc)
				CAM_ERR(CAM_CRM,
					"Unlink failed dev name %s hdl %x",
					dev->dev_info.name,
					dev->dev_hdl);
		}
		dev->dev_hdl = 0;
		dev->parent = NULL;
		dev->ops = NULL;
	}

	return rc;
}

/**
 * __cam_req_mgr_destroy_link_info()
 *
 * @brief    : Cleans up the mem allocated while linking
 * @link     : pointer to link, mem associated with this link is freed
 */
static void __cam_req_mgr_destroy_link_info(struct cam_req_mgr_core_link *link)
{
	__cam_req_mgr_destroy_all_tbl(&link->req.l_tbl);
	__cam_req_mgr_reset_in_q(&link->req);
	link->req.num_tbl = 0;
	mutex_destroy(&link->req.lock);

	link->pd_mask = 0;
	link->num_devs = 0;
	link->max_delay = CAM_PIPELINE_DELAY_0;
	link->min_delay = CAM_PIPELINE_DELAY_2;
}

/**
 * __cam_req_mgr_reserve_link()
 *
 * @brief: Reserves one link data struct within session
 * @session: session identifier
 *
 * @return: pointer to link reserved
 *
 */
static struct cam_req_mgr_core_link *__cam_req_mgr_reserve_link(
	struct cam_req_mgr_core_session *session)
{
	struct cam_req_mgr_core_link *link;
	struct cam_req_mgr_req_queue *in_q;
	int i;

	if (!session || !g_crm_core_dev) {
		CAM_ERR(CAM_CRM, "NULL session/core_dev ptr");
		return NULL;
	}

	if (session->num_links >= MAXIMUM_LINKS_PER_SESSION) {
		CAM_ERR(CAM_CRM, "Reached max links %d per session limit %d",
			session->num_links, MAXIMUM_LINKS_PER_SESSION);
		return NULL;
	}
	for (i = 0; i < MAXIMUM_LINKS_CAPACITY; i++) {
		if (!atomic_cmpxchg(&g_links[i].is_used, 0, 1)) {
			link = &g_links[i];
			CAM_DBG(CAM_CRM, "alloc link index %d", i);
			cam_req_mgr_core_link_reset(link);
			break;
		}
	}
	if (i == MAXIMUM_LINKS_CAPACITY)
		return NULL;

	in_q = CAM_MEM_ZALLOC(sizeof(struct cam_req_mgr_req_queue),
		GFP_KERNEL);
	if (!in_q) {
		CAM_ERR(CAM_CRM, "failed to create input queue, no mem");
		return NULL;
	}

	mutex_lock(&link->lock);
	link->num_devs = 0;
	link->max_delay = CAM_PIPELINE_DELAY_0;
	link->min_delay = CAM_PIPELINE_DELAY_2;
	memset(in_q->slot, 0,
		sizeof(struct cam_req_mgr_slot) * MAX_REQ_SLOTS);
	link->req.in_q = in_q;
	in_q->num_slots = 0;

	spin_lock_bh(&link->link_state_spin_lock);
	link->state = CAM_CRM_LINK_STATE_IDLE;
	spin_unlock_bh(&link->link_state_spin_lock);

	link->parent = (void *)session;
	link->is_sending_req = false;

	for (i = 0; i < MAXIMUM_LINKS_PER_SESSION - 1; i++)
		link->sync_link[i] = NULL;

	mutex_unlock(&link->lock);

	mutex_lock(&session->lock);
	/*  Loop through and find a free index */
	for (i = 0; i < MAXIMUM_LINKS_PER_SESSION; i++) {
		if (!session->links[i]) {
			CAM_DBG(CAM_CRM,
				"Free link index %d found, num_links=%d",
				i, session->num_links);
			session->links[i] = link;
			break;
		}
	}

	if (i == MAXIMUM_LINKS_PER_SESSION) {
		CAM_ERR(CAM_CRM, "Free link index not found");
		goto error;
	}

	session->num_links++;
	CAM_DBG(CAM_CRM, "Active session links (%d)",
		session->num_links);
	mutex_unlock(&session->lock);

	return link;
error:
	mutex_unlock(&session->lock);
	CAM_MEM_FREE(in_q);
	return NULL;
}

/*
 * __cam_req_mgr_free_link()
 *
 * @brief: Frees the link and its request queue
 *
 * @link: link identifier
 *
 */
static void __cam_req_mgr_free_link(struct cam_req_mgr_core_link *link)
{
	ptrdiff_t i;
	CAM_MEM_FREE(link->req.in_q);
	link->parent = NULL;
	i = link - g_links;
	CAM_DBG(CAM_CRM, "free link index %d", i);
	cam_req_mgr_core_link_reset(link);
	atomic_set(&g_links[i].is_used, 0);
}

/**
 * __cam_req_mgr_unreserve_link()
 *
 * @brief  : Removes the link data struct from the session and frees it
 * @session: session identifier
 * @link   : link identifier
 *
 */
static void __cam_req_mgr_unreserve_link(
	struct cam_req_mgr_core_session *session,
	struct cam_req_mgr_core_link *link)
{
	int i, j;

	if (!session || !link) {
		CAM_ERR(CAM_CRM, "NULL session/link ptr %pK %pK",
			session, link);
		return;
	}

	mutex_lock(&session->lock);
	if (!session->num_links) {
		CAM_WARN(CAM_CRM, "No active link or invalid state: hdl %x",
			link->link_hdl);
		mutex_unlock(&session->lock);
		return;
	}

	for (i = 0; i < MAXIMUM_LINKS_PER_SESSION; i++) {
		if (session->links[i] == link)
			session->links[i] = NULL;
		for (j = 0; j < MAXIMUM_LINKS_PER_SESSION - 1; j++) {
			if (link->sync_link[j]) {
				if (link->sync_link[j] == session->links[i])
					session->links[i]->sync_link[j] = NULL;
			}
		}
	}

	for (j = 0; j < MAXIMUM_LINKS_PER_SESSION - 1; j++)
		link->sync_link[j] = NULL;
	session->num_links--;
	CAM_DBG(CAM_CRM, "Active session links (%d)", session->num_links);
	mutex_unlock(&session->lock);
	__cam_req_mgr_free_link(link);
}

/* Workqueue context processing section */

static inline bool cam_req_mgr_check_for_functional_flush_caps(
	uint32_t bit_location,
	struct cam_req_mgr_flush_info *flush_info)
{
	if (!flush_info->reserved)
		return false;

	if (flush_info->reserved & bit_location)
		return true;

	return false;
}

static int __cam_req_mgr_flush_dev_with_max_pd(struct cam_req_mgr_core_link *link,
	struct cam_req_mgr_flush_info *flush_info, int max_pd)
{
	struct cam_req_mgr_connected_device *device;
	struct cam_req_mgr_flush_request flush_req = {0};
	int i, rc = 0;
	bool enable_standby = cam_req_mgr_check_for_functional_flush_caps(
			CAM_REQ_MGR_ENABLE_SENSOR_STANDBY, flush_info);

	for (i = 0; i < link->num_devs; i++) {
		device = &link->l_dev[i];
		if (device->dev_info.p_delay > max_pd)
			continue;

		flush_req.link_hdl = flush_info->link_hdl;
		flush_req.dev_hdl = device->dev_hdl;
		flush_req.req_id = flush_info->req_id;
		flush_req.type = flush_info->flush_type;
		flush_req.enable_sensor_standby =
			(!g_crm_core_dev->disable_sensor_standby && enable_standby);

		if (device->ops && device->ops->flush_req)
			rc = device->ops->flush_req(&flush_req);
	}

	return rc;
}

static int __cam_req_mgr_get_dev_pd(struct cam_req_mgr_core_link *link,
	enum cam_req_mgr_device_id dev_id)
{
	struct cam_req_mgr_connected_device *dev;
	int i;

	for (i = 0; i < link->num_devs; i++) {
		dev = &link->l_dev[i];
		if (dev->dev_info.dev_id == dev_id)
			return dev->dev_info.p_delay;
	}

	return -ENODEV;
}

/**
 * __cam_req_mgr_try_cancel_req()
 *
 * @brief      : Attempt to cancel a request. Requests that have not been
 *               applied to IFE can be cancelled.
 * @link       : Link on which to cancel
 * @flush_info : Contains info about the request to be flushed
 *
 * @return: 0 on success.
 */
static int __cam_req_mgr_try_cancel_req(struct cam_req_mgr_core_link *link,
	struct cam_req_mgr_flush_info *flush_info)
{
	struct cam_req_mgr_slot *slot = NULL;
	struct cam_req_mgr_req_queue *in_q = link->req.in_q;
	int idx, pd = CAM_PIPELINE_DELAY_MAX;

	idx = __cam_req_mgr_find_slot_for_req(in_q, flush_info->req_id);
	if (idx < 0) {
		CAM_ERR(CAM_CRM, "req_id %lld not found in input queue", flush_info->req_id);
		return -ENOENT;
	}

	slot = &in_q->slot[idx];
	if ((slot->req_id > 0) && slot->num_sync_links)
		__cam_req_mgr_disconnect_req_on_sync_link(link, slot);

	switch (slot->status) {
	case CRM_SLOT_STATUS_REQ_PENDING:
	case CRM_SLOT_STATUS_REQ_APPLIED:
		pd = __cam_req_mgr_get_dev_pd(link, CAM_REQ_MGR_DEVICE_IFE);
		if ((pd < 0) || (pd >= CAM_PIPELINE_DELAY_MAX)) {
			CAM_ERR(CAM_CRM, "pd: %d link_hdl: 0x%x red_id: %d", pd,
				link->link_hdl, flush_info->req_id);
			return pd;
		}

		if (flush_info->req_id <= link->req.prev_apply_data[pd].req_id) {
			CAM_WARN(CAM_CRM, "req %lld already applied to IFE on link 0x%x",
				flush_info->req_id, flush_info->link_hdl);
			return -EPERM;
		}

		/* find highest pd device that can be flushed */
		while (pd + 1 <= link->max_delay) {
			if (flush_info->req_id <= link->req.prev_apply_data[pd + 1].req_id)
				break;
			pd++;
		}
		fallthrough;
	case CRM_SLOT_STATUS_REQ_READY:
	case CRM_SLOT_STATUS_REQ_ADDED:
		slot->additional_timeout = 0;
		__cam_req_mgr_in_q_skip_idx(in_q, idx);
		break;
	default:
		CAM_ERR(CAM_CRM, "invalid state for slot link:0x%x req:%lld",
			flush_info->link_hdl, flush_info->req_id);
		return -EINVAL;
	}

	CAM_DBG(CAM_CRM, "cancelling request %lld on link 0x%x for devices with pd less than %d",
		flush_info->req_id, flush_info->link_hdl, pd);
	__cam_req_mgr_flush_dev_with_max_pd(link, flush_info, pd);
	link->open_req_cnt--;
	return 0;
}

/**
 * cam_req_mgr_process_flush_req()
 *
 * @brief: This runs in workque thread context. Call core funcs to check
 *         which requests need to be removed/cancelled.
 * @priv : link information.
 * @data : contains information about frame_id, link etc.
 *
 * @return: 0 on success.
 */
int cam_req_mgr_process_flush_req(void *priv, void *data)
{
	int                                  rc = 0;
	struct cam_req_mgr_flush_info       *flush_info = NULL;
	struct cam_req_mgr_core_link        *link = NULL;
	struct crm_task_payload             *task_data = NULL;

	if (!data || !priv) {
		CAM_ERR(CAM_CRM, "input args NULL %pK %pK", data, priv);
		return -EINVAL;
	}

	link = (struct cam_req_mgr_core_link *)priv;
	task_data = (struct crm_task_payload *)data;
	flush_info  = (struct cam_req_mgr_flush_info *)&task_data->u;
	CAM_DBG(CAM_REQ, "link_hdl %x req_id %lld type %d",
		flush_info->link_hdl,
		flush_info->req_id,
		flush_info->flush_type);

	trace_cam_flush_req(link, flush_info);

	mutex_lock(&link->req.lock);
	switch (flush_info->flush_type) {
	case CAM_REQ_MGR_FLUSH_TYPE_ALL:
		link->last_flush_id = flush_info->req_id;
		CAM_INFO(CAM_CRM, "Last request id to flush is %lld on link 0x%x",
			flush_info->req_id, link->link_hdl);
		__cam_req_mgr_flush_req_slot(link);
		__cam_req_mgr_reset_apply_data(link);
		__cam_req_mgr_flush_dev_with_max_pd(link, flush_info, link->max_delay);
		link->open_req_cnt = 0;
		link->resume_sync_curr_mask = 0x0;
		break;
	case CAM_REQ_MGR_FLUSH_TYPE_CANCEL_REQ:
		link->last_flush_id = flush_info->req_id;
		CAM_DBG(CAM_CRM, "Canceling req %lld on link 0x%x",
			flush_info->req_id, link->link_hdl);
		rc = __cam_req_mgr_try_cancel_req(link, flush_info);
		if (rc)
			CAM_WARN(CAM_CRM, "cannot cancel req_id %lld on link 0x%x",
				flush_info->req_id, flush_info->link_hdl);
		break;
	default:
		CAM_ERR(CAM_CRM, "Invalid flush type %u", flush_info->flush_type);
		rc = -EINVAL;
		break;
	}

	complete(&link->workq_comp);
	mutex_unlock(&link->req.lock);

	return rc;
}

/**
 * cam_req_mgr_process_sched_req()
 *
 * @brief: This runs in workque thread context. Call core funcs to check
 *         which peding requests can be processed.
 * @priv : link information.
 * @data : contains information about frame_id, link etc.
 *
 * @return: 0 on success.
 */
int cam_req_mgr_process_sched_req(void *priv, void *data)
{
	int                                  rc = 0, i, sync_idx = 0;
	struct cam_req_mgr_core_sched_req   *sched_req = NULL;
	struct cam_req_mgr_core_link        *link = NULL;
	struct cam_req_mgr_req_queue        *in_q = NULL;
	struct cam_req_mgr_slot             *slot = NULL;

	if (!data || !priv) {
		CAM_ERR(CAM_CRM, "input args NULL %pK %pK", data, priv);
		rc = -EINVAL;
		goto end;
	}
	link = (struct cam_req_mgr_core_link *)priv;
	sched_req  = (struct cam_req_mgr_core_sched_req *)data;
	in_q = link->req.in_q;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	CAM_DBG(CAM_CRM,
		"link_hdl %x req_id %lld at slot %d sync_mode %d is_master %d exp_timeout_val %d ms shift_timestamp:%u ns",
		sched_req->link_hdl, sched_req->req_id,
		in_q->wr_idx, sched_req->sync_mode,
		link->is_master,
		sched_req->additional_timeout,
		sched_req->params[0]);
#else
	CAM_DBG(CAM_CRM,
		"link_hdl %x req_id %lld at slot %d sync_mode %d is_master %d exp_timeout_val %d ms",
		sched_req->link_hdl, sched_req->req_id,
		in_q->wr_idx, sched_req->sync_mode,
		link->is_master,
		sched_req->additional_timeout);
#endif
	mutex_lock(&link->req.lock);
	slot = &in_q->slot[in_q->wr_idx];

	if (slot->status != CRM_SLOT_STATUS_NO_REQ &&
		slot->status != CRM_SLOT_STATUS_REQ_APPLIED)
		CAM_WARN(CAM_CRM, "in_q overwrite %d", slot->status);

	slot->status = CRM_SLOT_STATUS_REQ_ADDED;
	slot->req_id = sched_req->req_id;
	slot->sync_mode = sched_req->sync_mode;
	slot->skip_idx = 0;
	slot->recover = sched_req->bubble_enable;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	link->shift_timestamp = sched_req->params[0];
#endif

	if ((sched_req->num_valid_params > 0) &&
		(sched_req->param_mask & CAM_CRM_MISMATCHED_FRAME_MODE_MASK))
		slot->mismatched_frame_mode = sched_req->params[0];

	if (sched_req->additional_timeout < 0) {
		CAM_WARN(CAM_CRM,
			"Requested timeout is invalid [%dms]",
			sched_req->additional_timeout);
		slot->additional_timeout = 0;
	} else if (sched_req->additional_timeout >
		CAM_REQ_MGR_WATCHDOG_TIMEOUT_MAX) {
		CAM_WARN(CAM_CRM,
			"Requested timeout [%dms] max supported timeout [%dms] resetting to max",
			sched_req->additional_timeout,
			CAM_REQ_MGR_WATCHDOG_TIMEOUT_MAX);
		slot->additional_timeout = CAM_REQ_MGR_WATCHDOG_TIMEOUT_MAX;
	} else
		slot->additional_timeout = sched_req->additional_timeout;

	for (i = 0; i < sched_req->num_links; i++) {
		if (link->link_hdl != sched_req->link_hdls[i]) {
			slot->sync_link_hdls[sync_idx] = sched_req->link_hdls[i];
			CAM_DBG(CAM_REQ, "link:0x%x req:%lld sync_link[%d]:0x%x",
				link->link_hdl, slot->req_id, sync_idx,
				slot->sync_link_hdls[sync_idx]);
			sync_idx++;
		}
	}
	slot->num_sync_links = sync_idx;

	link->open_req_cnt++;
	CAM_DBG(CAM_REQ, "Open_req_cnt:%u after scheduling req:%d mismatched_frame_mode:%d",
		link->open_req_cnt, sched_req->req_id, slot->mismatched_frame_mode);
	__cam_req_mgr_inc_idx(&in_q->wr_idx, 1, in_q->num_slots);

	if (slot->sync_mode == CAM_REQ_MGR_SYNC_MODE_SYNC) {
		if (link->initial_sync_req == -1)
			link->initial_sync_req = slot->req_id;
	} else {
		link->initial_sync_req = -1;
		for (i = 0; i < link->num_sync_links; i++) {
			if (link->sync_link[i])
				link->sync_link[i]->initial_sync_req = -1;
		}
	}

	mutex_unlock(&link->req.lock);

end:
	return rc;
}

/**
 * cam_req_mgr_sync_info_for_sync_mode()
 *
 * @brief    : Keep links in sync by syncing the according req
 *             table slot info on the sync link during sync mode.
 * @link     : pointer to link whose input queue and req tbl are
 *             traversed through.
 * @add_req  : information about new request available at a device.
 * @slot     : pointer to the current slot being processed.
 * @type     : sync type
 * @return   : 0 for success, negative for failure.
 */
int cam_req_mgr_sync_info_for_sync_mode(
	struct cam_req_mgr_core_link   *link,
	struct cam_req_mgr_add_request *add_req,
	struct cam_req_mgr_tbl_slot    *slot,
	uint32_t type)
{
	int                                  rc = 0, i = 0, j = 0;
	int                                  idx;
	int32_t                              sync_dev_hdl = -1;
	char                                *dev_name = NULL;
	struct cam_req_mgr_connected_device *device = NULL;
	struct cam_req_mgr_req_tbl          *tbl = NULL;
	struct cam_req_mgr_tbl_slot         *sync_slot = NULL;

	for (i = 0; i < link->num_sync_links; i++) {
		mutex_lock(&link->sync_link[i]->req.lock);
		idx = __cam_req_mgr_find_slot_for_req(
			link->sync_link[i]->req.in_q, add_req->req_id);
		if (idx < 0) {
			CAM_ERR(CAM_CRM,
				"Req_id %lld not found in in_q for dev %s on link 0x%x",
				add_req->req_id, link->l_dev[i].dev_info.name,
				link->sync_link[i]->link_hdl);
			rc = -EBADSLT;
			mutex_unlock(&link->sync_link[i]->req.lock);
			goto end;
		}

		dev_name = __cam_req_mgr_dev_handle_to_name(
			add_req->dev_hdl, link);
		sync_dev_hdl = __cam_req_mgr_name_to_dev_handle(
			dev_name, link->sync_link[i]);

		for (j = 0; j < link->sync_link[i]->num_devs; j++) {
			device = &link->sync_link[i]->l_dev[j];
			CAM_DBG(CAM_CRM,
				"Req %lld device hdl %x : sync link dev hdl %x",
				add_req->req_id, device->dev_hdl, sync_dev_hdl);
			if (device->dev_hdl == sync_dev_hdl) {
				tbl = device->pd_tbl;
				break;
			}
		}

		if (!tbl) {
			CAM_ERR_RATE_LIMIT(CAM_CRM,
				"Sync link dev hdl %x not found", sync_dev_hdl);
			rc = -EINVAL;
			mutex_unlock(&link->sync_link[i]->req.lock);
			goto end;
		}

		sync_slot = &tbl->slot[idx];

		switch (type) {
		case CAM_SYNC_TYPE_DELAY_AT_SOF:
			sync_slot->inject_delay_at_sof = slot->inject_delay_at_sof;
			break;
		case CAM_SYNC_TYPE_DELAY_AT_EOF:
			sync_slot->inject_delay_at_eof = slot->inject_delay_at_eof;
			break;
		case CAM_SYNC_TYPE_APPLY_AT_EOF:
			sync_slot->ops.apply_at_eof = slot->ops.apply_at_eof;
			if (sync_slot->ops.num_dev == MAX_DEV_FOR_SPECIAL_OPS)
				CAM_WARN(CAM_CRM,
					"Sync link 0x%x slot:%d already has %d devices, can't add more",
					link->sync_link[i]->link_hdl, idx, sync_slot->ops.num_dev);
			else
				sync_slot->ops.dev_hdl[sync_slot->ops.num_dev++] = sync_dev_hdl;
			break;
		default:
			CAM_ERR(CAM_CRM, "Invalid Sync type:%u", type);
			rc = -EINVAL;
			break;
		}
		mutex_unlock(&link->sync_link[i]->req.lock);
	}
end:
	return rc;
}

/**
 * cam_req_mgr_process_add_req()
 *
 * @brief: This runs in workque thread context. Call core funcs to check
 *         which peding requests can be processed.
 * @priv : link information.
 * @data : contains information about frame_id, link etc.
 *
 * @return: 0 on success.
 */
int cam_req_mgr_process_add_req(void *priv, void *data)
{
	int                                  rc = 0, i = 0;
	int                                  idx;
	struct cam_req_mgr_add_request      *add_req = NULL;
	struct cam_req_mgr_core_link        *link = NULL;
	struct cam_req_mgr_connected_device *device = NULL;
	struct cam_req_mgr_req_tbl          *tbl = NULL;
	struct cam_req_mgr_tbl_slot         *slot = NULL;
	struct crm_task_payload             *task_data = NULL;
	struct cam_req_mgr_slot             *link_slot = NULL;
	struct cam_req_mgr_state_monitor     state;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	char trace[64] = {0};
#endif
	if (!data || !priv) {
		CAM_ERR(CAM_CRM, "input args NULL %pK %pK", data, priv);
		rc = -EINVAL;
		goto end;
	}

	link = (struct cam_req_mgr_core_link *)priv;
	task_data = (struct crm_task_payload *)data;
	add_req = (struct cam_req_mgr_add_request *)&task_data->u;

	for (i = 0; i < link->num_devs; i++) {
		device = &link->l_dev[i];
		if (device->dev_hdl == add_req->dev_hdl) {
			tbl = device->pd_tbl;
			break;
		}
	}
	if (!tbl) {
		CAM_ERR_RATE_LIMIT(CAM_CRM, "dev_hdl not found %x, %x %x",
			add_req->dev_hdl,
			link->l_dev[0].dev_hdl,
			link->l_dev[1].dev_hdl);
		rc = -EINVAL;
		goto end;
	}
	/*
	 * Go through request table and add
	 * request id to proper table
	 * 1. find req slot in in_q matching req_id.sent by dev
	 * 2. goto table of this device based on p_delay
	 * 3. mark req_ready_map with this dev_bit.
	 */

	mutex_lock(&link->req.lock);
	idx = __cam_req_mgr_find_slot_for_req(link->req.in_q, add_req->req_id);
	if (idx < 0) {
		CAM_ERR(CAM_CRM,
			"Req_id %lld not found in in_q for dev %s on link 0x%x",
			add_req->req_id, device->dev_info.name, link->link_hdl);
		rc = -EBADSLT;
		mutex_unlock(&link->req.lock);
		goto end;
	}

	link_slot = &link->req.in_q->slot[idx];
	slot = &tbl->slot[idx];

	if ((add_req->skip_at_sof & 0xFF) > slot->inject_delay_at_sof) {
		slot->inject_delay_at_sof = (add_req->skip_at_sof & 0xFF);
		CAM_DBG(CAM_CRM,
			"Req_id %llu injecting delay %llu frame at SOF by %s on link 0x%x",
			add_req->req_id,
			slot->inject_delay_at_sof,
			device->dev_info.name,
			link->link_hdl);

		if (link_slot->sync_mode == CAM_REQ_MGR_SYNC_MODE_SYNC) {
			rc = cam_req_mgr_sync_info_for_sync_mode(
				link, add_req, slot, CAM_SYNC_TYPE_DELAY_AT_SOF);
			if (rc < 0) {
				CAM_ERR(CAM_CRM,
					"Req_id %llu injecting delay %llu failed frame at SOF by %s for sync link 0x%x",
					add_req->req_id,
					slot->inject_delay_at_sof,
					device->dev_info.name,
					link->link_hdl);
				mutex_unlock(&link->req.lock);
				goto end;
			} else {
				CAM_DBG(CAM_CRM,
					"Req_id %llu injecting delay %llu frame at SOF by %s for sync link 0x%x",
					add_req->req_id,
					slot->inject_delay_at_sof,
					device->dev_info.name,
					link->link_hdl);
			}
		}
	}

	if ((add_req->skip_at_eof & 0xFF) > slot->inject_delay_at_eof) {
		slot->inject_delay_at_eof = (add_req->skip_at_eof & 0xFF);
		CAM_DBG(CAM_CRM,
			"Req_id %llu injecting delay %llu frame at EOF by %s on link 0x%x",
			add_req->req_id,
			slot->inject_delay_at_eof,
			device->dev_info.name,
			link->link_hdl);

		if (link_slot->sync_mode == CAM_REQ_MGR_SYNC_MODE_SYNC) {
			rc = cam_req_mgr_sync_info_for_sync_mode(
				link, add_req, slot, CAM_SYNC_TYPE_DELAY_AT_EOF);
			if (rc < 0) {
				CAM_ERR(CAM_CRM,
					"Req_id %llu injecting delay %llu failed frame at EOF by %s for sync link 0x%x",
					add_req->req_id,
					slot->inject_delay_at_eof,
					device->dev_info.name,
					link->link_hdl);
				mutex_unlock(&link->req.lock);
				goto end;
			} else {
				CAM_DBG(CAM_CRM,
					"Req_id %llu injecting delay %llu frame at EOF by %s for sync link 0x%x",
					add_req->req_id,
					slot->inject_delay_at_eof,
					device->dev_info.name,
					link->link_hdl);
			}
		}
	}

	if (add_req->trigger_eof) {
		slot->ops.apply_at_eof = true;
		if (slot->ops.num_dev == MAX_DEV_FOR_SPECIAL_OPS) {
			CAM_ERR(CAM_REQ,
				"Req_id %llu slot:%d already has %d devices, can't add more",
				add_req->req_id, idx, slot->ops.num_dev);
			mutex_unlock(&link->req.lock);
			return -EINVAL;
		}

		slot->ops.dev_hdl[slot->ops.num_dev++] = add_req->dev_hdl;
		CAM_DBG(CAM_REQ,
			"Req_id %llu slot:%d added for EOF tigger for Device: %s on link 0x%x",
			add_req->req_id, idx, device->dev_info.name, link->link_hdl);

		if (link_slot->sync_mode == CAM_REQ_MGR_SYNC_MODE_SYNC) {
			rc = cam_req_mgr_sync_info_for_sync_mode(
				link, add_req, slot, CAM_SYNC_TYPE_APPLY_AT_EOF);
			if (rc < 0) {
				CAM_ERR(CAM_REQ,
					"Req_id %llu slot:%d added failed for EOF tigger for Device: %s for sync link 0x%x",
					add_req->req_id,
					idx,
					device->dev_info.name,
					link->link_hdl);
				mutex_unlock(&link->req.lock);
				goto end;
			} else {
				CAM_DBG(CAM_REQ,
					"Req_id %llu slot:%d added for EOF tigger for Device: %s for sync link 0x%x",
					add_req->req_id,
					idx,
					device->dev_info.name,
					link->link_hdl);
			}
		}
	}

	if (slot->state != CRM_REQ_STATE_PENDING &&
		slot->state != CRM_REQ_STATE_EMPTY) {
		CAM_WARN(CAM_CRM,
			"Unexpected state %d for slot %d map %x for dev %s on link 0x%x",
			slot->state, idx, slot->req_ready_map,
			device->dev_info.name, link->link_hdl);
	}

	slot->state = CRM_REQ_STATE_PENDING;
	slot->req_ready_map |= BIT(device->dev_bit);

	CAM_DBG(CAM_CRM, "idx %d dev_hdl %x req_id %lld pd %d ready_map %x on link 0x%x",
		idx, add_req->dev_hdl, add_req->req_id, tbl->pd,
		slot->req_ready_map, link->link_hdl);

	trace_cam_req_mgr_add_req(link, idx, add_req, tbl, device);

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	snprintf(trace, sizeof(trace), "KMD %d_2 AddRequest %d_%s", add_req->link_hdl, device->dev_info.dev_id, device->dev_info.name);
	trace_long(trace, add_req->req_id);
	trace_int(trace, 0);
#endif

	if (slot->req_ready_map == tbl->dev_mask) {
		CAM_DBG(CAM_REQ,
			"link 0x%x idx %d req_id %lld pd %d SLOT READY",
			link->link_hdl, idx, add_req->req_id, tbl->pd);
		slot->state = CRM_REQ_STATE_READY;

		state.req_state = CAM_CRM_REQ_READY;
		state.req_id = add_req->req_id;
		state.dev_hdl = -1;
		state.frame_id = -1;
		__cam_req_mgr_update_state_monitor_array(link, &state);
	}
	mutex_unlock(&link->req.lock);

end:
	return rc;
}

/**
 * __cam_req_mgr_apply_on_bubble()
 *
 * @brief    : This API tries to apply settings to the device
 *             with highest pd on the bubbled frame
 * @link     : link information.
 * @err_info : contains information about frame_id, trigger etc.
 *
 */
void __cam_req_mgr_apply_on_bubble(
	struct cam_req_mgr_core_link    *link,
	struct cam_req_mgr_error_notify *err_info)
{
	int rc = 0;
	struct cam_req_mgr_trigger_notify trigger_data;

	trigger_data.dev_hdl = err_info->dev_hdl;
	trigger_data.frame_id = err_info->frame_id;
	trigger_data.link_hdl = err_info->link_hdl;
	trigger_data.sof_timestamp_val =
		err_info->sof_timestamp_val;
	trigger_data.trigger = err_info->trigger;

	rc = __cam_req_mgr_process_req(link, &trigger_data);
	if (rc)
		CAM_ERR(CAM_CRM,
			"Failed to apply request on bubbled frame");
}


/**
 * cam_req_mgr_issue_resume()
 *
 * @brief: This runs in workque thread context to issue
 *         resume cmd to relevant devices
 * @priv : link information
 * @data : contains information on the resume message
 *
 * @return: 0 on success
 */
int cam_req_mgr_issue_resume(void *priv, void *data)
{
	int rc = 0;
	struct cam_req_mgr_core_link *link = NULL;

	if (!data || !priv) {
		CAM_ERR(CAM_CRM, "input args NULL %pK %pK", data, priv);
		rc = -EINVAL;
		goto end;
	}

	link = (struct cam_req_mgr_core_link *)priv;
	rc = __cam_req_mgr_send_evt(0, CAM_REQ_MGR_LINK_EVT_RESUME_HW,
		CRM_KMD_ERR_MAX, link);

end:
	return rc;
}

/**
 * cam_req_mgr_process_error()
 *
 * @brief: This runs in workque thread context. bubble /err recovery.
 * @priv : link information.
 * @data : contains information about frame_id, link etc.
 *
 * @return: 0 on success.
 */
int cam_req_mgr_process_error(void *priv, void *data)
{
	int                                  rc = 0, idx = -1;
	int                                  i, slot_diff;
	struct cam_req_mgr_error_notify     *err_info = NULL;
	struct cam_req_mgr_core_link        *link = NULL;
	struct cam_req_mgr_req_queue        *in_q = NULL;
	struct cam_req_mgr_slot             *slot = NULL;
	struct crm_task_payload             *task_data = NULL;
	struct cam_req_mgr_state_monitor     state;
	struct cam_req_mgr_req_tbl          *tbl;

	if (!data || !priv) {
		CAM_ERR(CAM_CRM, "input args NULL %pK %pK", data, priv);
		rc = -EINVAL;
		goto end;
	}

	link = (struct cam_req_mgr_core_link *)priv;
	task_data = (struct crm_task_payload *)data;
	err_info  = (struct cam_req_mgr_error_notify *)&task_data->u;
	CAM_DBG(CAM_CRM, "link_hdl %x req_id %lld error %d",
		err_info->link_hdl,
		err_info->req_id,
		err_info->error);

	in_q = link->req.in_q;
	tbl = link->req.l_tbl;

	state.req_state = CAM_CRM_PROCESS_ERR;
	state.req_id = err_info->req_id;
	state.dev_hdl = -1;
	state.frame_id = -1;
	__cam_req_mgr_update_state_monitor_array(link, &state);

	mutex_lock(&link->req.lock);
	switch (err_info->error) {
	case CRM_KMD_ERR_BUBBLE:
	case CRM_KMD_WARN_INTERNAL_RECOVERY:
		idx = __cam_req_mgr_find_slot_for_req(in_q, err_info->req_id);
		if (idx < 0) {
			CAM_ERR_RATE_LIMIT(CAM_CRM,
				"req_id %lld not found in input queue",
				err_info->req_id);
		} else {
			CAM_DBG(CAM_CRM, "req_id %lld found at idx %d last_applied %d",
				err_info->req_id, idx, in_q->last_applied_idx);
			slot = &in_q->slot[idx];
			if (!slot->recover) {
				CAM_WARN(CAM_CRM,
					"err recovery disabled req_id %lld",
					err_info->req_id);
				mutex_unlock(&link->req.lock);
				return 0;
			} else if (slot->status != CRM_SLOT_STATUS_REQ_PENDING
			&& slot->status != CRM_SLOT_STATUS_REQ_APPLIED) {
				CAM_WARN(CAM_CRM,
					"req_id %lld can not be recovered %d",
					err_info->req_id, slot->status);
				mutex_unlock(&link->req.lock);
				return -EINVAL;
			}
			/* Bring processing pointer to bubbled req id */
			__cam_req_mgr_tbl_set_all_skip_cnt(&link->req.l_tbl);
			in_q->rd_idx = idx;
			/* Increment bubble counter only for bubble errors */
			if (err_info->error == CRM_KMD_ERR_BUBBLE)
				in_q->slot[idx].bubble_times++;
			in_q->slot[idx].status = CRM_SLOT_STATUS_REQ_ADDED;

			/* Reset request apply map for all pd tables */
			while (tbl) {
				tbl->slot[idx].req_apply_map = 0;
				tbl = tbl->next;
			}

			if (link->sync_link[0]) {
				in_q->slot[idx].sync_mode = 0;
				__cam_req_mgr_inc_idx(&idx, 1,
					link->req.l_tbl->num_slots);
				in_q->slot[idx].sync_mode = 0;
			}

			/*
			 * We need do internal recovery for back to back bubble, but if
			 * the request had been internal recovered, we won't recover
			 * it again.
			 */
			if ((in_q->slot[idx].bubble_times >= REQ_MAXIMUM_BUBBLE_TIMES) &&
				!in_q->slot[idx].internal_recovered) {
				link->try_for_internal_recovery = true;
				/*
				 * Notify all devices in the link about the error, need to pause
				 * watch dog before internal recovery because the recover may
				 * take a long time. if don't pause watch dog here, SOF freeze
				 * may be triggered. After recovery, need to restart watch dog.
				 */
				crm_timer_reset(link->watchdog);
				link->watchdog->pause_timer = true;
				__cam_req_mgr_send_evt(err_info->req_id,
					CAM_REQ_MGR_LINK_EVT_STALLED, CRM_KMD_ERR_FATAL, link);
				crm_timer_reset(link->watchdog);
				link->watchdog->pause_timer = false;
				in_q->slot[idx].internal_recovered = true;
				link->try_for_internal_recovery = false;
			}

			/*
			 * Reset till last applied, even if there are scheduling delays
			 * we start fresh from the request on which bubble has
			 * been reported
			 */
			idx = in_q->rd_idx;
			if (in_q->last_applied_idx >= 0) {
				slot_diff = in_q->last_applied_idx - idx;
				if (slot_diff < 0)
					slot_diff += link->req.l_tbl->num_slots;
			} else {
				/* Next req at the minimum may be applied */
				slot_diff = 1;
			}

			for (i = 0; i < slot_diff; i++) {
				__cam_req_mgr_inc_idx(&idx, 1,
					link->req.l_tbl->num_slots);

				CAM_DBG(CAM_CRM,
					"Recovery on idx: %d reset slot [idx: %d status: %d]",
					in_q->rd_idx, idx, in_q->slot[idx].status);
				if (in_q->slot[idx].status == CRM_SLOT_STATUS_REQ_APPLIED) {
					in_q->slot[idx].status = CRM_SLOT_STATUS_REQ_ADDED;

					tbl = link->req.l_tbl;
					/* Reset request apply map for all pd tables */
					while (tbl) {
						tbl->slot[idx].req_apply_map = 0;
						tbl = tbl->next;
					}
				}
			}

			spin_lock_bh(&link->link_state_spin_lock);
			link->state = CAM_CRM_LINK_STATE_ERR;
			spin_unlock_bh(&link->link_state_spin_lock);
			link->open_req_cnt++;

			/* Apply immediately to highest pd device on same frame */
			__cam_req_mgr_apply_on_bubble(link, err_info);
		}
		break;
	case CRM_KMD_ERR_FATAL:
		rc = __cam_req_mgr_send_evt(err_info->req_id,
			CAM_REQ_MGR_LINK_EVT_ERR, err_info->error, link);
		break;
	default:
		break;
	}
	mutex_unlock(&link->req.lock);

end:
	return rc;
}

/**
 * cam_req_mgr_process_stop()
 *
 * @brief: This runs in workque thread context. stop notification.
 * @priv : link information.
 * @data : contains information about frame_id, link etc.
 *
 * @return: 0 on success.
 */
int cam_req_mgr_process_stop(void *priv, void *data)
{
	int                                  rc = 0;
	struct cam_req_mgr_core_link        *link = NULL;

	if (!data || !priv) {
		CAM_ERR(CAM_CRM, "input args NULL %pK %pK", data, priv);
		rc = -EINVAL;
		goto end;
	}

	link = (struct cam_req_mgr_core_link *)priv;
	__cam_req_mgr_flush_req_slot(link);
end:
	return rc;
}

/**
 * cam_req_mgr_process_trigger()
 *
 * @brief: This runs in workque thread context. Call core funcs to check
 *         which peding requests can be processed.
 * @priv : link information.
 * @data : contains information about frame_id, link etc.
 *
 * @return: 0 on success.
 */
static int cam_req_mgr_process_trigger(void *priv, void *data)
{
	int                                  rc = 0;
	int32_t                              idx = -1;
	struct cam_req_mgr_trigger_notify   *trigger_data = NULL;
	struct cam_req_mgr_core_link        *link = NULL;
	struct cam_req_mgr_req_queue        *in_q = NULL;
	struct crm_task_payload             *task_data = NULL;
	int                                  reset_step = 0;
	int                                  i = 0;
	struct cam_req_mgr_state_monitor     state;

	if (!data || !priv) {
		CAM_ERR(CAM_CRM, "input args NULL %pK %pK", data, priv);
		rc = -EINVAL;
		goto end;
	}

	link = (struct cam_req_mgr_core_link *)priv;
	task_data = (struct crm_task_payload *)data;
	trigger_data = (struct cam_req_mgr_trigger_notify *)&task_data->u;

	CAM_DBG(CAM_REQ, "link_hdl %x frame_id %lld, trigger %x\n",
		trigger_data->link_hdl,
		trigger_data->frame_id,
		trigger_data->trigger);

	in_q = link->req.in_q;

	state.req_state = CAM_CRM_PROCESS_TRIGGER;
	state.req_id = in_q->slot[in_q->rd_idx].req_id;
	state.dev_hdl = -1;
	state.frame_id = trigger_data->frame_id;
	__cam_req_mgr_update_state_monitor_array(link, &state);

	mutex_lock(&link->req.lock);

	if (trigger_data->trigger == CAM_TRIGGER_POINT_SOF) {
		idx = __cam_req_mgr_find_slot_for_req(in_q,
			trigger_data->req_id);
		if (idx >= 0) {
			if (idx == in_q->last_applied_idx)
				in_q->last_applied_idx = -1;
			if (idx == in_q->rd_idx)
				__cam_req_mgr_dec_idx(&idx, 1, in_q->num_slots);

			reset_step = link->max_delay;
			for (i = 0; i < link->num_sync_links; i++) {
				if (link->sync_link[i]) {
					if ((link->in_msync_mode) &&
						(link->sync_link[i]->max_delay >
							reset_step))
						reset_step =
						link->sync_link[i]->max_delay;
				}
			}

			__cam_req_mgr_dec_idx(
				&idx, reset_step + 1,
				in_q->num_slots);

			__cam_req_mgr_reset_req_slot(link, idx);
		}
	} else if (trigger_data->trigger == CAM_TRIGGER_POINT_EOF) {
		if (link->properties_mask & CAM_LINK_PROPERTY_SENSOR_STANDBY_AFTER_EOF) {
			rc = __cam_req_mgr_send_evt(0, CAM_REQ_MGR_LINK_EVT_EOF,
				CRM_KMD_ERR_MAX, link);
			if (!rc)
				CAM_DBG(CAM_CRM, "Notify EOF event done on link:0x%x",
					link->link_hdl);
		}
	}

	/*
	 * Check if current read index is in applied state, if yes make it free
	 *    and increment read index to next slot.
	 */
	CAM_DBG(CAM_CRM, "link_hdl %x curent idx %d req_status %d",
		link->link_hdl, in_q->rd_idx, in_q->slot[in_q->rd_idx].status);

	spin_lock_bh(&link->link_state_spin_lock);

	if (link->state < CAM_CRM_LINK_STATE_READY) {
		CAM_WARN(CAM_CRM, "invalid link state:%d for link 0x%x",
			link->state, link->link_hdl);
		spin_unlock_bh(&link->link_state_spin_lock);
		rc = -EPERM;
		goto release_lock;
	}

	if (link->state == CAM_CRM_LINK_STATE_ERR)
		CAM_WARN_RATE_LIMIT(CAM_CRM, "Error recovery idx %d status %d",
			in_q->rd_idx,
			in_q->slot[in_q->rd_idx].status);

	spin_unlock_bh(&link->link_state_spin_lock);

	/*
	 * Move to next req at SOF only in case
	 * the rd_idx is updated at EOF.
	 */
	if ((trigger_data->trigger == CAM_TRIGGER_POINT_SOF) &&
		(in_q->slot[in_q->rd_idx].status == CRM_SLOT_STATUS_REQ_APPLIED)) {
		/*
		 * Do NOT reset req q slot data here, it can not be done
		 * here because we need to preserve the data to handle bubble.
		 *
		 * Check if any new req is pending in slot, if not finish the
		 * lower pipeline delay device with available req ids.
		 */
		CAM_DBG(CAM_CRM, "link[%x] Req[%lld] invalidating slot",
			link->link_hdl, in_q->slot[in_q->rd_idx].req_id);
		rc = __cam_req_mgr_move_to_next_req_slot(link);
		if (rc) {
			CAM_DBG(CAM_REQ,
				"No pending req to apply to lower pd devices");
			rc = 0;
			__cam_req_mgr_notify_frame_skip(link, trigger_data->trigger);
			goto release_lock;
		}
	}

	rc = __cam_req_mgr_process_req(link, trigger_data);

release_lock:
	mutex_unlock(&link->req.lock);
end:
	return rc;
}

/* Linked devices' Callback section */

/**
 * cam_req_mgr_cb_add_req()
 *
 * @brief    : Drivers call this function to notify new packet is available.
 * @add_req  : Information about new request available at a device.
 *
 * @return   : 0 on success, negative in case of failure
 *
 */
static int cam_req_mgr_cb_add_req(struct cam_req_mgr_add_request *add_req)
{
	int                             rc = 0, i = 0, idx;
	struct crm_workq_task          *task = NULL;
	struct cam_req_mgr_core_link   *link = NULL;
	struct cam_req_mgr_add_request *dev_req;
	struct crm_task_payload        *task_data;

	if (!add_req) {
		CAM_ERR(CAM_CRM, "sof_data is NULL");
		return -EINVAL;
	}

	link = cam_get_link_priv(add_req->link_hdl);
	if (!link) {
		CAM_DBG(CAM_CRM, "link ptr NULL %x", add_req->link_hdl);
		return -EINVAL;
	}

	CAM_DBG(CAM_REQ,
		"dev name %s dev_hdl %d dev req %lld, trigger_eof %d link_state %d on link 0x%x",
		__cam_req_mgr_dev_handle_to_name(add_req->dev_hdl, link),
		add_req->dev_hdl, add_req->req_id, add_req->trigger_eof,
		link->state, link->link_hdl);

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	trace_begin_end("ReqMgr AddRequest dev name %s dev_hdl %d dev req %lld, skip_before_applying %d link_state %d",
		__cam_req_mgr_dev_handle_to_name(add_req->dev_hdl, link),
		add_req->dev_hdl, add_req->req_id, add_req->trigger_eof,
		link->state);
#endif

	mutex_lock(&link->lock);
	/* Validate if req id is present in input queue */
	idx = __cam_req_mgr_find_slot_for_req(link->req.in_q, add_req->req_id);
	if (idx < 0) {
		if (((uint32_t)add_req->req_id) <= (link->last_flush_id)) {
			CAM_INFO(CAM_CRM,
				"req %lld not found in in_q; it has been flushed [last_flush_req %lld] link 0x%x",
				add_req->req_id, link->last_flush_id, link->link_hdl);
			rc = -EBADR;
		} else {
			CAM_ERR(CAM_CRM,
				"req %lld not found in in_q on link 0x%x [last_flush_req %lld]",
				add_req->req_id, link->link_hdl, link->last_flush_id);
			rc = -ENOENT;
		}
		goto end;
	}

	task = cam_req_mgr_workq_get_task(link->workq);
	if (!task) {
		CAM_ERR_RATE_LIMIT(CAM_CRM, "no empty task dev %x req %lld",
			add_req->dev_hdl, add_req->req_id);
		rc = -EBUSY;
		goto end;
	}

	task_data = (struct crm_task_payload *)task->payload;
	task_data->type = CRM_WORKQ_TASK_DEV_ADD_REQ;
	dev_req = (struct cam_req_mgr_add_request *)&task_data->u;
	dev_req->req_id = add_req->req_id;
	dev_req->link_hdl = add_req->link_hdl;
	dev_req->dev_hdl = add_req->dev_hdl;
	dev_req->trigger_eof = add_req->trigger_eof;
	dev_req->skip_at_sof = add_req->skip_at_sof;
	dev_req->skip_at_eof = add_req->skip_at_eof;
	if (dev_req->trigger_eof) {
		atomic_inc(&link->eof_event_cnt);
		CAM_DBG(CAM_REQ, "Req_id: %llu, eof_event_cnt: %d, link 0x%x",
			dev_req->req_id, link->eof_event_cnt, link->link_hdl);
		for (i = 0; i < link->num_sync_links; i++) {
			mutex_lock(&link->sync_link[i]->req.lock);
			atomic_set(&link->sync_link[i]->eof_event_cnt, link->eof_event_cnt.counter);
			CAM_DBG(CAM_REQ,
				"Sync link idx: %d, sync link_hdl: 0x%x, eof_event_cnt: %d",
				i, link->sync_link[i]->link_hdl,
				link->sync_link[i]->eof_event_cnt);
			mutex_unlock(&link->sync_link[i]->req.lock);
		}
	}

	task->process_cb = &cam_req_mgr_process_add_req;
	rc = cam_req_mgr_workq_enqueue_task(task, link, CRM_TASK_PRIORITY_0);
	CAM_DBG(CAM_CRM, "X: dev %x dev req %lld on link 0x%x",
		add_req->dev_hdl, add_req->req_id, link->link_hdl);

end:
	mutex_unlock(&link->lock);
	return rc;
}

/**
 * cam_req_mgr_cb_notify_err()
 *
 * @brief    : Error received from device, sends bubble recovery
 * @err_info : contains information about error occurred like bubble/overflow
 *
 * @return   : 0 on success, negative in case of failure
 *
 */
static int cam_req_mgr_cb_notify_err(
	struct cam_req_mgr_error_notify *err_info)
{
	int                              rc = 0;
	struct crm_workq_task           *task = NULL;
	struct cam_req_mgr_core_link    *link = NULL;
	struct cam_req_mgr_error_notify *notify_err;
	struct crm_task_payload         *task_data;
	struct cam_req_mgr_state_monitor state;

	if (!err_info) {
		CAM_ERR(CAM_CRM, "err_info is NULL");
		rc = -EINVAL;
		goto end;
	}

	link = cam_get_link_priv(err_info->link_hdl);
	if (!link) {
		CAM_DBG(CAM_CRM, "link ptr NULL %x", err_info->link_hdl);
		rc = -EINVAL;
		goto end;
	}

	state.req_state = CAM_CRM_NOTIFY_ERR;
	state.req_id = err_info->req_id;
	state.dev_hdl = -1;
	state.frame_id = err_info->frame_id;
	__cam_req_mgr_update_state_monitor_array(link, &state);

	spin_lock_bh(&link->link_state_spin_lock);
	if (link->state != CAM_CRM_LINK_STATE_READY) {
		CAM_WARN(CAM_CRM, "invalid link state:%d", link->state);
		spin_unlock_bh(&link->link_state_spin_lock);
		rc = -EPERM;
		goto end;
	}
	crm_timer_reset(link->watchdog);
	spin_unlock_bh(&link->link_state_spin_lock);

	task = cam_req_mgr_workq_get_task(link->workq);
	if (!task) {
		CAM_ERR(CAM_CRM, "no empty task req_id %lld", err_info->req_id);
		rc = -EBUSY;
		goto end;
	}

	task_data = (struct crm_task_payload *)task->payload;
	task_data->type = CRM_WORKQ_TASK_NOTIFY_ERR;
	notify_err = (struct cam_req_mgr_error_notify *)&task_data->u;
	notify_err->req_id = err_info->req_id;
	notify_err->link_hdl = err_info->link_hdl;
	notify_err->dev_hdl = err_info->dev_hdl;
	notify_err->error = err_info->error;
	notify_err->trigger = err_info->trigger;
	task->process_cb = &cam_req_mgr_process_error;
	rc = cam_req_mgr_workq_enqueue_task(task, link, CRM_TASK_PRIORITY_0);

end:
	return rc;
}

static int __cam_req_mgr_check_for_dual_trigger(
	struct cam_req_mgr_core_link    *link,
	uint32_t                         trigger)
{
	int rc  = -EAGAIN;

	CAM_DBG(CAM_CRM, "%s trigger_cnt [%u: %u]",
		(trigger == CAM_TRIGGER_POINT_SOF) ? "SOF" : "EOF",
		link->trigger_cnt[0][trigger], link->trigger_cnt[1][trigger]);

	if (link->trigger_cnt[0][trigger] == link->trigger_cnt[1][trigger]) {
		link->trigger_cnt[0][trigger] = 0;
		link->trigger_cnt[1][trigger] = 0;
		rc = 0;
		return rc;
	}

	if ((link->trigger_cnt[0][trigger] &&
		(link->trigger_cnt[0][trigger] - link->trigger_cnt[1][trigger] > 1)) ||
		(link->trigger_cnt[1][trigger] &&
		(link->trigger_cnt[1][trigger] - link->trigger_cnt[0][trigger] > 1))) {

		CAM_WARN(CAM_CRM,
			"One of the devices could not generate trigger");

		link->trigger_cnt[0][trigger] = 0;
		link->trigger_cnt[1][trigger] = 0;
		CAM_DBG(CAM_CRM, "Reset the trigger cnt");
		return rc;
	}

	CAM_DBG(CAM_CRM, "Only one device has generated trigger for %s",
		(trigger == CAM_TRIGGER_POINT_SOF) ? "SOF" : "EOF");

	return rc;
}

/**
 * cam_req_mgr_cb_notify_timer()
 *
 * @brief      : Notify SOF timer to pause after flush
 * @timer_data : contains information about frame_id, link etc.
 *
 * @return  : 0 on success
 *
 */
static int cam_req_mgr_cb_notify_timer(
	struct cam_req_mgr_timer_notify *timer_data)
{
	int                              rc = 0;
	struct cam_req_mgr_core_link    *link = NULL;

	if (!timer_data) {
		CAM_ERR(CAM_CRM, "timer data  is NULL");
		rc = -EINVAL;
		goto end;
	}

	link = cam_get_link_priv(timer_data->link_hdl);
	if (!link) {
		CAM_DBG(CAM_CRM, "link ptr NULL %x", timer_data->link_hdl);
		rc = -EINVAL;
		goto end;
	}

	spin_lock_bh(&link->link_state_spin_lock);
	if (link->state < CAM_CRM_LINK_STATE_READY) {
		CAM_WARN(CAM_CRM, "invalid link state:%d", link->state);
		spin_unlock_bh(&link->link_state_spin_lock);
		rc = -EPERM;
		goto end;
	}
	if (link->watchdog) {
		if (!timer_data->state)
			link->watchdog->pause_timer = true;
		else
			link->watchdog->pause_timer = false;
		crm_timer_reset(link->watchdog);
		CAM_DBG(CAM_CRM, "link %x pause_timer %d",
			link->link_hdl, link->watchdog->pause_timer);
	}

	spin_unlock_bh(&link->link_state_spin_lock);

end:
	return rc;
}

/*
 * cam_req_mgr_cb_notify_stop()
 *
 * @brief    : Stop received from device, resets the morked slots
 * @err_info : contains information about error occurred like bubble/overflow
 *
 * @return   : 0 on success, negative in case of failure
 *
 */
static int cam_req_mgr_cb_notify_stop(
	struct cam_req_mgr_notify_stop *stop_info)
{
	int                              rc = 0;
	struct crm_workq_task           *task = NULL;
	struct cam_req_mgr_core_link    *link = NULL;
	struct cam_req_mgr_notify_stop  *notify_stop;
	struct crm_task_payload         *task_data;

	if (!stop_info) {
		CAM_ERR(CAM_CRM, "stop_info is NULL");
		rc = -EINVAL;
		goto end;
	}

	link = cam_get_link_priv(stop_info->link_hdl);
	if (!link) {
		CAM_DBG(CAM_CRM, "link ptr NULL %x", stop_info->link_hdl);
		rc = -EINVAL;
		goto end;
	}

	spin_lock_bh(&link->link_state_spin_lock);
	if (link->state != CAM_CRM_LINK_STATE_READY) {
		CAM_WARN(CAM_CRM, "invalid link state:%d", link->state);
		spin_unlock_bh(&link->link_state_spin_lock);
		rc = -EPERM;
		goto end;
	}
	crm_timer_reset(link->watchdog);
	link->watchdog->pause_timer = true;
	spin_unlock_bh(&link->link_state_spin_lock);

	task = cam_req_mgr_workq_get_task(link->workq);
	if (!task) {
		CAM_ERR(CAM_CRM, "no empty task");
		rc = -EBUSY;
		goto end;
	}

	task_data = (struct crm_task_payload *)task->payload;
	task_data->type = CRM_WORKQ_TASK_NOTIFY_ERR;
	notify_stop = (struct cam_req_mgr_notify_stop *)&task_data->u;
	notify_stop->link_hdl = stop_info->link_hdl;
	task->process_cb = &cam_req_mgr_process_stop;
	rc = cam_req_mgr_workq_enqueue_task(task, link, CRM_TASK_PRIORITY_0);

end:
	return rc;
}



/**
 * cam_req_mgr_cb_notify_trigger()
 *
 * @brief   : SOF received from device, sends trigger through workqueue
 * @sof_data: contains information about frame_id, link etc.
 *
 * @return  : 0 on success
 *
 */
static int cam_req_mgr_cb_notify_trigger(
	struct cam_req_mgr_trigger_notify *trigger_data)
{
	int32_t                          rc = 0, trigger_id = 0;
	uint32_t                         trigger;
	struct crm_workq_task           *task = NULL;
	struct cam_req_mgr_core_link    *link = NULL;
	struct cam_req_mgr_trigger_notify   *notify_trigger;
	struct crm_task_payload         *task_data;
	struct cam_req_mgr_state_monitor state;
	struct cam_req_mgr_req_queue    *in_q = NULL;

	if (!trigger_data) {
		CAM_ERR(CAM_CRM, "trigger_data is NULL");
		rc = -EINVAL;
		goto end;
	}

	link = cam_get_link_priv(trigger_data->link_hdl);
	if (!link) {
		CAM_DBG(CAM_CRM, "link ptr NULL %x", trigger_data->link_hdl);
		rc = -EINVAL;
		goto end;
	}

	CAM_DBG(CAM_REQ, "link_hdl %x frame_id %lld, trigger %x\n",
		trigger_data->link_hdl,
		trigger_data->frame_id,
		trigger_data->trigger);

	trigger_id = trigger_data->trigger_id;
	trigger = trigger_data->trigger;

	spin_lock_bh(&link->req.reset_link_spin_lock);
	in_q = link->req.in_q;
	if (!in_q) {
		CAM_DBG(CAM_CRM, "in_q ptr NULL, link_hdl %x", trigger_data->link_hdl);
		spin_unlock_bh(&link->req.reset_link_spin_lock);
		rc = -EINVAL;
		goto end;
	}

	state.req_state = CAM_CRM_NOTIFY_TRIGGER;
	state.req_id = in_q->slot[in_q->rd_idx].req_id;
	state.dev_hdl = -1;
	state.frame_id = trigger_data->frame_id;
	__cam_req_mgr_update_state_monitor_array(link, &state);
	spin_unlock_bh(&link->req.reset_link_spin_lock);

	/*
	 * Reduce the workq overhead when there is
	 * not any eof event found.
	 */
	if (trigger == CAM_TRIGGER_POINT_EOF) {
		if (!atomic_read(&link->eof_event_cnt) &&
			!(link->properties_mask & CAM_LINK_PROPERTY_SENSOR_STANDBY_AFTER_EOF)) {
			CAM_DBG(CAM_CRM, "No any request to schedule at EOF");
			goto end;
		}
	}

	spin_lock_bh(&link->link_state_spin_lock);
	if (link->state < CAM_CRM_LINK_STATE_READY) {
		CAM_WARN_RATE_LIMIT(CAM_CRM, "invalid link state:%d", link->state);
		spin_unlock_bh(&link->link_state_spin_lock);
		rc = -EPERM;
		goto end;
	}

	if ((link->watchdog) && (link->watchdog->pause_timer) &&
		(trigger == CAM_TRIGGER_POINT_SOF))
		link->watchdog->pause_timer = false;

	if (link->dual_trigger) {
		if ((trigger_id >= 0) && (trigger_id <
			CAM_REQ_MGR_MAX_TRIGGERS)) {
			link->trigger_cnt[trigger_id][trigger]++;
			rc = __cam_req_mgr_check_for_dual_trigger(link, trigger);
			if (rc) {
				spin_unlock_bh(&link->link_state_spin_lock);
				goto end;
			}
		} else {
			CAM_ERR(CAM_CRM, "trigger_id invalid %d", trigger_id);
			rc = -EINVAL;
			spin_unlock_bh(&link->link_state_spin_lock);
			goto end;
		}
	}

	if (trigger_data->trigger == CAM_TRIGGER_POINT_SOF)
		crm_timer_reset(link->watchdog);

	if ((in_q->slot[in_q->rd_idx].mismatched_frame_mode == CRM_DROP_MISMATCHED_FRMAE) &&
		link->last_applied_done_timestamp &&
		((trigger_data->boot_timestamp - CAM_CRM_SENSOR_APPLIY_DELAY_THRESHOLD) <
		link->last_applied_done_timestamp)) {
		CAM_WARN(CAM_REQ,
			"Apply delayed req:%lld, link:0x%x, timestamp boot:0x%llx applied:0x%llx",
			in_q->slot[in_q->rd_idx].req_id, trigger_data->link_hdl,
			trigger_data->boot_timestamp, link->last_applied_done_timestamp);
		spin_unlock_bh(&link->link_state_spin_lock);
		goto end;
	}
	spin_unlock_bh(&link->link_state_spin_lock);

	task = cam_req_mgr_workq_get_task(link->workq);
	if (!task) {
		CAM_ERR_RATE_LIMIT(CAM_CRM, "no empty task frame %lld",
			trigger_data->frame_id);
		rc = -EBUSY;
		spin_lock_bh(&link->link_state_spin_lock);
		if ((link->watchdog) && !(link->watchdog->pause_timer))
			link->watchdog->pause_timer = true;
		spin_unlock_bh(&link->link_state_spin_lock);
		goto end;
	}
	task_data = (struct crm_task_payload *)task->payload;
	task_data->type = (trigger_data->trigger == CAM_TRIGGER_POINT_SOF) ?
		CRM_WORKQ_TASK_NOTIFY_SOF : CRM_WORKQ_TASK_NOTIFY_EOF;
	notify_trigger = (struct cam_req_mgr_trigger_notify *)&task_data->u;
	notify_trigger->frame_id = trigger_data->frame_id;
	notify_trigger->link_hdl = trigger_data->link_hdl;
	notify_trigger->dev_hdl = trigger_data->dev_hdl;
	notify_trigger->trigger = trigger_data->trigger;
	notify_trigger->req_id = trigger_data->req_id;
	notify_trigger->sof_timestamp_val = trigger_data->sof_timestamp_val;
	task->process_cb = &cam_req_mgr_process_trigger;
	rc = cam_req_mgr_workq_enqueue_task(task, link, CRM_TASK_PRIORITY_0);

end:
	return rc;
}

static int cam_req_mgr_cb_notify_msg(
	struct cam_req_mgr_notify_msg *msg)
{
	int                                  i, slot_idx, rc = 0;
	struct cam_req_mgr_slot             *slot;
	struct cam_req_mgr_core_link        *link = NULL;
	struct cam_req_mgr_req_queue        *in_q = NULL;
	struct cam_req_mgr_connected_device *dev = NULL;
	struct cam_req_mgr_link_evt_data     evt_data;

	if (!msg) {
		CAM_ERR(CAM_CRM, "msg is NULL");
		return -EINVAL;
	}

	link = cam_get_link_priv(msg->link_hdl);
	if (!link) {
		CAM_DBG(CAM_CRM, "link ptr NULL %x", msg->link_hdl);
		return -EINVAL;
	}

	CAM_DBG(CAM_REQ, "link_hdl 0x%x request id:%llu msg type:%d",
		link->link_hdl, msg->req_id, msg->msg_type);

	switch (msg->msg_type) {
	case CAM_REQ_MGR_MSG_SENSOR_FRAME_INFO:
		spin_lock_bh(&link->req.reset_link_spin_lock);
		in_q = link->req.in_q;
		if (!in_q) {
			CAM_ERR(CAM_CRM, "in_q ptr NULL, link_hdl %x", msg->link_hdl);
			spin_unlock_bh(&link->req.reset_link_spin_lock);
			return -EINVAL;
		}

		slot_idx = __cam_req_mgr_find_slot_for_req(
			in_q, msg->req_id);
		if (slot_idx == -1) {
			CAM_ERR(CAM_CRM, "Req: %lld not found on link: 0x%x",
				msg->req_id, link->link_hdl);
			spin_unlock_bh(&link->req.reset_link_spin_lock);
			return -EINVAL;
		}

		slot = &in_q->slot[slot_idx];
		spin_unlock_bh(&link->req.reset_link_spin_lock);
		slot->frame_sync_shift = msg->u.frame_info.frame_sync_shift;

		for (i = 0; i < link->num_devs; i++) {
			dev = &link->l_dev[i];
			if (msg->dev_hdl == dev->dev_hdl)
				continue;

			evt_data.req_id = msg->req_id;
			evt_data.dev_hdl = dev->dev_hdl;
			evt_data.link_hdl = link->link_hdl;
			evt_data.evt_type = CAM_REQ_MGR_LINK_EVT_SENSOR_FRAME_INFO;
			evt_data.u.frame_info = msg->u.frame_info;

			if (dev->ops && dev->ops->process_evt) {
				rc = dev->ops->process_evt(&evt_data);
				if (rc)
					CAM_ERR(CAM_CRM,
						"Failed to set properties on link 0x%x dev 0x%x",
						link->link_hdl, dev->dev_hdl);
			}
		}
		break;
	case CAM_REQ_MGR_MSG_NOTIFY_FOR_SYNCED_RESUME:  {
		struct crm_task_payload *task_data = NULL;
		struct crm_workq_task *task = NULL;
		struct cam_req_mgr_notify_msg *notify_msg;

		for (i = 0; i < link->num_devs; i++) {
			dev = &link->l_dev[i];
			if (msg->dev_hdl != dev->dev_hdl)
				continue;

			link->resume_sync_curr_mask |= BIT(dev->dev_info.dev_id);
		}

		if (link->resume_sync_curr_mask == link->resume_sync_dev_mask) {
			task = cam_req_mgr_workq_get_task(link->workq);
			if (!task) {
				CAM_ERR(CAM_CRM, "no empty task");
				rc = -EBUSY;
				break;
			}

			link->resume_sync_curr_mask = 0;
			task_data = (struct crm_task_payload *)task->payload;
			task_data->type = CRM_WORKQ_TASK_TRIGGER_SYNCED_RESUME;
			notify_msg = (struct cam_req_mgr_notify_msg *)&task_data->u;
			notify_msg->req_id = msg->req_id;
			notify_msg->link_hdl = msg->link_hdl;
			notify_msg->dev_hdl = msg->dev_hdl;
			notify_msg->msg_type = msg->msg_type;
			task->process_cb = &cam_req_mgr_issue_resume;
			rc = cam_req_mgr_workq_enqueue_task(task, link, CRM_TASK_PRIORITY_0);
		}
	}
		break;
	case CAM_REQ_MGR_MSG_UPDATE_DEVICE_INFO: {
		for (i = 0; i < link->num_devs; i++) {
			dev = &link->l_dev[i];
			if (dev->dev_hdl != msg->dev_hdl)
				continue;

			snprintf(dev->dev_info.name, sizeof(dev->dev_info.name), "%s(%s)",
				"cam-isp", msg->u.ife_hw_name);
			break;
		}
	}
		break;
	default:
		rc = -EINVAL;
		CAM_ERR(CAM_CRM,
			"link:0x%x gets an invalid msg:%d from dev:0x%x at req:%llu",
			link->link_hdl, msg->msg_type,
			msg->dev_hdl, msg->req_id);
		break;
	}

	return rc;
}

static struct cam_req_mgr_crm_cb cam_req_mgr_ops = {
	.notify_trigger = cam_req_mgr_cb_notify_trigger,
	.notify_err     = cam_req_mgr_cb_notify_err,
	.add_req        = cam_req_mgr_cb_add_req,
	.notify_timer   = cam_req_mgr_cb_notify_timer,
	.notify_stop    = cam_req_mgr_cb_notify_stop,
	.notify_msg     = cam_req_mgr_cb_notify_msg,
};

/**
 * __cam_req_mgr_setup_link_info()
 *
 * @brief     : Sets up input queue, create pd based tables, communicate with
 *              devs connected on this link and setup communication.
 * @link      : pointer to link to setup
 * @link_info : link_info coming from CSL to prepare link
 *
 * @return    : 0 on success, negative in case of failure
 *
 */
static int __cam_req_mgr_setup_link_info(struct cam_req_mgr_core_link *link,
	struct cam_req_mgr_ver_info *link_info)
{
	int                                     rc = 0, i = 0, num_devices = 0;
	struct cam_req_mgr_core_dev_link_setup  link_data;
	struct cam_req_mgr_connected_device    *dev;
	struct cam_req_mgr_req_tbl             *pd_tbl;
	enum cam_pipeline_delay                 max_delay;
	enum cam_modeswitch_delay               max_modeswitch;
	uint32_t num_trigger_devices = 0;
	if (link_info->version == VERSION_1) {
		if (link_info->u.link_info_v1.num_devices >
			CAM_REQ_MGR_MAX_HANDLES)
			return -EPERM;
	} else if (link_info->version == VERSION_2) {
		if (link_info->u.link_info_v2.num_devices >
			CAM_REQ_MGR_MAX_HANDLES_V2)
			return -EPERM;
	}

	mutex_init(&link->req.lock);
	CAM_DBG(CAM_CRM, "LOCK_DBG in_q lock %pK", &link->req.lock);
	link->req.num_tbl = 0;

	rc = __cam_req_mgr_setup_in_q(&link->req);
	if (rc < 0)
		return rc;

	max_delay = CAM_PIPELINE_DELAY_0;
	max_modeswitch = CAM_MODESWITCH_DELAY_0;
	if (link_info->version == VERSION_1)
		num_devices = link_info->u.link_info_v1.num_devices;
	else if (link_info->version == VERSION_2)
		num_devices = link_info->u.link_info_v2.num_devices;
	for (i = 0; i < num_devices; i++) {
		dev = &link->l_dev[i];
		/* Using dev hdl, get ops ptr to communicate with device */
		if (link_info->version == VERSION_1)
			dev->ops = (struct cam_req_mgr_kmd_ops *)
					cam_get_device_ops(
					link_info->u.link_info_v1.dev_hdls[i]);
		else if (link_info->version == VERSION_2)
			dev->ops = (struct cam_req_mgr_kmd_ops *)
					cam_get_device_ops(
					link_info->u.link_info_v2.dev_hdls[i]);
		if (!dev->ops ||
			!dev->ops->get_dev_info ||
			!dev->ops->link_setup) {
			CAM_ERR(CAM_CRM, "FATAL: device ops NULL");
			rc = -ENXIO;
			goto error;
		}
		if (link_info->version == VERSION_1)
			dev->dev_hdl = link_info->u.link_info_v1.dev_hdls[i];
		else if (link_info->version == VERSION_2)
			dev->dev_hdl = link_info->u.link_info_v2.dev_hdls[i];
		dev->parent = (void *)link;
		dev->dev_info.dev_hdl = dev->dev_hdl;
		rc = dev->ops->get_dev_info(&dev->dev_info);

		trace_cam_req_mgr_connect_device(link, &dev->dev_info);
		if (link_info->version == VERSION_1)
			CAM_DBG(CAM_CRM,
				"%x: connected: %s, id %d, delay %d, trigger %x",
				link_info->u.link_info_v1.session_hdl,
				dev->dev_info.name,
				dev->dev_info.dev_id, dev->dev_info.p_delay,
				dev->dev_info.trigger);
		else if (link_info->version == VERSION_2)
			CAM_DBG(CAM_CRM,
				"%x: connected: %s, id %d, delay %d, trigger %x",
				link_info->u.link_info_v2.session_hdl,
				dev->dev_info.name,
				dev->dev_info.dev_id, dev->dev_info.p_delay,
				dev->dev_info.trigger);
		if (rc < 0 ||
			dev->dev_info.p_delay >=
			CAM_PIPELINE_DELAY_MAX ||
			dev->dev_info.p_delay <
			CAM_PIPELINE_DELAY_0) {
			CAM_ERR(CAM_CRM, "get device info failed");
			goto error;
		} else if (dev->dev_info.m_delay >=
			CAM_MODESWITCH_DELAY_MAX ||
			dev->dev_info.m_delay <
			CAM_MODESWITCH_DELAY_0) {
			CAM_ERR(CAM_CRM, "get mode switch info failed");
			goto error;
		} else {
			if (link_info->version == VERSION_1) {
				CAM_DBG(CAM_CRM, "%x: connected: %s, delay %d",
					link_info->u.link_info_v1.session_hdl,
					dev->dev_info.name,
					dev->dev_info.p_delay);
				}
			else if (link_info->version == VERSION_2) {
				CAM_DBG(CAM_CRM, "%x: connected: %s, delay %d",
					link_info->u.link_info_v2.session_hdl,
					dev->dev_info.name,
					dev->dev_info.p_delay);
				}
			if (dev->dev_info.p_delay > max_delay)
				max_delay = dev->dev_info.p_delay;

			if (dev->dev_info.m_delay > max_modeswitch)
				max_modeswitch = dev->dev_info.m_delay;
		}

		if (dev->dev_info.trigger_on)
			num_trigger_devices++;

		if (dev->dev_info.resume_sync_on)
			link->resume_sync_dev_mask |= BIT(dev->dev_info.dev_id);
	}

	if (num_trigger_devices > CAM_REQ_MGR_MAX_TRIGGERS) {
		CAM_ERR(CAM_CRM,
			"Unsupported number of trigger devices %u",
			num_trigger_devices);
		rc = -EINVAL;
		goto error;
	}

	link_data.link_enable = 1;
	link_data.link_hdl = link->link_hdl;
	link_data.crm_cb = &cam_req_mgr_ops;
	link_data.max_delay = max_delay;
	link_data.mode_switch_max_delay = max_modeswitch;
	if (num_trigger_devices == CAM_REQ_MGR_MAX_TRIGGERS)
		link->dual_trigger = true;

	num_trigger_devices = 0;
	for (i = 0; i < num_devices; i++) {
		dev = &link->l_dev[i];

		link_data.dev_hdl = dev->dev_hdl;
		/*
		 * For unique pipeline delay table create request
		 * tracking table
		 */
		if (link->pd_mask & BIT(dev->dev_info.p_delay)) {
			pd_tbl = __cam_req_mgr_find_pd_tbl(link->req.l_tbl,
				dev->dev_info.p_delay);
			if (!pd_tbl) {
				CAM_ERR(CAM_CRM, "pd %d tbl not found",
					dev->dev_info.p_delay);
				rc = -ENXIO;
				goto error;
			}
		} else {
			pd_tbl = __cam_req_mgr_create_pd_tbl(
				dev->dev_info.p_delay);
			if (pd_tbl == NULL) {
				CAM_ERR(CAM_CRM, "create new pd tbl failed");
				rc = -ENXIO;
				goto error;
			}
			pd_tbl->pd = dev->dev_info.p_delay;
			link->pd_mask |= BIT(pd_tbl->pd);
			/*
			 * Add table to list and also sort list
			 * from max pd to lowest
			 */
			__cam_req_mgr_add_tbl_to_link(&link->req.l_tbl, pd_tbl);
		}
		dev->dev_bit = pd_tbl->dev_count++;
		dev->pd_tbl = pd_tbl;
		pd_tbl->dev_mask |= BIT(dev->dev_bit);
		CAM_DBG(CAM_CRM, "dev_bit %u name %s pd %u mask %d",
			dev->dev_bit, dev->dev_info.name, pd_tbl->pd,
			pd_tbl->dev_mask);
		link_data.trigger_id = -1;
		if ((dev->dev_info.trigger_on) && (link->dual_trigger)) {
			link_data.trigger_id = num_trigger_devices;
			num_trigger_devices++;
		}

		/* Communicate with dev to establish the link */
		dev->ops->link_setup(&link_data);

		/* Compute max/min pipeline delay */
		if (dev->dev_info.p_delay > link->max_delay)
			link->max_delay = dev->dev_info.p_delay;
		if (dev->dev_info.p_delay < link->min_delay)
			link->min_delay = dev->dev_info.p_delay;

		/* Compute max/min modeswitch delay */
		if (dev->dev_info.m_delay > link->max_mswitch_delay)
			link->max_mswitch_delay = dev->dev_info.m_delay;
		if (dev->dev_info.m_delay < link->min_mswitch_delay)
			link->min_mswitch_delay = dev->dev_info.m_delay;
	}

	link->num_devs = num_devices;

	/* Assign id for pd tables */
	__cam_req_mgr_tbl_set_id(link->req.l_tbl, &link->req);

	/* At start, expect max pd devices, all are in skip state */
	__cam_req_mgr_tbl_set_all_skip_cnt(&link->req.l_tbl);

	return 0;

error:
	__cam_req_mgr_destroy_link_info(link);
	return rc;
}

/* IOCTLs handling section */
int cam_req_mgr_create_session(
	struct cam_req_mgr_session_info *ses_info)
{
	int                              rc = 0;
	int32_t                          session_hdl;
	struct cam_req_mgr_core_session *cam_session = NULL;

	if (!ses_info) {
		CAM_DBG(CAM_CRM, "NULL session info pointer");
		return -EINVAL;
	}
	mutex_lock(&g_crm_core_dev->crm_lock);
	cam_session = CAM_MEM_ZALLOC(sizeof(*cam_session),
		GFP_KERNEL);
	if (!cam_session) {
		rc = -ENOMEM;
		goto end;
	}

	session_hdl = cam_create_session_hdl((void *)cam_session);
	if (session_hdl < 0) {
		CAM_ERR(CAM_CRM, "unable to create session_hdl = %x",
			session_hdl);
		rc = session_hdl;
		CAM_MEM_FREE(cam_session);
		goto end;
	}
	ses_info->session_hdl = session_hdl;

	mutex_init(&cam_session->lock);
	CAM_DBG(CAM_CRM, "LOCK_DBG session lock %pK hdl 0x%x",
		&cam_session->lock, session_hdl);

	mutex_lock(&cam_session->lock);
	cam_session->session_hdl = session_hdl;
	cam_session->num_links = 0;
	cam_session->sync_mode = CAM_REQ_MGR_SYNC_MODE_NO_SYNC;
	list_add(&cam_session->entry, &g_crm_core_dev->session_head);
	mutex_unlock(&cam_session->lock);
end:
	mutex_unlock(&g_crm_core_dev->crm_lock);
	return rc;
}

/**
 * __cam_req_mgr_unlink()
 *
 * @brief  : Unlink devices on a link structure from the session
 *           This API is to be invoked with session mutex held
 * @session: session of the link
 * @link   : Pointer to the link structure
 *
 * @return: 0 for success, negative for failure
 *
 */
static int __cam_req_mgr_unlink(
	struct cam_req_mgr_core_session *session,
	struct cam_req_mgr_core_link *link)
{
	int rc;

	spin_lock_bh(&link->link_state_spin_lock);
	link->state = CAM_CRM_LINK_STATE_IDLE;
	spin_unlock_bh(&link->link_state_spin_lock);

	if (!link->is_shutdown) {
		rc = __cam_req_mgr_disconnect_link(link);
		if (rc)
			CAM_ERR(CAM_CORE,
				"Unlink for all devices was not successful");
	}

	mutex_lock(&link->lock);
	spin_lock_bh(&link->link_state_spin_lock);
	/* Destroy timer of link */
	crm_timer_exit(&link->watchdog);
	spin_unlock_bh(&link->link_state_spin_lock);
	/* Release session mutex for workq processing */
	mutex_unlock(&session->lock);
	/* Destroy workq of link */
	cam_req_mgr_workq_destroy(&link->workq);
	/* Acquire session mutex after workq flush */
	mutex_lock(&session->lock);
	/* Cleanup request tables and unlink devices */
	__cam_req_mgr_destroy_link_info(link);
	/* Free memory holding data of linked devs */

	__cam_req_mgr_destroy_subdev(&link->l_dev);

	/* Destroy the link handle */
	rc = cam_destroy_link_hdl(link->link_hdl);
	if (rc < 0) {
		CAM_ERR(CAM_CRM, "error destroying link hdl %x rc %d",
			link->link_hdl, rc);
	} else
		link->link_hdl = -1;

	mutex_unlock(&link->lock);
	return rc;
}

int cam_req_mgr_destroy_session(
		struct cam_req_mgr_session_info *ses_info,
		bool is_shutdown)
{
	int rc;
	int i;
	struct cam_req_mgr_core_session *cam_session = NULL;
	struct cam_req_mgr_core_link *link;

	if (!ses_info) {
		CAM_DBG(CAM_CRM, "NULL session info pointer");
		return -EINVAL;
	}

	mutex_lock(&g_crm_core_dev->crm_lock);
	cam_session = cam_get_session_priv(ses_info->session_hdl);
	if (!cam_session || (cam_session->session_hdl != ses_info->session_hdl)) {
		CAM_ERR(CAM_CRM, "session: %s, ses_info->ses_hdl:%x, session->ses_hdl:%x",
			CAM_IS_NULL_TO_STR(cam_session), ses_info->session_hdl,
			(!cam_session) ? CAM_REQ_MGR_DEFAULT_HDL_VAL : cam_session->session_hdl);
		rc = -ENOENT;
		goto end;

	}

	mutex_lock(&cam_session->lock);
	if (cam_session->num_links) {
		CAM_DBG(CAM_CRM, "destroy session %x num_active_links %d",
			ses_info->session_hdl,
			cam_session->num_links);

		for (i = 0; i < MAXIMUM_LINKS_PER_SESSION; i++) {
			link = cam_session->links[i];
			if (!link)
				continue;

			CAM_DBG(CAM_CRM, "Unlink link hdl 0x%x",
				link->link_hdl);
			/* Ignore return value since session is going away */
			link->is_shutdown = is_shutdown;
			__cam_req_mgr_unlink(cam_session, link);
			__cam_req_mgr_free_link(link);
		}
	}
	list_del(&cam_session->entry);
	mutex_unlock(&cam_session->lock);
	mutex_destroy(&cam_session->lock);

	CAM_MEM_FREE(cam_session);

	rc = cam_destroy_session_hdl(ses_info->session_hdl);
	if (rc < 0)
		CAM_ERR(CAM_CRM, "unable to destroy session_hdl = %x rc %d",
			ses_info->session_hdl, rc);

end:
	mutex_unlock(&g_crm_core_dev->crm_lock);
	return rc;
}

static void cam_req_mgr_process_workq_link_worker(struct work_struct *w)
{
	cam_req_mgr_process_workq(w);
}

int cam_req_mgr_link(struct cam_req_mgr_ver_info *link_info)
{
	int                                     rc = 0;
	int                                     wq_flag = 0;
	char                                    buf[128];
	struct cam_create_dev_hdl               root_dev = {0};
	struct cam_req_mgr_core_session        *cam_session;
	struct cam_req_mgr_core_link           *link;

	if (!link_info) {
		CAM_DBG(CAM_CRM, "NULL pointer");
		return -EINVAL;
	}
	if (link_info->u.link_info_v1.num_devices > CAM_REQ_MGR_MAX_HANDLES) {
		CAM_ERR(CAM_CRM, "Invalid num devices %d",
			link_info->u.link_info_v1.num_devices);
		return -EINVAL;
	}

	mutex_lock(&g_crm_core_dev->crm_lock);

	/* session hdl's priv data is cam session struct */
	cam_session = cam_get_session_priv(link_info->u.link_info_v1.session_hdl);
	if (!cam_session || (cam_session->session_hdl != link_info->u.link_info_v1.session_hdl)) {
		CAM_ERR(CAM_CRM, "session: %s, link_info->ses_hdl:%x, session->ses_hdl:%x",
			CAM_IS_NULL_TO_STR(cam_session), link_info->u.link_info_v1.session_hdl,
			(!cam_session) ? CAM_REQ_MGR_DEFAULT_HDL_VAL : cam_session->session_hdl);
		mutex_unlock(&g_crm_core_dev->crm_lock);
		return -EINVAL;
	}

	/* Allocate link struct and map it with session's request queue */
	link = __cam_req_mgr_reserve_link(cam_session);
	if (!link) {
		CAM_ERR(CAM_CRM, "failed to reserve new link");
		mutex_unlock(&g_crm_core_dev->crm_lock);
		return -EINVAL;
	}
	CAM_DBG(CAM_CRM, "link reserved %pK %x", link, link->link_hdl);

	root_dev.session_hdl = link_info->u.link_info_v1.session_hdl;
	root_dev.priv = (void *)link;
	root_dev.dev_id = CAM_CRM;

	mutex_lock(&link->lock);
	/* Create unique handle for link */
	link->link_hdl = cam_create_link_hdl(&root_dev);
	if (link->link_hdl < 0) {
		CAM_ERR(CAM_CRM,
			"Insufficient memory to create new device handle");
		rc = link->link_hdl;
		goto link_hdl_fail;
	}
	link_info->u.link_info_v1.link_hdl = link->link_hdl;
	link->last_flush_id = 0;

	/* Allocate memory to hold data of all linked devs */
	rc = __cam_req_mgr_create_subdevs(&link->l_dev,
		link_info->u.link_info_v1.num_devices);
	if (rc < 0) {
		CAM_ERR(CAM_CRM,
			"Insufficient memory to create new crm subdevs");
		goto create_subdev_failed;
	}

	/* Using device ops query connected devs, prepare request tables */
	rc = __cam_req_mgr_setup_link_info(link, link_info);
	if (rc < 0)
		goto setup_failed;

	spin_lock_bh(&link->link_state_spin_lock);
	link->state = CAM_CRM_LINK_STATE_READY;
	spin_unlock_bh(&link->link_state_spin_lock);

	/* Create worker for current link */
#ifndef OPLUS_FEATURE_CAMERA_COMMON
	snprintf(buf, sizeof(buf), "%x-%x",
		link_info->u.link_info_v1.session_hdl, link->link_hdl);
#else
	snprintf(buf, sizeof(buf), "CRMCORE_%x-%x",
			link_info->u.link_info_v1.session_hdl, link->link_hdl);
#endif
	wq_flag = CAM_WORKQ_FLAG_HIGH_PRIORITY | CAM_WORKQ_FLAG_SERIAL;
	rc = cam_req_mgr_workq_create(buf, CRM_WORKQ_NUM_TASKS,
		&link->workq, CRM_WORKQ_USAGE_NON_IRQ, wq_flag,
		cam_req_mgr_process_workq_link_worker);
	if (rc < 0) {
		CAM_ERR(CAM_CRM, "FATAL: unable to create worker");
		__cam_req_mgr_destroy_link_info(link);
		goto setup_failed;
	}

	/* Assign payload to workqueue tasks */
	rc = __cam_req_mgr_setup_payload(link->workq);
	if (rc < 0) {
		__cam_req_mgr_destroy_link_info(link);
		cam_req_mgr_workq_destroy(&link->workq);
		goto setup_failed;
	}

	mutex_unlock(&link->lock);
	mutex_unlock(&g_crm_core_dev->crm_lock);
	return rc;
setup_failed:
	__cam_req_mgr_destroy_subdev(&link->l_dev);
create_subdev_failed:
	cam_destroy_link_hdl(link->link_hdl);
	link_info->u.link_info_v1.link_hdl = -1;
link_hdl_fail:
	mutex_unlock(&link->lock);
	__cam_req_mgr_unreserve_link(cam_session, link);
	mutex_unlock(&g_crm_core_dev->crm_lock);
	return rc;
}

int cam_req_mgr_link_v2(struct cam_req_mgr_ver_info *link_info)
{
	int                                     rc = 0;
	int                                     wq_flag = 0;
	char                                    buf[128];
	struct cam_create_dev_hdl               root_dev = {0};
	struct cam_req_mgr_core_session        *cam_session;
	struct cam_req_mgr_core_link           *link;

	if (!link_info) {
		CAM_DBG(CAM_CRM, "NULL pointer");
		return -EINVAL;
	}
	if (link_info->u.link_info_v2.num_devices >
		CAM_REQ_MGR_MAX_HANDLES_V2) {
		CAM_ERR(CAM_CRM, "Invalid num devices %d",
			link_info->u.link_info_v2.num_devices);
		return -EINVAL;
	}

	mutex_lock(&g_crm_core_dev->crm_lock);

	/* session hdl's priv data is cam session struct */
	cam_session = cam_get_session_priv(link_info->u.link_info_v2.session_hdl);
	if (!cam_session || (cam_session->session_hdl != link_info->u.link_info_v2.session_hdl)) {
		CAM_ERR(CAM_CRM, "session: %s, link_info->ses_hdl:%x, session->ses_hdl:%x",
			CAM_IS_NULL_TO_STR(cam_session), link_info->u.link_info_v2.session_hdl,
			(!cam_session) ? CAM_REQ_MGR_DEFAULT_HDL_VAL : cam_session->session_hdl);
		mutex_unlock(&g_crm_core_dev->crm_lock);
		return -EINVAL;
	}

	/* Allocate link struct and map it with session's request queue */
	link = __cam_req_mgr_reserve_link(cam_session);
	if (!link) {
		CAM_ERR(CAM_CRM, "failed to reserve new link");
		mutex_unlock(&g_crm_core_dev->crm_lock);
		return -EINVAL;
	}
	CAM_DBG(CAM_CRM, "link reserved %pK %x", link, link->link_hdl);

	root_dev.session_hdl = link_info->u.link_info_v2.session_hdl;
	root_dev.priv = (void *)link;
	root_dev.dev_id = CAM_CRM;

	mutex_lock(&link->lock);
	/* Create unique handle for link */
	link->link_hdl = cam_create_link_hdl(&root_dev);
	if (link->link_hdl < 0) {
		CAM_ERR(CAM_CRM,
			"Insufficient memory to create new device handle");
		rc = link->link_hdl;
		goto link_hdl_fail;
	}
	link_info->u.link_info_v2.link_hdl = link->link_hdl;
	link->last_flush_id = 0;

	/* Allocate memory to hold data of all linked devs */
	rc = __cam_req_mgr_create_subdevs(&link->l_dev,
		link_info->u.link_info_v2.num_devices);
	if (rc < 0) {
		CAM_ERR(CAM_CRM,
			"Insufficient memory to create new crm subdevs");
		goto create_subdev_failed;
	}

	/* Using device ops query connected devs, prepare request tables */
	rc = __cam_req_mgr_setup_link_info(link, link_info);
	if (rc < 0)
		goto setup_failed;

	spin_lock_bh(&link->link_state_spin_lock);
	link->state = CAM_CRM_LINK_STATE_READY;
	spin_unlock_bh(&link->link_state_spin_lock);

	/* Create worker for current link */
#ifndef OPLUS_FEATURE_CAMERA_COMMON
	snprintf(buf, sizeof(buf), "%x-%x",
		link_info->u.link_info_v2.session_hdl, link->link_hdl);
#else
	snprintf(buf, sizeof(buf), "CRMCORE_%x-%x",
		link_info->u.link_info_v2.session_hdl, link->link_hdl);
#endif
	wq_flag = CAM_WORKQ_FLAG_HIGH_PRIORITY | CAM_WORKQ_FLAG_SERIAL;
	rc = cam_req_mgr_workq_create(buf, CRM_WORKQ_NUM_TASKS,
		&link->workq, CRM_WORKQ_USAGE_NON_IRQ, wq_flag,
		cam_req_mgr_process_workq_link_worker);
	if (rc < 0) {
		CAM_ERR(CAM_CRM, "FATAL: unable to create worker");
		__cam_req_mgr_destroy_link_info(link);
		goto setup_failed;
	}

	/* Assign payload to workqueue tasks */
	rc = __cam_req_mgr_setup_payload(link->workq);
	if (rc < 0) {
		__cam_req_mgr_destroy_link_info(link);
		cam_req_mgr_workq_destroy(&link->workq);
		goto setup_failed;
	}

	link->trigger_cnt[0][CAM_TRIGGER_POINT_SOF] = 0;
	link->trigger_cnt[0][CAM_TRIGGER_POINT_EOF] = 0;
	link->trigger_cnt[1][CAM_TRIGGER_POINT_SOF] = 0;
	link->trigger_cnt[1][CAM_TRIGGER_POINT_EOF] = 0;

	mutex_unlock(&link->lock);
	mutex_unlock(&g_crm_core_dev->crm_lock);
	return rc;
setup_failed:
	__cam_req_mgr_destroy_subdev(&link->l_dev);
create_subdev_failed:
	cam_destroy_link_hdl(link->link_hdl);
	link_info->u.link_info_v2.link_hdl = -1;
link_hdl_fail:
	mutex_unlock(&link->lock);
	__cam_req_mgr_unreserve_link(cam_session, link);
	mutex_unlock(&g_crm_core_dev->crm_lock);
	return rc;
}


int cam_req_mgr_unlink(struct cam_req_mgr_unlink_info *unlink_info)
{
	int                              rc = 0;
	struct cam_req_mgr_core_session *cam_session;
	struct cam_req_mgr_core_link    *link;

	if (!unlink_info) {
		CAM_ERR(CAM_CRM, "NULL pointer");
		return -EINVAL;
	}

	mutex_lock(&g_crm_core_dev->crm_lock);
	CAM_DBG(CAM_CRM, "link_hdl %x", unlink_info->link_hdl);

	/* session hdl's priv data is cam session struct */
	cam_session = cam_get_session_priv(unlink_info->session_hdl);
	if (!cam_session || (cam_session->session_hdl != unlink_info->session_hdl)) {
		CAM_ERR(CAM_CRM, "session: %s, unlink_info->ses_hdl:%x, cam_session->ses_hdl:%x",
			CAM_IS_NULL_TO_STR(cam_session), unlink_info->session_hdl,
			(!cam_session) ? CAM_REQ_MGR_DEFAULT_HDL_VAL : cam_session->session_hdl);
		mutex_unlock(&g_crm_core_dev->crm_lock);
		return -EINVAL;
	}

	/* link hdl's priv data is core_link struct */
	link = cam_get_link_priv(unlink_info->link_hdl);
	if (!link || (link->link_hdl != unlink_info->link_hdl)) {
		CAM_ERR(CAM_CRM, "link: %s, unlink_info->link_hdl:%x, link->link_hdl:%x",
			CAM_IS_NULL_TO_STR(link), unlink_info->link_hdl,
			(!link) ? CAM_REQ_MGR_DEFAULT_HDL_VAL : link->link_hdl);
		rc = -EINVAL;
		goto done;
	}
	mutex_lock(&cam_session->lock);
	rc = __cam_req_mgr_unlink(cam_session, link);
	mutex_unlock(&cam_session->lock);

	/* Free curent link and put back into session's free pool of links */
	__cam_req_mgr_unreserve_link(cam_session, link);

done:
	mutex_unlock(&g_crm_core_dev->crm_lock);
	return rc;
}

int cam_req_mgr_schedule_request(
	struct cam_req_mgr_sched_request *sched_req)
{
	int                                  rc = 0;
	struct cam_req_mgr_core_link        *link = NULL;
	struct cam_req_mgr_core_session     *session = NULL;
	struct cam_req_mgr_core_sched_req    sched;

	if (!sched_req) {
		CAM_ERR(CAM_CRM, "csl_req is NULL");
		return -EINVAL;
	}

	mutex_lock(&g_crm_core_dev->crm_lock);
	link = cam_get_link_priv(sched_req->link_hdl);
	if (!link || (link->link_hdl != sched_req->link_hdl)) {
		CAM_ERR(CAM_CRM, "link: %s, sched_req->link_hdl:%x, link->link_hdl:%x",
			CAM_IS_NULL_TO_STR(link), sched_req->link_hdl,
			(!link) ? CAM_REQ_MGR_DEFAULT_HDL_VAL : link->link_hdl);
		rc = -EINVAL;
		goto end;
	}

	session = (struct cam_req_mgr_core_session *)link->parent;
	if (!session) {
		CAM_WARN(CAM_CRM, "session ptr NULL %x", sched_req->link_hdl);
		rc = -EINVAL;
		goto end;
	}

	if (sched_req->req_id <= link->last_flush_id) {
		CAM_INFO(CAM_CRM,
			"request %lld is flushed, last_flush_id to flush %d",
			sched_req->req_id, link->last_flush_id);
		rc = -EBADR;
		goto end;
	}

	if (sched_req->req_id > link->last_flush_id)
		link->last_flush_id = 0;

	CAM_DBG(CAM_CRM, "link 0x%x req %lld, sync_mode %d",
		sched_req->link_hdl, sched_req->req_id, sched_req->sync_mode);

	memset(&sched, 0, sizeof(sched));

	sched.req_id = sched_req->req_id;
	sched.sync_mode = sched_req->sync_mode;
	sched.link_hdl = sched_req->link_hdl;
	sched.additional_timeout = sched_req->additional_timeout;
	if (session->force_err_recovery == AUTO_RECOVERY) {
		sched.bubble_enable = sched_req->bubble_enable;
	} else {
		sched.bubble_enable =
		(session->force_err_recovery == FORCE_ENABLE_RECOVERY) ? 1 : 0;
	}

	rc = cam_req_mgr_process_sched_req(link, &sched);

	CAM_DBG(CAM_REQ, "Open req %lld on link 0x%x with sync_mode %d",
		sched_req->req_id, sched_req->link_hdl, sched_req->sync_mode);
end:
	mutex_unlock(&g_crm_core_dev->crm_lock);
	return rc;
}

int cam_req_mgr_schedule_request_v2(
	struct cam_req_mgr_sched_request_v2 *sched_req)
{
	int                                  i = 0;
	int                                  rc = 0;
	struct cam_req_mgr_core_link        *link = NULL;
	struct cam_req_mgr_core_session     *session = NULL;
	struct cam_req_mgr_core_sched_req    sched;
	struct cam_req_mgr_core_link        *sync_links[MAXIMUM_LINKS_PER_SESSION];
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	char trace[64] = {0};
#endif

	mutex_lock(&g_crm_core_dev->crm_lock);
	link = cam_get_link_priv(sched_req->link_hdl);
	if (!link || (link->link_hdl != sched_req->link_hdl)) {
		CAM_ERR(CAM_CRM, "link: %s, sched_req->link_hdl:%x, link->link_hdl:%x",
			CAM_IS_NULL_TO_STR(link), sched_req->link_hdl,
			(!link) ? CAM_REQ_MGR_DEFAULT_HDL_VAL : link->link_hdl);
		rc = -EINVAL;
		goto end;
	}

	session = (struct cam_req_mgr_core_session *)link->parent;
	if (!session) {
		CAM_WARN(CAM_CRM, "session ptr NULL %x", sched_req->link_hdl);
		rc = -EINVAL;
		goto end;
	}

	if (sched_req->req_id <= link->last_flush_id) {
		CAM_INFO(CAM_CRM,
			"request %lld is flushed, last_flush_id to flush %d",
			sched_req->req_id, link->last_flush_id);
		rc = -EBADR;
		goto end;
	}

	if (sched_req->req_id > link->last_flush_id)
		link->last_flush_id = 0;

	CAM_DBG(CAM_CRM, "link 0x%x req %lld, sync_mode %d num_links %d",
		sched_req->link_hdl, sched_req->req_id, sched_req->sync_mode,
		sched_req->num_links);

	memset(&sched, 0, sizeof(sched));

	sched.req_id = sched_req->req_id;
	sched.sync_mode = sched_req->sync_mode;
	sched.link_hdl = sched_req->link_hdl;
	sched.additional_timeout = sched_req->additional_timeout;
	sched.num_valid_params = sched_req->num_valid_params;
	sched.param_mask = sched_req->param_mask;
	sched.params = sched_req->params;

	if (session->force_err_recovery == AUTO_RECOVERY) {
		sched.bubble_enable = sched_req->bubble_enable;
	} else {
		sched.bubble_enable =
		(session->force_err_recovery == FORCE_ENABLE_RECOVERY) ? 1 : 0;
	}

	if (sched_req->sync_mode == CAM_REQ_MGR_SYNC_MODE_SYNC) {
		if ((sched_req->num_links <= 0) ||
			(sched_req->num_links > MAXIMUM_LINKS_PER_SESSION)) {
			CAM_ERR(CAM_CRM, "link:0x%x req:%lld invalid num_links:%d",
				link->link_hdl, sched_req->req_id, sched_req->num_links);
			rc = -EINVAL;
			goto end;
		}

		for (i = 0; i < sched_req->num_links; i++) {
			if (!sched_req->link_hdls[i]) {
				CAM_ERR(CAM_CRM, "link handle %d in sched_req is null", i);
				rc = -EINVAL;
				goto end;
			}

			sync_links[i] = cam_get_link_priv(sched_req->link_hdls[i]);
			if (!sync_links[i] ||
				(sync_links[i]->link_hdl != sched_req->link_hdls[i])) {
				CAM_ERR(CAM_CRM,
					"Invalid sync link, sync link[%d]: %s sched_req->link_hdl: %x sync_links->link_hdl: 0x%x",
					i, CAM_IS_NULL_TO_STR(sync_links[i]),
					sched_req->link_hdls[i],
					((!sync_links[i]) ? CAM_REQ_MGR_DEFAULT_HDL_VAL :
					sync_links[i]->link_hdl));
				rc = -EINVAL;
				goto end;
			}
		}
		sched.num_links = sched_req->num_links;
		sched.link_hdls = sched_req->link_hdls;
	} else
		sched.num_links = 0;

	rc = cam_req_mgr_process_sched_req(link, &sched);

	CAM_DBG(CAM_REQ, "Open req %lld on link 0x%x with sync_mode %d",
		sched_req->req_id, sched_req->link_hdl, sched_req->sync_mode);

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	snprintf(trace, sizeof(trace), "KMD %d_1 OpenRequest", sched_req->link_hdl);
	trace_long(trace, sched_req->req_id);
	trace_int(trace, 0);
#endif

end:
	mutex_unlock(&g_crm_core_dev->crm_lock);
	return rc;
}

int cam_req_mgr_schedule_request_v3(
	struct cam_req_mgr_sched_request_v3 *sched_req)
{
	int                                  i = 0;
	int                                  rc = 0;
	struct cam_req_mgr_core_link        *link = NULL;
	struct cam_req_mgr_core_session     *session = NULL;
	struct cam_req_mgr_core_sched_req    sched;
	struct cam_req_mgr_core_link        *sync_links[MAXIMUM_LINKS_PER_SESSION];
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	char trace[64] = {0};
#endif

	mutex_lock(&g_crm_core_dev->crm_lock);

	link = cam_get_link_priv(sched_req->link_hdl);
	if (!link || (link->link_hdl != sched_req->link_hdl)) {
		CAM_ERR(CAM_CRM, "link: %s, sched_req->link_hdl:%x, link->link_hdl:%x",
			CAM_IS_NULL_TO_STR(link), sched_req->link_hdl,
			(!link) ? CAM_REQ_MGR_DEFAULT_HDL_VAL : link->link_hdl);
		rc = -EINVAL;
		goto end;
	}

	session = (struct cam_req_mgr_core_session *)link->parent;
	if (!session) {
		CAM_WARN(CAM_CRM, "session ptr NULL %x", sched_req->link_hdl);
		rc = -EINVAL;
		goto end;
	}

	if (sched_req->req_id <= link->last_flush_id) {
		CAM_INFO(CAM_CRM,
			"request %lld is flushed, last_flush_id to flush %d",
			sched_req->req_id, link->last_flush_id);
		rc = -EBADR;
		goto end;
	}

	if (sched_req->req_id > link->last_flush_id)
		link->last_flush_id = 0;

	CAM_DBG(CAM_CRM, "link 0x%x req %lld, sync_mode %d num_links %d",
		sched_req->link_hdl, sched_req->req_id, sched_req->sync_mode,
		sched_req->num_links);

	memset(&sched, 0, sizeof(sched));

	sched.req_id = sched_req->req_id;
	sched.sync_mode = sched_req->sync_mode;
	sched.link_hdl = sched_req->link_hdl;
	sched.additional_timeout = sched_req->additional_timeout;
	sched.num_valid_params = sched_req->num_valid_params;
	sched.param_mask = sched_req->param_mask;
	sched.params = sched_req->params;

	if (session->force_err_recovery == AUTO_RECOVERY) {
		sched.bubble_enable = sched_req->bubble_enable;
	} else {
		sched.bubble_enable =
		(session->force_err_recovery == FORCE_ENABLE_RECOVERY) ? 1 : 0;
	}

	if (sched_req->sync_mode == CAM_REQ_MGR_SYNC_MODE_SYNC) {
		if ((sched_req->num_links <= 0) ||
			(sched_req->num_links > MAXIMUM_LINKS_PER_SESSION)) {
			CAM_ERR(CAM_CRM, "link:0x%x req:%lld invalid num_links:%d",
				link->link_hdl, sched_req->req_id, sched_req->num_links);
			rc = -EINVAL;
			goto end;
		}

		for (i = 0; i < sched_req->num_links; i++) {
			if (!sched_req->link_hdls[i]) {
				CAM_ERR(CAM_CRM, "link handle %d in sched_req is null", i);
				rc = -EINVAL;
				goto end;
			}

			sync_links[i] = cam_get_link_priv(sched_req->link_hdls[i]);
			if (!sync_links[i] ||
				(sync_links[i]->link_hdl != sched_req->link_hdls[i])) {
				CAM_ERR(CAM_CRM,
					"Invalid sync link, sync link[%d]: %s sched_req->link_hdl: %x sync_links->link_hdl: 0x%x",
					i, CAM_IS_NULL_TO_STR(sync_links[i]),
					sched_req->link_hdls[i],
					((!sync_links[i]) ? CAM_REQ_MGR_DEFAULT_HDL_VAL :
					sync_links[i]->link_hdl));
				rc = -EINVAL;
				goto end;
			}
		}
		sched.num_links = sched_req->num_links;
		sched.link_hdls = sched_req->link_hdls;
	} else
		sched.num_links = 0;

	rc = cam_req_mgr_process_sched_req(link, &sched);

	CAM_DBG(CAM_REQ, "Open req %lld on link 0x%x with sync_mode %d",
		sched_req->req_id, sched_req->link_hdl, sched_req->sync_mode);
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	snprintf(trace, sizeof(trace), "KMD %d_1 OpenRequest", sched_req->link_hdl);
	trace_long(trace, sched_req->req_id);
	trace_int(trace, 0);
#endif
end:
	mutex_unlock(&g_crm_core_dev->crm_lock);
	return rc;
}

int cam_req_mgr_sync_config(
	struct cam_req_mgr_sync_mode *sync_info)
{
	int                              i, j, rc = 0;
	int                              sync_idx = 0;
	struct cam_req_mgr_core_session *cam_session;
	struct cam_req_mgr_core_link    *link[MAX_LINKS_PER_SESSION];

	if (!sync_info) {
		CAM_ERR(CAM_CRM, "NULL pointer");
		return -EINVAL;
	}

	if ((sync_info->num_links < 0) ||
		(sync_info->num_links >
		MAX_LINKS_PER_SESSION)) {
		CAM_ERR(CAM_CRM, "Invalid num links %d", sync_info->num_links);
		return -EINVAL;
	}

	if ((sync_info->sync_mode != CAM_REQ_MGR_SYNC_MODE_SYNC) &&
		(sync_info->sync_mode != CAM_REQ_MGR_SYNC_MODE_NO_SYNC)) {
		CAM_ERR(CAM_CRM, "Invalid sync mode %d", sync_info->sync_mode);
		return -EINVAL;
	}

	if ((!sync_info->link_hdls[0]) || (!sync_info->link_hdls[1])) {
		CAM_WARN(CAM_CRM, "Invalid link handles 0x%x 0x%x",
			sync_info->link_hdls[0], sync_info->link_hdls[1]);
		return -EINVAL;
	}

	mutex_lock(&g_crm_core_dev->crm_lock);
	/* session hdl's priv data is cam session struct */
	cam_session = cam_get_session_priv(sync_info->session_hdl);
	if (!cam_session || (cam_session->session_hdl != sync_info->session_hdl)) {
		CAM_ERR(CAM_CRM, "session: %s, sync_info->session_hdl:%x, session->ses_hdl:%x",
			CAM_IS_NULL_TO_STR(cam_session), sync_info->session_hdl,
			(!cam_session) ? CAM_REQ_MGR_DEFAULT_HDL_VAL : cam_session->session_hdl);
		mutex_unlock(&g_crm_core_dev->crm_lock);
		return -EINVAL;
	}

	mutex_lock(&cam_session->lock);

	for (i = 0; i < sync_info->num_links; i++) {

		if (!sync_info->link_hdls[i]) {
			CAM_ERR(CAM_CRM, "link handle %d is null", i);
			rc = -EINVAL;
			goto done;
		}

		link[i] = cam_get_link_priv(sync_info->link_hdls[i]);
		if (!link[i] || (link[i]->link_hdl != sync_info->link_hdls[i])) {
			CAM_ERR(CAM_CRM, "link: %s, sync_info->link_hdl:%x, link->link_hdl:%x",
				CAM_IS_NULL_TO_STR(link), sync_info->link_hdls[i],
				(!link[i]) ? CAM_REQ_MGR_DEFAULT_HDL_VAL : link[i]->link_hdl);
			rc = -EINVAL;
			goto done;
		}

		link[i]->sync_link_sof_skip = false;
		link[i]->is_master = false;
		link[i]->in_msync_mode = false;
		link[i]->initial_sync_req = -1;
		link[i]->num_sync_links = 0;

		for (j = 0; j < sync_info->num_links-1; j++)
			link[i]->sync_link[j] = NULL;
	}

	if (sync_info->sync_mode == CAM_REQ_MGR_SYNC_MODE_SYNC) {
		for (i = 0; i < sync_info->num_links; i++) {
			j = 0;
			sync_idx = 0;
			CAM_DBG(CAM_REQ, "link %x adds sync link:",
				link[i]->link_hdl);
			while (j < sync_info->num_links) {
				if (i != j) {
					link[i]->sync_link[sync_idx++] =
						link[j];
					link[i]->num_sync_links++;
					CAM_DBG(CAM_REQ, "sync_link[%d] : %x",
						sync_idx-1, link[j]->link_hdl);
				}
				j++;
			}
			link[i]->initial_skip = true;
			link[i]->sof_timestamp = 0;
		}
	} else {
		for (j = 0; j < sync_info->num_links; j++) {
			link[j]->initial_skip = true;
			link[j]->sof_timestamp = 0;
		}
	}

	cam_session->sync_mode = sync_info->sync_mode;
	CAM_DBG(CAM_REQ,
		"Sync config completed on %d links with sync_mode %d",
		sync_info->num_links, sync_info->sync_mode);

done:
	mutex_unlock(&cam_session->lock);
	mutex_unlock(&g_crm_core_dev->crm_lock);
	return rc;
}

int cam_req_mgr_flush_requests(
	struct cam_req_mgr_flush_info *flush_info)
{
	int                               rc = 0;
	struct crm_workq_task            *task = NULL;
	struct cam_req_mgr_core_link     *link = NULL;
	struct cam_req_mgr_flush_info    *flush;
	struct crm_task_payload          *task_data;
	struct cam_req_mgr_core_session  *session = NULL;

	if (!flush_info) {
		CAM_ERR(CAM_CRM, "flush req is NULL");
		return -EFAULT;
	}
	if (flush_info->flush_type >= CAM_REQ_MGR_FLUSH_TYPE_MAX) {
		CAM_ERR(CAM_CRM, "incorrect flush type %x",
			flush_info->flush_type);
		return -EINVAL;
	}

	mutex_lock(&g_crm_core_dev->crm_lock);

	/* session hdl's priv data is cam session struct */
	session = cam_get_session_priv(flush_info->session_hdl);
	if (!session || (session->session_hdl != flush_info->session_hdl)) {
		CAM_ERR(CAM_CRM, "session: %s, flush_info->ses_hdl:%x, session->ses_hdl:%x",
			CAM_IS_NULL_TO_STR(session), flush_info->session_hdl,
			(!session) ? CAM_REQ_MGR_DEFAULT_HDL_VAL : session->session_hdl);
		rc = -EINVAL;
		goto end;
	}
	if (session->num_links <= 0) {
		CAM_WARN(CAM_CRM, "No active links in session %x",
		flush_info->session_hdl);
		goto end;
	}

	link = cam_get_link_priv(flush_info->link_hdl);
	if (!link || (link->link_hdl != flush_info->link_hdl)) {
		CAM_ERR(CAM_CRM, "link: %s, flush_info->link_hdl:%x, link->link_hdl:%x",
			CAM_IS_NULL_TO_STR(link), flush_info->link_hdl,
			(!link) ? CAM_REQ_MGR_DEFAULT_HDL_VAL : link->link_hdl);
		rc = -EINVAL;
		goto end;
	}

	task = cam_req_mgr_workq_get_task(link->workq);
	if (!task) {
		rc = -ENOMEM;
		goto end;
	}

	task_data = (struct crm_task_payload *)task->payload;
	task_data->type = CRM_WORKQ_TASK_FLUSH_REQ;
	flush = (struct cam_req_mgr_flush_info *)&task_data->u;
	flush->req_id = flush_info->req_id;
	flush->link_hdl = flush_info->link_hdl;
	flush->flush_type = flush_info->flush_type;
	flush->reserved = flush_info->reserved;
	task->process_cb = &cam_req_mgr_process_flush_req;
	init_completion(&link->workq_comp);
	rc = cam_req_mgr_workq_enqueue_task(task, link, CRM_TASK_PRIORITY_0);
	if (rc) {
		CAM_ERR(CAM_CRM, "Failed to queue flush task rc: %d", rc);
		goto end;
	}

	/* Blocking call */
	rc = cam_common_wait_for_completion_timeout(
		&link->workq_comp,
		msecs_to_jiffies(CAM_REQ_MGR_SCHED_REQ_TIMEOUT));
	if (!rc)
		CAM_WARN(CAM_CRM, "Flush call timeout for session_hdl %u link_hdl %u type: %d",
			flush_info->link_hdl, flush_info->session_hdl,
			flush_info->flush_type);
end:
	mutex_unlock(&g_crm_core_dev->crm_lock);
	return rc;
}

int cam_req_mgr_link_control(struct cam_req_mgr_link_control *control)
{
	int                                 rc = 0, i;
	struct cam_req_mgr_core_link       *link = NULL;
	int                                 init_timeout = 0;

	if (!control) {
		CAM_ERR(CAM_CRM, "Control command is NULL");
		rc = -EINVAL;
		goto end;
	}

	if ((control->num_links <= 0) ||
		(control->num_links > MAX_LINKS_PER_SESSION)) {
		CAM_ERR(CAM_CRM, "Invalid number of links %d",
			control->num_links);
		rc = -EINVAL;
		goto end;
	}

	mutex_lock(&g_crm_core_dev->crm_lock);
	for (i = 0; i < control->num_links; i++) {
		link = cam_get_link_priv(control->link_hdls[i]);
		if (!link || (link->link_hdl != control->link_hdls[i])) {
			CAM_ERR(CAM_CRM, "link: %s, control->link_hdl:%x, link->link_hdl:%x",
				CAM_IS_NULL_TO_STR(link), control->link_hdls[i],
				(!link) ? CAM_REQ_MGR_DEFAULT_HDL_VAL : link->link_hdl);
			rc = -EINVAL;
			break;
		}

		mutex_lock(&link->lock);
		if (control->ops == CAM_REQ_MGR_LINK_ACTIVATE) {
			spin_lock_bh(&link->link_state_spin_lock);
			link->state = CAM_CRM_LINK_STATE_READY;
			spin_unlock_bh(&link->link_state_spin_lock);
			if (control->init_timeout[i])
				link->skip_init_frame = true;
			init_timeout = (2 * control->init_timeout[i]);
			CAM_DBG(CAM_CRM,
				"Activate link: 0x%x init_timeout: %d ms",
				link->link_hdl, control->init_timeout[i]);
			/* Start SOF watchdog timer */
			rc = crm_timer_init(&link->watchdog,
				(init_timeout + CAM_REQ_MGR_WATCHDOG_TIMEOUT),
				link, &__cam_req_mgr_sof_freeze);
			if (rc < 0) {
				CAM_ERR(CAM_CRM,
					"SOF timer start fails: link=0x%x",
					link->link_hdl);
				rc = -EFAULT;
			}
			/* Wait for the streaming of sync link */
			link->initial_skip = true;
			/* Pause the timer before sensor stream on */
			spin_lock_bh(&link->link_state_spin_lock);
			link->watchdog->pause_timer = true;
			spin_unlock_bh(&link->link_state_spin_lock);
			/* notify nodes */
			__cam_req_mgr_send_evt(0, CAM_REQ_MGR_LINK_EVT_RESUME,
				CRM_KMD_ERR_MAX, link);
		} else if (control->ops == CAM_REQ_MGR_LINK_DEACTIVATE) {
			/* notify nodes */
			__cam_req_mgr_send_evt(0, CAM_REQ_MGR_LINK_EVT_PAUSE,
				CRM_KMD_ERR_MAX, link);

			/* Destroy SOF watchdog timer */
			spin_lock_bh(&link->link_state_spin_lock);
			link->state = CAM_CRM_LINK_STATE_IDLE;
			link->skip_init_frame = false;
			crm_timer_exit(&link->watchdog);
			spin_unlock_bh(&link->link_state_spin_lock);
			CAM_DBG(CAM_CRM,
				"De-activate link: 0x%x", link->link_hdl);
		} else {
			CAM_ERR(CAM_CRM, "Invalid link control command");
			rc = -EINVAL;
		}
		mutex_unlock(&link->lock);
	}
	mutex_unlock(&g_crm_core_dev->crm_lock);
end:
	return rc;
}

int cam_req_mgr_link_properties(struct cam_req_mgr_link_properties *properties)
{
	int                                    rc = 0;
	struct cam_req_mgr_core_link          *link = NULL;

	mutex_lock(&g_crm_core_dev->crm_lock);
	link = cam_get_link_priv(properties->link_hdl);
	if (!link || (link->link_hdl != properties->link_hdl)) {
		CAM_ERR(CAM_CRM, "link: %s, properties->link_hdl:0x%x, link->link_hdl:0x%x",
			CAM_IS_NULL_TO_STR(link), properties->link_hdl,
			(!link) ? CAM_REQ_MGR_DEFAULT_HDL_VAL : link->link_hdl);
		rc = -EINVAL;
		goto end;
	}

	spin_lock_bh(&link->link_state_spin_lock);
	if (link->state != CAM_CRM_LINK_STATE_READY) {
		spin_unlock_bh(&link->link_state_spin_lock);
		CAM_ERR(CAM_CRM,
			"Only can config link 0x%x properties in ready state",
			link->link_hdl);
		rc = -EAGAIN;
		goto end;
	}
	spin_unlock_bh(&link->link_state_spin_lock);

	mutex_lock(&link->lock);
	link->properties_mask = properties->properties_mask;

	rc = __cam_req_mgr_send_evt(0, CAM_REQ_MGR_LINK_EVT_UPDATE_PROPERTIES,
				CRM_KMD_ERR_MAX, link);
	if (rc)
		CAM_ERR(CAM_CRM,
			"Failed to set properties on link 0x%x", link->link_hdl);
	else
		CAM_DBG(CAM_CRM, "link 0x%x set properties mask:0x%x successfully",
			link->link_hdl, link->properties_mask);

	mutex_unlock(&link->lock);
end:
	mutex_unlock(&g_crm_core_dev->crm_lock);
	return rc;
}

static void *cam_req_mgr_user_dump_state_monitor_array_info(
	void *dump_struct, uint8_t *addr_ptr)
{
	struct cam_req_mgr_state_monitor  *evt = NULL;
	uint64_t                          *addr;
	struct cam_common_hw_dump_header  *hdr;
	uint8_t                           *dst;

	evt = (struct cam_req_mgr_state_monitor *)dump_struct;

	addr = (uint64_t *)addr_ptr;

	*addr++ = evt->req_id;

	dst = (uint8_t *)addr;
	hdr = (struct cam_common_hw_dump_header *)dst;
	scnprintf(hdr->tag, CAM_CRM_DUMP_TAG_MAX_LEN,
		"%s", evt->name);
	hdr->word_size = sizeof(uint64_t);
	addr = (uint64_t *)(dst + sizeof(struct cam_common_hw_dump_header));

	dst = (uint8_t *)addr;
	hdr = (struct cam_common_hw_dump_header *)dst;
	scnprintf(hdr->tag, CAM_CRM_DUMP_TAG_MAX_LEN,
		"%s", __cam_req_mgr_operation_type_to_str(evt->req_state));
	hdr->word_size = sizeof(uint64_t);
	addr = (uint64_t *)(dst + sizeof(struct cam_common_hw_dump_header));

	*addr++ = evt->dev_hdl;
	*addr++ = evt->frame_id;
	*addr++ = evt->time_stamp.tv_sec;
	*addr++ = evt->time_stamp.tv_nsec / NSEC_PER_USEC;
	return addr;
}

/**
 * cam_req_mgr_dump_state_monitor_info()
 *
 * @brief     : dump the state monitor array, dump from index next_wr_idx
 *             to save state information in time order.
 * @link      : link pointer
 * @dump_info : dump payload
 */
static int cam_req_mgr_dump_state_monitor_info(
	struct cam_req_mgr_core_link *link,
	struct cam_req_mgr_dump_info *dump_info)
{
	int                             rc = 0;
	int                             i;
	int                             idx = link->req.next_state_idx;
	struct cam_common_hw_dump_args  dump_args;
	size_t                          buf_len;
	size_t                          remain_len;
	uint32_t                        min_len;
	uintptr_t                       cpu_addr;

	rc = cam_mem_get_cpu_buf(dump_info->buf_handle,
		&cpu_addr, &buf_len);
	if (rc) {
		CAM_ERR(CAM_CRM, "Invalid handle %u rc %d",
			dump_info->buf_handle, rc);
		cam_mem_put_cpu_buf(dump_info->buf_handle);
		return rc;
	}
	if (buf_len <= dump_info->offset) {
		CAM_WARN(CAM_CRM, "Dump buffer overshoot len %zu offset %zu",
			buf_len, dump_info->offset);
		cam_mem_put_cpu_buf(dump_info->buf_handle);
		return -ENOSPC;
	}

	remain_len = buf_len - dump_info->offset;
	min_len = sizeof(struct cam_common_hw_dump_header) +
		(MAX_REQ_STATE_MONITOR_NUM * (CAM_CRM_DUMP_EVENT_NUM_WORDS * sizeof(uint64_t) +
		sizeof(struct cam_common_hw_dump_header) * CAM_CRM_DUMP_EVENT_NUM_HEADERS));

	if (remain_len < min_len) {
		CAM_WARN(CAM_CRM, "Dump buffer exhaust remain %zu min %u",
			remain_len, min_len);
		cam_mem_put_cpu_buf(dump_info->buf_handle);
		return -ENOSPC;
	}

	dump_args.req_id = dump_info->req_id;
	dump_args.cpu_addr = cpu_addr;
	dump_args.buf_len = buf_len;
	dump_args.offset = dump_info->offset;
	dump_args.ctxt_to_hw_map = NULL;

	for (i = 0; i < MAX_REQ_STATE_MONITOR_NUM; i++) {
		if (link->req.state_monitor[idx].req_state != CAM_CRM_STATE_INVALID) {
			rc = cam_common_user_dump_helper(&dump_args,
				cam_req_mgr_user_dump_state_monitor_array_info,
				&link->req.state_monitor[idx],
				sizeof(uint64_t), "CRM_STATE_MONITOR.%x:", link->link_hdl);
			if (rc) {
				CAM_ERR(CAM_CRM,
					"Dump state info failed, rc: %d",
					rc);
				cam_mem_put_cpu_buf(dump_info->buf_handle);
				return rc;
			}
		}
		__cam_req_mgr_inc_idx(&idx, 1, MAX_REQ_STATE_MONITOR_NUM);
	}

	dump_info->offset = dump_args.offset;
	cam_mem_put_cpu_buf(dump_info->buf_handle);
	return rc;
}

int cam_req_mgr_dump_request(struct cam_dump_req_cmd *dump_req)
{
	int                                  rc = 0;
	int                                  i;
	struct cam_req_mgr_dump_info         info;
	struct cam_req_mgr_core_link        *link = NULL;
	struct cam_req_mgr_core_session     *session = NULL;
	struct cam_req_mgr_connected_device *device = NULL;

	if (!dump_req) {
		CAM_ERR(CAM_CRM, "dump req is NULL");
		return -EFAULT;
	}

	mutex_lock(&g_crm_core_dev->crm_lock);
	/* session hdl's priv data is cam session struct */
	session = cam_get_session_priv(dump_req->session_handle);
	if (!session || (session->session_hdl != dump_req->session_handle)) {
		CAM_ERR(CAM_CRM, "session: %s, dump_req->ses_hdl:%x, session->ses_hdl:%x",
			CAM_IS_NULL_TO_STR(session), dump_req->session_handle,
			(!session) ? CAM_REQ_MGR_DEFAULT_HDL_VAL : session->session_hdl);
		rc = -EINVAL;
		goto end;
	}
	if (session->num_links <= 0) {
		CAM_WARN(CAM_CRM, "No active links in session %x",
			dump_req->session_handle);
		goto end;
	}

	link = cam_get_link_priv(dump_req->link_hdl);
	if (!link || (link->link_hdl != dump_req->link_hdl)) {
		CAM_ERR(CAM_CRM, "link: %s, dump_req->link_hdl:%x, link->link_hdl:%x",
			CAM_IS_NULL_TO_STR(link), dump_req->link_hdl,
			(!link) ? CAM_REQ_MGR_DEFAULT_HDL_VAL : link->link_hdl);
		rc = -EINVAL;
		goto end;
	}

	/*
	 * Dump Mem MGR
	 */
	rc = cam_mem_mgr_dump_user(dump_req);
	if (rc)
		CAM_ERR(CAM_REQ,
			"Fail to dump mem mgr rc %d", rc);

	/*
	 * Dump CRM
	 */
	info.offset = dump_req->offset;
	info.link_hdl = dump_req->link_hdl;
	info.dev_hdl = 0;
	info.req_id = dump_req->issue_req_id;
	info.buf_handle = dump_req->buf_handle;
	info.error_type = dump_req->error_type;
	rc =  cam_req_mgr_dump_state_monitor_info(link, &info);
	if (rc)
		CAM_ERR(CAM_REQ,
			"Fail to dump state monitor req %llu rc %d",
			info.req_id, rc);

	info.buf_handle = dump_req->buf_handle;
	rc = cam_cpas_dump_state_monitor_info(&info);
	if (rc)
		CAM_ERR(CAM_CRM, "Fail to dump cpas vote info, rc %d", rc);

	for (i = 0; i < link->num_devs; i++) {
		device = &link->l_dev[i];
		info.link_hdl = dump_req->link_hdl;
		info.dev_hdl = device->dev_hdl;
		info.req_id = dump_req->issue_req_id;
		info.buf_handle = dump_req->buf_handle;
		info.error_type = dump_req->error_type;
		if (device->ops && device->ops->dump_req) {
			rc = device->ops->dump_req(&info);
			if (rc)
				CAM_ERR(CAM_REQ,
					"Fail dump req %llu dev %d rc %d",
					info.req_id, device->dev_hdl, rc);
		}
	}
	dump_req->offset = info.offset;
	CAM_INFO(CAM_REQ, "req %llu, offset %zu",
		dump_req->issue_req_id, dump_req->offset);
end:
	mutex_unlock(&g_crm_core_dev->crm_lock);
	return rc;
}

static unsigned long cam_req_mgr_core_mini_dump_cb(void *dst, unsigned long len,
	void *priv_data)
{
	struct cam_req_mgr_core_link *link;
	struct cam_req_mgr_core_mini_dump *md;
	struct cam_req_mgr_core_link_mini_dump *md_link;
	struct cam_req_mgr_req_tbl   *l_tbl = NULL;
	uint8_t *waddr;
	unsigned long dumped_len = 0;
	unsigned long remain_len = len;
	uint32_t i = 0, j = 0, k = 0;

	if (!dst || len < sizeof(*md)) {
		CAM_ERR(CAM_CRM, "Insufficient received length %lu dumped len: %u",
			len, dumped_len);
		return 0;
	}

	md = (struct cam_req_mgr_core_mini_dump *)dst;
	md->num_link = 0;
	waddr = (uint8_t *)dst + sizeof(*md);
	dumped_len += sizeof(*md);
	remain_len -= dumped_len;

	for (i = 0; i < MAXIMUM_LINKS_CAPACITY; i++) {
		if (remain_len < sizeof(*md_link)) {
			CAM_ERR(CAM_CRM,
			"Insufficent received length: %lu, dumped_len %lu",
			len, dumped_len);
			goto end;
		}

		if (atomic_read(&g_links[i].is_used) == 0)
			continue;

		waddr += (k * sizeof(*md_link));
		md_link = (struct cam_req_mgr_core_link_mini_dump *)waddr;
		md->link[k] = md_link;
		link = &g_links[i];
		md_link->sync_link_sof_skip = link->sync_link_sof_skip;
		md_link->num_sync_links = link->num_sync_links;
		md_link->initial_sync_req = link->initial_sync_req;
		md_link->last_flush_id = link->last_flush_id;
		md_link->is_used = atomic_read(&link->is_used);
		md_link->retry_cnt = link->retry_cnt;
		md_link->eof_event_cnt = atomic_read(&link->eof_event_cnt);
		md_link->pd_mask = link->pd_mask;
		md_link->link_hdl = link->link_hdl;
		md_link->num_devs = link->num_devs;
		md_link->max_delay = link->max_delay;
		md_link->dual_trigger = link->dual_trigger;
		md_link->state = link->state;
		md_link->is_shutdown = link->is_shutdown;
		md_link->skip_init_frame = link->skip_init_frame;
		md_link->is_master = link->is_master;
		md_link->initial_skip = link->initial_skip;
		md_link->in_msync_mode = link->in_msync_mode;
		md_link->wq_congestion = link->wq_congestion;
		memcpy(md_link->req.apply_data, link->req.apply_data,
			sizeof(link->req.apply_data));
		memcpy(md_link->req.prev_apply_data, link->req.prev_apply_data,
			sizeof(link->req.prev_apply_data));
		memcpy(&md_link->req.in_q, link->req.in_q,
			sizeof(struct cam_req_mgr_req_queue));
		md_link->req.num_tbl = link->req.num_tbl;

		md_link->workq.workq_scheduled_ts =
					    link->workq->workq_scheduled_ts;
		md_link->workq.task.pending_cnt =
				atomic_read(&link->workq->task.pending_cnt);
		md_link->workq.task.free_cnt =
				atomic_read(&link->workq->task.free_cnt);
		md_link->workq.task.num_task = link->workq->task.num_task;

		l_tbl = link->req.l_tbl;
		j = 0;
		while(l_tbl) {
			md_link->req.l_tbl[j].id = l_tbl->id;
			md_link->req.l_tbl[j].pd = l_tbl->id;
			md_link->req.l_tbl[j].dev_count = l_tbl->dev_count;
			md_link->req.l_tbl[j].dev_mask = l_tbl->dev_mask;
			md_link->req.l_tbl[j].skip_traverse =
				l_tbl->skip_traverse;
			md_link->req.l_tbl[j].pd_delta = l_tbl->pd_delta;
			md_link->req.l_tbl[j].num_slots = l_tbl->num_slots;
			memcpy(md_link->req.l_tbl[j].slot, l_tbl->slot,
				sizeof(l_tbl->slot));
			l_tbl = l_tbl->next;
			j++;
		}
		md->num_link++;
		dumped_len += sizeof(*md_link);
		remain_len = len - dumped_len;
	}
end:
	return dumped_len;
}

void cam_req_mgr_dump_linked_devices_on_err(int32_t link_hdl)
{
	int                                  i, log_buf_size, buf_used = 0;
	struct cam_req_mgr_connected_device *dev;
	struct cam_req_mgr_core_link        *link;
	struct cam_req_mgr_core_session     *session;
	char                                 log_buf[CAM_CRM_DUMP_LINKED_DEVICES_MAX_LEN];

	link = cam_get_link_priv(link_hdl);
	if (!link || link->link_hdl != link_hdl) {
		CAM_DBG(CAM_CRM, "Invalid link hdl 0x%x", link_hdl);
		return;
	}

	session = (struct cam_req_mgr_core_session *)link->parent;
	for (i = 0; i < link->num_devs; i++) {
		dev = &link->l_dev[i];

		log_buf_size = CAM_CRM_DUMP_LINKED_DEVICES_MAX_LEN - buf_used;
		buf_used += snprintf(log_buf + buf_used, log_buf_size, " %s",
			dev->dev_info.name);
	}

	CAM_INFO(CAM_CRM, "Connected devices on the link 0x%x in session 0x%x:%s",
		link->link_hdl, session->session_hdl, log_buf);
}

int cam_req_mgr_core_device_init(void)
{
	int i;
	CAM_DBG(CAM_CRM, "Enter g_crm_core_dev %pK", g_crm_core_dev);

	if (g_crm_core_dev) {
		CAM_WARN(CAM_CRM, "core device is already initialized");
		return 0;
	}
	g_crm_core_dev = CAM_MEM_ZALLOC(sizeof(*g_crm_core_dev),
		GFP_KERNEL);
	if (!g_crm_core_dev)
		return -ENOMEM;

	CAM_DBG(CAM_CRM, "g_crm_core_dev %pK", g_crm_core_dev);
	INIT_LIST_HEAD(&g_crm_core_dev->session_head);
	mutex_init(&g_crm_core_dev->crm_lock);
	cam_req_mgr_debug_register(g_crm_core_dev);

	for (i = 0; i < MAXIMUM_LINKS_CAPACITY; i++) {
		mutex_init(&g_links[i].lock);
		spin_lock_init(&g_links[i].link_state_spin_lock);
		spin_lock_init(&g_links[i].req.monitor_slock);
		spin_lock_init(&g_links[i].req.reset_link_spin_lock);
		atomic_set(&g_links[i].is_used, 0);
		cam_req_mgr_core_link_reset(&g_links[i]);
	}
	cam_common_register_mini_dump_cb(cam_req_mgr_core_mini_dump_cb,
		"CAM_CRM", NULL);

	return 0;
}

int cam_req_mgr_core_device_deinit(void)
{
	if (!g_crm_core_dev) {
		CAM_ERR(CAM_CRM, "NULL pointer");
		return -EINVAL;
	}

	CAM_DBG(CAM_CRM, "g_crm_core_dev %pK", g_crm_core_dev);
	cam_req_mgr_debug_unregister();
	mutex_destroy(&g_crm_core_dev->crm_lock);
	CAM_MEM_FREE(g_crm_core_dev);
	g_crm_core_dev = NULL;

	return 0;
}