// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023-2023 Oplus. All rights reserved.
 */

/* charge protocol arbitration */

#define pr_fmt(fmt) "[CPA]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/jiffies.h>
#include <linux/wait.h>

#include <oplus_chg.h>
#include <oplus_chg_module.h>
#include <oplus_mms.h>
#include <oplus_mms_wired.h>
#include <oplus_chg_cpa.h>
#include <oplus_chg_vooc.h>
#include <oplus_chg_wired.h>
#include <oplus_chg_ufcs.h>
#include <oplus_chg_pps.h>
#include <oplus_chg_voter.h>
#include <oplus_chg_state_retention.h>
#include <recovery/state_keep.h>
#include <oplus_mms_gauge.h>

#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
#include "oplus_cfg.h"
#endif

#define PROTOCAL_SWITCH_REPLY_TIMEOUT_MS	1000
#define PROTOCAL_READY_TIMEOUT_MS		200000
#define PROTOCAL_WAIT_TIMEOUT_MS 		1000
#define WIRED_PLUGOUT_TO_PRESENT_MS		2000

struct oplus_cpa_protocol_info {
	enum oplus_chg_protocol_type type;
	int power_mw;
	int max_power_mw;
	int max_power_dynamic;
};

struct oplus_cpa_protocol_wait_info {
	enum oplus_chg_protocol_type type;
	int time; /* ms */
};

#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
struct oplus_cpa_state_keep_info {
	bool wired_state_keep;
	bool state_keep_ready;
	int wired_type;
	int protocol_power[CHG_PROTOCOL_MAX];
	int protocol_max_power[CHG_PROTOCOL_MAX];
	uint32_t protocol_to_be_switched;
	unsigned long protocol_disable_mask;
	struct mutex ready_lock;
};
#endif

struct oplus_cpa {
	struct device *dev;
	struct oplus_mms *cpa_topic;
	struct oplus_mms *wired_topic;
	struct oplus_mms *vooc_topic;
	struct mms_subscribe *wired_subs;
	struct mms_subscribe *vooc_subs;
	struct oplus_mms *ufcs_topic;
	struct oplus_mms *pps_topic;
	struct mms_subscribe *ufcs_subs;
	struct mms_subscribe *pps_subs;
	struct oplus_mms *retention_topic;
	struct mms_subscribe *retention_subs;
	struct oplus_mms *keep_topic;
	struct mms_subscribe *keep_subs;

	struct votable *req_lock_votable;
	struct work_struct protocol_switch_work;
	struct work_struct chg_type_change_work;
	struct work_struct fast_chg_type_change_work;
	struct work_struct wired_offline_work;
	struct work_struct wired_online_work;
	struct work_struct switch_end_work;
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	struct work_struct power_change_work;
	struct work_struct state_keep_wired_offline_work;
#endif
	struct delayed_work protocol_switch_timeout_work;
	struct delayed_work protocol_ready_timeout_work;

#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
	struct oplus_cfg debug_cfg;
#endif

	enum oplus_chg_protocol_type current_protocol_type;
	uint32_t protocol_to_be_switched;
	unsigned long protocol_disable_mask;
	uint32_t default_protocol_type;
	unsigned long ready_protocol_type;
	uint32_t protocol_supported_type;
	struct oplus_cpa_protocol_info protocol_prio_table[CHG_PROTOCOL_MAX];
	bool def_req;
	bool request_pending;
	bool started;
	bool request_locked;
	bool status_reset;

	bool wired_online;
	int cc_detect;
	int wired_type;
	int wired_real_chg_type;

	bool vooc_started;
	bool vooc_online;
	bool vooc_charging;
	bool vooc_online_keep;
	unsigned int vooc_sid;

	bool ufcs_online;
	bool ufcs_charging;
	bool ufcs_oplus_adapter;
	u32 ufcs_adapter_id;

	bool pps_online;
	bool pps_online_keep;
	bool pps_charging;
	bool pps_oplus_adapter;

	bool retention_state;
	bool pre_retention_state;
	bool retention_state_ready;

	bool wired_present;
	unsigned long wired_plugout_time;

	struct mutex cpa_request_lock;
	struct mutex start_lock;

	uint8_t region_id;

	bool pd_comleted;
	bool protocol_wait_support;
	bool bc12_completed;
	u32 bc12_check_timeout_ms;
	int protocol_wait_cnt;
	struct completion pd_completed_ack;
	wait_queue_head_t bc12_wq;
	struct oplus_cpa_protocol_wait_info protocol_wait_table[CHG_PROTOCOL_MAX];

#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	struct oplus_cpa_state_keep_info keep;
#endif
};

const char * const protocol_name_str[] = {
	[CHG_PROTOCOL_BC12]	= "BC1.2",
	[CHG_PROTOCOL_PD]	= "PD",
	[CHG_PROTOCOL_PPS]	= "PPS",
	[CHG_PROTOCOL_VOOC]	= "VOOC",
	[CHG_PROTOCOL_UFCS]	= "UFCS",
	[CHG_PROTOCOL_QC]	= "QC",
};

const char *get_protocol_name_str(enum oplus_chg_protocol_type type)
{
	if (type < 0 || type >= CHG_PROTOCOL_MAX)
		return "Unknown";
	return protocol_name_str[type];
}

static enum oplus_chg_protocol_type oplus_cpa_chg_type_to_protocol_type(int chg_type)
{
	switch (chg_type) {
	case OPLUS_CHG_USB_TYPE_QC2:
	case OPLUS_CHG_USB_TYPE_QC3:
		return CHG_PROTOCOL_QC;
	case OPLUS_CHG_USB_TYPE_PD:
	case OPLUS_CHG_USB_TYPE_PD_DRP:
		return CHG_PROTOCOL_PD;
	case OPLUS_CHG_USB_TYPE_PD_PPS:
		return CHG_PROTOCOL_PPS;
	case OPLUS_CHG_USB_TYPE_VOOC:
	case OPLUS_CHG_USB_TYPE_SVOOC:
		return CHG_PROTOCOL_VOOC;
	case OPLUS_CHG_USB_TYPE_UFCS:
		return CHG_PROTOCOL_UFCS;
	case OPLUS_CHG_USB_TYPE_SDP:
	case OPLUS_CHG_USB_TYPE_DCP:
	case OPLUS_CHG_USB_TYPE_CDP:
	case OPLUS_CHG_USB_TYPE_ACA:
	case OPLUS_CHG_USB_TYPE_C:
	case OPLUS_CHG_USB_TYPE_APPLE_BRICK_ID:
	case OPLUS_CHG_USB_TYPE_PD_SDP:
		return CHG_PROTOCOL_BC12;
	default:
		break;
	}

	return CHG_PROTOCOL_INVALID;
}

static enum oplus_chg_protocol_type get_highest_priority_protocol_type(struct oplus_cpa *cpa, uint32_t protocol)
{
	enum oplus_chg_protocol_type type;
	enum oplus_chg_protocol_type backup_type[CHG_PROTOCOL_MAX] = { CHG_PROTOCOL_INVALID };
	int backup_power[CHG_PROTOCOL_MAX] = { 0 };
	int i, m, n;

	for (i = 0; i < CHG_PROTOCOL_MAX; i++) {
		type = cpa->protocol_prio_table[i].type;
		chg_debug("type=%s, power=%d\n", get_protocol_name_str(type), cpa->protocol_prio_table[i].power_mw);
		if (cpa->protocol_prio_table[i].power_mw <= 0) {
			backup_type[i] = type;
			backup_power[i] = cpa->protocol_prio_table[i].power_mw;
			continue;
		}
		for (m = 0; m < i; m++) {
			if (backup_power[m] <= 0)
				continue;
			if (cpa->protocol_prio_table[i].power_mw <= backup_power[m])
				continue;
			for (n = i; n > m; n--) {
				backup_type[n] = backup_type[n - 1];
				backup_power[n] = backup_power[n - 1];
			}
			break;
		}
		backup_type[m] = type;
		backup_power[m] = cpa->protocol_prio_table[i].power_mw;
	}

	for (i = 0; i < CHG_PROTOCOL_MAX; i++) {
		type = backup_type[i];
		if (type != CHG_PROTOCOL_INVALID && (protocol & BIT(type)))
			return type;
	}

	return CHG_PROTOCOL_INVALID;
}

#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
static bool oplus_cpa_need_wired_state_keep(struct oplus_cpa *cpa)
{
	if (IS_ERR_OR_NULL(cpa->keep_topic))
		return false;
	if (!cpa->keep.state_keep_ready)
		return true;

	return cpa->keep.wired_state_keep;
}
#endif

static int oplus_cpa_pd_completed_set(struct oplus_cpa *cpa, bool done)
{
	if (done) {
		complete_all(&cpa->pd_completed_ack);
	} else {
		complete_all(&cpa->pd_completed_ack);
		reinit_completion(&cpa->pd_completed_ack);
	}

	return 0;
}

static void oplus_cpa_bc12_completed_set(struct oplus_cpa *cpa, bool done)
{
	if (!cpa->bc12_check_timeout_ms)
		return;

	WRITE_ONCE(cpa->bc12_completed, done);
	wake_up(&cpa->bc12_wq);
}

static void oplus_cpa_bc12_wait(struct oplus_cpa *cpa, enum oplus_chg_protocol_type type)
{
	unsigned long left;

	if (!cpa->bc12_check_timeout_ms)
		return;

	if (type <= CHG_PROTOCOL_INVALID || type == CHG_PROTOCOL_BC12)
		return;

	left = wait_event_timeout(cpa->bc12_wq,
				  READ_ONCE(cpa->bc12_completed) ||
				  !cpa->wired_online,
				  msecs_to_jiffies(cpa->bc12_check_timeout_ms));
	if (!left)
		chg_err("wait bc1.2 completed timeout\n");
	else if (!cpa->wired_online)
		chg_info("bc1.2 wait aborted, wired offline\n");
	else
		chg_info("bc1.2 wait done, left=%u ms\n", jiffies_to_msecs(left));
}

static int oplus_cpa_protocol_wait(struct oplus_cpa *cpa, enum oplus_chg_protocol_type type)
{
	int rc = 0;
	int wait_time = 0;
	unsigned long left = 0;
	int i;

	if (!cpa->protocol_wait_support)
		return -ENOTSUPP;

	for (i = 0; i < cpa->protocol_wait_cnt; i++) {
		if (cpa->protocol_wait_table[i].type == type) {
			wait_time = cpa->protocol_wait_table[i].time;
			chg_info("set %s to wait %d ms\n", get_protocol_name_str(type), wait_time);
			left = wait_for_completion_timeout(&cpa->pd_completed_ack, msecs_to_jiffies(wait_time));
			if (!left) {
				chg_debug("timeout waiting for pd_ack\n");
				rc = -ETIMEDOUT;
			} else {
				chg_info("exit pd wait. left time = %u\n", jiffies_to_msecs(left));
			}
		}
	}

	return rc;
}

static int oplus_cpa_set_current_protocol_type(struct oplus_cpa *cpa, enum oplus_chg_protocol_type type)
{
	struct mms_msg *msg;
	int rc;

	if (cpa->current_protocol_type == type)
		return 0;

	oplus_cpa_protocol_wait(cpa, type);
	oplus_cpa_bc12_wait(cpa, type);

	chg_info("set current_protocol_type to %s\n", get_protocol_name_str(type));
	cpa->current_protocol_type = type;

	if (cpa->current_protocol_type != CHG_PROTOCOL_INVALID) {
		msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, CPA_ITEM_CHG_TYPE);
		if (msg == NULL) {
			chg_err("alloc msg error\n");
		} else {
			rc = oplus_mms_publish_msg(cpa->cpa_topic, msg);
			if (rc < 0) {
				chg_err("publish cpa charger type msg error, rc=%d\n", rc);
				kfree(msg);
			}
		}
	}

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, CPA_ITEM_ALLOW);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg_sync(cpa->cpa_topic, msg);
	if (rc < 0) {
		chg_err("publish protocol switch allow msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static inline bool oplus_cpa_is_allow_switch(enum oplus_chg_protocol_type current_type, uint32_t bit_target_type)
{
	return (current_type == CHG_PROTOCOL_PD && bit_target_type == BIT(CHG_PROTOCOL_PPS));
}

static int __protocol_identify_request(struct oplus_cpa *cpa, enum oplus_chg_protocol_type type)
{
	int rc;
	enum oplus_chg_protocol_type current_protocol_type;

	if ((type >= CHG_PROTOCOL_MAX) || (type <= CHG_PROTOCOL_INVALID)) {
		chg_err("unsupported protocol type, protocol=%d\n", type);
		return -EINVAL;
	}

	current_protocol_type = READ_ONCE(cpa->current_protocol_type);
	if (current_protocol_type != CHG_PROTOCOL_INVALID &&
	    current_protocol_type != CHG_PROTOCOL_BC12 &&
	    !oplus_cpa_is_allow_switch(current_protocol_type, BIT(type))) {
		chg_info("current %s protocol, not allow switch to %s protocol\n",
			get_protocol_name_str(current_protocol_type), get_protocol_name_str(type));
		return -EBUSY;
	}

	cpa->started = false;
	rc = oplus_cpa_set_current_protocol_type(cpa, type);
	chg_info("start %s protocol identify\n", get_protocol_name_str(type));

	return rc;
}

static int protocol_identify_request(struct oplus_cpa *cpa, uint32_t protocol)
{
	uint32_t protocol_to_be_switched;
	enum oplus_chg_protocol_type current_protocol_type;

	if (!cpa->retention_state) {
		if (!cpa->wired_online) {
			chg_info("wired_online is false\n");
			return -EFAULT;
		}
		if (!oplus_wired_is_present()) {
			chg_info("wired_present is false\n");
			return -EFAULT;
		}
	}

	protocol_to_be_switched = READ_ONCE(cpa->protocol_to_be_switched) | protocol;
	WRITE_ONCE(cpa->protocol_to_be_switched, protocol_to_be_switched);

	current_protocol_type = READ_ONCE(cpa->current_protocol_type);
	if (current_protocol_type != CHG_PROTOCOL_INVALID &&
	    current_protocol_type != CHG_PROTOCOL_BC12 &&
	    !oplus_cpa_is_allow_switch(current_protocol_type, protocol)) {
		chg_info("current %s protocol, not allow switch to 0x%0x protocol\n",
			get_protocol_name_str(current_protocol_type), protocol);
		return -EBUSY;
	}

	/* Suspend other requests before opening the request default protocol */
	if (!cpa->def_req) {
		chg_info("not request default protocol\n");
		return 0;
	}
	if (cpa->request_locked) {
		chg_info("request_locked\n");
		return 0;
	}

	schedule_work(&cpa->protocol_switch_work);

	return 0;
}

static int oplus_cpa_request_default_protocol(struct oplus_cpa *cpa)
{
	uint32_t protocol = cpa->default_protocol_type;

#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	if (!IS_ERR_OR_NULL(cpa->keep_topic) &&
	    !oplus_cpa_protocol_check_enable(cpa->cpa_topic, CHG_PROTOCOL_VOOC))
		protocol |= BIT(CHG_PROTOCOL_QC);
#endif
	return protocol_identify_request(cpa, protocol);
}

static int oplus_cpa_request_lock_vote_callback(struct votable *votable,
						void *data, int locked,
						const char *client, bool step)
{
	struct oplus_cpa *cpa = data;
	int rc;

	if (votable == NULL) {
		chg_err("votable is NUL\n");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NUL\n");
		return -EINVAL;
	}

	if (cpa->request_locked == !!locked)
		return 0;
	cpa->request_locked = !!locked;
	if (!!locked)
		chg_info("cpa request locked by %s\n", client);
	else
		chg_info("cpa request unlock\n");
	if (cpa->request_locked)
		return 0;

	if (!cpa->request_pending) {
		mutex_lock(&cpa->cpa_request_lock);
		chg_info("start request to_be_switched=0x%x protocol\n", READ_ONCE(cpa->protocol_to_be_switched));
		protocol_identify_request(cpa, READ_ONCE(cpa->protocol_to_be_switched));
		mutex_unlock(&cpa->cpa_request_lock);
		return 0;
	}

	cpa->request_pending = false;
	if (cpa->def_req) {
		chg_info("already request default protocol\n");
		return 0;
	}
	mutex_lock(&cpa->cpa_request_lock);
	cpa->def_req = true;
	chg_info("start request default protocol\n");
	rc = oplus_cpa_request_default_protocol(cpa);
	/* If setting protocol_to_be_switched fails, def_req should be set to false */
	if (rc < 0 && rc != -EBUSY)
		cpa->def_req = false;
	mutex_unlock(&cpa->cpa_request_lock);

	return 0;
}

static inline bool oplus_cpa_is_supported_protocol(struct oplus_cpa *cpa, enum oplus_chg_protocol_type protocol_type)
{
	if ((protocol_type >= CHG_PROTOCOL_MAX) || (protocol_type <= CHG_PROTOCOL_INVALID)) {
		chg_err("unsupported protocol type, protocol=%d\n", protocol_type);
		return false;
	}
	return !!(cpa->protocol_supported_type & BIT(protocol_type));
}

static bool oplus_cpa_is_enabled_protocol(struct oplus_cpa *cpa, enum oplus_chg_protocol_type protocol_type)
{
	if ((protocol_type >= CHG_PROTOCOL_MAX) || (protocol_type <= CHG_PROTOCOL_INVALID)) {
		chg_err("unsupported protocol type, protocol=%d\n", protocol_type);
		return false;
	}
	if (!oplus_cpa_is_supported_protocol(cpa, protocol_type))
		return false;
	return !test_bit(protocol_type, &cpa->protocol_disable_mask);
}

static void oplus_cpa_remove_unsupported_protocol(struct oplus_cpa *cpa, struct protocol_map *map)
{
	int i;

	for (i = 0; i < CHG_PROTOCOL_MAX; i++) {
		if (!(map->protocol & BIT(i)))
			continue;
		if (oplus_cpa_is_supported_protocol(cpa, i))
			continue;
		map->protocol &= ~BIT(i);
		/*
		 * When a protocol is not supported, find the next protocol
		 * version compatible with the protocol
		 */
		switch (i) {
		case CHG_PROTOCOL_PD:
		case CHG_PROTOCOL_PPS:
			if (oplus_cpa_is_supported_protocol(cpa, CHG_PROTOCOL_PD)) {
				map->protocol |= BIT(CHG_PROTOCOL_PD);
				map->type[CHG_PROTOCOL_PD] = OPLUS_CHG_USB_TYPE_PD;
				break;
			}
			fallthrough;
		case CHG_PROTOCOL_VOOC:
		case CHG_PROTOCOL_UFCS:
		case CHG_PROTOCOL_QC:
			if (oplus_cpa_is_supported_protocol(cpa, CHG_PROTOCOL_BC12)) {
				map->protocol |= BIT(CHG_PROTOCOL_BC12);
				map->type[CHG_PROTOCOL_BC12] = OPLUS_CHG_USB_TYPE_DCP;
			}
			break;
		default:
			break;
		}
	}
}

static void oplus_cpa_remove_disabled_protocol(struct oplus_cpa *cpa, struct protocol_map *map)
{
	int i;

	for (i = 0; i < CHG_PROTOCOL_MAX; i++) {
		if (!(map->protocol & BIT(i)))
			continue;
		if (oplus_cpa_is_enabled_protocol(cpa, i))
			continue;
		map->protocol &= ~BIT(i);
		/*
		 * When a protocol is disabled, find the next protocol
		 * version compatible with the protocol
		 */
		switch (i) {
		case CHG_PROTOCOL_PD:
		case CHG_PROTOCOL_PPS:
			if (oplus_cpa_is_enabled_protocol(cpa, CHG_PROTOCOL_PD)) {
				map->protocol |= BIT(CHG_PROTOCOL_PD);
				map->type[CHG_PROTOCOL_PD] = OPLUS_CHG_USB_TYPE_PD;
				break;
			}
			fallthrough;
		case CHG_PROTOCOL_VOOC:
		case CHG_PROTOCOL_UFCS:
		case CHG_PROTOCOL_QC:
			if (oplus_cpa_is_enabled_protocol(cpa, CHG_PROTOCOL_BC12)) {
				map->protocol |= BIT(CHG_PROTOCOL_BC12);
				map->type[CHG_PROTOCOL_BC12] = OPLUS_CHG_USB_TYPE_DCP;
			}
			break;
		default:
			break;
		}
	}
}

static int oplus_cpa_get_final_chg_type(struct oplus_cpa *cpa, bool update)
{
	int wired_type;
	enum oplus_chg_protocol_type protocol_type;
	struct protocol_map map = { 0 };
	union mms_msg_data data = { 0 };
	int rc;

	rc = oplus_mms_get_item_data(cpa->wired_topic, WIRED_ITEM_REAL_CHG_TYPE, &data, update);
	if (rc < 0)
		wired_type = OPLUS_CHG_USB_TYPE_UNKNOWN;
	else
		wired_type = data.intval;
	oplus_cpa_protocol_add_type(&map, wired_type);

	if (sid_to_adapter_chg_type(cpa->vooc_sid) == CHARGER_TYPE_VOOC)
		wired_type = OPLUS_CHG_USB_TYPE_VOOC;
	else if (sid_to_adapter_chg_type(cpa->vooc_sid) == CHARGER_TYPE_SVOOC)
		wired_type = OPLUS_CHG_USB_TYPE_SVOOC;
	oplus_cpa_protocol_add_type(&map, wired_type);

	if (cpa->ufcs_online)
		oplus_cpa_protocol_add_type(&map, OPLUS_CHG_USB_TYPE_UFCS);

	oplus_cpa_remove_unsupported_protocol(cpa, &map);
	oplus_cpa_remove_disabled_protocol(cpa, &map);
	wired_type = oplus_cpa_get_high_prio_wired_type(cpa->cpa_topic, &map);

	protocol_type = oplus_cpa_chg_type_to_protocol_type(wired_type);
	chg_debug("wired_type=%s, protocol_type=%s, current_protocol_type=%s\n",
		  oplus_wired_get_chg_type_str(wired_type),
		  get_protocol_name_str(protocol_type),
		  get_protocol_name_str(cpa->current_protocol_type));
	if (protocol_type == CHG_PROTOCOL_INVALID) {
		wired_type = OPLUS_CHG_USB_TYPE_UNKNOWN;
	} else if (cpa->current_protocol_type == CHG_PROTOCOL_PD && protocol_type == CHG_PROTOCOL_PPS) {
		/* The PPS type can be compatible with PD, so it needs to be displayed as PD here */
		wired_type = OPLUS_CHG_USB_TYPE_PD;
	} else if (protocol_type != CHG_PROTOCOL_BC12 && cpa->current_protocol_type != protocol_type) {
		/* For protocols that are not currently allowed, DCP is displayed first */
		wired_type = OPLUS_CHG_USB_TYPE_DCP;
	}

	return wired_type;
}

static void oplus_cpa_protocol_switch_work(struct work_struct *work)
{
	enum oplus_chg_protocol_type type;
	enum oplus_chg_protocol_type current_type;
	struct oplus_cpa *cpa =
		container_of(work, struct oplus_cpa, protocol_switch_work);
	uint32_t protocol;
	uint32_t protocol_to_be_switched;
	int rc;

	/* Suspend other requests before opening the request default protocol */
	if (!cpa->def_req) {
		chg_info("not request default protocol\n");
		return;
	}

	if (!cpa->retention_state) {
		if (!cpa->wired_online) {
			chg_info("wired_online is false\n");
			return;
		}
		if (!oplus_wired_is_present()) {
			chg_info("wired_present is false\n");
			return;
		}
	}

	chg_info("protocol_disable_mask=0x%lx, protocol_to_be_switched=0x%x",
		 cpa->protocol_disable_mask, cpa->protocol_to_be_switched);
	protocol_to_be_switched = READ_ONCE(cpa->protocol_to_be_switched);
	protocol_to_be_switched &= ~READ_ONCE(cpa->protocol_disable_mask);
	type = get_highest_priority_protocol_type(cpa, protocol_to_be_switched);
	if (type == CHG_PROTOCOL_INVALID)
		return;

	current_type = oplus_cpa_chg_type_to_protocol_type(oplus_cpa_get_final_chg_type(cpa, false));
	if (current_type != CHG_PROTOCOL_INVALID) {
		protocol = protocol_to_be_switched | BIT(current_type);
		current_type = get_highest_priority_protocol_type(cpa, protocol);
		/* existing protocols have higher priority */
		if (current_type != type)
			return;
	}

	cancel_delayed_work_sync(&cpa->protocol_switch_timeout_work);
	rc = __protocol_identify_request(cpa, type);
	if (rc < 0) {
		if (rc != -EBUSY) {
			chg_err("switch %s protocol error, rc=%d\n", get_protocol_name_str(type), rc);
			oplus_cpa_set_current_protocol_type(cpa, CHG_PROTOCOL_INVALID);
			mutex_lock(&cpa->cpa_request_lock);
			protocol_identify_request(cpa, cpa->protocol_to_be_switched);
			mutex_unlock(&cpa->cpa_request_lock);
		}
		return;
	} else {
		mutex_lock(&cpa->start_lock);
		if (!READ_ONCE(cpa->started)) {
			schedule_delayed_work(&cpa->protocol_switch_timeout_work,
				msecs_to_jiffies(PROTOCAL_SWITCH_REPLY_TIMEOUT_MS));
			chg_info("switch %s schedule protocol_switch_timeout_work\n", get_protocol_name_str(type));
		}
		mutex_unlock(&cpa->start_lock);
		mutex_lock(&cpa->cpa_request_lock);
		protocol = READ_ONCE(cpa->protocol_to_be_switched);
		protocol &= ~BIT(type);
		WRITE_ONCE(cpa->protocol_to_be_switched, protocol);
		mutex_unlock(&cpa->cpa_request_lock);
	}
	chg_info("switch to %s protocol\n", get_protocol_name_str(type));
}

static void oplus_cpa_switch_end_work(struct work_struct *work)
{
	struct oplus_cpa *cpa =
		container_of(work, struct oplus_cpa, switch_end_work);
	struct mms_msg *msg;
	enum oplus_chg_protocol_type type;
	int rc;

	if (cpa == NULL) {
		chg_err("cpa is NULL\n");
		return;
	}

	type = cpa->current_protocol_type;
	WRITE_ONCE(cpa->started, false);
	oplus_cpa_set_current_protocol_type(cpa, CHG_PROTOCOL_INVALID);

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, CPA_ITEM_CHG_TYPE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
	} else {
		rc = oplus_mms_publish_msg(cpa->cpa_topic, msg);
		if (rc < 0) {
			chg_err("publish cpa charger type msg error, rc=%d\n", rc);
			kfree(msg);
		}
	}

	mutex_lock(&cpa->cpa_request_lock);
	chg_info("%s protocol identify end, to_be_switched=0x%x, disable_mask=0x%lx\n",
		 get_protocol_name_str(type), cpa->protocol_to_be_switched,
		 cpa->protocol_disable_mask);
	protocol_identify_request(cpa, READ_ONCE(cpa->protocol_to_be_switched));
	mutex_unlock(&cpa->cpa_request_lock);
}

static void oplus_cpa_chg_type_change_work(struct work_struct *work)
{
	struct oplus_cpa *cpa =
		container_of(work, struct oplus_cpa, chg_type_change_work);
	union mms_msg_data data = { 0 };
	struct mms_msg *msg;
	int wired_type;
	int rc;

	if (!cpa->wired_online) {
		wired_type = OPLUS_CHG_USB_TYPE_UNKNOWN;
	} else {
		rc = oplus_mms_get_item_data(cpa->wired_topic, WIRED_ITEM_REAL_CHG_TYPE, &data, false);
		if (rc < 0)
			wired_type = OPLUS_CHG_USB_TYPE_UNKNOWN;
		else
			wired_type = data.intval;
		if (cpa->wired_real_chg_type != wired_type) {
			switch (wired_type) {
			case OPLUS_CHG_USB_TYPE_UNKNOWN:
			case OPLUS_CHG_USB_TYPE_SDP:
			case OPLUS_CHG_USB_TYPE_PD_DRP:
			case OPLUS_CHG_USB_TYPE_PD_SDP:
			case OPLUS_CHG_USB_TYPE_CDP:
				break;
			case OPLUS_CHG_USB_TYPE_PD_PPS:
				/* switch to pps from pd because some third adapter give pd first
				   and then give pps later, otherwise fall-through default */
				if (cpa->wired_real_chg_type == OPLUS_CHG_USB_TYPE_PD &&
				    (cpa->default_protocol_type & BIT(CHG_PROTOCOL_PPS)) &&
				    test_bit(CHG_PROTOCOL_PPS, &cpa->ready_protocol_type)) {
					if (cpa->retention_state) {
						chg_info("retention state online, not retry PPS");
						break;
					}
					chg_info("wired_type change to PPS, retry PPS");
					mutex_lock(&cpa->cpa_request_lock);
					protocol_identify_request(cpa, BIT(CHG_PROTOCOL_PPS));
					mutex_unlock(&cpa->cpa_request_lock);
					break;
				}
				fallthrough;
			default:
				if (!cpa->def_req) {
					if ((cpa->ready_protocol_type & cpa->default_protocol_type) !=
					    cpa->default_protocol_type) {
						chg_info("request pending, ready_protocol_type=0x%lx, "
							"default_protocol_type=0x%x\n",
							cpa->ready_protocol_type, cpa->default_protocol_type);
						cpa->request_pending = true;
					}
					if (cpa->request_locked) {
						chg_info("cpa request locked by %s\n",
							get_effective_client(cpa->req_lock_votable));
						cpa->request_pending = true;
						break;
					}
					/* prevent online status changes */
					if (!READ_ONCE(cpa->wired_online))
						break;
					mutex_lock(&cpa->cpa_request_lock);
					cpa->def_req = true;
					chg_info("start request default protocol\n");
					rc = oplus_cpa_request_default_protocol(cpa);
					/* If setting protocol_to_be_switched fails, def_req should be set to false */
					if (rc < 0 && rc != -EBUSY)
						cpa->def_req = false;
					mutex_unlock(&cpa->cpa_request_lock);
				}
				break;
			}
			cpa->wired_real_chg_type = wired_type;
		}
	}

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, CPA_ITEM_CHG_TYPE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(cpa->cpa_topic, msg);
	if (rc < 0) {
		chg_err("publish cpa charger type msg error, rc=%d\n", rc);
		kfree(msg);
	}

#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	schedule_work(&cpa->power_change_work);
#endif
}

static void oplus_cpa_fast_chg_type_change_work(struct work_struct *work)
{
	struct oplus_cpa *cpa =
		container_of(work, struct oplus_cpa, fast_chg_type_change_work);
	struct mms_msg *msg;
	int rc;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, CPA_ITEM_CHG_TYPE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(cpa->cpa_topic, msg);
	if (rc < 0) {
		chg_err("publish cpa charger type msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
static void oplus_cpa_power_change_work(struct work_struct *work)
{
	struct oplus_cpa *cpa =
		container_of(work, struct oplus_cpa, power_change_work);
	struct mms_msg *msg;
	int rc;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, CPA_ITEM_POWER);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(cpa->cpa_topic, msg);
	if (rc < 0) {
		chg_err("publish cpa power change msg error, rc=%d\n", rc);
		kfree(msg);
	}
}
#endif

static void oplus_cpa_protocol_switch_timeout_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_cpa *cpa = container_of(dwork,
		struct oplus_cpa, protocol_switch_timeout_work);
	struct mms_msg *msg;
	int rc;

	chg_err("%s timeout\n", get_protocol_name_str(cpa->current_protocol_type));
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, CPA_ITEM_TIMEOUT);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg_sync(cpa->cpa_topic, msg);
	if (rc < 0) {
		chg_err("publish protocol switch timeout msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_cpa_protocol_ready_timeout_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_cpa *cpa = container_of(dwork,
		struct oplus_cpa, protocol_ready_timeout_work);

	chg_err("wait protocol ready timeout, ready_protocol_type=0x%lx, request_pending=%d\n",
		cpa->ready_protocol_type, cpa->request_pending);

	vote(cpa->req_lock_votable, DEF_VOTER, false, 0, false);
}

#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
static void oplus_cpa_state_keep_offline_check(struct oplus_cpa *cpa)
{
	int i;

	if (IS_ERR_OR_NULL(cpa->keep_topic))
		return;

	mutex_lock(&cpa->keep.ready_lock);
	if (!cpa->keep.state_keep_ready) {
		vote(cpa->req_lock_votable, STATE_KEEP_VOTER, true, 1, false);
		cpa->keep.protocol_disable_mask = cpa->protocol_disable_mask;
		cpa->keep.protocol_to_be_switched = cpa->protocol_to_be_switched;
		for (i = 0; i < CHG_PROTOCOL_MAX; i++) {
			if ((cpa->protocol_prio_table[i].type < CHG_PROTOCOL_MAX) &&
			    (cpa->protocol_prio_table[i].type > CHG_PROTOCOL_INVALID)) {
				cpa->keep.protocol_power[cpa->protocol_prio_table[i].type] =
					cpa->protocol_prio_table[i].power_mw;
				cpa->keep.protocol_max_power[cpa->protocol_prio_table[i].type] =
					cpa->protocol_prio_table[i].max_power_dynamic;
			}
		}
	} else {
		cpa->keep.protocol_disable_mask = 0;
		cpa->keep.protocol_to_be_switched = 0;
		memset(cpa->keep.protocol_power, 0, sizeof(cpa->keep.protocol_power));
		memset(cpa->keep.protocol_max_power, 0, sizeof(cpa->keep.protocol_max_power));
	}
	mutex_unlock(&cpa->keep.ready_lock);
}
#endif /* CONFIG_OPLUS_CHG_STATE_KEEP */

static void oplus_cpa_common_clear(struct oplus_cpa *cpa)
{
	int i;

#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	if (oplus_cpa_need_wired_state_keep(cpa))
		return;
#endif
	cpa->protocol_to_be_switched = 0;
	cpa->protocol_disable_mask = 0;
	for (i = 0; i < CHG_PROTOCOL_MAX; i++) {
		if (cpa->protocol_prio_table[i].type != CHG_PROTOCOL_INVALID) {
			oplus_cpa_protocol_clear_power(cpa->cpa_topic,
				cpa->protocol_prio_table[i].type);
			oplus_cpa_protocol_restore_max_power(cpa->cpa_topic,
				cpa->protocol_prio_table[i].type);
		}
	}
}

static void oplus_cpa_offline_clear(struct oplus_cpa *cpa)
{
	oplus_cpa_set_current_protocol_type(cpa, CHG_PROTOCOL_INVALID);
	cpa->def_req = false;
	cpa->request_pending = false;
	cpa->wired_real_chg_type = OPLUS_CHG_USB_TYPE_UNKNOWN;
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	mutex_lock(&cpa->keep.ready_lock);
	oplus_cpa_common_clear(cpa);
	mutex_unlock(&cpa->keep.ready_lock);
#else
	oplus_cpa_common_clear(cpa);
#endif
}

static void oplus_cpa_wired_offline_work(struct work_struct *work)
{
	struct oplus_cpa *cpa =
		container_of(work, struct oplus_cpa, wired_offline_work);

	if (READ_ONCE(cpa->status_reset)) {
		chg_info("status has been reset\n");
		return;
	}
	if (cpa->wired_online) {
		chg_err("wired is online\n");
		return;
	}

#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	oplus_cpa_state_keep_offline_check(cpa);
#endif
	cancel_delayed_work_sync(&cpa->protocol_switch_timeout_work);
	oplus_cpa_offline_clear(cpa);
	WRITE_ONCE(cpa->status_reset, true);
	chg_info("cpa status reset done\n");
}

static bool oplus_wired_offline_clear_cpa_queue(struct oplus_cpa *cpa)
{
	unsigned long old_time;

	old_time = cpa->wired_plugout_time + msecs_to_jiffies(WIRED_PLUGOUT_TO_PRESENT_MS);
	return time_is_before_jiffies(old_time);
}

static void oplus_cpa_wired_online_work(struct work_struct *work)
{
	struct oplus_cpa *cpa =
		container_of(work, struct oplus_cpa, wired_online_work);
	union mms_msg_data data = { 0 };
	int rc;

	if (!READ_ONCE(cpa->status_reset)) {
		chg_info("cpa status not reset\n");
		if (cpa->retention_topic) {
			if (cpa->cc_detect == CC_DETECT_NOTPLUG ||
			    oplus_wired_offline_clear_cpa_queue(cpa)) {
				chg_info("cc_detect or offline clear cpa queue, cc_detect=%d\n", cpa->cc_detect);
				schedule_work(&cpa->wired_offline_work);
			} else {
				chg_info("cc_detect=%d, not schedule wired_offline_work\n", cpa->cc_detect);
			}
		} else {
			chg_info("schedule wired_offline_work\n");
			schedule_work(&cpa->wired_offline_work);
		}
	}
	(void)flush_work(&cpa->wired_offline_work);
	rc = oplus_mms_get_item_data(cpa->wired_topic, WIRED_ITEM_ONLINE, &data, false);
	if (rc < 0) {
		chg_err("can't get wired online status, rc=%d\n", rc);
		return;
	}
	if (!data.intval) {
		chg_err("wired_online is false\n");
		return;
	}
	cpa->wired_online = true;
	WRITE_ONCE(cpa->status_reset, false);

	rc = oplus_mms_get_item_data(cpa->wired_topic, WIRED_ITEM_REAL_CHG_TYPE, &data, false);
	if ((rc < 0) || (data.intval == OPLUS_CHG_USB_TYPE_UNKNOWN))
		return;
	schedule_work(&cpa->chg_type_change_work);
	chg_info("done\n");
}

static void oplus_cpa_wired_online_change(struct oplus_cpa *cpa, bool online)
{
	if (!online) {
		cpa->wired_online = false;
		cpa->pd_comleted = false;
		oplus_cpa_pd_completed_set(cpa, cpa->pd_comleted);
		oplus_cpa_bc12_completed_set(cpa, false);
		if (!cpa->retention_topic) {
			chg_info("schedule wired_offline_work\n");
			schedule_work(&cpa->wired_offline_work);
		} else if ((cpa->retention_state_ready || cpa->pre_retention_state)
			&& !cpa->retention_state) {
			chg_info("retention_state is false, schedule wired_offline_work\n");
			schedule_work(&cpa->wired_offline_work);
		}
		cpa->pre_retention_state = cpa->retention_state;
	} else {
		if (!cpa->retention_state)
			cpa->retention_state_ready = false;
		schedule_work(&cpa->wired_online_work);
	}
}

static void oplus_cpa_wired_subs_callback(struct mms_subscribe *subs,
					  enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_cpa *cpa = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WIRED_ITEM_ONLINE:
			oplus_mms_get_item_data(cpa->wired_topic, id, &data, false);
			oplus_cpa_wired_online_change(cpa, !!data.intval);
			break;
		case WIRED_ITEM_REAL_CHG_TYPE:
			oplus_mms_get_item_data(cpa->wired_topic, id, &data, false);
			chg_info("type=%s\n", oplus_wired_get_chg_type_str(data.intval));
			schedule_work(&cpa->chg_type_change_work);
			break;
		case WIRED_ITEM_CC_DETECT:
			oplus_mms_get_item_data(cpa->wired_topic, id, &data, false);
			cpa->cc_detect = data.intval;
			if (!!cpa->retention_topic && !cpa->wired_online &&
				cpa->cc_detect == CC_DETECT_NOTPLUG) {
				chg_info("cc_detect notplug, schedule wired_offline_work\n");
				schedule_work(&cpa->wired_offline_work);
			}
			break;
		case WIRED_ITEM_CHG_TYPE:
			oplus_mms_get_item_data(cpa->wired_topic, id, &data,
						false);
			cpa->wired_type = data.intval;
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
			schedule_work(&cpa->power_change_work);
#endif
			break;
		case WIRED_ITEM_PD_COMPLETED:
			oplus_mms_get_item_data(cpa->wired_topic, id, &data, false);
			cpa->pd_comleted = data.intval;
			oplus_cpa_pd_completed_set(cpa, cpa->pd_comleted);
			break;
		case WIRED_ITEM_BC12_COMPLETED:
			oplus_mms_get_item_data(cpa->wired_topic, id, &data, false);
			oplus_cpa_bc12_completed_set(cpa, !!data.intval);
			break;
		case WIRED_ITEM_PRESENT:
			oplus_mms_get_item_data(cpa->wired_topic, id, &data, false);
			cpa->wired_present = data.intval;
			if (!cpa->wired_present)
				cpa->wired_plugout_time = jiffies;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_cpa_subscribe_wired_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_cpa *cpa = prv_data;
	union mms_msg_data data = { 0 };

	cpa->wired_topic = topic;
	cpa->wired_subs =
		oplus_mms_subscribe(cpa->wired_topic, cpa,
				    oplus_cpa_wired_subs_callback, "cpa");
	if (IS_ERR_OR_NULL(cpa->wired_subs)) {
		chg_err("subscribe gauge topic error, rc=%ld\n",
			PTR_ERR(cpa->wired_subs));
		return;
	}

	oplus_mms_get_item_data(cpa->wired_topic, WIRED_ITEM_ONLINE, &data, true);
	if (data.intval)
		schedule_work(&cpa->wired_online_work);
	oplus_mms_get_item_data(cpa->wired_topic, WIRED_ITEM_PRESENT, &data, true);
	cpa->wired_present = !!data.intval;
	if (!cpa->wired_present)
		cpa->wired_plugout_time = jiffies;
}

static void oplus_cpa_vooc_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_cpa *cpa = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case VOOC_ITEM_VOOC_STARTED:
			oplus_mms_get_item_data(cpa->vooc_topic, id, &data,
						false);
			cpa->vooc_started = data.intval;
			break;
		case VOOC_ITEM_VOOC_CHARGING:
			oplus_mms_get_item_data(cpa->vooc_topic, id, &data,
						false);
			cpa->vooc_charging = data.intval;
			break;
		case VOOC_ITEM_ONLINE:
			oplus_mms_get_item_data(cpa->vooc_topic, id, &data,
						false);
			cpa->vooc_online = data.intval;
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
			schedule_work(&cpa->power_change_work);
#endif
			break;
		case VOOC_ITEM_SID:
			oplus_mms_get_item_data(cpa->vooc_topic, id, &data,
						false);
			cpa->vooc_sid = (unsigned int)data.intval;
			schedule_work(&cpa->fast_chg_type_change_work);
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
			schedule_work(&cpa->power_change_work);
#endif
			break;
		case VOOC_ITEM_ONLINE_KEEP:
			oplus_mms_get_item_data(cpa->vooc_topic, id, &data,
						false);
			cpa->vooc_online_keep = (unsigned int)data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_cpa_subscribe_vooc_topic(struct oplus_mms *topic,
					   void *prv_data)
{
	struct oplus_cpa *cpa = prv_data;
	union mms_msg_data data = { 0 };

	cpa->vooc_topic = topic;
	cpa->vooc_subs = oplus_mms_subscribe(cpa->vooc_topic, cpa,
					     oplus_cpa_vooc_subs_callback,
					     "cpa");
	if (IS_ERR_OR_NULL(cpa->vooc_subs)) {
		chg_err("subscribe vooc topic error, rc=%ld\n",
			PTR_ERR(cpa->vooc_subs));
		return;
	}

	oplus_mms_get_item_data(cpa->vooc_topic, VOOC_ITEM_VOOC_STARTED, &data,
				true);
	cpa->vooc_started = data.intval;
	oplus_mms_get_item_data(cpa->vooc_topic, VOOC_ITEM_VOOC_CHARGING,
				&data, true);
	cpa->vooc_charging = data.intval;
	oplus_mms_get_item_data(cpa->vooc_topic, VOOC_ITEM_ONLINE, &data,
				true);
	cpa->vooc_online = data.intval;
	oplus_mms_get_item_data(cpa->vooc_topic, VOOC_ITEM_ONLINE_KEEP, &data,
				true);
	cpa->vooc_online_keep = data.intval;

	oplus_mms_get_item_data(cpa->vooc_topic, VOOC_ITEM_SID, &data, true);
	cpa->vooc_sid = (unsigned int)data.intval;
}

static void oplus_cpa_ufcs_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_cpa *cpa = subs->priv_data;
	union mms_msg_data data = { 0 };
	int rc;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case UFCS_ITEM_ONLINE:
			oplus_mms_get_item_data(cpa->ufcs_topic, id, &data, false);
			cpa->ufcs_online = !!data.intval;
			chg_info("ufcs_online=%d\n", cpa->ufcs_online);
			schedule_work(&cpa->fast_chg_type_change_work);
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
			schedule_work(&cpa->power_change_work);
#endif
			break;
		case UFCS_ITEM_CHARGING:
			rc = oplus_mms_get_item_data(cpa->ufcs_topic, id, &data, false);
			if (rc < 0)
				break;
			cpa->ufcs_charging = !!data.intval;
			break;
		case UFCS_ITEM_ADAPTER_ID:
			rc = oplus_mms_get_item_data(cpa->ufcs_topic, id, &data, false);
			if (rc < 0)
				break;
			cpa->ufcs_adapter_id = (u32)data.intval;
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
			schedule_work(&cpa->power_change_work);
#endif
			break;
		case UFCS_ITEM_OPLUS_ADAPTER:
			rc = oplus_mms_get_item_data(cpa->ufcs_topic, id, &data, false);
			if (rc < 0)
				break;
			cpa->ufcs_oplus_adapter = !!data.intval;
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
			schedule_work(&cpa->power_change_work);
#endif
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_cpa_subscribe_ufcs_topic(struct oplus_mms *topic,
					   void *prv_data)
{
	struct oplus_cpa *cpa = prv_data;
	union mms_msg_data data = { 0 };
	int rc;

	cpa->ufcs_topic = topic;
	cpa->ufcs_subs = oplus_mms_subscribe(topic, cpa,
					     oplus_cpa_ufcs_subs_callback,
					     "cpa");
	if (IS_ERR_OR_NULL(cpa->ufcs_subs)) {
		chg_err("subscribe ufcs topic error, rc=%ld\n",
			PTR_ERR(cpa->ufcs_subs));
		return;
	}

	oplus_mms_get_item_data(cpa->ufcs_topic, UFCS_ITEM_ONLINE, &data, true);
	cpa->ufcs_online = !!data.intval;

	rc = oplus_mms_get_item_data(cpa->ufcs_topic, UFCS_ITEM_CHARGING, &data, true);
	if (rc >= 0)
		cpa->ufcs_charging = !!data.intval;
	rc = oplus_mms_get_item_data(cpa->ufcs_topic, UFCS_ITEM_ADAPTER_ID, &data, true);
	if (rc >= 0)
		cpa->ufcs_adapter_id = (u32)data.intval;
	rc = oplus_mms_get_item_data(cpa->ufcs_topic, UFCS_ITEM_OPLUS_ADAPTER, &data, true);
	if (rc >= 0)
		cpa->ufcs_oplus_adapter = !!data.intval;
}

static void oplus_cpa_pps_subs_callback(struct mms_subscribe *subs,
	enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_cpa *cpa = subs->priv_data;
	union mms_msg_data data = { 0 };
	int rc;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case PPS_ITEM_ONLINE:
			oplus_mms_get_item_data(cpa->pps_topic, id, &data, false);
			cpa->pps_online = !!data.intval;
			chg_info("pps_online=%d\n", cpa->pps_online);
			schedule_work(&cpa->fast_chg_type_change_work);
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
			schedule_work(&cpa->power_change_work);
#endif
			break;
		case PPS_ITEM_CHARGING:
			rc = oplus_mms_get_item_data(cpa->pps_topic, id, &data, false);
			if (rc < 0)
				break;
			cpa->pps_charging = !!data.intval;
			break;
		case PPS_ITEM_OPLUS_ADAPTER:
			rc = oplus_mms_get_item_data(cpa->pps_topic, id, &data, false);
			if (rc < 0)
				break;
			cpa->pps_oplus_adapter = !!data.intval;
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
			schedule_work(&cpa->power_change_work);
#endif
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_cpa_subscribe_pps_topic(struct oplus_mms *topic,
	  void *prv_data)
{
	struct oplus_cpa *cpa = prv_data;
	union mms_msg_data data = { 0 };
	int rc;

	cpa->pps_topic = topic;
	cpa->pps_subs = oplus_mms_subscribe(topic, cpa,
		oplus_cpa_pps_subs_callback,
		"cpa");
	if (IS_ERR_OR_NULL(cpa->pps_subs)) {
		chg_err("subscribe pps topic error, rc=%ld\n",
			PTR_ERR(cpa->pps_subs));
		return;
	}

	oplus_mms_get_item_data(cpa->pps_topic, PPS_ITEM_ONLINE, &data, true);
	cpa->pps_online = !!data.intval;

	rc = oplus_mms_get_item_data(cpa->pps_topic, PPS_ITEM_CHARGING, &data, true);
	if (rc >= 0)
		cpa->pps_charging = !!data.intval;
	rc = oplus_mms_get_item_data(cpa->pps_topic, PPS_ITEM_OPLUS_ADAPTER, &data, true);
	if (rc >= 0)
		cpa->pps_oplus_adapter = !!data.intval;
}

static void oplus_cpa_retention_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_cpa *cpa = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case RETENTION_ITEM_CONNECT_STATUS:
			oplus_mms_get_item_data(cpa->retention_topic, id, &data,
						false);
			if (!data.intval && cpa->retention_state != !!data.intval) {
				chg_info("retention_state true to false, schedule wired_offline_work\n");
				schedule_work(&cpa->wired_offline_work);
			}
			cpa->retention_state = !!data.intval;
			if (cpa->retention_state)
				cpa->pre_retention_state = cpa->retention_state;
			if (cpa->wired_present && !cpa->retention_state)
				cpa->pre_retention_state = false;
			break;
		case RETENTION_ITEM_STATE_READY:
			cpa->retention_state_ready = true;
			if (!cpa->retention_state && !cpa->wired_online) {
				chg_info("retention_state false, schedule wired_offline_work\n");
				schedule_work(&cpa->wired_offline_work);
			}
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_cpa_subscribe_retention_topic(struct oplus_mms *topic,
					   void *prv_data)
{
	struct oplus_cpa *cpa = prv_data;
	union mms_msg_data data = { 0 };
	int rc;

	cpa->retention_topic = topic;
	cpa->retention_subs = oplus_mms_subscribe(topic, cpa,
					     oplus_cpa_retention_subs_callback,
					     "cpa");
	if (IS_ERR_OR_NULL(cpa->retention_subs)) {
		chg_err("subscribe retention topic error, rc=%ld\n",
			PTR_ERR(cpa->retention_subs));
		return;
	}
	rc = oplus_mms_get_item_data(cpa->retention_topic, RETENTION_ITEM_CONNECT_STATUS, &data, true);
	if (rc >= 0)
		cpa->retention_state = !!data.intval;
}

#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)

static void oplus_cpa_keep_status_clear(struct oplus_cpa *cpa)
{
	cpa->keep.protocol_disable_mask = 0;
	cpa->keep.protocol_to_be_switched = 0;
	memset(cpa->keep.protocol_power, 0, sizeof(cpa->keep.protocol_power));
	memset(cpa->keep.protocol_max_power, 0, sizeof(cpa->keep.protocol_max_power));
	oplus_cpa_common_clear(cpa);
}


static void oplus_cpa_state_keep_wired_offline_work(struct work_struct *work)
{
	struct oplus_cpa *cpa = container_of(work, struct oplus_cpa, state_keep_wired_offline_work);

	chg_info("wired offline clear, ready=%d\n", cpa->keep.state_keep_ready);
	if (!cpa->keep.state_keep_ready)
		return;

	mutex_lock(&cpa->keep.ready_lock);
	oplus_cpa_keep_status_clear(cpa);
	mutex_unlock(&cpa->keep.ready_lock);
}

static void oplus_cpa_keep_ready_check(struct oplus_cpa *cpa, bool update)
{
	union mms_msg_data data = { 0 };
	int i;
	int rc;

	mutex_lock(&cpa->keep.ready_lock);
	rc = oplus_mms_get_item_data(cpa->keep_topic, STATE_KEEP_ITEM_READY, &data, update);
	if (rc < 0) {
		chg_err("get ready status error, rc=%d\n", rc);
		cpa->keep.state_keep_ready = false;
	} else {
		cpa->keep.state_keep_ready = !!data.intval;
	}

	if (!cpa->keep.state_keep_ready) {
		mutex_unlock(&cpa->keep.ready_lock);
		return;
	}
	if (!cpa->keep.wired_state_keep) {
		oplus_cpa_keep_status_clear(cpa);
		goto done;
	}

	cpa->protocol_disable_mask |= cpa->keep.protocol_disable_mask;
	cpa->keep.protocol_disable_mask = 0;
	cpa->protocol_to_be_switched |= cpa->keep.protocol_to_be_switched;
	cpa->keep.protocol_to_be_switched = 0;
	chg_info("protocol_disable_mask=0x%lx, protocol_to_be_switched=0x%x",
		 cpa->protocol_disable_mask, cpa->protocol_to_be_switched);
	for (i = 0; i < CHG_PROTOCOL_MAX; i++) {
		if ((cpa->protocol_prio_table[i].type < CHG_PROTOCOL_MAX) &&
		    (cpa->protocol_prio_table[i].type > CHG_PROTOCOL_INVALID)) {
			if ((cpa->keep.protocol_power[cpa->protocol_prio_table[i].type] != 0)) {
				cpa->protocol_prio_table[i].power_mw =
					cpa->keep.protocol_power[cpa->protocol_prio_table[i].type];
			}
			if ((cpa->keep.protocol_max_power[cpa->protocol_prio_table[i].type] != 0)) {
				cpa->protocol_prio_table[i].max_power_dynamic =
					cpa->keep.protocol_max_power[cpa->protocol_prio_table[i].type];
			}
		}
	}
	memset(cpa->keep.protocol_power, 0, sizeof(cpa->keep.protocol_power));
	memset(cpa->keep.protocol_max_power, 0, sizeof(cpa->keep.protocol_max_power));

done:
	vote(cpa->req_lock_votable, STATE_KEEP_VOTER, false, 0, false);
	mutex_unlock(&cpa->keep.ready_lock);
}

static void oplus_cpa_wired_keep_check(struct oplus_cpa *cpa, bool update)
{
	union mms_msg_data data = { 0 };
	int rc;

	rc = oplus_mms_get_item_data(cpa->keep_topic, STATE_KEEP_ITEM_WIRED_KEEP, &data, update);
	if (rc < 0) {
		chg_err("get ready status error, rc=%d\n", rc);
		cpa->keep.wired_state_keep = false;
	} else {
		cpa->keep.wired_state_keep = !!data.intval;
	}
}

static void oplus_cpa_keep_wired_type_check(struct oplus_cpa *cpa, bool update)
{
	union mms_msg_data data = { 0 };
	int rc;

	rc = oplus_mms_get_item_data(cpa->keep_topic, STATE_KEEP_ITEM_WIRED_TYPE, &data, update);
	if (rc < 0)
		chg_err("get ready status error, rc=%d\n", rc);
	else
		cpa->keep.wired_type = data.intval;
}

static void oplus_cpa_state_keep_reset(struct oplus_cpa *cpa)
{
	chg_info("state keep reset\n");
	mutex_lock(&cpa->keep.ready_lock);
	oplus_cpa_keep_status_clear(cpa);
	mutex_unlock(&cpa->keep.ready_lock);
}

static void oplus_cpa_keep_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_cpa *cpa = subs->priv_data;
	union mms_msg_data data = { 0 };
	int rc;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case STATE_KEEP_ITEM_READY:
			oplus_cpa_keep_ready_check(cpa, false);
			break;
		case STATE_KEEP_ITEM_WIRED_KEEP:
			oplus_cpa_wired_keep_check(cpa, false);
			break;
		case STATE_KEEP_ITEM_WIRED_TYPE:
			oplus_cpa_keep_wired_type_check(cpa, false);
			break;
		case STATE_KEEP_ITEM_WIRED_ONLINE:
			rc = oplus_mms_get_item_data(cpa->keep_topic, id, &data, false);
			if (rc < 0)
				chg_err("get wired online status error, rc=%d\n", rc);
			else if (!data.intval)
					schedule_work(&cpa->state_keep_wired_offline_work);
			break;
		case STATE_KEEP_ITEM_RESET:
			oplus_cpa_state_keep_reset(cpa);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_cpa_subscribe_keep_topic(struct oplus_mms *topic,
					   void *prv_data)
{
	struct oplus_cpa *cpa = prv_data;

	cpa->keep_topic = topic;
	cpa->keep_subs = oplus_mms_subscribe(topic, cpa,
					     oplus_cpa_keep_subs_callback,
					     "cpa");
	if (IS_ERR_OR_NULL(cpa->keep_subs)) {
		chg_err("subscribe state keep topic error, rc=%ld\n",
			PTR_ERR(cpa->keep_subs));
		return;
	}

	oplus_cpa_wired_keep_check(cpa, true);
	oplus_cpa_keep_ready_check(cpa, true);
	oplus_cpa_keep_wired_type_check(cpa, true);
}
#endif /* CONFIG_OPLUS_CHG_STATE_KEEP */

static int oplus_cpa_update_chg_type(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_cpa *cpa;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	cpa = oplus_mms_get_drvdata(mms);

	data->intval = oplus_cpa_get_final_chg_type(cpa, true);

	return 0;
}

static int oplus_cpa_update_allow(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_cpa *cpa;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	cpa = oplus_mms_get_drvdata(mms);

	data->intval = cpa->current_protocol_type;

	return 0;
}

static int oplus_cpa_update_timeout(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_cpa *cpa;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	cpa = oplus_mms_get_drvdata(mms);

	data->intval = cpa->current_protocol_type;

	return 0;
}

static int oplus_cpa_update_power(struct oplus_mms *mms, union mms_msg_data *data)
{
	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}

	data->intval = oplus_cpa_get_actual_used_power(mms);

	return 0;
}

static void oplus_cpa_update(struct oplus_mms *mms, bool publish)
{
}

static struct mms_item oplus_cpa_item[] = {
	{
		.desc = {
			.item_id = CPA_ITEM_CHG_TYPE,
			.update = oplus_cpa_update_chg_type,
		}
	},
	{
		.desc = {
			.item_id = CPA_ITEM_ALLOW,
			.update = oplus_cpa_update_allow,
		}
	},
	{
		.desc = {
			.item_id = CPA_ITEM_TIMEOUT,
			.update = oplus_cpa_update_timeout,
		}
	},
	{
		.desc = {
			.item_id = CPA_ITEM_POWER,
			.update = oplus_cpa_update_power,
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
			.dead_thr_enable = true,
			.dead_zone_thr = 1,
#endif
		}
	},
};

static const struct oplus_mms_desc oplus_cpa_desc = {
	.name = "cpa",
	.type = OPLUS_MMS_TYPE_CPA,
	.item_table = oplus_cpa_item,
	.item_num = ARRAY_SIZE(oplus_cpa_item),
	.update_items = NULL,
	.update_items_num = 0,
	.update_interval = 0, /* ms */
	.update = oplus_cpa_update,
};

static int oplus_cpa_topic_init(struct oplus_cpa *chip)
{
	struct oplus_mms_config mms_cfg = {};
	int rc;

	mms_cfg.drv_data = chip;
	mms_cfg.of_node = chip->dev->of_node;

	if (of_property_read_bool(mms_cfg.of_node,
				  "oplus,topic-update-interval")) {
		rc = of_property_read_u32(mms_cfg.of_node,
					  "oplus,topic-update-interval",
					  &mms_cfg.update_interval);
		if (rc < 0)
			mms_cfg.update_interval = 0;
	}

	chip->cpa_topic = devm_oplus_mms_register(chip->dev, &oplus_cpa_desc, &mms_cfg);
	if (IS_ERR(chip->cpa_topic)) {
		chg_err("Couldn't register cpa topic\n");
		rc = PTR_ERR(chip->cpa_topic);
		return rc;
	}

	return 0;
}

#define DEFAULT_REGION_ID 0xFF
static bool oplus_cpa_regionid_from_cmdline(struct oplus_cpa *chip)
{
	struct device_node *np;
	const char *bootparams = NULL;
	char *str;
	int temp_region = 0;
	int ret = 0;

	if (chip == NULL)
		return false;

	if (chip->region_id != DEFAULT_REGION_ID) {
		return true;
	} else {
		np = of_find_node_by_path("/chosen");
		if (np) {
			ret = of_property_read_string(np, "bootargs", &bootparams);
			if (!bootparams || ret < 0) {
				chg_err("failed to get bootargs property");
				return false;
			}

			str = strstr(bootparams, "oplus_region=");
			if (str) {
				str += strlen("oplus_region=");
				ret = get_option(&str, &temp_region);
				if (ret == 1) {
					chip->region_id = temp_region & 0xFF;
					chg_info("oplus_region=0x%02x", chip->region_id);
					return true;
				}
			}
		}
	}
	return false;
}

static int oplus_cpa_parse_dt(struct oplus_cpa *cpa)
{
#define PPS_REGION_COUNT_MAX 16

	struct device_node *cpa_node = oplus_get_node_by_child_gauge(cpa->dev->of_node);
	struct device_node *node = cpa_node;
	struct device_node *child;
	int rc, num, i;
	uint32_t data, power;
	u8 pps_region_list[PPS_REGION_COUNT_MAX];
	int len;
	uint32_t wait_data, wait_time;

	if (oplus_cpa_regionid_from_cmdline(cpa) && cpa->region_id != DEFAULT_REGION_ID) {
		chg_info("region_id = 0x%02x", cpa->region_id);
		for_each_child_of_node(cpa_node, child) {
			rc = of_property_count_elems_of_size(child, "oplus,region_id", sizeof(u8));
			if (rc > 0) {
				len = rc <= PPS_REGION_COUNT_MAX ? rc : PPS_REGION_COUNT_MAX;
				rc = of_property_read_u8_array(child, "oplus,region_id", pps_region_list, len);
				if (rc < 0) {
					chg_err("parse %s region_id failed, rc=%d", child->name, rc);
					continue;
				} else {
					for (i = 0; i < len; i++) {
						if (pps_region_list[i] == cpa->region_id) {
							node = child;
							chg_info("got the region node: %s", child->name);
							goto FOUND_NODE;
						}
					}
				}
			} else {
				chg_err("get size of %s reogin_id failed, rc=%d", child->name, rc);
				continue;
			}
		}
	}

FOUND_NODE:
	num = of_property_count_elems_of_size(node, "oplus,protocol_list", sizeof(uint32_t));
	if (num < 0) {
		chg_err("read oplus,protocol_list failed, rc=%d\n", num);
		num = 0;
	} else if ((num >> 1) > CHG_PROTOCOL_MAX) {
		chg_err("too many items in \"oplus,protocol_list\"\n");
		num = CHG_PROTOCOL_MAX;
	} else {
		num /= 2;
	}

	for (i = 0; i < num; i++) {
		cpa->protocol_prio_table[i].type = CHG_PROTOCOL_INVALID;
		cpa->protocol_prio_table[i].max_power_mw = 0;
		cpa->protocol_prio_table[i].max_power_dynamic = 0;
		cpa->protocol_prio_table[i].power_mw = 0;
		rc = of_property_read_u32_index(node, "oplus,protocol_list", i * 2, &data);
		if (rc < 0) {
			chg_err("read oplus,protocol_list index %d failed, rc=%d\n", i * 2, rc);
			continue;
		} else {
			if (data >= CHG_PROTOCOL_MAX) {
				chg_err("oplus,protocol_list index %d data error, data=%u\n", i, data);
				continue;
			} else {
				cpa->protocol_prio_table[i].type = data;
				cpa->protocol_supported_type |= BIT(data);
			}
		}

		rc = of_property_read_u32_index(node, "oplus,protocol_list", i * 2 + 1, &power);
		if (rc < 0) {
			chg_err("read oplus,protocol_list index %d failed, rc=%d\n", i * 2 + 1, rc);
			cpa->protocol_prio_table[i].type = CHG_PROTOCOL_INVALID;
			continue;
		} else {
			/* convert from w to mw */
			power *= 1000;
			cpa->protocol_prio_table[i].max_power_mw = power;
			cpa->protocol_prio_table[i].max_power_dynamic = power;

			/*
			 * The following protocols have fixed power and do not
			 * require secondary power information acquisition
			 */
			switch (cpa->protocol_prio_table[i].type) {
			case CHG_PROTOCOL_PD:
			case CHG_PROTOCOL_QC:
			case CHG_PROTOCOL_BC12:
				cpa->protocol_prio_table[i].power_mw = power;
				break;
			default:
				break;
			}
		}
	}
	if (i < CHG_PROTOCOL_MAX) {
		cpa->protocol_prio_table[i].type = CHG_PROTOCOL_BC12;
		cpa->protocol_prio_table[i].max_power_mw = 10000;
		cpa->protocol_prio_table[i].max_power_dynamic = 10000;
		cpa->protocol_prio_table[i].power_mw = 10000;
		cpa->protocol_supported_type |= BIT(CHG_PROTOCOL_BC12);
		for (++i; i < CHG_PROTOCOL_MAX; i++) {
			cpa->protocol_prio_table[i].type = CHG_PROTOCOL_INVALID;
			cpa->protocol_prio_table[i].max_power_mw = 0;
			cpa->protocol_prio_table[i].max_power_dynamic = 0;
			cpa->protocol_prio_table[i].power_mw = 0;
		}
	}

	num = of_property_count_elems_of_size(node, "oplus,default_protocol_list", sizeof(uint32_t));
	if (num < 0) {
		chg_err("read oplus,default_protocol_list failed, rc=%d\n", num);
		num = 0;
	} else if (num > CHG_PROTOCOL_MAX) {
		chg_err("too many items in oplus,default_protocol_list\n");
		num = CHG_PROTOCOL_MAX;
	}
	for (i = 0; i < num; i++) {
		rc = of_property_read_u32_index(node, "oplus,default_protocol_list", i, &data);
		if (rc < 0) {
			chg_err("read oplus,default_protocol_list index %d failed, rc=%d\n", i, rc);
		} else {
			if (data >= CHG_PROTOCOL_MAX)
				chg_err("oplus,default_protocol_list index %d data error, data=%u\n", i, data);
			else
				cpa->default_protocol_type |= BIT(data);
		}
	}

	/* not necessary for every project*/
	num = of_property_count_elems_of_size(node, "oplus,protocol_wait_list", sizeof(uint32_t));
	if (num < 0) {
		chg_err("read oplus,protocol_wait_list failed, rc=%d\n", num);
		num = 0;
	} else if ((num >> 1) > CHG_PROTOCOL_MAX) {
		chg_err("too many items in \"oplus,protocol_wait_list\"\n");
		num = CHG_PROTOCOL_MAX;
		cpa->protocol_wait_support = true;
	} else {
		num /= 2;
		cpa->protocol_wait_support = true;
	}
	cpa->protocol_wait_cnt = num;

	for (i = 0; i < cpa->protocol_wait_cnt; i++) {
		cpa->protocol_wait_table[i].type = CHG_PROTOCOL_INVALID;
		cpa->protocol_wait_table[i].time = PROTOCAL_WAIT_TIMEOUT_MS;
		rc = of_property_read_u32_index(node, "oplus,protocol_wait_list", i * 2, &wait_data);
		if (rc < 0) {
			chg_err("read oplus,protocol_wait_list index %d failed, rc=%d\n", i * 2, rc);
			continue;
		} else {
			if (wait_data >= CHG_PROTOCOL_MAX) {
				chg_err("oplus,protocol_wait_list index %d wait_data error, wait_data=%u\n", i, wait_data);
				continue;
			} else {
				cpa->protocol_wait_table[i].type = wait_data;
			}
		}

		rc = of_property_read_u32_index(node, "oplus,protocol_wait_list", i * 2 + 1, &wait_time);
		if (rc < 0) {
			chg_err("read oplus,protocol_wait_list index %d failed, rc=%d\n", i * 2 + 1, rc);
			cpa->protocol_prio_table[i].type = CHG_PROTOCOL_INVALID;
			continue;
		} else {
			cpa->protocol_wait_table[i].time = wait_time;
		}
	}

	rc = of_property_read_u32(node, "oplus,bc12_check_timeout_ms", &cpa->bc12_check_timeout_ms);
	if (rc < 0) {
		chg_err("read oplus,bc12_check_timeout_ms failed");
		cpa->bc12_check_timeout_ms = 0;
	} else {
		chg_info("bc12_check_timeout_ms=%u", cpa->bc12_check_timeout_ms);
	}

	return 0;
}

#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
#include "config/dynamic_cfg/oplus_cpa_cfg.h"
#endif

static int oplus_cpa_probe(struct platform_device *pdev)
{
	struct oplus_cpa *cpa;
	int rc;

	cpa = devm_kzalloc(&pdev->dev, sizeof(struct oplus_cpa), GFP_KERNEL);
	if (cpa == NULL) {
		chg_err("alloc cpa buffer error\n");
		return 0;
	}
	cpa->dev = &pdev->dev;
	platform_set_drvdata(pdev, cpa);

	cpa->current_protocol_type = CHG_PROTOCOL_INVALID;
	cpa->default_protocol_type = BIT(CHG_PROTOCOL_BC12);
	cpa->protocol_to_be_switched = 0;
	cpa->protocol_disable_mask = 0;
	cpa->protocol_supported_type = 0;
	cpa->request_pending = false;
	cpa->ready_protocol_type = 0;
	cpa->request_locked = false;
	cpa->status_reset = true;
	cpa->region_id = DEFAULT_REGION_ID;
	WRITE_ONCE(cpa->bc12_completed, false);
	mutex_init(&cpa->cpa_request_lock);
	mutex_init(&cpa->start_lock);
	init_completion(&cpa->pd_completed_ack);
	init_waitqueue_head(&cpa->bc12_wq);
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	mutex_init(&cpa->keep.ready_lock);
#endif

	oplus_cpa_parse_dt(cpa);
	INIT_WORK(&cpa->protocol_switch_work, oplus_cpa_protocol_switch_work);
	INIT_WORK(&cpa->chg_type_change_work, oplus_cpa_chg_type_change_work);
	INIT_WORK(&cpa->fast_chg_type_change_work, oplus_cpa_fast_chg_type_change_work);
	INIT_WORK(&cpa->wired_offline_work, oplus_cpa_wired_offline_work);
	INIT_WORK(&cpa->wired_online_work, oplus_cpa_wired_online_work);
	INIT_WORK(&cpa->switch_end_work, oplus_cpa_switch_end_work);
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	INIT_WORK(&cpa->power_change_work, oplus_cpa_power_change_work);
	INIT_WORK(&cpa->state_keep_wired_offline_work, oplus_cpa_state_keep_wired_offline_work);
#endif
	INIT_DELAYED_WORK(&cpa->protocol_switch_timeout_work, oplus_cpa_protocol_switch_timeout_work);
	INIT_DELAYED_WORK(&cpa->protocol_ready_timeout_work, oplus_cpa_protocol_ready_timeout_work);

	cpa->req_lock_votable = create_votable("CPA_REQ_LOCK", VOTE_SET_ANY,
				oplus_cpa_request_lock_vote_callback,
				cpa);
	if (IS_ERR(cpa->req_lock_votable)) {
		rc = PTR_ERR(cpa->req_lock_votable);
		cpa->req_lock_votable = NULL;
		goto votable_init_err;
	}
	vote(cpa->req_lock_votable, DEF_VOTER, true, 1, false);

	rc = oplus_cpa_topic_init(cpa);
	if (rc < 0)
		goto topic_init_err;

	oplus_mms_wait_topic("wired", oplus_cpa_subscribe_wired_topic, cpa);
	oplus_mms_wait_topic("vooc", oplus_cpa_subscribe_vooc_topic, cpa);
	oplus_mms_wait_topic("ufcs", oplus_cpa_subscribe_ufcs_topic, cpa);
	oplus_mms_wait_topic("pps", oplus_cpa_subscribe_pps_topic, cpa);
	oplus_mms_wait_topic("retention", oplus_cpa_subscribe_retention_topic, cpa);
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	oplus_mms_wait_topic("state_keep", oplus_cpa_subscribe_keep_topic, cpa);
#endif

	schedule_delayed_work(&cpa->protocol_ready_timeout_work,
		msecs_to_jiffies(PROTOCAL_READY_TIMEOUT_MS));

#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
	(void)oplus_cpa_reg_debug_config(cpa);
#endif

	return 0;

topic_init_err:
	destroy_votable(cpa->req_lock_votable);
votable_init_err:
	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, cpa);
	return rc;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void oplus_cpa_remove(struct platform_device *pdev)
#else
static int oplus_cpa_remove(struct platform_device *pdev)
#endif
{
	struct oplus_cpa *cpa = platform_get_drvdata(pdev);

#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
	oplus_cpa_unreg_debug_config(cpa);
#endif

	if (!IS_ERR_OR_NULL(cpa->retention_subs))
		oplus_mms_unsubscribe(cpa->retention_subs);
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	if (!IS_ERR_OR_NULL(cpa->keep_subs))
		oplus_mms_unsubscribe(cpa->keep_subs);
#endif
	destroy_votable(cpa->req_lock_votable);
	devm_kfree(&pdev->dev, cpa);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return 0;
#endif
}

static const struct of_device_id oplus_cpa_match[] = {
	{ .compatible = "oplus,cpa" },
	{},
};

static struct platform_driver oplus_cpa_driver = {
	.driver		= {
		.name = "oplus-cpa",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(oplus_cpa_match),
	},
	.probe		= oplus_cpa_probe,
	.remove		= oplus_cpa_remove,
};

/* never return an error value */
static __init int oplus_cpa_init(void)
{
#if __and(IS_BUILTIN(CONFIG_OPLUS_CHG), IS_BUILTIN(CONFIG_OPLUS_CHG_V2))
	struct device_node *node;

	node = of_find_node_by_path("/soc/oplus_chg_core");
	if (node == NULL)
		return 0;
	if (!of_property_read_bool(node, "oplus,chg_framework_v2"))
		return 0;
#endif /* CONFIG_OPLUS_CHG_V2 */
	return platform_driver_register(&oplus_cpa_driver);
}

static __exit void oplus_cpa_exit(void)
{
#if __and(IS_BUILTIN(CONFIG_OPLUS_CHG), IS_BUILTIN(CONFIG_OPLUS_CHG_V2))
	struct device_node *node;

	node = of_find_node_by_path("/soc/oplus_chg_core");
	if (node == NULL)
		return;
	if (!of_property_read_bool(node, "oplus,chg_framework_v2"))
		return;
#endif /* CONFIG_OPLUS_CHG_V2 */
	platform_driver_unregister(&oplus_cpa_driver);
}

oplus_chg_module_register(oplus_cpa);

/* charge protocol arbitration module api */
bool oplus_cpa_support(void)
{
	struct device_node *node;

	node = of_find_node_by_path("/soc/oplus,cpa");
	if (node == NULL)
		return false;
	return true;
}

int oplus_cpa_request(struct oplus_mms *topic, enum oplus_chg_protocol_type type)
{
	struct oplus_cpa *cpa;
	int rc;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -EINVAL;
	}
	if ((type >= CHG_PROTOCOL_MAX) || (type <= CHG_PROTOCOL_INVALID)) {
		chg_err("unsupported protocol type, protocol=%d\n", type);
		return -EINVAL;
	}

	cpa = oplus_mms_get_drvdata(topic);
	if (!cpa->retention_state) {
		if (!cpa->wired_online) {
			chg_err("wired is offline\n");
			return -EFAULT;
		}
	}

	if (type == CHG_PROTOCOL_PPS && !(oplus_cpa_is_supported_protocol(cpa, type))) {
		chg_info("no support pps forced conversion to pd\n");
		type = CHG_PROTOCOL_PD;
	}

	chg_info("%s protocol identify request\n", get_protocol_name_str(type));

	mutex_lock(&cpa->cpa_request_lock);
	rc = protocol_identify_request(cpa, BIT(type));
	mutex_unlock(&cpa->cpa_request_lock);
	return rc;
}

int oplus_cpa_protocol_ready(struct oplus_mms *topic, enum oplus_chg_protocol_type type)
{
	struct oplus_cpa *cpa;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -EINVAL;
	}
	if ((type >= CHG_PROTOCOL_MAX) || (type <= CHG_PROTOCOL_INVALID)) {
		chg_err("unsupported protocol type, protocol=%d\n", type);
		return -EINVAL;
	}

	cpa = oplus_mms_get_drvdata(topic);
	set_bit(type, &cpa->ready_protocol_type);

	chg_info("%s ready, ready_protocol_type=0x%lx, default_protocol_type=0x%x\n",
		 get_protocol_name_str(type),
		 cpa->ready_protocol_type, cpa->default_protocol_type);
	if ((cpa->ready_protocol_type & cpa->default_protocol_type) != cpa->default_protocol_type)
		return 0;

	cancel_delayed_work_sync(&cpa->protocol_ready_timeout_work);
	vote(cpa->req_lock_votable, DEF_VOTER, false, 0, false);

	return 0;
}

int oplus_cpa_switch_start(struct oplus_mms *topic, enum oplus_chg_protocol_type type)
{
	struct oplus_cpa *cpa;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -EINVAL;
	}
	cpa = oplus_mms_get_drvdata(topic);
	if (type != cpa->current_protocol_type)
		return -EINVAL;

	mutex_lock(&cpa->start_lock);
	cancel_delayed_work_sync(&cpa->protocol_switch_timeout_work);
	WRITE_ONCE(cpa->started, true);
	mutex_unlock(&cpa->start_lock);
	chg_info("%s protocol identify start\n", get_protocol_name_str(type));

	return 0;
}

int oplus_cpa_switch_end(struct oplus_mms *topic, enum oplus_chg_protocol_type type)
{
	struct oplus_cpa *cpa;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -EINVAL;
	}
	cpa = oplus_mms_get_drvdata(topic);
	if (type != cpa->current_protocol_type)
		return -EINVAL;

	return schedule_work(&cpa->switch_end_work);

}

int oplus_cpa_get_high_prio_wired_type(struct oplus_mms *topic, struct protocol_map *map)
{
	struct oplus_cpa *cpa;
	enum oplus_chg_protocol_type type;
	uint32_t protocol;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return OPLUS_CHG_USB_TYPE_UNKNOWN;
	}
	if (map == NULL) {
		chg_err("protocol_map is NULL\n");
		return OPLUS_CHG_USB_TYPE_UNKNOWN;
	}

	cpa = oplus_mms_get_drvdata(topic);
	protocol = map->protocol;
	protocol &= ~READ_ONCE(cpa->protocol_disable_mask);
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	protocol &= ~READ_ONCE(cpa->keep.protocol_disable_mask);
	if (cpa->protocol_disable_mask != 0 || cpa->keep.protocol_disable_mask != 0)
		chg_info("protocol_disable_mask=0x%lx, keep.protocol_disable_mask=0x%lx\n",
			cpa->protocol_disable_mask, cpa->keep.protocol_disable_mask);
#endif
	type = get_highest_priority_protocol_type(cpa, protocol);
	if (type == CHG_PROTOCOL_INVALID || type >= CHG_PROTOCOL_MAX)
		return OPLUS_CHG_USB_TYPE_UNKNOWN;
	return map->type[type];
}

enum oplus_chg_protocol_type oplus_cpa_curr_high_prio_protocol_type(struct oplus_mms *topic)
{
	struct oplus_cpa *cpa;
	enum oplus_chg_protocol_type type;
	uint32_t protocol_to_be_switched;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return CHG_PROTOCOL_INVALID;
	}
	cpa = oplus_mms_get_drvdata(topic);

	protocol_to_be_switched = READ_ONCE(cpa->protocol_to_be_switched);
	if ((cpa->current_protocol_type > CHG_PROTOCOL_INVALID) &&
	    (cpa->current_protocol_type < CHG_PROTOCOL_MAX))
		protocol_to_be_switched |= BIT(cpa->current_protocol_type);
	protocol_to_be_switched &= ~READ_ONCE(cpa->protocol_disable_mask);
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	protocol_to_be_switched &= ~READ_ONCE(cpa->keep.protocol_disable_mask);
#endif
	type = get_highest_priority_protocol_type(cpa, protocol_to_be_switched);

	return type;
}

void oplus_cpa_protocol_add_type(struct protocol_map *map, int type)
{
	if (map == NULL)
		return;

	switch (type) {
	case OPLUS_CHG_USB_TYPE_QC2:
	case OPLUS_CHG_USB_TYPE_QC3:
		map->protocol |= BIT(CHG_PROTOCOL_QC);
		map->type[CHG_PROTOCOL_QC] = type;
		break;
	case OPLUS_CHG_USB_TYPE_PD:
	case OPLUS_CHG_USB_TYPE_PD_DRP:
		map->protocol |= BIT(CHG_PROTOCOL_PD);
		map->type[CHG_PROTOCOL_PD] = type;
		break;
	case OPLUS_CHG_USB_TYPE_PD_PPS:
		map->protocol |= BIT(CHG_PROTOCOL_PPS);
		map->type[CHG_PROTOCOL_PPS] = type;
		break;
	case OPLUS_CHG_USB_TYPE_VOOC:
	case OPLUS_CHG_USB_TYPE_SVOOC:
		map->protocol |= BIT(CHG_PROTOCOL_VOOC);
		map->type[CHG_PROTOCOL_VOOC] = type;
		break;
	case OPLUS_CHG_USB_TYPE_UFCS:
		map->protocol |= BIT(CHG_PROTOCOL_UFCS);
		map->type[CHG_PROTOCOL_UFCS] = type;
		break;
	case OPLUS_CHG_USB_TYPE_SDP:
	case OPLUS_CHG_USB_TYPE_DCP:
	case OPLUS_CHG_USB_TYPE_CDP:
	case OPLUS_CHG_USB_TYPE_ACA:
	case OPLUS_CHG_USB_TYPE_C:
	case OPLUS_CHG_USB_TYPE_APPLE_BRICK_ID:
	case OPLUS_CHG_USB_TYPE_PD_SDP:
		map->protocol |= BIT(CHG_PROTOCOL_BC12);
		map->type[CHG_PROTOCOL_BC12] = type;
		break;
	default:
		break;
	}
}

int oplus_cpa_protocol_set_max_power(struct oplus_mms *topic, enum oplus_chg_protocol_type type, int power_mw)
{
	struct oplus_cpa *cpa;
	int i;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -EINVAL;
	}
	if (type <= CHG_PROTOCOL_INVALID || type >= CHG_PROTOCOL_MAX) {
		chg_err("invalid protocol type, type=%d\n", type);
		return -EINVAL;
	}
	cpa = oplus_mms_get_drvdata(topic);

	for (i = 0; i < CHG_PROTOCOL_MAX; i++) {
		if (cpa->protocol_prio_table[i].type == type) {
			if (power_mw > cpa->protocol_prio_table[i].max_power_mw)
				power_mw = cpa->protocol_prio_table[i].max_power_mw;
			cpa->protocol_prio_table[i].max_power_dynamic = power_mw;
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
			cpa->keep.protocol_max_power[type] = power_mw;
#endif
			chg_info("set %s max power to %d", get_protocol_name_str(type), power_mw);
			if (cpa->protocol_prio_table[i].power_mw > power_mw)
				oplus_cpa_protocol_set_power(topic, type, power_mw);
			return 0;
		}
	}

	chg_debug("unsupported protocol type, type=%d\n", type);
	return -ENOTSUPP;
}

int oplus_cpa_protocol_restore_max_power(struct oplus_mms *topic, enum oplus_chg_protocol_type type)
{
	struct oplus_cpa *cpa;
	int i;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -EINVAL;
	}
	if (type <= CHG_PROTOCOL_INVALID || type >= CHG_PROTOCOL_MAX) {
		chg_err("invalid protocol type, type=%d\n", type);
		return -EINVAL;
	}
	cpa = oplus_mms_get_drvdata(topic);

	for (i = 0; i < CHG_PROTOCOL_MAX; i++) {
		if (cpa->protocol_prio_table[i].type == type) {
			cpa->protocol_prio_table[i].max_power_dynamic =
				cpa->protocol_prio_table[i].max_power_mw;
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
			cpa->keep.protocol_max_power[type] =
				cpa->protocol_prio_table[i].max_power_mw;
#endif
			return 0;
		}
	}

	chg_debug("unsupported protocol type, type=%d\n", type);
	return -ENOTSUPP;
}

int oplus_cpa_protocol_restore_max_power_all(struct oplus_mms *topic)
{
	enum oplus_chg_protocol_type type;
	struct oplus_cpa *cpa;
	int i;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -EINVAL;
	}
	cpa = oplus_mms_get_drvdata(topic);

	for (i = 0; i < CHG_PROTOCOL_MAX; i++) {
		type = cpa->protocol_prio_table[i].type;
		cpa->protocol_prio_table[i].max_power_dynamic =
			cpa->protocol_prio_table[i].max_power_mw;
		if (type != CHG_PROTOCOL_INVALID) {
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
			cpa->keep.protocol_max_power[type] =
				cpa->protocol_prio_table[i].max_power_mw;
#endif
		}
	}

	return 0;
}

int oplus_cpa_protocol_set_power(struct oplus_mms *topic, enum oplus_chg_protocol_type type, int power_mw)
{
	struct oplus_cpa *cpa;
	struct mms_msg *msg;
	int rc = 0;
	int i;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -EINVAL;
	}
	if (type <= CHG_PROTOCOL_INVALID || type >= CHG_PROTOCOL_MAX) {
		chg_err("invalid protocol type, type=%d\n", type);
		return -EINVAL;
	}
	cpa = oplus_mms_get_drvdata(topic);

	for (i = 0; i < CHG_PROTOCOL_MAX; i++) {
		if (cpa->protocol_prio_table[i].type == type) {
			if (power_mw > cpa->protocol_prio_table[i].max_power_dynamic)
				power_mw = cpa->protocol_prio_table[i].max_power_dynamic;
			cpa->protocol_prio_table[i].power_mw = power_mw;
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
			cpa->keep.protocol_power[type] = power_mw;
#endif
			chg_info("set %s power to %d", get_protocol_name_str(type), power_mw);

			msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, CPA_ITEM_POWER);
			if (msg == NULL) {
				chg_err("alloc msg error\n");
			} else {
				rc = oplus_mms_publish_msg(cpa->cpa_topic, msg);
				if (rc < 0) {
					chg_err("publish cpa power msg error, rc=%d\n", rc);
					kfree(msg);
				}
			}

			return 0;
		}
	}

	chg_debug("unsupported protocol type, type=%d\n", type);
	return -ENOTSUPP;
}

int oplus_cpa_protocol_clear_power(struct oplus_mms *topic, enum oplus_chg_protocol_type type)
{
	struct oplus_cpa *cpa;
	int i;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -EINVAL;
	}
	if (type <= CHG_PROTOCOL_INVALID || type >= CHG_PROTOCOL_MAX) {
		chg_err("invalid protocol type, type=%d\n", type);
		return -EINVAL;
	}
	cpa = oplus_mms_get_drvdata(topic);

	for (i = 0; i < CHG_PROTOCOL_MAX; i++) {
		if (cpa->protocol_prio_table[i].type == type) {
			switch (type) {
			case CHG_PROTOCOL_PD:
			case CHG_PROTOCOL_QC:
			case CHG_PROTOCOL_BC12:
				cpa->protocol_prio_table[i].power_mw = cpa->protocol_prio_table[i].max_power_mw;
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
				cpa->keep.protocol_power[type] = cpa->protocol_prio_table[i].max_power_mw;
#endif
				break;
			default:
				cpa->protocol_prio_table[i].power_mw = 0;
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
				cpa->keep.protocol_power[type] = 0;
#endif
				break;
			}
			return 0;
		}
	}

	chg_err("unsupported protocol type, type=%d\n", type);
	return -ENOTSUPP;
}

int oplus_cpa_protocol_clear_power_all(struct oplus_mms *topic)
{
	enum oplus_chg_protocol_type type;
	struct oplus_cpa *cpa;
	int i;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -EINVAL;
	}
	cpa = oplus_mms_get_drvdata(topic);

	for (i = 0; i < CHG_PROTOCOL_MAX; i++) {
		type = cpa->protocol_prio_table[i].type;
		switch (type) {
		case CHG_PROTOCOL_PD:
		case CHG_PROTOCOL_QC:
		case CHG_PROTOCOL_BC12:
			cpa->protocol_prio_table[i].power_mw = cpa->protocol_prio_table[i].max_power_mw;
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
			cpa->keep.protocol_power[type] = cpa->protocol_prio_table[i].max_power_mw;
#endif
			break;
		default:
			cpa->protocol_prio_table[i].power_mw = 0;
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
			if (type != CHG_PROTOCOL_INVALID)
				cpa->keep.protocol_power[type] = 0;
#endif
			break;
		}
	}

	return 0;
}

int oplus_cpa_protocol_get_power(struct oplus_mms *topic, enum oplus_chg_protocol_type type)
{
	struct oplus_cpa *cpa;
	int i;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -EINVAL;
	}
	if (type <= CHG_PROTOCOL_INVALID || type >= CHG_PROTOCOL_MAX) {
		chg_err("invalid protocol type, type=%d\n", type);
		return -EINVAL;
	}
	cpa = oplus_mms_get_drvdata(topic);

	for (i = 0; i < CHG_PROTOCOL_MAX; i++) {
		if (cpa->protocol_prio_table[i].type == type)
			return cpa->protocol_prio_table[i].power_mw;
	}

	chg_err("unsupported protocol type, type=%d\n", type);
	return -ENOTSUPP;
}

int oplus_cpa_protocol_get_max_power(struct oplus_mms *topic)
{
	struct oplus_cpa *cpa;
	int i;
	int project_max_power_mw = 0;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -EINVAL;
	}
	cpa = oplus_mms_get_drvdata(topic);

	for (i = 0; i < CHG_PROTOCOL_MAX; i++) {
		if (cpa->protocol_prio_table[i].max_power_dynamic > project_max_power_mw)
			project_max_power_mw = cpa->protocol_prio_table[i].max_power_dynamic;
	}

	return project_max_power_mw;
}

int oplus_cpa_protocol_get_max_power_by_type(struct oplus_mms *topic, enum oplus_chg_protocol_type type)
{
	struct oplus_cpa *cpa;
	int i;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -EINVAL;
	}

	if ((type >= CHG_PROTOCOL_MAX) || (type <= CHG_PROTOCOL_INVALID)) {
		chg_err("unsupported protocol type, type=%d\n", type);
		return -EINVAL;
	}
	cpa = oplus_mms_get_drvdata(topic);

	for (i = 0; i < CHG_PROTOCOL_MAX; i++) {
		if (cpa->protocol_prio_table[i].type == type)
			return cpa->protocol_prio_table[i].max_power_dynamic;
	}

	chg_err("unsupported protocol type, type=%d\n", type);
	return -ENOTSUPP;
}

static int oplus_cpa_get_adapter_power(struct oplus_cpa *chip)
{
	int power = 0;
	union mms_msg_data data = { 0 };
	int cur_sid = 0;

	if (chip->ufcs_online) {
		power = oplus_ufcs_get_ufcs_power(chip->ufcs_topic) * 1000;
	} else if (chip->pps_online || chip->pps_online_keep) {
		power = oplus_pps_get_adapter_power_mw(chip->pps_topic);
	} else if (chip->vooc_online) {
		cur_sid = chip->vooc_sid;
		chg_info("cur_sid = 0x%08x\n", cur_sid);

		if (sid_to_adapter_id(cur_sid) == 0) {
			if (sid_to_adapter_chg_type(cur_sid) == CHARGER_TYPE_VOOC)
				power = 20000;
			else if (sid_to_adapter_chg_type(cur_sid) == CHARGER_TYPE_SVOOC)
				power = 50000;
		} else {
			power = sid_to_adapter_power(cur_sid) * 1000;
		}
	} else {
		switch (chip->wired_type) {
		case OPLUS_CHG_USB_TYPE_PD_PPS:
			oplus_mms_get_item_data(chip->cpa_topic, CPA_ITEM_ALLOW, &data, false);
			if (data.intval == CHG_PROTOCOL_PPS) {
				/* switching pps */
				power = 10000;
				break;
			}
			fallthrough;
		case OPLUS_CHG_USB_TYPE_QC2:
		case OPLUS_CHG_USB_TYPE_QC3:
		case OPLUS_CHG_USB_TYPE_PD:
		case OPLUS_CHG_USB_TYPE_PD_DRP:
			power = 18000;
			break;
		case OPLUS_CHG_USB_TYPE_SDP:
			power = 2500;
			break;
		case OPLUS_CHG_USB_TYPE_DCP:
		case OPLUS_CHG_USB_TYPE_ACA:
		case OPLUS_CHG_USB_TYPE_C:
		case OPLUS_CHG_USB_TYPE_PD_SDP:
		case OPLUS_CHG_USB_TYPE_APPLE_BRICK_ID:
			power = 10000;
			break;
		case OPLUS_CHG_USB_TYPE_CDP:
			power = 7500;
			break;
		default:
			break;
		}
	}
	return power;
}

static int oplus_cpa_get_project_watt(struct oplus_cpa *chip)
{
	int project_power = 0;

	project_power = oplus_cpa_protocol_get_max_power(chip->cpa_topic);
	if (project_power < 0)
		project_power = 0;

	return project_power;
}

int oplus_cpa_get_actual_used_power(struct oplus_mms *topic)
{
	struct oplus_cpa *chip;
	int adapter_power = 0;
	int project_power = 0;
	int cpa_power = 0;
	int pps_or_ufcs_ing = 0;
	int pps_or_ufcs_power = 0;
	int actual_power_used = 0;
	static int pre_cpa_power = 0;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(topic);

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	adapter_power = oplus_cpa_get_adapter_power(chip);
	project_power = oplus_cpa_get_project_watt(chip);

	if (chip->ufcs_online) {
		if (chip->ufcs_oplus_adapter)
			pps_or_ufcs_ing = oplus_ufcs_adapter_id_to_protocol_type(chip->ufcs_adapter_id);
		else
			pps_or_ufcs_ing = PROTOCOL_CHARGING_UFCS_THIRD;

		pps_or_ufcs_power = oplus_ufcs_get_ufcs_power(chip->ufcs_topic);
	} else 	if (chip->pps_online || chip->pps_online_keep) {
		if (chip->pps_oplus_adapter) {
			pps_or_ufcs_ing = PROTOCOL_CHARGING_PPS_OPLUS;
		} else {
			pps_or_ufcs_ing = PROTOCOL_CHARGING_PPS_THIRD;
		}
		pps_or_ufcs_power = oplus_pps_get_charging_power_watt(chip->pps_topic);
	}

	if (pps_or_ufcs_ing > 0)
		cpa_power = pps_or_ufcs_power * 1000;
	else
		cpa_power = min(adapter_power, project_power);

	if (cpa_power < 0)
		cpa_power = 0;

	if (pre_cpa_power != cpa_power) {
		pre_cpa_power = cpa_power;
		chg_info("%d %d %d %d, %d %d %d %d, %d %d\n",
			  adapter_power, project_power, chip->ufcs_online, chip->pps_online,
			  chip->ufcs_oplus_adapter, chip->pps_oplus_adapter, pps_or_ufcs_ing, pps_or_ufcs_power,
			  cpa_power, chip->ufcs_adapter_id);
	}

	actual_power_used = cpa_power;
	return actual_power_used;
}

int oplus_cpa_request_lock(struct oplus_mms *topic, const char *name)
{
	struct oplus_cpa *cpa;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -EINVAL;
	}
	if (name == NULL) {
		chg_err("name is NULL\n");
		return -EINVAL;
	}
	cpa = oplus_mms_get_drvdata(topic);

	return vote(cpa->req_lock_votable, name, true, 1, false);
}

int oplus_cpa_request_unlock(struct oplus_mms *topic, const char *name)
{
	struct oplus_cpa *cpa;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -EINVAL;
	}
	if (name == NULL) {
		chg_err("name is NULL\n");
		return -EINVAL;
	}
	cpa = oplus_mms_get_drvdata(topic);

	return vote(cpa->req_lock_votable, name, false, 0, false);
}

int oplus_cpa_protocol_disable(struct oplus_mms *topic, enum oplus_chg_protocol_type type)
{
	struct oplus_cpa *cpa;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -EINVAL;
	}

	if ((type >= CHG_PROTOCOL_MAX) || (type <= CHG_PROTOCOL_INVALID)) {
		chg_err("unsupported protocol type, type=%d\n", type);
		return -EINVAL;
	}
	cpa = oplus_mms_get_drvdata(topic);

	set_bit(type, &cpa->protocol_disable_mask);

	return 0;
}

int oplus_cpa_protocol_enable(struct oplus_mms *topic, enum oplus_chg_protocol_type type)
{
	struct oplus_cpa *cpa;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -EINVAL;
	}

	if ((type >= CHG_PROTOCOL_MAX) || (type <= CHG_PROTOCOL_INVALID)) {
		chg_err("unsupported protocol type, type=%d\n", type);
		return -EINVAL;
	}
	cpa = oplus_mms_get_drvdata(topic);

	clear_bit(type, &cpa->protocol_disable_mask);
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	clear_bit(type, &cpa->keep.protocol_disable_mask);
#endif

	return 0;
}

int oplus_cpa_protocol_enable_all(struct oplus_mms *topic)
{
	struct oplus_cpa *cpa;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -EINVAL;
	}
	cpa = oplus_mms_get_drvdata(topic);

	cpa->protocol_disable_mask = 0;
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	cpa->keep.protocol_disable_mask = 0;
#endif

	return 0;
}

bool oplus_cpa_protocol_check_enable(struct oplus_mms *topic, enum oplus_chg_protocol_type type)
{
	struct oplus_cpa *cpa;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return false;
	}
	cpa = oplus_mms_get_drvdata(topic);

	return oplus_cpa_is_enabled_protocol(cpa, type);
}
