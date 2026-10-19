// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025-2025 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[REC/SK/WDD]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/slab.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>

#include <oplus_chg.h>
#include <oplus_mms.h>
#include <oplus_mms_wired.h>
#include <oplus_chg_cpa.h>
#include <recovery/state_keep.h>

struct wdd_protocol_switch_info {
	enum oplus_chg_protocol_type protocol;
	uint32_t power_thr_mw;
	uint32_t high_power_count;
	uint32_t low_power_count;
};
#define WDD_PROTOCOL_SWITCH_INFO_ITEM_NUM 4

struct wdd_config {
	struct wdd_protocol_switch_info *switch_table;
	int switch_table_num;
	unsigned long support_protocol_mask;
};

struct wired_disconnect_detection {
	struct device_node *node;
	struct state_keep_client *client;

	struct oplus_mms *wired_topic;
	struct mms_subscribe *wired_subs;
	struct oplus_mms *keep_topic;
	struct mms_subscribe *keep_subs;
	struct oplus_mms *cpa_topic;
	struct mms_subscribe *cpa_subs;

	struct wdd_config config;
	struct work_struct wired_online_check_work;
	struct work_struct hw_detect_update_work;
	struct delayed_work hw_detect_check_work;
	struct delayed_work final_check_work;
	struct completion check_complete;

	bool check_start;
	bool keep_online;
	int hw_detect;
	unsigned int hw_detect_check_count;
	unsigned long disable_protocol_mask;

	bool wired_online;
};

static struct wired_disconnect_detection *g_wdd;

#define CHECK_MAX_TIME_MS		2000
#define HW_DETECT_CHECK_TIME_MS		50
#define HW_DETECT_CHECK_COUNT		3
#define DISCONNECT_TIME_THR_MS		2000

static void wdd_state_reset(struct wired_disconnect_detection *wdd)
{
	chg_info("reset\n");
	wdd->check_start = false;
	wdd->keep_online = false;
	wdd->hw_detect_check_count = 0;
	wdd->disable_protocol_mask = 0;
}

static bool wdd_is_invalid_protocol(enum oplus_chg_protocol_type protocol)
{
	if (protocol <= CHG_PROTOCOL_INVALID ||
	    protocol >= CHG_PROTOCOL_MAX)
		return true;
	return false;
}

static int wdd_client_reset(struct state_keep_client *client)
{
	struct wired_disconnect_detection *wdd = client->priv_data;

	if (wdd == NULL) {
		chg_err("wdd is null\n");
		return -EINVAL;
	}
	wdd_state_reset(wdd);

	return 0;
}

static int wdd_client_enable(struct state_keep_client *client)
{
	struct wired_disconnect_detection *wdd = client->priv_data;

	if (wdd == NULL) {
		chg_err("wdd is null\n");
		return -EINVAL;
	}
	wdd_state_reset(wdd);
	client->enabled = true;

	return 0;
}

static int wdd_client_disable(struct state_keep_client *client)
{
	client->enabled = false;

	return 0;
}

static int wdd_client_start_check(struct state_keep_client *client, enum oplus_chg_protocol_type protocol)
{
	struct wired_disconnect_detection *wdd = client->priv_data;

	if (wdd == NULL) {
		chg_err("wdd is null\n");
		return -EINVAL;
	}

	wdd->wired_online = false;
	cancel_delayed_work_sync(&wdd->hw_detect_check_work);
	wdd->check_start = true;
	wdd->keep_online = false;
	wdd->hw_detect_check_count = 0;
	reinit_completion(&wdd->check_complete);
	schedule_delayed_work(&wdd->hw_detect_check_work, 0);

	return 0;
}

static bool wdd_client_need_keep(struct state_keep_client *client, enum oplus_chg_protocol_type protocol)
{
	struct wired_disconnect_detection *wdd = client->priv_data;
	unsigned long left;

	if (wdd == NULL) {
		chg_err("wdd is null\n");
		return false;
	}

	if (wdd_is_invalid_protocol(protocol)) {
		chg_info("current_protocol is invalid\n");
		return false;
	}
	if ((wdd->config.support_protocol_mask & BIT(protocol)) == 0) {
		chg_debug("%s protocol is not supported\n", get_protocol_name_str(protocol));
		return false;
	}
	if (!wdd->check_start)
		return wdd->keep_online;
	left = wait_for_completion_timeout(&wdd->check_complete, msecs_to_jiffies(CHECK_MAX_TIME_MS));
	if (left == 0) {
		chg_err("check timeout\n");
		return false;
	}
	return wdd->keep_online;
}

static unsigned int wdd_client_switch_protocol(struct state_keep_client *client,
	unsigned int total_count, unsigned int protocol_count,
	enum oplus_chg_protocol_type protocol, int power)
{
	struct wired_disconnect_detection *wdd = client->priv_data;
	int i;
	int power_count;
	int power_thr;
	unsigned int switch_info = 0;

	if (wdd == NULL) {
		chg_err("wdd is null\n");
		return (SK_SWITCH_SWITCH_PROTOCOL | SK_SWITCH_DISABLE_PROTOCOL);
	}
	if (wdd_is_invalid_protocol(protocol)) {
		chg_err("current_protocol is invalid\n");
		return 0;
	}

	for (i = 0; i < wdd->config.switch_table_num; i++) {
		if (wdd->config.switch_table[i].protocol != protocol)
			continue;
		if ((power > wdd->config.switch_table[i].power_thr_mw) &&
		    !test_bit(protocol, &wdd->disable_protocol_mask)) {
			power_count = wdd->config.switch_table[i].high_power_count;
			power_thr = wdd->config.switch_table[i].power_thr_mw;
		} else {
			power_count = wdd->config.switch_table[i].low_power_count;
			power_thr = 0;
		}

		if (protocol_count <= power_count)
			break;

		if (power_thr > 0) {
			chg_info("set %s protocol power to %dmW\n",
				 get_protocol_name_str(protocol), power_thr);
			set_bit(protocol, &wdd->disable_protocol_mask);
			oplus_cpa_protocol_set_max_power(wdd->cpa_topic, protocol, power_thr);
			oplus_cpa_request(wdd->cpa_topic, protocol);
		} else {
			switch_info |= SK_SWITCH_DISABLE_PROTOCOL;
		}
		switch_info |= SK_SWITCH_SWITCH_PROTOCOL;
		break;
	}

	if (switch_info & SK_SWITCH_SWITCH_PROTOCOL) {
		if (protocol == CHG_PROTOCOL_BC12) {
			chg_info("restart wired disconnect detection\n");
			wdd->disable_protocol_mask = 0;
		}
	}
	return switch_info;
}

static void wdd_check_done(struct wired_disconnect_detection *wdd, bool keep)
{
	chg_info("keep=%d\n", keep);
	wdd->check_start = false;
	wdd->keep_online = keep;
	wdd->hw_detect_check_count = 0;
	complete_all(&wdd->check_complete);
	if (!keep)
		wdd->disable_protocol_mask = 0;
}

static void wdd_final_check_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct wired_disconnect_detection *wdd =
		container_of(dwork, struct wired_disconnect_detection, final_check_work);

	wdd_check_done(wdd, wdd->wired_online);
}

static void wdd_hw_detect_check_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct wired_disconnect_detection *wdd =
		container_of(dwork, struct wired_disconnect_detection, hw_detect_check_work);
	int hw_detect;
	unsigned int time;

	hw_detect = oplus_wired_get_hw_detect();
	chg_info("hw_detect=%d, check_count=%u\n", hw_detect, wdd->hw_detect_check_count);
	if (hw_detect != CC_DETECT_PLUGIN) {
		wdd_check_done(wdd, false);
		return;
	}

	wdd->hw_detect_check_count++;
	if (wdd->hw_detect_check_count < HW_DETECT_CHECK_COUNT && !wdd->wired_online)
		goto next;

	cancel_delayed_work_sync(&wdd->final_check_work);
	if (wdd->wired_online) {
		wdd_check_done(wdd, true);
	} else {
		time = DISCONNECT_TIME_THR_MS - (wdd->hw_detect_check_count - 1) * HW_DETECT_CHECK_TIME_MS;
		schedule_delayed_work(&wdd->final_check_work, msecs_to_jiffies(time));
	}

	return;

next:
	schedule_delayed_work(&wdd->hw_detect_check_work, msecs_to_jiffies(HW_DETECT_CHECK_TIME_MS));
}

static void wdd_wired_online_check_work(struct work_struct *work)
{
	struct wired_disconnect_detection *wdd =
		container_of(work, struct wired_disconnect_detection, wired_online_check_work);
	union mms_msg_data data = { 0 };
	int rc;

	rc = oplus_mms_get_item_data(wdd->wired_topic, WIRED_ITEM_ONLINE, &data, false);
	if (rc < 0) {
		chg_err("failed to get wired online, rc=%d\n", rc);
		return;
	}
	wdd->wired_online = !!data.intval;
	chg_info("wired online: %d\n", wdd->wired_online);

	if (wdd->wired_online) {
		if (delayed_work_pending(&wdd->hw_detect_check_work)) {
			cancel_delayed_work_sync(&wdd->hw_detect_check_work);
			wdd_check_done(wdd, true);
		} else if (delayed_work_pending(&wdd->final_check_work)) {
			cancel_delayed_work_sync(&wdd->final_check_work);
			wdd_check_done(wdd, true);
		}
	}
}

static void wdd_hw_detect_update_work(struct work_struct *work)
{
	struct wired_disconnect_detection *wdd =
		container_of(work, struct wired_disconnect_detection, hw_detect_update_work);
	union mms_msg_data data = { 0 };
	int hw_detect;
	int rc;

	rc = oplus_mms_get_item_data(wdd->wired_topic, WIRED_ITEM_CC_DETECT, &data, false);
	if (rc < 0) {
		chg_err("failed to get wired hw_detect, rc=%d\n", rc);
		return;
	}
	hw_detect = data.intval;
	chg_info("hw_detect: %d\n", hw_detect);

	if (hw_detect == CC_DETECT_PLUGIN)
		return;

	if (delayed_work_pending(&wdd->final_check_work)) {
		cancel_delayed_work_sync(&wdd->final_check_work);
		wdd_check_done(wdd, false);
	}
}

static void wdd_wired_subs_callback(struct mms_subscribe *subs,
				    enum mms_msg_type type, u32 id, bool sync)
{
	struct wired_disconnect_detection *wdd = subs->priv_data;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WIRED_ITEM_ONLINE:
			schedule_work(&wdd->wired_online_check_work);
			break;
		case WIRED_ITEM_CC_DETECT:
			schedule_work(&wdd->hw_detect_update_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void wdd_subscribe_wired_topic(struct oplus_mms *topic, void *prv_data)
{
	struct wired_disconnect_detection *wdd = prv_data;

	wdd->wired_topic = topic;
	wdd->wired_subs =
		oplus_mms_subscribe(wdd->wired_topic, wdd,
				    wdd_wired_subs_callback,
				    "wired_disconnect_detection");
	if (IS_ERR_OR_NULL(wdd->wired_subs)) {
		chg_err("subscribe wired topic error, rc=%ld\n",
			PTR_ERR(wdd->wired_subs));
		return;
	}
}

static void wdd_subscribe_cpa_topic(struct oplus_mms *topic, void *prv_data)
{
	struct wired_disconnect_detection *wdd = prv_data;

	wdd->cpa_topic = topic;
}

static struct state_keep_client_desc g_state_keep_client_desc = {
	.name = "wired_disconnect_detection",
	.priority = STATE_KEEP_CLIENT_WIRED_DISCONNECT_DETECTION,
	.ops = {
		.reset = wdd_client_reset,
		.enable = wdd_client_enable,
		.disable = wdd_client_disable,
		.start_check = wdd_client_start_check,
		.need_keep = wdd_client_need_keep,
		.switch_protocol = wdd_client_switch_protocol,
	}
};

static int wdd_client_register(struct wired_disconnect_detection *wdd)
{
	struct state_keep_client *client;

	client = state_keep_client_register(wdd->keep_topic, &g_state_keep_client_desc, wdd);
	if (IS_ERR_OR_NULL(client)) {
		chg_err("register wired_disconnect_detection client failed\n");
		return -EINVAL;
	}
	wdd->client = client;
	state_keep_client_enable(client);

	return 0;
}

static void wdd_subscribe_state_keep_topic(struct oplus_mms *topic, void *prv_data)
{
	struct wired_disconnect_detection *wdd = prv_data;

	wdd->keep_topic = topic;
	wdd_client_register(wdd);
}

static int wdd_parse_dt(struct wired_disconnect_detection *wdd)
{
	int i;
	int rc;

	rc = of_property_count_elems_of_size(wdd->node, "protocol_switch_table", sizeof(uint32_t));
	if (rc < 0) {
		chg_err("read protocol_switch_table failed, rc=%d\n", rc);
		return rc;
	}
	if (rc % WDD_PROTOCOL_SWITCH_INFO_ITEM_NUM) {
		chg_err("protocol_switch_table format error\n");
		return -EFAULT;
	}

	wdd->config.switch_table_num = rc / WDD_PROTOCOL_SWITCH_INFO_ITEM_NUM;
	wdd->config.switch_table =
		kzalloc(sizeof(struct wdd_protocol_switch_info) * wdd->config.switch_table_num, GFP_KERNEL);
	if (wdd->config.switch_table == NULL) {
		wdd->config.switch_table_num = 0;
		chg_err("alloc switch_table failed\n");
		return -ENOMEM;
	}

	wdd->config.support_protocol_mask = 0;
	for (i = 0; i < wdd->config.switch_table_num; i++) {
		rc = of_property_read_u32_index(wdd->node, "protocol_switch_table",
			i * WDD_PROTOCOL_SWITCH_INFO_ITEM_NUM, &wdd->config.switch_table[i].protocol);
		if (rc < 0)
			goto err;
		if (wdd_is_invalid_protocol(wdd->config.switch_table[i].protocol)) {
			chg_err("switch_table[%d].protocol is invalid, protocol = %d\n",
				i, wdd->config.switch_table[i].protocol);
			continue;
		}
		wdd->config.support_protocol_mask |= BIT(wdd->config.switch_table[i].protocol);
		rc = of_property_read_u32_index(wdd->node, "protocol_switch_table",
			i * WDD_PROTOCOL_SWITCH_INFO_ITEM_NUM + 1, &wdd->config.switch_table[i].power_thr_mw);
		if (rc < 0)
			goto err;
		/* convert from w to mw */
		wdd->config.switch_table[i].power_thr_mw *= 1000;
		rc = of_property_read_u32_index(wdd->node, "protocol_switch_table",
			i * WDD_PROTOCOL_SWITCH_INFO_ITEM_NUM + 2, &wdd->config.switch_table[i].high_power_count);
		if (rc < 0)
			goto err;
		rc = of_property_read_u32_index(wdd->node, "protocol_switch_table",
			i * WDD_PROTOCOL_SWITCH_INFO_ITEM_NUM + 3, &wdd->config.switch_table[i].low_power_count);
		if (rc < 0)
			goto err;
	}

	return 0;

err:
	chg_err("read oplus,protocol_list[%d] data failed, rc=%d\n", i, rc);
	wdd->config.switch_table_num = 0;
	kfree(wdd->config.switch_table);
	return rc;
}

int wired_disconnect_detection_init(struct device_node *node)
{
	struct wired_disconnect_detection *wdd;
	int rc;

	wdd = kzalloc(sizeof(struct wired_disconnect_detection), GFP_KERNEL);
	if (wdd == NULL) {
		chg_err("failed to alloc memory for wdd");
		return -ENOMEM;
	}
	g_wdd = wdd;
	wdd->node = node;

	rc = wdd_parse_dt(wdd);
	if (rc < 0)
		goto parse_dt_err;

	init_completion(&wdd->check_complete);
	wdd_state_reset(wdd);

	INIT_WORK(&wdd->wired_online_check_work, wdd_wired_online_check_work);
	INIT_WORK(&wdd->hw_detect_update_work, wdd_hw_detect_update_work);
	INIT_DELAYED_WORK(&wdd->hw_detect_check_work, wdd_hw_detect_check_work);
	INIT_DELAYED_WORK(&wdd->final_check_work, wdd_final_check_work);

	oplus_mms_wait_topic("state_keep", wdd_subscribe_state_keep_topic, wdd);
	oplus_mms_wait_topic("wired", wdd_subscribe_wired_topic, wdd);
	oplus_mms_wait_topic("cpa", wdd_subscribe_cpa_topic, wdd);

	return 0;

parse_dt_err:
	g_wdd = NULL;
	kfree(wdd);
	return rc;
}

void wired_disconnect_monitor_exit(void)
{
	if (!g_wdd)
		return;

	cancel_delayed_work_sync(&g_wdd->final_check_work);
	cancel_delayed_work_sync(&g_wdd->hw_detect_check_work);

	if (!IS_ERR_OR_NULL(g_wdd->keep_subs))
		oplus_mms_unsubscribe(g_wdd->keep_subs);
	if (!IS_ERR_OR_NULL(g_wdd->wired_subs))
		oplus_mms_unsubscribe(g_wdd->wired_subs);
	if (g_wdd->client)
		state_keep_client_unregister(g_wdd->client);
	if (g_wdd->config.switch_table)
		kfree(g_wdd->config.switch_table);

	g_wdd->config.switch_table_num = 0;
	kfree(g_wdd);
	g_wdd = NULL;
}
