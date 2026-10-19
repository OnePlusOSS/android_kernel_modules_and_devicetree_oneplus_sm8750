// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[FS]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/configfs.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/nls.h>
#include <linux/kdev_t.h>
#include <linux/random.h>

#include <oplus_chg.h>
#include <oplus_chg_voter.h>
#include <oplus_chg_module.h>
#include <oplus_chg_comm.h>
#include <oplus_chg_vooc.h>
#include <oplus_mms.h>
#include <oplus_mms_gauge.h>
#include <oplus_mms_wired.h>
#include <oplus_smart_chg.h>
#include <oplus_battery_log.h>
#include <oplus_chg_state_retention.h>
#include <oplus_parallel.h>
#include <oplus_chg_exception.h>
#include <oplus_chg_dual_chan.h>
#include <oplus_chg_cpa.h>
#include <oplus_chg_ufcs.h>
#include <oplus_chg_pps.h>
#include <oplus_chg_monitor.h>
#include <oplus_chg_wls.h>
#include <oplus_batt_bal.h>
#include <oplus_dischg_boost.h>
#include "monitor/oplus_chg_track.h"
#include <oplus_chg_plc.h>
#include <linux/completion.h>
#include <linux/mutex.h>
#include <oplus_sec.h>
#include <oplus_reverse_chg.h>
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/system/oplus_project.h>
#endif
#include <recovery/state_keep.h>
#include <oplus_chg_dual_cells_protection.h>

struct oplus_sec_ic_test_res {
	struct completion ack;
	struct mutex lock;
	bool busy;
	int cmd;
	int valid;
};

struct oplus_configfs_device {
	struct class *oplus_chg_class;
	struct device *oplus_usb_dir;
	struct device *oplus_battery_dir;
	struct device *oplus_wireless_dir;
	struct device *oplus_common_dir;
	struct mms_subscribe *gauge_subs;
	struct mms_subscribe *wired_subs;
	struct mms_subscribe *wls_subs;
	struct mms_subscribe *vooc_subs;
	struct mms_subscribe *comm_subs;
	struct mms_subscribe *retention_subs;
	struct oplus_mms *wired_topic;
	struct oplus_mms *gauge_topic;
	struct oplus_mms *wls_topic;
	struct oplus_mms *vooc_topic;
	struct oplus_mms *comm_topic;
	struct oplus_mms *parallel_topic;
	struct oplus_mms *ufcs_topic;
	struct oplus_mms *pps_topic;
	struct oplus_mms *err_topic;
	struct oplus_mms *cpa_topic;
	struct oplus_mms *reverse_topic;
	struct oplus_mms *batt_bal_topic;
	struct oplus_mms *protection_topic;
	struct oplus_mms *retention_topic;
	struct oplus_mms *plc_topic;
	struct oplus_mms *keep_topic;
	struct oplus_mms *dischg_boost_topic;
	struct mms_subscribe *ufcs_subs;
	struct mms_subscribe *pps_subs;
	struct mms_subscribe *plc_subs;
	struct mms_subscribe *reverse_subs;
	struct mms_subscribe *dischg_boost_subs;

	struct work_struct gauge_update_work;
	struct work_struct eis_reset_work;
	struct work_struct eis_status_update_work;
	struct delayed_work eis_timeout_work;
	struct delayed_work plc_enable_work;
	struct delayed_work clean_plc_enable_work;
	struct delayed_work plc_status_change_work;

	struct votable *wired_icl_votable;
	struct votable *wired_fcc_votable;
	struct votable *chg_disable_votable;
	struct votable *chg_suspend_votable;
	struct votable *cool_down_votable;
	struct votable *vooc_curr_votable;
	struct votable *pd_boost_disable_votable;
	struct votable *pd_svooc_votable;
	struct votable *ufcs_curr_votable;
	struct votable *pps_curr_votable;
	struct votable *wls_fcc_curr_votable;
	struct votable *wired_disable_votable;
	struct votable *plc_force_buck_votable;

	struct oplus_sec_ic_test_res sec_ic_test_res;

	bool batt_exist;
	int vbat_mv;
	int batt_temp;
	int ibat_ma;
	int soc;
	int batt_fcc;
	int batt_cc;
	int batt_rm;
	int batt_soh;
	int batt_fcc_coeff;
	int batt_soh_coeff;
	int gauge_vbat;
	int gauge_car_c;
	int debug_batt_cc;
	bool deep_support;
	bool super_endurance_mode_status;
	int super_endurance_mode_count;
	bool subboard_temp_err;

	bool wired_online;
	int wired_type;
	int power_role;

	bool wls_online;
	int wls_type;

	bool vooc_started;
	bool vooc_online;
	bool vooc_charging;
	bool vooc_online_keep;
	unsigned int vooc_sid;

	bool ship_mode;
	bool slow_chg_enable;
	bool retention_state;
	int slow_chg_pct;
	int slow_chg_watt;
	unsigned int notify_code;

	int bcc_current;
	bool mos_test_result;

	bool ufcs_online;
	bool ufcs_charging;
	bool ufcs_oplus_adapter;
	u32 ufcs_adapter_id;

	bool pps_online;
	bool pps_online_keep;
	bool pps_charging;
	bool pps_oplus_adapter;

	int vbat_uv_thr;
	int real_cool_down;
	int reverse_chg_type;
	unsigned int nvid_support_flags;
	int eis_status;
	int eis_current;
	int plc_status;
	bool plc_user_enable;
	bool batt_health;
	int boost_ic_type;
	int boost_dev_id;
	int boost_cv;
};

static struct oplus_configfs_device *g_cfg_dev;

__maybe_unused static bool
is_wired_disable_votable_available(struct oplus_configfs_device *chip)
{
	if (!chip->wired_disable_votable)
		chip->wired_disable_votable = find_votable("WIRED_CHARGING_DISABLE");
	return !!chip->wired_disable_votable;
}

__maybe_unused static bool
is_wired_fcc_votable_available(struct oplus_configfs_device *chip)
{
	if (!chip->wired_fcc_votable)
		chip->wired_fcc_votable = find_votable("WIRED_FCC");
	return !!chip->wired_fcc_votable;
}

__maybe_unused static bool
is_chg_disable_votable_available(struct oplus_configfs_device *chip)
{
	if (!chip->chg_disable_votable)
		chip->chg_disable_votable = find_votable("CHG_DISABLE");
	return !!chip->chg_disable_votable;
}

__maybe_unused static bool
is_chg_suspend_votable_available(struct oplus_configfs_device *chip)
{
	if (!chip->chg_suspend_votable)
		chip->chg_suspend_votable = find_votable("CHG_SUSPEND");
	return !!chip->chg_suspend_votable;
}

__maybe_unused static bool
is_cool_down_votable_available(struct oplus_configfs_device *chip)
{
	if (!chip->cool_down_votable)
		chip->cool_down_votable = find_votable("COOL_DOWN");
	return !!chip->cool_down_votable;
}

__maybe_unused static bool
is_vooc_curr_votable_available(struct oplus_configfs_device *chip)
{
	if (!chip->vooc_curr_votable)
		chip->vooc_curr_votable = find_votable("VOOC_CURR");
	return !!chip->vooc_curr_votable;
}

__maybe_unused static bool
is_ufcs_curr_votable_available(struct oplus_configfs_device *chip)
{
	if (!chip->ufcs_curr_votable)
		chip->ufcs_curr_votable = find_votable("UFCS_CURR");
	return !!chip->ufcs_curr_votable;
}

__maybe_unused static bool
is_pps_curr_votable_available(struct oplus_configfs_device *chip)
{
	if (!chip->pps_curr_votable)
		chip->pps_curr_votable = find_votable("PPS_CURR");
	return !!chip->pps_curr_votable;
}

__maybe_unused static bool
is_wls_curr_votable_available(struct oplus_configfs_device *chip)
{
	if (!chip->wls_fcc_curr_votable)
		chip->wls_fcc_curr_votable = find_votable("WLS_FCC");
	return !!chip->wls_fcc_curr_votable;
}

static bool is_cpa_topic_available(struct oplus_configfs_device *chip)
{
	if (!chip->cpa_topic)
		chip->cpa_topic = oplus_mms_get_by_name("cpa");

	return !!chip->cpa_topic;
}

static bool is_parallel_topic_available(struct oplus_configfs_device *chip)
{
	if (!chip->parallel_topic)
		chip->parallel_topic = oplus_mms_get_by_name("parallel");

	return !!chip->parallel_topic;
}

__maybe_unused static bool is_err_topic_available(struct oplus_configfs_device *chip)
{
	if (!chip->err_topic)
		chip->err_topic = oplus_mms_get_by_name("error");
	return !!chip->err_topic;
}

static bool is_batt_bal_topic_available(struct oplus_configfs_device *chip)
{
	if (!chip->batt_bal_topic)
		chip->batt_bal_topic = oplus_mms_get_by_name("batt_bal");

	return !!chip->batt_bal_topic;
}

static bool is_dischg_boost_topic_available(struct oplus_configfs_device *chip)
{
	if (!chip->dischg_boost_topic)
		chip->dischg_boost_topic = oplus_mms_get_by_name("dischg_boost");

	return !!chip->dischg_boost_topic;
}

static bool is_plc_force_buck_votable_available(struct oplus_configfs_device *chip)
{
	if (!chip->plc_force_buck_votable)
		chip->plc_force_buck_votable = find_votable("PLC_FORCE_BUCK");
	return !!chip->plc_force_buck_votable;
}

__maybe_unused static bool is_comm_topic_available(struct oplus_configfs_device *chip)
{
	if (!chip->comm_topic)
		chip->comm_topic = oplus_mms_get_by_name("common");
	return !!chip->comm_topic;
}

static int oplus_configfs_push_mmi_chg_info_msg(struct oplus_configfs_device *chip, int mmi_chg)
{
	struct mms_msg *msg;
	int rc;

	if (!is_err_topic_available(chip)) {
		chg_err("error topic not found\n");
		return -ENODEV;
	}

	msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_MMI_CHG, "$$mmi_chg@@%d", mmi_chg);
	if (msg == NULL) {
		chg_err("alloc mmi chg error msg error\n");
		return -ENOMEM;
	}

	rc = oplus_mms_publish_msg(chip->err_topic, msg);
	if (rc < 0) {
		chg_err("publish mmi chg error msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static int oplus_configfs_push_slow_chg_info_msg(struct oplus_configfs_device *chip, int pct, int watt, int en)
{
	struct mms_msg *msg;
	int rc;

	if (!is_err_topic_available(chip)) {
		chg_err("error topic not found\n");
		return -ENODEV;
	}

	msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_SLOW_CHG,
				      "$$slow_chg_pct@@%d$$slow_chg_watt@@%d$$slow_chg_en@@%d", pct, watt, en);
	if (msg == NULL) {
		chg_err("alloc slow chg error msg error\n");
		return -ENOMEM;
	}

	rc = oplus_mms_publish_msg(chip->err_topic, msg);
	if (rc < 0) {
		chg_err("publish slow chg error msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static int oplus_configfs_push_eis_status_msg(struct oplus_configfs_device *chip, int status)
{
	struct mms_msg *msg;
	int rc;

	if (!is_comm_topic_available(chip)) {
		chg_err("common topic not found\n");
		return -ENODEV;
	}

	msg = oplus_mms_alloc_int_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, COMM_ITEM_EIS_STATUS, status);
	if (msg == NULL) {
		chg_err("<EIS>alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("<EIS>publish eis msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static int oplus_configfs_push_eis_timeout_info_msg(struct oplus_configfs_device *chip,
	const char *err_txt)
{
	struct mms_msg *msg;
	int rc;

	if (!is_err_topic_available(chip)) {
		chg_err("error topic not found\n");
		return -ENODEV;
	}

	msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_EIS_TIMEOUT,
		err_txt);
	if (msg == NULL) {
		chg_err("alloc eis timeout error msg error\n");
		return -ENOMEM;
	}

	rc = oplus_mms_publish_msg(chip->err_topic, msg);
	if (rc < 0) {
		chg_err("publish eis timeout error msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static void oplus_configfs_reset_eis_status(struct oplus_configfs_device *chip)
{
	chg_info("<EIS> reset eis status\n");
	if (is_wired_fcc_votable_available(chip))
		vote(chip->wired_fcc_votable, EIS_VOTER, false, 0, false);

	if (is_vooc_curr_votable_available(chip))
		vote(chip->vooc_curr_votable, EIS_VOTER, false, 0, false);

	if (is_ufcs_curr_votable_available(chip))
		vote(chip->ufcs_curr_votable, EIS_VOTER, false, 0, false);

	if (is_chg_disable_votable_available(chip))
		vote(chip->chg_disable_votable, EIS_VOTER, false, 0, false);

	oplus_configfs_push_eis_status_msg(chip, EIS_STATUS_DISABLE);
}

static int fast_chg_type_by_user = -1;
static ssize_t fast_chg_type_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int fast_chg_type = 0;
	bool pd_use_default = false;
	union mms_msg_data data = { 0 };
	int rc;

	if (!chip->pd_boost_disable_votable)
		chip->pd_boost_disable_votable = find_votable("PD_BOOST_DISABLE");

	if (!chip->pd_svooc_votable)
		chip->pd_svooc_votable = find_votable("PD_SVOOC");

	if (chip->vooc_sid == 0) {
		switch (chip->wired_type) {
		case OPLUS_CHG_USB_TYPE_QC2:
		case OPLUS_CHG_USB_TYPE_QC3:
			fast_chg_type = CHARGER_SUBTYPE_QC;
			break;
		case OPLUS_CHG_USB_TYPE_UFCS:
			fast_chg_type = CHARGER_SUBTYPE_UFCS;
			break;
		case OPLUS_CHG_USB_TYPE_PD_PPS:
			if (chip->pps_online || chip->pps_online_keep) {
				fast_chg_type = CHARGER_SUBTYPE_PPS;
				break;
			} else if (is_cpa_topic_available(chip)) {
				oplus_mms_get_item_data(chip->cpa_topic, CPA_ITEM_ALLOW, &data, false);
				if (data.intval == CHG_PROTOCOL_PPS) {
					/* switching pps */
					pd_use_default = true;
					break;
				}
			}
			fallthrough;
		case OPLUS_CHG_USB_TYPE_PD:
		case OPLUS_CHG_USB_TYPE_PD_DRP:
			if (chip->pps_online_keep) {
				fast_chg_type = CHARGER_SUBTYPE_PPS;
			} else {
				if (chip->pd_boost_disable_votable &&
				    is_client_vote_enabled(chip->pd_boost_disable_votable, SVID_VOTER))
					pd_use_default = true;
				if (chip->pd_svooc_votable &&
				    !!get_effective_result(chip->pd_svooc_votable))
					pd_use_default = true;

				if (pd_use_default)
					fast_chg_type = CHARGER_SUBTYPE_DEFAULT;
				else
					fast_chg_type = CHARGER_SUBTYPE_PD;
			}
			break;
		default:
			break;
		}
	} else {
		if (sid_to_adapter_id(chip->vooc_sid) == 0) {
			if (sid_to_adapter_chg_type(chip->vooc_sid) ==
			    CHARGER_TYPE_VOOC)
				fast_chg_type = CHARGER_SUBTYPE_FASTCHG_VOOC;
			else if (sid_to_adapter_chg_type(chip->vooc_sid) ==
				 CHARGER_TYPE_SVOOC)
				fast_chg_type = CHARGER_SUBTYPE_FASTCHG_SVOOC;
		} else {
			if (sid_to_adapter_chg_type(chip->vooc_sid) == CHARGER_TYPE_VOOC)
				fast_chg_type = CHARGER_SUBTYPE_FASTCHG_VOOC;
			else
				fast_chg_type = sid_to_adapter_id(chip->vooc_sid);
		}
	}

	if (chip->wls_online) {
		rc = oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_WLS_TYPE, &data, true);
		if (rc < 0)
			fast_chg_type = CHARGER_SUBTYPE_DEFAULT;
		else if (data.intval == OPLUS_CHG_WLS_VOOC)
			fast_chg_type = CHARGER_SUBTYPE_FASTCHG_VOOC;
		else if (data.intval == OPLUS_CHG_WLS_SVOOC || data.intval == OPLUS_CHG_WLS_PD_65W)
			fast_chg_type = WLS_ADAPTER_TYPE_SVOOC;
		else
			fast_chg_type = CHARGER_SUBTYPE_DEFAULT;
	}

	if (fast_chg_type_by_user > 0)
		fast_chg_type = fast_chg_type_by_user;

	return sprintf(buf, "%d\n", fast_chg_type);
}

static ssize_t fast_chg_type_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	fast_chg_type_by_user = val;

	return count;
}
static DEVICE_ATTR_RW(fast_chg_type);

static ssize_t reverse_chg_type_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	chg_info("reverse_chg_type = %d\n", chip->reverse_chg_type);
	return sprintf(buf, "%d\n", chip->reverse_chg_type);
}

static ssize_t reverse_chg_type_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int val = 0;
	int rc;

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	rc = oplus_set_reverse_chg_type(chip->reverse_topic, val);
	if (rc < 0)
		chg_err("set reverse_chg_type %d error\n", val);

	return count;
}
static DEVICE_ATTR_RW(reverse_chg_type);

static ssize_t power_role_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	return sprintf(buf, "%d\n", chip->power_role);
}
static DEVICE_ATTR_RO(power_role);

#define VDM_INFO_MAX	5
static ssize_t vdm_info_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	u32 vdm_info[VDM_INFO_MAX] = {0};
	int vdm_cnt = VDM_INFO_MAX;
	ssize_t count = 0;
	int rc;
	int i;

	rc = oplus_wired_get_vdm_info(chip->wired_topic, vdm_info, &vdm_cnt);
	chg_info("vdm_info: rc=%d, vdm_cnt=%d\n", rc, vdm_cnt);
	/* vdm_info[0] is protocol header, skip it */
	if (rc < 0 || vdm_cnt <= 1)
		return scnprintf(buf, PAGE_SIZE, "\n");
	for (i = 1; i < vdm_cnt; i++) {
		count += scnprintf(buf + count, PAGE_SIZE, "0x%08x ", vdm_info[i]);
	}
	count += scnprintf(buf + count, PAGE_SIZE, "\n");
	chg_info("vdm_info: %s\n", buf);
	return count;
}
static DEVICE_ATTR_RO(vdm_info);

static ssize_t otg_online_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int otg_online = 0;

	otg_online = oplus_wired_get_otg_online_status(chip->wired_topic);

	return sprintf(buf, "%d\n", otg_online);
}
static DEVICE_ATTR_RO(otg_online);

static ssize_t otg_switch_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	/* struct oplus_configfs_device *chip = dev->driver_data; */

	return sprintf(buf, "%d\n", oplus_wired_get_otg_switch_status());
}

static ssize_t otg_switch_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	int val = 0;
	/* struct oplus_configfs_device *chip = dev->driver_data; */
	int rc;

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	rc = oplus_wired_set_otg_switch_status(val);
	if (rc < 0) {
		chg_err("can't %s otg switch\n", !!val ? "open" : "close");
		return rc;
	}

	return count;
}
static DEVICE_ATTR_RW(otg_switch);

static ssize_t usb_status_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	union mms_msg_data data = { 0 };
	union mms_msg_data wls_data = { 0 };

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_USB_STATUS, &data,
				true);
	if (chip->wls_topic) {
		oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_NOTIFY, &wls_data, true);
		data.intval = data.intval | wls_data.intval;
	}

	return sprintf(buf, "%d\n", data.intval);
}
static DEVICE_ATTR_RO(usb_status);

static ssize_t usbtemp_volt_l_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	union mms_msg_data data = { 0 };

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_USB_TEMP_VOLT_L,
				&data, true);

	return sprintf(buf, "%d\n", data.intval);
}
static DEVICE_ATTR_RO(usbtemp_volt_l);

static ssize_t usbtemp_volt_r_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	union mms_msg_data data = { 0 };

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_USB_TEMP_VOLT_R,
				&data, true);

	return sprintf(buf, "%d\n", data.intval);
}
static DEVICE_ATTR_RO(usbtemp_volt_r);

static ssize_t typec_cc_orientation_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	return sprintf(buf, "%d\n", oplus_wired_get_cc_orientation());
}
static DEVICE_ATTR_RO(typec_cc_orientation);

static struct device_attribute *oplus_usb_attributes[] = {
	&dev_attr_otg_online,
	&dev_attr_otg_switch,
	&dev_attr_usb_status,
	&dev_attr_typec_cc_orientation,
	&dev_attr_fast_chg_type,
	&dev_attr_usbtemp_volt_l,
	&dev_attr_usbtemp_volt_r,
	&dev_attr_reverse_chg_type,
	&dev_attr_power_role,
	&dev_attr_vdm_info,
	NULL
};

/**********************************************************************
* battery device nodes
**********************************************************************/
static ssize_t authenticate_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	if (is_comm_topic_available(chip) && oplus_comm_get_hmac_not_pop_up(chip->comm_topic))
		return sprintf(buf, "%d\n", true);
	else
		return sprintf(buf, "%d\n", oplus_gauge_get_batt_authenticate());
}
static DEVICE_ATTR_RO(authenticate);

static ssize_t battery_cc_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	return sprintf(buf, "%d\n", chip->batt_cc);
}
static DEVICE_ATTR_RO(battery_cc);

static ssize_t battery_fcc_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int fcc = chip->batt_fcc;

	if (chip->batt_soh > 0 && chip->batt_soh <= 100 && chip->batt_fcc_coeff)
		fcc = min(chip->batt_fcc + chip->batt_fcc_coeff * chip->batt_soh / 100,
			  oplus_gauge_get_batt_capacity_mah(chip->gauge_topic));

	return sprintf(buf, "%d\n", fcc);
}
static DEVICE_ATTR_RO(battery_fcc);

#define BATT_RM_LEN 10
static ssize_t battery_rm_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int batt_rm;

	batt_rm = chip->batt_rm < 0 ? 0 : chip->batt_rm;
	return scnprintf(buf, BATT_RM_LEN, "%d\n", batt_rm);
}
static DEVICE_ATTR_RO(battery_rm);

static ssize_t battery_soh_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int soh = chip->batt_soh;

	if (chip->batt_soh > 0 && chip->batt_soh <= 100 && chip->batt_soh_coeff)
		soh = min(chip->batt_soh + chip->batt_soh_coeff * chip->batt_soh / 100, 100);

	return sprintf(buf, "%d\n", soh);
}
static DEVICE_ATTR_RO(battery_soh);

static ssize_t design_capacity_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int val;

	val = oplus_gauge_get_batt_capacity_mah(chip->gauge_topic);

	return sprintf(buf, "%d\n", val);
}
static DEVICE_ATTR_RO(design_capacity);

static ssize_t smartchg_soh_support_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int val;

	val = oplus_smart_chg_get_soh_support();
	if (val < 0)
		return val;

	return sprintf(buf, "%d\n", val);
}
static DEVICE_ATTR_RO(smartchg_soh_support);

static ssize_t battery_sn_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	int len = 0;
	int ret = 0;
	char batt_sn[OPLUS_BATT_SERIAL_NUM_SIZE * 2] = {"\0"};
	struct oplus_configfs_device *chip = dev->driver_data;

	ret = oplus_gauge_get_battinfo_sn(chip->gauge_topic, batt_sn, sizeof(batt_sn));
	if (ret < 0)
		chg_err("get battery sn error");
	else
		len = sprintf(buf, "%s\n", batt_sn);

	return len;
}
static DEVICE_ATTR_RO(battery_sn);

static ssize_t battery_chem_id_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	int len = 0;
	int ret = 0;
	char batt_chem_id[CHEMID_MAX_LENGTH] = {"\0"};
	struct oplus_configfs_device *chip = dev->driver_data;

	ret = oplus_gauge_show_batt_chem_id(chip->gauge_topic, batt_chem_id, sizeof(batt_chem_id));
	if (ret <= 0)
		chg_debug("get battery chem id error");
	else
		len = sprintf(buf, "%s\n,", batt_chem_id);

	return len;
}
static DEVICE_ATTR_RO(battery_chem_id);

static ssize_t battery_manu_date_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	int len = 0;
	int ret = 0;
	char batt_date[OPLUS_BATTINFO_DATE_SIZE * 2] = {"\0"};
	struct oplus_configfs_device *chip = dev->driver_data;

	ret = oplus_gauge_get_battinfo_manu_date(chip->gauge_topic, batt_date, sizeof(batt_date));
	if (ret < 0)
		chg_err("get battery manu date error");
	else
		len = sprintf(buf, "%s\n", batt_date);

	return len;
}
static DEVICE_ATTR_RO(battery_manu_date);

#define BATT_SN_SIZE 25
#define SOH_BUFFER_SIZE 128
static ssize_t reset_gauge_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	int buff[3] = {0};
	struct oplus_configfs_device *chip = dev->driver_data;

	if (buf == NULL) {
		chg_err("NULL pointer detected\n");
		return 0;
	}

	sscanf(buf, "%d,%d,%d", &buff[0], &buff[1], &buff[2]);
	chg_info("reset_gauge_store = %s, %d %d %d\n", buf, buff[0], buff[1], buff[2]);

	rc = oplus_set_reset_gauge_parms(chip->gauge_topic, buff);
	if (rc < 0)
		return rc;

	return count;
}
DEVICE_ATTR_WO(reset_gauge);

#define HISTSOH_DATA_SIZE 16
static ssize_t historic_soh_date_show(struct device *dev, struct device_attribute *attr,
			      char *output_str)
{
	int ret = 0;
	int read_pagedata[HISTSOH_DATA_SIZE] = {0};
	struct oplus_configfs_device *chip = dev->driver_data;
	int base_index = 0;
	int read_checksum;   /* Checksum (1 byte) */
	int num;             /* Number of members (1 byte) */
	int set_menber[HISTSOH_DATA_SIZE] = {0};
	int i;
	int len;

	/* Parameter validity check */
	if (output_str == NULL) {
		chg_err("NULL pointer detected\n");
		return 0;
	}

	/* read page data for digital signature */
	ret = oplus_gauge_get_historic_soh_date(chip->gauge_topic, read_pagedata, sizeof(read_pagedata));
	if (ret < 0) {
		chg_err("get battery get_historic_soh error");
		return ret;
	}

	/* Parse the basic fields */
	read_checksum = read_pagedata[0];   /* Checksum (1 byte) */
	num = read_pagedata[1];             /* Number of members (1 byte) */

	/* Check the validity of the number of members */
	if (num > HISTSOH_DATA_SIZE || num < 0) { /* Validate data size constraints */
		chg_err("Invalid num: %d\n", num);
		return 0;
	}

	/* Parse initial two elements */
	set_menber[0] = read_pagedata[2]; /* Direct raw value assignment */
	set_menber[1] = read_pagedata[3];

	/* Process subsequent elements (16-bit value composition) */
	for (i = 2; i < num && i < HISTSOH_DATA_SIZE; i++) {
		base_index = 4 + (i - 2) * 2; /* Calculate data offset */
		if (base_index + 1 >= HISTSOH_DATA_SIZE - 1) { /* Boundary check */
			chg_err("Data overflow at index %d\n", i);
			return 0;
		}
		/* Combine high/low bytes */
		set_menber[i] = (read_pagedata[base_index] << 8) | read_pagedata[base_index + 1];
	}

	/* Construct output string */
	len = snprintf(output_str, SOH_BUFFER_SIZE, "%X,%d", read_checksum, num); /* Hex checksum header */
	for (i = 0; i < num; i++) {
		if (len >= SOH_BUFFER_SIZE) { /* Buffer capacity check */
			chg_err("Output buffer overflow\n");
			return 0;
		}
		len += snprintf(output_str + len, SOH_BUFFER_SIZE - len, ",%d", set_menber[i]); /* Append numeric data value */
	}
	chg_info("historic_soh_date_show is %s\n", output_str);

	return len;
}

static ssize_t historic_soh_date_store(struct device *dev,
			       struct device_attribute *attr, const char *set_soh_data,
			       size_t count)
{
	int pagedata1[HISTSOH_DATA_SIZE] = {0};
	int read_checksum;
	int set_menber[HISTSOH_DATA_SIZE] = {0};
	int num;
	int i;
	char original_data[SOH_BUFFER_SIZE] = {0};
	int ret = 0;
	struct oplus_configfs_device *chip = dev->driver_data;
	int *members;
	char *values;

	if (set_soh_data == NULL) {
		chg_err("set_soh_data == NULL failed\n");
		return 0;
	}
	chg_err("write digital signature %s\n", set_soh_data);

	if (sizeof(set_soh_data) >= SOH_BUFFER_SIZE) {
		chg_err("sizeof(set_soh_data) > 128 failed\n");
		return 0;
	}
	strncpy((char *)original_data, set_soh_data, SOH_BUFFER_SIZE - 1);
	original_data[SOH_BUFFER_SIZE - 1] = '\0';

	chg_err("write digital signature %s\n", original_data);

	/* read page data for digital signature */
	values = &original_data[0];
	sscanf(values, "%X,%d", &read_checksum, &num);

	/* Check the validity of the number of members */
	if (num > HISTSOH_DATA_SIZE || num < 0) {         /* Prevent array from going out of bounds */
		chg_err("Invalid num: %d\n", num);
		return 0;
	}

	chg_err("sscanf pack read_checksum: %d, num: %d\n", read_checksum, num);
	values = strchr(values, ',');
	if (values != NULL) {
		values++;
		values = strchr(values, ',');
		if (values != NULL)
			values++;
		else
			return 0;
	} else {
		return 0;
	}

	members = &set_menber[0];
	for (i = 0; i < num; i++) {
		if (sscanf(values, "%d", members + i) == 1) {
			values = strchr(values, ',');
			if (values != NULL)
				values++;
			else
				break;
		} else {
			break;
		}
	}
	pagedata1[0] = read_checksum;
	pagedata1[1] = num;
	pagedata1[2] = set_menber[0];
	pagedata1[3] = set_menber[1];

	chg_info("historic_soh_date_store:%d,%d,", pagedata1[0], pagedata1[1]);
	for (i = 2; i < num && i < HISTSOH_DATA_SIZE; i++) {
		int base_index = 4 + (i - 2) * 2; /* Calculate the starting index */
		if (base_index + 1 >= HISTSOH_DATA_SIZE - 1) {        /* Prevent out-of-bounds access */
			chg_err("Data overflow at index %d\n", i);
			return 0;
		}
		pagedata1[base_index] = (set_menber[i] & 0xFF00) >> 8; /* High 8 bit */
		pagedata1[base_index + 1] = set_menber[i] & 0xFF;          /* Low 8 bit */
		chg_info("%d,%d,", pagedata1[base_index], pagedata1[base_index + 1]);
	}

	ret = oplus_set_histrioc_soh_date(chip->gauge_topic, pagedata1);
	if (ret < 0)
		chg_err("set battery first usage date error");

	return count;
}
static DEVICE_ATTR_RW(historic_soh_date);

#define UI_SOH_SN_DATA_TEST_TAG "test_sn_is:"
#define UI_SOH_SN_DATA_TEST_NULL_TAG "test_sn_NULL"
static char batt_sn_buf[BATT_SN_SIZE + 1] = {"\0"};
static ssize_t batt_ic_sn_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	int len = 0;
	int ret = 0;
	char batt_date[BATT_SN_SIZE + 1] = {"\0"};
	struct oplus_configfs_device *chip = dev->driver_data;

	if (buf == NULL) {
		chg_err("batt_ic_sn_show buf NULL failed\n");
		return 0;
	}
	ret = oplus_gauge_get_batt_sn(chip->gauge_topic, batt_date, sizeof(batt_date));
	if (strstr(batt_sn_buf, UI_SOH_SN_DATA_TEST_TAG))
		memcpy(batt_date, batt_sn_buf, strlen(batt_sn_buf));
	else if (strstr(batt_sn_buf, UI_SOH_SN_DATA_TEST_NULL_TAG))
		memset(batt_date, 0, sizeof(batt_date));

	batt_date[BATT_SN_SIZE] = '\0';
	if (ret < 0)
		chg_err("get battery batt_ic_sn_show date error");
	else
		len = sprintf(buf, "%s", batt_date);

	chg_info("batt_ic_sn_show:%s\n", buf);
	return len;
}

static ssize_t batt_ic_sn_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	if (strlen(buf) > BATT_SN_SIZE) {
		chg_err("batt_ic_sn_store:strlen > BATT_SN_SIZE\n");
		return 0;
	}
	memcpy(batt_sn_buf, buf, strlen(buf));
	chg_info("batt_ic_sn_store:%s\n", batt_sn_buf);

	return count;
}
static DEVICE_ATTR_RW(batt_ic_sn);

static ssize_t battery_first_usage_date_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	int len = 0;
	int ret = 0;
	char batt_date[OPLUS_BATTINFO_DATE_SIZE] = {"\0"};
	struct oplus_configfs_device *chip = dev->driver_data;

	ret = oplus_gauge_get_battinfo_first_usage_date(chip->gauge_topic, batt_date, sizeof(batt_date));
	if (ret < 0)
		chg_err("get battery first usage date error");
	else
		len = sprintf(buf, "%s\n", batt_date);

	return len;
}

static ssize_t  battery_first_usage_date_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int ret = 0;

	struct oplus_configfs_device *chip = dev->driver_data;

	ret = oplus_gauge_set_battinfo_first_usage_date(chip->gauge_topic, buf);
	if (ret < 0)
		chg_err("set battery first usage date error");

	return count;
}
static DEVICE_ATTR_RW(battery_first_usage_date);

static ssize_t battery_ui_cc_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	int ui_cycle_count = 0;
	int len = 0;
	struct oplus_configfs_device *chip = dev->driver_data;

	ui_cycle_count = oplus_gauge_get_ui_cc(chip->gauge_topic);
	if (ui_cycle_count < 0)
		chg_err("get battery ui cycle count error");
	else
		len = sprintf(buf, "%d\n", ui_cycle_count);

	return len;
}

static ssize_t  battery_ui_cc_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int ret = 0;
	int ui_cycle_count;
	struct oplus_configfs_device *chip = dev->driver_data;

	if (kstrtos32(buf, 0, &ui_cycle_count)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	ret = oplus_gauge_set_ui_cc(chip->gauge_topic, ui_cycle_count);
	if (ret < 0)
		chg_err("set battery ui cycle count error");

	return count;
}
static DEVICE_ATTR_RW(battery_ui_cc);

static ssize_t battery_ui_soh_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	int ui_soh = 0;
	int len = 0;
	struct oplus_configfs_device *chip = dev->driver_data;

	ui_soh = oplus_gauge_get_ui_soh(chip->gauge_topic);
	if (ui_soh < 0)
		chg_err("get battery ui soh error");
	else
		len = sprintf(buf, "%d\n", ui_soh);

	return len;
}

static ssize_t  battery_ui_soh_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int ret = 0;
	int ui_soh;
	struct oplus_configfs_device *chip = dev->driver_data;

	if (kstrtos32(buf, 0, &ui_soh)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	ret = oplus_gauge_set_ui_soh(chip->gauge_topic, ui_soh);
	if (ret < 0)
		chg_err("set battery ui soh error");

	return count;
}
static DEVICE_ATTR_RW(battery_ui_soh);

static ssize_t battery_used_flag_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	int used_flag = 0;
	int len = 0;
	struct oplus_configfs_device *chip = dev->driver_data;

	used_flag = oplus_gauge_get_used_flag(chip->gauge_topic);
	if (used_flag < 0)
		chg_err("get battery used flag error");
	else
		len = sprintf(buf, "%d\n", used_flag);

	return len;
}

static ssize_t  battery_used_flag_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int ret = 0;
	int used_flag;
	struct oplus_configfs_device *chip = dev->driver_data;

	if (kstrtos32(buf, 0, &used_flag)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	ret = oplus_gauge_set_used_flag(chip->gauge_topic, used_flag);
	if (ret < 0)
		chg_err("set battery used flag error");

	return count;
}
static DEVICE_ATTR_RW(battery_used_flag);

#ifdef CONFIG_OPLUS_CALL_MODE_SUPPORT
static ssize_t call_mode_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	/* struct oplus_configfs_device *chip = dev->driver_data; */

	/* return sprintf(buf, "%d\n", chip->calling_on); */
	return 0;
}

static ssize_t call_mode_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int val = 0;
	/* struct oplus_configfs_device *chip = dev->driver_data; */

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	return count;
}
static DEVICE_ATTR_RW(call_mode);
#endif /*CONFIG_OPLUS_CALL_MODE_SUPPORT*/

static ssize_t charge_technology_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	return sprintf(buf, "%u\n", oplus_vooc_get_project(chip->vooc_topic));
}

static ssize_t charge_technology_store(struct device *dev,
				       struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;
	struct oplus_configfs_device *chip = dev->driver_data;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	oplus_vooc_set_project(chip->vooc_topic, (uint32_t)val);
	chg_info("val = %d, vooc_project = %d\n", val, oplus_vooc_get_project(chip->vooc_topic));

	return count;
}
static DEVICE_ATTR_RW(charge_technology);

#ifdef CONFIG_OPLUS_CHIP_SOC_NODE
static ssize_t chip_soc_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	return sprintf(buf, "%d\n", chip->soc);
}
static DEVICE_ATTR_RO(chip_soc);
#endif /*CONFIG_OPLUS_CHIP_SOC_NODE*/

static ssize_t gauge_vbat_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	return sprintf(buf, "%d\n", chip->gauge_vbat);
}
static DEVICE_ATTR_RO(gauge_vbat);

#ifdef CONFIG_OPLUS_SMART_CHARGER_SUPPORT
static ssize_t cool_down_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	union mms_msg_data data = { 0 };
	int rc;

	if (chip->comm_topic)
		rc = oplus_mms_get_item_data(chip->comm_topic,
					     COMM_ITEM_COOL_DOWN, &data, false);
	else
		rc = -ENOTSUPP;
	if (rc < 0) {
		chg_err("can't get cool_down, rc=%d\n", rc);
		return rc;
	}

	return sprintf(buf, "%d\n", data.intval);
}

static ssize_t cool_down_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int val = 0;
	struct oplus_configfs_device *chip = dev->driver_data;
	int rc;

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	if (val == SALE_MODE_COOL_DOWN || val == SALE_MODE_COOL_DOWN_TWO) {
		chip->real_cool_down = val;
		val = SALE_MODE_COOL_DOWN_VAL;
	} else {
		chip->real_cool_down = 0;
	}
	oplus_comm_set_sale_mode(chip->comm_topic, chip->real_cool_down);
	if (is_cool_down_votable_available(chip))
		rc = vote(chip->cool_down_votable, USER_VOTER,
			  val > 0 ? true : false, val, false);
	else
		rc = -ENOTSUPP;
	if (rc < 0) {
		chg_err("can't set cool_down to %d, rc=%d\n", val, rc);
		return rc;
	}

	return count;
}
static DEVICE_ATTR_RW(cool_down);
#endif /*CONFIG_OPLUS_SMART_CHARGER_SUPPORT*/

static ssize_t fast_charge_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	union mms_msg_data data = { 0 };
	bool fastchg;
	int rc;

	fastchg = chip->vooc_online || chip->vooc_online_keep;

	fastchg |= chip->ufcs_online;

	fastchg |= chip->pps_online || chip->pps_online_keep;

	if (chip->wls_online) {
		rc = oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_WLS_TYPE, &data, true);
		if (rc < 0)
			chg_err("can't get wls type, rc=%d\n", rc);
		else if (data.intval == OPLUS_CHG_WLS_SVOOC || data.intval == OPLUS_CHG_WLS_PD_65W)
			fastchg = true;

		rc = oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_ICON_TYPE, &data, true);
		if (rc < 0)
			chg_err("can't get wls type, rc=%d\n", rc);
		else if (data.intval == OPLUS_CHG_WLS_SVOOC || data.intval == OPLUS_CHG_WLS_PD_65W)
			fastchg = true;
	}

	return sprintf(buf, "%d\n", fastchg);
}
static DEVICE_ATTR_RO(fast_charge);

static ssize_t mmi_charging_enable_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int val;

	if (is_chg_disable_votable_available(chip))
		val = !get_client_vote(chip->chg_disable_votable, MMI_CHG_VOTER);
	else
		return -ENOTSUPP;

	return sprintf(buf, "%d\n", val);
}

static ssize_t mmi_charging_enable_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int val = 0;
	struct oplus_configfs_device *chip = dev->driver_data;
	int rc;

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	oplus_configfs_push_mmi_chg_info_msg(chip, !!val);
	if (is_chg_disable_votable_available(chip))
		rc = vote(chip->chg_disable_votable, MMI_CHG_VOTER,
			  !val ? true : false, !val, false);
	else
		rc = -ENOTSUPP;
	if (rc < 0)
		chg_err("can't set charging %s, rc=%d\n",
		       !val ? "disable" : "enable", rc);
	else
		chg_info("mmi set charging %s\n", !val ? "disable" : "enable");

	return (rc < 0) ? rc : count;
}
static DEVICE_ATTR_RW(mmi_charging_enable);

static ssize_t battery_notify_code_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int val = 0;

	if (!chip) {
                chg_err("chip is NULL\n");
                return -EINVAL;
        }

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	chg_info("notify_code %d, val %d\n", chip->notify_code, val);

	oplus_comm_set_anti_expansion_status(chip->comm_topic, val);

	return count;
}

static ssize_t battery_notify_code_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	return sprintf(buf, "%u\n", chip->notify_code);
}
static DEVICE_ATTR_RW(battery_notify_code);

#ifdef CONFIG_OPLUS_SHIP_MODE_SUPPORT
static ssize_t ship_mode_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	return sprintf(buf, "%d\n", chip->ship_mode);
}

static ssize_t ship_mode_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int val = 0;
	struct oplus_configfs_device *chip = dev->driver_data;
	int rc, rc2;

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	rc = oplus_wired_shipmode_enable(!!val);
	if (rc < 0) {
		chg_err("can't %s ship mode, rc=%d\n",
		       !!val ? "enable" : "disable", rc);
		return rc;
	} else {
		chg_err("%s charging\n", !!val ? "enable" : "disable");
		chip->ship_mode = !!val;
	}
	if (chip->ship_mode) {
		rc2 = oplus_gauge_update_soc_smooth_parameter();
		if (rc2 < 0)
			chg_err("can't update_soc_smooth_parameter in %s ship mode, rc=%d\n",
				!!val ? "enable" : "disable", rc2);
	}

	return (rc < 0) ? rc : count;
}
static DEVICE_ATTR_RW(ship_mode);
#endif /*CONFIG_OPLUS_SHIP_MODE_SUPPORT*/

#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
#ifdef CONFIG_OPLUS_SHORT_USERSPACE
static ssize_t short_c_limit_chg_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	/* struct oplus_configfs_device *chip = dev->driver_data; */

	/* return sprintf(buf, "%d\n", (int)chip->short_c_batt.limit_chg); */
	return 0;
}

static ssize_t short_c_limit_chg_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	int val = 0;
	/* struct oplus_configfs_device *chip = dev->driver_data; */

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	return count;
}
static DEVICE_ATTR_RW(short_c_limit_chg);

static ssize_t short_c_limit_rechg_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	/* struct oplus_configfs_device *chip = dev->driver_data; */

	/* return sprintf(buf, "%d\n", (int)chip->short_c_batt.limit_rechg); */
	return 0;
}

static ssize_t short_c_limit_rechg_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int val = 0;
	/* struct oplus_configfs_device *chip = dev->driver_data; */

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	return count;
}
static DEVICE_ATTR_RW(short_c_limit_rechg);

static ssize_t charge_term_current_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	/* struct oplus_configfs_device *chip = dev->driver_data; */

	/* return sprintf(buf, "%d\n", chip->limits.iterm_ma); */
	return 0;
}
static DEVICE_ATTR_RO(charge_term_current);

static ssize_t input_current_settled_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	int val = 0;
	/* struct oplus_configfs_device *chip = dev->driver_data; */

	return sprintf(buf, "%d\n", val);
}
static DEVICE_ATTR_RO(input_current_settled);
#endif /*CONFIG_OPLUS_SHORT_USERSPACE*/
#endif /*CONFIG_OPLUS_SHORT_C_BATT_CHECK*/

#ifdef CONFIG_OPLUS_SHORT_HW_CHECK
static ssize_t short_c_hw_feature_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	/* struct oplus_configfs_device *chip = dev->driver_data; */

	/* return sprintf(buf, "%d\n", chip->short_c_batt.is_feature_hw_on); */
	return 0;
}

static ssize_t short_c_hw_feature_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int val = 0;
	/* struct oplus_configfs_device *chip = dev->driver_data; */

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	return count;
}
static DEVICE_ATTR_RW(short_c_hw_feature);

static ssize_t short_c_hw_status_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	/* struct oplus_configfs_device *chip = dev->driver_data; */

	/* return sprintf(buf, "%d\n", chip->short_c_batt.shortc_gpio_status); */
	return 0;
}
static DEVICE_ATTR_RO(short_c_hw_status);
#endif /*CONFIG_OPLUS_SHORT_HW_CHECK*/

#ifdef CONFIG_OPLUS_SHORT_IC_CHECK
static ssize_t short_ic_otp_status_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	/* struct oplus_configfs_device *chip = dev->driver_data; */

	/* return sprintf(buf, "%d\n", chip->short_c_batt.ic_short_otp_st); */
	return 0;
}
static DEVICE_ATTR_RO(short_ic_otp_status);

static ssize_t short_ic_volt_thresh_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	/* struct oplus_configfs_device *chip = dev->driver_data; */

	/* return sprintf(buf, "%d\n", chip->short_c_batt.ic_volt_threshold); */
	return 0;
}

static ssize_t short_ic_volt_thresh_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	int val = 0;
	/* struct oplus_configfs_device *chip = dev->driver_data; */

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	return count;
}
static DEVICE_ATTR_RW(short_ic_volt_thresh);

static ssize_t short_ic_otp_value_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	/* struct oplus_configfs_device *chip = dev->driver_data; */

	/* return sprintf(buf, "%d\n", oplus_short_ic_get_otp_error_value(chip)); */
	return 0;
}
static DEVICE_ATTR_RO(short_ic_otp_value);
#endif /*CONFIG_OPLUS_SHORT_IC_CHECK*/

static ssize_t voocchg_ing_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	return sprintf(buf, "%d\n", chip->vooc_charging);
}
static DEVICE_ATTR_RO(voocchg_ing);

static ssize_t ppschg_ing_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int val = 0;

	if (chip->ufcs_online) {
		if (chip->ufcs_oplus_adapter)
			val = oplus_ufcs_adapter_id_to_protocol_type(chip->ufcs_adapter_id);
		else
			val = PROTOCOL_CHARGING_UFCS_THIRD;
	} else 	if (chip->pps_online || chip->pps_online_keep) {
		if (chip->pps_oplus_adapter)
			val = PROTOCOL_CHARGING_PPS_OPLUS;
		else
			val = PROTOCOL_CHARGING_PPS_THIRD;
	}

	return sprintf(buf, "%d\n", val);
}
static DEVICE_ATTR_RO(ppschg_ing);

static ssize_t ppschg_power_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int power = 0;

	if (chip->ufcs_online) {
		power = oplus_ufcs_get_ufcs_power(chip->ufcs_topic);
	} else 	if (chip->pps_online || chip->pps_online_keep) {
		power = oplus_pps_get_charging_power_watt(chip->pps_topic);
	}

	return sprintf(buf, "%d\n", power);
}
static DEVICE_ATTR_RO(ppschg_power);

static ssize_t bcc_exception_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	chg_err("power off\n");

	return count;
}
static DEVICE_ATTR_WO(bcc_exception);

int __attribute__((weak)) oplus_gauge_get_bcc_parameters(char *buf)
{
	return 0;
}

int __attribute__((weak)) oplus_gauge_set_bcc_parameters(const char *buf)
{
	return 0;
}

static bool eis_ctrl_fastchg(struct oplus_configfs_device *chip, int voocphy_support)
{
	int eis_disable_chg = 0;
	int eis_vooc_current = 0;
	int eis_ufcs_current = 0;

	if (is_chg_disable_votable_available(chip))
		eis_disable_chg = get_client_vote(chip->chg_disable_votable, EIS_VOTER);

	if (is_vooc_curr_votable_available(chip))
		eis_vooc_current = get_client_vote(chip->vooc_curr_votable, EIS_VOTER);

	if (is_ufcs_curr_votable_available(chip))
		eis_ufcs_current = get_client_vote(chip->ufcs_curr_votable, EIS_VOTER);

	if ((eis_vooc_current > 0 || eis_ufcs_current > 0 || eis_disable_chg > 0) &&
		voocphy_support != NO_VOOCPHY)
		return true;

	return false;
}

#define VOOC_MCU_PROJECT 7
#define VOOC_MCU_PROJECT_100W	8
static ssize_t bcc_parms_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int val;
	ssize_t len = 0;
	struct oplus_configfs_device *chip = dev->driver_data;
	int rc = 0;
	int vooc_project = 0;
	int voocphy_support = 0;
	int chg_type = oplus_wired_get_chg_type();

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (chip->vooc_topic) {
		vooc_project = oplus_vooc_get_project(chip->vooc_topic);
		voocphy_support = oplus_vooc_get_voocphy_support(chip->vooc_topic);
	} else {
		rc = -ENOTSUPP;
	}
	if (rc < 0) {
		chg_err("can't get vooc_started, rc=%d\n", rc);
	}

	/* I2C access conflict may occur when using MCU fast charging scheme.
	   only SVOOC charging need to get the bcc parameters.*/
	if ((vooc_project == VOOC_MCU_PROJECT || vooc_project == VOOC_MCU_PROJECT_100W)
		&& chg_type == OPLUS_CHG_USB_TYPE_SVOOC && voocphy_support == NO_VOOCPHY
		&& !eis_ctrl_fastchg(chip, voocphy_support)) {
		val = oplus_gauge_get_prev_bcc_parameters(buf);
		val = oplus_smart_chg_get_prev_battery_bcc_parameters(buf);
	} else {
		val = oplus_gauge_get_bcc_parameters(buf);
		val = oplus_smart_chg_get_battery_bcc_parameters(buf);
	}
	if (val < 0)
		chg_err("bcc parms get error\n");

	len = strlen(buf);

	return len;
}

static ssize_t bcc_parms_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int ret = 0;
	struct oplus_configfs_device *chip = dev->driver_data;
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	ret = oplus_gauge_set_bcc_parameters(buf);
	ret = oplus_smart_chg_set_bcc_debug_parameters(buf);
	if (ret < 0) {
		chg_err("error\n");
		return -EINVAL;
	}

	return count;
}
static DEVICE_ATTR_RW(bcc_parms);

static ssize_t bcc_current_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->bcc_current);
}

static ssize_t bcc_current_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val = 0;
	int val_ma = 0;
	struct oplus_configfs_device *chip = dev->driver_data;
	union mms_msg_data data = { 0 };
	int batt_num = 0;
	int wired_val_ma = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	val_ma = val * 100; /* 0.1A to mA */

	batt_num = oplus_gauge_get_batt_num();
	if (batt_num == 2)
		wired_val_ma = val_ma;
	else if (batt_num == 1)
		wired_val_ma = val_ma / 2;
	else {
		chg_err("batt_num[%d] is invalid\n", batt_num);
		return -EINVAL;
	}

	if (is_vooc_curr_votable_available(chip)) {
		chg_err("bcc current = %d, %d\n", val, wired_val_ma);
		/* turn current to mA, hidl will send as 73, it means 7300mA */
		vote(chip->vooc_curr_votable, BCC_VOTER,
			(wired_val_ma == 0) ? false : true, wired_val_ma, false);
	}

	if (chip->ufcs_online && is_ufcs_curr_votable_available(chip)) {
		chg_info("ufcs bcc current = %d, %d\n", val, wired_val_ma);
		vote(chip->ufcs_curr_votable, BCC_VOTER,
			(wired_val_ma == 0) ? false : true, wired_val_ma, false);
	}

	if (chip->pps_online && is_pps_curr_votable_available(chip)) {
		chg_info("pps bcc current = %d, %d\n", val, wired_val_ma);
		vote(chip->pps_curr_votable, BCC_VOTER,
			(wired_val_ma == 0) ? false : true, wired_val_ma, false);
	}

	if (chip->wls_topic) {
		oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_FASTCHG_STATUS, &data, true);

		if ((!!data.intval) && is_wls_curr_votable_available(chip)) {
			oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_FCC_TO_ICL, &data, true);
			chg_info("wls bcc current = %d, %d; fcc_to_icl:%d.\n", val, val_ma, data.intval);
			vote(chip->wls_fcc_curr_votable, BCC_CURRENT_VOTER, (val_ma == 0) ? false : true,
				(data.intval != 0) ? (val_ma / data.intval) : (val_ma / 2), false);
		}
	}

	oplus_wired_set_bcc_curr_request(chip->wired_topic);

	if (chip->wls_online)
		oplus_wired_check_bcc_curr_done(chip->wired_topic);

	/* oplus_wired_get_bcc_curr_done_status(); */

	return count;
}
static DEVICE_ATTR_RW(bcc_current);

static ssize_t eis_current_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int rc = 0;
	int voocphy_support = NO_VOOCPHY;
	int eis_current;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (chip->vooc_topic)
		voocphy_support = oplus_vooc_get_voocphy_support(chip->vooc_topic);
	else
		rc = -ENOTSUPP;

	if (rc < 0)
		chg_err("can't get vooc_started, rc=%d\n", rc);

	if (eis_ctrl_fastchg(chip, voocphy_support))
		eis_current = 1;
	else
		eis_current = -1;

	return sprintf(buf, "%d\n", eis_current);
}

#define EIS_MONITOR_TIMEOUT_MAX	45
static ssize_t eis_current_store(
	struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;
	struct oplus_configfs_device *chip = dev->driver_data;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	chg_info("<EIS> set eis current[%d]\n", val);
	chip->eis_current = val;
	schedule_work(&chip->eis_status_update_work);

	if (val >= 0) {
		chg_info("<EIS> enable eis_timeout_timer\n");
		cancel_delayed_work_sync(&chip->eis_timeout_work);
		schedule_delayed_work(&chip->eis_timeout_work,
			msecs_to_jiffies(EIS_MONITOR_TIMEOUT_MAX * 1000));
	} else {
		chg_info("<EIS> disable eis_timeout_timer\n");
		cancel_delayed_work_sync(&chip->eis_timeout_work);
	}

	return count;
}
static DEVICE_ATTR_RW(eis_current);

static ssize_t normal_current_now_store(struct device *dev, struct device_attribute *attr, const char *buf,
					size_t count)
{
	int val = 0;
	int rc;

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	val = val & 0XFFFF;
	rc = oplus_smart_chg_set_normal_current(val);
	if (rc < 0)
		return rc;

	return count;
}
DEVICE_ATTR_WO(normal_current_now);

static ssize_t normal_cool_down_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;
	int rc;

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}
	rc = oplus_smart_chg_set_normal_cool_down(val);
	if (rc < 0)
		return rc;

	return count;
}
DEVICE_ATTR_WO(normal_cool_down);

static ssize_t get_quick_mode_time_gain_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	long rc;

	rc = oplus_smart_chg_get_quick_mode_time_gain();
	if (rc < 0)
		return rc;

	return sprintf(buf, "%ld\n", rc);
}
static DEVICE_ATTR_RO(get_quick_mode_time_gain);

static ssize_t get_quick_mode_percent_gain_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc;

	rc = oplus_smart_chg_get_quick_mode_percent_gain();
	if (rc < 0)
		return rc;

	return sprintf(buf, "%d\n", rc);
}
static DEVICE_ATTR_RO(get_quick_mode_percent_gain);

static ssize_t aging_ffc_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int version, max_step;
	int ffc_pre_normal_fv_max, ffc_normal_fv_max;
	int index = 0;
	int i;

	version = oplus_comm_get_wired_aging_ffc_version(chip->comm_topic);
	max_step = oplus_comm_get_wired_ffc_step_max(chip->comm_topic);

	/* TODO: parallel charger */
	index += sprintf(buf, "%d,%d,%d,%d,%d", version, oplus_gauge_get_batt_num(),
			 0, chip->debug_batt_cc, chip->batt_cc);
	for (i = 0; i < max_step && i < FFC_CHG_STEP_MAX; i++) {
		ffc_pre_normal_fv_max =
			oplus_comm_get_wired_ffc_cutoff_fv(chip->comm_topic, i, FFC_TEMP_REGION_PRE_NORMAL) +
			oplus_comm_get_wired_aging_ffc_offset(chip->comm_topic, i);
		if (ffc_pre_normal_fv_max <= 0)
			break;
		ffc_normal_fv_max = oplus_comm_get_wired_ffc_cutoff_fv(chip->comm_topic, i, FFC_TEMP_REGION_NORMAL) +
				    oplus_comm_get_wired_aging_ffc_offset(chip->comm_topic, i);
		if (ffc_normal_fv_max <= 0)
			break;
		index += sprintf(buf + index, ",%d,%d", ffc_pre_normal_fv_max, ffc_normal_fv_max);
	}
	index += sprintf(buf + index, "\n");

	return index;
}

static ssize_t aging_ffc_data_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;
	struct oplus_configfs_device *chip = dev->driver_data;

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	chip->debug_batt_cc = val;

	return count;
}
static DEVICE_ATTR_RW(aging_ffc_data);

static ssize_t battery_log_head_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (oplus_battery_log_support() != true) {
		return -ENODEV;
	}

	return battery_log_common_operate(BATTERY_LOG_DUMP_LOG_HEAD,
		buf, BATTERY_LOG_MAX_SIZE);
}
static DEVICE_ATTR_RO(battery_log_head);

static ssize_t battery_log_content_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (oplus_battery_log_support() != true) {
		return -ENODEV;
	}

	return battery_log_common_operate(BATTERY_LOG_DUMP_LOG_CONTENT,
		buf, BATTERY_LOG_MAX_SIZE);
}
static DEVICE_ATTR_RO(battery_log_content);

static ssize_t parallel_chg_mos_test_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	union mms_msg_data data = { 0 };
	bool mos_status;
	int balancing_bat_status;
	int mos_test_result;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}
	if (!is_support_parallel_battery(chip->gauge_topic))
		return -ENODEV;

	if (is_parallel_topic_available(chip)) {
		oplus_mms_get_item_data(chip->parallel_topic, SWITCH_ITEM_HW_ENABLE_STATUS,
					&data, true);
		mos_status = data.intval;
		oplus_mms_get_item_data(chip->parallel_topic, SWITCH_ITEM_STATUS,
					&data, true);
		balancing_bat_status = data.intval;
	} else {
		chg_err("can't get parallel_topic\n");
		return -ENODEV;
	}


	if (!mos_status ||
	    balancing_bat_status == PARALLEL_BAT_BALANCE_ERROR_STATUS8 ||
	    balancing_bat_status == PARALLEL_BAT_BALANCE_ERROR_STATUS9) {
			chg_err("mos: %d, test next time!\n", mos_status);
			return 0;
	}
	mos_test_result = oplus_parallel_chg_mos_test(chip->parallel_topic);

	return sprintf(buf, "%d\n", mos_test_result);
}
static DEVICE_ATTR_RO(parallel_chg_mos_test);

static ssize_t parallel_chg_mos_status_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	union mms_msg_data data = { 0 };

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}
	if (!is_support_parallel_battery(chip->gauge_topic))
		return -ENODEV;

	if (is_parallel_topic_available(chip)) {
		oplus_mms_get_item_data(chip->parallel_topic, SWITCH_ITEM_HW_ENABLE_STATUS,
					&data, true);
	} else {
		chg_err("can't get parallel_topic\n");
		return -ENODEV;
	}

	return sprintf(buf, "%d\n", data.intval);
}
static DEVICE_ATTR_RO(parallel_chg_mos_status);

static ssize_t bms_heat_temp_compensation_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int bms_heat_temp_compensation;

	bms_heat_temp_compensation = oplus_comm_get_bms_heat_temp_compensation(chip->comm_topic);
	chg_debug("get bms compensation = %d\n", bms_heat_temp_compensation);

	return sprintf(buf, "%d\n", bms_heat_temp_compensation);
}

static ssize_t bms_heat_temp_compensation_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	int val = 0;
	struct oplus_configfs_device *chip = dev->driver_data;
	int bms_heat_temp_compensation;

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	bms_heat_temp_compensation = val;
	chg_debug("set bms compensation = %d\n", bms_heat_temp_compensation);

	oplus_comm_set_bms_heat_temp_compensation(chip->comm_topic, val);

	return count;
}
static DEVICE_ATTR_RW(bms_heat_temp_compensation);

static ssize_t pkg_name_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	oplus_chg_set_app_info(buf);
	return count;
}
static DEVICE_ATTR_WO(pkg_name);

static ssize_t dual_chan_info_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int val;
	ssize_t len = 0;

	val = oplus_get_dual_chan_info(buf);
	if (val < 0)
		chg_debug("dual_chan_chip is NULL\n");

	len = strlen(buf);
	return len;
}

static DEVICE_ATTR_RO(dual_chan_info);

static ssize_t slow_chg_en_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int slow_chg_batt_limit = 0;
	union mms_msg_data data = { 0 };
	int rc;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (chip->vooc_online && chip->vooc_topic) {
		rc = oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_SLOW_CHG_BATT_LIMIT, &data, true);
		if (rc == 0)
			slow_chg_batt_limit = data.intval;
	} else if (chip->ufcs_online && chip->ufcs_topic) {
		rc = oplus_mms_get_item_data(chip->ufcs_topic, UFCS_ITEM_SLOW_CHG_BATT_LIMIT, &data, true);
		if (rc == 0)
			slow_chg_batt_limit = data.intval;
	}

	return sprintf(buf, "%d,%d,%d,%d\n", chip->slow_chg_pct, chip->slow_chg_watt, chip->slow_chg_enable,
		       slow_chg_batt_limit);
}

static ssize_t slow_chg_en_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int pct = 0, watt = 0, en = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (!buf) {
		chg_err("buf is NULL\n");
		return -EINVAL;
	}

	if (sscanf(buf, "%d,%d,%d", &pct, &watt, &en) != 3) {
		chg_err("invalid buff %s\n", buf);
		return -EINVAL;
	}

	if (pct <= 0 || pct > 100 || watt <= 0 || watt >= 0xffff) {
		chg_err("pct %d or watt %d invalid\n", pct, watt);
		return -EINVAL;
	}

	oplus_configfs_push_slow_chg_info_msg(chip, pct, watt, (bool)!!en);
	oplus_comm_set_slow_chg(chip->comm_topic, pct, watt, (bool)!!en);
	chg_info("%d %d %d\n", pct, watt, en);
	return count;
}
static DEVICE_ATTR_RW(slow_chg_en);

static ssize_t battery_type_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	char battery_type_str[OPLUS_BATTERY_TYPE_LEN] = { 0 };
	int rc = oplus_gauge_get_battery_type_str(battery_type_str);

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (rc == 0)
		return sprintf(buf, "%s\n", battery_type_str);

	if (chip->deep_support)
		return sprintf(buf, "silicon\n");

	return sprintf(buf, "graphite\n");
}
static DEVICE_ATTR_RO(battery_type);

static ssize_t gauge_type_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int rc = -1;
	struct oplus_configfs_device *chip = dev->driver_data;
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	rc = oplus_get_gauge_type();

	return sprintf(buf, "%d\n", rc);
}
static DEVICE_ATTR_RO(gauge_type);

static ssize_t  battery_seal_flag_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int ret = 0;
	int seal_flag;
	struct oplus_configfs_device *chip = dev->driver_data;

	if (kstrtos32(buf, 0, &seal_flag)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	ret = oplus_gauge_set_seal_flag(chip->gauge_topic, seal_flag);
	if (ret < 0 && ret != -ENOTSUPP) {
		chg_err("set seal_flag error");
		 return -EINVAL;
	}

	return count;
}
static DEVICE_ATTR_WO(battery_seal_flag);

static ssize_t vbat_uv_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	return sprintf(buf, "%d\n", chip->vbat_uv_thr);
}
static DEVICE_ATTR_RO(vbat_uv);

static ssize_t subboard_temp_err_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	return sprintf(buf, "%d\n", chip->subboard_temp_err);
}
static DEVICE_ATTR_RO(subboard_temp_err);

static ssize_t eco_design_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	bool eco_design_support;
	struct oplus_configfs_device *chip = dev->driver_data;

	eco_design_support = !!(chip->nvid_support_flags & BIT(ECO_DESIGN_SUPPORT_REGION));
	return sprintf(buf, "%d\n", eco_design_support);
}
static DEVICE_ATTR_RO(eco_design_status);
static ssize_t rechg_soc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int rechg_soc;
	bool en;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	oplus_comm_get_rechg_soc_limit(chip->comm_topic, &rechg_soc, &en);

	return sprintf(buf, "%d,%d\n", en, rechg_soc);
}

static ssize_t rechg_soc_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int rechg_soc = 0, en = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (!buf) {
		chg_err("buf is NULL\n");
		return -EINVAL;
	}

	if (sscanf(buf, "%d,%d", &en, &rechg_soc) != 2) {
		chg_err("invalid buff %s\n", buf);
		return -EINVAL;
	}

	if (rechg_soc < 0 || rechg_soc > 100) {
		chg_err("rechg_soc %d invalid\n", rechg_soc);
		return -EINVAL;
	} else if ((en == 1) && (rechg_soc == 100)) {
		chg_err("disallow soc_rechg at 100\n");
		return -EINVAL;
	}

	oplus_comm_set_rechg_soc_limit(chip->comm_topic, rechg_soc, (bool)!!en);

	chg_info("%d,%d\n", en, rechg_soc);
	return count;
}
static DEVICE_ATTR_RW(rechg_soc);

static ssize_t batt_bal_data_show(
				struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc;
	ssize_t count = 0;
	union mms_msg_data data = { 0 };
	struct oplus_configfs_device *chip = dev->driver_data;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (is_batt_bal_topic_available(chip)) {
		rc = oplus_mms_get_item_data(
			chip->batt_bal_topic, BATT_BAL_ITEM_STATUS, &data, true);
		if (!rc && data.strval && strlen(data.strval))
			count = sprintf(buf, "%s\n", data.strval);
	}

	return count;
}
static DEVICE_ATTR_RO(batt_bal_data);

#define SEC_IC_TEST_TIMEOUT_MS (3000)
static void sec_ic_test_pack_res(struct oplus_sec_ic_test_res *res,
	int cmd, int valid)
{
	if (res == NULL) {
		chg_err("res or buff is NULL\n");
		return;
	}

	res->cmd = cmd;
	res->valid = valid;
}

#define SEC_IC_TEST_RES_BUSY 2
static ssize_t sec_ic_test_store(
	struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	struct oplus_sec_ic_test_res *res = NULL;
	int cmd = -1;
	char cmd_buf[2] = {0};
	int rc = 0;

	if (chip == NULL || buf == NULL) {
		chg_err("chip or buf is NULL\n");
		return -EINVAL;
	}

	memmove(cmd_buf, buf, sizeof(cmd_buf));
	cmd_buf[sizeof(cmd_buf) - 1] = '\0';
	res = &chip->sec_ic_test_res;

	if (READ_ONCE(res->busy)) {
		chg_err("sec_ic_test busy\n");
		return -EBUSY;
	}

	rc = sscanf(cmd_buf, "%d", &cmd);
	if (rc != 1) {
		chg_err("sscanf failed\n");
		return count;
	}
	chg_info("cmd:%d\n", cmd);

	WRITE_ONCE(res->busy, true);
	mutex_lock(&res->lock);
	reinit_completion(&res->ack);

	rc = oplus_sec_test_helper(chip->gauge_topic, cmd);
	if (rc == -EBUSY)
		sec_ic_test_pack_res(res, cmd, SEC_IC_TEST_RES_BUSY);
	else
		sec_ic_test_pack_res(res, cmd, (rc == 0));

	complete_all(&res->ack);
	mutex_unlock(&res->lock);
	WRITE_ONCE(res->busy, false);

	return count;
}

#define SEC_IC_TEST_RET_BUF_LEN 16
static ssize_t sec_ic_test_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	struct oplus_sec_ic_test_res *res = NULL;
	ssize_t count = 0;
	int rc = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	res = &chip->sec_ic_test_res;
	rc = wait_for_completion_timeout(&res->ack, msecs_to_jiffies(SEC_IC_TEST_TIMEOUT_MS));
	mutex_lock(&res->lock);
	if (!rc) {
		chg_err("sec ic test timeout\n");
		count = scnprintf(buf, SEC_IC_TEST_RET_BUF_LEN, "%d %d\n", res->cmd, -EINVAL);
		mutex_unlock(&res->lock);
		return count;
	}
	count = scnprintf(buf, SEC_IC_TEST_RET_BUF_LEN, "%d %d\n", res->cmd, res->valid);
	mutex_unlock(&res->lock);

	return count;
}
static DEVICE_ATTR_RW(sec_ic_test);

#define REVERSE_CHG_INFO_LEN 20
static ssize_t reverse_chg_info_store(
	struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	struct oplus_configfs_device *chip = dev->driver_data;
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	chg_err(" %s\n", buf);
	ret = oplus_reverse_chg_set_level(buf, count);
	if (ret < 0) {
		chg_err("error\n");
		return -EINVAL;
	}

	return count;
}

static ssize_t reverse_chg_info_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	int val;
	ssize_t len = 0;
	struct oplus_configfs_device *chip = dev->driver_data;

	if (!chip) {
		chg_err("reverse_chg_info_show chip is NULL\n");
		return -EINVAL;
	}

	val = oplus_reverse_chg_info_show(buf);
	if (val < 0) {
		chg_err("reverse_chg_info_show bcc parms get error\n");
		return val;
	}

	len = strlen(buf);

	chg_err(" reverse_chg_info_show end %s  %ld \n", buf, len);
	return len;
}
static DEVICE_ATTR_RW(reverse_chg_info);

#define GAUGE_CAR_C_BUFF_LEN 16
static ssize_t gauge_car_c_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return scnprintf(buf, GAUGE_CAR_C_BUFF_LEN, "%d\n", chip->gauge_car_c);
}
static DEVICE_ATTR_RO(gauge_car_c);

static ssize_t dual_cells_batt_health_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int health_status;
	int health_reason;
	int ret;
	struct oplus_configfs_device *chip = dev->driver_data;
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	ret = oplus_chg_get_dual_cells_batt_health(
			chip->protection_topic, &health_status, &health_reason);
	if (ret < 0)
		return sprintf(buf, "unsupport");

	chip->batt_health = health_status;
	return sprintf(buf, "%d,%d\n", health_reason, health_status);
}

static ssize_t dual_cells_batt_health_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int val = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}
	if (!buf) {
		chg_err("buf is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	oplus_chg_set_dual_cells_batt_health(chip->protection_topic, val);

	return count;
}
static DEVICE_ATTR_RW(dual_cells_batt_health);

static ssize_t chg_path_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;
	int callname = 0;
	struct oplus_configfs_device *chip = NULL;

#define PARAMETER_NUM	2

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if (get_eng_version() == OEM_RELEASE) {
		chg_err("OEM_RELEASE is ENOTSUPP\n");
		return -ENOTSUPP;
	}
#endif

	if (!dev || !buf) {
		chg_err("dev or buf is NULL\n");
		return -EINVAL;
	}

	chip = dev->driver_data;
	if (!chip || !chip->wired_topic) {
		chg_err("chip or wired_topic is NULL\n");
		return -EINVAL;
	}

	if (sscanf(buf, "chgpath=%dcallname=%d", &val, &callname) != PARAMETER_NUM) {
		chg_err("buf format error\n");
		return -EINVAL;
	}

	if (val < 0 || val >= CHG_PATH_MAX) {
		chg_err("data error\n");
		return -EINVAL;
	}
	chg_info("set chg path=%d, callname=%d\n", val, callname);
	oplus_wired_set_chg_path(chip->wired_topic, val);

	return count;
}
static DEVICE_ATTR_WO(chg_path);

static ssize_t bdd_voltdiff_trend_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int voltdiff_trend;

	voltdiff_trend = oplus_comm_get_bdd_voltdiff_trend(chip->comm_topic);
	chg_debug("get dual volt diff trend = %d\n", voltdiff_trend);

	return sprintf(buf, "%d\n", voltdiff_trend);
}
static DEVICE_ATTR_RO(bdd_voltdiff_trend);

static ssize_t gauge_nvram_stress_test_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	int test_count = 0;
	int interval = 0;
	struct oplus_configfs_device *chip = dev->driver_data;

	if (buf == NULL) {
		chg_err("NULL pointer detected\n");
		return 0;
	}

	if (sscanf(buf, "%d,%d", &test_count, &interval) != 2) {
		chg_err("invalid buff %s\n", buf);
		return -EINVAL;
	}

	chg_info("input param = %s, test_count[%d], internal[%d] start\n",
			 buf, test_count, interval);
	rc = oplus_gauge_start_stress_read_test(chip->gauge_topic, test_count, interval);
	if (rc < 0) {
		chg_err("input param[%s], test_count[%d], internal[%d], start read gauge failed. rc=%d\n",
			buf, test_count, interval, rc);
		return rc;
	}

	rc = oplus_gauge_start_nvram_stress_test(chip->gauge_topic, test_count, interval);
	if (rc < 0) {
		chg_err("input param[%s], test_count[%d], internal[%d], start nvram stress test failed. rc=%d\n",
			buf, test_count, interval, rc);
		return rc;
	}
	rc = oplus_gauge_start_term_volt_stress_test(chip->gauge_topic, test_count, interval);
	if (rc < 0) {
		chg_err("input param[%s], test_count[%d], internal[%d], start term volt stress test failed. rc=%d\n",
			buf, test_count, interval, rc);
		return rc;
	}
	chg_info("input param = %s, test_count[%d], internal[%d] start success\n",
		buf, test_count, interval);
	return count;
}

#define GAUGE_NVRAM_STRESS_TEST_BUFF_LEN 512
static ssize_t gauge_nvram_stress_test_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	struct oplus_gauge_nvram_stress_test data;
	int index = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	memset((char *)&data, 0, sizeof(struct oplus_gauge_nvram_stress_test));
	oplus_gauge_get_nvram_stress_test(chip->gauge_topic, &data);
	index = scnprintf(buf, GAUGE_NVRAM_STRESS_TEST_BUFF_LEN - index,
				"@nvram_test|input_cnt=%d|sum_cnt=%d|fail_cnt=%d|suc_cnt=%d|test_state=%d" \
				"@term_volt_test|input_cnt=%d|sum_cnt=%d|fail_cnt=%d|suc_cnt=%d|test_state=%d" \
				"@read_test|input_cnt=%d|sum_cnt=%d|fail_cnt=%d|suc_cnt=%d|test_state=%d\n",
				data.input_count, data.sum_cnt, data.fail_cnt,
				data.suc_cnt, data.test_state,
				data.input_count, data.term_volt_sum_cnt, data.term_volt_fail_cnt,
				data.term_volt_suc_cnt, data.term_volt_test_state,
				data.input_count, data.read_sum_cnt, data.read_fail_cnt,
				data.read_suc_cnt, data.read_test_state);
	return index;
}
DEVICE_ATTR_RW(gauge_nvram_stress_test);

#define GAUGE_THREE_LEVEL_TEST_BUFF_LEN 128
static ssize_t get_three_level_term_volt_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int data[4] = { 0 };
	oplus_gauge_get_three_level_term_volt(chip->gauge_topic, data);
	return scnprintf(buf, GAUGE_THREE_LEVEL_TEST_BUFF_LEN,
			"first_term_volt=%d,second_term_volt=%d,third_term_volt=%d\n",
			data[0], data[1], data[2]);
}
DEVICE_ATTR_RO(get_three_level_term_volt);

static struct device_attribute *oplus_battery_attributes[] = {
	&dev_attr_authenticate,
	&dev_attr_battery_cc,
	&dev_attr_battery_fcc,
	&dev_attr_battery_rm,
	&dev_attr_battery_soh,
#ifdef CONFIG_OPLUS_CALL_MODE_SUPPORT
	&dev_attr_call_mode,
#endif
	&dev_attr_charge_technology,
#ifdef CONFIG_OPLUS_CHIP_SOC_NODE
	&dev_attr_chip_soc,
#endif
	&dev_attr_gauge_vbat,
#ifdef CONFIG_OPLUS_SMART_CHARGER_SUPPORT
	&dev_attr_cool_down,
#endif
	&dev_attr_fast_charge,
	&dev_attr_mmi_charging_enable,
	&dev_attr_battery_notify_code,
#ifdef CONFIG_OPLUS_SHIP_MODE_SUPPORT
	&dev_attr_ship_mode,
#endif
#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
#ifdef CONFIG_OPLUS_SHORT_USERSPACE
	&dev_attr_short_c_limit_chg,
	&dev_attr_short_c_limit_rechg,
	&dev_attr_charge_term_current,
	&dev_attr_input_current_settled,
#endif
#endif
#ifdef CONFIG_OPLUS_SHORT_HW_CHECK
	&dev_attr_short_c_hw_feature,
	&dev_attr_short_c_hw_status,
#endif
#ifdef CONFIG_OPLUS_SHORT_IC_CHECK
	&dev_attr_short_ic_otp_status,
	&dev_attr_short_ic_volt_thresh,
	&dev_attr_short_ic_otp_value,
#endif
	&dev_attr_voocchg_ing,
	&dev_attr_ppschg_ing,
	&dev_attr_ppschg_power,
	&dev_attr_bcc_parms,
	&dev_attr_bcc_current,
	&dev_attr_bcc_exception,
	&dev_attr_normal_cool_down,
	&dev_attr_normal_current_now,
	&dev_attr_get_quick_mode_time_gain,
	&dev_attr_get_quick_mode_percent_gain,
	&dev_attr_aging_ffc_data,
	&dev_attr_design_capacity,
	&dev_attr_smartchg_soh_support,
	&dev_attr_battery_log_head,
	&dev_attr_battery_log_content,
	&dev_attr_parallel_chg_mos_test,
	&dev_attr_parallel_chg_mos_status,
	&dev_attr_bms_heat_temp_compensation,
	&dev_attr_pkg_name,
	&dev_attr_dual_chan_info,
	&dev_attr_slow_chg_en,
	&dev_attr_vbat_uv,
	&dev_attr_battery_type,
	&dev_attr_battery_sn,
	&dev_attr_battery_chem_id,
	&dev_attr_subboard_temp_err,
	&dev_attr_eco_design_status,
	&dev_attr_rechg_soc,
	&dev_attr_battery_manu_date,
	&dev_attr_battery_first_usage_date,
	&dev_attr_reset_gauge,
	&dev_attr_historic_soh_date,
	&dev_attr_batt_ic_sn,
	&dev_attr_battery_ui_cc,
	&dev_attr_battery_ui_soh,
	&dev_attr_battery_used_flag,
	&dev_attr_eis_current,
	&dev_attr_batt_bal_data,
	&dev_attr_gauge_type,
	&dev_attr_battery_seal_flag,
	&dev_attr_sec_ic_test,
	&dev_attr_gauge_car_c,
	&dev_attr_chg_path,
	&dev_attr_bdd_voltdiff_trend,
	&dev_attr_gauge_nvram_stress_test,
	&dev_attr_get_three_level_term_volt,
	&dev_attr_dual_cells_batt_health,
	NULL
};

/**********************************************************************
* wireless device nodes
**********************************************************************/
static ssize_t tx_voltage_now_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	/* struct oplus_configfs_device *chip = dev->driver_data; */

	return sprintf(buf, "%d\n", 0);
}
static DEVICE_ATTR_RO(tx_voltage_now);

static ssize_t tx_current_now_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	/* struct oplus_configfs_device *chip = dev->driver_data; */

	return sprintf(buf, "%d\n", 0);
}
static DEVICE_ATTR_RO(tx_current_now);

static ssize_t cp_voltage_now_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	/* struct oplus_configfs_device *chip = dev->driver_data; */

	return sprintf(buf, "%d\n", 0);
}
static DEVICE_ATTR_RO(cp_voltage_now);

static ssize_t cp_current_now_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	/* struct oplus_configfs_device *chip = dev->driver_data; */

	return sprintf(buf, "%d\n", 0);
}
static DEVICE_ATTR_RO(cp_current_now);

static ssize_t wireless_mode_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	/* struct oplus_configfs_device *chip = dev->driver_data; */

	return sprintf(buf, "%d\n", 0);
}
static DEVICE_ATTR_RO(wireless_mode);

static ssize_t wireless_type_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	/* struct oplus_configfs_device *chip = dev->driver_data; */

	return sprintf(buf, "%d\n", 0);
}
static DEVICE_ATTR_RO(wireless_type);

static ssize_t cep_info_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	/* struct oplus_configfs_device *chip = dev->driver_data; */

	return sprintf(buf, "%d\n", 0);
}
static DEVICE_ATTR_RO(cep_info);

static ssize_t real_type_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	union mms_msg_data data = { 0 };
	enum power_supply_type real_type = POWER_SUPPLY_TYPE_UNKNOWN;
	int rc;

	if (chip->wls_topic) {
		rc = oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_REAL_TYPE, &data, true);
		if (rc < 0)
			chg_err("can't get real type, rc=%d\n", rc);
		else
			real_type = data.intval;
	}

	return sprintf(buf, "%d\n", real_type);
}
static DEVICE_ATTR_RO(real_type);

static ssize_t status_keep_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int status_keep = 0;
	struct oplus_configfs_device *chip = NULL;
	union mms_msg_data data = { 0 };
	int rc;

	if (!dev || !buf) {
		chg_err("dev or buf is NULL\n");
		return -EINVAL;
	}

	chip = dev->driver_data;
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (chip->wls_topic) {
		rc = oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_STATUS_KEEP, &data, true);
		if (rc < 0)
			chg_err("can't get status_keep, rc=%d\n", rc);
		else
			status_keep = data.intval;
	}
	return sprintf(buf, "%d\n", status_keep);
}

static ssize_t status_keep_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;
	struct oplus_configfs_device *chip = NULL;
	union mms_msg_data data = { 0 };
	struct power_supply *batt_psy;
	int rc;

	if (!dev || !buf) {
		chg_err("dev or buf is NULL\n");
		return -EINVAL;
	}

	chip = dev->driver_data;
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	chg_info("set wls_status_keep=%d\n", val);
	if (val == WLS_SK_BY_HAL) {
		rc = oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_STATUS_KEEP, &data, true);
		if (rc < 0) {
			chg_err("can't get status_keep, rc=%d\n", rc);
		} else {
			if (data.intval == WLS_SK_NULL) {
				val = WLS_SK_NULL;
				chg_info("force to set wls_status_keep=%d\n", val);
			}
		}
	}
	(void)oplus_chg_wls_set_status_keep(chip->wls_topic, val);
	if (val == 0) {
		batt_psy = power_supply_get_by_name("battery");
		if (batt_psy)
			power_supply_changed(batt_psy);
	}

	return count;
}
static DEVICE_ATTR_RW(status_keep);

static ssize_t rx_disable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = NULL;

	if (!dev || !buf) {
		chg_err("dev or buf is NULL\n");
		return -EINVAL;
	}

	chip = dev->driver_data;
	if (!chip || !chip->wls_topic) {
		chg_err("chip or wls is NULL\n");
		return -ENODEV;
	}

	return oplus_chg_wls_rx_disable_show(chip->wls_topic, buf);
}

static ssize_t rx_disable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct oplus_configfs_device *chip = NULL;

	if (!dev || !buf) {
		chg_err("dev or buf is NULL\n");
		return -EINVAL;
	}

	chip = dev->driver_data;
	if (!chip || !chip->wls_topic) {
		chg_err("chip or wls is NULL\n");
		return -ENODEV;
	}

	count = oplus_chg_wls_rx_disable_store(chip->wls_topic, buf, count);

	return count;
}
static DEVICE_ATTR_RW(rx_disable);

static ssize_t bt_info_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct oplus_configfs_device *chip = NULL;

	if (!dev || !buf) {
		chg_err("dev or buf is NULL\n");
		return -EINVAL;
	}

	chip = dev->driver_data;
	if (!chip || !chip->wls_topic) {
		chg_err("chip or wls is NULL\n");
		return -ENODEV;
	}

	count = oplus_chg_wls_bt_info_store(chip->wls_topic, buf, count);

	return count;
}
static DEVICE_ATTR_WO(bt_info);

static ssize_t wlspen_info_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct oplus_configfs_device *chip = NULL;

	if (!dev || !buf) {
		chg_err("dev or buf is NULL\n");
		return -EINVAL;
	}

	chip = dev->driver_data;
	if (!chip || !chip->wls_topic) {
		chg_err("chip or wls is NULL\n");
		return -ENODEV;
	}

	count = oplus_chg_wls_wlspen_info_store(chip->wls_topic, buf, count);

	return count;
}
static DEVICE_ATTR_WO(wlspen_info);

static ssize_t wlspen_soc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = NULL;
	union mms_msg_data data = { 0 };
	int rc;

	if (!dev || !buf) {
		chg_err("dev or buf is NULL\n");
		return -EINVAL;
	}

	chip = dev->driver_data;
	if (!chip || !chip->wls_topic) {
		chg_err("chip or wls is NULL\n");
		return -ENODEV;
	}

	rc = oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_WLSPEN_SOC, &data, true);
	if (rc < 0) {
		chg_err("can't get wlspen_soc, rc=%d\n", rc);
		return rc;
	}

	return snprintf(buf, 16, "%d\n", data.intval);
}

static ssize_t wlspen_soc_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct oplus_configfs_device *chip = NULL;

	if (!dev || !buf) {
		chg_err("dev or buf is NULL\n");
		return -EINVAL;
	}

	chip = dev->driver_data;
	if (!chip || !chip->wls_topic) {
		chg_err("chip or wls is NULL\n");
		return -ENODEV;
	}

	count = oplus_chg_wls_wlspen_soc_store(chip->wls_topic, buf, count);

	return count;
}
static DEVICE_ATTR_RW(wlspen_soc);

static ssize_t wlspen_dischg_soc_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct oplus_configfs_device *chip = NULL;

	if (!dev || !buf) {
		chg_err("dev or buf is NULL\n");
		return -EINVAL;
	}

	chip = dev->driver_data;
	if (!chip || !chip->wls_topic) {
		chg_err("chip or wls is NULL\n");
		return -ENODEV;
	}

	count = oplus_chg_wls_wlspen_dischg_soc_store(chip->wls_topic, buf, count);

	return count;
}
static DEVICE_ATTR_WO(wlspen_dischg_soc);

static ssize_t ping_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	union mms_msg_data data = { 0 };
	int ping_time = 0;
	int rc;

	if (chip->wls_topic) {
		rc = oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_PING_TIME, &data, true);
		if (rc < 0)
			chg_err("can't get ping time, rc=%d\n", rc);
		else
			ping_time = data.intval;
	}

	return snprintf(buf, 16, "%d\n", ping_time);
}
static DEVICE_ATTR_RO(ping_time);

#ifdef WLS_QI_DEBUG
ssize_t __attribute__((weak))
oplus_chg_wls_upgrade_fw_show(struct oplus_mms *mms, char *buf)
{
	return 0;
}

ssize_t __attribute__((weak))
oplus_chg_wls_upgrade_fw_store(struct oplus_mms *mms, const char *buf, size_t count)
{
	return 0;
}

static ssize_t upgrade_firmware_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return oplus_chg_wls_upgrade_fw_show(chip->wls_topic, buf);
}

static ssize_t upgrade_firmware_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	count = oplus_chg_wls_upgrade_fw_store(chip->wls_topic, buf, count);

	return count;
}
static DEVICE_ATTR_RW(upgrade_firmware);
#endif /*WLS_QI_DEBUG*/

static struct device_attribute *oplus_wireless_attributes[] = {
	&dev_attr_tx_voltage_now,
	&dev_attr_tx_current_now,
	&dev_attr_cp_voltage_now,
	&dev_attr_cp_current_now,
	&dev_attr_wireless_mode,
	&dev_attr_wireless_type,
	&dev_attr_cep_info,
	&dev_attr_real_type,
	&dev_attr_status_keep,
	&dev_attr_rx_disable,
	&dev_attr_bt_info,
	&dev_attr_wlspen_info,
	&dev_attr_wlspen_soc,
	&dev_attr_wlspen_dischg_soc,
	&dev_attr_ping_time,
#ifdef WLS_QI_DEBUG
	&dev_attr_upgrade_firmware,
#endif
	NULL
};

/**********************************************************************
* common device nodes
**********************************************************************/
static ssize_t common_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	/* struct oplus_configfs_device *chip = dev->driver_data; */

	return sprintf(buf, "%s\n", "common: hello world!");
}
static DEVICE_ATTR(common, S_IRUGO, common_show, NULL);

static ssize_t track_hidl_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}


	oplus_chg_track_set_hidl_info(buf, count);

	return count;
}
static DEVICE_ATTR_WO(track_hidl);

static ssize_t boot_completed_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", oplus_comm_get_boot_completed());
}
static DEVICE_ATTR_RO(boot_completed);

static ssize_t chg_olc_config_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	oplus_chg_olc_config_set(buf);
	return count;
}

static ssize_t chg_olc_config_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int len = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	len = oplus_chg_olc_config_get(buf);
	return len;
}
static DEVICE_ATTR_RW(chg_olc_config);

static ssize_t time_zone_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	oplus_chg_track_time_zone_set(buf);
	return count;
}

static ssize_t time_zone_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return oplus_chg_track_time_zone_get(buf);
}
static DEVICE_ATTR_RW(time_zone);

static ssize_t battlog_push_config_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int val = 0;
	struct oplus_configfs_device *chip = dev->driver_data;
	int rc = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	if (!!val) {
		rc = oplus_chg_batterylog_exception_push();
		if (rc < 0)
			chg_err("push batterylog failed, rc=%d\n", rc);
		else
			chg_info("push batterylog successed\n");
	}

	return (rc < 0) ? rc : count;
}
static DEVICE_ATTR_WO(battlog_push_config);

static ssize_t deep_dischg_counts_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int counts = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	counts = oplus_gauge_show_deep_dischg_count(chip->gauge_topic);
	if (counts < 0)
		return counts;

	return sprintf(buf, "%d\n", counts);
}

static ssize_t deep_dischg_counts_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int val = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	oplus_gauge_set_deep_dischg_count(chip->gauge_topic, val);

	return count;
}
static DEVICE_ATTR_RW(deep_dischg_counts);

static ssize_t deep_dischg_count_cali_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int volt = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	volt = oplus_gauge_get_deep_count_cali(chip->gauge_topic);

	return sprintf(buf, "%d\n", volt);
}

static ssize_t deep_dischg_count_cali_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int val = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}
	oplus_gauge_set_deep_count_cali(chip->gauge_topic, val);

	return count;
}
static DEVICE_ATTR_RW(deep_dischg_count_cali);

static ssize_t  deep_dischg_ratio_thr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int thr = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	thr = oplus_gauge_get_deep_dischg_ratio_thr(chip->gauge_topic);

	return sprintf(buf, "%d\n", thr);
}


static ssize_t deep_dischg_ratio_thr_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int val = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	oplus_gauge_set_deep_dischg_ratio_thr(chip->gauge_topic, val);

	return count;
}
static DEVICE_ATTR_RW(deep_dischg_ratio_thr);

static ssize_t dec_delta_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int delta = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	oplus_comm_get_dec_delta(chip->comm_topic, &delta);

	return sprintf(buf, "delta=%d\n", delta);
}

static ssize_t dec_delta_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int val = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}
	oplus_comm_set_dec_delta(chip->comm_topic, val);

	return count;
}
static DEVICE_ATTR_RW(dec_delta);

#define PLC_ENABLE_DELAY_MS 2000
static void oplus_configfs_plc_enable_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_configfs_device *chip = container_of(dwork, struct oplus_configfs_device, plc_enable_work);
	int rc;

	if (!chip->plc_user_enable || !chip->wired_online)
		return;

	if (chip->plc_status == PLC_STATUS_NOT_ALLOW) {
		chip->plc_user_enable = false;
		chg_err("status not allow, change plc_user_enable to false\n");
		return;
	}

	chg_info("enable plc by kernel\n");
	rc = oplus_chg_plc_enable(chip->plc_topic, true);
	if (rc < 0)
		chg_err("plc enable error, rc=%d\n", rc);
}

static void oplus_configfs_plc_status_change_work(struct work_struct *work)
{
	struct power_supply *batt_psy;

	batt_psy = power_supply_get_by_name("battery");
	if (batt_psy) {
		oplus_power_supply_changed_gp(batt_psy, 0);
		power_supply_put(batt_psy);
	}
}

#define CLEAN_PLC_ENABLE_DELAY_MS 1600
static void oplus_configfs_clean_plc_enable_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_configfs_device *chip = container_of(dwork, struct oplus_configfs_device, clean_plc_enable_work);

	if (!chip->wired_online) {
		chip->plc_user_enable = false;
		chg_info("offline, change plc_user_enable to false\n");
	} else if (!chip->retention_state) {
		chip->plc_user_enable = false;
		chg_info("retention_state[false], change plc_user_enable to false\n");
	}
}

static ssize_t plc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	union mms_msg_data data = { 0 };
	int counts = 0;
	int rc;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	counts = chip->plc_status;
	if (chip->retention_state && chip->plc_user_enable)
		counts = PLC_STATUS_ENABLE;
	if (chip->keep_topic) {
		rc = oplus_mms_get_item_data(chip->keep_topic, STATE_KEEP_ITEM_PLC_STATUS, &data, true);
		if (rc < 0)
			chg_err("cannot get state_keep plc status, rc=%d\n", rc);
		else
			counts = data.intval;
	}

	return sprintf(buf, "status=%d\n", counts);
}

static ssize_t plc_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int val = 0;
	char key[64] = { 0 };
	int rc = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}
	if (!buf) {
		chg_err("buf is NULL\n");
		return -EINVAL;
	}

	if (sscanf(buf, "%63[^=]=%d", key, &val) != 2) {
		chg_info("buf %s error\n", buf);
		return -EINVAL;
	}

	if (sysfs_streq("switch", key)) {
		if (strncmp(buf, "switch=1|callname=", 18) && strncmp(buf, "switch=0|callname=", 18)) {
			chg_info("buf invalid: %s\n", buf);
			return -EINVAL;
		}
		chg_info("buf=[%s], change switch to  %d\n", buf, val);
		chip->plc_user_enable = !!val;
		rc = oplus_chg_plc_enable(chip->plc_topic, !!val);
		if (rc < 0) {
			chg_err("plc %s error, rc=%d\n", !!val ? "enable" : "disable", rc);
			return rc;
		}
	} else if (sysfs_streq("buck", key)) {
		chg_info("buf=[%s], change buck to %x\n", buf, val);
		if (is_plc_force_buck_votable_available(chip))
			vote(chip->plc_force_buck_votable, USER_VOTER, !!val, val, false);
	}

	return count;
}
static DEVICE_ATTR_RW(plc);

static int get_adapter_power(struct oplus_configfs_device *chip)
{
	int power = 0;
	int rc = 0;
	union mms_msg_data data = { 0 };
	int cur_sid = 0;

	if (chip->wls_online) {
		rc = oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_MAX_POWER, &data, true);
		if (rc == 0)
			power = data.intval;
	} else if (chip->ufcs_online) {
		power = oplus_ufcs_get_ufcs_power(chip->ufcs_topic) * 1000;
	} else 	if (chip->pps_online || chip->pps_online_keep) {
		power = oplus_pps_get_adapter_power_mw(chip->pps_topic);
	} else if (chip->vooc_online) {
		if (fast_chg_type_by_user > 0) {
			cur_sid = oplus_adapter_id_to_sid(chip->vooc_topic, fast_chg_type_by_user);
			chg_info("fast_chg_type_by_user sid = 0x%08x\n", cur_sid);
		} else {
			cur_sid = chip->vooc_sid;
		}
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
			if (is_cpa_topic_available(chip)) {
				oplus_mms_get_item_data(chip->cpa_topic, CPA_ITEM_ALLOW, &data, false);
				if (data.intval == CHG_PROTOCOL_PPS) {
					/* switching pps */
					power = 10000;
					break;
				}
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

static int adapter_power_by_user = -1;
static ssize_t adapter_power_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int power = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	power = get_adapter_power(chip);

	if (adapter_power_by_user > 0)
		power = adapter_power_by_user;
	return sprintf(buf, "%d\n", power);
}
static ssize_t adapter_power_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	adapter_power_by_user = val;

	return count;
}
static DEVICE_ATTR_RW(adapter_power);

static int protocol_type_by_user = -1;
static int protocol_type_get(void *priv_data)
{
	struct oplus_configfs_device *chip = priv_data;
	int fast_chg_type = 0;
	static int pre_fast_chg_type = 0;
	bool pd_use_default = false;
	union mms_msg_data data = { 0 };
	int rc;

	if (!chip->pd_boost_disable_votable)
		chip->pd_boost_disable_votable = find_votable("PD_BOOST_DISABLE");

	if (!chip->pd_svooc_votable)
		chip->pd_svooc_votable = find_votable("PD_SVOOC");

	if (chip->vooc_sid == 0) {
		switch (chip->wired_type) {
		case OPLUS_CHG_USB_TYPE_QC2:
		case OPLUS_CHG_USB_TYPE_QC3:
			fast_chg_type = CHARGER_SUBTYPE_QC;
			break;
		case OPLUS_CHG_USB_TYPE_UFCS:
			fast_chg_type = CHARGER_SUBTYPE_UFCS;
			break;
		case OPLUS_CHG_USB_TYPE_PD_PPS:
			if (chip->pps_online || chip->pps_online_keep) {
				fast_chg_type = CHARGER_SUBTYPE_PPS;
				break;
			} else if (is_cpa_topic_available(chip)) {
				oplus_mms_get_item_data(chip->cpa_topic, CPA_ITEM_ALLOW, &data, false);
				if (data.intval == CHG_PROTOCOL_PPS) {
					/* switching pps */
					pd_use_default = true;
					break;
				}
			}
			fallthrough;
		case OPLUS_CHG_USB_TYPE_PD:
		case OPLUS_CHG_USB_TYPE_PD_DRP:
			if (chip->pd_boost_disable_votable &&
			    is_client_vote_enabled(chip->pd_boost_disable_votable,
				   SVID_VOTER))
				pd_use_default = true;
			if (chip->pd_svooc_votable &&
			    !!get_effective_result(chip->pd_svooc_votable))
				pd_use_default = true;

			if (pd_use_default)
				fast_chg_type = CHARGER_SUBTYPE_DEFAULT;
			else
				fast_chg_type = CHARGER_SUBTYPE_PD;

			if (chip->pps_online_keep)
				fast_chg_type = CHARGER_SUBTYPE_PPS;
			break;
		default:
			break;
		}
	} else {
		if (sid_to_adapter_chg_type(chip->vooc_sid) ==
		    CHARGER_TYPE_VOOC)
			fast_chg_type = CHARGER_SUBTYPE_FASTCHG_VOOC;
		else if (sid_to_adapter_chg_type(chip->vooc_sid) ==
			 CHARGER_TYPE_SVOOC)
			fast_chg_type = CHARGER_SUBTYPE_FASTCHG_SVOOC;
	}

	if (chip->wls_online && !chip->wired_online) {
		rc = oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_WLS_TYPE, &data, true);
		if (rc < 0)
			fast_chg_type = CHARGER_SUBTYPE_DEFAULT;
		else if (data.intval == OPLUS_CHG_WLS_VOOC)
			fast_chg_type = CHARGER_SUBTYPE_FASTCHG_VOOC;
		else if (data.intval == OPLUS_CHG_WLS_SVOOC || data.intval == OPLUS_CHG_WLS_PD_65W)
			fast_chg_type = WLS_ADAPTER_TYPE_SVOOC;
		else
			fast_chg_type = CHARGER_SUBTYPE_DEFAULT;

		rc = oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_ICON_TYPE, &data, true);
		if (rc < 0)
			chg_err("wls get WLS_ITEM_ICON_TYPE err\n");
		else if (data.intval == OPLUS_CHG_WLS_SVOOC || data.intval == OPLUS_CHG_WLS_PD_65W)
			fast_chg_type = WLS_ADAPTER_TYPE_SVOOC;
	}

	if (protocol_type_by_user > 0)
		fast_chg_type = protocol_type_by_user;

	if (pre_fast_chg_type != fast_chg_type) {
		pre_fast_chg_type = fast_chg_type;
		chg_info("protocol_type_show %d %d %d,%d %d %d,%d %d %d\n",
			  fast_chg_type, chip->vooc_sid, chip->wired_type,
			  chip->pps_online, chip->pps_online_keep, pd_use_default,
			  chip->wls_online, protocol_type_by_user, chip->vooc_online);
	}

	return fast_chg_type;
}

static ssize_t protocol_type_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int fast_chg_type;
	union mms_msg_data data = { 0 };
	int rc;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (chip->keep_topic != NULL) {
		rc = oplus_mms_get_item_data(chip->keep_topic, STATE_KEEP_ITEM_FAST_CHG_TYPE, &data, true);
		if (rc < 0) {
			chg_err("get cpa_power failed, rc=%d\n", rc);
			fast_chg_type = 0;
		} else {
			fast_chg_type = data.intval;
		}
	} else {
		fast_chg_type = protocol_type_get(chip);
	}

	return sprintf(buf, "%u\n", fast_chg_type);
}

static ssize_t protocol_type_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	protocol_type_by_user = val;
	chg_info("protocol_type_by_user %d\n", protocol_type_by_user);

	return count;
}
static DEVICE_ATTR_RW(protocol_type);

static int oplus_get_project_watt(struct oplus_configfs_device *chip)
{
	int project_power = 0;

	if (is_cpa_topic_available(chip)) {
		project_power = oplus_cpa_protocol_get_max_power(chip->cpa_topic);
		if (project_power < 0)
			project_power = 0;
	} else {
		chg_err("can't get cpa_topic\n");
		return 0;
	}

	return project_power;
}

static int ui_power_by_user = -1;
static int ui_power_get(void *priv_data)
{
	struct oplus_configfs_device *chip = priv_data;
	int adapter_power = 0;
	int project_power = 0;
	int ui_power = 0;
	static int pre_ui_power = 0;
	int pps_or_ufcs_ing = 0;
	int pps_or_ufcs_power = 0;
	union mms_msg_data data = { 0 };
	int rc = 0;

	if (oplus_comm_get_dis_ui_power_state(chip->comm_topic))
		return ui_power;

	adapter_power = get_adapter_power(chip);
	project_power = oplus_get_project_watt(chip);

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
		ui_power = pps_or_ufcs_power * 1000;
	else
		ui_power = min(adapter_power, project_power);

	/* Display policy: when the ui_power is less than the project_power, the ui_power is 0. */
	if (ui_power < 0 || ui_power < project_power)
		ui_power = 0;

	if (chip->wls_online) {
			rc = oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_UI_POWER, &data, true);
			if (rc == 0)
				ui_power = data.intval;
	}

	if (ui_power_by_user > 0)
		ui_power = ui_power_by_user;

	if (pre_ui_power != ui_power) {
		pre_ui_power = ui_power;
		chg_info("ui_power_show %d %d %d %d %d, %d %d %d %d, %d %d %d\n",
			  adapter_power, project_power, chip->ufcs_online, chip->pps_online, chip->pps_online_keep,
			  chip->ufcs_oplus_adapter, chip->pps_oplus_adapter, pps_or_ufcs_ing, pps_or_ufcs_power,
			  ui_power, chip->ufcs_adapter_id, ui_power_by_user);
	}

	return ui_power;
}

static ssize_t ui_power_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int ui_power;
	union mms_msg_data data = { 0 };
	int rc;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (chip->keep_topic != NULL) {
		rc = oplus_mms_get_item_data(chip->keep_topic, STATE_KEEP_ITEM_UI_POWER, &data, true);
		if (rc < 0) {
			chg_err("get ui_power failed, rc=%d\n", rc);
			ui_power = 0;
		} else {
			ui_power = data.intval;
		}
	} else {
		ui_power = ui_power_get(chip);
	}

	return sprintf(buf, "%u\n", ui_power);
}

static ssize_t ui_power_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	ui_power_by_user = val;
	chg_info("ui_power_by_user %d\n", ui_power_by_user);

	return count;
}
static DEVICE_ATTR_RW(ui_power);

static int device_power_by_user = -1;
static ssize_t device_power_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int project_power = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	project_power = oplus_get_project_watt(chip);

	if (project_power < 0)
		project_power = 0;
	if (device_power_by_user > 0)
		project_power = device_power_by_user;
	return sprintf(buf, "%u\n", project_power);
}
static ssize_t device_power_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	device_power_by_user = val;
	chg_info("device_power_by_user %d\n", device_power_by_user);

	return count;
}
static DEVICE_ATTR_RW(device_power);

static int cpa_power_by_user = -1;
static int cpa_power_get(void *data)
{
	struct oplus_configfs_device *chip = data;
	int adapter_power = 0;
	int project_power = 0;
	int cpa_power = 0;
	int pps_or_ufcs_ing = 0;
	int pps_or_ufcs_power = 0;
	static int pre_cpa_power = 0;

	adapter_power = get_adapter_power(chip);
	project_power = oplus_get_project_watt(chip);

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
	if (cpa_power_by_user > 0)
		cpa_power = cpa_power_by_user;

	if (pre_cpa_power != cpa_power) {
		pre_cpa_power = cpa_power;
		chg_info("cpa_power_show %d %d %d %d, %d %d %d %d, %d %d %d\n",
			  adapter_power, project_power, chip->ufcs_online, chip->pps_online,
			  chip->ufcs_oplus_adapter, chip->pps_oplus_adapter, pps_or_ufcs_ing, pps_or_ufcs_power,
			  cpa_power, chip->ufcs_adapter_id, cpa_power_by_user);
	}

	return cpa_power;
}

static ssize_t cpa_power_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int cpa_power;
	union mms_msg_data data = { 0 };
	int rc;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (chip->keep_topic != NULL) {
		rc = oplus_mms_get_item_data(chip->keep_topic, STATE_KEEP_ITEM_CPA_POWER, &data, true);
		if (rc < 0) {
			chg_err("get cpa_power failed, rc=%d\n", rc);
			cpa_power = 0;
		} else {
			cpa_power = data.intval;
		}
	} else {
		cpa_power = cpa_power_get(chip);
	}

	return sprintf(buf, "%u\n", cpa_power);
}
static ssize_t cpa_power_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	cpa_power_by_user = val;
	chg_info("cpa_power_by_user %d\n", cpa_power_by_user);

	return count;
}
static DEVICE_ATTR_RW(cpa_power);

#define CHG_UP_PAGE_SIZE 128
#define PARMS_LEN 10
#define SEPRATOR_SIGN ","
char chg_up_buf[CHG_UP_PAGE_SIZE] = {0};
int oplus_update_chg_up_limit_parms(struct oplus_configfs_device *chip, const char *buf)
{
	int ret = 0;
	char  temp_buf[CHG_UP_PAGE_SIZE] = {0};
	char *buf_temp = temp_buf;
	char *buf_to_int_begian = temp_buf;
	int lenth_before = 0;
	int lenth_after = 0;
	char buf_atoi[CHG_UP_PAGE_SIZE] = {0};
	int parms[PARMS_LEN] = {0};
	int n = 0;

	if (NULL == buf) {
		return -ENOMEM;
	}

	if (strlen(buf) > CHG_UP_PAGE_SIZE) {
		chg_info("buf:%s\n", buf);
		return -ENOMEM;
	}
	strncpy(temp_buf, buf, strlen(buf));
	strncpy(chg_up_buf, buf, strlen(buf));

	while ((*buf_temp != '\0') && (n < 10)) {
		if (n >= PARMS_LEN) {
			chg_info("array lens is invalid\n");
			break;
		}

		buf_to_int_begian = buf_temp;
		lenth_before = strlen(buf_temp);
		buf_temp = strstr(buf_temp, SEPRATOR_SIGN);

		if (buf_to_int_begian == NULL) {
			return -ENOMEM;
		}

		if (buf_temp == NULL) {
			if (kstrtos32(buf_to_int_begian, 0, &parms[n])) {
				chg_err("buf error\n");
			}
			break;
		}
		buf_temp += 1;

		lenth_after = strlen(buf_temp);

		if (lenth_before <= lenth_after + 1) {
			return -ENOMEM;
		}

		strncpy(buf_atoi, buf_to_int_begian, (lenth_before - lenth_after - 1));
		if (kstrtos32(buf_atoi, 0, &parms[n])) {
			chg_err("buf_atoi[%d] error\n", n);
		}
		memset(buf_atoi, 0, sizeof(char) * CHG_UP_PAGE_SIZE);
		n++;
	}

	chg_info("chg_up_limit_show %d %d %d %d %d\n", parms[0], parms[1], parms[2], parms[3], parms[4]);
	ret = oplus_set_chg_up_limit(chip->comm_topic, parms[0], parms[1], parms[2], parms[3], parms[4]);

	return ret;
}
static ssize_t chg_up_limit_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%s\n", chg_up_buf);
}

static ssize_t chg_up_limit_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct oplus_configfs_device *chip = dev->driver_data;

	chg_info("chg_up_limit_store:%s\n", buf);
	rc = oplus_update_chg_up_limit_parms(chip, buf);
	if (rc < 0)
		return rc;

	return count;
}
DEVICE_ATTR_RW(chg_up_limit);

#define LPD_CONFIG_BUF_SIZE		128
static char lpd_config_buf[LPD_CONFIG_BUF_SIZE];
static ssize_t lpd_config_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%s\n", lpd_config_buf);
}

static ssize_t lpd_config_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char *parse_ptr, *trimmed, *token;
	struct oplus_configfs_device *chip = dev->driver_data;
	int values[MAX_LPD_CONFIG_NUM];
	int i = 0;
	int ret;
	size_t copy_size = min_t(size_t, count, LPD_CONFIG_BUF_SIZE - 1);

	memcpy(lpd_config_buf, buf, copy_size);
	lpd_config_buf[copy_size] = '\0';
	parse_ptr = lpd_config_buf;

	while ((token = strsep(&parse_ptr, ",")) != NULL && i < MAX_LPD_CONFIG_NUM) {
		trimmed = strim(token);
		if (*trimmed == '\0') {
			chg_err("Empty token at position %d\n", i);
			continue;
		}

		ret = kstrtoint(trimmed, 10, &values[i]);
		if (ret != 0) {
			chg_err("Invalid number '%s' at position %d\n", trimmed, i);
			return -EINVAL;
		}

		chg_info("values[%d] = %d\n", i, values[i]);
		i++;
	}

	oplus_wired_set_lpd_config(chip->wired_topic, values);
	return count;
}
DEVICE_ATTR_RW(lpd_config);

static ssize_t super_endurance_mode_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (!chip->deep_support) {
		chg_err("deep not support\n");
		return -ENOTSUPP;
	}

	return sprintf(buf, "%d\n", chip->super_endurance_mode_status);
}

static ssize_t super_endurance_mode_status_store(struct device *dev, struct device_attribute *attr, const char *buf,
						 size_t count)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int val = 0;
	struct mms_msg *msg;
	int rc;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (!chip->deep_support) {
		chg_err("deep not support\n");
		return -ENOTSUPP;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	if (!!val != chip->super_endurance_mode_status) {
		msg = oplus_mms_alloc_int_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, COMM_ITEM_SUPER_ENDURANCE_STATUS, !!val);
		if (msg == NULL) {
			chg_err("alloc msg error\n");
			return -ENOMEM;
		}
		rc = oplus_mms_publish_msg(chip->comm_topic, msg);
		if (rc < 0) {
			chg_err("publish comm super endurance status msg error, rc=%d\n", rc);
			kfree(msg);
		}
		chip->super_endurance_mode_status = !!val;
	}

	return count;
}
static DEVICE_ATTR_RW(super_endurance_mode_status);

static ssize_t super_endurance_mode_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (!chip->deep_support) {
		chg_err("deep not support\n");
		return -ENOTSUPP;
	}

	return sprintf(buf, "%d\n", chip->super_endurance_mode_count);
}

static ssize_t super_endurance_mode_count_store(struct device *dev, struct device_attribute *attr, const char *buf,
						size_t count)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int val = 0;
	struct mms_msg *msg;
	int rc;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (!chip->deep_support) {
		chg_err("deep not support\n");
		return -ENOTSUPP;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	if (val != chip->super_endurance_mode_count) {
		msg = oplus_mms_alloc_int_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, COMM_ITEM_SUPER_ENDURANCE_COUNT, val);
		if (msg == NULL) {
			chg_err("alloc msg error\n");
			return -ENOMEM;
		}
		rc = oplus_mms_publish_msg(chip->comm_topic, msg);
		if (rc < 0) {
			chg_err("publish comm super endurance count msg error, rc=%d\n", rc);
			kfree(msg);
		}
		chip->super_endurance_mode_count = val;
	}

	return count;
}
static DEVICE_ATTR_RW(super_endurance_mode_count);

static ssize_t read_gauge_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int value = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	value = oplus_mms_get_update_mode(chip->gauge_topic);

	return sprintf(buf, "%d\n", value);
}

static ssize_t read_gauge_reg_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int val = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	oplus_mms_set_update_mode(chip->gauge_topic, !!val);

	return count;
}
static DEVICE_ATTR_RW(read_gauge_reg);

static ssize_t sili_spare_power_enable_show(
				struct device *dev, struct device_attribute *attr, char *buf)
{
	union mms_msg_data data = { 0 };
	struct oplus_configfs_device *chip = dev->driver_data;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (!chip->gauge_topic) {
		chg_err("gauge_topic not ready\n");
		return -ENOTSUPP;
	}

	oplus_mms_get_item_data(
		chip->gauge_topic, GAUGE_ITEM_SPARE_POWER_ENABLE, &data, false);

	return sprintf(buf, "%d\n", data.intval);
}

static ssize_t sili_spare_power_enable_store(
		struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	int val = 0;
	struct mms_msg *msg;
	union mms_msg_data data = { 0 };
	struct oplus_configfs_device *chip = dev->driver_data;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (!chip->gauge_topic) {
		chg_err("gauge_topic not ready\n");
		return -ENOTSUPP;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	oplus_mms_get_item_data(
		chip->gauge_topic, GAUGE_ITEM_SPARE_POWER_ENABLE, &data, false);

	if (!!val != data.intval) {
		msg = oplus_mms_alloc_int_msg(
			MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, GAUGE_ITEM_SPARE_POWER_ENABLE, !!val);
		if (msg == NULL) {
			chg_err("alloc msg error\n");
			return -ENOMEM;
		}
		rc = oplus_mms_publish_msg(chip->gauge_topic, msg);
		if (rc < 0) {
			chg_err("publish comm sili spare power enable msg error, rc=%d\n", rc);
			kfree(msg);
		}
	}

	return count;
}
static DEVICE_ATTR_RW(sili_spare_power_enable);

static ssize_t sili_ic_alg_cfg_store(
		struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	int val = 0;
	struct mms_msg *msg;
	struct oplus_configfs_device *chip = dev->driver_data;
	static int alg_cfg = -EINVAL;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	if (val != alg_cfg) {
		msg = oplus_mms_alloc_int_msg(
			MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, GAUGE_ITEM_SILI_IC_ALG_CFG, val);
		if (msg == NULL) {
			chg_err("alloc msg error\n");
			return -ENOMEM;
		}
		rc = oplus_mms_publish_msg(chip->gauge_topic, msg);
		if (rc < 0) {
			chg_err("publish comm sili ic alg cfg msg error, rc=%d\n", rc);
			kfree(msg);
		}
		alg_cfg = val;
	}

	return count;
}
static DEVICE_ATTR_WO(sili_ic_alg_cfg);

#define BYB_INFO_LEN	1024
static ssize_t byb_status_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	char buffer[BYB_INFO_LEN] = {0};
	int rc = 0;

	if (chip == NULL) {
		chg_err("NULL pointer chip detected\n");
		return 0;
	}

	if (buf == NULL) {
		chg_err("NULL pointer buf detected\n");
		return 0;
	}
	rc = oplus_wired_get_byb_status_info(chip->wired_topic, buffer);
	if (rc < 0) {
		chg_err("can't get byb_status_info %s,%d\n", buffer, rc);
		return scnprintf(buf, PAGE_SIZE, "cannot_get_byb_status_info\n");
	}

	return scnprintf(buf, PAGE_SIZE, "%s\n", buffer);
}
static DEVICE_ATTR_RO(byb_status);

static ssize_t byb_vout_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	char buffer[BYB_INFO_LEN] = {0};
	int rc = 0;

	if (chip == NULL) {
		chg_err("NULL pointer chip detected\n");
		return 0;
	}

	if (buf == NULL) {
		chg_err("NULL pointer buf detected\n");
		return 0;
	}
	rc = oplus_wired_get_byb_vout_info(chip->wired_topic, buffer);
	if (rc < 0) {
		chg_err("can't get byb_vout_info %s,%d\n", buffer, rc);
		return scnprintf(buf, PAGE_SIZE, "cannot_get_byb_vout_info\n");
	}

	return scnprintf(buf, PAGE_SIZE, "%s\n", buffer);
}

static ssize_t byb_vout_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	int val = 0;
	struct oplus_configfs_device *chip = dev->driver_data;
	int rc;

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	rc = oplus_wired_set_byb_vout_info(chip->wired_topic, val);
	if (rc < 0) {
		chg_err("can't set byb_vout_info %s,%d\n", buf, val);
		return rc;
	}

	return count;
}
static DEVICE_ATTR_RW(byb_vout);

static ssize_t boost_ic_type_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	return sprintf(buf, "%d\n", chip->boost_ic_type);
}
static DEVICE_ATTR_RO(boost_ic_type);

static ssize_t boost_dev_id_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;

	return sprintf(buf, "%d\n", chip->boost_dev_id);
}
static DEVICE_ATTR_RO(boost_dev_id);

static ssize_t boost_cv_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int cv_mv = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (!(is_dischg_boost_topic_available(chip))) {
		chg_err("dischg_boost_topic is NULL\n");
		return -ENODEV;
	}

	cv_mv = oplus_boost_cv_mv_show(chip->dischg_boost_topic);
	if (cv_mv < 0)
		return cv_mv;

	return sprintf(buf, "%d\n", cv_mv);
}

static ssize_t boost_cv_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int val = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (!is_dischg_boost_topic_available(chip)) {
		chg_err("dischg_boost_topic is NULL\n");
		return -ENODEV;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	oplus_boost_cv_mv_store(chip->dischg_boost_topic, val);

	return count;
}
static DEVICE_ATTR_RW(boost_cv);

static ssize_t vbat_pwr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int vbat_pwr = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	vbat_pwr = oplus_wired_get_vbat_pwr();
	if (vbat_pwr < 0)
		return vbat_pwr;

	return sprintf(buf, "%d\n", vbat_pwr);
}
static DEVICE_ATTR_RO(vbat_pwr);

static ssize_t boost_disable_auto_mode_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int val = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (!is_dischg_boost_topic_available(chip)) {
		chg_err("dischg_boost_topic is NULL\n");
		return -ENODEV;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	oplus_boost_disable_auto_mode_store(chip->dischg_boost_topic, val);

	return count;
}
static DEVICE_ATTR_WO(boost_disable_auto_mode);

static ssize_t update_secondary_smooth_map_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct oplus_configfs_device *chip = dev->driver_data;
	int val = 0;
	int rc = 0;

	if (!chip || !is_comm_topic_available(chip)) {
		chg_err("chip or common topic is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	if (val != 0 && val != 1) {
		chg_err("invalid value: %d, expected 0 or 1\n", val);
		return -EINVAL;
	}

	rc = oplus_comm_smooth_strategy_rus_set_secondary_smooth_map(chip->comm_topic, !!val);
	if (rc < 0)
		chg_err("update failed\n");
	else
		chg_info("update succeeded\n");

	return count;
}
static DEVICE_ATTR_WO(update_secondary_smooth_map);

static struct device_attribute *oplus_common_attributes[] = {
	&dev_attr_common,
	&dev_attr_boot_completed,
	&dev_attr_chg_olc_config,
	&dev_attr_track_hidl,
	&dev_attr_time_zone,
	&dev_attr_battlog_push_config,
	&dev_attr_deep_dischg_counts,
	&dev_attr_deep_dischg_count_cali,
	&dev_attr_deep_dischg_ratio_thr,
	&dev_attr_adapter_power,
	&dev_attr_super_endurance_mode_status,
	&dev_attr_super_endurance_mode_count,
	&dev_attr_read_gauge_reg,
	&dev_attr_protocol_type,
	&dev_attr_ui_power,
	&dev_attr_device_power,
	&dev_attr_cpa_power,
	&dev_attr_sili_spare_power_enable,
	&dev_attr_sili_ic_alg_cfg,
	&dev_attr_chg_up_limit,
	&dev_attr_plc,
	&dev_attr_dec_delta,
	&dev_attr_lpd_config,
	&dev_attr_byb_status,
	&dev_attr_byb_vout,
	&dev_attr_reverse_chg_info,
	&dev_attr_boost_ic_type,
	&dev_attr_boost_dev_id,
	&dev_attr_boost_cv,
	&dev_attr_vbat_pwr,
	&dev_attr_boost_disable_auto_mode,
	&dev_attr_update_secondary_smooth_map,
	NULL
};

static int oplus_usb_dir_create(struct oplus_configfs_device *chip)
{
	int status = 0;
	dev_t devt;
	struct device_attribute **attrs;
	struct device_attribute *attr;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	status = alloc_chrdev_region(&devt, 0, 1, "usb");
	if (status < 0) {
		chg_err("alloc_chrdev_region usb fail!\n");
		return -ENOMEM;
	}
	chip->oplus_usb_dir = device_create(chip->oplus_chg_class, NULL, devt,
					    NULL, "%s", "usb");
	chip->oplus_usb_dir->devt = devt;
	dev_set_drvdata(chip->oplus_usb_dir, chip);

	attrs = oplus_usb_attributes;
	for (attr = *attrs; attr != NULL; attr = *(++attrs)) {
		int err;

		err = device_create_file(chip->oplus_usb_dir, attr);
		if (err) {
			chg_err("device_create_file fail!\n");
			device_destroy(chip->oplus_usb_dir->class,
				       chip->oplus_usb_dir->devt);
			return err;
		}
	}

	return 0;
}

static void oplus_usb_dir_destroy(struct device *dir_dev)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = oplus_usb_attributes;
	for (attr = *attrs; attr != NULL; attr = *(++attrs))
		device_remove_file(dir_dev, attr);
	device_destroy(dir_dev->class, dir_dev->devt);
	unregister_chrdev_region(dir_dev->devt, 1);
}

static int oplus_battery_dir_create(struct oplus_configfs_device *chip)
{
	int status = 0;
	dev_t devt;
	struct device_attribute **attrs;
	struct device_attribute *attr;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	status = alloc_chrdev_region(&devt, 0, 1, "battery");
	if (status < 0) {
		chg_err("alloc_chrdev_region battery fail!\n");
		return -ENOMEM;
	}

	chip->oplus_battery_dir = device_create(chip->oplus_chg_class, NULL,
						devt, NULL, "%s", "battery");
	chip->oplus_battery_dir->devt = devt;
	dev_set_drvdata(chip->oplus_battery_dir, chip);

	attrs = oplus_battery_attributes;
	for (attr = *attrs; attr != NULL; attr = *(++attrs)) {
		int err;

		err = device_create_file(chip->oplus_battery_dir, attr);
		if (err) {
			chg_err("device_create_file fail!\n");
			device_destroy(chip->oplus_battery_dir->class,
				       chip->oplus_battery_dir->devt);
			return err;
		}
	}

	return 0;
}

static void oplus_battery_dir_destroy(struct device *dir_dev)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = oplus_battery_attributes;
	for (attr = *attrs; attr != NULL; attr = *(++attrs))
		device_remove_file(dir_dev, attr);
	device_destroy(dir_dev->class, dir_dev->devt);
	unregister_chrdev_region(dir_dev->devt, 1);
}

static int oplus_wireless_dir_create(struct oplus_configfs_device *chip)
{
	int status = 0;
	dev_t devt;
	struct device_attribute **attrs;
	struct device_attribute *attr;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (chip->oplus_wireless_dir) {
		chg_info("oplus_wireless_dir already exist!\n");
		return 0;
	}

	status = alloc_chrdev_region(&devt, 0, 1, "wireless");
	if (status < 0) {
		chg_err("alloc_chrdev_region wireless fail!\n");
		return -ENOMEM;
	}

	chip->oplus_wireless_dir = device_create(chip->oplus_chg_class, NULL,
						 devt, NULL, "%s", "wireless");
	chip->oplus_wireless_dir->devt = devt;
	dev_set_drvdata(chip->oplus_wireless_dir, chip);

	attrs = oplus_wireless_attributes;
	for (attr = *attrs; attr != NULL; attr = *(++attrs)) {
		int err;

		err = device_create_file(chip->oplus_wireless_dir, attr);
		if (err) {
			chg_err("device_create_file fail!\n");
			device_destroy(chip->oplus_wireless_dir->class,
				       chip->oplus_wireless_dir->devt);
			return err;
		}
	}

	return 0;
}

static void oplus_wireless_dir_destroy(struct device *dir_dev)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = oplus_wireless_attributes;
	for (attr = *attrs; attr != NULL; attr = *(++attrs))
		device_remove_file(dir_dev, attr);
	device_destroy(dir_dev->class, dir_dev->devt);
	unregister_chrdev_region(dir_dev->devt, 1);
}

static int oplus_common_dir_create(struct oplus_configfs_device *chip)
{
	int status = 0;
	dev_t devt;
	struct device_attribute **attrs;
	struct device_attribute *attr;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	status = alloc_chrdev_region(&devt, 0, 1, "common");
	if (status < 0) {
		chg_err("alloc_chrdev_region common fail!\n");
		return -ENOMEM;
	}

	chip->oplus_common_dir = device_create(chip->oplus_chg_class, NULL,
					       devt, NULL, "%s", "common");
	chip->oplus_common_dir->devt = devt;
	dev_set_drvdata(chip->oplus_common_dir, chip);

	attrs = oplus_common_attributes;
	for (attr = *attrs; attr != NULL; attr = *(++attrs)) {
		int err;

		err = device_create_file(chip->oplus_common_dir, attr);
		if (err) {
			chg_err("device_create_file fail!\n");
			device_destroy(chip->oplus_common_dir->class,
				       chip->oplus_common_dir->devt);
			return err;
		}
	}

	return 0;
}

static void oplus_common_dir_destroy(struct device *dir_dev)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = oplus_common_attributes;
	for (attr = *attrs; attr != NULL; attr = *(++attrs))
		device_remove_file(dir_dev, attr);
	device_destroy(dir_dev->class, dir_dev->devt);
	unregister_chrdev_region(dir_dev->devt, 1);
}

/**********************************************************************
* configfs init APIs
**********************************************************************/
int oplus_usb_node_add(struct device_attribute **usb_attributes)
{
	struct oplus_configfs_device *chip = g_cfg_dev;
	struct device_attribute **attrs;
	struct device_attribute *attr;

	if (chip == NULL)
		return -ENODEV;
	if (chip->oplus_usb_dir == NULL)
		return -ENODEV;

	attrs = usb_attributes;
	for (attr = *attrs; attr != NULL; attr = *(++attrs)) {
		int err;

		err = device_create_file(chip->oplus_usb_dir, attr);
		if (err) {
			chg_err("device_create_file fail!\n");
			device_destroy(chip->oplus_usb_dir->class,
				       chip->oplus_usb_dir->devt);
			return err;
		}
	}

	return 0;
}

int oplus_usb_node_delete(struct device_attribute **usb_attributes)
{
	struct oplus_configfs_device *chip = g_cfg_dev;
	struct device_attribute **attrs;
	struct device_attribute *attr;

	if (chip == NULL)
		return -ENODEV;
	if (chip->oplus_usb_dir == NULL)
		return -ENODEV;

	attrs = usb_attributes;
	for (attr = *attrs; attr != NULL; attr = *(++attrs))
		device_remove_file(chip->oplus_usb_dir, attr);

	return 0;
}

int oplus_battery_node_add(struct device_attribute **battery_attributes)
{
	struct oplus_configfs_device *chip = g_cfg_dev;
	struct device_attribute **attrs;
	struct device_attribute *attr;

	if (chip == NULL)
		return -ENODEV;
	if (chip->oplus_battery_dir == NULL)
		return -ENODEV;

	attrs = battery_attributes;
	for (attr = *attrs; attr != NULL; attr = *(++attrs)) {
		int err;

		err = device_create_file(chip->oplus_battery_dir, attr);
		if (err) {
			chg_err("device_create_file fail!\n");
			device_destroy(chip->oplus_battery_dir->class,
				       chip->oplus_battery_dir->devt);
			return err;
		}
	}

	return 0;
}

int oplus_battery_node_delete(struct device_attribute **battery_attributes)
{
	struct oplus_configfs_device *chip = g_cfg_dev;
	struct device_attribute **attrs;
	struct device_attribute *attr;

	if (chip == NULL)
		return -ENODEV;
	if (chip->oplus_battery_dir == NULL)
		return -ENODEV;

	attrs = battery_attributes;
	for (attr = *attrs; attr != NULL; attr = *(++attrs))
		device_remove_file(chip->oplus_battery_dir, attr);

	return 0;
}

int oplus_wireless_node_add(struct device_attribute **wireless_attributes)
{
	struct oplus_configfs_device *chip = g_cfg_dev;
	struct device_attribute **attrs;
	struct device_attribute *attr;

	if (chip == NULL)
		return -ENODEV;
	if (chip->oplus_wireless_dir == NULL)
		return -ENODEV;

	attrs = wireless_attributes;
	for (attr = *attrs; attr != NULL; attr = *(++attrs)) {
		int err;

		err = device_create_file(chip->oplus_wireless_dir, attr);
		if (err) {
			chg_err("device_create_file fail!\n");
			device_destroy(chip->oplus_wireless_dir->class,
				       chip->oplus_wireless_dir->devt);
			return err;
		}
	}

	return 0;
}

int oplus_wireless_node_delete(struct device_attribute **wireless_attributes)
{
	struct oplus_configfs_device *chip = g_cfg_dev;
	struct device_attribute **attrs;
	struct device_attribute *attr;

	if (chip == NULL)
		return -ENODEV;
	if (chip->oplus_wireless_dir == NULL)
		return -ENODEV;

	attrs = wireless_attributes;
	for (attr = *attrs; attr != NULL; attr = *(++attrs))
		device_remove_file(chip->oplus_wireless_dir, attr);

	return 0;
}

int oplus_common_node_add(struct device_attribute **common_attributes)
{
	struct oplus_configfs_device *chip = g_cfg_dev;
	struct device_attribute **attrs;
	struct device_attribute *attr;

	if (chip == NULL)
		return -ENODEV;
	if (chip->oplus_common_dir == NULL)
		return -ENODEV;

	attrs = common_attributes;
	for (attr = *attrs; attr != NULL; attr = *(++attrs)) {
		int err;

		err = device_create_file(chip->oplus_common_dir, attr);
		if (err) {
			chg_err("device_create_file fail!\n");
			device_destroy(chip->oplus_common_dir->class,
				       chip->oplus_common_dir->devt);
			return err;
		}
	}

	return 0;
}

int oplus_common_node_delete(struct device_attribute **common_attributes)
{
	struct oplus_configfs_device *chip = g_cfg_dev;
	struct device_attribute **attrs;
	struct device_attribute *attr;

	if (chip == NULL)
		return -ENODEV;
	if (chip->oplus_common_dir == NULL)
		return -ENODEV;

	attrs = common_attributes;
	for (attr = *attrs; attr != NULL; attr = *(++attrs))
		device_remove_file(chip->oplus_common_dir, attr);

	return 0;
}

#define EIS_TIMEOUT_TXT	"$$err_reason@@timeout"
static void oplus_configfs_eis_timeout_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_configfs_device *chip =
		container_of(dwork, struct oplus_configfs_device, eis_timeout_work);

	chg_err("<EIS> eis process timeout!\n");
	oplus_configfs_reset_eis_status(chip);
	oplus_configfs_push_eis_timeout_info_msg(chip, EIS_TIMEOUT_TXT);
}

static void oplus_configfs_eis_reset_work(struct work_struct *work)
{
	struct oplus_configfs_device *chip = container_of(
		work, struct oplus_configfs_device, eis_reset_work);

	oplus_configfs_reset_eis_status(chip);
}

static void oplus_configfs_eis_common(struct oplus_configfs_device *chip)
{
	if (is_vooc_curr_votable_available(chip)) {
		if (chip->eis_current > 0)
			vote(chip->vooc_curr_votable, EIS_VOTER, true, chip->eis_current, false);
		else if (chip->eis_current < 0)
			vote(chip->vooc_curr_votable, EIS_VOTER, false, 0, false);
	}

	if (is_ufcs_curr_votable_available(chip)) {
		if (chip->eis_current > 0)
			vote(chip->ufcs_curr_votable, EIS_VOTER, true, chip->eis_current, false);
		else if (chip->eis_current < 0)
			vote(chip->ufcs_curr_votable, EIS_VOTER, false, 0, false);
	}
}

static void oplus_configfs_eis_with_charger(struct oplus_configfs_device *chip)
{
	if (is_wired_fcc_votable_available(chip)) {
		if (chip->eis_current > 0)
			vote(chip->wired_fcc_votable, EIS_VOTER, true, chip->eis_current, false);
		else
			vote(chip->wired_fcc_votable, EIS_VOTER, false, 0, false);
	}

	if (chip->eis_current > 0) {
		if (chip->eis_status == EIS_STATUS_DISABLE)
			oplus_configfs_push_eis_status_msg(chip, EIS_STATUS_PREPARE);
	} else if (chip->eis_current < 0) {
		oplus_configfs_push_eis_status_msg(chip, EIS_STATUS_DISABLE);
	} else {
		if (chip->eis_status == EIS_STATUS_HIGH_CURRENT)
			oplus_configfs_push_eis_status_msg(chip, EIS_STATUS_LOW_CURRENT);
		else
			oplus_configfs_push_eis_status_msg(chip, EIS_STATUS_DISABLE);
	}
}

static void oplus_configfs_eis_with_mos(struct oplus_configfs_device *chip)
{
	if (is_chg_disable_votable_available(chip)) {
		if (chip->eis_current)
			vote(chip->chg_disable_votable, EIS_VOTER, false, 0, false);
		else
			vote(chip->chg_disable_votable, EIS_VOTER, true, 1, false);
	}
}

static void oplus_configfs_eis_status_update_work(struct work_struct *work)
{
	struct oplus_configfs_device *chip = container_of(
		work, struct oplus_configfs_device, eis_status_update_work);
	int batt_num = 0;
	union mms_msg_data data = { 0 };
	int rc;

	oplus_configfs_eis_common(chip);

	rc = oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_BUCK_EIS_CURRENT_RATE, &data, true);
	if (rc < 0)
		data.intval = 0;

	batt_num = oplus_gauge_get_batt_num();
	chg_info("<EIS> eis current[%d], batt_num[%d], curr_rate[%d], eis_status[%d]\n",
		chip->eis_current, batt_num, data.intval, chip->eis_status);
	if ((batt_num == 1) || (data.intval != 0)) {
		if (data.intval != 0)
			chip->eis_current = chip->eis_current * data.intval;
		oplus_configfs_eis_with_charger(chip);
	} else {
		oplus_configfs_eis_with_mos(chip);
	}
}

static void oplus_configfs_gauge_update_work(struct work_struct *work)
{
	struct oplus_configfs_device *chip = container_of(
		work, struct oplus_configfs_device, gauge_update_work);
	union mms_msg_data data = { 0 };

	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOC, &data,
				false);
	chip->soc = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CC, &data,
				false);
	chip->batt_cc = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_FCC, &data,
				false);
	chip->batt_fcc = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_RM, &data,
				false);
	chip->batt_rm = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOH, &data,
				false);
	chip->batt_soh = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_GAUGE_VBAT, &data,
				false);
	chip->gauge_vbat = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CAR_C, &data,
				false);
	chip->gauge_car_c = data.intval;
}

static void oplus_configfs_gauge_subs_callback(struct mms_subscribe *subs,
					       enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_configfs_device *chip = subs->priv_data;
	union mms_msg_data data = { 0 };
	int rc;

	switch (type) {
	case MSG_TYPE_TIMER:
		schedule_work(&chip->gauge_update_work);
		break;
	case MSG_TYPE_ITEM:
		switch (id) {
		case GAUGE_ITEM_SUBBOARD_TEMP_ERR:
			rc = oplus_mms_get_item_data(chip->gauge_topic, id, &data, false);
			if (rc == 0)
				chip->subboard_temp_err = !!data.intval;
			break;
		case GAUGE_ITEM_FCC_COEFF:
			oplus_mms_get_item_data(chip->gauge_topic, id, &data, false);
			chip->batt_fcc_coeff = data.intval;
			break;
		case GAUGE_ITEM_SOH_COEFF:
			oplus_mms_get_item_data(chip->gauge_topic, id, &data, false);
			chip->batt_soh_coeff = data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_configfs_subscribe_gauge_topic(struct oplus_mms *topic,
						 void *prv_data)
{
	struct oplus_configfs_device *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->gauge_topic = topic;
	chip->gauge_subs =
		oplus_mms_subscribe(chip->gauge_topic, chip,
				    oplus_configfs_gauge_subs_callback,
				    "configfs");
	if (IS_ERR_OR_NULL(chip->gauge_subs)) {
		chg_err("subscribe gauge topic error, rc=%ld\n",
			PTR_ERR(chip->gauge_subs));
		return;
	}

	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOC, &data,
				true);
	chip->soc = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CC, &data,
				true);
	chip->batt_cc = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_FCC, &data,
				true);
	chip->batt_fcc = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_RM, &data,
				true);
	chip->batt_rm = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOH, &data,
				true);
	chip->batt_soh = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_GAUGE_VBAT, &data,
				true);
	chip->gauge_vbat = data.intval;

	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_DEEP_SUPPORT, &data, true);
	chip->deep_support = !!data.intval;

	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_FCC_COEFF, &data, true);
	chip->batt_fcc_coeff = data.intval;

	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOH_COEFF, &data, true);
	chip->batt_soh_coeff = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CAR_C, &data, true);
	chip->gauge_car_c = data.intval;

	return;
}

static void oplus_configfs_wired_subs_callback(struct mms_subscribe *subs,
					       enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_configfs_device *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WIRED_ITEM_ONLINE:
			oplus_mms_get_item_data(chip->wired_topic, id, &data,
						false);
			chip->wired_online = data.intval;
			if (!chip->wired_online) {
				schedule_work(&chip->eis_reset_work);
				if (chip->plc_topic && chip->plc_user_enable) {
					cancel_delayed_work(&chip->clean_plc_enable_work);
					schedule_delayed_work(&chip->clean_plc_enable_work,
						msecs_to_jiffies(CLEAN_PLC_ENABLE_DELAY_MS));
				}
			}
			break;
		case WIRED_ITEM_CHG_TYPE:
			oplus_mms_get_item_data(chip->wired_topic, id, &data,
						false);
			chip->wired_type = data.intval;
			break;
		case WIRED_ITEM_POWER_ROLE:
			oplus_mms_get_item_data(chip->wired_topic, id, &data, false);
			chip->power_role = data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_configfs_subscribe_wired_topic(struct oplus_mms *topic,
						 void *prv_data)
{
	struct oplus_configfs_device *chip = prv_data;
	/* union mms_msg_data data = { 0 }; */
	int rc;

	chip->wired_topic = topic;
	chip->wired_subs =
		oplus_mms_subscribe(chip->wired_topic, chip,
				    oplus_configfs_wired_subs_callback,
				    "configfs");
	if (IS_ERR_OR_NULL(chip->wired_subs)) {
		chg_err("subscribe wired topic error, rc=%ld\n",
			PTR_ERR(chip->wired_subs));
		return;
	}

	rc = oplus_usb_dir_create(chip);
	if (rc < 0) {
		chg_err("oplus_usb_dir_create fail!, rc=%d\n", rc);
		goto dir_err;
	}

	return;

dir_err:
	chip->oplus_usb_dir = NULL;
	oplus_mms_unsubscribe(chip->wired_subs);
}

static void oplus_configfs_wls_subs_callback(struct mms_subscribe *subs,
					     enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_configfs_device *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WLS_ITEM_PRESENT:
			oplus_mms_get_item_data(chip->wls_topic, id, &data, false);
			chip->wls_online = !!data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_configfs_subscribe_wls_topic(struct oplus_mms *topic,
					       void *prv_data)
{
	struct oplus_configfs_device *chip = prv_data;
	union mms_msg_data data = { 0 };
	int rc;

	chip->wls_topic = topic;
	chip->wls_subs = oplus_mms_subscribe(chip->wls_topic, chip,
					     oplus_configfs_wls_subs_callback,
					     "configfs");
	if (IS_ERR_OR_NULL(chip->wls_subs)) {
		chg_err("subscribe wls topic error, rc=%ld\n", PTR_ERR(chip->wls_subs));
		return;
	}

	oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_PRESENT, &data, true);
	chip->wls_online = !!data.intval;

	rc = oplus_wireless_dir_create(chip);
	if (rc < 0) {
		chg_err("oplus_wireless_dir_create fail!, rc=%d\n", rc);
		goto dir_err;
	}

	return;

dir_err:
	chip->oplus_wireless_dir = NULL;
	oplus_mms_unsubscribe(chip->wls_subs);
}

static void oplus_configfs_vooc_subs_callback(struct mms_subscribe *subs,
					      enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_configfs_device *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case VOOC_ITEM_VOOC_STARTED:
			oplus_mms_get_item_data(chip->vooc_topic, id, &data,
						false);
			chip->vooc_started = data.intval;
			break;
		case VOOC_ITEM_VOOC_CHARGING:
			oplus_mms_get_item_data(chip->vooc_topic, id, &data,
						false);
			chip->vooc_charging = data.intval;
			break;
		case VOOC_ITEM_ONLINE:
			oplus_mms_get_item_data(chip->vooc_topic, id, &data,
						false);
			chip->vooc_online = data.intval;
			break;
		case VOOC_ITEM_SID:
			oplus_mms_get_item_data(chip->vooc_topic, id, &data,
						false);
			chip->vooc_sid = (unsigned int)data.intval;
			break;
		case VOOC_ITEM_ONLINE_KEEP:
			oplus_mms_get_item_data(chip->vooc_topic, id, &data,
						false);
			chip->vooc_online_keep = (unsigned int)data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_configfs_subscribe_vooc_topic(struct oplus_mms *topic,
						void *prv_data)
{
	struct oplus_configfs_device *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->vooc_topic = topic;
	chip->vooc_subs = oplus_mms_subscribe(chip->vooc_topic, chip,
					      oplus_configfs_vooc_subs_callback,
					      "configfs");
	if (IS_ERR_OR_NULL(chip->vooc_subs)) {
		chg_err("subscribe vooc topic error, rc=%ld\n",
			PTR_ERR(chip->vooc_subs));
		return;
	}

	oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_VOOC_STARTED, &data,
				true);
	chip->vooc_started = data.intval;
	oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_VOOC_CHARGING,
				&data, true);
	chip->vooc_charging = data.intval;
	oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_ONLINE, &data,
				true);
	chip->vooc_online = data.intval;
	oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_ONLINE_KEEP, &data,
				true);
	chip->vooc_online_keep = data.intval;
	oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_SID, &data, true);
	chip->vooc_sid = (unsigned int)data.intval;
}

static void oplus_configfs_comm_subs_callback(struct mms_subscribe *subs,
					      enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_configfs_device *chip = subs->priv_data;
	union mms_msg_data data = { 0 };
	bool not_pop_up;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case COMM_ITEM_TEMP_REGION:
			break;
		case COMM_ITEM_NOTIFY_CODE:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->notify_code = data.intval;
			if (chip->notify_code | BIT(NOTIFY_BAT_NOT_CONNECT) ||
			    chip->notify_code | BIT(NOTIFY_BAT_FULL_THIRD_BATTERY)) {
				not_pop_up = oplus_comm_get_hmac_not_pop_up(chip->comm_topic);
				chg_err("not_pop_up = %d\n", not_pop_up);
				if (not_pop_up)
					chip->notify_code &= ~(BIT(NOTIFY_BAT_NOT_CONNECT) |
						BIT(NOTIFY_BAT_FULL_THIRD_BATTERY));
			}
			break;
		case COMM_ITEM_SLOW_CHG:
			oplus_mms_get_item_data(chip->comm_topic, id, &data, false);
			chip->slow_chg_pct = SLOW_CHG_TO_PCT(data.intval);
			chip->slow_chg_watt = SLOW_CHG_TO_WATT(data.intval);
			chip->slow_chg_enable = !!SLOW_CHG_TO_ENABLE(data.intval);
			break;
		case COMM_ITEM_VBAT_UV_THR:
			oplus_mms_get_item_data(chip->comm_topic, id, &data, false);
			chip->vbat_uv_thr = data.intval;
			break;
		case COMM_ITEM_NVID_SUPPORT_FLAGS:
			oplus_mms_get_item_data(chip->comm_topic, id, &data, false);
			chip->nvid_support_flags = data.intval;
			break;
		case COMM_ITEM_EIS_STATUS:
			oplus_mms_get_item_data(chip->comm_topic, id, &data, false);
			chip->eis_status = data.intval;
			chg_info("<EIS> update eis_status = %d\n", chip->eis_status);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_configfs_subscribe_comm_topic(struct oplus_mms *topic,
						void *prv_data)
{
	struct oplus_configfs_device *chip = prv_data;
	union mms_msg_data data = { 0 };
	int rc;
	bool not_pop_up;

	chip->comm_topic = topic;
	chip->comm_subs = oplus_mms_subscribe(chip->comm_topic, chip,
					      oplus_configfs_comm_subs_callback,
					      "configfs");
	if (IS_ERR_OR_NULL(chip->comm_subs)) {
		chg_err("subscribe common topic error, rc=%ld\n",
			PTR_ERR(chip->comm_subs));
		return;
	}

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_NOTIFY_CODE, &data,
				true);
	chip->notify_code = data.intval;
	if (chip->notify_code | BIT(NOTIFY_BAT_NOT_CONNECT) ||
	    chip->notify_code | BIT(NOTIFY_BAT_FULL_THIRD_BATTERY)) {
		not_pop_up = oplus_comm_get_hmac_not_pop_up(chip->comm_topic);
		chg_err("not_pop_up = %d\n", not_pop_up);
		if (not_pop_up)
			chip->notify_code &= ~(BIT(NOTIFY_BAT_NOT_CONNECT) |
				BIT(NOTIFY_BAT_FULL_THIRD_BATTERY));
	}
	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_VBAT_UV_THR, &data,
				true);
	chip->vbat_uv_thr = data.intval;

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_SLOW_CHG, &data, true);
	chip->slow_chg_pct = SLOW_CHG_TO_PCT(data.intval);
	chip->slow_chg_watt = SLOW_CHG_TO_WATT(data.intval);
	chip->slow_chg_enable = !!SLOW_CHG_TO_ENABLE(data.intval);

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_NVID_SUPPORT_FLAGS, &data, true);
	chip->nvid_support_flags = data.intval;

	rc = oplus_common_dir_create(chip);
	if (rc < 0) {
		chg_err("oplus_common_dir_create fail!, rc=%d\n", rc);
		goto dir_err;
	}

	return;

dir_err:
	chip->oplus_common_dir = NULL;
	oplus_mms_unsubscribe(chip->comm_subs);
}

static void oplus_configfs_ufcs_subs_callback(struct mms_subscribe *subs,
					     enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_configfs_device *chip = subs->priv_data;
	union mms_msg_data data = { 0 };
	int rc;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case UFCS_ITEM_ONLINE:
			rc = oplus_mms_get_item_data(chip->ufcs_topic, id, &data, false);
			if (rc < 0)
				break;
			chip->ufcs_online = !!data.intval;
			break;
		case UFCS_ITEM_CHARGING:
			rc = oplus_mms_get_item_data(chip->ufcs_topic, id, &data, false);
			if (rc < 0)
				break;
			chip->ufcs_charging = !!data.intval;
			break;
		case UFCS_ITEM_ADAPTER_ID:
			rc = oplus_mms_get_item_data(chip->ufcs_topic, id, &data, false);
			if (rc < 0)
				break;
			chip->ufcs_adapter_id = (u32)data.intval;
			break;
		case UFCS_ITEM_OPLUS_ADAPTER:
			rc = oplus_mms_get_item_data(chip->ufcs_topic, id, &data, false);
			if (rc < 0)
				break;
			chip->ufcs_oplus_adapter = !!data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_configfs_subscribe_ufcs_topic(struct oplus_mms *topic,
					       void *prv_data)
{
	struct oplus_configfs_device *chip = prv_data;
	union mms_msg_data data = { 0 };
	int rc;

	chip->ufcs_topic = topic;
	chip->ufcs_subs =
		oplus_mms_subscribe(chip->ufcs_topic, chip,
				    oplus_configfs_ufcs_subs_callback,
				    "configfs");
	if (IS_ERR_OR_NULL(chip->ufcs_subs)) {
		chg_err("subscribe ufcs topic error, rc=%ld\n",
			PTR_ERR(chip->ufcs_subs));
		return;
	}

	rc = oplus_mms_get_item_data(chip->ufcs_topic, UFCS_ITEM_ONLINE, &data, true);
	if (rc >= 0)
		chip->ufcs_online = !!data.intval;
	rc = oplus_mms_get_item_data(chip->ufcs_topic, UFCS_ITEM_CHARGING, &data, true);
	if (rc >= 0)
		chip->ufcs_charging = !!data.intval;
	rc = oplus_mms_get_item_data(chip->ufcs_topic, UFCS_ITEM_ADAPTER_ID, &data, true);
	if (rc >= 0)
		chip->ufcs_adapter_id = (u32)data.intval;
	rc = oplus_mms_get_item_data(chip->ufcs_topic, UFCS_ITEM_OPLUS_ADAPTER, &data, true);
	if (rc >= 0)
		chip->ufcs_oplus_adapter = !!data.intval;
}

static void oplus_configfs_pps_subs_callback(struct mms_subscribe *subs,
					     enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_configfs_device *chip = subs->priv_data;
	union mms_msg_data data = { 0 };
	int rc;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case PPS_ITEM_ONLINE:
			rc = oplus_mms_get_item_data(chip->pps_topic, id, &data, false);
			if (rc < 0)
				break;
			chip->pps_online = !!data.intval;
			break;
		case PPS_ITEM_ONLINE_KEEP:
			rc = oplus_mms_get_item_data(chip->pps_topic, id, &data, false);
			if (rc < 0)
				break;
			chip->pps_online_keep = !!data.intval;
			break;
		case PPS_ITEM_CHARGING:
			rc = oplus_mms_get_item_data(chip->pps_topic, id, &data, false);
			if (rc < 0)
				break;
			chip->pps_charging = !!data.intval;
			break;
		case PPS_ITEM_OPLUS_ADAPTER:
			rc = oplus_mms_get_item_data(chip->pps_topic, id, &data, false);
			if (rc < 0)
				break;
			chip->pps_oplus_adapter = !!data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_configfs_subscribe_pps_topic(struct oplus_mms *topic,
					       void *prv_data)
{
	struct oplus_configfs_device *chip = prv_data;
	union mms_msg_data data = { 0 };
	int rc;

	chip->pps_topic = topic;
	chip->pps_subs =
		oplus_mms_subscribe(chip->pps_topic, chip,
				    oplus_configfs_pps_subs_callback,
				    "configfs");
	if (IS_ERR_OR_NULL(chip->pps_subs)) {
		chg_err("subscribe ufcs topic error, rc=%ld\n",
			PTR_ERR(chip->ufcs_subs));
		return;
	}

	rc = oplus_mms_get_item_data(chip->pps_topic, PPS_ITEM_ONLINE, &data, true);
	if (rc >= 0)
		chip->pps_online = !!data.intval;
	rc = oplus_mms_get_item_data(chip->pps_topic, PPS_ITEM_ONLINE_KEEP, &data, true);
        if (rc >= 0)
                chip->pps_online_keep = !!data.intval;
	rc = oplus_mms_get_item_data(chip->pps_topic, PPS_ITEM_CHARGING, &data, true);
	if (rc >= 0)
		chip->pps_charging = !!data.intval;
	rc = oplus_mms_get_item_data(chip->pps_topic, PPS_ITEM_OPLUS_ADAPTER, &data, true);
	if (rc >= 0)
		chip->pps_oplus_adapter = !!data.intval;
}

static void oplus_configfs_retention_subs_callback(struct mms_subscribe *subs,
					     enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_configfs_device *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case RETENTION_ITEM_CONNECT_STATUS:
			oplus_mms_get_item_data(chip->retention_topic, id, &data, false);
			chip->retention_state = data.intval;
			if (chip->plc_topic && chip->plc_user_enable && chip->retention_state) {
				cancel_delayed_work(&chip->plc_enable_work);
				schedule_delayed_work(&chip->plc_enable_work, msecs_to_jiffies(PLC_ENABLE_DELAY_MS));
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

static void oplus_configfs_subscribe_retention_topic(struct oplus_mms *topic,
					     void *prv_data)
{
	struct oplus_configfs_device *chip = prv_data;
	union mms_msg_data data = { 0 };
	int rc;

	chip->retention_topic = topic;
	chip->retention_subs = oplus_mms_subscribe(chip->retention_topic, chip,
		oplus_configfs_retention_subs_callback, "configfs");
	if (IS_ERR_OR_NULL(chip->retention_subs)) {
		chg_err("subscribe retention topic error, rc=%ld\n",
			PTR_ERR(chip->retention_subs));
		return;
	}
	rc = oplus_mms_get_item_data(chip->retention_topic, RETENTION_ITEM_CONNECT_STATUS, &data, true);
	if (rc >= 0)
		chip->retention_state = !!data.intval;
}

static void oplus_configfs_subscribe_protection_topic(struct oplus_mms *topic,
					     void *prv_data)
{
	struct oplus_configfs_device *chip = prv_data;
	union mms_msg_data data = { 0 };
	int rc;

	chip->protection_topic = topic;
	rc = oplus_mms_get_item_data(chip->protection_topic, DUAL_CELLS_BATT_STATUS, &data, true);
	if (rc >= 0)
		chip->batt_health = !!data.intval;
}

static void oplus_configfs_plc_subs_callback(struct mms_subscribe *subs,
					      enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_configfs_device *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case PLC_ITEM_STATUS:
			oplus_mms_get_item_data(chip->plc_topic, id, &data, false);
			if (chip->plc_status != data.intval)
				schedule_delayed_work(&chip->plc_status_change_work, 0);
			chip->plc_status = data.intval;
			chg_info(" update plc_status=%d\n", chip->plc_status);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_configfs_subscribe_plc_topic(struct oplus_mms *topic,
						void *prv_data)
{
	struct oplus_configfs_device *chip = prv_data;
	union mms_msg_data data = { 0 };
	int rc;

	chip->plc_topic = topic;
	chip->plc_subs = oplus_mms_subscribe(chip->plc_topic, chip,
					      oplus_configfs_plc_subs_callback,
					      "configfs");
	if (IS_ERR_OR_NULL(chip->plc_topic)) {
		chg_err("subscribe plc topic error, rc=%ld\n",
			PTR_ERR(chip->plc_topic));
		return;
	}

	rc = oplus_mms_get_item_data(chip->plc_topic, PLC_ITEM_STATUS, &data, true);
	if (rc < 0)
		chg_err("can't get plc status data, rc=%d", rc);
	else
		chip->plc_status = data.intval;

	return;
}

static void oplus_configfs_dischg_boost_subs_callback(struct mms_subscribe *subs,
					      enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_configfs_device *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case DISCHG_BOOST_ITEM_IC_TYPE:
			oplus_mms_get_item_data(chip->dischg_boost_topic, id, &data, false);
			chip->boost_ic_type = data.intval;
			chg_info(" update boost_ic_type=%d\n", chip->boost_ic_type);
			break;
		case DISCHG_BOOST_ITEM_DEV_ID:
			oplus_mms_get_item_data(chip->dischg_boost_topic, id, &data, false);
			chip->boost_dev_id = data.intval;
			chg_info(" update boost_dev_id=%d\n", chip->boost_dev_id);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_configfs_subscribe_dischg_boost_topic(struct oplus_mms *topic,
						void *prv_data)
{
	struct oplus_configfs_device *chip = prv_data;
	union mms_msg_data data = { 0 };
	int rc;

	chip->dischg_boost_topic = topic;
	chip->dischg_boost_subs = oplus_mms_subscribe(chip->dischg_boost_topic, chip,
					      oplus_configfs_dischg_boost_subs_callback,
					      "configfs");
	if (IS_ERR_OR_NULL(chip->dischg_boost_topic)) {
		chg_err("subscribe dischg boost topic error, rc=%ld\n",
			PTR_ERR(chip->dischg_boost_topic));
		return;
	}

	rc = oplus_mms_get_item_data(chip->dischg_boost_topic, DISCHG_BOOST_ITEM_IC_TYPE, &data, true);
	if (rc < 0)
		chg_err("can't get dischg boost status data, rc=%d", rc);
	else
		chip->boost_ic_type = data.intval;

	rc = oplus_mms_get_item_data(chip->dischg_boost_topic, DISCHG_BOOST_ITEM_DEV_ID, &data, true);
	if (rc < 0)
		chg_err("can't get dischg boost status data, rc=%d", rc);
	else
		chip->boost_dev_id = data.intval;

	return;
}

#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
static void oplus_configfs_subscribe_keep_topic(struct oplus_mms *topic,
						void *prv_data)
{
	struct oplus_configfs_device *chip = prv_data;
	int rc;

	chip->keep_topic = topic;
	rc = state_keep_status_info_register(topic, STATE_KEEP_STATUS_CPA_POWER, cpa_power_get, chip);
	if (rc < 0)
		chg_err("can't register cpa power info, rc=%d", rc);
	rc = state_keep_status_info_register(topic, STATE_KEEP_STATUS_FAST_CHG_TYPE, protocol_type_get, chip);
	if (rc < 0)
		chg_err("can't register cpa power info, rc=%d", rc);
	rc = state_keep_status_info_register(topic, STATE_KEEP_STATUS_UI_POWER, ui_power_get, chip);
	if (rc < 0)
		chg_err("can't register cpa power info, rc=%d", rc);

	return;
}
#endif

static void oplus_configfs_reverse_subs_callback(struct mms_subscribe *subs,
					     enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_configfs_device *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case REVERSE_ITEM_REVERSE_CHG_TYPE:
			oplus_mms_get_item_data(chip->reverse_topic, id, &data, false);
			chip->reverse_chg_type = data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_configfs_subscribe_reverse_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_configfs_device *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->reverse_topic = topic;
	chip->reverse_subs = oplus_mms_subscribe(chip->reverse_topic, chip,
					      oplus_configfs_reverse_subs_callback,
					      "configfs");
	if (IS_ERR_OR_NULL(chip->reverse_subs)) {
		chg_err("subscribe reverse topic error, rc=%ld\n", PTR_ERR(chip->reverse_subs));
		return;
	}

	oplus_mms_get_item_data(chip->reverse_topic, REVERSE_ITEM_REVERSE_CHG_TYPE, &data, true);
	chip->reverse_chg_type = data.intval;
};

static __init int oplus_configfs_init(void)
{
	struct oplus_configfs_device *chip;
	int rc;

	chip = kzalloc(sizeof(struct oplus_configfs_device), GFP_KERNEL);
	if (chip == NULL) {
		chg_err("alloc memory error\n");
		return -ENOMEM;
	}
	g_cfg_dev = chip;

	INIT_WORK(&chip->gauge_update_work, oplus_configfs_gauge_update_work);
	INIT_WORK(&chip->eis_reset_work, oplus_configfs_eis_reset_work);
	INIT_WORK(&chip->eis_status_update_work, oplus_configfs_eis_status_update_work);
	INIT_DELAYED_WORK(&chip->eis_timeout_work, oplus_configfs_eis_timeout_work);
	INIT_DELAYED_WORK(&chip->plc_enable_work, oplus_configfs_plc_enable_work);
	INIT_DELAYED_WORK(&chip->clean_plc_enable_work, oplus_configfs_clean_plc_enable_work);
	INIT_DELAYED_WORK(&chip->plc_status_change_work, oplus_configfs_plc_status_change_work);
	init_completion(&chip->sec_ic_test_res.ack);
	mutex_init(&chip->sec_ic_test_res.lock);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0))
	chip->oplus_chg_class = class_create("oplus_chg");
#else
	chip->oplus_chg_class = class_create(THIS_MODULE, "oplus_chg");
#endif
	if (IS_ERR(chip->oplus_chg_class)) {
		chg_err("oplus_chg_configfs_init fail!\n");
		rc = PTR_ERR(chip->oplus_chg_class);
		goto class_create_err;
	}

	/* TODO: need to delete the wireless dir create after the wireless is supported */
	rc = oplus_wireless_dir_create(chip);
	if (rc < 0) {
		chg_err("oplus_usb_dir_create fail!, rc=%d\n", rc);
		chip->oplus_wireless_dir = NULL;
	}

	rc = oplus_battery_dir_create(chip);
	if (rc < 0) {
		chg_err("oplus_battery_dir_create fail!, rc=%d\n", rc);
		chip->oplus_battery_dir = NULL;
	}

	oplus_mms_wait_topic("gauge", oplus_configfs_subscribe_gauge_topic,
			     chip);
	oplus_mms_wait_topic("wired", oplus_configfs_subscribe_wired_topic,
			     chip);
	oplus_mms_wait_topic("wireless", oplus_configfs_subscribe_wls_topic,
			     chip);
	oplus_mms_wait_topic("vooc", oplus_configfs_subscribe_vooc_topic, chip);
	oplus_mms_wait_topic("common", oplus_configfs_subscribe_comm_topic,
			     chip);
	oplus_mms_wait_topic("ufcs", oplus_configfs_subscribe_ufcs_topic, chip);
	oplus_mms_wait_topic("pps", oplus_configfs_subscribe_pps_topic, chip);
	oplus_mms_wait_topic("retention", oplus_configfs_subscribe_retention_topic, chip);
	oplus_mms_wait_topic("plc", oplus_configfs_subscribe_plc_topic, chip);
	oplus_mms_wait_topic("reverse", oplus_configfs_subscribe_reverse_topic, chip);
	oplus_mms_wait_topic("dischg_boost", oplus_configfs_subscribe_dischg_boost_topic, chip);
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	oplus_mms_wait_topic("state_keep", oplus_configfs_subscribe_keep_topic, chip);
#endif
	oplus_mms_wait_topic("protection", oplus_configfs_subscribe_protection_topic, chip);

	return 0;

class_create_err:
	kfree(chip);
	return rc;
}

static __exit void oplus_configfs_exit(void)
{
	if (g_cfg_dev == NULL)
		return;

	if (g_cfg_dev->oplus_usb_dir)
		oplus_usb_dir_destroy(g_cfg_dev->oplus_usb_dir);
	if (g_cfg_dev->oplus_battery_dir)
		oplus_battery_dir_destroy(g_cfg_dev->oplus_battery_dir);
	if (g_cfg_dev->oplus_wireless_dir)
		oplus_wireless_dir_destroy(g_cfg_dev->oplus_wireless_dir);
	if (g_cfg_dev->oplus_common_dir)
		oplus_common_dir_destroy(g_cfg_dev->oplus_common_dir);
	if (!IS_ERR_OR_NULL(g_cfg_dev->wired_subs))
		oplus_mms_unsubscribe(g_cfg_dev->wired_subs);
	if (!IS_ERR_OR_NULL(g_cfg_dev->gauge_subs))
		oplus_mms_unsubscribe(g_cfg_dev->gauge_subs);
	if (!IS_ERR_OR_NULL(g_cfg_dev->wls_subs))
		oplus_mms_unsubscribe(g_cfg_dev->wls_subs);
	if (!IS_ERR_OR_NULL(g_cfg_dev->ufcs_subs))
		oplus_mms_unsubscribe(g_cfg_dev->ufcs_subs);
	if (!IS_ERR_OR_NULL(g_cfg_dev->pps_subs))
		oplus_mms_unsubscribe(g_cfg_dev->pps_subs);
	if (!IS_ERR_OR_NULL(g_cfg_dev->retention_subs))
		oplus_mms_unsubscribe(g_cfg_dev->retention_subs);
	if (!IS_ERR_OR_NULL(g_cfg_dev->plc_subs))
		oplus_mms_unsubscribe(g_cfg_dev->plc_subs);

	state_keep_status_info_unregister(g_cfg_dev->keep_topic, STATE_KEEP_STATUS_FAST_CHG_TYPE);
	state_keep_status_info_unregister(g_cfg_dev->keep_topic, STATE_KEEP_STATUS_CPA_POWER);
	state_keep_status_info_unregister(g_cfg_dev->keep_topic, STATE_KEEP_STATUS_UI_POWER);

	mutex_destroy(&g_cfg_dev->sec_ic_test_res.lock);

	if (!IS_ERR(g_cfg_dev->oplus_chg_class))
		class_destroy(g_cfg_dev->oplus_chg_class);

	kfree(g_cfg_dev);
	g_cfg_dev = NULL;
}

oplus_chg_module_register(oplus_configfs);
