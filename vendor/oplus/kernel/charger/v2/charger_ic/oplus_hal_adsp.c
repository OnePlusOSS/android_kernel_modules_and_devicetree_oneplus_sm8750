// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[ADSP]([%s][%d]): " fmt, __func__, __LINE__

#ifdef OPLUS_FEATURE_CHG_BASIC
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <linux/time.h>
#include <linux/jiffies.h>
#include <linux/sched/clock.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include <linux/iio/consumer.h>
#include <linux/pinctrl/consumer.h>
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/system/boot_mode.h>
#include <soc/oplus/system/oplus_project.h>
#endif
#include <linux/remoteproc/qcom_rproc.h>
#include <linux/rtc.h>
#include <linux/device.h>
#include <linux/of_platform.h>

#include <oplus_chg_ic.h>
#include <oplus_chg_module.h>
#include <oplus_chg.h>
#include <oplus_mms_wired.h>
#include <oplus_chg_vooc.h>
#include <oplus_chg_comm.h>
#include <oplus_chg_voter.h>
#include <oplus_mms_gauge.h>
#include <oplus_mms.h>
#include <oplus_chg_wls.h>
#include <../voocphy/oplus_adsp_voocphy.h>
#include <oplus_chg_pps.h>
#include <oplus_chg_ufcs.h>
#include <oplus_impedance_check.h>
#include "oplus_hal_adsp.h"
#include <plat_ufcs/plat_ufcs_notify.h>
#include <oplus_chg_cpa.h>
#include <ufcs_class.h>
#include <oplus_chg_monitor.h>
#include <oplus_chg_plc.h>
#include "oplus_pmic_glink_id.h"
#include <linux/pm_qos.h>

#define BCC_TYPE_IS_SVOOC 1
#define BCC_TYPE_IS_VOOC 0
#define LCM_CHECK_COUNT 5
#define LCM_CHARGER_VOL_THR_MV 2500
#define LCM_FREQUENCY_INTERVAL 5000
#define CPU_CLOCK_TIME_MS	1000000
#define OPLUS_HVDCP_DISABLE_INTERVAL round_jiffies_relative(msecs_to_jiffies(15000))
#define OPLUS_HVDCP_DETECT_TO_DETACH_TIME 3600
#define OEM_MISC_CTL_DATA_PAIR(cmd, enable) ((enable ? 0x3 : 0x1) << cmd)
#define OPLUS_PD_ONLY_CHECK_INTERVAL round_jiffies_relative(msecs_to_jiffies(300))
#define OPLUS_GET_BATT_INFO_FROM_ADSP_INTERVAL round_jiffies_relative(msecs_to_jiffies(100))
#define OPLUS_HBOOST_NOTIFY_INTERVAL round_jiffies_relative(msecs_to_jiffies(3000))
#define ABNORMAL_CHECK_USBIN_DELAY_MS	500
#define ABNORMAL_CHECK_USBIN_CNT	10

#define OPLUS_PD_5V 5000
#define OPLUS_PD_9V 9000
#define OPLUS_PD_12V 12000
#define CID_STATUS_DELAY_MS 55
#define OPLUS_VND_POWER_INFO_MAX 7
#define GAUGE_INVALID_TEMP (-400)
#define IBATT_FULL_CURR_DEFAULT 1000

#define AICL_POINT_VOL_5V           4100
#define HW_AICL_POINT_VOL_5V_PHASE1 4400
#define HW_AICL_POINT_VOL_5V_PHASE2 4500
#define USB_HW_AICL_POINT           4600
#define USB_SW_AICL_POINT           4620

#define TRACK_UPLOAD_COUNT_MAX			10
#define TRACK_LOCAL_T_NS_TO_S_THD		1000000000
#define TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD	(24 * 3600)

QCOM_PM_TYPEC_PORT_ROLE_TYPE qcom_typec_port_role[] = {
	QCOM_TYPEC_PORT_ROLE_DRP,
	QCOM_TYPEC_PORT_ROLE_SNK,
	QCOM_TYPEC_PORT_ROLE_SRC,
	QCOM_TYPEC_PORT_ROLE_DRP,
	QCOM_TYPEC_PORT_ROLE_INVALID,
	QCOM_TYPEC_PORT_ROLE_DISABLE,
	QCOM_TYPEC_PORT_ROLE_INVALID,
	QCOM_TYPEC_PORT_ROLE_INVALID
};

static const int oplus_prop_map[OPLUS_PROPERTY_MAX];
struct battery_chg_dev *g_bcdev = NULL;
static int oplus_get_vchg_trig_status(void);
static bool oplus_vchg_trig_is_support(void);
extern void oplus_usb_set_none_role(void);
static int oplus_get_voocphy_enable(struct battery_chg_dev *bcdev);
static int oplus_voocphy_enable(struct battery_chg_dev *bcdev, bool enable);
static int fg_sm8350_get_battery_soc(void);
static int oplus_chg_8350_output_is_suspend(struct oplus_chg_ic_dev *ic_dev, bool *suspend);
static int oplus_chg_8350_get_charger_type(struct oplus_chg_ic_dev *ic_dev, int *type);
static int smbchg_get_charge_enable(void);
static int oplus_chg_8350_get_icl(struct oplus_chg_ic_dev *ic_dev, int *icl_ma);
static int oplus_chg_set_input_current_with_no_aicl(struct battery_chg_dev *bcdev, int current_ma);
static bool oplus_vooc_get_fastchg_ing(struct battery_chg_dev *bcdev);
static bool is_common_topic_available(struct battery_chg_dev *bcdev);
static bool oplus_get_ufcs_charging(struct battery_chg_dev *bcdev);
static bool oplus_get_oplus_ufcs(struct battery_chg_dev *bcdev);
static unsigned int oplus_update_batt_full_para(struct battery_chg_dev *bcdev);
__maybe_unused static bool oplus_get_pps_charging(struct battery_chg_dev *bcdev);
__maybe_unused static bool oplus_get_oplus_pps(struct battery_chg_dev *bcdev);
static int oplus_chg_set_input_current(struct battery_chg_dev *bcdev, int current_ma);
static int oplus_get_pps_info_from_adsp(struct oplus_chg_ic_dev *ic_dev, u32 *pdo, int num);
static int oplus_chg_set_aicl_point(struct oplus_chg_ic_dev *ic_dev, int vbatt);
static int oplus_sm8350_get_lpd_info(struct oplus_chg_ic_dev *ic_dev, u32 *buf, u32 flag);
#endif /*OPLUS_FEATURE_CHG_BASIC*/

#ifdef OPLUS_FEATURE_CHG_BASIC
/*for p922x compile*/
void __attribute__((weak)) oplus_set_wrx_otg_value(void)
{
	return;
}
int __attribute__((weak)) oplus_get_idt_en_val(void)
{
	return -1;
}
int __attribute__((weak)) oplus_get_wrx_en_val(void)
{
	return -1;
}
int __attribute__((weak)) oplus_get_wrx_otg_val(void)
{
	return 0;
}
void __attribute__((weak)) oplus_wireless_set_otg_en_val(void)
{
	return;
}
void __attribute__((weak)) oplus_dcin_irq_enable(void)
{
	return;
}

static RAW_NOTIFIER_HEAD(hboost_notifier);

int register_hboost_event_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_register(&hboost_notifier, nb);
}
EXPORT_SYMBOL(register_hboost_event_notifier);

int unregister_hboost_event_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_unregister(&hboost_notifier, nb);
}
EXPORT_SYMBOL(unregister_hboost_event_notifier);

#define BATTERY_TYPE_EVENT 99
static void oplus_hboost_notify_work(struct work_struct *work)
{
	char battery_type_str[OPLUS_BATTERY_TYPE_LEN] = { 0 };
	int rc;
	int type;

	chg_info("start hboost notify work\n");
	rc = oplus_gauge_get_battery_type_str(battery_type_str);
	if (rc)
		chg_err("get battery type failed, rc=%d\n", rc);

	if (!strncmp(battery_type_str, "silicon", strlen("silicon"))) {
		type = 1;
		raw_notifier_call_chain(&hboost_notifier, BATTERY_TYPE_EVENT, &type);
		chg_err("notify hboost batt_type is silicon\n");
	}
}
#endif /*OPLUS_FEATURE_CHG_BASIC*/

static int oplus_chg_disable_charger(bool disable, const char *client_str)
{
	struct votable *disable_votable;
	int rc;

	disable_votable = find_votable("WIRED_CHARGING_DISABLE");
	if (!disable_votable) {
		chg_err("WIRED_CHARGING_DISABLE votable not found\n");
		return -EINVAL;
	}

	rc = vote(disable_votable, client_str, disable, 1, false);
	if (rc < 0)
		chg_err("%s charger error, rc = %d\n",
			     disable ? "disable" : "enable", rc);
	else
		chg_info("%s charger\n", disable ? "disable" : "enable");

	return rc;
}

static int oplus_chg_suspend_charger(bool suspend, const char *client_str)
{
	struct votable *suspend_votable;
	int rc;

	suspend_votable = find_votable("WIRED_CHARGE_SUSPEND");
	if (!suspend_votable) {
		chg_err("WIRED_CHARGE_SUSPEND votable not found\n");
		return -EINVAL;
	}

	rc = vote(suspend_votable, client_str, suspend, 1, false);
	if (rc < 0)
		chg_err("%s charger error, rc = %d\n",
			     suspend ? "suspend" : "unsuspend", rc);
	else
		chg_info("%s charger\n", suspend ? "suspend" : "unsuspend");

	return rc;
}

int oplus_chg_set_icl_by_vote(int icl, const char *client_str)
{
	struct votable *icl_votable;
	int rc;

	icl_votable = find_votable("WIRED_ICL");
	if (!icl_votable) {
		chg_err("WIRED_ICL votable not found\n");
		return -EINVAL;
	}

	rc = vote(icl_votable, client_str, true, icl, true);
	if (rc < 0)
		chg_err("set icl error: icl = %d, rc = %d\n", icl, rc);
	else
		chg_info("real icl = %d\n", icl);

	return rc;
}

__maybe_unused static bool is_usb_psy_available(struct battery_chg_dev *bcdev)
{
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (!pst->psy)
		pst->psy = power_supply_get_by_name("usb");
	return !!pst->psy;
}

__maybe_unused static bool is_batt_psy_available(struct battery_chg_dev *bcdev)
{
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	if (!pst->psy)
		pst->psy = power_supply_get_by_name("battery");
	return !!pst->psy;
}

__maybe_unused static bool is_wls_psy_available(struct battery_chg_dev *bcdev)
{
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_WLS];
	if (!pst->psy)
		pst->psy = power_supply_get_by_name("wireless");
	return !!pst->psy;
}

static const int battery_prop_map[BATT_PROP_MAX] = {
	[BATT_STATUS]		= POWER_SUPPLY_PROP_STATUS,
	[BATT_HEALTH]		= POWER_SUPPLY_PROP_HEALTH,
	[BATT_PRESENT]		= POWER_SUPPLY_PROP_PRESENT,
	[BATT_CHG_TYPE]		= POWER_SUPPLY_PROP_CHARGE_TYPE,
	[BATT_CAPACITY]		= POWER_SUPPLY_PROP_CAPACITY,
	[BATT_VOLT_OCV]		= POWER_SUPPLY_PROP_VOLTAGE_OCV,
	[BATT_VOLT_NOW]		= POWER_SUPPLY_PROP_VOLTAGE_NOW,
	[BATT_VOLT_MAX]		= POWER_SUPPLY_PROP_VOLTAGE_MAX,
	[BATT_CURR_NOW]		= POWER_SUPPLY_PROP_CURRENT_NOW,
	[BATT_CHG_CTRL_LIM]	= POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	[BATT_CHG_CTRL_LIM_MAX]	= POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	[BATT_TEMP]		= POWER_SUPPLY_PROP_TEMP,
	[BATT_TECHNOLOGY]	= POWER_SUPPLY_PROP_TECHNOLOGY,
	[BATT_CHG_COUNTER]	= POWER_SUPPLY_PROP_CHARGE_COUNTER,
	[BATT_CYCLE_COUNT]	= POWER_SUPPLY_PROP_CYCLE_COUNT,
	[BATT_CHG_FULL_DESIGN]	= POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	[BATT_CHG_FULL]		= POWER_SUPPLY_PROP_CHARGE_FULL,
	[BATT_MODEL_NAME]	= POWER_SUPPLY_PROP_MODEL_NAME,
	[BATT_TTF_AVG]		= POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	[BATT_TTE_AVG]		= POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	[BATT_POWER_NOW]	= POWER_SUPPLY_PROP_POWER_NOW,
	[BATT_POWER_AVG]	= POWER_SUPPLY_PROP_POWER_AVG,
};

static const int usb_prop_map[USB_PROP_MAX] = {
	[USB_ONLINE]		= POWER_SUPPLY_PROP_ONLINE,
	[USB_VOLT_NOW]		= POWER_SUPPLY_PROP_VOLTAGE_NOW,
	[USB_VOLT_MAX]		= POWER_SUPPLY_PROP_VOLTAGE_MAX,
	[USB_CURR_NOW]		= POWER_SUPPLY_PROP_CURRENT_NOW,
	[USB_CURR_MAX]		= POWER_SUPPLY_PROP_CURRENT_MAX,
	[USB_INPUT_CURR_LIMIT]	= POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	[USB_ADAP_TYPE]		= POWER_SUPPLY_PROP_USB_TYPE,
	[USB_TEMP]		= POWER_SUPPLY_PROP_TEMP,
};

static const int wls_prop_map[WLS_PROP_MAX] = {
	[WLS_ONLINE]		= POWER_SUPPLY_PROP_ONLINE,
	[WLS_VOLT_NOW]		= POWER_SUPPLY_PROP_VOLTAGE_NOW,
	[WLS_VOLT_MAX]		= POWER_SUPPLY_PROP_VOLTAGE_MAX,
	[WLS_CURR_NOW]		= POWER_SUPPLY_PROP_CURRENT_NOW,
	[WLS_CURR_MAX]		= POWER_SUPPLY_PROP_CURRENT_MAX,
	[WLS_INPUT_CURR_LIMIT]	= POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	[WLS_CONN_TEMP]		= POWER_SUPPLY_PROP_TEMP,
};

/* Standard usb_type definitions similar to power_supply_sysfs.c */
static const char * const power_supply_usb_type_text[] = {
	"Unknown", "SDP", "DCP", "CDP", "ACA", "C",
	"PD", "PD_DRP", "PD_PPS", "BrickID"
};

/* Custom usb_type definitions */
static const char * const qc_power_supply_usb_type_text[] = {
	"HVDCP", "HVDCP_3", "HVDCP_3P5"
};

#ifdef OPLUS_FEATURE_CHG_BASIC
static bool is_err_topic_available(struct battery_chg_dev *chip)
{
	if (!chip->err_topic)
		chip->err_topic = oplus_mms_get_by_name("error");
	return !!chip->err_topic;
}

static void oplus_publish_close_cp_item_work(struct work_struct *work)
{
	struct battery_chg_dev *chip = container_of(work, struct battery_chg_dev, publish_close_cp_item_work.work);
	struct mms_msg *msg;
	int rc;

	if (!is_err_topic_available(chip)) {
		chg_err("error topic not found\n");
		return;
	}

	msg = oplus_mms_alloc_int_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_CLOSE_CP, 1);
	if (msg == NULL) {
		chg_err("alloc close cp msg error\n");
		return;
	}

	rc = oplus_mms_publish_msg(chip->err_topic, msg);
	if (rc < 0) {
		chg_err("publish close cp msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static int oem_battery_chg_write(struct battery_chg_dev *bcdev, void *data,
	int len)
{
	int rc;

	if (atomic_read(&bcdev->state) == PMIC_GLINK_STATE_DOWN) {
		chg_err("glink state is down\n");
		return -ENOTCONN;
	}

	mutex_lock(&bcdev->read_buffer_lock);
	reinit_completion(&bcdev->oem_read_ack);
	rc = pmic_glink_write(bcdev->client, data, len);
	if (!rc) {
		rc = wait_for_completion_timeout(&bcdev->oem_read_ack,
			msecs_to_jiffies(OEM_READ_WAIT_TIME_MS));
		if (!rc) {
			chg_err("Error, timed out sending message\n");
			mutex_unlock(&bcdev->read_buffer_lock);
			return -ETIMEDOUT;
		}

		rc = 0;
	}

	mutex_unlock(&bcdev->read_buffer_lock);

	return rc;
}

static int oem_read_buffer(struct battery_chg_dev *bcdev)
{
	struct oem_read_buffer_req_msg req_msg = { { 0 } };

	req_msg.data_size = sizeof(bcdev->read_buffer_dump.data_buffer);
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_OPCODE_READ_BUFFER;

	return oem_battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

static void oplus_adsp_voocphy_set_full_para_qbg(struct battery_chg_dev *bcdev, u32 fcss_status);
static void oplus_get_props_from_adsp_by_buffer(void)
{
	struct battery_chg_dev *bcdev = g_bcdev;
	union mms_msg_data data = { 0 };
	unsigned int ffc_status = 0, temp_region = 5;
	bool fastchg_ing = false;
	static u32 pre_fcss_status = 0;
	u32 fcss_status = 0;

	if (!bcdev) {
		chg_err("bcdev is null, oplus_get_batt_argv_buffer\n");
		return;
	}
	oem_read_buffer(bcdev);

	if(bcdev->read_buffer_dump.data_buffer[14] == 0)
		return;

	if (!bcdev->batt_full_method_new) {
		if (is_common_topic_available(bcdev)) {
			oplus_mms_get_item_data(bcdev->common_topic, COMM_ITEM_TEMP_REGION,
					&data, false);
			temp_region = data.intval;
			oplus_mms_get_item_data(bcdev->common_topic, COMM_ITEM_FFC_STATUS,
					&data, false);
			ffc_status = data.intval;
		} else {
			chg_err("common topic not found\n");
		}

		fastchg_ing = oplus_vooc_get_fastchg_ing(bcdev) ||
		        oplus_get_ufcs_charging(bcdev) ||
		        oplus_get_pps_charging(bcdev);
		fcss_status = fastchg_ing | ((u8)ffc_status << 1) | ((u8)temp_region << 3);
	} else {
		fcss_status = oplus_update_batt_full_para(bcdev);
	}

	if(pre_fcss_status != fcss_status) {
		oplus_adsp_voocphy_set_full_para_qbg(bcdev, fcss_status);
		pre_fcss_status = fcss_status;
		chg_info("method %d, fcss_status is %d\n", bcdev->batt_full_method_new, fcss_status);
	}
}

static void handle_oem_read_buffer(struct battery_chg_dev *bcdev,
	struct oem_read_buffer_resp_msg *resp_msg, size_t len)
{
	u32 buf_len;

	chg_debug("correct length received: %zu expected: %lu\n", len,
		  sizeof(bcdev->read_buffer_dump));

	if (len > sizeof(bcdev->read_buffer_dump)) {
		chg_err("Incorrect length received: %zu expected: %lu\n", len,
		sizeof(bcdev->read_buffer_dump));
		return;
	}

	buf_len = resp_msg->data_size;
	if (buf_len > sizeof(bcdev->read_buffer_dump.data_buffer)) {
		chg_err("Incorrect buffer length: %u\n", buf_len);
		return;
	}

	if (buf_len == 0) {
		chg_err("Incorrect buffer length: %u\n", buf_len);
		return;
	}
	memcpy(bcdev->read_buffer_dump.data_buffer, resp_msg->data_buffer, buf_len);

	if (bcdev->read_buffer_dump.data_buffer[9] == 0) {
		oplus_chg_ic_virq_trigger(bcdev->buck_ic, OPLUS_IC_VIRQ_SUSPEND_CHECK);
	}

	bcdev->gauge_data_initialized = true;
	complete(&bcdev->oem_read_ack);
}

static int ap_set_message_id(struct battery_chg_dev *bcdev, u32 message_id, u32 value)
{
	struct oplus_ap_read_req_msg req_msg = { { 0 } };
	int rc = 0;

	req_msg.message_id = message_id;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = AP_OPCODE_READ_BUFFER;
	req_msg.value = value;

	if (atomic_read(&bcdev->state) == PMIC_GLINK_STATE_DOWN) {
		chg_err("glink state is down\n");
		return -ENOTCONN;
	}

	reinit_completion(&bcdev->ap_read_ack[AP_MESSAGE_ACK]);
	rc = pmic_glink_write(bcdev->client, &req_msg, sizeof(req_msg));
	if (!rc) {
		rc = wait_for_completion_timeout(&bcdev->ap_read_ack[AP_MESSAGE_ACK], msecs_to_jiffies(AP_READ_WAIT_TIME_MS));
		if (!rc) {
			chg_err("Error, timed out sending message\n");
			return -ETIMEDOUT;
		}
		rc = 0;
	}

	return rc;
}

static void handle_ap_read_buffer(struct battery_chg_dev *bcdev,
	struct oplus_ap_read_buffer_resp_msg *resp_msg, size_t len)
{
	u32 buf_len;

	chg_info("correct length received: %zu expected: %zu id=%u\n", len, sizeof(*bcdev->ap_read_buffer_dump), resp_msg->message_id);

	if (len > sizeof(*bcdev->ap_read_buffer_dump)) {
		chg_err("Incorrect length received: %zu expected: %zu\n", len, sizeof(*bcdev->ap_read_buffer_dump));
		memset(bcdev->ap_read_buffer_dump, 0, sizeof(*bcdev->ap_read_buffer_dump));
		return;
	}

	buf_len = resp_msg->data_size;
	if (buf_len > sizeof(bcdev->ap_read_buffer_dump->data_buffer)) {
		chg_err("Incorrect buffer length: %u\n", buf_len);
		memset(bcdev->ap_read_buffer_dump, 0, sizeof(*bcdev->ap_read_buffer_dump));
		return;
	}

	if (resp_msg->message_id == AP_MESSAGE_ACK) {
		complete(&bcdev->ap_read_ack[resp_msg->message_id]);
		return;
	}

	if (buf_len == 0) {
		chg_err("Incorrect buffer length: %u\n", buf_len);
		memset(bcdev->ap_read_buffer_dump, 0, sizeof(*bcdev->ap_read_buffer_dump));
		return;
	}
	if (resp_msg->message_id >= AP_MESSAGE_MAX_SIZE) {
		chg_err("message_id %d invalid\n", resp_msg->message_id);
		memset(bcdev->ap_read_buffer_dump, 0, sizeof(*bcdev->ap_read_buffer_dump));
		return;
	}
	memcpy(bcdev->ap_read_buffer_dump->data_buffer, resp_msg->data_buffer, buf_len);
	bcdev->ap_read_buffer_dump->data_size = buf_len;
	bcdev->ap_read_buffer_dump->message_id = resp_msg->message_id;
	complete(&bcdev->ap_read_ack[resp_msg->message_id]);
}

static int oplus_ap_write_buffer(struct battery_chg_dev *bcdev, u32 message_id, char *buf, int len)
{
	struct oplus_ap_write_req_msg req_msg = { 0 };
	int rc = 0;

	if (len > MAX_AP_PROPERTY_DATA_SIZE) {
		chg_err("message_id=%d len=%d invalid\n", message_id, len);
		return -EINVAL;
	}

	mutex_lock(&bcdev->ap_write_buffer_lock);
	req_msg.message_id = message_id;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = AP_OPCODE_WRITE_BUFFER;
	memcpy(req_msg.data_buffer, buf, len);
	req_msg.data_size = len;
	if (atomic_read(&bcdev->state) == PMIC_GLINK_STATE_DOWN) {
		chg_err("glink state is down\n");
		mutex_unlock(&bcdev->ap_write_buffer_lock);
		return -ENOTCONN;
	}

	reinit_completion(&bcdev->ap_write_ack);
	rc = pmic_glink_write(bcdev->client, &req_msg, sizeof(req_msg));
	if (!rc) {
		rc = wait_for_completion_timeout(&bcdev->ap_write_ack, msecs_to_jiffies(AP_READ_WAIT_TIME_MS));
		if (!rc) {
			chg_err("message_id=%d write error, timed out sending message\n", message_id);
			mutex_unlock(&bcdev->ap_write_buffer_lock);
			return -ETIMEDOUT;
		}
		rc = 0;
	}

	mutex_unlock(&bcdev->ap_write_buffer_lock);
	return rc;
}

static void handle_ap_write_buffer(struct battery_chg_dev *bcdev,
	struct oplus_ap_write_buffer_resp_msg *resp_msg, size_t len)
{
	if (resp_msg->ret)
		chg_err("write message_id=%d fail, ret=%d\n", resp_msg->message_id, resp_msg->ret);

	complete(&bcdev->ap_write_ack);
}

static int oplus_vooc_get_fast_chg_type(struct battery_chg_dev *bcdev);
static int bcc_battery_chg_write(struct battery_chg_dev *bcdev, void *data,
	int len)
{
	int rc;

	if ((NULL == bcdev) || (NULL == data)) {
		chg_err("bcdev is NULL");
		return -ENODEV;
	}

	if (atomic_read(&bcdev->state) == PMIC_GLINK_STATE_DOWN) {
		chg_err("glink state is down\n");
		return -ENOTCONN;
	}

	mutex_lock(&bcdev->bcc_read_buffer_lock);
	reinit_completion(&bcdev->bcc_read_ack);
	rc = pmic_glink_write(bcdev->client, data, len);
	if (!rc) {
		rc = wait_for_completion_timeout(&bcdev->bcc_read_ack,
			msecs_to_jiffies(OEM_READ_WAIT_TIME_MS));
		if (!rc) {
			chg_err("Error, timed out sending message\n");
			mutex_unlock(&bcdev->bcc_read_buffer_lock);
			return -ETIMEDOUT;
		}

		rc = 0;
	}
	mutex_unlock(&bcdev->bcc_read_buffer_lock);

	return rc;
}

static int bcc_read_buffer(struct battery_chg_dev *bcdev)
{
	struct oem_read_buffer_req_msg req_msg = { { 0 } };

	if (NULL == bcdev) {
		chg_err("bcdev is NULL");
		return -ENODEV;
	}

	req_msg.data_size = sizeof(bcdev->bcc_read_buffer_dump.data_buffer);
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = BCC_OPCODE_READ_BUFFER;

	return bcc_battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

static void handle_bcc_read_buffer(struct battery_chg_dev *bcdev,
	struct oem_read_buffer_resp_msg *resp_msg, size_t len)
{
	u32 buf_len;
	struct oplus_mms *wired_topic;

	if ((NULL == bcdev) || (NULL == resp_msg)) {
		chg_err("bcdev is NULL");
		return;
	}

	if (len > sizeof(bcdev->bcc_read_buffer_dump)) {
		chg_err("Incorrect length received: %zu expected: %u\n", len,
		(unsigned int)sizeof(bcdev->bcc_read_buffer_dump));
		return;
	}

	buf_len = resp_msg->data_size;
	if (buf_len > sizeof(bcdev->bcc_read_buffer_dump.data_buffer)) {
		chg_err("Incorrect buffer length: %u\n", buf_len);
		return;
	}

	if (buf_len == 0) {
		chg_err("Incorrect buffer length: %u\n", buf_len);
		return;
	}
	memcpy(bcdev->bcc_read_buffer_dump.data_buffer, resp_msg->data_buffer, buf_len);

	if (oplus_vooc_get_fastchg_ing(bcdev)
		&& oplus_vooc_get_fast_chg_type(bcdev) != CHARGER_SUBTYPE_FASTCHG_VOOC) {
		bcdev->bcc_read_buffer_dump.data_buffer[15] = 1;
	} else {
		bcdev->bcc_read_buffer_dump.data_buffer[15] = 0;
	}

	if (bcdev->bcc_read_buffer_dump.data_buffer[9] == 0) {
		bcdev->bcc_read_buffer_dump.data_buffer[15] = 0;
	}

	bcdev->bcc_read_buffer_dump.data_buffer[8] = DIV_ROUND_CLOSEST((int)bcdev->bcc_read_buffer_dump.data_buffer[8], 1000);

	wired_topic = oplus_mms_get_by_name("wired");
	bcdev->bcc_read_buffer_dump.data_buffer[16] = oplus_wired_get_bcc_curr_done_status(wired_topic);

	bcdev->bcc_read_buffer_dump.data_buffer[18] = 0; /* DOUBLE_SERIES_WOUND_CELLS; */

	chg_info("----dod0_1[%d], dod0_2[%d], dod0_passed_q[%d], qmax_1[%d], qmax_2[%d], qmax_passed_q[%d], "
		"voltage_cell1[%d], temperature[%d], batt_current[%d], max_current[%d], min_current[%d], voltage_cell2[%d], "
		"soc_ext_1[%d], soc_ext_2[%d], atl_last_geat_current[%d], charging_flag[%d], bcc_curr_done[%d], guage[%d], batt_type[%d]",
		bcdev->bcc_read_buffer_dump.data_buffer[0], bcdev->bcc_read_buffer_dump.data_buffer[1], bcdev->bcc_read_buffer_dump.data_buffer[2],
		bcdev->bcc_read_buffer_dump.data_buffer[3], bcdev->bcc_read_buffer_dump.data_buffer[4], bcdev->bcc_read_buffer_dump.data_buffer[5],
		bcdev->bcc_read_buffer_dump.data_buffer[6], bcdev->bcc_read_buffer_dump.data_buffer[7], bcdev->bcc_read_buffer_dump.data_buffer[8],
		bcdev->bcc_read_buffer_dump.data_buffer[9], bcdev->bcc_read_buffer_dump.data_buffer[10], bcdev->bcc_read_buffer_dump.data_buffer[11],
		bcdev->bcc_read_buffer_dump.data_buffer[12], bcdev->bcc_read_buffer_dump.data_buffer[13], bcdev->bcc_read_buffer_dump.data_buffer[14],
		bcdev->bcc_read_buffer_dump.data_buffer[15], bcdev->bcc_read_buffer_dump.data_buffer[16], bcdev->bcc_read_buffer_dump.data_buffer[17],
		bcdev->bcc_read_buffer_dump.data_buffer[18]);
	complete(&bcdev->bcc_read_ack);
}

int oplus_adsp_voocphy_get_bcc_max_current(void)
{
	struct battery_chg_dev *bcdev = g_bcdev;

	if (!bcdev) {
		chg_err("bcdev is NULL!\n");
		return -1;
	}

	return bcdev->bcc_read_buffer_dump.data_buffer[9];
}

int oplus_adsp_voocphy_get_bcc_min_current(void)
{
	struct battery_chg_dev *bcdev = g_bcdev;

	if (!bcdev) {
		chg_err("bcdev is NULL!\n");
		return -1;
	}

	return bcdev->bcc_read_buffer_dump.data_buffer[10];
}

int oplus_adsp_voocphy_get_atl_last_geat_current(void)
{
	struct battery_chg_dev *bcdev = g_bcdev;

	if (!bcdev) {
		chg_err("bcdev is NULL!\n");
		return -1;
	}

	return bcdev->bcc_read_buffer_dump.data_buffer[14];
}

static int pps_battery_chg_write(struct battery_chg_dev *bcdev, void *data,
	int len)
{
	int rc;

	if (atomic_read(&bcdev->state) == PMIC_GLINK_STATE_DOWN) {
		pr_err("glink state is down\n");
		return -ENOTCONN;
	}

	mutex_lock(&bcdev->pps_read_buffer_lock);
	reinit_completion(&bcdev->pps_read_ack);
	rc = pmic_glink_write(bcdev->client, data, len);
	if (!rc) {
		rc = wait_for_completion_timeout(&bcdev->pps_read_ack,
			msecs_to_jiffies(OEM_READ_WAIT_TIME_MS));
		if (!rc) {
			chg_err("Error, timed out sending message\n");
			mutex_unlock(&bcdev->pps_read_buffer_lock);
			return -ETIMEDOUT;
		}

		rc = 0;
	}

	mutex_unlock(&bcdev->pps_read_buffer_lock);

	return rc;
}

static int pps_read_buffer(struct battery_chg_dev *bcdev)
{
	struct oem_read_buffer_req_msg req_msg = { { 0 } };

	if (!bcdev) {
		return false;
	}

	req_msg.data_size = sizeof(bcdev->pps_read_buffer_dump.data_buffer);
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = PPS_OPCODE_READ_BUFFER;

	return pps_battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

static void handle_pps_read_buffer(struct battery_chg_dev *bcdev,
	struct oem_read_buffer_resp_msg *resp_msg, size_t len)
{
	u32 buf_len;

	if (len > sizeof(bcdev->pps_read_buffer_dump)) {
		chg_err("Incorrect length received: %zu expected: %zd\n", len,
			sizeof(bcdev->pps_read_buffer_dump));
		return;
	}

	buf_len = resp_msg->data_size;
	if (buf_len > sizeof(bcdev->pps_read_buffer_dump.data_buffer)) {
		chg_err("Incorrect buffer length: %u\n", buf_len);
		return;
	}

	if (buf_len == 0) {
		chg_err("Incorrect buffer length: %u\n", buf_len);
		return;
	}
	memcpy(bcdev->pps_read_buffer_dump.data_buffer, resp_msg->data_buffer, buf_len);
	complete(&bcdev->pps_read_ack);
}

#define BCC_SET_DEBUG_PARMS 1
#define BCC_PARMS_COUNT 19
#define BCC_PAGE_SIZE 256
#define BCC_N_DEBUG 0
#define BCC_Y_DEBUG 1
static int bcc_debug_mode  = BCC_N_DEBUG;
static char bcc_debug_buf[BCC_PAGE_SIZE] = {0};
static int oplus_get_bcc_parameters_from_adsp(struct oplus_chg_ic_dev *ic_dev, char *buf)
{
	int ret = 0;
	struct battery_chg_dev *bcdev;
	char *tmpbuf;
	int len = 0;
	int i = 0;
	int idx = 0;

	if ((ic_dev == NULL) || (buf == NULL)) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (!bcdev) {
		chg_err("!!!bcdev null, oplus_get_batt_argv_buffer\n");
		return -1;
	}

	ret = bcc_read_buffer(bcdev);

	tmpbuf = kzalloc(PAGE_SIZE + 1, GFP_KERNEL);
	if (tmpbuf == NULL)
		return -ENOMEM;
	for (i = 0; i < BCC_PARMS_COUNT - 1; i++) {
		len = snprintf(tmpbuf, BCC_PAGE_SIZE - idx,
			"%d,", bcdev->bcc_read_buffer_dump.data_buffer[i]);
		memcpy(&buf[idx], tmpbuf, len);
		idx += len;
	}
	len = snprintf(tmpbuf, BCC_PAGE_SIZE - idx,
		"%d", bcdev->bcc_read_buffer_dump.data_buffer[i]);
	memcpy(&buf[idx], tmpbuf, len);
	kfree(tmpbuf);
#ifdef BCC_SET_DEBUG_PARMS
	if (bcc_debug_mode & BCC_Y_DEBUG) {
		memcpy(&buf[0], bcc_debug_buf, BCC_PAGE_SIZE);
		chg_err("bcc_debug_buf:%s\n", bcc_debug_buf);
		return ret;
	}
#endif
	chg_info("buf:%s\n", buf);
	return ret;
}

#define BCC_DEBUG_PARAM_SIZE 8
static int oplus_set_bcc_debug_parameters(struct oplus_chg_ic_dev *ic_dev, const char *buf)
{
	int ret = 0;
#ifdef BCC_SET_DEBUG_PARMS
	char temp_buf[10] = {0};
#endif

	if ((ic_dev == NULL) || (buf == NULL)) {
		chg_err("!!!ic_dev null\n");
		return -ENODEV;
	}

#ifdef BCC_SET_DEBUG_PARMS
	if (strlen(buf) <= BCC_PAGE_SIZE) {
		if (strncpy(temp_buf, buf, 7)) {
			chg_info("temp_buf:%s\n", temp_buf);
		}
		if (!strncmp(temp_buf, "Y_DEBUG", 7)) {
			bcc_debug_mode = BCC_Y_DEBUG;
			chg_info("BCC_Y_DEBUG:%d\n", bcc_debug_mode);
		} else {
			bcc_debug_mode = BCC_N_DEBUG;
			chg_info("BCC_N_DEBUG:%d\n", bcc_debug_mode);
		}
		strncpy(bcc_debug_buf, buf + BCC_DEBUG_PARAM_SIZE, BCC_PAGE_SIZE);
		chg_info("bcc_debug_buf:%s, temp_buf:%s\n", bcc_debug_buf, temp_buf);
		return ret;
	}
#endif

	chg_info("buf:%s\n", buf);
	return ret;
}
#endif

static int ufcs_battery_chg_write(struct battery_chg_dev *bcdev, void *data,
	int len)
{
	int rc;

	if (atomic_read(&bcdev->state) == PMIC_GLINK_STATE_DOWN) {
		pr_err("glink state is down\n");
		return -ENOTCONN;
	}

	mutex_lock(&bcdev->ufcs_read_buffer_lock);
	reinit_completion(&bcdev->ufcs_read_ack);
	rc = pmic_glink_write(bcdev->client, data, len);
	if (!rc) {
		rc = wait_for_completion_timeout(&bcdev->ufcs_read_ack,
			msecs_to_jiffies(AP_UFCS_WAIT_TIME_MS));
		if (!rc) {
			chg_err("Error, timed out sending message\n");
			mutex_unlock(&bcdev->ufcs_read_buffer_lock);
			return -ETIMEDOUT;
		}

		rc = 0;
	}

	mutex_unlock(&bcdev->ufcs_read_buffer_lock);

	return rc;
}

static int ufcs_read_buffer(struct battery_chg_dev *bcdev, enum ufcs_read_msg_id msg_id)
{
	struct oplus_ap_read_ufcs_req_msg req_msg = { { 0 } };

	if (!bcdev) {
		return false;
	}

	req_msg.data_size = sizeof(bcdev->ufcs_read_buffer_dump.data_buffer);
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = AP_OPCODE_UFCS_BUFFER;
	req_msg.msg_id = msg_id;

	return ufcs_battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

static void handle_ufcs_read_buffer(struct battery_chg_dev *bcdev,
	struct oplus_ap_read_ufcs_resp_msg *resp_msg, size_t len)
{
	u32 buf_len;

	chg_info("got the ufcs msg_id=%d\n", resp_msg->msg_id);

	if (len > sizeof(bcdev->ufcs_read_buffer_dump)) {
		chg_err("Incorrect length received: %zu expected: %zd\n", len,
			sizeof(bcdev->ufcs_read_buffer_dump));
		complete(&bcdev->ufcs_read_ack);
		return;
	}

	buf_len = resp_msg->data_size;
	if (buf_len > sizeof(bcdev->ufcs_read_buffer_dump.data_buffer)) {
		chg_err("Incorrect buffer length: %u\n", buf_len);
		complete(&bcdev->ufcs_read_ack);
		return;
	}

	if (buf_len == 0) {
		chg_err("Incorrect buffer length: %u\n", buf_len);
		return;
	}

	switch (resp_msg->msg_id) {
	case UFCS_PDO_INFO:
		memcpy(bcdev->ufcs_read_buffer_dump.data_buffer, resp_msg->data_buffer, buf_len);
		break;
	case UFCS_VDM_PWR_INFO:
		memcpy(bcdev->ufcs_read_buffer_dump.data_buffer, resp_msg->data_buffer, buf_len);
		break;
	case UFCS_VDM_EMARK_INFO:
		bcdev->ufcs_read_buffer_dump.data_buffer[0] = resp_msg->data_buffer[0];
		break;
	case UFCS_ADAPTER_VERIFY:
		bcdev->adapter_verify_auth = !!resp_msg->data_buffer[0];
		chg_info("bcdev->adapter_verify_auth=%d\n", bcdev->adapter_verify_auth);
		break;
	default:
		chg_err("got an undefined ufcs message id\n");
		break;
	}
	complete(&bcdev->ufcs_read_ack);
}

static int battery_chg_fw_write(struct battery_chg_dev *bcdev, void *data,
				int len)
{
	int rc;

	if (atomic_read(&bcdev->state) == PMIC_GLINK_STATE_DOWN) {
		pr_debug("glink state is down\n");
		return -ENOTCONN;
	}

	reinit_completion(&bcdev->fw_buf_ack);
	rc = pmic_glink_write(bcdev->client, data, len);
	if (!rc) {
		rc = wait_for_completion_timeout(&bcdev->fw_buf_ack,
					msecs_to_jiffies(WLS_FW_WAIT_TIME_MS));
		if (!rc) {
			chg_err("Error, timed out sending message\n");
			return -ETIMEDOUT;
		}

		rc = 0;
	}

	return rc;
}

static int battery_chg_write(struct battery_chg_dev *bcdev, void *data,
				int len)
{
	int rc;

	/*
	 * When the subsystem goes down, it's better to return the last
	 * known values until it comes back up. Hence, return 0 so that
	 * pmic_glink_write() is not attempted until pmic glink is up.
	 */
	if (atomic_read(&bcdev->state) == PMIC_GLINK_STATE_DOWN) {
		pr_debug("glink state is down\n");
		return 0;
	}

	if (bcdev->debug_battery_detected && bcdev->block_tx)
		return 0;

	mutex_lock(&bcdev->rw_lock);
	reinit_completion(&bcdev->ack);
	bcdev->error_prop = false;
	rc = pmic_glink_write(bcdev->client, data, len);
	if (!rc) {
		rc = wait_for_completion_timeout(&bcdev->ack,
					msecs_to_jiffies(BC_WAIT_TIME_MS));
		if (!rc) {
			chg_err("Error, timed out sending message\n");
			mutex_unlock(&bcdev->rw_lock);
			return -ETIMEDOUT;
		}

		rc = 0;

		if (bcdev->error_prop) {
			bcdev->error_prop = false;
			rc = -ENODATA;
		}
	}
	mutex_unlock(&bcdev->rw_lock);

	return rc;
}

static int write_property_id(struct battery_chg_dev *bcdev,
			struct psy_state *pst, u32 prop_id, u32 val)
{
	struct battery_charger_req_msg req_msg = { { 0 } };

	req_msg.property_id = prop_id;
	req_msg.battery_id = 0;
	req_msg.value = val;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_set;

	pr_debug("psy: %s prop_id: %u val: %u\n", pst->psy->desc->name,
		req_msg.property_id, val);

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

static int read_property_id(struct battery_chg_dev *bcdev,
			struct psy_state *pst, u32 prop_id)
{
	struct battery_charger_req_msg req_msg = { { 0 } };

	req_msg.property_id = prop_id;
	req_msg.battery_id = 0;
	req_msg.value = 0;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_get;

	pr_debug("psy: %s prop_id: %u\n", pst->psy->desc->name,
		req_msg.property_id);

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

static int get_property_id(struct psy_state *pst,
			enum power_supply_property prop)
{
	u32 i;

	for (i = 0; i < pst->prop_count; i++)
		if (pst->map[i] == prop)
			return i;

	chg_err("No property id for property %d in psy %s\n", prop,
		pst->psy->desc->name);

	return -ENOENT;
}

static void battery_chg_notify_enable(struct battery_chg_dev *bcdev)
{
	struct battery_charger_set_notify_msg req_msg = { { 0 } };
	int rc;

	/* Send request to enable notification */
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_NOTIFY;
	req_msg.hdr.opcode = BC_SET_NOTIFY_REQ;

	rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0)
		chg_err("Failed to enable notification rc=%d\n", rc);
}

static void battery_chg_subsys_up_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
					struct battery_chg_dev, subsys_up_work);

	battery_chg_notify_enable(bcdev);
}

#ifdef OPLUS_FEATURE_CHG_BASIC
static void oplus_typec_disable(void)
{
	int rc = 0;
	struct battery_chg_dev *bcdev = g_bcdev;
	struct psy_state *pst = NULL;

	if (!bcdev) {
		chg_err("bcdev not ready\n");
		return;
	}

	pst = &bcdev->psy_list[PSY_TYPE_USB];

	/* set disable typec mode */
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_TYPEC_MODE, QCOM_TYPEC_PORT_ROLE_DRP);
	else
		rc = write_property_id(bcdev, pst, USB_TYPEC_MODE, QCOM_TYPEC_PORT_ROLE_DRP);

	if (rc < 0) {
		chg_info("Couldn't write 0x2b44[3] rc=%d\n", rc);
	}
}

static bool is_common_topic_available(struct battery_chg_dev *bcdev)
{
	if (!bcdev->common_topic)
		bcdev->common_topic = oplus_mms_get_by_name("common");

	return !!bcdev->common_topic;
}

static void oplus_adsp_voocphy_set_full_para_qbg(struct battery_chg_dev *bcdev, u32 fcss_status)
{
	int rc = 0;
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	rc = write_property_id(bcdev, pst, BATT_BAT_FULL_CURR_SET, fcss_status);
	if (rc) {
		chg_err("set current level fail, rc=%d\n", rc);
		return;
	}
}

void oplus_chg_set_curr_level_to_voocphy(struct battery_chg_dev *bcdev)
{
	int rc = 0;
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	union mms_msg_data data = {0};
	int cool_down = 0;

	if (is_common_topic_available(bcdev)) {
		oplus_mms_get_item_data(bcdev->common_topic,
					COMM_ITEM_COOL_DOWN, &data, false);
	} else {
		chg_err("common topic not found\n");
	}
	cool_down = data.intval;

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_SET_COOL_DOWN, cool_down);
	else
		rc = write_property_id(bcdev, pst, BATT_SET_COOL_DOWN, cool_down);
	if (rc) {
		chg_err("set curr level fail, rc=%d\n", rc);
		return;
	}

	chg_info("ap set curr level[%d] to voocphy\n", cool_down);
}

static void oplus_adsp_voocphy_cancle_err_check(struct battery_chg_dev *bcdev)
{
	if (bcdev->voocphy_err_check == true) {
		cancel_delayed_work_sync(&bcdev->voocphy_err_work);
	}
	bcdev->voocphy_err_check = false;
}

static bool is_vooc_topic_available(struct battery_chg_dev *bcdev)
{
	if (!bcdev->vooc_topic)
		bcdev->vooc_topic = oplus_mms_get_by_name("vooc");

	return !!bcdev->vooc_topic;
}

static bool is_cpa_topic_available(struct battery_chg_dev *bcdev)
{
	if (!bcdev->cpa_topic)
		bcdev->cpa_topic = oplus_mms_get_by_name("cpa");

	return !!bcdev->cpa_topic;
}

static int oplus_chg_get_voocphy_support(struct battery_chg_dev *bcdev)
{
	int voocphy_support = 0;

	if (is_vooc_topic_available(bcdev))
		voocphy_support = oplus_vooc_get_voocphy_support(bcdev->vooc_topic);
	else
		chg_err("vooc topic not found\n");

	return voocphy_support;
}

static bool oplus_vooc_get_fastchg_ing(struct battery_chg_dev *bcdev)
{
	bool fastchg_status;
	union mms_msg_data data = { 0 };

	if (!is_vooc_topic_available(bcdev)) {
		chg_info("vooc_topic is null\n");
		return 0;
	}

	oplus_mms_get_item_data(bcdev->vooc_topic, VOOC_ITEM_VOOC_CHARGING,
				&data, true);
	fastchg_status = !!data.intval;
	chg_debug("get fastchg status = %d\n", fastchg_status);

	return fastchg_status;
}

static int oplus_vooc_get_fast_chg_type(struct battery_chg_dev *bcdev)
{
	int svooc_type = 0;
	union mms_msg_data data = { 0 };

	if (!is_vooc_topic_available(bcdev)) {
		chg_info("vooc_topic is null\n");
		return 0;
	}

	oplus_mms_get_item_data(bcdev->vooc_topic,
				VOOC_ITEM_GET_BCC_SVOOC_TYPE, &data, true);
	svooc_type = data.intval;
	chg_debug("get svooc type = %d\n", svooc_type);

	return svooc_type;
}

static int oplus_cpa_get_protocol_allow(struct battery_chg_dev *bcdev)
{
	union mms_msg_data data = { 0 };

	if (!is_cpa_topic_available(bcdev)) {
		chg_info("cpa_topic is null\n");
		return CHG_PROTOCOL_INVALID;
	}

	oplus_mms_get_item_data(bcdev->cpa_topic, CPA_ITEM_ALLOW, &data, true);
	chg_debug("get protocol allow = %d\n", data.intval);

	return data.intval;
}

static bool is_gauge_topic_available(struct battery_chg_dev *bcdev)
{
	if (!bcdev->gauge_topic)
		bcdev->gauge_topic = oplus_mms_get_by_name("gauge");

	return !!bcdev->gauge_topic;
}

static int oplus_chg_get_battery_soc(struct battery_chg_dev *bcdev)
{
	union mms_msg_data data = {0};

	if (is_gauge_topic_available(bcdev)) {
		oplus_mms_get_item_data(bcdev->gauge_topic,
					GAUGE_ITEM_SOC, &data, false);
	} else {
		chg_err("gauge topic not found\n");
		data.intval = 50;
	}

	return data.intval;
}

#define  VOLTAGE_2000MV  2000
#define  COUNT_SIX      6
#define  COUNT_THR      3
#define  COUNT_TEN      10
#define  CHECK_CURRENT_LOW       300
#define  CHECK_CURRENT_HIGH      900
#define  VBUS_VOLT_LOW      6000
static void oplus_recheck_input_current_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, recheck_input_current_work.work);
	bool fastchg_ing = oplus_vooc_get_fastchg_ing(bcdev);
	int fast_chg_type = oplus_vooc_get_fast_chg_type(bcdev);
	int chg_vol = 0;
	int ibus_curr = 0;
	static int count = 0;
	static int err_cnt = 0;

	chg_err("reset input current count:%d\n", count);
	chg_vol = oplus_wired_get_vbus();
	if (!fastchg_ing)
		ibus_curr = oplus_wired_get_ibus();

	if (chg_vol > VOLTAGE_2000MV) {
		count++;

		if ((count > COUNT_THR) && (ibus_curr > CHECK_CURRENT_LOW) &&
		    (ibus_curr < CHECK_CURRENT_HIGH))
			err_cnt++;
		else
			err_cnt = 0;

		if (count > COUNT_TEN) {
			chg_err("reset input current err_cnt: %d,chg_vol:%d,"
				"fastchg_ing:%d,ibus_curr:%d,fast_chg_type:%d\n",
				err_cnt, chg_vol, fastchg_ing, ibus_curr,
				fast_chg_type);
			if (bcdev->charger_type != POWER_SUPPLY_TYPE_USB_DCP) {
				chg_err("reset input current charger_type: %d\n",
					bcdev->charger_type);
				count = 0;
				return;
			}
			if (err_cnt > COUNT_THR) {
				chg_err("reset icl setting!\n");
				oplus_chg_ic_virq_trigger(bcdev->buck_ic,
							  OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);
			}
			if (fastchg_ing && (fast_chg_type != BCC_TYPE_IS_VOOC)) {
				chg_err("reset voocphy setting, chg_vol:%d\n", chg_vol);
				if ((chg_vol < VBUS_VOLT_LOW))
					oplus_adsp_voocphy_reset_status();
			}
			count = 0;
		} else {
			schedule_delayed_work(&bcdev->recheck_input_current_work, msecs_to_jiffies(2000));
		}
	} else {
		count = 0;
	}
}

static void oplus_unsuspend_usb_work(struct work_struct *work)
{
	oplus_chg_suspend_charger(false, DEF_VOTER);
}

static int oplus_get_max_current_from_fixed_pdo(struct battery_chg_dev *bcdev, int volt)
{
	int i = 0;
	if (bcdev->pdo[0].pdo_data == 0) {
		chg_err("get pdo info error\n");
		return -EINVAL;
	}

	if (!oplus_chg_get_common_charge_icl_support_flags())
		return -EINVAL;

	for (i = 0; i < (PPS_PDO_MAX - 1); i++) {
		if (bcdev->pdo[i].pdo_type != USBPD_PDMSG_PDOTYPE_FIXED_SUPPLY)
			continue;

		if (volt <= PD_PDO_VOL(bcdev->pdo[i].voltage_50mv)) {
			chg_info("SourceCap[%d]: %08X, FixedSupply PDO V=%d mV, I=%d mA,"
				"UsbCommCapable=%d, USBSuspendSupported:%d\n", i,
				bcdev->pdo[i].pdo_data, PD_PDO_VOL(bcdev->pdo[i].voltage_50mv),
				PD_PDO_CURR_MAX(bcdev->pdo[i].max_current_10ma),
				bcdev->pdo[i].usb_comm_capable, bcdev->pdo[i].usb_suspend_supported);
			return PD_PDO_CURR_MAX(bcdev->pdo[i].max_current_10ma);
		}
	}
	return -EINVAL;
}

#define SUSPEND_RECOVERY_DELAY_MS 2000
static void oplus_sourcecap_suspend_recovery_work(struct work_struct *work)
{
	chg_info("sourcecap suspend recovery, unsuspend\n");
	oplus_chg_suspend_charger(false, PD_PDO_ICL_VOTER);
}

static void oplus_update_pd_svooc_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, update_pd_svooc_work.work);
	int rc = 0;

	if (bcdev->soccp_support) {
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_GET_PD_SVOOC);
		if (rc < 0) {
			chg_err("read OPLUS_GET_PD_SVOOC fail\n");
			return;
		}
		bcdev->pd_svooc = bcdev->oplus_psy.prop[OPLUS_GET_PD_SVOOC];
	}
}

#define OPLUS_BC_ENABLE_VIRQ_TRIG_MAX_RETRY		15
static void oplus_wired_otg_enable_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, wired_otg_enable_work);
	struct psy_state *pst = NULL;
	int retry_count = 0;
	int ret = 0;

	pst = &bcdev->psy_list[PSY_TYPE_USB];

	while (retry_count < OPLUS_BC_ENABLE_VIRQ_TRIG_MAX_RETRY) {
		if (bcdev->otg_online == false) {
			break;
		}
		ret = oplus_chg_ic_virq_trigger(bcdev->buck_ic, OPLUS_IC_VIRQ_OTG_ENABLE);
		if (ret != -EAGAIN) {
			break;
		} else {
			retry_count++;
			msleep(100);
		}
	}

	if (pst && is_usb_psy_available(bcdev))
		power_supply_changed(pst->psy);
}

static void oplus_adsp_set_iterm_check_status(struct battery_chg_dev *bcdev, bool iterm_status)
{
	int rc = 0;
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_BATT_ITERM_CHECK_STAT, iterm_status);
	else
		rc = write_property_id(bcdev, pst, BATT_ITERM_CHECK_STAT, iterm_status);
	if (rc)
		chg_err("set current level fail, rc=%d\n", rc);
}

static void oplus_iterm_timeout_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work, struct battery_chg_dev, iterm_timeout_work.work);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	static int upload_count = 0, pre_upload_time = 0;
	int curr_time, rc = 0;
	const int extra_num = 16;
	bool chg_en = false;
	char *buf;
	size_t index = 0;
	int chg_type = 0, ui_soc = 0, soc = 0, vbat = 0, ibat = 0;
	int tbat = 0, fcc = 0, cc = 0, soh = 0;
	union mms_msg_data data = {0};
#define ERR_MSG_BUF PAGE_SIZE

	curr_time = local_clock() / TRACK_LOCAL_T_NS_TO_S_THD;
	if (curr_time - pre_upload_time > TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD)
		upload_count = 0;

	if (upload_count >= TRACK_UPLOAD_COUNT_MAX)
		goto done;

	pre_upload_time = local_clock() / TRACK_LOCAL_T_NS_TO_S_THD;

	if (is_common_topic_available(bcdev)) {
		oplus_mms_get_item_data(bcdev->common_topic, COMM_ITEM_UI_SOC, &data, false);
		ui_soc = data.intval;
	}
	if (is_gauge_topic_available(bcdev)) {
		oplus_mms_get_item_data(bcdev->gauge_topic, GAUGE_ITEM_SOC, &data, false);
		soc = data.intval;
		oplus_mms_get_item_data(bcdev->gauge_topic, GAUGE_ITEM_VOL_MAX, &data, false);
		vbat = data.intval;
		oplus_mms_get_item_data(bcdev->gauge_topic, GAUGE_ITEM_CURR, &data, true);
		ibat = data.intval;
		oplus_mms_get_item_data(bcdev->gauge_topic, GAUGE_ITEM_TEMP, &data, false);
		tbat = data.intval;
		oplus_mms_get_item_data(bcdev->gauge_topic, GAUGE_ITEM_FCC, &data, false);
		fcc = data.intval;
		oplus_mms_get_item_data(bcdev->gauge_topic, GAUGE_ITEM_CC, &data, false);
		cc = data.intval;
		oplus_mms_get_item_data(bcdev->gauge_topic, GAUGE_ITEM_SOH, &data, false);
		soh = data.intval;
	}
	oplus_chg_8350_output_is_suspend(bcdev->buck_ic, &chg_en);
	oplus_chg_8350_get_charger_type(bcdev->buck_ic, &chg_type);
	oem_read_buffer(bcdev);
	buf = kzalloc(ERR_MSG_BUF, GFP_KERNEL);
	if (buf == NULL)
		goto done;
	index += scnprintf(buf + index, ERR_MSG_BUF - index,
		"ui_soc=%d, soc=%d, vbat=%d, ibat=%d, tbat=%d, fcc=%d, cc=%d, soh=%d, "
		"chg_en=%d, suspend=%d, pd_svooc=%d, subtype=0x%02x, "
		"oplus_UsbCommCapable=%d, oplus_pd_svooc=%d, typec_mode=%d, cid_status=0x%02x, usb_in_status=%d, "
		"0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x",
		ui_soc, soc, vbat, ibat, tbat, fcc, cc, soh,
		chg_en, bcdev->read_buffer_dump.data_buffer[9], bcdev->read_buffer_dump.data_buffer[11], chg_type,
		bcdev->read_buffer_dump.data_buffer[10], bcdev->read_buffer_dump.data_buffer[11],
		bcdev->read_buffer_dump.data_buffer[12], bcdev->cid_status, bcdev->usb_in_status,
		bcdev->read_buffer_dump.data_buffer[extra_num - 1], bcdev->read_buffer_dump.data_buffer[extra_num],
		bcdev->read_buffer_dump.data_buffer[extra_num + 27], bcdev->read_buffer_dump.data_buffer[extra_num + 28],
		bcdev->read_buffer_dump.data_buffer[extra_num + 105], bcdev->read_buffer_dump.data_buffer[extra_num + 106],
		bcdev->read_buffer_dump.data_buffer[extra_num + 107], bcdev->read_buffer_dump.data_buffer[extra_num + 108],
		bcdev->read_buffer_dump.data_buffer[extra_num + 109], bcdev->read_buffer_dump.data_buffer[extra_num + 110]);

	if (index > 0 && index < ERR_MSG_BUF)
		buf[index] = 0;
	oplus_chg_ic_creat_err_msg(bcdev->buck_ic, OPLUS_IC_ERR_BUCK_BOOST, 0, "$$err_scene@@iterm_full"
		"$$err_reason@@pmic_leakage$$reg_info@@%s", buf);
	oplus_chg_ic_virq_trigger(bcdev->buck_ic, OPLUS_IC_VIRQ_ERR);
	upload_count++;
	kfree(buf);

done:
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_BATT_ITERM_TIMEOUT, true);
	else
		rc = write_property_id(bcdev, pst, BATT_ITERM_TIMEOUT, true);
	if (rc) {
		chg_err("set current level fail, rc=%d\n", rc);
		return;
	}
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static struct pm_qos_request pm_qos_req;
#endif
static void oplus_mms_wired_qos_request(int new_value)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
	static int last_value = -1;
	int value = 0;

	if (new_value == PM_QOS_DEFAULT_VALUE)
		value = PM_QOS_DEFAULT_VALUE;
	if (!cpu_latency_qos_request_active(&pm_qos_req))
		cpu_latency_qos_add_request(&pm_qos_req, value);
	else
		cpu_latency_qos_update_request(&pm_qos_req, value);

	if (last_value != new_value) {
		last_value = new_value;
		if (new_value ==  PM_QOS_DEFAULT_VALUE)
			chg_info("oplus_mms_wired_qos_request PM_QOS_DEFAULT_VALUE \n");
		else
			chg_info("oplus_mms_wired_qos_request value = %d \n", new_value);
	}
#endif
}

static void oplus_request_qos_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, request_qos_work.work);

	bcdev->qos_status = true;
	oplus_mms_wired_qos_request(1200);
}

static void oplus_ufcs_reset_work(struct work_struct *work)
{
	int ufcs_notify_val = UFCS_NOTIFY_UFCS_RESET_NOTIFY;

	plat_ufcs_send_state(UFCS_NOTIFY_EXIT_COMM, (void *)&ufcs_notify_val);
}

static void oplus_release_qos_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, release_qos_work.work);

	oplus_mms_wired_qos_request(PM_QOS_DEFAULT_VALUE);
	bcdev->qos_status = false;
}

static int oplus_chg_get_source_pdo(struct oplus_chg_ic_dev *ic_dev, u32 *data, int *num)
{
	struct battery_chg_dev *chip;
	int pdo_index;

	if (ic_dev  == NULL) {
		chg_err("ic_dev is NULL");
		return -EINVAL;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*num = PPS_PDO_MAX;
	for (pdo_index = 0; pdo_index < PPS_PDO_MAX; pdo_index++)
		data[pdo_index] = chip->pdo[pdo_index].pdo_data;

	return 0;
}

static void oplus_sourcecap_done_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, sourcecap_done_work.work);
	int max_pdo_current = 0;
	int rc = 0;

	rc = oplus_get_pps_info_from_adsp(bcdev->buck_ic, (u32 *)bcdev->pdo, PPS_PDO_MAX);
	if (rc < 0) {
		chg_err("get pdo info error\n");
		return;
	}
	oplus_chg_ic_virq_trigger(bcdev->buck_ic, OPLUS_IC_VIRQ_POWER_CHANGED);
	/*set default input current from pdo*/
	max_pdo_current = oplus_get_max_current_from_fixed_pdo(bcdev, OPLUS_PD_5V);
	if (max_pdo_current >= 0)
		oplus_chg_set_icl_by_vote(max_pdo_current, PD_PDO_ICL_VOTER);
}

static void oplus_pdo_update_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, pdo_update_work.work);
	int rc = 0;

	rc = oplus_get_pps_info_from_adsp(bcdev->buck_ic, (u32 *)bcdev->pdo, PPS_PDO_MAX);
}

static void oplus_adsp_voocphy_status_func(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, adsp_voocphy_status_work.work);
	struct psy_state *pst = NULL;
	struct psy_state *pst_batt = NULL;
	int rc;
	int intval = 0;

	pst = &bcdev->psy_list[PSY_TYPE_USB];
	pst_batt = &bcdev->psy_list[PSY_TYPE_BATTERY];
	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_VOOC_STATUS);
	else
		rc = read_property_id(bcdev, pst, USB_VOOCPHY_STATUS);
	if (rc < 0) {
		chg_err("read adsp voocphy status fail\n");
		return;
	}
	if (bcdev->soccp_support)
		intval = bcdev->oplus_psy.prop[OPLUS_USB_VOOC_STATUS];
	else
		intval = pst->prop[USB_VOOCPHY_STATUS];

	if ((intval & 0xFF) == ADSP_VPHY_FAST_NOTIFY_ERR_COMMU
		|| (intval & 0xFF) == ADSP_VPHY_FAST_NOTIFY_COMMU_TIME_OUT
		|| (intval & 0xFF) == ADSP_VPHY_FAST_NOTIFY_COMMU_CLK_ERR) {
		/* unplug svooc but usb_in_status (in oplus_plugin_irq_work) was 1 sometimes */
		schedule_delayed_work(&bcdev->voocphy_enable_check_work, round_jiffies_relative(msecs_to_jiffies(5000)));
		schedule_delayed_work(&bcdev->plugin_irq_work, 0);
		schedule_delayed_work(&bcdev->recheck_input_current_work, msecs_to_jiffies(3000));
	}
	if ((intval & 0xFF) == ADSP_VPHY_FAST_NOTIFY_BATT_TEMP_OVER) {
		/* fast charge warm switch to normal charge,input current limmit to 500mA,rerun ICL setting */
		schedule_delayed_work(&bcdev->recheck_input_current_work, msecs_to_jiffies(3000));
	}

	oplus_adsp_voocphy_fastchg_event_handle(intval);
/*	if ((intval & 0xFF) == ADSP_VPHY_FAST_NOTIFY_PRESENT
		|| (intval & 0xFF) == ADSP_VPHY_FAST_NOTIFY_ONGOING) {
		oplus_chg_set_curr_level_to_voocphy(bcdev);
	}*/

	if ((intval & 0xFF) != ADSP_VPHY_FAST_NOTIFY_PRESENT)
		oplus_adsp_voocphy_cancle_err_check(bcdev);
}

#define DISCONNECT			0
#define STANDARD_TYPEC_DEV_CONNECT	BIT(0)
#define OTG_DEV_CONNECT			BIT(1)
int oplus_get_otg_online_status_with_cid_scheme(struct battery_chg_dev *bcdev)
{
	int rc = 0;
	int cid_status = 0;
	int online = 0;
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_CID_STATUS);
	else
		rc = read_property_id(bcdev, pst, USB_CID_STATUS);
	if (rc < 0) {
		chg_err("!!!read cid_status fail\n");
		return 0;
	}

	if (bcdev->soccp_support)
		cid_status = bcdev->oplus_psy.prop[OPLUS_USB_CID_STATUS];
	else
		cid_status = pst->prop[USB_CID_STATUS];
	bcdev->cid_status = cid_status;
	online = (cid_status == 1) ? STANDARD_TYPEC_DEV_CONNECT : DISCONNECT;
	chg_info("cid_status = %d, online = %d\n", cid_status, online);

	return online;
}

static void oplus_ccdetect_enable(struct battery_chg_dev *bcdev)
{
	int rc = 0;
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_TYPEC_MODE);
	else
		rc = read_property_id(bcdev, pst, USB_TYPEC_MODE);
	if (rc < 0) {
		chg_err("Couldn't read USB_TYPEC_MODE, rc=%d\n", rc);
		return;
	}

	/* set DRP mode */
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_TYPEC_MODE, QCOM_TYPEC_PORT_ROLE_DRP);
	else
		rc = write_property_id(bcdev, pst, USB_TYPEC_MODE, QCOM_TYPEC_PORT_ROLE_DRP);
	if (rc < 0) {
		chg_err("Couldn't clear 0x2b44[0] rc=%d\n", rc);
	}

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_TYPEC_MODE);
	else
		rc = read_property_id(bcdev, pst, USB_TYPEC_MODE);
	if (rc < 0) {
		chg_err("Couldn't read USB_TYPEC_MODE, rc=%d\n", rc);
		return;
	} else {
		if (bcdev->soccp_support)
			chg_err("reg0x2b44[0x%x], bit[2:0]=0(DRP)\n", bcdev->oplus_psy.prop[OPLUS_USB_TYPEC_MODE]);
		else
			chg_err("reg0x2b44[0x%x], bit[2:0]=0(DRP)\n", pst->prop[USB_TYPEC_MODE]);
	}
}

static int oplus_otg_ap_enable(struct battery_chg_dev *bcdev, bool enable)
{
	int rc = 0;
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_OTG_AP_ENABLE, enable);
	else
		rc = write_property_id(bcdev, pst, USB_OTG_AP_ENABLE, enable);
	if (rc) {
		chg_err("oplus_otg_ap_enable fail, rc=%d\n", rc);
	} else {
		chg_err("oplus_otg_ap_enable, rc=%d\n", rc);
	}
	oplus_get_otg_online_status_with_cid_scheme(bcdev);
	if (bcdev->cid_status != 0) {
		chg_err("Oplus_otg_ap_enable,flag bcdev->cid_status != 0\n");
		oplus_ccdetect_enable(bcdev);
	}

	return rc;
}

static void oplus_otg_init_status_func(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, otg_init_work.work);

	if (bcdev->otg_online) {
		bcdev->otg_online = false;
		chg_err("oplus_otg_init_status_func disable otg");
		oplus_chg_ic_virq_trigger(bcdev->buck_ic, OPLUS_IC_VIRQ_OTG_ENABLE);
	}
	oplus_otg_ap_enable(bcdev, true);
}

static void oplus_chg_clear_vdm_info(struct battery_chg_dev *bcdev)
{
	if (!bcdev) {
		chg_err("bcdev is NULL");
		return;
	}

	chg_info("clear vdm_info\n");
	mutex_lock(&bcdev->vdm_info_lock);
	bcdev->vdm_info_cnt = 0;
	memset(bcdev->vdm_info_data, 0, sizeof(bcdev->vdm_info_data));
	mutex_unlock(&bcdev->vdm_info_lock);
}

static void oplus_cid_status_change_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, cid_status_change_work.work);
	struct psy_state *pst = NULL;
	int cid_status = 0;
	int rc = 0;

	pst = &bcdev->psy_list[PSY_TYPE_USB];
	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_CID_STATUS);
	else
		rc = read_property_id(bcdev, pst, USB_CID_STATUS);
	if (rc < 0) {
		chg_err("!!!%s, read cid_status fail\n", __func__);
		return;
	}

	if (bcdev->soccp_support)
		cid_status = bcdev->oplus_psy.prop[OPLUS_USB_CID_STATUS];
	else
		cid_status = pst->prop[USB_CID_STATUS];
	bcdev->cid_status = cid_status;
	if (bcdev->soccp_support && bcdev->qos_status && bcdev->cid_status == 0) {
		cancel_delayed_work(&bcdev->request_qos_work);
		schedule_delayed_work(&bcdev->release_qos_work, 0);
	}
	chg_info("cid_status[%d]\n", cid_status);
	if (cid_status == 0)
		oplus_chg_clear_vdm_info(bcdev);
	if (pst && is_usb_psy_available(bcdev))
		power_supply_changed(pst->psy);
}

static int oplus_oem_misc_ctl(void)
{
	int rc = 0;
	struct battery_chg_dev *bcdev = g_bcdev;
	struct psy_state *pst = NULL;

	if (!bcdev) {
		chg_err("bcdev is NULL!\n");
		return -1;
	}
	pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_OEM_MISC_CTL, bcdev->oem_misc_ctl_data);
	else
		rc = write_property_id(bcdev, pst, USB_OEM_MISC_CTL, bcdev->oem_misc_ctl_data);
	if (rc)
		chg_err("oplus_oem_misc_ctl fail, rc=%d\n", rc);
	else
		chg_err("oem_misc_ctl_data: 0x%x\n", bcdev->oem_misc_ctl_data);

	return rc;
}

static void oplus_oem_lcm_en_check_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = g_bcdev;
	int enable, vph_track_high;
	static int last_enable = -1, last_vph_track_high = -1;

	if (!bcdev) {
		chg_err("bcdev is NULL\n");
		return;
	}

	enable = (bcdev->oem_usb_online ? 0 : 1);
	/* vph_track_high = (chip->batt_full ? 1 : 0); */
	vph_track_high = 0; /* TODO */

	if (bcdev->oem_usb_online && (enable == last_enable) && (last_vph_track_high == vph_track_high)) {
		schedule_delayed_work(&bcdev->oem_lcm_en_check_work, round_jiffies_relative(msecs_to_jiffies(5000)));
		return;
	}

	bcdev->oem_misc_ctl_data = 0;
	bcdev->oem_misc_ctl_data |= OEM_MISC_CTL_DATA_PAIR(OEM_MISC_CTL_CMD_LCM_EN, enable);
	bcdev->oem_misc_ctl_data |= OEM_MISC_CTL_DATA_PAIR(OEM_MISC_CTL_CMD_NCM_AUTO_MODE, enable);
	bcdev->oem_misc_ctl_data |= OEM_MISC_CTL_DATA_PAIR(OEM_MISC_CTL_CMD_VPH_TRACK_HIGH, vph_track_high);
	oplus_oem_misc_ctl();
	last_enable = enable;
	last_vph_track_high = vph_track_high;

	if (bcdev->oem_usb_online) {
		schedule_delayed_work(&bcdev->oem_lcm_en_check_work, round_jiffies_relative(msecs_to_jiffies(5000)));
	}
}

static int oplus_otg_boost_en_gpio_init(struct battery_chg_dev *bcdev)
{
	if (!bcdev) {
		chg_err("bcdev not ready\n");
		return -EINVAL;
	}

	bcdev->oplus_custom_gpio.otg_boost_en_pinctrl = devm_pinctrl_get(bcdev->dev);
	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.otg_boost_en_pinctrl)) {
		chg_err("get otg_boost_en_pinctrl fail\n");
		return -EINVAL;
	}
	bcdev->oplus_custom_gpio.otg_boost_en_active =
		pinctrl_lookup_state(bcdev->oplus_custom_gpio.otg_boost_en_pinctrl, "otg_booster_en_active");
	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.otg_boost_en_active)) {
		chg_err("get otg_boost_en_active\n");
		return -EINVAL;
	}
	bcdev->oplus_custom_gpio.otg_boost_en_sleep =
		pinctrl_lookup_state(bcdev->oplus_custom_gpio.otg_boost_en_pinctrl, "otg_booster_en_sleep");
	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.otg_boost_en_sleep)) {
		chg_err("get otg_booster_en_sleep\n");
		return -EINVAL;
	}

	pinctrl_select_state(bcdev->oplus_custom_gpio.otg_boost_en_pinctrl,
		bcdev->oplus_custom_gpio.otg_boost_en_sleep);

	return 0;
}

static int oplus_otg_ovp_en_gpio_init(struct battery_chg_dev *bcdev)
{
	if (!bcdev) {
		chg_err("bcdev not ready\n");
		return -EINVAL;
	}

	bcdev->oplus_custom_gpio.otg_ovp_en_pinctrl = devm_pinctrl_get(bcdev->dev);
	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.otg_ovp_en_pinctrl)) {
		chg_err("get otg_ovp_en_pinctrl fail\n");
		return -EINVAL;
	}
	bcdev->oplus_custom_gpio.otg_ovp_en_active =
		pinctrl_lookup_state(bcdev->oplus_custom_gpio.otg_ovp_en_pinctrl, "otg_ovp_en_active");
	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.otg_ovp_en_active)) {
		chg_err("get otg_ovp_en_active\n");
		return -EINVAL;
	}
	bcdev->oplus_custom_gpio.otg_ovp_en_sleep =
		pinctrl_lookup_state(bcdev->oplus_custom_gpio.otg_ovp_en_pinctrl, "otg_ovp_en_sleep");
	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.otg_ovp_en_sleep)) {
		chg_err("get otg_ovp_en_sleep\n");
		return -EINVAL;
	}

	pinctrl_select_state(bcdev->oplus_custom_gpio.otg_ovp_en_pinctrl,
		bcdev->oplus_custom_gpio.otg_ovp_en_sleep);

	return 0;
}

static void oplus_set_otg_boost_en_val(struct battery_chg_dev *bcdev, int value)
{
	if (bcdev->oplus_custom_gpio.otg_boost_en_gpio <= 0) {
		chg_err("otg_boost_en_gpio not exist, return\n");
		return;
	}

	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.otg_boost_en_pinctrl)
		|| IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.otg_boost_en_active)
		|| IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.otg_boost_en_sleep)) {
		chg_err("otg_boost_en pinctrl null, return\n");
		return;
	}

	if (value) {
		gpio_direction_output(bcdev->oplus_custom_gpio.otg_boost_en_gpio , 1);
		pinctrl_select_state(bcdev->oplus_custom_gpio.otg_boost_en_pinctrl,
				bcdev->oplus_custom_gpio.otg_boost_en_active);
	} else {
		gpio_direction_output(bcdev->oplus_custom_gpio.otg_boost_en_gpio, 0);
		pinctrl_select_state(bcdev->oplus_custom_gpio.otg_boost_en_pinctrl,
				bcdev->oplus_custom_gpio.otg_boost_en_sleep);
	}

	chg_err("<~OTG~>set value:%d, gpio_val:%d\n", value,
		gpio_get_value(bcdev->oplus_custom_gpio.otg_boost_en_gpio));
}

static void oplus_set_otg_ovp_en_val(struct battery_chg_dev *bcdev, int value)
{
	if (bcdev->oplus_custom_gpio.otg_ovp_en_gpio <= 0) {
		chg_err("otg_ovp_en_gpio not exist, return\n");
		return;
	}

	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.otg_ovp_en_pinctrl)
		|| IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.otg_ovp_en_active)
		|| IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.otg_ovp_en_sleep)) {
		chg_err("otg_ovp_en pinctrl null, return\n");
		return;
	}

	if (value) {
		gpio_direction_output(bcdev->oplus_custom_gpio.otg_ovp_en_gpio , 1);
		pinctrl_select_state(bcdev->oplus_custom_gpio.otg_ovp_en_pinctrl,
				bcdev->oplus_custom_gpio.otg_ovp_en_active);
	} else {
		gpio_direction_output(bcdev->oplus_custom_gpio.otg_ovp_en_gpio, 0);
		pinctrl_select_state(bcdev->oplus_custom_gpio.otg_ovp_en_pinctrl,
				bcdev->oplus_custom_gpio.otg_ovp_en_sleep);
	}

	chg_err("<~OTG~>set value:%d, gpio_val:%d\n", value,
		gpio_get_value(bcdev->oplus_custom_gpio.otg_ovp_en_gpio));
}

static bool is_support_tx_boost(struct battery_chg_dev *bcdev)
{
	if (!bcdev) {
		chg_err("bcdev not ready!\n");
		return false;
	}

	if (gpio_is_valid(bcdev->oplus_custom_gpio.tx_boost_en_gpio))
		return true;
	return false;
}

static int oplus_tx_boost_en_gpio_init(struct battery_chg_dev *bcdev)
{
	if (!bcdev) {
		chg_err("bcdev not ready!\n");
		return -EINVAL;
	}

	bcdev->oplus_custom_gpio.tx_boost_en_pinctrl = devm_pinctrl_get(bcdev->dev);
	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.tx_boost_en_pinctrl)) {
		chg_err("get tx_boost_en_pinctrl fail\n");
		return -EINVAL;
	}
	bcdev->oplus_custom_gpio.tx_boost_en_active =
		pinctrl_lookup_state(bcdev->oplus_custom_gpio.tx_boost_en_pinctrl, "tx_boost_en_active");
	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.tx_boost_en_active)) {
		chg_err("get tx_boost_en_active fail\n");
		return -EINVAL;
	}
	bcdev->oplus_custom_gpio.tx_boost_en_sleep =
		pinctrl_lookup_state(bcdev->oplus_custom_gpio.tx_boost_en_pinctrl, "tx_boost_en_sleep");
	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.tx_boost_en_sleep)) {
		chg_err("get tx_boost_en_sleep fail\n");
		return -EINVAL;
	}

	gpio_direction_output(bcdev->oplus_custom_gpio.tx_boost_en_gpio, 0);
	pinctrl_select_state(bcdev->oplus_custom_gpio.tx_boost_en_pinctrl,
		bcdev->oplus_custom_gpio.tx_boost_en_sleep);

	return 0;
}

#ifdef SUPORT_TX_BOOST
static void oplus_set_tx_boost_en_val(struct battery_chg_dev *bcdev, int value)
{
	if (!bcdev) {
		chg_err("bcdev not ready!\n");
		return;
	}

	if (bcdev->oplus_custom_gpio.tx_boost_en_gpio <= 0) {
		chg_err("tx_boost_en_gpio not exist, return\n");
		return;
	}

	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.tx_boost_en_pinctrl) ||
	    IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.tx_boost_en_active) ||
	    IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.tx_boost_en_sleep)) {
		chg_err("tx_boost_en pinctrl null, return\n");
		return;
	}

	if (value) {
		gpio_direction_output(bcdev->oplus_custom_gpio.tx_boost_en_gpio , 1);
		pinctrl_select_state(bcdev->oplus_custom_gpio.tx_boost_en_pinctrl,
			bcdev->oplus_custom_gpio.tx_boost_en_active);
	} else {
		gpio_direction_output(bcdev->oplus_custom_gpio.tx_boost_en_gpio, 0);
		pinctrl_select_state(bcdev->oplus_custom_gpio.tx_boost_en_pinctrl,
			bcdev->oplus_custom_gpio.tx_boost_en_sleep);
	}

	chg_info("set value:%d, gpio_val:%d\n", value,
		gpio_get_value(bcdev->oplus_custom_gpio.tx_boost_en_gpio));
}
#endif

static int oplus_tx_ovp_en_gpio_init(struct battery_chg_dev *bcdev)
{
	if (!bcdev) {
		chg_err("bcdev not ready!\n");
		return -EINVAL;
	}

	bcdev->oplus_custom_gpio.tx_ovp_en_pinctrl = devm_pinctrl_get(bcdev->dev);
	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.tx_ovp_en_pinctrl)) {
		chg_err("get tx_ovp_en_pinctrl fail\n");
		return -EINVAL;
	}
	bcdev->oplus_custom_gpio.tx_ovp_en_active =
		pinctrl_lookup_state(bcdev->oplus_custom_gpio.tx_ovp_en_pinctrl, "tx_ovp_en_active");
	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.tx_ovp_en_active)) {
		chg_err("get tx_ovp_en_active fail\n");
		return -EINVAL;
	}
	bcdev->oplus_custom_gpio.tx_ovp_en_sleep =
		pinctrl_lookup_state(bcdev->oplus_custom_gpio.tx_ovp_en_pinctrl, "tx_ovp_en_sleep");
	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.tx_ovp_en_sleep)) {
		chg_err("get tx_ovp_en_sleep fail\n");
		return -EINVAL;
	}

	gpio_direction_output(bcdev->oplus_custom_gpio.tx_ovp_en_gpio, 0);
	pinctrl_select_state(bcdev->oplus_custom_gpio.tx_ovp_en_pinctrl,
		bcdev->oplus_custom_gpio.tx_ovp_en_sleep);

	return 0;
}

static void oplus_set_tx_ovp_en_val(struct battery_chg_dev *bcdev, int value)
{
	if (!bcdev) {
		chg_err("bcdev not ready!\n");
		return;
	}

	if (bcdev->oplus_custom_gpio.tx_ovp_en_gpio <= 0) {
		chg_err("tx_ovp_en_gpio not exist, return\n");
		return;
	}

	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.tx_ovp_en_pinctrl) ||
	    IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.tx_ovp_en_active) ||
	    IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.tx_ovp_en_sleep)) {
		chg_err("otg_ovp_en pinctrl null, return\n");
		return;
	}

	if (value) {
		gpio_direction_output(bcdev->oplus_custom_gpio.tx_ovp_en_gpio , 1);
		pinctrl_select_state(bcdev->oplus_custom_gpio.tx_ovp_en_pinctrl,
			bcdev->oplus_custom_gpio.tx_ovp_en_active);
	} else {
		gpio_direction_output(bcdev->oplus_custom_gpio.tx_ovp_en_gpio, 0);
		pinctrl_select_state(bcdev->oplus_custom_gpio.tx_ovp_en_pinctrl,
			bcdev->oplus_custom_gpio.tx_ovp_en_sleep);
	}

	chg_info("set value:%d, gpio_val:%d\n", value,
		gpio_get_value(bcdev->oplus_custom_gpio.tx_ovp_en_gpio));
}

static int oplus_wrx_ovp_off_gpio_init(struct battery_chg_dev *bcdev)
{
	if (!bcdev) {
		chg_err("bcdev not ready!\n");
		return -EINVAL;
	}

	bcdev->oplus_custom_gpio.wrx_ovp_off_pinctrl = devm_pinctrl_get(bcdev->dev);
	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.wrx_ovp_off_pinctrl)) {
		chg_err("get wrx_ovp_off_pinctrl fail\n");
		return -EINVAL;
	}
	bcdev->oplus_custom_gpio.wrx_ovp_off_active =
		pinctrl_lookup_state(bcdev->oplus_custom_gpio.wrx_ovp_off_pinctrl, "wrx_ovp_off_active");
	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.wrx_ovp_off_active)) {
		chg_err("get tx_ovp_en_active fail\n");
		return -EINVAL;
	}
	bcdev->oplus_custom_gpio.wrx_ovp_off_sleep =
		pinctrl_lookup_state(bcdev->oplus_custom_gpio.wrx_ovp_off_pinctrl, "wrx_ovp_off_sleep");
	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.wrx_ovp_off_sleep)) {
		chg_err("get wrx_ovp_off_sleep fail\n");
		return -EINVAL;
	}

	gpio_direction_output(bcdev->oplus_custom_gpio.wrx_ovp_off_gpio, 0);
	pinctrl_select_state(bcdev->oplus_custom_gpio.wrx_ovp_off_pinctrl,
		bcdev->oplus_custom_gpio.tx_ovp_en_sleep);

	return 0;
}

#ifdef SUPORT_WRX_OVP_OFF
static void oplus_set_wrx_ovp_off_val(struct battery_chg_dev *bcdev, int value)
{
	if (!bcdev) {
		chg_err("bcdev not ready!\n");
		return;
	}

	if (bcdev->oplus_custom_gpio.wrx_ovp_off_gpio <= 0) {
		chg_err("wrx_ovp_off_gpio not exist, return\n");
		return;
	}

	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.wrx_ovp_off_pinctrl) ||
	    IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.wrx_ovp_off_active) ||
	    IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.wrx_ovp_off_sleep)) {
		chg_err("wrx_ovp_off pinctrl null, return\n");
		return;
	}

	if (value) {
		gpio_direction_output(bcdev->oplus_custom_gpio.wrx_ovp_off_gpio , 1);
		pinctrl_select_state(bcdev->oplus_custom_gpio.wrx_ovp_off_pinctrl,
			bcdev->oplus_custom_gpio.wrx_ovp_off_active);
	} else {
		gpio_direction_output(bcdev->oplus_custom_gpio.wrx_ovp_off_gpio, 0);
		pinctrl_select_state(bcdev->oplus_custom_gpio.wrx_ovp_off_pinctrl,
			bcdev->oplus_custom_gpio.wrx_ovp_off_sleep);
	}

	chg_info("set value:%d, gpio_val:%d\n", value,
		gpio_get_value(bcdev->oplus_custom_gpio.wrx_ovp_off_gpio));
}
#endif

static int oplus_wrx_otg_en_gpio_init(struct battery_chg_dev *bcdev)
{
	if (!bcdev) {
		chg_err("bcdev not ready!\n");
		return -EINVAL;
	}

	bcdev->oplus_custom_gpio.wrx_otg_en_pinctrl = devm_pinctrl_get(bcdev->dev);
	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.wrx_otg_en_pinctrl)) {
		chg_err("get wrx_otg_en_pinctrl fail\n");
		return -EINVAL;
	}

	bcdev->oplus_custom_gpio.wrx_otg_en_active =
		pinctrl_lookup_state(bcdev->oplus_custom_gpio.wrx_otg_en_pinctrl, "wrx_otg_en_active");
	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.wrx_otg_en_active)) {
		chg_err("get wrx_otg_en_active fail\n");
		return -EINVAL;
	}

	bcdev->oplus_custom_gpio.wrx_otg_en_sleep =
		pinctrl_lookup_state(bcdev->oplus_custom_gpio.wrx_otg_en_pinctrl, "wrx_otg_en_sleep");
	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.wrx_otg_en_sleep)) {
		chg_err("get wrx_otg_en_sleep fail\n");
		return -EINVAL;
	}

	gpio_direction_output(bcdev->oplus_custom_gpio.wrx_otg_en_gpio, 0);
	pinctrl_select_state(bcdev->oplus_custom_gpio.wrx_otg_en_pinctrl,
		bcdev->oplus_custom_gpio.wrx_otg_en_sleep);

	return 0;
}

static void oplus_set_wrx_otg_en_val(struct battery_chg_dev *bcdev, int value)
{
	if (!bcdev) {
		chg_err("bcdev not ready!\n");
		return;
	}

	if (bcdev->oplus_custom_gpio.wrx_otg_en_gpio <= 0) {
		chg_err("wrx_otg_en_gpio not exist, return\n");
		return;
	}

	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.wrx_otg_en_pinctrl) ||
	   IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.wrx_otg_en_active) ||
	   IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.wrx_otg_en_sleep)) {
		chg_err("wrx_otg_en pinctrl null, return\n");
		return;
	}

	if (value) {
		gpio_direction_output(bcdev->oplus_custom_gpio.wrx_otg_en_gpio , 1);
		pinctrl_select_state(bcdev->oplus_custom_gpio.wrx_otg_en_pinctrl,
			bcdev->oplus_custom_gpio.wrx_otg_en_active);
	} else {
		gpio_direction_output(bcdev->oplus_custom_gpio.wrx_otg_en_gpio, 0);
		pinctrl_select_state(bcdev->oplus_custom_gpio.wrx_otg_en_pinctrl,
			bcdev->oplus_custom_gpio.wrx_otg_en_sleep);
	}

	chg_info("set value:%d, gpio_val:%d\n", value,
		gpio_get_value(bcdev->oplus_custom_gpio.wrx_otg_en_gpio));
}

int oplus_adsp_batt_curve_current(void)
{
	int rc;
	static int batt_current = 0;
	struct battery_chg_dev *bcdev = g_bcdev;
	struct psy_state *pst;

	if (!bcdev) {
		chg_err("bcdev is NULL!\n");
		return -ENODEV;
	}
	pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_GET_BATT_CURR);
	else
		rc = read_property_id(bcdev, pst, USB_GET_BATT_CURR);
	if (rc < 0) {
		chg_err("read battery curr fail, rc=%d\n", rc);
		return batt_current * 100;
	}
	if (bcdev->soccp_support)
		batt_current = (int)bcdev->oplus_psy.prop[OPLUS_USB_GET_BATT_CURR];
	else
		batt_current = (int)pst->prop[USB_GET_BATT_CURR];
	batt_current = DIV_ROUND_CLOSEST((int)batt_current, 1000);
	chg_debug("get batt_curr = %d \n", batt_current);
	return batt_current * 100;
}

int oplus_adsp_voocphy_get_fast_chg_type(void)
{
	int rc = 0;
	struct battery_chg_dev *bcdev = g_bcdev;
	struct psy_state *pst = NULL;
	int fast_chg_type = 0;

	if (!bcdev) {
		chg_err("bcdev is NULL!\n");
		return -ENODEV;
	}

	pst = &bcdev->psy_list[PSY_TYPE_USB];
	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_VOOC_FAST_CHG_TYPE);
	else
		rc = read_property_id(bcdev, pst, USB_VOOC_FAST_CHG_TYPE);
	if (rc < 0) {
		chg_err("read vooc_fast_chg_type fail, rc=%d\n", rc);
		return 0;
	}
	if (bcdev->soccp_support)
		fast_chg_type = bcdev->oplus_psy.prop[OPLUS_USB_VOOC_FAST_CHG_TYPE] & 0x7F;
	else
		fast_chg_type = (pst->prop[USB_VOOC_FAST_CHG_TYPE]) & 0x7F;

	return fast_chg_type;
}

int oplus_adsp_voocphy_enable(bool enable)
{
	int rc = 0;
	struct battery_chg_dev *bcdev = g_bcdev;
	struct psy_state *pst = NULL;

	if (!bcdev) {
		chg_err("bcdev is NULL!\n");
		return -ENODEV;
	}
	if (oplus_chg_get_voocphy_support(bcdev) != ADSP_VOOCPHY)
		return rc;
	pst = &bcdev->psy_list[PSY_TYPE_USB];
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_VOOCPHY_ENABLE, enable);
	else
		rc = write_property_id(bcdev, pst, USB_VOOCPHY_ENABLE, enable);
	if (rc) {
		chg_err("set %s adsp voocphy fail, rc=%d\n", enable ? "enable" : "disable", rc);
	} else {
		chg_err("set %s adsp voocphy success\n", enable ? "enable" : "disable");
	}

	return rc;
}

static int oplus_adsp_pps_mos_ctrl(struct battery_chg_dev *bcdev, int on)
{
	int rc = 0;
	struct psy_state *pst = NULL;

	pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_PPS_MOS_CTRL, on);
	else
		rc = write_property_id(bcdev, pst, USB_PPS_MOS_CTRL, on);
	if (rc < 0) {
		chg_err("%s mos error, rc = %d\n", on ? "open" : "close", rc);
		return rc;
	}
	chg_info("%s mos\n", on ? "open" : "close");

	return 0;
}

void oplus_turn_off_power_when_adsp_crash(void)
{
	struct battery_chg_dev *bcdev = g_bcdev;
	int ufcs_notify_val = UFCS_NOTIFY_EXIT_BY_CRASH;

	if (!bcdev) {
		chg_err("bcdev is null\n");
		return;
	}
	bcdev->last_charger_type = bcdev->charger_type;
	bcdev->adsp_crash = 1;

	if (bcdev->adsp_reboot_discnt_chg_support)
		atomic_set(&bcdev->adsp_reboot, 1);

	chg_err("last_charger_type:%d\n", bcdev->last_charger_type);
	bcdev->is_chargepd_ready = false;
	bcdev->pd_svooc = false;
	bcdev->pd_check_completed = false;
	oplus_chg_ic_virq_trigger(bcdev->buck_ic, OPLUS_IC_VIRQ_SVID);

	schedule_delayed_work(&bcdev->plugin_irq_work, 0);
	oplus_adsp_voocphy_fastchg_event_handle(ADSP_VPHY_FAST_NOTIFY_CRASH);
	plat_ufcs_send_state(UFCS_NOTIFY_EXIT_COMM, (void *)&ufcs_notify_val);

	/* Delay 1s to wait for minidump to be generated */
	if (bcdev->soccp_support)
		schedule_delayed_work(&bcdev->crash_track_work, msecs_to_jiffies(1000));

	if (bcdev->adsp_reboot_discnt_chg_support)
		schedule_delayed_work(&bcdev->crash_timeout_work, msecs_to_jiffies(5000));
}
EXPORT_SYMBOL(oplus_turn_off_power_when_adsp_crash);

bool oplus_is_pd_svooc(void)
{
	struct battery_chg_dev *bcdev = g_bcdev;

	if (!bcdev) {
		chg_err("bcdev is null\n");
		return false;
	}

	chg_info("pd_svooc = %d\n", bcdev->pd_svooc);

	return bcdev->pd_svooc;
}
EXPORT_SYMBOL(oplus_is_pd_svooc);

void oplus_adsp_crash_recover_work(void)
{
	struct battery_chg_dev *bcdev = g_bcdev;

	if (!bcdev) {
		chg_err("bcdev is null\n");
		return;
	}
	chg_err("oplus_adsp_crash_recover_work");

	/* Fix the issue of the PMIC not enable after ADSP SSR,
	 * force the usb in status as 0 in the irq_work and clean the charge status.
	 */
	if (bcdev->adsp_reboot_discnt_chg_support)
		atomic_set(&bcdev->adsp_reboot, 0);

	schedule_delayed_work(&bcdev->adsp_crash_recover_work,
			      round_jiffies_relative(msecs_to_jiffies(1500)));
}
EXPORT_SYMBOL(oplus_adsp_crash_recover_work);

static int oplus_ap_init_adsp_gague(struct battery_chg_dev *bcdev)
{
	int rc = 0;
	struct psy_state *pst = pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	rc = write_property_id(bcdev, pst, BATT_ADSP_GAUGE_INIT, 1);
	if (rc)
		chg_err("init adsp gague fail, rc=%d\n", rc);
	else
		chg_err("init adsp gague sucess.");

	return rc;
}

static void oplus_adsp_crash_recover_func(struct work_struct *work)
{
	struct battery_chg_dev *bcdev =
		container_of(work, struct battery_chg_dev, adsp_crash_recover_work.work);
	struct psy_state *pst = NULL;
	int ufcs_notify_val = UFCS_NOTIFY_RESTART_FROM_CRASH;

	chg_err("oplus_adsp_crash_recover_func");
	if (bcdev->soccp_support)
		schedule_delayed_work(&bcdev->update_common_charge_flag_work, 0);
	if (oplus_chg_get_voocphy_support(bcdev) == ADSP_VOOCPHY) {
		oplus_ap_init_adsp_gague(bcdev);
	}
	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	if (bcdev->last_charger_type == OPLUS_CHG_USB_TYPE_QC2) {
		chg_err("recover QC");
		if (bcdev->soccp_support)
			write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_SET_QC, 0);
		else
			write_property_id(bcdev, pst, BATT_SET_QC, 0);
	}
	schedule_delayed_work(&bcdev->otg_init_work, msecs_to_jiffies(500));
	schedule_delayed_work(&bcdev->plugin_irq_work, 0);
	msleep(2000);
	bcdev->adsp_crash = 0;
	bcdev->ufcs_key_to_adsp_done = false;
	if (bcdev->last_charger_type == OPLUS_CHG_USB_TYPE_QC2) {
		chg_err("recover QC OPLUS_IC_VIRQ_CHG_TYPE_CHANGE");
		oplus_chg_ic_virq_trigger(bcdev->buck_ic, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);
	}
	plat_ufcs_send_state(UFCS_NOTIFY_RESTART_COMM, (void *)&ufcs_notify_val);
}

static bool oplus_chg_crash_track_limit(void)
{
	static int upload_count = 0;
	static int pre_upload_time = 0;
	int curr_time;

	curr_time = local_clock() / TRACK_LOCAL_T_NS_TO_S_THD;
	if (curr_time - pre_upload_time > TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD)
		upload_count = 0;

	if (upload_count >= TRACK_UPLOAD_COUNT_MAX)
		return true;
	pre_upload_time = local_clock() / TRACK_LOCAL_T_NS_TO_S_THD;
	upload_count++;

	return false;
}

static void oplus_crash_track_work(struct work_struct *work)
{
	struct battery_chg_dev *chip =
		container_of(work, struct battery_chg_dev, crash_track_work.work);
	struct mms_msg *msg;
	int rc;

	if (!is_err_topic_available(chip)) {
		chg_err("error topic not found\n");
		return;
	}

	if (oplus_chg_crash_track_limit())
		return;

	msg = oplus_mms_alloc_int_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_SOCCP_CRASH, 0);
	if (msg == NULL) {
		chg_err("alloc soccp crash msg error\n");
		return;
	}

	rc = oplus_mms_publish_msg_sync(chip->err_topic, msg);
	if (rc < 0) {
		chg_err("publish soccp crash msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_crash_timeout_work(struct work_struct *work)
{
	struct battery_chg_dev *chip =
		container_of(work, struct battery_chg_dev, crash_timeout_work.work);

	if (atomic_read(&chip->adsp_reboot)) {
		chg_err("adsp_reboot is not clear by the ADSP recovery, force to clear it.\n");
		atomic_set(&chip->adsp_reboot, 0);
	}
}

static bool is_chg_disable_votable_available(struct battery_chg_dev *bcdev)
{
	if (!bcdev->chg_disable_votable)
		bcdev->chg_disable_votable = find_votable("CHG_DISABLE");

	return !!bcdev->chg_disable_votable;
}

static void oplus_voocphy_enable_check_func(struct work_struct *work)
{
	int rc;
	int voocphy_enable = 0;
	int mmi_chg = 1;
	int charger_type;
	int prop_id = 0;
	struct psy_state *pst;
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, voocphy_enable_check_work.work);

	if (oplus_chg_get_voocphy_support(bcdev) != ADSP_VOOCPHY)
		return;

	if (is_chg_disable_votable_available(bcdev))
		mmi_chg = !get_client_vote(bcdev->chg_disable_votable, MMI_CHG_VOTER);
	if (mmi_chg == 0)
		goto done;

	pst = &bcdev->psy_list[PSY_TYPE_USB];
	prop_id = get_property_id(pst, POWER_SUPPLY_PROP_USB_TYPE);
	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_TYPE);
	else
		rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0) {
		chg_err("read usb charger_type fail, rc=%d\n", rc);
		return;
	}

	if (bcdev->soccp_support)
		charger_type = bcdev->oplus_psy.prop[OPLUS_USB_TYPE];
	else
		charger_type = pst->prop[prop_id];
	chg_info("%s: mmi_chg = %d, charger_type = %d\n", __func__, mmi_chg, charger_type);

	if (charger_type != POWER_SUPPLY_TYPE_USB_DCP) {
		chg_err("charger_type != POWER_SUPPLY_TYPE_USB_DCP\n");
		goto done;
	}

	voocphy_enable = oplus_get_voocphy_enable(bcdev);
	if (voocphy_enable == 0) {
		chg_err("need enable voocphy again\n");
		rc = oplus_voocphy_enable(bcdev, true);
		schedule_delayed_work(&bcdev->voocphy_enable_check_work,
				      round_jiffies_relative(msecs_to_jiffies(500)));
		return;
	}
done:
	schedule_delayed_work(&bcdev->voocphy_enable_check_work,
			      round_jiffies_relative(msecs_to_jiffies(5000)));
}

static void otg_notification_handler(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
					struct battery_chg_dev, otg_vbus_enable_work.work);

	if (!bcdev) {
		chg_info("bcdev is null, return\n");
		return;
	}

	if (bcdev->otg_boost_src == OTG_BOOST_SOURCE_EXTERNAL) {
		if (bcdev->otg_online) {
			oplus_set_otg_ovp_en_val(bcdev, 1);
			oplus_set_otg_boost_en_val(bcdev, 1);
		} else {
			oplus_set_otg_boost_en_val(bcdev, 0);
			oplus_set_otg_ovp_en_val(bcdev, 0);
		}
	}
}

static bool oplus_chg_is_usb_present(struct battery_chg_dev *bcdev)
{
	bool vbus_rising = bcdev->usb_in_status;

	if (oplus_vchg_trig_is_support() == true
			&& oplus_get_vchg_trig_status() == 1 && vbus_rising == true) {
		vbus_rising = false;
	}

#ifdef OPLUS_CHG_UNDEF /* TODO */
	if (vbus_rising == false && (oplus_wpc_get_wireless_charge_start() || oplus_chg_is_wls_present())) {
		chg_err("USBIN_PLUGIN_RT_STS_BIT low but wpc has started\n");
		vbus_rising = true;
	}
#endif
	return vbus_rising;
}

static void oplus_hvdcp_disable_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, hvdcp_disable_work.work);

	if (oplus_chg_is_usb_present(bcdev) == false) {
		chg_info("set bcdev->hvdcp_disable false\n");
		bcdev->hvdcp_disable = false;
	}
}

static void oplus_pd_only_check_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, pd_only_check_work.work);

	if (bcdev->pd_svooc == false) {
		oplus_chg_ic_virq_trigger(bcdev->buck_ic, OPLUS_IC_VIRQ_SVID);
		chg_info("!!!pd_svooc[%d]\n", bcdev->pd_svooc);
	}
}

static int oplus_chg_get_match_temp(struct battery_chg_dev *bcdev)
{
	int temp;
	union mms_msg_data data = {0};

	if (is_common_topic_available(bcdev)) {
		oplus_mms_get_item_data(bcdev->common_topic,
					COMM_ITEM_SHELL_TEMP, &data, false);
		temp = data.intval;
	} else {
		chg_err("common topic not found\n");
		temp = 320;
	}

	return temp;
}

static int oplus_chg_get_ui_soc(struct battery_chg_dev *bcdev)
{
	int ui_soc;
	union mms_msg_data data = {0};

	if (is_common_topic_available(bcdev)) {
		oplus_mms_get_item_data(bcdev->common_topic,
				COMM_ITEM_UI_SOC, &data, false);
		ui_soc = data.intval;
	} else {
		chg_err("common topic not found\n");
		ui_soc = 50;
	}

	return ui_soc;
}

#define OTG_SKIN_TEMP_HIGH 450
#define OTG_SKIN_TEMP_MAX 540
static int oplus_get_bat_info_for_otg_status_check(struct battery_chg_dev *bcdev,
						   int *soc, int *ichaging)
{
	struct psy_state *pst = NULL;
	int rc = 0;
	int prop_id = 0;

	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	prop_id = get_property_id(pst, POWER_SUPPLY_PROP_CURRENT_NOW);
	rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0) {
		chg_err("read battery curr fail, rc=%d\n", rc);
		return -1;
	}
	*ichaging = DIV_ROUND_CLOSEST((int)pst->prop[prop_id], 1000);

	prop_id = get_property_id(pst, POWER_SUPPLY_PROP_CAPACITY);
	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_CAPACITY);
	else
		rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0) {
		chg_err("read battery soc fail, rc=%d\n", rc);
		return -1;
	}

	if (bcdev->soccp_support)
		*soc = DIV_ROUND_CLOSEST(bcdev->oplus_psy.prop[OPLUS_CAPACITY], 100);
	else
		*soc = DIV_ROUND_CLOSEST(pst->prop[prop_id], 100);

	return 0;
}

#define OTG_PROHIBITED_CURR_HIGH_THR	3000
#define OTG_PROHIBITED_CURR_LOW_THR	1700
static void oplus_otg_status_check_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct battery_chg_dev *bcdev = container_of(dwork,
			struct battery_chg_dev, otg_status_check_work);
	int rc;
	int skin_temp = 0, batt_current = 0, real_soc = 0;
	bool contion1 = false, contion2 = false, contion3 = false, contion4 = false, contion5 = false;
	static int otg_protect_cnt = 0;

	if (bcdev == NULL) {
		pr_err("battery_chg_dev is NULL\n");
		return;
	}

	skin_temp = oplus_chg_get_match_temp(bcdev);
	rc = oplus_get_bat_info_for_otg_status_check(bcdev, &real_soc, &batt_current);
	if (rc < 0) {
		pr_err("Error oplus_get_bat_info_for_otg_status_check, rc = %d\n", rc);
		return;
	}

	real_soc = fg_sm8350_get_battery_soc();
	chg_info("batt_current = %d, skin_temp = %d, real_soc = %d, otg_protect_cnt(%d)\n",
		batt_current, skin_temp, real_soc, otg_protect_cnt);
	contion1 = ((batt_current > OTG_PROHIBITED_CURR_LOW_THR) && (skin_temp > OTG_SKIN_TEMP_HIGH));
	contion2 = (batt_current > OTG_PROHIBITED_CURR_HIGH_THR);
	contion3 = (skin_temp > OTG_SKIN_TEMP_MAX);
	contion4 = ((real_soc < 10) && (batt_current > OTG_PROHIBITED_CURR_LOW_THR));
	contion5 = ((skin_temp < 0) && (batt_current > OTG_PROHIBITED_CURR_LOW_THR));

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if ((contion1 || contion2 || contion3 || contion4 || contion5) && (get_eng_version() != HIGH_TEMP_AGING)) {
#else
	if ((contion1 || contion2 || contion3 || contion4 || contion5)) {
#endif
		otg_protect_cnt++;
		if (otg_protect_cnt >= 2) {
			if (!bcdev->otg_prohibited) {
				bcdev->otg_prohibited = true;
				schedule_delayed_work(&bcdev->otg_vbus_enable_work, 0);
				chg_err("OTG prohibited, batt_current = %d, skin_temp = %d, real_soc = %d\n",
					batt_current, skin_temp, real_soc);
			}
		}
	} else {
		otg_protect_cnt = 0;
	}

	if (!bcdev->otg_online) {
		if (bcdev->otg_prohibited) {
			bcdev->otg_prohibited = false;
		}
		chg_err("otg_online is false, exit\n");
		return;
	}

	schedule_delayed_work(&bcdev->otg_status_check_work, msecs_to_jiffies(1000));
}

static void oplus_vbus_enable_adc_work(struct work_struct *work)
{
	oplus_chg_disable_charger(true, FASTCHG_VOTER);
	oplus_chg_suspend_charger(true, FASTCHG_VOTER);
}
#endif /*OPLUS_FEATURE_CHG_BASIC*/

#ifdef OPLUS_FEATURE_CHG_BASIC
#ifdef OPLUS_CHG_UNDEF
static void oplus_wait_wired_charge_on_work(struct work_struct *work)
{
	chg_info("<~WPC~> wait_wired_charge_on\n");
	oplus_wpc_set_wrx_en_value(0);
	oplus_wpc_set_wls_pg_value(1);
	msleep(100);
	oplus_wpc_set_booster_en_val(1);
	oplus_wpc_set_ext2_wireless_otg_en_val(1);
	msleep(100);
	oplus_wpc_set_tx_start();
	return;
}

static void oplus_switch_to_wired_charge(struct battery_chg_dev *bcdev)
{
	oplus_wpc_dis_wireless_chg(1);
	if (oplus_wpc_get_wireless_charge_start() == true) {
		/*oplus_wpc_dis_wireless_chg(1);*/
		oplus_wpc_set_vbat_en_val(0);
		msleep(100);
		oplus_wpc_set_wrx_en_value(0);
		oplus_wpc_set_wls_pg_value(1);
	}

	if (oplus_wpc_get_otg_charging()) {
		/*oplus_wpc_dis_wireless_chg(1);*/
		mp2650_wireless_set_mps_otg_en_val(0);
		oplus_wpc_set_wrx_otg_en_value(0);

		cancel_delayed_work_sync(&bcdev->wait_wired_charge_on);
		schedule_delayed_work(&bcdev->wait_wired_charge_on, msecs_to_jiffies(100));
	}
}

static void oplus_wait_wired_charge_off_work(struct work_struct *work)
{
	chg_info("<~WPC~> wait_wired_charge_off\n");
	oplus_wpc_dis_wireless_chg(0);
	oplus_wpc_set_rtx_function_prepare();
	oplus_wpc_set_rtx_function(true);
	return;
}

static void oplus_switch_from_wired_charge(struct battery_chg_dev *bcdev)
{
	if (oplus_wpc_get_otg_charging()) {
		oplus_wpc_set_booster_en_val(0);
		oplus_wpc_set_ext2_wireless_otg_en_val(0);
		oplus_wpc_set_wls_pg_value(0);
		cancel_delayed_work_sync(&bcdev->wait_wired_charge_off);
		schedule_delayed_work(&bcdev->wait_wired_charge_off, msecs_to_jiffies(100));
	} else {
		if (oplus_wpc_get_fw_updating() == false)
			oplus_wpc_dis_wireless_chg(0);
	}
}
#endif

__maybe_unused
static bool oplus_get_wired_otg_online(void)
{
	struct battery_chg_dev *bcdev = g_bcdev;

	if (!bcdev) {
		chg_err("bcdev is NULL!\n");
		return false;
	}

	if (bcdev->wls_fw_update == true)
		return false;
	return bcdev->otg_online;
}

bool oplus_get_wired_chg_present(void)
{
	struct battery_chg_dev *bcdev = g_bcdev;

	if (!bcdev) {
		chg_err("bcdev is NULL!\n");
		return false;
	}
	return bcdev->usb_in_status;
}
#endif /*OPLUS_FEATURE_CHG_BASIC*/

static void battery_chg_state_cb(void *priv, enum pmic_glink_state state)
{
	struct battery_chg_dev *bcdev = priv;

	pr_debug("state: %d\n", state);

	atomic_set(&bcdev->state, state);
	if (state == PMIC_GLINK_STATE_UP)
		schedule_work(&bcdev->subsys_up_work);
}

/**
 * qti_battery_charger_get_prop() - Gets the property being requested
 *
 * @name: Power supply name
 * @prop_id: Property id to be read
 * @val: Pointer to value that needs to be updated
 *
 * Return: 0 if success, negative on error.
 */
int qti_battery_charger_get_prop(const char *name,
				enum battery_charger_prop prop_id, int *val)
{
	struct power_supply *psy;
	struct battery_chg_dev *bcdev;
	struct psy_state *pst;
	int rc = 0;

	if (prop_id >= BATTERY_CHARGER_PROP_MAX)
		return -EINVAL;

	if (strcmp(name, "battery") && strcmp(name, "usb") &&
	    strcmp(name, "wireless"))
		return -EINVAL;

	psy = power_supply_get_by_name(name);
	if (!psy)
		return -ENODEV;

#ifndef OPLUS_FEATURE_CHG_BASIC
	bcdev = power_supply_get_drvdata(psy);
#else
	bcdev = g_bcdev;
#endif
	if (!bcdev)
		return -ENODEV;

	power_supply_put(psy);

	switch (prop_id) {
	case BATTERY_RESISTANCE:
		pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
		rc = read_property_id(bcdev, pst, BATT_RESISTANCE);
		if (!rc)
			*val = pst->prop[BATT_RESISTANCE];
		break;
	default:
		break;
	}

	return rc;
}
EXPORT_SYMBOL(qti_battery_charger_get_prop);

static bool validate_message(struct battery_chg_dev *bcdev,
			struct battery_charger_resp_msg *resp_msg, size_t len)
{
	if (len != sizeof(*resp_msg)) {
		chg_err("Incorrect response length %zu for opcode %#x\n", len,
			resp_msg->hdr.opcode);
		return false;
	}

	if (resp_msg->ret_code) {
		chg_err("Error in response for opcode %#x prop_id %u, rc=%d\n",
			resp_msg->hdr.opcode, resp_msg->property_id,
			(int)resp_msg->ret_code);
		bcdev->error_prop = true;
		return false;
	}

	return true;
}

#define MODEL_DEBUG_BOARD	"Debug_Board"
static void handle_message(struct battery_chg_dev *bcdev, void *data,
				size_t len)
{
	struct battery_charger_resp_msg *resp_msg = data;
	struct battery_model_resp_msg *model_resp_msg = data;
	struct wireless_fw_check_resp *fw_check_msg;
	struct wireless_fw_push_buf_resp *fw_resp_msg;
	struct wireless_fw_update_status *fw_update_msg;
	struct wireless_fw_get_version_resp *fw_ver_msg;
	struct psy_state *pst;
	bool ack_set = false;

	switch (resp_msg->hdr.opcode) {
	case BC_BATTERY_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

		/* Handle model response uniquely as it's a string */
		if (pst->model && len == sizeof(*model_resp_msg)) {
			memcpy(pst->model, model_resp_msg->model, MAX_STR_LEN);
			ack_set = true;
			bcdev->debug_battery_detected = !strcmp(pst->model,
					MODEL_DEBUG_BOARD);
			break;
		}

		/* Other response should be of same type as they've u32 value */
		if (validate_message(bcdev, resp_msg, len) &&
		    resp_msg->property_id < pst->prop_count) {
			pst->prop[resp_msg->property_id] = resp_msg->value;
			ack_set = true;
		}

		break;
	case BC_USB_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_USB];
		if (validate_message(bcdev, resp_msg, len) &&
		    resp_msg->property_id < pst->prop_count) {
			pst->prop[resp_msg->property_id] = resp_msg->value;
			ack_set = true;
		}

		break;
	case BC_WLS_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_WLS];
		if (validate_message(bcdev, resp_msg, len) &&
		    resp_msg->property_id < pst->prop_count) {
			pst->prop[resp_msg->property_id] = resp_msg->value;
			ack_set = true;
		}

		break;
	case BC_BATTERY_STATUS_SET:
	case BC_USB_STATUS_SET:
	case BC_WLS_STATUS_SET:
		if (validate_message(bcdev, data, len))
			ack_set = true;

		break;
	case BC_SET_NOTIFY_REQ:
	case BC_SHUTDOWN_NOTIFY:
		/* Always ACK response for notify request */
		ack_set = true;
		break;
	case BC_WLS_FW_CHECK_UPDATE:
		if (len == sizeof(*fw_check_msg)) {
			fw_check_msg = data;
			if (fw_check_msg->ret_code == 1)
				bcdev->wls_fw_update_reqd = true;
			ack_set = true;
		} else {
			chg_err("Incorrect response length %zu for wls_fw_check_update\n",
				len);
		}
		break;
	case BC_WLS_FW_PUSH_BUF_RESP:
		if (len == sizeof(*fw_resp_msg)) {
			fw_resp_msg = data;
			if (fw_resp_msg->fw_update_status == 1)
				complete(&bcdev->fw_buf_ack);
		} else {
			chg_err("Incorrect response length %zu for wls_fw_push_buf_resp\n",
				len);
		}
		break;
	case BC_WLS_FW_UPDATE_STATUS_RESP:
		if (len == sizeof(*fw_update_msg)) {
			fw_update_msg = data;
			if (fw_update_msg->fw_update_done == 1)
				complete(&bcdev->fw_update_ack);
		} else {
			chg_err("Incorrect response length %zu for wls_fw_update_status_resp\n",
				len);
		}
		break;
	case BC_WLS_FW_GET_VERSION:
		if (len == sizeof(*fw_ver_msg)) {
			fw_ver_msg = data;
			bcdev->wls_fw_version = fw_ver_msg->fw_version;
			ack_set = true;
		} else {
			chg_err("Incorrect response length %zu for wls_fw_get_version\n",
				len);
		}
		break;
	default:
		chg_err("Unknown opcode: %u\n", resp_msg->hdr.opcode);
		break;
	}

	if (ack_set || bcdev->error_prop)
		complete(&bcdev->ack);
}

#ifdef OPLUS_FEATURE_CHG_BASIC
static void oplus_handle_message(struct battery_chg_dev *bcdev, void *data,
				size_t len)
{
	struct battery_charger_resp_msg *resp_msg = data;
	struct psy_state *pst;
	bool ack_set = false;

	switch (resp_msg->hdr.opcode) {
	case OPLUS_OPCODE_GET_REQ:
		pst = &bcdev->oplus_psy;
		if (validate_message(bcdev, resp_msg, len) &&
		    resp_msg->property_id < pst->prop_count) {
			pst->prop[resp_msg->property_id] = resp_msg->value;
			ack_set = true;
		}

		break;
	case OPLUS_OPCODE_SET_REQ:
		if (validate_message(bcdev, data, len))
			ack_set = true;

		break;
	default:
		chg_err("Unknown opcode: %u\n", resp_msg->hdr.opcode);
		break;
	}

	if (ack_set || bcdev->error_prop)
		complete(&bcdev->ack);
}
#endif

static void oplus_pd_completed_check(struct battery_chg_dev *bcdev, int adap_type)
{
	if (!bcdev)
		return;

	if (!bcdev->usb_in_status) {
		chg_info("unplug no check\n");
		return;
	}

	if (bcdev->pd_check_completed)
		return;

	chg_info("adap_type %d \n", adap_type);

	if (adap_type == POWER_SUPPLY_USB_TYPE_PD || adap_type == POWER_SUPPLY_USB_TYPE_PD_PPS) {
		bcdev->pd_check_completed = true;
		oplus_chg_ic_virq_trigger(bcdev->buck_ic, OPLUS_IC_VIRQ_PD_COMPLETED);
		chg_info("pd or pps send pd completed\n");
	}
}

static struct power_supply_desc usb_psy_desc;

static void battery_chg_update_usb_type_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
					struct battery_chg_dev, usb_type_work);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
#ifdef OPLUS_FEATURE_CHG_BASIC
	static int last_usb_adap_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	int adap_type = 0;
#endif
	int rc;

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_TYPE);
	else
		rc = read_property_id(bcdev, pst, USB_ADAP_TYPE);
	if (rc < 0) {
		chg_err("Failed to read USB_ADAP_TYPE rc=%d\n", rc);
		return;
	}

	if (bcdev->soccp_support)
		adap_type = bcdev->oplus_psy.prop[OPLUS_USB_TYPE];
	else
		adap_type = pst->prop[USB_ADAP_TYPE];
	chg_info("usb_adap_type: %u\n", adap_type);

	switch (adap_type) {
	case POWER_SUPPLY_USB_TYPE_SDP:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB;
		break;
	case POWER_SUPPLY_USB_TYPE_DCP:
	case POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID:
	case QTI_POWER_SUPPLY_USB_TYPE_HVDCP:
	case QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3:
	case QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3P5:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case POWER_SUPPLY_USB_TYPE_CDP:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case POWER_SUPPLY_USB_TYPE_ACA:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_ACA;
		break;
	case POWER_SUPPLY_USB_TYPE_C:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_TYPE_C;
		break;
	case POWER_SUPPLY_USB_TYPE_PD:
	case POWER_SUPPLY_USB_TYPE_PD_DRP:
	case POWER_SUPPLY_USB_TYPE_PD_PPS:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_PD;
		oplus_pd_completed_check(bcdev, adap_type);
		break;
	default:
#ifndef OPLUS_FEATURE_CHG_BASIC
		rc = read_property_id(bcdev, pst, USB_ONLINE);
		if (rc < 0) {
			chg_err("Failed to read USB_ONLINE rc=%d\n", rc);
			return;
		}
		if (pst->prop[USB_ONLINE] == 0)
			usb_psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
#else
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB;
#endif
		break;
	}

#ifdef OPLUS_FEATURE_CHG_BASIC
	chg_debug("trigger virq OPLUS_IC_VIRQ_CHG_TYPE_CHANGE");
	oplus_chg_ic_virq_trigger(bcdev->buck_ic, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);
	last_usb_adap_type = adap_type;
#endif
}

static int voocphy_push_gan_mos_err(struct oplus_chg_ic_dev *ic_dev)
{
	oplus_chg_ic_creat_err_msg(ic_dev,
			OPLUS_IC_ERR_GAN_MOS_ERROR, 0, "$$err_reason@@Gan_mos_10V");
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_ERR);
	return 0;
}

static void oplus_check_adspfg_status_work(struct work_struct *work)
{
	int ufcs_notify_val = UFCS_NOTIFY_FGRESET_EXIT_COMM;
	static unsigned long fgrset_check_timeout;

	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, check_adspfg_status.work);

	if (!bcdev || oplus_chg_get_voocphy_support(bcdev) != ADSP_VOOCPHY)
		return;
	if (!bcdev->adspfg_i2c_reset_notify_done) {
		fgrset_check_timeout = jiffies;
		ufcs_notify_val = UFCS_NOTIFY_FGRESET_EXIT_COMM;
		plat_ufcs_send_state(UFCS_NOTIFY_EXIT_COMM, (void *)&ufcs_notify_val);
		bcdev->adspfg_i2c_reset_notify_done = true;
	} else {
		if (bcdev->read_buffer_dump.data_buffer[0] == GAUGE_INVALID_TEMP) {
			if (time_is_after_jiffies(fgrset_check_timeout + (unsigned long)(60 * HZ))) {
				ufcs_notify_val = UFCS_NOTIFY_FGRESET_RESTART_COMM;
				plat_ufcs_send_state(UFCS_NOTIFY_RESTART_COMM, (void *)&ufcs_notify_val);
				bcdev->adspfg_i2c_reset_processing = false;
			} else {
				oem_read_buffer(bcdev);
			}
		} else {
			ufcs_notify_val = UFCS_NOTIFY_FGRESET_RESTART_COMM;
			bcdev->adspfg_i2c_reset_processing = false;
			plat_ufcs_send_state(UFCS_NOTIFY_RESTART_COMM, (void *)&ufcs_notify_val);
		}
	}
	chg_err("tbat %d", bcdev->read_buffer_dump.data_buffer[0]);
	if (bcdev->adspfg_i2c_reset_processing)
		schedule_delayed_work(&bcdev->check_adspfg_status, msecs_to_jiffies(10000));
}

static void oplus_chg_store_vdm_info(struct battery_chg_dev *bcdev,
	u32 *vdm_info_data, size_t vdm_info_cnt)
{
	int i = 0;

	if (vdm_info_cnt > OPLUS_VDM_INFO_MAX) {
		vdm_info_cnt = OPLUS_VDM_INFO_MAX;
		chg_err("Incorrect buffer length: %zu, max: %u\n", vdm_info_cnt, OPLUS_VDM_INFO_MAX);
	}
	mutex_lock(&bcdev->vdm_info_lock);
	memmove(bcdev->vdm_info_data, vdm_info_data, vdm_info_cnt * sizeof(u32));
	bcdev->vdm_info_cnt = vdm_info_cnt;
	mutex_unlock(&bcdev->vdm_info_lock);
	for (i = 0; i < vdm_info_cnt; i++) {
		chg_info("vdm info buffer: 0x%08x\n", bcdev->vdm_info_data[i]);
	}
}

static void handle_pd_info_buffer(struct battery_chg_dev *bcdev,
	struct oplus_ap_read_pd_info_msg *resp_msg, size_t len)
{
	size_t buf_len;

	if (len != sizeof(struct oplus_ap_read_pd_info_msg)) {
		chg_err("Incorrect buffer length: %zu\n", len);
		return;
	}
	chg_info("got the pd partner info msg_id=%d, len=%zu\n", resp_msg->msg_id, len);

	buf_len = resp_msg->data_size;
	if (buf_len == 0 || buf_len > sizeof(resp_msg->data_buffer)) {
		chg_err("Incorrect buffer length: %zu\n", buf_len);
		return;
	}

	switch (resp_msg->msg_id) {
	case OPLUS_PD_INFO_MSG_ID_VDM_ID:
		if (buf_len % sizeof(u32) != 0) {
			chg_err("Incorrect buffer length: %zu\n", buf_len);
			return;
		}
		oplus_chg_store_vdm_info(bcdev, resp_msg->data_buffer, buf_len / sizeof(u32));
		break;
	default:
		chg_err("got an undefined pd info msg_id\n");
		break;
	}
}

static void handle_notification(struct battery_chg_dev *bcdev, void *data,
				size_t len)
{
	struct battery_charger_notify_msg *notify_msg = data;
	struct psy_state *pst = NULL;
	int ret = 0;

	if (len != sizeof(*notify_msg)) {
		chg_err("Incorrect response length %zu\n", len);
		return;
	}

	chg_info("%s: notification: 0x%x\n", __func__, notify_msg->notification);

	switch (notify_msg->notification) {
	case BC_BATTERY_STATUS_GET:
	case BC_GENERIC_NOTIFY:
		pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
		break;
	case BC_USB_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_USB];
		schedule_work(&bcdev->usb_type_work);
		break;
	case BC_WLS_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_WLS];
		break;
#ifdef OPLUS_FEATURE_CHG_BASIC
	case BC_PD_SVOOC:
#ifdef OPLUS_CHG_UNDEF /* TODO */
		if ((get_oplus_chg_chip() && get_oplus_chg_chip()->wireless_support == false)
			|| oplus_get_wired_chg_present() == true) {
			chg_info("should set pd_svooc\n");
			oplus_usb_set_none_role();
			bcdev->pd_svooc = true;
		}
#endif
		bcdev->pd_svooc = true;
		oplus_chg_ic_virq_trigger(bcdev->buck_ic, OPLUS_IC_VIRQ_SVID);
		chg_info("pd_svooc = %d\n", bcdev->pd_svooc);
		break;
	case BC_VOOC_STATUS_GET:
		schedule_delayed_work(&bcdev->adsp_voocphy_status_work, 0);
		break;
	case BC_OTG_ENABLE:
		chg_info("enable otg\n");
		pst = &bcdev->psy_list[PSY_TYPE_USB];
		bcdev->otg_online = true;
		bcdev->pd_svooc = false;
		ret = oplus_chg_ic_virq_trigger(bcdev->buck_ic, OPLUS_IC_VIRQ_OTG_ENABLE);
		if (ret != -EAGAIN) {
			if (pst && is_usb_psy_available(bcdev))
				power_supply_changed(pst->psy);
		} else {
			schedule_work(&bcdev->wired_otg_enable_work);
		}
		break;
	case BC_OTG_DISABLE:
		chg_info("disable otg\n");
		pst = &bcdev->psy_list[PSY_TYPE_USB];
		bcdev->otg_online = false;
		oplus_chg_ic_virq_trigger(bcdev->buck_ic, OPLUS_IC_VIRQ_OTG_ENABLE);
		schedule_delayed_work(&bcdev->cid_status_change_work, msecs_to_jiffies(800));
		if (pst && is_usb_psy_available(bcdev)) {
			power_supply_changed(pst->psy);
		}
		bcdev->reverse_enable = false;
		oplus_chg_ic_virq_trigger(bcdev->reverse_chg_ic_dev, OPLUS_IC_VIRQ_REVERSE_ENABLE);
		break;
	case BC_VOOC_VBUS_ADC_ENABLE:
		chg_info("BC_VOOC_VBUS_ADC_ENABLE\n");
		bcdev->voocphy_err_check = true;
		cancel_delayed_work_sync(&bcdev->voocphy_err_work);
		schedule_delayed_work(&bcdev->voocphy_err_work, msecs_to_jiffies(8500));
		if (bcdev->is_external_chg) {
			/* excute in glink loop for real time */
			oplus_chg_disable_charger(true, FASTCHG_VOTER);
			oplus_chg_suspend_charger(true, FASTCHG_VOTER);
		} else {
			/* excute in work to avoid glink dead loop */
			schedule_delayed_work(&bcdev->vbus_adc_enable_work, 0);
		}
		break;
	case BC_CID_DETECT:
		chg_info("cid detect\n");
		schedule_delayed_work(&bcdev->cid_status_change_work, 0);
		oplus_chg_ic_virq_trigger(bcdev->buck_ic, OPLUS_IC_VIRQ_CC_DETECT);
		break;
	case BC_QC_DETECT:
		bcdev->hvdcp_detect_ok = true;
		break;
	case BC_TYPEC_STATE_CHANGE:
		oplus_chg_ic_virq_trigger(bcdev->buck_ic, OPLUS_IC_VIRQ_CC_CHANGED);
		oplus_chg_ic_virq_trigger(bcdev->buck_ic, OPLUS_IC_VIRQ_TYPEC_STATE);
		break;
	case BC_POWER_ROLE_STATUS:
		chg_info("BC_POWER_ROLE_STATUS\n");
		oplus_chg_ic_virq_trigger(bcdev->buck_ic, OPLUS_IC_VIRQ_POWER_ROLE_STATUS);
		break;
	case BC_PLUGIN_IRQ:
		chg_info("BC_PLUGIN_IRQ\n");
		schedule_delayed_work(&bcdev->plugin_irq_work, 0);
		break;
	case BC_APSD_DONE:
		bcdev->bc12_completed = true;
		chg_info("BC_APSD_DONE\n");
		break;
	case BC_CHG_STATUS_SET:
		chg_info("BC_CHG_STATUS_SET");
		schedule_delayed_work(&bcdev->unsuspend_usb_work, 0);
		break;
	case BC_UFCS_TEST_MODE_TRUE:
		bcdev->ufcs_test_mode = true;
		chg_info("ufcs test mode change = %d\n", bcdev->ufcs_test_mode);
		break;
	case BC_UFCS_TEST_MODE_FALSE:
		bcdev->ufcs_test_mode = false;
		chg_info("ufcs test mode change = %d\n", bcdev->ufcs_test_mode);
		break;
	case BC_UFCS_POWER_READY:
		bcdev->ufcs_power_ready = true;
		chg_info("ufcs power ready = %d\n", bcdev->ufcs_power_ready);
		break;
	case BC_UFCS_HANDSHAKE_OK:
		bcdev->ufcs_handshake_ok = true;
		chg_info("ufcs handshake ok = %d\n", bcdev->ufcs_handshake_ok);
		break;
	case BC_VOOC_GAN_MOS_ERROR:
		voocphy_push_gan_mos_err(bcdev->buck_ic);
		chg_err("gan_mos_err\n");
		break;
	case BC_UFCS_DISABLE_MOS:
		if (oplus_cpa_get_protocol_allow(bcdev) != CHG_PROTOCOL_VOOC) {
			chg_info("ufcs exit and disabe mos");
			plat_ufcs_send_state(PLAT_UFCS_NOTIFY_EXIT, NULL);
		}
		schedule_delayed_work(&bcdev->publish_close_cp_item_work, 0);
		break;
	case BC_UFCS_PDO_READY:
		bcdev->ufcs_pdo_ready = true;
		chg_info("ufcs pdo ready = %d\n", bcdev->ufcs_pdo_ready);
		break;
	case BC_UFCS_VERIFY_AUTH_READY:
		bcdev->ufcs_verify_auth_ready = true;
		break;
	case BC_UFCS_PWR_INFO_READY:
		bcdev->ufcs_power_info_ready = true;
		chg_info("ufcs power info ready = %d\n", bcdev->ufcs_power_info_ready);
		break;
	case BC_UFCS_VDM_EMARK_READY:
		bcdev->ufcs_vdm_emark_ready = true;
		chg_info("ufcs vnd emark ready = %d\n", bcdev->ufcs_vdm_emark_ready);
		break;
	case BC_BATTERY_RESET_START:
		if (oplus_chg_get_voocphy_support(bcdev) == ADSP_VOOCPHY &&
		    oplus_get_ufcs_charging(bcdev) && !bcdev->adspfg_i2c_reset_processing) {
			bcdev->adspfg_i2c_reset_processing = true;
			bcdev->adspfg_i2c_reset_notify_done = false;
			schedule_delayed_work(&bcdev->check_adspfg_status, 0);
		}
		break;
	case PD_SOURCECAP_DONE:
		chg_info("PD_SOURCECAP_DONE\n");
		if (oplus_chg_get_common_charge_icl_support_flags())
			schedule_delayed_work(&bcdev->sourcecap_done_work, 0);
		break;
	case REQUEST_QOS:
		chg_info("REQUEST_QOS\n");
		cancel_delayed_work(&bcdev->release_qos_work);
		schedule_delayed_work(&bcdev->request_qos_work, 0);
		break;
	case RELEASE_QOS:
		chg_info("RELEASE_QOS\n");
		cancel_delayed_work(&bcdev->request_qos_work);
		schedule_delayed_work(&bcdev->release_qos_work, 0);
		break;
	case HMAC_UPDATE:
		oplus_chg_ic_virq_trigger(bcdev->gauge_ic, OPLUS_IC_VIRQ_HMAC_UPDATE);
		break;
	case GAUGE_INITED:
		chg_info("GAUGE_INITED notify\n");
		cancel_delayed_work(&bcdev->gauge_register_work);
		schedule_delayed_work(&bcdev->gauge_register_work, 0);
		break;
	case UFCS_EXIT_MODE_NOTIFY:
		chg_info("ufcs reset notify\n");
		schedule_delayed_work(&bcdev->ufcs_reset_work, msecs_to_jiffies(500));
		break;
	case PD_CONNECT_HARD_RESET:
		chg_info("PD_CONNECT_HARD_RESET\n");
		oplus_chg_ic_virq_trigger(bcdev->reverse_chg_ic_dev, OPLUS_IC_VIRQ_HARD_RESET);
		break;
	case PD_PPS_CHECK_COMPLETED:
		chg_info("pd_check_completed %d usb_in_status %d\n", bcdev->pd_check_completed, bcdev->usb_in_status);
		if (!bcdev->pd_check_completed) {
			bcdev->pd_check_completed = true;
			oplus_chg_ic_virq_trigger(bcdev->buck_ic, OPLUS_IC_VIRQ_PD_COMPLETED);
		}
		break;
#endif
	default:
		break;
	}

	if (pst && pst->psy) {
		/*
		 * For charger mode, keep the device awake at least for 50 ms
		 * so that device won't enter suspend when a non-SDP charger
		 * is removed. This would allow the userspace process like
		 * "charger" to be able to read power supply uevents to take
		 * appropriate actions (e.g. shutting down when the charger is
		 * unplugged).
		 */
		pm_wakeup_dev_event(bcdev->dev, 50, true);
	}
}

#define VBUS_9V	9000
#define VBUS_5V	5000
static void oplus_chg_set_high_reverse_enable(struct oplus_chg_ic_dev *ic_dev)
{
	struct battery_chg_dev *bcdev;
	bool en = false;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (bcdev == NULL) {
		chg_err("bcdev is NULL");
		return;
	}

	if (bcdev->sink_req_volt > VBUS_5V)
		en = true;
	else
		en = false;

	if (bcdev->high_reverse_enable == en)
		return;

	chg_info("high_reverse_enable = %d\n", en);
	bcdev->high_reverse_enable = en;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_HIGH_REVERSE_ENABLE);
}

static void  handle_rechg_msg_handle(struct battery_chg_dev *bcdev, void *data,
				size_t len)
{
	struct reverse_chg_msg *resp_msg = data;
	u32 buf_len;

	chg_info("got the reverse chg msg_id=%d\n", resp_msg->msg_id);

	if (len > sizeof(bcdev->rvs_chg_msg_t)) {
		chg_err("Incorrect length received: %zu expected: %zd\n", len,
			sizeof(bcdev->rvs_chg_msg_t));
		complete(&bcdev->rvs_chg_msg_ack);
		return;
	}

	buf_len = resp_msg->data_size;
	if (buf_len > sizeof(bcdev->rvs_chg_msg_t.data_buffer)) {
		chg_err("Incorrect buffer length: %u\n", buf_len);
		complete(&bcdev->rvs_chg_msg_ack);
		return;
	}

	if (buf_len == 0) {
		chg_err("Incorrect buffer length: %u\n", buf_len);
		complete(&bcdev->rvs_chg_msg_ack);
		return;
	}
	chg_info("Buffer length: %d\n", buf_len);
	memcpy(bcdev->rvs_chg_msg_t.data_buffer, resp_msg->data_buffer, buf_len);

	switch (resp_msg->msg_id) {
	case RESERVED_CHG_MSG_GET_SINK_PDO:
		bcdev->rvs_msg_type = REVERSE_CHG_MSG_TYPE_SINK_REQ_PDO;
		bcdev->sink_req_volt = bcdev->rvs_chg_msg_t.data_buffer[0];
		bcdev->sink_req_curr = bcdev->rvs_chg_msg_t.data_buffer[1];
		chg_info("sink_req_volt = %d, sink_req_curr = %d\n", bcdev->sink_req_volt, bcdev->sink_req_curr);
		oplus_chg_ic_virq_trigger(bcdev->reverse_chg_ic_dev, OPLUS_IC_VIRQ_SINK_REQ_MSG);

		oplus_chg_set_high_reverse_enable(bcdev->reverse_chg_ic_dev);
		break;
	default:
		chg_err("got an undefined reverse chg message id\n");
		break;
	}
	complete(&bcdev->rvs_chg_msg_ack);
	return;
}


static int battery_chg_callback(void *priv, void *data, size_t len)
{
	struct pmic_glink_hdr *hdr = data;
	struct battery_chg_dev *bcdev = priv;

	if (!bcdev->is_chargepd_ready)
		bcdev->is_chargepd_ready = true;

	if (hdr->opcode == BC_NOTIFY_IND)
		handle_notification(bcdev, data, len);
#ifdef OPLUS_FEATURE_CHG_BASIC
	else if (hdr->opcode == OEM_OPCODE_READ_BUFFER)
		handle_oem_read_buffer(bcdev, data, len);
	else if (hdr->opcode == BCC_OPCODE_READ_BUFFER)
		handle_bcc_read_buffer(bcdev, data, len);
	else if (hdr->opcode == PPS_OPCODE_READ_BUFFER)
		handle_pps_read_buffer(bcdev, data, len);
	else if (hdr->opcode == AP_OPCODE_UFCS_BUFFER)
		handle_ufcs_read_buffer(bcdev, data, len);
	else if (hdr->opcode == AP_OPCODE_READ_BUFFER)
		handle_ap_read_buffer(bcdev, data, len);
	else if (hdr->opcode == AP_OPCODE_WRITE_BUFFER)
		handle_ap_write_buffer(bcdev, data, len);
	else if (hdr->opcode == OPLUS_OPCODE_SET_REQ ||
		 hdr->opcode == OPLUS_OPCODE_GET_REQ)
		oplus_handle_message(bcdev, data, len);
	else if (hdr->opcode == OPLUS_OPCODE_GET_SINK_MSG) {
		chg_info("OPLUS_OPCODE_GET_SINK_MSG handle\n");
		handle_rechg_msg_handle(bcdev, data, len);
	} else if (hdr->opcode == AP_OPCODE_PD_INFO_BUFFER) {
		chg_info("AP_OPCODE_PD_INFO_BUFFER handle\n");
		handle_pd_info_buffer(bcdev, data, len);
	}
#endif
	else
		handle_message(bcdev, data, len);

	return 0;
}

static int wls_psy_get_prop(struct power_supply *psy,
		enum power_supply_property prop,
		union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_WLS];
	int prop_id, rc;

	pval->intval = -ENODATA;

	prop_id = get_property_id(pst, prop);
	if (prop_id < 0)
		return prop_id;

	rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0)
		return rc;

	pval->intval = pst->prop[prop_id];

	return 0;
}

static int wls_psy_set_prop(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *pval)
{
	return 0;
}

static int wls_psy_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property prop)
{
	return 0;
}

static enum power_supply_property wls_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_TEMP,
#ifdef OPLUS_FEATURE_CHG_BASIC
	POWER_SUPPLY_PROP_PRESENT,
#endif
};

static const struct power_supply_desc wls_psy_desc = {
	.name			= "wireless",
	.type			= POWER_SUPPLY_TYPE_WIRELESS,
	.properties		= wls_props,
	.num_properties		= ARRAY_SIZE(wls_props),
	.get_property		= wls_psy_get_prop,
	.set_property		= wls_psy_set_prop,
	.property_is_writeable	= wls_psy_prop_is_writeable,
};

static const char *get_usb_type_name(u32 usb_type)
{
	u32 i;

	if (usb_type >= QTI_POWER_SUPPLY_USB_TYPE_HVDCP &&
	    usb_type <= QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3P5) {
		for (i = 0; i < ARRAY_SIZE(qc_power_supply_usb_type_text);
		     i++) {
			if (i == (usb_type - QTI_POWER_SUPPLY_USB_TYPE_HVDCP))
				return qc_power_supply_usb_type_text[i];
		}
		return "Unknown";
	}

	for (i = 0; i < ARRAY_SIZE(power_supply_usb_type_text); i++) {
		if (i == usb_type)
			return power_supply_usb_type_text[i];
	}

	return "Unknown";
}

#ifndef OPLUS_FEATURE_CHG_BASIC
static int usb_psy_set_icl(struct battery_chg_dev *bcdev, u32 prop_id, int val)
{
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	u32 temp;
	int rc;
	int adap_type = 0;

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_TYPE);
	else
		rc = read_property_id(bcdev, pst, USB_ADAP_TYPE);
	if (rc < 0)
		return rc;
	if (bcdev->soccp_support)
		adap_type = bcdev->oplus_psy.prop[OPLUS_USB_TYPE];
	else
		adap_type = pst->prop[USB_ADAP_TYPE];
	/* Allow this only for SDP or USB_PD and not for other charger types */
	if (adap_type != POWER_SUPPLY_USB_TYPE_SDP &&
	    adap_type != POWER_SUPPLY_USB_TYPE_PD)
		return -EINVAL;

	/*
	 * Input current limit (ICL) can be set by different clients. E.g. USB
	 * driver can request for a current of 500/900 mA depending on the
	 * port type. Also, clients like EUD driver can pass 0 or -22 to
	 * suspend or unsuspend the input for its use case.
	 */

	temp = val;
	if (val < 0)
		temp = UINT_MAX;
	if (bcdev->soccp_support)
		pst = &bcdev->oplus_psy;
	rc = write_property_id(bcdev, pst, prop_id, temp);
	if (!rc)
		pr_debug("Set ICL to %u\n", temp);

	return rc;
}
#endif /*OPLUS_FEATURE_CHG_BASIC*/

#ifdef OPLUS_FEATURE_CHG_BASIC
void oplus_chg_set_match_temp_ui_soc_to_voocphy(void)
{
	int rc = 0;
	struct battery_chg_dev *bcdev = g_bcdev;
	struct psy_state *pst = NULL;
	int match_temp = 0;
	int ui_soc = 0;

	if (!bcdev) {
		chg_err("bcdev is NULL!\n");
		return;
	}
	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	match_temp = oplus_chg_get_match_temp(bcdev);
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_SET_MATCH_TEMP, match_temp);
	else
		rc = write_property_id(bcdev, pst, BATT_SET_MATCH_TEMP, match_temp);
	if (rc) {
		chg_err("set match temp fail, rc=%d\n", rc);
		return;
	}
	ui_soc = oplus_chg_get_ui_soc(bcdev);
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_UI_SOC, ui_soc);
	else
		rc = write_property_id(bcdev, pst, BATT_UI_SOC, ui_soc);
	if (rc) {
		chg_err("set match temp fail, rc=%d\n", rc);
		return;
	}

	chg_err("ap set match temp[%d] ui_soc[%d] to voocphy\n", match_temp, ui_soc);
}

void oplus_chg_set_ap_fastchg_allow_to_voocphy(int allow)
{
	int rc = 0;
	struct battery_chg_dev *bcdev = g_bcdev;
	struct psy_state *pst = NULL;

	if (!bcdev) {
		chg_err("bcdev is NULL!\n");
		return;
	}
	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_AP_FASTCHG_ALLOW, allow);
	else
		rc = write_property_id(bcdev, pst, BATT_AP_FASTCHG_ALLOW, allow);
	if (rc) {
		chg_err("set_ap_fastchg_allow_to_voocphy, fail rc=%d\n", rc);
		return;
	}

	chg_err("set_ap_fastchg_allow_to_voocphy allow:%d\n", allow);
}

int oplus_set_bcc_curr_to_voocphy(struct oplus_chg_ic_dev *ic_dev, int *bcc_curr)
{
	int rc = 0;
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (bcc_curr == NULL) {
		chg_err("bcc_curr is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_SET_BCC_CURRENT, *bcc_curr);
	else
		rc = write_property_id(bcdev, pst, BATT_SET_BCC_CURRENT, *bcc_curr);
	if (rc) {
		chg_err("set bcc current fail, rc=%d\n", rc);
		return rc;
	}

	chg_debug("ap set bcc current[%d] to voocphy\n", *bcc_curr);
	return rc;
}
#endif /*OPLUS_FEATURE_CHG_BASIC*/

#ifdef OPLUS_FEATURE_CHG_BASIC

#ifdef OPLUS_CHG_UNDEF
static unsigned int get_chg_ctl_param_info(struct battery_chg_dev *bcdev)
{
	struct psy_state *pst = NULL;
	int rc = 0;
	int intval = 0;
	unsigned int project = 0, index = 0;

	pst = &bcdev->psy_list[PSY_TYPE_USB];
	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_VOOC_CHG_PARAM_INFO);
	else
		rc = read_property_id(bcdev, pst, USB_VOOC_CHG_PARAM_INFO);
	if (rc < 0) {
		chg_err("read USB_VOOC_CHG_PARAM_INFO fail\n");
		return 0;
	}
	if (bcdev->soccp_support)
		intval = bcdev->oplus_psy.prop[OPLUS_USB_VOOC_CHG_PARAM_INFO];
	else
		intval = pst->prop[USB_VOOC_CHG_PARAM_INFO];
	index = (intval & 0xFF);
	project = ((intval >> 8) & 0xFFFFFF);
	return (project * 100 + index);
}
#endif
#endif /*OPLUS_FEATURE_CHG_BASIC*/

static int usb_psy_get_prop(struct power_supply *psy,
		enum power_supply_property prop,
		union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int prop_id, rc;

	pval->intval = -ENODATA;

	prop_id = get_property_id(pst, prop);
	if (prop_id < 0)
		return prop_id;

	if (bcdev->soccp_support)
		return 0;
	else
		rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0)
		return rc;

	pval->intval = pst->prop[prop_id];
	if (prop == POWER_SUPPLY_PROP_TEMP)
		pval->intval = DIV_ROUND_CLOSEST((int)pval->intval, 10);

	return 0;
}

#ifdef OPLUS_FEATURE_CHG_BASIC
#ifdef OPLUS_CHG_UNDEF /* TODO */
int oplus_get_fast_chg_type(void)
{
	int fast_chg_type = 0;

	fast_chg_type = oplus_vooc_get_fast_chg_type();
	if (fast_chg_type == 0) {
		fast_chg_type = oplus_chg_get_charger_subtype();
	}
	if (fast_chg_type == 0) {
		if (oplus_wpc_get_adapter_type() == CHARGER_SUBTYPE_FASTCHG_VOOC
			|| oplus_wpc_get_adapter_type() == CHARGER_SUBTYPE_FASTCHG_SVOOC)
			fast_chg_type = oplus_wpc_get_adapter_type();
	}

	return fast_chg_type;
}
#endif
#endif

static int usb_psy_set_prop(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int prop_id, rc = 0;

	prop_id = get_property_id(pst, prop);
	if (prop_id < 0)
		return prop_id;

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
#ifndef OPLUS_FEATURE_CHG_BASIC
	if (bcdev->soccp_support)
		rc = usb_psy_set_icl(bcdev, &bcdev->oplus_psy, OPLUS_USB_INPUT_CURRENT_LIMIT, pval->intval);
	else
		rc = usb_psy_set_icl(bcdev, prop_id, pval->intval);
#endif
		break;
	default:
		break;
	}

	return rc;
}

static int usb_psy_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return 1;
	default:
		break;
	}

	return 0;
}

static enum power_supply_property usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_TEMP,
};

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
static enum power_supply_usb_type usb_psy_supported_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_ACA,
	POWER_SUPPLY_USB_TYPE_C,
	POWER_SUPPLY_USB_TYPE_PD,
	POWER_SUPPLY_USB_TYPE_PD_DRP,
	POWER_SUPPLY_USB_TYPE_PD_PPS,
	POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID,
};
#endif

static struct power_supply_desc usb_psy_desc = {
	.name			= "usb",
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= usb_props,
	.num_properties		= ARRAY_SIZE(usb_props),
	.get_property		= usb_psy_get_prop,
	.set_property		= usb_psy_set_prop,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	.usb_types		= usb_psy_supported_types,
	.num_usb_types		= ARRAY_SIZE(usb_psy_supported_types),
#else
	.usb_types		= BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
				  BIT(POWER_SUPPLY_USB_TYPE_SDP)     |
				  BIT(POWER_SUPPLY_USB_TYPE_DCP)     |
				  BIT(POWER_SUPPLY_USB_TYPE_CDP)     |
				  BIT(POWER_SUPPLY_USB_TYPE_ACA)     |
				  BIT(POWER_SUPPLY_USB_TYPE_C)       |
				  BIT(POWER_SUPPLY_USB_TYPE_PD)      |
				  BIT(POWER_SUPPLY_USB_TYPE_PD_DRP)  |
				  BIT(POWER_SUPPLY_USB_TYPE_PD_PPS)  |
				  BIT(POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID),
#endif
	.property_is_writeable	= usb_psy_prop_is_writeable,
};

static int __battery_psy_set_charge_current(struct battery_chg_dev *bcdev,
					u32 fcc_ua)
{
	int rc;

	if (bcdev->restrict_chg_en) {
		fcc_ua = min_t(u32, fcc_ua, bcdev->restrict_fcc_ua);
		fcc_ua = min_t(u32, fcc_ua, bcdev->thermal_fcc_ua);
	}

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_BATTERY],
				BATT_CHG_CTRL_LIM, fcc_ua);
	if (rc < 0)
		chg_err("Failed to set FCC %u, rc=%d\n", fcc_ua, rc);
	else
		pr_debug("Set FCC to %u uA\n", fcc_ua);

	return rc;
}

static int battery_psy_set_charge_current(struct battery_chg_dev *bcdev,
					int val)
{
	int rc;
	u32 fcc_ua, prev_fcc_ua;

	if (!bcdev->num_thermal_levels)
		return 0;

	if (bcdev->num_thermal_levels < 0) {
		chg_err("Incorrect num_thermal_levels\n");
		return -EINVAL;
	}

	if (val < 0 || val > bcdev->num_thermal_levels)
		return -EINVAL;

	fcc_ua = bcdev->thermal_levels[val];
	prev_fcc_ua = bcdev->thermal_fcc_ua;
	bcdev->thermal_fcc_ua = fcc_ua;

	rc = __battery_psy_set_charge_current(bcdev, fcc_ua);
	if (!rc)
		bcdev->curr_thermal_level = val;
	else
		bcdev->thermal_fcc_ua = prev_fcc_ua;

	return rc;
}

static int battery_psy_get_prop(struct power_supply *psy,
		enum power_supply_property prop,
		union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int prop_id, rc;

	pval->intval = -ENODATA;

	prop_id = get_property_id(pst, prop);
	if (prop_id < 0)
		return prop_id;
	if (bcdev->soccp_support)
		return 0;
	else
		rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0)
		return rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_MODEL_NAME:
		pval->strval = pst->model;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		pval->intval = DIV_ROUND_CLOSEST(pst->prop[prop_id], 100);
		if (IS_ENABLED(CONFIG_QTI_PMIC_GLINK_CLIENT_DEBUG) &&
		   (bcdev->fake_soc >= 0 && bcdev->fake_soc <= 100))
			pval->intval = bcdev->fake_soc;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		pval->intval = DIV_ROUND_CLOSEST((int)pst->prop[prop_id], 10);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		pval->intval = bcdev->curr_thermal_level;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		pval->intval = bcdev->num_thermal_levels;
		break;
	default:
		pval->intval = pst->prop[prop_id];
		break;
	}

	return rc;
}

static int battery_psy_set_prop(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		return battery_psy_set_charge_current(bcdev, pval->intval);
	default:
		return -EINVAL;
	}

	return 0;
}

static int battery_psy_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		return 1;
	default:
		break;
	}

	return 0;
}

static enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_POWER_AVG,
#ifdef OPLUS_FEATURE_CHG_BASIC
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
#endif
};

static const struct power_supply_desc batt_psy_desc = {
	.name			= "battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.properties		= battery_props,
	.num_properties		= ARRAY_SIZE(battery_props),
	.get_property		= battery_psy_get_prop,
	.set_property		= battery_psy_set_prop,
	.property_is_writeable	= battery_psy_prop_is_writeable,
};

__maybe_unused static int battery_chg_init_psy(struct battery_chg_dev *bcdev)
{
	struct power_supply_config psy_cfg = {};
	int rc;

	psy_cfg.drv_data = bcdev;
	psy_cfg.of_node = bcdev->dev->of_node;
	bcdev->psy_list[PSY_TYPE_BATTERY].psy =
		devm_power_supply_register(bcdev->dev, &batt_psy_desc,
						&psy_cfg);
	if (IS_ERR(bcdev->psy_list[PSY_TYPE_BATTERY].psy)) {
		rc = PTR_ERR(bcdev->psy_list[PSY_TYPE_BATTERY].psy);
		chg_err("Failed to register battery power supply, rc=%d\n", rc);
		return rc;
	}

	bcdev->psy_list[PSY_TYPE_USB].psy =
		devm_power_supply_register(bcdev->dev, &usb_psy_desc, &psy_cfg);
	if (IS_ERR(bcdev->psy_list[PSY_TYPE_USB].psy)) {
		rc = PTR_ERR(bcdev->psy_list[PSY_TYPE_USB].psy);
		chg_err("Failed to register USB power supply, rc=%d\n", rc);
		return rc;
	}

	bcdev->psy_list[PSY_TYPE_WLS].psy =
		devm_power_supply_register(bcdev->dev, &wls_psy_desc, &psy_cfg);
	if (IS_ERR(bcdev->psy_list[PSY_TYPE_WLS].psy)) {
		rc = PTR_ERR(bcdev->psy_list[PSY_TYPE_WLS].psy);
		chg_err("Failed to register wireless power supply, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int wireless_fw_send_firmware(struct battery_chg_dev *bcdev,
					const struct firmware *fw)
{
	struct wireless_fw_push_buf_req msg = {};
	const u8 *ptr;
	u32 i, num_chunks, partial_chunk_size;
	int rc;

	num_chunks = fw->size / WLS_FW_BUF_SIZE;
	partial_chunk_size = fw->size % WLS_FW_BUF_SIZE;

	if (!num_chunks)
		return -EINVAL;

	pr_debug("Updating FW...\n");

	ptr = fw->data;
	msg.hdr.owner = MSG_OWNER_BC;
	msg.hdr.type = MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = BC_WLS_FW_PUSH_BUF_REQ;

	for (i = 0; i < num_chunks; i++, ptr += WLS_FW_BUF_SIZE) {
		msg.fw_chunk_id = i + 1;
		memcpy(msg.buf, ptr, WLS_FW_BUF_SIZE);

		pr_debug("sending FW chunk %u\n", i + 1);
		rc = battery_chg_fw_write(bcdev, &msg, sizeof(msg));
		if (rc < 0)
			return rc;
	}

	if (partial_chunk_size) {
		msg.fw_chunk_id = i + 1;
		memset(msg.buf, 0, WLS_FW_BUF_SIZE);
		memcpy(msg.buf, ptr, partial_chunk_size);

		pr_debug("sending partial FW chunk %u\n", i + 1);
		rc = battery_chg_fw_write(bcdev, &msg, sizeof(msg));
		if (rc < 0)
			return rc;
	}

	return 0;
}

static int wireless_fw_check_for_update(struct battery_chg_dev *bcdev,
					u32 version, size_t size)
{
	struct wireless_fw_check_req req_msg = {};

	bcdev->wls_fw_update_reqd = false;

	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = BC_WLS_FW_CHECK_UPDATE;
	req_msg.fw_version = version;
	req_msg.fw_size = size;
	req_msg.fw_crc = bcdev->wls_fw_crc;

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

#define IDT_FW_MAJOR_VER_OFFSET		0x94
#define IDT_FW_MINOR_VER_OFFSET		0x96
static int wireless_fw_update(struct battery_chg_dev *bcdev, bool force)
{
	const struct firmware *fw;
	struct psy_state *pst;
	u32 version;
	u16 maj_ver, min_ver;
	int rc;
	int online = 0;
	int capacity = 0;

	pm_stay_awake(bcdev->dev);

	/*
	 * Check for USB presence. If nothing is connected, check whether
	 * battery SOC is at least 50% before allowing FW update.
	 */
	pst = &bcdev->psy_list[PSY_TYPE_USB];
	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_ONLINE);
	else
		rc = read_property_id(bcdev, pst, USB_ONLINE);
	if (rc < 0)
		goto out;
	if (bcdev->soccp_support)
		online = bcdev->oplus_psy.prop[OPLUS_USB_ONLINE];
	else
		online = pst->prop[USB_ONLINE];

	if (online) {
		pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
		if (bcdev->soccp_support)
			rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_CAPACITY);
		else
			rc = read_property_id(bcdev, pst, BATT_CAPACITY);
		if (rc < 0)
			goto out;
		if (bcdev->soccp_support)
			capacity = bcdev->oplus_psy.prop[OPLUS_CAPACITY];
		else
			capacity = pst->prop[BATT_CAPACITY];

		if ((capacity / 100) < 50) {
			chg_err("Battery SOC should be at least 50%% or connect charger\n");
			rc = -EINVAL;
			goto out;
		}
	}

	rc = firmware_request_nowarn(&fw, bcdev->wls_fw_name, bcdev->dev);
	if (rc) {
		chg_err("Couldn't get firmware rc=%d\n", rc);
		goto out;
	}

	if (!fw || !fw->data || !fw->size) {
		chg_err("Invalid firmware\n");
		rc = -EINVAL;
		goto release_fw;
	}

	if (fw->size < SZ_16K) {
		chg_err("Invalid firmware size %zu\n", fw->size);
		rc = -EINVAL;
		goto release_fw;
	}

	maj_ver = le16_to_cpu(*(__le16 *)(fw->data + IDT_FW_MAJOR_VER_OFFSET));
	min_ver = le16_to_cpu(*(__le16 *)(fw->data + IDT_FW_MINOR_VER_OFFSET));
	version = maj_ver << 16 | min_ver;

	if (force)
		version = UINT_MAX;

	pr_debug("FW size: %zu version: %#x\n", fw->size, version);

	rc = wireless_fw_check_for_update(bcdev, version, fw->size);
	if (rc < 0) {
		chg_err("Wireless FW update not needed, rc=%d\n", rc);
		goto release_fw;
	}

	if (!bcdev->wls_fw_update_reqd) {
		pr_warn("Wireless FW update not required\n");
		goto release_fw;
	}

	/* Wait for IDT to be setup by charger firmware */
	msleep(WLS_FW_PREPARE_TIME_MS);

	reinit_completion(&bcdev->fw_update_ack);
	rc = wireless_fw_send_firmware(bcdev, fw);
	if (rc < 0) {
		chg_err("Failed to send FW chunk, rc=%d\n", rc);
		goto release_fw;
	}

	rc = wait_for_completion_timeout(&bcdev->fw_update_ack,
				msecs_to_jiffies(WLS_FW_WAIT_TIME_MS));
	if (!rc) {
		chg_err("Error, timed out updating firmware\n");
		rc = -ETIMEDOUT;
		goto release_fw;
	} else {
		rc = 0;
	}

	chg_info("Wireless FW update done\n");

release_fw:
	release_firmware(fw);
out:
	pm_relax(bcdev->dev);

	return rc;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0))
static ssize_t wireless_fw_version_show(const struct class *c,
					const struct class_attribute *attr,
					char *buf)
#else
static ssize_t wireless_fw_version_show(struct class *c,
					struct class_attribute *attr,
					char *buf)
#endif
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct wireless_fw_get_version_req req_msg = {};
	int rc;

	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = BC_WLS_FW_GET_VERSION;

	rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		chg_err("Failed to get FW version rc=%d\n", rc);
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "%#x\n", bcdev->wls_fw_version);
}
static CLASS_ATTR_RO(wireless_fw_version);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0))
static ssize_t wireless_fw_force_update_store(const struct class *c,
					const struct class_attribute *attr,
					const char *buf, size_t count)
#else
static ssize_t wireless_fw_force_update_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
#endif
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	bool val;
	int rc;

	if (kstrtobool(buf, &val) || !val)
		return -EINVAL;

	rc = wireless_fw_update(bcdev, true);
	if (rc < 0)
		return rc;

	return count;
}
static CLASS_ATTR_WO(wireless_fw_force_update);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0))
static ssize_t wireless_fw_update_store(const struct class *c,
					const struct class_attribute *attr,
					const char *buf, size_t count)
#else
static ssize_t wireless_fw_update_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
#endif
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	bool val;
	int rc;

	if (kstrtobool(buf, &val) || !val)
		return -EINVAL;

	rc = wireless_fw_update(bcdev, false);
	if (rc < 0)
		return rc;

	return count;
}
static CLASS_ATTR_WO(wireless_fw_update);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0))
static ssize_t usb_typec_compliant_show(const struct class *c,
				const struct class_attribute *attr, char *buf)
#else
static ssize_t usb_typec_compliant_show(struct class *c,
				struct class_attribute *attr, char *buf)
#endif
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc;

	rc = read_property_id(bcdev, pst, USB_TYPEC_COMPLIANT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			(int)pst->prop[USB_TYPEC_COMPLIANT]);
}
static CLASS_ATTR_RO(usb_typec_compliant);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0))
static ssize_t usb_real_type_show(const struct class *c,
				const struct class_attribute *attr, char *buf)
#else
static ssize_t usb_real_type_show(struct class *c,
				struct class_attribute *attr, char *buf)
#endif
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc;

	rc = read_property_id(bcdev, pst, USB_REAL_TYPE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%s\n",
			get_usb_type_name(pst->prop[USB_REAL_TYPE]));
}
static CLASS_ATTR_RO(usb_real_type);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0))
static ssize_t restrict_cur_store(const struct class *c,
				const struct class_attribute *attr,
				const char *buf, size_t count)
#else
static ssize_t restrict_cur_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
#endif
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	u32 fcc_ua, prev_fcc_ua;

	if (kstrtou32(buf, 0, &fcc_ua) || fcc_ua > bcdev->thermal_fcc_ua)
		return -EINVAL;

	prev_fcc_ua = bcdev->restrict_fcc_ua;
	bcdev->restrict_fcc_ua = fcc_ua;
	if (bcdev->restrict_chg_en) {
		rc = __battery_psy_set_charge_current(bcdev, fcc_ua);
		if (rc < 0) {
			bcdev->restrict_fcc_ua = prev_fcc_ua;
			return rc;
		}
	}

	return count;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0))
static ssize_t restrict_cur_show(const struct class *c,
				const struct class_attribute *attr,
				char *buf)
#else
static ssize_t restrict_cur_show(struct class *c, struct class_attribute *attr,
				char *buf)
#endif
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%u\n", bcdev->restrict_fcc_ua);
}
static CLASS_ATTR_RW(restrict_cur);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0))
static ssize_t restrict_chg_store(const struct class *c,
				const struct class_attribute *attr,
				const char *buf, size_t count)
#else
static ssize_t restrict_chg_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
#endif
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	bcdev->restrict_chg_en = val;
	rc = __battery_psy_set_charge_current(bcdev, bcdev->restrict_chg_en ?
			bcdev->restrict_fcc_ua : bcdev->thermal_fcc_ua);
	if (rc < 0)
		return rc;

	return count;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0))
static ssize_t restrict_chg_show(const struct class *c,
				const struct class_attribute *attr,
				char *buf)
#else
static ssize_t restrict_chg_show(struct class *c, struct class_attribute *attr,
				char *buf)
#endif
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->restrict_chg_en);
}
static CLASS_ATTR_RW(restrict_chg);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0))
static ssize_t fake_soc_store(const struct class *c,
				const struct class_attribute *attr,
				const char *buf, size_t count)
#else
static ssize_t fake_soc_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
#endif
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	bcdev->fake_soc = val;
	pr_debug("Set fake soc to %d\n", val);

	if (IS_ENABLED(CONFIG_QTI_PMIC_GLINK_CLIENT_DEBUG) && is_batt_psy_available(bcdev))
		power_supply_changed(pst->psy);

	return count;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0))
static ssize_t fake_soc_show(const struct class *c,
				const struct class_attribute *attr,
				char *buf)
#else
static ssize_t fake_soc_show(struct class *c, struct class_attribute *attr,
				char *buf)
#endif
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->fake_soc);
}
static CLASS_ATTR_RW(fake_soc);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0))
static ssize_t wireless_boost_en_store(const struct class *c,
					const struct class_attribute *attr,
					const char *buf, size_t count)
#else
static ssize_t wireless_boost_en_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
#endif
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev,  &bcdev->oplus_psy,
				OPLUS_DC_BOOST_EN, val);
	else
		rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_WLS],
				WLS_BOOST_EN, val);
	if (rc < 0)
		return rc;

	return count;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0))
static ssize_t wireless_boost_en_show(const struct class *c,
					const struct class_attribute *attr,
					char *buf)
#else
static ssize_t wireless_boost_en_show(struct class *c,
					struct class_attribute *attr, char *buf)
#endif
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_WLS];
	int rc;

	rc = read_property_id(bcdev, pst, WLS_BOOST_EN);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[WLS_BOOST_EN]);
}
static CLASS_ATTR_RW(wireless_boost_en);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0))
static ssize_t moisture_detection_en_store(const struct class *c,
					const struct class_attribute *attr,
					const char *buf, size_t count)
#else
static ssize_t moisture_detection_en_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
#endif
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_USB],
				USB_MOISTURE_DET_EN, val);
	if (rc < 0)
		return rc;

	return count;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0))
static ssize_t moisture_detection_en_show(const struct class *c,
					const struct class_attribute *attr,
					char *buf)
#else
static ssize_t moisture_detection_en_show(struct class *c,
					struct class_attribute *attr, char *buf)
#endif
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc;

	rc = read_property_id(bcdev, pst, USB_MOISTURE_DET_EN);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			pst->prop[USB_MOISTURE_DET_EN]);
}
static CLASS_ATTR_RW(moisture_detection_en);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0))
static ssize_t moisture_detection_status_show(const struct class *c,
					const struct class_attribute *attr,
					char *buf)
#else
static ssize_t moisture_detection_status_show(struct class *c,
					struct class_attribute *attr, char *buf)
#endif
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc;

	rc = read_property_id(bcdev, pst, USB_MOISTURE_DET_STS);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			pst->prop[USB_MOISTURE_DET_STS]);
}
static CLASS_ATTR_RO(moisture_detection_status);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0))
static ssize_t resistance_show(const struct class *c,
					const struct class_attribute *attr,
					char *buf)
#else
static ssize_t resistance_show(struct class *c,
					struct class_attribute *attr, char *buf)
#endif
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int rc;

	rc = read_property_id(bcdev, pst, BATT_RESISTANCE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[BATT_RESISTANCE]);
}
static CLASS_ATTR_RO(resistance);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0))
static ssize_t soh_show(const struct class *c,
			const struct class_attribute *attr, char *buf)
#else
static ssize_t soh_show(struct class *c, struct class_attribute *attr,
			char *buf)
#endif
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int rc;

	rc = read_property_id(bcdev, pst, BATT_SOH);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[BATT_SOH]);
}
static CLASS_ATTR_RO(soh);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0))
static ssize_t ship_mode_en_store(const struct class *c,
				const struct class_attribute *attr,
				const char *buf, size_t count)
#else
static ssize_t ship_mode_en_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
#endif
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	if (kstrtobool(buf, &bcdev->ship_mode_en))
		return -EINVAL;

	return count;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0))
static ssize_t ship_mode_en_show(const struct class *c,
				const struct class_attribute *attr, char *buf)
#else
static ssize_t ship_mode_en_show(struct class *c, struct class_attribute *attr,
				char *buf)
#endif
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->ship_mode_en);
}
static CLASS_ATTR_RW(ship_mode_en);

#ifdef OPLUS_FEATURE_CHG_BASIC
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0))
static ssize_t uart_log_store(const struct class *c,
				const struct class_attribute *attr,
				const char *buf, size_t count)
#else
static ssize_t uart_log_store(struct class *c,
				struct class_attribute *attr,
				const char *buf, size_t count)
#endif
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	u32 val = 0;
	int rc;

	if (!bcdev->soccp_support)
		return -ENOTSUPP;

	if (count > 0 && buf[0] == '1')
		val = 1;

	rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_SET_UART_LOG_ENABLE, val);
	if (rc < 0)
		return rc;

	return count;
}
static CLASS_ATTR_WO(uart_log);
#endif

static struct attribute *battery_class_attrs[] = {
	&class_attr_soh.attr,
	&class_attr_resistance.attr,
	&class_attr_moisture_detection_status.attr,
	&class_attr_moisture_detection_en.attr,
	&class_attr_wireless_boost_en.attr,
	&class_attr_fake_soc.attr,
	&class_attr_wireless_fw_update.attr,
	&class_attr_wireless_fw_force_update.attr,
	&class_attr_wireless_fw_version.attr,
	&class_attr_ship_mode_en.attr,
	&class_attr_restrict_chg.attr,
	&class_attr_restrict_cur.attr,
	&class_attr_usb_real_type.attr,
	&class_attr_usb_typec_compliant.attr,
#ifdef OPLUS_FEATURE_CHG_BASIC
	&class_attr_uart_log.attr,
#endif
	NULL,
};
ATTRIBUTE_GROUPS(battery_class);

#ifdef CONFIG_DEBUG_FS
static void battery_chg_add_debugfs(struct battery_chg_dev *bcdev)
{
	int rc;
	struct dentry *dir;

	dir = debugfs_create_dir("battery_charger", NULL);
	if (IS_ERR(dir)) {
		rc = PTR_ERR(dir);
		chg_err("Failed to create charger debugfs directory, rc=%d\n",
			rc);
		return;
	}

	debugfs_create_bool("block_tx", 0600, dir, &bcdev->block_tx);
	bcdev->debugfs_dir = dir;

	return;
}
#else
static void battery_chg_add_debugfs(struct battery_chg_dev *bcdev) { }
#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
static bool oplus_vchg_trig_is_support(void)
{
	struct battery_chg_dev *bcdev = g_bcdev;

	if (!bcdev) {
		chg_err("bcdev is NULL!\n");
		return false;
	}
	if (bcdev->oplus_custom_gpio.vchg_trig_gpio <= 0)
		return false;
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if (get_PCB_Version() >= EVT1)
		return true;
#endif
	return false;
}

static int oplus_vchg_trig_gpio_init(struct battery_chg_dev *bcdev)
{
	if (!bcdev) {
		chg_err("bcdev not ready\n");
		return -EINVAL;
	}

	bcdev->oplus_custom_gpio.vchg_trig_pinctrl = devm_pinctrl_get(bcdev->dev);

	bcdev->oplus_custom_gpio.vchg_trig_default =
		pinctrl_lookup_state(bcdev->oplus_custom_gpio.vchg_trig_pinctrl, "vchg_trig_default");
	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.vchg_trig_default)) {
		chg_err("get vchg_trig_default\n");
		return -EINVAL;
	}

	if (bcdev->oplus_custom_gpio.vchg_trig_gpio > 0) {
		gpio_direction_input(bcdev->oplus_custom_gpio.vchg_trig_gpio);
	}
	pinctrl_select_state(bcdev->oplus_custom_gpio.vchg_trig_pinctrl,
		bcdev->oplus_custom_gpio.vchg_trig_default);

	chg_err("get vchg_trig_default level[%d]\n", gpio_get_value(bcdev->oplus_custom_gpio.vchg_trig_gpio));
	return 0;
}

static int oplus_get_vchg_trig_gpio_val(void)
{
	int level = 1;
	static int pre_level = 1;
	struct battery_chg_dev *bcdev = g_bcdev;

	if (!bcdev) {
		chg_err("chip is NULL!\n");
		return -1;
	}

	if (bcdev->oplus_custom_gpio.vchg_trig_gpio <= 0) {
		chg_err("vchg_trig_gpio not exist, return\n");
		return -1;
	}

	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.vchg_trig_pinctrl)
			|| IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.vchg_trig_default)) {
		chg_err("pinctrl null, return\n");
		return -1;
	}

	level = gpio_get_value(bcdev->oplus_custom_gpio.vchg_trig_gpio);
	if (pre_level ^ level) {
		pre_level = level;
		chg_err("!!!!! vchg_trig gpio level[%d], wired[%d]\n", level, !level);
	}
	return level;
}

static int vchg_trig_status = -1;
static int oplus_get_vchg_trig_status(void)
{
	if (vchg_trig_status == -1) {
		vchg_trig_status = !!oplus_get_vchg_trig_gpio_val();
	}
	return vchg_trig_status;
}

static void oplus_vchg_trig_work(struct work_struct *work)
{
#ifdef OPLUS_CHG_UNDEF /* TODO */
	int level;
	static bool pre_otg = false;
	struct battery_chg_dev *bcdev = g_bcdev;
	struct oplus_chg_chip *chip = get_oplus_chg_chip();

	if (!chip || !bcdev) {
		chg_err("chip or bcdev is NULL!\n");
		return;
	}

	level = oplus_get_vchg_trig_gpio_val();
	vchg_trig_status = !!level;
	if (level == 0) {
		if (bcdev->otg_online == true) {
			pre_otg = true;
			return;
		}
		if (chip->wireless_support)
			oplus_switch_to_wired_charge(bcdev);
	} else {
		if (pre_otg == true) {
			pre_otg = false;
			return;
		}
		if (chip->wireless_support
			&& chip->voocphy.fastchg_to_warm == false
			&& chip->voocphy.fastchg_to_normal == false)
			oplus_switch_from_wired_charge(bcdev);
	}

	if (chip->voocphy.fastchg_to_warm == false
		&& chip->voocphy.fastchg_to_normal == false) {
		oplus_chg_wake_update_work();
	}
#endif
}

static void oplus_vchg_trig_irq_init(struct battery_chg_dev *bcdev)
{
	if (!bcdev) {
		chg_err("bcdev not ready\n");
		return;
	}

	bcdev->vchg_trig_irq = gpio_to_irq(bcdev->oplus_custom_gpio.vchg_trig_gpio);
	chg_info("vchg_trig_irq[%d]\n", bcdev->vchg_trig_irq);
}

#define VCHG_TRIG_DELAY_MS	50
static irqreturn_t oplus_vchg_trig_change_handler(int irq, void *data)
{
	struct battery_chg_dev *bcdev = data;

	cancel_delayed_work_sync(&bcdev->vchg_trig_work);
	chg_info("scheduling vchg_trig work\n");
	schedule_delayed_work(&bcdev->vchg_trig_work, msecs_to_jiffies(VCHG_TRIG_DELAY_MS));

	return IRQ_HANDLED;
}

static void oplus_vchg_trig_irq_register(struct battery_chg_dev *bcdev)
{
	int ret = 0;

	if (!bcdev) {
		chg_err("bcdev not ready\n");
		return;
	}

	ret = devm_request_threaded_irq(bcdev->dev, bcdev->vchg_trig_irq,
			NULL, oplus_vchg_trig_change_handler, IRQF_TRIGGER_FALLING
			| IRQF_TRIGGER_RISING | IRQF_ONESHOT, "vchg_trig_change", bcdev);
	if (ret < 0)
		chg_err("Unable to request vchg_trig_change irq: %d\n", ret);

	ret = enable_irq_wake(bcdev->vchg_trig_irq);
	if (ret != 0)
		chg_err("enable_irq_wake: vchg_trig_irq failed %d\n", ret);
}
#endif /*OPLUS_FEATURE_CHG_BASIC*/

#ifdef OPLUS_FEATURE_CHG_BASIC
static void smbchg_enter_shipmode_pmic(struct battery_chg_dev *bcdev)
{
	int rc = 0;
	struct psy_state *pst = NULL;

	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_SET_SHIP_MODE, 1);
	else
		rc = write_property_id(bcdev, pst, BATT_SET_SHIP_MODE, 1);
	if (rc) {
		chg_err("set ship mode fail, rc=%d\n", rc);
		return;
	}
	chg_debug("power off after 15s\n");
}

static int oplus_shaft_fpc_iio_init(struct battery_chg_dev *bcdev)
{
	int rc = 0;

	atomic_set(&bcdev->is_shaft_btb_over, 0);
	rc = of_property_match_string(bcdev->dev->of_node, "io-channel-names",
				      "shaft_fpc_therm_adc");
	if (rc >= 0) {
		bcdev->iio.shaft_btb_temp_chan = iio_channel_get(bcdev->dev,
								"shaft_fpc_therm_adc");
		if (IS_ERR(bcdev->iio.shaft_btb_temp_chan)) {
			rc = PTR_ERR(bcdev->iio.shaft_btb_temp_chan);
			if (rc != -EPROBE_DEFER)
				chg_err("shaft_fpc_therm_adc get error, %d\n", rc);
			bcdev->iio.shaft_btb_temp_chan = NULL;
			return rc;
		}
	} else {
		chg_info("no support shaft_fpc_therm_adc \n");
	}

	return rc;
}

static int oplus_subboard_temp_iio_init(struct battery_chg_dev *bcdev)
{
	int rc = 0;

	rc = of_property_match_string(bcdev->dev->of_node, "io-channel-names",
				      "subboard_temp_adc");
	if (rc >= 0) {
		bcdev->iio.subboard_temp_chan = iio_channel_get(bcdev->dev,
								"subboard_temp_adc");
		if (IS_ERR(bcdev->iio.subboard_temp_chan)) {
			rc = PTR_ERR(bcdev->iio.subboard_temp_chan);
			if (rc != -EPROBE_DEFER)
				chg_err("subboard_temp_chan get error, %d\n", rc);
			bcdev->iio.subboard_temp_chan = NULL;
			return rc;
		}
	} else
		chg_err("can't find subboard_temp_adc node\n");

	rc = of_property_match_string(bcdev->dev->of_node, "io-channel-names",
				      "batt_con_therm_adc");
	if (rc >= 0) {
		bcdev->iio.batt_con_btb_chan = iio_channel_get(bcdev->dev,
								"batt_con_therm_adc");
		if (IS_ERR(bcdev->iio.batt_con_btb_chan)) {
			rc = PTR_ERR(bcdev->iio.batt_con_btb_chan);
			if (rc != -EPROBE_DEFER)
				chg_err("batt_con_btb_chan get error, %d\n", rc);
			bcdev->iio.batt_con_btb_chan = NULL;
			return rc;
		}
		bcdev->iio.sub_batt_con_btb_chan = iio_channel_get(bcdev->dev,
								"sub_batt_con_therm_adc");
		if (IS_ERR(bcdev->iio.sub_batt_con_btb_chan)) {
			rc = PTR_ERR(bcdev->iio.sub_batt_con_btb_chan);
			bcdev->iio.sub_batt_con_btb_chan = NULL;
		} else {
			chg_err("sub_batt_con_btb_chan get OK, %d\n", rc);
		}
	} else {
		chg_err("can't find batt_con_btb_chan node\n");
	}

	rc = of_property_match_string(bcdev->dev->of_node, "io-channel-names",
				      "usb_con_therm_adc");
	if (rc >= 0) {
		bcdev->iio.usb_con_btb_chan = iio_channel_get(bcdev->dev,
								"usb_con_therm_adc");
		if (IS_ERR(bcdev->iio.usb_con_btb_chan)) {
			rc = PTR_ERR(bcdev->iio.usb_con_btb_chan);
			if (rc != -EPROBE_DEFER)
				chg_err("usb_con_btb_chan get error, %d\n", rc);
			bcdev->iio.usb_con_btb_chan = NULL;
			return rc;
		}
	} else {
		chg_err("can't find usb_con_btb_chan node\n");
	}

	rc = of_property_match_string(bcdev->dev->of_node, "io-channel-names",
				      "svooc_mos_btb_therm");
	if (rc >= 0) {
		bcdev->iio.chg_mos_temp_chan = iio_channel_get(bcdev->dev,
								"svooc_mos_btb_therm");
		if (IS_ERR(bcdev->iio.chg_mos_temp_chan)) {
			rc = PTR_ERR(bcdev->iio.chg_mos_temp_chan);
			if (rc != -EPROBE_DEFER)
				chg_err("chg_mos_temp_chan get error, %d\n", rc);
			bcdev->iio.chg_mos_temp_chan = NULL;
			return rc;
		}
	} else {
		chg_err("can't find chg_mos_temp_chan node\n");
	}

	rc = of_property_match_string(bcdev->dev->of_node, "io-channel-names",
				      "sub_con_btb_detect_adc");
	if (rc >= 0) {
		bcdev->iio.sub_con_btb_detect_adc = iio_channel_get(bcdev->dev,
								"sub_con_btb_detect_adc");
		if (IS_ERR(bcdev->iio.sub_con_btb_detect_adc)) {
			rc = PTR_ERR(bcdev->iio.sub_con_btb_detect_adc);
			if (rc != -EPROBE_DEFER)
				chg_err("chg_mos_temp_chan get error, %d\n", rc);
			bcdev->iio.sub_con_btb_detect_adc = NULL;
			return rc;
		}
	} else {
		chg_err("can't find chg_mos_temp_chan node\n");
	}

	rc = of_property_match_string(bcdev->dev->of_node, "io-channel-names", "vph_pwr");
	if (rc >= 0) {
		bcdev->iio.vph_pwr_chan = iio_channel_get(bcdev->dev, "vph_pwr");
		if (IS_ERR(bcdev->iio.vph_pwr_chan)) {
			rc = PTR_ERR(bcdev->iio.vph_pwr_chan);
			if (rc != -EPROBE_DEFER)
				chg_err("vph_pwr_chan get error, %d\n", rc);
			bcdev->iio.vph_pwr_chan = NULL;
			return rc;
		}
	} else {
		chg_err("can't find vph_pwr node\n");
	}

	rc = of_property_match_string(bcdev->dev->of_node, "io-channel-names", "vbat_sns_qbg");
	if (rc >= 0) {
		bcdev->iio.vbat_sns_qbg_chan = iio_channel_get(bcdev->dev, "vbat_sns_qbg");
		if (IS_ERR(bcdev->iio.vbat_sns_qbg_chan)) {
			rc = PTR_ERR(bcdev->iio.vbat_sns_qbg_chan);
			if (rc != -EPROBE_DEFER)
				chg_err("vbat_sns_qbg_chan get error, %d\n", rc);
			bcdev->iio.vbat_sns_qbg_chan = NULL;
			return rc;
		}
	} else {
		chg_err("can't find vbat_sns_qbg node\n");
	}

	rc = of_property_match_string(bcdev->dev->of_node, "io-channel-names", "pmic_vbat_adc");
	if (rc >= 0) {
		bcdev->iio.pmic_vbat_adc = iio_channel_get(bcdev->dev, "pmic_vbat_adc");
		if (IS_ERR(bcdev->iio.pmic_vbat_adc)) {
			rc = PTR_ERR(bcdev->iio.pmic_vbat_adc);
			if (rc != -EPROBE_DEFER)
				chg_err("pmic_vbat_adc channel get error, %d\n", rc);
			bcdev->iio.pmic_vbat_adc = NULL;
			return rc;
		}
	} else {
		chg_err("can't find pmic_vbat_adc channel node\n");
	}
	return rc;
}

#define SUBBOARD_HIGH_TEMP 690
#define SUBBOARD_TEMP_PRE_DEFAULT 250
static int oplus_get_subboard_temp(struct oplus_chg_ic_dev *ic_dev, int *get_temp)
{
	int rc = 0;
	int subboard_temp = 0;
	static int subboard_temp_pre = SUBBOARD_TEMP_PRE_DEFAULT;
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (IS_ERR_OR_NULL(bcdev->iio.subboard_temp_chan)) {
		chg_err("bcdev->iio.subboard_temp_v_chan is NULL\n");
		subboard_temp = subboard_temp_pre;
		goto exit;
	}

	rc = iio_read_channel_processed(bcdev->iio.subboard_temp_chan, &subboard_temp);
	if (rc < 0) {
		chg_err("iio_read_channel_processed get error[%d]\n", rc);
		subboard_temp = subboard_temp_pre;
		goto exit;
	}
	subboard_temp = subboard_temp / 100;

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if ((get_eng_version() == HIGH_TEMP_AGING) || (get_eng_version() == PTCRB)) {
		chg_err("CONFIG_HIGH_TEMP_VERSION enable here, disable high subboard temp shutdown\n");
		if (subboard_temp > SUBBOARD_HIGH_TEMP)
			subboard_temp = SUBBOARD_HIGH_TEMP;
	}
#endif

	subboard_temp_pre = subboard_temp;
	*get_temp = subboard_temp;
exit:
	return rc;
}

static int oplus_subboard_temp_gpio_init(struct battery_chg_dev *bcdev)
{
	bcdev->oplus_custom_gpio.subboard_temp_gpio_pinctrl = devm_pinctrl_get(bcdev->dev);
	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.subboard_temp_gpio_pinctrl)) {
		chg_err("get subboard_temp_gpio_pinctrl fail\n");
		return -EINVAL;
	}

	bcdev->oplus_custom_gpio.subboard_temp_gpio_default =
		pinctrl_lookup_state(bcdev->oplus_custom_gpio.subboard_temp_gpio_pinctrl, "subboard_temp_gpio_default");
	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.subboard_temp_gpio_default)) {
		chg_err("set subboard_temp_gpio_default error\n");
		return -EINVAL;
	}

	pinctrl_select_state(bcdev->oplus_custom_gpio.subboard_temp_gpio_pinctrl,
		bcdev->oplus_custom_gpio.subboard_temp_gpio_default);

	return 0;
}

static int oplus_btb_temp_gpio_init(struct battery_chg_dev *bcdev)
{
	bcdev->oplus_custom_gpio.btb_temp_gpio_pinctrl = devm_pinctrl_get(bcdev->dev);
	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.btb_temp_gpio_pinctrl)) {
		chg_err("get btb_temp_gpio_pinctrl fail\n");
		return -EINVAL;
	}

	bcdev->oplus_custom_gpio.btb_temp_gpio_default =
		pinctrl_lookup_state(bcdev->oplus_custom_gpio.btb_temp_gpio_pinctrl, "btb_temp_gpio_default");
	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.btb_temp_gpio_default)) {
		chg_err("set btb_temp_gpio_default error\n");
		return -EINVAL;
	}

	pinctrl_select_state(bcdev->oplus_custom_gpio.btb_temp_gpio_pinctrl,
		bcdev->oplus_custom_gpio.btb_temp_gpio_default);

	return 0;
}


static int oplus_chg_parse_custom_wls_dt(struct battery_chg_dev *bcdev)
{
	int rc = 0;
	struct device_node *node = NULL;

	if (!bcdev) {
		chg_err("bcdev is NULL!\n");
		return -ENODEV;
	}
	node = bcdev->dev->of_node;

	bcdev->wls_boost_soft_start = of_property_read_bool(bcdev->dev->of_node, "oplus,wls_boost_soft_start");
	rc = of_property_read_u32(bcdev->dev->of_node, "oplus,wls_boost_vol_start_mv", &bcdev->wls_boost_vol_start_mv);
	if (rc < 0)
		bcdev->wls_boost_vol_start_mv = 0;
	rc = of_property_read_u32(bcdev->dev->of_node, "oplus,wls_boost_vol_max_mv", &bcdev->wls_boost_vol_max_mv);
	if (rc < 0)
		bcdev->wls_boost_vol_max_mv = 5500;
	rc = of_property_read_u32(node, "oplus,wls_boost_src", &bcdev->wls_boost_src);
	if (rc < 0)
		bcdev->wls_boost_src = -EINVAL;

	bcdev->oplus_custom_gpio.tx_boost_en_gpio = of_get_named_gpio(node, "oplus,tx_boost_en-gpio", 0);
	if (bcdev->oplus_custom_gpio.tx_boost_en_gpio <= 0) {
		chg_err("Couldn't read qcom,tx_boost_en-gpio, rc = %d\n", rc);
	} else {
		if (gpio_is_valid(bcdev->oplus_custom_gpio.tx_boost_en_gpio)) {
			rc = gpio_request(bcdev->oplus_custom_gpio.tx_boost_en_gpio, "tx_boost_en-gpio");
			if (rc) {
				chg_err("unable to tx_boost_en-gpio:%d\n", bcdev->oplus_custom_gpio.tx_boost_en_gpio);
			} else {
				rc = oplus_tx_boost_en_gpio_init(bcdev);
				if (rc)
					chg_err("unable to init tx_boost_en-gpio:%d\n",
						bcdev->oplus_custom_gpio.tx_boost_en_gpio);
				else
					chg_info("init tx_boost_en_gpio level[%d]\n",
						gpio_get_value(bcdev->oplus_custom_gpio.tx_boost_en_gpio));
			}
		}
		chg_info("tx_boost_en-gpio:%d\n", bcdev->oplus_custom_gpio.tx_boost_en_gpio);
	}

	bcdev->oplus_custom_gpio.tx_ovp_en_gpio = of_get_named_gpio(node, "oplus,tx_ovp_en-gpio", 0);
	if (bcdev->oplus_custom_gpio.tx_ovp_en_gpio <= 0) {
		chg_err("Couldn't read qcom,tx_ovp_en-gpio, rc = %d\n", rc);
	} else {
		if (gpio_is_valid(bcdev->oplus_custom_gpio.tx_ovp_en_gpio)) {
			rc = gpio_request(bcdev->oplus_custom_gpio.tx_ovp_en_gpio, "tx_ovp_en-gpio");
			if (rc) {
				chg_err("unable to tx_ovp_en-gpio:%d\n", bcdev->oplus_custom_gpio.tx_ovp_en_gpio);
			} else {
				rc = oplus_tx_ovp_en_gpio_init(bcdev);
				if (rc)
					chg_err("unable to init tx_ovp_en-gpio:%d\n",
						bcdev->oplus_custom_gpio.tx_ovp_en_gpio);
				else
					chg_info("init tx_ovp_en_gpio level[%d]\n",
						gpio_get_value(bcdev->oplus_custom_gpio.tx_ovp_en_gpio));
			}
		}
		chg_info("tx_ovp_en-gpio:%d\n", bcdev->oplus_custom_gpio.tx_ovp_en_gpio);
	}

	bcdev->oplus_custom_gpio.wrx_ovp_off_gpio = of_get_named_gpio(node, "oplus,wrx_ovp_off-gpio", 0);
	if (bcdev->oplus_custom_gpio.wrx_ovp_off_gpio <= 0) {
		chg_err("Couldn't read qcom,wrx_ovp_off-gpio, rc = %d\n", rc);
	} else {
		if (gpio_is_valid(bcdev->oplus_custom_gpio.wrx_ovp_off_gpio)) {
			rc = gpio_request(bcdev->oplus_custom_gpio.wrx_ovp_off_gpio, "wrx_ovp_off-gpio");
			if (rc) {
				chg_err("unable to wrx_ovp_off-gpio:%d\n", bcdev->oplus_custom_gpio.wrx_ovp_off_gpio);
			} else {
				rc = oplus_wrx_ovp_off_gpio_init(bcdev);
				if (rc)
					chg_err("unable to init wrx_ovp_off-gpio:%d\n",
						bcdev->oplus_custom_gpio.wrx_ovp_off_gpio);
				else
					chg_info("init wrx_ovp_off_gpio level[%d]\n",
						gpio_get_value(bcdev->oplus_custom_gpio.wrx_ovp_off_gpio));
			}
		}
		chg_info("wrx_ovp_off-gpio:%d\n", bcdev->oplus_custom_gpio.wrx_ovp_off_gpio);
	}

	bcdev->oplus_custom_gpio.wrx_otg_en_gpio = of_get_named_gpio(node, "oplus,wrx_otg_en-gpio", 0);
	if (bcdev->oplus_custom_gpio.wrx_otg_en_gpio <= 0) {
		chg_err("Couldn't read qcom,wrx_otg_en-gpio, rc = %d\n", rc);
	} else {
		if (gpio_is_valid(bcdev->oplus_custom_gpio.wrx_otg_en_gpio)) {
			rc = gpio_request(bcdev->oplus_custom_gpio.wrx_otg_en_gpio, "wrx_otg_en-gpio");
			if (rc) {
				chg_err("unable to wrx_otg_en-gpio:%d\n", bcdev->oplus_custom_gpio.wrx_otg_en_gpio);
			} else {
				rc = oplus_wrx_otg_en_gpio_init(bcdev);
				if (rc)
					chg_err("unable to init wrx_otg_en-gpio:%d\n",
						bcdev->oplus_custom_gpio.wrx_otg_en_gpio);
				else
					chg_info("init wrx_otg_en_gpio level[%d]\n",
						gpio_get_value(bcdev->oplus_custom_gpio.wrx_otg_en_gpio));
			}
		}
		chg_info("wrx_otg_en-gpio:%d\n", bcdev->oplus_custom_gpio.wrx_otg_en_gpio);
	}

	return 0;
}

static int oplus_get_power_mos_gpio(struct battery_chg_dev *bcdev)
{
	struct device_node *node = NULL;
	int rc  = 0;

	if (!bcdev) {
		chg_err("bcdev is NULL!\n");
		return -EINVAL;
	}
	node = bcdev->dev->of_node;

	bcdev->oplus_custom_gpio.supplementary_power_mos_gpio =
					of_get_named_gpio(node, "oplus,supplementary-power-gpio", 0);
	if (!gpio_is_valid(bcdev->oplus_custom_gpio.supplementary_power_mos_gpio)) {
		chg_err("supplementary-power-gpio not support\n");
		return -ENOTSUPP;
	}

	rc = gpio_request(bcdev->oplus_custom_gpio.supplementary_power_mos_gpio, "supplementary-power-gpio");
	if (rc) {
		chg_err("unable to supplementary_power_mos_gpio:%d\n",
						bcdev->oplus_custom_gpio.supplementary_power_mos_gpio);
		return -EINVAL;
	}

	return 0;
}

static int oplus_get_power_mos_pinctrl(struct battery_chg_dev *bcdev)
{
	if (!bcdev) {
		chg_err("bcdev is NULL!\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(bcdev->oplus_custom_gpio.supplementary_power_mos_gpio))
		return -EINVAL;

	bcdev->oplus_custom_gpio.supplementary_power_pinctrl = devm_pinctrl_get(bcdev->dev);
	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.supplementary_power_pinctrl)) {
		chg_err("get supplementary_power_pinctrl fail\n");
		goto free_supplementary_power_gpio;
	}

	bcdev->oplus_custom_gpio.supplementary_power_mos_active =
				pinctrl_lookup_state(bcdev->oplus_custom_gpio.supplementary_power_pinctrl,
									"oplus_supplementary_power_active");
	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.supplementary_power_mos_active)) {
		chg_err("get supplementary_power_mos_active fail\n");
		goto free_supplementary_power_gpio;
	}

	bcdev->oplus_custom_gpio.supplementary_power_mos_sleep =
				pinctrl_lookup_state(bcdev->oplus_custom_gpio.supplementary_power_pinctrl,
									"oplus_supplementary_power_sleep");
	if (IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.supplementary_power_mos_sleep)) {
		chg_err("get supplementary_power_mos_sleep fail\n");
		goto free_supplementary_power_gpio;
	}

	return 0;

free_supplementary_power_gpio:
	if (gpio_is_valid(bcdev->oplus_custom_gpio.supplementary_power_mos_gpio))
		gpio_free(bcdev->oplus_custom_gpio.supplementary_power_mos_gpio);
	return -EINVAL;
}

static int oplus_check_supplementary_power_pinctrl(struct battery_chg_dev *bcdev)
{
	if (!bcdev) {
		chg_err("bcdev not ready!\n");
		return -ENODEV;
	}

	if (!gpio_is_valid(bcdev->oplus_custom_gpio.supplementary_power_mos_gpio) ||
	    IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.supplementary_power_pinctrl) ||
	    IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.supplementary_power_mos_active) ||
	    IS_ERR_OR_NULL(bcdev->oplus_custom_gpio.supplementary_power_mos_sleep)) {
		chg_err("supplementary_power pmos pinctrl error\n");
		return -EINVAL;
	}

	return 0;
}

static int oplus_supplementary_power_mos_init(struct battery_chg_dev *bcdev)
{
	if (!bcdev) {
		chg_err("bcdev is NULL!\n");
		return -EINVAL;
	}

	if ((oplus_get_power_mos_gpio(bcdev) != 0) ||
	    (!gpio_is_valid(bcdev->oplus_custom_gpio.supplementary_power_mos_gpio)))
		return -EINVAL;

	if (oplus_get_power_mos_pinctrl(bcdev) != 0)
		return -EINVAL;

	if (oplus_check_supplementary_power_pinctrl(bcdev) != 0)
		return -EINVAL;

	mutex_lock(&bcdev->oplus_custom_gpio.pinctrl_mutex);
	pinctrl_select_state(bcdev->oplus_custom_gpio.supplementary_power_pinctrl,
				bcdev->oplus_custom_gpio.supplementary_power_mos_sleep);
	bcdev->power_mos_status = false;
	bcdev->need_check_mos = true;
	bcdev->mos_retry_cnt = 2;
	mutex_unlock(&bcdev->oplus_custom_gpio.pinctrl_mutex);

	chg_info("get supplementary_power_mos_gpio level[%d]\n",
				gpio_get_value_cansleep(bcdev->oplus_custom_gpio.supplementary_power_mos_gpio));

	return 0;
}

static void oplus_parse_aicl_thr_table(struct battery_chg_dev *bcdev, struct device_node *node)
{
	int rc = 0;

	bcdev->aicl_thr_table_init = false;
	rc = of_property_count_elems_of_size(node, "oplus,aicl_thr_table", sizeof(u32));
	if (rc >= 0) {
		int length = rc;

		if (length == AICL_POINT_INDEX_MAX * 3) {
			rc = of_property_read_u32_array(node, "oplus,aicl_thr_table",
				(u32 *)bcdev->aicl_thr_table, length);
			if (rc < 0) {
				chg_err("read %s failed, rc=%d\n", "oplus,aicl_thr_table", rc);
			} else {
				bcdev->aicl_thr_table_init = true;
				chg_err("aicl_thr_table loaded from DT\n");
			}
		} else {
			chg_err("Count %s invalid, rc=%d (expect %d)\n",
				"oplus,aicl_thr_table", rc, AICL_POINT_INDEX_MAX * 3);
		}
	}
}

static int oplus_chg_parse_custom_dt(struct battery_chg_dev *bcdev)
{
	int rc = 0;
	struct device_node *node = NULL;
	int i = 0;

	if (!bcdev) {
		chg_err("bcdev is NULL!\n");
		return -1;
	}
	node = bcdev->dev->of_node;

	bcdev->oplus_custom_gpio.vchg_trig_gpio =
		of_get_named_gpio(node, "qcom,vchg_trig-gpio", 0);
	if (bcdev->oplus_custom_gpio.vchg_trig_gpio <= 0) {
		chg_err("Couldn't read qcom,vchg_trig-gpio rc = %d, vchg_trig-gpio:%d\n",
					rc, bcdev->oplus_custom_gpio.vchg_trig_gpio);
	} else {
		if (oplus_vchg_trig_is_support() == true) {
			rc = gpio_request(bcdev->oplus_custom_gpio.vchg_trig_gpio, "vchg_trig-gpio");
			if (rc) {
				chg_err("unable to vchg_trig-gpio:%d\n",
							bcdev->oplus_custom_gpio.vchg_trig_gpio);
			} else {
				rc = oplus_vchg_trig_gpio_init(bcdev);
				if (rc)
					chg_err("unable to init vchg_trig-gpio:%d\n",
							bcdev->oplus_custom_gpio.vchg_trig_gpio);
				else
					oplus_vchg_trig_irq_init(bcdev);
			}
		}
		chg_err("vchg_trig-gpio:%d\n", bcdev->oplus_custom_gpio.vchg_trig_gpio);
	}

	bcdev->oplus_custom_gpio.otg_boost_en_gpio =
		of_get_named_gpio(node, "qcom,otg-booster-en-gpio", 0);
	if (bcdev->oplus_custom_gpio.otg_boost_en_gpio <= 0) {
		chg_err("Couldn't read qcom,otg_booster-en-gpio rc = %d, qcom,otg-booster-en-gpio:%d\n",
			rc, bcdev->oplus_custom_gpio.otg_boost_en_gpio);
	} else {
		if (gpio_is_valid(bcdev->oplus_custom_gpio.otg_boost_en_gpio) == true) {
			rc = gpio_request(bcdev->oplus_custom_gpio.otg_boost_en_gpio, "otg-boost-en-gpio");
			if (rc) {
				chg_err("unable to request otg-boost-en-gpio:%d\n", bcdev->oplus_custom_gpio.otg_boost_en_gpio);
			} else {
				rc = oplus_otg_boost_en_gpio_init(bcdev);
				if (rc)
					chg_err("unable to init otg-boost-en-gpio:%d\n",
						bcdev->oplus_custom_gpio.otg_boost_en_gpio);
				else
					chg_err("init otg-boost-en-gpio level[%d]\n",
						gpio_get_value(bcdev->oplus_custom_gpio.otg_boost_en_gpio));
			}
		}
		chg_err("otg-boost-en-gpio:%d\n", bcdev->oplus_custom_gpio.otg_boost_en_gpio);
	}

	bcdev->oplus_custom_gpio.otg_ovp_en_gpio =
			of_get_named_gpio(node, "qcom,otg-ovp-en-gpio", 0);
	if (bcdev->oplus_custom_gpio.otg_ovp_en_gpio <= 0) {
		chg_err("Couldn't read qcom,otg-ovp-en-gpio rc = %d, qcom,otg-ovp-en-gpio:%d\n",
			rc, bcdev->oplus_custom_gpio.otg_ovp_en_gpio);
	} else {
		if (gpio_is_valid(bcdev->oplus_custom_gpio.otg_ovp_en_gpio) == true) {
			rc = gpio_request(bcdev->oplus_custom_gpio.otg_ovp_en_gpio, "otg-ovp-en-gpio");
			if (rc) {
				chg_err("unable to request otg-ovp-en-gpio:%d\n", bcdev->oplus_custom_gpio.otg_ovp_en_gpio);
			} else {
				rc = oplus_otg_ovp_en_gpio_init(bcdev);
				if (rc)
					chg_err("unable to init otg-ovp-en-gpio:%d\n",
						bcdev->oplus_custom_gpio.otg_ovp_en_gpio);
				else
					chg_err("init otg-ovp-en-gpio level[%d]\n",
						gpio_get_value(bcdev->oplus_custom_gpio.otg_ovp_en_gpio));
			}
		}
		chg_err("otg-ovp-en-gpio:%d\n", bcdev->oplus_custom_gpio.otg_ovp_en_gpio);
	}

	oplus_supplementary_power_mos_init(bcdev);

	rc = of_property_read_u32(node, "oplus,otg_scheme",
				  &bcdev->otg_scheme);
	if (rc) {
		bcdev->otg_scheme = OTG_SCHEME_UNDEFINE;
	}

	rc = of_property_read_u32(node, "qcom,otg_boost_src",
				  &bcdev->otg_boost_src);
	if (rc) {
		bcdev->otg_boost_src = OTG_BOOST_SOURCE_EXTERNAL;
	}

	bcdev->real_mvolts_min_support = !of_property_read_bool(node, "oplus,vbat_min_bypass_max_channel");
	chg_info("real_mvolts_min_support:%d\n", bcdev->real_mvolts_min_support);
	bcdev->bypass_vooc_support = of_property_read_bool(node, "oplus,bypass_vooc_support");
	bcdev->ufcs_run_check_support = of_property_read_bool(node, "oplus,ufcs_run_check_support");
	bcdev->usb_aicl_enhance = of_property_read_bool(node, "oplus,usb_aicl_enhance");
	bcdev->soccp_support = of_property_read_bool(node, "oplus,soccp_support");

	rc = read_signed_data_from_node(node, "oplus,sub_btb_valid_temp",
					(s32 *)(bcdev->sub_btb_valid_temp),
					OPLUS_SUB_BTB_MAX);
	if (rc < 0) {
		chg_err("get sub_btb_valid_temp, rc = %d\n", rc);
		for (i = 0; i < OPLUS_SUB_BTB_MAX - 1; i++)
			bcdev->sub_btb_valid_temp[i] = 0;
	} else {
		for (i = 0; i < OPLUS_SUB_BTB_MAX - 1; i++)
			chg_info(" valid_temp[%d] = %d", i, bcdev->sub_btb_valid_temp[i]);
	}

	bcdev->batt_full_method_new = of_property_read_bool(node, "oplus,batt_full_method_new");
	chg_info("get batt_full_method_new %d\n", bcdev->batt_full_method_new);

	if (bcdev->batt_full_method_new) {
		rc = read_signed_data_from_node(node, "oplus,batt_full_temp",
						(u32 *)bcdev->batt_full_temp, QBG_TEMP_MAX);
		if (rc < 0) {
			chg_err("get oplus,batt_full_temp property error, use old method, rc=%d\n",
				rc);
			bcdev->batt_full_method_new = false;
		}
	}

	if (bcdev->batt_full_method_new) {
		rc = read_unsigned_data_from_node(node, "oplus,batt_full_para",
						(u32 *)bcdev->batt_full_para,
						CHARGING_TYPE_MAX * QBG_TEMP_MAX);
		if (rc < 0) {
			chg_err("get oplus,batt_full_para property error, use old method, rc=%d\n",
				rc);
			bcdev->batt_full_method_new = false;
		}
	}

	bcdev->adsp_reboot_discnt_chg_support = of_property_read_bool(node, "oplus,adsp_reboot_discnt_chg_support");
	chg_info("adsp_reboot_discnt_chg_support:%d\n", bcdev->adsp_reboot_discnt_chg_support);

	oplus_parse_aicl_thr_table(bcdev, node);
	return 0;
}
#endif /*OPLUS_FEATURE_CHG_BASIC*/

static int battery_chg_parse_dt(struct battery_chg_dev *bcdev)
{
	struct device_node *node = bcdev->dev->of_node;
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int i, rc, len;
	u32 prev, val;

#ifdef OPLUS_FEATURE_CHG_BASIC
	bcdev->otg_online = false;
	bcdev->pd_svooc = false;
	bcdev->pd_check_completed = false;
#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
	oplus_subboard_temp_gpio_init(bcdev);
	oplus_btb_temp_gpio_init(bcdev);
#endif
	of_property_read_string(node, "qcom,wireless-fw-name",
				&bcdev->wls_fw_name);
	bcdev->oem_lcm_check = of_property_read_bool(node, "oplus,oem-lcm-check");
	rc = of_property_count_elems_of_size(node, "qcom,thermal-mitigation",
						sizeof(u32));
	if (rc <= 0)
		return 0;

	len = rc;

	rc = read_property_id(bcdev, pst, BATT_CHG_CTRL_LIM_MAX);
	if (rc < 0)
		return rc;

	prev = pst->prop[BATT_CHG_CTRL_LIM_MAX];

	for (i = 0; i < len; i++) {
		rc = of_property_read_u32_index(node, "qcom,thermal-mitigation",
						i, &val);
		if (rc < 0)
			return rc;

		if (val > prev) {
			chg_err("Thermal levels should be in descending order\n");
			bcdev->num_thermal_levels = -EINVAL;
			return 0;
		}

		prev = val;
	}

	bcdev->thermal_levels = devm_kcalloc(bcdev->dev, len + 1,
					sizeof(*bcdev->thermal_levels),
					GFP_KERNEL);
	if (!bcdev->thermal_levels)
		return -ENOMEM;

	/*
	 * Element 0 is for normal charging current. Elements from index 1
	 * onwards is for thermal mitigation charging currents.
	 */

	bcdev->thermal_levels[0] = pst->prop[BATT_CHG_CTRL_LIM_MAX];

	rc = of_property_read_u32_array(node, "qcom,thermal-mitigation",
					&bcdev->thermal_levels[1], len);
	if (rc < 0) {
		chg_err("Error in reading qcom,thermal-mitigation, rc=%d\n", rc);
		return rc;
	}

	bcdev->num_thermal_levels = len;
	bcdev->thermal_fcc_ua = pst->prop[BATT_CHG_CTRL_LIM_MAX];

	return 0;
}

static int battery_chg_ship_mode(struct notifier_block *nb, unsigned long code,
		void *unused)
{
	struct battery_charger_ship_mode_req_msg msg = { { 0 } };
	struct battery_chg_dev *bcdev = container_of(nb, struct battery_chg_dev,
						     reboot_notifier);
	int rc;

	if (!bcdev->ship_mode_en)
		return NOTIFY_DONE;

	msg.hdr.owner = MSG_OWNER_BC;
	msg.hdr.type = MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = BC_SHIP_MODE_REQ_SET;
	msg.ship_mode_type = SHIP_MODE_PMIC;

	if (code == SYS_POWER_OFF) {
		rc = battery_chg_write(bcdev, &msg, sizeof(msg));
		if (rc < 0)
			pr_emerg("Failed to write ship mode: %d\n", rc);
	}

	return NOTIFY_DONE;
}

/**********************************************************************
 * battery charge ops *
 **********************************************************************/
#ifdef OPLUS_FEATURE_CHG_BASIC
static int oplus_get_voocphy_enable(struct battery_chg_dev *bcdev)
{
	int rc = 0;
	struct psy_state *pst = NULL;
	int vooc_enable = 0;

	if (!bcdev) {
		chg_err("bcdev is NULL!\n");
		return 0;
	}
	if (oplus_chg_get_voocphy_support(bcdev) != ADSP_VOOCPHY)
		return 0;
	pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_VOOCPHY_ENABLE);
	else
		rc = read_property_id(bcdev, pst, USB_VOOCPHY_ENABLE);
	if (rc) {
		chg_err("get enable voocphy fail, rc=%d\n", rc);
		return 0;
	} else {
		chg_err("get enable voocphy success, rc=%d\n", rc);
	}
	if (bcdev->soccp_support)
		vooc_enable = bcdev->oplus_psy.prop[OPLUS_USB_VOOCPHY_ENABLE];
	else
		vooc_enable = pst->prop[USB_VOOCPHY_ENABLE];

	return vooc_enable;
}

static int oplus_voocphy_enable(struct battery_chg_dev *bcdev, bool enable)
{
	int rc = 0;
	struct psy_state *pst = NULL;

	if (!bcdev) {
		chg_err("bcdev is NULL!\n");
		return -1;
	}
	if (oplus_chg_get_voocphy_support(bcdev) != ADSP_VOOCPHY)
		return rc;
	pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_VOOCPHY_ENABLE, enable);
	else
		rc = write_property_id(bcdev, pst, USB_VOOCPHY_ENABLE, enable);
	if (rc) {
		chg_err("set %s voocphy fail, rc=%d\n", enable ? "enable" : "disable", rc);
	} else {
		chg_err("set %s voocphy success, rc=%d\n", enable ? "enable" : "disable", rc);
	}

	return rc;
}


int oplus_adsp_voocphy_reset_again(void)
{
	int rc = 0;
	struct battery_chg_dev *bcdev = g_bcdev;
	struct psy_state *pst;

	if (!bcdev) {
		chg_err("bcdev is NULL!\n");
		return -1;
	}

	pst = &bcdev->psy_list[PSY_TYPE_USB];
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_VOOCPHY_RESET_AGAIN, true);
	else
		rc = write_property_id(bcdev, pst, USB_VOOCPHY_RESET_AGAIN, true);
	if (rc) {
		chg_err("set voocphy_reset_again fail, rc=%d\n", rc);
	} else {
		chg_err("set voocphy_reset_again success, rc=%d\n", rc);
	}

	return rc;
}

static void oplus_voocphy_err_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, voocphy_err_work.work);
	int mmi_chg = 1;

	chg_info("start voocphy error check\n");
	if (oplus_vooc_get_fastchg_ing(bcdev) == false && bcdev->voocphy_err_check) {
		chg_err("!!!happend\n");
		bcdev->voocphy_err_check = false;
		oplus_chg_suspend_charger(true, DEF_VOTER);
		usleep_range(1000000, 1000010);
		if (is_chg_disable_votable_available(bcdev))
			mmi_chg = !get_client_vote(bcdev->chg_disable_votable, MMI_CHG_VOTER);
		if (mmi_chg) {
			oplus_chg_suspend_charger(false, DEF_VOTER);
			oplus_chg_disable_charger(false, DEF_VOTER);
			oplus_adsp_voocphy_reset_again();
		}
	}
}

static int smbchg_lcm_en(struct battery_chg_dev *bcdev, bool en)
{
	int rc = 0;
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (bcdev->soccp_support) {
		if (en)
			rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_RELEASE_FIXED_FREQUENCE, 0);
		else
			rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_RELEASE_FIXED_FREQUENCE, 1);
	} else {
		if (en)
			rc = write_property_id(bcdev, pst, USB_POWER_SUPPLY_RELEASE_FIXED_FREQUENCE, 0);
		else
			rc = write_property_id(bcdev, pst, USB_POWER_SUPPLY_RELEASE_FIXED_FREQUENCE, 1);
	}
	if (rc < 0)
		chg_info("set lcm to %u error, rc = %d\n", en, rc);
	else
		chg_info("set lcm to %d \n", en);

	return rc;
}

static int oplus_get_batt_full_status(struct battery_chg_dev *bcdev)
{
	union mms_msg_data data = {0};

	if (is_common_topic_available(bcdev)) {
		oplus_mms_get_item_data(bcdev->common_topic,
					COMM_ITEM_CHG_FULL, &data, false);
	} else {
		chg_err("common topic not found\n");
	}

	return data.intval;
}

void lcm_frequency_ctrl(struct battery_chg_dev *bcdev)
{
	static int lcm_en_flag = LCM_EN_DEAFULT;
	static int  check_count = 0;

	check_count++;
	if (check_count > LCM_CHECK_COUNT) {
		lcm_en_flag = LCM_EN_DEAFULT;
		check_count = 0;
	}

	if ((oplus_wired_get_vbus() > LCM_CHARGER_VOL_THR_MV)) {
		if (oplus_get_batt_full_status(bcdev) || smbchg_get_charge_enable()) {
			if (lcm_en_flag != LCM_EN_ENABLE) {
				lcm_en_flag = LCM_EN_ENABLE;
				smbchg_lcm_en(bcdev, true);
				chg_info("lcm_en_flag:%d\n", lcm_en_flag);
			}
		} else {
			if (lcm_en_flag != LCM_EN_DISABLE) {
				lcm_en_flag = LCM_EN_DISABLE;
				smbchg_lcm_en(bcdev, false);
				chg_info(" lcm_en_flag:%d\n", lcm_en_flag);
			}
		}

		mod_delayed_work(system_highpri_wq, &bcdev->ctrl_lcm_frequency,
				 LCM_FREQUENCY_INTERVAL);
	} else {
			if (lcm_en_flag != LCM_EN_ENABLE) {
				lcm_en_flag = LCM_EN_ENABLE;
				smbchg_lcm_en(bcdev, true);
				chg_info(" lcm_en_flag:%d\n", lcm_en_flag);
			}
	}
}

static bool oplus_chg_wls_is_present(struct battery_chg_dev *bcdev)
{
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc = 0;
	int usb_in_status = 0;
	int usb_sns_status = 0;

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_IN_STATUS);
	else
		rc = read_property_id(bcdev, pst, USB_IN_STATUS);
	if (rc) {
		if (bcdev->soccp_support)
			bcdev->oplus_psy.prop[OPLUS_USB_IN_STATUS] = 0;
		else
			pst->prop[USB_IN_STATUS] = 0;
		chg_info("read usb_in_status fail, rc=%d\n", rc);
	}
	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_SNS_STATUS);
	else
		rc = read_property_id(bcdev, pst, USB_SNS_STATUS);
	if (rc) {
		if (bcdev->soccp_support)
			bcdev->oplus_psy.prop[OPLUS_USB_IN_STATUS] = 0;
		else
			pst->prop[USB_IN_STATUS] = 0;
		chg_info("read usb_sns_status fail, rc=%d\n", rc);
	}

	if (bcdev->soccp_support) {
		usb_in_status = bcdev->oplus_psy.prop[OPLUS_USB_IN_STATUS];
		usb_sns_status = bcdev->oplus_psy.prop[OPLUS_USB_SNS_STATUS];
	} else {
		usb_in_status = pst->prop[USB_IN_STATUS];
		usb_sns_status = pst->prop[USB_SNS_STATUS];
	}
	if (usb_in_status > 0 && usb_sns_status == 0)
		return true;
	return false;

	/*struct oplus_mms *wls_topic = NULL;
	union mms_msg_data data = { 0 };
	bool present = false;

	wls_topic = oplus_mms_get_by_name("wireless");
	if (wls_topic) {
		oplus_mms_get_item_data(wls_topic, WLS_ITEM_PRESENT, &data, true);
		present = !!data.intval;
		return present;
	} else {
		return false;
	}*/
}

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
static bool oplus_get_wls_fastcharging(struct battery_chg_dev *bcdev)
{
	union mms_msg_data data = { 0 };

	if (!bcdev) {
		return false;
	}

	if (!bcdev->wls_topic)
		bcdev->wls_topic = oplus_mms_get_by_name("wireless");

	if (bcdev->wls_topic)
		oplus_mms_get_item_data(bcdev->wls_topic, WLS_ITEM_FASTCHG_STATUS, &data, true);

	return !!data.intval;
}
#endif

static void oplus_chg_ctrl_lcm_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, ctrl_lcm_frequency.work);

	lcm_frequency_ctrl(bcdev);
}

static void oplus_plugin_irq_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, plugin_irq_work.work);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	static bool usb_pre_plugin_status;
	static bool usb_plugin_status;
	int rc = 0;
	int prop_id = 0;
	int type = 0;
	int sub_type = 0;
	static int pre_type = 0;
	static int pre_sub_type = 0;
	int usb_in = 0;

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_IN_STATUS);
	else
		rc = read_property_id(bcdev, pst, USB_IN_STATUS);
	if (rc) {
		bcdev->usb_in_status = 0;
		chg_err("read usb_in_status fail, rc=%d\n", rc);
		return;
	}
	if (bcdev->soccp_support)
		usb_in = bcdev->oplus_psy.prop[OPLUS_USB_IN_STATUS];
	else
		usb_in = pst->prop[USB_IN_STATUS];

	/* Fix the issue of not charging or not enter vooc/svooc when adsp reboot */
	if (bcdev->adsp_reboot_discnt_chg_support && usb_in &&
	    atomic_read(&bcdev->adsp_reboot) == 1) {
		chg_err("adsp_reboot force the usb_in = %d as 0.\n", usb_in);
		usb_in = 0;
	}

	if (usb_in > 0 && oplus_chg_wls_is_present(bcdev)) {
		chg_info("USBIN irq but wls present\n");
		return;
	}
	if (usb_in > 0) {
		bcdev->rerun_max = 3;
		bcdev->usb_in_status = 1;
	} else {
		bcdev->usb_in_status = 0;
		bcdev->abnormal_usbin_count = 0;
		cancel_delayed_work(&bcdev->check_abnormal_usbin_status_work);
	}
	usb_plugin_status = usb_in & 0xff;
	chg_info("prop[%d], usb_online[%d]\n", usb_in,
		 bcdev->usb_in_status);

	if (bcdev && bcdev->ctrl_lcm_frequency.work.func) {
		mod_delayed_work(system_highpri_wq, &bcdev->ctrl_lcm_frequency, 50);
	}

	prop_id = get_property_id(pst, POWER_SUPPLY_PROP_USB_TYPE);
	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_TYPE);
	else
		rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0) {
		chg_err("read usb charger_type fail, rc=%d\n", rc);
	} else {
		if (bcdev->soccp_support)
			type = bcdev->oplus_psy.prop[OPLUS_USB_TYPE];
		else
			type = pst->prop[prop_id];
	}

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_SUBTYPE);
	else
		rc = read_property_id(bcdev, pst, USB_ADAP_SUBTYPE);
	if (rc < 0) {
		chg_err("read charger subtype fail, rc=%d\n", rc);
	} else {
		if (bcdev->soccp_support)
			sub_type = bcdev->oplus_psy.prop[OPLUS_USB_SUBTYPE];
		else
			sub_type = pst->prop[USB_ADAP_SUBTYPE];
	}
	if (usb_plugin_status && usb_plugin_status == usb_pre_plugin_status &&
	    type == pre_type && sub_type == pre_sub_type) {
		chg_info("usb_plugin_status:%d, type:%d, sub_type:%d, no change, return\n",
			 usb_plugin_status, type, sub_type);
		return;
	}

	pre_type = type;
	pre_sub_type = sub_type;

#ifdef OPLUS_CHG_UNDEF /* TODO */
	if (bcdev->usb_ocm) {
		if (bcdev->usb_in_status == 1) {
			if (g_oplus_chip && g_oplus_chip->charger_type == POWER_SUPPLY_TYPE_WIRELESS)
				g_oplus_chip->charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
			oplus_chg_global_event(bcdev->usb_ocm, OPLUS_CHG_EVENT_ONLINE);
		} else {
			if ((oplus_get_wired_chg_present() == false)
			    && (g_oplus_chip->charger_volt < CHARGER_PRESENT_VOLT_MV)) {
				bcdev->pd_svooc = false; /* remove svooc flag */
			}
			oplus_chg_global_event(bcdev->usb_ocm, OPLUS_CHG_EVENT_OFFLINE);
		}
	}
#endif
	chg_info("usb_pre_plugin_status[%d], usb_plugin_status[%d]\n",
		 usb_pre_plugin_status, usb_plugin_status);
	if (usb_pre_plugin_status != usb_plugin_status || !usb_plugin_status) {
		oplus_chg_suspend_charger(false, PD_PDO_ICL_VOTER);
		oplus_chg_ic_virq_trigger(bcdev->buck_ic, OPLUS_IC_VIRQ_PLUGIN);
		if (bcdev->qcom_gauge_cali_track_support &&
		    usb_pre_plugin_status != usb_plugin_status)
			schedule_work(&bcdev->gauge_cali_track_by_plug_work);
	}
	if (usb_pre_plugin_status != usb_plugin_status && !usb_pre_plugin_status)
		bcdev->read_by_reg = 0;

	if (bcdev->usb_in_status == 0 && usb_pre_plugin_status != 0) {
		bcdev->pd_svooc = false;
		bcdev->ufcs_power_ready = false;
		bcdev->ufcs_handshake_ok = false;
		bcdev->ufcs_pdo_ready = false;
		bcdev->ufcs_verify_auth_ready = false;
		bcdev->adapter_verify_auth = false;
		bcdev->ufcs_power_info_ready = false;
		bcdev->ufcs_vdm_emark_ready = false;
		bcdev->bc12_completed = false;
		bcdev->ufcs_exiting = false;
		bcdev->pd_chg_volt = OPLUS_PD_5V;
		bcdev->pd_check_completed = false;
		oplus_chg_clear_vdm_info(bcdev);
		bcdev->hvdcp_detach_time = cpu_clock(smp_processor_id()) / CPU_CLOCK_TIME_MS;
		chg_err("the hvdcp_detach_time:%llu, detect time %llu \n",
			bcdev->hvdcp_detach_time, bcdev->hvdcp_detect_time);
		if (bcdev->hvdcp_detach_time - bcdev->hvdcp_detect_time <= OPLUS_HVDCP_DETECT_TO_DETACH_TIME) {
			bcdev->hvdcp_disable = true;
			schedule_delayed_work(&bcdev->hvdcp_disable_work, OPLUS_HVDCP_DISABLE_INTERVAL);
		} else {
			bcdev->hvdcp_detect_ok = false;
			bcdev->hvdcp_detect_time = 0;
			bcdev->hvdcp_disable = false;
		}
		bcdev->voocphy_err_check = false;
		if (bcdev->soccp_support && bcdev->qos_status) {
			cancel_delayed_work(&bcdev->request_qos_work);
			schedule_delayed_work(&bcdev->release_qos_work, 0);
		}
		cancel_delayed_work_sync(&bcdev->voocphy_err_work);
	}

	/* Note: triger chg type change to update the real charger type. */
	if ((bcdev->usb_in_status == 1) && (usb_pre_plugin_status != usb_plugin_status))
		schedule_delayed_work(&bcdev->pd_only_check_work, OPLUS_PD_ONLY_CHECK_INTERVAL);
	else
		oplus_chg_ic_virq_trigger(bcdev->buck_ic, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);

	usb_pre_plugin_status = usb_plugin_status;
}

#endif /* OPLUS_FEATURE_CHG_BASIC */

/**********************************************************************
 * battery gauge ops *
 **********************************************************************/
#ifdef OPLUS_FEATURE_CHG_BASIC
__maybe_unused static bool oplus_get_pps_charging(struct battery_chg_dev *bcdev)
{
	bool pps_charging = false;
	union mms_msg_data data = {0};

	if (!bcdev) {
		return false;
	}

	if (!bcdev->pps_topic)
		bcdev->pps_topic = oplus_mms_get_by_name("pps");

	if (bcdev->pps_topic) {
		oplus_mms_get_item_data(bcdev->pps_topic,
					PPS_ITEM_CHARGING, &data, false);
		pps_charging = !!data.intval;
	}
	return pps_charging;
}

__maybe_unused static bool oplus_get_oplus_pps(struct battery_chg_dev *bcdev)
{
	bool oplus_pps = false;
	union mms_msg_data data = {0};

	if (!bcdev)
		return false;

	if (!bcdev->pps_topic)
		bcdev->pps_topic = oplus_mms_get_by_name("pps");

	if (bcdev->pps_topic) {
		oplus_mms_get_item_data(bcdev->pps_topic,
					PPS_ITEM_OPLUS_ADAPTER, &data, false);
		oplus_pps = !!data.intval;
	}
	return oplus_pps;
}

static bool oplus_get_ufcs_charging(struct battery_chg_dev *bcdev)
{
	bool ufcs_charging = false;
	union mms_msg_data data = {0};

	if (!bcdev) {
		return false;
	}

	if (!bcdev->ufcs_topic)
		bcdev->ufcs_topic = oplus_mms_get_by_name("ufcs");

	if (bcdev->ufcs_topic) {
		oplus_mms_get_item_data(bcdev->ufcs_topic,
					UFCS_ITEM_CHARGING, &data, false);
		ufcs_charging = !!data.intval;
	}
	return ufcs_charging;
}

static bool oplus_get_oplus_ufcs(struct battery_chg_dev *bcdev)
{
	bool oplus_ufcs = false;
	union mms_msg_data data = {0};

	if (!bcdev)
		return false;

	if (!bcdev->ufcs_topic)
		bcdev->ufcs_topic = oplus_mms_get_by_name("ufcs");

	if (bcdev->ufcs_topic) {
		oplus_mms_get_item_data(bcdev->ufcs_topic,
					UFCS_ITEM_OPLUS_ADAPTER, &data, false);
		oplus_ufcs = !!data.intval;
	}
	return oplus_ufcs;
}


static unsigned int oplus_update_batt_full_para(struct battery_chg_dev *bcdev)
{
	unsigned int ibatt_full_cur = 0;
	union mms_msg_data data = {0};
	int charging_status = 0;
	int temp_region = 0;
	static unsigned int pre_ibatt_full_cur = IBATT_FULL_CURR_DEFAULT;
	int temp = 250;
	unsigned int ffc_status = 0;

	if (is_common_topic_available(bcdev)) {
		oplus_mms_get_item_data(bcdev->common_topic, COMM_ITEM_FFC_STATUS,
				&data, false);
		ffc_status = data.intval;

		oplus_mms_get_item_data(bcdev->common_topic, COMM_ITEM_SHELL_TEMP,
				&data, false);
		temp = data.intval;
	} else {
		chg_err("common topic not found, return\n");
		goto exit;
	}

	if (temp < bcdev->batt_full_temp[QBG_TEMP_COOL])    /* - 12 */
		temp_region = QBG_TEMP_COLD;
	else if (temp < bcdev->batt_full_temp[QBG_TEMP_NORMAL])   /* 12 - 21 */
		temp_region = QBG_TEMP_COOL;
	else if (temp < bcdev->batt_full_temp[QBG_TEMP_WARM])  /* 21 - 44 */
		temp_region = QBG_TEMP_NORMAL;
	else  /* 44 -  */
		temp_region = QBG_TEMP_WARM;

	if (oplus_vooc_get_fastchg_ing(bcdev)) {
		charging_status = CHARGING_TYPE_VOOC_SVOOC;
	} else if (oplus_get_ufcs_charging(bcdev)) {
		if (oplus_get_oplus_ufcs(bcdev))
			charging_status = CHARGING_TYPE_OPLUS_UFCS;
		else
			charging_status = CHARGING_TYPE_THIRD_UFCS;
	} else if (oplus_get_pps_charging(bcdev)) {
		if (oplus_get_oplus_pps(bcdev))
			charging_status = CHARGING_TYPE_OPLUS_PPS;
		else
			charging_status = CHARGING_TYPE_THIRD_PPS;
	} else {
		if (ffc_status == FFC_WAIT || ffc_status == FFC_FAST)
			charging_status = CHARGING_TYPE_FFC;
		else
			charging_status = CHARGING_TYPE_UNKNOW;
	}

	if (charging_status < CHARGING_TYPE_MAX && temp_region < QBG_TEMP_MAX)
		ibatt_full_cur = bcdev->batt_full_para[charging_status][temp_region];
	else
		goto exit;

	if (pre_ibatt_full_cur == ibatt_full_cur)
		goto exit;

	chg_info("pre_ibatt_full_cur = %d, ibatt_full_cur = %d, charging_status = %d, temp = %d, temp_region = %d\n",
		pre_ibatt_full_cur, ibatt_full_cur, charging_status, temp, temp_region);

	pre_ibatt_full_cur = ibatt_full_cur;

	return ibatt_full_cur;

exit:
	return pre_ibatt_full_cur;
}

__maybe_unused static int fg_sm8350_get_battery_mvolts_max(void)
{
	int rc = 0;
	int prop_id = 0;
	static int volt = 4000;
	struct battery_chg_dev *bcdev = g_bcdev;
	struct psy_state *pst = NULL;

	if (!bcdev) {
		return -1;
	}

	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if (oplus_chg_get_voocphy_support(bcdev) == ADSP_VOOCPHY &&
	    !oplus_get_pps_charging(bcdev) && !oplus_get_ufcs_charging(bcdev) &&
	    !oplus_get_wls_fastcharging(bcdev) &&
	    !(bcdev->usb_in_status == 1 && get_eng_version() == FACTORY)) {
		volt = DIV_ROUND_CLOSEST(bcdev->read_buffer_dump.data_buffer[2], 1000);
		return volt;
	}
#else
	if (oplus_chg_get_voocphy_support(bcdev) == ADSP_VOOCPHY) {
		volt = DIV_ROUND_CLOSEST(bcdev->read_buffer_dump.data_buffer[2], 1000);
		return volt;
	}
#endif

	prop_id = get_property_id(pst, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0) {
		chg_err("read battery volt fail, rc=%d\n", rc);
		return volt;
	}
	volt = DIV_ROUND_CLOSEST(pst->prop[prop_id], 1000);

	return volt;
}

__maybe_unused static int fg_sm8350_get_battery_mvolts_min(void)
{
	int rc = 0;
	int prop_id = 0;
	static int volt = 4000;
	struct battery_chg_dev *bcdev = g_bcdev;
	struct psy_state *pst = NULL;

	if (!bcdev) {
		return -1;
	}

	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if (oplus_chg_get_voocphy_support(bcdev) == ADSP_VOOCPHY &&
	    !oplus_get_pps_charging(bcdev) && !oplus_get_ufcs_charging(bcdev) &&
	    !oplus_get_wls_fastcharging(bcdev) &&
	    !(bcdev->usb_in_status == 1 && get_eng_version() == FACTORY)) {
		volt = bcdev->read_buffer_dump.data_buffer[13];
		return volt;
	}
#else
	if (oplus_chg_get_voocphy_support(bcdev) == ADSP_VOOCPHY) {
		volt = bcdev->read_buffer_dump.data_buffer[13];
		return volt;
	}
#endif

	if (bcdev->real_mvolts_min_support && oplus_chg_get_voocphy_support(bcdev) == ADSP_VOOCPHY)
		prop_id = BATT_VOLT_MIN;
	else
		prop_id = get_property_id(pst, POWER_SUPPLY_PROP_VOLTAGE_NOW);

	if (bcdev->soccp_support && prop_id == BATT_VOLT_MIN)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_VOLTAGE_MIN);
	else
		rc = read_property_id(bcdev, pst, prop_id);

	if (rc < 0) {
		chg_err("read battery volt fail, rc=%d\n", rc);
		return volt;
	}
	if (bcdev->soccp_support && prop_id == BATT_VOLT_MIN)
		volt = bcdev->oplus_psy.prop[OPLUS_VOLTAGE_MIN];
	else
		volt = pst->prop[prop_id];

	volt = DIV_ROUND_CLOSEST(volt, 1000);

	return volt;
}

static int fg_sm8350_get_battery_temperature(void)
{
	int rc = 0;
	int prop_id = 0;
	static int temp = 250;
	struct battery_chg_dev *bcdev = g_bcdev;
	struct psy_state *pst = NULL;

	if (!bcdev) {
		return -1;
	}

	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if (oplus_chg_get_voocphy_support(bcdev) == ADSP_VOOCPHY) {
		temp = bcdev->read_buffer_dump.data_buffer[0];
		if (bcdev->gauge_data_initialized == true)
			goto HIGH_TEMP;
	}

	prop_id = get_property_id(pst, POWER_SUPPLY_PROP_TEMP);
	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_BATTERY_TEMP);
	else
		rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0) {
		chg_err("read battery temp fail, rc=%d\n", rc);
		return temp;
	}
	if (bcdev->soccp_support)
		temp = (int)bcdev->oplus_psy.prop[OPLUS_BATTERY_TEMP];
	else
		temp = (int)pst->prop[prop_id];
	temp = DIV_ROUND_CLOSEST((int)temp, 10);
HIGH_TEMP:
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if (get_eng_version() == HIGH_TEMP_AGING) {
		chg_err("CONFIG_HIGH_TEMP_VERSION enable here,"
			 "disable high tbat shutdown\n");
		if (temp > 690)
			temp = 690;
	}
#endif
	return temp;
}

static int fg_sm8350_get_batt_remaining_capacity(void)
{
	int rc = 0;
	static int batt_rm = 0;
	struct battery_chg_dev *bcdev = g_bcdev;
	struct psy_state *pst = NULL;

	if (!bcdev) {
		return batt_rm;
	}

	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if (oplus_chg_get_voocphy_support(bcdev) == ADSP_VOOCPHY) {
		batt_rm = bcdev->read_buffer_dump.data_buffer[4];
		return batt_rm;
	}

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_CHARGER_COUNTER);
	else
		rc = read_property_id(bcdev, pst, BATT_CHG_COUNTER);
	if (rc < 0) {
		chg_err("read battery chg counter fail, rc=%d\n", rc);
		return batt_rm;
	}
	if (bcdev->soccp_support)
		batt_rm = bcdev->oplus_psy.prop[OPLUS_CHARGER_COUNTER];
	else
		batt_rm = pst->prop[BATT_CHG_COUNTER];
	batt_rm = DIV_ROUND_CLOSEST(batt_rm, 1000);

	return batt_rm;
}

static int fg_sm8350_get_battery_soc(void)
{
	int rc = 0;
	int prop_id = 0;
	static int soc = 50;
	struct battery_chg_dev *bcdev = g_bcdev;
	struct psy_state *pst = NULL;

	if (!bcdev) {
		return -1;
	}

	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if (oplus_chg_get_voocphy_support(bcdev) == ADSP_VOOCPHY) {
		soc = DIV_ROUND_CLOSEST(bcdev->read_buffer_dump.data_buffer[3], 100);
		if (bcdev->gauge_data_initialized == true)
			return soc;
	}

	prop_id = get_property_id(pst, POWER_SUPPLY_PROP_CAPACITY);
	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_CAPACITY);
	else
		rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0) {
		chg_err("read battery soc fail, rc=%d\n", rc);
		return soc;
	}
	if (bcdev->soccp_support)
		soc = bcdev->oplus_psy.prop[OPLUS_CAPACITY];
	else
		soc = pst->prop[prop_id];
	soc = DIV_ROUND_CLOSEST(soc, 100);

	return soc;
}

static int fg_sm8350_get_average_current(void)
{
	int rc = 0;
	int prop_id = 0;
	static int curr = 0;
	struct battery_chg_dev *bcdev = g_bcdev;
	struct psy_state *pst = NULL;

	if (!bcdev) {
		return -1;
	}

	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if (oplus_chg_get_voocphy_support(bcdev) == ADSP_VOOCPHY &&
	    !oplus_get_pps_charging(bcdev) && !oplus_get_ufcs_charging(bcdev) &&
	    !oplus_get_wls_fastcharging(bcdev) &&
	    !(bcdev->usb_in_status == 1 && get_eng_version() == FACTORY) &&
		!bcdev->read_by_reg) {
		curr = DIV_ROUND_CLOSEST((int)bcdev->read_buffer_dump.data_buffer[1], 1000);
		return curr;
	}
#else
	if (oplus_chg_get_voocphy_support(bcdev) == ADSP_VOOCPHY) {
		curr = DIV_ROUND_CLOSEST((int)bcdev->read_buffer_dump.data_buffer[1], 1000);
		return curr;
	}
#endif

	prop_id = get_property_id(pst, POWER_SUPPLY_PROP_CURRENT_NOW);
	rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0) {
		chg_err("read battery curr fail, rc=%d\n", rc);
		return curr;
	}
	curr = DIV_ROUND_CLOSEST((int)pst->prop[prop_id], 1000);

	return curr;
}

static int fg_sm8350_get_battery_fcc(void)
{
	static int fcc = 0;
	struct battery_chg_dev *bcdev = g_bcdev;

	if (!bcdev) {
		return -1;
	}


	fcc = bcdev->read_buffer_dump.data_buffer[6];

	return fcc;
}

static int fg_sm8350_get_battery_cc(void)
{
	static int cc = 0;
	struct battery_chg_dev *bcdev = g_bcdev;

	if (!bcdev) {
		return -1;
	}

	cc = bcdev->read_buffer_dump.data_buffer[7];

	return cc;
}

static int fg_sm8350_get_battery_soh(void)
{
	static int soh = 0;
	struct battery_chg_dev *bcdev = g_bcdev;

	if (!bcdev) {
		return -1;
	}

	soh = bcdev->read_buffer_dump.data_buffer[8];

	return soh;
}

static bool fg_sm8350_get_battery_authenticate(void)
{
	int rc = 0;
	struct battery_chg_dev *bcdev = g_bcdev;
	struct psy_state *pst = NULL;

	if (!bcdev) {
		return false;
	}

	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_BATTERY_AUTH);
	else
		rc = read_property_id(bcdev, pst, BATT_BATTERY_AUTH);
	if (rc < 0) {
		chg_err("read battery auth fail, rc=%d\n", rc);
		return false;
	}
	if (bcdev->soccp_support) {
		chg_info("read battery auth success, auth=%d\n", bcdev->oplus_psy.prop[OPLUS_BATTERY_AUTH]);
		return bcdev->oplus_psy.prop[OPLUS_BATTERY_AUTH];
	}
	else {
		chg_info("read battery auth success, auth=%d\n", pst->prop[BATT_BATTERY_AUTH]);
		return pst->prop[BATT_BATTERY_AUTH];
	}
}

static bool fg_sm8350_get_battery_hmac(struct battery_chg_dev *bcdev)
{
	int rc = 0;
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_BATTERY_HMAC);
	else
		rc = read_property_id(bcdev, pst, BATT_BATTERY_HMAC);
	if (rc < 0) {
		chg_err("read battery hmac fail, rc=%d\n", rc);
		return false;
	}
	if (bcdev->soccp_support) {
		chg_info("read battery hmac success, auth=%d\n", bcdev->oplus_psy.prop[OPLUS_BATTERY_HMAC]);
		return bcdev->oplus_psy.prop[OPLUS_BATTERY_HMAC];
	}
	else {
		chg_info("read battery hmac success, auth=%d\n", pst->prop[BATT_BATTERY_HMAC]);
		return pst->prop[BATT_BATTERY_HMAC];
	}
}

static int fg_sm8350_set_battery_full(struct oplus_chg_ic_dev *ic_dev, bool full)
{
	int rc = 0;
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (!bcdev)
		return -ENODEV;

	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_SET_FULL, full);
	else
		rc = write_property_id(bcdev, pst, BATT_SET_BATT_FULL, full);
	chg_info(" rc=%d, full= %d \n", rc, full);

	if(full && bcdev->qcom_gauge_cali_track_support)
		schedule_work(&bcdev->gauge_cali_track_by_full_work);

	return rc;
}

/*
static int fg_sm8350_get_prev_battery_mvolts(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		return -1;
	}

	return chip->batt_volt;
}

static int fg_sm8350_get_prev_battery_temperature(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		return -1;
	}

	return chip->temperature;
}

static int fg_sm8350_get_prev_battery_soc(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		return -1;
	}

	return chip->soc;
}

static int fg_sm8350_get_prev_average_current(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		return -1;
	}

	return chip->icharging;
}

static int fg_sm8350_get_prev_batt_remaining_capacity(void)
{
	return 0;
}
*/
static int fg_sm8350_get_battery_mvolts_2cell_max(void)
{
	return fg_sm8350_get_battery_mvolts_max();
}

static int fg_sm8350_get_battery_mvolts_2cell_min(void)
{
	int volt;

	volt = fg_sm8350_get_battery_mvolts_min();
	if (volt <= 0)
		return fg_sm8350_get_battery_mvolts_max();

	return volt;
}
/*
static int fg_sm8350_get_prev_battery_mvolts_2cell_max(void)
{
	return 4000;
}

static int fg_sm8350_get_prev_battery_mvolts_2cell_min(void)
{
	return 4000;
}
*/
static int fg_bq28z610_modify_dod0(void)
{
	return 0;
}

static int fg_bq28z610_update_soc_smooth_parameter(struct battery_chg_dev *bcdev)
{
	int rc = 0;
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int sleep_mode_status = -1;
	int retry = 0;

	while (retry++ < 3) {
		if (bcdev->soccp_support)
			rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_UPDATE_SOC_SMOOTH_PARAM, 1);
		else
			rc = write_property_id(bcdev, pst, BATT_UPDATE_SOC_SMOOTH_PARAM, 1);

		if (rc) {
			chg_err("set smooth fail, rc=%d\n", rc);
			return -1;
		}

		if (bcdev->soccp_support)
			rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_UPDATE_SOC_SMOOTH_PARAM);
		else
			rc = read_property_id(bcdev, pst, BATT_UPDATE_SOC_SMOOTH_PARAM);

		if (rc) {
			chg_err("read debug reg fail, rc=%d\n", rc);
		} else {
			if (bcdev->soccp_support)
				sleep_mode_status = bcdev->oplus_psy.prop[OPLUS_UPDATE_SOC_SMOOTH_PARAM];
			else
				sleep_mode_status = pst->prop[BATT_UPDATE_SOC_SMOOTH_PARAM];
		}
		chg_info("bq8z610 sleep mode status = %d\n", sleep_mode_status);
		if (sleep_mode_status != 1) {
			chg_err("bq8z610 sleep mode status = %d, retry = %d, enable failed!\n",
				 sleep_mode_status, retry);
			msleep(2000);
			continue;
		} else {
			chg_info("bq8z610 sleep mode status = %d, retry = %d, enable success!\n",
                                 sleep_mode_status, retry);
			return 0;
		}
	}

	return rc;
}

static int fg_bq28z610_get_battery_balancing_status(void)
{
	return 0;
}
#endif /* OPLUS_FEATURE_CHG_BASIC */

#ifdef OPLUS_FEATURE_CHG_BASIC
static ssize_t proc_debug_reg_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	uint8_t ret = 0;
	char page[10];
	int rc = 0;
	int reg_data = 0;
	struct battery_chg_dev *bcdev = g_bcdev;
	struct psy_state *pst = NULL;

	if (!bcdev) {
		chg_err("bcdev is NULL!\n");
		return 0;
	}
	pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_DEBUG_REG);
	else
		rc = read_property_id(bcdev, pst, USB_DEBUG_REG);
	if (rc) {
		chg_err("get enable voocphy fail, rc=%d\n", rc);
	} else {
		chg_err("get enable voocphy success, rc=%d\n", rc);
	}
	if (bcdev->soccp_support)
		reg_data = bcdev->oplus_psy.prop[OPLUS_USB_DEBUG_REG];
	else
		reg_data = pst->prop[USB_DEBUG_REG];

	sprintf(page, "0x%x\n", reg_data);
	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

/*write data 0x1e to reg 0x2a52: echo 0x2a521e > /proc/8350_reg/reg*/
static ssize_t proc_debug_reg_write(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	int rc = 0;
	char buffer[10] = {0};
	int add_data = 0;
	struct battery_chg_dev *bcdev = g_bcdev;
	struct psy_state *pst = NULL;

	if (!bcdev) {
		chg_err("bcdev is NULL!\n");
		return -1;
	}
	pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (count > 10) {
		chg_err("%s: count so len.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buffer, buf, count)) {
		chg_err("%s: read proc input error.\n", __func__);
		return -EFAULT;
	}

	if (1 != sscanf(buffer, "0x%x", &add_data)) {
		chg_err("invalid content: '%s', length = %zd\n", buf, count);
		return -EFAULT;
	}
	chg_info("%s: add:0x%x, data:0x%x\n", __func__, (add_data >> 8) & 0xffff, (add_data & 0xff));

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_DEBUG_REG, add_data);
	else
		rc = write_property_id(bcdev, pst, USB_DEBUG_REG, add_data);
	if (rc) {
		chg_err("set usb_debug_reg fail, rc=%d\n", rc);
	} else {
		chg_err("set usb_debug_reg success, rc=%d\n", rc);
	}

	return count;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations proc_debug_reg_ops =
{
	.read = proc_debug_reg_read,
	.write  = proc_debug_reg_write,
	.open  = simple_open,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops proc_debug_reg_ops =
{
	.proc_write  = proc_debug_reg_write,
	.proc_read  = proc_debug_reg_read,
	.proc_open  = simple_open,
	.proc_lseek = noop_llseek,
};
#endif

#ifdef WLS_QI_DEBUG
static int wls_dbg_icl_ma = 0;
static int wls_dbg_fcc_ma = 0;
static ssize_t proc_icl_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	uint8_t ret = 0;
	char page[16];

	sprintf(page, "%d\n", wls_dbg_icl_ma);
	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_icl_write(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	int rc = 0;
	char buffer[16] = {0};
	int icl_data = 0;
	struct battery_chg_dev *bcdev = g_bcdev;
	struct psy_state *pst = NULL;

	if (!bcdev) {
		chg_err("bcdev is NULL!\n");
		return -ENODEV;
	}
	pst = &bcdev->psy_list[PSY_TYPE_WLS];

	if (count > sizeof(buffer)) {
		chg_err("%s: count > buffer.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buffer, buf, count)) {
		chg_err("%s: read proc icl error.\n", __func__);
		return -EFAULT;
	}

	sscanf(buffer, "%d", &icl_data);
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_DC_INPUT_CURRENT_LIMIT, icl_data * 1000);
	else
		rc = write_property_id(bcdev, pst, WLS_INPUT_CURR_LIMIT, icl_data * 1000);
	if (rc) {
		chg_err("set wls icl fail, rc=%d\n", rc);
	} else {
		wls_dbg_icl_ma = icl_data;
		chg_err("set wls icl[%d]ma success\n", icl_data);
	}

	return count;
}

static const struct proc_ops proc_icl_ops =
{
	.proc_read = proc_icl_read,
	.proc_write  = proc_icl_write,
	.proc_open  = simple_open,
	.proc_lseek = seq_lseek,
};

static ssize_t proc_fcc_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	uint8_t ret = 0;
	char page[16];

	sprintf(page, "%d\n", wls_dbg_fcc_ma);
	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_fcc_write(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	int rc = 0;
	char buffer[16] = {0};
	int fcc_data = 0;
	struct battery_chg_dev *bcdev = g_bcdev;
	struct psy_state *pst = NULL;

	if (!bcdev) {
		chg_err("bcdev is NULL!\n");
		return -ENODEV;
	}
	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if (count > sizeof(buffer)) {
		chg_err("%s: count > buffer.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buffer, buf, count)) {
		chg_err("%s: read proc input error.\n", __func__);
		return -EFAULT;
	}

	sscanf(buffer, "%d", &fcc_data);
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_CHARGE_CONTROL_LIMIT, fcc_data * 1000);
	else
		rc = write_property_id(bcdev, pst, BATT_CHG_CTRL_LIM, fcc_data * 1000);
	if (rc) {
		chg_err("set fcc fail, rc=%d\n", rc);
	} else {
		wls_dbg_fcc_ma = fcc_data;
		chg_err("set fcc[%d]ma success\n", fcc_data);
	}

	return count;
}

static const struct proc_ops proc_fcc_ops =
{
	.proc_read = proc_fcc_read,
	.proc_write  = proc_fcc_write,
	.proc_open  = simple_open,
	.proc_lseek = seq_lseek,
};
#endif /*WLS_QI_DEBUG*/

static int init_debug_reg_proc(struct battery_chg_dev *bcdev)
{
	int ret = 0;
	struct proc_dir_entry *pr_entry_da = NULL;
	struct proc_dir_entry *pr_entry_tmp = NULL;

	pr_entry_da = proc_mkdir("8350_reg", NULL);
	if (pr_entry_da == NULL) {
		ret = -ENOMEM;
		chg_debug("%s: Couldn't create debug_reg proc entry\n", __func__);
	}

	pr_entry_tmp = proc_create_data("reg", 0644, pr_entry_da, &proc_debug_reg_ops, bcdev);
	if (pr_entry_tmp == NULL) {
		ret = -ENOMEM;
		chg_debug("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
	}

#ifdef WLS_QI_DEBUG
	pr_entry_tmp = proc_create_data("icl_ma", 0644, pr_entry_da, &proc_icl_ops, bcdev);
	if (pr_entry_tmp == NULL) {
		ret = -ENOMEM;
		chg_debug("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
	}

	pr_entry_tmp = proc_create_data("fcc_ma", 0644, pr_entry_da, &proc_fcc_ops, bcdev);
	if (pr_entry_tmp == NULL) {
		ret = -ENOMEM;
		chg_debug("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
	}
#endif

	return 0;
}

static int battery_chg_pm_resume(struct device *dev)
{
	struct battery_chg_dev *bcdev = dev_get_drvdata(dev);
	atomic_set(&bcdev->suspended, 0);
	oplus_chg_ic_virq_trigger(bcdev->gauge_ic, OPLUS_IC_VIRQ_RESUME);
	return 0;
}

static int battery_chg_pm_suspend(struct device *dev)
{
	struct battery_chg_dev *bcdev = dev_get_drvdata(dev);
	atomic_set(&bcdev->suspended, 1);
	return 0;
}

static const struct dev_pm_ops battery_chg_pm_ops = {
	.resume		= battery_chg_pm_resume,
	.suspend	= battery_chg_pm_suspend,
};
#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
static void oplus_exit_reverse_chg(void)
{
	struct battery_chg_dev *bcdev = g_bcdev;

	if (bcdev == NULL || bcdev->reverse_chg_ic_dev == NULL) {
		chg_err("bcdev is NULL");
		return;
	}

	bcdev->reverse_enable = false;
	oplus_chg_ic_virq_trigger(bcdev->reverse_chg_ic_dev, OPLUS_IC_VIRQ_REVERSE_ENABLE);
}
static int oplus_chg_ssr_notifier_cb(struct notifier_block *nb,
				     unsigned long code, void *data)
{
	chg_err("code: %lu\n", code);

	switch (code) {
	case QCOM_SSR_BEFORE_SHUTDOWN:
		oplus_turn_off_power_when_adsp_crash();
		oplus_exit_reverse_chg();
		break;
	case QCOM_SSR_AFTER_POWERUP:
		oplus_adsp_crash_recover_work();
		break;
	default:
		break;
	}

	return NOTIFY_DONE;
}

#define QCOM_PLATFORM_FG_TRACK_VER "1.0"
#define OPLUS_GAUGE_CYCLE_COUNT_JUMP_ERR_NUM 3
#define OPLUS_GAUGE_CALI_TRACK_PLUG_TIME_THD_MS (2 * 60 * 1000)
enum oplus_track_item_idx {
	TRACK_ITEM_START = 0,
	TRACK_PRE_VBAT = TRACK_ITEM_START,
	TRACK_CUR_VBAT,
	TRACK_PRE_TBAT,
	TRACK_CUR_TBAT,
	TRACK_PRE_SOC,
	TRACK_CUR_SOC,
	TRACK_BATT_CC,
	TRACK_PRE_LEARN_CAPACITY,
	TRACK_CUR_LEARN_CAPACITY,
	TRACK_PRE_IMP,
	TRACK_CUR_IMP,
	TRACK_PRE_SOH,
	TRACK_CUR_SOH,
	TRACK_ITEM_END
};

const static unsigned int oplus_chg_track_pattern[] = {
	/*plugout*/
	[GAUGE_TRACK_CALI_FLAG_PLUGOUT] =
		BIT(TRACK_PRE_VBAT)            | BIT(TRACK_CUR_VBAT)   | BIT(TRACK_PRE_TBAT)      | BIT(TRACK_CUR_TBAT)           |
		BIT(TRACK_PRE_SOC)             | BIT(TRACK_CUR_SOC)    | BIT(TRACK_BATT_CC)       | BIT(TRACK_PRE_LEARN_CAPACITY) |
		BIT(TRACK_CUR_LEARN_CAPACITY)  | BIT(TRACK_PRE_IMP)    | BIT(TRACK_CUR_IMP)       | BIT(TRACK_PRE_SOH)            |
		BIT(TRACK_CUR_SOH),

	/*full*/
	[GAUGE_TRACK_CALI_FLAG_CHG_FULL] =
		BIT(TRACK_PRE_VBAT)            | BIT(TRACK_CUR_VBAT)   | BIT(TRACK_PRE_TBAT)      | BIT(TRACK_CUR_TBAT)           |
		BIT(TRACK_PRE_SOC)             | BIT(TRACK_CUR_SOC)    | BIT(TRACK_BATT_CC)       | BIT(TRACK_PRE_LEARN_CAPACITY) |
		BIT(TRACK_CUR_LEARN_CAPACITY)  | BIT(TRACK_PRE_IMP)    | BIT(TRACK_CUR_IMP)       | BIT(TRACK_PRE_SOH)            |
		BIT(TRACK_CUR_SOH)
};

static int oplus_plat_cali_info_item_to_val(struct gauge_track_cali_info_s *info,
	enum oplus_track_item_idx idx)
{
	switch (idx) {
	case TRACK_PRE_VBAT:
	case TRACK_CUR_VBAT:
		return info->vbat;
	case TRACK_PRE_TBAT:
	case TRACK_CUR_TBAT:
		return info->tbat;
	case TRACK_PRE_SOC:
	case TRACK_CUR_SOC:
		return info->soc;
	case TRACK_BATT_CC:
		return info->cycle_count;
	case TRACK_PRE_LEARN_CAPACITY:
	case TRACK_CUR_LEARN_CAPACITY:
		return info->learn_capacity;
	case TRACK_PRE_IMP:
	case TRACK_CUR_IMP:
		return info->imp;
	case TRACK_PRE_SOH:
	case TRACK_CUR_SOH:
		return info->soh;
	default:
		return 0;
	}
	return 0;
}

static int oplus_plat_cali_info(struct gauge_track_cali_info_s *pre,
	struct gauge_track_cali_info_s *cur, int reason, char *buf)
{
	int i;
	int index = 0;
	int offset = 0;
	unsigned int pattern;

	pattern = oplus_chg_track_pattern[reason];
	index = scnprintf(buf, OPLUS_CHG_TRACK_PLAT_CALI_INFO_LEN,
			"$$track_reason@@%d$$err_scene@@%s$$info_ver@@%s$$qcom_info@@(",
			reason, "gauge_cali", QCOM_PLATFORM_FG_TRACK_VER);
	for (i = TRACK_ITEM_START; i < TRACK_ITEM_END; i++) {
		if (i != TRACK_ITEM_START)
			index += scnprintf(buf + index, OPLUS_CHG_TRACK_PLAT_CALI_INFO_LEN - index, ",");
		if((pattern & BIT(i)) == 0)
			continue;

		if (i == TRACK_BATT_CC) {
			offset++;
			index += scnprintf(buf + index, OPLUS_CHG_TRACK_PLAT_CALI_INFO_LEN - index,
				"%d", oplus_plat_cali_info_item_to_val(cur, i));
			continue;
		}
		if ((offset + i) % 2 == 0)
			index += scnprintf(buf + index, OPLUS_CHG_TRACK_PLAT_CALI_INFO_LEN - index,
				"%d", oplus_plat_cali_info_item_to_val(pre, i));
		else
			index += scnprintf(buf + index, OPLUS_CHG_TRACK_PLAT_CALI_INFO_LEN - index,
				"%d", oplus_plat_cali_info_item_to_val(cur, i));
	}
	index += scnprintf(buf + index, OPLUS_CHG_TRACK_PLAT_CALI_INFO_LEN - index, ")");

	if (index > OPLUS_CHG_TRACK_PLAT_CALI_INFO_LEN) {
		chg_err("track info exceeds length limit.");
		return -EINVAL;
	}

	return index;
}

static int oplus_plat_trigger_gauge_cali_track(struct gauge_track_cali_info_s *pre_info,
	struct gauge_track_cali_info_s *cur_info, int reason)
{
	char *buf = NULL;
	int len = 0;
	struct battery_chg_dev *bcdev = g_bcdev;

	chg_info("trigger reason:%d\n", reason);

	if (bcdev == NULL)
		return -EINVAL;

	buf = kzalloc(OPLUS_CHG_TRACK_PLAT_CALI_INFO_LEN, GFP_KERNEL);
	if (buf == NULL) {
		chg_err("buf alloc error.\n");
		return -ENOMEM;
	}

	len = oplus_plat_cali_info(pre_info, cur_info, reason, buf);

	if (len > 0) {
		oplus_chg_ic_creat_err_msg(bcdev->gauge_ic, OPLUS_IC_ERR_GAUGE, TRACK_GAGUE_QCOM_CALI_INFO, buf);
		oplus_chg_ic_virq_trigger(bcdev->gauge_ic, OPLUS_IC_VIRQ_ERR);
	}
	kfree(buf);
	return 0;
}

static void oplus_chg_update_gauge_cali_track_info_internal(struct battery_chg_dev *bcdev,
	struct gauge_track_cali_info_s *info)
{
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int rc = 0;

	if (bcdev == NULL || info == NULL || pst == NULL) {
		chg_err("input is null\n");
		return;
	}

	oem_read_buffer(bcdev);
	rc = read_property_id(bcdev, pst, BATT_RESISTANCE);
	if (rc < 0)
		chg_err("get r_final_total fail, rc=%d\n", rc);

	info->tbat = bcdev->read_buffer_dump.data_buffer[0];
	info->vbat = bcdev->read_buffer_dump.data_buffer[2];
	info->soc = oplus_chg_get_battery_soc(bcdev);
	info->cycle_count = bcdev->read_buffer_dump.data_buffer[7];
	info->learn_capacity = bcdev->read_buffer_dump.data_buffer[6];
	info->imp = pst->prop[BATT_RESISTANCE];
	info->soh = bcdev->read_buffer_dump.data_buffer[8];
}

static bool oplus_plat_gauge_cali_changed(struct gauge_track_cali_info_s *pre_info,
					  struct gauge_track_cali_info_s *cur_info)
{
	if ((pre_info->cycle_count - cur_info->cycle_count) > OPLUS_GAUGE_CYCLE_COUNT_JUMP_ERR_NUM ||
	    (pre_info->learn_capacity != cur_info->learn_capacity) ||
	    (pre_info->soh != cur_info->soh)) {
		chg_info("guege_info: pre[%d %d %d %d %d %d %d], cur[%d %d %d %d %d %d %d]",
			 pre_info->tbat, pre_info->vbat, pre_info->soc, pre_info->cycle_count,
			 pre_info->learn_capacity, pre_info->imp, pre_info->soh,
			 cur_info->tbat, cur_info->vbat, cur_info->soc, cur_info->cycle_count,
                         cur_info->learn_capacity, cur_info->imp, cur_info->soh);
		return true;
	}
	return false;
}

static struct gauge_track_cali_info_s pre_info;
static struct gauge_track_cali_info_s cur_info;
static void oplus_plat_gauge_cali_track_by_plug_work(struct work_struct *work)
{
	static ktime_t online_time;
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, gauge_cali_track_by_plug_work);

	if (bcdev == NULL || bcdev->pre_info == NULL)
		return;

	if (bcdev->usb_in_status) {
		online_time = ktime_get();
		mutex_lock(&bcdev->pre_info_lock);
		oplus_chg_update_gauge_cali_track_info_internal(bcdev, &pre_info);
		mutex_unlock(&bcdev->pre_info_lock);
		bcdev->pre_info = &pre_info;
	} else {
		if(ktime_ms_delta(ktime_get(), online_time) < OPLUS_GAUGE_CALI_TRACK_PLUG_TIME_THD_MS)
			return;

		mutex_lock(&bcdev->cur_info_lock);
		oplus_chg_update_gauge_cali_track_info_internal(bcdev, &cur_info);
		mutex_unlock(&bcdev->cur_info_lock);
		if (oplus_plat_gauge_cali_changed(&pre_info, &cur_info)) {
			oplus_plat_trigger_gauge_cali_track(&pre_info, &cur_info,
							    GAUGE_TRACK_CALI_FLAG_PLUGOUT);
			bcdev->pre_info = &cur_info;
		}
	}
}

static void oplus_plat_gauge_cali_track_by_full_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, gauge_cali_track_by_full_work);

	if (bcdev == NULL || bcdev->pre_info == NULL)
		return;

	mutex_lock(&bcdev->cur_info_lock);
	oplus_chg_update_gauge_cali_track_info_internal(bcdev, &cur_info);
	mutex_unlock(&bcdev->cur_info_lock);
	if (oplus_plat_gauge_cali_changed(bcdev->pre_info, &cur_info)) {
		oplus_plat_trigger_gauge_cali_track(bcdev->pre_info, &cur_info,
						    GAUGE_TRACK_CALI_FLAG_CHG_FULL);
		bcdev->pre_info = &cur_info;
	}
}
#endif

static int oplus_chg_8350_init(struct oplus_chg_ic_dev *ic_dev)
{
	ic_dev->online = true;
	return 0;
}

static int oplus_chg_8350_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (!ic_dev->online)
		return 0;

	ic_dev->online = false;
	return 0;
}

static int oplus_chg_8350_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	struct battery_chg_dev *bcdev;
	const int extra_num = 16;
	bool chg_en = false;
	int chg_type;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	oplus_chg_8350_output_is_suspend(ic_dev, &chg_en);
	oplus_chg_8350_get_charger_type(ic_dev, &chg_type);
	oem_read_buffer(bcdev);
	if (bcdev->qcom_gauge_cali_track_support != bcdev->read_buffer_dump.data_buffer[14]) {
		bcdev->qcom_gauge_cali_track_support = bcdev->read_buffer_dump.data_buffer[14];
		chg_info("update qcom_gauge_cali_track_support:%d", bcdev->qcom_gauge_cali_track_support);
	}
	chg_info("sm8450_st_dump: [chg_en=%d, suspend=%d, pd_svooc=%d], subtype=0x%02x],"
			"[oplus_UsbCommCapable=%d, oplus_pd_svooc=%d, typec_mode=%d, cid_status=0x%02x, usb_in_status=%d],"
			"[0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x], "
			"[0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x], "
			"[0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x], "
			"[0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x], "
			"[0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x], "
			"[0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x], "
			"[0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x], "
			"[0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x], "
			"[0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x], "
			"[0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x], "
			"[0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x], "
			"[0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x], "
			"[0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x, 0x%4x=0x%02x], "
			"[0x%4x=0x%02x], \n",
	 chg_en, bcdev->read_buffer_dump.data_buffer[9], bcdev->read_buffer_dump.data_buffer[11], chg_type,
	 bcdev->read_buffer_dump.data_buffer[10], bcdev->read_buffer_dump.data_buffer[11],
	 bcdev->read_buffer_dump.data_buffer[12], bcdev->cid_status, bcdev->usb_in_status,
	 bcdev->read_buffer_dump.data_buffer[extra_num - 1], bcdev->read_buffer_dump.data_buffer[extra_num],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 1], bcdev->read_buffer_dump.data_buffer[extra_num + 2],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 3], bcdev->read_buffer_dump.data_buffer[extra_num + 4],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 5], bcdev->read_buffer_dump.data_buffer[extra_num + 6],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 7], bcdev->read_buffer_dump.data_buffer[extra_num + 8],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 9], bcdev->read_buffer_dump.data_buffer[extra_num + 10],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 11], bcdev->read_buffer_dump.data_buffer[extra_num + 12],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 13], bcdev->read_buffer_dump.data_buffer[extra_num + 14],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 15], bcdev->read_buffer_dump.data_buffer[extra_num + 16],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 17], bcdev->read_buffer_dump.data_buffer[extra_num + 18],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 19], bcdev->read_buffer_dump.data_buffer[extra_num + 20],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 21], bcdev->read_buffer_dump.data_buffer[extra_num + 22],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 23], bcdev->read_buffer_dump.data_buffer[extra_num + 24],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 25], bcdev->read_buffer_dump.data_buffer[extra_num + 26],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 27], bcdev->read_buffer_dump.data_buffer[extra_num + 28],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 29], bcdev->read_buffer_dump.data_buffer[extra_num + 30],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 31], bcdev->read_buffer_dump.data_buffer[extra_num + 32],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 33], bcdev->read_buffer_dump.data_buffer[extra_num + 34],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 35], bcdev->read_buffer_dump.data_buffer[extra_num + 36],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 37], bcdev->read_buffer_dump.data_buffer[extra_num + 38],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 39], bcdev->read_buffer_dump.data_buffer[extra_num + 40],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 41], bcdev->read_buffer_dump.data_buffer[extra_num + 42],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 43], bcdev->read_buffer_dump.data_buffer[extra_num + 44],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 45], bcdev->read_buffer_dump.data_buffer[extra_num + 46],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 47], bcdev->read_buffer_dump.data_buffer[extra_num + 48],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 49], bcdev->read_buffer_dump.data_buffer[extra_num + 50],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 51], bcdev->read_buffer_dump.data_buffer[extra_num + 52],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 53], bcdev->read_buffer_dump.data_buffer[extra_num + 54],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 55], bcdev->read_buffer_dump.data_buffer[extra_num + 56],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 57], bcdev->read_buffer_dump.data_buffer[extra_num + 58],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 59], bcdev->read_buffer_dump.data_buffer[extra_num + 60],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 61], bcdev->read_buffer_dump.data_buffer[extra_num + 62],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 63], bcdev->read_buffer_dump.data_buffer[extra_num + 64],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 65], bcdev->read_buffer_dump.data_buffer[extra_num + 66],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 67], bcdev->read_buffer_dump.data_buffer[extra_num + 68],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 69], bcdev->read_buffer_dump.data_buffer[extra_num + 70],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 71], bcdev->read_buffer_dump.data_buffer[extra_num + 72],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 73], bcdev->read_buffer_dump.data_buffer[extra_num + 74],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 75], bcdev->read_buffer_dump.data_buffer[extra_num + 76],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 77], bcdev->read_buffer_dump.data_buffer[extra_num + 78],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 79], bcdev->read_buffer_dump.data_buffer[extra_num + 80],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 81], bcdev->read_buffer_dump.data_buffer[extra_num + 82],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 83], bcdev->read_buffer_dump.data_buffer[extra_num + 84],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 85], bcdev->read_buffer_dump.data_buffer[extra_num + 86],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 87], bcdev->read_buffer_dump.data_buffer[extra_num + 88],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 89], bcdev->read_buffer_dump.data_buffer[extra_num + 90],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 91], bcdev->read_buffer_dump.data_buffer[extra_num + 92],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 93], bcdev->read_buffer_dump.data_buffer[extra_num + 94],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 95], bcdev->read_buffer_dump.data_buffer[extra_num + 96],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 97], bcdev->read_buffer_dump.data_buffer[extra_num + 98],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 99], bcdev->read_buffer_dump.data_buffer[extra_num + 100],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 101], bcdev->read_buffer_dump.data_buffer[extra_num + 102],
	 bcdev->read_buffer_dump.data_buffer[extra_num + 103], bcdev->read_buffer_dump.data_buffer[extra_num + 104]);

	return 0;
}

static int  oplus_chg_8350_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	return 0;
}

static int oplus_chg_8350_input_present(struct oplus_chg_ic_dev *ic_dev, bool *present)
{
	struct battery_chg_dev *bcdev;
	bool vbus_rising = false;
	static bool pre_vbus_rising = false;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_IN_STATUS);
	else
		rc = read_property_id(bcdev, pst, USB_IN_STATUS);
	if (rc < 0) {
		chg_err("read usb vbus_rising fail, rc=%d\n", rc);
		return rc;
	}

	if (bcdev->soccp_support)
		vbus_rising = bcdev->oplus_psy.prop[OPLUS_USB_IN_STATUS];
	else
		vbus_rising = pst->prop[USB_IN_STATUS];

	if (vbus_rising && oplus_chg_wls_is_present(bcdev)) {
		chg_info("USBIN high but wls present\n");
		vbus_rising = false;
	}

	*present = vbus_rising;
	if (pre_vbus_rising != vbus_rising) {
		pre_vbus_rising = vbus_rising;
		chg_info("vbus_rising=%d\n", vbus_rising);
		if (!vbus_rising && bcdev->usb_in_status &&
		    !work_busy(&bcdev->check_abnormal_usbin_status_work.work))
			schedule_delayed_work(&bcdev->check_abnormal_usbin_status_work,
				msecs_to_jiffies(ABNORMAL_CHECK_USBIN_DELAY_MS));
	}

	/* Fix the issue of not charging or not enter vooc/svooc when adsp reboot */
	if (bcdev->adsp_reboot_discnt_chg_support &&
	    vbus_rising && atomic_read(&bcdev->adsp_reboot) == 1) {
		chg_info("force vbus_rising[%d], present as 0\n", vbus_rising);
		vbus_rising = 0;
		*present = false;
	}

	return vbus_rising;
}

static void oplus_publish_usbin_abnormal_item(struct battery_chg_dev *bcdev)
{
	struct mms_msg *msg;
	int rc;

	if (!is_err_topic_available(bcdev)) {
		chg_err("error topic not found\n");
		return;
	}

	msg = oplus_mms_alloc_int_msg(MSG_TYPE_ITEM,
		MSG_PRIO_MEDIUM, ERR_ITEM_USBIN_ABNORMAL, 1);
	if (msg == NULL) {
		chg_err("alloc usbin_abnormal msg error\n");
		return;
	}

	rc = oplus_mms_publish_msg(bcdev->err_topic, msg);
	if (rc < 0) {
		chg_err("publish usbin_abnormal msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_update_pd_completed_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, update_pd_completed_work.work);
	int rc = 0;

	if (bcdev->soccp_support) {
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_CHECK_PD_COMPLETED);
		if (rc < 0) {
			chg_err("read OPLUS_GET_PD_SVOOC fail\n");
			return;
		}
		bcdev->pd_check_completed = bcdev->oplus_psy.prop[OPLUS_CHECK_PD_COMPLETED];
		chg_info("pd completed %d\n", bcdev->pd_check_completed);
		if (bcdev->pd_check_completed) {
			oplus_chg_ic_virq_trigger(bcdev->buck_ic, OPLUS_IC_VIRQ_PD_COMPLETED);
			chg_info("pd or pps send pd completed\n");
		}
	}
}

static void oplus_check_abnormal_usbin_status_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, check_abnormal_usbin_status_work.work);
	bool vbus_rising;

	if (!bcdev->usb_in_status) {
		chg_info("usb_in_status=0, return\n");
		bcdev->abnormal_usbin_count = 0;
		return;
	}

	oplus_chg_8350_input_present(bcdev->buck_ic, &vbus_rising);
	if (!vbus_rising && bcdev->usb_in_status) {
		bcdev->abnormal_usbin_count++;
		chg_info("abnormal detected, count=%d\n", bcdev->abnormal_usbin_count);

		if (bcdev->abnormal_usbin_count >= ABNORMAL_CHECK_USBIN_CNT) {
			chg_info("schedule plugin_irq_work\n");
			oplus_publish_usbin_abnormal_item(bcdev);
			schedule_delayed_work(&bcdev->plugin_irq_work, 0);
			bcdev->abnormal_usbin_count = 0;
			return;
		}
	} else {
		chg_info("vbus_rising=%d bcdev->usb_in_status=%d\n", vbus_rising, bcdev->usb_in_status);
		bcdev->abnormal_usbin_count = 0;
		return;
	}

	schedule_delayed_work(&bcdev->check_abnormal_usbin_status_work,
		msecs_to_jiffies(ABNORMAL_CHECK_USBIN_DELAY_MS));
}

static int oplus_chg_8350_input_suspend(struct oplus_chg_ic_dev *ic_dev, bool suspend)
{
	struct battery_chg_dev *bcdev;
	int prop_id = 0;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_USB];

	prop_id = get_property_id(pst, POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT);
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_INPUT_CURRENT_LIMIT, suspend ? 0 : 0xFFFFFFFF);
	else
		rc = write_property_id(bcdev, pst, prop_id, suspend ? 0 : 0xFFFFFFFF);
	chg_err("set %s, rc=%d\n",
			suspend ? "suspend" : "unsuspend", rc);

	return rc;
}

static int oplus_chg_8350_input_is_suspend(struct oplus_chg_ic_dev *ic_dev, bool *suspend)
{
	return 0;
}

static int oplus_chg_8350_output_suspend(struct oplus_chg_ic_dev *ic_dev, bool suspend)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	int is_rf_ftm_mode;
	int rc = 0;
	bool en_change = false;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	is_rf_ftm_mode = oplus_is_rf_ftm_mode();

	if (!suspend && is_rf_ftm_mode) {
		chg_info("is_rf_ftm_mode, force disable charger");
		if (bcdev->soccp_support)
			rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_CHG_EN, 0);
		else
			rc = write_property_id(bcdev, pst, BATT_CHG_EN, 0);
		mutex_lock(&bcdev->chg_en_lock);
		if (bcdev->chg_en != false)
			en_change = true;
		bcdev->chg_en = false;
		mutex_unlock(&bcdev->chg_en_lock);
		chg_err("set suspend charging, rc=%d\n", rc);
	} else {
		if (bcdev->soccp_support)
			rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_CHG_EN, suspend ? 0 : 1);
		else
			rc = write_property_id(bcdev, pst, BATT_CHG_EN,
				       suspend ? 0 : 1);
		mutex_lock(&bcdev->chg_en_lock);
		if (bcdev->chg_en != !suspend)
			en_change = true;
		bcdev->chg_en = suspend ? 0 : 1;
		mutex_unlock(&bcdev->chg_en_lock);
		chg_err("set %s charging, rc=%d\n",
				suspend ? "suspend" : "unsuspend", rc);
	}
	if (en_change)
		mod_delayed_work(system_highpri_wq, &bcdev->ctrl_lcm_frequency, 0);

	return rc;
}

static int oplus_chg_8350_output_is_suspend(struct oplus_chg_ic_dev *ic_dev, bool *suspend)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_CHG_EN);
	else
		rc = read_property_id(bcdev, pst, BATT_CHG_EN);
	if (rc) {
		chg_err("get battery charging status fail, rc=%d\n", rc);
		return rc;
	}
	if (bcdev->soccp_support)
		*suspend = !!bcdev->oplus_psy.prop[OPLUS_CHG_EN];
	else
		*suspend = !!pst->prop[BATT_CHG_EN];

	return rc;
}

static int smbchg_get_charge_enable(void)
{
	int rc = 0;
	struct battery_chg_dev *bcdev = g_bcdev;

	if (!bcdev) {
		chg_err("bcdev is null\n");
		return rc;
	}

	mutex_lock(&bcdev->chg_en_lock);
	rc = bcdev->chg_en;
	mutex_unlock(&bcdev->chg_en_lock);

	return rc;
}

static int qpnp_get_prop_charger_voltage_now(struct battery_chg_dev *bcdev)
{
	int rc = 0;
	int prop_id = 0;
	static int vbus_volt = 0;
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];

	prop_id = get_property_id(pst, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0) {
		chg_err("read usb vbus_volt fail, rc=%d\n", rc);
		return vbus_volt;
	}
	vbus_volt = pst->prop[prop_id] / 1000;

	return vbus_volt;
}

static int usb_icl[] = {
	300, 500, 900, 1200, 1350, 1500, 1750, 2000, 2500, 3000,
};

static bool qpnp_get_prop_vbus_collapse_status(struct battery_chg_dev *bcdev)
{
	int rc = 0;
	bool collapse_status = false;
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];

	pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_VBUS_COLLAPSE_STATUS);
	else
		rc = read_property_id(bcdev, pst, USB_VBUS_COLLAPSE_STATUS);
	if (rc < 0) {
		chg_err("read usb vbus_collapse_status fail, rc=%d\n", rc);
		return false;
	}
	if (bcdev->soccp_support)
		collapse_status = bcdev->oplus_psy.prop[OPLUS_USB_VBUS_COLLAPSE_STATUS];
	else
		collapse_status = pst->prop[USB_VBUS_COLLAPSE_STATUS];

	chg_info("read usb vbus_collapse_status[%d]\n",
			collapse_status);
	return collapse_status;
}

static int oplus_input_current_limit_ctrl_by_vooc_write(struct oplus_chg_ic_dev *ic_dev, int current_ma)
{
	struct battery_chg_dev *bcdev;
	int rc;
	int cur_usb_icl = 0;
	int temp_curr;

	rc = oplus_chg_8350_get_icl(ic_dev, &cur_usb_icl);
	chg_info(" get cur_usb_icl = %d\n", cur_usb_icl);
	if (rc)
		return rc;

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (current_ma > cur_usb_icl) {
		for (temp_curr = cur_usb_icl; temp_curr < current_ma; temp_curr += 500) {
			msleep(35);
			rc = oplus_chg_set_input_current_with_no_aicl(bcdev, temp_curr);
			chg_info("[up] set input_current = %d\n", temp_curr);
		}
	} else {
		for (temp_curr = cur_usb_icl; temp_curr > current_ma; temp_curr -= 500) {
			msleep(35);
			rc = oplus_chg_set_input_current_with_no_aicl(bcdev, temp_curr);
			chg_info("[down] set input_current = %d\n", temp_curr);
		}
	}

	rc = oplus_chg_set_input_current_with_no_aicl(bcdev, current_ma);
	return rc;
}

static int oplus_chg_usb_set_input_current(struct battery_chg_dev *bcdev, int current_ma,
	int aicl_point)
{
	int rc = 0, i = 0;
	int chg_vol = 0;
	int prop_id = 0;
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	bool pre_step = false;

	prop_id = get_property_id(pst, POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT);
	for (i = 1; i <= current_ma / 100; i++) {
		if (bcdev->soccp_support)
			rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_INPUT_CURRENT_LIMIT, i * 100000);
		else
			rc = write_property_id(bcdev, pst, prop_id, i * 100000);
		if (rc) {
			chg_err("set icl to %d mA fail, rc=%d\n", i * 100, rc);
			return rc;
		} else {
			chg_info("set icl to %d mA\n", i * 100);
		}
		usleep_range(50000, 51000);
		if (qpnp_get_prop_vbus_collapse_status(bcdev) == true) {
			i = i - 1;
			pre_step = true;
			break;
		}
		chg_vol = qpnp_get_prop_charger_voltage_now(bcdev);
		if (chg_vol < aicl_point) {
			i = i - 1;
			pre_step = true;
			break;
		}
		if (i == current_ma / 100)
			break;
	}
	if (i <= 0)
		i = 1;
	if (pre_step) {
		if (bcdev->soccp_support)
			rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_INPUT_CURRENT_LIMIT, i * 100000);
		else
			rc = write_property_id(bcdev, pst, prop_id, i * 100000);
		if (rc) {
			chg_err("set icl to %d mA fail, rc=%d\n", i * 100, rc);
			return rc;
		} else {
			chg_info("set icl to %d mA\n", i * 100);
		}
	}
	chg_info("usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_end\n",
		 chg_vol, i, i * 100, aicl_point);

	return rc;
}

static int oplus_chg_get_aicl_thr_index(struct battery_chg_dev *bcdev, int vbatt)
{
	int index = 0;
	for (index = 0; index < AICL_POINT_INDEX_MAX; index++) {
		if (vbatt <= bcdev->aicl_thr_table[index].vbat_thr)
			break;
	}
	if (index >= AICL_POINT_INDEX_MAX - 1)
		index = AICL_POINT_INDEX_MAX - 1;
	return index;
}

#define UNKONW_CURR 500
#define DEFAULT_CURR_BY_CC 100
static int oplus_chg_set_input_current(struct battery_chg_dev *bcdev, int current_ma)
{
	int rc = 0, i = 0;
	int chg_vol = 0;
	int aicl_point = 0;
	int prop_id = 0;
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	struct oplus_mms *gauge_topic;
	bool present;
	int max_pdo_current;
	int batt_volt;
	int type = 0;
	int aicl_thr_index = 0;

	prop_id = get_property_id(pst, POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT);

	chg_info("usb input max current limit=%d setting %02x\n", current_ma, i);
	gauge_topic = oplus_mms_get_by_name("gauge");
	if (gauge_topic) {
		union mms_msg_data data = {0};

		oplus_mms_get_item_data(gauge_topic,
					GAUGE_ITEM_VOL_MAX, &data, false);
		batt_volt = data.intval;
		if (bcdev->aicl_thr_table_init) {
			aicl_thr_index = oplus_chg_get_aicl_thr_index(bcdev, batt_volt);
			aicl_point = bcdev->aicl_thr_table[aicl_thr_index].sw_aicl;
		} else {
			if (batt_volt > 4100) {
				aicl_point = 4550;
			} else {
				aicl_point = 4500;
			}
		}
	} else {
		chg_info("gauge_topic is null, use default aicl_point 4500\n");
		aicl_point = 4500;
	}
	if (bcdev->usb_aicl_enhance) {
		oplus_chg_8350_input_present(bcdev->buck_ic, &present);
		oplus_chg_8350_get_charger_type(bcdev->buck_ic, &type);
		if ((bcdev->charger_type == OPLUS_CHG_USB_TYPE_SDP ||
		     bcdev->charger_type == OPLUS_CHG_USB_TYPE_CDP ||
		     (bcdev->charger_type == OPLUS_CHG_USB_TYPE_UNKNOWN && current_ma == UNKONW_CURR)) &&
		    present) {
			if (bcdev->charger_type == OPLUS_CHG_USB_TYPE_SDP) {
				aicl_point = USB_SW_AICL_POINT;
				oplus_chg_set_aicl_point(bcdev->buck_ic, batt_volt);
			}
			rc = oplus_chg_usb_set_input_current(bcdev, current_ma, aicl_point);
			goto aicl_return;
		}
	}

	if (oplus_chg_get_common_charge_icl_support_flags()) {
		max_pdo_current = oplus_get_max_current_from_fixed_pdo(bcdev, bcdev->pd_chg_volt);
		chg_info("max_pdo_current:%d ma\n", max_pdo_current);

		if (max_pdo_current >= 0)
			current_ma = min(current_ma, max_pdo_current);
		if (current_ma < DEFAULT_CURR_BY_CC) {
			cancel_delayed_work(&bcdev->sourcecap_suspend_recovery_work);
			oplus_chg_suspend_charger(true, PD_PDO_ICL_VOTER);
			schedule_delayed_work(&bcdev->sourcecap_suspend_recovery_work,
				msecs_to_jiffies(SUSPEND_RECOVERY_DELAY_MS));
			goto aicl_return;
		} else if (current_ma < usb_icl[0]) {
			oplus_chg_suspend_charger(false, PD_PDO_ICL_VOTER);
			goto common_charge_aicl_end;
		} else {
			oplus_chg_suspend_charger(false, PD_PDO_ICL_VOTER);
		}
	}

	if (current_ma < 500) {
		i = 0;
		goto aicl_end;
	}

	i = 1; /* 500 */
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_INPUT_CURRENT_LIMIT, usb_icl[i] * 1000);
	else
		rc = write_property_id(bcdev, pst, prop_id, usb_icl[i] * 1000);
	if (rc) {
		chg_err("set icl to %d mA fail, rc=%d\n", usb_icl[i], rc);
	} else {
		chg_err("set icl to %d mA\n", usb_icl[i]);
	}
	usleep_range(50000, 51000);
	if (qpnp_get_prop_vbus_collapse_status(bcdev) == true) {
		if (bcdev->rerun_max > 0 && bcdev->usb_in_status) {
			schedule_delayed_work(&bcdev->vbus_collapse_rerun_icl_work,
				msecs_to_jiffies(3000)); /* vbus_collapse_status resumes after three seconds */
			bcdev->rerun_max--;
		}
		goto aicl_boost_back;
	}
	chg_vol = qpnp_get_prop_charger_voltage_now(bcdev);
	if (chg_vol < aicl_point) {
		chg_debug("use 500 here\n");
		goto aicl_end;
	} else if (current_ma < 900)
		goto aicl_end;

	i = 2; /* 900 */
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_INPUT_CURRENT_LIMIT, usb_icl[i] * 1000);
	else
		rc = write_property_id(bcdev, pst, prop_id, usb_icl[i] * 1000);
	if (rc) {
		chg_err("set icl to %d mA fail, rc=%d\n", usb_icl[i], rc);
	} else {
		chg_err("set icl to %d mA\n", usb_icl[i]);
	}
	usleep_range(50000, 51000);
	if (qpnp_get_prop_vbus_collapse_status(bcdev) == true) {
		i = i - 1;
		goto aicl_boost_back;
	}
	chg_vol = qpnp_get_prop_charger_voltage_now(bcdev);
	if (chg_vol < aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (current_ma < 1200)
		goto aicl_end;

	i = 3; /* 1200 */
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_INPUT_CURRENT_LIMIT, usb_icl[i] * 1000);
	else
		rc = write_property_id(bcdev, pst, prop_id, usb_icl[i] * 1000);
	if (rc) {
		chg_err("set icl to %d mA fail, rc=%d\n", usb_icl[i], rc);
	} else {
		chg_err("set icl to %d mA\n", usb_icl[i]);
	}
	usleep_range(50000, 51000);
	if (qpnp_get_prop_vbus_collapse_status(bcdev) == true) {
		i = i - 1;
		goto aicl_boost_back;
	}
	chg_vol = qpnp_get_prop_charger_voltage_now(bcdev);
	if (chg_vol < aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	}

	i = 4; /* 1350 */
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_INPUT_CURRENT_LIMIT, usb_icl[i] * 1000);
	else
		rc = write_property_id(bcdev, pst, prop_id, usb_icl[i] * 1000);
	if (rc) {
		chg_err("set icl to %d mA fail, rc=%d\n", usb_icl[i], rc);
	} else {
		chg_err("set icl to %d mA\n", usb_icl[i]);
	}
	usleep_range(50000, 51000);
	if (qpnp_get_prop_vbus_collapse_status(bcdev) == true) {
		i = i - 1;
		goto aicl_boost_back;
	}
	chg_vol = qpnp_get_prop_charger_voltage_now(bcdev);
	if (chg_vol < aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	}

	i = 5; /* 1500 */
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_INPUT_CURRENT_LIMIT, usb_icl[i] * 1000);
	else
		rc = write_property_id(bcdev, pst, prop_id, usb_icl[i] * 1000);
	if (rc) {
		chg_err("set icl to %d mA fail, rc=%d\n", usb_icl[i], rc);
	} else {
		chg_err("set icl to %d mA\n", usb_icl[i]);
	}
	usleep_range(50000, 51000);
	if (qpnp_get_prop_vbus_collapse_status(bcdev) == true) {
		i = i - 2;
		goto aicl_boost_back;
	}
	chg_vol = qpnp_get_prop_charger_voltage_now(bcdev);
	if (chg_vol < aicl_point) {
		i = i - 2; /*We DO NOT use 1.2A here*/
		goto aicl_pre_step;
	} else if (current_ma < 1500) {
		i = i - 2; /*We use 1.2A here*/
		goto aicl_end;
	} else if (current_ma < 2000)
		goto aicl_end;

	i = 6; /* 1750 */
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_INPUT_CURRENT_LIMIT, usb_icl[i] * 1000);
	else
		rc = write_property_id(bcdev, pst, prop_id, usb_icl[i] * 1000);
	if (rc) {
		chg_err("set icl to %d mA fail, rc=%d\n", usb_icl[i], rc);
	} else {
		chg_err("set icl to %d mA\n", usb_icl[i]);
	}
	usleep_range(50000, 51000);
	if (qpnp_get_prop_vbus_collapse_status(bcdev) == true) {
		i = i - 3;
		goto aicl_boost_back;
	}
	chg_vol = qpnp_get_prop_charger_voltage_now(bcdev);
	if (chg_vol < aicl_point) {
		i = i - 3; /*1.2*/
		goto aicl_pre_step;
	}

	i = 7; /* 2000 */
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_INPUT_CURRENT_LIMIT, usb_icl[i] * 1000);
	else
		rc = write_property_id(bcdev, pst, prop_id, usb_icl[i] * 1000);

	if (rc) {
		chg_err("set icl to %d mA fail, rc=%d\n", usb_icl[i], rc);
	} else {
		chg_err("set icl to %d mA\n", usb_icl[i]);
	}
	usleep_range(50000, 51000);
	if (qpnp_get_prop_vbus_collapse_status(bcdev) == true) {
		i = i - 2;
		goto aicl_boost_back;
	}
	chg_vol = qpnp_get_prop_charger_voltage_now(bcdev);
	if (chg_vol < aicl_point) {
		i =  i - 2; /*1.5*/
		goto aicl_pre_step;
	} else if (current_ma < 2500)
		goto aicl_end;

	i = 8; /* 2500 */
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_INPUT_CURRENT_LIMIT, usb_icl[i] * 1000);
	else
		rc = write_property_id(bcdev, pst, prop_id, usb_icl[i] * 1000);
	if (rc)
		chg_err("set icl to %d mA fail, rc=%d\n", usb_icl[i], rc);
	else
		chg_info("set icl to %d mA\n", usb_icl[i]);
	usleep_range(50000, 51000);
	if (qpnp_get_prop_vbus_collapse_status(bcdev) == true) {
		i = i - 1;
		goto aicl_boost_back;
	}
	chg_vol = qpnp_get_prop_charger_voltage_now(bcdev);
	if (chg_vol < aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (current_ma < 3000) {
		goto aicl_end;
	}

	i = 9; /* 3000 */
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_INPUT_CURRENT_LIMIT, usb_icl[i] * 1000);
	else
		rc = write_property_id(bcdev, pst, prop_id, usb_icl[i] * 1000);
	if (rc) {
		chg_err("set icl to %d mA fail, rc=%d\n", usb_icl[i], rc);
	} else {
		chg_err("set icl to %d mA\n", usb_icl[i]);
	}
	usleep_range(50000, 51000);
	if (qpnp_get_prop_vbus_collapse_status(bcdev) == true) {
		i = i - 1;
		goto aicl_boost_back;
	}
	chg_vol = qpnp_get_prop_charger_voltage_now(bcdev);
	if (chg_vol < aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (current_ma >= 3000)
		goto aicl_end;

aicl_pre_step:
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_INPUT_CURRENT_LIMIT, usb_icl[i] * 1000);
	else
		rc = write_property_id(bcdev, pst, prop_id, usb_icl[i] * 1000);
	if (rc) {
		chg_err("set icl to %d mA fail, rc=%d\n", usb_icl[i], rc);
	} else {
		chg_err("set icl to %d mA\n", usb_icl[i]);
	}
	chg_info("usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_pre_step\n", chg_vol, i, usb_icl[i], aicl_point);
	goto aicl_return;
aicl_end:
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_INPUT_CURRENT_LIMIT, usb_icl[i] * 1000);
	else
		rc = write_property_id(bcdev, pst, prop_id, usb_icl[i] * 1000);
	if (rc) {
		chg_err("set icl to %d mA fail, rc=%d\n", usb_icl[i], rc);
	} else {
		chg_err("set icl to %d mA\n", usb_icl[i]);
	}
	chg_info("usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_end\n", chg_vol, i, usb_icl[i], aicl_point);
	goto aicl_return;
aicl_boost_back:
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_INPUT_CURRENT_LIMIT, usb_icl[i] * 1000);
	else
		rc = write_property_id(bcdev, pst, prop_id, usb_icl[i] * 1000);
	if (rc) {
		chg_err("set icl to %d mA fail, rc=%d\n", usb_icl[i], rc);
	} else {
		chg_err("set icl to %d mA\n", usb_icl[i]);
	}
	chg_info("usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_boost_back\n", chg_vol, i, usb_icl[i], aicl_point);
	goto aicl_return;
common_charge_aicl_end:
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_INPUT_CURRENT_LIMIT, DEFAULT_CURR_BY_CC * 1000);
	else
		rc = write_property_id(bcdev, pst, prop_id, DEFAULT_CURR_BY_CC * 1000);
	chg_info("common_charge_aicl_end set icl:%d mA, rc=%d\n", DEFAULT_CURR_BY_CC, rc);
	goto aicl_return;
aicl_return:
	return rc;
}

static void oplus_vbus_collapse_rerun_icl_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, vbus_collapse_rerun_icl_work.work);
	struct votable *icl_votable = find_votable("WIRED_ICL");

	if (!bcdev->usb_in_status) {
		chg_info("usb unpluged, return\n");
		return;
	}

	chg_info("retun icl\n");
	if (icl_votable)
		rerun_election(icl_votable, true);
}

static int oplus_chg_8350_set_icl(struct oplus_chg_ic_dev *ic_dev,
				  bool vooc_mode, bool step, int icl_ma)
{
	struct battery_chg_dev *bcdev;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (!bcdev->bypass_vooc_support && vooc_mode && icl_ma > 0)
		return oplus_input_current_limit_ctrl_by_vooc_write(ic_dev, icl_ma);

	if (step)
		rc = oplus_chg_set_input_current(bcdev, icl_ma);
	else
		rc = oplus_chg_set_input_current_with_no_aicl(bcdev, icl_ma);

	if (rc)
		chg_err("set icl to %d mA fail, rc=%d\n", icl_ma, rc);
	else
		chg_info("set icl to %d mA\n", icl_ma);

	return rc;
}

static int oplus_chg_8350_get_icl(struct oplus_chg_ic_dev *ic_dev, int *icl_ma)
{
	struct battery_chg_dev *bcdev;
	int prop_id = 0;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_USB];

	prop_id = get_property_id(pst, POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT);
	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_INPUT_CURRENT_LIMIT);
	else
		rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0) {
		chg_err("read usb icl fail, rc=%d\n", rc);
		return rc;
	}
	if (bcdev->soccp_support)
		*icl_ma = DIV_ROUND_CLOSEST((int)bcdev->oplus_psy.prop[OPLUS_USB_INPUT_CURRENT_LIMIT], 1000);
	else
		*icl_ma = DIV_ROUND_CLOSEST((int)pst->prop[prop_id], 1000);

	return 0;
}

static int oplus_chg_8350_set_fcc(struct oplus_chg_ic_dev *ic_dev, int fcc_ma)
{
	struct battery_chg_dev *bcdev;
	int prop_id = 0;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

#ifdef WLS_QI_DEBUG
	if (wls_dbg_fcc_ma != 0)
		fcc_ma = wls_dbg_fcc_ma;
#endif

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	prop_id = get_property_id(pst, POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT);
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_CHARGE_CONTROL_LIMIT, fcc_ma * 1000);
	else
		rc = write_property_id(bcdev, pst, prop_id, fcc_ma * 1000);
	if (rc)
		chg_err("set fcc to %d mA fail, rc=%d\n", fcc_ma, rc);

	return rc;
}

static int oplus_chg_8350_set_fv(struct oplus_chg_ic_dev *ic_dev, int fv_mv)
{
	struct battery_chg_dev *bcdev;
	int prop_id = 0;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if (oplus_chg_get_voocphy_support(bcdev) == ADSP_VOOCPHY) {
		bool fastchg_ing = oplus_vooc_get_fastchg_ing(bcdev);
		int fast_chg_type = oplus_vooc_get_fast_chg_type(bcdev);
		if (fastchg_ing && (fast_chg_type == BCC_TYPE_IS_SVOOC)) {
			chg_info("fastchg started, do not set fv\n");
			return rc;
		}
	}

	if (!bcdev->voocphy_bidirect_cp_support)
		fv_mv *= bcdev->batt_num;
	prop_id = get_property_id(pst, POWER_SUPPLY_PROP_VOLTAGE_MAX);
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_VOLTAGE_MAX, fv_mv);
	else
		rc = write_property_id(bcdev, pst, prop_id, fv_mv);
	if (rc)
		chg_err("set fv to %d mV fail, rc=%d\n", fv_mv, rc);

	return rc;
}

static int oplus_chg_8350_set_iterm(struct oplus_chg_ic_dev *ic_dev, int iterm_ma)
{
#ifdef OPLUS_CHG_UNDEF /* TODO */
	int rc = 0;
	u8 val_raw = 0;
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (term_current < 0 || term_current > 750)
		term_current = 150;

	val_raw = term_current / 50;
	rc = smblib_masked_write(bcdev, TCCC_CHARGE_CURRENT_TERMINATION_CFG_REG,
				 TCCC_CHARGE_CURRENT_TERMINATION_SETTING_MASK, val_raw);
	if (rc < 0)
		chg_err("Couldn't write TCCC_CHARGE_CURRENT_TERMINATION_CFG_REG rc=%d\n", rc);
	return rc;
#endif

	return 0;
}

static int oplus_chg_8350_set_rechg_vol(struct oplus_chg_ic_dev *ic_dev, int vol_mv)
{
	return 0;
}

static int oplus_chg_8350_get_input_curr(struct oplus_chg_ic_dev *ic_dev, int *curr_ma)
{
	struct battery_chg_dev *bcdev;
	int prop_id = 0;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_USB];

	prop_id = get_property_id(pst, POWER_SUPPLY_PROP_CURRENT_NOW);
	rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0) {
		chg_err("read ibus fail, rc=%d\n", rc);
		return rc;
	}
	*curr_ma = DIV_ROUND_CLOSEST((int)pst->prop[prop_id], 1000);

	return rc;
}

static int oplus_chg_8350_get_input_vol(struct oplus_chg_ic_dev *ic_dev, int *vol_mv)
{
	struct battery_chg_dev *bcdev;
	int prop_id = 0;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_USB];

	prop_id = get_property_id(pst, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0) {
		chg_err("read usb vbus_volt fail, rc=%d\n", rc);
		return rc;
	}
	*vol_mv = pst->prop[prop_id] / 1000;

	return rc;
}

static int oplus_chg_8350_otg_boost_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (bcdev->otg_boost_src == OTG_BOOST_SOURCE_EXTERNAL) {
		if (en) {
			oplus_set_otg_ovp_en_val(bcdev, 1);
			oplus_set_otg_boost_en_val(bcdev, 1);
		} else {
			oplus_set_otg_boost_en_val(bcdev, 0);
			oplus_set_otg_ovp_en_val(bcdev, 0);
		}
		return rc;
	}

	pst = &bcdev->psy_list[PSY_TYPE_USB];
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_OTG_VBUS_REGULATOR_ENABLE, en ? 1 : 0);
	else
		rc = write_property_id(bcdev, pst, USB_OTG_VBUS_REGULATOR_ENABLE, en ? 1 : 0);
	if (rc) {
		chg_err("%s otg boost fail, rc=%d\n", en ? "enable" : "disable", rc);
		return rc;
	}
	schedule_delayed_work(&bcdev->otg_status_check_work, 0);

	return rc;
}

static int oplus_chg_8350_set_otg_boost_vol(struct oplus_chg_ic_dev *ic_dev, int vol_mv)
{
	return 0;
}

#define LOW_BATT_SOC 1
#define LOW_BATT_OTG_BOOST_CURR_MA 500
static int oplus_chg_8350_set_otg_boost_curr_limit(struct oplus_chg_ic_dev *ic_dev, int curr_ma)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	int rc = 0;
	char battery_type_str[OPLUS_BATTERY_TYPE_LEN] = { 0 };
	int batt_soc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	batt_soc = oplus_chg_get_battery_soc(bcdev);

	rc = oplus_gauge_get_battery_type_str(battery_type_str);
	if (rc)
		chg_err("get battery type failed, rc=%d\n", rc);

	if (!strncmp(battery_type_str, "silicon", strlen("silicon")) && batt_soc <= LOW_BATT_SOC)
		curr_ma = LOW_BATT_OTG_BOOST_CURR_MA;

	pst = &bcdev->psy_list[PSY_TYPE_USB];
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_OTG_BOOST_CURRENT, curr_ma);
	else
		rc = write_property_id(bcdev, pst, USB_OTG_BOOST_CURRENT, curr_ma);
	if (rc) {
		chg_err("set otg boost curr limit %d mA failed, rc=%d\n", curr_ma, rc);
		return rc;
	}
	chg_err("set otg boost curr limit %d mA batt_type=%s soc=%d\n", curr_ma, battery_type_str, batt_soc);

	return rc;
}

static int oplus_chg_8350_aicl_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	/* TODO */

	return rc;
}

static int oplus_chg_8350_aicl_rerun(struct oplus_chg_ic_dev *ic_dev)
{
	int rc = 0;
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_SET_RERUN_AICL, 0);
	else
		rc = write_property_id(bcdev, pst, USB_SET_RERUN_AICL, 0);
	if (rc)
		chg_err("rerun aicl fail, rc=%d\n", rc);

	return rc;
}

static int oplus_chg_8350_aicl_reset(struct oplus_chg_ic_dev *ic_dev)
{
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	/* TODO */

	return rc;
}

static int oplus_chg_8350_get_cc_orientation(struct oplus_chg_ic_dev *ic_dev, int *orientation)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_USB];
	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_TYPEC_CC_ORIENTATION);
	else
		rc = read_property_id(bcdev, pst, USB_TYPEC_CC_ORIENTATION);
	if (rc < 0) {
		chg_err("read typec_cc_orientation fail\n");
		return rc;
	}
	if (bcdev->soccp_support)
		*orientation = bcdev->oplus_psy.prop[OPLUS_USB_TYPEC_CC_ORIENTATION];
	else
		*orientation = pst->prop[USB_TYPEC_CC_ORIENTATION];


	return rc;
}

static int oplus_chg_8350_get_power_role(struct oplus_chg_ic_dev *ic_dev, int *power_role)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_USB];
	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_GET_POWER_ROLE);
	else
		rc = read_property_id(bcdev, pst, USB_GET_POWER_ROLE);

	if (rc < 0) {
		chg_err("read power_role fail, rc=%d\n", rc);
		return rc;
	}
	if (bcdev->soccp_support)
		*power_role = bcdev->oplus_psy.prop[OPLUS_USB_GET_POWER_ROLE];
	else
		*power_role = pst->prop[USB_GET_POWER_ROLE];
	bcdev->power_role = *power_role;
	if (bcdev->power_role == 1) {
		chg_info("begin reverse chg\n");
		bcdev->reverse_enable = bcdev->power_role;
		oplus_chg_ic_virq_trigger(bcdev->reverse_chg_ic_dev, OPLUS_IC_VIRQ_REVERSE_ENABLE);
	}
	return rc;
}

static int oplus_chg_8350_get_hw_detect(struct oplus_chg_ic_dev *ic_dev, int *detected, bool recheck)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	if(recheck)
		msleep(CID_STATUS_DELAY_MS);
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_USB];
	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_CID_STATUS);
	else
		rc = read_property_id(bcdev, pst, USB_CID_STATUS);
	if (rc < 0) {
		chg_err("read cid_status fail, rc=%d\n", rc);
		return rc;
	}
	if (bcdev->soccp_support)
		*detected = bcdev->oplus_psy.prop[OPLUS_USB_CID_STATUS];
	else
		*detected = pst->prop[USB_CID_STATUS];

	return 0;
}

static int oplus_chg_8350_get_charger_type(struct oplus_chg_ic_dev *ic_dev, int *type)
{
	struct battery_chg_dev *bcdev;
	int prop_id = 0;
	struct psy_state *pst = NULL;
	int rc = 0;
	int usb_type = 0;
	int usb_sub_type = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_USB];

	prop_id = get_property_id(pst, POWER_SUPPLY_PROP_USB_TYPE);
	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_TYPE);
	else
		rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0) {
		chg_err("read usb charger_type fail, rc=%d\n", rc);
		return rc;
	}
	if (bcdev->soccp_support)
		usb_type = bcdev->oplus_psy.prop[OPLUS_USB_TYPE];
	else
		usb_type = pst->prop[prop_id];
	switch (usb_type) {
	case POWER_SUPPLY_USB_TYPE_UNKNOWN:
		*type = OPLUS_CHG_USB_TYPE_UNKNOWN;
		break;
	case POWER_SUPPLY_USB_TYPE_SDP:
		*type = OPLUS_CHG_USB_TYPE_SDP;
		break;
	case POWER_SUPPLY_USB_TYPE_DCP:
		*type = OPLUS_CHG_USB_TYPE_DCP;
		break;
	case POWER_SUPPLY_USB_TYPE_CDP:
		*type = OPLUS_CHG_USB_TYPE_CDP;
		break;
	case POWER_SUPPLY_USB_TYPE_ACA:
		*type = OPLUS_CHG_USB_TYPE_ACA;
		break;
	case POWER_SUPPLY_USB_TYPE_C:
		*type = OPLUS_CHG_USB_TYPE_C;
		break;
	case POWER_SUPPLY_USB_TYPE_PD:
		*type = OPLUS_CHG_USB_TYPE_PD;
		break;
	case POWER_SUPPLY_USB_TYPE_PD_DRP:
		*type = OPLUS_CHG_USB_TYPE_PD_DRP;
		break;
	case POWER_SUPPLY_USB_TYPE_PD_PPS:
		*type = OPLUS_CHG_USB_TYPE_PD_PPS;
		break;
	case POWER_SUPPLY_USB_TYPE_PD_SDP:
		*type = OPLUS_CHG_USB_TYPE_PD_SDP;
		break;
	case POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID:
		*type = OPLUS_CHG_USB_TYPE_DCP;
		break;
	}

	if (*type != POWER_SUPPLY_USB_TYPE_PD &&
	    *type != POWER_SUPPLY_USB_TYPE_PD_DRP &&
	    *type != POWER_SUPPLY_USB_TYPE_PD_PPS) {
		if (bcdev->soccp_support)
			rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_SUBTYPE);
		else
			rc = read_property_id(bcdev, pst, USB_ADAP_SUBTYPE);
		if (rc < 0) {
			chg_err("read charger subtype fail, rc=%d\n", rc);
			rc = 0;
		}
		if (bcdev->soccp_support)
			usb_sub_type = bcdev->oplus_psy.prop[OPLUS_USB_SUBTYPE];
		else
			usb_sub_type = pst->prop[USB_ADAP_SUBTYPE];
		switch (usb_sub_type) {
		case CHARGER_SUBTYPE_FASTCHG_VOOC:
			*type = OPLUS_CHG_USB_TYPE_VOOC;
			break;
		case CHARGER_SUBTYPE_FASTCHG_SVOOC:
			*type = OPLUS_CHG_USB_TYPE_SVOOC;
			break;
		case CHARGER_SUBTYPE_QC:
			*type = OPLUS_CHG_USB_TYPE_QC2;
			break;
		default:
			break;
		}
	}

	bcdev->charger_type = *type;

	return 0;
}

static int oplus_chg_8350_rerun_bc12(struct oplus_chg_ic_dev *ic_dev)
{
	return 0;
}

static int oplus_chg_8350_qc_detect_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if (bcdev->hvdcp_disable == true) {
		chg_err("hvdcp_disable!\n");
		return -EINVAL;
	}

	if (en) {
		if (bcdev->soccp_support)
			rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_SET_QC, 0);
		else
			rc = write_property_id(bcdev, pst, BATT_SET_QC, 0);
		bcdev->hvdcp_detect_time = cpu_clock(smp_processor_id()) / CPU_CLOCK_TIME_MS;
	}

	return rc;
}

#define PWM_COUNT	5
static int oplus_chg_8350_shipmode_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	smbchg_enter_shipmode_pmic(bcdev);

	return 0;
}

#define VBUS_QC9V_THR	8000
#define VBUS_QC12V_THR	11000
static int oplus_chg_8350_set_qc_config(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_qc_version version, int vol_mv)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	int vbus_vol = 0;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if (bcdev->adsp_crash) {
		return -EAGAIN;
	}

	switch (version) {
	case OPLUS_CHG_QC_2_0:
		if (vol_mv != 5000 && vol_mv != 9000 && vol_mv != 12000) {
			chg_err("Unsupported qc voltage(=%d)\n", vol_mv);
			return -EINVAL;
		}
		if (bcdev->soccp_support)
			rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_SET_QC, vol_mv);
		else
			rc = write_property_id(bcdev, pst, BATT_SET_QC, vol_mv);
		if (rc)
			chg_err("set QC to %d mV fail, rc=%d\n", vol_mv, rc);
		else
			chg_err("set QC to %d mV, rc=%d\n", vol_mv, rc);
		msleep(350);
		vbus_vol = qpnp_get_prop_charger_voltage_now(bcdev);
		if (vol_mv < 12000 && vbus_vol < VBUS_QC9V_THR) {
			chg_err("Non-standard QC-liked adapter detected,unabled to request 9V,falls back to 5V");
			if (bcdev->soccp_support)
				rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_SET_QC, 5000);
			else
				rc = write_property_id(bcdev, pst, BATT_SET_QC, 5000);
			if (rc)
				chg_err("Fall back to QC 5V fail, rc=%d\n", rc);
			else
				chg_err("Fall back to QC 5V OK\n");
		}
		if (vol_mv == 12000 && vbus_vol < VBUS_QC12V_THR) {
			if (vbus_vol >= VBUS_QC9V_THR) {
				chg_info("Non-standard QC-liked adapter detected,unabled to request 12V,falls back to 9V");
				if (bcdev->soccp_support)
					rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_SET_QC, 9000);
				else
					rc = write_property_id(bcdev, pst, BATT_SET_QC, 9000);
				if (rc)
					chg_err("Fall back to QC 9V fail, rc=%d\n", rc);
			} else {
				chg_info("req qc12V fail, vbus=%d mV\n", vbus_vol);
				return -EAGAIN;
			}
		}
		break;
	case OPLUS_CHG_QC_3_0:
	default:
		chg_err("Unsupported qc version(=%d)\n", version);
		return -EINVAL;
	}

	return rc;
}

#define WAIT_SOURCECPA_DOWN 1000

static void oplus_source_pdo_check_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, source_pdo_check_work.work);
	int max_pdo_current = 0;

	if (bcdev->pd_chg_volt == OPLUS_PD_9V)
		max_pdo_current = oplus_get_max_current_from_fixed_pdo(bcdev, OPLUS_PD_9V);
	else if (bcdev->pd_chg_volt == OPLUS_PD_12V)
		max_pdo_current = oplus_get_max_current_from_fixed_pdo(bcdev, OPLUS_PD_12V);
	else
		max_pdo_current = oplus_get_max_current_from_fixed_pdo(bcdev, OPLUS_PD_5V);

	if (max_pdo_current >= 0)
		oplus_chg_set_icl_by_vote(max_pdo_current, PD_PDO_ICL_VOTER);
}

static int oplus_chg_8350_set_pd_config(struct oplus_chg_ic_dev *ic_dev, u32 pdo)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	int vol_mv;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	switch (PD_SRC_PDO_TYPE(pdo)) {
	case PD_SRC_PDO_TYPE_FIXED:
		vol_mv = PD_SRC_PDO_FIXED_VOLTAGE(pdo) * 50;
		if (vol_mv != OPLUS_PD_5V && vol_mv != OPLUS_PD_9V && vol_mv != OPLUS_PD_12V) {
			chg_err("Unsupported pd voltage(=%d)\n", vol_mv);
			return -EINVAL;
		}
		if (bcdev->soccp_support)
			rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_SET_PDO, vol_mv);
		else
			rc = write_property_id(bcdev, pst, BATT_SET_PDO, vol_mv);
		if (rc) {
			chg_err("set PD to %d mV fail, rc=%d\n", vol_mv, rc);
		} else {
			chg_err("set PD to %d mV, rc=%d\n", vol_mv, rc);
			bcdev->pd_chg_volt = vol_mv;
		}
		break;
	case PD_SRC_PDO_TYPE_BATTERY:
	case PD_SRC_PDO_TYPE_VARIABLE:
	case PD_SRC_PDO_TYPE_AUGMENTED:
	default:
		chg_err("Unsupported pdo type(=%d)\n", PD_SRC_PDO_TYPE(pdo));
		return -EINVAL;
	}

	schedule_delayed_work(&bcdev->source_pdo_check_work,
				msecs_to_jiffies(WAIT_SOURCECPA_DOWN));

	return rc;
}

static int oplus_chg_8350_get_props_from_adsp_by_buffer(struct oplus_chg_ic_dev *ic_dev)
{
	oplus_get_props_from_adsp_by_buffer();
	return 0;
}

static int oplus_chg_8350_gauge_is_suspend(struct oplus_chg_ic_dev *ic_dev, bool *suspend)
{
	struct battery_chg_dev *bcdev;

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	*suspend = atomic_read(&bcdev->suspended);

	return 0;
}

static int oplus_chg_8350_voocphy_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	/* return oplus_voocphy_enable(en); */
	return 0;
}

static int oplus_chg_8350_voocphy_reset_again(struct oplus_chg_ic_dev *ic_dev)
{
	/* return oplus_adsp_voocphy_reset_again; */
	return 0;
}

static int oplus_chg_8350_get_charger_cycle(struct oplus_chg_ic_dev *ic_dev, int *cycle)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if (oplus_chg_get_voocphy_support(bcdev) == ADSP_VOOCPHY) {
		*cycle = bcdev->read_buffer_dump.data_buffer[5];
		return 0;
	}

	rc = read_property_id(bcdev, pst, BATT_CYCLE_COUNT);
	if (rc) {
		chg_err("get charger_cycle fail, rc=%d\n", rc);
		return rc;
	}

	*cycle = pst->prop[BATT_CYCLE_COUNT];

	return rc;
}

static int oplus_chg_8350_get_shutdown_soc(struct oplus_chg_ic_dev *ic_dev, int *soc)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_RTC_SOC);
	else
		rc = read_property_id(bcdev, pst, BATT_RTC_SOC);
	if (rc < 0) {
		chg_err("read battery rtc soc fail, rc=%d\n", rc);
		return rc;
	}
	if (bcdev->soccp_support)
		*soc = bcdev->oplus_psy.prop[OPLUS_RTC_SOC];
	else
		*soc = pst->prop[BATT_RTC_SOC];
	chg_info("read battery rtc soc success, rtc_soc=%d\n", *soc);

	return rc;
}

static int oplus_chg_8350_backup_soc(struct oplus_chg_ic_dev *ic_dev, int soc)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_RTC_SOC, soc);
	else
		rc = write_property_id(bcdev, pst, BATT_RTC_SOC, soc);
	if (rc) {
		chg_err("set battery rtc soc fail, rc=%d\n", rc);
		return 0;
	}
	chg_info("write battery rtc soc success, rtc_soc=%d\n", soc);

	return rc;
}

static int oplus_chg_8350_get_vbus_collapse_status(struct oplus_chg_ic_dev *ic_dev, bool *collapse)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_VBUS_COLLAPSE_STATUS);
	else
		rc = read_property_id(bcdev, pst, USB_VBUS_COLLAPSE_STATUS);
	if (rc < 0) {
		chg_err("read usb vbus_collapse_status fail, rc=%d\n", rc);
		return rc;
	}
	if (bcdev->soccp_support)
		*collapse = bcdev->oplus_psy.prop[OPLUS_USB_VBUS_COLLAPSE_STATUS];
	else
		*collapse = pst->prop[USB_VBUS_COLLAPSE_STATUS];
	chg_info("read usb vbus_collapse_status[%d]\n", *collapse);
	return rc;
}

static int oplus_chg_8350_get_typec_mode(struct oplus_chg_ic_dev *ic_dev,
					enum oplus_chg_typec_port_role_type *mode)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	int rc = 0;
	int typec_mode = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_TYPEC_MODE);
	else
		rc = read_property_id(bcdev, pst, USB_TYPEC_MODE);
	if (rc < 0) {
		chg_err("read typec mode fail, rc=%d\n", rc);
		return rc;
	}
	if (bcdev->soccp_support)
		typec_mode = bcdev->oplus_psy.prop[OPLUS_USB_TYPEC_MODE];
	else
		typec_mode = pst->prop[USB_TYPEC_MODE];
	if (typec_mode == 0)
		*mode = TYPEC_PORT_ROLE_SNK;
	else
		*mode = TYPEC_PORT_ROLE_SRC;

	return rc;
}

static int oplus_chg_8350_set_typec_mode(struct oplus_chg_ic_dev *ic_dev,
					enum oplus_chg_typec_port_role_type mode)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (mode >= ARRAY_SIZE(qcom_typec_port_role)) {
		chg_err("typec mode(=%d) error\n", mode);
		return -EINVAL;
	}
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_TYPEC_MODE, qcom_typec_port_role[mode]);
	else
		rc = write_property_id(bcdev, pst, USB_TYPEC_MODE, qcom_typec_port_role[mode]);
	if (rc < 0)
		chg_err("set typec mode(=%d) error\n", mode);

	return rc;
}

static int oplus_chg_8350_set_otg_switch_status(struct oplus_chg_ic_dev *ic_dev,
						bool en)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_OTG_SWITCH, en);
	else
		rc = write_property_id(bcdev, pst, USB_OTG_SWITCH, en);
	if (rc < 0)
		chg_err("%s otg switch error, rc=%d\n",
			en ? "enable" : "disable", rc);
	return rc;
}

static int oplus_chg_8350_get_otg_switch_status(struct oplus_chg_ic_dev *ic_dev,
						bool *en)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_OTG_SWITCH);
	else
		rc = read_property_id(bcdev, pst, USB_OTG_SWITCH);
	if (rc < 0) {
		chg_err("get otg switch status error, rc=%d\n", rc);
		return rc;
	}
	if (bcdev->soccp_support)
		*en = !!bcdev->oplus_psy.prop[OPLUS_USB_OTG_SWITCH];
	else
		*en = !!pst->prop[USB_OTG_SWITCH];

	return rc;
}

static int oplus_chg_8350_cc_detect_happened(struct oplus_chg_ic_dev *ic_dev)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_CCDETECT_HAPPENED, 1);
	else
		rc = write_property_id(bcdev, pst, USB_CCDETECT_HAPPENED, 1);
	if (rc < 0)
		chg_err("write ccdetect plugout fail, rc=%d\n", rc);
	else
		chg_info("cc detect plugout, rc=%d\n", rc);

	return rc;
}

static int oplus_chg_8350_set_curr_level(struct oplus_chg_ic_dev *ic_dev, int cool_down)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_SET_COOL_DOWN, cool_down);
	else
		rc = write_property_id(bcdev, pst, BATT_SET_COOL_DOWN, cool_down);
	if (rc < 0)
		chg_err("write cool down fail, rc=%d\n", rc);
	else
		chg_info("set cool down to %d, rc=%d\n", cool_down, rc);

	return rc;
}

int oplus_adsp_voocphy_set_cool_down(int cool_down, int curr_ma)
{
	int rc = 0;
	struct battery_chg_dev *bcdev = g_bcdev;
	struct psy_state *pst = NULL;

	if (!bcdev) {
		chg_err("bcdev is NULL!\n");
		return -1;
	}
	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_SET_COOL_DOWN,
			(cool_down & 0xff) | ((curr_ma & 0xffff) << 16));
	else
		rc = write_property_id(bcdev, pst, BATT_SET_COOL_DOWN,
			(cool_down & 0xff) | ((curr_ma & 0xffff) << 16));
	if (rc < 0)
		chg_err("write cool down fail, rc=%d\n", rc);
	else
		chg_info("set cool down to %d, curr to %dma, rc=%d\n", cool_down, curr_ma, rc);

	return rc;
}

int oplus_adsp_voocphy_set_curve_num(int number)
{
	int rc = 0;
	struct battery_chg_dev *bcdev = g_bcdev;
	struct psy_state *pst = NULL;

	if (!bcdev) {
		chg_err("bcdev is NULL!\n");
		return -1;
	}
	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_SET_VOOC_CURVE_NUM, number);
	else
		rc = write_property_id(bcdev, pst, BATT_SET_VOOC_CURVE_NUM, number);
	if (rc < 0)
		chg_err("write curve num fail, rc=%d\n", rc);
	else
		chg_info("set curve num to %d, rc=%d\n", number, rc);

	return rc;
}

static int oplus_chg_set_ovp_forced(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	int rc = 0;
	struct psy_state *pst = NULL;
	struct battery_chg_dev *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}
	pst = &chip->psy_list[PSY_TYPE_USB];

	if (chip->soccp_support) {
		rc = write_property_id(chip, &chip->oplus_psy, OPLUS_USB_SET_OVP_FORCED, en);
		if (rc < 0)
			chg_err("set ovp fail, rc=%d\n", rc);
		else
			chg_info("set ovp to %d, rc=%d\n", en, rc);
	} else {
		rc = -EOPNOTSUPP;
		chg_err("oplus_chg_set_ovp_forced not support adsp, rc=%d\n", rc);
	}
	return rc;
}

static int oplus_chg_8350_set_match_temp(struct oplus_chg_ic_dev *ic_dev, int match_temp)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_SET_MATCH_TEMP, match_temp);
	else
		rc = write_property_id(bcdev, pst, BATT_SET_MATCH_TEMP, match_temp);
	if (rc < 0)
		chg_err("write match temp fail, rc=%d\n", rc);

	return rc;
}

static int oplus_chg_8350_get_otg_enbale(struct oplus_chg_ic_dev *ic_dev, bool *enable)
{
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	*enable = bcdev->otg_online;

	return 0;
}

static int oplus_chg_set_input_current_with_no_aicl(struct battery_chg_dev *bcdev, int current_ma)
{
	int rc = 0;
	int prop_id = 0;
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (current_ma == 0)
		current_ma = usb_icl[0];

	prop_id = get_property_id(pst, POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT);
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_INPUT_CURRENT_LIMIT, current_ma * 1000);
	else
		rc = write_property_id(bcdev, pst, prop_id, current_ma * 1000);
	if (rc)
		chg_err("set icl to %d mA fail, rc=%d\n", current_ma, rc);
	else
		chg_info("set icl to %d mA\n", current_ma);

	return rc;
}

static int oplus_chg_8350_set_usb_drv(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	int rc = 0;
	struct psy_state *pst = NULL;
	struct battery_chg_dev *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &chip->psy_list[PSY_TYPE_USB];

	if (chip->soccp_support)
		rc = write_property_id(chip, &chip->oplus_psy, OPLUS_USB_SET_OVP_CFG, en);
	else
		rc = write_property_id(chip, pst, USB_SET_OVP_CFG, en);
	if (rc < 0)
		chg_err("can't %s usb drv cfg, rc=%d\n", en ? "enable" : "disable", rc);
	else
		chg_info("usb drv cfg write %d succees!\n", en);

	return rc;
}

static int oplus_chg_8350_is_oplus_svid(struct oplus_chg_ic_dev *ic_dev, bool *oplus_svid)
{
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	*oplus_svid = bcdev->pd_svooc;

	return 0;
}

static int oplus_chg_8350_hardware_init(struct oplus_chg_ic_dev *ic_dev)
{
	struct battery_chg_dev *bcdev;
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	int boot_mode = get_boot_mode();
#endif

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if (boot_mode != MSM_BOOT_MODE__RF && boot_mode != MSM_BOOT_MODE__WLAN) {
		oplus_chg_8350_input_suspend(ic_dev, false);
	} else {
		oplus_chg_8350_input_suspend(ic_dev, true);
	}
	chg_info("boot_mode:%d\n", boot_mode);
#else
	oplus_chg_8350_input_suspend(ic_dev, false);
#endif
	oplus_chg_set_input_current_with_no_aicl(bcdev, 500);
	oplus_chg_8350_output_suspend(ic_dev, false);

	return 0;
}

#define BTB_TEMP_DEFAULT	25
static int oplus_chg_get_battery_btb_temp_cal(struct battery_chg_dev *bcdev)
{
	int rc;
	int temp = BTB_TEMP_DEFAULT;
	int batt_con_btb_temp = BTB_TEMP_DEFAULT;

	if (!bcdev) {
		chg_err("bcdev not ready\n");
		return temp;
	}

	if (!IS_ERR_OR_NULL(bcdev->iio.batt_con_btb_chan)) {
		rc = iio_read_channel_processed(bcdev->iio.batt_con_btb_chan, &temp);
		if (rc < 0) {
			chg_err("iio_read_channel_processed get error\n");
		} else {
			batt_con_btb_temp = temp / 1000;
		}
	} else {
		chg_err("batt_con_btb_chan is NULL !\n");
	}

	if (!IS_ERR_OR_NULL(bcdev->iio.sub_batt_con_btb_chan)) {
		rc = iio_read_channel_processed(bcdev->iio.sub_batt_con_btb_chan, &temp);
		if (rc < 0) {
			chg_err("sub_batt_con_btb_chan get error\n");
		} else {
			temp = temp / 1000;
			if (temp > batt_con_btb_temp)
				batt_con_btb_temp = temp;
		}
	}

	chg_debug("batt_con_btb_temp %d\n",  batt_con_btb_temp);

	return batt_con_btb_temp;
}

static int oplus_chg_get_usb_btb_temp_cal(struct battery_chg_dev *bcdev)
{
	int rc;
	int temp = 25;

	if (!bcdev) {
		chg_err("bcdev not ready\n");
		return temp;
	}

	if (IS_ERR_OR_NULL(bcdev->iio.usb_con_btb_chan)) {
		chg_err("bcdev->iio.usb_con_btb_chan is NULL\n");
		return temp;
	}

	rc = iio_read_channel_processed(bcdev->iio.usb_con_btb_chan, &temp);
	if (rc < 0) {
		chg_err("iio_read_channel_processed get error\n");
		return temp;
	}

	return temp / 1000;
}

static int oplus_chg_get_sub_detect_adc_temp_cal(struct battery_chg_dev *bcdev, int *temp)
{
	int rc = 0;

	if (!bcdev) {
		chg_err("bcdev not ready\n");
		return -ENODEV;
	}

	if (!IS_ERR_OR_NULL(bcdev->iio.sub_con_btb_detect_adc)) {
		rc = iio_read_channel_processed(bcdev->iio.sub_con_btb_detect_adc, temp);
		if (rc < 0) {
			chg_err("iio_read_channel_processed get  sub_con_btb_detect_adc error\n");
		} else {
			*temp = *temp / 1000;
		}
	} else {
		rc = -EOPNOTSUPP;
		chg_err("sub_con_btb_detect_adc is NULL !\n");
	}

	chg_debug("sub_con_btb_detect_adc %d\n", *temp);

	return rc;
}

static int oplus_get_sub_btb_state(struct battery_chg_dev *bcdev)
{
	int temp = 0;
	int state = BATT_BTB_STATE_NOT_SUPPORT;
	int rc = 0;

	if (NULL == bcdev) {
		chg_err("bcdev is NULL");
		return -ENODEV;
	}

	/* check if the btb_state can be get by battery temp adc which is by ADC channel. */
	if (bcdev->sub_btb_valid_temp[0] || bcdev->sub_btb_valid_temp[1]) {
		/* get the real batt temp */
		rc = oplus_chg_get_sub_detect_adc_temp_cal(bcdev, &temp);
		if (rc < 0) {
			chg_err("get the battery temp adc failed, rc = %d", rc);
			return rc;
		}

		/* check the silicon sub_btb state */
		if (bcdev->sub_btb_valid_temp[0] && bcdev->sub_btb_valid_temp[1]) {
			/* valid temp range: min < ~ < max graphite  battery of sub_btb is online. */
			if (temp > bcdev->sub_btb_valid_temp[1] || temp < bcdev->sub_btb_valid_temp[0])
				state = BATT_BTB_STATE_NOT_CONNECT;
			else
				state = BATT_BTB_STATE_CONNECT;
		}  else if (bcdev->sub_btb_valid_temp[0] && !bcdev->sub_btb_valid_temp[1]) {
				/*
				* valid temp range: < max, for [-350, 0], more than 1000,
				* battery of sub_btb is Onlne.
				*/
				if (temp < bcdev->sub_btb_valid_temp[0])
					state = BATT_BTB_STATE_NOT_CONNECT;
				else
					state = BATT_BTB_STATE_CONNECT;
		}
	}
	chg_debug("temp = %d, adc_valid[%d - %d], state = %d", temp, bcdev->sub_btb_valid_temp[0],
		  bcdev->sub_btb_valid_temp[1], state);

	return state;
}

static int oplus_gauge_get_sub_btb_state(struct oplus_chg_ic_dev *ic_dev,
					 enum oplus_sub_btb_state *state)
{
	struct battery_chg_dev *bcdev;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	rc = oplus_get_sub_btb_state(bcdev);

	if (rc < 0) {
		chg_err("get sub_btb_state fail, rc = %d", rc);
		return rc;
	} else {
		*state = rc;
	}

	return 0;
}

static int oplus_chg_get_pmic_vbat_vol(struct battery_chg_dev *bcdev, int *vol_mv)
{
	int rc = 0;

	if (!bcdev) {
		chg_err("bcdev not ready\n");
		return -ENODEV;
	}

	if (!IS_ERR_OR_NULL(bcdev->iio.pmic_vbat_adc)) {
		rc = iio_read_channel_processed(bcdev->iio.pmic_vbat_adc, vol_mv);
		if (rc < 0) {
			chg_err("iio_read_channel_processed get  pmic_vbat_adc error\n");
		} else {
			*vol_mv = *vol_mv / 1000;
		}
	} else {
		rc = -EOPNOTSUPP;
		chg_err("pmic_vbat_adc is NULL !\n");
	}

	return rc;
}

static int oplus_chg_get_chg_mos_temp_cal(struct battery_chg_dev *bcdev)
{
	int rc;
	int temp = 25;

	if (!bcdev) {
		chg_err("bcdev not ready\n");
		return temp;
	}

	if (IS_ERR_OR_NULL(bcdev->iio.chg_mos_temp_chan)) {
		chg_err("bcdev->iio.chg_mos_temp_chan is NULL\n");
		return temp;
	}

	rc = iio_read_channel_processed(bcdev->iio.chg_mos_temp_chan, &temp);
	if (rc < 0) {
		chg_err("iio_read_channel_processed get chg_mos_temp_chan error\n");
		return temp;
	}

	return temp / 1000;
}

static int oplus_chg_adsp_get_usb_btb_temp(struct oplus_chg_ic_dev *ic_dev,
					   int *usb_btb_temp)
{
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	*usb_btb_temp = oplus_chg_get_usb_btb_temp_cal(bcdev);

	return 0;
}

static int oplus_chg_adsp_get_batt_btb_temp(struct oplus_chg_ic_dev *ic_dev,
					    int *batt_btb_temp)
{
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	*batt_btb_temp = oplus_chg_get_battery_btb_temp_cal(bcdev);

	return 0;
}

#define SHAFT_BTB_TEMP_DEFAULT 25
static int oplus_chg_get_shaft_btb_temp_cal(struct battery_chg_dev *bcdev)
{
	int rc;
	int temp = SHAFT_BTB_TEMP_DEFAULT;
	int shaft_con_btb_temp = SHAFT_BTB_TEMP_DEFAULT;

	if (!bcdev) {
		chg_err("bcdev not ready\n");
		return SHAFT_BTB_TEMP_DEFAULT;
	}

	if (!IS_ERR_OR_NULL(bcdev->iio.shaft_btb_temp_chan)) {
		rc = iio_read_channel_processed(bcdev->iio.shaft_btb_temp_chan, &temp);
		if (rc < 0) {
			chg_err("iio_read_channel_processed get error\n");
		} else {
			shaft_con_btb_temp = temp / 1000;
		}
	} else {
		chg_err("batt_con_btb_chan is NULL !\n");
		return SHAFT_BTB_TEMP_DEFAULT;
	}

	chg_debug("shaft_con_btb_temp %d \n", shaft_con_btb_temp);
	return shaft_con_btb_temp;
}

static int oplus_chg_adsp_get_shaft_btb_temp(struct oplus_chg_ic_dev *ic_dev,
					    int *shaft_btb_temp)
{
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	*shaft_btb_temp = oplus_chg_get_shaft_btb_temp_cal(bcdev);

	return 0;
}

static bool oplus_chg_shaft_track_limit(void)
{
	static int shaft_err_upload_count = 0;
	static int pre_upload_time = 0;
	int curr_time;

	curr_time = local_clock() / TRACK_LOCAL_T_NS_TO_S_THD;
	if (curr_time - pre_upload_time > TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD)
		shaft_err_upload_count = 0;

	chg_info("shaft_err_upload_count %d max cnt %d",
		shaft_err_upload_count, TRACK_UPLOAD_COUNT_MAX);

	if (shaft_err_upload_count >= TRACK_UPLOAD_COUNT_MAX)
		return true;

	pre_upload_time = local_clock() / TRACK_LOCAL_T_NS_TO_S_THD;
	shaft_err_upload_count++;

	return false;
}

static int oplus_chg_adsp_set_shaft_btb_over(struct oplus_chg_ic_dev *ic_dev,
					bool is_shaft_btb_over)
{
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (bcdev == NULL) {
		chg_err("bcdev is NULL");
		return -ENODEV;
	}

	if (atomic_read(&bcdev->is_shaft_btb_over) == 0 && is_shaft_btb_over && !oplus_chg_shaft_track_limit()) {
		oplus_chg_ic_creat_err_msg(bcdev->buck_ic,
			OPLUS_IC_ERR_NTC, 0, "$$err_reason@@shaf_btb_over$$shaft_btb_temp@@%d",
			oplus_chg_get_shaft_btb_temp_cal(bcdev));
		oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_ERR);
	}
	if (is_shaft_btb_over)
		atomic_set(&bcdev->is_shaft_btb_over, 1);
	else
		atomic_set(&bcdev->is_shaft_btb_over, 0);
	return 0;
}

static int oplus_chg_set_chg_path(struct oplus_chg_ic_dev *ic_dev, int path)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_SET_CHG_PATH, path);
	else
		rc = write_property_id(bcdev, pst, BATT_SET_CHG_PATH, path);
	if (rc < 0)
		chg_err("set chg path to %d fail\n", path);
	else
		chg_info("set chg path to %d succees!\n", path);

	return rc;
}

static int oplus_chg_get_chg_path_status(struct oplus_chg_ic_dev *ic_dev, int *status)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_GET_CHG_PATH_STATUS);
	else
		rc = read_property_id(bcdev, pst, BATT_GET_CHG_PATH_STATUS);
	if (rc < 0) {
		chg_err("get chg path status fail\n");
		return rc;
	}
	if (bcdev->soccp_support)
		*status = bcdev->oplus_psy.prop[OPLUS_GET_CHG_PATH_STATUS];
	else
		*status = pst->prop[BATT_GET_CHG_PATH_STATUS];

	return rc;
}

static int oplus_chg_adsp_get_chg_mos_temp(struct oplus_chg_ic_dev *ic_dev,
					    int *chg_mos_temp)
{
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	*chg_mos_temp = oplus_chg_get_chg_mos_temp_cal(bcdev);

	return 0;
}

static bool is_ext_boost(struct battery_chg_dev *bcdev)
{
	if (!bcdev) {
		chg_err("bcdev not ready!\n");
		return false;
	}
	return oplus_get_wired_chg_present();
}

static int oplus_chg_wls_boost_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	static bool pre_wired_chg_present = false;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_WLS];

	if (bcdev->wls_boost_src == WLS_BOOST_SOURCE_PMIC_OTG) {
		if (bcdev->soccp_support)
			rc = write_property_id(bcdev, &bcdev->oplus_psy,
				OPLUS_USB_OTG_VBUS_REGULATOR_ENABLE, en ? 1 : 0);
		else
			rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_USB],
				USB_OTG_VBUS_REGULATOR_ENABLE, en ? 1 : 0);
		if (rc)
			chg_err("%s otg boost fail, rc=%d\n", en ? "enable" : "disable", rc);
		else
			chg_info("%s otg boost ok, rc=%d\n", en ? "enable" : "disable", rc);
		return rc;
	} else if (bcdev->wls_boost_src == WLS_BOOST_SOURCE_PMIC_WLS) {
		if (bcdev->soccp_support)
			rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_DC_BOOST_EN, en ? 1 : 0);
		else
			rc = write_property_id(bcdev, pst, WLS_BOOST_EN, en ? 1 : 0);
		if (rc)
			chg_err("%s wls boost fail, rc=%d\n", en ? "enable" : "disable", rc);
		else
			chg_info("%s wls boost ok, rc=%d\n", en ? "enable" : "disable", rc);
		return rc;
	}

	if ((is_ext_boost(bcdev) && en) || pre_wired_chg_present) {
		if (en) {
			oplus_set_tx_ovp_en_val(bcdev, 1);
			/*for HW spec,need 100ms delay*/
			msleep(100);
			oplus_set_otg_boost_en_val(bcdev, 1);
			pre_wired_chg_present = true;
		} else {
			pre_wired_chg_present = false;
			oplus_set_otg_boost_en_val(bcdev, 0);
			/*for HW spec,need 100ms delay*/
			msleep(100);
			oplus_set_tx_ovp_en_val(bcdev, 0);
		}
		return 0;
	}
	if (en) {
		oplus_set_wrx_otg_en_val(bcdev, 1);
		/*for HW spec,need 100ms delay*/
		msleep(100);
	}

	if (en && bcdev->wls_boost_soft_start && bcdev->wls_set_boost_vol != WLS_BOOST_VOL_MIN_MV) {
		if (bcdev->soccp_support)
			rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_DC_BOOST_VOLTAGE, WLS_BOOST_VOL_MIN_MV);
		else
			rc = write_property_id(bcdev, pst, WLS_BOOST_VOLT, WLS_BOOST_VOL_MIN_MV);
		if (rc < 0) {
			chg_err("set boost vol to WLS_BOOST_VOL_MIN_MV error, rc=%d\n", rc);
			return rc;
		}
	}

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_DC_BOOST_EN, en ? 1 : 0);
	else
		rc = write_property_id(bcdev, pst, WLS_BOOST_EN, en ? 1 : 0);
	if (rc)
		chg_err("%s wls boost fail, rc=%d\n", en ? "enable" : "disable", rc);

	if (en && bcdev->wls_boost_soft_start && bcdev->wls_set_boost_vol != WLS_BOOST_VOL_MIN_MV) {
		msleep(2);
		if (bcdev->soccp_support)
			rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_DC_BOOST_VOLTAGE, bcdev->wls_set_boost_vol);
		else
			rc = write_property_id(bcdev, pst, WLS_BOOST_VOLT, bcdev->wls_set_boost_vol);
		if (rc < 0)
			chg_err("set boost vol to %d mV error, rc=%d\n", bcdev->wls_set_boost_vol, rc);
	}

	if (!en) {
		/*for HW spec,need 100ms delay*/
		msleep(100);
		oplus_set_wrx_otg_en_val(bcdev, 0);
	}

	return rc;
}

static int oplus_chg_set_wls_boost_vol(struct oplus_chg_ic_dev *ic_dev, int vol_mv)
{
	int rc = 0;
	struct psy_state *pst = NULL;
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_WLS];

	if (is_support_tx_boost(bcdev) || is_ext_boost(bcdev))
		return 0;

	if (vol_mv == WLS_TRX_MODE_VOL_MV && bcdev->wls_boost_vol_start_mv > 0)
		vol_mv = bcdev->wls_boost_vol_start_mv;
	if (vol_mv > bcdev->wls_boost_vol_max_mv)
		vol_mv = bcdev->wls_boost_vol_max_mv;

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_DC_BOOST_VOLTAGE, vol_mv);
	else
		rc = write_property_id(bcdev, pst, WLS_BOOST_VOLT, vol_mv);
	if (rc < 0)
		chg_err("set boost vol to %d mV error, rc=%d\n", vol_mv, rc);
	else
		bcdev->wls_set_boost_vol = vol_mv;

	return rc;
}

static int oplus_chg_set_wls_boost_curr_limit(struct oplus_chg_ic_dev *ic_dev, int curr_ma)
{
	int rc = 0;
	struct psy_state *pst = NULL;
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_WLS];

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_DC_BOOST_CURRENT, curr_ma);
	else
		rc = write_property_id(bcdev, pst, WLS_BOOST_CURRENT, curr_ma);
	if (rc < 0)
		chg_err("set boost current to %d mA error, rc=%d\n", curr_ma, rc);

	return rc;
}

static int oplus_chg_wls_input_suspend(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	return 0;
}

static int oplus_chg_set_wls_icl(struct oplus_chg_ic_dev *ic_dev, int icl_ma)
{
	int rc = 0;
	struct psy_state *pst = NULL;
	struct battery_chg_dev *chip;

#ifdef WLS_QI_DEBUG
	if (wls_dbg_icl_ma != 0)
		icl_ma = wls_dbg_icl_ma;
#endif

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &chip->psy_list[PSY_TYPE_WLS];

	if (chip->soccp_support)
		rc = write_property_id(chip, &chip->oplus_psy, OPLUS_DC_INPUT_CURRENT_LIMIT, icl_ma * 1000);
	else
		rc = write_property_id(chip, pst, WLS_INPUT_CURR_LIMIT, icl_ma * 1000);
	if (rc < 0)
		chg_err("set wls icl to %u error, rc=%d\n", icl_ma, rc);
	else
		chg_info("set icl to %d mA\n", icl_ma);

	return rc;
}

static int oplus_chg_get_wls_icl(struct oplus_chg_ic_dev *ic_dev, int *icl_ma)
{
	return 0;
}

static int oplus_chg_get_wls_input_curr(struct oplus_chg_ic_dev *ic_dev, int *curr_ma)
{
	int rc = 0;
	struct psy_state *pst = NULL;
	struct battery_chg_dev *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &chip->psy_list[PSY_TYPE_WLS];

	rc = read_property_id(chip, pst, WLS_CURR_NOW);
	if (rc < 0) {
		chg_err("get wls input curr error, rc=%d\n", rc);
		return rc;
	}
	*curr_ma = pst->prop[WLS_CURR_NOW] / 1000;

	return rc;
}

static int oplus_chg_get_wls_input_vol(struct oplus_chg_ic_dev *ic_dev, int *vol_mv)
{
	int rc = 0;
	struct psy_state *pst = NULL;
	struct battery_chg_dev *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &chip->psy_list[PSY_TYPE_WLS];

	rc = read_property_id(chip, pst, WLS_VOLT_NOW);
	if (rc < 0) {
		chg_err("set wls input vol error, rc=%d\n", rc);
		return rc;
	}
	*vol_mv = pst->prop[WLS_VOLT_NOW] / 1000;

	return rc;
}

static int oplus_chg_wls_aicl_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	int rc = 0;
	struct psy_state *pst = NULL;
	struct battery_chg_dev *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &chip->psy_list[PSY_TYPE_WLS];

	if (chip->soccp_support)
		rc = write_property_id(chip, &chip->oplus_psy, OPLUS_DC_AICL_ENABLE, en);
	else
		rc = write_property_id(chip, pst, WLS_BOOST_AICL_ENABLE, en);
	if (rc < 0)
		chg_err("can't %s aicl, rc=%d\n", en ? "enable" : " disable", rc);

	return rc;
}

static int oplus_chg_wls_aicl_rerun(struct oplus_chg_ic_dev *ic_dev)
{
	int rc = 0;
	struct psy_state *pst = NULL;
	struct battery_chg_dev *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &chip->psy_list[PSY_TYPE_WLS];

	if (chip->soccp_support)
		rc = write_property_id(chip, &chip->oplus_psy, OPLUS_DC_AICL_RERUN, 1);
	else
		rc = write_property_id(chip, pst, WLS_BOOST_AICL_RERUN, 1);
	if (rc < 0)
		chg_err("can't rerun aicl, rc=%d\n", rc);

	return rc;
}

static int oplus_chg_8350_get_usb_aicl_enhance(struct oplus_chg_ic_dev *ic_dev, bool *enable)
{
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	*enable = bcdev->usb_aicl_enhance;

	return 0;
}

static int oplus_chg_8350_set_aicl_vol(struct battery_chg_dev *bcdev, int vol)
{
	struct psy_state *pst = NULL;
	int rc = 0;

	if (bcdev == NULL) {
		chg_err("bcdev is NULL");
		return -ENODEV;
	}

	pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_SET_AICL_VOL, vol);
	else
		rc = write_property_id(bcdev, pst, USB_SET_AICL_VOL, vol);
	if (rc)
		chg_err("set aicl vol fail, rc=%d\n", rc);

	return rc;
}

static int oplus_chg_8350_get_aicl_vol(struct battery_chg_dev *bcdev, int *vol)
{
	struct psy_state *pst = NULL;
	int rc = 0;

	if (bcdev == NULL) {
		chg_err("bcdev is NULL");
		return -ENODEV;
	}

	pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_GET_AICL_VOL);
	else
		rc = read_property_id(bcdev, pst, USB_GET_AICL_VOL);
	if (rc < 0) {
		chg_err("get aicl vol fail, rc = %d\n", rc);
		return -1;
	}

	if (bcdev->soccp_support)
		*vol = bcdev->oplus_psy.prop[OPLUS_USB_GET_AICL_VOL];
	else
		*vol = pst->prop[USB_GET_AICL_VOL];

	return rc;
}

static int oplus_chg_set_aicl_point(struct oplus_chg_ic_dev *ic_dev, int vbatt)
{
	struct battery_chg_dev *bcdev;
	int hw_aicl_point = HW_AICL_POINT_VOL_5V_PHASE1;
	int curr_aicl = 0;
	int type = 0;
	bool present = false;
	int aicl_thr_index = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (!bcdev || !bcdev->usb_aicl_enhance)
		return 0;

	if (bcdev->aicl_thr_table_init) {
		aicl_thr_index = oplus_chg_get_aicl_thr_index(bcdev, vbatt);
		hw_aicl_point = bcdev->aicl_thr_table[aicl_thr_index].hw_aicl;
	} else {
		if (vbatt > AICL_POINT_VOL_5V)
			hw_aicl_point = HW_AICL_POINT_VOL_5V_PHASE2;
		else if (vbatt <= AICL_POINT_VOL_5V)
			hw_aicl_point = HW_AICL_POINT_VOL_5V_PHASE1;
	}

	oplus_chg_8350_input_present(bcdev->buck_ic, &present);
	oplus_chg_8350_get_charger_type(bcdev->buck_ic, &type);
	if (bcdev->charger_type == OPLUS_CHG_USB_TYPE_SDP && present)
		hw_aicl_point = USB_HW_AICL_POINT;

	oplus_chg_8350_get_aicl_vol(bcdev, &curr_aicl);
	if (!present && curr_aicl <= hw_aicl_point)
		return 0;
	if (curr_aicl != hw_aicl_point) {
		chg_info("curr_aicl = %d new aicl %d\n", curr_aicl, hw_aicl_point);
		oplus_chg_8350_set_aicl_vol(bcdev, hw_aicl_point);
	}

	return 0;
}

#define ITERM_RECHECK_WORK_MS 1500
static int oplus_chg_8350_iterm_check(struct oplus_chg_ic_dev *ic_dev, bool check)
{
	struct battery_chg_dev *bcdev;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (!bcdev) {
		chg_err("bcdev not ready!\n");
		return -ENODEV;
	}

	oplus_adsp_set_iterm_check_status(bcdev, check);
	chg_info("[%d, %d]\n", bcdev->chg_en, oplus_get_batt_full_status(bcdev));
	cancel_delayed_work_sync(&bcdev->iterm_timeout_work);
	schedule_delayed_work(&bcdev->iterm_timeout_work, msecs_to_jiffies(ITERM_RECHECK_WORK_MS));
	return rc;
}

static int oplus_get_supplementary_power_mos_enable(struct oplus_chg_ic_dev *ic_dev, bool *enable)
{
	struct battery_chg_dev *bcdev;
	struct oplus_custom_gpio_pinctrl *poplus_custom_gpio;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (!bcdev) {
		chg_err("bcdev not ready!\n");
		return -ENODEV;
	}


	poplus_custom_gpio = &bcdev->oplus_custom_gpio;
	if (!gpio_is_valid(poplus_custom_gpio->supplementary_power_mos_gpio) ||
	    IS_ERR_OR_NULL(poplus_custom_gpio->supplementary_power_pinctrl) ||
	    IS_ERR_OR_NULL(poplus_custom_gpio->supplementary_power_mos_active) ||
	    IS_ERR_OR_NULL(poplus_custom_gpio->supplementary_power_mos_sleep)) {
		chg_info("supplementary_power pmos pinctrl no support\n");
		return -EINVAL;
	}


	if (gpio_get_value_cansleep(poplus_custom_gpio->supplementary_power_mos_gpio))
		*enable = true;
	else
		*enable = false;

	return 0;
}

static bool oplus_chg_supplementary_power_mos_track_limit(void)
{
	static int mos_upload_count = 0;
	static int pre_upload_time = 0;
	int curr_time;

	curr_time = local_clock() / TRACK_LOCAL_T_NS_TO_S_THD;
	if (curr_time - pre_upload_time > TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD)
		mos_upload_count = 0;

	chg_info("mos_err_upload_count %d max cnt %d",
		mos_upload_count, TRACK_UPLOAD_COUNT_MAX);

	if (mos_upload_count >= TRACK_UPLOAD_COUNT_MAX)
		return true;

	pre_upload_time = local_clock() / TRACK_LOCAL_T_NS_TO_S_THD;
	mos_upload_count++;

	return false;
}

static int oplus_chg_supplementary_power_mos_track(struct oplus_chg_ic_dev *ic_dev,
					bool enable)
{
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (bcdev == NULL) {
		chg_err("bcdev is NULL");
		return -ENODEV;
	}


	if (oplus_is_rf_ftm_mode() || oplus_chg_supplementary_power_mos_track_limit())
		return 0;

	oplus_chg_ic_creat_err_msg(bcdev->buck_ic,
		OPLUS_IC_ERR_GPIO, 0, "$$err_reason@@supplementary_power_mos$$expected@@%d", enable);
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_ERR);

	return 0;
}

static bool oplus_chg_supplementary_check_power_mos(struct oplus_chg_ic_dev *ic_dev, bool mos_status)
{
	int current_mos_gpio = 0;
	struct battery_chg_dev *bcdev;
	bool rc = false;

	if (ic_dev == NULL) {
		chg_err("battery_chg_dev is NULL");
		return false;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (bcdev == NULL) {
		chg_err("bcdev is NULL");
		return false;
	}

	if (bcdev->power_mos_status != mos_status) {
		bcdev->mos_retry_cnt = 2;
		return true;
	}
	if (!bcdev->need_check_mos)
		rc = false;

	if (bcdev->need_check_mos) {
		bcdev->need_check_mos = false;
		current_mos_gpio = gpio_get_value_cansleep(bcdev->oplus_custom_gpio.supplementary_power_mos_gpio);
		chg_info("power_mos_status:%d  gpio_val %d\n", bcdev->power_mos_status, current_mos_gpio);
		if (current_mos_gpio != bcdev->power_mos_status) {
			if (bcdev->mos_retry_cnt > 0) {
				rc = true;
				bcdev->mos_retry_cnt--;
			} else {
				rc = false;
				bcdev->mos_retry_cnt = 0;
				oplus_chg_supplementary_power_mos_track(ic_dev, bcdev->power_mos_status);
			}
		} else {
			rc = false;
			bcdev->mos_retry_cnt = 2;
		}
	}

	return rc;
}

static int oplus_set_supplementary_power_mos_enable(struct oplus_chg_ic_dev *ic_dev, bool enable)
{
	int rc;
	struct battery_chg_dev *bcdev;
	bool mos_status = enable;
	struct oplus_custom_gpio_pinctrl *poplus_custom_gpio;

	if (ic_dev == NULL) {
		chg_err("battery_chg_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (bcdev == NULL) {
		chg_err("bcdev is NULL");
		return -ENODEV;
	}

	if (oplus_check_supplementary_power_pinctrl(bcdev) != 0)
		return -EINVAL;

	poplus_custom_gpio = &bcdev->oplus_custom_gpio;
	if (!oplus_chg_supplementary_check_power_mos(ic_dev, enable))
		return 0;

	mutex_lock(&poplus_custom_gpio->pinctrl_mutex);
	bcdev->power_mos_status = mos_status;

	if (mos_status)
		rc = pinctrl_select_state(poplus_custom_gpio->supplementary_power_pinctrl,
			poplus_custom_gpio->supplementary_power_mos_active);
	else
		rc = pinctrl_select_state(poplus_custom_gpio->supplementary_power_pinctrl,
			poplus_custom_gpio->supplementary_power_mos_sleep);

	bcdev->need_check_mos = true;
	mutex_unlock(&poplus_custom_gpio->pinctrl_mutex);

	if (rc < 0) {
		chg_err("supplementary power mos can't %s\n", mos_status ? "enable" : "disable");
	} else {
		rc = 0;
		chg_info("set value:%d sucess \n", mos_status);
	}

	return rc;
}

static int oplus_supplementary_power_mos_config(struct oplus_chg_ic_dev *ic_dev, bool enable)
{
	bool mos_status = enable;
	int rc;

	if (oplus_is_rf_ftm_mode())
		mos_status = false;

	rc = oplus_set_supplementary_power_mos_enable(ic_dev, mos_status);

	return rc;
}

#ifdef OPLUS_FEATURE_CHG_BASIC

static int oplus_chg_adsp_set_plc_status(struct battery_chg_dev *bcdev, int status)
{
	struct psy_state *pst;
	int rc;

	if (bcdev == NULL) {
		chg_err("bcdev is NULL");
		return -ENODEV;
	}

	pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_SET_PLC_STATUS, status);
	else
		rc = write_property_id(bcdev, pst, USB_SET_PLC_STATUS, status);
	if (rc)
		chg_err("set plc status fail, rc=%d\n", rc);

	return rc;
}

static int oplus_set_usb_dpdm_ovp_disable(struct oplus_chg_ic_dev *ic_dev, bool disable)
{
	struct psy_state *pst = NULL;
	int rc = 0;
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (!bcdev) {
		chg_err("bcdev is NULL");
		return -ENODEV;
	}

	pst = &bcdev->psy_list[PSY_TYPE_USB];
	if (bcdev->soccp_support) {
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_WCD_OVP_DISABLE, disable);
		chg_info("set USB_WCD_OVP_DISABLE=%d, rc=%d\n", disable,  rc);
	}
	if (rc)
		chg_err("oplus_set_usb_dpdm_ovp_disable=%d error, rc = %d\n", disable, rc);

	return rc;
}

static int oplus_chg_get_vdm_info(struct oplus_chg_ic_dev *ic_dev, u32 *data, int *cnt)
{
	struct battery_chg_dev *bcdev;
	int copy_cnt;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (data == NULL || cnt == NULL) {
		chg_err("data or cnt is NULL");
		return -EINVAL;
	}
	if (*cnt < 0) {
		chg_err("cnt is invalid");
		return -EINVAL;
	}

	mutex_lock(&bcdev->vdm_info_lock);
	copy_cnt = min(*cnt, bcdev->vdm_info_cnt);
	memmove(data, bcdev->vdm_info_data, sizeof(u32) * copy_cnt);
	*cnt = copy_cnt;
	mutex_unlock(&bcdev->vdm_info_lock);

	return 0;
}

static int oplus_send_get_sink_cap(struct battery_chg_dev *bcdev)
{
	struct psy_state *pst = NULL;
	int rc = 0;
	pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (bcdev->soccp_support) {
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_SET_PD_SINK_CAP, true);
	} else {
		rc = write_property_id(bcdev, pst, USB_PD_SEND_GET_SINK_CAP, true);
	}
	if (rc)
		chg_err("adsp send get_sink_cap failed, rc=%d\n", rc);

	return rc;
}

static int oplus_chg_send_get_sink_cap(struct oplus_chg_ic_dev *ic_dev)
{
	struct battery_chg_dev *bcdev;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (bcdev == NULL) {
		chg_err("bcdev is NULL");
		return -ENODEV;
	}

	if (!bcdev->otg_online) {
		chg_err("not source, skip");
		return -EINVAL;
	}

	rc = oplus_send_get_sink_cap(bcdev);
	return rc;
}

static void *oplus_chg_8350_buck_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, oplus_chg_8350_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, oplus_chg_8350_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP, oplus_chg_8350_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST, oplus_chg_8350_smt_test);
		break;
	case OPLUS_IC_FUNC_BUCK_INPUT_PRESENT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_INPUT_PRESENT, oplus_chg_8350_input_present);
		break;
	case OPLUS_IC_FUNC_BUCK_INPUT_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_INPUT_SUSPEND, oplus_chg_8350_input_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_INPUT_IS_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_INPUT_IS_SUSPEND, oplus_chg_8350_input_is_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_OUTPUT_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_OUTPUT_SUSPEND, oplus_chg_8350_output_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_OUTPUT_IS_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_OUTPUT_IS_SUSPEND, oplus_chg_8350_output_is_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_ICL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_ICL, oplus_chg_8350_set_icl);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_ICL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_ICL, oplus_chg_8350_get_icl);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_FCC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_FCC, oplus_chg_8350_set_fcc);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_FV:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_FV, oplus_chg_8350_set_fv);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_ITERM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_ITERM, oplus_chg_8350_set_iterm);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_RECHG_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_RECHG_VOL, oplus_chg_8350_set_rechg_vol);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_INPUT_CURR:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_INPUT_CURR, oplus_chg_8350_get_input_curr);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_INPUT_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_INPUT_VOL, oplus_chg_8350_get_input_vol);
		break;
	case OPLUS_IC_FUNC_OTG_BOOST_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_OTG_BOOST_ENABLE, oplus_chg_8350_otg_boost_enable);
		break;
	case OPLUS_IC_FUNC_SET_OTG_BOOST_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_OTG_BOOST_VOL, oplus_chg_8350_set_otg_boost_vol);
		break;
	case OPLUS_IC_FUNC_SET_OTG_BOOST_CURR_LIMIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_OTG_BOOST_CURR_LIMIT, oplus_chg_8350_set_otg_boost_curr_limit);
		break;
	case OPLUS_IC_FUNC_BUCK_AICL_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_AICL_ENABLE, oplus_chg_8350_aicl_enable);
		break;
	case OPLUS_IC_FUNC_BUCK_AICL_RERUN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_AICL_RERUN, oplus_chg_8350_aicl_rerun);
		break;
	case OPLUS_IC_FUNC_BUCK_AICL_RESET:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_AICL_RESET, oplus_chg_8350_aicl_reset);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_CC_ORIENTATION:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_CC_ORIENTATION, oplus_chg_8350_get_cc_orientation);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_HW_DETECT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_HW_DETECT, oplus_chg_8350_get_hw_detect);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_POWER_ROLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_POWER_ROLE, oplus_chg_8350_get_power_role);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_CHARGER_TYPE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_CHARGER_TYPE, oplus_chg_8350_get_charger_type);
		break;
	case OPLUS_IC_FUNC_BUCK_RERUN_BC12:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_RERUN_BC12, oplus_chg_8350_rerun_bc12);
		break;
	case OPLUS_IC_FUNC_BUCK_QC_DETECT_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_QC_DETECT_ENABLE, oplus_chg_8350_qc_detect_enable);
		break;
	case OPLUS_IC_FUNC_BUCK_SHIPMODE_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SHIPMODE_ENABLE, oplus_chg_8350_shipmode_enable);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_QC_CONFIG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_QC_CONFIG, oplus_chg_8350_set_qc_config);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_PD_CONFIG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_PD_CONFIG, oplus_chg_8350_set_pd_config);
		break;
	case OPLUS_IC_FUNC_GAUGE_UPDATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_UPDATE, oplus_chg_8350_get_props_from_adsp_by_buffer);
		break;
	case OPLUS_IC_FUNC_VOOCPHY_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_VOOCPHY_ENABLE, oplus_chg_8350_voocphy_enable);
		break;
	case OPLUS_IC_FUNC_VOOCPHY_RESET_AGAIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_VOOCPHY_RESET_AGAIN, oplus_chg_8350_voocphy_reset_again);
		break;
	case OPLUS_IC_FUNC_GET_CHARGER_CYCLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_CHARGER_CYCLE, oplus_chg_8350_get_charger_cycle);
		break;
	case OPLUS_IC_FUNC_GET_SHUTDOWN_SOC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_SHUTDOWN_SOC, oplus_chg_8350_get_shutdown_soc);
		break;
	case OPLUS_IC_FUNC_BACKUP_SOC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BACKUP_SOC, oplus_chg_8350_backup_soc);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_VBUS_COLLAPSE_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_VBUS_COLLAPSE_STATUS, oplus_chg_8350_get_vbus_collapse_status);
		break;
	case OPLUS_IC_FUNC_GET_TYPEC_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_TYPEC_MODE, oplus_chg_8350_get_typec_mode);
		break;
	case OPLUS_IC_FUNC_SET_TYPEC_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_TYPEC_MODE, oplus_chg_8350_set_typec_mode);
		break;
	case OPLUS_IC_FUNC_SET_OTG_SWITCH_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_OTG_SWITCH_STATUS, oplus_chg_8350_set_otg_switch_status);
		break;
	case OPLUS_IC_FUNC_GET_OTG_SWITCH_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_OTG_SWITCH_STATUS, oplus_chg_8350_get_otg_switch_status);
		break;
	case OPLUS_IC_FUNC_CC_DETECT_HAPPENED:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CC_DETECT_HAPPENED, oplus_chg_8350_cc_detect_happened);
		break;
	case OPLUS_IC_FUNC_VOOCPHY_SET_CURR_LEVEL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_VOOCPHY_SET_CURR_LEVEL, oplus_chg_8350_set_curr_level);
		break;
	case OPLUS_IC_FUNC_VOOCPHY_SET_MATCH_TEMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_VOOCPHY_SET_MATCH_TEMP, oplus_chg_8350_set_match_temp);
		break;
	case OPLUS_IC_FUNC_GET_OTG_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_OTG_ENABLE, oplus_chg_8350_get_otg_enbale);
		break;
	case OPLUS_IC_FUNC_IS_OPLUS_SVID:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_IS_OPLUS_SVID, oplus_chg_8350_is_oplus_svid);
		break;
	case OPLUS_IC_FUNC_BUCK_HARDWARE_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_HARDWARE_INIT, oplus_chg_8350_hardware_init);
		break;
	case OPLUS_IC_FUNC_VOOCPHY_SET_BCC_CURR:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_VOOCPHY_SET_BCC_CURR, oplus_set_bcc_curr_to_voocphy);
		break;

	case OPLUS_IC_FUNC_WLS_BOOST_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_WLS_BOOST_ENABLE, oplus_chg_wls_boost_enable);
		break;
	case OPLUS_IC_FUNC_SET_WLS_BOOST_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_WLS_BOOST_VOL, oplus_chg_set_wls_boost_vol);
		break;
	case OPLUS_IC_FUNC_SET_WLS_BOOST_CURR_LIMIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_WLS_BOOST_CURR_LIMIT, oplus_chg_set_wls_boost_curr_limit);
		break;

	case OPLUS_IC_FUNC_BUCK_WLS_INPUT_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_WLS_INPUT_SUSPEND, oplus_chg_wls_input_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_WLS_ICL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_WLS_ICL, oplus_chg_set_wls_icl);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_WLS_ICL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_WLS_ICL, oplus_chg_get_wls_icl);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_WLS_INPUT_CURR:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_WLS_INPUT_CURR, oplus_chg_get_wls_input_curr);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_WLS_INPUT_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_WLS_INPUT_VOL, oplus_chg_get_wls_input_vol);
		break;
	case OPLUS_IC_FUNC_BUCK_WLS_AICL_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_WLS_AICL_ENABLE, oplus_chg_wls_aicl_enable);
		break;
	case OPLUS_IC_FUNC_BUCK_WLS_AICL_RERUN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_WLS_AICL_RERUN, oplus_chg_wls_aicl_rerun);
		break;
	case OPLUS_IC_FUNC_GET_TYPEC_ROLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_TYPEC_ROLE, oplus_chg_8350_get_typec_mode);
		break;
	case OPLUS_IC_FUNC_BUCK_DIS_INSERT_DETECT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_DIS_INSERT_DETECT, oplus_chg_8350_set_usb_drv);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_USB_BTB_TEMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_USB_BTB_TEMP, oplus_chg_adsp_get_usb_btb_temp);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_BATT_BTB_TEMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_BATT_BTB_TEMP, oplus_chg_adsp_get_batt_btb_temp);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_TYPEC_STATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_TYPEC_STATE, oplus_chg_8350_get_cc_orientation);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_USB_AICL_ENHANCE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_USB_AICL_ENHANCE, oplus_chg_8350_get_usb_aicl_enhance);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_AICL_POINT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_AICL_POINT, oplus_chg_set_aicl_point);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_LPD_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_LPD_INFO, oplus_sm8350_get_lpd_info);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_CHG_PATH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_CHG_PATH, oplus_chg_set_chg_path);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_CHG_PATH_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_CHG_PATH_STATUS, oplus_chg_get_chg_path_status);
		break;
	case OPLUS_IC_FUNC_BUCK_ITEM_CHECK:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_ITEM_CHECK, oplus_chg_8350_iterm_check);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_SHAFT_BATT_BTB_TEMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_SHAFT_BATT_BTB_TEMP, oplus_chg_adsp_get_shaft_btb_temp);
		break;
	case OPLUS_IC_FUNC_BUCK_PUSH_SHAFT_BATT_BTB_OVER:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_PUSH_SHAFT_BATT_BTB_OVER, oplus_chg_adsp_set_shaft_btb_over);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_POWER_MOS_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_POWER_MOS_ENABLE, oplus_supplementary_power_mos_config);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_POWER_MOS_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_POWER_MOS_ENABLE, oplus_get_supplementary_power_mos_enable);
		break;
	case OPLUS_IC_FUNC_GET_SOURCE_PDO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_SOURCE_PDO,
					       oplus_chg_get_source_pdo);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_OVP_FORCED:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_OVP_FORCED, oplus_chg_set_ovp_forced);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_DPDM_OVP_DISABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_DPDM_OVP_DISABLE, oplus_set_usb_dpdm_ovp_disable);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_VDM_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_VDM_INFO, oplus_chg_get_vdm_info);
		break;
	case OPLUS_IC_FUNC_BUCK_SEND_GET_SINK_CAP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SEND_GET_SINK_CAP, oplus_chg_send_get_sink_cap);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq oplus_chg_8350_buck_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_CC_DETECT },
	{ .virq_id = OPLUS_IC_VIRQ_PLUGIN },
	{ .virq_id = OPLUS_IC_VIRQ_CC_CHANGED },
	{ .virq_id = OPLUS_IC_VIRQ_SUSPEND_CHECK },
	{ .virq_id = OPLUS_IC_VIRQ_CHG_TYPE_CHANGE },
	{ .virq_id = OPLUS_IC_VIRQ_OTG_ENABLE },
	{ .virq_id = OPLUS_IC_VIRQ_RESUME },
	{ .virq_id = OPLUS_IC_VIRQ_SVID },
	{ .virq_id = OPLUS_IC_VIRQ_TYPEC_STATE},
	{.virq_id = OPLUS_IC_VIRQ_POWER_ROLE_STATUS},
	{.virq_id = OPLUS_IC_VIRQ_POWER_CHANGED},
	{.virq_id = OPLUS_IC_VIRQ_PD_COMPLETED },
};

static int oplus_sm8350_init(struct oplus_chg_ic_dev *ic_dev)
{
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("ic_dev null\n");
		return -ENODEV;
	}

	ic_dev->online = true;
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (!bcdev) {
		chg_err("bcdev null\n");
		return -ENODEV;
	}

	oem_read_buffer(bcdev);
	bcdev->qcom_gauge_cali_track_support = bcdev->read_buffer_dump.data_buffer[14];
	chg_info("qcom_gauge_cali_track_support = %d\n", bcdev->qcom_gauge_cali_track_support);

	if (bcdev->qcom_gauge_cali_track_support) {
		mutex_lock(&bcdev->pre_info_lock);
		oplus_chg_update_gauge_cali_track_info_internal(bcdev, &pre_info);
		mutex_unlock(&bcdev->pre_info_lock);
	}

	return 0;
}

static int oplus_sm8350_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (!ic_dev->online)
		return 0;

	ic_dev->online = false;

	return 0;
}

static int oplus_sm8350_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	return 0;
}

static int oplus_sm8350_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	return 0;
}

static int oplus_sm8350_get_batt_max(struct oplus_chg_ic_dev *ic_dev,
				      int *vol_mv)
{
	struct battery_chg_dev *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*vol_mv = fg_sm8350_get_battery_mvolts_2cell_max();

	return 0;
}

static int oplus_sm8350_get_batt_min(struct oplus_chg_ic_dev *ic_dev,
				      int *vol_mv)
{
	struct battery_chg_dev *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*vol_mv = fg_sm8350_get_battery_mvolts_2cell_min();

	return 0;
}

static int oplus_sm8350_get_batt_curr(struct oplus_chg_ic_dev *ic_dev,
				       int *curr_ma)
{
	struct battery_chg_dev *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*curr_ma = fg_sm8350_get_average_current();

	return 0;
}

static int oplus_sm8350_get_batt_temp(struct oplus_chg_ic_dev *ic_dev,
				       int *temp)
{
	struct battery_chg_dev *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*temp = fg_sm8350_get_battery_temperature();

	return 0;
}

static int oplus_sm8350_get_batt_soc(struct oplus_chg_ic_dev *ic_dev, int *soc)
{
	struct battery_chg_dev *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*soc = fg_sm8350_get_battery_soc();

	return 0;
}

static int oplus_sm8350_get_batt_fcc(struct oplus_chg_ic_dev *ic_dev, int *fcc)
{
	struct battery_chg_dev *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*fcc = fg_sm8350_get_battery_fcc();

	return 0;
}

static int oplus_sm8350_get_batt_cc(struct oplus_chg_ic_dev *ic_dev, int *cc)
{
	struct battery_chg_dev *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*cc = fg_sm8350_get_battery_cc();

	return 0;
}

static int oplus_sm8350_get_batt_rm(struct oplus_chg_ic_dev *ic_dev, int *rm)
{
	struct battery_chg_dev *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*rm = fg_sm8350_get_batt_remaining_capacity();

	return 0;
}

static int oplus_sm8350_get_batt_soh(struct oplus_chg_ic_dev *ic_dev, int *soh)
{
	struct battery_chg_dev *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*soh = fg_sm8350_get_battery_soh();

	return 0;
}

static int oplus_sm8350_get_batt_auth(struct oplus_chg_ic_dev *ic_dev,
				       bool *pass)
{
	*pass = fg_sm8350_get_battery_authenticate();

	return 0;
}

static int oplus_sm8350_get_batt_hmac(struct oplus_chg_ic_dev *ic_dev,
				       bool *pass)
{
	struct battery_chg_dev *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*pass = fg_sm8350_get_battery_hmac(chip);

	return 0;
}

static int oplus_sm8350_get_vbatt_fcl(struct oplus_chg_ic_dev *ic_dev, int *vol_mv)
{
	struct battery_chg_dev *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	oplus_chg_get_pmic_vbat_vol(chip, vol_mv);

	return 0;
}

static int oplus_sm8350_set_batt_full(struct oplus_chg_ic_dev *ic_dev,
				       bool full)
{
	return fg_sm8350_set_battery_full(ic_dev, full);
}

static int oplus_sm8350_update_dod0(struct oplus_chg_ic_dev *ic_dev)
{
	return fg_bq28z610_modify_dod0();
}

static int
oplus_sm8350_update_soc_smooth_parameter(struct oplus_chg_ic_dev *ic_dev)
{
	struct battery_chg_dev *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	return fg_bq28z610_update_soc_smooth_parameter(chip);
}

static int oplus_sm8350_get_cb_status(struct oplus_chg_ic_dev *ic_dev,
				       int *status)
{
	*status = fg_bq28z610_get_battery_balancing_status();

	return 0;
}

static int oplus_sm8350_set_lock(struct oplus_chg_ic_dev *ic_dev, bool lock)
{
	return 0;
}

static int oplus_sm8350_is_locked(struct oplus_chg_ic_dev *ic_dev, bool *locked)
{
	*locked = false;
	return 0;
}

static int oplus_sm8350_get_batt_num(struct oplus_chg_ic_dev *ic_dev, int *num)
{
	struct battery_chg_dev *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*num = chip->batt_num;

	return 0;
}

static int oplus_sm8350_get_device_type(struct oplus_chg_ic_dev *ic_dev,
					 int *type)
{
	*type = 0;

	return 0;
}

static int
oplus_sm8350_get_device_type_for_vooc(struct oplus_chg_ic_dev *ic_dev,
				       int *type)
{
	*type = 0;

	return 0;
}

static int
oplus_sm8350_get_battery_dod0(struct oplus_chg_ic_dev *ic_dev, int index,
				       int *val)
{
	struct battery_chg_dev *bcdev;

	if ((ic_dev == NULL) || (val == NULL)) {
		chg_err("!!!ic_dev null\n");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	switch (index) {
	case 0:
		*val = bcdev->bcc_read_buffer_dump.data_buffer[0];
		break;
	case 1:
		*val = bcdev->bcc_read_buffer_dump.data_buffer[1];
		break;
	default:
		chg_err("Unknown index(=%d), max is 2\n", index);
		return -EINVAL;
	}

	return 0;
}

static int
oplus_sm8350_get_battery_dod0_passed_q(struct oplus_chg_ic_dev *ic_dev,
				       int *val)
{
	struct battery_chg_dev *bcdev;

	if ((ic_dev == NULL) || (val == NULL)) {
		chg_err("!!!ic_dev null\n");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	*val = bcdev->bcc_read_buffer_dump.data_buffer[2];

	return 0;
}

static int
oplus_sm8350_get_battery_qmax(struct oplus_chg_ic_dev *ic_dev, int index,
				       int *val)
{
	struct battery_chg_dev *bcdev;

	if ((ic_dev == NULL) || (val == NULL)) {
		chg_err("!!!ic_dev null\n");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	switch (index) {
	case 0:
		*val = bcdev->bcc_read_buffer_dump.data_buffer[3];
		break;
	case 1:
		*val = bcdev->bcc_read_buffer_dump.data_buffer[4];
		break;
	default:
		chg_err("Unknown index(=%d), max is 2\n", index);
		return -EINVAL;
	}

	return 0;
}

static int
oplus_sm8350_get_battery_qmax_passed_q(struct oplus_chg_ic_dev *ic_dev,
				       int *val)
{
	struct battery_chg_dev *bcdev;

	if ((ic_dev == NULL) || (val == NULL)) {
		chg_err("!!!ic_dev null\n");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	*val = bcdev->bcc_read_buffer_dump.data_buffer[5];

	return 0;
}

static int
oplus_sm8350_get_batt_vol(struct oplus_chg_ic_dev *ic_dev, int index,
				       int *val)
{
	struct battery_chg_dev *bcdev;

	if ((ic_dev == NULL) || (val == NULL)) {
		chg_err("!!!ic_dev null\n");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	switch (index) {
	case 0:
		*val = bcdev->bcc_read_buffer_dump.data_buffer[6];
		break;
	case 1:
		*val = bcdev->bcc_read_buffer_dump.data_buffer[11];
		break;
	default:
		chg_err("Unknown index(=%d), max is 2\n", index);
		return -EINVAL;
	}

	return 0;
}

static int
oplus_sm8350_get_battery_gauge_type_for_bcc(struct oplus_chg_ic_dev *ic_dev,
				       int *type)
{
	struct battery_chg_dev *bcdev;

	if ((ic_dev == NULL) || (type == NULL)) {
		chg_err("!!!ic_dev null\n");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (bcdev->bcc_read_buffer_dump.data_buffer[11] == SW_GAUGE) {
		*type = DEVICE_ZY0603;
	} else {
		*type = DEVICE_BQ27541;
	}

	return 0;
}

static int
oplus_sm8350_get_real_time_current(struct oplus_chg_ic_dev *ic_dev,
				       int *val)
{
	struct battery_chg_dev *bcdev;

	if ((ic_dev == NULL) || (val == NULL)) {
		chg_err("!!!ic_dev null\n");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	*val = bcdev->bcc_read_buffer_dump.data_buffer[8];

	return 0;
}

static int oplus_sm8350_get_reg_info(struct oplus_chg_ic_dev *ic_dev, u8 *info, int len)
{
	struct battery_chg_dev *bcdev;
	int index = 0;
	int rc = 0;

	if (ic_dev == NULL || !info) {
		chg_err("oplus_chg_ic_dev or info is NULL");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (!bcdev || !bcdev->ap_read_buffer_dump)
		return -ENODEV;

	mutex_lock(&bcdev->ap_read_buffer_lock);
	rc = ap_set_message_id(bcdev, AP_MESSAGE_GET_GAUGE_REG_INFO, 0);
	if (rc)
		goto err;

	reinit_completion(&bcdev->ap_read_ack[AP_MESSAGE_GET_GAUGE_REG_INFO]);
	rc = wait_for_completion_timeout(&bcdev->ap_read_ack[AP_MESSAGE_GET_GAUGE_REG_INFO],
					 msecs_to_jiffies(AP_READ_WAIT_TIME_MS));
	if (!rc) {
		chg_err("Error, timed out sending message\n");
		goto err;
	}

	index = bcdev->ap_read_buffer_dump->data_size;
	if (index >= len)
		goto err;

	memcpy(info, bcdev->ap_read_buffer_dump->data_buffer, index);
	memset(bcdev->ap_read_buffer_dump, 0, sizeof(*bcdev->ap_read_buffer_dump));
	mutex_unlock(&bcdev->ap_read_buffer_lock);
	return index;
err:
	memset(bcdev->ap_read_buffer_dump, 0, sizeof(*bcdev->ap_read_buffer_dump));
	mutex_unlock(&bcdev->ap_read_buffer_lock);
	return -EINVAL;
}

static int oplus_sm8350_get_lpd_info(struct oplus_chg_ic_dev *ic_dev, u32 *buf, u32 flag)
{
	int index = 0;
	int rc = 0;
	int i = 0;
	u8 info[OPLUS_LPD_SEL_INVALID * 4] = {0};
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (bcdev == NULL || buf == NULL || !bcdev->ap_read_buffer_dump) {
		chg_err("oplus_chg_ic_dev or info is NULL");
		return -ENODEV;
	}

	mutex_lock(&bcdev->ap_read_buffer_lock);
	rc = ap_set_message_id(bcdev, AP_MESSAGE_GET_LPD_INFO, flag);
	if (rc)
		goto err;

	reinit_completion(&bcdev->ap_read_ack[AP_MESSAGE_GET_LPD_INFO]);
	rc = wait_for_completion_timeout(&bcdev->ap_read_ack[AP_MESSAGE_GET_LPD_INFO],
					 msecs_to_jiffies(AP_READ_WAIT_TIME_MS));
	if (!rc) {
		chg_err("Error, timed out sending message\n");
		goto err;
	}

	index = bcdev->ap_read_buffer_dump->data_size;
	if (index > OPLUS_LPD_SEL_INVALID * 4)
		goto err;

	memcpy(info, bcdev->ap_read_buffer_dump->data_buffer, index);
	memset(bcdev->ap_read_buffer_dump, 0, sizeof(*bcdev->ap_read_buffer_dump));
	mutex_unlock(&bcdev->ap_read_buffer_lock);
	for (i = 0; i < OPLUS_LPD_SEL_INVALID; i++) {
		if (flag & (0x1 << i))
			buf[i] = info[4 * i] | (info[4 * i + 1] << 8) |
				(info[4 * i + 2] << 16) | (info[4 * i + 3] << 24);
	}
	return index;
err:
	memset(bcdev->ap_read_buffer_dump, 0, sizeof(*bcdev->ap_read_buffer_dump));
	mutex_unlock(&bcdev->ap_read_buffer_lock);
	return -EINVAL;
}

static int oplus_sm8350_get_calib_time(
	struct oplus_chg_ic_dev *ic_dev, int *dod_calib_time, int *qmax_calib_time, char *calib_args, int len)
{
	struct battery_chg_dev *bcdev;
	int rc = 0;
	struct gauge_calib_info info = { 0 };

	if (ic_dev == NULL)
		return -ENODEV;

	if (calib_args == NULL || !dod_calib_time || !qmax_calib_time || !len)
		return -EINVAL;

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (!bcdev || !bcdev->ap_read_buffer_dump)
		return -ENODEV;

	if (!bcdev->calib_info_init) {
		*dod_calib_time = -1;
		*qmax_calib_time = -1;
		return 0;
	}

	mutex_lock(&bcdev->ap_read_buffer_lock);
	rc = ap_set_message_id(bcdev, AP_MESSAGE_GET_GAUGE_CALIB_TIME, 0);
	if (rc)
		goto err;

	reinit_completion(&bcdev->ap_read_ack[AP_MESSAGE_GET_GAUGE_CALIB_TIME]);
	rc = wait_for_completion_timeout(&bcdev->ap_read_ack[AP_MESSAGE_GET_GAUGE_CALIB_TIME],
					 msecs_to_jiffies(AP_READ_WAIT_TIME_MS));
	if (!rc) {
		chg_err("Error, timed out sending message\n");
		goto err;
	}

	memcpy(&info, bcdev->ap_read_buffer_dump->data_buffer, sizeof(struct gauge_calib_info));
	memset(bcdev->ap_read_buffer_dump, 0, sizeof(*bcdev->ap_read_buffer_dump));
	mutex_unlock(&bcdev->ap_read_buffer_lock);
	*dod_calib_time = info.dod_time;
	*qmax_calib_time = info.qmax_time;
	memcpy(calib_args, info.calib_args, GAUGE_CALIB_ARGS_LEN);
	return 0;
err:
	memset(bcdev->ap_read_buffer_dump, 0, sizeof(*bcdev->ap_read_buffer_dump));
	mutex_unlock(&bcdev->ap_read_buffer_lock);
	return -EINVAL;
}

static int oplus_sm8350_set_calib_time(struct oplus_chg_ic_dev *ic_dev,
	int dod_calib_time, int qmax_calib_time, char *calib_args, int len)
{
	struct battery_chg_dev *bcdev;
	struct gauge_calib_info info = { 0 };

	if (ic_dev == NULL)
		return -ENODEV;

	if (calib_args == NULL || !len)
		return -EINVAL;

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (!bcdev)
		return -EINVAL;

	if (len != GAUGE_CALIB_ARGS_LEN) {
		chg_err("len not match\n");
		return -EINVAL;
	}

	if (dod_calib_time) {
		info.dod_time = dod_calib_time;
		info.qmax_time = qmax_calib_time;
	} else {
		info.dod_time = 1;
		info.qmax_time = 1;
	}
	memcpy(info.calib_args, calib_args, len);
	oplus_ap_write_buffer(bcdev, AP_MESSAGE_WRITE_CALIB_TIME, (char *)&info, sizeof(struct gauge_calib_info));
	bcdev->calib_info_init = true;
	return 0;
}

static int oplus_sm8350_get_gauge_r_info(struct oplus_chg_ic_dev *ic_dev, u8 *info, int len)
{
	struct battery_chg_dev *bcdev;
	int index = 0;
	int rc = 0;

	if (ic_dev == NULL || !info) {
		chg_err("oplus_chg_ic_dev or info is NULL");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (!bcdev || !bcdev->ap_read_buffer_dump)
		return -ENODEV;

	mutex_lock(&bcdev->ap_read_buffer_lock);
	rc = ap_set_message_id(bcdev, AP_MESSAGE_GET_GAUGE_R_INFO, 0);
	if (rc)
		goto err;

	reinit_completion(&bcdev->ap_read_ack[AP_MESSAGE_GET_GAUGE_R_INFO]);
	rc = wait_for_completion_timeout(&bcdev->ap_read_ack[AP_MESSAGE_GET_GAUGE_R_INFO],
					 msecs_to_jiffies(BC_WAIT_TIME_MS));
	if (!rc) {
		chg_err("Error, timed out sending message\n");
		goto err;
	}

	index = bcdev->ap_read_buffer_dump->data_size;
	if (index >= len)
		goto err;

	memcpy(info, bcdev->ap_read_buffer_dump->data_buffer, index);
	memset(bcdev->ap_read_buffer_dump, 0, sizeof(*bcdev->ap_read_buffer_dump));
	mutex_unlock(&bcdev->ap_read_buffer_lock);
	return index;
err:
	memset(bcdev->ap_read_buffer_dump, 0, sizeof(*bcdev->ap_read_buffer_dump));
	mutex_unlock(&bcdev->ap_read_buffer_lock);
	return -EINVAL;
}

static int oplus_get_three_level_term_volt(
			struct oplus_chg_ic_dev *ic_dev, char *args, int len)
{
	struct battery_chg_dev *bcdev;
	int rc = 0;

	if (ic_dev == NULL)
		return -ENODEV;

	if (args == NULL || len <= 0 || len > OPLUS_GAUGE_THREE_LEVEL_TERM_VOLT_LEN) {
		chg_err("len = %d is invalid or args is NULL.\n", len);
		return -EINVAL;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (!bcdev || !bcdev->ap_read_buffer_dump)
		return -ENODEV;

	mutex_lock(&bcdev->ap_read_buffer_lock);
	rc = ap_set_message_id(bcdev, AP_MESSAGE_GET_GAUGE_THREE_LEVEL_TERM_VOLT, 0);
	if (rc)
		goto err;

	reinit_completion(&bcdev->ap_read_ack[AP_MESSAGE_GET_GAUGE_THREE_LEVEL_TERM_VOLT]);
	rc = wait_for_completion_timeout(&bcdev->ap_read_ack[AP_MESSAGE_GET_GAUGE_THREE_LEVEL_TERM_VOLT],
					 msecs_to_jiffies(AP_READ_WAIT_TIME_MS));
	if (!rc) {
		chg_err("Error, timed out sending message\n");
		goto err;
	}

	memcpy(args, bcdev->ap_read_buffer_dump->data_buffer, len);
	memset(bcdev->ap_read_buffer_dump, 0, sizeof(*bcdev->ap_read_buffer_dump));
	mutex_unlock(&bcdev->ap_read_buffer_lock);

	return 0;
err:
	memset(bcdev->ap_read_buffer_dump, 0, sizeof(*bcdev->ap_read_buffer_dump));
	mutex_unlock(&bcdev->ap_read_buffer_lock);
	return -EINVAL;
}

static int oplus_get_three_level_term_volt0(
			struct oplus_chg_ic_dev *ic_dev, int *volt)
{
	int rc = 0;
	char buf[OPLUS_GAUGE_THREE_LEVEL_TERM_VOLT_LEN] = { 0 };

	if (ic_dev == NULL || !volt)
		return -EINVAL;

	rc = oplus_get_three_level_term_volt(ic_dev, buf, OPLUS_GAUGE_THREE_LEVEL_TERM_VOLT_LEN);

	if (!rc)
		*volt = ((buf[1] << 8) + buf[0]);

	return rc;
}

static int oplus_set_three_level_term_volt(struct oplus_chg_ic_dev *ic_dev,
			char *args, int len)
{
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL)
		return -ENODEV;

	if (args == NULL)
		return -EINVAL;

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (!bcdev)
		return -EINVAL;

	if (len <= 0 || len > OPLUS_GAUGE_THREE_LEVEL_TERM_VOLT_LEN) {
		chg_err("len = %d not match\n", len);
		return -EINVAL;
	}

	return oplus_ap_write_buffer(bcdev,
		AP_MESSAGE_WRITE_THREE_LEVEL_TERM_VOLT, args, len);
}

static int oplus_set_batt_true_fcc(struct oplus_chg_ic_dev *ic_dev)
{
	int rc = 0;
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (!bcdev)
		return -ENODEV;

	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_SET_TRUE_FCC, true);
	else
		rc = write_property_id(bcdev, pst, BATT_SET_TRUE_FCC, true);
	chg_info(" rc=%d\n", rc);

	return rc;
}

static int oplus_get_batt_soc_centi(struct oplus_chg_ic_dev *ic_dev, int *soc_centi)
{
	struct battery_chg_dev *bcdev;
	int rc = 0;

	if (ic_dev == NULL || !soc_centi) {
		chg_err("oplus_chg_ic_dev is NULL");
		rc = -EINVAL;
		goto err;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (!bcdev) {
		rc = -ENODEV;
		goto err;
	}

	if (!bcdev->soccp_support) {
		rc = -ENOTSUPP;
		goto err;
	}

	rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_GET_SOC_CENTI);
	if (rc < 0) {
		chg_err("read battery soc centi fail, rc=%d\n", rc);
		goto err;
	}
	*soc_centi = bcdev->oplus_psy.prop[OPLUS_GET_SOC_CENTI];

	return 0;
err:
	*soc_centi = rc;
	return 0;
}

static void oplus_get_manu_battinfo_work(struct work_struct *work)
{
	int rc = 0;
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, get_manu_battinfo_work.work);

	if (!bcdev || !bcdev->ap_read_buffer_dump)
		return;

	mutex_lock(&bcdev->ap_read_buffer_lock);
	rc = ap_set_message_id(bcdev, AP_MESSAGE_GET_GAUGE_BATTINFO, 0);
	if (rc)
		goto err;

	reinit_completion(&bcdev->ap_read_ack[AP_MESSAGE_GET_GAUGE_BATTINFO]);
	rc = wait_for_completion_timeout(&bcdev->ap_read_ack[AP_MESSAGE_GET_GAUGE_BATTINFO],
					 msecs_to_jiffies(AP_READ_WAIT_TIME_MS));
	if (!rc) {
		chg_err("Error, timed out sending message\n");
		goto err;
	}

	memcpy(&bcdev->battinfo, bcdev->ap_read_buffer_dump->data_buffer,
			sizeof(struct battery_manufacture_info));
err:
	memset(bcdev->ap_read_buffer_dump, 0, sizeof(*bcdev->ap_read_buffer_dump));
	mutex_unlock(&bcdev->ap_read_buffer_lock);
}

static int oplus_get_battinfo_sn(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	struct battery_chg_dev *bcdev;
	int bsnlen = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (!bcdev || !buf || len < OPLUS_BATT_SERIAL_NUM_SIZE)
		return -EINVAL;

	chg_info("BattSN:%s", bcdev->battinfo.batt_serial_num);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
	bsnlen = strscpy(buf, bcdev->battinfo.batt_serial_num, OPLUS_BATT_SERIAL_NUM_SIZE);
#else
	bsnlen = strlcpy(buf, bcdev->battinfo.batt_serial_num, OPLUS_BATT_SERIAL_NUM_SIZE);
#endif

	return bsnlen;
}

static int oplus_set_read_mode(struct oplus_chg_ic_dev *ic_dev, int value)
{
	struct battery_chg_dev *bcdev;

        if (ic_dev == NULL) {
                chg_err("oplus_chg_ic_dev is NULL");
                return -ENODEV;
        }

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (!bcdev)
		return -EINVAL;

	bcdev->read_by_reg = value;
	return 0;
}

static int oplus_fg_get_batt_deep_dischg_count(struct oplus_chg_ic_dev *ic_dev, int *deep_dischg_count)
{
	int rc = 0;
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;

	if (ic_dev == NULL || !deep_dischg_count) {
		chg_err("oplus_chg_ic_dev or deep_dischg_count is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (!bcdev)
		return -ENODEV;

	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_DEEP_DISCHG_COUNT);
	else
		rc = read_property_id(bcdev, pst, BATT_DEEP_DISCHG_COUNT);
	if (rc < 0) {
		chg_err("get battery deep dischg count fail, rc=%d\n", rc);
		return false;
	}
	if (bcdev->soccp_support)
		*deep_dischg_count = bcdev->oplus_psy.prop[OPLUS_DEEP_DISCHG_COUNT];
	else
		*deep_dischg_count = pst->prop[BATT_DEEP_DISCHG_COUNT];
	chg_info("fg_get_batt_deep_dischg_count, deep_dischg_count=%d\n", *deep_dischg_count);

	return *deep_dischg_count;
}

static int oplus_fg_set_batt_deep_dischg_count(struct oplus_chg_ic_dev *ic_dev, int deep_dischg_count)
{
	int rc = 0;
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (!bcdev)
		return -ENODEV;

	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_DEEP_DISCHG_COUNT, deep_dischg_count);
	else
		rc = write_property_id(bcdev, pst, BATT_DEEP_DISCHG_COUNT, deep_dischg_count);
	chg_info("fg_set_batt_deep_dischg_count rc=%d, count = %d\n", rc, deep_dischg_count);
	return rc;
}

static int oplus_fg_set_deep_term_volt(struct oplus_chg_ic_dev *ic_dev, int deep_term_volt)
{
	int rc = 0;
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (!bcdev)
		return -ENODEV;

	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_DEEP_TERM_VOLT, deep_term_volt);
	else
		rc = write_property_id(bcdev, pst, BATT_DEEP_TERM_VOLT, deep_term_volt);
	chg_info("oplus_set_deep_term_volt rc=%d, volt = %d\n", rc, deep_term_volt);
	return rc;
}

static int oplus_fg_get_deep_term_volt(struct oplus_chg_ic_dev *ic_dev, int *volt)
{
	int rc = 0;
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;

	if (ic_dev == NULL || !volt) {
		chg_err("oplus_chg_ic_dev or volt is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (!bcdev)
		return -ENODEV;

	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_DEEP_TERM_VOLT);
	else
		rc = read_property_id(bcdev, pst, BATT_DEEP_TERM_VOLT);
	if (rc < 0) {
		chg_err("get battery deep term volt fail, rc=%d\n", rc);
		return false;
	}
	if (bcdev->soccp_support)
		*volt = bcdev->oplus_psy.prop[OPLUS_DEEP_TERM_VOLT];
	else
		*volt = pst->prop[BATT_DEEP_TERM_VOLT];
	chg_info("oplus_fg_get_deep_term_volt, deep_term_volt=%d\n", *volt);

	return *volt;
}

static int oplus_get_manu_date(struct oplus_chg_ic_dev *ic_dev, char *buf, int len)
{
	struct battery_chg_dev *bcdev;
	int date_len = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (!bcdev || !buf || len < OPLUS_BATTINFO_DATE_SIZE)
		return -EINVAL;

	chg_info("BattManuDate:0x%04x", bcdev->battinfo.manu_date);
	date_len = snprintf(buf, len, "%d-%02d-%02d", (((bcdev->battinfo.manu_date >> 9) & 0x7F) + 1980),
			(bcdev->battinfo.manu_date >> 5) & 0xF, bcdev->battinfo.manu_date & 0x1F);

	return date_len;
}

static int oplus_get_first_usage_date(struct oplus_chg_ic_dev *ic_dev, char *buf, int len)
{
	struct battery_chg_dev *bcdev;
	int date_len = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (!bcdev || !buf || len < OPLUS_BATTINFO_DATE_SIZE)
		return -EINVAL;

	chg_info("BattFirstUsage:0x%04x", bcdev->battinfo.first_usage_date);

	date_len = snprintf(buf, len, "%d-%02d-%02d", (((bcdev->battinfo.first_usage_date >> 9) & 0x7F) + 1980),
			(bcdev->battinfo.first_usage_date >> 5) & 0xF, bcdev->battinfo.first_usage_date & 0x1F);

	return date_len;
}

static int oplus_set_first_usage_date(struct oplus_chg_ic_dev *ic_dev, const char *buf)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst;
	u32 data = 0x00;
	u16 date = 0x00;
	u8 check_sum = 0x00;
	int year = 0;
	int month = 0;
	int day = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (!bcdev || !buf)
		return -EINVAL;

	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	sscanf(buf, "%d-%d-%d", &year, &month, &day);
	date = (((year - 1980) & 0x7F) << 9) | ((month & 0xF) << 5) | (day & 0x1F);
	check_sum = 0xFF - ((date >> 8) & 0xFF) - (date & 0xFF);
	data = date << 8 | check_sum;
	chg_info("%d-%d-%d", year, month, day);
	chg_info("date=0x%04x, data=0x%08x", date, data);
	if (bcdev->soccp_support)
		write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_SET_FIRST_USAGE_DATE, data);
	else
		write_property_id(bcdev, pst, BATT_SET_FIRST_USAGE_DATE, data);

	cancel_delayed_work_sync(&bcdev->get_manu_battinfo_work);
	schedule_delayed_work(&bcdev->get_manu_battinfo_work, OPLUS_GET_BATT_INFO_FROM_ADSP_INTERVAL);

	return 0;
}

static int oplus_get_ui_cycle_count(struct oplus_chg_ic_dev *ic_dev, u16 *ui_cycle_count)
{
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (!bcdev || !ui_cycle_count)
		return -EINVAL;

	chg_info("BattUICyclecount:%d", bcdev->battinfo.ui_cycle_count);
	*ui_cycle_count = bcdev->battinfo.ui_cycle_count;
	return 0;
}

static int oplus_set_ui_cycle_count(struct oplus_chg_ic_dev *ic_dev, u16 ui_cycle_count)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst;
	u8 check_sum = 0x00;
	u32 data = 0x00;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (!bcdev)
		return -EINVAL;

	check_sum = 0xFF - ((ui_cycle_count >> 8) & 0xFF) - (ui_cycle_count & 0xFF);
	data = ui_cycle_count << 8 | check_sum;
	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	if (bcdev->soccp_support)
		write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_SET_UI_CYCLE_COUNT, data);
	else
		write_property_id(bcdev, pst, BATT_SET_UI_CYCLE_COUNT, data);

	cancel_delayed_work_sync(&bcdev->get_manu_battinfo_work);
	schedule_delayed_work(&bcdev->get_manu_battinfo_work, OPLUS_GET_BATT_INFO_FROM_ADSP_INTERVAL);

	return 0;
}

static int oplus_get_ui_soh(struct oplus_chg_ic_dev *ic_dev, u8 *ui_soh)
{
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (!bcdev || !ui_soh)
		return -EINVAL;

	chg_info("BattUISoh:%d", bcdev->battinfo.ui_soh);
	*ui_soh = bcdev->battinfo.ui_soh;

	return 0;
}

static int oplus_set_ui_soh(struct oplus_chg_ic_dev *ic_dev, u8 ui_soh)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst;
	u8 check_sum = 0x00;
	u32 data = 0x00;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (!bcdev)
		return -EINVAL;

	check_sum = 0xFF - (ui_soh & 0xFF);
	data = ui_soh << 8 | check_sum;
	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	if (bcdev->soccp_support)
		write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_SET_UI_SOH, data);
	else
		write_property_id(bcdev, pst, BATT_SET_UI_SOH, data);

	cancel_delayed_work_sync(&bcdev->get_manu_battinfo_work);
	schedule_delayed_work(&bcdev->get_manu_battinfo_work, OPLUS_GET_BATT_INFO_FROM_ADSP_INTERVAL);

	return 0;
}

static int oplus_get_used_flag(struct oplus_chg_ic_dev *ic_dev, u8 *used_flag)
{
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (!bcdev || !used_flag)
		return -EINVAL;

	*used_flag = bcdev->battinfo.used_flag;

	return 0;
}

static int oplus_set_used_flag(struct oplus_chg_ic_dev *ic_dev, u8 used_flag)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst;
	u8 check_sum = 0x00;
	u32 data = 0x00;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (!bcdev)
		return -EINVAL;

	check_sum = 0xFF - (used_flag & 0xFF);
	data = used_flag << 8 | check_sum;
	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	if (bcdev->soccp_support)
		write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_SET_USED_FLAG, data);
	else
		write_property_id(bcdev, pst, BATT_SET_USED_FLAG, data);

	cancel_delayed_work_sync(&bcdev->get_manu_battinfo_work);
	schedule_delayed_work(&bcdev->get_manu_battinfo_work, OPLUS_GET_BATT_INFO_FROM_ADSP_INTERVAL);
	return 0;
}

static int oplus_fg_get_last_cc(struct oplus_chg_ic_dev *ic_dev, int *cc)
{
	int rc = 0;
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;

	if (ic_dev == NULL || !cc) {
		chg_err("oplus_chg_ic_dev or deep_dischg_count is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (!bcdev)
		return -ENODEV;

	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_LAST_CC);
	else
		rc = read_property_id(bcdev, pst, BATT_DEEP_DISCHG_LAST_CC);
	if (rc < 0) {
		chg_err("get battery deep dischg count fail, rc=%d\n", rc);
		return false;
	}
	if (bcdev->soccp_support)
		*cc = bcdev->oplus_psy.prop[OPLUS_LAST_CC];
	else
		*cc = pst->prop[BATT_DEEP_DISCHG_LAST_CC];
	chg_info(" cc=%d\n", *cc);

	return *cc;
}

static int oplus_fg_set_last_cc(struct oplus_chg_ic_dev *ic_dev, int cc)
{
	int rc = 0;
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (!bcdev)
		return -ENODEV;

	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_LAST_CC, cc);
	else
		rc = write_property_id(bcdev, pst, BATT_DEEP_DISCHG_LAST_CC, cc);
	chg_info(" rc=%d, cc = %d\n", rc, cc);
	return rc;
}

static int oplus_fg_get_dec_fg_type(struct oplus_chg_ic_dev *ic_dev, int *fg_type)
{
	if (ic_dev == NULL || !fg_type) {
		chg_err("oplus_chg_ic_dev or deep_dischg_count is NULL");
		return -ENODEV;
	}
/* TODO DEC_CV_PACK_SOH/DEC_CV_MB_TI */

	*fg_type = DEC_CV_QCOM_FG;

	return 0;
}

static int oplus_fg_get_dec_cv_soh(struct oplus_chg_ic_dev *ic_dev, int *dec_soh)
{
	if (ic_dev == NULL || !dec_soh) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	*dec_soh = fg_sm8350_get_battery_cc();

	return 0;
}

static int fg_get_battery_vct(struct battery_chg_dev *bcdev)
{
	int rc = 0;
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int vct = 0;

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_GET_VCT);
	else
		rc = read_property_id(bcdev, pst, BATT_GET_VCT);
	if (rc < 0) {
		chg_err("read battery vct fail, rc=%d\n", rc);
		return 0;
	}

	if (bcdev->soccp_support)
		vct = bcdev->oplus_psy.prop[OPLUS_GET_VCT];
	else
		vct = pst->prop[BATT_GET_VCT];

	chg_debug("read battery vct success, vct = %d\n", vct);

	return vct;
}

static int oplus_fg_get_vct(struct oplus_chg_ic_dev *ic_dev, int *vct)
{
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL || !vct) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (!bcdev)
		return -ENODEV;

	*vct = fg_get_battery_vct(bcdev);

	return 0;
}

static int oplus_fg_set_vct(struct oplus_chg_ic_dev *ic_dev, int vct)
{
	int rc = 0;
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (!bcdev)
		return -ENODEV;

	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_SET_VCT, vct);
	else
		rc = write_property_id(bcdev, pst, BATT_SET_VCT, vct);
	chg_debug(" rc=%d vct = %d\n", rc, vct);

	return rc;
}


static int fg_get_battery_car_c(struct battery_chg_dev *bcdev)
{
	int rc = 0;
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_GET_CAR_C);
	else
		rc = read_property_id(bcdev, pst, BATT_GET_CAR_C);
	if (rc < 0) {
		chg_err("read battery car_c fail, rc=%d\n", rc);
		return 0;
	}

	if (bcdev->soccp_support) {
		chg_debug("read battery car_c success, car_c = %d\n", bcdev->oplus_psy.prop[OPLUS_GET_CAR_C]);
		return bcdev->oplus_psy.prop[OPLUS_GET_CAR_C];
	} else {
		chg_debug("read battery car_c success, car_c = %d\n", pst->prop[BATT_GET_CAR_C]);
		return pst->prop[BATT_GET_CAR_C];
	}
}

static int oplus_fg_get_car_c(struct oplus_chg_ic_dev *ic_dev, int *car_c)
{
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL || !car_c) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (!bcdev)
		return -ENODEV;

	*car_c = fg_get_battery_car_c(bcdev);

	return 0;
}

static int oplus_fg_set_cuv_state(struct oplus_chg_ic_dev *ic_dev, int cuv_state)
{
	int rc = 0;
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (!bcdev)
		return -ENODEV;

	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_SET_CUV_STATE, cuv_state);
	else
		rc = write_property_id(bcdev, pst, BATT_SET_CUV_STATE, cuv_state);
	chg_info(" rc=%d, *cuv_state= %d \n", rc, cuv_state);

	return rc;
}

static int fg_get_battery_cuv_state(struct battery_chg_dev *bcdev)
{
	int rc = 0;
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int cuv_state;

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_GET_CUV_STATE);
	else
		rc = read_property_id(bcdev, pst, BATT_GET_CUV_STATE);

	if (rc < 0) {
		chg_err("read battery cuv_state fail, rc=%d\n", rc);
		return 0;
	}
	if (bcdev->soccp_support)
		cuv_state = bcdev->oplus_psy.prop[OPLUS_GET_CUV_STATE];
	else
		cuv_state = pst->prop[BATT_GET_CUV_STATE];

	return cuv_state;
}

static int oplus_fg_get_cuv_state(struct oplus_chg_ic_dev *ic_dev, int *cuv_state)
{
	struct battery_chg_dev *bcdev;
	int rc = 0;

	if (ic_dev == NULL || !cuv_state) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (!bcdev)
		return -ENODEV;

	*cuv_state = fg_get_battery_cuv_state(bcdev);
	chg_debug(" rc = %d, *cuv_state = %d \n", rc, *cuv_state);

	return rc;
}

static int oplus_fg_get_sili_lifetime_status(
	struct oplus_chg_ic_dev *ic_dev, struct oplus_gauge_lifetime *lifetime_status)
{
	struct battery_chg_dev *bcdev;
	int rc = 0;

	if (ic_dev == NULL)
		return -ENODEV;

	if (lifetime_status == NULL)
		return -EINVAL;

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (!bcdev || !bcdev->ap_read_buffer_dump)
		return -ENODEV;

	mutex_lock(&bcdev->ap_read_buffer_lock);
	rc = ap_set_message_id(bcdev, AP_MESSAGE_GET_GAUGE_LIFETIME_STATUS, 0);
	if (rc)
		goto err;

	reinit_completion(&bcdev->ap_read_ack[AP_MESSAGE_GET_GAUGE_LIFETIME_STATUS]);
	rc = wait_for_completion_timeout(&bcdev->ap_read_ack[AP_MESSAGE_GET_GAUGE_LIFETIME_STATUS],
						 msecs_to_jiffies(AP_READ_WAIT_TIME_MS));
	if (!rc) {
		chg_err("Error, timed out sending message\n");
		goto err;
	}

	if (bcdev->ap_read_buffer_dump->data_size != sizeof(struct oplus_gauge_lifetime)) {
		chg_err("Error, len not match\n");
		goto err;
	}

	memcpy(lifetime_status, bcdev->ap_read_buffer_dump->data_buffer, sizeof(struct oplus_gauge_lifetime));
	memset(bcdev->ap_read_buffer_dump, 0, sizeof(*bcdev->ap_read_buffer_dump));
	mutex_unlock(&bcdev->ap_read_buffer_lock);
	return 0;

err:
	memset(bcdev->ap_read_buffer_dump, 0, sizeof(*bcdev->ap_read_buffer_dump));
	mutex_unlock(&bcdev->ap_read_buffer_lock);
	return -EINVAL;
}

static int oplus_fg_get_sili_lifetime_info(struct oplus_chg_ic_dev *ic_dev, u8 *info, int len)
{
	struct battery_chg_dev *bcdev;
	int index = 0;
	int rc = 0;

	if (ic_dev == NULL || !info) {
		chg_err("oplus_chg_ic_dev or info is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (!bcdev || !bcdev->ap_read_buffer_dump)
		return -ENODEV;

	mutex_lock(&bcdev->ap_read_buffer_lock);
	rc = ap_set_message_id(bcdev, AP_MESSAGE_GET_GAUGE_LIFETIME_INFO, 0);
	if (rc)
		goto err;

	reinit_completion(&bcdev->ap_read_ack[AP_MESSAGE_GET_GAUGE_LIFETIME_INFO]);
	rc = wait_for_completion_timeout(&bcdev->ap_read_ack[AP_MESSAGE_GET_GAUGE_LIFETIME_INFO],
					 msecs_to_jiffies(AP_READ_WAIT_TIME_MS));
	if (!rc) {
		chg_err("Error, timed out sending message\n");
		goto err;
	}

	index = bcdev->ap_read_buffer_dump->data_size;
	if (index >= len) {
		chg_err("Error, len not match\n");
		goto err;
	}

	memcpy(info, bcdev->ap_read_buffer_dump->data_buffer, index);
	memset(bcdev->ap_read_buffer_dump, 0, sizeof(*bcdev->ap_read_buffer_dump));
	mutex_unlock(&bcdev->ap_read_buffer_lock);
	return index;
err:
	memset(bcdev->ap_read_buffer_dump, 0, sizeof(*bcdev->ap_read_buffer_dump));
	mutex_unlock(&bcdev->ap_read_buffer_lock);
	return -EINVAL;
}

static int adsp_publish_ic_err_msg(struct battery_chg_dev *bcdev,
	char *ic_name, int type, int sub_type, char *msg)
{
	int rc;
	struct mms_msg *topic_msg;

	if (!is_err_topic_available(bcdev)) {
		chg_err("error topic not found\n");
		return -EINVAL;
	}

	topic_msg =
		oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, ERR_ITEM_IC,
					"[%s]-[%d]-[%d]:%s", ic_name, type, sub_type, msg);
	if (topic_msg == NULL) {
		chg_err("alloc topic msg error\n");
		return -ENOMEM;
	}

	rc = oplus_mms_publish_msg_sync(bcdev->err_topic, topic_msg);
	if (rc < 0) {
		chg_err("publish error topic msg error, rc=%d\n", rc);
		kfree(topic_msg);
	}

	return rc;
}

static int oplus_fg_get_ra0_status(struct battery_chg_dev *bcdev, u8 *info, int len)
{
	int index = 0;
	int rc = 0;

	if (!bcdev || !bcdev->ap_read_buffer_dump || !info)
		return -EINVAL;

	mutex_lock(&bcdev->ap_read_buffer_lock);
	rc = ap_set_message_id(bcdev, AP_MESSAGE_GET_GAUGE_RA0_INFO, 0);
	if (rc)
		goto err;

	reinit_completion(&bcdev->ap_read_ack[AP_MESSAGE_GET_GAUGE_RA0_INFO]);
	rc = wait_for_completion_timeout(&bcdev->ap_read_ack[AP_MESSAGE_GET_GAUGE_RA0_INFO],
					 msecs_to_jiffies(BC_WAIT_TIME_MS));
	if (!rc) {
		chg_err("Error, timed out sending message\n");
		goto err;
	}

	index = bcdev->ap_read_buffer_dump->data_size;
	if (index >= len) {
		chg_err("Error, len not match\n");
		goto err;
	}

	memmove(info, bcdev->ap_read_buffer_dump->data_buffer, index);
	memset(bcdev->ap_read_buffer_dump, 0, sizeof(*bcdev->ap_read_buffer_dump));
	mutex_unlock(&bcdev->ap_read_buffer_lock);
	return index;
err:
	memset(bcdev->ap_read_buffer_dump, 0, sizeof(*bcdev->ap_read_buffer_dump));
	mutex_unlock(&bcdev->ap_read_buffer_lock);
	return -EINVAL;
}

static int oplus_fg_get_imp_status(struct battery_chg_dev *bcdev, u8 *info, int len)
{
	int index = 0;
	int rc = 0;

	if (!bcdev || !bcdev->ap_read_buffer_dump || !info)
		return -EINVAL;

	mutex_lock(&bcdev->ap_read_buffer_lock);
	rc = ap_set_message_id(bcdev, AP_MESSAGE_GET_GAUGE_IMP_INFO, 0);
	if (rc)
		goto err;

	reinit_completion(&bcdev->ap_read_ack[AP_MESSAGE_GET_GAUGE_IMP_INFO]);
	rc = wait_for_completion_timeout(&bcdev->ap_read_ack[AP_MESSAGE_GET_GAUGE_IMP_INFO],
					 msecs_to_jiffies(BC_WAIT_TIME_MS));
	if (!rc) {
		chg_err("Error, timed out sending message\n");
		goto err;
	}

	index = bcdev->ap_read_buffer_dump->data_size;
	if (index >= len) {
		chg_err("Error, len not match\n");
		goto err;
	}

	memmove(info, bcdev->ap_read_buffer_dump->data_buffer, index);
	memset(bcdev->ap_read_buffer_dump, 0, sizeof(*bcdev->ap_read_buffer_dump));
	mutex_unlock(&bcdev->ap_read_buffer_lock);
	return index;
err:
	memset(bcdev->ap_read_buffer_dump, 0, sizeof(*bcdev->ap_read_buffer_dump));
	mutex_unlock(&bcdev->ap_read_buffer_lock);
	return -EINVAL;
}

static int oplus_fg_get_delta_voltage_status(struct battery_chg_dev *bcdev, u8 *info, int len)
{
	int index = 0;
	int rc = 0;

	if (!bcdev || !bcdev->ap_read_buffer_dump || !info)
		return -EINVAL;

	mutex_lock(&bcdev->ap_read_buffer_lock);
	rc = ap_set_message_id(bcdev, AP_MESSAGE_GET_GAUGE_DELTA_VOLTAGE_INFO, 0);
	if (rc)
		goto err;

	reinit_completion(&bcdev->ap_read_ack[AP_MESSAGE_GET_GAUGE_DELTA_VOLTAGE_INFO]);
	rc = wait_for_completion_timeout(&bcdev->ap_read_ack[AP_MESSAGE_GET_GAUGE_DELTA_VOLTAGE_INFO],
					 msecs_to_jiffies(AP_READ_WAIT_TIME_MS));
	if (!rc) {
		chg_err("Error, timed out sending message\n");
		goto err;
	}

	index = bcdev->ap_read_buffer_dump->data_size;
	if (index >= len) {
		chg_err("Error, len not match\n");
		goto err;
	}

	memmove(info, bcdev->ap_read_buffer_dump->data_buffer, index);
	memset(bcdev->ap_read_buffer_dump, 0, sizeof(*bcdev->ap_read_buffer_dump));
	mutex_unlock(&bcdev->ap_read_buffer_lock);
	return index;
err:
	memset(bcdev->ap_read_buffer_dump, 0, sizeof(*bcdev->ap_read_buffer_dump));
	mutex_unlock(&bcdev->ap_read_buffer_lock);
	return -EINVAL;
}

#define ADSP_GAUGE_NAME_LEN	32
static void oplus_gauge_ra0_check_work(struct work_struct *work)
{
	int rc;
	u8 *ra0_data;
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, gauge_ra0_check_work);
	char name[ADSP_GAUGE_NAME_LEN] = "adsp_gauge";

	ra0_data = (u8 *)kzalloc(MAX_AP_PROPERTY_DATA_SIZE, GFP_KERNEL);
	if (ra0_data == NULL) {
		chg_err("gauge ra0 data buf kzalloc error\n");
		return;
	}

	rc = oplus_fg_get_ra0_status(bcdev, ra0_data, MAX_AP_PROPERTY_DATA_SIZE - 1);
	if (rc > 0 && strstr(ra0_data, "err_scene")) {
		if (is_gauge_topic_available(bcdev)) {
			memset(name, 0, sizeof(name));
			oplus_gauge_get_physical_name(bcdev->gauge_topic, name, ADSP_GAUGE_NAME_LEN - 1);
		}
		adsp_publish_ic_err_msg(bcdev, name, OPLUS_IC_ERR_GAUGE,
			TRACK_GAGUE_FCC_RA0_ERR_INFO, ra0_data);
	}

	kfree(ra0_data);
}

static void oplus_gauge_imp_check_work(struct work_struct *work)
{
	u8 *imp_data;
	int rc;
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, gauge_imp_check_work);
	char name[ADSP_GAUGE_NAME_LEN] = "adsp_gauge";

	imp_data = (u8 *)kzalloc(MAX_AP_PROPERTY_DATA_SIZE, GFP_KERNEL);
	if (imp_data == NULL) {
		chg_err("gauge imp data buf kzalloc error\n");
		return;
	}

	rc = oplus_fg_get_imp_status(bcdev, imp_data, MAX_AP_PROPERTY_DATA_SIZE - 1);
	if (rc > 0 && strstr(imp_data, "err_scene")) {
		if (is_gauge_topic_available(bcdev)) {
			memset(name, 0, sizeof(name));
			oplus_gauge_get_physical_name(bcdev->gauge_topic, name, ADSP_GAUGE_NAME_LEN - 1);
		}
		adsp_publish_ic_err_msg(bcdev, name, OPLUS_IC_ERR_GAUGE,
			TRACK_GAGUE_FCC_RA_T_ERR_INFO, imp_data);
	}

	kfree(imp_data);
}

static void oplus_gauge_delta_voltage_check_work(struct work_struct *work)
{
	u8 *delta_voltage_data;
	int rc;
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, gauge_delta_voltage_check_work);
	char name[ADSP_GAUGE_NAME_LEN] = "adsp_gauge";

	delta_voltage_data = (u8 *)kzalloc(MAX_AP_PROPERTY_DATA_SIZE, GFP_KERNEL);
	if (delta_voltage_data == NULL) {
		chg_err("gauge delta voltage data buf kzalloc error\n");
		return;
	}

	rc = oplus_fg_get_delta_voltage_status(bcdev, delta_voltage_data, MAX_AP_PROPERTY_DATA_SIZE - 1);
	if (rc > 0 && strstr(delta_voltage_data, "err_scene")) {
		if (is_gauge_topic_available(bcdev)) {
			memset(name, 0, sizeof(name));
			oplus_gauge_get_physical_name(bcdev->gauge_topic, name, ADSP_GAUGE_NAME_LEN - 1);
		}
		adsp_publish_ic_err_msg(bcdev, name, OPLUS_IC_ERR_GAUGE,
			TRACK_GAGUE_FCC_VDELTA_ERR_INFO, delta_voltage_data);
	}

	kfree(delta_voltage_data);
}

static int oplus_gauge_ra0_check(struct oplus_chg_ic_dev *ic_dev)
{
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (!bcdev)
		return -ENODEV;

	if (work_busy(&bcdev->gauge_ra0_check_work) & (WORK_BUSY_RUNNING | WORK_BUSY_PENDING))
		return 0;

	schedule_work(&bcdev->gauge_ra0_check_work);

	return 0;
}

static int oplus_gauge_imp_check(struct oplus_chg_ic_dev *ic_dev)
{
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (!bcdev)
		return -ENODEV;

	if (work_busy(&bcdev->gauge_imp_check_work) & (WORK_BUSY_RUNNING | WORK_BUSY_PENDING))
		return 0;

	schedule_work(&bcdev->gauge_imp_check_work);

	return 0;
}

static int oplus_gauge_delta_voltage_check(struct oplus_chg_ic_dev *ic_dev, bool cv)
{
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (!bcdev)
		return -ENODEV;

	if (work_busy(&bcdev->gauge_delta_voltage_check_work) & (WORK_BUSY_RUNNING | WORK_BUSY_PENDING))
		return 0;

	schedule_work(&bcdev->gauge_delta_voltage_check_work);

	return 0;
}

static void *oplus_chg_8350_gauge_get_func(struct oplus_chg_ic_dev *ic_dev,
					   enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT,
					       oplus_sm8350_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT,
					       oplus_sm8350_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP,
					       oplus_sm8350_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST,
					       oplus_sm8350_smt_test);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_VOL,
					       oplus_sm8350_get_batt_vol);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_MAX:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_MAX,
					       oplus_sm8350_get_batt_max);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_MIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_MIN,
					       oplus_sm8350_get_batt_min);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_CURR:
		func = OPLUS_CHG_IC_FUNC_CHECK(
			OPLUS_IC_FUNC_GAUGE_GET_BATT_CURR,
			oplus_sm8350_get_batt_curr);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_REAL_TIME_CURR:
		func = OPLUS_CHG_IC_FUNC_CHECK(
			OPLUS_IC_FUNC_GAUGE_GET_REAL_TIME_CURR,
			oplus_sm8350_get_real_time_current);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_TEMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(
			OPLUS_IC_FUNC_GAUGE_GET_BATT_TEMP,
			oplus_sm8350_get_batt_temp);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC,
					       oplus_sm8350_get_batt_soc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_FCC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_FCC,
					       oplus_sm8350_get_batt_fcc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_CC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_CC,
					       oplus_sm8350_get_batt_cc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_RM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_RM,
					       oplus_sm8350_get_batt_rm);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_SOH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_SOH,
					       oplus_sm8350_get_batt_soh);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_AUTH:
		func = OPLUS_CHG_IC_FUNC_CHECK(
			OPLUS_IC_FUNC_GAUGE_GET_BATT_AUTH,
			oplus_sm8350_get_batt_auth);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_HMAC:
		func = OPLUS_CHG_IC_FUNC_CHECK(
			OPLUS_IC_FUNC_GAUGE_GET_BATT_HMAC,
			oplus_sm8350_get_batt_hmac);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_BATT_FULL:
		func = OPLUS_CHG_IC_FUNC_CHECK(
			OPLUS_IC_FUNC_GAUGE_SET_BATT_FULL,
			oplus_sm8350_set_batt_full);
		break;
	case OPLUS_IC_FUNC_GAUGE_UPDATE_DOD0:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_UPDATE_DOD0,
					       oplus_sm8350_update_dod0);
		break;
	case OPLUS_IC_FUNC_GAUGE_UPDATE_SOC_SMOOTH:
		func = OPLUS_CHG_IC_FUNC_CHECK(
			OPLUS_IC_FUNC_GAUGE_UPDATE_SOC_SMOOTH,
			oplus_sm8350_update_soc_smooth_parameter);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_CB_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(
			OPLUS_IC_FUNC_GAUGE_GET_CB_STATUS,
			oplus_sm8350_get_cb_status);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_LOCK:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_LOCK,
					       oplus_sm8350_set_lock);
		break;
	case OPLUS_IC_FUNC_GAUGE_IS_LOCKED:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_IS_LOCKED,
					       oplus_sm8350_is_locked);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_NUM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_NUM,
					       oplus_sm8350_get_batt_num);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE:
		func = OPLUS_CHG_IC_FUNC_CHECK(
			OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE,
			oplus_sm8350_get_device_type);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE_FOR_VOOC:
		func = OPLUS_CHG_IC_FUNC_CHECK(
			OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE_FOR_VOOC,
			oplus_sm8350_get_device_type_for_vooc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BCC_PARMS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BCC_PARMS,
			oplus_get_bcc_parameters_from_adsp);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_BCC_PARMS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_BCC_PARMS,
			oplus_set_bcc_debug_parameters);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DOD0:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DOD0,
			oplus_sm8350_get_battery_dod0);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DOD0_PASSED_Q:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DOD0_PASSED_Q,
			oplus_sm8350_get_battery_dod0_passed_q);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_QMAX:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_QMAX,
			oplus_sm8350_get_battery_qmax);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_QMAX_PASSED_Q:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_QMAX_PASSED_Q,
			oplus_sm8350_get_battery_qmax_passed_q);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE_FOR_BCC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE_FOR_BCC,
			oplus_sm8350_get_battery_gauge_type_for_bcc);
		break;
	case OPLUS_IC_FUNC_GAUGE_UPDATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_UPDATE,
			oplus_chg_8350_get_props_from_adsp_by_buffer);
		break;
	case OPLUS_IC_FUNC_GAUGE_IS_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_IS_SUSPEND,
			oplus_chg_8350_gauge_is_suspend);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_SUBBOARD_TEMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_SUBBOARD_TEMP,
					       oplus_get_subboard_temp);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_REG_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_REG_INFO,
					      oplus_sm8350_get_reg_info);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_CALIB_TIME:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_CALIB_TIME,
					      oplus_sm8350_get_calib_time);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_CALIB_TIME:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_CALIB_TIME,
					      oplus_sm8350_set_calib_time);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_SN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_SN,
					       oplus_get_battinfo_sn);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_READ_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_READ_MODE,
			oplus_set_read_mode);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEEP_DISCHG_COUNT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DEEP_DISCHG_COUNT,
						  oplus_fg_get_batt_deep_dischg_count);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_DEEP_DISCHG_COUNT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_DEEP_DISCHG_COUNT,
						  oplus_fg_set_batt_deep_dischg_count);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_DEEP_TERM_VOLT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_DEEP_TERM_VOLT,
						  oplus_fg_set_deep_term_volt);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEEP_TERM_VOLT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DEEP_TERM_VOLT,
					      oplus_fg_get_deep_term_volt);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_MANU_DATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_MANU_DATE,
					       oplus_get_manu_date);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_FIRST_USAGE_DATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_FIRST_USAGE_DATE,
					       oplus_get_first_usage_date);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_FIRST_USAGE_DATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_FIRST_USAGE_DATE,
					       oplus_set_first_usage_date);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_UI_CC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_UI_CC,
					       oplus_get_ui_cycle_count);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_UI_CC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_UI_CC,
					       oplus_set_ui_cycle_count);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_UI_SOH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_UI_SOH,
					       oplus_get_ui_soh);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_UI_SOH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_UI_SOH,
					       oplus_set_ui_soh);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_USED_FLAG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_USED_FLAG,
					       oplus_get_used_flag);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_USED_FLAG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_USED_FLAG,
					      oplus_set_used_flag);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_LAST_CC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DEEP_DISCHG_COUNT,
						  oplus_fg_get_last_cc);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_LAST_CC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_DEEP_DISCHG_COUNT,
						  oplus_fg_set_last_cc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_SUB_BTB_CONNECT_STATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_SUB_BTB_CONNECT_STATE,
						  oplus_gauge_get_sub_btb_state);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEC_FG_TYPE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DEC_FG_TYPE,
						   oplus_fg_get_dec_fg_type);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEC_CV_SOH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DEC_CV_SOH,
						   oplus_fg_get_dec_cv_soh);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_VCT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_VCT,
					       oplus_fg_set_vct);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_VCT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_VCT,
					       oplus_fg_get_vct);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_GAUGE_CAR_C:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_GAUGE_CAR_C,
						oplus_fg_get_car_c);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_CUV_STATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_CUV_STATE,
					       oplus_fg_set_cuv_state);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_CUV_STATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_CUV_STATE,
					       oplus_fg_get_cuv_state);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_SILI_LIFETIME_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_SILI_LIFETIME_STATUS,
					       oplus_fg_get_sili_lifetime_status);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_SILI_LIFETIME_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_SILI_LIFETIME_INFO,
					       oplus_fg_get_sili_lifetime_info);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_GAUGE_R_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_GAUGE_R_INFO,
					       oplus_sm8350_get_gauge_r_info);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_THREE_LEVEL_TERM_VOLT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_THREE_LEVEL_TERM_VOLT,
						oplus_get_three_level_term_volt);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_THREE_LEVEL_TERM_VOLT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_THREE_LEVEL_TERM_VOLT,
						oplus_set_three_level_term_volt);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC_CENTI:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC_CENTI,
			oplus_get_batt_soc_centi);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_FCL_VOLT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_FCL_VOLT,
					       oplus_sm8350_get_vbatt_fcl);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_TRUE_FCC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_TRUE_FCC,
			oplus_set_batt_true_fcc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_SILI_SIMULATE_TERM_VOLT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_SILI_SIMULATE_TERM_VOLT,
						oplus_get_three_level_term_volt0);
		break;
	case OPLUS_IC_FUNC_GAUGE_FCC_VDELTA_CHECK:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_FCC_VDELTA_CHECK,
			oplus_gauge_delta_voltage_check);
		break;
	case OPLUS_IC_FUNC_GAUGE_FFC_T_RA_CHECK:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_FFC_T_RA_CHECK,
			oplus_gauge_imp_check);
		break;
	case OPLUS_IC_FUNC_GAUGE_FFC_RA0_CHECK:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_FFC_RA0_CHECK,
			oplus_gauge_ra0_check);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq oplus_chg_8350_gauge_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_HMAC_UPDATE },
};

static int oplus_chg_adsp_cp_init(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	ic_dev->online = true;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_ONLINE);

	return 0;
}

static int oplus_chg_adsp_cp_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	ic_dev->online = false;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_OFFLINE);

	return 0;
}

static int oplus_chg_adsp_cp_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	struct battery_chg_dev *chip;
	unsigned char *data_buf = NULL;
	int rc = 0;

#define REG_DUMP_START 0x2560
#define REG_DUMP_END   0x2569
#define REG_DUMP_SIZE  (REG_DUMP_END - REG_DUMP_START + 1)

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (chip->regmap == NULL) {
		chg_err("pm8550b regmap not found\n");
		return -ENODEV;
	}

	data_buf = kzalloc(REG_DUMP_SIZE, GFP_KERNEL);
	if (data_buf == NULL) {
		chg_err("memory allocation failed\n");
		return -ENOMEM;
	}

	rc = regmap_bulk_read(chip->regmap, REG_DUMP_START, data_buf, REG_DUMP_SIZE);
	if (rc < 0) {
		pr_err("pm8550b read reg error, addr=0x%04x, len=%d, rc=%d\n",
			REG_DUMP_START, REG_DUMP_SIZE, rc);
		goto out;
	}

	print_hex_dump(KERN_ERR, "OPLUS_CHG[PM8550B]: ", DUMP_PREFIX_OFFSET,
			32, 1, data_buf, REG_DUMP_SIZE, false);

out:
	kfree(data_buf);
	return rc;
}

static int oplus_chg_adsp_cp_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	return 0;
}

static int oplus_chg_adsp_cp_get_vin(struct oplus_chg_ic_dev *ic_dev, int *vin)
{
	struct battery_chg_dev *bcdev;
	int prop_id = 0;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_USB];

	prop_id = get_property_id(pst, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0) {
		chg_err("read usb vbus_volt fail, rc=%d\n", rc);
		return rc;
	}
	*vin = pst->prop[prop_id] / 1000;

	return rc;
}

static int oplus_chg_adsp_cp_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	return 0;
}

static int oplus_chg_adsp_cp_hw_init(struct oplus_chg_ic_dev *ic_dev)
{
	return 0;
}

static int oplus_chg_adsp_cp_set_work_mode(struct oplus_chg_ic_dev *ic_dev, enum oplus_cp_work_mode mode)
{
	if (mode != CP_WORK_MODE_BYPASS)
		return -ENOTSUPP;

	return 0;
}

static int oplus_chg_adsp_cp_get_work_mode(struct oplus_chg_ic_dev *ic_dev, enum oplus_cp_work_mode *mode)
{
	*mode = CP_WORK_MODE_BYPASS;

	return 0;
}

static int oplus_chg_adsp_cp_check_work_mode_support(struct oplus_chg_ic_dev *ic_dev, enum oplus_cp_work_mode mode)
{
	if (mode != CP_WORK_MODE_BYPASS)
		return false;
	return true;
}

static int oplus_chg_adsp_cp_set_iin(struct oplus_chg_ic_dev *ic_dev, int iin)
{
	return 0;
}

static int oplus_chg_adsp_cp_set_work_start(struct oplus_chg_ic_dev *ic_dev, bool start)
{
	struct battery_chg_dev *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	rc = oplus_adsp_pps_mos_ctrl(chip, start);
	if (rc < 0)
		return rc;
	oplus_imp_node_set_active(chip->input_imp_node, start);

	return 0;
}

static int oplus_chg_adsp_cp_get_work_status(struct oplus_chg_ic_dev *ic_dev, bool *start)
{
	struct battery_chg_dev *chip;
	int rc = 0;
	struct psy_state *pst = NULL;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &chip->psy_list[PSY_TYPE_USB];

	if (chip->soccp_support)
		rc = read_property_id(chip, &chip->oplus_psy, OPLUS_USB_PPS_MOS_CTRL);
	else
		rc = read_property_id(chip, pst, USB_PPS_MOS_CTRL);
	if (rc < 0) {
		chg_err("read mos open status error, rc = %d\n", rc);
		return rc;
	}
	if (chip->soccp_support)
		*start = chip->oplus_psy.prop[OPLUS_USB_PPS_MOS_CTRL];
	else
		*start = pst->prop[USB_PPS_MOS_CTRL];

	return 0;
}

static int oplus_chg_adsp_cp_adc_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	return 0;
}

static int oplus_chg_adsp_cp_watchdog_reset(struct oplus_chg_ic_dev *ic_dev)
{
	bool work_start;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	rc = oplus_chg_adsp_cp_get_work_status(ic_dev, &work_start);
	if (rc < 0)
		return rc;
	if (!work_start)
		return 0;

	return oplus_chg_adsp_cp_set_work_start(ic_dev, true);
}

static void *oplus_chg_adsp_cp_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	if (!oplus_chg_ic_func_is_support(ic_dev, func_id)) {
		chg_info("%s: this func(=%d) is not supported\n", ic_dev->name, func_id);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, oplus_chg_adsp_cp_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, oplus_chg_adsp_cp_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP, oplus_chg_adsp_cp_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST, oplus_chg_adsp_cp_smt_test);
		break;
	case OPLUS_IC_FUNC_CP_GET_VIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_VIN, oplus_chg_adsp_cp_get_vin);
		break;
	case OPLUS_IC_FUNC_CP_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_ENABLE, oplus_chg_adsp_cp_enable);
		break;
	case OPLUS_IC_FUNC_CP_HW_INTI:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_HW_INTI, oplus_chg_adsp_cp_hw_init);
		break;
	case OPLUS_IC_FUNC_CP_SET_WORK_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_WORK_MODE, oplus_chg_adsp_cp_set_work_mode);
		break;
	case OPLUS_IC_FUNC_CP_GET_WORK_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_WORK_MODE, oplus_chg_adsp_cp_get_work_mode);
		break;
	case OPLUS_IC_FUNC_CP_CHECK_WORK_MODE_SUPPORT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_CHECK_WORK_MODE_SUPPORT,
			oplus_chg_adsp_cp_check_work_mode_support);
		break;
	case OPLUS_IC_FUNC_CP_SET_IIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_IIN, oplus_chg_adsp_cp_set_iin);
		break;
	case OPLUS_IC_FUNC_CP_SET_WORK_START:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_WORK_START, oplus_chg_adsp_cp_set_work_start);
		break;
	case OPLUS_IC_FUNC_CP_GET_WORK_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_WORK_STATUS, oplus_chg_adsp_cp_get_work_status);
		break;
	case OPLUS_IC_FUNC_CP_SET_ADC_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_ADC_ENABLE, oplus_chg_adsp_cp_adc_enable);
		break;
	case OPLUS_IC_FUNC_CP_GET_TEMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_TEMP, oplus_chg_adsp_get_chg_mos_temp);
		break;
	case OPLUS_IC_FUNC_CP_WATCHDOG_RESET:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_WATCHDOG_RESET, oplus_chg_adsp_cp_watchdog_reset);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq oplus_chg_adsp_cp_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
};

static int oplus_chg_adsp_dpdm_switch_init(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	if (ic_dev->online)
		return 0;
	ic_dev->online = true;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_ONLINE);

	return 0;
}

static int oplus_chg_adsp_dpdm_switch_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (!ic_dev->online)
		return 0;
	ic_dev->online = false;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_OFFLINE);

	return 0;
}

static int oplus_chg_adsp_dpdm_switch_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	return 0;
}

static int oplus_chg_adsp_dpdm_switch_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	return 0;
}

static int oplus_chg_adsp_dpdm_switch_set_switch_mode(struct oplus_chg_ic_dev *ic_dev,
	enum oplus_dpdm_switch_mode mode)
{
	struct battery_chg_dev *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	switch (mode) {
	case DPDM_SWITCH_TO_AP:
		chg_info("dpdm switch to ap\n");
		rc = oplus_adsp_voocphy_enable(false);
		break;
	case DPDM_SWITCH_TO_VOOC:
		chg_info("dpdm switch to vooc\n");
		rc = oplus_adsp_voocphy_enable(true);
		break;
	case DPDM_SWITCH_TO_UFCS:
		chg_info("dpdm switch to ufcs\n");
		rc = oplus_adsp_voocphy_enable(false);
		break;
	default:
		chg_err("not supported mode, mode=%d\n", mode);
		return -EINVAL;
	}
	chip->dpdm_switch_mode = mode;

	return rc;
}

static int oplus_chg_adsp_dpdm_switch_get_switch_mode(struct oplus_chg_ic_dev *ic_dev,
	enum oplus_dpdm_switch_mode *mode)
{
	struct battery_chg_dev *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*mode = chip->dpdm_switch_mode;

	return 0;
}

static void *oplus_chg_adsp_dpdm_switch_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}
	if (!oplus_chg_ic_func_is_support(ic_dev, func_id)) {
		chg_info("%s: this func(=%d) is not supported\n",  ic_dev->name, func_id);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, oplus_chg_adsp_dpdm_switch_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, oplus_chg_adsp_dpdm_switch_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP, oplus_chg_adsp_dpdm_switch_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST, oplus_chg_adsp_dpdm_switch_smt_test);
		break;
	case OPLUS_IC_FUNC_SET_DPDM_SWITCH_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_DPDM_SWITCH_MODE,
			oplus_chg_adsp_dpdm_switch_set_switch_mode);
		break;
	case OPLUS_IC_FUNC_GET_DPDM_SWITCH_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_DPDM_SWITCH_MODE,
			oplus_chg_adsp_dpdm_switch_get_switch_mode);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq oplus_chg_adsp_dpdm_switch_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
};

static int oplus_chg_get_r_cool_down(void) {
	int rc = 0;
	struct psy_state *pst = NULL;
	struct battery_chg_dev *bcdev = g_bcdev;
	int cool_down = 0;

	if (!bcdev) {
		return false;
	}

	pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_PPS_GET_R_COOL_DOWN);
	else
		rc = read_property_id(bcdev, pst, USB_PPS_GET_R_COOL_DOWN);
	if (rc < 0) {
		chg_err(" fail, rc = %d\n", rc);
		return -1;
	}
	if (bcdev->soccp_support)
		cool_down = bcdev->oplus_psy.prop[OPLUS_USB_PPS_GET_R_COOL_DOWN];
	else
		cool_down = pst->prop[USB_PPS_GET_R_COOL_DOWN];
	chg_err("cool_down = %d\n", cool_down);
	return cool_down;
}

static int oplus_chg_adsp_pps_init(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	ic_dev->online = true;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_ONLINE);

	return 0;
}

static int oplus_chg_adsp_pps_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	ic_dev->online = false;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_OFFLINE);

	return 0;
}

static int oplus_pps_get_authentiate(struct oplus_chg_ic_dev *ic_dev) {
	int rc = 0;
	struct psy_state *pst = NULL;
	struct battery_chg_dev *bcdev;
	int auth;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_USB];
	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_PPS_GET_AUTHENTICATE);
	else
		rc = read_property_id(bcdev, pst, USB_PPS_GET_AUTHENTICATE);
	if (rc < 0) {
		chg_err("oplus_pps_get_authentiate fail, rc = %d\n", rc);
		return -1;
	}

	if (bcdev->soccp_support)
		auth = bcdev->oplus_psy.prop[OPLUS_USB_PPS_GET_AUTHENTICATE];
	else
		auth = pst->prop[USB_PPS_GET_AUTHENTICATE];
	chg_err("oplus_pps_get_authentiate  = %d\n", auth);

	return auth;
}

static int oplus_chg_set_pps_config(struct oplus_chg_ic_dev *ic_dev, int vbus_mv, int ibus_ma)
{
	int rc1, rc2 = 0;
	struct psy_state *pst = NULL;
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	chg_err("%s:%dmV-%dmA\n", __func__, vbus_mv, ibus_ma);
	pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (bcdev->soccp_support) {
		rc1 = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_SET_PPS_VOLT, vbus_mv);
		rc2 = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_SET_PPS_CURR, ibus_ma);
	} else {
		rc1 = write_property_id(bcdev, pst, USB_SET_PPS_VOLT, vbus_mv);
		rc2 = write_property_id(bcdev, pst, USB_SET_PPS_CURR, ibus_ma);
	}

	if (rc1 < 0 || rc2 < 0) {
		chg_err("set pps config fail, rc1,rc2 = %d, %d\n", rc1, rc2);
		return -1;
	}

	return 0;
}

static int oplus_chg_set_fixed_pd_config(struct oplus_chg_ic_dev *ic_dev, int vol_mv, int curr_ma)
{
	int rc = 0;

#define OPLUS_PD_5V_PDO 0x31912c
#define OPLUS_PD_9V_PDO 0x32d12c
#define OPLUS_PD_12V_PDO 0x33c12c

	if (vol_mv == OPLUS_PD_5V)
		rc = oplus_chg_8350_set_pd_config(ic_dev, OPLUS_PD_5V_PDO);
	else if (vol_mv == OPLUS_PD_9V)
		rc = oplus_chg_8350_set_pd_config(ic_dev, OPLUS_PD_9V_PDO);
	else if (vol_mv == OPLUS_PD_12V)
		rc = oplus_chg_8350_set_pd_config(ic_dev, OPLUS_PD_12V_PDO);
	else
		return -1;

	return rc;
}

static int oplus_get_pps_info_from_adsp(struct oplus_chg_ic_dev *ic_dev, u32 *pdo, int num)
{
	int pdo_index = 0;
	int pdo_max = num > PPS_PDO_MAX ? PPS_PDO_MAX:num;
	struct battery_chg_dev *bcdev;
	int imax = 0, vmax = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	pps_read_buffer(bcdev);
	imax = bcdev->pps_read_buffer_dump.data_buffer[0];
	vmax = bcdev->pps_read_buffer_dump.data_buffer[1];
	chg_err("oplus_get_pps_parameters_from_adsp imax = %d, vmax = %d\n", imax, vmax);

	for (pdo_index = 0; pdo_index < pdo_max; pdo_index++) {
		pdo[pdo_index] = bcdev->pps_read_buffer_dump.data_buffer[pdo_index + 2];
	}
	return 0;
}

static int oplus_chg_get_pps_status(struct oplus_chg_ic_dev *ic_dev, u32 *src_info)
{
	int rc = 0;
	struct psy_state *pst = NULL;
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	oplus_chg_get_r_cool_down();

	pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_GET_PPS_STATUS);
	else
		rc = read_property_id(bcdev, pst, USB_GET_PPS_STATUS);
	if (rc < 0) {
		chg_err("get pps status fail, rc = %d\n", rc);
		return -1;
	}

	if (bcdev->soccp_support)
		*src_info = bcdev->oplus_psy.prop[OPLUS_USB_GET_PPS_STATUS];
	else
		*src_info = pst->prop[USB_GET_PPS_STATUS];
	chg_err("PPS status = 0x%x\n", *src_info);

	return 0;
}

static void *oplus_chg_adsp_pps_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, oplus_chg_adsp_pps_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, oplus_chg_adsp_pps_exit);
		break;
	case OPLUS_IC_FUNC_PPS_PDO_SET:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_PPS_PDO_SET, oplus_chg_set_pps_config);
		break;
	case OPLUS_IC_FUNC_FIXED_PDO_SET:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_FIXED_PDO_SET, oplus_chg_set_fixed_pd_config);
		break;
	case OPLUS_IC_FUNC_PPS_VERIFY_ADAPTER:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_PPS_VERIFY_ADAPTER, oplus_pps_get_authentiate);
		break;
	case OPLUS_IC_FUNC_PPS_GET_PDO_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_PPS_GET_PDO_INFO, oplus_get_pps_info_from_adsp);
		break;
	case OPLUS_IC_FUNC_GET_PPS_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_PPS_STATUS, oplus_chg_get_pps_status);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq oplus_chg_adsp_pps_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
};

static int oplus_chg_adsp_ufcs_init(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	if (ic_dev->online)
		return 0;
	ic_dev->online = true;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_ONLINE);

	return 0;
}

static int oplus_chg_adsp_ufcs_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (!ic_dev->online)
		return 0;
	ic_dev->online = false;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_OFFLINE);

	return 0;
}

static int oplus_chg_adsp_ufcs_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	return 0;
}

static int oplus_chg_adsp_ufcs_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	return 0;
}

static int oplus_chg_adsp_ufcs_handshake(struct oplus_chg_ic_dev *ic_dev)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	int rc = -1, rc1 = -1;
	int start = 1;
	int retry_count = 12;
	int bc12_wait_count = 12;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_USB];

	/*add for prevent bc1.2 and ufcs waveforms from overlap*/
	if (bcdev->charger_type == OPLUS_CHG_USB_TYPE_PD ||
	    bcdev->charger_type == OPLUS_CHG_USB_TYPE_PD_DRP ||
	    bcdev->charger_type == OPLUS_CHG_USB_TYPE_PD_PPS) {
		while (bc12_wait_count--) {
			chg_info("bcdev->bc12_completed = %d\n", bcdev->bc12_completed);
			if (bcdev->bc12_completed) {
				break;
			}
			msleep(20);
		}
	}

	if (bcdev->soccp_support)
		rc1 = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_SET_UFCS, start);
	else
		rc1 = write_property_id(bcdev, pst, USB_SET_UFCS_START, start);
	if (rc1 < 0) {
		chg_err("start UFCS func error, rc1=%d\n", rc1);
		return rc1;
	}

	while (retry_count--) {
		chg_info("bcdev->ufcs_handshake_ok = %d\n", bcdev->ufcs_handshake_ok);
		if (bcdev->ufcs_handshake_ok) {
			rc = 0;
			break;
		} else if (!bcdev->usb_in_status) {
			chg_info("usb unpluged, not retry.\n");
			break;
		}
		msleep(10);
	}

	return rc;
}

static int oplus_chg_adsp_ufcs_pdo_set(struct oplus_chg_ic_dev *ic_dev, int vbus_mv, int ibus_ma)
{
	int rc1 = 0, rc2 = 0;
	struct psy_state *pst = NULL;
	struct battery_chg_dev *bcdev;
	int retry_count = 37;
	bool pdo_set_success = false;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_USB];

	while (retry_count--) {
		chg_info("bcdev->ufcs_power_ready = %d\n", bcdev->ufcs_power_ready);
		if (bcdev->ufcs_exiting)
			break;
		if (bcdev->ufcs_power_ready && !bcdev->ufcs_exiting) {
			chg_info("success set vbus_mv = %d, ibus_ma = %d\n", vbus_mv, ibus_ma);
			if (bcdev->soccp_support) {
				rc1 = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_SET_UFCS_VOLT, vbus_mv);
				rc2 = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_SET_UFCS_CURRENT, ibus_ma);
			} else {
				rc1 = write_property_id(bcdev, pst, USB_SET_UFCS_VOLT, vbus_mv);
				rc2 = write_property_id(bcdev, pst, USB_SET_UFCS_CURRENT, ibus_ma);
			}
			if (rc1 < 0 || rc2 < 0) {
				chg_err("set ufcs config fail, rc1,rc2 = %d, %d\n", rc1, rc2);
				return -1;
			}
			bcdev->ufcs_power_ready = 0;
			pdo_set_success = true;
			break;
		} else if (!bcdev->usb_in_status) {
			chg_info("usb unpluged, not retry.\n");
			break;
		}
		msleep(20);
	}

	if (!pdo_set_success) {
		chg_err("set ufcs config failed because of wait ufcs_power_ready timeout.");
		return -1;
	}

	return 0;
}

static int oplus_chg_adsp_ufcs_get_dev_info(struct oplus_chg_ic_dev *ic_dev, u64 *dev_info)
{
	struct psy_state *pst = NULL;
	struct battery_chg_dev *bcdev;
	int rc1 = 0, rc2 = 0;
	u64 dev_info_l = 0;
	u64 dev_info_h = 0;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_USB];
	if (bcdev->soccp_support) {
		rc1 = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_GET_UFCS_DEV_INFO_L);
		rc2 = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_GET_UFCS_DEV_INFO_H);
	} else {
		rc1 = read_property_id(bcdev, pst, USB_GET_DEV_INFO_L);
		rc2 = read_property_id(bcdev, pst, USB_GET_DEV_INFO_H);
	}
	if (rc1 < 0 || rc2 < 0) {
		chg_err("get ufcs device info fail, rc1 = %d, rc2 = %d\n", rc1, rc2);
		return -1;
	}

	if (bcdev->soccp_support) {
		dev_info_l = bcdev->oplus_psy.prop[OPLUS_USB_GET_UFCS_DEV_INFO_L];
		dev_info_h = bcdev->oplus_psy.prop[OPLUS_USB_GET_UFCS_DEV_INFO_H];
	} else {
		dev_info_l = pst->prop[USB_GET_DEV_INFO_L];
		dev_info_h = pst->prop[USB_GET_DEV_INFO_H];
	}

	*dev_info = dev_info_l | (dev_info_h << 32);
	chg_err("dev_info_l = 0x%llx, dev_info_h = 0x%llx, *dev_info = 0x%llx\n", dev_info_l, dev_info_h, *dev_info);

	return 0;
}

#define UFCS_PDO_MAX 7
static int oplus_chg_adsp_ufcs_get_pdo_info_buffer(struct oplus_chg_ic_dev *ic_dev, u64 *pdo, int num)
{
	int pdo_index = 0;
	int pdo_max = num > UFCS_PDO_MAX ? UFCS_PDO_MAX : num;
	int pdo_num = -1;
	struct battery_chg_dev *bcdev;
	int retry_count = 12;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	while (retry_count--) {
		if (bcdev->ufcs_pdo_ready) {
			ufcs_read_buffer(bcdev, UFCS_PDO_INFO);
			for (pdo_index = 0; pdo_index < pdo_max; pdo_index++) {
				pdo[pdo_index] = bcdev->ufcs_read_buffer_dump.data_buffer[pdo_index];
				if (pdo[pdo_index] == 0) {
					pdo_num = pdo_index;
					break;
				}
				chg_err("pdo[%d] = 0x%llx\n", pdo_index, pdo[pdo_index]);
			}
			bcdev->ufcs_pdo_ready = 0;
			break;
		} else if (!bcdev->usb_in_status) {
			chg_info("usb unpluged, not retry.\n");
			break;
		}
		msleep(10);
	}
	return pdo_num;
}

static int oplus_chg_adsp_ufcs_get_src_info(struct oplus_chg_ic_dev *ic_dev, u64 *src_info)
{
	struct psy_state *pst = NULL;
	struct battery_chg_dev *bcdev;
	int rc1 = 0, rc2 = 0;
	u64 src_info_l = 0;
	u64 src_info_h = 0;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_USB];
	if (bcdev->soccp_support) {
		rc1 = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_GET_UFCS_SRC_INFO_L);
		rc2 = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_GET_UFCS_SRC_INFO_H);
	} else {
		rc1 = read_property_id(bcdev, pst, USB_GET_SRC_INFO_L);
		rc2 = read_property_id(bcdev, pst, USB_GET_SRC_INFO_H);
	}
	if (rc1 < 0 || rc2 < 0) {
		chg_err("get ufcs source info fail, rc1 = %d, rc2 = %d\n", rc1, rc2);
		return -1;
	}

	if (bcdev->soccp_support) {
		src_info_l = bcdev->oplus_psy.prop[OPLUS_USB_GET_UFCS_SRC_INFO_L];
		src_info_h = bcdev->oplus_psy.prop[OPLUS_USB_GET_UFCS_SRC_INFO_H];
	} else {
		src_info_l = pst->prop[USB_GET_SRC_INFO_L];
		src_info_h = pst->prop[USB_GET_SRC_INFO_H];
	}

	*src_info = src_info_l | (src_info_h << 32);
	chg_err("src_info_l = 0x%llx, src_info_h = 0x%llx, *src_info = 0x%llx\n", src_info_l, src_info_h, *src_info);

	return 0;
}

static int oplus_chg_adsp_ufcs_is_test_mode(struct oplus_chg_ic_dev *ic_dev, bool *en)
{
	struct battery_chg_dev *bcdev = g_bcdev;
	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}

	if (en == NULL) {
		chg_err("en is NULL");
		return -EINVAL;
	}

	*en = bcdev->ufcs_test_mode;
	chg_info("ufcs test mode is %d\n", bcdev->ufcs_test_mode);

	return 0;
}

static int oplus_chg_adsp_ufcs_is_vol_acc_test_mode(struct oplus_chg_ic_dev *ic_dev, bool *en)
{
	return 0;/*TODO*/
}

static int oplus_chg_adsp_ufcs_config_wd(struct oplus_chg_ic_dev *ic_dev, u16 time_ms)
{
	int rc = 0;
	/*struct psy_state *pst = NULL;
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chg_info("time_ms = %d\n", time_ms);
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_USB];

	rc = write_property_id(bcdev, pst, USB_SET_WD_TIME, time_ms);
	if (rc < 0) {
		chg_err("set ufcs config fail, rc= %d\n", rc);
		return rc;
	}*/

	return rc;
}

static int oplus_chg_adsp_ufcs_running_state(struct oplus_chg_ic_dev *ic_dev, bool *state)
{
	int rc = 0;
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (!bcdev) {
		chg_err("bcdev is NULL");
		return -ENODEV;
	}

	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	if (bcdev->soccp_support)
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_GET_UFCS_RUNNING_STATE);
	else
		rc = read_property_id(bcdev, pst, BATT_GET_UFCS_RUNNING_STATE);
	if (rc < 0) {
		chg_err("rc is %d read failed!", rc);
	} else {
		if (bcdev->soccp_support)
			*state = bcdev->oplus_psy.prop[OPLUS_GET_UFCS_RUNNING_STATE];
		else
			*state = pst->prop[BATT_GET_UFCS_RUNNING_STATE];
		rc = 0;
	}

	return rc;
}

#define OPLUS_UFCS_WAIT_EXIT_MAX_RETRY		30

static int oplus_chg_adsp_ufcs_exit_ufcs_mode(struct oplus_chg_ic_dev *ic_dev)
{
	int rc = 0;
	struct psy_state *pst = NULL;
	struct battery_chg_dev *bcdev;
	int exit = 1;
	bool state = true;
	int retry_count = 0; /* wait at most 600ms */

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (!bcdev->ufcs_handshake_ok) {
		chg_err("ufcs handshake fail or plugout, can't send exit ufcs mode\n");
		return -1;
	}
	bcdev->ufcs_exiting = true;
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_SET_UFCS_EXIT, exit);
	else
		rc = write_property_id(bcdev, pst, USB_SET_EXIT, exit);
	if (rc < 0) {
		chg_err("set ufcs config fail, rc= %d\n", rc);
		rc = -1;
		goto exit;
	}

	if (bcdev->ufcs_run_check_support) {
		/* wait until the ufcs is realy exited to avoid DP/DM access conflict! */
		while (retry_count < OPLUS_UFCS_WAIT_EXIT_MAX_RETRY) {
			rc = oplus_chg_adsp_ufcs_running_state(ic_dev, &state);
			chg_info("retry_count = %d, state = %d, rc = %d", retry_count, state, rc);

			if ((rc < 0) || (state == false)) {
				chg_info("ufcs is exited now, not wait, retry_count %d\n", retry_count);
				break;
			}

			/* when the usb is not connected, no need to wait! */
			if (!bcdev->cid_status) {
				chg_info("usb unpluged, not retry.\n");
				break;
			}
			retry_count++;
			msleep(20);
		}
	}

exit:
	bcdev->ufcs_power_ready = false;
	bcdev->ufcs_handshake_ok = false;
	bcdev->ufcs_pdo_ready = false;
	bcdev->ufcs_verify_auth_ready = false;
	bcdev->ufcs_power_info_ready = false;
	bcdev->ufcs_vdm_emark_ready = false;
	bcdev->ufcs_exiting = false;

	return rc;
}

static int oplus_chg_ufcs_adapter_verify(struct battery_chg_dev *bcdev, u8 key_index, u8 *auth_data, u8 data_len)
{
	int key_len = 0;
	struct psy_state *pst = NULL;
	struct adapter_verify_req_msg req_msg = { { 0 } };

	pst = &bcdev->psy_list[PSY_TYPE_USB];

	req_msg.ufcs_req.hdr.owner = MSG_OWNER_BC;
	req_msg.ufcs_req.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.ufcs_req.hdr.opcode = AP_OPCODE_UFCS_BUFFER;
	req_msg.ufcs_req.msg_id = UFCS_ADAPTER_VERIFY;
	req_msg.key_index = key_index;
	req_msg.auth_data_size = data_len;

	key_len = data_len <= ADAPTER_VERIFY_AUTH_DATA_SIZE ? data_len : ADAPTER_VERIFY_AUTH_DATA_SIZE;
	memcpy(req_msg.auth_data, auth_data, key_len);

	return ufcs_battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

static int oplus_chg_adsp_ufcs_verify_adapter(struct oplus_chg_ic_dev *ic_dev, u8 key_index, u8 *auth_data, u8 data_len)
{
	struct battery_chg_dev *bcdev;
	int retry_count = 12;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (bcdev->ufcs_key_to_adsp_done) {
		while (retry_count--) {
			if (bcdev->ufcs_verify_auth_ready) {
				oplus_chg_ufcs_adapter_verify(bcdev, key_index, auth_data, data_len);
				break;
			} else if (!bcdev->usb_in_status) {
				chg_info("usb unpluged, not retry.\n");
				break;
			}
			msleep(10);
		}
	} else {
		oplus_chg_ufcs_adapter_verify(bcdev, key_index, auth_data, data_len);
		bcdev->ufcs_key_to_adsp_done = true;
	}

	chg_info("bcdev->adapter_verify_auth = %d\n", bcdev->adapter_verify_auth);

	return bcdev->adapter_verify_auth;
}

static int oplus_chg_adsp_ufcs_get_power_info_ext(struct oplus_chg_ic_dev *ic_dev, u64 *info, int num)
{
	int pwr_index = 0;
	int pwr_max = 0;
	int pwr_num = -1;
	struct battery_chg_dev *bcdev;
	int retry_count = 20;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	pwr_max = num > OPLUS_VND_POWER_INFO_MAX ? OPLUS_VND_POWER_INFO_MAX : num;
	while (retry_count--) {
		chg_err("retry %d time, ufcs_pwr_ready=%d\n", 20 - retry_count, bcdev->ufcs_power_info_ready);
		if (bcdev->ufcs_power_info_ready) {
			ufcs_read_buffer(bcdev, UFCS_VDM_PWR_INFO);
			for (pwr_index = 0; pwr_index < pwr_max; pwr_index++) {
				info[pwr_index] = bcdev->ufcs_read_buffer_dump.data_buffer[pwr_index];
				if (info[pwr_index] == 0)
					break;
				chg_err("pwr[%d] = 0x%016llX\n", pwr_index, info[pwr_index]);
			}
			pwr_num = pwr_index;
			bcdev->ufcs_power_info_ready = false;
			break;
		} else if (!bcdev->usb_in_status) {
			chg_info("usb unpluged, not retry.\n");
			break;
		}
		msleep(10);
	}
	return pwr_num;
}

#define UFCS_GET_EMARK_INFO_RETRY_COUNT (60)
static int oplus_chg_adsp_ufcs_get_emark_info(struct oplus_chg_ic_dev *ic_dev, u64 *info)
{
	struct battery_chg_dev *bcdev;
	int retry_count = UFCS_GET_EMARK_INFO_RETRY_COUNT;
	int rc = -1;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	while (retry_count--) {
		chg_err("retry %d time, ufcs_vdm_emark_ready=%d\n", UFCS_GET_EMARK_INFO_RETRY_COUNT - retry_count, bcdev->ufcs_vdm_emark_ready);
		if (bcdev->ufcs_vdm_emark_ready) {
			ufcs_read_buffer(bcdev, UFCS_VDM_EMARK_INFO);
			*info = bcdev->ufcs_read_buffer_dump.data_buffer[0];
			bcdev->ufcs_vdm_emark_ready = false;
			rc = 1;
			break;
		} else if (!bcdev->usb_in_status) {
			chg_info("usb unpluged, not retry.\n");
			break;
		}
		msleep(10);
	}

	return rc;/*TODO*/
}

static int oplus_chg_adsp_ufcs_set_sm_period(struct oplus_chg_ic_dev *ic_dev, u16 time_ms)
{
	int rc = 0;
	struct psy_state *pst = NULL;
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chg_info("time_ms = %d\n", time_ms);
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_SET_UFCS_SM_PERIOD, time_ms);
	else
		rc = write_property_id(bcdev, pst, USB_SET_UFCS_SM_PERIOD, time_ms);
	if (rc < 0) {
		chg_err("set ufcs config fail, rc= %d\n", rc);
		return rc;
	}

	return rc;
}

static void *oplus_chg_adsp_ufcs_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}
	if (!oplus_chg_ic_func_is_support(ic_dev, func_id)) {
		chg_info("%s: this func(=%d) is not supported\n",  ic_dev->name, func_id);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT,
			oplus_chg_adsp_ufcs_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT,
			oplus_chg_adsp_ufcs_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP,
			oplus_chg_adsp_ufcs_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST,
			oplus_chg_adsp_ufcs_smt_test);
		break;
	case OPLUS_IC_FUNC_UFCS_HANDSHAKE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_HANDSHAKE,
			oplus_chg_adsp_ufcs_handshake);
		break;
	case OPLUS_IC_FUNC_UFCS_PDO_SET:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_PDO_SET,
			oplus_chg_adsp_ufcs_pdo_set);
		break;
	case OPLUS_IC_FUNC_UFCS_GET_DEV_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_GET_DEV_INFO,
			oplus_chg_adsp_ufcs_get_dev_info);
		break;
	case OPLUS_IC_FUNC_UFCS_GET_PDO_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_GET_PDO_INFO,
			oplus_chg_adsp_ufcs_get_pdo_info_buffer);
		break;
	case OPLUS_IC_FUNC_UFCS_IS_TEST_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_IS_TEST_MODE,
			oplus_chg_adsp_ufcs_is_test_mode);
		break;
	case OPLUS_IC_FUNC_UFCS_IS_VOL_ACC_TEST_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_IS_VOL_ACC_TEST_MODE,
			oplus_chg_adsp_ufcs_is_vol_acc_test_mode);
		break;
	case OPLUS_IC_FUNC_UFCS_CONFIG_WD:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_CONFIG_WD,
			oplus_chg_adsp_ufcs_config_wd);
		break;
	case OPLUS_IC_FUNC_UFCS_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_EXIT,
			oplus_chg_adsp_ufcs_exit_ufcs_mode);
		break;
	case OPLUS_IC_FUNC_UFCS_VERIFY_ADAPTER:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_VERIFY_ADAPTER,
			oplus_chg_adsp_ufcs_verify_adapter);
		break;
	case OPLUS_IC_FUNC_UFCS_GET_POWER_INFO_EXT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_GET_POWER_INFO_EXT,
			oplus_chg_adsp_ufcs_get_power_info_ext);
		break;
	case OPLUS_IC_FUNC_UFCS_GET_EMARK_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_GET_EMARK_INFO,
			oplus_chg_adsp_ufcs_get_emark_info);
		break;
	case OPLUS_IC_FUNC_UFCS_GET_SRC_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_GET_SRC_INFO,
			oplus_chg_adsp_ufcs_get_src_info);
		break;
	/*case OPLUS_IC_FUNC_UFCS_GET_CABLE_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_GET_CABLE_INFO,
			oplus_chg_adsp_ufcs_get_cable_info);
		break;
	case OPLUS_IC_FUNC_UFCS_HARD_RESET:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_HARD_RESET,
			oplus_chg_adsp_ufcs_hard_reset);
		break;
	case OPLUS_IC_FUNC_UFCS_GET_ERR_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_GET_ERR_INFO,
			oplus_chg_adsp_ufcs_get_err_info);
		break;
	case OPLUS_IC_FUNC_UFCS_GET_POWER_CHANGE_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_GET_POWER_CHANGE_INFO,
			oplus_chg_adsp_ufcs_get_power_change_info);
		break;*/
	case OPLUS_IC_FUNC_UFCS_SET_SM_PERIOD:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_SET_SM_PERIOD,
			oplus_chg_adsp_ufcs_set_sm_period);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

#ifdef CONFIG_OPLUS_CHG_IC_DEBUG
static ssize_t vph_show(struct device *dev, struct device_attribute *attr,
				    char *buf)
{
	struct oplus_chg_ic_dev *ic_dev = dev_get_drvdata(dev);
	struct battery_chg_dev *bcdev;
	int size = 0;
	int rc = 0;
	int vph = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return size;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (IS_ERR_OR_NULL(bcdev->iio.vph_pwr_chan)) {
		chg_err("bcdev->iio.vph_pwr_chan is NULL\n");
		goto exit;
	}

	rc = iio_read_channel_processed(bcdev->iio.vph_pwr_chan, &vph);
	if (rc < 0) {
		chg_err("iio_read_channel_processed get error[%d]\n", rc);
		goto exit;
	}

	size += snprintf(buf + size, PAGE_SIZE - size, "%d\n", vph);
exit:
	return size;
}

static ssize_t vbat_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct oplus_chg_ic_dev *ic_dev = dev_get_drvdata(dev);
	struct battery_chg_dev *bcdev;
	int size = 0;
	int rc = 0;
	int vbat = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return size;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);

	if (IS_ERR_OR_NULL(bcdev->iio.vbat_sns_qbg_chan)) {
		chg_err("bcdev->iio.vbat_sns_qbg_chan is NULL\n");
		goto exit;
	}

	rc = iio_read_channel_processed(bcdev->iio.vbat_sns_qbg_chan, &vbat);
	if (rc < 0) {
		chg_err("iio_read_channel_processed get error[%d]\n", rc);
		goto exit;
	}

	size += snprintf(buf + size, PAGE_SIZE - size, "%d\n", vbat);
	exit:
	return size;
}

static struct device_attribute dev_attr_vbat = {
	.attr = {
		.name = __stringify(vbat_sns_qbg),
		.mode = 0444
	},
	.show = vbat_show,
};

static struct device_attribute dev_attr_vph = {
	.attr = {
		.name = __stringify(vph_pwr),
		.mode = 0444
	},
	.show = vph_show,
};


static struct device_attribute *adsp_buck_attributes[] = {
	&dev_attr_vbat,
	&dev_attr_vph,
	NULL
};
#endif

struct oplus_chg_ic_virq oplus_chg_adsp_ufcs_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
};

static int oplus_sm8350_get_gauge_devinfo(struct device_node *node, const char *name[])
{
	int count;
	int i;

	if (!of_property_read_bool(node, "oplus,ic_devinfo"))
		return -ENOTSUPP; /* not support */

	count = of_property_count_strings(node, "oplus,ic_devinfo");
	if (count < 0 || count != 2) {
		chg_err("Error reading oplus,ic_devinfo: %d\n", count);
		return count;
	}
	for (i = 0; i < count; i++) {
		if (of_property_read_string_index(node, "oplus,ic_devinfo", i, &name[i])) {
			chg_err("Error reading devinfo %d\n", i);
			return -EINVAL;
		} else
			chg_info("name[%d] = %s\n", i, name[i]);
	}

	return 0;
}

static int oplus_chg_get_reverse_enbale(struct oplus_chg_ic_dev *ic_dev, bool *enable)
{
	struct battery_chg_dev *chip;
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*enable = chip->reverse_enable;
	chg_info("oplus chg get reverse enbale\n");
	return 0;
}

static int reverse_chg_init(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	ic_dev->online = true;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_ONLINE);
	return 0;
}

static int reverse_chg_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (!ic_dev->online)
		return 0;

	ic_dev->online = false;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_OFFLINE);
	return 0;
}

static int reverse_chg_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	return 0;
}

static int  reverse_chg_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	return 0;
}

static int  oplus_chg_set_wdt_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	return 0;
}

static int  oplus_chg_set_kick_wdt(struct oplus_chg_ic_dev *ic_dev)
{
	return 0;
}

static int  oplus_chg_get_rvs_chg_msg_type(struct oplus_chg_ic_dev *ic_dev,
	int *msg_type)
{
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (!bcdev)
		return -ENODEV;

	*msg_type = bcdev->rvs_msg_type;
	return 0;
}

static int  oplus_chg_get_sink_req_pdo(struct oplus_chg_ic_dev *ic_dev,
	int *req_voltage, int *req_current)
{
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (!bcdev)
		return -ENODEV;

	*req_voltage = bcdev->sink_req_volt;
	*req_current = bcdev->sink_req_curr;
	return 0;
}

static int oplus_chg_get_high_reverse_enbale(struct oplus_chg_ic_dev *ic_dev, bool *enable)
{
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (!bcdev)
		return -ENODEV;

	*enable = bcdev->high_reverse_enable;
	return 0;
}

static int oplus_chg_adsp_get_svid(struct battery_chg_dev *bcdev, u32 *svid)
{
	int rc = 0;
	struct psy_state *pst = NULL;

	if (bcdev == NULL) {
		chg_err("bcdev is NULL\n");
		return -ENODEV;
	}
	if (svid == NULL) {
		chg_err("svid is NULL\n");
		return -EINVAL;
	}

	if (!bcdev->soccp_support) {
		pst = &bcdev->psy_list[PSY_TYPE_USB];
		rc = read_property_id(bcdev, pst, USB_ADAPTER_SVID);
		if (rc < 0) {
			chg_err("read USB_ADAPTER_SVID fail, rc=%d\n", rc);
			return rc;
		}
		*svid = pst->prop[USB_ADAPTER_SVID] & 0xFFFF;
	} else {
		rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_GET_ADAPTER_SVID);
		if (rc < 0) {
			chg_err("read OPLUS_GET_ADAPTER_SVID fail, rc=%d\n", rc);
			return rc;
		}
		*svid = bcdev->oplus_psy.prop[OPLUS_GET_ADAPTER_SVID] & 0xFFFF;
	}
	chg_info("get svid=0x%04x (soccp_support=%d)\n", *svid, bcdev->soccp_support);
	return 0;
}

static int oplus_chg_get_reverse_chg_svid(struct oplus_chg_ic_dev *ic_dev, u32 *svid)
{
	struct battery_chg_dev *bcdev;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (svid == NULL) {
		chg_err("svid is NULL\n");
		return -EINVAL;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (!bcdev)
		return -ENODEV;

	return oplus_chg_adsp_get_svid(bcdev, svid);
}

static int oplus_chg_set_reverse_boost_pdo
	(struct oplus_chg_ic_dev *ic_dev, int pdo0_volt, int pdo0_curr, int pdo_voltage, int pdo_current)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (!bcdev)
		return 0;

	pst = &bcdev->psy_list[PSY_TYPE_USB];

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_REVERSE_CHG_SET_VOLT, pdo_voltage);
	else
		rc = write_property_id(bcdev, pst, USB_REVERSE_CHG_SET_VOLT, pdo_voltage);
	if (rc)
		chg_err("set volt fail, rc=%d\n", rc);

	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_REVERSE_CHG_SET_CURRENT, pdo_current);
	else
		rc = write_property_id(bcdev, pst, USB_REVERSE_CHG_SET_CURRENT, pdo_current);
	if (rc)
		chg_err("set current fail, rc=%d\n", rc);
	chg_info("oplus chg set reverse pdo_voltage = %d, pdo_current = %d\n",
			pdo_voltage, pdo_current);
	return rc;
}

static int oplus_chg_set_rvs_high_mode_en(struct oplus_chg_ic_dev *ic_dev, int en)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (!bcdev)
		return 0;

	pst = &bcdev->psy_list[PSY_TYPE_USB];

	rc = write_property_id(bcdev, pst, USB_RVS_HIGH_MODE_EN, en);
	if (rc)
		chg_err("set volt fail, rc=%d\n", rc);

	chg_info("oplus chg set rvs_high_mode_en[%d]\n", en);
	return rc;
}

static int oplus_chg_set_normal_rvs_ocp(struct oplus_chg_ic_dev *ic_dev, int curr_ma)
{
	struct battery_chg_dev *bcdev;
	struct psy_state *pst = NULL;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	bcdev = oplus_chg_ic_get_drvdata(ic_dev);
	if (!bcdev) {
		chg_err("bcdev is NULL");
		return -ENODEV;
	}

	pst = &bcdev->psy_list[PSY_TYPE_USB];
	if (bcdev->soccp_support)
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_OTG_BOOST_CURRENT, curr_ma);
	else
		rc = write_property_id(bcdev, pst, USB_OTG_BOOST_CURRENT, curr_ma);
	if (rc) {
		chg_err("set rvs ocp curr limit %d mA failed, rc=%d\n", curr_ma, rc);
		return rc;
	}

	chg_info("set rvs ocp curr limit %d mA\n", curr_ma);

	return rc;
}

static void *oplus_chg_reverse_chg_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT,
					       reverse_chg_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT,
					       reverse_chg_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP,
					       reverse_chg_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST,
					       reverse_chg_smt_test);
		break;
	case OPLUS_IC_FUNC_GET_REVERSE_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_REVERSE_ENABLE,
					       oplus_chg_get_reverse_enbale);
		break;
	case OPLUS_IC_FUNC_RVS_SET_HIGH_PWR_MODE_EN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RVS_SET_HIGH_PWR_MODE_EN,
			       oplus_chg_set_rvs_high_mode_en);
		break;
	case OPLUS_IC_FUNC_SET_REVERSE_SRC_PDO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_REVERSE_SRC_PDO,
							oplus_chg_set_reverse_boost_pdo);
		break;
	case OPLUS_IC_FUNC_REVERSE_CHG_WDT_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REVERSE_CHG_WDT_ENABLE,
					       oplus_chg_set_wdt_enable);
		break;
	case OPLUS_IC_FUNC_REVERSE_CHG_KICK_WDT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REVERSE_CHG_KICK_WDT,
					       oplus_chg_set_kick_wdt);
		break;
	case OPLUS_IC_FUNC_GET_RVS_CHG_MSG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_RVS_CHG_MSG,
					       oplus_chg_get_rvs_chg_msg_type);
		break;
	case OPLUS_IC_FUNC_GET_SINK_REQ_PDO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_SINK_REQ_PDO,
					       oplus_chg_get_sink_req_pdo);
		break;
	case OPLUS_IC_FUNC_GET_REVERSE_CHG_SVID:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_REVERSE_CHG_SVID,
					       oplus_chg_get_reverse_chg_svid);
		break;
	case OPLUS_IC_FUNC_GET_HIGH_REVERSE_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_HIGH_REVERSE_ENABLE,
					       oplus_chg_get_high_reverse_enbale);
		break;
	case OPLUS_IC_FUNC_SET_NORMAL_REVERSE_HW_OCP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_NORMAL_REVERSE_HW_OCP,
					       oplus_chg_set_normal_rvs_ocp);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq oplus_chg_reverse_chg_virq_table[] = {
	{.virq_id = OPLUS_IC_VIRQ_ERR},
	{.virq_id = OPLUS_IC_VIRQ_ONLINE},
	{.virq_id = OPLUS_IC_VIRQ_OFFLINE},
	{.virq_id = OPLUS_IC_VIRQ_REVERSE_ENABLE},
	{.virq_id = OPLUS_IC_VIRQ_HARD_RESET},
	{.virq_id = OPLUS_IC_VIRQ_SINK_REQ_MSG},
	{.virq_id = OPLUS_IC_VIRQ_HIGH_REVERSE_ENABLE},
};

#define DEVINFO_DATA_NUM	2
#define GAUGE_WAIT_TIMEOUT	3000
static void oplus_gauge_register_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, gauge_register_work.work);
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	struct device_node *child;
	struct oplus_chg_ic_dev *ic_dev = NULL;
	struct oplus_chg_ic_cfg ic_cfg;
	int rc;
	const char *gauge_name[DEVINFO_DATA_NUM];

	if (bcdev->fg_register_flag)
		return;

	bcdev->fg_register_flag = true;
	for_each_child_of_node(bcdev->dev->of_node, child) {
		rc = of_property_read_u32(child, "oplus,ic_type", &ic_type);
		if (rc < 0) {
			chg_err("can't get %s ic type, rc=%d\n", child->name, rc);
			continue;
		}
		rc = of_property_read_u32(child, "oplus,ic_index", &ic_index);
		if (rc < 0) {
			chg_err("can't get %s ic index, rc=%d\n", child->name, rc);
			continue;
		}
		if (ic_type != OPLUS_CHG_IC_GAUGE)
			continue;
		ic_cfg.name = child->name;
		ic_cfg.index = ic_index;
		ic_cfg.type = ic_type;
		ic_cfg.of_node = child;

		rc = oplus_sm8350_get_gauge_devinfo(child, gauge_name);
		if (rc != 0)
			gauge_name[0] = "bq28z610";
		else
			chg_info("gauge_name=%s\n", gauge_name[0]);
		if (oplus_chg_get_voocphy_support(bcdev) == ADSP_VOOCPHY)
			snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "gauge-%s", gauge_name[0]);
		else
			snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "gauge-adsp");
		snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
		ic_cfg.get_func = oplus_chg_8350_gauge_get_func;
		ic_cfg.virq_data = oplus_chg_8350_gauge_virq_table;
		ic_cfg.virq_num = ARRAY_SIZE(oplus_chg_8350_gauge_virq_table);

		ic_dev = devm_oplus_chg_ic_register(bcdev->dev, &ic_cfg);
		if (!ic_dev) {
			rc = -ENODEV;
			chg_err("register %s error\n", child->name);
			continue;
		}
		chg_info("register %s\n", child->name);

		bcdev->gauge_ic = ic_dev;

		of_platform_populate(child, NULL, NULL, bcdev->dev);
	}
}

static int oplus_sm8350_ic_register(struct battery_chg_dev *bcdev)
{
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	struct device_node *child;
	struct oplus_chg_ic_dev *ic_dev = NULL;
	struct oplus_chg_ic_cfg ic_cfg;
	int rc;
	struct device_attribute **attrs;
	struct device_attribute *attr;
	const char *gauge_name[DEVINFO_DATA_NUM];
	int check_fg = 0;

	for_each_child_of_node(bcdev->dev->of_node, child) {
		rc = of_property_read_u32(child, "oplus,ic_type", &ic_type);
		if (rc < 0) {
			chg_err("can't get %s ic type, rc=%d\n", child->name, rc);
			continue;
		}
		rc = of_property_read_u32(child, "oplus,ic_index", &ic_index);
		if (rc < 0) {
			chg_err("can't get %s ic index, rc=%d\n", child->name, rc);
			continue;
		}
		ic_cfg.name = child->name;
		ic_cfg.index = ic_index;
		ic_cfg.type = ic_type;
		ic_cfg.of_node = child;
		switch (ic_type) {
		case OPLUS_CHG_IC_BUCK:
			snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "buck-sm8350");
			snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
			ic_cfg.get_func = oplus_chg_8350_buck_get_func;
			ic_cfg.virq_data = oplus_chg_8350_buck_virq_table;
			ic_cfg.virq_num = ARRAY_SIZE(oplus_chg_8350_buck_virq_table);
			break;
		case OPLUS_CHG_IC_GAUGE:
			if (bcdev->soccp_support) {
				rc = read_property_id(bcdev, &bcdev->oplus_psy, OPLUS_CHECK_FG);
				if (rc < 0) {
					chg_err("read OPLUS_CHECK_FG fail\n");
					check_fg = 0;
				} else {
					check_fg = bcdev->oplus_psy.prop[OPLUS_CHECK_FG];
				}
				if (!check_fg) {
					schedule_delayed_work(&bcdev->gauge_register_work,
						msecs_to_jiffies(GAUGE_WAIT_TIMEOUT));
				} else {
					schedule_delayed_work(&bcdev->gauge_register_work, 0);
				}
				continue;
			}
			rc = oplus_sm8350_get_gauge_devinfo(child, gauge_name);
			if (rc != 0)
				gauge_name[0] = "bq28z610";
			else
				chg_info("gauge_name=%s\n", gauge_name[0]);
			if (oplus_chg_get_voocphy_support(bcdev) == ADSP_VOOCPHY)
				snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "gauge-%s", gauge_name[0]);
			else
				snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "gauge-adsp");
			snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
			ic_cfg.get_func = oplus_chg_8350_gauge_get_func;
			ic_cfg.virq_data = oplus_chg_8350_gauge_virq_table;
			ic_cfg.virq_num = ARRAY_SIZE(oplus_chg_8350_gauge_virq_table);
			break;
		case OPLUS_CHG_IC_CP:
			snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "cp-adsp");
			snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
			ic_cfg.get_func = oplus_chg_adsp_cp_get_func;
			ic_cfg.virq_data = oplus_chg_adsp_cp_virq_table;
			ic_cfg.virq_num = ARRAY_SIZE(oplus_chg_adsp_cp_virq_table);
			break;
		case OPLUS_CHG_IC_MISC:
			snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "misc-adsp_dpdm_switch");
			snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
			ic_cfg.get_func = oplus_chg_adsp_dpdm_switch_get_func;
			ic_cfg.virq_data = oplus_chg_adsp_dpdm_switch_virq_table;
			ic_cfg.virq_num = ARRAY_SIZE(oplus_chg_adsp_dpdm_switch_virq_table);
			break;
		case OPLUS_CHG_IC_PPS:
			snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "pps-adsp");
			snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
			ic_cfg.get_func = oplus_chg_adsp_pps_get_func;
			ic_cfg.virq_data = oplus_chg_adsp_pps_virq_table;
			ic_cfg.virq_num = ARRAY_SIZE(oplus_chg_adsp_pps_virq_table);
			break;
		case OPLUS_CHG_IC_UFCS:
			snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "ufcs-adsp");
			snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
			ic_cfg.get_func = oplus_chg_adsp_ufcs_get_func;
			ic_cfg.virq_data = oplus_chg_adsp_ufcs_virq_table;
			ic_cfg.virq_num = ARRAY_SIZE(oplus_chg_adsp_ufcs_virq_table);
			break;
		case OPLUS_CHG_IC_REVERSE:
			chg_info("reverse_ic register\n");
			snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "reverse-adsp");
			snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
			ic_cfg.get_func = oplus_chg_reverse_chg_get_func;
			ic_cfg.virq_data = oplus_chg_reverse_chg_virq_table;
			ic_cfg.virq_num = ARRAY_SIZE(oplus_chg_reverse_chg_virq_table);
			break;
		default:
			chg_err("not support ic_type(=%d)\n", ic_type);
			continue;
		}
		ic_dev = devm_oplus_chg_ic_register(bcdev->dev, &ic_cfg);
		if (!ic_dev) {
			rc = -ENODEV;
			chg_err("register %s error\n", child->name);
			continue;
		}
		chg_info("register %s\n", child->name);

		switch (ic_dev->type) {
		case OPLUS_CHG_IC_BUCK:
			bcdev->buck_ic = ic_dev;
			break;
		case OPLUS_CHG_IC_GAUGE:
			bcdev->gauge_ic = ic_dev;
			break;
		case OPLUS_CHG_IC_CP:
			bcdev->cp_ic = ic_dev;
			oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_INIT);
			break;
		case OPLUS_CHG_IC_MISC:
			bcdev->misc_ic = ic_dev;
			oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_INIT);
			break;
		case OPLUS_CHG_IC_PPS:
			bcdev->pps_ic = ic_dev;
			oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_INIT);
			break;
		case OPLUS_CHG_IC_UFCS:
			bcdev->ufcs_ic = ic_dev;
			oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_INIT);
			break;
		case OPLUS_CHG_IC_REVERSE:
			bcdev->reverse_chg_ic_dev = ic_dev;
			oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_INIT);
			break;
		default:
			chg_err("not support ic_type(=%d)\n", ic_dev->type);
			continue;
		}

		of_platform_populate(child, NULL, NULL, bcdev->dev);
	}
#ifdef CONFIG_OPLUS_CHG_IC_DEBUG
	attrs = adsp_buck_attributes;
	while ((attr = *attrs++)) {
		rc = device_create_file(bcdev->buck_ic->debug_dev, attr);
		if (rc) {
			chg_err("device_create_file fail!\n");
		}
	}
#endif

	return 0;
}

static int dev_is_pm8550b(struct device *dev, void *name)
{
	if (dev->of_node != NULL && !strcmp(dev->of_node->name, name))
		return 1;

	return 0;
}

struct device *soc_find_pm8550b(struct device *soc_dev)
{
	char *path[] = {"qcom,spmi", "qcom,spmi", "qcom,pm8550b"};
	struct device *dev;
	int i;

	dev = soc_dev;
	for (i = 0; i < (sizeof(path)/sizeof(char *)); i++) {
		dev = device_find_child(dev, path[i], dev_is_pm8550b);
		if (dev == NULL) {
			chg_err("dev %s not found!!", path[i]);
			return NULL;
		} else {
			chg_info("dev %s found!!", path[i]);
		}
	}
	chg_info("pm8550b found!\n");

	return dev;
}

static void oplus_adsp_get_regmap_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct battery_chg_dev *chip = container_of(dwork,
				struct battery_chg_dev, get_regmap_work);
	struct device *dev;
	struct device *soc_dev;
	static int count = 5;
	soc_dev = chip->dev->parent->parent;

	dev = soc_find_pm8550b(soc_dev);
	if (dev == NULL) {
		if (count --) {
			chg_err("pm8550b not found, retry count: %d\n", count);
			schedule_delayed_work(&chip->get_regmap_work, msecs_to_jiffies(1000));
		} else {
			chg_err("pm8550b not found, retry done\n");
		}
		return;
	} else {
		chip->regmap = dev_get_regmap(dev, NULL);
		if (!chip->regmap) {
			chg_err("pm8550b regmap is missing\n");
			return;
		}
	}
}

static void oplus_chg_adsp_plc_status_update_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev =
		container_of(work, struct battery_chg_dev, plc_status_update_work);
	union mms_msg_data data = { 0 };
	int rc;

	rc = oplus_mms_get_item_data(bcdev->plc_topic, PLC_ITEM_STATUS, &data, false);
	if (rc < 0) {
		chg_err("get plc status error, rc=%d\n", rc);
		return;
	}
	(void)oplus_chg_adsp_set_plc_status(bcdev, data.intval);
}

static void oplus_chg_adsp_plc_subs_callback(struct mms_subscribe *subs,
					     enum mms_msg_type type, u32 id, bool sync)
{
	struct battery_chg_dev *bcdev = subs->priv_data;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case PLC_ITEM_STATUS:
			schedule_work(&bcdev->plc_status_update_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_chg_adsp_subscribe_plc_topic(struct oplus_mms *topic,
					       void *prv_data)
{
	struct battery_chg_dev *bcdev = prv_data;
	union mms_msg_data data = { 0 };
	int rc;

	bcdev->plc_topic = topic;
	bcdev->plc_subs = oplus_mms_subscribe(bcdev->plc_topic, bcdev,
					      oplus_chg_adsp_plc_subs_callback,
					      "adsp");
	if (IS_ERR_OR_NULL(bcdev->plc_subs)) {
		chg_err("subscribe plc topic error, rc=%ld\n",
			PTR_ERR(bcdev->plc_subs));
		return;
	}

	rc = oplus_mms_get_item_data(bcdev->plc_topic, PLC_ITEM_STATUS, &data, true);
	if (rc >= 0)
		(void)oplus_chg_adsp_set_plc_status(bcdev, data.intval);
}

static int oplus_chg_adsp_set_wired_usb_status(struct battery_chg_dev *bcdev, u32 status)
{
	struct psy_state *pst = NULL;
	int rc = 0;

	if (!bcdev)
		return -EINVAL;

	pst = &bcdev->psy_list[PSY_TYPE_USB];
	if (!bcdev->soccp_support){
		rc = write_property_id(bcdev, pst, USB_SET_WIRED_USB_STATUS, status);
		chg_info("USB_SET_WIRED_USB_STATUS=%d, rc=%d\n", status,  rc);
	} else {
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_USB_SET_WIRED_USB_STATUS, status);
		chg_info("USB_SET_WIRED_USB_STATUS=%d, rc=%d\n", status,  rc);
	}
	if (rc < 0) {
		chg_err("set_wired_usb_status %d error, rc = %d\n", status, rc);
		return rc;
	}
	return 0;
}

static void oplus_chg_adsp_wired_subs_callback(struct mms_subscribe *subs,
					     enum mms_msg_type type, u32 id, bool sync)
{
	struct battery_chg_dev *bcdev = subs->priv_data;
	int rc = 0;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_TIMER:
		break;
	case MSG_TYPE_ITEM:
		switch (id) {
		case WIRED_ITEM_USB_STATUS:
			rc = oplus_mms_get_item_data(bcdev->wired_topic, id, &data, false);
			if (rc >= 0)
				oplus_chg_adsp_set_wired_usb_status(bcdev, data.intval);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_chg_adsp_subscribe_wired_topic(struct oplus_mms *topic,
					       void *prv_data)
{
	struct battery_chg_dev *bcdev = prv_data;
	union mms_msg_data data = { 0 };
	int rc = 0;

	bcdev->wired_topic = topic;
	bcdev->wired_subs = oplus_mms_subscribe(bcdev->wired_topic, bcdev,
					      oplus_chg_adsp_wired_subs_callback,
					      "adsp");
	if (IS_ERR_OR_NULL(bcdev->wired_subs)) {
		chg_err("subscribe wired topic error, rc=%ld\n",
			PTR_ERR(bcdev->wired_subs));
		return;
	}

	rc = oplus_mms_get_item_data(bcdev->wired_topic, WIRED_ITEM_USB_STATUS, &data, true);
	if (rc >= 0)
		oplus_chg_adsp_set_wired_usb_status(bcdev, data.intval);
}
#endif /* OPLUS_FEATURE_CHG_BASIC */

static void oplus_update_common_charge_flag_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
		struct battery_chg_dev, update_common_charge_flag_work.work);
	int rc = 0;

	if (bcdev->soccp_support &&
		!!(oplus_chg_get_nvid_support_flags() & BIT(COMMON_CHG_SUPPORT_REGION))) {
		chg_info("update common charge flag\n");
		rc = write_property_id(bcdev, &bcdev->oplus_psy, OPLUS_SET_COMMON_CHG, true);
		if (rc)
			chg_err("update common charge flag fail, rc=%d\n", rc);
	}
}

static void battery_chg_init_works(struct battery_chg_dev *bcdev)
{
	INIT_WORK(&bcdev->subsys_up_work, battery_chg_subsys_up_work);
	INIT_WORK(&bcdev->usb_type_work, battery_chg_update_usb_type_work);
	INIT_WORK(&bcdev->plc_status_update_work, oplus_chg_adsp_plc_status_update_work);
#ifdef OPLUS_FEATURE_CHG_BASIC
	INIT_WORK(&bcdev->gauge_ra0_check_work, oplus_gauge_ra0_check_work);
	INIT_WORK(&bcdev->gauge_imp_check_work, oplus_gauge_imp_check_work);
	INIT_WORK(&bcdev->gauge_delta_voltage_check_work, oplus_gauge_delta_voltage_check_work);
	INIT_WORK(&bcdev->gauge_cali_track_by_plug_work, oplus_plat_gauge_cali_track_by_plug_work);
	INIT_WORK(&bcdev->gauge_cali_track_by_full_work, oplus_plat_gauge_cali_track_by_full_work);
	INIT_DELAYED_WORK(&bcdev->adsp_voocphy_status_work, oplus_adsp_voocphy_status_func);
	INIT_DELAYED_WORK(&bcdev->unsuspend_usb_work, oplus_unsuspend_usb_work);
	INIT_DELAYED_WORK(&bcdev->otg_init_work, oplus_otg_init_status_func);
	INIT_DELAYED_WORK(&bcdev->cid_status_change_work, oplus_cid_status_change_work);
	INIT_DELAYED_WORK(&bcdev->adsp_crash_recover_work, oplus_adsp_crash_recover_func);
	INIT_DELAYED_WORK(&bcdev->crash_track_work, oplus_crash_track_work);
	INIT_DELAYED_WORK(&bcdev->voocphy_enable_check_work, oplus_voocphy_enable_check_func);
	INIT_DELAYED_WORK(&bcdev->otg_vbus_enable_work, otg_notification_handler);
	INIT_DELAYED_WORK(&bcdev->hvdcp_disable_work, oplus_hvdcp_disable_work);
	INIT_DELAYED_WORK(&bcdev->pd_only_check_work, oplus_pd_only_check_work);
	INIT_DELAYED_WORK(&bcdev->otg_status_check_work, oplus_otg_status_check_work);
	INIT_DELAYED_WORK(&bcdev->vbus_adc_enable_work, oplus_vbus_enable_adc_work);
	INIT_DELAYED_WORK(&bcdev->oem_lcm_en_check_work, oplus_oem_lcm_en_check_work);
	INIT_DELAYED_WORK(&bcdev->voocphy_err_work, oplus_voocphy_err_work);
	INIT_DELAYED_WORK(&bcdev->ctrl_lcm_frequency, oplus_chg_ctrl_lcm_work);
	INIT_DELAYED_WORK(&bcdev->plugin_irq_work, oplus_plugin_irq_work);
	INIT_DELAYED_WORK(&bcdev->recheck_input_current_work, oplus_recheck_input_current_work);
	INIT_DELAYED_WORK(&bcdev->vbus_collapse_rerun_icl_work, oplus_vbus_collapse_rerun_icl_work);
	INIT_DELAYED_WORK(&bcdev->check_adspfg_status, oplus_check_adspfg_status_work);
	INIT_DELAYED_WORK(&bcdev->publish_close_cp_item_work, oplus_publish_close_cp_item_work);
	INIT_DELAYED_WORK(&bcdev->hboost_notify_work, oplus_hboost_notify_work);
	INIT_DELAYED_WORK(&bcdev->sourcecap_done_work, oplus_sourcecap_done_work);
	INIT_DELAYED_WORK(&bcdev->pdo_update_work, oplus_pdo_update_work);
	INIT_DELAYED_WORK(&bcdev->sourcecap_suspend_recovery_work, oplus_sourcecap_suspend_recovery_work);
	INIT_DELAYED_WORK(&bcdev->update_pd_svooc_work, oplus_update_pd_svooc_work);
	INIT_DELAYED_WORK(&bcdev->iterm_timeout_work, oplus_iterm_timeout_work);
	INIT_DELAYED_WORK(&bcdev->request_qos_work, oplus_request_qos_work);
	INIT_DELAYED_WORK(&bcdev->release_qos_work, oplus_release_qos_work);
	INIT_WORK(&bcdev->wired_otg_enable_work, oplus_wired_otg_enable_work);
	INIT_DELAYED_WORK(&bcdev->ufcs_reset_work, oplus_ufcs_reset_work);
	INIT_DELAYED_WORK(&bcdev->gauge_register_work, oplus_gauge_register_work);
	INIT_DELAYED_WORK(&bcdev->update_common_charge_flag_work, oplus_update_common_charge_flag_work);
	INIT_DELAYED_WORK(&bcdev->check_abnormal_usbin_status_work, oplus_check_abnormal_usbin_status_work);
	INIT_DELAYED_WORK(&bcdev->source_pdo_check_work, oplus_source_pdo_check_work);
	INIT_DELAYED_WORK(&bcdev->update_pd_completed_work, oplus_update_pd_completed_work);
	INIT_DELAYED_WORK(&bcdev->crash_timeout_work, oplus_crash_timeout_work);
	INIT_DELAYED_WORK(&bcdev->vchg_trig_work, oplus_vchg_trig_work);
#endif
}

static int battery_chg_probe(struct platform_device *pdev)
{
	struct battery_chg_dev *bcdev;
	struct device *dev = &pdev->dev;
	struct pmic_glink_client_data client_data = { };
	const char *chg_name;
	int rc, i;

#ifdef OPLUS_FEATURE_CHG_BASIC
	chg_info("battery_chg_probe start...\n");
#endif /*OPLUS_FEATURE_CHG_BASIC*/

	bcdev = devm_kzalloc(&pdev->dev, sizeof(*bcdev), GFP_KERNEL);
	if (!bcdev)
		return -ENOMEM;

#ifdef OPLUS_FEATURE_CHG_BASIC
	g_bcdev = bcdev;
	bcdev->rerun_max = 3;
	bcdev->hvdcp_detect_time = 0;
	bcdev->hvdcp_detach_time = 0;
	bcdev->hvdcp_detect_ok = false;
	bcdev->hvdcp_disable = false;
	bcdev->voocphy_err_check = false;
	bcdev->usb_in_status = 0;
	bcdev->ufcs_test_mode = false;
	bcdev->ufcs_power_ready = false;
	bcdev->ufcs_handshake_ok = false;
	bcdev->ufcs_key_to_adsp_done = false;
	bcdev->bc12_completed = false;
	bcdev->ufcs_exiting = false;
	bcdev->read_by_reg = 0;
	bcdev->adspfg_i2c_reset_processing = false;
	bcdev->adspfg_i2c_reset_notify_done = false;
	bcdev->pd_chg_volt = OPLUS_PD_5V;
	bcdev->pre_info = &pre_info;

	bcdev->oplus_psy.map = oplus_prop_map;
	bcdev->oplus_psy.prop_count = OPLUS_PROPERTY_MAX;
	bcdev->oplus_psy.opcode_get = OPLUS_OPCODE_GET_REQ;
	bcdev->oplus_psy.opcode_set = OPLUS_OPCODE_SET_REQ;
	bcdev->oplus_psy.prop = devm_kcalloc(&pdev->dev, bcdev->oplus_psy.prop_count,
					sizeof(u32), GFP_KERNEL);
	if (!bcdev->oplus_psy.prop)
		return -ENOMEM;
#endif

	bcdev->psy_list[PSY_TYPE_BATTERY].map = battery_prop_map;
	bcdev->psy_list[PSY_TYPE_BATTERY].prop_count = BATT_PROP_MAX;
	bcdev->psy_list[PSY_TYPE_BATTERY].opcode_get = BC_BATTERY_STATUS_GET;
	bcdev->psy_list[PSY_TYPE_BATTERY].opcode_set = BC_BATTERY_STATUS_SET;
	bcdev->psy_list[PSY_TYPE_USB].map = usb_prop_map;
	bcdev->psy_list[PSY_TYPE_USB].prop_count = USB_PROP_MAX;
	bcdev->psy_list[PSY_TYPE_USB].opcode_get = BC_USB_STATUS_GET;
	bcdev->psy_list[PSY_TYPE_USB].opcode_set = BC_USB_STATUS_SET;
	bcdev->psy_list[PSY_TYPE_WLS].map = wls_prop_map;
	bcdev->psy_list[PSY_TYPE_WLS].prop_count = WLS_PROP_MAX;
	bcdev->psy_list[PSY_TYPE_WLS].opcode_get = BC_WLS_STATUS_GET;
	bcdev->psy_list[PSY_TYPE_WLS].opcode_set = BC_WLS_STATUS_SET;

	for (i = 0; i < PSY_TYPE_MAX; i++) {
		bcdev->psy_list[i].prop =
			devm_kcalloc(&pdev->dev, bcdev->psy_list[i].prop_count,
					sizeof(u32), GFP_KERNEL);
		if (!bcdev->psy_list[i].prop)
			return -ENOMEM;
	}

	bcdev->psy_list[PSY_TYPE_BATTERY].model =
		devm_kzalloc(&pdev->dev, MAX_STR_LEN, GFP_KERNEL);
	if (!bcdev->psy_list[PSY_TYPE_BATTERY].model)
		return -ENOMEM;

	mutex_init(&bcdev->rw_lock);
#ifdef OPLUS_FEATURE_CHG_BASIC
	mutex_init(&bcdev->oplus_custom_gpio.pinctrl_mutex);
	mutex_init(&bcdev->read_buffer_lock);
	init_completion(&bcdev->oem_read_ack);
	mutex_init(&bcdev->bcc_read_buffer_lock);
	init_completion(&bcdev->bcc_read_ack);
	mutex_init(&bcdev->chg_en_lock);
	mutex_init(&bcdev->pps_read_buffer_lock);
	init_completion(&bcdev->pps_read_ack);
	mutex_init(&bcdev->ufcs_read_buffer_lock);
	init_completion(&bcdev->ufcs_read_ack);
	mutex_init(&bcdev->rvs_chg_msg_lock);
	init_completion(&bcdev->rvs_chg_msg_ack);
	mutex_init(&bcdev->ap_read_buffer_lock);
	mutex_init(&bcdev->pre_info_lock);
	mutex_init(&bcdev->cur_info_lock);
	for (i = 0; i< AP_MESSAGE_MAX_SIZE; i++)
		init_completion(&bcdev->ap_read_ack[i]);
	mutex_init(&bcdev->ap_write_buffer_lock);
	mutex_init(&bcdev->vdm_info_lock);
	init_completion(&bcdev->ap_write_ack);
	bcdev->ap_read_buffer_dump = devm_kzalloc(&pdev->dev, sizeof(*bcdev->ap_read_buffer_dump), GFP_KERNEL);
	if (!bcdev->ap_read_buffer_dump)
		return -ENOMEM;
#endif
	init_completion(&bcdev->ack);
	init_completion(&bcdev->fw_buf_ack);
	init_completion(&bcdev->fw_update_ack);
	battery_chg_init_works(bcdev);
	atomic_set(&bcdev->state, PMIC_GLINK_STATE_UP);
	bcdev->dev = dev;
	bcdev->gauge_data_initialized = false;

	client_data.id = MSG_OWNER_BC;
	client_data.name = "battery_charger";
	client_data.msg_cb = battery_chg_callback;
	client_data.priv = bcdev;
	client_data.state_cb = battery_chg_state_cb;

	bcdev->client = pmic_glink_register_client(dev, &client_data);
	if (IS_ERR(bcdev->client)) {
		rc = PTR_ERR(bcdev->client);
		if (rc != -EPROBE_DEFER)
			dev_err(dev, "Error in registering with pmic_glink %d\n",
				rc);
		return rc;
	}

	bcdev->reboot_notifier.notifier_call = battery_chg_ship_mode;
	bcdev->reboot_notifier.priority = 255;
	register_reboot_notifier(&bcdev->reboot_notifier);
#ifdef OPLUS_FEATURE_CHG_BASIC
	oplus_ap_init_adsp_gague(bcdev);
#endif

	rc = battery_chg_parse_dt(bcdev);
	if (rc < 0)
		goto error;

	bcdev->restrict_fcc_ua = DEFAULT_RESTRICT_FCC_UA;
	platform_set_drvdata(pdev, bcdev);
	bcdev->fake_soc = -EINVAL;
#ifndef OPLUS_FEATURE_CHG_BASIC
	rc = battery_chg_init_psy(bcdev);
	if (rc < 0)
		goto error;
#endif

	bcdev->battery_class.name = "qcom-battery";
	bcdev->battery_class.class_groups = battery_class_groups;
	rc = class_register(&bcdev->battery_class);
	if (rc < 0) {
		chg_err("Failed to create battery_class rc=%d\n", rc);
		goto error;
	}

#ifdef OPLUS_FEATURE_CHG_BASIC
	oplus_subboard_temp_iio_init(bcdev);
	oplus_shaft_fpc_iio_init(bcdev);
	oplus_chg_parse_custom_dt(bcdev);
	oplus_chg_parse_custom_wls_dt(bcdev);

	bcdev->ssr_nb.notifier_call = oplus_chg_ssr_notifier_cb;
	if (!bcdev->soccp_support)
		bcdev->subsys_handle = qcom_register_ssr_notifier("lpass",
							  &bcdev->ssr_nb);
	else
		bcdev->subsys_handle = qcom_register_ssr_notifier("soccp",
							  &bcdev->ssr_nb);
	if (IS_ERR(bcdev->subsys_handle)) {
		rc = PTR_ERR(bcdev->subsys_handle);
		pr_err("Failed in qcom_register_ssr_notifier rc=%d\n", rc);
	}
#ifdef OPLUS_CHG_UNDEF
	main_psy = power_supply_get_by_name("main");
	if (main_psy) {
		pval.intval = 1000 * oplus_chg_get_fv(oplus_chip);
		power_supply_set_property(main_psy,
				POWER_SUPPLY_PROP_VOLTAGE_MAX,
				&pval);
		pval.intval = 1000 * oplus_chg_get_charging_current(oplus_chip);
		power_supply_set_property(main_psy,
				POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
				&pval);
	}
#endif
	/* oplus_chg_wake_update_work(); */

	if (oplus_vchg_trig_is_support() == true) {
		schedule_delayed_work(&bcdev->vchg_trig_work, msecs_to_jiffies(3000));
		oplus_vchg_trig_irq_register(bcdev);
	}
#endif /*OPLUS_FEATURE_CHG_BASIC*/

	battery_chg_add_debugfs(bcdev);
	battery_chg_notify_enable(bcdev);
	device_init_wakeup(bcdev->dev, true);
#ifdef OPLUS_FEATURE_CHG_BASIC
	oplus_voocphy_enable(bcdev, false);
	schedule_delayed_work(&bcdev->otg_init_work, 0);

	init_debug_reg_proc(bcdev);

	rc = of_property_read_u32(dev->of_node, "oplus,batt_num", &bcdev->batt_num);
	if (rc < 0) {
		chg_err("can't get oplus,batt_num, rc=%d\n", rc);
		bcdev->batt_num = 1;
	}

	bcdev->voocphy_bidirect_cp_support = of_property_read_bool(dev->of_node,
		"oplus,voocphy_bidirect_cp_support");
	chg_info("%s support voocphy bidirect cp\n",
		 bcdev->voocphy_bidirect_cp_support ? "" : "no");

	rc = of_property_read_string(dev->of_node, "oplus,chg_ops",  &chg_name);
	if (rc >= 0) {
		if (strncmp(chg_name, "plat-pmic", 64) == 0)
			bcdev->is_external_chg = false;
		else
			bcdev->is_external_chg = true;
	} else {
		chg_info("can't get oplus,chg_ops, rc=%d\n", rc);
		bcdev->is_external_chg = false;
	}

	rc = oplus_sm8350_ic_register(bcdev);
	if (rc < 0)
		goto error;

	oplus_mms_wait_topic("plc", oplus_chg_adsp_subscribe_plc_topic, bcdev);
	oplus_mms_wait_topic("wired", oplus_chg_adsp_subscribe_wired_topic, bcdev);

	INIT_DELAYED_WORK(&bcdev->get_regmap_work, oplus_adsp_get_regmap_work);
	schedule_delayed_work(&bcdev->get_regmap_work, 0);
	mod_delayed_work(system_highpri_wq, &bcdev->ctrl_lcm_frequency, 0);

	INIT_DELAYED_WORK(&bcdev->get_manu_battinfo_work, oplus_get_manu_battinfo_work);
	schedule_delayed_work(&bcdev->get_manu_battinfo_work, OPLUS_GET_BATT_INFO_FROM_ADSP_INTERVAL);
	schedule_delayed_work(&bcdev->hboost_notify_work, OPLUS_HBOOST_NOTIFY_INTERVAL);
	if (bcdev->soccp_support) {
		schedule_delayed_work(&bcdev->update_pd_svooc_work, 0);
		schedule_delayed_work(&bcdev->plugin_irq_work, 0);
		schedule_delayed_work(&bcdev->update_pd_completed_work, 0);
		schedule_delayed_work(&bcdev->update_common_charge_flag_work, 0);
		schedule_delayed_work(&bcdev->pdo_update_work, 0);
	}

	chg_info("battery_chg_probe end...\n");
#endif
	return 0;
error:
	pmic_glink_unregister_client(bcdev->client);
	unregister_reboot_notifier(&bcdev->reboot_notifier);
	return rc;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void battery_chg_remove(struct platform_device *pdev)
#else
static int battery_chg_remove(struct platform_device *pdev)
#endif
{
	struct battery_chg_dev *bcdev = platform_get_drvdata(pdev);
	int rc;

	if (!IS_ERR_OR_NULL(bcdev->plc_subs))
		oplus_mms_unsubscribe(bcdev->plc_subs);
	if (!IS_ERR_OR_NULL(bcdev->wired_subs))
		oplus_mms_unsubscribe(bcdev->wired_subs);

	device_init_wakeup(bcdev->dev, false);
	debugfs_remove_recursive(bcdev->debugfs_dir);
	class_unregister(&bcdev->battery_class);
	unregister_reboot_notifier(&bcdev->reboot_notifier);
	rc = pmic_glink_unregister_client(bcdev->client);
	if (rc < 0) {
		chg_err("Error unregistering from pmic_glink, rc=%d\n", rc);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
		return rc;
#else
		return;
#endif
	}
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return 0;
#endif
}


static void oplus_adsp_set_cuv_state(struct battery_chg_dev *bcdev, int state)
{
	int cuv_state = 0;
	int cuv_ret;

	if (!bcdev)
		return;

	if (!is_gauge_topic_available(bcdev)) {
		chg_err("gauge_topic is not available.\n");
		return;
	}

	/* set the cuv state to CUV_1*/
	cuv_ret = oplus_gauge_set_cuv_state(bcdev->gauge_topic, state);
	chg_info("cuv_ret = %d \n", cuv_ret);

	if (cuv_ret != -ENOTSUPP) {
		msleep(600);
		cuv_ret = oplus_gauge_get_cuv_state(bcdev->gauge_topic, &cuv_state);
		if (cuv_ret == -ENOTSUPP)
			return;

		chg_info("set cuv_state to %d, after set, get the cuv_state = %d \n",
                          OPLUS_GAUGE_CUV_STATE_1, cuv_state);
		if (cuv_state != state)
			chg_err("set cuv_state to %d failed, current state = %d \n", state, cuv_state);
	}
}

#ifdef OPLUS_FEATURE_CHG_BASIC
static void battery_chg_shutdown(struct platform_device *pdev)
{
	struct battery_chg_dev *bcdev = g_bcdev;

	if (bcdev && bcdev->otg_online) {
		bcdev->otg_online = false;
		oplus_chg_ic_virq_trigger(bcdev->buck_ic, OPLUS_IC_VIRQ_OTG_ENABLE);
	}

	if (bcdev) {
		chg_err("disable voocphy");
		cancel_delayed_work_sync(&bcdev->voocphy_enable_check_work);
		oplus_typec_disable();
		oplus_voocphy_enable(bcdev, false);
		if (bcdev->buck_ic && oplus_chg_get_voocphy_support(bcdev) == ADSP_VOOCPHY) {
			oplus_chg_8350_input_suspend(bcdev->buck_ic, true);
			msleep(1000);
			oplus_chg_8350_input_suspend(bcdev->buck_ic, false);
		}

		/* set the cuv state to CUV_1*/
		oplus_adsp_set_cuv_state(bcdev, OPLUS_GAUGE_CUV_STATE_1);
	}

#ifdef OPLUS_CHG_UNDEF /* TODO */
	if (g_oplus_chip
		&& g_oplus_chip->chg_ops->charger_suspend
		&& g_oplus_chip->chg_ops->charger_unsuspend) {
		g_oplus_chip->chg_ops->charger_suspend();
		msleep(1000);
		g_oplus_chip->chg_ops->charger_unsuspend();
	}

	if (g_oplus_chip && g_oplus_chip->enable_shipmode) {
		smbchg_enter_shipmode(g_oplus_chip);
		msleep(1000);
	}
	if (!is_ext_chg_ops()) {
		bcdev->oem_misc_ctl_data = 0;
		bcdev->oem_misc_ctl_data |= OEM_MISC_CTL_DATA_PAIR(OEM_MISC_CTL_CMD_LCM_25K, false);
		oplus_oem_misc_ctl();
	}
#endif
}
#endif /* OPLUS_FEATURE_CHG_BASIC */

static const struct of_device_id battery_chg_match_table[] = {
	{ .compatible = "oplus,hal_sm8350" },
	{},
};

static struct platform_driver battery_chg_driver = {
	.driver = {
		.name = "qti_battery_charger",
		.of_match_table = battery_chg_match_table,
#ifdef OPLUS_FEATURE_CHG_BASIC
		.pm	= &battery_chg_pm_ops,
#endif
	},
	.probe = battery_chg_probe,
	.remove = battery_chg_remove,
#ifdef OPLUS_FEATURE_CHG_BASIC
	.shutdown = battery_chg_shutdown,
#endif
};

#ifdef OPLUS_FEATURE_CHG_BASIC
static int __init sm8350_chg_init(void)
{
	int ret;

	ret = platform_driver_register(&battery_chg_driver);
	return ret;
}

static void __exit sm8350_chg_exit(void)
{
	platform_driver_unregister(&battery_chg_driver);
}

oplus_chg_module_register(sm8350_chg);
#else
module_platform_driver(battery_chg_driver);
#endif

MODULE_DESCRIPTION("QTI Glink battery charger driver");
MODULE_LICENSE("GPL v2");
