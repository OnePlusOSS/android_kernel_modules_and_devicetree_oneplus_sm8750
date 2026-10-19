// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2024 . Oplus All rights reserved.
 */

#define pr_fmt(fmt) "[PPS]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/completion.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/sched/clock.h>
#include <linux/debugfs.h>
#include <linux/reboot.h>

#include <oplus_chg.h>
#include <oplus_chg_module.h>
#include <oplus_chg_ic.h>
#include <oplus_mms.h>
#include <oplus_mms_gauge.h>
#include <oplus_mms_wired.h>
#include <oplus_chg_vooc.h>
#include <oplus_chg_voter.h>
#include <oplus_chg_comm.h>
#include <oplus_chg_cpa.h>
#include <oplus_impedance_check.h>
#include <oplus_chg_monitor.h>
#include <oplus_chg_vooc.h>
#include <oplus_mms_wired.h>
#include <oplus_chg_pps.h>
#include <oplus_batt_bal.h>
#include <oplus_chg_state_retention.h>
#include <oplus_chg_plc.h>
#include "monitor/oplus_chg_track.h"

#define PPS_MONITOR_CYCLE_MS		1500
#define PPS_WATCHDOG_TIME_MS		3000
#define PPS_START_DEF_VOL_MV		5500
#define PPS_MONITOR_TIME_MS		500
#define PD_SVOOC_WAIT_MS		300
#define OPLUS_FIXED_PDO_CURR_MA		3000
#define OPLUS_FIXED_PDO_DEF_VOL		5000
#define OPLUS_PPS_REBOOT_NB_PRIORITY	200
#define OPLUS_PPS_UW_MV_TRANSFORM	1000
#define PPS_GET_CP_VIN_DELAY		30
#define PPS_START_DEF_CURR_MA_OPLUS	800
#define PPS_START_DEF_CURR_MA_THIRD	1000
#define PPS_START_DEF_CURR_MA		2000
#define PPS_START_CP_CURR_MA		1000

#define PPS_UPDATE_PDO_TIME		5
#define PPS_UPDATE_FASTCHG_TIME	1
#define PPS_UPDATE_TEMP_TIME		1
#define PPS_UPDATE_IBAT_TIME		1

#define PPS_TEMP_OVER_COUNTS		2
#define PPS_TEMP_WARM_RANGE_THD	10
#define PPS_TEMP_LOW_RANGE_THD		10
#define PPS_COLD_TEMP_RANGE_THD		10

#define PPS_R_AVG_NUM			10
#define PPS_R_ROW_NUM			7

#define FULL_PPS_SYS_MAX		6
#define PPS_ERROR_COUNT_MAX		3

#define PPS_BTB_OVER_TEMP		80
#define BTB_TEMP_OVER_MAX_INPUT_CUR	1000

#define PPS_CONNECT_ERROR_COUNT_LEVEL	3
#define PPS_CONNECT_ERROR_COUNT_MAX_LEVEL	15
#define SUPPORT_THIRD_PPS_POWER		33000
#define WAIT_BC1P2_GET_TYPE		600

#define BOOT_TIME_CNTL_CURR_MS		60000
#define BOOT_TIME_CNTL_CURR_DEB_MS	5000
#define BOOT_SYS_CONSUME_MA		1000
#define BOOT_ADAPTER_CURR_MIN		2500
#define LED_ON_SYS_CONSUME_MA		500
#define LOCAL_T_NS_TO_MS_THD		1000000

#define CP_WATCHDOG_TIMEOUT_5S		5000
#define CP_WATCHDOG_DISABLE		0

enum {
	PPS_BAT_TEMP_NATURAL = 0,
	PPS_BAT_TEMP_HIGH0,
	PPS_BAT_TEMP_HIGH1,
	PPS_BAT_TEMP_HIGH2,
	PPS_BAT_TEMP_HIGH3,
	PPS_BAT_TEMP_HIGH4,
	PPS_BAT_TEMP_HIGH5,
	PPS_BAT_TEMP_LOW0,
	PPS_BAT_TEMP_LOW1,
	PPS_BAT_TEMP_LOW2,
	PPS_BAT_TEMP_LITTLE_COOL,
	PPS_BAT_TEMP_LITTLE_COOL_HIGH,
	PPS_BAT_TEMP_COOL,
	PPS_BAT_TEMP_NORMAL_LOW_PRE,
	PPS_BAT_TEMP_NORMAL_LOW,
	PPS_BAT_TEMP_NORMAL_HIGH,
	PPS_BAT_TEMP_LITTLE_COLD,
	PPS_BAT_TEMP_WARM,
	PPS_BAT_TEMP_EXIT,
	PPS_BAT_TEMP_SWITCH_CURVE,
};

enum {
	PPS_TEMP_RANGE_INIT = 0,
	PPS_TEMP_RANGE_LITTLE_COLD, /* 0 ~ 5 */
	PPS_TEMP_RANGE_COOL, /* 5 ~ 12 */
	PPS_TEMP_RANGE_LITTLE_COOL, /* 12~16 */
	PPS_TEMP_RANGE_LITTLE_COOL_HIGH,
	PPS_TEMP_RANGE_NORMAL_LOW_PRE,
	PPS_TEMP_RANGE_NORMAL_LOW, /* 16~25 */
	PPS_TEMP_RANGE_NORMAL_HIGH, /* 25~43 */
	PPS_TEMP_RANGE_WARM, /* 43-52 */
	PPS_TEMP_RANGE_NORMAL,
};

enum {
	PPS_LOW_CURR_FULL_CURVE_TEMP_LITTLE_COOL,
	PPS_LOW_CURR_FULL_CURVE_TEMP_NORMAL_LOW,
	PPS_LOW_CURR_FULL_CURVE_TEMP_NORMAL_HIGH,
	PPS_LOW_CURR_FULL_CURVE_TEMP_MAX,
};

struct oplus_pps_config {
	unsigned int target_vbus_mv;
	unsigned int pps_target_curr_max_ma;
	int curr_max_ma;
	uint8_t *curve_strategy_name;
	int curr_max_ma_percent_75;
	int curr_max_ma_percent_85;
	int pps_full_recheck_temp;
};

struct oplus_pps_timer {
	struct timespec pdo_timer;
	struct timespec fastchg_timer;
	struct timespec temp_timer;
	unsigned long ibat_jiffies;
	int pps_max_time_ms;
	unsigned long monitor_jiffies;
};

struct pps_protection_counts {
	int cool_fw;
	int warm_fw;
	int sw_full;
	int hw_full;
	int low_curr_full;
	int ibat_low;
	int ibat_high;
	int curr_over;
	int curr_abnormal;
	int btb_high;
	int tbatt_over;
	int tfg_over;
	int output_low;
	int ibus_over;
	int pps_recovery;
	int req_vol_over;
};

struct oplus_pps_limits {
	int default_pps_normal_high_temp;
	int default_pps_little_cool_temp;
	int default_pps_little_cool_high_temp;
	int default_pps_cool_temp;
	int default_pps_little_cold_temp;
	int default_pps_normal_low_pre_temp;
	int default_pps_normal_low_temp;
	int pps_warm_allow_vol;
	int pps_warm_allow_soc;

	int pps_batt_over_low_temp;
	int pps_low_temp;
	int pps_little_cold_temp;
	int pps_cool_temp;
	int pps_little_cool_temp;
	int pps_little_cool_high_temp;
	int pps_normal_low_pre_temp;
	int pps_normal_low_temp;
	int pps_normal_high_temp;
	int pps_batt_over_high_temp;
	int pps_strategy_temp_num;

	int pps_strategy_soc_over_low;
	int pps_strategy_soc_min;
	int pps_strategy_soc_low;
	int pps_strategy_soc_mid_low;
	int pps_strategy_soc_mid;
	int pps_strategy_soc_mid_high;
	int pps_strategy_soc_high;
	int pps_strategy_soc_num;

	int pps_strategy_normal_current;
	int pps_strategy_batt_high_temp0;
	int pps_strategy_batt_high_temp1;
	int pps_strategy_batt_high_temp2;
	int pps_strategy_batt_low_temp2;
	int pps_strategy_batt_low_temp1;
	int pps_strategy_batt_low_temp0;
	int pps_strategy_high_current0;
	int pps_strategy_high_current1;
	int pps_strategy_high_current2;
	int pps_strategy_low_current2;
	int pps_strategy_low_current1;
	int pps_strategy_low_current0;

	int pps_low_curr_full_cool_temp;
	int pps_low_curr_full_little_cool_temp;
	int pps_low_curr_full_normal_low_temp;
	int pps_low_curr_full_normal_high_temp;

	int pps_over_high_or_low_current;
	int pps_strategy_change_count;
	int pps_full_cool_sw_vbat;
	int pps_full_normal_sw_vbat;
	int pps_full_normal_hw_vbat;
	int pps_full_warm_vbat;
	int pps_full_cool_sw_vbat_third;
	int pps_full_normal_sw_vbat_third;
	int pps_full_normal_hw_vbat_third;
	int pps_timeout_third;
	int pps_timeout_oplus;
	int pps_ibat_over_third;
	int pps_ibat_over_oplus;
};

struct pps_full_curve {
	unsigned int iterm;
	unsigned int vterm;
	bool exit;
};

struct pps_full_curves_temp {
	struct pps_full_curve full_curves[FULL_PPS_SYS_MAX];
	int full_curve_num;
};

#define PPS_CMD_BUF_SIZE		128
#define PPS_GET_AUTH_DATA_TIMEOUT_MS	1000
enum pps_dev_cmd_type {
	PPS_DEV_CMD_GET_AUTH_DATA,
};

struct pps_dev_cmd {
	unsigned int cmd;
	unsigned int data_size;
	unsigned char data_buf[PPS_CMD_BUF_SIZE];
};

enum pps_vbus_check_status {
	PPS_VBUS_CHECK_INVALID = 0,
	PPS_VBUS_NEED_CHECK_UP,
	PPS_VBUS_NEED_CHECK_DOWN,
	PPS_VBUS_CHECK_DONE,
};

enum exit_pps_flag_status {
	EXIT_PPS_FALSE,
	EXIT_HIGH_PPS,
	EXIT_THIRD_PPS,
};

struct oplus_pps {
	struct device *dev;
	struct oplus_mms *err_topic;
	struct mms_subscribe *err_subs;
	struct oplus_mms *pps_topic;
	struct oplus_mms *cpa_topic;
	struct mms_subscribe *cpa_subs;
	struct oplus_mms *wired_topic;
	struct mms_subscribe *wired_subs;
	struct oplus_mms *comm_topic;
	struct mms_subscribe *comm_subs;
	struct oplus_mms *gauge_topic;
	struct mms_subscribe *gauge_subs;
	struct oplus_mms *batt_bal_topic;
	struct mms_subscribe *batt_bal_subs;
	struct oplus_mms *retention_topic;
	struct mms_subscribe *retention_subs;
	struct oplus_mms *plc_topic;
	struct mms_subscribe *plc_subs;
	struct oplus_chg_ic_dev *pps_ic;
	struct oplus_chg_ic_dev *cp_ic;
	struct oplus_chg_ic_dev *dpdm_switch;

	struct notifier_block nb;

	struct votable *pps_curr_votable;
	struct votable *pps_disable_votable;
	struct votable *pps_not_allow_votable;
	struct votable *wired_suspend_votable;
	struct votable *chg_disable_votable;
	struct votable *pps_boot_votable;
	struct votable *wired_icl_votable;

	struct delayed_work switch_check_work;
	struct delayed_work monitor_work;
	struct delayed_work current_work;
	struct delayed_work imp_uint_init_work;
	struct delayed_work boot_curr_limit_work;
	struct delayed_work switch_end_recheck_work;
	struct delayed_work retention_state_ready_work;

	struct work_struct wired_online_work;
	struct work_struct type_change_work;
	struct work_struct force_exit_work;
	struct work_struct soft_exit_work;
	struct work_struct gauge_update_work;
	struct work_struct retention_disconnect_work;
	struct work_struct close_cp_work;
	bool process_close_cp_item;
	struct work_struct cp_err_handler_work;
	struct work_struct cp_online_handler_work;
	struct work_struct cp_offline_handler_work;
	struct work_struct set_fcs_icl_work;

	wait_queue_head_t read_wq;
	struct miscdevice misc_dev;
	struct mutex read_lock;
	struct mutex cmd_data_lock;
	struct completion cmd_ack;
	struct completion pd_svooc_wait_ack;
	struct pps_dev_cmd cmd;
	bool cmd_data_ok;

	struct oplus_chg_strategy *oplus_curve_strategy;
	struct oplus_chg_strategy *third_curve_strategy;
	struct oplus_chg_strategy *strategy;
	struct oplus_chg_strategy *vfa_strategy;
	struct oplus_chg_strategy *ccd_strategy;
	struct mutex ccd_lock;

	struct oplus_chg_strategy *oplus_lcf_strategy;
	struct oplus_chg_strategy *third_lcf_strategy;

	struct oplus_chg_strategy *temperature_strategy;
	uint8_t *temperature_strategy_name;
	uint8_t *temperature_strategy_data;
	uint32_t temperature_strategy_data_size;

	struct oplus_impedance_node *input_imp_node;
	struct oplus_impedance_unit *imp_uint;

	struct oplus_plc_protocol *opp;

	struct oplus_pps_config config;
	struct oplus_pps_timer timer;
	struct pps_protection_counts count;
	struct oplus_pps_limits limits;

	struct pps_full_curves_temp low_curr_full_curves_temp[PPS_LOW_CURR_FULL_CURVE_TEMP_MAX];

	u64 dev_info;
	u32 pps_status_info;
	pps_msg_data pdo[PPS_PDO_MAX];
	int pdo_num;
	bool pps_online;
	bool pps_online_keep;
	bool pps_charging;
	bool enable_pps_reboot_notifier;
	bool pps_reboot_nb_registered;
	struct notifier_block pps_reboot_nb;
	bool oplus_pps_adapter;
	bool pps_disable;
	bool pps_not_allow;
	enum oplus_cp_work_mode cp_work_mode;
	int last_target_curr_ma;
	int target_curr_ma;
	int target_vbus_mv;
	int curr_set_ma;
	int vol_set_mv;
	int bcc_max_curr;
	int bcc_min_curr;
	int bcc_exit_curr;
	int pps_connect_error_count;
	int pdo_set_error_count;
	enum exit_pps_flag_status retention_exit_pps_flag;
	enum oplus_chg_protocol_type cpa_current_type;
	int rmos_mohm;
	int cool_down;
	int error_count;
	int cp_ratio;

	bool wired_online;
	bool irq_plugin;
	bool disconnect_change;
	bool retention_state;
	bool retention_oplus_adapter;
	bool retention_state_ready;
	bool retention_ready_for_switch_end;
	bool led_on;
	bool need_check_current;
	bool mos_on_check;
	bool batt_hmac;
	bool batt_auth;
	int wired_type;
	bool support_cp_ibus;
	bool support_pps_status;
	int pps_curr_ma_from_pps_status;
	atomic_t cp_offline;

	int ui_soc;
	int shell_temp;
	enum oplus_temp_region batt_temp_region;
	bool shell_temp_ready;
	int adapter_max_curr;
	int boot_time;
	int allow_check_soc;

	int pps_fastchg_batt_temp_status;
	int pps_temp_cur_range;
	int temp_region_cnt;
	int pps_cool_full_temp_range;
	int pps_low_curr_full_temp_status;
	bool quit_pps_protocol;
	int batt_bal_curr_limit;
	bool pdsvooc_id_adapter;
	bool request_vbus_too_low_flag;
	bool chg_ctrl_by_sale_mode;
	int delta_vbus[PPS_TEMP_RANGE_NORMAL + 1];
	bool enable_pps_status;
	bool lift_vbus_use_cpvout;
	int plc_status;
	unsigned int usb_status;

	struct dentry *debugfs_pps;
	int debug_force_pps_err;
	long pps_online_time;
	bool fcl_support;
	bool ss_check;
	bool fcl_trigger;
};

struct current_level {
	int level;
	int curr_ma;
};

static const struct current_level g_pps_current_table[] = {
	{ 1, 1500 },   { 2, 1500 },   { 3, 2000 },   { 4, 2500 },   { 5, 3000 },  { 6, 3500 },   { 7, 4000 },
	{ 8, 4500 },   { 9, 5000 },   { 10, 5500 },  { 11, 6000 },  { 12, 6300 }, { 13, 6500 },  { 14, 7000 },
	{ 15, 7300 },  { 16, 8000 },  { 17, 8500 },  { 18, 9000 },  { 19, 9500 }, { 20, 10000 },  { 21, 10500 },
	{ 22, 11000 },  { 23, 11500 },  { 24, 12000 },  { 25, 12500 },  { 26, 13000 }, { 27, 13500 }, { 28, 14000 },
	{ 29, 14500 }, { 30, 15000 },
};

static const int pps_cool_down_oplus_curve[] = {
	15000, 1500,  1500,  2000,  2500,  3000,  3500,  4000,  4500,  5000,
	5500,  6000,  6300,  6500,  7000,  7300,  8000,  8500,  9000,  9500,
	10000, 10500, 11000, 11500, 12000, 12500, 13000, 13500, 14000, 14500, 15000
};

/* cp  batt curr from 3.6 spec table, at last it will convert to ibus and sent to cp voophy */
static const int pps_cp_cool_down_oplus_curve[] = {
	20000, 2000, 2000, 2400, 3000, 3400, 4000, 4400, 5000, 5400, 6000, 6400, 7000, 7400, 8000,
	9000, 10000, 11000, 12000, 12600, 13000, 14000, 15000, 16000, 17000, 18000, 19000, 20000
};

static const struct current_level g_pps_cp_current_table[] = {
	{ 1, 2000 },   { 2, 2000 },   { 3, 2400 },   { 4, 3000 },   { 5, 3400 },   { 6, 4000 },  { 7, 4400 },
	{ 8, 5000 },   { 9, 5400 },   { 10, 6000 },  { 11, 6400 },  { 12, 7000 },  { 13, 7400 }, { 14, 8000 },
	{ 15, 9000 },  { 16, 10000 }, { 17, 11000 }, { 18, 12000 }, { 19, 12600 }, { 20, 13000 }, { 21, 14000 },
	{ 22, 15000 }, { 23, 16000 }, { 24, 17000 }, { 25, 18000 }, { 26, 19000 }, { 27, 20000 },
};

static int oplus_pps_temp_cur_range_init(struct oplus_pps *chip);
static void oplus_pps_reset_temp_range(struct oplus_pps *chip);
static int oplus_pps_reboot_notify_call(struct notifier_block *nb, unsigned long action, void *data);
static void oplus_pps_reboot_nb_register(struct oplus_pps *chip);
static void oplus_pps_reboot_nb_unregister(struct oplus_pps *chip);

__maybe_unused static bool
is_disable_charger_vatable_available(struct oplus_pps *chip)
{
	if (!chip->chg_disable_votable)
		chip->chg_disable_votable = find_votable("WIRED_CHARGING_DISABLE");
	return !!chip->chg_disable_votable;
}

__maybe_unused static bool
is_wired_suspend_votable_available(struct oplus_pps *chip)
{
	if (!chip->wired_suspend_votable)
		chip->wired_suspend_votable = find_votable("WIRED_CHARGE_SUSPEND");
	return !!chip->wired_suspend_votable;
}

__maybe_unused static bool
is_wired_icl_votable_available(struct oplus_pps *chip)
{
	if (!chip->wired_icl_votable)
		chip->wired_icl_votable = find_votable("WIRED_ICL");
	return !!chip->wired_icl_votable;
}

#define PPS_FCS_ICL1_MA		1500
#define PPS_FCS_ICL2_MA		800
static int oplus_pps_set_fcs_icl_vote(int icl_ma)
{
	struct votable *icl_votable;
	int rc = 0;

	if (!oplus_chg_get_fcs_support_flags())
		return rc;

	icl_votable = find_votable("WIRED_ICL");
	if (!icl_votable) {
		chg_err("WIRED_ICL votable not found\n");
		return -EINVAL;
	}

	if (icl_ma > 0)
		rc = vote(icl_votable, FCS_ICL_VOTER, true, icl_ma, false);
	else
		rc = vote(icl_votable, FCS_ICL_VOTER, false, 0, false);

	if (rc < 0)
		chg_err("set icl error: icl_ma = %d, rc = %d\n", icl_ma, rc);
	else
		chg_info("real icl = %d\n", icl_ma);

	return rc;
}

static void oplus_pps_set_fcs_icl_work(struct work_struct *work)
{
	oplus_pps_set_fcs_icl_vote(0);
}

static void oplus_pps_clear_fcs_icl(struct oplus_pps *chip)
{
	if (!chip)
		return;

	/*
	 * DESIGN REQUIREMENT: this must stay asynchronous.
	 *
	 * Do NOT replace with flush_work()/cancel_work_sync() in exit/reset paths.
	 * Waiting for WIRED_ICL vote completion can be blocked by
	 * COMMON_POWER_CHECK/AICL lock hold and significantly increase protocol
	 * handover latency (UFCS->SVOOC), which violates the timing target.
	 */
	schedule_work(&chip->set_fcs_icl_work);
}

__maybe_unused static bool
is_gauge_topic_available(struct oplus_pps *chip)
{
	if (!chip->gauge_topic)
		chip->gauge_topic = oplus_mms_get_by_name("gauge");
	return !!chip->gauge_topic;
}

__maybe_unused static bool
is_err_topic_available(struct oplus_pps *chip)
{
	if (!chip->err_topic)
		chip->err_topic = oplus_mms_get_by_name("error");
	return !!chip->err_topic;
}

__maybe_unused
static int pps_find_current_to_level(int val, const struct current_level * const table, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		if (table[i].level == val)
			return table[i].curr_ma;
	}
	return 0;
}

static const char *const pps_user_err_type_str[] = {
	[PPS_ERR_BTB_OVER] = "BTB_ERROR",
	[PPS_ERR_TFG_OVER] = "TFG_OVER",
	[PPS_ERR_IBAT_OVER] = "IBAT_OVER",
	[PPS_ERR_REQUEST_VOLT_OVER] = "REQUEST_VOLT_OVER",
};

static const char *oplus_pps_get_err_type_str(enum pps_user_err_type type)
{
	if (type < PPS_ERR_BTB_OVER || type >= PPS_ERR_MAX)
		return "Inval";
	return pps_user_err_type_str[type];
}

static inline long oplus_chg_pps_get_current_time_s(void)
{
	struct timespec ts;
	struct rtc_time tm;

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	tm.tm_year = tm.tm_year + 1900;
	tm.tm_mon = tm.tm_mon + 1;
	return ts.tv_sec;
}

#define PPS_BURIED_POINT_INTERVAL	(60 * 60)
static void oplus_pps_push_err_info(struct oplus_pps *chip, enum pps_user_err_type type, int value)
{
	struct mms_msg *msg;
	int rc;
	char *buf;
	size_t index;
	static long last_time[PPS_ERR_MAX + 1], time, pre_pps_online_time;
	static u32 mask = 0;

	if (!is_err_topic_available(chip)) {
		chg_err("err_topic is NULL\n");
		return;
	}

	if (type >= PPS_ERR_MAX || type < PPS_ERR_BTB_OVER) {
		chg_err("Invalid type value: %d[%d,%d)\n", type, PPS_ERR_BTB_OVER, PPS_ERR_MAX);
		return;
	}

	time = oplus_chg_pps_get_current_time_s();
	if (mask & BIT(type)) {
		if ((time < last_time[type]) ||
		    ((time - last_time[type]) < PPS_BURIED_POINT_INTERVAL) ||
		    (pre_pps_online_time == chip->pps_online_time)) {
			chg_err("Time anomaly: last_time[%d] = %ld, time = %ld, online_time = %ld\n",
				type, last_time[type], time, chip->pps_online_time);
			return;
		} else {
			mask &= 0 << type;
		}
	} else {
		mask |= BIT(type);
	}
	pre_pps_online_time = chip->pps_online_time;
	last_time[type] = time;

	buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (buf == NULL)
		return;

	index = scnprintf(buf, PAGE_SIZE, "$$err_scene@@%s$$err_reason@@%s",
			  OPLUS_CHG_TRACK_SCENE_PPS_ERR, oplus_pps_get_err_type_str(type));
	if (chip->debug_force_pps_err != 0)
		index += scnprintf(buf + index, PAGE_SIZE, "$$value@@NA$$debug@@%d",
				   chip->debug_force_pps_err);
	else
		index += scnprintf(buf + index, PAGE_SIZE, "$$value@@%d$$debug@@%d",
                                   value, chip->debug_force_pps_err);

	msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_PPS, buf);
	kfree(buf);
	rc = oplus_mms_publish_msg_sync(chip->err_topic, msg);
	if (rc < 0) {
		chg_err("publish usbtemp error msg error, rc=%d\n", rc);
		kfree(msg);
	}
	return;
}

static int pps_find_level_to_current(int val, const struct current_level * const table, int len)
{
	int i;
	bool find_out_flag = false;

	for (i = 0; i < len; i++) {
		if (table[i].curr_ma > val) {
			find_out_flag = true;
			break;
		}
		find_out_flag = false;
	}
	if (find_out_flag == true && i != 0)
		return table[i - 1].level;
	return 0;
}

static int oplus_pps_parse_temperature_strategy_init(struct oplus_pps *chip)
{
	int rc = 0;
	struct device_node *node = oplus_get_node_by_type(chip->dev->of_node);

	rc = of_property_read_string(node, "oplus,temperature_strategy_name", (const char **)&chip->temperature_strategy_name);
	if (rc < 0) {
		chg_err("can't read oplus,temperature_strategy_name, rc=%d\n", rc);
		goto not_support;
	}

	rc = oplus_chg_strategy_read_data(chip->dev, "oplus,temperature_strategy_data", &chip->temperature_strategy_data);
	if (rc < 0) {
		chg_err("read oplus,temperature_strategy_data failed, rc=%d\n", rc);
		goto not_support;
	}

	chip->temperature_strategy_data_size = rc;

	chip->temperature_strategy = oplus_chg_strategy_alloc(chip->temperature_strategy_name, chip->temperature_strategy_data, chip->temperature_strategy_data_size);
	if (chip->temperature_strategy == NULL)
		chg_err("%s strategy alloc error\n", chip->temperature_strategy_name);
	devm_kfree(chip->dev, chip->temperature_strategy_data);
	chip->temperature_strategy_data = NULL;
	return 0;
not_support:
	chip->temperature_strategy = NULL;
	chip->temperature_strategy_name = NULL;
	chip->temperature_strategy_data = NULL;
	chip->temperature_strategy_data_size = 0;
	return -ENOTSUPP;
}

static enum oplus_cp_work_mode vbus_to_cp_work_mode(int vbus_mv)
{
	enum oplus_cp_work_mode mode;
	int batt_num;

#define BATT_VOL_MAX_MV	5000

	batt_num = oplus_gauge_get_batt_num();

	if (vbus_mv >= batt_num * BATT_VOL_MAX_MV) {
		switch (vbus_mv / (batt_num * BATT_VOL_MAX_MV)) {
		case 1:
			mode = CP_WORK_MODE_BYPASS;
			break;
		case 2:
			mode = CP_WORK_MODE_2_TO_1;
			break;
		case 3:
			mode = CP_WORK_MODE_3_TO_1;
			break;
		case 4:
			mode = CP_WORK_MODE_4_TO_1;
			break;
		default:
			mode = CP_WORK_MODE_UNKNOWN;
			break;
		}
	} else {
		/* TODO */
		mode = CP_WORK_MODE_UNKNOWN;
	}

	return mode;
}

__maybe_unused
static int32_t oplus_pps_get_curve_vbus(struct oplus_pps *chip)
{
	struct puc_strategy_ret_data data;
	int rc;

	if (chip->strategy == NULL)
		return -EINVAL;

	rc = oplus_chg_strategy_get_data(chip->strategy, &data);
	if (rc < 0) {
		chg_err("can't get curve vbus, rc=%d\n", rc);
		return rc;
	}

	return data.target_vbus;
}

static int oplus_pps_apply_ccd(struct oplus_pps *chip, int target_ibus)
{
	int derated_ibus;

	if (!chip || !chip->ccd_strategy)
		return target_ibus;
	mutex_lock(&chip->ccd_lock);
	oplus_chg_strategy_set_process_data(chip->ccd_strategy, "curve_ibus", target_ibus);
	if (oplus_chg_strategy_get_data(chip->ccd_strategy, &derated_ibus) >= 0)
		target_ibus = derated_ibus;
	mutex_unlock(&chip->ccd_lock);
	return target_ibus;
}

static void oplus_pps_ccd_strategy_init(struct oplus_pps *chip)
{
	int rc;
	u32 ccd_temp_region;

	if (!chip || !chip->ccd_strategy)
		return;
	ccd_temp_region = chip->pps_temp_cur_range > 0 ? chip->pps_temp_cur_range - 1 : 0;
	rc = oplus_chg_strategy_init(chip->ccd_strategy);
	if (rc < 0) {
		oplus_chg_strategy_release(chip->ccd_strategy);
		chip->ccd_strategy = NULL;
		return;
	}
	oplus_chg_strategy_set_process_data(chip->ccd_strategy, "temp_region", ccd_temp_region);
}

int oplus_pps_get_curve_ibus(struct oplus_mms *mms)
{
	struct oplus_pps *chip;
	struct puc_strategy_ret_data data;
	int rc;
	int target_ibus;

	if (mms == NULL)
		return -EINVAL;

	chip = oplus_mms_get_drvdata(mms);
	if (chip == NULL || chip->strategy == NULL)
		return -EINVAL;

	rc = oplus_chg_strategy_get_data(chip->strategy, &data);
	if (rc < 0) {
		chg_err("can't get curve ibus, rc=%d\n", rc);
		return rc;
	}
	target_ibus = data.target_ibus;
	if (chip->ccd_strategy)
		target_ibus = oplus_pps_apply_ccd(chip, data.target_ibus);
	return min(target_ibus, chip->adapter_max_curr);
}

int oplus_chg_get_pdo_info(struct oplus_mms *mms, u32 *pdo)
{
	struct oplus_pps *chip;
	int pdo_index;

	if (mms == NULL) {
		chg_err("mms is NULL\n");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (pdo == NULL) {
		chg_err("pdo is NULL\n");
                return -EINVAL;
	}

	for (pdo_index = 0; pdo_index < PPS_PDO_MAX; pdo_index++) {
		pdo[pdo_index] = *((u32 *)chip->pdo + pdo_index);
	}

	return 0;
}

__maybe_unused
static int oplus_pps_pdo_set(struct oplus_pps *chip, int vol_mv, int curr_ma)
{
	int rc;

	if (chip->pps_ic == NULL) {
		chg_err("pps_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->pps_ic, OPLUS_IC_FUNC_PPS_PDO_SET, vol_mv, curr_ma);
	if (rc == 0) {
		chip->curr_set_ma = curr_ma;
		chip->vol_set_mv = vol_mv;
	}

	return rc;
}

__maybe_unused
static int oplus_pps_hard_reset(struct oplus_pps *chip)
{
	int rc;

	if (chip->pps_ic == NULL) {
		chg_err("pps_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->pps_ic, OPLUS_IC_FUNC_PPS_HARD_RESET);

	return rc;
}

__maybe_unused
static int oplus_pps_exit_pps_mode(struct oplus_pps *chip)
{
	int rc;

	if (chip->pps_ic == NULL) {
		chg_err("pps_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->pps_ic, OPLUS_IC_FUNC_PPS_EXIT);
	if (rc < 0)
		chg_err("exit pps mode error, rc=%d", rc);

	return rc;
}

__maybe_unused
static int oplus_pps_get_err_info(struct oplus_pps *chip, u64 *err_info)
{
	int rc;

	if (chip->pps_ic == NULL) {
		chg_err("pps_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->pps_ic, OPLUS_IC_FUNC_PPS_GET_ERR_INFO, err_info);

	return rc;
}

__maybe_unused
static int oplus_pps_get_pps_status_info(struct oplus_pps *chip, u32 *pps_status_info)
{
	int rc;

	if (chip->pps_ic == NULL) {
		chg_err("pps_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->pps_ic, OPLUS_IC_FUNC_GET_PPS_STATUS, pps_status_info);
	if (rc < 0)
		chip->error_count++;
	else
		chip->error_count = 0;

	return rc;
}

__maybe_unused
static int oplus_pps_get_pdo_info(struct oplus_pps *chip, u32 *pdo, int num)
{
	int rc;

	if (chip->pps_ic == NULL) {
		chg_err("pps_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->pps_ic, OPLUS_IC_FUNC_PPS_GET_PDO_INFO, pdo, num);

	return rc;
}

__maybe_unused
static int oplus_pps_verify_adapter(struct oplus_pps *chip)
{
	int rc;

	if (chip->pps_ic == NULL) {
		chg_err("pps_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->pps_ic, OPLUS_IC_FUNC_PPS_VERIFY_ADAPTER);

	return rc;
}

__maybe_unused
static int oplus_pps_get_power_change_info(
	struct oplus_pps *chip, u32 *pwr_change_info, int num)
{
	int rc;

	if (chip->pps_ic == NULL) {
		chg_err("pps_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->pps_ic, OPLUS_IC_FUNC_PPS_GET_POWER_CHANGE_INFO,
		pwr_change_info, num);

	return rc;
}

__maybe_unused
static int oplus_pps_cp_enable(struct oplus_pps *chip, bool en)
{
	int rc = 0;

	if (chip->cp_ic == NULL) {
		chg_err("cp_ic is NULL\n");
		return -ENODEV;
	}

	/* Not call CP_ENABLE temporary, revert this change after
	   the multi-CP startup logic is completed.
	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_ENABLE, en);
	*/

	return rc;
}

__maybe_unused
static int oplus_pps_cp_watchdog_enable(struct oplus_pps *chip, int time_ms)
{
	int rc;

	if (chip->cp_ic == NULL) {
		chg_err("cp_ic is NULL\n");
		return -ENODEV;
	}
	chg_err("pps set cp watchdog time to %dms\n", time_ms);

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_WATCHDOG_ENABLE, time_ms);

	return rc;
}

__maybe_unused
static int oplus_pps_cp_hw_init(struct oplus_pps *chip)
{
	int rc;

	if (chip->cp_ic == NULL) {
		chg_err("cp_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_HW_INTI);

	return rc;
}

__maybe_unused
static int oplus_pps_cp_set_work_mode(struct oplus_pps *chip, enum oplus_cp_work_mode mode)
{
	int rc;

	if (chip->cp_ic == NULL) {
		chg_err("cp_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_SET_WORK_MODE, mode);

	if (rc >= 0) {
		switch (mode) {
		case CP_WORK_MODE_4_TO_1:
			chip->cp_ratio = 4;
			break;
		case CP_WORK_MODE_3_TO_1:
			chip->cp_ratio = 3;
			break;
		case CP_WORK_MODE_2_TO_1:
			chip->cp_ratio = 2;
			break;
		case CP_WORK_MODE_BYPASS:
			chip->cp_ratio = 1;
			break;
		default:
			chg_err("unsupported cp work mode, mode=%d\n", mode);
			rc = -ENOTSUPP;
			break;
		}
	}

	return rc;
}

__maybe_unused
static int oplus_pps_cp_get_work_mode(struct oplus_pps *chip, enum oplus_cp_work_mode *mode)
{
	int rc;

	if (chip->cp_ic == NULL) {
		chg_err("cp_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_GET_WORK_MODE, mode);

	return rc;
}

__maybe_unused
static int oplus_pps_cp_check_work_mode_support(struct oplus_pps *chip, enum oplus_cp_work_mode mode)
{
	int rc;

	if (chip->cp_ic == NULL) {
		chg_err("cp_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_CHECK_WORK_MODE_SUPPORT, mode);

	return rc;
}

__maybe_unused
static int oplus_pps_cp_set_iin(struct oplus_pps *chip, int iin)
{
	int rc;

	if (chip->cp_ic == NULL) {
		chg_err("cp_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_SET_IIN, iin);

	return rc;
}

__maybe_unused
static int oplus_pps_cp_get_vin(struct oplus_pps *chip, int *vin)
{
	int rc;

	if (chip->cp_ic == NULL) {
		chg_err("cp_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_GET_VIN, vin);

	return rc;
}

__maybe_unused
static int oplus_pps_cp_get_iin(struct oplus_pps *chip, int *iin)
{
	int rc;

	if (chip->cp_ic == NULL) {
		chg_err("cp_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_GET_IIN, iin);

	return rc;
}

__maybe_unused
static int oplus_pps_cp_get_vout(struct oplus_pps *chip, int *vout)
{
	int rc;

	if (chip->cp_ic == NULL) {
		chg_err("cp_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_GET_VOUT, vout);

	return rc;
}

__maybe_unused
static int oplus_pps_cp_get_iout(struct oplus_pps *chip, int *iout)
{
	int rc;

	if (chip->cp_ic == NULL) {
		chg_err("cp_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_GET_IOUT, iout);

	return rc;
}

__maybe_unused
static int oplus_pps_cp_get_vac(struct oplus_pps *chip, int *vac)
{
	int rc;

	if (chip->cp_ic == NULL) {
		chg_err("cp_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_GET_VAC, vac);

	return rc;
}

__maybe_unused
static int oplus_pps_cp_set_work_start(struct oplus_pps *chip, bool start)
{
	int rc;

	if (chip->cp_ic == NULL) {
		chg_err("cp_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_SET_WORK_START, start);

	return rc;
}

__maybe_unused
static int oplus_pps_cp_sstimeout_ucp_enable(struct oplus_pps *chip, bool enable)
{
	int rc;

	if (chip->cp_ic == NULL) {
		chg_err("cp_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_SET_SSTIMEOUT_UCP_ENABLE, enable);
	chip->ss_check = enable;

	return rc;
}

__maybe_unused
static int oplus_pps_cp_set_pmid2vout_ovp_enable(struct oplus_pps *chip, bool enable)
{
	int rc;

	if (chip->cp_ic == NULL) {
		chg_err("cp_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_SET_PMID2VOUT_OVP_ENABLE, enable);

	return rc;
}

__maybe_unused
static int oplus_pps_cp_get_work_status(struct oplus_pps *chip, bool *start)
{
	int rc;

	if (chip->cp_ic == NULL) {
		chg_err("cp_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_GET_WORK_STATUS, start);

	return rc;
}

__maybe_unused
static int oplus_pps_cp_adc_enable(struct oplus_pps *chip, bool en)
{
	int rc;

	if (chip->cp_ic == NULL) {
		chg_err("cp_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_SET_ADC_ENABLE, en);

	return rc;
}

__maybe_unused
static int oplus_pps_cp_get_adc_enable(struct oplus_pps *chip, bool *en)
{
	int rc = 0;

	if (chip->cp_ic == NULL) {
		chg_err("cp_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_GET_ADC_ENABLE, en);

	return rc;
}

__maybe_unused
static int oplus_pps_cp_reg_dump(struct oplus_pps *chip)
{
	int rc;

	if (chip->cp_ic == NULL) {
		chg_err("cp_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_REG_DUMP);

	return rc;
}

static int oplus_pps_cp_get_iin_max(struct oplus_pps *chip,
	enum oplus_cp_work_mode mode, int *iin_max)
{
	int rc;

	if (chip->cp_ic == NULL) {
		chg_err("cp_ic is NULL\n");
		return -ENODEV;
	}
	if (mode == CP_WORK_MODE_UNKNOWN)
		return -EINVAL;

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_GET_IIN_MAX, mode, iin_max);
	if (rc < 0 && rc != -ENOTSUPP)
		return rc;

	return 0;
}

static int oplus_pps_cp_watchdog_reset(struct oplus_pps *chip)
{
	int rc;

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_WATCHDOG_RESET);
	if (rc < 0 && rc != -ENOTSUPP)
		return rc;

	return 0;
}

static void oplus_pps_set_ic_pps_curr_vote(struct oplus_pps *chip, enum oplus_cp_work_mode mode)
{
	int iin_max;
	int rc;

	rc = oplus_pps_cp_get_iin_max(chip, mode, &iin_max);
	if (rc < 0) {
		chg_err("can't get cp iin max, rc=%d\n", rc);
		return;
	}
	vote(chip->pps_curr_votable, IC_VOTER, true, iin_max, false);
}

static int oplus_pps_switch_to_normal(struct oplus_pps *chip)
{
	int rc;

	if (chip->dpdm_switch == NULL) {
		chg_err("dpdm_switch ic is NULL\n");
		return -ENODEV;
	}

	/* switch to 5v when switch to normal */
	if (chip->wired_online)
		rc = oplus_chg_ic_func(chip->pps_ic, OPLUS_IC_FUNC_FIXED_PDO_SET,
				OPLUS_FIXED_PDO_DEF_VOL, OPLUS_FIXED_PDO_CURR_MA);

	rc = oplus_chg_ic_func(chip->dpdm_switch,
		OPLUS_IC_FUNC_SET_DPDM_SWITCH_MODE, DPDM_SWITCH_TO_AP);

	return rc;
}

static int oplus_pps_publish_cpa_power(struct oplus_pps *chip)
{
	struct mms_msg *msg;
	int rc;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  PPS_ITEM_CPA_POWER);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg_sync(chip->pps_topic, msg);
	if (rc < 0) {
		chg_err("publish pps online msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static int oplus_pps_publish_adapter_power(struct oplus_pps *chip)
{
	struct mms_msg *msg;
	int rc;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  PPS_ITEM_ADAPTER_POWER);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg_sync(chip->pps_topic, msg);
	if (rc < 0) {
		chg_err("publish pps online msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static int oplus_pps_publish_adapter_info(struct oplus_pps *chip)
{
	struct mms_msg *msg;
	int rc;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  PPS_ITEM_PPS_ADAPTER_INFO);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg_sync(chip->pps_topic, msg);
	if (rc < 0) {
		chg_err("publish pps online msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static int oplus_pps_track_set_fcl_info(struct device *dev, int batt_volt, int batt_curr, int batt_temp)
{
	int rc;
	struct mms_msg *msg;
	struct oplus_mms *err_topic = oplus_mms_get_by_name("error");

	if (!err_topic)
		return -ENODEV;

	if (batt_volt)
		msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM,
			MSG_PRIO_MEDIUM, ERR_ITEM_FCL_INFO, "one_time_full==%d,%d,%d",
			batt_volt, batt_curr, batt_temp);
	else
		msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM,
			MSG_PRIO_MEDIUM, ERR_ITEM_FCL_INFO, "n_time_full=%d,%d,%d",
			batt_volt, batt_curr, batt_temp);
	if (msg == NULL) {
		chg_err("alloc device id msg error\n");
		return -ENOMEM;
	}

	rc = oplus_mms_publish_msg_sync(err_topic, msg);
	if (rc < 0) {
		chg_err("publish pcl info msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static int oplus_pps_set_online(struct oplus_pps *chip, bool online)
{
	struct mms_msg *msg;
	int rc;

	if (chip->pps_online == online)
		return 0;

	chip->pps_online = online;
	if (chip->pps_online)
		chip->pps_online_time = oplus_chg_pps_get_current_time_s();
	chg_info("set pps_online=%s, online_time = %ld\n", online ? "true" : "false", chip->pps_online_time);

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  PPS_ITEM_ONLINE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->pps_topic, msg);
	if (rc < 0) {
		chg_err("publish pps online msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static int oplus_pps_set_online_keep(struct oplus_pps *chip, bool online)
{
	struct mms_msg *msg;
	int rc;

	if (chip->pps_online_keep == online)
		return 0;

	chip->pps_online_keep = online;
	chg_info("set pps_online_keep=%s\n", online ? "true" : "false");

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  PPS_ITEM_ONLINE_KEEP);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->pps_topic, msg);
	if (rc < 0) {
		chg_err("publish pps online keep msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static int oplus_pps_set_charging(struct oplus_pps *chip, bool charging)
{
	struct mms_msg *msg;
	enum oplus_plc_chg_mode chg_mode;
	int rc;

	if (chip->pps_charging == charging)
		return 0;

	chip->pps_charging = charging;
	chg_info("set pps_charging=%s\n", charging ? "true" : "false");
	if (!chip->pps_charging)
		chip->adapter_max_curr = 0;
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  PPS_ITEM_CHARGING);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->pps_topic, msg);
	if (rc < 0) {
		chg_err("publish pps charging msg error, rc=%d\n", rc);
		kfree(msg);
		return rc;
	}

	if (chip->plc_topic == NULL)
		return 0;

	if (charging)
		chg_mode = PLC_CHG_MODE_CP;
	else
		chg_mode = PLC_CHG_MODE_BUCK;
	msg = oplus_mms_alloc_int_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH,
				      PLC_ITEM_CHG_MODE, chg_mode);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->plc_topic, msg);
	if (rc < 0) {
		chg_err("publish plc chg mode msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static int oplus_pps_set_oplus_adapter(struct oplus_pps *chip, bool oplus_adapter)
{
	struct mms_msg *msg;
	int rc;

	if (chip->oplus_pps_adapter == oplus_adapter)
		return 0;

	chip->oplus_pps_adapter = oplus_adapter;
	if (oplus_adapter)
		chip->retention_oplus_adapter = true;
	chg_info("set oplus_pps_adapter=%s\n", oplus_adapter ? "true" : "false");

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  PPS_ITEM_OPLUS_ADAPTER);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->pps_topic, msg);
	if (rc < 0) {
		chg_err("publish oplus adapter msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static void oplus_pps_switch_end_recheck_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_pps *chip =
		container_of(dwork, struct oplus_pps, switch_end_recheck_work);

	chg_info("pps switch end recheck\n");
	if (chip->pps_online)
		return;
	if (chip->retention_state_ready)
		return;
	chg_info("pps switch end\n");
	oplus_cpa_switch_end(chip->cpa_topic, CHG_PROTOCOL_PPS);
}

#define SWITCH_END_RECHECK_DELAY_MS	1000
static int oplus_pps_cpa_switch_end(struct oplus_pps *chip)
{
	if (chip->pps_online)
		return 0;
	if (chip->retention_topic && !chip->retention_ready_for_switch_end && !chip->irq_plugin) {
		chg_info("pps plugout,wait retention state ready\n");
		return 0;
	}
	if (!chip->retention_state || chip->retention_exit_pps_flag == EXIT_THIRD_PPS) {
		oplus_cpa_switch_end(chip->cpa_topic, CHG_PROTOCOL_PPS);
	} else {
		if (!chip->retention_state_ready)
			schedule_delayed_work(&chip->switch_end_recheck_work,
				msecs_to_jiffies(SWITCH_END_RECHECK_DELAY_MS));
	}
	return 0;
}

static void oplus_pps_charge_btb_allow_check(struct oplus_pps *chip)
{
#define PPS_BTB_CHECK_MAX_CNT 3
#define PPS_BTB_CHECK_TIME_US 10000

	int btb_check_cnt = PPS_BTB_CHECK_MAX_CNT;
	int batt_btb_temp;
	int usb_btb_temp;
	bool shaft_btb_normal = true;

	while (btb_check_cnt != 0) {
		batt_btb_temp = oplus_wired_get_batt_btb_temp();
		usb_btb_temp = oplus_wired_get_usb_btb_temp();
		shaft_btb_normal = oplus_wired_get_shaft_btb_is_normal();

		if (batt_btb_temp < PPS_BTB_OVER_TEMP && usb_btb_temp < PPS_BTB_OVER_TEMP && shaft_btb_normal)
			break;

		btb_check_cnt--;
		if (btb_check_cnt > 0)
			usleep_range(PPS_BTB_CHECK_TIME_US, PPS_BTB_CHECK_TIME_US);
	}
	if (btb_check_cnt == 0) {
		chg_info("batt_btb_temp: %d, usb_btb_temp = %d shaft_btb_normal %d",
			batt_btb_temp, usb_btb_temp, shaft_btb_normal);
		vote(chip->pps_not_allow_votable, BTB_TEMP_OVER_VOTER, true, 1, false);
		if (is_wired_icl_votable_available(chip))
			vote(chip->wired_icl_votable, BTB_TEMP_OVER_VOTER, true,
			     BTB_TEMP_OVER_MAX_INPUT_CUR, true);
		oplus_pps_push_err_info(chip, PPS_ERR_BTB_OVER, 0);
		if (!shaft_btb_normal)
			oplus_wired_set_shaft_btb_over(true);
	} else {
		oplus_wired_set_shaft_btb_over(false);
	}
}

static int oplus_pps_parse_vfa_strategy(struct oplus_pps *chip)
{
	struct device_node *vfa_node;

	vfa_node = of_parse_phandle(oplus_get_node_by_type(chip->dev->of_node), "oplus,pps_vfa_strategy", 0);
	if (!vfa_node) {
		chg_info("pps_vfa_strategy not found, skip vfa strategy\n");
		return 0;
	}

	chip->vfa_strategy = oplus_chg_strategy_alloc_by_node("vfa_strategy", vfa_node);
	of_node_put(vfa_node);
	if (IS_ERR_OR_NULL(chip->vfa_strategy)) {
		chg_info("pps_vfa_strategy alloc fail, rc=%ld", PTR_ERR(chip->vfa_strategy));
		chip->vfa_strategy = NULL;
		return -EFAULT;
	}

	return 0;
}

static void oplus_pps_vfa_reset(struct oplus_pps *chip)
{
	if (chip->vfa_strategy)
		oplus_chg_strategy_set_process_data(chip->vfa_strategy, "reset", 0);
	vote(chip->pps_not_allow_votable, VFA_VOTER, false, 0, false);
}

static void oplus_pps_vfa_allow(struct oplus_pps *chip, bool do_init)
{
	int rc;
	int allow_fastchg = 1;

	if (!chip->vfa_strategy)
		goto done;

	if (do_init) {
		rc = oplus_chg_strategy_init(chip->vfa_strategy);
		if (rc < 0) {
			chg_err("vfa_strategy init fail, rc=%d\n", rc);
			allow_fastchg = 1;
			goto done;
		}
	}

	rc = oplus_chg_strategy_get_data(chip->vfa_strategy, &allow_fastchg);
	if (rc < 0) {
		chg_err("vfa_strategy get fail, rc=%d\n", rc);
		allow_fastchg = 1;
		goto done;
	}

done:
	vote(chip->pps_not_allow_votable, VFA_VOTER, !allow_fastchg, 1, false);
}

static void oplus_pps_offset_current_temp_range(struct oplus_pps *chip, int up_thr_offset, int down_thr_offset)
{
	int vbat_temp_cur;

	vbat_temp_cur = chip->shell_temp;
	oplus_pps_reset_temp_range(chip);
	if (vbat_temp_cur < chip->limits.pps_little_cold_temp) { /*<5C*/
		chip->limits.pps_little_cold_temp += up_thr_offset;
	} else if (vbat_temp_cur < chip->limits.pps_cool_temp) { /*<12C*/
		chip->limits.pps_cool_temp += up_thr_offset;
		chip->limits.pps_little_cold_temp += down_thr_offset;
	} else if (vbat_temp_cur < chip->limits.pps_little_cool_temp) { /*<20C*/
		chip->limits.pps_little_cool_temp += up_thr_offset;
		chip->limits.pps_cool_temp += down_thr_offset;
	} else if (chip->limits.pps_little_cool_high_temp != -EINVAL &&
	    vbat_temp_cur < chip->limits.pps_little_cool_high_temp) { /*<21C*/
		chip->limits.pps_little_cool_high_temp += up_thr_offset;
		chip->limits.pps_little_cool_temp += down_thr_offset;
	} else if (chip->limits.pps_normal_low_pre_temp != -EINVAL &&
	    vbat_temp_cur < chip->limits.pps_normal_low_pre_temp) {
		chip->limits.pps_normal_low_pre_temp += up_thr_offset;
		if (chip->limits.pps_little_cool_high_temp != -EINVAL)
			chip->limits.pps_little_cool_high_temp += down_thr_offset;
		else
			chip->limits.pps_little_cool_temp += down_thr_offset;
	} else if (vbat_temp_cur < chip->limits.pps_normal_low_temp) { /*<35C*/
		chip->limits.pps_normal_low_temp += up_thr_offset;
		if (chip->limits.pps_normal_low_pre_temp != -EINVAL)
			chip->limits.pps_normal_low_pre_temp += down_thr_offset;
		else if (chip->limits.pps_little_cool_high_temp != -EINVAL)
			chip->limits.pps_little_cool_high_temp += down_thr_offset;
		else
			chip->limits.pps_little_cool_temp += down_thr_offset;
	} else if (vbat_temp_cur < chip->limits.pps_normal_high_temp) { /*<43C*/
		chip->limits.pps_normal_high_temp += up_thr_offset;
		chip->limits.pps_normal_low_temp += down_thr_offset;
	} else {							/*>=43*/
		chip->limits.pps_normal_high_temp += down_thr_offset;
	}
}

static bool oplus_pps_charge_allow_check(struct oplus_pps *chip)
{
	union mms_msg_data data = { 0 };
	int vbat_mv = 0;
	int chg_temp = 0;
	int rc = 0;

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX, &data, false);
	if (unlikely(rc < 0)) {
		chg_err("can't get vbat, rc=%d\n", rc);
		vbat_mv = 3800;
	} else {
		vbat_mv = data.intval;
	}

	rc = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_SHELL_TEMP, &data, true);
	if (unlikely(rc < 0)) {
		chg_err("can't get comm_item_shell_temp, rc=%d\n", rc);
		rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_TEMP, &data, false);
		if (unlikely(rc < 0)) {
			chg_err("can't get gauge_item_temp, rc=%d\n", rc);
			chg_temp = 250;
		} else {
			chg_temp = data.intval;
		}
	} else {
		chg_temp = data.intval;
	}
	chip->shell_temp = chg_temp;

	if (chg_temp < chip->limits.pps_low_temp ||
	    chg_temp >= (chip->limits.pps_batt_over_high_temp - PPS_TEMP_WARM_RANGE_THD)) {
		vote(chip->pps_not_allow_votable, BATT_TEMP_VOTER, true, 1, false);
	} else if (chg_temp > (chip->limits.pps_normal_high_temp - PPS_TEMP_WARM_RANGE_THD)) {
		if (chip->ui_soc > chip->limits.pps_warm_allow_soc ||
		    vbat_mv > chip->limits.pps_warm_allow_vol)
			vote(chip->pps_not_allow_votable, BATT_TEMP_VOTER, true, 1, false);
		else
			vote(chip->pps_not_allow_votable, BATT_TEMP_VOTER, false, 0, false);
	} else {
		vote(chip->pps_not_allow_votable, BATT_TEMP_VOTER, false, 0, false);
	}

	if (chip->ui_soc < chip->limits.pps_strategy_soc_over_low ||
	    chip->ui_soc > chip->limits.pps_strategy_soc_high) {
		vote(chip->pps_not_allow_votable, BATT_SOC_VOTER, true, 1, false);
	} else {
		vote(chip->pps_not_allow_votable, BATT_SOC_VOTER, false, 0, false);
		chip->allow_check_soc = chip->ui_soc;
	}

	if (is_client_vote_enabled(chip->pps_not_allow_votable, CHG_FULL_COOL_VOTER)) {
		oplus_pps_temp_cur_range_init(chip);
		if (chip->pps_temp_cur_range > chip->pps_cool_full_temp_range) {
			chg_info("allow pps charging, cur_temp_range=%d, full_temp_range=%d, shell_temp=%d\n",
				chip->pps_temp_cur_range, chip->pps_cool_full_temp_range, chip->shell_temp);
			oplus_pps_offset_current_temp_range(chip, 0, -PPS_TEMP_LOW_RANGE_THD);
			chip->pps_cool_full_temp_range = PPS_TEMP_RANGE_WARM;
			vote(chip->pps_not_allow_votable, CHG_FULL_COOL_VOTER, false, 0, false);
		}
	}

	oplus_pps_charge_btb_allow_check(chip);

	if (is_client_vote_enabled(chip->pps_not_allow_votable, VFA_VOTER))
		oplus_pps_vfa_allow(chip, false);

	return !chip->pps_not_allow;
}

static void oplus_pps_count_init(struct oplus_pps *chip)
{
	chip->count.cool_fw = 0;
	chip->count.sw_full = 0;
	chip->count.hw_full = 0;
	chip->count.low_curr_full = 0;
	chip->count.ibat_low = 0;
	chip->count.ibat_high = 0;
	chip->count.curr_over = 0;
	chip->count.curr_abnormal = 0;
	chip->count.btb_high = 0;
	chip->count.tbatt_over = 0;
	chip->count.tfg_over = 0;
	chip->count.output_low = 0;
	chip->count.ibus_over = 0;
	chip->count.pps_recovery = 0;
	chip->count.req_vol_over = 0;
}

static void oplus_pps_votable_reset(struct oplus_pps *chip)
{
	vote(chip->pps_disable_votable, NO_DATA_VOTER, false, 0, false);
	vote(chip->pps_disable_votable, CHG_FULL_VOTER, false, 0, false);
	vote(chip->pps_disable_votable, IBAT_OVER_VOTER, false, 0, false);
	vote(chip->pps_disable_votable, BTB_TEMP_OVER_VOTER, false, 0, false);
	vote(chip->pps_disable_votable, BATT_TEMP_VOTER, false, 0, false);
	vote(chip->pps_disable_votable, TFG_VOTER, false, 0, false);
	vote(chip->pps_disable_votable, IMP_VOTER, false, 0, false);
	vote(chip->pps_disable_votable, TIMEOUT_VOTER, false, 0, false);
	vote(chip->pps_not_allow_votable, CHG_FULL_COOL_VOTER, false, 0, false);
	if (!chip->retention_state)
		vote(chip->pps_disable_votable, CONNECT_VOTER, false, 0, false);
	vote(chip->pps_disable_votable, PPS_IBAT_ABNOR_VOTER, false, 0, false);
	vote(chip->pps_disable_votable, SWITCH_RANGE_VOTER, false, 0, false);
	vote(chip->pps_not_allow_votable, BTB_TEMP_OVER_VOTER, false, 0, false);

	vote(chip->pps_curr_votable, IMP_VOTER, false, 0, false);
	vote(chip->pps_curr_votable, STEP_VOTER, false, 0, false);
	vote(chip->pps_curr_votable, ADAPTER_MAX_POWER, false, 0, false);
	vote(chip->pps_curr_votable, LED_ON_VOTER, false, 0, false);
	vote(chip->pps_curr_votable, BATT_TEMP_VOTER, false, 0, false);
	vote(chip->pps_curr_votable, COOL_DOWN_VOTER, false, 0, false);
	vote(chip->pps_curr_votable, BATT_BAL_VOTER, false, 0, false);
	vote(chip->pps_curr_votable, SALE_MODE_VOTER, false, 0, false);
	vote(chip->pps_curr_votable, IC_VOTER, false, 0, false);
	vote(chip->pps_curr_votable, BATT_SOC_VOTER, false, 0, false);
	vote(chip->pps_curr_votable, LIMIT_FCL_VOTER, false, 0, false);
}

static int oplus_pps_temp_cur_range_init(struct oplus_pps *chip)
{
	int vbat_temp_cur;

	vbat_temp_cur = chip->shell_temp;
	if (vbat_temp_cur < chip->limits.pps_little_cold_temp) { /*0-5C*/
		chip->pps_temp_cur_range = PPS_TEMP_RANGE_LITTLE_COLD;
		chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_LITTLE_COLD;
	} else if (vbat_temp_cur < chip->limits.pps_cool_temp) { /*5-12C*/
		chip->pps_temp_cur_range = PPS_TEMP_RANGE_COOL;
		chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_COOL;
	} else if (vbat_temp_cur < chip->limits.pps_little_cool_temp) { /*12-20C*/
		chip->pps_temp_cur_range = PPS_TEMP_RANGE_LITTLE_COOL;
		chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_LITTLE_COOL;
	} else if (chip->limits.pps_little_cool_high_temp != -EINVAL &&
	    vbat_temp_cur < chip->limits.pps_little_cool_high_temp) {
		chip->pps_temp_cur_range = PPS_TEMP_RANGE_LITTLE_COOL_HIGH;
		chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_LITTLE_COOL_HIGH;
	} else if (chip->limits.pps_normal_low_pre_temp != -EINVAL &&
	    vbat_temp_cur < chip->limits.pps_normal_low_pre_temp) {
		chip->pps_temp_cur_range = PPS_TEMP_RANGE_NORMAL_LOW_PRE;
		chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_NORMAL_LOW_PRE;
	} else if (vbat_temp_cur < chip->limits.pps_normal_low_temp) { /*20-35C*/
		chip->pps_temp_cur_range = PPS_TEMP_RANGE_NORMAL_LOW;
		chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_NORMAL_LOW;
	} else if (vbat_temp_cur < chip->limits.pps_normal_high_temp) { /*35C-43C*/
		chip->pps_temp_cur_range = PPS_TEMP_RANGE_NORMAL_HIGH;
		chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_NORMAL_HIGH;
	} else {
		chip->pps_temp_cur_range = PPS_TEMP_RANGE_WARM;
		chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_WARM;
	}

	return 0;
}
static void oplus_pps_variables_init(struct oplus_pps *chip)
{
	oplus_pps_count_init(chip);

	chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_NATURAL;
	chip->pps_temp_cur_range = PPS_TEMP_RANGE_INIT;
	chip->error_count = 0;
	chip->pdo_set_error_count = 0;
	chip->need_check_current = false;
	chip->quit_pps_protocol = false;
	chip->mos_on_check = false;
	chip->batt_bal_curr_limit = 0;
	chip->support_pps_status = true;
	chip->pps_curr_ma_from_pps_status = 0;
	chip->support_cp_ibus = false;
	chip->request_vbus_too_low_flag = false;
	chip->ss_check = false;
	chip->fcl_trigger = false;
	chip->allow_check_soc = chip->ui_soc;
	/* Keep async clear to avoid blocking init/transition path on ICL lock. */
	oplus_pps_clear_fcs_icl(chip);

	chip->timer.fastchg_timer = oplus_current_kernel_time();
	chip->timer.temp_timer = oplus_current_kernel_time();
	chip->timer.pdo_timer = oplus_current_kernel_time();
	chip->timer.ibat_jiffies = jiffies;
	if (!chip->oplus_pps_adapter)
		chip->timer.pps_max_time_ms = chip->limits.pps_timeout_third * 1000;
	else
		chip->timer.pps_max_time_ms = chip->limits.pps_timeout_oplus * 1000;

	chip->pps_low_curr_full_temp_status =
		PPS_LOW_CURR_FULL_CURVE_TEMP_NORMAL_LOW;

	(void)oplus_imp_uint_reset(chip->imp_uint);
	chip->last_target_curr_ma = chip->target_curr_ma;
}

static void oplus_pps_force_exit(struct oplus_pps *chip)
{
	/*
	 * Keep ICL clear asynchronous here to avoid exit-path blocking on
	 * WIRED_ICL vote/AICL lock contention (e.g. COMMON_POWER_CHECK voter),
	 * which can noticeably delay UFCS->SVOOC handover.
	 * Residual FCS_ICL_VOTER (if any) is expected to be overridden by the
	 * next protocol initialization path (for example SVOOC re-initialization).
	 * DO NOT add flush_work()/cancel_work_sync() here.
	 */
	oplus_pps_clear_fcs_icl(chip);
	oplus_pps_set_charging(chip, false);
	oplus_pps_set_oplus_adapter(chip, false);
	chip->cp_work_mode = CP_WORK_MODE_UNKNOWN;
	chip->cp_ratio = 0;
	chip->pdo_set_error_count = 0;
	oplus_pps_exit_pps_mode(chip);
	oplus_pps_cp_set_work_start(chip, false);
	oplus_pps_cp_enable(chip, false);
	oplus_pps_cp_watchdog_enable(chip, CP_WATCHDOG_DISABLE);
	oplus_pps_cp_adc_enable(chip, false);
	oplus_pps_switch_to_normal(chip);
	oplus_pps_set_online(chip, false);
	oplus_pps_cpa_switch_end(chip);
	vote(chip->pps_curr_votable, STEP_VOTER, false, 0, false);
	vote(chip->pps_curr_votable, PLC_VOTER, false, 0, false);
	if (is_wired_suspend_votable_available(chip))
		vote(chip->wired_suspend_votable, PPS_VOTER, false, 0, false);

	if (is_disable_charger_vatable_available(chip))
		vote(chip->chg_disable_votable, PPS_VOTER, false, 0, false);
}

static void oplus_pps_soft_exit(struct oplus_pps *chip)
{
	/*
	 * Same rationale as force_exit(): prioritize fast protocol exit/handover
	 * and avoid synchronous lock wait in WIRED_ICL voting path.
	 * Residual FCS_ICL_VOTER (if any) is expected to be overridden by the
	 * next protocol initialization path (for example SVOOC re-initialization).
	 * DO NOT add flush_work()/cancel_work_sync() here.
	 */
	oplus_pps_clear_fcs_icl(chip);
	oplus_pps_set_charging(chip, false);
	oplus_pps_set_oplus_adapter(chip, false);
	chip->cp_work_mode = CP_WORK_MODE_UNKNOWN;
	chip->cp_ratio = 0;
	chip->pdo_set_error_count = 0;
	oplus_pps_exit_pps_mode(chip);
	oplus_pps_cp_set_work_start(chip, false);
	oplus_pps_cp_enable(chip, false);
	oplus_pps_cp_watchdog_enable(chip, CP_WATCHDOG_DISABLE);
	oplus_pps_cp_adc_enable(chip, false);
	oplus_pps_switch_to_normal(chip);
	vote(chip->pps_curr_votable, STEP_VOTER, false, 0, false);
	vote(chip->pps_curr_votable, PLC_VOTER, false, 0, false);
	if (is_wired_suspend_votable_available(chip))
		vote(chip->wired_suspend_votable, PPS_VOTER, false, 0, false);
	if (is_disable_charger_vatable_available(chip))
		vote(chip->chg_disable_votable, PPS_VOTER, false, 0, false);
}

static void oplus_pps_shutdown_exit(struct oplus_pps *chip)
{
	vote(chip->pps_disable_votable, SHUTDOWN_VOTER, true, 1, false);
	cancel_delayed_work_sync(&chip->switch_check_work);
	cancel_delayed_work_sync(&chip->monitor_work);
	cancel_delayed_work_sync(&chip->current_work);
	oplus_pps_soft_exit(chip);
}

static void oplus_pps_sub_btb_connnect_check(struct oplus_pps *chip)
{
	int btb_abnormal_current = 0;
	union mms_msg_data data = { 0 };

	if (NULL == chip || NULL == chip->gauge_topic)
		return;

	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SUB_BTB_STATE, &data,  false);
	if (data.intval == BATT_BTB_STATE_NOT_CONNECT) {
		btb_abnormal_current = oplus_gauge_get_sub_btb_curr_limit(chip->gauge_topic);
		chg_debug("batt_sub_btb abnormal, max curr set %d mA.\n",
			   btb_abnormal_current);
		if (btb_abnormal_current > 0)
			vote(chip->pps_curr_votable, BAD_SUB_BTB_VOTER, true,
			     btb_abnormal_current, false);
	} else {
		vote(chip->pps_curr_votable, BAD_SUB_BTB_VOTER, false, 0, false);
	}
}

static void oplus_pps_switch_check_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_pps *chip =
		container_of(dwork, struct oplus_pps, switch_check_work);
	enum oplus_cp_work_mode cp_mode;
	union mms_msg_data data = { 0 };
	int rc;
	int i;
	bool pdo_ok = false;
	int max_curr = 0;
	int max_power = 0;
	int target_vbus_max = 0;
	int wired_type;
	int local_time_ms;
	int delta_time;
	bool enable_adc = false;

	rc = oplus_cpa_switch_start(chip->cpa_topic, CHG_PROTOCOL_PPS);
	if (rc < 0) {
		chg_info("cpa protocol not pps, return\n");
		return;
	}

	if (!chip->pdsvooc_id_adapter) {
		reinit_completion(&chip->pd_svooc_wait_ack);
		rc = wait_for_completion_timeout(&chip->pd_svooc_wait_ack,
			msecs_to_jiffies(PD_SVOOC_WAIT_MS));
		if (rc)
			chg_info("svid wait timeout\n");
	}
	rc = oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_REAL_CHG_TYPE,
				&data, true);
	if (rc < 0)
		wired_type = OPLUS_CHG_USB_TYPE_UNKNOWN;
	else
		wired_type = data.intval;

	if (wired_type != OPLUS_CHG_USB_TYPE_PD_PPS) {
		if (chip->retention_state) {
			msleep(WAIT_BC1P2_GET_TYPE);
			rc = oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_REAL_CHG_TYPE,
					&data, false);
			if (rc >= 0)
				wired_type = data.intval;
			if (wired_type != OPLUS_CHG_USB_TYPE_PD_PPS) {
				chg_err("wired_type=%d, Not PPS adapter", wired_type);
				oplus_cpa_switch_end(chip->cpa_topic, CHG_PROTOCOL_PPS);
				return;
			}
			/* alreday re-into pps_charging during thread blocking period */
			if (chip->pps_online)
				return;
		} else {
			chg_err("wired_type=%d, Not PPS adapter", wired_type);
			oplus_cpa_switch_end(chip->cpa_topic, CHG_PROTOCOL_PPS);
			return;
		}
	}

	oplus_pps_switch_to_normal(chip);
	oplus_pps_set_charging(chip, false);
	oplus_pps_votable_reset(chip);
	if (chip->pps_disable) {
		oplus_pps_cpa_switch_end(chip);
		return;
	}
	if (chip->pps_ic == NULL || chip->cp_ic == NULL || !is_gauge_topic_available(chip)) {
		oplus_pps_cpa_switch_end(chip);
		return;
	}

	oplus_pps_set_online(chip, true);
	oplus_pps_set_online_keep(chip, true);

	if (chip->pdsvooc_id_adapter) {
		rc = oplus_pps_verify_adapter(chip);
		if (rc <= 0) {
			/* rc=0 if PPS_GET_AUTHENTICATE failed */
			chg_err("adapter verify fail, rc=%d\n", rc);
			chip->oplus_pps_adapter = false;
		} else {
			chg_info("adapter verify pass\n");
			chip->oplus_pps_adapter = true;
		}
	}
	chg_info("oplus_pps_adapter=%s\n", chip->oplus_pps_adapter ? "true" : "false");

	rc = oplus_pps_get_pdo_info(chip, (u32*)chip->pdo, PPS_PDO_MAX);
	if (rc < 0) {
		chg_err("pps get pdo info error\n");
		goto err;
	}
	chip->pdo_num = PPS_PDO_MAX;
	oplus_pps_publish_adapter_info(chip);

	for (i = 0; i < chip->pdo_num; i++) {
		target_vbus_max = chip->config.target_vbus_mv;

		if (chip->pdo[i].pdo_type != USBPD_PDMSG_PDOTYPE_AUGMENTED)
			continue;

		chg_err("pdo[%d]: voltage[%u-%umV,  max current[%umA]\n", i,
			 PPS_PDO_VOL_MIN(chip->pdo[i].min_voltage100mv),
			 PPS_PDO_VOL_MAX(chip->pdo[i].max_voltage100mv),
			 PPS_PDO_CURR_MAX(chip->pdo[i].max_current50ma));

		if (PPS_PDO_VOL_MIN(chip->pdo[i].min_voltage100mv) > PPS_START_DEF_VOL_MV) {
			chg_err("The output voltage does not support 5V\n");
			continue;
		}

		if (target_vbus_max <= PPS_PDO_VOL_MAX(chip->pdo[i].max_voltage100mv) &&
		    target_vbus_max >= PPS_PDO_VOL_MIN(chip->pdo[i].min_voltage100mv)) {
			pdo_ok = true;
			if (PPS_PDO_CURR_MAX(chip->pdo[i].max_current50ma) > max_curr) {
				max_curr = PPS_PDO_CURR_MAX(chip->pdo[i].max_current50ma);
				chip->adapter_max_curr = max_curr;
				chg_info("adapter max_curr = %d\n", max_curr);
			}
		}
	}
	if (!pdo_ok) {
		chg_err("The pdo range of the current adapter does not support PPS charging\n");
		goto err;
	}
	rc = oplus_cpa_protocol_set_power(chip->cpa_topic, CHG_PROTOCOL_PPS, max_curr * target_vbus_max / OPLUS_PPS_UW_MV_TRANSFORM);
	if (!rc) {
		max_power = oplus_cpa_protocol_get_power(chip->cpa_topic, CHG_PROTOCOL_PPS);
		if (max_power > 0) {
			chip->config.pps_target_curr_max_ma = max_curr;
			max_curr = max_power * OPLUS_PPS_UW_MV_TRANSFORM / target_vbus_max;
			chg_info("final max_curr = %d\n", max_curr);
		}
	}
	vote(chip->pps_curr_votable, BASE_MAX_VOTER, true, max_curr, false);
	oplus_pps_variables_init(chip);
	oplus_pps_vfa_allow(chip, true);
	if (!oplus_pps_charge_allow_check(chip)) {
		chg_info("pps charge not allow, exit pps mode\n");
		goto exit;
	}

	cp_mode = vbus_to_cp_work_mode(chip->config.target_vbus_mv);
	if (cp_mode == CP_WORK_MODE_UNKNOWN) {
		chg_err("can't find a suitable CP working mode\n");
		goto err;
	}
	rc = oplus_pps_cp_check_work_mode_support(chip, cp_mode);
	if (rc <= 0) {
		chg_err("cp not support %s mode, rc=%d\n", oplus_cp_work_mode_str(cp_mode), rc);
		goto err;
	}
	rc = oplus_pps_cp_set_work_mode(chip, cp_mode);
	if (rc < 0) {
		chg_err("cp set %s mode error, rc=%d\n", oplus_cp_work_mode_str(cp_mode), rc);
		goto err;
	}
	chip->cp_work_mode = cp_mode;
	oplus_pps_set_ic_pps_curr_vote(chip, cp_mode);
	oplus_pps_cp_watchdog_enable(chip, CP_WATCHDOG_TIMEOUT_5S);

	rc = oplus_pps_cp_adc_enable(chip, true);
	if (rc < 0)
		chg_err("enable the adc failed, rc = %d\n", rc);

	rc = oplus_pps_cp_get_adc_enable(chip, &enable_adc);
	if ((rc != -ENOTSUPP) && (rc < 0 || enable_adc == false)) {
		chg_err("adc is not enabled, rc = %d\n", rc);
		rc = oplus_pps_cp_adc_enable(chip, true);
		if (rc < 0 && rc != -ENOTSUPP)
			chg_err("enable the adc failed, rc = %d\n", rc);
	}

	rc = oplus_pps_cp_get_iin(chip, &max_curr);
	if (rc == -ENOTSUPP) {
		chip->support_cp_ibus = false;
		chg_info("Can't get Ibus, monitor Ibat in current_check");
	} else {
		chip->support_cp_ibus = true;
		chg_info("Could get Ibus, monitor Ibus in current_check");
	}

	local_time_ms = local_clock() / LOCAL_T_NS_TO_MS_THD;
	delta_time = local_time_ms - chip->boot_time;
	chg_info("local_time_ms %d, boot time %d delta time %d, adapter max curr %d \n", local_time_ms, chip->boot_time, delta_time, chip->adapter_max_curr);
	if (delta_time > 0 && delta_time + BOOT_TIME_CNTL_CURR_DEB_MS < BOOT_TIME_CNTL_CURR_MS
	    && chip->adapter_max_curr >= BOOT_ADAPTER_CURR_MIN && !chip->support_cp_ibus) {
	    chg_info("booting time, limit pps current %d \n", chip->adapter_max_curr - BOOT_SYS_CONSUME_MA);
	    vote(chip->pps_curr_votable, ADAPTER_MAX_POWER, true, chip->adapter_max_curr - BOOT_SYS_CONSUME_MA, false);
	    schedule_delayed_work(&chip->boot_curr_limit_work, msecs_to_jiffies(BOOT_TIME_CNTL_CURR_MS - delta_time));
	}

	if (oplus_chg_get_fcs_support_flags()) {
		if (is_disable_charger_vatable_available(chip))
			vote(chip->chg_disable_votable, PPS_VOTER, true, 1, false);

		oplus_pps_set_fcs_icl_vote(PPS_FCS_ICL1_MA);
	} else {
		if (is_wired_suspend_votable_available(chip))
			vote(chip->wired_suspend_votable, PPS_VOTER, true, 1, false);
	}
	rc = oplus_pps_pdo_set(chip, PPS_START_DEF_VOL_MV, PPS_START_DEF_CURR_MA);
	if (rc < 0) {
		chg_err("pdo set error, rc=%d\n", rc);
		goto err;
	}

	if (chip->pps_online) {
		oplus_pps_publish_cpa_power(chip);
		oplus_pps_publish_adapter_power(chip);
	}

	oplus_pps_sub_btb_connnect_check(chip);
	schedule_delayed_work(&chip->monitor_work, msecs_to_jiffies(PPS_GET_CP_VIN_DELAY));

	return;
err:
	chg_err("error, pps exit\n");
	oplus_pps_force_exit(chip);
	oplus_pps_set_online_keep(chip, false);
	return;

exit:
	oplus_pps_exit_pps_mode(chip);
	chip->cp_work_mode = CP_WORK_MODE_UNKNOWN;
	chip->cp_ratio = 0;
}

enum {
	OPLUS_PPS_VOLT_UPDATE_100MV = 100,
	OPLUS_PPS_VOLT_UPDATE_200MV = 200,
	OPLUS_PPS_VOLT_UPDATE_500MV = 500,
	OPLUS_PPS_VOLT_UPDATE_1V = 1000,
	OPLUS_PPS_VOLT_UPDATE_2V = 2000,
	OPLUS_PPS_VOLT_UPDATE_5V = 5000,
};

static int oplus_pps_charge_start(struct oplus_pps *chip)
{
	union mms_msg_data data = { 0 };
	int vbat_mv;
	int rc;
	int target_vbus, update_size, req_vol;
	int cp_vin, delta_vbus, cp_vout;
	int pps_vbus;
	static int retry_count = 0;
	int batt_num;
	int soc;
	const char temp_region[] = "temp_region";
	const char allow_soc[] = "allow_soc";

#define PPS_START_VOL_THR_4_TO_1_600MV	600
#define PPS_START_VOL_THR_3_TO_1_400MV	400
#define PPS_START_VOL_THR_2_TO_1_450MV	450
#define PPS_START_VOL_THR_1_TO_1_300MV	300
#define PPS_START_RETAY_MAX		3
#define PPS_START_CHECK_DELAY_MS	500
#define PPS_START_PDO_DELAY_MS		500
#define PPS_START_COLD_OFFSET_SOC	75

	batt_num = oplus_gauge_get_batt_num();
	if (chip->lift_vbus_use_cpvout) {
		rc = oplus_pps_cp_get_vout(chip, &cp_vout);
		if (rc < 0) {
			chg_err("can't get cp output voltage, rc=%d\n", rc);
			return rc;
		}
		vbat_mv = cp_vout * batt_num;
	} else {
		rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX, &data, true);
		if (rc < 0) {
			chg_err("can't get vbat, rc=%d\n", rc);
			return rc;
		}
		vbat_mv = data.intval * batt_num;
	}

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOC, &data, false);
	if (rc < 0) {
		chg_err("can't get soc, rc=%d\n", rc);
	}
	soc = data.intval;

	rc = oplus_pps_temp_cur_range_init(chip);
	if (rc < 0) {
		chg_err("temp range init fail, rc=%d\n", rc);
		return rc;
	}

	switch (chip->cp_work_mode) {
	case CP_WORK_MODE_4_TO_1:
		delta_vbus = PPS_START_VOL_THR_4_TO_1_600MV;
		break;
	case CP_WORK_MODE_3_TO_1:
		delta_vbus = PPS_START_VOL_THR_3_TO_1_400MV;
		break;
	case CP_WORK_MODE_2_TO_1:
		delta_vbus = PPS_START_VOL_THR_2_TO_1_450MV;
		break;
	case CP_WORK_MODE_BYPASS:
		delta_vbus = PPS_START_VOL_THR_1_TO_1_300MV;
		if (chip->delta_vbus[PPS_TEMP_RANGE_LITTLE_COLD] &&
		    chip->pps_temp_cur_range == PPS_TEMP_RANGE_LITTLE_COLD && soc < PPS_START_COLD_OFFSET_SOC) {
			delta_vbus = chip->delta_vbus[PPS_TEMP_RANGE_LITTLE_COLD];
			chg_err("delta_vbus = %d\n", delta_vbus);
		}
		break;
	default:
		chg_err("unsupported cp work mode, mode=%d\n", chip->cp_work_mode);
		return -ENOTSUPP;
	}
	target_vbus = ((vbat_mv * chip->cp_ratio) / 100) * 100 + delta_vbus;

	rc = oplus_pps_cp_get_vin(chip, &cp_vin);
	if (rc < 0) {
		chg_err("can't get cp input voltage, rc=%d\n", rc);
		return rc;
	}
	chg_err("cp_work_mode=%d, vbat=%d, cp_vin=%d\n", chip->cp_work_mode, vbat_mv, cp_vin);

	if ((cp_vin >= target_vbus && cp_vin <= (target_vbus + OPLUS_PPS_VOLT_UPDATE_200MV)) ||
	    chip->mos_on_check) {
		if (chip->mos_on_check) {
			bool work_start;

			chip->mos_on_check = false;
			rc = oplus_pps_cp_get_work_status(chip, &work_start);
			if (rc < 0) {
				chg_err("can't get cp work status, rc=%d\n", rc);
			} else {
				if (work_start) {
					if (oplus_chg_get_fcs_support_flags() && is_wired_suspend_votable_available(chip))
						vote(chip->wired_suspend_votable, PPS_VOTER, true, 1, false);
					if (chip->temperature_strategy)
						oplus_chg_strategy_init(chip->temperature_strategy);

					if (chip->oplus_pps_adapter)
						chip->strategy = chip->oplus_curve_strategy;
					else
						chip->strategy = chip->third_curve_strategy;
					oplus_chg_strategy_set_process_data(chip->strategy, temp_region, chip->pps_temp_cur_range);
					oplus_chg_strategy_set_process_data(chip->strategy, allow_soc, chip->allow_check_soc);
					rc = oplus_chg_strategy_init(chip->strategy);
					if (rc < 0) {
						chg_err("strategy_init error, not support pps fast charge\n");
						return rc;
					}
					oplus_pps_ccd_strategy_init(chip);
					if (chip->oplus_pps_adapter)
						rc = oplus_chg_strategy_init(chip->oplus_lcf_strategy);
					else
						rc = oplus_chg_strategy_init(chip->third_lcf_strategy);
					if (rc < 0)
						chg_err("lcf_strategy_init error, not support low curr full\n");

					retry_count = 0;
					oplus_pps_set_charging(chip, true);
					oplus_pps_vfa_reset(chip);
					if (chip->led_on && chip->adapter_max_curr > LED_ON_SYS_CONSUME_MA &&
					    chip->pps_charging && !chip->support_cp_ibus)
						vote(chip->pps_curr_votable, LED_ON_VOTER, true,
						     chip->adapter_max_curr - LED_ON_SYS_CONSUME_MA, false);
					else
						vote(chip->pps_curr_votable, LED_ON_VOTER, false, 0, false);
					if (chip->oplus_pps_adapter)
						chip->target_vbus_mv = chip->config.target_vbus_mv;
					else
						chip->target_vbus_mv = chip->vol_set_mv;
					chip->timer.monitor_jiffies = jiffies;
					schedule_delayed_work(&chip->current_work, 0);
					oplus_pps_cp_reg_dump(chip);
					return 0;
				}
				chg_err("cp not work, retry=%d\n", retry_count);
			}
			if (retry_count >= PPS_START_RETAY_MAX) {
				retry_count = 0;
				oplus_pps_cp_set_work_start(chip, false);
				oplus_pps_cp_enable(chip, false);
				oplus_pps_cp_watchdog_enable(chip, CP_WATCHDOG_DISABLE);
				oplus_pps_cp_adc_enable(chip, false);
				oplus_pps_cp_reg_dump(chip);
				chg_err("cp not working\n");
				return -EFAULT;
			}
			retry_count++;
			return PPS_START_CHECK_DELAY_MS;
		}
		if (oplus_chg_get_fcs_support_flags()) {
			oplus_pps_set_fcs_icl_vote(PPS_FCS_ICL2_MA);
			rc = oplus_pps_pdo_set(chip, chip->vol_set_mv, PPS_START_CP_CURR_MA);
			if (rc < 0) {
				chg_err("pdo set error, rc=%d\n", rc);
				return rc;
			}
		}
		rc = oplus_pps_cp_enable(chip, true);
		if (rc < 0) {
			chg_err("set cp enable error, rc=%d\n", rc);
			return rc;
		}
		rc = oplus_pps_cp_set_work_start(chip, true);
		if (rc < 0) {
			chg_err("set cp work start error, rc=%d\n", rc);
			return rc;
		}
		chip->mos_on_check = true;

		return PPS_START_CHECK_DELAY_MS;
	}

	retry_count = 0;
	chip->mos_on_check = false;

	if (abs(cp_vin - target_vbus) >= OPLUS_PPS_VOLT_UPDATE_2V)
		update_size = OPLUS_PPS_VOLT_UPDATE_2V;
	else if (abs(cp_vin - target_vbus) >= OPLUS_PPS_VOLT_UPDATE_1V)
		update_size = OPLUS_PPS_VOLT_UPDATE_1V;
	else if (abs(cp_vin - target_vbus) >= OPLUS_PPS_VOLT_UPDATE_500MV)
		update_size = OPLUS_PPS_VOLT_UPDATE_500MV;
	else if (abs(cp_vin - target_vbus) >= OPLUS_PPS_VOLT_UPDATE_200MV)
		update_size = OPLUS_PPS_VOLT_UPDATE_200MV;
	else
		update_size = OPLUS_PPS_VOLT_UPDATE_100MV;

	req_vol = chip->vol_set_mv;
	if (cp_vin < target_vbus)
		req_vol += update_size;
	else
		req_vol -= update_size;
	chg_info("cp_vin=%d, target_vbus=%d, update_size=%d, req_vol=%d\n",
		 cp_vin, target_vbus, update_size, req_vol);

	if (req_vol > chip->config.target_vbus_mv) {
		chip->count.req_vol_over++;
		req_vol = chip->config.target_vbus_mv;
		if (chip->count.req_vol_over > PPS_START_RETAY_MAX) {
			chg_err("target_vbus=%d, req_vol=%d, req_vol_over = %d req vol failed\n",
				 target_vbus, req_vol, chip->count.req_vol_over);
			return -EFAULT;
		}
	}

	rc = oplus_pps_cp_set_iin(chip, PPS_START_DEF_CURR_MA);
	if (rc < 0) {
		chg_err("set cp input current error, rc=%d\n", rc);
		return rc;
	}

#define PPS_STATUS_VLOT_SHAKE	500
	if (chip->enable_pps_status && !chip->support_cp_ibus && chip->support_pps_status) {
		rc = oplus_pps_get_pps_status_info(chip, &chip->pps_status_info);
		if (rc < 0) {
			chg_err("get pps status error, rc=%d\n", rc);
			chip->support_pps_status = false;
		} else {
			pps_vbus = PPS_STATUS_VOLT(chip->pps_status_info) * 20;
			chip->pps_curr_ma_from_pps_status = 0;
			chg_info("PPS_STATUS: volt = %d, vol_set_mv = %d\n",
							pps_vbus, chip->vol_set_mv);

			if ((chip->vol_set_mv > 0) && (pps_vbus > 0)) {
				if (pps_vbus < chip->vol_set_mv + PPS_STATUS_VLOT_SHAKE) {
					chip->support_pps_status = true;
				} else {
					chip->support_pps_status = false;
				}
			}
		}
	}

	rc = oplus_pps_pdo_set(chip, req_vol, PPS_START_DEF_CURR_MA);
	if (rc < 0) {
		chg_err("pdo set error, rc=%d\n", rc);
		return rc;
	}

	return PPS_START_PDO_DELAY_MS;
}

static void oplus_pps_reset_temp_range(struct oplus_pps *chip)
{
	chip->limits.pps_normal_high_temp =
		chip->limits.default_pps_normal_high_temp;
	chip->limits.pps_little_cold_temp =
		chip->limits.default_pps_little_cold_temp;
	chip->limits.pps_cool_temp = chip->limits.default_pps_cool_temp;
	chip->limits.pps_little_cool_temp =
		chip->limits.default_pps_little_cool_temp;
	chip->limits.pps_little_cool_high_temp = chip->limits.default_pps_little_cool_high_temp;
	chip->limits.pps_normal_low_pre_temp =
		chip->limits.default_pps_normal_low_pre_temp;
	chip->limits.pps_normal_low_temp =
		chip->limits.default_pps_normal_low_temp;
}

static int oplus_pps_set_current_warm_range(struct oplus_pps *chip,
					     int vbat_temp_cur)
{
	int ret = chip->limits.pps_strategy_normal_current;

	if (chip->limits.pps_batt_over_high_temp != -EINVAL &&
	    vbat_temp_cur > chip->limits.pps_batt_over_high_temp) {
		chip->limits.pps_strategy_change_count++;
		if (chip->limits.pps_strategy_change_count >=
		    PPS_TEMP_OVER_COUNTS) {
			chip->limits.pps_strategy_change_count = 0;
			chip->pps_fastchg_batt_temp_status =
				PPS_BAT_TEMP_EXIT;
			ret = chip->limits.pps_over_high_or_low_current;
			chg_err("temp_over:%d", vbat_temp_cur);
		}
	} else if (vbat_temp_cur < chip->limits.pps_normal_high_temp) {
		chip->limits.pps_strategy_change_count++;
		if (chip->limits.pps_strategy_change_count >=
		    PPS_TEMP_OVER_COUNTS) {
			chip->limits.pps_strategy_change_count = 0;
			chip->pps_fastchg_batt_temp_status =
				PPS_BAT_TEMP_SWITCH_CURVE;
			chip->pps_temp_cur_range = PPS_TEMP_RANGE_INIT;
			ret = chip->limits.pps_strategy_normal_current;
			oplus_pps_reset_temp_range(chip);
			(void)oplus_chg_strategy_init(chip->strategy);
			chip->limits.pps_normal_high_temp +=
				PPS_TEMP_WARM_RANGE_THD;
			chg_err("switch temp range:%d", vbat_temp_cur);
		}

	} else {
		chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_WARM;
		chip->pps_temp_cur_range = PPS_TEMP_RANGE_WARM;
		ret = chip->limits.pps_strategy_normal_current;
	}

	return ret;
}

static int
oplus_pps_set_current_temp_normal_range(struct oplus_pps *chip,
					 int vbat_temp_cur)
{
	int ret = chip->limits.pps_strategy_normal_current;
	int batt_vol;
	union mms_msg_data data = { 0 };
	int rc;

	if (!is_gauge_topic_available(chip)) {
		chg_err("gauge topic not found\n");
		vote(chip->pps_disable_votable, NO_DATA_VOTER, true, 1, false);
		return -ENODEV;
	}
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX, &data, false);
	if (rc < 0) {
		chg_err("can't get vbat, rc=%d\n", rc);
		vote(chip->pps_disable_votable, NO_DATA_VOTER, true, 1, false);
		return rc;
	}
	batt_vol = data.intval;

	switch (chip->pps_fastchg_batt_temp_status) {
	case PPS_BAT_TEMP_NORMAL_HIGH:
		if (vbat_temp_cur >=
		    chip->limits.pps_strategy_batt_high_temp0) {
			chip->pps_fastchg_batt_temp_status =
				PPS_BAT_TEMP_HIGH0;
			ret = chip->limits.pps_strategy_high_current0;
		} else if (vbat_temp_cur >= chip->limits.pps_normal_low_temp) {
			chip->pps_fastchg_batt_temp_status =
				PPS_BAT_TEMP_NORMAL_HIGH;
			ret = chip->limits.pps_strategy_normal_current;
		} else {
			chip->pps_fastchg_batt_temp_status =
				PPS_BAT_TEMP_NORMAL_LOW;
			chip->pps_temp_cur_range = PPS_TEMP_RANGE_NORMAL_LOW;
			ret = chip->limits.pps_strategy_normal_current;
			oplus_pps_reset_temp_range(chip);
			chip->limits.pps_normal_low_temp +=
				PPS_TEMP_WARM_RANGE_THD;
		}
		break;
	case PPS_BAT_TEMP_HIGH0:
		if (vbat_temp_cur >=
		    chip->limits.pps_strategy_batt_high_temp1) {
			chip->pps_fastchg_batt_temp_status =
				PPS_BAT_TEMP_HIGH1;
			ret = chip->limits.pps_strategy_high_current1;
		} else if (vbat_temp_cur <
			   chip->limits.pps_strategy_batt_low_temp0) {
			chip->pps_fastchg_batt_temp_status =
				PPS_BAT_TEMP_LOW0;
			ret = chip->limits.pps_strategy_low_current0;
		} else {
			chip->pps_fastchg_batt_temp_status =
				PPS_BAT_TEMP_HIGH0;
			ret = chip->limits.pps_strategy_high_current0;
		}
		break;
	case PPS_BAT_TEMP_HIGH1:
		if (vbat_temp_cur >=
		    chip->limits.pps_strategy_batt_high_temp2) {
			chip->pps_fastchg_batt_temp_status =
				PPS_BAT_TEMP_HIGH2;
			ret = chip->limits.pps_strategy_high_current2;
		} else if (vbat_temp_cur <
			   chip->limits.pps_strategy_batt_low_temp1) {
			chip->pps_fastchg_batt_temp_status =
				PPS_BAT_TEMP_LOW1;
			ret = chip->limits.pps_strategy_low_current1;
		} else {
			chip->pps_fastchg_batt_temp_status =
				PPS_BAT_TEMP_HIGH1;
			ret = chip->limits.pps_strategy_high_current1;
		}
		break;
	case PPS_BAT_TEMP_HIGH2:
		if (chip->limits.pps_normal_high_temp != -EINVAL &&
		    vbat_temp_cur >= chip->limits.pps_normal_high_temp) {
			chip->limits.pps_strategy_change_count++;
			if (chip->limits.pps_strategy_change_count >=
			    PPS_TEMP_OVER_COUNTS) {
				chip->limits.pps_strategy_change_count = 0;
				if (chip->ui_soc <= chip->limits.pps_warm_allow_soc &&
				    batt_vol <= chip->limits.pps_warm_allow_vol) {
					chip->pps_fastchg_batt_temp_status =
						PPS_BAT_TEMP_SWITCH_CURVE;
					chip->pps_temp_cur_range =
						PPS_TEMP_RANGE_INIT;
					ret = chip->limits.pps_strategy_high_current2;
					oplus_pps_reset_temp_range(chip);
					(void)oplus_chg_strategy_init(chip->strategy);
					chip->limits.pps_normal_high_temp -=
						PPS_TEMP_WARM_RANGE_THD;
					chg_err("switch temp range:%d", vbat_temp_cur);
				} else {
					chg_err("high temp_over:%d", vbat_temp_cur);
					chip->pps_fastchg_batt_temp_status =
						PPS_BAT_TEMP_EXIT;
					ret = chip->limits
						      .pps_over_high_or_low_current;
				}
			}
		} else if (chip->limits.pps_batt_over_high_temp != -EINVAL &&
			   vbat_temp_cur >=
				   chip->limits.pps_batt_over_high_temp) {
			chip->limits.pps_strategy_change_count++;
			if (chip->limits.pps_strategy_change_count >=
			    PPS_TEMP_OVER_COUNTS) {
				chip->limits.pps_strategy_change_count = 0;
				chip->pps_fastchg_batt_temp_status =
					PPS_BAT_TEMP_EXIT;
				ret = chip->limits.pps_over_high_or_low_current;
				chg_err("over_high temp_over:%d", vbat_temp_cur);
			}
		} else if (vbat_temp_cur <
			   chip->limits.pps_strategy_batt_low_temp2) {
			chip->pps_fastchg_batt_temp_status =
				PPS_BAT_TEMP_LOW2;
			ret = chip->limits.pps_strategy_low_current2;
		} else {
			chip->pps_fastchg_batt_temp_status =
				PPS_BAT_TEMP_HIGH2;
			ret = chip->limits.pps_strategy_high_current2;
		}
		break;
	case PPS_BAT_TEMP_LOW0:
		if (vbat_temp_cur >=
		    chip->limits.pps_strategy_batt_high_temp0) {
			chip->pps_fastchg_batt_temp_status =
				PPS_BAT_TEMP_HIGH0;
			ret = chip->limits.pps_strategy_high_current0;
		} else if (vbat_temp_cur <
			   chip->limits.pps_strategy_batt_low_temp0) { /* T<39C */
			chip->pps_fastchg_batt_temp_status =
				PPS_BAT_TEMP_NORMAL_HIGH;
			ret = chip->limits.pps_strategy_normal_current;
			oplus_pps_reset_temp_range(chip);
			chip->limits.pps_strategy_batt_low_temp0 +=
				PPS_TEMP_WARM_RANGE_THD;
		} else {
			chip->pps_fastchg_batt_temp_status =
				PPS_BAT_TEMP_LOW0;
			ret = chip->limits.pps_strategy_low_current0;
		}
		break;
	case PPS_BAT_TEMP_LOW1:
		if (vbat_temp_cur >=
		    chip->limits.pps_strategy_batt_high_temp1) {
			chip->pps_fastchg_batt_temp_status =
				PPS_BAT_TEMP_HIGH1;
			ret = chip->limits.pps_strategy_high_current2;
		} else if (vbat_temp_cur <
			   chip->limits.pps_strategy_batt_low_temp0) {
			chip->pps_fastchg_batt_temp_status =
				PPS_BAT_TEMP_LOW0;
			ret = chip->limits.pps_strategy_low_current0;
		} else {
			chip->pps_fastchg_batt_temp_status =
				PPS_BAT_TEMP_LOW1;
			ret = chip->limits.pps_strategy_low_current1;
		}
		break;
	case PPS_BAT_TEMP_LOW2:
		if (vbat_temp_cur >=
		    chip->limits.pps_strategy_batt_high_temp2) {
			chip->pps_fastchg_batt_temp_status =
				PPS_BAT_TEMP_HIGH2;
			ret = chip->limits.pps_strategy_high_current2;
		} else if (vbat_temp_cur <
			   chip->limits.pps_strategy_batt_low_temp1) {
			chip->pps_fastchg_batt_temp_status =
				PPS_BAT_TEMP_LOW1;
			ret = chip->limits.pps_strategy_low_current1;
		} else {
			chip->pps_fastchg_batt_temp_status =
				PPS_BAT_TEMP_LOW2;
			ret = chip->limits.pps_strategy_low_current2;
		}
		break;
	default:
		break;
	}
	chg_info("the ret: %d, the temp =%d, status = %d\r\n", ret,
		 vbat_temp_cur, chip->pps_fastchg_batt_temp_status);
	return ret;
}

static int
oplus_pps_set_current_temp_low_normal_range(struct oplus_pps *chip,
					     int vbat_temp_cur)
{
	int ret = chip->limits.pps_strategy_normal_current;
	int start_temp = chip->limits.pps_little_cool_temp;

	if (chip->limits.pps_little_cool_high_temp != -EINVAL)
		start_temp = chip->limits.pps_little_cool_high_temp;
	if (chip->limits.pps_normal_low_pre_temp != -EINVAL)
		start_temp = chip->limits.pps_normal_low_pre_temp;

	if (vbat_temp_cur < chip->limits.pps_normal_low_temp &&
	    vbat_temp_cur >= start_temp) { /* 20C<=T<35C */
		chip->pps_temp_cur_range = PPS_TEMP_RANGE_NORMAL_LOW;
		chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_NORMAL_LOW;
		ret = chip->limits.pps_strategy_normal_current;
	} else {
		if (vbat_temp_cur >= chip->limits.pps_normal_low_temp) {
			chip->limits.pps_normal_low_temp -=
				PPS_TEMP_LOW_RANGE_THD;
			chip->pps_fastchg_batt_temp_status =
				PPS_BAT_TEMP_NORMAL_HIGH;
			ret = chip->limits.pps_strategy_normal_current;
			chip->pps_temp_cur_range = PPS_TEMP_RANGE_NORMAL_HIGH;
		} else {
			if (chip->limits.pps_normal_low_pre_temp != -EINVAL) {
				chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_NORMAL_LOW_PRE;
				chip->pps_temp_cur_range = PPS_TEMP_RANGE_NORMAL_LOW_PRE;
				ret = chip->limits.pps_strategy_normal_current;
				oplus_pps_reset_temp_range(chip);
				chip->limits.pps_normal_low_pre_temp += PPS_TEMP_LOW_RANGE_THD;
			} else if (chip->limits.pps_little_cool_high_temp != -EINVAL) {
				chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_LITTLE_COOL_HIGH;
				chip->pps_temp_cur_range = PPS_TEMP_RANGE_LITTLE_COOL_HIGH;
				ret = chip->limits.pps_strategy_normal_current;
				oplus_pps_reset_temp_range(chip);
				chip->limits.pps_little_cool_high_temp += PPS_TEMP_LOW_RANGE_THD;
			} else {
				chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_LITTLE_COOL;
				chip->pps_temp_cur_range = PPS_TEMP_RANGE_LITTLE_COOL;
				ret = chip->limits.pps_strategy_normal_current;
				oplus_pps_reset_temp_range(chip);
				chip->limits.pps_little_cool_temp += PPS_TEMP_LOW_RANGE_THD;
			}
		}
	}

	return ret;
}

static int oplus_pps_set_current_temp_normal_low_pre_range(struct oplus_pps *chip, int vbat_temp_cur)
{
	int ret = 0;
	int start_temp = chip->limits.pps_little_cool_high_temp;

	if (chip->limits.pps_little_cool_high_temp == -EINVAL)
		start_temp = chip->limits.pps_little_cool_temp;

	if (vbat_temp_cur < chip->limits.pps_normal_low_pre_temp &&
	    vbat_temp_cur >= start_temp) {
		chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_NORMAL_LOW_PRE;
		ret = chip->limits.pps_strategy_normal_current;
		chip->pps_temp_cur_range = PPS_TEMP_RANGE_NORMAL_LOW_PRE;
	} else if (vbat_temp_cur >= chip->limits.pps_normal_low_pre_temp) {
		if (chip->ui_soc <= chip->limits.pps_strategy_soc_high) {
			chip->limits.pps_strategy_change_count++;
			if (chip->limits.pps_strategy_change_count >= PPS_TEMP_OVER_COUNTS) {
				chip->limits.pps_strategy_change_count = 0;
				chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_SWITCH_CURVE;
				chip->pps_temp_cur_range = PPS_TEMP_RANGE_INIT;
				(void)oplus_chg_strategy_init(chip->strategy);
				chg_err("switch temp range:%d", vbat_temp_cur);
			}
		} else {
			chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_NORMAL_LOW;
			chip->pps_temp_cur_range = PPS_TEMP_RANGE_NORMAL_LOW;
		}
		ret = chip->limits.pps_strategy_normal_current;
		oplus_pps_reset_temp_range(chip);
		chip->limits.pps_normal_low_pre_temp -= PPS_TEMP_LOW_RANGE_THD;
	} else {
		if (chip->limits.pps_little_cool_high_temp != -EINVAL) {
			chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_LITTLE_COOL_HIGH;
			chip->pps_temp_cur_range = PPS_TEMP_RANGE_LITTLE_COOL_HIGH;
			ret = chip->limits.pps_strategy_normal_current;
			oplus_pps_reset_temp_range(chip);
			chip->limits.pps_little_cool_high_temp += PPS_TEMP_LOW_RANGE_THD;
		} else {
			chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_LITTLE_COOL;
			chip->pps_temp_cur_range = PPS_TEMP_RANGE_LITTLE_COOL;
			ret = chip->limits.pps_strategy_normal_current;
			oplus_pps_reset_temp_range(chip);
			chip->limits.pps_little_cool_temp += PPS_TEMP_LOW_RANGE_THD;
		}
	}

	return ret;
}

static int oplus_pps_set_current_temp_little_cool_high_range(struct oplus_pps *chip, int vbat_temp_cur)
{
	int ret = 0;

	if (vbat_temp_cur < chip->limits.pps_little_cool_high_temp &&
	    vbat_temp_cur >= chip->limits.pps_little_cool_temp) {
		chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_LITTLE_COOL_HIGH;
		ret = chip->limits.pps_strategy_normal_current;
		chip->pps_temp_cur_range = PPS_TEMP_RANGE_LITTLE_COOL_HIGH;
	} else if (vbat_temp_cur >= chip->limits.pps_little_cool_high_temp) {
		if (chip->ui_soc <= chip->limits.pps_strategy_soc_high) {
			chip->limits.pps_strategy_change_count++;
			if (chip->limits.pps_strategy_change_count >= PPS_TEMP_OVER_COUNTS) {
				chip->limits.pps_strategy_change_count = 0;
				chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_SWITCH_CURVE;
				chip->pps_temp_cur_range = PPS_TEMP_RANGE_INIT;
				(void)oplus_chg_strategy_init(chip->strategy);
				chg_err("switch temp range:%d", vbat_temp_cur);
			}
		} else {
			if (chip->limits.pps_normal_low_pre_temp != -EINVAL &&
			    vbat_temp_cur < chip->limits.pps_normal_low_pre_temp) {
				chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_NORMAL_LOW_PRE;
				chip->pps_temp_cur_range = PPS_TEMP_RANGE_NORMAL_LOW_PRE;
			} else {
				chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_NORMAL_LOW;
				chip->pps_temp_cur_range = PPS_TEMP_RANGE_NORMAL_LOW;
			}
		}
		ret = chip->limits.pps_strategy_normal_current;
		oplus_pps_reset_temp_range(chip);
		chip->limits.pps_little_cool_high_temp -= PPS_TEMP_LOW_RANGE_THD;
	} else {
		chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_LITTLE_COOL;
		chip->pps_temp_cur_range = PPS_TEMP_RANGE_LITTLE_COOL;
		ret = chip->limits.pps_strategy_normal_current;
		oplus_pps_reset_temp_range(chip);
		chip->limits.pps_little_cool_temp += PPS_TEMP_LOW_RANGE_THD;
	}

	return ret;
}

static int
oplus_pps_set_current_temp_little_cool_range(struct oplus_pps *chip,
					      int vbat_temp_cur)
{
	int ret = 0;

	if (vbat_temp_cur < chip->limits.pps_little_cool_temp &&
	    vbat_temp_cur >= chip->limits.pps_cool_temp) { /* 12C<=T<20C */
		chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_LITTLE_COOL;
		ret = chip->limits.pps_strategy_normal_current;
		chip->pps_temp_cur_range = PPS_TEMP_RANGE_LITTLE_COOL;
	} else if (vbat_temp_cur >= chip->limits.pps_little_cool_temp) {
		if (chip->limits.pps_little_cool_high_temp != -EINVAL) {
			if (chip->ui_soc <= chip->limits.pps_strategy_soc_high) {
				chip->limits.pps_strategy_change_count++;
				if (chip->limits.pps_strategy_change_count >= PPS_TEMP_OVER_COUNTS) {
					chip->limits.pps_strategy_change_count = 0;
					chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_SWITCH_CURVE;
					chip->pps_temp_cur_range = PPS_TEMP_RANGE_INIT;
					(void)oplus_chg_strategy_init(chip->strategy);
					chg_err("switch temp range:%d", vbat_temp_cur);
				}
			} else {
				chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_LITTLE_COOL_HIGH;
				chip->pps_temp_cur_range = PPS_TEMP_RANGE_LITTLE_COOL_HIGH;
			}
			ret = chip->limits.pps_strategy_normal_current;
			oplus_pps_reset_temp_range(chip);
			chip->limits.pps_little_cool_temp -= PPS_TEMP_LOW_RANGE_THD;
		} else {
			chip->limits.pps_little_cool_temp -= PPS_TEMP_LOW_RANGE_THD;
			if (chip->limits.pps_normal_low_pre_temp != -EINVAL) {
				chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_NORMAL_LOW_PRE;
				chip->pps_temp_cur_range = PPS_TEMP_RANGE_NORMAL_LOW_PRE;
			} else {
				chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_NORMAL_LOW;
				chip->pps_temp_cur_range = PPS_TEMP_RANGE_NORMAL_LOW;
			}
			ret = chip->limits.pps_strategy_normal_current;
		}
	} else {
		if (chip->ui_soc <= chip->limits.pps_strategy_soc_high) {
			chip->limits.pps_strategy_change_count++;
			if (chip->limits.pps_strategy_change_count >=
			    PPS_TEMP_OVER_COUNTS) {
				chip->limits.pps_strategy_change_count = 0;
				chip->pps_fastchg_batt_temp_status =
					PPS_BAT_TEMP_SWITCH_CURVE;
				chip->pps_temp_cur_range =
					PPS_TEMP_RANGE_INIT;
				(void)oplus_chg_strategy_init(chip->strategy);
				chg_err("switch temp range:%d", vbat_temp_cur);
			}
		} else {
			chip->pps_fastchg_batt_temp_status =
				PPS_BAT_TEMP_COOL;
			chip->pps_temp_cur_range = PPS_TEMP_RANGE_COOL;
		}
		ret = chip->limits.pps_strategy_normal_current;
		oplus_pps_reset_temp_range(chip);
		chip->limits.pps_cool_temp += PPS_TEMP_LOW_RANGE_THD;
	}

	return ret;
}

static int oplus_pps_set_current_temp_cool_range(struct oplus_pps *chip,
						  int vbat_temp_cur)
{
	int ret = chip->limits.pps_strategy_normal_current;;
	if (chip->limits.pps_batt_over_low_temp != -EINVAL &&
	    vbat_temp_cur < chip->limits.pps_batt_over_low_temp) {
		chip->limits.pps_strategy_change_count++;
		if (chip->limits.pps_strategy_change_count >=
		    PPS_TEMP_OVER_COUNTS) {
			chip->limits.pps_strategy_change_count = 0;
			chip->pps_fastchg_batt_temp_status =
				PPS_BAT_TEMP_EXIT;
			ret = chip->limits.pps_over_high_or_low_current;
			chg_err("temp_over:%d", vbat_temp_cur);
		}
	} else if (vbat_temp_cur < chip->limits.pps_cool_temp &&
		   vbat_temp_cur >=
			   chip->limits.pps_little_cold_temp) { /* 5C <=T<12C */
		chip->pps_temp_cur_range = PPS_TEMP_RANGE_COOL;
		chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_COOL;
		ret = chip->limits.pps_strategy_normal_current;
	} else if (vbat_temp_cur >= chip->limits.pps_cool_temp) {
		chip->limits.pps_strategy_change_count++;
		if (chip->limits.pps_strategy_change_count >=
		    PPS_TEMP_OVER_COUNTS) {
			chip->limits.pps_strategy_change_count = 0;
			if (chip->ui_soc <= chip->limits.pps_strategy_soc_high) {
				chip->pps_fastchg_batt_temp_status =
					PPS_BAT_TEMP_SWITCH_CURVE;
				chip->pps_temp_cur_range =
					PPS_TEMP_RANGE_INIT;
				(void)oplus_chg_strategy_init(chip->strategy);
				chg_info("switch temp range:%d", vbat_temp_cur);
			} else {
				chip->pps_fastchg_batt_temp_status =
					PPS_BAT_TEMP_LITTLE_COOL;
				chip->pps_temp_cur_range =
					PPS_TEMP_RANGE_LITTLE_COOL;
			}
			oplus_pps_reset_temp_range(chip);
			chip->limits.pps_cool_temp -= PPS_TEMP_LOW_RANGE_THD;
		}

		ret = chip->limits.pps_strategy_normal_current;
	} else {
		chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_LITTLE_COLD;
		chip->pps_temp_cur_range = PPS_TEMP_RANGE_LITTLE_COLD;
		ret = chip->limits.pps_strategy_normal_current;
		oplus_pps_reset_temp_range(chip);
		chip->limits.pps_little_cold_temp += PPS_TEMP_LOW_RANGE_THD;
	}
	return ret;
}

static int
oplus_pps_set_current_temp_little_cold_range(struct oplus_pps *chip,
					      int vbat_temp_cur)
{
	int ret = chip->limits.pps_strategy_normal_current;

	if (chip->limits.pps_batt_over_low_temp != -EINVAL &&
	    vbat_temp_cur < chip->limits.pps_batt_over_low_temp) {
		chip->limits.pps_strategy_change_count++;
		if (chip->limits.pps_strategy_change_count >=
		    PPS_TEMP_OVER_COUNTS) {
			chip->limits.pps_strategy_change_count = 0;
			ret = chip->limits.pps_over_high_or_low_current;
			chip->pps_fastchg_batt_temp_status =
				PPS_BAT_TEMP_EXIT;
			chg_err("temp_over:%d", vbat_temp_cur);
		}
	} else if (vbat_temp_cur <
		   chip->limits.pps_little_cold_temp) { /* 0C<=T<5C */
		chip->pps_temp_cur_range = PPS_TEMP_RANGE_LITTLE_COLD;
		chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_LITTLE_COLD;
		ret = chip->limits.pps_strategy_normal_current;
	} else {
		chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_COOL;
		ret = chip->limits.pps_strategy_normal_current;
		chip->pps_temp_cur_range = PPS_TEMP_RANGE_COOL;
		oplus_pps_reset_temp_range(chip);
		chip->limits.pps_little_cold_temp -= PPS_TEMP_LOW_RANGE_THD;
	}

	return ret;
}

static int oplus_pps_get_batt_temp_curr(struct oplus_pps *chip)

{
	int ret;
	int vbat_temp_cur;
	union mms_msg_data data = { 0 };
	int temperature_current = 0, rc = 0;

	if (!is_gauge_topic_available(chip) || !chip->comm_topic) {
		chg_err("comm or gauge topic not found\n");
		vote(chip->pps_disable_votable, NO_DATA_VOTER, true, 1, false);
		return -ENODEV;
	}
	ret = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_SHELL_TEMP, &data, true);
	if (ret < 0) {
		chg_err("can't get shell temp data, rc=%d", ret);
	} else {
		chip->shell_temp = data.intval;
	}

	vbat_temp_cur = chip->shell_temp;
	ret = chip->limits.pps_strategy_normal_current;
	switch (chip->pps_temp_cur_range) {
	case PPS_TEMP_RANGE_WARM:
		ret = oplus_pps_set_current_warm_range(chip, vbat_temp_cur);
		break;
	case PPS_TEMP_RANGE_NORMAL_HIGH:
		ret = oplus_pps_set_current_temp_normal_range(chip,
							       vbat_temp_cur);
		break;
	case PPS_TEMP_RANGE_NORMAL_LOW:
		ret = oplus_pps_set_current_temp_low_normal_range(
			chip, vbat_temp_cur);
		break;
	case PPS_TEMP_RANGE_LITTLE_COOL:
		ret = oplus_pps_set_current_temp_little_cool_range(
			chip, vbat_temp_cur);
		break;
	case PPS_TEMP_RANGE_LITTLE_COOL_HIGH:
		ret = oplus_pps_set_current_temp_little_cool_high_range(
			chip, vbat_temp_cur);
		break;
	case PPS_TEMP_RANGE_NORMAL_LOW_PRE:
		ret = oplus_pps_set_current_temp_normal_low_pre_range(
			chip, vbat_temp_cur);
		break;
	case PPS_TEMP_RANGE_COOL:
		ret = oplus_pps_set_current_temp_cool_range(chip,
							     vbat_temp_cur);
		break;
	case PPS_TEMP_RANGE_LITTLE_COLD:
		ret = oplus_pps_set_current_temp_little_cold_range(
			chip, vbat_temp_cur);
		break;
	default:
		break;
	}

	if (chip->temperature_strategy) {
		rc = oplus_chg_strategy_get_data(chip->temperature_strategy, &temperature_current);
		if (rc == 0 && temperature_current > 0)
			ret = temperature_current;
	}

	if (ret > 0)
		vote(chip->pps_curr_votable, BATT_TEMP_VOTER, true, ret, false);
	else if (ret == 0)
		ret = -EINVAL;

	return ret;
}

static void oplus_pps_check_low_curr_temp_status(struct oplus_pps *chip)
{
	static int t_cnts = 0;
	int vbat_temp_cur;
	union mms_msg_data data = { 0 };
	int rc;

	if (!is_gauge_topic_available(chip) || !chip->comm_topic) {
		chg_err("gauge or comm topic not found\n");
		vote(chip->pps_disable_votable, NO_DATA_VOTER, true, 1, false);
		return;
	}

	rc = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_SHELL_TEMP, &data, false);
	if (rc < 0) {
		chg_err("can't get battery temp, rc=%d\n", rc);
		vote(chip->pps_disable_votable, NO_DATA_VOTER, true, 1, false);
		return;
	}
	vbat_temp_cur = data.intval;

	if (((vbat_temp_cur >=
	      chip->limits.pps_low_curr_full_normal_high_temp) ||
	     (vbat_temp_cur < chip->limits.pps_low_curr_full_cool_temp)) &&
	    (chip->pps_low_curr_full_temp_status !=
	     PPS_LOW_CURR_FULL_CURVE_TEMP_MAX)) {
		t_cnts++;
		if (t_cnts >= PPS_TEMP_OVER_COUNTS) {
			chip->pps_low_curr_full_temp_status =
				PPS_LOW_CURR_FULL_CURVE_TEMP_MAX;
			t_cnts = 0;
		}
	} else if ((vbat_temp_cur >=
		    chip->limits.pps_low_curr_full_normal_low_temp) &&
		   (chip->pps_low_curr_full_temp_status !=
		    PPS_LOW_CURR_FULL_CURVE_TEMP_NORMAL_HIGH)) {
		t_cnts++;
		if (t_cnts >= PPS_TEMP_OVER_COUNTS) {
			chip->pps_low_curr_full_temp_status =
				PPS_LOW_CURR_FULL_CURVE_TEMP_NORMAL_HIGH;
			t_cnts = 0;
		}
	} else if ((vbat_temp_cur >=
		    chip->limits.pps_low_curr_full_little_cool_temp) &&
		   (chip->pps_low_curr_full_temp_status !=
		    PPS_LOW_CURR_FULL_CURVE_TEMP_NORMAL_LOW)) {
		t_cnts++;
		if (t_cnts >= PPS_TEMP_OVER_COUNTS) {
			chip->pps_low_curr_full_temp_status =
				PPS_LOW_CURR_FULL_CURVE_TEMP_NORMAL_LOW;
			t_cnts = 0;
		}
	} else if (chip->pps_low_curr_full_temp_status !=
		   PPS_LOW_CURR_FULL_CURVE_TEMP_LITTLE_COOL) {
		t_cnts++;
		if (t_cnts >= PPS_TEMP_OVER_COUNTS) {
			chip->pps_low_curr_full_temp_status =
				PPS_LOW_CURR_FULL_CURVE_TEMP_LITTLE_COOL;
			t_cnts = 0;
		}
	} else {
		t_cnts = 0;
	}
}

static void oplus_pps_check_sw_full(struct oplus_pps *chip, struct puc_strategy_ret_data *data)
{
	int cool_sw_vth, normal_sw_vth, normal_hw_vth;
	struct oplus_pps_config *config = &chip->config;
	union mms_msg_data mms_data = { 0 };
	int batt_temp, vbat_mv;
	int rc;

#define PPS_FULL_COUNTS_COOL		6
#define PPS_FULL_COUNTS_SW		6
#define PPS_FULL_COUNTS_HW		3

	if (!chip->oplus_pps_adapter) {
		cool_sw_vth = chip->limits.pps_full_cool_sw_vbat_third;
		normal_sw_vth = chip->limits.pps_full_normal_sw_vbat_third;
		normal_hw_vth = chip->limits.pps_full_normal_hw_vbat_third;
	} else {
		cool_sw_vth = chip->limits.pps_full_cool_sw_vbat;
		normal_sw_vth = chip->limits.pps_full_normal_sw_vbat;
		normal_hw_vth = chip->limits.pps_full_normal_hw_vbat;
	}

	if (!is_gauge_topic_available(chip) || !chip->comm_topic) {
		chg_err("gauge or comm topic not found\n");
		vote(chip->pps_disable_votable, NO_DATA_VOTER, true, 1, false);
		return;
	}
	rc = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_SHELL_TEMP, &mms_data, false);
	if (rc < 0) {
		chg_err("can't get battery temp, rc=%d\n", rc);
		vote(chip->pps_disable_votable, NO_DATA_VOTER, true, 1, false);
		return;
	}
	batt_temp = mms_data.intval;
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX, &mms_data, false);
	if (rc < 0) {
		chg_err("can't get vbat, rc=%d\n", rc);
		vote(chip->pps_disable_votable, NO_DATA_VOTER, true, 1, false);
		return;
	}
	vbat_mv = mms_data.intval;

	if ((batt_temp < chip->limits.pps_cool_temp) && (vbat_mv > cool_sw_vth)) {
		chip->count.cool_fw++;
		if (chip->count.cool_fw >= PPS_FULL_COUNTS_COOL) {
			chip->count.cool_fw = 0;
			if (chip->config.pps_full_recheck_temp != -EINVAL &&
			    batt_temp <= config->pps_full_recheck_temp) {
				chip->pps_cool_full_temp_range = chip->pps_temp_cur_range;
				vote(chip->pps_not_allow_votable, CHG_FULL_COOL_VOTER, true, 1, false);
			} else {
				vote(chip->pps_disable_votable, CHG_FULL_VOTER, true, 1, false);
			}
			return;
		}
	} else {
		chip->count.cool_fw = 0;
	}

	if ((batt_temp >= chip->limits.pps_cool_temp) &&
	    (batt_temp < chip->limits.pps_batt_over_high_temp)) {
		if ((vbat_mv > normal_sw_vth) && data->last_gear) {
			chip->count.sw_full++;
			if (chip->count.sw_full >= PPS_FULL_COUNTS_SW) {
				chip->count.sw_full = 0;
				if (config->pps_full_recheck_temp != -EINVAL &&
				    batt_temp <= config->pps_full_recheck_temp) {
					chip->pps_cool_full_temp_range = chip->pps_temp_cur_range;
					vote(chip->pps_not_allow_votable, CHG_FULL_COOL_VOTER, true, 1, false);
				} else {
					vote(chip->pps_disable_votable, CHG_FULL_VOTER, true, 1, false);
				}
				return;
			}
		}
		if ((vbat_mv > normal_hw_vth)) {
			chip->count.hw_full++;
			if (chip->count.hw_full >= PPS_FULL_COUNTS_HW) {
				chip->count.hw_full = 0;
				if (config->pps_full_recheck_temp != -EINVAL &&
				    batt_temp <= config->pps_full_recheck_temp) {
					chip->pps_cool_full_temp_range = chip->pps_temp_cur_range;
					vote(chip->pps_not_allow_votable, CHG_FULL_COOL_VOTER, true, 1, false);
				} else {
					vote(chip->pps_disable_votable, CHG_FULL_VOTER, true, 1, false);
				}
				return;
			}
		}
	} else {
		chip->count.sw_full = 0;
		chip->count.hw_full = 0;
	}

	if ((chip->pps_fastchg_batt_temp_status == PPS_BAT_TEMP_WARM) &&
	    (vbat_mv > chip->limits.pps_full_warm_vbat)) {
		chip->count.warm_fw++;
		if (chip->count.warm_fw >= PPS_FULL_COUNTS_COOL) {
			chip->count.warm_fw = 0;
			vote(chip->pps_disable_votable, CHG_FULL_VOTER, true, 1, false);
			return;
		}
	} else {
		chip->count.warm_fw = 0;
	}
}

static void oplus_pps_check_low_curr_full(struct oplus_pps *chip)
{
	int i, temp_current, temp_vbatt, temp_status, iterm, vterm;
	bool low_curr = false;
	union mms_msg_data data = { 0 };
	int rc;

#define PPS_FULL_COUNTS_LOW_CURR	6

	/* third pps not support low current full check */
	if (!chip->oplus_pps_adapter)
		return;

	oplus_pps_check_low_curr_temp_status(chip);

	if (!is_gauge_topic_available(chip)) {
		chg_err("gauge topic not found\n");
		vote(chip->pps_disable_votable, NO_DATA_VOTER, true, 1, false);
		return;
	}
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX, &data, false);
	if (rc < 0) {
		chg_err("can't get vbat, rc=%d\n", rc);
		vote(chip->pps_disable_votable, NO_DATA_VOTER, true, 1, false);
		return;
	}
	temp_vbatt = data.intval;
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CURR, &data, false);
	if (rc < 0) {
		chg_err("can't get ibat, rc=%d\n", rc);
		vote(chip->pps_disable_votable, NO_DATA_VOTER, true, 1, false);
		return;
	}
	temp_current = -data.intval;

	temp_status = chip->pps_low_curr_full_temp_status;
	if (temp_status == PPS_LOW_CURR_FULL_CURVE_TEMP_MAX)
		return;

	for (i = 0; i < chip->low_curr_full_curves_temp[temp_status].full_curve_num; i++) {
		iterm = chip->low_curr_full_curves_temp[temp_status].full_curves[i].iterm;
		vterm = chip->low_curr_full_curves_temp[temp_status].full_curves[i].vterm;

		if ((temp_current <= iterm) && (temp_vbatt >= vterm)) {
			low_curr = true;
			break;
		}
	}

	if (low_curr) {
		chip->count.low_curr_full++;
		if (chip->count.low_curr_full > PPS_FULL_COUNTS_LOW_CURR) {
			chip->count.low_curr_full = 0;
			vote(chip->pps_disable_votable, CHG_FULL_VOTER, true, 1, false);
		}
	} else {
		chip->count.low_curr_full = 0;
	}
}

static void oplus_pps_check_timeout(struct oplus_pps *chip)
{
	unsigned long tmp_time;

	if (chip->plc_status == PLC_STATUS_ENABLE) {
		chip->timer.monitor_jiffies = jiffies;
		return;
	}

	tmp_time = jiffies - chip->timer.monitor_jiffies;
	chip->timer.monitor_jiffies = jiffies;
	if (chip->timer.pps_max_time_ms <= jiffies_to_msecs(tmp_time)) {
		chip->timer.pps_max_time_ms = 0;
		vote(chip->pps_disable_votable, TIMEOUT_VOTER, true, 1, false);
		return;
	}
	chip->timer.pps_max_time_ms -= jiffies_to_msecs(tmp_time);
}

static void oplus_pps_update_low_curr_full(struct oplus_pps *chip)
{
	int rc;
	int ret_val = 0;
	void *lcf_strategy;

	if (oplus_get_chg_spec_version() >= OPLUS_CHG_SPEC_VER_V3P7) {
		if (chip->oplus_lcf_strategy == NULL || chip->third_lcf_strategy == NULL ||
							chip->pps_disable_votable == NULL) {
			return;
		}
		lcf_strategy = chip->oplus_pps_adapter ? chip->oplus_lcf_strategy : chip->third_lcf_strategy;
		rc = oplus_chg_strategy_get_data(lcf_strategy, &ret_val);
		if (rc < 0) {
			chg_err("can't get lcf_strategy data, rc=%d\n", rc);
			return;
		}
		if (!ret_val)
			return;
		vote(chip->pps_disable_votable, CHG_FULL_VOTER, true, 1, false);
		chg_info("CHG_FULL_VOTER is true, diable pps charger\n");
	} else {
		/* third pps not support low current full check */
		if (!chip->oplus_pps_adapter) {
			return;
		}
		oplus_pps_check_low_curr_full(chip);
	}
}

static void oplus_pps_check_full(struct oplus_pps *chip, struct puc_strategy_ret_data *data)
{
	oplus_pps_check_sw_full(chip, data);
	oplus_pps_update_low_curr_full(chip);
}

static bool oplus_pps_btb_temp_check(struct oplus_pps *chip)
{
	bool btb_status = true;
	int btb_temp, usb_temp;
	bool shaft_btb_normal = true;

	btb_temp = oplus_wired_get_batt_btb_temp();
	usb_temp = oplus_wired_get_usb_btb_temp();
	shaft_btb_normal = oplus_wired_get_shaft_btb_is_normal();


	if (btb_temp >= PPS_BTB_OVER_TEMP || usb_temp >= PPS_BTB_OVER_TEMP || !shaft_btb_normal) {
		btb_status = false;
		chg_err("btb or usb temp over");
	}

	return btb_status;
}

#define PPS_UCP_SS_IBUS_MIN 600
#define PPS_IBAT_LOW_MIN	2000
#define PPS_IBAT_LOW_MIN_SINGLE	3000
#define PPS_IBAT_UCP_CNT	(7 / PPS_UPDATE_IBAT_TIME) /* 7s */
static void oplus_pps_check_ibat_safety(struct oplus_pps *chip)
{
	union mms_msg_data data = { 0 };
	int ibat;
	int ibat_hi_th;
	int rc;
	int ibat_low_th = (oplus_gauge_get_batt_num() == 1) ? PPS_IBAT_LOW_MIN_SINGLE : PPS_IBAT_LOW_MIN;
	int curr_ma;

#define PPS_IBAT_HIGH_CNT	8

	if (time_is_after_jiffies(chip->timer.ibat_jiffies + (unsigned long)(PPS_UPDATE_IBAT_TIME * HZ)))
		return;
	chip->timer.ibat_jiffies = jiffies;

	if (!is_gauge_topic_available(chip)) {
		chg_err("gauge topic not found\n");
		vote(chip->pps_disable_votable, NO_DATA_VOTER, true, 1, false);
		return;
	}
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CURR, &data, true);
	if (rc < 0) {
		chg_err("can't get ibat, rc=%d\n", rc);
		vote(chip->pps_disable_votable, NO_DATA_VOTER, true, 1, false);
		return;
	}
	ibat = data.intval;

	if (chip->support_cp_ibus) {
		rc = oplus_pps_cp_get_iin(chip, &curr_ma);
		if (rc < 0)
			chg_err("can't get cp iin, rc=%d\n", rc);
		chg_err("ibat now :%d, ibus: %d\n", ibat, curr_ma);
	} else {
		chg_err("ibat now :%d\n", ibat);
	}

	if (!chip->oplus_pps_adapter)
		ibat_hi_th = -chip->limits.pps_ibat_over_third;
	else
		ibat_hi_th = -chip->limits.pps_ibat_over_oplus;
	if (ibat < ibat_hi_th) {
		chip->count.ibat_high++;
		if (chip->count.ibat_high >= PPS_IBAT_HIGH_CNT) {
			chip->quit_pps_protocol = true;
			vote(chip->pps_disable_votable, IBAT_OVER_VOTER, true, 1, false);
			oplus_pps_push_err_info(chip, PPS_ERR_IBAT_OVER, ibat);
			return;
		}
	} else {
		chip->count.ibat_high = 0;
	}

	if (chip->plc_status == PLC_STATUS_ENABLE)
		return;

	if (ibat > ibat_low_th) {
		chip->count.ibat_low++;
		if (chip->count.ibat_low > PPS_IBAT_UCP_CNT) {
			chip->quit_pps_protocol = true;
			vote(chip->pps_disable_votable, IBAT_OVER_VOTER, true, 1, false);
			return;
		}
	} else {
		chip->count.ibat_low = 0;
	}

	if (chip->cp_ratio > 0 && !chip->ss_check && (-ibat / chip->cp_ratio) > PPS_UCP_SS_IBUS_MIN) {
		oplus_pps_cp_sstimeout_ucp_enable(chip, true);
		oplus_pps_cp_set_pmid2vout_ovp_enable(chip, true);
	}
}

static void oplus_pps_check_temp(struct oplus_pps *chip)
{
	struct timespec ts_current;
	union mms_msg_data data = { 0 };
	int batt_temp;
	int rc;
	bool shaft_btb_normal = true;
#define PPS_FG_TEMP_PROTECTION	800
#define PPS_TFG_OV_CNT		6
#define PPS_BTB_OV_CNT		8
#define PPS_TBATT_OV_CNT	1

	ts_current = oplus_current_kernel_time();
	if ((ts_current.tv_sec - chip->timer.temp_timer.tv_sec) < PPS_UPDATE_TEMP_TIME)
		return;

	chip->timer.temp_timer = ts_current;
	shaft_btb_normal = oplus_wired_get_shaft_btb_is_normal();

	if (!oplus_pps_btb_temp_check(chip)) {
		chip->count.btb_high++;
		if (chip->count.btb_high >= PPS_BTB_OV_CNT) {
			if (!shaft_btb_normal)
				oplus_wired_set_shaft_btb_over(true);

			chip->count.btb_high = 0;
			vote(chip->pps_disable_votable, BTB_TEMP_OVER_VOTER, true, 1, false);
			if (is_wired_icl_votable_available(chip))
				vote(chip->wired_icl_votable, BTB_TEMP_OVER_VOTER, true,
				     BTB_TEMP_OVER_MAX_INPUT_CUR, true);
		}
	} else {
		chip->count.btb_high = 0;
	}

	if (chip->pps_fastchg_batt_temp_status == PPS_BAT_TEMP_EXIT) {
		chg_err("pps battery temp out of range\n");
		chip->count.tbatt_over++;
		if (chip->count.tbatt_over >= PPS_TBATT_OV_CNT)
			vote(chip->pps_not_allow_votable, BATT_TEMP_VOTER, true, 1, false);
	} else {
		chip->count.tbatt_over = 0;
	}

	if (chip->pps_fastchg_batt_temp_status == PPS_BAT_TEMP_SWITCH_CURVE) {
		chg_err("pps battery temp switch curve range\n");
		vote(chip->pps_disable_votable, SWITCH_RANGE_VOTER, true, 1, false);
	}

	if (is_gauge_topic_available(chip)) {
		rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_REAL_TEMP, &data, false);
		if (rc >= 0) {
			batt_temp = data.intval;
			if (batt_temp >= PPS_FG_TEMP_PROTECTION) {
				chg_err("pps tfg > 80\n");
				chip->count.tfg_over++;
				if (chip->count.tfg_over >= PPS_TFG_OV_CNT) {
					chip->count.tfg_over = 0;
					vote(chip->pps_disable_votable, TFG_VOTER, true, 1, false);
					oplus_pps_push_err_info(chip, PPS_ERR_TFG_OVER, batt_temp);
				}
			} else {
				chip->count.tfg_over = 0;
			}
		}
	}
}

static void oplus_pps_set_cool_down_curr(struct oplus_pps *chip, int cool_down)
{
	int target_curr;

	if (chip->chg_ctrl_by_sale_mode) {
		if (!chip->support_cp_ibus) {
			if (ARRAY_SIZE(pps_cool_down_oplus_curve) >= 2)
				target_curr = pps_cool_down_oplus_curve[SALE_MODE_COOL_DOWN_VAL];
			else
				target_curr = pps_cool_down_oplus_curve[0];
		} else {
			if (ARRAY_SIZE(pps_cp_cool_down_oplus_curve) >= 2)
				target_curr = pps_cp_cool_down_oplus_curve[SALE_MODE_COOL_DOWN_VAL];
			else
				target_curr = pps_cp_cool_down_oplus_curve[0];
		}
	} else {
		if (!chip->support_cp_ibus) {
			if (cool_down >= ARRAY_SIZE(pps_cool_down_oplus_curve))
				cool_down = ARRAY_SIZE(pps_cool_down_oplus_curve) - 1;
			target_curr = pps_cool_down_oplus_curve[cool_down];
		} else {
			if (cool_down >= ARRAY_SIZE(pps_cp_cool_down_oplus_curve))
				cool_down = ARRAY_SIZE(pps_cp_cool_down_oplus_curve) - 1;
			target_curr = pps_cp_cool_down_oplus_curve[cool_down];
		}
	}

	if (chip->cp_ratio != 0) {
		target_curr /= chip->cp_ratio;
	} else {
		chg_err("unsupported cp work mode, mode=%d, ratio=%d\n", chip->cp_work_mode, chip->cp_ratio);
		return;
	}

	vote(chip->pps_curr_votable, SALE_MODE_VOTER, chip->chg_ctrl_by_sale_mode, target_curr, false);
	vote(chip->pps_curr_votable, COOL_DOWN_VOTER, !chip->chg_ctrl_by_sale_mode, target_curr, false);
}

int oplus_pps_level_to_current(struct oplus_mms *mms, int cool_down)
{
	struct oplus_pps *chip;
	int target_curr = -EINVAL;
	int cp_ratio = 0;

	if (mms == NULL)
		return -EINVAL;

	chip = oplus_mms_get_drvdata(mms);
	if (chip == NULL)
		return -EINVAL;
	cp_ratio = chip->cp_ratio;

	if (cool_down <= 0 || cp_ratio <= 0)
		return 0;

	if (!chip->support_cp_ibus) {
		if (cool_down >= ARRAY_SIZE(pps_cool_down_oplus_curve))
			cool_down = ARRAY_SIZE(pps_cool_down_oplus_curve) - 1;
		target_curr = pps_cool_down_oplus_curve[cool_down];
	} else {
		if (cool_down >= ARRAY_SIZE(pps_cp_cool_down_oplus_curve))
			cool_down = ARRAY_SIZE(pps_cp_cool_down_oplus_curve) - 1;
		target_curr = pps_cp_cool_down_oplus_curve[cool_down];
	}
	return target_curr / cp_ratio;
}

static void oplus_pps_set_batt_bal_curr(struct oplus_pps *chip)
{
	int target_curr;

	if (!chip->batt_bal_topic)
		return;

	target_curr = chip->batt_bal_curr_limit;
	if (chip->cp_ratio != 0) {
		target_curr /= chip->cp_ratio;
	} else {
		chg_err("unsupported cp work mode, mode=%d, ratio=%d\n",
			chip->cp_work_mode, chip->cp_ratio);
		return;
	}

	if (target_curr)
		vote(chip->pps_curr_votable, BATT_BAL_VOTER, true, target_curr, false);
	else
		vote(chip->pps_curr_votable, BATT_BAL_VOTER, false, 0, false);
}

static void oplus_pps_get_full_vth(struct oplus_pps *chip, int *hw_vth, int *sw_vth)
{
        int tbat = chip->shell_temp;

	if (!chip->oplus_pps_adapter)
		*hw_vth = chip->limits.pps_full_normal_hw_vbat_third;
	else
		*hw_vth = chip->limits.pps_full_normal_hw_vbat;

	if (tbat < chip->limits.pps_cool_temp) {
		if (!chip->oplus_pps_adapter)
			*sw_vth = chip->limits.pps_full_cool_sw_vbat_third;
		else
			*sw_vth = chip->limits.pps_full_cool_sw_vbat;
	} else if (chip->limits.pps_batt_over_high_temp != -EINVAL && tbat > chip->limits.pps_batt_over_high_temp) {
		*sw_vth = chip->limits.pps_full_warm_vbat;
	} else {
		if (!chip->oplus_pps_adapter)
			*sw_vth = chip->limits.pps_full_normal_sw_vbat_third;
		else
			*sw_vth = chip->limits.pps_full_normal_sw_vbat;
	}
}

static int oplus_pps_set_fcl_curr(struct oplus_pps *chip)
{
	int rc = 0;
	int ibus_ma = 0, ifg = 0, ibat = 0;
	int vbat_fcl = 0, vbat_fg = 0, vbat_cp = 0, vbat_r = 0;
	int sw_vth = 0, hw_vth = 0;
	int ibat_limit = 0, fcl_limit = 0;
	int vb_offset = 0, vbat_offset = 0;
	int curr_dec = 0, min_curr = 0;
	bool hw_status = false;
	static int limit_cnts = 0;
	union mms_msg_data data = { 0 };
#define ROUND_DOWN(a, n) (((a) / (n)) * (n))
#define FCL_LIMIT_CNTS 3

	if (!chip->fcl_support)
		return rc;

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CURR, &data, true);
	if (rc < 0) {
		chg_err("can't get ibat, rc=%d\n", rc);
		vote(chip->pps_disable_votable, NO_DATA_VOTER, true, 1, false);
		return rc;
	}
	ifg = -data.intval;
	ibat = abs(ifg);
	if (chip->support_cp_ibus) {
		rc = oplus_pps_cp_get_iin(chip, &ibus_ma);
		if (rc < 0) {
			chg_err("can't get cp iin, rc=%d\n", rc);
		} else {
			ibat = ibus_ma * chip->cp_ratio;
		}
	}
	oplus_gauge_get_vb_offset(chip->gauge_topic, &vb_offset);
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_FCL, &data, true);
	if (rc >= 0)
		vbat_fcl = data.intval;

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX, &data, true);
	if (rc >= 0)
		vbat_fg = data.intval;

	rc = oplus_pps_cp_get_vout(chip, &vbat_cp);
	if (rc < 0)
		vbat_cp = 0;

	if (vbat_fcl) {
		vbat_offset = vbat_fcl - (ifg * vb_offset / 1000);
		vbat_r = max(vbat_offset, vbat_fg);
	} else if (vbat_cp) {
		vbat_offset = vbat_cp - (ibus_ma * vb_offset / 1000);
		if (is_support_parallel_battery(chip->gauge_topic) == PARALLEL_CONNECT_TYPE)
			vbat_r = max(vbat_offset, vbat_fg);
		else
			vbat_r = vbat_offset;

	} else {
		vbat_r = vbat_fg;
	}
	oplus_pps_get_full_vth(chip, &hw_vth, &sw_vth);
	if (chip->fcl_trigger) {
		limit_cnts++;
		if (limit_cnts > FCL_LIMIT_CNTS && chip->curr_set_ma < chip->target_curr_ma) {
			limit_cnts = 0;
			chip->fcl_trigger = false;
		}
		return 0;
	}

	chip->fcl_trigger = oplus_gauge_get_fcl_curr(
		hw_vth, sw_vth, vbat_r, &curr_dec, &min_curr, &hw_status);

	if (chip->fcl_trigger && chip->cp_ratio) {
		ibat_limit = ibat / chip->cp_ratio - curr_dec;
		fcl_limit = ibat_limit > min_curr ?
			ibat_limit : min_curr;
		if (hw_status)
			oplus_pps_track_set_fcl_info(chip->dev, vbat_r, ibat, chip->shell_temp);
		else
			oplus_pps_track_set_fcl_info(chip->dev, 0, 0, 0);
		chg_info(" [%d, %d][%d, %d, %d][%d]\n", ibat, ibus_ma, vbat_r, vbat_fcl, vbat_cp, vb_offset);

		fcl_limit = ROUND_DOWN(fcl_limit, 50);
		if (fcl_limit)
			vote(chip->pps_curr_votable, LIMIT_FCL_VOTER, true,
				max(fcl_limit, PPS_START_CP_CURR_MA), false);
		else
			vote(chip->pps_curr_votable, LIMIT_FCL_VOTER, false, 0, false);
	}

	return 0;
}

static void oplus_pps_imp_check(struct oplus_pps *chip)
{
	int curr;
	int rc;

	/* third pps not support impedance check */
	if (!chip->oplus_pps_adapter)
		return;
	if (chip->plc_status == PLC_STATUS_ENABLE)
		return;

	if (!chip->imp_uint) {
		/* prevent triggering watchdog */
		rc = oplus_pps_get_pps_status_info(chip, &chip->pps_status_info);
		if (rc < 0)
			chg_err("pps get src info error\n");
		return;
	}

	curr = oplus_imp_uint_get_allow_curr(chip->imp_uint);
	if (curr < 0) {
		chg_err("can't get pps channel impedance current limite, exit pps charging\n");
		vote(chip->pps_disable_votable, IMP_VOTER, true, 1, false);
	} else if (curr == 0) {
		chg_err("pps channel impedance is too high, exit pps charging\n");
		vote(chip->pps_disable_votable, IMP_VOTER, true, 1, false);
	} else {
		vote(chip->pps_curr_votable, IMP_VOTER, true, curr, false);
	}
}

static void oplus_pps_watchdog(struct oplus_pps *chip)
{
	int rc;
	bool work_start;

	rc = oplus_pps_cp_get_work_status(chip, &work_start);
	if (rc < 0) {
		chg_err("can't get cp work status, rc=%d\n", rc);
		return;
	}
	if (!work_start)
		return;
	rc = oplus_pps_cp_watchdog_reset(chip);
	if  (rc < 0)
		chg_err("can't reset cp watchdog, rc=%d\n", rc);
}

static void oplus_pps_protection_check(struct oplus_pps *chip, struct puc_strategy_ret_data *data)
{
	oplus_pps_check_timeout(chip);
	oplus_pps_check_full(chip, data);
	oplus_pps_check_ibat_safety(chip);
	oplus_pps_check_temp(chip);
	oplus_pps_imp_check(chip);
	oplus_pps_watchdog(chip);
}

enum oplus_ibat_check_result {
	OPLUS_CURR_LOW,
	OPLUS_CURR_OK,
	OPLUS_CURR_HIGH,
	OPLUS_CURR_OVER,
	OPLUS_CURR_ABNOR,
};

static int oplus_third_pps_current_check(struct oplus_pps *chip)
{
	int rc;
	int curr_ma;
	int curr_ok_thd;
	int curr_high_thd;
	int curr_over_thd;
	int curr_abnormal_thd;
	int curr_ratio;
	int delta_curr_ok;
	union mms_msg_data msg_data = { 0 };

#define OPLUS_PPS_OK_CURRENT_DEBOUNCE (200)
#define OPLUS_PPS_IBUS_OK_CURR_DEBOUNCE (100)
#define OPLUS_PPS_OVER_CURRENT_THD (500)
#define OPLUS_PPS_MOS_ABNORMAL_CURR (100)
#define OPLUS_PPS_CP_DISCONNECT_CURR (300)

	if (!chip->support_cp_ibus) {
		rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CURR, &msg_data, true);
		if (unlikely(rc < 0)) {
			chg_err("can't get ibat, rc=%d\n", rc);
			return OPLUS_CURR_ABNOR;
		} else {
			curr_ma = -msg_data.intval;
			if (chip->enable_pps_status && chip->support_pps_status
						    && (chip->pps_curr_ma_from_pps_status > 0)) {
				chg_info("curr_ma:%d, pps_curr_ma_from_pps_status:%d\n",
						curr_ma, chip->pps_curr_ma_from_pps_status);
				curr_ma = (curr_ma < chip->pps_curr_ma_from_pps_status)
						? chip->pps_curr_ma_from_pps_status : curr_ma;
			}
			/* target_curr_ma control ibus, convert thd fit ibat */
			curr_ratio = chip->cp_ratio;
		}
		delta_curr_ok = OPLUS_PPS_OK_CURRENT_DEBOUNCE;
		curr_abnormal_thd = OPLUS_PPS_MOS_ABNORMAL_CURR;
	} else {
		rc = oplus_pps_cp_get_iin(chip, &curr_ma);
		if (unlikely(rc < 0)) {
			chg_err("can't get ibat, rc=%d\n", rc);
			return OPLUS_CURR_ABNOR;
		}
		/* got ibus here, needn't convert */
		curr_ratio = 1;
		if (!chip->request_vbus_too_low_flag)
			delta_curr_ok = OPLUS_PPS_IBUS_OK_CURR_DEBOUNCE;
		else
			delta_curr_ok = OPLUS_PPS_OK_CURRENT_DEBOUNCE;

		curr_abnormal_thd = OPLUS_PPS_CP_DISCONNECT_CURR;
	}

	curr_ok_thd = chip->target_curr_ma * curr_ratio - delta_curr_ok;
	curr_high_thd = chip->target_curr_ma * curr_ratio + delta_curr_ok;
	curr_over_thd = chip->target_curr_ma * curr_ratio + OPLUS_PPS_OVER_CURRENT_THD;

	if (curr_ma > curr_over_thd) {
		rc = OPLUS_CURR_OVER;
		chg_err("curr=%d, curr_over_thd=%d", curr_ma, curr_over_thd);
	} else if (curr_ma > curr_high_thd) {
		rc = OPLUS_CURR_HIGH;
		chg_err("curr=%d, ibat_ok_thd=%d", curr_ma, curr_high_thd);
	} else if (curr_ma > curr_ok_thd) {
		rc = OPLUS_CURR_OK;
	} else if (curr_ma < curr_abnormal_thd) {
		rc = OPLUS_CURR_ABNOR;
		chg_err("curr=%d, little than ABNORMAL_CURR", curr_ma);
	} else {
		rc = OPLUS_CURR_LOW;
	}

	return rc;
}

static int oplus_third_pps_target_voltage_check(struct oplus_pps *chip)
{
	int rc;
	int vtarget_mv;
	int batt_num;
	int allowed_vbus_min;
	int next_target_vbus_mv;
	union mms_msg_data msg_data = { 0 };

#define TARGET_VBUS_STEP (40)
#define TARGET_VBUS_STEP_IOVER (100)
#define TARGET_VBUS_DEBOUNCE (80)
#define CURR_OVER_ITARGET_CNT (10)
#define CURR_ABNORMAL_CNT (4)
#define REQ_VOLT_ABNORMAL_CNT (4)
#define CP_MIN_VOLT_DELTA (100)
#define CP_MAX_VOLT_DELTA (500)

	batt_num = oplus_gauge_get_batt_num();
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX, &msg_data, true);
	if (unlikely(rc < 0)) {
		chg_err("can't get vbat, rc=%d\n", rc);
		return rc;
	} else {
		allowed_vbus_min = msg_data.intval * batt_num * chip->cp_ratio;
	}
	if (chip->curr_set_ma != chip->target_curr_ma) {
		chg_info("Now current is %d, Adapter Request current is %d, Now voltage is %d\n",
			chip->curr_set_ma, chip->target_curr_ma, chip->vol_set_mv);
		return chip->vol_set_mv;
	}
	if (!chip->support_cp_ibus) {
		/* default common resistor 0.35ohm, calculate the best request-vbus */
		vtarget_mv = allowed_vbus_min + chip->target_curr_ma * 35 / 100;
	} else {
		/* CP allowed request-vbus range from allowed_vbus_min to vtarget_mv */
		vtarget_mv = chip->config.target_vbus_mv - CP_MAX_VOLT_DELTA;
		allowed_vbus_min += CP_MIN_VOLT_DELTA;
	}

	chg_info("last_vol_set_mv=%d, vtarget_mv=%d, vbus_min=%d, Itarget=%d", chip->vol_set_mv,
		vtarget_mv, allowed_vbus_min, chip->target_curr_ma);
	if (chip->vol_set_mv < vtarget_mv - TARGET_VBUS_DEBOUNCE) {
		chip->need_check_current = true;
		next_target_vbus_mv = chip->vol_set_mv + TARGET_VBUS_STEP;
	} else if (chip->vol_set_mv > vtarget_mv + TARGET_VBUS_DEBOUNCE) {
		chip->need_check_current = false;
		next_target_vbus_mv = chip->vol_set_mv - TARGET_VBUS_STEP;
	} else {
		chip->need_check_current = true;
		next_target_vbus_mv = chip->vol_set_mv;
	}

	if (chip->need_check_current) {
		rc = oplus_third_pps_current_check(chip);
		switch (rc) {
		case OPLUS_CURR_OVER:
			if (!chip->support_cp_ibus)
				chip->count.curr_over++;

			if (chip->count.curr_over >= CURR_OVER_ITARGET_CNT) {
				chg_err("current too high continues, exit pps and not recovery");
				chip->quit_pps_protocol = true;
				vote(chip->pps_disable_votable, IBAT_OVER_VOTER, true, 1, false);
			} else {
				next_target_vbus_mv = chip->vol_set_mv - TARGET_VBUS_STEP_IOVER;
			}
			chip->count.curr_abnormal = 0;
			break;
		case OPLUS_CURR_HIGH:
			chip->count.curr_over = 0;
			chip->count.curr_abnormal = 0;
			next_target_vbus_mv = chip->vol_set_mv - TARGET_VBUS_STEP;
			break;
		case OPLUS_CURR_OK:
			chip->count.curr_over = 0;
			chip->count.curr_abnormal = 0;
			next_target_vbus_mv = chip->vol_set_mv;
			break;
		case OPLUS_CURR_ABNOR:
			chip->count.curr_abnormal++;
			if (chip->count.curr_abnormal >= CURR_ABNORMAL_CNT) {
				chg_err("current abnormal continues, exit pps but allow recovery");
				chip->quit_pps_protocol = true;
				vote(chip->pps_disable_votable, PPS_IBAT_ABNOR_VOTER, true, 1, false);
				chip->count.pps_recovery = 0;
			}
			chg_err("count.curr_abnormal=%d", chip->count.curr_abnormal);
			break;
		case OPLUS_CURR_LOW:
			/* request-vbus calculate by vtarget_mv, don't change it in this case */
			chip->count.curr_over = 0;
			chip->count.curr_abnormal = 0;
			break;
		default:
			chip->count.curr_over = 0;
			chip->count.curr_abnormal = 0;
			break;
		}
	}

	/* Check the request-vbus not too low or too high */
	if (next_target_vbus_mv < allowed_vbus_min) {
		if (next_target_vbus_mv < chip->vol_set_mv) {
			/* Not allow decrease request-vbus */
			next_target_vbus_mv = chip->vol_set_mv;
			if (chip->count.output_low > REQ_VOLT_ABNORMAL_CNT) {
				chg_err("output abnormal continues, exit pps and not recovery");
				chip->quit_pps_protocol = true;
				vote(chip->pps_disable_votable, IBAT_OVER_VOTER, true, 1, false);
				chip->count.pps_recovery = 0;
			}
			chip->count.output_low++;
			chip->request_vbus_too_low_flag = true;
			chg_err("request vbus too low, times:%d, set flag true", chip->count.output_low);
		} else {
			/* Allow increase or keep request-vbus */
			chip->count.output_low = 0;
		}
	} else if (next_target_vbus_mv > chip->config.target_vbus_mv) {
		next_target_vbus_mv = chip->config.target_vbus_mv;
		chip->count.output_low = 0;
	} else {
		chip->count.output_low = 0;
	}

	chip->target_vbus_mv = next_target_vbus_mv;

	return 0;
}

static void oplus_pps_set_soc_current(struct oplus_pps *chip)
{
	int curr_ma = 0;

	if (chip->config.curr_max_ma_percent_75 <= 0 || chip->config.curr_max_ma_percent_85 <= 0)
		return;

	if (chip->ui_soc > 85) {
		if (chip->last_target_curr_ma > 0 && chip->last_target_curr_ma > chip->target_curr_ma) {
			curr_ma = max(chip->target_curr_ma, chip->config.curr_max_ma_percent_85);
			vote(chip->pps_curr_votable, BATT_SOC_VOTER, true, curr_ma, false);
		}
	} else if (chip->ui_soc > 75) {
		if (chip->last_target_curr_ma > 0 && chip->last_target_curr_ma > chip->target_curr_ma) {
			curr_ma = max(chip->target_curr_ma, chip->config.curr_max_ma_percent_75);
			vote(chip->pps_curr_votable, BATT_SOC_VOTER, true, curr_ma, false);
		}
	} else {
		vote(chip->pps_curr_votable, BATT_SOC_VOTER, false, 0, false);
	}
	chip->last_target_curr_ma = chip->target_curr_ma;
}

static void oplus_pps_monitor_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_pps *chip =
		container_of(dwork, struct oplus_pps, monitor_work);
	struct puc_strategy_ret_data data;
	int rc;
	int delay = PPS_MONITOR_TIME_MS;
	int target_ibus;
	bool switch_to_ffc = false;

	rc = oplus_pps_get_batt_temp_curr(chip);
	if (rc < 0)
		goto exit;

	if (!chip->pps_charging) {
		if (oplus_chg_get_fcs_support_flags()) {
			if (is_disable_charger_vatable_available(chip))
				vote(chip->chg_disable_votable, PPS_VOTER, true, 1, false);
		} else {
			if (is_wired_suspend_votable_available(chip))
				vote(chip->wired_suspend_votable, PPS_VOTER, true, 1, false);
		}
		rc = oplus_pps_charge_start(chip);
		if (rc < 0) {
			chip->quit_pps_protocol = true;
			goto exit;
		}
		oplus_pps_cp_watchdog_reset(chip);
		delay = rc;
	} else {
		rc = oplus_chg_strategy_get_data(chip->strategy, &data);
		if (rc < 0) {
			chg_err("can't get strategy data, rc=%d", rc);
			goto exit;
		}
		if (data.exit) {
			chg_info("exit pps fast charge, start ffc\n");
			if (chip->config.pps_full_recheck_temp != -EINVAL &&
			    chip->shell_temp <= chip->config.pps_full_recheck_temp) {
				chip->pps_cool_full_temp_range = chip->pps_temp_cur_range;
				vote(chip->pps_not_allow_votable, CHG_FULL_COOL_VOTER, true, 1, false);
			} else {
				vote(chip->pps_disable_votable, CHG_FULL_VOTER, true, 1, false);
			}
			switch_to_ffc = true;
			goto exit;
		}

		oplus_pps_set_cool_down_curr(chip, chip->cool_down);
		oplus_pps_set_batt_bal_curr(chip);
		oplus_pps_set_fcl_curr(chip);
		oplus_pps_protection_check(chip, &data);
		if (get_client_vote(chip->pps_disable_votable, CHG_FULL_VOTER) > 0 ||
		    get_client_vote(chip->pps_not_allow_votable, CHG_FULL_COOL_VOTER) > 0) {
			chg_info("exit pps fast charge, start ffc\n");
			switch_to_ffc = true;
			goto exit;
		}
		if (chip->error_count > PPS_ERROR_COUNT_MAX) {
			chg_info("pps communication exception, exit pps mode\n");
			goto exit;
		}
		if (chip->pdo_set_error_count > PPS_ERROR_COUNT_MAX) {
			chg_info("pps pdo_set_error exception, exit pps mode\n");
			chip->quit_pps_protocol = true;
			goto exit;
		}

		if (chip->oplus_pps_adapter) {
			chip->target_vbus_mv = data.target_vbus;
		} else {
			rc = oplus_third_pps_target_voltage_check(chip);
			if (rc < 0)
				chip->target_vbus_mv = data.target_vbus;
		}
		if (chip->ccd_strategy) {
			target_ibus = oplus_pps_apply_ccd(chip, data.target_ibus);
			vote(chip->pps_curr_votable, STEP_VOTER, true, target_ibus, false);
		} else {
			vote(chip->pps_curr_votable, STEP_VOTER, true, data.target_ibus, false);
		}
		oplus_pps_set_soc_current(chip);
	}

	if (chip->debug_force_pps_err >= PPS_ERR_BTB_OVER &&
	    chip->debug_force_pps_err < PPS_ERR_MAX) {
		oplus_pps_push_err_info(chip, chip->debug_force_pps_err, 0);
		chip->debug_force_pps_err = 0;
	}

	if (chip->pps_not_allow || chip->pps_disable) {
		chg_info("pps charge not allow or disable, exit pps mode\n");
		goto exit;
	}

	if (chip->wired_online)
		schedule_delayed_work(&chip->monitor_work, msecs_to_jiffies(delay));
	return;

exit:
	if (unlikely(chip->quit_pps_protocol)) {
		oplus_pps_force_exit(chip);
		chip->quit_pps_protocol = false;
		if (chip->cpa_current_type != CHG_PROTOCOL_PD)
			oplus_cpa_request(chip->cpa_topic, CHG_PROTOCOL_PD);
	} else {
		oplus_pps_soft_exit(chip);
		if (switch_to_ffc) {
			if (is_disable_charger_vatable_available(chip))
				vote(chip->chg_disable_votable, FASTCHG_VOTER, true, 1, false);
			oplus_comm_switch_ffc(chip->comm_topic);
		}
		if (chip->pps_fastchg_batt_temp_status == PPS_BAT_TEMP_SWITCH_CURVE) {
			chg_info("pps switch_curve, need retry start pps\n");
			chip->pps_fastchg_batt_temp_status = PPS_BAT_TEMP_NATURAL;
			schedule_delayed_work(&chip->switch_check_work, msecs_to_jiffies(100));
		}
	}
}

enum {
	OPLUS_PPS_CURR_UPDATE_50MA = 50,
	OPLUS_PPS_CURR_UPDATE_100MA = 100,
	OPLUS_PPS_CURR_UPDATE_200MA = 200,
	OPLUS_PPS_CURR_UPDATE_300MA = 300,
	OPLUS_PPS_CURR_UPDATE_400MA = 400,
	OPLUS_PPS_CURR_UPDATE_500MA = 500,
	OPLUS_PPS_CURR_UPDATE_1A = 1000,
	OPLUS_PPS_CURR_UPDATE_1P5A = 1500,
	OPLUS_PPS_CURR_UPDATE_2A = 2000,
	OPLUS_PPS_CURR_UPDATE_2P5A = 2500,
	OPLUS_PPS_CURR_UPDATE_3A = 3000,
};

static void oplus_pps_current_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_pps *chip =
		container_of(dwork, struct oplus_pps, current_work);
	int target_vbus;
	int curr_set, update_size;
	int rc;

	if (!chip->pps_charging)
		return;

#define PPS_CURR_CHANGE_UPDATE_DELAY	500
#define PPS_CURR_NO_CHANGE_UPDATE_DELAY	500

	if (abs(chip->curr_set_ma - chip->target_curr_ma) >= OPLUS_PPS_CURR_UPDATE_500MA)
		update_size = OPLUS_PPS_CURR_UPDATE_500MA;
	else if (abs(chip->curr_set_ma - chip->target_curr_ma) >= OPLUS_PPS_CURR_UPDATE_300MA)
		update_size = OPLUS_PPS_CURR_UPDATE_300MA;
	else if (abs(chip->curr_set_ma - chip->target_curr_ma) >= OPLUS_PPS_CURR_UPDATE_100MA)
		update_size = OPLUS_PPS_CURR_UPDATE_100MA;
	else
		update_size = OPLUS_PPS_CURR_UPDATE_50MA;
	curr_set = chip->curr_set_ma;

	if ((curr_set > chip->target_curr_ma) && ((curr_set - chip->target_curr_ma) >= update_size))
		curr_set -= update_size;
	else if ((chip->target_curr_ma > curr_set) && ((chip->target_curr_ma - curr_set) >= update_size))
		curr_set += update_size;
	else
		curr_set = chip->target_curr_ma;

#define OPLUS_PPS_STATUS_ABNORMAL (7000)
	if (chip->enable_pps_status && !chip->support_cp_ibus && chip->support_pps_status) {
		rc = oplus_pps_get_pps_status_info(chip, &chip->pps_status_info);
		if (rc < 0) {
			chg_err("pps get src info error\n");
			chip->support_pps_status = false;
		} else {
			chip->pps_curr_ma_from_pps_status =
						PPS_STATUS_CUR(chip->pps_status_info) * 50;

			if (chip->pps_curr_ma_from_pps_status >= OPLUS_PPS_STATUS_ABNORMAL) {
				chip->support_pps_status = false;
			} else {
				chg_info("pps_curr_ma_from_pps_status = %d\n",
						chip->pps_curr_ma_from_pps_status);
			}
		}
	}

	target_vbus = chip->target_vbus_mv;
	if ((target_vbus != chip->vol_set_mv) || (curr_set != chip->curr_set_ma)) {
		rc = oplus_pps_cp_set_iin(chip, curr_set);
		if (rc < 0) {
			chg_err("set cp input current error, rc=%d\n", rc);
		} else {
			rc = oplus_pps_pdo_set(chip, target_vbus, curr_set);
			if (rc < 0) {
				chip->pdo_set_error_count++;
				chg_err("pdo set error, cnt=%d, rc=%d\n", chip->pdo_set_error_count, rc);
			}
		}
	}

	if (chip->curr_set_ma != chip->target_curr_ma)
		schedule_delayed_work(&chip->current_work, msecs_to_jiffies(PPS_CURR_CHANGE_UPDATE_DELAY));
	else
		schedule_delayed_work(&chip->current_work, msecs_to_jiffies(PPS_CURR_NO_CHANGE_UPDATE_DELAY));
}

static void oplus_pps_wired_online_work(struct work_struct *work)
{
	struct oplus_pps *chip =
		container_of(work, struct oplus_pps, wired_online_work);
	union mms_msg_data data = { 0 };

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_ONLINE, &data,
				true);
	WRITE_ONCE(chip->wired_online, !!data.intval);

	if (!chip->wired_online) {
		oplus_pps_force_exit(chip);
		oplus_pps_set_online_keep(chip, false); /* Keep the pps ui show until charger is pluged out.*/
		cancel_delayed_work_sync(&chip->monitor_work);
		cancel_delayed_work_sync(&chip->current_work);
		cancel_delayed_work_sync(&chip->switch_end_recheck_work);

		if (is_wired_icl_votable_available(chip))
			vote(chip->wired_icl_votable, BTB_TEMP_OVER_VOTER,
			     false, 0, true);
		chip->wired_type = OPLUS_CHG_USB_TYPE_UNKNOWN;
		chip->pdsvooc_id_adapter = false;
		vote(chip->pps_curr_votable, BAD_SUB_BTB_VOTER, false, 0, false);
		vote(chip->pps_not_allow_votable, CHG_FULL_COOL_VOTER, false, 0, false);
		/* switch_check_work needs to be cancelled last, as it may blocks current threads, otherwise
		   blocking thread may continuing to cancel work in wired_online state */
		cancel_delayed_work_sync(&chip->switch_check_work);
		complete_all(&chip->pd_svooc_wait_ack);
	} else {
		chip->retention_state_ready = false;
		if (READ_ONCE(chip->disconnect_change) && !chip->pps_online &&
		    chip->retention_state && chip->cpa_current_type == CHG_PROTOCOL_PPS) {
			schedule_delayed_work(&chip->switch_check_work,
				msecs_to_jiffies(WAIT_BC1P2_GET_TYPE));
			WRITE_ONCE(chip->disconnect_change, false);
		}
		oplus_pps_vfa_reset(chip);
	}
}

static void oplus_pps_type_change_work(struct work_struct *work)
{
	struct oplus_pps *chip =
		container_of(work, struct oplus_pps, type_change_work);
	int wired_type;
	union mms_msg_data data = { 0 };
	int rc;

	if (!chip->wired_online) {
		wired_type = OPLUS_CHG_USB_TYPE_UNKNOWN;
	} else {
		rc = oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_REAL_CHG_TYPE,
				&data, false);
		if (rc < 0)
			wired_type = OPLUS_CHG_USB_TYPE_UNKNOWN;
		else
			wired_type = data.intval;
	}
	chg_info("type=%s\n", oplus_wired_get_chg_type_str(data.intval));
	if (chip->wired_type != wired_type && !chip->retention_state) {
		/* get pps type first time and support pps and pps ready, request pps */
		if (wired_type == OPLUS_CHG_USB_TYPE_PD_PPS &&
		    !get_effective_result(chip->pps_boot_votable))
			oplus_cpa_request(chip->cpa_topic, CHG_PROTOCOL_PPS);
		chip->wired_type = wired_type;
	}
}

static void oplus_pps_force_exit_work(struct work_struct *work)
{
	struct oplus_pps *chip =
		container_of(work, struct oplus_pps, force_exit_work);

	if (chip->pps_online) {
		oplus_pps_force_exit(chip);
		cancel_delayed_work_sync(&chip->switch_check_work);
		cancel_delayed_work_sync(&chip->monitor_work);
		cancel_delayed_work_sync(&chip->current_work);
	}
}

static void oplus_pps_soft_exit_work(struct work_struct *work)
{
	struct oplus_pps *chip =
		container_of(work, struct oplus_pps, soft_exit_work);

	if (chip->pps_charging) {
		oplus_pps_soft_exit(chip);
		cancel_delayed_work_sync(&chip->switch_check_work);
		cancel_delayed_work_sync(&chip->monitor_work);
		cancel_delayed_work_sync(&chip->current_work);
	}
}

static void oplus_pps_gauge_update_work(struct work_struct *work)
{
	int rc = 0;
	union mms_msg_data msg_data = { 0 };
	int wired_type = OPLUS_CHG_USB_TYPE_UNKNOWN;
	struct oplus_pps *chip =
		container_of(work, struct oplus_pps, gauge_update_work);

#define PPS_RECOVERY_IBAT_THD (-600)
#define PPS_RECOVERY_IBAT_TIMES (3)

	if (chip->pps_not_allow)
		oplus_pps_charge_allow_check(chip);

	rc = oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_REAL_CHG_TYPE, &msg_data, false);
	if (rc < 0)
		wired_type = OPLUS_CHG_USB_TYPE_UNKNOWN;
	else
		wired_type = msg_data.intval;

	if ((wired_type == OPLUS_CHG_USB_TYPE_PD_PPS) &&
	    is_client_vote_enabled(chip->pps_disable_votable, PPS_IBAT_ABNOR_VOTER)) {
		rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CURR, &msg_data, true);
		if (unlikely(rc < 0)) {
			chg_err("can't get ibat, rc=%d\n", rc);
		} else {
			chg_info("current = %d, recovery=%d, pd_pps is disable by the PPS_IBAT_ABNOR_VOTER\n",
				 msg_data.intval, chip->count.pps_recovery);
			if (msg_data.intval < PPS_RECOVERY_IBAT_THD) {
				if (chip->count.pps_recovery >= PPS_RECOVERY_IBAT_TIMES) {
					chg_info("retry to recovery the pps charging!\n");
					vote(chip->pps_disable_votable, PPS_IBAT_ABNOR_VOTER, false, 0, false);
					oplus_cpa_request(chip->cpa_topic, CHG_PROTOCOL_PPS);
				}
				chip->count.pps_recovery++;
			} else {
				chip->count.pps_recovery = 0;
			}
		}
	}

	if (!chip->shell_temp_ready) {
		rc = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_SHELL_TEMP, &msg_data, false);
		if (rc < 0) {
			chg_err("can't get battery temp, rc=%d\n", rc);
		} else {
			chip->shell_temp = msg_data.intval;
			if (chip->shell_temp != GAUGE_INVALID_TEMP) {
				chip->shell_temp_ready = true;
				vote(chip->pps_boot_votable, SHELL_TEMP_VOTER, false, 0, false);
			}
		}
	}
}

static void oplus_pps_flash_mode_handle(struct oplus_pps *chip)
{
	union mms_msg_data data = { 0 };
	int rc;

	if (!chip || !chip->comm_topic) {
		chg_err("invalid chip or comm_topic\n");
		return;
	}

	rc = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_FLASH_MODE,
				    &data, false);
	if (rc < 0)
		chg_err("can't get flash mode status, rc=%d", rc);
	else
		vote(chip->pps_not_allow_votable, FLASH_MODE_VOTER, !!data.intval, data.intval, false);
}

static void oplus_pps_comm_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_pps *chip = subs->priv_data;
	union mms_msg_data data = { 0 };
	int rc;

	switch (type) {
	case MSG_TYPE_TIMER:
		break;
	case MSG_TYPE_ITEM:
		switch (id) {
		case COMM_ITEM_UI_SOC:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->ui_soc = data.intval;
			break;
		case COMM_ITEM_TEMP_REGION:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->batt_temp_region = data.intval;
			break;
		case COMM_ITEM_CHARGE_SUSPEND:
		case COMM_ITEM_CHARGING_DISABLE:
			rc = oplus_mms_get_item_data(chip->comm_topic, id,
						     &data, false);
			if (rc < 0)
				chg_err("can't get charge suspend status, rc=%d", rc);
			else
				vote(chip->pps_not_allow_votable, USER_VOTER, !!data.intval, data.intval, false);
			break;
		case COMM_ITEM_FLASH_MODE:
			oplus_pps_flash_mode_handle(chip);
			break;
		case COMM_ITEM_COOL_DOWN:
			rc = oplus_mms_get_item_data(chip->comm_topic, id,
						     &data, false);
			if (rc < 0)
				chg_err("can't get cool down data, rc=%d", rc);
			else
				chip->cool_down = data.intval;
			break;
		case COMM_ITEM_SHELL_TEMP:
			rc = oplus_mms_get_item_data(chip->comm_topic, id,
						     &data, false);
			if (rc < 0) {
				chg_err("can't get shell temp data, rc=%d", rc);
			} else {
				chip->shell_temp = data.intval;
				if (unlikely(!chip->shell_temp_ready && chip->shell_temp != GAUGE_INVALID_TEMP)) {
					chip->shell_temp_ready = true;
					vote(chip->pps_boot_votable, SHELL_TEMP_VOTER, false, 0, false);
				}
			}
			break;
		case COMM_ITEM_LED_ON:
			rc = oplus_mms_get_item_data(chip->comm_topic, id,
						     &data, false);
			if (rc < 0)
				chg_err("can't get led_on data, rc=%d", rc);
			else
				chip->led_on = !!data.intval;
			if (chip->led_on && chip->adapter_max_curr > LED_ON_SYS_CONSUME_MA &&
			    chip->pps_charging && !chip->support_cp_ibus)
				vote(chip->pps_curr_votable, LED_ON_VOTER, true,
				     chip->adapter_max_curr - LED_ON_SYS_CONSUME_MA, false);
			else
				vote(chip->pps_curr_votable, LED_ON_VOTER, false, 0, false);
			break;
		case COMM_ITEM_SALE_MODE:
			rc = oplus_mms_get_item_data(chip->comm_topic, id,
						     &data, false);
			if (rc < 0)
				chg_err("can't get sale mode data, rc=%d", rc);
			else
				chip->chg_ctrl_by_sale_mode = (bool)data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_pps_subscribe_comm_topic(struct oplus_mms *topic,
					     void *prv_data)
{
	struct oplus_pps *chip = prv_data;
	union mms_msg_data data = { 0 };
	int rc;

	chip->comm_topic = topic;
	chip->comm_subs =
		oplus_mms_subscribe(topic, chip,
				    oplus_pps_comm_subs_callback, "pps");
	if (IS_ERR_OR_NULL(chip->comm_subs)) {
		chg_err("subscribe comm topic error, rc=%ld\n",
			PTR_ERR(chip->comm_subs));
		return;
	}

	oplus_mms_get_item_data(topic, COMM_ITEM_UI_SOC, &data, true);
	chip->ui_soc = data.intval;
	oplus_mms_get_item_data(topic, COMM_ITEM_TEMP_REGION, &data, true);
	chip->batt_temp_region = data.intval;

	rc = oplus_mms_get_item_data(topic, COMM_ITEM_CHARGE_SUSPEND, &data, true);
	if (rc < 0)
		chg_err("can't get charge suspend status, rc=%d", rc);
	else
		vote(chip->pps_not_allow_votable, USER_VOTER, !!data.intval, data.intval, false);
	rc = oplus_mms_get_item_data(topic, COMM_ITEM_CHARGING_DISABLE, &data, true);
	if (rc < 0)
		chg_err("can't get charge disable status, rc=%d", rc);
	else
		vote(chip->pps_not_allow_votable, USER_VOTER, !!data.intval, data.intval, false);

	rc = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_COOL_DOWN, &data, true);
	if (rc < 0)
		chg_err("can't get cool down data, rc=%d", rc);
	else
		chip->cool_down = data.intval;
	rc = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_LED_ON, &data, true);
	if (rc < 0)
		chg_err("can't get led on data, rc=%d", rc);
	else
		chip->led_on = !!data.intval;

	if (chip->led_on && chip->adapter_max_curr > LED_ON_SYS_CONSUME_MA &&
	    chip->pps_charging && !chip->support_cp_ibus)
		vote(chip->pps_curr_votable, LED_ON_VOTER, true,
		     chip->adapter_max_curr - LED_ON_SYS_CONSUME_MA, false);
	else
		vote(chip->pps_curr_votable, LED_ON_VOTER, false, 0, false);
	rc = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_SHELL_TEMP, &data, true);
	if (rc < 0) {
		chg_err("can't get shell temp data, rc=%d", rc);
	} else {
		chip->shell_temp = data.intval;
		if (unlikely(!chip->shell_temp_ready && chip->shell_temp != GAUGE_INVALID_TEMP)) {
			chip->shell_temp_ready = true;
			vote(chip->pps_boot_votable, SHELL_TEMP_VOTER, false, 0, false);
		}
	}
	rc = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_SALE_MODE, &data, true);
	if (rc < 0)
		chg_err("can't get sale mode data, rc=%d", rc);
	else
		chip->chg_ctrl_by_sale_mode = (bool)data.intval;

	vote(chip->pps_boot_votable, COMM_TOPIC_VOTER, false, 0, false);
}

static void oplus_pps_wired_subs_callback(struct mms_subscribe *subs,
					  enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_pps *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_TIMER:
		break;
	case MSG_TYPE_ITEM:
		switch (id) {
		case WIRED_ITEM_ONLINE:
			schedule_work(&chip->wired_online_work);
			break;
		case WIRED_ITEM_REAL_CHG_TYPE:
			schedule_work(&chip->type_change_work);
			break;
		case WIRED_TIME_ABNORMAL_ADAPTER:
			oplus_mms_get_item_data(chip->wired_topic, id, &data,
						false);
			chip->pdsvooc_id_adapter = !!data.intval;
			if (chip->pdsvooc_id_adapter)
				complete(&chip->pd_svooc_wait_ack);
			break;
		case WIRED_ITEM_PRESENT:
			oplus_mms_get_item_data(chip->wired_topic, id, &data, false);
			chip->irq_plugin = !!data.intval;
			break;
		case WIRED_ITEM_USB_STATUS:
			oplus_mms_get_item_data(chip->wired_topic, id, &data, true);
			chip->usb_status = data.intval;
			if ((chip->usb_status & USB_TEMP_HIGH) == USB_TEMP_HIGH) {
				chg_err("!!!USB TEMP HIGH disable cp/mos and exit pps\n");
				schedule_work(&chip->soft_exit_work);
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

static void oplus_pps_subscribe_wired_topic(struct oplus_mms *topic,
					     void *prv_data)
{
	struct oplus_pps *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->wired_topic = topic;
	chip->wired_subs =
		oplus_mms_subscribe(chip->wired_topic, chip,
				    oplus_pps_wired_subs_callback, "pps");
	if (IS_ERR_OR_NULL(chip->wired_subs)) {
		chg_err("subscribe wired topic error, rc=%ld\n",
			PTR_ERR(chip->wired_subs));
		return;
	}

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_ONLINE, &data,
				true);
	chip->wired_online = !!data.intval;
	oplus_mms_get_item_data(chip->wired_topic, WIRED_TIME_ABNORMAL_ADAPTER, &data,
					true);
	chip->pdsvooc_id_adapter = !!data.intval;
	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_USB_STATUS, &data,
				true);
	chip->usb_status = data.intval;
	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_PRESENT, &data,
				false);
	chip->irq_plugin = !!data.intval;

	vote(chip->pps_boot_votable, WIRED_TOPIC_VOTER, false, 0, false);
}

static void oplus_pps_cpa_subs_callback(struct mms_subscribe *subs,
					enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_pps *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_TIMER:
		break;
	case MSG_TYPE_ITEM:
		switch (id) {
		case CPA_ITEM_ALLOW:
			oplus_mms_get_item_data(chip->cpa_topic, id, &data,
						false);
			chg_err("type=%d", data.intval);
			if (chip->cpa_current_type == CHG_PROTOCOL_PPS && data.intval != CHG_PROTOCOL_PPS)
				oplus_pps_vfa_reset(chip);
			chip->cpa_current_type = data.intval;
			if (chip->cpa_current_type == CHG_PROTOCOL_PPS)
				schedule_delayed_work(&chip->switch_check_work, 0);
			break;
		case CPA_ITEM_TIMEOUT:
			oplus_mms_get_item_data(chip->cpa_topic, id, &data,
						false);
			if (data.intval == CHG_PROTOCOL_PPS) {
				oplus_pps_switch_to_normal(chip);
				oplus_pps_cpa_switch_end(chip);
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

static int read_property_if_matches(struct device_node *node, const char *property_name, int match_value,
					int *out_values, int value_count)
{
	int rc = 0, num_elements, i;
	int *property_values = NULL;

	num_elements = of_property_count_elems_of_size(node, property_name, sizeof(int));
	if (num_elements <= 0) {
		chg_err("%s num_elements %d invalid; must be positive\n", property_name, num_elements);
		return -EINVAL;
	}

	if (num_elements % (value_count + 1)) {
		chg_err("%s num_elements %d invalid; must be a multiple of %d (value_count + 1)\n",
			property_name, num_elements, value_count + 1);
		return -EINVAL;
	}

	property_values = kzalloc(num_elements * sizeof(int), GFP_KERNEL);
	if (!property_values) {
		chg_err("memory allocation for %s failed\n", property_name);
		return -ENOMEM;
	}

	rc = of_property_read_u32_array(node, property_name, (u32 *)property_values, num_elements);
	if (rc) {
		chg_err("read %s failed, rc=%d\n", property_name, rc);
		kfree(property_values);
		return rc;
	}

	for (i = 0; i < num_elements / (value_count + 1); i++) {
		if (match_value == property_values[i * (value_count + 1)]) {
			memcpy(out_values, &property_values[i * (value_count + 1) + 1], value_count * sizeof(int));
			kfree(property_values);
			return 0;
		}
	}

	chg_err("%s not found with match_value=%d\n", property_name, match_value);
	kfree(property_values);
	return -ENODATA;
}

static int oplus_pps_parse_power_dt(struct oplus_pps *chip)
{
	struct device_node *node = oplus_get_node_by_type(chip->dev->of_node);
	int power = 0, rc = 0;

	power = oplus_cpa_protocol_get_max_power_by_type(chip->cpa_topic, CHG_PROTOCOL_PPS);
	if (power <= 0) {
		chg_info("pps power %d invalid\n", power);
		return -EINVAL;
	}
	power = power / 1000;

	rc = read_property_if_matches(node, "oplus,pps_ibat_over_third", power, &chip->limits.pps_ibat_over_third, 1);
	if (!rc)
		chg_info("oplus,pps_ibat_over_third is %d\n", chip->limits.pps_ibat_over_third);

	rc = read_property_if_matches(node, "oplus,pps_ibat_over_oplus", power, &chip->limits.pps_ibat_over_oplus, 1);
	if (!rc)
		chg_info("oplus,pps_ibat_over_oplus is %d\n", chip->limits.pps_ibat_over_oplus);

	return rc;
}

static void oplus_pps_subscribe_cpa_topic(struct oplus_mms *topic,
					   void *prv_data)
{
	struct oplus_pps *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->cpa_topic = topic;
	chip->cpa_subs =
		oplus_mms_subscribe(chip->cpa_topic, chip,
				    oplus_pps_cpa_subs_callback, "pps");
	if (IS_ERR_OR_NULL(chip->cpa_subs)) {
		chg_err("subscribe cpa topic error, rc=%ld\n",
			PTR_ERR(chip->cpa_subs));
		return;
	}

	oplus_pps_parse_power_dt(chip);

	oplus_mms_get_item_data(chip->cpa_topic, CPA_ITEM_ALLOW, &data, true);
	if (data.intval == CHG_PROTOCOL_PPS)
		schedule_delayed_work(&chip->switch_check_work, msecs_to_jiffies(PPS_MONITOR_CYCLE_MS));

	vote(chip->pps_boot_votable, CPA_TOPIC_VOTER, false, 0, false);
}

static void oplus_pps_batt_bal_subs_callback(struct mms_subscribe *subs,
					     enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_pps *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case BATT_BAL_ITEM_ABNORMAL_STATE:
			oplus_mms_get_item_data(chip->batt_bal_topic, id, &data, false);
			if (data.intval) {
				oplus_pps_set_online(chip, false);
				oplus_pps_set_online_keep(chip, false);
				chg_err("batt bal abnormal, online set false\n");
			}
			break;
		case BATT_BAL_ITEM_CURR_LIMIT:
			oplus_mms_get_item_data(chip->batt_bal_topic, id, &data, false);
			chip->batt_bal_curr_limit = data.intval;
			chg_info("batt_bal_curr_limit=%d\n", chip->batt_bal_curr_limit);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_pps_subscribe_batt_bal_topic(struct oplus_mms *topic,
					     void *prv_data)
{
	struct oplus_pps *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->batt_bal_topic = topic;
	chip->batt_bal_subs =
		oplus_mms_subscribe(chip->batt_bal_topic, chip,
				    oplus_pps_batt_bal_subs_callback, "pps");
	if (IS_ERR_OR_NULL(chip->batt_bal_subs)) {
		chg_err("subscribe batt bal topic error, rc=%ld\n",
			PTR_ERR(chip->batt_bal_subs));
		return;
	}

	oplus_mms_get_item_data(chip->batt_bal_topic,
		BATT_BAL_ITEM_ABNORMAL_STATE, &data, true);
	chg_info("abnormal state=%d\n", data.intval);
	if (data.intval) {
		oplus_pps_set_online(chip, false);
		oplus_pps_set_online_keep(chip, false);
		chg_err("batt bal abnormal, online set false\n");
	}

	oplus_mms_get_item_data(chip->batt_bal_topic, BATT_BAL_ITEM_CURR_LIMIT, &data, true);
	chip->batt_bal_curr_limit = data.intval;
	chg_info("batt_bal_curr_limit=%d\n", chip->batt_bal_curr_limit);
}

static void oplus_pps_gauge_subs_callback(struct mms_subscribe *subs,
					  enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_pps *chip = subs->priv_data;
	int rc = 0;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_TIMER:
		schedule_work(&chip->gauge_update_work);
		break;
	case MSG_TYPE_ITEM:
		switch (id) {
		case GAUGE_ITEM_AUTH:
			rc = oplus_mms_get_item_data(chip->gauge_topic, id,
						     &data, false);
			if (rc < 0) {
				chg_err("can't get GAUGE_ITEM_AUTH data, rc=%d\n",
					rc);
				chip->batt_auth = false;
			} else {
				chip->batt_auth = !!data.intval;
			}
			vote(chip->pps_disable_votable, NON_STANDARD_VOTER,
			     (!chip->batt_hmac || !chip->batt_auth), 0, false);
			break;
		case GAUGE_ITEM_HMAC:
			rc = oplus_mms_get_item_data(chip->gauge_topic, id,
						     &data, false);
			if (rc < 0) {
				chg_err("can't get GAUGE_ITEM_HMAC data, rc=%d\n",
					rc);
				chip->batt_hmac = false;
			} else {
				chip->batt_hmac = !!data.intval;
			}
			vote(chip->pps_disable_votable, NON_STANDARD_VOTER,
			     (!chip->batt_hmac || !chip->batt_auth), 0, false);
			break;
		case GAUGE_ITEM_SUB_BTB_STATE:
			oplus_pps_sub_btb_connnect_check(chip);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_pps_subscribe_gauge_topic(struct oplus_mms *topic,
					     void *prv_data)
{
	struct oplus_pps *chip = prv_data;
	int rc = 0;
	union mms_msg_data data = { 0 };

	chip->gauge_topic = topic;
	chip->gauge_subs =
		oplus_mms_subscribe(chip->gauge_topic, chip,
				    oplus_pps_gauge_subs_callback, "pps");
	if (IS_ERR_OR_NULL(chip->gauge_subs)) {
		chg_err("subscribe gauge topic error, rc=%ld\n",
			PTR_ERR(chip->gauge_subs));
		return;
	}

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_HMAC, &data,
				     true);
	if (rc < 0) {
		chg_err("can't get GAUGE_ITEM_HMAC data, rc=%d\n", rc);
		chip->batt_hmac = false;
	} else {
		chip->batt_hmac = !!data.intval;
	}

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_AUTH, &data,
				     true);
	if (rc < 0) {
		chg_err("can't get GAUGE_ITEM_AUTH data, rc=%d\n", rc);
		chip->batt_auth = false;
	} else {
		chip->batt_auth = !!data.intval;
	}

	chg_info("hmac=%d, authenticate=%d\n", chip->batt_hmac, chip->batt_auth);
	vote(chip->pps_disable_votable, NON_STANDARD_VOTER, !chip->batt_hmac || !chip->batt_auth, 0, false);

	oplus_gauge_get_fcl_support(chip->gauge_topic, &chip->fcl_support);

	vote(chip->pps_boot_votable, GAUGE_TOPIC_VOTER, false, 0, false);
}

static void oplus_pps_retention_disconnect_work(struct work_struct *work)
{
	struct oplus_pps *chip =
		container_of(work, struct oplus_pps, retention_disconnect_work);
	union mms_msg_data data = { 0 };
	int pps_power;

	pps_power = oplus_cpa_protocol_get_power(chip->cpa_topic, CHG_PROTOCOL_PPS);
	oplus_mms_get_item_data(chip->retention_topic, RETENTION_ITEM_DISCONNECT_COUNT, &data, true);
	chip->pps_connect_error_count = data.intval;
	if (chip->pps_connect_error_count > PPS_CONNECT_ERROR_COUNT_LEVEL
		&& chip->cpa_current_type == CHG_PROTOCOL_PPS) {
		if (pps_power > SUPPORT_THIRD_PPS_POWER && chip->retention_oplus_adapter) {
			oplus_cpa_protocol_set_power(chip->cpa_topic, CHG_PROTOCOL_PPS, SUPPORT_THIRD_PPS_POWER);
			oplus_cpa_request(chip->cpa_topic, CHG_PROTOCOL_PPS);
			oplus_pps_force_exit(chip);
			oplus_cpa_switch_end(chip->cpa_topic, CHG_PROTOCOL_PPS);
			chip->retention_exit_pps_flag = EXIT_HIGH_PPS;
		} else {
			vote(chip->pps_disable_votable, CONNECT_VOTER, true, true, false);
			oplus_cpa_protocol_disable(chip->cpa_topic, CHG_PROTOCOL_PPS);
			oplus_pps_force_exit(chip);
			oplus_cpa_switch_end(chip->cpa_topic, CHG_PROTOCOL_PPS);
			chip->retention_exit_pps_flag = EXIT_THIRD_PPS;
		}
		return;
	}

	if (READ_ONCE(chip->wired_online)) {
		flush_work(&chip->wired_online_work);
		if (!chip->pps_online && chip->retention_state &&
		    chip->cpa_current_type == CHG_PROTOCOL_PPS)
			schedule_delayed_work(&chip->switch_check_work,
				msecs_to_jiffies(WAIT_BC1P2_GET_TYPE));
		WRITE_ONCE(chip->disconnect_change, false);
	} else {
		WRITE_ONCE(chip->disconnect_change, true);
	}
}

static void oplus_pps_retention_state_ready_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_pps *chip =
		container_of(dwork, struct oplus_pps, retention_state_ready_work);

	oplus_pps_cpa_switch_end(chip);
}

static void oplus_pps_retention_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_pps *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case RETENTION_ITEM_CONNECT_STATUS:
			oplus_mms_get_item_data(chip->retention_topic, id, &data,
						false);
			chip->retention_state = !!data.intval;
			if (!chip->retention_state) {
				chip->retention_oplus_adapter = false;
				chip->retention_ready_for_switch_end = false;
				vote(chip->pps_disable_votable, CONNECT_VOTER, false, false, false);
				vote(chip->pps_boot_votable, CONNECT_VOTER, false, false, false);
				chip->retention_exit_pps_flag = EXIT_PPS_FALSE;
			} else {
				if (!chip->wired_online) {
					chip->retention_state_ready = true;
					cancel_delayed_work(&chip->switch_end_recheck_work);
				}
			}
			break;
		case RETENTION_ITEM_DISCONNECT_COUNT:
			schedule_work(&chip->retention_disconnect_work);
			break;
		case RETENTION_ITEM_STATE_READY:
			if (!chip->wired_online) {
				chip->retention_state_ready = true;
				chip->retention_ready_for_switch_end = true;
				cancel_delayed_work(&chip->switch_end_recheck_work);
				schedule_delayed_work(&chip->retention_state_ready_work, 0);
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

static void oplus_pps_subscribe_retention_topic(struct oplus_mms *topic,
					   void *prv_data)
{
	struct oplus_pps *chip = prv_data;
	union mms_msg_data data = { 0 };
	int rc;

	chip->retention_topic = topic;
	chip->retention_subs = oplus_mms_subscribe(chip->retention_topic, chip,
					     oplus_pps_retention_subs_callback,
					     "pps");
	if (IS_ERR_OR_NULL(chip->retention_subs)) {
		chg_err("subscribe retention topic error, rc=%ld\n",
			PTR_ERR(chip->retention_subs));
		return;
	}
	rc = oplus_mms_get_item_data(chip->retention_topic, RETENTION_ITEM_DISCONNECT_COUNT, &data, true);
	if (rc >= 0)
		chip->pps_connect_error_count = data.intval;
}

static void oplus_pps_close_cp_work(struct work_struct *work)
{
	struct oplus_pps *chip = container_of(work, struct oplus_pps, close_cp_work);
	int rc = 0;

	rc = oplus_pps_cp_set_work_start(chip, false);
	chg_info("close cp, rc = %d\n", rc);
}

static void oplus_pps_error_subs_callback(struct mms_subscribe *subs, enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_pps *chip = subs->priv_data;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case ERR_ITEM_CLOSE_CP:
			if (chip->process_close_cp_item && chip->pps_charging)
				schedule_work(&chip->close_cp_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_pps_subscribe_error_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_pps *chip = prv_data;

	chip->err_topic = topic;
	chip->err_subs = oplus_mms_subscribe(chip->err_topic, chip, oplus_pps_error_subs_callback, "pps");
	if (IS_ERR_OR_NULL(chip->err_subs)) {
		chg_err("subscribe error topic error, rc=%ld\n", PTR_ERR(chip->err_subs));
		return;
	}
}

static int oplus_pps_plc_enable(struct oplus_plc_protocol *opp, enum oplus_plc_chg_mode mode)
{
	struct oplus_pps *chip;
	int rc;

	chip = oplus_plc_protocol_get_priv_data(opp);
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (mode == PLC_CHG_MODE_AUTO) {
		if (get_effective_result(chip->pps_not_allow_votable) > 0) {
			mode = PLC_CHG_MODE_BUCK;
			chg_info("mode chang to %s by %s", oplus_plc_chg_mode_str(mode),
				 get_effective_client(chip->pps_not_allow_votable));
		} else if (get_effective_result(chip->pps_disable_votable) > 0) {
			mode = PLC_CHG_MODE_BUCK;
			chg_info("mode chang to %s by %s", oplus_plc_chg_mode_str(mode),
				 get_effective_client(chip->pps_disable_votable));
		} else {
			mode = PLC_CHG_MODE_CP;
			chg_info("mode chang to %s", oplus_plc_chg_mode_str(mode));
		}
		if (mode == PLC_CHG_MODE_BUCK)
			return 0;
	}

	if (mode == PLC_CHG_MODE_BUCK)
		rc = vote(chip->pps_not_allow_votable, PLC_VOTER, true, 1, false);
	else
		rc = vote(chip->pps_not_allow_votable, PLC_VOTER, false, 0, false);

	return rc;
}

static int oplus_pps_plc_disable(struct oplus_plc_protocol *opp)
{
	struct oplus_pps *chip;

	chip = oplus_plc_protocol_get_priv_data(opp);
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	vote(chip->pps_curr_votable, PLC_VOTER, false, 0, false);
	return vote(chip->pps_not_allow_votable, PLC_VOTER, false, 0, false);
}

static int oplus_pps_plc_reset_protocol(struct oplus_plc_protocol *opp)
{
	return 0;
}

static int oplus_pps_plc_set_ibus(struct oplus_plc_protocol *opp, int ibus_ma)
{
	struct oplus_pps *chip;

	chip = oplus_plc_protocol_get_priv_data(opp);
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	vote(chip->pps_curr_votable, PLC_VOTER, true, ibus_ma, false);
	return 0;
}

static int oplus_pps_plc_get_ibus(struct oplus_plc_protocol *opp)
{
	struct oplus_pps *chip;

	chip = oplus_plc_protocol_get_priv_data(opp);
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return get_client_vote(chip->pps_curr_votable, PLC_VOTER);
}

static int oplus_pps_plc_get_chg_mode(struct oplus_plc_protocol *opp)
{
	struct oplus_pps *chip;

	chip = oplus_plc_protocol_get_priv_data(opp);
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (chip->pps_charging)
		return PLC_CHG_MODE_CP;
	return PLC_CHG_MODE_BUCK;
}

static struct oplus_plc_protocol_desc g_plc_protocol_desc = {
	.name = "pps",
	.protocol = BIT(CHG_PROTOCOL_PPS),
	.current_active = false,
	.ops = {
		.enable = oplus_pps_plc_enable,
		.disable = oplus_pps_plc_disable,
		.reset_protocol = oplus_pps_plc_reset_protocol,
		.set_ibus = oplus_pps_plc_set_ibus,
		.get_ibus = oplus_pps_plc_get_ibus,
		.get_chg_mode = oplus_pps_plc_get_chg_mode
	}
};

static void oplus_pps_plc_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_pps *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case PLC_ITEM_STATUS:
			oplus_mms_get_item_data(chip->plc_topic, id, &data,
						false);
			chip->plc_status = data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_pps_subscribe_plc_topic(struct oplus_mms *topic,
					   void *prv_data)
{
	struct oplus_pps *chip = prv_data;
	union mms_msg_data data = { 0 };
	int rc;

	chip->plc_topic = topic;
	chip->plc_subs = oplus_mms_subscribe(chip->plc_topic, chip,
					     oplus_pps_plc_subs_callback,
					     "pps");
	if (IS_ERR_OR_NULL(chip->plc_subs)) {
		chg_err("subscribe plc topic error, rc=%ld\n",
			PTR_ERR(chip->plc_subs));
		return;
	}

	rc = oplus_mms_get_item_data(chip->plc_topic, PLC_ITEM_STATUS, &data, true);
	if (rc >= 0)
		chip->plc_status = data.intval;

	chip->opp = oplus_plc_register_protocol(chip->plc_topic,
		&g_plc_protocol_desc, chip->dev->of_node, chip);
	if (chip->opp == NULL)
		chg_err("register pps plc protocol error");
}

static int oplus_pps_update_online(struct oplus_mms *mms,
				    union mms_msg_data *data)
{
	struct oplus_pps *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->pps_online;

	return 0;
}

static int oplus_pps_update_online_keep(struct oplus_mms *mms,
					union mms_msg_data *data)
{
	struct oplus_pps *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);
	if (chip)
		data->intval = chip->pps_online_keep;

	return 0;
}

static int oplus_pps_update_charging(struct oplus_mms *mms,
				      union mms_msg_data *data)
{
	struct oplus_pps *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->pps_charging;

	return 0;
}

static int oplus_pps_update_oplus_adapter(struct oplus_mms *mms,
					   union mms_msg_data *data)
{
	struct oplus_pps *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->oplus_pps_adapter;

	return 0;
}

static int oplus_pps_update_cpa_power(struct oplus_mms *mms,
                                           union mms_msg_data *data)
{
	data->intval = 0;
	return 0;
}

static int oplus_pps_update_adapter_power(struct oplus_mms *mms,
                                           union mms_msg_data *data)
{
	data->intval = 0;
	return 0;
}

static int oplus_chg_track_upload_pps_adapter_info(struct oplus_mms *mms,
                                           union mms_msg_data *data)
{
	data->intval = 0;
	return 0;
}

static int oplus_pps_track_debugfs_init(struct oplus_pps *chip)
{
	int ret = 0;
	struct dentry *debugfs_root;

	debugfs_root = oplus_chg_track_get_debugfs_root();
	if (!debugfs_root) {
		ret = -ENOENT;
		return ret;
	}

	chip->debugfs_pps = debugfs_create_dir("pps", debugfs_root);
	if (!chip->debugfs_pps) {
		ret = -ENOENT;
		return ret;
	}

	debugfs_create_u32("debug_force_pps_err", 0644, chip->debugfs_pps, &(chip->debug_force_pps_err));
	return ret;
}

static void oplus_pps_topic_update(struct oplus_mms *mms, bool publish)
{
}

static struct mms_item oplus_pps_item[] = {
	{
		.desc = {
			.item_id = PPS_ITEM_ONLINE,
			.update = oplus_pps_update_online,
		}
	},
	{
		.desc = {
			.item_id = PPS_ITEM_CHARGING,
			.update = oplus_pps_update_charging,
		}
	},
	{
		.desc = {
			.item_id = PPS_ITEM_OPLUS_ADAPTER,
			.update = oplus_pps_update_oplus_adapter,
		}
	},
	{
		.desc = {
			.item_id = PPS_ITEM_ONLINE_KEEP,
			.update = oplus_pps_update_online_keep,
		}
	},
	{
		.desc = {
			.item_id = PPS_ITEM_CPA_POWER,
			.update = oplus_pps_update_cpa_power,
		}
	},
	{
		.desc = {
			.item_id = PPS_ITEM_ADAPTER_POWER,
			.update = oplus_pps_update_adapter_power,
		}
	},
	{
		.desc = {
			.item_id = PPS_ITEM_PPS_ADAPTER_INFO,
			.update = oplus_chg_track_upload_pps_adapter_info,
		}
	},
};

static const struct oplus_mms_desc oplus_pps_desc = {
	.name = "pps",
	.type = OPLUS_MMS_TYPE_PPS,
	.item_table = oplus_pps_item,
	.item_num = ARRAY_SIZE(oplus_pps_item),
	.update_items = NULL,
	.update_items_num = 0,
	.update_interval = 0, /* ms */
	.update = oplus_pps_topic_update,
};

static int oplus_pps_topic_init(struct oplus_pps *chip)
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

	chip->pps_topic =
		devm_oplus_mms_register(chip->dev, &oplus_pps_desc, &mms_cfg);
	if (IS_ERR(chip->pps_topic)) {
		chg_err("Couldn't register pps topic\n");
		rc = PTR_ERR(chip->pps_topic);
		return rc;
	}

	return 0;
}

static int oplus_pps_virq_reg(struct oplus_pps *chip)
{
	return 0;
}

static void oplus_pps_virq_unreg(struct oplus_pps *chip)
{}

static void oplus_pps_ic_reg_callback(struct oplus_chg_ic_dev *ic, void *data, bool timeout)
{
	struct oplus_pps *chip;
	int rc;

	if (data == NULL) {
		chg_err("ic(%s) data is NULL\n", ic->name);
		return;
	}
	chip = data;

	chip->pps_ic = ic;
	rc = oplus_pps_virq_reg(chip);
	if (rc < 0) {
		chg_err("virq register error, rc=%d\n", rc);
		chip->pps_ic = NULL;
		return;
	}

	rc = oplus_pps_topic_init(chip);
	if (rc < 0)
		goto topic_reg_err;

	oplus_mms_wait_topic("common", oplus_pps_subscribe_comm_topic, chip);
	oplus_mms_wait_topic("wired", oplus_pps_subscribe_wired_topic, chip);
	oplus_mms_wait_topic("cpa", oplus_pps_subscribe_cpa_topic, chip);
	oplus_mms_wait_topic("gauge", oplus_pps_subscribe_gauge_topic, chip);
	oplus_mms_wait_topic("batt_bal", oplus_pps_subscribe_batt_bal_topic, chip);
	oplus_mms_wait_topic("retention", oplus_pps_subscribe_retention_topic, chip);
	oplus_mms_wait_topic("error", oplus_pps_subscribe_error_topic, chip);
	oplus_mms_wait_topic("plc", oplus_pps_subscribe_plc_topic, chip);
	return;

topic_reg_err:
	oplus_pps_virq_unreg(chip);
}

static int oplus_pps_dpdm_switch_virq_reg(struct oplus_pps *chip)
{
	return 0;
}

static void oplus_pps_dpdm_switch_virq_unreg(struct oplus_pps *chip)
{}

static void oplus_pps_dpdm_switch_reg_callback(struct oplus_chg_ic_dev *ic, void *data, bool timeout)
{
	struct oplus_pps *chip;
	const char *name;
	int rc;

	if (data == NULL) {
		chg_err("ic(%s) data is NULL\n", ic->name);
		return;
	}
	chip = data;

	rc = oplus_pps_dpdm_switch_virq_reg(chip);
	if (rc < 0) {
		chg_err("dpdm_switch virq register error, rc=%d\n", rc);
		return;
	}
	chip->dpdm_switch = ic;

	name = of_get_oplus_chg_ic_name(chip->dev->of_node, "oplus,pps_ic", 0);
	oplus_chg_ic_wait_ic(name, oplus_pps_ic_reg_callback, chip);
}

static void oplus_pps_cp_error_handler(struct oplus_pps *chip)
{
	oplus_pps_set_ic_pps_curr_vote(chip, chip->cp_work_mode);
}

static void oplus_pps_cp_err_analyze(struct oplus_pps *chip, struct oplus_chg_ic_err_msg *msg)
{
	if (msg->type != OPLUS_IC_ERR_CP)
		return;
	oplus_pps_cp_error_handler(chip);
}

static void oplus_pps_cp_err_handler_work(struct work_struct *work)
{
	struct oplus_pps *chip =
		container_of(work, struct oplus_pps, cp_err_handler_work);
	struct oplus_chg_ic_err_msg *msg, *tmp;
	struct list_head msg_list;

	if (!chip->pps_online)
		return;

	INIT_LIST_HEAD(&msg_list);
	spin_lock(&chip->cp_ic->err_list_lock);
	if (!list_empty(&chip->cp_ic->err_list))
		list_replace_init(&chip->cp_ic->err_list, &msg_list);
	spin_unlock(&chip->cp_ic->err_list_lock);

	list_for_each_entry_safe(msg, tmp, &msg_list, list) {
		if (is_err_topic_available(chip))
			oplus_mms_publish_ic_err_msg(chip->err_topic,
						     ERR_ITEM_IC, msg);
		oplus_pps_cp_err_analyze(chip, msg);
		oplus_print_ic_err(msg);
		list_del(&msg->list);
		kfree(msg);
	}
}

static void oplus_pps_cp_online_handler_work(struct work_struct *work)
{
	struct oplus_pps *chip =
		container_of(work, struct oplus_pps, cp_online_handler_work);
	bool cp_offline = !!atomic_read(&chip->cp_offline);
	vote(chip->pps_disable_votable, CP_OFFLINE_VOTER, cp_offline, cp_offline, false);
}

static void oplus_pps_cp_offline_handler_work(struct work_struct *work)
{
	struct oplus_pps *chip =
		container_of(work, struct oplus_pps, cp_offline_handler_work);
	bool cp_offline = !!atomic_read(&chip->cp_offline);
	vote(chip->pps_disable_votable, CP_OFFLINE_VOTER, cp_offline, cp_offline, false);
}

static void oplus_pps_cp_err_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_pps *chip = virq_data;

	schedule_work(&chip->cp_err_handler_work);
}

static void oplus_pps_cp_online_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_pps *chip = virq_data;

	chg_info("%s online\n", ic_dev->manu_name);
	atomic_set(&chip->cp_offline, 0);
	schedule_work(&chip->cp_online_handler_work);
}

static void oplus_pps_cp_offline_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_pps *chip = virq_data;

	chg_err("%s offline\n", ic_dev->manu_name);
	atomic_set(&chip->cp_offline, 1);
	schedule_work(&chip->cp_offline_handler_work);
}

static int oplus_pps_cp_virq_reg(struct oplus_pps *chip)
{
	int rc;

	rc = oplus_chg_ic_virq_register(chip->cp_ic, OPLUS_IC_VIRQ_ERR,
		oplus_pps_cp_err_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_ERR error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(chip->cp_ic, OPLUS_IC_VIRQ_ONLINE,
		oplus_pps_cp_online_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_ONLINE error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(chip->cp_ic, OPLUS_IC_VIRQ_OFFLINE,
		oplus_pps_cp_offline_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_OFFLINE error, rc=%d", rc);
	return 0;
}

static void oplus_pps_cp_virq_unreg(struct oplus_pps *chip)
{
	oplus_chg_ic_virq_release(chip->cp_ic, OPLUS_IC_VIRQ_ERR, chip);
	oplus_chg_ic_virq_release(chip->cp_ic, OPLUS_IC_VIRQ_ONLINE, chip);
	oplus_chg_ic_virq_release(chip->cp_ic, OPLUS_IC_VIRQ_OFFLINE, chip);
}

static void oplus_pps_cp_ic_reg_callback(struct oplus_chg_ic_dev *ic, void *data, bool timeout)
{
	struct oplus_pps *chip;
	const char *name;
	int rc;

	if (data == NULL) {
		chg_err("ic(%s) data is NULL\n", ic->name);
		return;
	}
	chip = data;

	chip->cp_ic = ic;
	rc = oplus_pps_cp_virq_reg(chip);
	if (rc < 0) {
		chg_err("cp virq register error, rc=%d\n", rc);
		return;
	}

	name = of_get_oplus_chg_ic_name(chip->dev->of_node, "oplus,dpdm_switch_ic", 0);
	oplus_chg_ic_wait_ic(name, oplus_pps_dpdm_switch_reg_callback, chip);
}

static int oplus_pps_curr_vote_callback(struct votable *votable, void *data,
					 int curr_ma, const char *client,
					 bool step)
{
	struct oplus_pps *chip = data;

	if (curr_ma < 0)
		return -EINVAL;

	chip->target_curr_ma = curr_ma;
	chg_info("%s set pps target current to %d mA\n", client, curr_ma);

	return 0;
}

static int oplus_pps_disable_vote_callback(struct votable *votable, void *data,
					    int disable, const char *client,
					    bool step)
{
	struct oplus_pps *chip = data;

	if (disable < 0)
		chip->pps_disable = false;
	else
		chip->pps_disable = !!disable;
	chg_info("%s set pps disable to %s\n", client, chip->pps_disable ? "true" : "false");

	return 0;
}

static int oplus_pps_not_allow_vote_callback(struct votable *votable, void *data,
					      int not_allow, const char *client,
					      bool step)
{
	struct oplus_pps *chip = data;
	union mms_msg_data mms_data = { 0 };
	int rc;

	if (not_allow < 0)
		chip->pps_not_allow = false;
	else
		chip->pps_not_allow = !!not_allow;
	chg_info("%s set pps not allow to %s\n", client, chip->pps_not_allow ? "true" : "false");

	if (!chip->pps_not_allow) {
		rc = oplus_mms_get_item_data(chip->cpa_topic, CPA_ITEM_ALLOW, &mms_data, true);
		if (rc < 0)
			return 0;
		if (mms_data.intval == CHG_PROTOCOL_PPS)
			schedule_delayed_work(&chip->switch_check_work, 0);
	}

	return 0;
}

static int oplus_pps_boot_vote_callback(struct votable *votable, void *data,
					 int boot, const char *client,
					 bool step)
{
	struct oplus_pps *chip = data;

	chg_info("pps_boot_vote change to %s by %s\n", (!!boot) ? "true" : "false", client);
	/* The PPS module startup has not been completed and will not be processed temporarily */
	if (!!boot)
		return 0;

	oplus_cpa_protocol_ready(chip->cpa_topic, CHG_PROTOCOL_PPS);
	return 0;
}

static int oplus_pps_parse_charge_strategy(struct oplus_pps *chip)
{
	int rc;
	int length = 0;
	int soc_tmp[7] = { 0, 15, 30, 50, 75, 85, 95 };
	int rang_temp_tmp[7] = { 0, 50, 120, 200, 350, 440, 510 };
	int high_temp_tmp[6] = { 425, 430, 435, 400, 415, 420 };
	int high_curr_tmp[6] = { 4000, 3000, 2000, 3000, 4000, 4000 };
	int low_curr_temp_tmp[4] = { 120, 200, 300, 440 };
	struct device_node *node = oplus_get_node_by_type(chip->dev->of_node);

	rc = of_property_read_u32(node, "oplus,pps_warm_allow_vol",
				  &chip->limits.pps_warm_allow_vol);
	if (rc)
		chip->limits.pps_warm_allow_vol = 4000;

	rc = of_property_read_u32(node, "oplus,pps_warm_allow_soc",
				  &chip->limits.pps_warm_allow_soc);
	if (rc)
		chip->limits.pps_warm_allow_soc = 50;

	rc = of_property_read_u32(node, "oplus,pps_strategy_normal_current",
				  &chip->limits.pps_strategy_normal_current);
	if (rc)
		chip->limits.pps_strategy_normal_current = 6000;

	rc = of_property_read_u32(node, "oplus,pps_over_high_or_low_current",
				  &chip->limits.pps_over_high_or_low_current);
	if (rc)
		chip->limits.pps_over_high_or_low_current = -EINVAL;

	rc = of_property_read_u32(node, "oplus,pps_full_cool_sw_vbat",
				  &chip->limits.pps_full_cool_sw_vbat);
	if (rc)
		chip->limits.pps_full_cool_sw_vbat = 4430;

	rc = of_property_read_u32(node, "oplus,pps_full_normal_sw_vbat",
				  &chip->limits.pps_full_normal_sw_vbat);
	if (rc)
		chip->limits.pps_full_normal_sw_vbat = 4495;

	rc = of_property_read_u32(node, "oplus,pps_full_normal_hw_vbat",
				  &chip->limits.pps_full_normal_hw_vbat);
	if (rc)
		chip->limits.pps_full_normal_hw_vbat = 4500;

	rc = of_property_read_u32(node, "oplus,pps_full_cool_sw_vbat_third",
				  &chip->limits.pps_full_cool_sw_vbat_third);
	if (rc)
		chip->limits.pps_full_cool_sw_vbat_third = 4430;

	rc = of_property_read_u32(node, "oplus,pps_full_normal_sw_vbat_third",
				  &chip->limits.pps_full_normal_sw_vbat_third);
	if (rc)
		chip->limits.pps_full_normal_sw_vbat_third = 4430;

	rc = of_property_read_u32(node, "oplus,pps_full_normal_hw_vbat_third",
				  &chip->limits.pps_full_normal_hw_vbat_third);
	if (rc)
		chip->limits.pps_full_normal_hw_vbat_third = 4440;

	rc = of_property_read_u32(node, "oplus,pps_full_warm_vbat",
				  &chip->limits.pps_full_warm_vbat);
	if (rc)
		chip->limits.pps_full_warm_vbat = 4150;

	rc = of_property_read_u32(node, "oplus,pps_timeout_third",
				  &chip->limits.pps_timeout_third);
	if (rc)
		chip->limits.pps_timeout_third = 9000;

	rc = of_property_read_u32(node, "oplus,pps_timeout_oplus",
				  &chip->limits.pps_timeout_oplus);
	if (rc)
		chip->limits.pps_timeout_oplus = 7200;

	chg_info("pps_warm_allow_vol=%d, pps_warm_allow_soc=%d, normal_current=%d, over_current=%d,"
		 "pps_full_cool_sw_vbat=%d, pps_full_normal_sw_vbat=%d, pps_full_normal_sw_vbat=%d,"
		 "pps_full_normal_hw_vbat=%d, pps_full_cool_sw_vbat_third=%d, pps_full_normal_sw_vbat_third=%d,"
		 "pps_full_normal_hw_vbat_third=%d, pps_timeout_third=%d, pps_timeout_oplus=%d\n",
		  chip->limits.pps_warm_allow_vol,
		  chip->limits.pps_warm_allow_soc,
		  chip->limits.pps_strategy_normal_current,
		  chip->limits.pps_over_high_or_low_current,
		  chip->limits.pps_full_cool_sw_vbat,
		  chip->limits.pps_full_normal_sw_vbat,
		  chip->limits.pps_full_normal_hw_vbat,
		  chip->limits.pps_full_cool_sw_vbat_third,
		  chip->limits.pps_full_normal_sw_vbat_third,
		  chip->limits.pps_full_normal_hw_vbat_third,
		  chip->limits.pps_full_warm_vbat,
		  chip->limits.pps_timeout_third,
		  chip->limits.pps_timeout_oplus);

	rc = of_property_count_elems_of_size(node, "oplus,pps_ibat_over_third", sizeof(u32));
	if (rc <= 1) {
		rc = of_property_read_u32(node, "oplus,pps_ibat_over_third", &chip->limits.pps_ibat_over_third);
		if (rc)
			chip->limits.pps_ibat_over_third = 4000;
	} else {
		chip->limits.pps_ibat_over_third = 7400;
	}

	rc = of_property_count_elems_of_size(node, "oplus,pps_ibat_over_oplus", sizeof(u32));
	if (rc <= 1) {
		rc = of_property_read_u32(node, "oplus,pps_ibat_over_oplus", &chip->limits.pps_ibat_over_oplus);
		if (rc)
			chip->limits.pps_ibat_over_oplus = 17000;
	} else {
		chip->limits.pps_ibat_over_oplus = 7400;
	}

	chg_info("pps_ibat_over_third=%d, pps_ibat_over_oplus=%d\n",
		  chip->limits.pps_ibat_over_third,
		  chip->limits.pps_ibat_over_oplus);

	rc = of_property_read_u32(node, "oplus,pps_bcc_max_curr",
				  &chip->bcc_max_curr);
	if (rc)
		chip->bcc_max_curr = 15000;
	else
		chg_info("oplus,pps_bcc_max_curr is %d\n", chip->bcc_max_curr);

	rc = of_property_read_u32(node, "oplus,pps_bcc_min_curr",
				  &chip->bcc_min_curr);
	if (rc)
		chip->bcc_min_curr = 1000;

	rc = of_property_read_u32(node, "oplus,pps_bcc_exit_curr",
				  &chip->bcc_exit_curr);
	if (rc)
		chip->bcc_exit_curr = 1000;

	chg_info("bcc_max_curr=%d, bcc_min_curr=%d, bcc_exit_curr=%d\n",
		  chip->bcc_max_curr, chip->bcc_min_curr, chip->bcc_exit_curr);

	rc = of_property_count_elems_of_size(
		node, "oplus,pps_strategy_batt_high_temp", sizeof(u32));
	if (rc >= 0) {
		length = rc;
		if (length > 6)
			length = 6;
		rc = of_property_read_u32_array(node, "oplus,pps_strategy_batt_high_temp",
						(u32 *)high_temp_tmp, length);
		if (rc < 0)
			chg_err("parse pps_strategy_batt_high_temp failed, "
				 "rc=%d\n", rc);
	} else {
		length = 6;
		chg_err("parse pps_strategy_batt_high_temp error, rc=%d\n", rc);
	}

	chip->enable_pps_status = of_property_read_bool(node, "oplus,pps_enable_pps_status");
	chg_info("oplus,enable_pps_status is %d", chip->enable_pps_status);

	chip->limits.pps_strategy_batt_high_temp0 = high_temp_tmp[0];
	chip->limits.pps_strategy_batt_high_temp1 = high_temp_tmp[1];
	chip->limits.pps_strategy_batt_high_temp2 = high_temp_tmp[2];
	chip->limits.pps_strategy_batt_low_temp0 = high_temp_tmp[3];
	chip->limits.pps_strategy_batt_low_temp1 = high_temp_tmp[4];
	chip->limits.pps_strategy_batt_low_temp2 = high_temp_tmp[5];
	chg_info("length = %d, high_temp[%d, %d, %d, %d, %d, %d]\n", length,
		 high_temp_tmp[0], high_temp_tmp[1], high_temp_tmp[2],
		 high_temp_tmp[3], high_temp_tmp[4], high_temp_tmp[5]);

	rc = of_property_count_elems_of_size(
		node, "oplus,pps_strategy_high_current", sizeof(u32));
	if (rc >= 0) {
		length = rc;
		if (length > 6)
			length = 6;
		rc = of_property_read_u32_array(node, "oplus,pps_strategy_high_current",
						(u32 *)high_curr_tmp, length);
		if (rc < 0) {
			chg_err("parse pps_strategy_high_current failed, "
				 "rc=%d\n", rc);
		}
	} else {
		length = 6;
		chg_err("parse pps_strategy_high_current error, rc=%d\n", rc);
	}
	chip->limits.pps_strategy_high_current0 = high_curr_tmp[0];
	chip->limits.pps_strategy_high_current1 = high_curr_tmp[1];
	chip->limits.pps_strategy_high_current2 = high_curr_tmp[2];
	chip->limits.pps_strategy_low_current0 = high_curr_tmp[3];
	chip->limits.pps_strategy_low_current1 = high_curr_tmp[4];
	chip->limits.pps_strategy_low_current2 = high_curr_tmp[5];
	chg_info("length = %d, high_current[%d, %d, %d, %d, %d, %d]\n", length,
		 high_curr_tmp[0], high_curr_tmp[1], high_curr_tmp[2],
		 high_curr_tmp[3], high_curr_tmp[4], high_curr_tmp[5]);

	rc = of_property_count_elems_of_size(
		node, "oplus,pps_charge_strategy_temp", sizeof(u32));
	if (rc >= 0) {
		chip->limits.pps_strategy_temp_num = rc;
		if (chip->limits.pps_strategy_temp_num > 7)
			chip->limits.pps_strategy_temp_num = 7;
		rc = of_property_read_u32_array(
			node, "oplus,pps_charge_strategy_temp",
			(u32 *)rang_temp_tmp,
			chip->limits.pps_strategy_temp_num);
		if (rc < 0)
			chg_err("parse pps_charge_strategy_temp failed, "
				"rc=%d\n", rc);
	} else {
		chip->limits.pps_strategy_temp_num = 7;
		chg_err("parse pps_charge_strategy_temp error, rc=%d\n", rc);
	}
	chip->limits.pps_low_temp = rang_temp_tmp[0];
	chip->limits.pps_batt_over_low_temp = rang_temp_tmp[0] - PPS_COLD_TEMP_RANGE_THD;
	chip->limits.pps_little_cold_temp = rang_temp_tmp[1];
	chip->limits.pps_cool_temp = rang_temp_tmp[2];
	chip->limits.pps_little_cool_temp = rang_temp_tmp[3];
	chip->limits.pps_normal_low_temp = rang_temp_tmp[4];
	chip->limits.pps_normal_high_temp = rang_temp_tmp[5];
	chip->limits.pps_batt_over_high_temp = rang_temp_tmp[6];
	rc = of_property_read_u32(node, "oplus,pps_little_cool_high_temp", &chip->limits.pps_little_cool_high_temp);
	if (rc)
		chip->limits.pps_little_cool_high_temp = -EINVAL;
	rc = of_property_read_u32(node, "oplus,pps_normal_low_pre_temp", &chip->limits.pps_normal_low_pre_temp);
	if (rc)
		chip->limits.pps_normal_low_pre_temp = -EINVAL;
	chg_info("pps_charge_strategy_temp num = %d, [%d, %d, %d, %d, %d, %d, %d, %d, %d]\n",
		 chip->limits.pps_strategy_temp_num, rang_temp_tmp[0],
		 rang_temp_tmp[1], rang_temp_tmp[2], rang_temp_tmp[3],
		 chip->limits.pps_little_cool_high_temp,
		 chip->limits.pps_normal_low_pre_temp,
		 rang_temp_tmp[4], rang_temp_tmp[5], rang_temp_tmp[6]);
	chip->limits.default_pps_normal_high_temp =
		chip->limits.pps_normal_high_temp;
	chip->limits.default_pps_normal_low_temp =
		chip->limits.pps_normal_low_temp;
	chip->limits.default_pps_normal_low_pre_temp =
		chip->limits.pps_normal_low_pre_temp;
	chip->limits.default_pps_little_cool_temp =
		chip->limits.pps_little_cool_temp;
	chip->limits.default_pps_little_cool_high_temp = chip->limits.pps_little_cool_high_temp;
	chip->limits.default_pps_cool_temp = chip->limits.pps_cool_temp;
	chip->limits.default_pps_little_cold_temp =
		chip->limits.pps_little_cold_temp;

	rc = of_property_count_elems_of_size(
		node, "oplus,pps_charge_strategy_soc", sizeof(u32));
	if (rc >= 0) {
		chip->limits.pps_strategy_soc_num = rc;
		if (chip->limits.pps_strategy_soc_num > 7)
			chip->limits.pps_strategy_soc_num = 7;
		rc = of_property_read_u32_array(
			node, "oplus,pps_charge_strategy_soc", (u32 *)soc_tmp,
			chip->limits.pps_strategy_soc_num);
		if (rc < 0)
			chg_err("parse pps_charge_strategy_soc failed, "
				"rc=%d\n", rc);
	} else {
		chip->limits.pps_strategy_soc_num = 7;
		chg_err("parse pps_charge_strategy_soc error, rc=%d\n", rc);
	}

	chip->limits.pps_strategy_soc_over_low = soc_tmp[0];
	chip->limits.pps_strategy_soc_min = soc_tmp[1];
	chip->limits.pps_strategy_soc_low = soc_tmp[2];
	chip->limits.pps_strategy_soc_mid_low = soc_tmp[3];
	chip->limits.pps_strategy_soc_mid = soc_tmp[4];
	chip->limits.pps_strategy_soc_mid_high = soc_tmp[5];
	chip->limits.pps_strategy_soc_high = soc_tmp[6];
	chg_info("pps_charge_strategy_soc num = %d, [%d, %d, %d, %d, %d, %d, "
		 "%d]\n",
		 chip->limits.pps_strategy_soc_num, soc_tmp[0], soc_tmp[1],
		 soc_tmp[2], soc_tmp[3], soc_tmp[4], soc_tmp[5], soc_tmp[6]);

	rc = of_property_count_elems_of_size(
		node, "oplus,pps_low_curr_full_strategy_temp", sizeof(u32));
	if (rc >= 0) {
		length = rc;
		if (length > 4)
			length = 4;
		rc = of_property_read_u32_array(
			node, "oplus,pps_low_curr_full_strategy_temp",
			(u32 *)low_curr_temp_tmp, length);
		if (rc < 0)
			chg_err("parse pps_low_curr_full_strategy_temp "
				"failed, rc=%d\n", rc);
	} else {
		length = 4;
		chg_err("parse pps_low_curr_full_strategy_temp error, rc=%d\n", rc);
	}

	chip->limits.pps_low_curr_full_cool_temp = low_curr_temp_tmp[0];
	chip->limits.pps_low_curr_full_little_cool_temp = low_curr_temp_tmp[1];
	chip->limits.pps_low_curr_full_normal_low_temp = low_curr_temp_tmp[2];
	chip->limits.pps_low_curr_full_normal_high_temp = low_curr_temp_tmp[3];
	chg_info("length = %d, low_curr_temp[%d, %d, %d, %d]\n", length,
		 low_curr_temp_tmp[0], low_curr_temp_tmp[1],
		 low_curr_temp_tmp[2], low_curr_temp_tmp[3]);

	rc = of_property_read_u32(node, "oplus,pps_low_temp", &chip->limits.pps_low_temp);
	if (rc < 0) {
		chip->limits.pps_low_temp = rang_temp_tmp[0];
	} else {
		chip->limits.pps_batt_over_low_temp = chip->limits.pps_low_temp - PPS_COLD_TEMP_RANGE_THD;
		chg_info("oplus,pps_low_temp is %d %d\n", chip->limits.pps_low_temp, chip->limits.pps_batt_over_low_temp);
	}
	return rc;
}

static int oplus_pps_parse_dt(struct oplus_pps *chip)
{
	struct device_node *node = oplus_get_node_by_type(chip->dev->of_node);
	struct oplus_pps_config *config = &chip->config;
	int rc;
	int length;
	int i;
	int data[2] = { 0 };

	rc = of_property_read_u32(node, "oplus,target_vbus_mv",
				  &config->target_vbus_mv);
	if (rc < 0) {
		chg_err("get oplus,target_vbus_mv property error, rc=%d\n",
			rc);
		config->target_vbus_mv = 11000;
	}

	rc = of_property_read_u32(node, "oplus,curr_max_ma",
				  &config->curr_max_ma);
	if (rc < 0) {
		chg_err("get oplus,curr_max_ma property error, rc=%d\n",
			rc);
		config->curr_max_ma = 3000;
	}

	rc = of_property_read_u32_array(node, "oplus,curr_max_ma_percent", data, 2);
	if (rc < 0) {
		chip->config.curr_max_ma_percent_75 = 0;
		chip->config.curr_max_ma_percent_85 = 0;
	} else {
		chip->config.curr_max_ma_percent_75 = data[0];
		chip->config.curr_max_ma_percent_85 = data[1];
	}
	chg_info("curr_max_ma_percent 75%%:%dmA 85%%:%dmA\n", chip->config.curr_max_ma_percent_75,
		chip->config.curr_max_ma_percent_85);


	rc = of_property_read_string(node, "oplus,curve_strategy_name", (const char **)&config->curve_strategy_name);
	if (rc < 0) {
		chg_err("oplus,curve_strategy_name reading failed, rc=%d\n", rc);
		config->curve_strategy_name = "pps_ufcs_curve";
	}
	chg_info("curve_strategy_name=%s\n", config->curve_strategy_name);

	rc = of_property_count_elems_of_size(node, "oplus,delta_vbus", sizeof(u32));
	if (rc < 0) {
		chg_err("Count oplus,delta_vbus, rc=%d\n", rc);
		memset(chip->delta_vbus, 0, sizeof(chip->delta_vbus));
	} else {
		length = rc;
		memset(chip->delta_vbus, 0, sizeof(chip->delta_vbus));
		if (length < PPS_TEMP_RANGE_COOL || length > PPS_TEMP_RANGE_NORMAL) {
			chg_err("wrong length oplus,delta_vbus, rc=%d\n", rc);
		} else {
			rc = of_property_read_u32_array(node, "oplus,delta_vbus", (u32 *)chip->delta_vbus,
							length);
			for (i = 1; i < length; i++) {
				chg_info("delta vbus %d\n", chip->delta_vbus[i]);
			}
		}
	}

	chip->lift_vbus_use_cpvout = of_property_read_bool(node, "oplus,lift_vbus_use_cpvout");
	chg_info("lift_vbus_use_cpvout:%d\n", chip->lift_vbus_use_cpvout);

	chip->process_close_cp_item = of_property_read_bool(node, "oplus,process_close_cp_item");
	chg_info("process_close_cp_item:%d\n", chip->process_close_cp_item);

	chip->enable_pps_reboot_notifier = of_property_read_bool(node, "oplus,enable_pps_reboot_notifier");
	chg_info("enable_pps_reboot_notifier:%d\n", chip->enable_pps_reboot_notifier);

	rc = of_property_read_u32(node, "oplus,pps_full_recheck_temp", &config->pps_full_recheck_temp);
	if (rc < 0) {
		chg_err("not support pps full recheck\n");
		config->pps_full_recheck_temp = -EINVAL;
	}

	(void)oplus_pps_parse_charge_strategy(chip);

	return 0;
}

static int oplus_pps_vote_init(struct oplus_pps *chip)
{
	int rc;

	chip->pps_curr_votable = create_votable("PPS_CURR", VOTE_MIN,
					   oplus_pps_curr_vote_callback, chip);
	if (IS_ERR(chip->pps_curr_votable)) {
		rc = PTR_ERR(chip->pps_curr_votable);
		chg_err("creat pps_curr_votable error, rc=%d\n", rc);
		chip->pps_curr_votable = NULL;
		return rc;
	}
	vote(chip->pps_curr_votable, MAX_VOTER, true, chip->config.curr_max_ma, false);

	chip->pps_disable_votable =
		create_votable("PPS_DISABLE", VOTE_SET_ANY, oplus_pps_disable_vote_callback, chip);
	if (IS_ERR(chip->pps_disable_votable)) {
		rc = PTR_ERR(chip->pps_disable_votable);
		chg_err("creat pps_disable_votable error, rc=%d\n", rc);
		chip->pps_disable_votable = NULL;
		goto creat_disable_votable_err;
	}

	chip->pps_not_allow_votable =
		create_votable("PPS_NOT_ALLOW", VOTE_MIN, oplus_pps_not_allow_vote_callback, chip);
	if (IS_ERR(chip->pps_not_allow_votable)) {
		rc = PTR_ERR(chip->pps_not_allow_votable);
		chg_err("creat pps_not_allow_votable error, rc=%d\n", rc);
		chip->pps_not_allow_votable = NULL;
		goto creat_not_allow_votable_err;
	}

	chip->pps_boot_votable =
		create_votable("PPS_BOOT", VOTE_SET_ANY, oplus_pps_boot_vote_callback, chip);
	if (IS_ERR(chip->pps_boot_votable)) {
		rc = PTR_ERR(chip->pps_boot_votable);
		chg_err("creat pps_boot_votable error, rc=%d\n", rc);
		chip->pps_boot_votable = NULL;
		goto creat_boot_votable_err;
	}
	vote(chip->pps_boot_votable, SHELL_TEMP_VOTER, true, 1, false);
	vote(chip->pps_boot_votable, COMM_TOPIC_VOTER, true, 1, false);
	vote(chip->pps_boot_votable, WIRED_TOPIC_VOTER, true, 1, false);
	vote(chip->pps_boot_votable, GAUGE_TOPIC_VOTER, true, 1, false);
	vote(chip->pps_boot_votable, CPA_TOPIC_VOTER, true, 1, false);
	chip->shell_temp_ready = false;

	return 0;

creat_boot_votable_err:
	destroy_votable(chip->pps_not_allow_votable);
creat_not_allow_votable_err:
	destroy_votable(chip->pps_disable_votable);
creat_disable_votable_err:
	destroy_votable(chip->pps_curr_votable);
	return rc;
}

static int oplus_pps_get_input_node_impedance(void *data)
{
	struct oplus_pps *chip;
	int vac, iin, vchg;
	int r_mohm;
	int rc;

	if (data == NULL)
		return -EINVAL;
	chip = data;

	if (!chip->pps_charging) {
		chg_err("pps_charging is false, can't read input node impedance\n");
		return -EINVAL;
	}

	rc = oplus_pps_cp_get_vac(chip, &vac);
	if (rc < 0) {
		chg_err("can't get cp vac, rc=%d\n", rc);
		return rc;
	}
	rc = oplus_pps_cp_get_iin(chip, &iin);
	if (rc < 0) {
		chg_err("can't get cp iin, rc=%d\n", rc);
		return rc;
	}

	rc = oplus_pps_get_pps_status_info(chip, &chip->pps_status_info);
	if (rc < 0) {
		chg_err("pps get src info error\n");
		return rc;
	}
	vchg = PPS_STATUS_VOLT(chip->pps_status_info);

	r_mohm = (vchg - vac) * 1000 / iin;
	if (r_mohm < 0) {
		chg_err("pps_input_node: r_mohm=%d\n", r_mohm);
		r_mohm = 0;
	}

	return r_mohm;
}

static int oplus_pps_init_imp_node(struct oplus_pps *chip)
{
	struct device_node *imp_node;
	struct device_node *child;
	const char *name;
	int rc;

	imp_node = of_get_child_by_name(chip->dev->of_node, "oplus,impedance_node");
	if (imp_node == NULL)
		return 0;

	for_each_child_of_node(imp_node, child) {
		rc = of_property_read_string(child, "node_name", &name);
		if (rc < 0) {
			chg_err("can't read %s node_name, rc=%d\n", child->name, rc);
			continue;
		}
		if (!strcmp(name, "pps_input")) {
			chip->input_imp_node =
				oplus_imp_node_register(child, chip->dev, chip, oplus_pps_get_input_node_impedance);
			if (IS_ERR_OR_NULL(chip->input_imp_node)) {
				chg_err("%s register error, rc=%ld\n", child->name, PTR_ERR(chip->input_imp_node));
				chip->input_imp_node = NULL;
				continue;
			}
			oplus_imp_node_set_active(chip->input_imp_node, true);
		} else {
			chg_err("unknown node_name: %s\n", name);
		}
	}

	return 0;
}

static void oplus_pps_imp_uint_init_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_pps *chip =
		container_of(dwork, struct oplus_pps, imp_uint_init_work);
	static int retry_count = 0;
#define IMP_UINT_INIT_RETRY_MAX	500

	if (!of_property_read_bool(chip->dev->of_node, "oplus,impedance_unit")) {
		chg_err("oplus,impedance_unit not found\n");
		return;
	}

	chip->imp_uint = of_find_imp_uint(chip->dev->of_node, "oplus,impedance_unit");
	if (chip->imp_uint != NULL) {
		vote(chip->pps_not_allow_votable, IMP_VOTER, false, 0, false);
		return;
	}

	retry_count++;
	if (retry_count > IMP_UINT_INIT_RETRY_MAX) {
		chg_err("can't get pps imp uint\n");
		return;
	}

	/* Try once every 100ms, accumulatively try 50s */
	schedule_delayed_work(&chip->imp_uint_init_work, msecs_to_jiffies(100));
}

static void oplus_pps_boot_curr_limit_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_pps *chip =
		container_of(dwork, struct oplus_pps, boot_curr_limit_work);

	vote(chip->pps_curr_votable, ADAPTER_MAX_POWER, false, 0, false);
}

static int oplus_pps_dev_open(struct inode *inode, struct file *filp)
{
	struct oplus_pps *chip = container_of(filp->private_data,
		struct oplus_pps, misc_dev);

	filp->private_data = chip;
	chg_info("pps_dev: %d-%d\n", imajor(inode), iminor(inode));
	return 0;
}

static ssize_t oplus_pps_dev_read(struct file *filp, char __user *buf,
				   size_t count, loff_t *offset)
{
	struct oplus_pps *chip = filp->private_data;
	struct pps_dev_cmd cmd;
	int rc;

	mutex_lock(&chip->read_lock);
	rc = wait_event_interruptible(chip->read_wq, chip->cmd_data_ok);
	mutex_unlock(&chip->read_lock);
	if (rc)
		return rc;
	if (!chip->cmd_data_ok)
		chg_err("pps false wakeup, rc=%d\n", rc);

	mutex_lock(&chip->cmd_data_lock);
	chip->cmd_data_ok = false;
	memcpy(&cmd, &chip->cmd, sizeof(struct pps_dev_cmd));
	mutex_unlock(&chip->cmd_data_lock);
	if (copy_to_user(buf, &cmd, sizeof(struct pps_dev_cmd))) {
		chg_err("failed to copy to user space\n");
		return -EFAULT;
	}

	return sizeof(struct pps_dev_cmd);
}

#define PPS_IOC_MAGIC			0x66
#define PPS_NOTIFY_GET_AUTH_DATA	_IOW(PPS_IOC_MAGIC, 1, char)

static long oplus_pps_dev_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	struct oplus_pps *chip = filp->private_data;

	switch (cmd) {
	case PPS_NOTIFY_GET_AUTH_DATA:
		complete(&chip->cmd_ack);
		break;
		break;
	default:
		chg_err("bad ioctl %u\n", cmd);
		return -EINVAL;
	}

	return 0;
}

static ssize_t oplus_pps_dev_write(struct file *filp, const char __user *buf,
				    size_t count, loff_t *offset)
{
	return count;
}

static const struct file_operations oplus_pps_dev_fops = {
	.owner			= THIS_MODULE,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
	.llseek			= noop_llseek,
#else
	.llseek			= no_llseek,
#endif
	.write			= oplus_pps_dev_write,
	.read			= oplus_pps_dev_read,
	.open			= oplus_pps_dev_open,
	.unlocked_ioctl		= oplus_pps_dev_ioctl,
};

static int oplus_pps_misc_dev_reg(struct oplus_pps *chip)
{
	int rc;

	mutex_init(&chip->read_lock);
	mutex_init(&chip->cmd_data_lock);
	init_waitqueue_head(&chip->read_wq);
	chip->cmd_data_ok = false;
	init_completion(&chip->cmd_ack);

	chip->misc_dev.minor = MISC_DYNAMIC_MINOR;
	chip->misc_dev.name = "pps_dev";
	chip->misc_dev.fops = &oplus_pps_dev_fops;
	rc = misc_register(&chip->misc_dev);
	if (rc)
		chg_err("misc_register failed, rc=%d\n", rc);

	return rc;
}

static int oplus_pps_parse_lcf_strategy_dt(struct oplus_pps *chip)
{
	struct device_node *startegy_node;
	int rc = 0;

	startegy_node = of_get_child_by_name(oplus_get_node_by_type(chip->dev->of_node), "pps_oplus_lcf_strategy");
	if (startegy_node == NULL) {
		chg_err("pps_oplus_lcf_strategy not found\n");
		rc = -ENODEV;
	} else {
		chip->oplus_lcf_strategy = oplus_chg_strategy_alloc_by_node("low_curr_full_strategy", startegy_node);
		if (IS_ERR_OR_NULL(chip->oplus_lcf_strategy)) {
			chg_err("alloc oplus_lcf_strategy error, rc=%ld", PTR_ERR(chip->oplus_lcf_strategy));
			chip->oplus_lcf_strategy = NULL;
			rc = -EFAULT;
		}
	}

	startegy_node = of_get_child_by_name(oplus_get_node_by_type(chip->dev->of_node), "pps_third_lcf_strategy");
	if (startegy_node == NULL) {
		chg_err("pps_third_lcf_strategy not found\n");
		rc = -ENODEV;
	} else {
		chip->third_lcf_strategy = oplus_chg_strategy_alloc_by_node("low_curr_full_strategy", startegy_node);
		if (IS_ERR_OR_NULL(chip->third_lcf_strategy)) {
			chg_err("alloc third_lcf_strategy error, rc=%ld", PTR_ERR(chip->third_lcf_strategy));
			chip->third_lcf_strategy = NULL;
			rc = -EFAULT;
		}
	}

	return rc;
}

static int oplus_pps_parse_ccd_strategy(struct oplus_pps *chip)
{
	struct device_node *node = oplus_get_node_by_type(chip->dev->of_node);

	chip->ccd_strategy = oplus_chg_strategy_alloc_by_node("cycle_current_derating", node);
	if (IS_ERR_OR_NULL(chip->ccd_strategy))
		chip->ccd_strategy = NULL;
	if (chip->ccd_strategy)
		oplus_chg_strategy_set_process_data(chip->ccd_strategy, "temp_region_cnt", chip->temp_region_cnt);
	return 0;
}

int oplus_pps_current_to_level(struct oplus_mms *mms, int ibus_curr)
{
	int level = 0;
	struct oplus_pps *chip;
	int ibat_curr = 0;

	if (ibus_curr <= 0)
		return level;
	if (mms == NULL) {
		chg_err("mms is NULL");
		return level;
	}
	chip = oplus_mms_get_drvdata(mms);
	if (chip == NULL)
		return -EINVAL;

	ibat_curr = ibus_curr * chip->cp_ratio;

	chip = oplus_mms_get_drvdata(mms);
	if (chip->support_cp_ibus)
		level = pps_find_level_to_current(ibat_curr, g_pps_cp_current_table, ARRAY_SIZE(g_pps_cp_current_table));
	else
		level = pps_find_level_to_current(ibus_curr, g_pps_current_table, ARRAY_SIZE(g_pps_current_table));

	return level;
}

int oplus_pps_get_charging_power_watt(struct oplus_mms *mms)
{
	int power_mw = 0;
	struct oplus_pps *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return power_mw;
	}
	chip = oplus_mms_get_drvdata(mms);
	if (chip == NULL)
		return power_mw;

	power_mw = oplus_cpa_protocol_get_power(chip->cpa_topic, CHG_PROTOCOL_PPS) / 1000; /*mW to Watt*/
	return power_mw;
}

int oplus_pps_get_adapter_power_mw(struct oplus_mms *mms)
{
	int adapter_power = 0;
	struct oplus_pps *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return adapter_power;
	}
	chip = oplus_mms_get_drvdata(mms);
	if (chip == NULL)
		return adapter_power;

	adapter_power = chip->adapter_max_curr * chip->config.target_vbus_mv / OPLUS_PPS_UW_MV_TRANSFORM;

	return adapter_power;
}

static void oplus_pps_get_temp_region_cnt(struct oplus_pps *chip, struct device_node *node)
{
	int rc;

	if (chip == NULL || node == NULL) {
		chg_err("chip or node is NULL\n");
		if (chip)
			chip->temp_region_cnt = 0;
		return;
	}
	rc = of_property_count_elems_of_size(node, "oplus,temp_range", sizeof(u32));
	if (rc <= 0) {
		chg_err("get oplus,temp_range invalid, rc=%d\n", rc);
		chip->temp_region_cnt = 0;
		return;
	}
	chip->temp_region_cnt = rc - 1;
}

static int oplus_pps_probe(struct platform_device *pdev)
{
	struct oplus_pps *chip;
	const char *name;
	struct device_node *startegy_node;
	int rc;

	chip = devm_kzalloc(&pdev->dev, sizeof(struct oplus_pps), GFP_KERNEL);
	if (chip == NULL) {
		chg_err("alloc oplus_pps struct buffer error\n");
		return 0;
	}
	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);
	mutex_init(&chip->ccd_lock);

	oplus_pps_parse_dt(chip);
	atomic_set(&chip->cp_offline, 0);
	INIT_DELAYED_WORK(&chip->switch_check_work, oplus_pps_switch_check_work);
	INIT_DELAYED_WORK(&chip->monitor_work, oplus_pps_monitor_work);
	INIT_DELAYED_WORK(&chip->current_work, oplus_pps_current_work);
	INIT_DELAYED_WORK(&chip->imp_uint_init_work, oplus_pps_imp_uint_init_work);
	INIT_DELAYED_WORK(&chip->boot_curr_limit_work, oplus_pps_boot_curr_limit_work);
	INIT_DELAYED_WORK(&chip->switch_end_recheck_work, oplus_pps_switch_end_recheck_work);
	INIT_DELAYED_WORK(&chip->retention_state_ready_work,
			  oplus_pps_retention_state_ready_work);
	INIT_WORK(&chip->wired_online_work, oplus_pps_wired_online_work);
	INIT_WORK(&chip->type_change_work, oplus_pps_type_change_work);
	INIT_WORK(&chip->force_exit_work, oplus_pps_force_exit_work);
	INIT_WORK(&chip->soft_exit_work, oplus_pps_soft_exit_work);
	INIT_WORK(&chip->gauge_update_work, oplus_pps_gauge_update_work);
	INIT_WORK(&chip->retention_disconnect_work, oplus_pps_retention_disconnect_work);
	INIT_WORK(&chip->close_cp_work, oplus_pps_close_cp_work);
	INIT_WORK(&chip->cp_err_handler_work, oplus_pps_cp_err_handler_work);
	INIT_WORK(&chip->cp_online_handler_work, oplus_pps_cp_online_handler_work);
	INIT_WORK(&chip->cp_offline_handler_work, oplus_pps_cp_offline_handler_work);
	INIT_WORK(&chip->set_fcs_icl_work, oplus_pps_set_fcs_icl_work);
	init_completion(&chip->pd_svooc_wait_ack);

	oplus_pps_parse_temperature_strategy_init(chip);

	rc = oplus_pps_init_imp_node(chip);
	if (rc < 0)
		goto imp_node_init_err;

	rc = oplus_pps_vote_init(chip);
	if (rc < 0)
		goto vote_init_err;

	rc = oplus_pps_misc_dev_reg(chip);
	if (rc < 0)
		goto misc_dev_reg_err;

	startegy_node = of_get_child_by_name(oplus_get_node_by_type(pdev->dev.of_node), "pps_charge_third_strategy");
	if (startegy_node == NULL) {
		chg_err("pps_charge_third_strategy not found\n");
		rc = -ENODEV;
		goto third_startegy_err;
	}
	chip->third_curve_strategy = oplus_chg_strategy_alloc_by_node(chip->config.curve_strategy_name, startegy_node);
	if (IS_ERR_OR_NULL(chip->third_curve_strategy)) {
		chg_err("alloc third curve startegy error, rc=%ld", PTR_ERR(chip->third_curve_strategy));
		chip->third_curve_strategy = NULL;
		rc = -EFAULT;
		goto third_startegy_err;
	}
	oplus_pps_get_temp_region_cnt(chip, startegy_node);
	startegy_node = of_get_child_by_name(oplus_get_node_by_type(pdev->dev.of_node), "pps_charge_oplus_strategy");
	if (startegy_node == NULL) {
		chg_err("pps_charge_oplus_strategy not found\n");
		rc = -ENODEV;
		goto oplus_startegy_err;
	}
	chip->oplus_curve_strategy = oplus_chg_strategy_alloc_by_node(chip->config.curve_strategy_name, startegy_node);
	if (IS_ERR_OR_NULL(chip->oplus_curve_strategy)) {
		chg_err("alloc oplus curve startegy error, rc=%ld", PTR_ERR(chip->oplus_curve_strategy));
		chip->oplus_curve_strategy = NULL;
		rc = -EFAULT;
		goto oplus_startegy_err;
	}

	oplus_pps_parse_lcf_strategy_dt(chip);

	oplus_pps_parse_vfa_strategy(chip);

	oplus_pps_parse_ccd_strategy(chip);

	name = of_get_oplus_chg_ic_name(pdev->dev.of_node, "oplus,cp_ic", 0);
	oplus_chg_ic_wait_ic(name, oplus_pps_cp_ic_reg_callback, chip);
	if (of_property_read_bool(chip->dev->of_node, "oplus,impedance_unit")) {
		vote(chip->pps_not_allow_votable, IMP_VOTER, true, 1, false);
		schedule_delayed_work(&chip->imp_uint_init_work, 0);
	} else {
		chg_err("oplus,impedance_unit not found\n");
	}
	chip->boot_time = local_clock() / LOCAL_T_NS_TO_MS_THD;
	chip->debug_force_pps_err = 0;

	oplus_pps_track_debugfs_init(chip);
	oplus_pps_reboot_nb_register(chip);

	return 0;

oplus_startegy_err:
	if (chip->third_curve_strategy != NULL)
		oplus_chg_strategy_release(chip->third_curve_strategy);
third_startegy_err:
	misc_deregister(&chip->misc_dev);
misc_dev_reg_err:
	if (chip->pps_boot_votable != NULL)
		destroy_votable(chip->pps_boot_votable);
	if (chip->pps_not_allow_votable != NULL)
		destroy_votable(chip->pps_not_allow_votable);
	if (chip->pps_disable_votable != NULL)
		destroy_votable(chip->pps_disable_votable);
	if (chip->pps_curr_votable != NULL)
		destroy_votable(chip->pps_curr_votable);
vote_init_err:
	if (chip->input_imp_node != NULL)
		oplus_imp_node_unregister(chip->dev, chip->input_imp_node);
imp_node_init_err:
	if (chip->temperature_strategy)
		oplus_chg_strategy_release(chip->temperature_strategy);
	mutex_destroy(&chip->ccd_lock);
	oplus_pps_reboot_nb_unregister(chip);
	devm_kfree(&pdev->dev, chip);
	return rc;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void oplus_pps_remove(struct platform_device *pdev)
#else
static int oplus_pps_remove(struct platform_device *pdev)
#endif
{
	struct oplus_pps *chip = platform_get_drvdata(pdev);

	oplus_pps_reboot_nb_unregister(chip);

	if (chip->pps_ic)
		oplus_pps_virq_unreg(chip);
	if (chip->dpdm_switch)
		oplus_pps_dpdm_switch_virq_unreg(chip);
	if (chip->cp_ic)
		oplus_pps_cp_virq_unreg(chip);
	oplus_plc_release_protocol(chip->plc_topic, chip->opp);
	if (!IS_ERR_OR_NULL(chip->cpa_subs))
		oplus_mms_unsubscribe(chip->cpa_subs);
	if (!IS_ERR_OR_NULL(chip->wired_subs))
		oplus_mms_unsubscribe(chip->wired_subs);
	if (!IS_ERR_OR_NULL(chip->comm_subs))
		oplus_mms_unsubscribe(chip->comm_subs);
	if (!IS_ERR_OR_NULL(chip->retention_subs))
		oplus_mms_unsubscribe(chip->retention_subs);
	if (!IS_ERR_OR_NULL(chip->plc_subs))
		oplus_mms_unsubscribe(chip->plc_subs);
	if (chip->vfa_strategy != NULL)
		oplus_chg_strategy_release(chip->vfa_strategy);
	if (chip->ccd_strategy != NULL)
		oplus_chg_strategy_release(chip->ccd_strategy);
	if (chip->oplus_curve_strategy != NULL)
		oplus_chg_strategy_release(chip->oplus_curve_strategy);
	if (chip->third_curve_strategy != NULL)
		oplus_chg_strategy_release(chip->third_curve_strategy);
	if (chip->oplus_lcf_strategy != NULL)
		oplus_chg_strategy_release(chip->oplus_lcf_strategy);
	if (chip->third_lcf_strategy != NULL)
		oplus_chg_strategy_release(chip->third_lcf_strategy);
	misc_deregister(&chip->misc_dev);
	if (chip->pps_boot_votable != NULL)
		destroy_votable(chip->pps_boot_votable);
	if (chip->pps_not_allow_votable != NULL)
		destroy_votable(chip->pps_not_allow_votable);
	if (chip->pps_disable_votable != NULL)
		destroy_votable(chip->pps_disable_votable);
	if (chip->pps_curr_votable != NULL)
		destroy_votable(chip->pps_curr_votable);
	if (chip->input_imp_node != NULL)
		oplus_imp_node_unregister(chip->dev, chip->input_imp_node);
	if (chip->temperature_strategy)
		oplus_chg_strategy_release(chip->temperature_strategy);
	if (chip->debugfs_pps)
		debugfs_remove_recursive(chip->debugfs_pps);
	mutex_destroy(&chip->ccd_lock);
	devm_kfree(&pdev->dev, chip);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return 0;
#endif
}

static void oplus_pps_reboot_nb_register(struct oplus_pps *chip)
{
	if (!chip->enable_pps_reboot_notifier)
		return;

	chip->pps_reboot_nb.notifier_call = oplus_pps_reboot_notify_call;
	chip->pps_reboot_nb.priority = OPLUS_PPS_REBOOT_NB_PRIORITY;
	register_reboot_notifier(&chip->pps_reboot_nb);
	chip->pps_reboot_nb_registered = true;
}

static void oplus_pps_reboot_nb_unregister(struct oplus_pps *chip)
{
	if (!chip->enable_pps_reboot_notifier || !chip->pps_reboot_nb_registered)
		return;

	unregister_reboot_notifier(&chip->pps_reboot_nb);
	chip->pps_reboot_nb_registered = false;
}

static int oplus_pps_reboot_notify_call(struct notifier_block *nb, unsigned long action, void *data)
{
	struct oplus_pps *chip = container_of(nb, struct oplus_pps, pps_reboot_nb);

	if (action == SYS_RESTART || action == SYS_POWER_OFF) {
		if (chip->pps_charging || chip->pps_online || chip->vol_set_mv > PPS_START_DEF_VOL_MV) {
			chg_info("reboot nb shutdown exit, vol=%d\n", chip->vol_set_mv);
			oplus_pps_shutdown_exit(chip);
		}
	}

	return NOTIFY_DONE;
}

static void oplus_pps_shutdown(struct platform_device *pdev)
{
	struct oplus_pps *chip = platform_get_drvdata(pdev);

	if (chip == NULL) {
		chg_err("device not found\n");
		return;
	}

	if (chip->pps_charging) {
		vote(chip->pps_disable_votable, SHUTDOWN_VOTER, true, 1, false);
		cancel_delayed_work(&chip->monitor_work);
		oplus_pps_soft_exit(chip);
	}
}

static const struct of_device_id oplus_pps_match[] = {
	{ .compatible = "oplus,pps_charge" },
	{},
};

static struct platform_driver oplus_pps_driver = {
	.driver		= {
		.name = "oplus-pps_charge",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(oplus_pps_match),
	},
	.probe		= oplus_pps_probe,
	.remove		= oplus_pps_remove,
	.shutdown	= oplus_pps_shutdown,
};

static __init int oplus_pps_init(void)
{
	return platform_driver_register(&oplus_pps_driver);
}

static __exit void oplus_pps_exit(void)
{
	platform_driver_unregister(&oplus_pps_driver);
}

oplus_chg_module_register(oplus_pps);

