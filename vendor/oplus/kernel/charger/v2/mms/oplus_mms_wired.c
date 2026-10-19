#define pr_fmt(fmt) "[MMS_WIRED]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/list.h>
#include <linux/kthread.h>
#include <linux/usb/typec.h>
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/system/boot_mode.h>
#include <soc/oplus/device_info.h>
#include <soc/oplus/system/oplus_project.h>
#endif
#include <oplus_chg_module.h>
#include <oplus_chg.h>
#include <oplus_chg_voter.h>
#include <oplus_chg_ic.h>
#include <oplus_chg_vooc.h>
#include <oplus_chg_comm.h>
#include <oplus_batt_bal.h>
#include <oplus_mms.h>
#include <oplus_mms_gauge.h>
#include <oplus_chg_monitor.h>
#include <oplus_mms_wired.h>
#include <oplus_chg_wired.h>
#include <oplus_chg_cpa.h>
#include <oplus_chg_pps.h>
#include <oplus_chg_ufcs.h>
#include <oplus_chg_wls.h>
#include <oplus_reverse_chg.h>
#include <oplus_dischg_boost.h>

#include "../charger_ic/op_charge.h"

#define ONLINE_STATUS_ERR_CHECK_DELAY_MS	1000
#define ONLINE_STATUS_ERR_CHECK_MAX		5
#define OTG_OUTPUT_LEVEL_MAX			5
#define DEFAULT_BATT_REAL_TEMP			250

#define VOOCPHY_DISCHARG_RETRY_CNT_MAX		15
#define VOOCPHY_DISCHARG_RETRY_DELAY		1000

enum oplus_usbtemp_timer_stage {
	OPLUS_USBTEMP_TIMER_STAGE0 = 0,
	OPLUS_USBTEMP_TIMER_STAGE1,
};

struct otg_output_config {
	int otg_curr;
	int otg_vol;
};

struct oplus_usbtemp_spec_config {
	int usbtemp_batt_temp_low;
	int usbtemp_batt_temp_high;
	int usbtemp_batt_temp_over_hot;
	int usbtemp_ntc_temp_low;
	int usbtemp_ntc_temp_high;
	int usbtemp_temp_gap_low_with_batt_temp;
	int usbtemp_temp_gap_with_batt_temp_in_over_hot;
	int usbtemp_temp_gap_high_with_batt_temp;
	int usbtemp_temp_gap_low_without_batt_temp;
	int usbtemp_temp_gap_high_without_batt_temp;
	int usbtemp_rise_fast_temp_low;
	int usbtemp_rise_fast_temp_high;
	int usbtemp_rise_fast_temp_count_low;
	int usbtemp_rise_fast_temp_count_high;
	int usbtemp_ntc_temp_l_compensation;
	int usbtemp_ntc_temp_r_compensation;
	bool support_hot_enter_kpoc;

	int usbtemp_cool_down_ntc_low;
	int usbtemp_cool_down_ntc_high;
	int usbtemp_cool_down_gap_low;
	int usbtemp_cool_down_gap_high;
	int usbtemp_cool_down_recover_ntc_low;
	int usbtemp_cool_down_recover_ntc_high;
	int usbtemp_cool_down_recover_gap_low;
	int usbtemp_cool_down_recover_gap_high;

	int usbtemp_batttemp_gap;
	int usbtemp_batttemp_recover_gap;
	int usbtemp_batttemp_current_gap;
	bool usbtemp_change_gap;
	int usbtemp_max_temp_diff;
	int usbtemp_cool_down_temp_gap;
	int usbtemp_cool_down_temp_first_gap;
	int usbtemp_cool_down_temp_second_gap;
	int usbtemp_cool_down_recovery_temp_gap;
	int usbtemp_cool_down_recovery_temp_first_gap;
	int usbtemp_cool_down_recovery_temp_second_gap;
	int usbtemp_max_temp_thr;
	int usbtemp_temp_up_time_thr;
	int usbtemp_otg_cc_boot_current_limit;
	struct otg_output_config otg_output[OTG_OUTPUT_LEVEL_MAX];
};
typedef union {
	int buf[24];
	struct {
		int plugin_sbu1_volt;
		int plugin_sbu2_volt;
		int plugin_sbu1_pullup_volt;
		int plugin_sbu2_pullup_volt;
		int cc1_volt;
		int cc2_volt;
		int dp_volt;
		int dm_volt;
		int plugout_sbu1_volt;
		int plugout_sbu2_volt;
		int plugout_sbu1_pullup_volt;
		int plugout_sbu2_pullup_volt;
		int sbu1_max_volt;
		int sbu1_min_volt;
		int sbu1_pullup_max_volt;
		int sbu1_pullup_min_volt;
		int sbu2_max_volt;
		int sbu2_min_volt;
		int sbu2_pullup_max_volt;
		int sbu2_pullup_min_volt;
		int cc1_max_volt;
		int cc1_min_volt;
		int cc2_max_volt;
		int cc2_min_volt;
	} info;
} oplus_lpd_info;

struct oplus_lpd_rang {
	int low_thr_mv;
	int high_thr_mv;
};

struct oplus_lpd_spec_config {
	u32 support_status;
	struct oplus_lpd_rang sbu_rang;
	struct oplus_lpd_rang sbu_pullup_rang;
	u32 lpd_sbu_ovp_thr_mv;
	int lpd_retry_count;
	int lpd_info_status;
	int lpd_cc_status;
	int sbu1_err;
	int sbu2_err;
	int sbu1_diff_err;
	int sbu2_diff_err;
	int ignore_sbu1_err_data;
	int ignore_sbu2_err_data;
};

struct oplus_mms_wired_abnormal_monitor {
	unsigned int err_code;
	int online_status_err_count;

	struct delayed_work online_status_err_work;
};

struct oplus_mms_wired {
	struct device *dev;
	struct oplus_chg_ic_dev *buck_ic;
	struct oplus_chg_ic_dev *voocphy_ic;
	struct oplus_mms *wired_topic;
	struct oplus_mms *gauge_topic;
	struct oplus_mms *vooc_topic;
	struct oplus_mms *comm_topic;
	struct oplus_mms *err_topic;
	struct oplus_mms *cpa_topic;
	struct oplus_mms *ufcs_topic;
	struct oplus_mms *wls_topic;
	struct oplus_mms *reverse_topic;
	struct oplus_mms *batt_bal_topic;
	struct oplus_mms *dischg_boost_topic;
	struct mms_subscribe *gauge_subs;
	struct mms_subscribe *vooc_subs;
	struct mms_subscribe *comm_subs;
	struct mms_subscribe *cpa_subs;
	struct mms_subscribe *ufcs_subs;
	struct mms_subscribe *wls_subs;
	struct mms_subscribe *reverse_subs;
	struct mms_subscribe *dischg_boost_subs;

	struct votable *chg_icl_votable;
	struct votable *chg_suspend_votable;
	struct votable *chg_disable_votable;
	struct votable *cool_down_votable;
	struct votable *pd_svooc_votable;
	struct votable *vooc_chg_auto_mode_votable;
	struct votable *fv_max_votable;
	struct votable *fv_min_votable;
	struct votable *wired_fcc_votable;
	struct votable *vooc_curr_votable;
	struct votable *ufcs_curr_votable;
	struct votable *otg_disable_votable;
	struct votable *flash_mode_votable;

	struct delayed_work mms_wired_init_work;
	struct delayed_work usbtemp_recover_work;
	struct delayed_work ccdetect_work;
	struct delayed_work typec_state_change_work;
	struct delayed_work power_role_change_work;
	struct delayed_work typec_state_notify_work;
	struct delayed_work svid_handler_work;
	struct delayed_work power_changed_work;
	struct delayed_work voocphy_dischg_work;
	struct work_struct err_handler_work;
	struct work_struct plugin_handler_work;
	struct work_struct chg_type_change_handler_work;
	struct work_struct cpa_chg_type_change_handler_work;
	struct work_struct gauge_update_work;
	struct delayed_work otg_enable_handler_work;
	struct work_struct voltage_change_work;
	struct work_struct current_change_work;
	struct work_struct bc12_completed_work;
	struct work_struct data_role_changed_handler_work;
	struct work_struct back_ui_soc_work;
	struct work_struct otg_enable_pending_work;
	struct work_struct pd_completed_handler_work;
	struct work_struct icl_done_handler_work;
	struct work_struct reverse_usbtemp_check_work;

	struct wakeup_source *usbtemp_wakelock;
	struct adc_vol_temp_info *adc_vol_temp_info;
	struct task_struct *oplus_usbtemp_kthread;
	wait_queue_head_t oplus_usbtemp_wq;
	struct alarm usbtemp_alarm_timer;
	struct work_struct usbtemp_restart_work;
	struct oplus_usbtemp_spec_config usbtemp_spec;
	struct oplus_mms_wired_abnormal_monitor wam;
	struct power_supply *batt_psy;

	int vbat_mv;
	int batt_temp;
	int batt_realy_temp;
	int batt_curr;
	int shell_temp;
	int usbtemp_volt_l;
	int usbtemp_volt_r;
	int usb_temp_l;
	int usb_temp_r;
	int smart_charge_user;

	enum oplus_usbtemp_timer_stage usbtemp_timer_stage;
	bool support_usbtemp_protect_v2;
	bool support_usbtemp_discharge_protect;
	bool new_usbtemp_cool_down_support;
	bool high_temp_scheme;
	int buck_eis_current_rate;
	int usbtemp_curr_status;
	int usbtemp_batt_current;
	int usbtemp_pre_batt_current;
	int voocphy_dischg_retry_cnt;
	spinlock_t voocphy_dischg_enable_lock;

	bool wired_present;
	bool wired_online;
	bool wired_online_init_flag;
	bool vooc_started;
	bool vooc_online;
	bool vooc_charging;
	bool vooc_online_keep;
	bool ship_mode;
	bool dischg_flag;
	bool usbtemp_check;
	bool reverse_usbtemp_check;
	bool otg_enable;
	bool otg_disbale_status;
	bool charge_suspend;
	bool charging_disable;
	bool bc12_completed;
	bool cc_detect_support;
	bool voocphy_dischg_enable;
	int pdo_volt;
	pd_msg_data pdo[PPS_PDO_MAX];
	unsigned int vooc_sid;
	unsigned int usb_status;
	bool abnormal_adapter;
	int ui_soc;
	struct mutex fcc_lock;

	int bcc_current;
	int bcc_cool_down;
	struct mutex bcc_curr_done_mutex;
	int bcc_curr_done;

	bool pre_ship_mode_support;
	bool cpa_support;
	bool ufcs_online;
	bool wls_online;
	bool otg_enable_pending;
	bool support_wlsotg_non_coexistence;

	int wls_upgrading;
	bool otg_online_status;
	struct work_struct wls_upgrading_work;
	bool pd_completed;
	oplus_lpd_info lpd_info;
	struct delayed_work lpd_info_update_work;
	struct oplus_lpd_spec_config lpd_spec;
	bool set_icl_done;
	int otg_cur;
	bool support_adjust_reverse_chg_cur;
	int shaft_btb_temp_threshod;
	bool high_reverse_charging;
	int power_role;
	bool gauge_not_ready;
};

static struct oplus_mms_wired *g_mms_wired;

#define USBTEMP_CHARGING_CURRENT_LIMIT	3000
#define USBTEMP_CURR_TABLE_MAX		5
#define OPLUS_USBTEMP_HIGH_CURR		1
#define OPLUS_USBTEMP_LOW_CURR		0
#define OPLUS_USBTEMP_CURR_CHANGE_TEMP	3
#define OPLUS_USBTEMP_CHANGE_RANGE_TIME	30

#define OPCHG_USBTEMP_BATT_TEMP_LOW			50
#define OPCHG_USBTEMP_BATT_TEMP_HIGH			50
#define OPCHG_USBTEMP_BATT_TEMP_OVER_HOT		60
#define OPCHG_USBTEMP_NTC_TEMP_LOW			57
#define OPCHG_USBTEMP_NTC_TEMP_HIGH			69
#define OPCHG_USBTEMP_GAP_LOW_WITH_BATT_TEMP		7
#define OPCHG_USBTEMP_GAP_WITH_BATT_TEMP_IN_OVER_HOT	15
#define OPCHG_USBTEMP_GAP_HIGH_WITH_BATT_TEMP		12
#define OPCHG_USBTEMP_GAP_LOW_WITHOUT_BATT_TEMP		12
#define OPCHG_USBTEMP_GAP_HIGH_WITHOUT_BATT_TEMP	24
#define OPCHG_USBTEMP_RISE_FAST_TEMP_LOW		3
#define OPCHG_USBTEMP_RISE_FAST_TEMP_HIGH		3
#define OPCHG_USBTEMP_RISE_FAST_TEMP_COUNT_LOW		30
#define OPCHG_USBTEMP_RISE_FAST_TEMP_COUNT_HIGH		20

#define OPCHG_USBTEMP_COOL_DOWN_NTC_LOW			54
#define OPCHG_USBTEMP_COOL_DOWN_NTC_HIGH		65
#define OPCHG_USBTEMP_COOL_DOWN_GAP_LOW			12
#define OPCHG_USBTEMP_COOL_DOWN_GAP_HIGH		20
#define OPCHG_USBTEMP_COOL_DOWN_RECOVER_NTC_LOW		48
#define OPCHG_USBTEMP_COOL_DOWN_RECOVER_NTC_HIGH	60
#define OPCHG_USBTEMP_COOL_DOWN_RECOVER_GAP_LOW		6
#define OPCHG_USBTEMP_COOL_DOWN_RECOVER_GAP_HIGH	15
#define OPCHG_USBTEMP_OTG_CC_BOOT_CURRENT_LIMIT		1300000

#define NORMAL_CURRENT_MA		2000
#define USBTEMP_RECOVER_INTERVAL	(14400 * 1000) /*4 hours*/
#define USBTEMP_CC_RECOVER_INTERVAL	(300 * 1000) /*5 mins*/
static int usbtemp_recover_interval = USBTEMP_RECOVER_INTERVAL;
module_param(usbtemp_recover_interval, int, 0644);

static int usbtemp_recover_test = 0;
module_param(usbtemp_recover_test, int, 0644);

static int usbtemp_debug = 0;
module_param(usbtemp_debug, int, 0644);
#define OPEN_LOG_BIT BIT(0)
#define TEST_FUNC_BIT BIT(1)
#define TEST_CURRENT_BIT BIT(2)
MODULE_PARM_DESC(usbtemp_debug, "debug usbtemp");

static int usbtemp_dbg_tempr = 0;
module_param(usbtemp_dbg_tempr, int, 0644);
MODULE_PARM_DESC(usbtemp_dbg_tempr, "debug usbtemp temp r");

static int usbtemp_dbg_templ = 0;
module_param(usbtemp_dbg_templ, int, 0644);
MODULE_PARM_DESC(usbtemp_dbg_templ, "debug usbtemp temp l");

static int usbtemp_dbg_curr_status = -1;
module_param(usbtemp_dbg_curr_status, int, 0644);
MODULE_PARM_DESC(usbtemp_dbg_curr_status, "debug usbtemp current status");

__maybe_unused static int voltage_max_table_mv[OPLUS_CHG_USB_TYPE_MAX] = {
	5000,	/* OPLUS_CHG_USB_TYPE_UNKNOWN */
	5000,	/* OPLUS_CHG_USB_TYPE_SDP */
	5000,	/* OPLUS_CHG_USB_TYPE_DCP */
	5000,	/* OPLUS_CHG_USB_TYPE_CDP */
	5000,	/* OPLUS_CHG_USB_TYPE_ACA */
	5000,	/* OPLUS_CHG_USB_TYPE_C */
	9000,	/* OPLUS_CHG_USB_TYPE_PD */
	9000,	/* OPLUS_CHG_USB_TYPE_PD_DRP */
	9000,	/* OPLUS_CHG_USB_TYPE_PD_PPS */
	5000,	/* OPLUS_CHG_USB_TYPE_PD_SDP */
	5000,	/* OPLUS_CHG_USB_TYPE_APPLE_BRICK_ID */
	9000,	/* OPLUS_CHG_USB_TYPE_QC2 */
	9000,	/* OPLUS_CHG_USB_TYPE_QC3 */
	5000,	/* OPLUS_CHG_USB_TYPE_VOOC */
	10000,	/* OPLUS_CHG_USB_TYPE_SVOOC */
	10000,	/* OPLUS_CHG_USB_TYPE_UFCS TODO:power */
};

static const char *const oplus_wired_chg_type_text[] = {
	[OPLUS_CHG_USB_TYPE_UNKNOWN] = "unknown",
	[OPLUS_CHG_USB_TYPE_SDP] = "sdp",
	[OPLUS_CHG_USB_TYPE_DCP] = "dcp",
	[OPLUS_CHG_USB_TYPE_CDP] = "cdp",
	[OPLUS_CHG_USB_TYPE_ACA] = "aca",
	[OPLUS_CHG_USB_TYPE_C] = "type-c",
	[OPLUS_CHG_USB_TYPE_PD] = "pd",
	[OPLUS_CHG_USB_TYPE_PD_DRP] = "pd_drp",
	[OPLUS_CHG_USB_TYPE_PD_PPS] = "pps",
	[OPLUS_CHG_USB_TYPE_PD_SDP] = "pd_sdp",
	[OPLUS_CHG_USB_TYPE_APPLE_BRICK_ID] = "ocp",
	[OPLUS_CHG_USB_TYPE_QC2] = "qc2",
	[OPLUS_CHG_USB_TYPE_QC3] = "qc3",
	[OPLUS_CHG_USB_TYPE_VOOC] = "vooc",
	[OPLUS_CHG_USB_TYPE_SVOOC] = "svooc",
	[OPLUS_CHG_USB_TYPE_UFCS] = "ufcs",
	[OPLUS_CHG_USB_TYPE_MAX] = "invalid",
};

const char *oplus_wired_get_chg_type_str(enum oplus_chg_usb_type type)
{
	if ((type < OPLUS_CHG_USB_TYPE_UNKNOWN) || (type >= OPLUS_CHG_USB_TYPE_MAX))
		type = OPLUS_CHG_USB_TYPE_MAX;
	return oplus_wired_chg_type_text[type];
}

__maybe_unused static bool
is_batt_psy_available(struct oplus_mms_wired *chip)
{
	if (!chip->batt_psy)
		chip->batt_psy = power_supply_get_by_name("battery");
	return !!chip->batt_psy;
}

__maybe_unused static bool
is_chg_icl_votable_available(struct oplus_mms_wired *chip)
{
	if (!chip->chg_icl_votable)
		chip->chg_icl_votable = find_votable("WIRED_ICL");
	return !!chip->chg_icl_votable;
}

__maybe_unused static bool
is_chg_suspend_votable_available(struct oplus_mms_wired *chip)
{
	if (!chip->chg_suspend_votable)
		chip->chg_suspend_votable = find_votable("WIRED_CHARGE_SUSPEND");
	return !!chip->chg_suspend_votable;
}

__maybe_unused static bool
is_chg_disable_votable_available(struct oplus_mms_wired *chip)
{
	if (!chip->chg_disable_votable)
		chip->chg_disable_votable = find_votable("WIRED_CHARGING_DISABLE");
	return !!chip->chg_disable_votable;
}

__maybe_unused static bool
is_cool_down_votable_available(struct oplus_mms_wired *chip)
{
	if (!chip->cool_down_votable)
		chip->cool_down_votable = find_votable("COOL_DOWN");
	return !!chip->cool_down_votable;
}

__maybe_unused static bool
is_vooc_curr_votable_available(struct oplus_mms_wired *chip)
{
	if (!chip->vooc_curr_votable)
		chip->vooc_curr_votable = find_votable("VOOC_CURR");
	return !!chip->vooc_curr_votable;
}

__maybe_unused static bool
is_ufcs_curr_votable_available(struct oplus_mms_wired *chip)
{
	if (!chip->ufcs_curr_votable)
		chip->ufcs_curr_votable = find_votable("UFCS_CURR");
	return !!chip->ufcs_curr_votable;
}

__maybe_unused static bool
is_pd_svooc_votable_available(struct oplus_mms_wired *chip)
{
	if (!chip->pd_svooc_votable)
		chip->pd_svooc_votable =
			find_votable("PD_SVOOC");
	return !!chip->pd_svooc_votable;
}

__maybe_unused static bool
is_err_topic_available(struct oplus_mms_wired *chip)
{
	if (!chip->err_topic)
		chip->err_topic = oplus_mms_get_by_name("error");
	return !!chip->err_topic;
}

__maybe_unused static bool
is_batt_bal_topic_available(struct oplus_mms_wired *chip)
{
	if (!chip->batt_bal_topic)
		chip->batt_bal_topic = oplus_mms_get_by_name("batt_bal");

	return !!chip->batt_bal_topic;
}

__maybe_unused static bool
is_vooc_chg_auto_mode_votable_available(struct oplus_mms_wired *chip)
{
	if (!chip->vooc_chg_auto_mode_votable)
		chip->vooc_chg_auto_mode_votable =
			find_votable("VOOC_CHG_AUTO_MODE");
	return !!chip->vooc_chg_auto_mode_votable;
}

__maybe_unused static bool
is_fv_max_votable_available(struct oplus_mms_wired *chip)
{
	if (!chip->fv_max_votable)
		chip->fv_max_votable =
			find_votable("FV_MAX");
	return !!chip->fv_max_votable;
}

__maybe_unused static bool
is_fv_min_votable_available(struct oplus_mms_wired *chip)
{
	if (!chip->fv_min_votable)
		chip->fv_min_votable =
			find_votable("FV_MIN");
	return !!chip->fv_min_votable;
}

static bool is_wired_fcc_votable_available(struct oplus_mms_wired *chip)
{
	if (!chip->wired_fcc_votable)
		chip->wired_fcc_votable = find_votable("WIRED_FCC");
	return !!chip->wired_fcc_votable;
}

int oplus_wired_get_charger_cycle(void)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;
	int cycle = 0;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
				OPLUS_IC_FUNC_GET_CHARGER_CYCLE,
				&cycle);
	if (rc < 0) {
		if (rc != -ENOTSUPP)
			chg_err("error: get charger cycle, rc=%d\n", rc);
	}

	return cycle;
}

void oplus_wired_dump_regs(void)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return;
	}

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_REG_DUMP);
	if (rc < 0)
		chg_err("can't dump wired reg, rc=%d\n", rc);
}

int oplus_wired_get_vbus(void)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;
	int vbus_mv;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
				OPLUS_IC_FUNC_BUCK_GET_INPUT_VOL,
				&vbus_mv);
	if (rc < 0) {
		chg_err("can't get vbus, rc=%d\n", rc);
		return rc;
	}

	return vbus_mv;
}

bool oplus_wired_is_usb_aicl_enhance(void)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;
	bool enable;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return false;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
				OPLUS_IC_FUNC_BUCK_GET_USB_AICL_ENHANCE,
				&enable);
	if (rc < 0)
		return false;

	return enable;
}

bool oplus_wired_get_vbus_collapse_status(void)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;
	bool collapse;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return false;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_BUCK_GET_VBUS_COLLAPSE_STATUS,
			       &collapse);
	if (rc < 0) {
		if (rc != -ENOTSUPP)
			chg_err("can't get wired vbus collapse status, rc=%d\n",
				rc);
		return false;
	}

	return collapse;
}

int oplus_wired_set_icl(int icl_ma, bool step)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}

	if (chip->charge_suspend) {
		chg_info("charge suspend, set icl to 0\n");
		icl_ma = 0;
		step = false;
	}
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if ((get_eng_version() == HIGH_TEMP_AGING || get_eng_version() == AGING) &&
			(oplus_wired_get_chg_type() != OPLUS_CHG_USB_TYPE_CDP &&
			 oplus_wired_get_chg_type() != OPLUS_CHG_USB_TYPE_SDP)) {
		icl_ma = NORMAL_CURRENT_MA;
		chg_err("HIGH_TEMP_AGING/AGING, set input_limit 2A\n");
	}
#endif
	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_BUCK_SET_ICL,
			       false, step, icl_ma);
	if (rc < 0)
		chg_err("set icl to %d error, rc=%d\n", icl_ma, rc);
	else
		chg_info("set icl to %d, step=%s\n", icl_ma,
			 step ? "true" : "false");

	if (chip->set_icl_done == false && chip->wired_present == true) {
		/* first insert need to inform icl status */
		schedule_work(&chip->icl_done_handler_work);
		chip->set_icl_done = true;
	}

	return rc;
}

int oplus_wired_set_icl_by_vooc(struct oplus_mms *topic, int icl_ma)
{
	struct oplus_mms_wired *chip;
	int rc;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -ENODEV;
	}
	chip = oplus_mms_get_drvdata(topic);

	if (chip->charge_suspend) {
		chg_info("charge suspend, set icl to 0\n");
		oplus_wired_set_icl(0, false);
		return 0;
	}

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_BUCK_SET_ICL,
			       true, false, icl_ma);
	if (rc)
		chg_err("set icl to %d mA fail, rc=%d\n", icl_ma, rc);
	else
		chg_info("set icl to %d mA\n", icl_ma);

	return rc;
}

int oplus_wired_set_fcc(int fcc_ma)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}

	mutex_lock(&chip->fcc_lock);
	if (READ_ONCE(chip->charging_disable)) {
		chg_info("charging disable\n");
		fcc_ma = 0;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
				OPLUS_IC_FUNC_BUCK_SET_FCC,
				fcc_ma);
	if (rc < 0)
		chg_err("set fcc to %d error, rc=%d\n", fcc_ma, rc);
	else
		chg_info("set fcc to %d\n", fcc_ma);
	mutex_unlock(&chip->fcc_lock);

	return rc;
}

int oplus_wired_set_aicl_point(void)
{
	int rc = 0;
	int vbatt = 0;
	struct oplus_mms_wired *chip = g_mms_wired;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}

	vbatt = oplus_gauge_get_batt_mvolts();
	rc = oplus_chg_ic_func(chip->buck_ic,
				OPLUS_IC_FUNC_BUCK_SET_AICL_POINT,
				vbatt);
	if (rc < 0)
		chg_err("set aicl to %d error, rc=%d\n", vbatt, rc);
	else
		chg_debug("set aicl to %d\n", vbatt);

	return rc;
}

int oplus_wired_smt_test(char buf[], int len)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_SMT_TEST, buf, len);

	return rc;
}

bool oplus_wired_is_present(void)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;
	bool present;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return false;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_BUCK_INPUT_PRESENT,
			       &present);
	if (rc < 0) {
		chg_err("can't get wired present status, rc=%d\n", rc);
		return false;
	}

	return present;
}

#define VDM_INFO_MAX	5
bool oplus_wired_is_gamepad_active(void)
{
	int rc;
	struct oplus_mms_wired *chip = g_mms_wired;
	u32 vdm_info[VDM_INFO_MAX] = {0};
	int vdm_cnt = VDM_INFO_MAX;
	int i = 0;
	bool cert_vdo = false;
	bool product_vdo = false;

	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return false;
	}
	if (chip->buck_ic == NULL) {
		chg_err("buck_ic is NULL\n");
		return false;
	}

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_BUCK_GET_VDM_INFO,
			       vdm_info, &vdm_cnt);
	if (rc < 0 && rc != -ENOTSUPP) {
		chg_err("get vdm info failed, rc=%d\n", rc);
		return false;
	}

	for (i = 0; i < vdm_cnt; i++) {
		if (vdm_info[i] == 0x22d9)
			cert_vdo = true;
		if (vdm_info[i] == 0x10007)
			product_vdo = true;
	}

	if (cert_vdo && product_vdo)
		return true;

	return false;
}

int oplus_wired_send_get_sink_cap(void)
{
	struct oplus_mms_wired *chip = g_mms_wired;
	int rc = 0;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	if (chip->buck_ic == NULL) {
		chg_err("buck_ic is NULL");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_BUCK_SEND_GET_SINK_CAP);
	if (rc < 0 && rc != -ENOTSUPP)
		chg_err("can't send get sink cap, rc=%d\n", rc);

	return rc;
}

int oplus_wired_get_vdm_info(struct oplus_mms *topic, u32 *vdm_data, int *vdm_cnt)
{
	struct oplus_mms_wired *chip;
	int rc = 0;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -ENODEV;
	}
	chip = oplus_mms_get_drvdata(topic);

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	if (chip->buck_ic == NULL) {
		chg_err("buck_ic is NULL");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_BUCK_GET_VDM_INFO, vdm_data, vdm_cnt);
	if (rc < 0)
		chg_err("can't get wired vdm info, rc=%d\n", rc);

	return rc;
}

static int oplus_wired_hardware_init(struct oplus_mms_wired *chip)
{
	int rc = 0;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_BUCK_HARDWARE_INIT);
	if (rc < 0)
		chg_err("can't hardware_init, rc=%d\n", rc);
	chg_info("hardware_init\n");

	return rc;
}

int oplus_wired_input_enable(bool enable)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_BUCK_INPUT_SUSPEND,
			       !enable);
	if (rc < 0)
		chg_err("can't %s wired input, rc=%d\n",
			enable ? "enable" : "disable", rc);
	else
		chg_info("%s wired input\n", enable ? "enable" : "disable");

	chip->charge_suspend = !enable;

	return rc;
}

bool oplus_wired_input_is_enable(void)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;
	bool enable;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return false;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_BUCK_INPUT_IS_SUSPEND,
			       &enable);
	if (rc < 0) {
		chg_err("can't get wired input enable status, rc=%d\n", rc);
		return rc;
	}

	return !enable;
}

static void oplus_wired_set_charging_disable(
		struct oplus_mms_wired *chip, bool disable)
{
	int rc;
	struct mms_msg *msg;

	if (chip->charging_disable == disable)
		return;

	chip->charging_disable = disable;
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  WIRED_ITEM_CHARGING_DISABLE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->wired_topic, msg);
	if (rc < 0) {
		chg_err("publish charging disable error, rc=%d\n", rc);
		kfree(msg);
	}

	chg_info("charging_disable=%d\n", chip->charging_disable);
}

int oplus_wired_output_enable(bool enable)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;
	static bool hw_init = false;
	static bool enable_err = false;

	if (chip == NULL) {
		chg_err("chip is NULL");
		enable_err = true;
		return -ENODEV;
	}

	oplus_wired_set_charging_disable(chip, !enable);

	if(enable && !hw_init) {
		hw_init = true;
		oplus_wired_hardware_init(chip);
		if (is_fv_max_votable_available(chip))
			rerun_election(chip->fv_max_votable, false);
		if (is_fv_min_votable_available(chip))
			rerun_election(chip->fv_min_votable, false);
		if (enable_err) {
			enable_err = false;
			chg_info("rerun fcc vote\n");
			if (is_wired_fcc_votable_available(chip))
				rerun_election(chip->wired_fcc_votable, false);
		}
	} else {
		hw_init = enable;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_BUCK_OUTPUT_SUSPEND,
			       !enable);
	if (rc < 0)
		chg_err("can't %s wired output, rc=%d\n",
			enable ? "enable" : "disable", rc);
	else
		chg_info("%s wired output\n", enable ? "enable" : "disable");

	return rc;
}

bool oplus_wired_output_is_enable(void)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;
	bool enable;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return false;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_BUCK_OUTPUT_IS_SUSPEND,
			       &enable);
	if (rc < 0) {
		chg_err("can't get wired output enable status, rc=%d\n", rc);
		return false;
	}

	return !enable;
}

int oplus_wired_iterm_check(struct oplus_mms *topic, bool enable)
{
	struct oplus_mms_wired *chip;
	int rc = 0;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -ENODEV;
	}
	chip = oplus_mms_get_drvdata(topic);

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_BUCK_ITEM_CHECK, enable);
	if (rc < 0)
		chg_err("can't %s wired iterm check, rc=%d\n", enable ? "enable" : "disable", rc);

	return rc;
}

bool oplus_wired_get_supplementary_power_mos(void)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;
	bool enable = false;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return false;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
				OPLUS_IC_FUNC_BUCK_GET_POWER_MOS_ENABLE,
				&enable);
	if (rc < 0)
		return false;

	return enable;
}

int oplus_wired_set_supplementary_power_mos(struct oplus_mms *topic, bool enable)
{
	struct oplus_mms_wired *chip;
	int rc = 0;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -ENODEV;
	}
	chip = oplus_mms_get_drvdata(topic);

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_BUCK_SET_POWER_MOS_ENABLE, enable);
	if (rc < 0)
		chg_err("can't %s wired supplementary power mos, rc=%d\n", enable ? "enable" : "disable", rc);

	return rc;
}

int oplus_wired_get_icl(void)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;
	int icl_ma;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_BUCK_GET_ICL,
			       &icl_ma);
	if (rc < 0) {
		chg_err("can't get wired icl, rc=%d\n", rc);
		return rc;
	}

	return icl_ma;
}

int oplus_wired_set_fv(int fv_mv)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
				OPLUS_IC_FUNC_BUCK_SET_FV,
				fv_mv);
	if (rc < 0)
		chg_err("set fv to %d error, rc=%d\n", fv_mv, rc);
	else
		chg_info("set fv to %d\n", fv_mv);

	return rc;
}

int oplus_wired_get_fv(int *fv_mv)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_BUCK_GET_FV,
			       fv_mv);
	if (rc < 0) {
		chg_err("can't get fv, rc=%d\n", rc);
		return rc;
	}

	return rc;
}

int oplus_wired_set_iterm(int iterm_ma)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
				OPLUS_IC_FUNC_BUCK_SET_ITERM,
				iterm_ma);
	if (rc < 0)
		chg_err("set iterm to %d error, rc=%d\n", iterm_ma, rc);
	else
		chg_info("set iterm to %d\n", iterm_ma);

	return rc;
}

int oplus_wired_set_rechg_vol(int vol_mv)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
				OPLUS_IC_FUNC_BUCK_SET_RECHG_VOL,
				vol_mv);
	if (rc < 0)
		chg_err("set rechg vol to %d error, rc=%d\n", vol_mv, rc);
	else
		chg_info("set rechg vol to %d\n", vol_mv);

	return rc;
}

int oplus_wired_get_ibus(void)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;
	int ibus_ma;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_BUCK_GET_INPUT_CURR,
			       &ibus_ma);
	if (rc < 0) {
		chg_err("can't get wired ibus, rc=%d\n", rc);
		return rc;
	}

	return ibus_ma;
}

static int oplus_wired_get_real_chg_type(struct oplus_mms_wired *chip)
{
	int rc = 0;
	int type;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return OPLUS_CHG_USB_TYPE_UNKNOWN;
	}

	if (chip->wired_present) {
		rc = oplus_chg_ic_func(chip->buck_ic,
				       OPLUS_IC_FUNC_BUCK_GET_CHARGER_TYPE,
				       &type);
		if (rc < 0) {
			chg_err("can't get wired chg type, rc=%d\n", rc);
			type = OPLUS_CHG_USB_TYPE_UNKNOWN;
		}
	} else {
		type = OPLUS_CHG_USB_TYPE_UNKNOWN;
	}

	if (!chip->cpa_support && chip->wired_online) {
		if (sid_to_adapter_chg_type(chip->vooc_sid) == CHARGER_TYPE_VOOC)
			type = OPLUS_CHG_USB_TYPE_VOOC;
		else if (sid_to_adapter_chg_type(chip->vooc_sid) == CHARGER_TYPE_SVOOC)
			type = OPLUS_CHG_USB_TYPE_SVOOC;
	}

	return type;
}

int oplus_wired_get_chg_type(void)
{
	union mms_msg_data data = { 0 };
	struct oplus_mms_wired *chip = g_mms_wired;
	int rc;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return OPLUS_CHG_USB_TYPE_UNKNOWN;
	}

	if (chip->cpa_topic == NULL)
		return oplus_wired_get_real_chg_type(chip);

	rc = oplus_mms_get_item_data(chip->cpa_topic, CPA_ITEM_CHG_TYPE, &data, false);
	if (rc < 0) {
		chg_err("can't get charger type by cpa, rc=%d\n", rc);
		return OPLUS_CHG_USB_TYPE_UNKNOWN;
	}

	return data.intval;
}

int oplus_wired_otg_boost_enable(bool enable)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_OTG_BOOST_ENABLE,
			       enable);
	if (rc < 0) {
		chg_err("can't %s otg boost, rc=%d\n",
			enable ? "enable" : "disable", rc);
		return rc;
	}

	if (is_vooc_chg_auto_mode_votable_available(chip))
		vote(chip->vooc_chg_auto_mode_votable, OTG_ONLINE_VOTER,
		     enable, enable, false);
	else
		chg_err("vooc_chg_auto_mode_votable not found\n");

	chg_info("%s otg boost\n", enable ? "enable" : "disable");

	return rc;
}

int oplus_wired_set_otg_boost_vol(int vol_mv)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_SET_OTG_BOOST_VOL,
			       vol_mv);
	if (rc < 0)
		chg_err("can't set otg boost vol to %d, rc=%d\n", vol_mv, rc);
	chg_info("set otg boost vol to %d\n", vol_mv);

	return rc;
}

int oplus_wired_set_otg_boost_curr_limit(int curr_ma)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_SET_OTG_BOOST_CURR_LIMIT,
			       curr_ma);
	if (rc < 0)
		chg_err("can't set otg boost curr limit to %d, rc=%d\n", curr_ma, rc);
	chg_info("set otg boost curr limit to %d\n", curr_ma);

	return rc;
}

int oplus_wired_aicl_enable(bool enable)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_BUCK_AICL_ENABLE,
			       enable);
	if (rc < 0)
		chg_err("can't %s aicl, rc=%d\n",
			enable ? "enable" : "disable", rc);
	chg_info("%s aicl\n", enable ? "enable" : "disable");

	return rc;
}

int oplus_wired_aicl_rerun(void)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_BUCK_AICL_RERUN);
	chg_info("rerun aicl, rc = %d\n", rc);

	return rc;
}

int oplus_wired_aicl_reset(void)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_BUCK_AICL_RESET);
	if (rc < 0)
		chg_err("can't reset aicl, rc=%d\n", rc);
	chg_info("reset aicl\n");

	return rc;
}

int oplus_wired_get_cc_orientation(void)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;
	int cc_orientation;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_BUCK_GET_CC_ORIENTATION,
			       &cc_orientation);
	if (rc < 0) {
		chg_err("can't get cc orientation, rc=%d\n", rc);
		return rc;
	}

	return cc_orientation;
}

int oplus_wired_get_hw_detect(void)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;
	int hw_detect;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return 0;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_BUCK_GET_HW_DETECT,
			       &hw_detect, false);
	if (rc < 0) {
		chg_err("can't get hw detect status, rc=%d\n", rc);
		if (rc == -ENOTSUPP)
			return CC_DETECT_NULL;
		else
			return rc;
	}

	if (!!hw_detect)
		return CC_DETECT_PLUGIN;
	else
		return CC_DETECT_NOTPLUG;
}

int oplus_wired_set_dischg_status(bool dischg_en)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return 0;
	}

	chg_err("set dischg  status, %d \n", dischg_en);
	rc = oplus_chg_ic_func(chip->buck_ic,
					OPLUS_IC_FUNC_SET_USB_DISCHG_ENABLE, dischg_en);
	if (rc < 0) {
		chg_err("can't set USB_DISCHG status, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

int oplus_set_ovp_forced(bool enable)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}
	if (chip->buck_ic == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
				OPLUS_IC_FUNC_BUCK_SET_OVP_FORCED,
				enable);
	if (rc < 0)
		chg_err("set ovp forced=%d error, rc=%d\n", enable, rc);
	else
		chg_debug("set ovp forced=%d\n", enable);

	return rc;
}

int oplus_wired_get_hw_detect_recheck(void)
{
        int rc = 0;
        struct oplus_mms_wired *chip = g_mms_wired;
        int hw_detect;

        if (chip == NULL) {
                chg_err("chip is NULL");
                return 0;
        }

        rc = oplus_chg_ic_func(chip->buck_ic,
                               OPLUS_IC_FUNC_BUCK_GET_HW_DETECT,
                               &hw_detect, true);
        if (rc < 0) {
                chg_err("can't get hw detect status, rc=%d\n", rc);
                if (rc == -ENOTSUPP)
                        return CC_DETECT_NULL;
                else
                        return rc;
        }

        if (!!hw_detect)
                return CC_DETECT_PLUGIN;
        else
                return CC_DETECT_NOTPLUG;
}

int oplus_wired_rerun_bc12(void)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_BUCK_RERUN_BC12);
	if (rc < 0)
		chg_err("can't rerun bc1.2, rc=%d\n", rc);
	chg_info("rerun bc1.2\n");

	return rc;
}

int oplus_wired_qc_detect_enable(bool enable)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_BUCK_QC_DETECT_ENABLE,
			       enable);
	if (rc < 0)
		chg_err("can't %s qc detect, rc=%d\n",
			enable ? "enable" : "disable", rc);
	chg_info("%s qc detect\n", enable ? "enable" : "disable");

	return rc;
}

int oplus_wired_shipmode_enable(bool enable)
{
	struct oplus_mms_wired *chip = g_mms_wired;
	int rc;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	chip->ship_mode = enable;
	if (chip->ship_mode && chip->pre_ship_mode_support == 1) {
		rc = oplus_chg_ic_func(chip->buck_ic,
				       OPLUS_IC_FUNC_BUCK_SHIPMODE_ENABLE,
				       true);
		if (rc < 0)
			chg_err("can't enable ship mode, rc=%d\n", rc);
		else
			chg_err("enable ship mode\n");
	}
	return 0;
}

bool oplus_wired_shipmode_is_enabled(void)
{
	struct oplus_mms_wired *chip = g_mms_wired;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return false;
	}

	return chip->ship_mode;
}

static bool is_voocphy_ic_available(struct oplus_mms_wired *chip)
{
	if (!chip->voocphy_ic)
		chip->voocphy_ic = of_get_oplus_chg_ic(chip->dev->of_node,
						       "oplus,voocphy_ic", 0);

	return !!chip->voocphy_ic;
}

int oplus_wired_set_qc_config(enum oplus_chg_qc_version version, int vol_mv)
{
	int rc;
	struct oplus_mms_wired *chip = g_mms_wired;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	if (is_voocphy_ic_available(chip)) {
		if (vol_mv > 5000) {
			rc = oplus_chg_ic_func(chip->voocphy_ic,
					       OPLUS_IC_FUNC_VOOCPHY_SET_PDQC_CONFIG);
			if (rc < 0) {
				chg_err("can't set pdqc config, rc=%d\n", rc);
				return rc;
			}
		}
	} else {
		chg_info("oplus,voocphy_ic not found\n");
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
				   OPLUS_IC_FUNC_BUCK_SET_QC_CONFIG,
				   version, vol_mv);
	if (rc < 0)
		chg_err("can't set qc to %dmV, rc=%d\n", vol_mv, rc);
	else
		chg_info("set qc to %dmV\n", vol_mv);

	return rc;
}

int oplus_wired_set_lpd_config(struct oplus_mms *topic, int *config)
{
	struct oplus_mms_wired *chip = g_mms_wired;
	struct oplus_lpd_spec_config *lpd_spec = &chip->lpd_spec;

	lpd_spec->support_status = config[0];
	lpd_spec->sbu_rang.low_thr_mv = config[1];
	lpd_spec->sbu_rang.high_thr_mv = config[2];
	lpd_spec->sbu_pullup_rang.low_thr_mv = config[3];
	lpd_spec->sbu_pullup_rang.high_thr_mv = config[4];
	lpd_spec->lpd_sbu_ovp_thr_mv = config[5];
	chg_info("support_status:%d, rang[%dmv-%dmv], rang[%dmv-%dmv], sbu_ovp:%d", lpd_spec->support_status,
		lpd_spec->sbu_rang.low_thr_mv, lpd_spec->sbu_rang.high_thr_mv,
		lpd_spec->sbu_pullup_rang.low_thr_mv, lpd_spec->sbu_pullup_rang.high_thr_mv,
		lpd_spec->lpd_sbu_ovp_thr_mv);
	return 0;
}

int lpd_set_value(u32 value, int *target_value, int *min_value, int *max_value)
{
	struct oplus_mms_wired *chip = g_mms_wired;

	if (target_value == NULL || min_value == NULL || max_value == NULL || chip == NULL) {
		chg_err("invalid input\n");
		return -EINVAL;
	}
	*target_value = value;
	if (chip->wired_present && *min_value == -1) {
		*min_value = value;
		*max_value = value;
	} else if (chip->wired_present) {
		*min_value = min_t(int, value, *min_value);
		*max_value = max_t(int, value, *max_value);
	}
	return 0;
}

int oplus_wired_update_lpd_info(struct oplus_mms_wired *chip, int flag, bool plugin)
{
	int rc = 0;
	int i = 0;
	u32 buf[OPLUS_LPD_SEL_INVALID] = {0};

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -EINVAL;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_BUCK_GET_LPD_INFO,
			       buf, flag);
	if (rc < 0)
		chg_err("can't get lpd info, rc=%d\n", rc);

	chg_info("flag=%d, plugin:%d\n", flag, plugin);
	for (i = 0; i < OPLUS_LPD_SEL_INVALID; i++) {
		switch (flag & (0x1 << i)) {
		case OPLUS_LPD_SEL_SBU1_MASK:
			if (plugin)
				lpd_set_value(buf[i], &chip->lpd_info.info.plugin_sbu1_volt,
					      &chip->lpd_info.info.sbu1_min_volt,
					      &chip->lpd_info.info.sbu1_max_volt);
			else
				lpd_set_value(buf[i], &chip->lpd_info.info.plugout_sbu1_volt,
					      &chip->lpd_info.info.sbu1_min_volt,
					      &chip->lpd_info.info.sbu1_max_volt);
			break;
		case OPLUS_LPD_SEL_SBU2_MASK:
			if (plugin)
				lpd_set_value(buf[i], &chip->lpd_info.info.plugin_sbu2_volt,
					      &chip->lpd_info.info.sbu2_min_volt,
					      &chip->lpd_info.info.sbu2_max_volt);
			else
				lpd_set_value(buf[i], &chip->lpd_info.info.plugout_sbu2_volt,
					      &chip->lpd_info.info.sbu2_min_volt,
					      &chip->lpd_info.info.sbu2_max_volt);
			break;
		case OPLUS_LPD_SEL_SBU1_PULLUP_MASK:
			if (plugin) {
				if (buf[i] == 0 && chip->lpd_spec.ignore_sbu1_err_data == 0) {
					chip->lpd_spec.ignore_sbu1_err_data = 1;
					break;
				}
				lpd_set_value(buf[i], &chip->lpd_info.info.plugin_sbu1_pullup_volt,
					      &chip->lpd_info.info.sbu1_pullup_min_volt,
					      &chip->lpd_info.info.sbu1_pullup_max_volt);
			} else {
				lpd_set_value(buf[i], &chip->lpd_info.info.plugout_sbu1_pullup_volt,
					      &chip->lpd_info.info.sbu1_pullup_min_volt,
					      &chip->lpd_info.info.sbu1_pullup_max_volt);
			}
			break;
		case OPLUS_LPD_SEL_SBU2_PULLUP_MASK:
			if (plugin) {
				if (buf[i] == 0 && chip->lpd_spec.ignore_sbu2_err_data == 0) {
					chip->lpd_spec.ignore_sbu2_err_data = 1;
					break;
				}
				lpd_set_value(buf[i], &chip->lpd_info.info.plugin_sbu2_pullup_volt,
					      &chip->lpd_info.info.sbu2_pullup_min_volt,
					      &chip->lpd_info.info.sbu2_pullup_max_volt);
			} else {
				lpd_set_value(buf[i], &chip->lpd_info.info.plugout_sbu2_pullup_volt,
					      &chip->lpd_info.info.sbu2_pullup_min_volt,
					      &chip->lpd_info.info.sbu2_pullup_max_volt);
			}
			break;
		case OPLUS_LPD_SEL_CC1_MASK:
			lpd_set_value(buf[i], &chip->lpd_info.info.cc1_volt,
				      &chip->lpd_info.info.cc1_min_volt,
				      &chip->lpd_info.info.cc1_max_volt);
			break;
		case OPLUS_LPD_SEL_CC2_MASK:
			lpd_set_value(buf[i], &chip->lpd_info.info.cc1_volt,
				      &chip->lpd_info.info.cc2_min_volt,
				      &chip->lpd_info.info.cc2_max_volt);
			break;
		case OPLUS_LPD_SEL_DP_MASK:
			chip->lpd_info.info.dp_volt = buf[i];
			break;
		case OPLUS_LPD_SEL_DM_MASK:
			chip->lpd_info.info.dm_volt = buf[i];
			break;
		default:
			break;
		}
		chg_info("buf[%d]=%d\n", i, buf[i]);
	}

	if (chip->wired_present && chip->lpd_info.info.sbu1_min_volt != -1) {
		chg_info("lpd_count:%d, sbu1:%d, sbu1_pullup:%d, sbu2:%d, sbu2_pullup:%d, "
			"sbu1_rang[%d,%d], sbu1_pullup_rang[%d,%d], sbu2_rang[%d,%d], sbu2_pullup_rang[%d,%d],"
			"cc1_rang[%d,%d], cc2_rang[%d,%d]\n",
			chip->lpd_spec.lpd_retry_count, chip->lpd_info.info.plugin_sbu1_volt,
			chip->lpd_info.info.plugin_sbu1_pullup_volt,
			chip->lpd_info.info.plugin_sbu2_volt, chip->lpd_info.info.plugin_sbu2_pullup_volt,
			chip->lpd_info.info.sbu1_min_volt, chip->lpd_info.info.sbu1_max_volt,
			chip->lpd_info.info.sbu1_pullup_min_volt,chip->lpd_info.info.sbu1_pullup_max_volt,
			chip->lpd_info.info.sbu2_min_volt, chip->lpd_info.info.sbu2_max_volt,
			chip->lpd_info.info.sbu2_pullup_min_volt, chip->lpd_info.info.sbu2_pullup_max_volt,
			chip->lpd_info.info.cc1_min_volt, chip->lpd_info.info.cc1_max_volt,
			chip->lpd_info.info.cc2_min_volt, chip->lpd_info.info.cc2_max_volt);
	}

	return rc;
}

static int oplus_wired_clear_sbu_info(struct oplus_mms_wired *chip)
{
	if (chip == NULL) {
		chg_err("chip is NULL");
		return -EINVAL;
	}
	chip->lpd_spec.lpd_retry_count = 0;
	chip->lpd_spec.lpd_info_status = OPLUS_LPD_NOT_DETECT;
	chip->lpd_spec.lpd_cc_status = OPLUS_LPD_NOT_DETECT;
	chip->lpd_info.info.sbu1_min_volt = -1;
	chip->lpd_info.info.sbu1_max_volt = -1;
	chip->lpd_info.info.sbu2_min_volt = -1;
	chip->lpd_info.info.sbu2_max_volt = -1;
	chip->lpd_info.info.sbu1_pullup_min_volt = -1;
	chip->lpd_info.info.sbu1_pullup_max_volt = -1;
	chip->lpd_info.info.sbu2_pullup_min_volt = -1;
	chip->lpd_info.info.sbu2_pullup_max_volt = -1;
	chip->lpd_info.info.cc1_min_volt = -1;
	chip->lpd_info.info.cc1_max_volt = -1;
	chip->lpd_info.info.cc2_min_volt = -1;
	chip->lpd_info.info.cc2_max_volt = -1;
	chip->lpd_spec.sbu1_err = 0;
	chip->lpd_spec.sbu2_err = 0;
	chip->lpd_spec.sbu1_diff_err = 0;
	chip->lpd_spec.sbu2_diff_err = 0;
	chip->lpd_spec.ignore_sbu1_err_data = 0;
	chip->lpd_spec.ignore_sbu2_err_data = 0;
	chg_info("clear");
	return 0;
}

#define LPD_DIFF_MV_MAX 350
#define LPD_CC_MV_MAX   2000
static int oplus_wired_check_sbu_info(oplus_lpd_info *info)
{
	struct oplus_mms_wired *chip = g_mms_wired;
	struct oplus_lpd_spec_config *lpd_spec;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -EINVAL;
	}

	lpd_spec = &chip->lpd_spec;
	if (lpd_spec == NULL) {
		chg_err("lpd_spec is NULL");
		return -EINVAL;
	}
	if ((info->info.plugin_sbu1_volt < lpd_spec->sbu_rang.low_thr_mv) ||
	    (info->info.plugin_sbu1_volt > lpd_spec->sbu_rang.high_thr_mv) ||
	    (info->info.plugin_sbu1_pullup_volt < lpd_spec->sbu_pullup_rang.low_thr_mv) ||
	    (info->info.plugin_sbu1_pullup_volt > lpd_spec->sbu_pullup_rang.high_thr_mv)) {
		chg_err("sbu1 error, sbu1_volt:%d, rang[%dmv-%dmv], sbu1_pullup_volt:%d, rang[%dmv-%dmv]",
		       info->info.plugin_sbu1_volt, lpd_spec->sbu_rang.low_thr_mv, lpd_spec->sbu_rang.high_thr_mv,
		       info->info.plugin_sbu1_pullup_volt, lpd_spec->sbu_pullup_rang.low_thr_mv,
		       lpd_spec->sbu_pullup_rang.high_thr_mv);
		lpd_spec->sbu1_err = 1;
	}

	if ((info->info.plugin_sbu2_volt < lpd_spec->sbu_rang.low_thr_mv) ||
	    (info->info.plugin_sbu2_volt > lpd_spec->sbu_rang.high_thr_mv) ||
	    (info->info.plugin_sbu2_pullup_volt < lpd_spec->sbu_pullup_rang.low_thr_mv) ||
	    (info->info.plugin_sbu2_pullup_volt > lpd_spec->sbu_pullup_rang.high_thr_mv)) {
		chg_err("sbu2 error, sbu2_volt:%d, rang[%dmv-%dmv], sbu2_pullup_volt:%d, rang[%dmv-%dmv]",
		       info->info.plugin_sbu2_volt, lpd_spec->sbu_rang.low_thr_mv, lpd_spec->sbu_rang.high_thr_mv,
		       info->info.plugin_sbu2_pullup_volt, lpd_spec->sbu_pullup_rang.low_thr_mv,
		       lpd_spec->sbu_pullup_rang.high_thr_mv);
		lpd_spec->sbu2_err = 1;
	}

	return 0;
}

static int oplus_wired_check_sbu_diff_info(oplus_lpd_info *info)
{
	struct oplus_mms_wired *chip = g_mms_wired;
	struct oplus_lpd_spec_config *lpd_spec;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -EINVAL;
	}

	lpd_spec = &chip->lpd_spec;
	if (lpd_spec == NULL) {
		chg_err("lpd_spec is NULL");
		return -EINVAL;
	}

	if (((info->info.sbu1_max_volt - info->info.sbu1_min_volt) > LPD_DIFF_MV_MAX) ||
	    ((info->info.sbu1_pullup_max_volt - info->info.sbu1_pullup_min_volt) > LPD_DIFF_MV_MAX)) {
		chg_err("sbu1 diff error, sbu1_diff[%d, %d], sbu1_pullup_diff[%d, %d]", info->info.sbu1_min_volt,
			info->info.sbu1_max_volt, info->info.sbu1_pullup_min_volt, info->info.sbu1_pullup_max_volt);
		lpd_spec->sbu1_diff_err = 1;
	}

	if (((info->info.sbu2_max_volt - info->info.sbu2_min_volt) > LPD_DIFF_MV_MAX) ||
	    ((info->info.sbu2_pullup_max_volt - info->info.sbu2_pullup_min_volt) > LPD_DIFF_MV_MAX)) {
		chg_err("sbu2 diff error, sbu2_diff[%d, %d], sbu2_pullup_diff[%d, %d]", info->info.sbu2_min_volt,
			info->info.sbu2_max_volt, info->info.sbu2_pullup_min_volt, info->info.sbu2_pullup_max_volt);
		lpd_spec->sbu2_diff_err = 1;
	}

	if ((info->info.cc1_max_volt > LPD_CC_MV_MAX) &&
	    (info->info.cc2_max_volt > LPD_CC_MV_MAX)) {
		chg_err("CC voltage error, cc1_max_volt:%d, cc2_max_volt:%d",
			info->info.cc1_max_volt,info->info.cc2_max_volt);
			chip->lpd_spec.lpd_cc_status = OPLUS_LPD_ERROR;
	}

	return 0;
}

static int oplus_wired_check_sbu_ovp_info(oplus_lpd_info *info)
{
	struct oplus_mms_wired *chip = g_mms_wired;
	struct oplus_lpd_spec_config *lpd_spec;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -EINVAL;
	}

	lpd_spec = &chip->lpd_spec;
	if (lpd_spec == NULL) {
		chg_err("lpd_spec is NULL");
		return -EINVAL;
	}

	if (info->info.sbu1_max_volt >= lpd_spec->lpd_sbu_ovp_thr_mv) {
		chg_err("sbu1 ovp, sbu1_max_volt:%d, sbu_ovp_thr_mv:%d",
		info->info.sbu1_max_volt, lpd_spec->lpd_sbu_ovp_thr_mv);
		lpd_spec->sbu1_diff_err = 1;
	}

	if (info->info.sbu2_max_volt >= lpd_spec->lpd_sbu_ovp_thr_mv) {
		chg_err("sbu2 ovp, sbu2_max_volt:%d, sbu_ovp_thr_mv:%d",
		info->info.sbu2_max_volt, lpd_spec->lpd_sbu_ovp_thr_mv);
		lpd_spec->sbu2_diff_err = 1;
	}

	return 0;
}

int oplus_wired_get_lpd_info_status(struct oplus_mms *topic)
{
	struct oplus_mms_wired *chip = g_mms_wired;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return OPLUS_LPD_NOT_DETECT;
	}

	if (chip->lpd_spec.support_status == OPLUS_LPD_NOT_SUPPORT)
		return OPLUS_LPD_NOT_DETECT;

	chg_info("lpd_info_status:%d", chip->lpd_spec.lpd_info_status);
	return chip->lpd_spec.lpd_info_status;
}


int oplus_wired_set_pd_config(u32 pdo)
{
	int rc;
	struct oplus_mms_wired *chip = g_mms_wired;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	if (is_voocphy_ic_available(chip)) {
		if ((PD_SRC_PDO_TYPE(pdo) == PD_SRC_PDO_TYPE_FIXED)
		     && ((PD_SRC_PDO_FIXED_VOLTAGE(pdo) * 50) > 5000)) {
			rc = oplus_chg_ic_func(chip->voocphy_ic,
					       OPLUS_IC_FUNC_VOOCPHY_SET_PDQC_CONFIG);
			if (rc < 0) {
				chg_err("can't set pdqc config,rc=%d\n", rc);
				return rc;
			}
		}
	} else {
		chg_info("oplus,voocphy_ic not found\n");
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_BUCK_SET_PD_CONFIG,
			       pdo);
	if (rc < 0)
		chg_err("can't set pdo(=0x%08x), rc=%d\n", pdo, rc);
	chg_info("set pdo(=0x%08x)\n", pdo);

	return rc;
}

#define USB_TEMP_DEF	25
int oplus_wired_get_usb_temp_volt(int *vol_l, int *vol_r)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_GET_USB_TEMP_VOLT,
			       vol_l, vol_r);
	if (rc < 0)
		chg_err("can't get usb temp volt, rc=%d\n", rc);
	chip->usbtemp_volt_l = *vol_l;
	chip->usbtemp_volt_r = *vol_r;
	return rc;
}

int oplus_wired_get_usb_temp(int *temp_l, int *temp_r)
{
	int i;
	struct oplus_mms_wired *chip = g_mms_wired;
	struct oplus_usbtemp_spec_config *spec;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}
	spec = &chip->usbtemp_spec;

	if (chip->adc_vol_temp_info == NULL)
		chip->adc_vol_temp_info = &adc_vol_temp_info_table[0];

	if (temp_l == NULL)
		goto next;

	for (i = chip->adc_vol_temp_info->con_volt_table_size - 1; i >= 0;
	     i--) {
		if (chip->adc_vol_temp_info->con_volt_table[i] >=
		    chip->usbtemp_volt_l)
			break;
		else if (i == 0)
			break;
	}

	*temp_l = chip->adc_vol_temp_info->con_temp_table[i];
	if (spec->usbtemp_ntc_temp_l_compensation)
		*temp_l += spec->usbtemp_ntc_temp_l_compensation;
	if (usbtemp_debug & OPEN_LOG_BIT)
		chg_err("usb_temp_l:%d,\n", *temp_l);

next:
	if (temp_r == NULL)
		return 0;

	for (i = chip->adc_vol_temp_info->con_volt_table_size - 1; i >= 0;
	     i--) {
		if (chip->adc_vol_temp_info->con_volt_table[i] >=
		    chip->usbtemp_volt_r)
			break;
		else if (i == 0)
			break;
	}

	*temp_r = chip->adc_vol_temp_info->con_temp_table[i];
	if (spec->usbtemp_ntc_temp_r_compensation)
		*temp_r += spec->usbtemp_ntc_temp_r_compensation;
	if (usbtemp_debug & OPEN_LOG_BIT)
		chg_err(" usb_temp_r:%d\n", *temp_r);

	return 0;
}

bool oplus_wired_usb_temp_check_is_support(void)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;
	bool support;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return false;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_USB_TEMP_CHECK_IS_SUPPORT,
			       &support);
	if (rc < 0) {
		chg_err("can't get usb temp check support status, rc=%d\n", rc);
		return false;
	}

	return support;
}

int oplus_wired_get_vbat_pwr(void)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;
	int vbat_pwr = 0;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_BUCK_GET_VBAT_PWR,
			       &vbat_pwr);
	if (rc < 0) {
		if (rc != -ENOTSUPP)
			chg_err("can't get vbat pwr, rc=%d\n", rc);
	}

	return vbat_pwr;
}

int oplus_wired_get_usb_btb_temp(void)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;
	int usb_btb_temp = 0;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_BUCK_GET_USB_BTB_TEMP,
			       &usb_btb_temp);
	if (rc < 0) {
		if (rc != -ENOTSUPP)
			chg_err("can't get usb btb temp, rc=%d\n", rc);
	}

	return usb_btb_temp;
}

int oplus_wired_get_batt_btb_temp(void)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;
	int batt_btb_temp = 0;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_BUCK_GET_BATT_BTB_TEMP,
			       &batt_btb_temp);
	if (rc < 0) {
		if (rc != -ENOTSUPP)
			chg_err("can't get batt btb temp, rc=%d\n", rc);
	}

	return batt_btb_temp;
}

enum oplus_chg_typec_port_role_type oplus_wired_get_typec_mode(void)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;
	enum oplus_chg_typec_port_role_type mode;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return TYPEC_PORT_ROLE_TRY_SNK;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_GET_TYPEC_MODE,
			       &mode);
	if (rc < 0) {
		chg_err("can't get typec mode, rc=%d\n", rc);
		return TYPEC_PORT_ROLE_TRY_SNK;
	}

	return mode;
}

int oplus_wired_set_typec_mode(enum oplus_chg_typec_port_role_type mode)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_SET_TYPEC_MODE,
			       mode);
	if (rc < 0)
		chg_err("can't set typec mode to %d, rc=%d\n", mode, rc);
	chg_info("set typec mode to %d, rc=%d\n", mode, rc);

	return rc;
}

int oplus_wired_set_otg_switch_status(bool en)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_SET_OTG_SWITCH_STATUS,
			       en);
	if (rc < 0)
		chg_err("can't %s otg switch, rc=%d\n",
			en ? "enable" : "disable", rc);
	chg_info("%s otg switch\n", en ? "enable" : "disable");

	return rc;
}

bool oplus_wired_get_otg_switch_status(void)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;
	bool en;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return false;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_GET_OTG_SWITCH_STATUS,
			       &en);
	if (rc < 0) {
		chg_err("can't get otg switch status, rc=%d\n", rc);
		return false;
	}

	return en;
}

int oplus_wired_wdt_enable(struct oplus_mms *topic, bool enable)
{
	struct oplus_mms_wired *chip;
	int rc;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -ENODEV;
	}
	chip = oplus_mms_get_drvdata(topic);

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_BUCK_WDT_ENABLE,
			       enable);
	if (rc < 0) {
		chg_err("can't %s wdt, rc=%d\n", enable ? "enable" : "disable",
			rc);
		return rc;
	}

	return 0;
}

int oplus_wired_kick_wdt(struct oplus_mms *topic)
{
	struct oplus_mms_wired *chip;
	int rc;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -ENODEV;
	}
	chip = oplus_mms_get_drvdata(topic);

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_BUCK_KICK_WDT);
	if (rc < 0) {
		chg_err("can't kick wdt, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

int oplus_wired_get_shutdown_soc(struct oplus_mms *topic)
{
	int rc = 0;
	struct oplus_mms_wired *chip;
	int soc;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -ENODEV;
	}
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if ((get_eng_version() == HIGH_TEMP_AGING) ||
	    (get_eng_version() == AGING)) {
		chg_info("HIGH_TEMP_AGING or AGING no support backup soc\n");
		return -ENOTSUPP;
	}
#endif
	chip = oplus_mms_get_drvdata(topic);

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_GET_SHUTDOWN_SOC,
			       &soc);
	if (rc < 0) {
		chg_err("can't get shutdown soc, rc=%d\n", rc);
		return rc;
	}

	return soc;
}

int oplus_wired_backup_soc(struct oplus_mms *topic, int soc)
{
	int rc = 0;
	struct oplus_mms_wired *chip;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -ENODEV;
	}
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if ((get_eng_version() == HIGH_TEMP_AGING) ||
	    (get_eng_version() == AGING)) {
		chg_info("HIGH_TEMP_AGING or AGING no support backup soc\n");
		return -ENOTSUPP;
	}
#endif
	chip = oplus_mms_get_drvdata(topic);

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_BACKUP_SOC, soc);
	if (rc < 0)
		chg_err("can't backup soc, rc=%d\n", rc);

	return rc;
}

int oplus_wired_get_voltage_max(struct oplus_mms *topic)
{
	if (topic == NULL) {
		chg_err("topic is NULL");
		return 5000;
	}

	return voltage_max_table_mv[oplus_wired_get_chg_type()];
}

static bool oplus_usbtemp_check_is_support(struct oplus_mms_wired *chip)
{
	bool support;
	int rc;

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_USB_TEMP_CHECK_IS_SUPPORT,
			       &support);
	if (rc < 0)
		return false;
	return support;
}

static bool oplus_wired_ccdetect_is_support(void)
{
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	int boot_mode = get_boot_mode();

	if (boot_mode == MSM_BOOT_MODE__RF ||
	    boot_mode == MSM_BOOT_MODE__WLAN ||
	    boot_mode == MSM_BOOT_MODE__FACTORY)
		return false;
#endif
	return true;
}

static int oplus_wired_ccdetect_enable(struct oplus_mms_wired *chip, bool en)
{
	int rc;

	if (!oplus_wired_ccdetect_is_support())
		return 0;

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_SET_TYPEC_MODE,
		en ? TYPEC_PORT_ROLE_TRY_SNK : TYPEC_PORT_ROLE_SNK);
	if (rc < 0)
		chg_err("%s ccdetect error, rc=%d\n",
			en ? "enable" : "disable", rc);

	return rc;
}

int oplus_wired_get_otg_online_status(struct oplus_mms *topic)
{
	struct oplus_mms_wired *chip;
	int online;
	int rc = 0;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return 0;
	}
	chip = oplus_mms_get_drvdata(topic);

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_GET_OTG_ONLINE_STATUS, &online);
	if (rc < 0) {
		if (rc != -ENOTSUPP)
			chg_err("can't get otg online status, rc=%d\n", rc);
		return 0;
	}
	/* chip->otg_online = online; */

	return online;
}

int oplus_wired_set_usb_status(struct oplus_mms_wired *chip, unsigned int status)
{
	unsigned int usb_status;
	struct mms_msg *msg;
	int rc;

	usb_status = chip->usb_status | status;

	if (chip->usb_status == usb_status)
		return 0;
	chip->usb_status = usb_status;
	chg_info("usb_status=0x%08x\n", usb_status);
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  WIRED_ITEM_USB_STATUS);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->wired_topic, msg);
	if (rc < 0) {
		chg_err("publish usb status msg error, rc=%d\n", rc);
		kfree(msg);
		return rc;
	}

	return 0;
}

int oplus_wired_clear_usb_status(struct oplus_mms_wired *chip, unsigned int status)
{
	unsigned int usb_status;
	struct mms_msg *msg;
	int rc;

	usb_status = chip->usb_status & (~status);

	if (chip->usb_status == usb_status)
		return 0;
	chip->usb_status = usb_status;
	chg_info("usb_status=0x%08x\n", usb_status);
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  WIRED_ITEM_USB_STATUS);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->wired_topic, msg);
	if (rc < 0) {
		chg_err("publish usb status msg error, rc=%d\n", rc);
		kfree(msg);
		return rc;
	}

	return 0;
}

int oplus_set_dpdm_ovp_disable(bool disable)
{
	int rc = 0;
	struct oplus_mms_wired *chip = g_mms_wired;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return -ENODEV;
	}
	if (chip->buck_ic == NULL) {
		chg_err("buck_ic is NULL");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_BUCK_SET_DPDM_OVP_DISABLE, disable);
	if (rc < 0)
		chg_err("can't set OPLUS_IC_FUNC_BUCK_SET_DPDM_OVP_DISABLE, rc=%d\n", rc);
	return rc;
}

#define USB_20C 20
#define USB_40C 40
#define USB_30C 30
#define USB_50C 50
#define USB_55C 55
#define USB_57C 57
#define USB_100C 100
#define USB_50C_VOLT 467
#define USB_55C_VOLT 400
#define USB_57C_VOLT 376
#define USB_100C_VOLT 100
#define VBUS_VOLT_THRESHOLD 400
#define VBUS_MONITOR_INTERVAL 3000
#define MIN_MONITOR_INTERVAL 50
#define MAX_MONITOR_INTERVAL 50
#define RETRY_CNT_DELAY 5
#define HIGH_TEMP_SHORT_CHECK_TIMEOUT 1000

static void oplus_init_usbtemp_wakelock(struct oplus_mms_wired *chip)
{
	static bool is_awake_init = false;
	if (!is_awake_init) {
		chg_err(" init usbtemp wakelock.\n");
		chip->usbtemp_wakelock = wakeup_source_register(
			NULL, "usbtemp suspend wakelock");
		is_awake_init = true;
	}
	return;
}

static void oplus_set_usbtemp_wakelock(struct oplus_mms_wired *chip, bool value)
{
	static bool pm_flag = false;
	if (value && !pm_flag) {
		__pm_stay_awake(chip->usbtemp_wakelock);
		pm_flag = true;
	} else if (!value && pm_flag) {
		__pm_relax(chip->usbtemp_wakelock);
		pm_flag = false;
	}
}

static void oplus_voocphy_dischg_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_wired *chip =
		container_of(dwork, struct oplus_mms_wired, voocphy_dischg_work);
	bool enable = false;
	int retry_cnt = 0;
	int rc = 0;

	if (!is_voocphy_ic_available(chip)) {
		chg_err("voocphy ic is not found\n");
		return;
	}
	if (!chip->voocphy_ic) {
		chg_err("voocphy ic is NULL\n");
		return;
	}

	spin_lock(&chip->voocphy_dischg_enable_lock);
	enable = chip->voocphy_dischg_enable;
	retry_cnt = chip->voocphy_dischg_retry_cnt;
	spin_unlock(&chip->voocphy_dischg_enable_lock);

	/*wait for voocphy ic online 15s when startup*/
	if (!chip->voocphy_ic->online) {
		if (retry_cnt < VOOCPHY_DISCHARG_RETRY_CNT_MAX) {
			chg_err("wait for voocphy ic online, retry = %d\n", retry_cnt);
			spin_lock(&chip->voocphy_dischg_enable_lock);
			chip->voocphy_dischg_retry_cnt++;
			spin_unlock(&chip->voocphy_dischg_enable_lock);
			schedule_delayed_work(&chip->voocphy_dischg_work, msecs_to_jiffies(VOOCPHY_DISCHARG_RETRY_DELAY));
		} else {
			chg_err("voocphy ic still offline after %d retrys, return\n", VOOCPHY_DISCHARG_RETRY_CNT_MAX);
		}
	} else {
		rc = oplus_chg_ic_func(chip->voocphy_ic,
			OPLUS_IC_FUNC_VOOCPHY_SET_USB_DISCHG_ENABLE, enable);
		if (rc < 0)
			chg_err("failed to %s usb dischg protect, rc=%d\n",
				enable ? "enable" : "disable", rc);
		else
			chg_info("oplus_usbtemp_dischg_action en=%d\n", enable);
	}
}

/* Must be called in sleepable context (contains cancel_delayed_work_sync) */
static void set_usbtemp_discharge_enable(struct oplus_mms_wired *chip, bool enable)
{
	if (chip == NULL || chip->buck_ic == NULL) {
		chg_err("chip is NULL");
		return;
	}

	(void)oplus_chg_ic_func(chip->buck_ic,
			  OPLUS_IC_FUNC_SET_USB_DISCHG_ENABLE, enable);
	if (chip->support_usbtemp_discharge_protect) {
		cancel_delayed_work_sync(&chip->voocphy_dischg_work);
		spin_lock(&chip->voocphy_dischg_enable_lock);
		chip->voocphy_dischg_enable = enable;
		chip->voocphy_dischg_retry_cnt = 0;
		spin_unlock(&chip->voocphy_dischg_enable_lock);
		schedule_delayed_work(&chip->voocphy_dischg_work, 0);
	}
}

static void oplus_usbtemp_recover_func(struct oplus_mms_wired *chip)
{
	int count_time = 0;
	int rc;

	oplus_set_usbtemp_wakelock(chip, false);

	if (chip->usb_status == USB_TEMP_HIGH) {
		oplus_set_usbtemp_wakelock(chip, true);
		do {
			oplus_wired_get_usb_temp_volt(&chip->usbtemp_volt_l,
						      &chip->usbtemp_volt_r);
			oplus_wired_get_usb_temp(&chip->usb_temp_l,
						 &chip->usb_temp_r);
			msleep(2000);
			count_time++;
		} while (!(((chip->usb_temp_r < USB_55C ||
			     chip->usb_temp_r == USB_100C) &&
			    (chip->usb_temp_l < USB_55C ||
			     chip->usb_temp_l == USB_100C)) ||
			   count_time == 30));
		oplus_set_usbtemp_wakelock(chip, false);
		if (count_time == 30) {
			chg_err("[OPLUS_USBTEMP] temp still high");
		} else {
			chip->dischg_flag = false;
			oplus_wired_clear_usb_status(chip, USB_TEMP_HIGH);
			set_usbtemp_discharge_enable(chip, false);
			rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_SET_TYPEC_MODE,
					       TYPEC_PORT_ROLE_ENABLE);
			if (rc < 0)
				chg_err("[OPLUS_USBTEMP] recover cc");
			chg_err("[OPLUS_USBTEMP] usbtemp recover");
			if (is_chg_suspend_votable_available(chip))
				rc = vote(chip->chg_suspend_votable, USB_VOTER,
					  false, 0, false);
			else
				rc = -ENOTSUPP;
			if (rc < 0)
				chg_err("can't set charge unsuspend, rc=%d\n",
					rc);
		}
	}
	return;
}

static void oplus_usbtemp_recover_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_wired *chip = container_of(dwork,
		struct oplus_mms_wired, usbtemp_recover_work);

	oplus_usbtemp_recover_func(chip);
}

static int oplus_find_otg_output_level(struct oplus_mms_wired *chip, int vol)
{
	int i = 0;
	int ret = 0;
	struct oplus_usbtemp_spec_config *spec = &chip->usbtemp_spec;

	for (i = 0; i < OTG_OUTPUT_LEVEL_MAX; i++) {
		if (vol > spec->otg_output[i].otg_vol) {
			ret = spec->otg_output[i].otg_curr;
			break;
		}
	}

	return ret;
}

static void  oplus_usbtemp_restart_work(struct work_struct *work)
{
	struct oplus_mms_wired *chip = container_of(
		work, struct oplus_mms_wired, usbtemp_restart_work);
	bool dischg;
	int rc;

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_GET_USB_DISCHG_STATUS, &dischg);
	if (rc < 0) {
		chg_err("can't get usb dischg status, rc=%d\n", rc);
		goto done;
	}

	switch (chip->usbtemp_timer_stage) {
	case OPLUS_USBTEMP_TIMER_STAGE0:
		chip->usbtemp_timer_stage = OPLUS_USBTEMP_TIMER_STAGE1;
		chg_info("stage0, set try sink\n");
		rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_SET_TYPEC_MODE, TYPEC_PORT_ROLE_TRY_SNK);
		if (rc < 0)
			chg_err("can't set typec mode to try sink, rc=%d\n", rc);
		if (alarmtimer_get_rtcdev()) {
			if (usbtemp_recover_interval > USBTEMP_CC_RECOVER_INTERVAL)
				alarm_start_relative(&chip->usbtemp_alarm_timer, ms_to_ktime(usbtemp_recover_interval- USBTEMP_CC_RECOVER_INTERVAL));
			else
				alarm_start_relative(&chip->usbtemp_alarm_timer, ms_to_ktime(10000));
		} else {
			chg_err("Failed to get soc alarm-timer");
		}
		goto done;
	case OPLUS_USBTEMP_TIMER_STAGE1:
		chip->usbtemp_timer_stage = OPLUS_USBTEMP_TIMER_STAGE0;
		chg_info("stage1, set vbus_short output low\n");
		break;
	default:
		chg_info("stage%d not support\n", chip->usbtemp_timer_stage);
		goto done;
	}

	if (dischg) {
		chip->dischg_flag = false;
		chg_info("dischg disable...\n");
		oplus_wired_clear_usb_status(chip, USB_TEMP_HIGH);
		set_usbtemp_discharge_enable(chip, false);
		chg_info("[OPLUS_USBTEMP] usbtemp recover");
		if (is_chg_suspend_votable_available(chip))
			rc = vote(chip->chg_suspend_votable, USB_VOTER, false, 0, false);
		else
			rc = -ENOTSUPP;
		if (rc < 0)
			chg_err("can't set charge unsuspend, rc=%d\n", rc);
		chg_info("usbtemp recover");
	}

done:
	oplus_set_usbtemp_wakelock(chip, false);
}

static enum alarmtimer_restart oplus_usbtemp_alarm_timer_func(struct alarm *alarm, ktime_t now)
{
	struct oplus_mms_wired *chip = container_of(
		alarm, struct oplus_mms_wired, usbtemp_alarm_timer);

	chg_info("timer is up now.\n");
	oplus_set_usbtemp_wakelock(chip, true);
	schedule_work(&chip->usbtemp_restart_work);

	return ALARMTIMER_NORESTART;
}

static int oplus_usbtemp_dischg_action(struct oplus_mms_wired *chip)
{
	int rc = 0;
	struct oplus_mms *vooc_topic;

	vooc_topic = oplus_mms_get_by_name("vooc");
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if (get_eng_version() != HIGH_TEMP_AGING) {
#endif
		oplus_wired_clear_usb_status(chip, USB_LPD_DETECT);
		oplus_wired_set_usb_status(chip, USB_TEMP_HIGH);
		rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_SET_TYPEC_MODE,
				TYPEC_PORT_ROLE_DISABLE);
		if (rc < 0)
			chg_err("can't open cc, rc=%d\n", rc);
		usleep_range(5000, 5000);
		/*TODO*/
		if(chip->vooc_charging && vooc_topic) {
			oplus_api_switch_normal_chg(vooc_topic);
			oplus_api_vooc_set_reset_sleep(vooc_topic);
		}
		usleep_range(10000, 10000);
		if (is_chg_suspend_votable_available(chip))
			rc = vote(chip->chg_suspend_votable, USB_VOTER,
				  true, 1, false);
		else
			rc = -ENOTSUPP;
		if (rc < 0)
			chg_err("can't set charge suspend, rc=%d\n", rc);
		usleep_range(20000, 20000);
		chg_err("set vbus down");
		if (!chip->high_reverse_charging || chip->wired_online)
			set_usbtemp_discharge_enable(chip, true);
		if (chip->vooc_charging && vooc_topic)
			oplus_api_vooc_turn_off_fastchg(vooc_topic);
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	} else {
		chg_err("CONFIG_HIGH_TEMP_VERSION enable here,do not set vbus down \n");
		set_usbtemp_discharge_enable(chip, false);
	}
#endif
	return 0;
}

static void oplus_usbtemp_set_recover(struct oplus_mms_wired *chip)
{
	if (alarmtimer_get_rtcdev()) {
		chip->usbtemp_timer_stage = OPLUS_USBTEMP_TIMER_STAGE0;
		alarm_start_relative(&chip->usbtemp_alarm_timer, ms_to_ktime(USBTEMP_CC_RECOVER_INTERVAL));
	} else {
		chg_err("Failed to get soc alarm-timer");
	}
}

#define RETRY_COUNT 3
static void oplus_update_usbtemp_current_status(struct oplus_mms_wired *chip)
{
	static int limit_cur_cnt_r = 0;
	static int limit_cur_cnt_l = 0;
	static int recover_cur_cnt_r = 0;
	static int recover_cur_cnt_l = 0;
	int batt_temp = chip->batt_realy_temp;

	if ((chip->usb_temp_l < USB_30C || chip->usb_temp_l > USB_100C) &&
	    (chip->usb_temp_r < USB_30C || chip->usb_temp_r > USB_100C)) {
		chip->smart_charge_user = SMART_CHARGE_USER_OTHER;
		if (is_cool_down_votable_available(chip))
			vote(chip->cool_down_votable, USB_VOTER, false, 0, false);
		limit_cur_cnt_r = 0;
		recover_cur_cnt_r = 0;
		limit_cur_cnt_l = 0;
		recover_cur_cnt_l = 0;
		return;
	}

	if ((chip->usb_temp_r - batt_temp / 10) >= 12) {
		limit_cur_cnt_r++;
		if (limit_cur_cnt_r >= RETRY_COUNT) {
			limit_cur_cnt_r = RETRY_COUNT;
		}
		recover_cur_cnt_r = 0;
	} else if ((chip->usb_temp_r - batt_temp / 10) <= 6) {
		recover_cur_cnt_r++;
		if (recover_cur_cnt_r >= RETRY_COUNT) {
			recover_cur_cnt_r = RETRY_COUNT;
		}
		limit_cur_cnt_r = 0;
	}

	if ((chip->usb_temp_l - batt_temp / 10) >= 12) {
		limit_cur_cnt_l++;
		if (limit_cur_cnt_l >= RETRY_COUNT) {
			limit_cur_cnt_l = RETRY_COUNT;
		}
		recover_cur_cnt_l = 0;
	} else if ((chip->usb_temp_l - batt_temp / 10) <= 6) {
		recover_cur_cnt_l++;
		if (recover_cur_cnt_l >= RETRY_COUNT) {
			recover_cur_cnt_l = RETRY_COUNT;
		}
		limit_cur_cnt_l = 0;
	}

	if ((RETRY_COUNT <= limit_cur_cnt_r ||
	     RETRY_COUNT <= limit_cur_cnt_l) &&
	    (chip->smart_charge_user == SMART_CHARGE_USER_OTHER)) {
		chip->smart_charge_user = SMART_CHARGE_USER_USBTEMP;
		if (is_cool_down_votable_available(chip))
			vote(chip->cool_down_votable, USB_VOTER, true, 3, false);
		limit_cur_cnt_r = 0;
		recover_cur_cnt_r = 0;
		limit_cur_cnt_l = 0;
		recover_cur_cnt_l = 0;
	} else if ((RETRY_COUNT <= recover_cur_cnt_r &&
		    RETRY_COUNT <= recover_cur_cnt_l) &&
		   (chip->smart_charge_user == SMART_CHARGE_USER_USBTEMP)) {
		chip->smart_charge_user = SMART_CHARGE_USER_OTHER;
		if (is_cool_down_votable_available(chip))
			vote(chip->cool_down_votable, USB_VOTER, false, 0, false);
		limit_cur_cnt_r = 0;
		recover_cur_cnt_r = 0;
		limit_cur_cnt_l = 0;
		recover_cur_cnt_l = 0;
	}

	return;
}

#define USBTEMP_TRIGGER_CONDITION_1			1
#define USBTEMP_TRIGGER_CONDITION_2			2
#define USBTEMP_TRIGGER_CONDITION_COOL_DOWN		3
#define USBTEMP_TRIGGER_CONDITION_COOL_DOWN_RECOVERY	4
static int oplus_usbtemp_push_err_msg(struct oplus_mms_wired *chip,
				      int condition, int last_usb_temp_l,
				      int last_usb_temp_r, int batt_curr)
{
	struct mms_msg *msg;
	int batt_temp = chip->batt_realy_temp;
	int rc;

	if (!is_err_topic_available(chip)) {
		chg_err("error topic not found\n");
		return -ENODEV;
	}

	if (condition == USBTEMP_TRIGGER_CONDITION_1) {
		msg = oplus_mms_alloc_str_msg(
			MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_USBTEMP,
			"$$reason@@%s$$batt_temp@@%d"
			"$$usb_temp_l@@%d$$usb_temp_r@@%d",
			"first_condition", batt_temp, chip->usb_temp_l,
			chip->usb_temp_r);
	} else if (condition == USBTEMP_TRIGGER_CONDITION_2) {
		msg = oplus_mms_alloc_str_msg(
			MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_USBTEMP,
			"$$reason@@%s$$batt_temp@@%d"
			"$$usb_temp_l@@%d$$last_usb_temp_l@@%d"
			"$$usb_temp_r@@%d$$last_usb_temp_r@@%d",
			"second_condition", batt_temp, chip->usb_temp_l,
			last_usb_temp_l, chip->usb_temp_r, last_usb_temp_r);
	} else if (condition == USBTEMP_TRIGGER_CONDITION_COOL_DOWN) {
		msg = oplus_mms_alloc_str_msg(
			MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_USBTEMP,
			"$$reason@@%s$$batt_temp@@%d"
			"$$usb_temp_l@@%d$$usb_temp_r@@%d"
			"$$batt_current@@%d",
			"cool_down_condition", batt_temp,
			chip->usb_temp_l, chip->usb_temp_r, batt_curr);
	} else if (condition == USBTEMP_TRIGGER_CONDITION_COOL_DOWN_RECOVERY) {
		msg = oplus_mms_alloc_str_msg(
			MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_USBTEMP,
			"$$reason@@%s$$batt_temp@@%d"
			"$$usb_temp_l@@%d$$usb_temp_r@@%d"
			"$$batt_current@@%d",
			"cool_down_recovery_condition", batt_temp,
			chip->usb_temp_l, chip->usb_temp_r, batt_curr);
	} else {
		chg_err("unknown condition\n");
		return -EINVAL;
	}
	if (msg == NULL) {
		chg_err("alloc usbtemp error msg error\n");
		return -ENOMEM;
	}

	/* track module also needs to collect some other information synchronously */
	rc = oplus_mms_publish_msg_sync(chip->err_topic, msg);
	if (rc < 0) {
		chg_err("publish usbtemp error msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static int oplus_lpd_push_track_msg(struct oplus_mms_wired *chip)
{
	struct mms_msg *msg;
	int rc;

	if (!is_err_topic_available(chip)) {
		chg_err("error topic not found\n");
		return -ENODEV;
	}

	msg = oplus_mms_alloc_str_msg(
		MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_LPD,
		"$$lpd_status@@[%d,%d]$$lpd_info@@[%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]"
		"$$lpd_d_info@@[%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]",
		chip->lpd_spec.lpd_info_status, chip->lpd_spec.lpd_cc_status,
		chip->lpd_info.buf[0], chip->lpd_info.buf[1], chip->lpd_info.buf[2], chip->lpd_info.buf[3],
		chip->lpd_info.buf[4], chip->lpd_info.buf[5], chip->lpd_info.buf[6], chip->lpd_info.buf[7],
		chip->lpd_info.buf[8], chip->lpd_info.buf[9], chip->lpd_info.buf[10], chip->lpd_info.buf[11],
		chip->lpd_info.info.sbu1_min_volt, chip->lpd_info.info.sbu1_max_volt,
		chip->lpd_info.info.sbu1_pullup_min_volt, chip->lpd_info.info.sbu1_pullup_max_volt,
		chip->lpd_info.info.sbu2_min_volt, chip->lpd_info.info.sbu2_max_volt,
		chip->lpd_info.info.sbu2_pullup_min_volt, chip->lpd_info.info.sbu2_pullup_max_volt,
		chip->lpd_info.info.cc1_max_volt, chip->lpd_info.info.cc2_max_volt);

	if (msg == NULL) {
		chg_err("alloc usbtemp error msg error\n");
		return -ENOMEM;
	}

	/* track module also needs to collect some other information synchronously */
	rc = oplus_mms_publish_msg_sync(chip->err_topic, msg);
	if (rc < 0) {
		chg_err("publish usbtemp error msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

#define LPD_RETRY_COUNT		10
#define LPD_RETRY_DELAY		200
static int oplus_wired_check_lpd(struct oplus_mms_wired *chip, int real_chg_type)
{
	static int last_real_chg_type = OPLUS_CHG_USB_TYPE_UNKNOWN;

	if ((last_real_chg_type == OPLUS_CHG_USB_TYPE_UNKNOWN) && (last_real_chg_type != real_chg_type) &&
	     chip->lpd_spec.support_status != OPLUS_LPD_NOT_SUPPORT) {
		oplus_wired_clear_sbu_info(chip);
		if (real_chg_type == OPLUS_CHG_USB_TYPE_DCP || real_chg_type == OPLUS_CHG_USB_TYPE_PD_PPS
		    || real_chg_type == OPLUS_CHG_USB_TYPE_PD) {
			oplus_wired_update_lpd_info(chip, OPLUS_LPD_SEL_CC1_MASK | OPLUS_LPD_SEL_CC2_MASK |
				OPLUS_LPD_SEL_DP_MASK | OPLUS_LPD_SEL_DM_MASK, 1);
			schedule_delayed_work(&chip->lpd_info_update_work, msecs_to_jiffies(LPD_RETRY_DELAY));
		} else {
			oplus_wired_update_lpd_info(chip, OPLUS_LPD_SEL_CC1_MASK | OPLUS_LPD_SEL_CC2_MASK, 1);
			schedule_delayed_work(&chip->lpd_info_update_work, msecs_to_jiffies(LPD_RETRY_DELAY));
		}
	} else if ((last_real_chg_type != real_chg_type) && (real_chg_type == OPLUS_CHG_USB_TYPE_UNKNOWN) &&
		   chip->lpd_spec.support_status != OPLUS_LPD_NOT_SUPPORT) {
		oplus_wired_update_lpd_info(chip, OPLUS_LPD_SEL_SBU1_MASK | OPLUS_LPD_SEL_SBU2_MASK |
			OPLUS_LPD_SEL_SBU1_PULLUP_MASK | OPLUS_LPD_SEL_SBU2_PULLUP_MASK, 0);
		oplus_lpd_push_track_msg(chip);
	}
	last_real_chg_type = real_chg_type;
	return 0;
}

static int oplus_usbtemp_monitor_main(void *data)
{
	int delay = 0;
	int vbus_volt = 0;
	static int count = 0;
	static int total_count = 0;
	static int last_usb_temp_l = 25;
	static int current_temp_l = 25;
	static int last_usb_temp_r = 25;
	static int current_temp_r = 25;
	int retry_cnt = 3, i = 0;
	int count_r = 1, count_l = 1;
	bool condition1 = false;
	bool condition2 = false;
	struct oplus_mms_wired *chip = data;
	static int log_count = 0;
	struct oplus_usbtemp_spec_config *spec = &chip->usbtemp_spec;
	int batt_temp = 0;

	chg_info("[oplus_usbtemp_monitor_main]:run first!");
	spec = &chip->usbtemp_spec;
	while (!kthread_should_stop()) {
		wait_event_interruptible(chip->oplus_usbtemp_wq, (chip->usbtemp_check || chip->reverse_usbtemp_check));
		if (chip->dischg_flag) {
			goto dischg;
		}
		oplus_wired_get_usb_temp_volt(&chip->usbtemp_volt_l, &chip->usbtemp_volt_r);
		oplus_wired_get_usb_temp(&chip->usb_temp_l, &chip->usb_temp_r);
		batt_temp = chip->batt_realy_temp;
		if ((chip->usb_temp_l < USB_50C) && (chip->usb_temp_r < USB_50C)) { /*get vbus when usbtemp < 50C*/
			vbus_volt = oplus_wired_get_vbus();
		} else {
			vbus_volt = 0;
		}
		if ((chip->usb_temp_l < USB_40C) && (chip->usb_temp_r < USB_40C)) {
			delay = MAX_MONITOR_INTERVAL;
			total_count = 10;
		} else {
			delay = MIN_MONITOR_INTERVAL;
			total_count = 30;
		}

		oplus_update_usbtemp_current_status(chip);

		if ((chip->usbtemp_volt_l < USB_50C) && (chip->usbtemp_volt_r < USB_50C) &&
		    (vbus_volt < VBUS_VOLT_THRESHOLD))
			delay = VBUS_MONITOR_INTERVAL;

		/*condition1  :the temp is higher than 57*/
		if (batt_temp / 10 <= USB_50C &&
		    (((chip->usb_temp_l >= USB_57C) && (chip->usb_temp_l < USB_100C)) ||
		     ((chip->usb_temp_r >= USB_57C) && (chip->usb_temp_r < USB_100C)))) {
			chg_err("in loop 1");
			for (i = 1; i < retry_cnt; i++) {
				mdelay(RETRY_CNT_DELAY);
				oplus_wired_get_usb_temp_volt(&chip->usbtemp_volt_l, &chip->usbtemp_volt_r);
				oplus_wired_get_usb_temp(&chip->usb_temp_l, &chip->usb_temp_r);
				if (chip->usb_temp_r >= USB_57C && chip->usb_temp_r < USB_100C)
					count_r++;
				if (chip->usb_temp_l >= USB_57C && chip->usb_temp_l < USB_100C)
					count_l++;
				chg_err("countl : %d", count_l);
			}
			if (count_r >= retry_cnt || count_l >= retry_cnt) {
				if (oplus_usbtemp_check_is_support(chip)) {
					chip->dischg_flag = true;
					condition1 = true;
					chg_err("dischg enable1...[%d, %d]\n", chip->usb_temp_l, chip->usb_temp_r);
				}
			}
			count_r = 1;
			count_l = 1;
			count = 0;
			last_usb_temp_r = chip->usb_temp_r;
			last_usb_temp_l = chip->usb_temp_l;
		}
		if (batt_temp / 10 > USB_50C &&
		    (((chip->usb_temp_l >= batt_temp / 10 + 7) && (chip->usb_temp_l < USB_100C)) ||
		     ((chip->usb_temp_r >= batt_temp / 10 + 7) && (chip->usb_temp_r < USB_100C)))) {
			chg_err("in loop 1");
			for (i = 1; i <= retry_cnt; i++) {
				mdelay(RETRY_CNT_DELAY);
				oplus_wired_get_usb_temp_volt(&chip->usbtemp_volt_l, &chip->usbtemp_volt_r);
				oplus_wired_get_usb_temp(&chip->usb_temp_l, &chip->usb_temp_r);
				if ((chip->usb_temp_r >= batt_temp / 10 + 7) && chip->usb_temp_r < USB_100C)
					count_r++;
				if ((chip->usb_temp_l >= batt_temp / 10 + 7) && chip->usb_temp_l < USB_100C)
					count_l++;
				chg_err("countl : %d", count_l);
			}
			if (count_r >= retry_cnt || count_l >= retry_cnt) {
				if (oplus_usbtemp_check_is_support(chip)) {
					chip->dischg_flag = true;
					condition1 = true;
					chg_err("dischg enable1...[%d, %d]\n", chip->usb_temp_l, chip->usb_temp_r);
				}
			}
			count_r = 1;
			count_l = 1;
			count = 0;
			last_usb_temp_r = chip->usb_temp_r;
			last_usb_temp_l = chip->usb_temp_l;
		}
		if (condition1) {
			chg_err("jump_to_dischg");
			goto dischg;
		}

		/*condition2  :the temp uprising to fast*/
		if ((((chip->usb_temp_l - batt_temp / 10) > spec->usbtemp_batttemp_gap) &&
		     (chip->usb_temp_l < USB_100C)) ||
		    (((chip->usb_temp_r - batt_temp / 10) > spec->usbtemp_batttemp_gap) &&
		     (chip->usb_temp_r < USB_100C))) {
			if (count == 0) {
				last_usb_temp_r = chip->usb_temp_r;
				last_usb_temp_l = chip->usb_temp_l;
			} else {
				current_temp_r = chip->usb_temp_r;
				current_temp_l = chip->usb_temp_l;
			}
			if (((current_temp_l - last_usb_temp_l) >= 3) || (current_temp_r - last_usb_temp_r) >= 3) {
				for (i = 1; i <= retry_cnt; i++) {
					mdelay(RETRY_CNT_DELAY);
					oplus_wired_get_usb_temp_volt(&chip->usbtemp_volt_l, &chip->usbtemp_volt_r);
					oplus_wired_get_usb_temp(&chip->usb_temp_l, &chip->usb_temp_r);
					if ((chip->usb_temp_r - last_usb_temp_r) >= 3 && chip->usb_temp_r < USB_100C)
						count_r++;
					if ((chip->usb_temp_l - last_usb_temp_l) >= 3 && chip->usb_temp_l < USB_100C)
						count_l++;
					chg_err("countl : %d,countr : %d", count_l, count_r);
				}
				current_temp_l = chip->usb_temp_l;
				current_temp_r = chip->usb_temp_r;
				if ((count_l >= retry_cnt && chip->usb_temp_l > USB_30C &&
				     chip->usb_temp_l < USB_100C) ||
				    (count_r >= retry_cnt && chip->usb_temp_r > USB_30C &&
				     chip->usb_temp_r < USB_100C)) {
					if (oplus_usbtemp_check_is_support(chip)) {
						chip->dischg_flag = true;
						chg_err("dischg enable3...,current_temp_l=%d,last_usb_temp_l=%d,current_temp_r=%d,last_usb_temp_r =%d\n",
							current_temp_l, last_usb_temp_l, current_temp_r,
							last_usb_temp_r);
						condition2 = true;
					}
				}
				count_r = 1;
				count_l = 1;
			}
			count++;
			if (count > total_count)
				count = 0;
		} else {
			count = 0;
			last_usb_temp_r = chip->usb_temp_r;
			last_usb_temp_l = chip->usb_temp_l;
		}
	/*judge whether to go the action*/
	dischg:
		if ((chip->usb_temp_l < USB_30C || chip->usb_temp_l > USB_100C) &&
		    (chip->usb_temp_r < USB_30C || chip->usb_temp_r > USB_100C)) {
			condition1 = false;
			condition2 = false;
			chip->dischg_flag = false;
		}

		if (usbtemp_recover_test || ((condition1 || condition2) && chip->dischg_flag)) {
			oplus_usbtemp_push_err_msg(
				chip,
				condition1 ? USBTEMP_TRIGGER_CONDITION_1 :
						   USBTEMP_TRIGGER_CONDITION_2,
				last_usb_temp_l, last_usb_temp_r, chip->batt_curr);
			if (!usbtemp_recover_test) {
				oplus_usbtemp_dischg_action(chip);
			}
			usbtemp_recover_test = 0;
			condition1 = false;
			condition2 = false;
			chg_info("start delay work for recover charging");
			oplus_init_usbtemp_wakelock(chip);
			oplus_set_usbtemp_wakelock(chip, true);
			cancel_delayed_work(&chip->usbtemp_recover_work);
			schedule_delayed_work(&chip->usbtemp_recover_work, msecs_to_jiffies(usbtemp_recover_interval));
		}
		msleep(delay);
		log_count++;
		/* about 1 minute */
		if (log_count == 960) {
			chg_info(
				"usbtemp_volt_l[%d], usb_temp_l[%d], usbtemp_volt_r[%d], usb_temp_r[%d]\n",
				chip->usbtemp_volt_l, chip->usb_temp_l,
				chip->usbtemp_volt_r, chip->usb_temp_r);
			log_count = 0;
		}
	}

	return 0;
}

static bool oplus_usbtemp_l_trigger_current_status(struct oplus_mms_wired *chip)
{
	struct oplus_usbtemp_spec_config *spec;
	int batt_temp;

	if (!chip)
		return false;
	spec = &chip->usbtemp_spec;
	batt_temp = chip->batt_realy_temp;

	if (chip->usb_temp_l < USB_30C || chip->usb_temp_l > USB_100C)
		return false;

	if (chip->usbtemp_curr_status == OPLUS_USBTEMP_LOW_CURR) {
		if ((chip->usb_temp_l >= spec->usbtemp_cool_down_ntc_low) ||
		    (chip->usb_temp_l - batt_temp / 10) >= spec->usbtemp_cool_down_gap_low)
			return true;
		return false;
	} else if (chip->usbtemp_curr_status == OPLUS_USBTEMP_HIGH_CURR) {
		if ((chip->usb_temp_l >= spec->usbtemp_cool_down_ntc_high) ||
		    (chip->usb_temp_l - batt_temp / 10) >= spec->usbtemp_cool_down_gap_high)
			return true;
		return false;
	} else {
		return false;
	}
}

static bool oplus_usbtemp_l_recovery_current_status(struct oplus_mms_wired *chip)
{
	struct oplus_usbtemp_spec_config *spec;
	int batt_temp;

	if (!chip)
		return false;
	spec = &chip->usbtemp_spec;
	batt_temp = chip->batt_realy_temp;

	if (chip->usbtemp_curr_status == OPLUS_USBTEMP_LOW_CURR) {
		if ((chip->usb_temp_l <= spec->usbtemp_cool_down_recover_ntc_low) &&
		    (chip->usb_temp_l - batt_temp / 10) <= spec->usbtemp_cool_down_recover_gap_low)
			return true;
		return false;
	} else if (chip->usbtemp_curr_status == OPLUS_USBTEMP_HIGH_CURR) {
		if ((chip->usb_temp_l <= spec->usbtemp_cool_down_recover_ntc_high) &&
		    (chip->usb_temp_l - batt_temp / 10) <= spec->usbtemp_cool_down_recover_gap_high)
			return true;
		return false;
	} else {
		return false;
	}
}

static bool oplus_usbtemp_r_trigger_current_status(struct oplus_mms_wired *chip)
{
	struct oplus_usbtemp_spec_config *spec;
	int batt_temp;

	if (!chip)
		return false;
	spec = &chip->usbtemp_spec;
	batt_temp = chip->batt_realy_temp;

	if (chip->usb_temp_r < USB_30C || chip->usb_temp_r > USB_100C)
		return false;

	if (chip->usbtemp_curr_status == OPLUS_USBTEMP_LOW_CURR) {
		if ((chip->usb_temp_r >= spec->usbtemp_cool_down_ntc_low) ||
		    (chip->usb_temp_r - batt_temp / 10) >= spec->usbtemp_cool_down_gap_low)
			return true;
		return false;
	} else if (chip->usbtemp_curr_status == OPLUS_USBTEMP_HIGH_CURR) {
		if ((chip->usb_temp_r >= spec->usbtemp_cool_down_ntc_high) ||
		    (chip->usb_temp_r - batt_temp / 10) >= spec->usbtemp_cool_down_gap_high)
			return true;
		return false;
	} else {
		return false;
	}
}

static bool oplus_usbtemp_r_recovery_current_status(struct oplus_mms_wired *chip)
{
	struct oplus_usbtemp_spec_config *spec;
	int batt_temp;

	if (!chip)
		return false;
	spec = &chip->usbtemp_spec;
	batt_temp = chip->batt_realy_temp;

	if (chip->usbtemp_curr_status == OPLUS_USBTEMP_LOW_CURR) {
		if ((chip->usb_temp_r <= spec->usbtemp_cool_down_recover_ntc_low) &&
		    (chip->usb_temp_r - batt_temp / 10) <= spec->usbtemp_cool_down_recover_gap_low)
			return true;
		return false;
	} else if (chip->usbtemp_curr_status == OPLUS_USBTEMP_HIGH_CURR) {
		if ((chip->usb_temp_r <= spec->usbtemp_cool_down_recover_ntc_high) &&
		    (chip->usb_temp_r - batt_temp / 10) <= spec->usbtemp_cool_down_recover_gap_high)
			return true;
		return false;
	} else {
		return false;
	}
}

#define RETRY_COUNT 3
static void oplus_update_usbtemp_current_status_new_method(struct oplus_mms_wired *chip)
{
	struct oplus_usbtemp_spec_config *spec = &chip->usbtemp_spec;
	static int limit_cur_cnt_r = 0;
	static int limit_cur_cnt_l = 0;
	static int recover_cur_cnt_r = 0;
	static int recover_cur_cnt_l = 0;
	int condition, batt_current;
	int last_usb_temp_l = 25;
	int last_usb_temp_r = 25;
	int cool_down;

	batt_current = chip->usbtemp_batt_current;
	if (oplus_vooc_get_project(chip->vooc_topic) == 0)
		return;

	if ((chip->usb_temp_l < USB_30C || chip->usb_temp_l > USB_100C) &&
	    (chip->usb_temp_r < USB_30C || chip->usb_temp_r > USB_100C)) {
		chip->smart_charge_user = SMART_CHARGE_USER_OTHER;
		if (is_cool_down_votable_available(chip))
			vote(chip->cool_down_votable, USB_VOTER, false, 0, false);
		limit_cur_cnt_r = 0;
		recover_cur_cnt_r = 0;
		limit_cur_cnt_l = 0;
		recover_cur_cnt_l = 0;
		return;
	}

	if (chip->new_usbtemp_cool_down_support) {
		if (batt_current > 5000) {
			if (spec->usbtemp_cool_down_temp_gap == spec->usbtemp_cool_down_temp_first_gap) {
				limit_cur_cnt_r = 0;
				recover_cur_cnt_r = 0;
				limit_cur_cnt_l = 0;
				recover_cur_cnt_l = 0;
				chg_info("usbtemp_cool_down_temp_gap to %d", spec->usbtemp_cool_down_temp_second_gap);
			}
			spec->usbtemp_cool_down_temp_gap = spec->usbtemp_cool_down_temp_second_gap;
			spec->usbtemp_cool_down_recovery_temp_gap = spec->usbtemp_cool_down_recovery_temp_second_gap;
		} else {
			if (spec->usbtemp_cool_down_temp_gap == spec->usbtemp_cool_down_temp_second_gap) {
				limit_cur_cnt_r = 0;
				recover_cur_cnt_r = 0;
				limit_cur_cnt_l = 0;
				recover_cur_cnt_l = 0;
				chg_info("usbtemp_cool_down_temp_gap to %d", spec->usbtemp_cool_down_temp_first_gap);
			}
			spec->usbtemp_cool_down_temp_gap = spec->usbtemp_cool_down_temp_first_gap;
			spec->usbtemp_cool_down_recovery_temp_gap = spec->usbtemp_cool_down_recovery_temp_first_gap;
		}
	}

	if (oplus_usbtemp_r_trigger_current_status(chip)) {
		limit_cur_cnt_r++;
		recover_cur_cnt_r = 0;
	} else if (oplus_usbtemp_r_recovery_current_status(chip)) {
		recover_cur_cnt_r++;
		limit_cur_cnt_r = 0;
	}

	if (oplus_usbtemp_l_trigger_current_status(chip)) {
		limit_cur_cnt_l++;
		recover_cur_cnt_l = 0;
	} else if (oplus_usbtemp_l_recovery_current_status(chip)) {
		recover_cur_cnt_l++;
		limit_cur_cnt_l = 0;
	}

	if ((RETRY_COUNT <= limit_cur_cnt_r || RETRY_COUNT <= limit_cur_cnt_l) &&
	    (chip->smart_charge_user == SMART_CHARGE_USER_OTHER)) {
		chip->smart_charge_user = SMART_CHARGE_USER_USBTEMP;
		if (chip->ufcs_online)
			cool_down = oplus_ufcs_current_to_level(chip->ufcs_topic, USBTEMP_CHARGING_CURRENT_LIMIT);
		else
			cool_down = oplus_vooc_current_to_level(chip->vooc_topic, USBTEMP_CHARGING_CURRENT_LIMIT);
		if (is_cool_down_votable_available(chip) && cool_down > 0)
			vote(chip->cool_down_votable, USB_VOTER, true, cool_down, false);
		limit_cur_cnt_r = 0;
		recover_cur_cnt_r = 0;
		limit_cur_cnt_l = 0;
		recover_cur_cnt_l = 0;
		condition = USBTEMP_TRIGGER_CONDITION_COOL_DOWN;
		oplus_usbtemp_push_err_msg(chip, condition, last_usb_temp_l, last_usb_temp_r, batt_current);
	} else if ((RETRY_COUNT <= recover_cur_cnt_r && RETRY_COUNT <= recover_cur_cnt_l) &&
		   (chip->smart_charge_user == SMART_CHARGE_USER_USBTEMP)) {
		chip->smart_charge_user = SMART_CHARGE_USER_OTHER;
		if (is_cool_down_votable_available(chip))
			vote(chip->cool_down_votable, USB_VOTER, false, 0, false);
		limit_cur_cnt_r = 0;
		recover_cur_cnt_r = 0;
		limit_cur_cnt_l = 0;
		recover_cur_cnt_l = 0;
		condition = USBTEMP_TRIGGER_CONDITION_COOL_DOWN_RECOVERY;
		oplus_usbtemp_push_err_msg(chip, condition, last_usb_temp_l, last_usb_temp_r, batt_current);
	}

	return;
}

static bool oplus_usbtemp_condition_temp_high(struct oplus_mms_wired *chip)
{
	struct oplus_usbtemp_spec_config *spec;
	int batt_temp;

	if (!chip)
		return false;
	spec = &chip->usbtemp_spec;
	batt_temp = chip->batt_realy_temp;

	if (chip->usbtemp_curr_status == OPLUS_USBTEMP_LOW_CURR) {
		if (batt_temp / 10 <= spec->usbtemp_batt_temp_low &&
		    (((chip->usb_temp_l >= spec->usbtemp_ntc_temp_low) && (chip->usb_temp_l < USB_100C)) ||
		     ((chip->usb_temp_r >= spec->usbtemp_ntc_temp_low) && (chip->usb_temp_r < USB_100C))))
			return true;
		return false;
	} else if (chip->usbtemp_curr_status == OPLUS_USBTEMP_HIGH_CURR) {
		if (batt_temp / 10 <= spec->usbtemp_batt_temp_high &&
		    (((chip->usb_temp_l >= spec->usbtemp_ntc_temp_high) && (chip->usb_temp_l < USB_100C)) ||
		     ((chip->usb_temp_r >= spec->usbtemp_ntc_temp_high) && (chip->usb_temp_r < USB_100C))))
			return true;
		return false;
	} else {
		return false;
	}
}

static bool oplus_usbtemp_temp_rise_fast_with_batt_temp(struct oplus_mms_wired *chip)
{
	struct oplus_usbtemp_spec_config *spec;
	int batt_temp;

	if (!chip)
		return false;
	spec = &chip->usbtemp_spec;
	batt_temp = chip->batt_realy_temp;

	if (chip->usbtemp_curr_status == OPLUS_USBTEMP_LOW_CURR) {
		if (oplus_is_power_off_charging() && (spec->support_hot_enter_kpoc || chip->high_temp_scheme)) {
			if (((batt_temp / 10 > spec->usbtemp_batt_temp_low) && (batt_temp / 10 <= spec->usbtemp_batt_temp_over_hot)) &&
				(((chip->usb_temp_l >= batt_temp / 10 + spec->usbtemp_temp_gap_low_with_batt_temp) &&
				  (chip->usb_temp_l < USB_100C)) ||
				 ((chip->usb_temp_r >= batt_temp / 10 + spec->usbtemp_temp_gap_low_with_batt_temp) &&
				  (chip->usb_temp_r < USB_100C))))
				return true;
			else if(batt_temp / 10 > spec->usbtemp_batt_temp_over_hot &&
				(((chip->usb_temp_l >= batt_temp / 10 + spec->usbtemp_temp_gap_with_batt_temp_in_over_hot) &&
				  (chip->usb_temp_l < USB_100C)) ||
				 ((chip->usb_temp_r >= batt_temp / 10 + spec->usbtemp_temp_gap_with_batt_temp_in_over_hot) &&
				  (chip->usb_temp_r < USB_100C))))
				return true;
			return false;
		} else {
			if (batt_temp / 10 > spec->usbtemp_batt_temp_low &&
				(((chip->usb_temp_l >= batt_temp / 10 + spec->usbtemp_temp_gap_low_with_batt_temp) &&
				  (chip->usb_temp_l < USB_100C)) ||
				 ((chip->usb_temp_r >= batt_temp / 10 + spec->usbtemp_temp_gap_low_with_batt_temp) &&
				  (chip->usb_temp_r < USB_100C))))
				return true;
			return false;
		}
	} else if (chip->usbtemp_curr_status == OPLUS_USBTEMP_HIGH_CURR) {
		if (batt_temp / 10 > spec->usbtemp_batt_temp_high &&
			(((chip->usb_temp_l >= batt_temp / 10 + spec->usbtemp_temp_gap_high_with_batt_temp) &&
			  (chip->usb_temp_l < USB_100C)) ||
			 ((chip->usb_temp_r >= batt_temp / 10 + spec->usbtemp_temp_gap_high_with_batt_temp) &&
			  (chip->usb_temp_r < USB_100C))))
			return true;
		return false;
	} else {
		return false;
	}
}

static bool oplus_usbtemp_without_batt_temp_low_curr(struct oplus_mms_wired *chip)
{
	struct oplus_usbtemp_spec_config *spec;
	int diff_batt_usbtemp_r = 0;
	int diff_batt_usbtemp_l = 0;
	bool usbtemp_without_batt_temp_r = false;
	bool usbtemp_without_batt_temp_l = false;

	spec = &chip->usbtemp_spec;
	diff_batt_usbtemp_r = chip->usb_temp_r - chip->batt_realy_temp / 10;
	diff_batt_usbtemp_l = chip->usb_temp_l - chip->batt_realy_temp / 10;

	usbtemp_without_batt_temp_r =
		(diff_batt_usbtemp_r >= spec->usbtemp_temp_gap_low_without_batt_temp) &&
		     (chip->usb_temp_r < USB_100C);

	usbtemp_without_batt_temp_l =
		(diff_batt_usbtemp_l >= spec->usbtemp_temp_gap_low_without_batt_temp) &&
		     (chip->usb_temp_l < USB_100C);

	if (usbtemp_without_batt_temp_l || usbtemp_without_batt_temp_r)
		return true;

	return false;
}

static bool oplus_usbtemp_temp_rise_fast_without_batt_temp(struct oplus_mms_wired *chip)
{
	struct oplus_usbtemp_spec_config *spec;
	int batt_temp;

	if (!chip)
		return false;
	batt_temp = chip->batt_realy_temp;
	spec = &chip->usbtemp_spec;

	if (chip->usbtemp_curr_status == OPLUS_USBTEMP_LOW_CURR) {
		if (oplus_usbtemp_without_batt_temp_low_curr(chip) && !chip->gauge_not_ready)
			return true;
		return false;
	} else if (chip->usbtemp_curr_status == OPLUS_USBTEMP_HIGH_CURR) {
		if ((((chip->usb_temp_l - batt_temp / 10) >= spec->usbtemp_temp_gap_high_without_batt_temp) &&
		     (chip->usb_temp_l < USB_100C)) ||
		    (((chip->usb_temp_r - batt_temp / 10) >= spec->usbtemp_temp_gap_high_without_batt_temp) &&
		     (chip->usb_temp_r < USB_100C)))
			return true;
		return false;
	} else {
		return false;
	}
}

static bool oplus_usbtemp_judge_temp_gap(
	struct oplus_mms_wired *chip, int current_temp, int last_temp)
{
	struct oplus_usbtemp_spec_config *spec;

	if (!chip)
		return false;
	spec = &chip->usbtemp_spec;

	if (chip->usbtemp_curr_status == OPLUS_USBTEMP_LOW_CURR) {
		if ((current_temp - last_temp) >= spec->usbtemp_rise_fast_temp_low)
			return true;
		return false;
	} else if (chip->usbtemp_curr_status == OPLUS_USBTEMP_HIGH_CURR) {
		if ((current_temp - last_temp) >= spec->usbtemp_rise_fast_temp_high)
			return true;
		return false;
	} else {
		return false;
	}
}

static bool oplus_usbtemp_change_curr_range(
	struct oplus_mms_wired *chip, int retry_cnt,
	int usbtemp_first_time_in_curr_range, bool curr_range_change)
{
	static int last_curr_change_usb_temp_l = 25;
	static int current_curr_change_temp_l = 25;
	static int last_curr_change_usb_temp_r = 25;
	static int current_curr_change_temp_r = 25;
	int count_curr_r = 1, count_curr_l = 1;
	int i = 0;
	int batt_temp;

	if (!chip)
		return false;
	batt_temp = chip->batt_realy_temp;

	chip->usbtemp_curr_status = OPLUS_USBTEMP_HIGH_CURR;
	if (usbtemp_first_time_in_curr_range == false) {
		last_curr_change_usb_temp_r = chip->usb_temp_r;
		last_curr_change_usb_temp_l = chip->usb_temp_l;
	} else {
		current_curr_change_temp_r = chip->usb_temp_r;
		current_curr_change_temp_l = chip->usb_temp_l;
	}
	if (((current_curr_change_temp_l - last_curr_change_usb_temp_l) >= OPLUS_USBTEMP_CURR_CHANGE_TEMP) ||
	    (current_curr_change_temp_r - last_curr_change_usb_temp_r) >= OPLUS_USBTEMP_CURR_CHANGE_TEMP) {
		for (i = 1; i <= retry_cnt; i++) {
			mdelay(RETRY_CNT_DELAY);
			oplus_wired_get_usb_temp_volt(&chip->usbtemp_volt_l, &chip->usbtemp_volt_r);
			oplus_wired_get_usb_temp(&chip->usb_temp_l, &chip->usb_temp_r);
			if ((chip->usb_temp_r - last_curr_change_usb_temp_r) >= OPLUS_USBTEMP_CURR_CHANGE_TEMP &&
			    chip->usb_temp_r < USB_100C)
				count_curr_r++;
			if ((chip->usb_temp_l - last_curr_change_usb_temp_l) >= OPLUS_USBTEMP_CURR_CHANGE_TEMP &&
			    chip->usb_temp_l < USB_100C)
				count_curr_l++;
			chg_info("countl: %d, countr: %d", count_curr_l, count_curr_r);
		}
		current_curr_change_temp_l = chip->usb_temp_l;
		current_curr_change_temp_r = chip->usb_temp_r;

		if ((count_curr_l >= retry_cnt && chip->usb_temp_l > USB_30C && chip->usb_temp_l < USB_100C) ||
		    (count_curr_r >= retry_cnt && chip->usb_temp_r > USB_30C && chip->usb_temp_r < USB_100C)) {
			chg_info("change curr range...,current_temp_l=%d,last_usb_temp_l=%d,current_temp_r=%d,last_usb_temp_r =%d, batt_temp = %d\n",
				current_curr_change_temp_l, last_curr_change_usb_temp_l, current_curr_change_temp_r,
				last_curr_change_usb_temp_r, batt_temp);
			count_curr_r = 1;
			count_curr_l = 1;
			return true;
		}
	}
	return false;
}

static bool oplus_usbtemp_trigger_for_high_temp(
	struct oplus_mms_wired *chip, int retry_cnt, int count_r, int count_l)
{
	struct oplus_usbtemp_spec_config *spec;
	int i = 0;

	if (!chip)
		return false;
	spec = &chip->usbtemp_spec;

	if (oplus_usbtemp_condition_temp_high(chip)) {
		chg_info("in usbtemp higher than 57 or 69!\n");
		for (i = 1; i < retry_cnt; i++) {
			mdelay(RETRY_CNT_DELAY);
			oplus_wired_get_usb_temp_volt(&chip->usbtemp_volt_l, &chip->usbtemp_volt_r);
			oplus_wired_get_usb_temp(&chip->usb_temp_l, &chip->usb_temp_r);
			if (chip->usbtemp_curr_status == OPLUS_USBTEMP_LOW_CURR) {
				if (chip->usb_temp_r >= spec->usbtemp_ntc_temp_low && chip->usb_temp_r < USB_100C)
					count_r++;
				if (chip->usb_temp_l >= spec->usbtemp_ntc_temp_low && chip->usb_temp_l < USB_100C)
					count_l++;
			} else if (chip->usbtemp_curr_status == OPLUS_USBTEMP_HIGH_CURR) {
				if (chip->usb_temp_r >= spec->usbtemp_ntc_temp_high && chip->usb_temp_r < USB_100C)
					count_r++;
				if (chip->usb_temp_l >= spec->usbtemp_ntc_temp_high && chip->usb_temp_l < USB_100C)
					count_l++;
			}
			chg_info("countl: %d countr: %d", count_l, count_r);
		}
	}
	if (count_r >= retry_cnt || count_l >= retry_cnt)
		return true;

	return false;
}

static bool oplus_usbtemp_trigger_for_rise_fast_temp(
	struct oplus_mms_wired *chip, int retry_cnt, int count_r, int count_l)
{
	struct oplus_usbtemp_spec_config *spec;
	int batt_temp;
	int i = 0;

	if (!chip)
		return false;
	spec = &chip->usbtemp_spec;
	batt_temp = chip->batt_realy_temp;

	if (oplus_usbtemp_temp_rise_fast_with_batt_temp(chip)) {
		chg_info("in usbtemp rise fast with usbtemp!\n");
		for (i = 1; i <= retry_cnt; i++) {
			mdelay(RETRY_CNT_DELAY);
			oplus_wired_get_usb_temp_volt(&chip->usbtemp_volt_l, &chip->usbtemp_volt_r);
			oplus_wired_get_usb_temp(&chip->usb_temp_l, &chip->usb_temp_r);
			if (chip->usbtemp_curr_status == OPLUS_USBTEMP_LOW_CURR) {
				if ((chip->usb_temp_r >=
				     batt_temp / 10 + spec->usbtemp_temp_gap_low_with_batt_temp) &&
				    chip->usb_temp_r < USB_100C)
					count_r++;
				if ((chip->usb_temp_l >=
				     batt_temp / 10 + spec->usbtemp_temp_gap_low_with_batt_temp) &&
				    chip->usb_temp_l < USB_100C)
					count_l++;
			} else if (chip->usbtemp_curr_status == OPLUS_USBTEMP_HIGH_CURR) {
				if ((chip->usb_temp_r >=
				     batt_temp / 10 + spec->usbtemp_temp_gap_high_with_batt_temp) &&
				    chip->usb_temp_r < USB_100C)
					count_r++;
				if ((chip->usb_temp_l >=
				     batt_temp / 10 + spec->usbtemp_temp_gap_high_with_batt_temp) &&
				    chip->usb_temp_l < USB_100C)
					count_l++;
			}
			chg_info("countl: %d countr: %d", count_l, count_r);
		}
		if (count_r >= retry_cnt || count_l >= retry_cnt)
			return true;
	}

	return false;
}

static bool oplus_usbtemp_trigger_for_rise_fast_without_temp(
	struct oplus_mms_wired *chip, int retry_cnt, int count_r, int count_l, int total_count)
{
	static int count = 0;
	static int last_usb_temp_l = 25;
	static int current_temp_l = 25;
	static int last_usb_temp_r = 25;
	static int current_temp_r = 25;
	int i = 0;

	if (!chip)
		return false;

	if (oplus_usbtemp_temp_rise_fast_without_batt_temp(chip)) {
		if (count == 0) {
			last_usb_temp_r = chip->usb_temp_r;
			last_usb_temp_l = chip->usb_temp_l;
			current_temp_r = chip->usb_temp_r;
			current_temp_l = chip->usb_temp_l;
		} else {
			current_temp_r = chip->usb_temp_r;
			current_temp_l = chip->usb_temp_l;
		}
		if (oplus_usbtemp_judge_temp_gap(chip, current_temp_l, last_usb_temp_l) ||
		    oplus_usbtemp_judge_temp_gap(chip, current_temp_r, last_usb_temp_r)) {
			for (i = 1; i <= retry_cnt; i++) {
				mdelay(RETRY_CNT_DELAY);
				oplus_wired_get_usb_temp_volt(&chip->usbtemp_volt_l, &chip->usbtemp_volt_r);
				oplus_wired_get_usb_temp(&chip->usb_temp_l, &chip->usb_temp_r);
				current_temp_l = chip->usb_temp_l;
				current_temp_r = chip->usb_temp_r;
				if (oplus_usbtemp_judge_temp_gap(chip, current_temp_r, last_usb_temp_r) &&
				    chip->usb_temp_r < USB_100C)
					count_r++;
				if (oplus_usbtemp_judge_temp_gap(chip, current_temp_l, last_usb_temp_l) &&
				    chip->usb_temp_l < USB_100C)
					count_l++;
				chg_info("countl : %d,countr : %d", count_l, count_r);
			}
			current_temp_l = chip->usb_temp_l;
			current_temp_r = chip->usb_temp_r;
			if ((count_l >= retry_cnt && chip->usb_temp_l > USB_30C && chip->usb_temp_l < USB_100C) ||
			    (count_r >= retry_cnt && chip->usb_temp_r > USB_30C && chip->usb_temp_r < USB_100C)) {
				count = 0;
				return true;
			}
			count_r = 1;
			count_l = 1;
		}
		count++;
		if (count > total_count)
			count = 0;
	} else {
		count = 0;
		last_usb_temp_r = chip->usb_temp_r;
		last_usb_temp_l = chip->usb_temp_l;
	}

	return false;
}

#define OPCHG_LOW_USBTEMP_RETRY_COUNT 10
#define OPLUS_CHG_CURRENT_READ_COUNT 15
static bool oplus_gauge_not_ready(struct oplus_mms_wired *chip)
{
	return (chip->gauge_topic == NULL) && !chip->gauge_not_ready;
}

static bool oplus_gauge_just_ready(struct oplus_mms_wired *chip)
{
	return (chip->gauge_topic != NULL) && chip->gauge_not_ready;
}

int oplus_usbtemp_monitor_common_new_method(void *data)
{
	int delay = 0;
	int vbus_volt = 0;
	static int count = 0;
	static int total_count = 0;
	static int last_usb_temp_l = 25;
	static int last_usb_temp_r = 25;
	int retry_cnt = 3;
	int count_r = 1, count_l = 1;
	bool condition1 = false;
	bool condition2 = false;
	union mms_msg_data real_temp_data = { 0 };
	int rc = 0;
	int condition;
	static bool curr_range_change = false;
	int batt_current = 0;
	struct timespec curr_range_change_first_time = (struct timespec){ 0 };
	struct timespec curr_range_change_last_time = (struct timespec){ 0 };
	bool usbtemp_first_time_in_curr_range = false;
	static int current_read_count = 0;
	struct oplus_mms_wired *chip = (struct oplus_mms_wired *)data;
	struct oplus_usbtemp_spec_config *spec = &chip->usbtemp_spec;;

	if (alarmtimer_get_rtcdev()) {
		alarm_init(&chip->usbtemp_alarm_timer, ALARM_REALTIME, oplus_usbtemp_alarm_timer_func);
		INIT_WORK(&chip->usbtemp_restart_work, oplus_usbtemp_restart_work);
		chip->usbtemp_timer_stage = OPLUS_USBTEMP_TIMER_STAGE0;
	} else {
		chg_err("Failed to get soc alarm-timer");
	}

	if (spec->usbtemp_max_temp_thr <= 0)
		spec->usbtemp_max_temp_thr = USB_57C;
	if (spec->usbtemp_temp_up_time_thr <= 0)
		spec->usbtemp_temp_up_time_thr = 30;
	chg_info("run first, usbtemp_max_temp_thr[%d], usbtemp_temp_up_time_thr[%d]!",
		 spec->usbtemp_max_temp_thr, spec->usbtemp_temp_up_time_thr);

	while (!kthread_should_stop()) {
		wait_event_interruptible(chip->oplus_usbtemp_wq, (chip->usbtemp_check || chip->reverse_usbtemp_check));
		if (oplus_gauge_not_ready(chip)) {
			chip->batt_realy_temp = DEFAULT_BATT_REAL_TEMP;
			chip->gauge_not_ready = true;
		}
		if (oplus_gauge_just_ready(chip)) {
			rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_REAL_TEMP, &real_temp_data, false);
			if (rc < 0) {
				chg_err("can't get GAUGE_ITEM_REAL_TEMP data in  usbtemp_monitor new, rc=%d\n", rc);
				delay = 100;
				goto dischg;
			} else {
		 		chip->batt_realy_temp = real_temp_data.intval;
				chg_info("usbtemp_volt_l[%d], usb_temp_l[%d], usbtemp_volt_r[%d], usb_temp_r[%d] temp[%d]%d\n",
					chip->usbtemp_volt_l, chip->usb_temp_l, chip->usbtemp_volt_r, chip->usb_temp_r,
					chip->batt_realy_temp, spec->support_hot_enter_kpoc);
			}
			chip->gauge_not_ready = false;
		}
		if (chip->dischg_flag)
			goto dischg;
		oplus_wired_get_usb_temp_volt(&chip->usbtemp_volt_l, &chip->usbtemp_volt_r);
		oplus_wired_get_usb_temp(&chip->usb_temp_l, &chip->usb_temp_r);

		if ((chip->usb_temp_l < USB_50C) && (chip->usb_temp_r < USB_50C)) /*get vbus when usbtemp < 50C*/
			vbus_volt = oplus_wired_get_vbus();
		else
			vbus_volt = 0;
		if ((chip->usb_temp_l < USB_40C) && (chip->usb_temp_r < USB_40C)) {
			delay = MAX_MONITOR_INTERVAL;
			total_count = 10;
		} else {
			delay = MIN_MONITOR_INTERVAL;
			total_count = spec->usbtemp_temp_up_time_thr;
		}

		current_read_count = current_read_count + 1;
		if (current_read_count == OPLUS_CHG_CURRENT_READ_COUNT) {
			/* TODO: parallel charge */
			chip->usbtemp_batt_current = -oplus_gauge_get_batt_current();
			current_read_count = 0;
		}
		oplus_update_usbtemp_current_status_new_method(chip);
		batt_current = chip->usbtemp_batt_current;

		if (usbtemp_dbg_curr_status < OPLUS_USBTEMP_LOW_CURR ||
		    usbtemp_dbg_curr_status > OPLUS_USBTEMP_HIGH_CURR) {
			if (batt_current > 5000)
				chip->usbtemp_curr_status = OPLUS_USBTEMP_HIGH_CURR;
			else if (batt_current > 0 && batt_current <= 5000)
				chip->usbtemp_curr_status = OPLUS_USBTEMP_LOW_CURR;
		} else if (usbtemp_dbg_curr_status == OPLUS_USBTEMP_LOW_CURR ||
			   usbtemp_dbg_curr_status == OPLUS_USBTEMP_HIGH_CURR) {
			chip->usbtemp_curr_status = usbtemp_dbg_curr_status;
		}

		if (!curr_range_change && batt_current < 5000 &&
		    chip->usbtemp_pre_batt_current >= 5000) {
			curr_range_change = true;
			curr_range_change_first_time = oplus_current_kernel_time();
		} else if (curr_range_change && batt_current >= 5000 &&
			   chip->usbtemp_pre_batt_current < 5000) {
			curr_range_change = false;
		}

		if (curr_range_change && chip->usbtemp_curr_status == OPLUS_USBTEMP_LOW_CURR) {
			if (oplus_usbtemp_change_curr_range(chip, retry_cnt, usbtemp_first_time_in_curr_range,
							    curr_range_change)) {
				chip->usbtemp_curr_status = OPLUS_USBTEMP_LOW_CURR;
				curr_range_change = false;
			}
			if (!usbtemp_first_time_in_curr_range)
				usbtemp_first_time_in_curr_range = true;
			curr_range_change_last_time = oplus_current_kernel_time();
			if ((curr_range_change_last_time.tv_sec - curr_range_change_first_time.tv_sec) >=
			    OPLUS_USBTEMP_CHANGE_RANGE_TIME)
				chip->usbtemp_curr_status = OPLUS_USBTEMP_LOW_CURR;
		} else {
			usbtemp_first_time_in_curr_range = false;
		}

		if ((chip->usb_temp_l < USB_40C) && (chip->usb_temp_r < USB_40C))
			total_count = OPCHG_LOW_USBTEMP_RETRY_COUNT;
		else if (chip->usbtemp_curr_status == OPLUS_USBTEMP_LOW_CURR)
			total_count = spec->usbtemp_rise_fast_temp_count_low;
		else if (chip->usbtemp_curr_status == OPLUS_USBTEMP_HIGH_CURR)
			total_count = spec->usbtemp_rise_fast_temp_count_high;

		if ((chip->usbtemp_volt_l < USB_50C) && (chip->usbtemp_volt_r < USB_50C) &&
		    (vbus_volt < VBUS_VOLT_THRESHOLD))
			delay = VBUS_MONITOR_INTERVAL;

		if (oplus_usbtemp_trigger_for_high_temp(chip, retry_cnt, count_r, count_l)) {
			if (oplus_usbtemp_check_is_support(chip)) {
				chip->dischg_flag = true;
				condition1 = true;
				chg_err("dischg enable1...[%d, %d]\n", chip->usb_temp_l, chip->usb_temp_r);
			}
			count_r = 1;
			count_l = 1;
			count = 0;
			last_usb_temp_r = chip->usb_temp_r;
			last_usb_temp_l = chip->usb_temp_l;
		}

		if (oplus_usbtemp_trigger_for_rise_fast_temp(chip, retry_cnt, count_r, count_l)) {
			if (oplus_usbtemp_check_is_support(chip)) {
				chip->dischg_flag = true;
				condition1 = true;
				chg_err("dischg enable1...[%d, %d]\n", chip->usb_temp_l, chip->usb_temp_r);
			}
			count_r = 1;
			count_l = 1;
			count = 0;
			last_usb_temp_r = chip->usb_temp_r;
			last_usb_temp_l = chip->usb_temp_l;
		}
		if (condition1) {
			chg_err("jump_to_dischg");
			goto dischg;
		}

		if (oplus_usbtemp_trigger_for_rise_fast_without_temp(chip, retry_cnt, count_r, count_l, total_count)) {
			if (oplus_usbtemp_check_is_support(chip)) {
				chip->dischg_flag = true;
				condition2 = true;
			}
		}

	dischg:
		if ((chip->usb_temp_l < USB_30C || chip->usb_temp_l > USB_100C) &&
		    (chip->usb_temp_r < USB_30C || chip->usb_temp_r > USB_100C)) {
			condition1 = false;
			condition2 = false;
			chip->dischg_flag = false;
		}

		if ((condition1 || condition2) && chip->dischg_flag) {
			condition = condition1 ? USBTEMP_TRIGGER_CONDITION_1 : USBTEMP_TRIGGER_CONDITION_2;
			oplus_usbtemp_push_err_msg(chip, condition, last_usb_temp_l,
						   last_usb_temp_r, batt_current);
			oplus_usbtemp_dischg_action(chip);
			condition1 = false;
			condition2 = false;
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
			if ((get_eng_version() != HIGH_TEMP_AGING && !oplus_is_ptcrb_version()) &&
#else
			if (
#endif
			    (!chip->cc_detect_support)) {
				chg_info("start delay work for recover charging");
				count = 0;
				last_usb_temp_r = chip->usb_temp_r;
				last_usb_temp_l = chip->usb_temp_l;
				oplus_usbtemp_set_recover(chip);
			}
		}
		msleep(delay);
		chip->usbtemp_pre_batt_current = batt_current;
		if (usbtemp_debug & OPEN_LOG_BIT) {
			chg_info("usbtemp: delay %d %d %d", delay, spec->usbtemp_batt_temp_over_hot, spec->usbtemp_temp_gap_with_batt_temp_in_over_hot);
			chg_info("usbtemp_volt_l[%d], usb_temp_l[%d], usbtemp_volt_r[%d], usb_temp_r[%d] temp[%d]%d\n",
				 chip->usbtemp_volt_l, chip->usb_temp_l, chip->usbtemp_volt_r, chip->usb_temp_r, chip->batt_realy_temp, spec->support_hot_enter_kpoc);
		}
	}
	return 0;
}

static void oplus_usbtemp_thread_init(struct oplus_mms_wired *chip)
{
	init_waitqueue_head(&chip->oplus_usbtemp_wq);
	if (chip->support_usbtemp_protect_v2)
		chip->oplus_usbtemp_kthread = kthread_run(
			oplus_usbtemp_monitor_common_new_method, chip, "usbtemp_kthread");
	else
		chip->oplus_usbtemp_kthread = kthread_run(
			oplus_usbtemp_monitor_main, chip, "usbtemp_kthread");
	if (IS_ERR(chip->oplus_usbtemp_kthread))
		chg_err("failed to cread oplus_usbtemp_kthread\n");
}

void oplus_wake_up_usbtemp_thread(struct oplus_mms_wired *chip)
{
	if (oplus_usbtemp_check_is_support(chip))
		wake_up_interruptible(&chip->oplus_usbtemp_wq);
}

static void oplus_wired_gauge_subs_callback(struct mms_subscribe *subs,
					    enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_mms_wired *chip = subs->priv_data;

	switch (type) {
	case MSG_TYPE_TIMER:
		schedule_work(&chip->gauge_update_work);
		break;
	default:
		break;
	}
}

static void oplus_wired_subscribe_gauge_topic(struct oplus_mms *topic,
					      void *prv_data)
{
	struct oplus_mms_wired *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->gauge_topic = topic;
	chip->gauge_subs = oplus_mms_subscribe(chip->gauge_topic, chip,
					       oplus_wired_gauge_subs_callback,
					       "mms_wired");
	if (IS_ERR_OR_NULL(chip->gauge_subs)) {
		chg_err("subscribe gauge topic error, rc=%ld\n",
			PTR_ERR(chip->gauge_subs));
		return;
	}

	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX, &data,
				false);
	chip->vbat_mv = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_TEMP, &data,
				false);
	chip->batt_temp = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_REAL_TEMP, &data,
				false);
	chip->batt_realy_temp = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CURR, &data,
				false);
	chip->batt_curr = data.intval;
}

static void oplus_wired_gauge_update_work(struct work_struct *work)
{
	int otg_curr = 0;
	struct oplus_mms_wired *chip = container_of(work, struct oplus_mms_wired,
						gauge_update_work);
	union mms_msg_data data = { 0 };
	struct oplus_usbtemp_spec_config *spec;
	spec = &chip->usbtemp_spec;

	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX,
				&data, false);
	chip->vbat_mv = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_TEMP, &data,
				false);
	chip->batt_temp = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_REAL_TEMP, &data,
				false);
	chip->batt_realy_temp = data.intval;

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_SHELL_TEMP, &data,
				false);
	chip->shell_temp = data.intval;
	(void)oplus_chg_ic_func(chip->buck_ic,
				OPLUS_IC_FUNC_VOOCPHY_SET_MATCH_TEMP,
				chip->shell_temp);
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CURR, &data,
				false);
	chip->batt_curr = data.intval;
	if (oplus_wired_is_present() && !chip->vooc_started)
		oplus_wired_set_aicl_point();

	if (chip->otg_enable && chip->support_adjust_reverse_chg_cur) {
		otg_curr = oplus_find_otg_output_level(chip, chip->vbat_mv);
		if (chip->otg_cur > otg_curr) {
			chip->otg_cur = otg_curr;
			oplus_wired_set_otg_boost_curr_limit(chip->otg_cur);
		}
	}
}

static bool oplus_wls_init_complete(struct oplus_mms_wired *chip)
{
       union mms_msg_data data = { 0 };

       if (chip->wls_topic)
               oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_INIT_COMPLETED, &data, true);
       return !!data.intval;
}

static bool oplus_wls_is_present(struct oplus_mms_wired *chip)
{
       union mms_msg_data data = { 0 };

       if (chip->wls_topic)
               oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_PRESENT, &data, true);
       return !!data.intval;
}

#define OTG_BOOST_VOL_THRESHOLD 5000
static void oplus_wired_otg_enable_handler_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_wired *chip =
		container_of(dwork, struct oplus_mms_wired, otg_enable_handler_work);
	struct mms_msg *msg;
	bool enable;
	enum typec_data_role role;
	int rc;
	int vbatt = 0;
	struct oplus_usbtemp_spec_config *spec;

	if (!chip)
		return;
	spec = &chip->usbtemp_spec;

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_GET_OTG_ENABLE, &enable);
	if (rc < 0)
		enable = false;
	chg_info("enable=%d, otg_disbale_status = %d\n", enable, chip->otg_disbale_status);
	enable &= !chip->otg_disbale_status;
	chip->otg_online_status = enable;
	chg_info("otg_online status = %d\n", chip->otg_online_status);
	if (chip->wls_upgrading) {
		chg_info("wls is upgrading\n");
		return;
	}
	if (chip->support_wlsotg_non_coexistence) {
		if (enable && oplus_wls_is_present(chip)) {
			chip->otg_enable_pending = true;
			oplus_wired_set_usb_status(chip, OTG_ENABLE_PENDING);
			chg_info("otg enable pending...\n");
			if (is_batt_psy_available(chip))
				power_supply_changed(chip->batt_psy);
			return;
		}
		if (!enable && chip->otg_enable_pending) {
			chip->otg_enable_pending = false;
			oplus_wired_clear_usb_status(chip, OTG_ENABLE_PENDING);
			chg_info("cancel otg enable pending\n");
			if (is_batt_psy_available(chip))
				power_supply_changed(chip->batt_psy);
			return;
		}
	}

	if (chip->dischg_boost_topic) {
		chg_info("set boost ic otg mode %d!!\n", enable);
		oplus_boost_set_otg_mode(chip->dischg_boost_topic, enable);
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_GET_DATA_ROLE, (int *)&role);
	if (rc < 0)
		role = enable ? TYPEC_HOST : TYPEC_DEVICE;

	if (enable) {
		msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, WIRED_ITEM_PRE_OTG_ENABLE);
		if (msg == NULL) {
			chg_err("alloc msg error\n");
			return;
		}
		rc = oplus_mms_publish_msg_sync(chip->wired_topic, msg);
		if (rc < 0) {
			chg_err("publish otg pre enable msg error, rc=%d\n", rc);
			kfree(msg);
		}
	}

	if (enable) {
		rc = oplus_chg_ic_func(chip->buck_ic,
				       OPLUS_IC_FUNC_DISABLE_VBUS, true, false);
		if (rc < 0 && rc != -ENOTSUPP)
			chg_err("can't disable vbus\n");

		vbatt = oplus_gauge_get_batt_mvolts();

		if (chip->support_adjust_reverse_chg_cur)
			chip->otg_cur = oplus_find_otg_output_level(chip, vbatt);
		else
			chip->otg_cur = spec->usbtemp_otg_cc_boot_current_limit;

		rc = oplus_wired_set_otg_boost_curr_limit(chip->otg_cur);

		if (rc < 0 && rc != -ENOTSUPP)
			chg_err("can't set otg boost curr limit\n");

		rc = oplus_wired_set_otg_boost_vol(OTG_BOOST_VOL_THRESHOLD);
		if (rc < 0 && rc != -ENOTSUPP)
			chg_err("can't set otg boost vol\n");
	}
	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_OTG_BOOST_ENABLE, enable);
	if (rc < 0) {
		chg_err("can't %s otg power supply\n", enable ? "enable" : "disable");
		return;
	}
	chg_info("%s otg power supply, data_role=%s\n",
		 enable ? "enable" : "disable",
		 (role == TYPEC_HOST) ? "host" : "device");
	if (!enable && role == TYPEC_DEVICE) {
		rc = oplus_chg_ic_func(chip->buck_ic,
				       OPLUS_IC_FUNC_DISABLE_VBUS, false, true);
		if (rc < 0 && rc != -ENOTSUPP)
			chg_err("can't enable vbus\n");
	}

	if (chip->otg_enable == enable)
		return;
	chip->otg_enable = enable;
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  WIRED_ITEM_OTG_ENABLE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->wired_topic, msg);
	if (rc < 0) {
		chg_err("publish otg enable msg error, rc=%d\n", rc);
		kfree(msg);
	}

	if (is_vooc_chg_auto_mode_votable_available(chip))
		vote(chip->vooc_chg_auto_mode_votable, OTG_ONLINE_VOTER,
		     enable, enable, false);
	else
		chg_err("vooc_chg_auto_mode_votable not found\n");
}

static void oplus_wired_lpd_info_update_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_wired *chip =
		container_of(dwork, struct oplus_mms_wired, lpd_info_update_work);
	int rc = 0;

	oplus_wired_update_lpd_info(chip, OPLUS_LPD_SEL_SBU1_MASK | OPLUS_LPD_SEL_SBU2_MASK |
			OPLUS_LPD_SEL_SBU1_PULLUP_MASK | OPLUS_LPD_SEL_SBU2_PULLUP_MASK |
			OPLUS_LPD_SEL_CC1_MASK | OPLUS_LPD_SEL_CC2_MASK, 1);
	chip->lpd_spec.lpd_info_status = OPLUS_LPD_DETECT;
	chip->lpd_spec.lpd_cc_status = OPLUS_LPD_DETECT;
	oplus_wired_check_sbu_info(&chip->lpd_info);
	if (chip->lpd_spec.lpd_retry_count < LPD_RETRY_COUNT) {
		chip->lpd_spec.lpd_retry_count++;
		if (chip->wired_present) {
			rc = schedule_delayed_work(&chip->lpd_info_update_work, msecs_to_jiffies(LPD_RETRY_DELAY));
		}
		return;
	} else {
		oplus_wired_check_sbu_ovp_info(&chip->lpd_info);
		oplus_wired_check_sbu_diff_info(&chip->lpd_info);
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
		if (get_eng_version() == FACTORY) {
#else
		if (1) {
#endif
			if (chip->lpd_spec.sbu1_err && chip->lpd_spec.sbu2_err) {
				chip->lpd_spec.lpd_info_status = OPLUS_LPD_ERROR;
				rc = 1;
			}
		} else {
			if (chip->lpd_spec.sbu1_err && chip->lpd_spec.sbu2_err &&
			    chip->lpd_spec.sbu1_diff_err && chip->lpd_spec.sbu2_diff_err) {
				chip->lpd_spec.lpd_info_status = OPLUS_LPD_ERROR;
				rc = 1;
			}
		}
		if ((chip->lpd_spec.support_status == OPLUS_LPD_SUPPORT) &&
		    ((chip->usb_status & USB_TEMP_HIGH) != USB_TEMP_HIGH) &&
		    (rc != 0)) {
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
			chg_err("check lpd info error!eng_version:%d\n", get_eng_version());
#else
			chg_err("check lpd info error! DISABLE_OPLUS_FUNCTION!\n");
#endif
			oplus_wired_set_usb_status(chip, USB_LPD_DETECT);

		}
	}
}

static void oplus_wired_otg_enable_pending_work(struct work_struct *work)
{
	struct oplus_mms_wired *chip =
		container_of(work, struct oplus_mms_wired, otg_enable_pending_work);

	oplus_wired_clear_usb_status(chip, OTG_ENABLE_PENDING);
	if (is_batt_psy_available(chip))
		power_supply_changed(chip->batt_psy);
	chg_info("schedule otg enable work\n");
	schedule_delayed_work(&chip->otg_enable_handler_work, 0);
}

static void oplus_wired_data_role_changed_handler_work(struct work_struct *work)
{
	struct oplus_mms_wired *chip = container_of(
		work, struct oplus_mms_wired, data_role_changed_handler_work);
	struct votable *vooc_disable_votable;
	bool enable;
	enum typec_data_role role;
	int rc;

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_GET_OTG_ENABLE,
			       &enable);
	if (rc < 0)
		enable = false;
	chg_info("enable=%d, otg_disbale_status = %d\n", enable, chip->otg_disbale_status);
	enable &= !chip->otg_disbale_status;

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_GET_DATA_ROLE,
			       (int *)&role);
	if (rc < 0)
		role = enable ? TYPEC_HOST : TYPEC_DEVICE;

	chg_info("otg=%s, data_role=%s\n",
		 enable ? "enable" : "disable",
		 (role == TYPEC_HOST) ? "host" : "device");
	vooc_disable_votable = find_votable("VOOC_DISABLE");
	if (role == TYPEC_DEVICE) {
		if (!IS_ERR(vooc_disable_votable))
			vote(vooc_disable_votable, TYPEC_VOTER, false, 0, false);
		if (!enable) {
			rc = oplus_chg_ic_func(chip->buck_ic,
					OPLUS_IC_FUNC_DISABLE_VBUS, false, true);
			if (rc < 0 && rc != -ENOTSUPP)
				chg_err("can't enable vbus\n");
		}
	} else if (role == TYPEC_HOST) {
		if (!IS_ERR(vooc_disable_votable))
			vote(vooc_disable_votable, TYPEC_VOTER, true, 1, false);
	}
}

static void oplus_wls_upgrading_work(struct work_struct *work)
{
	struct oplus_mms_wired *chip = container_of(work,
			struct oplus_mms_wired, wls_upgrading_work);
	union mms_msg_data data = { 0 };

	oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_FW_UPGRADING, &data, false);
	chip->wls_upgrading = data.intval;
	chg_info("wls_upgrading value = %d\n", chip->wls_upgrading);
	if (!chip->wls_upgrading) {
		if (!chip->otg_online_status)
			return;
		schedule_delayed_work(&chip->otg_enable_handler_work, 0);
	}
}

static void oplus_wired_voltage_change_work(struct work_struct *work)
{
	struct oplus_mms_wired *chip =
		container_of(work, struct oplus_mms_wired, voltage_change_work);
	struct mms_msg *msg;
	int rc;

	/*
	 * Only send the message with the maximum voltage,
	 * the receiving place needs to process two
	 */
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  WIRED_ITEM_CHARGER_VOL_MAX);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->wired_topic, msg);
	if (rc < 0) {
		chg_err("publish charger vol max msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_wired_current_change_work(struct work_struct *work)
{
	struct oplus_mms_wired *chip =
		container_of(work, struct oplus_mms_wired, current_change_work);
	struct mms_msg *msg;
	int rc;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  WIRED_ITEM_CHARGER_CURR_MAX);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->wired_topic, msg);
	if (rc < 0) {
		chg_err("publish charger curr max msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_wired_bc12_completed_work(struct work_struct *work)
{
	struct oplus_mms_wired *chip =
		container_of(work, struct oplus_mms_wired, bc12_completed_work);
	struct mms_msg *msg;
	int rc;

	oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_BUCK_BC12_COMPLETED);

	chip->bc12_completed = true;
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  WIRED_ITEM_BC12_COMPLETED);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->wired_topic, msg);
	if (rc < 0) {
		chg_err("publish bc1.2 completed msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_back_ui_soc_work(struct work_struct *work)
{
	struct oplus_mms_wired *chip =
		container_of(work, struct oplus_mms_wired, back_ui_soc_work);
	union mms_msg_data data = { 0 };

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_UI_SOC, &data,
				false);
	chip->ui_soc = data.intval;
	if (chip->ui_soc < 0) {
		chg_err("ui soc not ready, rc=%d\n", chip->ui_soc);
		return;
	}
	oplus_wired_backup_soc(chip->wired_topic, chip->ui_soc);
}

static void oplus_wired_pd_completed_work(struct work_struct *work)
{
	struct oplus_mms_wired *chip =
		container_of(work, struct oplus_mms_wired, pd_completed_handler_work);
	struct mms_msg *msg;
	int rc;

	chip->pd_completed = true;
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH,
				  WIRED_ITEM_PD_COMPLETED);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->wired_topic, msg);
	if (rc < 0) {
		chg_err("publish pd completed msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_wired_icl_done_work(struct work_struct *work)
{
	struct oplus_mms_wired *chip =
		container_of(work, struct oplus_mms_wired, icl_done_handler_work);
	struct mms_msg *msg;
	int rc;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH,
				  WIRED_ITEM_ICL_DONE_STATUS);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}

	rc = oplus_mms_publish_msg(chip->wired_topic, msg);
	if (rc < 0) {
		chg_err("publish icl done msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_wired_vooc_subs_callback(struct mms_subscribe *subs,
					   enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_mms_wired *chip = subs->priv_data;
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
			if (!chip->cpa_support)
				schedule_work(&chip->chg_type_change_handler_work);
			break;
		case VOOC_ITEM_ONLINE_KEEP:
			oplus_mms_get_item_data(chip->vooc_topic, id, &data,
						false);
			chip->vooc_online_keep = (unsigned int)data.intval;
			schedule_work(&chip->plugin_handler_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_wired_subscribe_vooc_topic(struct oplus_mms *topic,
					     void *prv_data)
{
	struct oplus_mms_wired *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->vooc_topic = topic;
	chip->vooc_subs = oplus_mms_subscribe(chip->vooc_topic, chip,
					      oplus_wired_vooc_subs_callback,
					      "mms_wired");
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

static void oplus_wired_comm_subs_callback(struct mms_subscribe *subs,
					   enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_mms_wired *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case COMM_ITEM_UI_SOC:
			schedule_work(&chip->back_ui_soc_work);
			break;
		case COMM_ITEM_SHELL_TEMP:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->shell_temp = data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_wired_subscribe_comm_topic(struct oplus_mms *topic,
						void *prv_data)
{
	struct oplus_mms_wired *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->comm_topic = topic;
	chip->comm_subs = oplus_mms_subscribe(chip->comm_topic, chip,
					      oplus_wired_comm_subs_callback,
					      "mms_wired");
	if (IS_ERR_OR_NULL(chip->comm_subs)) {
		chg_err("subscribe common topic error, rc=%ld\n",
			PTR_ERR(chip->comm_subs));
		return;
	}

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_UI_SOC, &data,
				true);
	chip->ui_soc = data.intval;
	if (chip->ui_soc < 0) {
		chg_err("ui soc not ready, rc=%d\n", chip->ui_soc);
		chip->ui_soc = 0;
	}
	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_SHELL_TEMP, &data,
				true);
	chip->shell_temp = data.intval;
}

static void oplus_wired_cpa_subs_callback(struct mms_subscribe *subs,
					  enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_mms_wired *chip = subs->priv_data;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case CPA_ITEM_CHG_TYPE:
			schedule_work(&chip->cpa_chg_type_change_handler_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_wired_subscribe_cpa_topic(struct oplus_mms *topic,
					    void *prv_data)
{
	struct oplus_mms_wired *chip = prv_data;

	chip->cpa_topic = topic;
	chip->cpa_subs = oplus_mms_subscribe(chip->cpa_topic, chip,
					     oplus_wired_cpa_subs_callback,
					     "mms_wired");
	if (IS_ERR_OR_NULL(chip->cpa_subs)) {
		chg_err("subscribe cpa topic error, rc=%ld\n",
			PTR_ERR(chip->cpa_subs));
		return;
	}
}

static void oplus_wired_ufcs_subs_callback(struct mms_subscribe *subs,
					   enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_mms_wired *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case UFCS_ITEM_ONLINE:
			oplus_mms_get_item_data(chip->ufcs_topic, id, &data, false);
			chip->ufcs_online = !!data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_wired_subscribe_ufcs_topic(struct oplus_mms *topic,
					     void *prv_data)
{
	struct oplus_mms_wired *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->ufcs_topic = topic;
	chip->ufcs_subs = oplus_mms_subscribe(chip->ufcs_topic, chip,
					      oplus_wired_ufcs_subs_callback,
					      "mms_wired");
	if (IS_ERR_OR_NULL(chip->ufcs_subs)) {
		chg_err("subscribe ufcs topic error, rc=%ld\n", PTR_ERR(chip->ufcs_subs));
		return;
	}

	oplus_mms_get_item_data(chip->ufcs_topic, UFCS_ITEM_ONLINE, &data, true);
	chip->ufcs_online = !!data.intval;
};

static void oplus_wired_wls_subs_callback(struct mms_subscribe *subs,
					     enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_mms_wired *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WLS_ITEM_PRESENT:
			oplus_mms_get_item_data(chip->wls_topic, id, &data, false);
			chip->wls_online = !!data.intval;
			if (!chip->wls_online && chip->otg_enable_pending) {
				chip->otg_enable_pending = false;
				schedule_work(&chip->otg_enable_pending_work);
			}
			break;
		case WLS_ITEM_FW_UPGRADING:
			schedule_work(&chip->wls_upgrading_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_wired_subscribe_wls_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_mms_wired *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->wls_topic = topic;
	chip->wls_subs = oplus_mms_subscribe(chip->wls_topic, chip,
					      oplus_wired_wls_subs_callback,
					      "mms_wired");
	if (IS_ERR_OR_NULL(chip->wls_subs)) {
		chg_err("subscribe wls topic error, rc=%ld\n", PTR_ERR(chip->wls_subs));
		return;
	}

	oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_PRESENT, &data, true);
	chip->wls_online = !!data.intval;
};

static void oplus_reverse_usbtemp_check_work(struct work_struct *work)
{
	struct oplus_mms_wired *chip =
		container_of(work, struct oplus_mms_wired, reverse_usbtemp_check_work);

	oplus_wake_up_usbtemp_thread(chip);
}

static void oplus_wired_reverse_subs_callback(struct mms_subscribe *subs,
					     enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_mms_wired *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case HIGH_REVERSE_ITEM_STATUS:
			oplus_mms_get_item_data(chip->reverse_topic, id, &data, false);
			chip->high_reverse_charging = !!data.intval;
			if (!chip->high_reverse_charging) {
				chg_err("reverse usb temp off\n");
				chip->reverse_usbtemp_check = false;
			}
			break;
		case REVERSE_ITEM_SINK_REQUEST_VOLT:
			oplus_mms_get_item_data(chip->reverse_topic, id, &data, false);
			if (data.intval >= 5000) {
				chip->reverse_usbtemp_check = true;
				chg_err("reverse usb temp on\n");
				schedule_work(&chip->reverse_usbtemp_check_work);
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

static void oplus_wired_dischg_boost_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_wired_subscribe_dischg_boost_topic(struct oplus_mms *topic,
						void *prv_data)
{
	struct oplus_mms_wired *chip = prv_data;

	chip->dischg_boost_topic = topic;
	chip->dischg_boost_subs = oplus_mms_subscribe(chip->dischg_boost_topic, chip,
					    oplus_wired_dischg_boost_subs_callback,
					    "chg_wired");
	if (IS_ERR_OR_NULL(chip->dischg_boost_topic)) {
		chg_err("subscribe dischg boost topic error, rc=%ld\n",
			PTR_ERR(chip->dischg_boost_topic));
		return;
	}

	return;
}

static void oplus_wired_subscribe_reverse_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_mms_wired *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->reverse_topic = topic;
	chip->reverse_subs = oplus_mms_subscribe(chip->reverse_topic, chip,
					      oplus_wired_reverse_subs_callback,
					      "mms_wired");
	if (IS_ERR_OR_NULL(chip->reverse_subs)) {
		chg_err("subscribe reverse topic error, rc=%ld\n", PTR_ERR(chip->reverse_subs));
		return;
	}

	oplus_mms_get_item_data(chip->reverse_topic, HIGH_REVERSE_ITEM_STATUS, &data, true);
	chip->high_reverse_charging = !!data.intval;
};

static int oplus_mms_wired_virq_register(struct oplus_mms_wired *chip);
static int oplus_mms_wired_topic_init(struct oplus_mms_wired *chip);

static void oplus_mms_wired_init_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_wired *chip = container_of(dwork,
		struct oplus_mms_wired, mms_wired_init_work);
	struct device_node *node = chip->dev->of_node;
	static int retry = OPLUS_CHG_IC_INIT_RETRY_MAX;
	int cc_detect;
	int rc;

	chip->buck_ic = of_get_oplus_chg_ic(node, "oplus,buck_ic", 0);
	if (chip->buck_ic == NULL) {
		if (retry > 0) {
			retry--;
			schedule_delayed_work(&chip->mms_wired_init_work,
				msecs_to_jiffies(OPLUS_CHG_IC_INIT_RETRY_DELAY));
			return;
		} else {
			chg_err("oplus,buck_ic not found\n");
		}
		retry = 0;
		return;
	}

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_INIT);
	if (rc == -EAGAIN) {
		if (retry > 0) {
			retry--;
			schedule_delayed_work(&chip->mms_wired_init_work,
				msecs_to_jiffies(OPLUS_CHG_IC_INIT_RETRY_DELAY));
			return;
		} else {
			chg_err("buck_ic init timeout\n");
		}
		retry = 0;
		return;
	} else if (rc < 0) {
		chg_err("buck_ic init error, rc=%d\n", rc);
		retry = 0;
		return;
	}
	retry = 0;

	chg_info("mms wired OPLUS_IC_FUNC_INIT success\n");

	mutex_init(&chip->fcc_lock);
	oplus_mms_wired_virq_register(chip);
	g_mms_wired = chip;
	if (!oplus_chg_get_boot_reset_adapter_support_flags() && !oplus_chg_get_fcs_support_flags())
		oplus_wired_input_enable(false);
	oplus_wired_output_enable(false);
	cc_detect = oplus_wired_get_hw_detect();
	if (cc_detect < 0) {
		chg_err("can't get cc_detect status, rc=%d\n", rc);
		chip->cc_detect_support = false;
	} else if (cc_detect == CC_DETECT_NULL) {
		chg_info("cc_detect not support\n");
		chip->cc_detect_support = false;
	} else {
		chip->cc_detect_support = true;
	}
	if (oplus_usbtemp_check_is_support(chip))
		oplus_usbtemp_thread_init(chip);
	(void)oplus_mms_wired_topic_init(chip);
	oplus_mms_wait_topic("gauge", oplus_wired_subscribe_gauge_topic, chip);
	oplus_mms_wait_topic("vooc", oplus_wired_subscribe_vooc_topic, chip);
	oplus_mms_wait_topic("common", oplus_wired_subscribe_comm_topic, chip);
	oplus_mms_wait_topic("cpa", oplus_wired_subscribe_cpa_topic, chip);
	oplus_mms_wait_topic("ufcs", oplus_wired_subscribe_ufcs_topic, chip);
	oplus_mms_wait_topic("wireless", oplus_wired_subscribe_wls_topic, chip);
	oplus_mms_wait_topic("reverse", oplus_wired_subscribe_reverse_topic, chip);
	oplus_mms_wait_topic("dischg_boost", oplus_wired_subscribe_dischg_boost_topic, chip);

	oplus_chg_ic_virq_trigger(chip->buck_ic, OPLUS_IC_VIRQ_CC_DETECT);
	if (oplus_wired_is_present())
		oplus_chg_ic_virq_trigger(chip->buck_ic, OPLUS_IC_VIRQ_PLUGIN);
}

static void oplus_mms_wired_err_handler_work(struct work_struct *work)
{
	struct oplus_mms_wired *chip = container_of(work, struct oplus_mms_wired,
						err_handler_work);
	struct oplus_chg_ic_err_msg *msg, *tmp;
	struct list_head msg_list;

	INIT_LIST_HEAD(&msg_list);
	spin_lock(&chip->buck_ic->err_list_lock);
	if (!list_empty(&chip->buck_ic->err_list))
		list_replace_init(&chip->buck_ic->err_list, &msg_list);
	spin_unlock(&chip->buck_ic->err_list_lock);

	list_for_each_entry_safe(msg, tmp, &msg_list, list) {
		if (is_err_topic_available(chip))
			oplus_mms_publish_ic_err_msg(chip->err_topic,
						     ERR_ITEM_IC, msg);
		oplus_print_ic_err(msg);
		list_del(&msg->list);
		kfree(msg);
	}
}

static void oplus_mms_wired_bcc_parms_reset(struct oplus_mms_wired *chip)
{
	if (!chip) {
		return;
	}
	chip->bcc_cool_down = 0;
	chip->bcc_curr_done = BCC_CURR_DONE_UNKNOW;
}

static void oplus_mms_wired_online_status_err_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_wired *chip =
		container_of(dwork, struct oplus_mms_wired, wam.online_status_err_work);
	struct oplus_mms_wired_abnormal_monitor *wam = &chip->wam;
	struct mms_msg *msg;
	bool present;
	int hw_detect;
	int rc;
	bool report = false;

	present = oplus_wired_is_present();
	hw_detect = oplus_wired_get_hw_detect();
	if (chip->wired_online == present) {
		wam->online_status_err_count = 0;
		return;
	}

	if (wam->online_status_err_count >= ONLINE_STATUS_ERR_CHECK_MAX)
		report = true;
	if (report) {
		chg_err("report wired online status error, wired_online=%d, vooc_online=%d, vooc_online_keep=%d\r\n",
			chip->wired_online, chip->vooc_online, chip->vooc_online_keep);
		msg = oplus_mms_alloc_int_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
					      WIRED_ITEM_ONLINE_STATUS_ERR, 1);
		if (msg == NULL) {
			chg_err("alloc msg error\n");
		} else {
			rc = oplus_mms_publish_msg(chip->wired_topic, msg);
			if (rc < 0) {
				chg_err("publish online status err msg error, rc=%d\n", rc);
				kfree(msg);
			}
		}
		wam->online_status_err_count = 0;
		return;
	}

	wam->online_status_err_count++;
	schedule_delayed_work(&wam->online_status_err_work,
		msecs_to_jiffies(ONLINE_STATUS_ERR_CHECK_DELAY_MS));
}

static void oplus_mms_update_real_chg_type(struct oplus_mms_wired *chip)
{
	struct mms_msg *msg;
	int rc;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  chip->cpa_support ? WIRED_ITEM_REAL_CHG_TYPE : WIRED_ITEM_CHG_TYPE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg_sync(chip->wired_topic, msg);
	if (rc < 0) {
		chg_err("publish charge type change msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_mms_wired_rerun_present_elections(struct oplus_mms_wired *chip,
						    bool online, bool present,
						    bool present_changed)
{
	chip->usbtemp_curr_status = 0;
	/*
	 * the ICL current needs to be reconfigured when the VBUS is
	 * reconnected and there is no online notification to ensure
	 * that the ICL is configured correctly.
	 */
	if (online && present_changed && present &&
	    is_chg_icl_votable_available(chip))
		rerun_election(chip->chg_icl_votable, true);
	/* Ensure the charging status and current are reset */
	if (present && is_chg_suspend_votable_available(chip))
		rerun_election(chip->chg_suspend_votable, false);
	/* some IC will auto enable charge when vbus rise, ensure the charing status reset */
	if (present && is_chg_disable_votable_available(chip))
		rerun_election(chip->chg_disable_votable, false);
}

static bool oplus_mms_wired_same_online_skip_publish(struct oplus_mms_wired *chip,
						     bool online, bool present,
						     bool present_changed)
{
	if (chip->wired_online != online)
		return false;

	if (chip->wired_online_init_flag) {
		chg_info("Booted with USB connected\n");
		chip->wired_online_init_flag = false;
		return false;
	}

	oplus_mms_wired_rerun_present_elections(chip, online, present,
						present_changed);
	return true;
}

static void oplus_mms_wired_plugin_handler_work(struct work_struct *work)
{
	struct oplus_mms_wired *chip =
		container_of(work, struct oplus_mms_wired, plugin_handler_work);
	struct mms_msg *msg;
	union mms_msg_data data = { 0 };
	bool online, present;
	bool present_changed = false;
	enum typec_data_role role;
	struct votable *pd_boost_disable_votable;
	int rc;

	if (chip->wired_topic == NULL) {
		chg_err("wired_topic not ready\n");
		return;
	}

	if (chip->vooc_topic) {
		oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_ONLINE_KEEP,
					&data, false);
		chip->vooc_online_keep = data.intval;
	} else {
		chip->vooc_online_keep = false;
	}

	present = oplus_wired_is_present();
	if (!present) {
		chip->bc12_completed = false;
		chip->pd_completed = false;
		chip->set_icl_done = false;
		chip->smart_charge_user = SMART_CHARGE_USER_OTHER;
		/* Ensure usbtemp cool_down set 0 when plug out */
		if (is_cool_down_votable_available(chip))
			vote(chip->cool_down_votable, USB_VOTER, false, 0, false);
		/* need inform when quick attach/unattach */
		if (present != chip->wired_present)
			schedule_work(&chip->icl_done_handler_work);
		oplus_wired_clear_usb_status(chip, USB_LPD_DETECT);
	}

	if (chip->wired_present != present) {
		chg_info("wired_present=%d\n", present);
		chip->wired_present = present;
		present_changed = true;
		msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
					  WIRED_ITEM_PRESENT);
		if (msg == NULL) {
			chg_err("alloc msg error\n");
			goto skip_present;
		}
		rc = oplus_mms_publish_msg(chip->wired_topic, msg);
		if (rc < 0) {
			chg_err("publish wired present msg error, rc=%d\n", rc);
			kfree(msg);
		}

		schedule_work(&chip->chg_type_change_handler_work);
	}

	oplus_mms_wired_bcc_parms_reset(chip);

skip_present:
	online = present || chip->vooc_online_keep;
	chg_info("present=%d, vooc_online_keep=%d, pre_online=%d", present,
		 chip->vooc_online_keep, chip->wired_online);
	if (online != present) {
		if (!(work_busy(&chip->wam.online_status_err_work.work)))
			schedule_delayed_work(&chip->wam.online_status_err_work, 0);
	} else {
		cancel_delayed_work_sync(&chip->wam.online_status_err_work);
		chip->wam.online_status_err_count = 0;
	}

	if (!oplus_mms_wired_same_online_skip_publish(chip, online, present,
						      present_changed)) {
		if (is_pd_svooc_votable_available(chip) && !online)
			vote(chip->pd_svooc_votable, SVID_VOTER, false, 0, false);

		chip->wired_online = online;
		chip->usbtemp_check = online;
		if (chip->usbtemp_check)
			oplus_wake_up_usbtemp_thread(chip);
		/* TODO: add otg */

		/* before publish online status, need to update the real chg type */
		oplus_mms_update_real_chg_type(chip);

		msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
					  WIRED_ITEM_ONLINE);
		if (msg == NULL) {
			chg_err("alloc msg error\n");
			goto check_data_role;
		}
		rc = oplus_mms_publish_msg(chip->wired_topic, msg);
		if (rc < 0) {
			chg_err("publish wired online msg error, rc=%d\n", rc);
			kfree(msg);
		}
	}

check_data_role:
	pd_boost_disable_votable = find_votable("PD_BOOST_DISABLE");
	if (chip->wired_online && pd_boost_disable_votable &&
	    get_client_vote(pd_boost_disable_votable, SVID_VOTER) > 0) {
		chg_err("rerun svid_handler_work\n");
		schedule_delayed_work(&chip->svid_handler_work, 0);
	}
	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_GET_DATA_ROLE,
			       (int *)&role);
	if (rc < 0) {
		if (rc != -ENOTSUPP)
			chg_err("can't get typec data role, rc=%d\n", rc);
		return;
	}

	/* HW does not trigger type recognition when switch power role */
	if (role == TYPEC_HOST)
		oplus_chg_ic_virq_trigger(chip->buck_ic,
			OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);

	if (present_changed && !present)
		oplus_wired_set_aicl_point();
}

static void
oplus_mms_wired_chg_type_change_handler_work(struct work_struct *work)
{
	struct oplus_mms_wired *chip = container_of(
		work, struct oplus_mms_wired, chg_type_change_handler_work);
	struct mms_msg *msg;
	int rc;
	int real_chg_type;
	union mms_msg_data data = { 0 };

	if (chip->wired_topic == NULL) {
		chg_err("wired_topic not ready\n");
		return;
	}

	if (chip->wired_online != oplus_wired_is_present())
		oplus_chg_ic_virq_trigger(chip->buck_ic, OPLUS_IC_VIRQ_PLUGIN);

	real_chg_type = oplus_wired_get_real_chg_type(chip);
	chg_info("real_type=%s\n", oplus_wired_get_chg_type_str(oplus_wired_get_real_chg_type(chip)));
	oplus_wired_check_lpd(chip, real_chg_type);
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  chip->cpa_support ? WIRED_ITEM_REAL_CHG_TYPE : WIRED_ITEM_CHG_TYPE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->wired_topic, msg);
	if (rc < 0) {
		chg_err("publish charge type change msg error, rc=%d\n", rc);
		kfree(msg);
	}

	if (chip->cpa_support && chip->cpa_topic &&
	    (real_chg_type == OPLUS_CHG_USB_TYPE_PD || real_chg_type == OPLUS_CHG_USB_TYPE_PD_PPS)) {
		oplus_mms_get_item_data(chip->cpa_topic, CPA_ITEM_ALLOW, &data, true);
		if (data.intval != CHG_PROTOCOL_PD) {
			oplus_cpa_request(chip->cpa_topic, CHG_PROTOCOL_PD);
			chg_info("pd message came late. Retry arbitration.\n");
		} else {
			chg_info("%s message came late. do nothing.\n",
				 real_chg_type == OPLUS_CHG_USB_TYPE_PD ? "PD" : "PPS");
		}
	}
}

static void
oplus_mms_wired_cpa_chg_type_change_handler_work(struct work_struct *work)
{
	struct oplus_mms_wired *chip = container_of(
		work, struct oplus_mms_wired, cpa_chg_type_change_handler_work);
	struct mms_msg *msg;
	int rc;

	if (chip->wired_topic == NULL) {
		chg_err("wired_topic not ready\n");
		return;
	}

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  WIRED_ITEM_CHG_TYPE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->wired_topic, msg);
	if (rc < 0) {
		chg_err("publish cpa charge type change msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_mms_wired_ccdetect_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_wired *chip =
		container_of(dwork, struct oplus_mms_wired, ccdetect_work);
	struct mms_msg *msg;
	int hw_detect;
	int rc;

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_BUCK_GET_HW_DETECT, &hw_detect, false);
	if (rc < 0) {
		chg_err("can't get hw detect status, rc=%d\n", rc);
		return;
	}

	chg_info("hw_detect=%d\n", hw_detect);
	if (hw_detect == 1) {
		oplus_wired_ccdetect_enable(chip, true);
		if ((chip->usb_status & USB_TEMP_HIGH) == USB_TEMP_HIGH) {
			cancel_delayed_work(&chip->usbtemp_recover_work);
			schedule_delayed_work(&chip->usbtemp_recover_work, 0);
		}
		if (chip->wired_online && !chip->usbtemp_check) {
			chip->usbtemp_check = true;
			oplus_wake_up_usbtemp_thread(chip);
		}
	} else {
		chip->usbtemp_check = false;
		chip->abnormal_adapter = false;
		if((chip->usb_status & USB_TEMP_HIGH) == USB_TEMP_HIGH) {
			cancel_delayed_work(&chip->usbtemp_recover_work);
			schedule_delayed_work(&chip->usbtemp_recover_work, 0);
		}
		if (!oplus_wired_get_otg_switch_status()) {
			oplus_wired_ccdetect_enable(chip, false);
		}
	}
	(void)oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_CC_DETECT_HAPPENED);

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  WIRED_ITEM_CC_DETECT);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->wired_topic, msg);
	if (rc < 0) {
		chg_err("publish cc detect msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_mms_wired_typec_state_notify_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_wired *chip =
		container_of(dwork, struct oplus_mms_wired, typec_state_notify_work);
	struct mms_msg *msg;
	int typec_state;
	int rc;

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_BUCK_GET_TYPEC_STATE, &typec_state);
	if (rc < 0) {
		chg_err("can't get hw detect status, rc=%d\n", rc);
		return;
	}

	chg_info("typec_state=%d\n", typec_state);

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  WIRED_TIME_TYPEC_STATE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->wired_topic, msg);
	if (rc < 0) {
		chg_err("publish typec_state msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_mms_wired_typec_state_change_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_wired *chip = container_of(
		dwork, struct oplus_mms_wired, typec_state_change_work);
	int hw_detect;
	int rc;

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_BUCK_GET_HW_DETECT, &hw_detect, false);
	if (rc < 0) {
		chg_err("can't get hw detect status, rc=%d\n", rc);
		return;
	}

	chg_info("hw_detect=%d\n", hw_detect);

	if(oplus_wired_ccdetect_is_support()) {
		if (hw_detect == 0 && !oplus_wired_get_otg_switch_status())
			oplus_wired_ccdetect_enable(chip, false);
	}
}

static void oplus_mms_wired_power_role_change_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_wired *chip = container_of(
		dwork, struct oplus_mms_wired, power_role_change_work);
	struct mms_msg *msg;
	int rc;

	if (chip->buck_ic == NULL) {
		chg_err("chip->buck_ic error\n");
		return;
	}
	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_BUCK_GET_POWER_ROLE, &chip->power_role);
	if (rc < 0) {
		chg_err("can't get power role status, rc=%d\n", rc);
		return;
	}

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  WIRED_ITEM_POWER_ROLE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->wired_topic, msg);
	if (rc < 0) {
		chg_err("publish power_role msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_mms_wired_svid_handler_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_wired *chip =
		container_of(dwork, struct oplus_mms_wired, svid_handler_work);
	struct votable *pd_boost_disable_votable;
	bool oplus_svid;
	int rc;
	struct mms_msg *msg;

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_IS_OPLUS_SVID,
			       &oplus_svid);
	if (rc < 0) {
		chg_err("can't get oplus svid status, rc=%d\n", rc);
		oplus_svid = false;
	} else {
		chg_info("oplus_svid=%s\n", oplus_svid ? "true" : "false");
	}

	chip->abnormal_adapter = oplus_svid;
	if (oplus_svid && is_pd_svooc_votable_available(chip))
		vote(chip->pd_svooc_votable, SVID_VOTER, true, 1, false);

	pd_boost_disable_votable = find_votable("PD_BOOST_DISABLE");
	if (pd_boost_disable_votable)
		vote(pd_boost_disable_votable, SVID_VOTER, false, 0, false);

	oplus_chg_ic_virq_trigger(chip->buck_ic, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  WIRED_TIME_ABNORMAL_ADAPTER);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
	}
	rc = oplus_mms_publish_msg(chip->wired_topic, msg);
	if (rc < 0) {
		chg_err("publish oplus svid msg error, rc=%d\n", rc);
		kfree(msg);
		return;
	}
}

static void oplus_wired_err_handler(struct oplus_chg_ic_dev *ic_dev,
					void *virq_data)
{
	struct oplus_mms_wired *chip = virq_data;
	schedule_work(&chip->err_handler_work);
}

static void oplus_wired_plugin_handler(struct oplus_chg_ic_dev *ic_dev,
					void *virq_data)
{
	struct oplus_mms_wired *chip = virq_data;

	if (virq_data != NULL)
		schedule_work(&chip->plugin_handler_work);
	else
		chg_info("wired plugin virq_data null\n");
}

static void oplus_wired_type_change_handler(struct oplus_chg_ic_dev *ic_dev,
					void *virq_data)
{
	struct oplus_mms_wired *chip = virq_data;
	schedule_work(&chip->chg_type_change_handler_work);
}

static void oplus_wired_svid_handler(struct oplus_chg_ic_dev *ic_dev,
					void *virq_data)
{
	struct oplus_mms_wired *chip = virq_data;
	schedule_delayed_work(&chip->svid_handler_work, 0);
}

#define OTG_ENABLE_DELAY_MS	1500
static void oplus_wired_otg_enable_handler(struct oplus_chg_ic_dev *ic_dev,
					   void *virq_data)
{
	struct oplus_mms_wired *chip = virq_data;
	unsigned long otg_enable_delay_jiffies = 0;

	if (chip->support_wlsotg_non_coexistence && !oplus_wls_init_complete(chip))
		otg_enable_delay_jiffies = msecs_to_jiffies(OTG_ENABLE_DELAY_MS);
	schedule_delayed_work(&chip->otg_enable_handler_work, otg_enable_delay_jiffies);
}

#define OPLUS_CHG_VBUS_5V		5000
#define OPLUS_CHG_VBUS_9V		9000
static void oplus_mms_wired_power_changed_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_wired *chip = container_of(
		dwork, struct oplus_mms_wired, power_changed_work);
	struct mms_msg *msg;
	int num = 0;
	int i;
	int rc;

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_GET_SOURCE_PDO, (u32*)chip->pdo, &num);
	if (rc < 0)
		chg_err("can't get source pdo, rc=%d\n", rc);

	chip->pdo_volt = 0;
	for (i = 0; i < num; i++) {
		if (chip->pdo[i].pdo_type != USBPD_PDMSG_PDOTYPE_FIXED_SUPPLY)
			continue;

		chg_info("SourceCap[%d]: %08X, FixedSupply PDO V=%d mV, I=%d mA,"
			"UsbCommCapable=%d, USBSuspendSupported:%d\n", i,
			chip->pdo[i].pdo_data, PD_PDO_VOL(chip->pdo[i].voltage_50mv),
			PD_PDO_CURR_MAX(chip->pdo[i].max_current_10ma),
			chip->pdo[i].usb_comm_capable, chip->pdo[i].usb_suspend_supported);

		if (PD_PDO_VOL(chip->pdo[i].voltage_50mv) == OPLUS_CHG_VBUS_9V) {
			chip->pdo_volt = OPLUS_CHG_VBUS_9V;
			break;
		} else if (PD_PDO_VOL(chip->pdo[i].voltage_50mv) == OPLUS_CHG_VBUS_5V) {
			chip->pdo_volt = OPLUS_CHG_VBUS_5V;
		}
	}

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
					WIRED_ITEM_SOURCE_PDO_VOLT);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->wired_topic, msg);
	if (rc < 0) {
		chg_err("publish sink request msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_wired_power_changed_handler(struct oplus_chg_ic_dev *ic_dev,
					   void *virq_data)
{
	struct oplus_mms_wired *chip = virq_data;

	cancel_delayed_work(&chip->power_changed_work);
	schedule_delayed_work(&chip->power_changed_work, 0);
}


#define CCDETECT_DELAY_MS	50
static void oplus_wired_cc_detect_handler(struct oplus_chg_ic_dev *ic_dev,
					  void *virq_data)
{
	struct oplus_mms_wired *chip = virq_data;

	cancel_delayed_work(&chip->ccdetect_work);
	schedule_delayed_work(&chip->ccdetect_work, msecs_to_jiffies(CCDETECT_DELAY_MS));
}

#define TYPEC_STATE_DELAY_MS	50
static void oplus_wired_typec_state_notify_handler(struct oplus_chg_ic_dev *ic_dev,
					  void *virq_data)
{
	struct oplus_mms_wired *chip = virq_data;

	cancel_delayed_work(&chip->typec_state_notify_work);
	schedule_delayed_work(&chip->typec_state_notify_work, msecs_to_jiffies(CCDETECT_DELAY_MS));
}

static void oplus_wired_cc_changed_handler(struct oplus_chg_ic_dev *ic_dev,
					  void *virq_data)
{
	struct oplus_mms_wired *chip = virq_data;

	schedule_delayed_work(&chip->typec_state_change_work, 0);
}

static void oplus_wired_power_role_state_handler(struct oplus_chg_ic_dev *ic_dev,
					  void *virq_data)
{
	struct oplus_mms_wired *chip = virq_data;
	schedule_delayed_work(&chip->power_role_change_work, 0);
}

static void oplus_wired_voltage_changed_handler(struct oplus_chg_ic_dev *ic_dev,
						void *virq_data)
{
	struct oplus_mms_wired *chip = virq_data;

	schedule_work(&chip->voltage_change_work);
}

static void oplus_wired_current_changed_handler(struct oplus_chg_ic_dev *ic_dev,
						void *virq_data)
{
	struct oplus_mms_wired *chip = virq_data;

	schedule_work(&chip->current_change_work);
}

static void oplus_wired_bc12_completed_handler(struct oplus_chg_ic_dev *ic_dev,
					       void *virq_data)
{
	struct oplus_mms_wired *chip = virq_data;

	schedule_work(&chip->bc12_completed_work);
	schedule_work(&chip->chg_type_change_handler_work);
}

static void
oplus_wired_data_role_changed_handler(struct oplus_chg_ic_dev *ic_dev,
				      void *virq_data)
{
	struct oplus_mms_wired *chip = virq_data;

	schedule_work(&chip->data_role_changed_handler_work);
}

static void oplus_wired_pd_completed_notify_handler(struct oplus_chg_ic_dev *ic_dev,
						    void *virq_data)
{
	struct oplus_mms_wired *chip = virq_data;

	schedule_work(&chip->pd_completed_handler_work);
}

static int oplus_mms_wired_virq_register(struct oplus_mms_wired *chip)
{
	int rc;

	rc = oplus_chg_ic_virq_register(chip->buck_ic, OPLUS_IC_VIRQ_ERR,
		oplus_wired_err_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_ERR error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(chip->buck_ic, OPLUS_IC_VIRQ_PLUGIN,
		oplus_wired_plugin_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_PLUGIN error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(chip->buck_ic, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE,
		oplus_wired_type_change_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_CHG_TYPE_CHANGE error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(chip->buck_ic, OPLUS_IC_VIRQ_SVID,
		oplus_wired_svid_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_SVID error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(chip->buck_ic, OPLUS_IC_VIRQ_OTG_ENABLE,
		oplus_wired_otg_enable_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_OTG_ENABLE error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(chip->buck_ic, OPLUS_IC_VIRQ_POWER_CHANGED,
		oplus_wired_power_changed_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_POWER_CHANGED error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(chip->buck_ic, OPLUS_IC_VIRQ_CC_DETECT,
		oplus_wired_cc_detect_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_CC_DETECT error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(chip->buck_ic, OPLUS_IC_VIRQ_CC_CHANGED,
		oplus_wired_cc_changed_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_CC_CHANGED error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(chip->buck_ic, OPLUS_IC_VIRQ_POWER_ROLE_STATUS,
		oplus_wired_power_role_state_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_POWER_ROLE_STATUS error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(chip->buck_ic, OPLUS_IC_VIRQ_VOLTAGE_CHANGED,
		oplus_wired_voltage_changed_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_VOLTAGE_CHANGED error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(chip->buck_ic, OPLUS_IC_VIRQ_CURRENT_CHANGED,
		oplus_wired_current_changed_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_CURRENT_CHANGED error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(chip->buck_ic, OPLUS_IC_VIRQ_BC12_COMPLETED,
		oplus_wired_bc12_completed_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_BC12_COMPLETED error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(chip->buck_ic, OPLUS_IC_VIRQ_TYPEC_STATE,
		oplus_wired_typec_state_notify_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_TYPEC_STATE error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(chip->buck_ic, OPLUS_IC_VIRQ_PD_COMPLETED,
		oplus_wired_pd_completed_notify_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_PD_COMPLETED error, rc=%d", rc);

	rc = oplus_chg_ic_virq_register(chip->buck_ic,
					OPLUS_IC_VIRQ_DATA_ROLE_CHANGED,
					oplus_wired_data_role_changed_handler,
					chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_DATA_ROLE_CHANGED error, rc=%d",
			rc);

	return 0;
}

static int oplus_mms_wired_update_present(struct oplus_mms *mms,
					  union mms_msg_data *data)
{
	struct oplus_mms_wired *chip;
	bool present = false;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	present = chip->wired_present;
	data->intval = present;
	return 0;
}

static int oplus_mms_wired_update_online(struct oplus_mms *mms,
					 union mms_msg_data *data)
{
	struct oplus_mms_wired *chip;
	bool present = false;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	present = chip->wired_online;
	data->intval = present;
	return 0;
}

static int oplus_mms_wired_update_chg_type(struct oplus_mms *mms,
					   union mms_msg_data *data)
{
	struct oplus_mms_wired *chip;
	int type = OPLUS_CHG_USB_TYPE_UNKNOWN;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	type = oplus_wired_get_chg_type();
	if (type < 0) {
		chg_err("can't get charger type, rc=%s", oplus_wired_get_chg_type_str(type));
		goto end;
	}

end:
	chg_info("chg_type update to %s\n", oplus_wired_get_chg_type_str(type));
	data->intval = type;
	return 0;
}

static int oplus_mms_wired_update_bc12_completed(struct oplus_mms *mms,
						 union mms_msg_data *data)
{
	struct oplus_mms_wired *chip;
	bool bc12_completed = false;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	bc12_completed = chip->bc12_completed;
	data->intval = bc12_completed;
	return 0;
}

static int oplus_mms_wired_update_cc_mode(struct oplus_mms *mms,
					  union mms_msg_data *data)
{
	/* TODO */
	return 0;
}

static int oplus_mms_wired_update_cc_detect(struct oplus_mms *mms,
					    union mms_msg_data *data)
{
	struct oplus_mms_wired *chip;
	enum oplus_wired_cc_detect_status cc_detect_status = CC_DETECT_NULL;
	int cc_detect;
	int rc;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_BUCK_GET_HW_DETECT,
			       &cc_detect, false);
	if (rc < 0) {
		chg_err("can't get cc detect status, rc=%d\n", rc);
		cc_detect_status = CC_DETECT_NULL;
		goto end;
	}

	if (!!cc_detect)
		cc_detect_status = CC_DETECT_PLUGIN;
	else
		cc_detect_status = CC_DETECT_NOTPLUG;

end:
	data->intval = cc_detect_status;
	return 0;
}

static int oplus_mms_wired_update_usb_status(struct oplus_mms *mms,
					     union mms_msg_data *data)
{
	struct oplus_mms_wired *chip;
	int usb_status = 0;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	usb_status = (int)chip->usb_status;
	data->intval = usb_status;
	return 0;
}

#define USBTEMP_DEFAULT_VOLT_VALUE_MV 950
static int oplus_mms_wired_update_usb_temp_volt_l(struct oplus_mms *mms,
						  union mms_msg_data *data)
{
	struct oplus_mms_wired *chip;
	int volt = USBTEMP_DEFAULT_VOLT_VALUE_MV;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	volt = chip->usbtemp_volt_l;
	data->intval = volt;
	return 0;
}

static int oplus_mms_wired_update_usb_temp_volt_r(struct oplus_mms *mms,
						  union mms_msg_data *data)
{
	struct oplus_mms_wired *chip;
	int volt = USBTEMP_DEFAULT_VOLT_VALUE_MV;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	volt = chip->usbtemp_volt_r;
	data->intval = volt;
	return 0;
}

static int oplus_mms_wired_update_usb_temp_l(struct oplus_mms *mms,
					     union mms_msg_data *data)
{
	struct oplus_mms_wired *chip;
	int temp = 25;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	temp = chip->usb_temp_l;
	data->intval = temp;
	return 0;
}

static int oplus_mms_wired_update_usb_temp_r(struct oplus_mms *mms,
					     union mms_msg_data *data)
{
	struct oplus_mms_wired *chip;
	int temp = 25;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	temp = chip->usb_temp_l;
	data->intval = temp;
	return 0;
}

static int oplus_mms_wired_update_pre_otg_enable_status(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_wired *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);
	/*enable = chip->pre_otg_enable;
	data->intval = enable;*/

	return 0;
}

static int oplus_mms_wired_update_otg_enable_status(struct oplus_mms *mms,
						    union mms_msg_data *data)
{
	struct oplus_mms_wired *chip;
	bool enable = false;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	enable = chip->otg_enable;
	data->intval = enable;
	return 0;
}

static int oplus_mms_wired_update_chg_curr_max(struct oplus_mms *mms,
					       union mms_msg_data *data)
{
	struct oplus_mms_wired *chip;
	int curr_max;
	int rc;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_GET_CHARGER_CURR_MAX, &curr_max);
	if (rc < 0) {
		chg_err("can't get charger current max, rc=%d\n", rc);
		return rc;
	}
	chg_info("Maximum allowable charger current [%d]\n", curr_max);
	data->intval = curr_max;

	return 0;
}

static int oplus_mms_wired_update_pd_completed(struct oplus_mms *mms,
						 union mms_msg_data *data)
{
	struct oplus_mms_wired *chip;
	bool pd_completed = false;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	pd_completed = chip->pd_completed;
	data->intval = pd_completed;
	return 0;
}

static int oplus_mms_wired_update_icl_status(struct oplus_mms *mms,
						 union mms_msg_data *data)
{
	struct oplus_mms_wired *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}

	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	data->intval = chip->set_icl_done;

	return 0;
}

static int oplus_mms_wired_update_buck_eis_current_rate(struct oplus_mms *mms,
						union mms_msg_data *data)
{
	struct oplus_mms_wired *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}

	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	data->intval = chip->buck_eis_current_rate;

	return 0;
}

static int oplus_mms_wired_update_source_pdo_volt(struct oplus_mms *mms,
						    union mms_msg_data *data)
{
	struct oplus_mms_wired *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);
	data->intval = chip->pdo_volt;
	chg_info("update pdo volt:%d\n", chip->pdo_volt);
	return 0;
}

void oplus_wired_check_bcc_curr_done(struct oplus_mms *topic)
{
	struct oplus_mms_wired *chip = g_mms_wired;

	if (!chip) {
		return;
	}

	mutex_lock(&chip->bcc_curr_done_mutex);
	if (chip->bcc_curr_done == BCC_CURR_DONE_REQUEST) {
		chip->bcc_curr_done = BCC_CURR_DONE_ACK;
		chg_err("bcc_curr_done:%d\n", chip->bcc_curr_done);
	}
	mutex_unlock(&chip->bcc_curr_done_mutex);
}

void oplus_wired_set_bcc_curr_request(struct oplus_mms *topic)
{
	struct oplus_mms_wired *chip = g_mms_wired;

	if (!chip) {
		return;
	}

	mutex_lock(&chip->bcc_curr_done_mutex);
	chip->bcc_curr_done = BCC_CURR_DONE_REQUEST;
	chg_err("bcc_curr_done:%d\n", chip->bcc_curr_done);
	mutex_unlock(&chip->bcc_curr_done_mutex);
}

int oplus_wired_get_bcc_curr_done_status(struct oplus_mms *topic)
{
	struct oplus_mms_wired *chip = g_mms_wired;
	int ret = 0;

	if (!chip) {
		return 0;
	}

	mutex_lock(&chip->bcc_curr_done_mutex);
	ret = chip->bcc_curr_done;
	mutex_unlock(&chip->bcc_curr_done_mutex);

	return ret;
}

static int oplus_mms_wired_update_chg_vol_max(struct oplus_mms *mms,
					      union mms_msg_data *data)
{
	struct oplus_mms_wired *chip;
	int vol_max;
	int rc;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_GET_CHARGER_VOL_MAX,
			       &vol_max);
	if (rc < 0) {
		chg_err("can't get charger voltage max, rc=%d\n", rc);
		return rc;
	}
	chg_info("Maximum allowable charger voltage [%d]\n", vol_max);
	data->intval = vol_max;

	return 0;
}

static int oplus_mms_wired_update_chg_vol_min(struct oplus_mms *mms,
					      union mms_msg_data *data)
{
	struct oplus_mms_wired *chip;
	int vol_min;
	int rc;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_GET_CHARGER_VOL_MIN,
			       &vol_min);
	if (rc < 0) {
		chg_err("can't get charger voltage min, rc=%d\n", rc);
		return rc;
	}
	chg_info("Minimum allowable charger voltage [%d]\n", vol_min);
	data->intval = vol_min;

	return 0;
}

static int oplus_mms_abnoramal_adapter(struct oplus_mms *mms,
				       union mms_msg_data *data)
{
	struct oplus_mms_wired *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);
	data->intval = chip->abnormal_adapter;
	chg_info("abnormal adapter %d \n", data->intval);
	return 0;
}

static int oplus_mms_get_typec_state(struct oplus_mms *mms,
				       union mms_msg_data *data)
{
	struct oplus_mms_wired *chip;
	int typec_state;
	int rc;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_BUCK_GET_TYPEC_STATE,
			       &typec_state);
	if (rc < 0) {
		chg_err("can't get typec state fail, rc=%d\n", rc);
		return rc;
	}
	chg_info("typec_state = [%d]\n", typec_state);
	data->intval = typec_state;

	return 0;
}

static int oplus_mms_wired_update_power_role(struct oplus_mms *mms,
				       union mms_msg_data *data)
{
	struct oplus_mms_wired *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	data->intval = chip->power_role;
	chg_info("power role = [%d]\n", chip->power_role);

	return 0;
}

static int oplus_mms_wired_update_real_chg_type(
	struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_wired *chip;
	int type = OPLUS_CHG_USB_TYPE_UNKNOWN;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	type = oplus_wired_get_real_chg_type(chip);
	if (type < 0) {
		chg_err("can't get real charger type, rc=%d", type);
		return type;
	}

	chg_info("real chg_type update to %s\n", oplus_wired_get_chg_type_str(type));
	data->intval = type;
	return 0;
}

static int oplus_mms_wired_update_charging_disable(
	struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_wired *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);
	data->intval = chip->charging_disable;

	return 0;
}

static int oplus_mms_wired_update_coexistence(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_wired *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);
	data->intval = chip->support_wlsotg_non_coexistence;

	return 0;
}

static int oplus_mms_wired_vbus(
	struct oplus_mms *mms, union mms_msg_data *data)
{
	int vbus;

	vbus = oplus_wired_get_vbus();
	if (vbus < 0) {
		chg_err("can't get real charger vbus, rc=%d", vbus);
		return vbus;
	}

	data->intval = vbus;

	return 0;
}

int oplus_wired_get_byb_id_info(struct oplus_mms *topic)
{
	int rc = 0;
	struct oplus_mms_wired *chip;
	int bybid_info = 0;

	if (topic == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(topic);

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_BUCK_GET_BYBID_INFO,
			       &bybid_info);
	if (rc < 0) {
		if (rc != -ENOTSUPP)
			chg_err("can't get bybid info, rc=%d\n", rc);

		return GPIO_STATUS_NOT_SUPPORT;
	}

	return bybid_info;
}

int oplus_wired_get_byb_id_match_info(struct oplus_mms *topic)
{
	int rc = 0;
	struct oplus_mms_wired *chip;
	int bybid_match_info = ID_MATCH_IGNORE;

	if (topic == NULL) {
		chg_err("mms is NULL");
		return ID_MATCH_IGNORE;
	}
	chip = oplus_mms_get_drvdata(topic);

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_BUCK_GET_BYBID_MATCH_INFO,
			       &bybid_match_info);
	if (rc < 0) {
		if (rc != -ENOTSUPP)
			chg_err("can't get bybid match info, rc=%d\n", rc);

		return ID_MATCH_IGNORE;
	}

	return bybid_match_info;
}

int oplus_wired_get_byb_status_info(struct oplus_mms *topic, char *byb_buff)
{
	int rc = 0;
	struct oplus_mms_wired *chip;

	if (topic == NULL) {
		chg_err("mms is NULL");
		return ID_MATCH_IGNORE;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (chip == NULL || chip->buck_ic == NULL) {
		chg_err("chip is NULL");
		return ID_MATCH_IGNORE;
	}
	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_BUCK_GET_BYB_STATUS,
			       byb_buff);
	if (rc <= 0) {
		if (rc != -ENOTSUPP)
			chg_err("can't get_byb_status_info, rc=%d\n", rc);

	}

	return rc;
}


int oplus_wired_get_byb_vout_info(struct oplus_mms *topic, char *byb_buff)
{
	int rc = 0;
	struct oplus_mms_wired *chip;

	if (topic == NULL) {
		chg_err("mms is NULL");
		return ID_MATCH_IGNORE;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (chip == NULL || chip->buck_ic == NULL) {
		chg_err("chip is NULL");
		return ID_MATCH_IGNORE;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_BUCK_GET_BYB_VOUT,
			       byb_buff);
	if (rc <= 0) {
		if (rc != -ENOTSUPP)
			chg_err("can't get_byb_vout_info, rc=%d\n", rc);

	}

	return rc;
}

int oplus_wired_set_byb_vout_info(struct oplus_mms *topic, int byb_vout)
{
	int rc = 0;
	struct oplus_mms_wired *chip;

	if (topic == NULL) {
		chg_err("mms is NULL");
		return ID_MATCH_IGNORE;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (chip == NULL || chip->buck_ic == NULL) {
		chg_err("chip is NULL");
		return ID_MATCH_IGNORE;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_BUCK_SET_BYB_VOUT,
			       byb_vout);
	if (rc <= 0) {
		if (rc != -ENOTSUPP)
			chg_err("can't set_byb_vout_info, rc=%d\n", rc);
	}

	return rc;
}

int oplus_wired_set_chg_path(struct oplus_mms *topic, int path)
{
	int rc = 0;
	struct oplus_mms_wired *chip;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -ENODEV;
	}

	chip = oplus_mms_get_drvdata(topic);

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_BUCK_SET_CHG_PATH, path);
	if (rc < 0) {
		if (rc != -ENOTSUPP)
			chg_err("can't set chg path, rc=%d\n", rc);
	}

	return rc;
}

int oplus_wired_get_chg_path_status(struct oplus_mms *topic)
{
	int rc = 0;
	struct oplus_mms_wired *chip;
	int status = CHGP_STS_DEFAULT;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return CHGP_STS_DEFAULT;
	}

	chip = oplus_mms_get_drvdata(topic);

	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_BUCK_GET_CHG_PATH_STATUS, &status);
	if (rc < 0) {
		if (rc != -ENOTSUPP)
			chg_err("can't get chg path status, rc=%d\n", rc);
		return CHGP_STS_DEFAULT;
	}

	return status;
}

#define SHAFT_BTB_TEMP_DEFAULT 25
int oplus_wired_get_shaft_btb_temp(void)
{
	int rc = 0;
	int shaft_btb_temp = SHAFT_BTB_TEMP_DEFAULT;

	struct oplus_mms_wired *chip = g_mms_wired;

	if (!chip || !chip->buck_ic || (chip->shaft_btb_temp_threshod == 0)) {
		chg_err("chip is NULL!\n");
		return SHAFT_BTB_TEMP_DEFAULT;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_BUCK_GET_SHAFT_BATT_BTB_TEMP, &shaft_btb_temp);
	if (rc < 0) {
		if (rc != -ENOTSUPP)
			chg_err("get shaft_btb state error, rc=%d\n", rc);

		return SHAFT_BTB_TEMP_DEFAULT;
	}

	return shaft_btb_temp;
}

bool oplus_wired_get_shaft_btb_is_normal(void)
{
	struct oplus_mms_wired *chip = g_mms_wired;

	if (!chip || !chip->buck_ic || (chip->shaft_btb_temp_threshod == 0)) {
		return true;
	} else {
		if (oplus_wired_get_shaft_btb_temp() >= chip->shaft_btb_temp_threshod)
			return false;
		else
			return true;
	}
}

int oplus_wired_set_shaft_btb_over(bool is_shaft_btb_over)
{
	struct oplus_mms_wired *chip = g_mms_wired;
	int rc = 0;

	if (!chip || !chip->buck_ic || (chip->shaft_btb_temp_threshod == 0)) {
		return 0;
	}

	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_BUCK_PUSH_SHAFT_BATT_BTB_OVER, is_shaft_btb_over);
	if (rc < 0) {
		if (rc != -ENOTSUPP)
			chg_err("push shaft_btb state error, rc=%d\n", rc);
	}
	return rc;
}
int oplus_wired_push_info(struct oplus_mms *topic, const char *err_scene, const char *err_reason)
{
	struct oplus_mms_wired *chip;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -EINVAL;
	}
	if (!err_scene || !err_reason) {
		chg_err("err_scene or err_reason is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(topic);
	if (!is_err_topic_available(chip)) {
		chg_err("error topic not found\n");
		return -ENODEV;
	}

	oplus_chg_ic_creat_err_msg(chip->buck_ic, OPLUS_IC_ERR_BUCK_BOOST, 0,
		"$$err_scene@@%s$$err_reason@@%s", err_scene, err_reason);
	oplus_chg_ic_virq_trigger(chip->buck_ic, OPLUS_IC_VIRQ_ERR);

	return 0;
}

static void oplus_mms_wired_update(struct oplus_mms *mms, bool publish)
{
}

static struct mms_item oplus_mms_wired_item[] = {
	{
		.desc = {
			.item_id = WIRED_ITEM_PRESENT,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_wired_update_present,
		}
	},
	{
		.desc = {
			.item_id = WIRED_ITEM_ONLINE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_wired_update_online,
		}
	},
	{
		.desc = {
			.item_id = WIRED_ITEM_ERR_CODE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = WIRED_ITEM_CHG_TYPE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_wired_update_chg_type,
		}
	},
	{
		.desc = {
			.item_id = WIRED_ITEM_BC12_COMPLETED,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_wired_update_bc12_completed,
		}
	},
	{
		.desc = {
			.item_id = WIRED_ITEM_CC_MODE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_wired_update_cc_mode,
		}
	},
	{
		.desc = {
			.item_id = WIRED_ITEM_CC_DETECT,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_wired_update_cc_detect,
		}
	},
	{
		.desc = {
			.item_id = WIRED_ITEM_USB_STATUS,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_wired_update_usb_status,
		}
	},
	{
		.desc = {
			.item_id = WIRED_ITEM_USB_TEMP_VOLT_L,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_wired_update_usb_temp_volt_l,
		}
	},
	{
		.desc = {
			.item_id = WIRED_ITEM_USB_TEMP_VOLT_R,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_wired_update_usb_temp_volt_r,
		}
	},
	{
		.desc = {
			.item_id = WIRED_ITEM_USB_TEMP_L,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_wired_update_usb_temp_l,
		}
	},
	{
		.desc = {
			.item_id = WIRED_ITEM_USB_TEMP_R,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_wired_update_usb_temp_r,
		}
	},
	{
		.desc = {
			.item_id = WIRED_ITEM_PRE_OTG_ENABLE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_wired_update_pre_otg_enable_status,
		}
	},
	{
		.desc = {
			.item_id = WIRED_ITEM_OTG_ENABLE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_wired_update_otg_enable_status,
		}
	},
	{
		.desc = {
			.item_id = WIRED_ITEM_CHARGER_CURR_MAX,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_wired_update_chg_curr_max,
		}
	},
	{
		.desc = {
			.item_id = WIRED_ITEM_CHARGER_VOL_MAX,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_wired_update_chg_vol_max,
		}
	},
	{
		.desc = {
			.item_id = WIRED_ITEM_CHARGER_VOL_MIN,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_wired_update_chg_vol_min,
		}
	},
	{
		.desc = {
			.item_id = WIRED_TIME_ABNORMAL_ADAPTER,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_abnoramal_adapter,
		}
	},
	{
		.desc = {
			.item_id = WIRED_TIME_TYPEC_STATE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_get_typec_state,
		}
	},
	{
		.desc = {
			.item_id = WIRED_ITEM_REAL_CHG_TYPE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_wired_update_real_chg_type,
		}
	},
	{
		.desc = {
			.item_id = WIRED_ITEM_VBUS,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_wired_vbus,
		}
	},
	{
		.desc = {
			.item_id = WIRED_ITEM_ONLINE_STATUS_ERR,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = WIRED_ITEM_CHARGING_DISABLE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_wired_update_charging_disable,
		}
	},
	{
		.desc = {
			.item_id = WIRED_ITEM_COEXISTENCE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_wired_update_coexistence,
		}
	},
	{
		.desc = {
			.item_id = WIRED_ITEM_PD_COMPLETED,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_wired_update_pd_completed,
		}
	},
	{
		.desc = {
			.item_id = WIRED_ITEM_ICL_DONE_STATUS,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_wired_update_icl_status,
		}
	},
	{
		.desc = {
			.item_id = WIRED_ITEM_VBUS_VOL_TYPE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = WIRED_ITEM_BUCK_EIS_CURRENT_RATE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_wired_update_buck_eis_current_rate,
		}
	},
	{
		.desc = {
			.item_id = WIRED_ITEM_POWER_ROLE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_wired_update_power_role,
		}
	},
	{
		.desc = {
			.item_id = WIRED_ITEM_SOURCE_PDO_VOLT,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_wired_update_source_pdo_volt,
		}
	}
};

static const struct oplus_mms_desc oplus_mms_wired_desc = {
	.name = "wired",
	.type = OPLUS_MMS_TYPE_USB,
	.item_table = oplus_mms_wired_item,
	.item_num = ARRAY_SIZE(oplus_mms_wired_item),
	.update_items = NULL,
	.update_items_num = 0,
	.update_interval = 0, /* ms */
	.update = oplus_mms_wired_update,
};

static int oplus_mms_wired_topic_init(struct oplus_mms_wired *chip)
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

	chip->wired_online = oplus_wired_is_present();
	chip->wired_online_init_flag = chip->wired_online ? true : false;
	chip->wired_topic = devm_oplus_mms_register(chip->dev, &oplus_mms_wired_desc, &mms_cfg);
	if (IS_ERR(chip->wired_topic)) {
		chg_err("Couldn't register wired topic\n");
		rc = PTR_ERR(chip->wired_topic);
		return rc;
	}

	schedule_work(&chip->plugin_handler_work);
	schedule_work(&chip->chg_type_change_handler_work);

	return 0;
}

static bool oplus_mms_wired_parse_from_cmdline(struct oplus_mms_wired *chip)
{
	struct device_node *np;
	const char *bootparams = NULL;
	char *str;
	int ret = 0;
	struct oplus_usbtemp_spec_config *spec = &chip->usbtemp_spec;

	if (chip == NULL)
		return false;

	np = of_find_node_by_path("/chosen");
	if (np) {
		ret = of_property_read_string(np, "bootargs", &bootparams);
		if (!bootparams || ret < 0) {
			chg_err("failed to get bootargs property");
			return false;
		}

		str = strstr(bootparams, "support_hot_enter_kpoc=1");
		if (str) {
			spec->support_hot_enter_kpoc = true;
		} else {
			spec->support_hot_enter_kpoc = false;
		}
	}

	chg_err("support_hot_enter_kpoc=%d", spec->support_hot_enter_kpoc);
	return false;
}

static void oplus_mms_wired_otg_output_parse_dt(struct oplus_mms_wired *chip,
	struct device_node *node)
{
	struct oplus_usbtemp_spec_config *spec = &chip->usbtemp_spec;
	int rc = 0;
	int length = 0;

	rc = of_property_count_elems_of_size(node, "oplus_spec,otg_output", sizeof(u32));
	length = rc;
	if (length >= OTG_OUTPUT_LEVEL_MAX * 2 || length <= 0) {
		chg_err("length is too long or not valid, return");
		return;
	}

	rc = of_property_read_u32_array(node, "oplus_spec,otg_output",
		(u32 *)spec->otg_output, length);
}

static void oplus_mms_wired_usbtemp_v2_parse_dt(struct oplus_mms_wired *chip)
{
	struct oplus_usbtemp_spec_config *spec = &chip->usbtemp_spec;
	struct device_node *node = chip->dev->of_node;
	int rc;

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_batt_temp_low", &spec->usbtemp_batt_temp_low);
	if (rc)
		spec->usbtemp_batt_temp_low = OPCHG_USBTEMP_BATT_TEMP_LOW;

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_batt_temp_high", &spec->usbtemp_batt_temp_high);
	if (rc)
		spec->usbtemp_batt_temp_high = OPCHG_USBTEMP_BATT_TEMP_HIGH;

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_batt_temp_over_hot", &spec->usbtemp_batt_temp_over_hot);
	if (rc)
		spec->usbtemp_batt_temp_over_hot = OPCHG_USBTEMP_BATT_TEMP_OVER_HOT;

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_ntc_temp_low", &spec->usbtemp_ntc_temp_low);
	if (rc)
		spec->usbtemp_ntc_temp_low = OPCHG_USBTEMP_NTC_TEMP_LOW;

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_ntc_temp_high", &spec->usbtemp_ntc_temp_high);
	if (rc)
		spec->usbtemp_ntc_temp_high = OPCHG_USBTEMP_NTC_TEMP_HIGH;

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_temp_gap_low_with_batt_temp",
				  &spec->usbtemp_temp_gap_low_with_batt_temp);
	if (rc)
		spec->usbtemp_temp_gap_low_with_batt_temp = OPCHG_USBTEMP_GAP_LOW_WITH_BATT_TEMP;

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_temp_gap_with_batt_temp_in_over_hot",
				  &spec->usbtemp_temp_gap_with_batt_temp_in_over_hot);
	if (rc)
		spec->usbtemp_temp_gap_with_batt_temp_in_over_hot = OPCHG_USBTEMP_GAP_WITH_BATT_TEMP_IN_OVER_HOT;

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_temp_gap_high_with_batt_temp",
				  &spec->usbtemp_temp_gap_high_with_batt_temp);
	if (rc)
		spec->usbtemp_temp_gap_high_with_batt_temp = OPCHG_USBTEMP_GAP_HIGH_WITH_BATT_TEMP;

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_temp_gap_low_without_batt_temp",
				  &spec->usbtemp_temp_gap_low_without_batt_temp);
	if (rc)
		spec->usbtemp_temp_gap_low_without_batt_temp = OPCHG_USBTEMP_GAP_LOW_WITHOUT_BATT_TEMP;

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_temp_gap_high_without_batt_temp",
				  &spec->usbtemp_temp_gap_high_without_batt_temp);
	if (rc)
		spec->usbtemp_temp_gap_high_without_batt_temp = OPCHG_USBTEMP_GAP_HIGH_WITHOUT_BATT_TEMP;

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_rise_fast_temp_low", &spec->usbtemp_rise_fast_temp_low);
	if (rc)
		spec->usbtemp_rise_fast_temp_low = OPCHG_USBTEMP_RISE_FAST_TEMP_LOW;

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_rise_fast_temp_high", &spec->usbtemp_rise_fast_temp_high);
	if (rc)
		spec->usbtemp_rise_fast_temp_high = OPCHG_USBTEMP_RISE_FAST_TEMP_HIGH;

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_rise_fast_temp_count_low",
				  &spec->usbtemp_rise_fast_temp_count_low);
	if (rc)
		spec->usbtemp_rise_fast_temp_count_low = OPCHG_USBTEMP_RISE_FAST_TEMP_COUNT_LOW;

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_rise_fast_temp_count_high",
				  &spec->usbtemp_rise_fast_temp_count_high);
	if (rc)
		spec->usbtemp_rise_fast_temp_count_high = OPCHG_USBTEMP_RISE_FAST_TEMP_COUNT_HIGH;

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_cool_down_ntc_low", &spec->usbtemp_cool_down_ntc_low);
	if (rc)
		spec->usbtemp_cool_down_ntc_low = OPCHG_USBTEMP_COOL_DOWN_NTC_LOW;

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_cool_down_ntc_high", &spec->usbtemp_cool_down_ntc_high);
	if (rc)
		spec->usbtemp_cool_down_ntc_high = OPCHG_USBTEMP_COOL_DOWN_NTC_HIGH;

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_cool_down_gap_low", &spec->usbtemp_cool_down_gap_low);
	if (rc)
		spec->usbtemp_cool_down_gap_low = OPCHG_USBTEMP_COOL_DOWN_GAP_LOW;

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_cool_down_gap_high", &spec->usbtemp_cool_down_gap_high);
	if (rc)
		spec->usbtemp_cool_down_gap_high = OPCHG_USBTEMP_COOL_DOWN_GAP_HIGH;

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_cool_down_recover_ntc_low",
				  &spec->usbtemp_cool_down_recover_ntc_low);
	if (rc)
		spec->usbtemp_cool_down_recover_ntc_low = OPCHG_USBTEMP_COOL_DOWN_RECOVER_NTC_LOW;

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_cool_down_recover_ntc_high",
				  &spec->usbtemp_cool_down_recover_ntc_high);
	if (rc)
		spec->usbtemp_cool_down_recover_ntc_high = OPCHG_USBTEMP_COOL_DOWN_RECOVER_NTC_HIGH;

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_cool_down_recover_gap_low",
				  &spec->usbtemp_cool_down_recover_gap_low);
	if (rc)
		spec->usbtemp_cool_down_recover_gap_low = OPCHG_USBTEMP_COOL_DOWN_RECOVER_GAP_LOW;

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_cool_down_recover_gap_high",
				  &spec->usbtemp_cool_down_recover_gap_high);
	if (rc)
		spec->usbtemp_cool_down_recover_gap_high = OPCHG_USBTEMP_COOL_DOWN_RECOVER_GAP_HIGH;

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_otg_cc_boot_current_limit",
				  &spec->usbtemp_otg_cc_boot_current_limit);
	if (rc)
		spec->usbtemp_otg_cc_boot_current_limit = OPCHG_USBTEMP_OTG_CC_BOOT_CURRENT_LIMIT;

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_ntc_temp_l_compensation",
				  &spec->usbtemp_ntc_temp_l_compensation);
	if (rc)
		spec->usbtemp_ntc_temp_l_compensation = 0;

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_ntc_temp_r_compensation",
				  &spec->usbtemp_ntc_temp_r_compensation);
	if (rc)
		spec->usbtemp_ntc_temp_r_compensation = 0;

	oplus_mms_wired_otg_output_parse_dt(chip, node);
	oplus_mms_wired_parse_from_cmdline(chip);
}

static void oplus_mms_wired_parse_eis_params(struct oplus_mms_wired *chip)
{
	struct device_node *node = chip->dev->of_node;
	int rc;

	rc = of_property_read_u32(node, "oplus,series_batt_buck_eis_current_rate", &chip->buck_eis_current_rate);
	if (rc)
		chip->buck_eis_current_rate = 0;
	chg_info("<EIS>buck_eis_current_rate:%d", chip->buck_eis_current_rate);
}

static void oplus_mms_wired_parse_shaft_btb_dt(struct oplus_mms_wired *chip)
{
	int rc = 0;
	struct device_node *node = chip->dev->of_node;

	rc = of_property_read_u32(node, "oplus,shaft_btb_temp_threshod", &chip->shaft_btb_temp_threshod);
	if (rc)
		chip->shaft_btb_temp_threshod = 0;

	chg_info("shaft_btb_temp_threshod %d rc %d", chip->shaft_btb_temp_threshod, rc);
}

static void oplus_mms_wired_usbtemp_cool_down_parse_dt(struct oplus_mms_wired *chip)
{
	struct oplus_usbtemp_spec_config *spec = &chip->usbtemp_spec;
	struct device_node *node = chip->dev->of_node;
	int rc;

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_cool_down_temp_gap", &spec->usbtemp_cool_down_temp_gap);
	if (rc)
		spec->usbtemp_cool_down_temp_gap = 12;
	rc = of_property_read_u32(node, "oplus_spec,usbtemp_cool_down_recovery_temp_gap",
				  &spec->usbtemp_cool_down_recovery_temp_gap);
	if (rc)
		spec->usbtemp_cool_down_recovery_temp_gap = 6;
	rc = of_property_read_u32(node, "oplus_spec,usbtemp_cool_down_temp_first_gap",
				  &spec->usbtemp_cool_down_temp_first_gap);
	if (rc)
		spec->usbtemp_cool_down_temp_first_gap = 12;
	rc = of_property_read_u32(node, "oplus_spec,usbtemp_cool_down_recovery_temp_first_gap",
				  &spec->usbtemp_cool_down_recovery_temp_first_gap);
	if (rc)
		spec->usbtemp_cool_down_recovery_temp_first_gap = 6;
	rc = of_property_read_u32(node, "oplus_spec,usbtemp_cool_down_temp_second_gap",
				  &spec->usbtemp_cool_down_temp_second_gap);
	if (rc)
		spec->usbtemp_cool_down_temp_second_gap = 17;
	rc = of_property_read_u32(node, "oplus_spec,usbtemp_cool_down_recovery_temp_second_gap",
				  &spec->usbtemp_cool_down_recovery_temp_second_gap);
	if (rc)
		spec->usbtemp_cool_down_recovery_temp_second_gap = 11;

	chip->new_usbtemp_cool_down_support = of_property_read_bool(node, "oplus,new_usbtemp_cool_down_support");
}

static void oplus_mms_wired_usbtemp_parse_dt(struct oplus_mms_wired *chip)
{
	struct oplus_usbtemp_spec_config *spec = &chip->usbtemp_spec;
	struct device_node *node = chip->dev->of_node;
	int rc;

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_batttemp_gap", &spec->usbtemp_batttemp_gap);
	if (rc)
		spec->usbtemp_batttemp_gap = 18;

	oplus_mms_wired_usbtemp_cool_down_parse_dt(chip);

	rc = of_property_read_u32(node, "oplus_spec,usbtemp_batttemp_recover_gap", &spec->usbtemp_batttemp_recover_gap);
	if (rc)
		spec->usbtemp_batttemp_recover_gap = 6;
	rc = of_property_read_u32(node, "oplus_spec,usbtemp_batttemp_current_gap", &spec->usbtemp_batttemp_current_gap);
	if (rc)
		spec->usbtemp_batttemp_current_gap = 12;
	rc = of_property_read_u32(node, "oplus_spec,usbtemp_max_temp_diff", &spec->usbtemp_max_temp_diff);
	if (rc)
		spec->usbtemp_max_temp_diff = 7;
	rc = of_property_read_u32(node, "oplus_spec,usbtemp_max_temp_thr", &spec->usbtemp_max_temp_thr);
	if (rc)
		spec->usbtemp_max_temp_thr = 57;
	rc = of_property_read_u32(node, "oplus_spec,usbtemp_temp_up_time_thr", &spec->usbtemp_temp_up_time_thr);
	if (rc)
		spec->usbtemp_temp_up_time_thr = 30;
}

static void oplus_mms_wired_lpd_parse_dt(struct oplus_mms_wired *chip)
{
	struct device_node *node = chip->dev->of_node;
	struct oplus_lpd_spec_config *lpd_spec = &chip->lpd_spec;
	u32 buf[MAX_LPD_RANG_NUM] = {0};
	int rc;

	rc = of_property_read_u32(node, "oplus_spec,lpd_support_status", &lpd_spec->support_status);
	if (rc)
		lpd_spec->support_status = 0;

	rc = read_unsigned_data_from_node(node, "oplus_spec,lpd_sbu_thr_mv",
					  (u32 *)(buf),
					  MAX_LPD_RANG_NUM);
	if (rc < 0) {
		lpd_spec->sbu_rang.low_thr_mv = 0;
		lpd_spec->sbu_rang.high_thr_mv = 200;
	} else {
		lpd_spec->sbu_rang.low_thr_mv = buf[0];
		lpd_spec->sbu_rang.high_thr_mv = buf[1];
	}

	rc = read_unsigned_data_from_node(node, "oplus_spec,lpd_sbu_pullup_thr_mv",
					  (u32 *)(buf),
					  MAX_LPD_RANG_NUM);
	if (rc < 0) {
		lpd_spec->sbu_pullup_rang.low_thr_mv = 1180;
		lpd_spec->sbu_pullup_rang.high_thr_mv = 1780;
	} else {
		lpd_spec->sbu_pullup_rang.low_thr_mv = buf[0];
		lpd_spec->sbu_pullup_rang.high_thr_mv = buf[1];
	}

	rc = of_property_read_u32(node, "oplus_spec,lpd_sbu_ovp_thr_mv", &lpd_spec->lpd_sbu_ovp_thr_mv);
	if (rc)
		lpd_spec->lpd_sbu_ovp_thr_mv = 6500;

	chg_info("support_status:%d, rang[%dmv-%dmv], rang[%dmv-%dmv], sbu_ovp:%d", lpd_spec->support_status,
		lpd_spec->sbu_rang.low_thr_mv, lpd_spec->sbu_rang.high_thr_mv,
		lpd_spec->sbu_pullup_rang.low_thr_mv, lpd_spec->sbu_pullup_rang.high_thr_mv,
		lpd_spec->lpd_sbu_ovp_thr_mv);
}

static void oplus_mms_wired_parse_dt(struct oplus_mms_wired *chip)
{
	struct device_node *node = chip->dev->of_node;

	oplus_mms_wired_usbtemp_parse_dt(chip);
	chip->pre_ship_mode_support = of_property_read_bool(node, "oplus_spec,pre_ship_mode");
	chg_err("pre_ship_mode_support = %d\n", chip->pre_ship_mode_support);

	oplus_mms_wired_lpd_parse_dt(chip);

	chip->support_usbtemp_discharge_protect = of_property_read_bool(node, "oplus,support_usbtemp_discharge_protect");
	chg_info("support_usbtemp_discharge_protect=%d\n", chip->support_usbtemp_discharge_protect);
	chip->support_wlsotg_non_coexistence = of_property_read_bool(node, "oplus,support_wlsotg_non_coexistence");
	chip->support_usbtemp_protect_v2 = of_property_read_bool(node, "oplus,support_usbtemp_protect_v2");
	chg_info("support_usbtemp_protect_v2=%s, wlsotg_non_coexistence=%d\n",
		chip->support_usbtemp_protect_v2 ? "true" : "false", chip->support_wlsotg_non_coexistence);
	if (chip->support_usbtemp_protect_v2)
		oplus_mms_wired_usbtemp_v2_parse_dt(chip);
	chip->high_temp_scheme = of_property_read_bool(node, "high-temp-scheme");

	oplus_mms_wired_parse_eis_params(chip);
	chip->support_adjust_reverse_chg_cur = of_property_read_bool(node, "oplus,support_adjust_reverse_chg_cur");
	chg_info("support_adjust_reverse_chg_cur=%d\n", chip->support_adjust_reverse_chg_cur);
	oplus_mms_wired_parse_shaft_btb_dt(chip);
}

static int oplus_wired_otg_disable_vote_callback(struct votable *votable,
						 void *data, int disable,
						 const char *client,
						 bool step)
{
	struct oplus_mms_wired *chip = data;
	bool enable;
	int rc;

	if (disable < 0)
		chip->otg_disbale_status = false;
	else
		chip->otg_disbale_status = !!disable;
	chg_info("otg_disable vote client %s, status = %d\n", client, disable);


	rc = oplus_chg_ic_func(chip->buck_ic,
			       OPLUS_IC_FUNC_GET_OTG_ENABLE, &enable);
	if (rc < 0)
		enable = false;

	if (enable) {
		schedule_delayed_work(&chip->otg_enable_handler_work, 0);
		schedule_work(&chip->data_role_changed_handler_work);
	}

	return rc;
}

static int oplus_wired_flash_mode_vote_callback(struct votable *votable,
						 void *data, int enter,
						 const char *client,
						 bool step)
{
	bool enable;
	int rc;
	struct oplus_mms_wired *chip = data;

	if (enter < 0)
		enable = false;
	else
		enable = !!enter;

	chg_info("flash_mode vote client %s, enter = %d\n", client, enter);
	rc = oplus_chg_ic_func(chip->buck_ic, OPLUS_IC_FUNC_BUCK_SET_FLASH_MODE, enable);
	if (rc < 0 && rc != -ENOTSUPP)
		chg_err("can't enter flash mode, rc=%d\n", rc);
	return rc;
}

static int oplus_mms_wired_votable_init(struct oplus_mms_wired *chip)
{
	int rc;

	chip->otg_disable_votable =
		create_votable("OTG_DISABLE", VOTE_SET_ANY,
			       oplus_wired_otg_disable_vote_callback, chip);
	if (IS_ERR(chip->otg_disable_votable)) {
		rc = PTR_ERR(chip->otg_disable_votable);
		chg_err("otg_disable_votable create error, rc=%d\n", rc);
		chip->otg_disable_votable = NULL;
		return rc;
	}
	vote(chip->otg_disable_votable, DEF_VOTER, false, false, false);

	chip->flash_mode_votable =
		create_votable("FLASH_MODE", VOTE_SET_ANY,
			       oplus_wired_flash_mode_vote_callback, chip);
	if (IS_ERR(chip->flash_mode_votable)) {
		rc = PTR_ERR(chip->flash_mode_votable);
		chg_err("flash_mode_votable create error, rc=%d\n", rc);
		chip->flash_mode_votable = NULL;
		goto creat_flash_mode_votable_err;
	}

	return 0;

creat_flash_mode_votable_err:
	destroy_votable(chip->otg_disable_votable);
	return rc;
}

static void oplus_mms_wired_value_init(struct oplus_mms_wired *chip)
{
	oplus_wired_clear_sbu_info(chip);
	chip->dischg_flag = false;
	chip->cpa_support = oplus_cpa_support();
	chip->set_icl_done = false;
	chip->gauge_not_ready = false;

	schedule_delayed_work(&chip->mms_wired_init_work, 0);

	mutex_init(&chip->bcc_curr_done_mutex);
	oplus_mms_wired_bcc_parms_reset(chip);
}

static void oplus_mms_wired_work_init(struct oplus_mms_wired *chip)
{
	INIT_DELAYED_WORK(&chip->mms_wired_init_work,
			  oplus_mms_wired_init_work);
	INIT_DELAYED_WORK(&chip->usbtemp_recover_work, oplus_usbtemp_recover_work);
	INIT_DELAYED_WORK(&chip->ccdetect_work, oplus_mms_wired_ccdetect_work);
	INIT_DELAYED_WORK(&chip->typec_state_notify_work, oplus_mms_wired_typec_state_notify_work);
	INIT_DELAYED_WORK(&chip->typec_state_change_work, oplus_mms_wired_typec_state_change_work);
	INIT_DELAYED_WORK(&chip->power_role_change_work, oplus_mms_wired_power_role_change_work);
	INIT_DELAYED_WORK(&chip->svid_handler_work, oplus_mms_wired_svid_handler_work);
	INIT_DELAYED_WORK(&chip->wam.online_status_err_work, oplus_mms_wired_online_status_err_work);
	INIT_DELAYED_WORK(&chip->power_changed_work, oplus_mms_wired_power_changed_work);
	INIT_WORK(&chip->err_handler_work, oplus_mms_wired_err_handler_work);
	INIT_WORK(&chip->plugin_handler_work,
		  oplus_mms_wired_plugin_handler_work);
	INIT_WORK(&chip->chg_type_change_handler_work,
		  oplus_mms_wired_chg_type_change_handler_work);
	INIT_WORK(&chip->cpa_chg_type_change_handler_work,
		  oplus_mms_wired_cpa_chg_type_change_handler_work);
	INIT_WORK(&chip->gauge_update_work, oplus_wired_gauge_update_work);
	INIT_DELAYED_WORK(&chip->otg_enable_handler_work, oplus_wired_otg_enable_handler_work);
	INIT_DELAYED_WORK(&chip->lpd_info_update_work, oplus_wired_lpd_info_update_work);
	INIT_DELAYED_WORK(&chip->voocphy_dischg_work, oplus_voocphy_dischg_work);
	INIT_WORK(&chip->voltage_change_work, oplus_wired_voltage_change_work);
	INIT_WORK(&chip->current_change_work, oplus_wired_current_change_work);
	INIT_WORK(&chip->bc12_completed_work, oplus_wired_bc12_completed_work);
	INIT_WORK(&chip->data_role_changed_handler_work,
		  oplus_wired_data_role_changed_handler_work);
	INIT_WORK(&chip->back_ui_soc_work, oplus_back_ui_soc_work);
	INIT_WORK(&chip->otg_enable_pending_work, oplus_wired_otg_enable_pending_work);
	INIT_WORK(&chip->wls_upgrading_work, oplus_wls_upgrading_work);
	INIT_WORK(&chip->pd_completed_handler_work, oplus_wired_pd_completed_work);
	INIT_WORK(&chip->icl_done_handler_work, oplus_wired_icl_done_work);
	INIT_WORK(&chip->reverse_usbtemp_check_work, oplus_reverse_usbtemp_check_work);
}

static int oplus_mms_wired_probe(struct platform_device *pdev)
{
	struct oplus_mms_wired *chip;
	struct oplus_usbtemp_spec_config *spec;
	const char *name;
	int rc, i;

	chip = devm_kzalloc(&pdev->dev, sizeof(struct oplus_mms_wired),
			    GFP_KERNEL);
	if (chip == NULL) {
		chg_err("alloc memory error\n");
		return -ENOMEM;
	}
	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);
	spec = &chip->usbtemp_spec;

	of_platform_populate(chip->dev->of_node, NULL, NULL, chip->dev);
	oplus_mms_wired_parse_dt(chip);
	rc = oplus_mms_wired_votable_init(chip);
	if (rc < 0)
		goto votable_init_err;

	rc = of_property_read_string(chip->dev->of_node, "oplus,adc_info_name", &name);
	if (rc < 0) {
		chg_err("read oplus,adc_info_name error, rc=%d\n", rc);
		name = "default";
	}
	for (i = 0; i < ARRAY_SIZE(adc_vol_temp_info_table); i++) {
		if (strcmp(adc_vol_temp_info_table[i].name, name) == 0) {
			chip->adc_vol_temp_info = &adc_vol_temp_info_table[i];
		}
	}
	if (chip->adc_vol_temp_info == NULL) {
		chg_err("%s adc_vol_temp_info not found\n", name);
		chip->adc_vol_temp_info = &adc_vol_temp_info_table[0];
	}

	/* spinlock must be initialized before work_init, as voocphy_dischg_work handler uses this lock */
	spin_lock_init(&chip->voocphy_dischg_enable_lock);
	oplus_mms_wired_work_init(chip);
	oplus_mms_wired_value_init(chip);
	chg_info("probe success\n");
	return 0;

votable_init_err:
	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, chip);
	return rc;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void oplus_mms_wired_remove(struct platform_device *pdev)
#else
static int oplus_mms_wired_remove(struct platform_device *pdev)
#endif
{
	struct oplus_mms_wired *chip = platform_get_drvdata(pdev);

	if (!IS_ERR_OR_NULL(chip->gauge_subs))
		oplus_mms_unsubscribe(chip->gauge_subs);
	if (chip->flash_mode_votable != NULL)
		destroy_votable(chip->flash_mode_votable);
	if (chip->otg_disable_votable != NULL)
		destroy_votable(chip->otg_disable_votable);
	cancel_delayed_work_sync(&chip->voocphy_dischg_work);
	devm_kfree(&pdev->dev, chip);
	platform_set_drvdata(pdev, NULL);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return 0;
#endif
}

static void oplus_mms_wired_shutdown(struct platform_device *pdev)
{
	struct oplus_mms_wired *chip = platform_get_drvdata(pdev);
	int rc;

	cancel_delayed_work_sync(&chip->voocphy_dischg_work);
	if (chip->ship_mode && chip->pre_ship_mode_support == 0) {
		rc = oplus_chg_ic_func(chip->buck_ic,
				       OPLUS_IC_FUNC_BUCK_SHIPMODE_ENABLE,
				       true);
		if (rc < 0)
			chg_err("can't enable ship mode, rc=%d\n", rc);
		else
			chg_err("enable ship mode\n");
		msleep(1000);
	}
}

static const struct of_device_id oplus_mms_wired_match[] = {
	{ .compatible = "oplus,mms_wired" },
	{},
};

static struct platform_driver oplus_mms_wired_driver = {
	.driver		= {
		.name = "oplus-mms_wired",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(oplus_mms_wired_match),
	},
	.probe		= oplus_mms_wired_probe,
	.remove		= oplus_mms_wired_remove,
	.shutdown	= oplus_mms_wired_shutdown,
};

static __init int oplus_mms_wired_init(void)
{
	return platform_driver_register(&oplus_mms_wired_driver);
}

static __exit void oplus_mms_wired_exit(void)
{
	platform_driver_unregister(&oplus_mms_wired_driver);
}

oplus_chg_module_register(oplus_mms_wired);
