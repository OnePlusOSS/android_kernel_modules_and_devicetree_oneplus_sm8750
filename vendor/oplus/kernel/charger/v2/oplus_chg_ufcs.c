// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023-2023 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[UFCS]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/completion.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/sched/clock.h>
#include <linux/platform_device.h>

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
#include <oplus_chg_ufcs.h>
#include <oplus_batt_bal.h>
#include <ufcs_class.h>
#include <oplus_chg_state_retention.h>
#include "plat_ufcs/plat_ufcs_notify.h"
#include <oplus_chg_plc.h>

#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
#include "oplus_cfg.h"
#endif

#define UFCS_MONITOR_CYCLE_MS		1500
#define UFCS_WATCHDOG_TIME_MS		3000

#define UFCS_START_DEF_VOL_MV		5000
#define UFCS_START_MAX_VOL_MV		5500
#define UFCS_MONITOR_TIME_MS		500
#define UFCS_WD_TIME_MS			500
#define UFCS_STOP_DELAY_TIME		300
#define UFCS_TEMP_SWITCH_DELAY		100
#define UFCS_ADSP_TEMP_SWITCH_DELAY	600
#define UFCS_PRELIMINARY_IMP_TIME_MS	200
#define UFCS_RESTART_TIMEOUT_MS	10000
#define UFCS_RESET_MS			5000
#define UFCS_START_DEF_CURR_MA_OPLUS	800
#define UFCS_START_DEF_CURR_MA_THIRD	1000

#define UFCS_START_DEF_CURR_MA		2000
#define UFCS_START_CP_CURR_MA		1000

#define UFCS_WAIT_CURR_DOWN_TIMES	200
#define UFCS_QUIT_CP_OVER_CURR		4000

#define UFCS_EIS_VOL_MV			9000
#define UFCS_EIS_CURR_MA		2000
#define UFCS_EIS_MONITOR_TIME_MS	50
#define UFCS_EIS_COMMU_TIME_MS          500
#define UFCS_EIS_COMMU_DELAY_CNT        (UFCS_EIS_COMMU_TIME_MS / UFCS_EIS_MONITOR_TIME_MS)

#define UFCS_UPDATE_PDO_TIME		5
#define UFCS_UPDATE_FASTCHG_TIME	1
#define UFCS_UPDATE_TEMP_TIME		1
#define UFCS_UPDATE_IBAT_TIME		1

#define UFCS_TEMP_OVER_COUNTS		2
#define UFCS_TEMP_WARM_RANGE_THD	10
#define UFCS_TEMP_LOW_RANGE_THD		10
#define UFCS_COLD_TEMP_RANGE_THD	10

#define UFCS_R_AVG_NUM			10
#define UFCS_R_ROW_NUM			7

#define FULL_UFCS_SYS_MAX		6

#define UFCS_BCC_CURRENT_MIN		(1000 / 100)
#define UFCS_BATT_CURR_TO_BCC_CURR	100

#define UFCS_VERIFY_CURR_THR_MA		4000
#define UFCS_BOOT_RETRY_TIME_MAX_SECOND	12

#define UFCS_PRELIMINARY_IMP_CHECK_CURR	4000
#define SUBBOARD_TEMP_ABNORMAL_MAX_CURR	7300

#define UFCS_RESETADAPTER_SLEEP_1000MS	1000
#define UFCS_BOOT_RESETADAPTER_20S	20

#define UFCS_BTB_TEMP_MAX		80
#define UFCS_USB_TEMP_MAX		80
#define UFCS_CP_TEMP_MAX		80
#define BTB_TEMP_OVER_MAX_INPUT_CUR	1000
#define HANDSHAKE_ERROR_COUNT_LEVEL	2
#define CONNECT_ERROR_COUNT_LEVEL	3
#define CONNECT_ERROR_COUNT_MAX_LEVEL	15
#define SUPPORT_THIRD_UFCS_POWER	33000
#define WAIT_BC1P2_GET_TYPE		600
#define UFCS_CP_WATCHDOG_TIMEOUT_MS	5000
#define UFCS_CP_WATCHDOG_DISABLE	0

#define UFCS_POWER(v_mv, i_ma)		(v_mv * i_ma / 1000 / 1000 * 1000)

enum {
	UFCS_BAT_TEMP_NATURAL = 0,
	UFCS_BAT_TEMP_HIGH0,
	UFCS_BAT_TEMP_HIGH1,
	UFCS_BAT_TEMP_HIGH2,
	UFCS_BAT_TEMP_HIGH3,
	UFCS_BAT_TEMP_HIGH4,
	UFCS_BAT_TEMP_HIGH5,
	UFCS_BAT_TEMP_LOW0,
	UFCS_BAT_TEMP_LOW1,
	UFCS_BAT_TEMP_LOW2,
	UFCS_BAT_TEMP_LITTLE_COOL,
	UFCS_BAT_TEMP_LITTLE_COOL_HIGH,
	UFCS_BAT_TEMP_COOL,
	UFCS_BAT_TEMP_NORMAL_LOW_PRE,
	UFCS_BAT_TEMP_NORMAL_LOW,
	UFCS_BAT_TEMP_NORMAL_HIGH,
	UFCS_BAT_TEMP_LITTLE_COLD,
	UFCS_BAT_TEMP_WARM,
	UFCS_BAT_TEMP_EXIT,
	UFCS_BAT_TEMP_SWITCH_CURVE,
};

enum {
	UFCS_TEMP_RANGE_INIT = 0,
	UFCS_TEMP_RANGE_LITTLE_COLD, /* 0 ~ 5 */
	UFCS_TEMP_RANGE_COOL, /* 5 ~ 12 */
	UFCS_TEMP_RANGE_LITTLE_COOL, /* 12~20 */
	UFCS_TEMP_RANGE_LITTLE_COOL_HIGH,
	UFCS_TEMP_RANGE_NORMAL_LOW_PRE,
	UFCS_TEMP_RANGE_NORMAL_LOW, /* 20~35 */
	UFCS_TEMP_RANGE_NORMAL_HIGH, /* 35~44 */
	UFCS_TEMP_RANGE_WARM, /* 44-52 */
	UFCS_TEMP_RANGE_NORMAL,
};

enum {
	UFCS_LOW_CURR_FULL_CURVE_TEMP_LITTLE_COOL,
	UFCS_LOW_CURR_FULL_CURVE_TEMP_NORMAL_LOW,
	UFCS_LOW_CURR_FULL_CURVE_TEMP_NORMAL_HIGH,
	UFCS_LOW_CURR_FULL_CURVE_TEMP_MAX,
};

enum ufcs_emark_power {
	UFCS_EMARK_POWER_DEFAULT = 4000,
	UFCS_EMARK_POWER_THIRD = 5000,
	UFCS_EMARK_POWER_V0 = 6500,
	UFCS_EMARK_POWER_V1 = 10000,
	UFCS_EMARK_POWER_V2 = 12000,
	UFCS_EMARK_POWER_V3 = 13700,
	UFCS_EMARK_POWER_V4 = 15000,
	UFCS_EMARK_POWER_V5 = 20000,
	UFCS_EMARK_POWER_V6 = 24000,
	UFCS_EMARK_POWER_MAX = 30000,
};

enum ufcs_power_imax {
	UFCS_CURR_MAX_V0 = 3000,
	UFCS_CURR_MAX_V1 = 4100,
	UFCS_CURR_MAX_V2 = 5000,
	UFCS_CURR_MAX_V3 = 6100,
	UFCS_CURR_MAX_V4 = 7300,
	UFCS_CURR_MAX_V5 = 8300,
	UFCS_CURR_MAX_V6 = 10000,
	UFCS_CURR_MAX_V7 = 12000,
	UFCS_CURR_MAX_V8 = 15000,
	UFCS_CURR_MAX_V9 = 20000,
	UFCS_CURR_MAX_OTHER = 24000,
};


enum ufcs_power_vmax {
	UFCS_VOL_MAX_V0 = 5000,
	UFCS_VOL_MAX_V1 = 11000,
	UFCS_VOL_MAX_V2 = 20000,
	UFCS_VOL_MAX_V3 = 30000,
	UFCS_VOL_MAX_OTHER = 40000,
};

struct ufcs_power_param {
	enum ufcs_power_vmax vmax;
	enum ufcs_power_imax imax;
	enum ufcs_power_type power;
};

enum ufcs_emark_id {
	UFCS_EMARK_ID_V0,
	UFCS_EMARK_ID_V1 = 6,
	UFCS_EMARK_ID_V2 = 7,
	UFCS_EMARK_ID_V3 = 8,
	UFCS_EMARK_ID_V4 = 9,
	UFCS_EMARK_ID_V5 = 0x0A,
	UFCS_EMARK_ID_V6 = 0x0B,
	UFCS_EMARK_ID_VA = 0xAA,
	UFCS_EMARK_ID_MAX = 0xFF,
};

static struct ufcs_power_param ufcs_power_table[] = {
	{ UFCS_VOL_MAX_V0, UFCS_CURR_MAX_V0, UFCS_POWER_TYPE_UNKOWN },
	{ UFCS_VOL_MAX_V0, UFCS_CURR_MAX_V0, UFCS_POWER_TYPE_V0 },
	{ UFCS_VOL_MAX_V1, UFCS_CURR_MAX_V0, UFCS_POWER_TYPE_V0 },
	{ UFCS_VOL_MAX_V1, UFCS_CURR_MAX_V1, UFCS_POWER_TYPE_V1 },
	{ UFCS_VOL_MAX_V1, UFCS_CURR_MAX_V2, UFCS_POWER_TYPE_V2 },
	{ UFCS_VOL_MAX_V1, UFCS_CURR_MAX_V3, UFCS_POWER_TYPE_V3 },
	{ UFCS_VOL_MAX_V1, UFCS_CURR_MAX_V4, UFCS_POWER_TYPE_V4 },
	{ UFCS_VOL_MAX_V1, UFCS_CURR_MAX_V5, UFCS_POWER_TYPE_V5 },
	{ UFCS_VOL_MAX_V1, UFCS_CURR_MAX_V6, UFCS_POWER_TYPE_V6 },
	{ UFCS_VOL_MAX_V1, UFCS_CURR_MAX_V7, UFCS_POWER_TYPE_V7 },
	{ UFCS_VOL_MAX_V1, UFCS_CURR_MAX_V8, UFCS_POWER_TYPE_V8 },
	{ UFCS_VOL_MAX_V1, UFCS_CURR_MAX_V9, UFCS_POWER_TYPE_V9 },
	{ UFCS_VOL_MAX_V2, UFCS_CURR_MAX_V0, UFCS_POWER_TYPE_V0 },
	{ UFCS_VOL_MAX_V3, UFCS_CURR_MAX_V0, UFCS_POWER_TYPE_V0 },
};

struct ufcs_abnormal_adapter_struct {
	u16 sw_ver;
	u16 hw_ver;
	u16 ic_vendor;
	u16 dev_vendor;
};

struct oplus_ufcs_config {
	unsigned int target_vbus_mv;
	int curr_max_ma;
	bool adsp_ufcs_project;
	bool ufcs_need_reset_adapter;
	bool ufcs_wd_separate_work;
	int ufcs_boot_time_retry;
	uint8_t *curve_strategy_name;
	unsigned int high_imp_compensation_setting_mv;
	unsigned int upper_compensation_mv;
	int curr_max_ma_percent_75;
	int curr_max_ma_percent_85;
	int ufcs_watt_third;
	int ufcs_full_recheck_temp;
};

struct oplus_ufcs_timer {
	struct timespec pdo_timer;
	struct timespec fastchg_timer;
	struct timespec temp_timer;
	unsigned long ibat_jiffies;
	int ufcs_max_time_ms;
	unsigned long monitor_jiffies;
	unsigned long monitor_last_run_jiffies;
};

struct ufcs_protection_counts {
	int cool_fw;
	int warm_fw;
	int sw_full;
	int hw_full;
	int low_curr_full;
	int ibat_low;
	int ibat_high;
	int btb_high;
	int tbatt_over;
	int tfg_over;
	int output_low;
	int ibus_over;
	int cool_full_fw;
};

struct oplus_ufcs_limits {
	int default_ufcs_normal_high_temp;
	int default_ufcs_little_cool_temp;
	int default_ufcs_little_cool_high_temp;
	int default_ufcs_cool_temp;
	int default_ufcs_little_cold_temp;
	int default_ufcs_normal_low_pre_temp;
	int default_ufcs_normal_low_temp;
	int ufcs_warm_allow_vol;
	int ufcs_warm_allow_soc;

	int ufcs_batt_over_low_temp;
	int ufcs_little_cold_temp;
	int ufcs_cool_temp;
	int ufcs_little_cool_temp;
	int ufcs_little_cool_high_temp;
	int ufcs_normal_low_pre_temp;
	int ufcs_normal_low_temp;
	int ufcs_normal_high_temp;
	int ufcs_batt_over_high_temp;
	int ufcs_strategy_temp_num;

	int ufcs_strategy_normal_current;
	int ufcs_strategy_batt_high_temp0;
	int ufcs_strategy_batt_high_temp1;
	int ufcs_strategy_batt_high_temp2;
	int ufcs_strategy_batt_low_temp2;
	int ufcs_strategy_batt_low_temp1;
	int ufcs_strategy_batt_low_temp0;
	int ufcs_strategy_high_current0;
	int ufcs_strategy_high_current1;
	int ufcs_strategy_high_current2;
	int ufcs_strategy_low_current2;
	int ufcs_strategy_low_current1;
	int ufcs_strategy_low_current0;

	int ufcs_low_curr_full_cool_temp;
	int ufcs_low_curr_full_little_cool_temp;
	int ufcs_low_curr_full_normal_low_temp;
	int ufcs_low_curr_full_normal_high_temp;

	int ufcs_over_high_or_low_current;
	int ufcs_strategy_change_count;
	int ufcs_full_cool_sw_vbat;
	int ufcs_full_normal_sw_vbat;
	int ufcs_full_normal_hw_vbat;
	int ufcs_full_warm_vbat;
	int ufcs_full_cool_sw_vbat_third;
	int ufcs_full_normal_sw_vbat_third;
	int ufcs_full_normal_hw_vbat_third;
	int ufcs_timeout_third;
	int ufcs_timeout_oplus;
	int ufcs_ibat_over_third;
	int ufcs_ibat_over_oplus;
	int ufcs_ibus_over_oplus;

	int ufcs_low_temp;
	int ufcs_high_temp;
	int ufcs_low_soc;
	int ufcs_high_soc;
	int ufcs_high_soc_third;
	int ufcs_removed_bat_decidegc;
};

struct ufcs_full_curve {
	unsigned int iterm;
	unsigned int vterm;
	bool exit;
};

struct ufcs_full_curves_temp {
	struct ufcs_full_curve full_curves[FULL_UFCS_SYS_MAX];
	int full_curve_num;
};

#define UFCS_CMD_BUF_SIZE		128
#define UFCS_GET_AUTH_DATA_TIMEOUT_MS	3000000 /* wait for the daemon to start */
enum ufcs_dev_cmd_type {
	UFCS_DEV_CMD_EXIT,
	UFCS_DEV_CMD_GET_AUTH_DATA,
};

enum exit_ufcs_flag_status {
	EXIT_UFCS_FALSE,
	EXIT_HIGH_UFCS,
	EXIT_THIRD_UFCS,
};

struct ufcs_dev_cmd {
	unsigned int cmd;
	unsigned int data_size;
	unsigned char data_buf[UFCS_CMD_BUF_SIZE];
};

struct ufcs_bcc_info {
	int bcc_min_curr;
	int bcc_max_curr;
	int bcc_exit_curr;
};

#define UFCS_PR_START_DELAY_MS		200
enum ufcs_pr_state {
	UFCS_PR_IDLE,
	UFCS_PR_RISING,
	UFCS_PR_FALLING,
	UFCS_PR_CC,
	UFCS_PR_CV,
	UFCS_PR_FULL,
	UFCS_PR_PDO_ERR,
	UFCS_PR_MAX,
};

struct ufcs_pr_config {
	bool support_pr;
	int pr_r_input_mohm;
	int pr_r_btb_mohm;
};

struct ufcs_pr_data {
	enum ufcs_pr_state curr_state;
	enum ufcs_pr_state prev_state;

	struct puc_strategy_temp_curves curve;

	bool adjust;
	int adjust_rising_cnt;
	int adjust_rising_adapter_vbus_mv;
	int adjust_rising_req_vbus_mv;
	unsigned long adjust_jiffies;

	int emark_current_ma;
	int adapter_current_ma;
	int pdo_current_ma;
	int curve_current_ma;
	int plc_current_ma;
	int falling_current_ma;
	int err_current_ma;
	int max_current_ma;

	int request_voltage_mv;
	int request_current_ma;

	int target_current_ma;

	int adapter_vbus_mv;
	int adapter_ibus_ma;
	int ibat_ma;
	int vbat_mv;
	int pmic_vbat_mv;
	int iterm_ma;
	int pmic_vbus_mv;

	int vbat_full_mv;
	int vbat_full_comp_mv;
	int vbat_target_mv;
	int vbat_target_mv_by_ibus;
	int ibat_target_ma;
	int bcc_target_ma;
	unsigned long bcc_target_update_jiffies;
	int ibus_target_ma;
	int ibus_to_ibat_ratio;
	int vbus_to_vbat_ratio;
	int next_ibus_target_ma;

	int falling_cc_adjust;

	int vbus_max_mv;
	int req_vbus_max_mv;

	int cc_cnt;

	bool cv_curve;
	bool last_curve;
	int cv_full_cnt;
	int cv_iterm_cnt;
	int cv_target_cnt;
	int cv_delta_cnt;
	int cv_ibus_cnt;
	bool full;
	unsigned long entry_cv_jiffies;
	unsigned long cv_ibus_larget_jiffies;

	bool pdo_err;
	int pdo_err_high;
	int pdo_err_low;
};

struct oplus_ufcs {
	struct device *dev;
	struct oplus_mms *ufcs_topic;
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
	struct oplus_mms *err_topic;
	struct oplus_mms *retention_topic;
	struct mms_subscribe *retention_subs;
	struct oplus_mms *plc_topic;
	struct mms_subscribe *plc_subs;
	struct oplus_mms *keep_topic;
	struct oplus_chg_ic_dev *ufcs_ic;
	struct oplus_chg_ic_dev *cp_ic;
	struct oplus_chg_ic_dev *dpdm_switch;

	struct notifier_block nb;

	struct votable *ufcs_curr_votable;
	struct votable *ufcs_disable_votable;
	struct votable *ufcs_not_allow_votable;
	struct votable *ufcs_boot_votable;
	struct votable *wired_suspend_votable;
	struct votable *wired_icl_votable;
	struct votable *chg_disable_votable;

	struct delayed_work switch_check_work;
	struct delayed_work monitor_work;
	struct delayed_work current_work;
	struct delayed_work imp_uint_init_work;
	struct delayed_work wait_auth_data_work;
	struct delayed_work ufcs_restart_timeout_work;
	struct delayed_work ufcs_eis_work;
	struct delayed_work send_authdata_to_adsp_work;
	struct delayed_work switch_end_recheck_work;
	struct delayed_work ufcs_subsys_reset_work;
	struct delayed_work watchdog_work;

	struct work_struct wired_online_work;
	struct work_struct force_exit_work;
	struct work_struct soft_exit_work;
	struct work_struct gauge_update_work;
	struct work_struct err_flag_push_work;
	struct work_struct fifo_overflow_push_work;
	struct work_struct plat_notify_exit_work;
	struct work_struct ufcs_restart_work;
	struct work_struct retention_disconnect_work;
	struct work_struct handshake_error_work;
	struct work_struct fgreset_ufcs_retry_work;
	struct work_struct cp_err_handler_work;
	struct work_struct cp_online_handler_work;
	struct work_struct cp_offline_handler_work;
	struct work_struct set_fcs_icl_work;

	wait_queue_head_t read_wq;
	struct miscdevice misc_dev;
	struct mutex read_lock;
	struct mutex cmd_data_lock;
	struct completion cmd_ack;
	struct ufcs_dev_cmd cmd;
	bool cmd_data_ok;
	char auth_data[UFCS_VERIFY_AUTH_DATA_SIZE];
	bool auth_data_ok;
	bool wait_auth_data_done;

	struct oplus_chg_strategy *oplus_curve_strategy;
	struct oplus_chg_strategy *third_curve_strategy;
	struct oplus_chg_strategy *strategy;
	struct oplus_chg_strategy *vfa_strategy;
	struct oplus_chg_strategy *ccd_strategy;
	struct mutex ccd_lock;

	struct oplus_chg_strategy **oplus_lcf_strategy;
	struct oplus_chg_strategy **third_lcf_strategy;
	int oplus_lcf_num;
	int third_lcf_num;

	struct oplus_chg_strategy *temperature_strategy;
	uint8_t *temperature_strategy_name;
	uint8_t *temperature_strategy_data;
	uint32_t temperature_strategy_data_size;

	struct oplus_impedance_node *input_imp_node;
	struct oplus_impedance_unit *imp_uint;

	struct oplus_plc_protocol *opp;

#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
	struct oplus_cfg debug_cfg;
#endif

	struct oplus_ufcs_config config;
	struct oplus_ufcs_timer timer;
	struct ufcs_protection_counts count;
	struct oplus_ufcs_limits limits;
	struct ufcs_bcc_info bcc;

	struct ufcs_full_curves_temp low_curr_full_curves_temp[UFCS_LOW_CURR_FULL_CURVE_TEMP_MAX];

	u64 dev_info;
	u64 src_info;
	u64 cable_info;
	u64 emark_info;
	u64 pdo[UFCS_OUTPUT_MODE_MAX];
	int pdo_num;
	u64 pie[UFCS_OPLUS_VND_POWER_INFO_MAX];
	int pie_num;
	unsigned int err_flag;
	bool ufcs_online;
	bool ufcs_charging;
	bool handshake_ok;
	bool adapter_check_third_ufcs;
	int retention_ufcs_power;
	bool oplus_ufcs_adapter;
	bool ufcs_disable;
	bool ufcs_not_allow;
	enum oplus_cp_work_mode cp_work_mode;
	bool oplus_cp_ucp_disable;
	int emark_imax;
	int power_imax;
	int allow_check_soc;

	int last_target_curr_ma;
	int target_curr_ma;
	int target_vbus_mv;
	int curr_set_ma;
	int curr_cc;
	int vol_set_mv;
	int bcc_max_curr;
	int bcc_min_curr;
	int bcc_exit_curr;
	int rmos_mohm;
	int cool_down;
	int curr_table_type;
	u32 adapter_id;
	int start_retry_count;
	bool start_check;
	int cp_ratio;

	bool wired_online;
	bool batt_hmac;
	bool batt_auth;
	bool irq_plugin;
	bool disconnect_change;
	bool retention_state;
	bool retention_oplus_adapter;
	bool retention_state_ready;
	int connect_error_count;
	int connect_error_count_level;
	int ufcs_handshake_error_count;
	bool adapter_verify_fail_flag;
	enum exit_ufcs_flag_status retention_exit_ufcs_flag;
	enum oplus_chg_protocol_type cpa_current_type;

	int high_soc;
	int ui_soc;
	int shell_temp;
	enum oplus_temp_region batt_temp_region;
	bool shell_temp_ready;
	int adapter_max_curr;

	int ufcs_fastchg_batt_temp_status;
	int ufcs_temp_cur_range;
	int temp_region_cnt;
	int ufcs_cool_full_temp_range;
	int ufcs_low_curr_full_temp_status;
	int batt_bal_curr_limit;
	int preliminary_imp_check_cnt;
	bool need_preliminary_imp_check;
	atomic_t temp_recover_debounce_thd;

	bool led_on;
	struct ufcs_pr_config pr_config;
	struct ufcs_pr_data pr_data;

	bool reset_adapter;
	struct completion reset_abnormal_ack;
	bool slow_chg_enable;
	int slow_chg_pct;
	int slow_chg_watt;
	int slow_chg_batt_limit;
	int startup_retry_times;
	bool chg_ctrl_by_sale_mode;
	int subboard_ntc_abnormal_current;
	bool prot_crash;
	bool subsys_reset_ufcs;
	int eis_status;
	bool lift_vbus_use_cpvout;
	bool ufcs_dispatch_key_ok;
	int plc_status;
	int plc_curr;
	u16 ufcs_vid;
	int batt_alarm;
	unsigned int usb_status;
	int ufcs_err_type;
	bool fcl_support;
	bool ss_check;
	bool fcl_trigger;
	int third_curve_target_vbus_mv;
	int third_curve_target_ibus_ma;
	atomic_t cp_offline;
	int last_chg_flashmode;
};

static void ufcs_pr_init(struct oplus_ufcs *chip);
static void ufcs_pr_check_bcc_curr(struct oplus_ufcs *chip);

struct current_level {
	int level;
	int curr_ma;
};

static const struct current_level g_ufcs_current_table[] = {
	{ 1, 1500 },   { 2, 1500 },   { 3, 2000 },   { 4, 2500 },   { 5, 3000 },  { 6, 3500 },   { 7, 4000 },
	{ 8, 4500 },   { 9, 5000 },   { 10, 5500 },  { 11, 6000 },  { 12, 6300 }, { 13, 6500 },  { 14, 7000 },
	{ 15, 7300 },  { 16, 8000 },  { 17, 8500 },  { 18, 9000 },  { 19, 9500 }, { 20, 10000 },  { 21, 10500 },
	{ 22, 11000 },  { 23, 11500 },  { 24, 12000 },  { 25, 12500 },  { 26, 13000 }, { 27, 13500 }, { 28, 14000 },
	{ 29, 14500 }, { 30, 15000 },
};

static const int ufcs_cool_down_oplus_curve[] = {
	15000, 1500,  1500,  2000,  2500,  3000,  3500,  4000,  4500,  5000,
	5500,  6000,  6300,  6500,  7000,  7300,  8000,  8500,  9000,  9500,
	10000, 10500, 11000, 11500, 12000, 12500, 13000, 13500, 14000, 14500, 15000
};

/* cp  batt curr from 3.6 spec table, at last it will convert to ibus and sent to cp voophy */
static const int ufcs_cp_cool_down_oplus_curve[] = {
	20000, 2000, 2000, 2400, 3000, 3400, 4000, 4400, 5000, 5400, 6000, 6400, 7000, 7400, 8000,
	9000, 10000, 11000, 12000, 12600, 13000, 14000, 15000, 16000, 17000, 18000, 19000, 20000
};

static const struct current_level g_ufcs_cp_current_table[] = {
	{ 1, 2000 },   { 2, 2000 },   { 3, 2400 },   { 4, 3000 },   { 5, 3400 },   { 6, 4000 },  { 7, 4400 },
	{ 8, 5000 },   { 9, 5400 },   { 10, 6000 },  { 11, 6400 },  { 12, 7000 },  { 13, 7400 }, { 14, 8000 },
	{ 15, 9000 },  { 16, 10000 }, { 17, 11000 }, { 18, 12000 }, { 19, 12600 }, { 20, 13000 }, { 21, 14000 },
	{ 22, 15000 }, { 23, 16000 }, { 24, 17000 }, { 25, 18000 }, { 26, 19000 }, { 27, 20000 },
};

static struct ufcs_abnormal_adapter_struct adapter_dev_info_table[] = {
	{ 0x1006, 0x0, 0x0, 0x0},
	{ 0x1007, 0x0, 0x0, 0x0},
	{ 0x1008, 0x0, 0x0, 0x0},
};


static const char *const strategy_low_curr_full[] = {
	[UFCS_LOW_CURR_FULL_CURVE_TEMP_LITTLE_COOL] = "strategy_temp_little_cool",
	[UFCS_LOW_CURR_FULL_CURVE_TEMP_NORMAL_LOW] = "strategy_temp_normal_low",
	[UFCS_LOW_CURR_FULL_CURVE_TEMP_NORMAL_HIGH] = "strategy_temp_normal_high",
};

static const char *const ufcs_user_err_type_str[] = {
	[UFCS_ERR_IBUS_LIMIT] = "ibus_limit",
	[UFCS_ERR_CP_ENABLE] = "cp_enable",
	[UFCS_ERR_R_COOLDOWN] = "r_cooldown",
	[UFCS_ERR_BATT_BTB_COOLDOWN] = "batbtb_cooldown",
	[UFCS_ERR_IBAT_OVER] = "ibat_over",
	[UFCS_ERR_BTB_OVER] = "btb_over",
	[UFCS_ERR_MOS_OVER] = "mos_over",
	[UFCS_ERR_USBTEMP_OVER] = "usbtemp_over",
	[UFCS_ERR_TFG_OVER] = "tfg_over",
	[UFCS_ERR_VBAT_DIFF] = "vbat_diff",
	[UFCS_ERR_STARTUP_FAIL] = "startup_fail",
	[UFCS_ERR_CIRCUIT_SWITCH] = "circuit_switch",
	[UFCS_ERR_ANTHEN_ERR] = "ahthen_err",
	[UFCS_ERR_PDO_ERR] = "pdo_err",
	[UFCS_ERR_IMP] = "imp",
};

__maybe_unused static bool
is_disable_charger_vatable_available(struct oplus_ufcs *chip)
{
	if (!chip->chg_disable_votable)
		chip->chg_disable_votable = find_votable("WIRED_CHARGING_DISABLE");
	return !!chip->chg_disable_votable;
}


__maybe_unused static bool
is_wired_suspend_votable_available(struct oplus_ufcs *chip)
{
	if (!chip->wired_suspend_votable)
		chip->wired_suspend_votable = find_votable("WIRED_CHARGE_SUSPEND");
	return !!chip->wired_suspend_votable;
}

__maybe_unused static bool
is_wired_icl_votable_available(struct oplus_ufcs *chip)
{
	if (!chip->wired_icl_votable)
		chip->wired_icl_votable = find_votable("WIRED_ICL");
	return !!chip->wired_icl_votable;
}

__maybe_unused static bool
is_gauge_topic_available(struct oplus_ufcs *chip)
{
	if (!chip->gauge_topic)
		chip->gauge_topic = oplus_mms_get_by_name("gauge");
	return !!chip->gauge_topic;
}

__maybe_unused
static bool is_err_topic_available(struct oplus_ufcs *chip)
{
	if (!chip->err_topic)
		chip->err_topic = oplus_mms_get_by_name("error");
	return !!chip->err_topic;
}

__maybe_unused
static int ufcs_find_current_to_level(int val, const struct current_level * const table, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		if (table[i].level == val)
			return table[i].curr_ma;
	}
	return 0;
}

static int ufcs_find_level_to_current(int val, const struct current_level * const table, int len)
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

static int oplus_ufcs_parse_temperature_strategy_init(struct oplus_ufcs *chip)
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
static int32_t oplus_ufcs_get_curve_vbus(struct oplus_ufcs *chip)
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

static int oplus_ufcs_apply_ccd(struct oplus_ufcs *chip, int target_ibus)
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

static void oplus_ufcs_ccd_strategy_init(struct oplus_ufcs *chip)
{
	int rc;
	u32 ccd_temp_region;

	if (!chip || !chip->ccd_strategy)
		return;
	ccd_temp_region = chip->ufcs_temp_cur_range > 0 ? chip->ufcs_temp_cur_range - 1 : 0;
	rc = oplus_chg_strategy_init(chip->ccd_strategy);
	if (rc < 0) {
		oplus_chg_strategy_release(chip->ccd_strategy);
		chip->ccd_strategy = NULL;
		return;
	}
	oplus_chg_strategy_set_process_data(chip->ccd_strategy, "temp_region", ccd_temp_region);
}

int oplus_ufcs_get_curve_ibus(struct oplus_mms *topic)
{
	struct oplus_ufcs *chip;
	struct puc_strategy_ret_data data;
	int rc;
	int target_ibus;

	if (topic == NULL)
		return -EINVAL;

	chip = oplus_mms_get_drvdata(topic);
	if (chip == NULL || chip->strategy == NULL)
		return -EINVAL;

	rc = oplus_chg_strategy_get_data(chip->strategy, &data);
	if (rc < 0) {
		chg_err("can't get curve ibus, rc=%d\n", rc);
		return rc;
	}
	target_ibus = data.target_ibus;
	if (chip->ccd_strategy)
		target_ibus = oplus_ufcs_apply_ccd(chip, data.target_ibus);
	return min(target_ibus, chip->adapter_max_curr);
}

static int oplus_ufcs_switch_to_normal(struct oplus_ufcs *chip)
{
	int rc;

	if (chip->dpdm_switch == NULL) {
		chg_err("dpdm_switch ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->dpdm_switch,
		OPLUS_IC_FUNC_SET_DPDM_SWITCH_MODE, DPDM_SWITCH_TO_AP);

	return rc;
}

__maybe_unused
static int oplus_ufcs_switch_to_ufcs(struct oplus_ufcs *chip)
{
	int rc;

	if (chip->dpdm_switch == NULL) {
		chg_err("dpdm_switch ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->dpdm_switch,
		OPLUS_IC_FUNC_SET_DPDM_SWITCH_MODE, DPDM_SWITCH_TO_UFCS);

	return rc;
}

static int oplus_ufcs_config_cp_watchdog(struct oplus_ufcs *chip, int timeout_ms)
{
	int rc;

	if (chip->cp_ic == NULL) {
		chg_err("cp_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_WATCHDOG_ENABLE, timeout_ms);
	chg_info("ufcs set cp watchdog time to %dms, rc=%d\n", timeout_ms, rc);
	if (rc < 0 && rc != -ENOTSUPP)
		return rc;

	return 0;
}

static int oplus_ufcs_set_online(struct oplus_ufcs *chip, bool online)
{
	struct mms_msg *msg;
	int rc;

	if (chip->ufcs_online == online)
		return 0;

	chip->ufcs_online = online;
	chg_info("set ufcs_online=%s\n", online ? "true" : "false");

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  UFCS_ITEM_ONLINE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->ufcs_topic, msg);
	if (rc < 0) {
		chg_err("publish ufcs online msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static int oplus_ufcs_set_charging(struct oplus_ufcs *chip, bool charging)
{
	struct mms_msg *msg;
	enum oplus_plc_chg_mode chg_mode;
	int rc;

	if (chip->ufcs_charging == charging)
		return 0;

	chip->ufcs_charging = charging;
	chg_info("set ufcs_charging=%s\n", charging ? "true" : "false");

	if (!chip->ufcs_charging)
		chip->adapter_max_curr = 0;
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  UFCS_ITEM_CHARGING);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->ufcs_topic, msg);
	if (rc < 0) {
		chg_err("publish ufcs charging msg error, rc=%d\n", rc);
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

static int oplus_ufcs_set_adapter_id(struct oplus_ufcs *chip, u16 id)
{
	struct mms_msg *msg;
	int rc;

	if (chip->adapter_id == id)
		return 0;

	chip->adapter_id = id;
	chg_info("set adapter_id=%u\n", id);

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  UFCS_ITEM_ADAPTER_ID);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->ufcs_topic, msg);
	if (rc < 0) {
		chg_err("publish ufcs adapter id msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static int oplus_ufcs_set_oplus_adapter(struct oplus_ufcs *chip, bool oplus_adapter)
{
	struct mms_msg *msg;
	int rc;

	if (chip->oplus_ufcs_adapter == oplus_adapter)
		return 0;

	chip->oplus_ufcs_adapter = oplus_adapter;
	if (oplus_adapter)
		chip->retention_oplus_adapter = true;
	chg_info("set oplus_ufcs_adapter=%s\n", oplus_adapter ? "true" : "false");

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  UFCS_ITEM_OPLUS_ADAPTER);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->ufcs_topic, msg);
	if (rc < 0) {
		chg_err("publish oplus adapter msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static int oplus_ufcs_publish_test_mode(struct oplus_ufcs *chip)
{
	struct mms_msg *msg;
	int rc;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_LOW,
				  UFCS_ITEM_TEST_MODE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg_sync(chip->ufcs_topic, msg);
	if (rc < 0) {
		chg_err("publish test_mode msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static void oplus_ufcs_set_plc_strategy(struct oplus_ufcs *chip)
{
	if (chip->opp == NULL)
		return;

	if (chip->oplus_ufcs_adapter)
		oplus_plc_protocol_set_strategy(chip->opp, "default");
	else
		oplus_plc_protocol_set_strategy(chip->opp, "third");
}

static int oplus_ufcs_set_ufcs_vid(struct oplus_ufcs *chip, u16 vid)
{
	struct mms_msg *msg;
	int rc;

	if (chip->ufcs_vid == vid)
		return 0;

	chip->ufcs_vid = vid;
	chg_info("set ufcs vid=0x%x\n", chip->ufcs_vid);

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  UFCS_ITEM_UFCS_VID);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->ufcs_topic, msg);
	if (rc < 0) {
		chg_err("publish ufcs vid msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}


static void oplus_ufcs_switch_end_recheck_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_ufcs *chip =
		container_of(dwork, struct oplus_ufcs, switch_end_recheck_work);

	chg_info("ufcs switch end recheck\n");
	if (chip->ufcs_online)
		return;
	if (chip->retention_state_ready)
		return;
	chg_info("ufcs switch end\n");
	oplus_cpa_switch_end(chip->cpa_topic, CHG_PROTOCOL_UFCS);
}

#define SWITCH_END_RECHECK_DELAY_MS	1000
static int oplus_ufcs_cpa_switch_end(struct oplus_ufcs *chip)
{
	if (chip->ufcs_online)
		return 0;

	if (!chip->retention_state || chip->retention_exit_ufcs_flag == EXIT_THIRD_UFCS) {
		oplus_cpa_switch_end(chip->cpa_topic, CHG_PROTOCOL_UFCS);
	} else {
		if (!chip->retention_state_ready)
			schedule_delayed_work(&chip->switch_end_recheck_work,
				msecs_to_jiffies(SWITCH_END_RECHECK_DELAY_MS));
	}
	return 0;
}

const char *oplus_ufcs_get_err_type_str(enum ufcs_user_err_type type)
{
	if (type < UFCS_ERR_IBUS_LIMIT || type >= UFCS_ERR_MAX)
		return "Inval";
	return ufcs_user_err_type_str[type];
}

static int ufcs_debug_track = 0;
module_param(ufcs_debug_track, int, 0644);
MODULE_PARM_DESC(ufcs_debug_track, "ufcs debug track");
#define TRACK_UPLOAD_COUNT_MAX			3
#define TRACK_LOCAL_T_NS_TO_S_THD 		1000000000
#define TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD 	(24 * 3600)
int oplus_ufcs_track_upload_msg(struct oplus_ufcs *chip, char *ufcs_msg)
{
	struct oplus_mms *err_topic;
	struct mms_msg *msg;
	int rc;
	static int upload_count = 0;
	static int pre_upload_time = 0;
	int curr_time;

	if (!chip)
		return -ENODEV;

	curr_time = local_clock() / TRACK_LOCAL_T_NS_TO_S_THD;
	if (curr_time - pre_upload_time > TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD)
		upload_count = 0;

	if (upload_count >= TRACK_UPLOAD_COUNT_MAX)
		return -ENODEV;

	pre_upload_time = local_clock() / TRACK_LOCAL_T_NS_TO_S_THD;

	err_topic = oplus_mms_get_by_name("error");
	if (!err_topic) {
		chg_err("error topic not found\n");
		return -ENODEV;
	}

	msg = oplus_mms_alloc_str_msg(
		MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_UFCS, ufcs_msg);
	if (msg == NULL) {
		chg_err("alloc ufcs error msg error\n");
		return -ENOMEM;
	}

	rc = oplus_mms_publish_msg_sync(err_topic, msg);
	if (rc < 0) {
		chg_err("publish ufcs error msg error, rc=%d\n", rc);
		kfree(msg);
	}
	upload_count++;

	return rc;
}

static void oplus_ufcs_track_upload_err_info(struct oplus_ufcs *chip, int err_type, int value)
{
	char *buf;
	int i = 0;
	size_t index = 0;

	if (ufcs_debug_track)
		err_type = ufcs_debug_track;

	if (!is_err_topic_available(chip))
		return;
	if ((err_type == UFCS_ERR_ANTHEN_ERR) &&
	    (chip->config.curr_max_ma <= UFCS_VERIFY_CURR_THR_MA))
		return;

	buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (buf == NULL)
		return;

	chip->ufcs_err_type = err_type;
	index = scnprintf(buf, PAGE_SIZE, "$$err_scene@@ufcs_err");
	index += scnprintf(buf + index,
		PAGE_SIZE - index, "$$err_reason@@%s$$value@@%d", track_ufcs_sw_error_str(err_type), value);
	index += scnprintf(buf + index, PAGE_SIZE - index, "$$emark_imax@@%d$$power_imax@@%d$$power_cpa@@%d",
		chip->emark_imax, chip->power_imax, oplus_cpa_protocol_get_power(chip->cpa_topic, CHG_PROTOCOL_UFCS));
	index += scnprintf(buf + index, PAGE_SIZE - index,
		"$$cable_type@@%s$$hw_ver@@%llu$$curr_max@@%llu$$vol_max@@%llu",
		ufcs_get_oplus_emark_info_str(UFCS_OPLUS_VND_EMARK_INFO(chip->emark_info)),
		UFCS_OPLUS_VND_EMARK_HW_VER(chip->emark_info),
		UFCS_OPLUS_VND_EMARK_CURR_MAX(chip->emark_info),
		UFCS_OPLUS_VND_EMARK_VOL_MAX(chip->emark_info));

	if (chip->pie_num > 0)
		index += scnprintf(buf + index, PAGE_SIZE - index, "$$power_info@@");

	for (i = 0; i < chip->pie_num; i++) {
		if (UFCS_OPLUS_VND_POWER_INFO_TYPE(chip->pie[i]) == UFCS_OPLUS_POWER_INFO_UFCS) {
			index += scnprintf(buf + index, PAGE_SIZE - index, "index=%llu,curr_max=%llu,vol_max=%llu,",
				UFCS_OPLUS_VND_POWER_INFO_INDEX(chip->pie[i]),
				UFCS_OPLUS_VND_POWER_INFO_CURR_MAX(chip->pie[i]),
				UFCS_OPLUS_VND_POWER_INFO_VOL_MAX(chip->pie[i]));
		}
	}

	index += scnprintf(buf + index, PAGE_SIZE - index, "$$oplus_id@@0x%llx$$dev_info@@0x%llx"
		"$$cable_info@@0x%llx$$emark_info@@0x%llx", UFCS_DEVICE_INFO_HW_VER(chip->dev_info),
		chip->dev_info, chip->cable_info, chip->emark_info);

	if (chip->pdo_num > 0)
		index += scnprintf(buf + index, PAGE_SIZE - index, "$$pdo_info@@");
	for (i = 0; i < chip->pdo_num; i++) {
		index += scnprintf(buf + index, PAGE_SIZE - index, "pdo[%d]: voltage[%llu-%llumV, step:%llumV], "
			"current[%llu-%llumA, step:%llumA]", i,
			 UFCS_OUTPUT_MODE_VOL_MIN(chip->pdo[i]), UFCS_OUTPUT_MODE_VOL_MAX(chip->pdo[i]),
			 UFCS_OUTPUT_MODE_VOL_STEP(chip->pdo[i]), UFCS_OUTPUT_MODE_CURR_MIN(chip->pdo[i]),
			 UFCS_OUTPUT_MODE_CURR_MAX(chip->pdo[i]), UFCS_OUTPUT_MODE_CURR_STEP(chip->pdo[i]));
	}
	oplus_ufcs_track_upload_msg(chip, buf);
	kfree(buf);
}

static int oplus_ufcs_push_eis_status(struct oplus_ufcs *chip, int status)
{
	struct mms_msg *msg;
	int rc;

	if (!chip->comm_topic) {
		chg_info("comm_topic is null\n");
		return -ENODEV;
	}

	msg = oplus_mms_alloc_int_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, COMM_ITEM_EIS_STATUS, status);
	if (msg == NULL) {
		chg_err("alloc eis status msg error\n");
		return -ENOMEM;
	}

	rc = oplus_mms_publish_msg_sync(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish eis status msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static int oplus_ufcs_push_lcf_alarm_status(struct oplus_ufcs *chip, int status)
{
	struct mms_msg *msg;
	int rc;

	if (!chip->batt_bal_topic)
		return -ENODEV;

	msg = oplus_mms_alloc_int_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, BATT_BAL_ITEM_LCF_ALARM, status);
	if (msg == NULL) {
		chg_err("alloc lcf alarm status msg error\n");
		return -ENOMEM;
	}

	rc = oplus_mms_publish_msg_sync(chip->batt_bal_topic, msg);
	if (rc < 0) {
		chg_err("publish lcf alarm status msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static int oplus_ufcs_push_emark_power(struct oplus_ufcs *chip, int power_mw)
{
	struct mms_msg *msg;
	int rc;

	if (!chip->ufcs_topic)
		return -ENODEV;

	msg = oplus_mms_alloc_int_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, UFCS_ITEM_EMARK_POWER, power_mw);
	if (msg == NULL) {
		chg_err("alloc emark power msg error\n");
		return -ENOMEM;
	}

	rc = oplus_mms_publish_msg(chip->ufcs_topic, msg);
	if (rc < 0) {
		chg_err("publish emark power msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static int oplus_ufcs_push_adapter_power(struct oplus_ufcs *chip, int power_mw)
{
	struct mms_msg *msg;
	int rc;

	if (!chip->ufcs_topic)
		return -ENODEV;

	msg = oplus_mms_alloc_int_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, UFCS_ITEM_ADAPTER_POWER, power_mw);
	if (msg == NULL) {
		chg_err("alloc emark power msg error\n");
		return -ENOMEM;
	}

	rc = oplus_mms_publish_msg(chip->ufcs_topic, msg);
	if (rc < 0) {
		chg_err("publish emark power msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static int oplus_ufcs_track_set_fcl_info(struct device *dev, int batt_volt, int batt_curr, int batt_temp)
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

__maybe_unused
static int oplus_ufcs_handshake(struct oplus_ufcs *chip)
{
	int rc;

	if (chip->ufcs_ic == NULL) {
		chg_err("ufcs_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->ufcs_ic, OPLUS_IC_FUNC_UFCS_HANDSHAKE);

	return rc;
}

__maybe_unused
static int oplus_ufcs_pdo_set(struct oplus_ufcs *chip, int vol_mv, int curr_ma)
{
	int rc;

	if (chip->ufcs_ic == NULL) {
		chg_err("ufcs_ic is NULL\n");
		return -ENODEV;
	}
	/*The same voltage and current will not be applied repeatedly on the adsp ufcs protocol stack side*/
	if ((chip->config.adsp_ufcs_project) && (chip->curr_set_ma == curr_ma) &&
	    (chip->vol_set_mv == vol_mv)) {
		chg_err("request  current and voltage are exactly the same as the last\n");
		return 0;
	}

	rc = oplus_chg_ic_func(chip->ufcs_ic, OPLUS_IC_FUNC_UFCS_PDO_SET, vol_mv, curr_ma);
	if (rc < 0 && chip->ufcs_online) {
		oplus_ufcs_track_upload_err_info(chip, TRACK_UFCS_ERR_PDO_ERR, curr_ma);
	} else if (rc == 0) {
		chip->curr_set_ma = curr_ma;
		chip->vol_set_mv = vol_mv;
	}

	return rc;
}

__maybe_unused
static int oplus_ufcs_hard_reset(struct oplus_ufcs *chip)
{
	int rc;

	if (chip->ufcs_ic == NULL) {
		chg_err("ufcs_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->ufcs_ic, OPLUS_IC_FUNC_UFCS_HARD_RESET);

	return rc;
}

__maybe_unused
static int oplus_ufcs_exit_ufcs_mode(struct oplus_ufcs *chip)
{
	int rc;

	if (chip->ufcs_ic == NULL) {
		chg_err("ufcs_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->ufcs_ic, OPLUS_IC_FUNC_UFCS_EXIT);
	if (rc < 0)
		chg_err("exit ufcs mode error, rc=%d", rc);

	return rc;
}

__maybe_unused
static int oplus_ufcs_config_wd(struct oplus_ufcs *chip, u16 time_ms)
{
	int rc;

	if (chip->ufcs_ic == NULL) {
		chg_err("ufcs_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->ufcs_ic, OPLUS_IC_FUNC_UFCS_CONFIG_WD, time_ms);

	return rc;
}

__maybe_unused
static int oplus_ufcs_config_sm_period(struct oplus_ufcs *chip, u16 time_ms)
{
	int rc;

	if (chip->ufcs_ic == NULL) {
		chg_err("ufcs_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->ufcs_ic, OPLUS_IC_FUNC_UFCS_SET_SM_PERIOD, time_ms);

	return rc;
}

__maybe_unused
static int oplus_ufcs_get_dev_info(struct oplus_ufcs *chip, u64 *dev_info)
{
	int rc;

	if (chip->ufcs_ic == NULL) {
		chg_err("ufcs_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->ufcs_ic, OPLUS_IC_FUNC_UFCS_GET_DEV_INFO, dev_info);

	return rc;
}

__maybe_unused
static int oplus_ufcs_get_err_info(struct oplus_ufcs *chip, u64 *err_info)
{
	int rc;

	if (chip->ufcs_ic == NULL) {
		chg_err("ufcs_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->ufcs_ic, OPLUS_IC_FUNC_UFCS_GET_ERR_INFO, err_info);

	return rc;
}

__maybe_unused
static int oplus_ufcs_get_src_info(struct oplus_ufcs *chip, u64 *src_info)
{
	int rc;

	if (chip->ufcs_ic == NULL) {
		chg_err("ufcs_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->ufcs_ic, OPLUS_IC_FUNC_UFCS_GET_SRC_INFO, src_info);

	return rc;
}

__maybe_unused
static int oplus_ufcs_get_cable_info(struct oplus_ufcs *chip, u64 *cable_info)
{
	int rc;

	if (chip->ufcs_ic == NULL) {
		chg_err("ufcs_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->ufcs_ic, OPLUS_IC_FUNC_UFCS_GET_CABLE_INFO, cable_info);

	return rc;
}

__maybe_unused
static int oplus_ufcs_get_pdo_info(struct oplus_ufcs *chip, u64 *pdo, int num)
{
	int rc;

	if (chip->ufcs_ic == NULL) {
		chg_err("ufcs_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->ufcs_ic, OPLUS_IC_FUNC_UFCS_GET_PDO_INFO, pdo, num);

	return rc;
}

static int oplus_ufcs_verify_adapter(struct oplus_ufcs *chip,
	u8 index, u8 *auth_data, u8 data_len)
{
	int rc;

	if (chip->ufcs_ic == NULL) {
		chg_err("ufcs_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->ufcs_ic, OPLUS_IC_FUNC_UFCS_VERIFY_ADAPTER,
		index, auth_data, data_len);

	return rc;
}

__maybe_unused
static int oplus_ufcs_get_power_change_info(
	struct oplus_ufcs *chip, u32 *pwr_change_info, int num)
{
	int rc;

	if (chip->ufcs_ic == NULL) {
		chg_err("ufcs_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->ufcs_ic, OPLUS_IC_FUNC_UFCS_GET_POWER_CHANGE_INFO,
		pwr_change_info, num);

	return rc;
}

__maybe_unused
static int oplus_chg_ufcs_get_emark_info(struct oplus_ufcs *chip, u64 *info)
{
	int rc;

	if (chip->ufcs_ic == NULL) {
		chg_err("ufcs_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->ufcs_ic, OPLUS_IC_FUNC_UFCS_GET_EMARK_INFO, info);
	chg_err("emark_info=0x%llx\n", *info);

	return rc;
}

__maybe_unused
static int oplus_chg_ufcs_get_power_info_ext(struct oplus_ufcs *chip, u64 *info, int num)
{
	int rc;

	if (chip->ufcs_ic == NULL) {
		chg_err("ufcs_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->ufcs_ic, OPLUS_IC_FUNC_UFCS_GET_POWER_INFO_EXT, info, num);

	return rc;
}

__maybe_unused
static bool oplus_chg_ufcs_is_test_mode(struct oplus_ufcs *chip)
{
	bool en;
	int rc;

	if (chip->ufcs_ic == NULL) {
		chg_err("ufcs_ic is NULL\n");
		return false;
	}

	rc = oplus_chg_ic_func(chip->ufcs_ic, OPLUS_IC_FUNC_UFCS_IS_TEST_MODE, &en);
	if (rc < 0) {
		chg_err("can't get test mode info, rc=%d\n", rc);
		return false;
	}

	return en;
}

__maybe_unused
static bool oplus_chg_ufcs_is_vol_acc_test_mode(struct oplus_ufcs *chip)
{
	bool en;
	int rc;

	if (chip->ufcs_ic == NULL) {
		chg_err("ufcs_ic is NULL\n");
		return false;
	}

	rc = oplus_chg_ic_func(chip->ufcs_ic, OPLUS_IC_FUNC_UFCS_IS_VOL_ACC_TEST_MODE, &en);
	if (rc < 0) {
		chg_err("can't get vol acc test mode info, rc=%d\n", rc);
		return false;
	}

	return en;
}

__maybe_unused
static int oplus_ufcs_cp_enable(struct oplus_ufcs *chip, bool en)
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
static int oplus_ufcs_cp_hw_init(struct oplus_ufcs *chip)
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
static int oplus_ufcs_cp_set_work_mode(struct oplus_ufcs *chip, enum oplus_cp_work_mode mode)
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

	if (rc >= 0)
		chip->oplus_cp_ucp_disable = false;

	return rc;
}

__maybe_unused
static int oplus_ufcs_cp_get_work_mode(struct oplus_ufcs *chip, enum oplus_cp_work_mode *mode)
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
static int oplus_ufcs_cp_check_work_mode_support(struct oplus_ufcs *chip, enum oplus_cp_work_mode mode)
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
static int oplus_ufcs_cp_set_iin(struct oplus_ufcs *chip, int iin)
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
static int oplus_ufcs_cp_get_vin(struct oplus_ufcs *chip, int *vin)
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
static int oplus_ufcs_cp_get_iin(struct oplus_ufcs *chip, int *iin)
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
static int oplus_ufcs_cp_get_vout(struct oplus_ufcs *chip, int *vout)
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
static int oplus_ufcs_cp_get_iout(struct oplus_ufcs *chip, int *iout)
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
static int oplus_ufcs_cp_get_vac(struct oplus_ufcs *chip, int *vac)
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
static int oplus_ufcs_cp_set_work_start(struct oplus_ufcs *chip, bool start)
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
static int oplus_ufcs_cp_set_ucp_disable(struct oplus_ufcs *chip, bool disable)
{
	int rc;

	if (chip->cp_ic == NULL) {
		chg_err("cp_ic is NULL\n");
		return -ENODEV;
	}
	if (chip->opp == NULL)
		return -ENOTSUPP;

	if (chip->oplus_cp_ucp_disable == disable)
		return 0;

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_SET_UCP_DISABLE, disable);
	if (rc >= 0)
		chip->oplus_cp_ucp_disable = disable;
	return rc;
}

__maybe_unused
static int oplus_ufcs_cp_ss_timeout_enable(struct oplus_ufcs *chip, bool enable)
{
	int rc = 0;

	if (chip->cp_ic == NULL) {
		chg_err("cp_ic is NULL\n");
		return -ENODEV;
	}
	if (chip->ss_check || (chip->plc_status == PLC_STATUS_ENABLE))
		return rc;

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_SET_SSTIMEOUT_UCP_ENABLE, enable);
	chip->ss_check = enable;
	return rc;
}

__maybe_unused
static int oplus_ufcs_cp_set_pmid2vout_ovp_enable(struct oplus_ufcs *chip, bool enable)
{
	int rc = 0;

	if (chip->cp_ic == NULL) {
		chg_err("cp_ic is NULL\n");
		return -ENODEV;
	}
	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_SET_PMID2VOUT_OVP_ENABLE, enable);

	return rc;
}

__maybe_unused
static int oplus_ufcs_cp_get_work_status(struct oplus_ufcs *chip, bool *start)
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
static int oplus_ufcs_cp_adc_enable(struct oplus_ufcs *chip, bool en)
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
static int oplus_ufcs_cp_get_temp(struct oplus_ufcs *chip)
{
	int rc = 0;
	int cp_temp = 25;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return cp_temp;
	}

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_GET_TEMP, &cp_temp);
	if (rc < 0 && rc != -ENOTSUPP) {
		chg_err("can't get cp temp, rc=%d\n", rc);
	}

	return cp_temp;
}

static int oplus_ufcs_cp_watchdog_reset(struct oplus_ufcs *chip)
{
	int rc;

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_WATCHDOG_RESET);
	if (rc < 0 && rc != -ENOTSUPP)
		return rc;

	return 0;
}

static bool oplus_is_ufcs_abnormal_adapter(struct oplus_ufcs *chip, u64 id)
{
	struct ufcs_abnormal_adapter_struct *dev_info;
	int i;

	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return false;
	}

	for (i = 0; i < ARRAY_SIZE(adapter_dev_info_table); i++) {
		dev_info = &adapter_dev_info_table[i];
		if ((UFCS_DEVICE_INFO_SW_VER(id) == dev_info->sw_ver) &&
			    (UFCS_DEVICE_INFO_HW_VER(id) == dev_info->hw_ver) &&
			    (UFCS_DEVICE_INFO_IC_VENDOR(id) == dev_info->ic_vendor) &&
			    (UFCS_DEVICE_INFO_DEV_VENDOR(id) == dev_info->dev_vendor))
			return true;
	}

	return false;
}

__maybe_unused
static int oplus_ufcs_cp_reg_dump(struct oplus_ufcs *chip)
{
	int rc;

	if (chip->cp_ic == NULL) {
		chg_err("cp_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_REG_DUMP);

	return rc;
}

__maybe_unused
static int oplus_ufcs_reset_dpdm(struct oplus_ufcs *chip)
{
	int rc;

	if (chip->ufcs_ic == NULL) {
		chg_err("ufcs_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->ufcs_ic, OPLUS_IC_FUNC_UFCS_RESET_DPDM);

	return rc;
}

static void oplus_ufcs_deep_ratio_limit_curr(struct oplus_ufcs *chip)
{
	union mms_msg_data data = { 0 };
	int rc;

	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return;
	}

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_RATIO_LIMIT_CURR, &data, true);
	if (rc < 0)
		return;

	if (data.intval <= 0) {
		chg_err("get ratio limit curr error, data.intval=%d\n", data.intval);
		return;
	}

	chg_info("ration limit curr: %d", data.intval);
	vote(chip->ufcs_curr_votable, DEEP_RATIO_LIMIT_VOTER, true, data.intval, false);
}

static int oplus_ufcs_cp_get_iin_max(struct oplus_ufcs *chip,
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

static void oplus_ufcs_set_ic_ufcs_curr_vote(struct oplus_ufcs *chip, enum oplus_cp_work_mode mode)
{
	int iin_max;
	int rc;

	rc = oplus_ufcs_cp_get_iin_max(chip, mode, &iin_max);
	if (rc < 0) {
		chg_err("can't get cp iin max, rc=%d\n", rc);
		return;
	}
	chg_info("iin_max=%d\n", iin_max);
	if (iin_max <= 0)
		return;
	vote(chip->ufcs_curr_votable, IC_VOTER, true, iin_max, false);
}

static void oplus_ufcs_charge_btb_allow_check(struct oplus_ufcs *chip)
{
#define UFCS_BTB_CHECK_MAX_CNT 3
#define UFCS_BTB_CHECK_TIME_US 10000

	int btb_check_cnt = UFCS_BTB_CHECK_MAX_CNT;
	int batt_btb_temp;
	int usb_btb_temp;
	int cp_temp;
	bool shaft_btb_normal = true;

	while (btb_check_cnt != 0) {
		batt_btb_temp = oplus_wired_get_batt_btb_temp();
		usb_btb_temp = oplus_wired_get_usb_btb_temp();
		cp_temp = oplus_ufcs_cp_get_temp(chip);
		shaft_btb_normal = oplus_wired_get_shaft_btb_is_normal();

		if (batt_btb_temp < UFCS_BTB_TEMP_MAX &&
		    usb_btb_temp < UFCS_USB_TEMP_MAX &&
		    cp_temp < UFCS_CP_TEMP_MAX && shaft_btb_normal)
			break;

		btb_check_cnt--;
		if (btb_check_cnt > 0)
			usleep_range(UFCS_BTB_CHECK_TIME_US, UFCS_BTB_CHECK_TIME_US);
	}
	if (btb_check_cnt == 0) {
		chg_info("batt_btb_temp: %d, usb_btb_temp = %d shaft_btb_normal %d",
			batt_btb_temp, usb_btb_temp, shaft_btb_normal);
		vote(chip->ufcs_not_allow_votable, BTB_TEMP_OVER_VOTER, true, 1, false);
		if (is_wired_icl_votable_available(chip))
			vote(chip->wired_icl_votable, BTB_TEMP_OVER_VOTER, true,
			     BTB_TEMP_OVER_MAX_INPUT_CUR, true);
	} else {
		if (get_client_vote(chip->ufcs_not_allow_votable, BTB_TEMP_OVER_VOTER) > 0) {
			chg_info("batt_btb_temp: %d, usb_btb_temp = %d, recover it now.",
				  batt_btb_temp, usb_btb_temp);
			vote(chip->ufcs_not_allow_votable, BTB_TEMP_OVER_VOTER, false, 0, false);
		}
	}

	if (!shaft_btb_normal && btb_check_cnt == 0)
		oplus_wired_set_shaft_btb_over(true);
	else
		oplus_wired_set_shaft_btb_over(false);
}

static int oplus_ufcs_parse_vfa_strategy(struct oplus_ufcs *chip)
{
	struct device_node *vfa_node;

	vfa_node = of_parse_phandle(oplus_get_node_by_type(chip->dev->of_node), "oplus,ufcs_vfa_strategy", 0);
	if (!vfa_node) {
		chg_info("ufcs_vfa_strategy not found, skip vfa strategy\n");
		return 0;
	}

	chip->vfa_strategy = oplus_chg_strategy_alloc_by_node("vfa_strategy", vfa_node);
	of_node_put(vfa_node);
	if (IS_ERR_OR_NULL(chip->vfa_strategy)) {
		chg_err("alloc ufcs vfa startegy error, rc=%ld", PTR_ERR(chip->vfa_strategy));
		chip->vfa_strategy = NULL;
		return -EFAULT;
	}

	return 0;
}

static int oplus_ufcs_parse_ccd_strategy(struct oplus_ufcs *chip)
{
	struct device_node *node = oplus_get_node_by_type(chip->dev->of_node);

	chip->ccd_strategy = oplus_chg_strategy_alloc_by_node("cycle_current_derating", node);
	if (IS_ERR_OR_NULL(chip->ccd_strategy))
		chip->ccd_strategy = NULL;
	if (chip->ccd_strategy)
		oplus_chg_strategy_set_process_data(chip->ccd_strategy, "temp_region_cnt", chip->temp_region_cnt);
	return 0;
}

static void oplus_ufcs_vfa_reset(struct oplus_ufcs *chip)
{
	if (chip->vfa_strategy)
		oplus_chg_strategy_set_process_data(chip->vfa_strategy, "reset", 0);
	vote(chip->ufcs_not_allow_votable, VFA_VOTER, false, 0, false);
}

static void oplus_ufcs_vfa_allow(struct oplus_ufcs *chip, bool do_init)
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
	vote(chip->ufcs_not_allow_votable, VFA_VOTER, !allow_fastchg, 1, false);
}

static bool oplus_ufcs_charge_allow_check(struct oplus_ufcs *chip)
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

	if (chg_temp < chip->limits.ufcs_low_temp - atomic_read(&chip->temp_recover_debounce_thd) ||
	    chg_temp >= chip->limits.ufcs_high_temp + atomic_read(&chip->temp_recover_debounce_thd)) {
		chg_err("chg_temp:%d temp_recover_debounce_thd:%d \n", chg_temp, atomic_read(&chip->temp_recover_debounce_thd));
		atomic_set(&chip->temp_recover_debounce_thd, 0);
		vote(chip->ufcs_not_allow_votable, BATT_TEMP_VOTER, true, 1, false);
	} else if (chg_temp > (chip->limits.ufcs_normal_high_temp - UFCS_TEMP_WARM_RANGE_THD)) {
		if (chip->ui_soc > chip->limits.ufcs_warm_allow_soc ||
		    vbat_mv > chip->limits.ufcs_warm_allow_vol)
			vote(chip->ufcs_not_allow_votable, BATT_TEMP_VOTER, true, 1, false);
		else
			vote(chip->ufcs_not_allow_votable, BATT_TEMP_VOTER, false, 0, false);
	} else {
		vote(chip->ufcs_not_allow_votable, BATT_TEMP_VOTER, false, 0, false);
	}

	if (chip->ui_soc < chip->limits.ufcs_low_soc || chip->ui_soc > chip->high_soc) {
		vote(chip->ufcs_not_allow_votable, BATT_SOC_VOTER, true, 1, false);
	} else {
		vote(chip->ufcs_not_allow_votable, BATT_SOC_VOTER, false, 0, false);
		chip->allow_check_soc = chip->ui_soc;
	}

	oplus_ufcs_charge_btb_allow_check(chip);

	oplus_ufcs_vfa_allow(chip, true);

	return !chip->ufcs_not_allow;
}

static void oplus_ufcs_count_init(struct oplus_ufcs *chip)
{
	chip->count.cool_fw = 0;
	chip->count.cool_full_fw = 0;
	chip->count.sw_full = 0;
	chip->count.hw_full = 0;
	chip->count.low_curr_full = 0;
	chip->count.ibat_low = 0;
	chip->count.ibat_high = 0;
	chip->count.btb_high = 0;
	chip->count.tbatt_over = 0;
	chip->count.tfg_over = 0;
	chip->count.output_low = 0;
	chip->count.ibus_over = 0;
}

static void oplus_ufcs_votable_reset(struct oplus_ufcs *chip)
{
	vote(chip->ufcs_disable_votable, NO_DATA_VOTER, false, 0, false);
	vote(chip->ufcs_disable_votable, CHG_FULL_VOTER, false, 0, false);
	vote(chip->ufcs_disable_votable, IBAT_OVER_VOTER, false, 0, false);
	vote(chip->ufcs_disable_votable, BTB_TEMP_OVER_VOTER, false, 0, false);
	vote(chip->ufcs_disable_votable, BATT_TEMP_VOTER, false, 0, false);
	vote(chip->ufcs_disable_votable, SWITCH_RANGE_VOTER, false, 0, false);
	vote(chip->ufcs_disable_votable, TFG_VOTER, false, 0, false);
	vote(chip->ufcs_disable_votable, IMP_VOTER, false, 0, false);
	vote(chip->ufcs_disable_votable, TIMEOUT_VOTER, false, 0, false);
	vote(chip->ufcs_disable_votable, IOUT_CURR_VOTER, false, 0, false);
	vote(chip->ufcs_disable_votable, ADSP_CRASH_VOTER, false, 0, false);
	vote(chip->ufcs_disable_votable, PLC_RETRY_VOTER, false, 0, false);
	vote(chip->ufcs_disable_votable, PR_VOTER, false, 0, false);

	vote(chip->ufcs_not_allow_votable, CHG_FULL_COOL_VOTER, false, 0, false);

	vote(chip->ufcs_curr_votable, IMP_VOTER, false, 0, false);
	vote(chip->ufcs_curr_votable, STEP_VOTER, false, 0, false);
	vote(chip->ufcs_curr_votable, BATT_TEMP_VOTER, false, 0, false);
	vote(chip->ufcs_curr_votable, COOL_DOWN_VOTER, false, 0, false);
	vote(chip->ufcs_curr_votable, SALE_MODE_VOTER, false, 0, false);
	vote(chip->ufcs_curr_votable, BCC_VOTER, false, 0, false);
	vote(chip->ufcs_curr_votable, BATT_BAL_VOTER, false, 0, false);
	vote(chip->ufcs_curr_votable, IBUS_OVER_VOTER, false, 0, false);
	vote(chip->ufcs_curr_votable, SLOW_CHG_VOTER, false, 0, false);
	vote(chip->ufcs_curr_votable, CABLE_MAX_VOTER, false, 0, false);
	vote(chip->ufcs_curr_votable, ADAPTER_IMAX_VOTER, false, 0, false);
	vote(chip->ufcs_curr_votable, PLC_VOTER, false, 0, false);
	vote(chip->ufcs_curr_votable, IC_VOTER, false, 0, false);
	vote(chip->ufcs_curr_votable, BATT_SOC_VOTER, false, 0, false);
	vote(chip->ufcs_curr_votable, LIMIT_FCL_VOTER, false, 0, false);
	vote(chip->ufcs_curr_votable, PR_VOTER, false, 0, false);
}

static int oplus_ufcs_temp_cur_range_init(struct oplus_ufcs *chip)
{
	int vbat_temp_cur;

	vbat_temp_cur = chip->shell_temp;
	if (vbat_temp_cur < chip->limits.ufcs_little_cold_temp) { /*0-5C*/
		chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_LITTLE_COLD;
		chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_LITTLE_COLD;
	} else if (vbat_temp_cur < chip->limits.ufcs_cool_temp) { /*5-12C*/
		chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_COOL;
		chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_COOL;
	} else if (vbat_temp_cur < chip->limits.ufcs_little_cool_temp) { /*12-20C*/
		chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_LITTLE_COOL;
		chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_LITTLE_COOL;
	} else if (chip->limits.ufcs_little_cool_high_temp != -EINVAL &&
		vbat_temp_cur < chip->limits.ufcs_little_cool_high_temp) {
		chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_LITTLE_COOL_HIGH;
		chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_LITTLE_COOL_HIGH;
	} else if (chip->limits.ufcs_normal_low_pre_temp != -EINVAL &&
		vbat_temp_cur < chip->limits.ufcs_normal_low_pre_temp) {
		chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_NORMAL_LOW_PRE;
		chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_NORMAL_LOW_PRE;
	} else if (vbat_temp_cur < chip->limits.ufcs_normal_low_temp) { /*20-35C*/
		chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_NORMAL_LOW;
		chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_NORMAL_LOW;
	} else if (vbat_temp_cur < chip->limits.ufcs_normal_high_temp) { /*35C-43C*/
		chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_NORMAL_HIGH;
		chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_NORMAL_HIGH;
	} else {
		chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_WARM;
		chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_WARM;
	}

	return 0;
}

static void oplus_ufcs_variables_early_init(struct oplus_ufcs *chip)
{
	oplus_ufcs_count_init(chip);

	chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_NATURAL;
	chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_INIT;

	chip->src_info = 0;
	chip->dev_info = 0;
	chip->emark_info = 0;
	chip->cable_info = 0;
	chip->pdo_num = 0;
	chip->pie_num = 0;
	chip->ss_check = false;
	chip->fcl_trigger = false;
	oplus_ufcs_set_ufcs_vid(chip, 0);
	chip->subsys_reset_ufcs = false;
}

#define UFCS_FCS_ICL1_MA	1500
#define UFCS_FCS_ICL2_MA	800
static int oplus_ufcs_set_fcs_icl_vote(int icl_ma)
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

static void oplus_ufcs_set_fcs_icl_work(struct work_struct *work)
{
	oplus_ufcs_set_fcs_icl_vote(0);
}

static void oplus_ufcs_clear_fcs_icl(struct oplus_ufcs *chip)
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

static void oplus_ufcs_variables_init(struct oplus_ufcs *chip)
{
	chip->timer.fastchg_timer = oplus_current_kernel_time();
	chip->timer.temp_timer = oplus_current_kernel_time();
	chip->timer.pdo_timer = oplus_current_kernel_time();
	chip->timer.ibat_jiffies = jiffies;
	if (!chip->oplus_ufcs_adapter) {
		chip->timer.ufcs_max_time_ms = chip->limits.ufcs_timeout_third * 1000;
		chip->need_preliminary_imp_check = false;
		chip->high_soc = chip->limits.ufcs_high_soc_third;
	} else {
		chip->timer.ufcs_max_time_ms = chip->limits.ufcs_timeout_oplus * 1000;
		chip->need_preliminary_imp_check = true;
		chip->high_soc = chip->limits.ufcs_high_soc;
	}
	chip->ufcs_low_curr_full_temp_status =
		UFCS_LOW_CURR_FULL_CURVE_TEMP_NORMAL_LOW;
	chip->start_retry_count = 0;
	chip->startup_retry_times = 0;
	chip->start_check = false;
	chip->batt_bal_curr_limit = 0;
	chip->preliminary_imp_check_cnt = 0;
	(void)oplus_imp_uint_reset(chip->imp_uint);
	chip->eis_status = EIS_STATUS_DISABLE;
	chip->batt_alarm = 0;
	chip->last_target_curr_ma = chip->target_curr_ma;
	chip->allow_check_soc = chip->ui_soc;
	/* Keep async clear to avoid blocking init/transition path on ICL lock. */
	oplus_ufcs_clear_fcs_icl(chip);
}

static void oplus_ufcs_force_exit(struct oplus_ufcs *chip)
{
	chg_info("ufcs force exit!");

	oplus_cpa_request_lock(chip->cpa_topic, UFCS_VOTER);
	oplus_ufcs_set_online(chip, false);
	oplus_ufcs_set_charging(chip, false);
	oplus_ufcs_set_oplus_adapter(chip, false);
	chip->cp_work_mode = CP_WORK_MODE_UNKNOWN;
	chip->cp_ratio = 0;
	chip->start_check = false;
	chip->start_retry_count = 0;
	chip->startup_retry_times = 0;
	chip->emark_imax = 0;
	chip->power_imax = 0;
	/*
	 * Keep ICL clear asynchronous here to avoid exit-path blocking on
	 * WIRED_ICL vote/AICL lock contention (e.g. COMMON_POWER_CHECK voter),
	 * which can noticeably delay UFCS->SVOOC handover.
	 * Residual FCS_ICL_VOTER (if any) is expected to be overridden by the
	 * next protocol initialization path (for example SVOOC re-initialization).
	 * DO NOT add flush_work()/cancel_work_sync() here.
	 */
	oplus_ufcs_clear_fcs_icl(chip);
	oplus_ufcs_push_emark_power(chip, UFCS_POWER(chip->config.target_vbus_mv, 0));
	oplus_ufcs_push_adapter_power(chip, UFCS_POWER(chip->config.target_vbus_mv, 0));
	chip->curr_cc = 0;
	oplus_ufcs_config_cp_watchdog(chip, UFCS_CP_WATCHDOG_DISABLE);
	oplus_ufcs_cp_set_work_start(chip, false);
	oplus_ufcs_exit_ufcs_mode(chip);
	chip->handshake_ok = false;
	msleep(UFCS_STOP_DELAY_TIME);
	oplus_ufcs_cp_enable(chip, false);
	oplus_ufcs_cp_adc_enable(chip, false);
	oplus_ufcs_switch_to_normal(chip);
	oplus_ufcs_cpa_switch_end(chip);
	oplus_ufcs_set_adapter_id(chip, 0);
	vote(chip->ufcs_curr_votable, STEP_VOTER, false, 0, false);
	vote(chip->ufcs_curr_votable, EIS_VOTER, false, 0, false);
	vote(chip->ufcs_not_allow_votable, PLC_SOC_VOTER, false, 0, false);
	vote(chip->ufcs_disable_votable, PLC_RETRY_VOTER, false, 0, false);
	if (is_wired_suspend_votable_available(chip))
		vote(chip->wired_suspend_votable, UFCS_VOTER, false, 0, false);
	if (is_disable_charger_vatable_available(chip))
		vote(chip->chg_disable_votable, UFCS_VOTER, false, 0, false);
	oplus_cpa_request_unlock(chip->cpa_topic, UFCS_VOTER);
	oplus_ufcs_publish_test_mode(chip);
	atomic_set(&chip->temp_recover_debounce_thd, 0);
}

static void oplus_ufcs_soft_exit(struct oplus_ufcs *chip)
{
	oplus_ufcs_set_charging(chip, false);
	chip->cp_work_mode = CP_WORK_MODE_UNKNOWN;
	chip->cp_ratio = 0;
	chip->start_check = false;
	chip->start_retry_count = 0;
	chip->startup_retry_times = 0;
	oplus_ufcs_cp_set_work_start(chip, false);
	oplus_ufcs_exit_ufcs_mode(chip);
	chip->handshake_ok = false;
	chip->curr_cc = 0;
	msleep(UFCS_STOP_DELAY_TIME);
	/*
	 * Same rationale as force_exit(): keep this non-blocking to avoid extra
	 * exit latency from synchronous WIRED_ICL vote lock wait.
	 * Residual FCS_ICL_VOTER (if any) is expected to be overridden by the
	 * next protocol initialization path (for example SVOOC re-initialization).
	 * DO NOT add flush_work()/cancel_work_sync() here.
	 */
	oplus_ufcs_clear_fcs_icl(chip);
	oplus_ufcs_config_cp_watchdog(chip, UFCS_CP_WATCHDOG_DISABLE);
	oplus_ufcs_cp_enable(chip, false);
	oplus_ufcs_cp_adc_enable(chip, false);
	oplus_ufcs_switch_to_normal(chip);
	vote(chip->ufcs_curr_votable, STEP_VOTER, false, 0, false);
	vote(chip->ufcs_curr_votable, EIS_VOTER, false, 0, false);
	vote(chip->ufcs_curr_votable, PLC_VOTER, false, 0, false);
	if (is_wired_suspend_votable_available(chip))
		vote(chip->wired_suspend_votable, UFCS_VOTER, false, 0, false);
	if (is_disable_charger_vatable_available(chip))
		vote(chip->chg_disable_votable, UFCS_VOTER, false, 0, false);
	oplus_ufcs_publish_test_mode(chip);
	atomic_set(&chip->temp_recover_debounce_thd, 0);
}

static int oplus_ufcs_get_verify_data(struct oplus_ufcs *chip, int index)
{
	if (index < 1) {
		chg_err("index(=%d) error\n", index);
		return -EINVAL;
	}
	/* The index of the hal layer starts from 0 */
	index -= 1;

	mutex_lock(&chip->cmd_data_lock);
	memset(&chip->cmd, 0, sizeof(struct ufcs_dev_cmd));
	memset(chip->auth_data, 0, UFCS_VERIFY_AUTH_DATA_SIZE);
	chip->cmd.cmd = UFCS_DEV_CMD_GET_AUTH_DATA;
	chip->cmd.data_size = sizeof(index);
	*(int *)chip->cmd.data_buf = cpu_to_le32(index);
	chip->cmd_data_ok = true;
	mutex_unlock(&chip->cmd_data_lock);
	reinit_completion(&chip->cmd_ack);
	wake_up(&chip->read_wq);

	return 0;
}

static int oplus_ufcs_exit_daemon_process(struct oplus_ufcs *chip)
{
	mutex_lock(&chip->cmd_data_lock);
	memset(&chip->cmd, 0, sizeof(struct ufcs_dev_cmd));
	chip->cmd.cmd = UFCS_DEV_CMD_EXIT;
	chip->cmd.data_size = 0;
	chip->cmd_data_ok = true;
	mutex_unlock(&chip->cmd_data_lock);
	reinit_completion(&chip->cmd_ack);
	wake_up(&chip->read_wq);

	return 0;
}

static void oplus_ufcs_wait_auth_data_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_ufcs *chip =
		container_of(dwork, struct oplus_ufcs, wait_auth_data_work);

	chip->wait_auth_data_done = true;
	vote(chip->ufcs_boot_votable, AUTH_VOTER, false, 0, false);
	(void)oplus_ufcs_exit_daemon_process(chip);

	/* send auth_data to adsp */
	schedule_delayed_work(&chip->send_authdata_to_adsp_work, 0);
}

static int oplus_ufcs_get_emark_ability(struct oplus_ufcs *chip)
{
	enum ufcs_emark_power emark_imax = UFCS_EMARK_POWER_DEFAULT;

	switch (UFCS_OPLUS_VND_EMARK_INFO(chip->emark_info)) {
	case UFCS_OPLUS_EMARK_INFO_ERR:
	case UFCS_OPLUS_EMARK_INFO_OTHER:
		emark_imax = UFCS_EMARK_POWER_DEFAULT;
		return emark_imax;
	case UFCS_OPLUS_EMARK_INFO_OTHER_CABLE:
		emark_imax = UFCS_EMARK_POWER_THIRD;
		return emark_imax;
	case UFCS_OPLUS_EMARK_INFO_OPLUS_CABLE:
		if (chip->config.target_vbus_mv > UFCS_OPLUS_VND_EMARK_VOL_MAX(chip->emark_info)) {
			chg_err("the max withstand voltage of the cable is lower than the target_vbus");
			return -EFAULT;
		}
		emark_imax = UFCS_EMARK_POWER_V0;
		break;
	default:
		chg_err("unknown emark info type\n");
		return -EFAULT;
	}

	switch (UFCS_OPLUS_VND_EMARK_HW_VER(chip->emark_info)) {
	case UFCS_EMARK_ID_V0:
		emark_imax = UFCS_EMARK_POWER_V0;
		break;
	case UFCS_EMARK_ID_V1:
		emark_imax = UFCS_EMARK_POWER_V1;
		break;
	case UFCS_EMARK_ID_V2:
		emark_imax = UFCS_EMARK_POWER_V2;
		break;
	case UFCS_EMARK_ID_V3:
		emark_imax = UFCS_EMARK_POWER_V3;
		break;
	case UFCS_EMARK_ID_V4:
		emark_imax = UFCS_EMARK_POWER_V4;
		break;
	case UFCS_EMARK_ID_V5:
		emark_imax = UFCS_EMARK_POWER_V5;
		break;
	case UFCS_EMARK_ID_V6:
		emark_imax = UFCS_EMARK_POWER_V6;
		break;
	case UFCS_EMARK_ID_VA:
		emark_imax = UFCS_OPLUS_VND_EMARK_CURR_MAX(chip->emark_info);
		break;
	default:
		emark_imax = UFCS_OPLUS_VND_EMARK_CURR_MAX(chip->emark_info);
		break;
	}

	return emark_imax;
}

static int oplus_ufcs_deal_emark_info(struct oplus_ufcs *chip)
{
	int rc;

	rc = oplus_chg_ufcs_get_emark_info(chip, &chip->emark_info);
	if (rc < 0) {
		chg_err("can't get emark info, rc=%d\n", rc);
		chip->emark_info = 0;
		return rc;
	}

	chg_info("emark: cable_type=%s, hw_ver=%llu, curr_max=%llu mA, vol_max=%llumV, \n",
		 ufcs_get_oplus_emark_info_str(UFCS_OPLUS_VND_EMARK_INFO(chip->emark_info)),
		 UFCS_OPLUS_VND_EMARK_HW_VER(chip->emark_info),
		 UFCS_OPLUS_VND_EMARK_CURR_MAX(chip->emark_info),
		 UFCS_OPLUS_VND_EMARK_VOL_MAX(chip->emark_info));

	chip->emark_imax = oplus_ufcs_get_emark_ability(chip);
	if (chip->emark_imax < 0)
		return chip->emark_imax;

	vote(chip->ufcs_curr_votable, CABLE_MAX_VOTER, true, chip->emark_imax, false);
	oplus_ufcs_push_emark_power(chip, UFCS_POWER(chip->config.target_vbus_mv, chip->emark_imax));

	return 0;
}

static enum ufcs_power_imax oplus_ufcs_get_power_ability(struct oplus_ufcs *chip)
{
	int rc, i = 0;
	int power_mw, power_imax = 0;
	int ufcs_power;

	for (i = 0; i < chip->pie_num; i++) {
		chg_info("power_info[%d]: index=%llu, type=%s, curr_max=%llu, vol_max=%llu, power_imax = %d\n", i,
			 UFCS_OPLUS_VND_POWER_INFO_INDEX(chip->pie[i]),
			 ufcs_get_oplus_power_info_type_str(UFCS_OPLUS_VND_POWER_INFO_TYPE(chip->pie[i])),
			 UFCS_OPLUS_VND_POWER_INFO_CURR_MAX(chip->pie[i]),
			 UFCS_OPLUS_VND_POWER_INFO_VOL_MAX(chip->pie[i]), power_imax);

		power_mw = UFCS_OPLUS_VND_POWER_INFO_CURR_MAX(chip->pie[i]) *
			UFCS_OPLUS_VND_POWER_INFO_VOL_MAX(chip->pie[i]) / 1000;
		switch (UFCS_OPLUS_VND_POWER_INFO_TYPE(chip->pie[i])) {
		case UFCS_OPLUS_POWER_INFO_UFCS:
			if ((UFCS_OPLUS_VND_POWER_INFO_CURR_MAX(chip->pie[i]) > power_imax) &&
			    (UFCS_OPLUS_VND_POWER_INFO_VOL_MAX(chip->pie[i]) >= chip->config.target_vbus_mv))
				power_imax = UFCS_OPLUS_VND_POWER_INFO_CURR_MAX(chip->pie[i]);

			rc = oplus_cpa_protocol_get_power(chip->cpa_topic, CHG_PROTOCOL_UFCS);
			if (rc < 0) {
				chg_err("can't get ufcs protocol power info\n");
				continue;
			}
			if (power_mw <= rc) {
				if (chip->retention_state && chip->retention_exit_ufcs_flag == EXIT_HIGH_UFCS)
					power_imax = UFCS_CURR_MAX_V0;
				continue;
			}
			if (chip->retention_state && chip->retention_exit_ufcs_flag == EXIT_HIGH_UFCS) {
				rc = oplus_cpa_protocol_set_power(chip->cpa_topic, CHG_PROTOCOL_UFCS,
					chip->config.ufcs_watt_third);
				power_imax = UFCS_CURR_MAX_V0;
			} else {
				rc = oplus_cpa_protocol_set_power(chip->cpa_topic, CHG_PROTOCOL_UFCS, power_mw);
			}
			if (rc < 0) {
				chg_err("can't set ufcs protocol power data\n");
				continue;
			}
			ufcs_power = oplus_cpa_protocol_get_power(chip->cpa_topic, CHG_PROTOCOL_UFCS);
			if (ufcs_power > 0)
				chip->retention_ufcs_power = ufcs_power;
			break;
		case UFCS_OPLUS_POWER_INFO_PPS:
			rc = oplus_cpa_protocol_get_power(chip->cpa_topic, CHG_PROTOCOL_PPS);
			if (rc < 0) {
				chg_err("can't get pps protocol power info\n");
				continue;
			}
			if (power_mw <= rc)
				continue;
			rc = oplus_cpa_protocol_set_power(chip->cpa_topic, CHG_PROTOCOL_PPS, power_mw);
			if (rc < 0) {
				chg_err("can't set pps protocol power data\n");
				continue;
			}
			break;
		case UFCS_OPLUS_POWER_INFO_SVOOC:
			rc = oplus_cpa_protocol_get_power(chip->cpa_topic, CHG_PROTOCOL_VOOC);
			if (rc < 0) {
				chg_err("can't get vooc protocol power info\n");
				continue;
			}
			if (power_mw <= rc)
				continue;
			rc = oplus_cpa_protocol_set_power(chip->cpa_topic, CHG_PROTOCOL_VOOC, power_mw);
			if (rc < 0) {
				chg_err("can't set vooc protocol power data\n");
				continue;
			}
			break;
		default:
			chg_err("unknown power info type\n");
			continue;
		}
	}

	if (!power_imax)
		power_imax = UFCS_CURR_MAX_V0;

	return power_imax;
}

static int oplus_ufcs_deal_power_info(struct oplus_ufcs *chip)
{
	int rc;
	enum oplus_chg_protocol_type type;
	int power_mw;
	int ibus_ma;

	rc = oplus_chg_ufcs_get_power_info_ext(chip, chip->pie, UFCS_OPLUS_VND_POWER_INFO_MAX);
	if (rc <= 0) {
		chg_err("ufcs get power info ext error\n");
		chip->pie_num = 0;
		return -EINVAL;
	} else {
		chip->pie_num = min(rc, UFCS_OPLUS_VND_POWER_INFO_MAX);
	}
	chip->power_imax = oplus_ufcs_get_power_ability(chip);

	if (chip->keep_topic != NULL) {
		power_mw = oplus_cpa_protocol_get_power(chip->cpa_topic, CHG_PROTOCOL_UFCS);
		if (power_mw > 0) {
			ibus_ma = power_mw * 1000 / chip->config.target_vbus_mv;
			chg_info("ibus_ma=%d, power_mw=%d\n", ibus_ma, power_mw);
			if (ibus_ma < chip->power_imax)
				chip->power_imax = ibus_ma;
		}
	}
	if (chip->power_imax > 0)
		vote(chip->ufcs_curr_votable, ADAPTER_IMAX_VOTER, true, chip->power_imax, false);

	type = oplus_cpa_curr_high_prio_protocol_type(chip->cpa_topic);
	if (type == CHG_PROTOCOL_INVALID || type == CHG_PROTOCOL_UFCS)
		return 0;

	/*
	 * Need to switch to a higher priority protocol, so ufcs needs to
	 * re-request. The return value here should be -EBUSY.
	 */
	oplus_cpa_request(chip->cpa_topic, CHG_PROTOCOL_UFCS);

	return -EBUSY;
}

#define UFCS_LOCAL_T_NS_TO_S_THD		1000000000
static int oplus_chg_ufcs_get_local_time_s(void)
{
	int local_time_s;

	local_time_s = local_clock() / UFCS_LOCAL_T_NS_TO_S_THD;
	chg_info("local_time_s:%d\n", local_time_s);

	return local_time_s;
}

static bool oplus_ufcs_reset_adapter(struct oplus_ufcs *chip)
{
	struct timespec64 uptime;
	static bool need_boot_reset_adapter = false;
	int rc_reset_adapter = 0;

	/*UFCS and SVOOC projects with the same power need config ufcs_need_reset_adapter,
	If the current is greater than 1A and lasts for more than 5s, the adapter will activate and disconnect DP DM*/
	if (!need_boot_reset_adapter && chip->wired_online) {
		ktime_get_boottime_ts64(&uptime);
		if ((unsigned long)uptime.tv_sec < UFCS_BOOT_RESETADAPTER_20S &&
		    oplus_is_power_off_charging()) {
			need_boot_reset_adapter = true;
			chip->reset_adapter = true;
		}
	}

	if (chip->reset_adapter && chip->wired_online &&
	    (is_wired_suspend_votable_available(chip))) {
		chip->reset_adapter = false;
		reinit_completion(&chip->reset_abnormal_ack);
		vote(chip->wired_suspend_votable, UFCS_VOTER, true, 1, false);
		if (chip->wired_online) {
			rc_reset_adapter = wait_for_completion_timeout(&chip->reset_abnormal_ack,
					msecs_to_jiffies(UFCS_RESETADAPTER_SLEEP_1000MS));
			if (!rc_reset_adapter) {
				chg_info("reset adapter sucess\n");
			} else {
				chg_info("unplug when reset adapter\n");
				vote(chip->wired_suspend_votable, UFCS_VOTER, false, 0, false);
				return false;
			}
		}
		vote(chip->wired_suspend_votable, UFCS_VOTER, false, 0, false);
	} else {
		chip->reset_adapter = false;
	}
	return true;
}

static void oplus_ufcs_sub_btb_connnect_check(struct oplus_ufcs *chip)
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
			vote(chip->ufcs_curr_votable, BAD_SUB_BTB_VOTER, true,
			     btb_abnormal_current, false);
	} else {
		vote(chip->ufcs_curr_votable, BAD_SUB_BTB_VOTER, false, 0, false);
	}
}

static bool oplus_ufcs_check_ufcs_continue(struct oplus_ufcs *chip)
{
	if (chip->config.curr_max_ma <= UFCS_VERIFY_CURR_THR_MA && !chip->retention_topic) {
		if (!oplus_cpa_protocol_check_enable(chip->cpa_topic, CHG_PROTOCOL_VOOC))
			return true;
		chg_info("Not support high power UFCS, switch SVOOC");
		return false;
	}
	return true;
}

static int oplus_ufcs_deal_third_power_info(struct oplus_ufcs *chip)
{
	int power_mw = 0;
	int rc = 0;
	int project_power = 0;

	if (NULL == chip)
		return -EINVAL;

	power_mw = chip->config.target_vbus_mv * chip->adapter_max_curr / 1000;

	project_power = chip->third_curve_target_vbus_mv * chip->third_curve_target_ibus_ma / 1000;

	if (!project_power)
		project_power = UFCS_POWER_TYPE_V0;

	power_mw = min(power_mw, project_power);
	rc = oplus_cpa_protocol_set_power(chip->cpa_topic, CHG_PROTOCOL_UFCS, power_mw);
	if (rc < 0) {
		chg_err("can't set ufcs protocol power data\n");
		return -EINVAL;
	}

	chg_info("third power set %d mw.\n", power_mw);
	return 0;
}

static void oplus_ufcs_switch_check_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_ufcs *chip =
		container_of(dwork, struct oplus_ufcs, switch_check_work);
	enum oplus_cp_work_mode cp_mode;
	int rc;
	int i;
	bool pdo_ok = false;
	int max_curr = 0;
	int pdo_vol;
	bool ufcs_boot_retry = false;
	union mms_msg_data msg_data = { 0 };

	if (chip->handshake_ok) {
		chg_info("handshake_ok is true, exit\n");
		return;
	}

	chg_info("ufcs_switch_check_work\n");
	oplus_ufcs_set_charging(chip, false);
	rc = oplus_cpa_switch_start(chip->cpa_topic, CHG_PROTOCOL_UFCS);
	if (rc < 0) {
		chg_info("cpa protocol not ufcs, return\n");
		return;
	}

	oplus_ufcs_variables_early_init(chip);
	oplus_ufcs_votable_reset(chip);
	if (chip->ufcs_disable) {
		oplus_ufcs_cpa_switch_end(chip);
		return;
	}

	if (chip->ufcs_ic == NULL || chip->cp_ic == NULL || !is_gauge_topic_available(chip)) {
		oplus_ufcs_cpa_switch_end(chip);
		return;
	}

	if (chip->prot_crash) {
		chg_info("ufcs protocol core crashed, wait recover\n");
		return;
	}

	if (chip->config.ufcs_need_reset_adapter) {
		if (!oplus_ufcs_reset_adapter(chip))
			goto err;
	} else {
		chip->reset_adapter = false;
	}

	oplus_ufcs_switch_to_ufcs(chip);
	oplus_ufcs_reset_dpdm(chip);
	rc = oplus_ufcs_handshake(chip);
	if (rc < 0) {
		chg_err("ufcs handshake error\n");
		schedule_work(&chip->handshake_error_work);
		/* check if the ufcs can be restarted when power off charger or reboot */
		if (oplus_chg_ufcs_get_local_time_s() < chip->config.ufcs_boot_time_retry) {
			chg_info("need retry to ufcs handshack when bootup handshack failed.\n");
			ufcs_boot_retry = true;
		}
		goto err;
	}
	chip->handshake_ok = true;
	chg_info("ufcs handshake success\n");

	rc = oplus_ufcs_get_dev_info(chip, &chip->dev_info);
	if (rc < 0) {
		chg_err("ufcs get dev info error\n");
		goto err;
	}
	chg_info("dev_info=0x%llx\n", chip->dev_info);
	oplus_ufcs_set_adapter_id(chip, UFCS_DEVICE_INFO_HW_VER(chip->dev_info));

	if (!chip->pr_config.support_pr && oplus_is_ufcs_abnormal_adapter(chip, chip->dev_info)) {
		chg_info("is abnormal ufcs adapter, exit\n");
		goto next;
	}

	if (oplus_chg_ufcs_is_test_mode(chip)) {
		chg_info("test mode\n");
		oplus_ufcs_set_online(chip, true);
		oplus_ufcs_publish_test_mode(chip);
		return;
	}

	rc = oplus_ufcs_config_wd(chip, UFCS_WATCHDOG_TIME_MS);
	if (rc < 0) {
		chg_err("ufcs config watchdog error\n");
		goto err;
	}

	if (UFCS_DEVICE_INFO_IC_VENDOR(chip->dev_info) == 0) {
		if (UFCS_DEVICE_INFO_DEV_VENDOR(chip->dev_info) == UFCS_OPLUS_DEV_ID) {
			oplus_ufcs_set_ufcs_vid(chip, UFCS_OPLUS_DEV_ID);
			if (!oplus_ufcs_check_ufcs_continue(chip))
				goto next;
			if (unlikely(!chip->auth_data_ok)) {
				chg_err("auth data not ready");
			} else if (chip->adapter_verify_fail_flag && chip->retention_state) {
				chg_info("oplus adapter check third ufcs\n");
				chip->adapter_check_third_ufcs = true;
			} else {
				rc = oplus_ufcs_verify_adapter(chip, 1, chip->auth_data, UFCS_VERIFY_AUTH_DATA_SIZE);
				if (rc < 0) {
					chg_err("adapter verify error, rc=%d\n", rc);
					goto next;
				} else if (!!rc) {
					chg_info("adapter verify pass\n");
					oplus_ufcs_set_oplus_adapter(chip, true);
				} else {
					chg_err("adapter verify fail\n");
					if (chip->retention_exit_ufcs_flag != EXIT_THIRD_UFCS)
						chip->adapter_verify_fail_flag = true;
					oplus_ufcs_track_upload_err_info(chip, TRACK_UFCS_ERR_AUTHER_ERR, 0);
					goto next;
				}
			}
		} else if (UFCS_DEVICE_INFO_DEV_VENDOR(chip->dev_info) == 0) {
			chg_err("abnormal adapter id");
		}
	}
	chg_info("oplus_ufcs_adapter=%s\n", chip->oplus_ufcs_adapter ? "true" : "false");

	if (chip->oplus_ufcs_adapter) {
		rc = oplus_ufcs_deal_emark_info(chip);
		if (rc < 0)
			goto err;
		rc = oplus_ufcs_deal_power_info(chip);
		if (rc < 0)
			goto next;
	}
	oplus_ufcs_set_online(chip, true);
	oplus_ufcs_set_plc_strategy(chip);

	rc = oplus_ufcs_get_pdo_info(chip, chip->pdo, UFCS_OUTPUT_MODE_MAX);
	if (rc < 0) {
		chg_err("ufcs get pdo info error\n");
		goto err;
	}
	chip->pdo_num = rc;
	for (i = 0; i < chip->pdo_num; i++)
		chg_info("pdo[%d]: voltage[%llu-%llumV, step:%llumV], current[%llu-%llumA, step:%llumA]\n", i,
			 UFCS_OUTPUT_MODE_VOL_MIN(chip->pdo[i]), UFCS_OUTPUT_MODE_VOL_MAX(chip->pdo[i]),
			 UFCS_OUTPUT_MODE_VOL_STEP(chip->pdo[i]), UFCS_OUTPUT_MODE_CURR_MIN(chip->pdo[i]),
			 UFCS_OUTPUT_MODE_CURR_MAX(chip->pdo[i]), UFCS_OUTPUT_MODE_CURR_STEP(chip->pdo[i]));
	for (i = 0; i < chip->pdo_num; i++) {
		int target_vbus = chip->config.target_vbus_mv;
		if (i > 0) {
			if (UFCS_OUTPUT_MODE_VOL_MIN(chip->pdo[i]) >
			    UFCS_OUTPUT_MODE_VOL_MAX(chip->pdo[i - 1])) {
				chg_err("the output voltage range is discontinuous\n");
				break;
			}
		}
		if (target_vbus <= UFCS_OUTPUT_MODE_VOL_MAX(chip->pdo[i]) &&
		    target_vbus >= UFCS_OUTPUT_MODE_VOL_MIN(chip->pdo[i])) {
			pdo_ok = true;
			if (UFCS_OUTPUT_MODE_CURR_MAX(chip->pdo[i]) > max_curr) {
				max_curr = UFCS_OUTPUT_MODE_CURR_MAX(chip->pdo[i]);
				chip->adapter_max_curr = max_curr;
			}
		}
	}
	if (!pdo_ok) {
		chg_err("The pdo range of the current adapter does not support UFCS charging\n");
		oplus_ufcs_track_upload_err_info(chip, TRACK_UFCS_ERR_PDO_ERR, 0);
		goto err;
	}
	vote(chip->ufcs_curr_votable, BASE_MAX_VOTER, true, max_curr, false);
	oplus_ufcs_push_adapter_power(chip, UFCS_POWER(chip->config.target_vbus_mv, max_curr));

	if (!chip->oplus_ufcs_adapter) {
		rc = oplus_ufcs_deal_third_power_info(chip);
		if (rc < 0)
                        goto err;
	}

/* TODO
	rc = oplus_ufcs_get_cable_info(chip, &chip->cable_info);
	if (rc < 0) {
		chg_err("can't get cable info, rc=%d\n", rc);
		chip->cable_info = 0;
	} else {
		chg_err("cable_info=0x%llx\n", chip->cable_info);
	}
*/

	oplus_ufcs_variables_init(chip);
	if (!oplus_ufcs_charge_allow_check(chip)) {
		chg_info("ufcs charge not allow, exit ufcs mode\n");
		goto exit;
	}

	cp_mode = vbus_to_cp_work_mode(chip->config.target_vbus_mv);
	if (cp_mode == CP_WORK_MODE_UNKNOWN) {
		chg_err("can't find a suitable CP working mode\n");
		goto err;
	}
	rc = oplus_ufcs_cp_check_work_mode_support(chip, cp_mode);
	if (rc <= 0) {
		chg_err("cp not support %s mode, rc=%d\n", oplus_cp_work_mode_str(cp_mode), rc);
		goto err;
	}
	rc = oplus_ufcs_cp_set_work_mode(chip, cp_mode);
	if (rc < 0) {
		chg_err("cp set %s mode error, rc=%d\n", oplus_cp_work_mode_str(cp_mode), rc);
		goto err;
	}
	chip->cp_work_mode = cp_mode;
	oplus_ufcs_set_ic_ufcs_curr_vote(chip, cp_mode);
	oplus_ufcs_cp_adc_enable(chip, true);

	rc = oplus_ufcs_config_cp_watchdog(chip, UFCS_CP_WATCHDOG_TIMEOUT_MS);
	if (rc < 0)
		chg_err("ufcs config cp watchdog error.");

	rc = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_EIS_STATUS, &msg_data, false);
	if ((rc == 0) && (msg_data.intval == EIS_STATUS_PREPARE)) {
		chg_info("<EIS> go to ufcs_eis_work\n");
		chip->eis_status = EIS_STATUS_PREPARE;
		oplus_ufcs_push_eis_status(chip, EIS_STATUS_HIGH_CURRENT);
		schedule_delayed_work(&chip->ufcs_eis_work, 0);
		return;
	}

	if (oplus_chg_get_fcs_support_flags()) {
		if (is_disable_charger_vatable_available(chip))
			vote(chip->chg_disable_votable, UFCS_VOTER, true, 1, false);
		oplus_ufcs_set_fcs_icl_vote(UFCS_FCS_ICL1_MA);
	} else {
		if (is_wired_suspend_votable_available(chip))
			vote(chip->wired_suspend_votable, UFCS_VOTER, true, 1, false);
	}

	pdo_vol = UFCS_OUTPUT_MODE_VOL_MIN(chip->pdo[0]) > UFCS_START_DEF_VOL_MV ?
		  UFCS_OUTPUT_MODE_VOL_MIN(chip->pdo[0]) : UFCS_START_DEF_VOL_MV;
	pdo_vol = pdo_vol > UFCS_START_MAX_VOL_MV ? UFCS_START_MAX_VOL_MV : pdo_vol;
	rc = oplus_ufcs_pdo_set(chip, pdo_vol, UFCS_START_DEF_CURR_MA);
	if (rc < 0) {
		chg_err("pdo set error, rc=%d\n", rc);
		goto err;
	}

	oplus_ufcs_sub_btb_connnect_check(chip);
	schedule_delayed_work(&chip->monitor_work, 0);

	return;
err:
	chg_err("error, ufcs exit\n");
next:

	if (ufcs_boot_retry) {
		oplus_ufcs_soft_exit(chip);
		chg_info("retry ufcs check \n");
		schedule_delayed_work(&chip->switch_check_work, msecs_to_jiffies(2000));
	} else {
		oplus_ufcs_force_exit(chip);
	}
	return;

exit:
	chip->handshake_ok = false;
	oplus_ufcs_exit_ufcs_mode(chip);
	chip->cp_work_mode = CP_WORK_MODE_UNKNOWN;
	chip->cp_ratio = 0;
}

enum {
	OPLUS_UFCS_VOLT_UPDATE_V1 = 100,
	OPLUS_UFCS_VOLT_UPDATE_V2 = 200,
	OPLUS_UFCS_VOLT_UPDATE_V3 = 500,
	OPLUS_UFCS_VOLT_UPDATE_V4 = 1000,
	OPLUS_UFCS_VOLT_UPDATE_V5 = 2000,
	OPLUS_UFCS_VOLT_UPDATE_V6 = 5000,
};

static int oplus_ufcs_charge_start(struct oplus_ufcs *chip)
{
	union mms_msg_data data = { 0 };
	static int vbus_err_times = 0;
	int vbat_mv;
	int rc;
	int target_vbus, update_size, req_vol;
	int cp_vin, cp_vout;
	const char temp_region[] = "temp_region";
	const char allow_soc[] = "allow_soc";
	int batt_num;
	int i;

#define UFCS_RETRY_COUNT	1
#define UFCS_CHANGE_VBUS_ERR_TIMES	3
#define UFCS_START_VOL_THR_4_TO_1_MV	600
#define UFCS_START_VOL_THR_3_TO_1_MV	400
#define UFCS_DEFAULT_START_VOL_THR_2_TO_1_MV	200
#define UFCS_START_VOL_THR_2_TO_1_MV	300
#define UFCS_START_VOL_THR_1_TO_1_MV	450
#define UFCS_START_RETAY_MAX		3
#define UFCS_START_CHECK_DELAY_MS	500
#define UFCS_START_PDO_DELAY_MS		50
#define ADSP_UFCS_START_PDO_DELAY_MS	100

	batt_num = oplus_gauge_get_batt_num();
	if (chip->lift_vbus_use_cpvout) {
		rc = oplus_ufcs_cp_get_vout(chip, &cp_vout);
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

	switch (chip->cp_work_mode) {
	case CP_WORK_MODE_4_TO_1:
		target_vbus = ((vbat_mv * 4) / 100) * 100 + UFCS_START_VOL_THR_4_TO_1_MV;
		break;
	case CP_WORK_MODE_3_TO_1:
		target_vbus = ((vbat_mv * 3) / 100) * 100 + UFCS_START_VOL_THR_3_TO_1_MV;
		break;
	case CP_WORK_MODE_2_TO_1:
		if (chip->config.high_imp_compensation_setting_mv > 0) {
			target_vbus = ((vbat_mv * 2) / 100) * 100 + UFCS_DEFAULT_START_VOL_THR_2_TO_1_MV +
					chip->config.high_imp_compensation_setting_mv;
		} else {
			target_vbus = ((vbat_mv * 2) / 100) * 100 + UFCS_START_VOL_THR_2_TO_1_MV * batt_num;
		}
		break;
	case CP_WORK_MODE_BYPASS:
		target_vbus = (vbat_mv / 100) * 100 + UFCS_START_VOL_THR_1_TO_1_MV;
		break;
	default:
		chg_err("unsupported cp work mode, mode=%d\n", chip->cp_work_mode);
		return -ENOTSUPP;
	}

	rc = oplus_ufcs_cp_get_vin(chip, &cp_vin);
	if (rc < 0) {
		chg_err("can't get cp input voltage, rc=%d\n", rc);
		return rc;
	}
	chg_info("cp_vin=%d target_vbus=%d vbat=%d start_check=%d\n", cp_vin, target_vbus, vbat_mv, chip->start_check);
	if ((cp_vin >= target_vbus && cp_vin <= (target_vbus + chip->config.upper_compensation_mv))
		|| chip->start_check) {
		if (chip->start_check) {
			bool work_start;

			rc = oplus_ufcs_cp_get_work_status(chip, &work_start);
			if (rc < 0) {
				chg_err("can't get cp work status, rc=%d\n", rc);
			} else {
				if (work_start) {
					if (oplus_chg_get_fcs_support_flags() && is_wired_suspend_votable_available(chip))
						vote(chip->wired_suspend_votable, UFCS_VOTER, true, 1, false);
					rc = oplus_ufcs_temp_cur_range_init(chip);
					if (rc < 0)
						return rc;

					if (chip->temperature_strategy)
						oplus_chg_strategy_init(chip->temperature_strategy);
					if (chip->oplus_ufcs_adapter)
						chip->strategy = chip->oplus_curve_strategy;
					else
						chip->strategy = chip->third_curve_strategy;
					oplus_chg_strategy_set_process_data(chip->strategy, temp_region, chip->ufcs_temp_cur_range);
					oplus_chg_strategy_set_process_data(chip->strategy, allow_soc, chip->allow_check_soc);
					rc = oplus_chg_strategy_init(chip->strategy);
					if (rc < 0) {
						chg_err("strategy_init error, not support ufcs fast charge\n");
						return rc;
					}
					oplus_ufcs_ccd_strategy_init(chip);
					if (chip->oplus_ufcs_adapter) {
						for (i = 0; i < chip->oplus_lcf_num && chip->oplus_lcf_strategy[i]; i++) {
							rc = oplus_chg_strategy_init(chip->oplus_lcf_strategy[i]);
							if (rc < 0)
								chg_err("oplus_lcf_strategy_init %d error\n", i);
						}
					} else {
						for (i = 0; i < chip->third_lcf_num && chip->third_lcf_strategy[i]; i++) {
							rc = oplus_chg_strategy_init(chip->third_lcf_strategy[i]);
							if (rc < 0)
								chg_err("third_lcf_strategy_init %d error\n", i);
						}
					}

					if (chip->start_retry_count > 0) {
						oplus_ufcs_track_upload_err_info(chip, TRACK_UFCS_ERR_CP_ENABLE,
							chip->start_retry_count);
					}
					chip->start_retry_count = 0;
					chip->startup_retry_times = 0;
					chip->start_check = false;
					oplus_ufcs_set_charging(chip, true);
					oplus_ufcs_vfa_reset(chip);
					if (chip->oplus_ufcs_adapter && !chip->pr_config.support_pr)
						chip->target_vbus_mv = chip->config.target_vbus_mv;
					else
						chip->target_vbus_mv = chip->vol_set_mv;
					rc = oplus_ufcs_pdo_set(chip, chip->target_vbus_mv, UFCS_START_CP_CURR_MA);
					if (rc < 0) {
						chg_err("pdo set error, rc=%d\n", rc);
						return rc;
					}
					chip->timer.monitor_jiffies = jiffies;
					oplus_ufcs_deep_ratio_limit_curr(chip);

					if (chip->pr_config.support_pr) {
						ufcs_pr_init(chip);
						schedule_delayed_work(&chip->current_work,
							msecs_to_jiffies(UFCS_PR_START_DELAY_MS));
					} else {
						schedule_delayed_work(&chip->current_work,
							msecs_to_jiffies(UFCS_START_CHECK_DELAY_MS));
					}

					oplus_ufcs_cp_reg_dump(chip);
					return 0;
				}
				chg_err("cp not work, retry=%d\n", chip->start_retry_count);
			}
			if (chip->start_retry_count >= UFCS_START_RETAY_MAX) {
				oplus_ufcs_track_upload_err_info(
					chip, TRACK_UFCS_ERR_CP_ENABLE, chip->start_retry_count + 1);
				chip->start_retry_count = 0;
				chip->start_check = false;
				oplus_ufcs_cp_set_work_start(chip, false);
				oplus_ufcs_cp_enable(chip, false);
				oplus_ufcs_cp_adc_enable(chip, false);
				oplus_ufcs_cp_reg_dump(chip);
				chg_err("cp not working\n");
				return -EFAULT;
			}
			chip->start_retry_count++;
			rc = oplus_ufcs_cp_set_work_start(chip, false);
			if (rc < 0) {
				chg_err("set cp work start error, rc=%d\n", rc);
				return rc;
			}
			rc = oplus_ufcs_cp_set_work_mode(chip, chip->cp_work_mode);
			if (rc < 0) {
				chg_err("cp set %s mode error, rc=%d\n",
					oplus_cp_work_mode_str(chip->cp_work_mode), rc);
				return rc;
			}
			rc = oplus_ufcs_config_cp_watchdog(chip, UFCS_CP_WATCHDOG_TIMEOUT_MS);
			if (rc < 0) {
				chg_err("ufcs config cp watchdog error,rc=%d\n", rc);
				return rc;
			}
			oplus_ufcs_cp_adc_enable(chip, true);
			goto update_vol;
		}
		if (oplus_chg_get_fcs_support_flags()) {
			oplus_ufcs_set_fcs_icl_vote(UFCS_FCS_ICL2_MA);
			rc = oplus_ufcs_pdo_set(chip, chip->vol_set_mv, UFCS_START_CP_CURR_MA);
			if (rc < 0) {
				chg_err("pdo set error, rc=%d\n", rc);
				return rc;
			}
		}
		rc = oplus_ufcs_cp_enable(chip, true);
		if (rc < 0 && (rc != -ENOTSUPP)) {
			chg_err("set cp enable error, rc=%d\n", rc);
			return rc;
		}
		rc = oplus_ufcs_cp_set_work_start(chip, true);
		if (rc < 0) {
			chg_err("set cp work start error, rc=%d\n", rc);
			return rc;
		}
		chip->start_check = true;

		return UFCS_START_CHECK_DELAY_MS;
	}

update_vol:
	chip->start_check = false;

	if (abs(cp_vin - target_vbus) >= OPLUS_UFCS_VOLT_UPDATE_V6)
		update_size = OPLUS_UFCS_VOLT_UPDATE_V6;
	else if (abs(cp_vin - target_vbus) >= OPLUS_UFCS_VOLT_UPDATE_V5)
		update_size = OPLUS_UFCS_VOLT_UPDATE_V5;
	else if (abs(cp_vin - target_vbus) >= OPLUS_UFCS_VOLT_UPDATE_V4)
		update_size = OPLUS_UFCS_VOLT_UPDATE_V4;
	else if (abs(cp_vin - target_vbus) >= OPLUS_UFCS_VOLT_UPDATE_V3)
		update_size = OPLUS_UFCS_VOLT_UPDATE_V3;
	else if (abs(cp_vin - target_vbus) >= OPLUS_UFCS_VOLT_UPDATE_V2)
		update_size = OPLUS_UFCS_VOLT_UPDATE_V2;
	else
		update_size = OPLUS_UFCS_VOLT_UPDATE_V1;

	req_vol = chip->vol_set_mv;
	if (cp_vin < target_vbus) {
		req_vol += update_size;
		if (chip->vol_set_mv >= chip->config.target_vbus_mv) {
			vbus_err_times++;
			if (vbus_err_times >= UFCS_CHANGE_VBUS_ERR_TIMES) {
				vbus_err_times = 0;
				if (chip->startup_retry_times < UFCS_RETRY_COUNT) {
					chg_info(" retry startup");
					chip->startup_retry_times++;
				} else {
					chg_info("stop ufcs");
					oplus_ufcs_track_upload_err_info(chip, TRACK_UFCS_ERR_STARTUP_FAIL, cp_vin);
					return -EFAULT;
				}
			}
		} else {
			vbus_err_times = 0;
		}
	} else {
		req_vol -= update_size;
		vbus_err_times = 0;
	}
	if (req_vol > chip->config.target_vbus_mv)
		req_vol = chip->config.target_vbus_mv;
	chg_info("cp_vin=%d, target_vbus=%d, update_size=%d, req_vol=%d\n",
		 cp_vin, target_vbus, update_size, req_vol);

	rc = oplus_ufcs_cp_set_iin(chip, UFCS_START_DEF_CURR_MA);
	if (rc < 0) {
		chg_err("set cp input current error, rc=%d\n", rc);
		return rc;
	}
	rc = oplus_ufcs_pdo_set(chip, req_vol, UFCS_START_DEF_CURR_MA);
	if (rc < 0) {
		chg_err("pdo set error, rc=%d\n", rc);
		return rc;
	}

	if (chip->pr_config.support_pr)
		return UFCS_PR_START_DELAY_MS;

	if (chip->config.adsp_ufcs_project)
		return ADSP_UFCS_START_PDO_DELAY_MS;

	return UFCS_START_PDO_DELAY_MS;
}

static void oplus_ufcs_reset_temp_range(struct oplus_ufcs *chip)
{
	chip->limits.ufcs_normal_high_temp =
		chip->limits.default_ufcs_normal_high_temp;
	chip->limits.ufcs_little_cold_temp =
		chip->limits.default_ufcs_little_cold_temp;
	chip->limits.ufcs_cool_temp = chip->limits.default_ufcs_cool_temp;
	chip->limits.ufcs_little_cool_temp =
		chip->limits.default_ufcs_little_cool_temp;
	chip->limits.ufcs_little_cool_high_temp = chip->limits.default_ufcs_little_cool_high_temp;
	chip->limits.ufcs_normal_low_pre_temp =
		chip->limits.default_ufcs_normal_low_pre_temp;
	chip->limits.ufcs_normal_low_temp =
		chip->limits.default_ufcs_normal_low_temp;
}

static int oplus_ufcs_set_current_warm_range(struct oplus_ufcs *chip,
					     int vbat_temp_cur)
{
	int ret = chip->limits.ufcs_strategy_normal_current;

	if (chip->limits.ufcs_batt_over_high_temp != -EINVAL &&
	    vbat_temp_cur > chip->limits.ufcs_batt_over_high_temp) {
		chip->limits.ufcs_strategy_change_count++;
		if (chip->limits.ufcs_strategy_change_count >=
		    UFCS_TEMP_OVER_COUNTS) {
			chip->limits.ufcs_strategy_change_count = 0;
			chip->ufcs_fastchg_batt_temp_status =
				UFCS_BAT_TEMP_EXIT;
			ret = chip->limits.ufcs_over_high_or_low_current;
			chg_err("temp_over:%d", vbat_temp_cur);
		}
	} else if (vbat_temp_cur < chip->limits.ufcs_normal_high_temp) {
		chip->limits.ufcs_strategy_change_count++;
		if (chip->limits.ufcs_strategy_change_count >=
		    UFCS_TEMP_OVER_COUNTS) {
			chip->limits.ufcs_strategy_change_count = 0;
			chip->ufcs_fastchg_batt_temp_status =
				UFCS_BAT_TEMP_SWITCH_CURVE;
			chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_INIT;
			ret = chip->limits.ufcs_strategy_normal_current;
			oplus_ufcs_reset_temp_range(chip);
			(void)oplus_chg_strategy_init(chip->strategy);
			chip->limits.ufcs_normal_high_temp +=
				UFCS_TEMP_WARM_RANGE_THD;
			chg_err("switch temp range:%d", vbat_temp_cur);
		}

	} else {
		chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_WARM;
		chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_WARM;
		ret = chip->limits.ufcs_strategy_normal_current;
	}

	return ret;
}

static int
oplus_ufcs_set_current_temp_normal_range(struct oplus_ufcs *chip,
					 int vbat_temp_cur)
{
	int ret = chip->limits.ufcs_strategy_normal_current;
	int batt_vol;
	union mms_msg_data data = { 0 };
	int rc;

	if (!is_gauge_topic_available(chip)) {
		chg_err("gauge topic not found\n");
		vote(chip->ufcs_disable_votable, NO_DATA_VOTER, true, 1, false);
		return -ENODEV;
	}
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX, &data, true);
	if (rc < 0) {
		chg_err("can't get vbat, rc=%d\n", rc);
		vote(chip->ufcs_disable_votable, NO_DATA_VOTER, true, 1, false);
		return rc;
	}
	batt_vol = data.intval;

	switch (chip->ufcs_fastchg_batt_temp_status) {
	case UFCS_BAT_TEMP_NORMAL_HIGH:
		if (vbat_temp_cur >
		    chip->limits.ufcs_strategy_batt_high_temp0) {
			chip->ufcs_fastchg_batt_temp_status =
				UFCS_BAT_TEMP_HIGH0;
			ret = chip->limits.ufcs_strategy_high_current0;
		} else if (vbat_temp_cur >= chip->limits.ufcs_normal_low_temp) {
			chip->ufcs_fastchg_batt_temp_status =
				UFCS_BAT_TEMP_NORMAL_HIGH;
			ret = chip->limits.ufcs_strategy_normal_current;
		} else {
			chip->ufcs_fastchg_batt_temp_status =
				UFCS_BAT_TEMP_NORMAL_LOW;
			chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_NORMAL_LOW;
			ret = chip->limits.ufcs_strategy_normal_current;
			oplus_ufcs_reset_temp_range(chip);
			chip->limits.ufcs_normal_low_temp +=
				UFCS_TEMP_WARM_RANGE_THD;
		}
		break;
	case UFCS_BAT_TEMP_HIGH0:
		if (vbat_temp_cur >
		    chip->limits.ufcs_strategy_batt_high_temp1) {
			chip->ufcs_fastchg_batt_temp_status =
				UFCS_BAT_TEMP_HIGH1;
			ret = chip->limits.ufcs_strategy_high_current1;
		} else if (vbat_temp_cur <
			   chip->limits.ufcs_strategy_batt_low_temp0) {
			chip->ufcs_fastchg_batt_temp_status =
				UFCS_BAT_TEMP_LOW0;
			ret = chip->limits.ufcs_strategy_low_current0;
		} else {
			chip->ufcs_fastchg_batt_temp_status =
				UFCS_BAT_TEMP_HIGH0;
			ret = chip->limits.ufcs_strategy_high_current0;
		}
		break;
	case UFCS_BAT_TEMP_HIGH1:
		if (vbat_temp_cur >
		    chip->limits.ufcs_strategy_batt_high_temp2) {
			chip->ufcs_fastchg_batt_temp_status =
				UFCS_BAT_TEMP_HIGH2;
			ret = chip->limits.ufcs_strategy_high_current2;
		} else if (vbat_temp_cur <
			   chip->limits.ufcs_strategy_batt_low_temp1) {
			chip->ufcs_fastchg_batt_temp_status =
				UFCS_BAT_TEMP_LOW1;
			ret = chip->limits.ufcs_strategy_low_current1;
		} else {
			chip->ufcs_fastchg_batt_temp_status =
				UFCS_BAT_TEMP_HIGH1;
			ret = chip->limits.ufcs_strategy_high_current1;
		}
		break;
	case UFCS_BAT_TEMP_HIGH2:
		if (chip->limits.ufcs_normal_high_temp != -EINVAL &&
		    vbat_temp_cur > chip->limits.ufcs_normal_high_temp) {
			chip->limits.ufcs_strategy_change_count++;
			if (chip->limits.ufcs_strategy_change_count >=
			    UFCS_TEMP_OVER_COUNTS) {
				chip->limits.ufcs_strategy_change_count = 0;
				if (chip->ui_soc <= chip->limits.ufcs_warm_allow_soc &&
				    batt_vol <= chip->limits.ufcs_warm_allow_vol) {
					chip->ufcs_fastchg_batt_temp_status =
						UFCS_BAT_TEMP_SWITCH_CURVE;
					chip->ufcs_temp_cur_range =
						UFCS_TEMP_RANGE_INIT;
					ret = chip->limits.ufcs_strategy_high_current2;
					oplus_ufcs_reset_temp_range(chip);
					(void)oplus_chg_strategy_init(chip->strategy);
					chip->limits.ufcs_normal_high_temp -=
						UFCS_TEMP_WARM_RANGE_THD;
					chg_err("switch temp range:%d", vbat_temp_cur);
				} else {
					chg_err("high temp_over:%d", vbat_temp_cur);
					chip->ufcs_fastchg_batt_temp_status =
						UFCS_BAT_TEMP_EXIT;
					ret = chip->limits.ufcs_over_high_or_low_current;
				}
			}
		} else if (chip->limits.ufcs_batt_over_high_temp != -EINVAL &&
			   vbat_temp_cur >
				   chip->limits.ufcs_batt_over_high_temp) {
			chip->limits.ufcs_strategy_change_count++;
			if (chip->limits.ufcs_strategy_change_count >=
			    UFCS_TEMP_OVER_COUNTS) {
				chip->limits.ufcs_strategy_change_count = 0;
				chip->ufcs_fastchg_batt_temp_status =
					UFCS_BAT_TEMP_EXIT;
				ret = chip->limits.ufcs_over_high_or_low_current;
				chg_err("over_high temp_over:%d", vbat_temp_cur);
			}
		} else if (vbat_temp_cur <
			   chip->limits.ufcs_strategy_batt_low_temp2) {
			chip->ufcs_fastchg_batt_temp_status =
				UFCS_BAT_TEMP_LOW2;
			ret = chip->limits.ufcs_strategy_low_current2;
		} else {
			chip->ufcs_fastchg_batt_temp_status =
				UFCS_BAT_TEMP_HIGH2;
			ret = chip->limits.ufcs_strategy_high_current2;
		}
		break;
	case UFCS_BAT_TEMP_LOW0:
		if (vbat_temp_cur >
		    chip->limits.ufcs_strategy_batt_high_temp0) {
			chip->ufcs_fastchg_batt_temp_status =
				UFCS_BAT_TEMP_HIGH0;
			ret = chip->limits.ufcs_strategy_high_current0;
		} else if (vbat_temp_cur <
			   chip->limits.ufcs_strategy_batt_low_temp0) { /* T<39C */
			chip->ufcs_fastchg_batt_temp_status =
				UFCS_BAT_TEMP_NORMAL_HIGH;
			ret = chip->limits.ufcs_strategy_normal_current;
		} else {
			chip->ufcs_fastchg_batt_temp_status =
				UFCS_BAT_TEMP_LOW0;
			ret = chip->limits.ufcs_strategy_low_current0;
		}
		break;
	case UFCS_BAT_TEMP_LOW1:
		if (vbat_temp_cur >
		    chip->limits.ufcs_strategy_batt_high_temp1) {
			chip->ufcs_fastchg_batt_temp_status =
				UFCS_BAT_TEMP_HIGH1;
			ret = chip->limits.ufcs_strategy_high_current2;
		} else if (vbat_temp_cur <
			   chip->limits.ufcs_strategy_batt_low_temp0) {
			chip->ufcs_fastchg_batt_temp_status =
				UFCS_BAT_TEMP_LOW0;
			ret = chip->limits.ufcs_strategy_low_current0;
		} else {
			chip->ufcs_fastchg_batt_temp_status =
				UFCS_BAT_TEMP_LOW1;
			ret = chip->limits.ufcs_strategy_low_current1;
		}
		break;
	case UFCS_BAT_TEMP_LOW2:
		if (vbat_temp_cur >
		    chip->limits.ufcs_strategy_batt_high_temp2) {
			chip->ufcs_fastchg_batt_temp_status =
				UFCS_BAT_TEMP_HIGH2;
			ret = chip->limits.ufcs_strategy_high_current2;
		} else if (vbat_temp_cur <
			   chip->limits.ufcs_strategy_batt_low_temp1) {
			chip->ufcs_fastchg_batt_temp_status =
				UFCS_BAT_TEMP_LOW1;
			ret = chip->limits.ufcs_strategy_low_current1;
		} else {
			chip->ufcs_fastchg_batt_temp_status =
				UFCS_BAT_TEMP_LOW2;
			ret = chip->limits.ufcs_strategy_low_current2;
		}
		break;
	default:
		break;
	}
	chg_info("the ret: %d, the temp =%d, status = %d\n", ret,
		 vbat_temp_cur, chip->ufcs_fastchg_batt_temp_status);
	return ret;
}

static int
oplus_ufcs_set_current_temp_low_normal_range(struct oplus_ufcs *chip,
					     int vbat_temp_cur)
{
	int ret = chip->limits.ufcs_strategy_normal_current;
	int start_temp = chip->limits.ufcs_little_cool_temp;

	if (chip->limits.ufcs_little_cool_high_temp != -EINVAL)
		start_temp = chip->limits.ufcs_little_cool_high_temp;
	if (chip->limits.ufcs_normal_low_pre_temp != -EINVAL)
		start_temp = chip->limits.ufcs_normal_low_pre_temp;

	if (vbat_temp_cur < chip->limits.ufcs_normal_low_temp &&
	    vbat_temp_cur >= start_temp) { /* 20C<=T<35C */
		chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_NORMAL_LOW;
		chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_NORMAL_LOW;
		ret = chip->limits.ufcs_strategy_normal_current;
	} else {
		if (vbat_temp_cur >= chip->limits.ufcs_normal_low_temp) {
			chip->limits.ufcs_normal_low_temp -=
				UFCS_TEMP_LOW_RANGE_THD;
			chip->ufcs_fastchg_batt_temp_status =
				UFCS_BAT_TEMP_NORMAL_HIGH;
			ret = chip->limits.ufcs_strategy_normal_current;
			chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_NORMAL_HIGH;
		} else {
			if (chip->limits.ufcs_normal_low_pre_temp != -EINVAL) {
				chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_NORMAL_LOW_PRE;
				chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_NORMAL_LOW_PRE;
				ret = chip->limits.ufcs_strategy_normal_current;
				oplus_ufcs_reset_temp_range(chip);
				chip->limits.ufcs_normal_low_pre_temp += UFCS_TEMP_LOW_RANGE_THD;
			} else if (chip->limits.ufcs_little_cool_high_temp != -EINVAL) {
				chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_LITTLE_COOL_HIGH;
				chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_LITTLE_COOL_HIGH;
				ret = chip->limits.ufcs_strategy_normal_current;
				oplus_ufcs_reset_temp_range(chip);
				chip->limits.ufcs_little_cool_high_temp += UFCS_TEMP_LOW_RANGE_THD;
			} else {
				chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_LITTLE_COOL;
				chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_LITTLE_COOL;
				ret = chip->limits.ufcs_strategy_normal_current;
				oplus_ufcs_reset_temp_range(chip);
				chip->limits.ufcs_little_cool_temp += UFCS_TEMP_LOW_RANGE_THD;
			}
		}
	}

	return ret;
}

static int oplus_ufcs_set_current_temp_normal_low_pre_range(struct oplus_ufcs *chip, int vbat_temp_cur)
{
	int ret = 0;
	int start_temp = chip->limits.ufcs_little_cool_high_temp;

	if (chip->limits.ufcs_little_cool_high_temp == -EINVAL)
		start_temp = chip->limits.ufcs_little_cool_temp;

	if (vbat_temp_cur < chip->limits.ufcs_normal_low_pre_temp &&
	    vbat_temp_cur >= start_temp) {
		chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_NORMAL_LOW_PRE;
		ret = chip->limits.ufcs_strategy_normal_current;
		chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_NORMAL_LOW_PRE;
	} else if (vbat_temp_cur >= chip->limits.ufcs_normal_low_pre_temp) {
		if (chip->ui_soc <= chip->high_soc) {
			chip->limits.ufcs_strategy_change_count++;
			if (chip->limits.ufcs_strategy_change_count >= UFCS_TEMP_OVER_COUNTS) {
				chip->limits.ufcs_strategy_change_count = 0;
				chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_SWITCH_CURVE;
				chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_INIT;
				(void)oplus_chg_strategy_init(chip->strategy);
				chg_err("switch temp range:%d", vbat_temp_cur);
			}
		} else {
			chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_NORMAL_LOW;
			chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_NORMAL_LOW;
		}
		ret = chip->limits.ufcs_strategy_normal_current;
		oplus_ufcs_reset_temp_range(chip);
		chip->limits.ufcs_normal_low_pre_temp -= UFCS_TEMP_LOW_RANGE_THD;
	} else {
		if (chip->limits.ufcs_little_cool_high_temp != -EINVAL) {
			chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_LITTLE_COOL_HIGH;
			chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_LITTLE_COOL_HIGH;
			ret = chip->limits.ufcs_strategy_normal_current;
			oplus_ufcs_reset_temp_range(chip);
			chip->limits.ufcs_little_cool_high_temp += UFCS_TEMP_LOW_RANGE_THD;
		} else {
			chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_LITTLE_COOL;
			chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_LITTLE_COOL;
			ret = chip->limits.ufcs_strategy_normal_current;
			oplus_ufcs_reset_temp_range(chip);
			chip->limits.ufcs_little_cool_temp += UFCS_TEMP_LOW_RANGE_THD;
		}
	}

	return ret;
}

static int oplus_ufcs_set_current_temp_little_cool_high_range(struct oplus_ufcs *chip, int vbat_temp_cur)
{
	int ret = 0;

	if (vbat_temp_cur < chip->limits.ufcs_little_cool_high_temp &&
	    vbat_temp_cur >= chip->limits.ufcs_little_cool_temp) {
		chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_LITTLE_COOL_HIGH;
		ret = chip->limits.ufcs_strategy_normal_current;
		chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_LITTLE_COOL_HIGH;
	} else if (vbat_temp_cur >= chip->limits.ufcs_little_cool_high_temp) {
		if (chip->ui_soc <= chip->high_soc) {
			chip->limits.ufcs_strategy_change_count++;
			if (chip->limits.ufcs_strategy_change_count >= UFCS_TEMP_OVER_COUNTS) {
				chip->limits.ufcs_strategy_change_count = 0;
				chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_SWITCH_CURVE;
				chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_INIT;
				(void)oplus_chg_strategy_init(chip->strategy);
				chg_err("switch temp range:%d", vbat_temp_cur);
			}
		} else {
			if (chip->limits.ufcs_normal_low_pre_temp != -EINVAL &&
			    vbat_temp_cur < chip->limits.ufcs_normal_low_pre_temp) {
				chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_NORMAL_LOW_PRE;
				chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_NORMAL_LOW_PRE;
			} else {
				chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_NORMAL_LOW;
				chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_NORMAL_LOW;
			}
		}
		ret = chip->limits.ufcs_strategy_normal_current;
		oplus_ufcs_reset_temp_range(chip);
		chip->limits.ufcs_little_cool_high_temp -= UFCS_TEMP_LOW_RANGE_THD;
	} else {
		chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_LITTLE_COOL;
		chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_LITTLE_COOL;
		ret = chip->limits.ufcs_strategy_normal_current;
		oplus_ufcs_reset_temp_range(chip);
		chip->limits.ufcs_little_cool_temp += UFCS_TEMP_LOW_RANGE_THD;
	}

	return ret;
}

static int
oplus_ufcs_set_current_temp_little_cool_range(struct oplus_ufcs *chip,
					      int vbat_temp_cur)
{
	int ret = 0;

	if (vbat_temp_cur < chip->limits.ufcs_little_cool_temp &&
	    vbat_temp_cur >= chip->limits.ufcs_cool_temp) { /* 12C<=T<20C */
		chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_LITTLE_COOL;
		ret = chip->limits.ufcs_strategy_normal_current;
		chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_LITTLE_COOL;
	} else if (vbat_temp_cur >= chip->limits.ufcs_little_cool_temp) {
		if (chip->limits.ufcs_little_cool_high_temp != -EINVAL) {
			if (chip->ui_soc <= chip->high_soc) {
				chip->limits.ufcs_strategy_change_count++;
				if (chip->limits.ufcs_strategy_change_count >= UFCS_TEMP_OVER_COUNTS) {
					chip->limits.ufcs_strategy_change_count = 0;
					chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_SWITCH_CURVE;
					chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_INIT;
					(void)oplus_chg_strategy_init(chip->strategy);
					chg_err("switch temp range:%d", vbat_temp_cur);
				}
			} else {
				chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_LITTLE_COOL_HIGH;
				chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_LITTLE_COOL_HIGH;
			}
			ret = chip->limits.ufcs_strategy_normal_current;
			oplus_ufcs_reset_temp_range(chip);
			chip->limits.ufcs_little_cool_temp -= UFCS_TEMP_LOW_RANGE_THD;
		} else {
			chip->limits.ufcs_little_cool_temp -= UFCS_TEMP_LOW_RANGE_THD;
			if (chip->limits.ufcs_normal_low_pre_temp != -EINVAL) {
				chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_NORMAL_LOW_PRE;
				chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_NORMAL_LOW_PRE;
			} else {
				chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_NORMAL_LOW;
				chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_NORMAL_LOW;
			}
			ret = chip->limits.ufcs_strategy_normal_current;
		}
	} else {
		if (chip->ui_soc <= chip->high_soc) {
			chip->limits.ufcs_strategy_change_count++;
			if (chip->limits.ufcs_strategy_change_count >=
			    UFCS_TEMP_OVER_COUNTS) {
				chip->limits.ufcs_strategy_change_count = 0;
				chip->ufcs_fastchg_batt_temp_status =
					UFCS_BAT_TEMP_SWITCH_CURVE;
				chip->ufcs_temp_cur_range =
					UFCS_TEMP_RANGE_INIT;
				(void)oplus_chg_strategy_init(chip->strategy);
				chg_err("switch temp range:%d", vbat_temp_cur);
			}
		} else {
			chip->ufcs_fastchg_batt_temp_status =
				UFCS_BAT_TEMP_COOL;
			chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_COOL;
		}
		ret = chip->limits.ufcs_strategy_normal_current;
		oplus_ufcs_reset_temp_range(chip);
		chip->limits.ufcs_cool_temp += UFCS_TEMP_LOW_RANGE_THD;
	}

	return ret;
}

static int oplus_ufcs_set_current_temp_cool_range(struct oplus_ufcs *chip,
						  int vbat_temp_cur)
{
	int ret = chip->limits.ufcs_strategy_normal_current;

	if (chip->limits.ufcs_batt_over_low_temp != -EINVAL &&
	    vbat_temp_cur < chip->limits.ufcs_batt_over_low_temp) {
		chip->limits.ufcs_strategy_change_count++;
		if (chip->limits.ufcs_strategy_change_count >=
		    UFCS_TEMP_OVER_COUNTS) {
			chip->limits.ufcs_strategy_change_count = 0;
			chip->ufcs_fastchg_batt_temp_status =
				UFCS_BAT_TEMP_EXIT;
			ret = chip->limits.ufcs_over_high_or_low_current;
			chg_err("temp_over:%d", vbat_temp_cur);
		}
	} else if (vbat_temp_cur < chip->limits.ufcs_cool_temp &&
		   vbat_temp_cur >=
			   chip->limits.ufcs_little_cold_temp) { /* 5C <=T<12C */
		chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_COOL;
		chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_COOL;
		ret = chip->limits.ufcs_strategy_normal_current;
	} else if (vbat_temp_cur >= chip->limits.ufcs_cool_temp) {
		chip->limits.ufcs_strategy_change_count++;
		if (chip->limits.ufcs_strategy_change_count >=
		    UFCS_TEMP_OVER_COUNTS) {
			chip->limits.ufcs_strategy_change_count = 0;
			if (chip->ui_soc <= chip->high_soc) {
				chip->ufcs_fastchg_batt_temp_status =
					UFCS_BAT_TEMP_SWITCH_CURVE;
				chip->ufcs_temp_cur_range =
					UFCS_TEMP_RANGE_INIT;
				(void)oplus_chg_strategy_init(chip->strategy);
				chg_info("switch temp range:%d", vbat_temp_cur);
			} else {
				chip->ufcs_fastchg_batt_temp_status =
					UFCS_BAT_TEMP_LITTLE_COOL;
				chip->ufcs_temp_cur_range =
					UFCS_TEMP_RANGE_LITTLE_COOL;
			}
			oplus_ufcs_reset_temp_range(chip);
			chip->limits.ufcs_cool_temp -= UFCS_TEMP_LOW_RANGE_THD;
		}

		ret = chip->limits.ufcs_strategy_normal_current;
	} else {
		chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_LITTLE_COLD;
		chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_LITTLE_COLD;
		ret = chip->limits.ufcs_strategy_normal_current;
		oplus_ufcs_reset_temp_range(chip);
		chip->limits.ufcs_little_cold_temp += UFCS_TEMP_LOW_RANGE_THD;
	}
	return ret;
}

static int
oplus_ufcs_set_current_temp_little_cold_range(struct oplus_ufcs *chip,
					      int vbat_temp_cur)
{
	int ret = chip->limits.ufcs_strategy_normal_current;

	if (chip->limits.ufcs_batt_over_low_temp != -EINVAL &&
	    vbat_temp_cur < chip->limits.ufcs_batt_over_low_temp) {
		chip->limits.ufcs_strategy_change_count++;
		if (chip->limits.ufcs_strategy_change_count >=
		    UFCS_TEMP_OVER_COUNTS) {
			chip->limits.ufcs_strategy_change_count = 0;
			ret = chip->limits.ufcs_over_high_or_low_current;
			chip->ufcs_fastchg_batt_temp_status =
				UFCS_BAT_TEMP_EXIT;
			chg_err("temp_over:%d", vbat_temp_cur);
		}
	} else if (vbat_temp_cur <
		   chip->limits.ufcs_little_cold_temp) { /* 0C<=T<5C */
		chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_LITTLE_COLD;
		chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_LITTLE_COLD;
		ret = chip->limits.ufcs_strategy_normal_current;
	} else {
		chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_COOL;
		ret = chip->limits.ufcs_strategy_normal_current;
		chip->ufcs_temp_cur_range = UFCS_TEMP_RANGE_COOL;
		oplus_ufcs_reset_temp_range(chip);
		chip->limits.ufcs_little_cold_temp -= UFCS_TEMP_LOW_RANGE_THD;
	}

	return ret;
}

static int oplus_ufcs_get_batt_temp_curr(struct oplus_ufcs *chip)

{
	int ret, rc;
	int vbat_temp_cur;
	union mms_msg_data data = { 0 };
	int temperature_current = 0;

	if (!is_gauge_topic_available(chip)) {
		chg_err("gauge topic not found\n");
		vote(chip->ufcs_disable_votable, NO_DATA_VOTER, true, 1, false);
		return -ENODEV;
	}

	rc = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_SHELL_TEMP, &data, false);
	if (rc < 0) {
		chg_err("can't get shell temp data, rc=%d", rc);
	} else {
		chip->shell_temp = data.intval;
	}

	vbat_temp_cur = chip->shell_temp;
	ret = chip->limits.ufcs_strategy_normal_current;
	switch (chip->ufcs_temp_cur_range) {
	case UFCS_TEMP_RANGE_WARM:
		ret = oplus_ufcs_set_current_warm_range(chip, vbat_temp_cur);
		break;
	case UFCS_TEMP_RANGE_NORMAL_HIGH:
		ret = oplus_ufcs_set_current_temp_normal_range(chip,
							       vbat_temp_cur);
		break;
	case UFCS_TEMP_RANGE_NORMAL_LOW:
		ret = oplus_ufcs_set_current_temp_low_normal_range(
			chip, vbat_temp_cur);
		break;
	case UFCS_TEMP_RANGE_LITTLE_COOL:
		ret = oplus_ufcs_set_current_temp_little_cool_range(
			chip, vbat_temp_cur);
		break;
	case UFCS_TEMP_RANGE_LITTLE_COOL_HIGH:
		ret = oplus_ufcs_set_current_temp_little_cool_high_range(
			chip, vbat_temp_cur);
		break;
	case UFCS_TEMP_RANGE_NORMAL_LOW_PRE:
		ret = oplus_ufcs_set_current_temp_normal_low_pre_range(
			chip, vbat_temp_cur);
		break;
	case UFCS_TEMP_RANGE_COOL:
		ret = oplus_ufcs_set_current_temp_cool_range(chip,
							     vbat_temp_cur);
		break;
	case UFCS_TEMP_RANGE_LITTLE_COLD:
		ret = oplus_ufcs_set_current_temp_little_cold_range(
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
		vote(chip->ufcs_curr_votable, BATT_TEMP_VOTER, true, ret, false);
	else if (ret == 0)
		ret = -EINVAL;

	return ret;
}

static void oplus_ufcs_check_low_curr_temp_status(struct oplus_ufcs *chip)
{
	static int t_cnts = 0;
	int vbat_temp_cur;
	union mms_msg_data data = { 0 };
	int rc;

	if (!is_gauge_topic_available(chip)) {
		chg_err("gauge topic not found\n");
		vote(chip->ufcs_disable_votable, NO_DATA_VOTER, true, 1, false);
		return;
	}

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_REAL_TEMP, &data, true);
	if (rc < 0) {
		chg_err("can't get battery temp, rc=%d\n", rc);
		vote(chip->ufcs_disable_votable, NO_DATA_VOTER, true, 1, false);
		return;
	}
	vbat_temp_cur = data.intval;

	if (((vbat_temp_cur >=
	      chip->limits.ufcs_low_curr_full_normal_high_temp) ||
	     (vbat_temp_cur < chip->limits.ufcs_low_curr_full_cool_temp)) &&
	    (chip->ufcs_low_curr_full_temp_status !=
	     UFCS_LOW_CURR_FULL_CURVE_TEMP_MAX)) {
		t_cnts++;
		if (t_cnts >= UFCS_TEMP_OVER_COUNTS) {
			chip->ufcs_low_curr_full_temp_status =
				UFCS_LOW_CURR_FULL_CURVE_TEMP_MAX;
			t_cnts = 0;
		}
	} else if ((vbat_temp_cur >=
		    chip->limits.ufcs_low_curr_full_normal_low_temp) &&
		   (chip->ufcs_low_curr_full_temp_status !=
		    UFCS_LOW_CURR_FULL_CURVE_TEMP_NORMAL_HIGH)) {
		t_cnts++;
		if (t_cnts >= UFCS_TEMP_OVER_COUNTS) {
			chip->ufcs_low_curr_full_temp_status =
				UFCS_LOW_CURR_FULL_CURVE_TEMP_NORMAL_HIGH;
			t_cnts = 0;
		}
	} else if ((vbat_temp_cur >=
		    chip->limits.ufcs_low_curr_full_little_cool_temp) &&
		   (chip->ufcs_low_curr_full_temp_status !=
		    UFCS_LOW_CURR_FULL_CURVE_TEMP_NORMAL_LOW)) {
		t_cnts++;
		if (t_cnts >= UFCS_TEMP_OVER_COUNTS) {
			chip->ufcs_low_curr_full_temp_status =
				UFCS_LOW_CURR_FULL_CURVE_TEMP_NORMAL_LOW;
			t_cnts = 0;
		}
	} else if (chip->ufcs_low_curr_full_temp_status !=
		   UFCS_LOW_CURR_FULL_CURVE_TEMP_LITTLE_COOL) {
		t_cnts++;
		if (t_cnts >= UFCS_TEMP_OVER_COUNTS) {
			chip->ufcs_low_curr_full_temp_status =
				UFCS_LOW_CURR_FULL_CURVE_TEMP_LITTLE_COOL;
			t_cnts = 0;
		}
	} else {
		t_cnts = 0;
	}
}

static void oplus_ufcs_check_sw_full(struct oplus_ufcs *chip, struct puc_strategy_ret_data *data)
{
	int cool_sw_vth, normal_sw_vth, normal_hw_vth;
	struct oplus_ufcs_config *config = &chip->config;
	union mms_msg_data mms_data = { 0 };
	int batt_temp, vbat_mv;
	int rc;
#define UFCS_FULL_COUNTS_COOL		6
#define UFCS_FULL_COUNTS_SW		6
#define UFCS_FULL_COUNTS_HW		3
#define UFCS_PR_FULL_COUNTS_HW		12
	int hw_full_cnt = chip->pr_config.support_pr ? UFCS_PR_FULL_COUNTS_HW : UFCS_FULL_COUNTS_HW;

	if (!chip->oplus_ufcs_adapter) {
		cool_sw_vth = chip->limits.ufcs_full_cool_sw_vbat_third;
		normal_sw_vth = chip->limits.ufcs_full_normal_sw_vbat_third;
		normal_hw_vth = chip->limits.ufcs_full_normal_hw_vbat_third;
	} else {
		cool_sw_vth = chip->limits.ufcs_full_cool_sw_vbat;
		normal_sw_vth = chip->limits.ufcs_full_normal_sw_vbat;
		normal_hw_vth = chip->limits.ufcs_full_normal_hw_vbat;
	}

	if (!is_gauge_topic_available(chip)) {
		chg_err("gauge topic not found\n");
		vote(chip->ufcs_disable_votable, NO_DATA_VOTER, true, 1, false);
		return;
	}
	rc = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_SHELL_TEMP, &mms_data, true);
	if (rc < 0) {
		chg_err("can't get battery temp, rc=%d\n", rc);
		vote(chip->ufcs_disable_votable, NO_DATA_VOTER, true, 1, false);
		return;
	}
	batt_temp = mms_data.intval;
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX, &mms_data, false);
	if (rc < 0) {
		chg_err("can't get vbat, rc=%d\n", rc);
		vote(chip->ufcs_disable_votable, NO_DATA_VOTER, true, 1, false);
		return;
	}
	vbat_mv = mms_data.intval;

	if (!chip->pr_config.support_pr && (batt_temp < chip->limits.ufcs_cool_temp) && (vbat_mv > cool_sw_vth)) {
		chip->count.cool_fw++;
		if (chip->count.cool_fw >= UFCS_FULL_COUNTS_COOL) {
			chip->count.cool_fw = 0;
			chg_info("batt_temp:%d vbat_mv:%d cool sw full\n", batt_temp, vbat_mv);
			if (chip->config.ufcs_full_recheck_temp != -EINVAL &&
			    batt_temp <= config->ufcs_full_recheck_temp) {
				chip->ufcs_cool_full_temp_range = chip->ufcs_temp_cur_range;
				vote(chip->ufcs_not_allow_votable, CHG_FULL_COOL_VOTER, true, 1, false);
			} else {
				vote(chip->ufcs_disable_votable, CHG_FULL_VOTER, true, 1, false);
			}
			return;
		}
	} else {
		chip->count.cool_fw = 0;
	}

	if ((batt_temp >= chip->limits.ufcs_cool_temp) &&
	    (batt_temp < chip->limits.ufcs_batt_over_high_temp)) {
		if (!chip->pr_config.support_pr && (vbat_mv > normal_sw_vth) && data->last_gear) {
			chip->count.sw_full++;
			if (chip->count.sw_full >= UFCS_FULL_COUNTS_SW) {
				chip->count.sw_full = 0;
				chg_info("batt_temp:%d vbat_mv:%d normal sw full\n", batt_temp, vbat_mv);
				if (chip->config.ufcs_full_recheck_temp != -EINVAL &&
				    batt_temp <= config->ufcs_full_recheck_temp) {
					chip->ufcs_cool_full_temp_range = chip->ufcs_temp_cur_range;
					vote(chip->ufcs_not_allow_votable, CHG_FULL_COOL_VOTER, true, 1, false);
				} else {
					vote(chip->ufcs_disable_votable, CHG_FULL_VOTER, true, 1, false);
				}
				return;
			}
		} else {
			chip->count.sw_full = 0;
		}

		if ((vbat_mv > normal_hw_vth)) {
			chip->count.hw_full++;
			if (chip->count.hw_full >= hw_full_cnt) {
				chip->count.hw_full = 0;
				chg_info("batt_temp:%d vbat_mv:%d normal hw full\n", batt_temp, vbat_mv);
				if (chip->config.ufcs_full_recheck_temp != -EINVAL &&
				    batt_temp <= config->ufcs_full_recheck_temp) {
					chip->ufcs_cool_full_temp_range = chip->ufcs_temp_cur_range;
					vote(chip->ufcs_not_allow_votable, CHG_FULL_COOL_VOTER, true, 1, false);
				} else {
					vote(chip->ufcs_disable_votable, CHG_FULL_VOTER, true, 1, false);
				}
				return;
			}
		} else {
			chip->count.hw_full = 0;
		}
	} else {
		chip->count.sw_full = 0;
		chip->count.hw_full = 0;
	}

	if ((chip->ufcs_fastchg_batt_temp_status == UFCS_BAT_TEMP_WARM) &&
	    (vbat_mv > chip->limits.ufcs_full_warm_vbat)) {
		chip->count.warm_fw++;
		if (chip->count.warm_fw >= UFCS_FULL_COUNTS_COOL) {
			chip->count.warm_fw = 0;
			vote(chip->ufcs_not_allow_votable, CHG_FULL_WARM_VOTER, true, 1, false);
			return;
		}
	} else {
		chip->count.warm_fw = 0;
	}
}

static void oplus_ufcs_check_low_curr_full(struct oplus_ufcs *chip)
{
	int i, temp_current, temp_vbatt, temp_status, iterm, vterm;
	bool low_curr = false;
	union mms_msg_data data = { 0 };
	int rc;

#define UFCS_FULL_COUNTS_LOW_CURR	6

	oplus_ufcs_check_low_curr_temp_status(chip);

	if (!is_gauge_topic_available(chip)) {
		chg_err("gauge topic not found\n");
		vote(chip->ufcs_disable_votable, NO_DATA_VOTER, true, 1, false);
		return;
	}
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX, &data, true);
	if (rc < 0) {
		chg_err("can't get vbat, rc=%d\n", rc);
		vote(chip->ufcs_disable_votable, NO_DATA_VOTER, true, 1, false);
		return;
	}
	temp_vbatt = data.intval;
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CURR, &data, true);
	if (rc < 0) {
		chg_err("can't get ibat, rc=%d\n", rc);
		vote(chip->ufcs_disable_votable, NO_DATA_VOTER, true, 1, false);
		return;
	}
	temp_current = -data.intval;

	temp_status = chip->ufcs_low_curr_full_temp_status;
	if (temp_status == UFCS_LOW_CURR_FULL_CURVE_TEMP_MAX)
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
		if (chip->count.low_curr_full > UFCS_FULL_COUNTS_LOW_CURR) {
			chip->count.low_curr_full = 0;
			vote(chip->ufcs_disable_votable, CHG_FULL_VOTER, true, 1, false);
		}
	} else {
		chip->count.low_curr_full = 0;
	}
}

static void oplus_ufcs_check_timeout(struct oplus_ufcs *chip)
{
	unsigned long tmp_time;
	if (chip->plc_status == PLC_STATUS_ENABLE) {
		chip->timer.monitor_jiffies = jiffies;
		return;
	}
	tmp_time = jiffies - chip->timer.monitor_jiffies;
	chip->timer.monitor_jiffies = jiffies;
	if (chip->timer.ufcs_max_time_ms <= jiffies_to_msecs(tmp_time)) {
		chip->timer.ufcs_max_time_ms = 0;
		vote(chip->ufcs_disable_votable, TIMEOUT_VOTER, true, 1, false);
		return;
	}
	chip->timer.ufcs_max_time_ms -= jiffies_to_msecs(tmp_time);
}

static void oplus_ufcs_update_low_curr_full(struct oplus_ufcs *chip)
{
	int rc;
	int ret_val = 0;
	int i;
	int batt_alarm = 0;
	const char alarm[] = "alarm";

	if (oplus_get_chg_spec_version() >= OPLUS_CHG_SPEC_VER_V3P7) {
		if (chip->oplus_ufcs_adapter) {
			for (i = 0; i < chip->oplus_lcf_num && chip->oplus_lcf_strategy[i]; i++) {
				rc = oplus_chg_strategy_get_data(chip->oplus_lcf_strategy[i], &ret_val);
				if (rc < 0) {
					chg_err("can't get lcf_strategy %d data, rc=%d\n", i, rc);
				} else if (ret_val) {
					vote(chip->ufcs_disable_votable, CHG_FULL_VOTER, true, 1, false);
					chg_info("CHG_FULL_VOTER is true, diable ufcs charger\n");
					break;
				}
				rc = oplus_chg_strategy_get_custom_data(chip->oplus_lcf_strategy[i], alarm, &ret_val);
				if (rc >= 0 && ret_val)
					batt_alarm |= BIT(i);
			}
		} else {
			for (i = 0; i < chip->third_lcf_num && chip->third_lcf_strategy[i]; i++) {
				rc = oplus_chg_strategy_get_data(chip->third_lcf_strategy[i], &ret_val);
				if (rc < 0) {
					chg_err("can't get lcf_strategy %d data, rc=%d\n", i, rc);
				} else if (ret_val) {
					vote(chip->ufcs_disable_votable, CHG_FULL_VOTER, true, 1, false);
					chg_info("CHG_FULL_VOTER is true, diable ufcs charger\n");
					break;
				}
				rc = oplus_chg_strategy_get_custom_data(chip->third_lcf_strategy[i], alarm, &ret_val);
				if (rc >= 0 && ret_val)
					batt_alarm |= BIT(i);
			}
		}
		if (chip->batt_alarm != batt_alarm) {
			chip->batt_alarm = batt_alarm;
			oplus_ufcs_push_lcf_alarm_status(chip, batt_alarm);
		}
	} else {
		oplus_ufcs_check_low_curr_full(chip);
	}
}

static void oplus_ufcs_check_full(struct oplus_ufcs *chip, struct puc_strategy_ret_data *data)
{
	oplus_ufcs_check_sw_full(chip, data);
	oplus_ufcs_update_low_curr_full(chip);
}

static bool oplus_ufcs_btb_temp_check(struct oplus_ufcs *chip)
{
	bool btb_status = true;
	int btb_temp, usb_temp, cp_temp;
	static unsigned char temp_over_count = 0;
	bool shaft_btb_normal = true;

#define UFCS_BTB_USB_OVER_CNTS		9

	btb_temp = oplus_wired_get_batt_btb_temp();
	usb_temp = oplus_wired_get_usb_btb_temp();
	cp_temp = oplus_ufcs_cp_get_temp(chip);
	shaft_btb_normal = oplus_wired_get_shaft_btb_is_normal();

	if (btb_temp >= UFCS_BTB_TEMP_MAX ||
	    usb_temp >= UFCS_USB_TEMP_MAX ||
	    cp_temp >= UFCS_CP_TEMP_MAX || !shaft_btb_normal) {
		temp_over_count++;
		if (temp_over_count > UFCS_BTB_USB_OVER_CNTS) {
			btb_status = false;
			chg_err("btb[%d] or usb[%d] or mos[%d]  is_shaft_btb_normal[%d] temp over\n",
				btb_temp, usb_temp, cp_temp, shaft_btb_normal);
			if (btb_temp >= UFCS_BTB_TEMP_MAX)
				oplus_ufcs_track_upload_err_info(chip, TRACK_UFCS_ERR_BTB_OVER, btb_temp);
			else if (cp_temp >= UFCS_CP_TEMP_MAX)
				oplus_ufcs_track_upload_err_info(chip, TRACK_UFCS_ERR_MOS_OVER, cp_temp);
			else if (!shaft_btb_normal)
				oplus_wired_set_shaft_btb_over(true);
			else
				oplus_ufcs_track_upload_err_info(chip, TRACK_UFCS_ERR_USBTEMP_OVER, usb_temp);
		}
	} else {
		temp_over_count = 0;
	}

	if (ufcs_debug_track) {
		chg_info(" %d\n", ufcs_debug_track);
		oplus_ufcs_track_upload_err_info(chip, TRACK_UFCS_ERR_USBTEMP_OVER, usb_temp);
		ufcs_debug_track = 0;
	}
	return btb_status;
}

#define UFCS_IBAT_LOW_MIN 2000
#define UFCS_IBAT_LOW_MIN_SINGLE 3000
#define UFCS_IBAT_UCP_CNT (7 / UFCS_UPDATE_IBAT_TIME) /* 7s */
#define UFCS_UCP_SS_IBUS_MIN 600
static void oplus_ufcs_check_ibat_safety(struct oplus_ufcs *chip)
{
	union mms_msg_data data = { 0 };
	int ibat;
	int chg_ith;
	int rc;
	int ibat_low_th = (oplus_gauge_get_batt_num() == 1) ? UFCS_IBAT_LOW_MIN_SINGLE : UFCS_IBAT_LOW_MIN;
	int ibus = 0;

#define UFCS_IBAT_HIGH_CNT	8

	if (time_is_after_jiffies(chip->timer.ibat_jiffies + (unsigned long)(UFCS_UPDATE_IBAT_TIME * HZ)))
		return;
	chip->timer.ibat_jiffies = jiffies;
	ibus = UFCS_SOURCE_INFO_CURR(chip->src_info);

	if (ibus > UFCS_UCP_SS_IBUS_MIN) {
		oplus_ufcs_cp_ss_timeout_enable(chip, true);
		oplus_ufcs_cp_set_pmid2vout_ovp_enable(chip, true);
	}


	if (!is_gauge_topic_available(chip)) {
		chg_err("gauge topic not found\n");
		vote(chip->ufcs_disable_votable, NO_DATA_VOTER, true, 1, false);
		return;
	}
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CURR, &data, true);
	if (rc < 0) {
		chg_err("can't get ibat, rc=%d\n", rc);
		vote(chip->ufcs_disable_votable, NO_DATA_VOTER, true, 1, false);
		return;
	}
	ibat = data.intval;

	if (!chip->oplus_ufcs_adapter)
		chg_ith = -chip->limits.ufcs_ibat_over_third;
	else
		chg_ith = -chip->limits.ufcs_ibat_over_oplus;
	if (ibat < chg_ith) {
		chip->count.ibat_high++;
		if (chip->count.ibat_high >= UFCS_IBAT_HIGH_CNT) {
			vote(chip->ufcs_disable_votable, IBAT_OVER_VOTER, true, 1, false);
			oplus_ufcs_track_upload_err_info(chip, TRACK_UFCS_ERR_IBAT_OVER, ibat);
			return;
		}
	} else {
		chip->count.ibat_high = 0;
	}

	if (chip->plc_status == PLC_STATUS_ENABLE)
		return;

	if (ibat > ibat_low_th) {
		chip->count.ibat_low++;
		if (chip->count.ibat_low > UFCS_IBAT_UCP_CNT) {
			vote(chip->ufcs_disable_votable, IBAT_OVER_VOTER, true, 1, false);
			oplus_ufcs_track_upload_err_info(chip, TRACK_UFCS_ERR_IBAT_OVER, ibat);
			return;
		}
	} else {
		chip->count.ibat_low = 0;
	}
}

static void oplus_ufcs_check_temp(struct oplus_ufcs *chip)
{
	struct timespec ts_current;
	union mms_msg_data data = { 0 };
	int batt_temp;
	int rc;

#define UFCS_FG_TEMP_PROTECTION	800
#define UFCS_TFG_OV_CNT		6
#define UFCS_BTB_OV_CNT		8
#define UFCS_TBATT_OV_CNT	1
#define UFCS_FG_CONTACT_CNT	2

	ts_current = oplus_current_kernel_time();
	if ((ts_current.tv_sec - chip->timer.temp_timer.tv_sec) < UFCS_UPDATE_TEMP_TIME)
		return;

	chip->timer.temp_timer = ts_current;

	if (!oplus_ufcs_btb_temp_check(chip)) {
		chip->count.btb_high++;
		if (chip->count.btb_high >= UFCS_BTB_OV_CNT) {
			chip->count.btb_high = 0;
			vote(chip->ufcs_disable_votable, BTB_TEMP_OVER_VOTER, true, 1, false);
			oplus_ufcs_track_upload_err_info(chip, TRACK_UFCS_ERR_BTB_OVER, 0);
			if (is_wired_icl_votable_available(chip))
				vote(chip->wired_icl_votable, BTB_TEMP_OVER_VOTER, true,
				     BTB_TEMP_OVER_MAX_INPUT_CUR, true);
		}
	} else {
		chip->count.btb_high = 0;
	}

	if (chip->ufcs_fastchg_batt_temp_status == UFCS_BAT_TEMP_EXIT) {
		chg_err("ufcs battery temp out of range\n");
		chip->count.tbatt_over++;
		if (chip->count.tbatt_over >= UFCS_TBATT_OV_CNT)
			vote(chip->ufcs_not_allow_votable, BATT_TEMP_VOTER, true, 1, false);
	} else {
		chip->count.tbatt_over = 0;
	}

	if (chip->ufcs_fastchg_batt_temp_status == UFCS_BAT_TEMP_SWITCH_CURVE) {
		chg_err("ufcs battery temp switch curve range\n");
		if (chip->target_curr_ma >= UFCS_QUIT_CP_OVER_CURR) {
			cancel_delayed_work_sync(&chip->current_work);
			oplus_ufcs_pdo_set(chip, chip->target_vbus_mv, UFCS_QUIT_CP_OVER_CURR);
			msleep(UFCS_WAIT_CURR_DOWN_TIMES);
		}
		vote(chip->ufcs_disable_votable, SWITCH_RANGE_VOTER, true, 1, false);
	}

	if (is_gauge_topic_available(chip)) {
		rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_REAL_TEMP, &data, true);
		if (rc >= 0) {
			batt_temp = data.intval;
			if (batt_temp >= UFCS_FG_TEMP_PROTECTION) {
				chg_err("ufcs tfg > 80\n");
				chip->count.tfg_over++;
				if (chip->count.tfg_over >= UFCS_TFG_OV_CNT) {
					chip->count.tfg_over = 0;
					vote(chip->ufcs_disable_votable, TFG_VOTER, true, 1, false);
					oplus_ufcs_track_upload_err_info(chip, TRACK_UFCS_ERR_TFG_OVER, batt_temp);
				}
			} else if (batt_temp < chip->limits.ufcs_removed_bat_decidegc) {
				chg_err("ufcs tfg < -39\n");
				chip->count.tfg_over++;
				if (chip->count.tfg_over >= UFCS_FG_CONTACT_CNT) {
					chip->count.tfg_over = 0;
					vote(chip->ufcs_not_allow_votable, BATT_TEMP_VOTER, true, 1, false);
					oplus_ufcs_track_upload_err_info(chip, TRACK_UFCS_ERR_TFG_OVER, batt_temp);
				}
			} else {
				chip->count.tfg_over = 0;
			}
		}
	}
}

#define BCC_CURRENT_MIN_STEP		(500 / 100)
static void oplus_ufcs_check_bcc_curr(struct oplus_ufcs *chip, struct puc_strategy_ret_data *data)
{
	int bcc_min_curr, bcc_max_curr, bcc_exit_curr;
	struct puc_strategy_temp_curves curve = { 0 };
	int curr_ratio;
	int rc = 0;

	switch (chip->cp_work_mode) {
	case CP_WORK_MODE_4_TO_1:
		curr_ratio = 4;
		break;
	case CP_WORK_MODE_3_TO_1:
		curr_ratio = 3;
		break;
	case CP_WORK_MODE_2_TO_1:
		curr_ratio = 2;
		break;
	case CP_WORK_MODE_BYPASS:
		curr_ratio = 1;
		break;
	default:
		chg_err("unsupported cp work mode, mode=%d\n", chip->cp_work_mode);
		return;
	}

	bcc_min_curr = data->target_ibus * curr_ratio / 100; /* mA to 0.1A */
	bcc_max_curr = data->target_ibus * curr_ratio / 100; /* mA to 0.1A */
	rc = oplus_chg_strategy_get_metadata(chip->strategy, &curve);
	if (rc < 0 || !curve.data || curve.num < 1) {
		chg_err("can't get charger curve %d 0x%p %d\n", rc, curve.data, curve.num);
		return;
	}
	bcc_exit_curr = curve.data[curve.num - 1].target_ibus * curr_ratio;

	if ((bcc_min_curr - UFCS_BCC_CURRENT_MIN) < (bcc_exit_curr / UFCS_BATT_CURR_TO_BCC_CURR))
		bcc_min_curr = bcc_exit_curr / UFCS_BATT_CURR_TO_BCC_CURR;
	else
		bcc_min_curr = ((bcc_min_curr - UFCS_BCC_CURRENT_MIN) / BCC_CURRENT_MIN_STEP) * BCC_CURRENT_MIN_STEP;

	chip->bcc.bcc_min_curr = bcc_min_curr;
	chip->bcc.bcc_max_curr = bcc_max_curr;
	chip->bcc.bcc_exit_curr = bcc_exit_curr;
	chg_debug("bcc_info: index=%d, min=%d, max=%d, exit=%d\n",
		  data->index, bcc_min_curr, bcc_max_curr, bcc_exit_curr);

	return;
}

static void oplus_ufcs_set_cool_down_curr(struct oplus_ufcs *chip, int cool_down)
{
	int target_curr;

	if (chip->chg_ctrl_by_sale_mode) {
		if (chip->curr_table_type == UFCS_CURR_CP_TABLE) {
			if (ARRAY_SIZE(ufcs_cp_cool_down_oplus_curve) >= 2)
				target_curr = ufcs_cp_cool_down_oplus_curve[SALE_MODE_COOL_DOWN_VAL];
			else
				target_curr = ufcs_cp_cool_down_oplus_curve[0];
		} else {
			if (ARRAY_SIZE(ufcs_cool_down_oplus_curve) >= 2)
				target_curr = ufcs_cool_down_oplus_curve[SALE_MODE_COOL_DOWN_VAL];
			else
				target_curr = ufcs_cool_down_oplus_curve[0];
		}
	} else {
		if (chip->curr_table_type == UFCS_CURR_CP_TABLE) {
			if (cool_down >= ARRAY_SIZE(ufcs_cp_cool_down_oplus_curve))
				cool_down = ARRAY_SIZE(ufcs_cp_cool_down_oplus_curve) - 1;
			target_curr = ufcs_cp_cool_down_oplus_curve[cool_down];
		} else {
			if (cool_down >= ARRAY_SIZE(ufcs_cool_down_oplus_curve))
				cool_down = ARRAY_SIZE(ufcs_cool_down_oplus_curve) - 1;
			target_curr = ufcs_cool_down_oplus_curve[cool_down];
		}
	}

	switch (chip->cp_work_mode) {
	case CP_WORK_MODE_4_TO_1:
		target_curr /= 4;
		break;
	case CP_WORK_MODE_3_TO_1:
		target_curr /= 3;
		break;
	case CP_WORK_MODE_2_TO_1:
		target_curr /= 2;
		break;
	case CP_WORK_MODE_BYPASS:
		break;
	default:
		chg_err("unsupported cp work mode, mode=%d\n", chip->cp_work_mode);
		return;
	}

	vote(chip->ufcs_curr_votable, SALE_MODE_VOTER, chip->chg_ctrl_by_sale_mode, target_curr, false);
	vote(chip->ufcs_curr_votable, COOL_DOWN_VOTER, !chip->chg_ctrl_by_sale_mode, target_curr, false);
}

int oplus_ufcs_level_to_current(struct oplus_mms *mms, int cool_down)
{
	struct oplus_ufcs *chip;
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

	if (chip->curr_table_type == UFCS_CURR_CP_TABLE) {
		if (cool_down >= ARRAY_SIZE(ufcs_cp_cool_down_oplus_curve))
			cool_down = ARRAY_SIZE(ufcs_cp_cool_down_oplus_curve) - 1;
		target_curr = ufcs_cp_cool_down_oplus_curve[cool_down];
	} else {
		if (cool_down >= ARRAY_SIZE(ufcs_cool_down_oplus_curve))
			cool_down = ARRAY_SIZE(ufcs_cool_down_oplus_curve) - 1;
		target_curr = ufcs_cool_down_oplus_curve[cool_down];
	}
	return target_curr / cp_ratio;
}

static void oplus_ufcs_set_batt_bal_curr(struct oplus_ufcs *chip)
{
	int target_curr;

	if (!chip->batt_bal_topic)
		return;

	target_curr = chip->batt_bal_curr_limit;
	switch (chip->cp_work_mode) {
	case CP_WORK_MODE_4_TO_1:
		target_curr /= 4;
		break;
	case CP_WORK_MODE_3_TO_1:
		target_curr /= 3;
		break;
	case CP_WORK_MODE_2_TO_1:
		target_curr /= 2;
		break;
	case CP_WORK_MODE_BYPASS:
		break;
	default:
		chg_err("unsupported cp work mode, mode=%d\n", chip->cp_work_mode);
		return;
	}

	if (target_curr)
		vote(chip->ufcs_curr_votable, BATT_BAL_VOTER, true, target_curr, false);
	else
		vote(chip->ufcs_curr_votable, BATT_BAL_VOTER, false, 0, false);
}

static void oplus_ufcs_set_plc_curr(struct oplus_ufcs *chip)
{
	int target_curr;

	if (chip->plc_status != PLC_STATUS_ENABLE)
		return;

	target_curr = chip->plc_curr;
	if (target_curr)
		vote(chip->ufcs_curr_votable, PLC_VOTER, true, target_curr, false);
	else
		vote(chip->ufcs_curr_votable, PLC_VOTER, false, 0, false);
}

static void oplus_ufcs_imp_check(struct oplus_ufcs *chip)
{
	int curr = 0;
	int pre_curr;

#define UFCS_PRELIMINARY_IMP_CHEKC_CNT		10
#define UFCS_ADSP_STATEMACHINE_PERIOD_DEFAULT	0
#define UFCS_ADSP_STATEMACHINE_PERIOD_FAST	100
#define UFCS_ADSP_WAIT_SM_SWITCH_TIMES		200

	if (!chip->imp_uint)
		return;
	if (chip->plc_status == PLC_STATUS_ENABLE) {
		chip->need_preliminary_imp_check = false;
		return;
	}

	pre_curr = get_client_vote(chip->ufcs_curr_votable, IMP_VOTER);
	curr = oplus_imp_uint_get_allow_curr(chip->imp_uint);
	if (curr < 0) {
		chg_err("can't get ufcs channel impedance current limite, exit ufcs charging\n");
		vote(chip->ufcs_disable_votable, IMP_VOTER, true, 1, false);
	} else if (curr == 0) {
		chg_err("ufcs channel impedance is too high, exit ufcs charging\n");
		vote(chip->ufcs_disable_votable, IMP_VOTER, true, 1, false);
	} else {
		vote(chip->ufcs_curr_votable, IMP_VOTER, true, curr, false);
	}

	if ((pre_curr != curr || curr <= 0) && curr < chip->curr_set_ma)
		oplus_ufcs_track_upload_err_info(chip, TRACK_UFCS_ERR_R_COOLDOWN, curr);

	if (chip->curr_set_ma == UFCS_PRELIMINARY_IMP_CHECK_CURR
	    && chip->need_preliminary_imp_check) {
		if (chip->preliminary_imp_check_cnt > UFCS_PRELIMINARY_IMP_CHEKC_CNT) {
			chip->need_preliminary_imp_check = false;
			oplus_ufcs_config_sm_period(chip, UFCS_ADSP_STATEMACHINE_PERIOD_DEFAULT);
		} else if (chip->preliminary_imp_check_cnt == 0) {
			oplus_ufcs_config_sm_period(chip, UFCS_ADSP_STATEMACHINE_PERIOD_FAST);
			msleep(UFCS_ADSP_WAIT_SM_SWITCH_TIMES);
		}
		chip->preliminary_imp_check_cnt++;
	}
}

static void oplus_ufcs_watchdog(struct oplus_ufcs *chip)
{
	int rc;
	bool work_start;

	rc = oplus_ufcs_cp_get_work_status(chip, &work_start);
	if (rc < 0) {
		chg_err("can't get cp work status, rc=%d\n", rc);
		return;
	}
	if (!work_start)
		return;
	rc = oplus_ufcs_cp_watchdog_reset(chip);
	if  (rc < 0)
		chg_err("can't reset cp watchdog, rc=%d\n", rc);
}

static void oplus_ufcs_check_current_low(struct oplus_ufcs *chip)
{
#define UFCS_CURR_LOW_LIMIT	150
#define UFCS_CURR_LOW_CNT	6

	if (UFCS_SOURCE_INFO_CURR(chip->src_info) >= UFCS_CURR_LOW_LIMIT) {
		chip->count.output_low = 0;
		return;
	}

	chip->count.output_low++;
	if (chip->count.output_low < UFCS_CURR_LOW_CNT)
		return;

	if (chip->plc_status == PLC_STATUS_ENABLE)
		vote(chip->ufcs_not_allow_votable, PLC_VOTER, true, 1, false);
	else
		vote(chip->ufcs_disable_votable, IOUT_CURR_VOTER, true, 1, false);
	chip->count.output_low = 0;
}

static void oplus_ufcs_check_ibus_curr(struct oplus_ufcs *chip)
{
	int ibus = 0;

#define UFCS_CP_IBUS_OVER_COUNTS 3
#define OPLUS_UFCS_CURRENT_LIMIT_2A 2000

	if (!chip || !chip->oplus_ufcs_adapter)
		return;

	ibus = UFCS_SOURCE_INFO_CURR(chip->src_info);

	if (ibus > chip->limits.ufcs_ibus_over_oplus) {
		chip->count.ibus_over++;
		if (chip->count.ibus_over > UFCS_CP_IBUS_OVER_COUNTS) {
			chg_err("ibus over than %d, limit curr to %d", chip->limits.ufcs_ibus_over_oplus,
				OPLUS_UFCS_CURRENT_LIMIT_2A);
			vote(chip->ufcs_curr_votable, IBUS_OVER_VOTER, true, OPLUS_UFCS_CURRENT_LIMIT_2A, false);
			chip->count.ibus_over = 0;
			oplus_ufcs_track_upload_err_info(chip, TRACK_UFCS_ERR_IBUS_LIMIT, ibus);
		}
	} else {
		chip->count.ibus_over = 0;
	}
}

static void oplus_ufcs_check_slow_chg_curr(struct oplus_ufcs *chip, struct puc_strategy_ret_data *data)
{
	int curve_current = 0, slow_chg_current = 0, bcc_current = 0, ibat_factor = 1;

#define POWER_10V_5A		50
#define POWER_10V_6P5A		65
#define SLOW_CHG_MIN_CURR	1500
#define POWER_WATT_TO_MA_10V	1000 / 10
#define POWER_WATT_TO_MA_11V	1000 / 11

	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return;
	}

	if (!chip->oplus_ufcs_adapter) {
		chip->slow_chg_batt_limit = 0;
		return;
	}

	if (!chip->slow_chg_enable) {
		vote(chip->ufcs_curr_votable, SLOW_CHG_VOTER, false, 0, false);
		chip->slow_chg_batt_limit = 0;
		return;
	}

	switch (chip->cp_work_mode) {
	case CP_WORK_MODE_4_TO_1:
		ibat_factor = 4;
		break;
	case CP_WORK_MODE_3_TO_1:
		ibat_factor = 3;
		break;
	case CP_WORK_MODE_2_TO_1:
		ibat_factor = 2;
		break;
	case CP_WORK_MODE_BYPASS:
		ibat_factor = 1;
		break;
	default:
		ibat_factor = 1;
		break;
	}

	bcc_current = get_client_vote(chip->ufcs_curr_votable, BCC_VOTER);
	curve_current = data->target_ibus;
	if (curve_current <= 0)
		curve_current = 0;
	else if (bcc_current > 0 && curve_current > bcc_current)
		curve_current = bcc_current;

	/* convert watt to ibus*/
	if (chip->slow_chg_watt == POWER_10V_5A || chip->slow_chg_watt == POWER_10V_6P5A) /* 10VxA */
		slow_chg_current = chip->slow_chg_watt * POWER_WATT_TO_MA_10V;
	else /* 11VxA */
		slow_chg_current = chip->slow_chg_watt * POWER_WATT_TO_MA_11V;

	if (curve_current > 0)
		slow_chg_current = min(slow_chg_current, curve_current * chip->slow_chg_pct / 100);

	if (slow_chg_current != 0)
		slow_chg_current = max(slow_chg_current, SLOW_CHG_MIN_CURR);

	/* convert slow ibus current to ibat, show to upper-layer */
	chip->slow_chg_batt_limit = slow_chg_current * ibat_factor;

	vote(chip->ufcs_curr_votable, SLOW_CHG_VOTER, (slow_chg_current == 0) ? false : true, slow_chg_current, false);
}

static void oplus_ufcs_get_full_vth(struct oplus_ufcs *chip, int *hw_vth, int *sw_vth)
{
        int tbat = chip->shell_temp;

	if (!chip->oplus_ufcs_adapter)
		*hw_vth = chip->limits.ufcs_full_normal_hw_vbat_third;
	else
		*hw_vth = chip->limits.ufcs_full_normal_hw_vbat;

	if (tbat < chip->limits.ufcs_cool_temp) {
		if (!chip->oplus_ufcs_adapter)
			*sw_vth = chip->limits.ufcs_full_cool_sw_vbat_third;
		else
			*sw_vth = chip->limits.ufcs_full_cool_sw_vbat;
	} else if (chip->limits.ufcs_normal_high_temp != -EINVAL && tbat > chip->limits.ufcs_normal_high_temp) {
		*sw_vth = chip->limits.ufcs_full_warm_vbat;
	} else {
		if (!chip->oplus_ufcs_adapter)
			*sw_vth = chip->limits.ufcs_full_normal_sw_vbat_third;
		else
			*sw_vth = chip->limits.ufcs_full_normal_sw_vbat;
	}
}

static int oplus_ufcs_set_fcl_curr(struct oplus_ufcs *chip)
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

	if (chip->pr_config.support_pr)
		return 0;

	if (!chip->fcl_support)
	    return rc;

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CURR, &data, true);
	if (rc < 0) {
		chg_err("can't get ibat, rc=%d\n", rc);
		vote(chip->ufcs_disable_votable, NO_DATA_VOTER, true, 1, false);
		return rc;
	}
	ifg = -data.intval;
	ibat = abs(ifg);

	rc = oplus_ufcs_cp_get_iin(chip, &ibus_ma);
	if (rc >= 0 && ibus_ma > 0)
		ibat = ibus_ma * chip->cp_ratio;

	oplus_gauge_get_vb_offset(chip->gauge_topic, &vb_offset);
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_FCL, &data, true);
	if (rc >= 0)
		vbat_fcl = data.intval;

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX, &data, true);
	if (rc >= 0)
		vbat_fg = data.intval;

	rc = oplus_ufcs_cp_get_vout(chip, &vbat_cp);
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
	oplus_ufcs_get_full_vth(chip, &hw_vth, &sw_vth);

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
			oplus_ufcs_track_set_fcl_info(chip->dev, vbat_r, ibat, chip->shell_temp);
		else
			oplus_ufcs_track_set_fcl_info(chip->dev, 0, 0, 0);
		chg_info(" [%d, %d][%d, %d, %d][%d]\n", ibat, ibus_ma, vbat_r, vbat_fcl, vbat_cp, vb_offset);

		fcl_limit = ROUND_DOWN(fcl_limit, 50);
		if (fcl_limit)
			vote(chip->ufcs_curr_votable, LIMIT_FCL_VOTER, true,
				max(fcl_limit, UFCS_START_CP_CURR_MA), false);
		else
			vote(chip->ufcs_curr_votable, LIMIT_FCL_VOTER, false, 0, false);
	}

	return 0;
}

static void oplus_ufcs_protection_check(struct oplus_ufcs *chip, struct puc_strategy_ret_data *data)
{
	int rc;

	rc = oplus_ufcs_get_src_info(chip, &chip->src_info);
	if (rc < 0) {
		chg_err("ufcs get src info error\n");
		chip->src_info = 0;
	}

	oplus_ufcs_check_timeout(chip);
	oplus_ufcs_check_full(chip, data);
	oplus_ufcs_check_ibus_curr(chip);
	oplus_ufcs_check_ibat_safety(chip);
	oplus_ufcs_check_temp(chip);
	oplus_ufcs_imp_check(chip);
	if (chip->config.ufcs_wd_separate_work) {
		chip->timer.monitor_last_run_jiffies = jiffies;
		if (!work_busy(&chip->watchdog_work.work)) {
			oplus_ufcs_watchdog(chip);
			schedule_delayed_work(&chip->watchdog_work, msecs_to_jiffies(UFCS_WD_TIME_MS));
		}
	} else {
		oplus_ufcs_watchdog(chip);
	}
}

static void oplus_ufcs_set_soc_current(struct oplus_ufcs *chip)
{
	int curr_ma = 0;

	if (chip->pr_config.support_pr)
		return;

	if (chip->config.curr_max_ma_percent_75 <= 0 || chip->config.curr_max_ma_percent_85 <= 0)
		return;

	if (chip->ui_soc > 85) {
		if (chip->last_target_curr_ma > 0 && chip->last_target_curr_ma > chip->target_curr_ma) {
			curr_ma = max(chip->target_curr_ma, chip->config.curr_max_ma_percent_85);
			vote(chip->ufcs_curr_votable, BATT_SOC_VOTER, true, curr_ma, false);
		}
	} else if (chip->ui_soc > 75) {
		if (chip->last_target_curr_ma > 0 && chip->last_target_curr_ma > chip->target_curr_ma) {
			curr_ma = max(chip->target_curr_ma, chip->config.curr_max_ma_percent_75);
			vote(chip->ufcs_curr_votable, BATT_SOC_VOTER, true, curr_ma, false);
		}
	} else {
		vote(chip->ufcs_curr_votable, BATT_SOC_VOTER, false, 0, false);
	}
	chip->last_target_curr_ma = chip->target_curr_ma;
}

static void oplus_ufcs_monitor_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_ufcs *chip =
		container_of(dwork, struct oplus_ufcs, monitor_work);
	struct puc_strategy_ret_data data;
	int rc;
	int delay = UFCS_MONITOR_TIME_MS;
	int range_switch_dealy = UFCS_TEMP_SWITCH_DELAY;
	int target_ibus;
	bool switch_to_ffc = false;

	if (!chip->ufcs_charging) {
		rc = oplus_ufcs_charge_start(chip);
		if (rc < 0) {
			if (chip->ufcs_online && !chip->prot_crash && !chip->subsys_reset_ufcs) {
				chg_info("rc=%d, goto next\n", rc);
				goto next;
			} else {
				chg_info("rc=%d, goto exit\n", rc);
				goto exit;
			}
		}
		oplus_ufcs_cp_watchdog_reset(chip);
		delay = rc;
	} else {
		rc = oplus_ufcs_get_batt_temp_curr(chip);
		if (rc < 0)
			goto exit;

		rc = oplus_chg_strategy_get_data(chip->strategy, &data);
		if (rc < 0) {
			chg_err("can't get strategy data, rc=%d", rc);
			goto exit;
		}
		if (data.exit || chip->pr_data.full) {
			chg_info("exit ufcs fast charge, start ffc\n");
			switch_to_ffc = true;
			if (chip->ufcs_fastchg_batt_temp_status == UFCS_BAT_TEMP_WARM) {
				vote(chip->ufcs_not_allow_votable, CHG_FULL_WARM_VOTER, true, 1, false);
			} else if (chip->config.ufcs_full_recheck_temp != -EINVAL &&
				   chip->shell_temp <= chip->config.ufcs_full_recheck_temp) {
				chip->ufcs_cool_full_temp_range = chip->ufcs_temp_cur_range;
				vote(chip->ufcs_not_allow_votable, CHG_FULL_COOL_VOTER, true, 1, false);
			} else {
				vote(chip->ufcs_disable_votable, CHG_FULL_VOTER, true, 1, false);
			}
			goto exit;
		}

		if (chip->pr_config.support_pr)
			ufcs_pr_check_bcc_curr(chip);
		else
			oplus_ufcs_check_bcc_curr(chip, &data);
		oplus_ufcs_set_cool_down_curr(chip, chip->cool_down);
		oplus_ufcs_set_batt_bal_curr(chip);
		oplus_ufcs_check_slow_chg_curr(chip, &data);
		oplus_ufcs_set_fcl_curr(chip);
		oplus_ufcs_set_plc_curr(chip);
		oplus_ufcs_protection_check(chip, &data);
		if (chip->pr_config.support_pr)
			oplus_ufcs_check_current_low(chip);
		if (get_client_vote(chip->ufcs_disable_votable, CHG_FULL_VOTER) > 0 ||
			get_client_vote(chip->ufcs_not_allow_votable, CHG_FULL_COOL_VOTER) > 0) {
			chg_info("exit ufcs fast charge, start ffc\n");
			switch_to_ffc = true;
			goto exit;
		}
		if (chip->ccd_strategy) {
			target_ibus = oplus_ufcs_apply_ccd(chip, data.target_ibus);
			vote(chip->ufcs_curr_votable, STEP_VOTER, true, target_ibus, false);
		} else {
			vote(chip->ufcs_curr_votable, STEP_VOTER, true, data.target_ibus, false);
		}
		oplus_ufcs_set_soc_current(chip);

		if (chip->curr_set_ma == UFCS_PRELIMINARY_IMP_CHECK_CURR
		    && chip->need_preliminary_imp_check) {
			delay = UFCS_PRELIMINARY_IMP_TIME_MS;
		}
	}

	if (chip->ufcs_not_allow || chip->ufcs_disable) {
		chg_info("ufcs charge not allow or disable, exit ufcs mode\n");
		goto exit;
	}

	if (chip->wired_online)
		schedule_delayed_work(&chip->monitor_work, msecs_to_jiffies(delay));
	return;

exit:
	if (!chip->subsys_reset_ufcs)
		oplus_ufcs_soft_exit(chip);
	if (switch_to_ffc) {
		if (is_disable_charger_vatable_available(chip))
			vote(chip->chg_disable_votable, FASTCHG_VOTER, true, 1, false);
		oplus_comm_switch_ffc(chip->comm_topic);
	}
	if (chip->config.adsp_ufcs_project)
		range_switch_dealy = UFCS_ADSP_TEMP_SWITCH_DELAY;
	if (chip->ufcs_fastchg_batt_temp_status == UFCS_BAT_TEMP_SWITCH_CURVE) {
		chg_info("ufcs need retry enter ufcs\n");
		chip->ufcs_fastchg_batt_temp_status = UFCS_BAT_TEMP_NATURAL;
		schedule_delayed_work(&chip->switch_check_work, msecs_to_jiffies(range_switch_dealy));
	}
	if (get_client_vote(chip->ufcs_disable_votable, PLC_RETRY_VOTER) > 0) {
		chg_info("ufcs PLC need retry enter ufcs\n");
		chip->ufcs_fastchg_batt_temp_status = UFCS_MONITOR_CYCLE_MS;
		schedule_delayed_work(&chip->switch_check_work, msecs_to_jiffies(range_switch_dealy));
	}
	return;
next:
	oplus_ufcs_force_exit(chip);
}

enum {
	OPLUS_UFCS_CURR_UPDATE_V1 = 50,
	OPLUS_UFCS_CURR_UPDATE_V2 = 100,
	OPLUS_UFCS_CURR_UPDATE_V3 = 200,
	OPLUS_UFCS_CURR_UPDATE_V4 = 300,
	OPLUS_UFCS_CURR_UPDATE_V5 = 400,
	OPLUS_UFCS_CURR_UPDATE_V6 = 500,
	OPLUS_UFCS_CURR_UPDATE_V7 = 1000,
	OPLUS_UFCS_CURR_UPDATE_V8 = 1500,
	OPLUS_UFCS_CURR_UPDATE_V9 = 2000,
	OPLUS_UFCS_CURR_UPDATE_V10 = 2500,
	OPLUS_UFCS_CURR_UPDATE_V11 = 3000,
};

static void oplus_ufcs_get_current_cc(struct oplus_ufcs *chip)
{
	int curr_set, update_size;
	if (chip->vol_set_mv < chip->config.target_vbus_mv && chip->curr_cc)
		curr_set = chip->curr_cc;
	else
		curr_set = chip->curr_set_ma;
	if (curr_set > (chip->target_curr_ma + OPLUS_UFCS_CURR_UPDATE_V6)) {
		if (abs(curr_set - chip->target_curr_ma) >= OPLUS_UFCS_CURR_UPDATE_V10)
			update_size = OPLUS_UFCS_CURR_UPDATE_V10;
		else if (abs(curr_set - chip->target_curr_ma) >= OPLUS_UFCS_CURR_UPDATE_V8)
			update_size = OPLUS_UFCS_CURR_UPDATE_V8;
		else if (abs(curr_set - chip->target_curr_ma) >= OPLUS_UFCS_CURR_UPDATE_V7)
			update_size = OPLUS_UFCS_CURR_UPDATE_V7;
		else
			update_size = OPLUS_UFCS_CURR_UPDATE_V6;
		curr_set -= update_size;
		if (curr_set < chip->target_curr_ma)
			curr_set = chip->target_curr_ma;
	} else if (curr_set < chip->target_curr_ma) {
		if (abs(curr_set - chip->target_curr_ma) >= OPLUS_UFCS_CURR_UPDATE_V3)
			update_size = OPLUS_UFCS_CURR_UPDATE_V3;
		else
			update_size = OPLUS_UFCS_CURR_UPDATE_V1;
		curr_set += update_size;
		if (curr_set > chip->target_curr_ma)
			curr_set = chip->target_curr_ma;
	} else {
		curr_set = chip->target_curr_ma;
	}
	chip->curr_cc = curr_set;
}


struct update_step {
	int istep_ma;
	int vstep_mv;
};

static const struct update_step g_ufcs_update_table[] = {
	{ 0, 20 },
	{ 200, 40 },
	{ 400, 80 },
	{ 800, 160 },
	{ 2000, 240 },
	{ 3000, 320 },
};

static int oplus_ufcs_get_update_vstep(struct oplus_ufcs *chip, int ibus)
{
	int i = 0, asize = 0;
	int vstep = 0;
	if (ibus < 0)
		return vstep;

	asize = sizeof(g_ufcs_update_table) / sizeof(struct update_step);
	for (i = asize - 1; i >= 0; i--) {
		if (ibus >= g_ufcs_update_table[i].istep_ma) {
			vstep = g_ufcs_update_table[i].vstep_mv;
			break;
		}
	}

	return vstep;
}

static int oplus_ufcs_get_charger_vstep(struct oplus_ufcs *chip)
{
	int i, pdo_vstep = 0;

	for (i = 0; i < chip->pdo_num; i++) {
		if (chip->target_vbus_mv <= UFCS_OUTPUT_MODE_VOL_MAX(chip->pdo[i]) &&
			chip->target_vbus_mv >= UFCS_OUTPUT_MODE_VOL_MIN(chip->pdo[i]))
			pdo_vstep = pdo_vstep > UFCS_OUTPUT_MODE_VOL_STEP(chip->pdo[i]) ?
				pdo_vstep : UFCS_OUTPUT_MODE_VOL_STEP(chip->pdo[i]);
	}

	if (pdo_vstep <= 0)
		pdo_vstep = 40;

	return pdo_vstep;
}

static void oplus_ufcs_volt_update_check(struct oplus_ufcs *chip)
{
	union mms_msg_data msg_data = { 0 };
	int update_size = 0, update_vstep = 0, pdo_vstep = 0;
	int cp_ibus = 0, vchg = 0, next_target_vbus_mv = chip->vol_set_mv;
	int ratio = 1, batt_num = 1, vbat = 0;
	int charger_volt_min = 0, vbus_over = 0;
	int rc = 0;

#define UFCS_VOLT_ACTION_START_DIFF_V1_V (20)
#define UFCS_VOLT_ASK_VOLT_MAX_V (5500)
#define UFCS_VOLT_IBUS_ADJUST_OK_THLD (50)

	pdo_vstep = oplus_ufcs_get_charger_vstep(chip);
	batt_num = oplus_gauge_get_batt_num();
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX, &msg_data, true);
	if (unlikely(rc < 0)) {
		chg_err("can't get vbat, rc=%d\n", rc);
		goto exit;
	} else {
		vbat = msg_data.intval;
	}
	vbat *= batt_num;

	switch (chip->cp_work_mode) {
	case CP_WORK_MODE_4_TO_1:
		ratio = 4;
		charger_volt_min = ((vbat * ratio) / pdo_vstep) * pdo_vstep + UFCS_VOLT_ACTION_START_DIFF_V1_V;
		break;
	case CP_WORK_MODE_3_TO_1:
		ratio = 3;
		charger_volt_min = ((vbat * ratio) / pdo_vstep) * pdo_vstep + UFCS_VOLT_ACTION_START_DIFF_V1_V;
		break;
	case CP_WORK_MODE_2_TO_1:
		ratio = 2;
		charger_volt_min = ((vbat * ratio) / pdo_vstep) * pdo_vstep + UFCS_VOLT_ACTION_START_DIFF_V1_V;
		break;
	case CP_WORK_MODE_BYPASS:
		ratio = 1;
		charger_volt_min = ((vbat * ratio) / pdo_vstep) * pdo_vstep + UFCS_VOLT_ACTION_START_DIFF_V1_V;
		break;
	default:
		chg_err("unsupported cp work mode, mode=%d\n", chip->cp_work_mode);
		goto exit;
	}
	vbus_over = UFCS_VOLT_ASK_VOLT_MAX_V * batt_num * ratio;

	rc = oplus_ufcs_get_src_info(chip, &chip->src_info);
	if (rc < 0) {
		chg_err("ufcs get src info error\n");
		goto exit;
	}

	cp_ibus = UFCS_SOURCE_INFO_CURR(chip->src_info);
	vchg = UFCS_SOURCE_INFO_VOL(chip->src_info);
	update_vstep = oplus_ufcs_get_update_vstep(chip, abs(cp_ibus - chip->target_curr_ma));

	if (cp_ibus > chip->target_curr_ma + UFCS_VOLT_IBUS_ADJUST_OK_THLD) {
		update_size = -update_vstep;
	} else if (cp_ibus < chip->target_curr_ma - UFCS_VOLT_IBUS_ADJUST_OK_THLD) {
		update_size = update_vstep;
	} else {
		update_size = 0;
		chg_info(" UFCS chargering volt okay\n");
	}

	next_target_vbus_mv += update_size;
	if (next_target_vbus_mv < charger_volt_min)
		next_target_vbus_mv = charger_volt_min;
	else if (next_target_vbus_mv > vbus_over)
		next_target_vbus_mv = vbus_over;

	chip->target_vbus_mv = next_target_vbus_mv;

	return;

exit:
	chip->target_vbus_mv = next_target_vbus_mv;
	return;
}

#include <oplus_chg_ufcs_pr.h>
static void oplus_ufcs_pr_current_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_ufcs *chip = container_of(dwork, struct oplus_ufcs, current_work);
	int rc = 0;
	int delay_ms = UFCS_PR_DELAY_MS;

	if (!chip->ufcs_charging)
		return;

	rc = oplus_ufcs_pr_update_data(chip);
	if (rc < 0)
		goto done;

	delay_ms = ufcs_pr_run(chip);
done:
	schedule_delayed_work(&chip->current_work, msecs_to_jiffies(delay_ms));
}

static void oplus_ufcs_current_work(struct work_struct *work)
{
#define UFCS_CURR_CHANGE_UPDATE_DELAY		200
#define UFCS_CURR_NO_CHANGE_UPDATE_DELAY	500
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_ufcs *chip =
		container_of(dwork, struct oplus_ufcs, current_work);
	int curr_set = 0;
	int delay_time_ms = UFCS_CURR_NO_CHANGE_UPDATE_DELAY;
	int rc = 0;

#define UFCS_THIRD_IBUS_PLC_THLD (50)

	if (!chip->ufcs_charging)
		return;

	oplus_ufcs_check_current_low(chip);
	oplus_ufcs_get_current_cc(chip);
	oplus_ufcs_volt_update_check(chip);

	if (chip->oplus_ufcs_adapter) {
		if  (chip->plc_status != PLC_STATUS_ENABLE || chip->curr_cc >= PLC_IBUS_MAX) {
			chip->target_vbus_mv = chip->config.target_vbus_mv;
			curr_set = chip->curr_cc;
			/* stay at 4A temporarily to preliminary-imp check */
			if (chip->need_preliminary_imp_check
				&& chip->curr_set_ma == UFCS_PRELIMINARY_IMP_CHECK_CURR) {
				if (get_client_vote(chip->ufcs_curr_votable, LIMIT_FCL_VOTER) > 0)
					curr_set = chip->target_curr_ma;
				else
					curr_set = chip->curr_set_ma;
			}
			delay_time_ms = UFCS_CURR_CHANGE_UPDATE_DELAY;
		}
	} else {
		curr_set = min(chip->target_curr_ma, get_client_vote(chip->ufcs_curr_votable, BASE_MAX_VOTER));
	}

	if ((chip->plc_status == PLC_STATUS_ENABLE) && curr_set < PLC_IBUS_MAX) {
		if (chip->vol_set_mv == chip->config.target_vbus_mv) {
			chip->target_vbus_mv = UFCS_SOURCE_INFO_VOL(chip->src_info) + UFCS_THIRD_IBUS_PLC_THLD;
		}
		oplus_ufcs_cp_set_ucp_disable(chip, true);
		curr_set = PLC_IBUS_MAX;
	} else if (curr_set <= PLC_IBUS_DEFAULT) {
		curr_set = PLC_IBUS_DEFAULT;
	}

	chg_info("[%d, %d][%d, %d, %d, %d]\n", curr_set, chip->target_vbus_mv, chip->curr_cc, chip->target_curr_ma,
		chip->curr_set_ma, chip->vol_set_mv);

	if ((chip->target_vbus_mv != chip->vol_set_mv) || (curr_set != chip->curr_set_ma)) {
		rc = oplus_ufcs_cp_set_iin(chip, curr_set);
		if (rc < 0) {
			chg_err("set cp input current error, rc=%d\n", rc);
		} else {
			if (oplus_wired_get_bcc_curr_done_status(chip->wired_topic) == BCC_CURR_DONE_REQUEST)
				oplus_wired_check_bcc_curr_done(chip->wired_topic);
			rc = oplus_ufcs_pdo_set(chip, chip->target_vbus_mv, curr_set);
			if (rc < 0)
				chg_err("pdo set error, rc=%d\n", rc);
		}
	}

	schedule_delayed_work(&chip->current_work, msecs_to_jiffies(delay_time_ms));
}

static void oplus_ufcs_wired_online_work(struct work_struct *work)
{
	struct oplus_ufcs *chip =
		container_of(work, struct oplus_ufcs, wired_online_work);
	union mms_msg_data data = { 0 };

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_ONLINE, &data,
				true);
	WRITE_ONCE(chip->wired_online, !!data.intval);

	if (!chip->wired_online) {
		chip->reset_adapter = false;
		if (chip->config.ufcs_need_reset_adapter)
			complete(&chip->reset_abnormal_ack);

		oplus_ufcs_force_exit(chip);
		cancel_delayed_work_sync(&chip->switch_check_work);
		cancel_delayed_work_sync(&chip->monitor_work);
		cancel_delayed_work_sync(&chip->current_work);
		cancel_delayed_work_sync(&chip->switch_end_recheck_work);
		if (chip->config.ufcs_wd_separate_work)
			cancel_delayed_work_sync(&chip->watchdog_work);
		chip->slow_chg_batt_limit = 0;
		if (is_wired_icl_votable_available(chip))
			vote(chip->wired_icl_votable, BTB_TEMP_OVER_VOTER,
			     false, 0, true);
		oplus_ufcs_reset_temp_range(chip);
		oplus_ufcs_set_ufcs_vid(chip, 0);
		vote(chip->ufcs_curr_votable, BAD_SUB_BTB_VOTER, false, 0, false);
		vote(chip->ufcs_not_allow_votable, CHG_FULL_COOL_VOTER, false, 0, false);
		oplus_ufcs_vfa_reset(chip);
	} else {
		chip->retention_state_ready = false;
		if (READ_ONCE(chip->disconnect_change) && !chip->ufcs_online &&
		    chip->retention_state && chip->cpa_current_type == CHG_PROTOCOL_UFCS) {
			schedule_delayed_work(&chip->switch_check_work,
				msecs_to_jiffies(WAIT_BC1P2_GET_TYPE));
			WRITE_ONCE(chip->disconnect_change, false);
		}
	}
}

static void oplus_ufcs_force_exit_work(struct work_struct *work)
{
	struct oplus_ufcs *chip =
		container_of(work, struct oplus_ufcs, force_exit_work);

	if (chip->ufcs_online) {
		oplus_ufcs_force_exit(chip);
		cancel_delayed_work_sync(&chip->switch_check_work);
		cancel_delayed_work_sync(&chip->monitor_work);
		cancel_delayed_work_sync(&chip->current_work);
		if (chip->config.ufcs_wd_separate_work)
			cancel_delayed_work_sync(&chip->watchdog_work);
	}
}

static void oplus_ufcs_soft_exit_work(struct work_struct *work)
{
	struct oplus_ufcs *chip =
		container_of(work, struct oplus_ufcs, soft_exit_work);

	if (chip->ufcs_online) {
		oplus_ufcs_soft_exit(chip);
		cancel_delayed_work_sync(&chip->switch_check_work);
		cancel_delayed_work_sync(&chip->monitor_work);
		cancel_delayed_work_sync(&chip->current_work);
		if (chip->config.ufcs_wd_separate_work)
			cancel_delayed_work_sync(&chip->watchdog_work);
	}
}

static void oplus_ufcs_offset_current_temp_range(struct oplus_ufcs *chip, int up_thr_offset, int down_thr_offset)
{
	int vbat_temp_cur;

	vbat_temp_cur = chip->shell_temp;
	oplus_ufcs_reset_temp_range(chip);
	if (vbat_temp_cur < chip->limits.ufcs_little_cold_temp) { /*<5C*/
		chip->limits.ufcs_little_cold_temp += up_thr_offset;
	} else if (vbat_temp_cur < chip->limits.ufcs_cool_temp) { /*<12C*/
		chip->limits.ufcs_cool_temp += up_thr_offset;
		chip->limits.ufcs_little_cold_temp += down_thr_offset;
	} else if (vbat_temp_cur < chip->limits.ufcs_little_cool_temp) { /*<20C*/
		chip->limits.ufcs_little_cool_temp += up_thr_offset;
		chip->limits.ufcs_cool_temp += down_thr_offset;
	} else if (chip->limits.ufcs_little_cool_high_temp != -EINVAL &&
		vbat_temp_cur < chip->limits.ufcs_little_cool_high_temp) { /*<21C*/
		chip->limits.ufcs_little_cool_high_temp += up_thr_offset;
		chip->limits.ufcs_little_cool_temp += down_thr_offset;
	} else if (chip->limits.ufcs_normal_low_pre_temp != -EINVAL &&
		vbat_temp_cur < chip->limits.ufcs_normal_low_pre_temp) {
		chip->limits.ufcs_normal_low_pre_temp += up_thr_offset;
		if (chip->limits.ufcs_little_cool_high_temp != -EINVAL)
			chip->limits.ufcs_little_cool_high_temp += down_thr_offset;
		else
			chip->limits.ufcs_little_cool_temp += down_thr_offset;
	} else if (vbat_temp_cur < chip->limits.ufcs_normal_low_temp) { /*<35C*/
		chip->limits.ufcs_normal_low_temp += up_thr_offset;
		if (chip->limits.ufcs_normal_low_pre_temp != -EINVAL)
			chip->limits.ufcs_normal_low_pre_temp += down_thr_offset;
		else if (chip->limits.ufcs_little_cool_high_temp != -EINVAL)
			chip->limits.ufcs_little_cool_high_temp += down_thr_offset;
		else
			chip->limits.ufcs_little_cool_temp += down_thr_offset;
	} else if (vbat_temp_cur < chip->limits.ufcs_normal_high_temp) { /*<43C*/
		chip->limits.ufcs_normal_high_temp += up_thr_offset;
		chip->limits.ufcs_normal_low_temp += down_thr_offset;
	} else {							/*>=43*/
		chip->limits.ufcs_normal_high_temp += down_thr_offset;
	}
}

static void oplus_ufcs_allow_recover_check(struct oplus_ufcs *chip)
{
	int rc;
	union mms_msg_data data = { 0 };
	int vbat_mv = 0;

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX, &data, false);
	if (unlikely(rc < 0)) {
		chg_err("can't get vbat, rc=%d\n", rc);
		vbat_mv = 3800;
	} else {
		vbat_mv = data.intval;
	}

	rc = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_SHELL_TEMP, &data, true);
	if (rc < 0) {
		chg_err("can't get shell temp data, rc=%d", rc);
	} else {
		chip->shell_temp = data.intval;
	}

	if (is_client_vote_enabled(chip->ufcs_not_allow_votable, BATT_TEMP_VOTER)) {
		if (chip->shell_temp >= chip->limits.ufcs_low_temp &&
		    chip->shell_temp <= (chip->limits.ufcs_normal_high_temp - UFCS_TEMP_WARM_RANGE_THD)) {
			chg_info("allow ufcs charging, shell_temp=%d\n", chip->shell_temp);
			if (chip->config.ufcs_need_reset_adapter)
				chip->reset_adapter = true;
			atomic_set(&chip->temp_recover_debounce_thd, 1);
			vote(chip->ufcs_not_allow_votable, BATT_TEMP_VOTER, false, 0, false);
		} else if (chip->shell_temp <= chip->limits.ufcs_high_temp &&
		    chip->shell_temp > (chip->limits.ufcs_normal_high_temp - UFCS_TEMP_WARM_RANGE_THD)) {
			if (vbat_mv <= chip->limits.ufcs_warm_allow_vol &&
			    chip->ui_soc <= chip->limits.ufcs_warm_allow_soc) {
				chg_info("allow ufcs warm_temp charging, shell_temp=%d, vbat_mv=%d, ui_soc=%d\n",
					    chip->shell_temp, vbat_mv, chip->ui_soc);
				if (chip->config.ufcs_need_reset_adapter)
					chip->reset_adapter = true;
				atomic_set(&chip->temp_recover_debounce_thd, 1);
				vote(chip->ufcs_not_allow_votable, BATT_TEMP_VOTER, false, 0, false);
			}
		}
	}
	if (is_client_vote_enabled(chip->ufcs_not_allow_votable, BATT_SOC_VOTER)) {
		if (chip->ui_soc >= chip->limits.ufcs_low_soc &&
		    chip->ui_soc <= chip->high_soc) {
			chg_info("allow ufcs charging, ui_soc=%d\n", chip->ui_soc);
			if (chip->config.ufcs_need_reset_adapter)
				chip->reset_adapter = true;
			vote(chip->ufcs_not_allow_votable, BATT_SOC_VOTER, false, 0, false);
		}
	}

	if (is_client_vote_enabled(chip->ufcs_not_allow_votable, CHG_FULL_WARM_VOTER) > 0) {
		if (chip->shell_temp >= chip->limits.ufcs_low_temp &&
		    chip->shell_temp <= (chip->limits.ufcs_normal_high_temp - UFCS_TEMP_WARM_RANGE_THD)) {
			if (chip->config.ufcs_need_reset_adapter)
				chip->reset_adapter = true;
			vote(chip->ufcs_not_allow_votable, CHG_FULL_WARM_VOTER, false, 0, false);
		}
	}

	if (is_client_vote_enabled(chip->ufcs_not_allow_votable, CHG_FULL_COOL_VOTER)) {
		oplus_ufcs_temp_cur_range_init(chip);
		if (chip->ufcs_temp_cur_range > chip->ufcs_cool_full_temp_range) {
			chg_info("allow ufcs charging, cur_temp_range=%d, full_temp_range=%d, shell_temp=%d\n",
				chip->ufcs_temp_cur_range, chip->ufcs_cool_full_temp_range, chip->shell_temp);
			oplus_ufcs_offset_current_temp_range(chip, 0, -UFCS_TEMP_LOW_RANGE_THD);
			if (chip->config.ufcs_need_reset_adapter)
				chip->reset_adapter = true;
			chip->ufcs_cool_full_temp_range = UFCS_TEMP_RANGE_WARM;
			vote(chip->ufcs_not_allow_votable, CHG_FULL_COOL_VOTER, false, 0, false);
		}
	}

	if (is_client_vote_enabled(chip->ufcs_not_allow_votable, VFA_VOTER))
		oplus_ufcs_vfa_allow(chip, false);
}

static void oplus_ufcs_gauge_update_work(struct work_struct *work)
{
	struct oplus_ufcs *chip =
		container_of(work, struct oplus_ufcs, gauge_update_work);

	oplus_ufcs_allow_recover_check(chip);
}

static void oplus_ufcs_err_flag_push_work(struct work_struct *work)
{
	struct oplus_ufcs *chip =
		container_of(work, struct oplus_ufcs, err_flag_push_work);
	struct mms_msg *msg;
	unsigned int ignored_flag;
	int rc;
	char *buf;
	int i;
	size_t index;

	ignored_flag = BIT(UFCS_HW_ERR_HARD_RESET) |
		BIT(UFCS_RECV_ERR_SENT_CMP) |
		BIT(UFCS_RECV_ERR_DATA_READY) |
		BIT(UFCS_COMM_ERR_BAUD_RATE_CHANGE);
	if ((chip->err_flag & (~ignored_flag)) == 0)
		return;

	if (!is_err_topic_available(chip))
		return;

	buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (buf == NULL)
		return;

	index = scnprintf(buf, PAGE_SIZE, "$$reason@@0x%x$$dev_info@@0x%llx"
		"$$cable_info@@0x%llx$$emark_info@@0x%llx",
		chip->err_flag, chip->dev_info, chip->cable_info, chip->emark_info);
	if (chip->pdo_num > 0)
		index += scnprintf(buf + index, PAGE_SIZE, "$$pdo_info@@");
	for (i = 0; i < chip->pdo_num; i++) {
		if (i == chip->pdo_num - 1)
			index += scnprintf(buf + index, PAGE_SIZE, "0x%llx", chip->pdo[i]);
		else
			index += scnprintf(buf + index, PAGE_SIZE, "0x%llx,", chip->pdo[i]);
	}

	if (chip->pie_num > 0)
		index += scnprintf(buf + index, PAGE_SIZE, "$$pie_info@@");
	for (i = 0; i < chip->pie_num; i++) {
		if (i == chip->pie_num - 1)
			index += scnprintf(buf + index, PAGE_SIZE, "0x%llx", chip->pie[i]);
		else
			index += scnprintf(buf + index, PAGE_SIZE, "0x%llx,", chip->pie[i]);
	}
	index += scnprintf(buf + index, PAGE_SIZE, "$$power_max@@%d",
		oplus_cpa_protocol_get_power(chip->cpa_topic, CHG_PROTOCOL_UFCS));

	msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_UFCS, buf);
	kfree(buf);
	rc = oplus_mms_publish_msg_sync(chip->err_topic, msg);
	if (rc < 0) {
		chg_err("publish ufcs error msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_ufcs_fifo_overflow_push_work(struct work_struct *work)
{
	struct oplus_ufcs *chip =
		container_of(work, struct oplus_ufcs, fifo_overflow_push_work);
	struct mms_msg *msg;
	int rc;
	char *buf;
	int i;
	size_t index;

	if (!is_err_topic_available(chip))
		return;

	buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (buf == NULL)
		return;

	index = scnprintf(buf, PAGE_SIZE, "$$reason@@fifo_overflow$$err_flag@@0x%x"
		"$$dev_info@@0x%llx$$cable_info@@0x%llx$$emark_info@@0x%llx",
		chip->err_flag, chip->dev_info, chip->cable_info, chip->emark_info);
	if (chip->pdo_num > 0)
		index += scnprintf(buf + index, PAGE_SIZE, "$$pdo_info@@");
	for (i = 0; i < chip->pdo_num; i++) {
		if (i == chip->pdo_num - 1)
			index += scnprintf(buf + index, PAGE_SIZE, "0x%llx", chip->pdo[i]);
		else
			index += scnprintf(buf + index, PAGE_SIZE, "0x%llx,", chip->pdo[i]);
	}

	if (chip->pie_num > 0)
		index += scnprintf(buf + index, PAGE_SIZE, "$$pie_info@@");
	for (i = 0; i < chip->pie_num; i++) {
		if (i == chip->pie_num - 1)
			index += scnprintf(buf + index, PAGE_SIZE, "0x%llx", chip->pie[i]);
		else
			index += scnprintf(buf + index, PAGE_SIZE, "0x%llx,", chip->pie[i]);
	}
	index += scnprintf(buf + index, PAGE_SIZE, "$$power_max@@%d",
		oplus_cpa_protocol_get_power(chip->cpa_topic, CHG_PROTOCOL_UFCS));

	msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_UFCS, buf);
	kfree(buf);
	rc = oplus_mms_publish_msg_sync(chip->err_topic, msg);
	if (rc < 0) {
		chg_err("publish ufcs error msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_ufcs_plat_notify_exit_work(struct work_struct *work)
{
	struct oplus_ufcs *chip =
		container_of(work, struct oplus_ufcs, plat_notify_exit_work);

	oplus_ufcs_cp_set_work_start(chip, false);
}

static void oplus_ufcs_restart_timeout_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_ufcs *chip =
		container_of(dwork, struct oplus_ufcs, ufcs_restart_timeout_work);

	if (chip->prot_crash) {
		chip->prot_crash = false;
		schedule_work(&chip->ufcs_restart_work);
	}
}

static void oplus_fgreset_ufcs_retry_work(struct work_struct *work)
{
	struct oplus_ufcs *chip =
		container_of(work, struct oplus_ufcs, fgreset_ufcs_retry_work);

	if (!is_client_vote_enabled(chip->ufcs_not_allow_votable, BATT_TEMP_VOTER))
		vote(chip->ufcs_not_allow_votable, BATT_TEMP_VOTER, true, 1, false);

	if (is_client_vote_enabled(chip->ufcs_not_allow_votable, BATT_FG_I2CREST_VOTER))
		vote(chip->ufcs_not_allow_votable, BATT_FG_I2CREST_VOTER, false, 1, false);
	chg_err("retry ufcs \n");
}

static void oplus_ufcs_restart_work(struct work_struct *work)
{
	struct oplus_ufcs *chip =
		container_of(work, struct oplus_ufcs, ufcs_restart_work);
	union mms_msg_data data = { 0 };

	/* send auth_data to adsp after restart */
	if (chip->config.adsp_ufcs_project)
		oplus_ufcs_verify_adapter(chip, 1, chip->auth_data, UFCS_VERIFY_AUTH_DATA_SIZE);

	oplus_mms_get_item_data(chip->cpa_topic, CPA_ITEM_ALLOW, &data, true);
	if (data.intval == CHG_PROTOCOL_UFCS)
		schedule_delayed_work(&chip->switch_check_work, msecs_to_jiffies(UFCS_MONITOR_CYCLE_MS));
}

static void oplus_ufcs_subsys_reset_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_ufcs *chip =
		container_of(dwork, struct oplus_ufcs, ufcs_subsys_reset_work);
	union mms_msg_data data = { 0 };

	chip->subsys_reset_ufcs = false;
	oplus_mms_get_item_data(chip->cpa_topic, CPA_ITEM_ALLOW, &data, true);
	if (data.intval == CHG_PROTOCOL_UFCS)
		schedule_delayed_work(&chip->switch_check_work, msecs_to_jiffies(UFCS_MONITOR_CYCLE_MS));
}

static void oplus_ufcs_watchdog_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_ufcs *chip =
		container_of(dwork, struct oplus_ufcs, watchdog_work);
	unsigned long timeout_jiffies = msecs_to_jiffies(5000);
	unsigned long current_jiffies = jiffies;

	if (!chip->ufcs_charging || !chip->wired_online || !chip->config.ufcs_wd_separate_work)
		return;

	/* Check if monitor_work hasn't run for more than 5 seconds */
	if (chip->timer.monitor_last_run_jiffies != 0) {
		if (time_after(current_jiffies, chip->timer.monitor_last_run_jiffies + timeout_jiffies)) {
			chg_err("monitor_work hasn't run for more than 5 seconds, stop watchdog_work\n");
			return;
		}
	}

	oplus_ufcs_watchdog(chip);

	/* Schedule next run after 500ms */
	if (chip->wired_online)
		schedule_delayed_work(&chip->watchdog_work, msecs_to_jiffies(UFCS_WD_TIME_MS));
}

static void oplus_ufcs_eis_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_ufcs *chip =
		container_of(dwork, struct oplus_ufcs, ufcs_eis_work);
	int eis_status = EIS_STATUS_DISABLE;
	union mms_msg_data msg_data = { 0 };
	static unsigned int get_src_info_timer = 0;
	int rc;
	int vchg;

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_EIS_STATUS, &msg_data, false);
	eis_status = msg_data.intval;
	if (eis_status == EIS_STATUS_HIGH_CURRENT) {
		if (chip->eis_status == EIS_STATUS_PREPARE) {
			chg_err("<EIS>request adapter output 9V2A\n");
			/*Reset this variable to prevent the situation where the requested voltage */
			/*and current are the same and no request is sent.*/
			chip->curr_set_ma = 0;
			rc = oplus_ufcs_pdo_set(chip, UFCS_EIS_VOL_MV, UFCS_EIS_CURR_MA);
			if (rc < 0) {
				chg_err("pdo set error, rc=%d\n", rc);
				goto err;
			}
			chip->eis_status = EIS_STATUS_HIGH_CURRENT;
			get_src_info_timer = UFCS_EIS_COMMU_DELAY_CNT;
		}
	} else if (eis_status == EIS_STATUS_LOW_CURRENT) {
		if (chip->eis_status == EIS_STATUS_HIGH_CURRENT) {
			chg_err("<EIS>disable charger output\n");
			if (is_disable_charger_vatable_available(chip))
				vote(chip->chg_disable_votable, UFCS_VOTER, true, 1, false);
			chip->eis_status = EIS_STATUS_LOW_CURRENT;
		}
	} else if (eis_status == EIS_STATUS_DISABLE) {
		chg_err("<EIS>exit eis mode\n");
		if (is_wired_suspend_votable_available(chip))
			vote(chip->wired_suspend_votable, UFCS_VOTER, true, 1, false);
		if (is_disable_charger_vatable_available(chip))
			vote(chip->chg_disable_votable, UFCS_VOTER, false, 0, false);

		schedule_delayed_work(&chip->monitor_work, 0);
		return;
	}

	if (get_src_info_timer > 0) {
		get_src_info_timer--;
	} else {
		rc = oplus_ufcs_get_src_info(chip, &chip->src_info);
		if (rc < 0) {
			chg_err("ufcs get src info error\n");
			goto err;
		}
		vchg = UFCS_SOURCE_INFO_VOL(chip->src_info);
		chg_err("<EIS>eis_status[%d], cp_vin[%d]\n", eis_status, vchg);
		get_src_info_timer = UFCS_EIS_COMMU_DELAY_CNT;
	}

	schedule_delayed_work(&chip->ufcs_eis_work, msecs_to_jiffies(UFCS_EIS_MONITOR_TIME_MS));
	return;

err:
	chg_err("error, ufcs exit\n");
	oplus_ufcs_force_exit(chip);
}

static void oplus_ufcs_send_authdata_to_adsp_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_ufcs *chip =
		container_of(dwork, struct oplus_ufcs, send_authdata_to_adsp_work);
	int ret = 0;

	if (chip->config.adsp_ufcs_project && !chip->ufcs_dispatch_key_ok) {
		ret = oplus_ufcs_verify_adapter(chip, 1, chip->auth_data, UFCS_VERIFY_AUTH_DATA_SIZE);
		if (ret < 0) {
			chip->ufcs_dispatch_key_ok = false;
			chg_err("ufcs send auth data to adsp failed ret=%d.\n", ret);
		} else {
			chip->ufcs_dispatch_key_ok = true;
			chg_info("ufcs send auth data to adsp success.\n");
		}
	}
}

static void oplus_ufcs_flash_mode_handle(struct oplus_ufcs *chip)
{
	union mms_msg_data data = { 0 };
	int rc;

	if (!chip || !chip->comm_topic) {
		chg_err("invalid chip or comm_topic\n");
		return;
	}

	rc = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_FLASH_MODE,
				     &data, false);
	if (rc < 0) {
		chg_err("can't get flashmode status, rc=%d", rc);
	} else {
		if (chip->config.ufcs_need_reset_adapter && chip->wired_online &&
		    !!data.intval == 0 && chip->last_chg_flashmode == 1) {
			chip->reset_adapter = true;
			chg_info("get flashmode status, %d %d %d %d", chip->config.ufcs_need_reset_adapter,
				chip->wired_online, !!data.intval, chip->last_chg_flashmode);
		}
		vote(chip->ufcs_not_allow_votable, FLASH_MODE_VOTER, !!data.intval, data.intval, false);
		chip->last_chg_flashmode = !!data.intval;
	}
}

static void oplus_ufcs_comm_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_ufcs *chip = subs->priv_data;
	union mms_msg_data data = { 0 };
	int rc;
	static int last_charge_disable = 0;
	static int last_charge_suspend = 0;

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
			chip->batt_temp_region = !!data.intval;
			break;
		case COMM_ITEM_CHARGING_DISABLE:
			rc = oplus_mms_get_item_data(chip->comm_topic, id,
						     &data, false);
			if (rc < 0) {
				chg_err("can't get charging disable status, rc=%d", rc);
			} else {
				if (chip->config.ufcs_need_reset_adapter && chip->wired_online &&
				    !!data.intval == 0 && last_charge_disable == 1) {
					chip->reset_adapter = true;
					chg_err("get charge disable status, %d %d %d %d", chip->config.ufcs_need_reset_adapter,
						chip->wired_online, !!data.intval, last_charge_disable);
				}
				vote(chip->ufcs_not_allow_votable, MMI_CHG_VOTER, !!data.intval, data.intval, false);
				last_charge_disable = !!data.intval;
			}
			break;
		case COMM_ITEM_CHARGE_SUSPEND:
			rc = oplus_mms_get_item_data(chip->comm_topic, id,
						     &data, false);
			if (rc < 0) {
				chg_err("can't get charge suspend status, rc=%d", rc);
			} else {
				if (chip->config.ufcs_need_reset_adapter && chip->wired_online &&
				    !!data.intval == 0 && last_charge_suspend == 1) {
					chip->reset_adapter = true;
					chg_err("get charge suspend status, %d %d %d %d", chip->config.ufcs_need_reset_adapter,
						chip->wired_online, !!data.intval, last_charge_suspend);
				}
				vote(chip->ufcs_not_allow_votable, USER_VOTER, !!data.intval, data.intval, false);
				last_charge_suspend = !!data.intval;
			}
			break;
		case COMM_ITEM_FLASH_MODE:
			oplus_ufcs_flash_mode_handle(chip);
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
					vote(chip->ufcs_boot_votable, SHELL_TEMP_VOTER, false, 0, false);
				}
			}
			break;
		case COMM_ITEM_SLOW_CHG:
			rc = oplus_mms_get_item_data(chip->comm_topic, id, &data, false);
			if (rc < 0) {
				chg_err("can't get slow chg data, rc=%d", rc);
			} else {
				chip->slow_chg_pct = SLOW_CHG_TO_PCT(data.intval);
				chip->slow_chg_watt = SLOW_CHG_TO_WATT(data.intval);
				chip->slow_chg_enable = !!SLOW_CHG_TO_ENABLE(data.intval);
			}
			break;
		case COMM_ITEM_SALE_MODE:
			rc = oplus_mms_get_item_data(chip->comm_topic, id,
						     &data, false);
			if (rc < 0)
				chg_err("can't get sale mode data, rc=%d", rc);
			chip->chg_ctrl_by_sale_mode = (bool)data.intval;
			break;
		case COMM_ITEM_LED_ON:
			rc = oplus_mms_get_item_data(chip->comm_topic, id, &data, false);
			if (rc < 0)
				chg_err("can't get led on data, rc=%d", rc);
			chip->led_on = !!data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_ufcs_subscribe_comm_topic(struct oplus_mms *topic,
					     void *prv_data)
{
	struct oplus_ufcs *chip = prv_data;
	union mms_msg_data data = { 0 };
	int rc;

	chip->comm_topic = topic;
	chip->comm_subs =
		oplus_mms_subscribe(topic, chip,
				    oplus_ufcs_comm_subs_callback, "ufcs");
	if (IS_ERR_OR_NULL(chip->comm_subs)) {
		chg_err("subscribe comm topic error, rc=%ld\n",
			PTR_ERR(chip->comm_subs));
		return;
	}

	oplus_mms_get_item_data(topic, COMM_ITEM_UI_SOC, &data, true);
	chip->ui_soc = data.intval;
	oplus_mms_get_item_data(topic, COMM_ITEM_TEMP_REGION, &data, true);
	chip->batt_temp_region = !!data.intval;
	rc = oplus_mms_get_item_data(topic, COMM_ITEM_CHARGING_DISABLE, &data, true);
	if (rc < 0)
		chg_err("can't get charging disable status, rc=%d", rc);
	else
		vote(chip->ufcs_not_allow_votable, MMI_CHG_VOTER, !!data.intval, data.intval, false);
	rc = oplus_mms_get_item_data(topic, COMM_ITEM_CHARGE_SUSPEND, &data, true);
	if (rc < 0)
		chg_err("can't get charge suspend status, rc=%d", rc);
	else
		vote(chip->ufcs_not_allow_votable, USER_VOTER, !!data.intval, data.intval, false);
	rc = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_COOL_DOWN, &data, true);
	if (rc < 0)
		chg_err("can't get cool down data, rc=%d", rc);
	else
		chip->cool_down = data.intval;
	rc = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_SHELL_TEMP, &data, true);
	if (rc < 0) {
		chg_err("can't get shell temp data, rc=%d", rc);
	} else {
		chip->shell_temp = data.intval;
		if (unlikely(!chip->shell_temp_ready && chip->shell_temp != GAUGE_INVALID_TEMP)) {
			chip->shell_temp_ready = true;
			vote(chip->ufcs_boot_votable, SHELL_TEMP_VOTER, false, 0, false);
		}
	}
	rc = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_SLOW_CHG, &data, true);
	if (rc < 0) {
		chg_err("can't get slow chg data, rc=%d", rc);
	} else {
		chip->slow_chg_pct = SLOW_CHG_TO_PCT(data.intval);
		chip->slow_chg_watt = SLOW_CHG_TO_WATT(data.intval);
		chip->slow_chg_enable = !!SLOW_CHG_TO_ENABLE(data.intval);
	}
	rc = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_SALE_MODE, &data, true);
	if (rc < 0)
		chg_err("can't get sale mode data, rc=%d", rc);
	else
		chip->chg_ctrl_by_sale_mode = (bool)data.intval;

	rc = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_LED_ON, &data, true);
	if (rc < 0)
		chg_err("can't get sale mode data, rc=%d", rc);
	else
		chip->led_on = !!data.intval;

	vote(chip->ufcs_boot_votable, COMM_TOPIC_VOTER, false, 0, false);

	chip->limits.ufcs_removed_bat_decidegc =
		oplus_comm_get_removed_bat_decidegc(chip->comm_topic);
	chg_info("chip->limits.ufcs_removed_bat_decidegc = %d", chip->limits.ufcs_removed_bat_decidegc);
}

static void oplus_ufcs_wired_subs_callback(struct mms_subscribe *subs,
					   enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_ufcs *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_TIMER:
		break;
	case MSG_TYPE_ITEM:
		switch (id) {
		case WIRED_ITEM_ONLINE:
			schedule_work(&chip->wired_online_work);
			break;
		case WIRED_ITEM_CC_DETECT:
			oplus_mms_get_item_data(chip->wired_topic, id, &data,
						false);
			if (data.intval == CC_DETECT_NOTPLUG) {
				chip->adapter_verify_fail_flag = false;
				chip->retention_exit_ufcs_flag = EXIT_UFCS_FALSE;
			}
			break;
		case WIRED_ITEM_PRESENT:
			oplus_mms_get_item_data(chip->wired_topic, id, &data, false);
			chip->irq_plugin = !!data.intval;
			break;
		case WIRED_ITEM_USB_STATUS:
			oplus_mms_get_item_data(chip->wired_topic, id, &data, true);
			chip->usb_status = data.intval;
			if ((chip->usb_status & USB_TEMP_HIGH) == USB_TEMP_HIGH) {
				chg_err("!!!USB TEMP HIGH disable cp/mos and exit ufcs\n");
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

static void oplus_ufcs_subscribe_wired_topic(struct oplus_mms *topic,
					     void *prv_data)
{
	struct oplus_ufcs *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->wired_topic = topic;
	chip->wired_subs =
		oplus_mms_subscribe(chip->wired_topic, chip,
				    oplus_ufcs_wired_subs_callback, "ufcs");
	if (IS_ERR_OR_NULL(chip->wired_subs)) {
		chg_err("subscribe wired topic error, rc=%ld\n",
			PTR_ERR(chip->wired_subs));
		return;
	}

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_ONLINE, &data,
				true);
	chip->wired_online = !!data.intval;

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_USB_STATUS, &data,
				true);
	chip->usb_status = data.intval;

	vote(chip->ufcs_boot_votable, WIRED_TOPIC_VOTER, false, 0, false);
}

static void oplus_ufcs_cpa_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_ufcs *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_TIMER:
		break;
	case MSG_TYPE_ITEM:
		switch (id) {
		case CPA_ITEM_ALLOW:
			oplus_mms_get_item_data(chip->cpa_topic, id, &data,
						false);
			if (chip->cpa_current_type == CHG_PROTOCOL_UFCS && data.intval != CHG_PROTOCOL_UFCS)
				oplus_ufcs_vfa_reset(chip);
			chip->cpa_current_type = data.intval;
			/* Add work delay for BC1.2 check, for the Protocol-retention 1.0 may
			   cause switching to the UFCS error when BC1.2 is not ready. */
			/* Todo: revert this submission when Protocol-retention 2.0 has been
			   imported */
			if (chip->cpa_current_type == CHG_PROTOCOL_UFCS) {
				oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_REAL_CHG_TYPE,
							&data, false);
				if (data.intval != OPLUS_CHG_USB_TYPE_UNKNOWN)
					schedule_delayed_work(&chip->switch_check_work, 0);
				else
					schedule_delayed_work(&chip->switch_check_work,
						msecs_to_jiffies(WAIT_BC1P2_GET_TYPE));
			}
			break;
		case CPA_ITEM_TIMEOUT:
			oplus_mms_get_item_data(chip->cpa_topic, id, &data,
						false);
			if (data.intval == CHG_PROTOCOL_UFCS) {
				oplus_ufcs_switch_to_normal(chip);
				oplus_ufcs_cpa_switch_end(chip);
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

static void oplus_ufcs_subscribe_cpa_topic(struct oplus_mms *topic,
					   void *prv_data)
{
	struct oplus_ufcs *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->cpa_topic = topic;
	chip->cpa_subs =
		oplus_mms_subscribe(chip->cpa_topic, chip,
				    oplus_ufcs_cpa_subs_callback, "ufcs");
	if (IS_ERR_OR_NULL(chip->cpa_subs)) {
		chg_err("subscribe cpa topic error, rc=%ld\n",
			PTR_ERR(chip->cpa_subs));
		return;
	}

	oplus_mms_get_item_data(chip->cpa_topic, CPA_ITEM_ALLOW, &data, true);
	if (data.intval == CHG_PROTOCOL_UFCS)
		schedule_delayed_work(&chip->switch_check_work, msecs_to_jiffies(UFCS_MONITOR_CYCLE_MS));

	if (chip->wait_auth_data_done)
		vote(chip->ufcs_boot_votable, AUTH_VOTER, false, 0, false);
	vote(chip->ufcs_boot_votable, CPA_TOPIC_VOTER, false, 0, false);
}

static void oplus_ufcs_batt_bal_subs_callback(struct mms_subscribe *subs,
					      enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_ufcs *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case BATT_BAL_ITEM_ABNORMAL_STATE:
			oplus_mms_get_item_data(chip->batt_bal_topic, id, &data, false);
			if (data.intval) {
				oplus_ufcs_set_online(chip, false);
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

static void oplus_ufcs_subscribe_batt_bal_topic(struct oplus_mms *topic,
					     void *prv_data)
{
	struct oplus_ufcs *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->batt_bal_topic = topic;
	chip->batt_bal_subs =
		oplus_mms_subscribe(chip->batt_bal_topic, chip,
				    oplus_ufcs_batt_bal_subs_callback, "ufcs");
	if (IS_ERR_OR_NULL(chip->batt_bal_subs)) {
		chg_err("subscribe batt bal topic error, rc=%ld\n",
			PTR_ERR(chip->batt_bal_subs));
		return;
	}

	oplus_mms_get_item_data(chip->batt_bal_topic,
		BATT_BAL_ITEM_ABNORMAL_STATE, &data, true);
	chg_info("abnormal state=%d\n", data.intval);
	if (data.intval) {
		oplus_ufcs_set_online(chip, false);
		chg_err("batt bal abnormal, online set false\n");
	}

	oplus_mms_get_item_data(chip->batt_bal_topic, BATT_BAL_ITEM_CURR_LIMIT, &data, true);
	chip->batt_bal_curr_limit = data.intval;
	chg_info("batt_bal_curr_limit=%d\n", chip->batt_bal_curr_limit);
}

static void oplus_ufcs_gauge_subs_callback(struct mms_subscribe *subs,
					   enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_ufcs *chip = subs->priv_data;
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
			vote(chip->ufcs_disable_votable, NON_STANDARD_VOTER,
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
			vote(chip->ufcs_disable_votable, NON_STANDARD_VOTER,
			     (!chip->batt_hmac || !chip->batt_auth), 0, false);
			break;
		case GAUGE_ITEM_SUBBOARD_TEMP_ERR:
			rc = oplus_mms_get_item_data(chip->gauge_topic, id,
						     &data, false);
			if (rc < 0) {
				chg_err("can't get GAUGE_ITEM_SUBBOARD_TEMP_ERR data, rc=%d\n",
					rc);
			} else {
				if (!!data.intval) {
					chg_info("subboard temp err, max curr set %d\n", chip->subboard_ntc_abnormal_current);
					vote(chip->ufcs_curr_votable, BAD_SUBBOARD_VOTER, true,
					    chip->subboard_ntc_abnormal_current, false);
				} else {
					chg_info("subboard temp ok, need cancel set curr\n");
					vote(chip->ufcs_curr_votable, BAD_SUBBOARD_VOTER, false, 0, false);
				}
			}
			break;
		case GAUGE_ITEM_SUB_BTB_STATE:
			oplus_ufcs_sub_btb_connnect_check(chip);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_ufcs_subscribe_gauge_topic(struct oplus_mms *topic,
					     void *prv_data)
{
	struct oplus_ufcs *chip = prv_data;
	int rc = 0;
	union mms_msg_data data = { 0 };

	chip->gauge_topic = topic;
	chip->gauge_subs =
		oplus_mms_subscribe(chip->gauge_topic, chip,
				    oplus_ufcs_gauge_subs_callback, "ufcs");
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
	vote(chip->ufcs_disable_votable, NON_STANDARD_VOTER, !chip->batt_hmac || !chip->batt_auth, 0, false);

	oplus_gauge_get_fcl_support(chip->gauge_topic, &chip->fcl_support);

	vote(chip->ufcs_boot_votable, GAUGE_TOPIC_VOTER, false, 0, false);
}

static void oplus_ufcs_handshake_error_work(struct work_struct *work)
{
	struct oplus_ufcs *chip =
		container_of(work, struct oplus_ufcs, handshake_error_work);

	chip->ufcs_handshake_error_count++;
}

static void oplus_ufcs_retention_disconnect_work(struct work_struct *work)
{
	struct oplus_ufcs *chip =
		container_of(work, struct oplus_ufcs, retention_disconnect_work);
	union mms_msg_data data = { 0 };

	if (!chip->irq_plugin && chip->retention_state &&
		!chip->adapter_check_third_ufcs && chip->adapter_verify_fail_flag) {
		oplus_cpa_protocol_set_power(chip->cpa_topic, CHG_PROTOCOL_UFCS, chip->config.ufcs_watt_third);
		oplus_cpa_request(chip->cpa_topic, CHG_PROTOCOL_UFCS);
		chip->adapter_check_third_ufcs = true;
		chg_info("add ufcs to request list\n");
	}

	oplus_mms_get_item_data(chip->retention_topic, RETENTION_ITEM_DISCONNECT_COUNT, &data, true);
	chip->connect_error_count = data.intval;
	if (chip->connect_error_count > CONNECT_ERROR_COUNT_LEVEL &&
		chip->cpa_current_type == CHG_PROTOCOL_UFCS) {
		if (chip->retention_ufcs_power > chip->config.ufcs_watt_third && chip->retention_oplus_adapter) {
			oplus_cpa_protocol_set_power(chip->cpa_topic, CHG_PROTOCOL_UFCS, chip->config.ufcs_watt_third);
			oplus_cpa_request(chip->cpa_topic, CHG_PROTOCOL_UFCS);
			oplus_ufcs_force_exit(chip);
			oplus_cpa_switch_end(chip->cpa_topic, CHG_PROTOCOL_UFCS);
			chip->retention_exit_ufcs_flag = EXIT_HIGH_UFCS;
			chg_info("exit_ufcs_flag, rc=%d\n", EXIT_HIGH_UFCS);
		} else {
			vote(chip->ufcs_disable_votable, CONNECT_VOTER, true, true, false);
			oplus_cpa_protocol_disable(chip->cpa_topic, CHG_PROTOCOL_UFCS);
			oplus_ufcs_force_exit(chip);
			oplus_cpa_switch_end(chip->cpa_topic, CHG_PROTOCOL_UFCS);
			chip->adapter_verify_fail_flag = false;
			chip->retention_exit_ufcs_flag = EXIT_THIRD_UFCS;
			chg_info("exit_ufcs_flag, rc=%d\n", EXIT_THIRD_UFCS);
		}
		return;
	} else if (chip->connect_error_count >= HANDSHAKE_ERROR_COUNT_LEVEL &&
		chip->ufcs_handshake_error_count >= HANDSHAKE_ERROR_COUNT_LEVEL) {
		vote(chip->ufcs_disable_votable, CONNECT_VOTER, true, true, false);
		oplus_cpa_protocol_disable(chip->cpa_topic, CHG_PROTOCOL_UFCS);
		oplus_ufcs_force_exit(chip);
		oplus_cpa_switch_end(chip->cpa_topic, CHG_PROTOCOL_UFCS);
		chip->adapter_verify_fail_flag = false;
		chip->retention_exit_ufcs_flag = EXIT_THIRD_UFCS;
		chg_info("exit_ufcs_flag, handshake_error_count=%d\n", chip->ufcs_handshake_error_count);
		return;
	}

	if (READ_ONCE(chip->wired_online)) {
		flush_work(&chip->wired_online_work);
		if (!chip->ufcs_online && chip->retention_state &&
		    chip->cpa_current_type == CHG_PROTOCOL_UFCS)
			schedule_delayed_work(&chip->switch_check_work,
				msecs_to_jiffies(WAIT_BC1P2_GET_TYPE));
		WRITE_ONCE(chip->disconnect_change, false);
	} else {
		WRITE_ONCE(chip->disconnect_change, true);
	}
}

static void oplus_ufcs_retention_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_ufcs *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case RETENTION_ITEM_CONNECT_STATUS:
			oplus_mms_get_item_data(chip->retention_topic, id, &data,
						false);
			chip->retention_state = !!data.intval;
			if (!chip->retention_state) {
				chip->adapter_verify_fail_flag = false;
				chip->retention_oplus_adapter = false;
				chip->retention_exit_ufcs_flag = EXIT_UFCS_FALSE;
				chip->ufcs_handshake_error_count = 0;
				chip->adapter_check_third_ufcs = false;
				vote(chip->ufcs_disable_votable, CONNECT_VOTER, false, false, false);
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
				cancel_delayed_work(&chip->switch_end_recheck_work);
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

static void oplus_ufcs_subscribe_retention_topic(struct oplus_mms *topic,
					   void *prv_data)
{
	struct oplus_ufcs *chip = prv_data;
	union mms_msg_data data = { 0 };
	int rc;

	chip->retention_topic = topic;
	chip->retention_subs = oplus_mms_subscribe(chip->retention_topic, chip,
					     oplus_ufcs_retention_subs_callback,
					     "ufcs");
	if (IS_ERR_OR_NULL(chip->retention_subs)) {
		chg_err("subscribe retention topic error, rc=%ld\n",
			PTR_ERR(chip->retention_subs));
		return;
	}
	chip->connect_error_count_level = CONNECT_ERROR_COUNT_LEVEL;

	rc = oplus_mms_get_item_data(chip->retention_topic, RETENTION_ITEM_DISCONNECT_COUNT, &data, true);
	if (rc >= 0)
		chip->connect_error_count = data.intval;
}

static int oplus_ufcs_plc_enable(struct oplus_plc_protocol *opp, enum oplus_plc_chg_mode mode)
{
	struct oplus_ufcs *chip;
	int rc;

	chip = oplus_plc_protocol_get_priv_data(opp);
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (mode == PLC_CHG_MODE_AUTO) {
		if (get_effective_result(chip->ufcs_not_allow_votable) > 0) {
			mode = PLC_CHG_MODE_BUCK;
			chg_info("mode chang to %s by %s", oplus_plc_chg_mode_str(mode),
				 get_effective_client(chip->ufcs_not_allow_votable));
		} else if (get_effective_result(chip->ufcs_disable_votable) > 0) {
			mode = PLC_CHG_MODE_BUCK;
			chg_info("mode chang to %s by %s", oplus_plc_chg_mode_str(mode),
				 get_effective_client(chip->ufcs_disable_votable));
		} else {
			mode = PLC_CHG_MODE_CP;
			chg_info("mode chang to %s", oplus_plc_chg_mode_str(mode));
		}
		if (mode == PLC_CHG_MODE_BUCK)
			return 0;
	}

	if (mode == PLC_CHG_MODE_BUCK)
		rc = vote(chip->ufcs_not_allow_votable, PLC_VOTER, true, 1, false);
	else
		rc = vote(chip->ufcs_not_allow_votable, PLC_VOTER, false, 0, false);

	return rc;
}

static int oplus_ufcs_plc_disable(struct oplus_plc_protocol *opp)
{
	struct oplus_ufcs *chip;

	chip = oplus_plc_protocol_get_priv_data(opp);
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	vote(chip->ufcs_curr_votable, PLC_VOTER, false, 0, false);
	return vote(chip->ufcs_not_allow_votable, PLC_VOTER, false, 0, false);
}

static int oplus_ufcs_plc_reset_protocol(struct oplus_plc_protocol *opp)
{
	struct oplus_ufcs *chip;
	int rc;

	chip = oplus_plc_protocol_get_priv_data(opp);
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}
	if (!chip->oplus_ufcs_adapter)
		return 0;

	if (!chip->ufcs_charging) {
		if (!is_wired_suspend_votable_available(chip))
			return 0;
		rc = vote(chip->wired_suspend_votable, UFCS_VOTER, true, 1, false);
		msleep(UFCS_RESETADAPTER_SLEEP_1000MS);
		rc |= vote(chip->wired_suspend_votable, UFCS_VOTER, false, 0, false);
		return rc;
	}

	return vote(chip->ufcs_disable_votable, PLC_RETRY_VOTER, true, 1, false);
}

static int oplus_ufcs_plc_set_ibus(struct oplus_plc_protocol *opp, int ibus_ma)
{
	struct oplus_ufcs *chip;

	chip = oplus_plc_protocol_get_priv_data(opp);
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	chip->plc_curr = ibus_ma;
	return 0;
}

static int oplus_ufcs_plc_get_ibus(struct oplus_plc_protocol *opp)
{
	struct oplus_ufcs *chip;

	chip = oplus_plc_protocol_get_priv_data(opp);
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return get_client_vote(chip->ufcs_curr_votable, PLC_VOTER);
}

static int oplus_ufcs_plc_get_chg_mode(struct oplus_plc_protocol *opp)
{
	struct oplus_ufcs *chip;

	chip = oplus_plc_protocol_get_priv_data(opp);
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (chip->ufcs_charging)
		return PLC_CHG_MODE_CP;
	return PLC_CHG_MODE_BUCK;
}

static struct oplus_plc_protocol_desc g_plc_protocol_desc = {
	.name = "ufcs",
	.protocol = BIT(CHG_PROTOCOL_UFCS),
	.current_active = false,
	.ops = {
		.enable = oplus_ufcs_plc_enable,
		.disable = oplus_ufcs_plc_disable,
		.reset_protocol = oplus_ufcs_plc_reset_protocol,
		.set_ibus = oplus_ufcs_plc_set_ibus,
		.get_ibus = oplus_ufcs_plc_get_ibus,
		.get_chg_mode = oplus_ufcs_plc_get_chg_mode
	}
};

static void oplus_ufcs_plc_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_ufcs *chip = subs->priv_data;
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

static void oplus_ufcs_subscribe_plc_topic(struct oplus_mms *topic,
					   void *prv_data)
{
	struct oplus_ufcs *chip = prv_data;
	union mms_msg_data data = { 0 };
	int rc;

	chip->plc_topic = topic;
	chip->plc_subs = oplus_mms_subscribe(chip->plc_topic, chip,
					     oplus_ufcs_plc_subs_callback,
					     "ufcs");
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
		chg_err("register ufcs plc protocol error");
}

#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
static void oplus_ufcs_subscribe_keep_topic(struct oplus_mms *topic,
					    void *prv_data)
{
	struct oplus_ufcs *chip = prv_data;

	chip->keep_topic = topic;
}
#endif

static int oplus_ufcs_update_online(struct oplus_mms *mms,
				    union mms_msg_data *data)
{
	struct oplus_ufcs *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->ufcs_online;

	return 0;
}

static int oplus_ufcs_update_charging(struct oplus_mms *mms,
				      union mms_msg_data *data)
{
	struct oplus_ufcs *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->ufcs_charging;

	return 0;
}

static int oplus_ufcs_update_adapter_id(struct oplus_mms *mms,
					union mms_msg_data *data)
{
	struct oplus_ufcs *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = (int)chip->adapter_id;

	return 0;
}

static int oplus_ufcs_update_oplus_adapter(struct oplus_mms *mms,
					   union mms_msg_data *data)
{
	struct oplus_ufcs *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->oplus_ufcs_adapter;

	return 0;
}

static int oplus_ufcs_update_bcc_max_curr(struct oplus_mms *mms,
					  union mms_msg_data *data)
{
	struct oplus_ufcs *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->bcc.bcc_max_curr;

	return 0;
}

static int oplus_ufcs_update_bcc_min_curr(struct oplus_mms *mms,
					  union mms_msg_data *data)
{
	struct oplus_ufcs *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->bcc.bcc_min_curr;

	return 0;
}

static int oplus_ufcs_update_bcc_exit_curr(struct oplus_mms *mms,
					   union mms_msg_data *data)
{
	struct oplus_ufcs *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->bcc.bcc_exit_curr;

	return 0;
}

static int oplus_ufcs_update_bcc_temp_range(struct oplus_mms *mms,
					    union mms_msg_data *data)
{
	struct oplus_ufcs *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	if ((chip->ufcs_temp_cur_range == UFCS_TEMP_RANGE_NORMAL_LOW) ||
	    (chip->ufcs_temp_cur_range == UFCS_TEMP_RANGE_NORMAL_HIGH) ||
	    (chip->ufcs_temp_cur_range == UFCS_TEMP_RANGE_NORMAL_LOW_PRE))
		data->intval = true;
	else
		data->intval = false;

	return 0;
}

static int oplus_ufcs_update_slow_chg_batt_limit(struct oplus_mms *mms,
					    union mms_msg_data *data)
{
	struct oplus_ufcs *chip;
	int limit = 0;

	if (mms == NULL) {
		chg_err("topic is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (chip)
		limit = chip->slow_chg_batt_limit;

	data->intval = limit;
	return 0;
}

static int oplus_ufcs_update_ufcs_vid(struct oplus_mms *mms,
						union mms_msg_data *data)
{
	struct oplus_ufcs *chip;
	int vid = 0;

	if (mms == NULL) {
		chg_err("topic is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (chip)
		vid = chip->ufcs_vid;

	data->intval = vid;
	return 0;
}

static int oplus_ufcs_update_ufcs_err_type(
	struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_ufcs *chip;
	int err_type = 0;

	if (mms == NULL) {
		chg_err("topic is NULL\n");
		return -EINVAL;
	}

	if (data == NULL) {
		chg_err("data is NULL\n");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (chip)
		err_type = chip->ufcs_err_type;

	data->intval = err_type;
	return 0;
}

static int oplus_ufcs_update_test_mode(
	struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_ufcs *chip;
	bool test_mode = false;

	if (mms == NULL) {
		chg_err("topic is NULL\n");
		return -EINVAL;
	}

	if (data == NULL) {
		chg_err("data is NULL\n");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (chip != NULL && chip->ufcs_online)
		test_mode = oplus_chg_ufcs_is_test_mode(chip);

	data->intval = test_mode;
	return 0;
}

static void oplus_ufcs_topic_update(struct oplus_mms *mms, bool publish)
{
}

static struct mms_item oplus_ufcs_item[] = {
	{
		.desc = {
			.item_id = UFCS_ITEM_ONLINE,
			.update = oplus_ufcs_update_online,
		}
	},
	{
		.desc = {
			.item_id = UFCS_ITEM_CHARGING,
			.update = oplus_ufcs_update_charging,
		}
	},
	{
		.desc = {
			.item_id = UFCS_ITEM_ADAPTER_ID,
			.update = oplus_ufcs_update_adapter_id,
		}
	},
	{
		.desc = {
			.item_id = UFCS_ITEM_OPLUS_ADAPTER,
			.update = oplus_ufcs_update_oplus_adapter,
		}
	},
	{
		.desc = {
			.item_id = UFCS_ITEM_BCC_MAX_CURR,
			.update = oplus_ufcs_update_bcc_max_curr,
		}
	},
	{
		.desc = {
			.item_id = UFCS_ITEM_BCC_MIN_CURR,
			.update = oplus_ufcs_update_bcc_min_curr,
		}
	},
	{
		.desc = {
			.item_id = UFCS_ITEM_BCC_EXIT_CURR,
			.update = oplus_ufcs_update_bcc_exit_curr,
		}
	},
	{
		.desc = {
			.item_id = UFCS_ITEM_BCC_TEMP_RANGE,
			.update = oplus_ufcs_update_bcc_temp_range,
		}
	},
	{
		.desc = {
			.item_id = UFCS_ITEM_SLOW_CHG_BATT_LIMIT,
			.update = oplus_ufcs_update_slow_chg_batt_limit,
		}
	},
	{
		.desc = {
			.item_id = UFCS_ITEM_UFCS_VID,
			.update = oplus_ufcs_update_ufcs_vid,
		}
	},
	{
		.desc = {
			.item_id = UFCS_ITEM_EMARK_POWER,
		}
	},
	{
		.desc = {
			.item_id = UFCS_ITEM_ADAPTER_POWER,
		}
	},
	{
		.desc = {
			.item_id = UFCS_ITEM_ERR_TYPE,
			.update = oplus_ufcs_update_ufcs_err_type,
		}
	},
	{
		.desc = {
			.item_id = UFCS_ITEM_TEST_MODE,
			.update = oplus_ufcs_update_test_mode,
			.dead_thr_enable = true,
			.dead_zone_thr = 1,
		}
	},
};

static const struct oplus_mms_desc oplus_ufcs_desc = {
	.name = "ufcs",
	.type = OPLUS_MMS_TYPE_UFCS,
	.item_table = oplus_ufcs_item,
	.item_num = ARRAY_SIZE(oplus_ufcs_item),
	.update_items = NULL,
	.update_items_num = 0,
	.update_interval = 0, /* ms */
	.update = oplus_ufcs_topic_update,
};

static int oplus_ufcs_topic_init(struct oplus_ufcs *chip)
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

	chip->ufcs_topic =
		devm_oplus_mms_register(chip->dev, &oplus_ufcs_desc, &mms_cfg);
	if (IS_ERR(chip->ufcs_topic)) {
		chg_err("Couldn't register ufcs topic\n");
		rc = PTR_ERR(chip->ufcs_topic);
		return rc;
	}

	return 0;
}

static void oplus_ufcs_online_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_ufcs *chip = virq_data;

	if (chip->auth_data_ok)
		schedule_delayed_work(&chip->send_authdata_to_adsp_work, 0);
}

static int oplus_ufcs_virq_reg(struct oplus_ufcs *chip)
{
	int rc = 0;

	rc = oplus_chg_ic_virq_register(chip->ufcs_ic,
			OPLUS_IC_VIRQ_ONLINE, oplus_ufcs_online_handler, chip);
	if (rc < 0 && rc != -ENOTSUPP)
		chg_err("register OPLUS_IC_VIRQ_ONLINE error, rc=%d", rc);

	return 0;
}

static void oplus_ufcs_virq_unreg(struct oplus_ufcs *chip)
{
	oplus_chg_ic_virq_release(chip->ufcs_ic, OPLUS_IC_VIRQ_ONLINE, chip);
}

static void oplus_ufcs_ic_reg_callback(struct oplus_chg_ic_dev *ic, void *data, bool timeout)
{
	struct oplus_ufcs *chip;
	int rc;

	if (data == NULL) {
		chg_err("ic(%s) data is NULL\n", ic->name);
		return;
	}
	chip = data;

	chip->ufcs_ic = ic;
	rc = oplus_ufcs_virq_reg(chip);
	if (rc < 0) {
		chg_err("virq register error, rc=%d\n", rc);
		chip->ufcs_ic = NULL;
		return;
	}

	rc = oplus_ufcs_topic_init(chip);
	if (rc < 0)
		goto topic_reg_err;

	if (chip->auth_data_ok)
		schedule_delayed_work(&chip->send_authdata_to_adsp_work, 0);

	oplus_mms_wait_topic("common", oplus_ufcs_subscribe_comm_topic, chip);
	oplus_mms_wait_topic("wired", oplus_ufcs_subscribe_wired_topic, chip);
	oplus_mms_wait_topic("cpa", oplus_ufcs_subscribe_cpa_topic, chip);
	oplus_mms_wait_topic("gauge", oplus_ufcs_subscribe_gauge_topic, chip);
	oplus_mms_wait_topic("batt_bal", oplus_ufcs_subscribe_batt_bal_topic, chip);
	oplus_mms_wait_topic("retention", oplus_ufcs_subscribe_retention_topic, chip);
	oplus_mms_wait_topic("plc", oplus_ufcs_subscribe_plc_topic, chip);
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	oplus_mms_wait_topic("state_keep", oplus_ufcs_subscribe_keep_topic, chip);
#endif
	return;

topic_reg_err:
	oplus_ufcs_virq_unreg(chip);
}

static int oplus_ufcs_dpdm_switch_virq_reg(struct oplus_ufcs *chip)
{
	return 0;
}

static void oplus_ufcs_dpdm_switch_virq_unreg(struct oplus_ufcs *chip)
{}

static void oplus_ufcs_dpdm_switch_reg_callback(struct oplus_chg_ic_dev *ic, void *data, bool timeout)
{
	struct oplus_ufcs *chip;
	const char *name;
	int rc;

	if (data == NULL) {
		chg_err("ic(%s) data is NULL\n", ic->name);
		return;
	}
	chip = data;

	rc = oplus_ufcs_dpdm_switch_virq_reg(chip);
	if (rc < 0) {
		chg_err("dpdm_switch virq register error, rc=%d\n", rc);
		return;
	}
	chip->dpdm_switch = ic;

	name = of_get_oplus_chg_ic_name(chip->dev->of_node, "oplus,ufcs_ic", 0);
	oplus_chg_ic_wait_ic(name, oplus_ufcs_ic_reg_callback, chip);
}

static void oplus_ufcs_cp_error_handler(struct oplus_ufcs *chip)
{
	oplus_ufcs_set_ic_ufcs_curr_vote(chip, chip->cp_work_mode);
}

static void oplus_ufcs_cp_err_analyze(struct oplus_ufcs *chip, struct oplus_chg_ic_err_msg *msg)
{
	if (msg->type != OPLUS_IC_ERR_CP)
		return;
	oplus_ufcs_cp_error_handler(chip);
}

static void oplus_ufcs_cp_err_handler_work(struct work_struct *work)
{
	struct oplus_ufcs *chip =
		container_of(work, struct oplus_ufcs, cp_err_handler_work);
	struct oplus_chg_ic_err_msg *msg, *tmp;
	struct list_head msg_list;

	if (!chip->ufcs_online)
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
		oplus_ufcs_cp_err_analyze(chip, msg);
		oplus_print_ic_err(msg);
		list_del(&msg->list);
		kfree(msg);
	}
}

static void oplus_ufcs_cp_online_handler_work(struct work_struct *work)
{
	struct oplus_ufcs *chip =
		container_of(work, struct oplus_ufcs, cp_online_handler_work);
	bool cp_offline = !!atomic_read(&chip->cp_offline);
	vote(chip->ufcs_disable_votable, CP_OFFLINE_VOTER, cp_offline, cp_offline, false);
}

static void oplus_ufcs_cp_offline_handler_work(struct work_struct *work)
{
	struct oplus_ufcs *chip =
		container_of(work, struct oplus_ufcs, cp_offline_handler_work);
	bool cp_offline = !!atomic_read(&chip->cp_offline);
	vote(chip->ufcs_disable_votable, CP_OFFLINE_VOTER, cp_offline, cp_offline, false);
}

static void oplus_ufcs_cp_err_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_ufcs *chip = virq_data;

	schedule_work(&chip->cp_err_handler_work);
}

static void oplus_ufcs_cp_online_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_ufcs *chip = virq_data;

	chg_info("%s online\n", ic_dev->manu_name);
	atomic_set(&chip->cp_offline, 0);
	schedule_work(&chip->cp_online_handler_work);
}

static void oplus_ufcs_cp_offline_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_ufcs *chip = virq_data;

	chg_err("%s offline\n", ic_dev->manu_name);
	atomic_set(&chip->cp_offline, 1);
	schedule_work(&chip->cp_offline_handler_work);
}

static int oplus_ufcs_cp_virq_reg(struct oplus_ufcs *chip)
{
	int rc;

	rc = oplus_chg_ic_virq_register(chip->cp_ic, OPLUS_IC_VIRQ_ERR,
		oplus_ufcs_cp_err_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_ERR error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(chip->cp_ic, OPLUS_IC_VIRQ_ONLINE,
		oplus_ufcs_cp_online_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_ONLINE error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(chip->cp_ic, OPLUS_IC_VIRQ_OFFLINE,
		oplus_ufcs_cp_offline_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_OFFLINE error, rc=%d", rc);
	return 0;
}

static void oplus_ufcs_cp_virq_unreg(struct oplus_ufcs *chip)
{
	oplus_chg_ic_virq_release(chip->cp_ic, OPLUS_IC_VIRQ_ERR, chip);
	oplus_chg_ic_virq_release(chip->cp_ic, OPLUS_IC_VIRQ_ONLINE, chip);
	oplus_chg_ic_virq_release(chip->cp_ic, OPLUS_IC_VIRQ_OFFLINE, chip);
}

static void oplus_ufcs_cp_ic_reg_callback(struct oplus_chg_ic_dev *ic, void *data, bool timeout)
{
	struct oplus_ufcs *chip;
	const char *name;
	int rc;

	if (data == NULL) {
		chg_err("ic(%s) data is NULL\n", ic->name);
		return;
	}
	chip = data;

	chip->cp_ic = ic;
	rc = oplus_ufcs_cp_virq_reg(chip);
	if (rc < 0) {
		chg_err("cp virq register error, rc=%d\n", rc);
		return;
	}

	name = of_get_oplus_chg_ic_name(chip->dev->of_node, "oplus,dpdm_switch_ic", 0);
	oplus_chg_ic_wait_ic(name, oplus_ufcs_dpdm_switch_reg_callback, chip);
}

static int oplus_ufcs_curr_vote_callback(struct votable *votable, void *data,
					 int curr_ma, const char *client,
					 bool step)
{
	struct oplus_ufcs *chip = data;

	if (curr_ma < 0)
		return -EINVAL;

	chip->target_curr_ma = curr_ma;
	chg_info("%s set ufcs target current to %d mA\n", client, curr_ma);

	return 0;
}

static int oplus_ufcs_disable_vote_callback(struct votable *votable, void *data,
					    int disable, const char *client,
					    bool step)
{
	struct oplus_ufcs *chip = data;

	if (disable < 0)
		chip->ufcs_disable = false;
	else
		chip->ufcs_disable = !!disable;
	chg_info("%s set ufcs disable to %s\n", client, chip->ufcs_disable ? "true" : "false");

	return 0;
}

static int oplus_ufcs_not_allow_vote_callback(struct votable *votable, void *data,
					      int not_allow, const char *client,
					      bool step)
{
	struct oplus_ufcs *chip = data;
	union mms_msg_data mms_data = { 0 };
	int rc;

	if (not_allow < 0)
		chip->ufcs_not_allow = false;
	else
		chip->ufcs_not_allow = !!not_allow;
	chg_info("%s set ufcs not allow to %s\n", client, chip->ufcs_not_allow ? "true" : "false");
	if (!chip->ufcs_not_allow) {
		if (!(work_busy(&chip->switch_check_work.work) & (WORK_BUSY_RUNNING))) {
			rc = oplus_mms_get_item_data(chip->cpa_topic, CPA_ITEM_ALLOW, &mms_data, false);
			if (rc < 0)
				return 0;
			if (mms_data.intval == CHG_PROTOCOL_UFCS)
				schedule_delayed_work(&chip->switch_check_work, 0);
		} else {
			chg_info("switch_check_work is running now!");
		}
	}

	return 0;
}

static int oplus_ufcs_boot_vote_callback(struct votable *votable, void *data,
					 int boot, const char *client,
					 bool step)
{
	struct oplus_ufcs *chip = data;

	/* The UFCS module startup has not been completed and will not be processed temporarily */
	if (!!boot)
		return 0;

	oplus_cpa_protocol_ready(chip->cpa_topic, CHG_PROTOCOL_UFCS);
	return 0;
}

static int oplus_ufcs_parse_charge_strategy(struct oplus_ufcs *chip)
{
	int rc;
	int length = 0;
	int rang_temp_tmp[7] = { 0, 50, 120, 200, 350, 440, 510 };
	int high_temp_tmp[6] = { 425, 430, 435, 400, 415, 420 };
	int high_curr_tmp[6] = { 4000, 3000, 2000, 3000, 4000, 4000 };
	int low_curr_temp_tmp[4] = { 120, 200, 300, 440 };
	struct device_node *node = oplus_get_node_by_type(chip->dev->of_node);

	rc = of_property_read_u32(node, "oplus,ufcs_warm_allow_vol",
				  &chip->limits.ufcs_warm_allow_vol);
	if (rc)
		chip->limits.ufcs_warm_allow_vol = 4000;

	rc = of_property_read_u32(node, "oplus,ufcs_warm_allow_soc",
				  &chip->limits.ufcs_warm_allow_soc);
	if (rc)
		chip->limits.ufcs_warm_allow_soc = 50;

	rc = of_property_read_u32(node, "oplus,ufcs_strategy_normal_current",
				  &chip->limits.ufcs_strategy_normal_current);
	if (rc)
		chip->limits.ufcs_strategy_normal_current = 6000;

	rc = of_property_read_u32(node, "oplus,ufcs_over_high_or_low_current",
				  &chip->limits.ufcs_over_high_or_low_current);
	if (rc)
		chip->limits.ufcs_over_high_or_low_current = -EINVAL;

	rc = of_property_read_u32(node, "oplus,ufcs_full_cool_sw_vbat",
				  &chip->limits.ufcs_full_cool_sw_vbat);
	if (rc)
		chip->limits.ufcs_full_cool_sw_vbat = 4430;

	rc = of_property_read_u32(node, "oplus,ufcs_full_normal_sw_vbat",
				  &chip->limits.ufcs_full_normal_sw_vbat);
	if (rc)
		chip->limits.ufcs_full_normal_sw_vbat = 4495;

	rc = of_property_read_u32(node, "oplus,ufcs_full_normal_hw_vbat",
				  &chip->limits.ufcs_full_normal_hw_vbat);
	if (rc)
		chip->limits.ufcs_full_normal_hw_vbat = 4500;

	rc = of_property_read_u32(node, "oplus,ufcs_full_cool_sw_vbat_third",
				  &chip->limits.ufcs_full_cool_sw_vbat_third);
	if (rc)
		chip->limits.ufcs_full_cool_sw_vbat_third = 4430;

	rc = of_property_read_u32(node, "oplus,ufcs_full_normal_sw_vbat_third",
				  &chip->limits.ufcs_full_normal_sw_vbat_third);
	if (rc)
		chip->limits.ufcs_full_normal_sw_vbat_third = 4430;

	rc = of_property_read_u32(node, "oplus,ufcs_full_normal_hw_vbat_third",
				  &chip->limits.ufcs_full_normal_hw_vbat_third);
	if (rc)
		chip->limits.ufcs_full_normal_hw_vbat_third = 4440;

	rc = of_property_read_u32(node, "oplus,ufcs_full_warm_vbat",
				  &chip->limits.ufcs_full_warm_vbat);
	if (rc)
		chip->limits.ufcs_full_warm_vbat = 4150;

	chg_info("ufcs_warm_allow_vol=%d, ufcs_warm_allow_soc=%d, ufcs_strategy_normal_current=%d,"
		 "ufcs_over_high_or_low_current=%d, ufcs_full_cool_sw_vbat=%d, ufcs_full_normal_sw_vbat=%d,"
		 "limits.ufcs_full_normal_hw_vbat=%d, ufcs_full_cool_sw_vbat_third=%d, ufcs_full_normal_sw_vbat_third=%d,"
		 "ufcs_full_normal_hw_vbat_third=%d, ufcs_full_warm_vbat=%d \n",
		  chip->limits.ufcs_warm_allow_vol,
		  chip->limits.ufcs_warm_allow_soc,
		  chip->limits.ufcs_strategy_normal_current,
		  chip->limits.ufcs_over_high_or_low_current,
		  chip->limits.ufcs_full_cool_sw_vbat,
		  chip->limits.ufcs_full_normal_sw_vbat,
		  chip->limits.ufcs_full_normal_hw_vbat,
		  chip->limits.ufcs_full_cool_sw_vbat_third,
		  chip->limits.ufcs_full_normal_sw_vbat_third,
		  chip->limits.ufcs_full_normal_hw_vbat_third,
		  chip->limits.ufcs_full_warm_vbat);

	rc = of_property_read_u32(node, "oplus,ufcs_timeout_third",
				  &chip->limits.ufcs_timeout_third);
	if (rc)
		chip->limits.ufcs_timeout_third = 9000;

	rc = of_property_read_u32(node, "oplus,ufcs_timeout_oplus",
				  &chip->limits.ufcs_timeout_oplus);
	if (rc)
		chip->limits.ufcs_timeout_oplus = 7200;

	rc = of_property_read_u32(node, "oplus,ufcs_ibat_over_third",
				  &chip->limits.ufcs_ibat_over_third);
	if (rc)
		chip->limits.ufcs_ibat_over_third = 4000;

	rc = of_property_read_u32(node, "oplus,ufcs_ibat_over_oplus",
				  &chip->limits.ufcs_ibat_over_oplus);
	if (rc)
		chip->limits.ufcs_ibat_over_oplus = 17000;

	rc = of_property_read_u32(node, "oplus,ufcs_ibus_over_oplus",
				  &chip->limits.ufcs_ibus_over_oplus);
	if (rc)
		chip->limits.ufcs_ibus_over_oplus = 15000;

	rc = of_property_read_u32(node, "oplus,ufcs_bcc_max_curr",
				  &chip->bcc_max_curr);
	if (rc)
		chip->bcc_max_curr = 15000;

	rc = of_property_read_u32(node, "oplus,ufcs_bcc_min_curr",
				  &chip->bcc_min_curr);
	if (rc)
		chip->bcc_min_curr = 1000;

	rc = of_property_read_u32(node, "oplus,ufcs_bcc_exit_curr",
				  &chip->bcc_exit_curr);
	if (rc)
		chip->bcc_exit_curr = 1000;

	chg_info("ufcs_timeout_third=%d, ufcs_timeout_oplus=%d, ufcs_ibat_over_third=%d,"
		 "ufcs_ibat_over_oplus=%d, ufcs_ibus_over_oplus=%d, "
		 "bcc_max_curr=%d, bcc_min_curr=%d, bcc_exit_curr=%d \n",
		 chip->limits.ufcs_timeout_third, chip->limits.ufcs_timeout_oplus,
		 chip->limits.ufcs_ibat_over_third, chip->limits.ufcs_ibat_over_oplus,
		 chip->limits.ufcs_ibus_over_oplus,
		 chip->bcc_max_curr, chip->bcc_min_curr, chip->bcc_exit_curr);

	rc = of_property_count_elems_of_size(
		node, "oplus,ufcs_strategy_batt_high_temp", sizeof(u32));
	if (rc >= 0) {
		length = rc;
		if (length > 6)
			length = 6;
		rc = of_property_read_u32_array(node, "oplus,ufcs_strategy_batt_high_temp",
						(u32 *)high_temp_tmp, length);
		if (rc < 0)
			chg_err("parse ufcs_strategy_batt_high_temp failed, "
				 "rc=%d\n", rc);
	} else {
		length = 6;
		chg_err("parse ufcs_strategy_batt_high_temp error, rc=%d\n", rc);
	}

	chip->limits.ufcs_strategy_batt_high_temp0 = high_temp_tmp[0];
	chip->limits.ufcs_strategy_batt_high_temp1 = high_temp_tmp[1];
	chip->limits.ufcs_strategy_batt_high_temp2 = high_temp_tmp[2];
	chip->limits.ufcs_strategy_batt_low_temp0 = high_temp_tmp[3];
	chip->limits.ufcs_strategy_batt_low_temp1 = high_temp_tmp[4];
	chip->limits.ufcs_strategy_batt_low_temp2 = high_temp_tmp[5];
	chg_info("length = %d, high_temp[%d, %d, %d, %d, %d, %d]\n", length,
		 high_temp_tmp[0], high_temp_tmp[1], high_temp_tmp[2],
		 high_temp_tmp[3], high_temp_tmp[4], high_temp_tmp[5]);

	rc = of_property_count_elems_of_size(
		node, "oplus,ufcs_strategy_high_current", sizeof(u32));
	if (rc >= 0) {
		length = rc;
		if (length > 6)
			length = 6;
		rc = of_property_read_u32_array(node, "oplus,ufcs_strategy_high_current",
						(u32 *)high_curr_tmp, length);
		if (rc < 0) {
			chg_err("parse ufcs_strategy_high_current failed, "
				 "rc=%d\n", rc);
		}
	} else {
		length = 6;
		chg_err("parse ufcs_strategy_high_current error, rc=%d\n", rc);
	}
	chip->limits.ufcs_strategy_high_current0 = high_curr_tmp[0];
	chip->limits.ufcs_strategy_high_current1 = high_curr_tmp[1];
	chip->limits.ufcs_strategy_high_current2 = high_curr_tmp[2];
	chip->limits.ufcs_strategy_low_current0 = high_curr_tmp[3];
	chip->limits.ufcs_strategy_low_current1 = high_curr_tmp[4];
	chip->limits.ufcs_strategy_low_current2 = high_curr_tmp[5];
	chg_info("length = %d, high_current[%d, %d, %d, %d, %d, %d]\n", length,
		 high_curr_tmp[0], high_curr_tmp[1], high_curr_tmp[2],
		 high_curr_tmp[3], high_curr_tmp[4], high_curr_tmp[5]);

	rc = of_property_count_elems_of_size(
		node, "oplus,ufcs_charge_strategy_temp", sizeof(u32));
	if (rc >= 0) {
		chip->limits.ufcs_strategy_temp_num = rc;
		if (chip->limits.ufcs_strategy_temp_num > 7)
			chip->limits.ufcs_strategy_temp_num = 7;
		rc = of_property_read_u32_array(
			node, "oplus,ufcs_charge_strategy_temp",
			(u32 *)rang_temp_tmp,
			chip->limits.ufcs_strategy_temp_num);
		if (rc < 0)
			chg_err("parse ufcs_charge_strategy_temp failed, "
				"rc=%d\n", rc);
	} else {
		chip->limits.ufcs_strategy_temp_num = 7;
		chg_err("parse ufcs_charge_strategy_temp error, rc=%d\n", rc);
	}
	chip->limits.ufcs_batt_over_low_temp = rang_temp_tmp[0] - UFCS_COLD_TEMP_RANGE_THD;
	chip->limits.ufcs_little_cold_temp = rang_temp_tmp[1];
	chip->limits.ufcs_cool_temp = rang_temp_tmp[2];
	chip->limits.ufcs_little_cool_temp = rang_temp_tmp[3];
	chip->limits.ufcs_normal_low_temp = rang_temp_tmp[4];
	chip->limits.ufcs_normal_high_temp = rang_temp_tmp[5];
	chip->limits.ufcs_batt_over_high_temp = rang_temp_tmp[6];
	rc = of_property_read_u32(node, "oplus,ufcs_little_cool_high_temp", &chip->limits.ufcs_little_cool_high_temp);
	if (rc)
		chip->limits.ufcs_little_cool_high_temp = -EINVAL;
	rc = of_property_read_u32(node, "oplus,ufcs_normal_low_pre_temp", &chip->limits.ufcs_normal_low_pre_temp);
	if (rc)
		chip->limits.ufcs_normal_low_pre_temp = -EINVAL;
	chg_info("ufcs_charge_strategy_temp num = %d, [%d, %d, %d, %d, %d, %d, %d, %d, %d]\n",
		 chip->limits.ufcs_strategy_temp_num, rang_temp_tmp[0],
		 rang_temp_tmp[1], rang_temp_tmp[2], rang_temp_tmp[3],
		 chip->limits.ufcs_little_cool_high_temp,
		 chip->limits.ufcs_normal_low_pre_temp,
		 rang_temp_tmp[4], rang_temp_tmp[5], rang_temp_tmp[6]);
	chip->limits.default_ufcs_normal_high_temp =
		chip->limits.ufcs_normal_high_temp;
	chip->limits.default_ufcs_normal_low_temp =
		chip->limits.ufcs_normal_low_temp;
	chip->limits.default_ufcs_normal_low_pre_temp =
		chip->limits.ufcs_normal_low_pre_temp;
	chip->limits.default_ufcs_little_cool_temp =
		chip->limits.ufcs_little_cool_temp;
	chip->limits.default_ufcs_little_cool_high_temp = chip->limits.ufcs_little_cool_high_temp;
	chip->limits.default_ufcs_cool_temp = chip->limits.ufcs_cool_temp;
	chip->limits.default_ufcs_little_cold_temp =
		chip->limits.ufcs_little_cold_temp;

	rc = of_property_count_elems_of_size(
		node, "oplus,ufcs_low_curr_full_strategy_temp", sizeof(u32));
	if (rc >= 0) {
		length = rc;
		if (length > 4)
			length = 4;
		rc = of_property_read_u32_array(
			node, "oplus,ufcs_low_curr_full_strategy_temp",
			(u32 *)low_curr_temp_tmp, length);
		if (rc < 0)
			chg_err("parse ufcs_low_curr_full_strategy_temp "
				"failed, rc=%d\n", rc);
	} else {
		length = 4;
		chg_err("parse ufcs_low_curr_full_strategy_temp error, rc=%d\n", rc);
	}

	chip->limits.ufcs_low_curr_full_cool_temp = low_curr_temp_tmp[0];
	chip->limits.ufcs_low_curr_full_little_cool_temp = low_curr_temp_tmp[1];
	chip->limits.ufcs_low_curr_full_normal_low_temp = low_curr_temp_tmp[2];
	chip->limits.ufcs_low_curr_full_normal_high_temp = low_curr_temp_tmp[3];
	chg_info("length = %d, low_curr_temp[%d, %d, %d, %d]\n", length,
		 low_curr_temp_tmp[0], low_curr_temp_tmp[1],
		 low_curr_temp_tmp[2], low_curr_temp_tmp[3]);

	rc = of_property_read_u32(node, "oplus_spec,ufcs_low_temp", &chip->limits.ufcs_low_temp);
	if (rc < 0) {
		chip->limits.ufcs_low_temp = 0;
	} else {
		chip->limits.ufcs_batt_over_low_temp = chip->limits.ufcs_low_temp - UFCS_COLD_TEMP_RANGE_THD;
		chg_info("oplus_spec,ufcs_low_temp is %d %d\n", chip->limits.ufcs_low_temp,
			chip->limits.ufcs_batt_over_low_temp);
	}
	rc = of_property_read_u32(node, "oplus_spec,ufcs_high_temp", &chip->limits.ufcs_high_temp);
	if (rc < 0)
		chip->limits.ufcs_high_temp = 500;
	else
		chg_info("oplus_spec,ufcs_high_temp is %d\n", chip->limits.ufcs_high_temp);
	rc = of_property_read_u32(node, "oplus_spec,ufcs_low_soc", &chip->limits.ufcs_low_soc);
	if (rc < 0)
		chip->limits.ufcs_low_soc = 0;
	else
		chg_info("oplus_spec,ufcs_low_soc is %d\n", chip->limits.ufcs_low_soc);
	rc = of_property_read_u32(node, "oplus_spec,ufcs_high_soc", &chip->limits.ufcs_high_soc);
	if (rc < 0)
		chip->limits.ufcs_high_soc = 90;
	else
		chg_info("oplus_spec,ufcs_high_soc is %d\n", chip->limits.ufcs_high_soc);

	rc = of_property_read_u32(node, "oplus_spec,ufcs_high_soc_third", &chip->limits.ufcs_high_soc_third);
	if (rc < 0)
		chip->limits.ufcs_high_soc_third = chip->limits.ufcs_high_soc;
	chg_info("oplus_spec,ufcs_high_soc_third is %d\n", chip->limits.ufcs_high_soc_third);
	chip->high_soc = chip->limits.ufcs_high_soc;

	return rc;
}

static int oplus_ufcs_parse_low_curr_full_curves(struct oplus_ufcs *chip)
{
	struct device_node *node, *full_node;
	int rc = 0, i, length;

	if (!chip)
		return -ENODEV;

	node = chip->dev->of_node;

	full_node = of_get_child_by_name(node, "ufcs_charge_low_curr_full");
	if (!full_node) {
		chg_err("Can not find ufcs_charge_low_curr_full node\n");
		return -EINVAL;
	}

	for (i = 0; i < UFCS_LOW_CURR_FULL_CURVE_TEMP_MAX; i++) {
		rc = of_property_count_elems_of_size(full_node, strategy_low_curr_full[i], sizeof(u32));
		if (rc < 0) {
			chg_err("Count low curr %s failed, rc=%d\n", strategy_low_curr_full[i], rc);
			return rc;
		}

		length = rc;
		switch (i) {
		case UFCS_LOW_CURR_FULL_CURVE_TEMP_LITTLE_COOL:
			rc = of_property_read_u32_array(full_node, strategy_low_curr_full[i],
							(u32 *)chip->low_curr_full_curves_temp[i].full_curves, length);
			chip->low_curr_full_curves_temp[i].full_curve_num = length / 3;
			break;
		case UFCS_LOW_CURR_FULL_CURVE_TEMP_NORMAL_LOW:
			rc = of_property_read_u32_array(full_node, strategy_low_curr_full[i],
							(u32 *)chip->low_curr_full_curves_temp[i].full_curves, length);
			chip->low_curr_full_curves_temp[i].full_curve_num = length / 3;
			break;
		case UFCS_LOW_CURR_FULL_CURVE_TEMP_NORMAL_HIGH:
			rc = of_property_read_u32_array(full_node, strategy_low_curr_full[i],
							(u32 *)chip->low_curr_full_curves_temp[i].full_curves, length);
			chip->low_curr_full_curves_temp[i].full_curve_num = length / 3;
			break;
		default:
			break;
		}
	}

	return rc;
}

static int oplus_ufcs_parse_dt(struct oplus_ufcs *chip)
{
	struct device_node *node = oplus_get_node_by_type(chip->dev->of_node);
	struct oplus_ufcs_config *config = &chip->config;
	int rc;
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
	config->adsp_ufcs_project = of_property_read_bool(node, "oplus,adsp_ufcs_project");

	config->ufcs_need_reset_adapter = of_property_read_bool(node, "oplus,ufcs_need_reset_adapter");
	chg_info("ufcs_need_reset_adapter:%d\n", config->ufcs_need_reset_adapter);

	rc = of_property_read_u32(node, "oplus,curr_table_type",
				  &chip->curr_table_type);
	if (rc < 0) {
		chip->curr_table_type = UFCS_CURR_BIDIRECT_TABLE;
		chg_err("UFCS_CURR_CP_TABLE not obtained,curr_table_type = %d\n",
			chip->curr_table_type);
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

	rc = of_property_read_u32(node, "oplus,ufcs_boot_time_retry",
					 &config->ufcs_boot_time_retry);
	if (rc < 0)
		config->ufcs_boot_time_retry = 0;
	if (config->ufcs_boot_time_retry > UFCS_BOOT_RETRY_TIME_MAX_SECOND)
		config->ufcs_boot_time_retry = UFCS_BOOT_RETRY_TIME_MAX_SECOND;

	rc = of_property_read_string(node, "oplus,curve_strategy_name", (const char **)&config->curve_strategy_name);
	if (rc < 0) {
		chg_err("oplus,curve_strategy_name reading failed, rc=%d\n", rc);
		config->curve_strategy_name = "pps_ufcs_curve";
	}
	chg_info("curve_strategy_name=%s\n", config->curve_strategy_name);

	rc = of_property_read_u32(node, "oplus,high_imp_compensation_setting_mv",
				  &config->high_imp_compensation_setting_mv);
	if (rc < 0) {
		chg_err("get oplus,high_imp_compensation_setting_mv error, rc=%d\n",
			rc);
		config->high_imp_compensation_setting_mv = 0;
	}
	chg_info("high_imp_compensation_setting_mv=%u\n", config->high_imp_compensation_setting_mv);

	rc = of_property_read_u32(node, "oplus,upper_compensation_mv",
				  &config->upper_compensation_mv);
	if (rc < 0) {
		chg_err("get oplus,upper_compensation_mv error, rc=%d\n",
			rc);
		config->upper_compensation_mv = 300;
	}
	chg_info("upper_compensation_mv=%u\n", config->upper_compensation_mv);

	rc = of_property_read_u32(node, "oplus,subboard_ntc_abnormal_current", &chip->subboard_ntc_abnormal_current);
	if (rc)
		chip->subboard_ntc_abnormal_current = SUBBOARD_TEMP_ABNORMAL_MAX_CURR;
	chg_info("subboard_ntc_abnormal_cool_down:%d\n", chip->subboard_ntc_abnormal_current);

	chip->lift_vbus_use_cpvout = of_property_read_bool(node, "oplus,lift_vbus_use_cpvout");
	chg_info("lift_vbus_use_cpvout:%d\n", chip->lift_vbus_use_cpvout);

	rc = of_property_read_u32(node, "oplus,ufcs_watt_third",
				  &config->ufcs_watt_third);
	if (rc < 0) {
		chg_err("get oplus,ufcs_watt_third property error, rc=%d\n",
			rc);
		config->ufcs_watt_third = SUPPORT_THIRD_UFCS_POWER;
	}

	rc = of_property_read_u32(node, "oplus,ufcs_full_recheck_temp", &config->ufcs_full_recheck_temp);
	if (rc < 0) {
		chg_err("not support ufcs full recheck\n");
		config->ufcs_full_recheck_temp = -EINVAL;
	}

	config->ufcs_wd_separate_work = of_property_read_bool(node, "oplus,ufcs_wd_separate_work");
	chg_info("ufcs_wd_separate_work=%d\n", config->ufcs_wd_separate_work);

	ufcs_pr_parse_dt(chip, node);

	(void)oplus_ufcs_parse_charge_strategy(chip);
	(void)oplus_ufcs_parse_low_curr_full_curves(chip);

	return 0;
}

static int oplus_ufcs_vote_init(struct oplus_ufcs *chip)
{
	int rc;

	chip->ufcs_curr_votable = create_votable("UFCS_CURR", VOTE_MIN, oplus_ufcs_curr_vote_callback, chip);
	if (IS_ERR(chip->ufcs_curr_votable)) {
		rc = PTR_ERR(chip->ufcs_curr_votable);
		chg_err("creat ufcs_curr_votable error, rc=%d\n", rc);
		chip->ufcs_curr_votable = NULL;
		return rc;
	}
	vote(chip->ufcs_curr_votable, MAX_VOTER, true, chip->config.curr_max_ma, false);

	chip->ufcs_disable_votable =
		create_votable("UFCS_DISABLE", VOTE_SET_ANY, oplus_ufcs_disable_vote_callback, chip);
	if (IS_ERR(chip->ufcs_disable_votable)) {
		rc = PTR_ERR(chip->ufcs_disable_votable);
		chg_err("creat ufcs_disable_votable error, rc=%d\n", rc);
		chip->ufcs_disable_votable = NULL;
		goto creat_disable_votable_err;
	}

	chip->ufcs_not_allow_votable =
		create_votable("UFCS_NOT_ALLOW", VOTE_MIN, oplus_ufcs_not_allow_vote_callback, chip);
	if (IS_ERR(chip->ufcs_not_allow_votable)) {
		rc = PTR_ERR(chip->ufcs_not_allow_votable);
		chg_err("creat ufcs_not_allow_votable error, rc=%d\n", rc);
		chip->ufcs_not_allow_votable = NULL;
		goto creat_not_allow_votable_err;
	}

	chip->ufcs_boot_votable =
		create_votable("UFCS_BOOT", VOTE_SET_ANY, oplus_ufcs_boot_vote_callback, chip);
	if (IS_ERR(chip->ufcs_boot_votable)) {
		rc = PTR_ERR(chip->ufcs_boot_votable);
		chg_err("creat ufcs_boot_votable error, rc=%d\n", rc);
		chip->ufcs_boot_votable = NULL;
		goto creat_boot_votable_err;
	}
	vote(chip->ufcs_boot_votable, AUTH_VOTER, true, 1, false);
	vote(chip->ufcs_boot_votable, SHELL_TEMP_VOTER, true, 1, false);
	vote(chip->ufcs_boot_votable, COMM_TOPIC_VOTER, true, 1, false);
	vote(chip->ufcs_boot_votable, WIRED_TOPIC_VOTER, true, 1, false);
	vote(chip->ufcs_boot_votable, GAUGE_TOPIC_VOTER, true, 1, false);
	vote(chip->ufcs_boot_votable, CPA_TOPIC_VOTER, true, 1, false);

	return 0;

creat_boot_votable_err:
	destroy_votable(chip->ufcs_not_allow_votable);
creat_not_allow_votable_err:
	destroy_votable(chip->ufcs_disable_votable);
creat_disable_votable_err:
	destroy_votable(chip->ufcs_curr_votable);
	return rc;
}

static int oplus_ufcs_get_input_node_impedance(void *data)
{
	struct oplus_ufcs *chip;
	int vac, iin, vchg;
	int r_mohm;
	int rc;

	if (data == NULL)
		return -EINVAL;
	chip = data;

	if (!chip->ufcs_charging) {
		chg_err("ufcs_charging is false, can't read input node impedance\n");
		return -EINVAL;
	}

	rc = oplus_ufcs_cp_get_vac(chip, &vac);
	if (rc < 0) {
		chg_debug("not support get cp vac, rc=%d, try vin\n", rc);
		rc = oplus_ufcs_cp_get_vin(chip, &vac);
		if (rc < 0) {
			chg_err("cant get cp vin, rc=%d\n", rc);
			return rc;
		}
	}
	rc = oplus_ufcs_cp_get_iin(chip, &iin);
	if (rc < 0 || iin == 0) {
		chg_debug("direct charge project use sourceinfo iin, rc=%d\n", rc);
		iin =  UFCS_SOURCE_INFO_CURR(chip->src_info);
		if (iin == 0) {
			chg_err("iin is 0, can't calculate input node impedance\n");
			return -EINVAL;
		}
	}

	vchg = UFCS_SOURCE_INFO_VOL(chip->src_info);

	r_mohm = (vchg - vac) * 1000 / iin;
	if (r_mohm < 0) {
		chg_err("ufcs_input_node: r_mohm=%d\n", r_mohm);
		r_mohm = 0;
	}
	chg_info("r_mohm = %d, vchg = %d, vac = %d, iin = %d\n", r_mohm, vchg, vac, iin);

	return r_mohm;
}

static int oplus_ufcs_get_mos_node_impedance(void *data)
{
	struct oplus_ufcs *chip;
	int iin, vchg;
	int r_mohm;
	int rc;
	int batt_num = 1, vbatt = 3800;
	union mms_msg_data msg_data = { 0 };

	if (data == NULL)
		return -EINVAL;
	chip = data;

	if (!chip->ufcs_charging) {
		chg_err("ufcs_charging is false, can't read input node impedance\n");
		return -EINVAL;
	}
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MIN, &msg_data, true);
	if (unlikely(rc < 0)) {
		vbatt = 3800;
	} else {
		vbatt = msg_data.intval;
	}

	iin =  UFCS_SOURCE_INFO_CURR(chip->src_info);
	if (iin == 0) {
		chg_err("iin is 0, can't calculate input node impedance\n");
		return -EINVAL;
	}
	vchg = UFCS_SOURCE_INFO_VOL(chip->src_info);
	batt_num = oplus_gauge_get_batt_num();

	r_mohm = (vchg - batt_num * vbatt) * 1000 / iin;
	if (r_mohm < 0) {
		chg_err("ufcs_input_node: r_mohm=%d\n", r_mohm);
		r_mohm = 0;
	}
	chg_err("r_mohm = %d, vchg = %d, vbatt = %d, batt_num = %d, iin = %d\n", r_mohm, vchg, vbatt, batt_num, iin);

	return r_mohm;
}

static int oplus_ufcs_init_imp_node(struct oplus_ufcs *chip)
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
		if (!strcmp(name, "ufcs_input")) {
			chip->input_imp_node =
				oplus_imp_node_register(child, chip->dev, chip, oplus_ufcs_get_input_node_impedance);
			if (IS_ERR_OR_NULL(chip->input_imp_node)) {
				chg_err("%s register error, rc=%ld\n", child->name, PTR_ERR(chip->input_imp_node));
				chip->input_imp_node = NULL;
				continue;
			}
			oplus_imp_node_set_active(chip->input_imp_node, true);
		} else if (!strcmp(name, "ufcs_mos")) {
			chip->input_imp_node =
				oplus_imp_node_register(child, chip->dev, chip, oplus_ufcs_get_mos_node_impedance);
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

static void oplus_ufcs_imp_uint_init_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_ufcs *chip =
		container_of(dwork, struct oplus_ufcs, imp_uint_init_work);
	static int retry_count = 0;
#define IMP_UINT_INIT_RETRY_MAX	500

	if (!of_property_read_bool(chip->dev->of_node, "oplus,impedance_unit")) {
		chg_err("oplus,impedance_unit not found\n");
		return;
	}

	chip->imp_uint = of_find_imp_uint(chip->dev->of_node, "oplus,impedance_unit");
	if (chip->imp_uint != NULL) {
		vote(chip->ufcs_not_allow_votable, IMP_VOTER, false, 0, false);
		return;
	}

	retry_count++;
	if (retry_count > IMP_UINT_INIT_RETRY_MAX) {
		chg_err("can't get ufcs imp uint\n");
		return;
	}

	/* Try once every 100ms, accumulatively try 50s */
	schedule_delayed_work(&chip->imp_uint_init_work, msecs_to_jiffies(100));
}

static int oplus_ufcs_dev_open(struct inode *inode, struct file *filp)
{
	struct oplus_ufcs *chip = container_of(filp->private_data,
		struct oplus_ufcs, misc_dev);

	filp->private_data = chip;
	chg_info("ufcs_dev: %d-%d\n", imajor(inode), iminor(inode));
	return 0;
}

static ssize_t oplus_ufcs_dev_read(struct file *filp, char __user *buf,
				   size_t count, loff_t *offset)
{
	struct oplus_ufcs *chip = filp->private_data;
	struct ufcs_dev_cmd cmd;
	int rc;

	mutex_lock(&chip->read_lock);
	rc = wait_event_interruptible(chip->read_wq, chip->cmd_data_ok);
	mutex_unlock(&chip->read_lock);
	if (rc)
		return rc;
	if (!chip->cmd_data_ok)
		chg_err("ufcs false wakeup, rc=%d\n", rc);

	mutex_lock(&chip->cmd_data_lock);
	chip->cmd_data_ok = false;
	memcpy(&cmd, &chip->cmd, sizeof(struct ufcs_dev_cmd));
	mutex_unlock(&chip->cmd_data_lock);
	if (copy_to_user(buf, &cmd, sizeof(struct ufcs_dev_cmd))) {
		chg_err("failed to copy to user space\n");
		return -EFAULT;
	}

	return sizeof(struct ufcs_dev_cmd);
}

#define UFCS_IOC_MAGIC			0x66
#define UFCS_NOTIFY_GET_AUTH_DATA	_IOW(UFCS_IOC_MAGIC, 1, char)

static long oplus_ufcs_dev_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	struct oplus_ufcs *chip = filp->private_data;
	void __user *argp = (void __user *)arg;
	int rc;

	switch (cmd) {
	case UFCS_NOTIFY_GET_AUTH_DATA:
		rc = copy_from_user(&chip->auth_data, argp, UFCS_VERIFY_AUTH_DATA_SIZE);
		if (rc) {
			chg_err("failed copy to user space\n");
			return rc;
		}
		chip->auth_data_ok = true;
		chg_info("auth data ok\n");
		if (delayed_work_pending(&chip->wait_auth_data_work)) {
			cancel_delayed_work_sync(&chip->wait_auth_data_work);
			schedule_delayed_work(&chip->wait_auth_data_work, 0);
		}

		break;
	default:
		chg_err("bad ioctl %u\n", cmd);
		return -EINVAL;
	}

	return 0;
}

static ssize_t oplus_ufcs_dev_write(struct file *filp, const char __user *buf,
				    size_t count, loff_t *offset)
{
	return count;
}

static const struct file_operations oplus_ufcs_dev_fops = {
	.owner			= THIS_MODULE,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
	.llseek			= noop_llseek,
#else
	.llseek			= no_llseek,
#endif
	.write			= oplus_ufcs_dev_write,
	.read			= oplus_ufcs_dev_read,
	.open			= oplus_ufcs_dev_open,
	.unlocked_ioctl		= oplus_ufcs_dev_ioctl,
};

static int oplus_ufcs_misc_dev_reg(struct oplus_ufcs *chip)
{
	int rc;

	mutex_init(&chip->read_lock);
	mutex_init(&chip->cmd_data_lock);
	init_waitqueue_head(&chip->read_wq);
	chip->cmd_data_ok = false;
	init_completion(&chip->cmd_ack);

	chip->misc_dev.minor = MISC_DYNAMIC_MINOR;
	chip->misc_dev.name = "ufcs_dev";
	chip->misc_dev.fops = &oplus_ufcs_dev_fops;
	rc = misc_register(&chip->misc_dev);
	if (rc)
		chg_err("misc_register failed, rc=%d\n", rc);

	return rc;
}

static int oplus_ufcs_event_notifier_call(struct notifier_block *nb, unsigned long val, void *v)
{
	struct oplus_ufcs *chip = container_of(nb, struct oplus_ufcs, nb);

	switch (val) {
	case UFCS_NOTIFY_SOURCE_HW_RESET:
		chg_info("source hard reset\n");
		chip->handshake_ok = false;
		schedule_work(&chip->force_exit_work);
		break;
	case UFCS_NOTIFY_CABLE_HW_RESET:
		chg_info("cable hard reset\n");
		break;
	case UFCS_NOTIFY_POWER_CHANGE:
		chg_info("power changed\n");
		break;
	case UFCS_NOTIFY_EXIT:
		chg_info("exit ufcs mode\n");
		chip->handshake_ok = false;
		schedule_work(&chip->soft_exit_work);
		if (v != NULL && (*(int *)v == UFCS_NOTIFY_EXIT_BY_CRASH)) {
			chip->prot_crash = true;
			schedule_delayed_work(&chip->ufcs_restart_timeout_work,
					msecs_to_jiffies(UFCS_RESTART_TIMEOUT_MS));
		}
		break;
	case UFCS_NOTIFY_TEST_MODE_CHANGED:
		if (!oplus_chg_ufcs_is_test_mode(chip)) {
			chg_info("exit test mode\n");
			schedule_work(&chip->soft_exit_work);
		} else {
			chg_info("enter test mode\n");
		}
		break;
	case UFCS_NOTIFY_VOL_ACC_TEST_MODE_CHANGED:
		chg_info("vol_acc_test_mode=%d\n", oplus_chg_ufcs_is_vol_acc_test_mode(chip));
		break;
	case UFCS_NOTIFY_ERR_FLAG:
		chip->err_flag = *((unsigned int *)v);
		chg_info("err_flag=0x%x\n", chip->err_flag);
		break;
	case UFCS_NOTIFY_FIFO_OVERFLOW:
		schedule_work(&chip->fifo_overflow_push_work);
		break;
	case UFCS_NOTIFY_RESTART:
		chg_info("ufcs restart occurs\n");
		if (v != NULL  && (*(int *)v == UFCS_NOTIFY_RESTART_FROM_CRASH))
			chip->prot_crash = false;
		schedule_work(&chip->ufcs_restart_work);
		break;
	default:
		chg_err("unknown event=%lu\n", val);
		break;
	}

	return NOTIFY_OK;
}

static int oplus_ufcs_notify_reg(struct oplus_ufcs *chip)
{
	int rc;

	chip->nb.notifier_call = oplus_ufcs_event_notifier_call;
	rc = ufcs_reg_event_notifier(&chip->nb);
	if (rc < 0) {
		chg_err("register ufcs event notifier error, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int oplus_plat_ufcs_event_notifier_call(struct notifier_block *nb, unsigned long val, void *v)
{
	struct oplus_ufcs *chip = container_of(nb, struct oplus_ufcs, nb);

	switch (val) {
	case PLAT_UFCS_NOTIFY_EXIT:
		chg_info("exit ufcs\n");
		schedule_work(&chip->plat_notify_exit_work);
		break;
	case UFCS_NOTIFY_EXIT_COMM:
		chg_info("exit ufcs mode\n");
		if (v != NULL && (*(int *)v == UFCS_NOTIFY_FGRESET_EXIT_COMM)) {
			if (chip->ufcs_online &&
			    !is_client_vote_enabled(chip->ufcs_not_allow_votable, BATT_TEMP_VOTER)) {
				vote(chip->ufcs_not_allow_votable, BATT_FG_I2CREST_VOTER, true, 1, false);
			}
		}

		schedule_work(&chip->soft_exit_work);
		if (v != NULL && (*(int *)v == UFCS_NOTIFY_EXIT_BY_CRASH)) {
			chip->prot_crash = true;
			schedule_delayed_work(&chip->ufcs_restart_timeout_work,
					msecs_to_jiffies(UFCS_RESTART_TIMEOUT_MS));
		}
		if (v != NULL && (*(int *)v == UFCS_NOTIFY_UFCS_RESET_NOTIFY)) {
			chip->subsys_reset_ufcs = true;
			schedule_delayed_work(&chip->ufcs_subsys_reset_work,
					msecs_to_jiffies(UFCS_RESET_MS));
		}
		break;
	case UFCS_NOTIFY_RESTART_COMM:
		chg_info("ufcs restart occurs\n");
		if (v != NULL  && (*(int *)v == UFCS_NOTIFY_RESTART_FROM_CRASH)) {
			chip->prot_crash = false;
			schedule_work(&chip->ufcs_restart_work);
		} else {
			if (v != NULL  && (*(int *)v == UFCS_NOTIFY_FGRESET_RESTART_COMM))
				schedule_work(&chip->fgreset_ufcs_retry_work);
		}
		break;
	default:
		chg_err("unknown event=%lu\n", val);
		break;
	}

	return NOTIFY_OK;
}

static int oplus_plat_ufcs_notify_reg(struct oplus_ufcs *chip)
{
	int rc = 0;

	chip->nb.notifier_call = oplus_plat_ufcs_event_notifier_call;
	rc = plat_ufcs_reg_event_notifier(&chip->nb);
	if (rc < 0) {
		chg_err("register plat ufcs event notifier error, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
#include "config/dynamic_cfg/oplus_ufcs_cfg.h"
#endif

#define LCF_NAME_LENGTH 32
static int oplus_ufcs_parse_lcf_strategy_dt(struct oplus_ufcs *chip)
{
	struct device_node *startegy_node;
	int rc = 0;
	int i;
	char lcf_name[LCF_NAME_LENGTH] = {0};
	struct device_node *child_node;

	chip->oplus_lcf_num = 0;
	for_each_child_of_node(chip->dev->of_node, child_node) {
		if (strncmp(child_node->name, "ufcs_oplus_lcf_strategy", strlen("ufcs_oplus_lcf_strategy")) == 0)
			chip->oplus_lcf_num++;
	}
	chip->oplus_lcf_strategy = devm_kzalloc(chip->dev, sizeof(struct oplus_chg_strategy *) * chip->oplus_lcf_num, GFP_KERNEL);
	chip->third_lcf_num = 0;
	for_each_child_of_node(chip->dev->of_node, child_node) {
		if (!child_node)
			continue;
		if (strncmp(child_node->name, "ufcs_third_lcf_strategy", strlen("ufcs_third_lcf_strategy")) == 0)
			chip->third_lcf_num++;
	}
	chip->third_lcf_strategy = devm_kzalloc(chip->dev, sizeof(struct oplus_chg_strategy *) * chip->third_lcf_num, GFP_KERNEL);

	startegy_node = of_get_child_by_name(oplus_get_node_by_type(chip->dev->of_node), "ufcs_oplus_lcf_strategy");
	if (startegy_node == NULL) {
		chg_err("ufcs_oplus_lcf_strategy not found\n");
		rc = -ENODEV;
	} else {
		chip->oplus_lcf_strategy[0] = oplus_chg_strategy_alloc_by_node("low_curr_full_strategy", startegy_node);
		if (IS_ERR_OR_NULL(chip->oplus_lcf_strategy[0])) {
			chg_err("alloc oplus_lcf_strategy error, rc=%ld", PTR_ERR(chip->oplus_lcf_strategy[0]));
			chip->oplus_lcf_strategy[0] = NULL;
			rc = -EFAULT;
		} else {
			chg_info("oplus_lcf_strategy 0 alloc success");
		}
	}
	for (i = 1; i < chip->oplus_lcf_num && rc >= 0; i++) {
		snprintf(lcf_name, sizeof(lcf_name), "ufcs_oplus_lcf_strategy_%d", i);
		startegy_node = of_get_child_by_name(oplus_get_node_by_type(chip->dev->of_node), lcf_name);
		if (startegy_node == NULL) {
			rc = -ENODEV;
			break;
		} else {
			chip->oplus_lcf_strategy[i] = oplus_chg_strategy_alloc_by_node("low_curr_full_strategy", startegy_node);
			if (IS_ERR_OR_NULL(chip->oplus_lcf_strategy[i])) {
				chg_err("alloc oplus_lcf_strategy error, rc=%ld", PTR_ERR(chip->oplus_lcf_strategy[i]));
				chip->oplus_lcf_strategy[i] = NULL;
				rc = -EFAULT;
				break;
			} else {
				chg_info("oplus_lcf_strategy %d alloc success", i);
			}
		}
	}

	rc = 0;
	startegy_node = of_get_child_by_name(oplus_get_node_by_type(chip->dev->of_node), "ufcs_third_lcf_strategy");
	if (startegy_node == NULL) {
		chg_err("ufcs_third_lcf_strategy not found\n");
		rc = -ENODEV;
	} else {
		chip->third_lcf_strategy[0] = oplus_chg_strategy_alloc_by_node("low_curr_full_strategy", startegy_node);
		if (IS_ERR_OR_NULL(chip->third_lcf_strategy[0])) {
			chg_err("alloc third_lcf_strategy error, rc=%ld", PTR_ERR(chip->third_lcf_strategy[0]));
			chip->third_lcf_strategy[0] = NULL;
			rc = -EFAULT;
		} else {
			chg_info("third_lcf_strategy 0 alloc success");
		}
	}
	for (i = 1; i < chip->third_lcf_num && rc >= 0; i++) {
		snprintf(lcf_name, sizeof(lcf_name), "ufcs_third_lcf_strategy_%d", i);
		startegy_node = of_get_child_by_name(oplus_get_node_by_type(chip->dev->of_node), lcf_name);
		if (startegy_node == NULL) {
			rc = -ENODEV;
			break;
		} else {
			chip->third_lcf_strategy[i] = oplus_chg_strategy_alloc_by_node("low_curr_full_strategy", startegy_node);
			if (IS_ERR_OR_NULL(chip->third_lcf_strategy[i])) {
				chg_err("alloc third_lcf_strategy error, rc=%ld", PTR_ERR(chip->third_lcf_strategy[i]));
				chip->third_lcf_strategy[i] = NULL;
				rc = -EFAULT;
				break;
			}  else {
				chg_info("third_lcf_strategy %d alloc success", i);
			}
		}
	}

	chg_info("oplus_lcf_num %d, third_lcf_num %d", chip->oplus_lcf_num, chip->third_lcf_num);

	return rc;
}

#define THIRD_MAX_POWER_DATA_LENGTH 5
static int oplus_ufcs_get_third_max_power(struct oplus_ufcs *chip, struct device_node *node)
{
	int rc;
	struct device_node *soc_node;
	int data[THIRD_MAX_POWER_DATA_LENGTH] = { 0 };

	if (node == NULL) {
		chg_err("node is NULL\n");
		rc = -ENODEV;
		goto exit;
	}

	soc_node = of_get_child_by_name(node, "strategy_soc_range_min");
	if (!soc_node) {
		chg_err("can't find strategy_soc_range_mid node\n");
		rc = -ENODEV;
		goto exit;
	}

	rc = of_property_read_u32_array(
			soc_node, "strategy_temp_normal_high",
			(u32 *)data, THIRD_MAX_POWER_DATA_LENGTH);
	if (rc < 0) {
		chg_err("read strategy_temp_normal_high property error, rc=%d\n", rc);
		goto exit;
	}
	chip->third_curve_target_vbus_mv = data[0];
	chip->third_curve_target_ibus_ma = data[2];

	chg_info("third_curve_target_vbus_mv=%d, third_curve_target_ibus_ma=%d\n",
			chip->third_curve_target_vbus_mv, chip->third_curve_target_ibus_ma);

	return 0;
exit:
	return rc;
}

static void oplus_ufcs_get_temp_region_cnt(struct oplus_ufcs *chip, struct device_node *node)
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

static int oplus_ufcs_probe(struct platform_device *pdev)
{
	struct oplus_ufcs *chip;
	const char *name;
	struct device_node *startegy_node;
	int rc;

	chip = devm_kzalloc(&pdev->dev, sizeof(struct oplus_ufcs), GFP_KERNEL);
	if (chip == NULL) {
		chg_err("alloc oplus_ufcs struct buffer error\n");
		return 0;
	}
	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);
	mutex_init(&chip->ccd_lock);
	chip->reset_adapter = false;
	chip->handshake_ok = false;
	init_completion(&chip->reset_abnormal_ack);

	oplus_ufcs_parse_dt(chip);
	atomic_set(&chip->cp_offline, 0);
	INIT_DELAYED_WORK(&chip->switch_check_work, oplus_ufcs_switch_check_work);
	INIT_DELAYED_WORK(&chip->monitor_work, oplus_ufcs_monitor_work);
	if (chip->pr_config.support_pr)
		INIT_DELAYED_WORK(&chip->current_work, oplus_ufcs_pr_current_work);
	else
		INIT_DELAYED_WORK(&chip->current_work, oplus_ufcs_current_work);
	INIT_DELAYED_WORK(&chip->imp_uint_init_work, oplus_ufcs_imp_uint_init_work);
	INIT_DELAYED_WORK(&chip->wait_auth_data_work, oplus_ufcs_wait_auth_data_work);
	INIT_DELAYED_WORK(&chip->ufcs_restart_timeout_work, oplus_ufcs_restart_timeout_work);
	INIT_DELAYED_WORK(&chip->ufcs_eis_work, oplus_ufcs_eis_work);
	INIT_DELAYED_WORK(&chip->send_authdata_to_adsp_work, oplus_ufcs_send_authdata_to_adsp_work);
	INIT_DELAYED_WORK(&chip->switch_end_recheck_work, oplus_ufcs_switch_end_recheck_work);
	INIT_DELAYED_WORK(&chip->ufcs_subsys_reset_work, oplus_ufcs_subsys_reset_work);
	INIT_DELAYED_WORK(&chip->watchdog_work, oplus_ufcs_watchdog_work);
	INIT_WORK(&chip->wired_online_work, oplus_ufcs_wired_online_work);
	INIT_WORK(&chip->force_exit_work, oplus_ufcs_force_exit_work);
	INIT_WORK(&chip->soft_exit_work, oplus_ufcs_soft_exit_work);
	INIT_WORK(&chip->gauge_update_work, oplus_ufcs_gauge_update_work);
	INIT_WORK(&chip->err_flag_push_work, oplus_ufcs_err_flag_push_work);
	INIT_WORK(&chip->fifo_overflow_push_work, oplus_ufcs_fifo_overflow_push_work);
	INIT_WORK(&chip->plat_notify_exit_work, oplus_ufcs_plat_notify_exit_work);
	INIT_WORK(&chip->ufcs_restart_work, oplus_ufcs_restart_work);
	INIT_WORK(&chip->retention_disconnect_work, oplus_ufcs_retention_disconnect_work);
	INIT_WORK(&chip->handshake_error_work, oplus_ufcs_handshake_error_work);
	INIT_WORK(&chip->fgreset_ufcs_retry_work, oplus_fgreset_ufcs_retry_work);
	INIT_WORK(&chip->cp_err_handler_work, oplus_ufcs_cp_err_handler_work);
	INIT_WORK(&chip->cp_online_handler_work, oplus_ufcs_cp_online_handler_work);
	INIT_WORK(&chip->cp_offline_handler_work, oplus_ufcs_cp_offline_handler_work);
	INIT_WORK(&chip->set_fcs_icl_work, oplus_ufcs_set_fcs_icl_work);

	oplus_ufcs_parse_temperature_strategy_init(chip);
	atomic_set(&chip->temp_recover_debounce_thd, 0);

	rc = oplus_ufcs_init_imp_node(chip);
	if (rc < 0)
		goto imp_node_init_err;

	rc = oplus_ufcs_vote_init(chip);
	if (rc < 0)
		goto vote_init_err;

	rc = oplus_ufcs_misc_dev_reg(chip);
	if (rc < 0)
		goto misc_dev_reg_err;

	if (chip->config.adsp_ufcs_project) {
		rc = oplus_plat_ufcs_notify_reg(chip);
		if (rc < 0) {
			chg_err("register plat ufcs failed.");
			goto nb_reg_err;
		}
	} else {
		rc = oplus_ufcs_notify_reg(chip);
		if (rc < 0)
			goto nb_reg_err;
	}

	startegy_node = of_get_child_by_name(oplus_get_node_by_type(pdev->dev.of_node), "ufcs_charge_third_strategy");
	if (startegy_node == NULL) {
		chg_err("ufcs_charge_third_strategy not found\n");
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
	rc = oplus_ufcs_get_third_max_power(chip, startegy_node);
	if (rc < 0)
		goto third_startegy_err;

	oplus_ufcs_get_temp_region_cnt(chip, startegy_node);
	startegy_node = of_get_child_by_name(oplus_get_node_by_type(pdev->dev.of_node), "ufcs_charge_oplus_strategy");
	if (startegy_node == NULL) {
		chg_err("ufcs_charge_oplus_strategy not found\n");
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

	oplus_ufcs_parse_lcf_strategy_dt(chip);

	oplus_ufcs_parse_vfa_strategy(chip);

	oplus_ufcs_parse_ccd_strategy(chip);

	/* Encryption is not required when the maximum current is less than 3A */
	if (chip->config.curr_max_ma <= UFCS_VERIFY_CURR_THR_MA) {
		chg_info("ignore encrypted data\n");
		chip->wait_auth_data_done = true;
		chip->auth_data_ok = true;
		chip->ufcs_dispatch_key_ok = true;
	} else {
		chip->wait_auth_data_done = false;
		chip->auth_data_ok = false;
		chip->ufcs_dispatch_key_ok = false;
	}

	name = of_get_oplus_chg_ic_name(pdev->dev.of_node, "oplus,cp_ic", 0);
	oplus_chg_ic_wait_ic(name, oplus_ufcs_cp_ic_reg_callback, chip);
	if (of_property_read_bool(chip->dev->of_node, "oplus,impedance_unit")) {
		vote(chip->ufcs_not_allow_votable, IMP_VOTER, true, 1, false);
		schedule_delayed_work(&chip->imp_uint_init_work, 0);
	} else {
		chg_err("oplus,impedance_unit not found\n");
	}

	rc = oplus_ufcs_get_verify_data(chip, 1);
	if (rc < 0)
		chg_err("get verify auth data fail, rc=%d\n", rc);
	schedule_delayed_work(&chip->wait_auth_data_work, msecs_to_jiffies(UFCS_GET_AUTH_DATA_TIMEOUT_MS));

#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
	(void)oplus_ufcs_reg_debug_config(chip);
#endif

	return 0;

oplus_startegy_err:
	if (chip->ccd_strategy != NULL) {
		oplus_chg_strategy_release(chip->ccd_strategy);
		chip->ccd_strategy = NULL;
	}
	if (chip->third_curve_strategy != NULL)
		oplus_chg_strategy_release(chip->third_curve_strategy);
third_startegy_err:
	if (chip->config.adsp_ufcs_project)
		plat_ufcs_unreg_event_notifier(&chip->nb);
	else
		ufcs_unreg_event_notifier(&chip->nb);
nb_reg_err:
	misc_deregister(&chip->misc_dev);
misc_dev_reg_err:
	if (chip->ufcs_boot_votable != NULL)
		destroy_votable(chip->ufcs_boot_votable);
	if (chip->ufcs_not_allow_votable != NULL)
		destroy_votable(chip->ufcs_not_allow_votable);
	if (chip->ufcs_disable_votable != NULL)
		destroy_votable(chip->ufcs_disable_votable);
	if (chip->ufcs_curr_votable != NULL)
		destroy_votable(chip->ufcs_curr_votable);
vote_init_err:
	if (chip->input_imp_node != NULL)
		oplus_imp_node_unregister(chip->dev, chip->input_imp_node);
imp_node_init_err:
	if (chip->temperature_strategy)
		oplus_chg_strategy_release(chip->temperature_strategy);
	mutex_destroy(&chip->ccd_lock);
	devm_kfree(&pdev->dev, chip);
	return rc;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void oplus_ufcs_remove(struct platform_device *pdev)
#else
static int oplus_ufcs_remove(struct platform_device *pdev)
#endif
{
	struct oplus_ufcs *chip = platform_get_drvdata(pdev);
	int i;

#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
	oplus_ufcs_unreg_debug_config(chip);
#endif

	if (chip->ufcs_ic)
		oplus_ufcs_virq_unreg(chip);
	if (chip->dpdm_switch)
		oplus_ufcs_dpdm_switch_virq_unreg(chip);
	if (chip->cp_ic)
		oplus_ufcs_cp_virq_unreg(chip);
	oplus_plc_release_protocol(chip->plc_topic, chip->opp);
	if (!IS_ERR_OR_NULL(chip->cpa_subs))
		oplus_mms_unsubscribe(chip->cpa_subs);
	if (!IS_ERR_OR_NULL(chip->wired_subs))
		oplus_mms_unsubscribe(chip->wired_subs);
	if (!IS_ERR_OR_NULL(chip->comm_subs))
		oplus_mms_unsubscribe(chip->comm_subs);
	if (!IS_ERR_OR_NULL(chip->gauge_subs))
		oplus_mms_unsubscribe(chip->gauge_subs);
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
	for (i = 0; i < chip->oplus_lcf_num; i++) {
		if (chip->oplus_lcf_strategy[i] != NULL)
			oplus_chg_strategy_release(chip->oplus_lcf_strategy[i]);
	}
	for (i = 0; i < chip->third_lcf_num; i++) {
		if (chip->third_lcf_strategy[i] != NULL)
			oplus_chg_strategy_release(chip->third_lcf_strategy[i]);
	}
	devm_kfree(chip->dev, chip->oplus_lcf_strategy);
	devm_kfree(chip->dev, chip->third_lcf_strategy);
	if (chip->config.adsp_ufcs_project)
		plat_ufcs_unreg_event_notifier(&chip->nb);
	else
		ufcs_unreg_event_notifier(&chip->nb);
	misc_deregister(&chip->misc_dev);
	if (chip->ufcs_boot_votable != NULL)
		destroy_votable(chip->ufcs_boot_votable);
	if (chip->ufcs_not_allow_votable != NULL)
		destroy_votable(chip->ufcs_not_allow_votable);
	if (chip->ufcs_disable_votable != NULL)
		destroy_votable(chip->ufcs_disable_votable);
	if (chip->ufcs_curr_votable != NULL)
		destroy_votable(chip->ufcs_curr_votable);
	if (chip->input_imp_node != NULL)
		oplus_imp_node_unregister(chip->dev, chip->input_imp_node);
	if (chip->temperature_strategy)
		oplus_chg_strategy_release(chip->temperature_strategy);
	mutex_destroy(&chip->ccd_lock);
	devm_kfree(&pdev->dev, chip);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return 0;
#endif
}

static const struct of_device_id oplus_ufcs_match[] = {
	{ .compatible = "oplus,ufcs_charge" },
	{},
};

static struct platform_driver oplus_ufcs_driver = {
	.driver		= {
		.name = "oplus-ufcs_charge",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(oplus_ufcs_match),
	},
	.probe		= oplus_ufcs_probe,
	.remove		= oplus_ufcs_remove,
};

static __init int oplus_ufcs_init(void)
{
	return platform_driver_register(&oplus_ufcs_driver);
}

static __exit void oplus_ufcs_exit(void)
{
	platform_driver_unregister(&oplus_ufcs_driver);
}

oplus_chg_module_register(oplus_ufcs);

int oplus_ufcs_current_to_level(struct oplus_mms *mms, int ibus_curr)
{
	int level = 0;
	struct oplus_ufcs *chip;
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

	if (chip->curr_table_type == UFCS_CURR_CP_TABLE)
		level = ufcs_find_level_to_current(ibat_curr, g_ufcs_cp_current_table, ARRAY_SIZE(g_ufcs_cp_current_table));
	else
		level = ufcs_find_level_to_current(ibus_curr, g_ufcs_current_table, ARRAY_SIZE(g_ufcs_current_table));

	return level;
}

enum fastchg_protocol_type oplus_ufcs_adapter_id_to_protocol_type(u32 id)
{
	switch (id) {
	case UFCS_FASTCHG_TYPE_V1:
		return PROTOCOL_CHARGING_UFCS_OPLUS;
	case UFCS_FASTCHG_TYPE_THIRD:
		return PROTOCOL_CHARGING_UFCS_THIRD;
	default:
		return PROTOCOL_CHARGING_UFCS_OPLUS;
	}
}

static enum ufcs_power_type oplus_ufcs_imax_covert_power(enum ufcs_power_imax curr, int vbus)
{
	int i = 0, asize = 0;
	enum ufcs_power_type power_type = UFCS_POWER_TYPE_V9;

	asize = sizeof(ufcs_power_table) / sizeof(struct ufcs_power_param);
	for (i = 0; i < asize; i++) {
		if (vbus == ufcs_power_table[i].vmax && curr <= ufcs_power_table[i].imax) {
			power_type = ufcs_power_table[i].power;
			break;
		}
	}

	return power_type;
}

int oplus_ufcs_get_ufcs_power(struct oplus_mms *mms)
{
	int power_result, cpa_power;
	struct oplus_ufcs *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return UFCS_POWER_TYPE_UNKOWN;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (IS_ERR_OR_NULL(chip->cpa_topic))
		return UFCS_POWER_TYPE_UNKOWN;
	if (!chip->oplus_ufcs_adapter)
		return oplus_cpa_protocol_get_power(chip->cpa_topic, CHG_PROTOCOL_UFCS) / 1000;

	cpa_power = oplus_cpa_protocol_get_max_power_by_type(chip->cpa_topic, CHG_PROTOCOL_UFCS) / 1000;
	power_result = oplus_ufcs_imax_covert_power(min(chip->emark_imax, chip->power_imax), chip->config.target_vbus_mv);
	if (cpa_power && power_result > cpa_power)
		power_result = cpa_power;

	return power_result;
}

