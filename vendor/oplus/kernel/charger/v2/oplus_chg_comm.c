#define pr_fmt(fmt) "[CHG_COMM]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of_platform.h>
#include <linux/iio/consumer.h>
#include <uapi/linux/sched/types.h>
#include <linux/thermal.h>
#include <linux/proc_fs.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/power_supply.h>
#include <linux/rtc.h>
#include <linux/reboot.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/random.h>
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/system/boot_mode.h>
#include <soc/oplus/system/oplus_project.h>
#endif
#if IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)
#include <linux/msm_drm_notify.h>
#endif
#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_OPLUS_CHG_DRM_PANEL_NOTIFY)
#include <linux/soc/qcom/panel_event_notifier.h>
#include <drm/drm_panel.h>
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
#if IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
#include <linux/mtk_panel_ext.h>
#include <linux/mtk_disp_notify.h>
#endif
#else
#if IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY_CHG)
#include <mtk_panel_ext.h>
#include <mtk_disp_notify.h>
#endif
#endif


#ifdef CONFIG_OPLUS_CHARGER_MTK
#include <mt-plat/mtk_boot_common.h>
#endif
#include <oplus_chg_module.h>
#include <oplus_chg.h>
#include <oplus_chg_voter.h>
#include <oplus_chg_comm.h>
#include <oplus_mms.h>
#include <oplus_mms_wired.h>
#include <oplus_chg_wls.h>
#include <oplus_mms_gauge.h>
#include <oplus_chg_vooc.h>
#include <oplus_chg_monitor.h>
#include <oplus_chg_ufcs.h>
#include <oplus_chg_pps.h>
#include <oplus_chg_plc.h>
#include "monitor/oplus_chg_track.h"
#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
#include "oplus_cfg.h"
#endif
#include <linux/mutex.h>
#include <oplus_parallel.h>
#include <linux/ktime.h>
#include <linux/sched/clock.h>
#include <oplus_chg_state_retention.h>
#include <oplus_chg_mutual.h>
#include <oplus_chg_cpa.h>
#include <recovery/state_keep.h>
#include <oplus_reverse_chg.h>

#define FULL_COUNTS_SW		5
#define FULL_COUNTS_HW		4
#define VFLOAT_OVER_NUM		2
#define RECHG_COUNT_MAX		5
#define FFC_START_DELAY		msecs_to_jiffies(15000)
#define TEN_MINUTES		600
#define ONE_MINUTE 		60
#define MAX_UI_DECIMAL_TIME	16
#define UPDATE_TIME		1
#define PLUGOUT_SOC_THRESHOLD	95
#define NORMAL_FULL_SOC		100
#define VBAT_GAP_CHECK_CNT	3
#define VBAT_MAX_GAP		50
#define TEMP_BATTERY_STATUS__REMOVED 190
#define COUNT_TIMELIMIT		4
#define MIN_DELTA_SOC		1
#define MAX_DELTA_SOC		10
#define LOW_VOLT_UISOC_LADDER_NUMBER	(10)
#define ALLOW_UISOC_DOWN_CURRENT	(-50)
#define DEC_VOL_CC_THR_COUNT		3
#define VBAT_COLD_WARM_COMP 		10
#define DEC_VOL_CC_FULL_THR_COUNT	6
#define FLASH_MODE_DELAY		10000
#define FLASH_MODE_SAFETY_VOLTAGE	5400
#define FLASH_MODE_SAFETY_VOLTAGE_DETECT_COUNT		25
#define FLASH_MODE_SAFETY_VOLTAGE_QUERY_INTERVAL	40
#define SOC_DOWN_DELAY_FOR_REVERSE_CHARGING		30

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 17, 0))
#define pde_data(inode) PDE_DATA(inode)
#endif

static struct oplus_chg_comm *g_comm_dev;

static int oplus_dbg_tbat = 0;
module_param(oplus_dbg_tbat, int, 0644);
MODULE_PARM_DESC(oplus_dbg_tbat, "oplus debug battery temperature");

static int oplus_dbg_timeout = 0;
module_param(oplus_dbg_timeout, int, 0644);
MODULE_PARM_DESC(oplus_dbg_timeout, "oplus debug chg timeout");

static int oplus_gauge_r_data = 0;
module_param(oplus_gauge_r_data, int, 0644);
MODULE_PARM_DESC(oplus_gauge_r_data, "oplus gauge r data flag");

#define DEC_CV_DOWN_TAG_LEN 14
struct dec_cv_down_load {
	char tag_info[DEC_CV_DOWN_TAG_LEN];
	int dec_cc;
	int dec_vol;
	int dec_vct;
};

struct dec_cv_full_data {
	bool del_status;
	bool dec_init;
	bool vct_en;
	bool aging_soh;
	bool vct_boot_flag;
	int spec_step;
	int pack_type;
	int vct_cur;
	int last_cc;
	int vct_up;
	int soh_fact;
	int dec_soh;
	int spec_cc_thr[DEC_VOL_CC_FULL_THR_COUNT];
	int spec_fv_mv[DEC_VOL_CC_FULL_THR_COUNT];
	int spec_vct_mv[DEC_VOL_CC_FULL_THR_COUNT];
};

struct dec_cv_lite_data {
	int32_t dec_vol_cc_thr[DEC_VOL_CC_THR_COUNT];
	int32_t dec_vol_fv_mv[DEC_VOL_CC_THR_COUNT][TEMP_REGION_MAX];
};

struct dec_cv_data {
	bool dec_track;
	int dec_spec_support;
	int index;
	int dec_vol;
	int dec_delta;
	int dbg_soh;
	struct dec_cv_lite_data lite;
	struct dec_cv_full_data full;
};

enum dec_cv_support_type {
	DEC_CV_SUPPORT_NOT,
	DEC_CV_SUPPORT_LITE,
	DEC_CV_SUPPORT_FULL,
	DEC_CV_SUPPORT_MAX,
};

enum bdd_voltdiff_trend {
	BDD_VOLT_DIFF_TREND_NONE,
	BDD_VOLT_DIFF_TREND_START,
	BDD_VOLT_DIFF_TREND_CONFIRM,
	BDD_VOLT_DIFF_TREND_TIMEOUT,
};

struct oplus_comm_spec_config {
	int32_t temp_region_max;
	int32_t ffc_temp_region_max;
	int32_t batt_temp_thr[TEMP_REGION_MAX - 1];
	int32_t iterm_ma;
	int32_t sub_iterm_ma;
	struct dec_cv_data dec_cv;
	int32_t fv_mv[TEMP_REGION_MAX];
	int32_t sw_fv_mv[TEMP_REGION_MAX];
	int32_t hw_fv_inc_mv[TEMP_REGION_MAX];
	int32_t sw_over_fv_mv[TEMP_REGION_MAX];
	int32_t removed_bat_decidegc;
	int32_t sw_over_fv_dec_mv;
	int32_t non_standard_sw_fv_mv;
	int32_t non_standard_fv_mv;
	int32_t non_standard_hw_fv_inc_mv;
	int32_t non_standard_sw_over_fv_mv;
	int32_t ffc_temp_thr[FFC_TEMP_REGION_MAX - 1];
	int32_t wired_ffc_step_max;
	int32_t wired_ffc_fv_mv[FFC_CHG_STEP_MAX];
	int32_t wired_ffc_fv_cutoff_mv[FFC_CHG_STEP_MAX]
				       [FFC_TEMP_REGION_MAX - 2];
	int32_t wired_ffc_fcc_ma[FFC_CHG_STEP_MAX][FFC_TEMP_REGION_MAX - 2];
	int32_t wired_ffc_fcc_cutoff_ma[FFC_CHG_STEP_MAX]
				       [FFC_TEMP_REGION_MAX - 2];
	int32_t wired_ffc_fcc_sub_cutoff_ma[FFC_CHG_STEP_MAX]
				       [FFC_TEMP_REGION_MAX - 2];
	int32_t wired_aging_ffc_version;
	int32_t wired_aging_ffc_offset_mv[FFC_CHG_STEP_MAX][AGAIN_FFC_CYCLY_THR_COUNT];
	int32_t wired_aging_ffc_cycle_thr[AGAIN_FFC_CYCLY_THR_COUNT];
	int32_t wls_ffc_step_max;
	int32_t wls_ffc_fv_mv[FFC_CHG_STEP_MAX];
	int32_t wls_ffc_fv_cutoff_mv[FFC_CHG_STEP_MAX][FFC_TEMP_REGION_MAX - 2];
	int32_t wls_ffc_icl_ma[FFC_CHG_STEP_MAX][FFC_TEMP_REGION_MAX - 2];
	int32_t wls_ffc_fcc_ma[FFC_CHG_STEP_MAX][FFC_TEMP_REGION_MAX - 2];
	int32_t wls_ffc_fcc_cutoff_ma[FFC_CHG_STEP_MAX][FFC_TEMP_REGION_MAX - 2];
	int32_t wls_ffc_fcc_sub_cutoff_ma[FFC_CHG_STEP_MAX][FFC_TEMP_REGION_MAX - 2];
	int32_t wired_vbatdet_mv[TEMP_REGION_MAX];
	int32_t wls_vbatdet_mv[TEMP_REGION_MAX];
	int32_t non_standard_vbatdet_mv;

	int32_t fcc_gear_cnt;
	int32_t fcc_gear_thr_mv[FCC_GEAR_MAX - 1][TEMP_REGION_MAX];
	int32_t fcc_gear_shake_mv[FCC_GEAR_MAX - 1][TEMP_REGION_MAX];

	int32_t full_pre_ffc_mv;
	bool full_pre_ffc_judge;
	int32_t bdd_vbatdiff_mv;
	int32_t bdd_enter_current_ma;
	int32_t bdd_enter_voltage_mv;
	int32_t bdd_vbatdiff_timeout_min;

	int32_t vbatt_ov_thr_mv;
	int32_t max_chg_time_sec;
	int32_t vbat_uv_thr_mv;
	int32_t vbat_charging_uv_thr_mv;
	int32_t vbat_charging_uv_delata_mv;
	int32_t tbatt_power_off_cali_temp;
	int32_t gauge_stuck_threshold;
	int32_t gauge_stuck_time;
	int32_t poweroff_high_batt_temp;
	int32_t poweroff_emergency_batt_temp;
	int32_t sub_vbat_uv_thr_mv;
	int32_t sub_vbat_charging_uv_thr_mv;
	/*x24 sw full configuration silicon battery need to be configured in dts 12*/
	int32_t sw_check_full_cnt;
	bool support_hot_enter_kpoc;
	int32_t batt_full_time;
	int32_t batt_full_low_soc;
	int32_t batt_full_low_soc_time;
} __attribute__ ((packed));

struct oplus_comm_config {
	uint32_t ui_soc_decimal_speedmin;
	uint8_t vooc_show_ui_soc_decimal;
	uint8_t vooc_dis_show_ui_power;
	uint8_t gauge_stuck_jump_support;
	uint8_t smooth_switch;
	uint32_t reserve_soc;
	int32_t ui_soc_2_voltage_comp_mv;
	int32_t ui_soc_3_voltage_comp_mv;

	uint8_t *smooth_strategy_name;

	int32_t temp_ladder_of_drop_soc_2[LOW_VOLT_UISOC_LADDER_NUMBER];
	int32_t volt_diff_ladder_of_drop_soc_2[LOW_VOLT_UISOC_LADDER_NUMBER];
	int32_t temp_ladder_of_keep_soc_2[LOW_VOLT_UISOC_LADDER_NUMBER];
	int32_t volt_diff_ladder_of_keep_soc_2[LOW_VOLT_UISOC_LADDER_NUMBER];
	int32_t temp_ladder_of_keep_soc_3[LOW_VOLT_UISOC_LADDER_NUMBER];
	int32_t volt_diff_ladder_of_keep_soc_3[LOW_VOLT_UISOC_LADDER_NUMBER];
	int32_t soc_down_ladder[LOW_VOLT_UISOC_LADDER_NUMBER];
	int32_t soc_down_delay_hz_ladder[LOW_VOLT_UISOC_LADDER_NUMBER];
	int32_t soc_down_chg_delay_hz_ladder[LOW_VOLT_UISOC_LADDER_NUMBER];
	int32_t soc_down_default_delay_hz;
	int32_t back_rm_of_drop_soc_2;
	int32_t back_rm_of_drop_soc_1;
	int32_t load_current_of_drop_soc_2;
	int32_t load_current_of_drop_soc_1;
	int32_t current_limit_of_drop_soc_2;
	int32_t volt_of_fast_drop_soc_1;
	bool support_uisoc_low_battery_control;

	int32_t chg_shutdown_max_mv;
	uint8_t hidden_soc_switch;
	uint32_t hidden_soc_percent;
	bool dual_panel_support;
	bool not_pop_up;
	bool support_gauge_r_track;
	bool support_hw_iterm_check;
	bool support_boost_ic;
} __attribute__ ((packed));

struct uv_dec_speed_entry {
	s32 temp;
	u32 speed;
};

#define UV_DEC_SPEED_ENTRY_U32_CNT	(sizeof(struct uv_dec_speed_entry) / sizeof(u32))

struct ui_soc_decimal {
	int ui_soc_decimal;
	int ui_soc_integer;
	int last_decimal_ui_soc;
	int init_decimal_ui_soc;
	int calculate_decimal_time;
	bool boot_completed;
	bool decimal_control;
	int eis_dummy_cnt;
};

#define UV_ADJUSTED_LOW_THR		400
#define UV_ADJUSTED_HIGH_THR	1000
#define UV_ADJUSTED_VOLT_THR	50
#define UV_ADJUSTED_CHECK_TIME	60
#define UV_ADJUSTED_CHECK_SOC	20
#define SMOOTH_SOC_MAX_FIFO_LEN 	4
#define SMOOTH_SOC_MIN_FIFO_LEN 	1
#define RESERVE_SOC_MIN 		1
#define RESERVE_SOC_DEFAULT		3
#define RESERVE_SOC_MAX 		5
#define RESERVE_SOC_OFF 		0
#define OPLUS_FULL_SOC			100
#define SOC_JUMP_RANGE_VAL		1
#define PARTITION_UISOC_GAP		10
#define POWER_OFF_SOC			0
#define HIDDEN_SOC_PERCENT_MAX		100
#define HIDDEN_SOC_PERCENT_MIN		20
#define HIDDEN_SOC_PERCENT_DEFAULT	20
struct reserve_soc_data {
	int rus_reserve_soc;
	int smooth_soc_fifo[SMOOTH_SOC_MAX_FIFO_LEN];
	int smooth_soc_index;
	int smooth_soc_avg_cnt;
	bool is_soc_jump_range;
};

struct oplus_chg_comm {
	struct device *dev;
	struct oplus_mms *wired_topic;
	struct oplus_mms *gauge_topic;
	struct oplus_mms *main_gauge_topic;
	struct oplus_mms *sub_gauge_topic;
	struct oplus_mms *parallel_topic;
	struct oplus_mms *comm_topic;
	struct oplus_mms *vooc_topic;
	struct oplus_mms *wls_topic;
	struct oplus_mms *ufcs_topic;
	struct oplus_mms *pps_topic;
	struct oplus_mms *err_topic;
	struct oplus_mms *retention_topic;
	struct oplus_mms *plc_topic;
	struct oplus_mms *cpa_topic;
	struct oplus_mms *keep_topic;
	struct oplus_mms *reverse_topic;
	struct mms_subscribe *gauge_subs;
	struct mms_subscribe *wired_subs;
	struct mms_subscribe *vooc_subs;
	struct mms_subscribe *wls_subs;
	struct mms_subscribe *ufcs_subs;
	struct mms_subscribe *pps_subs;
	struct mms_subscribe *comm_subs;
	struct mms_subscribe *retention_subs;
	struct mms_subscribe *plc_subs;
	struct mms_subscribe *keep_subs;
	struct mms_subscribe *reverse_subs;

	spinlock_t remuse_lock;

	struct oplus_comm_spec_config spec;
	struct oplus_comm_config config;
	int uv_dec_speed_tbl_cnt;
	struct uv_dec_speed_entry *uv_dec_speed_tbl;
	struct ui_soc_decimal soc_decimal;
	struct reserve_soc_data rsd;

	struct work_struct gauge_check_work;
	struct work_struct plugin_work;
	struct work_struct chg_type_change_work;
	struct work_struct chg_power_role_change_work;
	struct work_struct gauge_remuse_work;
	struct work_struct noplug_batt_volt_work;
	struct work_struct wired_chg_check_work;
	struct work_struct led_on_off_handler_work;
	struct work_struct offline_delayed_process_work;

	struct delayed_work ffc_start_work;
	struct delayed_work charge_timeout_work;
	struct delayed_work ui_soc_update_work;
	struct delayed_work ui_soc_decimal_work;
	struct delayed_work lcd_notify_reg_work;
	struct delayed_work fg_soft_reset_work;
	struct delayed_work iterm_check_work;
	struct delayed_work passed_q_save_work;
#ifdef CONFIG_OPLUS_CHARGER_MTK
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
	struct delayed_work power_off_check_work;
#endif
#endif

	struct votable *fv_max_votable;
	struct votable *fv_min_votable;
	struct votable *cool_down_votable;
	struct votable *chg_disable_votable;
	struct votable *chg_suspend_votable;
	struct votable *wired_charge_suspend_votable;
	struct votable *wired_icl_votable;
	struct votable *wired_fcc_votable;
	struct votable *wired_charging_disable_votable;
	struct votable *wls_charge_suspend_votable;
	struct votable *wls_icl_votable;
	struct votable *wls_fcc_votable;
	struct votable *wls_charging_disable_votable;
	struct votable *gauge_term_voltage_votable;
	struct votable *wls_fastchg_disable_votable;
	struct votable *wls_comu_votable;
	struct votable *vooc_curr_votable;
	struct votable *ufcs_curr_votable;
	struct votable *flash_mode_votable;

	struct thermal_zone_device *shell_themal;
#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_OPLUS_CHG_DRM_PANEL_NOTIFY)
	struct drm_panel *active_panel;
	struct drm_panel *active_panel_sec;

	void *notifier_cookie;
	void *notifier_cookie_sec;
#else
	struct notifier_block chg_fb_notify;
#endif /* CONFIG_DRM_PANEL_NOTIFY */
	struct task_struct *tbatt_pwroff_task;

	int batt_temp_dynamic_thr[TEMP_REGION_MAX - 1];
	int ffc_temp_dynamic_thr[FFC_TEMP_REGION_MAX - 1];
	enum oplus_temp_region temp_region;
	enum oplus_ffc_temp_region ffc_temp_region;

	bool wired_online;
	bool keep_wired_online;
	bool wls_online;
	bool sw_full;
	bool hw_full_by_sw;
	bool sw_sub_batt_full;
	bool hw_sub_batt_full_by_sw;
	bool batt_full;
	bool sub_batt_full;
	bool batt_cv_full;
	bool authenticate;
	bool hmac;
	bool gauge_remuse;
	bool comm_remuse;
	bool fv_over;
	bool retention_state;
	bool real_chg_full;

	bool batt_exist;
	int vbat_mv;
	int vbat_mv_max;
	int vbat_min_mv;
	int batt_temp;
	int ibat_ma;
	int soc;
	int main_vbat_mv;
	int main_batt_temp;
	int main_ibat_ma;
	int main_soc;
	int sub_vbat_mv;
	int sub_batt_temp;
	int sub_ibat_ma;
	int sub_soc;
	int batt_rm;
	int batt_fcc;
	int batt_cc;
	int deep_support;
	enum oplus_fcc_gear fcc_gear;
	int sw_full_count;
	int hw_full_count;
	int sw_sub_batt_full_count;
	int hw_sub_batt_full_count;
	int fv_over_count;
	int rechg_count;
	bool rechging;
	bool rechg_soc_en;
	bool uisoc_down_in_full;
	bool rechg_now;
	int rechg_soc;
	int batt_status;
	int batt_health;
	int batt_chg_type;
	int shutdown_soc;
	int partition_uisoc;
	bool need_start_timeout_work;
	enum power_role_type power_role;
	enum oplus_wired_cc_detect_status cc_detect_status;
	bool high_reverse_charging;

	unsigned int wired_err_code;
	unsigned int wls_err_code;
	unsigned int gauge_err_code;

	/* ffc */
	bool ffc_charging;
	int ffc_step;
	int ffc_fcc_count;
	int ffc_fv_count;
	enum oplus_chg_ffc_status ffc_status;

	bool charging_disable;
	bool charge_suspend;
	int cool_down;
	int cool_down_sale_mode;
	int ui_soc;
	int smooth_soc;
	int delta_soc;
	int uisoc_keep_2_err;
	int uisoc_keep_3_err;
	int shell_temp;
	unsigned long soc_up_update_jiffies;
	unsigned long soc_down_update_jiffies;
	unsigned long vbat_uv_jiffies;
	unsigned long batt_full_jiffies;
	unsigned long sleep_tm_sec;
	unsigned long save_sleep_tm_sec;
	unsigned long low_temp_check_jiffies;

	int passed_q;
	int passed_q_last;
	unsigned long passed_q_save_time;

	bool chg_full;
	bool vbatt_over;
	bool chging_over_time;
	bool led_on;
	int factory_test_mode;
	unsigned int notify_code;
	unsigned int notify_flag;
	unsigned int vooc_sid;
	bool vooc_by_normal_path;

	bool vooc_charging;
	bool vooc_online;
	bool vooc_online_keep;

	struct mutex decimal_lock;
	bool ufcs_online;
	bool ufcs_charging;
	bool pps_online;
	bool pps_charging;

	bool unwakelock_chg;
	bool chg_powersave;
	bool lcd_notify_reg;
	bool fg_soft_reset_done;
	bool gauge_stuck;
	bool gauge_soc_jump;
	int fg_soft_reset_fail_cnt;
	int fg_check_ibat_cnt;
	int bms_heat_temp_compensation;
	bool anti_expansion_warning;
	bool anti_expansion_error;
	int chg_cycle_status;
	unsigned int cv_cutoff_volt_curr;
	unsigned int ffc_cutoff_curr;
	int bdd_voltdiff_trend;

	struct oplus_chg_strategy *smooth_strategy;
	int smooth_strategy_chg_reserve;

	int slow_chg_param;
	struct mutex slow_chg_mutex;
	struct mutex sale_mode_mutex;

#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
	struct oplus_cfg spec_debug_cfg;
	struct oplus_cfg normal_debug_cfg;
#endif

	unsigned int nvid_support_flags;
	int plc_status;
	bool use_pm_power_off_with_hightemp;
	bool dec_cv_down_init;
	struct mutex dec_data_lock;
	struct dec_cv_down_load dec_cv_down_info;
	struct oplus_chg_mutual_notifier dec_cv_down_obtain_nb;
	struct oplus_chg_mutual_notifier dec_cv_down_update_nb;
	struct delayed_work get_reserve_dec_cv_down_info_work;
	struct delayed_work dec_vol_info_trigger_work;
	struct work_struct set_reserve_dec_cv_down_info_work;
	struct delayed_work dec_cv_check_work;
	struct delayed_work gauge_r_info_work;
	int flash_mode;
	struct delayed_work flash_mode_boost_work;
	struct delayed_work offline_clean_work;
	const int (*ui_soc_smooth_table)[RESERVE_SOC_MAX];
};

typedef struct {
	int charge_limit_enable;
	int charge_limit_value;
	int is_force_set_charge_limit;
	int charge_limit_recharge_value;
	int callname;
}chg_up_limit_info;
static chg_up_limit_info chg_up_limit_data;

static struct oplus_comm_spec_config default_spec = {
	.batt_temp_thr = {
		-100, -50, 0, 50, 120, 160, 350, 450, 530
	},
	.fcc_gear_cnt = 2,
	.fcc_gear_thr_mv = {
		[FCC_GEAR_LOW] = { 4180, 4180, 4180, 4180, 4180, 4180, 4180, 4180, 4180, 4180 },
		[FCC_GEAR_LV1] = { 0 },
		[FCC_GEAR_LV2] = { 0 },
		[FCC_GEAR_LV3] = { 0 },
	},
	.fcc_gear_shake_mv = {
		[FCC_GEAR_LOW] = { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 },
		[FCC_GEAR_LV1] = { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 },
		[FCC_GEAR_LV2] = { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 },
		[FCC_GEAR_LV3] = { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 },
	},
};
static int tbatt_pwroff_enable = 1;
static int noplug_temperature;
static int noplug_batt_volt_max;
static int noplug_batt_volt_min;
static bool g_ui_soc_ready;
static void oplus_comm_set_batt_full(struct oplus_chg_comm *chip, bool full);
static void oplus_comm_set_sub_batt_full(struct oplus_chg_comm *chip, bool full);
static void oplus_comm_fginfo_reset(struct oplus_chg_comm *chip);
static void oplus_comm_set_chg_cycle_status(struct oplus_chg_comm *chip, int status);
static bool oplus_comm_is_not_charging(struct oplus_chg_comm *chip);
static bool oplus_comm_is_discharging(struct oplus_chg_comm *chip);
static void oplus_comm_update_dec_cv_down_info(struct oplus_chg_comm *chip);
static int oplus_comm_get_current_time(unsigned long *now_tm_sec);
static bool g_boot_completed;

static bool fg_reset_test = false;
module_param(fg_reset_test, bool, 0644);
MODULE_PARM_DESC(fg_reset_test, "zy0603 fg reset test");


static const char *const oplus_comm_temp_region_text[] = {
	[TEMP_REGION_COLD] = "cold",
	[TEMP_REGION_LITTLE_COLD] = "little-cold",
	[TEMP_REGION_LITTLE_COLD_HIGH] ="little-cold-high",
	[TEMP_REGION_COOL] = "cool",
	[TEMP_REGION_LITTLE_COOL] = "little-cool",
	[TEMP_REGION_PRE_NORMAL] = "pre-normal",
	[TEMP_REGION_NORMAL] = "normal",
	[TEMP_REGION_NORMAL_HIGH] = "normal-high",
	[TEMP_REGION_WARM] = "warm",
	[TEMP_REGION_HOT] = "hot",
	[TEMP_REGION_MAX] = "invalid",
};

static const char *const oplus_comm_ffc_temp_region_text[] = {
	[FFC_TEMP_REGION_COOL] = "cool",
	[FFC_TEMP_REGION_PRE_NORMAL] = "pre-normal",
	[FFC_TEMP_REGION_PRE_NORMAL_HIGH] = "pre-normal-high",
	[FFC_TEMP_REGION_NORMAL] = "normal",
	[FFC_TEMP_REGION_WARM] = "warm",
	[FFC_TEMP_REGION_MAX] = "invalid",
};

static const char * const POWER_SUPPLY_STATUS_TEXT[] = {
	[POWER_SUPPLY_STATUS_UNKNOWN]		= "Unknown",
	[POWER_SUPPLY_STATUS_CHARGING]		= "Charging",
	[POWER_SUPPLY_STATUS_DISCHARGING]	= "Discharging",
	[POWER_SUPPLY_STATUS_NOT_CHARGING]	= "Not charging",
	[POWER_SUPPLY_STATUS_FULL]		= "Full",
};

static const char * const POWER_SUPPLY_CHARGE_TYPE_TEXT[] = {
	[POWER_SUPPLY_CHARGE_TYPE_UNKNOWN]	= "Unknown",
	[POWER_SUPPLY_CHARGE_TYPE_NONE]		= "N/A",
	[POWER_SUPPLY_CHARGE_TYPE_TRICKLE]	= "Trickle",
	[POWER_SUPPLY_CHARGE_TYPE_FAST]		= "Fast",
	[POWER_SUPPLY_CHARGE_TYPE_STANDARD]	= "Standard",
	[POWER_SUPPLY_CHARGE_TYPE_ADAPTIVE]	= "Adaptive",
	[POWER_SUPPLY_CHARGE_TYPE_CUSTOM]	= "Custom",
};

static const char *const POWER_SUPPLY_HEALTH_TEXT[] = {
	[POWER_SUPPLY_HEALTH_UNKNOWN]		    = "Unknown",
	[POWER_SUPPLY_HEALTH_GOOD]		    = "Good",
	[POWER_SUPPLY_HEALTH_OVERHEAT]		    = "Overheat",
	[POWER_SUPPLY_HEALTH_DEAD]		    = "Dead",
	[POWER_SUPPLY_HEALTH_OVERVOLTAGE]	    = "Over voltage",
	[POWER_SUPPLY_HEALTH_UNSPEC_FAILURE]	    = "Unspecified failure",
	[POWER_SUPPLY_HEALTH_COLD]		    = "Cold",
	[POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE] = "Watchdog timer expire",
	[POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE]   = "Safety timer expire",
	[POWER_SUPPLY_HEALTH_OVERCURRENT]	    = "Over current",
	[POWER_SUPPLY_HEALTH_CALIBRATION_REQUIRED]  = "Calibration required",
	[POWER_SUPPLY_HEALTH_WARM]		    = "Warm",
	[POWER_SUPPLY_HEALTH_COOL]		    = "Cool",
	[POWER_SUPPLY_HEALTH_HOT]		    = "Hot",
};

static void oplus_comm_update_soc_jiffies(struct oplus_chg_comm *chip)
{
	chip->soc_up_update_jiffies = jiffies;
	chip->soc_down_update_jiffies = jiffies;
}

static void oplus_comm_plugout_update_down_jiffies(struct oplus_chg_comm *chip, bool wired_online, bool wls_online)
{
	if (!wired_online && !wls_online && chip->smooth_soc >= PLUGOUT_SOC_THRESHOLD &&
	    chip->ui_soc >= PLUGOUT_SOC_THRESHOLD) {
		chg_info("smooth_soc=%d, ui_soc=%d, plugout soc_down from %ums to %ums\n", chip->smooth_soc,
			 chip->ui_soc, jiffies_to_msecs(chip->soc_down_update_jiffies), jiffies_to_msecs(jiffies));
		chip->soc_down_update_jiffies = jiffies;
	}
}

bool oplus_comm_get_boot_completed(void)
{
	return g_boot_completed;
}

bool oplus_comm_get_hmac_not_pop_up(struct oplus_mms *topic)
{
	struct oplus_chg_comm *chip;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return 0;
	}
	chip = oplus_mms_get_drvdata(topic);
	return chip->config.not_pop_up;
}

const char *oplus_comm_get_temp_region_str(enum oplus_temp_region temp_region)
{
	return oplus_comm_temp_region_text[temp_region];
}

const char *
oplus_comm_get_ffc_temp_region_str(enum oplus_ffc_temp_region temp_region)
{
	return oplus_comm_ffc_temp_region_text[temp_region];
}

__maybe_unused static bool is_wired_icl_votable_available(struct oplus_chg_comm *chip)
{
	if (!chip->wired_icl_votable)
		chip->wired_icl_votable = find_votable("WIRED_ICL");
	return !!chip->wired_icl_votable;
}

__maybe_unused static bool is_wired_fcc_votable_available(struct oplus_chg_comm *chip)
{
	if (!chip->wired_fcc_votable)
		chip->wired_fcc_votable = find_votable("WIRED_FCC");
	return !!chip->wired_fcc_votable;
}

__maybe_unused static bool is_wls_icl_votable_available(struct oplus_chg_comm *chip)
{
	if (!chip->wls_icl_votable)
		chip->wls_icl_votable = find_votable("WLS_NOR_ICL");
	return !!chip->wls_icl_votable;
}

__maybe_unused static bool is_wls_fcc_votable_available(struct oplus_chg_comm *chip)
{
	if (!chip->wls_fcc_votable)
		chip->wls_fcc_votable = find_votable("WLS_NOR_FCC");
	return !!chip->wls_fcc_votable;
}

__maybe_unused static bool
is_wired_charging_disable_votable_available(struct oplus_chg_comm *chip)
{
	if (!chip->wired_charging_disable_votable)
		chip->wired_charging_disable_votable =
			find_votable("WIRED_CHARGING_DISABLE");
	return !!chip->wired_charging_disable_votable;
}

__maybe_unused static bool
is_wired_charge_suspend_votable_available(struct oplus_chg_comm *chip)
{
	if (!chip->wired_charge_suspend_votable)
		chip->wired_charge_suspend_votable =
			find_votable("WIRED_CHARGE_SUSPEND");
	return !!chip->wired_charge_suspend_votable;
}

__maybe_unused static bool
is_gauge_term_voltage_votable_available(struct oplus_chg_comm *chip)
{
	if (!chip->gauge_term_voltage_votable)
		chip->gauge_term_voltage_votable =
			find_votable("GAUGE_TERM_VOLTAGE");
	return !!chip->gauge_term_voltage_votable;
}

__maybe_unused static bool
is_wls_charging_disable_votable_available(struct oplus_chg_comm *chip)
{
	if (!chip->wls_charging_disable_votable)
		chip->wls_charging_disable_votable =
			find_votable("WLS_NOR_OUT_DISABLE");
	return !!chip->wls_charging_disable_votable;
}

__maybe_unused static bool
is_wls_charge_suspend_votable_available(struct oplus_chg_comm *chip)
{
	if (!chip->wls_charge_suspend_votable)
		chip->wls_charge_suspend_votable =
			find_votable("WLS_CHARGE_SUSPEND");
	return !!chip->wls_charge_suspend_votable;
}

__maybe_unused static bool
is_wls_fastchg_disable_votable_available(struct oplus_chg_comm *chip)
{
	if (!chip->wls_fastchg_disable_votable)
		chip->wls_fastchg_disable_votable =
			find_votable("WLS_FASTCHG_DISABLE");
	return !!chip->wls_fastchg_disable_votable;
}

__maybe_unused static bool
is_wls_comu_votable_available(struct oplus_chg_comm *chip)
{
	if (!chip->wls_comu_votable)
		chip->wls_comu_votable =
			find_votable("WLS_COMU");
	return !!chip->wls_comu_votable;
}

__maybe_unused static bool
is_vooc_curr_votable_available(struct oplus_chg_comm *chip)
{
	if (!chip->vooc_curr_votable)
		chip->vooc_curr_votable = find_votable("VOOC_CURR");
	return !!chip->vooc_curr_votable;
}

__maybe_unused static bool
is_ufcs_curr_votable_available(struct oplus_chg_comm *chip)
{
	if (!chip->ufcs_curr_votable)
		chip->ufcs_curr_votable = find_votable("UFCS_CURR");
	return !!chip->ufcs_curr_votable;
}

static bool is_main_gauge_topic_available(struct oplus_chg_comm *chip)
{
	if (!chip->main_gauge_topic)
		chip->main_gauge_topic = oplus_mms_get_by_name("gauge:0");

	return !!chip->main_gauge_topic;
}

static bool is_sub_gauge_topic_available(struct oplus_chg_comm *chip)
{
	if (!chip->sub_gauge_topic)
		chip->sub_gauge_topic = oplus_mms_get_by_name("gauge:1");

	return !!chip->sub_gauge_topic;
}

__maybe_unused static bool
is_cpa_topic_available(struct oplus_chg_comm *chip)
{
	if (!chip->cpa_topic)
		chip->cpa_topic = oplus_mms_get_by_name("cpa");

	return !!chip->cpa_topic;
}

static bool is_parallel_topic_available(struct oplus_chg_comm *chip)
{
	if (!chip->parallel_topic)
		chip->parallel_topic = oplus_mms_get_by_name("parallel");

	return !!chip->parallel_topic;
}

__maybe_unused static bool is_err_topic_available(struct oplus_chg_comm *chip)
{
	if (!chip->err_topic)
		chip->err_topic = oplus_mms_get_by_name("error");
	return !!chip->err_topic;
}

__maybe_unused static bool
is_flash_mode_votable_available(struct oplus_chg_comm *chip)
{
	if (!chip->flash_mode_votable)
		chip->flash_mode_votable = find_votable("FLASH_MODE");
	return !!chip->flash_mode_votable;
}

static bool is_wls_fastchg_started(struct oplus_chg_comm *chip)
{
	union mms_msg_data data = { 0 };

	if (is_wls_charging_disable_votable_available(chip))
		if (get_client_vote(chip->wls_charging_disable_votable, FFC_VOTER) > 0)
			return true;
	if (chip->wls_topic)
		oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_FASTCHG_STATUS, &data, true);
	return !!data.intval;
}

static enum oplus_temp_region
oplus_comm_get_temp_region(struct oplus_chg_comm *chip)
{
	int temp;
	enum oplus_temp_region temp_region = TEMP_REGION_MAX;
	int i;

	temp = chip->shell_temp;
	for (i = 0; i < TEMP_REGION_MAX - 1; i++) {
		if (temp < chip->batt_temp_dynamic_thr[i]) {
			temp_region = i;
			break;
		}
	}
	if (temp_region == TEMP_REGION_MAX)
		temp_region = TEMP_REGION_MAX - 1;

	return temp_region;
}

static void oplus_comm_temp_thr_init(struct oplus_chg_comm *chip)
{
	struct oplus_comm_spec_config *spec = NULL;
	int i;

	if (!chip)
		return;

	spec = &chip->spec;
	for (i = 0; i < TEMP_REGION_MAX - 1; i++)
		chip->batt_temp_dynamic_thr[i] = spec->batt_temp_thr[i];
}

static void oplus_comm_temp_thr_update(struct oplus_chg_comm *chip,
				     enum oplus_temp_region now_temp_region,
				     enum oplus_temp_region pre_temp_region)
{
	struct oplus_comm_spec_config *spec = NULL;

	if (!chip)
		return;

	spec = &chip->spec;
	oplus_comm_temp_thr_init(chip);
	chg_debug("now_temp_region=%d, per_temp_region=%d\n", now_temp_region, pre_temp_region);

	/* temp_region change to lower temp_region */
	if ((pre_temp_region > now_temp_region) &&
	    (pre_temp_region - now_temp_region == 1) &&
	    (now_temp_region >= TEMP_REGION_COLD) &&
	    (now_temp_region < TEMP_REGION_WARM)) {
		if (pre_temp_region < TEMP_REGION_MAX - 1) {
			chip->batt_temp_dynamic_thr[now_temp_region] =
				spec->batt_temp_thr[now_temp_region] + BATT_TEMP_HYST;

			chg_info("now_temp_region=%d, pre_temp_region=%d, update the current batt_temp_thr[%d] to %d\n",
				  now_temp_region,
				  pre_temp_region,
				  now_temp_region,
				  chip->batt_temp_dynamic_thr[now_temp_region]);
		} else {
			chg_err("now_temp_region=%d is out of the batt_temp_dynamic_thr\n", now_temp_region);
		}
	} else if ((pre_temp_region < now_temp_region) &&
		   (now_temp_region - pre_temp_region == 1) &&
		   (now_temp_region > TEMP_REGION_LITTLE_COLD) &&
		   (now_temp_region < TEMP_REGION_MAX)) {
		if (pre_temp_region < TEMP_REGION_MAX - 1) {
			chip->batt_temp_dynamic_thr[pre_temp_region] =
				spec->batt_temp_thr[pre_temp_region] - BATT_TEMP_HYST;

			chg_info("now_temp_region=%d, pre_temp_region = %d, update the lower batt_temp_thr[%d] to %d\n",
				  now_temp_region,
				  pre_temp_region,
				  pre_temp_region,
				  chip->batt_temp_dynamic_thr[pre_temp_region]);
		} else {
			chg_err("pre_temp_region=%d is out of the batt_temp_dynamic_thr\n", pre_temp_region);
		}
	}

	chg_debug("temp_region: %d %d, shell_temp: %d, temp_thr: %d %d %d %d %d %d %d %d\n",
		   now_temp_region,
		   pre_temp_region,
		   chip->shell_temp,
		   chip->batt_temp_dynamic_thr[0],
		   chip->batt_temp_dynamic_thr[1],
		   chip->batt_temp_dynamic_thr[2],
		   chip->batt_temp_dynamic_thr[3],
		   chip->batt_temp_dynamic_thr[4],
		   chip->batt_temp_dynamic_thr[5],
		   chip->batt_temp_dynamic_thr[6],
		   chip->batt_temp_dynamic_thr[7]);
}

static enum oplus_ffc_temp_region
oplus_comm_get_ffc_temp_region(struct oplus_chg_comm *chip)
{
	int temp;
	enum oplus_ffc_temp_region ffc_temp_region = FFC_TEMP_REGION_MAX;
	int i;

	temp = chip->shell_temp;
	for (i = 0; i < FFC_TEMP_REGION_MAX - 1; i++) {
		if (temp < chip->ffc_temp_dynamic_thr[i]) {
			ffc_temp_region = i;
			break;
		}
	}
	if (ffc_temp_region == FFC_TEMP_REGION_MAX)
		ffc_temp_region = FFC_TEMP_REGION_MAX - 1;

	return ffc_temp_region;
}

static void
oplus_comm_ffc_temp_thr_init(struct oplus_chg_comm *chip,
			     enum oplus_ffc_temp_region ffc_temp_region)
{
	struct oplus_comm_spec_config *spec = &chip->spec;
	int i;

	for (i = 0; i < FFC_TEMP_REGION_MAX - 1; i++) {
		if (i == ffc_temp_region - 1 && ffc_temp_region > FFC_TEMP_REGION_NORMAL)
			chip->ffc_temp_dynamic_thr[i] =
				spec->ffc_temp_thr[i] - BATT_TEMP_HYST;
		else if (i == ffc_temp_region && ffc_temp_region < FFC_TEMP_REGION_NORMAL)
			chip->ffc_temp_dynamic_thr[i] =
				spec->ffc_temp_thr[i] + BATT_TEMP_HYST;
		else
			chip->ffc_temp_dynamic_thr[i] = spec->ffc_temp_thr[i];
	}

	for (i = 1; i < FFC_TEMP_REGION_MAX - 1; i++) {
		if (chip->ffc_temp_dynamic_thr[i] <
		    chip->ffc_temp_dynamic_thr[i - 1])
			chip->ffc_temp_dynamic_thr[i] =
				chip->ffc_temp_dynamic_thr[i - 1];
	}
}

static int oplus_comm_set_vbat_uv_thr(struct oplus_chg_comm *chip, int uv_thr)
{
	struct mms_msg *msg;
	int rc;

	chg_info("set uv_thr=%d\n", uv_thr);
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  COMM_ITEM_VBAT_UV_THR);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish uv_thr msg error, rc=%d\n", rc);
		kfree(msg);
		return rc;
	}

	return 0;
}

/* This function is called only when reducing FV */
static bool oplus_comm_check_allow_set_fv(struct oplus_chg_comm *chip, int fv_mv)
{
	bool ret = true;  /* default is allowed to set the CV. */

	if (NULL == chip) {
		chg_err("chip is NULL");
		return ret;
	}

	/*
	 *  Resolve the issue of PMIC not providing online current
	 *  when battery voltage exceeds CV voltage in high-temperature scenarios
	 */
	if ((chip->vbat_mv_max > fv_mv - VBAT_COLD_WARM_COMP) &&
	    (chip->temp_region >= TEMP_REGION_WARM)) {
		chg_info("temp region = %s, not allow set to fv_mv %d!\n",
			  oplus_comm_get_temp_region_str(chip->temp_region), fv_mv);
		ret = false;
	}

	return ret;
}

#define GAUGE_VBAT_UV_DELATA	100
static void oplus_comm_check_temp_region(struct oplus_chg_comm *chip)
{
	struct oplus_comm_spec_config *spec = &chip->spec;
	enum oplus_temp_region temp_region = TEMP_REGION_MAX;
	struct mms_msg *msg;
	int fv_mv = 0;
	int rc;

	temp_region = oplus_comm_get_temp_region(chip);
	if (chip->temp_region == temp_region)
		return;

	if (temp_region != chip->temp_region) {
		if (chip->temp_region == TEMP_REGION_WARM &&
				temp_region < TEMP_REGION_WARM) {
			if (chip->soc != NORMAL_FULL_SOC && chip->batt_full
					&& chip->batt_status == POWER_SUPPLY_STATUS_FULL) {
				chip->sw_full = false;
				chip->hw_full_by_sw = false;
				oplus_comm_set_batt_full(chip, false);
				if (is_support_parallel_battery(chip->gauge_topic)) {
					chip->sw_sub_batt_full = false;
					chip->hw_sub_batt_full_by_sw = false;
					oplus_comm_set_sub_batt_full(chip, false);
				}
				chg_err("clear warm full status\n");
				if (chip->wls_online) {
					if (is_wls_charging_disable_votable_available(chip))
						vote(chip->wls_charging_disable_votable,
							CHG_FULL_VOTER, false, 0, false);
					else
						chg_err("wls_charging_disable_votable not found, can't enable charging");
				} else {
					if (is_wired_charging_disable_votable_available(chip))
						vote(chip->wired_charging_disable_votable,
							CHG_FULL_VOTER, false, 0, false);
					else
						chg_err("wired_charging_disable_votable not found, can't enable charging");
				}
			}
		}
	}

	oplus_comm_temp_thr_update(chip, temp_region, chip->temp_region);
	chip->temp_region = temp_region;

	if (chip->wired_online || chip->wls_online) {
		fv_mv = spec->fv_mv[temp_region] - spec->dec_cv.dec_vol;
		if (oplus_comm_check_allow_set_fv(chip, fv_mv))
			vote(chip->fv_max_votable, SPEC_VOTER, true,
			     fv_mv, false);

		if (chip->fv_over) {
			fv_mv = spec->fv_mv[chip->temp_region] -
				spec->dec_cv.dec_vol -
				spec->sw_over_fv_dec_mv;
			if (fv_mv <= 0) {
				vote(chip->fv_min_votable, OVER_FV_VOTER, false,
				     0, false);
			} else {
				if (oplus_comm_check_allow_set_fv(chip, fv_mv))
					vote(chip->fv_min_votable, OVER_FV_VOTER, true, fv_mv, false);
			}
		} else {
			vote(chip->fv_min_votable, OVER_FV_VOTER, false, 0,
			     false);
		}
	}

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  COMM_ITEM_TEMP_REGION);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		goto out;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish msg error, rc=%d\n", rc);
		kfree(msg);
	}

out:
	chg_info("temp region = %s\n",
		 oplus_comm_get_temp_region_str(temp_region));
}

static enum oplus_fcc_gear oplus_comm_calc_fcc_gear(struct oplus_chg_comm *chip)
{
	static enum oplus_temp_region prev_temp_region = TEMP_REGION_MAX;
	enum oplus_fcc_gear fcc_gear = FCC_GEAR_LOW;
	int gear_cnt = chip->spec.fcc_gear_cnt;
	int vbat = chip->vbat_mv_max;
	int temp_region = chip->temp_region;
	bool temp_changed = (temp_region != prev_temp_region);
	int thr;
	int shake;
	int boundary;
	int i;

	for (i = 0; i < gear_cnt - 1; i++) {
		thr = chip->spec.fcc_gear_thr_mv[i][temp_region];
		shake = chip->spec.fcc_gear_shake_mv[i][temp_region];

		if (temp_changed || chip->fcc_gear <= i)
			boundary = thr;
		else
			boundary = thr - shake;

		if (vbat >= boundary)
			fcc_gear = i + 1;
		else
			break;
	}

	prev_temp_region = temp_region;

	if (fcc_gear >= max(1, gear_cnt - 1))
		fcc_gear = FCC_GEAR_HIGH;

	if (fcc_gear != chip->fcc_gear)
		chg_info("fcc_gear change: prev=%d, new=%d, vbat=%d, region=%d, thr_mv=%d, shake_mv=%d boundary_mv=%d\n",
			 chip->fcc_gear, fcc_gear, vbat, temp_region, thr, shake, boundary);
	return fcc_gear;
}

static void oplus_comm_check_fcc_gear(struct oplus_chg_comm *chip, bool reset)
{
	enum oplus_fcc_gear fcc_gear = FCC_GEAR_LOW;
	struct mms_msg *msg;
	static bool first_init = true;
	int rc;

	if (reset)
		goto push_fcc_gear;
	fcc_gear = oplus_comm_calc_fcc_gear(chip);

	if (fcc_gear == chip->fcc_gear && !first_init)
		return;
	first_init = false;

push_fcc_gear:
	chip->fcc_gear = fcc_gear;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  COMM_ITEM_FCC_GEAR);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		goto out;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish msg error, rc=%d\n", rc);
		kfree(msg);
	}

out:
	chg_info("fcc_gear = %d, gear_cnt = %d, reset = %d\n",
		 fcc_gear, chip->spec.fcc_gear_cnt, reset);
}

static void oplus_comm_set_cv_cutoff_volt_curr(
		struct oplus_chg_comm *chip, int volt, int curr, int sub_curr)
{
	int rc;
	unsigned int cutoff_volt_curr;
	struct mms_msg *msg;

	if (volt < 0 || curr < 0)
		return;

	cutoff_volt_curr = (volt << CUTOFF_DATA_SHIFT) | (curr << CUTOFF_ITERM_SHIFT) | sub_curr;
	if (chip->cv_cutoff_volt_curr == cutoff_volt_curr)
		return;

	chip->cv_cutoff_volt_curr = cutoff_volt_curr;
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  COMM_ITEM_CV_CUTOFF_VOLT_CURR);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish battery full msg error, rc=%d\n", rc);
		kfree(msg);
	}

	chg_info("cutoff_volt_curr=0x%x\n", chip->cv_cutoff_volt_curr);
}

static void oplus_comm_set_ffc_cutoff_curr(
		struct oplus_chg_comm *chip, int cutoff_curr, int ffc_temp_region)
{
	int rc;
	int sub_cutoff_curr = 0;
	unsigned int eq_cutoff_curr;
	struct mms_msg *msg;
	struct oplus_comm_spec_config *spec = &chip->spec;

	if (cutoff_curr < 0 || sub_cutoff_curr < 0)
		return;

	if (is_support_parallel_battery(chip->gauge_topic)) {
		if (chip->wired_online)
			sub_cutoff_curr = spec->wired_ffc_fcc_sub_cutoff_ma[chip->ffc_step][ffc_temp_region - 1];
		else if (chip->wls_online)
			sub_cutoff_curr = spec->wls_ffc_fcc_sub_cutoff_ma[chip->ffc_step][ffc_temp_region - 1];
		else
			chg_err("wired and wireless charge is offline\n");
	}

	eq_cutoff_curr = (cutoff_curr << CUTOFF_DATA_SHIFT) | sub_cutoff_curr;
	if (chip->ffc_cutoff_curr == eq_cutoff_curr)
		return;

	chip->ffc_cutoff_curr = eq_cutoff_curr;
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  COMM_ITEM_FFC_CUTOFF_CURR);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish battery full msg error, rc=%d\n", rc);
		kfree(msg);
	}

	chg_info("cutoff_curr=0x%x\n", chip->ffc_cutoff_curr);
}

static void oplus_comm_set_rechging(struct oplus_chg_comm *chip, bool rechging)
{
	struct mms_msg *msg;
	int rc;

	if (chip->rechging == rechging)
		return;
	chip->rechging = rechging;
	chg_info("rechging=%s\n", rechging ? "true" : "false");

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  COMM_ITEM_RECHGING);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish rechging msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_comm_set_batt_cv_full(struct oplus_chg_comm *chip)
{
	bool batt_cv_full;
	struct mms_msg *msg;
	int rc;

	if (chip->sw_full || chip->hw_full_by_sw)
		batt_cv_full = true;
	else
		batt_cv_full = false;

	if (chip->batt_cv_full == batt_cv_full)
		return;

	chip->batt_cv_full = batt_cv_full;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH,
				  COMM_ITEM_BATT_CV_FULL);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish battery cv full msg error, rc=%d\n", rc);
		kfree(msg);
	}

	chg_info("batt_cv_full=%s\n", batt_cv_full ? "true" : "false");
}

static void oplus_comm_set_batt_full(struct oplus_chg_comm *chip, bool full)
{
	struct mms_msg *msg;
	int rc;

	oplus_comm_set_batt_cv_full(chip);

	full |= chip->sw_full || chip->hw_full_by_sw;

	if (chip->batt_full == full)
		return;
	chip->batt_full = full;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH,
				  COMM_ITEM_CHG_FULL);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish battery full msg error, rc=%d\n", rc);
		kfree(msg);
	}

	if (full)
		oplus_comm_set_rechging(chip, false);

	chg_info("batt_full=%s\n", full ? "true" : "false");
}

static void oplus_comm_set_sub_batt_full(struct oplus_chg_comm *chip, bool full)
{
	struct mms_msg *msg;
	int rc;

	full = full || chip->sw_sub_batt_full || chip->hw_sub_batt_full_by_sw;

	if (chip->sub_batt_full == full)
		return;
	chip->sub_batt_full = full;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH,
				  COMM_ITEM_CHG_SUB_BATT_FULL);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish battery full msg error, rc=%d\n", rc);
		kfree(msg);
	}

	chg_info("sub_batt_full=%s\n", full ? "true" : "false");
}

static void oplus_comm_start_timeout_work(struct oplus_chg_comm *chip)
{
	int max_chg_time_sec = oplus_dbg_timeout ? oplus_dbg_timeout : chip->spec.max_chg_time_sec;

	if (max_chg_time_sec <= 0 || (!chip->wired_online && !chip->wls_online))
		return;

	if (!chip->need_start_timeout_work)
		return;

	if (chip->wls_online)
		goto start;

	if (chip->chging_over_time)
		return;

	if (chip->plc_status != PLC_STATUS_ENABLE)
		goto start;

	if (delayed_work_pending(&chip->charge_timeout_work))
		cancel_delayed_work_sync(&chip->charge_timeout_work);

	return;
start:
	schedule_delayed_work(&chip->charge_timeout_work, msecs_to_jiffies(max_chg_time_sec * 1000));
}

static void oplus_comm_check_sw_full(struct oplus_chg_comm *chip)
{
	struct oplus_comm_spec_config *spec = &chip->spec;
	int sw_fv_mv;
	int ibat_ma;

	if (!chip->wired_online && !chip->wls_online)
		return;
	if (chip->hw_full_by_sw || chip->sw_full)
		return;
	if (chip->ffc_status != FFC_DEFAULT)
		return;

	if (chip->vooc_by_normal_path) {
		if (chip->vooc_charging &&
		    sid_to_adapter_chg_type(chip->vooc_sid) != CHARGER_TYPE_VOOC)
			return;
	} else {
		if (chip->vooc_charging)
			return;
	}

	if (chip->ufcs_charging)
		return;

	if (chip->pps_charging)
		return;

	if (is_wls_fastchg_started(chip))
		return;

	sw_fv_mv = spec->sw_fv_mv[chip->temp_region] - spec->dec_cv.dec_vol;
	if (!chip->authenticate || !chip->hmac)
		sw_fv_mv = spec->non_standard_sw_fv_mv;
	oplus_comm_set_cv_cutoff_volt_curr(chip, sw_fv_mv,
		spec->iterm_ma, spec->sub_iterm_ma);
	if (sw_fv_mv <= 0)
		goto clean;
	if (chip->vbat_mv <= sw_fv_mv)
		goto clean;

	if (is_support_parallel_battery(chip->gauge_topic))
		ibat_ma = chip->main_ibat_ma;
	else
		ibat_ma = chip->ibat_ma;

	if (ibat_ma < 0 && abs(ibat_ma) <= spec->iterm_ma) {
		chip->sw_full_count++;
		if (chip->sw_full_count > spec->sw_check_full_cnt) {
			chip->sw_full_count = 0;
			chip->sw_full = true;
			if (chip->wls_online) {
				if (is_wls_charging_disable_votable_available(chip)) {
					vote(chip->wls_charging_disable_votable, CHG_FULL_VOTER, true, 1, false);
					if (is_wls_icl_votable_available(chip))
						vote(chip->wls_icl_votable, CHG_FULL_VOTER, true, 300, true);
					if (chip->rechg_soc_en && chip->ui_soc == 100) {
						if (is_wls_fastchg_disable_votable_available(chip) &&
						    is_wls_comu_votable_available(chip)) {
							vote(chip->wls_fastchg_disable_votable, CHG_FULL_VOTER, true, 1, false);
							vote(chip->wls_comu_votable, CHG_FULL_VOTER, true, WLS_RX_COMU_CAP_AB, true);
						}
					}
				} else {
					chg_err("wls_charging_disable_votable not found, can't disable charging");
				}
			} else {
				if (is_wired_charging_disable_votable_available(chip))
					vote(chip->wired_charging_disable_votable, CHG_FULL_VOTER, true, 1, false);
				else
					chg_err("wired_charging_disable_votable not found, can't disable charging");
			}
			chip->fv_over = false;
			vote(chip->fv_min_votable, OVER_FV_VOTER, false, 0, false);
			chip->need_start_timeout_work = false;
			cancel_delayed_work_sync(&chip->charge_timeout_work);
			chg_info("battery full\n");
			oplus_comm_set_batt_full(chip, true);
			return;
		}
	} else if (ibat_ma >= 0) {
		chip->sw_full_count++;
		if (chip->sw_full_count > spec->sw_check_full_cnt * 2) {
			chip->sw_full_count = 0;
			chip->sw_full = true;
			if (chip->wls_online) {
				if (is_wls_charging_disable_votable_available(chip)) {
					vote(chip->wls_charging_disable_votable, CHG_FULL_VOTER, true, 1, false);
					if (is_wls_icl_votable_available(chip))
						vote(chip->wls_icl_votable, CHG_FULL_VOTER, true, 300, true);
					if (chip->rechg_soc_en && chip->ui_soc == 100) {
						if (is_wls_fastchg_disable_votable_available(chip) &&
						    is_wls_comu_votable_available(chip)) {
							vote(chip->wls_fastchg_disable_votable, CHG_FULL_VOTER, true, 1, false);
							vote(chip->wls_comu_votable, CHG_FULL_VOTER, true, WLS_RX_COMU_CAP_AB, true);
						}
					}
				} else {
					chg_err("wls_charging_disable_votable not found, can't disable charging");
				}
			} else {
				if (is_wired_charging_disable_votable_available(chip))
					vote(chip->wired_charging_disable_votable, CHG_FULL_VOTER, true, 1, false);
				else
					chg_err("wired_charging_disable_votable not found, can't disable charging");
			}
			chip->fv_over = false;
			vote(chip->fv_min_votable, OVER_FV_VOTER, false, 0, false);
			chip->need_start_timeout_work = false;
			cancel_delayed_work_sync(&chip->charge_timeout_work);
			chg_info("Battery full by sw when ibat>=0!!\n");
			oplus_comm_set_batt_full(chip, true);
			return;
		}
	} else {
		goto clean;
	}

	return;

clean:
	chip->sw_full = false;
	chip->sw_full_count = 0;
}

static void oplus_comm_check_hw_full(struct oplus_chg_comm *chip)
{
	struct oplus_comm_spec_config *spec = &chip->spec;
	int hw_fv_mv;

	if (!chip->wired_online && !chip->wls_online)
		return;
	if (chip->hw_full_by_sw || chip->sw_full)
		return;
	if (chip->ffc_status != FFC_DEFAULT)
		return;

	if (chip->vooc_by_normal_path) {
		if (chip->vooc_charging &&
		    sid_to_adapter_chg_type(chip->vooc_sid) != CHARGER_TYPE_VOOC)
			return;
	} else {
		if (chip->vooc_charging)
			return;
	}

	if (chip->ufcs_charging)
		return;

	if (chip->pps_charging)
		return;

	if (is_wls_fastchg_started(chip))
		return;

	hw_fv_mv = spec->fv_mv[chip->temp_region] - spec->dec_cv.dec_vol + spec->hw_fv_inc_mv[chip->temp_region];
	if (!chip->authenticate || !chip->hmac)
		hw_fv_mv = spec->non_standard_fv_mv +
			   spec->non_standard_hw_fv_inc_mv;
	if (hw_fv_mv <= 0)
		goto clean;
	if (chip->vbat_mv <= hw_fv_mv)
		goto clean;

	chip->hw_full_count++;
	if (chip->hw_full_count > FULL_COUNTS_HW) {
		chip->hw_full_count = 0;
		chip->hw_full_by_sw = true;
		if (chip->wls_online) {
			if (is_wls_charging_disable_votable_available(chip)) {
				vote(chip->wls_charging_disable_votable, CHG_FULL_VOTER, true, 1, false);
				if (is_wls_icl_votable_available(chip))
					vote(chip->wls_icl_votable, CHG_FULL_VOTER, true, 300, true);
				if (chip->rechg_soc_en && chip->ui_soc == 100) {
					if (is_wls_fastchg_disable_votable_available(chip) &&
					    is_wls_comu_votable_available(chip)) {
						vote(chip->wls_fastchg_disable_votable, CHG_FULL_VOTER, true, 1, false);
						vote(chip->wls_comu_votable, CHG_FULL_VOTER, true, WLS_RX_COMU_CAP_AB, true);
					}
				}
			} else {
				chg_err("wls_charging_disable_votable not found, can't disable charging");
			}
		} else {
			if (is_wired_charging_disable_votable_available(chip))
				vote(chip->wired_charging_disable_votable, CHG_FULL_VOTER, true, 1, false);
			else
				chg_err("wired_charging_disable_votable not found, can't disable charging");
		}
		chip->fv_over = false;
		vote(chip->fv_min_votable, OVER_FV_VOTER, false, 0, false);
		chip->need_start_timeout_work = false;
		cancel_delayed_work_sync(&chip->charge_timeout_work);
		chg_info("battery full\n");
		oplus_comm_set_batt_full(chip, true);
	}

	return;

clean:
	chip->hw_full_by_sw = false;
	chip->hw_full_count = 0;
}

static void oplus_comm_check_sw_sub_batt_full(struct oplus_chg_comm *chip)
{
	struct oplus_comm_spec_config *spec = &chip->spec;
	int sw_fv_mv;

	if (!chip->wired_online && !chip->wls_online)
		return;
	if (chip->hw_sub_batt_full_by_sw || chip->sw_sub_batt_full)
		return;
	if (chip->ffc_status != FFC_DEFAULT)
		return;

	if (chip->vooc_by_normal_path) {
		if (chip->vooc_charging &&
		    sid_to_adapter_chg_type(chip->vooc_sid) != CHARGER_TYPE_VOOC)
			return;
	} else {
		if (chip->vooc_charging)
			return;
	}

	if (chip->ufcs_charging)
		return;

	if (chip->pps_charging)
		return;

	if (is_wls_fastchg_started(chip))
		return;

	sw_fv_mv = spec->sw_fv_mv[chip->temp_region] - spec->dec_cv.dec_vol;
	if (!chip->authenticate || !chip->hmac)
		sw_fv_mv = spec->non_standard_sw_fv_mv;
	if (sw_fv_mv <= 0)
		goto clean;
	if (chip->sub_vbat_mv <= sw_fv_mv)
		goto clean;

	if (chip->sub_ibat_ma < 0 && abs(chip->sub_ibat_ma) <= spec->sub_iterm_ma) {
		chip->sw_sub_batt_full_count++;
		if (chip->sw_sub_batt_full_count > spec->sw_check_full_cnt) {
			chip->sw_sub_batt_full_count = 0;
			chip->sw_sub_batt_full = true;
			chg_info("sub battery full\n");
			oplus_comm_set_sub_batt_full(chip, true);
			if (is_support_parallel_battery(chip->gauge_topic) == PARALLEL_CONNECT_TYPE)
				return;
			chip->sw_full_count = 0;
			chip->sw_full = true;
			if (chip->wls_online) {
				if (is_wls_charging_disable_votable_available(chip)) {
					vote(chip->wls_charging_disable_votable, CHG_FULL_VOTER, true, 1, false);
					if (is_wls_icl_votable_available(chip))
						vote(chip->wls_icl_votable, CHG_FULL_VOTER, true, 300, true);
					if (chip->rechg_soc_en && chip->ui_soc == 100) {
						if (is_wls_fastchg_disable_votable_available(chip) &&
						    is_wls_comu_votable_available(chip)) {
							vote(chip->wls_fastchg_disable_votable, CHG_FULL_VOTER, true,
							     1, false);
							vote(chip->wls_comu_votable, CHG_FULL_VOTER, true,
							     WLS_RX_COMU_CAP_AB, true);
						}
					}
				} else {
					chg_err("wls_charging_disable_votable not found, can't disable charging");
				}
			} else {
				if (is_wired_charging_disable_votable_available(chip))
					vote(chip->wired_charging_disable_votable, CHG_FULL_VOTER, true, 1, false);
				else
					chg_err("wired_charging_disable_votable not found, can't disable charging");
			}
			chip->fv_over = false;
			vote(chip->fv_min_votable, OVER_FV_VOTER, false, 0, false);
			chip->need_start_timeout_work = false;
			cancel_delayed_work_sync(&chip->charge_timeout_work);
			chg_info("battery full\n");
			oplus_comm_set_batt_full(chip, true);
			return;
		}
	} else if (chip->sub_ibat_ma >= 0) {
		chip->sw_sub_batt_full_count++;
		if (chip->sw_sub_batt_full_count > spec->sw_check_full_cnt * 2) {
			chip->sw_sub_batt_full_count = 0;
			chip->sw_sub_batt_full = true;
			chg_info("sub Battery full by sw when ibat>=0!!\n");
			oplus_comm_set_sub_batt_full(chip, true);
			if (is_support_parallel_battery(chip->gauge_topic) == PARALLEL_CONNECT_TYPE)
				return;
			chip->sw_full_count = 0;
			chip->sw_full = true;
			if (chip->wls_online) {
				if (is_wls_charging_disable_votable_available(chip)) {
					vote(chip->wls_charging_disable_votable, CHG_FULL_VOTER, true, 1, false);
					if (is_wls_icl_votable_available(chip))
						vote(chip->wls_icl_votable, CHG_FULL_VOTER, true, 300, true);
					if (chip->rechg_soc_en && chip->ui_soc == 100) {
						if (is_wls_fastchg_disable_votable_available(chip) &&
						    is_wls_comu_votable_available(chip)) {
							vote(chip->wls_fastchg_disable_votable, CHG_FULL_VOTER, true,
							     1, false);
							vote(chip->wls_comu_votable, CHG_FULL_VOTER, true,
							     WLS_RX_COMU_CAP_AB, true);
						}
					}
				} else {
					chg_err("wls_charging_disable_votable not found, can't disable charging");
				}
			} else {
				if (is_wired_charging_disable_votable_available(chip))
					vote(chip->wired_charging_disable_votable, CHG_FULL_VOTER, true, 1, false);
				else
					chg_err("wired_charging_disable_votable not found, can't disable charging");
			}
			chip->fv_over = false;
			vote(chip->fv_min_votable, OVER_FV_VOTER, false, 0, false);
			chip->need_start_timeout_work = false;
			cancel_delayed_work_sync(&chip->charge_timeout_work);
			chg_info("Battery full by sw when ibat>=0!!\n");
			oplus_comm_set_batt_full(chip, true);
			return;
		}
	} else {
		goto clean;
	}

	return;

clean:
	chip->sw_sub_batt_full = false;
	chip->sw_sub_batt_full_count = 0;
	oplus_comm_set_sub_batt_full(chip, false);
}

static void oplus_comm_check_hw_sub_batt_full(struct oplus_chg_comm *chip)
{
	struct oplus_comm_spec_config *spec = &chip->spec;
	int hw_fv_mv;

	if (!chip->wired_online && !chip->wls_online)
		return;
	if (chip->hw_sub_batt_full_by_sw || chip->sw_sub_batt_full)
		return;
	if (chip->ffc_status != FFC_DEFAULT)
		return;

	if (chip->vooc_by_normal_path) {
		if (chip->vooc_charging &&
		    sid_to_adapter_chg_type(chip->vooc_sid) != CHARGER_TYPE_VOOC)
			return;
	} else {
		if (chip->vooc_charging)
			return;
	}

	if (chip->ufcs_charging)
		return;

	if (chip->pps_charging)
		return;

	if (is_wls_fastchg_started(chip))
		return;

	hw_fv_mv = spec->fv_mv[chip->temp_region] - spec->dec_cv.dec_vol +
		   spec->hw_fv_inc_mv[chip->temp_region];
	if (!chip->authenticate || !chip->hmac)
		hw_fv_mv = spec->non_standard_fv_mv +
			   spec->non_standard_hw_fv_inc_mv;
	if (hw_fv_mv <= 0)
		goto clean;
	if (chip->sub_vbat_mv <= hw_fv_mv)
		goto clean;

	chip->hw_sub_batt_full_count++;
	if (chip->hw_sub_batt_full_count > FULL_COUNTS_HW) {
		chip->hw_sub_batt_full_count = 0;
		chip->hw_sub_batt_full_by_sw = true;
		chg_info("sub battery full hw\n");
		oplus_comm_set_sub_batt_full(chip, true);
		if (is_support_parallel_battery(chip->gauge_topic) == PARALLEL_CONNECT_TYPE)
			return;
		chip->hw_full_count = 0;
		chip->hw_full_by_sw = true;
		if (chip->wls_online) {
			if (is_wls_charging_disable_votable_available(chip)) {
				vote(chip->wls_charging_disable_votable, CHG_FULL_VOTER, true, 1, false);
				if (is_wls_icl_votable_available(chip))
					vote(chip->wls_icl_votable, CHG_FULL_VOTER, true, 300, true);
				if (chip->rechg_soc_en && chip->ui_soc == 100) {
					if (is_wls_fastchg_disable_votable_available(chip) &&
					    is_wls_comu_votable_available(chip)) {
						vote(chip->wls_fastchg_disable_votable, CHG_FULL_VOTER, true,
						     1, false);
						vote(chip->wls_comu_votable, CHG_FULL_VOTER, true,
						     WLS_RX_COMU_CAP_AB, true);
					}
				}
			} else {
				chg_err("wls_charging_disable_votable not found, can't disable charging");
			}
		} else {
			if (is_wired_charging_disable_votable_available(chip))
				vote(chip->wired_charging_disable_votable, CHG_FULL_VOTER, true, 1, false);
			else
				chg_err("wired_charging_disable_votable not found, can't disable charging");
		}
		chip->fv_over = false;
		vote(chip->fv_min_votable, OVER_FV_VOTER, false, 0, false);
		chip->need_start_timeout_work = false;
		cancel_delayed_work_sync(&chip->charge_timeout_work);
		chg_info("battery full\n");
		oplus_comm_set_batt_full(chip, true);
	}

	return;

clean:
	chip->hw_sub_batt_full_by_sw = false;
	chip->hw_sub_batt_full_count = 0;
	oplus_comm_set_sub_batt_full(chip, false);
}

static void oplus_comm_check_fv_over(struct oplus_chg_comm *chip)
{
	struct oplus_comm_spec_config *spec = &chip->spec;
	int sw_over_fv_mv;
	int fv_mv;

	if (!chip->wired_online && !chip->wls_online)
		return;
	if (chip->ffc_status != FFC_DEFAULT)
		return;
	if (chip->vooc_charging)
		return;
	if (chip->ufcs_charging)
		return;
	if (chip->pps_charging)
		return;
	if (oplus_comm_is_not_charging(chip) || oplus_comm_is_discharging(chip))
		return;
	if (is_wired_charging_disable_votable_available(chip)) {
		if (get_effective_result(chip->wired_charging_disable_votable) != 0)
			return;
	}

	if (chip->wls_online && is_wls_charging_disable_votable_available(chip)) {
		if (get_effective_result(chip->wls_charging_disable_votable) > 0)
			return;
	}
	if (is_wls_fastchg_started(chip))
		return;

	sw_over_fv_mv = spec->sw_over_fv_mv[chip->temp_region] - spec->dec_cv.dec_vol;
	if (!chip->authenticate || !chip->hmac)
		sw_over_fv_mv = spec->non_standard_sw_over_fv_mv;
	if (sw_over_fv_mv <= 0)
		goto clean;
	if (is_sub_gauge_topic_available(chip)) {
		if (chip->main_vbat_mv <= sw_over_fv_mv && chip->sub_vbat_mv <= sw_over_fv_mv)
			goto clean;
	} else if (chip->vbat_mv <= sw_over_fv_mv) {
		goto clean;
	}

	chip->fv_over_count++;
	if (chip->fv_over_count > VFLOAT_OVER_NUM) {
		chip->fv_over_count = 0;
		chip->fv_over = true;
		fv_mv = get_effective_result(chip->fv_min_votable);
		fv_mv = fv_mv - spec->sw_over_fv_dec_mv;
		if (fv_mv <= 0)
			return;
		if (oplus_comm_check_allow_set_fv(chip, fv_mv))
			vote(chip->fv_min_votable, OVER_FV_VOTER, true, fv_mv, false);
	}

	return;

clean:
	chip->fv_over_count = 0;
}

void oplus_comm_set_rechg_soc_limit(struct oplus_mms *topic, int rechg_soc, bool en)
{
	struct oplus_chg_comm *chip;
	struct mms_msg *msg;
	int rc = 0;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return;
	}
	chip = oplus_mms_get_drvdata(topic);

	if (chip->rechg_soc_en == en
	    && chip->rechg_soc == rechg_soc) {
		chg_err("the same status has been set\n");
		return;
	}

	chip->rechg_soc = rechg_soc;
	chip->rechg_soc_en = en;

	if (!en && (chip->ui_soc < 100)
	    && (chip->temp_region > TEMP_REGION_COLD)
	    && (chip->temp_region < TEMP_REGION_WARM)) {
		chip->rechg_now = true;
		chg_info("recharge directly if disable ui_soc rechg.\n");
	} else if (en && (chip->ui_soc == 100)) {
		oplus_comm_update_soc_jiffies(chip);
		chg_info("keep 100 for 5minutes when enable ui_soc rechg.\n");
	}

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  COMM_ITEM_RECHG_SOC_EN_STATUS);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish rechg_soc_en_status msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

void oplus_comm_get_rechg_soc_limit(struct oplus_mms *topic, int *rechg_soc, bool *en)
{
	struct oplus_chg_comm *chip;
	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return;
	}
	if (!rechg_soc || !en) {
		chg_err("invalid value");
		return;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (!chip) {
		chg_err("oplus_chg_comm chip is NULL\n");
		return;
	}

	*rechg_soc = chip->rechg_soc;
	*en = chip->rechg_soc_en;
}

static void oplus_comm_iterm_check_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_comm *chip = container_of(dwork,
		struct oplus_chg_comm, iterm_check_work);

	oplus_wired_iterm_check(chip->wired_topic, true);
}

static int oplus_chg_track_upload_dec_vol_info(struct oplus_chg_comm *chip)
{
#define TRACK_UPLOAD_COUNT_MAX 3
#define TRACK_LOCAL_T_NS_TO_S_THD 1000000000
#define TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD (24 * 3600)
#define TRACK_DEC_CV_MSG_LENTH 256

	struct oplus_comm_spec_config *spec = &chip->spec;
	struct oplus_mms *err_topic;
	struct mms_msg *msg;
	int rc;
	static int upload_count = 0;
	static int pre_upload_time = 0;
	int curr_time;
	char msg_info[TRACK_DEC_CV_MSG_LENTH] = { 0 };
	int qmax1= 0, bat_soh = 0;


	spec->dec_cv.dec_track = false;
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

	bat_soh = oplus_gauge_get_batt_soh();
	if (spec->dec_cv.dec_spec_support == DEC_CV_SUPPORT_LITE) {
		scnprintf(msg_info, TRACK_DEC_CV_MSG_LENTH,
			 "$$spec_dec_cv_mv@@%d$$dec_delta@@%d$$dec_vol@@%d$$cc@@%d$$fcc@@%d$$soh@@%d$$rm@@%d",
			 spec->dec_cv.lite.dec_vol_fv_mv[spec->dec_cv.index][chip->temp_region], spec->dec_cv.dec_delta,
			 spec->dec_cv.dec_vol, chip->batt_cc, chip->batt_fcc, bat_soh, chip->batt_rm);
	} else {
		oplus_gauge_get_qmax(chip->gauge_topic, 0, &qmax1);
		scnprintf(msg_info, TRACK_DEC_CV_MSG_LENTH,
			  "$$dec_type@@%d$$last_cc@@%d$$cc@@%d$$dbg_soh@@%d$$fact@@%d$$dec_soh@@%d"
			  "$$index@@%d$$vol_dec@@%d$$vct_dec@@%d$$dec_step@@%d"
			  "$$bk_cc@@%d$$bk_vol@@%d$$bk_vct@@%d$$reg_vct@@%d$$n_vol@@%d$$n_vct@@%d"
			  "$$fcc@@%d$$soh@@%d$$rm@@%d$$qmax1@@%d$$qmax2@@%d",
			 spec->dec_cv.full.pack_type, spec->dec_cv.full.last_cc, chip->batt_cc, spec->dec_cv.dbg_soh,
			 spec->dec_cv.full.soh_fact, spec->dec_cv.full.dec_soh, spec->dec_cv.index,
			 spec->dec_cv.full.spec_fv_mv[spec->dec_cv.index], spec->dec_cv.full.spec_vct_mv[spec->dec_cv.index],
			 spec->dec_cv.full.spec_step, chip->dec_cv_down_info.dec_cc, chip->dec_cv_down_info.dec_vol,
			 chip->dec_cv_down_info.dec_vct, spec->dec_cv.full.vct_cur,
			 spec->dec_cv.dec_vol, spec->dec_cv.full.vct_up, chip->batt_fcc, bat_soh, chip->batt_rm, qmax1, qmax1);
	}

	msg = oplus_mms_alloc_str_msg(
		MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_DEC_CV_INFO, msg_info);
	if (msg == NULL) {
		chg_err("alloc ERR_ITEM_DEC_CV_INFO error msg error\n");
		return -ENOMEM;
	}

	rc = oplus_mms_publish_msg_sync(err_topic, msg);
	if (rc < 0) {
		chg_err("publish dec cv error msg error, rc=%d\n", rc);
		kfree(msg);
	}
	upload_count++;

	return rc;
}

static void oplus_chg_track_dec_vol_info_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_comm *chip = container_of(dwork, struct oplus_chg_comm, dec_vol_info_trigger_work);

	oplus_chg_track_upload_dec_vol_info(chip);
}

static int oplus_chg_track_upload_gauge_r_info(struct oplus_chg_comm *chip)
{
#define TRACK_UPLOAD_COUNT_MAX 3
#define TRACK_LOCAL_T_NS_TO_S_THD 1000000000
#define TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD (24 * 3600)
#define TRACK_GAUGE_R_MSG_LENTH 512

	struct oplus_mms *err_topic;
	struct mms_msg *msg;
	union mms_msg_data data = { 0 };
	static int upload_count = 0, pre_upload_time = 0, curr_time;
	char msg_info[TRACK_GAUGE_R_MSG_LENTH] = { 0 };
	int bat_soh = 0;
	int rc, index = 0;


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

	bat_soh = oplus_gauge_get_batt_soh();
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_GAUGE_R_INFO, &data, true);
	if (rc == 0 && data.strval && strlen(data.strval)) {
		chg_err("[gauge_r_reg_info] %s", data.strval);
		index += scnprintf(&(msg_info[index]),
			TRACK_GAUGE_R_MSG_LENTH - index, "$$gauge_r_info@@%s", data.strval);
	}

	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_FCC, &data, false);
	chip->batt_fcc = data.intval;
	index += scnprintf(&(msg_info[index]),
			TRACK_GAUGE_R_MSG_LENTH - index, "$$soc@@%d$$smooth_soc@@%d$$uisoc@@%d$$vbatt_max@@%d$$vbatt_min@@%d"
			"$$ibat_ma@@%d$$batt_temp@@%d$$batt_cc@@%d$$batt_fcc@@%d$$bat_soh@@%d",
			chip->soc, chip->smooth_soc, chip->ui_soc, chip->vbat_mv, chip->vbat_min_mv,
			chip->ibat_ma, chip->batt_temp, chip->batt_cc, chip->batt_fcc, bat_soh);

	msg = oplus_mms_alloc_str_msg(
		MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_GAUGE_R_INFO, msg_info);
	if (msg == NULL) {
		chg_err("alloc GAUGE_ITEM_GAUGE_R_INFO error msg error\n");
		return -ENOMEM;
	}

	rc = oplus_mms_publish_msg_sync(err_topic, msg);
	if (rc < 0) {
		chg_err("publish gauge r para error msg error, rc=%d\n", rc);
		kfree(msg);
	}
	upload_count++;

	return rc;
}

static void oplus_chg_track_gauge_r_info_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_comm *chip = container_of(dwork, struct oplus_chg_comm, gauge_r_info_work);

	oplus_chg_track_upload_gauge_r_info(chip);
}

#define DEC_VOL_UPDATE_MAX 150
#define DEC_VCT_UPDATE_MAX 180
static void oplus_comm_dec_vct_init(struct oplus_chg_comm *chip)
{
	int cnts = 2;
	struct oplus_comm_spec_config *spec = &chip->spec;

	if (!spec->dec_cv.full.vct_en) {
		spec->dec_cv.full.dec_init = true;
		spec->dec_cv.full.vct_cur = 0;
		chip->dec_cv_down_info.dec_vct = 0;
		return;
	}
	if (chip->dec_cv_down_info.dec_vct <= 0 && !spec->dec_cv.full.dec_init) {
		spec->dec_cv.full.vct_cur = oplus_gauge_get_fg_vct(chip->gauge_topic);

		while (cnts-- && (spec->dec_cv.full.vct_cur > DEC_VCT_UPDATE_MAX || spec->dec_cv.full.vct_cur < 0)) {
			spec->dec_cv.full.vct_cur = oplus_gauge_get_fg_vct(chip->gauge_topic);
		}

		if (spec->dec_cv.full.vct_cur > DEC_VCT_UPDATE_MAX || spec->dec_cv.full.vct_cur < 0)
			spec->dec_cv.full.vct_cur = 0;
		chip->dec_cv_down_info.dec_vct = spec->dec_cv.full.vct_cur;
		spec->dec_cv.full.dec_init = true;
		spec->dec_cv.dec_track = true;
	} else if (!spec->dec_cv.full.vct_cur && !spec->dec_cv.full.dec_init) {
		spec->dec_cv.full.dec_init = true;
		spec->dec_cv.full.vct_cur = chip->dec_cv_down_info.dec_vct;
	}
}

static void oplus_comm_get_dec_soh(struct oplus_chg_comm *chip)
{
	int dec_soh = 0;
	struct oplus_comm_spec_config *spec = &chip->spec;

	if (!spec->dec_cv.full.pack_type)
		spec->dec_cv.full.pack_type = oplus_gauge_get_dec_pack_type(chip->gauge_topic);

	dec_soh = oplus_gauge_get_dec_cv_soh(chip->gauge_topic);
	spec->dec_cv.full.dec_soh = spec->dec_cv.dbg_soh > 0 ? spec->dec_cv.dbg_soh : dec_soh;
	spec->dec_cv.full.aging_soh = false;
}

static void oplus_comm_get_full_index(struct oplus_chg_comm *chip)
{
	int i, cc;
	struct oplus_comm_spec_config *spec = &chip->spec;

	cc = spec->dec_cv.dbg_soh > 0 ? spec->dec_cv.dbg_soh : spec->dec_cv.full.dec_soh;
	for (i = DEC_VOL_CC_FULL_THR_COUNT - 1; i >= 0; i--) {
		if (cc >= spec->dec_cv.full.spec_cc_thr[i])
			break;
	}
	if (i < 0 || i >= DEC_VOL_CC_FULL_THR_COUNT)
		i = 0;

	spec->dec_cv.index = i;
}

static void oplus_comm_track_dec_cv(struct oplus_chg_comm *chip)
{
	struct oplus_comm_spec_config *spec = &chip->spec;

	if (spec->dec_cv.dec_track) {
		oplus_comm_update_dec_cv_down_info(chip);
		schedule_delayed_work(&chip->dec_vol_info_trigger_work, 0);
	}
}


#define DEC_VOL_UPDATE_CC_DELTA 20
#define DEC_VOL_UPDATE_SOH_DELTA 5
static void oplus_comm_get_dec_scv(struct oplus_chg_comm *chip)
{
	int last_cc = 0, cc = 0;
	int index, update_vol = 0, update_vct = 0, vol_fv = 0, vol_vct = 0;
	struct oplus_comm_spec_config *spec = &chip->spec;

	if (!spec->dec_cv.full.spec_step || !spec->dec_cv.full.pack_type)
		return;
	mutex_lock(&chip->dec_data_lock);
	cc = spec->dec_cv.dbg_soh > 0 ? spec->dec_cv.dbg_soh : spec->dec_cv.full.dec_soh;
	last_cc = chip->dec_cv_down_info.dec_cc;
	spec->dec_cv.full.last_cc = last_cc;
	index = spec->dec_cv.index;
	vol_fv = (spec->dec_cv.full.spec_fv_mv[index] - spec->dec_cv.dec_delta) > 0 ?
		(spec->dec_cv.full.spec_fv_mv[index] - spec->dec_cv.dec_delta) : 0;
	vol_vct = (spec->dec_cv.full.spec_vct_mv[index] - spec->dec_cv.dec_delta) > 0 ?
		(spec->dec_cv.full.spec_vct_mv[index] - spec->dec_cv.dec_delta) : 0;
	if (chip->dec_cv_down_info.dec_vol != vol_fv || chip->dec_cv_down_info.dec_vct != vol_vct) {
		if ((last_cc <= 0) || (!spec->dec_cv.full.aging_soh && cc >= (last_cc + DEC_VOL_UPDATE_CC_DELTA)) ||
			(spec->dec_cv.full.aging_soh && cc <= (last_cc - DEC_VOL_UPDATE_SOH_DELTA)) ||
			(chip->dec_cv_down_info.dec_vol <= 0) || (spec->dec_cv.full.vct_en && chip->dec_cv_down_info.dec_vct <= 0)) {
			chip->dec_cv_down_info.dec_cc = cc;
			spec->dec_cv.dec_track = true;
			if (vol_fv > chip->dec_cv_down_info.dec_vol)
				update_vol = (vol_fv - chip->dec_cv_down_info.dec_vol) > spec->dec_cv.full.spec_step ?
					(chip->dec_cv_down_info.dec_vol + spec->dec_cv.full.spec_step) : vol_fv;
			else
				update_vol = (chip->dec_cv_down_info.dec_vol - vol_fv) > spec->dec_cv.full.spec_step ?
					(chip->dec_cv_down_info.dec_vol - spec->dec_cv.full.spec_step) : vol_fv;

			if (vol_vct > chip->dec_cv_down_info.dec_vct)
				update_vct = (vol_vct - chip->dec_cv_down_info.dec_vct) > spec->dec_cv.full.spec_step ?
					(chip->dec_cv_down_info.dec_vct + spec->dec_cv.full.spec_step) : vol_vct;
			else
				update_vct = (chip->dec_cv_down_info.dec_vct - vol_vct) > spec->dec_cv.full.spec_step ?
					(chip->dec_cv_down_info.dec_vct - spec->dec_cv.full.spec_step) : vol_vct;
		} else if (((!spec->dec_cv.full.aging_soh && cc < last_cc) ||
			(spec->dec_cv.full.aging_soh && cc > last_cc)) || spec->dec_cv.full.del_status) {
			update_vol = vol_fv;
			update_vct = vol_vct;
			chip->dec_cv_down_info.dec_cc = cc;
			spec->dec_cv.full.del_status = false;
			spec->dec_cv.dec_track = true;
		} else {
			update_vol = chip->dec_cv_down_info.dec_vol;
			update_vct = chip->dec_cv_down_info.dec_vct;
		}
	} else {
		update_vol = chip->dec_cv_down_info.dec_vol;
		update_vct = chip->dec_cv_down_info.dec_vct;
	}

	chg_info(" [%d, %d, %d, %d, %d, %d][%d, %d, %d, %d, %d, %d, %d]\n",
		vol_fv, chip->dec_cv_down_info.dec_vol, vol_vct, chip->dec_cv_down_info.dec_vct, cc, last_cc,
		update_vol, update_vct, spec->dec_cv.dec_delta, spec->dec_cv.full.vct_cur,
		spec->dec_cv.dbg_soh, chip->batt_cc, spec->dec_cv.full.pack_type);

	update_vol = update_vol > DEC_VOL_UPDATE_MAX ? DEC_VOL_UPDATE_MAX : update_vol;
	update_vct = update_vct > DEC_VCT_UPDATE_MAX ? DEC_VCT_UPDATE_MAX : update_vct;
	spec->dec_cv.dec_vol = update_vol;
	chip->dec_cv_down_info.dec_vol = update_vol;
	if (spec->dec_cv.full.vct_en) {
		spec->dec_cv.full.vct_up = update_vct;
		chip->dec_cv_down_info.dec_vct = update_vct;
	}
	mutex_unlock(&chip->dec_data_lock);
	oplus_comm_track_dec_cv(chip);
}

static void oplus_comm_get_dec_fcv(struct oplus_chg_comm *chip)
{
	int cc = 0;
	int index, vol_fv = 0, vol_vct = 0;
	struct oplus_comm_spec_config *spec = &chip->spec;

	if (spec->dec_cv.full.spec_step || !spec->dec_cv.full.pack_type)
		return;

	mutex_lock(&chip->dec_data_lock);
	cc = spec->dec_cv.dbg_soh > 0 ? spec->dec_cv.dbg_soh : spec->dec_cv.full.dec_soh;
	index = spec->dec_cv.index;
	vol_fv = (spec->dec_cv.full.spec_fv_mv[index] - spec->dec_cv.dec_delta) > 0 ?
		(spec->dec_cv.full.spec_fv_mv[index] - spec->dec_cv.dec_delta) : 0;
	vol_vct = (spec->dec_cv.full.spec_vct_mv[index] - spec->dec_cv.dec_delta) > 0 ?
		(spec->dec_cv.full.spec_vct_mv[index] - spec->dec_cv.dec_delta) : 0;
	if (spec->dec_cv.dec_vol != vol_fv || spec->dec_cv.full.vct_up != vol_vct) {
		spec->dec_cv.full.del_status = false;
		spec->dec_cv.dec_track = true;
	}

	vol_fv = vol_fv > DEC_VOL_UPDATE_MAX ? DEC_VOL_UPDATE_MAX : vol_fv;
	vol_vct = vol_vct > DEC_VCT_UPDATE_MAX ? DEC_VCT_UPDATE_MAX : vol_vct;
	spec->dec_cv.dec_vol = vol_fv;
	if (spec->dec_cv.full.vct_en)
		spec->dec_cv.full.vct_up = vol_vct;

	mutex_unlock(&chip->dec_data_lock);
	oplus_comm_track_dec_cv(chip);
}

static void oplus_comm_set_dec_vct(struct oplus_chg_comm *chip)
{
	bool rc = false;
	int gauge_type = 0;
	struct oplus_comm_spec_config *spec = &chip->spec;

	if (!spec->dec_cv.full.vct_en || !spec->dec_cv.full.pack_type)
		return;

	gauge_type = oplus_get_gauge_type();
	if (spec->dec_cv.full.vct_cur != spec->dec_cv.full.vct_up ||
		(gauge_type == GAUGE_TYPE_PLATFORM && !spec->dec_cv.full.vct_boot_flag)) {
		rc = oplus_gauge_set_fg_vct(chip->gauge_topic, spec->dec_cv.full.vct_up);
		if (rc)
			spec->dec_cv.full.vct_cur = spec->dec_cv.full.vct_up;
		spec->dec_cv.full.vct_boot_flag = true;
	}
}

void oplus_comm_set_dec_delta(struct oplus_mms *topic, int val)
{
	struct oplus_chg_comm *chip;
#define DEC_VOL_DELTA_MAX 100

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return;
	}

	chg_info(" [%d]\n", val);
	if (val > DEC_VOL_DELTA_MAX || val < -DEC_VOL_DELTA_MAX)
		return;
	chip = oplus_mms_get_drvdata(topic);
	chip->spec.dec_cv.dec_delta = val;
	chip->spec.dec_cv.dec_track = true;
	chip->spec.dec_cv.full.del_status = true;
}

void oplus_comm_get_dec_delta(struct oplus_mms *topic, int *val)
{
	struct oplus_chg_comm *chip;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return;
	}

	chip = oplus_mms_get_drvdata(topic);
	if (!chip) {
		chg_err("oplus_chg_comm chip is NULL\n");
		return;
	}

	*val = chip->spec.dec_cv.dec_delta;
}

static int oplus_comm_get_lite_index(struct oplus_chg_comm *chip)
{
	struct oplus_comm_spec_config *spec = &chip->spec;
	int i;
	int cc;

	if (!spec->dec_cv.dec_spec_support)
		return 0;

	cc = spec->dec_cv.dbg_soh > 0 ? spec->dec_cv.dbg_soh : chip->batt_cc;
	for (i = DEC_VOL_CC_THR_COUNT - 1; i >= 0; i--) {
		if (cc >= spec->dec_cv.lite.dec_vol_cc_thr[i])
			return i;
	}

	return 0;
}

static void oplus_comm_dec_cv_lite(struct oplus_chg_comm *chip)
{
	int vol_fv = 0;
	struct oplus_comm_spec_config *spec = &chip->spec;

#define DEC_VOL_CV_MAX 150
	if (spec->dec_cv.dec_spec_support != DEC_CV_SUPPORT_LITE)
		return;

	spec->dec_cv.index = oplus_comm_get_lite_index(chip);

	vol_fv = max((spec->dec_cv.lite.dec_vol_fv_mv[spec->dec_cv.index][chip->temp_region] - spec->dec_cv.dec_delta), 0);
	vol_fv = min(vol_fv, DEC_VOL_CV_MAX);
	spec->dec_cv.dec_vol = vol_fv;

	schedule_delayed_work(&chip->dec_vol_info_trigger_work, 0);
}

static void oplus_comm_dec_cv_full(struct oplus_chg_comm *chip)
{
	struct oplus_comm_spec_config *spec = &chip->spec;
#define DEC_VOL_CC_MAX 5000

	if (spec->dec_cv.dec_spec_support != DEC_CV_SUPPORT_FULL)
		return;

	if (spec->dec_cv.full.spec_step && !chip->dec_cv_down_init)
		return;

	if (spec->dec_cv.dbg_soh < 0 || chip->batt_cc < 0 ||
		spec->dec_cv.dbg_soh > DEC_VOL_CC_MAX || chip->batt_cc > DEC_VOL_CC_MAX)
		return;

	oplus_comm_dec_vct_init(chip);
	oplus_comm_get_dec_soh(chip);
	oplus_comm_get_full_index(chip);
	oplus_comm_get_dec_fcv(chip);
	oplus_comm_get_dec_scv(chip);
	oplus_comm_set_dec_vct(chip);
}

static void oplus_comm_dec_cv_check(struct oplus_chg_comm *chip)
{
	struct oplus_comm_spec_config *spec = &chip->spec;

	if (!spec->dec_cv.dec_spec_support)
		return;

	oplus_comm_dec_cv_lite(chip);
	oplus_comm_dec_cv_full(chip);
}

static void oplus_comm_dec_cv_check_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_comm *chip =
		container_of(dwork, struct oplus_chg_comm, dec_cv_check_work);

	oplus_comm_dec_cv_check(chip);
}

static int oplus_comm_dec_cv_down_info_obtain_notifier_call(
		struct notifier_block *nb, unsigned long param, void *v)
{
	struct oplus_chg_comm *chip;
	struct dec_cv_down_load *dec_cv_down_info;
	struct oplus_chg_mutual_notifier *notifier;

	notifier = container_of(nb, struct oplus_chg_mutual_notifier, nb);
	chip = container_of(notifier, struct oplus_chg_comm, dec_cv_down_obtain_nb);

	if (mutual_info_to_cmd(param) != CMD_DEC_CV_DOWN_OBTAIN) {
		chg_err("cmd is not matching, should return\n");
		return NOTIFY_OK;
	}

	if (mutual_info_to_data_size(param) != sizeof(struct dec_cv_down_load)) {
		chg_err("data_len is not ok, datas is invalid\n");
		return NOTIFY_DONE;
	}

	dec_cv_down_info = (struct dec_cv_down_load *)v;
	if (dec_cv_down_info)
		memcpy(&chip->dec_cv_down_info, dec_cv_down_info, sizeof(struct dec_cv_down_load));

	chg_info("tag:%s, dec_cc:%d, dec_vol:%d, dec_vct:%d",
			chip->dec_cv_down_info.tag_info, chip->dec_cv_down_info.dec_cc,
			chip->dec_cv_down_info.dec_vol, chip->dec_cv_down_info.dec_vct);

	if (chip->dec_cv_down_init == false) {
		chip->dec_cv_down_init = true;
		schedule_delayed_work(&chip->dec_cv_check_work, 0);
	}
	return NOTIFY_OK;
}

static int oplus_comm_dec_cv_down_info_update_notifier_call(
		struct notifier_block *nb, unsigned long param, void *v)
{
	struct oplus_chg_comm *chip;
	struct oplus_chg_mutual_notifier *notifier;

	notifier = container_of(nb, struct oplus_chg_mutual_notifier, nb);
	chip = container_of(notifier, struct oplus_chg_comm, dec_cv_down_update_nb);

	if (mutual_info_to_cmd(param) != CMD_DEC_CV_DOWN_UPDATE) {
		chg_err("cmd is not matching, should return\n");
		return NOTIFY_OK;
	};

	return NOTIFY_OK;
}

static void oplus_comm_get_reserve_dec_cv_down_info_work(struct work_struct *work)
{
	int rc;
	static int try_count = 3; /*TODO: max try 3 counts*/
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_comm *chip =
		container_of(dwork, struct oplus_chg_comm, get_reserve_dec_cv_down_info_work);

	rc = oplus_chg_set_mutual_cmd(CMD_DEC_CV_DOWN_OBTAIN, 0, NULL);
	if (rc != CMD_ACK_OK && try_count--)
		schedule_delayed_work(&chip->get_reserve_dec_cv_down_info_work, msecs_to_jiffies(2000));
}

static void oplus_comm_set_reserve_dec_cv_down_info_work(struct work_struct *work)
{
	int rc;
	struct oplus_chg_comm *chip = container_of(work, struct oplus_chg_comm,
		set_reserve_dec_cv_down_info_work);

	rc = oplus_chg_set_mutual_cmd(CMD_DEC_CV_DOWN_UPDATE,
		sizeof(chip->dec_cv_down_info), &(chip->dec_cv_down_info));
	if (rc != CMD_ACK_OK)
		chg_info("fail\n");
}

static void oplus_comm_get_reserve_dec_cv_down_info(struct oplus_chg_comm *chip)
{
	static bool update = false;
	struct oplus_comm_spec_config *spec = &chip->spec;

	if (!spec->dec_cv.full.spec_step)
		return;

	if (!update) {
		update = true;
		schedule_delayed_work(&chip->get_reserve_dec_cv_down_info_work, msecs_to_jiffies(2000));
	}
}

static void oplus_comm_update_dec_cv_down_info(struct oplus_chg_comm *chip)
{
	chg_info("[%d, %d, %d, %d]\n",
		chip->dec_cv_down_init, chip->dec_cv_down_info.dec_cc,
		chip->dec_cv_down_info.dec_vol, chip->dec_cv_down_info.dec_vct);

	if (chip->dec_cv_down_init)
		queue_work(system_highpri_wq, &chip->set_reserve_dec_cv_down_info_work);
}

static int oplus_comm_dec_cv_down_info_obtain_init(struct oplus_chg_comm *chip)
{
	int rc = 0;

	chip->dec_cv_down_obtain_nb.name = "dec_cv_down_info_obtain";
	chip->dec_cv_down_obtain_nb.cmd = CMD_DEC_CV_DOWN_OBTAIN;
	chip->dec_cv_down_obtain_nb.nb.notifier_call = oplus_comm_dec_cv_down_info_obtain_notifier_call;
	rc = oplus_chg_reg_mutual_notifier(&chip->dec_cv_down_obtain_nb);
	if (rc < 0) {
		chg_err("register dec cv down obtain event notifier error, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int oplus_comm_dec_cv_down_info_update_init(struct oplus_chg_comm *chip)
{
	int rc = 0;

	chip->dec_cv_down_update_nb.name = "dec_cv_down_info_update";
	chip->dec_cv_down_update_nb.cmd = CMD_DEC_CV_DOWN_UPDATE;
	chip->dec_cv_down_update_nb.nb.notifier_call = oplus_comm_dec_cv_down_info_update_notifier_call;
	rc = oplus_chg_reg_mutual_notifier(&chip->dec_cv_down_update_nb);
	if (rc < 0) {
		chg_err("register dec cv down update event notifier error, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static void oplus_comm_check_rechg(struct oplus_chg_comm *chip)
{
	struct oplus_comm_spec_config *spec = &chip->spec;
	int vbatdet_mv;

	if (!chip->sw_full && !chip->hw_full_by_sw)
		return;

	if (chip->wired_online)
		vbatdet_mv = spec->wired_vbatdet_mv[chip->temp_region] -
			spec->dec_cv.dec_vol;
	else if (chip->wls_online)
		vbatdet_mv = spec->wls_vbatdet_mv[chip->temp_region] -
			spec->dec_cv.dec_vol;
	else
		return;
	if (!chip->authenticate || !chip->hmac)
		vbatdet_mv = spec->non_standard_vbatdet_mv;

	if (!chip->rechg_soc_en) {
		if (chip->vbat_mv <= vbatdet_mv &&
		    (is_support_parallel_battery(chip->gauge_topic) != SERIAL_CONNECT_TYPE ||
		     (is_support_parallel_battery(chip->gauge_topic) == SERIAL_CONNECT_TYPE &&
		      chip->sub_vbat_mv <= vbatdet_mv))) {
			chip->rechg_count++;
		} else if (chip->rechg_now) {
			chip->rechg_count = RECHG_COUNT_MAX + 1;
			chip->rechg_now = false;
		} else {
			chip->rechg_count = 0;
			return;
		}
	} else {
		/* when ui_soc rechg enabled:
		   little cold or cool or normal, check ui_soc only,
		   otherwise, check both ui_soc and vbatt.
		*/
		if (chip->ui_soc <= chip->rechg_soc) {
			if ((chip->temp_region > TEMP_REGION_COLD)
			    && (chip->temp_region < TEMP_REGION_WARM)
			    && chip->uisoc_down_in_full)
				chip->rechg_count++;
			else if (chip->vbat_mv_max <= vbatdet_mv)
				chip->rechg_count++;
			else
				chip->rechg_count = 0;
		} else {
			chip->rechg_count = 0;
			return;
		}
	}

	if (chip->rechg_count > RECHG_COUNT_MAX) {
		chg_info("rechg start\n");
		oplus_comm_fginfo_reset(chip);
		chip->rechg_count = 0;
		chip->sw_full = false;
		chip->hw_full_by_sw = false;
		chip->uisoc_down_in_full = false;
		if (is_support_parallel_battery(chip->gauge_topic)) {
			chip->sw_sub_batt_full = false;
			chip->hw_sub_batt_full_by_sw = false;
		}
		oplus_comm_set_rechging(chip, true);
		if (chip->wls_online) {
			if (is_wls_charging_disable_votable_available(chip)) {
				vote(chip->wls_charging_disable_votable, CHG_FULL_VOTER, false, 0, false);
				if (is_wls_icl_votable_available(chip))
					vote(chip->wls_icl_votable, CHG_FULL_VOTER, false, 0, true);
				if (is_wls_fastchg_disable_votable_available(chip) &&
				    is_wls_comu_votable_available(chip)) {
					vote(chip->wls_fastchg_disable_votable, CHG_FULL_VOTER, false, 0, false);
					vote(chip->wls_comu_votable, CHG_FULL_VOTER, false, 0, true);
				}
			} else {
				chg_err("wls_charging_disable_votable not found, can't enable charging");
			}
		} else {
			if (is_wired_charging_disable_votable_available(chip)) {
				vote(chip->wired_charging_disable_votable, CHG_FULL_VOTER, false, 0, false);
				vote(chip->wired_charging_disable_votable, FFC_VOTER, false, 0, false);
				vote(chip->wired_charging_disable_votable, FASTCHG_VOTER, false, 0, false);
			} else {
				chg_err("wired_charging_disable_votable not found, can't enable charging");
			}
		}

		/* ensure that max_chg_time_sec has been obtained */
		chip->need_start_timeout_work = true;
		oplus_comm_start_timeout_work(chip);
	}
}

#define VBAT_CNT	1
#define VBAT_OV_OFFSET	200
static void oplus_comm_check_vbatt_is_good(struct oplus_chg_comm *chip)
{
	struct oplus_comm_spec_config *spec = &chip->spec;
	int ov_thr;
	static int vbat_counts = 0;

	if (chip->vbatt_over)
		ov_thr = spec->vbatt_ov_thr_mv - VBAT_OV_OFFSET;
	else
		ov_thr = spec->vbatt_ov_thr_mv;

	if (chip->vbat_mv_max >= ov_thr && !chip->vbatt_over) {
		vbat_counts++;
		if (vbat_counts >= VBAT_CNT) {
			vbat_counts = 0;
			chip->vbatt_over = true;
			vote(chip->chg_disable_votable, UOVP_VOTER, true, 1,
			     false);
		}
	} else if (chip->vbat_mv_max < ov_thr && chip->vbatt_over) {
		vbat_counts = 0;
		chip->vbatt_over = false;
		vote(chip->chg_disable_votable, UOVP_VOTER, false, 0, false);
	} else {
		vbat_counts = 0;
	}
}

int oplus_comm_get_dis_ui_power_state(struct oplus_mms *topic)
{
	struct oplus_chg_comm *chip;
	struct oplus_comm_config *config;
	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return 0;
	}

	chip = oplus_mms_get_drvdata(topic);
	config = &chip->config;
	return config->vooc_dis_show_ui_power;
}

static void oplus_comm_charge_timeout_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_comm *chip = container_of(dwork,
		struct oplus_chg_comm, charge_timeout_work);

#ifdef SELL_MODE
	if (chip->chging_over_time)
		vote(chip->chg_disable_votable, TIMEOUT_VOTER, false, 0, false);
	chip->chging_over_time = false;
	chg_info("oplus_chg_check_time_is_good by sell_mode\n");
	return;
#endif /* SELL_MODE */

	if (!chip->wired_online && !chip->wls_online) {
		vote(chip->chg_disable_votable, TIMEOUT_VOTER, false, 0, false);
		chip->chging_over_time = false;
		return;
	}

	chip->chging_over_time = true;
	vote(chip->chg_disable_votable, TIMEOUT_VOTER, true, 1, false);
}

static bool oplus_comm_is_not_charging(struct oplus_chg_comm *chip)
{
	if (get_effective_result(chip->chg_disable_votable) != 0)
		return true;

	if (chip->wired_online &&
	    is_wired_charging_disable_votable_available(chip)) {
		if (get_client_vote(chip->wired_charging_disable_votable,
				    BATT_TEMP_VOTER) > 0)
			return true;
		if (get_client_vote(chip->wired_charging_disable_votable,
				    DEF_VOTER) > 0)
			return true;
		if (get_client_vote(chip->wired_charging_disable_votable,
				    UOVP_VOTER) > 0)
			return true;
	} else if (chip->wls_online &&
		   is_wls_charging_disable_votable_available(chip)) {
		if (get_client_vote(chip->wls_charging_disable_votable,
				    BATT_TEMP_VOTER) > 0)
			return true;
		if (get_client_vote(chip->wls_charging_disable_votable,
				    DEF_VOTER) > 0)
			return true;
	}

	return false;
}

static bool oplus_comm_is_discharging(struct oplus_chg_comm *chip)
{
	if (get_effective_result(chip->chg_suspend_votable) != 0)
		return true;
	if (chip->wired_online &&
	    is_wired_charge_suspend_votable_available(chip)) {
		if (get_client_vote(chip->wired_charge_suspend_votable,
				    UOVP_VOTER) > 0)
			return true;
		if (get_client_vote(chip->wired_charge_suspend_votable,
				    WLAN_VOTER) > 0)
			return true;
		if (get_client_vote(chip->wired_charge_suspend_votable,
				    DEF_VOTER) > 0)
			return true;
	} else if (chip->wls_online &&
		  is_wls_charge_suspend_votable_available(chip)) {
		if (get_client_vote(chip->wls_charge_suspend_votable,
				    UOVP_VOTER) > 0)
			return true;
		if (get_client_vote(chip->wls_charge_suspend_votable,
				    WLAN_VOTER) > 0)
			return true;
		if (get_client_vote(chip->wls_charge_suspend_votable,
				    DEF_VOTER) > 0)
			return true;
	}

	return false;
}

static int oplus_comm_get_mmi_state(struct oplus_chg_comm *chip)
{
	int mmi_chg = 1;

	if (!chip->chg_disable_votable)
		chip->chg_disable_votable = find_votable("CHG_DISABLE");
	if (!chip->chg_suspend_votable)
		chip->chg_suspend_votable = find_votable("CHG_SUSPEND");

	if (chip->chg_disable_votable)
		mmi_chg = ((!(get_client_vote(chip->chg_disable_votable, MMI_CHG_VOTER) && !chip->wls_online)) &&
		    (!get_client_vote(chip->chg_disable_votable, CHG_LIMIT_CHG_VOTER)));
	if (chip->chg_suspend_votable)
		mmi_chg = mmi_chg && (!get_client_vote(chip->chg_suspend_votable, CHG_LIMIT_CHG_VOTER));

	return mmi_chg;
}

static void oplus_comm_check_battery_status(struct oplus_chg_comm *chip)
{
	int batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	struct mms_msg *msg;
	int rc;
	int mmi_chg = 1;
	static unsigned long last_normal_jiffies = 0;
	unsigned long jiffies_diff = 0;

	mmi_chg = oplus_comm_get_mmi_state(chip);
	if (!chip->authenticate || !chip->batt_exist) {
		batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		goto check_done;
	}
	if (!chip->wired_online && !chip->wls_online) {
		batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
		goto check_done;
	}
	if (chip->batt_full && mmi_chg) {
		if ((chip->temp_region > TEMP_REGION_COLD) &&
		    (chip->temp_region < TEMP_REGION_WARM) &&
		    chip->hmac) {
			if (chip->ui_soc == 100) {
				batt_status = POWER_SUPPLY_STATUS_FULL;
				goto check_done;
			} else if (chip->uisoc_down_in_full) {
				batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
				goto check_done;
			}
		} else {
			batt_status = POWER_SUPPLY_STATUS_FULL;
			goto check_done;
		}
	}

	if (oplus_comm_is_discharging(chip))
		batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
	else if (oplus_comm_is_not_charging(chip))
		batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	else
		batt_status = POWER_SUPPLY_STATUS_CHARGING;

	if (get_client_vote(chip->chg_disable_votable, CHG_LIMIT_CHG_VOTER) > 0 ||
	    get_client_vote(chip->chg_suspend_votable, CHG_LIMIT_CHG_VOTER) > 0) {
		jiffies_diff = jiffies > last_normal_jiffies ? jiffies - last_normal_jiffies : last_normal_jiffies - jiffies;
		if (jiffies_diff < msecs_to_jiffies(5000))
			batt_status = POWER_SUPPLY_STATUS_CHARGING;
	} else {
		last_normal_jiffies = jiffies;
	}
check_done:
	if (chip->batt_status == batt_status)
		return;
	chip->batt_status = batt_status;
	chg_info("batt_status is %s\n", POWER_SUPPLY_STATUS_TEXT[batt_status]);
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  COMM_ITEM_BATT_STATUS);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish battery status msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_comm_check_battery_health(struct oplus_chg_comm *chip)
{
	int batt_health;
	struct mms_msg *msg;
	int rc;

	if (!chip->batt_exist) {
		batt_health = POWER_SUPPLY_HEALTH_DEAD;
	} else if (chip->vbatt_over) {
		batt_health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	} else if (chip->temp_region == TEMP_REGION_HOT) {
		batt_health = POWER_SUPPLY_HEALTH_OVERHEAT;
	} else if (chip->temp_region == TEMP_REGION_COLD) {
		batt_health = POWER_SUPPLY_HEALTH_COLD;
	} else {
		batt_health = POWER_SUPPLY_HEALTH_GOOD;
	}

	if (chip->batt_health == batt_health)
		return;
	chip->batt_health = batt_health;
	chg_info("batt_health is %s\n", POWER_SUPPLY_HEALTH_TEXT[batt_health]);
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  COMM_ITEM_BATT_HEALTH);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish battery health msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_comm_check_battery_charge_type(struct oplus_chg_comm *chip)
{
	struct oplus_comm_spec_config *spec = &chip->spec;
	int batt_chg_type;
	struct mms_msg *msg;
	int rc;

	if (!chip->wired_online && !chip->wls_online) {
		batt_chg_type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		goto check_done;
	}
	if (chip->batt_status != POWER_SUPPLY_STATUS_CHARGING) {
		batt_chg_type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		goto check_done;
	}
	if (chip->vooc_charging || chip->ufcs_charging || chip->pps_charging ||
	    chip->ffc_status != FFC_DEFAULT) {
		batt_chg_type = POWER_SUPPLY_CHARGE_TYPE_FAST;
		goto check_done;
	}
	if (chip->vbat_mv_max >= (spec->fv_mv[chip->temp_region] - spec->dec_cv.dec_vol)) {
		batt_chg_type = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	} else {
		if (chip->batt_chg_type == POWER_SUPPLY_CHARGE_TYPE_TRICKLE)
			return;
		else
			batt_chg_type = POWER_SUPPLY_CHARGE_TYPE_FAST;
	}

check_done:
	if (chip->batt_chg_type == batt_chg_type)
		return;
	chip->batt_chg_type = batt_chg_type;
	chg_info("batt_chg_type is %s\n",
		POWER_SUPPLY_CHARGE_TYPE_TEXT[chip->batt_chg_type]);
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  COMM_ITEM_BATT_CHG_TYPE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish battery charge type msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static int oplus_comm_push_ui_soc_shutdown_msg(struct oplus_chg_comm *chip)
{
	struct oplus_mms *err_topic;
	struct mms_msg *msg;
	int rc;

	err_topic = oplus_mms_get_by_name("error");
	if (!err_topic) {
		chg_err("error topic not found\n");
		return -ENODEV;
	}

	msg = oplus_mms_alloc_int_msg(
		MSG_TYPE_ITEM, MSG_PRIO_HIGH, ERR_ITEM_UI_SOC_SHUTDOWN,
		0);
	if (msg == NULL) {
		chg_err("alloc ui_soc_shutdown msg error\n");
		return -ENOMEM;
	}

	/*
	 * the phone is about to be turned off, and need to ensure
	 * that the message is delivered
	 */
	rc = oplus_mms_publish_msg_sync(err_topic, msg);
	if (rc < 0) {
		chg_err("publish ui_soc_shutdown msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static int oplus_comm_publish_ui_soc(struct oplus_chg_comm *chip)
{
	struct mms_msg *msg;
	int rc;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  COMM_ITEM_UI_SOC);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish ui soc msg error, rc=%d\n", rc);
		kfree(msg);
		return rc;
	}

	return 0;
}

static int oplus_comm_set_ui_soc(struct oplus_chg_comm *chip, int soc)
{
	if (g_ui_soc_ready == false) {
		chg_err("g_ui_soc_ready is false, ui_soc = %d", chip->ui_soc);
		return 0;
	}

	if (chip->ui_soc == soc)
		return 0;
	chip->ui_soc = soc;

	chg_info("set ui_soc=%d\n", soc);
	oplus_comm_update_soc_jiffies(chip);
	chip->batt_full_jiffies = jiffies;

	if (soc == 0)
		oplus_comm_push_ui_soc_shutdown_msg(chip);

	oplus_comm_publish_ui_soc(chip);
	return 0;
}

static int oplus_comm_set_smooth_soc(struct oplus_chg_comm *chip, int soc)
{
	struct mms_msg *msg;
	int rc;

	if (chip->smooth_soc == soc)
		return 0;

	chip->smooth_soc = soc;
	chg_info("set smooth_soc=%d\n", soc);

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  COMM_ITEM_SMOOTH_SOC);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish smooth soc msg error, rc=%d\n", rc);
		kfree(msg);
		return rc;
	}

	return 0;
}

static int oplus_comm_set_delta_soc(struct oplus_chg_comm *chip, int delta_soc)
{
	struct mms_msg *msg;
	int rc;

	if (chip->delta_soc == delta_soc)
		return 0;

	chip->delta_soc = delta_soc;
	chg_info("set delta_soc=%d\n", delta_soc);

	msg = oplus_mms_alloc_int_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, COMM_ITEM_DELTA_SOC, chip->delta_soc);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish delta soc msg error, rc=%d\n", rc);
		kfree(msg);
		return rc;
	}

	return 0;
}

#define TARGET_UI_SOC_DROP_TO_2		(2)
#define TARGET_UI_SOC_DROP_TO_1		(1)
static int oplus_comm_push_uisoc_drop_msg(struct oplus_chg_comm *chip,  int err, int term_volt, int load_current,
    int drop_time, char end_reason[])
{
	struct oplus_mms *err_topic;
	struct mms_msg *msg;
	static int pre_uisoc_low_power_drop_err = 0;
	int rc;
	static int Vterm = 0;
	static int start_UISoc = 0;
	static int start_Soc = 0;
	static int start_volt = 0;
	static int start_voltmin = 0;
	static int start_battemp = 0;
	static int start_curr = 0;
	static int time_uisoc = 0;
	static int target_uisoc = 0;
	static int avg_current = 0;
	static unsigned long start_utc_t = 0;

	if (pre_uisoc_low_power_drop_err == err)
		return 0;

	err_topic = oplus_mms_get_by_name("error");
	if (!err_topic) {
		chg_err("error topic not found\n");
		return -ENODEV;
	}

	chg_info("set uisoc_low_power_drop_err=%d\n", err);
	pre_uisoc_low_power_drop_err = err;
	if (err == 0) {
		msg = oplus_mms_alloc_str_msg(
		    MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, COMM_ITEM_UISOC_DROP_ERROR,
		    "$$Vterm@@%d$$start_UISoc@@%d$$start_Soc@@%d$$start_volt@@%d$$start_voltmin@@%d$$start_battemp@@%d"
		    "$$start_curr@@%d$$time_uisoc@@%d$$target_uisoc@@%d$$avg_current@@%d$$end_UISoc@@%d$$end_Soc@@%d"
		    "$$end_volt@@%d$$end_voltmin@@%d$$end_battemp@@%d$$end_curr@@%d$$start_utc_t@@%lu$$end_utc_t@@%lu$$cc@@%d"
		    "$$end_reason@@%s",
		    Vterm, start_UISoc, start_Soc, start_volt, start_voltmin, start_battemp, start_curr, time_uisoc,
		    target_uisoc, avg_current, chip->ui_soc, chip->soc, chip->vbat_mv, chip->vbat_min_mv, chip->batt_temp,
		    chip->ibat_ma, start_utc_t, (jiffies / msecs_to_jiffies(1000)), chip->batt_cc, end_reason);
		Vterm = 0;
		start_UISoc = 0;
		start_Soc = 0;
		start_volt = 0;
		start_voltmin = 0;
		start_battemp = 0;
		start_curr = 0;
		time_uisoc = 0;
		target_uisoc = 0;
		avg_current = 0;
		start_utc_t = 0;
	} else if (err == TARGET_UI_SOC_DROP_TO_1 || err == TARGET_UI_SOC_DROP_TO_2) {
		Vterm = term_volt;
		start_UISoc = chip->ui_soc;
		start_Soc = chip->soc;
		start_volt = chip->vbat_mv;
		start_voltmin = chip->vbat_min_mv;
		start_battemp = chip->batt_temp;
		start_curr = chip->ibat_ma;
		time_uisoc = drop_time;
		target_uisoc = err;
		avg_current = load_current;
		start_utc_t = (jiffies / msecs_to_jiffies(1000));
		return 0;
	} else {
		return 0;
	}

	if (msg == NULL) {
		chg_err("alloc usbtemp error msg error\n");
		return -ENOMEM;
	}

	rc = oplus_mms_publish_msg(err_topic, msg);
	if (rc < 0) {
		chg_err("publish uisoc_low_power_drop_err msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static int oplus_comm_set_uisoc_keep_2_error(struct oplus_chg_comm *chip, int err)
{
	struct mms_msg *msg;
	int rc;

	if (chip->uisoc_keep_2_err == err)
		return 0;

	chip->uisoc_keep_2_err = err;
	chg_info("set uisoc_keep_2_err=%d\n", err);

	msg = oplus_mms_alloc_int_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, COMM_ITEM_UISOC_KEEP_2_ERROR, err);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish uisoc_keep_2_err msg error, rc=%d\n", rc);
		kfree(msg);
		return rc;
	}

	return 0;
}

static int oplus_comm_set_uisoc_keep_3_error(struct oplus_chg_comm *chip, int err)
{
	struct mms_msg *msg;
	int rc;

	if (chip->uisoc_keep_3_err == err)
		return 0;

	chip->uisoc_keep_3_err = err;
	chg_info("set uisoc_keep_3_err=%d\n", err);

	msg = oplus_mms_alloc_int_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, COMM_ITEM_UISOC_KEEP_3_ERROR, err);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish uisoc_keep_3_err msg error, rc=%d\n", rc);
		kfree(msg);
		return rc;
	}

	return 0;
}

static int oplus_comm_push_vbat_too_low_msg(struct oplus_chg_comm *chip)
{
	struct oplus_mms *err_topic;
	struct mms_msg *msg;
	int rc;

	err_topic = oplus_mms_get_by_name("error");
	if (!err_topic) {
		chg_err("error topic not found\n");
		return -ENODEV;
	}

	msg = oplus_mms_alloc_str_msg(
		MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_VBAT_TOO_LOW,
		"$$soc@@%d$$smooth_soc@@%d$$uisoc@@%d$$vbatt_max@@%d$$vbatt_min@@%d",
		chip->soc, chip->smooth_soc, chip->ui_soc, chip->vbat_mv, chip->vbat_min_mv);
	if (msg == NULL) {
		chg_err("alloc usbtemp error msg error\n");
		return -ENOMEM;
	}

	rc = oplus_mms_publish_msg(err_topic, msg);
	if (rc < 0) {
		chg_err("publish usbtemp error msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static void oplus_hidden_soc_smooth(struct oplus_chg_comm *chip)
{
	int soc = chip->soc;
	int temp_soc = soc;
	int smooth_soc = soc;
	int real_soc = soc;
	int reserve_soc = chip->config.reserve_soc;
	int hidden_soc_percent = chip->config.hidden_soc_percent;
	int hidden_soc_coefficient = (hidden_soc_percent - reserve_soc) * 100 / hidden_soc_percent;
	int hidden_soc_smooth = hidden_soc_percent - reserve_soc + 1;
	union mms_msg_data data = { 0 };

	if (chip->gauge_topic != NULL) {
		if (chip->wired_online || chip->wls_online)
			oplus_mms_topic_update(chip->gauge_topic, false);
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_RM, &data, false);
		chip->batt_rm = data.intval;
		if (chip->batt_rm < 0)
			chip->batt_rm = 0;

		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_FCC, &data, false);
		chip->batt_fcc = data.intval;
		if (chip->batt_fcc == 0) {
			chg_err("batt_fcc is 0\n");
			chip->batt_fcc = 4500; /* default set to 4500mAh */
		}
	}

	if (!chip->config.hidden_soc_switch || reserve_soc < RESERVE_SOC_MIN ||
		reserve_soc > RESERVE_SOC_MAX || hidden_soc_percent < HIDDEN_SOC_PERCENT_MIN ||
		hidden_soc_percent > HIDDEN_SOC_PERCENT_MAX) {
		goto reserve_soc_error;
	}

	if (soc < hidden_soc_smooth) {
		temp_soc = ((chip->batt_rm * 10000 / chip->batt_fcc) / hidden_soc_coefficient) + 1;
		real_soc = (chip->batt_rm * 100 / chip->batt_fcc) + 1;
		chg_info("soc =%d temp_soc =%d real_soc =%d \r\n", soc, temp_soc, real_soc);
		if (abs(soc - real_soc) > 1)
			temp_soc = soc;

		if (soc == POWER_OFF_SOC)
			temp_soc = POWER_OFF_SOC;
	}
	if (soc >= hidden_soc_smooth)
		temp_soc = soc + reserve_soc;

	chg_info("soc[%d %d %d %d %d %d %d]\n", soc, chip->soc, temp_soc, chip->config.reserve_soc,
		hidden_soc_smooth, hidden_soc_coefficient, hidden_soc_percent);
	smooth_soc = (temp_soc <= OPLUS_FULL_SOC) ? temp_soc : OPLUS_FULL_SOC;
	oplus_comm_set_smooth_soc(chip, smooth_soc);
	return;

reserve_soc_error:
	chip->config.hidden_soc_switch = false;
	oplus_comm_set_smooth_soc(chip, chip->soc);
}

static const int ui_soc_smooth_table[RESERVE_SOC_MAX + 1][RESERVE_SOC_MAX] = {
	{ -1, -1, -1, -1, -1 }, /* reserve 0 */
	{ 55, -1, -1, -1, -1 }, /* reserve 1 */
	{ 36, 71, -1, -1, -1 }, /* reserve 2 */
	{ 29, 53, 74, -1, -1 }, /* reserve 3 */
	{ 25, 43, 60, 78, -1 }, /* reserve 4 */
	{ 22, 36, 50, 64, 78 }, /* reserve 5 */
};

static void oplus_comm_smooth_to_soc(struct oplus_chg_comm *chip, bool force)
{
	static int pre_soc = -EINVAL;
	static unsigned long smooth_update_jiffies = 0;
	int soc = chip->soc;
	int reserve_soc = chip->config.reserve_soc;
	int temp_soc = soc;
	int sum = 0, cnt, index, i;
	int valid_soc_cnt = 0;
	int smooth_soc = soc;

	if (!chip->config.smooth_switch)
		goto reserve_soc_error;

	if (reserve_soc < RESERVE_SOC_MIN || reserve_soc > RESERVE_SOC_MAX) {
		chg_err("reserve_soc %d invalid\n", reserve_soc);
		goto reserve_soc_error;
	}

	if (force || pre_soc == -EINVAL) { /* force or first entry */
		chip->rsd.is_soc_jump_range = false;
		chip->rsd.smooth_soc_avg_cnt = SMOOTH_SOC_MIN_FIFO_LEN;
		for (i = 0; i < SMOOTH_SOC_MAX_FIFO_LEN; i++)
			chip->rsd.smooth_soc_fifo[i] = -EINVAL;
		chip->rsd.smooth_soc_index = 0;
		smooth_update_jiffies = jiffies + (unsigned long)(9 * HZ / 2);
	} else if ((pre_soc == soc) && (time_is_after_jiffies(smooth_update_jiffies))) {
		chg_debug("current_time is less than update_time %ums\n",
			jiffies_to_msecs(smooth_update_jiffies - jiffies));
		return;
	} else {
		smooth_update_jiffies = jiffies + (unsigned long)(9 * HZ / 2);
	}

	for (i = reserve_soc - 1; i >= 0; i--) {
		if (chip->ui_soc_smooth_table[reserve_soc][i] < 0) {
			chg_err("ui_soc_smooth_table invalid, please check it.\n");
			goto reserve_soc_error;
		}

		if (soc >= chip->ui_soc_smooth_table[reserve_soc][i]) {
			temp_soc = soc + i + 1;
			break;
		}
	}

	temp_soc = (temp_soc <= OPLUS_FULL_SOC) ? temp_soc : OPLUS_FULL_SOC;
	chip->rsd.smooth_soc_fifo[chip->rsd.smooth_soc_index] = temp_soc;

	if (force || pre_soc != soc) {
		chip->rsd.is_soc_jump_range = false;
		if (abs(pre_soc - soc) > 1)
			chip->rsd.smooth_soc_avg_cnt = SMOOTH_SOC_MIN_FIFO_LEN;

		for (i = 0; i < reserve_soc; i++) {
			if (soc >= (chip->ui_soc_smooth_table[reserve_soc][i] - SOC_JUMP_RANGE_VAL) &&
			    soc <= (chip->ui_soc_smooth_table[reserve_soc][i] + SOC_JUMP_RANGE_VAL)) {
				chg_debug("soc:%d index:%d soc_jump:%d\n", soc, i, chip->ui_soc_smooth_table[reserve_soc][i]);
				chip->rsd.is_soc_jump_range = true;
				chip->rsd.smooth_soc_avg_cnt = SMOOTH_SOC_MAX_FIFO_LEN;
				break;
			}
		}
	}

	if (chip->rsd.smooth_soc_avg_cnt == 1) {
		smooth_soc = chip->rsd.smooth_soc_fifo[chip->rsd.smooth_soc_index];
	} else {
		cnt = chip->rsd.smooth_soc_avg_cnt;
		index = chip->rsd.smooth_soc_index;
		while (cnt--) {
			if (chip->rsd.smooth_soc_fifo[index] > 0) {
				valid_soc_cnt++;
				sum += chip->rsd.smooth_soc_fifo[index];
			}
			index = (index + SMOOTH_SOC_MAX_FIFO_LEN - 1) % SMOOTH_SOC_MAX_FIFO_LEN;
		}

		if (valid_soc_cnt > 0)
			smooth_soc = sum / valid_soc_cnt;
		else
			smooth_soc = chip->rsd.smooth_soc_fifo[chip->rsd.smooth_soc_index];
	}

	chg_info("soc[%d %d %d %d %d %d] avg[%d %d %d %d %d] fifo[%d %d %d %d]\n", chip->rsd.rus_reserve_soc,
		reserve_soc, pre_soc, soc, smooth_soc, chip->ui_soc, chip->rsd.smooth_soc_index,
		chip->rsd.smooth_soc_avg_cnt, valid_soc_cnt, sum, chip->rsd.is_soc_jump_range,
		chip->rsd.smooth_soc_fifo[0], chip->rsd.smooth_soc_fifo[1], chip->rsd.smooth_soc_fifo[2],
		chip->rsd.smooth_soc_fifo[3]);

	pre_soc = soc;
	chip->rsd.smooth_soc_index++;
	chip->rsd.smooth_soc_index = chip->rsd.smooth_soc_index % SMOOTH_SOC_MAX_FIFO_LEN;
	if (!chip->rsd.is_soc_jump_range && chip->rsd.smooth_soc_avg_cnt > SMOOTH_SOC_MIN_FIFO_LEN)
		chip->rsd.smooth_soc_avg_cnt--;
	oplus_comm_set_smooth_soc(chip, smooth_soc);
	return;

reserve_soc_error:
	chip->config.smooth_switch = false;
	oplus_comm_set_smooth_soc(chip, chip->soc);
}

#define UI_SOC_DEC_SPEED_OF_UV_BATT		15
#define UI_SOC_DEC_SPEED_OF_LOW_BATT		60
#define DELAY_OF_LOW_BATT_CONTROL_ENABLE	(10*1000)
#define DELAY_OF_ONCE_LOW_BATT_CONTROL	(2500)
#define TIMES_OF_LOW_BATT_CONTROL_ENABLE	(DELAY_OF_LOW_BATT_CONTROL_ENABLE / DELAY_OF_ONCE_LOW_BATT_CONTROL)
#define SEC_OF_ONE_HOUR		(3600)
#define UI_SOC_LOW_LIMIT		(3)

static void oplus_comm_parse_uv_dec_speed_tbl(struct oplus_chg_comm *comm_dev,
						    struct device_node *node)
{
	const char *prop = "oplus_spec,uv_dec_speed_tbl";
	int elems, rc, i;

	elems = of_property_count_elems_of_size(node, prop, sizeof(u32));
	if (elems <= 0 || (elems % UV_DEC_SPEED_ENTRY_U32_CNT)) {
		chg_err("%s invalid elems=%d\n", prop, elems);
		return;
	}

	comm_dev->uv_dec_speed_tbl_cnt = elems / UV_DEC_SPEED_ENTRY_U32_CNT;
	comm_dev->uv_dec_speed_tbl = devm_kcalloc(comm_dev->dev, comm_dev->uv_dec_speed_tbl_cnt,
						  sizeof(*comm_dev->uv_dec_speed_tbl), GFP_KERNEL);
	if (!comm_dev->uv_dec_speed_tbl) {
		chg_err("alloc uv_dec_speed_tbl fail, cnt=%d\n", comm_dev->uv_dec_speed_tbl_cnt);
		comm_dev->uv_dec_speed_tbl_cnt = 0;
		return;
	}

	rc = of_property_read_u32_array(node, prop, (u32 *)comm_dev->uv_dec_speed_tbl, elems);
	if (rc < 0) {
		chg_err("read %s fail, rc=%d\n", prop, rc);
		goto out_free;
	}

	for (i = 0; i < comm_dev->uv_dec_speed_tbl_cnt; i++) {
		if (comm_dev->uv_dec_speed_tbl[i].speed <= 0) {
			chg_err("invalid speed, i=%d speed=%u\n",
				i, comm_dev->uv_dec_speed_tbl[i].speed);
			goto out_free;
		}
		if (i > 0 && comm_dev->uv_dec_speed_tbl[i].temp <= comm_dev->uv_dec_speed_tbl[i - 1].temp) {
			chg_err("temp not increasing, i=%d temp=%d prev_temp=%d\n",
				i, comm_dev->uv_dec_speed_tbl[i].temp,
				comm_dev->uv_dec_speed_tbl[i - 1].temp);
			goto out_free;
		}
	}

	return;

out_free:
	devm_kfree(comm_dev->dev, comm_dev->uv_dec_speed_tbl);
	comm_dev->uv_dec_speed_tbl = NULL;
	comm_dev->uv_dec_speed_tbl_cnt = 0;
}

static unsigned int oplus_comm_get_uv_dec_speed(struct oplus_chg_comm *chip)
{
	int i;

	if (!chip->uv_dec_speed_tbl || chip->uv_dec_speed_tbl_cnt <= 0)
		return UI_SOC_DEC_SPEED_OF_UV_BATT;

	for (i = 0; i < chip->uv_dec_speed_tbl_cnt - 1; i++) {
		if (chip->shell_temp < chip->uv_dec_speed_tbl[i].temp)
			return chip->uv_dec_speed_tbl[i].speed;
	}

	return chip->uv_dec_speed_tbl[chip->uv_dec_speed_tbl_cnt - 1].speed;
}

static unsigned long oplus_comm_ui_soc_low_battery_control(struct oplus_chg_comm *chip, unsigned long soc_down_jiffies,
	int vbat_min, bool *p_force_down_1)
{
	int dex = 0;
	static int vbat_low_soc_to_1 = 0;
	static int vbat_low_soc_to_2 = 0;
	int back_rm = 0;
	int load_current = 0;
	int t_sum = 0;
	static unsigned int t_soc_x_to_2 = 0;
	static unsigned int t_soc_x_to_1 = 0;
	int volt_diff_of_soc_2 = 50;
	static unsigned long last_jiffies = 0;
	unsigned long jiffies_diff = 0;
	int term_voltage = 0;
	int ui_soc;
	bool charging = chip->wired_online || chip->wls_online;

	if (p_force_down_1 == NULL)
		return soc_down_jiffies;

	jiffies_diff = jiffies > last_jiffies ? jiffies - last_jiffies : last_jiffies - jiffies;

	if ((jiffies_diff > msecs_to_jiffies(DELAY_OF_ONCE_LOW_BATT_CONTROL) ||
	    vbat_low_soc_to_2 == TIMES_OF_LOW_BATT_CONTROL_ENABLE ||
	    vbat_low_soc_to_1 == TIMES_OF_LOW_BATT_CONTROL_ENABLE)) {

		last_jiffies = jiffies;
		ui_soc = chip->ui_soc;
		if (is_gauge_term_voltage_votable_available(chip))
			term_voltage = get_effective_result(chip->gauge_term_voltage_votable);

		/* When the voltage is less than or equal to soc 2%,
		 * and the ui_soc is greater than 2%, it will be quickly smoothed to 2%.
		 */
		for (dex = 0; dex < LOW_VOLT_UISOC_LADDER_NUMBER - 1; dex++) {
			if (chip->config.temp_ladder_of_drop_soc_2[dex] == 0)
				break;
			if (chip->shell_temp < chip->config.temp_ladder_of_drop_soc_2[dex]) {
				volt_diff_of_soc_2 = chip->config.volt_diff_ladder_of_drop_soc_2[dex];
				break;
			} else {
				volt_diff_of_soc_2 = chip->config.volt_diff_ladder_of_drop_soc_2[dex+1];
			}
		}

		if ((vbat_min < term_voltage + volt_diff_of_soc_2) && (charging == false) &&
		    (chip->ibat_ma < chip->config.current_limit_of_drop_soc_2) && (ui_soc >= UI_SOC_LOW_LIMIT)) {
			vbat_low_soc_to_2 ++;
			if (vbat_low_soc_to_2 > TIMES_OF_LOW_BATT_CONTROL_ENABLE) {
				vbat_low_soc_to_2 = TIMES_OF_LOW_BATT_CONTROL_ENABLE;

				if (t_soc_x_to_2 == 0) {
					back_rm = chip->config.back_rm_of_drop_soc_2;
					load_current = chip->config.load_current_of_drop_soc_2;

					t_sum = back_rm * SEC_OF_ONE_HOUR / load_current;
					t_soc_x_to_2 = t_sum / (ui_soc - 2);
					chg_err("ui_soc x to 2 smooth soc:%d temp:%d %d vbat_min:%d, term_v:%d %d jiff_n:%ld jiff:%ld t_soc:%d %d %d %d",
					    ui_soc, chip->shell_temp, volt_diff_of_soc_2, vbat_min,
					    term_voltage, vbat_low_soc_to_2, soc_down_jiffies, chip->soc_down_update_jiffies,
					    t_soc_x_to_2, back_rm, load_current, chip->ibat_ma);
				}

				if (t_soc_x_to_2 < UI_SOC_DEC_SPEED_OF_LOW_BATT)
					t_soc_x_to_2 = UI_SOC_DEC_SPEED_OF_LOW_BATT;
				soc_down_jiffies = chip->soc_down_update_jiffies + (unsigned long)(t_soc_x_to_2 * HZ);
				*p_force_down_1 = true;
			}
		} else {
			vbat_low_soc_to_2 = 0;
			t_soc_x_to_2 = 0;
			*p_force_down_1 = false;
		}

		/* Solve the ui_soc 5% jump to 0% problem. When the actual soc is 0%,
		 * take out 1% in Power saving mode to smooth the ui_soc.
		 */
		if ((vbat_min < term_voltage && (chip->smooth_soc == 0)) && (ui_soc > 1) && (charging == false)) {
			vbat_low_soc_to_1 ++;
			if (vbat_low_soc_to_1 > TIMES_OF_LOW_BATT_CONTROL_ENABLE) {
				vbat_low_soc_to_1 = TIMES_OF_LOW_BATT_CONTROL_ENABLE;

				if (t_soc_x_to_1 == 0) {
					back_rm = chip->config.back_rm_of_drop_soc_1;
					load_current = chip->config.load_current_of_drop_soc_1;

					t_sum = back_rm * SEC_OF_ONE_HOUR / load_current;
					t_soc_x_to_1 = t_sum / (ui_soc - 1);
					chg_err("ui_soc x to 1 smooth soc:%d temp:%d %d vbat_min:%d, term_v:%d %d jiff_n:%ld jiff:%ld t_soc:%d %d %d %d\n",
					    ui_soc, chip->shell_temp, volt_diff_of_soc_2, vbat_min,
					    term_voltage, vbat_low_soc_to_1, soc_down_jiffies, chip->soc_down_update_jiffies,
					    t_soc_x_to_1, *p_force_down_1, back_rm, load_current);
				}

				t_soc_x_to_1 = max(t_soc_x_to_1, oplus_comm_get_uv_dec_speed(chip));
				soc_down_jiffies = chip->soc_down_update_jiffies + (unsigned long)(t_soc_x_to_1 * HZ);
			}
		} else {
			vbat_low_soc_to_1 = 0;
			t_soc_x_to_1 = 0;
		}

		if (t_soc_x_to_1 > 0)
			oplus_comm_push_uisoc_drop_msg(chip, TARGET_UI_SOC_DROP_TO_1, term_voltage, load_current, t_soc_x_to_1, "normal");
		else if (t_soc_x_to_2 > 0)
			oplus_comm_push_uisoc_drop_msg(chip, TARGET_UI_SOC_DROP_TO_2, term_voltage, load_current, t_soc_x_to_2, "normal");
		else
			oplus_comm_push_uisoc_drop_msg(chip, 0, 0, 0, 0, "normal");
	}
	return soc_down_jiffies;
}

static bool oplus_comm_is_under_uv(struct oplus_chg_comm *chip)
{
	struct oplus_comm_spec_config *spec = &chip->spec;
	struct oplus_comm_config *config = &chip->config;
	bool charging;
	int charging_uv_thr_mv = spec->vbat_charging_uv_thr_mv;

	if (chip->deep_support && config->chg_shutdown_max_mv != -EINVAL)
		charging_uv_thr_mv = min(charging_uv_thr_mv, config->chg_shutdown_max_mv);

	charging = chip->wired_online || chip->wls_online;

	if (chip->vbat_min_mv < (charging ? charging_uv_thr_mv : spec->vbat_uv_thr_mv) ||
	    (is_support_parallel_battery(chip->gauge_topic) &&
	     spec->sub_vbat_charging_uv_thr_mv > 0 && spec->sub_vbat_uv_thr_mv > 0 &&
	     chip->sub_vbat_mv > 2500 &&
	     chip->sub_vbat_mv <
	     (charging ? spec->sub_vbat_charging_uv_thr_mv : spec->sub_vbat_uv_thr_mv)))
		return true;
	else
		return false;
}

static void oplus_comm_gauge_r_info_check(struct oplus_chg_comm *chip)
{
	struct oplus_comm_spec_config *spec = &chip->spec;
	int vbat_uv_thr_mv = spec->vbat_uv_thr_mv + GAUGE_VBAT_UV_DELATA;
	static int pre_soc = 0;

	if (!chip->config.support_gauge_r_track)
		return;

	if (oplus_gauge_r_data > 0 || (pre_soc > 0 && chip->ui_soc >= 2 && chip->soc == 0 && chip->vbat_min_mv > vbat_uv_thr_mv))
		schedule_delayed_work(&chip->gauge_r_info_work, 0);

	oplus_gauge_r_data = 0;
	pre_soc = chip->soc;
}

/**
 * oplus_comm_get_time_diff_for_current_calc - Get time difference for current calculation
 * @chip: oplus_chg_comm pointer
 * @time_diff_sec: output parameter for time difference in seconds
 * @now_tm_sec: output parameter for current time in seconds
 *
 * Returns 0 on success, negative error code on failure.
 * On success, also updates chip->passed_q_save_time for next use.
 */
static int oplus_comm_get_time_diff_for_current_calc(struct oplus_chg_comm *chip,
						      unsigned long *time_diff_sec,
						      unsigned long *now_tm_sec)
{
	int rc;

	if (chip == NULL || time_diff_sec == NULL || now_tm_sec == NULL)
		return -EINVAL;

	/* Get current real time */
	rc = oplus_comm_get_current_time(now_tm_sec);
	if (rc < 0 || chip->passed_q_save_time == 0) {
		/* If get time failed or save_time is not set, use default time */
		*time_diff_sec = UV_ADJUSTED_CHECK_TIME;
		chg_info("use default time %lu sec for current calculation\n", *time_diff_sec);
	} else {
		/* Calculate actual time difference */
		if (*now_tm_sec > chip->passed_q_save_time) {
			*time_diff_sec = *now_tm_sec - chip->passed_q_save_time;
		} else {
			/* Time went backwards or invalid, use default */
			*time_diff_sec = UV_ADJUSTED_CHECK_TIME;
			chg_info("invalid time diff, use default time %lu sec\n", *time_diff_sec);
		}
		if (*time_diff_sec < 1)
			*time_diff_sec = 1;
	}

	/* Update passed_q_save_time after calculation for next use */
	if (rc >= 0)
		chip->passed_q_save_time = *now_tm_sec;

	return 0;
}

/**
 * oplus_comm_calc_vbat_uv_by_current - Calculate and adjust vbat_uv_thr_mv based on current
 * @chip: oplus_chg_comm pointer
 * @base_vbat_uv_thr_mv: base vbat uv threshold in mV
 * @vbat_uv_thr_mv_adjusted: output parameter for adjusted vbat uv threshold in mV
 * @current_avg: output parameter for average current in mA
 *
 * This function calculates average current and adjusts vbat_uv_thr_mv accordingly
 * when support_boost_ic is enabled and SOC < 20%.
 * If conditions are not met, vbat_uv_thr_mv_adjusted will be set to base_vbat_uv_thr_mv.
 */
static void oplus_comm_calc_vbat_uv_by_current(struct oplus_chg_comm *chip,
						int base_vbat_uv_thr_mv,
						int *vbat_uv_thr_mv_adjusted,
						int *current_avg)
{
	unsigned long now_tm_sec = 0;
	unsigned long time_diff_sec = 0;

	if (chip == NULL || vbat_uv_thr_mv_adjusted == NULL || current_avg == NULL) {
		if (vbat_uv_thr_mv_adjusted)
			*vbat_uv_thr_mv_adjusted = base_vbat_uv_thr_mv;
		if (current_avg)
			*current_avg = 0;
		return;
	}

	/* Initialize with base value */
	*vbat_uv_thr_mv_adjusted = base_vbat_uv_thr_mv;
	*current_avg = 0;


	/* Get time difference for current calculation */
	oplus_comm_get_time_diff_for_current_calc(chip, &time_diff_sec, &now_tm_sec);

	/* Adjust vbat_uv_thr_mv based on average current when SOC < 20% */
	if (chip->config.support_boost_ic &&
	    chip->soc < UV_ADJUSTED_CHECK_SOC) {

		/* Calculate average current using real time difference */
		/* current_avg = (passed_q - passed_q_last) * 3600 / time_diff_sec */
		/* passed_q is in mAh, so (passed_q - passed_q_last) is mAh change */
		/* Multiply by 3600 to convert mAh to mAs, then divide by seconds to get mA */
		if (time_diff_sec > 0)
			*current_avg = -(chip->passed_q - chip->passed_q_last) * 3600 / (int)time_diff_sec;
		else
			*current_avg = -1;

		chg_info("SOC %d < 20%%, current_avg=%dmA, passed_q=%d, passed_q_last=%d, time_diff=%lu sec\n",
				chip->soc, *current_avg, chip->passed_q, chip->passed_q_last, time_diff_sec);
		if (*current_avg < 0) {
			*vbat_uv_thr_mv_adjusted = base_vbat_uv_thr_mv;
			chg_info("charging, no adjust vbat_uv_thr_mv: %d -> %d\n",
				base_vbat_uv_thr_mv, *vbat_uv_thr_mv_adjusted);
		} else if (*current_avg < UV_ADJUSTED_LOW_THR) {
			*vbat_uv_thr_mv_adjusted = base_vbat_uv_thr_mv + UV_ADJUSTED_VOLT_THR;
			chg_info("current_avg < 400, adjust vbat_uv_thr_mv: %d -> %d\n",
				base_vbat_uv_thr_mv, *vbat_uv_thr_mv_adjusted);
		} else if (*current_avg > UV_ADJUSTED_HIGH_THR) {
			*vbat_uv_thr_mv_adjusted = base_vbat_uv_thr_mv - UV_ADJUSTED_VOLT_THR;
			chg_info("current_avg > 1000, adjust vbat_uv_thr_mv: %d -> %d\n",
				base_vbat_uv_thr_mv, *vbat_uv_thr_mv_adjusted);
		}
	}
}

static void oplus_comm_adjust_vbat_uv_by_current(struct oplus_chg_comm *chip,
						  int base_vbat_uv_thr_mv)
{
	struct oplus_comm_spec_config *spec = &chip->spec;
	int vbat_uv_thr_mv_adjusted = base_vbat_uv_thr_mv;
	int current_avg = 0;

	/* Calculate and adjust vbat_uv_thr_mv based on current */
	oplus_comm_calc_vbat_uv_by_current(chip, base_vbat_uv_thr_mv,
					  &vbat_uv_thr_mv_adjusted, &current_avg);

	if (vbat_uv_thr_mv_adjusted != base_vbat_uv_thr_mv) {
		spec->vbat_uv_thr_mv = vbat_uv_thr_mv_adjusted;
		spec->vbat_charging_uv_thr_mv = spec->vbat_uv_thr_mv - spec->vbat_charging_uv_delata_mv;
		spec->sub_vbat_uv_thr_mv = spec->vbat_uv_thr_mv;
		spec->sub_vbat_charging_uv_thr_mv = spec->vbat_charging_uv_thr_mv;

		/* Add track logging for ShutdownVol */
		if (chip->gauge_topic) {
			struct oplus_mms *err_topic;
			struct mms_msg *msg;
			union mms_msg_data data = { 0 };
			int tbat = 0;
			int rc;
			char crux_info[OPLUS_CHG_TRACK_CURX_INFO_LEN] = { 0 };
			int index = 0;

			rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_TEMP,
						     &data, false);
			if (rc >= 0)
				tbat = data.intval;

			index += scnprintf(&crux_info[index],
					   OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
					   "$$Avg_Cur@@%d", current_avg);
			index += scnprintf(&crux_info[index],
					   OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
					   "$$ShutdownVol@@%d", vbat_uv_thr_mv_adjusted);
			index += scnprintf(&crux_info[index],
					   OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
					   "$$Tbat@@%d", tbat);

			err_topic = oplus_mms_get_by_name("error");
			if (err_topic) {
				msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
							      ERR_ITEM_SHUTDOWN_VOL, crux_info);
				if (msg) {
					rc = oplus_mms_publish_msg_sync(err_topic, msg);
					if (rc < 0) {
						chg_err("publish ShutdownVol track msg error, rc=%d\n", rc);
						kfree(msg);
					}
				} else {
					chg_err("alloc ShutdownVol track msg error\n");
				}
			}
		}
	}
	spec->vbat_uv_thr_mv = vbat_uv_thr_mv_adjusted;
	oplus_comm_set_vbat_uv_thr(chip, spec->vbat_uv_thr_mv);
	chg_info("base_vbat_uv_thr_mv:%d, vbat_uv_thr_mv_adjusted:%d\n",
		base_vbat_uv_thr_mv, vbat_uv_thr_mv_adjusted);
}

static void oplus_comm_passed_q_save_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_comm *chip = container_of(dwork,
		struct oplus_chg_comm, passed_q_save_work);
	int rc;
	int passed_q_main = 0;
	int passed_q_sub = 0;
	int passed_q_total = 0;
	union mms_msg_data data = { 0 };

	if (!chip->config.support_boost_ic)
		return;

	if (!chip->gauge_topic)
		return;

	/* Save current passed_q */
	chip->passed_q_last = chip->passed_q;

	/* Get passed_q for main battery (index 0) */
	rc = oplus_gauge_get_car_c(chip->gauge_topic, 0, &passed_q_main);
	if (rc < 0)
		rc = oplus_gauge_get_dod0_passed_q(chip->gauge_topic, 0, &passed_q_main);
	if (rc < 0) {
		chg_err("get dod0_passed_q for main battery error, rc=%d\n", rc);
		schedule_delayed_work(&chip->passed_q_save_work,
				     msecs_to_jiffies(UV_ADJUSTED_CHECK_TIME * 1000));
		return;
	}

	passed_q_total = passed_q_main;

	/* If support parallel battery, also get passed_q for sub battery (index 1) */
	if (is_support_parallel_battery(chip->gauge_topic)) {
		rc = oplus_gauge_get_car_c(chip->gauge_topic, 1, &passed_q_sub);
		if (rc < 0)
			rc = oplus_gauge_get_dod0_passed_q(chip->gauge_topic, 1, &passed_q_sub);
		if (rc < 0) {
			chg_err("get dod0_passed_q for sub battery error, rc=%d\n", rc);
		} else {
			passed_q_total += passed_q_sub;
			chg_info("parallel battery: passed_q_main=%d, passed_q_sub=%d, total=%d\n",
				 passed_q_main, passed_q_sub, passed_q_total);
		}
	}

	chip->passed_q = passed_q_total;
	chg_info("save passed_q: %d, passed_q_last: %d, save_time: %lu\n",
		 chip->passed_q, chip->passed_q_last, chip->passed_q_save_time);

	/* Update vbat_uv_thr_mv based on current if needed */
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VBAT_UV,
					     &data, false);
	if (rc >= 0 && data.intval > GAUGE_VBAT_UV_DELATA) {
			oplus_comm_adjust_vbat_uv_by_current(chip, data.intval);
	}

	chg_info("base_vbat_uv_thr_mv:%d\n", data.intval);

	/* Schedule next save after 1 minute */
	schedule_delayed_work(&chip->passed_q_save_work,
			     msecs_to_jiffies(UV_ADJUSTED_CHECK_TIME * 1000));
}

#define CHG_UP_LIMIT_FAIL_THRESHOLD	3
#define CHG_UP_LIMIT_REAL_SOC_THRESHOLD	99
static bool get_chg_up_not_limit_state(int ui_soc, int smooth_soc)
{
	if (chg_up_limit_data.charge_limit_enable == 1 && ui_soc >= chg_up_limit_data.charge_limit_value &&
	    smooth_soc <= (ui_soc + CHG_UP_LIMIT_FAIL_THRESHOLD) && smooth_soc <= CHG_UP_LIMIT_REAL_SOC_THRESHOLD)
		return false;
	else
		return true;
}

static int calculate_soc_down_jiffies(struct oplus_comm_config *config, int ui_soc, bool charging)
{
	int soc_down_delay_hz = config->soc_down_default_delay_hz;
	static int last_soc_down_delay_hz = 0;
	int dex;

	for (dex = 0; dex < LOW_VOLT_UISOC_LADDER_NUMBER - 1; dex++) {
		if (config->soc_down_ladder[dex] == 0)
			break;
		if (ui_soc < config->soc_down_ladder[dex]) {
			if (charging)
				soc_down_delay_hz = config->soc_down_chg_delay_hz_ladder[dex];
			else
				soc_down_delay_hz = config->soc_down_delay_hz_ladder[dex];
			break;
		} else {
			if (charging)
				soc_down_delay_hz = config->soc_down_chg_delay_hz_ladder[dex + 1];
			else
				soc_down_delay_hz = config->soc_down_delay_hz_ladder[dex + 1];
		}
	}

	if (soc_down_delay_hz != last_soc_down_delay_hz) {
		chg_info("calculate_soc_down_jiffies final is %d %d", soc_down_delay_hz, last_soc_down_delay_hz);
		last_soc_down_delay_hz = soc_down_delay_hz;
	}
	return soc_down_delay_hz;
}

static int is_charging_full(struct oplus_chg_comm *chip)
{
	if (chip->wired_online && get_client_vote(chip->wired_charging_disable_votable, CHG_FULL_VOTER) > 0)
		return true;
	if (chip->wls_online && get_client_vote(chip->wls_charging_disable_votable, CHG_FULL_VOTER) > 0)
		return true;
	return false;
}

static void oplus_comm_smooth_strategy_set_full(struct oplus_chg_comm *chip)
{
	bool real_chg_full = false;

	if (chip->wired_online || chip->wls_online) {
		real_chg_full = (chip->temp_region > TEMP_REGION_COLD) &&
				(chip->temp_region < TEMP_REGION_WARM) &&
				chip->hmac && chip->authenticate &&
				is_charging_full(chip);
		if (chip->real_chg_full != real_chg_full)
			oplus_chg_strategy_set_process_data(chip->smooth_strategy, "chg_full", real_chg_full);
	}
}

static void oplus_comm_smooth_strategy_set_online(struct oplus_chg_comm *chip)
{
	if (!chip->smooth_strategy)
		return;

	oplus_chg_strategy_set_process_data(chip->smooth_strategy, "chg_online",
		(chip->wired_online || chip->wls_online));
}

int oplus_comm_smooth_strategy_rus_set_secondary_smooth_map(struct oplus_mms *topic, bool enable)
{
	struct oplus_chg_comm *chip = NULL;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(topic);
	if (!chip)
		return -EINVAL;
	if (!chip->smooth_strategy)
		return -EINVAL;

	oplus_chg_strategy_set_process_data(chip->smooth_strategy, "rus_set", enable);

	return 0;
}

static void oplus_comm_smooth_strategy_update(struct oplus_chg_comm *chip)
{
	int smooth_soc;
	int rc = 0;
	int chg_reserve;

	rc = oplus_chg_strategy_get_data(chip->smooth_strategy, &smooth_soc);
	if (rc || smooth_soc < 0 || smooth_soc > 100) {
		chg_info("smooth_soc %d invalid, rc=%d\n", smooth_soc, rc);
		smooth_soc =  chip->soc;
	}
	oplus_comm_set_smooth_soc(chip, smooth_soc);

	rc = oplus_chg_strategy_get_metadata(chip->smooth_strategy, &chg_reserve);
	if (rc || chg_reserve < 0) {
		chg_info("chg_reserve %d invalid, rc=%d\n", chg_reserve, rc);
		chg_reserve = 0;
	}
	chip->smooth_strategy_chg_reserve = chg_reserve;
}

static void oplus_comm_smooth_strategy_set_init_ui_soc(struct oplus_chg_comm *chip, int ui_soc)
{
	if (!chip->smooth_strategy)
		return;

	oplus_chg_strategy_set_process_data(chip->smooth_strategy, "init_ui_soc", ui_soc);
	oplus_comm_smooth_strategy_update(chip);
}

static void oplus_comm_smooth_soc_update(struct oplus_chg_comm *chip, bool init, bool check_full)
{
	if (chip->smooth_strategy) {
		if (init)
			oplus_chg_strategy_init(chip->smooth_strategy);
		if (check_full)
			oplus_comm_smooth_strategy_set_full(chip);
		oplus_comm_smooth_strategy_update(chip);
	} else if (chip->config.smooth_switch) {
		oplus_comm_smooth_to_soc(chip, false);
	} else if (chip->config.hidden_soc_switch) {
		oplus_hidden_soc_smooth(chip);
	} else {
		oplus_comm_set_smooth_soc(chip, chip->soc);
	}
}

static bool is_vooc_normalpath_chg(struct oplus_chg_comm *chip)
{
	return (chip->vooc_by_normal_path && chip->vooc_charging &&
		sid_to_adapter_chg_type(chip->vooc_sid) == CHARGER_TYPE_VOOC);
}

static bool report_full_check_fastchg(struct oplus_chg_comm *chip, bool charging)
{
	int mmi_chg = oplus_comm_get_mmi_state(chip);
	bool vooc_by_normalpath_chg = is_vooc_normalpath_chg(chip);

	return (charging && mmi_chg &&
		(!chip->vooc_charging || vooc_by_normalpath_chg) &&
		!chip->ufcs_charging &&
		!chip->pps_charging &&
		!is_wls_fastchg_started(chip));
}

static bool report_full_check_ffc(struct oplus_chg_comm *chip)
{
	return (chip->config.smooth_switch || chip->ffc_status == FFC_DEFAULT ||
		chip->config.hidden_soc_switch);
}

static bool should_report_full(struct oplus_chg_comm *chip, bool charging, int ui_soc)
{
	return (!chip->batt_full && ui_soc == 100 &&
		report_full_check_ffc(chip) &&
		report_full_check_fastchg(chip, charging));
}

static bool should_clear_full(struct oplus_chg_comm *chip, int ui_soc)
{
	return (ui_soc < 100 && chip->batt_full && !chip->sw_full && !chip->hw_full_by_sw);
}

static unsigned long oplus_comm_report_full(struct oplus_chg_comm *chip,
	bool charging, int ui_soc, unsigned long update_delay)
{
	struct oplus_comm_spec_config *spec = &chip->spec;
	unsigned long tmp;
	int full_time = spec->batt_full_time;

	if (should_report_full(chip, charging, ui_soc)) {
		if (chip->smooth_strategy)
			full_time = (chip->soc >= spec->batt_full_low_soc) ?
				spec->batt_full_time : spec->batt_full_low_soc_time;
		tmp = chip->batt_full_jiffies + (unsigned long)(full_time  * HZ);
		if (time_is_before_jiffies(tmp)) {
			chg_err("pre set battery full soc=%d timeout=%ds\n", chip->soc, full_time);
			oplus_comm_set_batt_full(chip, true);
		} else {
			if (update_delay > (tmp - jiffies))
				update_delay = tmp - jiffies;
		}
	} else if (should_clear_full(chip, ui_soc)) {
		chg_info("clean battery full status\n");
		oplus_comm_set_batt_full(chip, false);
	} else {
		chip->batt_full_jiffies = jiffies;
	}

	return update_delay;
}

#define CHARGE_FORCE_DEC_INTERVAL	60
#define NON_CHARGE_FORCE_DEC_INTERVAL	20
#define VBAT_MIN			2500
#define AGING_VERSION_SMOOTH_MIN_UISOC	1
static void oplus_comm_ui_soc_update(struct oplus_chg_comm *chip)
{
	struct oplus_comm_spec_config *spec = &chip->spec;
	struct oplus_comm_config *config = &chip->config;
	struct ui_soc_decimal *soc_decimal = &chip->soc_decimal;
	int ui_soc, smooth_soc;
	bool charging;
	bool force_down_1 = false;
	bool force_down_2 = false;
	unsigned long soc_up_jiffies, soc_down_jiffies, vbat_uv_jiffies;
	unsigned long sleep_tm = 0;
	unsigned long soc_reduce_margin;
	unsigned long update_delay = 0;
	unsigned long tmp;
	static bool pre_vbatt_too_low;
	bool vbatt_too_low = false;
	bool vbatt_low_to_cutoff = false;
	bool ui_soc_update;
	int force_dec_interval = 0;
	int mmi_chg = 1;
	int term_voltage = 0;
	int dex = 0;
	static unsigned long begin_vbatt_uv_jiffies = 0;
	int vbat_min = chip->vbat_min_mv;
	int power_role = chip->power_role;
	int cc_detect_status = chip->cc_detect_status;

	if (g_ui_soc_ready == false) {
		chg_err("g_ui_soc_ready is false %d", chip->ui_soc);
		return;
	}

	/* here shall wait the ui_soc is ready. */
	ui_soc = chip->ui_soc;
	smooth_soc = chip->smooth_soc;
	charging = chip->wired_online || chip->wls_online;

	if (is_support_parallel_battery(chip->gauge_topic) &&
	    chip->sub_vbat_mv > VBAT_MIN && chip->sub_vbat_mv < vbat_min)
		vbat_min = chip->sub_vbat_mv;

	soc_up_jiffies = chip->soc_up_update_jiffies + (unsigned long)(10 * HZ);
	soc_down_jiffies = chip->soc_down_update_jiffies +
		(calculate_soc_down_jiffies(config, ui_soc, charging) * HZ);
	if (!charging) {
		if (power_role == POWER_ROLE_SOURCE && cc_detect_status != CC_DETECT_NOTPLUG) {
			soc_down_jiffies = chip->soc_down_update_jiffies + SOC_DOWN_DELAY_FOR_REVERSE_CHARGING * HZ;
		}
	}

	if (chip->config.support_uisoc_low_battery_control)
		soc_down_jiffies = oplus_comm_ui_soc_low_battery_control(chip, soc_down_jiffies, vbat_min, &force_down_1);
	oplus_comm_gauge_r_info_check(chip);

	if (oplus_comm_is_under_uv(chip)) {
		if (charging)
			force_dec_interval = CHARGE_FORCE_DEC_INTERVAL;
		else
			force_dec_interval = NON_CHARGE_FORCE_DEC_INTERVAL;

		chg_err("[%d %d %d %d %d %d %d]\n",
			 spec->vbat_charging_uv_thr_mv, spec->vbat_uv_thr_mv,
			 charging, chip->vbat_min_mv, chip->vbat_mv,
			 force_dec_interval, config->chg_shutdown_max_mv);

		vbat_uv_jiffies = chip->vbat_uv_jiffies +
			(unsigned long)(force_dec_interval * HZ);
		/*
		 * When the battery voltage is too low, ensure at
		 * least one test every 5s.
		 */
		update_delay = msecs_to_jiffies(5000);

		vbatt_too_low = true;
		vbatt_low_to_cutoff = true;
		if (vbatt_too_low && !pre_vbatt_too_low)
			oplus_comm_push_vbat_too_low_msg(chip);
	} else {
		chip->vbat_uv_jiffies = jiffies;
		vbat_uv_jiffies = chip->vbat_uv_jiffies +
			(unsigned long)(NON_CHARGE_FORCE_DEC_INTERVAL * HZ);
	}

	if (chip->config.support_uisoc_low_battery_control) {
		if (vbat_min < chip->config.volt_of_fast_drop_soc_1) {
			if (charging)
				force_dec_interval = CHARGE_FORCE_DEC_INTERVAL;
			else
				force_dec_interval = NON_CHARGE_FORCE_DEC_INTERVAL;

			if (begin_vbatt_uv_jiffies == 0)
				begin_vbatt_uv_jiffies = jiffies;
			vbat_uv_jiffies = begin_vbatt_uv_jiffies +
				(unsigned long)(force_dec_interval * HZ);
		} else {
			begin_vbatt_uv_jiffies = jiffies;
		}

		if (vbat_min < chip->config.volt_of_fast_drop_soc_1 || ((chip->ui_soc == 1) &&
		    (vbatt_low_to_cutoff == true))) {
			/* Force ui_soc to drop to 0 when the voltage is too low */
			if (time_is_before_jiffies(vbat_uv_jiffies)) {
				chg_err("low volt force cutoff[%d %d %d %d %d %d %d %d]\n",
				    chip->config.volt_of_fast_drop_soc_1, vbatt_low_to_cutoff,
				    charging, vbat_min, chip->vbat_mv,
				    force_dec_interval, config->chg_shutdown_max_mv, chip->ui_soc);
				soc_down_jiffies = chip->soc_down_update_jiffies +
						   (unsigned long)(oplus_comm_get_uv_dec_speed(chip) * HZ);
				force_down_2 = true;
			} else {
				force_down_2 = false;
			}
		} else {
			force_down_2 = false;
		}
	} else if (vbatt_low_to_cutoff == true) {
		/* Force ui_soc to drop to 0 when the voltage is too low */
		if (time_is_before_jiffies(vbat_uv_jiffies)) {
			soc_down_jiffies = chip->soc_down_update_jiffies +
					   (unsigned long)(oplus_comm_get_uv_dec_speed(chip) * HZ);
			force_down_2 = true;
		} else {
			force_down_2 = false;
		}
	} else {
		force_down_2 = false;
	}
	pre_vbatt_too_low = vbatt_too_low;

	if (force_down_1 || force_down_2) {
		if (time_is_before_jiffies(soc_down_jiffies)) {
			ui_soc = (ui_soc > 0) ? (ui_soc - 1) : 0;
			oplus_comm_set_delta_soc(chip, min(MAX_DELTA_SOC, max(ui_soc - smooth_soc, MIN_DELTA_SOC)));
		}
		goto check_update_time;
	}

	/* Special handling after wake-up */
	if (chip->sleep_tm_sec > 0 && ui_soc > smooth_soc &&
	    !(chip->batt_full)) {
		ui_soc_update = true;
		sleep_tm = chip->sleep_tm_sec;
		chip->sleep_tm_sec = 0;
		soc_reduce_margin = sleep_tm / TEN_MINUTES;
		if (soc_reduce_margin == 0) {
			chip->save_sleep_tm_sec += sleep_tm;
			if (chip->save_sleep_tm_sec >= ONE_MINUTE && (ui_soc - smooth_soc) > 2) {
				ui_soc = (ui_soc > 1) ? (ui_soc - 1) : 1;
				chip->save_sleep_tm_sec = 0;
			} else {
				ui_soc_update = false;
			}
		} else if (soc_reduce_margin < (ui_soc - smooth_soc)) {
			ui_soc -= soc_reduce_margin;
			chip->save_sleep_tm_sec = 0;
		} else if (soc_reduce_margin >= (ui_soc - smooth_soc)) {
			ui_soc = smooth_soc;
			chip->save_sleep_tm_sec = 0;
		}
		if (ui_soc < 1)
			ui_soc = 1;
		if (ui_soc_update) {
			oplus_comm_set_delta_soc(chip, min(MAX_DELTA_SOC, max(ui_soc - smooth_soc, MIN_DELTA_SOC)));
			goto done;
		}
	}

	/* Here ui_soc is only allowed to drop to 1% as low as possible */
	mmi_chg = oplus_comm_get_mmi_state(chip);
	if (charging && mmi_chg && get_chg_up_not_limit_state(ui_soc, smooth_soc)) {
		if (ui_soc < smooth_soc &&
		    time_is_before_jiffies(soc_up_jiffies)) {
			ui_soc = (ui_soc < 100) ? (ui_soc + 1) : 100;
			chip->sleep_tm_sec = 0;
			chip->save_sleep_tm_sec = 0;
		} else if (ui_soc < 100 &&
			   (chip->sw_full || chip->hw_full_by_sw) &&
			   !chip->uisoc_down_in_full &&
			   (chip->temp_region < TEMP_REGION_WARM) &&
			   (chip->hmac && chip->authenticate) &&
			   time_is_before_jiffies(soc_up_jiffies)) {
			ui_soc = (ui_soc < 100) ? (ui_soc + 1) : 100;
			chip->sleep_tm_sec = 0;
			chip->save_sleep_tm_sec = 0;
		} else if (ui_soc > (smooth_soc + chip->delta_soc) &&
			   (!(chip->sw_full || chip->hw_full_by_sw) || chip->rechg_soc_en) &&
			   time_is_before_jiffies(soc_down_jiffies) &&
			   chip->ibat_ma > ALLOW_UISOC_DOWN_CURRENT) {
			ui_soc = (ui_soc > 1) ? (ui_soc - 1) : 1;
			chip->save_sleep_tm_sec = 0;
			if (chip->sw_full || chip->hw_full_by_sw)
				chip->uisoc_down_in_full = true;
		}
		if (ui_soc == smooth_soc)
			oplus_comm_set_delta_soc(chip, MIN_DELTA_SOC);
	} else {
		if (ui_soc > smooth_soc &&
		    time_is_before_jiffies(soc_down_jiffies)) {
			ui_soc = (ui_soc > 1) ? (ui_soc - 1) : 1;
			chip->save_sleep_tm_sec = 0;
		}
		oplus_comm_set_delta_soc(chip, min(MAX_DELTA_SOC, max(ui_soc - smooth_soc, MIN_DELTA_SOC)));
	}

check_update_time:
	/* determine the next update time */
	if (update_delay == 0)
		update_delay = msecs_to_jiffies(oplus_mms_update_interval(chip->gauge_topic));
	if (time_is_after_jiffies(soc_down_jiffies)) {
		tmp = soc_down_jiffies - jiffies;
		if (update_delay > tmp)
			update_delay = tmp;
	}
	if (time_is_after_jiffies(soc_up_jiffies)) {
		tmp = soc_up_jiffies - jiffies;
		if (update_delay > tmp)
			update_delay = tmp;
	}

done:
	//When the voltage is greater than soc 2% and the ui_soc is less than or equal to 2%, maintain 2% ui_soc.
	if (chip->deep_support && chip->config.ui_soc_2_voltage_comp_mv != INT_MAX &&
	    is_gauge_term_voltage_votable_available(chip)) {
		for (dex = 0; dex < LOW_VOLT_UISOC_LADDER_NUMBER - 1; dex++) {
			if (chip->config.temp_ladder_of_keep_soc_2[dex] == 0 ||
			    chip->config.support_uisoc_low_battery_control == false)
				break;
			if (chip->shell_temp < chip->config.temp_ladder_of_keep_soc_2[dex]) {
				chip->config.ui_soc_2_voltage_comp_mv = chip->config.volt_diff_ladder_of_keep_soc_2[dex];
				break;
			} else {
				chip->config.ui_soc_2_voltage_comp_mv = chip->config.volt_diff_ladder_of_keep_soc_2[dex+1];
			}
		}
		term_voltage = get_effective_result(chip->gauge_term_voltage_votable);
		if (term_voltage > 0 && !charging && chip->ui_soc >= 2 && ui_soc < 2 && smooth_soc <= 1 &&
		    (chip->shell_temp > -50 || chip->config.support_uisoc_low_battery_control) &&
		    vbat_min > (term_voltage + chip->config.ui_soc_2_voltage_comp_mv)) {
			ui_soc = 2;
			oplus_comm_set_uisoc_keep_2_error(chip, 1);
		} else {
			oplus_comm_set_uisoc_keep_2_error(chip, 0);
		}
	}

	//When the voltage is greater than soc 3% and the ui_soc is less than or equal to 3%, maintain 3% ui_soc.
	if (chip->deep_support && chip->config.ui_soc_3_voltage_comp_mv != INT_MAX &&
	    is_gauge_term_voltage_votable_available(chip)) {
		for (dex = 0; dex < LOW_VOLT_UISOC_LADDER_NUMBER - 1; dex++) {
			if (chip->config.temp_ladder_of_keep_soc_3[dex] == 0 ||
			    chip->config.support_uisoc_low_battery_control == false)
				break;
			if (chip->shell_temp < chip->config.temp_ladder_of_keep_soc_3[dex]) {
				chip->config.ui_soc_3_voltage_comp_mv = chip->config.volt_diff_ladder_of_keep_soc_3[dex];
				break;
			} else {
				chip->config.ui_soc_3_voltage_comp_mv = chip->config.volt_diff_ladder_of_keep_soc_3[dex+1];
			}
		}
		term_voltage = get_effective_result(chip->gauge_term_voltage_votable);
		if (term_voltage > 0 && !charging && chip->ui_soc >= 3 && ui_soc < 3 && smooth_soc <= 2 &&
		    (chip->shell_temp > -50 || chip->config.support_uisoc_low_battery_control) &&
		    vbat_min > (term_voltage + chip->config.ui_soc_3_voltage_comp_mv)) {
			ui_soc = 3;
			oplus_comm_set_uisoc_keep_3_error(chip, 1);
		} else {
			oplus_comm_set_uisoc_keep_3_error(chip, 0);
		}
	}

	/* When the decimal point is displayed, the battery is prohibited from dropping */
	if (chip->vooc_online && config->vooc_show_ui_soc_decimal &&
	    soc_decimal->decimal_control && ui_soc < chip->ui_soc)
		ui_soc = chip->ui_soc;

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if ((get_eng_version() == HIGH_TEMP_AGING || get_eng_version() == AGING || oplus_is_ptcrb_version()) &&
	    ui_soc > AGING_VERSION_SMOOTH_MIN_UISOC)
		ui_soc = max(chip->smooth_soc, AGING_VERSION_SMOOTH_MIN_UISOC);
#endif

	if (chip->ui_soc != ui_soc) {
		if (!config->vooc_show_ui_soc_decimal || !soc_decimal->decimal_control)
			oplus_comm_set_ui_soc(chip, ui_soc);
	} else {
		update_delay = oplus_comm_report_full(chip, charging, ui_soc, update_delay);
	}

	if (update_delay > 0)
		schedule_delayed_work(&chip->ui_soc_update_work, update_delay);
}

static void monitor_ui_soc_to_enable_chg_up_limit(struct oplus_chg_comm *chip, bool immediate_execut);
static void oplus_comm_ui_soc_update_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_comm *chip = container_of(dwork,
		struct oplus_chg_comm, ui_soc_update_work);

	oplus_comm_ui_soc_update(chip);
	monitor_ui_soc_to_enable_chg_up_limit(chip, false);
}

#ifdef CONFIG_OPLUS_CHARGER_MTK
#define POWER_OFF_VBUS_CHECK 2500
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
static void oplus_chg_power_off_check_work(struct work_struct *work)
{
	chg_info("Unplug Charger/USB In Kernel Power Off Charging Mode Shutdown OS!\n");
	kernel_power_off();
}
#endif

static void oplus_chg_kpoc_power_off_check(struct oplus_chg_comm *chip)
{
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
	int boot_mode = 0;
	int vbus_mv = 0;
	bool wired_online = true;
	union mms_msg_data data = { 0 };
	boot_mode = get_boot_mode();
	if (boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT
			|| boot_mode == LOW_POWER_OFF_CHARGING_BOOT) {
		vbus_mv = oplus_wired_get_vbus();
		oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_ONLINE, &data, true);
		wired_online = data.intval;
		if (!wired_online && vbus_mv < POWER_OFF_VBUS_CHECK) {
			if (!work_pending(&chip->power_off_check_work.work)) {
				/* add for discharge the capacitor completely */
				msleep(3000);
				chg_info("Unplug Charger/USB double check!\n");
				vbus_mv = oplus_wired_get_vbus();
				oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_ONLINE, &data, true);
				wired_online = data.intval;
				if (!wired_online && vbus_mv < POWER_OFF_VBUS_CHECK &&
					(!chip->vooc_online && !chip->vooc_online_keep)) {
					chg_info("Schedule power off check work after 57s\n");
					schedule_delayed_work(&chip->power_off_check_work, msecs_to_jiffies(57000));
				}
			}
		} else {
			cancel_delayed_work(&chip->power_off_check_work);
		}
	}
#endif
}
#endif

static int oplus_comm_chg_get_batt_curve_current(struct oplus_chg_comm *chip)
{
	if (chip == NULL) {
		chg_err("comm topic is NULL\n");
		return -ENODEV;
	}
	if (chip->vooc_charging)
		return oplus_vooc_get_batt_curve_current(chip->vooc_topic);
	else if (chip->ufcs_charging)
		/* The ibus returned by this interface takes into account the adapter current*/
		return oplus_ufcs_get_curve_ibus(chip->ufcs_topic);
	else if (chip->pps_charging)
		/* The ibus returned by this interface takes into account the adapter current*/
		return oplus_pps_get_curve_ibus(chip->pps_topic);

	return -EINVAL;
}

#define MIN_CURVE_CURR 2000
#define MAX_CURVE_CURR_SINGLE 9000
#define MAX_CURVE_CURR_DUAL 6000
#define MIN_CALCULATE_COUNT 1
static int oplus_ui_soc_get_current(struct oplus_chg_comm *chip)
{
	struct ui_soc_decimal *soc_decimal = &chip->soc_decimal;
	int batt_num = oplus_gauge_get_batt_num();
	int curr;
	static int last_curr = 0;

	if (batt_num == 1) {
		curr = oplus_comm_chg_get_batt_curve_current(chip) * 2 * 7 / 10;
		chg_info("get curve current %d\n", curr);
		curr = curr > MAX_CURVE_CURR_SINGLE ? MAX_CURVE_CURR_SINGLE : curr;
	} else {
		curr = oplus_comm_chg_get_batt_curve_current(chip) * 7 / 10;
		chg_info("get curve current %d\n", curr);
		curr = curr > MAX_CURVE_CURR_DUAL ? MAX_CURVE_CURR_DUAL : curr;
	}

	if (curr < MIN_CURVE_CURR)
		curr = MIN_CURVE_CURR;

	/* ensure increase speed */
	if (soc_decimal->calculate_decimal_time > MIN_CALCULATE_COUNT &&
	    curr < last_curr)
		return last_curr;

	last_curr = curr;

	return curr;
}

#define EIS_DUMMY_CURRENT_STEP 		1800
#define EIS_BCC_CURRENT_MA_MULT		100
#define EIS_CURRENT_THRESHOLD		1000
#define EIS_MAX_CURRENT_SINGLE		9000
#define EIS_MAX_CURRENT_DUAL		6000
#define EIS_MAX_CURRENT_OTHERS		6000
static long int oplus_comm_eis_dummy_current_limit(struct oplus_chg_comm *chip, long int d_current)
{
	int batt_num = oplus_gauge_get_batt_num();
	union mms_msg_data data = { 0 };
	int bcc_max_curr = 0;
	long int dummy_icharging = d_current;

	oplus_mms_get_item_data(chip->vooc_topic,
				VOOC_ITEM_GET_BCC_MAX_CURR, &data, false);
	bcc_max_curr = data.intval * EIS_BCC_CURRENT_MA_MULT;

	if ((bcc_max_curr > 0) && (dummy_icharging > bcc_max_curr))
		dummy_icharging = bcc_max_curr;

	if (batt_num == 1) {
		if (dummy_icharging > EIS_MAX_CURRENT_SINGLE)
			dummy_icharging = EIS_MAX_CURRENT_SINGLE;
	} else if (batt_num == 2) {
		if (dummy_icharging > EIS_MAX_CURRENT_DUAL)
			dummy_icharging = EIS_MAX_CURRENT_DUAL;
	} else if (dummy_icharging > EIS_MAX_CURRENT_OTHERS) {
		dummy_icharging = EIS_MAX_CURRENT_OTHERS;
	}

	return dummy_icharging;
}

static bool oplus_comm_calculate_eis_soc_speed(struct oplus_chg_comm *chip, long int *speed)
{
	struct ui_soc_decimal *soc_decimal = &chip->soc_decimal;
	union mms_msg_data data = { 0 };
	int batt_num = oplus_gauge_get_batt_num();
	int eis_status = 0;
	long int icharging = 0;
	long int dummy_icharging = 0;
	int eis_vooc_current = 0;
	int eis_ufcs_current = 0;

	if (is_vooc_curr_votable_available(chip))
		eis_vooc_current = get_client_vote(chip->vooc_curr_votable, EIS_VOTER);

	if (is_ufcs_curr_votable_available(chip))
		eis_ufcs_current = get_client_vote(chip->ufcs_curr_votable, EIS_VOTER);

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_EIS_STATUS, &data, false);
	eis_status = data.intval;

	data.intval = 0;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CURR, &data, false);
	icharging = -data.intval;

	if ((eis_vooc_current > 0) || (eis_ufcs_current > 0) ||
		(eis_status == EIS_STATUS_HIGH_CURRENT) ||
		(eis_status == EIS_STATUS_LOW_CURRENT)) {
		if ((icharging < EIS_CURRENT_THRESHOLD) && (soc_decimal->eis_dummy_cnt == 0)) {
			dummy_icharging = icharging;
		} else {
			dummy_icharging = icharging + soc_decimal->eis_dummy_cnt * EIS_DUMMY_CURRENT_STEP;
			soc_decimal->eis_dummy_cnt++;
		}

		dummy_icharging = oplus_comm_eis_dummy_current_limit(chip, dummy_icharging);

		*speed = 100000 * dummy_icharging * UPDATE_TIME * batt_num / (chip->batt_fcc * 3600);
		chg_info("<EIS> ichging[%ld], d_ichging[%ld], speed[%ld]",
				icharging, dummy_icharging, *speed);
		return true;
	}

	return false;
}

static unsigned int get_random_in_range(void)
{
    unsigned int random;

    get_random_bytes(&random, sizeof(random));
    return random % 501;
}

void oplus_comm_ui_soc_decimal_init(struct oplus_chg_comm *chip)
{
	struct ui_soc_decimal *soc_decimal = &chip->soc_decimal;
	union mms_msg_data data = { 0 };

	if (chip->gauge_topic != NULL) {
		if (chip->wired_online || chip->wls_online)
			oplus_mms_topic_update(chip->gauge_topic, false);
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_RM, &data, false);
		chip->batt_rm = data.intval;
		if (chip->batt_rm < 0) {
			chip->batt_rm = 0;
		}
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_FCC, &data, false);
		chip->batt_fcc = data.intval;
		if (chip->batt_fcc == 0) {
			chg_err("batt_fcc is 0\n");
			chip->batt_fcc = 4500; /* default set to 4500mAh */
		}
	}
	chg_info("ui_soc_decimal: soc=%d", (int)((chip->batt_rm * 10000) / chip->batt_fcc));

	if (chip->ui_soc == 100) {
		soc_decimal->ui_soc_integer = chip->ui_soc * 1000;
		soc_decimal->ui_soc_decimal = 0;
	} else {
		soc_decimal->ui_soc_integer = chip->ui_soc * 1000;
		soc_decimal->ui_soc_decimal =
			chip->batt_rm * 100000 / chip->batt_fcc - (chip->batt_rm * 100 / chip->batt_fcc) * 1000;
		if (chip->config.smooth_switch && chip->config.reserve_soc) {
			soc_decimal->ui_soc_decimal = soc_decimal->ui_soc_decimal * OPLUS_FULL_SOC /
						      (OPLUS_FULL_SOC - chip->config.reserve_soc);
		}
		if ((soc_decimal->ui_soc_integer + soc_decimal->ui_soc_decimal) > soc_decimal->last_decimal_ui_soc &&
		    soc_decimal->last_decimal_ui_soc != 0) {
			soc_decimal->ui_soc_decimal = ((soc_decimal->last_decimal_ui_soc % 1000 - 50) > 0) ?
							      (soc_decimal->last_decimal_ui_soc % 1000 - 50) : 0;
		}
		if (chip->ui_soc - chip->smooth_soc > 1) {
			soc_decimal->ui_soc_decimal = get_random_in_range();
			chg_info("UI-smooth gap is too large, use ramdam decimal, ui=%d, smmoth=%d, decimal=%d",
					chip->ui_soc, chip->smooth_soc, soc_decimal->ui_soc_decimal);
		}
	}
	soc_decimal->init_decimal_ui_soc = soc_decimal->ui_soc_integer + soc_decimal->ui_soc_decimal;
	if (soc_decimal->init_decimal_ui_soc > 100000) {
		soc_decimal->init_decimal_ui_soc = 100000;
		soc_decimal->ui_soc_integer = 100000;
		soc_decimal->ui_soc_decimal = 0;
	}
	soc_decimal->eis_dummy_cnt = 0;

	soc_decimal->decimal_control = true;
	chg_info("2VBUS ui_soc_decimal:%d", soc_decimal->ui_soc_integer + soc_decimal->ui_soc_decimal);

	soc_decimal->calculate_decimal_time = 1;
}

void oplus_comm_ui_soc_decimal_deinit(struct oplus_chg_comm *chip)
{
	struct ui_soc_decimal *soc_decimal = &chip->soc_decimal;
	int ui_soc;

	mutex_lock(&chip->decimal_lock);
	ui_soc = (soc_decimal->ui_soc_integer + soc_decimal->ui_soc_decimal) / 1000;
	mutex_unlock(&chip->decimal_lock);
	if (ui_soc != 0 && soc_decimal->ui_soc_decimal != 0) {
		if (soc_decimal->ui_soc_decimal != 0 && ui_soc < chip->smooth_soc &&
		    get_chg_up_not_limit_state(ui_soc, chip->smooth_soc))
			ui_soc = (ui_soc < 100) ? (ui_soc + 1) : 100;
		oplus_comm_set_ui_soc(chip, ui_soc);
	}
	soc_decimal->decimal_control = false;
	chg_info("ui_soc_decimal: ui_soc=%d, soc=%d,ui_soc_decimal=%d\n",
		 chip->ui_soc, chip->smooth_soc, soc_decimal->ui_soc_decimal);
	mutex_lock(&chip->decimal_lock);
	soc_decimal->ui_soc_integer = 0;
	soc_decimal->ui_soc_decimal = 0;
	mutex_unlock(&chip->decimal_lock);
	soc_decimal->init_decimal_ui_soc = 0;
}

#define UI_SOC_DECIMAL_SPEED_MIN	10L
static void oplus_comm_show_ui_soc_decimal(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_comm *chip = container_of(dwork, struct oplus_chg_comm, ui_soc_decimal_work);
	struct oplus_comm_config *config = &chip->config;
	struct ui_soc_decimal *soc_decimal = &chip->soc_decimal;
	union mms_msg_data data = { 0 };
	int batt_num = oplus_gauge_get_batt_num();
	long int speed, icharging;
	int temp_soc;
	int mmi_chg;
	int curve_current;

	if (chip->gauge_topic != NULL) {
		if (chip->wired_online || chip->wls_online)
			oplus_mms_topic_update(chip->gauge_topic, false);
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_RM, &data, false);
		chip->batt_rm = data.intval;
		if (chip->batt_rm < 0) {
			chip->batt_rm = 0;
		}
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_FCC, &data, false);
		chip->batt_fcc = data.intval;
		if (chip->batt_fcc == 0) {
			chg_err("batt_fcc is 0\n");
			chip->batt_fcc = 4500; /* default set to 4500mAh */
		}
	}
	curve_current = oplus_ui_soc_get_current(chip);
	chg_info("curve_current is %d\n", curve_current);
	icharging = curve_current;

	monitor_ui_soc_to_enable_chg_up_limit(chip, false);

	/*calculate the speed*/
	mmi_chg = oplus_comm_get_mmi_state(chip);
	if (icharging > 0 && mmi_chg && get_chg_up_not_limit_state(chip->ui_soc, chip->smooth_soc)) {
		if (!oplus_comm_calculate_eis_soc_speed(chip, &speed))
			speed = 100000 * icharging * UPDATE_TIME * batt_num / (chip->batt_fcc * 3600);

		if (chip->config.smooth_switch && chip->config.reserve_soc)
			speed = speed * OPLUS_FULL_SOC / (OPLUS_FULL_SOC - chip->config.reserve_soc);
		chg_info("ui_soc_decimal: icharging=%d, batt_fcc=%d", curve_current, chip->batt_fcc);
		if (chip->ui_soc - chip->smooth_soc > 2) {
			speed = speed / 2;
		} else if (chip->ui_soc < chip->smooth_soc) {
			speed = speed * 2;
		}
		speed = max(speed, UI_SOC_DECIMAL_SPEED_MIN);
	} else {
		speed = 0;
		if (chip->batt_full)
			speed = config->ui_soc_decimal_speedmin;
	}
	if (speed > 500)
		speed = 500;

	if (get_chg_up_not_limit_state(chip->ui_soc, chip->smooth_soc) == false) {
		speed = 0;
		soc_decimal->ui_soc_decimal = 0;
	}

	mutex_lock(&chip->decimal_lock);
	soc_decimal->ui_soc_decimal += speed;
	chg_info("ui_soc_decimal: (ui_soc_decimal+ui_soc)=%d, speed=%ld, soc=%d, calculate_decimal_time:%d\n",
		 (soc_decimal->ui_soc_decimal + soc_decimal->ui_soc_integer), speed,
		 ((chip->batt_rm * 10000) / chip->batt_fcc), soc_decimal->calculate_decimal_time);
	if (soc_decimal->ui_soc_integer + soc_decimal->ui_soc_decimal >= 100000) {
		soc_decimal->ui_soc_integer = 100000;
		soc_decimal->ui_soc_decimal = 0;
	}
	mutex_unlock(&chip->decimal_lock);

	if (soc_decimal->calculate_decimal_time <= MAX_UI_DECIMAL_TIME) {
		soc_decimal->calculate_decimal_time++;
		/* update ui_soc */
		mutex_lock(&chip->decimal_lock);
		temp_soc = (soc_decimal->ui_soc_integer + soc_decimal->ui_soc_decimal) / 1000;
		mutex_unlock(&chip->decimal_lock);
		if (temp_soc != 0)
			oplus_comm_set_ui_soc(chip, temp_soc);
		schedule_delayed_work(&chip->ui_soc_decimal_work, msecs_to_jiffies(UPDATE_TIME * 1000));
	} else {
		oplus_comm_ui_soc_decimal_deinit(chip);
	}
}

__maybe_unused static bool
is_chg_disable_votable_available(struct oplus_chg_comm *chip)
{
	if (!chip->chg_disable_votable)
		chip->chg_disable_votable = find_votable("CHG_DISABLE");
	return !!chip->chg_disable_votable;
}

__maybe_unused static bool
is_chg_suspend_votable_available(struct oplus_chg_comm *chip)
{
	if (!chip->chg_suspend_votable)
		chip->chg_suspend_votable = find_votable("CHG_SUSPEND");
	return !!chip->chg_suspend_votable;
}

int oplus_set_chg_up_limit(struct oplus_mms *topic, int charge_limit_enable, int charge_limit_value,
	int is_force_set_charge_limit, int charge_limit_recharge_value, int callname)
{
	struct oplus_chg_comm *chip;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -ENODEV;
	}
	chip = oplus_mms_get_drvdata(topic);

	chg_up_limit_data.charge_limit_enable = charge_limit_enable;
	chg_up_limit_data.charge_limit_value = charge_limit_value;
	chg_up_limit_data.is_force_set_charge_limit = is_force_set_charge_limit;
	chg_up_limit_data.charge_limit_recharge_value = charge_limit_recharge_value;
	chg_up_limit_data.callname = callname;

	monitor_ui_soc_to_enable_chg_up_limit(chip, true);
	return 1;
}

#define DEFAULT_OVER_CHARGE_DOD 550
static bool chg_up_limit_decimal_enable(struct oplus_chg_comm *chip)
{
	union mms_msg_data data = { 0 };
	int ui_soc_decimal;
	int batt_rm;
	int batt_fcc;

	if (chip == NULL || chip->gauge_topic == NULL) {
		chg_err("chg_up chip->gauge_topic == NULL\n");
		return true;
	}

	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_RM, &data, false);
	batt_rm = data.intval;
	if (batt_rm < 0) {
		chg_err("batt_rm is < 0\n");
		goto out;
	}
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_FCC, &data, false);
	batt_fcc = data.intval;
	if (batt_fcc == 0) {
		chg_err("batt_fcc is 0\n");
		goto out;
	}
	ui_soc_decimal =
		batt_rm * 100000 / batt_fcc;
	if (ui_soc_decimal >
	    (chg_up_limit_data.charge_limit_value - 1) * 1000 + DEFAULT_OVER_CHARGE_DOD ||
	    chip->smooth_soc > chg_up_limit_data.charge_limit_value) {
		chg_info("chg_up get_ui_soc_decimal %d %d %d %d %d\n",
		    ui_soc_decimal, batt_rm, batt_fcc, chip->smooth_soc,
		    chg_up_limit_data.charge_limit_value);
		return true;
	}
	return false;

out:
	if (chip->smooth_soc > chg_up_limit_data.charge_limit_value) {
		chg_info("chg_up get_ui_soc_decimal %d %d\n",
		    chip->smooth_soc, chg_up_limit_data.charge_limit_value);
		return true;
	}
	return false;
}

static int oplus_enforce_chg_up_limit_result(struct oplus_chg_comm *chip, bool cut_off_charge)
{
	int val = cut_off_charge;
	int rc = 0;
	static int pre_val = 0;
	static int pre_is_force_set_flag = 0;
	bool decimal_status;

	if (chip == NULL) {
		chg_err("chip == NULL\n");
		return rc;
	}

	chg_debug("oplus_set_chg_up_limit %d\n", val);
	if ((pre_val == val) && (pre_is_force_set_flag == chg_up_limit_data.is_force_set_charge_limit)) {
		if ((val == true && ((get_client_vote(chip->chg_disable_votable, CHG_LIMIT_CHG_VOTER) > 0 &&
		    chip->ui_soc == chg_up_limit_data.charge_limit_value) ||
		    (get_client_vote(chip->chg_suspend_votable, CHG_LIMIT_CHG_VOTER) > 0 &&
		    chip->ui_soc > chg_up_limit_data.charge_limit_value))) ||
		    (val == false && (get_client_vote(chip->chg_disable_votable, CHG_LIMIT_CHG_VOTER) == 0 &&
		    get_client_vote(chip->chg_suspend_votable, CHG_LIMIT_CHG_VOTER) == 0))) {
			chg_debug("set same chg up limit command %d %d\n", val, pre_is_force_set_flag);
			return rc;
		}
	}

	/* When the charging upper limit suspend is enabled before,
	and ui_soc drops to the charging upper limit value, change it to disable charge. */
	if (cut_off_charge == true &&
	    chip->ui_soc == chg_up_limit_data.charge_limit_value &&
		chg_up_limit_data.is_force_set_charge_limit == 1 &&
		get_client_vote(chip->chg_suspend_votable, CHG_LIMIT_CHG_VOTER) > 0) {
		rc = vote(chip->chg_suspend_votable, CHG_LIMIT_CHG_VOTER, false, 0, false);
		if (is_chg_disable_votable_available(chip))
			rc = vote(chip->chg_disable_votable, CHG_LIMIT_CHG_VOTER,
				  true, val, false);
		else
			rc = -ENOTSUPP;
	}

	if (chip->ui_soc == chg_up_limit_data.charge_limit_value && cut_off_charge == true) {
		decimal_status = chg_up_limit_decimal_enable(chip);
		if (decimal_status == false)
			return rc;
	}

	if (chg_up_limit_data.is_force_set_charge_limit == 0 || chip->ui_soc == chg_up_limit_data.charge_limit_value) {
		rc = vote(chip->chg_suspend_votable, CHG_LIMIT_CHG_VOTER, false, 0, false);
		if (is_chg_disable_votable_available(chip))
			rc = vote(chip->chg_disable_votable, CHG_LIMIT_CHG_VOTER,
				  val ? true : false, val, false);
		else
			rc = -ENOTSUPP;
	} else {
		rc = vote(chip->chg_disable_votable, CHG_LIMIT_CHG_VOTER, false, 0, false);
		if (is_chg_suspend_votable_available(chip))
			rc = vote(chip->chg_suspend_votable, CHG_LIMIT_CHG_VOTER,
				  val ? true : false, val, false);
		else
			rc = -ENOTSUPP;
	}

	if (chip->wls_online) {
		if (is_wls_icl_votable_available(chip) &&
		    is_wls_fastchg_disable_votable_available(chip) &&
		    is_wls_comu_votable_available(chip) &&
		    is_wls_charging_disable_votable_available(chip)) {
			vote(chip->wls_charging_disable_votable, CHG_LIMIT_CHG_VOTER, val ? true : false, val, false);
			vote(chip->wls_fastchg_disable_votable, CHG_LIMIT_CHG_VOTER, val ? true : false, val, false);
			vote(chip->wls_icl_votable, CHG_LIMIT_CHG_VOTER, val ? true : false, 300, true);
			vote(chip->wls_comu_votable, CHG_LIMIT_CHG_VOTER, val ? true : false, WLS_RX_COMU_CAP_AB, true);
		}
		else
			rc = -ENOTSUPP;
	}

	if (rc < 0)
		chg_err("can't set charging %s, rc=%d\n",
		       val ? "disable" : "enable", rc);
	else
		chg_info("chg_up set charging %s\n", val ? "disable" : "enable");

	chg_info("chg_up_limit_show %d %d,%d %d,%d %d, %d %d\n", val, chg_up_limit_data.charge_limit_enable,
		chg_up_limit_data.charge_limit_value, chg_up_limit_data.is_force_set_charge_limit,
		chg_up_limit_data.charge_limit_recharge_value, chg_up_limit_data.callname,
		pre_val, pre_is_force_set_flag);
	pre_val = val;
	pre_is_force_set_flag = chg_up_limit_data.is_force_set_charge_limit;
	return rc;
}

#define CHG_UP_DELAY_COUNT		4
#define CHG_UP_EXECUT_EACH_MS		1200
static void monitor_ui_soc_to_enable_chg_up_limit(struct oplus_chg_comm *chip, bool immediate_execut)
{
	static int over_count = 0;
	static unsigned long last_jiffies = 0;
	unsigned long jiffies_diff = 0;

	if (g_ui_soc_ready == false)
		return;

	if (!chip->wired_online && !chip->wls_online) {
		if (get_client_vote(chip->chg_suspend_votable, CHG_LIMIT_CHG_VOTER) > 0 ||
		    get_client_vote(chip->chg_disable_votable, CHG_LIMIT_CHG_VOTER) > 0)
			oplus_enforce_chg_up_limit_result(chip, false);
		over_count = 0;
		return;
	}

	jiffies_diff = jiffies > last_jiffies ? jiffies - last_jiffies : last_jiffies - jiffies;
	chg_debug("jiffies_diff %ld %ld %ld", jiffies_diff, jiffies, last_jiffies);
	if (jiffies_diff < msecs_to_jiffies(CHG_UP_EXECUT_EACH_MS) && (immediate_execut == false))
		return;

	last_jiffies = jiffies;

	chg_debug("monitor_ui_soc_to_enable_chg_up_limit: charge_limit_enable=%d, ui_soc=%d %d %d %d",
		chg_up_limit_data.charge_limit_enable, chip->ui_soc, chg_up_limit_data.charge_limit_value,
		chg_up_limit_data.charge_limit_recharge_value, over_count);

	if (chg_up_limit_data.charge_limit_enable == 1) {
		if (chip->ui_soc >= chg_up_limit_data.charge_limit_value &&
			chg_up_limit_data.charge_limit_value < OPLUS_FULL_SOC) {
			over_count++;
			if (over_count >= CHG_UP_DELAY_COUNT || chip->vooc_charging || chip->ufcs_charging ||
			    chip->pps_charging || is_wls_fastchg_started(chip)) {
				over_count = CHG_UP_DELAY_COUNT;
				oplus_enforce_chg_up_limit_result(chip, true);
			}
			return;
		} else if (chip->ui_soc >= chg_up_limit_data.charge_limit_recharge_value &&
			chg_up_limit_data.charge_limit_recharge_value < OPLUS_FULL_SOC) {
			over_count = 0;
			return;
		} else {
			over_count = 0;
			oplus_enforce_chg_up_limit_result(chip, false);
			return;
		}
	}
	over_count = 0;
	oplus_enforce_chg_up_limit_result(chip, false);
	return;
}

static void oplus_comm_set_ffc_step(struct oplus_chg_comm *chip, int step)
{
	struct mms_msg *msg;
	int rc;

	if (chip->ffc_step == step)
		return;
	chip->ffc_step = step;
	if (!chip->wired_online && !chip->wls_online)
		return;

	chg_info("ffc_step=%d\n", step);
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  COMM_ITEM_FFC_STEP);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish ffc_step msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

void oplus_comm_set_sale_mode(struct oplus_mms *topic, int sale_mode)
{
	struct oplus_chg_comm *chip;
	struct mms_msg *msg;
	int rc;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (chip->cool_down_sale_mode == sale_mode)
		return;

	mutex_lock(&chip->sale_mode_mutex);
	chip->cool_down_sale_mode = sale_mode;
	chg_info("set sale_mode to %d\n", sale_mode);
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  COMM_ITEM_SALE_MODE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		goto done;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish sale_mode msg error, rc=%d\n", rc);
		kfree(msg);
		goto done;
	}
done:
	mutex_unlock(&chip->sale_mode_mutex);
}

static void oplus_comm_set_ffc_status(struct oplus_chg_comm *chip,
				enum oplus_chg_ffc_status ffc_status)
{
	struct mms_msg *msg;
	int rc;

	if (chip->ffc_status == ffc_status)
		return;
	chip->ffc_status = ffc_status;

	chg_info("ffc_status=%d\n", ffc_status);
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  COMM_ITEM_FFC_STATUS);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish ffc_status msg error, rc=%d\n", rc);
		kfree(msg);
	}

	if (ffc_status == FFC_DEFAULT)
		oplus_comm_set_ffc_step(chip, 0);
}

int oplus_comm_switch_ffc(struct oplus_mms *topic)
{
	struct oplus_chg_comm *chip;
	enum oplus_ffc_temp_region ffc_temp_region;
	int rc;

	if (topic == NULL) {
		chg_err("topic is null\n");
		return -ENODEV;
	}
	chip = oplus_mms_get_drvdata(topic);
	oplus_comm_ffc_temp_thr_init(chip, FFC_TEMP_REGION_NORMAL);
	ffc_temp_region = oplus_comm_get_ffc_temp_region(chip);
	oplus_comm_ffc_temp_thr_init(chip, ffc_temp_region);

	oplus_comm_set_ffc_step(chip, 0);
	if (!chip->wired_online && !chip->wls_online) {
		chg_err("wired and wireless charge is offline\n");
		if (is_wired_charging_disable_votable_available(chip)) {
			vote(chip->wired_charging_disable_votable,
			     FASTCHG_VOTER, false, 0, false);
		}
		rc = -EINVAL;
		goto err;
	}

	chip->ffc_fcc_count = 0;
	chip->ffc_fv_count = 0;
	chip->ffc_charging = false;
	oplus_comm_set_ffc_status(chip, FFC_WAIT);
	schedule_delayed_work(&chip->ffc_start_work, FFC_START_DELAY);

	return 0;

err:
	chip->ffc_fcc_count = 0;
	chip->ffc_fv_count = 0;
	chip->ffc_charging = false;
	oplus_comm_set_ffc_status(chip, FFC_DEFAULT);
	return rc;
}

static void oplus_comm_ffc_start_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_comm *chip = container_of(dwork,
		struct oplus_chg_comm, ffc_start_work);
	struct oplus_comm_spec_config *spec = &chip->spec;
	enum oplus_ffc_temp_region ffc_temp_region;

	if (spec->full_pre_ffc_judge) {
		if (chip->vbat_mv_max >= spec->full_pre_ffc_mv && chip->soc == NORMAL_FULL_SOC) {
			chg_err("set batttery full, dont enter FFC/CV, chip->vbat_mv=%d\n", chip->vbat_mv_max);
			oplus_comm_set_batt_full(chip, true);
			if (chip->wls_online) {
				if (is_wls_charging_disable_votable_available(chip)) {
					vote(chip->wls_charging_disable_votable, CHG_FULL_VOTER, true, 1, false);
					if (is_wls_icl_votable_available(chip))
						vote(chip->wls_icl_votable, CHG_FULL_VOTER, true, 300, true);
					if (chip->rechg_soc_en && is_wls_fastchg_disable_votable_available(chip) &&
					    is_wls_comu_votable_available(chip)) {
						vote(chip->wls_fastchg_disable_votable, CHG_FULL_VOTER, true, 1, false);
						vote(chip->wls_comu_votable, CHG_FULL_VOTER, true, WLS_RX_COMU_CAP_AB, true);
					}
				} else {
					chg_err("wls_charging_disable_votable not found, can't disable charging");
				}
			} else {
				if (is_wired_charging_disable_votable_available(chip)) {
					vote(chip->wired_charging_disable_votable, CHG_FULL_VOTER, true, 1, false);
					vote(chip->wired_charging_disable_votable, FASTCHG_VOTER, false, 0, false);
				} else {
					chg_err("wired_charging_disable_votable not found, can't disable charging");
				}
			}
			goto err;
		}
	}

	ffc_temp_region = oplus_comm_get_ffc_temp_region(chip);
	oplus_comm_ffc_temp_thr_init(chip, ffc_temp_region);
	if (ffc_temp_region < FFC_TEMP_REGION_PRE_NORMAL || ffc_temp_region > FFC_TEMP_REGION_NORMAL) {
		chg_err("FFC charging is not possible in this temp region, temp_region=%s\n",
			oplus_comm_get_ffc_temp_region_str(ffc_temp_region));
		if (is_wired_charging_disable_votable_available(chip)) {
			vote(chip->wired_charging_disable_votable,
			     FASTCHG_VOTER, false, 0, false);
		}
		goto err;
	}

	chip->ffc_temp_region = ffc_temp_region;
	oplus_comm_set_ffc_step(chip, 0);
	if (chip->wired_online) {
		if (!is_wired_fcc_votable_available(chip)) {
			chg_err("wired_fcc_votable not found\n");
			goto err;
		}
		if (!is_wired_charging_disable_votable_available(chip)) {
			chg_err("wired_charging_disable_votable not found\n");
			goto err;
		}
		vote(chip->fv_max_votable, FFC_VOTER, true,
		     spec->wired_ffc_fv_mv[chip->ffc_step], false);
		vote(chip->wired_fcc_votable, FFC_VOTER, true,
		     spec->wired_ffc_fcc_ma[chip->ffc_step][ffc_temp_region - 1],
		     false);
		vote(chip->wired_charging_disable_votable, FASTCHG_VOTER, false,
		     0, false);
	} else if (chip->wls_online) {
		if (!is_wls_icl_votable_available(chip)) {
			chg_err("wls_icl_votable not found\n");
			goto err;
		}
		if (!is_wls_fcc_votable_available(chip)) {
			chg_err("wls_fcc_votable not found\n");
			goto err;
		}
		vote(chip->wls_icl_votable, FFC_VOTER, true,
		     spec->wls_ffc_icl_ma[chip->ffc_step][ffc_temp_region - 1],
		     true);
		vote(chip->fv_max_votable, FFC_VOTER, true,
		     spec->wls_ffc_fv_mv[chip->ffc_step], false);
		vote(chip->wls_fcc_votable, FFC_VOTER, true,
		     spec->wls_ffc_fcc_ma[chip->ffc_step][ffc_temp_region - 1],
		     false);
	} else {
		chg_err("wired and wireless charge is offline\n");
		goto err;
	}

	chip->ffc_fcc_count = 0;
	chip->ffc_fv_count = 0;
	chip->ffc_charging = true;
	oplus_comm_set_ffc_status(chip, FFC_FAST);
	return;

err:
	chip->ffc_fcc_count = 0;
	chip->ffc_fv_count = 0;
	chip->ffc_charging = false;
	oplus_comm_set_ffc_status(chip, FFC_DEFAULT);
}

static int oplus_comm_get_aging_ffc_offset(struct oplus_chg_comm *chip, int step)
{
	struct oplus_comm_spec_config *spec = &chip->spec;
	int i;

	if (spec->wired_aging_ffc_version == AGING_FFC_NOT_SUPPORT)
		return 0;

	/* TODO: parallel charger */

	for (i = AGAIN_FFC_CYCLY_THR_COUNT - 1; i >= 0; i--) {
		if (chip->batt_cc >= spec->wired_aging_ffc_cycle_thr[i])
			return spec->wired_aging_ffc_offset_mv[step][i];
	}

	return 0;
}

static int oplus_comm_check_sub_vbat_ffc_end(
	struct oplus_chg_comm *chip, int step, int ffc_temp_region)
{
	struct oplus_comm_spec_config *spec = &chip->spec;
	int fv_max_mv;

	if (chip->wired_online) {
		fv_max_mv = spec->wired_ffc_fv_cutoff_mv[step][ffc_temp_region - 1];
		fv_max_mv += oplus_comm_get_aging_ffc_offset(chip, step);
		fv_max_mv -= spec->dec_cv.dec_vol;
	} else if (chip->wls_online) {
		fv_max_mv = spec->wls_ffc_fv_cutoff_mv[step][ffc_temp_region - 1];
		fv_max_mv -= spec->dec_cv.dec_vol;
	} else {
		chg_err("wired and wireless charge is offline\n");
		return false;
	}

	if (chip->sub_vbat_mv > fv_max_mv) {
		chg_info("sub_vbat_mv(=%d) > fv_max_mv(=%d)\n",
			 chip->sub_vbat_mv, fv_max_mv);
		return true;
	} else {
		return false;
	}
}

static int oplus_comm_check_sub_ibat_ffc_end(
	struct oplus_chg_comm *chip, int step, int ffc_temp_region)
{
	struct oplus_comm_spec_config *spec = &chip->spec;
	int cutoff_ma;

	if (chip->wired_online) {
		cutoff_ma = spec->wired_ffc_fcc_sub_cutoff_ma[step][ffc_temp_region - 1];
	} else if (chip->wls_online) {
		cutoff_ma = spec->wls_ffc_fcc_sub_cutoff_ma[step][ffc_temp_region - 1];
	} else {
		chg_err("wired and wireless charge is offline\n");
		return false;
	}

	if (abs(chip->sub_ibat_ma) < cutoff_ma) {
		chg_info("sub_ibat_ma(=%d) < cutoff_ma(=%d)\n",
			 chip->sub_ibat_ma, cutoff_ma);
		return true;
	} else {
		return false;
	}
}

static void oplus_ffc_exit_reset_info(struct oplus_chg_comm *chip)
{
	vote(chip->fv_max_votable, FFC_VOTER, false, 0, false);

	if (is_wired_fcc_votable_available(chip) && chip->wired_online) {
		vote(chip->wired_fcc_votable, FFC_VOTER, false, 0,
		     false);
	}
}

static void oplus_comm_check_ffc(struct oplus_chg_comm *chip)
{
	struct oplus_comm_spec_config *spec = &chip->spec;
	enum oplus_ffc_temp_region ffc_temp_region;
	int fv_max_mv, cutoff_ma, step_max, ffc_step;
	bool sub_vbat_ffc_end_status = 0;
	bool sub_ibat_ffc_end_status = 0;

	if (!chip->ffc_charging)
		return;

	ffc_temp_region = oplus_comm_get_ffc_temp_region(chip);
	oplus_comm_ffc_temp_thr_init(chip, ffc_temp_region);

	switch (chip->ffc_status) {
	case FFC_FAST:
		if (ffc_temp_region < FFC_TEMP_REGION_PRE_NORMAL || ffc_temp_region > FFC_TEMP_REGION_NORMAL) {
			chg_err("FFC charging is not possible in this temp region, temp_region=%s\n",
				oplus_comm_get_ffc_temp_region_str(ffc_temp_region));
			oplus_ffc_exit_reset_info(chip);
			if (chip->wired_online) {
				if (chip->soc >= 100)
					goto err;
				if (is_wired_charging_disable_votable_available(chip))
					vote(chip->wired_charging_disable_votable, FFC_VOTER, true, 1, false);
				oplus_comm_set_ffc_status(chip, FFC_IDLE);
				return;
			} else {
				goto err;
			}
		}
		if (chip->ffc_temp_region != ffc_temp_region) {
			chip->ffc_temp_region = ffc_temp_region;
			if (chip->wired_online) {
				if (!is_wired_fcc_votable_available(chip)) {
					chg_err("wired_fcc_votable not found\n");
					goto err;
				}
				vote(chip->wired_fcc_votable, FFC_VOTER, true,
				     spec->wired_ffc_fcc_ma[chip->ffc_step]
							   [ffc_temp_region - 1],
				     false);
			} else if (chip->wls_online) {
				if (!is_wls_icl_votable_available(chip)) {
					chg_err("wls_icl_votable not found\n");
					goto err;
				}
				if (!is_wls_fcc_votable_available(chip)) {
					chg_err("wls_fcc_votable not found\n");
					goto err;
				}
				vote(chip->wls_icl_votable, FFC_VOTER, true,
				     spec->wls_ffc_icl_ma[chip->ffc_step][ffc_temp_region - 1], true);
				vote(chip->wls_fcc_votable, FFC_VOTER, true,
				     spec->wls_ffc_fcc_ma[chip->ffc_step][ffc_temp_region - 1], false);
			} else {
				chg_err("wired and wireless charge is offline\n");
				goto err;
			}
		}
		if (chip->wired_online) {
			fv_max_mv = spec->wired_ffc_fv_cutoff_mv[chip->ffc_step][ffc_temp_region - 1];
			fv_max_mv += oplus_comm_get_aging_ffc_offset(chip, chip->ffc_step);
			fv_max_mv -= spec->dec_cv.dec_vol;
			cutoff_ma = spec->wired_ffc_fcc_cutoff_ma[chip->ffc_step][ffc_temp_region - 1];
			step_max = spec->wired_ffc_step_max;
		} else if (chip->wls_online) {
			fv_max_mv = spec->wls_ffc_fv_cutoff_mv[chip->ffc_step][ffc_temp_region - 1];
			fv_max_mv -= spec->dec_cv.dec_vol;
			cutoff_ma = spec->wls_ffc_fcc_cutoff_ma[chip->ffc_step][ffc_temp_region - 1];
			step_max = spec->wls_ffc_step_max;
		} else {
			chg_err("wired and wireless charge is offline\n");
			goto err;
		}
		oplus_comm_set_ffc_cutoff_curr(chip, cutoff_ma, ffc_temp_region);
		if (is_support_parallel_battery(chip->gauge_topic)) {
			sub_vbat_ffc_end_status = oplus_comm_check_sub_vbat_ffc_end(
				chip, chip->ffc_step, ffc_temp_region);
			sub_ibat_ffc_end_status = oplus_comm_check_sub_ibat_ffc_end(
				chip, chip->ffc_step, ffc_temp_region);
		}
		ffc_step = chip->ffc_step;
		if (chip->vbat_mv >= fv_max_mv || sub_vbat_ffc_end_status)
			chip->ffc_fv_count++;
		if (chip->ffc_fv_count >= FFC_VOLT_COUNTS) {
			ffc_step = chip->ffc_step + 1;
			chip->ffc_fv_count = 0;
			chip->ffc_fcc_count = 0;
			chg_info("vbat_mv(=%d) > fv_max_mv(=%d), switch to next step(=%d)\n",
				 chip->vbat_mv, fv_max_mv, ffc_step);
			goto ffc_step_done;
		}
		if (abs(chip->ibat_ma) < cutoff_ma || sub_ibat_ffc_end_status)
			chip->ffc_fcc_count++;
		else
			chip->ffc_fcc_count = 0;
		if (chip->ffc_fcc_count >= FFC_CURRENT_COUNTS) {
			ffc_step = chip->ffc_step + 1;
			chip->ffc_fv_count = 0;
			chip->ffc_fcc_count = 0;
			chg_info("ibat_ma(=%d) > cutoff_ma(=%d), switch to next step(=%d)\n",
				chip->ibat_ma, cutoff_ma, ffc_step);
		}
ffc_step_done:
		if (ffc_step >= step_max) {
			chg_info("ffc charge done\n");
			oplus_ffc_exit_reset_info(chip);
			chip->ffc_fcc_count = 0;
			chip->ffc_fv_count = 0;
			if (chip->wired_online) {
				if (chip->soc >= 100) {
					chip->ffc_charging = false;
					oplus_comm_set_ffc_status(chip, FFC_DEFAULT);
				} else {
					if (is_wired_charging_disable_votable_available(chip))
						vote(chip->wired_charging_disable_votable, FFC_VOTER, true, 1, false);
					oplus_comm_set_ffc_status(chip, FFC_IDLE);
				}
			} else {
				oplus_comm_set_ffc_status(chip, FFC_IDLE);
			}
		} else if (ffc_step != chip->ffc_step) {
			oplus_comm_set_ffc_step(chip, ffc_step);
			if (chip->wired_online) {
				if (!is_wired_fcc_votable_available(chip)) {
					chg_err("wired_fcc_votable not found\n");
					goto err;
				}
				vote(chip->fv_max_votable, FFC_VOTER, true, spec->wired_ffc_fv_mv[chip->ffc_step],
				     false);
				vote(chip->wired_fcc_votable, FFC_VOTER, true,
				     spec->wired_ffc_fcc_ma[chip->ffc_step][ffc_temp_region - 1], false);
			} else if (chip->wls_online) {
				if (!is_wls_icl_votable_available(chip)) {
					chg_err("wls_icl_votable not found\n");
					goto err;
				}
				if (!is_wls_fcc_votable_available(chip)) {
					chg_err("wls_fcc_votable not found\n");
					goto err;
				}
				vote(chip->wls_icl_votable, FFC_VOTER, true,
				     spec->wls_ffc_icl_ma[chip->ffc_step][ffc_temp_region - 1], true);
				vote(chip->fv_max_votable, FFC_VOTER, true, spec->wls_ffc_fv_mv[chip->ffc_step], false);
				vote(chip->wls_fcc_votable, FFC_VOTER, true,
				     spec->wls_ffc_fcc_ma[chip->ffc_step][ffc_temp_region - 1], false);
			}
		}
		break;
	case FFC_IDLE:
		chip->ffc_fcc_count++;
		if (chip->ffc_fcc_count >= 12) {
			chip->ffc_charging = false;
			chip->ffc_fcc_count = 0;
			chip->ffc_fv_count = 0;
			if (chip->wired_online) {
				if (!is_wired_charging_disable_votable_available(chip)) {
					chg_err("wired_charging_disable_votable not found\n");
					goto err;
				}
				vote(chip->wired_charging_disable_votable, FFC_VOTER, false, 0, false);
			} else if (chip->wls_online) {
				if (!is_wls_charging_disable_votable_available(chip)) {
					chg_err("wls_charging_disable_votable not found\n");
					goto err;
				}
				vote(chip->wls_charging_disable_votable, FFC_VOTER, false, 0, false);
			}
			oplus_comm_set_ffc_status(chip, FFC_DEFAULT);
			return;
		}
		break;
	default:
		goto err;
	}

	return;

err:
	chip->ffc_charging = false;
	chip->ffc_fcc_count = 0;
	chip->ffc_fv_count = 0;
	oplus_comm_set_ffc_status(chip, FFC_DEFAULT);
}

static void oplus_comm_fginfo_reset(struct oplus_chg_comm *chip)
{
	chip->fg_soft_reset_done = false;
	chip->fg_check_ibat_cnt = 0;
	chip->fg_soft_reset_fail_cnt = 0;
	cancel_delayed_work_sync(&chip->fg_soft_reset_work);
}

static void oplus_comm_check_fgreset(struct oplus_chg_comm *chip)
{
	bool is_need_check = true;

	if (oplus_wired_get_chg_type() == OPLUS_CHG_USB_TYPE_UNKNOWN ||
	    oplus_wired_get_chg_type() == OPLUS_CHG_USB_TYPE_SDP ||
	    chip->temp_region != TEMP_REGION_NORMAL ||
	    chip->vbat_min_mv < SOFT_REST_VOL_THRESHOLD ||
	    chip->fg_soft_reset_done)
		is_need_check = false;

	if (!chip->fg_soft_reset_done &&
	    chip->fg_soft_reset_fail_cnt > SOFT_REST_RETRY_MAX_CNT)
		is_need_check = false;

	if (fg_reset_test)
		is_need_check = true;

	if (!chip->sw_full && !chip->hw_full_by_sw)
		is_need_check = false;

	if (!chip->wired_online && !chip->wls_online)
		is_need_check = false;

	if (oplus_gauge_afi_update_done() == false) {
		chg_info("zy gauge afi_update_done ing...\n");
		is_need_check = false;
	}

	if (!is_need_check) {
		chip->fg_check_ibat_cnt = 0;
		return;
	}

	schedule_delayed_work(&chip->fg_soft_reset_work, 0);
}

static int oplus_comm_charging_disable(struct oplus_chg_comm *chip, bool en)
{
	struct mms_msg *msg;
	int rc;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH,
				  COMM_ITEM_CHARGING_DISABLE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish charging disable msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static int oplus_comm_charge_suspend(struct oplus_chg_comm *chip, bool en)
{
	struct mms_msg *msg;
	int rc;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH,
				  COMM_ITEM_CHARGE_SUSPEND);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish charge suspend msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static int oplus_comm_set_cool_down_level(struct oplus_chg_comm *chip, int level)
{
	struct mms_msg *msg;
	int rc;

	if (chip->cool_down == level)
		return 0;
	chip->cool_down = level;
	chg_info("set cool_down=%d\n", level);

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  COMM_ITEM_COOL_DOWN);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish cool down msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

#define MULTIPLE 10
#define CNT_TIMELIMIT 1800
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
static void oplus_chg_gauge_stuck(struct oplus_chg_comm *chip)
{
	static int first_soc = 0;
	static int gauge_stuck_count = 0;
	static int theory_current_sum = 0;
	static int current_sum = 0;
	static unsigned long cnt_time = 0;
	static bool first_flag = true;
	union mms_msg_data data = { 0 };
	struct oplus_comm_spec_config *spec = &chip->spec;
	static bool last_gauge_stuck_flag = false;
	bool gauge_stuck_flag = false;

	if (first_flag) {
		first_flag = false;
		cnt_time = CNT_TIMELIMIT * HZ;
		cnt_time += jiffies;
		first_soc = chip->soc;
	}

	if (chip->soc != first_soc) {
		first_soc = chip->soc;
		current_sum = 0;
		theory_current_sum = 0;
		cnt_time = CNT_TIMELIMIT * HZ;
		cnt_time += jiffies;
		last_gauge_stuck_flag = false;
	}

	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_FCC, &data, false);
	chip->batt_fcc = data.intval;
	current_sum += chip->ibat_ma;
	theory_current_sum += (chip->batt_fcc / spec->gauge_stuck_time);

	if (time_after_eq(jiffies, cnt_time) || chip->soc == 100) {
		cnt_time = CNT_TIMELIMIT * HZ;
		cnt_time += jiffies;
		chg_info("normal in, current_sum=%d, theory_current_sum=%d, jiffies=%ld\n",
		           current_sum, theory_current_sum, jiffies);

		gauge_stuck_flag = (abs(current_sum) > (spec->gauge_stuck_threshold * theory_current_sum) / MULTIPLE)
		                    && (!abs(chip->soc - first_soc)) && (chip->soc != 100);

		if (gauge_stuck_flag && last_gauge_stuck_flag) {
			chip->gauge_stuck = true;
			last_gauge_stuck_flag = false;
			gauge_stuck_flag = false;
			chg_err("gauge_stuck_count = %d\n", ++gauge_stuck_count);
		}
		last_gauge_stuck_flag = gauge_stuck_flag;

		first_soc = chip->soc;
		current_sum = 0;
		theory_current_sum = 0;
	}
}

#define OPLUS_NOTIFY_SOC_JUMP_THD			3
static void oplus_comm_battery_notify_soc_check(struct oplus_chg_comm *chip)
{
	int smooth_soc;
	int uisoc;
	static int pre_smooth_soc = 0;
	static int pre_uisoc = 0;

	smooth_soc = chip->smooth_soc;
	uisoc = chip->ui_soc;
	if (!pre_smooth_soc && !pre_uisoc) {
		pre_smooth_soc = chip->smooth_soc;
		pre_uisoc = chip->ui_soc;
	}

	if (smooth_soc == pre_smooth_soc && uisoc == pre_uisoc && uisoc == pre_smooth_soc) {
		chip->gauge_soc_jump = false;
	} else if (abs(pre_smooth_soc - smooth_soc) > OPLUS_NOTIFY_SOC_JUMP_THD) {
		chip->gauge_soc_jump = true;
		chg_err("The gap between smooth_soc and pre_smooth_soc is too large\n");
	} else if (abs(pre_uisoc - uisoc) > OPLUS_NOTIFY_SOC_JUMP_THD) {
		chip->gauge_soc_jump = true;
		chg_err("The gap between uisoc and pre_uisoc is too large\n");
	} else if (abs(uisoc - pre_smooth_soc) > OPLUS_NOTIFY_SOC_JUMP_THD) {
		chip->gauge_soc_jump = true;
		chg_err("The gap between pre_smooth_soc and uisoc is too large\n");
	}

	pre_smooth_soc = smooth_soc;
	pre_uisoc = uisoc;
}
#endif /*CONFIG_DISABLE_OPLUS_FUNCTION*/

static void oplus_comm_battery_notify_tbat_check(struct oplus_chg_comm *chip,
						 unsigned int *notify_code)
{
	static int count_removed = 0;
	static int count_high = 0;

	if (!chip->wired_online)
		chip->low_temp_check_jiffies = jiffies;

	if (chip->temp_region == TEMP_REGION_HOT) {
		count_high++;
		if (count_high > 10) {
			count_high = 11;
			*notify_code |= BIT(NOTIFY_BAT_OVER_TEMP);
			chg_err("bat_temp(%d) > 53'C\n", chip->shell_temp);
		}
	} else {
		count_high = 0;
	}
	if (chip->temp_region == TEMP_REGION_COLD) {
		if (chip->batt_temp < chip->spec.removed_bat_decidegc) {
			chip->low_temp_check_jiffies = jiffies;
			*notify_code |= BIT(NOTIFY_BAT_NOT_CONNECT);
			chg_err("bat_temp(%d) < %d mC, Battery does not exist\n",
				chip->batt_temp, chip->spec.removed_bat_decidegc);
		} else {
			if (time_is_before_eq_jiffies(chip->low_temp_check_jiffies + (unsigned long)(15 * HZ))) {
				*notify_code |= BIT(NOTIFY_BAT_LOW_TEMP);
				chg_err("bat_temp(%d) < -10'C\n", chip->shell_temp);
			}
		}
	} else {
		chip->low_temp_check_jiffies = jiffies;
	}
	if (!chip->batt_exist) {
		count_removed++;
		if (count_removed > 10) {
			count_removed = 11;
			*notify_code |= BIT(NOTIFY_BAT_NOT_CONNECT);
			chg_err("Battery does not exist\n");
		}
	} else {
		count_removed = 0;
	}
}

#define PRE_HIGH_TEMP_RECOVERY_TIME	5
static void oplus_comm_battery_notify_vbat_check(struct oplus_chg_comm *chip,
						 unsigned int *notify_code)
{
	static int count = 0;
	static int pre_notify_code = 0;
	static bool pre_high_recov = false;
	static unsigned long pre_high_recov_jiffies = 0;

	if (chip->vbatt_over) {
		count++;
		chg_err("Battery is over VOL, count=%d\n", count);
		if (count > 10) {
			count = 11;
			*notify_code |= BIT(NOTIFY_BAT_OVER_VOL);
			chg_err("Battery is over VOL!, NOTIFY\n");
		}
	} else {
		count = 0;
		if (chip->batt_full) {
			if (chip->temp_region == TEMP_REGION_WARM &&
			    chip->ui_soc != 100) {
				*notify_code |=
					BIT(NOTIFY_BAT_FULL_PRE_HIGH_TEMP);
			} else if ((chip->temp_region == TEMP_REGION_COLD) &&
				   (chip->ui_soc != 100)) {
				*notify_code |=
					BIT(NOTIFY_BAT_FULL_PRE_LOW_TEMP);
			} else if (!chip->authenticate) {
				*notify_code |= BIT(NOTIFY_BAT_NOT_CONNECT);
			} else if (!chip->hmac) {
				*notify_code |=
					BIT(NOTIFY_BAT_FULL_THIRD_BATTERY);
			} else {
				if (chip->ui_soc == 100) {
					*notify_code |= 1 << NOTIFY_BAT_FULL;
				}
			}
		}
	}
	/*
	* add for UI show FULL issue. If notify code recovery too fast,
	* framework get normal temp and status full togeter sometimes
	*/
	if (pre_notify_code & BIT(NOTIFY_BAT_FULL_PRE_HIGH_TEMP) &&
	    !(*notify_code & BIT(NOTIFY_BAT_FULL_PRE_HIGH_TEMP))) {
		if (!pre_high_recov)
			pre_high_recov_jiffies =
				jiffies + (unsigned long)(PRE_HIGH_TEMP_RECOVERY_TIME * HZ);
		pre_high_recov = true;
		if (time_is_before_jiffies(pre_high_recov_jiffies)) {
			pre_high_recov = false;
			pre_notify_code = *notify_code;
		} else {
			*notify_code |= BIT(NOTIFY_BAT_FULL_PRE_HIGH_TEMP);
			chg_err("keep PRE_HIGH_TEMP notify_code\n");
		}
	} else {
		pre_high_recov = false;
		pre_notify_code = *notify_code;
	}
}

static int oplus_comm_set_notify_code(struct oplus_chg_comm *chip,
				      unsigned int notify_code)
{
	struct mms_msg *msg;
	int rc;

	if (chip->notify_code == notify_code)
		return 0;

	chip->notify_code = notify_code;
	chg_info("set notify_code=%08x\n", notify_code);

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  COMM_ITEM_NOTIFY_CODE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish notify code msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static int oplus_comm_set_notify_flag(struct oplus_chg_comm *chip,
				      unsigned int notify_flag)
{
	struct mms_msg *msg;
	int rc;

	if (chip->notify_flag == notify_flag)
		return 0;

	chip->notify_flag = notify_flag;
	chg_info("set notify_flag=%d\n", notify_flag);

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  COMM_ITEM_NOTIFY_FLAG);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish notify flag msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static void oplus_comm_battery_notify_check(struct oplus_chg_comm *chip)
{
	unsigned int notify_code = 0;
	union mms_msg_data data = { 0 };

	if (!IS_ERR_OR_NULL(chip->gauge_topic)) {
		oplus_comm_battery_notify_tbat_check(chip, &notify_code);
		if (!chip->authenticate)
			notify_code |= BIT(NOTIFY_BAT_NOT_CONNECT);
		if (!chip->hmac)
			notify_code |= BIT(NOTIFY_BAT_FULL_THIRD_BATTERY);
	}

	if (chip->wired_online) {
		if (chip->wired_err_code & BIT(OPLUS_ERR_CODE_OVP))
			notify_code |= BIT(NOTIFY_CHARGER_OVER_VOL);
		if (chip->wired_err_code & BIT(OPLUS_ERR_CODE_UVP))
			notify_code |= BIT(NOTIFY_CHARGER_LOW_VOL);
	} else if (chip->wls_online) {
		if (chip->wls_err_code & BIT(OPLUS_ERR_CODE_OVP))
			notify_code |= BIT(NOTIFY_CHARGER_OVER_VOL);
		if (chip->wls_err_code & BIT(OPLUS_ERR_CODE_UVP))
			notify_code |= BIT(NOTIFY_CHARGER_LOW_VOL);
	}

	if (!IS_ERR_OR_NULL(chip->gauge_topic))
		oplus_comm_battery_notify_vbat_check(chip, &notify_code);

	if (chip->chging_over_time)
		notify_code |= BIT(NOTIFY_CHGING_OVERTIME);
	if (chip->batt_full)
		notify_code |= BIT(NOTIFY_CHARGER_TERMINAL);
	if (chip->gauge_err_code & BIT(OPLUS_ERR_CODE_I2C))
		notify_code |= BIT(NOTIFY_GAUGE_I2C_ERR);

	if (notify_code &
	    (BIT(NOTIFY_BAT_NOT_CONNECT) | BIT(NOTIFY_GAUGE_I2C_ERR)))
		notify_code &= ~BIT(NOTIFY_BAT_LOW_TEMP);

	if (chip->gauge_stuck) {
		chip->gauge_stuck = false;
		notify_code |= BIT(NOTIFY_GAUGE_STUCK);
	}

	if (chip->anti_expansion_warning) {
		chip->anti_expansion_warning = false;
		notify_code |= BIT(NOTIFY_ANTI_EXPANSION_WARNING);
	}

	if (chip->anti_expansion_error)
		notify_code |= BIT(NOTIFY_ANTI_EXPANSION_ERROR);
	else
		notify_code &= ~BIT(NOTIFY_ANTI_EXPANSION_ERROR);

	if (chip->gauge_soc_jump)
		notify_code |= BIT(NOTIFY_GAUGE_SOC_JUMP);
	if (is_parallel_topic_available(chip)) {
		oplus_mms_get_item_data(chip->parallel_topic,
					SWITCH_ITEM_ERR_NOTIFY,
					&data, true);
		if (data.intval)
			notify_code |= data.intval;
	}

	oplus_comm_set_notify_code(chip, notify_code);
}

void oplus_comm_set_bdd_voltdiff_trend(struct oplus_mms *topic, int trend)
{
	struct oplus_chg_comm *chip;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return;
	}
	chip = oplus_mms_get_drvdata(topic);

	chip->bdd_voltdiff_trend = trend;
}

int oplus_comm_get_bdd_voltdiff_trend(struct oplus_mms *topic)
{
	struct oplus_chg_comm *chip;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -ENODEV;
	}
	chip = oplus_mms_get_drvdata(topic);

	return chip->bdd_voltdiff_trend;
}

#define ABNORMAL_COUNT_THRESHOLD 6
static void oplus_comm_dual_volt_diff_check(struct oplus_chg_comm *chip)
{
	int vol_diff = 0;
	static int start_count = 0;
	static int confirm_count = 0;
	static int abnormal_diff = 0;
	static unsigned long bdd_start_state_tm_sec;
	static bool bdd_start_state_tm_valid;
	unsigned long now_tm_sec = 0;
	unsigned long timeout_sec = 0;

	if (abs(chip->ibat_ma) >= chip->spec.bdd_enter_current_ma)
		return;

	if (chip->spec.bdd_vbatdiff_timeout_min > 0)
		timeout_sec = (unsigned long)chip->spec.bdd_vbatdiff_timeout_min * 60;

	if (timeout_sec > 0) {
		if (chip->bdd_voltdiff_trend == BDD_VOLT_DIFF_TREND_START &&
		    bdd_start_state_tm_valid &&
		    !oplus_comm_get_current_time(&now_tm_sec)) {
			if (now_tm_sec >= bdd_start_state_tm_sec + timeout_sec) {
				chg_info("bdd volt diff START state timeout, reset state\n");
				start_count = 0;
				confirm_count = 0;
				abnormal_diff = 0;
				bdd_start_state_tm_valid = false;
				oplus_comm_set_bdd_voltdiff_trend(chip->comm_topic, BDD_VOLT_DIFF_TREND_TIMEOUT);
			}
		} else if (chip->bdd_voltdiff_trend != BDD_VOLT_DIFF_TREND_START) {
			bdd_start_state_tm_valid = false;
		}
	}

	chg_info("ibat_ma=%d\n", chip->ibat_ma);
	vol_diff = abs(chip->vbat_mv - chip->vbat_min_mv);

	if (vol_diff > chip->spec.bdd_enter_voltage_mv) {
		if (++start_count > ABNORMAL_COUNT_THRESHOLD) {
			if (start_count == ABNORMAL_COUNT_THRESHOLD + 1)
				abnormal_diff = vol_diff;
			if (vol_diff <= abnormal_diff - chip->spec.bdd_vbatdiff_mv) {
				start_count = 0;
				confirm_count = 0;
				abnormal_diff = 0;
				bdd_start_state_tm_valid = false;
				oplus_comm_set_bdd_voltdiff_trend(chip->comm_topic, BDD_VOLT_DIFF_TREND_NONE);
				return;
			}
			if (timeout_sec > 0 && !bdd_start_state_tm_valid &&
				!oplus_comm_get_current_time(&bdd_start_state_tm_sec))
				bdd_start_state_tm_valid = true;
			oplus_comm_set_bdd_voltdiff_trend(chip->comm_topic, BDD_VOLT_DIFF_TREND_START);
			chg_info("volt diff > bdd_enter_voltage_mv, %d\n", vol_diff);

			if (vol_diff > abnormal_diff + chip->spec.bdd_vbatdiff_mv) {
				if (++confirm_count > ABNORMAL_COUNT_THRESHOLD) {
					oplus_comm_set_bdd_voltdiff_trend(chip->comm_topic, BDD_VOLT_DIFF_TREND_CONFIRM);
					bdd_start_state_tm_valid = false;
					chg_info("volt diff > CONFIRM_ABNORMAL_DIFF, %d\n", vol_diff);
					start_count = 0;
					confirm_count = 0;
					return;
				}
			} else {
				confirm_count = 0;
			}
		}
	} else if (vol_diff <= abnormal_diff - chip->spec.bdd_vbatdiff_mv || start_count <= ABNORMAL_COUNT_THRESHOLD) {
		start_count = 0;
		confirm_count = 0;
		abnormal_diff = 0;
		bdd_start_state_tm_valid = false;
		if (chip->bdd_voltdiff_trend == BDD_VOLT_DIFF_TREND_TIMEOUT)
			oplus_comm_set_bdd_voltdiff_trend(chip->comm_topic, BDD_VOLT_DIFF_TREND_TIMEOUT);
		else
			oplus_comm_set_bdd_voltdiff_trend(chip->comm_topic, BDD_VOLT_DIFF_TREND_NONE);
	}
}

static void oplus_comm_battery_notify_flag_check(struct oplus_chg_comm *chip)
{
	unsigned int notify_flag = 0;

	if (chip->notify_code & (1 << NOTIFY_CHGING_OVERTIME)) {
		notify_flag = NOTIFY_CHGING_OVERTIME;
	} else if (chip->notify_code & (1 << NOTIFY_CHARGER_OVER_VOL)) {
		notify_flag = NOTIFY_CHARGER_OVER_VOL;
	} else if (chip->notify_code & (1 << NOTIFY_CHARGER_LOW_VOL)) {
		notify_flag = NOTIFY_CHARGER_LOW_VOL;
	} else if (chip->notify_code & (1 << NOTIFY_BAT_OVER_TEMP)) {
		notify_flag = NOTIFY_BAT_OVER_TEMP;
	} else if (chip->notify_code & (1 << NOTIFY_BAT_LOW_TEMP)) {
		notify_flag = NOTIFY_BAT_LOW_TEMP;
	} else if (chip->notify_code & (1 << NOTIFY_BAT_NOT_CONNECT)) {
		notify_flag = NOTIFY_BAT_NOT_CONNECT;
	} else if (chip->notify_code & (1 << NOTIFY_BAT_FULL_THIRD_BATTERY)) {
		notify_flag = NOTIFY_BAT_FULL_THIRD_BATTERY;
	} else if (chip->notify_code & (1 << NOTIFY_BAT_OVER_VOL)) {
		notify_flag = NOTIFY_BAT_OVER_VOL;
	} else if (chip->notify_code & (1 << NOTIFY_BAT_FULL_PRE_HIGH_TEMP)) {
		notify_flag = NOTIFY_BAT_FULL_PRE_HIGH_TEMP;
	} else if (chip->notify_code & (1 << NOTIFY_BAT_FULL_PRE_LOW_TEMP)) {
		notify_flag = NOTIFY_BAT_FULL_PRE_LOW_TEMP;
	} else if (chip->notify_code & (1 << NOTIFY_BAT_FULL)) {
		notify_flag = NOTIFY_BAT_FULL;
	} else {
		notify_flag = 0;
	}

	oplus_comm_set_notify_flag(chip,notify_flag);
}

#define BATT_NTC_CTRL_THRESHOLD_LOW 320
#define BATT_NTC_CTRL_THRESHOLD_HIGH 600
static bool oplus_comm_override_by_shell_temp(struct oplus_chg_comm *chip,
					      int temp)
{
	struct oplus_comm_spec_config *spec = &chip->spec;
	int batt_temp_thr;

	if (oplus_is_power_off_charging())
		return false;

	if (chip->wls_online)
		return false;

	batt_temp_thr =
		min(spec->batt_temp_thr[ARRAY_SIZE(spec->batt_temp_thr) - 1],
		    BATT_NTC_CTRL_THRESHOLD_HIGH);
	if ((chip->wired_online || chip->high_reverse_charging) &&
	    (temp > BATT_NTC_CTRL_THRESHOLD_LOW) &&
	    (temp < batt_temp_thr))
		return true;

	return false;
}

void oplus_comm_set_anti_expansion_status(struct oplus_mms *topic,int val)
{
	struct oplus_chg_comm *chip;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return;
	}
	chip = oplus_mms_get_drvdata(topic);

	chip->anti_expansion_warning = (val & BIT(NOTIFY_ANTI_EXPANSION_WARNING));
	chip->anti_expansion_error = (val & BIT(NOTIFY_ANTI_EXPANSION_ERROR));
}

int oplus_comm_get_bms_heat_temp_compensation(struct oplus_mms *topic)
{
	struct oplus_chg_comm *chip;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -ENODEV;
	}
	chip = oplus_mms_get_drvdata(topic);

	return chip->bms_heat_temp_compensation;
}

void oplus_comm_set_bms_heat_temp_compensation(struct oplus_mms *topic, int bms_heat_temp_compensation)
{
	struct oplus_chg_comm *chip;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return;
	}
	chip = oplus_mms_get_drvdata(topic);

	chip->bms_heat_temp_compensation = bms_heat_temp_compensation;
}

static void oplus_comm_check_shell_temp(struct oplus_chg_comm *chip, bool update)
{
	struct mms_msg *msg;
	int shell_temp;
	int batt_temp_high;
	int diff, main_diff, sub_diff;
	int rc;
	struct thermal_zone_device *shell_themal = NULL;
	static bool update_pending = false;

	if (chip->shell_themal == NULL) {
		shell_themal =
			thermal_zone_get_zone_by_name("shell_back");
		if (IS_ERR(shell_themal)) {
			chg_err("Can't get shell_back\n");
			shell_themal = NULL;
		}
		chip->shell_themal = shell_themal;
	}

	batt_temp_high = chip->batt_temp_dynamic_thr[TEMP_REGION_WARM];

	if (IS_ERR_OR_NULL(chip->shell_themal)) {
		shell_temp = chip->batt_temp;
	} else {
		rc = thermal_zone_get_temp(chip->shell_themal, &shell_temp);

		if (rc) {
			chg_err("thermal_zone_get_temp get error");
			shell_temp = chip->batt_temp;
		} else {
			shell_temp = shell_temp / 100;
		}
		if (oplus_comm_override_by_shell_temp(chip, chip->batt_temp)) {
			diff = shell_temp - chip->batt_temp;
			if (is_support_parallel_battery(chip->gauge_topic)) {
				main_diff = shell_temp - chip->main_batt_temp;
				sub_diff = shell_temp - chip->sub_batt_temp;
				diff = abs(main_diff) > abs(sub_diff) ? abs(main_diff) : abs(sub_diff);
			}
			if (abs(diff) >= 150 || chip->batt_temp < 320 || chip->batt_temp > batt_temp_high)
				shell_temp = chip->batt_temp;
		} else {
			shell_temp = chip->batt_temp;
		}
	}

	if (chip->bms_heat_temp_compensation != 0 && shell_temp < OPLUS_BMS_HEAT_THRE)
		shell_temp = shell_temp - chip->bms_heat_temp_compensation;

	if ((chip->shell_temp == shell_temp) && !update_pending)
		return;
	chip->shell_temp = shell_temp;

	if (oplus_dbg_tbat) {
		chip->shell_temp = oplus_dbg_tbat;
		shell_temp = oplus_dbg_tbat;
	}

	if (!update) {
		update_pending = true;
		return;
	} else {
		update_pending = false;
	}

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH,
				  COMM_ITEM_SHELL_TEMP);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish shell temp msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_comm_hw_iterm_check(struct oplus_chg_comm *chip)
{
	static bool pre_full = false;

	if (!chip->config.support_hw_iterm_check)
		return;

	if ((chip->wired_online && get_client_vote(chip->wired_charging_disable_votable, CHG_FULL_VOTER) > 0) ||
		(chip->wls_online && get_client_vote(chip->wls_charging_disable_votable, CHG_FULL_VOTER) > 0))
		chip->chg_full = true;
	else
		chip->chg_full = false;

	if ((chip->chg_full != pre_full) && (chip->chg_full)) {
		pre_full = chip->chg_full;
		schedule_delayed_work(&chip->iterm_check_work, msecs_to_jiffies(10));
	}
	pre_full = chip->chg_full;
}

static void oplus_comm_gauge_check_work(struct work_struct *work)
{
	struct oplus_chg_comm *chip =
		container_of(work, struct oplus_chg_comm, gauge_check_work);
	union mms_msg_data data = { 0 };
	static unsigned long count_time;

	if (IS_ERR_OR_NULL(chip->gauge_topic)) {
		chg_err("gauge topic not ready\n");
		return;
	}

	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX, &data,
				false);
	chip->vbat_mv = data.intval;
	chip->vbat_mv_max = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MIN, &data,
				false);
	chip->vbat_min_mv = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CURR, &data,
				false);
	chip->ibat_ma = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_TEMP, &data,
				false);
	chip->batt_temp = data.intval;
	chip->main_batt_temp = chip->batt_temp;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOC, &data,
				false);
	chip->soc = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CC, &data,
				false);
	chip->batt_cc = data.intval;
	if (is_support_parallel_battery(chip->gauge_topic)) {
		if (is_main_gauge_topic_available(chip)) {
			oplus_mms_get_item_data(chip->main_gauge_topic, GAUGE_ITEM_VOL_MAX,
						&data, false);
			chip->main_vbat_mv = data.intval;
			chip->vbat_mv = chip->main_vbat_mv;
			oplus_mms_get_item_data(chip->main_gauge_topic, GAUGE_ITEM_CURR,
						&data, false);
			chip->main_ibat_ma = data.intval;
			oplus_mms_get_item_data(chip->main_gauge_topic, GAUGE_ITEM_TEMP,
						&data, false);
			chip->main_batt_temp = data.intval;
			oplus_mms_get_item_data(chip->main_gauge_topic, GAUGE_ITEM_SOC,
						&data, false);
			chip->main_soc = data.intval;
			chg_info("main_vbat_mv=%d, main_ibat_ma=%d, main_batt_temp=%d, main_soc=%d\n",
				 chip->main_vbat_mv, chip->main_ibat_ma,
				 chip->main_batt_temp, chip->main_soc);
		}
		if (is_sub_gauge_topic_available(chip)) {
			oplus_mms_get_item_data(chip->sub_gauge_topic, GAUGE_ITEM_VOL_MAX,
						&data, false);
			chip->sub_vbat_mv = data.intval;
			oplus_mms_get_item_data(chip->sub_gauge_topic, GAUGE_ITEM_CURR,
						&data, false);
			chip->sub_ibat_ma = data.intval;
			oplus_mms_get_item_data(chip->sub_gauge_topic, GAUGE_ITEM_TEMP,
						&data, false);
			chip->sub_batt_temp = data.intval;
			oplus_mms_get_item_data(chip->sub_gauge_topic, GAUGE_ITEM_SOC,
						&data, false);
			chip->sub_soc = data.intval;
			chg_info("sub_vbat_mv=%d, sub_ibat_ma=%d, sub_batt_temp=%d, sub_soc=%d\n",
				 chip->sub_vbat_mv, chip->sub_ibat_ma,
				 chip->sub_batt_temp, chip->sub_soc);
		}
		chg_info("vbat_mv=%d, ibat_ma=%d, batt_temp=%d, soc=%d\n",
			 chip->vbat_mv, chip->ibat_ma,
			 chip->batt_temp, chip->soc);
	}

	oplus_comm_smooth_soc_update(chip, false, true);

	oplus_comm_check_shell_temp(chip, true);
	oplus_comm_check_temp_region(chip);
	cancel_delayed_work(&chip->ui_soc_update_work);
	schedule_delayed_work(&chip->ui_soc_update_work, 0);
	if (time_after(jiffies, count_time)) {
		count_time = COUNT_TIMELIMIT * HZ;
		count_time += jiffies;
		oplus_comm_check_vbatt_is_good(chip);
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
		if (chip->config.gauge_stuck_jump_support &&
		    (get_eng_version() == AGING ||
		     get_eng_version() == HIGH_TEMP_AGING ||
		     get_eng_version() == FACTORY)) {
				 oplus_chg_gauge_stuck(chip);
				 oplus_comm_battery_notify_soc_check(chip);
		}
#endif
		if (chip->wired_online || chip->wls_online) {
			oplus_comm_check_fcc_gear(chip, false);
			oplus_comm_check_fv_over(chip);
			if (is_support_parallel_battery(chip->gauge_topic)) {
				oplus_comm_check_hw_sub_batt_full(chip);
				oplus_comm_check_sw_sub_batt_full(chip);
			}
			oplus_comm_check_hw_full(chip);
			oplus_comm_check_sw_full(chip);
			oplus_comm_check_rechg(chip);
			oplus_comm_check_ffc(chip);
			oplus_comm_check_fgreset(chip);
		}
	}
	oplus_comm_check_battery_status(chip);
	oplus_comm_check_battery_health(chip);
	oplus_comm_check_battery_charge_type(chip);
	oplus_comm_battery_notify_check(chip);
	oplus_comm_battery_notify_flag_check(chip);
	if (oplus_gauge_get_batt_num() == 2 &&
	    !is_support_parallel_battery(chip->gauge_topic) &&
	    chip->spec.bdd_vbatdiff_mv != 0)
		oplus_comm_dual_volt_diff_check(chip);
#ifdef CONFIG_OPLUS_CHARGER_MTK
	oplus_chg_kpoc_power_off_check(chip);
#endif
	oplus_comm_hw_iterm_check(chip);
}

static void oplus_comm_gauge_remuse_work(struct work_struct *work)
{
	struct oplus_chg_comm *chip =
		container_of(work, struct oplus_chg_comm, gauge_remuse_work);
	union mms_msg_data data = { 0 };

	spin_lock(&chip->remuse_lock);
	if (chip->comm_remuse && chip->gauge_remuse) {
		chg_info("remuse, update ui soc\n");
		chip->comm_remuse = false;
		chip->gauge_remuse = false;
		spin_unlock(&chip->remuse_lock);
		/* Trigger the update mechanism of gauge to ensure soc update */
		oplus_mms_topic_update(chip->gauge_topic, false);
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOC,
					&data, false);
		chip->soc = data.intval;

		oplus_comm_smooth_soc_update(chip, false, false);

		cancel_delayed_work_sync(&chip->ui_soc_update_work);
		schedule_delayed_work(&chip->ui_soc_update_work, 0);
	} else {
		spin_unlock(&chip->remuse_lock);
	}
}

static void oplus_comm_get_vol(struct oplus_chg_comm *chip)
{
	union mms_msg_data data = { 0 };

	if (is_support_parallel_battery(chip->gauge_topic)) {
		if (is_main_gauge_topic_available(chip)) {
			oplus_mms_get_item_data(chip->main_gauge_topic, GAUGE_ITEM_VOL_MAX,
						&data, true);
			chip->vbat_mv = data.intval;
		}
	} else {
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX,
				&data, true);
		chip->vbat_mv = data.intval;
	}

	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MIN,
				&data, true);
	chip->vbat_min_mv = data.intval;
	chg_info("vbat_mv=%d, vbat_min_mv=%d\n",
		chip->vbat_mv, chip->vbat_min_mv);
}

static void oplus_comm_noplug_batt_volt_work(struct work_struct *work)
{
	struct oplus_chg_comm *chip =
		container_of(work, struct oplus_chg_comm, noplug_batt_volt_work);
	int vbat_mv = 0;
	int vbat_min_mv =0;
	int vbat_grap, i;


	vbat_mv = chip->vbat_mv;
	vbat_min_mv = chip->vbat_min_mv;

	vbat_grap = vbat_mv - vbat_min_mv;
	if (vbat_grap < 0 || vbat_grap >= VBAT_MAX_GAP) {
		for (i = 0; i < VBAT_GAP_CHECK_CNT - 1; i++) {
			oplus_comm_get_vol(chip);
			vbat_mv += chip->vbat_mv;
			vbat_min_mv += chip->vbat_min_mv;
			if (i == 0)
				usleep_range(500000, 500000);
		}
		vbat_mv = vbat_mv / VBAT_GAP_CHECK_CNT;
		vbat_min_mv = vbat_min_mv / VBAT_GAP_CHECK_CNT;
	}
	noplug_batt_volt_max = vbat_mv;
	noplug_batt_volt_min = vbat_min_mv;
	chg_info("noplug_batt_volt_max=%d, noplug_batt_volt_min=%d\n",
		noplug_batt_volt_max, noplug_batt_volt_min);
}

static void oplus_comm_gauge_subs_callback(struct mms_subscribe *subs,
					   enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_chg_comm *chip = subs->priv_data;
	struct oplus_comm_spec_config *spec = &chip->spec;
	union mms_msg_data data = { 0 };
	int rc;

	switch (type) {
	case MSG_TYPE_TIMER:
		schedule_work(&chip->gauge_check_work);
		break;
	case MSG_TYPE_ITEM:
		switch (id) {
		case GAUGE_ITEM_BATT_EXIST:
			rc = oplus_mms_get_item_data(chip->gauge_topic, id,
						     &data, false);
			if (rc < 0)
				chip->batt_exist = false;
			else
				chip->batt_exist = data.intval;
			break;
		case GAUGE_ITEM_ERR_CODE:
			oplus_mms_get_item_data(chip->gauge_topic, id, &data,
						false);
			chip->gauge_err_code = data.intval;
			break;
		case GAUGE_ITEM_RESUME:
			spin_lock(&chip->remuse_lock);
			chip->gauge_remuse = true;
			if (chip->comm_remuse) {
				spin_unlock(&chip->remuse_lock);
				schedule_work(&chip->gauge_remuse_work);
			} else {
				spin_unlock(&chip->remuse_lock);
			}
			break;
		case GAUGE_ITEM_AUTH:
			rc = oplus_mms_get_item_data(chip->gauge_topic, id,
				&data, false);
			if (rc < 0) {
				chg_err("can't get GAUGE_ITEM_AUTH data, rc=%d\n", rc);
			} else {
				chip->authenticate = !!data.intval;
			}
			break;
		case GAUGE_ITEM_HMAC:
			rc = oplus_mms_get_item_data(chip->gauge_topic, id,
						     &data, false);
			if (rc < 0) {
				chg_err("can't get GAUGE_ITEM_HMAC data, rc=%d\n",
					rc);
			} else {
				chip->hmac = !!data.intval;
			}
			break;
		case GAUGE_ITEM_VBAT_UV:
			rc = oplus_mms_get_item_data(chip->gauge_topic, id,
							 &data, false);
			if (rc < 0) {
				chg_err("can't get GAUGE_ITEM_VBAT_UV data, rc=%d\n",
					rc);
			} else {
				if (data.intval > GAUGE_VBAT_UV_DELATA) {
					spec->vbat_uv_thr_mv = data.intval;
					spec->vbat_charging_uv_thr_mv = spec->vbat_uv_thr_mv - spec->vbat_charging_uv_delata_mv;
					spec->sub_vbat_uv_thr_mv = spec->vbat_uv_thr_mv;
					spec->sub_vbat_charging_uv_thr_mv = spec->vbat_charging_uv_thr_mv;
					if (!chip->config.support_boost_ic)
						oplus_comm_set_vbat_uv_thr(chip, spec->vbat_uv_thr_mv);
					 else
						oplus_comm_adjust_vbat_uv_by_current(chip, data.intval);
				}
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

#define MAH_TO_MAX_CHARGE_TIME(mah) ((mah) / 250 * 3600)
static void oplus_comm_subscribe_gauge_topic(struct oplus_mms *topic,
					     void *prv_data)
{
	struct oplus_chg_comm *chip = prv_data;
	struct oplus_comm_spec_config *spec = &chip->spec;
	union mms_msg_data data = { 0 };
	struct mms_msg *msg;
	int rc;
	int shutdown_soc;
	unsigned long now_tm_sec = 0;

	chip->gauge_subs =
		oplus_mms_subscribe(topic, chip,
				    oplus_comm_gauge_subs_callback, "chg_comm");
	if (IS_ERR_OR_NULL(chip->gauge_subs)) {
		chg_err("subscribe gauge topic error, rc=%ld\n",
			PTR_ERR(chip->gauge_subs));
		return;
	}

	oplus_mms_get_item_data(topic, GAUGE_ITEM_VOL_MAX, &data,
				true);
	chip->vbat_mv = data.intval;
	chip->vbat_mv_max = data.intval;
	oplus_comm_check_fcc_gear(chip, false);

	oplus_mms_get_item_data(topic, GAUGE_ITEM_DEEP_SUPPORT, &data, true);
	chip->deep_support = data.intval;
	oplus_mms_get_item_data(topic, GAUGE_ITEM_VOL_MIN, &data,
				true);
	chip->vbat_min_mv = data.intval;
	oplus_mms_get_item_data(topic, GAUGE_ITEM_CURR, &data,
				true);
	chip->ibat_ma = data.intval;
	oplus_mms_get_item_data(topic, GAUGE_ITEM_TEMP, &data,
				true);
	chip->batt_temp = data.intval;
	oplus_mms_get_item_data(topic, GAUGE_ITEM_SOC, &data,
				true);
	chip->soc = data.intval;
	oplus_mms_get_item_data(topic, GAUGE_ITEM_CC, &data,
				true);
	chip->batt_cc = data.intval;
	rc = oplus_mms_get_item_data(topic, GAUGE_ITEM_HMAC, &data,
				     true);
	if (rc < 0) {
		chg_err("can't get GAUGE_ITEM_HMAC data, rc=%d\n", rc);
		chip->hmac = false;
	} else {
		chip->hmac = !!data.intval;
	}
	rc = oplus_mms_get_item_data(topic, GAUGE_ITEM_AUTH, &data,
				true);
	if (rc < 0) {
		chg_err("can't get GAUGE_ITEM_AUTH data, rc=%d\n", rc);
		chip->authenticate = false;
	} else {
		chip->authenticate = !!data.intval;
	}
	chg_info("hmac=%d, authenticate=%d\n", chip->hmac, chip->authenticate);
	rc = oplus_mms_get_item_data(topic, GAUGE_ITEM_BATT_EXIST,
				     &data, true);
	if (rc < 0)
		chip->batt_exist = false;
	else
		chip->batt_exist = data.intval;
	rc = oplus_mms_get_item_data(topic, GAUGE_ITEM_ERR_CODE,
				     &data, true);
	if (rc < 0)
		chip->gauge_err_code = 0;
	else
		chip->gauge_err_code = data.intval;

	rc = oplus_mms_get_item_data(topic, GAUGE_ITEM_VBAT_UV, &data,
					 false);
	if (rc < 0) {
		chg_err("can't get GAUGE_ITEM_VBAT_UV data, rc=%d\n", rc);
	} else {
		if (data.intval > GAUGE_VBAT_UV_DELATA) {
			spec->vbat_uv_thr_mv = data.intval;
			spec->vbat_charging_uv_thr_mv = spec->vbat_uv_thr_mv - spec->vbat_charging_uv_delata_mv;
			spec->sub_vbat_uv_thr_mv = spec->vbat_uv_thr_mv;
			spec->sub_vbat_charging_uv_thr_mv = spec->vbat_charging_uv_thr_mv;
			oplus_comm_set_vbat_uv_thr(chip, spec->vbat_uv_thr_mv);
		}
	}

	/* Initialize passed_q save work if enabled */
	if (chip->config.support_boost_ic) {
		rc = oplus_gauge_get_car_c(chip->gauge_topic, 0, &chip->passed_q);
		if (rc < 0)
			rc = oplus_gauge_get_dod0_passed_q(chip->gauge_topic, 0, &chip->passed_q);
		chip->passed_q_last = 0;
		chip->passed_q_save_time = jiffies;
		rc = oplus_comm_get_current_time(&now_tm_sec);
		if (rc < 0) {
			chg_err("get current time failed for passed_q_save_time init, rc=%d\n", rc);
			chip->passed_q_save_time = 0;
		} else {
			chip->passed_q_save_time = now_tm_sec;
		}
		schedule_delayed_work(&chip->passed_q_save_work, msecs_to_jiffies(UV_ADJUSTED_CHECK_TIME * 1000));
		chg_info("passed_q tracking initialized for vbat_uv_curr_adjust\n");
	}

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if (get_eng_version() == HIGH_TEMP_AGING ||
	    get_eng_version() == AGING || get_eng_version() == FACTORY) {
		chg_info("HIGH_TEMP_AGING/AGING/FACTORY, disable chg timeout\n");
		chip->spec.max_chg_time_sec = -1;
	} else {
#endif
		chip->spec.max_chg_time_sec = MAH_TO_MAX_CHARGE_TIME(
			oplus_gauge_get_batt_capacity_mah(topic));
		chg_info("max_chg_time_sec=%d\n", chip->spec.max_chg_time_sec);
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	}
#endif

	chip->gauge_topic = topic;
	oplus_comm_check_shell_temp(chip, true);
	oplus_comm_check_temp_region(chip);

	chip->need_start_timeout_work = true;
	oplus_comm_start_timeout_work(chip);

	oplus_comm_smooth_soc_update(chip, true, false);

	oplus_comm_update_soc_jiffies(chip);
	chip->vbat_uv_jiffies = jiffies;
	chg_info("shutdown_soc=%d, soc=%d, partition_uisoc=%d, smooth_soc=%d\n",
	    chip->shutdown_soc, chip->soc, chip->partition_uisoc, chip->smooth_soc);

	/* to prevent smooth soc be changed to 0*/
	if (chip->smooth_soc == 0)
		chip->smooth_soc = chip->soc;

	shutdown_soc = chip->shutdown_soc;
	if (chip->shutdown_soc <= 0 && chip->partition_uisoc >= 0 && chip->partition_uisoc <= OPLUS_FULL_SOC) {
		chip->shutdown_soc = chip->partition_uisoc;
		if (abs(chip->partition_uisoc - chip->smooth_soc <= PARTITION_UISOC_GAP))
			shutdown_soc = chip->shutdown_soc;
		else if (chip->config.smooth_switch && chip->rsd.is_soc_jump_range)
			shutdown_soc = (chip->soc > (chip->smooth_soc - 1)) ? chip->soc : (chip->smooth_soc - 1);
		else
			shutdown_soc = chip->smooth_soc;
		chg_info("update shutdown_soc = %d\n", shutdown_soc);
	}

	/* soc is not allowed to be 0 when it is just turned on */
	if (shutdown_soc > 0 && abs(shutdown_soc - chip->soc) < 20)
		chip->ui_soc = shutdown_soc;
	else
		chip->ui_soc = chip->smooth_soc > 0 ? chip->smooth_soc : 1;
	oplus_comm_smooth_strategy_set_init_ui_soc(chip, chip->ui_soc);

	oplus_comm_update_soc_jiffies(chip);
	chip->batt_full_jiffies = jiffies;

	/* gauge is ready now!!! */
	g_ui_soc_ready = true;
	chg_info("set the init ui_soc = %d\n", chip->ui_soc);

	if (chip->ui_soc == 0)
                oplus_comm_push_ui_soc_shutdown_msg(chip);
	oplus_comm_publish_ui_soc(chip);

	oplus_comm_set_delta_soc(chip, MIN_DELTA_SOC);

	msg = oplus_mms_alloc_int_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, COMM_ITEM_SHUTDOWN_SOC,
		chip->shutdown_soc);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish shutdown soc msg error, rc=%d\n", rc);
		kfree(msg);
		return;
	}
}

static void oplus_comm_wired_subs_callback(struct mms_subscribe *subs,
					   enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_chg_comm *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WIRED_ITEM_ONLINE:
			schedule_work(&chip->plugin_work);
			break;
		case WIRED_ITEM_ERR_CODE:
			oplus_mms_get_item_data(chip->wired_topic,
						id, &data, false);
			chip->wired_err_code = data.intval;
			break;
		case WIRED_ITEM_CHG_TYPE:
			schedule_work(&chip->chg_type_change_work);
			break;
		case WIRED_ITEM_POWER_ROLE:
			schedule_work(&chip->chg_power_role_change_work);
			break;
		case WIRED_ITEM_CC_MODE:
			break;
		case WIRED_ITEM_CC_DETECT:
			oplus_mms_get_item_data(chip->wired_topic, id, &data, false);
			chip->cc_detect_status = data.intval;
			chg_info("cc_detect_status = %d\n", chip->cc_detect_status);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_comm_subscribe_wired_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_chg_comm *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->wired_topic = topic;
	chip->wired_subs =
		oplus_mms_subscribe(chip->wired_topic, chip,
				    oplus_comm_wired_subs_callback, "chg_comm");
	if (IS_ERR_OR_NULL(chip->wired_subs)) {
		chg_err("subscribe wired topic error, rc=%ld\n",
			PTR_ERR(chip->wired_subs));
		return;
	}

	/* Make sure to get the shutdown soc first */
	chip->shutdown_soc = oplus_wired_get_shutdown_soc(chip->wired_topic);
	oplus_mms_wait_topic("gauge", oplus_comm_subscribe_gauge_topic, chip);

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_ONLINE, &data,
				true);
	chip->wired_online = data.intval;
	/* WIRED_ITEM_ERR_CODE can't actively update */
	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_ERR_CODE, &data,
				false);
	chip->wired_err_code = data.intval;

	schedule_work(&chip->plugin_work);
}

static void oplus_comm_vooc_subs_callback(struct mms_subscribe *subs,
					  enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_chg_comm *chip = subs->priv_data;
	union mms_msg_data data = { 0 };
	struct ui_soc_decimal *soc_decimal = &chip->soc_decimal;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case VOOC_ITEM_VOOC_CHARGING:
			oplus_mms_get_item_data(chip->vooc_topic, id, &data,
						false);
			chip->vooc_charging = data.intval;
			break;
		case VOOC_ITEM_ONLINE:
			oplus_mms_get_item_data(chip->vooc_topic, id, &data,
						false);
			chip->vooc_online = data.intval;
			if (!chip->vooc_online &&
			    soc_decimal->calculate_decimal_time != 0) {
				if (soc_decimal->decimal_control) {
					cancel_delayed_work_sync(&chip->ui_soc_decimal_work);
					soc_decimal->last_decimal_ui_soc =
						(soc_decimal->ui_soc_integer + soc_decimal->ui_soc_decimal);
					oplus_comm_ui_soc_decimal_deinit(chip);
					chg_info("ui_soc_decimal: cancel last_decimal_ui_soc=%d",
						 soc_decimal->last_decimal_ui_soc);
				}
				soc_decimal->calculate_decimal_time = 0;
			}
			break;
		case VOOC_ITEM_SID:
			oplus_mms_get_item_data(chip->vooc_topic, id, &data,
						false);
			chip->vooc_sid = (unsigned int)data.intval;
			break;
		case VOOC_ITEM_VOOC_BY_NORMAL_PATH:
			oplus_mms_get_item_data(chip->vooc_topic, id, &data,
						false);
			chip->vooc_by_normal_path = (unsigned int)data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_comm_subscribe_vooc_topic(struct oplus_mms *topic,
						void *prv_data)
{
	struct oplus_chg_comm *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->vooc_topic = topic;
	chip->vooc_subs = oplus_mms_subscribe(chip->vooc_topic, chip,
					      oplus_comm_vooc_subs_callback,
					      "chg_comm");
	if (IS_ERR_OR_NULL(chip->vooc_subs)) {
		chg_err("subscribe vooc topic error, rc=%ld\n",
			PTR_ERR(chip->vooc_subs));
		return;
	}

	oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_VOOC_CHARGING,
				&data, true);
	chip->vooc_charging = data.intval;
	oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_ONLINE, &data,
				true);
	chip->vooc_online = data.intval;
	oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_ONLINE_KEEP, &data,
				true);
	chip->vooc_online_keep = data.intval;

	oplus_mms_get_item_data(chip->vooc_topic,
				VOOC_ITEM_VOOC_BY_NORMAL_PATH, &data,
				true);
	chip->vooc_by_normal_path = data.intval;
}

static void oplus_comm_reverse_subs_callback(struct mms_subscribe *subs,
					     enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_chg_comm *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case HIGH_REVERSE_ITEM_STATUS:
			oplus_mms_get_item_data(chip->reverse_topic, id, &data, false);
			chip->high_reverse_charging = !!data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_comm_subscribe_reverse_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_chg_comm *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->reverse_topic = topic;
	chip->reverse_subs = oplus_mms_subscribe(chip->reverse_topic, chip,
					      oplus_comm_reverse_subs_callback,
					      "chg_comm");
	if (IS_ERR_OR_NULL(chip->reverse_subs)) {
		chg_err("subscribe reverse topic error, rc=%ld\n", PTR_ERR(chip->reverse_subs));
		return;
	}

	oplus_mms_get_item_data(chip->reverse_topic, HIGH_REVERSE_ITEM_STATUS, &data, true);
	chip->high_reverse_charging = !!data.intval;
};

static void oplus_comm_wls_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_chg_comm *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WLS_ITEM_PRESENT:
			oplus_mms_get_item_data(chip->wls_topic, id, &data, false);
			oplus_comm_plugout_update_down_jiffies(chip, chip->wired_online, !!data.intval);
			chip->wls_online = !!data.intval;
			schedule_work(&chip->plugin_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_comm_subscribe_wls_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_chg_comm *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->wls_topic = topic;
	chip->wls_subs = oplus_mms_subscribe(chip->wls_topic, chip, oplus_comm_wls_subs_callback, "chg_comm");
	if (IS_ERR_OR_NULL(chip->wls_subs)) {
		chg_err("subscribe wls topic error, rc=%ld\n", PTR_ERR(chip->wls_subs));
		return;
	}
	oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_PRESENT, &data, true);
	chip->wls_online = !!data.intval;
	if (chip->wls_online)
		schedule_work(&chip->plugin_work);
}

static void oplus_comm_ufcs_subs_callback(struct mms_subscribe *subs,
					  enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_chg_comm *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case UFCS_ITEM_CHARGING:
			oplus_mms_get_item_data(chip->ufcs_topic, id, &data,
						false);
			chip->ufcs_charging = !!data.intval;
			break;
		case UFCS_ITEM_ONLINE:
			oplus_mms_get_item_data(chip->ufcs_topic, id, &data,
						false);
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

static void oplus_comm_subscribe_ufcs_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_chg_comm *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->ufcs_topic = topic;
	chip->ufcs_subs =
		oplus_mms_subscribe(topic, chip, oplus_comm_ufcs_subs_callback, "chg_comm");
	if (IS_ERR_OR_NULL(chip->ufcs_subs)) {
		chg_err("subscribe ufcs topic error, rc=%ld\n",
			PTR_ERR(chip->ufcs_subs));
		return;
	}

	oplus_mms_get_item_data(topic, UFCS_ITEM_CHARGING, &data, true);
	chip->ufcs_charging = !!data.intval;
	oplus_mms_get_item_data(topic, UFCS_ITEM_ONLINE, &data, true);
	chip->ufcs_online = !!data.intval;
}

static void oplus_comm_pps_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_chg_comm *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case PPS_ITEM_CHARGING:
			oplus_mms_get_item_data(chip->pps_topic, id, &data,
						false);
			chip->pps_charging = !!data.intval;
			break;
		case PPS_ITEM_ONLINE:
			oplus_mms_get_item_data(chip->pps_topic, id, &data,
						false);
			chip->pps_online = !!data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_comm_subscribe_pps_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_chg_comm *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->pps_topic = topic;
	chip->pps_subs =
		oplus_mms_subscribe(topic, chip, oplus_comm_pps_subs_callback, "chg_comm");
	if (IS_ERR_OR_NULL(chip->pps_subs)) {
		chg_err("subscribe pps topic error, rc=%ld\n",
			PTR_ERR(chip->pps_subs));
		return;
	}

	oplus_mms_get_item_data(topic, PPS_ITEM_CHARGING, &data, true);
	chip->pps_charging = !!data.intval;
	oplus_mms_get_item_data(topic, PPS_ITEM_ONLINE, &data, true);
	chip->pps_online = !!data.intval;
}

static void oplus_comm_retention_subs_callback(struct mms_subscribe *subs,
	enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_chg_comm *chip = subs->priv_data;
	union mms_msg_data data = { 0 };
	bool offline;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case RETENTION_ITEM_CONNECT_STATUS:
			oplus_mms_get_item_data(chip->retention_topic, id, &data,
						false);
			chip->retention_state = !!data.intval;
			offline = !chip->wired_online && !chip->wls_online;
			if (!chip->retention_state && offline)
				schedule_work(&chip->offline_delayed_process_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_comm_subscribe_retention_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_chg_comm *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->retention_topic = topic;
	chip->retention_subs =
		oplus_mms_subscribe(topic, chip, oplus_comm_retention_subs_callback, "chg_comm");
	if (IS_ERR_OR_NULL(chip->retention_subs)) {
		chg_err("subscribe retention topic error, rc=%ld\n",
			PTR_ERR(chip->retention_subs));
		return;
	}

	oplus_mms_get_item_data(topic, RETENTION_ITEM_CONNECT_STATUS, &data, true);
	chip->retention_state = !!data.intval;
}

static void oplus_comm_plc_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_chg_comm *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case PLC_ITEM_STATUS:
			oplus_mms_get_item_data(chip->plc_topic, id, &data,
						false);
			chip->plc_status = data.intval;
			oplus_comm_start_timeout_work(chip);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_comm_subscribe_plc_topic(struct oplus_mms *topic,
					   void *prv_data)
{
	struct oplus_chg_comm *chip = prv_data;
	union mms_msg_data data = { 0 };
	int rc;

	chip->plc_topic = topic;
	chip->plc_subs = oplus_mms_subscribe(chip->plc_topic, chip,
					     oplus_comm_plc_subs_callback,
					     "chg_comm");
	if (IS_ERR_OR_NULL(chip->plc_subs)) {
		chg_err("subscribe plc topic error, rc=%ld\n",
			PTR_ERR(chip->plc_subs));
		return;
	}

	rc = oplus_mms_get_item_data(chip->plc_topic, PLC_ITEM_STATUS, &data, true);
	if (rc >= 0)
		chip->plc_status = data.intval;
	oplus_comm_start_timeout_work(chip);
}

#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
static void oplus_comm_keep_subs_callback(struct mms_subscribe *subs,
					  enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_chg_comm *chip = subs->priv_data;
	union mms_msg_data data = { 0 };
	int rc;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case STATE_KEEP_ITEM_WIRED_ONLINE:
			rc = oplus_mms_get_item_data(chip->keep_topic, id, &data, false);
			if (rc < 0) {
				chg_err("get state_keep wired online failed, rc=%d\n", rc);
				break;
			}
			chip->keep_wired_online = !!data.intval;
			if (!chip->keep_wired_online)
				schedule_work(&chip->offline_delayed_process_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_comm_subscribe_keep_topic(struct oplus_mms *topic,
					    void *prv_data)
{
	struct oplus_chg_comm *chip = prv_data;
	union mms_msg_data data = { 0 };
	int rc;

	chip->keep_topic = topic;
	chip->keep_subs = oplus_mms_subscribe(chip->keep_topic, chip,
					     oplus_comm_keep_subs_callback,
					     "chg_comm");
	if (IS_ERR_OR_NULL(chip->keep_subs)) {
		chg_err("subscribe state_keep topic error, rc=%ld\n",
			PTR_ERR(chip->keep_subs));
		return;
	}

	rc = oplus_mms_get_item_data(chip->keep_topic, STATE_KEEP_ITEM_WIRED_ONLINE, &data, true);
	if (rc < 0)
		chg_err("get state_keep wired online failed, rc=%d\n", rc);
	else
		chip->keep_wired_online = !!data.intval;
}
#endif

static void oplus_comm_subs_comm_callback(struct mms_subscribe *subs,
						enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_chg_comm *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case COMM_ITEM_SUPER_ENDURANCE_STATUS:
			oplus_mms_get_item_data(chip->comm_topic, id, &data, false);
			chip->vbat_uv_jiffies = jiffies;
			break;
		case COMM_ITEM_BOOT_COMPLETED:
			oplus_comm_get_reserve_dec_cv_down_info(chip);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static int oplus_comm_subscribe_comm_topic(struct oplus_chg_comm *chip)
{
	union mms_msg_data data = { 0 };

	chip->comm_subs =
		oplus_mms_subscribe(chip->comm_topic, chip,
				    oplus_comm_subs_comm_callback,
				    "chg_comm");
	if (IS_ERR_OR_NULL(chip->comm_subs)) {
		chg_err("subscribe comm topic error, rc=%ld\n",
			PTR_ERR(chip->comm_subs));
		return PTR_ERR(chip->comm_subs);
	}

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_BOOT_COMPLETED, &data, true);
	if (data.intval)
		oplus_comm_get_reserve_dec_cv_down_info(chip);

	return 0;
}

static void oplus_comm_parse_dec_vol_dt(struct oplus_chg_comm *comm_dev)
{
	struct device_node *node = oplus_get_node_by_type(comm_dev->dev->of_node);
	struct oplus_comm_spec_config *spec = &comm_dev->spec;
	int rc = 0;

	rc = read_unsigned_data_from_node(node, "oplus_spec,dec-vol-cc-thr", (u32 *)spec->dec_cv.lite.dec_vol_cc_thr,
					  DEC_VOL_CC_THR_COUNT);
	if (rc < 0) {
		chg_err("get oplus_spec,dec-vol-cc-thr error, rc=%d\n", rc);
		goto full_support;
	}
	rc = read_unsigned_data_from_node(node, "oplus_spec,dec-vol-fv-mv", (u32 *)spec->dec_cv.lite.dec_vol_fv_mv,
					  DEC_VOL_CC_THR_COUNT * TEMP_REGION_MAX);
	if (rc < 0) {
		chg_err("get oplus_spec,dec-vol-fv-mv error, rc=%d\n", rc);
		goto full_support;
	}

	spec->dec_cv.dec_spec_support = DEC_CV_SUPPORT_LITE;
	return;

full_support:
	spec->dec_cv.full.pack_type = DEC_CV_PACK_UNKNOWN;
	rc = of_property_read_u32(node, "oplus_spec,dec_soh_fac", &spec->dec_cv.full.soh_fact);
	if (rc)
		spec->dec_cv.full.soh_fact = 100;
	if (spec->dec_cv.full.soh_fact < 0 || spec->dec_cv.full.soh_fact > 100)
		spec->dec_cv.full.soh_fact = 100;

	rc = of_property_read_u32(node, "oplus_spec,dec_step", &spec->dec_cv.full.spec_step);
	if (rc)
		spec->dec_cv.full.spec_step = 0;

	rc = of_property_count_elems_of_size(node, "oplus_spec,dec-vol-cc-full-thr", sizeof(u32));
	if (rc > 0 && rc <= DEC_VOL_CC_FULL_THR_COUNT) {
		rc = of_property_read_u32_array(node, "oplus_spec,dec-vol-cc-full-thr", (u32 *)spec->dec_cv.full.spec_cc_thr, rc);
	}
	if (rc < 0) {
		chg_err("get oplus_spec,dec-vol-cc-full-thr error, rc=%d\n", rc);
		goto not_support;
	}

	rc = of_property_count_elems_of_size(node, "oplus_spec,dec-vol-fv-full-mv", sizeof(u32));
	if (rc > 0 && rc <=  DEC_VOL_CC_FULL_THR_COUNT) {
		rc = of_property_read_u32_array(node, "oplus_spec,dec-vol-fv-full-mv", (u32 *)spec->dec_cv.full.spec_fv_mv, rc);
	}
	if (rc < 0) {
		chg_err("get oplus_spec,dec-vol-fv-full-mv error, rc=%d\n", rc);
		goto not_support;
	}

	rc = of_property_count_elems_of_size(node, "oplus_spec,dec-vol-vct-mv", sizeof(u32));
	if (rc > 0 && rc <=  DEC_VOL_CC_FULL_THR_COUNT) {
		rc = of_property_read_u32_array(node, "oplus_spec,dec-vol-vct-mv", (u32 *)spec->dec_cv.full.spec_vct_mv, rc);
	}
	if (rc < 0) {
		chg_err("dev vct not support\n");
		spec->dec_cv.full.vct_en = false;
	} else {
		spec->dec_cv.full.vct_en = true;
	}

	spec->dec_cv.dec_spec_support = DEC_CV_SUPPORT_FULL;
	return;

not_support:
	spec->dec_cv.dec_spec_support = DEC_CV_SUPPORT_NOT;
	memset(spec->dec_cv.lite.dec_vol_fv_mv, 0, sizeof(spec->dec_cv.lite.dec_vol_fv_mv));

	return;
}

int oplus_comm_get_dec_vol(struct oplus_mms *topic, int *fv_dec, int *wired_ffc_dec, int *wls_ffc_dec, int *vct)
{
	struct oplus_chg_comm *chip = NULL;
	enum oplus_ffc_temp_region ffc_temp_region = FFC_TEMP_REGION_MAX;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -ENODEV;
	}
	chip = oplus_mms_get_drvdata(topic);
	ffc_temp_region = chip->ffc_temp_region;
	*fv_dec = chip->spec.dec_cv.dec_vol;
	*vct = chip->spec.dec_cv.full.vct_up;

	if (ffc_temp_region < FFC_TEMP_REGION_PRE_NORMAL || ffc_temp_region > FFC_TEMP_REGION_NORMAL) {
		*wired_ffc_dec = 0;
		*wls_ffc_dec = 0;
	} else {
		*wired_ffc_dec = chip->spec.dec_cv.dec_vol;
		*wls_ffc_dec = chip->spec.dec_cv.dec_vol;
	}
	return 0;
}

static void oplus_comm_soc_decimal_offline(struct oplus_chg_comm *chip)
{
	struct ui_soc_decimal *soc_decimal = &chip->soc_decimal;

	if (soc_decimal->decimal_control) {
		cancel_delayed_work_sync(&chip->ui_soc_decimal_work);
		soc_decimal->last_decimal_ui_soc =
			(soc_decimal->ui_soc_integer + soc_decimal->ui_soc_decimal);
		oplus_comm_ui_soc_decimal_deinit(chip);
		chg_info("ui_soc_decimal: cancel last_decimal_ui_soc=%d, ui_soc_integer:%d,"
			"ui_soc_decimal:%d",
			soc_decimal->last_decimal_ui_soc, soc_decimal->ui_soc_integer,
			soc_decimal->ui_soc_decimal);
	}
	soc_decimal->calculate_decimal_time = 0;
}

/* For the status not controlled by the driver, clear it here */
static void oplus_comm_offline_clean_process(struct oplus_chg_comm *chip)
{
	if (chip->chg_cycle_status & CHG_CYCLE_VOTER__USER) {
		oplus_comm_set_chg_cycle_status(chip,
			chip->chg_cycle_status & (~(int)CHG_CYCLE_VOTER__USER));
		if (!chip->chg_cycle_status) {
			vote(chip->chg_suspend_votable, DEBUG_VOTER, false, 0, false);
			vote(chip->chg_disable_votable, MMI_CHG_VOTER, false, 0, false);
		}
	}
	oplus_comm_soc_decimal_offline(chip);
}

#define OFFLINE_CLEAN_DELAY 500
static void oplus_comm_plugin_work(struct work_struct *work)
{
	struct oplus_chg_comm *chip =
		container_of(work, struct oplus_chg_comm, plugin_work);
	union mms_msg_data data = { 0 };
	int fv_mv = 0;

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_ONLINE, &data,
				false);
	oplus_comm_plugout_update_down_jiffies(chip, !!data.intval, chip->wls_online);
	chip->wired_online = data.intval;

	chg_info("wired_online = %d, wls_online = %d\n", chip->wired_online, chip->wls_online);
	if (chip->wired_online || chip->wls_online) {
		flush_work(&chip->offline_delayed_process_work);
		chip->low_temp_check_jiffies = jiffies;
		oplus_comm_dec_cv_check(chip);
		oplus_comm_check_shell_temp(chip, true);
		oplus_comm_temp_thr_init(chip);
		oplus_comm_check_temp_region(chip);
		oplus_comm_check_fcc_gear(chip, false);
		oplus_comm_check_battery_status(chip);

		oplus_comm_fginfo_reset(chip);
		noplug_temperature = chip->main_batt_temp;
		schedule_work(&chip->noplug_batt_volt_work);
		oplus_comm_battery_notify_check(chip);
		oplus_comm_battery_notify_flag_check(chip);
		chip->fv_over = false;
		vote(chip->fv_min_votable, OVER_FV_VOTER, false, 0, false);
		fv_mv = chip->spec.fv_mv[chip->temp_region] - chip->spec.dec_cv.dec_vol;

		if (oplus_comm_check_allow_set_fv(chip, fv_mv))
			vote(chip->fv_max_votable, SPEC_VOTER, true, fv_mv, false);

		rerun_election(chip->fv_max_votable, false);
		rerun_election(chip->fv_min_votable, false);
		cancel_delayed_work_sync(&chip->charge_timeout_work);
		/* ensure that max_chg_time_sec has been obtained */
		chip->need_start_timeout_work = true;
		oplus_comm_start_timeout_work(chip);
		/*
		 * Make sure that it will not report full charge immediately
		 * after plugging in the charger.
		 */
		chip->batt_full_jiffies = jiffies;
	} else {
		chip->low_temp_check_jiffies = jiffies;
		if (is_wired_charging_disable_votable_available(chip)) {
			vote(chip->wired_charging_disable_votable,
			     CHG_FULL_VOTER, false, 0, false);
			vote(chip->wired_charging_disable_votable, FFC_VOTER,
			     false, 0, false);
		}
		if (is_wired_fcc_votable_available(chip)) {
			vote(chip->wired_fcc_votable, FFC_VOTER, false, 0,
			     false);
		}
		if (is_wls_charging_disable_votable_available(chip))
			vote(chip->wls_charging_disable_votable, CHG_FULL_VOTER, false, 0, false);
		cancel_delayed_work_sync(&chip->ffc_start_work);
		cancel_work_sync(&chip->noplug_batt_volt_work);
		chip->fg_soft_reset_done = true;
		chip->ffc_charging = false;
		chip->sw_full = false;
		chip->hw_full_by_sw = false;
		chip->cv_cutoff_volt_curr = 0;
		chip->ffc_cutoff_curr = 0;
		oplus_comm_set_batt_full(chip, false);
		chip->uisoc_down_in_full = false;
		chip->rechg_now = false;
		if (is_support_parallel_battery(chip->gauge_topic)) {
			chip->sw_sub_batt_full = false;
			chip->hw_sub_batt_full_by_sw = false;
			oplus_comm_set_sub_batt_full(chip, false);
		}
		oplus_comm_set_rechging(chip, false);
		oplus_comm_set_ffc_status(chip, FFC_DEFAULT);
		vote(chip->chg_disable_votable, TIMEOUT_VOTER, false, 0, false);
		chip->chging_over_time = false;
		vote(chip->chg_disable_votable, EIS_VOTER, false, 0, false);
		/* When wireless charging is activated, some shared resources cannot be cleared. */
		if (!chip->wls_online) {
			vote(chip->fv_max_votable, FFC_VOTER, false, 0, false);
			vote(chip->fv_min_votable, OVER_FV_VOTER, false, 0, false);
			chip->need_start_timeout_work = false;
			cancel_delayed_work_sync(&chip->charge_timeout_work);
			oplus_comm_battery_notify_check(chip);
			oplus_comm_battery_notify_flag_check(chip);
		}
		if (!chip->vooc_online) {
			chip->bms_heat_temp_compensation = 0;
			oplus_comm_set_slow_chg(chip->comm_topic, 0, 0, false);
		}
		if (chip->retention_topic) {
			schedule_delayed_work(&chip->offline_clean_work, msecs_to_jiffies(OFFLINE_CLEAN_DELAY));
		} else {
			if (!chip->keep_wired_online)
				oplus_comm_offline_clean_process(chip);
		}
		vote(chip->chg_suspend_votable, CHG_LIMIT_CHG_VOTER, false, 0, false);
		vote(chip->chg_disable_votable, CHG_LIMIT_CHG_VOTER, false, 0, false);
		vote(chip->chg_disable_votable, FLASH_MODE_VOTER, false, 0, false);
		oplus_comm_check_fcc_gear(chip, true);
	}

	oplus_comm_smooth_strategy_set_online(chip);
	/* Ensure that the charging status is updated in a timely manner */
	schedule_work(&chip->gauge_check_work);
}

static void oplus_comm_offline_delayed_process_work(struct work_struct *work)
{
	struct oplus_chg_comm *chip =
		container_of(work, struct oplus_chg_comm, offline_delayed_process_work);

	if (chip->wired_online || chip->wls_online)
		return;

	/* Handling actions intercepted by retention_state */
	oplus_comm_offline_clean_process(chip);
}

static void oplus_comm_chg_type_change_work(struct work_struct *work)
{
	struct oplus_chg_comm *chip =
		container_of(work, struct oplus_chg_comm, chg_type_change_work);

	/* Ensure that the charging status is updated in a timely manner */
	schedule_work(&chip->gauge_check_work);
}

static void oplus_comm_chg_power_role_change_work(struct work_struct *work)
{
	struct oplus_chg_comm *chip =
		container_of(work, struct oplus_chg_comm, chg_power_role_change_work);
	union mms_msg_data power_role = { -1 };
	oplus_mms_get_item_data(chip->wired_topic,
		WIRED_ITEM_POWER_ROLE, &power_role, false);
	chip->power_role = power_role.intval;
	chg_info("actual power role = %d", chip->power_role);
}

int oplus_comm_get_vbatt_over_threshold(struct oplus_mms *topic)
{
	struct oplus_chg_comm *chip;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return 0;
	}
	chip = oplus_mms_get_drvdata(topic);

	return chip->spec.vbatt_ov_thr_mv;
}

static void oplus_comm_set_unwakelock(struct oplus_chg_comm *chip, bool en)
{
	struct mms_msg *msg;
	int rc;

	if (chip->unwakelock_chg == en)
		return;
	chip->unwakelock_chg = en;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  COMM_ITEM_UNWAKELOCK);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish unwakelock msg error, rc=%d\n", rc);
		kfree(msg);
	}

	chg_info("unwakelock_chg=%s\n", en ? "true" : "false");
}

static void oplus_comm_set_power_save(struct oplus_chg_comm *chip, bool en)
{
	struct mms_msg *msg;
	int rc;

	if (chip->chg_powersave == en)
		return;
	chip->chg_powersave = en;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  COMM_ITEM_POWER_SAVE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish power save msg error, rc=%d\n", rc);
		kfree(msg);
	}

	chg_info("chg_powersave=%s\n", en ? "true" : "false");
}

static void oplus_comm_set_chg_cycle_status(struct oplus_chg_comm *chip, int status)
{
	struct mms_msg *msg;
	int rc;

	if (chip->chg_cycle_status == status)
		return;
	chip->chg_cycle_status = status;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  COMM_ITEM_CHG_CYCLE_STATUS);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish chg cycle status msg error, rc=%d\n", rc);
		kfree(msg);
	}

	chg_info("chg_cycle_status=%d\n", status);
}

static int oplus_comm_update_temp_region(struct oplus_mms *mms,
					 union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->temp_region;

	return 0;
}

static int oplus_comm_update_fcc_gear(struct oplus_mms *mms,
				       union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->fcc_gear;

	return 0;
}

static int oplus_comm_update_chg_full(struct oplus_mms *mms,
				       union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->batt_full;

	return 0;
}

static int oplus_comm_update_chg_sub_batt_full(struct oplus_mms *mms,
				      union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->sub_batt_full;

	return 0;
}

static int oplus_comm_update_batt_cv_full(struct oplus_mms *mms,
				       union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->batt_cv_full;

	return 0;
}

static int oplus_comm_update_ffc_status(struct oplus_mms *mms,
				       union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->ffc_status;

	return 0;
}

static int oplus_comm_update_cv_cutoff_volt_curr(struct oplus_mms *mms,
				       union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->cv_cutoff_volt_curr;

	return 0;
}

static int oplus_comm_update_ffc_cutoff_curr(struct oplus_mms *mms,
				       union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->ffc_cutoff_curr;

	return 0;
}

static int oplus_comm_update_chg_disable(struct oplus_mms *mms,
					 union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;

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

static int oplus_comm_update_chg_suspend(struct oplus_mms *mms,
					 union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->charge_suspend;

	return 0;
}

static int oplus_comm_update_cool_down(struct oplus_mms *mms,
				       union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->cool_down;

	return 0;
}

static int oplus_comm_update_sale_mode(struct oplus_mms *mms,
                                       union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	if (chip->cool_down_sale_mode == SALE_MODE_COOL_DOWN ||
	    chip->cool_down_sale_mode == SALE_MODE_COOL_DOWN_TWO)
		data->intval = chip->cool_down_sale_mode;
	else
		data->intval = 0;

	return 0;
}

static int oplus_comm_update_batt_status(struct oplus_mms *mms,
					 union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->batt_status;

	return 0;
}

static int oplus_comm_update_batt_health(struct oplus_mms *mms,
					 union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->batt_health;

	return 0;
}

static int oplus_comm_update_batt_chg_type(struct oplus_mms *mms,
					   union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->batt_chg_type;

	return 0;
}

static int oplus_comm_update_ui_soc(struct oplus_mms *mms,
				    union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	if (g_ui_soc_ready)
		data->intval = chip->ui_soc;
	else
		data->intval = -EINVAL;

	return 0;
}

static int oplus_comm_update_vbat_uv_thr(struct oplus_mms *mms,
				    union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;
	struct oplus_comm_spec_config *spec;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);
	spec = &chip->spec;

	data->intval = spec->vbat_uv_thr_mv;

	return 0;
}

static int oplus_comm_update_notify_code(struct oplus_mms *mms,
					 union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->notify_code;

	return 0;
}

static int oplus_comm_update_notify_flag(struct oplus_mms *mms,
					 union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->notify_flag;

	return 0;
}

static int oplus_comm_update_shell_temp(struct oplus_mms *mms,
					union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;
	union mms_msg_data tmp = { 0 };

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -40;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -40;
	}
	chip = oplus_mms_get_drvdata(mms);

	if (chip->gauge_topic != NULL) {
		/* Make sure to get the latest battery temperature */
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_TEMP,
					&tmp, false);
		chip->batt_temp = tmp.intval;
	} else {
		chip->batt_temp = GAUGE_INVALID_TEMP;
	}
	oplus_comm_check_shell_temp(chip, false);
	data->intval = chip->shell_temp;

	return 0;
}

static int oplus_comm_update_factory_test(struct oplus_mms *mms,
					  union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;
	int factory_test = 0;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	factory_test = chip->factory_test_mode;

	data->intval = factory_test;
	return 0;
}

static int oplus_comm_update_led_on(struct oplus_mms *mms,
				    union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;
	bool led_on = false;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	led_on = chip->led_on;
	data->intval = led_on;
	return 0;
}

static int oplus_comm_update_unwakelock(struct oplus_mms *mms,
					union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;
	bool unwakelock = false;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	unwakelock = chip->unwakelock_chg;
	data->intval = unwakelock;
	return 0;
}

static int oplus_comm_update_power_save(struct oplus_mms *mms,
					union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;
	bool power_save = false;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	power_save = chip->chg_powersave;
	data->intval = power_save;
	return 0;
}

static int oplus_comm_update_rechging(struct oplus_mms *mms,
				      union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->rechging;
	return 0;
}

static int oplus_comm_update_ffc_step(struct oplus_mms *mms,
				      union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	if (chip->ffc_status == FFC_FAST)
		data->intval = chip->ffc_step;
	else
		data->intval = 0;
	return 0;
}

static int oplus_comm_update_smooth_soc(struct oplus_mms *mms,
					union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->smooth_soc;
	return 0;
}

static int oplus_comm_update_boot_completed(struct oplus_mms *mms,
					union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}

	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	data->intval = chip->soc_decimal.boot_completed;

	return 0;
}

static int oplus_comm_update_chg_cycle_status(struct oplus_mms *mms,
				      union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->chg_cycle_status;
	return 0;
}

void oplus_comm_set_slow_chg(struct oplus_mms *topic, int pct, int watt, bool en)
{
	struct oplus_chg_comm *chip;
	struct mms_msg *msg;
	int rc, slow_chg_param = 0;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return;
	}
	chip = oplus_mms_get_drvdata(topic);
	mutex_lock(&chip->slow_chg_mutex);
	slow_chg_param = SLOW_CHG_TO_PARAM(pct, watt, en);
	if (slow_chg_param != chip->slow_chg_param) {
		chip->slow_chg_param = slow_chg_param;
		msg = oplus_mms_alloc_int_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, COMM_ITEM_SLOW_CHG, chip->slow_chg_param);
		if (msg == NULL) {
			chg_err("alloc msg error\n");
			goto done;
		}

		rc = oplus_mms_publish_msg(topic, msg);
		if (rc < 0) {
			chg_err("publish slow chg msg error, rc=%d\n", rc);
			kfree(msg);
			goto done;
		}
	}
done:
	mutex_unlock(&chip->slow_chg_mutex);
}

static int oplus_comm_update_chg_rechg_soc_en_status(struct oplus_mms *mms,
				      union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = RECHG_SOC_TO_PARAM(chip->rechg_soc, chip->rechg_soc_en);
	return 0;
}

static int oplus_comm_update_nvid_support_flags(struct oplus_mms *mms,
				      union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;

	if (mms == NULL) {
		chg_err("mms is NULL\n");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL\n");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->nvid_support_flags;
	return 0;
}

static void oplus_comm_update(struct oplus_mms *mms, bool publish)
{
}

static int oplus_comm_wired_update_flash_mode(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_comm *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->flash_mode;

	return 0;
}
static struct mms_item oplus_comm_item[] = {
	{
		.desc = {
			.item_id = COMM_ITEM_TEMP_REGION,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_update_temp_region,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_FCC_GEAR,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_update_fcc_gear,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_CHG_FULL,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_update_chg_full,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_CHG_SUB_BATT_FULL,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_update_chg_sub_batt_full,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_FFC_STATUS,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_update_ffc_status,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_CV_CUTOFF_VOLT_CURR,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_update_cv_cutoff_volt_curr,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_FFC_CUTOFF_CURR,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_update_ffc_cutoff_curr,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_CHARGING_DISABLE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_update_chg_disable,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_CHARGE_SUSPEND,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_update_chg_suspend,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_COOL_DOWN,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_update_cool_down,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_BATT_STATUS,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_update_batt_status,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_BATT_HEALTH,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_update_batt_health,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_BATT_CHG_TYPE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_update_batt_chg_type,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_UI_SOC,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_update_ui_soc,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_SHUTDOWN_SOC,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_VBAT_UV_THR,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_update_vbat_uv_thr,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_NOTIFY_CODE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_update_notify_code,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_NOTIFY_FLAG,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_update_notify_flag,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_SHELL_TEMP,
			.dead_thr_enable = true,
			/* Actively report when the temperature changes more than 0.5 degrees */
			.dead_zone_thr = 5,
			.update = oplus_comm_update_shell_temp,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_FACTORY_TEST,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_update_factory_test,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_LED_ON,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_update_led_on,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_UNWAKELOCK,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_update_unwakelock,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_POWER_SAVE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_update_power_save,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_RECHGING,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_update_rechging,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_FFC_STEP,
			.update = oplus_comm_update_ffc_step,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_SMOOTH_SOC,
			.update = oplus_comm_update_smooth_soc,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_CHG_CYCLE_STATUS,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_update_chg_cycle_status,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_SLOW_CHG,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_DELTA_SOC,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	}, {
		.desc = {
			.item_id = COMM_ITEM_SUPER_ENDURANCE_STATUS,
		}
	}, {
		.desc = {
			.item_id = COMM_ITEM_SUPER_ENDURANCE_COUNT,
		}
	}, {
		.desc = {
			.item_id = COMM_ITEM_UISOC_KEEP_2_ERROR,
		}
	}, {
		.desc = {
			.item_id = COMM_ITEM_UISOC_KEEP_3_ERROR,
		}
	}, {
		.desc = {
			.item_id = COMM_ITEM_RECHG_SOC_EN_STATUS,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_update_chg_rechg_soc_en_status,
		}
	}, {
		.desc = {
			.item_id = COMM_ITEM_SALE_MODE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_update_sale_mode,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_NVID_SUPPORT_FLAGS,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_update_nvid_support_flags,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_BOOT_COMPLETED,
			.update = oplus_comm_update_boot_completed,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_EIS_STATUS,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_FLASH_MODE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_wired_update_flash_mode,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_BATT_CV_FULL,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_comm_update_batt_cv_full,
		}
	}
};

static const struct oplus_mms_desc oplus_comm_desc = {
	.name = "common",
	.type = OPLUS_MMS_TYPE_COMM,
	.item_table = oplus_comm_item,
	.item_num = ARRAY_SIZE(oplus_comm_item),
	.update_items = NULL,
	.update_items_num = 0,
	.update_interval = 0, /* ms */
	.update = oplus_comm_update,
};

static int oplus_comm_topic_init(struct oplus_chg_comm *chip)
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

	chip->comm_topic =
		devm_oplus_mms_register(chip->dev, &oplus_comm_desc, &mms_cfg);
	if (IS_ERR(chip->comm_topic)) {
		chg_err("Couldn't register comm topic\n");
		rc = PTR_ERR(chip->comm_topic);
		return rc;
	}

	return 0;
}

int read_signed_data_from_node(struct device_node *node,
			       const char *prop_str,
			       s32 *addr, int len_max)
{
	int rc = 0, length;

	if (!node || !prop_str || !addr) {
		chg_err("Invalid parameters passed\n");
		return -EINVAL;
	}

	rc = of_property_count_elems_of_size(node, prop_str, sizeof(s32));
	if (rc < 0) {
		chg_err("Count %s failed, rc=%d\n", prop_str, rc);
		return rc;
	}

	length = rc;

	if (length > len_max) {
		chg_err("too many entries(%d), only %d allowed\n", length, len_max);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(node, prop_str, (u32 *)addr, length);
	if (rc) {
		chg_err("Read %s failed, rc=%d\n", prop_str, rc);
		return rc;
	}

	return rc;
}

int read_unsigned_data_from_node(struct device_node *node,
				 const char *prop_str, u32 *addr,
				 int len_max)
{
	int rc = 0, length;

	if (!node || !prop_str || !addr) {
		chg_err("Invalid parameters passed\n");
		return -EINVAL;
	}

	rc = of_property_count_elems_of_size(node, prop_str, sizeof(u32));
	if (rc < 0) {
		chg_err("Count %s failed, rc=%d\n", prop_str, rc);
		return rc;
	}

	length = rc;

	if (length > len_max) {
		chg_err("too many entries(%d), only %d allowed\n", length, len_max);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(node, prop_str, (u32 *)addr, length);
	if (rc < 0) {
		chg_err("Read %s failed, rc=%d\n", prop_str, rc);
		return rc;
	}

	return length;
}

int oplus_comm_temp_region_map(int index)
{
	if (oplus_get_chg_spec_version() < OPLUS_CHG_SPEC_VER_V3P7) {
		if (index >= TEMP_REGION_NORMAL_HIGH)
			index -= 1;
	}

	if (oplus_get_chg_spec_version() < OPLUS_CHG_SPEC_VER_V3P7_1) {
		if (index >= TEMP_REGION_LITTLE_COLD_HIGH)
			index -= 1;
	}
	if (index < 0) {
		chg_err("index=%d, temp region releated config error", index);
		index = 0;
	}

	return index;
}

int oplus_comm_get_temp_region_max(void)
{
	if (oplus_get_chg_spec_version() < OPLUS_CHG_SPEC_VER_V3P7)
		return V3P6_TEMP_REGION_MAX;

	if (oplus_get_chg_spec_version() < OPLUS_CHG_SPEC_VER_V3P7_1) {
		return V3P7_TEMP_REGION_MAX;
	}

	return TEMP_REGION_MAX;
}

static int oplus_comm_ffc_temp_region_map(int index)
{
	if (oplus_get_chg_spec_version() < OPLUS_CHG_SPEC_VER_V3P7) {
		if (index >= FFC_TEMP_REGION_PRE_NORMAL)
			return index - 1;
	}

	return index;
}

int oplus_comm_get_ffc_temp_region_max(void)
{
	if (oplus_get_chg_spec_version() < OPLUS_CHG_SPEC_VER_V3P7)
		return V3P6_FFC_TEMP_REGION_MAX;

	return FFC_TEMP_REGION_MAX;
}

int read_unsigned_temp_region_data(struct device_node *node, const char *prop_str, u32 *addr,
					    int col, int col_max, int row, int (*col_map)(int))
{
	int rc = 0, length, len_max = col_max * row, i, j, index;

	if (col == col_max)
		return read_unsigned_data_from_node(node, prop_str, addr, col * row);

	if (!node || !prop_str || !addr || !col_map) {
		chg_err("Invalid parameters passed\n");
		return -EINVAL;
	}

	rc = of_property_count_elems_of_size(node, prop_str, sizeof(u32));
	if (rc < 0) {
		chg_err("Count %s failed, rc=%d\n", prop_str, rc);
		return rc;
	}

	length = rc;

	if (length > len_max) {
		chg_err("too many entries(%d), only %d allowed\n", length, len_max);
		return -EINVAL;
	}

	for (i = 0; i < row; i++) {
		for (j = 0; j < col_max; j++) {
			index = col_map(j) + i * col;
			rc = of_property_read_u32_index(node, prop_str, index, (u32 *)(addr + j + i * col_max));
			if (rc < 0) {
				chg_err("Count %s failed, rc=%d\n", prop_str, rc);
				continue;
			}
		}
	}

	return 0;
}

int read_signed_temp_region_data(struct device_node *node, const char *prop_str, s32 *addr,
					    int col, int col_max, int row, int (*col_map)(int))
{
	int rc = 0, length, len_max = col_max * row, i, j, index;

	if (col == col_max)
		return read_signed_data_from_node(node, prop_str, addr, col * row);

	if (!node || !prop_str || !addr) {
		chg_err("Invalid parameters passed\n");
		return -EINVAL;
	}

	rc = of_property_count_elems_of_size(node, prop_str, sizeof(u32));
	if (rc < 0) {
		chg_err("Count %s failed, rc=%d\n", prop_str, rc);
		return rc;
	}

	length = rc;

	if (length > len_max) {
		chg_err("too many entries(%d), only %d allowed\n", length, len_max);
		return -EINVAL;
	}

	for (i = 0; i < row; i++) {
		for (j = 0; j < col_max; j++) {
			index = col_map(j) + i * col;
			rc = of_property_read_u32_index(node, prop_str, index, (u32 *)(addr + j + i * col_max));
			if (rc < 0)
				chg_err("Count %s failed, rc=%d\n", prop_str, rc);
		}
	}

	return rc;
}

static void oplus_comm_parse_aging_ffc_dt(struct oplus_chg_comm *comm_dev)
{
	comm_dev->spec.wired_aging_ffc_version = AGING_FFC_NOT_SUPPORT;
}

static bool oplus_comm_reserve_soc_by_rus(struct oplus_chg_comm *chip)
{
	struct device_node *np;
	const char *bootparams = NULL;
	int rus_soc = 0, last_uisoc = 0;
	char *str;

	chip->partition_uisoc = -EINVAL;
	np = of_find_node_by_path("/chosen");
	if (np) {
		of_property_read_string(np, "bootargs", &bootparams);
		if (!bootparams) {
			chg_err("failed to get bootargs property\n");
			return false;
		}

		str = strstr(bootparams, "oplus_uisoc=");
		if (str) {
			str += strlen("oplus_uisoc=");
			get_option(&str, &last_uisoc);
			chg_info("oplus_uisoc=%d\n", last_uisoc);
			if (((last_uisoc >> 8) & 0xFF) + (last_uisoc & 0xFF) == 0x7F) {
				chip->partition_uisoc = (last_uisoc >> 8) & 0xFF;
				chg_info("update partition_uisoc = %d\n", chip->partition_uisoc);
			}
		}

		str = strstr(bootparams, "reserve_soc=");
		if (str) {
			str += strlen("reserve_soc=");
			get_option(&str, &rus_soc);
			chg_err("reserve_soc=%d\n", rus_soc);
			chip->rsd.rus_reserve_soc = (rus_soc >> 8) & 0xFF;
			return true;
		}
	}

	return false;
}

static bool oplus_comm_parse_from_cmdline(struct oplus_chg_comm *chip)
{
	struct device_node *np;
	const char *bootparams = NULL;
	char *str;
	int ret = 0;
	struct oplus_comm_spec_config *spec = &chip->spec;

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

static void oplus_comm_parse_ui_soc_smooth_table_dt(struct oplus_chg_comm *chip, struct device_node *node)
{
	int rc = 0;
	int i = 0, j = 0, len = 0;
	int (*dynamic_table)[RESERVE_SOC_MAX];
	int expected_len = (RESERVE_SOC_MAX + 1) * RESERVE_SOC_MAX;
	u32 tmp_table[(RESERVE_SOC_MAX + 1) * RESERVE_SOC_MAX];

	if (!node) {
		chg_err("Invalid parameters passed\n");
		return;
	}

	len = of_property_count_u32_elems(node, "oplus,ui_soc_smooth_table");
	if (len > 0) {
		if (len != expected_len) {
			chg_err("ui_soc_smooth_table length invalid: len=%d (expected %d), use default\n", len, expected_len);
			return;
		}

		rc = of_property_read_u32_array(node, "oplus,ui_soc_smooth_table", tmp_table,
				(RESERVE_SOC_MAX + 1) * RESERVE_SOC_MAX);
		if (rc < 0) {
			chg_err("get oplus,ui_soc_smooth_table, use default\n");
			return;
		}

		dynamic_table = devm_kzalloc(chip->dev,
					sizeof(int) * (RESERVE_SOC_MAX + 1) * RESERVE_SOC_MAX, GFP_KERNEL);
		if (dynamic_table) {
			for (i = 0; i < RESERVE_SOC_MAX + 1; i++) {
				for (j = 0; j < RESERVE_SOC_MAX; j++) {
					dynamic_table[i][j] = (int)tmp_table[i * RESERVE_SOC_MAX + j];
				}
			}
			chip->ui_soc_smooth_table = dynamic_table;
			chg_info("use dts ui_soc_smooth_table, elements=%d\n", len);
		} else {
			chg_err("alloc ui_soc_smooth_table failed, use default\n");
		}
	} else {
		chg_info("no dts ui_soc_smooth_table, use default\n");
		return;
	}
}

static void oplus_comm_parse_smooth_soc_dt(struct oplus_chg_comm *chip)
{
	struct device_node *node = oplus_get_node_by_type(chip->dev->of_node);
	struct oplus_comm_config *config = &chip->config;
	int rc;

	chip->ui_soc_smooth_table = ui_soc_smooth_table;

	if (oplus_comm_reserve_soc_by_rus(chip)) {
		config->smooth_switch = true;
		config->reserve_soc = chip->rsd.rus_reserve_soc;
		oplus_comm_parse_ui_soc_smooth_table_dt(chip, node);
	} else {
		config->smooth_switch = of_property_read_bool(node, "oplus,smooth_switch");
		if (config->smooth_switch) {
			rc = of_property_read_u32(node, "oplus,reserve_chg_soc", &config->reserve_soc);
			if (rc)
				config->reserve_soc = RESERVE_SOC_DEFAULT;

			oplus_comm_parse_ui_soc_smooth_table_dt(chip, node);
		}
		chg_info("read from dts %d %d\n", config->smooth_switch, config->reserve_soc);
	}

	if (config->smooth_switch) {
		if (config->reserve_soc < RESERVE_SOC_MIN || config->reserve_soc > RESERVE_SOC_MAX) {
			config->reserve_soc = RESERVE_SOC_OFF;
			config->smooth_switch = false;
		}
	}

	chg_info("smooth_switch %d reserve_soc %d\n", config->smooth_switch, config->reserve_soc);
}

static void oplus_comm_parse_hidden_soc_dt(struct oplus_chg_comm *chip)
{
	struct device_node *node = chip->dev->of_node;
	struct oplus_comm_config *config = &chip->config;
	int rc;

	config->hidden_soc_switch = of_property_read_bool(node, "oplus,hidden_soc_switch");
	if (config->hidden_soc_switch) {
		rc = of_property_read_u32(node, "oplus,reserve_chg_soc", &config->reserve_soc);
		if (rc) {
			chg_err("get oplus,reserve_chg_soc error, rc=%d\n", rc);
			config->reserve_soc = RESERVE_SOC_DEFAULT;
		}
		rc = of_property_read_u32(node, "oplus,hidden_soc_percent", &config->hidden_soc_percent);
		if (rc) {
			chg_err("get oplus,hidden_soc_percent error, rc=%d\n", rc);
			config->hidden_soc_percent = HIDDEN_SOC_PERCENT_DEFAULT;
		}
	}

	if (config->hidden_soc_switch) {
		if (config->reserve_soc < RESERVE_SOC_MIN || config->reserve_soc > RESERVE_SOC_MAX ||
			config->hidden_soc_percent > HIDDEN_SOC_PERCENT_MAX ||
			config->hidden_soc_percent < HIDDEN_SOC_PERCENT_MIN) {
			config->reserve_soc = RESERVE_SOC_OFF;
			config->hidden_soc_switch = false;
		}
	}

	chg_info("hidden_soc_switch %d hidden_soc_percent %d\n", config->hidden_soc_switch,
		config->hidden_soc_percent);
}

static int oplus_comm_smooth_strategy_init(struct oplus_chg_comm *chip)
{
	int rc = 0;
	struct device_node *node = oplus_get_node_by_type(chip->dev->of_node);
	struct device_node *startegy_node;
	struct oplus_comm_spec_config *spec = &chip->spec;

	rc = of_property_read_string(node, "oplus_spec,smooth_strategy_name",
			(const char **)&chip->config.smooth_strategy_name);
	if (rc < 0) {
		chg_err("can't read oplus_spec,smooth_strategy_name, rc=%d\n", rc);
		goto not_support;
	}

	startegy_node = of_get_child_by_name(node, "smooth_strategy");
	chip->smooth_strategy = oplus_chg_strategy_alloc_by_node(chip->config.smooth_strategy_name, startegy_node);
	if (IS_ERR_OR_NULL(chip->smooth_strategy)) {
		chg_err("alloc smooth startegy error, rc=%ld", PTR_ERR(chip->smooth_strategy));
		goto not_support;
	}

	rc = of_property_read_u32(node, "oplus_spec,batt_full_low_soc_time", &spec->batt_full_low_soc_time);
	if (rc < 0) {
		chg_err("get oplus_spec,batt_full_low_soc_time error, rc=%d\n", rc);
		spec->batt_full_low_soc_time = 600;
	}

	rc = of_property_read_u32(node, "oplus_spec,batt_full_low_soc", &spec->batt_full_low_soc);
	if (rc < 0) {
		chg_err("get oplus_spec,batt_full_low_soc error, rc=%d\n", rc);
		spec->batt_full_low_soc = 99;
	}

	chip->config.smooth_switch = false;
	return 0;
not_support:
	chip->smooth_strategy = NULL;
	return -ENOTSUPP;
}

#define SOC_LADDER_0	2
#define SOC_LADDER_1	11
#define SOC_LADDER_2	60
#define SOC_LADDER_3	95
#define SOC_LADDER_4	100

#define DELAY_HZ_40	40
#define DELAY_HZ_60	60
#define DELAY_HZ_90	90
#define DELAY_HZ_150	150
#define DELAY_HZ_300	300
static void oplus_comm_parse_fcc_gear_dt(struct device_node *node,
		struct oplus_comm_spec_config *spec)
{
	u32 gear_cnt = 0;
	int thr_cnt;
	int i;
	int rc;
	u32 thr_tmp = 0;

	spec->fcc_gear_cnt = default_spec.fcc_gear_cnt;
	memcpy(spec->fcc_gear_thr_mv, default_spec.fcc_gear_thr_mv, sizeof(spec->fcc_gear_thr_mv));
	memcpy(spec->fcc_gear_shake_mv, default_spec.fcc_gear_shake_mv, sizeof(spec->fcc_gear_shake_mv));

	rc = of_property_read_u32(node, "oplus_spec,fcc-gear-cnt", &gear_cnt);
	if (gear_cnt > 0)
		spec->fcc_gear_cnt = min_t(int, gear_cnt, FCC_GEAR_MAX);

	thr_cnt = of_property_count_u32_elems(node, "oplus_spec,fcc-gear-thr-mv");
	if (thr_cnt == 1) {
		rc = of_property_read_u32(node, "oplus_spec,fcc-gear-thr-mv", &thr_tmp);
		if (rc < 0) {
			chg_err("get oplus_spec,fcc-gear-thr-mv property error, rc=%d\n", rc);
		} else {
			for (i = 0; i < spec->temp_region_max; i++)
				spec->fcc_gear_thr_mv[FCC_GEAR_LOW][i] = (int32_t)thr_tmp;
			spec->fcc_gear_cnt = 2;
		}
	} else if (thr_cnt > 1) {
		rc = read_unsigned_temp_region_data(node, "oplus_spec,fcc-gear-thr-mv",
						  (u32 *)spec->fcc_gear_thr_mv,
						  spec->temp_region_max, TEMP_REGION_MAX,
						  spec->fcc_gear_cnt - 1,
						  oplus_comm_temp_region_map);
		if (rc < 0)
			chg_err("get oplus_spec,fcc-gear-thr-mv property error, rc=%d\n", rc);
	}

	rc = read_signed_temp_region_data(node, "oplus_spec,fcc-gear-shake-mv",
					  (s32 *)spec->fcc_gear_shake_mv,
					  spec->temp_region_max, TEMP_REGION_MAX,
					  spec->fcc_gear_cnt - 1,
					  oplus_comm_temp_region_map);
	if (rc < 0)
		chg_err("get oplus_spec,fcc-gear-shake-mv property error, rc=%d\n", rc);
}

static int oplus_comm_parse_dt(struct oplus_chg_comm *comm_dev)
{
	struct device_node *node = oplus_get_node_by_type(comm_dev->dev->of_node);
	struct oplus_comm_spec_config *spec = &comm_dev->spec;
	struct oplus_comm_config *config = &comm_dev->config;
	int wls_ffc_fv_cutoff_temp[FFC_CHG_STEP_MAX] = { 0 };
	int i, m;
	int rc;

	spec->temp_region_max = oplus_comm_get_temp_region_max();
	spec->ffc_temp_region_max = oplus_comm_get_ffc_temp_region_max();

	chg_info("spec_ver:%d, temp_region_max:%d, ffc_temp_region_max:%d", oplus_get_chg_spec_version(),
		 spec->temp_region_max, spec->ffc_temp_region_max);

	memmove(spec->batt_temp_thr, default_spec.batt_temp_thr, sizeof(spec->batt_temp_thr));
	rc = read_signed_temp_region_data(node, "oplus_spec,batt-them-thr",
					(s32 *)spec->batt_temp_thr,
					spec->temp_region_max - 1, TEMP_REGION_MAX - 1, 1,
					oplus_comm_temp_region_map);
	if (rc < 0) {
		chg_err("get oplus_spec,batt-them-th property error, rc=%d\n",
			rc);
		for (i = 0; i < TEMP_REGION_MAX - 1; i++)
			spec->batt_temp_thr[i] = default_spec.batt_temp_thr[i];
	}

	rc = of_property_read_u32(node, "oplus_spec,iterm-ma", &spec->iterm_ma);
	if (rc < 0) {
		chg_err("get oplus_spec,iterm-ma property error, rc=%d\n", rc);
		spec->iterm_ma = default_spec.iterm_ma;
	}

	rc = of_property_read_u32(node, "oplus_spec,sub-iterm-ma", &spec->sub_iterm_ma);
	if (rc < 0) {
		chg_err("get oplus_spec,iterm-ma property error, rc=%d\n", rc);
		spec->sub_iterm_ma = default_spec.iterm_ma;
	}

	rc = read_unsigned_temp_region_data(node, "oplus_spec,fv-mv",
					  (u32 *)spec->fv_mv, spec->temp_region_max, TEMP_REGION_MAX, 1,
					  oplus_comm_temp_region_map);
	if (rc < 0) {
		chg_err("get oplus_spec,fv-mv property error, rc=%d\n", rc);
		for (i = 0; i < TEMP_REGION_MAX; i++)
			spec->fv_mv[i] = default_spec.fv_mv[i];
	}

	rc = read_unsigned_temp_region_data(node, "oplus_spec,sw-fv-mv",
					  (u32 *)spec->sw_fv_mv,
					  spec->temp_region_max, TEMP_REGION_MAX, 1,
					  oplus_comm_temp_region_map);
	if (rc < 0) {
		chg_err("get oplus_spec,sw-fv-mv property error, rc=%d\n", rc);
		for (i = 0; i < TEMP_REGION_MAX; i++)
			spec->sw_fv_mv[i] = default_spec.sw_fv_mv[i];
	}

	rc = read_unsigned_temp_region_data(node, "oplus_spec,hw-fv-inc-mv",
					  (u32 *)spec->hw_fv_inc_mv,
					  spec->temp_region_max, TEMP_REGION_MAX, 1,
					  oplus_comm_temp_region_map);
	if (rc < 0) {
		chg_err("get oplus_spec,hw-fv-inc-mv property error, rc=%d\n",
			rc);
		for (i = 0; i < TEMP_REGION_MAX; i++)
			spec->hw_fv_inc_mv[i] = default_spec.hw_fv_inc_mv[i];
	}

	rc = read_unsigned_temp_region_data(node, "oplus_spec,sw-over-fv-mv",
					  (u32 *)spec->sw_over_fv_mv,
					  spec->temp_region_max, TEMP_REGION_MAX, 1,
					  oplus_comm_temp_region_map);
	if (rc < 0) {
		chg_err("get oplus_spec,sw-over-fv-mv property error, rc=%d\n",
			rc);
		for (i = 0; i < TEMP_REGION_MAX; i++)
			spec->sw_over_fv_mv[i] = default_spec.sw_over_fv_mv[i];
	}

	rc = of_property_read_u32(node, "oplus_spec,sw-over-fv-dec-mv",
				  &spec->sw_over_fv_dec_mv);
	if (rc < 0) {
		chg_err("get oplus_spec,sw-over-fv-dec-mv property error, rc=%d\n",
			rc);
		spec->sw_over_fv_dec_mv = default_spec.sw_over_fv_dec_mv;
	}

	rc = of_property_read_u32(node, "oplus_spec,non-standard-sw-fv-mv",
				  &spec->non_standard_sw_fv_mv);
	if (rc < 0) {
		chg_err("get oplus_spec,non-standard-sw-fv-mv property error, rc=%d\n",
			rc);
		spec->non_standard_sw_fv_mv =
			default_spec.non_standard_sw_fv_mv;
	}
	rc = of_property_read_u32(node, "oplus_spec,non-standard-fv-mv",
				  &spec->non_standard_fv_mv);
	if (rc < 0) {
		chg_err("get oplus_spec,non-standard-fv-mv property error, rc=%d\n",
			rc);
		spec->non_standard_fv_mv = default_spec.non_standard_fv_mv;
	}
	rc = of_property_read_u32(node, "oplus_spec,non-standard-hw-fv-inc-mv",
				  &spec->non_standard_hw_fv_inc_mv);
	if (rc < 0) {
		chg_err("get oplus_spec,non-standard-hw-fv-inc-mv property error, rc=%d\n",
			rc);
		spec->non_standard_hw_fv_inc_mv =
			default_spec.non_standard_hw_fv_inc_mv;
	}
	rc = of_property_read_u32(node, "oplus_spec,non-standard-sw-over-fv-mv",
				  &spec->non_standard_sw_over_fv_mv);
	if (rc < 0) {
		chg_err("get oplus_spec,non-standard-sw-over-fv-mv property error, rc=%d\n",
			rc);
		spec->non_standard_sw_over_fv_mv =
			default_spec.non_standard_sw_over_fv_mv;
	}
	rc = of_property_read_u32(node, "oplus_spec,non-standard-vbatdet-mv",
				  &spec->non_standard_vbatdet_mv);
	if (rc < 0) {
		chg_err("get oplus_spec,non-standard-vbatdet-mv property error, rc=%d\n",
			rc);
		spec->non_standard_vbatdet_mv =
			default_spec.non_standard_vbatdet_mv;
	}

	rc = read_signed_temp_region_data(node, "oplus_spec,ffc-temp-thr",
					(s32 *)spec->ffc_temp_thr,
					spec->ffc_temp_region_max - 1 ,FFC_TEMP_REGION_MAX - 1, 1,
					oplus_comm_ffc_temp_region_map);
	if (rc < 0) {
		chg_err("get oplus_spec,ffc-temp-thr property error, rc=%d\n",
			rc);
		for (i = 0; i < FFC_TEMP_REGION_MAX - 1; i++)
			spec->ffc_temp_thr[i] = default_spec.ffc_temp_thr[i];
	}
	if (oplus_get_chg_spec_version() < OPLUS_CHG_SPEC_VER_V3P7)
		spec->ffc_temp_thr[FFC_TEMP_REGION_PRE_NORMAL_HIGH - 1] = 200;

	rc = of_property_read_u32(node, "oplus_spec,wired-ffc-step-max",
				  &spec->wired_ffc_step_max);
	if (rc < 0) {
		chg_err("get oplus_spec,wired-ffc-step-max property error, rc=%d\n",
			rc);
		spec->wired_ffc_step_max = default_spec.wired_ffc_step_max;
	}
	if (spec->wired_ffc_step_max > FFC_CHG_STEP_MAX) {
		chg_err("wired_ffc_step_max(=%d) more than %d\n", spec->wired_ffc_step_max, FFC_CHG_STEP_MAX);
		spec->wired_ffc_step_max = FFC_CHG_STEP_MAX;
	}
	rc = read_unsigned_data_from_node(node, "oplus_spec,wired-ffc-fv-mv",
					  (u32 *)spec->wired_ffc_fv_mv,
					  spec->wired_ffc_step_max);
	if (rc < 0) {
		chg_err("get oplus_spec,wired-ffc-fv-mv property error, rc=%d\n",
			rc);
		for (i = 0; i < spec->wired_ffc_step_max; i++)
			spec->wired_ffc_fv_mv[i] =
				default_spec.wired_ffc_fv_mv[i];
	}

	rc = of_property_read_u32(node, "oplus_spec,bdd-vbatdiff-mv", &spec->bdd_vbatdiff_mv);
	if (rc < 0) {
		chg_err("get oplus_spec,bdd-vbatdiff-mv property error, rc=%d\n", rc);
		spec->bdd_vbatdiff_mv = 0;
	}

	rc = of_property_read_u32(node, "oplus_spec,bdd-enter-current-ma", &spec->bdd_enter_current_ma);
	if (rc < 0) {
		chg_err("get oplus_spec,bdd-enter-current-ma property error, rc=%d\n", rc);
		spec->bdd_enter_current_ma = 0;
	}

	rc = of_property_read_u32(node, "oplus_spec,bdd-enter-voltage-mv", &spec->bdd_enter_voltage_mv);
	if (rc < 0) {
		chg_err("get oplus_spec,bdd-enter-voltage-mv property error, rc=%d\n", rc);
		spec->bdd_enter_voltage_mv = 0;
	}
	rc = of_property_read_u32(node, "oplus_spec,bdd-vbatdiff-timeout-min", &spec->bdd_vbatdiff_timeout_min);
	if (rc < 0) {
		chg_err("get oplus_spec,bdd-vbatdiff-timeout-min property error, rc=%d\n", rc);
		spec->bdd_vbatdiff_timeout_min = 0;
	}

	rc = read_unsigned_temp_region_data(node,
					  "oplus_spec,wired-ffc-fv-cutoff-mv",
					  (u32 *)spec->wired_ffc_fv_cutoff_mv,
					  spec->ffc_temp_region_max - 2, FFC_TEMP_REGION_MAX - 2,
					  spec->wired_ffc_step_max,
					  oplus_comm_ffc_temp_region_map);
	if (rc < 0) {
		chg_err("get oplus_spec,wired-ffc-fv-cutoff-mv property error, rc=%d\n",
			rc);
		for (i = 0; i < spec->wired_ffc_step_max; i++) {
			for (m = 0; m < (FFC_TEMP_REGION_MAX - 2); m++)
				spec->wired_ffc_fv_cutoff_mv[i][m] =
					default_spec.wired_ffc_fv_cutoff_mv[i][m];
		}
	}

	rc = read_unsigned_temp_region_data(node, "oplus_spec,wired-ffc-fcc-ma",
					  (u32 *)spec->wired_ffc_fcc_ma,
					  spec->ffc_temp_region_max - 2, FFC_TEMP_REGION_MAX - 2,
					  spec->wired_ffc_step_max,
					  oplus_comm_ffc_temp_region_map);
	if (rc < 0) {
		chg_err("get oplus_spec,wired-ffc-fcc-ma property error, rc=%d\n",
			rc);
		for (i = 0; i < spec->wired_ffc_step_max; i++) {
			for (m = 0; m < (FFC_TEMP_REGION_MAX - 2); m++)
				spec->wired_ffc_fcc_ma[i][m] =
					default_spec.wired_ffc_fcc_ma[i][m];
		}
	}
	rc = read_unsigned_temp_region_data(
		node, "oplus_spec,wired-ffc-fcc-cutoff-ma",
		(u32 *)spec->wired_ffc_fcc_cutoff_ma,
		spec->ffc_temp_region_max - 2, FFC_TEMP_REGION_MAX - 2,
		spec->wired_ffc_step_max,
		oplus_comm_ffc_temp_region_map);
	if (rc < 0) {
		chg_err("get oplus_spec,wired-ffc-fcc-cutoff-ma property error, rc=%d\n",
			rc);
		for (i = 0; i < spec->wired_ffc_step_max; i++) {
			for (m = 0; m < (FFC_TEMP_REGION_MAX - 2); m++)
				spec->wired_ffc_fcc_cutoff_ma[i][m] =
					default_spec
						.wired_ffc_fcc_cutoff_ma[i][m];
		}
	}

	rc = read_unsigned_temp_region_data(
		node, "oplus_spec,wired-ffc-fcc-sub-cutoff-ma",
		(u32 *)spec->wired_ffc_fcc_sub_cutoff_ma,
		spec->ffc_temp_region_max - 2, FFC_TEMP_REGION_MAX - 2,
		spec->wired_ffc_step_max,
		oplus_comm_ffc_temp_region_map);
	if (rc < 0) {
		chg_err("get oplus_spec,wired-ffc-fcc-sub-cutoff-ma property error, rc=%d\n",
			rc);
		for (i = 0; i < spec->wired_ffc_step_max; i++) {
			for (m = 0; m < (FFC_TEMP_REGION_MAX - 2); m++)
				spec->wired_ffc_fcc_sub_cutoff_ma[i][m] =
					default_spec
						.wired_ffc_fcc_cutoff_ma[i][m];
		}
	}

	rc = of_property_read_u32(node, "oplus_spec,wls-ffc-step-max",
				  &spec->wls_ffc_step_max);
	if (rc < 0) {
		chg_err("get oplus_spec,wls-ffc-step-max property error, rc=%d\n",
			rc);
		spec->wls_ffc_step_max = default_spec.wls_ffc_step_max;
	}
	if (spec->wls_ffc_step_max > FFC_CHG_STEP_MAX) {
		chg_err("wls_ffc_step_max(=%d) more than %d\n", spec->wls_ffc_step_max, FFC_CHG_STEP_MAX);
		spec->wls_ffc_step_max = FFC_CHG_STEP_MAX;
	}
	rc = read_unsigned_data_from_node(node, "oplus_spec,wls-ffc-fv-mv",
					  (u32 *)spec->wls_ffc_fv_mv,
					  spec->wls_ffc_step_max);
	if (rc < 0) {
		chg_err("get oplus_spec,wls-ffc-fv-mv property error, rc=%d\n",
			rc);
		for (i = 0; i < spec->wls_ffc_step_max; i++)
			spec->wls_ffc_fv_mv[i] = default_spec.wls_ffc_fv_mv[i];
	}
	rc = read_unsigned_temp_region_data(node,
					  "oplus_spec,wls-ffc-fv-cutoff-mv",
					  (u32 *)spec->wls_ffc_fv_cutoff_mv,
					  spec->ffc_temp_region_max - 2, FFC_TEMP_REGION_MAX - 2,
					  spec->wls_ffc_step_max,
					  oplus_comm_ffc_temp_region_map);
	if (rc < 0) {
		chg_err("get oplus_spec,wls-ffc-fv-cutoff-mv property error, rc=%d\n", rc);
		for (i = 0; i < spec->wls_ffc_step_max; i++) {
			for (m = 0; m < (FFC_TEMP_REGION_MAX - 2); m++)
				spec->wls_ffc_fv_cutoff_mv[i][m] =
					default_spec.wls_ffc_fv_cutoff_mv[i][m];
		}
	} else if (oplus_get_chg_spec_version() < OPLUS_CHG_SPEC_VER_V3P7) {
		if (of_property_count_elems_of_size(node, "oplus_spec,wls-ffc-fv-cutoff-mv", sizeof(u32)) ==
		    spec->wls_ffc_step_max) {
			if (read_unsigned_data_from_node(node, "oplus_spec,wls-ffc-fv-cutoff-mv",
					(u32 *)wls_ffc_fv_cutoff_temp, spec->wls_ffc_step_max) >= 0) {
				for (i = 0; i < spec->wls_ffc_step_max; i++) {
					for (m = 0; m < (FFC_TEMP_REGION_MAX - 2); m++)
						spec->wls_ffc_fv_cutoff_mv[i][m] = wls_ffc_fv_cutoff_temp[i];
				}
			}
		}
	}
	rc = read_unsigned_temp_region_data(
		node, "oplus_spec,wls-ffc-icl-ma", (u32 *)spec->wls_ffc_icl_ma,
		spec->ffc_temp_region_max - 2, FFC_TEMP_REGION_MAX - 2,
		spec->wls_ffc_step_max,
		oplus_comm_ffc_temp_region_map);
	if (rc < 0) {
		chg_err("get oplus_spec,wls-ffc-icl-ma property error, rc=%d\n",
			rc);
		for (i = 0; i < spec->wls_ffc_step_max; i++) {
			for (m = 0; m < (FFC_TEMP_REGION_MAX - 2); m++)
				spec->wls_ffc_icl_ma[i][m] =
					default_spec.wls_ffc_icl_ma[i][m];
		}
	}
	rc = read_unsigned_temp_region_data(
		node, "oplus_spec,wls-ffc-fcc-ma", (u32 *)spec->wls_ffc_fcc_ma,
		spec->ffc_temp_region_max - 2, FFC_TEMP_REGION_MAX - 2,
		spec->wls_ffc_step_max,
		oplus_comm_ffc_temp_region_map);
	if (rc < 0) {
		chg_err("get oplus_spec,wls-ffc-fcc-ma property error, rc=%d\n",
			rc);
		for (i = 0; i < spec->wls_ffc_step_max; i++) {
			for (m = 0; m < (FFC_TEMP_REGION_MAX - 2); m++)
				spec->wls_ffc_fcc_ma[i][m] =
					default_spec.wls_ffc_fcc_ma[i][m];
		}
	}
	rc = read_unsigned_temp_region_data(
		node, "oplus_spec,wls-ffc-fcc-cutoff-ma",
		(u32 *)spec->wls_ffc_fcc_cutoff_ma,
		spec->ffc_temp_region_max - 2, FFC_TEMP_REGION_MAX - 2,
		spec->wls_ffc_step_max,
		oplus_comm_ffc_temp_region_map);
	if (rc < 0) {
		chg_err("get oplus_spec,wls-ffc-fcc-cutoff-ma property error, rc=%d\n",
			rc);
		for (i = 0; i < spec->wls_ffc_step_max; i++) {
			for (m = 0; m < (FFC_TEMP_REGION_MAX - 2); m++)
				spec->wls_ffc_fcc_cutoff_ma[i][m] =
					default_spec.wls_ffc_fcc_cutoff_ma[i][m];
		}
	}
	rc = read_unsigned_temp_region_data(
		node, "oplus_spec,wls-ffc-fcc-sub-cutoff-ma",
		(u32 *)spec->wls_ffc_fcc_sub_cutoff_ma,
		spec->ffc_temp_region_max - 2, FFC_TEMP_REGION_MAX - 2,
		spec->wls_ffc_step_max,
		oplus_comm_ffc_temp_region_map);
	if (rc < 0) {
		chg_err("get oplus_spec,wls-ffc-fcc-sub-cutoff-ma property error, rc=%d\n",
			rc);
		for (i = 0; i < spec->wls_ffc_step_max; i++) {
			for (m = 0; m < (FFC_TEMP_REGION_MAX - 2); m++)
				spec->wls_ffc_fcc_sub_cutoff_ma[i][m] =
					default_spec.wls_ffc_fcc_cutoff_ma[i][m];
		}
	}

	oplus_comm_parse_aging_ffc_dt(comm_dev);

	rc = read_unsigned_temp_region_data(node, "oplus_spec,wired-vbatdet-mv",
					  (u32 *)spec->wired_vbatdet_mv,
					  spec->temp_region_max, TEMP_REGION_MAX, 1,
					  oplus_comm_temp_region_map);
	if (rc < 0) {
		chg_err("get oplus_spec,wired-vbatdet-mv property error, rc=%d\n",
			rc);
		for (i = 0; i < TEMP_REGION_MAX; i++)
			spec->wired_vbatdet_mv[i] =
				default_spec.wired_vbatdet_mv[i];
	}

	rc = read_unsigned_temp_region_data(node, "oplus_spec,wls-vbatdet-mv",
					  (u32 *)spec->wls_vbatdet_mv,
					  spec->temp_region_max, TEMP_REGION_MAX, 1,
					  oplus_comm_temp_region_map);
	if (rc < 0) {
		chg_err("get oplus_spec,wls-vbatdet-mv property error, rc=%d\n",
			rc);
		for (i = 0; i < TEMP_REGION_MAX; i++)
			spec->wls_vbatdet_mv[i] =
				default_spec.wls_vbatdet_mv[i];
	}

	oplus_comm_parse_fcc_gear_dt(node, spec);

	rc = of_property_read_u32(node, "oplus_spec,full-pre-ffc-mv",
					  &spec->full_pre_ffc_mv);
	if (rc < 0) {
		chg_err("get oplus_spec,full-pre-ffc-mv property error, rc=%d\n",
			rc);
		spec->full_pre_ffc_mv = 4455;
	}
	spec->full_pre_ffc_judge =
		of_property_read_bool(node, "oplus_spec,full_pre_ffc_judge");

	rc = of_property_read_u32(node, "oplus_spec,vbatt-ov-thr-mv",
				  &spec->vbatt_ov_thr_mv);
	if (rc < 0) {
		chg_err("get oplus_spec,vbatt-ov-thr-mv property error, rc=%d\n",
			rc);
		spec->vbatt_ov_thr_mv = default_spec.vbatt_ov_thr_mv;
	}

	rc = of_property_read_u32(node, "oplus_spec,vbat_uv_thr_mv",
				  &spec->vbat_uv_thr_mv);
	if (rc < 0) {
		chg_err("get oplus_spec,vbat_uv_thr_mv property error, rc=%d\n",
			rc);
		spec->vbat_uv_thr_mv = default_spec.vbat_uv_thr_mv;
	}
	rc = of_property_read_u32(node, "oplus_spec,vbat_charging_uv_thr_mv",
				  &spec->vbat_charging_uv_thr_mv);
	if (rc < 0) {
		chg_err("get oplus_spec,vbat_charging_uv_thr_mv property error, rc=%d\n",
			rc);
		spec->vbat_charging_uv_thr_mv =
			default_spec.vbat_charging_uv_thr_mv;
	}
	rc = of_property_read_u32(node, "oplus_spec,vbat_charging_uv_delata_mv",
				  &spec->vbat_charging_uv_delata_mv);
	if (rc < 0) {
		chg_err("get oplus_spec,vbat_charging_uv_delata_mv property error, rc=%d\n",
			rc);
		spec->vbat_charging_uv_delata_mv = GAUGE_VBAT_UV_DELATA;
	}

	rc = of_property_read_u32(node, "oplus_spec,tbatt_power_off_cali_temp",
		&spec->tbatt_power_off_cali_temp);
	if (rc < 0) {
		chg_err("get oplus_spec,tbatt_power_off_cali_temp error, rc=%d\n",
			rc);
		spec->tbatt_power_off_cali_temp = 0;
	}

	config->gauge_stuck_jump_support =
		of_property_read_bool(node, "oplus,gauge_stuck_jump_support");

	if (config->gauge_stuck_jump_support) {
		rc = of_property_read_u32(node, "oplus,gauge_stuck_threshold",
					&spec->gauge_stuck_threshold);
		if (rc < 0) {
			chg_err("get oplus_spec,gauge_stuck_threshold property error, rc=%d\n",
				rc);
			spec->gauge_stuck_threshold =
				default_spec.gauge_stuck_threshold;
		}

		rc = of_property_read_u32(node, "oplus,gauge_stuck_time",
					&spec->gauge_stuck_time);
		if (rc < 0) {
			chg_err("get oplus_spec,gauge_stuck_time property error, rc=%d\n",
				rc);
			spec->gauge_stuck_time =
				default_spec.gauge_stuck_time;
		}
	}

	rc = of_property_read_u32(node, "oplus_spec,sub_vbat_uv_thr_mv",
				  &spec->sub_vbat_uv_thr_mv);
	if (rc < 0) {
		chg_err("get oplus_spec,sub_vbat_uv_thr_mv property error, rc=%d\n",
			rc);
		spec->sub_vbat_uv_thr_mv = 0;
	}
	rc = of_property_read_u32(node, "oplus_spec,sub_vbat_charging_uv_thr_mv",
				  &spec->sub_vbat_charging_uv_thr_mv);
	if (rc < 0) {
		chg_err("get oplus_spec,sub_vbat_charging_uv_thr_mv property error, rc=%d\n",
			rc);
		spec->sub_vbat_charging_uv_thr_mv = 0;
	}

	rc = of_property_read_u32(node, "oplus_spec,removed_bat_decidegc",
			&spec->removed_bat_decidegc);
	if (rc < 0) {
		chg_err("get oplus_spec,removed_bat_decidegc, rc=%d, set as %d\n",
			rc, TEMP_BATTERY_STATUS__REMOVED);
		spec->removed_bat_decidegc = -TEMP_BATTERY_STATUS__REMOVED;
	} else {
		spec->removed_bat_decidegc = -spec->removed_bat_decidegc;
		chg_info("removed_bat_decidegc = %d \n", spec->removed_bat_decidegc);
	}
	rc = of_property_read_u32(node, "oplus_spec,poweroff_high_batt_temp",
				  &spec->poweroff_high_batt_temp);
	if (rc < 0) {
		chg_err("get oplus_spec,poweroff_high_batt_temp property error, rc=%d\n", rc);
		spec->poweroff_high_batt_temp = OPCHG_PWROFF_HIGH_BATT_TEMP;
	}

	rc = of_property_read_u32(node, "oplus_spec,poweroff_emergency_batt_temp",
				  &spec->poweroff_emergency_batt_temp);
	if (rc < 0) {
		chg_err("get oplus_spec,poweroff_emergency_batt_temp property error, rc=%d\n", rc);
		spec->poweroff_emergency_batt_temp = OPCHG_PWROFF_EMERGENCY_BATT_TEMP;
	}

	rc = of_property_read_u32(node, "oplus_spec,sw_check_full_cnt",
				  &spec->sw_check_full_cnt);
	if (rc < 0) {
		chg_err("get oplus_spec,sw_check_full_cnt error, rc=%d\n", rc);
		spec->sw_check_full_cnt = FULL_COUNTS_SW;
	}
	chg_info("spec->sw_check_full_cnt %d\n", spec->sw_check_full_cnt);

	rc = of_property_read_u32(node, "oplus,ui_soc_decimal_speedmin",
				  &config->ui_soc_decimal_speedmin);
	if (rc < 0) {
		chg_err("get oplus,ui_soc_decimal_speedmin property error, rc=%d\n",
			rc);
		config->ui_soc_decimal_speedmin = 2;
	}

	config->vooc_show_ui_soc_decimal =
		of_property_read_bool(node, "oplus,vooc_show_ui_soc_decimal");

	config->vooc_dis_show_ui_power =
		of_property_read_bool(node, "oplus,vooc_dis_show_ui_power");

	oplus_comm_parse_uv_dec_speed_tbl(comm_dev, node);

	rc = read_signed_data_from_node(node, "oplus_spec,drop_soc_2_temp_ladder",
					  (u32 *)&config->temp_ladder_of_drop_soc_2, LOW_VOLT_UISOC_LADDER_NUMBER);
	if (rc < 0) {
		chg_err("get oplus_spec,temp_ladder_of_drop_soc_2 property error, rc=%d\n",
			rc);
		config->temp_ladder_of_drop_soc_2[0] = (-50);
		for (i = 1; i < LOW_VOLT_UISOC_LADDER_NUMBER; i++)
			config->temp_ladder_of_drop_soc_2[i] = 0;
	}

	rc = read_unsigned_data_from_node(node, "oplus_spec,volt_diff_ladder_of_drop_soc_2",
					  (u32 *)&config->volt_diff_ladder_of_drop_soc_2, LOW_VOLT_UISOC_LADDER_NUMBER);
	if (rc < 0) {
		chg_err("get oplus_spec,volt_diff_ladder_of_drop_soc_2 property error, rc=%d\n",
			rc);
		config->volt_diff_ladder_of_drop_soc_2[0] = 20;
		config->volt_diff_ladder_of_drop_soc_2[1] = 50;
		for (i = 2; i < LOW_VOLT_UISOC_LADDER_NUMBER; i++)
			config->volt_diff_ladder_of_drop_soc_2[i] = 0;
	}

	rc = read_signed_data_from_node(node, "oplus_spec,keep_soc_2_temp_ladder",
					  (u32 *)&config->temp_ladder_of_keep_soc_2, LOW_VOLT_UISOC_LADDER_NUMBER);
	if (rc < 0) {
		chg_err("get oplus_spec,temp_ladder_of_keep_soc_2 property error, rc=%d\n",
			rc);
		config->temp_ladder_of_keep_soc_2[0] = (-200);
		config->temp_ladder_of_keep_soc_2[1] = (-50);
		for (i = 2; i < LOW_VOLT_UISOC_LADDER_NUMBER; i++)
			config->temp_ladder_of_keep_soc_2[i] = 0;
	}
	rc = read_unsigned_data_from_node(node, "oplus_spec,volt_diff_ladder_of_keep_soc_2",
					  (u32 *)&config->volt_diff_ladder_of_keep_soc_2, LOW_VOLT_UISOC_LADDER_NUMBER);
	if (rc < 0) {
		chg_err("get oplus_spec,volt_diff_ladder_of_keep_soc_2 property error, rc=%d\n",
			rc);
		config->volt_diff_ladder_of_keep_soc_2[0] = 10;
		config->volt_diff_ladder_of_keep_soc_2[1] = 20;
		config->volt_diff_ladder_of_keep_soc_2[2] = 30;
		for (i = 3; i < LOW_VOLT_UISOC_LADDER_NUMBER; i++)
			config->volt_diff_ladder_of_keep_soc_2[i] = 0;
	}

	rc = read_signed_data_from_node(node, "oplus_spec,keep_soc_3_temp_ladder",
					  (u32 *)&config->temp_ladder_of_keep_soc_3, LOW_VOLT_UISOC_LADDER_NUMBER);
	if (rc < 0) {
		chg_err("get oplus_spec,temp_ladder_of_keep_soc_3 property error, rc=%d\n",
			rc);
		for (i = 0; i < LOW_VOLT_UISOC_LADDER_NUMBER; i++)
			config->temp_ladder_of_keep_soc_3[i] = 0;
	}
	rc = read_unsigned_data_from_node(node, "oplus_spec,volt_diff_ladder_of_keep_soc_3",
					  (u32 *)&config->volt_diff_ladder_of_keep_soc_3, LOW_VOLT_UISOC_LADDER_NUMBER);
	if (rc < 0) {
		chg_err("get oplus_spec,volt_diff_ladder_of_keep_soc_3 property error, rc=%d\n",
			rc);
		for (i = 0; i < LOW_VOLT_UISOC_LADDER_NUMBER; i++)
			config->volt_diff_ladder_of_keep_soc_3[i] = 0;
	}

	rc = of_property_read_u32(node, "oplus_spec,back_rm_of_drop_soc_2", (u32 *)&config->back_rm_of_drop_soc_2);
	if (rc < 0)
		config->back_rm_of_drop_soc_2 = 60;

	rc = of_property_read_u32(node, "oplus_spec,back_rm_of_drop_soc_1", (u32 *)&config->back_rm_of_drop_soc_1);
	if (rc < 0)
		config->back_rm_of_drop_soc_1 = 60;

	rc = of_property_read_u32(node, "oplus_spec,load_current_of_drop_soc_2", (u32 *)&config->load_current_of_drop_soc_2);
	if (rc < 0)
		config->load_current_of_drop_soc_2 = 500;

	rc = of_property_read_u32(node, "oplus_spec,load_current_of_drop_soc_1", (u32 *)&config->load_current_of_drop_soc_1);
	if (rc < 0)
		config->load_current_of_drop_soc_1 = 1000;

	rc = of_property_read_u32(node, "oplus_spec,current_limit_of_drop_soc_2", (u32 *)&config->current_limit_of_drop_soc_2);
	if (rc < 0)
		config->current_limit_of_drop_soc_2 = 350;

	rc = of_property_read_u32(node, "oplus_spec,volt_of_fast_drop_soc_1", (u32 *)&config->volt_of_fast_drop_soc_1);
	if (rc < 0)
		config->volt_of_fast_drop_soc_1 = 3050;

	config->support_uisoc_low_battery_control =
		of_property_read_bool(node, "oplus_spec,support_uisoc_low_battery_control");

	rc = read_signed_data_from_node(node, "oplus_spec,soc_down_ladder",
					  (u32 *)&config->soc_down_ladder, LOW_VOLT_UISOC_LADDER_NUMBER);
	if (rc < 0) {
		chg_err("get oplus_spec,soc_down_ladder error, rc=%d\n", rc);
		for (i = 0; i < LOW_VOLT_UISOC_LADDER_NUMBER; i++)
			config->soc_down_ladder[i] = 0;
		config->soc_down_ladder[0] = SOC_LADDER_0;
		config->soc_down_ladder[1] = SOC_LADDER_1;
		config->soc_down_ladder[2] = SOC_LADDER_2;
		config->soc_down_ladder[3] = SOC_LADDER_3;
		config->soc_down_ladder[4] = SOC_LADDER_4;
	}

	rc = read_unsigned_data_from_node(node, "oplus_spec,soc_down_delay_hz_ladder",
					  (u32 *)&config->soc_down_delay_hz_ladder, LOW_VOLT_UISOC_LADDER_NUMBER);
	if (rc < 0) {
		chg_err("get oplus_spec,soc_down_delay_hz_ladder error, rc=%d\n", rc);
		for (i = 0; i < LOW_VOLT_UISOC_LADDER_NUMBER; i++)
			config->soc_down_delay_hz_ladder[i] = 0;
		config->soc_down_delay_hz_ladder[0] = DELAY_HZ_40;
		config->soc_down_delay_hz_ladder[1] = DELAY_HZ_40;
		if (config->support_uisoc_low_battery_control)
			config->soc_down_delay_hz_ladder[1] = UI_SOC_DEC_SPEED_OF_LOW_BATT;
		config->soc_down_delay_hz_ladder[2] = DELAY_HZ_40;
		config->soc_down_delay_hz_ladder[3] = DELAY_HZ_60;
		config->soc_down_delay_hz_ladder[4] = DELAY_HZ_150;
		config->soc_down_delay_hz_ladder[5] = DELAY_HZ_300;
	}

	rc = read_unsigned_data_from_node(node, "oplus_spec,soc_down_chg_delay_hz_ladder",
					  (u32 *)&config->soc_down_chg_delay_hz_ladder, LOW_VOLT_UISOC_LADDER_NUMBER);
	if (rc < 0) {
		chg_err("get oplus_spec,soc_down_chg_delay_hz_ladder error, rc=%d\n", rc);
		for (i = 0; i < LOW_VOLT_UISOC_LADDER_NUMBER; i++)
			config->soc_down_chg_delay_hz_ladder[i] = 0;
		config->soc_down_chg_delay_hz_ladder[0] = DELAY_HZ_90;
		config->soc_down_chg_delay_hz_ladder[1] = DELAY_HZ_40;
		if (config->support_uisoc_low_battery_control)
			config->soc_down_chg_delay_hz_ladder[1] = UI_SOC_DEC_SPEED_OF_LOW_BATT;
		config->soc_down_chg_delay_hz_ladder[2] = DELAY_HZ_40;
		config->soc_down_chg_delay_hz_ladder[3] = DELAY_HZ_60;
		config->soc_down_chg_delay_hz_ladder[4] = DELAY_HZ_150;
		config->soc_down_chg_delay_hz_ladder[5] = DELAY_HZ_300;
	}

	rc = of_property_read_u32(node, "oplus,soc_down_default_delay_hz", &config->soc_down_default_delay_hz);
	if (rc < 0) {
		chg_err("get oplus,soc_down_default_delay_hz error, rc=%d\n", rc);
		config->soc_down_default_delay_hz = (40);
	}

	rc = of_property_read_u32(node, "oplus,ui_soc_2_voltage_comp_mv", &config->ui_soc_2_voltage_comp_mv);
	if (rc < 0)
		config->ui_soc_2_voltage_comp_mv = INT_MAX;

	rc = of_property_read_u32(node, "oplus,ui_soc_3_voltage_comp_mv", &config->ui_soc_3_voltage_comp_mv);
	if (rc < 0)
		config->ui_soc_3_voltage_comp_mv = INT_MAX;

	rc = of_property_read_u32(node, "oplus,chg_shutdown_max_mv", &config->chg_shutdown_max_mv);
	if (rc < 0)
		config->chg_shutdown_max_mv = -EINVAL;
	config->dual_panel_support = of_property_read_bool(node, "oplus,dual_panel_support");
	config->not_pop_up = of_property_read_bool(node, "oplus,not_pop_up");
	chg_info("not_pop_up = %d\n", config->not_pop_up);

	comm_dev->use_pm_power_off_with_hightemp = of_property_read_bool(node, "oplus,pm_power_off_with_hightemp");
	chg_info("comm_dev->use_pm_power_off_with_hightemp = %d\n", comm_dev->use_pm_power_off_with_hightemp);
	rc = of_property_read_u32(node, "oplus_spec,batt_full_time", &spec->batt_full_time);
	if (rc < 0) {
		chg_err("get oplus_spec,batt_full_time error, rc=%d\n", rc);
		spec->batt_full_time = 60;
	}

	config->support_gauge_r_track = of_property_read_bool(node, "oplus_spec,support_gauge_r_track");
	config->support_hw_iterm_check = of_property_read_bool(node, "oplus_spec,hw_iterm_support");
	config->support_boost_ic = of_property_read_bool(node, "oplus_spec,support_boost_ic");
	chg_info("support_boost_ic = %d\n", config->support_boost_ic);
	oplus_comm_parse_dec_vol_dt(comm_dev);

	oplus_comm_parse_smooth_soc_dt(comm_dev);

	oplus_comm_parse_hidden_soc_dt(comm_dev);

	oplus_comm_parse_from_cmdline(comm_dev);

	oplus_comm_smooth_strategy_init(comm_dev);

	return 0;
}

static int oplus_comm_fv_max_vote_callback(struct votable *votable, void *data,
				int fv_mv, const char *client, bool step)
{
	struct oplus_chg_comm *chip = data;
	int rc;

	if (fv_mv < 0)
		fv_mv = 0;

	rc = vote(chip->fv_min_votable, FV_MAX_VOTER, true, fv_mv, false);

	return rc;
}

static int oplus_comm_fv_min_vote_callback(struct votable *votable, void *data,
				int fv_mv, const char *client, bool step)
{
	int rc;

	if (fv_mv < 0)
		fv_mv = 0;

	rc = oplus_wired_set_fv(fv_mv);

	return rc;
}

static int oplus_comm_chg_disable_vote_callback(struct votable *votable,
						void *data, int disable,
						const char *client, bool step)
{
	struct oplus_chg_comm *chip = data;
	int rc;

	if (chip->charging_disable == !!disable)
		return 0;
	chip->charging_disable = !!disable;
	rc = oplus_comm_charging_disable(chip, chip->charging_disable);
	if (rc < 0)
		chg_err("can't set charging %s\n", !!disable ? "disable" : "enable");
	else
		chg_info("set charging %s\n", !!disable ? "disable" : "enable");

	return rc;
}

static int oplus_comm_chg_suspend_vote_callback(struct votable *votable,
						void *data, int suspend,
						const char *client, bool step)
{
	struct oplus_chg_comm *chip = data;
	int rc;

	if (chip->charge_suspend == !!suspend)
		return 0;
	chip->charge_suspend = !!suspend;
	rc = oplus_comm_charge_suspend(chip, chip->charge_suspend);
	if (rc < 0)
		chg_err("can't set charge %s\n", !!suspend ? "suspend" : "unsuspend");
	else
		chg_info("set charge %s\n", !!suspend ? "suspend" : "unsuspend");

	return rc;
}

static int oplus_comm_cool_down_vote_callback(struct votable *votable,
					      void *data, int cool_down,
					      const char *client, bool step)
{
	struct oplus_chg_comm *chip = data;
	int rc;

	if (cool_down < 0) {
		chg_err("cool_down=%d\n", cool_down);
		cool_down = 0;
	}

	rc = oplus_comm_set_cool_down_level(chip, cool_down);

	return rc;
}

static int oplus_comm_vote_init(struct oplus_chg_comm *chip)
{
	int rc;

	chip->fv_max_votable = create_votable("FV_MAX", VOTE_MAX,
				oplus_comm_fv_max_vote_callback,
				chip);
	if (IS_ERR(chip->fv_max_votable)) {
		rc = PTR_ERR(chip->fv_max_votable);
		chip->fv_max_votable = NULL;
		return rc;
	}

	chip->fv_min_votable = create_votable("FV_MIN", VOTE_MIN,
				oplus_comm_fv_min_vote_callback,
				chip);
	if (IS_ERR(chip->fv_min_votable)) {
		rc = PTR_ERR(chip->fv_min_votable);
		chip->fv_min_votable = NULL;
		goto creat_fv_min_votable_err;
	}
	/* vote defult value avoid power on with high temp, write 0 to fv */
	vote(chip->fv_max_votable, SPEC_VOTER, true,
		chip->spec.fv_mv[TEMP_REGION_NORMAL], false);
	chip->chg_disable_votable =
		create_votable("CHG_DISABLE", VOTE_SET_ANY,
			       oplus_comm_chg_disable_vote_callback, chip);
	if (IS_ERR(chip->chg_disable_votable)) {
		rc = PTR_ERR(chip->chg_disable_votable);
		chip->chg_disable_votable = NULL;
		goto creat_chg_disable_votable_err;
	}

	chip->chg_suspend_votable =
		create_votable("CHG_SUSPEND", VOTE_SET_ANY,
			       oplus_comm_chg_suspend_vote_callback, chip);
	if (IS_ERR(chip->chg_suspend_votable)) {
		rc = PTR_ERR(chip->chg_suspend_votable);
		chip->chg_suspend_votable = NULL;
		goto creat_chg_suspend_votable_err;
	}

	chip->cool_down_votable =
		create_votable("COOL_DOWN", VOTE_MIN,
			       oplus_comm_cool_down_vote_callback, chip);
	if (IS_ERR(chip->cool_down_votable)) {
		rc = PTR_ERR(chip->cool_down_votable);
		chip->cool_down_votable = NULL;
		goto creat_cool_down_votable_err;
	}

	/* Make sure cool_down is set to 0 */
	vote(chip->cool_down_votable, USER_VOTER, true, 0, false);
	vote(chip->cool_down_votable, USER_VOTER, false, 0, false);

	return 0;

creat_cool_down_votable_err:
	destroy_votable(chip->chg_suspend_votable);
creat_chg_suspend_votable_err:
	destroy_votable(chip->chg_disable_votable);
creat_chg_disable_votable_err:
	destroy_votable(chip->fv_min_votable);
creat_fv_min_votable_err:
	destroy_votable(chip->fv_max_votable);
	return rc;
}

static void oplus_comm_set_boot_completed(
		struct oplus_chg_comm *chip, bool boot_completed)
{
	int rc;
	struct mms_msg *msg;

	if (chip->soc_decimal.boot_completed == boot_completed)
		return;

	chip->soc_decimal.boot_completed = boot_completed;
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  COMM_ITEM_BOOT_COMPLETED);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish boot completed error, rc=%d\n", rc);
		kfree(msg);
	}
}

static ssize_t proc_ui_soc_decimal_write(struct file *file,
               const char __user *buf, size_t len, loff_t *data)
{
	struct oplus_chg_comm *chip = pde_data(file_inode(file));
	char buffer[2] = {0};

	if (chip == NULL)
		  return  -EFAULT;

	if (len > 2)
		  return -EFAULT;

	if (copy_from_user(buffer, buf, 2)) {
		  chg_err("%s:  error.\n", __func__);
		  return -EFAULT;
	}
	if (buffer[0] == '0')
		g_boot_completed = false;
	else
		g_boot_completed = true;

	oplus_comm_set_boot_completed(chip, g_boot_completed);
	chg_info("ui_soc_decimal: boot_completed=%d", chip->soc_decimal.boot_completed);
	return len;
}

static ssize_t proc_ui_soc_decimal_read(struct file *file,
		char __user *buff, size_t count, loff_t *off)
{
	struct oplus_chg_comm *chip = pde_data(file_inode(file));
	struct oplus_comm_config *config = &chip->config;
	struct ui_soc_decimal *soc_decimal = &chip->soc_decimal;
	char page[256] = {0};
	char read_data[128] = {0};
	int len = 0;
	int val;
	int vooc_sid;

	/* Avoid timing issues caused by modifying the vooc adapter ID test */
	vooc_sid = chip->vooc_sid;
	if (config->vooc_show_ui_soc_decimal) {
		if (!soc_decimal->calculate_decimal_time &&
		    !chip->wls_online && ((chip->vooc_online && vooc_sid &&
		    sid_to_adapter_chg_type(vooc_sid) == CHARGER_TYPE_SVOOC)
		    || chip->ufcs_online || chip->pps_online)) {
			cancel_delayed_work_sync(&chip->ui_soc_decimal_work);
			oplus_comm_ui_soc_decimal_init(chip);
			schedule_delayed_work(&chip->ui_soc_decimal_work, 0);
		}

		val = (soc_decimal->ui_soc_integer + soc_decimal->ui_soc_decimal) / 10;
		if(!soc_decimal->decimal_control)
			val = 0;
	} else {
		val = 0;
	}

	if (get_chg_up_not_limit_state(chip->ui_soc, chip->smooth_soc) == false) {
		soc_decimal->init_decimal_ui_soc = 0;
		val = 0;
	}

	sprintf(read_data, "%d, %d", soc_decimal->init_decimal_ui_soc / 10, val);
	chg_info("APK successful, %d,%d", soc_decimal->init_decimal_ui_soc / 10, val);
	len = sprintf(page, "%s", read_data);

	if (len > *off) {
		len -= *off;
	} else {
		len = 0;
	}
	if (copy_to_user(buff, page, (len < count ? len : count))) {
		return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations ui_soc_decimal_ops =
{
	.write  = proc_ui_soc_decimal_write,
	.read = proc_ui_soc_decimal_read,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops ui_soc_decimal_ops =
{
	.proc_write  = proc_ui_soc_decimal_write,
	.proc_read  = proc_ui_soc_decimal_read,
};
#endif

static ssize_t proc_batt_param_noplug_write(struct file *filp,
					    const char __user *buf, size_t len,
					    loff_t *data)
{
	return len;
}

static ssize_t proc_batt_param_noplug_read(struct file *filp, char __user *buff,
					   size_t count, loff_t *off)
{
	char page[256] = { 0 };
	char read_data[128] = { 0 };
	int len;

	sprintf(read_data, "%d %d %d", noplug_temperature, noplug_batt_volt_max,
		noplug_batt_volt_min);
	len = sprintf(page, "%s", read_data);
	if (len > *off)
		len -= *off;
	else
		len = 0;
	if (copy_to_user(buff, page, (len < count ? len : count)))
		return -EFAULT;
	*off += len < count ? len : count;

	return (len < count ? len : count);
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations batt_param_noplug_proc_fops = {
	.write = proc_batt_param_noplug_write,
	.read = proc_batt_param_noplug_read,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops batt_param_noplug_proc_fops = {
	.proc_write = proc_batt_param_noplug_write,
	.proc_read = proc_batt_param_noplug_read,
};
#endif

#define OPLUS_TBATT_HIGH_PWROFF_COUNT	(18)
#define OPLUS_TBATT_EMERGENCY_PWROFF_COUNT	(6)

DECLARE_WAIT_QUEUE_HEAD(oplus_tbatt_pwroff_wq);

static int oplus_comm_tbatt_power_off_kthread(void *arg)
{
	int over_temp_count = 0, emergency_count = 0;
	struct oplus_chg_comm *chip = (struct oplus_chg_comm *)arg;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };
	union mms_msg_data data = { 0 };
	int pwroff_over_temp_th = chip->spec.poweroff_high_batt_temp;
	int pwroff_emerg_th = chip->spec.poweroff_emergency_batt_temp;
	int temp = 0;

	sched_setscheduler(current, SCHED_FIFO, &param);
	tbatt_pwroff_enable = 1;

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if (get_eng_version() == FACTORY) {
		pwroff_over_temp_th = FACTORY_PWROFF_HIGH_BATT_TEMP;
		pwroff_emerg_th = FACTORY_PWROFF_EMERGENCY_BATT_TEMP;
	}
#endif
	chg_info("tbatt_pwroff_th:[%d] tbatt_emergency_pwroff_th:[%d]\n", pwroff_over_temp_th, pwroff_emerg_th);

	while (!kthread_should_stop()) {
		schedule_timeout_interruptible(
			round_jiffies_relative(msecs_to_jiffies(5 * 1000)));
		if (!tbatt_pwroff_enable) {
			emergency_count = 0;
			over_temp_count = 0;
			wait_event_interruptible(oplus_tbatt_pwroff_wq,
						 tbatt_pwroff_enable == 1);
		}
		/*
		 * Get the battery temperature directly from the fuel gauge
		 * when the battery temperature exceeds 60 degrees
		 */
		oplus_mms_get_item_data(
			chip->gauge_topic, GAUGE_ITEM_TEMP, &data,
			(chip->batt_temp >= OPCHG_PWROFF_FORCE_UPDATE_BATT_TEMP));
		chip->batt_temp = data.intval;
		temp = chip->batt_temp + chip->spec.tbatt_power_off_cali_temp;
		if (temp > pwroff_emerg_th) {
			emergency_count++;
			chg_err(" emergency_count:%d \n", emergency_count);
		} else {
			emergency_count = 0;
		}
		if (temp > pwroff_over_temp_th) {
			over_temp_count++;
			chg_err("over_temp_count[%d] \n", over_temp_count);
		} else {
			over_temp_count = 0;
		}
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
		if (get_eng_version() != HIGH_TEMP_AGING) {
#endif
			if (oplus_is_power_off_charging() && chip->spec.support_hot_enter_kpoc) {
			} else {
				if (over_temp_count >= OPLUS_TBATT_HIGH_PWROFF_COUNT ||
					emergency_count >=
						OPLUS_TBATT_EMERGENCY_PWROFF_COUNT) {
					chg_err("battery temperature is too high, goto power off\n");
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
					machine_power_off();
#else
					if (chip->use_pm_power_off_with_hightemp && pm_power_off) {
						chg_err("pm_power_off\n");
						pm_power_off();
					} else {
						chg_err("kernel_power_off\n");
						kernel_power_off();
					}
#endif
				}
			}
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
		}
#endif
	}
	return 0;
}

#define TASK_INIT_RETRY_MAX	5
static int oplus_comm_tbatt_power_off_task_init(struct oplus_chg_comm *chip)
{
	int rc;
	static int retry_count;

retry:
	chip->tbatt_pwroff_task = kthread_create(
		oplus_comm_tbatt_power_off_kthread, chip, "tbatt_pwroff");
	if (!IS_ERR(chip->tbatt_pwroff_task)) {
		wake_up_process(chip->tbatt_pwroff_task);
	} else {
		rc = PTR_ERR(chip->tbatt_pwroff_task);
		chg_err("tbatt_pwroff_task creat fail, retry_count=%d, rc=%d\n",
			retry_count, rc);
		if (retry_count < TASK_INIT_RETRY_MAX) {
			retry_count++;
			goto retry;
		}
		return rc;
	}
	return 0;
}

static ssize_t proc_tbatt_pwroff_write(struct file *file,
		const char __user *buf, size_t len, loff_t *data)
{
	char buffer[2] = {0};

	if (len > 2) {
		return -EFAULT;
	}
	if (copy_from_user(buffer, buf, 2)) {
		chg_err("copy data error\n");
		return -EFAULT;
	}
	if (buffer[0] == '0') {
		tbatt_pwroff_enable = 0;
	} else if (buffer[0] == '1') {
		tbatt_pwroff_enable = 1;
		wake_up_interruptible(&oplus_tbatt_pwroff_wq);
	} else if (buffer[0] == '2') {
		chg_info("flash_screen_ctrl_status close\n");
	} else if (buffer[0] == '3') {
		chg_info("flash_screen_ctrl_status open\n");
	}
	chg_err("tbatt_pwroff_enable=%d\n", tbatt_pwroff_enable);
	return len;
}

static ssize_t proc_tbatt_pwroff_read(struct file *file,
		char __user *buff, size_t count, loff_t *off)
{
	char page[256] = {0};
	char read_data[3] = {0};
	int len = 0;

	if (tbatt_pwroff_enable == 1) {
		read_data[0] = '1';
	} else {
		read_data[0] = '0';
	}
	read_data[1] = '\0';
	len = sprintf(page, "%s", read_data);
	if (len > *off) {
		len -= *off;
	} else {
		len = 0;
	}
	if (copy_to_user(buff, page, (len < count ? len : count))) {
		return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations tbatt_pwroff_proc_fops = {
	.write = proc_tbatt_pwroff_write,
	.read = proc_tbatt_pwroff_read,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops tbatt_pwroff_proc_fops = {
	.proc_write = proc_tbatt_pwroff_write,
	.proc_read = proc_tbatt_pwroff_read,
};
#endif

static ssize_t proc_charger_factorymode_test_write(struct file *file,
						   const char __user *buf,
						   size_t count, loff_t *lo)
{
	struct oplus_chg_comm *chip = pde_data(file_inode(file));
	char buffer[2] = { 0 };
	struct mms_msg *msg;
	int rc;

	if (chip == NULL) {
		chg_err("file private data is empty\n");
		return -1;
	}

	if (count > 2) {
		return -1;
	}
	if (copy_from_user(buffer, buf, 1)) {
		chg_err("copy data error\n");
		return -1;
	}

	chip->factory_test_mode = buffer[0] - '0';
	chg_info("set factory_test_mode=%d\n", chip->factory_test_mode);
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  COMM_ITEM_FACTORY_TEST);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish factory test msg error, rc=%d\n", rc);
		kfree(msg);
		return rc;
	}

	return count;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations proc_charger_factorymode_test_ops =
{
	.write  = proc_charger_factorymode_test_write,
	.open  = simple_open,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops proc_charger_factorymode_test_ops =
{
	.proc_write  = proc_charger_factorymode_test_write,
	.proc_open  = simple_open,
};
#endif

#define PROC_READ_MAX_SIZE 32
#define PROC_READ_PAGE_SIZE 256
static ssize_t proc_integrate_gauge_fcc_flag_read(struct file *filp,
		char __user *buff, size_t count, loff_t *off)
{
	char page[PROC_READ_PAGE_SIZE] = {0};
	char read_data[PROC_READ_MAX_SIZE] = {0};
	int len = 0;

	read_data[0] = '1';
	len = sprintf(page, "%s", read_data);
	if(len > *off) {
		len -= *off;
	} else {
		len = 0;
	}
	if (copy_to_user(buff, page, (len < count ? len : count))) {
		return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations proc_integrate_gauge_fcc_flag_ops =
{
	.read  = proc_integrate_gauge_fcc_flag_read,
	.open  = simple_open,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops proc_integrate_gauge_fcc_flag_ops =
{
	.proc_read  = proc_integrate_gauge_fcc_flag_read,
	.proc_open  = simple_open,
};
#endif

static ssize_t proc_hmac_read(struct file *filp, char __user *buff,
			      size_t count, loff_t *off)
{
	struct oplus_chg_comm *chip = pde_data(file_inode(filp));
	char buf[2] = { 0 };
	int len;

	if (chip == NULL)
		return -EFAULT;

	chip->hmac = oplus_gauge_get_batt_hmac();
	if (chip->hmac || chip->config.not_pop_up)
		buf[0] = '1';
	else
		buf[0] = '0';
	len = ARRAY_SIZE(buf) - 1;
	if (len > *off)
		len -= *off;
	else
		len = 0;

	if (copy_to_user(buff, buf, (len < count ? len : count))) {
		chg_err("copy_to_user error, hmac = %d.\n", chip->hmac);
		return -EFAULT;
	}
	*off += len < count ? len : count;

	return (len < count ? len : count);
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations hmac_proc_fops = {
	.read = proc_hmac_read,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops hmac_proc_fops = {
	.proc_read = proc_hmac_read,
};
#endif

static void oplus_comm_reset_chginfo(struct oplus_chg_comm *chip)
{
	struct oplus_comm_spec_config *spec;
	if (chip == NULL)
		return;

	spec = &chip->spec;
	chip->ffc_charging = false;
	chip->sw_full = false;
	chip->hw_full_by_sw = false;
	oplus_comm_set_batt_full(chip, false);
	if (is_support_parallel_battery(chip->gauge_topic)) {
		chip->sw_sub_batt_full = false;
		chip->hw_sub_batt_full_by_sw = false;
		oplus_comm_set_sub_batt_full(chip, false);
	}
	oplus_comm_set_ffc_status(chip, FFC_DEFAULT);

	cancel_delayed_work_sync(&chip->charge_timeout_work);
	/* ensure that max_chg_time_sec has been obtained */
	chip->need_start_timeout_work = true;
	oplus_comm_start_timeout_work(chip);

	schedule_work(&chip->wired_chg_check_work);
}

static int oplus_comm_push_chg_cycle_info_msg(struct oplus_chg_comm *chip, char *chg_cycle)
{
	struct mms_msg *msg;
	int rc;

	if (!chg_cycle)
		return -EINVAL;

	if (!is_err_topic_available(chip)) {
		chg_err("error topic not found\n");
		return -ENODEV;
	}

	msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_CHG_CYCLE, "$$chg_cycle@@%s", chg_cycle);
	if (msg == NULL) {
		chg_err("alloc chg cycle error msg error\n");
		return -ENOMEM;
	}

	rc = oplus_mms_publish_msg(chip->err_topic, msg);
	if (rc < 0) {
		chg_err("publish chg cycle error msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static ssize_t oplus_comm_chg_cycle_write(struct file *file,
		const char __user *buff, size_t count, loff_t *ppos)
{
	struct oplus_chg_comm *chip = pde_data(file_inode(file));
	char proc_chg_cycle_data[16];

	if(count >= sizeof(proc_chg_cycle_data))
		count = sizeof(proc_chg_cycle_data) - 1;

	if (copy_from_user(&proc_chg_cycle_data, buff, count)) {
		chg_err("chg_cycle_write error.\n");
		return -EFAULT;
	}
	proc_chg_cycle_data[count] = '\0';

	if ((strncmp(proc_chg_cycle_data, "en808", 5) == 0) ||
	    (strncmp(proc_chg_cycle_data, "user_enable", 11) == 0)) {
		if(chip->unwakelock_chg) {
			chg_err("unwakelock testing, this test not allowed\n");
			return -EPERM;
		}
		if (strncmp(proc_chg_cycle_data, "en808", 5) == 0) {
			oplus_comm_set_chg_cycle_status(chip, chip->chg_cycle_status & (~(int)CHG_CYCLE_VOTER__ENGINEER));
			oplus_comm_push_chg_cycle_info_msg(chip, "en808");
		} else if (chip->chg_cycle_status & CHG_CYCLE_VOTER__USER) {
			oplus_comm_set_chg_cycle_status(chip, chip->chg_cycle_status & (~(int)CHG_CYCLE_VOTER__USER));
			oplus_comm_push_chg_cycle_info_msg(chip, "user_enable");
		} else {
			chg_err("user_enable already true %d\n", chip->chg_cycle_status);
			return -EPERM;
		}
		chg_info("%s allow charging status=%d\n", proc_chg_cycle_data, chip->chg_cycle_status);
		if (chip->chg_cycle_status != CHG_CYCLE_VOTER__NONE) {
			chg_info("voter not allow charging\n");
			return -EPERM;
		}
		chg_info("allow charging.\n");
		vote(chip->chg_suspend_votable, DEBUG_VOTER, false, 0, false);
		vote(chip->chg_disable_votable, MMI_CHG_VOTER, false, 0, false);
		vote(chip->chg_disable_votable, TIMEOUT_VOTER, false, 0, false);
		chip->chging_over_time = false;
		oplus_comm_reset_chginfo(chip);
	} else if ((strncmp(proc_chg_cycle_data, "dis808", 6) == 0) ||
		    (strncmp(proc_chg_cycle_data, "user_disable", 12) == 0)) {
		if(chip->unwakelock_chg) {
			chg_err("unwakelock testing, this test not allowed\n");
			return -EPERM;
		}
		if (strncmp(proc_chg_cycle_data, "dis808", 5) == 0) {
			oplus_comm_set_chg_cycle_status(chip, chip->chg_cycle_status | (int)CHG_CYCLE_VOTER__ENGINEER);
			oplus_comm_push_chg_cycle_info_msg(chip, "dis808");
		} else if ((chip->chg_cycle_status & CHG_CYCLE_VOTER__USER) == 0) {
			oplus_comm_set_chg_cycle_status(chip, chip->chg_cycle_status | (int)CHG_CYCLE_VOTER__USER);
			oplus_comm_push_chg_cycle_info_msg(chip, "user_disable");
		} else {
			chg_err("user_disable already true %d\n", chip->chg_cycle_status);
			return -EPERM;
		}
		chg_info("%s not allow charging status=%d\n", proc_chg_cycle_data, chip->chg_cycle_status);

		chg_info("not allow charging.\n");
		vote(chip->chg_suspend_votable, DEBUG_VOTER, true, 1, false);
		vote(chip->chg_disable_votable, MMI_CHG_VOTER, true, 1, false);
		oplus_comm_reset_chginfo(chip);
	} else if (strncmp(proc_chg_cycle_data, "wakelock", 8) == 0) {
		chg_info("set wakelock.\n");
		oplus_comm_push_chg_cycle_info_msg(chip, "wakelock");
		vote(chip->chg_disable_votable, DEBUG_VOTER, false, 0, false);
		vote(chip->chg_disable_votable, TIMEOUT_VOTER, false, 0, false);
		chip->chging_over_time = false;
		oplus_comm_set_unwakelock(chip, false);
		oplus_comm_set_power_save(chip, false);
		oplus_comm_reset_chginfo(chip);
	} else if (strncmp(proc_chg_cycle_data, "unwakelock", 10) == 0) {
		chg_info("set unwakelock.\n");
		oplus_comm_push_chg_cycle_info_msg(chip, "unwakelock");
		vote(chip->chg_disable_votable, DEBUG_VOTER, true, 1, false);
		oplus_comm_set_unwakelock(chip, true);
		oplus_comm_set_power_save(chip, true);
		oplus_comm_reset_chginfo(chip);
	} else if (strncmp(proc_chg_cycle_data, "powersave", 9) == 0) {
		chg_info("powersave: stop usbtemp monitor, etc.\n");
		oplus_comm_push_chg_cycle_info_msg(chip, "powersave");
		oplus_comm_set_power_save(chip, true);
	} else if (strncmp(proc_chg_cycle_data, "unpowersave", 11) == 0) {
		chg_info("unpowersave: start usbtemp monitor, etc.\n");
		oplus_comm_push_chg_cycle_info_msg(chip, "unpowersave");
		oplus_comm_set_power_save(chip, false);
	} else {
		return -EFAULT;
	}

	return count;
}

static ssize_t oplus_comm_chg_cycle_read(struct file *file, char __user *buff, size_t count, loff_t *ppos)
{
	char page[256] = {0};
	char read_data[32] = {0};
	int len = 0;
	unsigned int notify_code = 0;
	struct oplus_chg_comm *chip = pde_data(file_inode(file));
	union mms_msg_data data = { 0 };

	if (chip) {
		oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_NOTIFY_CODE,
					&data, true);
		notify_code = data.intval;
	} else {
		chg_err("chg_cycle_read notify_code error.\n");
		return -EFAULT;
	}

	if (notify_code & (1 << NOTIFY_GAUGE_I2C_ERR))
		strcpy(read_data, "FAIL");
	else
		strcpy(read_data, "PASS");

	len = sprintf(page, "%s", read_data);
	if (len > *ppos)
		len -= *ppos;
	else
		len = 0;

	if (copy_to_user(buff, page, (len < count ? len : count))) {
		chg_err("chg_cycle_read error.\n");
		return -EFAULT;
	}
	*ppos += len < count ? len : count;
	return (len < count ? len : count);
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations chg_cycle_proc_fops = {
	.write = oplus_comm_chg_cycle_write,
	.read = oplus_comm_chg_cycle_read,
	.llseek = noop_llseek,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops chg_cycle_proc_fops = {
	.proc_write = oplus_comm_chg_cycle_write,
	.proc_read = oplus_comm_chg_cycle_read,
	.proc_lseek = noop_llseek,
};
#endif

#define RESERVE_SOC_BUF_LEN 4
static ssize_t proc_reserve_soc_debug_write(struct file *file, const char __user *user_buf, size_t len, loff_t *data)
{
	struct oplus_chg_comm *chip = pde_data(file_inode(file));
	struct oplus_comm_config *config = &chip->config;
	char buf[RESERVE_SOC_BUF_LEN] = { 0 };
	int reserve_chg_soc, reserve_dis_soc;

	if (len < 1 || len > RESERVE_SOC_BUF_LEN) {
		chg_err("len %lu invalid\n", len);
		return -EFAULT;
	}

	if (copy_from_user(buf, user_buf, len))
		return -EFAULT;

	buf[RESERVE_SOC_BUF_LEN - 1] = '\0';

	if (sscanf(buf, "%d,%d", &reserve_chg_soc, &reserve_dis_soc) == 2) {
		if (reserve_chg_soc != config->reserve_soc) {
			chg_info("chg:%d dis:%d\n", reserve_chg_soc, reserve_dis_soc);
			if (reserve_chg_soc < RESERVE_SOC_MIN || reserve_chg_soc > RESERVE_SOC_MAX) {
				config->reserve_soc = RESERVE_SOC_OFF;
				if (!chip->smooth_strategy) {
					config->smooth_switch = false;
					oplus_comm_set_smooth_soc(chip, chip->soc);
				}
			} else {
				config->reserve_soc = reserve_chg_soc;
				if (!chip->smooth_strategy) {
					config->smooth_switch = true;
					oplus_comm_smooth_to_soc(chip, true);
				}
			}
		}
	}
	return len;
}

static ssize_t proc_reserve_soc_debug_read(struct file *file, char __user *user_buf, size_t count, loff_t *off)
{
	struct oplus_chg_comm *chip = pde_data(file_inode(file));
	struct oplus_comm_config *config = &chip->config;
	char buf[256] = { 0 };
	int len = 0;

	if (chip->smooth_strategy)
		len = sprintf(buf, "%d,0", chip->smooth_strategy_chg_reserve);
	else
		len = sprintf(buf, "%d,%d", config->reserve_soc, config->reserve_soc);
	if (len > *off)
		len -= *off;
	else
		len = 0;
	if (copy_to_user(user_buf, buf, (len < count ? len : count)))
		return -EFAULT;

	*off += len < count ? len : count;
	return (len < count ? len : count);
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations proc_reserve_soc_debug_ops = {
	.write = proc_reserve_soc_debug_write,
	.read = proc_reserve_soc_debug_read,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops proc_reserve_soc_debug_ops = {
	.proc_write = proc_reserve_soc_debug_write,
	.proc_read = proc_reserve_soc_debug_read,
	.proc_lseek = noop_llseek,
};
#endif

#define DEC_CV_CC_BUF_LEN 5
static ssize_t proc_dec_cv_cc_debug_write(struct file *file, const char __user *user_buf, size_t len, loff_t *data)
{
	struct oplus_chg_comm *chip = pde_data(file_inode(file));
	struct oplus_comm_spec_config *spec = &chip->spec;
	char buf[DEC_CV_CC_BUF_LEN] = { 0 };
	int dbg_cc = 0;

	if (!spec->dec_cv.dec_spec_support)
		return -EFAULT;

	if (len < 1 || len > DEC_CV_CC_BUF_LEN) {
		chg_err("len %lu invalid\n", len);
		return -EFAULT;
	}

	if (copy_from_user(buf, user_buf, len))
		return -EFAULT;

	buf[DEC_CV_CC_BUF_LEN - 1] = '\0';

	if (1 != sscanf(buf, "%d", &dbg_cc)) {
		chg_err("invalid buf: '%s', len = %zd\n", buf, len);
		return -EFAULT;
	}
	spec->dec_cv.dbg_soh = dbg_cc;

	return len;
}

static ssize_t proc_dec_cv_cc_debug_read(struct file *file, char __user *user_buf, size_t count, loff_t *off)
{
	struct oplus_chg_comm *chip = pde_data(file_inode(file));
	struct oplus_comm_spec_config *spec = &chip->spec;

	char buf[256] = { 0 };
	int len = 0;
	int fv_dec = 0, wired_ffc_dec = 0, wls_ffc_dec = 0, vct = 0;

	oplus_comm_get_dec_vol(chip->comm_topic, &fv_dec, &wired_ffc_dec, &wls_ffc_dec, &vct);


	len = sprintf(buf, "delta=%d, dbg_cc=%d, fv_dec=%d, vct=%d\n", chip->spec.dec_cv.dec_delta, spec->dec_cv.dbg_soh, fv_dec, vct);
	if (len > *off)
		len -= *off;
	else
		len = 0;
	if (copy_to_user(user_buf, buf, (len < count ? len : count)))
		return -EFAULT;

	*off += len < count ? len : count;

	return (len < count ? len : count);
}


#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations proc_dec_cv_cc_debug_ops = {
	.write = proc_dec_cv_cc_debug_write,
	.read = proc_dec_cv_cc_debug_read,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops proc_dec_cv_cc_debug_ops = {
	.proc_write = proc_dec_cv_cc_debug_write,
	.proc_read = proc_dec_cv_cc_debug_read,
	.proc_lseek = noop_llseek,
};
#endif

static int oplus_comm_init_proc(struct oplus_chg_comm *chip)
{
	struct proc_dir_entry *pr_entry_tmp;
	struct proc_dir_entry *pr_entry_da;
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	int eng_version = get_eng_version();
#endif

	pr_entry_tmp = proc_create_data("ui_soc_decimal", 0664, NULL,
					&ui_soc_decimal_ops, chip);
	if (pr_entry_tmp == NULL)
		chg_err("Couldn't create ui_soc_decimal proc entry\n");

	pr_entry_tmp = proc_create_data("charger_cycle", 0664, NULL,
					&chg_cycle_proc_fops, chip);
	if (pr_entry_tmp == NULL)
		chg_err("Couldn't create charger_cycle proc entry\n");

	pr_entry_tmp = proc_create_data("tbatt_pwroff", 0664, NULL,
					&tbatt_pwroff_proc_fops, chip);
	if (pr_entry_tmp == NULL)
		chg_err("Couldn't create tbatt_pwroff proc entry\n");

	pr_entry_tmp = proc_create_data("batt_param_noplug", 0664, NULL,
					&batt_param_noplug_proc_fops, chip);
	if (pr_entry_tmp == NULL)
		chg_err("Couldn't create batt_param_noplug proc entry\n");

	pr_entry_da = proc_mkdir("charger", NULL);
	if (pr_entry_da == NULL) {
		chg_err("Couldn't create charger proc entry\n");
		goto charger_fail;
	}

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if (eng_version != OEM_RELEASE) {
#endif
		pr_entry_tmp =
			proc_create_data("charger_factorymode_test",
					 0666, pr_entry_da,
					 &proc_charger_factorymode_test_ops,
					 chip);
		if (pr_entry_tmp == NULL)
			chg_err("Couldn't create charger_factorymode_test proc entry\n");
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	}
#endif

	pr_entry_tmp =
		proc_create_data("integrate_gauge_fcc_flag", 0664, pr_entry_da,
				 &proc_integrate_gauge_fcc_flag_ops, chip);
	if (pr_entry_tmp == NULL)
		chg_err("Couldn't create integrate_gauge_fcc_flag proc entry\n");

	pr_entry_tmp =
		proc_create_data("hmac", 0444, pr_entry_da,
				 &hmac_proc_fops, chip);
	if (pr_entry_tmp == NULL)
		chg_err("Couldn't create hmac proc entry\n");
	pr_entry_tmp =
		proc_create_data("reserve_soc_debug", 0644, pr_entry_da,
				 &proc_reserve_soc_debug_ops, chip);
	if (pr_entry_tmp == NULL)
		chg_err("Couldn't create proc_reserve_soc_debug_ops entry\n");

	pr_entry_tmp =
		proc_create_data("dec_cv_cc", 0644, pr_entry_da,
				 &proc_dec_cv_cc_debug_ops, chip);
	if (pr_entry_tmp == NULL)
		chg_err("Couldn't create proc_dec_cv_cc_debug_ops entry\n");

charger_fail:
	return 0;
}

static void oplus_led_on_off_handler_work(struct work_struct *work)
{
	struct oplus_chg_comm *chip = container_of(work, struct oplus_chg_comm, led_on_off_handler_work);
	struct mms_msg *msg;
	int rc;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, COMM_ITEM_LED_ON);
	if (unlikely(!msg)) {
		chg_err("alloc msg failed\n");
		return;
	}

	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish led on msg error, rc=%d\n", rc);
		kfree(msg);
	}

	chg_info("set led_on=%d\n", chip->led_on);
}

__maybe_unused static int oplus_comm_set_led_on(struct oplus_chg_comm *chip, bool on)

{
	if (unlikely(!chip || !chip->comm_topic)) {
		chg_err("invalid chip or topic pointer\n");
		return -EINVAL;
	}

	if (chip->led_on == on) {
		chg_debug("led already %s\n", on ? "on" : "off");
		return 0;
	}

	chip->led_on = on;
	schedule_work(&chip->led_on_off_handler_work);

	chg_info("set led_on=%d wait result\n", chip->led_on);
	return 0;
}

#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_OPLUS_CHG_DRM_PANEL_NOTIFY)
static void oplus_comm_panel_notifier_callback(enum panel_event_notifier_tag tag,
		 struct panel_event_notification *notification, void *client_data)
{
	struct oplus_chg_comm *chip = client_data;

	if (!notification) {
		chg_err("Invalid notification\n");
		return;
	}

	chg_debug("Notification type:%d, early_trigger:%d",
			notification->notif_type,
			notification->notif_data.early_trigger);
	switch (notification->notif_type) {
	case DRM_PANEL_EVENT_UNBLANK:
		chg_info("received unblank event: %d\n", notification->notif_data.early_trigger);
		if (notification->notif_data.early_trigger)
			oplus_comm_set_led_on(chip, true);
		break;
	case DRM_PANEL_EVENT_BLANK:
		chg_info("received blank event: %d\n", notification->notif_data.early_trigger);
		if (notification->notif_data.early_trigger)
			oplus_comm_set_led_on(chip, false);
		break;
	case DRM_PANEL_EVENT_BLANK_LP:
		chg_info("received blank_lp event: %d\n", notification->notif_data.early_trigger);
		if (notification->notif_data.early_trigger)
			oplus_comm_set_led_on(chip, false);
		break;
	case DRM_PANEL_EVENT_FPS_CHANGE:
		break;
	default:
		break;
	}
}

#else /* CONFIG_DRM_PANEL_NOTIFY */

#if IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)
static int fb_notifier_callback(struct notifier_block *nb,
		unsigned long event, void *data)
{
	int blank;
	struct msm_drm_notifier *evdata = data;
	struct oplus_chg_comm *chip =
		container_of(nb, struct oplus_chg_comm, chg_fb_notify);

	if (!evdata || (evdata->id != 0)){
		return 0;
	}

	if (event == MSM_DRM_EARLY_EVENT_BLANK) {
		blank = *(int *)(evdata->data);
		if (blank == MSM_DRM_BLANK_UNBLANK)
			oplus_comm_set_led_on(chip, true);
		else if (blank == MSM_DRM_BLANK_POWERDOWN)
			oplus_comm_set_led_on(chip, false);
		else
			chg_err("receives wrong data EARLY_BLANK:%d\n", blank);
	}
	return 0;
}
#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY_CHG)
static int chg_mtk_drm_notifier_callback(struct notifier_block *nb,
			unsigned long event, void *data)
{
	int *blank = (int *)data;
	struct oplus_chg_comm *chip =
		container_of(nb, struct oplus_chg_comm, chg_fb_notify);

	if (!blank) {
		chg_err("get disp statu err, blank is NULL!");
		return 0;
	}

	if (!chip) {
		return 0;
	}
	chg_info("mtk gki notifier event:%lu, blank:%d", event, *blank);
	switch (event) {
	case MTK_DISP_EARLY_EVENT_BLANK:
		if (*blank == MTK_DISP_BLANK_UNBLANK)
			oplus_comm_set_led_on(chip, true);
		else if (*blank == MTK_DISP_BLANK_POWERDOWN)
			oplus_comm_set_led_on(chip, false);
		else
			chg_err("receives wrong data EARLY_BLANK:%d\n", *blank);
		break;
	case MTK_DISP_EVENT_BLANK:
		if (*blank == MTK_DISP_BLANK_UNBLANK)
			oplus_comm_set_led_on(chip, true);
		else if (*blank == MTK_DISP_BLANK_POWERDOWN)
			oplus_comm_set_led_on(chip, false);
		else
			chg_err("receives wrong data BLANK:%d\n", *blank);
		break;
	default:
		break;
	}
	return 0;
}
#else
static int fb_notifier_callback(struct notifier_block *nb,
		unsigned long event, void *data)
{
	int blank;
	struct fb_event *evdata = data;
	struct oplus_chg_comm *chip =
		container_of(nb, struct oplus_chg_comm, chg_fb_notify);

	if (evdata && evdata->data) {
		if (event == FB_EVENT_BLANK) {
			blank = *(int *)evdata->data;
			if (blank == FB_BLANK_UNBLANK)
				oplus_comm_set_led_on(chip, true);
			else if (blank == FB_BLANK_POWERDOWN)
				oplus_comm_set_led_on(chip, false);
		}
	}
	return 0;
}
#endif /* CONFIG_DRM_MSM */

#endif /* CONFIG_DRM_PANEL_NOTIFY */

static int oplus_comm_register_lcd_notify(struct oplus_chg_comm *chip)
{
	int rc = 0;

#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_OPLUS_CHG_DRM_PANEL_NOTIFY)
	int i;
	int count;
	struct device_node *node = NULL;
	struct drm_panel *panel = NULL;
	struct device_node *np = oplus_get_node_by_type(chip->dev->of_node);
	void *cookie = NULL;

#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY)
	count = of_count_phandle_with_args(np, "oplus,display_panel", NULL);
	if (count <= 0) {
		chg_err("oplus,display_panel not found\n");
		return 0;
	}

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, "oplus,display_panel", i);
		panel = of_drm_find_panel(node);
		of_node_put(node);
		if (!IS_ERR(panel)) {
			chip->active_panel = panel;
			rc = 0;
			chg_info("find active panel\n");
			break;
		} else {
			rc = PTR_ERR(panel);
		}
	}
#else
	np = of_find_node_by_name(NULL, "oplus,dsi-display-dev");
	if (!np) {
		chg_info("device tree info. missing\n");
		return 0;
	}

	count = of_count_phandle_with_args(np, "oplus,dsi-panel-primary", NULL);
	if (count <= 0) {
		chg_info("primary panel no found\n");
		return 0;
	}

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, "oplus,dsi-panel-primary", i);
		panel = of_drm_find_panel(node);
		of_node_put(node);
		if (!IS_ERR(panel)) {
			chip->active_panel = panel;
			rc = 0;
			chg_info("find active_panel rc\n");
			break;
		} else {
			rc = PTR_ERR(panel);
		}
	}
	if (chip->config.dual_panel_support) {
		count = of_count_phandle_with_args(np, "oplus,dsi-panel-secondary", NULL);
		if (count <= 0) {
			chg_info("secondary panel no found\n");
			return 0;
		}

		for (i = 0; i < count; i++) {
			node = of_parse_phandle(np, "oplus,dsi-panel-secondary", i);
			panel = of_drm_find_panel(node);
			of_node_put(node);
			if (!IS_ERR(panel)) {
				chip->active_panel_sec = panel;
				rc = 0;
				chg_info("find active_panel_sec rc\n");
				break;
			} else {
				rc = PTR_ERR(panel);
			}
		}
	}
#endif/* CONFIG_DRM_PANEL_NOTIFY */

	if (chip->active_panel) {
		cookie = panel_event_notifier_register(
			PANEL_EVENT_NOTIFICATION_PRIMARY,
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
			PANEL_EVENT_NOTIFIER_CLIENT_PRIMARY_CHG,
#else
			PANEL_EVENT_NOTIFIER_CLIENT_BATTERY_CHARGER,
#endif
			chip->active_panel, &oplus_comm_panel_notifier_callback,
			chip);
		if (!cookie) {
			chg_err("Unable to register chg_panel_notifier\n");
			return -EINVAL;
		} else {
			chg_info("success register chg_panel_notifier\n");
			chip->notifier_cookie = cookie;
		}
	} else {
		chg_info("can't find active panel, rc=%d\n", rc);
		if (rc == -EPROBE_DEFER)
			return rc;
		else
			return -ENODEV;
	}
	if (chip->config.dual_panel_support) {
		if (chip->active_panel_sec) {
			cookie = panel_event_notifier_register(
				PANEL_EVENT_NOTIFICATION_SECONDARY,
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
				PANEL_EVENT_NOTIFIER_CLIENT_SECONDARY_CHG,
#else
				PANEL_EVENT_NOTIFIER_CLIENT_BATTERY_CHARGER,
#endif
				chip->active_panel_sec, &oplus_comm_panel_notifier_callback,
				chip);
			if (!cookie) {
				chg_err("Unable to register sec chg_panel_notifier\n");
				return -EINVAL;
			} else {
				chg_info("success register sec chg_panel_notifier\n");
				chip->notifier_cookie_sec = cookie;
			}
		} else {
			chg_info("can't find sec active panel, rc=%d\n", rc);
			if (rc == -EPROBE_DEFER)
				return rc;
			else
				return -ENODEV;
		}
	}

#else /* CONFIG_DRM_PANEL_NOTIFY */

#if IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)
	chip->chg_fb_notify.notifier_call = fb_notifier_callback;
	rc = msm_drm_register_client(&chip->chg_fb_notify);
#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY_CHG)
	chip->chg_fb_notify.notifier_call = chg_mtk_drm_notifier_callback;
	if (mtk_disp_notifier_register("Oplus_Chg_v2", &chip->chg_fb_notify)) {
		chg_err("Failed to register disp notifier client!!\n");
	}
#else
	chip->chg_fb_notify.notifier_call = fb_notifier_callback;
	rc = fb_register_client(&chip->chg_fb_notify);
#endif /*CONFIG_DRM_MSM*/
	if (rc)
		chg_err("Unable to register chg_fb_notify, rc=%d\n", rc);

#endif /* CONFIG_DRM_PANEL_NOTIFY */

	return rc;
}

#define LCD_REG_RETRY_COUNT_MAX		100
#define LCD_REG_RETRY_DELAY_MS		100
static void oplus_comm_lcd_notify_reg_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_comm *chip =
		container_of(dwork, struct oplus_chg_comm, lcd_notify_reg_work);
	static int retry_count;
	int rc;

	if (retry_count >= LCD_REG_RETRY_COUNT_MAX)
		return;

	rc = oplus_comm_register_lcd_notify(chip);
	if (rc < 0) {
		if (rc != -EPROBE_DEFER) {
			chg_err("register lcd notify error, rc=%d\n", rc);
			return;
		}
		retry_count++;
		chg_info("lcd panel not ready, count=%d\n", retry_count);
		schedule_delayed_work(&chip->lcd_notify_reg_work,
				      msecs_to_jiffies(LCD_REG_RETRY_DELAY_MS));
		return;
	}
	retry_count = 0;
	chip->lcd_notify_reg = true;
}

static void oplus_fg_soft_reset_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_comm *chip =
		container_of(dwork, struct oplus_chg_comm, fg_soft_reset_work);

	if (chip->soc < SOFT_REST_SOC_THRESHOLD ||
	    oplus_gauge_check_reset_condition() ||
	    fg_reset_test) {
		if (abs(chip->ibat_ma) < SOFT_REST_CHECK_DISCHG_MAX_CUR)
			chip->fg_check_ibat_cnt++;
		else
			chip->fg_check_ibat_cnt = 0;

		if (chip->fg_check_ibat_cnt < SOFT_REST_RETRY_MAX_CNT + 1)
			return;

		if(oplus_gauge_reset()) {
			chip->fg_soft_reset_done = true;
			chip->fg_soft_reset_fail_cnt = 0;
		} else {
			chip->fg_soft_reset_done = false;
			chip->fg_soft_reset_fail_cnt++;
		}
		chip->fg_check_ibat_cnt = 0;
	} else {
		chip->fg_check_ibat_cnt = 0;
	}

}

static void oplus_wired_chg_check_work(struct work_struct *work)
{
	struct oplus_chg_comm *chip =
		container_of(work, struct oplus_chg_comm, wired_chg_check_work);

	if (is_wired_charging_disable_votable_available(chip)) {
		vote(chip->wired_charging_disable_votable,
		     CHG_FULL_VOTER, false, 0, false);
		vote(chip->wired_charging_disable_votable, FFC_VOTER,
		     false, 0, false);
	}
	chg_info("charging_disable [%s]\n",
		chip->charging_disable == true ?"true":"false");
	oplus_comm_check_battery_status(chip);
}

static void oplus_comm_parse_region_id_list(struct oplus_chg_comm *chip)
{
	int rc = 0;
	struct mms_msg *msg;
	chip->nvid_support_flags = oplus_chg_get_nvid_support_flags();

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				COMM_ITEM_NVID_SUPPORT_FLAGS);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish nvid_support_flags msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
#include "config/dynamic_cfg/oplus_comm_cfg.h"
#endif

static void oplus_set_flash_mode(struct oplus_chg_comm *chip, bool flash_mode)
{
	struct mms_msg *msg;
	int rc;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, COMM_ITEM_FLASH_MODE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg_sync(chip->comm_topic, msg);
	if (rc < 0) {
		chg_err("publish flash mode msg error, rc=%d\n", rc);
		kfree(msg);
		return;
	}
}

static void oplus_comm_flash_mode_boost_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_comm *chip = container_of(dwork, struct oplus_chg_comm, flash_mode_boost_work);

	chg_info("start boost\n");
	oplus_set_flash_mode(chip, false);
}

static void oplus_comm_flash_mode_enable_chg(struct oplus_chg_comm *chip)
{
	if (!chip) {
		chg_err("invalid chip\n");
		return;
	}

	if (is_wired_icl_votable_available(chip))
		rerun_election(chip->wired_icl_votable, true);
	if (is_wired_charging_disable_votable_available(chip))
		vote(chip->wired_charging_disable_votable, FLASH_MODE_VOTER, false, 0, false);
}

static void oplus_comm_offline_clean_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_comm *chip = container_of(dwork, struct oplus_chg_comm, offline_clean_work);

	if (!chip->retention_state)
		oplus_comm_offline_clean_process(chip);
}

void oplus_chg_set_camera_on(bool val)
{
	int count = FLASH_MODE_SAFETY_VOLTAGE_DETECT_COUNT;
	int vbus_volt = 0;
	struct oplus_chg_comm *chip = g_comm_dev;

	if (!chip || !chip->wired_topic)
		return;

	chg_info("set flash mode to %s\n", val ? "true" : "false");

	chip->flash_mode = val;
	if (val) {
		oplus_set_flash_mode(chip, val);
		cancel_delayed_work_sync(&chip->flash_mode_boost_work);
		vbus_volt = oplus_wired_get_vbus();
		chg_info("vbus_volt=%dmv\n", vbus_volt);
		while (vbus_volt > FLASH_MODE_SAFETY_VOLTAGE && count > 0) {
			msleep(FLASH_MODE_SAFETY_VOLTAGE_QUERY_INTERVAL);
			vbus_volt = oplus_wired_get_vbus();
			chg_info("vbus_volt=%dmv\n", vbus_volt);
			count--;
		}
		if (is_flash_mode_votable_available(chip))
			vote(chip->flash_mode_votable, FLASH_MODE_VOTER, true, 1, false);
	} else {
		if (is_flash_mode_votable_available(chip))
			vote(chip->flash_mode_votable, FLASH_MODE_VOTER, false, 0, false);
		schedule_delayed_work(&chip->flash_mode_boost_work, msecs_to_jiffies(FLASH_MODE_DELAY));
		oplus_comm_flash_mode_enable_chg(chip);
	}
	return;
}
EXPORT_SYMBOL(oplus_chg_set_camera_on);

static int oplus_comm_driver_probe(struct platform_device *pdev)
{
	struct oplus_chg_comm *comm_dev;
	int rc;

	comm_dev = devm_kzalloc(&pdev->dev, sizeof(struct oplus_chg_comm),
				GFP_KERNEL);
	if (comm_dev == NULL) {
		chg_err("alloc memory error\n");
		return -ENOMEM;
	}

	g_comm_dev = comm_dev;
	comm_dev->dev = &pdev->dev;
	platform_set_drvdata(pdev, comm_dev);

	rc = oplus_comm_parse_dt(comm_dev);
	if (rc < 0) {
		chg_err("oplus chg comm parse dts error, rc=%d\n", rc);
		goto parse_dt_err;
	}

	oplus_comm_temp_thr_init(comm_dev);
	comm_dev->temp_region = TEMP_REGION_NORMAL;
	oplus_comm_temp_thr_update(comm_dev, TEMP_REGION_NORMAL, TEMP_REGION_NORMAL);
	comm_dev->slow_chg_param = 0;
	comm_dev->rechg_soc = 100;
	comm_dev->rechg_soc_en = false;
	comm_dev->uisoc_down_in_full = false;
	comm_dev->rechg_now = false;
	comm_dev->ui_soc = 50; /* fix the issue of power off by ui_soc is 0 */
	mutex_init(&comm_dev->slow_chg_mutex);
	mutex_init(&comm_dev->sale_mode_mutex);
	mutex_init(&comm_dev->dec_data_lock);

	comm_dev->low_temp_check_jiffies = jiffies;

	rc = oplus_comm_vote_init(comm_dev);
	if (rc < 0)
		goto vote_init_err;
	rc = oplus_comm_topic_init(comm_dev);
	if (rc < 0)
		goto topic_reg_err;
	rc = oplus_comm_tbatt_power_off_task_init(comm_dev);
	if (rc < 0)
		goto task_init_err;

	rc = oplus_comm_dec_cv_down_info_obtain_init(comm_dev);
	if (rc < 0)
		goto dec_cv_down_info_obtain_nb_reg_err;

	rc = oplus_comm_dec_cv_down_info_update_init(comm_dev);
	if (rc < 0)
		goto dec_cv_down_info_update_nb_reg_err;

	rc = oplus_comm_init_proc(comm_dev);
	if (rc < 0)
		goto proc_init_err;

	INIT_DELAYED_WORK(&comm_dev->get_reserve_dec_cv_down_info_work,
		oplus_comm_get_reserve_dec_cv_down_info_work);
	INIT_WORK(&comm_dev->set_reserve_dec_cv_down_info_work, oplus_comm_set_reserve_dec_cv_down_info_work);
	INIT_DELAYED_WORK(&comm_dev->dec_cv_check_work, oplus_comm_dec_cv_check_work);
	INIT_WORK(&comm_dev->plugin_work, oplus_comm_plugin_work);
	INIT_WORK(&comm_dev->chg_type_change_work,
		  oplus_comm_chg_type_change_work);
	INIT_WORK(&comm_dev->chg_power_role_change_work, oplus_comm_chg_power_role_change_work);
	INIT_WORK(&comm_dev->gauge_check_work, oplus_comm_gauge_check_work);
	INIT_WORK(&comm_dev->gauge_remuse_work, oplus_comm_gauge_remuse_work);
	INIT_WORK(&comm_dev->noplug_batt_volt_work, oplus_comm_noplug_batt_volt_work);
	INIT_WORK(&comm_dev->wired_chg_check_work, oplus_wired_chg_check_work);
	INIT_WORK(&comm_dev->led_on_off_handler_work, oplus_led_on_off_handler_work);
	INIT_WORK(&comm_dev->offline_delayed_process_work, oplus_comm_offline_delayed_process_work);

	INIT_DELAYED_WORK(&comm_dev->ffc_start_work, oplus_comm_ffc_start_work);
	INIT_DELAYED_WORK(&comm_dev->charge_timeout_work, oplus_comm_charge_timeout_work);
	INIT_DELAYED_WORK(&comm_dev->ui_soc_update_work, oplus_comm_ui_soc_update_work);
	INIT_DELAYED_WORK(&comm_dev->ui_soc_decimal_work, oplus_comm_show_ui_soc_decimal);
	INIT_DELAYED_WORK(&comm_dev->lcd_notify_reg_work, oplus_comm_lcd_notify_reg_work);
	INIT_DELAYED_WORK(&comm_dev->fg_soft_reset_work, oplus_fg_soft_reset_work);
	INIT_DELAYED_WORK(&comm_dev->dec_vol_info_trigger_work, oplus_chg_track_dec_vol_info_trigger_work);
	INIT_DELAYED_WORK(&comm_dev->gauge_r_info_work, oplus_chg_track_gauge_r_info_trigger_work);
	INIT_DELAYED_WORK(&comm_dev->offline_clean_work, oplus_comm_offline_clean_work);
	INIT_DELAYED_WORK(&comm_dev->flash_mode_boost_work, oplus_comm_flash_mode_boost_work);
	INIT_DELAYED_WORK(&comm_dev->iterm_check_work, oplus_comm_iterm_check_work);
	INIT_DELAYED_WORK(&comm_dev->passed_q_save_work, oplus_comm_passed_q_save_work);
#ifdef CONFIG_OPLUS_CHARGER_MTK
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
	INIT_DELAYED_WORK(&comm_dev->power_off_check_work, oplus_chg_power_off_check_work);
#endif
#endif

	spin_lock_init(&comm_dev->remuse_lock);
	mutex_init(&comm_dev->decimal_lock);

	comm_dev->batt_temp = GAUGE_INVALID_TEMP;
	oplus_comm_subscribe_comm_topic(comm_dev);
	oplus_mms_wait_topic("wired", oplus_comm_subscribe_wired_topic, comm_dev);
	oplus_mms_wait_topic("vooc", oplus_comm_subscribe_vooc_topic, comm_dev);
	oplus_mms_wait_topic("wireless", oplus_comm_subscribe_wls_topic, comm_dev);
	oplus_mms_wait_topic("ufcs", oplus_comm_subscribe_ufcs_topic, comm_dev);
	oplus_mms_wait_topic("pps", oplus_comm_subscribe_pps_topic, comm_dev);
	oplus_mms_wait_topic("retention", oplus_comm_subscribe_retention_topic, comm_dev);
	oplus_mms_wait_topic("plc", oplus_comm_subscribe_plc_topic, comm_dev);
	oplus_mms_wait_topic("reverse", oplus_comm_subscribe_reverse_topic, comm_dev);
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	oplus_mms_wait_topic("state_keep", oplus_comm_subscribe_keep_topic, comm_dev);
#endif

#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_OPLUS_CHG_DRM_PANEL_NOTIFY)
	oplus_comm_set_led_on(comm_dev, true);
#endif

	schedule_delayed_work(&comm_dev->lcd_notify_reg_work, 0);

#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
	(void)oplus_comm_reg_debug_config(comm_dev);
#endif

	comm_dev->nvid_support_flags = 0x00;
	oplus_comm_parse_region_id_list(comm_dev);
	chg_info("probe done\n");

	return 0;

proc_init_err:
	oplus_chg_unreg_mutual_notifier(&comm_dev->dec_cv_down_update_nb);
dec_cv_down_info_update_nb_reg_err:
	oplus_chg_unreg_mutual_notifier(&comm_dev->dec_cv_down_obtain_nb);
dec_cv_down_info_obtain_nb_reg_err:
	kthread_stop(comm_dev->tbatt_pwroff_task);
task_init_err:
topic_reg_err:
	destroy_votable(comm_dev->cool_down_votable);
	destroy_votable(comm_dev->chg_suspend_votable);
	destroy_votable(comm_dev->chg_disable_votable);
	destroy_votable(comm_dev->fv_min_votable);
	destroy_votable(comm_dev->fv_max_votable);
vote_init_err:
parse_dt_err:
	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, comm_dev);
	g_comm_dev = NULL;
	return rc;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void oplus_comm_driver_remove(struct platform_device *pdev)
#else
static int oplus_comm_driver_remove(struct platform_device *pdev)
#endif
{
	struct oplus_chg_comm *comm_dev = platform_get_drvdata(pdev);

#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
	oplus_comm_unreg_debug_config(comm_dev);
#endif

	if (!IS_ERR_OR_NULL(comm_dev->wired_subs))
		oplus_mms_unsubscribe(comm_dev->wired_subs);
	if (!IS_ERR_OR_NULL(comm_dev->gauge_subs))
		oplus_mms_unsubscribe(comm_dev->gauge_subs);
	if (!IS_ERR_OR_NULL(comm_dev->vooc_subs))
		oplus_mms_unsubscribe(comm_dev->vooc_subs);
	if (!IS_ERR_OR_NULL(comm_dev->ufcs_subs))
		oplus_mms_unsubscribe(comm_dev->ufcs_subs);
	if (!IS_ERR_OR_NULL(comm_dev->wls_subs))
		oplus_mms_unsubscribe(comm_dev->wls_subs);
	if (!IS_ERR_OR_NULL(comm_dev->comm_subs))
		oplus_mms_unsubscribe(comm_dev->comm_subs);
	if (!IS_ERR_OR_NULL(comm_dev->retention_subs))
		oplus_mms_unsubscribe(comm_dev->retention_subs);
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	if (!IS_ERR_OR_NULL(comm_dev->keep_subs))
		oplus_mms_unsubscribe(comm_dev->keep_subs);
#endif

	if (comm_dev->lcd_notify_reg) {
#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_OPLUS_CHG_DRM_PANEL_NOTIFY)
		if (comm_dev->active_panel && comm_dev->notifier_cookie)
			panel_event_notifier_unregister(
				comm_dev->notifier_cookie);
		if (comm_dev->active_panel_sec && comm_dev->notifier_cookie_sec)
			panel_event_notifier_unregister(
				comm_dev->notifier_cookie_sec);
#else /* CONFIG_DRM_PANEL_NOTIFY */
#if IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)
		msm_drm_unregister_client(&comm_dev->chg_fb_notify);
#else
		fb_unregister_client(&comm_dev->chg_fb_notify);
#endif /* CONFIG_DRM_MSM */
#endif /* CONFIG_DRM_PANEL_NOTIFY */
	}

	remove_proc_entry("charger", NULL);
	remove_proc_entry("tbatt_pwroff", NULL);
	remove_proc_entry("charger_cycle", NULL);
	remove_proc_entry("ui_soc_decimal", NULL);

	kthread_stop(comm_dev->tbatt_pwroff_task);

	destroy_votable(comm_dev->cool_down_votable);
	destroy_votable(comm_dev->chg_suspend_votable);
	destroy_votable(comm_dev->chg_disable_votable);
	destroy_votable(comm_dev->fv_min_votable);
	destroy_votable(comm_dev->fv_max_votable);

	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, comm_dev);
	g_comm_dev = NULL;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return 0;
#endif
}

static unsigned long suspend_tm_sec = 0;
static int oplus_comm_get_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc = NULL;
	int rc = 0;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		chg_err("unable to open rtc device (%s)\n",
			CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		chg_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		chg_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}
	rtc_tm_to_time(&tm, now_tm_sec);

close_time:
	rtc_class_close(rtc);
	return rc;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
static int oplus_comm_pm_resume(struct device *dev)
{
	struct oplus_chg_comm *chip = dev_get_drvdata(dev);
	unsigned long resume_tm_sec = 0;
	unsigned long sleep_time = 0;
	int rc = 0;

	rc = oplus_comm_get_current_time(&resume_tm_sec);
	if (rc || suspend_tm_sec == -1) {
		chg_err("RTC read failed\n");
		sleep_time = 0;
	} else {
		sleep_time = resume_tm_sec - suspend_tm_sec;
	}

	chip->sleep_tm_sec = sleep_time;
	chg_info("resume, sleep_time=%lu\n", sleep_time);

	spin_lock(&chip->remuse_lock);
	chip->comm_remuse = true;
	if (chip->gauge_remuse) {
		spin_unlock(&chip->remuse_lock);
		schedule_work(&chip->gauge_remuse_work);
	} else {
		spin_unlock(&chip->remuse_lock);
	}

	return 0;
}

static int oplus_comm_pm_suspend(struct device *dev)
{
	struct oplus_chg_comm *chip = dev_get_drvdata(dev);

	if (oplus_comm_get_current_time(&suspend_tm_sec)) {
		chg_err("RTC read failed\n");
		suspend_tm_sec = -1;
	}
	spin_lock(&chip->remuse_lock);
	chip->comm_remuse = false;
	chip->gauge_remuse = false;
	spin_unlock(&chip->remuse_lock);
	return 0;
}

static const struct dev_pm_ops oplus_comm_pm_ops = {
	.resume		= oplus_comm_pm_resume,
	.suspend	= oplus_comm_pm_suspend,
};
#endif

static const struct of_device_id oplus_chg_comm_match[] = {
	{ .compatible = "oplus,common-charge" },
	{},
};

static struct platform_driver oplus_chg_comm_driver = {
	.driver = {
		.name = "oplus_chg_comm",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(oplus_chg_comm_match),
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
		.pm 	= &oplus_comm_pm_ops,
#endif
	},
	.probe = oplus_comm_driver_probe,
	.remove = oplus_comm_driver_remove,
};

static __init int oplus_comm_driver_init(void)
{
	return platform_driver_register(&oplus_chg_comm_driver);
}

static __exit void oplus_comm_driver_exit(void)
{
	platform_driver_unregister(&oplus_chg_comm_driver);
}

oplus_chg_module_register(oplus_comm_driver);

int oplus_comm_get_wired_aging_ffc_offset(struct oplus_mms *topic, int step)
{
	struct oplus_chg_comm *chip;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -ENODEV;
	}
	if (step >= FFC_CHG_STEP_MAX) {
		chg_err("step(=%d) is too big\n", step);
		return -EINVAL;
	} else if (step < 0) {
		chg_err("step(=%d) is too small\n", step);
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(topic);
	return oplus_comm_get_aging_ffc_offset(chip, step);
}

int oplus_comm_get_current_wired_ffc_cutoff_fv(struct oplus_mms *topic, int step)
{
	struct oplus_chg_comm *chip;
	struct oplus_comm_spec_config *spec;
	enum oplus_ffc_temp_region ffc_temp_region;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -ENODEV;
	}
	if (step >= FFC_CHG_STEP_MAX) {
		chg_err("step(=%d) is too big\n", step);
		return -EINVAL;
	} else if (step < 0) {
		chg_err("step(=%d) is too small\n", step);
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(topic);
	spec = &chip->spec;

	ffc_temp_region = oplus_comm_get_ffc_temp_region(chip);
	if (ffc_temp_region < FFC_TEMP_REGION_PRE_NORMAL || ffc_temp_region > FFC_TEMP_REGION_NORMAL) {
		chg_err("FFC charging is not possible in this temp region, temp_region=%s\n",
			oplus_comm_get_ffc_temp_region_str(ffc_temp_region));
		return -EINVAL;
	}
	return spec->wired_ffc_fv_cutoff_mv[step][ffc_temp_region - 1] - spec->dec_cv.dec_vol;
}

int oplus_comm_get_wired_ffc_cutoff_fv(struct oplus_mms *topic, int step,
	enum oplus_ffc_temp_region temp_region)
{
	struct oplus_chg_comm *chip;
	struct oplus_comm_spec_config *spec;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -ENODEV;
	}
	if (step >= FFC_CHG_STEP_MAX) {
		chg_err("step(=%d) is too big\n", step);
		return -EINVAL;
	} else if (step < 0) {
		chg_err("step(=%d) is too small\n", step);
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(topic);
	spec = &chip->spec;

	if (temp_region < FFC_TEMP_REGION_PRE_NORMAL || temp_region > FFC_TEMP_REGION_NORMAL) {
		chg_err("FFC charging is not possible in this temp region, temp_region=%s\n",
			oplus_comm_get_ffc_temp_region_str(temp_region));
		return -EINVAL;
	}

	return spec->wired_ffc_fv_cutoff_mv[step][temp_region - 1] - spec->dec_cv.dec_vol;
}

int oplus_comm_get_wired_ffc_step_max(struct oplus_mms *topic)
{
	struct oplus_chg_comm *chip;
	struct oplus_comm_spec_config *spec;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -ENODEV;
	}
	chip = oplus_mms_get_drvdata(topic);
	spec = &chip->spec;

	return spec->wired_ffc_step_max;
}

int oplus_comm_get_wired_aging_ffc_version(struct oplus_mms *topic)
{
	struct oplus_chg_comm *chip;
	struct oplus_comm_spec_config *spec;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -ENODEV;
	}
	chip = oplus_mms_get_drvdata(topic);
	spec = &chip->spec;

	return spec->wired_aging_ffc_version;
}

int oplus_comm_get_removed_bat_decidegc(struct oplus_mms *topic)
{
	struct oplus_chg_comm *chip;
	struct oplus_comm_spec_config *spec;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -ENODEV;
	}
	chip = oplus_mms_get_drvdata(topic);
	spec = &chip->spec;

	return spec->removed_bat_decidegc;
}
