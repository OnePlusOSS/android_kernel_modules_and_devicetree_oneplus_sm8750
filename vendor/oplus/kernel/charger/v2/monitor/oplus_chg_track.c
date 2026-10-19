#define pr_fmt(fmt) "[TRACK]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of_platform.h>
#include <linux/iio/consumer.h>
#include <linux/power_supply.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/rtc.h>
#include <linux/ktime.h>
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/kthread.h>
#include <linux/sched/clock.h>
#include <linux/string.h>
#include <linux/kfifo.h>
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/system/boot_mode.h>
#endif
#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) ||	\
	defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE)
#include <soc/oplus/system/kernel_fb.h>
#elif defined(CONFIG_OPLUS_KEVENT_UPLOAD)
#include <linux/oplus_kevent.h>
#endif
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/system/oplus_project.h>
#endif

#include <oplus_chg.h>
#include <oplus_chg_module.h>
#include <oplus_chg_voter.h>
#include <oplus_mms_gauge.h>
#include <oplus_chg_vooc.h>
#include <oplus_chg_ufcs.h>
#include <oplus_chg_comm.h>
#include <oplus_chg_exception.h>
#include <oplus_chg_wls.h>
#include "oplus_monitor_internal.h"
#include "oplus_chg_track.h"
#include "oplus_mms_wired.h"
#include "oplus_smart_chg.h"
#include <oplus_chg_plc.h>
#include <oplus_chg_wired.h>
#include "oplus_chg_pps.h"
#include <oplus_chg_mutual.h>
#include <oplus_reverse_chg.h>
#include <oplus_dischg_boost.h>

#define OPLUS_CHG_TRACK_MUL_BREAK_CNT			10
#define OPLUS_CHG_TRACK_WAIT_TIME_MS			3000
#define OPLUS_CHG_UPDATE_INFO_DELAY_MS			500
#define OPLUS_CHG_TRIGGER_MSG_LEN			(1024 * 2)
#define OPLUS_CHG_TRIGGER_REASON_TAG_LEN		32
#define OPLUS_CHG_TRACK_LOG_TAG				"OplusCharger"
#define OPLUS_CHG_TRACK_EVENT_ID			"charge_monitor"
#define OPLUS_CHG_TRACK_DWORK_RETRY_CNT			3

#define OPLUS_CHG_TRACK_UI_SOC_LOAD_JUMP_THD		5
#define OPLUS_CHG_TRACK_SOC_JUMP_THD			5
#define OPLUS_CHG_TRACK_UI_SOC_JUMP_THD			5
#define OPLUS_CHG_TRACK_UI_SOC_TO_SOC_JUMP_THD		3
#define OPLUS_CHG_TRACK_DEBUG_UISOC_SOC_INVALID		0XFF

#define OPLUS_CHG_TRACK_POWER_TYPE_LEN			24
#define OPLUS_CHG_TRACK_BATT_FULL_REASON_LEN		24
#define OPLUS_CHG_TRACK_CHG_ABNORMAL_REASON_LEN		24
#define OPLUS_CHG_TRACK_COOL_DOWN_STATS_LEN		24
#define OPLUS_CHG_TRACK_COOL_DOWN_PACK_LEN		320
#define OPLUS_CHG_TRACK_FASTCHG_BREAK_REASON_LEN	24
#define OPLUS_CHG_TRACK_VOOCPHY_NAME_LEN		16
#define OPLUS_CHG_TRACK_DEVICE_NAME_LEN			24

#define OPLUS_CHG_TRACK_CHG_ABNORMAL_REASON_LENS	160

#define TRACK_WLS_ADAPTER_TYPE_UNKNOWN 			0x00
#define TRACK_WLS_ADAPTER_TYPE_VOOC 			0x01
#define TRACK_WLS_ADAPTER_TYPE_SVOOC 			0x02
#define TRACK_WLS_ADAPTER_TYPE_USB 			0x03
#define TRACK_WLS_ADAPTER_TYPE_NORMAL 			0x04
#define TRACK_WLS_ADAPTER_TYPE_EPP 			0x05
#define TRACK_WLS_ADAPTER_TYPE_SVOOC_50W 		0x06
#define TRACK_WLS_ADAPTER_TYPE_PD_65W 			0x07

#define TRACK_WLS_DOCK_MODEL_0				0x00
#define TRACK_WLS_DOCK_MODEL_1				0x01
#define TRACK_WLS_DOCK_MODEL_2				0x02
#define TRACK_WLS_DOCK_MODEL_3				0x03
#define TRACK_WLS_DOCK_MODEL_4				0x04
#define TRACK_WLS_DOCK_MODEL_5				0x05
#define TRACK_WLS_DOCK_MODEL_6				0x06
#define TRACK_WLS_DOCK_MODEL_7				0x07
#define TRACK_WLS_DOCK_MODEL_8				0x08
#define TRACK_WLS_DOCK_MODEL_9				0x09
#define TRACK_WLS_DOCK_MODEL_A				0x0a
#define TRACK_WLS_DOCK_MODEL_B				0x0b
#define TRACK_WLS_DOCK_MODEL_10				0x10
#define TRACK_WLS_DOCK_MODEL_11				0x11
#define TRACK_WLS_DOCK_MODEL_12				0x12
#define TRACK_WLS_DOCK_MODEL_13				0x13
#define TRACK_WLS_DOCK_THIRD_PARTY			0x1F

#define TRACK_POWER_2000MW				2000
#define TRACK_POWER_2500MW				2500
#define TRACK_POWER_5000MW				5000
#define TRACK_POWER_6000MW				6000
#define TRACK_POWER_7000MW				7000
#define TRACK_POWER_7500MW				7500
#define TRACK_POWER_10000MW				10000
#define TRACK_POWER_12000MW				12000
#define TRACK_POWER_15000MW				15000
#define TRACK_POWER_18000MW				18000
#define TRACK_POWER_20000MW				20000
#define TRACK_POWER_30000MW				30000
#define TRACK_POWER_36000MW				36000
#define TRACK_POWER_40000MW				40000
#define TRACK_POWER_50000MW				50000
#define TRACK_POWER_65000MW				65000
#define TRACK_POWER_100000MW				100000
#define TRACK_POWER_120000MW				120000
#define TRACK_POWER_150000MW				150000

#define TRACK_INPUT_VOL_5000MV				5000
#define TRACK_INPUT_VOL_10000MV				10000
#define TRACK_INPUT_VOL_11000MV				11000
#define TRACK_INPUT_CURR_3000MA				3000
#define TRACK_INPUT_CURR_4000MA				4000
#define TRACK_INPUT_CURR_4100MA				4100
#define TRACK_INPUT_CURR_5000MA				5000
#define TRACK_INPUT_CURR_6000MA				6000
#define TRACK_INPUT_CURR_6100MA				6100
#define TRACK_INPUT_CURR_6500MA				6500
#define TRACK_INPUT_CURR_7300MA				7300
#define TRACK_INPUT_CURR_8000MA				8000
#define TRACK_INPUT_CURR_9100MA				9100
#define TRACK_INPUT_CURR_11000MA			11000
#define TRACK_INPUT_CURR_11400MA			11400

#define TRACK_CYCLE_RECORDIING_TIME_2MIN		120
#define TRACK_CYCLE_RECORDIING_TIME_90S			90

#define TRACK_TIME_1MIN_THD				60
#define TRACK_TIME_2MIN_THD				120
#define TRACK_TIME_30MIN_THD				1800
#define TRACK_TIME_1HOU_THD				3600
#define TRACK_TIME_7S_JIFF_THD				(7 * 1000)
#define TRACK_TIME_500MS_JIFF_THD			(5 * 100)
#define TRACK_TIME_1000MS_JIFF_THD			(1 * 1000)
#define TRACK_TIME_5MIN_JIFF_THD			(5 * 60 * 1000)
#define TRACK_TIME_10MIN_JIFF_THD			(10 * 60 * 1000)
#define TRACK_TIME_20MIN_JIFF_THD			(20 * 60 * 1000)
#define TRACK_TIME_30MIN_JIFF_THD			(30 * 60 * 1000)
#define TRACK_TIME_SCHEDULE_UI_SOC_LOAD_JUMP		90000
#define TRACK_THRAD_PERIOD_TIME_S			5
#define TRACK_NO_CHRGING_TIME_PCT			70
#define TRACK_COOLDOWN_CHRGING_TIME_PCT			70
#define TRACK_WLS_SKEWING_CHRGING_TIME_PCT		2

#define TRACK_PERIOD_CHG_CAP_MAX_SOC			100
#define TRACK_PERIOD_CHG_CAP_INIT			(-101)
#define TRACK_PERIOD_CHG_AVERAGE_SPEED_INIT		(-10000)
#define TRACK_T_THD_1000_MS				1000
#define TRACK_T_THD_500_MS				500
#define TRACK_T_THD_6000_MS				6000
#define TRACK_T_THD_2000_MS				2000

#define TRACK_LOCAL_T_NS_TO_MS_THD			1000000
#define TRACK_LOCAL_T_NS_TO_S_THD			1000000000

#define TRACK_LED_MONITOR_SOC_POINT			98
#define TRACK_CHG_VOOC_BATT_VOL_DIFF_MV			100

#define TRACK_REF_SOC_50				50
#define TRACK_REF_SOC_75				75
#define TRACK_REF_SOC_85				85
#define TRACK_REF_SOC_90				90
#define TRACK_REF_VOL_5000MV				5000
#define TRACK_REF_VOL_10000MV				10000
#define TRACK_REF_TIME_6S				6
#define TRACK_REF_TIME_8S				8
#define TRACK_REF_TIME_10S				10
#define TRACK_NO_CHG_CNT_60S				60
#define TRACK_NO_CHARGING_IBAT				(-50)
#define TRACK_HIDL_PARALLELCHG_FOLDMODE_INFO_LEN	512
#define TRACK_HIDL_TTF_INFO_LEN				512

#define TRACK_FASTCHG_HIGH_TEMP				440
#define TRACK_FASTCHG_LOW_TEMP				0
#define TRACK_PROTOCOL_SWITCH_COUNT			3
#define TRACK_COOL_DOWN_TOO_SMALL			3
#define TRACK_COOL_DOWN_TOO_SMALL_SINGLE_CELL		5

#define TRACK_HIDL_DATA_LEN			512
#define TRACK_HIDL_BCC_INFO_COUNT		8
#define TRACK_HIDL_BCC_CMD_LEN			48
#define TRACK_HIDL_BCC_ERR_LEN			256
#define TRACK_HIDL_BCC_ERR_REASON_LEN		24
#define TRACK_HIDL_BCC_INFO_LEN			(TRACK_HIDL_BCC_INFO_COUNT * TRACK_HIDL_BCC_CMD_LEN)
#define TRACK_HIDL_BMS_INFO_LEN 		320
#define TRACK_HIDL_UISOH_INFO_LEN		512
#define TRACK_HIDL_CHG_UP_LIMIT_INFO_LEN		512
#define TRACK_HIDL_HYPER_INFO_LEN		256
#define TRACK_HIDL_BDD_INFO_LEN			512
#define TRACK_HIDL_BAT_INFO_LEN			256
#define TRACK_SOFT_ABNORMAL_UPLOAD_PERIOD	(24 * 3600)
#define TRACK_SOFT_UPLOAD_COUNT_MAX		10
#define TRACK_SOFT_SOH_UPLOAD_COUNT_MAX		2000

#define TRACK_APP_REAL_NAME_LEN			48
#define TRACK_APP_TOP_INDEX_DEFAULT		255
#define TRACK_APP_REAL_NAME_DEFAULT		"com.android.launcher"

#define GAUGE_INFO_TRACK_FIFO_NUMS		4
#define GAUGE_INFO_TRACK_FIFO_ONE_SIZE		512

#define OPLUS_CHG_TRACK_POWER_INFO_LEN		256
#define TRACK_GAUGE_UPLOAD_PERIOD		(24 * 3600)

#define TRACK_GAUGE_UPLOAD_COUNT_MAX		10
#define OPLUS_CHG_TRACK_SOC_THD(x)		(x)
#define OPLUS_CHG_TRACK_TIME_THD_S(x)		(x)
#define OPLUS_CHG_TRACK_BATT_VOL_MV(x)		(x)
#define OPLUS_CHG_TRACK_PCT_THD(x)		x / 100
#define OPLUS_CHG_TRACK_SOH_THD(x)		(x)
#define OPLUS_CHG_TRACK_CC_THD(x)		(x)
#define OPLUS_CHG_TRACK_TEMP_THD(x)		(x)

#define OPLUS_CHG_TRACK_SCENE_GAGUE_DEFAULT	"default"
#define OPLUS_CHG_TRACK_SCENE_GAGUE_SOC_1_PCT	"soc_smooth_to_1"
#define OPLUS_CHG_TRACK_SCENE_GAGUE_TERM_VOLT_OK	"term_volt_ok"
#define OPLUS_CHG_TRACK_SCENE_GAGUE_LIFETIME_INFO	"lifetime_info"
#define TRACK_GAUGE_LIFETIME_UPLOAD_PERIOD		(48 * 3600)
#define TRACK_GAUGE_LIFETIME_CHECK_PERIOD		1800

#define OPLUS_CHG_TRACK_DEVICE_ERR_NAME_LEN	32
#define TRACK_GAUGE_NAME_LEN			12
#define TRACK_CP_ABNORMAL_CNT_MAX		3

#define SILI_ALG_MONITOR_BATT_TEMP_THR 250
#define SILI_ALG_MONITOR_SYS_TERM_VOLT_DIFF 20
#define SILI_ALG_MONITOR_FIRMWARE_TERM_VOLT_TEMP_THR 270

#define TRACK_VALID_UTC_MIN_TIME		946656000
#define PRE_CHG_BATT_R_LEN            		64

#define TRACK_MONITOR_BATT_START_SOC	30

#define MILLIMETER	1000

#define TRACK_FCL_INFO_LEN 16
#define PRE_CHG_BATT_R_LEN 64
enum oplus_chg_track_voocphy_type {
	TRACK_NO_VOOCPHY = 0,
	TRACK_ADSP_VOOCPHY,
	TRACK_AP_SINGLE_CP_VOOCPHY,
	TRACK_AP_DUAL_CP_VOOCPHY,
	TRACK_MCU_VOOCPHY,
};

enum oplus_chg_track_fastchg_status {
	TRACK_FASTCHG_STATUS_UNKOWN = 0,
	TRACK_FASTCHG_STATUS_START,
	TRACK_FASTCHG_STATUS_NORMAL,
	TRACK_FASTCHG_STATUS_WARM,
	TRACK_FASTCHG_STATUS_DUMMY,
};

enum oplus_chg_track_power_type {
	TRACK_CHG_TYPE_UNKNOW,
	TRACK_CHG_TYPE_WIRE,
	TRACK_CHG_TYPE_WIRELESS,
};

enum oplus_chg_track_soc_section {
	TRACK_SOC_SECTION_DEFAULT,
	TRACK_SOC_SECTION_LOW,
	TRACK_SOC_SECTION_MEDIUM,
	TRACK_SOC_SECTION_HIGH,
	TRACK_SOC_SECTION_OVER,
};

enum oplus_chg_track_hidl_type {
	TRACK_HIDL_DEFAULT,
	TRACK_HIDL_BCC_INFO,
	TRACK_HIDL_BCC_ERR,
	TRACK_HIDL_BMS_INFO,
	TRACK_HIDL_HYPER_INFO,
	TRACK_HIDL_WLS_THIRD_ERR,
	TRACK_HIDL_UISOH_INFO,
	TRACK_HIDL_PARALLELCHG_FOLDMODE_INFO,
	TRACK_HIDL_TTF_INFO,
	TRACK_HIDL_BCC_SI_INFO,
	TRACK_HIDL_BCC_SI_ERR,
	TRACK_HIDL_EIS_INFO,
	TRACK_HIDL_EIS_ERR,
	TRACK_HIDL_ANTI_EXPANSION_INFO,
	TRACK_HIDL_CHG_UP_LIMIT_INFO,
	TRACK_HIDL_BAT_INFO,
	TRACK_HIDL_BDD_INFO,
};

enum oplus_chg_track_sys_term_vol_temp_inr {
	TRACK_SYS_TERM_VOL_TEMP_LOW,
	TRACK_SYS_TERM_VOL_TEMP_HIGH,
	TRACK_SYS_TERM_VOL_TEMP_MAX,
};

enum oplus_chg_track_device_name {
	TRACK_PPS_DEVICE_NAME,
	TRACK_VOOC_DEVICE_NAME,
	TRACK_UFCS_DEVICE_NAME,
	TRACK_BUCK_DEVICE_NAME,
	TRACK_DEVICE_NAME_MAX,
};

enum oplus_chg_track_ui_online_check_status {
	TRACK_UI_ONLINE_DEFAULT,
	TRACK_UI_ONLINE_CHECK_START,
	TRACK_UI_ONLINE_CHECK_END
};

struct sili_sys_term_vol_maximum{
	int max_volt;
	int min_volt;
	int max_point_batt_temp;
	int min_point_batt_temp;
	int max_point_batt_soc;
	int min_point_batt_soc;
};

struct oplus_chg_track_app_ref {
	u8 *alias_name;
	u8 *real_name;
	u32 cont_time;
};

struct oplus_chg_track_fcl {
	int one_full_trigger_cnt;
	int n_full_trigger_cnt;
	unsigned char info[TRACK_FCL_INFO_LEN];
};

struct oplus_chg_track_app_status{
	struct mutex app_lock;
	u32 change_t;
	bool app_cal;
	u8 curr_top_index;
	u8 pre_top_name[TRACK_APP_REAL_NAME_LEN];
	u8 curr_top_name[TRACK_APP_REAL_NAME_LEN];
};

struct oplus_chg_track_vooc_type {
	int chg_type;
	int vol;
	int cur;
	char name[OPLUS_CHG_TRACK_POWER_TYPE_LEN];
};

struct oplus_chg_track_type {
	int type;
	int power;
	char name[OPLUS_CHG_TRACK_POWER_TYPE_LEN];
};

struct oplus_chg_track_wired_type {
	int power;
	int adapter_id;
	char adapter_type[OPLUS_CHG_TRACK_POWER_TYPE_LEN];
	int emark_power;
};

struct oplus_chg_track_wls_type {
	int power;
	char adapter_type[OPLUS_CHG_TRACK_POWER_TYPE_LEN];
	char dock_type[OPLUS_CHG_TRACK_POWER_TYPE_LEN];
	char first_wls_adapter_type[OPLUS_CHG_TRACK_POWER_TYPE_LEN];
	int tx_manu_id;
	int vendor_id;
	int product_id;
	bool wls_fastchg_break;
};

struct oplus_chg_track_vbus_vol_type {
	int vbus_vol_type;
	int type;
	int power;
	char name[OPLUS_CHG_TRACK_POWER_TYPE_LEN];
};

struct oplus_chg_track_power {
	int power_type;
	char power_mode[OPLUS_CHG_TRACK_POWER_TYPE_LEN];
	struct oplus_chg_track_wired_type wired_info;
	struct oplus_chg_track_wls_type wls_info;
};

struct oplus_chg_track_batt_full_reason {
	int notify_flag;
	char reason[OPLUS_CHG_TRACK_BATT_FULL_REASON_LEN];
};

struct oplus_chg_track_chg_abnormal_reason {
	int notify_code;
	char reason[OPLUS_CHG_TRACK_CHG_ABNORMAL_REASON_LEN];
	bool happened;
};

struct oplus_chg_track_cool_down_stats {
	int level;
	int time;
	char level_name[OPLUS_CHG_TRACK_COOL_DOWN_STATS_LEN];
};

#define SILI_CC_TERM_VOL_CURVE_MAX 6
struct gauge_sili_cc_term_vol_curve {
	u32 cc;
	u32 term_volt;
};

struct oplus_chg_track_gauge_sili_cc_term_vol_curve {
	struct gauge_sili_cc_term_vol_curve limits[SILI_CC_TERM_VOL_CURVE_MAX];
	int nums;
};

struct track_cp_abnormal_info {
	int record_cnt;
	struct mutex lock;
	char err_type_vector[TRACK_CP_ABNORMAL_CNT_MAX][OPLUS_CHG_TRACK_DEVICE_ERR_NAME_LEN];
};

struct mul_break_single_ele {
	unsigned long break_jiffies;
	char reason[OPLUS_CHG_TRACK_FASTCHG_BREAK_REASON_LEN];
	struct oplus_chg_track_wired_type wired_type;
	struct oplus_chg_track_wls_type wls_type;
};

struct track_mul_break_info {
	int ele_nums;
	int break_cnt;
	int total_break_cnt;
	struct mutex lock;
	char reason[OPLUS_CHG_TRACK_FASTCHG_BREAK_REASON_LEN];
	char last_wired_adapter_type[OPLUS_CHG_TRACK_POWER_TYPE_LEN];
	struct oplus_chg_track_wired_type wired_type;
	struct oplus_chg_track_wls_type wls_type;
	struct mul_break_single_ele ele_vector[OPLUS_CHG_TRACK_MUL_BREAK_CNT];
	enum oplus_chg_track_ui_online_check_status ui_online_check_status;
	struct delayed_work ui_online_check_again_work;
	bool ui_online;
	bool temp_ui_online;
};

struct track_device_name {
	const char *suffixes;
	enum oplus_chg_track_device_name index;
};

struct oplus_chg_track_slowchg_info {
	struct mutex lock;
	bool retention_once;
	bool usbtemp_once;
	char first_adapter_type[OPLUS_CHG_TRACK_POWER_TYPE_LEN];
	char last_adapter_type[OPLUS_CHG_TRACK_POWER_TYPE_LEN];
	char last_fastchg_break_reason[OPLUS_CHG_TRACK_FASTCHG_BREAK_REASON_LEN];
};

struct oplus_chg_track_cfg {
	int track_ver;
	int fast_chg_break_t_thd;
	int general_chg_break_t_thd;
	int wls_chg_break_t_thd;
	int wls_normal_chg_break_t_thd;
	int voocphy_type;
	int wired_fast_chg_scheme;
	int wls_fast_chg_scheme;
	int wls_epp_chg_scheme;
	int wls_bpp_chg_scheme;
	int wls_max_power;
	int wired_max_power;
	struct exception_data exception_data;
	bool track_gauge_ctrl;
	int external_gauge_num;
	int nominal_fcc1;
	int nominal_fcc2;
	int nominal_qmax1;
	int nominal_qmax2;
	bool low_soc_monitor;

	struct oplus_chg_track_gauge_sili_cc_term_vol_curve sili_cc_term_vol_curve;
	int gauge_max_cell_vol;
	int gauge_max_charge_curr;
	int gauge_max_dischg_curr;
	int gauge_max_cell_temp;
	int gauge_min_cell_temp;
	int gauge_firmware_term_volt;
	bool gauge_lifetime_support;
};

struct oplus_chg_track_fastchg_break {
	int code;
	char name[OPLUS_CHG_TRACK_FASTCHG_BREAK_REASON_LEN];
};

struct oplus_chg_track_voocphy_info {
	int voocphy_type;
	char name[OPLUS_CHG_TRACK_VOOCPHY_NAME_LEN];
};

struct oplus_chg_track_speed_ref {
	int ref_soc;
	int ref_power;
	int ref_curr;
};

struct oplus_chg_track_hidl_cmd {
	u32 cmd;
	u32 data_size;
	u8 data_buf[TRACK_HIDL_DATA_LEN];
};

struct oplus_chg_track_hidl_bcc_info_cmd {
	u32 err_code;
	u8 data_buf[TRACK_HIDL_BCC_CMD_LEN];
};

struct oplus_chg_track_hidl_bcc_info {
	u8 count;
	u32 err_code;
	u8 data_buf[TRACK_HIDL_BCC_INFO_LEN];
};

struct oplus_chg_track_hidl_bcc_err_cmd {
	u8 err_reason[TRACK_HIDL_BCC_ERR_REASON_LEN];
	u8 data_buf[TRACK_HIDL_BCC_ERR_LEN];
};

struct oplus_chg_track_hidl_bcc_err {
	struct mutex track_bcc_err_lock;
	bool bcc_err_uploading;
	oplus_chg_track_trigger *bcc_err_load_trigger;
	struct delayed_work bcc_err_load_trigger_work;
	struct oplus_chg_track_hidl_bcc_err_cmd bcc_err;
};

struct oplus_chg_track_hidl_chg_up_info_cmd {
	u8 data_buf[TRACK_HIDL_CHG_UP_LIMIT_INFO_LEN];
};
struct oplus_chg_track_hidl_chg_up_info{
	struct mutex track_chg_up_info_lock;
	bool chg_up_info_uploading;
	oplus_chg_track_trigger *chg_up_info_load_trigger;
	struct delayed_work chg_up_info_load_trigger_work;
	struct oplus_chg_track_hidl_chg_up_info_cmd chg_up_info;
};

struct oplus_chg_track_hidl_bdd_info_cmd {
	u8 data_buf[TRACK_HIDL_BDD_INFO_LEN];
};
struct oplus_chg_track_hidl_bdd_info{
	struct mutex track_bdd_info_lock;
	bool bdd_info_uploading;
	oplus_chg_track_trigger *bdd_info_load_trigger;
	struct delayed_work bdd_info_load_trigger_work;
	struct oplus_chg_track_hidl_bdd_info_cmd bdd_info;
};


struct oplus_chg_track_hidl_uisoh_info_cmd {
	u8 data_buf[TRACK_HIDL_UISOH_INFO_LEN];
};
struct oplus_chg_track_hidl_uisoh_info{
	struct mutex track_uisoh_info_lock;
	bool uisoh_info_uploading;
	oplus_chg_track_trigger *uisoh_info_load_trigger;
	struct delayed_work uisoh_info_load_trigger_work;
	struct oplus_chg_track_hidl_uisoh_info_cmd uisoh_info;
};

struct oplus_chg_track_hidl_bae_info_cmd {
	int anti_expansion_status;
	int anti_expansion_rus_status;
	int anti_expansion_high_risk_of_6hours;
	int anti_expansion_risk_state_of_21days;
};
struct oplus_chg_track_hidl_bae_info{
	struct mutex track_bae_info_lock;
	bool bae_info_uploading;
	oplus_chg_track_trigger *bae_info_load_trigger;
	struct delayed_work bae_info_load_trigger_work;
	struct oplus_chg_track_hidl_bae_info_cmd bae_info;
};

struct oplus_parallelchg_track_hidl_foldmode_info_cmd {
	u8 data_buf[TRACK_HIDL_PARALLELCHG_FOLDMODE_INFO_LEN];
};

struct oplus_parallelchg_track_hidl_foldmode_info {
	struct mutex track_lock;
	bool info_uploading;
	oplus_chg_track_trigger *load_trigger_info;
	struct delayed_work load_trigger_work;
	struct oplus_parallelchg_track_hidl_foldmode_info_cmd parallelchg_foldmode_info;
};

struct oplus_chg_track_hidl_ttf_info_cmd {
	u8 data_buf[TRACK_HIDL_TTF_INFO_LEN];
};

struct oplus_chg_track_hidl_ttf_info {
	struct mutex track_lock;
	bool info_uploading;
	oplus_chg_track_trigger *load_trigger_info;
	struct delayed_work load_trigger_work;
	struct oplus_chg_track_hidl_ttf_info_cmd ttf_info;
};

struct oplus_chg_track_hidl_bat_info_cmd {
	u8 data_buf[TRACK_HIDL_BAT_INFO_LEN];
};

struct oplus_chg_track_hidl_bat_info {
	struct mutex track_bat_info_lock;
	bool bat_info_uploading;
	oplus_chg_track_trigger *bat_info_load_trigger;
	struct delayed_work bat_info_load_trigger_work;
	struct oplus_chg_track_hidl_bat_info_cmd bat_info;
};

struct oplus_chg_track_hidl_hyper_info {
	char hyper_en;
};

struct oplus_chg_track_gague_err_reason {
	int err_type;
	char err_name[OPLUS_CHG_TRACK_DEVICE_ERR_NAME_LEN];
};

struct oplus_chg_track_gauge_params {
	struct oplus_mms *gauge_topic;
	int batt_volt;
	int batt_curr;
	int batt_temp;
	int qmax;
	int soc;
	int pre_soc;
	int pre_cc;
	int pre_soh;
	int pre_record_soc;
	int soh;
	int fcc;
	int cc;
	int ui_soc;
	int pre_ui_soc;
	int rsoc_smooth_base_time;
	int rsoc_smooth_base_soc;

	int pre_batt_temp;
	int sili_application_sys_term_volt;
	int pre_sili_monitor_sys_term_volt;
	int sili_monitor_sys_term_volt;
	struct sili_sys_term_vol_maximum sys_term_vol_maximum[TRACK_SYS_TERM_VOL_TEMP_MAX];
	struct sili_sys_term_vol_maximum pre_sys_term_vol_maximum[TRACK_SYS_TERM_VOL_TEMP_MAX];
	struct oplus_gauge_lifetime lifetime;
};

struct oplus_chg_track_gauge_info {
	u32 debug_soc_record_thd;
	u32 debug_err_type;
	u32 debug_upload_period_t;
	struct kfifo fifo;
	struct mutex track_lock;
	u32 debug_force_trigger;
	unsigned long trigger_type_flag;
	oplus_chg_track_trigger *load_trigger;
	struct delayed_work load_trigger_work;
	int upload_count;
	int pre_upload_time;
	struct oplus_chg_track_gauge_params params;
	int pre_check_time;
	int pre_time;
	int plugout_t;
	int nominal_fcc;
	int nominal_qmax;
	char device_name[TRACK_GAUGE_NAME_LEN];

	bool check_alg_application;
	struct mutex sili_alg_application_lock;
	struct delayed_work sili_alg_application_load_trigger_work;
	oplus_chg_track_trigger *sili_alg_application_load_trigger;

	struct mutex sili_alg_monitor_lock;
	struct delayed_work sili_alg_monitor_load_trigger_work;
	oplus_chg_track_trigger *sili_alg_monitor_load_trigger;

	struct mutex sili_alg_lifetime_lock;
	struct delayed_work sili_alg_lifetime_load_trigger_work;
	oplus_chg_track_trigger *sili_alg_lifetime_load_trigger;

	struct mutex batt_monitor_lock;
	struct delayed_work batt_monitor_load_trigger_work;
	oplus_chg_track_trigger *batt_monitor_load_trigger;
	bool charger_exist;
	bool single_dischg_check;
	int lifetime_check_time;
	int lifetime_upload_time;
};

struct oplus_chg_track_pps_info {
	int cpa_power;
	int adapter_power;
	int adapter_power_max_index;
	u32 *adapter_pdo;
};

struct oplus_chg_track_hidl_bcc_si_cmd {
	u8 type;
	u8 data_buf[TRACK_HIDL_DATA_LEN];
};

struct oplus_chg_track_hidl_bcc_si {
	struct mutex bcc_si_lock;
	bool bcc_si_uploading;
	oplus_chg_track_trigger *bcc_si_load_trigger;
	struct delayed_work bcc_si_load_trigger_work;
	u8 data_buf[TRACK_HIDL_DATA_LEN];
	struct oplus_chg_track_hidl_bcc_si_cmd bcc_si;
};

struct oplus_chg_track_hidl_eis_cmd {
	u8 type;
	u8 data_buf[TRACK_HIDL_DATA_LEN];
};

struct oplus_chg_track_hidl_eis {
	struct mutex eis_lock;
	bool eis_uploading;
	oplus_chg_track_trigger *eis_load_trigger;
	struct delayed_work eis_load_trigger_work;
	u8 data_buf[TRACK_HIDL_DATA_LEN];
	struct oplus_chg_track_hidl_eis_cmd eis;
};

struct oplus_chg_track_status {
	int curr_soc;
	int pre_soc;
	int pre_ui_soc;
	int curr_smooth_soc;
	int pre_smooth_soc;
	int curr_uisoc;
	int pre_uisoc;
	int pre_vbatt;
	int pre_time_utc;
	int pre_rm;
	int pre_fcc;
	int soc_jump_pre_batt_temp;
	bool soc_jumped;
	bool uisoc_jumped;
	bool uisoc_to_soc_jumped;
	bool uisoc_load_jumped;

	u8 debug_soc;
	u8 debug_uisoc;
	u8 debug_normal_charging_state;
	u8 debug_fast_prop_status;
	u8 debug_normal_prop_status;
	u8 debug_no_charging;
	u8 debug_slow_charging_force;
	u32 debug_chg_notify_flag;
	u32 debug_chg_notify_code;
	u8 debug_slow_charging_reason;
	bool debug_close_3hours;
	u8 debug_plugout_state;
	u8 debug_break_code;

	struct oplus_chg_track_power power_info;
	struct mutex track_status_lock;
	int fast_chg_type;
	int pre_fastchg_type;
	int pre_wired_type;
	int pre_vbus_vol_type;

	int chg_start_rm;
	int chg_start_soc;
	int chg_end_soc;
	int chg_start_temp;
	int chg_end_temp;
	int batt_start_temp;
	int batt_max_temp;
	int batt_max_vol;
	int batt_max_curr;
	int chg_max_vol;
	int chg_start_vol;
	int chg_start_cur;
	int chg_start_time;
	int chg_end_time;
	int chg_soc50_time;
	int chg_fast_full_time;
	int chg_normal_full_time;
	int chg_report_full_time;
	bool has_judge_speed;
	bool aging_ffc_trig;
	int aging_ffc_judge_vol[FFC_CHG_STEP_MAX];
	int aging_ffc_start_time;
	int ffc_end_time;
	int ffc_time;
	int cv_time;
	int ffc_start_main_soc;
	int ffc_start_sub_soc;
	int ffc_end_main_soc;
	int ffc_end_sub_soc;
	int aging_ffc_to_full_time;
	int chg_plugin_utc_t;
	int chg_plugout_utc_t;
	int rechg_counts;
	int in_rechging;
	struct rtc_time chg_plugin_rtc_t;
	struct rtc_time chg_plugout_rtc_t;
	struct rtc_time mmi_chg_open_rtc_t;
	struct rtc_time mmi_chg_close_rtc_t;
	int ufcs_err_type;

	int chg_five_mins_cap;
	int chg_ten_mins_cap;
	int chg_twenty_mins_cap;
	int chg_thirty_mins_cap;
	int chg_average_speed;
	char batt_full_reason[OPLUS_CHG_TRACK_BATT_FULL_REASON_LEN];

	int chg_max_temp;
	int chg_no_charging_cnt;
	int chg_no_chg_time_s;
	int no_charging_cal_curr_time;
	int no_charging_cal_pre_time;
	int continue_ledon_time;
	int continue_ledoff_time;
	int ledon_ave_speed;
	int ledoff_ave_speed;
	int ledon_rm;
	int ledoff_rm;
	int ledon_time;
	int ledoff_time;
	int led_on;
	int led_change_t;
	int prop_status;
	int led_change_rm;
	bool led_onoff_power_cal;
	int chg_total_cnt;
	int chg_total_time;
	unsigned long long chg_attach_time_ms;
	unsigned long long chg_detach_time_ms;
	unsigned long long wls_attach_time_ms;
	unsigned long long wls_detach_time_ms;
	struct oplus_chg_track_fastchg_break fastchg_break_info;
	char wired_break_crux_info[OPLUS_CHG_TRACK_CURX_INFO_LEN];
	char wls_break_crux_info[OPLUS_CHG_TRACK_CURX_INFO_LEN];

	bool usbtemp_is_high;
	bool mmi_chg;
	bool once_mmi_chg;
	int mmi_chg_open_t;
	int mmi_chg_close_t;
	int mmi_chg_constant_t;

	int slow_chg_open_t;
	int slow_chg_close_t;
	int slow_chg_open_n_t;
	int slow_chg_duration;
	int slow_chg_open_cnt;
	int slow_chg_watt;
	int slow_chg_pct;

	unsigned long not_record_reason;

	bool chg_speed_is_slow;
	bool tbatt_warm_once;
	bool tbatt_cold_once;
	int cool_down_effect_cnt;
	int cool_down_status;
	int cool_down_status_change_t;
	int soc_sect_status;
	int soc_low_sect_incr_rm;
	int soc_low_sect_cont_time;
	int soc_medium_sect_incr_rm;
	int soc_medium_sect_cont_time;
	int soc_high_sect_incr_rm;
	int soc_high_sect_cont_time;
	struct oplus_chg_track_speed_ref *wired_speed_ref;
	struct oplus_chg_track_speed_ref *wls_airvooc_speed_ref;
	struct oplus_chg_track_speed_ref *wls_epp_speed_ref;
	struct oplus_chg_track_speed_ref *wls_bpp_speed_ref;

	bool wls_status_keep;
	bool wls_need_upload;
	bool wired_need_upload;
	int wls_prop_status;
	int wls_skew_effect_cnt;
	bool chg_verity;
	char chg_abnormal_reason[OPLUS_CHG_TRACK_CHG_ABNORMAL_REASON_LENS];
	struct oplus_chg_track_hidl_bcc_info *bcc_info;
	struct oplus_chg_track_hidl_bcc_err bcc_err;
	struct oplus_chg_track_hidl_uisoh_info uisoh_info_s;
	struct oplus_chg_track_hidl_chg_up_info chg_up_limit_info_s;
	struct oplus_chg_track_hidl_bae_info bae_info_s;
	struct oplus_parallelchg_track_hidl_foldmode_info parallelchg_info;
	struct oplus_chg_track_hidl_ttf_info *ttf_info;
	struct oplus_chg_track_hidl_bcc_si bcc_si;
	struct oplus_chg_track_hidl_eis eis;
	struct oplus_chg_track_hidl_bat_info bat_info_s;
	struct oplus_chg_track_hidl_bdd_info bdd_info_s;
	int wired_max_power;
	int wls_max_power;
	struct oplus_chg_track_app_status app_status;
	int once_chg_cycle_status;
	int dual_chan_start_time;
	int dual_chan_time;
	int dual_chan_open_count;
	int track_gmtoff;

	u8 bms_info[TRACK_HIDL_BMS_INFO_LEN];
	u8 hyper_info[TRACK_HIDL_HYPER_INFO_LEN];

	char hyper_en;
	int hyper_stop_soc;
	int hyper_stop_temp;
	int hyper_last_time;
	int hyper_est_save_time;
	int hyper_ave_speed;

	int anti_expansion_status;
	int anti_expansion_rus_status;
	int anti_expansion_high_risk_of_6hours;
	int anti_expansion_risk_state_of_21days;

	int wls_pmw_pulse;
	unsigned long long wls_audio_t;
	unsigned long long wls_audio_t_total;
	unsigned long long wls_camera_t;
	unsigned long long wls_camera_t_total;
	bool wls_quiet_mdoe;
	unsigned long long wls_quiet_mdoe_t;
	unsigned long long wls_quiet_mdoe_t_total;
	unsigned long long wls_chg_t;
	unsigned long long wls_chg_t_total;
	int wls_pre_dock_type;
	int wls_pre_max_power;
	bool wls_manu_break;
	struct oplus_chg_track_fcl fcl;
};

struct oplus_chg_track {
	struct device *dev;
	struct oplus_monitor *monitor;
	struct task_struct *track_upload_kthread;

	struct mms_subscribe *err_subs;
	struct mms_subscribe *pps_subs;

	bool trigger_data_ok;
	struct mutex upload_lock;
	struct mutex trigger_data_lock;
	struct mutex trigger_ack_lock;
	oplus_chg_track_trigger trigger_data;
	struct completion trigger_ack;
	wait_queue_head_t upload_wq;

	struct workqueue_struct *trigger_upload_wq;
#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || \
	defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE) || \
	defined(CONFIG_OPLUS_KEVENT_UPLOAD)
	struct kernel_packet_info *dcs_info;
#endif
	struct delayed_work upload_info_dwork;
	struct mutex dcs_info_lock;
	int dwork_retry_cnt;

	struct oplus_chg_track_status track_status;
	struct oplus_chg_track_cfg track_cfg;

	int uisoc_1_start_vbatt_max;
	int uisoc_1_start_vbatt_min;
	int uisoc_1_start_batt_rm;
	int uisoc_1_start_time;

	oplus_chg_track_trigger uisoc_load_trigger;
	oplus_chg_track_trigger soc_trigger;
	oplus_chg_track_trigger uisoc_trigger;
	oplus_chg_track_trigger uisoc_to_soc_trigger;
	oplus_chg_track_trigger charger_info_trigger;
	oplus_chg_track_trigger no_charging_trigger;
	oplus_chg_track_trigger slow_charging_trigger;
	oplus_chg_track_trigger charging_break_trigger;
	oplus_chg_track_trigger wls_charging_break_trigger;
	oplus_chg_track_trigger usbtemp_load_trigger;
	oplus_chg_track_trigger vbatt_too_low_load_trigger;
	oplus_chg_track_trigger uisoc_drop_err_trigger;
	oplus_chg_track_trigger vbatt_diff_over_load_trigger;
	oplus_chg_track_trigger uisoc_keep_1_t_load_trigger;
	oplus_chg_track_trigger ic_err_msg_load_trigger;
	oplus_chg_track_trigger chg_into_liquid_load_trigger;
	oplus_chg_track_trigger plugout_state_trigger;
	oplus_chg_track_trigger dual_chan_err_load_trigger;
	oplus_chg_track_trigger usb_lpd_load_trigger;
	oplus_chg_track_trigger cycle_current_derating_trigger;
	struct delayed_work uisoc_load_trigger_work;
	struct delayed_work soc_trigger_work;
	struct delayed_work uisoc_trigger_work;
	struct delayed_work uisoc_to_soc_trigger_work;
	struct delayed_work charger_info_trigger_work;
	struct delayed_work cal_chg_five_mins_capacity_work;
	struct delayed_work cal_chg_ten_mins_capacity_work;
	struct delayed_work cal_chg_twenty_mins_capacity_work;
	struct delayed_work cal_chg_thirty_mins_capacity_work;
	struct delayed_work no_charging_trigger_work;
	struct delayed_work slow_charging_trigger_work;
	struct delayed_work charging_break_trigger_work;
	struct delayed_work wls_charging_break_trigger_work;
	struct delayed_work usbtemp_load_trigger_work;
	struct delayed_work usb_lpd_load_trigger_work;
	struct delayed_work usbtemp_trigger_no_charging_work;
	struct delayed_work vbatt_too_low_load_trigger_work;
	struct delayed_work vbatt_diff_over_load_trigger_work;
	struct delayed_work uisoc_keep_1_t_load_trigger_work;
	struct delayed_work ic_err_msg_trigger_work;
	struct delayed_work chg_into_liquid_trigger_work;
	struct delayed_work plugout_state_work;
	struct delayed_work dual_chan_err_load_trigger_work;
	struct delayed_work wired_online_err_trigger_work;
	struct delayed_work uisoc_keep_2_err_trigger_work;
	struct delayed_work uisoc_keep_3_err_trigger_work;
	struct delayed_work uisoc_drop_err_trigger_work;
	struct delayed_work endurance_change_work;
	struct delayed_work wired_retention_online_trigger_work;
	struct delayed_work wired_mul_break_trigger_work;
	struct delayed_work wls_mul_break_trigger_work;
	struct delayed_work prechg_r_update_work;
	struct delayed_work wired_reverse_chg_trigger_work;
	struct delayed_work wired_high_reverse_err_trigger_work;

	struct oplus_chg_mutual_notifier prechg_r_mutual;
	char prechg_r_data[PRE_CHG_BATT_R_LEN];

	oplus_chg_track_trigger *mmi_chg_info_trigger;
	oplus_chg_track_trigger *slow_chg_info_trigger;
	oplus_chg_track_trigger *chg_cycle_info_trigger;
	oplus_chg_track_trigger *wls_info_trigger;
	oplus_chg_track_trigger *ufcs_info_trigger;
	oplus_chg_track_trigger *pps_info_trigger;
	oplus_chg_track_trigger *deep_dischg_info_trigger;
	oplus_chg_track_trigger *wired_online_err_trigger;
	oplus_chg_track_trigger *uisoc_keep_2_err_trigger;
	oplus_chg_track_trigger *uisoc_keep_3_err_trigger;
	oplus_chg_track_trigger *rechg_info_trigger;
	oplus_chg_track_trigger *endurance_info_trigger;
	oplus_chg_track_trigger *bidirect_cp_info_trigger;
	oplus_chg_track_trigger *eis_timeout_info_trigger;
	oplus_chg_track_trigger *wired_retention_online_trigger;
	oplus_chg_track_trigger *wired_reverse_chg_trigger;
	oplus_chg_track_trigger *wired_high_reverse_err_trigger;
	oplus_chg_track_trigger *bal_info_trigger;
	oplus_chg_track_trigger *plc_info_trigger;
	oplus_chg_track_trigger *phy_cp_info_trigger;
	oplus_chg_track_trigger *soccp_crash_trigger;
	oplus_chg_track_trigger *bs_info_trigger;
	oplus_chg_track_trigger *state_keep_info_trigger;
	oplus_chg_track_trigger *usbin_abnormal_trigger;
	oplus_chg_track_trigger *sec_ic_meminfo_trigger;

	struct delayed_work mmi_chg_info_trigger_work;
	struct delayed_work slow_chg_info_trigger_work;
	struct delayed_work chg_cycle_info_trigger_work;
	struct delayed_work wls_info_trigger_work;
	struct delayed_work ufcs_info_trigger_work;
	struct delayed_work pps_info_trigger_work;
	struct delayed_work deep_dischg_info_trigger_work;
	struct delayed_work rechg_info_trigger_work;
	struct delayed_work bidirect_cp_info_trigger_work;
	struct delayed_work eis_timeout_info_trigger_work;
	struct delayed_work bal_info_trigger_work;
	struct delayed_work plc_info_trigger_work;
	struct delayed_work phy_cp_info_trigger_work;
	struct delayed_work soccp_crash_trigger_work;
	struct delayed_work bs_info_trigger_work;
	struct delayed_work state_keep_info_trigger_work;
	struct delayed_work usbin_abnormal_trigger_work;
	struct delayed_work sec_ic_meminfo_trigger_work;

	struct mutex mmi_chg_info_lock;
	struct mutex slow_chg_info_lock;
	struct mutex chg_cycle_info_lock;
	struct mutex wls_info_lock;
	struct mutex ufcs_info_lock;
	struct mutex pps_info_lock;
	struct mutex deep_dischg_info_lock;
	struct mutex rechg_info_lock;
	struct mutex bidirect_cp_info_lock;
	struct mutex eis_timeout_info_lock;
	struct mutex bal_info_lock;
	struct mutex plc_info_lock;
	struct mutex no_charging_info_lock;
	struct mutex phy_cp_info_lock;
	struct mutex soccp_crash_lock;
	struct mutex bs_info_lock;
	struct mutex state_keep_info_lock;
	struct mutex usbin_abnormal_lock;
	struct mutex sec_ic_meminfo_lock;

	char voocphy_name[OPLUS_CHG_TRACK_VOOCPHY_NAME_LEN];

	struct mutex access_lock;

	struct oplus_chg_track_gauge_info gauge_info;
	struct oplus_chg_track_gauge_info sub_gauge_info;

	struct track_mul_break_info mul_break_info;
	struct track_mul_break_info wls_mul_break_info;
	char device_ic_name[TRACK_DEVICE_NAME_MAX][OPLUS_CHG_TRACK_DEVICE_NAME_LEN];
	struct track_cp_abnormal_info cp_abnormal_info;
	struct oplus_chg_track_slowchg_info slowchg_info;
	struct oplus_chg_track_pps_info pps_info;
};

struct type_reason_table {
	int type_reason;
	char type_reason_tag[OPLUS_CHG_TRIGGER_REASON_TAG_LEN];
};

struct flag_reason_table {
	int flag_reason;
	char flag_reason_tag[OPLUS_CHG_TRIGGER_REASON_TAG_LEN];
};

struct oplus_chg_track_cp_err_reason {
	int err_type;
	char err_name[OPLUS_CHG_TRACK_DEVICE_ERR_NAME_LEN];
};

struct oplus_chg_track_i2c_err_reason {
	int err_type;
	char err_name[OPLUS_CHG_TRACK_DEVICE_ERR_NAME_LEN];
};

struct oplus_chg_track_ufcs_err_reason {
	int err_type;
	char err_name[OPLUS_CHG_TRACK_DEVICE_ERR_NAME_LEN];
};

static struct oplus_chg_track_cp_err_reason bidirect_cp_err_reason_table[] = {
	{ TRACK_BIDIRECT_CP_ERR_SC_EN_STAT, "SC_EN_STAT" },
	{ TRACK_BIDIRECT_CP_ERR_V2X_OVP, "V2X_OVP" },
	{ TRACK_BIDIRECT_CP_ERR_V1X_OVP, "V1X_OVP" },
	{ TRACK_BIDIRECT_CP_ERR_VAC_OVP, "VAC_OVP" },
	{ TRACK_BIDIRECT_CP_ERR_FWD_OCP, "FWD_OCP" },
	{ TRACK_BIDIRECT_CP_ERR_RVS_OCP, "RVS_OCP" },
	{ TRACK_BIDIRECT_CP_ERR_TSHUT, "TSHUT" },
	{ TRACK_BIDIRECT_CP_ERR_VAC2V2X_OVP, "VAC2V2X_OVP" },
	{ TRACK_BIDIRECT_CP_ERR_VAC2V2X_UVP, "VAC2V2X_UVP" },
	{ TRACK_BIDIRECT_CP_ERR_V1X_ISS_OPP, "V1X_ISS_OPP" },
	{ TRACK_BIDIRECT_CP_ERR_WD_TIMEOUT, "WD_TIMEOUT" },
	{ TRACK_BIDIRECT_CP_ERR_LNC_SS_TIMEOUT, "LNC_SS_TIMEOUT" },
};

static struct oplus_chg_track *g_track_chip;
static struct dentry *track_debugfs_root;
static struct oplus_chg_track_status *temp_track_status;
static DEFINE_MUTEX(debugfs_root_mutex);
static DEFINE_SPINLOCK(gauge_fifo_lock);

#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || \
	defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE) || \
	defined(CONFIG_OPLUS_KEVENT_UPLOAD)
static int oplus_chg_track_pack_dcs_info(struct oplus_chg_track *chip);
#endif
static int
oplus_chg_track_get_charger_type(struct oplus_monitor *monitor,
				 struct oplus_chg_track_status *track_status,
				 int type);
static int oplus_chg_track_obtain_wls_break_sub_crux_info(
	struct oplus_chg_track *track_chip, char *crux_info);
static int oplus_chg_track_upload_trigger_data(oplus_chg_track_trigger *data);
static int oplus_chg_track_get_current_time_s(struct rtc_time *tm);
static bool oplus_chg_track_get_mmi_chg(void);
static int oplus_chg_track_obtain_power_info(char *power_info, int len);
static int
oplus_chg_track_record_general_info(struct oplus_monitor *monitor,
				    struct oplus_chg_track_status *track_status,
				    oplus_chg_track_trigger *p_trigger_data,
				    int index);
static void oplus_chg_track_record_device_name(struct oplus_chg_track *track_chip,
	char *adapter_type,oplus_chg_track_trigger *p_trigger_data, int *index);
static int oplus_chg_track_handle_pps_pdo_info(struct oplus_chg_track *chip, char *p_data, int *ret, int t_len);
static int oplus_chg_track_force_upload_log(
	struct oplus_chg_track *chip, unsigned int type_reason, unsigned int flag_reason);
static void oplus_chg_track_gauge_sili_alg_application_work(struct work_struct *work);
static void oplus_chg_track_sub_gauge_sili_alg_application_work(struct work_struct *work);
static void oplus_chg_track_gauge_sili_alg_lifetime_work(struct work_struct *work);
static void oplus_chg_track_sub_gauge_sili_alg_lifetime_work(struct work_struct *work);
static void oplus_chg_track_gauge_sili_alg_monitor_work(struct work_struct *work);
static void oplus_chg_track_sub_gauge_sili_alg_monitor_work(struct work_struct *work);
static void oplus_chg_track_wired_mul_break_trigger_work(struct work_struct *work);
static void oplus_chg_track_wls_mul_break_trigger_work(struct work_struct *work);
static void oplus_chg_track_wired_mul_break_ui_online_check_again_work(struct work_struct *work);
static void oplus_chg_track_record_cp_abnormal_info(
	struct oplus_chg_track *track_chip, oplus_chg_track_trigger *p_trigger_data, int *index);
static int oplus_chg_track_slowchg_info_reset(struct oplus_chg_track_slowchg_info  *slowchg_info);
static void oplus_chg_track_gauge_batt_monitor_work(struct work_struct *work);
static void oplus_chg_track_sub_gauge_batt_monitor_work(struct work_struct *work);

#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || \
	defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE) || \
	defined(CONFIG_OPLUS_KEVENT_UPLOAD)
static struct type_reason_table track_type_reason_table[] = {
	{ TRACK_NOTIFY_TYPE_DEFAULT, "default" },
	{ TRACK_NOTIFY_TYPE_SOC_JUMP, "soc_error" },
	{ TRACK_NOTIFY_TYPE_GENERAL_RECORD, "general_record" },
	{ TRACK_NOTIFY_TYPE_NO_CHARGING, "no_charging" },
	{ TRACK_NOTIFY_TYPE_CHARGING_SLOW, "charge_slow" },
	{ TRACK_NOTIFY_TYPE_CHARGING_BREAK, "charge_break" },
	{ TRACK_NOTIFY_TYPE_DEVICE_ABNORMAL, "device_abnormal" },
	{ TRACK_NOTIFY_TYPE_SOFTWARE_ABNORMAL, "software_abnormal" },
	{ TRACK_NOTIFY_TYPE_UPLOAD_LOG, "upload_log" },
	{ TRACK_NOTIFY_TYPE_CRASH, "crash" },
};

static struct flag_reason_table track_flag_reason_table[] = {
	{ TRACK_NOTIFY_FLAG_DEFAULT, "default" },
	{ TRACK_NOTIFY_FLAG_UI_SOC_LOAD_JUMP, "UiSoc_LoadSocJump" },
	{ TRACK_NOTIFY_FLAG_SOC_JUMP, "SocJump" },
	{ TRACK_NOTIFY_FLAG_UI_SOC_JUMP, "UiSocJump" },
	{ TRACK_NOTIFY_FLAG_UI_SOC_TO_SOC_JUMP, "UiSoc-SocJump" },

	{ TRACK_NOTIFY_FLAG_CHARGER_INFO, "ChargerInfo" },
	{ TRACK_NOTIFY_FLAG_UISOC_KEEP_1_T_INFO, "UisocKeep1TInfo" },
	{ TRACK_NOTIFY_FLAG_VBATT_TOO_LOW_INFO, "VbattTooLowInfo" },
	{ TRACK_NOTIFY_FLAG_UISOC_DROP_ERROR, "UisocDropError" },
	{ TRACK_NOTIFY_FLAG_USBTEMP_INFO, "UsbTempInfo" },
	{ TRACK_NOTIFY_FLAG_VBATT_DIFF_OVER_INFO, "VbattDiffOverInfo" },
	{ TRACK_NOTIFY_FLAG_SERVICE_UPDATE_WLS_THIRD_INFO, "UpdateWlsThirdInfo" },
	{ TRACK_NOTIFY_FLAG_WLS_INFO, "WlsInfo" },
	{ TRACK_NOTIFY_FLAG_PARALLELCHG_FOLDMODE_INFO, "ParallelChgFoldModeInfo" },
	{ TRACK_NOTIFY_FLAG_MMI_CHG_INFO, "MmiChgInfo" },
	{ TRACK_NOTIFY_FLAG_SLOW_CHG_INFO, "SlowChgInfo" },
	{ TRACK_NOTIFY_FLAG_CHG_CYCLE_INFO, "ChgCycleInfo" },
	{ TRACK_NOTIFY_FLAG_TTF_INFO, "TtfInfo" },
	{ TRACK_NOTIFY_FLAG_UISOH_INFO, "UiSohInfo" },
	{ TRACK_NOTIFY_FLAG_CHG_UP_INFO, "ChgUpLimitInfo" },
	{ TRACK_NOTIFY_FLAG_BDD_INFO, "BattDamageDetectInfo" },
	{ TRACK_NOTIFY_FLAG_GAUGE_INFO, "GaugeInfo" },
	{ TRACK_NOTIFY_FLAG_BYPASS_BOOST_INFO, "DeepDischgInfo" },
	{ TRACK_NOTIFY_FLAG_DEEP_DISCHG_PROFILE, "DeepDischgProfile" },
	{ TRACK_NOTIFY_FLAG_RECHG_INFO, "RechgSocInfo" },
	{ TRACK_NOTIFY_FLAG_BCC_SI_INFO, "BccSiInfo" },
	{ TRACK_NOTIFY_FLAG_ENDURANCE_INFO, "EnduranceInfo" },
	{ TRACK_NOTIFY_FLAG_EIS_INFO, "EisInfo" },
	{ TRACK_NOTIFY_FLAG_PLC_INFO, "PlcInfo" },
	{ TRACK_NOTIFY_FLAG_LPD_INFO, "LPDInfo" },
	{ TRACK_NOTIFY_FLAG_ANTI_EXPANSION_INFO, "AntiExpansionInfo" },
	{ TRACK_NOTIFY_FLAG_DEC_CV_INFO, "DecCvInfo" },
	{ TRACK_NOTIFY_FLAG_WIRED_RETENTION_ONLINE, "WiredRetentionOnline" },
	{ TRACK_NOTIFY_FLAG_LOW_BATT_MONITOR, "LowBattMonitor" },
	{ TRACK_NOTIFY_FLAG_PRECHG_BATT_R_INFO, "PreChgBattR" },
	{ TRACK_NOTIFY_FLAG_BS_INFO, "BatterySmoothInfo" },
	{ TRACK_NOTIFY_FLAG_STATE_KEEP_INFO, "StateKeepInfo" },
	{ TRACK_NOTIFY_FLAG_WIRED_REVERSE_CHG_INFO, "UsbHpReverseCharging" },
	{ TRACK_NOTIFY_FLAG_WIRED_HIGH_REVERSE_ERR, "HighReverseErr" },
	{ TRACK_NOTIFY_FLAG_SHUTDOWN_VOL, "ShutdownVol" },

	{ TRACK_NOTIFY_FLAG_NO_CHARGING, "NoCharging" },
	{ TRACK_NOTIFY_FLAG_NO_CHARGING_OTG_ONLINE, "OtgOnline" },
	{ TRACK_NOTIFY_FLAG_NO_CHARGING_VBATT_LEAK, "VBattLeakage" },
	{ TRACK_NOTIFY_FLAG_NO_CHARGING_IBAT_LOW, "IbatLow" },
	{ TRACK_NOTIFY_FLAG_NO_CHARGING_USBTEMP_HIGH, "UsbTempHigh" },

	{ TRACK_NOTIFY_FLAG_CHG_SLOW_TBATT_WARM, "BattTempWarm" },
	{ TRACK_NOTIFY_FLAG_CHG_SLOW_TBATT_COLD, "BattTempCold" },
	{ TRACK_NOTIFY_FLAG_CHG_SLOW_NON_STANDARD_PA, "NonStandardAdatpter" },
	{ TRACK_NOTIFY_FLAG_CHG_SLOW_BATT_CAP_HIGH, "BattCapHighWhenPlugin" },
	{ TRACK_NOTIFY_FLAG_CHG_SLOW_COOLDOWN, "CoolDownCtlLongTime" },
	{ TRACK_NOTIFY_FLAG_CHG_SLOW_WLS_SKEW, "WlsSkew" },
	{ TRACK_NOTIFY_FLAG_CHG_SLOW_VERITY_FAIL, "VerityFail" },
	{ TRACK_NOTIFY_FLAG_CHG_SLOW_OTHER, "Other" },
	{ TRACK_NOTIFY_FLAG_CHG_SLOW_POWER_DEVICE_ABNORMAL, "PowerDeviceAbnormal" },
	{ TRACK_NOTIFY_FLAG_CHG_SLOW_RETENTION_FALLBACK, "RetentionOn" },
	{ TRACK_NOTIFY_FLAG_CHG_SLOW_COMMUNICATION_ABNORMAL, "CommuAbnormal" },
	{ TRACK_NOTIFY_FLAG_CHG_SLOW_BTB_OVER_TEMP, "BtbOverTemp" },
	{ TRACK_NOTIFY_FLAG_CHG_SLOW_R_COOLDOWN, "R_CoolDown" },
	{ TRACK_NOTIFY_FLAG_CHG_SLOW_QUIET_MODE, "QuietModeLong" },
	{ TRACK_NOTIFY_FLAG_CHG_SLOW_CYCLE_CURR_DERATING, "CycleCurrentDerating" },

	{ TRACK_NOTIFY_FLAG_FAST_CHARGING_BREAK, "FastChgBreak" },
	{ TRACK_NOTIFY_FLAG_GENERAL_CHARGING_BREAK, "GeneralChgBreak" },
	{ TRACK_NOTIFY_FLAG_WLS_CHARGING_BREAK, "WlsChgBreak" },
	{ TRACK_NOTIFY_FLAG_CHG_FEED_LIQUOR, "ChgintoliquidAbnormal" },

	{ TRACK_NOTIFY_FLAG_WLS_ABNORMAL, "WlsAbnormal" },
	{ TRACK_NOTIFY_FLAG_GPIO_ABNORMAL, "GpioAbnormal" },

	{ TRACK_NOTIFY_FLAG_CP_ABNORMAL, "CpAbnormal" },
	{ TRACK_NOTIFY_FLAG_PLAT_PMIC_ABNORMAL, "PlatPmicAbnormal" },
	{ TRACK_NOTIFY_FLAG_EXTERN_PMIC_ABNORMAL, "ExternPmicAbnormal" },
	{ TRACK_NOTIFY_FLAG_GAGUE_ABNORMAL, "GagueAbnormal" },
	{ TRACK_NOTIFY_FLAG_DCHG_ABNORMAL, "DchgAbnormal" },
	{ TRACK_NOTIFY_FLAG_PARALLEL_UNBALANCE_ABNORMAL, "ParallelUnbalance" },
	{ TRACK_NOTIFY_FLAG_MOS_ERROR_ABNORMAL, "MosError" },
	{ TRACK_NOTIFY_FLAG_HK_ABNORMAL, "HouseKeepingAbnormal" },
	{ TRACK_NOTIFY_FLAG_UFCS_IC_ABNORMAL, "UFCSICAbnormal" },
	{ TRACK_NOTIFY_FLAG_ADAPTER_ABNORMAL, "AdapterAbnormal" },
	{ TRACK_NOTIFY_FLAG_BATT_ID_INFO, "Batt_Id_Info" },
	{ TRACK_NOTIFY_FLAG_I2C_ABNORMAL, "I2cAbnormal" },
	{ TRACK_NOTIFY_FLAG_BOOST_BUCK_ERR, "BoostICAbnormal" },
	{ TRACK_NOTIFY_FLAG_NTC_ABNORMAL, "NTCAbnormal" },
	{ TRACK_NOTIFY_FLAG_BAL_IC_ABNORMAL, "BalICAbnormal" },
	{ TRACK_NOTIFY_FLAG_FPGA_RST_ABNORMAL, "FPGARstAbnormal" },
	{ TRACK_NOTIFY_FLAG_POWER_DOWN_ABNORMAL, "PowerDownAbnormal" },
	{ TRACK_NOTIFY_FLAG_IC_BURN, "IcBurn" },


	{ TRACK_NOTIFY_FLAG_UFCS_ABNORMAL, "UfcsAbnormal" },
	{ TRACK_NOTIFY_FLAG_COOLDOWN_ABNORMAL, "CoolDownAbnormal" },
	{ TRACK_NOTIFY_FLAG_SMART_CHG_ABNORMAL, "SmartChgAbnormal" },
	{ TRACK_NOTIFY_FLAG_WLS_THIRD_ENCRY_ABNORMAL, "WlsThirdEncryAbnormal" },
	{ TRACK_NOTIFY_FLAG_PEN_MATCH_STATE_ABNORMAL, "PenMatchStateAbnormal" },
	{ TRACK_NOTIFY_FLAG_PPS_ABNORMAL, "PPSAbnormal" },
	{ TRACK_NOTIFY_FLAG_FASTCHG_START_ABNORMAL, "FastchgStartClearError" },
	{ TRACK_NOTIFY_FLAG_DUAL_CHAN_ABNORMAL, "DualChanAbnormal" },
	{ TRACK_NOTIFY_FLAG_DUMMY_START_ABNORMAL, "DummyStartClearError" },
	{ TRACK_NOTIFY_FLAG_WIRED_ONLINE_ERROR, "WiredOnlineStatusError" },
	{ TRACK_NOTIFY_FLAG_UISOC_KEEP_2_ERROR, "UisocKeep2Error" },
	{ TRACK_NOTIFY_FLAG_UISOC_KEEP_3_ERROR, "UisocKeep3Error" },
	{ TRACK_NOTIFY_FLAG_BCC_SI_ABNORMAL, "BccSiAbnormal" },
	{ TRACK_NOTIFY_FLAG_EIS_ABNORMAL, "EisAbnormal" },
	{ TRACK_NOTIFY_FLAG_BAL_ABNORMAL, "BalAbnormal" },
	{ TRACK_NOTIFY_FLAG_STATE_KEEP_ABNORMAL, "StateKeepAbnormal" },
	{ TRACK_NOTIFY_FLAG_USBIN_ABNORMAL, "UsbinAbnormal" },
	{ TRACK_NOTIFY_FLAG_SEC_IC_MEMINFO, "SecICMemInfo" },

	{ TRACK_NOTIFY_FLAG_UPLOAD_BREAK_LOG, "UploadBreakLog" },
	{ TRACK_NOTIFY_FLAG_UPLOAD_NO_CHG_LOG, "UploadNoChgLog" },
	{ TRACK_NOTIFY_FLAG_UPLOAD_WLS_BREAK_LOG, "UploadWlsBreakLog" },

	{ TRACK_NOTIFY_FLAG_SOCCP_CRASH, "SoccpCrash" },
};
#endif

static struct oplus_chg_track_type wired_type_table[] = {
	{ OPLUS_CHG_USB_TYPE_UNKNOWN, TRACK_POWER_2500MW, "unknow" },
	{ OPLUS_CHG_USB_TYPE_SDP, TRACK_POWER_2500MW, "sdp" },
	{ OPLUS_CHG_USB_TYPE_DCP, TRACK_POWER_10000MW, "dcp" },
	{ OPLUS_CHG_USB_TYPE_CDP, TRACK_POWER_7500MW, "cdp" },
	{ OPLUS_CHG_USB_TYPE_ACA, TRACK_POWER_10000MW, "dcp" },
	{ OPLUS_CHG_USB_TYPE_C, TRACK_POWER_10000MW, "dcp" },
	{ OPLUS_CHG_USB_TYPE_APPLE_BRICK_ID, TRACK_POWER_10000MW, "dcp" },
	{ OPLUS_CHG_USB_TYPE_PD_SDP, TRACK_POWER_10000MW, "pd_sdp" },
	{ OPLUS_CHG_USB_TYPE_PD, TRACK_POWER_18000MW, "pd" },
	{ OPLUS_CHG_USB_TYPE_PD_DRP, TRACK_POWER_18000MW, "pd" },
	{ OPLUS_CHG_USB_TYPE_QC2, TRACK_POWER_18000MW, "qc" },
	{ OPLUS_CHG_USB_TYPE_QC3, TRACK_POWER_18000MW, "qc" },
	{ OPLUS_CHG_USB_TYPE_PD_PPS, TRACK_POWER_30000MW, "pps" },
	{ OPLUS_CHG_USB_TYPE_VOOC, TRACK_POWER_20000MW, "vooc" },
	{ OPLUS_CHG_USB_TYPE_SVOOC, 0, "svooc" },
	{ OPLUS_CHG_USB_TYPE_UFCS, TRACK_POWER_10000MW, "ufcs" },
};

static struct oplus_chg_track_vbus_vol_type wired_vbus_vol_type_table[] = {
	{ OPLUS_VBUS_12V, OPLUS_CHG_USB_TYPE_PD, TRACK_POWER_36000MW, "pd12v" },
	{ OPLUS_VBUS_12V, OPLUS_CHG_USB_TYPE_PD_DRP, TRACK_POWER_36000MW, "pd12v" },
	{ OPLUS_VBUS_12V, OPLUS_CHG_USB_TYPE_QC2, TRACK_POWER_36000MW, "qc12v" },
	{ OPLUS_VBUS_12V, OPLUS_CHG_USB_TYPE_QC3, TRACK_POWER_36000MW, "qc12v" },
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static struct oplus_chg_track_type wls_adapter_type_table[] = {
	{ OPLUS_CHG_WLS_UNKNOWN, TRACK_POWER_5000MW, "unknow" },
	{ OPLUS_CHG_WLS_VOOC, TRACK_POWER_20000MW, "airvooc" },
	{ OPLUS_CHG_WLS_BPP, TRACK_POWER_5000MW, "bpp" },
	{ OPLUS_CHG_WLS_EPP, TRACK_POWER_10000MW, "epp" },
	{ OPLUS_CHG_WLS_EPP_PLUS, TRACK_POWER_10000MW, "epp" },
	{ OPLUS_CHG_WLS_SVOOC, TRACK_POWER_50000MW, "airsvooc" },
	{ OPLUS_CHG_WLS_PD_65W, TRACK_POWER_50000MW, "airsvooc" },
};
#else
static struct oplus_chg_track_type wls_adapter_type_table[] = {
	{ TRACK_WLS_ADAPTER_TYPE_UNKNOWN, TRACK_POWER_5000MW, "unknow" },
	{ TRACK_WLS_ADAPTER_TYPE_VOOC, TRACK_POWER_20000MW, "airvooc" },
	{ TRACK_WLS_ADAPTER_TYPE_SVOOC, TRACK_POWER_50000MW, "airsvooc" },
	{ TRACK_WLS_ADAPTER_TYPE_USB, TRACK_POWER_5000MW, "bpp" },
	{ TRACK_WLS_ADAPTER_TYPE_NORMAL, TRACK_POWER_5000MW, "bpp" },
	{ TRACK_WLS_ADAPTER_TYPE_EPP, TRACK_POWER_10000MW, "epp" },
	{ TRACK_WLS_ADAPTER_TYPE_SVOOC_50W, TRACK_POWER_50000MW, "airsvooc" },
	{ TRACK_WLS_ADAPTER_TYPE_PD_65W, TRACK_POWER_50000MW, "airsvooc" },
};
#endif

static struct oplus_chg_track_type wls_dock_type_table[] = {
	{ TRACK_WLS_DOCK_MODEL_0, TRACK_POWER_30000MW, "model_0" },
	{ TRACK_WLS_DOCK_MODEL_1, TRACK_POWER_40000MW, "model_1" },
	{ TRACK_WLS_DOCK_MODEL_2, TRACK_POWER_50000MW, "model_2" },
	{ TRACK_WLS_DOCK_MODEL_3, TRACK_POWER_50000MW, "model_3" },
	{ TRACK_WLS_DOCK_MODEL_4, TRACK_POWER_50000MW, "model_4" },
	{ TRACK_WLS_DOCK_MODEL_5, TRACK_POWER_50000MW, "model_5" },
	{ TRACK_WLS_DOCK_MODEL_6, TRACK_POWER_50000MW, "model_6" },
	{ TRACK_WLS_DOCK_MODEL_7, TRACK_POWER_50000MW, "model_7" },
	{ TRACK_WLS_DOCK_MODEL_8, TRACK_POWER_50000MW, "model_8" },
	{ TRACK_WLS_DOCK_MODEL_9, TRACK_POWER_50000MW, "model_9" },
	{ TRACK_WLS_DOCK_MODEL_A, TRACK_POWER_100000MW, "model_a" },
	{ TRACK_WLS_DOCK_MODEL_B, TRACK_POWER_100000MW, "model_b" },
	{ TRACK_WLS_DOCK_MODEL_10, TRACK_POWER_100000MW, "model_10" },
	{ TRACK_WLS_DOCK_MODEL_11, TRACK_POWER_30000MW, "model_11" },
	{ TRACK_WLS_DOCK_MODEL_12, TRACK_POWER_30000MW, "model_12" },
	{ TRACK_WLS_DOCK_MODEL_13, TRACK_POWER_30000MW, "model_13" },
	{ TRACK_WLS_DOCK_THIRD_PARTY, TRACK_POWER_50000MW, "third_party" },
};

static struct oplus_chg_track_vooc_type vooc_type_table[] = {
	{ 0x01, TRACK_INPUT_VOL_5000MV, TRACK_INPUT_CURR_4000MA, "vooc" },
	{ 0x13, TRACK_INPUT_VOL_5000MV, TRACK_INPUT_CURR_4000MA, "vooc" },
	{ 0x15, TRACK_INPUT_VOL_5000MV, TRACK_INPUT_CURR_4000MA, "vooc" },
	{ 0x16, TRACK_INPUT_VOL_5000MV, TRACK_INPUT_CURR_4000MA, "vooc" },
	{ 0x34, TRACK_INPUT_VOL_5000MV, TRACK_INPUT_CURR_4000MA, "vooc" },
	{ 0x45, TRACK_INPUT_VOL_5000MV, TRACK_INPUT_CURR_4000MA, "vooc" },

	{ 0x17, TRACK_INPUT_VOL_5000MV, TRACK_INPUT_CURR_6000MA, "vooc" },
	{ 0x18, TRACK_INPUT_VOL_5000MV, TRACK_INPUT_CURR_6000MA, "vooc" },
	{ 0x19, TRACK_INPUT_VOL_5000MV, TRACK_INPUT_CURR_6000MA, "vooc" },
	{ 0x29, TRACK_INPUT_VOL_5000MV, TRACK_INPUT_CURR_6000MA, "vooc" },
	{ 0x41, TRACK_INPUT_VOL_5000MV, TRACK_INPUT_CURR_6000MA, "vooc" },
	{ 0x42, TRACK_INPUT_VOL_5000MV, TRACK_INPUT_CURR_6000MA, "vooc" },
	{ 0x43, TRACK_INPUT_VOL_5000MV, TRACK_INPUT_CURR_6000MA, "vooc" },
	{ 0x44, TRACK_INPUT_VOL_5000MV, TRACK_INPUT_CURR_6000MA, "vooc" },
	{ 0x46, TRACK_INPUT_VOL_5000MV, TRACK_INPUT_CURR_6000MA, "vooc" },

	{ 0x1A, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_3000MA, "svooc" },
	{ 0x1B, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_3000MA, "svooc" },
	{ 0x49, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_3000MA, "svooc" },
	{ 0x4A, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_3000MA, "svooc" },
	{ 0x61, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_3000MA, "svooc" },

	{ 0x1C, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_4100MA, "svooc" },
	{ 0x1D, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_4000MA, "svooc" },
	{ 0x1E, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_4000MA, "svooc" },
	{ 0x22, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_4000MA, "svooc" },

	{ 0x11, TRACK_INPUT_VOL_10000MV, TRACK_INPUT_CURR_5000MA, "svooc" },
	{ 0x12, TRACK_INPUT_VOL_10000MV, TRACK_INPUT_CURR_5000MA, "svooc" },
	{ 0x21, TRACK_INPUT_VOL_10000MV, TRACK_INPUT_CURR_5000MA, "svooc" },
	{ 0x23, TRACK_INPUT_VOL_10000MV, TRACK_INPUT_CURR_5000MA, "svooc" },
	{ 0x31, TRACK_INPUT_VOL_10000MV, TRACK_INPUT_CURR_5000MA, "svooc" },
	{ 0x33, TRACK_INPUT_VOL_10000MV, TRACK_INPUT_CURR_5000MA, "svooc" },
	{ 0x62, TRACK_INPUT_VOL_10000MV, TRACK_INPUT_CURR_5000MA, "svooc" },

	{ 0x24, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_5000MA, "svooc" },
	{ 0x25, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_5000MA, "svooc" },
	{ 0x26, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_5000MA, "svooc" },
	{ 0X27, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_5000MA, "svooc" },

	{ 0x14, TRACK_INPUT_VOL_10000MV, TRACK_INPUT_CURR_6500MA, "svooc" },
	{ 0x28, TRACK_INPUT_VOL_10000MV, TRACK_INPUT_CURR_6500MA, "svooc" },
	{ 0x2A, TRACK_INPUT_VOL_10000MV, TRACK_INPUT_CURR_6500MA, "svooc" },
	{ 0x35, TRACK_INPUT_VOL_10000MV, TRACK_INPUT_CURR_6500MA, "svooc" },
	{ 0x63, TRACK_INPUT_VOL_10000MV, TRACK_INPUT_CURR_6500MA, "svooc" },
	{ 0x66, TRACK_INPUT_VOL_10000MV, TRACK_INPUT_CURR_6500MA, "svooc" },
	{ 0x6E, TRACK_INPUT_VOL_10000MV, TRACK_INPUT_CURR_6500MA, "svooc" },

	{ 0x2B, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_6000MA, "svooc" },
	{ 0x36, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_6000MA, "svooc" },
	{ 0x64, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_6000MA, "svooc" },

	{ 0x2C, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_6100MA, "svooc" },
	{ 0x2D, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_6100MA, "svooc" },
	{ 0x2E, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_6100MA, "svooc" },
	{ 0x6C, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_6100MA, "svooc" },
	{ 0x6D, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_6100MA, "svooc" },

	{ 0x4B, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_7300MA, "svooc" },
	{ 0x4C, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_7300MA, "svooc" },
	{ 0x4D, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_7300MA, "svooc" },
	{ 0x4E, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_7300MA, "svooc" },
	{ 0x65, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_7300MA, "svooc" },

	{ 0x37, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_8000MA, "svooc" },
	{ 0x38, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_8000MA, "svooc" },
	{ 0x39, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_8000MA, "svooc" },
	{ 0x3A, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_8000MA, "svooc" },

	{ 0x3B, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_9100MA, "svooc" },
	{ 0x3C, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_9100MA, "svooc" },
	{ 0x3D, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_9100MA, "svooc" },
	{ 0x3E, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_9100MA, "svooc" },
	{ 0x69, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_9100MA, "svooc" },
	{ 0x6A, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_9100MA, "svooc" },

	{ 0x32, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_11000MA, "svooc" },
	{ 0x47, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_11000MA, "svooc" },
	{ 0x48, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_11000MA, "svooc" },
	{ 0x6B, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_11000MA, "svooc" },

	{ 0x51, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_11400MA, "svooc" },
	{ 0x67, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_11400MA, "svooc" },
	{ 0x68, TRACK_INPUT_VOL_11000MV, TRACK_INPUT_CURR_11400MA, "svooc" },
};

static struct oplus_chg_track_batt_full_reason batt_full_reason_table[] = {
	{ NOTIFY_BAT_FULL, "full_normal" },
	{ NOTIFY_BAT_FULL_PRE_LOW_TEMP, "full_low_temp" },
	{ NOTIFY_BAT_FULL_PRE_HIGH_TEMP, "full_high_temp" },
	{ NOTIFY_BAT_FULL_THIRD_BATTERY, "full_third_batt" },
	{ NOTIFY_BAT_NOT_CONNECT, "full_no_batt" },
};

static struct oplus_chg_track_chg_abnormal_reason chg_abnormal_reason_table[] = {
	{ NOTIFY_CHGING_OVERTIME, "chg_over_time", 0 },
	{ NOTIFY_CHARGER_OVER_VOL, "chg_over_vol", 0 },
	{ NOTIFY_CHARGER_LOW_VOL, "chg_low_vol", 0 },
	{ NOTIFY_BAT_OVER_TEMP, "batt_over_temp", 0 },
	{ NOTIFY_BAT_LOW_TEMP, "batt_low_temp", 0 },
	{ NOTIFY_BAT_OVER_VOL, "batt_over_vol", 0 },
	{ NOTIFY_BAT_NOT_CONNECT, "batt_no_conn", 0 },
	{ NOTIFY_BAT_FULL_THIRD_BATTERY, "batt_no_auth", 0 },
};

static struct oplus_chg_track_cool_down_stats cool_down_stats_table[] = {
	{ 0, 0, "L_0" },   { 1, 0, "L_1" },   { 2, 0, "L_2" },
	{ 3, 0, "L_3" },   { 4, 0, "L_4" },   { 5, 0, "L_5" },
	{ 6, 0, "L_6" },   { 7, 0, "L_7" },   { 8, 0, "L_8" },
	{ 9, 0, "L_9" },   { 10, 0, "L_10" }, { 11, 0, "L_11" },
	{ 12, 0, "L_12" }, { 13, 0, "L_13" }, { 14, 0, "L_14" },
	{ 15, 0, "L_15" }, { 16, 0, "L_16" }, { 17, 0, "L_17" },
	{ 18, 0, "L_18" }, { 19, 0, "L_19" }, { 20, 0, "L_20" },
	{ 21, 0, "L_21" }, { 22, 0, "L_22" }, { 23, 0, "L_23" },
	{ 24, 0, "L_24" }, { 25, 0, "L_25" }, { 26, 0, "L_26" },
	{ 27, 0, "L_27" }, { 28, 0, "L_28" }, { 29, 0, "L_29" },
	{ 30, 0, "L_30" }, { 31, 0, "L_31" },
};

static struct oplus_chg_track_fastchg_break mcu_voocphy_break_table[] = {
	{ TRACK_MCU_VOOCPHY_FAST_ABSENT, "absent" },
	{ TRACK_MCU_VOOCPHY_BAD_CONNECTED, "bad_connect" },
	{ TRACK_MCU_VOOCPHY_BTB_TEMP_OVER, "btb_temp_over" },
	{ TRACK_MCU_VOOCPHY_DATA_ERROR, "data_error" },
	{ TRACK_MCU_VOOCPHY_OTHER, "other" },
	{ TRACK_MCU_VOOCPHY_HEAD_ERROR, "head_error" },
	{ TRACK_MCU_VOOCPHY_ADAPTER_FW_UPDATE, "adapter_fw_update" },
	{ TRACK_MCU_VOOCPHY_CURR_OVER_ERROR, "curr_over_error" },
	{ TRACK_MCU_VOOCPHY_ADAPTER_COPYCAT, "adapter_copycat" },
};

static struct oplus_chg_track_fastchg_break adsp_voocphy_break_table[] = {
	{ TRACK_ADSP_VOOCPHY_BAD_CONNECTED, "bad_connect" },
	{ TRACK_ADSP_VOOCPHY_FRAME_H_ERR, "frame_head_error" },
	{ TRACK_ADSP_VOOCPHY_CLK_ERR, "clk_error" },
	{ TRACK_ADSP_VOOCPHY_HW_VBATT_HIGH, "hw_vbatt_high" },
	{ TRACK_ADSP_VOOCPHY_HW_TBATT_HIGH, "hw_ibatt_high" },
	{ TRACK_ADSP_VOOCPHY_COMMU_TIME_OUT, "commu_time_out" },
	{ TRACK_ADSP_VOOCPHY_ADAPTER_COPYCAT, "adapter_copycat" },
	{ TRACK_ADSP_VOOCPHY_BTB_TEMP_OVER, "btb_temp_over" },
	{ TRACK_ADSP_VOOCPHY_OTHER, "other" },
};

static struct oplus_chg_track_fastchg_break ap_voocphy_break_table[] = {
	{ TRACK_CP_VOOCPHY_FAST_ABSENT, "absent" },
	{ TRACK_CP_VOOCPHY_BAD_CONNECTED, "bad_connect" },
	{ TRACK_CP_VOOCPHY_FRAME_H_ERR, "frame_head_error" },
	{ TRACK_CP_VOOCPHY_BTB_TEMP_OVER, "btb_temp_over" },
	{ TRACK_CP_VOOCPHY_COMMU_TIME_OUT, "commu_time_out" },
	{ TRACK_CP_VOOCPHY_ADAPTER_COPYCAT, "adapter_copycat" },
	{ TRACK_CP_VOOCPHY_ADAPTER_ABNORMAL, "adapter_abnormal" },
	{ TRACK_CP_VOOCPHY_OTHER, "other" },
};

static struct oplus_chg_track_voocphy_info voocphy_info_table[] = {
	{ TRACK_NO_VOOCPHY, "unknow" },
	{ TRACK_ADSP_VOOCPHY, "adsp" },
	{ TRACK_AP_SINGLE_CP_VOOCPHY, "ap" },
	{ TRACK_AP_DUAL_CP_VOOCPHY, "ap" },
	{ TRACK_MCU_VOOCPHY, "mcu" },
};

const char* const high_reverse_err_name_str[] = {
	[HPR_NO_VOTER_DISABLE]		= "NO_VOTER",
	[HPR_USER_VOTER_DISABLE]	= "USER_VOTER",
	[HPR_BATT_TEMP_VOTER_DISABLE]	= "BATT_TEMP_VOTER",
	[HPR_BATT_SOC_VOTER_DISABLE]	= "BATT_SOC_VOTER",
	[HPR_CURR_ERR_VOTER_DISABLE]	= "CURR_ERR_VOTER",
	[HPR_AUTH_VOTER_DISABLE]	= "AUTH_VOTER",
	[HPR_SVID_VOTER_DISABLE]	= "SVID_VOTER",
	[HPR_USB_VOTER_DISABLE]		= "USBTEMP_VOTER",
	[HPR_HW_ERR_VOTER_DISABLE]	= "HARD_RESET_VOTER",
	[HPR_DEVICE_ERROR_DISABLE]	= "DEVICE_ERROR_VOTER",
	[HPR_UNDEFINED_VOTER_DISABLE]	= "UNDEFINED_VOTER",
};

const char *get_high_reverse_err_name_str(int flag)
{
	if (flag < 0 || flag >= HPR_UNDEFINED_VOTER_DISABLE)
		return "UNDEFINED_VOTER";
	return high_reverse_err_name_str[flag];
}
static int oplus_chg_track_get_current_time(struct rtc_time *tm);

static struct oplus_chg_track_gague_err_reason gague_err_reason_table[] = {
	{ TRACK_GAGUE_ERR_SEAL, "seal_fail" },
	{ TRACK_GAGUE_ERR_UNSEAL, "unseal_fail" },
	{ TRACK_GAGUE_GENERAL_INFO, "general_info" },
	{ TRACK_GAGUE_ERR_RSOC_JUMP, "rsoc_jump" },
	{ TRACK_GAGUE_ERR_VOLT_SOC_NOT_MATCH, "volt_soc_not_match" },
	{ TRACK_GAGUE_ERR_QMAX, "qmax_err" },
	{ TRACK_GAGUE_ERR_FCC, "fcc_err" },
	{ TRACK_GAGUE_ERR_RSOC_SMOOTH, "rsoc_smooth_err" },
	{ TRACK_GAGUE_ERR_TEMP, "temp_err" },
	{ TRACK_GAGUE_ERR_SOH_JUMP, "soh_jump" },
	{ TRACK_GAGUE_ERR_CC_JUMP, "cc_jump" },
	{ TRACK_GAGUE_ERR_SINGLE_DISCHG_TERM_VOLT, "single_dischg_term_volt" },
	{ TRACK_GAGUE_ERR_MULTIPLE_DISCHG_TERM_VOLT, "multiple_dischg_term_volt" },
	{ TRACK_GAGUE_ERR_CC_TERM_VOLT, "cc_term_volt" },
	{ TRACK_GAGUE_ERR_BELOW_FIRMWARE_TERM_VOLT, "below_firmware_term_volt" },
	{ TRACK_GAGUE_ERR_LIFETIME_OVER, "lifetime_over" },
	{ TRACK_GAGUE_MTK_CALI_INFO, "mtk_cali_info"},
	{ TRACK_GAGUE_ERR_SUB_BTB_CONNECT, "sub_btb_connect_state"},
	{ TRACK_GAGUE_BATT_MONITOR_BY_SOC, "batt_monitor_by_soc"},
	{ TRACK_GAGUE_BATT_MONITOR_BY_TIME, "batt_monitor_by_time"},
	{ TRACK_GAGUE_QCOM_CALI_INFO, "qcom_cali_info"},
};

static struct oplus_chg_track_speed_ref wired_series_double_cell_125w_150w[] = {
	{ TRACK_REF_SOC_50, TRACK_POWER_20000MW,
	  TRACK_POWER_20000MW * 1000 / TRACK_REF_VOL_10000MV },
	{ TRACK_REF_SOC_75, TRACK_POWER_18000MW,
	  TRACK_POWER_18000MW * 1000 / TRACK_REF_VOL_10000MV },
	{ TRACK_REF_SOC_90, TRACK_POWER_12000MW,
	  TRACK_POWER_12000MW * 1000 / TRACK_REF_VOL_10000MV },
};

static struct oplus_chg_track_speed_ref
	wired_series_double_cell_65w_80w_100w[] = {
		{ TRACK_REF_SOC_50, TRACK_POWER_15000MW,
		  TRACK_POWER_15000MW * 1000 / TRACK_REF_VOL_10000MV },
		{ TRACK_REF_SOC_75, TRACK_POWER_15000MW,
		  TRACK_POWER_15000MW * 1000 / TRACK_REF_VOL_10000MV },
		{ TRACK_REF_SOC_90, TRACK_POWER_12000MW,
		  TRACK_POWER_12000MW * 1000 / TRACK_REF_VOL_10000MV },
	};

static struct oplus_chg_track_speed_ref wired_equ_single_cell_60w_67w[] = {
	{ TRACK_REF_SOC_50, TRACK_POWER_15000MW,
	  TRACK_POWER_15000MW * 1000 / TRACK_REF_VOL_5000MV },
	{ TRACK_REF_SOC_75, TRACK_POWER_15000MW,
	  TRACK_POWER_15000MW * 1000 / TRACK_REF_VOL_5000MV },
	{ TRACK_REF_SOC_90, TRACK_POWER_12000MW,
	  TRACK_POWER_12000MW * 1000 / TRACK_REF_VOL_5000MV },
};

static struct oplus_chg_track_speed_ref wired_equ_single_cell_30w_33w[] = {
	{ TRACK_REF_SOC_50, TRACK_POWER_15000MW,
	  TRACK_POWER_15000MW * 1000 / TRACK_REF_VOL_5000MV },
	{ TRACK_REF_SOC_75, TRACK_POWER_15000MW,
	  TRACK_POWER_15000MW * 1000 / TRACK_REF_VOL_5000MV },
	{ TRACK_REF_SOC_90, TRACK_POWER_12000MW,
	  TRACK_POWER_12000MW * 1000 / TRACK_REF_VOL_5000MV },
};

static struct oplus_chg_track_speed_ref wired_single_cell_18w[] = {
	{ TRACK_REF_SOC_50, TRACK_POWER_10000MW,
	  TRACK_POWER_10000MW * 1000 / TRACK_REF_VOL_5000MV },
	{ TRACK_REF_SOC_75, TRACK_POWER_7000MW,
	  TRACK_POWER_7000MW * 1000 / TRACK_REF_VOL_5000MV },
	{ TRACK_REF_SOC_90, TRACK_POWER_6000MW,
	  TRACK_POWER_6000MW * 1000 / TRACK_REF_VOL_5000MV },
};

static struct oplus_chg_track_speed_ref wls_series_double_cell_40w_45w_50w[] = {
	{ TRACK_REF_SOC_50, TRACK_POWER_15000MW,
	  TRACK_POWER_15000MW * 1000 / TRACK_REF_VOL_10000MV },
	{ TRACK_REF_SOC_75, TRACK_POWER_15000MW,
	  TRACK_POWER_15000MW * 1000 / TRACK_REF_VOL_10000MV },
	{ TRACK_REF_SOC_90, TRACK_POWER_12000MW,
	  TRACK_POWER_12000MW * 1000 / TRACK_REF_VOL_10000MV },
};

static struct oplus_chg_track_speed_ref wls_series_double_cell_20w_30w[] = {
	{ TRACK_REF_SOC_50, TRACK_POWER_10000MW,
	  TRACK_POWER_10000MW * 1000 / TRACK_REF_VOL_10000MV },
	{ TRACK_REF_SOC_75, TRACK_POWER_10000MW,
	  TRACK_POWER_10000MW * 1000 / TRACK_REF_VOL_10000MV },
	{ TRACK_REF_SOC_90, TRACK_POWER_10000MW,
	  TRACK_POWER_10000MW * 1000 / TRACK_REF_VOL_10000MV },
};

static struct oplus_chg_track_speed_ref wls_series_double_cell_12w_15w[] = {
	{ TRACK_REF_SOC_50, TRACK_POWER_5000MW,
	  TRACK_POWER_5000MW * 1000 / TRACK_REF_VOL_10000MV },
	{ TRACK_REF_SOC_75, TRACK_POWER_5000MW,
	  TRACK_POWER_5000MW * 1000 / TRACK_REF_VOL_10000MV },
	{ TRACK_REF_SOC_90, TRACK_POWER_5000MW,
	  TRACK_POWER_5000MW * 1000 / TRACK_REF_VOL_10000MV },
};

static struct oplus_chg_track_speed_ref wls_equ_single_cell_12w_15w[] = {
	{ TRACK_REF_SOC_50, TRACK_POWER_5000MW,
	  TRACK_POWER_5000MW * 1000 / TRACK_REF_VOL_5000MV },
	{ TRACK_REF_SOC_75, TRACK_POWER_5000MW,
	  TRACK_POWER_5000MW * 1000 / TRACK_REF_VOL_5000MV },
	{ TRACK_REF_SOC_90, TRACK_POWER_5000MW,
	  TRACK_POWER_5000MW * 1000 / TRACK_REF_VOL_5000MV },
};

static struct oplus_chg_track_speed_ref wls_series_double_cell_epp15w_epp10w[] = {
	{ TRACK_REF_SOC_50, TRACK_POWER_5000MW,
	  TRACK_POWER_5000MW * 1000 / TRACK_REF_VOL_10000MV },
	{ TRACK_REF_SOC_75, TRACK_POWER_5000MW,
	  TRACK_POWER_5000MW * 1000 / TRACK_REF_VOL_10000MV },
	{ TRACK_REF_SOC_90, TRACK_POWER_5000MW,
	  TRACK_POWER_5000MW * 1000 / TRACK_REF_VOL_10000MV },
};

static struct oplus_chg_track_speed_ref wls_equ_single_cell_epp15w_epp10w[] = {
	{ TRACK_REF_SOC_50, TRACK_POWER_5000MW,
	  TRACK_POWER_5000MW * 1000 / TRACK_REF_VOL_5000MV },
	{ TRACK_REF_SOC_75, TRACK_POWER_5000MW,
	  TRACK_POWER_5000MW * 1000 / TRACK_REF_VOL_5000MV },
	{ TRACK_REF_SOC_90, TRACK_POWER_5000MW,
	  TRACK_POWER_5000MW * 1000 / TRACK_REF_VOL_5000MV },
};

static struct oplus_chg_track_speed_ref wls_series_double_cell_bpp[] = {
	{ TRACK_REF_SOC_50, TRACK_POWER_2000MW,
	  TRACK_POWER_2000MW * 1000 / TRACK_REF_VOL_10000MV },
	{ TRACK_REF_SOC_75, TRACK_POWER_2000MW,
	  TRACK_POWER_2000MW * 1000 / TRACK_REF_VOL_10000MV },
	{ TRACK_REF_SOC_90, TRACK_POWER_2500MW,
	  TRACK_POWER_2000MW * 1000 / TRACK_REF_VOL_10000MV },
};

static struct oplus_chg_track_speed_ref wls_equ_single_cell_bpp[] = {
	{ TRACK_REF_SOC_50, TRACK_POWER_2000MW,
	  TRACK_POWER_2000MW * 1000 / TRACK_REF_VOL_5000MV },
	{ TRACK_REF_SOC_75, TRACK_POWER_2000MW,
	  TRACK_POWER_2000MW * 1000 / TRACK_REF_VOL_5000MV },
	{ TRACK_REF_SOC_90, TRACK_POWER_2000MW,
	  TRACK_POWER_2000MW * 1000 / TRACK_REF_VOL_5000MV },
};

/*
 * digital represents wls bpp charging scheme
 * 0: wls 5w series dual cell bpp charging scheme
 * 1: wls 5w single cell bpp charging scheme
 * 2: wls 5w parallel double cell bpp charging scheme
 */
static struct oplus_chg_track_speed_ref *g_wls_bpp_speed_ref_standard[] = {
	wls_series_double_cell_bpp,
	wls_equ_single_cell_bpp,
	wls_equ_single_cell_bpp,
};

/*
 * digital represents wls epp charging scheme
 * 0: wls 10w or 15w series dual cell epp charging scheme
 * 1: wls 10w or 15w single cell epp charging scheme
 * 2: wls 10w or 15w parallel double cell epp charging scheme
 */
static struct oplus_chg_track_speed_ref *g_wls_epp_speed_ref_standard[] = {
	wls_series_double_cell_epp15w_epp10w,
	wls_equ_single_cell_epp15w_epp10w,
	wls_equ_single_cell_epp15w_epp10w,
};

/*
 * digital represents wls fast charging scheme
 * 0: wls 40w or 45w or 50w series dual cell charging scheme
 * 1: wls 20w or 30w series dual cell charging scheme
 * 2: wls 12w or 15w series dual cell charging scheme
 * 3: wls 12w or 15w parallel double cell charging scheme
 */
static struct oplus_chg_track_speed_ref* g_wls_fast_speed_ref_standard[] = {
	wls_series_double_cell_40w_45w_50w,
	wls_series_double_cell_20w_30w,
	wls_series_double_cell_12w_15w,
	wired_equ_single_cell_60w_67w,
	wls_equ_single_cell_12w_15w,
	wls_equ_single_cell_12w_15w,
};

/*
 * digital represents wired fast charging scheme
 * 0: wired 120w or 150w series dual cell charging scheme
 * 1: wired 65w or 80w and 100w series dual cell charging scheme
 * 2: wired 60w or 67w single cell charging scheme
 * 3: wired 60w or 67w parallel double cell charging scheme
 * 4: wired 30w or 33w single cell charging scheme
 * 5: wired 30w or 33w parallel double cell charging scheme
 * 6: wired 18w single cell charging scheme
 */
static struct oplus_chg_track_speed_ref *g_wired_speed_ref_standard[] = {
	wired_series_double_cell_125w_150w,
	wired_series_double_cell_65w_80w_100w,
	wired_equ_single_cell_60w_67w,
	wired_equ_single_cell_60w_67w,
	wired_equ_single_cell_30w_33w,
	wired_equ_single_cell_30w_33w,
	wired_single_cell_18w,
};

static struct oplus_chg_track_app_ref app_table[] = {
	{"T01", "pkg_01", 0},
	{"T02", "pkg_02", 0},
	{"T03", "pkg_03", 0},
	{"T04", "pkg_04", 0},
	{"T05", "pkg_05", 0},
	{"T06", "pkg_06", 0},
	{"T07", "pkg_07", 0},
	{"T08", "pkg_08", 0},
	{"T09", "pkg_09", 0},
	{"T10", "pkg_10", 0},
	{"T11", "pkg_11", 0},
	{"T12", "pkg_12", 0},
	{"T13", "pkg_13", 0},
	{"T14", "pkg_14", 0},
	{"T15", "pkg_15", 0},
	{"T16", "pkg_16", 0},
	{"T17", "pkg_17", 0},
	{"T18", "pkg_18", 0},
	{"T19", "pkg_19", 0},
	{"T20", "pkg_20", 0},
	{"T21", "pkg_21", 0},
	{"T22", "pkg_22", 0},
	{"T23", "pkg_23", 0},
	{"T24", "pkg_24", 0},
	{"T25", "pkg_25", 0},
	{"T26", "pkg_26", 0},
	{"T27", "pkg_27", 0},
	{"T28", "pkg_28", 0},
	{"T29", "pkg_29", 0},
	{"T30", "pkg_30", 0},
	{"T31", "pkg_31", 0},
	{"T32", "pkg_32", 0},
	{"T33", "pkg_33", 0},
	{"T34", "pkg_34", 0},
	{"T35", "pkg_35", 0},
	{"T36", "pkg_36", 0},
	{"T37", "pkg_37", 0},
	{"T38", "pkg_38", 0},
	{"T39", "pkg_39", 0},
	{"T40", "pkg_40", 0},
	{"T41", "pkg_41", 0},
	{"T42", "pkg_42", 0},
	{"T43", "pkg_43", 0},
	{"T44", "pkg_44", 0},
	{"T45", "pkg_45", 0},
	{"T46", "pkg_46", 0},
	{"T47", "pkg_47", 0},
	{"T48", "pkg_48", 0},
	{"T49", "pkg_49", 0},
	{"T50", "pkg_50", 0},
	{"Txx", "pkg_xx", 0},
};

__maybe_unused static bool is_fv_votable_available(struct oplus_monitor *chip)
{
	if (!chip->fv_votable)
		chip->fv_votable = find_votable("FV_MIN");
	return !!chip->fv_votable;
}

__maybe_unused static bool is_wls_icl_votable_available(struct oplus_monitor *chip)
{
	if (!chip->wls_icl_votable)
		chip->wls_icl_votable = find_votable("WLS_NOR_ICL");
	return !!chip->wls_icl_votable;
}

__maybe_unused static bool is_wls_fcc_votable_available(struct oplus_monitor *chip)
{
	if (!chip->wls_fcc_votable)
		chip->wls_fcc_votable = find_votable("WLS_FCC");
	return !!chip->wls_fcc_votable;
}

__maybe_unused static bool is_pps_topic_available(struct oplus_monitor *chip)
{
	if (!chip->pps_topic)
		chip->pps_topic = oplus_mms_get_by_name("pps");
	return !!chip->pps_topic;
}

__maybe_unused static bool is_wls_rx_disable_votable_available(struct oplus_monitor *chip)
{
	if (!chip->wls_rx_disable_votable)
		chip->wls_rx_disable_votable = find_votable("WLS_RX_DISABLE");
	return !!chip->wls_rx_disable_votable;
}

static int oplus_chg_track_pack_app_stats(u8 *curx, int *index)
{
	int i;
	int record_index = 0;
	int second_index = 0;
	u32 max_time = 0;

	if (!curx || !index)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(app_table) - 1; i++) {
		if (app_table[i].cont_time >= max_time) {
			second_index = record_index;
			record_index = i;
			max_time = app_table[i].cont_time;
		} else if (second_index == record_index ||
				app_table[i].cont_time > app_table[second_index].cont_time) {
			second_index = i;
		}
	}

	*index += scnprintf(&(curx[*index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - *index,
		"$$ledon_app@@%s,%d;%s,%d;%s,%d",
		app_table[record_index].alias_name, app_table[record_index].cont_time,
		app_table[second_index].alias_name, app_table[second_index].cont_time,
		app_table[ARRAY_SIZE(app_table) - 1].alias_name,
		app_table[ARRAY_SIZE(app_table) - 1].cont_time);

	return 0;
}

static void oplus_chg_track_clear_app_time(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(app_table); i++)
		app_table[i].cont_time = 0;
}

static int oplus_chg_track_match_app_info(u8 *app_name)
{
	int i;

	if (!app_name)
		return TRACK_APP_TOP_INDEX_DEFAULT;

	for (i = 0; i < ARRAY_SIZE(app_table); i++) {
		if (!strcmp(app_table[i].real_name, app_name))
			return i;
	}

	return TRACK_APP_TOP_INDEX_DEFAULT;
}

int oplus_chg_track_set_app_info(const char *buf)
{
	struct oplus_chg_track *track_chip = g_track_chip;
	struct oplus_chg_track_app_status *p_app_status;
	struct oplus_monitor *monitor;
	bool charger_exist;

	if (!track_chip || !buf)
		return -EINVAL;

	chg_debug("pkg_name:%s", buf);
	monitor = track_chip->monitor;
	charger_exist = monitor->wired_online || monitor->wls_online;
	p_app_status = &(track_chip->track_status.app_status);
	mutex_lock(&p_app_status->app_lock);
	if (charger_exist)
		strncpy(p_app_status->curr_top_name,
			buf, TRACK_APP_REAL_NAME_LEN - 1);
	mutex_unlock(&p_app_status->app_lock);
	return 0;
}

int oplus_chg_track_olc_config_set(const char *buf)
{
	struct oplus_chg_track_cfg *cfg_chip = NULL;
	char config_buf[OLC_CONFIG_SIZE] = { 0 };
	char *tmpbuf;
	int num = 0;
	int ret = 0;
	int len = 0;
	char *config = NULL;

	if (!g_track_chip)
		return -1;

	cfg_chip = &g_track_chip->track_cfg;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
	strscpy(config_buf, buf, OLC_CONFIG_SIZE);
#else
	strlcpy(config_buf, buf, OLC_CONFIG_SIZE);
#endif
	tmpbuf = config_buf;

	config = strsep(&tmpbuf, ",");
	while (config != NULL) {
		len = strlen(config);
		if (len > OLC_CONFIG_BIT_NUM) {	/* FFFFFFFFFFFFFFFF */
			chg_err("set wrong olc config\n");
			break;
		}

		ret = kstrtoull(config, 16, &cfg_chip->exception_data.olc_config[num]);
		if (ret < 0)
			chg_err("parse the %d olc config failed\n", num);
		else
			chg_info("set the %d olc config:%llx", num, cfg_chip->exception_data.olc_config[num]);
		num++;
		if (num >= OLC_CONFIG_NUM_MAX) {
			chg_err("set wrong olc config size\n");
			break;
		}
		config = strsep(&tmpbuf, ",");
	}

	return 0;
}

int oplus_chg_track_olc_config_get(char *buf)
{
	struct oplus_chg_track_cfg *cfg_chip = NULL;
	char tmpbuf[OLC_CONFIG_SIZE] = { 0 };
	int idx = 0;
	int num;
	int len = 0;

	if (!g_track_chip)
		return len;

	cfg_chip = &g_track_chip->track_cfg;

	for (num = 0; num < OLC_CONFIG_NUM_MAX; num++) {
		len = scnprintf(tmpbuf, OLC_CONFIG_SIZE - idx, "%llx,",
				cfg_chip->exception_data.olc_config[num]);
		memcpy(&buf[idx], tmpbuf, len);
		idx += len;
	}

	return (idx - 1);
}

int oplus_chg_track_time_zone_set(const char *buf)
{
	int val = 0;
	struct oplus_chg_track *track_chip;
	struct oplus_chg_track_status *track_status;

	track_chip = g_track_chip;
	track_status = &track_chip->track_status;

	if (!track_chip || !track_status)
		return -1;

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	track_status->track_gmtoff = val;

	return 0;
}

int oplus_chg_track_time_zone_get(char *buf)
{
	struct oplus_chg_track *track_chip;
	struct oplus_chg_track_status *track_status;

	track_chip = g_track_chip;
	if (!track_chip)
		return -1;

	track_status = &track_chip->track_status;
	if (!track_status)
		return -1;

	return sprintf(buf, "%d\n", track_status->track_gmtoff);
}

static int oplus_chg_track_set_hidl_bcc_info(struct oplus_chg_track_hidl_cmd *cmd, struct oplus_chg_track *track_chip)
{
	int len;
	struct oplus_chg_track_hidl_bcc_info_cmd *bcc_info_cmd;

	if (!cmd)
		return -EINVAL;

	if (cmd->data_size != sizeof(struct oplus_chg_track_hidl_bcc_info_cmd)) {
		chg_err("!!!size not match struct, ignore\n");
		return -EINVAL;
	}

	if (track_chip->track_status.bcc_info->count >= TRACK_HIDL_BCC_INFO_COUNT) {
		chg_err("!!!bcc record count has arrive max, ignore\n");
		return -EINVAL;
	}

	bcc_info_cmd = (struct oplus_chg_track_hidl_bcc_info_cmd *)(cmd->data_buf);
	track_chip->track_status.bcc_info->err_code |= bcc_info_cmd->err_code;
	track_chip->track_status.bcc_info->count++;
	len = strlen(track_chip->track_status.bcc_info->data_buf);
	if (!len)
		scnprintf(&(track_chip->track_status.bcc_info->data_buf[len]), TRACK_HIDL_BCC_INFO_LEN - len, "%s",
			 bcc_info_cmd->data_buf);
	else
		scnprintf(&(track_chip->track_status.bcc_info->data_buf[len]), TRACK_HIDL_BCC_INFO_LEN - len, ";%s",
			 bcc_info_cmd->data_buf);
	return 0;
}

static int oplus_chg_track_set_hidl_bcc_err(struct oplus_chg_track_hidl_cmd *cmd, struct oplus_chg_track *track_chip)
{
	struct oplus_chg_track_hidl_bcc_err_cmd *bcc_err_cmd;
	struct oplus_chg_track_hidl_bcc_err *bcc_err;

	if (!cmd)
		return -EINVAL;

	if (cmd->data_size != sizeof(struct oplus_chg_track_hidl_bcc_err_cmd)) {
		chg_err("!!!size not match struct, ignore\n");
		return -EINVAL;
	}

	bcc_err = &(track_chip->track_status.bcc_err);
	mutex_lock(&bcc_err->track_bcc_err_lock);
	if (bcc_err->bcc_err_uploading) {
		chg_debug("bcc_err_uploading, should return\n");
		mutex_unlock(&bcc_err->track_bcc_err_lock);
		return 0;
	}
	bcc_err_cmd = (struct oplus_chg_track_hidl_bcc_err_cmd *)(cmd->data_buf);
	memcpy(&bcc_err->bcc_err, bcc_err_cmd, sizeof(*bcc_err_cmd));
	mutex_unlock(&bcc_err->track_bcc_err_lock);

	schedule_delayed_work(&bcc_err->bcc_err_load_trigger_work, 0);
	return 0;
}

static int oplus_parallelchg_track_foldmode_info(
	struct oplus_chg_track_hidl_cmd *cmd, struct oplus_chg_track *track_chip)
{
	struct oplus_parallelchg_track_hidl_foldmode_info_cmd *parallelchg_info_cmd;
	struct oplus_parallelchg_track_hidl_foldmode_info *parallelchg_info_p;

	if (!cmd)
		return -EINVAL;

	if (cmd->data_size != strlen(cmd->data_buf)) {
		chg_err("!!!size not match struct, ignore\n");
		return -EINVAL;
	}

	parallelchg_info_p = &(track_chip->track_status.parallelchg_info);
	mutex_lock(&parallelchg_info_p->track_lock);
	if (parallelchg_info_p->info_uploading) {
		chg_info(" parallelchg_track_foldmode info uploading, should return\n");
		mutex_unlock(&parallelchg_info_p->track_lock);
		return 0;
	}
	parallelchg_info_cmd = (struct oplus_parallelchg_track_hidl_foldmode_info_cmd *)(cmd->data_buf);
	memcpy(&parallelchg_info_p->parallelchg_foldmode_info, parallelchg_info_cmd, sizeof(*parallelchg_info_cmd));
	mutex_unlock(&parallelchg_info_p->track_lock);

	schedule_delayed_work(&parallelchg_info_p->load_trigger_work, 0);
	return 0;
}

static int oplus_chg_track_set_hidl_ttf_info(
	struct oplus_chg_track_hidl_cmd *cmd, struct oplus_chg_track *track_chip)
{
	struct oplus_chg_track_hidl_ttf_info_cmd *ttf_info_cmd;
	struct oplus_chg_track_hidl_ttf_info *ttf_info_p;

	if (!cmd)
		return -EINVAL;

	if (cmd->data_size != sizeof(struct oplus_chg_track_hidl_ttf_info_cmd)) {
		chg_err("!!!size not match struct, ignore\n");
		return -EINVAL;
	}

	ttf_info_p = track_chip->track_status.ttf_info;
	if (IS_ERR_OR_NULL(ttf_info_p)) {
		chg_err("ttf_info_p is null\n");
		return -EINVAL;
	}
	mutex_lock(&ttf_info_p->track_lock);
	if (ttf_info_p->info_uploading) {
		chg_info("uploading, should return\n");
		mutex_unlock(&ttf_info_p->track_lock);
		return 0;
	}
	ttf_info_cmd = (struct oplus_chg_track_hidl_ttf_info_cmd *)(cmd->data_buf);
	memcpy(&ttf_info_p->ttf_info, ttf_info_cmd, sizeof(*ttf_info_cmd));
	mutex_unlock(&ttf_info_p->track_lock);

	schedule_delayed_work(&ttf_info_p->load_trigger_work, 0);
	return 0;
}

static void oplus_track_upload_ttf_info(struct work_struct *work)
{
	int index = 0;

	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track_hidl_ttf_info *ttf_info_p = container_of(
		dwork, struct oplus_chg_track_hidl_ttf_info, load_trigger_work);

	mutex_lock(&ttf_info_p->track_lock);
	if (ttf_info_p->info_uploading) {
		chg_info("ttf_info_uploading, should return\n");
		mutex_unlock(&ttf_info_p->track_lock);
		return;
	}

	if (ttf_info_p->load_trigger_info)
		kfree(ttf_info_p->load_trigger_info);
	ttf_info_p->load_trigger_info = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!ttf_info_p->load_trigger_info) {
		chg_err("ttf_info_load_trigger memory alloc fail\n");
		mutex_unlock(&ttf_info_p->track_lock);
		return;
	}
	ttf_info_p->load_trigger_info->type_reason = TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	ttf_info_p->load_trigger_info->flag_reason = TRACK_NOTIFY_FLAG_TTF_INFO;
	ttf_info_p->info_uploading = true;
	mutex_unlock(&ttf_info_p->track_lock);

	index += scnprintf(&(ttf_info_p->load_trigger_info->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			"%s", ttf_info_p->ttf_info.data_buf);

	oplus_chg_track_obtain_power_info(&(ttf_info_p->load_trigger_info->crux_info[index]),
					  OPLUS_CHG_TRACK_CURX_INFO_LEN - index);

	oplus_chg_track_upload_trigger_data(ttf_info_p->load_trigger_info);
	if (ttf_info_p->load_trigger_info) {
		kfree(ttf_info_p->load_trigger_info);
		ttf_info_p->load_trigger_info = NULL;
	}
	memset(&ttf_info_p->ttf_info, 0, sizeof(ttf_info_p->ttf_info));
	ttf_info_p->info_uploading = false;
	return;
}

static int oplus_chg_track_set_hidl_bdd_info(
	struct oplus_chg_track_hidl_cmd *cmd, struct oplus_chg_track *track_chip)
{
	struct oplus_chg_track_hidl_bdd_info_cmd *bdd_info_cmd;
	struct oplus_chg_track_hidl_bdd_info *bdd_info_p;

	if (!cmd)
		return -EINVAL;

	if (cmd->data_size != sizeof(struct oplus_chg_track_hidl_bdd_info_cmd)) {
		chg_err("!!!size not match struct, ignore\n");
		return -EINVAL;
	}

	bdd_info_p = &(track_chip->track_status.bdd_info_s);
	mutex_lock(&bdd_info_p->track_bdd_info_lock);
	if (bdd_info_p->bdd_info_uploading) {
		chg_err("bdd_info_uploading, should return\n");
		mutex_unlock(&bdd_info_p->track_bdd_info_lock);
		return 0;
	}
	bdd_info_cmd = (struct oplus_chg_track_hidl_bdd_info_cmd *)(cmd->data_buf);
	memcpy(&bdd_info_p->bdd_info, bdd_info_cmd, sizeof(*bdd_info_cmd));
	mutex_unlock(&bdd_info_p->track_bdd_info_lock);

	schedule_delayed_work(&bdd_info_p->bdd_info_load_trigger_work, 0);
	return 0;
}

static int oplus_chg_track_set_hidl_chg_up_info(
	struct oplus_chg_track_hidl_cmd *cmd, struct oplus_chg_track *track_chip)
{
	struct oplus_chg_track_hidl_chg_up_info_cmd *chg_up_info_cmd;
	struct oplus_chg_track_hidl_chg_up_info *chg_up_info_p;

	if (!cmd)
		return -EINVAL;

	if (cmd->data_size != sizeof(struct oplus_chg_track_hidl_chg_up_info_cmd)) {
		chg_err("!!!size not match struct, ignore\n");
		return -EINVAL;
	}

	chg_up_info_p = &(track_chip->track_status.chg_up_limit_info_s);
	mutex_lock(&chg_up_info_p->track_chg_up_info_lock);
	if (chg_up_info_p->chg_up_info_uploading) {
		chg_debug("chg_up_info_uploading, should return\n");
		mutex_unlock(&chg_up_info_p->track_chg_up_info_lock);
		return 0;
	}
	chg_up_info_cmd = (struct oplus_chg_track_hidl_chg_up_info_cmd *)(cmd->data_buf);
	memcpy(&chg_up_info_p->chg_up_info, chg_up_info_cmd, sizeof(*chg_up_info_cmd));
	mutex_unlock(&chg_up_info_p->track_chg_up_info_lock);

	schedule_delayed_work(&chg_up_info_p->chg_up_info_load_trigger_work, 0);
	return 0;
}

static int oplus_chg_track_set_hidl_uisoh_info(
	struct oplus_chg_track_hidl_cmd *cmd, struct oplus_chg_track *track_chip)
{
	struct oplus_chg_track_hidl_uisoh_info_cmd *uisoh_info_cmd;
	struct oplus_chg_track_hidl_uisoh_info *uisoh_info_p;

	if (!cmd)
		return -EINVAL;

	if (cmd->data_size != sizeof(struct oplus_chg_track_hidl_uisoh_info_cmd)) {
		chg_err("!!!size not match struct, ignore\n");
		return -EINVAL;
	}

	uisoh_info_p = &(track_chip->track_status.uisoh_info_s);
	mutex_lock(&uisoh_info_p->track_uisoh_info_lock);
	if (uisoh_info_p->uisoh_info_uploading) {
		chg_debug("uisoh_info_uploading, should return\n");
		mutex_unlock(&uisoh_info_p->track_uisoh_info_lock);
		return 0;
	}
	uisoh_info_cmd = (struct oplus_chg_track_hidl_uisoh_info_cmd *)(cmd->data_buf);
	memcpy(&uisoh_info_p->uisoh_info, uisoh_info_cmd, sizeof(*uisoh_info_cmd));
	mutex_unlock(&uisoh_info_p->track_uisoh_info_lock);

	schedule_delayed_work(&uisoh_info_p->uisoh_info_load_trigger_work, 0);
	return 0;
}

static int oplus_chg_track_set_anti_expansion(
	struct oplus_chg_track_hidl_cmd *cmd, struct oplus_chg_track *track_chip)
{
	struct oplus_chg_track_hidl_bae_info_cmd *bae_info_cmd;
	struct oplus_chg_track_hidl_bae_info *hidl_bae_i;

	if (!cmd)
                return -EINVAL;

        if (cmd->data_size != sizeof(struct oplus_chg_track_hidl_bae_info_cmd)) {
                pr_err("!!!size not match struct, ignore: [%u != %lu]\n", cmd->data_size, sizeof(struct oplus_chg_track_hidl_bae_info_cmd));
                return -EINVAL;
        }

	hidl_bae_i = &(track_chip->track_status.bae_info_s);
	mutex_lock(&hidl_bae_i->track_bae_info_lock);
	if (hidl_bae_i->bae_info_uploading) {
		chg_debug("bae_info_uploading, should return\n");
		mutex_unlock(&hidl_bae_i->track_bae_info_lock);
		return 0;
	}
	bae_info_cmd = (struct oplus_chg_track_hidl_bae_info_cmd *)(cmd->data_buf);
	memcpy(&hidl_bae_i->bae_info, bae_info_cmd, sizeof(*bae_info_cmd));
	mutex_unlock(&hidl_bae_i->track_bae_info_lock);

	schedule_delayed_work(&hidl_bae_i->bae_info_load_trigger_work, 0);

	chg_info("bae_info_cmd : %d,%d,%d,%d\n", bae_info_cmd->anti_expansion_status,
									bae_info_cmd->anti_expansion_rus_status,
									bae_info_cmd->anti_expansion_high_risk_of_6hours,
									bae_info_cmd->anti_expansion_risk_state_of_21days);
	return 0;
}

static void oplus_track_upload_parallelchg_foldmode_info(struct work_struct *work)
{
	int index = 0;

	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_parallelchg_track_hidl_foldmode_info *parallel_foldmode_p = container_of(
		dwork, struct oplus_parallelchg_track_hidl_foldmode_info, load_trigger_work);
	struct oplus_chg_track *track_chip = g_track_chip;
	struct oplus_monitor *monitor;
	struct oplus_chg_track_status *track_status;

	if (!parallel_foldmode_p || !track_chip || !track_chip->monitor) {
		chg_err(" parallel_foldmode_p or track_chip or monitor null return\n");
		return;
	}
	monitor = track_chip->monitor;
	track_status = &track_chip->track_status;

	mutex_lock(&parallel_foldmode_p->track_lock);
	if (parallel_foldmode_p->info_uploading) {
		chg_info("uisoh_info_uploading, should return\n");
		mutex_unlock(&parallel_foldmode_p->track_lock);
		return;
	}

	if (parallel_foldmode_p->load_trigger_info)
		kfree(parallel_foldmode_p->load_trigger_info);
	parallel_foldmode_p->load_trigger_info = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!parallel_foldmode_p->load_trigger_info) {
		chg_err("uisoh_info_load_trigger memery alloc fail\n");
		mutex_unlock(&parallel_foldmode_p->track_lock);
		return;
	}
	parallel_foldmode_p->load_trigger_info->type_reason =
		TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	parallel_foldmode_p->load_trigger_info->flag_reason =
		TRACK_NOTIFY_FLAG_PARALLELCHG_FOLDMODE_INFO;
	parallel_foldmode_p->info_uploading = true;

	mutex_unlock(&parallel_foldmode_p->track_lock);


	index += scnprintf(&(parallel_foldmode_p->load_trigger_info->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			"%s", parallel_foldmode_p->parallelchg_foldmode_info.data_buf);
	oplus_chg_track_record_general_info(monitor, track_status, parallel_foldmode_p->load_trigger_info, index);

	oplus_chg_track_upload_trigger_data(parallel_foldmode_p->load_trigger_info);
	if (parallel_foldmode_p->load_trigger_info) {
		kfree(parallel_foldmode_p->load_trigger_info);
		parallel_foldmode_p->load_trigger_info = NULL;
	}
	memset(&parallel_foldmode_p->parallelchg_foldmode_info, 0,
		sizeof(parallel_foldmode_p->parallelchg_foldmode_info));
	parallel_foldmode_p->info_uploading = false;
	return;
}

static void oplus_track_upload_bcc_err_info(struct work_struct *work)
{
	int index = 0;
	int curr_time;
	static int upload_count = 0;
	static int pre_upload_time = 0;
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track_hidl_bcc_err *bcc_err =
		container_of(dwork, struct oplus_chg_track_hidl_bcc_err, bcc_err_load_trigger_work);

	if (!bcc_err)
		return;

	curr_time = oplus_chg_track_get_local_time_s();
	if (curr_time - pre_upload_time > TRACK_SOFT_ABNORMAL_UPLOAD_PERIOD)
		upload_count = 0;

	if (upload_count > TRACK_SOFT_UPLOAD_COUNT_MAX)
		return;

	mutex_lock(&bcc_err->track_bcc_err_lock);
	if (bcc_err->bcc_err_uploading) {
		chg_debug("bcc_err_uploading, should return\n");
		mutex_unlock(&bcc_err->track_bcc_err_lock);
		return;
	}

	if (!strlen(bcc_err->bcc_err.err_reason)) {
		chg_debug("bcc_err len is invalid\n");
		mutex_unlock(&bcc_err->track_bcc_err_lock);
		return;
	}

	if (bcc_err->bcc_err_load_trigger)
		kfree(bcc_err->bcc_err_load_trigger);
	bcc_err->bcc_err_load_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!bcc_err->bcc_err_load_trigger) {
		chg_err("bcc_err_load_trigger memery alloc fail\n");
		mutex_unlock(&bcc_err->track_bcc_err_lock);
		return;
	}
	bcc_err->bcc_err_load_trigger->type_reason = TRACK_NOTIFY_TYPE_SOFTWARE_ABNORMAL;
	bcc_err->bcc_err_load_trigger->flag_reason = TRACK_NOTIFY_FLAG_SMART_CHG_ABNORMAL;
	bcc_err->bcc_err_uploading = true;
	upload_count++;
	pre_upload_time = oplus_chg_track_get_local_time_s();
	mutex_unlock(&bcc_err->track_bcc_err_lock);

	index += scnprintf(&(bcc_err->bcc_err_load_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$err_scene@@%s", "bcc_err");

	index += scnprintf(&(bcc_err->bcc_err_load_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$err_reason@@%s", bcc_err->bcc_err.err_reason);

	index += scnprintf(&(bcc_err->bcc_err_load_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$curx_info@@%s", bcc_err->bcc_err.data_buf);
	oplus_chg_track_obtain_power_info(&(bcc_err->bcc_err_load_trigger->crux_info[index]),
					  OPLUS_CHG_TRACK_CURX_INFO_LEN - index);

	oplus_chg_track_upload_trigger_data(bcc_err->bcc_err_load_trigger);
	if (bcc_err->bcc_err_load_trigger) {
		kfree(bcc_err->bcc_err_load_trigger);
		bcc_err->bcc_err_load_trigger = NULL;
	}
	memset(&bcc_err->bcc_err, 0, sizeof(bcc_err->bcc_err));
	bcc_err->bcc_err_uploading = false;
	chg_debug("success\n");
}

static void oplus_track_upload_uisoh_info(struct work_struct *work)
{
	int index = 0;
	int curr_time;
	static int upload_count = 0;
	static int pre_upload_time = 0;
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track_hidl_uisoh_info *uisoh_info_p = container_of(
		dwork, struct oplus_chg_track_hidl_uisoh_info, uisoh_info_load_trigger_work);
	struct oplus_chg_track *track_chip = g_track_chip;

	if (!track_chip || !track_chip->monitor)
		return;

	if (!uisoh_info_p)
		return;

	curr_time = oplus_chg_track_get_local_time_s();
	if (curr_time - pre_upload_time > TRACK_SOFT_ABNORMAL_UPLOAD_PERIOD)
		upload_count = 0;

	if (upload_count > TRACK_SOFT_SOH_UPLOAD_COUNT_MAX)
		return;

	mutex_lock(&uisoh_info_p->track_uisoh_info_lock);
	if (uisoh_info_p->uisoh_info_uploading) {
		chg_debug("uisoh_info_uploading, should return\n");
		mutex_unlock(&uisoh_info_p->track_uisoh_info_lock);
		return;
	}

	if (uisoh_info_p->uisoh_info_load_trigger)
		kfree(uisoh_info_p->uisoh_info_load_trigger);
	uisoh_info_p->uisoh_info_load_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!uisoh_info_p->uisoh_info_load_trigger) {
		chg_err("uisoh_info_load_trigger memery alloc fail\n");
		mutex_unlock(&uisoh_info_p->track_uisoh_info_lock);
		return;
	}
	uisoh_info_p->uisoh_info_load_trigger->type_reason =
		TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	uisoh_info_p->uisoh_info_load_trigger->flag_reason =
		TRACK_NOTIFY_FLAG_UISOH_INFO;
	uisoh_info_p->uisoh_info_uploading = true;
	upload_count++;
	pre_upload_time = oplus_chg_track_get_local_time_s();
	mutex_unlock(&uisoh_info_p->track_uisoh_info_lock);

	index += scnprintf(
		&(uisoh_info_p->uisoh_info_load_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$err_scene@@%s",
		"uisoh_info");

	index += scnprintf(&(uisoh_info_p->uisoh_info_load_trigger->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			"$$curx_info@@%s", uisoh_info_p->uisoh_info.data_buf);
	index += scnprintf(&(uisoh_info_p->uisoh_info_load_trigger->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			"$$ori_fcc@@%d", track_chip->monitor->batt_fcc);
	index += scnprintf(&(uisoh_info_p->uisoh_info_load_trigger->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			"$$ori_soh@@%d", track_chip->monitor->batt_soh);
	index += scnprintf(&(uisoh_info_p->uisoh_info_load_trigger->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			"$$design_capacity@@%d", oplus_gauge_get_batt_capacity_mah(track_chip->monitor->gauge_topic));

	oplus_chg_track_upload_trigger_data(uisoh_info_p->uisoh_info_load_trigger);
	if (uisoh_info_p->uisoh_info_load_trigger) {
		kfree(uisoh_info_p->uisoh_info_load_trigger);
		uisoh_info_p->uisoh_info_load_trigger = NULL;
	}
	memset(&uisoh_info_p->uisoh_info, 0, sizeof(uisoh_info_p->uisoh_info));
	uisoh_info_p->uisoh_info_uploading = false;
	chg_debug("success\n");
}

static void oplus_track_upload_chg_up_info(struct work_struct *work)
{
	int index = 0;
	int curr_time;
	static int upload_count = 0;
	static int pre_upload_time = 0;
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track_hidl_chg_up_info *chg_up_info_p = container_of(
		dwork, struct oplus_chg_track_hidl_chg_up_info, chg_up_info_load_trigger_work);
	struct oplus_chg_track *track_chip = g_track_chip;

	if (!track_chip || !track_chip->monitor)
		return;

	if (!chg_up_info_p)
		return;

	curr_time = oplus_chg_track_get_local_time_s();
	if (curr_time - pre_upload_time > TRACK_SOFT_ABNORMAL_UPLOAD_PERIOD)
		upload_count = 0;

	if (upload_count > TRACK_SOFT_UPLOAD_COUNT_MAX)
		return;

	mutex_lock(&chg_up_info_p->track_chg_up_info_lock);
	if (chg_up_info_p->chg_up_info_uploading) {
		chg_debug("chg_up_info_uploading, should return\n");
		mutex_unlock(&chg_up_info_p->track_chg_up_info_lock);
		return;
	}

	if (chg_up_info_p->chg_up_info_load_trigger)
		kfree(chg_up_info_p->chg_up_info_load_trigger);
	chg_up_info_p->chg_up_info_load_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chg_up_info_p->chg_up_info_load_trigger) {
		chg_err("chg_up_info_load_trigger memery alloc fail\n");
		mutex_unlock(&chg_up_info_p->track_chg_up_info_lock);
		return;
	}
	chg_up_info_p->chg_up_info_load_trigger->type_reason =
		TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	chg_up_info_p->chg_up_info_load_trigger->flag_reason =
		TRACK_NOTIFY_FLAG_CHG_UP_INFO;
	chg_up_info_p->chg_up_info_uploading = true;
	upload_count++;
	pre_upload_time = oplus_chg_track_get_local_time_s();
	mutex_unlock(&chg_up_info_p->track_chg_up_info_lock);

	index += scnprintf(
		&(chg_up_info_p->chg_up_info_load_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$err_scene@@%s",
		"chg_up_info");

	index += scnprintf(&(chg_up_info_p->chg_up_info_load_trigger->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			"$$curx_info@@%s", chg_up_info_p->chg_up_info.data_buf);

	oplus_chg_track_upload_trigger_data(chg_up_info_p->chg_up_info_load_trigger);
	if (chg_up_info_p->chg_up_info_load_trigger) {
		kfree(chg_up_info_p->chg_up_info_load_trigger);
		chg_up_info_p->chg_up_info_load_trigger = NULL;
	}
	memset(&chg_up_info_p->chg_up_info, 0, sizeof(chg_up_info_p->chg_up_info));
	chg_up_info_p->chg_up_info_uploading = false;
	chg_debug("success\n");
}

static void oplus_track_upload_bdd_info(struct work_struct *work)
{
	int index = 0;
	int curr_time;
	static int upload_count = 0;
	static int pre_upload_time = 0;
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track_hidl_bdd_info *bdd_info_p = container_of(
		dwork, struct oplus_chg_track_hidl_bdd_info, bdd_info_load_trigger_work);
	struct oplus_chg_track *track_chip = g_track_chip;

	if (!track_chip || !track_chip->monitor)
		return;

	if (!bdd_info_p)
		return;

	curr_time = oplus_chg_track_get_local_time_s();
	if (curr_time - pre_upload_time > TRACK_SOFT_ABNORMAL_UPLOAD_PERIOD)
		upload_count = 0;

	if (upload_count > TRACK_SOFT_UPLOAD_COUNT_MAX)
		return;

	mutex_lock(&bdd_info_p->track_bdd_info_lock);
	if (bdd_info_p->bdd_info_uploading) {
		chg_debug("bdd_info_uploading, should return\n");
		mutex_unlock(&bdd_info_p->track_bdd_info_lock);
		return;
	}

	if (bdd_info_p->bdd_info_load_trigger)
		kfree(bdd_info_p->bdd_info_load_trigger);
	bdd_info_p->bdd_info_load_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!bdd_info_p->bdd_info_load_trigger) {
		chg_err("bdd_info_load_trigger memery alloc fail\n");
		mutex_unlock(&bdd_info_p->track_bdd_info_lock);
		return;
	}
	bdd_info_p->bdd_info_load_trigger->type_reason =
		TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	bdd_info_p->bdd_info_load_trigger->flag_reason =
		TRACK_NOTIFY_FLAG_BDD_INFO;
	bdd_info_p->bdd_info_uploading = true;
	upload_count++;
	pre_upload_time = oplus_chg_track_get_local_time_s();
	mutex_unlock(&bdd_info_p->track_bdd_info_lock);

	index += scnprintf(
		&(bdd_info_p->bdd_info_load_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$err_scene@@%s",
		"bdd_info");

	index += scnprintf(&(bdd_info_p->bdd_info_load_trigger->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			"$$curx_info@@%s", bdd_info_p->bdd_info.data_buf);

	oplus_chg_track_upload_trigger_data(bdd_info_p->bdd_info_load_trigger);
	if (bdd_info_p->bdd_info_load_trigger) {
		kfree(bdd_info_p->bdd_info_load_trigger);
		bdd_info_p->bdd_info_load_trigger = NULL;
	}
	memset(&bdd_info_p->bdd_info, 0, sizeof(bdd_info_p->bdd_info));
	bdd_info_p->bdd_info_uploading = false;
	chg_debug("success\n");
}

static void oplus_track_upload_anti_expansion_info(struct work_struct *work)
{
	int index = 0;
	int curr_time;
	static int upload_count = 0;
	static int pre_upload_time = 0;
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track_hidl_bae_info *bae_info_p = container_of(
		dwork, struct oplus_chg_track_hidl_bae_info, bae_info_load_trigger_work);
	struct oplus_chg_track *track_chip = g_track_chip;

	if (!track_chip || !track_chip->monitor)
		return;

	if (!bae_info_p)
		return;

	curr_time = oplus_chg_track_get_local_time_s();
	if (curr_time - pre_upload_time > TRACK_SOFT_ABNORMAL_UPLOAD_PERIOD)
		upload_count = 0;

	if (upload_count > TRACK_SOFT_SOH_UPLOAD_COUNT_MAX)
		return;

	mutex_lock(&bae_info_p->track_bae_info_lock);
	if (bae_info_p->bae_info_uploading) {
		chg_debug("bae_info_uploading, should return\n");
		mutex_unlock(&bae_info_p->track_bae_info_lock);
		return;
	}

	if (bae_info_p->bae_info_load_trigger)
		kfree(bae_info_p->bae_info_load_trigger);
	bae_info_p->bae_info_load_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!bae_info_p->bae_info_load_trigger) {
		chg_err("bae_info_load_trigger memery alloc fail\n");
		mutex_unlock(&bae_info_p->track_bae_info_lock);
		return;
	}
	bae_info_p->bae_info_load_trigger->type_reason =
		TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	bae_info_p->bae_info_load_trigger->flag_reason =
		TRACK_NOTIFY_FLAG_ANTI_EXPANSION_INFO;
	bae_info_p->bae_info_uploading = true;
	upload_count++;
	pre_upload_time = oplus_chg_track_get_local_time_s();
	mutex_unlock(&bae_info_p->track_bae_info_lock);

	index += scnprintf(
		&(bae_info_p->bae_info_load_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$err_scene@@%s",
		"anti_expansion_info");

	index += scnprintf(&(bae_info_p->bae_info_load_trigger->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			"$$curx_info@@%d,%d,%d,%d", bae_info_p->bae_info.anti_expansion_status,
			bae_info_p->bae_info.anti_expansion_rus_status, bae_info_p->bae_info.anti_expansion_high_risk_of_6hours,
			bae_info_p->bae_info.anti_expansion_risk_state_of_21days);

	oplus_chg_track_upload_trigger_data(bae_info_p->bae_info_load_trigger);
	if (bae_info_p->bae_info_load_trigger) {
		kfree(bae_info_p->bae_info_load_trigger);
		bae_info_p->bae_info_load_trigger = NULL;
	}
	memset(&bae_info_p->bae_info, 0, sizeof(bae_info_p->bae_info));
	bae_info_p->bae_info_uploading = false;
	chg_info("success\n");
}

static void oplus_track_upload_shutdown_bat_info(struct work_struct *work)
{
	int index = 0;
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track_hidl_bat_info *bat_info_p = container_of(
		dwork, struct oplus_chg_track_hidl_bat_info, bat_info_load_trigger_work);
	struct oplus_chg_track *track_chip = g_track_chip;
	struct oplus_monitor *monitor;

	if (!track_chip || !track_chip->monitor)
		return;

	monitor = track_chip->monitor;
	mutex_lock(&bat_info_p->track_bat_info_lock);
	if (bat_info_p->bat_info_load_trigger)
		kfree(bat_info_p->bat_info_load_trigger);
	bat_info_p->bat_info_load_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!bat_info_p->bat_info_load_trigger) {
		chg_err("bat_info_load_trigger memery alloc fail\n");
		mutex_unlock(&bat_info_p->track_bat_info_lock);
		return;
	}
	bat_info_p->bat_info_load_trigger->type_reason =
		TRACK_NOTIFY_TYPE_DEVICE_ABNORMAL;
	bat_info_p->bat_info_load_trigger->flag_reason =
		TRACK_NOTIFY_FLAG_POWER_DOWN_ABNORMAL;
	bat_info_p->bat_info_uploading = true;
	mutex_unlock(&bat_info_p->track_bat_info_lock);

	index += scnprintf(&(bat_info_p->bat_info_load_trigger->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$err_scene@@%s", "uvlo");
	index += scnprintf(&(bat_info_p->bat_info_load_trigger->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$now_info@@");
	index += scnprintf(&(bat_info_p->bat_info_load_trigger->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$batt_temp@@%d", monitor->batt_temp);
	index += scnprintf(&(bat_info_p->bat_info_load_trigger->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$vbat_mv@@%d", monitor->vbat_mv);
	index += scnprintf(&(bat_info_p->bat_info_load_trigger->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$ibat_ma@@%d", monitor->ibat_ma);
	index += scnprintf(&(bat_info_p->bat_info_load_trigger->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$batt_soc@@%d", monitor->batt_soc);
	index += scnprintf(&(bat_info_p->bat_info_load_trigger->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$ui_soc@@%d", monitor->ui_soc);

	index += scnprintf(&(bat_info_p->bat_info_load_trigger->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			"$$shutdown_info@@%s", bat_info_p->bat_info.data_buf);

	oplus_chg_track_upload_trigger_data(bat_info_p->bat_info_load_trigger);
	if (bat_info_p->bat_info_load_trigger) {
		kfree(bat_info_p->bat_info_load_trigger);
		bat_info_p->bat_info_load_trigger = NULL;
	}
	chg_info("success\n");
}

static int oplus_chg_track_set_hidl_bat_info(
	struct oplus_chg_track_hidl_cmd *cmd, struct oplus_chg_track *track_chip)
{
	struct oplus_chg_track_hidl_bat_info *bat_info_p;

	if (!cmd)
		return -EINVAL;

	bat_info_p = &(track_chip->track_status.bat_info_s);
	mutex_lock(&bat_info_p->track_bat_info_lock);
	if (bat_info_p->bat_info_uploading) {
		chg_debug("bat_info have uploaded, should return\n");
		mutex_unlock(&bat_info_p->track_bat_info_lock);
		return 0;
	}
	memcpy(&bat_info_p->bat_info.data_buf, cmd->data_buf, cmd->data_size);
	chg_info("cmd data_size :%d\n", cmd->data_size);
	mutex_unlock(&bat_info_p->track_bat_info_lock);

	schedule_delayed_work(&bat_info_p->bat_info_load_trigger_work, msecs_to_jiffies(5000));
	return 0;
}

static int oplus_chg_track_bat_info_init(struct oplus_chg_track *chip)
{
	struct oplus_chg_track_hidl_bat_info *bat_info_p;

	if (!chip)
		return -EINVAL;

	bat_info_p = &(chip->track_status.bat_info_s);
	mutex_init(&bat_info_p->track_bat_info_lock);
	bat_info_p->bat_info_uploading = false;
	bat_info_p->bat_info_load_trigger = NULL;

	memset(&bat_info_p->bat_info, 0, sizeof(bat_info_p->bat_info));
	INIT_DELAYED_WORK(&bat_info_p->bat_info_load_trigger_work, oplus_track_upload_shutdown_bat_info);

	return 0;
}

static int oplus_chg_track_bcc_err_init(struct oplus_chg_track *chip)
{
	struct oplus_chg_track_hidl_bcc_err *bcc_err;

	if (!chip)
		return -EINVAL;

	bcc_err = &(chip->track_status.bcc_err);
	mutex_init(&bcc_err->track_bcc_err_lock);
	bcc_err->bcc_err_uploading = false;
	bcc_err->bcc_err_load_trigger = NULL;

	memset(&bcc_err->bcc_err, 0, sizeof(bcc_err->bcc_err));
	INIT_DELAYED_WORK(&bcc_err->bcc_err_load_trigger_work, oplus_track_upload_bcc_err_info);

	return 0;
}

static int oplus_chg_track_chg_up_err_init(struct oplus_chg_track *chip)
{
	struct oplus_chg_track_hidl_chg_up_info *chg_up_info_p;

	if (!chip)
		return - EINVAL;

	chg_up_info_p = &(chip->track_status.chg_up_limit_info_s);
	mutex_init(&chg_up_info_p->track_chg_up_info_lock);
	chg_up_info_p->chg_up_info_uploading = false;
	chg_up_info_p->chg_up_info_load_trigger = NULL;

	memset(&chg_up_info_p->chg_up_info, 0, sizeof(chg_up_info_p->chg_up_info));
	INIT_DELAYED_WORK(&chg_up_info_p->chg_up_info_load_trigger_work,
		oplus_track_upload_chg_up_info);

	return 0;
}

static int oplus_chg_track_bdd_err_init(struct oplus_chg_track *chip)
{
	struct oplus_chg_track_hidl_bdd_info *bdd_info_p;

	if (!chip)
		return - EINVAL;

	bdd_info_p = &(chip->track_status.bdd_info_s);
	mutex_init(&bdd_info_p->track_bdd_info_lock);
	bdd_info_p->bdd_info_uploading = false;
	bdd_info_p->bdd_info_load_trigger = NULL;

	memset(&bdd_info_p->bdd_info, 0, sizeof(bdd_info_p->bdd_info));
	INIT_DELAYED_WORK(&bdd_info_p->bdd_info_load_trigger_work,
		oplus_track_upload_bdd_info);

	return 0;
}

static int oplus_chg_track_uisoh_err_init(struct oplus_chg_track *chip)
{
	struct oplus_chg_track_hidl_uisoh_info *uisoh_info_p;

	if (!chip)
		return - EINVAL;

	uisoh_info_p = &(chip->track_status.uisoh_info_s);
	mutex_init(&uisoh_info_p->track_uisoh_info_lock);
	uisoh_info_p->uisoh_info_uploading = false;
	uisoh_info_p->uisoh_info_load_trigger = NULL;

	memset(&uisoh_info_p->uisoh_info, 0, sizeof(uisoh_info_p->uisoh_info));
	INIT_DELAYED_WORK(&uisoh_info_p->uisoh_info_load_trigger_work,
		oplus_track_upload_uisoh_info);

	return 0;
}

static int oplus_chg_track_anti_expansion_err_init(struct oplus_chg_track *chip)
{
	struct oplus_chg_track_hidl_bae_info *bae_info_i;

	if (!chip)
		return - EINVAL;

	bae_info_i = &(chip->track_status.bae_info_s);
	mutex_init(&bae_info_i->track_bae_info_lock);
	bae_info_i->bae_info_uploading = false;
	bae_info_i->bae_info_load_trigger = NULL;

	memset(&bae_info_i->bae_info, 0, sizeof(bae_info_i->bae_info));
	INIT_DELAYED_WORK(&bae_info_i->bae_info_load_trigger_work,
		oplus_track_upload_anti_expansion_info);

	return 0;
}

static int oplus_parallelchg_track_foldmode_init(struct oplus_chg_track *chip)
{
	struct oplus_parallelchg_track_hidl_foldmode_info *parallelchg_foldmode_info_p;

	if (!chip)
		return - EINVAL;

	parallelchg_foldmode_info_p = &(chip->track_status.parallelchg_info);
	mutex_init(&parallelchg_foldmode_info_p->track_lock);
	parallelchg_foldmode_info_p->info_uploading = false;
	parallelchg_foldmode_info_p->load_trigger_info = NULL;

	memset(&parallelchg_foldmode_info_p->parallelchg_foldmode_info, 0,
		sizeof(parallelchg_foldmode_info_p->parallelchg_foldmode_info));

	INIT_DELAYED_WORK(&parallelchg_foldmode_info_p->load_trigger_work,
		oplus_track_upload_parallelchg_foldmode_info);

	return 0;
}

static int oplus_chg_track_ttf_info_init(struct oplus_chg_track *chip)
{
	if (!chip)
		return - EINVAL;

	chip->track_status.ttf_info = (struct oplus_chg_track_hidl_ttf_info *)kzalloc(
		sizeof(struct oplus_chg_track_hidl_ttf_info), GFP_KERNEL);
	if (!chip->track_status.ttf_info) {
		chg_err("kzalloc mem fail\n");
		return -ENOMEM;
	}

	mutex_init(&chip->track_status.ttf_info->track_lock);
	chip->track_status.ttf_info->info_uploading = false;
	chip->track_status.ttf_info->load_trigger_info = NULL;

	INIT_DELAYED_WORK(&chip->track_status.ttf_info->load_trigger_work,
		oplus_track_upload_ttf_info);

	return 0;
}

static int oplus_chg_track_set_bcc_si(
	struct oplus_chg_track_hidl_cmd *cmd, struct oplus_chg_track *track_chip)
{
	struct oplus_chg_track_hidl_bcc_si *bcc_si;

	if (!cmd || !track_chip)
		return -EINVAL;

	if (cmd->data_size != strlen(cmd->data_buf)) {
		chg_err("!!!size[%d, %ld] not match, ignore\n",
			cmd->data_size, strlen(cmd->data_buf));
		return -EINVAL;
	}

	bcc_si = &(track_chip->track_status.bcc_si);
	mutex_lock(&bcc_si->bcc_si_lock);
	if (bcc_si->bcc_si_uploading) {
		chg_debug("bcc_si_uploading, should return\n");
		mutex_unlock(&bcc_si->bcc_si_lock);
		return 0;
	}

	bcc_si->bcc_si.type = cmd->cmd;
	strncpy(bcc_si->bcc_si.data_buf, cmd->data_buf, sizeof(bcc_si->bcc_si.data_buf) - 1);
	mutex_unlock(&bcc_si->bcc_si_lock);

	schedule_delayed_work(&bcc_si->bcc_si_load_trigger_work, 0);
	return 0;
}

static void oplus_track_upload_bcc_si(struct work_struct *work)
{
	int index = 0;
	int curr_time;
	static int upload_count = 0;
	static int pre_upload_time = 0;
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track_hidl_bcc_si *bcc_si = container_of(
		dwork, struct oplus_chg_track_hidl_bcc_si, bcc_si_load_trigger_work);

	curr_time = oplus_chg_track_get_local_time_s();
	if (curr_time - pre_upload_time > TRACK_SOFT_ABNORMAL_UPLOAD_PERIOD)
		upload_count = 0;

	if (upload_count > TRACK_SOFT_UPLOAD_COUNT_MAX) {
		chg_debug("bcc_si_upload count has reached the max %d\n",
			TRACK_SOFT_UPLOAD_COUNT_MAX);
		return;
	}

	mutex_lock(&bcc_si->bcc_si_lock);
	if (bcc_si->bcc_si_uploading) {
		chg_debug("bcc_si_uploading, should return\n");
		mutex_unlock(&bcc_si->bcc_si_lock);
		return;
	}

	if (bcc_si->bcc_si_load_trigger)
		kfree(bcc_si->bcc_si_load_trigger);

	bcc_si->bcc_si_load_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!bcc_si->bcc_si_load_trigger) {
		chg_err("bcc_si_load_trigger memery alloc fail\n");
		mutex_unlock(&bcc_si->bcc_si_lock);
		return;
	}

	if (bcc_si->bcc_si.type != TRACK_HIDL_BCC_SI_INFO) {
		bcc_si->bcc_si_load_trigger->type_reason = TRACK_NOTIFY_TYPE_SOFTWARE_ABNORMAL;
		bcc_si->bcc_si_load_trigger->flag_reason = TRACK_NOTIFY_FLAG_BCC_SI_ABNORMAL;
	} else {
		bcc_si->bcc_si_load_trigger->type_reason = TRACK_NOTIFY_TYPE_GENERAL_RECORD;
		bcc_si->bcc_si_load_trigger->flag_reason = TRACK_NOTIFY_FLAG_BCC_SI_INFO;
	}

	bcc_si->bcc_si_uploading = true;
	upload_count++;
	pre_upload_time = oplus_chg_track_get_local_time_s();
	mutex_unlock(&bcc_si->bcc_si_lock);

	index += scnprintf(&(bcc_si->bcc_si_load_trigger->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			"%s", bcc_si->bcc_si.data_buf);

	oplus_chg_track_upload_trigger_data(bcc_si->bcc_si_load_trigger);
	if (bcc_si->bcc_si_load_trigger) {
		kfree(bcc_si->bcc_si_load_trigger);
		bcc_si->bcc_si_load_trigger = NULL;
	}
	memset(&bcc_si->bcc_si, 0, sizeof(struct oplus_chg_track_hidl_bcc_si_cmd));
	bcc_si->bcc_si_uploading = false;
	chg_debug("success\n");
}

static int oplus_chg_track_bcc_si_init(struct oplus_chg_track *chip)
{
	struct oplus_chg_track_hidl_bcc_si *bcc_si;

	if (!chip)
		return - EINVAL;

	chg_debug("init\n");

	bcc_si = &(chip->track_status.bcc_si);
	mutex_init(&bcc_si->bcc_si_lock);
	bcc_si->bcc_si_uploading = false;
	bcc_si->bcc_si_load_trigger = NULL;

	memset(&bcc_si->bcc_si, 0, sizeof(struct oplus_chg_track_hidl_bcc_si_cmd));
	INIT_DELAYED_WORK(&bcc_si->bcc_si_load_trigger_work,
		oplus_track_upload_bcc_si);

	return 0;
}

static int oplus_chg_track_set_eis(
	struct oplus_chg_track_hidl_cmd *cmd, struct oplus_chg_track *track_chip)
{
	struct oplus_chg_track_hidl_eis *eis;

	if (!cmd || !track_chip)
		return -EINVAL;

	if (cmd->data_size != strlen(cmd->data_buf)) {
		chg_err("!!!size[%d, %ld] not match, ignore\n",
			cmd->data_size, strlen(cmd->data_buf));
		return -EINVAL;
	}

	eis = &(track_chip->track_status.eis);
	mutex_lock(&eis->eis_lock);
	if (eis->eis_uploading) {
		chg_debug("eis_uploading, should return\n");
		mutex_unlock(&eis->eis_lock);
		return 0;
	}

	eis->eis.type = cmd->cmd;
	strncpy(eis->eis.data_buf, cmd->data_buf, sizeof(eis->eis.data_buf) - 1);
	mutex_unlock(&eis->eis_lock);

	schedule_delayed_work(&eis->eis_load_trigger_work, 0);
	return 0;
}

static void oplus_track_upload_eis(struct work_struct *work)
{
	int index = 0;
	int curr_time;
	static int upload_count = 0;
	static int pre_upload_time = 0;
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track_hidl_eis *eis = container_of(
		dwork, struct oplus_chg_track_hidl_eis, eis_load_trigger_work);

	if (!eis)
		return;

	curr_time = oplus_chg_track_get_local_time_s();
	if (curr_time - pre_upload_time > TRACK_SOFT_ABNORMAL_UPLOAD_PERIOD)
		upload_count = 0;

	if (upload_count > TRACK_SOFT_UPLOAD_COUNT_MAX)
		return;

	mutex_lock(&eis->eis_lock);
	if (eis->eis_uploading) {
		chg_debug("eis_uploading, should return\n");
		mutex_unlock(&eis->eis_lock);
		return;
	}

	if (eis->eis_load_trigger)
		kfree(eis->eis_load_trigger);
	eis->eis_load_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!eis->eis_load_trigger) {
		chg_err("eis_load_trigger memery alloc fail\n");
		mutex_unlock(&eis->eis_lock);
		return;
	}
	if (eis->eis.type != TRACK_HIDL_EIS_INFO) {
		eis->eis_load_trigger->type_reason =
			TRACK_NOTIFY_TYPE_SOFTWARE_ABNORMAL;
		eis->eis_load_trigger->flag_reason =
			TRACK_NOTIFY_FLAG_EIS_ABNORMAL;
	} else {
		eis->eis_load_trigger->type_reason =
			TRACK_NOTIFY_TYPE_GENERAL_RECORD;
		eis->eis_load_trigger->flag_reason =
			TRACK_NOTIFY_FLAG_EIS_INFO;
	}
	eis->eis_uploading = true;
	upload_count++;
	pre_upload_time = oplus_chg_track_get_local_time_s();
	mutex_unlock(&eis->eis_lock);

	index += scnprintf(&(eis->eis_load_trigger->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			"%s", eis->eis.data_buf);

	oplus_chg_track_upload_trigger_data(eis->eis_load_trigger);
	if (eis->eis_load_trigger) {
		kfree(eis->eis_load_trigger);
		eis->eis_load_trigger = NULL;
	}
	memset(&eis->eis, 0, sizeof(struct oplus_chg_track_hidl_eis_cmd));
	eis->eis_uploading = false;
	chg_debug("success\n");
}

static int oplus_chg_track_eis_init(struct oplus_chg_track *chip)
{
	struct oplus_chg_track_hidl_eis *eis;

	if (!chip)
		return - EINVAL;

	eis = &(chip->track_status.eis);
	mutex_init(&eis->eis_lock);
	eis->eis_uploading = false;
	eis->eis_load_trigger = NULL;

	memset(&eis->eis, 0, sizeof(struct oplus_chg_track_hidl_eis_cmd));
	INIT_DELAYED_WORK(&eis->eis_load_trigger_work,
		oplus_track_upload_eis);

	return 0;
}

static int oplus_chg_track_set_hidl_bms_info(struct oplus_chg_track_hidl_cmd *cmd, struct oplus_chg_track *track_chip)
{
	int len;

	if (!cmd)
		return -EINVAL;

	if (cmd->data_size > sizeof(track_chip->track_status.bms_info)) {
		chg_err("!!!size not match struct, ignore\n");
		return -EINVAL;
	}

	len = strlen(track_chip->track_status.bms_info);
	if (strlen(cmd->data_buf) < TRACK_HIDL_BMS_INFO_LEN - len) {
		if (!len)
			scnprintf(&(track_chip->track_status.bms_info[len]), TRACK_HIDL_BMS_INFO_LEN - len, "%s",
				 cmd->data_buf);
		else
			scnprintf(&(track_chip->track_status.bms_info[len]), TRACK_HIDL_BMS_INFO_LEN - len, ";%s",
				 cmd->data_buf);
	}
	chg_info("bms_info: %s\n", track_chip->track_status.bms_info);
	return len;
}

static int oplus_chg_track_set_hidl_hyper_info(struct oplus_chg_track_hidl_cmd *cmd, struct oplus_chg_track *track_chip)
{
	struct oplus_chg_track_hidl_hyper_info *hidl_hyper_info;

	if (!cmd)
		return -EINVAL;

	if (cmd->data_size != sizeof(struct oplus_chg_track_hidl_hyper_info)) {
		pr_err("!!!size not match struct, ignore\n");
		return -EINVAL;
	}

	hidl_hyper_info = (struct oplus_chg_track_hidl_hyper_info *)(cmd->data_buf);
	track_chip->track_status.hyper_en = hidl_hyper_info->hyper_en;

	return 0;
}

int oplus_chg_track_set_hidl_info(const char *buf, size_t count)
{
	struct oplus_chg_track *track_chip = g_track_chip;
	struct oplus_chg_track_hidl_cmd *p_cmd;

	if (!track_chip)
		return -EINVAL;

	p_cmd = (struct oplus_chg_track_hidl_cmd *)buf;
	if (count != sizeof(struct oplus_chg_track_hidl_cmd)) {
		chg_err("!!!size of buf is not matched\n");
		return -EINVAL;
	}

	chg_debug("!!!cmd[%d]\n", p_cmd->cmd);
	switch (p_cmd->cmd) {
	case TRACK_HIDL_BCC_INFO:
		oplus_chg_track_set_hidl_bcc_info(p_cmd, track_chip);
		break;
	case TRACK_HIDL_BCC_ERR:
		oplus_chg_track_set_hidl_bcc_err(p_cmd, track_chip);
		break;
	case TRACK_HIDL_BMS_INFO:
		oplus_chg_track_set_hidl_bms_info(p_cmd, track_chip);
		break;
	case TRACK_HIDL_HYPER_INFO:
		oplus_chg_track_set_hidl_hyper_info(p_cmd, track_chip);
		break;
	case TRACK_HIDL_WLS_THIRD_ERR:
		break;
	case TRACK_HIDL_UISOH_INFO:
		oplus_chg_track_set_hidl_uisoh_info(p_cmd, track_chip);
		break;
	case TRACK_HIDL_PARALLELCHG_FOLDMODE_INFO:
		oplus_parallelchg_track_foldmode_info(p_cmd, track_chip);
		break;
	case TRACK_HIDL_TTF_INFO:
		oplus_chg_track_set_hidl_ttf_info(p_cmd, track_chip);
		break;
	case TRACK_HIDL_BCC_SI_INFO:
	case TRACK_HIDL_BCC_SI_ERR:
		oplus_chg_track_set_bcc_si(p_cmd, track_chip);
		break;
	case TRACK_HIDL_EIS_INFO:
	case TRACK_HIDL_EIS_ERR:
		oplus_chg_track_set_eis(p_cmd, track_chip);
		break;
	case TRACK_HIDL_ANTI_EXPANSION_INFO:
		oplus_chg_track_set_anti_expansion(p_cmd, track_chip);
		break;
	case TRACK_HIDL_CHG_UP_LIMIT_INFO:
		oplus_chg_track_set_hidl_chg_up_info(p_cmd, track_chip);
		break;
	case TRACK_HIDL_BAT_INFO:
		oplus_chg_track_set_hidl_bat_info(p_cmd, track_chip);
		break;
	case TRACK_HIDL_BDD_INFO:
		oplus_chg_track_set_hidl_bdd_info(p_cmd, track_chip);
		break;
	default:
		chg_err("!!!cmd error\n");
		break;
	}

	return 0;
}

struct dentry *oplus_chg_track_get_debugfs_root(void)
{
	mutex_lock(&debugfs_root_mutex);
	if (!track_debugfs_root) {
		track_debugfs_root =
			debugfs_create_dir("oplus_chg_track", NULL);
	}
	mutex_unlock(&debugfs_root_mutex);

	return track_debugfs_root;
}

static int oplus_chg_track_clear_cool_down_stats_time(
	struct oplus_chg_track_status *track_status)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cool_down_stats_table); i++)
		cool_down_stats_table[i].time = 0;
	return 0;
}

static int oplus_chg_track_set_voocphy_name(struct oplus_chg_track *track_chip)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(voocphy_info_table); i++) {
		if (voocphy_info_table[i].voocphy_type ==
		    track_chip->track_cfg.voocphy_type) {
			strncpy(track_chip->voocphy_name,
				voocphy_info_table[i].name,
				OPLUS_CHG_TRACK_VOOCPHY_NAME_LEN - 1);
			break;
		}
	}

	if (i == ARRAY_SIZE(voocphy_info_table))
		strncpy(track_chip->voocphy_name, voocphy_info_table[0].name,
			OPLUS_CHG_TRACK_VOOCPHY_NAME_LEN - 1);

	return 0;
}

static int oplus_chg_track_get_wired_type_info(int charge_type,
	int vbus_vol_type, struct oplus_chg_track_status *track_status)
{
	int i;
	int charge_index = -EINVAL;

	for (i = 0; i < ARRAY_SIZE(wired_type_table); i++) {
		if (wired_type_table[i].type == charge_type) {
			strncpy(track_status->power_info.wired_info.adapter_type,
				wired_type_table[i].name,
				OPLUS_CHG_TRACK_POWER_TYPE_LEN - 1);
			track_status->power_info.wired_info.power =
				wired_type_table[i].power;
			charge_index = i;
			break;
		}
	}
	for (i = 0; i < ARRAY_SIZE(wired_vbus_vol_type_table); i++) {
		if (wired_vbus_vol_type_table[i].type == charge_type &&
		    wired_vbus_vol_type_table[i].vbus_vol_type == vbus_vol_type) {
			strncpy(track_status->power_info.wired_info.adapter_type,
				wired_vbus_vol_type_table[i].name, OPLUS_CHG_TRACK_POWER_TYPE_LEN - 1);
			track_status->power_info.wired_info.power = wired_vbus_vol_type_table[i].power;
			break;
		}
	}

	return charge_index;
}

static int
oplus_chg_track_get_vooc_type_info(int vooc_type,
				   struct oplus_chg_track_status *track_status)
{
	int i;
	int vooc_index = -EINVAL;

	for (i = 0; i < ARRAY_SIZE(vooc_type_table); i++) {
		if (vooc_type_table[i].chg_type == vooc_type) {
			strncpy(track_status->power_info.wired_info.adapter_type,
				vooc_type_table[i].name,
				OPLUS_CHG_TRACK_POWER_TYPE_LEN - 1);
			track_status->power_info.wired_info.power =
				vooc_type_table[i].vol *
				vooc_type_table[i].cur / 1000 / 500;
			track_status->power_info.wired_info.power *= 500;
			track_status->power_info.wired_info.adapter_id =
				vooc_type_table[i].chg_type;
			vooc_index = i;
			break;
		}
	}

	return vooc_index;
}

static void oplus_chg_track_get_ufcs_type_info(struct oplus_chg_track_status *track_status,
	bool is_oplus, int id, int adapter_power, int emark_power)
{
	if (!strstr(track_status->power_info.wired_info.adapter_type, "ufcs"))
		return;

	if (is_oplus)
		strncpy(track_status->power_info.wired_info.adapter_type, "ufcs_oplus",
			OPLUS_CHG_TRACK_POWER_TYPE_LEN - 1);
	else
		strncpy(track_status->power_info.wired_info.adapter_type, "ufcs_third",
			OPLUS_CHG_TRACK_POWER_TYPE_LEN - 1);

	track_status->power_info.wired_info.adapter_id = id;
	if (adapter_power > 0)
		track_status->power_info.wired_info.power = adapter_power;
	track_status->power_info.wired_info.emark_power = emark_power;
}

__maybe_unused static int oplus_chg_track_get_wls_adapter_type_info(
	int charge_type, struct oplus_chg_track_status *track_status)
{
	int i;
	int charge_index = -EINVAL;

	for (i = 0; i < ARRAY_SIZE(wls_adapter_type_table); i++) {
		if (wls_adapter_type_table[i].type == charge_type) {
			strncpy(track_status->power_info.wls_info.adapter_type,
				wls_adapter_type_table[i].name,
				OPLUS_CHG_TRACK_POWER_TYPE_LEN - 1);
			track_status->power_info.wls_info.power =
				wls_adapter_type_table[i].power;
			charge_index = i;
			break;
		}
	}

	return charge_index;
}

__maybe_unused static int oplus_chg_track_get_wls_dock_type_info(
	int charge_type, struct oplus_chg_track_status *track_status)
{
	int i;
	int charge_index = -EINVAL;

	for (i = 0; i < ARRAY_SIZE(wls_dock_type_table); i++) {
		if (wls_dock_type_table[i].type == charge_type) {
			strncpy(track_status->power_info.wls_info.dock_type,
				wls_dock_type_table[i].name,
				OPLUS_CHG_TRACK_POWER_TYPE_LEN - 1);
			charge_index = i;
			break;
		}
	}

	return charge_index;
}

static int oplus_chg_track_get_batt_full_reason_info(
	int notify_flag, struct oplus_chg_track_status *track_status)
{
	int i;
	int charge_index = -EINVAL;

	for (i = 0; i < ARRAY_SIZE(batt_full_reason_table); i++) {
		if (batt_full_reason_table[i].notify_flag == notify_flag) {
			strncpy(track_status->batt_full_reason,
				batt_full_reason_table[i].reason,
				OPLUS_CHG_TRACK_BATT_FULL_REASON_LEN - 1);
			charge_index = i;
			break;
		}
	}

	return charge_index;
}

static int oplus_chg_track_get_chg_abnormal_reason_info(
	int notify_code, struct oplus_chg_track_status *track_status)
{
	int i;
	int charge_index = -EINVAL;
	int index;

	index = strlen(track_status->chg_abnormal_reason);
	for (i = 0; i < ARRAY_SIZE(chg_abnormal_reason_table); i++) {
		if (!chg_abnormal_reason_table[i].happened &&
		    chg_abnormal_reason_table[i].notify_code == notify_code) {
			chg_abnormal_reason_table[i].happened = true;
			if (!index)
				index += scnprintf(
					&(track_status
						  ->chg_abnormal_reason[index]),
					OPLUS_CHG_TRACK_CHG_ABNORMAL_REASON_LENS -
						index,
					"%s",
					chg_abnormal_reason_table[i].reason);
			else
				index += scnprintf(
					&(track_status
						  ->chg_abnormal_reason[index]),
					OPLUS_CHG_TRACK_CHG_ABNORMAL_REASON_LENS -
						index,
					",%s",
					chg_abnormal_reason_table[i].reason);
			charge_index = i;
			break;
		}
	}

	return charge_index;
}

int oplus_chg_track_get_gague_err_reason(int err_type, char *err_reason, int len)
{
	int i;
	int charge_index = -EINVAL;

	if (!err_reason || !len)
		return charge_index;

	for (i = 0; i < ARRAY_SIZE(gague_err_reason_table); i++) {
		if (gague_err_reason_table[i].err_type == err_type) {
			strncpy(err_reason, gague_err_reason_table[i].err_name, len);
			charge_index = i;
			break;
		}
	}

	if (i == ARRAY_SIZE(gague_err_reason_table))
		strncpy(err_reason, "unknow_err", len);

	return charge_index;
}

static int oplus_chg_track_parse_sili_dt(struct oplus_chg_track *track_dev)
{
	int i;
	int rc;
	struct device_node *node = oplus_get_node_by_type(track_dev->dev->of_node);

	track_dev->track_cfg.gauge_lifetime_support = of_property_read_bool(node, "track,gauge_lifetime_support");

	rc = of_property_read_u32(node, "track,gauge_max_cell_vol", &(track_dev->track_cfg.gauge_max_cell_vol));
	if (rc < 0) {
		pr_err("track,gauge_max_cell_vol reading failed, rc=%d\n", rc);
		track_dev->track_cfg.gauge_max_cell_vol = 4600;
	}

	rc = of_property_read_u32(node, "track,gauge_max_charge_curr", &(track_dev->track_cfg.gauge_max_charge_curr));
	if (rc < 0) {
		pr_err("track,gauge_max_charge_curr reading failed, rc=%d\n", rc);
		track_dev->track_cfg.gauge_max_charge_curr = 13000;
	}

	rc = of_property_read_u32(node, "track,gauge_max_dischg_curr", &(track_dev->track_cfg.gauge_max_dischg_curr));
	if (rc < 0) {
		pr_err("track,gauge_max_dischg_curr reading failed, rc=%d\n", rc);
		track_dev->track_cfg.gauge_max_dischg_curr = 3000;
	}

	rc = of_property_read_u32(node, "track,gauge_min_cell_temp", &(track_dev->track_cfg.gauge_min_cell_temp));
	if (rc < 0) {
		pr_err("track,gauge_min_cell_temp reading failed, rc=%d\n", rc);
		track_dev->track_cfg.gauge_min_cell_temp = 30;
	}

	rc = of_property_read_u32(node, "track,gauge_max_cell_temp", &(track_dev->track_cfg.gauge_max_cell_temp));
	if (rc < 0) {
		pr_err("track,gauge_max_cell_temp reading failed, rc=%d\n", rc);
		track_dev->track_cfg.gauge_max_cell_temp = 57;
	}

	rc = of_property_read_u32(node, "track,gauge_firmware_term_volt", &(track_dev->track_cfg.gauge_firmware_term_volt));
	if (rc < 0) {
		pr_err("track,gauge_firmware_term_volt reading failed, rc=%d\n", rc);
		track_dev->track_cfg.gauge_firmware_term_volt = 3100;
	}

	rc = of_property_count_elems_of_size(node, "track,sili_cc_term_vol_curve", sizeof(u32));
	if (rc < 0) {
		chg_err("parse sili_cc_term_vol_curve failed, rc=%d\n", rc);
		return rc;
	}
	if ((rc % 2) || ((rc / 2) > SILI_CC_TERM_VOL_CURVE_MAX))
		return rc;

	track_dev->track_cfg.sili_cc_term_vol_curve.nums = rc / 2;
	pr_info("rc=%d, sili_cc_term_vol_curve_nums=%d\n",
		rc, track_dev->track_cfg.sili_cc_term_vol_curve.nums);
	of_property_read_u32_array(node,
		"track,sili_cc_term_vol_curve", (u32 *)track_dev->track_cfg.sili_cc_term_vol_curve.limits, rc);
	for (i = 0; i < track_dev->track_cfg.sili_cc_term_vol_curve.nums; i++)
		pr_info("sili_cc_term_vol_curve [%d, %d, %d]\n",
			i, track_dev->track_cfg.sili_cc_term_vol_curve.limits[i].cc,
			track_dev->track_cfg.sili_cc_term_vol_curve.limits[i].term_volt);

	return 0;
}

static int oplus_chg_track_parse_dt(struct oplus_chg_track *track_dev)
{
	int rc = 0;
	struct device_node *node = oplus_get_node_by_type(track_dev->dev->of_node);
	int i = 0;
	int length = 0;

	rc = of_property_read_u32(node, "track,fast_chg_break_t_thd",
				  &(track_dev->track_cfg.fast_chg_break_t_thd));
	if (rc < 0) {
		chg_err("track,fast_chg_break_t_thd reading failed, rc=%d\n",
			rc);
		track_dev->track_cfg.fast_chg_break_t_thd = TRACK_T_THD_1000_MS;
	}

	rc = of_property_read_u32(
		node, "track,general_chg_break_t_thd",
		&(track_dev->track_cfg.general_chg_break_t_thd));
	if (rc < 0) {
		chg_err("track,general_chg_break_t_thd reading failed, rc=%d\n",
			rc);
		track_dev->track_cfg.general_chg_break_t_thd =
			TRACK_T_THD_500_MS;
	}

	rc = of_property_read_u32(node, "track,voocphy_type",
				  &(track_dev->track_cfg.voocphy_type));
	if (rc < 0) {
		chg_err("track,voocphy_type reading failed, rc=%d\n", rc);
		track_dev->track_cfg.voocphy_type = TRACK_NO_VOOCPHY;
	}

	rc = of_property_read_u32(node, "track,wls_chg_break_t_thd",
				  &(track_dev->track_cfg.wls_chg_break_t_thd));
	if (rc < 0) {
		chg_err("track,wls_chg_break_t_thd reading failed, rc=%d\n",
			rc);
		track_dev->track_cfg.wls_chg_break_t_thd = TRACK_T_THD_6000_MS;
	}

	rc = of_property_read_u32(node, "track,wls_normal_chg_break_t_thd",
				  &(track_dev->track_cfg.wls_normal_chg_break_t_thd));
	if (rc < 0) {
		chg_err("track,wls_normal_chg_break_t_thd reading failed, rc=%d\n", rc);
		track_dev->track_cfg.wls_normal_chg_break_t_thd = TRACK_T_THD_2000_MS;
	}

	rc = of_property_read_u32(
		node, "track,wired_fast_chg_scheme",
		&(track_dev->track_cfg.wired_fast_chg_scheme));
	if (rc < 0) {
		chg_err("track,wired_fast_chg_scheme reading failed, rc=%d\n",
			rc);
		track_dev->track_cfg.wired_fast_chg_scheme = -1;
	}

	rc = of_property_read_u32(node, "track,wls_fast_chg_scheme",
				  &(track_dev->track_cfg.wls_fast_chg_scheme));
	if (rc < 0) {
		chg_err("track,wls_fast_chg_scheme reading failed, rc=%d\n",
			rc);
		track_dev->track_cfg.wls_fast_chg_scheme = -1;
	}

	rc = of_property_read_u32(node, "track,wls_epp_chg_scheme",
				  &(track_dev->track_cfg.wls_epp_chg_scheme));
	if (rc < 0) {
		chg_err("track,wls_epp_chg_scheme reading failed, rc=%d\n", rc);
		track_dev->track_cfg.wls_epp_chg_scheme = -1;
	}

	rc = of_property_read_u32(node, "track,wls_bpp_chg_scheme",
				  &(track_dev->track_cfg.wls_bpp_chg_scheme));
	if (rc < 0) {
		chg_err("track,wls_bpp_chg_scheme reading failed, rc=%d\n", rc);
		track_dev->track_cfg.wls_bpp_chg_scheme = -1;
	}

	rc = of_property_read_u32(node, "track,wls_max_power",
				  &(track_dev->track_cfg.wls_max_power));
	if (rc < 0) {
		chg_err("track,wls_max_power reading failed, rc=%d\n", rc);
		track_dev->track_cfg.wls_max_power = 0;
	}

	rc = of_property_read_u32(node, "track,wired_max_power",
				  &(track_dev->track_cfg.wired_max_power));
	if (rc < 0) {
		chg_err("track,wired_max_power reading failed, rc=%d\n", rc);
		track_dev->track_cfg.wired_max_power = 0;
	}

	rc = of_property_read_u32(node, "track,track_ver",
				  &(track_dev->track_cfg.track_ver));
	if (rc < 0) {
		chg_err("track,track_ver reading failed, rc=%d\n", rc);
		track_dev->track_cfg.track_ver = TRACK_VER_3_4;
	}
	if (track_dev->track_cfg.track_ver >= TRACK_VER_MAX)
		track_dev->track_cfg.track_ver = TRACK_VER_3_4;

	track_dev->track_cfg.track_gauge_ctrl = of_property_read_bool(node, "track,gauge_status_ctrl");
	track_dev->track_cfg.low_soc_monitor = of_property_read_bool(node, "track,low_soc_monitor_ctrl");
	rc = of_property_read_u32(node, "track,external_gauge_num", &(track_dev->track_cfg.external_gauge_num));
	if (rc < 0) {
		pr_err("track,external_gauge_num reading failed, rc=%d\n", rc);
		track_dev->track_cfg.external_gauge_num = -1;
	}

	rc = of_property_read_u32(node, "track,nominal_qmax1", &(track_dev->track_cfg.nominal_qmax1));
	if (rc < 0) {
		pr_err("track,nominal_qmax1 reading failed, rc=%d\n", rc);
		track_dev->track_cfg.nominal_qmax1 = 0;
	}

	rc = of_property_read_u32(node, "track,nominal_qmax2", &(track_dev->track_cfg.nominal_qmax2));
	if (rc < 0) {
		pr_err("track,nominal_qmax2 reading failed, rc=%d\n", rc);
		track_dev->track_cfg.nominal_qmax2 = 0;
	}

	rc = of_property_read_u32(node, "track,nominal_fcc1", &(track_dev->track_cfg.nominal_fcc1));
	if (rc < 0) {
		pr_err("track,nominal_fcc1 reading failed, rc=%d\n", rc);
		track_dev->track_cfg.nominal_fcc1 = 0;
	}

	rc = of_property_read_u32(node, "track,nominal_fcc2", &(track_dev->track_cfg.nominal_fcc2));
	if (rc < 0) {
		pr_err("track,nominal_fcc2 reading failed, rc=%d\n", rc);
		track_dev->track_cfg.nominal_fcc2 = 0;
	}

	memset(&track_dev->track_cfg.exception_data, 0, sizeof(struct exception_data));
	rc = of_property_count_elems_of_size(node, "track,olc_config", sizeof(u64));
	if (rc < 0) {
		chg_err("Count track_olc_config failed, rc=%d\n", rc);
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
		if (get_eng_version() == PREVERSION) {
			chg_err("preversion open SocJump NoCharging FastChgBreak Crash olc config\n");
			track_dev->track_cfg.exception_data.olc_config[0] = 0x2;
			track_dev->track_cfg.exception_data.olc_config[2] = 0x19;
			track_dev->track_cfg.exception_data.olc_config[4] = 0x7;
			track_dev->track_cfg.exception_data.olc_config[5] = 0xffff;
			track_dev->track_cfg.exception_data.olc_config[7] = 0x3;
			track_dev->track_cfg.exception_data.olc_config[8] = 0x1;
		}
#endif
	} else {
		length = rc;
		if (length > OLC_CONFIG_NUM_MAX)
			length = OLC_CONFIG_NUM_MAX;
		chg_info("parse olc_config, size=%d\n", length);
		rc = of_property_read_u64_array(node, "track,olc_config",
				track_dev->track_cfg.exception_data.olc_config, length);
		if (rc < 0) {
			chg_err("parse chg_olc_config failed, rc=%d\n", rc);
		} else {
			for (i = 0; i < length; i++)
				chg_info("parse chg_olc_config[%d]=%llu\n", i, track_dev->track_cfg.exception_data.olc_config[i]);
		}
	}

	oplus_chg_track_parse_sili_dt(track_dev);

	return 0;
}

static void oplus_chg_track_uisoc_load_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(
		dwork, struct oplus_chg_track, uisoc_load_trigger_work);

	if (!chip)
		return;

	oplus_chg_track_upload_trigger_data(&chip->uisoc_load_trigger);
}

static void oplus_chg_track_soc_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip =
		container_of(dwork, struct oplus_chg_track, soc_trigger_work);

	if (!chip)
		return;

	oplus_chg_track_upload_trigger_data(&chip->soc_trigger);
}

static void oplus_chg_track_uisoc_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip =
		container_of(dwork, struct oplus_chg_track, uisoc_trigger_work);

	if (!chip)
		return;

	oplus_chg_track_upload_trigger_data(&chip->uisoc_trigger);
}

static void oplus_chg_track_uisoc_to_soc_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(
		dwork, struct oplus_chg_track, uisoc_to_soc_trigger_work);

	if (!chip)
		return;

	oplus_chg_track_upload_trigger_data(&chip->uisoc_to_soc_trigger);
}

#define MAX_SUB_BATT_INFO	32
static int
oplus_chg_track_record_general_info(struct oplus_monitor *monitor,
				    struct oplus_chg_track_status *track_status,
				    oplus_chg_track_trigger *p_trigger_data,
				    int index)
{
	char sub_batt_info[MAX_SUB_BATT_INFO] = {0};
	int idx = 0;

	if (!monitor || !p_trigger_data || !track_status)
		return -1;

	if (index < 0 || index >= OPLUS_CHG_TRACK_CURX_INFO_LEN) {
		chg_err("index is invalid\n");
		return -1;
	}

	if (monitor->main_gauge_topic && monitor->sub_gauge_topic)
		idx += scnprintf(sub_batt_info + idx, MAX_SUB_BATT_INFO - idx,
			"main_sub[%d %d %d %d]",
			monitor->main_ibat, monitor->main_soc, monitor->sub_ibat, monitor->sub_soc);

	index += scnprintf(
		&(p_trigger_data->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$other@@BATTERY[%d %d %d %d %d %d %d %d %d %d %d 0x%x], "
		"CHARGE[%d %d %d %d], "
		"WIRED[%d %d %d %d %d 0x%x %d %d %d %d %d], "
		"WIRELESS[%d %d %d %d %d 0x%x %d %d], "
		"VOOC[%d %d %d %d 0x%x], "
		"COMMON[%d %d %d 0x%x %d], "
		"SLOW[%d %d %d %d %d %d %d] "
		"%s",
		monitor->batt_temp, monitor->shell_temp, monitor->vbat_mv,
		monitor->vbat_min_mv, monitor->ibat_ma, monitor->batt_soc,
		monitor->smooth_soc, monitor->ui_soc, monitor->batt_rm, monitor->batt_fcc,
		monitor->batt_exist, monitor->batt_err_code, monitor->fv_mv,
		monitor->fcc_ma, monitor->chg_disable,
		monitor->chg_user_disable, monitor->wired_online,
		monitor->wired_ibus_ma, monitor->wired_vbus_mv,
		monitor->wired_icl_ma, monitor->wired_charge_type,
		monitor->wired_err_code, monitor->wired_suspend,
		monitor->wired_user_suspend, monitor->cc_mode,
		monitor->cc_detect, monitor->otg_enable, monitor->wls_online,
		monitor->wls_iout_ma, monitor->wls_vout_mv, monitor->wls_icl_ma,
		monitor->wls_charge_type, monitor->wls_err_code,
		monitor->wls_suspend, monitor->wls_user_suspend,
		monitor->vooc_online, monitor->vooc_started,
		monitor->vooc_charging, monitor->vooc_online_keep,
		monitor->vooc_sid, monitor->temp_region, monitor->ffc_status,
		monitor->cool_down, monitor->notify_code, monitor->led_on,
		track_status->has_judge_speed,
		track_status->soc_low_sect_incr_rm,
		track_status->soc_low_sect_cont_time,
		track_status->soc_medium_sect_incr_rm,
		track_status->soc_medium_sect_cont_time,
		track_status->soc_high_sect_incr_rm,
		track_status->soc_high_sect_cont_time,
		sub_batt_info);

	if (track_status->power_info.power_type == TRACK_CHG_TYPE_WIRELESS ||
	    (g_track_chip && p_trigger_data == &(g_track_chip->wls_charging_break_trigger))) {
		if (strlen(track_status->wls_break_crux_info))
			index += scnprintf(&(p_trigger_data->crux_info[index]),
					  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
					  "%s ",
					  track_status->wls_break_crux_info);
	}
	chg_info("index:%d\n", index);
	return 0;
}

static int oplus_chg_track_pack_cool_down_stats(
	struct oplus_chg_track_status *track_status, char *cool_down_pack)
{
	int i;
	int index = 0;

	if (cool_down_pack == NULL || track_status == NULL)
		return -1;

	for (i = 0; i < ARRAY_SIZE(cool_down_stats_table) - 1; i++) {
		index += scnprintf(&(cool_down_pack[index]),
				  OPLUS_CHG_TRACK_COOL_DOWN_PACK_LEN - index,
				  "%s,%d;", cool_down_stats_table[i].level_name,
				  cool_down_stats_table[i].time);
	}

	index += scnprintf(&(cool_down_pack[index]),
			  OPLUS_CHG_TRACK_COOL_DOWN_PACK_LEN - index, "%s,%d",
			  cool_down_stats_table[i].level_name,
			  cool_down_stats_table[i].time *
				  TRACK_THRAD_PERIOD_TIME_S);
	chg_info("i=%d, cool_down_pack[%s]\n", i, cool_down_pack);

	return 0;
}

static void
oplus_chg_track_record_charger_info(struct oplus_monitor *monitor,
				    oplus_chg_track_trigger *p_trigger_data,
				    struct oplus_chg_track_status *track_status)
{
	int index = 0;
	char cool_down_pack[OPLUS_CHG_TRACK_COOL_DOWN_PACK_LEN] = { 0 };
	int i;
	int fv_dec = 0, wired_ffc_dec = 0, wls_ffc_dec = 0, vct = 0;
	char adapter_type[OPLUS_CHG_TRACK_POWER_TYPE_LEN] = { 0 };
	bool wls_ocar_occur = false;
	bool cv_mode = false;

	if (monitor == NULL || p_trigger_data == NULL || track_status == NULL || monitor->track == NULL)
		return;

	memset(p_trigger_data->crux_info, 0, sizeof(p_trigger_data->crux_info));
	index += scnprintf(&(p_trigger_data->crux_info[index]),
			  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$power_mode@@%s",
			  track_status->power_info.power_mode);

	if (track_status->power_info.power_type == TRACK_CHG_TYPE_WIRE) {
		if (monitor->track->track_cfg.track_ver >= TRACK_VER_4_0 && track_status->chg_speed_is_slow)
			strncpy(adapter_type,
				monitor->track->slowchg_info.first_adapter_type, OPLUS_CHG_TRACK_POWER_TYPE_LEN - 1);
		else
			strncpy(adapter_type,
				track_status->power_info.wired_info.adapter_type, OPLUS_CHG_TRACK_POWER_TYPE_LEN - 1);
		index += scnprintf(&(p_trigger_data->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$adapter_t@@%s", adapter_type);
		if (strlen(g_track_chip->slowchg_info.last_adapter_type))
			index += scnprintf(&(p_trigger_data->crux_info[index]),
				OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$last_adapter_t@@%s",
				g_track_chip->slowchg_info.last_adapter_type);
		else
			index += scnprintf(&(p_trigger_data->crux_info[index]),
				OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$last_adapter_t@@unknow");

		if (track_status->power_info.wired_info.adapter_id)
			index += scnprintf(
				&(p_trigger_data->crux_info[index]),
				OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				"$$adapter_id@@0x%x",
				track_status->power_info.wired_info.adapter_id);
		index += scnprintf(&(p_trigger_data->crux_info[index]),
				  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$power@@%d",
				  track_status->power_info.wired_info.power);

		if (track_status->power_info.wired_info.emark_power > 0)
			index += scnprintf(&(p_trigger_data->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				"$$emark_power@@%d",
				track_status->power_info.wired_info.emark_power);

		if (track_status->wired_max_power <= 0)
			index += scnprintf(&(p_trigger_data->crux_info[index]),
				  OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$match_power@@%d", -1);
		else
			index += scnprintf(&(p_trigger_data->crux_info[index]),
				  OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$match_power@@%d",
				(track_status->power_info.wired_info.power >= track_status->wired_max_power));
	} else if (track_status->power_info.power_type ==
		   TRACK_CHG_TYPE_WIRELESS) {
		index += scnprintf(
			&(p_trigger_data->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			"$$adapter_t@@%s",
			track_status->power_info.wls_info.adapter_type);
		if (strlen(track_status->power_info.wls_info.dock_type))
			index += scnprintf(
				&(p_trigger_data->crux_info[index]),
				OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				"$$dock_type@@%s",
				track_status->power_info.wls_info.dock_type);
		index += scnprintf(&(p_trigger_data->crux_info[index]),
				  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$power@@%d",
				  track_status->power_info.wls_info.power);

		if (track_status->wls_max_power <= 0)
			index += scnprintf(&(p_trigger_data->crux_info[index]),
				  OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$match_power@@%d", -1);
		else
			index += scnprintf(&(p_trigger_data->crux_info[index]),
				  OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$match_power@@%d",
				(track_status->power_info.wls_info.power >= track_status->wls_max_power));

		if (monitor->track->track_cfg.track_ver >= TRACK_VER_4_0 &&
		    (p_trigger_data->type_reason == TRACK_NOTIFY_TYPE_CHARGING_SLOW ||
		    p_trigger_data->type_reason == TRACK_NOTIFY_TYPE_GENERAL_RECORD ||
		    p_trigger_data->type_reason == TRACK_NOTIFY_TYPE_NO_CHARGING)) {
			index += scnprintf(&(p_trigger_data->crux_info[index]),
				 OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				 "$$aud_t@@%llu",
				 track_status->wls_audio_t_total);
			index += scnprintf(&(p_trigger_data->crux_info[index]),
				 OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				 "$$cam_t@@%llu",
				 track_status->wls_camera_t_total);
			index += scnprintf(&(p_trigger_data->crux_info[index]),
				 OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				 "$$quiet_t@@%llu",
				 track_status->wls_quiet_mdoe_t_total);
			index += scnprintf(&(p_trigger_data->crux_info[index]),
				 OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				 "$$wls_t@@%llu",
				 track_status->wls_chg_t_total);
			if (is_wls_rx_disable_votable_available(monitor))
				wls_ocar_occur = get_client_vote(monitor->wls_rx_disable_votable, "CALL_NAME_2");
			index += scnprintf(&(p_trigger_data->crux_info[index]),
				 OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				 "$$ocar@@%d", wls_ocar_occur);
		}
	}

	index += scnprintf(&(p_trigger_data->crux_info[index]),
			  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$start_soc@@%d", track_status->chg_start_soc);
	index += scnprintf(&(p_trigger_data->crux_info[index]),
			  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$end_soc@@%d", track_status->chg_end_soc);
	index += scnprintf(&(p_trigger_data->crux_info[index]),
			  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$start_temp@@%d", track_status->chg_start_temp);
	index += scnprintf(&(p_trigger_data->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$end_temp@@%d", track_status->chg_end_temp);
	if (track_status->chg_soc50_time > 0)
		index += scnprintf(&(p_trigger_data->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$soc50_time@@%d", track_status->chg_soc50_time);
	index += scnprintf(&(p_trigger_data->crux_info[index]),
			  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$max_temp@@%d", track_status->chg_max_temp);
	index += scnprintf(&(p_trigger_data->crux_info[index]),
			  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$batt_start_temp@@%d",
			  track_status->batt_start_temp);
	index += scnprintf(&(p_trigger_data->crux_info[index]),
			  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$batt_max_temp@@%d", track_status->batt_max_temp);
	index += scnprintf(&(p_trigger_data->crux_info[index]),
			  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$batt_max_vol@@%d", track_status->batt_max_vol);
	index += scnprintf(&(p_trigger_data->crux_info[index]),
			  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$batt_max_curr@@%d", track_status->batt_max_curr);
	index += scnprintf(&(p_trigger_data->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$chg_max_vol@@%d", track_status->chg_max_vol);

	index += scnprintf(&(p_trigger_data->crux_info[index]),
			 OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			 "$$ledon_time@@%d", track_status->continue_ledon_time);
	if (track_status->ledon_ave_speed)
		index += scnprintf(&(p_trigger_data->crux_info[index]),
				  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$ledon_ave_speed@@%d",
				  track_status->ledon_ave_speed);

	index += scnprintf(&(p_trigger_data->crux_info[index]),
			  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$ledoff_time@@%d",
			  track_status->continue_ledoff_time);
	if (track_status->ledoff_ave_speed)
		index += scnprintf(&(p_trigger_data->crux_info[index]),
				  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$ledoff_ave_speed@@%d",
				  track_status->ledoff_ave_speed);

	if (track_status->chg_five_mins_cap != TRACK_PERIOD_CHG_CAP_INIT)
		index += scnprintf(&(p_trigger_data->crux_info[index]),
				  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$chg_five_mins_cap@@%d",
				  track_status->chg_five_mins_cap);

	if (track_status->chg_ten_mins_cap != TRACK_PERIOD_CHG_CAP_INIT)
		index += scnprintf(&(p_trigger_data->crux_info[index]),
				  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$chg_ten_mins_cap@@%d",
				  track_status->chg_ten_mins_cap);

	if (track_status->chg_twenty_mins_cap != TRACK_PERIOD_CHG_CAP_INIT)
		index += scnprintf(&(p_trigger_data->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$20mins_cap@@%d", track_status->chg_twenty_mins_cap);

	if (track_status->chg_thirty_mins_cap != TRACK_PERIOD_CHG_CAP_INIT)
		index += scnprintf(&(p_trigger_data->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$30mins_cap@@%d", track_status->chg_thirty_mins_cap);

	if (track_status->chg_average_speed !=
	    TRACK_PERIOD_CHG_AVERAGE_SPEED_INIT)
		index += scnprintf(&(p_trigger_data->crux_info[index]),
				  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$chg_average_speed@@%d",
				  track_status->chg_average_speed);

	if (track_status->chg_fast_full_time)
		index += scnprintf(&(p_trigger_data->crux_info[index]),
				  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$fast_full_time@@%d",
				  track_status->chg_fast_full_time);

	if (track_status->chg_report_full_time)
		index += scnprintf(&(p_trigger_data->crux_info[index]),
				  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$report_full_time@@%d",
				  track_status->chg_report_full_time);

	if (track_status->chg_normal_full_time)
		index += scnprintf(&(p_trigger_data->crux_info[index]),
				  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$normal_full_time@@%d",
				  track_status->chg_normal_full_time);

	if (strlen(track_status->batt_full_reason))
		index += scnprintf(&(p_trigger_data->crux_info[index]),
				  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$full_reason@@%s",
				  track_status->batt_full_reason);

	index += scnprintf(&(p_trigger_data->crux_info[index]),
			  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$chg_warm_once@@%d", track_status->tbatt_warm_once);
	index += scnprintf(&(p_trigger_data->crux_info[index]),
			  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$batt_fcc@@%d", monitor->batt_fcc);
	index += scnprintf(&(p_trigger_data->crux_info[index]),
			  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$batt_soh@@%d", monitor->batt_soh);
	index += scnprintf(&(p_trigger_data->crux_info[index]),
			  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$batt_cc@@%d", monitor->batt_cc);
	index += scnprintf(&(p_trigger_data->crux_info[index]),
			  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$rechg_counts@@%d", track_status->rechg_counts);

	if (strlen(track_status->chg_abnormal_reason))
		index += scnprintf(&(p_trigger_data->crux_info[index]),
				  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$chg_abnormal@@%s",
				  track_status->chg_abnormal_reason);

	/* nrr: not_record_reason */
	index += scnprintf(&(p_trigger_data->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$nrr@@%lu", track_status->not_record_reason);

	oplus_chg_track_pack_cool_down_stats(track_status, cool_down_pack);
	if (strlen(track_status->bcc_info->data_buf)) {
		index += scnprintf(&(p_trigger_data->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$bcc_trig_sta@@%s", track_status->bcc_info->data_buf);
		index += scnprintf(&(p_trigger_data->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$bcc_code@@0x%x", track_status->bcc_info->err_code);
	}

	if (strlen(track_status->bms_info)) {
		index += scnprintf(&(p_trigger_data->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$bms_sta@@%s", track_status->bms_info);
	}

	if (track_status->hyper_en == 1) {
		index += scnprintf(&(p_trigger_data->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$hyper_sta@@%s", track_status->hyper_info);
	} else {
		index += scnprintf(&(p_trigger_data->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$hyper_sta@@hyper_en=%d", track_status->hyper_en);
	}

	index += scnprintf(&(p_trigger_data->crux_info[index]),
			  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$cool_down_sta@@%s", cool_down_pack);

	oplus_chg_track_pack_app_stats(p_trigger_data->crux_info, &index);
	index += scnprintf(&(p_trigger_data->crux_info[index]),
			  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$plugin_utc_t@@%d", track_status->chg_plugin_utc_t);

	index += scnprintf(&(p_trigger_data->crux_info[index]),
			  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$plugout_utc_t@@%d",
			  track_status->chg_plugout_utc_t);

	index += scnprintf(&(p_trigger_data->crux_info[index]),
			  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$total_chg_time@@%d",
			  track_status->chg_total_time);

	if (track_status->aging_ffc_trig && index < OPLUS_CHG_TRACK_CURX_INFO_LEN) {
		index += scnprintf(&(p_trigger_data->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$aging_ffc@@%d", track_status->aging_ffc_trig);
		for (i = 0; i < FFC_CHG_STEP_MAX; i++) {
			if (track_status->aging_ffc_judge_vol[i] <= 0 ||
			    index >= OPLUS_CHG_TRACK_CURX_INFO_LEN)
				break;
			index += scnprintf(&(p_trigger_data->crux_info[index]),
					  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
					  ",%d", track_status->aging_ffc_judge_vol[i]);
		}
		if (track_status->aging_ffc_to_full_time && index < OPLUS_CHG_TRACK_CURX_INFO_LEN) {
			index += scnprintf(&(p_trigger_data->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
					  "$$aging_ffc_t@@%d", track_status->aging_ffc_to_full_time);
		}
	}

	index += scnprintf(&(p_trigger_data->crux_info[index]),
			  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$mmi_chg@@%d", track_status->once_mmi_chg);
	if(track_status->mmi_chg_open_t) {
		if(!track_status->mmi_chg_close_t) {
			track_status->mmi_chg_close_t = track_status->chg_plugout_utc_t;
			track_status->mmi_chg_constant_t =
				track_status->chg_plugout_utc_t - track_status->mmi_chg_open_t;
		}
		index += scnprintf(&(p_trigger_data->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				"$$mmi_sta@@open,%d;", track_status->mmi_chg_open_t);
		index += scnprintf(&(p_trigger_data->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				"close,%d;", track_status->mmi_chg_close_t);
		index += scnprintf(&(p_trigger_data->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				"constant,%d", track_status->mmi_chg_constant_t);
	}

	if (track_status->slow_chg_open_t) {
		if (!track_status->slow_chg_close_t) {
			if (track_status->slow_chg_open_n_t)
				track_status->slow_chg_duration +=
					track_status->chg_plugout_utc_t - track_status->slow_chg_open_n_t;
			else
				track_status->slow_chg_duration +=
					track_status->chg_plugout_utc_t - track_status->slow_chg_open_t;
		}
		index += scnprintf(&(p_trigger_data->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$slow_chg@@%d,%d,%d,%d,%d,%d,%d", track_status->slow_chg_open_t,
				  track_status->slow_chg_open_n_t, track_status->slow_chg_close_t,
				  track_status->slow_chg_open_cnt, track_status->slow_chg_duration,
				  track_status->slow_chg_pct, track_status->slow_chg_watt);
	}

	index += scnprintf(&(p_trigger_data->crux_info[index]),
			  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$chg_cycle_status@@%d", track_status->once_chg_cycle_status);
	if (track_status->ffc_time) {
		index += scnprintf(&(p_trigger_data->crux_info[index]),
				  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$ffc_cv_status@@"
				  "%d,%d,%d,%d,%d,%d",
				  track_status->ffc_time, track_status->ffc_start_main_soc,
				  track_status->ffc_start_sub_soc, track_status->ffc_end_main_soc,
				  track_status->ffc_end_sub_soc, track_status->cv_time);
	}
	if (track_status->dual_chan_time)
		index += scnprintf(&(p_trigger_data->crux_info[index]),
				  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$dual_chan_status@@%d,%d",
				  track_status->dual_chan_open_count, track_status->dual_chan_time);

	index += scnprintf(&(p_trigger_data->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$fcc_comp@@%d",
			  monitor->batt_fcc_comp);
	index += scnprintf(&(p_trigger_data->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$soh_comp@@%d",
			 monitor->batt_soh_comp);

	oplus_comm_get_dec_vol(monitor->comm_topic, &fv_dec, &wired_ffc_dec, &wls_ffc_dec, &vct);
	index += scnprintf(&(p_trigger_data->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			"$$v_dec@@%d,%d,%d,%d", fv_dec, wired_ffc_dec, wls_ffc_dec, vct);
	index += scnprintf(&(p_trigger_data->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$chg_cap@@%d",
			 monitor->batt_rm - track_status->chg_start_rm);

	if (monitor->plc_support) {
		index += scnprintf(&(p_trigger_data->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			"$$enable_count@@%d$$plc_init_sm_soc@@%d$$plc_init_ui_soc@@%d$$plc_init_temp@@%d",
			monitor->enable_count, monitor->plc_init_sm_soc, monitor->plc_init_ui_soc,
			monitor->plc_init_temp);
	}

	if (track_status->chg_no_charging_cnt >= TRACK_NO_CHG_CNT_60S) {
		index += scnprintf(&(p_trigger_data->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		    "$$nochgtime_last@@%d", track_status->chg_no_chg_time_s);
	}

	oplus_chg_track_record_device_name(monitor->track,
		track_status->power_info.wired_info.adapter_type, p_trigger_data, &index);

	if (monitor->track)
		oplus_chg_track_record_cp_abnormal_info(monitor->track, p_trigger_data, &index);
	if (track_status->ufcs_err_type != TRACK_UFCS_ERR_DEFAULT) {
		index += scnprintf(&(p_trigger_data->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			"$$ufcs_err@@%s", track_ufcs_sw_error_str(track_status->ufcs_err_type));
	}

	if (!track_status->fcl.one_full_trigger_cnt)
		scnprintf(track_status->fcl.info, TRACK_FCL_INFO_LEN - 1, "%s", "0,0,0");
	index += scnprintf(&(p_trigger_data->crux_info[index]),
                OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$fcl@@%d,%s,%d",
                track_status->fcl.one_full_trigger_cnt, track_status->fcl.info, track_status->fcl.n_full_trigger_cnt);
	memset(&(track_status->fcl), 0, sizeof(track_status->fcl));

	index += scnprintf(&(p_trigger_data->crux_info[index]),
			  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$Start_Vol@@%d", track_status->chg_start_vol);
	index += scnprintf(&(p_trigger_data->crux_info[index]),
			  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$Start_Cur@@%d", track_status->chg_start_cur);
	if (monitor->dischg_boost_topic) {
		cv_mode = oplus_boost_get_cv_mode(monitor->dischg_boost_topic);
	}
	index += scnprintf(&(p_trigger_data->crux_info[index]),
			  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$Boost_Mode@@%d", cv_mode ? 1 : 0);
	if (monitor->curr_derating_trig && monitor->track)
		index += scnprintf(&(p_trigger_data->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "%s",
			monitor->track->cycle_current_derating_trigger.crux_info);

	oplus_chg_track_record_general_info(monitor, track_status, p_trigger_data, index);
}

static int
oplus_chg_track_record_lpd_info(struct oplus_chg_track *track,
				oplus_chg_track_trigger *p_trigger_data)
{
	int index = 0;

	if (!p_trigger_data)
		return -1;

	if (oplus_wired_get_lpd_info_status(track->monitor->wls_topic) == OPLUS_LPD_NOT_DETECT)
			return -1;

	index = strlen(p_trigger_data->crux_info);
	if (index < 0 || index >= OPLUS_CHG_TRACK_CURX_INFO_LEN) {
		chg_err("index is invalid\n");
		return -1;
	}
	if (index < OPLUS_CHG_TRACK_CURX_INFO_LEN) {
		index += scnprintf(
			&(p_trigger_data->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "%s",
			track->usb_lpd_load_trigger.crux_info);
	}
	chg_info("%s\n", p_trigger_data->crux_info);
	return 0;
}

static void oplus_chg_track_charger_info_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(
		dwork, struct oplus_chg_track, charger_info_trigger_work);

	if (!chip)
		return;

	chip->track_status.wired_need_upload = false;
	chip->track_status.wls_need_upload = false;
	oplus_chg_track_record_lpd_info(chip, &chip->charger_info_trigger);
	oplus_chg_track_upload_trigger_data(&chip->charger_info_trigger);
}

static void oplus_chg_track_no_charging_trigger_work(struct work_struct *work)
{
	int curr_time;
	static int upload_count = 0;
	static int pre_upload_time = 0;
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(
		dwork, struct oplus_chg_track, no_charging_trigger_work);

	if (!chip)
		return;

	mutex_lock(&chip->no_charging_info_lock);
	curr_time = oplus_chg_track_get_local_time_s();
	if (curr_time - pre_upload_time > TRACK_SOFT_ABNORMAL_UPLOAD_PERIOD)
		upload_count = 0;

	chg_debug("no charging upload = %d\n", upload_count);
	if (upload_count > TRACK_SOFT_UPLOAD_COUNT_MAX) {
		chg_info("no charging track upload max\n");
		mutex_unlock(&chip->no_charging_info_lock);
		return;
	}

	upload_count++;
	pre_upload_time = oplus_chg_track_get_local_time_s();
	mutex_unlock(&chip->no_charging_info_lock);

	chip->track_status.wired_need_upload = false;
	chip->track_status.wls_need_upload = false;
	oplus_chg_track_record_lpd_info(chip, &chip->no_charging_trigger);
	oplus_chg_track_upload_trigger_data(&chip->no_charging_trigger);
}

static void oplus_chg_track_slow_charging_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(
		dwork, struct oplus_chg_track, slow_charging_trigger_work);

	if (!chip)
		return;

	chip->track_status.wired_need_upload = false;
	chip->track_status.wls_need_upload = false;
	oplus_chg_track_record_lpd_info(chip, &chip->slow_charging_trigger);
	oplus_chg_track_upload_trigger_data(&chip->slow_charging_trigger);
}

static void
oplus_chg_track_charging_break_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(
		dwork, struct oplus_chg_track, charging_break_trigger_work);

	if (!chip)
		return;

	oplus_chg_track_upload_trigger_data(&chip->charging_break_trigger);
}

static void
oplus_chg_track_wls_charging_break_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(
		dwork, struct oplus_chg_track, wls_charging_break_trigger_work);

	if (!chip)
		return;

	oplus_chg_track_upload_trigger_data(&chip->wls_charging_break_trigger);
}

static void oplus_chg_track_usbtemp_trigger_no_charging_work(struct work_struct *work)
{
	int wired_break_work_delay_t;
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(
		dwork, struct oplus_chg_track, usbtemp_trigger_no_charging_work);

	wired_break_work_delay_t = chip->track_cfg.fast_chg_break_t_thd +
		TRACK_TIME_500MS_JIFF_THD;

	if (!strncmp(chip->usbtemp_load_trigger.crux_info,
	    "$$reason@@first_condition", strlen("$$reason@@first_condition")) ||
	    !strncmp(chip->usbtemp_load_trigger.crux_info,
	    "$$reason@@second_condition", strlen("$$reason@@second_condition"))) {
		chg_info("usbtemp first or second condition!\n");
		chip->track_status.usbtemp_is_high = true;
	}
	if (chip->track_status.usbtemp_is_high) {
		chip->track_status.usbtemp_is_high = false;
		chip->no_charging_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_NO_CHARGING_USBTEMP_HIGH;
		oplus_chg_track_record_charger_info(chip->monitor,
			&chip->no_charging_trigger, &chip->track_status);
		schedule_delayed_work(&chip->no_charging_trigger_work,
			msecs_to_jiffies(wired_break_work_delay_t));
	}
}

static void oplus_chg_track_usbtemp_load_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(
		dwork, struct oplus_chg_track, usbtemp_load_trigger_work);

	if (!chip)
		return;

	oplus_chg_track_upload_trigger_data(&chip->usbtemp_load_trigger);
}

static void oplus_chg_track_usb_lpd_load_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(
		dwork, struct oplus_chg_track, usb_lpd_load_trigger_work);

	if (!chip)
		return;

	oplus_chg_track_upload_trigger_data(&chip->usb_lpd_load_trigger);
}

static void
oplus_chg_track_vbatt_too_low_load_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(
		dwork, struct oplus_chg_track, vbatt_too_low_load_trigger_work);

	if (!chip)
		return;

	oplus_chg_track_upload_trigger_data(&chip->vbatt_too_low_load_trigger);
}

static void
oplus_chg_track_vbatt_diff_over_load_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip =
		container_of(dwork, struct oplus_chg_track,
			     vbatt_diff_over_load_trigger_work);

	if (!chip)
		return;

	oplus_chg_track_upload_trigger_data(&chip->vbatt_diff_over_load_trigger);
}

static void
oplus_chg_track_uisoc_keep_1_t_load_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip =
		container_of(dwork, struct oplus_chg_track,
			     uisoc_keep_1_t_load_trigger_work);

	if (!chip)
		return;

	oplus_chg_track_upload_trigger_data(&chip->uisoc_keep_1_t_load_trigger);
}

static void oplus_chg_track_ic_err_msg_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(
		dwork, struct oplus_chg_track, ic_err_msg_trigger_work);

	if (!chip)
		return;

	oplus_chg_track_upload_trigger_data(&chip->ic_err_msg_load_trigger);
}

static void oplus_chg_track_chg_into_liquid_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(
		dwork, struct oplus_chg_track, chg_into_liquid_trigger_work);

	oplus_chg_track_upload_trigger_data(&chip->chg_into_liquid_load_trigger);
}

static void
oplus_chg_track_dual_chan_err_load_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(
		dwork, struct oplus_chg_track, dual_chan_err_load_trigger_work);

	oplus_chg_track_upload_trigger_data(&chip->dual_chan_err_load_trigger);
}

static void
oplus_chg_track_cal_chg_five_mins_capacity_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(
		dwork, struct oplus_chg_track, cal_chg_five_mins_capacity_work);
	struct oplus_monitor *monitor = chip->monitor;

	chip->track_status.chg_five_mins_cap =
		monitor->batt_soc - chip->track_status.chg_start_soc;
	chg_info(
		"chg_five_mins_soc:%d, start_chg_soc:%d, chg_five_mins_cap:%d\n",
		monitor->batt_soc, chip->track_status.chg_start_soc,
		chip->track_status.chg_five_mins_cap);
}

static void
oplus_chg_track_cal_chg_ten_mins_capacity_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(
		dwork, struct oplus_chg_track, cal_chg_ten_mins_capacity_work);
	struct oplus_monitor *monitor = chip->monitor;

	chip->track_status.chg_ten_mins_cap =
		monitor->batt_soc - chip->track_status.chg_start_soc;
	chg_info("chg_ten_mins_soc:%d, start_chg_soc:%d, chg_ten_mins_cap:%d\n",
		 monitor->batt_soc, chip->track_status.chg_start_soc,
		 chip->track_status.chg_ten_mins_cap);
}

static void oplus_chg_track_cal_chg_twenty_mins_capacity_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(dwork, struct oplus_chg_track, cal_chg_twenty_mins_capacity_work);
	struct oplus_monitor *monitor = chip->monitor;

	chip->track_status.chg_twenty_mins_cap = monitor->batt_soc - chip->track_status.chg_start_soc;
	chg_info("chg_twenty_mins_soc:%d, start_chg_soc:%d, chg_twenty_mins_cap:%d\n", monitor->batt_soc,
		 chip->track_status.chg_start_soc, chip->track_status.chg_twenty_mins_cap);
}

static void oplus_chg_track_cal_chg_thirty_mins_capacity_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(dwork, struct oplus_chg_track, cal_chg_thirty_mins_capacity_work);
	struct oplus_monitor *monitor = chip->monitor;

	chip->track_status.chg_thirty_mins_cap = monitor->batt_soc - chip->track_status.chg_start_soc;
	chg_info("chg_thirty_mins_soc:%d, start_chg_soc:%d, chg_thirty_mins_cap:%d\n", monitor->batt_soc,
		 chip->track_status.chg_start_soc, chip->track_status.chg_thirty_mins_cap);
}

static void oplus_chg_track_check_plugout_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(
		dwork, struct oplus_chg_track, plugout_state_work);
	struct oplus_monitor *monitor = chip->monitor;
	struct oplus_chg_track_status *track_status = &chip->track_status;

	if (((monitor->wired_online == false) &&
		(monitor->wired_vbus_mv < 2500)) &&
		((monitor->vooc_started == true))) {

		if (monitor->vooc_started == true)
			chip->plugout_state_trigger.flag_reason = TRACK_NOTIFY_FLAG_FASTCHG_START_ABNORMAL;

		oplus_chg_track_record_charger_info(monitor, &chip->plugout_state_trigger, track_status);
		oplus_chg_track_upload_trigger_data(&chip->plugout_state_trigger);
	}

	if (track_status->debug_plugout_state) {
		chip->plugout_state_trigger.flag_reason = track_status->debug_plugout_state;
		oplus_chg_track_record_charger_info(monitor, &chip->plugout_state_trigger, track_status);
		oplus_chg_track_upload_trigger_data(&chip->plugout_state_trigger);
		track_status->debug_plugout_state = 0;
	}
}

static void oplus_chg_track_mmi_chg_info_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(dwork, struct oplus_chg_track, mmi_chg_info_trigger_work);

	if (chip->mmi_chg_info_trigger) {
		oplus_chg_track_upload_trigger_data(chip->mmi_chg_info_trigger);
		kfree(chip->mmi_chg_info_trigger);
		chip->mmi_chg_info_trigger = NULL;
	}
	mutex_unlock(&chip->mmi_chg_info_lock);
}


static void oplus_chg_track_bal_info_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(dwork, struct oplus_chg_track, bal_info_trigger_work);

	if (chip->bal_info_trigger) {
		oplus_chg_track_upload_trigger_data(chip->bal_info_trigger);
		kfree(chip->bal_info_trigger);
		chip->bal_info_trigger = NULL;
	}
	mutex_unlock(&chip->bal_info_lock);
}

static void oplus_chg_track_slow_chg_info_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(dwork, struct oplus_chg_track, slow_chg_info_trigger_work);

	if (chip->slow_chg_info_trigger) {
		oplus_chg_track_upload_trigger_data(chip->slow_chg_info_trigger);
		kfree(chip->slow_chg_info_trigger);
		chip->slow_chg_info_trigger = NULL;
	}
	mutex_unlock(&chip->slow_chg_info_lock);
}

static void oplus_chg_track_chg_cycle_info_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(dwork, struct oplus_chg_track, chg_cycle_info_trigger_work);

	if (chip->chg_cycle_info_trigger) {
		oplus_chg_track_upload_trigger_data(chip->chg_cycle_info_trigger);
		kfree(chip->chg_cycle_info_trigger);
		chip->chg_cycle_info_trigger = NULL;
	}
	mutex_unlock(&chip->chg_cycle_info_lock);
}

static void oplus_chg_track_wls_info_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(dwork, struct oplus_chg_track, wls_info_trigger_work);

	if (chip->wls_info_trigger) {
		oplus_chg_track_upload_trigger_data(chip->wls_info_trigger);
		kfree(chip->wls_info_trigger);
		chip->wls_info_trigger = NULL;
	}
	mutex_unlock(&chip->wls_info_lock);
}

static void oplus_chg_track_ufcs_info_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(dwork, struct oplus_chg_track, ufcs_info_trigger_work);

	if (chip->ufcs_info_trigger) {
		oplus_chg_track_upload_trigger_data(chip->ufcs_info_trigger);
		kfree(chip->ufcs_info_trigger);
		chip->ufcs_info_trigger = NULL;
	}
	mutex_unlock(&chip->ufcs_info_lock);
}

static void oplus_chg_track_pps_info_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(dwork, struct oplus_chg_track,
						    pps_info_trigger_work);

	if (chip->pps_info_trigger) {
		oplus_chg_track_upload_trigger_data(chip->pps_info_trigger);
		kfree(chip->pps_info_trigger);
		chip->pps_info_trigger = NULL;
	}
	mutex_unlock(&chip->pps_info_lock);
}

static void oplus_chg_track_deep_dischg_info_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(dwork, struct oplus_chg_track, deep_dischg_info_trigger_work);

	if (chip->deep_dischg_info_trigger) {
		oplus_chg_track_upload_trigger_data(chip->deep_dischg_info_trigger);
		kfree(chip->deep_dischg_info_trigger);
		chip->deep_dischg_info_trigger = NULL;
	}
	mutex_unlock(&chip->deep_dischg_info_lock);
}
static void oplus_chg_track_plc_info_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(dwork, struct oplus_chg_track, plc_info_trigger_work);

	if (chip->plc_info_trigger) {
		oplus_chg_track_upload_trigger_data(chip->plc_info_trigger);
		kfree(chip->plc_info_trigger);
		chip->plc_info_trigger = NULL;
	}
	mutex_unlock(&chip->plc_info_lock);
}

static void oplus_chg_track_bidirect_cp_info_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(dwork, struct oplus_chg_track, bidirect_cp_info_trigger_work);

	if (chip->bidirect_cp_info_trigger) {
		oplus_chg_track_upload_trigger_data(chip->bidirect_cp_info_trigger);
		kfree(chip->bidirect_cp_info_trigger);
		chip->bidirect_cp_info_trigger = NULL;
	}
	mutex_unlock(&chip->bidirect_cp_info_lock);
}

static void oplus_chg_track_phy_cp_info_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(dwork, struct oplus_chg_track, phy_cp_info_trigger_work);

	if (chip->phy_cp_info_trigger) {
		oplus_chg_track_upload_trigger_data(chip->phy_cp_info_trigger);
		kfree(chip->phy_cp_info_trigger);
		chip->phy_cp_info_trigger = NULL;
	}
	mutex_unlock(&chip->phy_cp_info_lock);
}

static void oplus_chg_track_soccp_crash_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(dwork, struct oplus_chg_track, soccp_crash_trigger_work);

	if (chip->soccp_crash_trigger) {
		oplus_chg_track_upload_trigger_data(chip->soccp_crash_trigger);
		kfree(chip->soccp_crash_trigger);
		chip->soccp_crash_trigger = NULL;
	}
	mutex_unlock(&chip->soccp_crash_lock);
}

static void oplus_chg_track_bs_info_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(dwork, struct oplus_chg_track, bs_info_trigger_work);

	if (chip->bs_info_trigger) {
		oplus_chg_track_upload_trigger_data(chip->bs_info_trigger);
		kfree(chip->bs_info_trigger);
		chip->bs_info_trigger = NULL;
	}
	mutex_unlock(&chip->bs_info_lock);
}

static void oplus_chg_track_usbin_abnormal_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(dwork,
		struct oplus_chg_track, usbin_abnormal_trigger_work);

	if (chip->usbin_abnormal_trigger) {
		oplus_chg_track_upload_trigger_data(chip->usbin_abnormal_trigger);
		kfree(chip->usbin_abnormal_trigger);
		chip->usbin_abnormal_trigger = NULL;
	}
	mutex_unlock(&chip->usbin_abnormal_lock);
}

static void oplus_chg_track_state_keep_info_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(dwork, struct oplus_chg_track, state_keep_info_trigger_work);

	if (chip->state_keep_info_trigger) {
		oplus_chg_track_upload_trigger_data(chip->state_keep_info_trigger);
		kfree(chip->state_keep_info_trigger);
		chip->state_keep_info_trigger = NULL;
	}
	mutex_unlock(&chip->state_keep_info_lock);
}

static void oplus_chg_track_sec_ic_meminfo_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(dwork, struct oplus_chg_track, sec_ic_meminfo_trigger_work);

	if (chip->sec_ic_meminfo_trigger) {
		oplus_chg_track_upload_trigger_data(chip->sec_ic_meminfo_trigger);
		kfree(chip->sec_ic_meminfo_trigger);
		chip->sec_ic_meminfo_trigger = NULL;
	}
	mutex_unlock(&chip->sec_ic_meminfo_lock);
}

static void oplus_chg_track_eis_timeout_info_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(dwork, struct oplus_chg_track,
		eis_timeout_info_trigger_work);

	if (chip->eis_timeout_info_trigger) {
		oplus_chg_track_upload_trigger_data(chip->eis_timeout_info_trigger);
		kfree(chip->eis_timeout_info_trigger);
		chip->eis_timeout_info_trigger = NULL;
	}
	mutex_unlock(&chip->eis_timeout_info_lock);
}

static void oplus_chg_track_wired_retention_online_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(
		dwork, struct oplus_chg_track, wired_retention_online_trigger_work);
	struct oplus_monitor *monitor = chip->monitor;
	struct oplus_chg_track_status *track_status = &chip->track_status;
	int index = 0;

	if (!track_status || !monitor)
		return;
	if (chip->wired_retention_online_trigger)
		kfree(chip->wired_retention_online_trigger);

	chip->wired_retention_online_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->wired_retention_online_trigger) {
		chg_err("wired_retention_online_trigger memery alloc fail\n");
		return;
	}
	chip->wired_retention_online_trigger->type_reason = TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	chip->wired_retention_online_trigger->flag_reason = TRACK_NOTIFY_FLAG_WIRED_RETENTION_ONLINE;

	oplus_chg_track_get_charger_type(monitor, track_status,
					 TRACK_CHG_GET_LAST_TIME_TYPE);
	index += scnprintf(&(chip->wired_retention_online_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$retention_state@@%d", monitor->pre_retention_state);
	index += scnprintf(&(chip->wired_retention_online_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$retention_disconnect_count@@%d", monitor->total_disconnect_count);
	index += scnprintf(&(chip->wired_retention_online_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$retention_real_type@@%s", track_status->power_info.wired_info.adapter_type);

	oplus_chg_track_obtain_power_info(&(chip->wired_retention_online_trigger->crux_info[index]),
					  OPLUS_CHG_TRACK_CURX_INFO_LEN - index);
	oplus_chg_track_upload_trigger_data(chip->wired_retention_online_trigger);
	kfree(chip->wired_retention_online_trigger);
	chip->wired_retention_online_trigger = NULL;
}

void oplus_chg_track_upload_wired_retention_online_info(struct oplus_monitor *monitor)
{
	if (monitor->track != NULL)
		schedule_delayed_work(&monitor->track->wired_retention_online_trigger_work, 0);
}

static void oplus_chg_track_wired_reverse_chg_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(
		dwork, struct oplus_chg_track, wired_reverse_chg_trigger_work);
	struct oplus_monitor *monitor = chip->monitor;
	struct oplus_chg_track_status *track_status = &chip->track_status;
	struct rtc_time tm;
	int index = 0;
	unsigned long temp_reverse_screen_off_time = 0;
	if (!track_status || !monitor)
		return;
	if (chip->wired_reverse_chg_trigger)
		kfree(chip->wired_reverse_chg_trigger);

	chip->wired_reverse_chg_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->wired_reverse_chg_trigger) {
		chg_err("wired_reverse_chg_trigger memery alloc fail\n");
		return;
	}
	chip->wired_reverse_chg_trigger->type_reason = TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	chip->wired_reverse_chg_trigger->flag_reason = TRACK_NOTIFY_FLAG_WIRED_REVERSE_CHG_INFO;

	if (monitor->reverse_total_time >= monitor->reverse_screen_on_time)
		temp_reverse_screen_off_time = monitor->reverse_total_time - monitor->reverse_screen_on_time;
	else {
		temp_reverse_screen_off_time = monitor->reverse_total_time;
		monitor->reverse_screen_on_time = 0;
	}
	index += scnprintf(&(chip->wired_reverse_chg_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$high_reverse_count@@%d", monitor->high_reverse_count);
	index += scnprintf(&(chip->wired_reverse_chg_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$start_ui_soc@@%d", monitor->reverse_start_ui_soc);
	index += scnprintf(&(chip->wired_reverse_chg_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$chip_soc@@%d", monitor->reverse_start_chip_soc);
	index += scnprintf(&(chip->wired_reverse_chg_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$start_vbat@@%d", monitor->reverse_start_vbat);
	index += scnprintf(&(chip->wired_reverse_chg_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$shell_temp@@%d", monitor->reverse_start_shell_temp);
	index += scnprintf(&(chip->wired_reverse_chg_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$batt_temp@@%d", monitor->reverse_start_batt_temp);
	index += scnprintf(&(chip->wired_reverse_chg_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$sink_svid@@%04X", monitor->sink_svid);
	index += scnprintf(&(chip->wired_reverse_chg_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$max_shell_temp@@%d", monitor->reverse_max_shell_temp);
	index += scnprintf(&(chip->wired_reverse_chg_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$max_ibat@@%d", monitor->reverse_max_ibat);
	index += scnprintf(&(chip->wired_reverse_chg_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$min_vbat@@%d", monitor->reverse_min_vbat);
	index += scnprintf(&(chip->wired_reverse_chg_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$min_vbus@@%d", monitor->reverse_min_vbus);
	index += scnprintf(&(chip->wired_reverse_chg_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$max_source_cap_voltage@@%d", monitor->max_source_cap_voltage);
	index += scnprintf(&(chip->wired_reverse_chg_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$max_source_cap_current@@%d", monitor->max_source_cap_current);
	index += scnprintf(&(chip->wired_reverse_chg_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$max_sink_request_voltage@@%d", monitor->max_sink_request_voltage);
	index += scnprintf(&(chip->wired_reverse_chg_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$max_sink_request_current@@%d", monitor->max_sink_request_current);
	index += scnprintf(&(chip->wired_reverse_chg_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$screen_on_time@@%lu", monitor->reverse_screen_on_time);
	index += scnprintf(&(chip->wired_reverse_chg_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$screen_off_time@@%lu", temp_reverse_screen_off_time);
	index += scnprintf(&(chip->wired_reverse_chg_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$screen_on_count@@%d", monitor->reverse_screen_on_count);
	index += scnprintf(&(chip->wired_reverse_chg_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$total_time@@%lu", monitor->reverse_total_time);
	index += scnprintf(&(chip->wired_reverse_chg_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$end_ui_soc@@%d", monitor->reverse_end_ui_soc);
	index += scnprintf(&(chip->wired_reverse_chg_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$end_chip_soc@@%d", monitor->reverse_end_chip_soc);
	index += scnprintf(&(chip->wired_reverse_chg_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$end_vbat@@%d", monitor->reverse_end_vbat);
	index += scnprintf(&(chip->wired_reverse_chg_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$end_shell_temp@@%d", monitor->reverse_end_shell_temp);
	index += scnprintf(&(chip->wired_reverse_chg_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$end_batt_temp@@%d", monitor->reverse_end_batt_temp);

	oplus_chg_track_get_current_time(&tm);
	index += scnprintf(&(chip->wired_reverse_chg_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$time@@[%04d-%02d-%02d %02d:%02d:%02d]", tm.tm_year,
			tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min,
			tm.tm_sec);

	oplus_chg_track_upload_trigger_data(chip->wired_reverse_chg_trigger);
	kfree(chip->wired_reverse_chg_trigger);
	chip->wired_reverse_chg_trigger = NULL;
}


void oplus_chg_track_upload_reverse_chg_info(struct oplus_monitor *monitor)
{
	if (!monitor || !monitor->track)
		return;

	if (monitor->reverse_state) {
		monitor->reverse_start_ui_soc = monitor->ui_soc;
		monitor->reverse_start_chip_soc = monitor->batt_soc;
		monitor->reverse_start_vbat = monitor->vbat_mv;
		monitor->reverse_start_shell_temp = monitor->shell_temp;
		monitor->reverse_start_batt_temp = monitor->batt_temp;
	}
	if (monitor->pre_reverse_state != monitor->reverse_state && !monitor->reverse_state) {
		monitor->reverse_end_ui_soc = monitor->ui_soc;
		monitor->reverse_end_chip_soc = monitor->batt_soc;
		monitor->reverse_end_vbat = monitor->vbat_mv;
		monitor->reverse_end_shell_temp = monitor->shell_temp;
		monitor->reverse_end_batt_temp = monitor->batt_temp;
		monitor->max_source_cap_voltage = (monitor->max_source_cap & 0xFFFF0000) >> 16;
		monitor->max_source_cap_current = (monitor->max_source_cap & 0xFFFF);
		monitor->max_sink_request_voltage = (monitor->max_sink_request & 0xFFFF0000) >> 16;
		monitor->max_sink_request_current = (monitor->max_sink_request & 0xFFFF);
		if (monitor->reverse_total_time > 30)
			schedule_delayed_work(&monitor->track->wired_reverse_chg_trigger_work, 0);
	}
	monitor->pre_reverse_state = monitor->reverse_state;
}

static void oplus_chg_track_wired_high_reverse_err_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(
		dwork, struct oplus_chg_track, wired_high_reverse_err_trigger_work);
	struct oplus_monitor *monitor = chip->monitor;
	struct oplus_chg_track_status *track_status = &chip->track_status;
	struct rtc_time tm;
	int index = 0;

	if (!track_status || !monitor)
		return;
	if (chip->wired_high_reverse_err_trigger)
		kfree(chip->wired_high_reverse_err_trigger);

	chip->wired_high_reverse_err_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->wired_high_reverse_err_trigger) {
		chg_err("wired_high_reverse_err_trigger memery alloc fail\n");
		return;
	}
	chip->wired_high_reverse_err_trigger->type_reason = TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	chip->wired_high_reverse_err_trigger->flag_reason = TRACK_NOTIFY_FLAG_WIRED_HIGH_REVERSE_ERR;

	index += scnprintf(&(chip->wired_high_reverse_err_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$err_flag@@[%d]%s", monitor->high_reverse_err_flag,
		get_high_reverse_err_name_str(monitor->high_reverse_err_flag));
	index += scnprintf(&(chip->wired_high_reverse_err_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$end_ui_soc@@%d", monitor->reverse_end_ui_soc);
	index += scnprintf(&(chip->wired_high_reverse_err_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$end_chip_soc@@%d", monitor->reverse_end_chip_soc);
	index += scnprintf(&(chip->wired_high_reverse_err_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$end_vbat@@%d", monitor->reverse_end_vbat);
	index += scnprintf(&(chip->wired_high_reverse_err_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$end_shell_temp@@%d", monitor->reverse_end_shell_temp);
	index += scnprintf(&(chip->wired_high_reverse_err_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$end_batt_temp@@%d", monitor->reverse_end_batt_temp);
	index += scnprintf(&(chip->wired_high_reverse_err_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$source_cap_volt@@%d", monitor->err_source_pdo_volt);
	index += scnprintf(&(chip->wired_high_reverse_err_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$source_cap_curr@@%d", monitor->err_source_pdo_curr);
	index += scnprintf(&(chip->wired_high_reverse_err_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$sink_request_volt@@%d", monitor->err_sink_req_volt);
	index += scnprintf(&(chip->wired_high_reverse_err_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$sink_request_curr@@%d", monitor->err_sink_req_curr);

	oplus_chg_track_get_current_time(&tm);
	index += scnprintf(&(chip->wired_high_reverse_err_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$time@@[%04d-%02d-%02d %02d:%02d:%02d]", tm.tm_year,
			tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min,
			tm.tm_sec);

	oplus_chg_track_upload_trigger_data(chip->wired_high_reverse_err_trigger);
	kfree(chip->wired_high_reverse_err_trigger);
	chip->wired_high_reverse_err_trigger = NULL;
}


void oplus_chg_track_upload_high_reverse_err_info(struct oplus_monitor *monitor)
{
	int rc;
	union mms_msg_data data = { 0 };

	if (!monitor || !monitor->track)
		return;

	monitor->reverse_end_ui_soc = monitor->ui_soc;
	monitor->reverse_end_chip_soc = monitor->batt_soc;
	monitor->reverse_end_vbat = monitor->vbat_mv;
	if (monitor->comm_topic) {
		rc = oplus_mms_get_item_data(monitor->comm_topic, COMM_ITEM_SHELL_TEMP, &data, true);
		if (rc < 0) {
			chg_err("can't get COMM_ITEM_SHELL_TEMP data, rc=%d\n", rc);
		} else {
			monitor->reverse_end_shell_temp = data.intval;
		}
	}
	if (monitor->gauge_topic) {
		rc = oplus_mms_get_item_data(monitor->gauge_topic, GAUGE_ITEM_TEMP, &data, true);
		if (rc < 0) {
			chg_err("can't get GAUGE_ITEM_TEMP data, rc=%d\n", rc);
		} else {
			monitor->reverse_end_batt_temp = data.intval;
		}
	}
	schedule_delayed_work(&monitor->track->wired_high_reverse_err_trigger_work, 0);
}

static void oplus_chg_track_wired_online_err_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(
		dwork, struct oplus_chg_track, wired_online_err_trigger_work);
	int index = 0;

	if (!chip)
		return;

	if (chip->wired_online_err_trigger)
		kfree(chip->wired_online_err_trigger);

	chip->wired_online_err_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->wired_online_err_trigger) {
		chg_err("wired_online_err_trigger memery alloc fail\n");
		return;
	}

	chip->wired_online_err_trigger->type_reason = TRACK_NOTIFY_TYPE_SOFTWARE_ABNORMAL;
	chip->wired_online_err_trigger->flag_reason = TRACK_NOTIFY_FLAG_WIRED_ONLINE_ERROR;

	oplus_chg_track_obtain_power_info(&(chip->wired_online_err_trigger->crux_info[index]),
					  OPLUS_CHG_TRACK_CURX_INFO_LEN - index);
	oplus_chg_track_upload_trigger_data(chip->wired_online_err_trigger);
	kfree(chip->wired_online_err_trigger);
	chip->wired_online_err_trigger = NULL;
}

void oplus_chg_track_upload_wired_online_err_info(struct oplus_monitor *monitor)
{
	if (monitor->track != NULL)
		schedule_delayed_work(&monitor->track->wired_online_err_trigger_work, 0);
}

static void oplus_chg_track_uisoc_keep_2_err_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(dwork, struct oplus_chg_track, uisoc_keep_2_err_trigger_work);
	int index = 0;
	struct oplus_monitor *monitor = chip->monitor;

	if (chip->uisoc_keep_2_err_trigger)
		kfree(chip->uisoc_keep_2_err_trigger);

	chip->uisoc_keep_2_err_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->uisoc_keep_2_err_trigger) {
		chg_err("uisoc_keep_2_err_trigger memery alloc fail\n");
		return;
	}

	chip->uisoc_keep_2_err_trigger->type_reason = TRACK_NOTIFY_TYPE_SOFTWARE_ABNORMAL;
	chip->uisoc_keep_2_err_trigger->flag_reason = TRACK_NOTIFY_FLAG_UISOC_KEEP_2_ERROR;

	index += scnprintf(&(chip->uisoc_keep_2_err_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$err_scene@@%s", "uisoc_keep_2_err");

	index += scnprintf(&(chip->uisoc_keep_2_err_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$err_reason@@%s", "default");

	index += scnprintf(&(chip->uisoc_keep_2_err_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			"$$soc@@%d$$smooth_soc@@%d$$uisoc@@%d$$vbatt_max@@%d$$vbatt_min@@%d"
			"$$batt_rm@@%d$$batt_fcc@@%d$$batt_cc@@%d$$batt_curr@@%d",
			monitor->batt_soc, monitor->smooth_soc, monitor->ui_soc,
			monitor->vbat_mv, monitor->vbat_min_mv,
			monitor->batt_rm, monitor->batt_fcc, monitor->batt_cc, monitor->ibat_ma);

	oplus_chg_track_upload_trigger_data(chip->uisoc_keep_2_err_trigger);
	kfree(chip->uisoc_keep_2_err_trigger);
	chip->uisoc_keep_2_err_trigger = NULL;
}

void oplus_chg_track_upload_uisoc_keep_2_err_info(struct oplus_monitor *monitor)
{
	if (monitor->track != NULL)
		schedule_delayed_work(&monitor->track->uisoc_keep_2_err_trigger_work, 0);
}

static void oplus_chg_track_uisoc_keep_3_err_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(dwork, struct oplus_chg_track, uisoc_keep_3_err_trigger_work);
	int index = 0;
	struct oplus_monitor *monitor = chip->monitor;

	if (chip->uisoc_keep_3_err_trigger)
		kfree(chip->uisoc_keep_3_err_trigger);

	chip->uisoc_keep_3_err_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->uisoc_keep_3_err_trigger) {
		chg_err("uisoc_keep_3_err_trigger memery alloc fail\n");
		return;
	}

	chip->uisoc_keep_3_err_trigger->type_reason = TRACK_NOTIFY_TYPE_SOFTWARE_ABNORMAL;
	chip->uisoc_keep_3_err_trigger->flag_reason = TRACK_NOTIFY_FLAG_UISOC_KEEP_3_ERROR;

	index += scnprintf(&(chip->uisoc_keep_3_err_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$err_scene@@%s", "uisoc_keep_3_err");

	index += scnprintf(&(chip->uisoc_keep_3_err_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$err_reason@@%s", "default");

	index += scnprintf(&(chip->uisoc_keep_3_err_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			"$$soc@@%d$$smooth_soc@@%d$$uisoc@@%d$$vbatt_max@@%d$$vbatt_min@@%d"
			"$$batt_rm@@%d$$batt_fcc@@%d$$batt_cc@@%d$$batt_curr@@%d",
			monitor->batt_soc, monitor->smooth_soc, monitor->ui_soc,
			monitor->vbat_mv, monitor->vbat_min_mv,
			monitor->batt_rm, monitor->batt_fcc, monitor->batt_cc, monitor->ibat_ma);

	oplus_chg_track_upload_trigger_data(chip->uisoc_keep_3_err_trigger);
	kfree(chip->uisoc_keep_3_err_trigger);
	chip->uisoc_keep_3_err_trigger = NULL;
}

void oplus_chg_track_upload_uisoc_keep_3_err_info(struct oplus_monitor *monitor)
{
	if (monitor->track != NULL)
		schedule_delayed_work(&monitor->track->uisoc_keep_3_err_trigger_work, 0);
}

static void oplus_chg_track_uisoc_drop_err_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(dwork, struct oplus_chg_track, uisoc_drop_err_trigger_work);

	if (!chip)
		return;

	chip->uisoc_drop_err_trigger.type_reason = TRACK_NOTIFY_TYPE_SOFTWARE_ABNORMAL;
	chip->uisoc_drop_err_trigger.flag_reason = TRACK_NOTIFY_FLAG_UISOC_DROP_ERROR;

	chg_info("%s\n", chip->uisoc_drop_err_trigger.crux_info);
	oplus_chg_track_upload_trigger_data(&chip->uisoc_drop_err_trigger);
}

static void update_endurance_track_info(struct oplus_monitor *monitor, struct endurance_track_info *info)
{
	struct rtc_time tm;

	info->time = oplus_chg_track_get_current_time_s(&tm);
	info->batt_temp = monitor->batt_temp;
	info->batt_rm = monitor->batt_rm;
	info->soc = monitor->batt_soc;
	info->ui_soc = monitor->ui_soc;
	info->vol_max = monitor->vbat_mv;
	info->vol_min = monitor->vbat_min_mv;
	info->batt_fcc = monitor->batt_fcc;
}

static void update_endurance_exit_reason(struct oplus_monitor *monitor)
{
	if (monitor->sem_info.uisoc_0)
		monitor->sem_info.exit_reason = ENDURANCE_EXIT_SHUTDOWN;
	else if ((monitor->wired_online || monitor->wls_online) &&
		 (monitor->batt_status == POWER_SUPPLY_STATUS_CHARGING ||
		  monitor->batt_status == POWER_SUPPLY_STATUS_FULL))
		monitor->sem_info.exit_reason = ENDURANCE_EXIT_CHARGING;
	else
		monitor->sem_info.exit_reason = ENDURANCE_EXIT_USER;
}

static void upload_endurance_info(struct oplus_chg_track *chip, struct oplus_monitor *monitor)
{
	int rc = 0;
	int index = 0;
	union mms_msg_data data = { 0 };

	if (chip->endurance_info_trigger)
		kfree(chip->endurance_info_trigger);

	chip->endurance_info_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->endurance_info_trigger) {
		chg_err("endurance_info_trigger memery alloc fail\n");
		return;
	}

	chip->endurance_info_trigger->type_reason = TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	chip->endurance_info_trigger->flag_reason = TRACK_NOTIFY_FLAG_ENDURANCE_INFO;

	index += scnprintf(&(chip->endurance_info_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$start_soc@@%d$$end_soc@@%d"
			  "$$start_uisoc@@%d$$end_uisoc@@%d"
			  "$$start_vol_max@@%d$$end_vol_max@@%d"
			  "$$start_vol_min@@%d$$end_vol_min@@%d"
			  "$$start_batt_tmep@@%d$$end_batt_temp@@%d"
			  "$$start_batt_rm@@%d$$end_batt_rm@@%d"
			  "$$start_batt_fcc@@%d$$end_batt_fcc@@%d"
			  "$$duration_time@@%d$$count@@%d"
			  "$$exit_reason@@%s",
			  monitor->sem_info.start_info.soc, monitor->sem_info.end_info.soc,
			  monitor->sem_info.start_info.ui_soc, monitor->sem_info.end_info.ui_soc,
			  monitor->sem_info.start_info.vol_max, monitor->sem_info.end_info.vol_max,
			  monitor->sem_info.start_info.vol_min, monitor->sem_info.end_info.vol_min,
			  monitor->sem_info.start_info.batt_temp, monitor->sem_info.end_info.batt_temp,
			  monitor->sem_info.start_info.batt_rm, monitor->sem_info.end_info.batt_rm,
			  monitor->sem_info.start_info.batt_fcc, monitor->sem_info.end_info.batt_fcc,
			  monitor->sem_info.duration_time, monitor->sem_info.count,
			  endurance_exit_reason_str(monitor->sem_info.exit_reason));

	rc = oplus_mms_get_item_data(monitor->gauge_topic, GAUGE_ITEM_REG_INFO, &data, true);
	if (rc == 0 && data.strval && strlen(data.strval))
		index += scnprintf(&(chip->endurance_info_trigger->crux_info[index]),
				  OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$main_reg_info@@%s", data.strval);

	oplus_chg_track_upload_trigger_data(chip->endurance_info_trigger);
	kfree(chip->endurance_info_trigger);
	chip->endurance_info_trigger = NULL;
}

static void oplus_chg_track_endurance_change_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(dwork, struct oplus_chg_track, endurance_change_work);
	struct oplus_monitor *monitor = chip->monitor;

	chg_info("endurance status %d -> %d\n", monitor->sem_info.pre_status, monitor->sem_info.status);

	if (monitor->sem_info.status && !monitor->sem_info.pre_status) { /* off to on */
		update_endurance_track_info(monitor, &monitor->sem_info.start_info);
		monitor->sem_info.pre_status = monitor->sem_info.status;
		return;
	}

	if (monitor->sem_info.pre_status &&
	    (!monitor->sem_info.status || monitor->sem_info.uisoc_0)) { /* on to off/ 0% */
		update_endurance_track_info(monitor, &monitor->sem_info.end_info);
		monitor->sem_info.duration_time = monitor->sem_info.end_info.time - monitor->sem_info.start_info.time;
		update_endurance_exit_reason(monitor);
		upload_endurance_info(chip, monitor);
	}

	monitor->sem_info.pre_status = monitor->sem_info.status;
}

void oplus_chg_track_super_endurance_mode_change(struct oplus_monitor *monitor)
{
	if (monitor->track != NULL && monitor->deep_support)
		schedule_delayed_work(&monitor->track->endurance_change_work, 0);
}

static int oplus_chg_track_speed_ref_init(struct oplus_chg_track *chip)
{
	if (!chip)
		return -1;

	if (chip->track_cfg.wired_fast_chg_scheme >= 0 &&
	    chip->track_cfg.wired_fast_chg_scheme <
		    ARRAY_SIZE(g_wired_speed_ref_standard))
		chip->track_status.wired_speed_ref = g_wired_speed_ref_standard
			[chip->track_cfg.wired_fast_chg_scheme];

	if (chip->track_cfg.wls_fast_chg_scheme >= 0 &&
	    chip->track_cfg.wls_fast_chg_scheme <
		    ARRAY_SIZE(g_wls_fast_speed_ref_standard))
		chip->track_status.wls_airvooc_speed_ref =
			g_wls_fast_speed_ref_standard
				[chip->track_cfg.wls_fast_chg_scheme];

	if (chip->track_cfg.wls_epp_chg_scheme >= 0 &&
	    chip->track_cfg.wls_epp_chg_scheme <
		    ARRAY_SIZE(g_wls_epp_speed_ref_standard))
		chip->track_status.wls_epp_speed_ref =
			g_wls_epp_speed_ref_standard
				[chip->track_cfg.wls_epp_chg_scheme];

	if (chip->track_cfg.wls_bpp_chg_scheme >= 0 &&
	    chip->track_cfg.wls_bpp_chg_scheme <
		    ARRAY_SIZE(g_wls_bpp_speed_ref_standard))
		chip->track_status.wls_bpp_speed_ref =
			g_wls_bpp_speed_ref_standard
				[chip->track_cfg.wls_bpp_chg_scheme];

	return 0;
}

static int oplus_chg_track_gague_fifo_init(struct oplus_chg_track *track_dev)
{
	int rc = 0;

	if (track_dev->track_cfg.external_gauge_num) {
		rc = kfifo_alloc(&(track_dev->gauge_info.fifo),
			(GAUGE_INFO_TRACK_FIFO_NUMS * GAUGE_INFO_TRACK_FIFO_ONE_SIZE), GFP_KERNEL);
		if (rc) {
			pr_err("gauge kfifo_alloc error\n");
			rc = -ENOMEM;
			return rc;
		}
		if (track_dev->track_cfg.external_gauge_num == 2) {
			rc = kfifo_alloc(&(track_dev->sub_gauge_info.fifo),
				(GAUGE_INFO_TRACK_FIFO_NUMS * GAUGE_INFO_TRACK_FIFO_ONE_SIZE), GFP_KERNEL);
			if (rc) {
				kfifo_free(&(track_dev->gauge_info.fifo));
				pr_err("sub gauge kfifo_alloc error\n");
				rc = -ENOMEM;
				return rc;
			}
		}
	}

	return rc;
}


static void oplus_chg_track_rechg_info_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(dwork, struct oplus_chg_track, rechg_info_trigger_work);
	struct oplus_monitor *monitor = chip->monitor;
	int index = 0;

	mutex_lock(&chip->rechg_info_lock);
	if (chip->rechg_info_trigger)
		kfree(chip->rechg_info_trigger);

	chip->rechg_info_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->rechg_info_trigger) {
		chg_err("rechg_info_trigger memery alloc fail\n");
		mutex_unlock(&chip->rechg_info_lock);
		return;
	}

	chip->rechg_info_trigger->type_reason = TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	chip->rechg_info_trigger->flag_reason = TRACK_NOTIFY_FLAG_RECHG_INFO;

	index += scnprintf(&(chip->rechg_info_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			"$$rechging@@%d$$rechg_soc_en@@%d$$rechg_soc_thld@@%d$$uisoc@@%d$$soc@@%d$$vbatt_max@@%d$$charge_status@@%d",
			monitor->rechging, monitor->rechg_soc_en, monitor->rechg_soc_threshold, monitor->ui_soc,
			monitor->batt_soc, monitor->vbat_mv, monitor->batt_status);

	oplus_chg_track_upload_trigger_data(chip->rechg_info_trigger);
	kfree(chip->rechg_info_trigger);
	chip->rechg_info_trigger = NULL;

	mutex_unlock(&chip->rechg_info_lock);
}

int oplus_chg_track_upload_rechg_info(struct oplus_monitor *monitor)
{
	struct oplus_chg_track *chip;

	if (monitor == NULL) {
		chg_err("monitor is NULL\n");
		return -ENODEV;
	}
	chip = monitor->track;
	if (!chip)
		return -EINVAL;

	schedule_delayed_work(&chip->rechg_info_trigger_work, 0);
	chg_info("success\n");
	return 0;
}

static int oplus_chg_track_init(struct oplus_chg_track *track_dev)
{
	int ret = 0;
	struct oplus_chg_track *chip = track_dev;

	chip->trigger_data_ok = false;
	mutex_init(&chip->slowchg_info.lock);
	mutex_init(&chip->cp_abnormal_info.lock);
	mutex_init(&chip->upload_lock);
	mutex_init(&chip->trigger_data_lock);
	mutex_init(&chip->trigger_ack_lock);
	init_waitqueue_head(&chip->upload_wq);
	init_completion(&chip->trigger_ack);
	mutex_init(&track_dev->dcs_info_lock);
	mutex_init(&chip->access_lock);
	mutex_init(&chip->track_status.app_status.app_lock);
	mutex_init(&chip->mmi_chg_info_lock);
	mutex_init(&chip->slow_chg_info_lock);
	mutex_init(&chip->chg_cycle_info_lock);
	mutex_init(&chip->wls_info_lock);
	mutex_init(&chip->ufcs_info_lock);
	mutex_init(&chip->pps_info_lock);
	mutex_init(&chip->deep_dischg_info_lock);
	mutex_init(&chip->gauge_info.track_lock);
	mutex_init(&chip->sub_gauge_info.track_lock);
	mutex_init(&chip->rechg_info_lock);
	mutex_init(&chip->eis_timeout_info_lock);
	mutex_init(&chip->gauge_info.sili_alg_application_lock);
	mutex_init(&chip->sub_gauge_info.sili_alg_application_lock);
	mutex_init(&chip->gauge_info.sili_alg_monitor_lock);
	mutex_init(&chip->sub_gauge_info.sili_alg_monitor_lock);
	mutex_init(&chip->gauge_info.sili_alg_lifetime_lock);
	mutex_init(&chip->bal_info_lock);
	mutex_init(&chip->sub_gauge_info.sili_alg_lifetime_lock);
	mutex_init(&chip->bidirect_cp_info_lock);
	mutex_init(&chip->plc_info_lock);
	mutex_init(&(chip->no_charging_info_lock));
	mutex_init(&chip->phy_cp_info_lock);
	mutex_init(&chip->soccp_crash_lock);
	mutex_init(&chip->gauge_info.batt_monitor_lock);
	mutex_init(&chip->sub_gauge_info.batt_monitor_lock);
	mutex_init(&chip->bs_info_lock);
	mutex_init(&chip->state_keep_info_lock);
	mutex_init(&chip->usbin_abnormal_lock);

	chip->gauge_info.debug_err_type = TRACK_GAGUE_ERR_DEFAULT;
	chip->gauge_info.debug_upload_period_t = 0;
	chip->gauge_info.debug_soc_record_thd= 0;
	chip->sub_gauge_info.debug_err_type = TRACK_GAGUE_ERR_DEFAULT;
	chip->sub_gauge_info.debug_upload_period_t = 0;
	chip->sub_gauge_info.debug_soc_record_thd= 0;

	chip->track_status.curr_soc = -EINVAL;
	chip->track_status.curr_smooth_soc = -EINVAL;
	chip->track_status.curr_uisoc = -EINVAL;
	chip->track_status.pre_soc = -EINVAL;
	chip->track_status.pre_smooth_soc = -EINVAL;
	chip->track_status.pre_uisoc = -EINVAL;
	chip->track_status.soc_jumped = false;
	chip->track_status.uisoc_jumped = false;
	chip->track_status.uisoc_to_soc_jumped = false;
	chip->track_status.uisoc_load_jumped = false;
	chip->track_status.debug_soc = OPLUS_CHG_TRACK_DEBUG_UISOC_SOC_INVALID;
	chip->track_status.debug_uisoc =
		OPLUS_CHG_TRACK_DEBUG_UISOC_SOC_INVALID;
	chip->track_status.debug_close_3hours = false;
	chip->uisoc_load_trigger.type_reason = TRACK_NOTIFY_TYPE_SOC_JUMP;
	chip->uisoc_load_trigger.flag_reason =
		TRACK_NOTIFY_FLAG_UI_SOC_LOAD_JUMP;
	chip->soc_trigger.type_reason = TRACK_NOTIFY_TYPE_SOC_JUMP;
	chip->soc_trigger.flag_reason = TRACK_NOTIFY_FLAG_SOC_JUMP;
	chip->uisoc_trigger.type_reason = TRACK_NOTIFY_TYPE_SOC_JUMP;
	chip->uisoc_trigger.flag_reason = TRACK_NOTIFY_FLAG_UI_SOC_JUMP;
	chip->uisoc_to_soc_trigger.type_reason = TRACK_NOTIFY_TYPE_SOC_JUMP;
	chip->uisoc_to_soc_trigger.flag_reason =
		TRACK_NOTIFY_FLAG_UI_SOC_TO_SOC_JUMP;
	chip->charger_info_trigger.type_reason =
		TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	chip->charger_info_trigger.flag_reason = TRACK_NOTIFY_FLAG_CHARGER_INFO;
	chip->no_charging_trigger.type_reason = TRACK_NOTIFY_TYPE_NO_CHARGING;
	chip->no_charging_trigger.flag_reason = TRACK_NOTIFY_FLAG_NO_CHARGING;
	chip->slow_charging_trigger.type_reason =
		TRACK_NOTIFY_TYPE_CHARGING_SLOW;
	chip->charging_break_trigger.type_reason =
		TRACK_NOTIFY_TYPE_CHARGING_BREAK;
	chip->wls_charging_break_trigger.type_reason =
		TRACK_NOTIFY_TYPE_CHARGING_BREAK;
	chip->uisoc_keep_1_t_load_trigger.type_reason =
		TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	chip->uisoc_keep_1_t_load_trigger.flag_reason =
		TRACK_NOTIFY_FLAG_UISOC_KEEP_1_T_INFO;
	chip->vbatt_too_low_load_trigger.type_reason =
		TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	chip->vbatt_too_low_load_trigger.flag_reason =
		TRACK_NOTIFY_FLAG_VBATT_TOO_LOW_INFO;
	chip->usbtemp_load_trigger.type_reason =
		TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	chip->usbtemp_load_trigger.flag_reason = TRACK_NOTIFY_FLAG_USBTEMP_INFO;
	chip->usb_lpd_load_trigger.type_reason =
		TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	chip->usb_lpd_load_trigger.flag_reason = TRACK_NOTIFY_FLAG_LPD_INFO;
	chip->vbatt_diff_over_load_trigger.type_reason =
		TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	chip->vbatt_diff_over_load_trigger.flag_reason =
		TRACK_NOTIFY_FLAG_VBATT_DIFF_OVER_INFO;
	chip->ic_err_msg_load_trigger.type_reason =
		TRACK_NOTIFY_TYPE_DEVICE_ABNORMAL;
	chip->ic_err_msg_load_trigger.flag_reason =
		TRACK_NOTIFY_FLAG_DEFAULT;
	chip->chg_into_liquid_load_trigger.type_reason =
		TRACK_NOTIFY_TYPE_DEVICE_ABNORMAL;
	chip->chg_into_liquid_load_trigger.flag_reason =
		TRACK_NOTIFY_FLAG_CHG_FEED_LIQUOR;
	chip->dual_chan_err_load_trigger.type_reason =
		TRACK_NOTIFY_TYPE_SOFTWARE_ABNORMAL;
	chip->dual_chan_err_load_trigger.flag_reason =
		TRACK_NOTIFY_FLAG_DUAL_CHAN_ABNORMAL;

	memset(&(chip->track_status.power_info), 0,
	       sizeof(chip->track_status.power_info));
	strcpy(chip->track_status.power_info.power_mode, "unknow");
	chip->track_status.wired_max_power = chip->track_cfg.wired_max_power;
	chip->track_status.wls_max_power = chip->track_cfg.wls_max_power;
	chip->track_status.chg_no_charging_cnt = 0;
	chip->track_status.chg_total_cnt = 0;
	chip->track_status.chg_no_chg_time_s = 0;
	chip->track_status.no_charging_cal_curr_time = 0;
	chip->track_status.no_charging_cal_pre_time = 0;
	chip->track_status.chg_total_time = 0;
	chip->track_status.chg_max_temp = 0;
	chip->track_status.chg_fast_full_time = 0;
	chip->track_status.chg_normal_full_time = 0;
	chip->track_status.chg_report_full_time = 0;
	chip->track_status.debug_normal_charging_state =
		POWER_SUPPLY_STATUS_CHARGING;
	chip->track_status.debug_fast_prop_status = TRACK_FASTCHG_STATUS_UNKOWN;
	chip->track_status.debug_normal_prop_status =
		POWER_SUPPLY_STATUS_UNKNOWN;
	chip->track_status.debug_no_charging = 0;
	chip->track_status.chg_five_mins_cap = TRACK_PERIOD_CHG_CAP_INIT;
	chip->track_status.chg_ten_mins_cap = TRACK_PERIOD_CHG_CAP_INIT;
	chip->track_status.chg_twenty_mins_cap = TRACK_PERIOD_CHG_CAP_INIT;
	chip->track_status.chg_thirty_mins_cap = TRACK_PERIOD_CHG_CAP_INIT;
	chip->track_status.chg_average_speed =
		TRACK_PERIOD_CHG_AVERAGE_SPEED_INIT;
	chip->track_status.chg_attach_time_ms =
		chip->track_cfg.fast_chg_break_t_thd;
	chip->track_status.chg_detach_time_ms = 0;
	chip->track_status.wls_attach_time_ms =
		chip->track_cfg.wls_chg_break_t_thd;
	chip->track_status.wls_detach_time_ms = 0;
	chip->track_status.soc_sect_status = TRACK_SOC_SECTION_DEFAULT;
	chip->track_status.chg_speed_is_slow = false;
	chip->track_status.tbatt_warm_once = false;
	chip->track_status.tbatt_cold_once = false;
	chip->track_status.cool_down_effect_cnt = 0;
	chip->track_status.wls_need_upload = false;
	chip->track_status.wired_need_upload = false;

	chip->track_status.app_status.app_cal = false;
	chip->track_status.app_status.curr_top_index = TRACK_APP_TOP_INDEX_DEFAULT;
	strncpy(chip->track_status.app_status.curr_top_name,
		TRACK_APP_REAL_NAME_DEFAULT, TRACK_APP_REAL_NAME_LEN - 1);
	chip->track_status.once_mmi_chg = false;
	chip->track_status.once_chg_cycle_status = CHG_CYCLE_VOTER__NONE;
	chip->track_status.mmi_chg_open_t = 0;
	chip->track_status.mmi_chg_close_t = 0;
	chip->track_status.mmi_chg_constant_t = 0;

	chip->track_status.usbtemp_is_high = false;

	chip->track_status.slow_chg_open_t = 0;
	chip->track_status.slow_chg_close_t = 0;
	chip->track_status.slow_chg_open_n_t = 0;
	chip->track_status.slow_chg_duration = 0;
	chip->track_status.slow_chg_open_cnt = 0;
	chip->track_status.slow_chg_watt = 0;
	chip->track_status.slow_chg_pct = 0;

	chip->track_status.debug_plugout_state = 0;
	chip->track_status.debug_break_code = 0;
	chip->track_status.aging_ffc_start_time = 0;
	chip->track_status.ffc_end_time = 0;
	chip->track_status.ffc_time = 0;
	chip->track_status.cv_time = 0;
	chip->track_status.ffc_start_main_soc = 0;
	chip->track_status.ffc_start_sub_soc = 0;
	chip->track_status.ffc_end_main_soc = 0;
	chip->track_status.ffc_end_sub_soc = 0;
	chip->track_status.aging_ffc_to_full_time = 0;
	chip->track_status.dual_chan_start_time = 0;
	chip->track_status.dual_chan_time = 0;
	chip->track_status.dual_chan_open_count = 0;

	memset(&(chip->track_status.fastchg_break_info), 0,
	       sizeof(chip->track_status.fastchg_break_info));
	memset(&(chip->track_status.wired_break_crux_info), 0,
	       sizeof(chip->track_status.wired_break_crux_info));
	memset(&(chip->track_status.wls_break_crux_info), 0,
	       sizeof(chip->track_status.wls_break_crux_info));

	memset(&(chip->track_status.batt_full_reason), 0,
	       sizeof(chip->track_status.batt_full_reason));
	oplus_chg_track_clear_cool_down_stats_time(&(chip->track_status));

	memset(&(chip->voocphy_name), 0, sizeof(chip->voocphy_name));
	oplus_chg_track_set_voocphy_name(chip);
	oplus_chg_track_speed_ref_init(chip);
	oplus_chg_track_slowchg_info_reset(&chip->slowchg_info);

	INIT_DELAYED_WORK(&chip->uisoc_load_trigger_work,
			  oplus_chg_track_uisoc_load_trigger_work);
	INIT_DELAYED_WORK(&chip->soc_trigger_work,
			  oplus_chg_track_soc_trigger_work);
	INIT_DELAYED_WORK(&chip->uisoc_trigger_work,
			  oplus_chg_track_uisoc_trigger_work);
	INIT_DELAYED_WORK(&chip->uisoc_to_soc_trigger_work,
			  oplus_chg_track_uisoc_to_soc_trigger_work);
	INIT_DELAYED_WORK(&chip->charger_info_trigger_work,
			  oplus_chg_track_charger_info_trigger_work);
	INIT_DELAYED_WORK(&chip->cal_chg_five_mins_capacity_work,
			  oplus_chg_track_cal_chg_five_mins_capacity_work);
	INIT_DELAYED_WORK(&chip->cal_chg_ten_mins_capacity_work,
			  oplus_chg_track_cal_chg_ten_mins_capacity_work);
	INIT_DELAYED_WORK(&chip->cal_chg_twenty_mins_capacity_work, oplus_chg_track_cal_chg_twenty_mins_capacity_work);
	INIT_DELAYED_WORK(&chip->cal_chg_thirty_mins_capacity_work, oplus_chg_track_cal_chg_thirty_mins_capacity_work);
	INIT_DELAYED_WORK(&chip->no_charging_trigger_work,
			  oplus_chg_track_no_charging_trigger_work);
	INIT_DELAYED_WORK(&chip->slow_charging_trigger_work,
			  oplus_chg_track_slow_charging_trigger_work);
	INIT_DELAYED_WORK(&chip->charging_break_trigger_work,
			  oplus_chg_track_charging_break_trigger_work);
	INIT_DELAYED_WORK(&chip->wls_charging_break_trigger_work,
			  oplus_chg_track_wls_charging_break_trigger_work);
	INIT_DELAYED_WORK(&chip->usbtemp_load_trigger_work,
			  oplus_chg_track_usbtemp_load_trigger_work);
	INIT_DELAYED_WORK(&chip->usb_lpd_load_trigger_work,
			  oplus_chg_track_usb_lpd_load_trigger_work);
	INIT_DELAYED_WORK(&chip->usbtemp_trigger_no_charging_work,
			  oplus_chg_track_usbtemp_trigger_no_charging_work);
	INIT_DELAYED_WORK(&chip->vbatt_too_low_load_trigger_work,
			  oplus_chg_track_vbatt_too_low_load_trigger_work);
	INIT_DELAYED_WORK(&chip->vbatt_diff_over_load_trigger_work,
			  oplus_chg_track_vbatt_diff_over_load_trigger_work);
	INIT_DELAYED_WORK(&chip->uisoc_keep_1_t_load_trigger_work,
			  oplus_chg_track_uisoc_keep_1_t_load_trigger_work);
	INIT_DELAYED_WORK(&chip->ic_err_msg_trigger_work,
			  oplus_chg_track_ic_err_msg_trigger_work);
	INIT_DELAYED_WORK(&chip->chg_into_liquid_trigger_work,
			  oplus_chg_track_chg_into_liquid_trigger_work);
	INIT_DELAYED_WORK(&chip->plugout_state_work,
			  oplus_chg_track_check_plugout_work);
	INIT_DELAYED_WORK(&chip->dual_chan_err_load_trigger_work,
			  oplus_chg_track_dual_chan_err_load_trigger_work);
	INIT_DELAYED_WORK(&chip->mmi_chg_info_trigger_work, oplus_chg_track_mmi_chg_info_trigger_work);
	INIT_DELAYED_WORK(&chip->slow_chg_info_trigger_work, oplus_chg_track_slow_chg_info_trigger_work);
	INIT_DELAYED_WORK(&chip->chg_cycle_info_trigger_work, oplus_chg_track_chg_cycle_info_trigger_work);
	INIT_DELAYED_WORK(&chip->wls_info_trigger_work, oplus_chg_track_wls_info_trigger_work);
	INIT_DELAYED_WORK(&chip->ufcs_info_trigger_work, oplus_chg_track_ufcs_info_trigger_work);
	INIT_DELAYED_WORK(&chip->pps_info_trigger_work, oplus_chg_track_pps_info_trigger_work);
	INIT_DELAYED_WORK(&chip->deep_dischg_info_trigger_work, oplus_chg_track_deep_dischg_info_trigger_work);
	INIT_DELAYED_WORK(&chip->eis_timeout_info_trigger_work, oplus_chg_track_eis_timeout_info_trigger_work);
	INIT_DELAYED_WORK(&chip->wired_online_err_trigger_work, oplus_chg_track_wired_online_err_trigger_work);
	INIT_DELAYED_WORK(&chip->uisoc_keep_2_err_trigger_work, oplus_chg_track_uisoc_keep_2_err_trigger_work);
	INIT_DELAYED_WORK(&chip->uisoc_keep_3_err_trigger_work, oplus_chg_track_uisoc_keep_3_err_trigger_work);
	INIT_DELAYED_WORK(&chip->uisoc_drop_err_trigger_work, oplus_chg_track_uisoc_drop_err_trigger_work);
	INIT_DELAYED_WORK(&chip->rechg_info_trigger_work, oplus_chg_track_rechg_info_trigger_work);
	INIT_DELAYED_WORK(&track_dev->gauge_info.sili_alg_application_load_trigger_work,
		oplus_chg_track_gauge_sili_alg_application_work);
	INIT_DELAYED_WORK(&track_dev->sub_gauge_info.sili_alg_application_load_trigger_work,
		oplus_chg_track_sub_gauge_sili_alg_application_work);
	INIT_DELAYED_WORK(&track_dev->gauge_info.sili_alg_monitor_load_trigger_work,
		oplus_chg_track_gauge_sili_alg_monitor_work);
	INIT_DELAYED_WORK(&track_dev->sub_gauge_info.sili_alg_monitor_load_trigger_work,
		oplus_chg_track_sub_gauge_sili_alg_monitor_work);
	INIT_DELAYED_WORK(&track_dev->gauge_info.sili_alg_lifetime_load_trigger_work,
		oplus_chg_track_gauge_sili_alg_lifetime_work);
	INIT_DELAYED_WORK(&track_dev->sub_gauge_info.sili_alg_lifetime_load_trigger_work,
		oplus_chg_track_sub_gauge_sili_alg_lifetime_work);
	INIT_DELAYED_WORK(&track_dev->gauge_info.batt_monitor_load_trigger_work,
		oplus_chg_track_gauge_batt_monitor_work);
	INIT_DELAYED_WORK(&track_dev->sub_gauge_info.batt_monitor_load_trigger_work,
		oplus_chg_track_sub_gauge_batt_monitor_work);
	INIT_DELAYED_WORK(&chip->wired_high_reverse_err_trigger_work,
		oplus_chg_track_wired_high_reverse_err_trigger_work);
	INIT_DELAYED_WORK(&chip->endurance_change_work, oplus_chg_track_endurance_change_work);
	INIT_DELAYED_WORK(&chip->bidirect_cp_info_trigger_work, oplus_chg_track_bidirect_cp_info_trigger_work);
	INIT_DELAYED_WORK(&chip->wired_retention_online_trigger_work,
		oplus_chg_track_wired_retention_online_trigger_work);
	INIT_DELAYED_WORK(&chip->bal_info_trigger_work, oplus_chg_track_bal_info_trigger_work);
	INIT_DELAYED_WORK(&chip->plc_info_trigger_work, oplus_chg_track_plc_info_trigger_work);
	INIT_DELAYED_WORK(&chip->phy_cp_info_trigger_work, oplus_chg_track_phy_cp_info_trigger_work);
	INIT_DELAYED_WORK(&chip->soccp_crash_trigger_work, oplus_chg_track_soccp_crash_trigger_work);
	INIT_DELAYED_WORK(&chip->bs_info_trigger_work, oplus_chg_track_bs_info_trigger_work);
	INIT_DELAYED_WORK(&chip->state_keep_info_trigger_work, oplus_chg_track_state_keep_info_trigger_work);
	INIT_DELAYED_WORK(&chip->usbin_abnormal_trigger_work, oplus_chg_track_usbin_abnormal_trigger_work);
	INIT_DELAYED_WORK(&chip->sec_ic_meminfo_trigger_work, oplus_chg_track_sec_ic_meminfo_trigger_work);
	INIT_DELAYED_WORK(&chip->wired_reverse_chg_trigger_work, oplus_chg_track_wired_reverse_chg_trigger_work);

	return ret;
}

#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || \
	defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE) || \
	defined(CONFIG_OPLUS_KEVENT_UPLOAD)
static int oplus_chg_track_get_type_tag(int type_reason, char *type_reason_tag)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(track_type_reason_table); i++) {
		if (track_type_reason_table[i].type_reason == type_reason) {
			strncpy(type_reason_tag,
				track_type_reason_table[i].type_reason_tag,
				OPLUS_CHG_TRIGGER_REASON_TAG_LEN - 1);
			break;
		}
	}
	return i;
}

static int oplus_chg_track_get_flag_tag(int flag_reason, char *flag_reason_tag)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(track_flag_reason_table); i++) {
		if (track_flag_reason_table[i].flag_reason == flag_reason) {
			strncpy(flag_reason_tag,
				track_flag_reason_table[i].flag_reason_tag,
				OPLUS_CHG_TRIGGER_REASON_TAG_LEN - 1);
			break;
		}
	}
	return i;
}
#else
static int oplus_chg_track_get_flag_tag(int flag_reason, char *flag_reason_tag)
{
	return 0;
}
#endif

static bool oplus_chg_track_trigger_data_is_valid(oplus_chg_track_trigger *pdata)
{
	int i;
	int len;
	bool ret = false;
	int type_reason = pdata->type_reason;
	int flag_reason = pdata->flag_reason;

	len = strlen(pdata->crux_info);
	if (!len) {
		chg_err("crux_info lens is invaild\n");
		return ret;
	}

	switch (type_reason) {
	case TRACK_NOTIFY_TYPE_SOC_JUMP:
		for (i = TRACK_NOTIFY_FLAG_SOC_JUMP_FIRST; i <= TRACK_NOTIFY_FLAG_SOC_JUMP_LAST; i++) {
			if (flag_reason == i) {
				ret = true;
				break;
			}
		}
		break;
	case TRACK_NOTIFY_TYPE_GENERAL_RECORD:
		for (i = TRACK_NOTIFY_FLAG_GENERAL_RECORD_FIRST; i <= TRACK_NOTIFY_FLAG_GENERAL_RECORD_LAST; i++) {
			if (flag_reason == i) {
				ret = true;
				break;
			}
		}
		break;
	case TRACK_NOTIFY_TYPE_NO_CHARGING:
		for (i = TRACK_NOTIFY_FLAG_NO_CHARGING_FIRST; i <= TRACK_NOTIFY_FLAG_NO_CHARGING_LAST; i++) {
			if (flag_reason == i) {
				ret = true;
				break;
			}
		}
		break;
	case TRACK_NOTIFY_TYPE_CHARGING_SLOW:
		for (i = TRACK_NOTIFY_FLAG_CHARGING_SLOW_FIRST; i <= TRACK_NOTIFY_FLAG_CHARGING_SLOW_LAST; i++) {
			if (flag_reason == i) {
				ret = true;
				break;
			}
		}
		break;
	case TRACK_NOTIFY_TYPE_CHARGING_BREAK:
		for (i = TRACK_NOTIFY_FLAG_CHARGING_BREAK_FIRST; i <= TRACK_NOTIFY_FLAG_CHARGING_BREAK_LAST; i++) {
			if (flag_reason == i) {
				ret = true;
				break;
			}
		}
		break;
	case TRACK_NOTIFY_TYPE_DEVICE_ABNORMAL:
		for (i = TRACK_NOTIFY_FLAG_DEVICE_ABNORMAL_FIRST; i <= TRACK_NOTIFY_FLAG_DEVICE_ABNORMAL_LAST; i++) {
			if (flag_reason == i) {
				ret = true;
				break;
			}
		}
		break;
	case TRACK_NOTIFY_TYPE_SOFTWARE_ABNORMAL:
		for (i = TRACK_NOTIFY_FLAG_SOFTWARE_ABNORMAL_FIRST; i <= TRACK_NOTIFY_FLAG_SOFTWARE_ABNORMAL_LAST;
		     i++) {
			if (flag_reason == i) {
				ret = true;
				break;
			}
		}
		break;
	case TRACK_NOTIFY_TYPE_CRASH:
		for (i = TRACK_NOTIFY_FLAG_CRASH_FIRST; i <= TRACK_NOTIFY_FLAG_CRASH_LAST; i++) {
			if (flag_reason == i) {
				ret = true;
				break;
			}
		}
		break;
	default:
		ret = false;
		break;
	}

	if (!ret)
		chg_err("type_reason or flag_reason is invaild\n");
	return ret;
}

static int oplus_chg_track_upload_trigger_data(oplus_chg_track_trigger *data)
{
	int rc;
	struct oplus_chg_track *chip = g_track_chip;
	char flag_reason_tag[OPLUS_CHG_TRIGGER_REASON_TAG_LEN] = { 0 };
	oplus_chg_track_trigger *trigger_info = NULL;

	if (!g_track_chip || !data)
		return TRACK_CMD_ERROR_CHIP_NULL;

	trigger_info = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!trigger_info)
		return TRACK_CMD_ERROR_CHIP_NULL;
	memcpy(trigger_info, data, sizeof(oplus_chg_track_trigger));

	if (!oplus_chg_track_trigger_data_is_valid(trigger_info)) {
		kfree(trigger_info);
		return TRACK_CMD_ERROR_DATA_INVALID;
	}

	mutex_lock(&chip->trigger_ack_lock);
	mutex_lock(&chip->trigger_data_lock);
	memset(&chip->trigger_data, 0, sizeof(oplus_chg_track_trigger));
	chip->trigger_data.type_reason = trigger_info->type_reason;
	chip->trigger_data.flag_reason = trigger_info->flag_reason;
	strncpy(chip->trigger_data.crux_info, trigger_info->crux_info,
		OPLUS_CHG_TRACK_CURX_INFO_LEN - 1);
	kfree(trigger_info);
	chg_info("type_reason:%d, flag_reason:%d, crux_info[%s]\n",
		 chip->trigger_data.type_reason, chip->trigger_data.flag_reason,
		 chip->trigger_data.crux_info);
	chip->trigger_data_ok = true;
	oplus_chg_track_get_flag_tag(chip->trigger_data.flag_reason, flag_reason_tag);
	chg_exception_report(&chip->track_cfg.exception_data, chip->trigger_data.type_reason,
				chip->trigger_data.flag_reason, flag_reason_tag, sizeof(flag_reason_tag));
	mutex_unlock(&chip->trigger_data_lock);
	reinit_completion(&chip->trigger_ack);
	wake_up(&chip->upload_wq);

	rc = wait_for_completion_timeout(
		&chip->trigger_ack,
		msecs_to_jiffies(OPLUS_CHG_TRACK_WAIT_TIME_MS));
	if (!rc) {
		if (delayed_work_pending(&chip->upload_info_dwork))
			cancel_delayed_work_sync(&chip->upload_info_dwork);
		chg_err("Error, timed out upload trigger data\n");
		mutex_unlock(&chip->trigger_ack_lock);
		return TRACK_CMD_ERROR_TIME_OUT;
	}
	rc = 0;
	chg_info("success\n");
	mutex_unlock(&chip->trigger_ack_lock);

	return rc;
}

static int oplus_chg_track_thread(void *data)
{
	int rc = 0;
	struct oplus_chg_track *chip = (struct oplus_chg_track *)data;

	while (!kthread_should_stop()) {
		mutex_lock(&chip->upload_lock);
		rc = wait_event_interruptible(chip->upload_wq,
					      chip->trigger_data_ok);
		mutex_unlock(&chip->upload_lock);
		if (rc)
			return rc;
		if (!chip->trigger_data_ok)
			chg_err("oplus chg false wakeup, rc=%d\n", rc);
		mutex_lock(&chip->trigger_data_lock);
		chip->trigger_data_ok = false;
#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || \
	defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE) || \
	defined(CONFIG_OPLUS_KEVENT_UPLOAD)
		oplus_chg_track_pack_dcs_info(chip);
#endif
		chip->dwork_retry_cnt = OPLUS_CHG_TRACK_DWORK_RETRY_CNT;
		queue_delayed_work(chip->trigger_upload_wq,
				   &chip->upload_info_dwork, 0);
		mutex_unlock(&chip->trigger_data_lock);
	}

	return rc;
}

static int oplus_chg_track_thread_init(struct oplus_chg_track *track_dev)
{
	int rc = 0;
	struct oplus_chg_track *chip = track_dev;

	chip->track_upload_kthread = kthread_run(oplus_chg_track_thread, chip,
						 "track_upload_kthread");
	if (IS_ERR(chip->track_upload_kthread)) {
		chg_err("failed to create oplus_chg_track_thread\n");
		return -1;
	}

	return rc;
}

static int oplus_chg_track_get_current_time_s(struct rtc_time *tm)
{
	struct timespec ts;

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, tm);
	tm->tm_year = tm->tm_year + 1900;
	tm->tm_mon = tm->tm_mon + 1;
	return ts.tv_sec;
}

static int oplus_chg_track_get_current_time(struct rtc_time *tm)
{
	struct timespec ts;
	struct oplus_chg_track *track_chip;
	struct oplus_chg_track_status *track_status;

	track_chip = g_track_chip;
	track_status = &track_chip->track_status;

	if (!track_chip || !track_status)
		return -1;

	getnstimeofday(&ts);
	ts.tv_sec += track_status->track_gmtoff;
	rtc_time_to_tm(ts.tv_sec, tm);
	tm->tm_year = tm->tm_year + 1900;
	tm->tm_mon = tm->tm_mon + 1;

	return ts.tv_sec;
}

int oplus_chg_track_get_local_time_s(void)
{
	int local_time_s;

	local_time_s = local_clock() / TRACK_LOCAL_T_NS_TO_S_THD;
	chg_debug("local_time_s:%d\n", local_time_s);

	return local_time_s;
}

int oplus_chg_track_get_bidirect_cp_err_reason(int err_type, char *err_reason, int len)
{
	int i;
	int charge_index = -EINVAL;

	if (!err_reason || !len)
		return charge_index;

	for (i = 0; i < ARRAY_SIZE(bidirect_cp_err_reason_table); i++) {
		if (bidirect_cp_err_reason_table[i].err_type == err_type) {
			strncpy(err_reason, bidirect_cp_err_reason_table[i].err_name, len);
			charge_index = i;
			break;
		}
	}

	if (i == ARRAY_SIZE(bidirect_cp_err_reason_table))
		strncpy(err_reason, "unknow_err", len);

	return charge_index;
}

static int oplus_chg_track_handle_pps_pdo_info(struct oplus_chg_track *chip,
					       char *p_data, int *ret, int t_len) {
	int i = 0;
	pps_msg_data pps_data = {0};
	pd_msg_data pd_data = {0};

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -EINVAL;
	}

	if (p_data == NULL) {
                chg_err("p_data is NULL");
                return -EINVAL;
        }

	if (chip->pps_info.adapter_pdo == NULL) {
		chg_err("adapter_pdo is NULL");
		return -EINVAL;
	}

	if (ret == NULL) {
		chg_err("ret is NULL");
		return -EINVAL;
	}

	for (i = 0; i < PPS_PDO_MAX; i++) {
		pps_data = (pps_msg_data)chip->pps_info.adapter_pdo[i];
		if (pps_data.pdo_type
		    == USBPD_PDMSG_PDOTYPE_AUGMENTED) {
			*ret += scnprintf(&p_data[*ret], t_len - *ret,
					  "(%u %u-%umV %umA)", pps_data.pdo_type,
					  PPS_PDO_VOL_MIN(pps_data.min_voltage100mv),
					  PPS_PDO_VOL_MAX(pps_data.max_voltage100mv),
	                        	  PPS_PDO_CURR_MAX(pps_data.max_current50ma));
		} else if (chip->pps_info.adapter_pdo[i] > 0) {
			pd_data = (pd_msg_data)chip->pps_info.adapter_pdo[i];
			*ret += scnprintf(&p_data[*ret], t_len - *ret,
					  "(%u %umV %umA)", pd_data.pdo_type,
					  PD_PDO_VOL(pd_data.voltage_50mv),
					  PD_PDO_CURR_MAX(pd_data.max_current_10ma));
	        }
	}

	return 0;
}

/*
* track sub version
* 3: default version for chg track
* 3.1: compatible with historical keywords
* 3.2: add the pd_sdp type to solve the problem that the type is unknown
*      add match_power to judge the power match between the adapter and the mobile phone
*      add 44W/55W/88W/125W adapter id
*      add pd_svooc type
* 3.3: add app record track feature
* 3.4: update reserve soc track
*/
#define TRACK_VERSION	"3.4"

#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || \
	defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE) || \
	defined(CONFIG_OPLUS_KEVENT_UPLOAD)
static int oplus_chg_track_pack_dcs_info(struct oplus_chg_track *chip)
{
	int ret = 0;
	int len, index;
	struct rtc_time tm;
	char log_tag[] = OPLUS_CHG_TRACK_LOG_TAG;
	char event_id[] = OPLUS_CHG_TRACK_EVENT_ID;
	char *p_data = (char *)(chip->dcs_info);
	char type_reason_tag[OPLUS_CHG_TRIGGER_REASON_TAG_LEN] = { 0 };
	char flag_reason_tag[OPLUS_CHG_TRIGGER_REASON_TAG_LEN] = { 0 };
	char battery_type_str[OPLUS_BATTERY_TYPE_LEN] = { 0 };
	int rc = oplus_gauge_get_battery_type_str(battery_type_str);
	pps_msg_data pps_data = {0};

	if (rc != 0) {
		if (chip->monitor->deep_support)
			scnprintf(battery_type_str, OPLUS_BATTERY_TYPE_LEN,"silicon");
		else
			scnprintf(battery_type_str, OPLUS_BATTERY_TYPE_LEN,"graphite");
	}

	memset(p_data, 0x0, sizeof(char) * OPLUS_CHG_TRIGGER_MSG_LEN);
	ret += sizeof(struct kernel_packet_info);
	ret += scnprintf(&p_data[ret], OPLUS_CHG_TRIGGER_MSG_LEN - ret,
			OPLUS_CHG_TRACK_EVENT_ID);

	ret += scnprintf(&p_data[ret], OPLUS_CHG_TRIGGER_MSG_LEN - ret,
			"$$track_ver@@%s", track_get_version_str(chip->track_cfg.track_ver));

	ret += scnprintf(&p_data[ret], OPLUS_CHG_TRIGGER_MSG_LEN - ret, "$$battery_type@@%s", battery_type_str);

	oplus_chg_track_get_type_tag(chip->trigger_data.type_reason,
				     type_reason_tag);
	type_reason_tag[OPLUS_CHG_TRIGGER_REASON_TAG_LEN - 1] = 0;
	oplus_chg_track_get_flag_tag(chip->trigger_data.flag_reason,
				     flag_reason_tag);
	flag_reason_tag[OPLUS_CHG_TRIGGER_REASON_TAG_LEN - 1] = 0;
	ret += scnprintf(&p_data[ret], OPLUS_CHG_TRIGGER_MSG_LEN - ret,
			"$$type_reason@@%s", type_reason_tag);
	ret += scnprintf(&p_data[ret], OPLUS_CHG_TRIGGER_MSG_LEN - ret,
			"$$flag_reason@@%s", flag_reason_tag);

	oplus_chg_track_get_current_time(&tm);
	ret += scnprintf(&p_data[ret], OPLUS_CHG_TRIGGER_MSG_LEN - ret,
			"$$time@@[%04d-%02d-%02d %02d:%02d:%02d]", tm.tm_year,
			tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min,
			tm.tm_sec);

	if (is_pps_topic_available(chip->monitor)
	    && strstr(chip->trigger_data.crux_info, "pps") != NULL) {
		if (chip->trigger_data.flag_reason == TRACK_NOTIFY_FLAG_CHARGER_INFO) {
			ret += scnprintf(&p_data[ret],
					 OPLUS_CHG_TRIGGER_MSG_LEN - ret,
					 "$$pps_adapter_info");
			if (chip->pps_info.adapter_pdo != NULL) {
				ret += scnprintf(&p_data[ret],
						 OPLUS_CHG_TRIGGER_MSG_LEN - ret,
						 "@@[");
				oplus_chg_track_handle_pps_pdo_info(chip, p_data, &ret,
								    OPLUS_CHG_TRIGGER_MSG_LEN);
				ret += scnprintf(&p_data[ret],
						 OPLUS_CHG_TRIGGER_MSG_LEN - ret, "]");
			} else {
				ret += scnprintf(&p_data[ret],
						 OPLUS_CHG_TRIGGER_MSG_LEN - ret,
                                                 "@@NULL");
			}
		}

		if (chip->trigger_data.flag_reason == TRACK_NOTIFY_FLAG_CHARGER_INFO ||
		    chip->trigger_data.flag_reason == TRACK_NOTIFY_FLAG_USBTEMP_INFO ||
		    chip->trigger_data.type_reason == TRACK_NOTIFY_TYPE_NO_CHARGING ||
		    chip->trigger_data.type_reason == TRACK_NOTIFY_TYPE_CHARGING_BREAK ||
		    chip->trigger_data.type_reason == TRACK_NOTIFY_TYPE_CHARGING_SLOW) {
			index = chip->pps_info.adapter_power_max_index;
			if (chip->pps_info.adapter_pdo != NULL)
				pps_data = (pps_msg_data)chip->pps_info.adapter_pdo[index];
			if (chip->pps_info.adapter_pdo != NULL &&
			    pps_data.pdo_type == USBPD_PDMSG_PDOTYPE_AUGMENTED) {
				ret += scnprintf(&p_data[ret],
					OPLUS_CHG_TRIGGER_MSG_LEN - ret,
					"$$pps_adapter_max@@(%uW %umV %umA)",
					(PPS_PDO_VOL_MAX(pps_data.max_voltage100mv) / MILLIMETER *
					PPS_PDO_CURR_MAX(pps_data.max_current50ma) / MILLIMETER),
					PPS_PDO_VOL_MAX(pps_data.max_voltage100mv),
					PPS_PDO_CURR_MAX(pps_data.max_current50ma));
			} else {
				ret += scnprintf(&p_data[ret],
					OPLUS_CHG_TRIGGER_MSG_LEN - ret,
					"$$pps_adapter_max@@0");
			}
			ret += scnprintf(&p_data[ret],
				OPLUS_CHG_TRIGGER_MSG_LEN - ret,
				"$$cpa_power@@%d", chip->pps_info.cpa_power);
			ret += scnprintf(&p_data[ret],
				OPLUS_CHG_TRIGGER_MSG_LEN - ret,
				"$$adapter_power@@%d", chip->pps_info.adapter_power);
		}
	}

	ret += scnprintf(&p_data[ret], OPLUS_CHG_TRIGGER_MSG_LEN - ret, "%s",
			chip->trigger_data.crux_info);

	len = strlen(&(p_data[sizeof(struct kernel_packet_info)]));
	if (len) {
		mutex_lock(&chip->dcs_info_lock);
		memset(chip->dcs_info, 0x0, sizeof(struct kernel_packet_info));

		chip->dcs_info->type = 1;
		memcpy(chip->dcs_info->log_tag, log_tag, strlen(log_tag));
		memcpy(chip->dcs_info->event_id, event_id, strlen(event_id));
		chip->dcs_info->payload_length = len + 1;
		mutex_unlock(&chip->dcs_info_lock);
		chg_info("%s\n", chip->dcs_info->payload);
		return 0;
	}

	return -1;
}
#endif

static void oplus_chg_track_upload_info_dwork(struct work_struct *work)
{
	int ret = 0;
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip =
		container_of(dwork, struct oplus_chg_track, upload_info_dwork);

	if (!chip)
		return;

	mutex_lock(&chip->dcs_info_lock);
#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) ||	\
	defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE)
	ret = fb_kevent_send_to_user(chip->dcs_info);
#elif defined(CONFIG_OPLUS_KEVENT_UPLOAD)
	ret = kevent_send_to_user(chip->dcs_info);
#endif
	mutex_unlock(&chip->dcs_info_lock);
	if (!ret)
		complete(&chip->trigger_ack);
	else if (chip->dwork_retry_cnt > 0)
		queue_delayed_work(
			chip->trigger_upload_wq, &chip->upload_info_dwork,
			msecs_to_jiffies(OPLUS_CHG_UPDATE_INFO_DELAY_MS));

	chg_info("retry_cnt: %d, ret = %d\n", chip->dwork_retry_cnt, ret);
	chip->dwork_retry_cnt--;
}

static bool oplus_chg_track_has_protocol_changed(struct oplus_chg_track_status *track_status, int type)
{
	if (type != track_status->pre_wired_type &&
	   (type == OPLUS_CHG_USB_TYPE_PD_PPS ||
	    type == OPLUS_CHG_USB_TYPE_VOOC ||
	    type == OPLUS_CHG_USB_TYPE_UFCS ||
	    type == OPLUS_CHG_USB_TYPE_SVOOC)) {
		chg_info("protocol changed, %d->%d\n",  track_status->pre_wired_type, type);
		return true;
	}

	return false;
}

static bool oplus_chg_track_has_power_changed(struct oplus_monitor *monitor, int type)
{
	struct oplus_chg_track *track_chip;
	struct oplus_chg_track_status *track_status;

	if (!monitor)
		return false;
	track_chip = monitor->track;
	if (!track_chip)
		return false;
	track_status = &track_chip->track_status;

	if (type != TRACK_CHG_GET_THTS_TIME_TYPE)
		return false;

	if (strstr(track_status->power_info.wired_info.adapter_type, "ufcs") &&
	    (monitor->ufcs_adapter_power > track_status->power_info.wired_info.power ||
	    monitor->ufcs_emark_power > track_status->power_info.wired_info.emark_power)) {
		chg_info("power changed, Adapter %d->%d, Emark %d->%d\n",
			track_status->power_info.wired_info.power, monitor->ufcs_adapter_power,
			track_status->power_info.wired_info.emark_power, monitor->ufcs_emark_power);
		return true;
	}

	return false;
}

int oplus_chg_track_handle_wired_type_info(
	struct oplus_monitor *monitor, int type)
{
	struct oplus_chg_track *track_chip;
	struct oplus_chg_track_status *track_status;

	if (!monitor)
		return -EINVAL;
	track_chip = monitor->track;
	if (!track_chip)
		return -EINVAL;
	track_status = &track_chip->track_status;

	if (track_status->power_info.wired_info.adapter_id ||
	    strstr(track_status->power_info.wired_info.adapter_type, "vooc") ||
	    strstr(track_status->power_info.wired_info.adapter_type, "ufcs") ||
	    strstr(track_status->power_info.wired_info.adapter_type, "pps") ||
	    (!strcmp(track_status->power_info.wired_info.adapter_type, "pd") &&
	    monitor->wired_charge_type == OPLUS_CHG_USB_TYPE_DCP) ||
	    !strcmp(track_status->power_info.wired_info.adapter_type, "qc")) {
		if (!oplus_chg_track_has_protocol_changed(track_status, monitor->wired_charge_type) &&
		    !oplus_chg_track_has_power_changed(monitor, type)) {
			chg_debug("has know type and not handle\n");
			return 0;
		}
	}

	if (!monitor->wired_charge_type) {
		chg_debug("wired_charge_type is unknow, no update\n");
		return 0;
	}

	track_status->power_info.power_type = TRACK_CHG_TYPE_WIRE;
	memset(track_status->power_info.power_mode, 0,
	       sizeof(track_status->power_info.power_mode));
	strcpy(track_status->power_info.power_mode, "wired");

	if (type == TRACK_CHG_GET_THTS_TIME_TYPE) {
		track_status->pre_wired_type = monitor->wired_charge_type;
		track_status->pre_vbus_vol_type = monitor->vbus_vol_type;
		oplus_chg_track_get_wired_type_info(monitor->wired_charge_type,
			monitor->vbus_vol_type, track_status);
	} else {
		oplus_chg_track_get_wired_type_info(track_status->pre_wired_type,
			track_status->pre_vbus_vol_type, track_status);
	}

	if (type == TRACK_CHG_GET_THTS_TIME_TYPE) {
		track_status->fast_chg_type =
			sid_to_adapter_id(monitor->vooc_sid);
		oplus_chg_track_get_vooc_type_info(track_status->fast_chg_type,
						   track_status);
		oplus_chg_track_get_ufcs_type_info(track_status,
			monitor->ufcs_oplus_adapter, monitor->ufcs_adapter_id,
			monitor->ufcs_adapter_power, monitor->ufcs_emark_power);
	} else {
		oplus_chg_track_get_vooc_type_info(
			track_status->pre_fastchg_type, track_status);
		oplus_chg_track_get_ufcs_type_info(track_status,
			monitor->pre_ufcs_oplus_adapter, monitor->pre_ufcs_adapter_id,
			monitor->pre_ufcs_adapter_power, monitor->pre_ufcs_emark_power);
	}

	if (monitor->pd_svooc && !strcmp(track_status->power_info.wired_info.adapter_type, "svooc"))
		strncpy(track_status->power_info.wired_info.adapter_type,
			"pd_svooc", OPLUS_CHG_TRACK_POWER_TYPE_LEN - 1);

	chg_info("power_mode:%s, type:%s, adapter_id:0x%0x, power:%d, emark_power:%d\n",
		track_status->power_info.power_mode,
		track_status->power_info.wired_info.adapter_type,
		track_status->power_info.wired_info.adapter_id,
		track_status->power_info.wired_info.power,
		track_status->power_info.wired_info.emark_power);

	return 0;
}

static int oplus_chg_track_handle_wls_type_info(
	struct oplus_chg_track_status *track_status, int type)
{
	struct oplus_chg_track *track_chip;
	struct oplus_monitor *monitor;
	union mms_msg_data data = { 0 };
	int adapter_type;
	int dock_type;
	int max_wls_power;

	if (!g_track_chip)
		return -1;

	track_chip = g_track_chip;
	monitor = track_chip->monitor;

	track_status->power_info.power_type = TRACK_CHG_TYPE_WIRELESS;
	memset(track_status->power_info.power_mode, 0, sizeof(track_status->power_info.power_mode));
	strcpy(track_status->power_info.power_mode, "wireless");

	if (!track_chip->monitor || !track_chip->monitor->wls_topic)
		return false;

	if (type == TRACK_CHG_GET_THTS_TIME_TYPE) {
		oplus_mms_get_item_data(track_chip->monitor->wls_topic, WLS_ITEM_TRACK_RECORD_WLS_TYPE, &data, true);
		adapter_type = data.intval;
		oplus_mms_get_item_data(track_chip->monitor->wls_topic, WLS_ITEM_DOCK_TYPE, &data, true);
		dock_type = data.intval;
		oplus_mms_get_item_data(track_chip->monitor->wls_topic, WLS_ITEM_MAX_POWER, &data, true);
		max_wls_power = data.intval;
		if (monitor->wls_online && adapter_type)
			monitor->wls_pre_type = adapter_type;

		if (monitor->wls_online && dock_type)
			track_status->wls_pre_dock_type = dock_type;

		if (monitor->wls_online && max_wls_power)
			track_status->wls_pre_max_power = max_wls_power;
	} else {
		if (!monitor->wls_pre_type) {
			oplus_mms_get_item_data(track_chip->monitor->wls_topic, WLS_ITEM_TRACK_RECORD_WLS_TYPE, &data, true);
			adapter_type = data.intval;
			monitor->wls_pre_type = adapter_type;
		}
		adapter_type = monitor->wls_pre_type;
		dock_type = track_status->wls_pre_dock_type;
		max_wls_power = track_status->wls_pre_max_power;
	}

	oplus_chg_track_get_wls_adapter_type_info(adapter_type, track_status);
	if (dock_type || !strlen(track_status->power_info.wls_info.dock_type))
		oplus_chg_track_get_wls_dock_type_info(dock_type, track_status);

	if (max_wls_power)
		track_status->power_info.wls_info.power = max_wls_power;

	chg_debug("power_mode:%s, type:%s, dock_type:%s, power:%d\n", track_status->power_info.power_mode,
		 track_status->power_info.wls_info.adapter_type, track_status->power_info.wls_info.dock_type,
		 track_status->power_info.wls_info.power);

	return 0;
}

static int oplus_chg_track_handle_batt_full_reason(
	struct oplus_monitor *monitor,
	struct oplus_chg_track_status *track_status)
{
	int notify_flag = NOTIFY_BAT_FULL;

	if (monitor == NULL || track_status == NULL)
		return -1;

	if (monitor->notify_flag == NOTIFY_BAT_FULL ||
	    monitor->notify_flag == NOTIFY_BAT_FULL_PRE_LOW_TEMP ||
	    monitor->notify_flag == NOTIFY_BAT_FULL_PRE_HIGH_TEMP ||
	    monitor->notify_flag == NOTIFY_BAT_NOT_CONNECT ||
	    monitor->notify_flag == NOTIFY_BAT_FULL_THIRD_BATTERY)
		notify_flag = monitor->notify_flag;

	if (track_status->debug_chg_notify_flag)
		notify_flag = track_status->debug_chg_notify_flag;
	if (!strlen(track_status->batt_full_reason))
		oplus_chg_track_get_batt_full_reason_info(notify_flag,
							  track_status);
	chg_info("track_notify_flag:%d, chager_notify_flag:%d, full_reason[%s]\n",
		notify_flag, monitor->notify_flag,
		track_status->batt_full_reason);

	return 0;
}

static int
oplus_chg_track_check_chg_abnormal(struct oplus_monitor *monitor,
				   struct oplus_chg_track_status *track_status)
{
	int notify_code;

	if (monitor == NULL || track_status == NULL)
		return -1;

	if (track_status->debug_chg_notify_code)
		notify_code = track_status->debug_chg_notify_code;
	else
		notify_code = monitor->notify_code;

	if (notify_code & (1 << NOTIFY_CHGING_OVERTIME))
		oplus_chg_track_get_chg_abnormal_reason_info(
			NOTIFY_CHGING_OVERTIME, track_status);

	if (notify_code & (1 << NOTIFY_CHARGER_OVER_VOL)) {
		oplus_chg_track_get_chg_abnormal_reason_info(
			NOTIFY_CHARGER_OVER_VOL, track_status);
	} else if (notify_code & (1 << NOTIFY_CHARGER_LOW_VOL)) {
		oplus_chg_track_get_chg_abnormal_reason_info(
			NOTIFY_CHARGER_LOW_VOL, track_status);
	}

	if (notify_code & (1 << NOTIFY_BAT_OVER_TEMP)) {
		oplus_chg_track_get_chg_abnormal_reason_info(
			NOTIFY_BAT_OVER_TEMP, track_status);
	} else if (notify_code & (1 << NOTIFY_BAT_LOW_TEMP)) {
		oplus_chg_track_get_chg_abnormal_reason_info(
			NOTIFY_BAT_LOW_TEMP, track_status);
	}

	if (notify_code & (1 << NOTIFY_BAT_OVER_VOL)) {
		oplus_chg_track_get_chg_abnormal_reason_info(
			NOTIFY_BAT_OVER_VOL, track_status);
	}

	if (notify_code & (1 << NOTIFY_BAT_NOT_CONNECT)) {
		oplus_chg_track_get_chg_abnormal_reason_info(
			NOTIFY_BAT_NOT_CONNECT, track_status);
	} else if (notify_code & (1 << NOTIFY_BAT_FULL_THIRD_BATTERY)) {
		oplus_chg_track_get_chg_abnormal_reason_info(
			NOTIFY_BAT_FULL_THIRD_BATTERY, track_status);
	}

	chg_debug("track_notify_code:0x%x, chager_notify_code:0x%x, abnormal_reason[%s]\n",
		notify_code, monitor->notify_code,
		track_status->chg_abnormal_reason);

	return 0;
}

static int
oplus_chg_track_cal_chg_common_mesg(struct oplus_monitor *monitor,
				    struct oplus_chg_track_status *track_status)
{
	bool mmi_chg = false;
	static bool pre_slow_chg = false;
	struct rtc_time tm;
	int cep_curr_ma = 0;

	if (monitor == NULL || track_status == NULL)
		return -1;

	if (monitor->shell_temp > track_status->chg_max_temp)
		track_status->chg_max_temp = monitor->shell_temp;

	if (monitor->batt_temp > track_status->batt_max_temp)
		track_status->batt_max_temp = monitor->batt_temp;

	if (monitor->ibat_ma < track_status->batt_max_curr)
		track_status->batt_max_curr = monitor->ibat_ma;

	if (monitor->vbat_mv > track_status->batt_max_vol)
		track_status->batt_max_vol = monitor->vbat_mv;

	if (monitor->wired_vbus_mv > track_status->chg_max_vol)
		track_status->chg_max_vol = monitor->wired_vbus_mv;

	if (track_status->power_info.power_type == TRACK_CHG_TYPE_WIRELESS) {
		if (is_wls_fcc_votable_available(monitor)) {
			if (is_client_vote_enabled(monitor->wls_fcc_votable, CEP_VOTER))
				cep_curr_ma = get_client_vote(monitor->wls_fcc_votable, CEP_VOTER);
			else if (is_client_vote_enabled(monitor->wls_icl_votable, CEP_VOTER))
				cep_curr_ma = get_client_vote(monitor->wls_icl_votable, CEP_VOTER);
		}
		if (cep_curr_ma > 0 && monitor->batt_status == POWER_SUPPLY_STATUS_CHARGING)
			track_status->wls_skew_effect_cnt += 1;
	}

	mmi_chg = oplus_chg_track_get_mmi_chg();
	if (!track_status->once_mmi_chg && !mmi_chg) {
		track_status->once_mmi_chg = true;
		track_status->mmi_chg_open_t =
			oplus_chg_track_get_current_time_s(&track_status->mmi_chg_open_rtc_t);
	}

	if ((track_status->once_mmi_chg == true) && mmi_chg) {
		track_status->mmi_chg_close_t =
			oplus_chg_track_get_current_time_s(&track_status->mmi_chg_close_rtc_t);
		track_status->mmi_chg_constant_t =
			track_status->mmi_chg_close_t - track_status->mmi_chg_open_t;
	}

	if (!track_status->once_chg_cycle_status && monitor->chg_cycle_status)
		track_status->once_chg_cycle_status = monitor->chg_cycle_status;

	if (!pre_slow_chg && monitor->slow_chg_enable) {
		track_status->slow_chg_watt = monitor->slow_chg_watt;
		track_status->slow_chg_pct = monitor->slow_chg_pct;
		if (!track_status->slow_chg_open_t)
			track_status->slow_chg_open_t = oplus_chg_track_get_current_time_s(&tm);
		else
			track_status->slow_chg_open_n_t = oplus_chg_track_get_current_time_s(&tm);
		track_status->slow_chg_open_cnt++;
		track_status->slow_chg_close_t = 0;
	} else if (pre_slow_chg && !monitor->slow_chg_enable) {
		track_status->slow_chg_close_t = oplus_chg_track_get_current_time_s(&tm);
		if (!track_status->slow_chg_open_n_t) {
			track_status->slow_chg_duration +=
				track_status->slow_chg_close_t - track_status->slow_chg_open_t;
		} else {
			track_status->slow_chg_duration +=
				track_status->slow_chg_close_t - track_status->slow_chg_open_n_t;
			track_status->slow_chg_open_n_t = 0;
		}
	}
	pre_slow_chg = monitor->slow_chg_enable;

	chg_debug("chg_max_temp:%d, batt_max_temp:%d, batt_max_curr:%d, "
		"batt_max_vol:%d, once_mmi_chg:%d, once_chg_cycle_status:%d\n",
		track_status->chg_max_temp, track_status->batt_max_temp,
		track_status->batt_max_curr, track_status->batt_max_vol,
		track_status->once_mmi_chg, track_status->once_chg_cycle_status);

	return 0;
}

static int
oplus_chg_track_cal_cool_down_stats(struct oplus_monitor *monitor,
				    struct oplus_chg_track_status *track_status)
{
	int cool_down_max;
	int curr_time;
	bool chg_start;

	if (monitor == NULL || track_status == NULL)
		return -1;

	cool_down_max = ARRAY_SIZE(cool_down_stats_table) - 1;
	if (monitor->cool_down > cool_down_max || monitor->cool_down < 0) {
		chg_err("cool_down is invalid\n");
		return -1;
	}

	if (monitor->batt_status == POWER_SUPPLY_STATUS_CHARGING &&
	    monitor->cool_down)
		track_status->cool_down_effect_cnt++;

	curr_time = oplus_chg_track_get_local_time_s();
	chg_start =
		(track_status->prop_status != POWER_SUPPLY_STATUS_CHARGING &&
		 monitor->batt_status == POWER_SUPPLY_STATUS_CHARGING);
	if (chg_start ||
	    (track_status->prop_status != POWER_SUPPLY_STATUS_CHARGING)) {
		track_status->cool_down_status = monitor->cool_down;
		track_status->cool_down_status_change_t = curr_time;
		return 0;
	}

	if (track_status->cool_down_status != monitor->cool_down) {
		cool_down_stats_table[monitor->cool_down].time +=
			curr_time - track_status->cool_down_status_change_t;
	} else {
		cool_down_stats_table[track_status->cool_down_status].time +=
			curr_time - track_status->cool_down_status_change_t;
	}
	track_status->cool_down_status = monitor->cool_down;
	track_status->cool_down_status_change_t = curr_time;

	chg_debug("cool_down_status:%d, cool_down_tatus_change_t:%d\n",
		 track_status->cool_down_status,
		 track_status->cool_down_status_change_t);

	return 0;
}

static int
oplus_chg_track_cal_rechg_counts(struct oplus_monitor *monitor,
				 struct oplus_chg_track_status *track_status)

{
	if (monitor == NULL || track_status == NULL)
		return -1;

	if (!track_status->in_rechging && monitor->rechging)
		track_status->rechg_counts++;

	track_status->in_rechging = monitor->rechging;
	return 0;
}

static bool oplus_chg_track_is_cool_down_too_small(
	const struct oplus_chg_track_status *track_status)
{
	int thd;

	if (!track_status || !track_status->cool_down_status)
		return false;

	thd = (oplus_gauge_get_batt_num() == 1) ? TRACK_COOL_DOWN_TOO_SMALL_SINGLE_CELL :
						  TRACK_COOL_DOWN_TOO_SMALL;
	return track_status->cool_down_status < thd;
}

static int oplus_chg_track_cal_no_charging_stats_new(
	struct oplus_monitor *monitor, struct oplus_chg_track_status *track_status)
{
	int delta_time;
	int pre_no_charging_cnt;
	bool cool_down_status;

	if (monitor == NULL || track_status == NULL)
		return -1;

	track_status->no_charging_cal_curr_time = oplus_chg_track_get_local_time_s();
	if (track_status->no_charging_cal_pre_time == 0)
		track_status->no_charging_cal_pre_time = track_status->no_charging_cal_curr_time;
	delta_time = track_status->no_charging_cal_curr_time - track_status->no_charging_cal_pre_time;
	track_status->no_charging_cal_pre_time = track_status->no_charging_cal_curr_time;

	cool_down_status = oplus_chg_track_is_cool_down_too_small(track_status);
	if (monitor->batt_status == POWER_SUPPLY_STATUS_CHARGING) {
		track_status->chg_total_time += delta_time;
		if (monitor->ibat_ma > TRACK_NO_CHARGING_IBAT) {
			if (!oplus_chg_track_get_mmi_chg() ||
			    monitor->shell_temp > TRACK_FASTCHG_HIGH_TEMP ||
			    monitor->batt_temp < TRACK_FASTCHG_LOW_TEMP ||
			    cool_down_status ||
			    monitor->plc_status == PLC_STATUS_ENABLE ||
			    monitor->ffc_status != FFC_DEFAULT) {
				chg_info("mmi=%d,T_shell=%d,T_batt=%d,cool_down=%d,plc=%d,ffc=%d\n",
					 !oplus_chg_track_get_mmi_chg(),
					 monitor->shell_temp, monitor->batt_temp,
					 track_status->cool_down_status, monitor->plc_status,
					 (monitor->ffc_status != FFC_DEFAULT));
				if (track_status->chg_no_charging_cnt < TRACK_NO_CHG_CNT_60S)
					track_status->chg_no_charging_cnt = 0;
			} else {
				pre_no_charging_cnt = track_status->chg_no_charging_cnt;
				track_status->chg_no_charging_cnt += delta_time;
				track_status->chg_no_chg_time_s += delta_time;
				chg_info("chg_no_charging_cnt=%ds, no_chg_time=%ds, delta_time=%ds\n",
					 track_status->chg_no_charging_cnt,
					 track_status->chg_no_chg_time_s, delta_time);
				if ((track_status->chg_no_charging_cnt >= TRACK_NO_CHG_CNT_60S) &&
				    (pre_no_charging_cnt < TRACK_NO_CHG_CNT_60S)) {
					chg_info("first upload log\n");
					oplus_chg_track_force_upload_log(monitor->track,
						TRACK_NOTIFY_TYPE_UPLOAD_LOG,
						TRACK_NOTIFY_FLAG_UPLOAD_NO_CHG_LOG);
				}
			}
		} else if (track_status->chg_no_charging_cnt < TRACK_NO_CHG_CNT_60S) {
			track_status->chg_no_charging_cnt = 0;
			chg_debug("chg_no_charging_cnt reset\n");
		}
	}

	return 0;
}

static int oplus_chg_track_cal_no_charging_stats(
	struct oplus_monitor *monitor,
	struct oplus_chg_track_status *track_status)
{
	static int otg_online_cnt = 0;
	static int vbatt_leak_cnt = 0;
	bool no_charging_state = false;
	struct oplus_chg_track *track_chip;

	if (monitor == NULL || track_status == NULL)
		return -1;

	if (!g_track_chip)
		return -1;

	if (g_track_chip->track_cfg.track_ver >= TRACK_VER_4_0)
		return oplus_chg_track_cal_no_charging_stats_new(monitor, track_status);

	track_chip = g_track_chip;
	if (monitor->batt_status == POWER_SUPPLY_STATUS_CHARGING) {
		track_status->chg_total_cnt++;
		if (monitor->ibat_ma > 0) {
			track_status->chg_no_charging_cnt++;
			no_charging_state = true;
		}
		if (no_charging_state == true) {
			if (monitor->otg_enable)
				otg_online_cnt++;
			else if (monitor->vbat_mv >= monitor->wired_vbus_mv)
				vbatt_leak_cnt++;
		} else {
			otg_online_cnt = 0;
			vbatt_leak_cnt = 0;
		}
	} else {
		otg_online_cnt = 0;
		vbatt_leak_cnt = 0;
	}
	if (otg_online_cnt > 10)
		track_chip->no_charging_trigger.flag_reason =
					TRACK_NOTIFY_FLAG_NO_CHARGING_OTG_ONLINE;
	else if (vbatt_leak_cnt > 10)
		track_chip->no_charging_trigger.flag_reason =
					TRACK_NOTIFY_FLAG_NO_CHARGING_VBATT_LEAK;
	if (track_status->debug_no_charging)
		track_chip->no_charging_trigger.flag_reason =
					track_status->debug_no_charging + TRACK_NOTIFY_FLAG_NO_CHARGING_FIRST - 1;


	return 0;
}

static int oplus_chg_track_cal_ledon_ledoff_average_speed(
	struct oplus_chg_track_status *track_status)
{
	if (track_status->led_onoff_power_cal)
		return 0;

	if (!track_status->ledon_ave_speed) {
		if (!track_status->ledon_time)
			track_status->ledon_ave_speed = 0;
		else
			track_status->ledon_ave_speed =
				TRACK_TIME_1MIN_THD * track_status->ledon_rm /
				track_status->ledon_time;
	}

	if (!track_status->ledoff_ave_speed) {
		if (!track_status->ledoff_time)
			track_status->ledoff_ave_speed = 0;
		else
			track_status->ledoff_ave_speed =
				TRACK_TIME_1MIN_THD * track_status->ledoff_rm /
				track_status->ledoff_time;
	}

	track_status->led_onoff_power_cal = true;
	chg_info("ledon_ave_speed:%d, ledoff_ave_speed:%d\n",
		 track_status->ledon_ave_speed, track_status->ledoff_ave_speed);
	return 0;
}

static int
oplus_chg_track_cal_app_stats(struct oplus_monitor *monitor,
			      struct oplus_chg_track_status *track_status)
{
	int curr_time;
	bool chg_start;
	bool chg_end;
	bool charger_exist;

	if (monitor == NULL || track_status == NULL)
		return -EINVAL;

	if (track_status->app_status.app_cal)
		return 0;

	charger_exist = monitor->wired_online || monitor->wls_online;
	curr_time = oplus_chg_track_get_local_time_s();
	chg_start =
		(track_status->prop_status != POWER_SUPPLY_STATUS_CHARGING &&
		 monitor->batt_status == POWER_SUPPLY_STATUS_CHARGING);
	chg_end = (track_status->prop_status == POWER_SUPPLY_STATUS_CHARGING &&
		   monitor->batt_status != POWER_SUPPLY_STATUS_CHARGING);
	if (chg_start || (!track_status->led_on && monitor->led_on) ||
	    (track_status->prop_status != POWER_SUPPLY_STATUS_CHARGING)) {
		strncpy(track_status->app_status.pre_top_name,
			track_status->app_status.curr_top_name,
			TRACK_APP_REAL_NAME_LEN - 1);
		track_status->app_status.pre_top_name[TRACK_APP_REAL_NAME_LEN - 1] = '\0';
		track_status->app_status.change_t = curr_time;
		track_status->app_status.curr_top_index =
			oplus_chg_track_match_app_info(
			track_status->app_status.pre_top_name);
		return 0;
	}

	if (strcmp(track_status->app_status.pre_top_name,
	    track_status->app_status.curr_top_name) || chg_end ||
	    (track_status->led_on && !monitor->led_on)) {
		chg_debug("!!!app change or chg_end or led change, need update\n");
		if (monitor->led_on ||  (track_status->led_on && !monitor->led_on)) {
			if (track_status->app_status.curr_top_index <
			   (ARRAY_SIZE(app_table) - 1))
				app_table[track_status->app_status.curr_top_index].cont_time +=
				curr_time - track_status->app_status.change_t;
			else
				app_table[ARRAY_SIZE(app_table) - 1].cont_time +=
				curr_time - track_status->app_status.change_t;
		}
		track_status->app_status.change_t = curr_time;
		strncpy(track_status->app_status.pre_top_name,
			track_status->app_status.curr_top_name,
			TRACK_APP_REAL_NAME_LEN - 1);
		track_status->app_status.pre_top_name[TRACK_APP_REAL_NAME_LEN - 1] = '\0';
		track_status->app_status.curr_top_index =
			oplus_chg_track_match_app_info(
			track_status->app_status.pre_top_name);
	}

	if (!charger_exist || track_status->chg_report_full_time) {
		track_status->app_status.app_cal = true;
	}

	chg_debug("ch_t:%d, app_cal:%d, curr_top_index:%d, curr_top_name:%s\n",
		track_status->app_status.change_t, track_status->app_status.app_cal,
		track_status->app_status.curr_top_index,
		track_status->app_status.curr_top_name);
	return 0;
}

static int
oplus_chg_track_cal_led_on_stats(struct oplus_monitor *monitor,
				 struct oplus_chg_track_status *track_status)
{
	int curr_time;
	bool chg_start;
	bool chg_end;
	bool need_update = false;
	bool charger_exist;

	if (monitor == NULL || track_status == NULL)
		return -1;

	charger_exist = monitor->wired_online || monitor->wls_online;

	curr_time = oplus_chg_track_get_local_time_s();
	chg_start =
		(track_status->prop_status != POWER_SUPPLY_STATUS_CHARGING &&
		 monitor->batt_status == POWER_SUPPLY_STATUS_CHARGING);
	chg_end = (track_status->prop_status == POWER_SUPPLY_STATUS_CHARGING &&
		   monitor->batt_status != POWER_SUPPLY_STATUS_CHARGING);
	if (chg_start ||
	    (track_status->prop_status != POWER_SUPPLY_STATUS_CHARGING)) {
		track_status->led_on = monitor->led_on;
		track_status->led_change_t = curr_time;
		track_status->led_change_rm = monitor->batt_rm;
		return 0;
	}

	if (chg_end || track_status->chg_fast_full_time ||
	    monitor->batt_soc >= TRACK_LED_MONITOR_SOC_POINT)
		need_update = true;

	if (track_status->led_on && (!monitor->led_on || need_update)) {
		if (curr_time - track_status->led_change_t >
		    TRACK_CYCLE_RECORDIING_TIME_2MIN) {
			track_status->ledon_rm +=
				monitor->batt_rm - track_status->led_change_rm;
			track_status->ledon_time +=
				curr_time - track_status->led_change_t;
		}
		track_status->continue_ledon_time +=
			curr_time - track_status->led_change_t;
		track_status->led_change_t = curr_time;
		track_status->led_change_rm = monitor->batt_rm;
	} else if (!track_status->led_on && (monitor->led_on || need_update)) {
		if (curr_time - track_status->led_change_t >
		    TRACK_CYCLE_RECORDIING_TIME_2MIN) {
			track_status->ledoff_rm +=
				monitor->batt_rm - track_status->led_change_rm;
			track_status->ledoff_time +=
				curr_time - track_status->led_change_t;
		}
		track_status->continue_ledoff_time +=
			curr_time - track_status->led_change_t;
		track_status->led_change_t = curr_time;
		track_status->led_change_rm = monitor->batt_rm;
	}
	track_status->led_on = monitor->led_on;
	chg_debug("continue_ledoff_t:%d, continue_ledon_t:%d\n",
		 track_status->continue_ledoff_time,
		 track_status->continue_ledon_time);

	if (!charger_exist || track_status->chg_report_full_time ||
	    track_status->chg_fast_full_time ||
	    monitor->batt_soc >= TRACK_LED_MONITOR_SOC_POINT)
		oplus_chg_track_cal_ledon_ledoff_average_speed(track_status);

	chg_debug("ch_t:%d, ch_rm:%d, ledoff_rm:%d, ledoff_t:%d, ledon_rm:%d, ledon_t:%d\n",
		 track_status->led_change_t, track_status->led_change_rm,
		 track_status->ledoff_rm, track_status->ledoff_time,
		 track_status->ledon_rm, track_status->ledon_time);

	return 0;
}

static int oplus_chg_wls_track_status_reset(struct oplus_chg_track_status *track_status)
{
	track_status->wls_pmw_pulse = 0;
	track_status->wls_audio_t = 0;
	track_status->wls_audio_t_total = 0;
	track_status->wls_camera_t = 0;
	track_status->wls_camera_t_total = 0;
	track_status->wls_quiet_mdoe = false;
	track_status->wls_quiet_mdoe_t = 0;
	track_status->wls_quiet_mdoe_t_total = 0;
	track_status->wls_chg_t_total = 0;
	track_status->wls_chg_t = 0;
	track_status->wls_manu_break = false;
	return 0;
}

static int oplus_chg_track_wls_cal_quiet_mode_stats(struct oplus_monitor *monitor,
	  struct oplus_chg_track_status *track_status)
{
	int curr_time;
	bool chg_end = false;
	static int wls_pmw_pulse;
	static bool wls_quiet_mdoe;
	static bool pre_charger_exist;
	union mms_msg_data data = { 0 };

	if (monitor == NULL || track_status == NULL)
		return -1;

	if (monitor->wls_online) {
		pre_charger_exist = true;
	} else if (pre_charger_exist && !monitor->wls_online) {
		chg_end = true;
		pre_charger_exist = false;
	}else {
		pre_charger_exist = false;
		wls_pmw_pulse = 0;
		wls_quiet_mdoe = false;
		chg_end = false;
		return 0;
	}

	oplus_mms_get_item_data(g_track_chip->monitor->wls_topic, WLS_ITEM_AUDIO_CAM_PMW, &data, true);
	wls_pmw_pulse = data.intval;
	oplus_mms_get_item_data(g_track_chip->monitor->wls_topic, WLS_ITEM_QUIET_MODE, &data, true);
	wls_quiet_mdoe = data.intval;

	curr_time = oplus_chg_track_get_local_time_s();
	if (pre_charger_exist && !track_status->wls_chg_t)
		track_status->wls_chg_t = curr_time;
	else if (track_status->wls_chg_t)
		track_status->wls_chg_t_total = curr_time - track_status->wls_chg_t;

	if (track_status->wls_pmw_pulse != WLS_AUDIO_MODE && wls_pmw_pulse == WLS_AUDIO_MODE)
		track_status->wls_audio_t = curr_time;
	else if ((track_status->wls_pmw_pulse == WLS_AUDIO_MODE && (wls_pmw_pulse != WLS_AUDIO_MODE || chg_end)))
		track_status->wls_audio_t_total += curr_time - track_status->wls_audio_t;

	if (track_status->wls_pmw_pulse != WLS_CAMERA_MODE && wls_pmw_pulse == WLS_CAMERA_MODE)
		track_status->wls_camera_t = curr_time;
	else if ((track_status->wls_pmw_pulse == WLS_CAMERA_MODE && (wls_pmw_pulse != WLS_CAMERA_MODE || chg_end)))
		track_status->wls_camera_t_total += curr_time - track_status->wls_camera_t;

	if (track_status->wls_quiet_mdoe != true && wls_quiet_mdoe == true)
		track_status->wls_quiet_mdoe_t = curr_time;
	else if ((track_status->wls_quiet_mdoe == true && (wls_quiet_mdoe != true || chg_end)))
		track_status->wls_quiet_mdoe_t_total += curr_time - track_status->wls_quiet_mdoe_t;

	chg_debug("track_wls_pmw_pulse:%d, wls_pmw_pulse:%d, wls_audio_t:%llu,"
		 "wls_audio_t_total:%llu, wls_camera_t:%llu, wls_camera_t_total:%llu,"
		 "track_wls_quiet_mdoe:%d, wls_quiet_mdoe:%d, wls_quiet_mdoe_t_total:%llu,"
		 "wls_chg_t:%llu, wls_chg_t_total:%llu, end:%d\n",
		 track_status->wls_pmw_pulse, wls_pmw_pulse,
		 track_status->wls_audio_t, track_status->wls_audio_t_total,
		 track_status->wls_camera_t, track_status->wls_camera_t_total,
		 track_status->wls_quiet_mdoe, wls_quiet_mdoe, track_status->wls_quiet_mdoe_t_total,
		 track_status->wls_chg_t, track_status->wls_chg_t_total, chg_end);

	if (chg_end) {
		track_status->wls_pmw_pulse = 0;
		track_status->wls_quiet_mdoe = 0;
	} else {
		track_status->wls_pmw_pulse = wls_pmw_pulse;
		track_status->wls_quiet_mdoe = wls_quiet_mdoe;
	}

	return 0;
}

static bool oplus_chg_track_is_no_charging_new(struct oplus_chg_track_status *track_status)
{
	bool ret = false;
	char wired_adapter_type[OPLUS_CHG_TRACK_POWER_TYPE_LEN];
	char wls_adapter_type[OPLUS_CHG_TRACK_POWER_TYPE_LEN];
	struct oplus_chg_track *track_chip;
	int curr_time;

	if (track_status == NULL)
		return ret;

	if (!g_track_chip)
		return ret;

	track_chip = g_track_chip;
	if ((track_status->chg_total_time <= TRACK_TIME_2MIN_THD) || (track_status->curr_uisoc == 100))
		return ret;

	if (track_status->chg_no_charging_cnt >= TRACK_NO_CHG_CNT_60S) {
		track_chip->no_charging_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_NO_CHARGING_IBAT_LOW;
		ret = true;
	}

	memcpy(wired_adapter_type,
	       track_status->power_info.wired_info.adapter_type,
	       OPLUS_CHG_TRACK_POWER_TYPE_LEN);
	memcpy(wls_adapter_type, track_status->power_info.wls_info.adapter_type,
	       OPLUS_CHG_TRACK_POWER_TYPE_LEN);
	if (!strcmp(wired_adapter_type, "unknow") ||
	    !strcmp(wired_adapter_type, "sdp") ||
	    !strcmp(wired_adapter_type, "cdp") ||
	    !strcmp(wls_adapter_type, "unknow"))
		ret = false;

	if (track_status->chg_start_soc >= TRACK_REF_SOC_85)
		ret = false;

	curr_time = oplus_chg_track_get_local_time_s();
	chg_debug("cool_down_status=%d,cool_down = %d/%d\n", track_status->cool_down_status,
		(cool_down_stats_table[1].time + cool_down_stats_table[2].time),
		(curr_time - track_status->chg_start_time));
	if ((cool_down_stats_table[1].time + cool_down_stats_table[2].time) >
		(curr_time - track_status->chg_start_time) / 2) {
		chg_info("high cool_down level\n");
		ret = false;
	}

	if (track_status->debug_no_charging) {
		track_chip->no_charging_trigger.flag_reason =
			track_status->debug_no_charging + TRACK_NOTIFY_FLAG_NO_CHARGING_FIRST - 1;
		ret = true;
	}

	chg_info("chg_no_charging_cnt:%d, chg_total_time:%d, debug_no_charging:%d, chg_start_soc:%d,"
		"wired_adapter_type:%s, wls_adapter_type:%s, ret:%d",
		track_status->chg_no_charging_cnt, track_status->chg_total_time,
		track_status->debug_no_charging, track_status->chg_start_soc,
		 wired_adapter_type, wls_adapter_type, ret);
	return ret;
}

static bool
oplus_chg_track_is_no_charging(struct oplus_chg_track_status *track_status)
{
	bool ret = false;
	char wired_adapter_type[OPLUS_CHG_TRACK_POWER_TYPE_LEN];
	char wls_adapter_type[OPLUS_CHG_TRACK_POWER_TYPE_LEN];

	if (track_status == NULL)
		return ret;

	if (g_track_chip->track_cfg.track_ver >= TRACK_VER_4_0)
		return oplus_chg_track_is_no_charging_new(track_status);

	if ((track_status->chg_total_cnt <= 24) || (track_status->curr_uisoc == 100)) /* 5s*24=120s */
		return ret;

	if ((track_status->chg_no_charging_cnt * 100) /
		    track_status->chg_total_cnt >
	    TRACK_NO_CHRGING_TIME_PCT)
		ret = true;

	memcpy(wired_adapter_type,
	       track_status->power_info.wired_info.adapter_type,
	       OPLUS_CHG_TRACK_POWER_TYPE_LEN);
	memcpy(wls_adapter_type, track_status->power_info.wls_info.adapter_type,
	       OPLUS_CHG_TRACK_POWER_TYPE_LEN);
	chg_info("wired_adapter_type:%s, wls_adapter_type:%s\n",
		 wired_adapter_type, wls_adapter_type);
	if (!strcmp(wired_adapter_type, "unknow") ||
	    !strcmp(wired_adapter_type, "sdp") ||
	    !strcmp(wls_adapter_type, "unknow"))
		ret = false;

	if (track_status->debug_no_charging)
		ret = true;

	chg_info("chg_no_charging_cnt:%d, chg_total_cnt:%d, debug_no_charging:%d, ret:%d",
		track_status->chg_no_charging_cnt, track_status->chg_total_cnt,
		track_status->debug_no_charging, ret);

	return ret;
}

static void oplus_chg_track_record_break_charging_info(
	struct oplus_chg_track *track_chip, struct oplus_monitor *monitor,
	struct oplus_chg_track_power power_info, const char *sub_crux_info)
{
	int index = 0;
	struct oplus_chg_track_status *track_status;

	if (track_chip == NULL)
		return;

	chg_info("start, type=%d\n", power_info.power_type);
	track_status = &(track_chip->track_status);

	if (power_info.power_type == TRACK_CHG_TYPE_WIRE) {
		memset(track_chip->charging_break_trigger.crux_info, 0,
		       sizeof(track_chip->charging_break_trigger.crux_info));
		index += scnprintf(
			&(track_chip->charging_break_trigger.crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			"$$power_mode@@%s", power_info.power_mode);
		index += scnprintf(
			&(track_chip->charging_break_trigger.crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			"$$adapter_t@@%s", power_info.wired_info.adapter_type);
		if (power_info.wired_info.adapter_id)
			index += scnprintf(&(track_chip->charging_break_trigger
						    .crux_info[index]),
					  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
					  "$$adapter_id@@0x%x",
					  power_info.wired_info.adapter_id);
		index += scnprintf(
			&(track_chip->charging_break_trigger.crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$power@@%d",
			power_info.wired_info.power);
		if (power_info.wired_info.emark_power > 0)
			index += scnprintf(&(track_chip->charging_break_trigger.crux_info[index]),
				OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$emark_power@@%d",
				power_info.wired_info.emark_power);

		if (track_status->wired_max_power <= 0)
			index += scnprintf(&(track_chip->charging_break_trigger.crux_info[index]),
				OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$match_power@@%d", -1);
		else
			index += scnprintf(&(track_chip->charging_break_trigger.crux_info[index]),
				OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$match_power@@%d",
				(track_status->power_info.wired_info.power >= track_status->wired_max_power));

		if (strlen(track_status->fastchg_break_info.name)) {
			index += scnprintf(&(track_chip->charging_break_trigger
						    .crux_info[index]),
					  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
					  "$$voocphy_name@@%s",
					  track_chip->voocphy_name);
			index +=
				scnprintf(&(track_chip->charging_break_trigger
						   .crux_info[index]),
					 OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
					 "$$reason@@%s",
					 track_status->fastchg_break_info.name);
		}
		if (strlen(sub_crux_info)) {
			index += scnprintf(&(track_chip->charging_break_trigger
						    .crux_info[index]),
					  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
					  "$$crux_info@@%s", sub_crux_info);
		}
		oplus_chg_track_record_general_info(
			monitor, track_status,
			&(track_chip->charging_break_trigger), index);
		chg_info("wired[%s]\n",
			 track_chip->charging_break_trigger.crux_info);
	} else if (power_info.power_type == TRACK_CHG_TYPE_WIRELESS) {
		memset(track_chip->wls_charging_break_trigger.crux_info, 0,
		       sizeof(track_chip->wls_charging_break_trigger.crux_info));
		index += scnprintf(&(track_chip->wls_charging_break_trigger
					    .crux_info[index]),
				  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$power_mode@@%s", power_info.power_mode);
		index += scnprintf(&(track_chip->wls_charging_break_trigger
					    .crux_info[index]),
				  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$adapter_t@@%s",
				  power_info.wls_info.adapter_type);
		if (strlen(power_info.wls_info.dock_type))
			index += scnprintf(
				&(track_chip->wls_charging_break_trigger
					  .crux_info[index]),
				OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				"$$dock_type@@%s",
				power_info.wls_info.dock_type);
		index += scnprintf(&(track_chip->wls_charging_break_trigger
					    .crux_info[index]),
				  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$power@@%d", power_info.wls_info.power);

		if (track_status->wls_max_power <= 0)
			index += scnprintf(&(track_chip->wls_charging_break_trigger.crux_info[index]),
					  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
					  "$$match_power@@%d", -1);
		else
			index += scnprintf(&(track_chip->wls_charging_break_trigger.crux_info[index]),
					  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
					  "$$match_power@@%d",
					  (track_status->power_info.wls_info.power >= track_status->wls_max_power));
		index += scnprintf(&(track_chip->wls_charging_break_trigger.crux_info[index]),
				  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$delta_time_ms@@%llu",
				  track_status->wls_attach_time_ms - track_status->wls_detach_time_ms);
		if (strlen(sub_crux_info)) {
			index += scnprintf(
				&(track_chip->wls_charging_break_trigger
					  .crux_info[index]),
				OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				"$$crux_info@@%s", sub_crux_info);
		}
		oplus_chg_track_record_general_info(
			monitor, track_status,
			&(track_chip->wls_charging_break_trigger), index);
		chg_info("wls[%s]\n",
			 track_chip->wls_charging_break_trigger.crux_info);
	}
}

static int oplus_chg_track_get_fv_when_vooc(struct oplus_monitor *monitor)
{
	int flv;

	if (monitor->chg_ctrl_by_vooc && (monitor->ffc_status == FFC_DEFAULT)) {
		if (is_fv_votable_available(monitor))
			flv = get_effective_result(monitor->fv_votable);
		else
			flv = monitor->fv_mv;
	} else {
		flv = 0;
	}
	chg_info("flv=%d\n", flv);

	return flv;
}

int oplus_chg_track_set_fastchg_break_code(int fastchg_break_code)
{
	int flv;
	struct oplus_chg_track *track_chip;
	struct oplus_chg_track_status *track_status;
	struct oplus_monitor *monitor;
	unsigned vooc_sid;

	if (!g_track_chip)
		return -1;

	track_chip = g_track_chip;
	monitor = track_chip->monitor;
	track_status = &track_chip->track_status;
	if (track_chip->track_cfg.track_ver >= TRACK_VER_4_0 && track_status->debug_break_code)
		track_status->fastchg_break_info.code = track_status->debug_break_code;
	else
		track_status->fastchg_break_info.code = fastchg_break_code;
	if (track_chip->monitor->vooc_sid != 0)
		vooc_sid = track_chip->monitor->vooc_sid;
	else
		vooc_sid = track_chip->monitor->pre_vooc_sid;
	track_status->pre_fastchg_type = sid_to_adapter_chg_type(vooc_sid);
	flv = oplus_chg_track_get_fv_when_vooc(monitor);
	if ((track_status->pre_fastchg_type == CHARGER_TYPE_VOOC ||
	    track_status->pre_fastchg_type == CHARGER_TYPE_SVOOC) &&
	    flv && (flv - TRACK_CHG_VOOC_BATT_VOL_DIFF_MV < monitor->vbat_mv)) {
		if (g_track_chip->track_cfg.voocphy_type ==
		    TRACK_ADSP_VOOCPHY) {
			if (fastchg_break_code ==
			    TRACK_ADSP_VOOCPHY_COMMU_TIME_OUT)
				track_status->fastchg_break_info.code =
					TRACK_ADSP_VOOCPHY_BREAK_DEFAULT;
		} else if (g_track_chip->track_cfg.voocphy_type ==
				   TRACK_AP_SINGLE_CP_VOOCPHY ||
			   g_track_chip->track_cfg.voocphy_type ==
				   TRACK_AP_DUAL_CP_VOOCPHY) {
			if (fastchg_break_code ==
			    TRACK_CP_VOOCPHY_COMMU_TIME_OUT)
				track_status->fastchg_break_info.code =
					TRACK_CP_VOOCPHY_BREAK_DEFAULT;
		} else if (g_track_chip->track_cfg.voocphy_type == TRACK_MCU_VOOCPHY) {
			if (fastchg_break_code == TRACK_MCU_VOOCPHY_FAST_ABSENT)
				track_status->fastchg_break_info.code =
					TRACK_CP_VOOCPHY_BREAK_DEFAULT;
		}
	}

	return 0;
}

static int oplus_chg_track_match_mcu_voocphy_break_reason(
	struct oplus_chg_track_status *track_status)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mcu_voocphy_break_table); i++) {
		if (mcu_voocphy_break_table[i].code ==
		    track_status->fastchg_break_info.code) {
			strncpy(track_status->fastchg_break_info.name,
				mcu_voocphy_break_table[i].name,
				OPLUS_CHG_TRACK_FASTCHG_BREAK_REASON_LEN - 1);
			break;
		}
	}

	return 0;
}

static int oplus_chg_track_match_adsp_voocphy_break_reason(
	struct oplus_chg_track_status *track_status)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(adsp_voocphy_break_table); i++) {
		if (adsp_voocphy_break_table[i].code ==
		    track_status->fastchg_break_info.code) {
			strncpy(track_status->fastchg_break_info.name,
				adsp_voocphy_break_table[i].name,
				OPLUS_CHG_TRACK_FASTCHG_BREAK_REASON_LEN - 1);
			break;
		}
	}

	return 0;
}

static int oplus_chg_track_match_ap_voocphy_break_reason(
	struct oplus_chg_track_status *track_status)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ap_voocphy_break_table); i++) {
		if (ap_voocphy_break_table[i].code ==
		    track_status->fastchg_break_info.code) {
			strncpy(track_status->fastchg_break_info.name,
				ap_voocphy_break_table[i].name,
				OPLUS_CHG_TRACK_FASTCHG_BREAK_REASON_LEN - 1);
			break;
		}
	}

	return 0;
}

static int
oplus_chg_track_match_fastchg_break_reason(struct oplus_chg_track *track_chip)
{
	struct oplus_chg_track_status *track_status;

	if (!track_chip)
		return -1;
	track_status = &track_chip->track_status;

	chg_info("voocphy_type:%d, code:0x%x\n",
		 track_chip->track_cfg.voocphy_type,
		 track_status->fastchg_break_info.code);
	switch (track_chip->track_cfg.voocphy_type) {
	case TRACK_ADSP_VOOCPHY:
		oplus_chg_track_match_adsp_voocphy_break_reason(track_status);
		break;
	case TRACK_AP_SINGLE_CP_VOOCPHY:
	case TRACK_AP_DUAL_CP_VOOCPHY:
		oplus_chg_track_match_ap_voocphy_break_reason(track_status);
		break;
	case TRACK_MCU_VOOCPHY:
		oplus_chg_track_match_mcu_voocphy_break_reason(track_status);
		break;
	default:
		chg_info("!!!voocphy type is error, should not go here\n");
		break;
	}

	return 0;
}

static int oplus_chg_track_obtain_wls_break_sub_crux_info(
	struct oplus_chg_track *track_chip, char *crux_info)
{
	if (!track_chip || !track_chip->monitor || !track_chip->monitor->wls_topic || !crux_info)
		return -EINVAL;

	oplus_chg_wls_get_break_sub_crux_info(track_chip->monitor->wls_topic, crux_info);

	return 0;
}

static bool oplus_chg_track_wls_is_status_keep(struct oplus_chg_track *track_chip)
{
	union mms_msg_data data = { 0 };
	int rc;

	if (!track_chip || !track_chip->monitor || !track_chip->monitor->wls_topic)
		return false;
	rc = oplus_mms_get_item_data(track_chip->monitor->wls_topic, WLS_ITEM_STATUS_KEEP, &data, true);
	if (rc < 0) {
		chg_err("can't get status_keep, rc=%d\n", rc);
		return false;
	}
	if (data.intval == WLS_SK_BY_KERNEL || data.intval == WLS_SK_BY_HAL || data.intval == WLS_SK_WAIT_TIMEOUT)
		return true;

	return false;
}

static bool oplus_chg_track_wls_is_normal_chg(enum oplus_chg_wls_type wls_type)
{
	bool is_normal_chg;

	switch (wls_type) {
	case OPLUS_CHG_WLS_BPP:
	case OPLUS_CHG_WLS_EPP:
	case OPLUS_CHG_WLS_EPP_PLUS:
		is_normal_chg = true;
		break;
	default:
		is_normal_chg = false;
		break;
	}
	return is_normal_chg;
}

int oplus_chg_track_check_wls_charging_break(int wls_connect)
{
	struct oplus_chg_track *track_chip;
	struct oplus_chg_track_status *track_status;
	struct oplus_monitor *monitor;
	static struct oplus_chg_track_power power_info;
	static bool break_recording = 0;
	static bool pre_wls_connect = false;
	unsigned long long delta_time_ms;
	bool is_normal_chg;

	if (!g_track_chip || g_track_chip->track_cfg.track_ver >= TRACK_VER_4_0)
		return -1;

	track_chip = g_track_chip;
	track_status = &track_chip->track_status;
	monitor = track_chip->monitor;

	if (pre_wls_connect == wls_connect)
		return 0;

	is_normal_chg = oplus_chg_track_wls_is_normal_chg(monitor->wls_pre_type);
	if (wls_connect) {
		track_status->wls_status_keep = oplus_chg_track_wls_is_status_keep(track_chip);
		track_status->wls_attach_time_ms = local_clock() / TRACK_LOCAL_T_NS_TO_MS_THD;
		delta_time_ms = track_status->wls_attach_time_ms - track_status->wls_detach_time_ms;
		if ((delta_time_ms < track_chip->track_cfg.wls_chg_break_t_thd && track_status->wls_status_keep && !is_normal_chg) ||
		    (delta_time_ms < track_chip->track_cfg.wls_normal_chg_break_t_thd && track_status->wls_status_keep && is_normal_chg)) {
			if (!break_recording) {
				break_recording = true;
				oplus_chg_track_get_wls_adapter_type_info(monitor->wls_pre_type, track_status);
				track_chip->wls_charging_break_trigger.flag_reason =
					TRACK_NOTIFY_FLAG_WLS_CHARGING_BREAK;
				oplus_chg_track_record_break_charging_info(track_chip, monitor,
					power_info, track_status->wls_break_crux_info);
				schedule_delayed_work(&track_chip->wls_charging_break_trigger_work, 0);
			}
			if (!track_status->wired_need_upload) {
				cancel_delayed_work_sync(&track_chip->charger_info_trigger_work);
				cancel_delayed_work_sync(&track_chip->no_charging_trigger_work);
				cancel_delayed_work_sync(&track_chip->slow_charging_trigger_work);
			}
		} else {
			/*record one time in 6s*/
			break_recording = 0;
		}
		monitor->wls_pre_type = 0;
		chg_info("pre_wls_connect[%d], wls_connect[%d], break_recording[%d], status_keep[%d], "
			"detal_t:%llu, wls_attach_time:%llu\n",
			pre_wls_connect, wls_connect, break_recording, track_status->wls_status_keep,
			delta_time_ms, track_status->wls_attach_time_ms);
	} else {
		track_status->wls_detach_time_ms = local_clock() / TRACK_LOCAL_T_NS_TO_MS_THD;
		oplus_chg_track_handle_wls_type_info(track_status, TRACK_CHG_GET_LAST_TIME_TYPE);
		oplus_chg_track_obtain_wls_break_sub_crux_info(track_chip, track_status->wls_break_crux_info);
		power_info = track_status->power_info;
		/* oplus_chg_wake_update_work(); TODO:need check */
		chg_info("wls_detach_time = %llu\n", track_status->wls_detach_time_ms);
	}
	pre_wls_connect = wls_connect;

	return 0;
}

static bool
oplus_chg_track_wired_fastchg_exit_code(struct oplus_chg_track *track_chip)
{
	bool ret = true;
	int code;
	struct oplus_chg_track_status *track_status;

	if (!track_chip)
		return -1;
	track_status = &track_chip->track_status;
	code = track_status->fastchg_break_info.code;

	chg_info("voocphy_type:%d, code:0x%x\n",
		 track_chip->track_cfg.voocphy_type,
		 track_status->fastchg_break_info.code);
	switch (track_chip->track_cfg.voocphy_type) {
	case TRACK_ADSP_VOOCPHY:
		if (!code || code == TRACK_ADSP_VOOCPHY_FULL ||
		    code == TRACK_ADSP_VOOCPHY_BATT_TEMP_OVER ||
		    code == TRACK_ADSP_VOOCPHY_SWITCH_TEMP_RANGE)
			ret = true;
		else
			ret = false;
		break;
	case TRACK_AP_SINGLE_CP_VOOCPHY:
	case TRACK_AP_DUAL_CP_VOOCPHY:
		if (!code || code == TRACK_CP_VOOCPHY_FULL ||
		    code == TRACK_CP_VOOCPHY_BATT_TEMP_OVER ||
		    code == TRACK_CP_VOOCPHY_USER_EXIT_FASTCHG ||
		    code == TRACK_CP_VOOCPHY_SWITCH_TEMP_RANGE ||
		    code == TRACK_CP_VOOCPHY_CURR_LIMIT_SMALL ||
		    code == TRACK_CP_VOOCPHY_IC_BURN)
			ret = true;
		else
			ret = false;
		break;
	case TRACK_MCU_VOOCPHY:
		if (!code || code == TRACK_MCU_VOOCPHY_TEMP_OVER ||
		    code == TRACK_MCU_VOOCPHY_NORMAL_TEMP_FULL ||
		    code == TRACK_MCU_VOOCPHY_LOW_TEMP_FULL ||
		    code == TRACK_MCU_VOOCPHY_BAT_TEMP_EXIT)
			ret = true;
		else
			ret = false;
		break;
	default:
		chg_info("!!!voocphy type is error, should not go here\n");
		break;
	}

	return ret;
}

static bool oplus_chg_track_get_mmi_chg(void)
{
	struct votable *chg_disable_votable;
	struct votable *chg_suspend_votable;

	chg_disable_votable = find_votable("CHG_DISABLE");
	chg_suspend_votable = find_votable("CHG_SUSPEND");
	if (!chg_disable_votable || !chg_suspend_votable)
		return true;

	return ((!get_client_vote(chg_disable_votable, MMI_CHG_VOTER)) &&
	    (!get_client_vote(chg_disable_votable, CHG_LIMIT_CHG_VOTER)) &&
	    (!get_client_vote(chg_suspend_votable, CHG_LIMIT_CHG_VOTER)));
}

static int oplus_chg_track_slowchg_info_reset(struct oplus_chg_track_slowchg_info  *slowchg_info)
{
	if (!slowchg_info)
		return -EINVAL;

	mutex_lock(&slowchg_info->lock);
	slowchg_info->retention_once = false;
	slowchg_info->usbtemp_once = false;
	strcpy(slowchg_info->first_adapter_type, "unknow");
	strcpy(slowchg_info->last_adapter_type, "unknow");
	strcpy(slowchg_info->last_fastchg_break_reason, "");
	mutex_unlock(&slowchg_info->lock);

	return 0;
}

static int oplus_chg_track_cp_abnormal_info_reset(struct track_cp_abnormal_info *cp_abnormal_info)
{
	int ret = 0;

	if (!cp_abnormal_info)
		return -EINVAL;

	mutex_lock(&cp_abnormal_info->lock);
	cp_abnormal_info->record_cnt = 0;
	memset(&(cp_abnormal_info->err_type_vector), 0, sizeof(cp_abnormal_info->err_type_vector));
	mutex_unlock(&cp_abnormal_info->lock);

	return ret;
}

static void oplus_chg_track_record_cp_abnormal_info(
	struct oplus_chg_track *track_chip, oplus_chg_track_trigger *p_trigger_data, int *index)
{
	int i;
	struct track_cp_abnormal_info *cp_abnormal_info;

	if (!g_track_chip || !track_chip || !p_trigger_data || !index)
		return;

	cp_abnormal_info = &g_track_chip->cp_abnormal_info;
	mutex_lock(&cp_abnormal_info->lock);
	if (cp_abnormal_info->record_cnt)
		*index += scnprintf(&(p_trigger_data->crux_info[*index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - *index,
			"$$cp_err_reason@@");
	for (i = 0; i < cp_abnormal_info->record_cnt; i++) {
		*index += scnprintf(&(p_trigger_data->crux_info[*index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - *index,
			"%s,", cp_abnormal_info->err_type_vector[i]);
	}
	mutex_unlock(&cp_abnormal_info->lock);
}

static int oplus_chg_track_cp_abnormal_type_vector_ele_add(char *err_reason)
{
	int ret = 0;
	int vector_size;
	int len = OPLUS_CHG_TRACK_DEVICE_NAME_LEN - 1;
	struct track_cp_abnormal_info *cp_abnormal_info;
	char err_type_vector[OPLUS_CHG_TRACK_DEVICE_ERR_NAME_LEN] = {0};
	char *str = NULL;
	char *temp_str = NULL;

	if (!g_track_chip || !err_reason)
		return -EINVAL;

	str = strstr(err_reason, "$$err_reason@@");
	if (!str) {
		chg_info("not contain err_reason\n");
		return -EINVAL;
	}

	str += strlen("$$err_reason@@");
	temp_str = strstr(str, "$$");
	if (temp_str)
		len = abs((int)(str - temp_str));

	if (len > OPLUS_CHG_TRACK_DEVICE_NAME_LEN - 1)
		len = OPLUS_CHG_TRACK_DEVICE_NAME_LEN - 1;
	strncpy(err_type_vector, str, len);

	cp_abnormal_info = &g_track_chip->cp_abnormal_info;
	mutex_lock(&cp_abnormal_info->lock);
	vector_size = ARRAY_SIZE(cp_abnormal_info->err_type_vector);
	if (cp_abnormal_info->record_cnt == vector_size) {
		chg_info("ele vector is full\n");
		mutex_unlock(&cp_abnormal_info->lock);
		return ret;
	}

	strncpy(cp_abnormal_info->err_type_vector[cp_abnormal_info->record_cnt],
		err_type_vector, OPLUS_CHG_TRACK_DEVICE_ERR_NAME_LEN);
	cp_abnormal_info->record_cnt++;

	if (cp_abnormal_info->record_cnt > vector_size)
		cp_abnormal_info->record_cnt = vector_size;

	chg_debug("ele_nums=%d, err_type_ele=%s\n", cp_abnormal_info->record_cnt, err_type_vector);
	mutex_unlock(&cp_abnormal_info->lock);

	return ret;
}

static int oplus_chg_track_force_upload_log(
	struct oplus_chg_track *chip, unsigned int type_reason, unsigned int flag_reason)
{
	char flag_reason_tag[OPLUS_CHG_TRIGGER_REASON_TAG_LEN] = { 0 };

	if (!chip)
		return -EINVAL;

	if (type_reason != TRACK_NOTIFY_TYPE_UPLOAD_LOG ||
	    flag_reason < TRACK_NOTIFY_FLAG_UPLOAD_LOG_FIRST ||
	    flag_reason > TRACK_NOTIFY_FLAG_UPLOAD_LOG_LAST)
		return -EINVAL;

	chg_debug("type_reason=%d, flag_reason=%d\n", type_reason, flag_reason);
	oplus_chg_track_get_flag_tag(flag_reason, flag_reason_tag);
	mutex_lock(&chip->trigger_data_lock);
	chg_exception_report(&chip->track_cfg.exception_data, type_reason,
		flag_reason, flag_reason_tag, sizeof(flag_reason_tag));
	mutex_unlock(&chip->trigger_data_lock);

	return 0;
}

static bool oplus_chg_track_check_vooc_user_exit(struct oplus_chg_track_power *power_info)
{
	struct votable *vooc_disable_votable;

	vooc_disable_votable = find_votable("VOOC_DISABLE");
	if (!vooc_disable_votable)
		return false;

	if (get_client_vote(vooc_disable_votable, USER_VOTER) &&
	    !is_client_vote_enabled(vooc_disable_votable, FASTCHG_DUMMY_VOTER) &&
	    (!strcmp(power_info->wired_info.adapter_type, "vooc") ||
	    !strcmp(power_info->wired_info.adapter_type, "svooc")))
		return true;

	return false;
}

static int oplus_chg_track_mul_break_init(struct oplus_chg_track *chip)
{
	int rc = 0;

	mutex_init(&(chip->mul_break_info.lock));
	INIT_DELAYED_WORK(&chip->wired_mul_break_trigger_work,
		oplus_chg_track_wired_mul_break_trigger_work);
	INIT_DELAYED_WORK(&chip->mul_break_info.ui_online_check_again_work,
		oplus_chg_track_wired_mul_break_ui_online_check_again_work);

	mutex_init(&(chip->wls_mul_break_info.lock));
	INIT_DELAYED_WORK(&chip->wls_mul_break_trigger_work,
		oplus_chg_track_wls_mul_break_trigger_work);

	return rc;
}

void oplus_chg_track_update_break_ui_online(void)
{
	struct oplus_chg_track *track_chip = g_track_chip;
	struct oplus_monitor *monitor;
	bool temp_ui_online = false;

	if (!track_chip)
		return;

	monitor = track_chip->monitor;
	chg_debug("online_check_status:%d, temp_ui_online:%d\n",
		track_chip->mul_break_info.ui_online_check_status, track_chip->mul_break_info.temp_ui_online);
	if (track_chip->mul_break_info.ui_online_check_status != TRACK_UI_ONLINE_CHECK_START)
		return;

	if (!track_chip->mul_break_info.temp_ui_online)
		return;

	if(monitor->total_disconnect_count > TRACK_PROTOCOL_SWITCH_COUNT || monitor->switch_protocol)
		track_chip->slowchg_info.retention_once = true;

	temp_ui_online = (monitor->wired_online || monitor->retention_state || monitor->keep_wired_online);
	chg_debug("online:%d,%d, wired_online:%d, retention_state:%d, keep_wired_online:%d\n",
		track_chip->mul_break_info.temp_ui_online, temp_ui_online,
		monitor->wired_online, monitor->retention_state, monitor->keep_wired_online);
	if (!temp_ui_online) {
		track_chip->mul_break_info.ui_online_check_status = TRACK_UI_ONLINE_CHECK_END;
		cancel_delayed_work(&track_chip->mul_break_info.ui_online_check_again_work);
		schedule_delayed_work(&track_chip->mul_break_info.ui_online_check_again_work, msecs_to_jiffies(100));
	}
}

static int oplus_chg_track_mul_break_info_reset(struct track_mul_break_info *mul_break_info, const char *chg_type)
{
	int ret = 0;

	if (!mul_break_info)
		return -EINVAL;

	if (g_track_chip && strcmp(chg_type, "wireless")) {
		g_track_chip->track_status.ufcs_err_type = TRACK_UFCS_ERR_DEFAULT;
		oplus_chg_track_cp_abnormal_info_reset(&g_track_chip->cp_abnormal_info);
	}

	mutex_lock(&mul_break_info->lock);
	mul_break_info->break_cnt = 0;
	mul_break_info->total_break_cnt = 0;
	mul_break_info->ele_nums = 0;
	mul_break_info->ui_online = true;
	memset(&(mul_break_info->wired_type), 0, sizeof(struct oplus_chg_track_wired_type));
	memset(&(mul_break_info->wls_type), 0, sizeof(struct oplus_chg_track_wls_type));
	strcpy(mul_break_info->wired_type.adapter_type, "unknow");
	strcpy(mul_break_info->wls_type.adapter_type, "unknow");
	strcpy(mul_break_info->wls_type.dock_type, "model_0");
	strcpy(mul_break_info->last_wired_adapter_type, "unknow");
	strcpy(mul_break_info->wls_type.first_wls_adapter_type, "unknow");
	memset(&(mul_break_info->reason), 0, OPLUS_CHG_TRACK_FASTCHG_BREAK_REASON_LEN);
	memset(&(mul_break_info->ele_vector), 0, sizeof(mul_break_info->ele_vector));
	mutex_unlock(&mul_break_info->lock);

	return ret;
}

static int oplus_chg_track_mul_break_vector_ele_add(
	struct track_mul_break_info *mul_break_info, struct mul_break_single_ele *record_ele)
{
	int ret = 0;
	int vector_size;
	struct mul_break_single_ele *tmp_vector;

	if (!mul_break_info || !record_ele)
		return -EINVAL;

	mutex_lock(&mul_break_info->lock);
	vector_size = ARRAY_SIZE(mul_break_info->ele_vector);
	if (mul_break_info->ele_nums == vector_size) {
		chg_debug("ele vector is full\n");
		tmp_vector = kzalloc(sizeof(struct mul_break_single_ele) * vector_size, GFP_KERNEL);
		if (!tmp_vector) {
			chg_err("ele vector alloc error\n");
			mutex_unlock(&mul_break_info->lock);
			return -EINVAL;
		}
		memcpy(tmp_vector, &(mul_break_info->ele_vector[1]),
			(vector_size - 1) * sizeof(struct mul_break_single_ele));
		memcpy(&(mul_break_info->ele_vector[0]), tmp_vector,
			(vector_size - 1) * sizeof(struct mul_break_single_ele));
		memcpy(&(mul_break_info->ele_vector[mul_break_info->ele_nums - 1]),
			record_ele, sizeof(struct mul_break_single_ele));
		kfree(tmp_vector);
	} else {
		memcpy(&(mul_break_info->ele_vector[mul_break_info->ele_nums]),
			record_ele, sizeof(struct mul_break_single_ele));
		mul_break_info->ele_nums++;
		if (mul_break_info->ele_nums > vector_size)
			mul_break_info->ele_nums = vector_size;
	}
	chg_debug("ele_nums=%d\n", mul_break_info->ele_nums);
	mutex_unlock(&mul_break_info->lock);

	return ret;
}

static int oplus_chg_track_update_mul_break_single_ele(struct oplus_chg_track *track_chip,
	struct mul_break_single_ele *record_ele, struct oplus_chg_track_power *power_info)
{
	struct oplus_chg_track_status *track_status = &track_chip->track_status;

	if (power_info->power_type == TRACK_CHG_TYPE_WIRELESS) {
		record_ele->break_jiffies = jiffies / HZ;
		memcpy(&(record_ele->wls_type), &(power_info->wls_info), sizeof(struct oplus_chg_track_wls_type));
		chg_info("break_jiffies:%ld, wireless_type_info:%d,%s,%s\n",
			record_ele->break_jiffies, record_ele->wls_type.power,
			record_ele->wls_type.dock_type, record_ele->wls_type.adapter_type);
	} else {
		record_ele->break_jiffies = jiffies / HZ;
		strncpy(record_ele->reason, track_status->fastchg_break_info.name,
			OPLUS_CHG_TRACK_FASTCHG_BREAK_REASON_LEN - 1);
		memcpy(&(record_ele->wired_type), &(power_info->wired_info), sizeof(struct oplus_chg_track_wired_type));

		chg_info("break_jiffies:%ld, break_reason:%s, wired_type_info:%d,%d,%s\n",
			record_ele->break_jiffies, record_ele->reason, record_ele->wired_type.power,
			record_ele->wired_type.adapter_id, record_ele->wired_type.adapter_type);
	}

	return 0;
}

static int oplus_chg_track_update_mul_break_wired_info(
	struct track_mul_break_info *mul_break_info, struct mul_break_single_ele *temp_buffer)
{

	if (!strlen(temp_buffer->wired_type.adapter_type) ||
	    !strcmp(temp_buffer->wired_type.adapter_type, "unknow")) {
		chg_debug("adapter type not identify\n");
		return 0;
	}

	if (strstr(mul_break_info->wired_type.adapter_type, "ufcs") ||
	    strstr(mul_break_info->wired_type.adapter_type, "vooc") ||
	    strstr(mul_break_info->wired_type.adapter_type, "pps") ||
	    !strcmp(mul_break_info->wired_type.adapter_type, "qc")) {
		chg_debug("type is:%s, not need update\n", mul_break_info->wired_type.adapter_type);
		return 0;
	}

	if (strcmp(mul_break_info->wired_type.adapter_type, "unknow") &&
	    strlen(temp_buffer->wired_type.adapter_type) &&
	   (!strcmp(temp_buffer->wired_type.adapter_type, "dcp") ||
	    !strcmp(temp_buffer->wired_type.adapter_type, "sdp") ||
	    !strcmp(temp_buffer->wired_type.adapter_type, "pd_sdp") ||
	    !strcmp(temp_buffer->wired_type.adapter_type, "cdp"))) {
		chg_debug("type is:%s, not need update\n", temp_buffer->wired_type.adapter_type);
		return 0;
	}

	if (mul_break_info->reason[0] == '\0' && temp_buffer->reason[0] != '\0') {
		strncpy(mul_break_info->reason, temp_buffer->reason, OPLUS_CHG_TRACK_FASTCHG_BREAK_REASON_LEN - 1);
		mul_break_info->reason[OPLUS_CHG_TRACK_FASTCHG_BREAK_REASON_LEN - 1] = '\0';
	}

	memcpy(&(mul_break_info->wired_type), &(temp_buffer->wired_type), sizeof(struct oplus_chg_track_wired_type));

	return 0;
}

static int oplus_chg_track_update_mul_break_info(struct oplus_chg_track *track_chip,
		struct track_mul_break_info *mul_break_info, const char *chg_type)
{
	int i;
	int j;
	int len;
	int break_cnt;
	unsigned long break_jiffies;

	mutex_lock(&mul_break_info->lock);
	mul_break_info->total_break_cnt += 1; /* total number of records per break + 1 */
	len = mul_break_info->ele_nums;
	for (i = 0; i < len; i++) {
		break_cnt = 1; /* init one time break */
		break_jiffies = mul_break_info->ele_vector[i].break_jiffies;

		if (!strcmp(chg_type, "wired")) {
			chg_debug("break_jiffies:%ld, break_reason:%s, wired_type_info:%d,%d,%s\n",
				mul_break_info->ele_vector[i].break_jiffies, mul_break_info->ele_vector[i].reason,
				mul_break_info->ele_vector[i].wired_type.power,
				mul_break_info->ele_vector[i].wired_type.adapter_id,
				mul_break_info->ele_vector[i].wired_type.adapter_type);
			oplus_chg_track_update_mul_break_wired_info(mul_break_info, &mul_break_info->ele_vector[i]);
			strncpy(track_chip->slowchg_info.first_adapter_type,
				mul_break_info->wired_type.adapter_type, OPLUS_CHG_TRACK_POWER_TYPE_LEN - 1);
		} else if (!strcmp(chg_type, "wireless")) {
			chg_info("break_jiffies:%ld, wls_type_info:%d,%s,%s\n",
				mul_break_info->ele_vector[i].break_jiffies,
				mul_break_info->ele_vector[i].wls_type.power,
				mul_break_info->ele_vector[i].wls_type.dock_type,
				mul_break_info->ele_vector[i].wls_type.adapter_type);
		}
		for (j = 0; j < i; j++) {
			 /* count of discontinuities within 60s plus 1*/
			if (break_jiffies - mul_break_info->ele_vector[j].break_jiffies <= 60)
				break_cnt++;
		}
		for (j = i + 1; j < len; j++) {
			 /* count of discontinuities within 60s plus 1*/
			if (mul_break_info->ele_vector[j].break_jiffies -break_jiffies <= 60)
				break_cnt++;
		}
		chg_info("break_cnt:%d,%d\n", mul_break_info->break_cnt, break_cnt);
		if (mul_break_info->break_cnt < break_cnt)
			mul_break_info->break_cnt = break_cnt;
	}
	mutex_unlock(&mul_break_info->lock);

	return 0;
}

static enum oplus_chg_track_device_name oplus_chg_track_get_protocol_ic_name(char *adapter_type)
{
	int i;
	char *str;
	struct track_device_name device_name[] = {
		{"pps", TRACK_PPS_DEVICE_NAME},
		{"vooc", TRACK_VOOC_DEVICE_NAME},
		{"svooc", TRACK_VOOC_DEVICE_NAME},
		{"ufcs", TRACK_UFCS_DEVICE_NAME},
	};

	if (!adapter_type)
		return TRACK_BUCK_DEVICE_NAME;

	for (i = 0; i < ARRAY_SIZE(device_name); i++) {
		str = strstr(adapter_type, device_name[i].suffixes);
		if (str)
			return device_name[i].index;
	}

	return TRACK_BUCK_DEVICE_NAME;
}

static void oplus_chg_track_record_device_name(struct oplus_chg_track *track_chip,
	char *adapter_type, oplus_chg_track_trigger *p_trigger_data, int *index)
{
	enum oplus_chg_track_device_name device_name;

	if (!track_chip || !p_trigger_data || !index || !adapter_type)
		return;

	device_name = oplus_chg_track_get_protocol_ic_name(adapter_type);
	*index += scnprintf(&(p_trigger_data->crux_info[*index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - *index,
		"$$device_name@@%s", track_chip->device_ic_name[device_name]);
	if (device_name == TRACK_BUCK_DEVICE_NAME)
		track_chip->charging_break_trigger.flag_reason = TRACK_NOTIFY_FLAG_GENERAL_CHARGING_BREAK;
	else
		track_chip->charging_break_trigger.flag_reason = TRACK_NOTIFY_FLAG_FAST_CHARGING_BREAK;
}

static void oplus_chg_track_record_wired_break_info(struct oplus_chg_track *track_chip)
{
	int index = 0;
	struct oplus_chg_track_status *track_status;
	struct oplus_monitor *monitor;

	if (track_chip == NULL)
		return;

	monitor = track_chip->monitor;
	track_status = &(track_chip->track_status);
	memset(track_chip->charging_break_trigger.crux_info, 0,
		sizeof(track_chip->charging_break_trigger.crux_info));
	index += scnprintf(&(track_chip->charging_break_trigger.crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,"$$power_mode@@wired");
	index += scnprintf(&(track_chip->charging_break_trigger.crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$adapter_t@@%s", track_chip->mul_break_info.wired_type.adapter_type);
	index += scnprintf(&(track_chip->charging_break_trigger.crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$adapter_id@@0x%x",
		track_chip->mul_break_info.wired_type.adapter_id);
	index += scnprintf(&(track_chip->charging_break_trigger.crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$power@@%d", track_chip->mul_break_info.wired_type.power);

	if (strlen(track_chip->mul_break_info.last_wired_adapter_type))
		index += scnprintf(&(track_chip->charging_break_trigger.crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$last_adapter_t@@%s",
		track_chip->mul_break_info.last_wired_adapter_type);
	else
		index += scnprintf(&(track_chip->charging_break_trigger.crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$last_adapter_t@@unknow");

	index += scnprintf(&(track_chip->charging_break_trigger.crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$break_cnt_total@@%d", track_chip->mul_break_info.total_break_cnt);

	index += scnprintf(&(track_chip->charging_break_trigger.crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$break_cnt_1min@@%d", track_chip->mul_break_info.break_cnt);
	index += scnprintf(&(track_chip->charging_break_trigger.crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$ui_online@@%d", track_chip->mul_break_info.ui_online);

	if (strlen(track_chip->mul_break_info.reason))
		index += scnprintf(&(track_chip->charging_break_trigger.crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$reason@@%s", track_chip->mul_break_info.reason);
	else
		index += scnprintf(&(track_chip->charging_break_trigger.crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$reason@@unknow");

	oplus_chg_track_record_device_name(track_chip,
		track_chip->mul_break_info.wired_type.adapter_type, &(track_chip->charging_break_trigger), &index);

	oplus_chg_track_record_cp_abnormal_info(track_chip, &(track_chip->charging_break_trigger), &index);

	oplus_chg_track_record_general_info(monitor, track_status, &(track_chip->charging_break_trigger), index);
	chg_info("wired[%s]\n", track_chip->charging_break_trigger.crux_info);
}

static void oplus_chg_track_record_wls_break_info(struct oplus_chg_track *track_chip)
{
	int index = 0;
	struct oplus_chg_track_status *track_status;
	struct oplus_monitor *monitor;

	if (track_chip == NULL)
		return;

	monitor = track_chip->monitor;
	track_status = &(track_chip->track_status);
	memset(track_chip->wls_charging_break_trigger.crux_info, 0,
	       sizeof(track_chip->wls_charging_break_trigger.crux_info));
	index += scnprintf(&(track_chip->wls_charging_break_trigger.crux_info[index]),
	                   OPLUS_CHG_TRACK_CURX_INFO_LEN - index,"$$power_mode@@wireless");
	index += scnprintf(&(track_chip->wls_charging_break_trigger.crux_info[index]),
	                   OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
	                   "$$adapter_t@@%s", track_chip->wls_mul_break_info.wls_type.adapter_type);
	index += scnprintf(&(track_chip->wls_charging_break_trigger.crux_info[index]),
	                   OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$dock_type@@%s",
	                   track_chip->wls_mul_break_info.wls_type.dock_type);
	index += scnprintf(&(track_chip->wls_charging_break_trigger.crux_info[index]),
	                   OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$power@@%d", track_chip->wls_mul_break_info.wls_type.power);

	if (strlen(track_chip->wls_mul_break_info.wls_type.first_wls_adapter_type))
		index += scnprintf(&(track_chip->wls_charging_break_trigger.crux_info[index]),
		                   OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$first_adapter_t@@%s",
		                   track_chip->wls_mul_break_info.wls_type.first_wls_adapter_type);
	else
		index += scnprintf(&(track_chip->wls_charging_break_trigger.crux_info[index]),
		                   OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$first_adapter_t@@unknow");

	index += scnprintf(&(track_chip->wls_charging_break_trigger.crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
	                   "$$break_cnt_total@@%d", track_chip->wls_mul_break_info.total_break_cnt);

	index += scnprintf(&(track_chip->wls_charging_break_trigger.crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
	                   "$$break_cnt_1min@@%d", track_chip->wls_mul_break_info.break_cnt);
	index += scnprintf(&(track_chip->wls_charging_break_trigger.crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
	                   "$$ui_online@@%d", track_chip->wls_mul_break_info.ui_online);
	index += scnprintf(&(track_chip->wls_charging_break_trigger.crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
	                   "$$first_tx_manu_id@@0x%x", track_chip->wls_mul_break_info.wls_type.tx_manu_id);
	index += scnprintf(&(track_chip->wls_charging_break_trigger.crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
	                  "$$first_vendor_id@@0x%x", track_chip->wls_mul_break_info.wls_type.vendor_id);
	index += scnprintf(&(track_chip->wls_charging_break_trigger.crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
	                   "$$first_product_id@@0x%x", track_chip->wls_mul_break_info.wls_type.product_id);
	index += scnprintf(&(track_chip->wls_charging_break_trigger.crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
	                   "$$fastchg_break@@0x%d", track_chip->wls_mul_break_info.wls_type.wls_fastchg_break);

	oplus_chg_track_record_general_info(monitor, track_status, &(track_chip->wls_charging_break_trigger), index);
	chg_info("wls[%s]\n", track_chip->wls_charging_break_trigger.crux_info);
}


static void oplus_chg_track_wired_mul_break_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(dwork, struct oplus_chg_track, wired_mul_break_trigger_work);

	oplus_chg_track_record_wired_break_info(chip);
	oplus_chg_track_mul_break_info_reset(&chip->mul_break_info, "wired");
	schedule_delayed_work(&chip->charging_break_trigger_work, 0);
}

static void oplus_chg_track_wls_mul_break_trigger_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip = container_of(dwork, struct oplus_chg_track, wls_mul_break_trigger_work);
	struct oplus_monitor *monitor;
	struct oplus_chg_track_status *track_status;

	track_status = &chip->track_status;
	monitor = chip->monitor;


	oplus_chg_track_record_wls_break_info(chip);
	oplus_chg_track_mul_break_info_reset(&chip->wls_mul_break_info, "wireless");
	chip->wls_charging_break_trigger.flag_reason = TRACK_NOTIFY_FLAG_WLS_CHARGING_BREAK;
	schedule_delayed_work(&chip->wls_charging_break_trigger_work, 0);
}

static void oplus_chg_track_wired_mul_break_ui_online_check_again_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct track_mul_break_info *mul_break_chip =
		container_of(dwork, struct track_mul_break_info, ui_online_check_again_work);
	struct oplus_chg_track *track_chip = container_of(mul_break_chip, struct oplus_chg_track, mul_break_info);
	struct oplus_monitor *monitor;

	monitor = track_chip->monitor;
	chg_debug("online_check_status:%d, temp_ui_online:%d\n",
		track_chip->mul_break_info.ui_online_check_status, track_chip->mul_break_info.temp_ui_online);

	if (track_chip->mul_break_info.ui_online_check_status != TRACK_UI_ONLINE_CHECK_END)
		return;

	track_chip->mul_break_info.temp_ui_online = (monitor->wired_online ||
						     monitor->retention_state ||
						     monitor->keep_wired_online);
	chg_info("online:%d,wired_online:%d, retention_state:%d, keep_wired_online:%d\n",
		track_chip->mul_break_info.temp_ui_online,
		monitor->wired_online, monitor->retention_state,
		monitor->keep_wired_online);
}

static int oplus_chg_track_handle_no_user_break(struct oplus_chg_track *track_chip,
	struct mul_break_single_ele *record_ele, struct oplus_chg_track_power *power_info)
{
	if (!track_chip || !record_ele || !power_info)
		return -EINVAL;

	if (power_info->power_type == TRACK_CHG_TYPE_WIRELESS) {
		chg_info("power_type:%d,adapter_type:%s,ui_online:%d, track_wls_status_keep:%d\n",
		         power_info->power_type, power_info->wls_info.adapter_type,
		         track_chip->wls_mul_break_info.ui_online, track_chip->track_status.wls_status_keep);
		oplus_chg_track_update_mul_break_single_ele(track_chip, record_ele, power_info);
		oplus_chg_track_mul_break_vector_ele_add(&track_chip->wls_mul_break_info, record_ele);
		oplus_chg_track_update_mul_break_info(track_chip, &track_chip->wls_mul_break_info, "wireless");
		if (track_chip->wls_mul_break_info.total_break_cnt == 1) /* upload log for the first time break */
			oplus_chg_track_force_upload_log(track_chip, TRACK_NOTIFY_TYPE_UPLOAD_LOG, TRACK_NOTIFY_FLAG_UPLOAD_WLS_BREAK_LOG);
	} else {
		oplus_chg_track_match_fastchg_break_reason(track_chip);
		oplus_chg_track_update_mul_break_single_ele(track_chip, record_ele, power_info);
		oplus_chg_track_mul_break_vector_ele_add(&track_chip->mul_break_info, record_ele);
		oplus_chg_track_update_mul_break_info(track_chip, &track_chip->mul_break_info, "wired");
		if (track_chip->mul_break_info.total_break_cnt == 1) /* upload log for the first time break */
			oplus_chg_track_force_upload_log(
				track_chip, TRACK_NOTIFY_TYPE_UPLOAD_LOG, TRACK_NOTIFY_FLAG_UPLOAD_BREAK_LOG);

		chg_info("temp_ui_online:%d, ui_online:%d\n",
		track_chip->mul_break_info.temp_ui_online, track_chip->mul_break_info.ui_online);
		if (!track_chip->mul_break_info.temp_ui_online)
			track_chip->mul_break_info.ui_online = track_chip->mul_break_info.temp_ui_online;
	}

	return 0;
}

int oplus_chg_track_check_wired_mul_break_stat(int vbus_rising)
{
	struct oplus_chg_track *track_chip = g_track_chip;
	struct oplus_chg_track_status *track_status;
	static struct oplus_chg_track_power power_info;
	struct oplus_monitor *monitor;
	static bool pre_vbus_rising = false;
	static bool vooc_user_exit = false;
	bool fastchg_code_ok;
	struct mul_break_single_ele record_ele;
	int wired_break_work_delay_t;

	if (!track_chip || track_chip->track_cfg.track_ver < TRACK_VER_4_0)
		return -EINVAL;

	monitor = track_chip->monitor;
	track_status = &track_chip->track_status;
	wired_break_work_delay_t = track_chip->track_cfg.fast_chg_break_t_thd + TRACK_TIME_500MS_JIFF_THD;

	chg_info("pre_vbus_rising[%d], vbus_rising[%d]\n", pre_vbus_rising, vbus_rising);
	if (vbus_rising == pre_vbus_rising)
		return 0;

	if (vbus_rising) {
		track_status->chg_attach_time_ms = local_clock() / TRACK_LOCAL_T_NS_TO_MS_THD;
		fastchg_code_ok = oplus_chg_track_wired_fastchg_exit_code(track_chip);
		chg_info("detal_t:%llu, chg_attach_time = %llu, fastchg_break_code=0x%x, vooc_user_exit=%d\n",
			track_status->chg_attach_time_ms - track_status->chg_detach_time_ms,
			track_status->chg_attach_time_ms, track_status->fastchg_break_info.code, vooc_user_exit);
		if (track_status->chg_detach_time_ms &&
		   (track_status->chg_attach_time_ms - track_status->chg_detach_time_ms <
		    track_chip->track_cfg.fast_chg_break_t_thd)) {
			if (!vooc_user_exit && (!track_status->fastchg_break_info.code || !fastchg_code_ok))
				oplus_chg_track_handle_no_user_break(track_chip, &record_ele, &power_info);
			if (!track_status->wls_need_upload) {
				cancel_delayed_work_sync(&track_chip->charger_info_trigger_work);
			}
			cancel_delayed_work(&track_chip->wired_mul_break_trigger_work);
		} else {
			oplus_chg_track_slowchg_info_reset(&track_chip->slowchg_info);
			if (!delayed_work_pending(&track_chip->wired_mul_break_trigger_work)) {
				oplus_chg_track_mul_break_info_reset(&track_chip->mul_break_info, "wired");
			} else {
				cancel_delayed_work(&track_chip->wired_mul_break_trigger_work);
				schedule_delayed_work(&track_chip->wired_mul_break_trigger_work, 0);
			}
		}
		track_chip->mul_break_info.ui_online_check_status = TRACK_UI_ONLINE_CHECK_END;
		strncpy(track_chip->slowchg_info.last_fastchg_break_reason,
		track_status->fastchg_break_info.name, OPLUS_CHG_TRACK_FASTCHG_BREAK_REASON_LEN - 1);
		memset(&(track_status->fastchg_break_info), 0, sizeof(track_status->fastchg_break_info));
		memset(&(track_status->power_info), 0, sizeof(track_status->power_info));
		strcpy(track_status->power_info.power_mode, "unknow");
		track_chip->monitor->pre_vooc_sid = 0;
	} else {
		track_chip->mul_break_info.temp_ui_online = true;
		track_chip->mul_break_info.ui_online_check_status = TRACK_UI_ONLINE_CHECK_START;
		oplus_chg_track_update_break_ui_online();
		track_status->chg_detach_time_ms = local_clock() / TRACK_LOCAL_T_NS_TO_MS_THD;
		oplus_chg_track_handle_wired_type_info(monitor, TRACK_CHG_GET_LAST_TIME_TYPE);
		vooc_user_exit = oplus_chg_track_check_vooc_user_exit(&(track_status->power_info));
		power_info = track_status->power_info;
		strncpy(track_chip->mul_break_info.last_wired_adapter_type,
			power_info.wired_info.adapter_type, OPLUS_CHG_TRACK_POWER_TYPE_LEN - 1);
		strncpy(track_chip->slowchg_info.last_adapter_type,
			power_info.wired_info.adapter_type, OPLUS_CHG_TRACK_POWER_TYPE_LEN - 1);
		if (track_chip->mul_break_info.break_cnt)
			schedule_delayed_work(&track_chip->wired_mul_break_trigger_work,
				msecs_to_jiffies(wired_break_work_delay_t));
	}
	pre_vbus_rising = vbus_rising;

	return 0;

}

static bool oplus_chg_track_wls_break_manually(struct oplus_chg_track *track_chip)
{
	union mms_msg_data data = { 0 };
	bool manu_break = 0;

	if (!track_chip || !track_chip->monitor || !track_chip->monitor->wls_topic)
		return false;

	oplus_mms_get_item_data(track_chip->monitor->wls_topic, WLS_ITEM_TRACK_BREAK_MANU, &data, true);
	manu_break = data.intval;

	chg_info("manu_break = %d\n", manu_break);

	return manu_break;
}

static void oplus_chg_track_check_wls_mul_break_info_update(struct oplus_chg_track_power *power_info)
{
	struct oplus_chg_track *track_chip;
	struct oplus_chg_track_status *track_status;
	struct oplus_monitor *monitor;
	union mms_msg_data data = { 0 };
	struct track_mul_break_info *wls_mul_break_info;
	char *adapter_type = NULL;
	char *dock_type = NULL;

	if (!power_info)
		return;

	track_chip = g_track_chip;
	track_status = &track_chip->track_status;
	monitor = track_chip->monitor;
	wls_mul_break_info = &track_chip->wls_mul_break_info;

	if (!wls_mul_break_info->total_break_cnt || !wls_mul_break_info->wls_type.tx_manu_id) {
		oplus_mms_get_item_data(monitor->wls_topic, WLS_ITEM_TX_MANU_ID, &data, true);
		wls_mul_break_info->wls_type.tx_manu_id = data.intval;
		chg_debug("tx_manu_id = %x\n", wls_mul_break_info->wls_type.tx_manu_id);
	}
	if (!wls_mul_break_info->total_break_cnt || !wls_mul_break_info->wls_type.vendor_id) {
		oplus_mms_get_item_data(monitor->wls_topic, WLS_ITEM_VENDOR_ID, &data, true);
		wls_mul_break_info->wls_type.vendor_id = data.intval;
		chg_debug("vendor_id = %x\n", wls_mul_break_info->wls_type.vendor_id);
	}
	if (!wls_mul_break_info->total_break_cnt || !wls_mul_break_info->wls_type.product_id) {
		oplus_mms_get_item_data(monitor->wls_topic, WLS_ITEM_PRODUCT_ID, &data, true);
		wls_mul_break_info->wls_type.product_id = data.intval;
		chg_debug("product_id = %x\n", wls_mul_break_info->wls_type.product_id);
	}

	adapter_type = power_info->wls_info.adapter_type;
	if (adapter_type && strcmp(adapter_type, "unknow") && power_info->wls_info.adapter_type[0] != '\0') {
		strncpy(wls_mul_break_info->wls_type.adapter_type,
			power_info->wls_info.adapter_type, OPLUS_CHG_TRACK_POWER_TYPE_LEN - 1);
		wls_mul_break_info->wls_type.adapter_type[OPLUS_CHG_TRACK_POWER_TYPE_LEN - 1] = '\0';
	}

	dock_type = power_info->wls_info.dock_type;
	if (dock_type && strcmp(dock_type, "model_0") && power_info->wls_info.dock_type[0] != '\0') {
		strncpy(wls_mul_break_info->wls_type.dock_type,
			power_info->wls_info.dock_type, OPLUS_CHG_TRACK_POWER_TYPE_LEN - 1);
		wls_mul_break_info->wls_type.dock_type[OPLUS_CHG_TRACK_POWER_TYPE_LEN - 1] = '\0';
	}

	if (wls_mul_break_info->wls_type.power < power_info->wls_info.power)
		wls_mul_break_info->wls_type.power = power_info->wls_info.power;

	if (!strcmp(wls_mul_break_info->wls_type.first_wls_adapter_type, "unknow") &&
	    wls_mul_break_info->wls_type.adapter_type[0] != '\0') {
		strncpy(wls_mul_break_info->wls_type.first_wls_adapter_type,
			wls_mul_break_info->wls_type.adapter_type, OPLUS_CHG_TRACK_POWER_TYPE_LEN - 1);
		wls_mul_break_info->wls_type.first_wls_adapter_type[OPLUS_CHG_TRACK_POWER_TYPE_LEN - 1] = '\0';
	}

	oplus_mms_get_item_data(monitor->wls_topic, WLS_ITEM_TRACK_RECORD_FASTCHG, &data, true);
	wls_mul_break_info->wls_type.wls_fastchg_break |= data.intval;

	return;
}

int oplus_chg_track_check_wls_mul_break_stat(int wls_connect)
{
	struct oplus_chg_track *track_chip;
	struct oplus_chg_track_status *track_status;
	struct oplus_monitor *monitor;
	static struct oplus_chg_track_power power_info;
	static bool pre_wls_connect = false;
	unsigned long long delta_time_ms;
	bool is_normal_chg;
	struct mul_break_single_ele record_ele;
	union mms_msg_data data = { 0 };

	if (!g_track_chip || g_track_chip->track_cfg.track_ver < TRACK_VER_4_0)
		return -1;

	track_chip = g_track_chip;
	track_status = &track_chip->track_status;
	monitor = track_chip->monitor;


	if (pre_wls_connect == wls_connect)
		return 0;
	is_normal_chg = oplus_chg_track_wls_is_normal_chg(monitor->wls_pre_type);
	if (wls_connect) {
		track_status->wls_status_keep = oplus_chg_track_wls_is_status_keep(track_chip);
		oplus_mms_get_item_data(track_chip->monitor->wls_topic, WLS_ITEM_STATUS_KEEP, &data, true);
		track_status->wls_attach_time_ms = local_clock() / TRACK_LOCAL_T_NS_TO_MS_THD;
		delta_time_ms = track_status->wls_attach_time_ms - track_status->wls_detach_time_ms;
		chg_info("wls_attach_time = %llu, wls_manu_break:%d\n", track_status->wls_attach_time_ms, track_status->wls_manu_break);
		if ((delta_time_ms < track_chip->track_cfg.wls_chg_break_t_thd && !is_normal_chg && !track_status->wls_manu_break) ||
		    (delta_time_ms < track_chip->track_cfg.wls_normal_chg_break_t_thd && is_normal_chg && !track_status->wls_manu_break)) {
			power_info.power_type = TRACK_CHG_TYPE_WIRELESS;
			if (delta_time_ms > TRACK_T_THD_2000_MS)
				track_chip->wls_mul_break_info.ui_online &= track_chip->track_status.wls_status_keep;
			oplus_chg_track_handle_no_user_break(track_chip, &record_ele, &power_info);
			cancel_delayed_work(&track_chip->wls_mul_break_trigger_work);

			if (!track_status->wired_need_upload) {
				cancel_delayed_work_sync(&track_chip->charger_info_trigger_work);
				cancel_delayed_work_sync(&track_chip->no_charging_trigger_work);
				cancel_delayed_work_sync(&track_chip->slow_charging_trigger_work);
			}
		} else {
			if (!delayed_work_pending(&track_chip->wls_mul_break_trigger_work)) {
				oplus_chg_track_mul_break_info_reset(&track_chip->wls_mul_break_info, "wireless");
			} else {
				cancel_delayed_work(&track_chip->wls_mul_break_trigger_work);
				schedule_delayed_work(&track_chip->wls_mul_break_trigger_work, 0);
			}
		}
		memset(&(track_status->power_info), 0, sizeof(track_status->power_info));
		strcpy(track_status->power_info.power_mode, "unknow");
		track_status->power_info.power_type = TRACK_CHG_TYPE_WIRELESS;
		track_status->wls_pre_dock_type = 0;
		track_status->wls_pre_max_power = 0;
		monitor->wls_pre_type = 0;
		track_status->wls_manu_break = false;
	} else {
		track_status->wls_manu_break = oplus_chg_track_wls_break_manually(track_chip);
		track_status->wls_detach_time_ms = local_clock() / TRACK_LOCAL_T_NS_TO_MS_THD;
		chg_info("wls_detach_time = %llu\n", track_status->wls_detach_time_ms);
		oplus_chg_track_handle_wls_type_info(track_status, TRACK_CHG_GET_LAST_TIME_TYPE);
		oplus_chg_track_obtain_wls_break_sub_crux_info(track_chip, track_status->wls_break_crux_info);
		power_info = track_status->power_info;
		oplus_chg_track_check_wls_mul_break_info_update(&power_info);

		if (track_chip->wls_mul_break_info.total_break_cnt)
			schedule_delayed_work(&track_chip->wls_mul_break_trigger_work,
				msecs_to_jiffies(track_chip->track_cfg.wls_chg_break_t_thd));
	}
	pre_wls_connect = wls_connect;

	return 0;
}

int oplus_chg_track_check_wired_charging_break(int vbus_rising)
{
	struct oplus_chg_track *track_chip;
	struct oplus_chg_track_status *track_status;
	static struct oplus_chg_track_power power_info;
	struct oplus_monitor *monitor;
	static bool break_recording = 0;
	static bool pre_vbus_rising = false;
	bool fastchg_code_ok;

	if (!g_track_chip || g_track_chip->track_cfg.track_ver >= TRACK_VER_4_0)
		return -1;


	track_chip = g_track_chip;
	monitor = track_chip->monitor;
	track_status = &track_chip->track_status;

	chg_info("pre_vbus_rising[%d], vbus_rising[%d], break_recording[%d]\n",
		 pre_vbus_rising, vbus_rising, break_recording);

	if (vbus_rising && (pre_vbus_rising != vbus_rising)) {
		track_status->chg_attach_time_ms =
			local_clock() / TRACK_LOCAL_T_NS_TO_MS_THD;
		if (track_status->debug_break_code)
			track_status->fastchg_break_info.code = track_status->debug_break_code;
		fastchg_code_ok =
			oplus_chg_track_wired_fastchg_exit_code(track_chip);
		chg_info(
			"detal_t:%llu, chg_attach_time = %llu, fastchg_break_code=0x%x\n",
			track_status->chg_attach_time_ms -
				track_status->chg_detach_time_ms,
			track_status->chg_attach_time_ms,
			track_status->fastchg_break_info.code);
		if ((track_status->chg_attach_time_ms -
			     track_status->chg_detach_time_ms <
		     track_chip->track_cfg.fast_chg_break_t_thd) &&
		    (!fastchg_code_ok || strstr(power_info.wired_info.adapter_type, "ufcs")) &&
		    track_status->mmi_chg) {
			if (!break_recording) {
				break_recording = true;
				track_chip->charging_break_trigger.flag_reason =
					TRACK_NOTIFY_FLAG_FAST_CHARGING_BREAK;
				oplus_chg_track_match_fastchg_break_reason(
					track_chip);
				oplus_chg_track_record_break_charging_info(
					track_chip, monitor, power_info,
					track_status->wired_break_crux_info);
				memset(&(track_status->fastchg_break_info), 0,
				       sizeof(track_status->fastchg_break_info));
				schedule_delayed_work(
					&track_chip->charging_break_trigger_work,
					0);
			}
			if (!track_status->wls_need_upload) {
				cancel_delayed_work_sync(
					&track_chip->charger_info_trigger_work);
				cancel_delayed_work_sync(
					&track_chip->no_charging_trigger_work);
				cancel_delayed_work_sync(
					&track_chip->slow_charging_trigger_work);
			}
		} else if ((track_status->chg_attach_time_ms -
				    track_status->chg_detach_time_ms <
			    track_chip->track_cfg.general_chg_break_t_thd) &&
			   !track_status->fastchg_break_info.code &&
			   track_status->mmi_chg) {
			if (!break_recording) {
				break_recording = true;
				track_chip->charging_break_trigger.flag_reason =
					TRACK_NOTIFY_FLAG_GENERAL_CHARGING_BREAK;
				oplus_chg_track_record_break_charging_info(
					track_chip, monitor, power_info,
					track_status->wired_break_crux_info);
				schedule_delayed_work(
					&track_chip->charging_break_trigger_work,
					0);
			}
			if (!track_status->wls_need_upload) {
				cancel_delayed_work_sync(
					&track_chip->charger_info_trigger_work);
				cancel_delayed_work_sync(
					&track_chip->no_charging_trigger_work);
				cancel_delayed_work_sync(
					&track_chip->slow_charging_trigger_work);
			}
		} else {
			break_recording = 0;
		}
		memset(&(track_status->power_info), 0, sizeof(track_status->power_info));
		track_status->power_info.power_type = TRACK_CHG_TYPE_WIRE;
		strcpy(track_status->power_info.power_mode, "unknow");
	} else if (!vbus_rising && (pre_vbus_rising != vbus_rising)) {
		track_status->chg_detach_time_ms =
			local_clock() / TRACK_LOCAL_T_NS_TO_MS_THD;
		oplus_chg_track_handle_wired_type_info(
			monitor, TRACK_CHG_GET_LAST_TIME_TYPE);
		power_info = track_status->power_info;
		track_status->mmi_chg = oplus_chg_track_get_mmi_chg();
		chg_info("chg_detach_time = %llu, mmi_chg=%d\n",
			 track_status->chg_detach_time_ms,
			 track_status->mmi_chg);
	}

	if (vbus_rising) {
		track_status->mmi_chg = oplus_chg_track_get_mmi_chg();
		track_chip->monitor->pre_vooc_sid = 0;
		oplus_chg_track_set_fastchg_break_code(TRACK_VOOCPHY_BREAK_DEFAULT);
		monitor->pre_ufcs_oplus_adapter = monitor->ufcs_oplus_adapter;
		monitor->pre_ufcs_adapter_id = monitor->ufcs_adapter_id;
		monitor->pre_ufcs_emark_power = monitor->ufcs_emark_power;
		monitor->pre_ufcs_adapter_power = monitor->ufcs_adapter_power;
	}
	pre_vbus_rising = vbus_rising;

	return 0;
}

int oplus_chg_track_cal_tbatt_status(struct oplus_monitor *monitor)
{
	struct oplus_chg_track *track_chip;
	struct oplus_chg_track_status *track_status;

	if (!monitor)
		return -EINVAL;
	track_chip = monitor->track;
	if (!track_chip)
		return -EINVAL;
	track_status = &track_chip->track_status;

	if (monitor->batt_status != POWER_SUPPLY_STATUS_CHARGING)
	{
		chg_debug("not charging, should return\n");
		return 0;
	}

	if (!track_status->tbatt_warm_once &&
	    ((monitor->temp_region == TEMP_REGION_WARM) ||
	     (monitor->temp_region == TEMP_REGION_HOT)))
		track_status->tbatt_warm_once = true;

	if (!track_status->tbatt_cold_once &&
	    (monitor->temp_region == TEMP_REGION_COLD))
		track_status->tbatt_cold_once = true;

	chg_debug("tbatt_warm_once:%d, tbatt_cold_once:%d\n",
		 track_status->tbatt_warm_once, track_status->tbatt_cold_once);

	return 0;
}

int oplus_chg_track_aging_ffc_check(struct oplus_monitor *monitor, int step)
{
	struct oplus_chg_track *track_chip;
	struct oplus_chg_track_status *track_status;
	int rc;
	int cutoff_fv;

	if (!monitor)
		return -EINVAL;
	track_chip = monitor->track;
	if (!track_chip)
		return -EINVAL;
	track_status = &track_chip->track_status;

	if (step >= FFC_CHG_STEP_MAX) {
		chg_err("step(=%d) is too big\n", step);
		return -EINVAL;
	} else if (step < 0) {
		chg_err("step(=%d) is too small\n", step);
		return -EINVAL;
	}

	if (monitor->wired_online) {
		if (step == 0)
			track_status->aging_ffc_start_time = oplus_chg_track_get_local_time_s();
		rc = oplus_comm_get_wired_aging_ffc_offset(monitor->comm_topic, step);
		if (rc <= 0)
			return rc;
		track_status->aging_ffc_trig = true;
		cutoff_fv = oplus_comm_get_current_wired_ffc_cutoff_fv(monitor->comm_topic, step);
		if (cutoff_fv < 0)
			return cutoff_fv;
		track_status->aging_ffc_judge_vol[step] = cutoff_fv + rc;
	}

	return 0;
}

int oplus_chg_track_record_ffc_end_time(struct oplus_monitor *monitor)
{
	struct oplus_chg_track *track_chip;
	struct oplus_chg_track_status *track_status;

	if (!monitor)
		return -EINVAL;
	track_chip = monitor->track;
	if (!track_chip)
		return -EINVAL;
	track_status = &track_chip->track_status;

	if (monitor->wired_online) {
		track_status->ffc_end_time = oplus_chg_track_get_local_time_s();
	}

	return 0;
}

int oplus_chg_track_record_ffc_soc(struct oplus_monitor *monitor,
	int main_soc, int sub_soc, bool start)
{
	struct oplus_chg_track *track_chip;
	struct oplus_chg_track_status *track_status;

	if (!monitor)
		return -EINVAL;
	track_chip = monitor->track;
	if (!track_chip)
		return -EINVAL;
	track_status = &track_chip->track_status;

	if (start) {
		track_status->ffc_start_main_soc = main_soc;
		track_status->ffc_start_sub_soc = sub_soc;
	} else {
		track_status->ffc_end_main_soc = main_soc;
		track_status->ffc_end_sub_soc = sub_soc;
	}

	return 0;
}

int oplus_chg_track_record_dual_chan_start(struct oplus_monitor *monitor)
{
	struct oplus_chg_track *track_chip;
	struct oplus_chg_track_status *track_status;

	if (!monitor)
		return -EINVAL;
	track_chip = monitor->track;
	if (!track_chip)
		return -EINVAL;
	track_status = &track_chip->track_status;

	track_status->dual_chan_open_count++;
	track_status->dual_chan_start_time = oplus_chg_track_get_local_time_s();

	return 0;
}

int oplus_chg_track_record_dual_chan_end(struct oplus_monitor *monitor)
{
	struct oplus_chg_track *track_chip;
	struct oplus_chg_track_status *track_status;
	int dual_chan_end_time;

	if (!monitor)
		return -EINVAL;
	track_chip = monitor->track;
	if (!track_chip)
		return -EINVAL;
	track_status = &track_chip->track_status;

	dual_chan_end_time = oplus_chg_track_get_local_time_s();
	if (track_status->dual_chan_start_time) {
		track_status->dual_chan_time +=
			dual_chan_end_time - track_status->dual_chan_start_time;
		track_status->dual_chan_start_time = 0;
		chg_info("record_dual_chan_end: start_time:%d end_time:%d time:%d\n",
			 track_status->dual_chan_start_time, dual_chan_end_time, track_status->dual_chan_time);
	}

	return 0;
}

static int oplus_chg_track_cal_section_soc_inc_rm(
	struct oplus_monitor *monitor,
	struct oplus_chg_track_status *track_status)
{
	bool cal_cond = true;
	bool mmi_chg = false;
	static int time_go_next_status;
	static int rm_go_next_status;
	int curr_time;
	static int pre_soc_low_sect_incr_rm;
	static int pre_soc_low_sect_cont_time;
	static int pre_soc_medium_sect_incr_rm;
	static int pre_soc_medium_sect_cont_time;
	static int pre_soc_high_sect_incr_rm;
	static int pre_soc_high_sect_cont_time;

	if (!monitor || !track_status)
		return -1;

	if (track_status->soc_sect_status == TRACK_SOC_SECTION_DEFAULT) {
		track_status->soc_low_sect_incr_rm = 0;
		track_status->soc_low_sect_cont_time = 0;
		track_status->soc_medium_sect_incr_rm = 0;
		track_status->soc_medium_sect_cont_time = 0;
		track_status->soc_high_sect_incr_rm = 0;
		track_status->soc_high_sect_cont_time = 0;
		pre_soc_low_sect_incr_rm = 0;
		pre_soc_low_sect_cont_time = 0;
		pre_soc_medium_sect_incr_rm = 0;
		pre_soc_medium_sect_cont_time = 0;
		pre_soc_high_sect_incr_rm = 0;
		pre_soc_high_sect_cont_time = 0;
		if (monitor->batt_soc <= TRACK_REF_SOC_50)
			track_status->soc_sect_status = TRACK_SOC_SECTION_LOW;
		else if (monitor->batt_soc <= TRACK_REF_SOC_75)
			track_status->soc_sect_status =
				TRACK_SOC_SECTION_MEDIUM;
		else if (monitor->batt_soc <= TRACK_REF_SOC_90)
			track_status->soc_sect_status = TRACK_SOC_SECTION_HIGH;
		else
			track_status->soc_sect_status = TRACK_SOC_SECTION_OVER;
		time_go_next_status = oplus_chg_track_get_local_time_s();
		rm_go_next_status = monitor->batt_rm;
	}

	mmi_chg = oplus_chg_track_get_mmi_chg();
	if (monitor->track)
		cal_cond = (monitor->track->track_cfg.track_ver >= TRACK_VER_4_0 &&
			(!mmi_chg || monitor->plc_status == PLC_STATUS_ENABLE));

	if (monitor->batt_status != POWER_SUPPLY_STATUS_CHARGING || cal_cond) {
		if (monitor->batt_soc <= TRACK_REF_SOC_50)
			track_status->soc_sect_status = TRACK_SOC_SECTION_LOW;
		else if (monitor->batt_soc <= TRACK_REF_SOC_75)
			track_status->soc_sect_status =
				TRACK_SOC_SECTION_MEDIUM;
		else if (monitor->batt_soc <= TRACK_REF_SOC_90)
			track_status->soc_sect_status = TRACK_SOC_SECTION_HIGH;
		else
			track_status->soc_sect_status = TRACK_SOC_SECTION_OVER;
		pre_soc_low_sect_cont_time =
			track_status->soc_low_sect_cont_time;
		pre_soc_low_sect_incr_rm = track_status->soc_low_sect_incr_rm;
		pre_soc_medium_sect_cont_time =
			track_status->soc_medium_sect_cont_time;
		pre_soc_medium_sect_incr_rm =
			track_status->soc_medium_sect_incr_rm;
		pre_soc_high_sect_cont_time =
			track_status->soc_high_sect_cont_time;
		pre_soc_high_sect_incr_rm = track_status->soc_high_sect_incr_rm;
		time_go_next_status = oplus_chg_track_get_local_time_s();
		rm_go_next_status = monitor->batt_rm;
		return 0;
	}

	switch (track_status->soc_sect_status) {
	case TRACK_SOC_SECTION_LOW:
		curr_time = oplus_chg_track_get_local_time_s();
		track_status->soc_low_sect_cont_time =
			(curr_time - time_go_next_status) +
			pre_soc_low_sect_cont_time;
		track_status->soc_low_sect_incr_rm =
			(monitor->batt_rm - rm_go_next_status) +
			pre_soc_low_sect_incr_rm;
		if (monitor->batt_soc > TRACK_REF_SOC_50) {
			pre_soc_low_sect_cont_time =
				track_status->soc_low_sect_cont_time;
			pre_soc_low_sect_incr_rm =
				track_status->soc_low_sect_incr_rm;
			time_go_next_status = curr_time;
			rm_go_next_status = monitor->batt_rm;
			track_status->soc_sect_status =
				TRACK_SOC_SECTION_MEDIUM;
		}
		break;
	case TRACK_SOC_SECTION_MEDIUM:
		curr_time = oplus_chg_track_get_local_time_s();
		track_status->soc_medium_sect_cont_time =
			(curr_time - time_go_next_status) +
			pre_soc_medium_sect_cont_time;
		track_status->soc_medium_sect_incr_rm =
			(monitor->batt_rm - rm_go_next_status) +
			pre_soc_medium_sect_incr_rm;
		if (monitor->batt_soc <= TRACK_REF_SOC_50) {
			pre_soc_medium_sect_cont_time =
				track_status->soc_medium_sect_cont_time;
			pre_soc_medium_sect_incr_rm =
				track_status->soc_medium_sect_incr_rm;
			time_go_next_status = curr_time;
			rm_go_next_status = monitor->batt_rm;
			track_status->soc_sect_status = TRACK_SOC_SECTION_LOW;
		} else if (monitor->batt_soc > TRACK_REF_SOC_75) {
			pre_soc_medium_sect_cont_time =
				track_status->soc_medium_sect_cont_time;
			pre_soc_medium_sect_incr_rm =
				track_status->soc_medium_sect_incr_rm;
			time_go_next_status = curr_time;
			rm_go_next_status = monitor->batt_rm;
			track_status->soc_sect_status = TRACK_SOC_SECTION_HIGH;
		}
		break;
	case TRACK_SOC_SECTION_HIGH:
		curr_time = oplus_chg_track_get_local_time_s();
		track_status->soc_high_sect_cont_time =
			(curr_time - time_go_next_status) +
			pre_soc_high_sect_cont_time;
		track_status->soc_high_sect_incr_rm =
			(monitor->batt_rm - rm_go_next_status) +
			pre_soc_high_sect_incr_rm;
		if (monitor->batt_soc <= TRACK_REF_SOC_75) {
			pre_soc_high_sect_cont_time =
				track_status->soc_high_sect_cont_time;
			pre_soc_high_sect_incr_rm =
				track_status->soc_high_sect_incr_rm;
			time_go_next_status = curr_time;
			rm_go_next_status = monitor->batt_rm;
			track_status->soc_sect_status =
				TRACK_SOC_SECTION_MEDIUM;
		} else if (monitor->batt_soc > TRACK_REF_SOC_90) {
			pre_soc_high_sect_cont_time =
				track_status->soc_high_sect_cont_time;
			pre_soc_high_sect_incr_rm =
				track_status->soc_high_sect_incr_rm;
			time_go_next_status = curr_time;
			rm_go_next_status = monitor->batt_rm;
			track_status->soc_sect_status = TRACK_SOC_SECTION_OVER;
		}
		break;
	case TRACK_SOC_SECTION_OVER:
		curr_time = oplus_chg_track_get_local_time_s();
		if (monitor->batt_soc < TRACK_REF_SOC_90) {
			time_go_next_status = curr_time;
			rm_go_next_status = monitor->batt_rm;
			track_status->soc_sect_status = TRACK_SOC_SECTION_HIGH;
		}
		break;
	default:
		chg_err("!!!soc section status is invalid\n");
		break;
	}

	chg_debug(
		 "soc_sect_status:%d, time_go_next_status:%d, rm_go_next_status:%d\n",
		 track_status->soc_sect_status, time_go_next_status,
		 rm_go_next_status);

	chg_debug("soc_low_sect_cont_time:%d, soc_low_sect_incr_rm:%d, "
		  "soc_medium_sect_cont_time:%d, soc_medium_sect_incr_rm:%d "
		  "soc_high_sect_cont_time:%d, soc_high_sect_incr_rm:%d\n",
		  track_status->soc_low_sect_cont_time,
		  track_status->soc_low_sect_incr_rm,
		  track_status->soc_medium_sect_cont_time,
		  track_status->soc_medium_sect_incr_rm,
		  track_status->soc_high_sect_cont_time,
		  track_status->soc_high_sect_incr_rm);

	return 0;
}

static bool oplus_chg_track_burst_soc_sect_speed(
	struct oplus_chg_track_status *track_status,
	struct oplus_chg_track_speed_ref *speed_ref)
{
	bool ret = false;
	int soc_low_sect_incr_rm;
	int soc_medium_sect_incr_rm;
	int soc_high_sect_incr_rm;
	int batt_num;

	if (!track_status || !speed_ref)
		return false;

	if (!track_status->soc_high_sect_cont_time &&
	    !track_status->soc_medium_sect_cont_time &&
	    !track_status->soc_low_sect_cont_time) {
		if (g_track_chip && g_track_chip->track_cfg.track_ver >= TRACK_VER_4_0)
			return false;
		else
			return true;
	}

	chg_info("low_ref_curr:%d, medium_ref_curr:%d, high_ref_curr:%d\n",
		 speed_ref[TRACK_SOC_SECTION_LOW - 1].ref_curr,
		 speed_ref[TRACK_SOC_SECTION_MEDIUM - 1].ref_curr,
		 speed_ref[TRACK_SOC_SECTION_HIGH - 1].ref_curr);

	soc_low_sect_incr_rm = track_status->soc_low_sect_incr_rm;
	soc_medium_sect_incr_rm = track_status->soc_medium_sect_incr_rm;
	soc_high_sect_incr_rm = track_status->soc_high_sect_incr_rm;

	batt_num = oplus_gauge_get_batt_num();
	soc_low_sect_incr_rm *= (TRACK_TIME_1HOU_THD / batt_num);
	soc_medium_sect_incr_rm *= (TRACK_TIME_1HOU_THD / batt_num);
	soc_high_sect_incr_rm *= (TRACK_TIME_1HOU_THD / batt_num);

	if ((track_status->soc_low_sect_cont_time > TRACK_REF_TIME_6S) &&
	    ((soc_low_sect_incr_rm / track_status->soc_low_sect_cont_time) <
	     speed_ref[TRACK_SOC_SECTION_LOW - 1].ref_curr)) {
		chg_info("slow charging when soc low section\n");
		ret = true;
	}

	if (!ret &&
	    (track_status->soc_medium_sect_cont_time > TRACK_REF_TIME_8S) &&
	    ((soc_medium_sect_incr_rm /
	      track_status->soc_medium_sect_cont_time) <
	     speed_ref[TRACK_SOC_SECTION_MEDIUM - 1].ref_curr)) {
		chg_info("slow charging when soc medium section\n");
		ret = true;
	}

	if (!ret &&
	    (track_status->soc_high_sect_cont_time > TRACK_REF_TIME_10S) &&
	    ((soc_high_sect_incr_rm / track_status->soc_high_sect_cont_time) <
	     speed_ref[TRACK_SOC_SECTION_HIGH - 1].ref_curr)) {
		chg_info("slow charging when soc high section\n");
		ret = true;
	}

	if (g_track_chip != NULL && g_track_chip->track_cfg.track_ver >= TRACK_VER_4_0 &&
	   (track_status->soc_high_sect_cont_time + track_status->soc_medium_sect_cont_time +
	    track_status->soc_low_sect_cont_time) < TRACK_CYCLE_RECORDIING_TIME_2MIN)
		ret = false;

	return ret;
}

static bool oplus_chg_track_protocol_fastchg_to_normal(struct oplus_chg_track *chip)
{
	struct oplus_chg_track_slowchg_info *slowchg_info;

	if (!chip)
		return false;

	slowchg_info = &chip->slowchg_info;
	chg_debug("first adapter type :%s, last adapter type:%s\n",
		slowchg_info->first_adapter_type,
		slowchg_info->last_adapter_type);
	if ((strstr(slowchg_info->first_adapter_type, "ufcs") ||
	    strstr(slowchg_info->first_adapter_type, "vooc") ||
	    strstr(slowchg_info->first_adapter_type, "pps")) &&
	    strstr(slowchg_info->last_adapter_type, "dcp")) {
		chg_info("change\n");
		return true;
	}

	return false;
}
static int oplus_chg_track_get_speed_slow_reason(
	struct oplus_chg_track_status *track_status)
{
	bool fastchg_to_normal = false;
	int chg_total_time;
	struct oplus_chg_track *chip = g_track_chip;
	char wired_adapter_type[OPLUS_CHG_TRACK_POWER_TYPE_LEN];
	char wls_adapter_type[OPLUS_CHG_TRACK_POWER_TYPE_LEN];

	if (!track_status || !chip)
		return -1;

	if (chip->track_cfg.track_ver >= TRACK_VER_4_0)
		chg_total_time =  track_status->chg_total_time / TRACK_THRAD_PERIOD_TIME_S;
	else
		chg_total_time =  track_status->chg_total_cnt;

	memcpy(wired_adapter_type,
	       track_status->power_info.wired_info.adapter_type,
	       OPLUS_CHG_TRACK_POWER_TYPE_LEN);
	memcpy(wls_adapter_type, track_status->power_info.wls_info.adapter_type,
	       OPLUS_CHG_TRACK_POWER_TYPE_LEN);
	chg_info("wired_adapter_type:%s, wls_adapter_type:%s\n",
		 wired_adapter_type, wls_adapter_type);

	fastchg_to_normal = oplus_chg_track_protocol_fastchg_to_normal(chip);

	if (fastchg_to_normal && chip->cp_abnormal_info.record_cnt)
		chip->slow_charging_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_CHG_SLOW_POWER_DEVICE_ABNORMAL;
	else if(fastchg_to_normal &&
		(!strcmp(chip->slowchg_info.last_fastchg_break_reason, "commu_time_out") ||
		!strcmp(chip->slowchg_info.last_fastchg_break_reason, "adapter_copycat") ||
		!strcmp(chip->slowchg_info.last_fastchg_break_reason, "frame_head_error") ||
		!strcmp(chip->slowchg_info.last_fastchg_break_reason, "clk_error")))
		chip->slow_charging_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_CHG_SLOW_COMMUNICATION_ABNORMAL;
	else if (track_status->ufcs_err_type == TRACK_UFCS_ERR_R_COOLDOWN ||
		!strcmp(chip->slowchg_info.last_fastchg_break_reason, "bad_connect"))
		chip->slow_charging_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_CHG_SLOW_R_COOLDOWN;
	else if (fastchg_to_normal && chip->slowchg_info.retention_once)
		chip->slow_charging_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_CHG_SLOW_RETENTION_FALLBACK;
	else if(!strcmp(chip->slowchg_info.last_fastchg_break_reason, "btb_temp_over") ||
		track_status->ufcs_err_type == TRACK_UFCS_ERR_BTB_OVER ||
		track_status->ufcs_err_type == TRACK_UFCS_ERR_USBTEMP_OVER ||
		chip->slowchg_info.usbtemp_once)
		chip->slow_charging_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_CHG_SLOW_BTB_OVER_TEMP;
	else if (track_status->tbatt_warm_once)
		chip->slow_charging_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_CHG_SLOW_TBATT_WARM;
	else if (track_status->tbatt_cold_once)
		chip->slow_charging_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_CHG_SLOW_TBATT_COLD;
	else if ((track_status->cool_down_effect_cnt * 100 / chg_total_time) >
		 TRACK_COOLDOWN_CHRGING_TIME_PCT)
		chip->slow_charging_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_CHG_SLOW_COOLDOWN;
	else if (track_status->chg_start_soc >= TRACK_REF_SOC_75)
		chip->slow_charging_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_CHG_SLOW_BATT_CAP_HIGH;
	else if ((track_status->power_info.power_type ==
		  TRACK_CHG_TYPE_UNKNOW) ||
		 (track_status->power_info.power_type == TRACK_CHG_TYPE_WIRE &&
		  chip->track_cfg.wired_max_power >
			  track_status->power_info.wired_info.power))
		chip->slow_charging_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_CHG_SLOW_NON_STANDARD_PA;
	else if ((track_status->power_info.power_type ==
		  TRACK_CHG_TYPE_WIRELESS) &&
		 (chip->track_cfg.wls_max_power - TRACK_POWER_10000MW >
		  track_status->power_info.wls_info.power) &&
		 (strcmp(wls_adapter_type, "bpp")) &&
		 (strcmp(wls_adapter_type, "epp")))
		chip->slow_charging_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_CHG_SLOW_NON_STANDARD_PA;
	else if (track_status->power_info.power_type == TRACK_CHG_TYPE_WIRELESS &&
		!strcmp(track_status->power_info.wls_info.adapter_type,"airsvooc") &&
		(track_status->wls_quiet_mdoe_t_total > track_status->wls_chg_t_total * 70 / 100) )
		chip->slow_charging_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_CHG_SLOW_QUIET_MODE;
	else if ((track_status->wls_skew_effect_cnt * 100 / chg_total_time) >
			 TRACK_WLS_SKEWING_CHRGING_TIME_PCT &&
		 track_status->power_info.power_type == TRACK_CHG_TYPE_WIRELESS)
		chip->slow_charging_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_CHG_SLOW_WLS_SKEW;
	else if (!track_status->chg_verity &&
		 track_status->power_info.power_type == TRACK_CHG_TYPE_WIRELESS)
		chip->slow_charging_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_CHG_SLOW_VERITY_FAIL;
	else if (chip->monitor && chip->monitor->curr_derating_trig)
		chip->slow_charging_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_CHG_SLOW_CYCLE_CURR_DERATING;
	else
		chip->slow_charging_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_CHG_SLOW_OTHER;

	if (track_status->debug_slow_charging_reason)
		chip->slow_charging_trigger.flag_reason =
			track_status->debug_slow_charging_reason +
			TRACK_NOTIFY_FLAG_CHG_SLOW_TBATT_WARM - 1;

	chg_info("flag_reason:%d, btb_over:%d, %d, %s\n",
		chip->slow_charging_trigger.flag_reason, track_status->ufcs_err_type,
		chip->slowchg_info.usbtemp_once, chip->slowchg_info.last_fastchg_break_reason);

	return 0;
}

static bool oplus_chg_track_judge_speed_slow(struct oplus_monitor *monitor,
					     struct oplus_chg_track *track_chip)
{
	struct oplus_chg_track_status *track_status;

	if (!track_chip || !monitor)
		return 0;

	track_status = &(track_chip->track_status);
	mutex_lock(&track_chip->access_lock);
	if (track_status->has_judge_speed) {
		mutex_unlock(&track_chip->access_lock);
		return (track_status->chg_speed_is_slow || track_status->debug_slow_charging_force);
	}
	track_status->has_judge_speed = true;
	mutex_unlock(&track_chip->access_lock);

	if (track_status->power_info.power_type == TRACK_CHG_TYPE_UNKNOW) {
		track_status->chg_speed_is_slow = true;
	} else if (track_status->power_info.power_type ==
		   TRACK_CHG_TYPE_WIRELESS) {
		if (!strcmp(track_status->power_info.wls_info.adapter_type,
			    "unknow")) {
			track_status->chg_speed_is_slow = true;
		} else if (!strcmp(track_status->power_info.wls_info.adapter_type,"epp")) {
			if (!track_status->wls_epp_speed_ref)
				return false;
			track_status->chg_speed_is_slow =
				oplus_chg_track_burst_soc_sect_speed(
					track_status,
					track_status->wls_epp_speed_ref);
		} else if (!strcmp(track_status->power_info.wls_info.adapter_type,"bpp")) {
			if (!track_status->wls_bpp_speed_ref)
				return false;
			track_status->chg_speed_is_slow =
				oplus_chg_track_burst_soc_sect_speed(
					track_status,
					track_status->wls_bpp_speed_ref);
		} else {
			if (!track_status->wls_airvooc_speed_ref)
				return false;
			track_status->chg_speed_is_slow =
				oplus_chg_track_burst_soc_sect_speed(
					track_status,
					track_status->wls_airvooc_speed_ref);
		}
	} else {
		if (!track_status->wired_speed_ref)
			return false;
		track_status->chg_speed_is_slow =
			oplus_chg_track_burst_soc_sect_speed(
				track_status, track_status->wired_speed_ref);
	}

	if (track_status->chg_speed_is_slow ||
	    track_status->debug_slow_charging_force) {
		oplus_chg_track_get_speed_slow_reason(track_status);
	}

	return (track_status->chg_speed_is_slow ||
		track_status->debug_slow_charging_force);
}

static int is_charging_full(struct oplus_monitor *monitor)
{
	if (monitor->wired_online && get_client_vote(monitor->wired_charging_disable_votable, CHG_FULL_VOTER) > 0)
		return true;
	if (monitor->wls_online && get_client_vote(monitor->wls_charging_disable_votable, CHG_FULL_VOTER) > 0)
		return true;
	return false;
}

static int
oplus_chg_track_cal_batt_full_time(struct oplus_monitor *monitor,
				   struct oplus_chg_track_status *track_status)
{
	int curr_time;

	if (!monitor || !track_status || !g_track_chip)
		return -1;

	if (!track_status->chg_fast_full_time &&
	    ((monitor->ffc_status != FFC_DEFAULT) ||
	     track_status->wls_prop_status == TRACK_WLS_FASTCHG_FULL ||
	     track_status->debug_fast_prop_status ==
		     TRACK_FASTCHG_STATUS_NORMAL)) {
		track_status->chg_report_full_time = 0;
		track_status->chg_normal_full_time = 0;
		curr_time = oplus_chg_track_get_local_time_s();
		track_status->chg_fast_full_time =
			curr_time - track_status->chg_start_time;
		track_status->chg_fast_full_time /= TRACK_TIME_1MIN_THD;
		if (!track_status->chg_fast_full_time)
			track_status->chg_fast_full_time =
				TRACK_TIME_1MIN_THD / TRACK_TIME_1MIN_THD;

		if (track_status->chg_average_speed ==
		    TRACK_PERIOD_CHG_AVERAGE_SPEED_INIT)
			track_status->chg_average_speed =
				TRACK_TIME_1MIN_THD *
				(monitor->batt_rm -
				 track_status->chg_start_rm) /
				(curr_time - track_status->chg_start_time);
		chg_info("curr_time:%d, start_time:%d, fast_full_time:%d"
			 "curr_rm:%d, chg_start_rm:%d, chg_average_speed:%d\n",
			 curr_time, track_status->chg_start_time,
			 track_status->chg_fast_full_time, monitor->batt_rm,
			 track_status->chg_start_rm,
			 track_status->chg_average_speed);
	}

	if (!track_status->chg_report_full_time &&
	    (monitor->batt_status == POWER_SUPPLY_STATUS_FULL ||
	     track_status->debug_normal_prop_status ==
		     POWER_SUPPLY_STATUS_FULL)) {
		oplus_chg_track_handle_batt_full_reason(monitor, track_status);
		oplus_chg_track_judge_speed_slow(monitor, g_track_chip);
		curr_time = oplus_chg_track_get_local_time_s();
		track_status->chg_report_full_time =
			curr_time - track_status->chg_start_time;
		track_status->chg_report_full_time /= TRACK_TIME_1MIN_THD;
		if (!track_status->chg_report_full_time)
			track_status->chg_report_full_time =
				TRACK_TIME_1MIN_THD / TRACK_TIME_1MIN_THD;
	}

	if (!track_status->chg_normal_full_time &&
	    (is_charging_full(monitor) ||
	     track_status->debug_normal_charging_state ==
		     POWER_SUPPLY_STATUS_FULL)) {
		curr_time = oplus_chg_track_get_local_time_s();
		if (track_status->aging_ffc_start_time)
			track_status->aging_ffc_to_full_time = curr_time - track_status->aging_ffc_start_time;
		if (track_status->ffc_end_time) {
			track_status->ffc_time =
				track_status->ffc_end_time - track_status->aging_ffc_start_time;
			track_status->cv_time = curr_time - track_status->ffc_end_time;
		}
		track_status->chg_normal_full_time =
			curr_time - track_status->chg_start_time;
		track_status->chg_normal_full_time /= TRACK_TIME_1MIN_THD;
		if (!track_status->chg_normal_full_time)
			track_status->chg_normal_full_time =
				TRACK_TIME_1MIN_THD / TRACK_TIME_1MIN_THD;
		if (track_status->chg_average_speed ==
		    TRACK_PERIOD_CHG_AVERAGE_SPEED_INIT)
			track_status->chg_average_speed =
				TRACK_TIME_1MIN_THD *
				(monitor->batt_rm -
				 track_status->chg_start_rm) /
				(curr_time - track_status->chg_start_time);
		chg_info("curr_time:%d, start_time:%d, normal_full_time:%d"
			 "curr_rm:%d, chg_start_rm:%d, chg_average_speed:%d\n",
			 curr_time, track_status->chg_start_time,
			 track_status->chg_normal_full_time, monitor->batt_rm,
			 track_status->chg_start_rm,
			 track_status->chg_average_speed);
	}

	return 0;
}

static int
oplus_chg_track_get_charger_type(struct oplus_monitor *monitor,
				 struct oplus_chg_track_status *track_status,
				 int type)
{
	struct oplus_chg_track *track_chip;

	if (!monitor || !track_status)
		return -EINVAL;
	track_chip = monitor->track;
	if (!track_chip)
		return -EINVAL;

	if (monitor->wls_online) {
		oplus_chg_track_handle_wls_type_info(track_status, type);
	} else if (monitor->wired_online) {
		oplus_chg_track_handle_wired_type_info(monitor, type);
		*track_status = track_chip->track_status;
	}
	return 0;
}

static int oplus_chg_track_obtain_power_info(char *power_info, int len)
{
	int index = 0;
	struct oplus_chg_track_status *track_status = temp_track_status;
	struct oplus_monitor *monitor;

	if (!power_info || !g_track_chip || !track_status)
		return -1;
	mutex_lock(&temp_track_status->track_status_lock);
	memset(track_status, 0, sizeof(struct oplus_chg_track_status));
	monitor = g_track_chip->monitor;
	oplus_chg_track_get_charger_type(monitor, track_status,
					 TRACK_CHG_GET_THTS_TIME_TYPE);

	index += scnprintf(&(power_info[index]), len - index, "$$power_mode@@%s",
			  track_status->power_info.power_mode);

	if (track_status->power_info.power_type == TRACK_CHG_TYPE_WIRE) {
		index += scnprintf(
			&(power_info[index]), len - index, "$$adapter_t@@%s",
			track_status->power_info.wired_info.adapter_type);
		if (track_status->power_info.wired_info.adapter_id)
			index += scnprintf(
				&(power_info[index]), len - index,
				"$$adapter_id@@0x%x",
				track_status->power_info.wired_info.adapter_id);
		index += scnprintf(&(power_info[index]), len - index,
				  "$$power@@%d",
				  track_status->power_info.wired_info.power);
		if (track_status->power_info.wired_info.emark_power > 0)
			index += scnprintf(&(power_info[index]), len - index, "$$emark_power@@%d",
				track_status->power_info.wired_info.emark_power);

		if (g_track_chip->track_status.wired_max_power <= 0)
			index += scnprintf(&(power_info[index]), len - index, "$$match_power@@%d", -1);
		else
			index += scnprintf(&(power_info[index]), len - index, "$$match_power@@%d",
					  (track_status->power_info.wired_info.power >= g_track_chip->track_status.wired_max_power));
	} else if (track_status->power_info.power_type == TRACK_CHG_TYPE_WIRELESS) {
		index += scnprintf(
			&(power_info[index]), len - index, "$$adapter_t@@%s",
			track_status->power_info.wls_info.adapter_type);
		if (strlen(track_status->power_info.wls_info.dock_type))
			index += scnprintf(
				&(power_info[index]), len - index,
				"$$dock_type@@%s",
				track_status->power_info.wls_info.dock_type);
		index += scnprintf(&(power_info[index]), len - index,
				  "$$power@@%d",
				  track_status->power_info.wls_info.power);
		if (g_track_chip->track_status.wls_max_power <= 0)
			index += scnprintf(&(power_info[index]), len - index, "$$match_power@@%d", -1);
		else
			index += scnprintf(&(power_info[index]), len - index, "$$match_power@@%d",
					  (track_status->power_info.wls_info.power >= g_track_chip->track_status.wls_max_power));
	}

	index += scnprintf(&(power_info[index]), len - index, "$$soc@@%d",
			  monitor->batt_soc);
	index += scnprintf(&(power_info[index]), len - index, "$$smooth_soc@@%d",
			  monitor->smooth_soc);
	index += scnprintf(&(power_info[index]), len - index, "$$uisoc@@%d",
			  monitor->ui_soc);
	index += scnprintf(&(power_info[index]), len - index, "$$batt_temp@@%d",
			  monitor->batt_temp);
	index += scnprintf(&(power_info[index]), len - index, "$$shell_temp@@%d",
			  monitor->shell_temp);
	index += scnprintf(&(power_info[index]), len - index, "$$batt_vol@@%d",
			  monitor->vbat_mv);
	index += scnprintf(&(power_info[index]), len - index, "$$batt_curr@@%d",
			  monitor->ibat_ma);
	mutex_unlock(&temp_track_status->track_status_lock);

	return 0;
}

static int oplus_chg_track_get_err_comm_info(struct oplus_chg_track *track, char *buf, int buf_size)
{
	struct oplus_chg_track_status *track_status;
	struct oplus_monitor *monitor;
	int index;

	monitor = track->monitor;
	track_status = &track->track_status;

	oplus_chg_track_get_charger_type(monitor, track_status,
					 TRACK_CHG_GET_THTS_TIME_TYPE);
	index = scnprintf(buf, buf_size, "$$power_mode@@%s",
			 track_status->power_info.power_mode);

	if (track_status->power_info.power_type == TRACK_CHG_TYPE_WIRE) {
		index += scnprintf(
			buf + index, buf_size - index, "$$adapter_t@@%s",
			track_status->power_info.wired_info.adapter_type);
		if (track_status->power_info.wired_info.adapter_id)
			index += scnprintf(
				buf + index, buf_size - index,
				"$$adapter_id@@0x%x",
				track_status->power_info.wired_info.adapter_id);
		index += scnprintf(buf + index, buf_size - index, "$$power@@%d",
				  track_status->power_info.wired_info.power);
		if (track_status->power_info.wired_info.emark_power > 0)
			index += scnprintf(buf + index, buf_size - index, "$$emark_power@@%d",
				track_status->power_info.wired_info.power);
		index += scnprintf(buf + index, buf_size - index, "$$vbus@@%d",
				  monitor->wired_vbus_mv);
		if (track_status->wired_max_power <= 0)
			index += scnprintf(buf + index, buf_size - index, "$$match_power@@%d", -1);
		else
			index += scnprintf(buf + index, buf_size - index, "$$match_power@@%d",
					  (track_status->power_info.wired_info.power >= track_status->wired_max_power));
	} else if (track_status->power_info.power_type == TRACK_CHG_TYPE_WIRELESS) {
		index += scnprintf(
			buf + index, buf_size - index, "$$adapter_t@@%s",
			track_status->power_info.wls_info.adapter_type);
		if (strlen(track_status->power_info.wls_info.dock_type))
			index += scnprintf(
				buf + index, buf_size - index,
				"$$dock_type@@%s",
				track_status->power_info.wls_info.dock_type);
		index += scnprintf(buf + index, buf_size - index, "$$power@@%d",
				  track_status->power_info.wls_info.power);
		index += scnprintf(buf + index, buf_size - index, "$$vbus@@%d",
				  monitor->wls_vout_mv);
		if (track_status->wls_max_power <= 0)
			index += scnprintf(buf + index, buf_size - index, "$$match_power@@%d", -1);
		else
			index += scnprintf(buf + index, buf_size - index, "$$match_power@@%d",
					  (track_status->power_info.wls_info.power >= track_status->wls_max_power));
	}

	return index;
}

static int oplus_chg_track_upload_ic_err_info(struct oplus_chg_track *track)
{
	union mms_msg_data data = { 0 };
	int index, copy_size;
	int name_index, msg_index;
	int err_type, sub_err_type;
	char *msg_buf, *track_buf;
	int rc;

	rc = oplus_mms_get_item_data(track->monitor->err_topic, ERR_ITEM_IC,
				     &data, false);
	if (rc < 0) {
		chg_err("get msg data error, rc=%d\n", rc);
		return rc;
	}
	track_buf = track->ic_err_msg_load_trigger.crux_info;

	if (NULL == data.strval) {
		chg_err("data.strval is NULL");
		return -EINVAL;
	}

	msg_buf = kzalloc(TOPIC_MSG_STR_BUF, GFP_KERNEL);
	if (msg_buf == NULL) {
		chg_err("alloc msg buf error");
		return -ENOMEM;
	}

	copy_size = strlen(data.strval) > TOPIC_MSG_STR_BUF ? TOPIC_MSG_STR_BUF : strlen(data.strval);
	memcpy(msg_buf, data.strval, copy_size);

	rc = oplus_mms_analysis_ic_err_msg(msg_buf, TOPIC_MSG_STR_BUF,
					   &name_index, &err_type,
					   &sub_err_type, &msg_index);
	if (rc < 0) {
		chg_err("error msg could not be analysis,  rc=%d\n", rc);
		kfree(msg_buf);
		return rc;
	}
	chg_info("err_type=%d, sub_err_type=%d\n", err_type, sub_err_type);

	index = scnprintf(track_buf, OPLUS_CHG_TRACK_CURX_INFO_LEN,
			 "$$device_id@@%s$$err_type@@%d", msg_buf + name_index,
			 sub_err_type);

	switch (err_type) {
	case OPLUS_IC_ERR_GPIO:
		track->ic_err_msg_load_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_GPIO_ABNORMAL;
		break;
	case OPLUS_IC_ERR_BATT_ID:
		track->ic_err_msg_load_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_BATT_ID_INFO;
		break;
	case OPLUS_IC_ERR_PLAT_PMIC:
		track->ic_err_msg_load_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_PLAT_PMIC_ABNORMAL;
		index += oplus_chg_track_get_err_comm_info(track,
			track_buf + index, OPLUS_CHG_TRIGGER_MSG_LEN - index);
		break;
	case OPLUS_IC_ERR_BUCK_BOOST:
		track->ic_err_msg_load_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_BOOST_BUCK_ERR;
		break;
	case OPLUS_IC_ERR_GAUGE:
		track->ic_err_msg_load_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_GAGUE_ABNORMAL;
		break;
	case OPLUS_IC_ERR_WLS_RX:
		track->ic_err_msg_load_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_WLS_ABNORMAL;
		memset(track->track_status.power_info.power_mode, 0,
			sizeof(track->track_status.power_info.power_mode));
		strcpy(track->track_status.power_info.power_mode, "wireless");
		index += oplus_chg_track_get_err_comm_info(track,
			track_buf + index, OPLUS_CHG_TRIGGER_MSG_LEN - index);
		break;
	case OPLUS_IC_ERR_CP:
		track->ic_err_msg_load_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_CP_ABNORMAL;
		break;
	case OPLUS_IC_ERR_CC_LOGIC:
		track->ic_err_msg_load_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_EXTERN_PMIC_ABNORMAL;
		break;
	case OPLUS_IC_ERR_PARALLEL_UNBALANCE:
		track->ic_err_msg_load_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_PARALLEL_UNBALANCE_ABNORMAL;
		break;
	case OPLUS_IC_ERR_MOS_ERROR:
		track->ic_err_msg_load_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_MOS_ERROR_ABNORMAL;
		break;
	case OPLUS_IC_ERR_UFCS:
		track->ic_err_msg_load_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_UFCS_IC_ABNORMAL;
		index += oplus_chg_track_obtain_power_info(track_buf + index,
			OPLUS_CHG_TRIGGER_MSG_LEN - index);
		break;
	case OPLUS_IC_ERR_GAN_MOS_ERROR:
		track->ic_err_msg_load_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_DCHG_ABNORMAL;
		break;
	case OPLUS_IC_ERR_I2C:
		track->ic_err_msg_load_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_I2C_ABNORMAL;
		break;
	case OPLUS_IC_ERR_NTC:
		track->ic_err_msg_load_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_NTC_ABNORMAL;
		break;
	case OPLUS_IC_ERR_BATT_BAL:
		track->ic_err_msg_load_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_BAL_IC_ABNORMAL;
		break;
	case OPLUS_IC_ERR_FPGA:
		track->ic_err_msg_load_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_FPGA_RST_ABNORMAL;
		break;
	case OPLUS_IC_ERR_BURN:
		track->ic_err_msg_load_trigger.flag_reason =
			TRACK_NOTIFY_FLAG_IC_BURN;
		break;
	case OPLUS_IC_ERR_UNKNOWN:
	default:
		chg_err("unsupported error type(%d)\n", err_type);
		kfree(msg_buf);
		return -EINVAL;
	}

	index += scnprintf(track_buf + index, OPLUS_CHG_TRIGGER_MSG_LEN - index,
			  "$$ic_msg@@%s", msg_buf + msg_index);

	schedule_delayed_work(&track->ic_err_msg_trigger_work, 0);
	chg_info("%s\n", track_buf);
	kfree(msg_buf);

	return 0;
}

static int oplus_chg_track_upload_usbtemp_info(struct oplus_chg_track *track)
{
	union mms_msg_data data = { 0 };
	int index;
	int rc;
	int charge_time;

	rc = oplus_mms_get_item_data(track->monitor->err_topic,
				     ERR_ITEM_USBTEMP, &data, false);
	if (rc < 0) {
		chg_err("get msg data error, rc=%d\n", rc);
		return rc;
	}

	track->slowchg_info.usbtemp_once = true;
	index = scnprintf(track->usbtemp_load_trigger.crux_info,
			 OPLUS_CHG_TRACK_CURX_INFO_LEN, "%s", data.strval);
	oplus_chg_track_obtain_power_info(
		&track->usbtemp_load_trigger.crux_info[index],
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index);

	charge_time = oplus_chg_track_get_local_time_s() - track->track_status.chg_start_time;
	chg_debug("charge_time:%ds\n", charge_time);
	if (charge_time < TRACK_TIME_2MIN_THD && track->track_cfg.track_ver >= TRACK_VER_4_0)
		schedule_delayed_work(&track->usbtemp_trigger_no_charging_work, 0);

	schedule_delayed_work(&track->usbtemp_load_trigger_work, 0);
	chg_info("%s\n", track->usbtemp_load_trigger.crux_info);

	return 0;
}

static int oplus_chg_track_upload_usb_lpd_info(struct oplus_chg_track *track)
{
	union mms_msg_data data = { 0 };
	int index;
	int rc;


	rc = oplus_mms_get_item_data(track->monitor->err_topic,
				     ERR_ITEM_LPD, &data, false);
	if (rc < 0) {
		chg_err("get msg data error, rc=%d\n", rc);
		return rc;
	}
	index = scnprintf(track->usb_lpd_load_trigger.crux_info,
			 OPLUS_CHG_TRACK_CURX_INFO_LEN, "%s", data.strval);

	chg_info("%s\n", track->usb_lpd_load_trigger.crux_info);

	return 0;
}

static int oplus_chg_track_upload_uisoc_drop_err_info(struct oplus_chg_track *track)
{
	union mms_msg_data data = { 0 };
	int rc;

	rc = oplus_mms_get_item_data(track->monitor->err_topic,
				     COMM_ITEM_UISOC_DROP_ERROR, &data, false);
	if (rc < 0) {
		chg_err("get msg data error, rc=%d\n", rc);
		return rc;
	}
	scnprintf(track->uisoc_drop_err_trigger.crux_info,
		 OPLUS_CHG_TRACK_CURX_INFO_LEN, "%s", data.strval);

	schedule_delayed_work(&track->uisoc_drop_err_trigger_work, 0);
	chg_info("%s\n", track->uisoc_drop_err_trigger.crux_info);

	return 0;
}

static int
oplus_chg_track_upload_vbatt_too_low_info(struct oplus_chg_track *track)
{
	union mms_msg_data data = { 0 };
	int rc;

	rc = oplus_mms_get_item_data(track->monitor->err_topic,
				     ERR_ITEM_VBAT_TOO_LOW, &data, false);
	if (rc < 0) {
		chg_err("get msg data error, rc=%d\n", rc);
		return rc;
	}
	scnprintf(track->vbatt_too_low_load_trigger.crux_info,
		 OPLUS_CHG_TRACK_CURX_INFO_LEN, "%s", data.strval);

	schedule_delayed_work(&track->vbatt_too_low_load_trigger_work, 0);
	chg_info("%s\n", track->vbatt_too_low_load_trigger.crux_info);

	return 0;
}

static int
oplus_chg_track_upload_vbatt_diff_over_info(struct oplus_chg_track *track)
{
	struct oplus_monitor *monitor = track->monitor;

	(void)scnprintf(track->vbatt_diff_over_load_trigger.crux_info,
		       OPLUS_CHG_TRACK_CURX_INFO_LEN,
		       "$$soc@@%d$$smooth_soc@@%d$$uisoc@@%d$$vbatt_max@@%d$$vbatt_min@@%d"
		       "$$vbatt_diff@@%d$$batt_rm@@%d$$batt_fcc@@%d$$batt_cc@@%d"
			"$$charger_exist@@%d$$vdiff_type@@%s",
		       monitor->batt_soc, monitor->smooth_soc, monitor->ui_soc,
		       monitor->vbat_mv, monitor->vbat_min_mv,
		       monitor->vbat_mv - monitor->vbat_min_mv,
		       monitor->batt_rm, monitor->batt_fcc, monitor->batt_cc,
		       (monitor->wired_online || monitor->wls_online),
		       (monitor->batt_status == POWER_SUPPLY_STATUS_FULL) ?
			       "full" : "shutdown");

	schedule_delayed_work(&track->vbatt_diff_over_load_trigger_work, 0);
	chg_info("%s\n", track->vbatt_diff_over_load_trigger.crux_info);

	return 0;
}

int oplus_chg_track_charge_full(struct oplus_monitor *monitor)
{
	struct oplus_chg_track *chip;

	if (monitor == NULL) {
		chg_err("monitor is NULL\n");
		return -ENODEV;
	}

	chip = monitor->track;
	if (monitor->batt_status == POWER_SUPPLY_STATUS_FULL)
		oplus_chg_track_upload_vbatt_diff_over_info(chip);

	return 0;
}

static int
oplus_chg_track_upload_uisoc_keep_1_t_info(struct oplus_chg_track *chip)
{
	struct oplus_monitor *monitor = chip->monitor;
	int index = 0;
	int uisoc_1_end_time;

	uisoc_1_end_time = oplus_chg_track_get_local_time_s();

	chg_info("uisoc_1_end_time:%d, uisoc_1_start_time:%d\n",
		 uisoc_1_end_time, chip->uisoc_1_start_time);
	memset(chip->uisoc_keep_1_t_load_trigger.crux_info, 0,
	       sizeof(chip->uisoc_keep_1_t_load_trigger.crux_info));
	index += scnprintf(&(chip->uisoc_keep_1_t_load_trigger.crux_info[index]),
			  OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$uisoc_keep_1_t@@%d",
			  uisoc_1_end_time - chip->uisoc_1_start_time);

	index += scnprintf(
		&(chip->uisoc_keep_1_t_load_trigger.crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
		"$$pre_vbatt_max@@%d$$pre_vbatt_min@@%d$$curr_vbatt_max@@%d"
		"$$curr_vbatt_min@@%d$$soc@@%d$$smooth_soc@@%d$$uisoc@@%d$$start_batt_rm@@%d"
		"$$curr_batt_rm@@%d$$batt_curr@@%d",
		chip->uisoc_1_start_vbatt_max, chip->uisoc_1_start_vbatt_min,
		monitor->vbat_mv, monitor->vbat_min_mv, monitor->batt_soc,
		monitor->smooth_soc, monitor->ui_soc, chip->uisoc_1_start_batt_rm,
		monitor->batt_rm, monitor->ibat_ma);

	schedule_delayed_work(&chip->uisoc_keep_1_t_load_trigger_work, 0);
	chg_info("%s\n", chip->uisoc_keep_1_t_load_trigger.crux_info);
	msleep(200);

	return 0;
}

static int
oplus_chg_track_upload_dual_chan_err_info(struct oplus_chg_track *track)
{
	union mms_msg_data data = { 0 };
	int rc;

	rc = oplus_mms_get_item_data(track->monitor->err_topic,
				     ERR_ITEM_DUAL_CHAN, &data, false);
	if (rc < 0) {
		chg_err("get msg data error, rc=%d\n", rc);
		return rc;
	}
	scnprintf(track->dual_chan_err_load_trigger.crux_info,
		 OPLUS_CHG_TRACK_CURX_INFO_LEN, "%s", data.strval);

	schedule_delayed_work(&track->dual_chan_err_load_trigger_work, 0);
	chg_info("%s\n", track->dual_chan_err_load_trigger.crux_info);

	return 0;
}

int oplus_chg_track_set_uisoc_1_start(struct oplus_monitor *monitor)
{
	struct oplus_chg_track *chip;

	if (monitor == NULL) {
		chg_err("monitor is NULL\n");
		return -ENODEV;
	}

	chip = monitor->track;
	chip->uisoc_1_start_time = oplus_chg_track_get_local_time_s();
	chip->uisoc_1_start_batt_rm = monitor->batt_rm;
	chip->uisoc_1_start_vbatt_max = monitor->vbat_mv;
	chip->uisoc_1_start_vbatt_min = monitor->vbat_min_mv;

	return 0;
}

#define TRACK_UPLOAD_COUNT_MAX 1000
#define TRACK_LOCAL_T_NS_TO_S_THD 1000000000
#define TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD (24 * 3600)
static int oplus_chg_track_dischg_profile(char *deep_msg)
{
	struct oplus_mms *err_topic;
	struct mms_msg *msg;
	int rc;
	static int upload_count = 0;
	static int pre_upload_time = 0;
	int curr_time;

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
		MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_DEEP_DISCHG_PROFILE, deep_msg);
	if (msg == NULL) {
		chg_err("alloc profile error msg error\n");
		return -ENOMEM;
	}

	rc = oplus_mms_publish_msg(err_topic, msg);
	if (rc < 0) {
		chg_err("publish profile error msg error, rc=%d\n", rc);
		kfree(msg);
	}
	upload_count++;

	return rc;
}

#define REG_INFO_LEN 640
void oplus_chg_track_upload_dischg_profile(struct oplus_monitor *monitor)
{
	char profile_info[REG_INFO_LEN] = { 0 };
	int index;
	struct oplus_monitor *chip;

	if (monitor == NULL) {
		chg_err("monitor is NULL\n");
		return;
	}

	chip = monitor;
	if (!chip  || !chip->deep_support || chip->dischg_profile.upload)
		return;

	if (chip->dischg_profile.index >= DEEP_DISCHG_AVG_PROFILE_SIZE) {
		index = scnprintf(profile_info, REG_INFO_LEN, "$$voltage_10per@@%d$$voltage_5per@@%d$$voltage_avg@@%d"
			"$$vmin@@%d$$vmax@@%d$$tbat_max@@%d$$tbat_min@@%d$$tbat_avg@@%d$$tbat_now@@%d$$current_max@@%d"
			"$$current_min@@%d$$current_avg@@%d$$tsub_max@@%d$$tsub_min@@%d$$tsub_avg@@%d$$tsub_now@@%d"
			"$$tshell_max@@%d$$tshell_min@@%d$$tshell_avg@@%d$$tshell_now@@%d$$gaugesoc_final@@%d"
			"$$uisoc_final@@%d$$term_voltage@@%d$$term_dod1@@%d$$term_dod1@@%d$$dischg_counts@@%d"
			"$$cc@@%d$$ratio@@%d$$time_lowbat@@%d$$vmin0@@%d$$vmax0@@%d$$tsoc0@@%d$$sicc@@%d",
			chip->dischg_profile.vbat_10, chip->dischg_profile.vbat_5, chip->dischg_profile.vbat_avg,
			chip->dischg_profile.vbat_min, chip->dischg_profile.vbat_max, chip->dischg_profile.tbat_max,
			chip->dischg_profile.tbat_min, chip->dischg_profile.tbat_avg, chip->dischg_profile.tbat_now,
			chip->dischg_profile.ibat_max, chip->dischg_profile.ibat_min, chip->dischg_profile.ibat_avg,
			chip->dischg_profile.tsub_max, chip->dischg_profile.tsub_min, chip->dischg_profile.tsub_avg,
			chip->dischg_profile.tsub_now, chip->dischg_profile.tshell_max,
			chip->dischg_profile.tshell_min, chip->dischg_profile.tshell_avg, chip->dischg_profile.tshell_now,
			chip->dischg_profile.soc, chip->dischg_profile.ui_soc, chip->dischg_profile.vbat_term,
			chip->dischg_profile.dod1, chip->dischg_profile.dod2, chip->dischg_profile.counts,
			chip->dischg_profile.cc, chip->dischg_profile.ratio, chip->dischg_profile.time,
			chip->dischg_profile.vmin0, chip->dischg_profile.vmax0, chip->dischg_profile.tsoc0, chip->dischg_profile.sicc);

		oplus_chg_track_dischg_profile(profile_info);
		chip->dischg_profile.upload = true;
	}
}

#define INVALID_CC_VALUE 5000
#define DISCHG_PROFILE_RECORD_UI 10
#define DISCHG_PROFILE_SI_UI 20
#define FULL_DOD_TI 16384
void oplus_chg_track_update_dischg_profile(struct oplus_monitor *monitor)
{
	struct oplus_monitor *chip;
	union mms_msg_data data = { 0 };
	struct dischg_avg profile_sum = { 0 };
	unsigned long update_delay = msecs_to_jiffies(5000);
	int batt_temp, subboard_temp, vbat_uv, dischg_counts;
	int qmax_1 = 0, qmax_2 = 0, dod0_1 = 0, dod0_2 = 0, passed_q = 0;
	int i, rc;
	bool charging;

	if (monitor == NULL) {
		chg_err("monitor is NULL\n");
		return;
	}

	chip = monitor;

	if (chip->ui_soc > DISCHG_PROFILE_RECORD_UI) {
		schedule_delayed_work(&chip->dischg_profile_update_work, update_delay);
		return;
	}
	charging = chip->wired_online || chip->wls_online;
	if (charging)
		return;

	rc = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_VBAT_UV_THR, &data, true);
	if (rc < 0) {
		chg_err("can't get COMM_ITEM_VBAT_UV_THR data, rc=%d\n", rc);
		vbat_uv = 0;
	} else {
		vbat_uv = data.intval;
	}
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_TEMP, &data, true);
	if (rc < 0) {
		chg_err("can't get GAUGE_ITEM_TEMP data, rc=%d\n", rc);
		subboard_temp = 0;
	} else {
		subboard_temp = data.intval;
	}
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_REAL_TEMP, &data, true);
	if (rc < 0) {
		chg_err("can't get GAUGE_ITEM_REAL_TEMP data, rc=%d\n", rc);
		batt_temp = 0;
	} else {
		batt_temp = data.intval;
	}
	dischg_counts = oplus_gauge_show_deep_dischg_count(chip->gauge_topic);

	for (i = 0; i < DEEP_DISCHG_AVG_PROFILE_SIZE - 1; i++) {
		profile_sum.ibat += chip->dischg_profile.profile_avg[i].ibat;
		profile_sum.tbat += chip->dischg_profile.profile_avg[i].tbat;
		profile_sum.vbat += chip->dischg_profile.profile_avg[i].vbat;
		profile_sum.tsub += chip->dischg_profile.profile_avg[i].tsub;
		profile_sum.tshell += chip->dischg_profile.profile_avg[i].tshell;

		chip->dischg_profile.profile_avg[i].ibat = chip->dischg_profile.profile_avg[i + 1].ibat;
		chip->dischg_profile.profile_avg[i].tbat = chip->dischg_profile.profile_avg[i + 1].tbat;
		chip->dischg_profile.profile_avg[i].vbat = chip->dischg_profile.profile_avg[i + 1].vbat;
		chip->dischg_profile.profile_avg[i].tsub = chip->dischg_profile.profile_avg[i + 1].tsub;
		chip->dischg_profile.profile_avg[i].tshell = chip->dischg_profile.profile_avg[i + 1].tshell;

	}
	chip->dischg_profile.profile_avg[i].ibat = chip->ibat_ma;
	chip->dischg_profile.profile_avg[i].vbat = chip->vbat_min_mv;
	chip->dischg_profile.profile_avg[i].tbat = batt_temp;
	chip->dischg_profile.profile_avg[i].tsub = subboard_temp;
	chip->dischg_profile.profile_avg[i].tshell = chip->shell_temp;

	profile_sum.ibat += chip->ibat_ma;
	profile_sum.tbat += batt_temp;
	profile_sum.vbat += chip->vbat_min_mv;
	profile_sum.tsub += subboard_temp;
	profile_sum.tshell += chip->shell_temp;

	chip->dischg_profile.ibat_avg = profile_sum.ibat / DEEP_DISCHG_AVG_PROFILE_SIZE;
	chip->dischg_profile.tbat_avg = profile_sum.tbat / DEEP_DISCHG_AVG_PROFILE_SIZE;
	chip->dischg_profile.vbat_avg = profile_sum.vbat / DEEP_DISCHG_AVG_PROFILE_SIZE;;
	chip->dischg_profile.tsub_avg = profile_sum.tsub / DEEP_DISCHG_AVG_PROFILE_SIZE;
	chip->dischg_profile.tshell_avg = profile_sum.tshell / DEEP_DISCHG_AVG_PROFILE_SIZE;;


	if (chip->ui_soc == 5 && chip->vbat_min_mv > chip->dischg_profile.vbat_5)
		chip->dischg_profile.vbat_5 = chip->vbat_min_mv;
	else if (chip->ui_soc == 10 && chip->vbat_min_mv > chip->dischg_profile.vbat_10)
		chip->dischg_profile.vbat_10 = chip->vbat_min_mv;

	if (batt_temp > chip->dischg_profile.tbat_max)
		chip->dischg_profile.tbat_max = batt_temp;
	if (batt_temp < chip->dischg_profile.tbat_min)
		chip->dischg_profile.tbat_min = batt_temp;
	if (chip->ibat_ma > chip->dischg_profile.ibat_max)
		chip->dischg_profile.ibat_max = chip->ibat_ma;
	if (chip->ibat_ma < chip->dischg_profile.ibat_min)
		chip->dischg_profile.ibat_min = chip->ibat_ma;

	if (subboard_temp > chip->dischg_profile.tsub_max)
		chip->dischg_profile.tsub_max = subboard_temp;
	if (subboard_temp < chip->dischg_profile.tsub_min)
		chip->dischg_profile.tsub_min = subboard_temp;

	if (chip->shell_temp > chip->dischg_profile.tshell_max)
		chip->dischg_profile.tshell_max = chip->shell_temp;
	if (chip->shell_temp < chip->dischg_profile.tshell_min)
		chip->dischg_profile.tshell_min = chip->shell_temp;

	chip->dischg_profile.tbat_now = batt_temp;
	chip->dischg_profile.tsub_now = subboard_temp;
	chip->dischg_profile.tshell_now = chip->shell_temp;

	if (chip->dischg_profile.soc && !chip->batt_soc) {
		chip->dischg_profile.vmin0 = chip->vbat_min_mv;
		chip->dischg_profile.vmax0 = chip->vbat_mv;
		chip->dischg_profile.tsoc0 = batt_temp;
	}
	chip->dischg_profile.sicc = (DISCHG_PROFILE_SI_UI - chip->dischg_profile.ui_soc) * 100 / DISCHG_PROFILE_SI_UI;

	chip->dischg_profile.vbat_min = chip->vbat_min_mv;
	chip->dischg_profile.vbat_max = chip->vbat_mv;
	chip->dischg_profile.ui_soc = chip->ui_soc;
	chip->dischg_profile.soc = chip->batt_soc;
	chip->dischg_profile.cc = chip->batt_cc;
	chip->dischg_profile.counts = dischg_counts;

	chip->dischg_profile.vbat_term = vbat_uv;
	if (chip->dischg_profile.cc <= 0 || chip->dischg_profile.cc >= INVALID_CC_VALUE) {
		if (!chip->dischg_profile.counts)
			chip->dischg_profile.ratio = 100;
		else
			chip->dischg_profile.ratio = chip->dischg_profile.counts * 10;

	} else {
		chip->dischg_profile.ratio = chip->dischg_profile.counts * 10 / chip->dischg_profile.cc;
	}
	chip->dischg_profile.time = jiffies_to_msecs(jiffies - chip->dischg_profile.init_jiffies);

	oplus_gauge_get_qmax(chip->gauge_topic, 0, &qmax_1);
	oplus_gauge_get_qmax(chip->gauge_topic, 1, &qmax_2);
	oplus_gauge_get_dod0(chip->gauge_topic, 0, &dod0_1);
	oplus_gauge_get_dod0(chip->gauge_topic, 1, &dod0_2);
	oplus_gauge_get_dod0_passed_q(chip->gauge_topic, 0, &passed_q);
	/* TODO: need check for huaxin */
	chip->dischg_profile.dod1 = (qmax_1 == 0) ? 0 : (10000 - passed_q * 10000 / qmax_1 - dod0_1 * 10000 / FULL_DOD_TI);
	chip->dischg_profile.dod2 = (qmax_2 == 0) ? 0 : (10000 - passed_q * 10000 / qmax_2 - dod0_2 * 10000 / FULL_DOD_TI);

	chip->dischg_profile.index++;
	schedule_delayed_work(&chip->dischg_profile_update_work, update_delay);
}

#define OPLUS_PRECHG_BATT_R_LEN 	64
#define OPLUS_PRECHG_R_TAG 		"prechg_r:"
static void oplus_chg_track_prechg_r_update_work(struct work_struct *work)
{
	char *str;
	int mutual_rc;
	oplus_chg_track_trigger *trigger;
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip =
		container_of(dwork, struct oplus_chg_track, prechg_r_update_work);

	mutual_rc = oplus_chg_set_mutual_cmd(CMD_PRECHG_R_OBTAIN, 0, NULL);
	if (mutual_rc != CMD_ACK_OK)
		return;

	str = strstr(chip->prechg_r_data, OPLUS_PRECHG_R_TAG);
	if (str == NULL) {
		chg_err("not found %s tag\n", OPLUS_PRECHG_R_TAG);
		return;
	}
	str += strlen(OPLUS_PRECHG_R_TAG);

	trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!trigger) {
		chg_err("trigger memery alloc fail\n");
		return;
	}

	trigger->type_reason = TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	trigger->flag_reason = TRACK_NOTIFY_FLAG_PRECHG_BATT_R_INFO;
	scnprintf(trigger->crux_info, OPLUS_CHG_TRACK_CURX_INFO_LEN, "$$r_data@@%s", str);
	oplus_chg_track_upload_trigger_data(trigger);
	kfree(trigger);
}

void oplus_chg_track_update_prechg_r_data(struct oplus_monitor *monitor)
{
	if (monitor->track == NULL)
		return;

	schedule_delayed_work(&monitor->track->prechg_r_update_work, msecs_to_jiffies(3000));
}

static int oplus_chg_track_prechg_r_obtain_mutual_notifier_call(
	struct notifier_block *nb, unsigned long param, void *v)
{
	struct oplus_chg_track *chip;
	struct oplus_chg_mutual_notifier *notifier;

	notifier = container_of(nb, struct oplus_chg_mutual_notifier, nb);
	chip = container_of(notifier, struct oplus_chg_track, prechg_r_mutual);

	if (mutual_info_to_cmd(param) != CMD_PRECHG_R_OBTAIN) {
		chg_err("cmd is not matching, should return\n");
		return NOTIFY_OK;
	}

	if (mutual_info_to_data_size(param) != sizeof(chip->prechg_r_data)) {
		chg_err("data_len is not ok, datas is invalid\n");
		return NOTIFY_DONE;
	}

	if (v)
		memcpy(chip->prechg_r_data, v, sizeof(chip->prechg_r_data));

	chip->prechg_r_data[PRE_CHG_BATT_R_LEN - 1] = '\0';
	chg_info("prechg_r_data:%s\n", chip->prechg_r_data);

	return NOTIFY_OK;
}

static int oplus_chg_track_prechg_r_mutual_notify_reg(struct oplus_chg_track *chip)
{
	int rc = 0;

	chip->prechg_r_mutual.name = "prechg_r_obtain";
	chip->prechg_r_mutual.cmd = CMD_PRECHG_R_OBTAIN;
	chip->prechg_r_mutual.nb.notifier_call = oplus_chg_track_prechg_r_obtain_mutual_notifier_call;
	rc = oplus_chg_reg_mutual_notifier(&chip->prechg_r_mutual);
	if (rc < 0) {
		chg_err("register prechg r obtain mutual event notifier error, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

void oplus_chg_track_init_dischg_profile(struct oplus_monitor *monitor)
{
	struct oplus_monitor *chip;
	union mms_msg_data data = { 0 };
	int batt_temp, subboard_temp, vbat_uv, dischg_counts;
	int qmax_1 = 0, qmax_2 = 0, dod0_1 = 0, dod0_2 = 0, passed_q = 0;
	int i, rc;

	if (monitor == NULL) {
		chg_err("monitor is NULL\n");
		return;
	}

	chip = monitor;
	rc = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_VBAT_UV_THR, &data, true);
	if (rc < 0) {
		chg_err("can't get COMM_ITEM_VBAT_UV_THR data, rc=%d\n", rc);
		vbat_uv = 0;
	} else {
		vbat_uv = data.intval;
	}
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_TEMP, &data, true);
	if (rc < 0) {
		chg_err("can't get GAUGE_ITEM_TEMP data, rc=%d\n", rc);
		subboard_temp = 0;
	} else {
		subboard_temp = data.intval;
	}
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_REAL_TEMP, &data, true);
	if (rc < 0) {
		chg_err("can't get GAUGE_ITEM_REAL_TEMP data, rc=%d\n", rc);
		batt_temp = 0;
	} else {
		batt_temp = data.intval;
	}
	dischg_counts = oplus_gauge_show_deep_dischg_count(chip->gauge_topic);

	for (i = 0; i < DEEP_DISCHG_AVG_PROFILE_SIZE; i++) {
		chip->dischg_profile.profile_avg[i].ibat = chip->ibat_ma;
		chip->dischg_profile.profile_avg[i].vbat = chip->vbat_min_mv;
		chip->dischg_profile.profile_avg[i].tbat = batt_temp;
		chip->dischg_profile.profile_avg[i].tsub = subboard_temp;
		chip->dischg_profile.profile_avg[i].tshell = chip->shell_temp;
	}

	chip->dischg_profile.ibat_avg = chip->ibat_ma;
	chip->dischg_profile.tbat_avg = batt_temp;
	chip->dischg_profile.vbat_avg = chip->vbat_min_mv;
	chip->dischg_profile.tsub_avg = subboard_temp;
	chip->dischg_profile.tshell_avg = chip->shell_temp;

	chip->dischg_profile.vbat_10 = 0;
	chip->dischg_profile.vbat_5 = 0;

	chip->dischg_profile.tbat_max = batt_temp;
	chip->dischg_profile.tbat_min = batt_temp;

	chip->dischg_profile.ibat_max = chip->ibat_ma;
	chip->dischg_profile.ibat_min = chip->ibat_ma;

	chip->dischg_profile.tsub_max = subboard_temp;
	chip->dischg_profile.tsub_min = subboard_temp;

	chip->dischg_profile.tshell_max = chip->shell_temp;
	chip->dischg_profile.tshell_min = chip->shell_temp;

	chip->dischg_profile.tbat_now = batt_temp;
	chip->dischg_profile.tsub_now = subboard_temp;
	chip->dischg_profile.tshell_now = chip->shell_temp;

	chip->dischg_profile.vmax0 = INT_MAX;
	chip->dischg_profile.vmin0 = INT_MAX;
	chip->dischg_profile.tsoc0 = INT_MAX;
	chip->dischg_profile.sicc = 0;

	chip->dischg_profile.vbat_min = chip->vbat_min_mv;
	chip->dischg_profile.vbat_max = chip->vbat_mv;
	chip->dischg_profile.ui_soc = chip->ui_soc;
	chip->dischg_profile.soc = chip->batt_soc;
	chip->dischg_profile.cc = chip->batt_cc;
	chip->dischg_profile.counts = dischg_counts;
	chip->dischg_profile.vbat_term = vbat_uv;
	if (!chip->dischg_profile.cc)
		chip->dischg_profile.ratio = chip->dischg_profile.counts * 100;
	else
		chip->dischg_profile.ratio = chip->dischg_profile.counts * 100 / chip->dischg_profile.cc;

	chip->dischg_profile.init_jiffies = jiffies;

	oplus_gauge_get_qmax(chip->gauge_topic, 0, &qmax_1);
	oplus_gauge_get_qmax(chip->gauge_topic, 1, &qmax_2);
	oplus_gauge_get_dod0(chip->gauge_topic, 0, &dod0_1);
	oplus_gauge_get_dod0(chip->gauge_topic, 1, &dod0_2);
	oplus_gauge_get_dod0_passed_q(chip->gauge_topic, 0, &passed_q);
	/* TODO: need check for huaxin */
	chip->dischg_profile.dod1 = (qmax_1 == 0) ? 0 : (10000 - passed_q * 10000 / qmax_1 - dod0_1 * 10000 / FULL_DOD_TI);
	chip->dischg_profile.dod2 = (qmax_2 == 0) ? 0 : (10000 - passed_q * 10000 / qmax_2 - dod0_2 * 10000 / FULL_DOD_TI);

	chip->dischg_profile.index = 0;
	chip->dischg_profile.upload = 0;
}


static int oplus_chg_track_upload_mmi_chg_info(struct oplus_chg_track *chip)
{
	int index = 0;
	union mms_msg_data data = { 0 };
	int rc = 0;

	if (!chip)
		return -EINVAL;

	mutex_lock(&chip->mmi_chg_info_lock);
	if (chip->mmi_chg_info_trigger)
		kfree(chip->mmi_chg_info_trigger);

	chip->mmi_chg_info_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->mmi_chg_info_trigger) {
		pr_err("mmi_chg_info_trigger memery alloc fail\n");
		mutex_unlock(&chip->mmi_chg_info_lock);
		return -ENOMEM;
	}

	chip->mmi_chg_info_trigger->type_reason = TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	chip->mmi_chg_info_trigger->flag_reason = TRACK_NOTIFY_FLAG_MMI_CHG_INFO;

	rc = oplus_mms_get_item_data(chip->monitor->err_topic, ERR_ITEM_MMI_CHG, &data, false);
	if (rc < 0) {
		chg_err("get msg data error, rc=%d\n", rc);
		kfree(chip->mmi_chg_info_trigger);
		chip->mmi_chg_info_trigger = NULL;
		mutex_unlock(&chip->mmi_chg_info_lock);
		return rc;
	}

	index += scnprintf(&(chip->mmi_chg_info_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "%s",
			  data.strval);
	oplus_chg_track_obtain_power_info(&(chip->mmi_chg_info_trigger->crux_info[index]),
					  OPLUS_CHG_TRACK_CURX_INFO_LEN - index);

	schedule_delayed_work(&chip->mmi_chg_info_trigger_work, 0);
	chg_info("success\n");
	return 0;
}

static int oplus_chg_track_upload_slow_chg_info(struct oplus_chg_track *chip)
{
	int index = 0;
	union mms_msg_data data = { 0 };
	int rc = 0;

	if (!chip)
		return -EINVAL;

	mutex_lock(&chip->slow_chg_info_lock);
	if (chip->slow_chg_info_trigger)
		kfree(chip->slow_chg_info_trigger);

	chip->slow_chg_info_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->slow_chg_info_trigger) {
		pr_err("slow_chg_info_trigger memery alloc fail\n");
		mutex_unlock(&chip->slow_chg_info_lock);
		return -ENOMEM;
	}

	chip->slow_chg_info_trigger->type_reason = TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	chip->slow_chg_info_trigger->flag_reason = TRACK_NOTIFY_FLAG_SLOW_CHG_INFO;

	rc = oplus_mms_get_item_data(chip->monitor->err_topic, ERR_ITEM_SLOW_CHG, &data, false);
	if (rc < 0) {
		chg_err("get msg data error, rc=%d\n", rc);
		kfree(chip->slow_chg_info_trigger);
		chip->slow_chg_info_trigger = NULL;
		mutex_unlock(&chip->slow_chg_info_lock);
		return rc;
	}

	index += scnprintf(&(chip->slow_chg_info_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "%s",
			  data.strval);
	oplus_chg_track_obtain_power_info(&(chip->slow_chg_info_trigger->crux_info[index]),
					  OPLUS_CHG_TRACK_CURX_INFO_LEN - index);

	schedule_delayed_work(&chip->slow_chg_info_trigger_work, 0);
	chg_info("success\n");
	return 0;
}

static int oplus_chg_track_upload_chg_cycle_info(struct oplus_chg_track *chip)
{
	int index = 0;
	union mms_msg_data data = { 0 };
	int rc = 0;

	if (!chip)
		return -EINVAL;

	mutex_lock(&chip->chg_cycle_info_lock);
	if (chip->chg_cycle_info_trigger)
		kfree(chip->chg_cycle_info_trigger);

	chip->chg_cycle_info_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->chg_cycle_info_trigger) {
		pr_err("chg_cycle_info_trigger memery alloc fail\n");
		mutex_unlock(&chip->chg_cycle_info_lock);
		return -ENOMEM;
	}

	chip->chg_cycle_info_trigger->type_reason = TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	chip->chg_cycle_info_trigger->flag_reason = TRACK_NOTIFY_FLAG_CHG_CYCLE_INFO;

	rc = oplus_mms_get_item_data(chip->monitor->err_topic, ERR_ITEM_CHG_CYCLE, &data, false);
	if (rc < 0) {
		chg_err("get msg data error, rc=%d\n", rc);
		kfree(chip->chg_cycle_info_trigger);
		chip->chg_cycle_info_trigger = NULL;
		mutex_unlock(&chip->chg_cycle_info_lock);
		return rc;
	}

	index += scnprintf(&(chip->chg_cycle_info_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "%s$$status@@%d", data.strval, chip->monitor->chg_cycle_status);
	oplus_chg_track_obtain_power_info(&(chip->chg_cycle_info_trigger->crux_info[index]),
					  OPLUS_CHG_TRACK_CURX_INFO_LEN - index);

	schedule_delayed_work(&chip->chg_cycle_info_trigger_work, 0);
	chg_info("success\n");
	return 0;
}

static int oplus_chg_track_upload_wls_info(struct oplus_chg_track *chip)
{
	int index = 0;
	union mms_msg_data data = { 0 };
	int rc = 0;

	if (!chip)
		return -EINVAL;

	mutex_lock(&chip->wls_info_lock);
	if (chip->wls_info_trigger)
		kfree(chip->wls_info_trigger);

	chip->wls_info_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->wls_info_trigger) {
		chg_err("wls_info_trigger memery alloc fail\n");
		mutex_unlock(&chip->wls_info_lock);
		return -ENOMEM;
	}

	chip->wls_info_trigger->type_reason = TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	chip->wls_info_trigger->flag_reason = TRACK_NOTIFY_FLAG_WLS_INFO;

	rc = oplus_mms_get_item_data(chip->monitor->err_topic, ERR_ITEM_WLS_INFO, &data, false);
	if (rc < 0) {
		chg_err("get msg data error, rc=%d\n", rc);
		kfree(chip->wls_info_trigger);
		chip->wls_info_trigger = NULL;
		mutex_unlock(&chip->wls_info_lock);
		return rc;
	}

	index += scnprintf(&(chip->wls_info_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "%s", data.strval);
	oplus_chg_track_obtain_power_info(&(chip->wls_info_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index);

	schedule_delayed_work(&chip->wls_info_trigger_work, 0);
	chg_info("success\n");
	return 0;
}

static int oplus_chg_track_upload_ufcs_info(struct oplus_chg_track *chip)
{
	int index = 0;
	union mms_msg_data data = { 0 };
	int rc = 0;

	if (!chip)
		return -EINVAL;

	oplus_mms_get_item_data(chip->monitor->ufcs_topic, UFCS_ITEM_ERR_TYPE, &data, true);
	chip->track_status.ufcs_err_type = data.intval;

	mutex_lock(&chip->ufcs_info_lock);
	if (chip->ufcs_info_trigger)
		kfree(chip->ufcs_info_trigger);

	chip->ufcs_info_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->ufcs_info_trigger) {
		chg_err("ufcs_info_trigger memery alloc fail\n");
		mutex_unlock(&chip->ufcs_info_lock);
		return -ENOMEM;
	}

	chip->ufcs_info_trigger->type_reason = TRACK_NOTIFY_TYPE_SOFTWARE_ABNORMAL;
	chip->ufcs_info_trigger->flag_reason = TRACK_NOTIFY_FLAG_UFCS_ABNORMAL;

	rc = oplus_mms_get_item_data(chip->monitor->err_topic, ERR_ITEM_UFCS, &data, false);
	if (rc < 0) {
		chg_err("get msg data error, rc=%d\n", rc);
		kfree(chip->ufcs_info_trigger);
		chip->ufcs_info_trigger = NULL;
		mutex_unlock(&chip->ufcs_info_lock);
		return rc;
	}

	index += scnprintf(&(chip->ufcs_info_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "%s", data.strval);
	oplus_chg_track_obtain_power_info(&(chip->ufcs_info_trigger->crux_info[index]),
					  OPLUS_CHG_TRACK_CURX_INFO_LEN - index);

	schedule_delayed_work(&chip->ufcs_info_trigger_work, 0);
	chg_info("success\n");
	return 0;
}

static int oplus_chg_track_upload_pps_info(struct oplus_chg_track *chip)
{
	int index = 0, temp_index = 0;
	union mms_msg_data data = { 0 };
	pps_msg_data pps_data = {0};
	int rc = 0;

	if (!chip)
		return -EINVAL;

	mutex_lock(&chip->pps_info_lock);
	if (chip->pps_info_trigger)
		kfree(chip->pps_info_trigger);

	chip->pps_info_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->pps_info_trigger) {
		chg_err("pps_info_trigger memery alloc fail\n");
		mutex_unlock(&chip->pps_info_lock);
		return -ENOMEM;
	}

	chip->pps_info_trigger->type_reason = TRACK_NOTIFY_TYPE_SOFTWARE_ABNORMAL;
	chip->pps_info_trigger->flag_reason = TRACK_NOTIFY_FLAG_PPS_ABNORMAL;

	rc = oplus_mms_get_item_data(chip->monitor->err_topic, ERR_ITEM_PPS, &data, false);
	if (rc < 0) {
		chg_err("get msg data error, rc=%d\n", rc);
		kfree(chip->pps_info_trigger);
		chip->pps_info_trigger = NULL;
		mutex_unlock(&chip->pps_info_lock);
		return rc;
	}

	index += scnprintf(&(chip->pps_info_trigger->crux_info[index]),
			   OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "%s", data.strval);
	index += scnprintf(&(chip->pps_info_trigger->crux_info[index]),
			   OPLUS_CHG_TRIGGER_MSG_LEN - index,
                           "$$cpa_power@@%d", chip->pps_info.cpa_power);
	index += scnprintf(&(chip->pps_info_trigger->crux_info[index]),
			   OPLUS_CHG_TRIGGER_MSG_LEN - index,
                           "$$adapter_power@@%d", chip->pps_info.adapter_power);

	index += scnprintf(&(chip->pps_info_trigger->crux_info[index]),
			   OPLUS_CHG_TRIGGER_MSG_LEN - index, "$$pps_adapter_info@@[");
	oplus_chg_track_handle_pps_pdo_info(chip, chip->pps_info_trigger->crux_info,
					    &index, OPLUS_CHG_TRIGGER_MSG_LEN);
	index += scnprintf(&(chip->pps_info_trigger->crux_info[index]),
			   OPLUS_CHG_TRIGGER_MSG_LEN - index, "]");

	temp_index = chip->pps_info.adapter_power_max_index;
	if (chip->pps_info.adapter_pdo != NULL)
		pps_data = (pps_msg_data)chip->pps_info.adapter_pdo[temp_index];
	if (!(chip->pps_info.adapter_pdo) && pps_data.pdo_type == USBPD_PDMSG_PDOTYPE_AUGMENTED) {
		index += scnprintf(&(chip->pps_info_trigger->crux_info[index]),
				 OPLUS_CHG_TRIGGER_MSG_LEN - index,
                                 "$$pps_adapter_max@@(%uW %umV %umA)",
				 PPS_PDO_VOL_MAX(pps_data.max_voltage100mv) / MILLIMETER *
				 PPS_PDO_CURR_MAX(pps_data.max_current50ma) / MILLIMETER,
				 PPS_PDO_VOL_MAX(pps_data.max_voltage100mv),
				 PPS_PDO_CURR_MAX(pps_data.max_current50ma));
	} else {
		index += scnprintf(&(chip->pps_info_trigger->crux_info[index]),
				   OPLUS_CHG_TRIGGER_MSG_LEN - index, "$$pps_adapter_max@@0");
	}

	oplus_chg_track_obtain_power_info(&(chip->pps_info_trigger->crux_info[index]),
					  OPLUS_CHG_TRACK_CURX_INFO_LEN - index);

	schedule_delayed_work(&chip->pps_info_trigger_work, 0);
	chg_info("success\n");
	return 0;
}

static int oplus_chg_track_upload_cycle_current_derating_info(struct oplus_chg_track *track)
{
	union mms_msg_data data = { 0 };
	int rc;

	if (!track || !track->monitor || !track->monitor->err_topic) {
		chg_err("invalid track/monitor/err_topic\n");
		return -EINVAL;
	}

	rc = oplus_mms_get_item_data(track->monitor->err_topic, ERR_ITEM_CYCLE_CURRENT_DERATING, &data, false);
	if (rc < 0) {
		chg_err("get msg data error, rc=%d\n", rc);
		return rc;
	}
	track->monitor->curr_derating_trig = true;
	scnprintf(track->cycle_current_derating_trigger.crux_info, OPLUS_CHG_TRACK_CURX_INFO_LEN, "%s", data.strval);

	return 0;
}

static int oplus_chg_track_upload_deep_dischg_info(struct oplus_chg_track *chip , u32 id)
{
	int index = 0;
	union mms_msg_data data = { 0 };
	int rc = 0;

	if (!chip ||
	    (id != ERR_ITEM_DEEP_DISCHG_INFO && id != ERR_ITEM_SUB_DEEP_DISCHG_INFO))
		return -EINVAL;

	mutex_lock(&chip->deep_dischg_info_lock);
	if (chip->deep_dischg_info_trigger)
		kfree(chip->deep_dischg_info_trigger);

	chip->deep_dischg_info_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->deep_dischg_info_trigger) {
		chg_err("deep_dischg_info_trigger memery alloc fail\n");
		mutex_unlock(&chip->deep_dischg_info_lock);
		return -ENOMEM;
	}

	chip->deep_dischg_info_trigger->type_reason = TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	chip->deep_dischg_info_trigger->flag_reason = TRACK_NOTIFY_FLAG_BYPASS_BOOST_INFO;

	rc = oplus_mms_get_item_data(chip->monitor->err_topic, id, &data, false);
	if (rc < 0) {
		chg_err("get msg data error, rc=%d\n", rc);
		kfree(chip->deep_dischg_info_trigger);
		chip->deep_dischg_info_trigger = NULL;
		mutex_unlock(&chip->deep_dischg_info_lock);
		return rc;
	}
	index += scnprintf(&(chip->deep_dischg_info_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "%s",
			  data.strval);

	schedule_delayed_work(&chip->deep_dischg_info_trigger_work, 0);
	chg_info("success\n");
	return 0;
}

static int oplus_chg_track_update_device_id(struct oplus_chg_track *chip)
{
	union mms_msg_data data = { 0 };
	int rc = 0;
	int i;
	char *str = NULL;
	char *temp_str = NULL;
	struct track_device_name device_name[] = {
		{"pps_ic=", TRACK_PPS_DEVICE_NAME},
		{"vooc_ic=", TRACK_VOOC_DEVICE_NAME},
		{"ufcs_ic=", TRACK_UFCS_DEVICE_NAME},
		{"buck_ic=", TRACK_BUCK_DEVICE_NAME},
	};

	if (!chip)
		return -EINVAL;

	rc = oplus_mms_get_item_data(chip->monitor->err_topic, ERR_ITEM_DEVICE_ID, &data, false);
	if (rc < 0) {
		chg_err("get msg data error, rc=%d\n", rc);
		return rc;
	}

	if (data.strval) {
		chg_info("%s\n", data.strval);
		for (i = 0; i < ARRAY_SIZE(device_name); i++) {
			str = strstr(data.strval, device_name[i].suffixes);
			if (str) {
				str += strlen(device_name[i].suffixes);
				temp_str = strstr(str, "oplus,");
				if (temp_str)
					str = temp_str + strlen("oplus,");
				strncpy(chip->device_ic_name[device_name[i].index], str,
					OPLUS_CHG_TRACK_DEVICE_NAME_LEN - 1);
				chg_debug("str=%s, index=%d\n", str, device_name[i].index);
				break;
			}
		}
	}

	return rc;
}

static int oplus_chg_track_upload_plc_info(struct oplus_chg_track *chip)
{
	int index = 0;
	union mms_msg_data data = { 0 };
	int rc = 0;

	if (!chip)
		return -EINVAL;

	mutex_lock(&chip->plc_info_lock);
	if (chip->plc_info_trigger)
		kfree(chip->plc_info_trigger);

	chip->plc_info_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->plc_info_trigger) {
		chg_err("plc_info_trigger memery alloc fail\n");
		mutex_unlock(&chip->plc_info_lock);
		return -ENOMEM;
	}

	chip->plc_info_trigger->type_reason = TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	chip->plc_info_trigger->flag_reason = TRACK_NOTIFY_FLAG_PLC_INFO;

	rc = oplus_mms_get_item_data(chip->monitor->err_topic, ERR_ITEM_PLC_INFO, &data, false);
	if (rc < 0) {
		chg_err("get msg data error, rc=%d\n", rc);
		kfree(chip->plc_info_trigger);
		chip->plc_info_trigger = NULL;
		mutex_unlock(&chip->plc_info_lock);
		return rc;
	}
	index += scnprintf(&(chip->plc_info_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "%s",
			  data.strval);

	schedule_delayed_work(&chip->plc_info_trigger_work, 0);
	chg_info("success\n");
	return 0;
}

static int oplus_chg_track_upload_dec_cv_info(struct oplus_chg_track *chip)
{
	int index = 0;
	union mms_msg_data data = { 0 };
	int rc = 0;

	if (!chip)
		return -EINVAL;

	mutex_lock(&chip->deep_dischg_info_lock);
	if (chip->deep_dischg_info_trigger)
		kfree(chip->deep_dischg_info_trigger);

	chip->deep_dischg_info_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->deep_dischg_info_trigger) {
		chg_err("dec_cv_info_trigger memery alloc fail\n");
		mutex_unlock(&chip->deep_dischg_info_lock);
		return -ENOMEM;
	}

	chip->deep_dischg_info_trigger->type_reason = TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	chip->deep_dischg_info_trigger->flag_reason = TRACK_NOTIFY_FLAG_DEC_CV_INFO;

	rc = oplus_mms_get_item_data(chip->monitor->err_topic, ERR_ITEM_DEC_CV_INFO, &data, false);
	if (rc < 0) {
		chg_err("get msg data error, rc=%d\n", rc);
		kfree(chip->deep_dischg_info_trigger);
		chip->deep_dischg_info_trigger = NULL;
		mutex_unlock(&chip->deep_dischg_info_lock);
		return rc;
	}
	index += scnprintf(&(chip->deep_dischg_info_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "%s",
			  data.strval);

	schedule_delayed_work(&chip->deep_dischg_info_trigger_work, 0);
	chg_info("success\n");
	return 0;
}

static int oplus_chg_track_upload_fcl_info(struct oplus_chg_track *chip)
{
	union mms_msg_data data = { 0 };
	int rc = 0;
	char *str;

	if (!chip)
		return -EINVAL;

	rc = oplus_mms_get_item_data(chip->monitor->err_topic, ERR_ITEM_FCL_INFO, &data, false);
	if (rc < 0) {
		chg_err("get msg data error, rc=%d\n", rc);
		return rc;
	}

	if (!data.strval)
		return -EINVAL;

	str = strstr(data.strval, "one_time_full=");
	if (str) {
		str += strlen("one_time_full=");
		chip->track_status.fcl.one_full_trigger_cnt++;
		if (!strlen(chip->track_status.fcl.info))
			scnprintf(chip->track_status.fcl.info, TRACK_FCL_INFO_LEN - 1, "%s", str);
	} else {
		str = strstr(data.strval, "n_time_full=");
		if (str)
			chip->track_status.fcl.n_full_trigger_cnt++;
		else
			chg_err("fcl info error");
	}

	chg_info("fcl info:%s, cnt:%d,%d\n", data.strval,
		chip->track_status.fcl.one_full_trigger_cnt, chip->track_status.fcl.n_full_trigger_cnt);

	return rc;
}

static int oplus_chg_track_upload_chg_into_liquid(struct oplus_chg_track *track)
{
	union mms_msg_data data = { 0 };
	int rc;

	rc = oplus_mms_get_item_data(track->monitor->err_topic,
				     ERR_ITEM_CHG_INTO_LIQUID, &data, false);
	if (rc < 0) {
		chg_err("get msg data error, rc=%d\n", rc);
		return rc;
	}
	scnprintf(track->chg_into_liquid_load_trigger.crux_info,
		 OPLUS_CHG_TRACK_CURX_INFO_LEN, "%s", data.strval);

	schedule_delayed_work(&track->chg_into_liquid_trigger_work, 0);
	chg_info("%s\n", track->chg_into_liquid_load_trigger.crux_info);

	return 0;
}


static int oplus_chg_track_upload_gauge_r_info(struct oplus_chg_track *chip)
{
	int index = 0;
	union mms_msg_data data = { 0 };
	int rc = 0;

	if (!chip)
		return -EINVAL;

	mutex_lock(&chip->deep_dischg_info_lock);
	if (chip->deep_dischg_info_trigger)
		kfree(chip->deep_dischg_info_trigger);

	chip->deep_dischg_info_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->deep_dischg_info_trigger) {
		chg_err("deep_dischg_info_trigger memery alloc fail\n");
		mutex_unlock(&chip->deep_dischg_info_lock);
		return -ENOMEM;
	}

	chip->deep_dischg_info_trigger->type_reason = TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	chip->deep_dischg_info_trigger->flag_reason = TRACK_NOTIFY_FLAG_GAUGE_INFO;
	index += scnprintf(&(chip->deep_dischg_info_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				  "$$err_scene@@end_r_info");
	rc = oplus_mms_get_item_data(chip->monitor->err_topic, ERR_ITEM_GAUGE_R_INFO, &data, false);
	if (rc < 0) {
		chg_err("get msg data error, rc=%d\n", rc);
		kfree(chip->deep_dischg_info_trigger);
		chip->deep_dischg_info_trigger = NULL;
		mutex_unlock(&chip->deep_dischg_info_lock);
		return rc;
	}
	index += scnprintf(&(chip->deep_dischg_info_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "%s",
			  data.strval);

	schedule_delayed_work(&chip->deep_dischg_info_trigger_work, 0);
	chg_info("success\n");
	return 0;
}

static int oplus_chg_track_upload_shutdown_vol_info(struct oplus_chg_track *chip)
{
	oplus_chg_track_trigger *trigger;
	union mms_msg_data data = { 0 };
	int rc = 0;

	if (!chip)
		return -EINVAL;

	rc = oplus_mms_get_item_data(chip->monitor->err_topic, ERR_ITEM_SHUTDOWN_VOL, &data, false);
	if (rc < 0) {
		chg_err("get msg data error, rc=%d\n", rc);
		return rc;
	}

	trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!trigger) {
		chg_err("shutdown_vol_trigger memery alloc fail\n");
		return -ENOMEM;
	}

	trigger->type_reason = TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	trigger->flag_reason = TRACK_NOTIFY_FLAG_SHUTDOWN_VOL;
	strncpy(trigger->crux_info, data.strval, OPLUS_CHG_TRACK_CURX_INFO_LEN - 1);
	trigger->crux_info[OPLUS_CHG_TRACK_CURX_INFO_LEN - 1] = '\0';

	oplus_chg_track_upload_trigger_data(trigger);
	kfree(trigger);
	chg_info("success\n");
	return 0;
}

static int oplus_chg_track_upload_deep_dischg_profile(struct oplus_chg_track *chip)
{
	int index = 0;
	union mms_msg_data data = { 0 };
	int rc = 0;

	if (!chip)
		return -EINVAL;

	mutex_lock(&chip->deep_dischg_info_lock);
	if (chip->deep_dischg_info_trigger)
		kfree(chip->deep_dischg_info_trigger);

	chip->deep_dischg_info_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->deep_dischg_info_trigger) {
		chg_err("deep_dischg_info_trigger memery alloc fail\n");
		mutex_unlock(&chip->deep_dischg_info_lock);
		return -ENOMEM;
	}

	chip->deep_dischg_info_trigger->type_reason = TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	chip->deep_dischg_info_trigger->flag_reason = TRACK_NOTIFY_FLAG_DEEP_DISCHG_PROFILE;

	rc = oplus_mms_get_item_data(chip->monitor->err_topic, ERR_ITEM_DEEP_DISCHG_PROFILE, &data, false);
	if (rc < 0) {
		chg_err("get msg data error, rc=%d\n", rc);
		kfree(chip->deep_dischg_info_trigger);
		chip->deep_dischg_info_trigger = NULL;
		mutex_unlock(&chip->deep_dischg_info_lock);
		return rc;
	}
	index += scnprintf(&(chip->deep_dischg_info_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "%s",
			  data.strval);

	schedule_delayed_work(&chip->deep_dischg_info_trigger_work, 0);
	chg_info("success\n");
	return 0;
}

static int oplus_chg_track_upload_bidirect_cp_info(struct oplus_chg_track *chip)
{
	int index = 0;
	union mms_msg_data data = { 0 };
	int rc = 0;

	if (!chip)
		return -EINVAL;

	mutex_lock(&chip->bidirect_cp_info_lock);
	if (chip->bidirect_cp_info_trigger)
		kfree(chip->bidirect_cp_info_trigger);

	chip->bidirect_cp_info_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->bidirect_cp_info_trigger) {
		chg_err("memery alloc fail\n");
		mutex_unlock(&chip->bidirect_cp_info_lock);
		return -ENOMEM;
	}

	chip->bidirect_cp_info_trigger->type_reason = TRACK_NOTIFY_TYPE_DEVICE_ABNORMAL;
	chip->bidirect_cp_info_trigger->flag_reason = TRACK_NOTIFY_FLAG_CP_ABNORMAL;

	rc = oplus_mms_get_item_data(chip->monitor->err_topic, ERR_ITEM_BIDIRECT_CP_INFO, &data, false);
	if (rc < 0) {
		chg_err("get msg data error, rc=%d\n", rc);
		kfree(chip->bidirect_cp_info_trigger);
		chip->bidirect_cp_info_trigger = NULL;
		mutex_unlock(&chip->bidirect_cp_info_lock);
		return rc;
	}

	index += scnprintf(&(chip->bidirect_cp_info_trigger->crux_info[index]),
			  OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "%s",
			  data.strval);
	oplus_chg_track_obtain_power_info(&(chip->bidirect_cp_info_trigger->crux_info[index]),
					  OPLUS_CHG_TRACK_CURX_INFO_LEN - index);

	schedule_delayed_work(&chip->bidirect_cp_info_trigger_work, 0);
	chg_info("success\n");
	return 0;
}

static int oplus_chg_track_upload_eis_timeout_info(struct oplus_chg_track *chip)
{
	int index = 0;
	union mms_msg_data data = { 0 };
	int rc = 0;

	if (!chip)
		return -EINVAL;

	mutex_lock(&chip->eis_timeout_info_lock);
	if (chip->eis_timeout_info_trigger)
		kfree(chip->eis_timeout_info_trigger);

	chip->eis_timeout_info_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->eis_timeout_info_trigger) {
		pr_err("eis_timeout_info_trigger memery alloc fail\n");
		mutex_unlock(&chip->eis_timeout_info_lock);
		return -ENOMEM;
	}

	chip->eis_timeout_info_trigger->type_reason = TRACK_NOTIFY_TYPE_SOFTWARE_ABNORMAL;
	chip->eis_timeout_info_trigger->flag_reason = TRACK_NOTIFY_FLAG_EIS_ABNORMAL;

	rc = oplus_mms_get_item_data(chip->monitor->err_topic, ERR_ITEM_EIS_TIMEOUT, &data, false);
	if (rc < 0) {
		chg_err("get msg data error, rc=%d\n", rc);
		kfree(chip->eis_timeout_info_trigger);
		chip->eis_timeout_info_trigger = NULL;
		mutex_unlock(&chip->eis_timeout_info_lock);
		return rc;
	}

	index += scnprintf(&(chip->eis_timeout_info_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "%s", data.strval);

	schedule_delayed_work(&chip->eis_timeout_info_trigger_work, 0);
	chg_info("success\n");
	return 0;
}

static int oplus_chg_track_upload_bal_info(struct oplus_chg_track *chip)
{
	int index = 0;
	union mms_msg_data data = { 0 };
	int rc = 0;

	if (!chip)
		return -EINVAL;

	mutex_lock(&chip->bal_info_lock);
	if (chip->bal_info_trigger)
		kfree(chip->bal_info_trigger);

	chip->bal_info_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->bal_info_trigger) {
		chg_err("mmi_chg_info_trigger memery alloc fail\n");
		mutex_unlock(&chip->bal_info_lock);
		return -ENOMEM;
	}

	chip->bal_info_trigger->type_reason = TRACK_NOTIFY_TYPE_SOFTWARE_ABNORMAL;
	chip->bal_info_trigger->flag_reason = TRACK_NOTIFY_FLAG_BAL_ABNORMAL;

	rc = oplus_mms_get_item_data(chip->monitor->err_topic, ERR_ITEM_BAL_INFO, &data, false);
	if (rc < 0) {
		chg_err("get msg data error, rc=%d\n", rc);
		kfree(chip->bal_info_trigger);
		chip->bal_info_trigger = NULL;
		mutex_unlock(&chip->bal_info_lock);
		return rc;
	}

	index += scnprintf(&(chip->bal_info_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "%s",
			  data.strval);
	oplus_chg_track_obtain_power_info(&(chip->bal_info_trigger->crux_info[index]),
					  OPLUS_CHG_TRACK_CURX_INFO_LEN - index);

	schedule_delayed_work(&chip->bal_info_trigger_work, 0);
	chg_info("success\n");
	return 0;
}

static int oplus_chg_track_upload_phy_cp_info(struct oplus_chg_track *chip)
{
	int index = 0;
	union mms_msg_data data = { 0 };
	int rc = 0;

	if (!chip)
		return -EINVAL;

	mutex_lock(&chip->phy_cp_info_lock);
	if (chip->phy_cp_info_trigger)
		kfree(chip->phy_cp_info_trigger);

	chip->phy_cp_info_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->phy_cp_info_trigger) {
		chg_err("memery alloc fail\n");
		mutex_unlock(&chip->phy_cp_info_lock);
		return -ENOMEM;
	}

	chip->phy_cp_info_trigger->type_reason = TRACK_NOTIFY_TYPE_DEVICE_ABNORMAL;
	chip->phy_cp_info_trigger->flag_reason = TRACK_NOTIFY_FLAG_CP_ABNORMAL;

	rc = oplus_mms_get_item_data(chip->monitor->err_topic, ERR_ITEM_ERR_PHY_CP_INFO, &data, false);
	if (rc < 0) {
		chg_err("get msg data error, rc=%d\n", rc);
		kfree(chip->phy_cp_info_trigger);
		chip->phy_cp_info_trigger = NULL;
		mutex_unlock(&chip->phy_cp_info_lock);
		return rc;
	}

	oplus_chg_track_cp_abnormal_type_vector_ele_add(data.strval);

	index += scnprintf(&(chip->phy_cp_info_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "%s", data.strval);
	oplus_chg_track_obtain_power_info(&(chip->phy_cp_info_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index);

	schedule_delayed_work(&chip->phy_cp_info_trigger_work, 0);
	chg_info("success\n");

	return 0;
}

static int oplus_chg_track_upload_soccp_crash_info(struct oplus_chg_track *chip)
{
	int index = 0;
	union mms_msg_data data = { 0 };
	u32 crash_reason = 0;
	int rc = 0;

	if (!chip)
		return -EINVAL;

	mutex_lock(&chip->soccp_crash_lock);
	if (chip->soccp_crash_trigger)
		kfree(chip->soccp_crash_trigger);

	chip->soccp_crash_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->soccp_crash_trigger) {
		chg_err("memery alloc fail\n");
		mutex_unlock(&chip->soccp_crash_lock);
		return -ENOMEM;
	}

	chip->soccp_crash_trigger->type_reason = TRACK_NOTIFY_TYPE_CRASH;
	chip->soccp_crash_trigger->flag_reason = TRACK_NOTIFY_FLAG_SOCCP_CRASH;

	rc = oplus_mms_get_item_data(chip->monitor->err_topic, ERR_ITEM_SOCCP_CRASH, &data, false);
	if (rc < 0)
		chg_err("get crash_reason error, rc=%d\n", rc);
	else
		crash_reason = (u32)(data.intval);

	index += scnprintf(&(chip->soccp_crash_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$crash_reason@@%u", crash_reason);

	schedule_delayed_work(&chip->soccp_crash_trigger_work, 0);
	chg_info("success\n");

	return 0;
}

static int oplus_chg_track_upload_bs_info(struct oplus_chg_track *chip)
{
	int index = 0;
	union mms_msg_data data = { 0 };
	int rc = 0;

	if (!chip)
		return -EINVAL;

	mutex_lock(&chip->bs_info_lock);
	if (chip->bs_info_trigger)
		kfree(chip->bs_info_trigger);

	chip->bs_info_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->bs_info_trigger) {
		chg_err("bs_info_trigger memery alloc fail\n");
		mutex_unlock(&chip->bs_info_lock);
		return -ENOMEM;
	}

	chip->bs_info_trigger->type_reason = TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	chip->bs_info_trigger->flag_reason = TRACK_NOTIFY_FLAG_BS_INFO;

	rc = oplus_mms_get_item_data(chip->monitor->err_topic, ERR_ITEM_BS_INFO, &data, false);
	if (rc < 0) {
		chg_err("get msg data error, rc=%d\n", rc);
		kfree(chip->bs_info_trigger);
		chip->bs_info_trigger = NULL;
		mutex_unlock(&chip->bs_info_lock);
		return rc;
	}
	index += scnprintf(&(chip->bs_info_trigger->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$bs_info@@%s", data.strval);

	schedule_delayed_work(&chip->bs_info_trigger_work, 0);
	chg_info("success\n");
	return 0;
}

static int oplus_chg_track_upload_usbin_abnormal(struct oplus_chg_track *chip)
{
	int index = 0;

	if (!chip)
		return -EINVAL;

	mutex_lock(&chip->usbin_abnormal_lock);
	if (chip->usbin_abnormal_trigger)
		kfree(chip->usbin_abnormal_trigger);

	chip->usbin_abnormal_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->usbin_abnormal_trigger) {
		chg_err("usbin_abnormal_trigger memery alloc fail\n");
		mutex_unlock(&chip->usbin_abnormal_lock);
		return -ENOMEM;
	}

	chip->usbin_abnormal_trigger->type_reason = TRACK_NOTIFY_TYPE_SOFTWARE_ABNORMAL;
	chip->usbin_abnormal_trigger->flag_reason = TRACK_NOTIFY_FLAG_USBIN_ABNORMAL;
	oplus_chg_track_obtain_power_info(&(chip->usbin_abnormal_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index);

	schedule_delayed_work(&chip->usbin_abnormal_trigger_work, 0);
	chg_info("success\n");
	return 0;
}

static int oplus_chg_track_upload_state_keep_info(struct oplus_chg_track *chip)
{
	union mms_msg_data data = { 0 };
	int rc;

	if (!chip)
		return -EINVAL;

	mutex_lock(&chip->state_keep_info_lock);
	if (chip->state_keep_info_trigger)
		kfree(chip->state_keep_info_trigger);

	chip->state_keep_info_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->state_keep_info_trigger) {
		chg_err("memory alloc fail\n");
		mutex_unlock(&chip->state_keep_info_lock);
		return -ENOMEM;
	}

	chip->state_keep_info_trigger->type_reason = TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	chip->state_keep_info_trigger->flag_reason = TRACK_NOTIFY_FLAG_STATE_KEEP_INFO;

	rc = oplus_mms_get_item_data(chip->monitor->err_topic, ERR_ITEM_STATE_KEEP_INFO, &data, false);
	if (rc < 0) {
		chg_err("get state keep info error, rc=%d\n", rc);
		kfree(chip->state_keep_info_trigger);
		chip->state_keep_info_trigger = NULL;
		mutex_unlock(&chip->state_keep_info_lock);
		return -ENOMEM;
	}

	scnprintf(chip->state_keep_info_trigger->crux_info,
		OPLUS_CHG_TRACK_CURX_INFO_LEN, "%s", data.strval);
	schedule_delayed_work(&chip->state_keep_info_trigger_work, 0);
	chg_info("success\n");

	return 0;
}

static int oplus_chg_track_upload_state_keep_abnormal(struct oplus_chg_track *chip)
{
	union mms_msg_data data = { 0 };
	int rc;

	if (!chip)
		return -EINVAL;

	mutex_lock(&chip->state_keep_info_lock);
	if (chip->state_keep_info_trigger)
		kfree(chip->state_keep_info_trigger);

	chip->state_keep_info_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->state_keep_info_trigger) {
		chg_err("memory alloc fail\n");
		mutex_unlock(&chip->state_keep_info_lock);
		return -ENOMEM;
	}

	chip->state_keep_info_trigger->type_reason = TRACK_NOTIFY_TYPE_SOFTWARE_ABNORMAL;
	chip->state_keep_info_trigger->flag_reason = TRACK_NOTIFY_FLAG_STATE_KEEP_ABNORMAL;

	rc = oplus_mms_get_item_data(chip->monitor->err_topic, ERR_ITEM_STATE_KEEP_ABNORMAL, &data, false);
	if (rc < 0) {
		chg_err("get state keep abnormal error, rc=%d\n", rc);
		kfree(chip->state_keep_info_trigger);
		chip->state_keep_info_trigger = NULL;
		mutex_unlock(&chip->state_keep_info_lock);
		return -ENOMEM;
	}

	scnprintf(chip->state_keep_info_trigger->crux_info,
		OPLUS_CHG_TRACK_CURX_INFO_LEN, "%s", data.strval);
	schedule_delayed_work(&chip->state_keep_info_trigger_work, 0);
	chg_info("success\n");

	return 0;
}

static int oplus_chg_track_upload_sec_ic_mem_info(struct oplus_chg_track *chip)
{
	union mms_msg_data data = { 0 };
	int rc;

	if (!chip)
		return -EINVAL;

	mutex_lock(&chip->sec_ic_meminfo_lock);
	if (chip->sec_ic_meminfo_trigger)
		kfree(chip->sec_ic_meminfo_trigger);

	chip->sec_ic_meminfo_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!chip->sec_ic_meminfo_trigger) {
		chg_err("memory alloc fail\n");
		mutex_unlock(&chip->sec_ic_meminfo_lock);
		return -ENOMEM;
	}

	chip->sec_ic_meminfo_trigger->type_reason = TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	chip->sec_ic_meminfo_trigger->flag_reason = TRACK_NOTIFY_FLAG_SEC_IC_MEMINFO;

	rc = oplus_mms_get_item_data(chip->monitor->err_topic, ERR_ITEM_SEC_IC_MEM_INFO, &data, false);
	if (rc < 0) {
		chg_err("get sec ic meminfo error, rc=%d\n", rc);
		kfree(chip->sec_ic_meminfo_trigger);
		chip->sec_ic_meminfo_trigger = NULL;
		mutex_unlock(&chip->sec_ic_meminfo_lock);
		return -ENOMEM;
	}

	scnprintf(chip->sec_ic_meminfo_trigger->crux_info,
		OPLUS_CHG_TRACK_CURX_INFO_LEN, "%s", data.strval);
	schedule_delayed_work(&chip->sec_ic_meminfo_trigger_work, 0);
	chg_info("success\n");
	return 0;
}

static int
oplus_chg_track_cal_period_chg_capaticy(struct oplus_chg_track *track_chip)
{
	int ret = 0;

	if (!track_chip)
		return -EFAULT;

	if (track_chip->track_status.chg_start_soc >
	    TRACK_PERIOD_CHG_CAP_MAX_SOC)
		return ret;

	chg_debug("enter\n");
	schedule_delayed_work(&track_chip->cal_chg_five_mins_capacity_work,
			      msecs_to_jiffies(TRACK_TIME_5MIN_JIFF_THD));
	schedule_delayed_work(&track_chip->cal_chg_ten_mins_capacity_work,
			      msecs_to_jiffies(TRACK_TIME_10MIN_JIFF_THD));
	schedule_delayed_work(&track_chip->cal_chg_twenty_mins_capacity_work,
			      msecs_to_jiffies(TRACK_TIME_20MIN_JIFF_THD));
	schedule_delayed_work(&track_chip->cal_chg_thirty_mins_capacity_work,
			      msecs_to_jiffies(TRACK_TIME_30MIN_JIFF_THD));

	return ret;
}

static int oplus_chg_track_cancel_cal_period_chg_capaticy(
	struct oplus_chg_track *track_chip)
{
	int ret = 0;

	if (!track_chip)
		return -EFAULT;

	chg_debug("enter\n");
	if (delayed_work_pending(&track_chip->cal_chg_five_mins_capacity_work))
		cancel_delayed_work_sync(
			&track_chip->cal_chg_five_mins_capacity_work);

	if (delayed_work_pending(&track_chip->cal_chg_ten_mins_capacity_work))
		cancel_delayed_work_sync(
			&track_chip->cal_chg_ten_mins_capacity_work);

	if (delayed_work_pending(&track_chip->cal_chg_twenty_mins_capacity_work))
		cancel_delayed_work_sync(&track_chip->cal_chg_twenty_mins_capacity_work);

	if (delayed_work_pending(&track_chip->cal_chg_thirty_mins_capacity_work))
		cancel_delayed_work_sync(&track_chip->cal_chg_thirty_mins_capacity_work);

	return ret;
}

static int oplus_chg_track_cal_period_chg_average_speed(
	struct oplus_chg_track_status *track_status,
	struct oplus_monitor *monitor)
{
	int ret = 0;
	int curr_time;

	if (!track_status || !monitor)
		return -EFAULT;

	chg_debug("enter\n");
	if (track_status->chg_average_speed ==
	    TRACK_PERIOD_CHG_AVERAGE_SPEED_INIT) {
		curr_time = oplus_chg_track_get_local_time_s();
		track_status->chg_average_speed =
			TRACK_TIME_1MIN_THD *
			(monitor->batt_rm - track_status->chg_start_rm) /
			(curr_time - track_status->chg_start_time);
		chg_info("curr_rm:%d, chg_start_rm:%d, curr_time:%d, chg_start_time:%d,"
			"chg_average_speed:%d\n",
			monitor->batt_rm, track_status->chg_start_rm, curr_time,
			track_status->chg_start_time,
			track_status->chg_average_speed);
	}

	return ret;
}

static int
oplus_chg_track_cal_chg_end_soc(struct oplus_monitor *monitor,
				struct oplus_chg_track_status *track_status)
{
	if (!track_status || !monitor)
		return -EFAULT;

	if (monitor->batt_status == POWER_SUPPLY_STATUS_CHARGING)
		track_status->chg_end_soc = monitor->batt_soc;

	return 0;
}

static int oplus_chg_track_cal_chg_temp(struct oplus_monitor *monitor, struct oplus_chg_track_status *track_status)
{
	if (!track_status || !monitor)
		return -EFAULT;

	if (monitor->batt_status == POWER_SUPPLY_STATUS_CHARGING)
		track_status->chg_end_temp = monitor->shell_temp;

	return 0;
}

static int oplus_chg_track_cal_soc_time(struct oplus_monitor *monitor, struct oplus_chg_track_status *track_status)
{
	if (!track_status || !monitor)
		return -EFAULT;

	if (track_status->chg_soc50_time <= 0 && monitor->batt_status == POWER_SUPPLY_STATUS_CHARGING &&
	    track_status->chg_start_soc < 50 && monitor->batt_soc == 50)
		track_status->chg_soc50_time =
			(oplus_chg_track_get_local_time_s() - track_status->chg_start_time) / TRACK_TIME_1MIN_THD;
	return 0;
}

static int oplus_chg_track_cal_hyper_speed_status(struct oplus_chg_track_status *track_status)
{
	struct oplus_smart_hyper_param param = { 0 };
	int rc = 0;

	if (!track_status)
		return -EFAULT;

	rc = oplus_smart_chg_get_hyper_param(&param);
	if (rc < 0)
		return rc;

	track_status->hyper_stop_soc = param.stop_soc;
	track_status->hyper_stop_temp = param.stop_temp;
	track_status->hyper_last_time = param.stop_time - param.start_time;
	if (track_status->hyper_last_time > 0)
		track_status->hyper_ave_speed =
			(param.stop_cap - param.start_cap) / (track_status->hyper_last_time / 60);
	else
		track_status->hyper_ave_speed = 0;
	track_status->hyper_est_save_time = param.gain_time_ms / 1000;
	if ((param.start_time != 0) && (track_status->hyper_en == 0))
		track_status->hyper_en = 0xff;

	return 0;
}

static void oplus_chg_track_hyper_speed_record(struct oplus_chg_track_status *track_status)
{
	int index = 0;

	if (track_status->hyper_en) {
		index += scnprintf(&(track_status->hyper_info[index]), TRACK_HIDL_HYPER_INFO_LEN - index,
				  "hyper_en=%d,", track_status->hyper_en);

		index += scnprintf(&(track_status->hyper_info[index]), TRACK_HIDL_HYPER_INFO_LEN - index,
				  "hyper_stop_cap=%d,", track_status->hyper_stop_soc);

		index += scnprintf(&(track_status->hyper_info[index]), TRACK_HIDL_HYPER_INFO_LEN - index,
				  "hyper_stop_temp=%d,", track_status->hyper_stop_temp);

		index += scnprintf(&(track_status->hyper_info[index]), TRACK_HIDL_HYPER_INFO_LEN - index,
				  "hyper_last_t=%d,", track_status->hyper_last_time);

		index += scnprintf(&(track_status->hyper_info[index]), TRACK_HIDL_HYPER_INFO_LEN - index,
				  "hyper_ave_speed=%d,", track_status->hyper_ave_speed);

		index += scnprintf(&(track_status->hyper_info[index]), TRACK_HIDL_HYPER_INFO_LEN - index,
				  "hyper_est_save_t=%d", track_status->hyper_est_save_time);
	}
}

static void oplus_chg_track_reset_chg_abnormal_happened_flag(
	struct oplus_chg_track_status *track_status)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(chg_abnormal_reason_table); i++)
		chg_abnormal_reason_table[i].happened = false;
}

static int
oplus_chg_track_status_reset(struct oplus_chg_track_status *track_status)
{
	memset(&(track_status->fcl), 0, sizeof(track_status->fcl));
	track_status->chg_no_charging_cnt = 0;
	track_status->chg_no_chg_time_s = 0;
	track_status->no_charging_cal_curr_time = 0;
	track_status->no_charging_cal_pre_time = 0;
	track_status->ledon_ave_speed = 0;
	track_status->ledoff_ave_speed = 0;
	track_status->ledon_rm = 0;
	track_status->ledoff_rm = 0;
	track_status->ledon_time = 0;
	track_status->ledoff_time = 0;
	track_status->continue_ledon_time = 0;
	track_status->continue_ledoff_time = 0;
	track_status->chg_total_cnt = 0;
	track_status->chg_total_time = 0;
	track_status->chg_fast_full_time = 0;
	track_status->chg_normal_full_time = 0;
	track_status->chg_report_full_time = 0;
	track_status->aging_ffc_trig = false;
	memset(&(track_status->aging_ffc_judge_vol), 0, sizeof(track_status->aging_ffc_judge_vol));
	track_status->aging_ffc_start_time = 0;
	track_status->aging_ffc_to_full_time = 0;
	track_status->ffc_end_time = 0;
	track_status->ffc_time = 0;
	track_status->cv_time = 0;
	track_status->ffc_start_main_soc = 0;
	track_status->ffc_start_sub_soc = 0;
	track_status->ffc_end_main_soc = 0;
	track_status->ffc_end_sub_soc = 0;
	track_status->dual_chan_open_count = 0;
	track_status->dual_chan_start_time = 0;
	track_status->dual_chan_time = 0;
	track_status->chg_five_mins_cap = TRACK_PERIOD_CHG_CAP_INIT;
	track_status->chg_ten_mins_cap = TRACK_PERIOD_CHG_CAP_INIT;
	track_status->chg_twenty_mins_cap = TRACK_PERIOD_CHG_CAP_INIT;
	track_status->chg_thirty_mins_cap = TRACK_PERIOD_CHG_CAP_INIT;
	track_status->chg_average_speed = TRACK_PERIOD_CHG_AVERAGE_SPEED_INIT;
	track_status->soc_sect_status = TRACK_SOC_SECTION_DEFAULT;
	track_status->tbatt_warm_once = false;
	track_status->tbatt_cold_once = false;
	track_status->cool_down_effect_cnt = 0;
	track_status->chg_speed_is_slow = false;
	track_status->in_rechging = false;
	track_status->rechg_counts = 0;
	track_status->led_onoff_power_cal = false;
	track_status->cool_down_status = 0;
	track_status->has_judge_speed = false;
	track_status->wls_prop_status = TRACK_CHG_DEFAULT;
	track_status->wls_skew_effect_cnt = 0;
	track_status->chg_verity = true;
	memset(&(track_status->batt_full_reason), 0,
	       sizeof(track_status->batt_full_reason));
	oplus_chg_track_clear_cool_down_stats_time(track_status);
	memset(&(track_status->chg_abnormal_reason), 0,
	       sizeof(track_status->chg_abnormal_reason));
	oplus_chg_track_reset_chg_abnormal_happened_flag(track_status);
	memset(track_status->bcc_info, 0, sizeof(struct oplus_chg_track_hidl_bcc_info));

	track_status->app_status.app_cal = false;
	track_status->app_status.curr_top_index = TRACK_APP_TOP_INDEX_DEFAULT;
	mutex_lock(&track_status->app_status.app_lock);
	strncpy(track_status->app_status.curr_top_name,
		TRACK_APP_REAL_NAME_DEFAULT, TRACK_APP_REAL_NAME_LEN - 1);
	mutex_unlock(&track_status->app_status.app_lock);
	oplus_chg_track_clear_app_time();

	return 0;
}

static int oplus_chg_track_status_reset_when_plugin(
	struct oplus_monitor *monitor,
	struct oplus_chg_track_status *track_status)
{
	track_status->soc_sect_status = TRACK_SOC_SECTION_DEFAULT;
	track_status->chg_speed_is_slow = false;
	track_status->chg_start_time = oplus_chg_track_get_local_time_s();
	track_status->chg_end_time = track_status->chg_start_time;
	track_status->chg_start_soc = monitor->batt_soc;
	track_status->chg_end_soc = monitor->batt_soc;
	track_status->led_on = monitor->led_on;
	track_status->led_change_t = track_status->chg_start_time;
	track_status->led_change_rm = monitor->batt_rm;
	track_status->chg_start_temp = monitor->shell_temp;
	track_status->chg_end_temp = monitor->shell_temp;
	track_status->chg_soc50_time = 0;
	track_status->batt_start_temp = monitor->batt_temp;
	track_status->batt_max_temp = monitor->batt_temp;
	track_status->batt_max_vol = monitor->vbat_mv;
	track_status->batt_max_curr = monitor->ibat_ma;
	track_status->chg_max_vol = monitor->wired_vbus_mv;
	track_status->chg_start_vol = monitor->vbat_mv;
	track_status->chg_start_cur = monitor->ibat_ma;
	track_status->chg_start_rm = monitor->batt_rm;
	track_status->chg_max_temp = monitor->shell_temp;
	track_status->ledon_time = 0;
	track_status->ledoff_time = 0;
	track_status->continue_ledon_time = 0;
	track_status->continue_ledoff_time = 0;
	track_status->cool_down_status = monitor->cool_down;
	track_status->cool_down_status_change_t = track_status->chg_start_time;
	track_status->has_judge_speed = false;
	track_status->wls_prop_status = TRACK_CHG_DEFAULT;
	track_status->wls_skew_effect_cnt = 0;
	track_status->chg_verity = true;
	track_status->chg_plugin_utc_t = oplus_chg_track_get_current_time_s(
		&track_status->chg_plugin_rtc_t);
	oplus_chg_track_cal_period_chg_capaticy(g_track_chip);
	track_status->prop_status = monitor->batt_status;
	strncpy(track_status->app_status.pre_top_name,
		track_status->app_status.curr_top_name, TRACK_APP_REAL_NAME_LEN - 1);
	track_status->app_status.pre_top_name[TRACK_APP_REAL_NAME_LEN - 1] = '\0';
	track_status->app_status.change_t = track_status->chg_start_time;
	track_status->app_status.curr_top_index =
		oplus_chg_track_match_app_info(
		track_status->app_status.pre_top_name);
	track_status->once_mmi_chg = false;
	track_status->once_chg_cycle_status = CHG_CYCLE_VOTER__NONE;
	track_status->mmi_chg_open_t = 0;
	track_status->mmi_chg_close_t = 0;
	track_status->mmi_chg_constant_t = 0;
	track_status->slow_chg_open_t = 0;
	track_status->slow_chg_close_t = 0;
	track_status->slow_chg_open_n_t = 0;
	track_status->slow_chg_duration = 0;
	track_status->slow_chg_open_cnt = 0;
	track_status->slow_chg_watt = 0;
	track_status->slow_chg_pct = 0;
	track_status->not_record_reason = 0;
	track_status->pre_fastchg_type = 0;

	chg_info("chg_start_time:%d, chg_start_soc:%d, chg_start_temp:%d, prop_status:%d\n",
		track_status->chg_start_time, track_status->chg_start_soc,
		track_status->chg_start_temp, track_status->prop_status);

	return 0;
}

static int oplus_chg_track_no_record_reason(struct oplus_monitor *monitor,
	struct oplus_chg_track_status *track_status)
{
	if (!track_status || !monitor)
		return -EFAULT;

	if (!monitor->chg_cycle_status && !oplus_chg_track_get_mmi_chg())
		set_bit(NOT_RECORD_MMI, &track_status->not_record_reason);

	if (monitor->chg_cycle_status & CHG_CYCLE_VOTER__ENGINEER)
		set_bit(NOT_RECORD_ENGINEER, &track_status->not_record_reason);

	if (monitor->chg_cycle_status & CHG_CYCLE_VOTER__USER)
		set_bit(NOT_RECORD_SAFETY, &track_status->not_record_reason);

	if (monitor->slow_chg_enable)
		set_bit(NOT_RECORD_SLOW_CHG, &track_status->not_record_reason);

	if (monitor->plc_status == PLC_STATUS_ENABLE)
		set_bit(NOT_RECORD_PLC, &track_status->not_record_reason);

	return 0;
}

static int oplus_chg_track_speed_check(struct oplus_monitor *monitor)
{
	int ret = 0;
	int wls_break_work_delay_t;
	int wired_break_work_delay_t;
	int track_recording_time;
	static bool track_reset = true;
	static bool track_record_charger_info = false;
	struct oplus_chg_track_status *track_status;
	bool track_chg_is_no_charging,track_chg_speed_is_slow;
	int wls_connect_time = 0;
	union mms_msg_data data = { 0 };

	if (!g_track_chip)
		return -EFAULT;

	track_status = &g_track_chip->track_status;
	wls_break_work_delay_t = g_track_chip->track_cfg.wls_chg_break_t_thd +
				 TRACK_TIME_1000MS_JIFF_THD;
	wired_break_work_delay_t =
		g_track_chip->track_cfg.fast_chg_break_t_thd +
		TRACK_TIME_500MS_JIFF_THD;

	if (!monitor->wired_online && !monitor->wls_online) {
		if (track_status->power_info.power_type == TRACK_CHG_TYPE_WIRELESS &&
			track_status->wls_manu_break) {
				chg_info("wls_manu_break:%d\n", track_status->wls_manu_break);
			 	goto record_continue;
		}
		if (track_record_charger_info) {
			chg_info("record charger info and upload charger info\n");
			track_status->chg_plugout_utc_t =
				oplus_chg_track_get_current_time_s(
					&track_status->chg_plugout_rtc_t);
			oplus_chg_track_cal_app_stats(monitor, track_status);
			oplus_chg_track_cal_led_on_stats(monitor, track_status);
			oplus_chg_track_wls_cal_quiet_mode_stats(monitor, track_status);
			oplus_chg_track_cal_period_chg_average_speed(
				track_status, monitor);
			oplus_chg_track_cal_ledon_ledoff_average_speed(
				track_status);
			oplus_chg_track_hyper_speed_record(track_status);
			if (track_status->power_info.power_type ==
			    TRACK_CHG_TYPE_WIRELESS)
				track_status->wls_need_upload = true;
			else
				track_status->wired_need_upload = true;
			track_chg_is_no_charging = oplus_chg_track_is_no_charging(track_status);
			track_chg_speed_is_slow = oplus_chg_track_judge_speed_slow(monitor, g_track_chip);
			if (track_chg_is_no_charging) {
				oplus_chg_track_record_charger_info(monitor,
					&g_track_chip->no_charging_trigger, track_status);
				if (track_status->power_info.power_type == TRACK_CHG_TYPE_WIRELESS)
					schedule_delayed_work(&g_track_chip->no_charging_trigger_work,
						msecs_to_jiffies(wls_break_work_delay_t));
				else
					schedule_delayed_work(&g_track_chip->no_charging_trigger_work,
						msecs_to_jiffies(wired_break_work_delay_t));
			}

			if ((!track_chg_is_no_charging || g_track_chip->track_cfg.track_ver >= TRACK_VER_4_0) &&
			    track_chg_speed_is_slow) {
				oplus_chg_track_record_charger_info(monitor,
					&g_track_chip->slow_charging_trigger,track_status);
				if (track_status->power_info.power_type == TRACK_CHG_TYPE_WIRELESS)
					schedule_delayed_work(&g_track_chip->slow_charging_trigger_work,
						msecs_to_jiffies(wls_break_work_delay_t));
				else
					schedule_delayed_work(&g_track_chip->slow_charging_trigger_work,
						msecs_to_jiffies(wired_break_work_delay_t));
				oplus_chg_track_slowchg_info_reset(&g_track_chip->slowchg_info);

			}

			if (!track_chg_is_no_charging && !track_chg_speed_is_slow) {
				oplus_chg_track_record_charger_info(
					monitor, &g_track_chip->charger_info_trigger, track_status);
				if (track_status->power_info.power_type == TRACK_CHG_TYPE_WIRELESS)
					schedule_delayed_work(&g_track_chip->charger_info_trigger_work,
						msecs_to_jiffies(wls_break_work_delay_t));
				else
					schedule_delayed_work(&g_track_chip->charger_info_trigger_work,
						msecs_to_jiffies(wired_break_work_delay_t));
			}
			memset(track_status->bms_info, 0, TRACK_HIDL_BMS_INFO_LEN);
		}
		track_reset = true;
		track_record_charger_info = false;
		oplus_chg_track_cancel_cal_period_chg_capaticy(g_track_chip);
		oplus_chg_track_status_reset(track_status);
		oplus_chg_wls_track_status_reset(track_status);

		return ret;
	}

record_continue:
	if ((monitor->wired_online || monitor->wls_online) && track_reset) {
		track_reset = false;
		if(g_track_chip->track_cfg.track_ver < TRACK_VER_4_0)
			g_track_chip->no_charging_trigger.flag_reason = TRACK_NOTIFY_FLAG_NO_CHARGING;
		else
			g_track_chip->no_charging_trigger.flag_reason = TRACK_NOTIFY_FLAG_NO_CHARGING_IBAT_LOW;
		oplus_chg_track_status_reset_when_plugin(monitor, track_status);
		oplus_chg_wls_track_status_reset(track_status);
	}

	track_status->chg_end_time = oplus_chg_track_get_local_time_s();
	track_recording_time =
		track_status->chg_end_time - track_status->chg_start_time;
	if (monitor->wls_online)
		wls_connect_time = track_status->chg_end_time - track_status->wls_attach_time_ms / 1000;
	if (!track_record_charger_info &&
	    ((track_recording_time <= TRACK_CYCLE_RECORDIING_TIME_90S) ||
		(monitor->wls_online && wls_connect_time < TRACK_CYCLE_RECORDIING_TIME_90S)))
		oplus_chg_track_get_charger_type(monitor, track_status,
						 TRACK_CHG_GET_THTS_TIME_TYPE);

	oplus_chg_track_cal_tbatt_status(monitor);
	oplus_chg_track_cal_section_soc_inc_rm(monitor, track_status);
	oplus_chg_track_cal_batt_full_time(monitor, track_status);
	oplus_chg_track_cal_chg_common_mesg(monitor, track_status);
	oplus_chg_track_cal_cool_down_stats(monitor, track_status);
	oplus_chg_track_cal_no_charging_stats(monitor, track_status);
	oplus_chg_track_cal_app_stats(monitor, track_status);
	oplus_chg_track_cal_led_on_stats(monitor, track_status);
	oplus_chg_track_wls_cal_quiet_mode_stats(monitor, track_status);
	oplus_chg_track_check_chg_abnormal(monitor, track_status);
	oplus_chg_track_cal_rechg_counts(monitor, track_status);
	oplus_chg_track_cal_chg_end_soc(monitor, track_status);
	oplus_chg_track_cal_chg_temp(monitor, track_status);
	oplus_chg_track_cal_soc_time(monitor, track_status);
	oplus_chg_track_cal_hyper_speed_status(track_status);
	oplus_chg_track_no_record_reason(monitor, track_status);
	if(!strcmp(g_track_chip->slowchg_info.first_adapter_type, "unknow") &&
	    track_recording_time >= TRACK_CYCLE_RECORDIING_TIME_90S)
		strncpy(g_track_chip->slowchg_info.first_adapter_type,
			track_status->power_info.wired_info.adapter_type, OPLUS_CHG_TRACK_POWER_TYPE_LEN - 1);
	track_status->prop_status = monitor->batt_status;
	if (!track_record_charger_info &&
	    monitor->batt_status == POWER_SUPPLY_STATUS_FULL &&
	    track_recording_time < TRACK_CYCLE_RECORDIING_TIME_2MIN)
		oplus_chg_track_cancel_cal_period_chg_capaticy(g_track_chip);

	if (!track_record_charger_info &&
	    track_recording_time >= TRACK_CYCLE_RECORDIING_TIME_2MIN)
		track_record_charger_info = true;

	if (monitor->wls_topic)
		oplus_mms_get_item_data(monitor->wls_topic, WLS_ITEM_STATUS_KEEP, &data, true);
	if (monitor->wired_online || !data.intval)
		track_status->wls_manu_break = false;

	chg_debug("track_recording_time=%d, track_record_charger_info=%d\n",
		  track_recording_time, track_record_charger_info);

	return ret;
}

static int oplus_chg_track_uisoc_soc_jump_check(struct oplus_monitor *monitor)
{
	int ret = 0;
	int curr_time_utc;
	int curr_vbatt;
	struct oplus_chg_track_status *track_status;
	int judge_curr_soc = 0;
	struct rtc_time tm;
	int curr_fcc = 0, curr_rm = 0;
	int avg_current = 0;
	static int pre_local_time = 0;
	int curr_local_time = oplus_chg_track_get_local_time_s();

	if (!g_track_chip)
		return -EFAULT;

	curr_time_utc = oplus_chg_track_get_current_time_s(&tm);
	curr_vbatt = monitor->vbat_mv;
	curr_fcc = monitor->batt_fcc;
	curr_rm = monitor->batt_rm;
	track_status = &g_track_chip->track_status;
	if (track_status->curr_soc == -EINVAL) {
		track_status->curr_soc = monitor->batt_soc;
		track_status->pre_soc = monitor->batt_soc;
		track_status->curr_smooth_soc = monitor->smooth_soc;
		track_status->pre_smooth_soc = monitor->smooth_soc;
		track_status->curr_uisoc = monitor->ui_soc;
		track_status->pre_uisoc = monitor->ui_soc;
		track_status->pre_vbatt = curr_vbatt;
		track_status->pre_time_utc = curr_time_utc;
		track_status->pre_fcc = curr_fcc;
		track_status->pre_rm = curr_rm;
		track_status->soc_jump_pre_batt_temp = monitor->batt_temp;
		pre_local_time = curr_local_time;
		judge_curr_soc = track_status->curr_smooth_soc;
		if (monitor->soc_load >= 0 && abs(judge_curr_soc - monitor->soc_load) > OPLUS_CHG_TRACK_UI_SOC_LOAD_JUMP_THD) {
			track_status->uisoc_load_jumped = true;
			chg_info("The gap between loaded uisoc and soc is too large\n");
			memset(g_track_chip->uisoc_load_trigger.crux_info, 0,
			       sizeof(g_track_chip->uisoc_load_trigger.crux_info));
			ret = scnprintf(g_track_chip->uisoc_load_trigger.crux_info, OPLUS_CHG_TRACK_CURX_INFO_LEN,
				       "$$curr_uisoc@@%d$$curr_soc@@%d$$load_uisoc_soc_gap@@%d"
				       "$$pre_vbatt@@%d$$curr_vbatt@@%d"
				       "$$pre_time_utc@@%d$$curr_time_utc@@%d"
				       "$$charger_exist@@%d$$curr_smooth_soc@@%d"
				       "$$curr_fcc@@%d$$curr_rm@@%d$$current@@%d"
				       "$$soc_load@@%d$$judge_curr_soc@@%d",
				       track_status->curr_uisoc, track_status->curr_soc,
				       judge_curr_soc - monitor->soc_load, track_status->pre_vbatt, curr_vbatt,
				       track_status->pre_time_utc, curr_time_utc,
				       (monitor->wired_online || monitor->wls_online), track_status->curr_smooth_soc,
				       curr_fcc, curr_rm, monitor->ibat_ma, monitor->soc_load, judge_curr_soc);
			schedule_delayed_work(&g_track_chip->uisoc_load_trigger_work,
					      msecs_to_jiffies(TRACK_TIME_SCHEDULE_UI_SOC_LOAD_JUMP));
		}
	} else {
		track_status->curr_soc = track_status->debug_soc != OPLUS_CHG_TRACK_DEBUG_UISOC_SOC_INVALID ?
						 track_status->debug_soc :
						 monitor->batt_soc;
		track_status->curr_smooth_soc = track_status->debug_soc != OPLUS_CHG_TRACK_DEBUG_UISOC_SOC_INVALID ?
							track_status->debug_soc :
							monitor->smooth_soc;
		track_status->curr_uisoc = track_status->debug_uisoc != OPLUS_CHG_TRACK_DEBUG_UISOC_SOC_INVALID ?
						   track_status->debug_uisoc :
						   monitor->ui_soc;
	}

	if (curr_time_utc > track_status->pre_time_utc)
		avg_current = (track_status->pre_rm - curr_rm) / oplus_gauge_get_batt_num() * TRACK_TIME_1HOU_THD /
			      (curr_time_utc - track_status->pre_time_utc);

	if (!track_status->soc_jumped &&
	    abs(track_status->curr_soc - track_status->pre_soc) >= OPLUS_CHG_TRACK_SOC_JUMP_THD) {
		track_status->soc_jumped = true;
		chg_info("The gap between curr_soc and pre_soc is too large\n");
		memset(g_track_chip->soc_trigger.crux_info, 0, sizeof(g_track_chip->soc_trigger.crux_info));
		ret = scnprintf(g_track_chip->soc_trigger.crux_info, OPLUS_CHG_TRACK_CURX_INFO_LEN,
			       "$$curr_soc@@%d$$pre_soc@@%d$$curr_soc_pre_soc_gap@@%d"
			       "$$pre_vbatt@@%d$$curr_vbatt@@%d"
			       "$$pre_time_utc@@%d$$curr_time_utc@@%d$$kernel_diff_t@@%d"
			       "$$charger_exist@@%d$$avg_current@@%d$$current@@%d"
			       "$$pre_fcc@@%d$$pre_rm@@%d$$curr_fcc@@%d$$curr_rm@@%d"
			       "$$pre_batt_temp@@%d$$curr_batt_temp@@%d",
			       track_status->curr_soc, track_status->pre_soc,
			       track_status->curr_soc - track_status->pre_soc, track_status->pre_vbatt, curr_vbatt,
			       track_status->pre_time_utc, curr_time_utc, (curr_local_time - pre_local_time),
			       (monitor->wired_online || monitor->wls_online), avg_current, monitor->ibat_ma,
			       track_status->pre_fcc, track_status->pre_rm, curr_fcc, curr_rm,
			       track_status->soc_jump_pre_batt_temp, monitor->batt_temp);
		schedule_delayed_work(&g_track_chip->soc_trigger_work, 0);
	} else {
		if (track_status->soc_jumped && track_status->curr_soc == track_status->pre_soc)
			track_status->soc_jumped = false;
	}

	if (!track_status->uisoc_jumped &&
	    abs(track_status->curr_uisoc - track_status->pre_uisoc) > OPLUS_CHG_TRACK_UI_SOC_JUMP_THD) {
		track_status->uisoc_jumped = true;
		chg_info("The gap between curr_uisoc and pre_uisoc is too large\n");
		memset(g_track_chip->uisoc_trigger.crux_info, 0, sizeof(g_track_chip->uisoc_trigger.crux_info));
		ret = scnprintf(g_track_chip->uisoc_trigger.crux_info, OPLUS_CHG_TRACK_CURX_INFO_LEN,
			       "$$curr_uisoc@@%d$$pre_uisoc@@%d$$curr_uisoc_pre_uisoc_gap@@%d"
			       "$$pre_vbatt@@%d$$curr_vbatt@@%d"
			       "$$pre_time_utc@@%d$$curr_time_utc@@%d$$kernel_diff_t@@%d"
			       "$$charger_exist@@%d$$avg_current@@%d$$current@@%d"
			       "$$pre_fcc@@%d$$pre_rm@@%d$$curr_fcc@@%d$$curr_rm@@%d",
			       track_status->curr_uisoc, track_status->pre_uisoc,
			       track_status->curr_uisoc - track_status->pre_uisoc, track_status->pre_vbatt, curr_vbatt,
			       track_status->pre_time_utc, curr_time_utc, (curr_local_time - pre_local_time),
			       (monitor->wired_online || monitor->wls_online), avg_current, monitor->ibat_ma,
			       track_status->pre_fcc, track_status->pre_rm, curr_fcc, curr_rm);
		schedule_delayed_work(&g_track_chip->uisoc_trigger_work, 0);
	} else {
		if (track_status->uisoc_jumped && track_status->curr_uisoc == track_status->pre_uisoc)
			track_status->uisoc_jumped = false;
	}

	judge_curr_soc = track_status->curr_smooth_soc;

	if (!track_status->uisoc_to_soc_jumped && !track_status->uisoc_load_jumped &&
	    abs(track_status->curr_uisoc - judge_curr_soc) > OPLUS_CHG_TRACK_UI_SOC_TO_SOC_JUMP_THD) {
		track_status->uisoc_to_soc_jumped = true;
		chg_info("The gap between curr_uisoc and curr_soc is too large\n");
		memset(g_track_chip->uisoc_to_soc_trigger.crux_info, 0,
		       sizeof(g_track_chip->uisoc_to_soc_trigger.crux_info));
		ret = scnprintf(g_track_chip->uisoc_to_soc_trigger.crux_info, OPLUS_CHG_TRACK_CURX_INFO_LEN,
			       "$$curr_uisoc@@%d$$curr_soc@@%d$$curr_uisoc_curr_soc_gap@@%d"
			       "$$pre_vbatt@@%d$$curr_vbatt@@%d"
			       "$$pre_time_utc@@%d$$curr_time_utc@@%d$$kernel_diff_t@@%d"
			       "$$charger_exist@@%d$$curr_smooth_soc@@%d$$avg_current@@%d$$current@@%d"
			       "$$pre_fcc@@%d$$pre_rm@@%d$$curr_fcc@@%d$$curr_rm@@%d",
			       track_status->curr_uisoc, track_status->curr_soc,
			       track_status->curr_uisoc - judge_curr_soc, track_status->pre_vbatt, curr_vbatt,
			       track_status->pre_time_utc, curr_time_utc, (curr_local_time - pre_local_time),
			       (monitor->wired_online || monitor->wls_online), track_status->curr_smooth_soc,
			       avg_current, monitor->ibat_ma,
			       track_status->pre_fcc, track_status->pre_rm, curr_fcc, curr_rm);
		schedule_delayed_work(&g_track_chip->uisoc_to_soc_trigger_work, 0);
	} else {
		if (track_status->curr_uisoc == judge_curr_soc) {
			track_status->uisoc_to_soc_jumped = false;
			track_status->uisoc_load_jumped = false;
		}
	}

	chg_debug("debug_soc:0x%x, debug_uisoc:0x%x, pre_soc:%d, curr_soc:%d,"
		  "pre_uisoc:%d, curr_uisoc:%d, pre_smooth_soc:%d, curr_smooth_soc:%d\n",
		  track_status->debug_soc, track_status->debug_uisoc, track_status->pre_soc, track_status->curr_soc,
		  track_status->pre_uisoc, track_status->curr_uisoc, track_status->pre_smooth_soc,
		  track_status->curr_smooth_soc);

	track_status->pre_soc = track_status->curr_soc;
	track_status->pre_smooth_soc = track_status->curr_smooth_soc;
	track_status->pre_uisoc = track_status->curr_uisoc;
	track_status->pre_vbatt = curr_vbatt;
	track_status->pre_time_utc = curr_time_utc;
	track_status->pre_fcc = curr_fcc;
	track_status->pre_rm = curr_rm;
	track_status->soc_jump_pre_batt_temp = monitor->batt_temp;
	pre_local_time = curr_local_time;

	return ret;
}

static bool oplus_chg_track_rsoc_smooth_to_1_pct(struct oplus_chg_track_gauge_params *batt_params)
{
	bool ret = false;

	if (!batt_params)
		return ret;

	if (batt_params->soc <= 1 && batt_params->pre_soc > batt_params->soc)
		ret = true;
	chg_debug("ret=%d, pre_soc=%d, soc=%d\n", ret, batt_params->pre_soc, batt_params->soc);

	return ret;
}

static int oplus_chg_track_pack_gauge_info(struct kfifo *kfifo, u8 *crux_info, int len)
{
	int count;
	u8 *gauge_data;
	int index = 0;

	if (!kfifo)
		return -EINVAL;

	gauge_data = kmalloc(GAUGE_INFO_TRACK_FIFO_ONE_SIZE, GFP_KERNEL);
	if (!gauge_data) {
		chg_err("gauge_buf error\n");
		return -ENOMEM;
	}

	while (!kfifo_is_empty(kfifo)) {
		count = kfifo_out_spinlocked(kfifo, gauge_data, GAUGE_INFO_TRACK_FIFO_ONE_SIZE, &gauge_fifo_lock);
		if (count != GAUGE_INFO_TRACK_FIFO_ONE_SIZE) {
			chg_err("gauge_data size is error, count=%d\n", count);
			kfree(gauge_data);
			return -EINVAL;
		}
		chg_debug("len:%zu, reg_info:%s\n", strlen(gauge_data), gauge_data);
		index += scnprintf(crux_info + index, len - index,"%s", gauge_data);
		if (!kfifo_is_empty(kfifo))
			index += scnprintf(crux_info + index, len - index, "||");
	}

	kfree(gauge_data);
	return 0;
}

static int oplus_chg_track_gauge_info_record(
	struct oplus_chg_track_gauge_info *p_gauge_info, int err_type, int delta_time)
{
	int rc;
	int index = 0;
	int curr_time;
	struct rtc_time tm;
	char err_reason[OPLUS_CHG_TRACK_DEVICE_ERR_NAME_LEN] = {0};
	union mms_msg_data data = { 0 };

	if (!p_gauge_info)
		return -EINVAL;

	if (err_type <= TRACK_GAGUE_ERR_DEFAULT || err_type >= TRACK_GAGUE_ERR_MAX) {
		chg_info("err_type not match\n");
		return -EINVAL;
	}

	if (err_type == TRACK_GAGUE_GENERAL_INFO)
		oplus_mms_get_item_data(p_gauge_info->params.gauge_topic, GAUGE_ITEM_CALIB_TIME, &data, true);

	mutex_lock(&p_gauge_info->track_lock);
	curr_time = oplus_chg_track_get_current_time_s(&tm);
	if (curr_time - p_gauge_info->pre_upload_time > TRACK_GAUGE_UPLOAD_PERIOD)
		p_gauge_info->upload_count = 0;

	if (p_gauge_info->upload_count > TRACK_GAUGE_UPLOAD_COUNT_MAX) {
		chg_debug("uploading count arrive max\n");
		mutex_unlock(&p_gauge_info->track_lock);
		return 0;
	}

	if (p_gauge_info->load_trigger)
		kfree(p_gauge_info->load_trigger);
	p_gauge_info->load_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!p_gauge_info->load_trigger) {
		chg_err("gauge load_trigger memery alloc fail\n");
		mutex_unlock(&p_gauge_info->track_lock);
		return -ENOMEM;
	}

	if (err_type != TRACK_GAGUE_GENERAL_INFO && err_type != TRACK_GAGUE_SOC_1_PCT_INFO) {
		p_gauge_info->load_trigger->type_reason = TRACK_NOTIFY_TYPE_DEVICE_ABNORMAL;
		p_gauge_info->load_trigger->flag_reason = TRACK_NOTIFY_FLAG_GAGUE_ABNORMAL;
	} else {
		p_gauge_info->load_trigger->type_reason = TRACK_NOTIFY_TYPE_GENERAL_RECORD;
		p_gauge_info->load_trigger->flag_reason = TRACK_NOTIFY_FLAG_GAUGE_INFO;
	}
	p_gauge_info->upload_count++;
	p_gauge_info->pre_upload_time = curr_time;

	index += scnprintf(&(p_gauge_info->load_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$device_id@@%s", p_gauge_info->device_name);
	if (err_type != TRACK_GAGUE_SOC_1_PCT_INFO)
		index += scnprintf(&(p_gauge_info->load_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$err_scene@@%s", OPLUS_CHG_TRACK_SCENE_GAGUE_DEFAULT);
	else
		index += scnprintf(&(p_gauge_info->load_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$err_scene@@%s", OPLUS_CHG_TRACK_SCENE_GAGUE_SOC_1_PCT);

	oplus_chg_track_get_gague_err_reason(err_type, err_reason, sizeof(err_reason));
	index += scnprintf(&(p_gauge_info->load_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$err_reason@@%s", err_reason);

	if (err_type == TRACK_GAGUE_GENERAL_INFO)
		index += scnprintf(&(p_gauge_info->load_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$calib_t@@%s", data.strval);
	index += scnprintf(&(p_gauge_info->load_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			  "$$delta_time@@%d", delta_time);
	index += scnprintf(&(p_gauge_info->load_trigger->crux_info[index]), OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
			"$$reg_info@@");
	rc = oplus_chg_track_pack_gauge_info(&p_gauge_info->fifo, &(p_gauge_info->load_trigger->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index);
	if (!rc) {
		schedule_delayed_work(&p_gauge_info->load_trigger_work, 0);
		pr_debug("success\n");

	}

	return 0;
}

static void oplus_chg_track_gauge_info_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip =
		container_of(dwork, struct oplus_chg_track, gauge_info.load_trigger_work);

	if (chip->gauge_info.load_trigger) {
		oplus_chg_track_upload_trigger_data(chip->gauge_info.load_trigger);
		kfree(chip->gauge_info.load_trigger);
		chip->gauge_info.load_trigger = NULL;
	}
	mutex_unlock(&chip->gauge_info.track_lock);
}

static void oplus_chg_track_sub_gauge_info_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip =
		container_of(dwork, struct oplus_chg_track, sub_gauge_info.load_trigger_work);

	if (chip->sub_gauge_info.load_trigger) {
		oplus_chg_track_upload_trigger_data(chip->sub_gauge_info.load_trigger);
		kfree(chip->sub_gauge_info.load_trigger);
		chip->sub_gauge_info.load_trigger = NULL;
	}
	mutex_unlock(&chip->sub_gauge_info.track_lock);
}

static void oplus_chg_track_gauge_sili_alg_application_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip =
		container_of(dwork, struct oplus_chg_track, gauge_info.sili_alg_application_load_trigger_work);

	if (chip->gauge_info.sili_alg_application_load_trigger) {
		oplus_chg_track_upload_trigger_data(chip->gauge_info.sili_alg_application_load_trigger);
		kfree(chip->gauge_info.sili_alg_application_load_trigger);
		chip->gauge_info.sili_alg_application_load_trigger = NULL;
	}
	mutex_unlock(&chip->gauge_info.sili_alg_application_lock);
}

static void oplus_chg_track_sub_gauge_sili_alg_application_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip =
		container_of(dwork, struct oplus_chg_track, sub_gauge_info.sili_alg_application_load_trigger_work);

	if (chip->sub_gauge_info.sili_alg_application_load_trigger) {
		oplus_chg_track_upload_trigger_data(chip->sub_gauge_info.sili_alg_application_load_trigger);
		kfree(chip->sub_gauge_info.sili_alg_application_load_trigger);
		chip->sub_gauge_info.sili_alg_application_load_trigger = NULL;
	}
	mutex_unlock(&chip->sub_gauge_info.sili_alg_application_lock);
}

static void oplus_chg_track_gauge_sili_alg_monitor_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip =
		container_of(dwork, struct oplus_chg_track, gauge_info.sili_alg_monitor_load_trigger_work);

	if (chip->gauge_info.sili_alg_monitor_load_trigger) {
		oplus_chg_track_upload_trigger_data(chip->gauge_info.sili_alg_monitor_load_trigger);
		kfree(chip->gauge_info.sili_alg_monitor_load_trigger);
		chip->gauge_info.sili_alg_monitor_load_trigger = NULL;
	}
	mutex_unlock(&chip->gauge_info.sili_alg_monitor_lock);
}

static void oplus_chg_track_sub_gauge_sili_alg_monitor_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip =
		container_of(dwork, struct oplus_chg_track, sub_gauge_info.sili_alg_monitor_load_trigger_work);

	if (chip->sub_gauge_info.sili_alg_monitor_load_trigger) {
		oplus_chg_track_upload_trigger_data(chip->sub_gauge_info.sili_alg_monitor_load_trigger);
		kfree(chip->sub_gauge_info.sili_alg_monitor_load_trigger);
		chip->sub_gauge_info.sili_alg_monitor_load_trigger = NULL;
	}
	mutex_unlock(&chip->sub_gauge_info.sili_alg_monitor_lock);
}

static void oplus_chg_track_gauge_sili_alg_lifetime_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip =
		container_of(dwork, struct oplus_chg_track, gauge_info.sili_alg_lifetime_load_trigger_work);

	if (chip->gauge_info.sili_alg_lifetime_load_trigger) {
		oplus_chg_track_upload_trigger_data(chip->gauge_info.sili_alg_lifetime_load_trigger);
		kfree(chip->gauge_info.sili_alg_lifetime_load_trigger);
		chip->gauge_info.sili_alg_lifetime_load_trigger = NULL;
	}
	mutex_unlock(&chip->gauge_info.sili_alg_lifetime_lock);
}

static void oplus_chg_track_sub_gauge_sili_alg_lifetime_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip =
		container_of(dwork, struct oplus_chg_track, sub_gauge_info.sili_alg_lifetime_load_trigger_work);

	if (chip->sub_gauge_info.sili_alg_lifetime_load_trigger) {
		oplus_chg_track_upload_trigger_data(chip->sub_gauge_info.sili_alg_lifetime_load_trigger);
		kfree(chip->sub_gauge_info.sili_alg_lifetime_load_trigger);
		chip->sub_gauge_info.sili_alg_lifetime_load_trigger = NULL;
	}
	mutex_unlock(&chip->sub_gauge_info.sili_alg_lifetime_lock);
}

static void oplus_chg_track_gauge_batt_monitor_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip =
		container_of(dwork, struct oplus_chg_track, gauge_info.batt_monitor_load_trigger_work);

	if (chip->gauge_info.batt_monitor_load_trigger) {
		oplus_chg_track_upload_trigger_data(chip->gauge_info.batt_monitor_load_trigger);
		kfree(chip->gauge_info.batt_monitor_load_trigger);
		chip->gauge_info.batt_monitor_load_trigger = NULL;
	}
	mutex_unlock(&chip->gauge_info.batt_monitor_lock);
}

static void oplus_chg_track_sub_gauge_batt_monitor_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_track *chip =
		container_of(dwork, struct oplus_chg_track, sub_gauge_info.batt_monitor_load_trigger_work);

	if (chip->sub_gauge_info.batt_monitor_load_trigger) {
		oplus_chg_track_upload_trigger_data(chip->sub_gauge_info.batt_monitor_load_trigger);
		kfree(chip->sub_gauge_info.batt_monitor_load_trigger);
		chip->sub_gauge_info.batt_monitor_load_trigger = NULL;
	}
	mutex_unlock(&chip->sub_gauge_info.batt_monitor_lock);
}

static int oplus_chg_track_gauge_fifo_push(struct kfifo *kfifo, struct oplus_mms *gauge_topic)
{
	int ret = 0;
	int count;
	u8 *gauge_data;
	struct rtc_time tm;
	union mms_msg_data data = { 0 };

	if (!kfifo)
		return ret;

	oplus_chg_track_get_current_time(&tm);
	gauge_data = kzalloc(GAUGE_INFO_TRACK_FIFO_ONE_SIZE, GFP_KERNEL);
	if (!gauge_data) {
		chg_err("gauge_buf error\n");
		return -ENOMEM;
	}

	if (kfifo_is_full(kfifo)) {
		count = kfifo_out_spinlocked(kfifo, gauge_data, GAUGE_INFO_TRACK_FIFO_ONE_SIZE, &gauge_fifo_lock);
		if (count != GAUGE_INFO_TRACK_FIFO_ONE_SIZE) {
			chg_err("gauge_data size is error\n");
			kfree(gauge_data);
			return -EINVAL;
		}
	}

	memset(gauge_data, 0, GAUGE_INFO_TRACK_FIFO_ONE_SIZE);
	ret = oplus_mms_get_item_data(gauge_topic, GAUGE_ITEM_REG_INFO, &data, true);
	if (ret == 0 && data.strval && strlen(data.strval)) {
		ret += scnprintf(&gauge_data[ret], GAUGE_INFO_TRACK_FIFO_ONE_SIZE - ret,
			"time[%04d-%02d-%02d %02d:%02d:%02d]-%s",
			tm.tm_year, tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, data.strval);
	} else {
		kfree(gauge_data);
		return -EINVAL;
	}

	count = kfifo_in_spinlocked(kfifo, gauge_data, GAUGE_INFO_TRACK_FIFO_ONE_SIZE, &gauge_fifo_lock);
	if (count != GAUGE_INFO_TRACK_FIFO_ONE_SIZE) {
		chg_err("gauge kfifo in error\n");
		kfree(gauge_data);
		return -EINVAL;
	}

	kfree(gauge_data);
	return 0;
}

static bool oplus_chg_track_judge_rsoc_smooth(struct oplus_chg_track_gauge_params *batt_params)
{
	int rc;
	bool ret = false;
	struct rtc_time tm;
	int curr_time;
	int batt_curr;
	union mms_msg_data data = { 0 };

	if (!batt_params)
		return ret;

	curr_time = oplus_chg_track_get_current_time_s(&tm);
	if (curr_time < TRACK_VALID_UTC_MIN_TIME)
		return ret;

	rc = oplus_mms_get_item_data(batt_params->gauge_topic, GAUGE_ITEM_CURR, &data, true);
	if (rc < 0)
		return ret;
	batt_curr = data.intval;

	if (!batt_params->batt_curr || (batt_params->batt_curr > 0 && batt_curr <= 0) ||
	    (batt_params->batt_curr < 0 && batt_curr > 0)) {
		batt_params->rsoc_smooth_base_time = curr_time;
		batt_params->rsoc_smooth_base_soc = batt_params->soc;
	}

	if (batt_curr > 0 &&
	    curr_time - batt_params->rsoc_smooth_base_time > OPLUS_CHG_TRACK_TIME_THD_S(300)) {
		if (batt_params->soc > batt_params->rsoc_smooth_base_soc)
			ret = true;
		batt_params->rsoc_smooth_base_time = curr_time;
		batt_params->rsoc_smooth_base_soc = batt_params->soc;
	} else if (batt_curr < 0 &&
	    curr_time - batt_params->rsoc_smooth_base_time > OPLUS_CHG_TRACK_TIME_THD_S(300)) {
		if (batt_params->soc < batt_params->rsoc_smooth_base_soc)
			ret = true;
		batt_params->rsoc_smooth_base_time = curr_time;
		batt_params->rsoc_smooth_base_soc = batt_params->soc;
	}

	batt_params->batt_curr = batt_curr;

	return ret;
}

static bool oplus_chg_track_judge_volt_soc_match(
		struct oplus_chg_track *track_chip,  struct oplus_chg_track_gauge_info *batt_info)
{
	bool ret = false;
	int curr_time;
	struct rtc_time tm;
	struct oplus_monitor *monitor;
	struct oplus_chg_track_gauge_params *batt_params;

	if (!track_chip || !batt_info)
		return false;

	batt_params = &(batt_info->params);
	monitor = track_chip->monitor;
	curr_time = oplus_chg_track_get_current_time_s(&tm);
	if (curr_time < TRACK_VALID_UTC_MIN_TIME)
		return ret;

	if (monitor->wired_online || monitor->wls_online) {
		if (monitor->ffc_status != FFC_DEFAULT &&
		    batt_params->soc < OPLUS_CHG_TRACK_SOC_THD(80))
			ret = true;
		batt_info->plugout_t = 0;
		return ret;
	}

	if (!batt_info->plugout_t)
		batt_info->plugout_t = curr_time;

	if (!batt_params->batt_volt)
		return ret;

	if (curr_time - batt_info->plugout_t > OPLUS_CHG_TRACK_TIME_THD_S(1800) &&
	    ((batt_params->soc <= OPLUS_CHG_TRACK_SOC_THD(40) &&
	    batt_params->batt_volt >= OPLUS_CHG_TRACK_BATT_VOL_MV(4000)) ||
	    (batt_params->soc >= OPLUS_CHG_TRACK_SOC_THD(40) &&
	    batt_params->batt_volt <= OPLUS_CHG_TRACK_BATT_VOL_MV(3600))))
		ret = true;

	if (ret)
		chg_info("ret:%d, plugout_t:%d, curr_time:%d, soc:%d, batt_volt:%d\n",
			ret, batt_info->plugout_t, curr_time, batt_params->soc, batt_params->batt_volt);

	return ret;
}

static bool oplus_chg_track_judge_fcc(
				struct oplus_chg_track *track_chip, int fcc_ref,
				struct oplus_chg_track_gauge_params *batt_params)
{
	struct oplus_monitor *monitor;

	if (!track_chip || !batt_params || !fcc_ref)
		return false;

	monitor = track_chip->monitor;
	if (monitor->temp_region != TEMP_REGION_NORMAL &&
	    monitor->temp_region != TEMP_REGION_NORMAL_HIGH &&
	    monitor->temp_region != TEMP_REGION_WARM)
		return false;

	if (batt_params->fcc && (batt_params->fcc < fcc_ref * OPLUS_CHG_TRACK_PCT_THD(80) ||
	    batt_params->fcc > fcc_ref * OPLUS_CHG_TRACK_PCT_THD(120)))
		return true;

	return false;
}

static bool oplus_chg_track_judge_qmax(
				struct oplus_chg_track *track_chip, int qmax_ref,
				struct oplus_chg_track_gauge_params *batt_params)
{
	struct oplus_monitor *monitor;

	if (!track_chip || !batt_params || !qmax_ref)
		return false;

	monitor = track_chip->monitor;
	if (monitor->temp_region != TEMP_REGION_NORMAL &&
	    monitor->temp_region != TEMP_REGION_NORMAL_HIGH &&
	    monitor->temp_region != TEMP_REGION_WARM)
		return false;

	if (batt_params->qmax && (batt_params->qmax < qmax_ref * OPLUS_CHG_TRACK_PCT_THD(80) ||
	    batt_params->qmax > qmax_ref * OPLUS_CHG_TRACK_PCT_THD(120)))
		return true;

	return false;
}

static int oplus_chg_track_get_gauge_status(struct oplus_chg_track *track_chip,
					    struct oplus_chg_track_gauge_info *batt_info,
					    struct oplus_chg_track_gauge_params *batt_params)
{
	int ret = 0;
	int delta_time;
	struct rtc_time tm;
	bool record_reg_info = false;
	int err_type = TRACK_GAGUE_ERR_DEFAULT;
	int curr_time = oplus_chg_track_get_current_time_s(&tm);

	if (abs(batt_params->pre_soc - batt_params->soc) >= OPLUS_CHG_TRACK_SOC_JUMP_THD)
		err_type = TRACK_GAGUE_ERR_RSOC_JUMP;
	else if (oplus_chg_track_judge_rsoc_smooth(batt_params))
		err_type = TRACK_GAGUE_ERR_RSOC_SMOOTH;
	else if (!test_bit(TRACK_GAGUE_ERR_VOLT_SOC_NOT_MATCH, &batt_info->trigger_type_flag) &&
		 oplus_chg_track_judge_volt_soc_match(track_chip, batt_info))
		err_type = TRACK_GAGUE_ERR_VOLT_SOC_NOT_MATCH;
	else if (!test_bit(TRACK_GAGUE_ERR_QMAX, &batt_info->trigger_type_flag) &&
		 oplus_chg_track_judge_qmax(track_chip, batt_info->nominal_qmax, batt_params))
		err_type = TRACK_GAGUE_ERR_QMAX;
	else if (!test_bit(TRACK_GAGUE_ERR_FCC, &batt_info->trigger_type_flag) &&
		 oplus_chg_track_judge_fcc(track_chip, batt_info->nominal_fcc, batt_params))
		err_type = TRACK_GAGUE_ERR_FCC;
	else if (batt_params->soh && abs(batt_params->pre_soh - batt_params->soh) >= OPLUS_CHG_TRACK_SOH_THD(3))
		err_type = TRACK_GAGUE_ERR_SOH_JUMP;
	else if (abs(batt_params->pre_cc - batt_params->cc) >= OPLUS_CHG_TRACK_CC_THD(3))
		err_type = TRACK_GAGUE_ERR_CC_JUMP;
	else if (!test_bit(TRACK_GAGUE_ERR_TEMP, &batt_info->trigger_type_flag) &&
		 batt_params->batt_temp <= OPLUS_CHG_TRACK_TEMP_THD(-400))
		err_type = TRACK_GAGUE_ERR_TEMP;
	else if (oplus_chg_track_rsoc_smooth_to_1_pct(batt_params))
		err_type = TRACK_GAGUE_SOC_1_PCT_INFO;

	if (batt_info->pre_time == 0 || curr_time < TRACK_VALID_UTC_MIN_TIME) {
		batt_info->pre_time = curr_time;
		batt_info->pre_check_time = curr_time;
	}

	if (batt_info->debug_err_type && !test_bit(batt_info->debug_err_type, &batt_info->trigger_type_flag))
		err_type = batt_info->debug_err_type;

	if (err_type != TRACK_GAGUE_ERR_DEFAULT)
		record_reg_info = true;

	if (abs(batt_params->soc - batt_params->pre_record_soc) >= OPLUS_CHG_TRACK_SOC_THD(25) ||
	    (batt_params->soc == OPLUS_CHG_TRACK_SOC_THD(100) && (batt_params->soc != batt_params->pre_record_soc)) ||
	    (batt_info->debug_soc_record_thd &&
	     abs(batt_params->soc - batt_params->pre_record_soc) >= batt_info->debug_soc_record_thd)) {
		record_reg_info = true;
		batt_params->pre_record_soc = batt_params->soc;
	}

	if (curr_time - batt_info->pre_check_time > TRACK_GAUGE_UPLOAD_PERIOD ||
	    (batt_info->debug_upload_period_t &&
	     curr_time - batt_info->pre_check_time > batt_info->debug_upload_period_t)) {
		batt_info->pre_check_time = curr_time;
		record_reg_info = true;
		batt_info->trigger_type_flag = 0;
		if (!err_type)
			err_type = TRACK_GAGUE_GENERAL_INFO;
	}

	if (err_type)
		set_bit(err_type, &batt_info->trigger_type_flag);

	delta_time = curr_time - batt_info->pre_time;

	if (record_reg_info)
		oplus_chg_track_gauge_fifo_push(&batt_info->fifo, batt_params->gauge_topic);

	if (err_type)
		oplus_chg_track_gauge_info_record(batt_info, err_type, delta_time);

	batt_info->pre_time = curr_time;

	return ret;
}

static int oplus_chg_track_gauge_sili_alg_application_info_record(
			struct oplus_chg_track_gauge_info *p_gauge_info, int pre_sys_term_volt)
{
	int index = 0;

	if (!p_gauge_info)
		return -EINVAL;

	mutex_lock(&p_gauge_info->sili_alg_application_lock);
	if (p_gauge_info->sili_alg_application_load_trigger)
		kfree(p_gauge_info->sili_alg_application_load_trigger);
	p_gauge_info->sili_alg_application_load_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!p_gauge_info->sili_alg_application_load_trigger) {
		pr_err("gauge load_trigger memery alloc fail\n");
		mutex_unlock(&p_gauge_info->sili_alg_application_lock);
		return -ENOMEM;
	}

	p_gauge_info->sili_alg_application_load_trigger->type_reason = TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	p_gauge_info->sili_alg_application_load_trigger->flag_reason = TRACK_NOTIFY_FLAG_GAUGE_INFO;

	index += scnprintf(&(p_gauge_info->sili_alg_application_load_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$device_id@@%s", p_gauge_info->device_name);
	index += scnprintf(&(p_gauge_info->sili_alg_application_load_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$err_scene@@application");
	index += scnprintf(&(p_gauge_info->sili_alg_application_load_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$sys_term_volt@@%d,%d",
		pre_sys_term_volt, p_gauge_info->params.sili_application_sys_term_volt);
	index += scnprintf(&(p_gauge_info->sili_alg_application_load_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$cc@@%d, %d",
		p_gauge_info->params.pre_cc, p_gauge_info->params.cc);
	index += scnprintf(&(p_gauge_info->sili_alg_application_load_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$reg_info@@");

	oplus_gauge_get_sili_alg_application_info(p_gauge_info->params.gauge_topic,
		&(p_gauge_info->sili_alg_application_load_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index);

	schedule_delayed_work(&p_gauge_info->sili_alg_application_load_trigger_work, 0);

	return 0;
}

static int oplus_chg_track_get_gauge_sili_alg_application_status(
			struct oplus_chg_track *track_chip, struct oplus_chg_track_gauge_info *batt_info,
			struct oplus_chg_track_gauge_params *batt_params)
{
	bool record = false;
	bool sili_ic_alg_dsg_enable = false;
	int sili_ic_alg_term_volt = 0;
	union mms_msg_data data = { 0 };
	int pre_sys_term_volt = batt_params->sili_application_sys_term_volt;

	oplus_mms_get_item_data(batt_params->gauge_topic, GAUGE_ITEM_SILI_IC_ALG_DSG_ENABLE, &data, true);
	sili_ic_alg_dsg_enable = data.intval;
	if (!sili_ic_alg_dsg_enable) {
		batt_info->check_alg_application = false;
		return -EINVAL;
	}

	oplus_mms_get_item_data(batt_params->gauge_topic, GAUGE_ITEM_SILI_IC_ALG_TERM_VOLT, &data, true);
	sili_ic_alg_term_volt = data.intval;
	if (!sili_ic_alg_term_volt) {
		batt_info->check_alg_application = false;
		return -EINVAL;
	}

	pr_info("soc[%d, %d], cc[%d, %d], sys_term_volt[%d, %d], check_alg_application[%d]\n",
		batt_params->soc, batt_params->pre_soc, batt_params->cc, batt_params->pre_cc,
		batt_params->sili_application_sys_term_volt, sili_ic_alg_term_volt, batt_info->check_alg_application);
	if (!batt_info->check_alg_application && batt_params->soc <= 10 && batt_params->pre_soc > batt_params->soc) {
		batt_info->check_alg_application = true;
		if (batt_params->sili_application_sys_term_volt != sili_ic_alg_term_volt)
			record = true;
		batt_params->sili_application_sys_term_volt = sili_ic_alg_term_volt;
	}

	if (batt_info->check_alg_application &&
	    batt_params->soc > OPLUS_CHG_TRACK_SOC_THD(10) &&
	    batt_params->pre_soc < batt_params->soc)
		batt_info->check_alg_application = false;

	if (batt_params->pre_cc != batt_params->cc &&
	   (!batt_params->cc || !(batt_params->cc % OPLUS_CHG_TRACK_SOC_THD(10))))
		record = true;

	if (record)
		oplus_chg_track_gauge_sili_alg_application_info_record(batt_info, pre_sys_term_volt);

	return 0;
}

static void oplus_chg_track_get_gauge_sili_single_dischg_term_volt_status(
		struct oplus_chg_track *track_chip, struct oplus_chg_track_gauge_info *batt_info,
		struct oplus_chg_track_gauge_params *batt_params, int *err_type)
{
	bool charger_exist = (track_chip->monitor->wired_online || track_chip->monitor->wls_online);

	if ((batt_params->pre_batt_temp > SILI_ALG_MONITOR_BATT_TEMP_THR &&
	    batt_params->batt_temp < SILI_ALG_MONITOR_BATT_TEMP_THR) ||
	    (batt_params->pre_batt_temp < SILI_ALG_MONITOR_BATT_TEMP_THR &&
	    batt_params->batt_temp > SILI_ALG_MONITOR_BATT_TEMP_THR))
			batt_params->pre_sili_monitor_sys_term_volt = 0;

	if (batt_params->pre_sili_monitor_sys_term_volt &&
	    abs(batt_params->pre_sili_monitor_sys_term_volt - batt_params->sili_monitor_sys_term_volt) >
	    SILI_ALG_MONITOR_SYS_TERM_VOLT_DIFF)
		*err_type = TRACK_GAGUE_ERR_SINGLE_DISCHG_TERM_VOLT;

	if (!batt_info->single_dischg_check &&
	   (batt_params->batt_temp > SILI_ALG_MONITOR_FIRMWARE_TERM_VOLT_TEMP_THR &&
	    batt_params->sili_monitor_sys_term_volt < track_chip->track_cfg.gauge_firmware_term_volt)) {
		*err_type = TRACK_GAGUE_ERR_BELOW_FIRMWARE_TERM_VOLT;
		batt_info->single_dischg_check = true;
	}

	if (charger_exist)
		batt_info->single_dischg_check = false;

	pr_info("sys_term_volt[%d, %d], batt_temp[%d, %d], err_type[%d]\n",
		batt_params->pre_sili_monitor_sys_term_volt, batt_params->sili_monitor_sys_term_volt,
		batt_params->pre_batt_temp, batt_params->batt_temp, *err_type);
}

static bool oplus_chg_track_multiple_dischg_term_volt_over(
				int cmp_volt_1, int cmp_volt_2)
{
	bool ret = false;

	if (cmp_volt_1 && cmp_volt_2 &&
	    abs(cmp_volt_1 - cmp_volt_2) > SILI_ALG_MONITOR_SYS_TERM_VOLT_DIFF)
		ret = true;

	return ret;
}

static void oplus_chg_track_get_gauge_sili_multiple_dischg_term_volt_status(
		struct oplus_chg_track *track_chip, struct oplus_chg_track_gauge_info *batt_info,
		struct oplus_chg_track_gauge_params *batt_params, int *err_type)
{
	bool over;
	int temp_inr;
	int ui_soc;
	union mms_msg_data data = { 0 };
	bool charger_exist = (track_chip->monitor->wired_online || track_chip->monitor->wls_online);

	if (!batt_params->sili_monitor_sys_term_volt)
		return;

	oplus_mms_get_item_data(track_chip->monitor->comm_topic, COMM_ITEM_UI_SOC, &data, false);
	ui_soc = data.intval;

	if (batt_params->batt_temp > SILI_ALG_MONITOR_BATT_TEMP_THR)
		temp_inr = TRACK_SYS_TERM_VOL_TEMP_HIGH;
	else
		temp_inr = TRACK_SYS_TERM_VOL_TEMP_LOW;

	if (!batt_params->sys_term_vol_maximum[temp_inr].min_volt ||
	    batt_params->sili_monitor_sys_term_volt < batt_params->sys_term_vol_maximum[temp_inr].min_volt) {
		batt_params->sys_term_vol_maximum[temp_inr].min_volt = batt_params->sili_monitor_sys_term_volt;
		batt_params->sys_term_vol_maximum[temp_inr].min_point_batt_soc = batt_params->soc;
		batt_params->sys_term_vol_maximum[temp_inr].min_point_batt_temp = batt_params->batt_temp;
	}

	if (!batt_params->sys_term_vol_maximum[temp_inr].max_volt ||
	    batt_params->sili_monitor_sys_term_volt > batt_params->sys_term_vol_maximum[temp_inr].max_volt) {
		batt_params->sys_term_vol_maximum[temp_inr].max_volt = batt_params->sili_monitor_sys_term_volt;
		batt_params->sys_term_vol_maximum[temp_inr].max_point_batt_soc = batt_params->soc;
		batt_params->sys_term_vol_maximum[temp_inr].max_point_batt_temp = batt_params->batt_temp;
	}

	if ((!batt_info->charger_exist && charger_exist) ||
	    (batt_params->ui_soc == OPLUS_CHG_TRACK_SOC_THD(1) && batt_params->ui_soc != ui_soc)) {
	    	pr_info("enter\n");
		over = oplus_chg_track_multiple_dischg_term_volt_over(
			batt_params->sys_term_vol_maximum[TRACK_SYS_TERM_VOL_TEMP_LOW].max_volt,
			batt_params->pre_sys_term_vol_maximum[TRACK_SYS_TERM_VOL_TEMP_LOW].max_volt);
		if (over)
			*err_type = TRACK_GAGUE_ERR_MULTIPLE_DISCHG_TERM_VOLT;

		over = oplus_chg_track_multiple_dischg_term_volt_over(
			batt_params->sys_term_vol_maximum[TRACK_SYS_TERM_VOL_TEMP_LOW].min_volt,
			batt_params->pre_sys_term_vol_maximum[TRACK_SYS_TERM_VOL_TEMP_LOW].min_volt);
		if (over)
			*err_type = TRACK_GAGUE_ERR_MULTIPLE_DISCHG_TERM_VOLT;

		over = oplus_chg_track_multiple_dischg_term_volt_over(
			batt_params->sys_term_vol_maximum[TRACK_SYS_TERM_VOL_TEMP_HIGH].max_volt,
			batt_params->pre_sys_term_vol_maximum[TRACK_SYS_TERM_VOL_TEMP_HIGH].max_volt);
		if (over)
			*err_type = TRACK_GAGUE_ERR_MULTIPLE_DISCHG_TERM_VOLT;

		over = oplus_chg_track_multiple_dischg_term_volt_over(
			batt_params->sys_term_vol_maximum[TRACK_SYS_TERM_VOL_TEMP_HIGH].min_volt,
			batt_params->pre_sys_term_vol_maximum[TRACK_SYS_TERM_VOL_TEMP_HIGH].min_volt);
		if (over)
			*err_type = TRACK_GAGUE_ERR_MULTIPLE_DISCHG_TERM_VOLT;
	}

	chg_info("batt_temp=%d, temp_inr=%d, err_type=%d, sys_term_vol_maximum=[%d, %d, %d, %d], \
		pre_sys_term_vol_maximum=[%d, %d, %d, %d]\n",
		batt_params->batt_temp, temp_inr, *err_type,
		batt_params->sys_term_vol_maximum[TRACK_SYS_TERM_VOL_TEMP_LOW].max_volt,
		batt_params->sys_term_vol_maximum[TRACK_SYS_TERM_VOL_TEMP_LOW].min_volt,
		batt_params->sys_term_vol_maximum[TRACK_SYS_TERM_VOL_TEMP_HIGH].max_volt,
		batt_params->sys_term_vol_maximum[TRACK_SYS_TERM_VOL_TEMP_HIGH].min_volt,
		batt_params->pre_sys_term_vol_maximum[TRACK_SYS_TERM_VOL_TEMP_LOW].max_volt,
		batt_params->pre_sys_term_vol_maximum[TRACK_SYS_TERM_VOL_TEMP_LOW].min_volt,
		batt_params->pre_sys_term_vol_maximum[TRACK_SYS_TERM_VOL_TEMP_HIGH].max_volt,
		batt_params->pre_sys_term_vol_maximum[TRACK_SYS_TERM_VOL_TEMP_HIGH].min_volt);
}

static int oplus_chg_track_get_gauge_sili_cc_term_volt_status(
		struct oplus_chg_track *track_chip, struct oplus_chg_track_gauge_params *batt_params, int *err_type)
{
	int i;
	int term_volt_ref = 0;

	if (!track_chip->track_cfg.sili_cc_term_vol_curve.nums)
		return term_volt_ref;

	for (i = 0; i < track_chip->track_cfg.sili_cc_term_vol_curve.nums; i++) {
		if (batt_params->cc < track_chip->track_cfg.sili_cc_term_vol_curve.limits[i].cc)
			break;
	}

	if (i == track_chip->track_cfg.sili_cc_term_vol_curve.nums)
		i = track_chip->track_cfg.sili_cc_term_vol_curve.nums - 1;
	term_volt_ref = track_chip->track_cfg.sili_cc_term_vol_curve.limits[i].term_volt;

	if (batt_params->pre_sili_monitor_sys_term_volt && term_volt_ref < batt_params->sili_monitor_sys_term_volt &&
	    batt_params->pre_sili_monitor_sys_term_volt != batt_params->sili_monitor_sys_term_volt)
		*err_type = TRACK_GAGUE_ERR_CC_TERM_VOLT;

	pr_info("term_volt_ref=%d, cc=%d,  sys_term_volt[%d, %d], err_type=%d\n",
		term_volt_ref, batt_params->cc, batt_params->pre_sili_monitor_sys_term_volt,
		batt_params->sili_monitor_sys_term_volt, *err_type);

	return term_volt_ref;
}

static int oplus_chg_track_gauge_sili_alg_monitor_record(
	struct oplus_chg_track_gauge_info *p_gauge_info, int err_type, int term_volt_ref)
{
	int i;
	char *p_temp;
	int index = 0;
	int curr_time;
	struct rtc_time tm;
	static int upload_count = 0;
	static int pre_upload_time = 0;
	char err_reason[OPLUS_CHG_TRACK_DEVICE_ERR_NAME_LEN] = { 0 };

	if (!p_gauge_info)
		return -EINVAL;

	if (err_type <= TRACK_GAGUE_ERR_DEFAULT || err_type >= TRACK_GAGUE_ERR_MAX) {
		pr_info("err_type not match\n");
		return -EINVAL;
	}

	mutex_lock(&p_gauge_info->sili_alg_monitor_lock);
	curr_time = oplus_chg_track_get_current_time_s(&tm);
	if (curr_time - pre_upload_time > TRACK_GAUGE_UPLOAD_PERIOD)
		upload_count = 0;

	if (upload_count > TRACK_GAUGE_UPLOAD_COUNT_MAX) {
		pr_debug("uploading count arrive max\n");
		mutex_unlock(&p_gauge_info->sili_alg_monitor_lock);
		return 0;
	}

	if (p_gauge_info->sili_alg_monitor_load_trigger)
		kfree(p_gauge_info->sili_alg_monitor_load_trigger);
	p_gauge_info->sili_alg_monitor_load_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!p_gauge_info->sili_alg_monitor_load_trigger) {
		pr_err("gauge load_trigger memery alloc fail\n");
		mutex_unlock(&p_gauge_info->sili_alg_monitor_lock);
		return -ENOMEM;
	}

	if (err_type != TRACK_GAGUE_TERM_VOLT_OK) {
		p_gauge_info->sili_alg_monitor_load_trigger->type_reason = TRACK_NOTIFY_TYPE_DEVICE_ABNORMAL;
		p_gauge_info->sili_alg_monitor_load_trigger->flag_reason = TRACK_NOTIFY_FLAG_GAGUE_ABNORMAL;
	} else {
		p_gauge_info->sili_alg_monitor_load_trigger->type_reason = TRACK_NOTIFY_TYPE_GENERAL_RECORD;
		p_gauge_info->sili_alg_monitor_load_trigger->flag_reason = TRACK_NOTIFY_FLAG_GAUGE_INFO;
	}
	upload_count++;
	pre_upload_time = curr_time;
	index += scnprintf(&(p_gauge_info->sili_alg_monitor_load_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$device_id@@%s", p_gauge_info->device_name);
	if (err_type != TRACK_GAGUE_TERM_VOLT_OK)
		index +=
			scnprintf(&(p_gauge_info->sili_alg_monitor_load_trigger->crux_info[index]),
				OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$err_scene@@%s",
				OPLUS_CHG_TRACK_SCENE_GAGUE_DEFAULT);
	else
		index +=
			scnprintf(&(p_gauge_info->sili_alg_monitor_load_trigger->crux_info[index]),
				OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				 "$$err_scene@@%s", OPLUS_CHG_TRACK_SCENE_GAGUE_TERM_VOLT_OK);

	oplus_chg_track_get_gague_err_reason(err_type, err_reason, sizeof(err_reason));
	index += scnprintf(&(p_gauge_info->sili_alg_monitor_load_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$err_reason@@%s", err_reason);
	index += scnprintf(&(p_gauge_info->sili_alg_monitor_load_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$batt_temp@@%d", p_gauge_info->params.batt_temp);
	index += scnprintf(&(p_gauge_info->sili_alg_monitor_load_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$cc_term_volt_ref@@%d,%d",
		p_gauge_info->params.cc, term_volt_ref);
	index += scnprintf(&(p_gauge_info->sili_alg_monitor_load_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$sys_term_volt@@%d,%d",
		p_gauge_info->params.pre_sili_monitor_sys_term_volt, p_gauge_info->params.sili_monitor_sys_term_volt);
	index += scnprintf(&(p_gauge_info->sili_alg_monitor_load_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$term_vol_maximum@@");
	p_temp = (char *)p_gauge_info->params.sys_term_vol_maximum;
	for (i = 0; i < sizeof(p_gauge_info->params.sys_term_vol_maximum); i += sizeof(int))
		index += scnprintf(&(p_gauge_info->sili_alg_monitor_load_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "%d,", *(int *)(p_temp + i));
	index += scnprintf(&(p_gauge_info->sili_alg_monitor_load_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "||");
	p_temp = (char *)p_gauge_info->params.pre_sys_term_vol_maximum;
	for (i = 0; i < sizeof(p_gauge_info->params.pre_sys_term_vol_maximum); i += sizeof(int))
		index += scnprintf(&(p_gauge_info->sili_alg_monitor_load_trigger->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "%d,", *(int *)(p_temp + i));

	schedule_delayed_work(&p_gauge_info->sili_alg_monitor_load_trigger_work, 0);

	return 0;
}

static int oplus_chg_track_get_gauge_sili_alg_monitor_status(
	struct oplus_chg_track *track_chip, struct oplus_chg_track_gauge_info *batt_info,
	struct oplus_chg_track_gauge_params *batt_params)
{
	bool charger_exist;
	int ui_soc;
	int term_volt_ref;
	int err_type = TRACK_GAGUE_ERR_DEFAULT;
	bool sili_ic_alg_dsg_enable = false;
	int sili_ic_alg_term_volt = 0;
	union mms_msg_data data = { 0 };

	oplus_mms_get_item_data(batt_params->gauge_topic, GAUGE_ITEM_SILI_IC_ALG_DSG_ENABLE, &data, true);
	sili_ic_alg_dsg_enable = data.intval;

	if (!sili_ic_alg_dsg_enable)
		return -EINVAL;

	oplus_mms_get_item_data(batt_params->gauge_topic, GAUGE_ITEM_SILI_IC_ALG_TERM_VOLT, &data, false);
	sili_ic_alg_term_volt = data.intval;
	oplus_mms_get_item_data(track_chip->monitor->comm_topic, COMM_ITEM_UI_SOC, &data, false);
	ui_soc = data.intval;
	charger_exist = (track_chip->monitor->wired_online || track_chip->monitor->wls_online);

	if (ui_soc > OPLUS_CHG_TRACK_SOC_THD(15) ||
	   (batt_info->charger_exist && charger_exist)) {
		batt_params->pre_sili_monitor_sys_term_volt = 0;
		if (batt_params->sys_term_vol_maximum[TRACK_SYS_TERM_VOL_TEMP_HIGH].max_volt ||
		    batt_params->sys_term_vol_maximum[TRACK_SYS_TERM_VOL_TEMP_HIGH].min_volt ||
		    batt_params->sys_term_vol_maximum[TRACK_SYS_TERM_VOL_TEMP_LOW].max_volt ||
		    batt_params->sys_term_vol_maximum[TRACK_SYS_TERM_VOL_TEMP_LOW].min_volt)
			memcpy(batt_params->pre_sys_term_vol_maximum,
				batt_params->sys_term_vol_maximum, sizeof(batt_params->sys_term_vol_maximum));
		memset(batt_params->sys_term_vol_maximum, 0, sizeof(batt_params->sys_term_vol_maximum));
		return -EINVAL;
	}

	if (!sili_ic_alg_term_volt)
		return -EINVAL;

	batt_params->sili_monitor_sys_term_volt = sili_ic_alg_term_volt;
	oplus_chg_track_get_gauge_sili_single_dischg_term_volt_status(
		track_chip, batt_info, batt_params, &err_type);
	oplus_chg_track_get_gauge_sili_multiple_dischg_term_volt_status(
		track_chip, batt_info, batt_params, &err_type);
	term_volt_ref = oplus_chg_track_get_gauge_sili_cc_term_volt_status(
		track_chip, batt_params, &err_type);

	if (batt_info->debug_err_type == TRACK_GAGUE_ERR_SINGLE_DISCHG_TERM_VOLT ||
	    batt_info->debug_err_type == TRACK_GAGUE_ERR_MULTIPLE_DISCHG_TERM_VOLT ||
	    batt_info->debug_err_type == TRACK_GAGUE_ERR_CC_TERM_VOLT ||
	    batt_info->debug_err_type == TRACK_GAGUE_ERR_BELOW_FIRMWARE_TERM_VOLT)
		err_type = batt_info->debug_err_type;

	if (!err_type && ((!batt_info->charger_exist && charger_exist) ||
	   (ui_soc == OPLUS_CHG_TRACK_SOC_THD(1) && batt_params->ui_soc != ui_soc)))
		    err_type = TRACK_GAGUE_TERM_VOLT_OK;

	if (err_type)
		oplus_chg_track_gauge_sili_alg_monitor_record(batt_info, err_type, term_volt_ref);

	batt_info->charger_exist = charger_exist;
	batt_params->pre_sili_monitor_sys_term_volt = batt_params->sili_monitor_sys_term_volt;
	batt_params->ui_soc = ui_soc;

	return 0;
}

static int oplus_chg_track_gauge_sili_alg_lifetime_record(
	struct oplus_chg_track_gauge_info *p_gauge_info, int err_type)
{
	int index = 0;
	int curr_time;
	struct rtc_time tm;
	static int upload_count = 0;
	static int pre_upload_time = 0;
	char err_reason[OPLUS_CHG_TRACK_DEVICE_ERR_NAME_LEN] = { 0 };

	if (!p_gauge_info)
		return -EINVAL;

	if (err_type <= TRACK_GAGUE_ERR_DEFAULT || err_type >= TRACK_GAGUE_ERR_MAX) {
		pr_info("err_type not match\n");
		return -EINVAL;
	}

	mutex_lock(&p_gauge_info->sili_alg_lifetime_lock);
	curr_time = oplus_chg_track_get_current_time_s(&tm);
	if (curr_time - pre_upload_time > TRACK_GAUGE_UPLOAD_PERIOD)
		upload_count = 0;

	if (upload_count > TRACK_GAUGE_UPLOAD_COUNT_MAX) {
		pr_debug("uploading count arrive max\n");
		mutex_unlock(&p_gauge_info->sili_alg_lifetime_lock);
		return 0;
	}

	if (p_gauge_info->sili_alg_lifetime_load_trigger)
		kfree(p_gauge_info->sili_alg_lifetime_load_trigger);
	p_gauge_info->sili_alg_lifetime_load_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!p_gauge_info->sili_alg_lifetime_load_trigger) {
		pr_err("gauge load_trigger memery alloc fail\n");
		mutex_unlock(&p_gauge_info->sili_alg_lifetime_lock);
		return -ENOMEM;
	}

	if (err_type != TRACK_GAGUE_LIFETIME_INFO) {
		p_gauge_info->sili_alg_lifetime_load_trigger->type_reason = TRACK_NOTIFY_TYPE_DEVICE_ABNORMAL;
		p_gauge_info->sili_alg_lifetime_load_trigger->flag_reason = TRACK_NOTIFY_FLAG_GAGUE_ABNORMAL;
	} else {
		p_gauge_info->sili_alg_lifetime_load_trigger->type_reason = TRACK_NOTIFY_TYPE_GENERAL_RECORD;
		p_gauge_info->sili_alg_lifetime_load_trigger->flag_reason = TRACK_NOTIFY_FLAG_GAUGE_INFO;
	}
	upload_count++;
	pre_upload_time = curr_time;
	index += scnprintf(&(p_gauge_info->sili_alg_lifetime_load_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$device_id@@%s", p_gauge_info->device_name);
	if (err_type != TRACK_GAGUE_LIFETIME_INFO)
		index +=
			scnprintf(&(p_gauge_info->sili_alg_lifetime_load_trigger->crux_info[index]),
				OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				 "$$err_scene@@%s", OPLUS_CHG_TRACK_SCENE_GAGUE_DEFAULT);
	else
		index +=
			scnprintf(&(p_gauge_info->sili_alg_lifetime_load_trigger->crux_info[index]),
				OPLUS_CHG_TRACK_CURX_INFO_LEN - index,
				 "$$err_scene@@%s", OPLUS_CHG_TRACK_SCENE_GAGUE_LIFETIME_INFO);

	oplus_chg_track_get_gague_err_reason(err_type, err_reason, sizeof(err_reason));
	index += scnprintf(&(p_gauge_info->sili_alg_lifetime_load_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$err_reason@@%s", err_reason);
	index += scnprintf(&(p_gauge_info->sili_alg_lifetime_load_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$reg_info@@");
	oplus_gauge_get_sili_alg_lifetime_info(p_gauge_info->params.gauge_topic,
		&(p_gauge_info->sili_alg_lifetime_load_trigger->crux_info[index]),
		OPLUS_CHG_TRACK_CURX_INFO_LEN - index);

	schedule_delayed_work(&p_gauge_info->sili_alg_lifetime_load_trigger_work, 0);

	return 0;
}

static int oplus_chg_track_get_gauge_lifetime_status(
			struct oplus_chg_track *track_chip, struct oplus_chg_track_gauge_info *batt_info,
			struct oplus_chg_track_gauge_params *batt_params)
{
	int ret;
	struct rtc_time tm;
	union mms_msg_data data = { 0 };
	int err_type = TRACK_GAGUE_ERR_DEFAULT;
	struct oplus_gauge_lifetime *lifetime = NULL;
	int curr_time = oplus_chg_track_get_current_time_s(&tm);

	if (!track_chip->track_cfg.gauge_lifetime_support)
		return -ENOTSUPP;

	if (!batt_info->lifetime_check_time) {
		batt_info->lifetime_check_time = curr_time;
		batt_info->lifetime_upload_time = curr_time;
	}

	if (curr_time - batt_info->lifetime_check_time > TRACK_GAUGE_LIFETIME_CHECK_PERIOD) {
		batt_info->lifetime_check_time = curr_time;
		ret = oplus_mms_get_item_data(batt_params->gauge_topic, GAUGE_ITEM_LIFETIME_STATUS, &data, true);
		if (ret == 0 && data.strval)
			lifetime = (struct oplus_gauge_lifetime *)data.strval;

		if (lifetime && memcmp(lifetime, &batt_params->lifetime, sizeof(struct oplus_gauge_lifetime)) &&
		    batt_params->lifetime.max_cell_vol &&
		   (lifetime->max_cell_vol >= track_chip->track_cfg.gauge_max_cell_vol ||
		    lifetime->max_charge_curr >= track_chip->track_cfg.gauge_max_charge_curr ||
		    lifetime->max_dischg_curr <= track_chip->track_cfg.gauge_max_dischg_curr* (-1) ||
		    lifetime->max_cell_temp >= track_chip->track_cfg.gauge_max_cell_temp ||
		    lifetime->min_cell_temp <= track_chip->track_cfg.gauge_min_cell_temp * (-1)))
			err_type = TRACK_GAGUE_ERR_LIFETIME_OVER;
		if (lifetime)
			memcpy(&batt_params->lifetime, lifetime, sizeof(struct oplus_gauge_lifetime));
	}

	if (batt_info->debug_err_type == TRACK_GAGUE_ERR_LIFETIME_OVER)
		err_type = TRACK_GAGUE_ERR_LIFETIME_OVER;

	if (!err_type && batt_params->lifetime.max_cell_vol &&
	   (curr_time - batt_info->lifetime_upload_time > TRACK_GAUGE_LIFETIME_UPLOAD_PERIOD)) {
		batt_info->lifetime_upload_time = curr_time;
		err_type = TRACK_GAGUE_LIFETIME_INFO;
	}

	if (lifetime)
		pr_info("lifetime[%d, %d, %d, %d, %d], err_type=%d\n",
			lifetime->max_cell_vol, lifetime->max_charge_curr, lifetime->max_dischg_curr,
			lifetime->max_cell_temp, lifetime->min_cell_temp, err_type);

	if (err_type) {
		batt_info->lifetime_upload_time = curr_time;
		batt_info->lifetime_check_time = curr_time;
		oplus_chg_track_gauge_sili_alg_lifetime_record(batt_info, err_type);
	}

	return 0;
}

static int oplus_chg_track_gauge_batt_monitor_record(
			struct oplus_chg_track_gauge_info *p_gauge_info, int update_type)
{
	int index = 0;
	char err_reason[OPLUS_CHG_TRACK_DEVICE_ERR_NAME_LEN] = { 0 };

	if (!p_gauge_info)
		return -EINVAL;

	if (update_type <= TRACK_GAGUE_ERR_DEFAULT || update_type >= TRACK_GAGUE_ERR_MAX)
		return -EINVAL;

	mutex_lock(&p_gauge_info->batt_monitor_lock);
	if (p_gauge_info->batt_monitor_load_trigger)
		kfree(p_gauge_info->batt_monitor_load_trigger);

	p_gauge_info->batt_monitor_load_trigger = kzalloc(sizeof(oplus_chg_track_trigger), GFP_KERNEL);
	if (!p_gauge_info->batt_monitor_load_trigger) {
		pr_err("kzalloc batt_monitor_load_trigger failed\n");
		mutex_unlock(&p_gauge_info->batt_monitor_lock);
		return -ENOMEM;
	}

	p_gauge_info->batt_monitor_load_trigger->type_reason = TRACK_NOTIFY_TYPE_GENERAL_RECORD;
	p_gauge_info->batt_monitor_load_trigger->flag_reason = TRACK_NOTIFY_FLAG_LOW_BATT_MONITOR;
	oplus_chg_track_get_gague_err_reason(update_type, err_reason, sizeof(err_reason));
	index += scnprintf(&(p_gauge_info->batt_monitor_load_trigger->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$device_id@@%s", p_gauge_info->device_name);
	index += scnprintf(&(p_gauge_info->batt_monitor_load_trigger->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$err_scene@@%s", err_reason);

	index += scnprintf(&(p_gauge_info->batt_monitor_load_trigger->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$batt_temp@@%d", p_gauge_info->params.batt_temp);
	index += scnprintf(&(p_gauge_info->batt_monitor_load_trigger->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$vbat_mv@@%d", p_gauge_info->params.batt_volt);
	index += scnprintf(&(p_gauge_info->batt_monitor_load_trigger->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$batt_soc@@%d", p_gauge_info->params.soc);
	index += scnprintf(&(p_gauge_info->batt_monitor_load_trigger->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$ui_soc@@%d", p_gauge_info->params.ui_soc);
	index += scnprintf(&(p_gauge_info->batt_monitor_load_trigger->crux_info[index]),
			OPLUS_CHG_TRACK_CURX_INFO_LEN - index, "$$ibat_ma@@%d", p_gauge_info->params.batt_curr);

	schedule_delayed_work(&p_gauge_info->batt_monitor_load_trigger_work, 0);
	chg_info("monitor uploader success\n");
	return 0;
}

static int oplus_chg_track_get_gauge_low_soc_monitor_status(
			struct oplus_chg_track *track_chip, struct oplus_chg_track_gauge_info *batt_info,
			struct oplus_chg_track_gauge_params *batt_params)
{
	int curr_time;
	int time_diff = 0;
	int update_type = 0;
	struct rtc_time tm;
	static int pre_curr_time = 0;
	struct oplus_monitor *monitor;
	int charger_exist = 0;

	if (!track_chip->track_cfg.low_soc_monitor)
		return -ENOTSUPP;

	monitor = track_chip->monitor;
	charger_exist = monitor->wired_online || monitor->wls_online;

	if (batt_info->debug_err_type == TRACK_GAGUE_BATT_MONITOR_BY_SOC) {
		update_type = batt_info->debug_err_type;
	}

	if (batt_params->ui_soc > TRACK_MONITOR_BATT_START_SOC || charger_exist) {
		pre_curr_time = 0;
		return -ENOTSUPP;
	}

	if (batt_params->pre_ui_soc == batt_params->ui_soc + 1 && update_type == 0)
		update_type = TRACK_GAGUE_BATT_MONITOR_BY_SOC;

	if (pre_curr_time == 0)
		pre_curr_time = oplus_chg_track_get_current_time_s(&tm);

	curr_time = oplus_chg_track_get_current_time_s(&tm);
	time_diff = curr_time - pre_curr_time;

	if ((time_diff >= TRACK_TIME_30MIN_THD || (batt_info->debug_upload_period_t > 0 &&
					time_diff >= batt_info->debug_upload_period_t)) &&
					update_type == 0) {
		update_type = TRACK_GAGUE_BATT_MONITOR_BY_TIME;
		pre_curr_time = curr_time;
	}

	if (update_type) {
		oplus_chg_track_gauge_batt_monitor_record(batt_info, update_type);
		if (batt_info->debug_err_type == update_type)
			batt_info->debug_err_type = TRACK_GAGUE_ERR_DEFAULT;
	}

	return 0;
}

static void oplus_chg_track_gauge_apply_status_pipeline(
	struct oplus_chg_track *track_chip,
	struct oplus_chg_track_gauge_info *gauge_info,
	struct oplus_chg_track_gauge_params *params)
{
	oplus_chg_track_get_gauge_status(track_chip, gauge_info, params);
	oplus_chg_track_get_gauge_sili_alg_application_status(track_chip, gauge_info, params);
	oplus_chg_track_get_gauge_sili_alg_monitor_status(track_chip, gauge_info, params);
	oplus_chg_track_get_gauge_lifetime_status(track_chip, gauge_info, params);
	oplus_chg_track_get_gauge_low_soc_monitor_status(track_chip, gauge_info, params);
}

static void oplus_chg_track_gauge_apply_status_pipeline_dual_interleaved(
	struct oplus_chg_track *track_chip)
{
	oplus_chg_track_get_gauge_status(track_chip, &track_chip->gauge_info,
					 &track_chip->gauge_info.params);
	oplus_chg_track_get_gauge_status(track_chip, &track_chip->sub_gauge_info,
					 &track_chip->sub_gauge_info.params);
	oplus_chg_track_get_gauge_sili_alg_application_status(
		track_chip, &track_chip->gauge_info, &track_chip->gauge_info.params);
	oplus_chg_track_get_gauge_sili_alg_application_status(
		track_chip, &track_chip->sub_gauge_info, &track_chip->sub_gauge_info.params);
	oplus_chg_track_get_gauge_sili_alg_monitor_status(
		track_chip, &track_chip->gauge_info, &track_chip->gauge_info.params);
	oplus_chg_track_get_gauge_sili_alg_monitor_status(
		track_chip, &track_chip->sub_gauge_info, &track_chip->sub_gauge_info.params);
	oplus_chg_track_get_gauge_lifetime_status(
		track_chip, &track_chip->gauge_info, &track_chip->gauge_info.params);
	oplus_chg_track_get_gauge_lifetime_status(
		track_chip, &track_chip->sub_gauge_info, &track_chip->sub_gauge_info.params);
	oplus_chg_track_get_gauge_low_soc_monitor_status(
		track_chip, &track_chip->gauge_info, &track_chip->gauge_info.params);
	oplus_chg_track_get_gauge_low_soc_monitor_status(
		track_chip, &track_chip->sub_gauge_info, &track_chip->sub_gauge_info.params);
}

static void oplus_chg_track_gauge_single_init_first(
	struct oplus_chg_track *track_chip,
	struct oplus_monitor *monitor,
	char *name)
{
	int rc;

	track_chip->gauge_info.nominal_fcc = track_chip->track_cfg.nominal_fcc1;
	track_chip->gauge_info.nominal_qmax = track_chip->track_cfg.nominal_qmax1;
	track_chip->gauge_info.params.pre_soc = track_chip->gauge_info.params.soc;
	track_chip->gauge_info.params.pre_ui_soc = track_chip->gauge_info.params.ui_soc;
	track_chip->gauge_info.params.pre_cc = track_chip->gauge_info.params.cc;
	track_chip->gauge_info.params.pre_record_soc = track_chip->gauge_info.params.soc;
	track_chip->gauge_info.params.pre_batt_temp = track_chip->gauge_info.params.batt_temp;
	track_chip->gauge_info.params.pre_soh = track_chip->gauge_info.params.soh;
	rc = oplus_gauge_get_physical_name(monitor->gauge_topic, name, TRACK_GAUGE_NAME_LEN);
	if (rc == 0)
		scnprintf(track_chip->gauge_info.device_name, TRACK_GAUGE_NAME_LEN, "%s", name);
	else
		scnprintf(track_chip->gauge_info.device_name, TRACK_GAUGE_NAME_LEN, "%s", "main_gauge");
}

static int oplus_chg_track_gauge_status_check_single(
	struct oplus_chg_track *track_chip,
	struct oplus_monitor *monitor,
	bool *enter,
	char *name)
{
	track_chip->gauge_info.params.gauge_topic = monitor->gauge_topic;
	track_chip->gauge_info.params.soc = monitor->batt_soc;
	track_chip->gauge_info.params.cc = monitor->batt_cc;
	track_chip->gauge_info.params.batt_temp = monitor->batt_temp;
	track_chip->gauge_info.params.soh = monitor->batt_soh;
	track_chip->gauge_info.params.fcc = monitor->batt_fcc;
	track_chip->gauge_info.params.batt_volt = monitor->vbat_mv;
	track_chip->gauge_info.params.ui_soc = monitor->ui_soc;
	oplus_gauge_get_qmax(monitor->gauge_topic, 0, &track_chip->gauge_info.params.qmax);
	if (!*enter) {
		oplus_chg_track_gauge_single_init_first(track_chip, monitor, name);
		*enter = true;
	}
	oplus_chg_track_gauge_apply_status_pipeline(track_chip, &track_chip->gauge_info,
						  &track_chip->gauge_info.params);
	track_chip->gauge_info.params.pre_soc = track_chip->gauge_info.params.soc;
	track_chip->gauge_info.params.pre_ui_soc = track_chip->gauge_info.params.ui_soc;
	track_chip->gauge_info.params.pre_cc = track_chip->gauge_info.params.cc;
	track_chip->gauge_info.params.pre_batt_temp = track_chip->gauge_info.params.batt_temp;
	track_chip->gauge_info.params.pre_soh = track_chip->gauge_info.params.soh;

	return 0;
}

static void oplus_chg_track_gauge_dual_init_first(
	struct oplus_chg_track *track_chip,
	struct oplus_monitor *monitor,
	char *name)
{
	int rc;

	track_chip->gauge_info.nominal_fcc = track_chip->track_cfg.nominal_fcc1;
	track_chip->gauge_info.nominal_qmax = track_chip->track_cfg.nominal_qmax1;
	track_chip->gauge_info.params.pre_soc = track_chip->gauge_info.params.soc;
	track_chip->gauge_info.params.pre_ui_soc = track_chip->gauge_info.params.ui_soc;
	track_chip->gauge_info.params.pre_cc = track_chip->gauge_info.params.cc;
	track_chip->gauge_info.params.pre_record_soc = track_chip->gauge_info.params.soc;
	track_chip->gauge_info.params.pre_batt_temp = track_chip->gauge_info.params.batt_temp;
	track_chip->gauge_info.params.pre_soh = track_chip->gauge_info.params.soh;
	track_chip->sub_gauge_info.nominal_fcc = track_chip->track_cfg.nominal_fcc2;
	track_chip->sub_gauge_info.nominal_qmax = track_chip->track_cfg.nominal_qmax2;
	track_chip->sub_gauge_info.params.pre_soc = track_chip->sub_gauge_info.params.soc;
	track_chip->sub_gauge_info.params.pre_ui_soc = track_chip->sub_gauge_info.params.ui_soc;
	track_chip->sub_gauge_info.params.pre_cc = track_chip->sub_gauge_info.params.cc;
	track_chip->sub_gauge_info.params.pre_record_soc = track_chip->sub_gauge_info.params.soc;
	track_chip->sub_gauge_info.params.pre_batt_temp = track_chip->sub_gauge_info.params.batt_temp;
	track_chip->sub_gauge_info.params.pre_soh = track_chip->sub_gauge_info.params.soh;
	rc = oplus_gauge_get_physical_name(monitor->gauge_topic, name, TRACK_GAUGE_NAME_LEN);
	if (rc == 0) {
		scnprintf(track_chip->gauge_info.device_name, TRACK_GAUGE_NAME_LEN, "%s_0", name);
		scnprintf(track_chip->sub_gauge_info.device_name, TRACK_GAUGE_NAME_LEN, "%s_1", name);
	} else {
		scnprintf(track_chip->gauge_info.device_name, TRACK_GAUGE_NAME_LEN, "%s", "main_gauge");
		scnprintf(track_chip->sub_gauge_info.device_name, TRACK_GAUGE_NAME_LEN, "%s", "sub_gauge");
	}
}

static void oplus_chg_track_gauge_dual_fetch_params(
	struct oplus_chg_track *track_chip,
	struct oplus_monitor *monitor,
	struct oplus_mms *g0,
	struct oplus_mms *g1,
	union mms_msg_data *data)
{
	oplus_gauge_get_qmax(monitor->gauge_topic, 0, &track_chip->gauge_info.params.qmax);
	oplus_gauge_get_qmax(monitor->gauge_topic, 1, &track_chip->sub_gauge_info.params.qmax);
	oplus_mms_get_item_data(g0, GAUGE_ITEM_SOC, data, false);
	track_chip->gauge_info.params.soc = data->intval;
	oplus_mms_get_item_data(g1, GAUGE_ITEM_SOC, data, false);
	track_chip->sub_gauge_info.params.soc = data->intval;
	oplus_mms_get_item_data(track_chip->monitor->comm_topic, COMM_ITEM_UI_SOC, data, false);
	track_chip->gauge_info.params.ui_soc = data->intval;
	oplus_mms_get_item_data(track_chip->monitor->comm_topic, COMM_ITEM_UI_SOC, data, false);
	track_chip->sub_gauge_info.params.ui_soc = data->intval;
	oplus_mms_get_item_data(g0, GAUGE_ITEM_CC, data, true);
	track_chip->gauge_info.params.cc = data->intval;
	oplus_mms_get_item_data(g1, GAUGE_ITEM_CC, data, true);
	track_chip->sub_gauge_info.params.cc = data->intval;
	oplus_mms_get_item_data(g0, GAUGE_ITEM_TEMP, data, true);
	track_chip->gauge_info.params.batt_temp = data->intval;
	oplus_mms_get_item_data(g1, GAUGE_ITEM_TEMP, data, true);
	track_chip->sub_gauge_info.params.batt_temp = data->intval;
	oplus_mms_get_item_data(g0, GAUGE_ITEM_SOH, data, true);
	track_chip->gauge_info.params.soh = data->intval;
	oplus_mms_get_item_data(g1, GAUGE_ITEM_SOH, data, true);
	track_chip->sub_gauge_info.params.soh = data->intval;
	oplus_mms_get_item_data(g0, GAUGE_ITEM_FCC, data, true);
	track_chip->gauge_info.params.fcc = data->intval;
	oplus_mms_get_item_data(g1, GAUGE_ITEM_FCC, data, true);
	track_chip->sub_gauge_info.params.fcc = data->intval;
	oplus_mms_get_item_data(g0, GAUGE_ITEM_VOL_MAX, data, true);
	track_chip->gauge_info.params.batt_volt = data->intval;
	oplus_mms_get_item_data(g1, GAUGE_ITEM_VOL_MAX, data, true);
	track_chip->sub_gauge_info.params.batt_volt = data->intval;
}

static void oplus_chg_track_gauge_dual_stash_pre_params(struct oplus_chg_track *track_chip)
{
	track_chip->gauge_info.params.pre_soc = track_chip->gauge_info.params.soc;
	track_chip->sub_gauge_info.params.pre_soc = track_chip->sub_gauge_info.params.soc;
	track_chip->gauge_info.params.pre_ui_soc = track_chip->gauge_info.params.ui_soc;
	track_chip->sub_gauge_info.params.pre_ui_soc = track_chip->sub_gauge_info.params.ui_soc;
	track_chip->gauge_info.params.pre_cc = track_chip->gauge_info.params.cc;
	track_chip->sub_gauge_info.params.pre_cc = track_chip->sub_gauge_info.params.cc;
	track_chip->gauge_info.params.pre_batt_temp = track_chip->gauge_info.params.batt_temp;
	track_chip->sub_gauge_info.params.pre_batt_temp = track_chip->sub_gauge_info.params.batt_temp;
	track_chip->gauge_info.params.pre_soh = track_chip->gauge_info.params.soh;
	track_chip->sub_gauge_info.params.pre_soh = track_chip->sub_gauge_info.params.soh;
}

static int oplus_chg_track_gauge_status_check_dual(
	struct oplus_chg_track *track_chip,
	struct oplus_monitor *monitor,
	bool *enter,
	char *name,
	union mms_msg_data *data)
{
	struct oplus_mms *g0, *g1;

	track_chip->gauge_info.params.gauge_topic = oplus_mms_get_by_name("gauge:0");
	track_chip->sub_gauge_info.params.gauge_topic = oplus_mms_get_by_name("gauge:1");
	if (!track_chip->gauge_info.params.gauge_topic ||
	    !track_chip->sub_gauge_info.params.gauge_topic) {
		chg_err("external_gauge_num=2 not match\n");
		return -EINVAL;
	}
	g0 = track_chip->gauge_info.params.gauge_topic;
	g1 = track_chip->sub_gauge_info.params.gauge_topic;

	oplus_chg_track_gauge_dual_fetch_params(track_chip, monitor, g0, g1, data);

	if (!*enter) {
		oplus_chg_track_gauge_dual_init_first(track_chip, monitor, name);
		*enter = true;
	}
	oplus_chg_track_gauge_apply_status_pipeline_dual_interleaved(track_chip);
	oplus_chg_track_gauge_dual_stash_pre_params(track_chip);

	return 0;
}

static int oplus_chg_track_gauge_status_check_platform_zero(
	struct oplus_chg_track *track_chip,
	struct oplus_monitor *monitor,
	bool *enter)
{
	track_chip->gauge_info.params.gauge_topic = monitor->gauge_topic;
	track_chip->gauge_info.params.soc = monitor->batt_soc;
	track_chip->gauge_info.params.batt_temp = monitor->batt_temp;
	track_chip->gauge_info.params.batt_volt = monitor->vbat_mv;
	track_chip->gauge_info.params.ui_soc = monitor->ui_soc;
	track_chip->gauge_info.params.batt_curr = monitor->ibat_ma;
	if (!*enter) {
		track_chip->gauge_info.params.pre_ui_soc = track_chip->gauge_info.params.ui_soc;
		scnprintf(track_chip->gauge_info.device_name, TRACK_GAUGE_NAME_LEN, "%s", "platform_gauge");
		*enter = true;
	}
	oplus_chg_track_get_gauge_low_soc_monitor_status(
		track_chip, &track_chip->gauge_info, &track_chip->gauge_info.params);
	track_chip->gauge_info.params.pre_ui_soc = track_chip->gauge_info.params.ui_soc;

	return 0;
}

static int oplus_chg_track_gauge_status_check(struct oplus_monitor *monitor)
{
	struct oplus_chg_track *track_chip = g_track_chip;
	static bool enter = false;
	char name[TRACK_GAUGE_NAME_LEN] = { 0 };
	union mms_msg_data data = { 0 };
	int gauge_type;
	bool single_one_or_plat;

	if (!track_chip || !track_chip->track_cfg.track_gauge_ctrl)
		return -ENOTSUPP;
	if (!monitor->batt_exist)
		return -EINVAL;

	gauge_type = oplus_get_gauge_type();
	/* Support platform gauge even if external_gauge_num is 0 */
	if (track_chip->track_cfg.external_gauge_num <= 0 && gauge_type != GAUGE_TYPE_PLATFORM)
		return -ENOTSUPP;

	single_one_or_plat = track_chip->track_cfg.external_gauge_num == 1 ||
			     (track_chip->track_cfg.external_gauge_num == 0 &&
			      gauge_type == GAUGE_TYPE_PLATFORM);
	if (single_one_or_plat)
		return oplus_chg_track_gauge_status_check_single(track_chip, monitor, &enter, name);
	if (track_chip->track_cfg.external_gauge_num == 2)
		return oplus_chg_track_gauge_status_check_dual(track_chip, monitor, &enter, name, &data);
	if (track_chip->track_cfg.external_gauge_num == 0)
		return oplus_chg_track_gauge_status_check_platform_zero(track_chip, monitor, &enter);

	return 0;
}

int oplus_chg_track_comm_monitor(struct oplus_monitor *monitor)
{
	int ret = 0;

	if (!monitor)
		return -EFAULT;

	if (likely(monitor->ui_soc_ready))
		ret = oplus_chg_track_uisoc_soc_jump_check(monitor);
	ret |= oplus_chg_track_speed_check(monitor);
	ret |= oplus_chg_track_gauge_status_check(monitor);

	return ret;
}

static void oplus_chg_track_err_subs_callback(struct mms_subscribe *subs,
					      enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_chg_track *track = subs->priv_data;

	switch (type) {
	case MSG_TYPE_ITEM:
		chg_err("err msg id:%d\n", id);
		switch (id) {
		case ERR_ITEM_IC:
			oplus_chg_track_upload_ic_err_info(track);
			break;
		case ERR_ITEM_USBTEMP:
			oplus_chg_track_upload_usbtemp_info(track);
			break;
		case ERR_ITEM_VBAT_TOO_LOW:
			oplus_chg_track_upload_vbatt_too_low_info(track);
			break;
		case COMM_ITEM_UISOC_DROP_ERROR:
			oplus_chg_track_upload_uisoc_drop_err_info(track);
			break;
		case ERR_ITEM_VBAT_DIFF_OVER:
			break;
		case ERR_ITEM_UI_SOC_SHUTDOWN:
			track->monitor->sem_info.uisoc_0 = true;
			oplus_chg_track_super_endurance_mode_change(track->monitor);
			oplus_chg_track_upload_vbatt_diff_over_info(track);
			oplus_chg_track_upload_uisoc_keep_1_t_info(track);
			oplus_chg_track_upload_dischg_profile(track->monitor);
			break;
		case ERR_ITEM_DUAL_CHAN:
			oplus_chg_track_upload_dual_chan_err_info(track);
			break;
		case ERR_ITEM_MMI_CHG:
			oplus_chg_track_upload_mmi_chg_info(track);
			break;
		case ERR_ITEM_SLOW_CHG:
			oplus_chg_track_upload_slow_chg_info(track);
			break;
		case ERR_ITEM_CHG_CYCLE:
			oplus_chg_track_upload_chg_cycle_info(track);
			break;
		case ERR_ITEM_WLS_INFO:
			oplus_chg_track_upload_wls_info(track);
			break;
		case ERR_ITEM_UFCS:
			oplus_chg_track_upload_ufcs_info(track);
			break;
		case ERR_ITEM_PPS:
			oplus_chg_track_upload_pps_info(track);
			break;
		case ERR_ITEM_CYCLE_CURRENT_DERATING:
			oplus_chg_track_upload_cycle_current_derating_info(track);
			break;
		case ERR_ITEM_DEEP_DISCHG_INFO:
		case ERR_ITEM_SUB_DEEP_DISCHG_INFO:
			oplus_chg_track_upload_deep_dischg_info(track, id);
			break;
		case ERR_ITEM_CHG_INTO_LIQUID:
			oplus_chg_track_upload_chg_into_liquid(track);
			break;
		case ERR_ITEM_DEEP_DISCHG_PROFILE:
			oplus_chg_track_upload_deep_dischg_profile(track);
			break;
		case ERR_ITEM_BIDIRECT_CP_INFO:
			oplus_chg_track_upload_bidirect_cp_info(track);
			break;
		case ERR_ITEM_EIS_TIMEOUT:
			oplus_chg_track_upload_eis_timeout_info(track);
			break;
		case ERR_ITEM_BAL_INFO:
			oplus_chg_track_upload_bal_info(track);
			break;
		case ERR_ITEM_PLC_INFO:
			oplus_chg_track_upload_plc_info(track);
			break;
		case ERR_ITEM_DEC_CV_INFO:
			oplus_chg_track_upload_dec_cv_info(track);
			break;
		case ERR_ITEM_LPD:
			oplus_chg_track_upload_usb_lpd_info(track);
			break;
		case ERR_ITEM_DEVICE_ID:
			oplus_chg_track_update_device_id(track);
			break;
		case ERR_ITEM_ERR_PHY_CP_INFO:
			oplus_chg_track_upload_phy_cp_info(track);
			break;
		case ERR_ITEM_GAUGE_R_INFO:
			oplus_chg_track_upload_gauge_r_info(track);
			break;
		case ERR_ITEM_SOCCP_CRASH:
			oplus_chg_track_upload_soccp_crash_info(track);
			break;
		case ERR_ITEM_FCL_INFO:
			oplus_chg_track_upload_fcl_info(track);
			break;
		case ERR_ITEM_BS_INFO:
			oplus_chg_track_upload_bs_info(track);
			break;
		case ERR_ITEM_STATE_KEEP_INFO:
			oplus_chg_track_upload_state_keep_info(track);
			break;
		case ERR_ITEM_STATE_KEEP_ABNORMAL:
			oplus_chg_track_upload_state_keep_abnormal(track);
			break;
		case ERR_ITEM_USBIN_ABNORMAL:
			oplus_chg_track_upload_usbin_abnormal(track);
			break;
		case ERR_ITEM_SEC_IC_MEM_INFO:
			oplus_chg_track_upload_sec_ic_mem_info(track);
			break;
		case ERR_ITEM_SHUTDOWN_VOL:
			oplus_chg_track_upload_shutdown_vol_info(track);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_chg_track_pps_subs_callback(struct mms_subscribe *subs,
					      enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_chg_track *track = subs->priv_data;
	pps_msg_data pps_data = {0};
	int i, temp_i;

	if (track->pps_info.adapter_pdo == NULL)
		track->pps_info.adapter_pdo =
			(u32 *)devm_kzalloc(track->monitor->dev,
					    sizeof(u32) * PPS_PDO_MAX, GFP_KERNEL);
	if (track->pps_info.adapter_pdo == NULL) {
		chg_err("devm_kzalloc is failed!!!");
		return;
	}

	switch (type) {
	case MSG_TYPE_ITEM:
		chg_info("pps msg id:%d\n", id);
		switch(id) {
		case PPS_ITEM_CPA_POWER:
			track->pps_info.cpa_power =
				oplus_pps_get_charging_power_watt(track->monitor->pps_topic);
			break;
		case PPS_ITEM_ADAPTER_POWER:
			track->pps_info.adapter_power =
				oplus_pps_get_adapter_power_mw(track->monitor->pps_topic);
			break;
		case PPS_ITEM_PPS_ADAPTER_INFO:
			if (track->pps_info.adapter_pdo == NULL)
				break;

			oplus_chg_get_pdo_info(track->monitor->pps_topic, track->pps_info.adapter_pdo);
			for (i = 0; i < PPS_PDO_MAX; i++)
			{
				temp_i = track->pps_info.adapter_power_max_index;
				pps_data = (pps_msg_data)track->pps_info.adapter_pdo[i];
				if (pps_data.pdo_type == USBPD_PDMSG_PDOTYPE_AUGMENTED) {
					if (pps_data.pdo_type == USBPD_PDMSG_PDOTYPE_AUGMENTED &&
					    ((PPS_PDO_VOL_MAX(pps_data.max_voltage100mv) *
					      PPS_PDO_CURR_MAX(pps_data.max_current50ma)) >
					    ((PPS_PDO_VOL_MAX(pps_data.max_voltage100mv) *
                                              PPS_PDO_CURR_MAX(pps_data.max_current50ma))))) {
						track->pps_info.adapter_power_max_index = i;
					} else {
						track->pps_info.adapter_power_max_index = i;
					}
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

static int oplus_chg_track_subscribe_err_topic(struct oplus_chg_track *track)
{
	track->err_subs =
		oplus_mms_subscribe(track->monitor->err_topic, track,
				    oplus_chg_track_err_subs_callback, "track");
	if (IS_ERR_OR_NULL(track->err_subs)) {
		chg_err("subscribe error topic error, rc=%ld\n",
			PTR_ERR(track->err_subs));
		return PTR_ERR(track->err_subs);
	}

	return 0;
}

static void oplus_chg_track_subscribe_pps_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_chg_track *track = prv_data;

	track->monitor->pps_topic = topic;
	track->pps_subs =
		oplus_mms_subscribe(track->monitor->pps_topic, track,
				    oplus_chg_track_pps_subs_callback, "track");
	if (IS_ERR_OR_NULL(track->pps_subs)) {
		chg_err("subscribe pps topic error, rc=%ld\n",
			PTR_ERR(track->pps_subs));
	}
	return;
}

static void oplus_chg_track_subscribe_dischg_boost_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_chg_track *track = prv_data;

	track->monitor->dischg_boost_topic = topic;
	/* CV mode is now obtained directly via oplus_boost_get_cv_mode() when needed */
	return;
}

static int oplus_chg_track_debugfs_init(struct oplus_chg_track *track_dev)
{
	int ret = 0;
	struct dentry *debugfs_root;
	struct dentry *debugfs_general;
	struct dentry *debugfs_chg_slow;
	struct dentry *debugfs_gauge;

	debugfs_root = oplus_chg_track_get_debugfs_root();
	if (!debugfs_root) {
		ret = -ENOENT;
		return ret;
	}

	debugfs_general = debugfs_create_dir("general", debugfs_root);
	if (!debugfs_general) {
		ret = -ENOENT;
		return ret;
	}

	debugfs_chg_slow = debugfs_create_dir("chg_slow", debugfs_general);
	if (!debugfs_chg_slow) {
		ret = -ENOENT;
		return ret;
	}

	debugfs_gauge = debugfs_create_dir("gauge", debugfs_root);
	if (!debugfs_gauge) {
		ret = -ENOENT;
		return ret;
	}

	debugfs_create_u8("debug_soc", 0644, debugfs_general,
			  &(track_dev->track_status.debug_soc));
	debugfs_create_u8("debug_uisoc", 0644, debugfs_general,
			  &(track_dev->track_status.debug_uisoc));
	debugfs_create_u8("debug_fast_prop_status", 0644, debugfs_general,
			  &(track_dev->track_status.debug_fast_prop_status));
	debugfs_create_u8(
		"debug_normal_charging_state", 0644, debugfs_general,
		&(track_dev->track_status.debug_normal_charging_state));
	debugfs_create_u8("debug_normal_prop_status", 0644, debugfs_general,
			  &(track_dev->track_status.debug_normal_prop_status));
	debugfs_create_u8("debug_no_charging", 0644, debugfs_general,
			  &(track_dev->track_status.debug_no_charging));
	debugfs_create_u8("debug_slow_charging_force", 0644, debugfs_chg_slow,
			  &(track_dev->track_status.debug_slow_charging_force));
	debugfs_create_u8(
		"debug_slow_charging_reason", 0644, debugfs_chg_slow,
		&(track_dev->track_status.debug_slow_charging_reason));
	debugfs_create_u32("debug_fast_chg_break_t_thd", 0644, debugfs_general,
			   &(track_dev->track_cfg.fast_chg_break_t_thd));
	debugfs_create_u32("debug_general_chg_break_t_thd", 0644,
			   debugfs_general,
			   &(track_dev->track_cfg.general_chg_break_t_thd));
	debugfs_create_u32("debug_wls_chg_break_t_thd", 0644, debugfs_general,
			   &(track_dev->track_cfg.wls_chg_break_t_thd));
	debugfs_create_u32("debug_wls_normal_chg_break_t_thd", 0644, debugfs_general,
			   &(track_dev->track_cfg.wls_normal_chg_break_t_thd));
	debugfs_create_u32("debug_chg_notify_flag", 0644, debugfs_general,
			   &(track_dev->track_status.debug_chg_notify_flag));
	debugfs_create_u32("debug_chg_notify_code", 0644, debugfs_general,
			   &(track_dev->track_status.debug_chg_notify_code));
	debugfs_create_bool("debug_close_3hours", 0644, debugfs_general,
				&(track_dev->track_status.debug_close_3hours));
	debugfs_create_u8("debug_plugout_state", 0644, debugfs_general,
			  &(track_dev->track_status.debug_plugout_state));
	debugfs_create_u8("debug_break_code", 0644, debugfs_general,
			  &(track_dev->track_status.debug_break_code));

	debugfs_create_u32("debug_gauge_err_type", 0644, debugfs_gauge,
			  &(track_dev->gauge_info.debug_err_type));
	debugfs_create_u32("debug_gauge_upload_period_t", 0644, debugfs_gauge,
			  &(track_dev->gauge_info.debug_upload_period_t));
	debugfs_create_u32("debug_gauge_soc_record_thd", 0644, debugfs_gauge,
			  &(track_dev->gauge_info.debug_soc_record_thd));
	debugfs_create_u32("debug_sub_gauge_err_type", 0644, debugfs_gauge,
			  &(track_dev->sub_gauge_info.debug_err_type));
	debugfs_create_u32("debug_sub_gauge_upload_period_t", 0644, debugfs_gauge,
			  &(track_dev->sub_gauge_info.debug_upload_period_t));
	debugfs_create_u32("debug_sub_gauge_soc_record_thd", 0644, debugfs_gauge,
			  &(track_dev->sub_gauge_info.debug_soc_record_thd));
	return ret;
}

int oplus_chg_track_driver_init(struct oplus_monitor *monitor)
{
	struct oplus_chg_track *track_dev;
	int rc;

	if (!monitor) {
		chg_err("monitor is NULL\n");
		return -ENODEV;
	}

	track_dev = devm_kzalloc(monitor->dev, sizeof(struct oplus_chg_track),
				 GFP_KERNEL);
	if (track_dev == NULL) {
		chg_err("alloc memory error\n");
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&track_dev->prechg_r_update_work, oplus_chg_track_prechg_r_update_work);

	monitor->track = track_dev;
	track_dev->monitor = monitor;

#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || \
	defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE) || \
	defined(CONFIG_OPLUS_KEVENT_UPLOAD)
	track_dev->dcs_info = (struct kernel_packet_info *)kmalloc(
		sizeof(char) * OPLUS_CHG_TRIGGER_MSG_LEN, GFP_KERNEL);
	if (track_dev->dcs_info == NULL) {
		rc = -ENOMEM;
		goto dcs_info_kmalloc_fail;
	}
#endif

	temp_track_status = (struct oplus_chg_track_status *)kmalloc(
		sizeof(struct oplus_chg_track_status), GFP_KERNEL);
	if (temp_track_status == NULL) {
		rc = -ENOMEM;
		goto dcs_info_kmalloc_fail;
	}
	mutex_init(&temp_track_status->track_status_lock);

	track_dev->dev = monitor->dev;

	track_dev->track_status.bcc_info = (struct oplus_chg_track_hidl_bcc_info *)kzalloc(
		sizeof(struct oplus_chg_track_hidl_bcc_info), GFP_KERNEL);
	if (!track_dev->track_status.bcc_info) {
		rc = -ENOMEM;
		goto bcc_info_kzmalloc_fail;
	}

	rc = oplus_chg_track_debugfs_init(track_dev);
	if (rc < 0) {
		rc = -ENOENT;
		chg_err("oplus chg track debugfs init fail, rc=%d\n", rc);
		goto debugfs_create_fail;
	}

	rc = oplus_chg_track_parse_dt(track_dev);
	if (rc < 0) {
		chg_err("oplus chg track parse dts error, rc=%d\n", rc);
		goto parse_dt_err;
	}

	rc = oplus_chg_track_gague_fifo_init(track_dev);
	if (rc < 0)
		goto gauge_kfifo_err;

	oplus_chg_track_init(track_dev);
	oplus_chg_track_mul_break_init(track_dev);
	rc = oplus_chg_track_thread_init(track_dev);
	if (rc < 0) {
		chg_err("oplus chg track mod init error, rc=%d\n", rc);
		goto track_kthread_init_err;
	}

	rc = oplus_chg_track_subscribe_err_topic(track_dev);
	oplus_mms_wait_topic("pps", oplus_chg_track_subscribe_pps_topic, track_dev);
	oplus_mms_wait_topic("dischg_boost", oplus_chg_track_subscribe_dischg_boost_topic, track_dev);
	oplus_chg_track_bcc_err_init(track_dev);
	oplus_chg_track_uisoh_err_init(track_dev);
	oplus_chg_track_chg_up_err_init(track_dev);
	oplus_chg_track_bcc_si_init(track_dev);
	oplus_chg_track_eis_init(track_dev);
	oplus_chg_track_anti_expansion_err_init(track_dev);
	oplus_chg_track_bat_info_init(track_dev);
	oplus_chg_track_bdd_err_init(track_dev);
	if (rc < 0)
		goto subscribe_err_topic_err;

	rc= oplus_chg_track_prechg_r_mutual_notify_reg(track_dev);
	if (rc < 0)
		goto prechg_r_mutual_notify_reg_err;

	track_dev->trigger_upload_wq =
		create_workqueue("oplus_chg_trigger_upload_wq");
	INIT_DELAYED_WORK(&track_dev->upload_info_dwork, oplus_chg_track_upload_info_dwork);
	INIT_DELAYED_WORK(&track_dev->gauge_info.load_trigger_work, oplus_chg_track_gauge_info_work);
	INIT_DELAYED_WORK(&track_dev->sub_gauge_info.load_trigger_work, oplus_chg_track_sub_gauge_info_work);
	oplus_parallelchg_track_foldmode_init(track_dev);
	oplus_chg_track_ttf_info_init(track_dev);
	g_track_chip = track_dev;
	chg_info("probe done\n");

	return 0;

prechg_r_mutual_notify_reg_err:
	if (!IS_ERR_OR_NULL(track_dev->err_subs))
		oplus_mms_unsubscribe(track_dev->err_subs);
subscribe_err_topic_err:
track_kthread_init_err:
	kfifo_free(&(track_dev->gauge_info.fifo));
	kfifo_free(&(track_dev->sub_gauge_info.fifo));
gauge_kfifo_err:
parse_dt_err:
bcc_info_kzmalloc_fail:
	if (track_debugfs_root)
		debugfs_remove_recursive(track_debugfs_root);
debugfs_create_fail:
#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || \
	defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE) || \
	defined(CONFIG_OPLUS_KEVENT_UPLOAD)
	kfree(track_dev->dcs_info);
#endif
dcs_info_kmalloc_fail:
	devm_kfree(monitor->dev, track_dev);
	monitor->track = NULL;
	return rc;
}

int oplus_chg_track_driver_exit(struct oplus_monitor *monitor)
{
	struct oplus_chg_track *track_dev;

	if (!monitor) {
		chg_err("monitor is NULL\n");
		return -ENODEV;
	}
	track_dev = monitor->track;
	if (!track_dev) {
		chg_err("track_dev is NULL\n");
		return 0;
	}

	oplus_chg_unreg_mutual_notifier(&track_dev->prechg_r_mutual);

	if (!IS_ERR_OR_NULL(track_dev->err_subs))
		oplus_mms_unsubscribe(track_dev->err_subs);

	if (!IS_ERR_OR_NULL(track_dev->pps_subs))
		oplus_mms_unsubscribe(track_dev->pps_subs);

	if (track_debugfs_root)
		debugfs_remove_recursive(track_debugfs_root);
#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || \
	defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE) || \
	defined(CONFIG_OPLUS_KEVENT_UPLOAD)
	kfree(track_dev->dcs_info);
#endif
	kfree(track_dev->track_status.bcc_info);
	devm_kfree(monitor->dev, track_dev->pps_info.adapter_pdo);
	devm_kfree(monitor->dev, track_dev);
	return 0;
}
