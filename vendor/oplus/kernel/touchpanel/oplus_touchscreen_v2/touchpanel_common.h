/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#ifndef _TOUCHPANEL_COMMON_H_
#define _TOUCHPANEL_COMMON_H_

/*********PART1:Head files**********************/
#include <linux/i2c.h>
#include <linux/firmware.h>
#include <linux/input.h>

#include "util_interface/touch_interfaces.h"
#include "tp_devices.h"
#include <linux/miscdevice.h>
#include <linux/version.h>
#include "message_list.h"
#include "tp_ioctl.h"

#ifdef CONFIG_TOUCHIRQ_UPDATE_QOS
#include <linux/pm_qos.h>
#define PM_QOS_TOUCH_WAKEUP_VALUE 400
#endif

#ifndef CONFIG_REMOVE_OPLUS_FUNCTION
#include <soc/oplus/device_info.h>
#endif

#if IS_ENABLED(CONFIG_DRM_OPLUS_PANEL_NOTIFY) || IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
#include <drm/drm_panel.h>
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
#define PDE_DATA pde_data
#endif


#define TP_SUPPORT_MAX 3
#define TP_NAME_SIZE_MAX 30

#define TP_MSG_SIZE_MAX 32

#define EFTM (250)
#define FW_UPDATE_COMPLETE_TIMEOUT  msecs_to_jiffies(40*1000)

/*********PART2:Define Area**********************/

#define PAGESIZE 512
#define MAX_AIINFO_SIZE 4096
#define MAX_GESTURE_COORD 6
#define MAX_FINGER_NUM 10

#define UNKOWN_GESTURE		0
#define DOU_TAP			1   /* double tap*/
#define UP_VEE			2   /* V*/
#define DOWN_VEE		3   /* ^*/
#define LEFT_VEE		4   /* >*/
#define RIGHT_VEE		5   /* <*/
#define CIRCLE_GESTURE		6   /* O*/
#define DOU_SWIP		7   /* ||*/
#define LEFT2RIGHT_SWIP		8   /* -->*/
#define RIGHT2LEFT_SWIP		9   /* <--*/
#define UP2DOWN_SWIP		10  /* |v*/
#define DOWN2UP_SWIP		11  /* |^*/
#define M_GESTRUE		12  /* M*/
#define W_GESTURE		13  /* W*/
#define FINGER_PRINTDOWN	14
#define FRINGER_PRINTUP		15
#define SINGLE_TAP		16
#define HEART			17
#define S_GESTURE		18 /* new project not use this gesture type*/
#define PENDETECT		18

/* gesture type for fingerprint start */
#define FP_GESTURE_HOLD     19
#define FP_GESTURE_RELEASE  20
/* gesture type for fingerprint end */

#define WATERPROOF_RUS_BIT	7

#define TEST_RST_OK		0
#define TEST_RST_NG		1
#define TEST_RST_WAIT		2

#define TEST_COMPLETE		0
#define TEST_RUNNING		1
#define TEST_NOSTART		3

#define HEALTH_REPORT_GRIP          "grip_report"
#define HEALTH_REPORT_BASELINE_ERR  "baseline_err"
#define HEALTH_REPORT_NOISE         "noise_count"
#define HEALTH_REPORT_NOISE_CHARGE  "noise_in_charge"
#define HEALTH_REPORT_HOPPING       "freq_hopping"
#define HEALTH_REPORT_TEMP_DRIFT    "temp_drift"
#define HEALTH_REPORT_CHANEL_FILL   "broken_compensated"
#define HEALTH_REPORT_FOD_ABNORMAL  "fod_abnormal"
#define HEALTH_REPORT_SHIELD_PALM   "shield_palm"
#define HEALTH_REPORT_SHIELD_EDGE   "shield_edge"
#define HEALTH_REPORT_SHIELD_METAL  "shield_metal"
#define HEALTH_REPORT_SHIELD_WATER  "shield_water"
#define HEALTH_REPORT_SHIELD_ESD    "shield_esd"
#define HEALTH_REPORT_RST_HARD      "hard_rst"
#define HEALTH_REPORT_RST_INST      "inst_rst"
#define HEALTH_REPORT_RST_PARITY    "parity_rst"
#define HEALTH_REPORT_RST_WD        "wd_rst"
#define HEALTH_REPORT_RST_OTHER     "other_rst"
#define HEALTH_REPORT_GLOVE_ENTER	"glove_enterTimes"
#define HEALTH_REPORT_FW_FD			"firmware_fd"
#define HEALTH_REPORT_BASELINE_NEGATIVE  "baseline_negative"

#define FINGERPRINT_DOWN_DETECT 0X0f
#define FINGERPRINT_UP_DETECT 0X1f

#define SUPPORT_SINGLE_TAP 3
/* bit operation */
#define SET_BIT(data, flag) ((data) |= (flag))
#define CLR_BIT(data, flag) ((data) &= ~(flag))
#define CHK_BIT(data, flag) ((data) & (flag))
#define CHK_BIT_NUM(data, flag) ((data) & (1 << (flag)))
#define VK_TAB {KEY_MENU, KEY_HOMEPAGE, KEY_BACK, KEY_SEARCH}

#define SET_GESTURE_BIT(state, state_flag, config, config_flag)\
	if (CHK_BIT(state, (1 << state_flag))) {\
		SET_BIT(config, (1 << config_flag));\
	} else {\
		CLR_BIT(config, (1 << config_flag));\
	}


#define TOUCH_BIT_CHECK           0x3FF  /*max support 10 point report.using for detect non-valid points*/
#define MAX_FW_NAME_LENGTH        60
#define MAX_EXTRA_NAME_LENGTH     60

#define MAX_DEVICE_VERSION_LENGTH 16
#define MAX_DEVICE_MANU_LENGTH    16

#define MESSAGE_SIZE              (256)

#define SMOOTH_LEVEL_NUM            6
#define SENSITIVE_LEVEL_NUM         6
#define CLICK_SENSITIVE_LEVEL_NUM   6

#define PINCTRL_STATE_SPI_ACTIVE    "oplus_spi_active"
#define PINCTRL_STATE_INT_ACTIVE    "oplus_int_active"
#define PINCTRL_STATE_RST_ACTIVE    "oplus_reset_active"
#define PINCTRL_STATE_SPI_SUSPEND   "oplus_spi_suspend"
#define PINCTRL_STATE_INT_SUSPEND   "oplus_int_suspend"
#define PINCTRL_STATE_RST_SUSPEND   "oplus_reset_suspend"

#define SYNAPTICS_PREFIX    "SY_"
#define GOODIX_PREFIX       "GT_"
#define FOCAL_PREFIX        "FT_"

#define SMART_GESTURE_THRESHOLD 0x0A
#define SMART_GESTURE_LOW_VALUE 0x05

#define FW_UPDATE_DELAY        msecs_to_jiffies(2*1000)

#define RECORD_POINTS_COUNT 5

#define REPORT_RATE_GAME_300    300
#define REPORT_RATE_GAME_600    600

#define REPORT_RATE_MAX_TEST_TIME_S    20

#ifdef TP_ALL_GESTURE_SUPPORT
#undef TP_ALL_GESTURE_SUPPORT
#endif

#ifdef TP_ALL_GESTURE_ENABLE
#undef TP_ALL_GESTURE_ENABLE
#endif

#define FP_GRIP_TAG "[fingerprint grip]"
#define FP_GRIP_ENABLE  0x01 /* 0001 */
#define FP_GRIP_DISABLE 0x02 /* 0010 */

#define TP_ALL_GESTURE_SUPPORT \
	(ts->black_gesture_support || ts->fingerprint_underscreen_support || ts->fp_grip_support)
#define TP_ALL_GESTURE_ENABLE  \
	((ts->gesture_enable & 0x01) == 1 || ts->fp_enable || (ts->fp_grip_enable & FP_GRIP_ENABLE))

/* input_dev vendor/product info*/
#define SYNAPTICS    0x0901
#define S3910        0x0901
#define S3908        0x0902
#define S3910_SECOND 0x0903

#define GOODIX       0x0902
#define GT9966       0x0901
#define GT9916       0x0902
#define GT9966_SECOND 0x0903
#define GT9916_SECOND 0x0904

#define ABS_TOUCH_COST_TIME_KERNEL  0x21
#define ABS_TOUCH_COST_TIME_ALGO    0x22
#define ABS_TOUCH_COST_TIME_DAEMON  0x23
#define MAX_TOUCH_COST_TIME         1000 * 1000

#define SCEN_SINGLE_CMD_SIZE        (64)
#define SCEN_ALL_CMD_SIZE           (8192)
#define MAX_SCENE_LENS              5
typedef enum {
	SCREEN_LOCK_MODE = 0,
	SENSITIVE_LEVEL,
	SET_PACKAGE_TYPE,
	PEN_SENSITIVE_LEVEL,
	TOUCH_LEAVE_JITTER,
} scene_type;

static const char buffer_scen[MAX_SCENE_LENS][SCEN_SINGLE_CMD_SIZE] = { \
	"screen_lock_mode", \
	"sensitive_level", \
	"set_package_type", \
	"pen_sensitive_level", \
	"touch_leave_jitter" \
};

#define NOTIFY_TIME_OUT             60

#define MAX_TEMPERATURE             70
#define MIN_TEMPERATURE             -40

#define MAX_SUSPEND_TASK_DUMP_COUNT 3
#define MAX_AIUNIT_GET_NUM          30
#define MAX_AIUNIT_SET_NUM          7

#define MAX_REPORT_FRAME_TEST_FRAME 100
/*********PART3:Struct Area**********************/
typedef enum {
	TYPE_ONCELL = 0,   /*such as synaptic s3706*/
	TYPE_TDDI,         /*such as tddi with flash or no flash*/
	TYPE_TDDI_TCM,     /*TCM with flash or no flash*/
	TYPE_IC_MAX,
} tp_ic_type;

typedef enum debug_level {
	LEVEL_BASIC,    /*printk basic tp debug info*/
	LEVEL_DETAIL,   /*printk tp detail log for stress test*/
	LEVEL_DEBUG,    /*printk all tp debug info*/
	LEVEL_DEBUG_SC_OFF, /*printk screen off debug info*/
} tp_debug_level;

typedef enum {
	MODE_NORMAL,
	MODE_SLEEP,
	MODE_EDGE,
	MODE_GESTURE,
	MODE_GLOVE,
	MODE_RAINSTORM,
	MODE_CHARGE,
	MODE_GAME,
	MODE_PALM_REJECTION,
	MODE_FACE_DETECT,
	MODE_HEADSET,
	MODE_WIRELESS_CHARGE,
	MODE_PEN_SCAN,
	MODE_PEN_CTL,
	MODE_PALM_TO_SLEEP,
	MODE_WATERPROOF,
	MODE_LEATHER_COVER,
	MODE_INCELL_AOD,
} work_mode;

typedef enum {
	GLOVE_EXIT,
	GLOVE_ENTER,
} glove_status;

typedef enum {
	TP_BUS_I2C = 0,
	TP_BUS_SPI,
	TP_BUS_I3C,
} tp_bus_type;

typedef enum {
    TP_RATE_START,
    TP_RATE_CALC,
    TP_RATE_CLEAR,
} tp_rate;
/******For FW area********/
typedef enum {
	FW_NORMAL,     /*fw might update, depend on the fw id*/
	FW_ABNORMAL,   /*fw abnormal, need update*/
} fw_check_state;

typedef enum {
	FW_UPDATE_SUCCESS,
	FW_NO_NEED_UPDATE,
	FW_UPDATE_ERROR,
	FW_UPDATE_FATAL,
} fw_update_state;

struct firmware_headfile {
	const uint8_t *firmware_data;
	size_t firmware_size;
};

struct finger_health_info {
	u32 fp_enble_count;
	u32 faraway_pressed_fod_count;
	u32 pressed_not_in_area_count;
	u32 pressed_in_area_count;
	u32 first_detected_effetive_fod_count;
	u32 small_touch_fod_invail_count;
	u32 detect_effetive_area_count;
	u32 detect_fod_id_pass_count;
};

/******For IRQ area********/
typedef enum IRQ_TRIGGER_REASON {
	IRQ_IGNORE      = 0x00,
	IRQ_TOUCH       = 0x01,
	IRQ_GESTURE     = 0x02,
	IRQ_BTN_KEY     = 0x04,
	IRQ_EXCEPTION   = 0x08,
	IRQ_FW_CONFIG   = 0x10,
	IRQ_FW_HEALTH   = 0x20,
	IRQ_FW_AUTO_RESET = 0x40,
	IRQ_FACE_STATE    = 0x80,
	IRQ_FINGERPRINT   = 0x0100,
	IRQ_PEN           = 0x0200,
	IRQ_PALM          = 0x0400,
	IRQ_PEN_REPORT    = 0x0800,
} irq_reason;

typedef enum interrupt_mode {
	BANNABLE,
	UNBANNABLE,
	INTERRUPT_MODE_MAX,
} tp_interrupt_mode;

/******For suspend and resume area********/
typedef enum {
	TP_SUSPEND_EARLY_EVENT,
	TP_SUSPEND_COMPLETE,
	TP_RESUME_EARLY_EVENT,
	TP_RESUME_COMPLETE,
	TP_SPEEDUP_RESUME_COMPLETE,
} suspend_resume_state;

typedef enum {
	NOTIFY_DEFAULT = 0,
	NOTIFY_BLANK_EARLY_ENTER,
	NOTIFY_BLANK_EARLY_EXIT,
	NOTIFY_BLANK_ENTER,
	NOTIFY_BLANK_EXIT,
} notify_state;

typedef enum switch_mode_type {
	SEQUENCE,
	SINGLE,
} tp_switch_mode;

typedef enum resume_order {
	TP_LCD_RESUME,
	LCD_TP_RESUME,
} tp_resume_order;

typedef enum suspend_order {
	TP_LCD_SUSPEND,
	LCD_TP_SUSPEND,
} tp_suspend_order;

typedef enum lcd_power {
	LCD_POWER_OFF,
	LCD_POWER_ON,
} lcd_power_status;

typedef enum lcd_event_type {
	LCD_CTL_TP_LOAD_FW = 0x10,
	LCD_CTL_RST_ON,
	LCD_CTL_RST_OFF,
	LCD_CTL_TP_FTM,
	LCD_CTL_TP_FPS60,
	LCD_CTL_TP_FPS90,
	LCD_CTL_TP_FPS120,
	LCD_CTL_TP_FPS180,
	LCD_CTL_TP_FPS240,
	LCD_CTL_CS_ON,
	LCD_CTL_CS_OFF,
	LCD_CTL_IRQ_ON,
	LCD_CTL_IRQ_OFF,
	LCD_CTL_AOD_OFF,
	LCD_CTL_AOD_ON,
} lcd_event_type;

typedef enum {
	OEM_VERIFIED_BOOT_STATE_UNLOCKED,
	OEM_VERIFIED_BOOT_STATE_LOCKED,
} oem_verified_boot_state;

/******For INPUT area********/
typedef enum {
	AREA_NOTOUCH,
	AREA_EDGE,
	AREA_CRITICAL,
	AREA_NORMAL,
	AREA_CORNER,
} touch_area;

struct Coordinate {
	int x;
	int y;
};

enum touch_direction {
	VERTICAL_SCREEN,
	LANDSCAPE_SCREEN_90,
	LANDSCAPE_SCREEN_270,
	VERTICAL_SCREEN_180,
};

struct gesture_info {
	uint32_t gesture_type;
	uint32_t clockwise;
	struct Coordinate Point_start;
	struct Coordinate Point_end;
	struct Coordinate Point_1st;
	struct Coordinate Point_2nd;
	struct Coordinate Point_3rd;
	struct Coordinate Point_4th;
	uint8_t gesture_panel_id;
};

struct pen_info {
	uint16_t    x;
	uint16_t    y;
	uint16_t    z;
	uint16_t    d;
	uint16_t    btn1;
	uint16_t    btn2;
	uint16_t    battery;
	uint16_t    speed;
	int32_t     tilt_x;
	int32_t     tilt_y;
	uint8_t     status;
	uint8_t     point_type;
	uint16_t    min_x;
	uint16_t    min_y;
	uint16_t    max_x;
	uint16_t    max_y;
};

enum tp_state_in_pen {
	DEFAULT,
	CANCLE_TP,
};

struct pen_config {
	uint32_t	max_x;
	uint32_t	max_y;
	uint32_t	max_pressure;
	int		tilt_x_max;
	int		tilt_y_max;
};

struct point_info {
	uint16_t x;
	uint16_t y;
	uint16_t z;
	uint8_t  width_major;
	uint8_t  touch_major;
	uint8_t  status;
	uint8_t  tx_press;
	uint8_t  rx_press;
	uint8_t  tx_er;
	uint8_t  rx_er;
	touch_area type;
};

struct corner_info {
	uint8_t id;
	bool flag;
	struct point_info point;
};

struct touch_major_limit {
	int width_range;
	int height_range;
};

struct fp_underscreen_info {
	uint8_t touch_state;
	uint8_t area_rate;
	uint16_t x;
	uint16_t y;
};

struct specific_resume_data {
	suspend_resume_state suspend_state;
	bool in_test_process;
};

/******For button area********/
typedef enum {
	TYPE_PROPERTIES = 1,   /*using board_properties*/
	TYPE_AREA_SEPRATE,     /*using same IC (button zone &&  touch zone are seprate)*/
	TYPE_DIFF_IC,          /*using diffrent IC (button zone &&  touch zone are seprate)*/
	TYPE_NO_NEED,          /*No need of virtual key process*/
} vk_type;

typedef enum vk_bitmap {
	BIT_reserve    = 0x08,
	BIT_BACK       = 0x04,
	BIT_HOME       = 0x02,
	BIT_MENU       = 0x01,
} vk_bitmap;

struct button_map {
	int width_x;                                        /*width of each key area */
	int height_y;                                       /*height of each key area*/
	struct Coordinate coord_menu;                       /*Menu centre coordinates*/
	struct Coordinate coord_home;                       /*Home centre coordinates*/
	struct Coordinate coord_back;                       /*Back centre coordinates*/
};

/******For FP area********/
typedef enum {
	FP_NODE_OFF = 0,
	FP_NODE_ON = 1,
	FP_NODE_FOR_LSI = 2,/*for LSI touch fp_enable node*/
	FP_QUICK_START_ON = 3,/*for fingerprint quick start featrue*/
} fp_node_value;

/******For Game Hot Zone******/
struct tp_aiunit_game_info {
	u8  gametype;
	u8  aiunit_game_type;
	u16 left;
	u16 top;
	u16 right;
	u16 bottom;
};

/******For Long Strip Abnormal Detect******/
struct long_strip_abnormal_detect_info {
	u8  channels_max_thd;
	u8  er_max;
	u8  er_min;
	u16 center_width;
};

/******For HW resource area********/
struct panel_info {
	char    *fw_name;                               /*FW name*/
	char    *test_limit_name;                       /*test limit name*/
	char    *aging_test_limit_name;                 /*aging test limit name*/
	char    *extra;                                 /*for some ic, may need other information*/
	uint32_t tp_fw;                                 /*FW Version Read from IC*/
	tp_dev  tp_type;
	int    vid_len;                                 /*Length of tp name show in  test apk*/
	u32    project_id;
	int    report_rate_limit;                       /*chip report rate limit*/
	uint32_t    panel_type[10];
	uint32_t    platform_support_project[30];
	uint32_t    platform_support_project_dir[30];
	char  *platform_support_commandline[10];
	char  *firmware_name[10];
	char  *chip_name[10];                         /*chip name the panel is controlled by*/
	int    project_num;
	int    panel_num;
	int    chip_num;
	struct firmware_headfile  firmware_headfile;
	/*firmware headfile for noflash ic*/
#ifndef CONFIG_REMOVE_OPLUS_FUNCTION
	struct manufacture_info manufacture_info;       /*touchpanel device info*/
#endif
	void *touch_custom_data;
};

struct hw_resource {
	int irq_gpio;                                   /*irq GPIO num*/
	int reset_gpio;                                 /*Reset GPIO*/
	int cs_gpio;                                    /*CS GPIO*/
	int enable_avdd_gpio;                             /*avdd enable GPIO*/
	int enable_vddi_gpio;                             /*vddi enable GPIO*/

	/*TX&&RX Num*/
	int tx_num;
	int rx_num;
	int pen_tx_num;
	int pen_rx_num;
	int key_tx;                                     /*the tx num occupied by touchkey*/
	int key_rx;                                     /*the rx num occupied by touchkey*/

	/*power*/
	struct regulator *avdd;                      /*power avdd 2.8-3.3v*/
	struct regulator *vddi;                      /*power vddi 1.8v*/
	uint32_t vdd_volt;                              /*avdd specific volt*/
	uint32_t vddi_volt;                              /*vddi specific volt*/

	/*pinctrl*/
	struct pinctrl          *pinctrl;
	struct pinctrl_state    *pin_set_high;
	struct pinctrl_state    *pin_set_low;
	struct pinctrl_state    *pin_set_nopull;
	struct pinctrl_state    *pin_spi_mode_active;
	struct pinctrl_state    *pin_spi_mode_suspend;
	struct pinctrl_state    *pin_int_sta_active;
	struct pinctrl_state    *pin_int_sta_suspend;
	struct pinctrl_state    *pin_rst_sta_active;
	struct pinctrl_state    *pin_rst_sta_suspend;
	struct pinctrl_state    *pin_cs_high;
	struct pinctrl_state    *pin_cs_low;
};

struct resolution_info {
	uint32_t max_x;                                     /*touchpanel width */
	uint32_t max_y;                                     /*touchpanel height*/
	uint32_t LCD_WIDTH;                                 /*LCD WIDTH        */
	uint32_t LCD_HEIGHT;                                /*LCD HEIGHT       */
};

/******For debug node area********/
struct esd_information {
	bool esd_running_flag;
	int  esd_work_time;
	struct mutex             esd_lock;
	struct workqueue_struct *esd_workqueue;
	struct delayed_work      esd_check_work;
};

struct freq_hop_info {
	struct workqueue_struct *freq_hop_workqueue;
	struct delayed_work      freq_hop_work;
	bool freq_hop_simulating;
	int freq_hop_freq;                                  /*save frequency-hopping frequency, trigger frequency-hopping every freq_hop_freq seconds*/
};

struct register_info {
	uint8_t reg_length;
	uint16_t reg_addr;
	uint8_t *reg_result;
};

/******For auto test area********/
struct black_gesture_test {
	bool gesture_backup;                    /*store the gesture enable flag */
	bool flag;                                         /* indicate do black gesture test or not*/
	char *message;                               /* failure information if gesture test failed */
	int message_size;                           /* failure information size if gesture test failed */
};

struct com_test_data {
	const struct firmware *limit_fw;      /*test limit fw*/
	const struct firmware *black_test_fw;      /*test limit fw*/
	void *chip_test_ops;
	/*save auto test result data*/
	void *result_data;
	size_t result_max_len;
	size_t result_flag;
	size_t result_cur_len;
	/*save black screen test result data*/
	void *bs_result_data;
	size_t bs_result_max_len;
	size_t bs_result_flag;
	size_t bs_result_cur_len;
	/*raw cap test max min test*/
	int raw_cap_restriction;
};

/******For health monitor area********/
typedef enum {
	STATE_UP = 0,
	STATE_DOWN,
} down_up_state;

typedef enum {
	ACTION_CLICK = 0,
	ACTION_SWIPE,
} touch_action;

typedef enum {
	TYPE_RECORD_INT = 0,
	TYPE_RECORD_STR,
} value_record_type;

typedef enum {
	TYPE_NONE,
	TYPE_START_RECORD,
	TYPE_END_RECORD,
} grip_time_record_type;

typedef enum {
	TYPE_PENCIL_HAVON = 1,
	TYPE_PENCIL_MAXEYE = 2,
	TYPE_PENCIL_MAXEYE_2ND = 3,
	TYPE_PENCIL_SUNWODA = 4,
	TYPE_PENCIL_MAXEYE_3RD = 5,
} pencil_type;

typedef enum {
	HEALTH_SIMULATE_BIT_IRQ_GPIO = 0,
	HEALTH_SIMULATE_BIT_AVDD_VDDI,
	HEALTH_SIMULATE_BIT_ESD,
	HEALTH_SIMULATE_BIT_MODE_SWITCH,
	HEALTH_SIMULATE_BIT_BUS,
	HEALTH_SIMULATE_BIT_IC_HEALTHINFO = 5,
	HEALTH_SIMULATE_BIT_FW_UPDATE,
} health_simulate_bit;

typedef enum {
	REPORT_FPS_DEFAULT = 0,
	REPORT_FPS_60HZ,
	REPORT_FPS_125HZ,
	REPORT_FPS_180HZ,
	REPORT_FPS_240HZ,
	REPORT_FPS_288Hz, /* actually 300hz */
	REPORT_FPS_360HZ,
	REPORT_FPS_720HZ,
	REPORT_FPS_70HZ,
	REPORT_FPS_140HZ,
	REPORT_FPS_140HZ_INSERT,
	REPORT_FPS_250HZ_INSERT,
	REPORT_FPS_144HZ, /* actually 150hz */
	REPORT_FPS_165HZ, /* actually 173hz */
	REPORT_FPS_330HZ, /* actually 347hz */
	REPORT_FPS_90HZ,
	REPORT_FPS_180HZ_INSERT,
	REPORT_FPS_RATE_70  = 70,
	REPORT_FPS_RATE_120 = 120,
	REPORT_FPS_RATE_140 = 140,
	REPORT_FPS_RATE_140_INSERT = 141,
	REPORT_FPS_RATE_144 = 144,
	REPORT_FPS_RATE_180 = 180,
	REPORT_FPS_RATE_180_INSERT = 181,
	REPORT_FPS_RATE_240 = 240,
	REPORT_FPS_RATE_240_INSERT = 241,
	REPORT_FPS_RATE_288 = 288,
	REPORT_FPS_RATE_300 = 300,
	REPORT_FPS_RATE_360 = 360,
	REPORT_FPS_RATE_600 = 600,
	REPORT_FPS_RATE_720 = 720,
} ReportFps;

struct point_state_monitor {
	u64 time_counter;
	struct point_info first_point;
	struct point_info last_point;
	unsigned long last_swipe_distance_sq;
	unsigned long max_swipe_distance_sq;
	int jumping_times;
	down_up_state down_up_state;
	touch_action touch_action;
	bool is_down_handled;
};

struct health_value_count {
	struct list_head head;
	value_record_type value_type;
	void *value;
	int count;
};

struct points_record {
	int count;
	struct Coordinate points[RECORD_POINTS_COUNT];
};

struct swipes_record {
	int count;
	struct Coordinate start_points[RECORD_POINTS_COUNT];
	struct Coordinate end_points[RECORD_POINTS_COUNT];
};

struct irq_type_count{
	u64 in_suspend_irq_ignore_cnt;
	u64 in_resume_irq_ignore_cnt;
	u64 in_resume_irq_touch_cnt;
	u64 in_suspend_irq_gesture_cnt;
	u64 in_suspend_irq_btn_key_cnt;
	u64 in_resume_irq_btn_key_cnt;
	u64 in_suspend_irq_exception_cnt;
	u64 in_resume_irq_exception_cnt;
	u64 in_suspend_irq_fw_config_cnt;
	u64 in_resume_irq_fw_config_cnt;
	u64 in_resume_irq_fw_health_cnt;
	u64 in_suspend_irq_fw_auto_reset_cnt;
	u64 in_resume_irq_fw_auto_reset_cnt;
	u64 in_resume_irq_face_state_cnt;
	u64 in_suspend_irq_fingerprint_cnt;
	u64 in_resume_irq_fingerprint_cnt;
	u64 in_suspend_irq_pen_cnt;
	u64 in_resume_irq_pen_cnt;
	u64 in_resume_irq_palm_cnt;
	u64 in_suspend_irq_pen_report_cnt;
	u64 in_resume_irq_pen_report_cnt;
	u64 abnormal_in_suspend_irq_palm_cnt;
	u64 abnormal_in_suspend_irq_touch_cnt;
	u64 abnormal_in_suspend_irq_fw_health_cnt;
	u64 abnormal_in_suspend_irq_face_state_cnt;
	u64 abnormal_in_resume_irq_gesture_cnt;
};

struct monitor_data {
	void  *chip_data; /*debug info data*/
	struct debug_info_proc_operations  *debug_info_ops; /*debug info data*/
	uint32_t health_simulate_trigger;

	u64 boot_time;
	u64 stat_time;
	u64 probe_time;
	u64 max_resume_time;
	u64 max_suspend_time;
	u64 max_fw_update_time;
	char *fw_version;
	const char *tp_ic;
	char *vendor;

	bool health_monitor_support;
	bool kernel_grip_support;
	int max_finger_support;
	int tx_num;
	int rx_num;
	uint32_t max_x;
	uint32_t max_y;
	uint32_t display_max_x;
	uint32_t display_max_y;
	uint32_t long_swipe_judge_distance;
	uint32_t swipe_broken_judge_distance;
	uint32_t jumping_point_judge_distance;

	int report_rate;
	int report_rate_in_game;
	int in_game_mode;

	struct point_info *touch_points;
	int touch_num;
	int direction;

	struct point_state_monitor *points_state;

	struct list_head        dead_zone_list;
	struct list_head        condition_zone_list;
	struct list_head        elimination_zone_list;
	struct points_record    dead_zone_points;
	struct points_record    condition_zone_points;
	struct points_record    elimination_zone_points;
	struct points_record    lanscape_elimination_zone_points;
	int elizone_point_tophalf_i;
	int elizone_point_bothalf_i;

	int32_t *jumping_points_count_array;
	int32_t *stuck_points_count_array;
	int32_t *lanscape_stuck_points_count_array;
	int32_t *broken_swipes_count_array;
	struct swipes_record    long_swipes;

	int32_t *jumping_point_delta_data;
	int32_t *stuck_point_delta_data;

	int max_jumping_times;
	int max_touch_num;
	int max_touch_num_in_game;
	int current_touch_num;

	int click_count;
	int swipe_count;
	/*int32_t *click_count_array;*/

	u64 touch_timer;
	u64 holding_touch_time;
	u64 total_touch_time;
	u64 *total_touch_time_in_game;
	u64 *total_touch_time_in_game_300;
	u64 *total_touch_time_in_game_600;
	u64 max_holding_touch_time;

	ktime_t irq_to_report_timer;

	u64 total_grip_time_no_touch;
	u64 total_grip_time_no_touch_one_sec;
	u64 total_grip_time_no_touch_two_sec;
	u64 total_grip_time_no_touch_three_sec;
	u64 total_grip_time_no_touch_five_sec;
	u64 grip_start_time_no_touch;
	grip_time_record_type grip_time_record_flag;
	struct grip_monitor_data  *p_grip_moni_data;
	struct irq_type_count  *p_irq_type_count;

	u32 edge_tx_ewr_zero_count;
	u32 edge_rx_ewr_zero_count;
	u32 corner_trx_ewr_zero_count;
	u32 center_tx_ewr_not_zero_count;
	u32 center_rx_ewr_not_zero_count;

	/*u32 point_x_zero_count;*/
	u32 point_x_over_max_count;
	/*u32 point_y_zero_count;*/
	u32 point_y_over_max_count;

	u64 screenon_timer;
	u64 total_screenon_time;

	u64 glove_en_timer;
	u64 total_glove_en_time;
	u64 glove_enter_count;

	int auto_test_total_times;
	int auto_test_failed_times;

	int blackscreen_test_total_times;
	int blackscreen_test_failed_times;

	int gesture_waiting;
	bool is_gesture_waiting_read;
	u64 gesture_received_time;

	struct list_head        gesture_values_list;
	struct list_head        invalid_gesture_values_list;
	struct list_head        fp_area_rate_list;
	struct list_head        fd_values_list;
	struct list_head        health_report_list;
	struct list_head        bus_errs_list;
	struct list_head        bus_errs_buff_list;
	struct list_head        alloc_err_funcs_list;
	struct list_head        fw_update_result_list;

	unsigned char *bus_buf;
	uint16_t bus_len;

	long alloced_size;
	long max_alloc_err_size;
	long min_alloc_err_size;

	u32 smooth_level_chosen;
	u32 sensitive_level_chosen;
	u32 click_sensitive_level_chosen;
	int RATE_MIN;
	int below_rate_counts;
	tp_rate tp_rate_type;
	int work_freq;

	int avdd;
	int vddi;
	u64 pm_resume_count;
	u64 pm_suspend_count;
	u64 force_bus_ready_count;
	u64 bus_not_ready_early_event_count;
	u64 bus_not_ready_event_count;
	u64 bus_not_ready_notify_count;
	u64 bus_not_ready_off_early_event_count;
	u64 bus_not_ready_off_event_count;
	u64 bus_not_ready_tp_suspend_count;
	u64 bus_not_ready_gesture_write_count;
	u64 bus_not_ready_temperature_work_count;
	u64 wait_for_notify_suspend_count;
	u64 abnormal_temperature_count;
	/*max count*/
	u64 irq_need_dev_resume_max_count;
	/*all count*/
	u64 irq_need_dev_resume_all_count;
	u64 irq_bus_not_ready_count;

	struct finger_health_info *p_finger_health_info;
};

#define MAX_BUS_ERROR_COUNT         10
#define MAX_BUS_ERROR_UPLOAD_COUNT  2
#define MAX_BUS_READY_UPLOAD_COUNT  3

struct exception_data {
	void  *chip_data; /*debug info data*/
	bool exception_upload_support;
	u32 exception_upload_count;
	u32 bus_error_count;
	u32 bus_error_upload_count;
	u32 bus_ready_upload_count;
};

struct com_api_data {
	spinlock_t tp_irq_lock;
	int tp_irq_disable;
};

typedef enum {
	CHARGE = 1,
	HEADSET,
	WIRELESS_CHARGE,
} misc_device_type;

/******For debug apk area********/
/*#define CONFIG_OPLUS_TP_APK please define this in arch/arm64/configs*/
#ifdef CONFIG_OPLUS_TP_APK

typedef enum {
	APK_NULL       = 0,
	APK_CHARGER    = 'C',
	APK_DATA       = 'D',
	APK_EARPHONE   = 'E',
	APK_GESTURE    = 'G',
	APK_INFO       = 'I',
	APK_NOISE      = 'N',
	APK_PROXIMITY  = 'P',
	APK_WATER      = 'W',
	APK_DEBUG_MODE = 'd',
	APK_GAME_MODE  = 'g'
} APK_SWITCH_TYPE;

typedef enum {
	DATA_NULL   = 0,
	BASE_DATA   = 'B',
	DIFF_DATA   = 'D',
	DEBUG_INFO  = 'I',
	RAW_DATA    = 'R',
	BACK_DATA   = 'T'
} APK_DATA_TYPE;

typedef struct apk_proc_operations {
	void (*apk_game_set)(void *chip_data, bool on_off);
	bool (*apk_game_get)(void *chip_data);
	void (*apk_debug_set)(void *chip_data, bool on_off);
	bool (*apk_debug_get)(void *chip_data);
	void (*apk_noise_set)(void *chip_data, bool on_off);
	bool (*apk_noise_get)(void *chip_data);
	void (*apk_water_set)(void *chip_data, int type);
	int (*apk_water_get)(void *chip_data);
	void (*apk_proximity_set)(void *chip_data, bool on_off);
	int (*apk_proximity_dis)(void *chip_data);
	void (*apk_gesture_debug)(void *chip_data, bool on_off);
	bool (*apk_gesture_get)(void *chip_data);
	int (*apk_gesture_info)(void *chip_data, char *buf, int len);
	void (*apk_earphone_set)(void *chip_data, bool on_off);
	bool (*apk_earphone_get)(void *chip_data);
	void (*apk_charger_set)(void *chip_data, bool on_off);
	bool (*apk_charger_get)(void *chip_data);
	int (*apk_tp_info_get)(void *chip_data, char *buf, int len);
	void (*apk_data_type_set)(void *chip_data, int type);
	int (*apk_rawdata_get)(void *chip_data, char *buf, int len);
	int (*apk_diffdata_get)(void *chip_data, char *buf, int len);
	int (*apk_basedata_get)(void *chip_data, char *buf, int len);
	int (*apk_backdata_get)(void *chip_data, char *buf, int len);
} APK_OPERATION;

#endif

#define SNR_RESET(snr)  \
	do{\
		snr.max = 0; snr.min = 0; snr.sum = 0; snr.noise = 0;\
	} while(0)

struct touchpanel_snr {
	uint16_t x;
	uint16_t y;
	uint8_t channel_x;
	uint8_t channel_y;
	uint8_t width_major;
	uint8_t area_len;
	uint8_t point_status;
	u64 *data;
	bool doing;
	int max;
	int min;
	int sum;
	int noise;
};

struct touchpanel_last_x_y_point {
	uint16_t x;
	uint16_t y;
};

struct touch_scene_info {
	uint16_t  screen_lock_mode;
	uint16_t  sensitive_level;
	uint16_t  set_package_type;
	uint16_t  pen_sensitive_level;
	uint16_t  touch_leave_jitter;
};

struct aging_test_proc_operations;
struct debug_info_proc_operations;
struct touchpanel_data {
	/******For feature area********/
	bool register_is_16bit;                             /*register is 16bit*/
	bool glove_mode_support;                            /*glove_mode support feature*/
	bool glove_mode_v2_support;                         /*glove_mode support feature*/
	bool rainstorm_mode_v2_support;                     /*rainstorm_mode support feature*/
	bool leather_cover_mode_support;                    /*leather_cover support feature*/
	bool black_gesture_support;                         /*black_gesture support feature*/
	bool black_gesture_indep_support;                   /*black_gesture indep control support feature*/
	bool charger_pump_support;                          /*charger_pump support feature*/
	bool wireless_charger_support;                      /*wireless_charger support feature*/
	bool headset_pump_support;                          /*headset_pump support feature*/
	bool fw_edge_limit_support;                         /*edge_limit by FW support feature*/
	bool report_rate_v2_support;                         /*report rate support feature*/
	bool esd_handle_support;                            /*esd handle support feature*/
	bool gesture_test_support;                          /*indicate test black gesture or not*/
	bool game_switch_support;                           /*indicate game switch support or not*/
	bool game_enable_in_tddi_support;                   /*send game cmd to tddi support feature*/
	bool face_detect_support;                           /*touch porximity function*/
	bool fingerprint_underscreen_support;               /*fingerprint underscreen support*/
	bool fingerprint_not_report_in_suspend;             /*fingerprint not report in suspending*/
	bool fingerprint_error_report_support;              /*fingerprint error report support*/
	bool sec_long_low_trigger;                          /*samsung s6d7ate ic int feature*/
	bool suspend_gesture_cfg;
	bool auto_test_force_pass_support;                  /*auto test force pass in early project*/
	bool freq_hop_simulate_support;                     /*frequency hopping simulate feature*/
	bool lcd_trigger_load_tp_fw_support;                /*trigger load tp fw by lcd driver after lcd reset*/
	bool fw_update_app_support;                         /*bspFwUpdate is used*/
	bool health_monitor_support;                        /*health_monitor is used*/
	bool irq_trigger_hdl_support;                       /*some no-flash ic (such as TD4330) need irq to trigger hdl*/
	bool noise_modetest_support;                        /*noise mode test is used*/
	bool fw_update_in_probe_with_headfile;
	bool optimized_show_support;                       /*support to show total optimized time*/
	bool regulator_count_not_support;
	uint32_t single_optimized_time;                    /*single touch optimized time*/
	uint32_t total_operate_times;                      /*record total touch down and up count*/
	struct firmware                 *firmware_in_dts;
	bool kernel_grip_support;                           /*using grip function in kernel touch driver*/
	bool grip_no_driver_support;
	bool high_frame_rate_support;
	uint32_t high_frame_rate_time;
	bool snr_read_support;                              /*feature to support reading snr data*/
	bool trusted_touch_support;                         /*feature to support tui function*/
	int tp_index;                                       /*default is 0 for single panel*/
	int chip_index;                                     /*feature to support for one more panel dts*/
	bool exception_upload_support;
	bool report_rate_support;                           /*feature used to calculate report rate*/
	bool major_rate_limit_support;                      /*feature used to major report rate limit*/
	bool touchz_to_pressure_support;                    /*feature used to report touchZ to Pressure */
	bool palm_to_sleep_support;                         /*feature used to sleep when device trigger palm gesture in screen lock*/
	bool tp_data_record_support;                        /*feature used to data record when get tp log*/
	bool suspend_work_support;                          /*feature used to support suspend work queue*/
	int glove_enable;                                   /*control state of glove gesture*/
	int rainstorm_enable;                               /*control state of rainstorm mode*/
	int pocket_prevent_mode;
	int leather_cover_enable;                           /*control state of leather_cover gesture*/
	bool force_bus_ready_support;                       /*force bus ready to true afer notify*/
	bool skip_reinit_device_support;                    /*spi need skip complete_all, prevent error in access reg*/
	bool edge_pull_out_support;                         /*feature used to edge coordinates pull out*/
	bool aiunit_game_info_support;                      /*feature used to aiunit game info*/
	bool fpga_support;
	bool disable_touch_event_support;                      /*feature to support underwater photo*/
	bool input_timestamp_in_top_irq_support;                  /* set input time when top half of interrupt*/
	bool screenshot_not_reset_support;               /*screenshot disable tp reset*/
	bool fp_grip_support;                               /* edge grip for fingerprint */
	bool long_strip_abnormal_detect_support;
	bool fp_grip_hold;
	bool idle_freq_support;
	int  fp_grip_enable;
	int idle_freq_enable;
	u8 aiunit_game_get_num;
	u8 aiunit_game_set_num;
	int aiunit_game_enable;
	u32 aiunit_game_valid_bits;
	/******For FW update area********/
	bool loading_fw;                                    /*touchpanel FW updating*/
	int firmware_update_type;                           /*firmware_update_type: 0=check firmware version 1=force update; 2=for FAE debug*/
	struct completion fw_complete;						/*completion for control fw update*/
	struct work_struct     fw_update_work;              /*using for fw update*/
	/*trigger load tp fw by lcd driver after lcd reset*/
	struct work_struct lcd_trigger_load_tp_fw_work;
	/*trigger laod tp fw by lcd driver after lcd reset*/
	struct workqueue_struct *lcd_trigger_load_tp_fw_wq;
	bool lpwg_fw_support;				    /*feature to support td4377 lpwg_fw_support*/

	/******For auto test area********/
	bool in_test_process;                     /*flag whether in test process*/
	struct black_gesture_test gesture_test;  /*screen off gesture test struct*/
	struct com_test_data com_test_data;	/*test comon data*/
	struct engineer_test_operations   *engineer_ops;     /*call_back function*/
	bool auto_test_need_cal_support;
	bool sportify_aod_gesture_support;
	bool incell_aod_gesture_support;
	/******For button key area********/
	/*every bit declear one state of key "reserve(keycode)|home(keycode)|menu(keycode)|back(keycode)"*/
	u8   vk_bitmap;
	vk_type  vk_type;                           /*virtual_key type*/
	struct button_map      button_map;          /*virtual_key button area*/

	/******For irq flow area********/
	uint32_t irq_flags;                      /*irq setting flag*/
	int irq;                                 /*irq num*/
	uint32_t irq_flags_cover;                /*cover irq setting flag*/
	tp_interrupt_mode int_mode;	/*whether interrupt and be disabled*/
	wait_queue_head_t wait;
	bool up_status;
	int touch_report_num;
	int touch_frame_num;
	u32 max_report_rate_limit;
	struct point_info last_point;
	int last_width_major;
	int last_touch_major;
	u8 major_rate_limit_times;
	int point_num;
	char irq_name[TP_NAME_SIZE_MAX];

	/******For gesture area********/
	bool disable_gesture_ctrl;                          /*when lcd_trigger_load_tp_fw start no need to control gesture*/
	int irq_need_dev_resume_time;                       /*control setting of wait resume time*/
	int gesture_enable;                                 /*control state of black gesture*/
	struct gesture_info    gesture;                     /*gesture related info*/
	int gesture_enable_indep;                         /*independent control state of black gesture*/
	bool incell_aod_flag;

	/******For fingerprint area********/
	int fp_enable;                                      /*underscreen fingerprint enable or not*/
	int fp_quick_start_data;                            /*for fingerprint quick start featrue*/
	int fp_disable_after_resume;
	struct fp_underscreen_info fp_info;	/*tp info used for underscreen fingerprint*/
	struct thermal_zone_device *oplus_shell_themal;
	/* fp variables */
	int fp_down_time;
	int fp_up_time;
	int fp_down_cnt;
	int fp_up_cnt;
	bool fp_unlock_status_support;
	u8  fp_unlock_status;

	/******For pm suspend and resume area********/
	bool bus_ready;                                     /*spi or i2c resume status*/
	int limit_edge;                                     /*control state of limit edge*/
	int is_suspended;                                   /*suspend/resume flow exec flag*/
	suspend_resume_state suspend_state;	/*detail suspend/resume state*/
	tp_resume_order tp_resume_order;
	tp_suspend_order tp_suspend_order;
	bool skip_reset_in_resume;                          /*some incell ic is reset by lcd reset*/

	/*LCD and TP is in one chip,lcd power off in suspend at first, can not operate i2c when tp suspend*/
	bool skip_suspend_operate;
	/******For hw resource area********/
	struct panel_info panel_data;	/*GPIO control(id && pinctrl && tp_type)*/
	struct hw_resource     hw_res;                      /*hw resourc information*/
	struct resolution_info resolution_info;	/*resolution of touchpanel && LCD*/
	/*used for control touch major reporting area*/
	struct touch_major_limit touch_major_limit;


	/******For debug node area********/
	struct esd_information  esd_info;                    /*debug info esd check*/
	struct freq_hop_info    freq_hop_info;                /*debug info freq_hop*/
	bool in_force_water_mode;                /*debug info force_water_mode*/
	unsigned short water_mode;                /*debug info force_water_mode*/
	/*struct proc_dir_entry of "/proc/touchpanel"*/
	struct proc_dir_entry *prEntry_tp;
	/*struct proc_dir_entry of "/proc/touchpanel/debug_info"*/
	struct proc_dir_entry *prEntry_debug_tp;

	struct debug_info_proc_operations  *debug_info_ops; /*debug info data*/
	struct aging_test_proc_operations  *aging_test_ops;
	struct register_info reg_info;	/*debug node for register length*/
	struct monitor_data    monitor_data;                /*health monitor data*/
	struct touchpanel_snr   snr[MAX_FINGER_NUM];        /*snr data*/
	struct touchpanel_last_x_y_point   last_x_y_point[MAX_FINGER_NUM];        /*last_x_y_point data*/
	struct exception_data    exception_data;			/*exception_data monitor data*/

	/******For Game Hot Zone*******/
	struct tp_aiunit_game_info tp_ic_aiunit_game_info[MAX_AIUNIT_SET_NUM];    /*tp ic aiunit game info*/
	struct tp_aiunit_game_info tp_get_aiunit_game_info[MAX_AIUNIT_GET_NUM];   /*tp get aiunit game info*/

	/******For prevention area********/
	struct mutex		report_mutex;                /*mutex for lock input report flow*/
	struct kernel_grip_info *grip_info;	/*grip setting and resources*/

	/******For comon data area********/
	struct mutex		mutex;                       /*mutex for lock i2c related flow*/
	struct device         *dev;                         /*used for i2c->dev*/
	struct i2c_client     *client;                        /*used for i2c client*/
	struct spi_device     *s_client;                    /*used for spi client*/
	struct input_id       id;                           /*used for record input_dev id*/
	struct input_dev      *input_dev;                   /*used for touch*/
	struct input_dev      *kpd_input_dev;               /*used for key*/
	struct input_dev      *ps_input_dev;                /*used for face dectet*/
	struct input_dev      *pen_input_dev;               /*used for pen*/
	struct oplus_touchpanel_operations *ts_ops;          /*call_back function*/

	void    *chip_data;                                 /*Chip Related data*/
	void    *private_data;                              /*Reserved Private data*/
	tp_ic_type    tp_ic_type;
	/******For TP report rate data********/
    int irq_num;/*Record the tp irq number*/
    u64 curr_time;/*Record the interruption time to kernel*/
    u64 irq_interval;/*Record the interruption time to calculate the reporting rate*/
    u64 irq_handle_time;/*Record the interruption handle time*/
	u64 touch_down_time;/*Record the touch down time*/
	/******For other feature area********/
	struct wakeup_source *ws;                           /*Qualcomm KBA-211220012446, To make power manager stay awake*/
	struct wakeup_source *tp_wakelock;  				/* speed_resume add  awakelock*/
	bool is_incell_panel;                               /*touchpanel is incell*/
	bool disable_suspend_irq_handler;                   /*touchpanel is support disable suspend irq handler */
	bool is_noflash_ic;                                 /*noflash ic*/
	bool report_rate_testing;                            /*is report rate testing*/
	tp_bus_type bus_type;                                 /* tp bus type*/
	int palm_enable;                                    /*palm enable or not*/
	int fd_enable;                                        /*face dectet enable or not*/
	int touch_count;                                    /*touch number*/
	int boot_mode;                                      /*boot up mode */
	int view_area_touched;                              /*view area touched flag*/
	int force_update;                                   /*force update flag*/
	int max_num;                                        /*max muti-touch num supportted*/
	int irq_slot;                                       /*debug use, for print all finger's first touch log*/
	uint8_t	report_point_first_enable;               /*reporting points first enable :1 ,disable 0;*/
	int noise_level;                                    /*for game mode control*/
	int high_frame_value;
	int limit_enable;                                   /*control state of limit enable */
	int edge_limit_switch_write_value;                  /*control limit_switch enable */
	int report_rate_write_value;                        /*report rate, 90: 90HZ, 120:120HZ, 240:240HZ*/
	int tp_ic_touch_num;                                 /*tp ic get touch num */
	int last_tp_ic_touch_num;                            /*last tp ic get touch num */
	int pen_mode_tp_state;
	u32 get_frame_num;
	u16 report_rate_test_time;
	u32 touch_major_sum;


	int palm_to_sleep_enable;                            /*detect palm need to sleep when device in Screen lock*/

	int waterproof_support;
	int waterproof;

	bool report_rate_white_list_support;

	bool hall_status;                                   /*control state of hall status*/
	bool is_hall_near_resume;                           /*state of hall status when tp resume*/
	bool pen_support;                                   /*support pen control*/
	bool pen_support_opp;                               /*support pen private protocol*/
	bool no_need_osctest;
	bool tp_scene_para_switch_support;
	bool bus_ready_check_support;                       /*not transfer if bus_ready false*/
	bool tp_lcd_suspend_in_lp_support;                  /*tp suspend before lcd set lp(aod) mode*/
	bool is_pen_attracted;                              /*state of pen attract*/
	bool is_pen_connected;                              /*state of pen connect*/
	struct pen_config		pen_config;
	struct message_list *pen_msg_list;
	int pen_init_success;

	bool fpga_spi_agg_support;

	int rate_ctrl_level;
	bool temperature_detect_support;
	bool temperature_detect_shellback_support;
	int old_temp;
	struct iio_channel *skin_therm_chan;
	struct hrtimer		temp_timer;
	struct work_struct get_temperature_work;
	struct touch_scene_info scene_info;
	struct long_strip_abnormal_detect_info long_strip_abnormal_detect;

	/******For fb notify area********/
	struct work_struct     speed_up_work;               /*using for speedup resume*/
	struct work_struct     suspend_work;               /*using for speedup resume*/
	/*using for touchpanel speedup resume wq*/
	struct workqueue_struct *speedup_resume_wq;
	struct workqueue_struct *suspend_wq;
#if IS_ENABLED(CONFIG_DRM_OPLUS_PANEL_NOTIFY)
	struct drm_panel *active_panel;
	struct notifier_block fb_notif; /*register to control suspend/resume*/
#elif IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
	struct drm_panel *active_panel;
	void *notifier_cookie;
#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
	struct notifier_block disp_notifier;
#elif IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY) || IS_ENABLED(CONFIG_FB)
	struct notifier_block fb_notif;	/*register to control suspend/resume*/
#endif
	notify_state notify_state;	/*detail notify state*/
	wait_queue_head_t notify_wait; /*notify wait*/

	/******For usb or headset notify area********/
	bool is_headset_checked;                            /*state of headset for usb*/
	int cur_headset_state;                                /*current state of headset for usb*/

	bool is_usb_checked;                                /*state of charger for tp*/
	int disable_touch_event;                            /*disable touch event for underwater photo mode*/
	int cur_usb_state;                                    /*current state of charger for usb*/
	bool is_wireless_checked;                           /*state of wireless charger*/

	struct workqueue_struct *charger_pump_wq;           /*state of charger or usb*/
	struct work_struct     charger_pump_work;           /*state of charger or usb*/
	struct workqueue_struct *key_trigger_wq;            /*state of volume_key trigger*/
	struct work_struct     key_trigger_work;            /*state of volume_key trigger*/
	struct workqueue_struct *headset_pump_wq;           /*state of headset or usb*/
	struct work_struct     headset_pump_work;           /*state of headset or usb*/
	/******For debug apk area********/
#ifdef CONFIG_OPLUS_TP_APK
	APK_OPERATION *apk_op;
	APK_SWITCH_TYPE type_now;
	APK_DATA_TYPE data_now;
	u8 *log_buf;
	u8 *gesture_buf;
	bool gesture_debug_sta;
#endif
	/******For QOS area********/
#ifdef CONFIG_TOUCHIRQ_UPDATE_QOS

	struct pm_qos_request pm_qos_req;

	int pm_qos_value;
	int pm_qos_state;
#endif
	/*******i2c spi bus data*****/
	struct interface_data interface_data;
	/*******comon api data*****/
	struct com_api_data com_api_data;

	/******For aging test area********/
	int aging_test;
	int aging_mode;

	/******For smooth sensitive area********/
	bool smooth_level_array_support;
	bool sensitive_level_array_support;
	bool click_sensitive_level_array_support;
	u32 smooth_level_array[SMOOTH_LEVEL_NUM];
	u32 smooth_level_charging_array[SMOOTH_LEVEL_NUM];
	u32 sensitive_level_array[SENSITIVE_LEVEL_NUM];
	u32 sensitive_level_charging_array[SENSITIVE_LEVEL_NUM];
	u32 click_sensitive_level_array[CLICK_SENSITIVE_LEVEL_NUM];
	u32 *smooth_level_used_array;
	u32 *sensitive_level_used_array;
	u32 *click_sensitive_level_used_array;
	u32 smooth_level_chosen;
	u32 sensitive_level_chosen;
	u32 click_sensitive_level_chosen;
	u32 smooth_level_default;
	u32 sensitive_level_default;

	/******For smooth report_threshold area********/
	bool diaphragm_touch_support;
	u32 diaphragm_touch_level_chosen;

	/******For lcd fps area********/
	bool lcd_tp_refresh_support;                      /*lcd nofity tp refresh fps switch*/
	int lcd_fps;                                      /*save lcd refresh*/
	struct work_struct     tp_refresh_work;            /*using for tp_refresh resume*/
	struct workqueue_struct *tp_refresh_wq;            /*using for tp_refresh wq*/

	/******For log area********/
	int is_update_log;

	/******For other area********/
	bool enable_point_auto_change;
	struct miscdevice misc_device;
	bool misc_opened;
	struct message_list *msg_list;
	int ioc_init_size;
	char *ioc_init_buf;
	u8 en_touch_event_helper;
	const char *touch_environment;
	int communicate_test_cnt;
#ifdef CONFIG_TOUCHPANEL_TRUSTED_TOUCH
	struct trusted_touch_vm_info *vm_info;
	struct mutex clk_io_ctrl_mutex;
	struct completion trusted_touch_powerdown;
	struct clk *core_clk;
	struct clk *iface_clk;
	atomic_t trusted_touch_initialized;
	atomic_t trusted_touch_enabled;
	atomic_t trusted_touch_transition;
	atomic_t trusted_touch_event;
	atomic_t trusted_touch_abort_status;
	atomic_t delayed_vm_probe_pending;
	atomic_t trusted_touch_mode;
	int bus_type_tvm;
	int te_irq;
	struct mutex transition_lock;
	spinlock_t irq_lock;
	bool irq_disabled;
	uint32_t irq_tui_flags;
#endif
	struct task_struct *suspend_task;
	int supspend_task_error_cnt;
};

#ifdef CONFIG_OPLUS_TP_APK
void log_buf_write(struct touchpanel_data *ts, u8 value);
#endif

struct engineer_test_operations {
	int (*black_screen_test)(struct black_gesture_test *p,
				 struct touchpanel_data *ts);                 /*message of black gesture test*/
	int (*auto_test)(struct seq_file *s,
			 struct touchpanel_data *ts);         /*message of auto test*/
};

struct oplus_touchpanel_operations {
	int (*get_chip_info)(void *chip_data);	/*return 0:success;other:failed*/
	int (*mode_switch)(void *chip_data, work_mode mode,
			   int flag); /*return 0:success;other:failed*/
	int (*get_touch_points)(void *chip_data, struct point_info *points,
				int max_num);   /*return point bit-map*/
	int (*get_gesture_info)(void *chip_data,
				struct gesture_info *gesture);             /*return 0:success;other:failed*/
	int (*ftm_process)(void	   *chip_data);               /*ftm boot mode process*/
	void (*ftm_process_extra)(void *chip_data);
	int (*get_vendor)(void *chip_data,
			  struct panel_info
			  *panel_data);  /*distingush which panel we use, (TRULY/OFLIM/BIEL/TPK)*/

	int (*reset)(void *chip_data);  /*Reset Touchpanel*/
	int (*reinit_device)(void *chip_data);
	fw_check_state(*fw_check)(void *chip_data,
				  struct resolution_info *resolution_info,
				  struct panel_info *panel_data); /*return < 0 :failed; 0 sucess*/

	/*return 0 normal; return -1:update failed;*/
	fw_update_state(*fw_update)(void *chip_data, const struct firmware *fw,
				    bool force);
	/*return 0:success;other:abnormal, need to jump out*/
	int (*power_control)(void *chip_data, bool enable);

	int (*reset_gpio_control)(void *chip_data, bool enable); /*used for reset gpio*/
	u32(*trigger_reason)(void *chip_data, int gesture_enable, int is_suspended);
	u8(*get_keycode)(void *chip_data); /*get touch-key code*/
	int (*esd_handle)(void *chip_data);
	int (*fw_handle)(void *chip_data);	/*return 0 normal; return -1:update failed;*/
	void (*exit_esd_mode)(void *chip_data);  /*add for s4322 exit esd mode*/
	void (*register_info_read)(void *chip_data, uint16_t register_addr,
				   uint8_t *result, uint8_t length);    /*add for read registers*/
	/*some ic need opearation if resume prepare*/
	int (*speed_up_resume_prepare) (void *chip_data);
	/*some ic need specific opearation in resuming*/
	int (*specific_resume_operate)(void *chip_data,
				       struct specific_resume_data *p_resume_data);
	/*some ic need opearation if resume timed out*/
	void (*resume_timedout_operate)(void *chip_data);

	int (*get_face_state)(void *chip_info);  /*get face detect state*/
	void (*health_report)(void *chip_data,
			      struct monitor_data *mon_data); /*data logger get*/
	void (*enable_fingerprint)(void *chip_data, uint32_t enable);
	void (*enable_gesture_mask)(void *chip_data, uint32_t enable);
	void (*set_touch_direction)(void *chip_data, uint8_t dir);
	uint8_t (*get_touch_direction)(void *chip_data);
	/*get gesture info of fingerprint underscreen when screen on*/
	void (*screenon_fingerprint_info)(void *chip_data,
					  struct fp_underscreen_info *fp_tpinfo);
	void (*fingerprint_health_info)(void *chip_data);

	void (*freq_hop_trigger)(void *chip_data); /*trigger frequency-hopping*/
	void (*force_water_mode)(void *chip_data, bool enable); /*force enter water mode*/
	void (*set_fp_error_report)(void *chip_data, bool enable); /*set fp error report*/
	void (*inject_wdt_reset)(void *chip_data, int value); /*inject watchdog reset*/
	void (*get_water_mode)(void *chip_data); /*force enter water mode*/
	void (*get_glove_mode)(void *chip_data, int *enable, int *count); /*force enter glove mode*/
	void (*set_noise_modetest)(void *chip_data, bool enable);
	uint8_t (*get_noise_modetest)(void *chip_data);
	/*If the tp ic need do something, use this!*/
	void (*tp_queue_work_prepare)(void *chip_data);

	void (*enable_kernel_grip)(void *chip_data,
				   struct kernel_grip_info *grip_info);          /*enable kernel grip in fw*/
	bool (*tp_irq_throw_away)(void *chip_data);
	void (*rate_white_list_ctrl)(void *chip_data, int value);
	void (*edge_limit_switch_write)(void *chip_data, int value);
	void (*report_rate)(void *chip_data, int value);
	int (*smooth_lv_set)(void *chip_data, int level);
	int (*sensitive_lv_set)(void *chip_data, int level);
	int (*pen_sensitive_lv_set)(void *chip_data, int level);
	int (*click_sensitive_lv_set)(void *chip_data, int level);
	int (*touch_leave_jitter_set)(void *chip_data, int level);
	int (*set_package_type)(void *chip_data, int level);
	int (*diaphragm_touch_lv_set)(void *chip_data, int level);
	int (*send_temperature)       (void *chip_data, int value, bool status);
	int (*tp_refresh_switch)(void *chip_data, int fps);
	void (*set_gesture_state)(void *chip_data, int state);
	int (*get_touch_points_auto)(void *chip_data,
				     struct point_info *points,
				     int max_num,
				     struct resolution_info *resolution_info); /*return point bit-map auto*/

	int (*get_gesture_info_auto)(void *chip_data,
				     struct gesture_info *gesture,
				     struct resolution_info *resolution_info); /*return 0:success;other:failed*/

	void (*screenon_fingerprint_info_auto)(void *chip_data,
					       struct fp_underscreen_info *fp_tpinfo,
					       struct resolution_info *resolution_info); /*get gesture info of fingerprint underscreen when screen on*/
	void  (*calibrate)    (struct seq_file *s, void *chip_data);
	bool  (*get_cal_status)  (struct seq_file *s, void *chip_data);
	int (*get_touch_points_help)(void *chip_data,
				     struct point_info *points,
				     int max_num,
				     struct resolution_info *resolution_info); /*return point bit-map auto*/
	int (*set_high_frame_rate)(void *chip_data, int value, int time);
	void (*notify_pencil_type)(void *chip_data, uint8_t value);
	int (*notify_keyboard_open)(void *chip_data);
	int (*fp_unlock_status_write)(void *chip_data, uint8_t value); /*fp unlock status*/
	void (*get_pen_points)       (void *chip_data, struct pen_info *pen_info);
	int (*pen_uplink_msg)(void *chip_data, u32 buf_len, u8 *buf, u32 *out_len);
	int (*pen_downlink_msg)(void *chip_data, u32 cmd, u32 buf_len, u8 *buf);
	int (*communicate_test)(void *chip_data);
	void (*aiunit_game_info)(void *chip_data);
	int (*set_idle_freq_mode)(bool enable);
};

struct aging_test_proc_operations {
	void (*start_aging_test)(void *chip_data);
	void (*finish_aging_test)(void *chip_data);
};

struct debug_info_proc_operations {
	void (*delta_read)(struct seq_file *s, void *chip_data);
	void (*pen_delta_read)(struct seq_file *s, void *chip_data);
	void (*key_trigger_delta_read)(void *chip_data);
	void (*self_delta_read)(struct seq_file *s, void *chip_data);
	void (*self_raw_read)(struct seq_file *s, void *chip_data);
	void (*baseline_read)(struct seq_file *s, void *chip_data);
	void (*pen_baseline_read)(struct seq_file *s, void *chip_data);
	void (*baseline_blackscreen_read)(struct seq_file *s, void *chip_data);
	void (*main_register_read)(struct seq_file *s, void *chip_data);
	void (*reserve_read)(struct seq_file *s, void *chip_data);
	void (*abs_doze_read)(struct seq_file *s, void *chip_data);
	void (*reserve1)(struct seq_file *s, void *chip_data);
	void (*reserve2)(struct seq_file *s, void *chip_data);
	void (*reserve3)(struct seq_file *s, void *chip_data);
	void (*reserve4)(struct seq_file *s, void *chip_data);
	void (*get_delta_data)(void *chip_data, int32_t *deltadata);
	void (*delta_snr_read)(struct seq_file *s, void *chip_data, uint32_t count);
	void (*tp_data_record_write)(void *chip_data, int32_t count);
};

/*********PART3:function or variables for other files**********************/
struct touchpanel_data *common_touch_data_alloc(void);
int tp_register_irq_func(struct touchpanel_data *ts);
int  common_touch_data_free(struct touchpanel_data *pdata);
int  register_common_touch_device(struct touchpanel_data *pdata);
void  unregister_common_touch_device(struct touchpanel_data *pdata);
void tp_shutdown(struct touchpanel_data *ts);
void tp_pm_suspend(struct touchpanel_data *ts);
void tp_pm_resume(struct touchpanel_data *ts);

int  tp_powercontrol_vddi(struct hw_resource *hw_res, bool on);
int  tp_powercontrol_avdd(struct hw_resource *hw_res, bool on);

void operate_mode_switch(struct touchpanel_data *ts);
void input_report_key_oplus(struct touchpanel_data *ts, unsigned int code,
			    int value);
void esd_handle_switch(struct esd_information *esd_info, bool on);
void clear_view_touchdown_flag(unsigned int tp_index);
void tp_touch_btnkey_release(unsigned int tp_index);
extern int tp_util_get_vendor(struct hw_resource *hw_res,
			      struct panel_info *panel_data);
extern bool tp_judge_ic_match(char *tp_ic_name);
extern int tp_judge_ic_match_commandline(struct panel_info *panel_data);

void touch_call_fp_grip(struct touchpanel_data *ts, int state);

extern int request_firmware_select(const struct firmware **firmware_p, const char *name, struct device *device);
bool is_oem_unlocked(void);
int  get_oem_verified_boot_state(void);

#ifndef CONFIG_TOUCHPANEL_NOTIFY
int opticalfp_irq_handler(struct fp_underscreen_info *fp_tpinfo);
#endif

#endif
