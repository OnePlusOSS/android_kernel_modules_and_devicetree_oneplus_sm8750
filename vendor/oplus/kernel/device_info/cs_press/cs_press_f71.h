/*
 *-----------------------------------------------------------------------------
 * The confidential and proprietary information contained in this file may
 * only be used by a person authorised under and to the extent permitted
 * by a subsisting licensing agreement from  CHIPSEA.
 *
 *            (C) COPYRIGHT 2020 SHENZHEN CHIPSEA TECHNOLOGIES CO.,LTD.
 *                ALL RIGHTS RESERVED
 *
 * This entire notice must be reproduced on all copies of this file
 * and copies of this file may only be made by a person if such person is
 * permitted to do so under the terms of a subsisting license agreement
 * from CHIPSEA.
 *
 *      Release Information : CSA37F71 chip forcetouch fw driver head file
 *      version : v0.2
 *-----------------------------------------------------------------------------
 */

#ifndef CS_PRESS_F71_H
#define CS_PRESS_F71_H

#define CS_I2C_ADDR        0x50 /* (0xA0>>1) */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/pm.h>
#include <linux/firmware.h>
#include <linux/netdevice.h>
#include <linux/mount.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/completion.h>
#include <linux/kthread.h>
#include <linux/unistd.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_wakeup.h>
#include <linux/version.h>

#include <linux/types.h>

#ifdef CONFIG_MTK_I2C_EXTENSION
#include <linux/dma-mapping.h>
#endif
#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE)
#include <soc/oplus/system/kernel_fb.h>
#endif

#if IS_ENABLED(CONFIG_FB)
#include <linux/fb.h>
#include <linux/notifier.h>
#endif
#if IS_ENABLED(CONFIG_DRM_OPLUS_PANEL_NOTIFY)
#include <linux/msm_drm_notify.h>
#elif IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
#include <linux/soc/qcom/panel_event_notifier.h>
#include <linux/msm_drm_notify.h>
#include <drm/drm_panel.h>
#elif IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)
#include <linux/msm_drm_notify.h>
#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY) || IS_ENABLED(CONFIG_OPLUS_DEVICE_INFO_MTK_PLATFORM)
#include <linux/mtk_panel_ext.h>
#include <linux/mtk_disp_notify.h>
#endif

#define CS_PRESS_FB_BUS_TRANS_TYPE      "10001"
#define CS_PRESS_FB_OFFSET_TYPE         "10002"
#define CS_PRESS_FB_NOISE_VAR_TYPE      "10003"
#define CS_PRESS_FB_FW_UPDATE_TYPE      "10004"

#define FW_FILE_NAME            "shouder/cs_press.bin"

/*
*@ LOG format
*/
#define LOG_ERR(fmt, args...)       pr_err("[cs_press][ERR] [%s: %d] "fmt,  __func__, __LINE__, ##args)
#define LOG_INFO(fmt, args...)      pr_err("[cs_press][INFO] [%s: %d] "fmt,  __func__, __LINE__, ##args)
#define LOG_DEBUG(fmt, args...)     pr_err("[cs_press][DEBUG] [%s: %d] "fmt,  __func__, __LINE__, ##args)

#define RSTPIN_RESET_ENABLE             1
#define CS_PRESS_CNT                    1

#define RETRY_NUM                       3
#define DEBUG_MODE_DELAY_TIME           20
#define CS_MANUFACTURER_ID_LENGTH       2
#define CS_MODULE_ID_LENGTH             2
#define CS_FW_VERSION_LENGTH            6
#define CS_CALI_PARA_LENGTH             8
#define RST_GPIO_HIGH                   1
#define RST_GPIO_LOW                    0
#define POWER_GPIO_HIGH                 1
#define POWER_GPIO_LOW                  0
#define IIC_KEY_EVENT                   0xD3
#define CS_FORCE_TRIG_LENGTH            6
#define CS_SCANT_PERIOD_LENGTH          4
#define CS_WORK_MODE_LENGTH             3
#define CS_SHELL_TEMP_LENGTH            4

#define STRENGTH_PER_LEVEL              5
#define TIME_MS_PER_LEVEL               5

#define CS_PRESS_DEV_ADDR_M             0xE4
#define CS_PRESS_DEV_ADDR_F             0xA0
#define CS_PRESS_DEV_ADDR_HID           0x2A

#define CH_NUM                         (4)

#define BOOT_FW_WRITE_CMD {0xaa,0x55,0xa5,0x5a}
#define BOOT_FW_JUMP_CMD  {0xA6,0x00,0x00,0x5A}
#define BOOT_FW_WFLAG_CMD {0x50,0x41,0x53,0x73}
/*reg addr*/
/*ndt reg*/
#define AP_RW_TEST_REG                  0x00 /*R/W read write test len = 10*/
#define AP_VERSION_REG                  0x10 /*R firmware infomation len = 6*/
#define AP_RD_CAIL_REG                  0x24 /*R len = 8*/
#define AP_RD_SONSOR_SATUS_REG          0x29 /*R len = 2*/
#define AP_RD_GPIO_STATUS_REG           0x2E /*R gpio status len = 6*/

#define AP_RD_TP_INFO_REG               0x52

#define AP_RD_APPLICATION_SATUS_REG     0x55 /*R application status len = 1*/
#define AP_RW_BUTTON_FORCE_RELEASE_THR_REG      0x5B /*R/W button force triggering/releasing threshold value len = 8*/
#define AP_RW_BUTTON_FORCE_THR_REG      0x5D /*R/W button force triggeringthreshold value len = 6*/
#define AP_RW_SCANT_PERIOD_REG          0x5E /*R/W scant period value len = 4*/
#define AP_RW_WORK_MODE_REG             0x5F /*R/W work mode len = 4*/
#define AP_RW_SHEll_TEMP_REG            0x61 /*R/W shell temp len = 4*/
#define AP_RW_SHEll_TYPE_REG            0x62 /*R/W shell type len = 3*/

#define AP_W_GPIO_TEST_REG              0x30 /*W io test len = 2*/

#define DEBUG_CLEAR_MODE                0x00
#define DEBUG_RAW_MODE                  0x10 /*rawdata*/
#define DEBUG_FORCE_DATA_MODE           0x20 /*forcedata*/
#define DEBUG_WRITE_COEF                0x30
#define DEBUG_READ_COEF                 0x31
#define DEBUG_WRITE_THRESHOLD           0x32
#define DEBUG_READ_THRESHOLD            0x33

#define DEBUG_MODE_REG                  0xF6
#define DEBUG_READY_REG                 0xF7
#define DEBUG_DATA_REG                  0xF8

#define DEBUG_MODE_V2_REG               0xFB
#define DEBUG_READY_V2_REG              0xFC
#define DEBUG_DATA_V2_REG               0xFD

#define DEBUG_RESET_SOURCE_REG          0xB6

#define AP_RESET_MCU_REG                0x01
#define AP_DEVICE_ID_REG                0x02
#define AP_MANUFACTURER_ID_REG          0x03
#define AP_MODULE_ID_REG                0x04

#define AP_WAKEUP_REG                   0x06
#define AP_SLEEP_REG                    0x07
#define AP_CALIBRATION_REG              0x1c
#define AP_WATCH_MODE_REG               0x1d

#define AP_FORCEDATA_REG                0x20
#define AP_FORCEDATA_LEN                68

#define AP_OFFSET_REG                   0x17
#define AP_DAC_UV_CONVER_REG            0x1c

#define BOOT_CMD_REG                    0x0000
#define BOOT_RESET_REG                  0xF17C
#define BOOT_RESET_REG_HID              0x2010

/*updata info*/
#define FW_ADDR_CODE_LENGTH             0x0e
#define FW_ADDR_VERSION                 0x0A
#define FW_ADDR_CODE_START              0x100
#define FW_ONE_BLOCK_LENGTH_W           128
#define FW_ONE_BLOCK_LENGTH_R           256
#define BOOT_CMD_LENGTH                 4
#define FW_UPDATA_MAX_LEN              (64*1024)

/*mode config*/
#define AP_W_CAL_FACTOR_DEBUG_MODE      0x30
#define AP_R_CAL_FACTOR_DEBUG_MODE      0x31

#define AP_W_PRESS_LEVEL_DEBUG_MODE     0x32
#define AP_R_PRESS_LEVEL_DEBUG_MODE     0x33

#define AP_CALIBRATION_DEBUG_MODE       0x34
#define AP_R_SENSOR_STATUS_DEBUG_MODE   0x36
#define AP_R_OFFSET_DEBUG_MODE          0x37

#define AP_R_RAWDATA_DEBUG_MODE         0x01

#define AP_R_NOISE_DEBUG_MODE           0x11
#define AP_R_PROCESSED_DEBUG_MODE       0x14
#define AP_R_NDT_DEBUG_MODE             0xF8

/*linux driver info*/
#define CS_PRESS_NAME                   "cs_press"
#define MISC_DEVICE_NAME                "ndt"
#define CS_SYSFS_NAME                   "cs_press"

#define SHELL_TEMP_DELAY_TIME           (2000)
#define CHECK_DELAY_TIME                (20000)
#define I2C_CHECK_SCHEDULE
#define INT_SET_EN
/*#define SIDE_KEY*/
#define CAMERA_KEY
#define CALIBRATION_SUCCESS_FALG        0xf0
#define CALIBRATION_FAIL_FALG           0xf2
#define CALIBRATION_OVERTIME_FALG       0xff

#define AFE_MAX_CH                      8
#define AFE_USE_CH                      1
#define KEY_NUM                         1

#define HIGH_VER_FILE_UPDATE            0
#define FORCE_FILE_UPDATE               1

#define PRESS_GEAR_LEVEL_LOW            1
#define PRESS_GEAR_LEVEL_MID            2
#define PRESS_GEAR_LEVEL_HIGH           3

#define  SCENE_NUM                      10
#define  PRESS_NUM                      10

#define CAMERA_KEY_CFG_MAX              5
#define CAMERA_KEY_CFG_NAME_LEN         32

#define CH_COUNT                        2
#define NOISE_TEST_COUNT                50
#define LAG_MIN_COUNT                   5

#define  NAME_MAX_LENS                  256

#ifndef MIN
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

#define DEFAULT_RUN_DELAY_TIME          50
/*99-1 = 98*/
#define SCHEDULE_CS_PRESS_PRIORITY      1

typedef struct{
    unsigned short manufacturer_id;
    unsigned short module_id;
    unsigned short fw_version;
}CS_FW_INFO_Def;

typedef struct{
    short rawdata[AFE_MAX_CH];
}CS_RAWDATA_Def;

typedef struct{
    short arith_rawdata[AFE_USE_CH];
    short baseline[AFE_USE_CH];
    short diffdata[AFE_USE_CH];
    short energy_data[AFE_USE_CH];
}CS_PROCESSED_DATA_Def;

typedef struct{
    short noise_peak[AFE_USE_CH];
    unsigned int noise_std[AFE_USE_CH];
}CS_NOISE_DATA_Def;

typedef struct{
    short offset[AFE_USE_CH];
}CS_OFFSET_DATA_Def;

typedef struct{
    short status[AFE_USE_CH];
}CS_SENSOR_STATUS_Def;

typedef struct{
    unsigned char calibration_channel;
    unsigned char calibration_progress;
    unsigned short calibration_factor;
    short press_adc_1st;
    short press_adc_2nd;
    short press_adc_3rd;
}CS_CALIBRATION_RESULT_Def;

#define STRENGTH_CFG_NUM                12
typedef struct{
    uint8_t light_tap_down_strength;
    uint8_t light_tap_up_strength;
    uint8_t heavy_tap_down_strength;
    uint8_t heavy_tap_up_strength;
    uint8_t light_swipe_down_strength;
    uint8_t light_swipe_up_strength;
    uint8_t heavy_swipe_down_strength;
    uint8_t heavy_swipe_up_strength;
    uint8_t area_1;
    uint8_t area_2;
    uint8_t long_tap_judge_time;
    uint8_t muti_tap_judge_time;
} trigger_strength_config;

#define BUS_ERROR_MSG_SIZE          128
#define BUS_ERROR_MSG_CNT           5
struct cs_press_t {
    int major;
    struct class *class;
    struct device *device_irq;
    struct i2c_client *client;
    struct kobject *kobj;

    /*GPIO pin*/
    int rst_gpio;
    int power_gpio;
    struct regulator *vdd_2v8;

    struct proc_dir_entry *p_proc_dir;

    unsigned short read_reg_len;
    unsigned char read_reg_data[64];

    unsigned char update_type;
    unsigned char updating_flag;
    unsigned char charge_flag;

    unsigned char game_status;

    int normal_left_geat_val;
    int normal_right_geat_val;

    struct thermal_zone_device *cs_thermal_zone_device;
    int cs_shell_themal_init_flag;
    int cs_shell_themal_enable;
    int update_done;

    int camera_key_mode;
    bool is_light_tap_down;
    bool is_heavy_tap_down;
    bool is_long_tap_down;
    bool is_physical_tap_down;
    bool mode_switch_waiting_up;
    trigger_strength_config strength_cfg;
    trigger_strength_config realtime_cfg[CAMERA_KEY_CFG_MAX];
    trigger_strength_config delay_cfg[CAMERA_KEY_CFG_MAX];
    unsigned int cfg_chosen;
    bool quick_on_closed;
    /* health monitor */
    int32_t dac_offset[CH_COUNT];
    int32_t dac_offset_diff[CH_COUNT];
    int32_t dac_noise_var[CH_COUNT];
    int16_t tap_force_min;
    int16_t tap_force_max;
    int16_t swipe_force_min;
    int16_t swipe_force_max;
    int16_t double_tap_cnt;
    int32_t heavy_tap_lag[CH_COUNT][LAG_MIN_COUNT];
    int16_t heavy_tap_force_max[CH_COUNT];
    int hard_reset_cnt;
    int soft_reset_cnt;
    int wdt_reset_cnt;
    int bus_error_cnt;
    char bus_error_msg[BUS_ERROR_MSG_CNT][BUS_ERROR_MSG_SIZE];
    //int32_t dac_offset_boot[CH_COUNT];
    //int32_t dac_noise_var_boot[CH_COUNT];
    char fw_update_error;

    /* framebuffer callbacks notifier */
    bool is_suspended;
#if IS_ENABLED(CONFIG_DRM_OPLUS_PANEL_NOTIFY)
    struct drm_panel *active_panel;
    struct notifier_block fb_notif; /*register to control suspend/resume*/
#elif IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
    struct drm_panel *active_panel;
    void *notifier_cookie;
#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY) || IS_ENABLED(CONFIG_OPLUS_DEVICE_INFO_MTK_PLATFORM)
    struct notifier_block disp_notifier;
#elif IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY) \
|| IS_ENABLED(CONFIG_FB)
    struct notifier_block fb_notif;    /*register to control suspend/resume*/
#endif

    struct wakeup_source *suspend_lock;

#ifdef I2C_CHECK_SCHEDULE
    struct delayed_work i2c_check_worker;
#endif
    struct delayed_work update_worker;
    struct delayed_work shell_temp_worker;
    struct delayed_work delay_run_worker;
    struct pinctrl *pinctrl;
    struct pinctrl_state *irq_pin_input;

    /**log**/
    int is_update_log;
};

enum PRESS_LEVEL
{
    LOW,
    MEDIUM,
    HIGH,
    PRESS_LEVEL_NUM
};

enum SCENE_ACTIVE_EM
{
    SCENE_DISABLE,
    SCENE_ENABLE,
};

enum SAMPLE_MODE
{
    FTM_MODE,   /*for test current*/
    SLEEP_MODE, /*1Hz*/
    NORMAL_MODE,/*100Hz*/
    GAME_MODE,  /*166Hz*/
    SAMPLE_MODE_NUM,
};

enum CAMERA_KEY_MODE
{
    CAMERA_KEY_DEFAULT_MODE = 0,
    CAMERA_KEY_CAMERA_MODE,
    CAMERA_KEY_POPUP_MODE,
    CAMERA_KEY_SLEEP_MODE,
};

enum CAMERA_KEY_IC_MODE
{
    CAMERA_KEY_IC_DEFAULT_EVENT_MODE = 0x00,
    CAMERA_KEY_IC_DELAY_EVENT_MODE = 0x01,
    CAMERA_KEY_IC_REALTIME_EVENT_MODE = 0x02,
    CAMERA_KEY_IC_SLEEP_EVENT_MODE = 0x04,
    CAMERA_KEY_IC_NONE_EVENT_MODE = 0xFF,
};

enum CAMERA_KEY
{
    KEY_LIGHT_TAP = 720,
    KEY_HEAVY_TAP,
    KEY_SHORT_TAP,
    KEY_LONG_TAP,
    KEY_DOUBLE_TAP,
    KEY_SWIPE_UP,
    KEY_SWIPE_DOWN,
    KEY_PHYSICAL_TAP,
};

enum EVENT_ACTION_TAP_BIT
{
    BIT_ACTION_TAP_DOWN = 0x01,
    BIT_ACTION_TAP_UP = 0x02,
    BIT_ACTION_SHORT_TAP = 0x04,
    BIT_ACTION_PHYSICAL_TOUCHING = 0x08,
    BIT_ACTION_LONGER_TAP = 0x10,
    BIT_ACTION_DOUBLE_TAP = 0x20,
    BIT_ACTION_TRIBLE_TAP = 0x40,
};

enum EVENT_ACTION_SWIPE_BIT
{
    BIT_ACTION_FOLLOW_SWIPE_DOWN = 0x01,
    BIT_ACTION_FOLLOW_SWIPE_UP = 0x02,
    BIT_ACTION_LIFT_SWIPE_DOWN = 0x04,
    BIT_ACTION_LIFT_SWIPE_UP = 0x08,
    BIT_ACTION_LIFT_DOUBLE_SWIPE_DOWN = 0x10,
    BIT_ACTION_LIFT_DOUBLE_SWIPE_UP = 0x20,
    BIT_ACTION_LIFT_PHYSICAL_DOWN = 0x80,
};

enum RST_SOURCE_BIT
{
    RSTS_POR = 0x0001,
    RSTS_PIN = 0x0002,
    RSTS_WDT = 0x0004,
    RSTS_MCU = 0x0100,
    RSTS_IWDG = 0x0200,
    RSTS_SYS = 0x0400,
};

enum CAMERA_KEY_ACTION
{
    ACTION_UNKNOWN = -1,
    ACTION_UP = 0,
    ACTION_DOWN = 1,
};

enum EVENT_AREA_BIT
{
    BIT_AREA_1 = 0x01,
    BIT_AREA_2 = 0x02,
    BIT_AREA_3 = 0x04,
};

enum EVENT_LEVEL_BIT
{
    BIT_LEVEL_UNKNOWN = -1,
    BIT_LEVEL_TOUCH = 0x01,
    BIT_LEVEL_LIGHT = 0x02,
    BIT_LEVEL_HEAVY = 0x04,
};

struct scene_para_t
{
    char scene_name[NAME_MAX_LENS];
    int left_geat[PRESS_LEVEL_NUM];
    int right_geat[PRESS_LEVEL_NUM];
    int charge_right_geat[PRESS_LEVEL_NUM];
    int sample_mode;
    int priority;
    int left_geat_num;
    int right_geat_num;
    struct list_head node;
};

struct scene_client_t
{
    char client_name[NAME_MAX_LENS];
    struct scene_para_t *pscene;
    int state;
    int left_geat_level;
    int right_geat_level;
    struct list_head node;
};

#endif
