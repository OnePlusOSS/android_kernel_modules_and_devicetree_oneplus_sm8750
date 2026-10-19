/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#ifndef __FT3683G_CORE_H__
#define __FT3683G_CORE_H__

/*********PART1:Head files**********************/
#include <linux/i2c.h>
#include <linux/vmalloc.h>
#include "../focal_common.h"

/*********PART2:Define Area**********************/

#define RESET_TO_NORMAL_TIME                    200        /*Sleep time after reset*/
#define RESET_TO_FACTORY_TIME                   350        /*Sleep time after reset*/
#define POWEWRUP_TO_RESET_TIME                  10

#define INTERVAL_READ_REG                       200  /* unit:ms */
#define TIMEOUT_READ_REG                        1000 /* unit:ms */

#define FTS_VAL_CHIP_ID                         0x56
#define FTS_VAL_CHIP_ID2                        0x72
#define FTS_VAL_BT_ID                           0x36
#define FTS_VAL_BT_ID2                          0xB3

#define FTS_120HZ_REPORT_RATE                   0x0C
#define FTS_180HZ_REPORT_RATE                   0x12
#define FTS_REG_SMOOTH_LEVEL                    0x85
#define FTS_REG_GAME_MODE_EN                    0xC3
#define FTS_REG_CLICK_SENSITIVE                 0x80
#define FTS_REG_REPORT_RATE                     0x88/*0x12:180hz, 0x0C:120hz*/
#define FTS_IDLE_FREQ_240                       0x89
#define FTS_REG_HIGH_FRAME_TIME                 0x8A
#define FTS_REG_CHARGER_MODE_EN                 0x8B
#define FTS_REG_EDGE_LIMIT                      0x8C
#define FTS_REG_EDGE_LIMIT_BIT                  0x04
#define FTS_REG_CTRL                            0x8B
#define FTS_REG_STABLE_DISTANCE_AFTER_N         0xB9
#define FTS_REG_STABLE_DISTANCE                 0xBA
#define FTS_REG_HEADSET_MODE_EN                 0xC4
#define FTS_REG_SMART_TOUCH_MODE_EN             0xC5
#define FTS_REG_FOD_EN                          0xCF
#define FTS_REG_FOD_INFO                        0xE1
#define FTS_REG_FOD_INFO_LEN                    9
#define FTS_REG_FOD_ERROR_INFO                  0xE0
#define FTS_REG_FOD_ERROR_INFO_LEN              14
#define FTS_REG_AOD_INFO                        0xD3
#define FTS_REG_AOD_INFO_LEN                    6
#define FTS_REG_DIFFER_VERSION                	0xCD
#define FTS_DIFFER_VERSION_V1                	0
#define FTS_DIFFER_VERSION_V2                	1

#define FTS_REG_INT_CNT                         0x8F
#define FTS_REG_FLOW_WORK_CNT                   0x91
#define FTS_REG_CHIP_ID                         0xA3
#define FTS_REG_CHIP_ID                         0xA3
#define FTS_REG_CHIP_ID2                        0x9F
#define FTS_REG_POWER_MODE                      0xA5
#define FTS_REG_FW_VER                          0xA6
#define FTS_REG_VENDOR_ID                       0xA8
#define FTS_REG_GESTURE_EN                      0xD0
#define FTS_REG_GESTURE_CONFIG1                 0xD1
#define FTS_REG_GESTURE_CONFIG2                 0xD2
#define FTS_REG_GESTURE_CONFIG3                 0xD5
#define FTS_REG_GESTURE_CONFIG4                 0xD6
#define FTS_REG_GESTURE_CONFIG5                 0xD7
#define FTS_REG_GESTURE_CONFIG6                 0xD8
#define FTS_REG_PULSE_CONTROL                   0xB6
#define FTS_REG_WORK_MODE                       0x9E
#define FTS_REG_WORK_MODE_SNR_MODE              0x81
#define FTS_REG_WORK_MODE_FINAL_DIFF_MODE       0x01
#define FTS_REG_WORK_MODE_NORMAL_MODE           0x00
#define FTS_FW_INFO                             0x96
#define FTS_REG_TEMPERATURE                     0x97
#define FTS_REG_PALM_TO_SLEEP_STATUS            0x9B
#define FTS_REG_INJECT_WDT_RESET                0xB6
#define FTS_REG_FREQUENCE_WATER_MODE			0xBF
#define FTS_REG_SET_FP_ERROR_REPORT             0xBF /* bit7 */
#define FTS_REG_GESTURE_OUTPUT_ADDRESS          0xD3
#define FTS_REG_MODULE_ID                       0xE3
#define FTS_REG_LIC_VER                         0xE4
#define FTS_REG_AUTOCLB_ADDR                    0xEE
#define FTS_REG_SAMSUNG_SPECIFAL                0xFA
#define FTS_REG_HEALTH_1                        0xFD
#define FTS_REG_HEALTH_2                        0xFE
#define FTS_REG_HEALTH_BASELINE                 0x03
#define FTS_REG_GLOVE_MODE_SWITCH               0xC0
#define FTS_REG_GLOVE_MODE_STATE                0x01
#define FTS_REG_EDGE_LIMIT_SWITCH               0xCE

#define FTS_90HZ_REPORT_RATE                    0x09
#define FTS_120HZ_REPORT_RATE                   0x0C
#define FTS_180HZ_REPORT_RATE                   0x12
#define FTS_240HZ_REPORT_RATE                   0x18
#define FTS_360HZ_REPORT_RATE                   0x24
#define FTS_720HZ_REPORT_RATE                   0x24            /*not support*/

#define FTS_GET_RATE_120                        120
#define FTS_GET_RATE_180                        180
#define FTS_GET_RATE_240                        10
#define FTS_GET_RATE_300                        300
#define FTS_GET_RATE_600                        600

#define FTS_REG_CHARGER_MODE_EN_BIT             0x00
#define FTS_REG_GAME_MODE_EN_BIT                0x02
#define FTS_REG_HEADSET_MODE_EN_BIT             0x06

#define FTS_MAX_POINTS_SUPPORT                  10
#define FTS_MAX_ID                              0x0A
#define FTS_POINTS_ONE                          21  /*2 + 6*3 + 1*/
#define FTS_POINTS_TWO                          41  /*8*10 - 1*/
#define FTS_MAX_POINTS_LENGTH                   134  /* ((FTS_POINTS_ONE) + (FTS_POINTS_TWO))  */
#define FTS_MAX_POINTS_SNR_LENGTH               1824 /* FTS_MAX_POINTS_LENGTH + 2 + 2*tx*rx + (tx+rx+4)*2*2 */
#define FTS_MAX_POINTS_SNR_LENGTH_V2            2220 /* point_buffer:84 + status:10 + edge:40 + aux:18 + differ:tx*rx*2+(tx+rx)*2*2+2 + 8*rx+2 */
#define FTS_REG_POINTS                          0x01
#define FTS_REG_POINTS_N                        (FTS_POINTS_ONE + 1)
#define FTS_REG_POINTS_LB                       0x3E
#define FTS_MAX_TX_NUM                          20
#define FTS_MAX_RX_NUM                          41

#define FTS_MAX_TOUCH_BUF                       4096

#define FTS_DIFF_BUF_LENGTH                     720 /* tx*rx */
#define FTS_SC_BUF_LENGTH                       58 /* tx+rx */

#define FTS_GESTURE_DATA_LEN                    28


#define BYTES_PER_TIME                          (128)  /* max:128 */

/*
 * factory test registers
 */
#define ENTER_WORK_FACTORY_RETRIES              5
#define DEVIDE_MODE_ADDR                        0x00
#define FTS_FACTORY_MODE_VALUE                  0x40
#define FTS_WORK_MODE_VALUE                     0x00
#define FACTORY_TEST_RETRY                      50
#define FACTORY_TEST_DELAY                      18
#define FACTORY_TEST_RETRY_DELAY                100
#define FACTORY_FTS_RESET_TEST                  0xAD

/* mc_sc */
#define FACTORY_REG_LINE_ADDR                   0x01
#define FACTORY_REG_CHX_NUM                     0x02
#define FACTORY_REG_CHY_NUM                     0x03
#define FACTORY_REG_CLB                         0x04
#define FACTORY_REG_DATA_SELECT                 0x06
#define FACTORY_REG_FRE_LIST                    0x0A
#define FACTORY_REG_DATA_TYPE                   0x5B
#define FACTORY_REG_TOUCH_THR                   0x0D
#define FACTORY_REG_NORMALIZE                   0x16
#define FACTORY_REG_MAX_DIFF                    0x1B
#define FACTORY_REG_FRAME_NUM                   0x1C
#define FACTORY_REG_GCB                         0xBD

#define FACTORY_REG_RAWDATA_ADDR_MC_SC          0x36
#define FACTORY_REG_FIR                         0xFB
#define FACTORY_REG_WC_SEL                      0x09
#define FACTORY_REG_MC_SC_MODE                  0x44
#define FACTORY_REG_HC_SEL                      0x0F
#define FACTORY_REG_MC_SC_CB_H_ADDR_OFF         0x49
#define FACTORY_REG_MC_SC_CB_ADDR_OFF           0x45
#define FACTORY_REG_MC_SC_CB_ADDR               0x4E
#define FACTROY_REG_SHORT_TEST_EN               0x07
#define FACTROY_REG_SHORT_CA                    0x01
#define FACTROY_REG_SHORT_CC                    0x02
#define FACTROY_REG_SHORT_CG                    0x03
#define FACTROY_REG_SHORT_OFFSET                0x04
#define FACTROY_REG_SHORT_AB_CH                 0x58
#define FACTROY_REG_SHORT_DELAY                 0x5A
#define FACTORY_REG_SHORT_ADDR_MC               0xF4

#define FACTROY_REG_SCAP_CFG                    0x58
#define FACTROY_REG_SCAP_GCB_TX                 0xBC
#define FACTROY_REG_SCAP_GCB_RX                 0xBE
#define FACTROY_REG_CB_BUF_SEL                  0xBF

#define FACTROY_REG_SHORT2_TEST_EN              0xC0
#define FACTROY_REG_SHORT2_CA                   0x01
#define FACTROY_REG_SHORT2_CC                   0x02
#define FACTROY_REG_SHORT2_CG                   0x03
#define FACTROY_REG_SHORT2_OFFSET               0x04
#define FACTROY_REG_SHORT2_RES_LEVEL            0xC1
#define FACTROY_REG_SHORT2_DEALY                0xC2
#define FACTROY_REG_SHORT2_TEST_STATE           0xC3
#define FACTORY_REG_SHORT2_ADDR_MC              0xC4
#define FACTROY_REG_SHORT2_AB_CH                0xC6

#define SC_NUM_MAX                              256

#define GET_LEN_BY_WIDTH_MAJOR(width_major, len)\
({\
	if (width_major > 10 && width_major < 14)\
		*len = 5;\
	if (width_major > 12 && width_major < 16)\
		*len = 5;\
	if (width_major > 16 && width_major < 20)\
		*len = 7;\
	if (width_major > 18 && width_major < 22)\
		*len = 7;\
})

#define FACTORY_REG_PARAM_UPDATE_STATE_TOUCH    0xB5

#define FTS_MAX_COMMMAND_LENGTH                 16

#define TEST_RETVAL_00                          0x00
#define TEST_RETVAL_AA                          0xAA

#define FTS_EVENT_FOD                           0x26

#define MAX_PACKET_SIZE                         128

#define FTS_TOUCH_E_NUM                         1
#define FTS_ONE_TCH_LEN_V2                      8
#define FTS_MAX_ID                          0x0A
#define FTS_TOUCH_OFF_E_XH                  0
#define FTS_TOUCH_OFF_XL                    1
#define FTS_TOUCH_OFF_ID_YH                 2
#define FTS_TOUCH_OFF_YL                    3
#define FTS_TOUCH_OFF_PRE                   4
#define FTS_TOUCH_OFF_AREA                  5
#define FTS_TOUCH_OFF_MINOR                 6
#define FTS_HI_RES_X_MAX                    16

#define FTS_NOT_GAME_MODE                       0x00

#define FTS_NOT_GAME_MODE                       0x00
#define FTS_240HZ_GAME_MODE                     0x01
#define FTS_360HZ_GAME_MODE                     0x02
#define FTS_720HZ_GAME_MODE                     0x03
#define INTELLIGENT_GAME_MODE                   11
#define EXTREME_GAME_MODE                       12

#define FTS_POINTER_BUFFER_LEN                  150
#define FTS_EDG_BUFFER_LEN                      200
enum _FTS_RST_REASON {
	FTS_RST_REASON_UNKNOWN  = 0,
	FTS_RST_REASON_FWUPDATE = 0x01,
	FTS_RST_REASON_WDT      = 0x02,
	FTS_RST_REASON_EXTERNAL = 0x04,
	FTS_RST_REASON_PWR      = 0x08,
};

enum _FTS_FP_ERROR_TYPE {
	FTS_FINGERPRINT_DOWN_BEFORE_FP_ENABLE = 0x01,
	FTS_FINGERPRINT_X_Y_NOT_MATCH = 0x02,
	FTS_ANOTHER_FINGER_ON_NON_FP_ZONE = 0x04,
	FTS_FINGERPRINT_AREA_NOT_MATCH = 0x10,
	FTS_FINGERPRINT_OUT_MOVE_IN = 0x40,
};

enum _FTS_TOUCH_ETYPE {
	TOUCH_DEFAULT = 0x00,
	TOUCH_PROTOCOL_v2 = 0x02,
	TOUCH_EXTRA_MSG = 0x08,
	TOUCH_PEN = 0x0B,
	TOUCH_GESTURE = 0x80,
	TOUCH_FW_INIT = 0x81,
	TOUCH_DEFAULT_HI_RES = 0x82,
	TOUCH_IGNORE = 0xFE,
	TOUCH_ERROR = 0xFF,
};

enum SMART_MODE{
	SMART_DEFAULT_MODE = 0,
	SMART_FILM_MODE = 1,
	SMART_WATERPROOF_MODE = 2,
	SMART_FILM_WATERPROOF_MODE = 3,
};

struct fts_autotest_offset {
	int32_t *fts_raw_data_P;
	int32_t *fts_raw_data_N;
	int32_t *fts_panel_differ_data_P;
	int32_t *fts_panel_differ_data_N;
	int32_t *fts_noise_data_P;
	int32_t *fts_noise_data_N;
	int32_t *fts_uniformity_data_P;
	int32_t *fts_uniformity_data_N;
	int32_t *fts_scap_cb_data_P;
	int32_t *fts_scap_cb_data_N;
	int32_t *fts_scap_cb_data_waterproof_P;
	int32_t *fts_scap_cb_data_waterproof_N;
	int32_t *fts_scap_raw_data_P;
	int32_t *fts_scap_raw_data_N;
	int32_t *fts_scap_raw_waterproof_data_P;
	int32_t *fts_scap_raw_waterproof_data_N;
};

enum FW_STATUS {
	FTS_RUN_IN_ERROR,
	FTS_RUN_IN_APP,
	FTS_RUN_IN_ROM,
	FTS_RUN_IN_PRAM,
	FTS_RUN_IN_BOOTLOADER,
};

struct fts_fod_info {
	u8 fp_id;
	u8 event_type;
	u8 fp_area_rate;
	u8 tp_area;
	u16 fp_x;
	u16 fp_y;
	u8 fp_down;
	u8 fp_down_report;
};

struct ftxxxx_proc {
	struct proc_dir_entry *proc_entry;
	u8 opmode;
	u8 cmd_len;
	u8 cmd[FTS_MAX_COMMMAND_LENGTH];
};

enum FOD_HEALTH_INFO {
	FOD_ENABLE      	= 0x01,
	FOD_FARAWAY     	= 0x03,
	FOD_NOT_IN_AREA     = 0x04,
	FOD_IN_AREA    	 	= 0x10,
	FOD_IN_AREA_V2   	= 0x11,
	FOD_FIRST_EFFETIVE_PRESS   	= 0x20,
	FOD_SMALL_TOUCH   			= 0x21,
	FOD_DETECT_EFFETIVE_AREA 	= 0x22,
	FOD_DETECT_ID_REPORRE    	= 0x30,
};

enum DEBUG_INFO {
	RESET_TYPE		= 84,
	DOWN_THD		= 85,
	UP_THD			= 86,
	IDLE_THD		= 87,
	MAX_DIFF_H8		= 88,
	MAX_DIFF_L8		= 89,
};
struct fts_aod_info {
	u8 gesture_id;
	u8 point_num;
	u16 aod_x;
	u16 aod_y;
};

typedef enum {
	TYPE_NO_FOD_TRIGGER = 0,
	TYPE_SMALL_FOD_TRIGGER,
	TYPE_FOD_TRIGGER,
} fod_trigger_type;

struct chip_data_ft3683g {
	bool esd_check_need_stop;   /*true:esd check do nothing*/
	bool esd_check_enabled;
	bool use_panelfactory_limit;
	bool prc_support;
	bool prc_mode;
	bool touch_analysis_support;
	bool ft3683_grip_v2_support;
	bool is_ic_sleep;    /*ic sleep status*/
	u16 touch_size;
	u8 *touch_buf;
	int ta_flag;
	u32 ta_size;
	u8 *ta_buf;
	u8 irq_type;
	u8 fwver;
	u8 touch_direction;
	u8 fp_en;
	u8 fp_down;
	u8 ctrl_reg_state;

	u8 snr_buf[FTS_MAX_POINTS_SNR_LENGTH_V2];
	int diff_buf[FTS_DIFF_BUF_LENGTH];
	int sc_water[FTS_SC_BUF_LENGTH];
	int sc_nomal[FTS_SC_BUF_LENGTH];
	u16 cur_noise;
	u16 buffer_len;
	u32 time_since_last_frame;
	u32 frame_cnt;

	int rl_cnt;
	int scb_cnt;
	int srawdata_cnt;
	int last_mode;
	int csv_fd;
	int probe_done;
	int *noise_rawdata;
	int *rawdata;
	int *panel_differ;
	int *scap_cb;
	int *scap_rawdata;
	int *rawdata_linearity;
	int tp_index;
	int print_count;
	int print_count_glove;
	int *node_valid;
	int *node_valid_sc;
	int gesture_state;
	int tp_temperature;
	int freq_point;
	int glove_mode_flag;
	int glove_mode_enable_count;
	bool black_gesture_indep;
	u8 fre_num;
	u8 snr_count;
	u8 differ_mode;
	u8 tp_differ_version;

	u8 gesture_flag;

	char *test_limit_name;
	char *fw_name;
	tp_dev tp_type;             /*tp_devices.h*/

	u8 *bus_tx_buf;
	u8 *bus_rx_buf;
	struct mutex bus_lock;

	struct spi_device *ft_spi;
	struct hw_resource *hw_res;
	struct ftxxxx_proc proc;
	struct ftxxxx_proc proc_ta;
	struct fts_fod_info fod_info;
	struct fts_aod_info aod_info;
	struct seq_file *s;
	struct fts_autotest_offset *fts_autotest_offset;
	struct touchpanel_data *ts;
	struct monitor_data *monitor_data;
	struct delayed_work prc_work;
	struct workqueue_struct *ts_workqueue;
	struct resolution_info *resolution_info;
	wait_queue_head_t ts_waitqueue;
	unsigned long intr_jiffies;
	bool high_resolution_support;
	bool high_resolution_support_x8;
	bool snr_is_reading;
	bool snr_read_support;
	bool snr_data_is_ready;
	bool differ_read_every_frame;
	bool tp_data_record_support;
	unsigned int spi_speed;
	bool switch_game_rate_support;
	bool is_in_water;
	bool charger_connected;
	fod_trigger_type fod_trigger;
	bool water_mode;
	u8 last_waterlevel; /* last rain level for health report; init 0xFF */
	int extreme_game_report_rate;
	bool extreme_game_flag;
	bool fingerprint_error_report_support;              /*fingerprint error report support*/
};


extern struct chip_data_ft3683g *g_fts_data;

int fts_test_entry(struct chip_data_ft3683g *ts_data,
                   struct auto_testdata *focal_testdata);
int ft3683g_auto_preoperation(struct seq_file *s, void *chip_data,
                             struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info);
int ft3683g_noise_autotest(struct seq_file *s, void *chip_data,
                          struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info);
int ft3683g_rawdata_autotest(struct seq_file *s, void *chip_data,
                            struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info);
int ft3683g_uniformity_autotest(struct seq_file *s, void *chip_data,
                               struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info);
int ft3683g_scap_cb_autotest(struct seq_file *s, void *chip_data,
                            struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info);
int ft3683g_scap_rawdata_autotest(struct seq_file *s, void *chip_data,
                                 struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info);
int ft3683g_short_test(struct seq_file *s, void *chip_data,
                      struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info);
int ft3683g_panel_differ_test(struct seq_file *s, void *chip_data,
                             struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info);
int ft3683g_auto_endoperation(struct seq_file *s, void *chip_data,
                             struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info);
int ft3683g_membist_test(struct seq_file *s, void *chip_data,
                             struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info);
int ft3683g_cal_test(struct seq_file *s, void *chip_data,
                             struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info);
int ft3683g_rst_autotest(struct seq_file *s, void *chip_data,
			     struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info);

int fts_write(u8 *writebuf, u32 writelen);
int fts_write_reg(u8 addr, u8 value);
int fts_read(u8 *cmd, u32 cmdlen, u8 *data, u32 datalen);
int fts_read_reg(u8 addr, u8 *value);

int fts_spi_write_direct(u8 *writebuf, u32 writelen);
int fts_spi_read_direct(u8 *writebuf, u32 writelen, u8 *readbuf, u32 readlen);
int fts_set_spi_max_speed(unsigned int speed, char mode);
int fts_reset_proc(int hdelayms);
int ft3683g_rstpin_reset(void *chip_data);

#endif /*__FT3683G_CORE_H__*/
