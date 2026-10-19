// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */


#define pr_fmt(fmt) "<sensor_feedback>" fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/atomic.h>
#include <linux/kthread.h>
#include <linux/soc/qcom/smem.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/time64.h>
#include <linux/kdev_t.h>
#include <linux/vmalloc.h>
#include <linux/version.h>

#include "sensor_feedback.h"
#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE)
#include <soc/oplus/system/kernel_fb.h>
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_TRACE_SENSOR)
#include <soc/oplus/system/oplus_trace_sensor.h>
static bool enable_report = false;
#endif

#define ALIGN4(s) ((sizeof(s) + 3)&(~0x3))

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
#define PDE_DATA(inode) pde_data(inode)
#endif /* KERNEL_VERSION(6, 1, 0) */

struct msm_rpmh_master_stats {
	uint32_t version_id;
	uint32_t counts;
	uint64_t last_entered;
	uint64_t last_exited;
	uint64_t accumulated_duration;
};

enum master_smem_id {
	MPSS_ID = 605,
	ADSP_ID,
	CDSP_ID,
	SLPI_ID,
	GPU_ID,
	DISPLAY_ID,
	SLPI_ISLAND_ID = 613,
};

enum master_pid {
	PID_APSS = 0,
	PID_MPSS = 1,
	PID_ADSP = 2,
	PID_SLPI = 3,
	PID_CDSP = 5,
	PID_GPU = PID_APSS,
	PID_DISPLAY = PID_APSS,
};

struct msm_rpmh_master_data {
	char *master_name;
	enum master_smem_id smem_id;
	enum master_pid pid;
};

#ifndef SENSOR_DEVICE_TYPE
#define SENSOR_DEVICE_TYPE	  "10002"
#endif
#ifndef SENSOR_POWER_TYPE
#define SENSOR_POWER_TYPE	   "10003"
#endif
#ifndef SENSOR_STABILITY_TYPE
#define SENSOR_STABILITY_TYPE   "10004"
#endif
#ifndef SENSOR_PFMC_TYPE
#define SENSOR_PFMC_TYPE		"10005"
#endif
#ifndef SENSOR_MEMORY_TYPE
#define SENSOR_MEMORY_TYPE	  "10006"
#endif

#define SENSOR_DEBUG_DEVICE_TYPE      "20002"
#define SENSOR_DEBUG_POWER_TYPE       "20003"
#define SENSOR_DEBUG_STABILITY_TYPE   "20004"
#define SENSOR_DEBUG_PFMC_TYPE        "20005"
#define SENSOR_DEBUG_MEMORY_TYPE      "20006"

#define PHY_SENSOR_NUM          18
#define VIR_SENSOR_NUM          31

//extern int oplus_subsystem_sleeptime(char *name, u64 *sleeptime);
static struct sensor_fb_cxt *g_sensor_fb_cxt = NULL;
#define MSM_ARCH_TIMER_FREQ 19200000
static char *subsys_names[SUBSYS_COUNTS] = {"ADSP", "CDSP", "SLPI"};
static struct subsys_sleep_stats g_subsys_sleep_stats;
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#endif


static const struct msm_rpmh_master_data rpmh_masters[] = {
	{"MPSS", MPSS_ID, PID_MPSS},
	{"ADSP", ADSP_ID, PID_ADSP},
	{"ADSP_ISLAND", SLPI_ISLAND_ID, PID_ADSP},
	{"CDSP", CDSP_ID, PID_CDSP},
	{"SLPI", SLPI_ID, PID_SLPI},
	{"SLPI_ISLAND", SLPI_ISLAND_ID, PID_SLPI},
	{"GPU", GPU_ID, PID_GPU},
	{"DISPLAY", DISPLAY_ID, PID_DISPLAY},
};


/*fb_field :maxlen 19*/
struct sensor_fb_conf g_fb_conf[] = {
	{PS_INIT_FAIL_ID, "device_ps_init_fail", SENSOR_DEVICE_TYPE},
	{PS_I2C_ERR_ID, "device_ps_i2c_err", SENSOR_DEVICE_TYPE},
	{PS_ALLOC_FAIL_ID, "device_ps_alloc_fail", SENSOR_DEVICE_TYPE},
	{PS_ESD_REST_ID, "device_ps_esd_reset", SENSOR_DEVICE_TYPE},
	{PS_NO_INTERRUPT_ID, "device_ps_no_irq", SENSOR_DEVICE_TYPE},
	{PS_FIRST_REPORT_DELAY_COUNT_ID, "device_ps_rpt_delay", SENSOR_DEBUG_DEVICE_TYPE},
	{PS_ORIGIN_DATA_TO_ZERO_ID, "device_ps_to_zero", SENSOR_DEBUG_DEVICE_TYPE},
	{PS_CALI_DATA_ID, "device_ps_cali_data", SENSOR_DEBUG_DEVICE_TYPE},
	{PS_DYNAMIC_CALI_ID, "device_ps_dynamic_cali", SENSOR_DEBUG_DEVICE_TYPE},
	{PS_ZERO_CALI_ID, "device_ps_zero_cali", SENSOR_DEBUG_DEVICE_TYPE},
	{PS_ENABLE_FAIL_ID, "device_ps_enable_fail", SENSOR_DEBUG_DEVICE_TYPE},
	{PS_ABNORMAL_OFFSET_CALI_ID, "device_ps_abnormal_offset_cali", SENSOR_DEBUG_DEVICE_TYPE},
	{PS_SET_OFFSET_RETRY_ID, "device_ps_set_offset_retry", SENSOR_DEBUG_DEVICE_TYPE},
	{PS_RECOVER_DEVICE_ID, "device_ps_recover_device", SENSOR_DEBUG_DEVICE_TYPE},


	{ALS_INIT_FAIL_ID, "device_als_init_fail", SENSOR_DEVICE_TYPE},
	{ALS_I2C_ERR_ID, "device_als_i2c_err", SENSOR_DEVICE_TYPE},
	{ALS_ALLOC_FAIL_ID, "device_als_alloc_fail", SENSOR_DEVICE_TYPE},
	{ALS_ESD_REST_ID, "device_als_esd_reset", SENSOR_DEVICE_TYPE},
	{ALS_NO_INTERRUPT_ID, "device_als_no_irq", SENSOR_DEVICE_TYPE},
	{ALS_FIRST_REPORT_DELAY_COUNT_ID, "device_als_rpt_delay", SENSOR_DEBUG_DEVICE_TYPE},
	{ALS_ORIGIN_DATA_TO_ZERO_ID, "device_als_to_zero", SENSOR_DEBUG_DEVICE_TYPE},
	{ALS_CALI_DATA_ID, "device_als_cali_data", SENSOR_DEBUG_DEVICE_TYPE},
	{ALS_CG_RPT_INFO_ID, "device_als_cg_rpt_info", SENSOR_DEBUG_DEVICE_TYPE},
	/*Add for sip3625*/
	{ALS_RECOVER_DEVICE_ID, "device_als_recover_device", SENSOR_DEBUG_DEVICE_TYPE},
	{ALS_FIFO_RECOVER_DEVICE_ID, "device_als_fifo_recover_device", SENSOR_DEBUG_DEVICE_TYPE},
	/*Add for sip3625*/


	{ACCEL_INIT_FAIL_ID, "device_acc_init_fail", SENSOR_DEVICE_TYPE},
	{ACCEL_I2C_ERR_ID, "device_acc_i2c_err", SENSOR_DEVICE_TYPE},
	{ACCEL_ALLOC_FAIL_ID, "device_acc_alloc_fail", SENSOR_DEVICE_TYPE},
	{ACCEL_ESD_REST_ID, "device_acc_esd_reset", SENSOR_DEVICE_TYPE},
	{ACCEL_NO_INTERRUPT_ID, "device_acc_no_irq", SENSOR_DEVICE_TYPE},
	{ACCEL_FIRST_REPORT_DELAY_COUNT_ID, "device_acc_rpt_delay", SENSOR_DEBUG_DEVICE_TYPE},
	{ACCEL_ORIGIN_DATA_TO_ZERO_ID, "device_acc_to_zero", SENSOR_DEBUG_DEVICE_TYPE},
	{ACCEL_CALI_DATA_ID, "device_acc_cali_data", SENSOR_DEBUG_DEVICE_TYPE},
	{ACCEL_DATA_BLOCK_ID, "device_acc_data_block", SENSOR_DEVICE_TYPE},
	{ACCEL_SUB_DATA_BLOCK_ID, "device_sub_acc_data_block", SENSOR_DEVICE_TYPE},
	{ACCEL_DATA_FULL_RANGE_ID, "device_acc_data_full_range", SENSOR_DEVICE_TYPE},
	{ACCEL_SW_RESET_ID, "device_acc_sw_reset", SENSOR_DEVICE_TYPE},
	{ACCEL_NO_DATA_ID, "device_acc_no_data", SENSOR_DEVICE_TYPE},
	{ANY_MOTION_ENABLE_FAIL_ID, "device_any_motion_enable_fail", SENSOR_DEVICE_TYPE},
	{ACCEL_CHANGE_FSR_FAIL_ID, "device_acc_change_fsr_fail", SENSOR_DEVICE_TYPE},

	{ACCEL_SUB_I2C_ERR_ID, "device_acc_sub_i2c_err", SENSOR_DEVICE_TYPE},
	{ACCEL_SUB_NO_DATA_ID, "device_acc_sub_no_data", SENSOR_DEVICE_TYPE},
	{ACCEL_SUB_ESD_REST_ID, "device_acc_sub_esd_reset", SENSOR_DEVICE_TYPE},

	{GYRO_INIT_FAIL_ID, "device_gyro_init_fail", SENSOR_DEVICE_TYPE},
	{GYRO_I2C_ERR_ID, "device_gyro_i2c_err", SENSOR_DEVICE_TYPE},
	{GYRO_ALLOC_FAIL_ID, "device_gyro_alloc_fail", SENSOR_DEVICE_TYPE},
	{GYRO_ESD_REST_ID, "device_gyro_esd_reset", SENSOR_DEVICE_TYPE},
	{GYRO_NO_INTERRUPT_ID, "device_gyro_no_irq", SENSOR_DEVICE_TYPE},
	{GYRO_FIRST_REPORT_DELAY_COUNT_ID, "device_gyro_rpt_delay", SENSOR_DEBUG_DEVICE_TYPE},
	{GYRO_ORIGIN_DATA_TO_ZERO_ID, "device_gyro_to_zero", SENSOR_DEBUG_DEVICE_TYPE},
	{GYRO_CALI_DATA_ID, "device_gyro_cali_data", SENSOR_DEBUG_DEVICE_TYPE},
	{GYRO_DATA_BLOCK_ID, "device_gyro_data_block", SENSOR_DEVICE_TYPE},
	{GYRO_OTC_K_ID, "device_gyro_otc_k", SENSOR_DEVICE_TYPE},
	{GYRO_OTC_B_ID, "device_gyro_otc_b", SENSOR_DEVICE_TYPE},
	{GYRO_OTC_BIAS_ID, "device_gyro_otc_bias", SENSOR_DEVICE_TYPE},
	{GYRO_OTC_TEMP_ID, "device_gyro_otc_temp", SENSOR_DEVICE_TYPE},
	{GYRO_NO_DATA_ID, "device_gyro_no_data", SENSOR_DEVICE_TYPE},

	{GYRO_SUB_DATA_BLOCK_ID, "device_gyro_sub_data_block", SENSOR_DEVICE_TYPE},
	{GYRO_SUB_NO_DATA_ID, "device_gyro_sub_no_data", SENSOR_DEVICE_TYPE},
	{GYRO_SUB_ESD_REST_ID, "device_gyro_sub_esd_reset", SENSOR_DEVICE_TYPE},

	{MAG_INIT_FAIL_ID, "device_mag_init_fail", SENSOR_DEVICE_TYPE},
	{MAG_I2C_ERR_ID, "device_mag_i2c_err", SENSOR_DEVICE_TYPE},
	{MAG_ALLOC_FAIL_ID, "device_mag_alloc_fail", SENSOR_DEVICE_TYPE},
	{MAG_ESD_REST_ID, "device_mag_esd_reset", SENSOR_DEVICE_TYPE},
	{MAG_NO_INTERRUPT_ID, "device_mag_no_irq", SENSOR_DEVICE_TYPE},
	{MAG_FIRST_REPORT_DELAY_COUNT_ID, "device_mag_rpt_delay", SENSOR_DEBUG_DEVICE_TYPE},
	{MAG_ORIGIN_DATA_TO_ZERO_ID, "device_mag_to_zero", SENSOR_DEBUG_DEVICE_TYPE},
	{MAG_CALI_DATA_ID, "device_mag_cali_data", SENSOR_DEBUG_DEVICE_TYPE},
	{MAG_DATA_BLOCK_ID, "device_mag_data_block_data", SENSOR_DEBUG_DEVICE_TYPE},
	{MAG_DATA_FULL_RANGE_ID, "device_mag_data_full_range", SENSOR_DEBUG_DEVICE_TYPE},
	{MAG_NO_DATA_ID, "device_mag_no_data_range", SENSOR_DEVICE_TYPE},
	{MAG_ENABLE_FAIL_ID, "device_mag_enable_fail_range", SENSOR_DEVICE_TYPE},


	{SAR_INIT_FAIL_ID, "device_sar_init_fail", SENSOR_DEVICE_TYPE},
	{SAR_I2C_ERR_ID, "device_sar_i2c_err", SENSOR_DEVICE_TYPE},
	{SAR_ALLOC_FAIL_ID, "device_sar_alloc_fail", SENSOR_DEVICE_TYPE},
	{SAR_ESD_REST_ID, "device_sar_esd_reset", SENSOR_DEVICE_TYPE},
	{SAR_NO_INTERRUPT_ID, "device_sar_no_irq", SENSOR_DEVICE_TYPE},
	{SAR_FIRST_REPORT_DELAY_COUNT_ID, "device_sar_rpt_delay", SENSOR_DEBUG_DEVICE_TYPE},
	{SAR_ORIGIN_DATA_TO_ZERO_ID, "device_sar_to_zero", SENSOR_DEBUG_DEVICE_TYPE},
	{SAR_CALI_DATA_ID, "device_sar_cali_data", SENSOR_DEBUG_DEVICE_TYPE},
	{HALL_STATUS_ID, "device_hall_status", SENSOR_DEVICE_TYPE},
	{HALL_TRIGGER_COUNT, "hall_trigger_count", SENSOR_DEVICE_TYPE},

	{BAROMETER_I2C_ERR_ID, "device_barometer_i2c_err", SENSOR_DEVICE_TYPE},

	{HALL_I2C_ERR_ID, "device_hall_i2c_err", SENSOR_DEVICE_TYPE},
	{HALL_STYLUS_DETECT_INIT_FAIL_ID, "stylus_detect_init_fail", SENSOR_DEVICE_TYPE},
	{HALL_STYLUS_DETECT_I2C_ERR_ID, "stylus_detect_i2c_err", SENSOR_DEVICE_TYPE},
	{HALL_STYLUS_DETECT_NO_DATA_ID, "stylus_detect_no_data_err", SENSOR_DEVICE_TYPE},

	{FOLD_DEVICE_FOLDE_COUNT_ID, "device_fold_count", SENSOR_DEVICE_TYPE},
	{FOLD_DEVICE_USE_HALL_ANGLE_COUNT_ID, "device_use_hall_angle_count", SENSOR_DEVICE_TYPE},
	{HINGE_DETECT_INTERVAL_TIME_ID, "device_hinge_detect_interval_time_id", SENSOR_DEVICE_TYPE},
	{HINGE_DETECT_SUB_ACCEL_INIT_FAIL_ID, "device_hinge_detect_sub_accel_init_fail_id", SENSOR_DEVICE_TYPE},
	{HINGE_DETECT_SUB_ACCEL_DATA_LOST_ID, "device_hinge_detect_sub_accel_data_lost_id", SENSOR_DEVICE_TYPE},
	{HINGE_DETECT_FOLD_HALL_INIT_FAIL_ID, "device_hinge_detect_fold_hall_init_fail_id", SENSOR_DEVICE_TYPE},
	{HINGE_DETECT_FOLD_HALL_DATA_LOST_ID, "device_hinge_detect_fold_hall_data_lost_id", SENSOR_DEVICE_TYPE},
	{HINGE_DETECT_MAIN_ACCEL_INIT_FAIL_ID, "device_hinge_detect_main_accel_init_fail_id", SENSOR_DEVICE_TYPE},
	{HINGE_DETECT_MAIN_ACCEL_DATA_LOST_ID, "device_hinge_detect_main_accel_data_lost_id", SENSOR_DEVICE_TYPE},
	{HINGE_DETECT_MAIN_GYRO_INIT_FAIL_ID, "device_hinge_detect_main_gyro_init_fail_id", SENSOR_DEVICE_TYPE},
	{HINGE_DETECT_MAIN_GYRO_DATA_LOST_ID, "device_hinge_detect_main_gyro_data_lost_id", SENSOR_DEVICE_TYPE},
	{HINGE_DETECT_SUB_GYRO_INIT_FAIL_ID, "device_hinge_detect_sub_gyro_init_fail_id", SENSOR_DEVICE_TYPE},
	{HINGE_DETECT_SUB_GYRO_DATA_LOST_ID, "device_hinge_detect_sub_gyro_data_lost_id", SENSOR_DEVICE_TYPE},

	{FREE_FALL_TRIGGER_ID, "device_free_fall", SENSOR_DEVICE_TYPE},

	{POWER_SENSOR_INFO_ID, "debug_power_sns_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_WAKE_UP_RATE_ID, "debug_power_wakeup_rate", SENSOR_DEBUG_POWER_TYPE},
	{POWER_ADSP_SLEEP_RATIO_ID, "power_adsp_sleep_ratio", SENSOR_POWER_TYPE},
	{POWER_ADSP_SLEEP_STATS_ID, "power_adsp_sleep_stats", SENSOR_POWER_TYPE},
	{POWER_ISLAND_EXT_CNT_ID, "debug_power_island_ext_cnt", SENSOR_DEBUG_POWER_TYPE},

	{POWER_ACCEL_INFO_ID, "debug_power_acc_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_GYRO_INFO_ID, "debug_power_gyro_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_MAG_INFO_ID, "debug_power_mag_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_PROXIMITY_INFO_ID, "debug_power_prox_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_LIGHT_INFO_ID, "debug_power_light_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_WISE_LIGHT_INFO_ID, "debug_power_wise_ligt_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_AMBIENT_LIGHT_INFO_ID, "debug_power_ambient_light_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_WISE_RGB_INFO_ID, "debug_power_wise_rgb_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_RGB_INFO_ID, "debug_power_rgb_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_FLICKER_INFO_ID, "debug_power_flicker_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_AMBIENT_LIGHT_REAR_INFO_ID, "debug_power_ambient_light_rear_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_RGB_REAR_INFO_ID, "debug_power_rgb_rear_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_FLICKER_REAR_INFO_ID, "debug_power_flicker_rear_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_SPECTRAL_REAR_INFO_ID, "debug_power_spectral_rear_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_HALL_INFO_ID, "debug_power_hall_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_PRESSURE_INFO_ID, "debug_power_pressure_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_SAR_INFO_ID, "debug_power_sar_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_SARS_INFO_ID, "debug_power_sars_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_OIS_SYSTEM_INFO_ID, "debug_power_ois_system_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_DIRECT_CHANNEL_INFO_ID, "debug_power_direct_channel_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_SHAKING_INFO_ID, "debug_power_shaking_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_LAY_DETECT_INFO_ID, "debug_power_lay_detect_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_GESTURE_PROX_INFO_ID, "debug_power_gesture_prox_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_PHONE_PROX_INFO_ID, "debug_power_phone_prox_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_EXPLORER_GYRO_INFO_ID, "debug_power_explorer_gyro_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_HINGE_DETECT_INFO_ID, "debug_power_hinge_detect_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_FOLD_STATE_INFO_ID, "debug_power_fold_state_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_OPLUS_CAMERA_ORIENT_INFO_ID, "debug_power_oplus_camera_orient_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_TRESTLE_HALL_INFO_ID, "debug_power_trestle_hall_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_FOLD_FUSION_LIGHT_INFO_ID, "debug_power_fold_fusion_light_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_PAD_ALS_INFO_ID, "debug_power_pad_als_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_PAD_WISE_RGB_INFO_ID, "debug_power_pad_wise_rgb_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_POCKET_INFO_ID, "debug_power_pocket_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_SUB_DEVICE_ORIENT_INFO_ID, "debug_power_sub_device_orient_info", SENSOR_DEBUG_POWER_TYPE},
	{POWER_CHOP_INFO_ID, "debug_power_chop_info", SENSOR_DEBUG_POWER_TYPE},


	{DOUBLE_TAP_REPORTED_ID, "device_double_tap_reported", SENSOR_DEBUG_DEVICE_TYPE},
	{DOUBLE_TAP_PREVENTED_BY_NEAR_ID, "device_double_tap_prevented_by_near", SENSOR_DEBUG_DEVICE_TYPE},
	{DOUBLE_TAP_PREVENTED_BY_ATTITUDE_ID, "device_double_prevented_by_attitude", SENSOR_DEBUG_DEVICE_TYPE},
	{DOUBLE_TAP_PREVENTED_BY_FREEFALL_Z_ID, "device_double_prevented_by_freefall_z", SENSOR_DEBUG_DEVICE_TYPE},
	{DOUBLE_TAP_PREVENTED_BY_FREEFALL_SLOPE_ID, "device_double_prevented_by_freefall_slope", SENSOR_DEBUG_DEVICE_TYPE},

	{ALAILABLE_SENSOR_LIST_ID, "available_sensor_list", SENSOR_DEBUG_DEVICE_TYPE},

	{HAL_SENSOR_NOT_FOUND, "device_hal_not_found", SENSOR_DEVICE_TYPE},
	{HAL_QMI_ERROR, "device_hal_qmi_error", SENSOR_DEVICE_TYPE},
	{HAL_SENSOR_TIMESTAMP_ERROR, "device_hal_ts_error", SENSOR_DEBUG_DEVICE_TYPE}

};

/*see physical_sensor_list in <oplus_cm_island.c>*/
static char *physical_sensor_list[PHY_SENSOR_NUM] =
{
	"proximity",
	"ambient_light",
	"wise_light",
	"wise_rgb",
	"rgb",
	"flicker",
	"ambient_light_rear",
	"rgb_rear",
	"flicker_rear",
	"spectral_rear",
	"accel",
	"gyro",
	"mag",
	"hall",
	"pressure",
	"sar",
	"sars",
	"resampler",
};

/*see virtual_sensor_list in <oplus_cm_island.c>*/
static char *virtual_sensor_list[VIR_SENSOR_NUM] =
{
	"cm",
	"lux_aod",
	"pick_up_motion",
	"pedometer",
	"pedometer_minute",
	"free_fall",
	"amd_oplus",
	"oplus_activity_recognition",
	"oplus_double_tap",
	"oplus_measurement",
	"cmc_oplus",
	"device_orient",
	"camera_protect",
	"elevator_detect",
	"ois_system",
	"direct_channel",
	"shaking",
	"lay_detect",
	"gesture_prox",
	"phone_prox",
	"explorer_gyro",
	"hinge_detect",
	"fold_state",
	"oplus_camera_orient",
	"trestle_hall",
	"fold_fusion_light",
	"pad_als",
	"pad_wise_rgb",
	"pocket",
	"sub_device_orient",
	"chop",
};
void send_uevent_to_fb(int monitor_info) {
	struct device *dev=NULL;
	char *env[2]={0x00};

	pr_info("monitor_info=%d\n", monitor_info);
	dev = &g_sensor_fb_cxt->sensor_fb_dev->dev;
	env[0] = kasprintf(GFP_KERNEL, "monitor_info=%d", monitor_info);
	env[1] = NULL;
	kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, env);
}

EXPORT_SYMBOL(send_uevent_to_fb);

static int find_event_id(int16_t event_id)
{
	int len = sizeof(g_fb_conf) / sizeof(g_fb_conf[0]);
	int ret = -1;
	int index = 0;

	for (index = 0; index < len; index++) {
		if (g_fb_conf[index].event_id == event_id) {
			ret = index;
		}
	}
	return ret;
}

static struct timespec64 oplus_current_kernel_time(void) {
	struct timespec64 ts64;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 0))
        ts64 = current_kernel_time64();
#else
	ktime_get_coarse_real_ts64(&ts64);
#endif
	return ts64;
}

#define MSM_ARCH_TIMER_FREQ 19200000
static inline u64 get_time_in_msec(u64 counter)
{
	do_div(counter, (MSM_ARCH_TIMER_FREQ/MSEC_PER_SEC));
	return counter;
}

static u64 oplus_rpmh_master_get_sleeptime(struct msm_rpmh_master_stats *record)
{
	uint64_t accumulated_duration = record->accumulated_duration;
	/*
	 * If a master is in sleep when reading the sleep stats from SMEM
	 * adjust the accumulated sleep duration to show actual sleep time.
	 * This ensures that the displayed stats are real when used for
	 * the purpose of computing battery utilization.
	 */
        if (record->last_entered > record->last_exited) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
                accumulated_duration += (arch_counter_get_cntvct() - record->last_entered);
#else
                accumulated_duration += (__arch_counter_get_cntvct() - record->last_entered);
#endif
        }
	return get_time_in_msec(accumulated_duration);
}

static int oplus_subsystem_sleeptime(char *name, u64 *sleeptime)
{
	int i = 0, found = 0;
	size_t size = 0;
	struct msm_rpmh_master_stats *record = NULL;

	if((name == NULL) || (sleeptime == NULL))
		return 0;

	for (i = 0; i < ARRAY_SIZE(rpmh_masters); i++) {
		if(strncmp(rpmh_masters[i].master_name, name, strlen(name)) != 0){
			continue;
		}
		record = (struct msm_rpmh_master_stats *) qcom_smem_get(rpmh_masters[i].pid, rpmh_masters[i].smem_id, &size);
		if (!IS_ERR_OR_NULL(record)) {
			pr_info("%s : %s:0x%x\n", __func__, rpmh_masters[i].master_name, record->counts);
			pr_info("%s found: %s:0x%x\n", __func__, rpmh_masters[i].master_name, record->counts);
			found = 1;
			*sleeptime = oplus_rpmh_master_get_sleeptime(record);
			goto finish;
		}
	}
finish:
	return found;
}

static void reset_subsys_sleep_stats(void) {
	int index = 0;
	struct timespec64 now_time;

	memset(&g_subsys_sleep_stats, 0 , sizeof(struct subsys_sleep_stats));
	for (index = 0; index < SUBSYS_COUNTS; index++) {
		g_subsys_sleep_stats.sleep_info[index].name = subsys_names[index];
	}
	now_time = oplus_current_kernel_time();
	g_subsys_sleep_stats.time_s = (now_time.tv_sec * 1000 + now_time.tv_nsec / 1000000);
}

static void statistical_subsys_sleep_info(void) {
	int index = 0;
	int cnt = 0;
	for (index = 0; index < SUBSYS_COUNTS; index++) {
		uint64_t min = 100, max = 0, sum = 0, avr = 0;
		for (cnt = 0; cnt < g_subsys_sleep_stats.count; cnt++) {
			min = g_subsys_sleep_stats.sleep_info[index].arr[cnt] < min ?
						g_subsys_sleep_stats.sleep_info[index].arr[cnt] : min;
			max = g_subsys_sleep_stats.sleep_info[index].arr[cnt] > max ?
						g_subsys_sleep_stats.sleep_info[index].arr[cnt] : max;
			sum += g_subsys_sleep_stats.sleep_info[index].arr[cnt];
		}
		avr = sum / g_subsys_sleep_stats.count;
		g_subsys_sleep_stats.sleep_info[index].min = min;
		g_subsys_sleep_stats.sleep_info[index].max = max;
		g_subsys_sleep_stats.sleep_info[index].avr = avr;

		pr_info("subsys_sleep_stats: subsys_name=%s, cnt=%d, max=%llu, min=%llu, avr=%llu\n",
				g_subsys_sleep_stats.sleep_info[index].name, g_subsys_sleep_stats.count, max, min, avr);
	}
}

static void subsystem_desc_init(struct subsystem_desc *subsystem_desc) {
	int index = 0;
	for (index = 0; index < SUBSYS_COUNTS; index++) {
		subsystem_desc[index].subsys_sleep_time_s = 0;
		subsystem_desc[index].ap_sleep_time_s = 0;
		subsystem_desc[index].subsys_sleep_time_p = 0;
		subsystem_desc[index].ap_sleep_time_p = 0;
		subsystem_desc[index].subsys_sleep_ratio = 0;
		subsystem_desc[index].is_err = 0;
		subsystem_desc[index].subsys_name = subsys_names[index];
	}
}

static void read_subsystem_sleep_time(struct subsystem_desc *subsystem_desc, int status) {
	int index = 0;
	int ret = 0;
	struct timespec64 now_time;
	if (status == 1) {
		for (index = 0; index < SUBSYS_COUNTS; index++) {
			ret = oplus_subsystem_sleeptime(subsystem_desc[index].subsys_name,
					&subsystem_desc[index].subsys_sleep_time_p);
			now_time = oplus_current_kernel_time();
			subsystem_desc[index].ap_sleep_time_p = (now_time.tv_sec * 1000 + now_time.tv_nsec / 1000000);
			subsystem_desc[index].is_err = ret;
		}
	}else {
		for (index = 0; index < SUBSYS_COUNTS ; index++) {
			ret = oplus_subsystem_sleeptime(subsystem_desc[index].subsys_name,
					&subsystem_desc[index].subsys_sleep_time_s);
			now_time = oplus_current_kernel_time();
			subsystem_desc[index].ap_sleep_time_s = (now_time.tv_sec * 1000 + now_time.tv_nsec / 1000000);
			subsystem_desc[index].is_err = ret;
		}
	}
}

static void cal_subsystem_sleep_ratio(struct subsystem_desc *subsystem_desc) {
	int index = 0;
	uint64_t subsys_sleep_ratio = 0;
	uint64_t subsys_sleep = 0;
	uint64_t ap_sleep = 0;
	char *adsp_sleep_ratio_fied = "power_adsp_sleep_ratio";
	char *adsp_sleep_stats_fied = "power_adsp_sleep_stats";
	uint64_t subsys_sleep_stats = 0;
	char payload[1024] = {0x00};
	int flag = 0;
	bool stats_cnt = false;

	for (index = 0; index < SUBSYS_COUNTS; index++) {
		if(subsystem_desc[index].is_err != 0) {
			subsys_sleep = subsystem_desc[index].subsys_sleep_time_p
							- subsystem_desc[index].subsys_sleep_time_s;
			ap_sleep = subsystem_desc[index].ap_sleep_time_p - subsystem_desc[index].ap_sleep_time_s;
			subsys_sleep_ratio = subsys_sleep * 100 / ap_sleep;
			subsystem_desc[index].subsys_sleep_ratio = subsys_sleep_ratio;

			pr_info("subsys_sleep_ratio of %s: subsys*100/ap = %llu*100/%llu=%llu\n",
				subsystem_desc[index].subsys_name, subsys_sleep, ap_sleep, subsys_sleep_ratio);

			if (ap_sleep > (24 * 3600 * 1000 / SLEEP_INFO_CNT)) {
				g_subsys_sleep_stats.sleep_info[index].arr[g_subsys_sleep_stats.count] = subsystem_desc[index].subsys_sleep_ratio;
				stats_cnt = true;
				pr_info("subsys_sleep_stats %s[%d]: subsys*100/ap = %llu\n",
					subsystem_desc[index].subsys_name, g_subsys_sleep_stats.count,
					g_subsys_sleep_stats.sleep_info[index].arr[g_subsys_sleep_stats.count]);
			}

			if ((ap_sleep > (3 * 3600 * 1000)) && (subsys_sleep_ratio < 10)) {
					if (index == 0 || index == 2) {/*ADSP:0, SLPI:2*/
						flag = 0; /*Do not feedback power_info, omly print*/
					}
					memset(payload, 0 , sizeof(payload));
					scnprintf(payload, sizeof(payload),
							"NULL$$EventField@@%s$$FieldData@@%s$$detailData@@%llu",
							adsp_sleep_ratio_fied,
							subsystem_desc[index].subsys_name,
							subsys_sleep_ratio);
					#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE)
					oplus_kevent_fb(FB_SENSOR, SENSOR_POWER_TYPE, payload);
					#endif
			}
		}
	}
	if (stats_cnt) {
		g_subsys_sleep_stats.count++;
	}

	if ((g_subsys_sleep_stats.count >= SLEEP_INFO_CNT) ||
		(g_subsys_sleep_stats.count > 0 && (subsystem_desc[0].ap_sleep_time_p - g_subsys_sleep_stats.time_s >= (24 * 3600 * 1000)))) {
		statistical_subsys_sleep_info();
		for (index = 0; index < SUBSYS_COUNTS; index++) {
			subsys_sleep_stats = (g_subsys_sleep_stats.sleep_info[index].max << 0) |
							(g_subsys_sleep_stats.sleep_info[index].min << 3) | (g_subsys_sleep_stats.sleep_info[index].avr<< 6);
			memset(payload, 0 , sizeof(payload));
			scnprintf(payload, sizeof(payload),
					"NULL$$EventField@@%s$$FieldData@@%s$$detailData@@%llu",
					adsp_sleep_stats_fied,
					subsystem_desc[index].subsys_name,
					subsys_sleep_stats);
			#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE)
			oplus_kevent_fb(FB_SENSOR, SENSOR_POWER_TYPE, payload);
			#endif
		}
		reset_subsys_sleep_stats();
	}
        /*
        TO DO
	if (flag == 1) {
		send_uevent_to_fb(REQ_SSC_POWER_INFO);
	}
        */
}

static ssize_t adsp_notify_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensor_fb_cxt *sensor_fb_cxt = g_sensor_fb_cxt;
	uint16_t adsp_event_counts = 0;

	spin_lock(&sensor_fb_cxt->rw_lock);
	adsp_event_counts = sensor_fb_cxt->adsp_event_counts;
	spin_unlock(&sensor_fb_cxt->rw_lock);
	pr_info("adsp_value = %u\n", adsp_event_counts);
	return snprintf(buf, PAGE_SIZE, "%u\n", adsp_event_counts);
}

static ssize_t adsp_notify_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct sensor_fb_cxt *sensor_fb_cxt = g_sensor_fb_cxt;
	uint16_t adsp_event_counts = 0;
	uint16_t node_type = 0;
	int err = 0;

	err = sscanf(buf, "%hu %hu", &node_type, &adsp_event_counts);
	if (err < 0) {
		pr_err("adsp_notify_store error: err = %d\n", err);
		return err;
	}

	spin_lock(&sensor_fb_cxt->rw_lock);
	sensor_fb_cxt->adsp_event_counts = adsp_event_counts;
	sensor_fb_cxt->node_type = node_type;
	spin_unlock(&sensor_fb_cxt->rw_lock);
	pr_info("adsp_value = %d, node_type=%d\n", adsp_event_counts,
		node_type);

	set_bit(THREAD_WAKEUP, (unsigned long *)&sensor_fb_cxt->wakeup_flag);
	/*wake_up_interruptible(&sensor_fb_cxt->wq);*/
	wake_up(&sensor_fb_cxt->wq);

	return count;
}


static ssize_t hal_info_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	uint16_t event_ct = 0;
	uint16_t event_id = 0;
	char strbuf[32]= {0x00};
	int err = 0;
	int index = 0;
	unsigned char payload[1024] = {0x00};


	memset(strbuf, 0, 32);
	memset(payload, 0 ,1024);

	pr_info("hal_info_store\n");

	err = sscanf(buf, "%hu %hu %31s", &event_id, &event_ct, strbuf);
	if (err < 0) {
		pr_err("hal_info_store error: err = %d\n", err);
		return count;
	}

	strbuf[31] = '\0';

	index = find_event_id(event_id);
	if (index == -1) {
		pr_info("nout find event_id =%d\n", event_id);
		return count;
	}

	scnprintf(payload, sizeof(payload),
				"NULL$$EventField@@%s$$FieldData@@%d$$detailData@@%s",
				g_fb_conf[index].fb_field,
				event_ct,
				strbuf);
	pr_info("payload =%s\n", payload);

	#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE)
	oplus_kevent_fb(FB_SENSOR, g_fb_conf[index].fb_event_id, payload);
	#endif
	return count;
}


static ssize_t test_id_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct sensor_fb_cxt *sensor_fb_cxt = g_sensor_fb_cxt;
	uint16_t adsp_event_counts = 0;
	uint16_t node_type = 0;
	uint16_t event_id = 0;
	uint16_t event_data = 0;
	int err = 0;

	err = sscanf(buf, "%hu %hu %hu %hu", &node_type, &adsp_event_counts, &event_id,
			&event_data);

	if (err < 0) {
		pr_err("test_id_store error: err = %d\n", err);
		return count;
	}

	spin_lock(&sensor_fb_cxt->rw_lock);
	sensor_fb_cxt->adsp_event_counts = adsp_event_counts;
	sensor_fb_cxt->node_type = node_type;
	spin_unlock(&sensor_fb_cxt->rw_lock);

	sensor_fb_cxt->fb_smem.event[0].event_id = event_id;
	sensor_fb_cxt->fb_smem.event[0].count = event_data;

	pr_info("test_id_store adsp_value = %d, node_type=%d \n", adsp_event_counts,
		node_type);
	pr_info("test_id_store event_id = %d, event_data=%d \n", event_id, event_data);


	set_bit(THREAD_WAKEUP, (unsigned long *)&sensor_fb_cxt->wakeup_flag);
	/*wake_up_interruptible(&sensor_fb_cxt->wq);*/
	wake_up(&sensor_fb_cxt->wq);

	return count;
}

static ssize_t sensor_list_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensor_fb_cxt *sensor_fb_cxt = g_sensor_fb_cxt;
	uint16_t sensor_list[2] = {0x00};
	spin_lock(&sensor_fb_cxt->rw_lock);
	sensor_list[0] = sensor_fb_cxt->sensor_list[0];
	sensor_list[1] = sensor_fb_cxt->sensor_list[1];
	spin_unlock(&sensor_fb_cxt->rw_lock);
	pr_info("phy = 0x%x, virt = 0x%x\n", sensor_list[0], sensor_list[1]);

	return snprintf(buf, PAGE_SIZE, "phy = 0x%x, virt = 0x%x\n", sensor_list[0],
			sensor_list[1]);

}

static ssize_t adsp_recv_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	int value = 0;

	err = sscanf(buf, "%d", &value);
	if (err < 0) {
		pr_err("hal_info_store error: err = %d\n", err);
		return count;
	}

	send_uevent_to_fb(value);
	return count;
}

DEVICE_ATTR(adsp_notify, 0644, adsp_notify_show, adsp_notify_store);
DEVICE_ATTR(adsp_recv, 0644, NULL, adsp_recv_store);
DEVICE_ATTR(hal_info, 0644, NULL, hal_info_store);
DEVICE_ATTR(test_id, 0644, NULL, test_id_store);
DEVICE_ATTR(sensor_list, 0644, sensor_list_show, NULL);

static struct attribute *sensor_feedback_attributes[] = {
	&dev_attr_adsp_notify.attr,
	&dev_attr_adsp_recv.attr,
	&dev_attr_hal_info.attr,
	&dev_attr_test_id.attr,
	&dev_attr_sensor_list.attr,
	NULL
};

static struct attribute_group sensor_feedback_attribute_group = {
	.attrs = sensor_feedback_attributes
};

#define SMEM_SENSOR_FEEDBACK (128)
static int read_data_from_share_mem(struct sensor_fb_cxt *sensor_fb_cxt)
{
	size_t smem_size = 0;
	void *smem_addr = NULL;
	struct fb_event_smem *fb_event = NULL;

	smem_addr = qcom_smem_get(QCOM_SMEM_HOST_ANY,
			SMEM_SENSOR_FEEDBACK,
			&smem_size);

	if (IS_ERR(smem_addr)) {
		pr_err("unable to acquire smem SMEM_SENSOR_FEEDBACK entry\n");
		return -1;
	}

	fb_event = (struct fb_event_smem *)smem_addr;

	if (fb_event == ERR_PTR(-EPROBE_DEFER)) {
		fb_event = NULL;
		return -2;
	}

	memcpy((void *)&sensor_fb_cxt->fb_smem, (void *)fb_event, smem_size);
	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static int reserve_fb_share_mem(void)
{
	int rc = 0;
	size_t smem_size = 0;

	smem_size = ALIGN4(struct fb_event_smem);
	rc = qcom_smem_alloc(QCOM_SMEM_HOST_ANY, SMEM_SENSOR_FEEDBACK, smem_size);
	if (rc < 0 && rc != -EEXIST) {
		pr_err("%s smem_alloc fail\n", __func__);
		rc = -EFAULT;
		 return rc;
	}
	return 0;
}
#endif

static struct delivery_type delivery_t[2] = {
	{
		.name = "wakeup",
		.type = WAKE_UP,
	},
	{
		.name = "no_akeup",
		.type = NO_WAKEUP,
	},
};

static struct proc_type proc_t[6] = {
	{
		.name = "ssc",
		.type = SSC,
	},
	{
		.name = "apss",
		.type = APSS,
	},
	{
		.name = "adsp",
		.type = ADSP,
	},
	{
		.name = "mdsp",
		.type = MDSP,
	},
	{
		.name = "cdsp",
		.type = CDSP,
	},
	{
		.name = "ncs",
		.type = NCS,
	},
};

static struct req_msg_id req_msg_id_t[3] = {
	{
		.name = "513",
		.type = MSG_ID_513,
	},
	{
		.name = "514",
		.type = MSG_ID_514,
	},
	{
		.name = "other",
		.type = MSG_ID_OTHER,
	},
};

int procce_special_event_id(unsigned short event_id, int count,
	struct sensor_fb_cxt *sensor_fb_cxt)
{
	int ret = 0;
	int index = 0;
	int ii = 0;

	if (event_id == ALAILABLE_SENSOR_LIST_ID) {
		sensor_fb_cxt->sensor_list[0] = (uint32_t)
			sensor_fb_cxt->fb_smem.event[count].buff[0];
		sensor_fb_cxt->sensor_list[1] = (uint32_t)
			sensor_fb_cxt->fb_smem.event[count].buff[1];
		pr_info("available_sensor_list virt_sns = 0x%x, phy_sns = 0x%x\n",
			sensor_fb_cxt->sensor_list[0], sensor_fb_cxt->sensor_list[1]);
		for (ii = 0; ii < VIR_SENSOR_NUM; ii++) {
			if (sensor_fb_cxt->fb_smem.event[count].buff[0] & ((1 << ii))) {
				pr_info("available_virt_sns[ii]: %s= \n", virtual_sensor_list[ii]);
			}
		}
		for (ii = 0; ii < PHY_SENSOR_NUM; ii++) {
			if (sensor_fb_cxt->fb_smem.event[count].buff[1] & ((1 << ii))) {
				pr_info("available_phy_sns[ii]: %s= \n", physical_sensor_list[ii]);
			}
		}
		ret = 1;
	} else if (event_id >= POWER_ACCEL_INFO_ID && event_id <= POWER_CHOP_INFO_ID) {
		index = find_event_id(event_id);
		if (index >= 0) {
			pr_info("%s: req_count:%d, wakeup_rate:%d\n",
				g_fb_conf[index].fb_field,
				sensor_fb_cxt->fb_smem.event[count].buff[4],
				sensor_fb_cxt->fb_smem.event[count].buff[3]);
			for (ii = 0; ii < sensor_fb_cxt->fb_smem.event[count].buff[4]; ii++) {
				pr_info("%s req[%d] : proc_type=%s, delivery_type=%s, req_msg_id=%s\n",
					g_fb_conf[index].fb_field, ii,
					proc_t[(sensor_fb_cxt->fb_smem.event[count].buff[0] >> (ii * 3)) & 0x07].name,
					delivery_t[(sensor_fb_cxt->fb_smem.event[count].buff[1] >> (ii * 3)) & 0x07].name,
					req_msg_id_t[(sensor_fb_cxt->fb_smem.event[count].buff[2] >> (ii * 3)) & 0x07].name);
			}
		ret = 1;
		}
	} else if (event_id == POWER_SENSOR_INFO_ID) {
		pr_info("enable_sensor_list virt_sns = 0x%x, phy_sns = 0x%x\n",
			sensor_fb_cxt->fb_smem.event[count].buff[0],
			sensor_fb_cxt->fb_smem.event[count].buff[1]);
		for (ii = 0; ii < VIR_SENSOR_NUM; ii++) {
			if (sensor_fb_cxt->fb_smem.event[count].buff[0] & (1 << ii)) {
				pr_info("enable_virt_sns[%d]: %s, in_island %d\n",
					ii, virtual_sensor_list[ii],
					(sensor_fb_cxt->fb_smem.event[count].buff[2] >> ii) & 1);
			}
		}
		for (ii = 0; ii < PHY_SENSOR_NUM; ii++) {
			if (sensor_fb_cxt->fb_smem.event[count].buff[1] & ((1 << ii))) {
				pr_info("enable_phy_sns[%d]: %s, in_island %d\n",
					ii, physical_sensor_list[ii],
					(sensor_fb_cxt->fb_smem.event[count].buff[3] >> ii) & 1);
			}
		}
		ret = 1;
	} else if (event_id == POWER_WAKE_UP_RATE_ID) {
		pr_info("sensor_power_monitor: normal_mode_wakeup_rate:%d, island_mode_wakeup_rate:%d, mcps:%d, latency:%d\n",
			sensor_fb_cxt->fb_smem.event[count].buff[0],
			sensor_fb_cxt->fb_smem.event[count].buff[1],
			sensor_fb_cxt->fb_smem.event[count].buff[2],
			sensor_fb_cxt->fb_smem.event[count].buff[3]);
		ret = 1;
	} else if (event_id == POWER_ISLAND_EXT_CNT_ID) {
		pr_info("sensor_power_monitor: last 5 min, island exit cnt %d", (int)sensor_fb_cxt->fb_smem.event[count].buff[0]);
		ret = 1;
	}

	return ret;
}


static int parse_shr_info(struct sensor_fb_cxt *sensor_fb_cxt)
{
	int ret = 0;
	int count = 0;
	uint16_t event_id = 0;
	int index = 0;
	unsigned char payload[1024] = {0x00};
	int fb_len = 0;
	unsigned char detail_buff[128] = {0x00};
	uint16_t adsp_event_counts = 0;

	spin_lock(&sensor_fb_cxt->rw_lock);
	adsp_event_counts = sensor_fb_cxt->adsp_event_counts;
	spin_unlock(&sensor_fb_cxt->rw_lock);

	for (count = 0; count < adsp_event_counts; count ++) {
		event_id = sensor_fb_cxt->fb_smem.event[count].event_id;
		pr_info("event_id =%d, count =%d\n", event_id, count);

		index = find_event_id(event_id);
		if (index == -1) {
			pr_info("not find event_id =%d, count =%d\n", event_id, count);
			continue;
		}

		ret = procce_special_event_id(event_id, count, sensor_fb_cxt);
		if (ret == 1) {
			continue;
		}

		memset(payload, 0, sizeof(payload));
		memset(detail_buff, 0, sizeof(detail_buff));
		snprintf(detail_buff, sizeof(detail_buff), "%d %d %d %d %d",
			sensor_fb_cxt->fb_smem.event[count].buff[0],
			sensor_fb_cxt->fb_smem.event[count].buff[1],
			sensor_fb_cxt->fb_smem.event[count].buff[2],
			sensor_fb_cxt->fb_smem.event[count].buff[3],
			sensor_fb_cxt->fb_smem.event[count].buff[4]);
		fb_len += scnprintf(payload, sizeof(payload),
				"NULL$$EventField@@%s$$FieldData@@%d$$detailData@@%s$$SensorName@@0x%x",
				g_fb_conf[index].fb_field,
				sensor_fb_cxt->fb_smem.event[count].count,
				detail_buff,
				sensor_fb_cxt->fb_smem.event[count].name);
		pr_info("payload1 =%s\n", payload);
#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE)
		oplus_kevent_fb(FB_SENSOR, g_fb_conf[index].fb_event_id, payload);
		pr_info("send oplus kevent fb\n");
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_TRACE_SENSOR)

		if (enable_report) {
			oplus_trace_sensor_fault_report(g_fb_conf[index].event_id,
					g_fb_conf[index].fb_event_id, g_fb_conf[index].fb_field,
					sensor_fb_cxt->fb_smem.event[count].count);
		}
#endif
	}

	return ret;
}


static int sensor_report_thread(void *arg)
{
	int ret = 0;
	struct sensor_fb_cxt *sensor_fb_cxt = (struct sensor_fb_cxt *)arg;
	uint16_t node_type = 0;
	pr_info("sensor_feedback: sensor_report_thread step1!\n");

	while (!kthread_should_stop()) {
		wait_event_interruptible(sensor_fb_cxt->wq, test_bit(THREAD_WAKEUP,
				(unsigned long *)&sensor_fb_cxt->wakeup_flag));

		clear_bit(THREAD_WAKEUP, (unsigned long *)&sensor_fb_cxt->wakeup_flag);
		set_bit(THREAD_SLEEP, (unsigned long *)&sensor_fb_cxt->wakeup_flag);
		spin_lock(&sensor_fb_cxt->rw_lock);
		node_type = sensor_fb_cxt->node_type;
		spin_unlock(&sensor_fb_cxt->rw_lock);

		if (node_type == 0) {
			ret = read_data_from_share_mem(sensor_fb_cxt);
		} else if (node_type == 2) { //sleep ratio wakeup
			read_subsystem_sleep_time(sensor_fb_cxt->subsystem_desc, 1);
			cal_subsystem_sleep_ratio(sensor_fb_cxt->subsystem_desc);
		} else if (node_type == 3) { //power done
			read_subsystem_sleep_time(sensor_fb_cxt->subsystem_desc, 0);

		} else {
			pr_info("sensor_feedback test from node\n");
		}

		ret = parse_shr_info(sensor_fb_cxt);
		spin_lock(&sensor_fb_cxt->rw_lock);
		memset((void *)&sensor_fb_cxt->fb_smem, 0, sizeof(struct fb_event_smem));
		sensor_fb_cxt->adsp_event_counts = 0;
		spin_unlock(&sensor_fb_cxt->rw_lock);
	}

	pr_info("sensor_feedback ret =%d\n", ret);
	return ret;
}

static ssize_t sensor_list_read_proc(struct file *file, char __user *buf,
	size_t count, loff_t *off)
{
	char page[128] = {0};
	int len = 0;
	struct sensor_fb_cxt *sensor_fb_cxt = (struct sensor_fb_cxt *)PDE_DATA(
			file_inode(file));

	len = snprintf(page, sizeof(page), "phy = 0x%x, virt = 0x%x\n",
			sensor_fb_cxt->sensor_list[0], sensor_fb_cxt->sensor_list[1]);
	len = simple_read_from_buffer(buf, count, off, page, strlen(page));
	pr_info("phy = 0x%x, virt = 0x%x, len=%d \n", sensor_fb_cxt->sensor_list[0],
		sensor_fb_cxt->sensor_list[1],
		len);
	return len;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static const struct proc_ops sensor_list_fops = {
	.proc_read = sensor_list_read_proc,
	.proc_lseek = default_llseek,
};
#else
static struct file_operations sensor_list_fops = {
	.owner = THIS_MODULE,
	.read = sensor_list_read_proc,
};
#endif

//#ifdef CONFIG_FB
#if defined(CONFIG_DRM_MSM)
static int sensor_fb_notifier(struct notifier_block *nb,
	unsigned long event, void *data)
{
	int blank;
	struct msm_drm_notifier *evdata = data;
	struct sensor_fb_cxt *sns_cxt = container_of(nb, struct sensor_fb_cxt, fb_notif);

	if (!evdata || (evdata->id != 0)) {
		return 0;
	}

	//if(event == MSM_DRM_EARLY_EVENT_BLANK || event == MSM_DRM_EVENT_BLANK)
	if (event == MSM_DRM_EARLY_EVENT_BLANK) {
		blank = *(int *)(evdata->data);

		if (blank == MSM_DRM_BLANK_UNBLANK) {
			spin_lock(&sns_cxt->rw_lock);
			sns_cxt->node_type = 2; /*sleep ratio type resume*/
			spin_unlock(&sns_cxt->rw_lock);
			set_bit(THREAD_WAKEUP, (unsigned long *)&sns_cxt->wakeup_flag);

			/*wake_up_interruptible(&sensor_fb_cxt->wq);*/
			wake_up(&sns_cxt->wq);
			pr_info("%s: sensor_fb_notifier resume \n", __func__);
		} else if (blank == MSM_DRM_BLANK_POWERDOWN) {
			spin_lock(&sns_cxt->rw_lock);
			sns_cxt->node_type = 3; /*sleep ratio type suspend*/
			spin_unlock(&sns_cxt->rw_lock);

			set_bit(THREAD_WAKEUP, (unsigned long *)&sns_cxt->wakeup_flag);
			/*wake_up_interruptible(&sensor_fb_cxt->wq);*/
			wake_up(&sns_cxt->wq);
			pr_info("%s: sensor_fb_notifier suspend \n", __func__);
		} else {
			pr_info("%s: receives wrong data EARLY_BLANK:%d\n", __func__, blank);
		}
	}
	return 0;
}
#elif defined(CONFIG_FB)
static int sensor_fb_notifier(struct notifier_block *nb,
	unsigned long event, void *data)
{
	int blank;
	struct fb_event *evdata = data;
	struct sensor_fb_cxt *sns_cxt = container_of(nb, struct sensor_fb_cxt, fb_notif);
	struct timespec now_time;

	if (evdata && evdata->data) {
		//if(event == FB_EARLY_EVENT_BLANK || event == FB_EVENT_BLANK)
		if (event == FB_EVENT_BLANK) {
			blank = *(int *)evdata->data;

			if (blank == FB_BLANK_UNBLANK) { //resume
				pr_info("%s: sensor_fb_notifier resume \n", __func__);
			} else if (blank == FB_BLANK_POWERDOWN) { //suspend
				pr_info("%s: sensor_fb_notifier suspend \n", __func__);
			} else {
				pr_info("%s: receives wrong data EARLY_BLANK:%d\n", __func__, blank);
			}
		}
	}
	return 0;
}
#elif IS_ENABLED(CONFIG_OPLUS_SENSOR_DRM_PANEL_NOTIFY)
void ssc_fb_set_screen_status(int status)
{
	static int screen_status = SCREEN_INIT;
	struct sensor_fb_cxt *sns_cxt = g_sensor_fb_cxt;

	if (status == SCREEN_ON && screen_status != SCREEN_ON) {
		screen_status = SCREEN_ON;

		spin_lock(&sns_cxt->rw_lock);
		sns_cxt->node_type = 2; /*sleep ratio type resume*/
		spin_unlock(&sns_cxt->rw_lock);
		set_bit(THREAD_WAKEUP, (unsigned long *)&sns_cxt->wakeup_flag);

		/*wake_up_interruptible(&sensor_fb_cxt->wq);*/
		wake_up(&sns_cxt->wq);
		pr_info("%s: sensor_fb_notifier resume \n", __func__);
	} else if (status == SCREEN_OFF && screen_status != SCREEN_OFF) {
		screen_status = SCREEN_OFF;

		spin_lock(&sns_cxt->rw_lock);
		sns_cxt->node_type = 3; /*sleep ratio type suspend*/
		spin_unlock(&sns_cxt->rw_lock);

		set_bit(THREAD_WAKEUP, (unsigned long *)&sns_cxt->wakeup_flag);
		/*wake_up_interruptible(&sensor_fb_cxt->wq);*/
		wake_up(&sns_cxt->wq);
		pr_info("%s: sensor_fb_notifier suspend \n", __func__);
	}
	return;
}
EXPORT_SYMBOL(ssc_fb_set_screen_status);
#endif /* CONFIG_DRM_MSM */
//#endif /* CONFIG_FB */


static int create_sensor_node(struct sensor_fb_cxt *sensor_fb_cxt) {
	int err = 0;
	struct proc_dir_entry *pentry = NULL;

	err = sysfs_create_group(&sensor_fb_cxt->sensor_fb_dev->dev.kobj,
			&sensor_feedback_attribute_group);
	if (err < 0) {
		pr_err("unable to create sensor_feedback_attribute_group file err=%d\n", err);
		goto sysfs_create_failed;
	}
	kobject_uevent(&sensor_fb_cxt->sensor_fb_dev->dev.kobj, KOBJ_ADD);

	sensor_fb_cxt->proc_sns =  proc_mkdir("sns_debug", NULL);
	if (!sensor_fb_cxt->proc_sns) {
		pr_err("can't create sns_debug proc\n");
		err = -EFAULT;
		goto sysfs_create_failed;
	}

	pentry = proc_create_data("sensor_list", 0666, sensor_fb_cxt->proc_sns,
			&sensor_list_fops, sensor_fb_cxt);
	if (!pentry) {
		pr_err("create sensor_list proc failed.\n");
		err = -EFAULT;
		goto sysfs_create_failed;
	}

	return 0;
sysfs_create_failed:
	sysfs_remove_group(&sensor_fb_cxt->sensor_fb_dev->dev.kobj,
						&sensor_feedback_attribute_group);
	return err;
}

static int sensor_sleep_ratio_init(struct sensor_fb_cxt *sensor_fb_cxt) {
	int ret = 0;
	int err = 0;

	pr_err("sensor_sleep_ratio_init,err=%d\n", err);
	subsystem_desc_init(sensor_fb_cxt->subsystem_desc);
	reset_subsys_sleep_stats();
#if defined(CONFIG_DRM_MSM)
	sensor_fb_cxt->fb_notif.notifier_call = sensor_fb_notifier;
	err = msm_drm_register_client(&sensor_fb_cxt->fb_notif);
	if (err) {
		pr_err("Unable to register fb_notifier: %d\n", err);
	}
#elif defined(CONFIG_FB)
	sensor_fb_cxt->fb_notif.notifier_call = sensor_fb_notifier;
	err = fb_register_client(&sensor_fb_cxt->fb_notif);
	if (err) {
		pr_err("Unable to register fb_notifier: %d\n", err);
	}
#endif/*CONFIG_FB*/
	return ret;
}

static int sensor_feedback_probe(struct platform_device *pdev)
{
	int err = 0;
	struct sensor_fb_cxt *sensor_fb_cxt = NULL;

	sensor_fb_cxt = kzalloc(sizeof(struct sensor_fb_cxt), GFP_KERNEL);
	if (sensor_fb_cxt == NULL) {
		pr_err("kzalloc g_sensor_fb_cxt failed\n");
		err = -ENOMEM;
		goto alloc_sensor_fb_failed;
	}

	/*sensor_fb_cxt init*/
	sensor_fb_cxt->sensor_fb_dev = pdev;
	g_sensor_fb_cxt = sensor_fb_cxt;
	spin_lock_init(&sensor_fb_cxt->rw_lock);
	init_waitqueue_head(&sensor_fb_cxt->wq);
	set_bit(THREAD_SLEEP, (unsigned long *)&sensor_fb_cxt->wakeup_flag);
	platform_set_drvdata(pdev, sensor_fb_cxt);

	err = create_sensor_node(sensor_fb_cxt);
	if(err != 0) {
		pr_info("create_sensor_node failed\n");
		goto create_sensor_node_failed;

	}

	err = sensor_sleep_ratio_init(sensor_fb_cxt);
	if(err) {
		pr_info("sensor_sleep_ratio_probe failed\n");
		goto sleep_ratio_init_failed;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
	err = reserve_fb_share_mem();
	if(err) {
		pr_info("reserve_fb_share_mem failed\n");
		goto reserve_fb_share_mem_failed;
	}
#endif

	/*create sensor_feedback_task thread*/
	sensor_fb_cxt->report_task = kthread_create(sensor_report_thread,
			(void *)sensor_fb_cxt,
			"sensor_feedback_task");
	if (IS_ERR(sensor_fb_cxt->report_task)) {
		pr_info("kthread_create failed\n");
		err = PTR_ERR(sensor_fb_cxt->report_task);
		goto create_task_failed;
	}

	/*wake up thread of report_task*/
	wake_up_process(sensor_fb_cxt->report_task);
	pr_info("sensor_feedback_init success\n");

	return 0;
create_task_failed:
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
reserve_fb_share_mem_failed:
#endif
sleep_ratio_init_failed:
create_sensor_node_failed:
	kfree(sensor_fb_cxt);
	g_sensor_fb_cxt = NULL;
alloc_sensor_fb_failed:
	return err;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void sensor_feedback_remove(struct platform_device *pdev)
#else /* KERNEL_VERSION(6, 1, 0) */
static int sensor_feedback_remove(struct platform_device *pdev)
#endif /* KERNEL_VERSION(6, 1, 0) */
{
	struct sensor_fb_cxt *sensor_fb_cxt = g_sensor_fb_cxt;
	sysfs_remove_group(&sensor_fb_cxt->sensor_fb_dev->dev.kobj,
		&sensor_feedback_attribute_group);
	kfree(sensor_fb_cxt);
	g_sensor_fb_cxt = NULL;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return 0;
#endif /* KERNEL_VERSION(6, 1, 0) */
}

static const struct of_device_id of_drv_match[] = {
	{ .compatible = "oplus,sensor-feedback"},
	{},
};
MODULE_DEVICE_TABLE(of, of_drv_match);

static struct platform_driver _driver = {
	.probe      = sensor_feedback_probe,
	.remove     = sensor_feedback_remove,
	.driver     = {
		.name       = "sensor_feedback",
		.of_match_table = of_drv_match,
	},
};

static int __init sensor_feedback_init(void)
{
	pr_info("sensor_feedback_init call\n");

	platform_driver_register(&_driver);
	return 0;
}

/*
static int __exit sensor_feedback_exit(void)
{
	pr_info("sensor_feedback_exit call\n");

	platform_driver_unregister(&_driver);
	return 0;
}*/


core_initcall(sensor_feedback_init);

//module_init(sensor_feedback_init);
//module_exit(sensor_feedback_exit);


MODULE_AUTHOR("JangHua.Tang");
MODULE_LICENSE("GPL v2");

