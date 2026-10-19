// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/task_work.h>
#include <linux/thermal.h>
#include <linux/rtc.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/of_gpio.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/input/mt.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/version.h>
#include <linux/iio/consumer.h>
#include <linux/alarmtimer.h>
#include <linux/sched/debug.h> /* for sched_show_task */

#include "touchpanel_common.h"
#include <linux/pinctrl/consumer.h>
#include "touchpanel_proc.h"
#include "touch_comon_api/touch_comon_api.h"
#include "touchpanel_prevention/touchpanel_prevention.h"
#include "touchpanel_healthinfo/touchpanel_healthinfo.h"
#include "touchpanel_healthinfo/touchpanel_exception.h"
#include "touchpanel_tui_support/touchpanel_tui_support.h"
#include "touch_pen/touch_pen_core.h"

#ifndef CONFIG_REMOVE_OPLUS_FUNCTION
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
#include<mt-plat/mtk_boot_common.h>
#else
#include <soc/oplus/system/boot_mode.h>
#endif
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
#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
#include <linux/mtk_panel_ext.h>
#include <linux/mtk_disp_notify.h>
#endif

#if IS_ENABLED(CONFIG_TOUCHPANEL_NOTIFY)
#include "touchpanel_notify/touchpanel_event_notify.h"
#endif
#define MAX_NODE_DATA_LENGTH         60

/*******Part0:LOG TAG Declear************************/
#if defined(CONFIG_TOUCHPANEL_MTK_PLATFORM) && defined(CONFIG_TOUCHIRQ_UPDATE_QOS)
#error CONFIG_TOUCHPANEL_MTK_PLATFORM and CONFIG_TOUCHIRQ_UPDATE_QOS
#error can not defined same time
#endif

#define TP_ALL_GESTURE_SUPPORT \
	(ts->black_gesture_support || ts->fingerprint_underscreen_support || ts->fp_grip_support)
#define TP_ALL_GESTURE_ENABLE  \
	((ts->gesture_enable & 0x01) == 1 || ts->fp_enable || (ts->fp_grip_enable & FP_GRIP_ENABLE))

/******* many panel index match start *******/
#if IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
#define GET_INDEX(cur_tp_index) \
((match_panel_index[cur_tp_index].c_panel_index == (cur_tp_index)) \
	? match_panel_index[cur_tp_index].match_index \
	: PANEL_EVENT_NOTIFICATION_PRIMARY)

#define GET_TOUCH(cur_tp_index) \
((match_panel_index[cur_tp_index].c_panel_index == (cur_tp_index)) \
	? match_panel_index[cur_tp_index].match_touch \
	: PANEL_EVENT_NOTIFIER_CLIENT_PRIMARY_TOUCH)

typedef enum panel_index {
	PRIMARY   = 0,
	SECONDARY = 1,
	PANEL_MAX = 2
} p_index;

struct match_panel {
	p_index c_panel_index;
	int match_index;
	int match_touch;
};

struct match_panel match_panel_index[] = {
	{PRIMARY,   PANEL_EVENT_NOTIFICATION_PRIMARY,   PANEL_EVENT_NOTIFIER_CLIENT_PRIMARY_TOUCH},
	{SECONDARY, PANEL_EVENT_NOTIFICATION_SECONDARY, PANEL_EVENT_NOTIFIER_CLIENT_SECONDARY_TOUCH},
	{PANEL_MAX, PANEL_EVENT_NOTIFICATION_MAX,       PANEL_EVENT_NOTIFIER_CLIENT_ECM},
};
#endif
/******* many panel index match end *******/


/*******Part1:Global variables Area********************/
struct touchpanel_data *g_tp[TP_SUPPORT_MAX] = {NULL};
static DEFINE_MUTEX(tp_core_lock);
int cur_tp_index = 0;
EXPORT_SYMBOL(cur_tp_index);
/*******Part2:declear Area********************************/
static void speedup_resume(struct work_struct *work);
static void lcd_trigger_load_tp_fw(struct work_struct *work);
void esd_handle_switch(struct esd_information *esd_info, bool flag);
void tp_delta_read_triggered_by_key(int index);

#if IS_ENABLED(CONFIG_FB) || IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data);
#elif IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
static void ts_panel_notifier_callback(enum panel_event_notifier_tag tag,
		 struct panel_event_notification *event, void *client_data);
#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
static int ts_mtk_drm_notifier_callback(struct notifier_block *nb,
                unsigned long event, void *data);
#endif

static void tp_touch_release(struct touchpanel_data *ts);
static void tp_btnkey_release(struct touchpanel_data *ts);
static inline void tp_work_func(struct touchpanel_data *ts);
static void lcd_tp_refresh_work(struct work_struct *work);
#if IS_ENABLED(CONFIG_FB) || \
	IS_ENABLED(CONFIG_DRM_MSM) || \
	IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY) || \
	IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER) || \
	IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
static void tp_ftm_extra(unsigned int tp_index);
static int tp_control_reset_gpio(bool enable, unsigned int tp_index);
static int tp_control_irq_state(bool enable, unsigned int tp_index);
static void lcd_tp_load_fw(unsigned int tp_index);
static void lcd_tp_refresh_switch(unsigned int tp_index, int fps);
static void tp_suspend_work(struct work_struct *work);
#endif/*CONFIG_FB*/

static void tp_rate_calc(struct touchpanel_data *ts, tp_rate tp_rate_type);
static int tp_control_cs_gpio(bool enable, unsigned int tp_index);

extern int preconfig_power_control(struct touchpanel_data *ts);
extern  int reconfig_power_control(struct touchpanel_data *ts);

#if IS_ENABLED(CONFIG_TOUCHPANEL_NOTIFY)
static int tp_gesture_enable_flag(unsigned int tp_index);
extern int (*tp_gesture_enable_notifier)(unsigned int tp_index);
extern int (*tp_cs_gpio_notifier)(bool enable, unsigned int tp_index);
#endif

#ifndef CONFIG_REMOVE_OPLUS_FUNCTION
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
#ifndef CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY
extern enum boot_mode_t get_boot_mode(void);
#endif
#else
extern int get_boot_mode(void);
#endif
#endif

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
#ifndef CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY
extern void primary_display_esd_check_enable(int enable);
#endif
#endif
void display_esd_check_enable_bytouchpanel(bool enable);

/*******Part3:Function  Area********************************/
struct touchpanel_data *get_ts_data(unsigned int tp_index)
{
	struct touchpanel_data *ts = NULL;

	mutex_lock(&tp_core_lock);
	if (tp_index >= TP_SUPPORT_MAX) {
		TP_INFO(tp_index, "tp_index:%u, is beyond %d,\n", tp_index, TP_SUPPORT_MAX);
		ts = g_tp[0];
	} else {
		ts = g_tp[tp_index];
	}
	mutex_unlock(&tp_core_lock);
	return ts;
}
EXPORT_SYMBOL(get_ts_data);

bool inline is_ftm_boot_mode(struct touchpanel_data *ts)
{
#ifndef CONFIG_REMOVE_OPLUS_FUNCTION
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM

	if ((ts->boot_mode == META_BOOT || ts->boot_mode == FACTORY_BOOT))
#else
	if ((ts->boot_mode == MSM_BOOT_MODE__FACTORY
	     || ts->boot_mode == MSM_BOOT_MODE__RF || ts->boot_mode == MSM_BOOT_MODE__WLAN))
#endif
	{
		return true;
	}

#endif
	return false;
}

/**
 * operate_mode_switch - switch work mode based on current params
 * @ts: touchpanel_data struct using for common driver
 *
 * switch work mode based on current params(gesture_enable, limit_enable, glove_enable)
 * Do not care the result: Return void type
 */

int mode_switch_health(struct touchpanel_data *ts, work_mode mode, int flag)
{
	int ret = 0;

	ret = ts->ts_ops->mode_switch(ts->chip_data, mode, flag);

	if (ts->health_monitor_support && (ret < 0
		   || CHK_BIT_NUM(ts->monitor_data.health_simulate_trigger, HEALTH_SIMULATE_BIT_MODE_SWITCH))) {
		tp_healthinfo_report(&ts->monitor_data, HEALTH_REPORT,
			   (MODE_NORMAL == mode) ? "mode_normal_switch_fail" :
			   (MODE_SLEEP == mode) ? "mode_sleep_switch_fail" :
			   (MODE_EDGE == mode) ? "mode_edge_switch_fail" :
			   (MODE_GESTURE == mode) ? "mode_gesture_switch_fail" :
			   (MODE_GLOVE == mode) ? "mode_glove_mode_fail" :
			   (MODE_RAINSTORM == mode) ? "mode_rainstorm_mode_fail" :
			   (MODE_LEATHER_COVER == mode) ? "mode_leather_cover_mode_fail" :
			   (MODE_CHARGE == mode) ? "mode_charge_switch_fail" :
			   (MODE_GAME == mode) ? "mode_game_switch_fail" :
			   (MODE_PALM_REJECTION == mode) ? "mode_palm_switch_fail" :
			   (MODE_FACE_DETECT == mode) ? "mode_fd_switch_fail" :
			   (MODE_HEADSET == mode) ? "mode_headset_switch_fail" :
			   (MODE_WIRELESS_CHARGE == mode) ? "mode_wireless_charge_switch_fail" :
			   (MODE_PEN_SCAN == mode) ? "mode_pen_scan_switch_fail" :
			   (MODE_INCELL_AOD == mode) ? "mode_incell_aod_switch_fail" :
			   (MODE_PEN_CTL == mode) ? "mode_pen_ctl_switch_fail" : "mode_others_switch_fail");
	}

	return ret;
}

void operate_mode_switch(struct touchpanel_data *ts)
{
	if (!ts->ts_ops->mode_switch) {
		TP_INFO(ts->tp_index, "not support ts_ops->mode_switch callback\n");
		return;
	}
	touchpanel_trusted_touch_completion(ts);

	if (ts->is_suspended) {
		if (TP_ALL_GESTURE_SUPPORT && (!ts->hall_status)) {
			if (TP_ALL_GESTURE_ENABLE) {
				mode_switch_health(ts, MODE_GESTURE, ts->gesture_enable
							|| ts->fp_enable);
				mode_switch_health(ts, MODE_NORMAL, true);

				if (ts->fingerprint_underscreen_support) {
					ts->ts_ops->enable_fingerprint(ts->chip_data, !!ts->fp_enable);
				}

				if (((ts->gesture_enable & 0x01) != 1) && ts->ts_ops->enable_gesture_mask) {
					ts->ts_ops->enable_gesture_mask(ts->chip_data, 0);
				}

			} else {
				mode_switch_health(ts, MODE_GESTURE, false);
				mode_switch_health(ts, MODE_SLEEP, true);
			}

		} else {
			mode_switch_health(ts, MODE_SLEEP, true);
		}

	} else {
		if (ts->face_detect_support) {
			if (ts->fd_enable) {
				input_event(ts->ps_input_dev, EV_MSC, MSC_RAW, 0);
				input_sync(ts->ps_input_dev);
			}

			mode_switch_health(ts, MODE_FACE_DETECT, ts->fd_enable == 1);
		}

		if (ts->fingerprint_underscreen_support) {
			ts->ts_ops->enable_fingerprint(ts->chip_data, !!ts->fp_enable);
		}

		if (ts->black_gesture_support) {
			if (ts->incell_aod_gesture_support) {
				TP_INFO(ts->tp_index, "%s : incell_aod_flag = %d", __func__, ts->incell_aod_flag);
				if (ts->incell_aod_flag) {
					TP_INFO(ts->tp_index, "TP in mode aod start\n");
					ts->incell_aod_flag = false;
					mode_switch_health(ts,  MODE_INCELL_AOD, true);
					ts->is_suspended = 1;
				} else {
					TP_INFO(ts->tp_index, "TP out mode aod start\n");
					mode_switch_health(ts,  MODE_INCELL_AOD, false);
				}
			} else {
				mode_switch_health(ts, MODE_GESTURE, false);
			}
		}

		if (ts->fw_edge_limit_support) {
			mode_switch_health(ts, MODE_EDGE, ts->limit_enable);
		}

		if (ts->glove_mode_v2_support) {
			mode_switch_health(ts, MODE_GLOVE, ts->glove_enable && (!ts->pocket_prevent_mode));
		}

		if (ts->rainstorm_mode_v2_support) {
			mode_switch_health(ts, MODE_RAINSTORM, ts->rainstorm_enable);
		}

		if (ts->glove_mode_support || ts->leather_cover_mode_support) {
			mode_switch_health(ts, MODE_LEATHER_COVER, ts->glove_enable);
		}

		if (ts->charger_pump_support) {
			mode_switch_health(ts, MODE_CHARGE, ts->is_usb_checked);
		}

		if (ts->wireless_charger_support) {
			mode_switch_health(ts, MODE_WIRELESS_CHARGE,
						ts->is_wireless_checked);
		}

		if (ts->headset_pump_support) {
			mode_switch_health(ts, MODE_HEADSET, ts->is_headset_checked);
		}

		if (ts->kernel_grip_support && ts->ts_ops->enable_kernel_grip) {
			ts->ts_ops->enable_kernel_grip(ts->chip_data, ts->grip_info);
		}

		if(ts->ts_ops->set_high_frame_rate && ts->high_frame_rate_support) {
			ts->ts_ops->set_high_frame_rate(ts->chip_data, ts->high_frame_value, ts->high_frame_rate_time);
		}

		if(ts->ts_ops->mode_switch) {
			ts->ts_ops->mode_switch(ts->chip_data, MODE_GAME, ts->noise_level > 0);
		}

		if (ts->smooth_level_array_support && ts->ts_ops->smooth_lv_set) {
			ts->ts_ops->smooth_lv_set(ts->chip_data, ts->smooth_level_used_array[ts->smooth_level_chosen]);
		}

		if (ts->sensitive_level_array_support && ts->ts_ops->sensitive_lv_set) {
			ts->ts_ops->sensitive_lv_set(ts->chip_data, ts->sensitive_level_used_array[ts->sensitive_level_chosen]);
		}
		if (ts->click_sensitive_level_array_support &&  ts->ts_ops->click_sensitive_lv_set) {
			ts->ts_ops->click_sensitive_lv_set(ts->chip_data, ts->click_sensitive_level_used_array[ts->click_sensitive_level_chosen]);
		}
		if (ts->diaphragm_touch_support && ts->ts_ops->diaphragm_touch_lv_set) {
			ts->ts_ops->diaphragm_touch_lv_set(ts->chip_data, ts->diaphragm_touch_level_chosen);
		}
		if (ts->lcd_tp_refresh_support && ts->ts_ops->tp_refresh_switch) {
			ts->ts_ops->tp_refresh_switch(ts->chip_data, ts->lcd_fps);
		}

		if (ts->pen_support) {
			mode_switch_health(ts, MODE_PEN_SCAN, ts->is_pen_connected);
		}

		if (ts->waterproof_support) {
			mode_switch_health(ts, MODE_WATERPROOF, ts->waterproof & ~(0x1 << WATERPROOF_RUS_BIT));
		}

		if (ts->tp_scene_para_switch_support && ts->ts_ops->pen_sensitive_lv_set) {
			ts->ts_ops->pen_sensitive_lv_set(ts->chip_data, ts->scene_info.pen_sensitive_level);
		}

		mode_switch_health(ts, MODE_NORMAL, true);
	}
}
EXPORT_SYMBOL(operate_mode_switch);

/*
 * check_usb_state----expose to be called by charger int to get usb state
 * @usb_state : 1 if usb checked, otherwise is 0
*/
void switch_usb_state_work(struct work_struct *work)
{
	struct touchpanel_data *ts = container_of(work, struct touchpanel_data,
				     charger_pump_work);

	mutex_lock(&ts->mutex);

	if (ts->charger_pump_support && (ts->is_usb_checked != ts->cur_usb_state)) {
		ts->is_usb_checked = !!ts->cur_usb_state;
		TP_INFO(ts->tp_index, "%s: check usb state : %d, is_suspended: %d\n", __func__,
			ts->cur_usb_state, ts->is_suspended);

		if (!ts->is_suspended && (ts->suspend_state == TP_SPEEDUP_RESUME_COMPLETE)
		    && !ts->loading_fw) {
			mode_switch_health(ts, MODE_CHARGE, ts->is_usb_checked);

			if (ts->smooth_level_array_support && ts->ts_ops->smooth_lv_set) {
				if (ts->is_usb_checked) {
					ts->smooth_level_used_array = (u32 *)&(ts->smooth_level_charging_array);
				} else {
					ts->smooth_level_used_array = (u32 *)&(ts->smooth_level_array);
				}

				ts->ts_ops->smooth_lv_set(ts->chip_data, ts->smooth_level_used_array[ts->smooth_level_chosen]);
			}

			if (ts->sensitive_level_array_support && ts->ts_ops->sensitive_lv_set) {
				if (ts->is_usb_checked) {
					ts->sensitive_level_used_array = (u32 *)&(ts->sensitive_level_charging_array);
				} else {
					ts->sensitive_level_used_array = (u32 *)&(ts->sensitive_level_array);
				}
				ts->ts_ops->sensitive_lv_set(ts->chip_data, ts->sensitive_level_used_array[ts->sensitive_level_chosen]);
			}
		}
	}

	mutex_unlock(&ts->mutex);
}

/*
 * check_headset_state----expose to be called by audio int to get headset state
 * @headset_state : 1 if headset checked, otherwise is 0
 */
void switch_headset_work(struct work_struct *work)
{
	struct touchpanel_data *ts = container_of(work, struct touchpanel_data,
				     headset_pump_work);

	mutex_lock(&ts->mutex);

	if (ts->headset_pump_support
	    && (ts->is_headset_checked != ts->cur_headset_state)) {
		ts->is_headset_checked = !!ts->cur_headset_state;
		TP_INFO(ts->tp_index, "%s: check headset state : %d, is_suspended: %d\n",
			__func__, ts->cur_headset_state, ts->is_suspended);

		if (!ts->is_suspended && (ts->suspend_state == TP_SPEEDUP_RESUME_COMPLETE)
		    && !ts->loading_fw) {
			mode_switch_health(ts, MODE_HEADSET, ts->is_headset_checked);
		}
	}

	mutex_unlock(&ts->mutex);
}

int long_strip_abnormal_detect(struct touchpanel_data *ts, struct point_info *points, int obj_attention)
{
	int i = 0;
	int ret = 0;
	u16 channels_max_thd = ts->long_strip_abnormal_detect.channels_max_thd;
	u16 er_max = ts->long_strip_abnormal_detect.er_max;
	u16 er_min = ts->long_strip_abnormal_detect.er_min;
	u16 x_min = (ts->resolution_info.max_x - ts->long_strip_abnormal_detect.center_width) / 2;
	u16 x_max = (ts->resolution_info.max_x + ts->long_strip_abnormal_detect.center_width) / 2;
	u16 y_min = (ts->resolution_info.max_y - ts->long_strip_abnormal_detect.center_width) / 2;
	u16 y_max = (ts->resolution_info.max_y + ts->long_strip_abnormal_detect.center_width) / 2;

	for (i = 0; i < ts->max_num; i++) {
		if ((((obj_attention & TOUCH_BIT_CHECK) >> i) & 0x01) && (points[i].status != 0)) {
			if (points[i].rx_press <= channels_max_thd && points[i].rx_press != 0 \
				&& points[i].rx_er >= er_min && points[i].rx_er <= er_max \
				&& points[i].x >= x_min && points[i].x <= x_max \
				&& points[i].tx_press == 0 && points[i].tx_er == 0) {
				ret = 1;
				if (ts->health_monitor_support) {
					tp_healthinfo_report(&ts->monitor_data, HEALTH_REPORT, "X_Long_Strip_Abnormal_Detect");
				}
				TPD_INFO("x long_strip_abnormal_detect suc, error point[%d] status:[%u,%u][%d,%d,%d,%d].\n", i, \
					points[i].x, points[i].y, points[i].rx_press, points[i].tx_press, points[i].rx_er, points[i].tx_er);
			}
			if (points[i].tx_press <= channels_max_thd && points[i].tx_press != 0 \
				&& points[i].tx_er >= er_min && points[i].tx_er <= er_max \
				&& points[i].y >= y_min && points[i].y <= y_max \
				&& points[i].rx_press == 0 && points[i].rx_er == 0) {
				ret = 1;
				if (ts->health_monitor_support) {
					tp_healthinfo_report(&ts->monitor_data, HEALTH_REPORT, "Y_Long_Strip_Abnormal_Detect");
				}
				TPD_INFO("y long_strip_abnormal_detect suc, error point[%d] status:[%u,%u][%d,%d,%d,%d].\n", i, \
					points[i].x, points[i].y, points[i].rx_press, points[i].tx_press, points[i].rx_er, points[i].tx_er);
			}
		}
	}
	return ret;
}

int touch_points_detect_algo(struct touchpanel_data *ts, struct point_info *points, int obj_attention)
{
	int retval = 0;
	if (!ts || !points) {
		TPD_INFO("ts or points is null.\n");
		return 0;
	}
	if (ts->long_strip_abnormal_detect_support) {
		retval = long_strip_abnormal_detect(ts, points, obj_attention);
	}
	return retval;
}

void touch_call_fp_grip(struct touchpanel_data *ts, int state)
{
	struct touch_fp_grip_info event_data;
	if (!ts) {
		return;
	}
	event_data.value = state;
	touchpanel_event_call_notifier(EVENT_ACTION_FOR_FP_GIRP, (void *)&event_data);
	TP_INFO(ts->tp_index, "transfer girp of fp pass state:%d\n", event_data.value);
}

static void touch_call_notifier_fp(struct touchpanel_data *ts, struct fp_underscreen_info *fp_info)
{
#if IS_ENABLED(CONFIG_TOUCHPANEL_NOTIFY)
	struct touchpanel_event event_data;
	ktime_t timer;
	ktime_t delta_time;

	if (!ts) {
		return;
	}

	timer = ktime_get();

	memset(&event_data, 0, sizeof(struct touchpanel_event));

	event_data.touch_state = fp_info->touch_state;
	event_data.area_rate = fp_info->area_rate;
	event_data.x = fp_info->x;
	event_data.y = fp_info->y;

	touchpanel_event_call_notifier(EVENT_ACTION_FOR_FINGPRINT,
				       (void *)&event_data);

	if (ts->health_monitor_support) {
		if (event_data.touch_state == 1) {
			if (!ts->fp_down_cnt) {
				ts->fp_down_time = timer;
			}
			ts->fp_down_cnt++;
			ts->fp_up_cnt = 0;
		} else if (event_data.touch_state == 0) {
			if (!ts->fp_up_cnt) {
				ts->fp_up_time = timer;
				delta_time = ktime_to_us(ts->fp_up_time) - ktime_to_us(ts->fp_down_time);
				if (delta_time > 0 && delta_time <= FP_EVENT_COST_TIME_OVER_10MS) {
					tp_healthinfo_report(&ts->monitor_data, HEALTH_REPORT, "fp_event_cost_time_over_10ms_cnt");
				} else if (delta_time > FP_EVENT_COST_TIME_OVER_10MS && delta_time <= FP_EVENT_COST_TIME_OVER_18MS) {
					tp_healthinfo_report(&ts->monitor_data, HEALTH_REPORT, "fp_event_cost_time_over_18ms_cnt");
				} else if (delta_time > FP_EVENT_COST_TIME_OVER_18MS && delta_time <= FP_EVENT_COST_TIME_OVER_26MS) {
					tp_healthinfo_report(&ts->monitor_data, HEALTH_REPORT, "fp_event_cost_time_over_26ms_cnt");
				}
			}
			ts->fp_up_cnt++;
			ts->fp_down_cnt = 0;
		}
	}
#else
	opticalfp_irq_handler(fp_info);
#endif
}

static inline void tp_touch_down(struct touchpanel_data *ts, struct point_info points, int touch_report_num, int id)
{
	int cost_time = 0;
	if (ts->input_dev == NULL) {
		return;
	}

	input_report_key(ts->input_dev, BTN_TOUCH, 1);
	input_report_key(ts->input_dev, BTN_TOOL_FINGER, 1);

	if (touch_report_num == 1) {
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, points.width_major);
		ts->last_width_major = points.width_major;
		ts->last_touch_major = points.touch_major;

	} else if (!(touch_report_num & 0x7f) || touch_report_num == 30) {
		/*if touch_report_num == 127, every 127 points, change width_major*/
		/*down and keep long time, auto repeat per 5 seconds, for weixing*/
		/*report move event after down event, for weixing voice delay problem, 30 -> 300ms in order to avoid the intercept by shortcut*/
		if (ts->last_width_major == points.width_major) {
			ts->last_width_major = points.width_major + 1;

		} else {
			ts->last_width_major = points.width_major;
		}

		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, ts->last_width_major);
	}

	if (ts->major_rate_limit_support && !!(ts->noise_level) && ts->tp_ic_touch_num == ts->last_tp_ic_touch_num && \
					points.x == ts->last_x_y_point[id].x && points.y == ts->last_x_y_point[id].y) {
		if (!(touch_report_num & ts->major_rate_limit_times)) {
			ts->last_touch_major = points.touch_major;
		}
	} else {
		ts->last_touch_major = points.touch_major;
	}

	if ((points.x > ts->touch_major_limit.width_range)
	    && (points.x < ts->resolution_info.max_x - ts->touch_major_limit.width_range)
	    && \
	    (points.y > ts->touch_major_limit.height_range)
	    && (points.y < ts->resolution_info.max_y -
		ts->touch_major_limit.height_range)) {
		/*smart_gesture_support*/
		if (ts->last_touch_major > SMART_GESTURE_THRESHOLD) {
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, ts->last_touch_major);
		} else {
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, SMART_GESTURE_LOW_VALUE);
		}

		/*pressure_report_support*/
		if (ts->touchz_to_pressure_support) {
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
					points.z);
		} else {
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
					ts->last_touch_major);   /*add for fixing gripview tap no function issue*/
		}
	}

	if (!CHK_BIT(ts->irq_slot, (1 << id))) {
		TP_INFO(ts->tp_index, "first touch point id %d [%4d %4d %4d %4d %4d %4d %4d]\n", id, points.x, points.y, points.z,
					points.rx_press, points.tx_press, points.rx_er, points.tx_er);
		if(id == 0 && tp_debug == 1 && ts->monitor_data.RATE_MIN) {
			tp_rate_calc(ts, TP_RATE_CLEAR);
		}
	}

	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, points.x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, points.y);
	if(id == 0 && tp_debug == 1 && ts->monitor_data.RATE_MIN) {
		tp_rate_calc(ts, TP_RATE_CALC);
	}
	if (ts->last_x_y_point[id].x != points.x || ts->last_x_y_point[id].y != points.y) {
		cost_time = ktime_to_us(ktime_get()) - ktime_to_us(ts->monitor_data.irq_to_report_timer);
		input_report_abs(ts->input_dev, ABS_TOUCH_COST_TIME_KERNEL, (cost_time < MAX_TOUCH_COST_TIME) ? cost_time : MAX_TOUCH_COST_TIME);
	}
	ts->last_x_y_point[id].x = points.x;
	ts->last_x_y_point[id].y = points.y;

	if (id == 0 && ts->report_rate_testing) {
		ts->touch_major_sum += points.touch_major;
		ts->get_frame_num++;
	}

	TP_SPECIFIC_PRINT(ts->tp_index, ts->point_num, "Touchpanel id %d :Down[%4d %4d %4d %4d %4d %4d %4d] %d\n", id, points.x, points.y, points.z,
						points.rx_press, points.tx_press, points.rx_er, points.tx_er, cost_time);
}

static inline void tp_touch_up(struct touchpanel_data *ts)
{
	if (ts->input_dev == NULL) {
		return;
	}

	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
	ts->pen_mode_tp_state = DEFAULT;				/*need reset tp state, IC may report times palm, but common driver just report and clear once */
}

void notify_pen_state(int state, unsigned int index)
{
	struct touchpanel_data *ts = get_ts_data(index);

	if (!ts)
		return;
#ifndef CONFIG_REMOVE_OPLUS_FUNCTION
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
	if ((ts->boot_mode == META_BOOT || ts->boot_mode == FACTORY_BOOT))
#else
	if ((ts->boot_mode == MSM_BOOT_MODE__FACTORY ||
		ts->boot_mode == MSM_BOOT_MODE__RF ||
		ts->boot_mode == MSM_BOOT_MODE__WLAN))
#endif
#endif
	 {
		TP_INFO(ts->tp_index, "Ftm mode, no need to notify pen state\n");
		return;
	}

	if (ts->pen_support && (ts->is_pen_attracted != state)) {
		mutex_lock(&ts->mutex);
		ts->is_pen_attracted = !!state;
		TP_INFO(ts->tp_index, "%s: check pen state0 : %d %d, is_suspended: %d\n",
			__func__,
			ts->is_pen_connected,
			ts->is_pen_attracted,
			ts->is_suspended);

		if (!ts->is_suspended &&
			(ts->suspend_state == TP_SPEEDUP_RESUME_COMPLETE)) {	/* For close pencil scan under screen on mode */
			mode_switch_health(ts, MODE_PEN_SCAN, ts->is_pen_connected);
		} else if (ts->is_suspended &&
				((ts->gesture_enable & 0x01) == 1 ||
					ts->fp_enable) && (!ts->hall_status)) {	/* For close pencil scan under gesture mode */
			mode_switch_health(ts, MODE_PEN_SCAN, \
			(ts->gesture_enable_indep & (1 << PENDETECT)) &&
			ts->is_pen_connected && !ts->is_pen_attracted);
		}
		mutex_unlock(&ts->mutex);
	}
}
EXPORT_SYMBOL(notify_pen_state);

static void tp_exception_handle(struct touchpanel_data *ts)
{
	if (!ts->ts_ops->reset) {
		TP_INFO(ts->tp_index, "not support ts->ts_ops->reset callback\n");
		return;
	}

	ts->ts_ops->reset(
		ts->chip_data);    /* after reset, all registers set to default*/
	operate_mode_switch(ts);

	tp_btnkey_release(ts);
	tp_touch_release(ts);

	if (ts->fingerprint_underscreen_support) {
		ts->fp_info.touch_state = 0;
		touch_call_notifier_fp(ts, &ts->fp_info);
	}
	if (ts->exception_upload_support) {
		TP_INFO(ts->tp_index, "EXCEP_TOUCH_IC_RESET upload\n");
		tp_exception_report(&ts->exception_data, EXCEP_TOUCH_IC_RESET, "fw_status_err", sizeof("fw_status_err"));
	}
}

void tp_fw_auto_reset_handle(struct touchpanel_data *ts)
{
	TP_INFO(ts->tp_index, "%s\n", __func__);

	operate_mode_switch(ts);

	tp_btnkey_release(ts);
	tp_touch_release(ts);
}
EXPORT_SYMBOL(tp_fw_auto_reset_handle);

static void tp_geture_info_transform(struct gesture_info *gesture,
				     struct resolution_info *resolution_info)
{
	gesture->Point_start.x = gesture->Point_start.x *
				 resolution_info->LCD_WIDTH  / (resolution_info->max_x);
	gesture->Point_start.y = gesture->Point_start.y *
				 resolution_info->LCD_HEIGHT / (resolution_info->max_y);
	gesture->Point_end.x   = gesture->Point_end.x   *
				 resolution_info->LCD_WIDTH  / (resolution_info->max_x);
	gesture->Point_end.y   = gesture->Point_end.y   *
				 resolution_info->LCD_HEIGHT / (resolution_info->max_y);
	gesture->Point_1st.x   = gesture->Point_1st.x   *
				 resolution_info->LCD_WIDTH  / (resolution_info->max_x);
	gesture->Point_1st.y   = gesture->Point_1st.y   *
				 resolution_info->LCD_HEIGHT / (resolution_info->max_y);
	gesture->Point_2nd.x   = gesture->Point_2nd.x   *
				 resolution_info->LCD_WIDTH  / (resolution_info->max_x);
	gesture->Point_2nd.y   = gesture->Point_2nd.y   *
				 resolution_info->LCD_HEIGHT / (resolution_info->max_y);
	gesture->Point_3rd.x   = gesture->Point_3rd.x   *
				 resolution_info->LCD_WIDTH  / (resolution_info->max_x);
	gesture->Point_3rd.y   = gesture->Point_3rd.y   *
				 resolution_info->LCD_HEIGHT / (resolution_info->max_y);
	gesture->Point_4th.x   = gesture->Point_4th.x   *
				 resolution_info->LCD_WIDTH  / (resolution_info->max_x);
	gesture->Point_4th.y   = gesture->Point_4th.y   *
				 resolution_info->LCD_HEIGHT / (resolution_info->max_y);
}

static void tp_gesture_handle(struct touchpanel_data *ts)
{
	struct gesture_info gesture_info_temp;
	int retval = 0;
	int index = 0;
	struct touchpanel_data *g_ts = NULL;

	if (((!ts->ts_ops->get_gesture_info) && (!ts->enable_point_auto_change))
	    || ((!ts->ts_ops->get_gesture_info_auto) && ts->enable_point_auto_change)) {
		TP_INFO(ts->tp_index, "not support ts->ts_ops->get_gesture_info callback\n");
		return;
	}

	memset(&gesture_info_temp, 0, sizeof(struct gesture_info));
	if (!ts->enable_point_auto_change) {
		ts->ts_ops->get_gesture_info(ts->chip_data, &gesture_info_temp);
	} else {
		ts->ts_ops->get_gesture_info_auto(ts->chip_data, &gesture_info_temp, &ts->resolution_info);
	}
	tp_geture_info_transform(&gesture_info_temp, &ts->resolution_info);

	TP_INFO(ts->tp_index, "detect %s gesture\n",
		gesture_info_temp.gesture_type == DOU_TAP ? "double tap" :
		gesture_info_temp.gesture_type == UP_VEE ? "up vee" :
		gesture_info_temp.gesture_type == DOWN_VEE ? "down vee" :
		gesture_info_temp.gesture_type == LEFT_VEE ? "(>)" :
		gesture_info_temp.gesture_type == RIGHT_VEE ? "(<)" :
		gesture_info_temp.gesture_type == CIRCLE_GESTURE ? "circle" :
		gesture_info_temp.gesture_type == DOU_SWIP ? "(||)" :
		gesture_info_temp.gesture_type == LEFT2RIGHT_SWIP ? "(-->)" :
		gesture_info_temp.gesture_type == RIGHT2LEFT_SWIP ? "(<--)" :
		gesture_info_temp.gesture_type == UP2DOWN_SWIP ? "up to down |" :
		gesture_info_temp.gesture_type == DOWN2UP_SWIP ? "down to up |" :
		gesture_info_temp.gesture_type == M_GESTRUE ? "(M)" :
		gesture_info_temp.gesture_type == W_GESTURE ? "(W)" :
		gesture_info_temp.gesture_type == FINGER_PRINTDOWN ? "(fingerprintdown)" :
		gesture_info_temp.gesture_type == FRINGER_PRINTUP ? "(fingerprintup)" :
		gesture_info_temp.gesture_type == SINGLE_TAP ? "single tap" :
		gesture_info_temp.gesture_type == HEART ? "heart" :
		gesture_info_temp.gesture_type == PENDETECT ? "(pen detect)" :
		gesture_info_temp.gesture_type == FP_GESTURE_HOLD ? "fp_gesture_hold" :
		gesture_info_temp.gesture_type == FP_GESTURE_RELEASE ? "fp_gesture_release" :
		gesture_info_temp.gesture_type == S_GESTURE ? "(S)" : "unknown");

	if (ts->health_monitor_support) {
		tp_healthinfo_report(&ts->monitor_data, HEALTH_GESTURE,
				     &gesture_info_temp.gesture_type);
	}

#ifdef CONFIG_OPLUS_TP_APK

	if (ts->gesture_debug_sta) {
		input_report_key(ts->input_dev, KEY_POWER, 1);
		input_sync(ts->input_dev);
		input_report_key(ts->input_dev, KEY_POWER, 0);
		input_sync(ts->input_dev);
		return;
	}

#endif /* end of CONFIG_OPLUS_TP_APK*/

	if (gesture_info_temp.gesture_type != UNKOWN_GESTURE
	    && gesture_info_temp.gesture_type != FINGER_PRINTDOWN
	    && gesture_info_temp.gesture_type != FRINGER_PRINTUP) {
		retval = tp_memcpy(&ts->gesture, sizeof(ts->gesture), \
			  &gesture_info_temp, sizeof(struct gesture_info), \
			  sizeof(struct gesture_info));
		if (retval < 0) {
		    TP_INFO(ts->tp_index, "tp_memcpy failed.\n");
		}

		for (index = 0; index < TP_SUPPORT_MAX; index++) {
			g_ts = get_ts_data(index);
			if (g_ts) {
				g_ts->gesture.gesture_panel_id = ts->tp_index;
			}
		}

		input_report_key(ts->input_dev, KEY_F4, 1);
		input_sync(ts->input_dev);
		input_report_key(ts->input_dev, KEY_F4, 0);
		input_sync(ts->input_dev);

	} else if (gesture_info_temp.gesture_type == FINGER_PRINTDOWN) {
		ts->fp_info.touch_state = 1;
		ts->fp_info.x = gesture_info_temp.Point_start.x;
		ts->fp_info.y = gesture_info_temp.Point_start.y;
		TP_INFO(ts->tp_index, "screen off down : (%d, %d)\n", ts->fp_info.x, ts->fp_info.y);
		touch_call_notifier_fp(ts, &ts->fp_info);
	} else if (gesture_info_temp.gesture_type == FRINGER_PRINTUP) {
		ts->fp_info.touch_state = 0;
		ts->fp_info.x = gesture_info_temp.Point_start.x;
		ts->fp_info.y = gesture_info_temp.Point_start.y;
		TP_INFO(ts->tp_index, "screen off up : (%d, %d)\n", ts->fp_info.x, ts->fp_info.y);
		touch_call_notifier_fp(ts, &ts->fp_info);
	}

	if (ts->fp_grip_support) {
		if (gesture_info_temp.gesture_type == FP_GESTURE_HOLD) {
			ts->fp_grip_hold = true;
			TP_INFO(ts->tp_index, "FP_GESTURE_HOLD:%d\n", ts->fp_grip_hold);
			touch_call_fp_grip(ts, 1);
		} else if (gesture_info_temp.gesture_type == FP_GESTURE_RELEASE) {
			ts->fp_grip_hold = false;
			TP_INFO(ts->tp_index, "FP_GESTURE_RELEASE:%d\n", ts->fp_grip_hold);
			touch_call_fp_grip(ts, 0);
		}
	}
}

void tp_touch_btnkey_release(unsigned int tp_index)
{
	struct touchpanel_data *ts = NULL;

	if (tp_index >= TP_SUPPORT_MAX) {
		return;
	}

	ts = get_ts_data(tp_index);

	if (!ts) {
		return;
	}

	tp_touch_release(ts);
	tp_btnkey_release(ts);
}
EXPORT_SYMBOL(tp_touch_btnkey_release);

static void tp_touch_release(struct touchpanel_data *ts)
{
	int i = 0;
	if (ts->en_touch_event_helper) {
		post_message(ts->msg_list, 0, TYPE_RESET, NULL);
	}


	mutex_lock(&ts->report_mutex);

	if (ts->is_pen_connected && ts->pen_mode_tp_state == CANCLE_TP) {
		ts->pen_mode_tp_state = DEFAULT;
		input_mt_slot(ts->input_dev, 0);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_PALM, 1);
		input_sync(ts->input_dev);
		TP_INFO(ts->tp_index,
		"detect touch up in pen mode, need send 'cancle' to apk\n");
	}

	for (i = 0; i < ts->max_num; i++) {
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}

	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
	input_sync(ts->input_dev);
	mutex_unlock(&ts->report_mutex);

	TP_INFO(ts->tp_index,
		"release all touch point and key, clear tp touch down flag\n");
	ts->view_area_touched = 0; /*realse all touch point,must clear this flag*/
	ts->touch_count = 0;
	ts->irq_slot = 0;
}

static inline void tp_touch_helper_handle(struct touchpanel_data *ts)
{
	int obj_attention = 0;
	struct point_info points[MAX_FINGER_NUM];

	if (!ts->ts_ops->get_touch_points_help) {
		TP_INFO(ts->tp_index, "not support ts->ts_ops->get_touch_points_help callback\n");
		return;
	}

	memset(points, 0, sizeof(points));

	obj_attention = ts->ts_ops->get_touch_points_help(ts->chip_data,
				points,
				ts->max_num,
				&ts->resolution_info);
	if (obj_attention == -EINVAL) {
		TP_INFO(ts->tp_index, "Invalid points, ignore..\n");
		return;
	}

	post_message(ts->msg_list,sizeof(points), TYPE_POINT, (u8*)&points);
	return;
}

static void tp_palm_to_sleep_inScreenLock(struct touchpanel_data *ts)
{
	char *palm_report = NULL;

	if (ts->palm_to_sleep_support) {
		input_report_key(ts->input_dev, KEY_SLEEP, 1);
		input_sync(ts->input_dev);
		input_report_key(ts->input_dev, KEY_SLEEP, 0);
		input_sync(ts->input_dev);
		palm_report = tp_kzalloc(30, GFP_KERNEL);
		if (palm_report) {
			snprintf(palm_report, 30, "palm_to_sleep_in_screenLock");
			tp_healthinfo_report(&ts->monitor_data, HEALTH_REPORT, palm_report);
			tp_kfree((void **)&palm_report);
		}
	}
}

static inline void tp_touch_handle(struct touchpanel_data *ts)
{
	int i = 0;
	uint8_t finger_num = 0, touch_near_edge = 0, finger_num_center = 0;
	int obj_attention = 0;
	int retval = 0;
	struct point_info points[MAX_FINGER_NUM];
	int down_time_set = 0;
	u64 delta_kernel_time = 0;

	if (((!ts->ts_ops->get_touch_points) && (!ts->enable_point_auto_change))
	    || ((!ts->ts_ops->get_touch_points_auto) && ts->enable_point_auto_change)) {
		TP_INFO(ts->tp_index, "not support ts->ts_ops->get_touch_points callback\n");
		return;
	}

	if(tp_debug == 1 && ts->monitor_data.RATE_MIN) {
		tp_rate_calc(ts, TP_RATE_START);
	}

	memset(points, 0, sizeof(points));

	if (!ts->enable_point_auto_change) {
		obj_attention = ts->ts_ops->get_touch_points(ts->chip_data, points, ts->max_num);
		if (obj_attention == -EINVAL) {
			TP_INFO(ts->tp_index, "Invalid points, ignore..\n");
			return;
		}
	} else {
		obj_attention = ts->ts_ops->get_touch_points_auto(ts->chip_data,
				points,
				ts->max_num,
				&ts->resolution_info);
		if (obj_attention == -EINVAL) {
			TP_INFO(ts->tp_index, "Invalid points, ignore..\n");
			return;
		}
	}

	mutex_lock(&ts->report_mutex);
	if (ts->health_monitor_support) {
		ts->monitor_data.touch_points = points;
		tp_healthinfo_report(&ts->monitor_data, HEALTH_RAW_TOUCH, &obj_attention);
	}
	if (ts->kernel_grip_support && (!ts->grip_no_driver_support)) {
		obj_attention = notify_prevention_handle(ts->grip_info, obj_attention, points);
		if (ts->health_monitor_support) {
			tp_healthinfo_report(&ts->monitor_data, HEALTH_GRIP, ts->grip_info);
		}
	}

	retval = touch_points_detect_algo(ts, points, obj_attention);
	if (retval) {
		TP_INFO(ts->tp_index, "touch_points_detect_algo suc..\n");
	}

	if (ts->major_rate_limit_support && !!(ts->noise_level)) {
		ts->tp_ic_touch_num = 0;
		for (i = 0; i < ts->max_num; i++) {
			if (((obj_attention & TOUCH_BIT_CHECK) >> i) & 0x01
					&& (points[i].status != 0)) {
				ts->tp_ic_touch_num++;
			}
		}
	}

	if ((obj_attention & TOUCH_BIT_CHECK) != 0) {
		ts->up_status = false;

		for (i = 0; i < ts->max_num; i++) {
			if (((obj_attention & TOUCH_BIT_CHECK) >> i) & 0x01
			    && (points[i].status == 0)) { /* buf[0] == 0 is wrong point, no process*/
				continue;
			}
			if (points[i].x > ts->resolution_info.max_x - 1
			    || points[i].y > ts->resolution_info.max_y - 1) { /* x y over max, no process*/
				tp_healthinfo_report(&ts->monitor_data, HEALTH_REPORT, "x_y_over_max");
				continue;
			}

			if (((obj_attention & TOUCH_BIT_CHECK) >> i) & 0x01
			    && (points[i].status != 0) && (!ts->disable_touch_event)) {
				input_mt_slot(ts->input_dev, i);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
				ts->touch_report_num++;
				tp_touch_down(ts, points[i], ts->touch_report_num, i);
				SET_BIT(ts->irq_slot, (1 << i));
				finger_num++;

				if (ts->face_detect_support && ts->fd_enable && \
				    (points[i].y < ts->resolution_info.max_y / 2) && (points[i].x > 30)
				    && (points[i].x < ts->resolution_info.max_x - 30)) {
					finger_num_center++;
				}

				if (points[i].x > ts->resolution_info.max_x / 100
				    && points[i].x < ts->resolution_info.max_x * 99 / 100) {
					ts->view_area_touched = finger_num;

				} else {
					touch_near_edge++;
				}

				/*strore  the last point data*/
				retval = tp_memcpy(&ts->last_point, sizeof(ts->last_point), \
					  &points[i], sizeof(struct point_info), \
					  sizeof(struct point_info));
				if (retval < 0) {
					TP_INFO(ts->tp_index, "tp_memcpy failed.\n");
				}
				if ((down_time_set == 0) && (ts->max_report_rate_limit)) {
					ts->touch_frame_num++;
					down_time_set = 1;
					if (ts->touch_frame_num == 1) {
						ts->touch_down_time = ktime_to_ms(ktime_get());
					} else if (ts->touch_frame_num == (MAX_REPORT_FRAME_TEST_FRAME + 1)) {
						delta_kernel_time = ktime_to_ms(ktime_get()) - ts->touch_down_time;
						if (((1000 * MAX_REPORT_FRAME_TEST_FRAME) / delta_kernel_time) > ts->max_report_rate_limit) {
							TP_INFO(ts->tp_index, "tp report rate error. rate_rate:%llu, limit:%u \n", \
									((1000 * MAX_REPORT_FRAME_TEST_FRAME) / delta_kernel_time), ts->max_report_rate_limit);
							if (ts->health_monitor_support) {
								tp_healthinfo_report(&ts->monitor_data, HEALTH_REPORT, "report_rate_error");
							}
						}
					}
				}
			}
			else {
				input_mt_slot(ts->input_dev, i);

				if (ts->kernel_grip_support && ts->grip_info
				    && ts->grip_info->eli_reject_status[i]
				    && !(ts->grip_info->grip_disable_level & (1 << GRIP_DISABLE_UP2CANCEL))) {
					input_report_abs(ts->input_dev, ABS_MT_PRESSURE, UP2CANCEL_PRESSURE_VALUE);
				}

				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
				if (CHK_BIT(ts->irq_slot, (1 << i))) {
					TP_INFO(ts->tp_index, "touch point id %d up.\n", i);
					CLR_BIT(ts->irq_slot, (1 << i));
				}
			}
		}

		ts->last_tp_ic_touch_num = ts->tp_ic_touch_num;
		if (touch_near_edge ==
		    finger_num) {        /*means all the touchpoint is near the edge*/
			ts->view_area_touched = 0;
		}

	} else {
		if (ts->up_status) {
			tp_touch_up(ts);
			mutex_unlock(&ts->report_mutex);
			return;
		}

		ts->total_operate_times++;
		finger_num = 0;
		finger_num_center = 0;
		ts->touch_frame_num = 0;
		ts->touch_report_num = 0;
		ts->tp_ic_touch_num = 0;
		ts->last_tp_ic_touch_num = 0;

		if (ts->is_pen_connected && ts->pen_mode_tp_state == CANCLE_TP) {
			ts->pen_mode_tp_state = DEFAULT;
			input_mt_slot(ts->input_dev, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_PALM, 1);
			input_sync(ts->input_dev);
			TP_INFO(ts->tp_index,
			"detect touch up in pen mode,send touch cancle\n");
		}

		for (i = 0; i < ts->max_num; i++) {
			input_mt_slot(ts->input_dev, i);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
			ts->last_x_y_point[i].x = 0;
			ts->last_x_y_point[i].y = 0;
		}

		tp_touch_up(ts);
		ts->view_area_touched = 0;
		ts->irq_slot = 0;
		ts->up_status = true;
		TP_DETAIL(ts->tp_index, "all touch up,view_area_touched=%d finger_num=%d\n",
			  ts->view_area_touched, finger_num);
		TP_DETAIL(ts->tp_index, "last point x:%d y:%d\n", ts->last_point.x,
			  ts->last_point.y);
	}

	input_sync(ts->input_dev);
	ts->touch_count = (finger_num << 4) | (finger_num_center & 0x0F);
	mutex_unlock(&ts->report_mutex);

	if (ts->health_monitor_support) {
		ts->monitor_data.touch_points = points;
		ts->monitor_data.touch_num = finger_num;
		ts->monitor_data.direction = ts->grip_info ? ts->grip_info->touch_dir :
					     ts->limit_enable;
		tp_healthinfo_report(&ts->monitor_data, HEALTH_TOUCH, &obj_attention);
	}
}

/**
 * input_report_key_oplus - Using for report virtual key
 * @work: work struct using for this thread
 *
 * before report virtual key, detect whether touch_area has been touched
 * Do not care the result: Return void type
 */
void input_report_key_oplus(struct touchpanel_data *ts, unsigned int code,
			    int value)
{
	if (!ts) {
		return;
	}

	if (value) { /*report Key[down]*/
		if (ts->view_area_touched == 0) {
			input_report_key(ts->kpd_input_dev, code, value);

		} else {
			TP_INFO(ts->tp_index, "Sorry,tp is touch down,can not report touch key\n");
		}

	} else {
		input_report_key(ts->kpd_input_dev, code, value);
	}
}

static void tp_btnkey_release(struct touchpanel_data *ts)
{
	if (CHK_BIT(ts->vk_bitmap, BIT_MENU)) {
		input_report_key_oplus(ts, KEY_MENU, 0);
	}

	if (CHK_BIT(ts->vk_bitmap, BIT_HOME)) {
		input_report_key_oplus(ts, KEY_HOMEPAGE, 0);
	}

	if (CHK_BIT(ts->vk_bitmap, BIT_BACK)) {
		input_report_key_oplus(ts, KEY_BACK, 0);
	}

	input_sync(ts->kpd_input_dev);
}

static void tp_btnkey_handle(struct touchpanel_data *ts)
{
	u8 touch_state = 0;

	if (ts->vk_type != TYPE_AREA_SEPRATE) {
		TPD_DEBUG("TP vk_type not proper, checktouchpanel, button-type\n");

		return;
	}

	if (!ts->ts_ops->get_keycode) {
		TP_INFO(ts->tp_index, "not support ts->ts_ops->get_keycode callback\n");

		return;
	}

	touch_state = ts->ts_ops->get_keycode(ts->chip_data);

	if (CHK_BIT(ts->vk_bitmap, BIT_MENU)) {
		input_report_key_oplus(ts, KEY_MENU, CHK_BIT(touch_state, BIT_MENU));
	}

	if (CHK_BIT(ts->vk_bitmap, BIT_HOME)) {
		input_report_key_oplus(ts, KEY_HOMEPAGE, CHK_BIT(touch_state, BIT_HOME));
	}

	if (CHK_BIT(ts->vk_bitmap, BIT_BACK)) {
		input_report_key_oplus(ts, KEY_BACK, CHK_BIT(touch_state, BIT_BACK));
	}

	input_sync(ts->kpd_input_dev);
}

static void tp_config_handle(struct touchpanel_data *ts)
{
	int ret = 0;

	if (!ts->ts_ops->fw_handle) {
		TP_INFO(ts->tp_index, "not support ts->ts_ops->fw_handle callback\n");
		return;
	}

	ret = ts->ts_ops->fw_handle(ts->chip_data);
}

static void health_monitor_handle(struct touchpanel_data *ts)
{
	if (!ts->ts_ops->health_report) {
		TP_INFO(ts->tp_index,
			"not support ts->debug_info_ops->health_report callback\n");
		return;
	}

	if (tp_debug || ts->health_monitor_support) {
		ts->ts_ops->health_report(ts->chip_data, &ts->monitor_data);
	}
}

static void tp_face_detect_handle(struct touchpanel_data *ts)
{
	int ps_state = 0;

	if (!ts->ts_ops->get_face_state) {
		TP_INFO(ts->tp_index, "not support ts->ts_ops->get_face_state callback\n");
		return;
	}

	ps_state = ts->ts_ops->get_face_state(ts->chip_data);

	if (ps_state < 0) {
		return;
	}

	input_event(ts->ps_input_dev, EV_MSC, MSC_RAW, ps_state);
	input_sync(ts->ps_input_dev);

	if (ts->health_monitor_support) {
		tp_healthinfo_report(&ts->monitor_data, HEALTH_FACE_DETECT, &ps_state);
	}
}

static void tp_pen_handle(struct touchpanel_data *ts)
{
	struct pen_info pen_info;
	static int point_num = 0;
	static struct pen_info last_point = {.x = 0, .y = 0};
	static bool up_status = false;
	if (!ts->ts_ops->get_pen_points) {
		TP_INFO(ts->tp_index, "not support get_pen_points callback.\n");
		return;
	}

	memset(&pen_info, 0, sizeof(pen_info));

	ts->ts_ops->get_pen_points(ts->chip_data, &pen_info);

	if (ts->pen_support_opp) {
		touch_pen_speed_handle(ts, pen_info.speed);
	}
	if (pen_info.status == 1) {
		if (up_status == true) {
			TPD_DETAIL("first pen point [%4d %4d %4d]\n", pen_info.x, pen_info.y, pen_info.z);
		}
		up_status = false;
		input_report_abs(ts->pen_input_dev, ABS_X, pen_info.x);
		input_report_abs(ts->pen_input_dev, ABS_Y, pen_info.y);
		input_report_abs(ts->pen_input_dev, ABS_PRESSURE, pen_info.z);
		input_report_abs(ts->pen_input_dev, ABS_TILT_X, pen_info.tilt_x);
		input_report_abs(ts->pen_input_dev, ABS_TILT_Y, pen_info.tilt_y);
		input_report_abs(ts->pen_input_dev, ABS_DISTANCE, pen_info.d);
		input_report_key(ts->pen_input_dev, BTN_TOUCH, !!pen_info.z);
		input_report_key(ts->pen_input_dev, BTN_TOOL_PEN, !!pen_info.z || !!pen_info.d);
		input_report_key(ts->pen_input_dev, BTN_STYLUS, pen_info.btn1);
		input_report_key(ts->pen_input_dev, BTN_STYLUS2, pen_info.btn2);

		TPD_SPECIFIC_PRINT(point_num, "Pen Down[%4d %4d %4d %4d %4d]\n", \
								pen_info.x, pen_info.y, pen_info.z, pen_info.tilt_x, pen_info.tilt_y);
		/*strore  the last point data*/
		memcpy(&last_point, &pen_info, sizeof(struct pen_info));
	} else if ((pen_info.status == 0) && (up_status == false)) {
		input_report_abs(ts->pen_input_dev, ABS_X, 0);
		input_report_abs(ts->pen_input_dev, ABS_Y, 0);
		input_report_abs(ts->pen_input_dev, ABS_PRESSURE, 0);
		input_report_abs(ts->pen_input_dev, ABS_TILT_X, 0);
		input_report_abs(ts->pen_input_dev, ABS_TILT_Y, 0);
		input_report_abs(ts->pen_input_dev, ABS_DISTANCE, 0);
		input_report_key(ts->pen_input_dev, BTN_TOUCH, 0);
		input_report_key(ts->pen_input_dev, BTN_TOOL_PEN, 0);
		input_report_key(ts->pen_input_dev, BTN_STYLUS, 0);
		input_report_key(ts->pen_input_dev, BTN_STYLUS2, 0);
		up_status = true;
        TPD_DETAIL("Pen Up, last point x:%d y:%d\n", last_point.x, last_point.y);
	}

	input_sync(ts->pen_input_dev);
}

static void tp_fingerprint_handle(struct touchpanel_data *ts)
{
	struct fp_underscreen_info fp_tpinfo;

	if (((!ts->ts_ops->screenon_fingerprint_info) && (!ts->enable_point_auto_change))
	    || ((!ts->ts_ops->screenon_fingerprint_info_auto) && ts->enable_point_auto_change)) {
		TP_INFO(ts->tp_index, "not support screenon_fingerprint_info callback.\n");
		return;
	}

	if (!ts->enable_point_auto_change) {
		ts->ts_ops->screenon_fingerprint_info(ts->chip_data, &fp_tpinfo);
	} else {
		ts->ts_ops->screenon_fingerprint_info_auto(ts->chip_data,
							   &fp_tpinfo,
							   &ts->resolution_info);
	}
	ts->fp_info.area_rate = fp_tpinfo.area_rate;
	ts->fp_info.x = fp_tpinfo.x;
	ts->fp_info.y = fp_tpinfo.y;
	if (fp_tpinfo.touch_state == FINGERPRINT_DOWN_DETECT) {
		TP_INFO(ts->tp_index, "screen on down : (%d, %d)\n",
			ts->fp_info.x,
			ts->fp_info.y);

		ts->fp_info.touch_state = 1;
		touch_call_notifier_fp(ts, &ts->fp_info);
		if (ts->health_monitor_support) {
			tp_healthinfo_report(&ts->monitor_data,
					     HEALTH_FINGERPRINT,
					     &fp_tpinfo.area_rate);
		}
	} else if (fp_tpinfo.touch_state == FINGERPRINT_UP_DETECT) {
		TP_INFO(ts->tp_index, "screen on up : (%d, %d)\n", ts->fp_info.x, ts->fp_info.y);
		ts->fp_info.touch_state = 0;
		touch_call_notifier_fp(ts, &ts->fp_info);
	} else if (ts->fp_info.touch_state) {
		touch_call_notifier_fp(ts, &ts->fp_info);
	}
}

static inline void tp_work_func(struct touchpanel_data *ts)
{
	u32 cur_event = 0;

	if (!ts->ts_ops->trigger_reason) {
		TP_INFO(ts->tp_index, "not support ts_ops->trigger_reason callback\n");
		return;
	}

	/*
	 *  trigger_reason:this callback determine which trigger reason should be
	 *  The value returned has some policy!
	 *  1.IRQ_EXCEPTION /IRQ_GESTURE /IRQ_IGNORE /IRQ_FW_CONFIG --->should be only reported  individually
	 *  2.IRQ_TOUCH && IRQ_BTN_KEY --->should depends on real situation && set correspond bit on trigger_reason
	 */
	if (ts->ts_ops->trigger_reason) {
		cur_event = ts->ts_ops->trigger_reason(ts->chip_data, (ts->gesture_enable
						       || ts->fp_enable), ts->is_suspended);
	}

	if (ts->health_monitor_support) {
		tp_healthinfo_report(&ts->monitor_data, HEALTH_IRQ_TYPE, &cur_event);
	}

	if (CHK_BIT(cur_event, IRQ_TOUCH) || CHK_BIT(cur_event, IRQ_BTN_KEY)
		|| CHK_BIT(cur_event, IRQ_FW_HEALTH) || CHK_BIT(cur_event, IRQ_PEN) || \
		CHK_BIT(cur_event, IRQ_FACE_STATE) || CHK_BIT(cur_event, IRQ_FINGERPRINT)
		|| CHK_BIT(cur_event, IRQ_PALM) || CHK_BIT(cur_event, IRQ_PEN_REPORT)) {
		if (CHK_BIT(cur_event, IRQ_BTN_KEY)) {
			tp_btnkey_handle(ts);
		}

		if (CHK_BIT(cur_event, IRQ_PEN) && ts->pen_support) {
			tp_pen_handle(ts);
		}

		if (CHK_BIT(cur_event, IRQ_PALM)
			&& ts->palm_to_sleep_enable && !ts->is_suspended && !ts->disable_touch_event) {
			tp_palm_to_sleep_inScreenLock(ts);
		}

		if (CHK_BIT(cur_event, IRQ_TOUCH)) {
			if (ts->en_touch_event_helper) {
				tp_touch_helper_handle(ts);
			} else {
				tp_touch_handle(ts);
			}
		}
		if (CHK_BIT(cur_event, IRQ_PEN_REPORT)) {
			touch_pen_uplink_msg_handle(ts);
		}

		if (CHK_BIT(cur_event, IRQ_FW_HEALTH) && (!ts->is_suspended)) {
			health_monitor_handle(ts);
		}

		if (CHK_BIT(cur_event, IRQ_FACE_STATE) && ts->fd_enable) {
			tp_face_detect_handle(ts);
		}

		if (CHK_BIT(cur_event, IRQ_FINGERPRINT) && ts->fp_enable) {
			tp_fingerprint_handle(ts);
		}

	} else if (CHK_BIT(cur_event, IRQ_GESTURE)) {
		tp_gesture_handle(ts);

	} else if (CHK_BIT(cur_event, IRQ_EXCEPTION)) {
		tp_exception_handle(ts);

	} else if (CHK_BIT(cur_event, IRQ_FW_CONFIG)) {
		tp_config_handle(ts);

	} else if (CHK_BIT(cur_event, IRQ_FW_AUTO_RESET)) {
		tp_fw_auto_reset_handle(ts);

	} else {
		TPD_DEBUG("unknown irq trigger reason\n");
	}
}

#define TP_FWUP_HEADER    "FwUp"
static void tp_fw_update_work(struct work_struct *work)
{
	const struct firmware *fw = NULL;
	int ret, fw_update_result = 0;
	int count_tmp = 0, retry = 5;
	char *p_node = NULL;
	char *fw_name_fae = NULL;
	char *postfix = "_FAE";
	uint8_t copy_len = 0;
	u64 start_time = 0;

	struct touchpanel_data *ts = container_of(work, struct touchpanel_data,
				     fw_update_work);

	touchpanel_trusted_touch_completion(ts);

	if (!ts->ts_ops->fw_check || !ts->ts_ops->reset) {
		TP_INFO(ts->tp_index, "not support ts_ops->fw_check callback\n");
		complete(&ts->fw_complete);
		return;
	}

	TP_INFO(ts->tp_index, "%s: fw_name = %s\n", __func__, ts->panel_data.fw_name);

	if (ts->health_monitor_support) {
		reset_healthinfo_time_counter(&start_time);
	}

	mutex_lock(&ts->mutex);

	if (!ts->irq_trigger_hdl_support && ts->int_mode == BANNABLE) {
		disable_irq_nosync(ts->irq);
	}

	ts->loading_fw = true;

	if (ts->esd_handle_support) {
		esd_handle_switch(&ts->esd_info, false);
	}

	display_esd_check_enable_bytouchpanel(0);
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
#ifndef CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY
	primary_display_esd_check_enable(0); /*avoid rst pulled to low while updating*/
#endif
#endif

	if (ts->ts_ops->fw_update) {
		do {
			if (ts->firmware_update_type == 0 || ts->firmware_update_type == 1) {
				if (ts->fw_update_app_support) {
					fw_name_fae = tp_devm_kzalloc(ts->dev, MAX_FW_NAME_LENGTH, GFP_KERNEL);

					if (fw_name_fae == NULL) {
						TP_INFO(ts->tp_index, "fw_name_fae kzalloc error!\n");
						goto EXIT;
					}

					p_node  = strstr(ts->panel_data.fw_name, ".");

					if (p_node == NULL) {
						TP_INFO(ts->tp_index, "p_node strstr error!\n");
						goto EXIT;
					}

					copy_len = p_node - ts->panel_data.fw_name;
					memcpy(fw_name_fae, ts->panel_data.fw_name, copy_len);
					strlcat(fw_name_fae, postfix, MAX_FW_NAME_LENGTH);
					strlcat(fw_name_fae, p_node, MAX_FW_NAME_LENGTH);
					TP_INFO(ts->tp_index, "fw_name_fae is %s\n", fw_name_fae);
					ret = request_firmware(&fw, fw_name_fae, ts->dev);

					if (!ret) {
						break;
					}

				} else {
					ret = request_firmware(&fw, ts->panel_data.fw_name, ts->dev);

					if (!ret) {
						break;
					}
				}

			} else {
				ret = request_firmware_select(&fw, ts->panel_data.fw_name, ts->dev);

				if (!ret) {
					if (fw->size > sizeof(TP_FWUP_HEADER)) {
						TP_INFO(ts->tp_index, "fw_header:%*ph\n", (int)(sizeof(TP_FWUP_HEADER) - 1), fw->data);
						if (!strncmp(fw->data, TP_FWUP_HEADER, sizeof(TP_FWUP_HEADER) - 1)) {
							TP_INFO(ts->tp_index, "error ! fw_header is signed,do not update\n");
						}
					}
					break;
				}
			}
		} while ((ret < 0) && (--retry > 0));

		TP_INFO(ts->tp_index, "retry times %d\n", 5 - retry);

		if (!ret || ts->is_noflash_ic) {
			do {
				count_tmp++;
				ret = ts->ts_ops->fw_update(ts->chip_data, fw, ts->force_update);
				fw_update_result = ret;

				if (ret == FW_NO_NEED_UPDATE) {
					break;
				}

				if ((ts->fpga_spi_agg_support) && (ret > FW_NO_NEED_UPDATE)) {
					msleep(1000);
					ret = ts->ts_ops->fw_update(ts->chip_data, fw, ts->force_update);
					fw_update_result = ret;
				}

				if (!ts->is_noflash_ic) {       /*noflash update fw in reset and do bootloader reset in get_chip_info*/
					ret |= ts->ts_ops->reset(ts->chip_data);
					ret |= ts->ts_ops->get_chip_info(ts->chip_data);
				}

				ret |= ts->ts_ops->fw_check(ts->chip_data, &ts->resolution_info,
							    &ts->panel_data);
			} while ((count_tmp < 2) && (ret != 0));

			if (fw != NULL) {
				release_firmware(fw);
			}

		} else {
			TP_INFO(ts->tp_index, "%s: fw_name request failed %s %d\n", __func__,
				ts->panel_data.fw_name, ret);

			if (ts->health_monitor_support) {
				tp_healthinfo_report(&ts->monitor_data, HEALTH_FW_UPDATE, "FW_Request_Failed");
			}
			if (ts->exception_upload_support) {
				tp_exception_report(&ts->exception_data, EXCEP_FW_UPDATE, "FW_Request_Failed", sizeof("FW_Request_Failed"));
			}
			ts->is_update_log = 1;
			fw_update_result = FW_UPDATE_ERROR;
			goto EXIT;
		}
	}

	tp_touch_release(ts);
	tp_btnkey_release(ts);
	operate_mode_switch(ts);

EXIT:
	ts->loading_fw = false;

	display_esd_check_enable_bytouchpanel(1);
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
#ifndef CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY
	primary_display_esd_check_enable(1); /*avoid rst pulled to low while updating*/
#endif
#endif

	if (ts->esd_handle_support) {
		esd_handle_switch(&ts->esd_info, true);
	}

	tp_devm_kfree(ts->dev, (void **)&fw_name_fae, MAX_FW_NAME_LENGTH);

	if (ts->int_mode == BANNABLE) {
	enable_irq(ts->irq);
	}

	mutex_unlock(&ts->mutex);

	if (ts->health_monitor_support) {
		tp_healthinfo_report(&ts->monitor_data, HEALTH_FW_UPDATE_COST, &start_time);
	}

	if (fw_update_result == FW_UPDATE_ERROR || fw_update_result == FW_UPDATE_FATAL) {
		if (ts->health_monitor_support) {
			tp_healthinfo_report(&ts->monitor_data, HEALTH_FW_UPDATE, "FW_Update_Failed");
		}
		if (ts->exception_upload_support) {
			tp_exception_report(&ts->exception_data, EXCEP_FW_UPDATE, "FW_Update_Failed", sizeof("FW_Update_Failed"));
		}
		/*if probe_status is not exit, olc will update log*/
		ts->is_update_log = 1;
	} else {
		/*if probe_status is not exit, olc will update log*/
		ts->is_update_log = 0;
	}

	ts->force_update = 0;

	complete(&ts->fw_complete); /*notify to init.rc that fw update finished*/
	return;
}

static bool monitor_irq_bus_ready(struct touchpanel_data *ts)
{
	struct monitor_data *moni = NULL;

	moni = &ts->monitor_data;

	/*device suspend start*/
	if (false == ts->bus_ready) {
		moni->irq_need_dev_resume_all_count++;
		moni->irq_bus_not_ready_count++;
		TP_INFO(ts->tp_index, "The device not resume %d ms!", ts->irq_need_dev_resume_time);

		if (ts->suspend_task && ts->supspend_task_error_cnt < MAX_SUSPEND_TASK_DUMP_COUNT) {
			ts->supspend_task_error_cnt++;
			sched_show_task(ts->suspend_task);
		}
		if (ts->exception_upload_support) {
			tp_exception_report(&ts->exception_data, EXCEP_BUS_READY, "device not resume", sizeof("device not resume"));
		}
		return false;
	} else {/*device resume end*/
		if (moni->irq_bus_not_ready_count > moni->irq_need_dev_resume_max_count) {
			moni->irq_need_dev_resume_max_count = moni->irq_bus_not_ready_count;
		}
		moni->irq_bus_not_ready_count = 0;
		return true;
	}
	return true;
}

/**
 * @brief Set the input timestamp in top half interrupt object
 * @param irq
 * @param dev_id
 * @return irqreturn_t
 */
static irqreturn_t set_input_timestamp_in_top_irq(int irq, void *dev_id)
{
	struct touchpanel_data *ts = (struct touchpanel_data *)dev_id;

	if (ts->input_dev && ts->input_timestamp_in_top_irq_support) {
		input_set_timestamp(ts->input_dev, ktime_get());
	}
	return IRQ_WAKE_THREAD;
}

static irqreturn_t tp_irq_thread_fn(int irq, void *dev_id)
{
	struct touchpanel_data *ts = (struct touchpanel_data *)dev_id;
#ifdef CONFIG_TOUCHPANEL_TRUSTED_TOUCH
#ifndef CONFIG_ARCH_QTI_VM
	if (atomic_read(&ts->trusted_touch_transition) || atomic_read(&ts->trusted_touch_enabled))
		goto exit;
#endif
#endif
	ts->monitor_data.irq_to_report_timer = ktime_get();

	if (ts->ws) {
		__pm_stay_awake(ts->ws);
	}

	/*for qaulcomn to stop cpu go to C4 idle state*/
#ifdef CONFIG_TOUCHIRQ_UPDATE_QOS

	if (ts->pm_qos_state && !ts->is_suspended && !ts->touch_count) {
		ts->pm_qos_value = PM_QOS_TOUCH_WAKEUP_VALUE;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,10,0)

		if (!cpu_latency_qos_request_active(&ts->pm_qos_req)) {
			cpu_latency_qos_add_request(&ts->pm_qos_req, ts->pm_qos_value);
		} else {
			cpu_latency_qos_update_request(&ts->pm_qos_req, ts->pm_qos_value);
		}
#else
		pm_qos_update_request(&ts->pm_qos_req, ts->pm_qos_value);
#endif
	}

#endif

	if (ts->ts_ops->tp_irq_throw_away) {
		if (ts->ts_ops->tp_irq_throw_away(ts->chip_data)) {
			goto exit;
		}
	}

	/*for stop system go to sleep*/
	/*wake_lock_timeout(&ts->wakelock, 1*HZ);*/

	/*for check bus i2c/spi is ready or not*/
	if (ts->bus_ready == false) {
		/*TP_INFO(ts->tp_index, "Wait device resume!");*/
		wait_event_interruptible_timeout(ts->wait,
						 ts->bus_ready,
						 msecs_to_jiffies(ts->irq_need_dev_resume_time));
		/*TP_INFO(ts->tp_index, "Device maybe resume!");*/
	}

	if (false == monitor_irq_bus_ready(ts)) {
		goto exit;
	}

	/*for some ic such as samsung ic*/
	if (ts->sec_long_low_trigger) {
		disable_irq_nosync(ts->irq);
	}

	/*for normal ic*/
	if (ts->int_mode == BANNABLE) {
		mutex_lock(&ts->mutex);
		/*Only if the following four conditions are met at the same time:
		 tp is suspend && gesture is disable && irq flag is edge-triggered && disable_suspend_irq_handler is set to 1,
		tp_work_func(ts)  will be ignored */
		if (!ts->is_suspended || (TP_ALL_GESTURE_SUPPORT && TP_ALL_GESTURE_ENABLE) ||
				!(ts->irq_flags & (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING)) ||
				!(ts->disable_suspend_irq_handler)) {
			tp_work_func(ts);
		}
		mutex_unlock(&ts->mutex);

	} else { /*for some ic such as synaptic tcm need get data by interrupt*/
		tp_work_func(ts);
	}

	if (ts->sec_long_low_trigger) {
		enable_irq(ts->irq);
	}

exit:

#ifdef CONFIG_TOUCHIRQ_UPDATE_QOS

	if (PM_QOS_TOUCH_WAKEUP_VALUE == ts->pm_qos_value) {
		ts->pm_qos_value = PM_QOS_DEFAULT_VALUE;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,10,0)

		cpu_latency_qos_remove_request(&ts->pm_qos_req);
#else
		pm_qos_update_request(&ts->pm_qos_req, ts->pm_qos_value);
#endif
	}

#endif

        if (ts->ws) {
                __pm_relax(ts->ws);
        }

	return IRQ_HANDLED;
}

static ssize_t cap_vk_show(struct kobject *kobj, struct kobj_attribute *attr,
			   char *buf)
{
	struct button_map *button_map = NULL;
	struct touchpanel_data *ts = NULL;
	int i = 0;
	int retval = 0;
	int count = 0;

	for (i = 0; i < TP_SUPPORT_MAX;  i++) {
		ts = get_ts_data(i);

		if (!ts) {
			continue;
		}

		if (ts->tp_index == i) {
			button_map = &ts->button_map;
			retval =  snprintf(buf, PAGESIZE - count,
					   __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":%d:%d:%d:%d"
					   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE)   ":%d:%d:%d:%d"
					   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":%d:%d:%d:%d"
					   "\n", button_map->coord_menu.x, button_map->coord_menu.y, button_map->width_x,
					   button_map->height_y, \
					   button_map->coord_home.x, button_map->coord_home.y, button_map->width_x,
					   button_map->height_y, \
					   button_map->coord_back.x, button_map->coord_back.y, button_map->width_x,
					   button_map->height_y);

			if (retval < 0) {
				return retval;
			}

			buf += retval;
			count += retval;
		}
	}

	return count;
}

static struct kobj_attribute virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys."TPD_DEVICE,
		.mode = S_IRUGO,
	},
	.show = &cap_vk_show,
};

static struct attribute *properties_attrs[] = {
	&virtual_keys_attr.attr,
	NULL
};

static struct attribute_group properties_attr_group = {
	.attrs = properties_attrs,
};

/**
 * init_input_device - Using for register input device
 * @ts: touchpanel_data struct using for common driver
 *
 * we should using this function setting input report capbility && register input device
 * Returning zero(success) or negative errno(failed)
 */
static int init_input_device(struct touchpanel_data *ts)
{
	int ret = 0;
	struct kobject *vk_properties_kobj;
	static  bool board_properties = false;

	TP_BOOT_INFO(ts->tp_index, "%s is called\n", __func__);
	ts->input_dev = devm_input_allocate_device(ts->dev);

	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		TP_INFO(ts->tp_index, "Failed to allocate input device\n");
		return ret;
	}

	ts->kpd_input_dev  = devm_input_allocate_device(ts->dev);

	if (ts->kpd_input_dev == NULL) {
		ret = -ENOMEM;
		TP_INFO(ts->tp_index, "Failed to allocate key input device\n");
		return ret;
	}

	if (ts->face_detect_support) {
		ts->ps_input_dev  = devm_input_allocate_device(ts->dev);

		if (ts->ps_input_dev == NULL) {
			ret = -ENOMEM;
			TP_INFO(ts->tp_index, "Failed to allocate ps input device\n");
			return ret;
		}
		if (!ts->tp_index) {
			ts->ps_input_dev->name = TPD_DEVICE"_ps";
		} else {
			ts->ps_input_dev->name = TPD_DEVICE"_ps1";
		}
		set_bit(EV_MSC, ts->ps_input_dev->evbit);
		set_bit(MSC_RAW, ts->ps_input_dev->mscbit);
	}

	if (ts->pen_support) {
			ts->pen_input_dev = input_allocate_device();
			if (ts->pen_input_dev == NULL) {
				ret = -ENOMEM;
				TP_INFO(ts->tp_index, "Failed to allocate pen input device\n");
				return ret;
			}
			if (!ts->tp_index) {
				ts->pen_input_dev->name = TPD_DEVICE"_pen";
				ts->pen_input_dev->phys = TPD_DEVICE"_pen_phys_main";
			} else {
				ts->pen_input_dev->name = TPD_DEVICE"_pen1";
				ts->pen_input_dev->phys =  TPD_DEVICE"_pen_phys_secondary";
			}
			set_bit(EV_SYN, ts->pen_input_dev->evbit);
			set_bit(EV_ABS, ts->pen_input_dev->evbit);
			set_bit(EV_KEY, ts->pen_input_dev->evbit);
			set_bit(BTN_TOUCH, ts->pen_input_dev->keybit);
			set_bit(BTN_TOOL_PEN, ts->pen_input_dev->keybit);
			set_bit(BTN_STYLUS, ts->pen_input_dev->keybit);
			set_bit(BTN_STYLUS2, ts->pen_input_dev->keybit);
			set_bit(INPUT_PROP_DIRECT, ts->pen_input_dev->propbit);
	}

	if (!ts->tp_index) {
		ts->input_dev->name = TPD_DEVICE;
	} else {
		ts->input_dev->name = TPD_DEVICE"1";
	}
	ts->input_dev->id.bustype = ts->id.bustype;
	ts->input_dev->id.vendor = ts->id.vendor;
	ts->input_dev->id.product = ts->id.product;
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(ABS_MT_TOUCH_MAJOR, ts->input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, ts->input_dev->absbit);
	set_bit(ABS_MT_PRESSURE, ts->input_dev->absbit);
	set_bit(ABS_TOUCH_COST_TIME_KERNEL, ts->input_dev->absbit);
	/*set_bit(ABS_TOUCH_COST_TIME_ALGO, ts->input_dev->absbit);
	set_bit(ABS_TOUCH_COST_TIME_DAEMON, ts->input_dev->absbit);*/
	set_bit(ABS_MT_TOOL_TYPE, ts->input_dev->absbit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
	set_bit(BTN_TOOL_FINGER, ts->input_dev->keybit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);

	if (ts->palm_to_sleep_support)
		set_bit(KEY_SLEEP, ts->input_dev->keybit);

	if (ts->black_gesture_support) {
		set_bit(KEY_F4, ts->input_dev->keybit);
#ifdef CONFIG_OPLUS_TP_APK
		set_bit(KEY_POWER, ts->input_dev->keybit);
#endif /*end of CONFIG_OPLUS_TP_APK*/
	}
	if (!ts->tp_index) {
		ts->kpd_input_dev->name = TPD_DEVICE"_kpd";
	} else {
		ts->kpd_input_dev->name = TPD_DEVICE"_kpd1";
	}
	set_bit(EV_KEY, ts->kpd_input_dev->evbit);
	set_bit(EV_SYN, ts->kpd_input_dev->evbit);

	switch (ts->vk_type) {
	case TYPE_PROPERTIES : {
		if (!board_properties) {
			/*If more ic support more key, but have only one path*/
			TP_INFO(ts->tp_index, "Type 1: using board_properties\n");
			vk_properties_kobj = kobject_create_and_add("board_properties", NULL);

			if (vk_properties_kobj) {
				ret = sysfs_create_group(vk_properties_kobj, &properties_attr_group);
			}

			if (!vk_properties_kobj || ret) {
				TP_INFO(ts->tp_index, "failed to create board_properties\n");
			}

			board_properties = true;
		}

		break;
	}

	case TYPE_AREA_SEPRATE: {
		TPD_DEBUG("Type 2:using same IC (button zone &&  touch zone are seprate)\n");

		if (CHK_BIT(ts->vk_bitmap, BIT_MENU)) {
			set_bit(KEY_MENU, ts->kpd_input_dev->keybit);
		}

		if (CHK_BIT(ts->vk_bitmap, BIT_HOME)) {
			set_bit(KEY_HOMEPAGE, ts->kpd_input_dev->keybit);
		}

		if (CHK_BIT(ts->vk_bitmap, BIT_BACK)) {
			set_bit(KEY_BACK, ts->kpd_input_dev->keybit);
		}

		break;
	}

	default :
		break;
	}

	ret = input_mt_init_slots(ts->input_dev, ts->max_num, 0);
	if (!ret)
		TP_BOOT_INFO(ts->tp_index, "%s: input_mt_init_slots return %d\n", __func__, ret);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	//input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0,
			     ts->resolution_info.max_x - 1, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0,
			     ts->resolution_info.max_y - 1, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_TOUCH_COST_TIME_KERNEL, 0, MAX_TOUCH_COST_TIME, 0, 0);
	/*input_set_abs_params(ts->input_dev, ABS_TOUCH_COST_TIME_ALGO, 0, MAX_TOUCH_COST_TIME, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_TOUCH_COST_TIME_DAEMON, 0, MAX_TOUCH_COST_TIME, 0, 0);*/
	input_set_drvdata(ts->input_dev, ts);
	input_set_drvdata(ts->kpd_input_dev, ts);

	if (input_register_device(ts->input_dev)) {
		TP_INFO(ts->tp_index, "%s: Failed to register input device\n", __func__);
		input_free_device(ts->input_dev);
		return -1;
	}

	if (input_register_device(ts->kpd_input_dev)) {
		TP_INFO(ts->tp_index, "%s: Failed to register key input device\n", __func__);
		input_free_device(ts->kpd_input_dev);
		return -1;
	}

	if (ts->face_detect_support) {
		if (input_register_device(ts->ps_input_dev)) {
			TP_INFO(ts->tp_index, "%s: Failed to register ps input device\n", __func__);
			input_free_device(ts->ps_input_dev);
			return -1;
		}
	}

	if(ts->pen_support) {
			input_set_abs_params(ts->pen_input_dev, ABS_X, 0, ts->pen_config.max_x - 1, 0, 0);
			input_set_abs_params(ts->pen_input_dev, ABS_Y, 0, ts->pen_config.max_y - 1, 0, 0);
			input_set_abs_params(ts->pen_input_dev, ABS_PRESSURE, 0, ts->pen_config.max_pressure - 1, 0, 0);
			input_set_abs_params(ts->pen_input_dev, ABS_DISTANCE, 0, 1, 0, 0);
			input_set_abs_params(ts->pen_input_dev, ABS_TILT_X, -(ts->pen_config.tilt_x_max), ts->pen_config.tilt_x_max, 0, 0);
			input_set_abs_params(ts->pen_input_dev, ABS_TILT_Y, -(ts->pen_config.tilt_y_max), ts->pen_config.tilt_y_max, 0, 0);
			if (input_register_device(ts->pen_input_dev)) {
				TP_INFO(ts->tp_index, "%s: Failed to register pen input device\n", __func__);
				input_free_device(ts->pen_input_dev);
				return -1;
			}
	}

	return 0;
}

static struct device_node* is_support_child_node (struct device *dev,  struct touchpanel_data *ts)
{
	struct device_node *child_node = NULL;
	char *chip_name = NULL;
	char *panel_node = kzalloc(MAX_NODE_DATA_LENGTH, GFP_KERNEL|GFP_DMA);
	struct device_node *np = NULL;
	if (!ts || !dev) {
	    kfree(panel_node);
	    return child_node;
	}

	np = dev->of_node;
	if (ts->chip_index > 0 && ts->chip_index <= ts->panel_data.chip_num) {
		chip_name = ts->panel_data.chip_name[ts->chip_index];
	} else {
		chip_name = ts->panel_data.chip_name[0];
	}
	snprintf(panel_node, MAX_NODE_DATA_LENGTH, "%s_PANEL%d", chip_name, ts->panel_data.tp_type);
	TP_INFO(ts->tp_index, "%s: panel_node = %s.\n", __func__, panel_node);
	child_node = of_get_child_by_name(np, panel_node);

	if (!child_node) {
		TP_INFO(ts->tp_index, "child_node not defined.\n");
	}

	kfree(panel_node);
	return child_node;
}

/**
 * init_panel_config - parse panel dts, get resource defined in Dts
 * If you want to configure multiple panels, you can combine them "chip_name"_"panle_name"
 */
static void init_panel_config(struct device *dev, struct touchpanel_data *ts)
{
	struct device_node *chip_np;
	int temp_array[8];
	int tx_rx_num[2];
	int rc = 0;
	int i = 0;
	int val = 0;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
	int ret = 0;
	int buf[3] = {0, 0, 0};
#endif

	chip_np = is_support_child_node(dev, ts);
	if (!chip_np) {
		return;
	}

	/* irq gpio*/
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
	ts->hw_res.irq_gpio = of_get_named_gpio(chip_np, "irq-gpio", 0);
	ret = of_property_read_u32_array(chip_np, "irq-gpio", buf, 3);
	if (ret) {
		TP_INFO(ts->tp_index, "irq flag use default(0x2008)\n");
		ts->irq_flags = 0x2008;
	} else {
		ts->irq_flags = buf[2];
		TP_INFO(ts->tp_index, "irq flag use 0x%x.\n", ts->irq_flags);
	}
#else
	ts->hw_res.irq_gpio = of_get_named_gpio_flags(chip_np, "irq-gpio", 0,
			      &(ts->irq_flags));
#endif
	if (gpio_is_valid(ts->hw_res.irq_gpio)) {
		rc = devm_gpio_request(dev, ts->hw_res.irq_gpio, "tp_irq_gpio");
		if (rc) {
			TP_INFO(ts->tp_index, "unable to request gpio [%d]\n", ts->hw_res.irq_gpio);
		}
	} else {
		TP_BOOT_INFO(ts->tp_index, "irq-gpio not specified in dts\n");
	}

	/* reset gpio*/
	ts->hw_res.reset_gpio = of_get_named_gpio(chip_np, "reset-gpio", 0);
	if (gpio_is_valid(ts->hw_res.reset_gpio)) {
		rc = devm_gpio_request(dev, ts->hw_res.reset_gpio, "reset-gpio");
		if (rc) {
			TP_INFO(ts->tp_index, "unable to request gpio [%d]\n", ts->hw_res.reset_gpio);
		}
	} else {
		TP_BOOT_INFO(ts->tp_index, "ts->reset-gpio not specified\n");
	}

	TP_BOOT_INFO(ts->tp_index, "%s : irq_gpio = %d, irq_flags = 0x%x, reset_gpio = %d\n",
		 __func__, ts->hw_res.irq_gpio, ts->irq_flags, ts->hw_res.reset_gpio);
		/* resolution info*/
	rc = of_property_read_u32(chip_np, "touchpanel,max-num-support", &ts->max_num);

	if (rc) {
		TP_BOOT_INFO(ts->tp_index, "ts->max_num not specified\n");
		ts->max_num = 10;
	}

	rc = of_property_read_u32_array(chip_np, "touchpanel,tx-rx-num", tx_rx_num, 2);

	if (rc) {
		TP_BOOT_INFO(ts->tp_index, "tx-rx-num not set\n");
		ts->hw_res.tx_num = 0;
		ts->hw_res.rx_num = 0;

	} else {
		ts->hw_res.tx_num = tx_rx_num[0];
		ts->hw_res.rx_num = tx_rx_num[1];
	}

	TP_BOOT_INFO(ts->tp_index, "tx_num = %d, rx_num = %d \n", ts->hw_res.tx_num, ts->hw_res.rx_num);

	rc = of_property_read_u32_array(chip_np, "touchpanel,display-coords", temp_array, 2);

	if (rc) {
		TP_BOOT_INFO(ts->tp_index, "Lcd size not set\n");
		ts->resolution_info.LCD_WIDTH = 0;
		ts->resolution_info.LCD_HEIGHT = 0;

	} else {
		ts->resolution_info.LCD_WIDTH = temp_array[0];
		ts->resolution_info.LCD_HEIGHT = temp_array[1];
	}

	rc = of_property_read_u32_array(chip_np, "touchpanel,panel-coords", temp_array, 2);

	if (rc) {
		ts->resolution_info.max_x = 0;
		ts->resolution_info.max_y = 0;

	} else {
		ts->resolution_info.max_x = temp_array[0];
		ts->resolution_info.max_y = temp_array[1];
	}

	rc = of_property_read_u32_array(chip_np, "touchpanel,touchmajor-limit", temp_array,
					2);

	if (rc) {
		ts->touch_major_limit.width_range = 0;
		ts->touch_major_limit.height_range = 54;    /*set default value*/

	} else {
		ts->touch_major_limit.width_range = temp_array[0];
		ts->touch_major_limit.height_range = temp_array[1];
	}

	TP_BOOT_INFO(ts->tp_index, "LCD_WIDTH = %d, LCD_HEIGHT = %d, max_x = %d, max_y = %d, limit_witdh = %d, limit_height = %d\n",
		 ts->resolution_info.LCD_WIDTH, ts->resolution_info.LCD_HEIGHT,
		 ts->resolution_info.max_x, ts->resolution_info.max_y, \
		 ts->touch_major_limit.width_range, ts->touch_major_limit.height_range);
	rc = of_property_read_u32_array(chip_np, "touchpanel,smooth-level", temp_array, SMOOTH_LEVEL_NUM);
	if (rc) {
		TP_BOOT_INFO(ts->tp_index, "smooth_level_array not specified %d\n", rc);
	} else {
		ts->smooth_level_array_support = true;
		for (i=0; i < SMOOTH_LEVEL_NUM; i++) {
			ts->smooth_level_array[i] = temp_array[i];
		}

		rc = of_property_read_u32_array(chip_np,
						"touchpanel,smooth-level-charging",
						temp_array,
						SMOOTH_LEVEL_NUM);
		if (rc) {
			TP_BOOT_INFO(ts->tp_index, "smooth_level_charging_array not specified %d\n", rc);
			for (i=0; i < SMOOTH_LEVEL_NUM; i++) {
				ts->smooth_level_charging_array[i] = ts->smooth_level_array[i];
			}
		} else {
			for (i=0; i < SMOOTH_LEVEL_NUM; i++) {
				ts->smooth_level_charging_array[i] = temp_array[i];
			}
		}
		ts->smooth_level_used_array = (u32 *)&(ts->smooth_level_array);
	}

	rc = of_property_read_u32_array(chip_np, "touchpanel,sensitive-level", temp_array, SENSITIVE_LEVEL_NUM);
	if (rc) {
		TP_BOOT_INFO(ts->tp_index, "sensitive_level_array not specified %d\n", rc);
	} else {
		ts->sensitive_level_array_support = true;
		for (i=0; i < SENSITIVE_LEVEL_NUM; i++) {
			ts->sensitive_level_array[i] = temp_array[i];
		}

		rc = of_property_read_u32_array(chip_np,
						"touchpanel,sensitive-level-charging",
						temp_array,
						SENSITIVE_LEVEL_NUM);
		if (rc) {
			TP_BOOT_INFO(ts->tp_index, "sensitive_charging_array not specified %d\n", rc);
			for (i=0; i < SENSITIVE_LEVEL_NUM; i++) {
				ts->sensitive_level_charging_array[i] = ts->sensitive_level_array[i];
			}
		} else {
			for (i=0; i < SENSITIVE_LEVEL_NUM; i++) {
				ts->sensitive_level_charging_array[i] = temp_array[i];
			}
		}
		ts->sensitive_level_used_array = (u32 *)&(ts->sensitive_level_array);
	}

	rc = of_property_read_u32(chip_np, "touchpanel,single-optimized-time", &ts->single_optimized_time);
	if (rc) {
		TP_BOOT_INFO(ts->tp_index, "ts->single_optimized_time not specified\n");
		ts->single_optimized_time = 0;
		ts->optimized_show_support = false;
	} else {
		ts->total_operate_times = 0;
		ts->optimized_show_support = true;
	}

	rc = of_property_read_u32(chip_np, "touchpanel,high-frame-rate-time", &ts->high_frame_rate_time);
	if (rc) {
		TP_BOOT_INFO(ts->tp_index, "ts->high_frame_rate_time not specified or support\n");
		ts->high_frame_rate_time = 0;
		ts->high_frame_rate_support = false;
	} else {
		ts->high_frame_rate_support = true;
	}

	/*set disable suspend irq handler parameter, for of_property_read_bool return 1 when success and return 0 when item is not exist*/
	ts->disable_suspend_irq_handler = of_property_read_bool(chip_np, "disable_suspend_irq_handler_support");
	ts->tp_data_record_support = of_property_read_bool(chip_np, "tp_data_record_support");

	/* interrupt mode*/
	ts->int_mode = BANNABLE;
	rc = of_property_read_u32(chip_np, "touchpanel,int-mode", &val);

	if (rc) {
		TP_BOOT_INFO(ts->tp_index, "int-mode not specified\n");

	} else {
		if (val < INTERRUPT_MODE_MAX) {
			ts->int_mode = val;
		}
	}
}

/**
 * init_parse_dts - parse dts, get resource defined in Dts
 * @dev: i2c_client->dev using to get device tree
 * @ts: touchpanel_data, using for common driver
 *
 * If there is any Resource needed by chip_data, we can add a call-back func in this function
 * Do not care the result : Returning void type
 */
static int init_parse_dts(struct device *dev, struct touchpanel_data *ts)
{
	int rc, ret = 0;
	struct device_node *np;
	int temp_array[8];
	int tx_rx_num[2];
	int val = 0;
	int i = 0;
	int size = 0;
	char data_buf[32];
	np = dev->of_node;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
	int buf[3] = {0, 0, 0};
#endif

	/* tp_index*/
	rc = of_property_read_u32(np, "touchpanel,tp-index", &ts->tp_index);

	if (rc) {
		TPD_BOOT_INFO("ts->tp_index not specified\n");
		ts->tp_index = 0;

	} else {
		if (ts->tp_index >= TP_SUPPORT_MAX) {
			TP_INFO(ts->tp_index, "ts->tp_index is big than %d\n", TP_SUPPORT_MAX);
			ts->tp_index = 0;
		}
	}

	cur_tp_index = ts->tp_index;
	TPD_BOOT_INFO("ts->tp_index is %d\n",  cur_tp_index);

	ts->register_is_16bit       = of_property_read_bool(np, "register-is-16bit");
	ts->esd_handle_support      = of_property_read_bool(np, "esd_handle_support");
	ts->fw_edge_limit_support   = of_property_read_bool(np,
				      "fw_edge_limit_support");
	ts->report_rate_v2_support      = of_property_read_bool(np, "report_rate_v2_support");
	ts->charger_pump_support    = of_property_read_bool(np, "charger_pump_support");
	ts->wireless_charger_support = of_property_read_bool(np,
				       "wireless_charger_support");
	ts->headset_pump_support    = of_property_read_bool(np, "headset_pump_support");
	ts->black_gesture_support   = of_property_read_bool(np,
				      "black_gesture_support");

	ts->black_gesture_indep_support = of_property_read_bool(np,
								"black_gesture_indep_support");

	ts->gesture_test_support    = of_property_read_bool(np,
				      "black_gesture_test_support");
	ts->fw_update_app_support   = of_property_read_bool(np,
				      "fw_update_app_support");
	ts->game_switch_support     = of_property_read_bool(np, "game_switch_support");
	ts->game_enable_in_tddi_support     = of_property_read_bool(np, "game_enable_in_tddi_support");
	ts->glove_mode_support      = of_property_read_bool(np, "glove_mode_support");
	ts->glove_mode_v2_support      = of_property_read_bool(np, "glove_mode_v2_support");
	ts->rainstorm_mode_v2_support      = of_property_read_bool(np, "rainstorm_mode_v2_support");
	ts->leather_cover_mode_support      = of_property_read_bool(np, "leather_cover_mode_support");
	ts->is_noflash_ic           = of_property_read_bool(np, "noflash_support");
	ts->face_detect_support     = of_property_read_bool(np, "face_detect_support");
	ts->sec_long_low_trigger     = of_property_read_bool(np,
				       "sec_long_low_trigger");
	ts->health_monitor_support = of_property_read_bool(np,
				     "health_monitor_support");
	ts->lcd_trigger_load_tp_fw_support = of_property_read_bool(np,
					     "lcd_trigger_load_tp_fw_support");
	ts->fingerprint_underscreen_support = of_property_read_bool(np,
					      "fingerprint_underscreen_support");
	ts->fingerprint_not_report_in_suspend = of_property_read_bool(np,
					      "fingerprint_not_report_in_suspend");
	ts->fingerprint_error_report_support = of_property_read_bool(np,
					      "fingerprint_error_report_support");
	ts->suspend_gesture_cfg   = of_property_read_bool(np, "suspend_gesture_cfg");
	ts->auto_test_force_pass_support = of_property_read_bool(np,
					   "auto_test_force_pass_support");
	ts->freq_hop_simulate_support = of_property_read_bool(np,
					"freq_hop_simulate_support");
	ts->irq_trigger_hdl_support = of_property_read_bool(np,
				      "irq_trigger_hdl_support");
	ts->noise_modetest_support = of_property_read_bool(np,
				     "noise_modetest_support");
	ts->fw_update_in_probe_with_headfile = of_property_read_bool(np,
					       "fw_update_in_probe_with_headfile");
	ts->kernel_grip_support     = of_property_read_bool(np, "kernel_grip_support");
	ts->grip_no_driver_support = of_property_read_bool(np, "grip_no_driver_support");
	ts->report_rate_white_list_support = of_property_read_bool(np,
					     "report_rate_white_list_support");

	ts->lcd_tp_refresh_support = of_property_read_bool(np, "lcd_tp_refresh_support");
	ts->enable_point_auto_change = of_property_read_bool(np, "enable_point_auto_change");
	ts->temperature_detect_support = of_property_read_bool(np, "temperature_detect_support");
	ts->temperature_detect_shellback_support = of_property_read_bool(np, "temperature_detect_shellback_support");
	ts->auto_test_need_cal_support = of_property_read_bool(np, "auto_test_need_cal_support");
	ts->snr_read_support = of_property_read_bool(np, "snr_read_support");
	ts->major_rate_limit_support = of_property_read_bool(np, "major_rate_limit_support");
	ts->touchz_to_pressure_support = of_property_read_bool(np, "touchz_to_pressure_support");
	ts->palm_to_sleep_support = of_property_read_bool(np, "palm_to_sleep_support");
	ts->input_timestamp_in_top_irq_support = of_property_read_bool(np, "input_timestamp_in_top_irq_support");
	ts->waterproof_support = of_property_read_bool(np, "waterproof_support");
	if (ts->waterproof_support) {
		ts->waterproof = 1;
	}
	ts->tp_data_record_support = of_property_read_bool(np, "tp_data_record_support");
	ts->long_strip_abnormal_detect_support = of_property_read_bool(np, "long_strip_abnormal_detect_support");
	ts->skip_reinit_device_support = of_property_read_bool(np, "skip_reinit_device_support");
	ts->suspend_work_support = of_property_read_bool(np, "suspend_work_support");
	ts->fp_disable_after_resume = of_property_read_bool(np, "fp_disable_after_resume");
	ts->edge_pull_out_support = of_property_read_bool(np, "edge_pull_out_support");
	ts->diaphragm_touch_support = of_property_read_bool(np, "diaphragm_touch_support");
	ts->screenshot_not_reset_support = of_property_read_bool(np, "screenshot_not_reset_support");
	ts->fp_grip_support = of_property_read_bool(np, "fp_grip_support");
	ts->disable_touch_event_support = of_property_read_bool(np, "disable_touch_event_support");
	ts->lpwg_fw_support = of_property_read_bool(np, "lpwg_fw_support");
	ts->tp_scene_para_switch_support = of_property_read_bool(np, "tp_scene_para_switch_support");
	ts->fp_unlock_status_support = of_property_read_bool(np, "fp_unlock_status_support");
	ts->idle_freq_support = of_property_read_bool(np, "idle_freq_support");

#ifdef CONFIG_TOUCHPANEL_TRUSTED_TOUCH
	ts->trusted_touch_support = of_property_read_bool(np, "trusted_touch_support");
#else
	ts->trusted_touch_support = false;
#endif

	ts->exception_upload_support = of_property_read_bool(np,
				     "exception_upload_support");
	ts->sportify_aod_gesture_support = of_property_read_bool(np,
						 "sportify_aod_gesture_support");
	if (!ts->sportify_aod_gesture_support) {
		TP_INFO(ts->tp_index, "not support sportify_aod_gesture\n");
	}
	ts->incell_aod_gesture_support = of_property_read_bool(np,
						 "incell_aod_gesture_support");
	ts->regulator_count_not_support = of_property_read_bool(np, "regulator_count_not_support");

	ts->force_bus_ready_support = of_property_read_bool(np, "force_bus_ready_support");

	rc = of_property_read_u32(np, "vcc_1v8_volt", &ts->hw_res.vddi_volt);
	if (rc < 0) {
		ts->hw_res.vddi_volt = 0;
		TP_BOOT_INFO(ts->tp_index, "vdd_1v8_volt not defined\n");
	}

	ts->fpga_spi_agg_support = of_property_read_bool(np, "fpga_spi_agg_support");
	ts->pen_support = of_property_read_bool(np, "pen_support");
	ts->pen_support_opp = of_property_read_bool(np, "pen_support_opp");
	ts->no_need_osctest = of_property_read_bool(np, "no_need_osctest");
	ts->bus_ready_check_support = of_property_read_bool(np, "bus_ready_check_support");
	TP_INFO(ts->tp_index, "bus_ready_check_support is %d\n", ts->bus_ready_check_support);
	ts->aiunit_game_info_support = of_property_read_bool(np, "aiunit_game_info_support");

	ts->tp_lcd_suspend_in_lp_support = of_property_read_bool(np, "tp_lcd_suspend_in_lp_support");
	TP_INFO(ts->tp_index, "tp_lcd_suspend_in_lp_support is %d\n", ts->tp_lcd_suspend_in_lp_support);

	rc = of_property_read_u32(np, "vdd_2v8_volt", &ts->hw_res.vdd_volt);
	if (rc < 0) {
		ts->hw_res.vdd_volt = 0;
		TP_BOOT_INFO(ts->tp_index, "vdd_2v8_volt not defined\n");
	}

	/* irq gpio*/
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
	ts->hw_res.irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	ret = of_property_read_u32_array(np, "irq-gpio", buf, 3);
	if (ret) {
		TP_INFO(ts->tp_index, "irq flag use default(0x2008)\n");
		ts->irq_flags = 0x2008;
	} else {
		ts->irq_flags = buf[2];
		TP_INFO(ts->tp_index, "irq flag use 0x%x.\n", ts->irq_flags);
	}
#else
	ts->hw_res.irq_gpio = of_get_named_gpio_flags(np, "irq-gpio", 0,
			      &(ts->irq_flags));
#endif

	if (gpio_is_valid(ts->hw_res.irq_gpio)) {
		rc = devm_gpio_request(dev, ts->hw_res.irq_gpio, "tp_irq_gpio");

		if (rc) {
			TP_INFO(ts->tp_index, "unable to request gpio [%d]\n", ts->hw_res.irq_gpio);
		}

	} else {
		TP_INFO(ts->tp_index, "irq-gpio not specified in dts\n");
	}

#ifdef CONFIG_TOUCHPANEL_TRUSTED_TOUCH
	/* irq_tui_flags should be the opposite of the irq_flags configuration */
	rc = of_property_read_u32(np, "touchpanel,irq_tui_flags", &ts->irq_tui_flags);
	if (rc) {
		TP_INFO(ts->tp_index, "ts->max_num not specified\n");
	}
	TP_INFO(ts->tp_index, "%s:irq_tui_flags = 0x%x\n", __func__, ts->irq_tui_flags);
#endif

	/* reset gpio*/
	ts->hw_res.reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (gpio_is_valid(ts->hw_res.reset_gpio)) {
		rc = devm_gpio_request(dev, ts->hw_res.reset_gpio, "reset-gpio");

		if (rc) {
			TP_INFO(ts->tp_index, "unable to request gpio [%d]\n", ts->hw_res.reset_gpio);
		}

	} else {
		TP_INFO(ts->tp_index, "ts->reset-gpio not specified\n");
	}

	TP_BOOT_INFO(ts->tp_index, "%s : irq_gpio = %d, irq_flags = 0x%x, reset_gpio = %d\n",
		 __func__, ts->hw_res.irq_gpio, ts->irq_flags, ts->hw_res.reset_gpio);

	/* spi cs gpio */
	ts->hw_res.cs_gpio = of_get_named_gpio(np, "cs-gpio", 0);
	if (gpio_is_valid(ts->hw_res.cs_gpio)) {
		rc = gpio_request(ts->hw_res.cs_gpio, "cs-gpio");
		if (rc)
			TP_INFO(ts->tp_index, "unable to request gpio [%d]\n", ts->hw_res.cs_gpio);
		else
			TP_BOOT_INFO(ts->tp_index, "%s : irq_gpio = %d\n", __func__, ts->hw_res.cs_gpio);
	} else {
		TP_BOOT_INFO(ts->tp_index, "ts->cs-gpio not specified\n");
	}
	ts->hw_res.pinctrl = devm_pinctrl_get(dev);

	if (IS_ERR_OR_NULL(ts->hw_res.pinctrl)) {
		TP_INFO(ts->tp_index, "Getting pinctrl handle failed");
	} else {
		ts->hw_res.pin_set_high = pinctrl_lookup_state(ts->hw_res.pinctrl,
					  "pin_set_high");

		if (IS_ERR_OR_NULL(ts->hw_res.pin_set_high)) {
			TP_BOOT_INFO(ts->tp_index, "Failed to get the high state pinctrl handle\n");
		}


		ts->hw_res.pin_set_low = pinctrl_lookup_state(ts->hw_res.pinctrl,
					 "pin_set_low");

		if (IS_ERR_OR_NULL(ts->hw_res.pin_set_low)) {
			TP_BOOT_INFO(ts->tp_index, " Failed to get the low state pinctrl handle\n");
		}

		ts->hw_res.pin_cs_high = pinctrl_lookup_state(ts->hw_res.pinctrl,
					  "pin_cs_high");

		if (IS_ERR_OR_NULL(ts->hw_res.pin_cs_high)) {
			TP_BOOT_INFO(ts->tp_index, "Failed to get the cs-gpio high state pinctrl\n");
		}


		ts->hw_res.pin_cs_low = pinctrl_lookup_state(ts->hw_res.pinctrl,
					 "pin_cs_low");

		if (IS_ERR_OR_NULL(ts->hw_res.pin_cs_low)) {
			TP_BOOT_INFO(ts->tp_index, " Failed to get the cs-gpio low state pinctrl \n");
		}

		ts->hw_res.pin_set_nopull = pinctrl_lookup_state(ts->hw_res.pinctrl,
					    "pin_set_nopull");

		if (IS_ERR_OR_NULL(ts->hw_res.pin_set_nopull)) {
			TP_BOOT_INFO(ts->tp_index, "Failed to get the input state pinctrl handle\n");
		}

		/* active spi mode */
		ts->hw_res.pin_spi_mode_active = pinctrl_lookup_state(ts->hw_res.pinctrl, PINCTRL_STATE_SPI_ACTIVE);
		if (IS_ERR_OR_NULL(ts->hw_res.pin_spi_mode_active)) {
			rc = PTR_ERR(ts->hw_res.pin_spi_mode_active);
			TPD_BOOT_INFO("Failed to get pinctrl state:%s, r:%d",
					PINCTRL_STATE_SPI_ACTIVE, rc);
			ts->hw_res.pin_spi_mode_active = NULL;
		}

		/* int active state */
		ts->hw_res.pin_int_sta_active = pinctrl_lookup_state(ts->hw_res.pinctrl, PINCTRL_STATE_INT_ACTIVE);
		if (IS_ERR_OR_NULL(ts->hw_res.pin_int_sta_active)) {
			rc = PTR_ERR(ts->hw_res.pin_int_sta_active);
			TPD_BOOT_INFO("Failed to get pinctrl state:%s, r:%d",
					PINCTRL_STATE_INT_ACTIVE, rc);
			ts->hw_res.pin_int_sta_active = NULL;
		}

		/* rst active state */
		ts->hw_res.pin_rst_sta_active = pinctrl_lookup_state(ts->hw_res.pinctrl, PINCTRL_STATE_RST_ACTIVE);
		if (IS_ERR_OR_NULL(ts->hw_res.pin_rst_sta_active)) {
			rc = PTR_ERR(ts->hw_res.pin_rst_sta_active);
			TPD_BOOT_INFO("Failed to get pinctrl state:%s, r:%d",
					PINCTRL_STATE_RST_ACTIVE, rc);
			ts->hw_res.pin_rst_sta_active = NULL;
		}
		TPD_BOOT_INFO("success get avtive pinctrl state");

		/* suspend spi mode */
		ts->hw_res.pin_spi_mode_suspend = pinctrl_lookup_state(ts->hw_res.pinctrl, PINCTRL_STATE_SPI_SUSPEND);
		if (IS_ERR_OR_NULL(ts->hw_res.pin_spi_mode_suspend)) {
			rc = PTR_ERR(ts->hw_res.pin_spi_mode_suspend);
			TPD_BOOT_INFO("Failed to get pinctrl state:%s, r:%d",
					PINCTRL_STATE_SPI_SUSPEND, rc);
			ts->hw_res.pin_spi_mode_suspend = NULL;
		}

		/* int suspend state */
		ts->hw_res.pin_int_sta_suspend = pinctrl_lookup_state(ts->hw_res.pinctrl, PINCTRL_STATE_INT_SUSPEND);
		if (IS_ERR_OR_NULL(ts->hw_res.pin_int_sta_suspend)) {
			rc = PTR_ERR(ts->hw_res.pin_int_sta_suspend);
			TPD_BOOT_INFO("Failed to get pinctrl state:%s, r:%d",
					PINCTRL_STATE_INT_SUSPEND, rc);
			ts->hw_res.pin_int_sta_suspend = NULL;
		}

		/* suspend spi mode */
		ts->hw_res.pin_rst_sta_suspend = pinctrl_lookup_state(ts->hw_res.pinctrl, PINCTRL_STATE_RST_SUSPEND);
		if (IS_ERR_OR_NULL(ts->hw_res.pin_rst_sta_suspend)) {
			rc = PTR_ERR(ts->hw_res.pin_rst_sta_suspend);
			TPD_BOOT_INFO("Failed to get pinctrl state:%s, r:%d",
					PINCTRL_STATE_RST_SUSPEND, rc);
			ts->hw_res.pin_rst_sta_suspend = NULL;
		}
		TPD_BOOT_INFO("success get suspend pinctrl state");
	}

	ts->hw_res.enable_avdd_gpio = of_get_named_gpio(np, "enable2v8_gpio", 0);

	if (ts->hw_res.enable_avdd_gpio < 0) {
		TP_BOOT_INFO(ts->tp_index, "ts->hw_res.enable2v8_gpio not specified\n");

	} else {
		if (gpio_is_valid(ts->hw_res.enable_avdd_gpio)) {
			rc = devm_gpio_request(dev, ts->hw_res.enable_avdd_gpio, "vdd2v8-gpio");

			if (rc) {
				TP_INFO(ts->tp_index, "unable to request gpio [%d] %d\n", ts->hw_res.enable_avdd_gpio, rc);
			}
		}
	}

	ts->hw_res.enable_vddi_gpio = of_get_named_gpio(np, "enable1v8_gpio", 0);

	if (ts->hw_res.enable_vddi_gpio < 0) {
		TP_BOOT_INFO(ts->tp_index, "ts->hw_res.enable1v8_gpio not specified\n");

	} else {
		if (gpio_is_valid(ts->hw_res.enable_vddi_gpio)) {
			rc = devm_gpio_request(dev, ts->hw_res.enable_vddi_gpio, "vcc1v8-gpio");

			if (rc) {
				TP_INFO(ts->tp_index, "unable to request gpio [%d], %d\n", ts->hw_res.enable_vddi_gpio, rc);
			}
		}
	}

	/* interrupt mode*/
	ts->int_mode = BANNABLE;
	rc = of_property_read_u32(np, "touchpanel,int-mode", &val);

	if (rc) {
		TP_BOOT_INFO(ts->tp_index, "int-mode not specified\n");

	} else {
		if (val < INTERRUPT_MODE_MAX) {
			ts->int_mode = val;
		}
	}

	rc = of_property_read_u32(np, "project_id", &ts->panel_data.project_id);

	if (rc) {
		TP_BOOT_INFO(ts->tp_index, "project_id not specified\n");
	}

	rc = of_property_count_u32_elems(np, "platform_support_project");
	ts->panel_data.project_num = rc;

	if (!rc) {
		TP_BOOT_INFO(ts->tp_index, "project not specified\n");
	}

	if (ts->panel_data.project_num > 0) {
		rc = of_property_read_u32_array(np, "platform_support_project",
						ts->panel_data.platform_support_project, ts->panel_data.project_num);

		if (rc) {
			TP_INFO(ts->tp_index, "platform_support_project not specified");
			return -1;
		}

		rc = of_property_read_u32_array(np, "platform_support_project_dir",
						ts->panel_data.platform_support_project_dir, ts->panel_data.project_num);

		if (rc) {
			TP_INFO(ts->tp_index, "platform_support_project_dir not specified");
			return -1;
		}

	} else {
		TP_BOOT_INFO(ts->tp_index, "project and project not specified in dts, please update dts");
	}

	/*parse chip name*/
	rc = of_property_read_u32(np, "chip-num", &ts->panel_data.chip_num);

	if (rc)  {
		TP_BOOT_INFO(ts->tp_index, "panel_type not specified, need to default 1");
		ts->panel_data.chip_num = 1;
	}

	TP_BOOT_INFO(ts->tp_index, "find %d num chip in dts", ts->panel_data.chip_num);

	for (i = 0; i < ts->panel_data.chip_num; i++) {
		/*ts->panel_data.chip_name[i] = devm_kzalloc(dev, 100, GFP_KERNEL);

		if (ts->panel_data.chip_name[i] == NULL) {
			TPD_INFO("panel_data.chip_name kzalloc error\n");
			devm_kfree(dev, ts->panel_data.chip_name[i]);
			goto dts_match_error;
		}*/

		rc = of_property_read_string_index(np, "chip-name", i,
						   (const char **)&ts->panel_data.chip_name[i]);
		TP_BOOT_INFO(ts->tp_index, "panel_data.chip_name = %s\n", ts->panel_data.chip_name[i]);

		if (rc) {
			TP_BOOT_INFO(ts->tp_index, "chip-name not specified");
		}
	}

	/*parse panel type and commandline*/
	rc = of_property_count_u32_elems(np, "panel_type");

	if (!rc) {
		TP_BOOT_INFO(ts->tp_index, "panel_type not specified\n");

	} else if (rc) {
		TP_BOOT_INFO(ts->tp_index, "now has %d num panel in dts\n", rc);
		ts->panel_data.panel_num = rc;
	}
	/*tp max report rate limit*/
	rc = of_property_read_u32(np, "max_report_rate_limit", &ts->max_report_rate_limit);

	if (rc) {
		TP_BOOT_INFO(ts->tp_index, "max report rate not limit\n");
		ts->max_report_rate_limit = 0;
	}

	if (ts->panel_data.panel_num > 0) {
		rc = of_property_read_u32_array(np, "panel_type", ts->panel_data.panel_type,
						ts->panel_data.panel_num);

		if (rc) {
			TP_INFO(ts->tp_index, "panel_type not specified");
			goto dts_match_error;
		}
	}

	for (i = 0; i < ts->panel_data.panel_num; i++) {
		/*ts->panel_data.platform_support_commandline[i] = devm_kzalloc(dev, 100,
				GFP_KERNEL);

		if (ts->panel_data.platform_support_commandline[i] == NULL) {
			TPD_INFO("panel_data.platform_support_commandline kzalloc error\n");
			devm_kfree(dev, ts->panel_data.platform_support_commandline[i]);
			goto dts_match_error;
		}*/

		rc = of_property_read_string_index(np, "platform_support_project_commandline",
						   i,
						   (const char **)&ts->panel_data.platform_support_commandline[i]);

		if (rc) {
			TP_INFO(ts->tp_index, "platform_support_project_commandline not specified");
			goto dts_match_error;
		}

		//ts->panel_data.firmware_name[i] = devm_kzalloc(dev, 25, GFP_KERNEL);
		rc = of_property_read_string_index(np, "firmware_name", i,
						   (const char **)&ts->panel_data.firmware_name[i]);

		if (rc) {
			TP_BOOT_INFO(ts->tp_index, "firmware_name not specified");
			//devm_kfree(dev, ts->panel_data.firmware_name[i]);
		}
	}

	rc = tp_judge_ic_match_commandline(&ts->panel_data);
	snprintf(data_buf, 32, "firmware-data-%d", rc);
	ts->chip_index = rc;

	if (rc < 0) {
		TP_INFO(ts->tp_index, "commandline not match, please update dts");
		goto dts_match_error;
	}

	rc = of_property_read_u32(np, "tp_type", &ts->panel_data.tp_type);

	if (rc) {
		TP_BOOT_INFO(ts->tp_index, "tp_type not specified\n");
	}

	/*firmware*/
	TP_BOOT_INFO(ts->tp_index, "read %s firmware in dts", data_buf);
	size = of_property_count_u8_elems(np, data_buf);

	if (size <= 0) {
		TP_BOOT_INFO(ts->tp_index, "No firmware in dts !\n");

	} else {
		TP_BOOT_INFO(ts->tp_index, "The firmware len id %d!\n", size);
		ts->firmware_in_dts = kzalloc(sizeof(struct firmware), GFP_KERNEL);

		if (ts->firmware_in_dts != NULL) {
			ts->firmware_in_dts->size = size;
			ts->firmware_in_dts->data = kzalloc(size, GFP_KERNEL | GFP_DMA);

			if (ts->firmware_in_dts->data == NULL) {
				kfree(ts->firmware_in_dts);
				ts->firmware_in_dts = NULL;

			} else {
				rc = of_property_read_u8_array(np, data_buf, (u8 *)ts->firmware_in_dts->data,
							       size);

				if (rc) {
					TP_BOOT_INFO(ts->tp_index, "Can not get the firmware in dts!\n");
					kfree(ts->firmware_in_dts->data);
					ts->firmware_in_dts->data = NULL;
					kfree(ts->firmware_in_dts);
					ts->firmware_in_dts = NULL;
				}
			}
		}
	}

	rc = of_property_read_u32(np, "touchpanel,irq_need_dev_resume_time", &ts->irq_need_dev_resume_time);
	if (rc) {
		TP_BOOT_INFO(ts->tp_index, "ts->irq_need_dev_resume_time not specified\n");
		ts->irq_need_dev_resume_time = 50;
	}
	TP_INFO(ts->tp_index, "ts->irq_need_dev_resume_time = %d ms\n", ts->irq_need_dev_resume_time);

	/* resolution info*/
	rc = of_property_read_u32(np, "touchpanel,max-num-support", &ts->max_num);

	if (rc) {
		TP_BOOT_INFO(ts->tp_index, "ts->max_num not specified\n");
		ts->max_num = 10;
	}

	rc = of_property_read_u32_array(np, "touchpanel,tx-rx-num", tx_rx_num, 2);

	if (rc) {
		TP_BOOT_INFO(ts->tp_index, "tx-rx-num not set\n");
		ts->hw_res.tx_num = 0;
		ts->hw_res.rx_num = 0;

	} else {
		ts->hw_res.tx_num = tx_rx_num[0];
		ts->hw_res.rx_num = tx_rx_num[1];
	}

	TP_BOOT_INFO(ts->tp_index, "tx_num = %d, rx_num = %d \n", ts->hw_res.tx_num, ts->hw_res.rx_num);

	rc = of_property_read_u32_array(np, "touchpanel,display-coords", temp_array, 2);

	if (rc) {
		TP_BOOT_INFO(ts->tp_index, "Lcd size not set\n");
		ts->resolution_info.LCD_WIDTH = 0;
		ts->resolution_info.LCD_HEIGHT = 0;

	} else {
		ts->resolution_info.LCD_WIDTH = temp_array[0];
		ts->resolution_info.LCD_HEIGHT = temp_array[1];
	}

	rc = of_property_read_u32_array(np, "touchpanel,panel-coords", temp_array, 2);

	if (rc) {
		ts->resolution_info.max_x = 0;
		ts->resolution_info.max_y = 0;

	} else {
		ts->resolution_info.max_x = temp_array[0];
		ts->resolution_info.max_y = temp_array[1];
	}

	rc = of_property_read_u32_array(np, "touchpanel,touchmajor-limit", temp_array,
					2);

	if (rc) {
		ts->touch_major_limit.width_range = 0;
		ts->touch_major_limit.height_range = 54;    /*set default value*/

	} else {
		ts->touch_major_limit.width_range = temp_array[0];
		ts->touch_major_limit.height_range = temp_array[1];
	}

	TP_BOOT_INFO(ts->tp_index, "LCD_WIDTH = %d, LCD_HEIGHT = %d, max_x = %d, max_y = %d, limit_witdh = %d, limit_height = %d\n",
		 ts->resolution_info.LCD_WIDTH, ts->resolution_info.LCD_HEIGHT,
		 ts->resolution_info.max_x, ts->resolution_info.max_y, \
		 ts->touch_major_limit.width_range, ts->touch_major_limit.height_range);

	/* virturl key Related*/
	rc = of_property_read_u32_array(np, "touchpanel,button-type", temp_array, 2);

	if (rc < 0) {
		TP_BOOT_INFO(ts->tp_index, "error:button-type should be setting in dts!");

	} else {
		ts->vk_type = temp_array[0];
		ts->vk_bitmap = temp_array[1] & 0xFF;

		if (ts->vk_type == TYPE_PROPERTIES) {
			rc = of_property_read_u32_array(np, "touchpanel,button-map", temp_array, 8);

			if (rc) {
				TP_BOOT_INFO(ts->tp_index, "button-map not set\n");

			} else {
				ts->button_map.coord_menu.x = temp_array[0];
				ts->button_map.coord_menu.y = temp_array[1];
				ts->button_map.coord_home.x = temp_array[2];
				ts->button_map.coord_home.y = temp_array[3];
				ts->button_map.coord_back.x = temp_array[4];
				ts->button_map.coord_back.y = temp_array[5];
				ts->button_map.width_x = temp_array[6];
				ts->button_map.height_y = temp_array[7];
			}
		}
	}

	/*touchkey take tx num and rx num*/
	rc = of_property_read_u32_array(np, "touchpanel.button-TRx", temp_array, 2);

	if (rc < 0) {
		TP_BOOT_INFO(ts->tp_index, "error:button-TRx should be setting in dts!\n");
		ts->hw_res.key_tx = 0;
		ts->hw_res.key_rx = 0;

	} else {
		ts->hw_res.key_tx = temp_array[0];
		ts->hw_res.key_rx = temp_array[1];
		TP_BOOT_INFO(ts->tp_index, "key_tx is %d, key_rx is %d\n", ts->hw_res.key_tx, ts->hw_res.key_rx);
	}
	rc = of_property_read_u32_array(np, "touchpanel,smooth-level", temp_array, SMOOTH_LEVEL_NUM);
	if (rc) {
		TP_BOOT_INFO(ts->tp_index, "smooth_level_array not specified %d\n", rc);
	} else {
		ts->smooth_level_array_support = true;
		for (i=0; i < SMOOTH_LEVEL_NUM; i++) {
			ts->smooth_level_array[i] = temp_array[i];
		}

		rc = of_property_read_u32_array(np,
						"touchpanel,smooth-level-charging",
						temp_array,
						SMOOTH_LEVEL_NUM);
		if (rc) {
			TP_INFO(ts->tp_index, "smooth_level_charging_array not specified %d\n", rc);
			for (i=0; i < SMOOTH_LEVEL_NUM; i++) {
				ts->smooth_level_charging_array[i] = ts->smooth_level_array[i];
			}
		} else {
			for (i=0; i < SMOOTH_LEVEL_NUM; i++) {
				ts->smooth_level_charging_array[i] = temp_array[i];
			}
		}
		ts->smooth_level_used_array = (u32 *)&(ts->smooth_level_array);
	}

	rc = of_property_read_u32_array(np, "touchpanel,sensitive-level", temp_array, SENSITIVE_LEVEL_NUM);
	if (rc) {
		TP_BOOT_INFO(ts->tp_index, "sensitive_level_array not specified %d\n", rc);
	} else {
		ts->sensitive_level_array_support = true;
		for (i=0; i < SENSITIVE_LEVEL_NUM; i++) {
			ts->sensitive_level_array[i] = temp_array[i];
		}

		rc = of_property_read_u32_array(np,
						"touchpanel,sensitive-level-charging",
						temp_array,
						SENSITIVE_LEVEL_NUM);
		if (rc) {
			TP_BOOT_INFO(ts->tp_index, "sensitive_charging_array not specified %d\n", rc);
			for (i=0; i < SENSITIVE_LEVEL_NUM; i++) {
				ts->sensitive_level_charging_array[i] = ts->sensitive_level_array[i];
			}
		} else {
			for (i=0; i < SENSITIVE_LEVEL_NUM; i++) {
				ts->sensitive_level_charging_array[i] = temp_array[i];
			}
		}
		ts->sensitive_level_used_array = (u32 *)&(ts->sensitive_level_array);
	}

	rc = of_property_read_u32_array(np, "touchpanel,click-sensitive-level", temp_array, CLICK_SENSITIVE_LEVEL_NUM);
	if (rc) {
		TP_BOOT_INFO(ts->tp_index, "click_sensitive_level_array not specified %d\n", rc);
	} else {
		ts->click_sensitive_level_array_support = true;
		for (i=0; i < CLICK_SENSITIVE_LEVEL_NUM; i++) {
			ts->click_sensitive_level_array[i] = temp_array[i];
		}
		ts->click_sensitive_level_used_array = (u32 *)&(ts->click_sensitive_level_array);
	}

	rc = of_property_read_u32_array(np, "touchpanel,long_strip_abnormal_detect_thd", temp_array, 4);
	if (rc) {
		ts->long_strip_abnormal_detect.channels_max_thd = 1;
		ts->long_strip_abnormal_detect.er_max = 4;
		ts->long_strip_abnormal_detect.er_min = 3;
		ts->long_strip_abnormal_detect.center_width = 500;
	} else {
		ts->long_strip_abnormal_detect.channels_max_thd = temp_array[0];
		ts->long_strip_abnormal_detect.er_max = temp_array[1];
		ts->long_strip_abnormal_detect.er_min = temp_array[2];
		ts->long_strip_abnormal_detect.center_width = temp_array[3];
	}

	rc = of_property_read_u32_array(np, "touchpanel,game_perf_para_default", temp_array, 2);

	if (rc < 0) {
		TP_BOOT_INFO(ts->tp_index, "error:game_perf_para_default should be setting in dts!\n");
		ts->sensitive_level_default = 0;
		ts->smooth_level_default = 0;

	} else {
		ts->sensitive_level_default = temp_array[0];
		ts->smooth_level_default = temp_array[1];
		TP_BOOT_INFO(ts->tp_index, "sensitive_level_default is %d, smooth_level_default is %d\n",
			ts->sensitive_level_default, ts->smooth_level_default);
	}

	/*set disable suspend irq handler parameter, for of_property_read_bool return 1 when success and return 0 when item is not exist*/
	ts->disable_suspend_irq_handler = of_property_read_bool(np, "disable_suspend_irq_handler_support");

	/*set incell panel parameter, for of_property_read_bool return 1 when success and return 0 when item is not exist*/
	ts->is_incell_panel = of_property_read_bool(np, "incell_screen");
	rc = ts->is_incell_panel;

	if (rc > 0) {
		TP_BOOT_INFO(ts->tp_index, "panel is incell!\n");
		ts->is_incell_panel = 1;

	} else {
		TP_BOOT_INFO(ts->tp_index, "panel is oncell!\n");
		ts->is_incell_panel = 0;
	}

	ts->tp_ic_type = TYPE_ONCELL;
	rc = of_property_read_u32(np, "touchpanel,tp_ic_type", &val);

	if (rc) {
		TP_BOOT_INFO(ts->tp_index, "tp_ic_type not specified\n");

	} else {
		if (val < TYPE_IC_MAX) {
			ts->tp_ic_type = val;
		}
	}

	rc = of_property_read_u32(np, "touchpanel,single-optimized-time", &ts->single_optimized_time);
	if (rc) {
		TP_BOOT_INFO(ts->tp_index, "ts->single_optimized_time not specified\n");
		ts->single_optimized_time = 0;
		ts->optimized_show_support = false;
	} else {
		ts->total_operate_times = 0;
		ts->optimized_show_support = true;
	}

    rc = of_property_read_u32(np, "touchpanel,high-frame-rate-time", &ts->high_frame_rate_time);
    if (rc) {
        TP_BOOT_INFO(ts->tp_index, "ts->high_frame_rate_time not specified or support\n");
        ts->high_frame_rate_time = 0;
        ts->high_frame_rate_support = false;
    } else {
        ts->high_frame_rate_support = true;
    }

    rc = of_property_read_u32(np, "report_rate_limit", &ts->panel_data.report_rate_limit);
    if (rc) {
        TPD_BOOT_INFO("report_rate_limit not specified,we will set it 60hz\n");
        ts->monitor_data.RATE_MIN = 60;
    } else {
        ts->monitor_data.RATE_MIN = ts->panel_data.report_rate_limit;
    }

	if (ts->pen_support) {
		rc = of_property_read_u32_array(np, "touchpanel,pen-tx-rx-num", tx_rx_num, 2);
		if (rc) {
			TPD_BOOT_INFO("pen-tx-rx-num not set\n");
			ts->hw_res.pen_tx_num = 0;
			ts->hw_res.pen_rx_num = 0;
		} else {
			ts->hw_res.pen_tx_num = tx_rx_num[0];
			ts->hw_res.pen_rx_num = tx_rx_num[1];
		}
		TPD_BOOT_INFO("pen_tx_num = %d, pen_rx_num = %d \n", ts->hw_res.pen_tx_num, ts->hw_res.pen_rx_num);

		rc = of_property_read_u32_array(np, "touchpanel,pen-panel-coords", temp_array, 2);
		if (rc) {
			TPD_BOOT_INFO("panel for pen coords not set\n");
			ts->pen_config.max_x = 0;
			ts->pen_config.max_y = 0;
		} else {
			ts->pen_config.max_x = temp_array[0];
			ts->pen_config.max_y = temp_array[1];
		}
		TPD_BOOT_INFO("pen_max_x = %d, pen_max_y = %d \n", ts->pen_config.max_x, ts->pen_config.max_y);

		rc = of_property_read_u32(np, "touchpanel,pen-max-pressure", &(ts->pen_config.max_pressure));
		if (rc) {
			TPD_BOOT_INFO("pen pressure config not specified\n");
			ts->pen_config.max_pressure = 1023;
		}
		TPD_BOOT_INFO("pen_max_pressure = %d.\n", ts->pen_config.max_pressure);

		rc = of_property_read_u32_array(np, "touchpanel,pen-max-tilt", temp_array, 2);
		if (rc) {
			TPD_BOOT_INFO("panel for pen tilt not set\n");
			ts->pen_config.tilt_x_max = 90;
			ts->pen_config.tilt_y_max = 90;
		} else {
			ts->pen_config.tilt_x_max = temp_array[0];
			ts->pen_config.tilt_y_max = temp_array[1];
		}
		TPD_BOOT_INFO("pen_tilt_x = %d, pen_tilt_y = %d \n", ts->pen_config.tilt_x_max, ts->pen_config.tilt_y_max);
	}

	rc = of_property_read_string(np, "touchpanel,touch-environment", (char const **)&ts->touch_environment);
	if (rc) {
		dev_warn(ts->dev,
			"%s: No trusted touch mode environment\n", __func__);
		ts->touch_environment = "pvm";
	}
	else {
		TPD_BOOT_INFO("touch_environment:%s\n", ts->touch_environment);
	}

	rc = of_property_read_u32(np, "touchpanel,aiunit_game_valid_bits", &ts->aiunit_game_valid_bits);

	if (rc) {
		TP_BOOT_INFO(ts->tp_index, "tp aiunit game valid bits not specified\n");
		ts->aiunit_game_valid_bits = 19;
	}

	init_panel_config(dev, ts);

	return ret;

dts_match_error:
	return -1;
}

static int oplus_gpio_suspend(struct touchpanel_data *ts)
{
	int r = 0;

	if (ts->hw_res.pin_spi_mode_suspend != NULL) {
		r = pinctrl_select_state(ts->hw_res.pinctrl, ts->hw_res.pin_spi_mode_suspend);
		if (r < 0) {
			TP_INFO(ts->tp_index, "Failed to select pin_spi_mode_suspend, r:%d", r);
			goto err_suspend_pinctrl;
		}
		TP_INFO(ts->tp_index, "success set suspend for pin_spi_mode_suspend");
	}

	if (ts->hw_res.pin_int_sta_suspend != NULL) {
		r = pinctrl_select_state(ts->hw_res.pinctrl, ts->hw_res.pin_int_sta_suspend);
		if (r < 0) {
			TP_INFO(ts->tp_index, "Failed to select pin_int_sta_suspend, r:%d", r);
			goto err_suspend_pinctrl;
		}
		TP_INFO(ts->tp_index, "success set suspend for pin_int_sta_suspend");
	}

	if (ts->hw_res.pin_rst_sta_suspend != NULL) {
		r = pinctrl_select_state(ts->hw_res.pinctrl, ts->hw_res.pin_rst_sta_suspend);
		if (r < 0) {
			TP_INFO(ts->tp_index, "Failed to select pin_rst_sta_suspend, r:%d", r);
			goto err_suspend_pinctrl;
		}
		TP_INFO(ts->tp_index, "success set suspend for pin_rst_sta_suspend");
	}

err_suspend_pinctrl:
	return r;
}

static int oplus_gpio_resume(struct touchpanel_data *ts)
{
	int r = 0;

	if (ts->hw_res.pin_spi_mode_active != NULL) {
		r = pinctrl_select_state(ts->hw_res.pinctrl, ts->hw_res.pin_spi_mode_active);
		if (r < 0) {
			TP_INFO(ts->tp_index, "Failed to select pin_spi_mode_active, r:%d", r);
			goto err_resume_pinctrl;
		}
		TP_INFO(ts->tp_index, "success set resume for pin_spi_mode_active");
	}

	if (ts->hw_res.pin_int_sta_active != NULL) {
		r = pinctrl_select_state(ts->hw_res.pinctrl, ts->hw_res.pin_int_sta_active);
		if (r < 0) {
			TP_INFO(ts->tp_index, "Failed to select pin_int_sta_active, r:%d", r);
			goto err_resume_pinctrl;
		}
		TP_INFO(ts->tp_index, "success set resume for pin_int_sta_active");
	}

	if (ts->hw_res.pin_rst_sta_active != NULL) {
		r = pinctrl_select_state(ts->hw_res.pinctrl, ts->hw_res.pin_rst_sta_active);
		if (r < 0) {
			TP_INFO(ts->tp_index, "Failed to select pin_rst_sta_active, r:%d", r);
			goto err_resume_pinctrl;
		}
		TP_INFO(ts->tp_index, "success set resume for pin_rst_sta_active");
	}

err_resume_pinctrl:
		return r;
}

static int init_power_control(struct touchpanel_data *ts)
{
	int ret = 0;
	/* 1.8v*/
	ts->hw_res.vddi = regulator_get(ts->dev, "vcc_1v8");

	if (IS_ERR_OR_NULL(ts->hw_res.vddi)) {
		TP_BOOT_INFO(ts->tp_index, "Regulator get failed vcc_1v8, ret = %d\n", ret);

	} else {
		if (regulator_count_voltages(ts->hw_res.vddi) > 0 || ts->regulator_count_not_support) {

			if (ts->hw_res.vddi_volt) {
				ret = regulator_set_voltage(ts->hw_res.vddi, ts->hw_res.vddi_volt,
							    ts->hw_res.vddi_volt);
			} else {
				ret = regulator_set_voltage(ts->hw_res.vddi, 1800000, 1800000);
			}

			if (ret) {
				dev_err(ts->dev, "Regulator set_vtg failed vcc_i2c rc = %d\n", ret);
				goto err;
			}

			ret = regulator_set_load(ts->hw_res.vddi, 200000);

			if (ret < 0) {
				dev_err(ts->dev, "Failed to set vcc_1v8 mode(rc:%d)\n", ret);
				goto err;
			}
		} else {
			TPD_BOOT_INFO("regulator_count_voltages is not support\n");
		}
	}

	/* vdd 2.8v*/
	ts->hw_res.avdd = regulator_get(ts->dev, "vdd_2v8");

	if (IS_ERR_OR_NULL(ts->hw_res.avdd)) {
		TP_BOOT_INFO(ts->tp_index, "Regulator vdd2v8 get failed, ret = %d\n", ret);

	} else {
		if (regulator_count_voltages(ts->hw_res.avdd) > 0) {
			TP_BOOT_INFO(ts->tp_index, "set avdd voltage to %d uV\n", ts->hw_res.vdd_volt);

			if (ts->hw_res.vdd_volt) {
				ret = regulator_set_voltage(ts->hw_res.avdd, ts->hw_res.vdd_volt,
							    ts->hw_res.vdd_volt);

			} else {
				ret = regulator_set_voltage(ts->hw_res.avdd, 3100000, 3100000);
			}

			if (ret) {
				dev_err(ts->dev, "Regulator set_vtg failed vdd rc = %d\n", ret);
				goto err;
			}

			ret = regulator_set_load(ts->hw_res.avdd, 200000);

			if (ret < 0) {
				dev_err(ts->dev, "Failed to set vdd_2v8 mode(rc:%d)\n", ret);
				goto err;
			}
		} else {
			TPD_BOOT_INFO("regulator_count_voltages is not support\n");
		}
	}

	return 0;

err:
	return ret;
}

int tp_powercontrol_vddi(struct hw_resource *hw_res, bool on)
{
	int ret = 0;

	if (on) { /* 1v8 power on*/
		if (!IS_ERR_OR_NULL(hw_res->vddi)) {
			TPD_BOOT_INFO("Enable the Regulator vddi.\n");
			ret = regulator_enable(hw_res->vddi);

			if (ret) {
				TPD_INFO("Regulator vcc_i2c enable failed ret = %d\n", ret);
				return ret;
			}
		}

		if (hw_res->enable_vddi_gpio > 0) {
			TPD_BOOT_INFO("Enable the vddi_gpio\n");
			ret = gpio_direction_output(hw_res->enable_vddi_gpio, 1);

			if (ret) {
				TPD_INFO("enable the enable_vddi_gpio failed.\n");
				return ret;
			}
		}

	} else { /* 1v8 power off*/
		if (!IS_ERR_OR_NULL(hw_res->vddi)) {
			TPD_BOOT_INFO("disable the vddi_gpio\n");
			ret = regulator_disable(hw_res->vddi);
			if (ret) {
				TPD_INFO("Regulator vcc_i2c enable failed rc = %d\n", ret);
				return ret;
			}
		}

		if (hw_res->enable_vddi_gpio > 0) {
			TPD_BOOT_INFO("disable the enable_vddi_gpio\n");
			ret = gpio_direction_output(hw_res->enable_vddi_gpio, 0);

			if (ret) {
				TPD_INFO("disable the enable_vddi_gpio failed.\n");
				return ret;
			}
		}
	}

	return 0;
}
EXPORT_SYMBOL(tp_powercontrol_vddi);

int tp_powercontrol_avdd(struct hw_resource *hw_res, bool on)
{
	int ret = 0;

	if (on) { /* 2v8 power on*/
		if (!IS_ERR_OR_NULL(hw_res->avdd)) {
			TPD_BOOT_INFO("Enable the Regulator2v8.\n");
			ret = regulator_enable(hw_res->avdd);

			if (ret) {
				TPD_INFO("Regulator vdd enable failed ret = %d\n", ret);
				return ret;
			}
		}

		if (hw_res->enable_avdd_gpio > 0) {
			TPD_BOOT_INFO("Enable the enable_avdd_gpio, hw_res->enable2v8_gpio is %d\n",
				 hw_res->enable_avdd_gpio);
			ret = gpio_direction_output(hw_res->enable_avdd_gpio, 1);

			if (ret) {
				TPD_INFO("enable the enable_avdd_gpio failed.\n");
				return ret;
			}
		}

	} else { /* 2v8 power off*/
		if (!IS_ERR_OR_NULL(hw_res->avdd)) {
			ret = regulator_disable(hw_res->avdd);

			if (ret) {
				TPD_INFO("Regulator vdd disable failed rc = %d\n", ret);
				return ret;
			}
		}

		if (hw_res->enable_avdd_gpio > 0) {
			TPD_BOOT_INFO("disable the enable_avdd_gpio\n");
			ret = gpio_direction_output(hw_res->enable_avdd_gpio, 0);

			if (ret) {
				TPD_INFO("disable the enable_avdd_gpio failed.\n");
				return ret;
			}
		}
	}

	return ret;
}
EXPORT_SYMBOL(tp_powercontrol_avdd);


static void esd_handle_func(struct work_struct *work)
{
	int ret = 0;
	int gpio_en, regulator_en, regulator_vol = 0;
	struct touchpanel_data *ts = container_of(work, struct touchpanel_data,
				     esd_info.esd_check_work.work);

	if (ts->loading_fw) {
		TP_INFO(ts->tp_index, "FW is updating, stop esd handle!\n");
		return;
	}

	mutex_lock(&ts->esd_info.esd_lock);

	if (!ts->esd_info.esd_running_flag) {
		TP_INFO(ts->tp_index, "Esd protector has stopped!\n");
		goto ESD_END;
	}

	if (ts->is_suspended == 1) {
		TP_INFO(ts->tp_index, "Touch panel has suspended!\n");
		goto ESD_END;
	}

	if (!ts->ts_ops->esd_handle) {
		TP_INFO(ts->tp_index, "not support ts_ops->esd_handle callback\n");
		goto ESD_END;
	}

	ret = ts->ts_ops->esd_handle(ts->chip_data);

	if (CHK_BIT_NUM(ts->monitor_data.health_simulate_trigger, HEALTH_SIMULATE_BIT_ESD)
		   || ret == -1) {    /*-1 means esd hanppened: handled in IC part, recovery the state here*/
		tp_healthinfo_report(&ts->monitor_data, HEALTH_REPORT, "esd_happened");
		operate_mode_switch(ts);
		if (ts->exception_upload_support) {
			tp_exception_report(&ts->exception_data, EXCEP_HARDWARE, "esd_handle_failed", sizeof("esd_handle_failed"));
		}
	}

	if (!IS_ERR_OR_NULL(ts->hw_res.avdd) && (regulator_get_voltage(ts->hw_res.avdd) != -EINVAL)) {
		regulator_en = regulator_is_enabled(ts->hw_res.avdd);
		regulator_vol = regulator_get_voltage(ts->hw_res.avdd);
		if (!regulator_en || regulator_vol < AVDD_VOLTAGE_LIMIT_MIN || regulator_vol > AVDD_VOLTAGE_LIMIT_MAX
				   || CHK_BIT_NUM(ts->monitor_data.health_simulate_trigger, HEALTH_SIMULATE_BIT_AVDD_VDDI)) {
			TP_INFO(ts->tp_index, "avdd regulator enabled=%d, voltage=%d\n", regulator_en, regulator_vol);
			regulator_vol = regulator_en ? regulator_vol : VOLTAGE_STATE_REGULATOR_DISABLED;
			tp_healthinfo_report(&ts->monitor_data, HEALTH_AVDD, &regulator_vol);
		}
	}
	if (ts->hw_res.enable_avdd_gpio > 0) {
		gpio_en = gpio_get_value(ts->hw_res.enable_avdd_gpio);
		if (!gpio_en || CHK_BIT_NUM(ts->monitor_data.health_simulate_trigger, HEALTH_SIMULATE_BIT_AVDD_VDDI)) {
			TP_INFO(ts->tp_index, "avdd gpio is %d\n", gpio_en);
			regulator_vol = VOLTAGE_STATE_ENABLE_GPIO_LOW;
			tp_healthinfo_report(&ts->monitor_data, HEALTH_AVDD, &regulator_vol);
		}
	}

	if (!IS_ERR_OR_NULL(ts->hw_res.vddi) && (regulator_get_voltage(ts->hw_res.vddi) != -EINVAL)) {
		regulator_en = regulator_is_enabled(ts->hw_res.vddi);
		regulator_vol = regulator_get_voltage(ts->hw_res.vddi);
		if (!regulator_en || regulator_vol < VDDI_VOLTAGE_LIMIT_MIN || regulator_vol > VDDI_VOLTAGE_LIMIT_MAX
				   || CHK_BIT_NUM(ts->monitor_data.health_simulate_trigger, HEALTH_SIMULATE_BIT_AVDD_VDDI)) {
			TP_INFO(ts->tp_index, "vddi regulator enabled=%d, voltage=%d\n", regulator_en, regulator_vol);
			regulator_vol = regulator_en ? regulator_vol : VOLTAGE_STATE_REGULATOR_DISABLED;
			tp_healthinfo_report(&ts->monitor_data, HEALTH_VDDI, &regulator_vol);
		}
	}
	if (ts->hw_res.enable_vddi_gpio > 0) {
		gpio_en = gpio_get_value(ts->hw_res.enable_vddi_gpio);
		if (!gpio_en || CHK_BIT_NUM(ts->monitor_data.health_simulate_trigger, HEALTH_SIMULATE_BIT_AVDD_VDDI)) {
			TP_INFO(ts->tp_index, "vddi gpio is %d\n", gpio_en);
			regulator_vol = VOLTAGE_STATE_ENABLE_GPIO_LOW;
			tp_healthinfo_report(&ts->monitor_data, HEALTH_VDDI, &regulator_vol);
		}
	}

	if (ts->up_status && gpio_is_valid(ts->hw_res.irq_gpio)) {
		gpio_en = gpio_get_value(ts->hw_res.irq_gpio);
		if (!gpio_en || CHK_BIT_NUM(ts->monitor_data.health_simulate_trigger, HEALTH_SIMULATE_BIT_IRQ_GPIO)) {
			TP_INFO(ts->tp_index, "irq gpio is %d\n", gpio_en);
			tp_healthinfo_report(&ts->monitor_data, HEALTH_REPORT, "touch_up_irq_low");
		}
	}

	if (ts->esd_info.esd_running_flag) {
		queue_delayed_work(ts->esd_info.esd_workqueue, &ts->esd_info.esd_check_work,
				   ts->esd_info.esd_work_time);

	} else {
		TP_INFO(ts->tp_index, "Esd protector suspended!");
	}

ESD_END:
	mutex_unlock(&ts->esd_info.esd_lock);
	return;
}

/**
 * esd_handle_switch - open or close esd thread
 * @esd_info: touchpanel_data, using for common driver resource
 * @on: bool variable using for  indicating open or close esd check function.
 *     true:open;
 *     false:close;
 */
void esd_handle_switch(struct esd_information *esd_info, bool on)
{
	mutex_lock(&esd_info->esd_lock);

	if (on) {
		if (!esd_info->esd_running_flag) {
			esd_info->esd_running_flag = 1;

			TPD_INFO("Esd protector started, cycle: %d s\n", esd_info->esd_work_time / HZ);
			queue_delayed_work(esd_info->esd_workqueue, &esd_info->esd_check_work,
					   esd_info->esd_work_time);
		}

	} else {
		if (esd_info->esd_running_flag) {
			esd_info->esd_running_flag = 0;

			TPD_INFO("Esd protector stoped!\n");
			cancel_delayed_work(&esd_info->esd_check_work);
		}
	}

	mutex_unlock(&esd_info->esd_lock);
}
EXPORT_SYMBOL(esd_handle_switch);

static void tp_rate_calc(struct touchpanel_data *ts, tp_rate tp_rate_type)
{
	switch(tp_rate_type) {
	case TP_RATE_START:
		ts->curr_time = ktime_to_ms(ktime_get());
		break;
	case TP_RATE_CLEAR:
		ts->irq_num = 0;
		break;
	case TP_RATE_CALC:
		if (ts->irq_interval == 0) {
			ts->irq_interval = ts->curr_time;
		}
		ts->irq_interval = ts->curr_time - ts->irq_interval;
		ts->irq_handle_time = ktime_to_ms(ktime_get()) - ts->curr_time;
		if(ts->irq_num > 0 && ts->monitor_data.in_game_mode && ts->irq_num % 10 == 0) {
			TP_INFO(ts->tp_index, "rate = %llu,ts->irq_handle_time =%llu,curr_time = %llu\n", 1000/ts->irq_interval, ts->irq_handle_time, ts->curr_time);
		} else {
			if (ts->irq_num > 0 && ts->irq_num % 100 == 0) {
				TP_INFO(ts->tp_index, "rate = %llu,ts->irq_handle_time =%llu,curr_time = %llu\n", 1000/ts->irq_interval, ts->irq_handle_time, ts->curr_time);
			}
		}

		if (ts->irq_num > 0 && 1000/ts->irq_interval < ts->monitor_data.RATE_MIN && ts->health_monitor_support) {
			tp_healthinfo_report(&ts->monitor_data, HEALTH_BELOW_RATE, (void *)&ts->irq_num);
		}
		ts->irq_interval = ts->curr_time;
		ts->irq_num++;
		break;
	default:
		break;
	}
}

int tp_register_irq_func(struct touchpanel_data *ts)
{
	int ret = 0;

/* Avoid setting up hardware for TVM during probe */
#ifdef CONFIG_ARCH_QTI_VM
	if (!atomic_read(&ts->delayed_vm_probe_pending)) {
		atomic_set(&ts->delayed_vm_probe_pending, 1);
		return 0;
	}
	goto tvm_setup;
#endif

	if (gpio_is_valid(ts->hw_res.irq_gpio)) {
	TP_DEBUG(ts->tp_index, "%s, irq_gpio is %d, ts->irq is %d\n", __func__,
	ts->hw_res.irq_gpio, ts->irq);

		if (ts->irq_flags_cover) {
			ts->irq_flags = ts->irq_flags_cover;
			TP_BOOT_INFO(ts->tp_index, "%s irq_flags is covered by 0x%x\n", __func__,
				ts->irq_flags_cover);
		}

		if (ts->irq <= 0) {
			ts->irq = gpio_to_irq(ts->hw_res.irq_gpio);
			TP_BOOT_INFO(ts->tp_index, "%s ts->irq is %d\n", __func__, ts->irq);
		}

		snprintf(ts->irq_name, sizeof(ts->irq_name), "touch-%02d", ts->tp_index);

		TP_INFO(ts->tp_index, "%s registerFlag:%d and topIrq:%d\n", __func__, ts->irq_flags, ts->input_timestamp_in_top_irq_support);
		ret = devm_request_threaded_irq(ts->dev,
						ts->irq,
						set_input_timestamp_in_top_irq,
						tp_irq_thread_fn,
						ts->irq_flags | IRQF_ONESHOT,
						ts->irq_name, ts);
		if (ret < 0) {
			TP_INFO(ts->tp_index, "%s request_threaded_irq ret is %d\n", __func__, ret);
		}
	} else {
		TP_INFO(ts->tp_index, "%s:no valid irq\n", __func__);
		ret = -1;
	}
	return ret;
#ifdef CONFIG_ARCH_QTI_VM
tvm_setup:
	TP_INFO(ts->tp_index, "%s ts->irq is %d\n", __func__, ts->irq);
	snprintf(ts->irq_name, sizeof(ts->irq_name), "touch-%02d", ts->irq);
	ret = devm_request_threaded_irq(ts->dev, ts->irq, NULL,
				tp_irq_thread_fn,
				ts->irq_tui_flags | IRQF_ONESHOT,
				ts->irq_name, ts);
	return 0;
#endif
}

void tp_delta_read_triggered_by_key(int index)
{
	struct touchpanel_data *ts = get_ts_data(index);

	if (tp_debug != 2)
		return;

	if (!ts)
		return;
	if (!ts->key_trigger_wq)
		return;

	queue_work(ts->key_trigger_wq, &ts->key_trigger_work);
}
EXPORT_SYMBOL(tp_delta_read_triggered_by_key);

static void tp_delta_read_triggered_by_key_handle(struct work_struct *work)
{
	struct debug_info_proc_operations *debug_info_ops;
	struct touchpanel_data *ts = container_of(work, struct touchpanel_data,
				key_trigger_work);

	if (!ts)
		return;

	TP_INFO(ts->tp_index, "%s:tp_debug= %d\n", __func__, tp_debug);

	if (tp_debug != 2)
		return;

	touchpanel_trusted_touch_completion(ts);
	debug_info_ops = (struct debug_info_proc_operations *)ts->debug_info_ops;

	if (!debug_info_ops)
		return;
	if (!debug_info_ops->key_trigger_delta_read) {
		return;
	}
	if (!debug_info_ops->delta_read) {
		return;
	}

	if (ts->int_mode == BANNABLE) {
		disable_irq_nosync(ts->irq);
	}
	mutex_lock(&ts->mutex);
	debug_info_ops->key_trigger_delta_read(ts->chip_data);
	mutex_unlock(&ts->mutex);
	if (ts->int_mode == BANNABLE) {
	enable_irq(ts->irq);
	}
	return;
}


static int tp_paneldata_init(struct touchpanel_data *pdata)
{
	struct touchpanel_data *ts = pdata;
	int ret = -1;

	if (!ts) {
		return ret;
	}

	/*step7 : Alloc fw_name/devinfo memory space*/
	ts->panel_data.fw_name = tp_devm_kzalloc(ts->dev, MAX_FW_NAME_LENGTH,
				 GFP_KERNEL);

	if (ts->panel_data.fw_name == NULL) {
		ret = -ENOMEM;
		TP_INFO(ts->tp_index, "panel_data.fw_name kzalloc error\n");
		return ret;
	}

#ifndef CONFIG_REMOVE_OPLUS_FUNCTION
	ts->panel_data.manufacture_info.version = tp_devm_kzalloc(ts->dev,
			MAX_DEVICE_VERSION_LENGTH, GFP_KERNEL);

	if (ts->panel_data.manufacture_info.version == NULL) {
		ret = -ENOMEM;
		TP_INFO(ts->tp_index, "manufacture_info.version kzalloc error\n");
		return ret;
	}

	ts->panel_data.manufacture_info.manufacture = tp_devm_kzalloc(ts->dev,
			MAX_DEVICE_MANU_LENGTH, GFP_KERNEL);

	if (ts->panel_data.manufacture_info.manufacture == NULL) {
		ret = -ENOMEM;
		TP_INFO(ts->tp_index, "panel_data.fw_name kzalloc error\n");
		return ret;
	}

	/*step8 : touchpanel vendor*/
	tp_util_get_vendor(&ts->hw_res, &ts->panel_data);

	if (ts->ts_ops->get_vendor) {
		ts->ts_ops->get_vendor(ts->chip_data, &ts->panel_data);
	}
	if (ts->health_monitor_support) {
		ts->monitor_data.vendor = ts->panel_data.manufacture_info.manufacture;
	}
#endif

	return 0;
}

static int tp_power_init(struct touchpanel_data *pdata)
{
	struct touchpanel_data *ts = pdata;
	int ret = -1;

	if (!ts) {
		return ret;
	}

	preconfig_power_control(ts);
	ret = init_power_control(ts);

	if (ret) {
		ret = -EINVAL;
		TP_INFO(ts->tp_index, "%s: tp power init failed.\n", __func__);
		return ret;
	}

	ret = reconfig_power_control(ts);

	if (ret) {
		ret = -EINVAL;
		TP_INFO(ts->tp_index, "%s: reconfig power failed.\n", __func__);
		return ret;
	}

	if (!ts->ts_ops->power_control) {
		ret = -EINVAL;
		TP_INFO(ts->tp_index, "tp power_control NULL!\n");
		return ret;
	}

	ret = ts->ts_ops->power_control(ts->chip_data, true);

	if (ret) {
		ret = -EINVAL;
		TP_INFO(ts->tp_index, "%s: tp power init failed.\n", __func__);
		return ret;
	}

	return 0;
}

#ifndef CONFIG_ARCH_QTI_VM
static int init_get_adc_channels(struct touchpanel_data *ts)
{
	int rc = -1;
	const char *name;
	struct device_node *node = ts->dev->of_node;

	rc = of_property_read_string(node, "oplus,skin_temp_chan", &name);
	if (rc < 0) {
		pr_err("can't get oplus,skin_temp_chan, rc=%d\n", rc);
		return rc;
	}
	rc = of_property_match_string(node, "io-channel-names", name);
	if (rc < 0) {
		pr_err("can't read io-channel-names, rc=%d\n", rc);
		return rc;
	}

	ts->skin_therm_chan = iio_channel_get(ts->dev, name);
	if (IS_ERR(ts->skin_therm_chan)) {
		rc = PTR_ERR(ts->skin_therm_chan);
		if (rc != -EPROBE_DEFER)
			pr_err("%s channel unavailable, %d\n", name, rc);
		ts->skin_therm_chan = NULL;
	}
	TP_INFO(ts->tp_index, "%s finished\n", __func__);
	return rc;
}

static int pre_get_shellback(struct touchpanel_data *ts)
{
	int ret = 0;
	ts->oplus_shell_themal = thermal_zone_get_zone_by_name("shell_back");

	if (IS_ERR(ts->oplus_shell_themal)) {
		TP_INFO(ts->tp_index, "%s Can't get shell_back\n", __func__);
		ts->oplus_shell_themal = NULL;
		ret = -1;
	}

	TPD_DEBUG("%s get shell_back ret:%d\n", __func__, ret);

	return ret;
}

static void tp_get_temperature_work(struct work_struct *work)
{
	struct touchpanel_data *ts = container_of(work, struct touchpanel_data,
				get_temperature_work);
	int result = -40000;
	int rc;
	int temperature_debounce = 0;
	int temperature_cnt = 3;

#ifdef CONFIG_TOUCHPANEL_TRUSTED_TOUCH
	if (atomic_read(&ts->trusted_touch_enabled) == 1) {
		TP_INFO(ts->tp_index, "%s: Trusted touch is already enabled, do not get temp\n", __func__);
		return;
	}
#endif

	if (ts->temperature_detect_shellback_support) {
		do {
			rc = pre_get_shellback(ts);
			if (rc < 0) {
				TP_INFO(ts->tp_index, "%s can't pre_get_shellback\n", __func__);
				goto End;
			}
			rc = thermal_zone_get_temp(ts->oplus_shell_themal, &result);
			if (rc < 0)
				TP_INFO(ts->tp_index, "%s can't thermal_zone_get_temp, rc=%d\n", __func__, rc);

			if (result != 0) {
				TP_DEBUG(ts->tp_index, "%s temperature valid, not check again:%d temp:%d\n",
					__func__, temperature_debounce, result);
				break;
			}
			temperature_debounce++;
			msleep(4);
		} while (temperature_debounce >= temperature_cnt);
	} else {
		if (!ts->skin_therm_chan)
			return;
		rc = iio_read_channel_processed(ts->skin_therm_chan, &result);
		if (rc < 0)
			TP_INFO(ts->tp_index, "%s can't skin_therm_chan, rc=%d\n", __func__, rc);
	}

	result = result / 1000;

	TP_DEBUG(ts->tp_index, "[oldTemp:%d][curTemp:%d]\n",
		ts->old_temp, result);

	if (ts->old_temp == result)
		goto End;

	if (result > MAX_TEMPERATURE || result < MIN_TEMPERATURE) {
		ts->monitor_data.abnormal_temperature_count++;
		goto End;
	}

	 if (ts->bus_ready == false) {
		if (ts->health_monitor_support) {
			ts->monitor_data.bus_not_ready_temperature_work_count++;
		}
		if (ts->bus_ready_check_support) {
			TP_INFO(ts->tp_index, "bus_ready not ready, %s exit\n", __func__);
			goto End;
		}
	}

	if (ts->is_suspended == 0 && (ts->temperature_detect_support || ts->temperature_detect_shellback_support)) {
		mutex_lock(&ts->mutex);
		ts->ts_ops->send_temperature(ts->chip_data, result, true);
		mutex_unlock(&ts->mutex);
	}
	ts->old_temp = result;
End:
	hrtimer_start(&ts->temp_timer, ktime_set(10, 0), HRTIMER_MODE_REL);
}
static enum hrtimer_restart ts_get_temperature_timeout(struct hrtimer *timer)
{
	struct touchpanel_data *ts = container_of(timer, struct touchpanel_data, temp_timer);

	schedule_work(&ts->get_temperature_work);

	return HRTIMER_NORESTART;
}
#endif

#if IS_ENABLED(CONFIG_DRM_OPLUS_PANEL_NOTIFY) || IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
static int ts_check_panel_dt(struct device *dev, struct touchpanel_data *ts)
{
	int i;
	int count;
	struct device_node *node = NULL;
	struct drm_panel *panel = NULL;
	struct device_node *np = NULL;
	char disp_node[32] = {0};

	np = of_find_node_by_name(NULL, "oplus,dsi-display-dev");
	if (!np) {
		TP_BOOT_INFO(ts->tp_index, "%s: [oplus,dsi-display-dev] is missing, try to find [panel] node \n", __func__);
		np = dev->of_node;
		strncpy(disp_node, "panel", sizeof("panel"));
	} else {
		TP_BOOT_INFO(ts->tp_index, "%s: [oplus,dsi-display-dev] node found \n", __func__);
		if (ts->tp_index == 0) {
			strncpy(disp_node, "oplus,dsi-panel-primary", sizeof("oplus,dsi-panel-primary"));
		} else if (ts->tp_index == 1) {
			strncpy(disp_node, "oplus,dsi-panel-secondary", sizeof("oplus,dsi-panel-secondary"));
		} else {
			TP_BOOT_INFO(ts->tp_index, "%s: This branch will never be called now \n", __func__);
		}
		TP_BOOT_INFO(ts->tp_index, "%s: disp_node = %s \n", __func__, disp_node);
	}

	count = of_count_phandle_with_args(np, disp_node, NULL);
	if (count <= 0) {
		TP_BOOT_INFO(ts->tp_index, "%s: can not find [%s] node in dts \n", __func__, disp_node);
		return 0;
	}

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, disp_node, i);
		panel = of_drm_find_panel(node);

		TP_BOOT_INFO(ts->tp_index, "%s: panel[%d] IS_ERR =%d \n", __func__, i, IS_ERR(panel));
		of_node_put(node);
		if (!IS_ERR(panel)) {
			ts->active_panel = panel;
			return 0;
		}
	}

	return PTR_ERR(panel);
}
#endif

/**
 * register_common_touch_device - parse dts, get resource defined in Dts
 * @pdata: touchpanel_data, using for common driver
 *
 * entrance of common touch Driver
 * Returning zero(sucess) or negative errno(failed)
 */
int register_common_touch_device(struct touchpanel_data *pdata)
{
	struct touchpanel_data *ts = pdata;
	char name[TP_NAME_SIZE_MAX];
	struct device_node *chip_np = NULL;
	struct device_node *src_chip_np = NULL;
	int ret = -1;
	int i = 0;
#if IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
	void *cookie = NULL;
#endif
	TPD_BOOT_INFO("%s  is called\n", __func__);

	if (!ts->dev) {
		return -1;
	}

	/*step1 : dts parse*/
	ret = init_parse_dts(ts->dev, ts);

	if (ret < 0) {
		TP_INFO(ts->tp_index, "%s: dts init failed.\n", __func__);
		return -1;
	}

#if IS_ENABLED(CONFIG_DRM_OPLUS_PANEL_NOTIFY) || IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
	ret = ts_check_panel_dt(ts->dev, ts);
	if (ret < 0) {
		TP_INFO(ts->tp_index, "%s: ts_check_panel_dt failed.\n", __func__);
#ifndef  CONFIG_ARCH_QTI_VM
		/*retry*/
		return -EPROBE_DEFER;
#endif
	}
#endif

	/*step2 : initial health info parameter*/
	if (ts->health_monitor_support) {
		ret = tp_healthinfo_init(ts->dev, &ts->monitor_data);

		if (ret < 0) {
			TP_INFO(ts->tp_index, "health info init failed.\n");
		}

		ts->monitor_data.health_monitor_support = true;
		ts->monitor_data.chip_data = ts->chip_data;
		ts->monitor_data.debug_info_ops = ts->debug_info_ops;
	}

	if (ts->exception_upload_support) {
		ts->exception_data.exception_upload_support = true;
		ts->exception_data.chip_data = ts->chip_data;
	}

	/*step3 : interfaces init*/
	init_touch_interfaces(ts->dev, ts->register_is_16bit);
	// lcd_tp_refresh_support support
	if (ts->lcd_tp_refresh_support) {
		ts->tp_refresh_wq = create_singlethread_workqueue("tp_refresh_wq");
		if (!ts->tp_refresh_wq) {
		return -1;
		}
		INIT_WORK(&ts->tp_refresh_work, lcd_tp_refresh_work);
	}

	/*step4 : mutex init*/
	mutex_init(&ts->mutex);
	mutex_init(&ts->report_mutex);
	init_completion(&ts->fw_complete);
	init_waitqueue_head(&ts->wait);
	init_waitqueue_head(&ts->notify_wait);
	ts->com_api_data.tp_irq_disable = 1;
#ifdef CONFIG_TOUCHPANEL_TRUSTED_TOUCH
	touchpanel_trusted_touch_init(ts);
#endif
	/*wake_lock_init(&ts->wakelock, WAKE_LOCK_SUSPEND, "tp_wakelock");*/
	ts->is_update_log = 0;
	init_probe_status_proc(ts);

	/*step5 : power init*/
	if(strcmp(ts->touch_environment, "tvm") != 0) {
		ret = tp_power_init(ts);
		if (ret < 0) {
			return ret;
		}

		ret = oplus_gpio_resume(ts);
		if (ret < 0) {
			return ret;
		}

		/*step6 : I2C function check*/
		if ((!ts->is_noflash_ic) && (ts->bus_type == TP_BUS_I2C)) {
				if (!i2c_check_functionality(ts->client->adapter, I2C_FUNC_I2C)) {
					TP_INFO(ts->tp_index, "%s: need I2C_FUNC_I2C\n", __func__);
					ret = -ENODEV;
					goto err_check_functionality_failed;
				}
		}
	}
	/*step7 : touch input dev init*/
	ret = init_input_device(ts);

	if (ret < 0) {
		ret = -EINVAL;
		TP_INFO(ts->tp_index, "tp_input_init failed!\n");
		goto err_check_functionality_failed;
	}

	/*step8 : irq request setting*/
	if (ts->int_mode == UNBANNABLE) {
		ret = tp_register_irq_func(ts);

		if (ret < 0) {
			goto err_check_functionality_failed;
		}

		ts->bus_ready = true;
	}

	/*step9 : panel data init*/
	ret = tp_paneldata_init(ts);

	if (ret < 0) {
		goto err_check_functionality_failed;
	}

#ifndef CONFIG_REMOVE_OPLUS_FUNCTION
	    /*step10 : FTM process*/
	    ts->boot_mode = get_boot_mode();
#endif

#if IS_ENABLED(CONFIG_TOUCHPANEL_NOTIFY)
	tp_gesture_enable_notifier = tp_gesture_enable_flag;
	tp_cs_gpio_notifier = tp_control_cs_gpio;
#endif
	if (strcmp(ts->touch_environment, "tvm") != 0) {
		/*step10 : FTM process*/
		if (is_ftm_boot_mode(ts)) {
			ts->ts_ops->ftm_process(ts->chip_data);
			ret = -EFTM;

			if (ts->int_mode == UNBANNABLE) {
				devm_free_irq(ts->dev, ts->irq, ts);
			}
			mutex_lock(&tp_core_lock);
			g_tp[ts->tp_index] = ts;
			mutex_unlock(&tp_core_lock);
			TP_INFO(ts->tp_index, "%s: not int normal mode, return.\n", __func__);
			return ret;
		}

		/*step11:get chip info*/
		if (!ts->ts_ops->get_chip_info) {
			ret = -EINVAL;
			TP_INFO(ts->tp_index, "tp get_chip_info NULL!\n");
			goto err_check_functionality_failed;
		}

		ret = ts->ts_ops->get_chip_info(ts->chip_data);

		if (ret < 0) {
			ret = -EINVAL;
			TP_INFO(ts->tp_index, "tp get_chip_info failed!\n");
			goto err_check_functionality_failed;
		}

		/*step12 : touchpanel Fw check*/
		if (!ts->is_noflash_ic) {           /*noflash don't have firmware before fw update*/
			if (!ts->ts_ops->fw_check) {
				ret = -EINVAL;
				TP_INFO(ts->tp_index, "tp fw_check NULL!\n");
				goto err_check_functionality_failed;
			}

			ret = ts->ts_ops->fw_check(ts->chip_data, &ts->resolution_info,
						   &ts->panel_data);
			if (ret == FW_ABNORMAL) {
				ts->force_update = 1;
				TP_BOOT_INFO(ts->tp_index, "This FW need to be updated!\n");

			} else {
				ts->force_update = 0;
			}
		}

		/*step13 : enable touch ic irq output ability*/
		if (!ts->ts_ops->mode_switch) {
			ret = -EINVAL;
			TP_INFO(ts->tp_index, "tp mode_switch NULL!\n");
			goto err_check_functionality_failed;
		}

		ret = mode_switch_health(ts, MODE_NORMAL, true);

		if (ret < 0) {
			ret = -EINVAL;
			TP_INFO(ts->tp_index, "%s:modem switch failed!\n", __func__);
			/*goto err_check_functionality_failed;*/   /*mode swith fail don't goto end, still keep going upgrade*/
		}
	}
	/*step14 : irq request setting*/
	if (ts->int_mode == BANNABLE) {
		ret = tp_register_irq_func(ts);

		if (ret < 0) {
			goto err_check_functionality_failed;
		}
	}

	/*step15 : suspend && resume fuction register*/

	if (strcmp(ts->touch_environment, "tvm") != 0) {
#if IS_ENABLED(CONFIG_DRM_OPLUS_PANEL_NOTIFY)
		ts->fb_notif.notifier_call = fb_notifier_callback;

		if (ts->active_panel)
			ret = drm_panel_notifier_register(ts->active_panel,
				&ts->fb_notif);
#elif IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
		if (ts->active_panel) {
			cookie = panel_event_notifier_register(GET_INDEX(ts->tp_index),
					GET_TOUCH(ts->tp_index), ts->active_panel,
					&ts_panel_notifier_callback, ts);

			if (!cookie) {
				TP_INFO(ts->tp_index, "Unable to register fb_notifier: %d\n", ret);
				goto err_check_functionality_failed;
			}
			ts->notifier_cookie = cookie;
		}

#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
	ts->disp_notifier.notifier_call = ts_mtk_drm_notifier_callback;
	if (ts->tp_index == 0) {
		if (mtk_disp_notifier_register("Oplus_touch_v2", &ts->disp_notifier)) {
			TP_INFO(ts->tp_index, "Failed to register disp notifier client!!\n");
			goto err_check_functionality_failed;
		}
	}
#if IS_ENABLED(CONFIG_OPLUS_MTK_DRM_SUB_NOTIFY)
	if (ts->tp_index == 1) {
		if (mtk_disp_sub_notifier_register("Oplus_touch_v2", &ts->disp_notifier)) {
			TP_INFO(ts->tp_index, "Failed to register sub disp notifier client!!\n");
			goto err_check_functionality_failed;
		}
	}
#endif

#elif IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)
		ts->fb_notif.notifier_call = fb_notifier_callback;
		ret = msm_drm_register_client(&ts->fb_notif);

		if (ret) {
			TP_INFO(ts->tp_index, "Unable to register fb_notifier: %d\n", ret);
			goto err_check_functionality_failed;
		}

#elif IS_ENABLED(CONFIG_FB)
		ts->fb_notif.notifier_call = fb_notifier_callback;
		ret = fb_register_client(&ts->fb_notif);

		if (ret) {
			TP_INFO(ts->tp_index, "Unable to register fb_notifier: %d\n", ret);
			goto err_check_functionality_failed;
		}
#endif/*CONFIG_FB*/
	}

	if (ts->headset_pump_support) {
		snprintf(name, TP_NAME_SIZE_MAX, "headset_pump%d", ts->tp_index);
		ts->headset_pump_wq = create_singlethread_workqueue(name);

		if (!ts->headset_pump_wq) {
			ret = -ENOMEM;
			goto error_fb_notif;
		}

		INIT_WORK(&ts->headset_pump_work, switch_headset_work);
	}

	if (ts->charger_pump_support) {
		snprintf(name, TP_NAME_SIZE_MAX, "charger_pump%d", ts->tp_index);
		ts->charger_pump_wq = create_singlethread_workqueue(name);

		if (!ts->charger_pump_wq) {
			ret = -ENOMEM;
			goto error_headset_pump;
		}

		INIT_WORK(&ts->charger_pump_work, switch_usb_state_work);
	}

	/*step16 : workqueue create(speedup_resume)*/
	snprintf(name, TP_NAME_SIZE_MAX, "sp_resume%d", ts->tp_index);
	ts->speedup_resume_wq = create_singlethread_workqueue(name);

	if (!ts->speedup_resume_wq) {
		ret = -ENOMEM;
		goto error_charger_pump;
	}

	INIT_WORK(&ts->speed_up_work, speedup_resume);
	INIT_WORK(&ts->fw_update_work, tp_fw_update_work);

#if IS_ENABLED(CONFIG_FB) || \
		IS_ENABLED(CONFIG_DRM_MSM) || \
		IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY) || \
		IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER) || \
		IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
	if (ts->suspend_work_support) {
		snprintf(name, TP_NAME_SIZE_MAX, "suspend_wq%d", ts->tp_index);
		ts->suspend_wq = create_singlethread_workqueue(name);
		if (!ts->suspend_wq) {
			ret = -ENOMEM;
			goto error_speedup_resume_wq;
		}
		INIT_WORK(&ts->suspend_work, tp_suspend_work);
	}
#endif

	/*create workqueue for key trigger */
	snprintf(name, TP_NAME_SIZE_MAX, "volume_key_trigger%d", ts->tp_index);
	ts->key_trigger_wq = create_singlethread_workqueue(name);

	if (!ts->key_trigger_wq) {
		ret = -ENOMEM;
		goto error_charger_pump;
	}

	INIT_WORK(&ts->key_trigger_work, tp_delta_read_triggered_by_key_handle);

	/*step 17:incell lcd trigger load tp fw  support*/
	if (ts->lcd_trigger_load_tp_fw_support) {
		snprintf(name, TP_NAME_SIZE_MAX, "lcd_trigger_load_tp_fw_wq%d", ts->tp_index);
		ts->lcd_trigger_load_tp_fw_wq = create_singlethread_workqueue(name);

		if (!ts->lcd_trigger_load_tp_fw_wq) {
			ret = -ENOMEM;
			goto error_speedup_resume_wq;
		}

		INIT_WORK(&ts->lcd_trigger_load_tp_fw_work, lcd_trigger_load_tp_fw);
	}

	/*step 18:esd recover support*/
	if (ts->esd_handle_support) {
		snprintf(name, TP_NAME_SIZE_MAX, "esd_workthread%d", ts->tp_index);
		ts->esd_info.esd_workqueue = create_singlethread_workqueue(name);

		if (!ts->esd_info.esd_workqueue) {
			ret = -ENOMEM;
			goto error_tp_fw_wq;
		}

		INIT_DELAYED_WORK(&ts->esd_info.esd_check_work, esd_handle_func);

		mutex_init(&ts->esd_info.esd_lock);

		ts->esd_info.esd_running_flag = 0;
		ts->esd_info.esd_work_time = 2 *
					     HZ; /* HZ: clock ticks in 1 second generated by system*/
		TP_BOOT_INFO(ts->tp_index, "Clock ticks for an esd cycle: %d\n",
			ts->esd_info.esd_work_time);

		esd_handle_switch(&ts->esd_info, true);
	}

	/*step 19:frequency hopping simulate support*/
	if (ts->freq_hop_simulate_support) {
		snprintf(name, TP_NAME_SIZE_MAX, "syna_tcm_freq_hop%d", ts->tp_index);
		ts->freq_hop_info.freq_hop_workqueue = create_singlethread_workqueue(name);

		if (!ts->freq_hop_info.freq_hop_workqueue) {
			ret = -ENOMEM;
			goto error_esd_wq;
		}

		INIT_DELAYED_WORK(&ts->freq_hop_info.freq_hop_work, tp_freq_hop_work);
		ts->freq_hop_info.freq_hop_simulating = false;
		ts->freq_hop_info.freq_hop_freq = 0;
	}

	/*initial kernel grip parameter*/
	if (ts->kernel_grip_support) {
		chip_np = is_support_child_node(ts->dev, ts);

		if (!chip_np) {
			TP_INFO(ts->tp_index, "chip_np not defined.\n");
			ts->grip_info = kernel_grip_init(ts->dev);
		} else {
			src_chip_np = ts->dev->of_node;
			ts->dev->of_node = chip_np;
			ts->grip_info = kernel_grip_init(ts->dev);
			if (!ts->grip_info) {
				TP_BOOT_INFO(ts->tp_index, "kernel grip init node failed.\n");
			}
			ts->dev->of_node = src_chip_np;
		}
	}

	if (ts->pen_support) {	/* Default to enable pen function when boot up */
			ts->is_pen_connected = 0;
			ts->is_pen_attracted = 0;
	}

	/*step 22 : createproc proc files interface*/
	init_touchpanel_proc(ts);
	init_touch_misc_device(ts);
#ifndef CONFIG_ARCH_QTI_VM
	if (ts->temperature_detect_support || ts->temperature_detect_shellback_support) {
		ts->old_temp = 0;
		if (ts->temperature_detect_shellback_support)
			ret = 0;
		else
			ret = init_get_adc_channels(ts);

		if (ret < 0)
			TP_BOOT_INFO(ts->tp_index, "init get channels failed\n");
		else {
			hrtimer_init(&ts->temp_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
			ts->temp_timer.function = ts_get_temperature_timeout;
			INIT_WORK(&ts->get_temperature_work, tp_get_temperature_work);
			hrtimer_start(&ts->temp_timer, ktime_set(15, 0), HRTIMER_MODE_REL);
		}
	}
#endif

#ifdef CONFIG_TOUCHPANEL_TRUSTED_TOUCH
	ret = touchpanel_trusted_touch_create_sysfs(ts);
	if (ret) {
		TP_INFO(ts->tp_index, "create touchpanel_trusted_touch sysfs node fail");
	}
#endif

	/*step 23 : Other*****/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	ts->ws = wakeup_source_register(dev_name(ts->dev));
#else
        ts->ws = wakeup_source_register(ts->dev,dev_name(ts->dev));
	ts->tp_wakelock = wakeup_source_register(ts->dev, "tp_wakelock");
#endif
	if (ts->fw_edge_limit_support) {
		ts->limit_enable = 1;
	}

	ts->bus_ready = true;
	ts->loading_fw = false;
	ts->hall_status = false;
	ts->is_suspended = 0;
	ts->suspend_state = TP_SPEEDUP_RESUME_COMPLETE;
	ts->gesture_enable = 0;
	ts->fd_enable = 0;
	ts->fp_enable = 0;
	ts->aiunit_game_enable = 0;
	ts->fp_info.touch_state = 0;
	ts->palm_enable = 1;
	ts->touch_count = 0;
	ts->view_area_touched = 0;
	ts->tp_suspend_order = LCD_TP_SUSPEND;
	ts->tp_resume_order = TP_LCD_RESUME;
	ts->skip_suspend_operate = false;
	ts->skip_reset_in_resume = false;
	ts->irq_slot = 0;
	ts->firmware_update_type = 0;
	ts->major_rate_limit_times = 0;
	ts->palm_to_sleep_enable = 0;
	ts->tp_ic_touch_num = 0;
	ts->last_tp_ic_touch_num = 0;
	ts->report_rate_testing = false;
	ts->report_rate_test_time = 5;
	ts->touch_frame_num = 0;
	ts->idle_freq_enable = 0;
	for (i = 0; i < MAX_FINGER_NUM; i++) {
		ts->last_x_y_point[i].x = 0;
		ts->last_x_y_point[i].y = 0;
	}

	if (ts->is_noflash_ic || ts->bus_type == TP_BUS_SPI) {
		ts->irq = ts->s_client->irq;
	} else {
		ts->irq = ts->client->irq;
	}


	mutex_lock(&tp_core_lock);
	g_tp[ts->tp_index] = ts;
	mutex_unlock(&tp_core_lock);
	TP_BOOT_INFO(ts->tp_index, "Touch panel probe : normal end\n");
	return 0;

error_esd_wq:

	if (ts->esd_handle_support) {
		destroy_workqueue(ts->esd_info.esd_workqueue);
	}

error_tp_fw_wq:

	if (ts->lcd_trigger_load_tp_fw_support) {
		destroy_workqueue(ts->lcd_trigger_load_tp_fw_wq);
	}

error_speedup_resume_wq:

	if (ts->speedup_resume_wq) {
		destroy_workqueue(ts->speedup_resume_wq);
	}

error_charger_pump:

	if (ts->charger_pump_support) {
		destroy_workqueue(ts->charger_pump_wq);
	}

error_headset_pump:

	if (ts->headset_pump_support) {
		destroy_workqueue(ts->headset_pump_wq);
	}

error_fb_notif:

#if IS_ENABLED(CONFIG_DRM_OPLUS_PANEL_NOTIFY)
	if (ts->active_panel) {
		ret = drm_panel_notifier_unregister(ts->active_panel,
			&ts->fb_notif);
		if (ret) {
			TP_INFO(ts->tp_index, "Unable to unregister fb_notifier: %d\n", ret);
		}
	}
#elif IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
	if (ts->active_panel && ts->notifier_cookie) {
		panel_event_notifier_unregister(ts->notifier_cookie);
	}
#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
	if (ts->tp_index == 0) {
		mtk_disp_notifier_unregister(&ts->disp_notifier);
	}
#if IS_ENABLED(CONFIG_OPLUS_MTK_DRM_SUB_NOTIFY)
	if (ts->tp_index == 1) {
		mtk_disp_sub_notifier_unregister(&ts->disp_notifier);
	}
#endif
#elif IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)
	msm_drm_unregister_client(&ts->fb_notif);
#elif IS_ENABLED(CONFIG_FB)
	fb_unregister_client(&ts->fb_notif);
#endif

err_check_functionality_failed:
#ifndef CONFIG_ARCH_QTI_VM
	ts->ts_ops->power_control(ts->chip_data, false);
	if (gpio_is_valid(ts->hw_res.cs_gpio)) {
		gpio_free(ts->hw_res.cs_gpio);
	}

	if (ts->exception_upload_support) {
		tp_exception_report(&ts->exception_data, EXCEP_PROBE, "register_common_touch_device", sizeof("register_common_touch_device"));
	}
#endif
	/*wake_lock_destroy(&ts->wakelock);*/
	ret = -1;
	return ret;
}
EXPORT_SYMBOL(register_common_touch_device);

void unregister_common_touch_device(struct touchpanel_data *pdata)
{
	struct touchpanel_data *ts = pdata;
#if IS_ENABLED(CONFIG_FB) || IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)
	int ret;
#endif

	if (!pdata) {
		return;
	}
	touchpanel_trusted_touch_completion(ts);

	/*step1 :free irq*/
	devm_free_irq(ts->dev, ts->irq, ts);
	/*step2 :free proc node*/
	remove_touchpanel_proc(ts);
	uninit_touch_misc_device(ts);

	/*step3 :free the hw resource*/
	pdata->ts_ops->power_control(ts->chip_data, false);

	/*lcd_tp_refresh_support*/
	if (ts->lcd_tp_refresh_support) {
		if (ts->tp_refresh_wq) {
			flush_workqueue(ts->tp_refresh_wq);
			destroy_workqueue(ts->tp_refresh_wq);
		}
	}

	/*step4:frequency hopping simulate support*/
	if (ts->freq_hop_simulate_support) {
		if (ts->freq_hop_info.freq_hop_workqueue) {
			cancel_delayed_work_sync(&ts->freq_hop_info.freq_hop_work);
			flush_workqueue(ts->freq_hop_info.freq_hop_workqueue);
			destroy_workqueue(ts->freq_hop_info.freq_hop_workqueue);
		}
	}

	/*step5:esd recover support*/
	if (ts->esd_handle_support) {
		esd_handle_switch(&ts->esd_info, false);

		if (ts->esd_info.esd_workqueue) {
			cancel_delayed_work_sync(&ts->esd_info.esd_check_work);
			flush_workqueue(ts->esd_info.esd_workqueue);
			destroy_workqueue(ts->esd_info.esd_workqueue);
		}

		mutex_destroy(&ts->esd_info.esd_lock);
	}

	/*step6 : workqueue create(speedup_resume)*/
	if (ts->speedup_resume_wq) {
		cancel_work_sync(&ts->speed_up_work);
		flush_workqueue(ts->speedup_resume_wq);
		destroy_workqueue(ts->speedup_resume_wq);
	}

#if IS_ENABLED(CONFIG_FB) || \
		IS_ENABLED(CONFIG_DRM_MSM) || \
		IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY) || \
		IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER) || \
		IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
	if (ts->suspend_work_support) {
		if (ts->suspend_wq) {
			cancel_work_sync(&ts->suspend_work);
			flush_workqueue(ts->suspend_wq);
			destroy_workqueue(ts->suspend_wq);
		}
	}
#endif

	if (ts->lcd_trigger_load_tp_fw_wq) {
		cancel_work_sync(&ts->lcd_trigger_load_tp_fw_work);
		flush_workqueue(ts->lcd_trigger_load_tp_fw_wq);
		destroy_workqueue(ts->lcd_trigger_load_tp_fw_wq);
	}

	/*step7 : suspend && resume fuction register*/
#if IS_ENABLED(CONFIG_DRM_OPLUS_PANEL_NOTIFY)
	if (ts->active_panel && ts->fb_notif.notifier_call) {
		ret = drm_panel_notifier_unregister(ts->active_panel,
			&ts->fb_notif);
		if (ret) {
			TP_INFO(ts->tp_index, "Unable to unregister fb_notifier: %d\n", ret);
		}
	}
#elif IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
	if (ts->active_panel && ts->notifier_cookie) {
		panel_event_notifier_unregister(ts->notifier_cookie);
	}
#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
	if (ts->tp_index == 0) {
		if (ts->disp_notifier.notifier_call) {
			mtk_disp_notifier_unregister(&ts->disp_notifier);
		}
	}
#if IS_ENABLED(CONFIG_OPLUS_MTK_DRM_SUB_NOTIFY)
	if (ts->tp_index == 1) {
		if (ts->disp_notifier.notifier_call) {
			mtk_disp_sub_notifier_unregister(&ts->disp_notifier);
		}
	}
#endif
#elif IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)

	if (ts->fb_notif.notifier_call) {
		ret = msm_drm_unregister_client(&ts->fb_notif);

		if (ret) {
			TP_INFO(ts->tp_index, "Unable to register fb_notifier: %d\n", ret);
		}
	}

#elif IS_ENABLED(CONFIG_FB)

	if (ts->fb_notif.notifier_call) {
		ret = fb_unregister_client(&ts->fb_notif);

		if (ret) {
			TP_INFO(ts->tp_index, "Unable to unregister fb_notifier: %d\n", ret);
		}
	}

#endif/*CONFIG_FB*/

	/*free regulator*/
	if (!IS_ERR_OR_NULL(ts->hw_res.avdd)) {
		regulator_put(ts->hw_res.avdd);
		ts->hw_res.avdd = NULL;
	}

	if (!IS_ERR_OR_NULL(ts->hw_res.vddi)) {
		regulator_put(ts->hw_res.vddi);
		ts->hw_res.vddi = NULL;
	}

	/*free wakeup source*/
	if (ts->ws) {
		wakeup_source_unregister(ts->ws);
	}

	/*wake_lock_destroy(&ts->wakelock);*/
	/*step8 : mutex init*/
	mutex_destroy(&ts->mutex);
}
EXPORT_SYMBOL(unregister_common_touch_device);

struct touchpanel_data *common_touch_data_alloc(void)
{
	return tp_kzalloc(sizeof(struct touchpanel_data), GFP_KERNEL);
}
EXPORT_SYMBOL(common_touch_data_alloc);

int common_touch_data_free(struct touchpanel_data *pdata)
{
	if (pdata) {
		if (g_tp[pdata->tp_index]) {
			g_tp[pdata->tp_index] = NULL;
		}
		kfree(pdata);
	}

	return 0;
}
EXPORT_SYMBOL(common_touch_data_free);

#if IS_ENABLED(CONFIG_FB) || \
	IS_ENABLED(CONFIG_DRM_MSM) || \
	IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY) || \
	IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER) || \
	IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
/**
 * touchpanel_ts_suspend - touchpanel suspend function
 * @dev: i2c_client->dev using to get touchpanel_data resource
 *
 * suspend function bind to LCD on/off status
 * Returning zero(sucess) or negative errno(failed)
 */

static void tp_suspend_direct(struct touchpanel_data *ts)
{
	int ret;
	u64 start_time = 0;

	TP_INFO(ts->tp_index, "tp_suspend ts->bus_ready =%d\n", ts->bus_ready);

	/*
	* block this process to wait for exiting tvm mode
	* and return to pvm successfully tp prevent crash dump
	* caused by accessing spi resources
	*/
	touchpanel_trusted_touch_completion(ts);

	/*step1:detect whether we need to do suspend*/
	if (ts->input_dev == NULL) {
		TP_INFO(ts->tp_index, "input_dev  registration is not complete\n");
		goto EXIT;
	}

	if (ts->loading_fw) {
		TP_INFO(ts->tp_index, "FW is updating while suspending");
		goto EXIT;
	}

	if (ts->health_monitor_support) {
		reset_healthinfo_time_counter(&start_time);
	}

	/* release all complete first */
	if (!ts->skip_reinit_device_support) {
		if (ts->tp_ic_type == TYPE_TDDI_TCM) {
			if (ts->ts_ops->reinit_device) {
				if (ts->bus_type != TP_BUS_I2C) {
					if (!ts->is_suspended) {
						ts->ts_ops->reinit_device(ts->chip_data);
					}
				} else {
					ts->ts_ops->reinit_device(ts->chip_data);
				}
			}
		}
	}

	/*step2:get mutex && start process suspend flow*/
	mutex_lock(&ts->mutex);

	if (!ts->is_suspended) {
		ts->is_suspended = 1;
		ts->suspend_state = TP_SUSPEND_COMPLETE;

	} else {
		TP_INFO(ts->tp_index, "%s: do not suspend twice.\n", __func__);
		goto EXIT;
	}
	if ((ts->temperature_detect_support && ts->skin_therm_chan) || ts->temperature_detect_shellback_support)
		hrtimer_cancel(&ts->temp_timer);

	/*step3:Release key && touch event before suspend*/
	tp_btnkey_release(ts);
	tp_touch_release(ts);

	/*step4:cancel esd test*/
	if (ts->esd_handle_support) {
		esd_handle_switch(&ts->esd_info, false);
	}

	if (!ts->is_incell_panel || (ts->black_gesture_support
				     && ts->gesture_enable > 0)) {
		/*step5:gamde mode support*/
		if (ts->game_switch_support) {
			mode_switch_health(ts, MODE_GAME, false);
		}

		if (ts->report_rate_white_list_support && ts->ts_ops->rate_white_list_ctrl) {
			ts->ts_ops->rate_white_list_ctrl(ts->chip_data, 0);
		}

		if (ts->face_detect_support && ts->fd_enable) {
			mode_switch_health(ts, MODE_FACE_DETECT, false);
		}
	}

	/*step6:finger print support handle*/
	if (ts->fingerprint_underscreen_support) {
		operate_mode_switch(ts);
		if (!ts->fingerprint_not_report_in_suspend) {
			ts->fp_info.touch_state = 0;
			touch_call_notifier_fp(ts, &ts->fp_info);
		} else {
			TP_INFO(ts->tp_index, "%s, not report fp up for S3908 & S3910\n", __func__);
		}
		goto EXIT;
	}

	/*step7:gesture mode status process*/
	if (ts->black_gesture_support) {
		if (CHK_BIT(ts->gesture_enable, 0x01) && (!ts->hall_status)) {
			mode_switch_health(ts, MODE_GESTURE, ts->gesture_enable);
			goto EXIT;
		}
	}

	/*step for suspend_gesture_cfg when ps is near ts->gesture_enable == 2*/
	if (ts->suspend_gesture_cfg && ts->black_gesture_support
	    && ts->gesture_enable == 0x02 && (!ts->hall_status)) {
		mode_switch_health(ts, MODE_GESTURE, true);
		operate_mode_switch(ts);
		goto EXIT;
	}

	/*step8:skip suspend operate only when gesture_enable is 0*/
	if (ts->skip_suspend_operate && (!ts->gesture_enable)) {
		goto EXIT;
	}

	if (ts->fp_grip_support && (ts->fp_grip_enable & FP_GRIP_ENABLE)) {
		TP_INFO(ts->tp_index, "%s, enable fp grip\n", __func__);
		mode_switch_health(ts, MODE_GESTURE, true);
		goto EXIT;
	}

	/*step9:switch mode to sleep*/
	ret = mode_switch_health(ts, MODE_SLEEP, true);

	if (ret < 0) {
		TP_INFO(ts->tp_index, "%s, Touchpanel operate mode switch failed\n", __func__);
	}

EXIT:

	if (ts->health_monitor_support) {
		tp_healthinfo_report(&ts->monitor_data, HEALTH_SUSPEND, &start_time);
		if (false == ts->bus_ready) {
			ts->monitor_data.bus_not_ready_tp_suspend_count++;
		}
	}

	if (ts->en_touch_event_helper) {
		post_message(ts->msg_list, 0, TYPE_SUSPEND, NULL);
	}

	ts->is_hall_near_resume = false;

	TP_INFO(ts->tp_index, "%s: end.\n", __func__);
	mutex_unlock(&ts->mutex);
}
static int tp_suspend(struct device *dev)
{
	struct touchpanel_data *ts = NULL;

	ts = dev_get_drvdata(dev);
	if (ts->suspend_work_support) {
		queue_work(ts->suspend_wq, &ts->suspend_work);
	} else {
		tp_suspend_direct(ts);
	}
	return 0;
}

static void tp_suspend_work(struct work_struct *work)
{
	struct touchpanel_data *ts = NULL;

	ts = container_of(work, struct touchpanel_data, suspend_work);
	TP_INFO(ts->tp_index, "%s: start.\n", __func__);

	tp_suspend_direct(ts);
}

/**
 * touchpanel_ts_suspend - touchpanel resume function
 * @dev: i2c_client->dev using to get touchpanel_data resource
 *
 * resume function bind to LCD on/off status, this fuction start thread to speedup screen on flow.
 * Do not care the result: Return void type
 */
static void tp_resume(struct device *dev)
{
	struct touchpanel_data *ts = dev_get_drvdata(dev);
	struct device_node *chip_np = NULL;
	struct device_node *src_chip_np = NULL;
	TP_INFO(ts->tp_index, "%s start.\n", __func__);

	if (!ts->is_suspended) {
		TP_INFO(ts->tp_index, "%s: do not resume twice.\n", __func__);
		if (ts->incell_aod_gesture_support) {
			ts->incell_aod_flag = false;
		} else {
			goto NO_NEED_RESUME;
		}
	}

	touchpanel_trusted_touch_completion(ts);

	ts->is_suspended = 0;
	ts->suspend_state = TP_RESUME_COMPLETE;
	ts->disable_gesture_ctrl = false;

	if (ts->loading_fw) {
		goto NO_NEED_RESUME;
	}

	/*free irq at first*/
	if (!(ts->tp_ic_type == TYPE_TDDI_TCM && ts->is_noflash_ic)) {
		if (ts->int_mode == UNBANNABLE) {
			mutex_lock(&ts->mutex);
		}
		devm_free_irq(ts->dev, ts->irq, ts);
		if (ts->int_mode == UNBANNABLE) {
			mutex_unlock(&ts->mutex);
		}
	}

	if (!ts->skip_reinit_device_support) {
		if (ts->tp_ic_type == TYPE_TDDI_TCM) {
			if (ts->ts_ops->reinit_device) {
				ts->ts_ops->reinit_device(ts->chip_data);
			}
		}
	}

	if (ts->kernel_grip_support) {
		if (ts->grip_info) {
			kernel_grip_reset(ts->grip_info);

		} else {
			chip_np = is_support_child_node(ts->dev, ts);
			if (!chip_np) {
				TP_INFO(ts->tp_index, "chip_np not defined.\n");
				ts->grip_info = kernel_grip_init(ts->dev);
			} else {
				src_chip_np = ts->dev->of_node;
				ts->dev->of_node = chip_np;
				ts->grip_info = kernel_grip_init(ts->dev);
				ts->dev->of_node = src_chip_np;
			}
			init_kernel_grip_proc(ts->prEntry_tp, ts->grip_info);
		}
	}

	queue_work(ts->speedup_resume_wq, &ts->speed_up_work);
	return;

NO_NEED_RESUME:
	ts->suspend_state = TP_SPEEDUP_RESUME_COMPLETE;
}
#endif
/**
 * speedup_resume - speedup resume thread process
 * @work: work struct using for this thread
 *
 * do actully resume function
 * Do not care the result: Return void type
 */
static void speedup_resume(struct work_struct *work)
{
	int ret = 0;
	u64 start_time = 0;
	struct specific_resume_data specific_resume_data;
	struct touchpanel_data *ts = container_of(work, struct touchpanel_data,
				     speed_up_work);

	TP_INFO(ts->tp_index, "%s is called\n", __func__);
	if (ts->tp_wakelock) {
		__pm_stay_awake(ts->tp_wakelock);
	}

	/*step1: get mutex for locking i2c acess flow*/
	mutex_lock(&ts->mutex);

	if (ts->health_monitor_support) {
		reset_healthinfo_time_counter(&start_time);
	}

	/*step2:before Resume clear All of touch/key event Reset some flag to default satus*/
	tp_btnkey_release(ts);
	tp_touch_release(ts);

	if (ts->force_bus_ready_support && (false == ts->bus_ready)) {
		TP_INFO(ts->tp_index, "%s force bus_ready to true\n", __func__);
		ts->bus_ready = true;
		if (ts->health_monitor_support) {
			ts->monitor_data.force_bus_ready_count++;
		}
	}

	if (!(ts->tp_ic_type == TYPE_TDDI_TCM && ts->is_noflash_ic)) {
		if (ts->int_mode == UNBANNABLE) {
			tp_register_irq_func(ts);
		}
	}

	if (ts->ts_ops->speed_up_resume_prepare) {
		ts->ts_ops->speed_up_resume_prepare(ts->chip_data);
	}
	/*step3:Reset IC && switch work mode, ft8006 is reset by lcd, no more reset needed*/
	if (!ts->skip_reset_in_resume && !ts->fp_info.touch_state) {
		if (!ts->lcd_trigger_load_tp_fw_support) {
			if (ts->ts_ops->fingerprint_health_info && ts->fp_enable) {
				ts->ts_ops->fingerprint_health_info(ts->chip_data);
			}
			ts->ts_ops->reset(ts->chip_data);
		}
	}

	if (ts->ts_ops->specific_resume_operate && !ts->fp_info.touch_state) {
		specific_resume_data.suspend_state = ts->suspend_state;
		specific_resume_data.in_test_process = ts->in_test_process;
		ret =  ts->ts_ops->specific_resume_operate(ts->chip_data,
				&specific_resume_data);

		if (ret < 0) {
			goto EXIT;
		}
	}

	/*when fingerprint quick start featrue on ,need keep fp_enable = 1 befor tp resume,now need turn off*/
	if(ts->fp_disable_after_resume && ts->fp_quick_start_data == 1) {
		ts->fp_quick_start_data = 0;
		ts->fp_enable = 0;
	}

	if (ts->incell_aod_gesture_support) {
		ts->is_suspended = 0;
	}

	operate_mode_switch(ts);

	if (ts->esd_handle_support) {
		esd_handle_switch(&ts->esd_info, true);
	}

	/*step6:Request irq again*/
	if (!(ts->tp_ic_type == TYPE_TDDI_TCM && ts->is_noflash_ic)) {
		if (ts->int_mode == BANNABLE) {
			tp_register_irq_func(ts);
		}
	}

EXIT:
	ts->suspend_state = TP_SPEEDUP_RESUME_COMPLETE;
	if ((ts->temperature_detect_support && ts->skin_therm_chan) || ts->temperature_detect_shellback_support)
		hrtimer_start(&ts->temp_timer, ktime_set(15, 0), HRTIMER_MODE_REL);

	if (ts->health_monitor_support) {
		tp_healthinfo_report(&ts->monitor_data, HEALTH_RESUME, &start_time);
	}

	if (ts->fp_grip_support && true == ts->fp_grip_hold) {
		TP_INFO(ts->tp_index, "finger hold after resume!! need release\n");
		ts->fp_grip_hold = false;
		touch_call_fp_grip(ts, 0);
		tp_healthinfo_report(&ts->monitor_data, HEALTH_REPORT, "finger_hold_in_resume");
	}

	ts->is_hall_near_resume = ts->hall_status;

	/*step7:Unlock  && exit*/
	TP_INFO(ts->tp_index, "%s: end!\n", __func__);
	mutex_unlock(&ts->mutex);
	if (ts->tp_wakelock) {
		__pm_relax(ts->tp_wakelock);
	}
}

#if IS_ENABLED(CONFIG_FB) || \
	IS_ENABLED(CONFIG_DRM_MSM) || \
	IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY) || \
	IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER) || \
	IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
static void lcd_off_early_event(struct touchpanel_data *ts)
{
	ts->suspend_state = TP_SUSPEND_EARLY_EVENT;      /*set suspend_resume_state*/
	ts->notify_state = NOTIFY_BLANK_EARLY_ENTER;

	if (ts->health_monitor_support && (false == ts->bus_ready)) {
		ts->monitor_data.bus_not_ready_off_early_event_count++;
	}

	if (ts->esd_handle_support && ts->is_incell_panel
	    && (ts->tp_suspend_order == LCD_TP_SUSPEND)) {
		esd_handle_switch(&ts->esd_info, false);
		/*incell panel need cancel esd early*/
	}

	if (ts->tp_suspend_order == TP_LCD_SUSPEND) {
		tp_suspend(ts->dev);

	} else if (ts->tp_suspend_order == LCD_TP_SUSPEND) {
		if (!ts->gesture_enable && ts->is_incell_panel) {
			disable_irq_nosync(ts->irq);
		}
	}

	ts->notify_state = NOTIFY_BLANK_EARLY_EXIT;

	if (ts->bus_ready_check_support) {
		wake_up_interruptible(&ts->notify_wait);
	}
};

static void lcd_off_event(struct touchpanel_data *ts)
{
	ts->notify_state = NOTIFY_BLANK_ENTER;

	if (ts->health_monitor_support && (false == ts->bus_ready)) {
		ts->monitor_data.bus_not_ready_off_event_count++;
	}

	if (ts->tp_suspend_order == TP_LCD_SUSPEND) {
	} else if (ts->tp_suspend_order == LCD_TP_SUSPEND) {
		tp_suspend(ts->dev);
	}

	ts->notify_state = NOTIFY_BLANK_EXIT;
	if (ts->bus_ready_check_support) {
		wake_up_interruptible(&ts->notify_wait);
	}
};

static void lcd_on_early_event(struct touchpanel_data *ts)
{
	ts->suspend_state = TP_RESUME_EARLY_EVENT;        /*set suspend_resume_state*/

	if (ts->health_monitor_support && (false == ts->bus_ready)) {
		ts->monitor_data.bus_not_ready_early_event_count++;
	}

	if (ts->tp_resume_order == TP_LCD_RESUME) {
		tp_resume(ts->dev);

	} else if (ts->tp_resume_order == LCD_TP_RESUME) {
		if (!(ts->tp_ic_type == TYPE_TDDI_TCM && ts->is_noflash_ic)) {
			disable_irq_nosync(ts->irq);
		}
	}
};

static void lcd_on_event(struct touchpanel_data *ts)
{
	if (ts->health_monitor_support && (false == ts->bus_ready)) {
		ts->monitor_data.bus_not_ready_event_count++;
	}

	if (ts->tp_resume_order == LCD_TP_RESUME) {
		tp_resume(ts->dev);
		if (!(ts->tp_ic_type == TYPE_TDDI_TCM && ts->is_noflash_ic)) {
			enable_irq(ts->irq);
		}
	}
};

static void lcd_other_event(int *blank, struct touchpanel_data *ts)
{
	if (*blank == LCD_CTL_TP_LOAD_FW) {
		lcd_tp_load_fw(ts->tp_index);
	} else if (*blank == LCD_CTL_RST_ON) {
		tp_control_reset_gpio(1, ts->tp_index);
	} else if (*blank == LCD_CTL_RST_OFF) {
		tp_control_reset_gpio(0, ts->tp_index);
	} else if (*blank == LCD_CTL_TP_FTM) {
		tp_ftm_extra(ts->tp_index);
	} else if (*blank == LCD_CTL_TP_FPS60) {
		lcd_tp_refresh_switch(ts->tp_index, 60);
	} else if (*blank == LCD_CTL_TP_FPS90) {
		lcd_tp_refresh_switch(ts->tp_index, 90);
	} else if (*blank == LCD_CTL_CS_ON) {
		tp_control_cs_gpio(1, ts->tp_index);
	} else if (*blank == LCD_CTL_CS_OFF) {
		tp_control_cs_gpio(0, ts->tp_index);
	} else if (*blank == LCD_CTL_IRQ_ON) {
		tp_control_irq_state(1, ts->tp_index);
	} else if (*blank == LCD_CTL_IRQ_OFF) {
		tp_control_irq_state(0, ts->tp_index);
	} else if (*blank == LCD_CTL_AOD_OFF) {
		ts->incell_aod_flag = false;
	} else if (*blank == LCD_CTL_AOD_ON) {
		ts->incell_aod_flag = true;
	}
};

#if IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
static void ts_panel_notifier_callback(enum panel_event_notifier_tag tag,
		 struct panel_event_notification *notification, void *client_data)
{
	struct touchpanel_data *ts = client_data;

	if (!notification) {
		TP_INFO(ts->tp_index, "Invalid notification\n");
		return;
	}
	if (notification->notif_type <= DRM_PANEL_EVENT_FOR_TOUCH) {
		TP_INFO(ts->tp_index, "Notification type:%d, early_trigger:%d",
			notification->notif_type,
			notification->notif_data.early_trigger);
	}

	if (ts->bus_ready == false) {
		if (ts->health_monitor_support) {
			ts->monitor_data.bus_not_ready_notify_count++;
		}
		if (ts->bus_ready_check_support == true) {
			TP_INFO(ts->tp_index, "bus_ready not ready, tp exit\n");
			return;
		}
	}

	switch (notification->notif_type) {
	case DRM_PANEL_EVENT_UNBLANK:
		if (notification->notif_data.early_trigger) {
			if ((ts->suspend_work_support) && (ts->suspend_wq)) {
				flush_workqueue(ts->suspend_wq);        /*wait suspend_wq done*/
			}
			lcd_on_early_event(ts);
		} else {
			lcd_on_event(ts);
		}
		break;
	case DRM_PANEL_EVENT_BLANK:
		if (notification->notif_data.early_trigger) {
			if (ts->incell_aod_gesture_support) {
				ts->is_suspended = 0;
			}
			if (ts->speedup_resume_wq) {
				flush_workqueue(ts->speedup_resume_wq);        /*wait speedup_resume_wq done*/
			}
			lcd_off_early_event(ts);
		} else {
			lcd_off_event(ts);
		}
		break;
	case DRM_PANEL_EVENT_BLANK_LP:
		TP_INFO(ts->tp_index, "received lp event\n");
		if (!notification->notif_data.early_trigger) {
			lcd_off_event(ts);
		} else if (ts->tp_lcd_suspend_in_lp_support == true) {
			TP_INFO(ts->tp_index, "tp suspend before lcd set lp mode\n");
			if (ts->speedup_resume_wq) {
				flush_workqueue(ts->speedup_resume_wq);       /*wait speedup_resume_wq done*/
			}
			lcd_off_early_event(ts);
		}
		break;
	case DRM_PANEL_EVENT_FPS_CHANGE:
		TP_INFO(ts->tp_index, "shashank:Received fps change old fps:%d new fps:%d\n",
				notification->notif_data.old_fps,
				notification->notif_data.new_fps);
		break;
	case DRM_PANEL_EVENT_FOR_TOUCH:
		lcd_other_event(notification->notif_data.lcd_ctl_blank, ts);
		break;
	default:
		if (notification->notif_type <= DRM_PANEL_EVENT_FOR_TOUCH) {
			TP_INFO(ts->tp_index, "notification serviced :%d\n",
				notification->notif_type);
		}
		break;
	}
}

#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
static int ts_mtk_drm_notifier_callback(struct notifier_block *nb,
	unsigned long event, void *data)
{
	struct touchpanel_data *ts = container_of(nb, struct touchpanel_data,
		disp_notifier);
	int *blank = (int *)data;

	TP_INFO(ts->tp_index, "mtk gki notifier event:%lu, blank:%d",
			event, *blank);

	switch (event) {
	case MTK_DISP_EARLY_EVENT_BLANK:
		if (*blank == MTK_DISP_BLANK_UNBLANK) {
			if ((ts->suspend_work_support) && (ts->suspend_wq)) {
				flush_workqueue(ts->suspend_wq);        /*wait suspend_wq done*/
			}
			lcd_on_early_event(ts);
		} else if (*blank == MTK_DISP_BLANK_POWERDOWN) {
			if (ts->incell_aod_gesture_support) {
				ts->is_suspended = 0;
			}
			if (ts->speedup_resume_wq) {
				flush_workqueue(ts->speedup_resume_wq);		/*wait speedup_resume_wq done*/
			}
			lcd_off_early_event(ts);
		}
	break;
	case MTK_DISP_EVENT_BLANK:
		if (*blank == MTK_DISP_BLANK_UNBLANK) {
			lcd_on_event(ts);
		} else if (*blank == MTK_DISP_BLANK_POWERDOWN) {
			lcd_off_event(ts);
		}
	break;
	default:
		TP_INFO(ts->tp_index, "nuknown event :%lu\n", event);
		lcd_other_event(blank, ts);
	break;
	}

	return 0;
}

#else
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	int *blank;
#if IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)
	struct msm_drm_notifier *evdata = data;
#else
	struct fb_event *evdata = data;
#endif

	struct touchpanel_data *ts = container_of(self, struct touchpanel_data,
				     fb_notif);
	if (!ts) {
		return 0;
	}
	touchpanel_trusted_touch_completion(ts);

	/*to aviod some kernel bug (at fbmem.c some local veriable are not initialized)*/
#if IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)
	if (event != MSM_DRM_EARLY_EVENT_BLANK && event != MSM_DRM_EVENT_BLANK
	    && event != MSM_DRM_EVENT_FOR_TOUCH)
#else
	if (event != FB_EARLY_EVENT_BLANK && event != FB_EVENT_BLANK
	    && event != FB_EVENT_BLANK_FOR_TOUCH)
#endif
		return 0;

	if (evdata && evdata->data && ts && ts->chip_data) {
		blank = evdata->data;
		TP_INFO(ts->tp_index, "%s: event = %ld, blank = %d\n", __func__, event, *blank);
#if IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)

		if (*blank == MSM_DRM_BLANK_POWERDOWN) { /*suspend*/
			if (event == MSM_DRM_EARLY_EVENT_BLANK) {    /*early event*/
#else
		if (*blank == FB_BLANK_POWERDOWN) { /*suspend*/
			if (event == FB_EARLY_EVENT_BLANK) {    /*early event*/
#endif
				if (ts->speedup_resume_wq) {
					flush_workqueue(ts->speedup_resume_wq);        /*wait speedup_resume_wq done*/
				}

				lcd_off_early_event(ts);
#if IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)

			} else if (event == MSM_DRM_EVENT_BLANK) {   /*event*/
#else
			} else if (event == FB_EVENT_BLANK) {   /*event*/
#endif
				lcd_off_event(ts);
			}

#if IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)

		} else if (*blank == MSM_DRM_BLANK_UNBLANK) { /*resume*/
			if (event == MSM_DRM_EARLY_EVENT_BLANK) {    /*early event*/
#else
		} else if (*blank == FB_BLANK_UNBLANK) {  /*resume*/
			if (event == FB_EARLY_EVENT_BLANK) {    /*early event*/
#endif
				lcd_on_early_event(ts);

#if IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)

			} else if (event == MSM_DRM_EVENT_BLANK) {   /*event*/
#else
			} else if (event == FB_EVENT_BLANK) {   /*event*/
#endif
				lcd_on_event(ts);
			}
#if IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)
		} else if (event == MSM_DRM_EVENT_FOR_TOUCH) {   //event
#else
		} else if (event == FB_EVENT_BLANK_FOR_TOUCH) {   //event
#endif
			lcd_other_event(blank, ts);
		}
	}

	return 0;
}
#endif
#endif

/*
 * tp_shutdown - touchpanel shutdown function
 * @ts: ts using to get touchpanel_data resource
 * shutdown function is called when power off or reboot
 * Returning void
 */
void tp_shutdown(struct touchpanel_data *ts)
{
	if (!ts) {
		return;
	}

#ifdef CONFIG_TOUCHPANEL_TRUSTED_TOUCH
#ifndef CONFIG_ARCH_QTI_VM
	if (atomic_read(&ts->trusted_touch_enabled) == 1) {
		TP_INFO(ts->tp_index, "%s: Now is in tvm mode, do not control power resource in LA side\n", __func__);
		return;
	}
#endif
#endif

#ifdef CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY
	TP_INFO(ts->tp_index, "mtk gki2.0 need to unregister notifier");
	if (ts->tp_index == 0) {
		if (mtk_disp_notifier_unregister(&ts->disp_notifier)) {
			TP_INFO(ts->tp_index, "Failed to unregister mtk gki 2.0 disp notifier!!\n");
		}
	}
#endif
#ifdef CONFIG_OPLUS_MTK_DRM_SUB_NOTIFY
	if (ts->tp_index == 1) {
		if (mtk_disp_sub_notifier_unregister(&ts->disp_notifier)) {
			TP_INFO(ts->tp_index, "Failed to  unregister mtk gki 2.0 disp sub notifier!!\n");
		}
	}
#endif

#ifdef CONFIG_QCOM_PANEL_EVENT_NOTIFIER
	TP_INFO(ts->tp_index, "qcom gki2.0 need to unregister notifier");
	if (ts->active_panel && ts->notifier_cookie) {
		panel_event_notifier_unregister(ts->notifier_cookie);
	}
#endif
	/*step0 :close esd*/
	if (ts->esd_handle_support) {
		esd_handle_switch(&ts->esd_info, false);
	}
	oplus_gpio_suspend(ts);
	/*step1 :free the hw resource*/
	ts->ts_ops->power_control(ts->chip_data, false);
}
EXPORT_SYMBOL(tp_shutdown);

/*
 * wait_for_notify_suspend - touchpanel wait_for_notify_suspend function
 * @ts: ts using to get touchpanel_data resource
 * function is called when system go to sleep after notify suspend process
 * Returning void
 */

static void wait_for_notify_suspend(struct touchpanel_data *ts)
{
	if (!ts) {
		return;
	}

	if ((ts->notify_state == NOTIFY_BLANK_EARLY_ENTER) || (ts->notify_state == NOTIFY_BLANK_ENTER)) {
		wait_event_interruptible_timeout(ts->notify_wait,
						(ts->notify_state == NOTIFY_BLANK_EARLY_EXIT) || (ts->notify_state == NOTIFY_BLANK_EXIT),
						msecs_to_jiffies(NOTIFY_TIME_OUT));
		TP_INFO(ts->tp_index, "%s enter,notify_state %d!!\n", __func__, ts->notify_state);
		if (ts->health_monitor_support) {
			ts->monitor_data.wait_for_notify_suspend_count++;
		}
		return;
	}
}

/*
 * tp_pm_suspend - touchpanel pm suspend function
 * @ts: ts using to get touchpanel_data resource
 * suspend function is called when system go to sleep
 * Returning void
 */
void tp_pm_suspend(struct touchpanel_data *ts)
{
	if (!ts) {
		return;
	}
	touchpanel_trusted_touch_completion(ts);

	ts->bus_ready = false;

	if (ts->health_monitor_support) {
		ts->monitor_data.pm_suspend_count++;
	}

	ts->suspend_task = get_current();

	if (ts->bus_ready_check_support) {
		wait_for_notify_suspend(ts);
	}

	if (TP_ALL_GESTURE_SUPPORT) {
		if (TP_ALL_GESTURE_ENABLE) {
			/*enable gpio wake system through interrupt*/
			enable_irq_wake(ts->irq);
			return;
		}
	}

	disable_irq(ts->irq);
}
EXPORT_SYMBOL(tp_pm_suspend);

/*
 * tp_pm_resume - touchpanel pm resume function
 * @ts: ts using to get touchpanel_data resource
 * resume function is called when system wake up
 * Returning void
 */
void tp_pm_resume(struct touchpanel_data *ts)
{
	if (!ts) {
		return;
	}
	touchpanel_trusted_touch_completion(ts);
	if (TP_ALL_GESTURE_SUPPORT) {
		if (TP_ALL_GESTURE_ENABLE) {
			/*disable gpio wake system through intterrupt*/
			disable_irq_wake(ts->irq);
			goto OUT;
		}
	}

	enable_irq(ts->irq);

OUT:
	ts->bus_ready = true;

	if (ts->health_monitor_support) {
		ts->monitor_data.pm_resume_count++;
	}

	if ((ts->black_gesture_support || ts->fingerprint_underscreen_support)) {
		if ((ts->gesture_enable == 1 || ts->fp_enable)) {
			wake_up_interruptible(&ts->wait);
		}
	}
}
EXPORT_SYMBOL(tp_pm_resume);

void clear_view_touchdown_flag(unsigned int tp_index)
{
	struct touchpanel_data *ts = NULL;

	if (tp_index >= TP_SUPPORT_MAX) {
		return;
	}
	ts = get_ts_data(tp_index);

	if (ts) {
		ts->view_area_touched = 0;
	}
}
EXPORT_SYMBOL(clear_view_touchdown_flag);

static oem_verified_boot_state oem_verifiedbootstate =
	OEM_VERIFIED_BOOT_STATE_LOCKED;
bool is_oem_unlocked(void)
{
	return (oem_verifiedbootstate == OEM_VERIFIED_BOOT_STATE_UNLOCKED);
}
EXPORT_SYMBOL(is_oem_unlocked);

#ifndef CONFIG_REMOVE_OPLUS_FUNCTION
#if IS_MODULE(CONFIG_TOUCHPANEL_OPLUS)
extern char verified_bootstate[];
#endif
int get_oem_verified_boot_state(void)
{
#if IS_BUILTIN(CONFIG_TOUCHPANEL_OPLUS)
    if (strstr(saved_command_line, "androidboot.verifiedbootstate=orange")) {
        oem_verifiedbootstate = OEM_VERIFIED_BOOT_STATE_UNLOCKED;
    } else {
        oem_verifiedbootstate = OEM_VERIFIED_BOOT_STATE_LOCKED;
    }
#else
    if (strstr(verified_bootstate, "orange")) {
        oem_verifiedbootstate = OEM_VERIFIED_BOOT_STATE_UNLOCKED;
    } else {
        oem_verifiedbootstate = OEM_VERIFIED_BOOT_STATE_LOCKED;
    }
#endif
    return 0;
}
EXPORT_SYMBOL(get_oem_verified_boot_state);
#endif

/*******Part4:Extern Function  Area********************************/
static void lcd_trigger_load_tp_fw(struct work_struct *work)
{
	struct touchpanel_data *ts = container_of(work, struct touchpanel_data,
				     lcd_trigger_load_tp_fw_work);
	static bool is_running = false;
	u64 start_time = 0;

	if (ts->lcd_trigger_load_tp_fw_support) {
		if (is_running) {
			TP_INFO(ts->tp_index, "%s is running, can not repeat\n", __func__);

		} else {
			TP_INFO(ts->tp_index, "%s start\n", __func__);

			if (ts->health_monitor_support) {
				reset_healthinfo_time_counter(&start_time);
			}

			is_running = true;
			mutex_lock(&ts->mutex);
			ts->ts_ops->reset(ts->chip_data);
			mutex_unlock(&ts->mutex);
			is_running = false;

			if (ts->health_monitor_support) {
				tp_healthinfo_report(&ts->monitor_data, HEALTH_FW_UPDATE_COST, &start_time);
			}
		}
	}
}

#if IS_ENABLED(CONFIG_FB) || \
	IS_ENABLED(CONFIG_DRM_MSM) || \
	IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY) || \
	IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER) || \
	IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
static void lcd_tp_load_fw(unsigned int tp_index)
{
	struct touchpanel_data *ts = NULL;

	if (tp_index >= TP_SUPPORT_MAX) {
		return;
	}

	ts = get_ts_data(tp_index);

	if (!ts) {
		return;
	}

	touchpanel_trusted_touch_completion(ts);

	TP_INFO(ts->tp_index, "%s\n", __func__);

	if (ts->irq_trigger_hdl_support) {
		TP_INFO(ts->tp_index, "%s\n", __func__);
		devm_free_irq(ts->dev, ts->irq, ts);
		tp_register_irq_func(ts);

	} else if (ts->lcd_trigger_load_tp_fw_support) {
		TP_INFO(ts->tp_index, "%s\n", __func__);
		ts->disable_gesture_ctrl = true;

		if (ts->ts_ops) {
			if (ts->ts_ops->tp_queue_work_prepare) {
				mutex_lock(&ts->mutex);
				ts->ts_ops->tp_queue_work_prepare(ts->chip_data);
				mutex_unlock(&ts->mutex);
			}
		}

		queue_work(ts->lcd_trigger_load_tp_fw_wq, &(ts->lcd_trigger_load_tp_fw_work));
	}
}

/*
*Interface for lcd to control reset pin
*/
static int tp_control_reset_gpio(bool enable, unsigned int tp_index)
{
	struct touchpanel_data *ts = NULL;

	if (tp_index >= TP_SUPPORT_MAX) {
		return 0;
	}

	ts = get_ts_data(tp_index);

	if (!ts) {
		return 0;
	}

	if (gpio_is_valid(ts->hw_res.reset_gpio)) {
		if (ts->ts_ops->reset_gpio_control) {
			ts->ts_ops->reset_gpio_control(ts->chip_data, enable);
		}
	}

	return 0;
}

/*
*Interface for tp in ftm mode
*/
static void tp_ftm_extra(unsigned int tp_index)
{
	struct touchpanel_data *ts = NULL;

	if (tp_index >= TP_SUPPORT_MAX) {
		return;
	}

	ts = get_ts_data(tp_index);

	if (!ts) {
		return;
	}
	if (ts->ts_ops) {
		if (ts->ts_ops->ftm_process_extra) {
			ts->ts_ops->ftm_process_extra(ts->chip_data);
		}
	}
	return;
}

static void lcd_tp_refresh_switch(unsigned int tp_index, int fps)
{
	struct touchpanel_data *ts = NULL;

	if (tp_index >= TP_SUPPORT_MAX) {
		return;
	}

	ts = get_ts_data(tp_index);

	if (!ts) {
		return;
	}

	if (is_ftm_boot_mode(ts)) {
		return;
	}

	if (ts->lcd_tp_refresh_support) {
		TP_INFO(ts->tp_index, "%s:fps:%d\n", __func__, fps);
		ts->lcd_fps = fps;
		if (ts->ts_ops) {
			if (ts->ts_ops->tp_refresh_switch && !ts->is_suspended
				&& (ts->suspend_state == TP_SPEEDUP_RESUME_COMPLETE)
				&& !ts->loading_fw) {
				queue_work(ts->tp_refresh_wq, &ts->tp_refresh_work);
			}
		}
	}

}

static int tp_control_irq_state(bool enable, unsigned int tp_index)
{
	struct touchpanel_data *ts = NULL;

	if (tp_index >= TP_SUPPORT_MAX) {
		return 0;
	}
	ts = get_ts_data(tp_index);

	if (!ts) {
		return 0;
	}

	TP_INFO(ts->tp_index, "%s %d, %s ts->irq=%d\n", __func__, enable,
		enable ? "enable" : "disable", ts->irq);
	if (enable == 1) {
		enable_irq(ts->irq);
		TP_INFO(ts->tp_index, "%s: enable_irq.\n", __func__);
	} else {
		disable_irq_nosync(ts->irq);
		TP_INFO(ts->tp_index, "%s: disable_irq_nosync.\n", __func__);
	}

	return 0;
}
#endif/*CONFIG_FB*/

static int tp_control_cs_gpio(bool enable, unsigned int tp_index)
{
	struct touchpanel_data *ts = NULL;
	int rc = 0;

	if (tp_index >= TP_SUPPORT_MAX) {
		return 0;
	}

	ts = get_ts_data(tp_index);

	if (!ts) {
		return 0;
	}

	if (gpio_is_valid(ts->hw_res.cs_gpio)) {
		rc = gpio_direction_output(ts->hw_res.cs_gpio, enable);
		if (rc) {
			TP_INFO(ts->tp_index, "unable to set dir for cs_gpio rc=%d", rc);
		}
		gpio_set_value(ts->hw_res.cs_gpio, enable);
		TP_INFO(ts->tp_index, "%s:set cs %d\n", __func__, enable);
	}
	if (!IS_ERR_OR_NULL(ts->hw_res.pin_cs_high) && enable) {
		TP_INFO(ts->tp_index, "%s: going to set cs on gpio mode and high.\n", __func__);
		pinctrl_select_state(ts->hw_res.pinctrl, ts->hw_res.pin_cs_high);
	} else if (!IS_ERR_OR_NULL(ts->hw_res.pin_cs_low) && !enable) {
		TP_INFO(ts->tp_index, "%s: going to set cs on gpio mode and low.\n", __func__);
		pinctrl_select_state(ts->hw_res.pinctrl, ts->hw_res.pin_cs_low);
	} else {
		TP_INFO(ts->tp_index, "%s: not to contrl cs.\n", __func__);
	}

	return 0;
}

/**
 * tp_gesture_enable_flag -   expose gesture control status for other module.
 * Return gesture_enable status.
 */
#if IS_ENABLED(CONFIG_TOUCHPANEL_NOTIFY)
static int tp_gesture_enable_flag(unsigned int tp_index)
{
	struct touchpanel_data *ts = NULL;

	if (tp_index >= TP_SUPPORT_MAX) {
		return LCD_POWER_OFF;
	}

	ts = get_ts_data(tp_index);

	if (!ts || !ts->is_incell_panel) {
		return LCD_POWER_OFF;
	}

	TP_INFO(ts->tp_index, "gesture_enable is %d\n", ts->gesture_enable);

	return (ts->gesture_enable > 0) ? LCD_POWER_ON : LCD_POWER_OFF;
}
#endif

static void lcd_tp_refresh_work(struct work_struct *work)
{
	struct touchpanel_data *ts = container_of(work, struct touchpanel_data,
				     tp_refresh_work);

	if (ts->int_mode == BANNABLE) {
		mutex_lock(&ts->mutex);
	}
	if (!ts->is_suspended && (ts->suspend_state == TP_SPEEDUP_RESUME_COMPLETE)
		&& !ts->loading_fw) {
		ts->ts_ops->tp_refresh_switch(ts->chip_data, ts->lcd_fps);
	}
	if (ts->int_mode == BANNABLE) {
		mutex_unlock(&ts->mutex);
	}
}
MODULE_DESCRIPTION("Touchscreen common Driver");
MODULE_LICENSE("GPL");
