// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/input/mt.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/syscalls.h>
#include <linux/version.h>
#include <linux/regulator/consumer.h>

#include "touchpanel_common.h"
#include "touchpanel_prevention/touchpanel_prevention.h"
#include "touchpanel_autotest/touchpanel_autotest.h"
#include "touch_comon_api/touch_comon_api.h"
#include "touchpanel_tui_support/touchpanel_tui_support.h"

#ifndef CONFIG_REMOVE_OPLUS_FUNCTION
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
#include<mt-plat/mtk_boot_common.h>
#else
#include <soc/oplus/system/boot_mode.h>
#endif
#endif

#define NORMAL_MODE  0
#define GESTURE_MODE 1
#define SLEEP_MODE   2

#define TP_MAJOR_RATE_1_16  0x0F

#define ENABLE_HW_RES_AVDD   0
#define DISABLE_HW_RES_AVDD  1
#define ENABLE_HW_RES_VDDI   2
#define DISABLE_HW_RES_VDDI  3
#define ENABLE_HW_RESET      4
#define DISABLE_HW_RESET     5

/*******Part1:LOG TAG Declear************************/

/*******Part2:declear Area********************************/
#ifndef CONFIG_REMOVE_OPLUS_FUNCTION
extern int register_devinfo(char *name, struct manufacture_info *info);
extern int register_device_proc(char *name, char *version, char *manufacture);
#endif

extern bool is_ftm_boot_mode(struct touchpanel_data *ts);
extern int cur_tp_index;

/* return value
 * 0 : normal
 * 1 : gesture
 * 2 : sleep
 */
static int tp_status(struct touchpanel_data *ts)
{
    if (!!ts) {
        if (ts->suspend_state == TP_SPEEDUP_RESUME_COMPLETE) {
            return NORMAL_MODE;
        } else if (TP_ALL_GESTURE_ENABLE) {
            return GESTURE_MODE;
        } else {
            return SLEEP_MODE;
        }
    }
    return -1;
}

static int tp_irq_check(struct touchpanel_data *ts)
{
	struct irq_desc *desc = NULL;

	if (!ts) {
		return 0;
	}

	desc = irq_to_desc(ts->irq);
	if (!desc || desc->depth) {
		return 0;
	}

	return 1;
}

static void tp_set_fp_error_report(struct touchpanel_data *ts, unsigned int debug_level)
{
	if (!ts) {
		TPD_INFO("%s: ts is NULL\n", __func__);
		return;
	}

	if (!ts->ts_ops || !ts->ts_ops->set_fp_error_report) {
		TS_TP_INFO("%s: ts->ts_ops or ts->ts_ops->set_fp_error_report is NULL\n", __func__);
		return;
	}

	TP_INFO(ts->tp_index, "%s: set_fp_error_report value=%d\n", __func__, debug_level);

	ts->ts_ops->set_fp_error_report(ts->chip_data, (debug_level > LEVEL_BASIC) ? 1 : 0);
}

/*******Part3:Function node Function  Area********************/
/*oplus_optimized_time - For optimized time*/
static ssize_t proc_optimized_time_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[6] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		TPD_INFO("%s error:file_inode.\n", __func__);
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 5);

	if (kstrtoint(buf, 10, &value)) {
		TS_TP_INFO("%s: kstrtoint error\n", __func__);
		return count;
	}

	if (0 == value) {
		ts->total_operate_times = 0;        /*clear total count*/
	}

	return count;
}

static ssize_t proc_optimized_time_read(struct file *file,
					char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		snprintf(page, PAGESIZE - 1, "%d\n", -1); /*error handler*/

	} else {
		snprintf(page, PAGESIZE - 1, "%u\n",
			 ts->single_optimized_time * ts->total_operate_times);
	}

	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}


DECLARE_PROC_OPS(proc_optimized_time_fops, simple_open, proc_optimized_time_read, proc_optimized_time_write, NULL);

/*/proc/touchpanel/kernel_grip_default_para*/
static int tp_grip_default_para_read(struct seq_file *s, void *v)
{
	int ret = 0;
	int is_default_xml_exit = 0;
	struct touchpanel_data *ts = s->private;
	const struct firmware *fw = NULL;

	char *p_node_img = NULL;
	char *p_node_default_xml = NULL;
	char *grip_config_name_img = NULL;
	char *grip_config_name_default_xml = NULL;
	char *postfix_img = "_sys_edge_touch_config.img";
	char *postfix_default_xml = "/sys_edge_touch_config.xml";
	uint8_t copy_len = 0;

	TPD_INFO("%s:s->size:%lu,s->count:%lu\n", __func__, s->size,
		 s->count);

	if (!ts) {
		return 0;
	}

	if (s->size <= (PAGE_SIZE * 4)) {
		s->count = s->size;
		return 0;
	}

	grip_config_name_img = kzalloc(MAX_FW_NAME_LENGTH, GFP_KERNEL);
	if (grip_config_name_img == NULL) {
		TS_TP_INFO("grip_config_name_img kzalloc error!\n");
		return 0;
	}
	p_node_img = strstr(ts->panel_data.fw_name, ".");
	if (p_node_img == NULL) {
		TS_TP_INFO("p_node_img strstr error!\n");
		kfree(grip_config_name_img);
		return 0;
	}

	grip_config_name_default_xml = kzalloc(MAX_FW_NAME_LENGTH, GFP_KERNEL);
	if (grip_config_name_default_xml == NULL) {
		TS_TP_INFO("grip_config_name_default_xml kzalloc error!\n");
		return 0;
	}
	p_node_default_xml = strstr(strstr(ts->panel_data.fw_name, "/") + 1, "/");
	if (p_node_default_xml == NULL) {
		TS_TP_INFO("p_node_default_xml strstr error!\n");
		kfree(grip_config_name_default_xml);
		return 0;
	}

	copy_len = p_node_img - ts->panel_data.fw_name;
	memcpy(grip_config_name_img, ts->panel_data.fw_name, copy_len);
	strlcat(grip_config_name_img, postfix_img, MAX_FW_NAME_LENGTH);

	TS_TP_INFO("grip_config_name_img is %s\n", grip_config_name_img);

	copy_len = p_node_default_xml - ts->panel_data.fw_name;
	memcpy(grip_config_name_default_xml, ts->panel_data.fw_name, copy_len);
	strlcat(grip_config_name_default_xml, postfix_default_xml, MAX_FW_NAME_LENGTH);

	TS_TP_INFO("grip_config_name_default_xml is %s\n", grip_config_name_default_xml);

	ret = request_firmware(&fw, grip_config_name_default_xml, ts->dev);
	if (ret < 0) {
		TS_TP_INFO("Request firmware failed - %s (%d)\n", grip_config_name_default_xml, ret);
	} else {
		TS_TP_INFO("%s Request ok,size is:%lu\n", grip_config_name_default_xml, fw->size);
	}


	is_default_xml_exit = ret;
	if (is_default_xml_exit < 0) {
		ret = request_firmware(&fw, grip_config_name_img, ts->dev);
		if (ret < 0) {
			TS_TP_INFO("Request firmware failed - %s (%d)\n", grip_config_name_img, ret);
			seq_printf(s, "Request failed, Check the path %s\n", grip_config_name_img);
			seq_printf(s, "Request failed, Check the path %s\n", grip_config_name_default_xml);
			kfree(grip_config_name_img);
			kfree(grip_config_name_default_xml);
			return 0;
		}

		TS_TP_INFO("%s Request ok,size is:%lu\n", grip_config_name_img, fw->size);
	}


	if (fw->size > 0) {
		seq_write(s, fw->data, fw->size);
		TS_TP_INFO("%s:seq_write data ok\n", __func__);
	}

	release_firmware(fw);
	kfree(grip_config_name_img);
	kfree(grip_config_name_default_xml);
	return ret;
}

static int tp_grip_default_para_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_grip_default_para_read, PDE_DATA(inode));
}

DECLARE_PROC_OPS(tp_grip_default_para_fops, tp_grip_default_para_open, seq_read, NULL, single_release);


/*tp_index - For read current tp index
 * Output:
 * tp_index: current pointer tp touchpanel_data;
 */
static ssize_t proc_index_control_read(struct file *file, char __user *buffer,
				       size_t count, loff_t *ppos)
{
	uint8_t ret = 0;
	char page[PAGESIZE] = {0};

	TPD_INFO("%s: cur_tp_index = %d.\n", __func__, cur_tp_index);
	snprintf(page, PAGESIZE - 1, "%d\n", cur_tp_index);
	ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));

	return ret;
}

/*tp_index - For set current tp index
 * Output:
 * tp_index: current pointer tp touchpanel_data;
 */
static ssize_t proc_index_control_write(struct file *file,
					const char __user *buffer, size_t count, loff_t *ppos)
{
	int tmp = 0;
	char buf[4] = {0};

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 2);

	if (kstrtoint(buf, 10, &tmp)) {
		TPD_INFO("%s: kstrtoint error\n", __func__);
		return count;
	}

	cur_tp_index = tmp;
	return count;
}

DECLARE_PROC_OPS(proc_tp_index_ops, simple_open, proc_index_control_read, proc_index_control_write, NULL);

static int tp_gesture_support_read(struct seq_file *s, void *v)
{
	int ret = 0;
	struct touchpanel_data *ts = s->private;

	if (!ts) {
		TPD_INFO("%s : ts is NULL", __func__);
		goto OUT;
	}

	if (ts->sportify_aod_gesture_support) {
		ret = BIT(SINGLE_TAP);
		TS_TP_INFO("%s: sporitify aod support : %d", __func__, ret);
		seq_printf(s, "%d,\n", ret);
	} else {
		TS_TP_INFO("%s: not supprot special gesture type : %d", __func__, ret);
		seq_printf(s, "%d,\n", ret);
	}
OUT:
	return 0;
}

static int tp_gesture_support_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_gesture_support_read, PDE_DATA(inode));
}

DECLARE_PROC_OPS(tp_gesture_support_fops, tp_gesture_support_open, seq_read, NULL, single_release);

/* /proc/touchpanelx (0/1) /s/palm_to_sleep */
#define PALM_BUF_SIZE 10
#define PALM_BUF_MAX  10
static ssize_t proc_palm_to_sleep_read(struct file *file, char __user *buffer,
				       size_t count, loff_t *ppos)
{
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	uint8_t ret = 0;
	char buf[PALM_BUF_SIZE] = {0};

	if (!ts) {
		TPD_INFO("%s ts is null \n", __func__);
		goto OUT;
	}

	snprintf(buf, PALM_BUF_SIZE - 1, "%d", ts->palm_to_sleep_enable);
	ret = simple_read_from_buffer(buffer, count, ppos, buf, strlen(buf));
	TPD_DETAIL("%s: get palm_to_sleep\n", __func__);

OUT:
	return ret;
}

static ssize_t proc_palm_to_sleep_write(struct file *file,
					const char __user *buffer, size_t count, loff_t *ppos)
{
	int tmp = 0;
	char buf[PALM_BUF_SIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		TPD_INFO("%s ts is null \n", __func__);
		goto OUT;
	}

	if (count > PALM_BUF_MAX) {
		TS_TP_INFO("%s over size cmd is %lu\n", __func__, count);
		goto OUT;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, PALM_BUF_SIZE);

	if (kstrtoint(buf, 10, &tmp)) {
		TS_TP_INFO("%s: kstrtoint error,buf:%s\n", __func__, buf);
		goto OUT;
	}

	ts->palm_to_sleep_enable = tmp;

	TPD_DETAIL("%s: set palm_to_sleep:%d\n", __func__, ts->palm_to_sleep_enable);

OUT:
	return count;
}

DECLARE_PROC_OPS(tp_palm_to_sleep_fops, simple_open, proc_palm_to_sleep_read, proc_palm_to_sleep_write, NULL);

static ssize_t proc_palm_to_sleep_support_read(struct file *file, char __user *buffer,
				       size_t count, loff_t *ppos)
{
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	uint8_t ret = 0;
	char buf[PALM_BUF_SIZE] = {0};

	if (!ts) {
		TPD_INFO("%s ts is null \n", __func__);
		goto OUT;
	}

	snprintf(buf, PALM_BUF_SIZE - 1, "%d\n", ts->palm_to_sleep_support);
	ret = simple_read_from_buffer(buffer, count, ppos, buf, strlen(buf));
	TPD_DETAIL("%s: get palm_to_sleep\n", __func__);

OUT:
	return ret;
}

static ssize_t proc_palm_to_sleep_support_write(struct file *file,
					const char __user *buffer, size_t count, loff_t *ppos)
{
	int tmp = 0;
	char buf[PALM_BUF_SIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		TPD_INFO("%s ts is null \n", __func__);
		goto OUT;
	}

	if (count > PALM_BUF_MAX) {
		TS_TP_INFO("%s over size cmd is %lu\n", __func__, count);
		goto OUT;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, PALM_BUF_SIZE);

	if (kstrtoint(buf, 10, &tmp)) {
		TS_TP_INFO("%s: kstrtoint error,buf:%s\n", __func__, buf);
		goto OUT;
	}

	ts->palm_to_sleep_support = !!tmp;

	TPD_DETAIL("%s: set palm_to_sleep_support:%d\n", __func__, ts->palm_to_sleep_support);

OUT:
	return count;
}

DECLARE_PROC_OPS(tp_palm_to_sleep_support_fops, simple_open, proc_palm_to_sleep_support_read, proc_palm_to_sleep_support_write, NULL);

static ssize_t fp_grip_support_read(struct file *file, char __user *buffer,
				       size_t count, loff_t *ppos)
{
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	uint8_t ret = 0;
	char buf[50] = {0};

	if (!ts) {
		TPD_INFO("%s ts is null \n", FP_GRIP_TAG);
		goto OUT;
	}

	snprintf(buf, 50, "%s enabld:%d\n", FP_GRIP_TAG, ts->fp_grip_enable);
	ret = simple_read_from_buffer(buffer, count, ppos, buf, strlen(buf));
	TPD_DETAIL("%s: get fp_grip_support\n", FP_GRIP_TAG);

OUT:
	return ret;
}

static ssize_t fp_grip_support_write(struct file *file,
			const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[PALM_BUF_SIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		TPD_INFO("%s ts is null \n", FP_GRIP_TAG);
		goto OUT;
	}

	if (count > PALM_BUF_MAX) {
		TS_TP_INFO("%s over size cmd is %lu\n", FP_GRIP_TAG, count);
		goto OUT;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, PALM_BUF_SIZE);

	if (kstrtoint(buf, 10, &value)) {
		TS_TP_INFO("%s: kstrtoint error,buf:%s\n", FP_GRIP_TAG, buf);
		goto OUT;
	}

	TS_TP_INFO("%s:ready to set fingerprint %d, suspend:%d gesture_enable:%d\n", FP_GRIP_TAG, value, ts->is_suspended, ts->gesture_enable);

	mutex_lock(&ts->mutex);

	if (ts->fp_grip_support) {
		ts->fp_grip_enable = (value & FP_GRIP_ENABLE) | (1 << FP_GESTURE_HOLD);
		ts->ts_ops->set_gesture_state(ts->chip_data, ts->fp_grip_enable);
		if (value & FP_GRIP_ENABLE) {
			tp_healthinfo_report(&ts->monitor_data, HEALTH_REPORT, "fp_grip_enable_cnt");
		} else if (value & FP_GRIP_DISABLE) {
			tp_healthinfo_report(&ts->monitor_data, HEALTH_REPORT, "fp_grip_disable_cnt");
			if (!ts->gesture_enable && ts->is_suspended) {
				TS_TP_INFO("%s:not gesture,need to sleep\n", FP_GRIP_TAG);
				ts->ts_ops->mode_switch(ts->chip_data, MODE_SLEEP, true);
			} else if (ts->gesture_enable && ts->is_suspended) {
				TS_TP_INFO("%s:in gesture,need to gesture\n", FP_GRIP_TAG);
				ts->ts_ops->mode_switch(ts->chip_data, MODE_GESTURE, true);
			}
		}

		if (!(value & FP_GRIP_ENABLE)) {
			ts->fp_grip_hold = false;
			touch_call_fp_grip(ts, 0);
		}
	}

	mutex_unlock(&ts->mutex);

OUT:
	return count;
}

DECLARE_PROC_OPS(fp_grip_support_ops, simple_open, fp_grip_support_read, fp_grip_support_write, NULL);

/*proc/touchpanel/fp_unlock_status*/
static ssize_t proc_fp_unlock_status_write(struct file *file, const char __user *buffer,
				  size_t count, loff_t *ppos)
{
	int ret = 0;
	int value = 0;
	char buf[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		TPD_INFO("%s: ts is NULL\n", __func__);
		return count;
	}

	if (!ts->ts_ops->fp_unlock_status_write) {
		TS_TP_INFO("not support ts_ops->fp_unlock_status_write callback\n");
		return count;
	}

	if (!ts->fp_unlock_status_support) {
		TS_TP_INFO("not support fp_unlock_status_support.\n");
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 4);

	if (kstrtoint(buf, 10, &value)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	if (value < 0 || value > 255) {
		TP_INFO(ts->tp_index, "%s: value:%d is error\n", __func__, value);
		return count;
	}

	mutex_lock(&ts->mutex);
	ts->fp_unlock_status = value;
	TP_INFO(ts->tp_index, "%s: fp_unlock_status value=%d\n", __func__, value);
	ret = ts->ts_ops->fp_unlock_status_write(ts->chip_data, ts->fp_unlock_status);
	if (ret < 0) {
		TS_TP_INFO("%s, Touchpanel fp_unlock_status_write failed\n", __func__);
	}
	mutex_unlock(&ts->mutex);

	return count;
}

static ssize_t proc_fp_unlock_status_read(struct file *file, char __user *buffer,
				 size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		snprintf(page, PAGESIZE - 1, "%d\n", -1); /*no support*/
		TPD_INFO("fp_unlock_status is no support \n");
		ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));
		return ret;
	} else {
		/*support*/
		snprintf(page, PAGESIZE - 1, "%d.\n", ts->fp_unlock_status);
		ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));
		return ret;
	}
}

DECLARE_PROC_OPS(fp_unlock_status_ops, simple_open,
		  proc_fp_unlock_status_read, proc_fp_unlock_status_write, NULL);

static ssize_t proc_waterproof_read(struct file *file, char __user *buffer,
				       size_t count, loff_t *ppos)
{
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	uint8_t ret = 0;
	char buf[PALM_BUF_SIZE] = {0};

	if (!ts) {
		TPD_INFO("%s ts is null \n", __func__);
		goto OUT;
	}

	snprintf(buf, PALM_BUF_SIZE - 1, "%d\n", ts->waterproof);
	ret = simple_read_from_buffer(buffer, count, ppos, buf, strlen(buf));
	TPD_DETAIL("%s: get palm_to_sleep\n", __func__);

OUT:
	return ret;
}

static ssize_t proc_waterproof_write(struct file *file,
					 const char __user *buffer, size_t count, loff_t *ppos)
{
	int tmp = 0;
	int oldwaterproof = 0;
	char buf[PALM_BUF_SIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		TPD_INFO("%s ts is null \n", __func__);
		goto OUT;
	}

	if (count > PALM_BUF_MAX) {
		TS_TP_INFO("%s over size cmd is %lu\n", __func__, count);
		goto OUT;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, PALM_BUF_SIZE);

	if (kstrtoint(buf, 10, &tmp)) {
		TS_TP_INFO("%s: kstrtoint error,buf:%s\n", __func__, buf);
		goto OUT;
	}
	oldwaterproof = ts->waterproof;
	if ((ts->waterproof >> WATERPROOF_RUS_BIT) & 0x1) { /* RUS status*/
		if ((tmp >> WATERPROOF_RUS_BIT) & 0x1) { /* RUS command*/
			if (tmp & 0x1) {
				ts->waterproof = tmp & ~(0x1 << WATERPROOF_RUS_BIT);
				TPD_INFO("%s: exit RUS set WATERPROOF_NODE:%s  waterproof:%d\n",
				__func__, buf, ts->waterproof);
			} else {
				ts->waterproof = tmp;
			}
		} else {
			TPD_INFO("%s: RUS skip to set WATERPROOF_NODE:%s  waterproof:%d\n",
			__func__, buf, ts->waterproof);
			goto OUT;
		}
	} else {
		if (((tmp >> WATERPROOF_RUS_BIT) & 0x1) && (tmp & 0x1)) {
			ts->waterproof = tmp & ~(0x1 << WATERPROOF_RUS_BIT);
		} else {
			ts->waterproof = tmp;
		}
	}

	if (ts->ts_ops->mode_switch) {
		if (ts->ts_ops->mode_switch(ts->chip_data, MODE_WATERPROOF, ts->waterproof & ~(0x1 << WATERPROOF_RUS_BIT)) < 0) {
			TPD_INFO("%s:set waterproof failed. value = %d.", __func__, ts->waterproof & ~(0x1 << WATERPROOF_RUS_BIT));
			ts->waterproof = oldwaterproof;
			return count;
		}
	}
	else {
		TPD_INFO("%s:waterproof set not support.", __func__);
	}

OUT:
	return count;
}

DECLARE_PROC_OPS(tp_waterproof_fops, simple_open, proc_waterproof_read, proc_waterproof_write, NULL);

/*debug_level - For touch panel driver debug_level
 * Output:
 * tp_debug:0, LEVEL_BASIC;
 * tp_debug:1, LEVEL_DETAIL;
 * tp_debug:2, LEVEL_DEBUG;
 */
static ssize_t proc_debug_level_read(struct file *file, char __user *buffer,
				     size_t count, loff_t *ppos)
{
	uint8_t ret = 0;
	char page[PAGESIZE] = {0};

	TPD_INFO("%s: tp_debug = %u.\n", __func__, tp_debug);
	snprintf(page, PAGESIZE - 1, "%u", tp_debug);
	ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));

	return ret;
}

/*debug_level - For touch panel driver debug_level
 * Input:
 * tp_debug:0, LEVEL_BASIC;
 * tp_debug:1, LEVEL_DETAIL;
 * tp_debug:2, LEVEL_DEBUG;
 */
static ssize_t proc_debug_level_write(struct file *file,
				      const char __user *buffer, size_t count, loff_t *ppos)
{

	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	int tmp = 0;
	char buf[4] = {0};

	if (!ts) {
		TPD_INFO("%s: ts is NULL\n", __func__);
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 2);

	if (kstrtoint(buf, 10, &tmp)) {
		TPD_INFO("%s: kstrtoint error\n", __func__);
		return count;
	}

	tp_debug = tmp;
	touch_misc_state_change(PDE_DATA(file_inode(file)), IOC_STATE_DEBUG_LEVEL, tp_debug);

	mutex_lock(&ts->mutex);
	if (ts->fingerprint_error_report_support) {
		tp_set_fp_error_report(ts, tp_debug);
	}
	mutex_unlock(&ts->mutex);

#ifdef CONFIG_OPLUS_TP_APK

	if (ts && ts->apk_op && ts->apk_op->apk_debug_set) {
		if (tp_debug == 0) {
			ts->apk_op->apk_debug_set(ts->chip_data, false);

		} else {
			ts->apk_op->apk_debug_set(ts->chip_data, true);
		}
	}

#endif /* end of CONFIG_OPLUS_TP_APK*/

	return count;
}

DECLARE_PROC_OPS(proc_debug_level_ops, simple_open, proc_debug_level_read, proc_debug_level_write, NULL);

static ssize_t tp_communicate_test_read(struct file *file, char __user *buffer,
				size_t count, loff_t *ppos)
{
	int ret = 0;
	int result = TEST_RST_NG;
	int i = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = NULL;

	ts = PDE_DATA(file_inode(file));
	if (!ts) {
		TPD_INFO("%s ts is null \n", __func__);
		snprintf(page, PAGESIZE - 1, "%d", result);
		goto OUT;
	}

	for(i = 0; i < ts->communicate_test_cnt; i++) {
		if(ts->ts_ops->communicate_test) {
			ret = ts->ts_ops->communicate_test(ts->chip_data);
			if(ret) {
				TPD_INFO("%s: communicate_test  failed.\n", __func__);
				snprintf(page, PAGESIZE - 1, "%d", result);
				goto OUT;
			}
		} else {
			TPD_INFO("%s: communicate_test not support.\n", __func__);
			snprintf(page, PAGESIZE - 1, "%d", result);
			goto OUT;
		}
	}

	result = TEST_RST_OK;
	snprintf(page, PAGESIZE - 1, "%d", result);
OUT:
	ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t tp_communicate_test_write(struct file *file,
			const char __user *buffer, size_t count, loff_t *ppos)
{
	int tmp = 0;
	char buf[8] = {0};
	struct touchpanel_data *ts = NULL;

	ts = PDE_DATA(file_inode(file));
	if (!ts) {
		TPD_INFO("%s ts is null \n", __func__);
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 8);
	if (kstrtoint(buf, 10, &tmp)) {
		TPD_INFO("%s: kstrtoint error\n", __func__);
		ts->communicate_test_cnt = 100;
		return count;
	}

	ts->communicate_test_cnt = tmp;
	TPD_INFO("Get communicate_test cnt %d.\n", tmp);
	return count;
}

DECLARE_PROC_OPS(tp_communicate_test_ops, simple_open, tp_communicate_test_read, tp_communicate_test_write, NULL);

/*double_tap_enable - For black screen gesture
 * Input:
 * gesture_enable = 0 : disable gesture
 * gesture_enable = 1 : enable gesture when ps is far away
 * gesture_enable = 2 : disable gesture when ps is near
 * gesture_enable = 3 : enable single tap gesture when ps is far away
 * value = 5 : hall status is far way
 * value = 6 : hall status is near
 */
static ssize_t proc_gesture_control_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 2);

	if (kstrtoint(buf, 10, &value)) {
		TS_TP_INFO("%s: kstrtoint error\n", __func__);
		return count;
	}

	if (ts->gesture_test_support && ts->gesture_test.flag) {
		return count;
	}

	mutex_lock(&ts->mutex);

	switch(value) {
	case 0:
	case 1:
	case 2:
	case 3:
		if (ts->gesture_enable != value) {
			ts->gesture_enable = value;
			TP_INFO(ts->tp_index, "%s: gesture_enable = %d, is_suspended = %d\n", __func__,
				ts->gesture_enable, ts->is_suspended);

		if (0 == ts->gesture_enable && ts->is_suspended && ts->fp_grip_enable && ts->fp_grip_hold == true && ts->fp_grip_support) {
			TP_INFO(ts->tp_index, "%s: maybe prox near to disable gesture,nead send release touch to fp\n", __func__);
			touch_call_fp_grip(ts, 0);
		}

		if (ts->is_incell_panel && (ts->suspend_state == TP_RESUME_EARLY_EVENT
					    || ts->disable_gesture_ctrl) && (ts->tp_resume_order == LCD_TP_RESUME)) {
			TS_TP_INFO("tp will resume, no need mode_switch in incell panel\n"); /*avoid i2c error or tp rst pulled down in lcd resume*/

		} else if (ts->is_suspended) {
			if (ts->bus_ready == false) {
				if (ts->health_monitor_support) {
					ts->monitor_data.bus_not_ready_gesture_write_count++;
				}
			}
			if (ts->fingerprint_underscreen_support && ts->fp_enable
					&& ts->ts_ops->enable_gesture_mask) {
				ts->ts_ops->enable_gesture_mask(ts->chip_data,
								(ts->gesture_enable & 0x01) == 1);

			} else {
				operate_mode_switch(ts);
			}
		}

	}
	break;
	case 4:
	break;
	case 5:
		ts->hall_status = false;
		if (ts->ts_ops->notify_keyboard_open && ts->is_hall_near_resume) {
			ts->ts_ops->notify_keyboard_open(ts->chip_data);
			ts->is_hall_near_resume = false;
			if (ts->health_monitor_support) {
				tp_healthinfo_report(&ts->monitor_data, HEALTH_REPORT, "hall_near_resume");
			}
		}
		break;
	case 6:
		ts->hall_status = true;
		if ((ts->gesture_enable & 0x01) && ts->is_suspended)
			operate_mode_switch(ts);
		break;
	default:
		TPD_DEBUG("invalid setting %d\n", value);
	}
	TPD_INFO("%s: gesture_enable = %d, value = %d, hall_status = %d\n", __func__, ts->gesture_enable, value, ts->hall_status);

	mutex_unlock(&ts->mutex);

	return count;
}

/*double_tap_enable - For black screen gesture
 * Output:
 * gesture_enable = 0 : disable gesture
 * gesture_enable = 1 : enable gesture when ps is far away
 * gesture_enable = 2 : disable gesture when ps is near
 */
static ssize_t proc_gesture_control_read(struct file *file, char __user *buffer,
		size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		return 0;
	}

	TP_DEBUG(ts->tp_index, "double tap enable is: %d\n", ts->gesture_enable);
	ret = snprintf(page, PAGESIZE - 1, "%d\n", ts->gesture_enable);
	ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));

	return ret;
}

DECLARE_PROC_OPS(proc_gesture_control_fops, simple_open, proc_gesture_control_read, proc_gesture_control_write, NULL);

/*
 *    each bit cant enable or disable each gesture
 *    bit0: 1 for enable bit gesture, 0 for disable bit gesture
 *    bit1: 1 for enable bit gesture, 0 for disable bit gesture
 *    bit2: 1 for enable bit gesture, 0 for disable bit gesture
 */
static ssize_t proc_gesture_control_indep_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[9] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (count > 8) {
		return count;
	}

	if (!ts) {
		return count;
	}
	touchpanel_trusted_touch_completion(ts);

	if (copy_from_user(buf, buffer, count)) {
		TS_TP_INFO("%s: read proc input error.\n", __func__);
		return count;
	}
	if (sscanf(buf, "%d", &value) != 1) {
		TS_TP_INFO("%s: sscanf error.\n", __func__);
		return -EINVAL;
	}
	TS_TP_INFO("%s: value is %x.\n", __func__, value);

	mutex_lock(&ts->mutex);

	if (ts->ts_ops->set_gesture_state) {
		ts->gesture_enable_indep = value;
		ts->ts_ops->set_gesture_state(ts->chip_data, value);
	}

	if (ts->sportify_aod_gesture_support &&
		ts->is_suspended &&
		!!ts->gesture_enable) {
		TS_TP_INFO("%s:[SPORITFY AOD enable]set %d gesture\n", __func__, value);
		ts->ts_ops->mode_switch(ts->chip_data, MODE_GESTURE, true);
	}

	mutex_unlock(&ts->mutex);

	return count;
}

static ssize_t proc_gesture_control_indep_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		return 0;
	}

	if (!ts->ts_ops) {
		return 0;
	}

	TPD_DEBUG("gesture gesture_enable_indep is: %x\n", ts->gesture_enable_indep);
	ret = snprintf(page, PAGESIZE - 1, "%x\n", (unsigned int)(ts->gesture_enable_indep));
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

	return ret;
}


DECLARE_PROC_OPS(proc_gesture_control_indep_fops, simple_open, proc_gesture_control_indep_read, proc_gesture_control_indep_write, NULL);

/*coordinate - For black screen gesture coordinate*/
static ssize_t proc_coordinate_read(struct file *file, char __user *buffer,
				    size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		return 0;
	}

	TP_DEBUG(ts->tp_index, "%s:gesture_type=%u,gesture_panel_id=%u\n", __func__,
		ts->gesture.gesture_type, ts->gesture.gesture_panel_id);

	tp_healthinfo_report(&ts->monitor_data, HEALTH_GESTURE_READ, &ts->gesture.gesture_type);

	ret = snprintf(page, PAGESIZE - 1,
				"%u,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%u,%u\n", ts->gesture.gesture_type,
				ts->gesture.Point_start.x, ts->gesture.Point_start.y, ts->gesture.Point_end.x,
				ts->gesture.Point_end.y,
				ts->gesture.Point_1st.x,   ts->gesture.Point_1st.y,   ts->gesture.Point_2nd.x,
				ts->gesture.Point_2nd.y,
				ts->gesture.Point_3rd.x,   ts->gesture.Point_3rd.y,   ts->gesture.Point_4th.x,
				ts->gesture.Point_4th.y,
				ts->gesture.clockwise,
				ts->gesture.gesture_panel_id);
	ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));

	return ret;
}

DECLARE_PROC_OPS(proc_coordinate_fops, simple_open, proc_coordinate_read, NULL, NULL);

#define HIGH_FRAME_RATE "high_frame_rate"
/*game_switch_enable
 * Input:
 * noise_level:0, disable the game_switch;
 * noise_level:a, enable the game_switch;
 */
static ssize_t proc_game_switch_write(struct file *file,
				      const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[101] = {0};
        char *ptr = NULL;
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		TPD_INFO("%s: ts is NULL\n", __func__);
		return count;
	}

	touchpanel_trusted_touch_completion(ts);
	if (!ts->ts_ops->mode_switch) {
		TS_TP_INFO("%s:not support ts_ops->mode_switch callback\n", __func__);
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 100);
	ptr = strstr(buf, HIGH_FRAME_RATE);
	if (ptr) {
		if (sscanf(ptr+sizeof(HIGH_FRAME_RATE), "%d%d", &value, &ts->high_frame_rate_time)) {
			if(ts->ts_ops->set_high_frame_rate && ts->high_frame_rate_support) {
				mutex_lock(&ts->mutex);
				ts->high_frame_value = value;
				ts->ts_ops->set_high_frame_rate(ts->chip_data, value, ts->high_frame_rate_time);
				mutex_unlock(&ts->mutex);
			}
		}
	}

	if (kstrtoint(buf, 16, &value)) {
		TS_TP_INFO("%s: kstrtoint error\n", __func__);
		return count;
	}

	ts->noise_level = value;
	touch_misc_state_change(ts, IOC_STATE_GAME, value);
	if (ts->health_monitor_support) {
		ts->monitor_data.in_game_mode = value;
	}

	TP_INFO(ts->tp_index, "%s: game_switch value=0x%x\n", __func__, value);

	if (!ts->is_suspended) {
		mutex_lock(&ts->mutex);
		ts->ts_ops->mode_switch(ts->chip_data, MODE_GAME, value > 0);

		if (value > 0) {
			if (!ts->smooth_level_array_support && ts->ts_ops->smooth_lv_set)
				ts->ts_ops->smooth_lv_set(ts->chip_data, ts->smooth_level_default);
			if (!ts->sensitive_level_array_support && ts->ts_ops->sensitive_lv_set)
				ts->ts_ops->sensitive_lv_set(ts->chip_data, ts->sensitive_level_default);
			if (ts->major_rate_limit_support)
				ts->major_rate_limit_times = TP_MAJOR_RATE_1_16;
		} else {
			if (!ts->smooth_level_array_support && ts->ts_ops->smooth_lv_set)
				ts->ts_ops->smooth_lv_set(ts->chip_data, 0);
			if (!ts->sensitive_level_array_support && ts->ts_ops->sensitive_lv_set)
				ts->ts_ops->sensitive_lv_set(ts->chip_data, 0);
			if (ts->major_rate_limit_support)
				ts->major_rate_limit_times = 0;
		}
		mutex_unlock(&ts->mutex);

	} else {
		TP_INFO(ts->tp_index, "%s: game_switch_support is_suspended.\n", __func__);
	}

	return count;
}

static ssize_t proc_game_switch_read(struct file *file, char __user *buffer,
				     size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		snprintf(page, PAGESIZE - 1, "%d\n", -1); /*no support*/

	} else {
		snprintf(page, PAGESIZE - 100, "%d, high_frame_support:%d, high_frame_value:%d\n", ts->noise_level, ts->high_frame_rate_support, ts->high_frame_value); /*support*/
	}

	ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));
	return ret;
}

DECLARE_PROC_OPS(proc_game_switch_fops, simple_open, proc_game_switch_read, proc_game_switch_write, NULL);

/*proc_aiunit_game_info game hot zone
 * Input:
 * input '0': disable aiunit game
 * input '0.9580078,146.5039,277.66113,299.4336,428.9795,10,1,fangxiang_fz 0.98095703,50.410156,1887.4512,156.46484,1993.7988,10,2,jineng ' enable
 * 0.9580078,146.5039,277.66113,299.4336,428.9795,10      ,1               ,fangxiang_fz
 * score    ,left    ,top      ,right   ,bottom  ,gametype,aiunit_game_type,not used
 */

static ssize_t proc_aiunit_game_info_write(struct file *file,
				const char __user *buffer, size_t count, loff_t *ppos)
{
	int set_num = 0;
	int get_num = 0;
	int one_frame_buf_len = 0;
	int ret = 0;
	int one_float_len = 0;
	int gametype = 0;
	int aiunit_game_type = 0;
	int score = 0;
	int left = 0;
	int top = 0;
	int right = 0;
	int bottom = 0;
	int num = 0;
	int num2 = 0;
	int one_frame_num[7] = {0};
	u8 x_resolution_multiple = 16;
	u8 y_resolution_multiple = 16;
	u8 data_error = 0;
	u8 *get_all_buff = NULL;
	u8 *one_frame_buf_str = NULL;
	u8 *next_frame_buf_str = NULL;
	u8 *one_float_str = NULL;
	u8 *next_float_str = NULL;
	u8 *tmp_str = NULL;
	u8 *one_frame_buf_char_str = NULL;
	struct tp_aiunit_game_info *tp_set_aiunit_game_info = NULL;
	struct tp_aiunit_game_info *tp_get_aiunit_game_info = NULL;
	char one_frame_buf[100] = {0};
	char one_frame_buf_char[100] = {0};
	char *buf = NULL;
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	buf = kzalloc(PAGESIZE * 6, GFP_KERNEL);
	if (buf == NULL) {
		TS_TP_INFO("aiunit game info buf kzalloc error!\n");
		goto write_exit;
	}

	if (!ts) {
		TPD_INFO("%s: ts is NULL\n", __func__);
		goto write_exit;
	}

	ts->aiunit_game_get_num = 0;
	ts->aiunit_game_set_num = 0;
	tp_set_aiunit_game_info = ts->tp_ic_aiunit_game_info;
	tp_get_aiunit_game_info = ts->tp_get_aiunit_game_info;

	if (ts->resolution_info.LCD_WIDTH && ts->resolution_info.LCD_HEIGHT) {
		x_resolution_multiple = ts->resolution_info.max_x / ts->resolution_info.LCD_WIDTH;
		y_resolution_multiple = ts->resolution_info.max_y / ts->resolution_info.LCD_HEIGHT;
	}

	touchpanel_trusted_touch_completion(ts);
	if (!ts->ts_ops->aiunit_game_info || !ts->aiunit_game_info_support) {
		TS_TP_INFO("%s: not support tp aiunit game info callback\n", __func__);
		goto write_exit;
	}

	if (count > PAGESIZE * 6 - 1) {
		TS_TP_INFO("%s: input string is too long. error, error.\n", __func__);
		goto write_exit;
	}
	tp_copy_from_user(buf, PAGESIZE * 6, buffer, count, PAGESIZE * 6 - 1);
	buf[PAGESIZE * 6 - 1] = '\0';
	memset(tp_set_aiunit_game_info, 0, MAX_AIUNIT_SET_NUM * sizeof(struct tp_aiunit_game_info));
	memset(tp_get_aiunit_game_info, 0, MAX_AIUNIT_GET_NUM * sizeof(struct tp_aiunit_game_info));
	get_all_buff = &buf[0];
	TS_TP_INFO("%s: input all buf:%s size:%lu. \n", __func__, get_all_buff, count);
	if (count == 1 && buf[0] == '0') {
		TS_TP_INFO("%s:aiunit game exit.\n", __func__);
		ts->aiunit_game_enable = 0;
		mutex_lock(&ts->mutex);
		ts->ts_ops->aiunit_game_info(ts->chip_data);
		mutex_unlock(&ts->mutex);
		goto write_exit;
	}
	for (get_num = 0; get_num < MAX_AIUNIT_GET_NUM; get_num ++) {
		if (data_error != 0) {
			TS_TP_INFO("%s: getnum end:%d error:%u.\n", __func__, get_num, data_error);
			set_num = 0;
			break;
		}
		next_frame_buf_str = strchr(get_all_buff, ' ');
		if (next_frame_buf_str == NULL) {
			one_frame_buf_len = strlen(get_all_buff);
			TS_TP_INFO("input data end. buffer len:%d.\n", one_frame_buf_len);
		} else {
			one_frame_buf_len = next_frame_buf_str - get_all_buff;
			TS_TP_INFO("input data continue. buffer len:%d.\n", one_frame_buf_len);
		}

		if (one_frame_buf_len < 20 || one_frame_buf_len >= 80) {
			data_error = 1;
			TS_TP_INFO("%s:input error buf_len:%d. \n", __func__, one_frame_buf_len);
			set_num = 0;
			break;
		}

		strncpy(one_frame_buf, get_all_buff, one_frame_buf_len);
		one_frame_buf[one_frame_buf_len] = '\0';

		TS_TP_INFO("%s: one_frame_buf:%s.\n", __func__, one_frame_buf);
		one_frame_buf_str = &one_frame_buf[0];
		one_float_str = &one_frame_buf[0];

		for (num = 0; num < 7; num++) {
			next_float_str = strchr(one_float_str, ',');
			if (next_float_str == NULL) {
				data_error = 2;
				break;
			}
			one_float_len = next_float_str - one_float_str;
			if (one_float_len >= 20) {
				data_error = 3;
				break;
			}
			strncpy(one_frame_buf_char, one_float_str, one_float_len);
			one_frame_buf_char[one_float_len]= '\0';
			TS_TP_INFO("%s: one_frame_buf_char:%s.\n", __func__, one_frame_buf_char);

			one_frame_buf_char_str = &one_frame_buf_char[0];
			tmp_str = strchr(one_frame_buf_char_str, '.');
			if (tmp_str == NULL) {
				ret = sscanf(one_frame_buf_char_str, "%d", &one_frame_num[num]);
				if (ret == 1) {
					TS_TP_INFO("%s: one_frame_buf_char_str:%d.\n", __func__, one_frame_num[num]);
				} else {
					TS_TP_INFO("%s: ret:%d.\n", __func__, ret);
					data_error = 4;
				}
			} else {
				ret = sscanf(one_frame_buf_char_str, "%d.%d", &one_frame_num[num], &num2);
				if (ret == 2) {
					TS_TP_INFO("%s: one_frame_buf_char_str:%d,%d.\n", __func__, one_frame_num[num], num2);
				} else {
					TS_TP_INFO("%s: ret:%d.\n", __func__, ret);
					data_error = 4;
				}
			}

			if (strlen(one_float_str) - one_float_len > 1) {
				one_float_str = &one_float_str[one_float_len + 1];
			}
		}

		score = one_frame_num[0];
		left = one_frame_num[1];
		top = one_frame_num[2];
		right = one_frame_num[3];
		bottom = one_frame_num[4];
		gametype = one_frame_num[5];
		aiunit_game_type = one_frame_num[6];

		if (data_error == 0 && left >= 0 && left < right && right <= ts->resolution_info.LCD_WIDTH \
				&& top >= 0 && top < bottom && bottom <= ts->resolution_info.LCD_HEIGHT \
				&& aiunit_game_type >= 0 && aiunit_game_type < 30 \
				&& gametype >= 0 && gametype < 255) {
			TS_TP_INFO("%s:one frame data:%d,%d,%d,%d,%d,%d,%d.\n", __func__, score, left, top, right, bottom, gametype, aiunit_game_type);

			tp_get_aiunit_game_info[get_num].gametype = (uint8_t)gametype;
			tp_get_aiunit_game_info[get_num].aiunit_game_type = (uint8_t)aiunit_game_type;
			tp_get_aiunit_game_info[get_num].left = (uint16_t)left;
			tp_get_aiunit_game_info[get_num].top = (uint16_t)top;
			tp_get_aiunit_game_info[get_num].right = (uint16_t)right;
			tp_get_aiunit_game_info[get_num].bottom = (uint16_t)bottom;

			if ((1 << aiunit_game_type) & ts->aiunit_game_valid_bits) {
				tp_set_aiunit_game_info[set_num].gametype = (uint8_t)gametype;
				tp_set_aiunit_game_info[set_num].aiunit_game_type = (uint8_t)aiunit_game_type;
				tp_set_aiunit_game_info[set_num].left = (uint16_t)(left * x_resolution_multiple);
				tp_set_aiunit_game_info[set_num].top = (uint16_t)(top * y_resolution_multiple);
				tp_set_aiunit_game_info[set_num].right = (uint16_t)(right * x_resolution_multiple);
				tp_set_aiunit_game_info[set_num].bottom = (uint16_t)(bottom * y_resolution_multiple);

				TS_TP_INFO("%s: num:%d, type:%u,%u, size(%u,%u,%u,%u).\n", __func__, set_num, \
						tp_set_aiunit_game_info[set_num].gametype, tp_set_aiunit_game_info[set_num].aiunit_game_type, \
						tp_set_aiunit_game_info[set_num].left, tp_set_aiunit_game_info[set_num].top, \
						tp_set_aiunit_game_info[set_num].right, tp_set_aiunit_game_info[set_num].bottom);
				set_num++;
			} else {
				TS_TP_INFO("aiunit game type is not used.type:%d, valid_bits:%x.\n", aiunit_game_type, ts->aiunit_game_valid_bits);
			}
			if (set_num >= MAX_AIUNIT_SET_NUM) {
				TS_TP_INFO("%s:already get MAX_AIUNIT_SET_NUM:%d, break.\n", __func__, MAX_AIUNIT_SET_NUM);
				break;
			}
		} else {
			TS_TP_INFO("%s:one frame data error:%d,%d,%d,%d,%d,%d,%d.\n", __func__, score, left, top, right, bottom, gametype, aiunit_game_type);
			data_error = 1;
			set_num = 0;
			get_num = 0;
			break;
		}
		if (next_frame_buf_str == NULL) {
			get_num++;
			break;
		}
		if (strlen(get_all_buff) - one_frame_buf_len < 20) {
			get_num++;
			break;
		} else {
			get_all_buff = &get_all_buff[one_frame_buf_len + 1];
		}
	}

	ts->aiunit_game_get_num = get_num;
	ts->aiunit_game_set_num = set_num;

	if (data_error == 0 && set_num > 0 && ts->aiunit_game_info_support && ts->ts_ops->aiunit_game_info) {
		ts->aiunit_game_enable = 1;
		mutex_lock(&ts->mutex);
		ts->ts_ops->aiunit_game_info(ts->chip_data);
		mutex_unlock(&ts->mutex);
	}

write_exit:
	if (buf != NULL) {
		kfree(buf);
	}

	return count;
}

static ssize_t proc_aiunit_game_info_read(struct file *file, char __user *buffer,
				size_t count, loff_t *ppos)
{
	int ret = 0;
	int get_num = 0;
	int num = 0;
	char *page = NULL;
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	page = kzalloc(MAX_AIINFO_SIZE, GFP_KERNEL);
	if (page == NULL) {
		TS_TP_INFO("aiunit game info page kzalloc error!\n");
		goto read_exit;
	}

	if (!ts) {
		snprintf(page, MAX_AIINFO_SIZE - 1, "%d\n", -1); /*no support*/

	} else {
		get_num = ts->aiunit_game_get_num;
		if (get_num > 0 && ts->aiunit_game_enable) {
			for(num = 0; num < get_num; num++) {
				if (count > strlen(page)) {
					snprintf(&page[0] + strlen(page), MAX_AIINFO_SIZE - strlen(page),
							"GameInfo %d(GameType:%d HotZoneType:%d, Boarder:[%u,%u,%u,%u])\n", num,
							ts->tp_get_aiunit_game_info[num].gametype, ts->tp_get_aiunit_game_info[num].aiunit_game_type,
							ts->tp_get_aiunit_game_info[num].left, ts->tp_get_aiunit_game_info[num].top,
							ts->tp_get_aiunit_game_info[num].right, ts->tp_get_aiunit_game_info[num].bottom);
				}
			}
		} else {
			snprintf(page, MAX_AIINFO_SIZE - 1, "get num:%d game mode:%d, aiunit game is null.\n", get_num, ts->noise_level);
		}

		TS_TP_INFO("%s:AIUNIT_GAME_INFO_NODE get %s\n", __func__, page);
	}
	ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));

read_exit:
	if (page != NULL) {
		kfree(page);
	}

	return ret;
}

DECLARE_PROC_OPS(proc_aiunit_game_info_ops, simple_open, proc_aiunit_game_info_read, proc_aiunit_game_info_write, NULL);

static ssize_t proc_set_idle_freq_mode_write(struct file *file,
				      const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[9] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (count > 8) {
		return count;
	}

	if (!ts) {
		return count;
	}

	if (ts->ts_ops == NULL) {
		TS_TP_INFO("%s: ts->ts_ops is NULL.\n", __func__);
		return count;
	}

	if (copy_from_user(buf, buffer, count)) {
		TS_TP_INFO("%s: read proc input error.\n", __func__);
		return count;
	}
	if (sscanf(buf, "%d", &value) != 1) {
		TS_TP_INFO("%s: sscanf error.\n", __func__);
		return -EINVAL;
	}

	/* Validate value is 0 or 1 */
	if (value != 0 && value != 1) {
		TS_TP_INFO("%s: invalid value %d, should be 0 or 1\n", __func__, value);
		return -EINVAL;
	}
	TS_TP_INFO("%s: value is %x.\n", __func__, value);

	mutex_lock(&ts->mutex);
	ts->idle_freq_enable = value;
	if (ts->game_switch_support) {
		if (ts->ts_ops->set_idle_freq_mode) {
			ts->ts_ops->set_idle_freq_mode(value);
		} else {
			TS_TP_INFO("%s:not support set_idle_freq_mode\n", __func__);
		}
	}
	mutex_unlock(&ts->mutex);

	return count;
}

static ssize_t proc_set_idle_freq_mode_read(struct file *file, char __user *buffer,
				size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		return 0;
	}
	snprintf(page, PAGESIZE - 1, "value:%d\n", ts->idle_freq_enable);

	ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));
	return ret;
}
DECLARE_PROC_OPS(proc_set_idle_freq_mode_ops, simple_open, proc_set_idle_freq_mode_read, proc_set_idle_freq_mode_write, NULL);

/*irq_depth - For enable or disable irq
 * Output:
 * irq depth;
 * irq gpio state;
 */
static ssize_t proc_get_irq_depth_read(struct file *file, char __user *buffer,
				       size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	struct irq_desc *desc = NULL;

	if (!ts) {
		return 0;
	}

	desc = irq_to_desc(ts->irq);

	if (!desc) {
		return 0;
	}

	snprintf(page, PAGESIZE - 1, "depth:%u, state:%d\n", desc->depth,
		 gpio_get_value(ts->hw_res.irq_gpio));
	ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));
	return ret;
}
/*irq_depth - For enable or disable irq
 * Input:
 * value:1, enable_irq;
 * value:other, disable_irq_nosync;
 */
static ssize_t proc_irq_status_write(struct file *file,
				     const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 2);

	if (kstrtoint(buf, 10, &value)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	TP_INFO(ts->tp_index, "%s %d, %s ts->irq=%d\n", __func__, value,
		value ? "enable" : "disable", ts->irq);

	if (value == 1) {
		enable_irq(ts->irq);

	} else {
		disable_irq_nosync(ts->irq);
	}

	return count;
}

DECLARE_PROC_OPS(proc_get_irq_depth_fops, simple_open, proc_get_irq_depth_read, proc_irq_status_write, NULL);

static ssize_t proc_hardware_control_read(struct file *file, char __user *buffer,
				       size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		return 0;
	}

	snprintf(page, PAGESIZE - 1, "avdd_regulator_state:%d, vddi_regulator_state:%d, reset_state:%d\n",
		regulator_get_voltage(ts->hw_res.avdd), regulator_get_voltage(ts->hw_res.vddi), gpio_get_value(ts->hw_res.reset_gpio));

	ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_hardware_control_write(struct file *file,
				     const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	int ret = 0;

	if (!ts) {
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 2);

	if (kstrtoint(buf, 10, &value)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	if (value == ENABLE_HW_RES_AVDD) {
		ret = tp_powercontrol_avdd(&ts->hw_res, true);
		if (ret) {
			TP_INFO(ts->tp_index, "%s: tp_powercontrol_avdd error\n", __func__);
		}
		return count;
	}

	if (value == DISABLE_HW_RES_AVDD) {
		ret = tp_powercontrol_avdd(&ts->hw_res, false);
		if (ret) {
			TP_INFO(ts->tp_index, "%s: tp_powercontrol_avdd error\n", __func__);
		}
		return count;
	}

	if (value == ENABLE_HW_RES_VDDI) {
		ret = tp_powercontrol_vddi(&ts->hw_res, true);
		if (ret) {
			TP_INFO(ts->tp_index, "%s: tp_powercontrol_vddi error\n", __func__);
		}
		return count;
	}

	if (value == DISABLE_HW_RES_VDDI) {
		ret = tp_powercontrol_vddi(&ts->hw_res, false);
		if (ret) {
			TP_INFO(ts->tp_index, "%s: tp_powercontrol_vddi error\n", __func__);
		}
		return count;
	}

	if (value == ENABLE_HW_RESET) {
		if (gpio_is_valid(ts->hw_res.reset_gpio)) {
        		gpio_set_value(ts->hw_res.reset_gpio, true);
		}

		TP_INFO(ts->tp_index, "%s: reset_gpio enable\n", __func__);

		return count;
	}

	if (value == DISABLE_HW_RESET) {
		if (gpio_is_valid(ts->hw_res.reset_gpio)) {
        		gpio_set_value(ts->hw_res.reset_gpio, false);
		}

		TP_INFO(ts->tp_index, "%s: reset_gpio disable\n", __func__);

		return count;
	}
	return count;
}

DECLARE_PROC_OPS(proc_hardware_control_fops, simple_open, proc_hardware_control_read, proc_hardware_control_write, NULL);

static ssize_t proc_report_rate_test_read(struct file *file, char __user *buffer,
				       size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		return count;
	}

	if (ts->is_suspended) {
		TP_INFO(ts->tp_index, "%s: tp is suspended.\n", __func__);
		return count;
	}
	ts->get_frame_num = 0;
	ts->touch_major_sum = 0;
	ts->report_rate_testing = true;
	usleep_range(ts->report_rate_test_time * 1000 * 1000, ts->report_rate_test_time * 1000 * 1000 + 10);
	ts->report_rate_testing = false;

	if (ts->report_rate_test_time > 0 && ts->get_frame_num > 0) {
		snprintf(page, PAGESIZE - 1, "report frames:%u, rate:%uHZ. touch major avg:%u.\n",
			ts->get_frame_num, ts->get_frame_num / ts->report_rate_test_time, (ts->touch_major_sum + ts->get_frame_num / 2) / ts->get_frame_num);
		ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));
	} else {
		TP_INFO(ts->tp_index, "%s: get_frame_num or report_rate_test_time is 0.\n", __func__);
	}

	return ret;
}

static ssize_t proc_report_rate_test_write(struct file *file,
				     const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[5] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 4);

	if (kstrtoint(buf, 10, &value)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}
	if (value > REPORT_RATE_MAX_TEST_TIME_S) {
		TP_INFO(ts->tp_index, "%s: input test time too long:%uS, max time is:%uS. \n", __func__, value, REPORT_RATE_MAX_TEST_TIME_S);
		value = REPORT_RATE_MAX_TEST_TIME_S;
	}
	TP_INFO(ts->tp_index, "%s: report rate test time set:%u. \n", __func__, value);
	ts->report_rate_test_time = value;
	return count;
}

DECLARE_PROC_OPS(proc_report_rate_test_fops, simple_open, proc_report_rate_test_read, proc_report_rate_test_write, NULL);

static ssize_t proc_noise_modetest_read(struct file *file, char __user *buffer,
					size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts || !ts->ts_ops->get_noise_modetest) {
		return 0;
	}

	snprintf(page, PAGESIZE - 1, "%hhu\n",
		 ts->ts_ops->get_noise_modetest(ts->chip_data));
	ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_noise_modetest_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[8] = {0};
	int temp = 0;
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 2);

	if (kstrtoint(buf, 10, &temp)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	TPD_DETAIL("%s: value = %d\n", __func__, temp);

	mutex_lock(&ts->mutex);

	if (ts->ts_ops->set_noise_modetest) {
		ts->ts_ops->set_noise_modetest(ts->chip_data, temp);
	}

	mutex_unlock(&ts->mutex);

	return count;
}

DECLARE_PROC_OPS(proc_noise_modetest_fops, simple_open, proc_noise_modetest_read, proc_noise_modetest_write, NULL);

/*tp_fw_update - For touch panel fw update
 * Input:
 * firmware_update_type:0, fw update;
 * firmware_update_type:1, fore fw update;
 * firmware_update_type:2, app fw update;
 */
static ssize_t proc_fw_update_write(struct file *file,
				    const char __user *buffer, size_t count, loff_t *ppos)
{
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	int val = 0;
	int ret = 0;
	char buf[4] = {0};

	if (!ts) {
		return count;
	}
	touchpanel_trusted_touch_completion(ts);

#ifndef CONFIG_REMOVE_OPLUS_FUNCTION
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM

	if (ts->boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT)
#else
	if (ts->boot_mode == MSM_BOOT_MODE__CHARGE)
#endif
#endif
	{
		TP_INFO(ts->tp_index,
			"boot mode is MSM_BOOT_MODE__CHARGE,not need update tp firmware\n");
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 2);

	if (kstrtoint(buf, 10, &val)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	ts->firmware_update_type = val;

	if (!ts->force_update && ts->firmware_update_type != 2) {
		ts->force_update = !!val;
	}

	schedule_work(&ts->fw_update_work);

	ret = wait_for_completion_killable_timeout(&ts->fw_complete,
			FW_UPDATE_COMPLETE_TIMEOUT);

	if (ret < 0) {
		TP_INFO(ts->tp_index, "kill signal interrupt\n");
	}

#ifdef CONFIG_TOUCHIRQ_UPDATE_QOS

	if (!ts->pm_qos_state) {
		ts->pm_qos_value = PM_QOS_DEFAULT_VALUE;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,10,0)
		if (!cpu_latency_qos_request_active(&ts->pm_qos_req)) {
			cpu_latency_qos_add_request(&ts->pm_qos_req, ts->pm_qos_value);
		} else {
			cpu_latency_qos_update_request(&ts->pm_qos_req, ts->pm_qos_value);
		}
#else
		pm_qos_add_request(&ts->pm_qos_req, PM_QOS_CPU_DMA_LATENCY, ts->pm_qos_value);
#endif
		TP_INFO(ts->tp_index, "add qos request in touch driver.\n");
		ts->pm_qos_state = 1;
	}

#endif

	TP_INFO(ts->tp_index, "fw update finished\n");
	return count;
}

DECLARE_PROC_OPS(proc_fw_update_ops, simple_open, NULL, proc_fw_update_write, NULL);

/*oplus_register_info - For i2c debug way
 * Output:
 *first choose register_add and lenght, example: echo 000e,1 > oplus_register_info
 *second read: cat oplus_register_info
 */
static ssize_t proc_register_info_read(struct file *file, char __user *buffer,
				       size_t count, loff_t *ppos)
{
	int ret = 0;
	int i = 0;
	ssize_t num_read_chars = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		return 0;
	}

	if (ts->reg_info.reg_length < 1 || ts->reg_info.reg_length > 9) {
		TP_INFO(ts->tp_index, "ts->reg_info.reg_length error!\n");
		return 0;
	}

	ts->reg_info.reg_result = kzalloc(ts->reg_info.reg_length * (sizeof(uint16_t)),
					  GFP_KERNEL);

	if (!ts->reg_info.reg_result) {
		TP_INFO(ts->tp_index, "ts->reg_info.reg_result kzalloc error\n");
		return 0;
	}

	if (ts->ts_ops->register_info_read) {
		mutex_lock(&ts->mutex);
		ts->ts_ops->register_info_read(ts->chip_data, ts->reg_info.reg_addr,
					       ts->reg_info.reg_result, ts->reg_info.reg_length);
		mutex_unlock(&ts->mutex);

		for (i = 0; i < ts->reg_info.reg_length; i++) {
			num_read_chars += snprintf(&(page[num_read_chars]), PAGESIZE - 1,
						   "reg_addr(0x%x) = 0x%x\n", ts->reg_info.reg_addr, ts->reg_info.reg_result[i]);
		}

		ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));
	}

	tp_kfree((void **)&ts->reg_info.reg_result);
	return ret;
}

/*oplus_register_info - For i2c debug way
 * Input:
 *first choose register_add and lenght, example: echo 000e,1 > oplus_register_info
 *second read: cat oplus_register_info
 */
static ssize_t proc_register_info_write(struct file *file,
					const char __user *buffer, size_t count, loff_t *ppos)
{
	int addr = 0, length = 0;
	char buf[16] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		TPD_INFO("ts not exist!\n");
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 7);

	sscanf(buf, "%x,%d", (uint32_t *)&addr, &length);
	ts->reg_info.reg_addr = (uint16_t)addr;
	ts->reg_info.reg_length = (uint16_t)length;
	TP_INFO(ts->tp_index,
		"ts->reg_info.reg_addr = 0x%x, ts->reg_info.reg_lenght = %d\n",
		ts->reg_info.reg_addr, ts->reg_info.reg_length);

	return count;
}

DECLARE_PROC_OPS(proc_register_info_fops, simple_open, proc_register_info_read, proc_register_info_write, NULL);

static ssize_t proc_incell_panel_info_read(struct file *file,
		char __user *buffer, size_t count, loff_t *ppos)
{
	uint8_t ret = 0;
	int value = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	value = ts->is_incell_panel;
	if (ts->game_enable_in_tddi_support) {
		value |= ts->game_enable_in_tddi_support << 1;
	}

	snprintf(page, PAGESIZE - 1, "%d", value);
	ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));

	return ret;
}

DECLARE_PROC_OPS(proc_incell_panel_fops, simple_open, proc_incell_panel_info_read, NULL, NULL);

/*fd_enable - For touch panel face detect
 * Input:
 * fd_enable:0, enable face detect;
 * fd_enable:1, disable face detect;
 */
static ssize_t proc_fd_enable_write(struct file *file,
				    const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		return count;
	}
	touchpanel_trusted_touch_completion(ts);

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 2);

	if (kstrtoint(buf, 10, &value)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	if (value > 2) {
		return count;
	}

	TP_DEBUG(ts->tp_index, "%s value: %d, es_enable :%d\n", __func__, value,
		 ts->fd_enable);

	if (value == ts->fd_enable) {
		return count;
	}

	mutex_lock(&ts->mutex);
	ts->fd_enable = value;

	if (!ts->is_suspended && (ts->suspend_state == TP_SPEEDUP_RESUME_COMPLETE)) {
		if (ts->fd_enable) {
			input_event(ts->ps_input_dev, EV_MSC, MSC_RAW, 0);
			input_sync(ts->ps_input_dev);
		}

		ts->ts_ops->mode_switch(ts->chip_data, MODE_FACE_DETECT, ts->fd_enable == 1);
	}

	mutex_unlock(&ts->mutex);

	return count;
}

/*fd_enable - For touch panel face detect
 * Output:
 * fd_enable:0, enable face detect;
 * fd_enable:1, disable face detect;
 */
static ssize_t proc_fd_enable_read(struct file *file, char __user *buffer,
				   size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		return 0;
	}

	TP_DEBUG(ts->tp_index, "%s value: %d\n", __func__, ts->fd_enable);
	ret = snprintf(page, PAGESIZE - 1, "%d\n", ts->fd_enable);
	ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));

	return ret;
}

DECLARE_PROC_OPS(tp_fd_enable_fops, simple_open, proc_fd_enable_read, proc_fd_enable_write, NULL);

/*event_num - For touch panel face detect input num
 * Output:
 * face detect input num;
 */
static ssize_t proc_event_num_read(struct file *file, char __user *buffer,
				   size_t count, loff_t *ppos)
{
	int ret = 0;
	const char *devname = NULL;
	struct input_handle *handle;
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		return 0;
	}

	list_for_each_entry(handle, &ts->ps_input_dev->h_list, d_node) {
		if (strncmp(handle->name, "event", 5) == 0) {
			devname = handle->name;
			break;
		}
	}

	if (devname) {
		ret = simple_read_from_buffer(buffer, count, ppos, devname, strlen(devname));
	}

	return ret;
}

DECLARE_PROC_OPS(tp_event_num_fops, simple_open, proc_event_num_read, NULL, NULL);

/*touch_count - For face detect touch count
 * Output:
 * touch_count:0;
 * touch_count:1-F;
 */
static ssize_t proc_fd_touch_read(struct file *file, char __user *buffer,
				  size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		return 0;
	}

	TP_DEBUG(ts->tp_index, "%s value: %d\n", __func__, ts->touch_count);
	/*mutex_lock(&ts->mutex);*/
	ret = snprintf(page, PAGESIZE - 1, "%d\n", ts->touch_count & 0x0F);
	/*mutex_unlock(&ts->mutex);*/
	ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));

	return ret;
}

DECLARE_PROC_OPS(fd_touch_num_fops, simple_open, proc_fd_touch_read, NULL, NULL);

/*fp_enable - For touch panel triger fingerprint
 * Input:
 * fp_enable:0, enable;
 * fp_enable:1, disable;
 */
static ssize_t proc_fp_enable_write(struct file *file,
				    const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		return count;
	}

	touchpanel_trusted_touch_completion(ts);
	tp_copy_from_user(buf, sizeof(buf), buffer, count, 2);

	if (kstrtoint(buf, 10, &value)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	/*Fingerprint authentication completed and want to turn off the fingerprint function.*/
	if (value == FP_QUICK_START_ON && ts->fp_quick_start_data == 0 && ts->is_suspended && ts->fp_enable == 1) {
		if (ts->fp_disable_after_resume) {
			ts->fp_quick_start_data = 1;/*when fingerprint quick start featrue on ,need keep fp_enable = 1 befor tp resume*/
			TPD_DETAIL("%s value: %d, fp_quick_start_data :%d, fp_enable :%d\n", __func__, value, ts->fp_quick_start_data, ts->fp_enable);
			return count;
		} else {
			value = 0; /*set fp_enable = 0 as fingerprint want*/
		}
	}

	if (value > 2) {
		return count;
	}

	TPD_DETAIL("%s value: %d, fp_enable :%d\n", __func__, value, ts->fp_enable);

	if (value == ts->fp_enable) {
		return count;
	}

	mutex_lock(&ts->mutex);
	ts->fp_enable = value;

	if (!ts->fp_enable) {
		ts->fp_info.touch_state = 0;        /*reset touch state*/
	}

	if (!ts->is_suspended && (ts->suspend_state == TP_SPEEDUP_RESUME_COMPLETE)) {
		ts->ts_ops->enable_fingerprint(ts->chip_data, !!ts->fp_enable);

	} else if (ts->is_suspended) {
		if (ts->black_gesture_support && (1 == (ts->gesture_enable & 0x01))) {
			ts->ts_ops->enable_fingerprint(ts->chip_data, !!ts->fp_enable);

		} else {
			operate_mode_switch(ts);
		}
	}

	mutex_unlock(&ts->mutex);

	return count;
}

/*fp_enable - For touch panel triger fingerprint
 * Output:
 * fp_enable:0, enable;
 * fp_enable:1, disable;
 */
static ssize_t proc_fp_enable_read(struct file *file, char __user *buffer,
				   size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		return 0;
	}

	TP_DEBUG(ts->tp_index, "%s value: %d\n", __func__, ts->fp_enable);
	ret = snprintf(page, PAGESIZE - 1, "%d\n", ts->fp_enable);
	ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));

	return ret;
}

DECLARE_PROC_OPS(tp_fp_enable_fops, simple_open, proc_fp_enable_read, proc_fp_enable_write, NULL);

/*proc/touchpanel/baseline_test*/
static int tp_auto_test_read_func(struct seq_file *s, void *v)
{
	int ret = 0;

	ret = tp_auto_test(s, v);
	return ret;
}

static int baseline_autotest_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_auto_test_read_func, PDE_DATA(inode));
}

static ssize_t baseline_autotest_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 4);

	if (kstrtoint(buf, 10, &value)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	TP_INFO(ts->tp_index, "%s %d\n", __func__, value);
	ts->com_test_data.raw_cap_restriction = value;
	return count;
}

DECLARE_PROC_OPS(tp_auto_test_proc_fops, baseline_autotest_open, seq_read, baseline_autotest_write, single_release);


/*black_screen_test - For incell ic black screen test*/
static ssize_t proc_black_screen_test_read(struct file *file,
		char __user *buffer, size_t count, loff_t *ppos)
{
	int ret = 0;

	ret = tp_black_screen_test(file, buffer, count, ppos);
	return ret;
}

static ssize_t proc_black_screen_test_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 2);

	if (kstrtoint(buf, 10, &value)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	TP_INFO(ts->tp_index, "%s %d\n", __func__, value);

	ts->gesture_test.gesture_backup = ts->gesture_enable;
	ts->gesture_enable = true;
	ts->gesture_test.flag = !!value;

	return count;
}

DECLARE_PROC_OPS(proc_black_screen_test_fops, simple_open, proc_black_screen_test_read, proc_black_screen_test_write, NULL);

/*baseline_result - For GKI auto test result*/
static int tp_auto_test_result_read(struct seq_file *s, void *v)
{
	int ret = 0;

	ret = tp_auto_test_result(s, v);
	return ret;
}

static int tp_auto_test_result_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_auto_test_result_read, PDE_DATA(inode));
}

DECLARE_PROC_OPS(tp_auto_test_result_fops, tp_auto_test_result_open, seq_read, NULL, single_release);


/*baseline_result - For GKI auto test result*/
static int tp_black_screen_result_read(struct seq_file *s, void *v)
{
	int ret = 0;

	ret = tp_black_screen_result(s, v);
	return ret;
}

static int tp_black_screen_result_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_black_screen_result_read, PDE_DATA(inode));
}

DECLARE_PROC_OPS(proc_black_screen_result_fops, tp_black_screen_result_open, seq_read, NULL, single_release);

/*limit_enable - For touch panel direction
 * Output:
 * limit_enable:0, VERTICAL_SCREEN;
 * limit_enable:1, LANDSCAPE_SCREEN_90;
 * limit_enable:2, LANDSCAPE_SCREEN_270;
 */
static ssize_t proc_dir_control_read(struct file *file, char __user *user_buf,
				     size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		return 0;
	}

	TS_TP_INFO("limit_enable is: 0x%x\n", ts->limit_enable);
	ret = snprintf(page, PAGESIZE - 1, "%d\n", ts->limit_enable);

	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_dir_control_write(struct file *file,
				      const char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[8] = {0};
	int temp = 0;
	int ret = 0;
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		return count;
	}

	if (count > 2) {
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 2);

	if (kstrtoint(buf, 10, &temp)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}
	touch_misc_state_change(ts, IOC_STATE_DIR, temp);
	mutex_lock(&ts->mutex);

	ts->limit_enable = temp;
	TS_TP_INFO("%s: limit_enable = 0x%x\n", __func__, ts->limit_enable);

	if (ts->is_suspended == 0) {
		ret = ts->ts_ops->mode_switch(ts->chip_data, MODE_EDGE, ts->limit_enable);

		if (ret < 0) {
			TS_TP_INFO("%s, Touchpanel operate mode switch failed\n", __func__);
		}
	}


	mutex_unlock(&ts->mutex);

	return count;
}

DECLARE_PROC_OPS(touch_dir_proc_fops, simple_open, proc_dir_control_read, proc_dir_control_write, NULL);

static ssize_t proc_rate_white_list_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (count > 4) {
		TPD_INFO("%s:count > 4\n", __func__);
		return count;
	}

	if (!ts) {
		TPD_INFO("%s: ts is NULL\n", __func__);
		return count;
	}

	touchpanel_trusted_touch_completion(ts);
	if (!ts->ts_ops->rate_white_list_ctrl) {
		TS_TP_INFO("%s:not support ts_ops->rate_white_list_ctrl callback\n", __func__);
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 4);

	if (kstrtoint(buf, 10, &value)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	mutex_lock(&ts->mutex);

	ts->rate_ctrl_level = value;

	TS_TP_INFO("%s: write value=%d\n", __func__, value);

	if (!ts->is_suspended) {
		ts->ts_ops->rate_white_list_ctrl(ts->chip_data, value);

	} else {
		TS_TP_INFO("%s: TP is_suspended.\n", __func__);
	}

	mutex_unlock(&ts->mutex);

	return count;
}

static ssize_t proc_edge_limit_switch_read(struct file *file, char __user *user_buf,
				     size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		snprintf(page, PAGESIZE - 1, "%d\n", -1); /* no support */

	} else {
		snprintf(page, PAGESIZE - 1, "%d\n", ts->edge_limit_switch_write_value); /* support */
	}

	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t proc_edge_limit_switch_write(struct file *file,
				      const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (count > 4) {
		TPD_INFO("%s:count > 4\n", __func__);
		return -EINVAL;
	}

	if (!ts || !(ts->ts_ops)) {
		TPD_INFO("%s: ts is NULL\n", __func__);
		return -EINVAL;
	}

	touchpanel_trusted_touch_completion(ts);
	if (!ts->ts_ops->edge_limit_switch_write) {
		TS_TP_INFO("%s:not support ts_ops->edge_limit_switch_write callback\n", __func__);
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 4);

	if (kstrtoint(buf, 4, &value)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&ts->mutex);

	ts->edge_limit_switch_write_value = value;

	TS_TP_INFO("%s: write value=%d\n", __func__, value);

	if (!ts->is_suspended) {
		ts->ts_ops->edge_limit_switch_write(ts->chip_data, value);

	} else {
		TS_TP_INFO("%s: TP is_suspended.\n", __func__);
	}

	mutex_unlock(&ts->mutex);

	return count;
}

DECLARE_PROC_OPS(touch_edge_limit_switch_fops, simple_open, proc_edge_limit_switch_read, proc_edge_limit_switch_write, NULL);

static ssize_t proc_report_rate_read(struct file *file, char __user *user_buf,
				     size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		snprintf(page, PAGESIZE - 1, "%d\n", -1); /* no support */

	} else {
		snprintf(page, PAGESIZE - 1, "%d\n", ts->report_rate_write_value); /* support */
	}

	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t proc_report_rate_write(struct file *file,
				      const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (count > 4) {
		TPD_INFO("%s:count > 4\n", __func__);
		return -EINVAL;
	}

	if (!ts || !(ts->ts_ops)) {
		TPD_INFO("%s: ts is NULL\n", __func__);
		return -EINVAL;
	}

	touchpanel_trusted_touch_completion(ts);
	if (!ts->ts_ops->report_rate) {
		TS_TP_INFO("%s:not support ts_ops->report_rate callback\n", __func__);
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 4);

	if (kstrtoint(buf, 10, &value)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&ts->mutex);

	ts->report_rate_write_value = value;
	TS_TP_INFO("%s: write value=%d\n", __func__, value);

	if (!ts->is_suspended) {
		ts->ts_ops->report_rate(ts->chip_data, value);

	} else {
		TS_TP_INFO("%s: TP is_suspended.\n", __func__);
	}

	mutex_unlock(&ts->mutex);

	return count;
}
DECLARE_PROC_OPS(touch_report_rate_fops, simple_open, proc_report_rate_read, proc_report_rate_write, NULL);

static ssize_t proc_rate_white_list_read(struct file *file,
		char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		snprintf(page, PAGESIZE - 1, "%d\n", -1); /*no support*/

	} else {
		snprintf(page, PAGESIZE - 1, "%d\n", ts->rate_ctrl_level); /*support*/
	}

	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

DECLARE_PROC_OPS(proc_rate_white_list_fops, simple_open, proc_rate_white_list_read, proc_rate_white_list_write, NULL);

static ssize_t proc_switch_usb_state_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	int usb_state = 0;
	char buf[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (count > 4) {
		TPD_INFO("%s:count > 4\n", __func__);
		return count;
	}

	if (!ts) {
		TPD_INFO("%s: ts is NULL\n", __func__);
		return count;
	}
	touchpanel_trusted_touch_completion(ts);

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 2);

	if (kstrtoint(buf, 10, &usb_state)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	if (is_ftm_boot_mode(ts)) {
		TP_INFO(ts->tp_index, "Ftm mode, do not switch usb state\n");
		return count;
	}

	ts->cur_usb_state = usb_state;

	touch_misc_state_change(ts, IOC_STATE_CHARGER, usb_state);

	queue_work(ts->charger_pump_wq, &ts->charger_pump_work);

	return count;
}

static ssize_t proc_switch_usb_state_read(struct file *file,
		char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		snprintf(page, PAGESIZE - 1, "%d\n", -1); /*no support*/

	} else {
		snprintf(page, PAGESIZE - 1, "%d\n", ts->is_usb_checked); /*support*/
	}

	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

DECLARE_PROC_OPS(proc_switch_usb_state_fops, simple_open, proc_switch_usb_state_read, proc_switch_usb_state_write, NULL);


static ssize_t proc_disable_touch_event_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	int disable_touch_event = 0;
	char buf[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (count > 4) {
		TPD_INFO("%s:count > 4\n", __func__);
		return count;
	}

	if (!ts) {
		TPD_INFO("%s: ts is NULL\n", __func__);
		return count;
	}
	touchpanel_trusted_touch_completion(ts);

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 2);

	if (kstrtoint(buf, 10, &disable_touch_event)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	TP_INFO(ts->tp_index, "%s: write value=%d\n", __func__,
		disable_touch_event);
	ts->disable_touch_event = disable_touch_event;

	return count;
}

static ssize_t proc_disable_touch_event_read(struct file *file,
		char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		snprintf(page, PAGESIZE - 1, "%d\n", -1); /*no support*/

	} else {
		snprintf(page, PAGESIZE - 1, "%d\n", ts->disable_touch_event); /*support*/
	}

	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

DECLARE_PROC_OPS(proc_disable_touch_event_fops, simple_open, proc_disable_touch_event_read, proc_disable_touch_event_write, NULL);

static ssize_t proc_wireless_charge_detect_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *ppos)
{
	int wireless_state = 0;
	char buf[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (count > 4) {
		TPD_INFO("%s:count > 4\n", __func__);
		return count;
	}

	if (!ts) {
		TPD_INFO("%s: ts is NULL\n", __func__);
		return count;
	}
	touchpanel_trusted_touch_completion(ts);

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 2);

	if (kstrtoint(buf, 10, &wireless_state)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	if (is_ftm_boot_mode(ts)) {
		TS_TP_INFO("Ftm mode, do not switch wireless state\n");
		return count;
	}

	touch_misc_state_change(ts, IOC_STATE_WIRELESS_CHARGER, wireless_state);

	if (ts->wireless_charger_support
			&& (ts->is_wireless_checked != wireless_state)) {
		ts->is_wireless_checked = !!wireless_state;
		TS_TP_INFO("%s: check wireless state : %d, is_suspended: %d\n", __func__,
			 wireless_state, ts->is_suspended);

		if (!ts->is_suspended && (ts->suspend_state == TP_SPEEDUP_RESUME_COMPLETE)
				&& !ts->loading_fw) {
			mutex_lock(&ts->mutex);
			ts->ts_ops->mode_switch(ts->chip_data, MODE_WIRELESS_CHARGE,
						ts->is_wireless_checked);
			mutex_unlock(&ts->mutex);
		}
	}

	return count;
}

static ssize_t proc_wireless_charge_detect_read(struct file *file,
		char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		snprintf(page, PAGESIZE - 1, "%d\n", -1); /*no support*/

	} else {
		snprintf(page, PAGESIZE - 1, "%d\n", ts->is_wireless_checked); /*support*/
	}

	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

DECLARE_PROC_OPS(proc_wireless_charge_detect_fops, simple_open, proc_wireless_charge_detect_read, proc_wireless_charge_detect_write, NULL);

static ssize_t proc_headset_detect_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *ppos)
{
	int headset_state = 0;
	char buf[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (count > 4) {
		TPD_INFO("%s:count > 4\n", __func__);
		return count;
	}

	if (!ts) {
		TPD_INFO("%s: ts is NULL\n", __func__);
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 2);

	if (kstrtoint(buf, 10, &headset_state)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	if (is_ftm_boot_mode(ts)) {
		TP_INFO(ts->tp_index, "Ftm mode, do not switch usb state\n");
		return count;
	}

	ts->is_headset_checked = headset_state;

	queue_work(ts->headset_pump_wq, &ts->headset_pump_work);

	return count;
}

static ssize_t proc_headset_detect_read(struct file *file,
					char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		snprintf(page, PAGESIZE - 1, "%d\n", -1); /*no support*/

	} else {
		snprintf(page, PAGESIZE - 1, "%d\n", ts->is_headset_checked); /*support*/
	}

	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

DECLARE_PROC_OPS(proc_headset_detect_fops, simple_open, proc_headset_detect_read, proc_headset_detect_write, NULL);


static ssize_t proc_aging_test_read(struct file *file, char __user *user_buf,
				    size_t count, loff_t *ppos)
{
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	uint8_t ret = 0;
	char page[PAGESIZE] = {0};

	if (!ts) {
		return 0;
	}

	if (ts->aging_test_ops) {
		if ((ts->aging_test_ops->start_aging_test)
				&& (ts->aging_test_ops->finish_aging_test)) {
			ret = 1;
		}
	}

	if (!ret) {
		TS_TP_INFO("%s:no support aging_test2\n", __func__);
	}

	ret = snprintf(page, PAGESIZE - 1, "%hhu, %d\n", ret, ts->aging_test);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

/*tp_aging_test - For aging test mode
 * Output:
 * tp_aging_test = 0 : disable active
 * tp_aging_test = 1 : enable active
 * tp_aging_test = 2 : enable normal test
 * tp_aging_test = 3 : enable aging test
 */
static ssize_t proc_aging_test_write(struct file *file,
				     const char __user *buffer, size_t count, loff_t *ppos)
{
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	int val = 0;
	char buf[4] = {0};

	if (!ts) {
		return count;
	}
	touchpanel_trusted_touch_completion(ts);

	if (count > 2) {
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 2);

	if (kstrtoint(buf, 10, &val)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	TP_INFO(ts->tp_index, "%s:val:%d\n", __func__, val);

	if (AGING_TEST_MODE == val || NORMAL_TEST_MODE == val) {
		ts->aging_mode = val;
		return count;
	}

	if (!ts->aging_test_ops) {
		return count;
	}

	if ((!ts->aging_test_ops->start_aging_test)
			|| (!ts->aging_test_ops->finish_aging_test)) {
		return count;
	}

	mutex_lock(&ts->mutex);
	ts->aging_test = !!val;

	if (ts->aging_test) {
		ts->aging_test_ops->start_aging_test(ts->chip_data);

	} else {
		ts->aging_test_ops->finish_aging_test(ts->chip_data);
	}

	mutex_unlock(&ts->mutex);

	return count;
}

DECLARE_PROC_OPS(proc_aging_test_ops, simple_open, proc_aging_test_read, proc_aging_test_write, NULL);


static ssize_t proc_smooth_level_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0, raw_level = 0;
	char buf[6] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (count > 5) {
		TPD_INFO("%s:count > 5\n", __func__);
		return count;
	}

	if (!ts) {
	TPD_INFO("%s: ts is NULL\n", __func__);
		return count;
	}
	touchpanel_trusted_touch_completion(ts);

	if (!ts->ts_ops->smooth_lv_set) {
		TS_TP_INFO("%s:not support ts_ops->smooth_lv_set callback\n", __func__);
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 5);
	if (kstrtoint(buf, 10, &value)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	mutex_lock(&ts->mutex);
	if (ts->smooth_level_array_support) {
		if (value < 0) {
			raw_level = -value;
		} else {
			if (value < SMOOTH_LEVEL_NUM) {
				ts->smooth_level_chosen = value;
			} else {
				ts->smooth_level_chosen = SMOOTH_LEVEL_NUM - 1;
			}
			if (ts->health_monitor_support && ts->smooth_level_chosen) {
				ts->monitor_data.smooth_level_chosen = ts->smooth_level_chosen;
			}
			raw_level = ts->smooth_level_used_array[ts->smooth_level_chosen];
		}

		TS_TP_INFO("%s: level=%d value=%d\n", __func__, ts->smooth_level_chosen, raw_level);
	}
	if (!ts->is_suspended) {

		ts->ts_ops->smooth_lv_set(ts->chip_data, raw_level);

	} else {
		TS_TP_INFO("%s: TP is_suspended.\n", __func__);
    }

	mutex_unlock(&ts->mutex);

	return count;
}

static ssize_t proc_smooth_level_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    char page[PAGESIZE] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (ts && ts->smooth_level_array_support) {
        snprintf(page, PAGESIZE - 1, "%u\n", ts->smooth_level_chosen); //support
    } else {
        snprintf(page, PAGESIZE - 1, "%d\n", -1); //no support
    }
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    return ret;
}

DECLARE_PROC_OPS(proc_smooth_level_fops, simple_open, proc_smooth_level_read, proc_smooth_level_write, NULL);

static ssize_t proc_sensitive_level_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0, raw_level = 0;
	char buf[6] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (count > 5) {
		TPD_INFO("%s:count > 5\n", __func__);
		return count;
	}

	if (!ts) {
		TPD_INFO("%s: ts is NULL\n", __func__);
		return count;
	}
	touchpanel_trusted_touch_completion(ts);

	if (!ts->ts_ops->sensitive_lv_set) {
		TS_TP_INFO("%s:not support ts_ops->sensitive_lv_set callback\n", __func__);
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 5);

	if (kstrtoint(buf, 10, &value)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	mutex_lock(&ts->mutex);
	if (value < 0) {
		raw_level = -value;
	} else {
		if (value < SENSITIVE_LEVEL_NUM) {
			ts->sensitive_level_chosen = value;
		} else {
			ts->sensitive_level_chosen = SENSITIVE_LEVEL_NUM - 1;
		}
		if (ts->health_monitor_support && ts->sensitive_level_chosen) {
			ts->monitor_data.sensitive_level_chosen = ts->sensitive_level_chosen;
		}
		raw_level = ts->sensitive_level_used_array[ts->sensitive_level_chosen];
	}

	TS_TP_INFO("%s: level=%d value=%d\n", __func__, ts->sensitive_level_chosen, raw_level);
	if (!ts->is_suspended) {
		ts->ts_ops->sensitive_lv_set(ts->chip_data, raw_level);
	} else {
		TS_TP_INFO("%s: TP is_suspended.\n", __func__);
	}
	mutex_unlock(&ts->mutex);

	return count;
}

static ssize_t proc_sensitive_level_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    char page[PAGESIZE] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts || !ts->sensitive_level_array_support) {
        snprintf(page, PAGESIZE - 1, "%d\n", -1); //no support
    } else {
        snprintf(page, PAGESIZE - 1, "%u\n", ts->sensitive_level_chosen); //support
    }
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    return ret;
}

DECLARE_PROC_OPS(proc_sensitive_level_fops, simple_open, proc_sensitive_level_read, proc_sensitive_level_write, NULL);

static ssize_t proc_click_sensitive_level_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0, raw_level = 0;
	char buf[6] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (count > 5) {
		TPD_INFO("%s:count > 5\n", __func__);
		return count;
	}

	if (!ts) {
		TPD_INFO("%s: ts is NULL\n", __func__);
		return count;
	}

	if (!ts->ts_ops->click_sensitive_lv_set) {
		TS_TP_INFO("%s:not support ts_ops->click_sensitive_lv_set callback\n", __func__);
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 5);

	if (kstrtoint(buf, 10, &value)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	mutex_lock(&ts->mutex);
	if (value < 0) {
		raw_level = -value;
	} else {
		if (value < CLICK_SENSITIVE_LEVEL_NUM) {
			ts->click_sensitive_level_chosen = value;
		} else {
			ts->click_sensitive_level_chosen = CLICK_SENSITIVE_LEVEL_NUM - 1;
		}

		if (ts->health_monitor_support && ts->click_sensitive_level_chosen) {
			ts->monitor_data.click_sensitive_level_chosen = ts->click_sensitive_level_chosen;
		}

		raw_level = ts->click_sensitive_level_array[ts->click_sensitive_level_chosen];
	}

	TS_TP_INFO("%s: level = %d, value = %d.\n", __func__, ts->click_sensitive_level_chosen, raw_level);

	if (!ts->is_suspended) {
		ts->ts_ops->click_sensitive_lv_set(ts->chip_data, raw_level);
	} else {
		TS_TP_INFO("%s: TP is_suspended.\n", __func__);
		TS_TP_INFO("ClickSensitiveLevel:%u, value:%u\n",
			ts->click_sensitive_level_chosen,
			ts->click_sensitive_level_array[ts->click_sensitive_level_chosen]);
	}
	mutex_unlock(&ts->mutex);

	return count;
}

static ssize_t proc_click_sensitive_level_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		snprintf(page, PAGESIZE - 1, "%d\n", -1); /* no support */

	} else {
		snprintf(page, PAGESIZE - 1, "click_sensitive_level : %d\n", ts->click_sensitive_level_chosen); /* support */
	}

	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}
DECLARE_PROC_OPS(proc_click_sensitive_level_fops, simple_open, proc_click_sensitive_level_read, proc_click_sensitive_level_write, NULL);

/*/proc/touchpanel/touch_scene_default_para*/
static int touch_scene_default_para_read(struct seq_file *s, void *v)
{
	int ret = 0;
	int is_default_xml_exit = 0;
	struct touchpanel_data *ts = s->private;
	const struct firmware *fw = NULL;

	char *p_node_img = NULL;
	char *p_node_default_xml = NULL;
	char *touch_scene_config_name_img = NULL;
	char *touch_scene_config_name_default_xml = NULL;
	char *postfix_img = "_sys_touch_scene_config.img";
	char *postfix_default_xml = "/sys_touch_scene_config.xml";
	uint8_t copy_len = 0;

	TPD_INFO("%s: enter.\n",  __func__);
	TPD_INFO("%s:s->size:%lu,s->count:%lu\n", __func__, s->size,
		 s->count);

	if (!ts) {
		return 0;
	}

	if (s->size <= (PAGE_SIZE * 4)) {
		s->count = s->size;
		return 0;
	}

	touch_scene_config_name_img = kzalloc(MAX_FW_NAME_LENGTH, GFP_KERNEL);
	if (touch_scene_config_name_img == NULL) {
		TS_TP_INFO("touch_scene_config_name_img kzalloc error!\n");
		return 0;
	}
	p_node_img = strstr(ts->panel_data.fw_name, ".");
	if (p_node_img == NULL) {
		TS_TP_INFO("p_node_img strstr error!\n");
		kfree(touch_scene_config_name_img);
		return 0;
	}

	touch_scene_config_name_default_xml = kzalloc(MAX_FW_NAME_LENGTH, GFP_KERNEL);
	if (touch_scene_config_name_default_xml == NULL) {
		TS_TP_INFO("touch_scene_config_name_default_xml kzalloc error!\n");
		kfree(touch_scene_config_name_img);
		return 0;
	}
	p_node_default_xml = strstr(strstr(ts->panel_data.fw_name, "/") + 1, "/");
	if (p_node_default_xml == NULL) {
		TS_TP_INFO("p_node_default_xml strstr error!\n");
		kfree(touch_scene_config_name_img);
		kfree(touch_scene_config_name_default_xml);
		return 0;
	}

	copy_len = p_node_img - ts->panel_data.fw_name;
	memcpy(touch_scene_config_name_img, ts->panel_data.fw_name, copy_len);
	strlcat(touch_scene_config_name_img, postfix_img, MAX_FW_NAME_LENGTH);

	TS_TP_INFO("touch_scene_config_name_img is %s\n", touch_scene_config_name_img);

	copy_len = p_node_default_xml - ts->panel_data.fw_name;
	memcpy(touch_scene_config_name_default_xml, ts->panel_data.fw_name, copy_len);
	strlcat(touch_scene_config_name_default_xml, postfix_default_xml, MAX_FW_NAME_LENGTH);

	TS_TP_INFO("touch_scene_config_name_default_xml is %s\n", touch_scene_config_name_default_xml);

	ret = request_firmware(&fw, touch_scene_config_name_default_xml, ts->dev);
	if (ret < 0) {
		TS_TP_INFO("Request firmware failed - %s (%d)\n", touch_scene_config_name_default_xml, ret);
	} else {
		TS_TP_INFO("%s Request ok,size is:%lu\n", touch_scene_config_name_default_xml, fw->size);
	}


	is_default_xml_exit = ret;
	if (is_default_xml_exit < 0) {
		ret = request_firmware(&fw, touch_scene_config_name_img, ts->dev);
		if (ret < 0) {
			TS_TP_INFO("Request firmware failed - %s (%d)\n", touch_scene_config_name_img, ret);
			seq_printf(s, "Request failed, Check the path %s\n", touch_scene_config_name_img);
			seq_printf(s, "Request failed, Check the path %s\n", touch_scene_config_name_default_xml);
			kfree(touch_scene_config_name_img);
			kfree(touch_scene_config_name_default_xml);
			return 0;
		}

		TS_TP_INFO("%s Request ok,size is:%lu\n", touch_scene_config_name_img, fw->size);
	}


	if (fw->size > 0) {
		seq_write(s, fw->data, fw->size);
		TS_TP_INFO("%s:seq_write data ok\n", __func__);
	}

	release_firmware(fw);
	kfree(touch_scene_config_name_img);
	kfree(touch_scene_config_name_default_xml);
	return ret;
}

static int touch_scene_default_para_open(struct inode *inode, struct file *file)
{
	return single_open(file, touch_scene_default_para_read, PDE_DATA(inode));
}

DECLARE_PROC_OPS(touch_scene_default_para_fops, touch_scene_default_para_open, seq_read, NULL, single_release);

/*format is opearation object name:x,y,z,m,m*/
static void touch_scen_config_write(struct touchpanel_data *ts, char *input, int len)
{
	int buf_len = 0;
	int buffer_len = 0;
	int i = 0;
	int j = 0;
	int value = 0;
	int ret = 0;
	int raw_level = 0;
	char *buf = NULL;
	char *get_buf = NULL;
	char single_cmd[SCEN_SINGLE_CMD_SIZE] = {0};

	struct touch_scene_info *scene_info = &ts->scene_info;
	memcpy(single_cmd, input, len);
	get_buf = &single_cmd[0];

	TPD_INFO("single_cmd:%s len:%d.\n", single_cmd, len);
	buf = strchr(get_buf, ':');
	if (buf == NULL) {
		TPD_INFO("not find :, error.\n");
		return;
	}
	buf_len = buf - get_buf;
	if (buf_len >= len - 1) {
		TPD_INFO("buf_len:%d, error.\n", buf_len);
		return;
	}
	buf = &single_cmd[buf_len + 1];
	ret = sscanf(buf, "%d", &value);
	if (ret != 1) {
		TPD_INFO("not find value in buf:%s.\n", buf);
		return;
	}
	single_cmd[buf_len] = '\0';
	for (i = 0, j = 0; i < buf_len; i++) {
		if (get_buf[i] == ' ') {
			j++;
			if (j == 2) {
				break;
			}
		}
	}
	if (j == 2) {
		get_buf = &get_buf[i + 1];
		buffer_len = strlen(get_buf) + 1;
	} else {
		TPD_INFO("get_buff error.\n");
		return;
	}

	for (i = 0; i < MAX_SCENE_LENS; i++) {
		for (j = 0; j < buffer_len; j++) {
			if (get_buf[j] != buffer_scen[i][j]) {
				break;
			}
		}
		if (j == buffer_len) {
			TPD_INFO("get buffer suc, i = %d", i);
			break;
		}
	}
	if (i == MAX_SCENE_LENS) {
		TPD_INFO("input buffer error.\n");
		return;
	}

	switch (i) {
	case SCREEN_LOCK_MODE:
		TPD_INFO("screen_lock_mode not used.\n");
		break;
	case SENSITIVE_LEVEL:
		if (value == scene_info->sensitive_level) {
			TPD_INFO("sensitive_level:%u value:%d.\n", scene_info->sensitive_level, value);
			break;
		}
		mutex_lock(&ts->mutex);
		scene_info->sensitive_level = value;
		if (value < 0) {
			raw_level = -value;
		} else {
			if (value < SENSITIVE_LEVEL_NUM) {
				ts->sensitive_level_chosen = value;
			} else {
				ts->sensitive_level_chosen = SENSITIVE_LEVEL_NUM - 1;
			}
			if (ts->health_monitor_support && ts->sensitive_level_chosen) {
				ts->monitor_data.sensitive_level_chosen = ts->sensitive_level_chosen;
			}
			raw_level = ts->sensitive_level_used_array[ts->sensitive_level_chosen];
		}
		TS_TP_INFO("%s: level=%d value=%d\n", __func__, ts->sensitive_level_chosen, raw_level);
		if (!ts->is_suspended && ts->ts_ops->sensitive_lv_set && ts->tp_scene_para_switch_support) {
			ts->ts_ops->sensitive_lv_set(ts->chip_data, raw_level);
		} else {
			TS_TP_INFO("%s: TP is_suspended or not support.\n", __func__);
		}
		mutex_unlock(&ts->mutex);
		break;
	case SET_PACKAGE_TYPE:
		scene_info->set_package_type = value;
		TPD_INFO("set_package_type is %d\n", scene_info->set_package_type);
		break;
	case PEN_SENSITIVE_LEVEL:
		TPD_INFO("pen_sensitive_level:%u set value:%d.\n", scene_info->pen_sensitive_level, value);
		if (value == scene_info->pen_sensitive_level) {
			break;
		}
		mutex_lock(&ts->mutex);
		scene_info->pen_sensitive_level = value;
		TS_TP_INFO("%s: pen_sensitive_level set:%d.\n", __func__, value);
		if (!ts->is_suspended && ts->ts_ops->pen_sensitive_lv_set && ts->tp_scene_para_switch_support) {
			ts->ts_ops->pen_sensitive_lv_set(ts->chip_data, value);
		} else {
			TS_TP_INFO("%s: TP is_suspended.\n", __func__);
		}
		mutex_unlock(&ts->mutex);
		break;
	case TOUCH_LEAVE_JITTER:
		TPD_INFO("touch_leave_jitter:%u set value:%d.\n", i, value);
		if (value == scene_info->touch_leave_jitter) {
			break;
		}
		mutex_lock(&ts->mutex);
		scene_info->touch_leave_jitter = value;
		if (!ts->is_suspended && ts->ts_ops->touch_leave_jitter_set && ts->tp_scene_para_switch_support) {
			ts->ts_ops->touch_leave_jitter_set(ts->chip_data, value);
		} else {
			TS_TP_INFO("%s: TP is_suspended.\n", __func__);
		}
		mutex_unlock(&ts->mutex);
		break;
	default:
		TPD_INFO("not used.\n");
		break;
	}
	return;
}

static void transfer_scen_cmdList_to_single(struct touchpanel_data *ts, char *info, int w_size)
{
	char *token = NULL;
	char single_cmd[SCEN_SINGLE_CMD_SIZE] = {0};
	char *temp = NULL;

	if (info == NULL) {
		TPD_INFO("info is null\n");
		return;
	}

	temp = info;
	while ((token = strsep(&temp, ";")) != NULL) {
		if ((strlen(token) + 1) <= SCEN_SINGLE_CMD_SIZE) {
			memcpy(single_cmd, token, strlen(token));
			single_cmd[strlen(token)] = '\0';
			touch_scen_config_write(ts, single_cmd, strlen(single_cmd) + 1);
		} else {
			TPD_INFO("token:%s size is beyond\n", token);
		}
	}

	return;
}

static ssize_t proc_touch_scene_para_write(struct file *file, const char __user *buffer,
				 size_t count, loff_t *ppos)
{
	char *buf = NULL;
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	TPD_INFO("%s: enter.\n",  __func__);

	if (count > SCEN_ALL_CMD_SIZE) {
		TPD_INFO("%s: count is too large :%d.\n",  __func__, (int)count);
		return count;
	}

	buf = kzalloc(SCEN_ALL_CMD_SIZE, GFP_KERNEL);

	if (!buf) {
		TPD_INFO("%s kmalloc failed.\n", __func__);
		return count;
	}

	if (copy_from_user(buf, buffer, count)) {
		TPD_INFO("%s: read proc input error.\n", __func__);
		if (buf) {
			kfree(buf);
		}
		return count;
	}

	TPD_INFO("%s: input:%s size:%lu. \n", __func__, buf, count);
	transfer_scen_cmdList_to_single(ts, buf, SCEN_ALL_CMD_SIZE);

	if (buf) {
		kfree(buf);
	}
	return count;
}

int touch_scene_print_func(struct seq_file *s,
			   struct touch_scene_info *scene_info)
{
	if (!scene_info) {
		TPD_INFO("%s read scene info failed.\n", __func__);
		return 0;
	}
	TPD_INFO("%s: enter.\n",  __func__);
	seq_printf(s, "screen_lock_mode:%d\n", scene_info->screen_lock_mode);
	seq_printf(s, "sensitive_level:%d\n", scene_info->sensitive_level);
	seq_printf(s, "set_package_type:%d\n", scene_info->set_package_type);
	seq_printf(s, "pen_sensitive_level:%d\n", scene_info->pen_sensitive_level);
	seq_printf(s, "touch_leave_jitter:%d\n", scene_info->touch_leave_jitter);
	seq_printf(s, "\n");
	return 0;
}

static int proc_touch_scene_para_read(struct seq_file *s, void *v)
{
	struct touchpanel_data *ts = s->private;
	struct touch_scene_info *scene_info = &ts->scene_info;

	if (!scene_info) {
		TPD_INFO("%s read scene info failed.\n", __func__);
	}

	return touch_scene_print_func(s, scene_info);
}

static int proc_touch_scene_para_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_touch_scene_para_read, PDE_DATA(inode));
}

DECLARE_PROC_OPS(proc_touch_scene_para_fops, proc_touch_scene_para_open, seq_read, proc_touch_scene_para_write, single_release);

static ssize_t proc_diaphragm_touch_level_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[6] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		TPD_INFO("%s: ts is NULL\n", __func__);
		return count;
	}
	touchpanel_trusted_touch_completion(ts);

	if (!ts->ts_ops->diaphragm_touch_lv_set) {
		TPD_INFO("%s:not support ts_ops->diaphragm_touch_lv_set callback\n", __func__);
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 5);

	if (kstrtoint(buf, 10, &value)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	mutex_lock(&ts->mutex);
	ts->diaphragm_touch_level_chosen = value;

	TPD_INFO("%s: level=%d\n", __func__, ts->diaphragm_touch_level_chosen);
	if (!ts->is_suspended) {
		ts->ts_ops->diaphragm_touch_lv_set(ts->chip_data, value);
	} else {
		TPD_INFO("%s: TP is_suspended.\n", __func__);
	}
	mutex_unlock(&ts->mutex);

	return count;
}

static ssize_t proc_diaphragm_touch_level_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts || !ts->diaphragm_touch_support) {
		snprintf(page, PAGESIZE - 1, "%d\n", -1);
	} else {
		snprintf(page, PAGESIZE - 1, "%u\n", ts->diaphragm_touch_level_chosen);
	}
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

DECLARE_PROC_OPS(proc_diaphragm_touch_level_fops, simple_open, proc_diaphragm_touch_level_read, proc_diaphragm_touch_level_write, NULL);

static int calibrate_fops_read_func(struct seq_file *s, void *v)
{
	struct touchpanel_data *ts = s->private;

	if (!ts->ts_ops->calibrate) {
		TPD_INFO("[TP]no calibration, need back virtual calibration result");
		seq_printf(s, "0 error, calibration and verify success\n");
		return 0;
	}

	disable_irq_nosync(ts->irq);
	mutex_lock(&ts->mutex);
	if (!ts->touch_count) {
		ts->ts_ops->calibrate(s, ts->chip_data);
	} else {
		seq_printf(s, "1 error, release touch on the screen, now has %d touch\n", ts->touch_count);
	}
	mutex_unlock(&ts->mutex);
	enable_irq(ts->irq);

	return 0;
}

static int proc_calibrate_fops_open(struct inode *inode, struct file *file)
{
	return single_open(file, calibrate_fops_read_func, PDE_DATA(inode));
}

DECLARE_PROC_OPS(proc_calibrate_fops, proc_calibrate_fops_open, seq_read, NULL, single_release);


static int cal_status_read_func(struct seq_file *s, void *v)
{
	bool cal_needed = false;
	struct touchpanel_data *ts = s->private;

	if (!ts->ts_ops->get_cal_status) {
		TPD_INFO("[TP]no calibration status, need back virtual calibration status");
		seq_printf(s, "0 error, calibration data is ok\n");
		return 0;
	}

	mutex_lock(&ts->mutex);
	cal_needed = ts->ts_ops->get_cal_status(s, ts->chip_data);
	if (cal_needed) {
		seq_printf(s, "1 error, need do calibration\n");
	} else {
		seq_printf(s, "0 error, calibration data is ok\n");
	}
	mutex_unlock(&ts->mutex);

	return 0;
}

static int proc_cal_status_fops_open(struct inode *inode, struct file *file)
{
	return single_open(file, cal_status_read_func, PDE_DATA(inode));
}

DECLARE_PROC_OPS(proc_cal_status_fops, proc_cal_status_fops_open, seq_read, NULL, single_release);


#define MAX_CONTROL_CMD 255
#define BUF_SIZE        6

static ssize_t proc_pencil_connect_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[BUF_SIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		TPD_INFO("%s ts is null \n", __func__);
		return count;
	}

	if (count > BUF_SIZE - 1) {
		TS_TP_INFO("%s over size control cmd is %lu\n", __func__, count);
		return count;
	}

	if (tp_copy_from_user(buf, sizeof(buf), buffer, count, BUF_SIZE - 1)) {
		TS_TP_INFO("%s: read proc input error.\n", __func__);
		return count;
	}

	if (kstrtoint(buf, 10, &value)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	if (value != ts->is_pen_connected) {
		mutex_lock(&ts->mutex);
		ts->is_pen_connected = !!value;
		TS_TP_INFO("%s: check pen state1 : %d-%d %d, is_suspended: %d\n", __func__, value, ts->is_pen_connected, ts->is_pen_attracted, ts->is_suspended);
		if (ts->is_pen_connected && ts->ts_ops->notify_pencil_type) {
		ts->ts_ops->notify_pencil_type(ts->chip_data, value);
        }
		if (!ts->is_suspended && (ts->suspend_state == TP_SPEEDUP_RESUME_COMPLETE)) {	/* For close pencil scan under screen on mode */
			ts->ts_ops->mode_switch(ts->chip_data, MODE_PEN_SCAN, ts->is_pen_connected);
		} else if (ts->is_suspended && ((ts->gesture_enable & 0x01) == 1 || ts->fp_enable) && (!ts->hall_status)) {	/* For close pencil scan under gesture mode */
			ts->ts_ops->mode_switch(ts->chip_data, MODE_PEN_SCAN, \
			(ts->gesture_enable_indep & (1 << PENDETECT)) && ts->is_pen_connected && !ts->is_pen_attracted);
		}
		mutex_unlock(&ts->mutex);
	}

	return count;
}

static ssize_t proc_pencil_connect_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		snprintf(page, PAGESIZE - 1, "%d\n", -1); /* error handler */
	} else {
		snprintf(page, PAGESIZE - 1, "%d\n", ts->is_pen_connected);
	}
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}
DECLARE_PROC_OPS(proc_pencil_connect_fops, simple_open, proc_pencil_connect_read, proc_pencil_connect_write, NULL);

static ssize_t proc_pencil_control_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[BUF_SIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		TPD_INFO("%s ts is null \n", __func__);
		return count;
	}

	if (count > MAX_CONTROL_CMD) {
		TS_TP_INFO("%s over size control cmd is %lu\n", __func__, count);
		return count;
	}

	if (tp_copy_from_user(buf, sizeof(buf), buffer, count, BUF_SIZE - 1)) {
		TS_TP_INFO("%s: read proc input error.\n", __func__);
		return count;
	}

	if (kstrtoint(buf, 10, &value)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	mutex_lock(&ts->mutex);
	if (!ts->is_suspended) {
		ts->ts_ops->mode_switch(ts->chip_data, MODE_PEN_CTL, value);
	} else {
		TS_TP_INFO("%s : now not resume state\n", __func__);
	}
	mutex_unlock(&ts->mutex);

	return count;
}

#define PEN_CTL_FEEDBACK 0xffff
static ssize_t proc_pencil_control_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		snprintf(page, PAGESIZE - 1, "%d\n", -1); /* error handler */
	} else {
		snprintf(page, PAGESIZE - 1, "%d\n", ts->ts_ops->mode_switch(ts->chip_data, MODE_PEN_CTL, PEN_CTL_FEEDBACK));
	}
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}
DECLARE_PROC_OPS(proc_pencil_control_fops, simple_open, proc_pencil_control_read, proc_pencil_control_write, NULL);

static ssize_t proc_pencil_opp_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		snprintf(page, PAGESIZE - 1, "%d\n", -1); /* error handler */
	} else {
		snprintf(page, PAGESIZE - 1, "%d\n", ts->pen_support_opp);
	}
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}
DECLARE_PROC_OPS(proc_pencil_opp_fops, simple_open, proc_pencil_opp_read, NULL, NULL);

/*******Part4:Debug node Function  Area********************/

#ifdef CONFIG_OPLUS_TP_APK
void log_buf_write(struct touchpanel_data *ts, u8 value)
{
	if (ts->log_buf) {
		u16 *head;
		head = (u16 *)(&ts->log_buf[0]);

		if ((*head) == 0) {
			(*head) = 1;
		}

		if ((*head) < 1023) {
			(*head)++;

		} else {
			(*head) = 2;
		}

		ts->log_buf[*head] = value;
	}
}
EXPORT_SYMBOL(log_buf_write);

static int log_buf_read(struct touchpanel_data *ts, char *buf, int len)
{
	if (ts->log_buf == NULL) {
		return 0;
	}

	if (len > 1024) {
		len = 1024;
	}

	memcpy((u8 *)buf, ts->log_buf, len);
	return len;
}


static char apk_charger_sta_read(struct touchpanel_data *ts)
{
	if (ts->apk_op->apk_charger_get) {
		if (ts->apk_op->apk_charger_get(ts->chip_data)) {
			return '1';
		}

		return '0';
	}

	return '-';
}

static int apk_data_read(struct touchpanel_data *ts, char *buf, int len)
{
	switch (ts->data_now) {
	case BASE_DATA:
		if (ts->apk_op->apk_basedata_get) {
			return ts->apk_op->apk_basedata_get(ts->chip_data, buf, len);
		}

		break;

	case DIFF_DATA:
		if (ts->apk_op->apk_diffdata_get) {
			return ts->apk_op->apk_diffdata_get(ts->chip_data, buf, len);
		}

		break;

	case DEBUG_INFO:
		return log_buf_read(ts, buf, len);
		break;

	case RAW_DATA:
		if (ts->apk_op->apk_rawdata_get) {
			return ts->apk_op->apk_rawdata_get(ts->chip_data, buf, len);
		}

		break;

	case BACK_DATA:
		if (ts->apk_op->apk_backdata_get) {
			return ts->apk_op->apk_backdata_get(ts->chip_data, buf, len);
		}

		break;

	default:
		break;
	}

	buf[0] = '-';
	return 1;
}

static char apk_earphone_sta_read(struct touchpanel_data *ts)
{
	if (ts->apk_op->apk_earphone_get) {
		if (ts->apk_op->apk_earphone_get(ts->chip_data)) {
			return '1';
		}

		return '0';
	}

	return '-';
}

static int apk_gesture_read(struct touchpanel_data *ts, char *buf, int len)
{
	if (ts->apk_op->apk_gesture_get) {
		if (ts->apk_op->apk_gesture_get(ts->chip_data)) {
			return ts->apk_op->apk_gesture_info(ts->chip_data, buf, len);
		}

		buf[0] = '0';
		return 1;
	}

	buf[0] = '-';
	return 1;
}

static int apk_info_read(struct touchpanel_data *ts, char *buf, int len)
{
	if (ts->apk_op->apk_tp_info_get) {
		return ts->apk_op->apk_tp_info_get(ts->chip_data, buf, len);
	}

	buf[0] = '-';
	return 1;
}

static char apk_noise_read(struct touchpanel_data *ts)
{
	if (ts->apk_op->apk_noise_get) {
		if (ts->apk_op->apk_noise_get(ts->chip_data)) {
			return '1';
		}

		return '0';
	}

	return '-';
}

static char apk_water_read(struct touchpanel_data *ts)
{
	if (ts->apk_op->apk_water_get) {
		if (ts->apk_op->apk_water_get(ts->chip_data) == 1) {
			return '1';
		}

		if (ts->apk_op->apk_water_get(ts->chip_data) == 2) {
			return '2';
		}

		return '0';
	}

	return '-';
}

static char apk_proximity_read(struct touchpanel_data *ts)
{
	if (ts->apk_op->apk_proximity_dis) {
		int dis;
		dis = ts->apk_op->apk_proximity_dis(ts->chip_data);

		if (dis > 0) {
			if (dis == 1) {
				return '1';
			}

			return '2';
		}

		return '0';
	}

	return '-';
}

static char apk_debug_sta(struct touchpanel_data *ts)
{
	if (ts->apk_op->apk_debug_get) {
		if (ts->apk_op->apk_debug_get(ts->chip_data)) {
			return '1';
		}

		return '0';
	}

	return '-';
}

static char apk_game_read(struct touchpanel_data *ts)
{
	if (ts->apk_op->apk_game_get) {
		if (ts->apk_op->apk_game_get(ts->chip_data)) {
			return '1';
		}

		return '0';
	}

	return '-';
}

static ssize_t oplus_apk_read(struct file *file,
			      char __user *user_buf,
			      size_t count,
			      loff_t *ppos)
{
	char *buf;
	int len = 0;
	int ret = 0;
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		TPD_INFO("ts not exist!\n");
		return -ENODEV;
	}

	if (ts->apk_op == NULL) {
		TS_TP_INFO("ts apk_op not exist!\n");
		return -ENODEV;
	}

	if (*ppos != 0 || count < 1) {
		return 0;
	}

	TS_TP_INFO("apk read is %c, count is %d.\n", (char)ts->type_now, (int)count);
	buf = kzalloc(count, GFP_KERNEL);

	if (IS_ERR(buf) || buf == NULL) {
		ret = -EFAULT;
		goto read_exit;
	}

	mutex_lock(&ts->mutex);

	switch (ts->type_now) {
	case APK_CHARGER:
		buf[0] = apk_charger_sta_read(ts);
		len = 1;
		break;

	case APK_DATA:
		len = apk_data_read(ts, buf, count);
		break;

	case APK_EARPHONE:
		buf[0] = apk_earphone_sta_read(ts);
		len = 1;
		break;

	case APK_GESTURE:
		len = apk_gesture_read(ts, buf, count);
		break;

	case APK_INFO:
		len = apk_info_read(ts, buf, count);
		break;

	case APK_NOISE:
		buf[0] = apk_noise_read(ts);
		len = 1;
		break;

	case APK_WATER:
		buf[0] = apk_water_read(ts);
		len = 1;
		break;

	case APK_PROXIMITY:
		buf[0] = apk_proximity_read(ts);
		len = 1;
		break;

	case APK_DEBUG_MODE:
		buf[0] = apk_debug_sta(ts);
		len = 1;
		break;

	case APK_GAME_MODE:
		buf[0] = apk_game_read(ts);
		len = 1;
		break;

	default:
		break;
	}

	if (len == 1 && len < count) {
		buf[len] = '\n';
		len++;
	}

	mutex_unlock(&ts->mutex);

	if (copy_to_user(user_buf, buf, len)) {
		TS_TP_INFO("%s: can not copy the buf.\n", __func__);
		ret = -EFAULT;
		goto read_exit;
	}

	ret = len;
	*ppos += ret;

read_exit:

	if (buf != NULL) {
		kfree(buf);
	}

	return ret;
}

static void apk_charger_switch(struct touchpanel_data *ts, char on_off)
{
	if (ts->apk_op->apk_charger_set) {
		if (on_off == '1') {
			ts->apk_op->apk_charger_set(ts->chip_data, true);

		} else if (on_off == '0') {
			ts->apk_op->apk_charger_set(ts->chip_data, false);
		}
	}
}

static void apk_earphone_switch(struct touchpanel_data *ts, char on_off)
{
	if (ts->apk_op->apk_earphone_set) {
		if (on_off == '1') {
			ts->apk_op->apk_earphone_set(ts->chip_data, true);

		} else if (on_off == '0') {
			ts->apk_op->apk_earphone_set(ts->chip_data, false);
		}
	}
}

static void apk_gesture_debug(struct touchpanel_data *ts, char on_off)
{
	if (ts->apk_op->apk_gesture_debug) {
		if (on_off == '1') {
			if (ts->gesture_buf == NULL) {
				ts->gesture_buf = kzalloc(1024, GFP_KERNEL);
			}

			if (ts->gesture_buf) {
				ts->gesture_debug_sta = true;
				ts->apk_op->apk_gesture_debug(ts->chip_data, true);

			} else {
				ts->gesture_debug_sta = false;
				ts->apk_op->apk_gesture_debug(ts->chip_data, false);
			}

		} else if (on_off == '0') {
			ts->apk_op->apk_gesture_debug(ts->chip_data, false);

			if (ts->gesture_buf) {
				kfree(ts->gesture_buf);
				ts->gesture_buf = NULL;
			}

			ts->gesture_debug_sta = false;
		}
	}
}

static void apk_noise_switch(struct touchpanel_data *ts, char on_off)
{
	if (ts->apk_op->apk_noise_set) {
		if (on_off == '1') {
			ts->apk_op->apk_noise_set(ts->chip_data, true);

		} else if (on_off == '0') {
			ts->apk_op->apk_noise_set(ts->chip_data, false);
		}
	}
}

static void apk_water_switch(struct touchpanel_data *ts, char on_off)
{
	if (ts->apk_op->apk_water_set) {
		if (on_off == '1') {
			ts->apk_op->apk_water_set(ts->chip_data, 1);

		} else if (on_off == '2') {
			ts->apk_op->apk_water_set(ts->chip_data, 2);

		} else if (on_off == '0') {
			ts->apk_op->apk_water_set(ts->chip_data, 0);
		}
	}
}

static void apk_proximity_switch(struct touchpanel_data *ts,
				 char on_off)
{
	if (ts->apk_op->apk_proximity_set) {
		if (on_off == '1') {
			ts->apk_op->apk_proximity_set(ts->chip_data, true);

		} else if (on_off == '0') {
			ts->apk_op->apk_proximity_set(ts->chip_data, false);
		}
	}
}

static void apk_debug_switch(struct touchpanel_data *ts, char on_off)
{
	if (ts->apk_op->apk_debug_set) {
		if (on_off == '1') {
			ts->apk_op->apk_debug_set(ts->chip_data, true);

			if (ts->log_buf == NULL) {
				ts->log_buf = kzalloc(1024, GFP_KERNEL);
			}

		} else if (on_off == '0') {
			ts->apk_op->apk_debug_set(ts->chip_data, false);

			if (ts->log_buf) {
				kfree(ts->log_buf);
				ts->log_buf = NULL;
			}
		}
	}
}

static void apk_data_type_set(struct touchpanel_data *ts, char ch)
{
	APK_DATA_TYPE type;
	type = (APK_DATA_TYPE)ch;

	switch (type) {
	case BASE_DATA:
	case DIFF_DATA:
	case DEBUG_INFO:
	case RAW_DATA:
	case BACK_DATA:
		ts->data_now = type;

		if (ts->apk_op->apk_data_type_set) {
			ts->apk_op->apk_data_type_set(ts->chip_data, ts->data_now);
		}

		break;

	default:
		break;
	}
}

static void apk_game_switch(struct touchpanel_data *ts, char on_off)
{
	if (ts->apk_op->apk_game_set) {
		if (on_off == '1') {
			ts->apk_op->apk_game_set(ts->chip_data, true);

		} else if (on_off == '0') {
			ts->apk_op->apk_game_set(ts->chip_data, false);
		}
	}
}

static ssize_t oplus_apk_write(struct file *file,
			       const char __user *buffer,
			       size_t count,
			       loff_t *ppos)
{
	char *buf;
	APK_SWITCH_TYPE type;
	int ret = count;
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		TPD_INFO("ts not exist!\n");
		return -ENODEV;
	}

	if (ts->apk_op == NULL) {
		TS_TP_INFO("ts apk_op not exist!\n");
		return -ENODEV;
	}

	if (count < 1) {
		return 0;
	}


	buf = kzalloc(count, GFP_KERNEL);

	if (IS_ERR(buf) || buf == NULL) {
		ret = -EFAULT;
		goto write_exit;
	}

	if (copy_from_user(buf, buffer, count)) {
		TS_TP_INFO("%s: can not copy the buf.\n", __func__);
		ret = -EFAULT;
		goto write_exit;
	}

	mutex_lock(&ts->mutex);

	type = (APK_SWITCH_TYPE)buf[0];
	TS_TP_INFO("apk write type is %c, count is %d.\n", (char)type,
		 (int)count);

	if (count > 1) {
		switch (type) {
		case APK_CHARGER:
			apk_charger_switch(ts, buf[1]);
			break;

		case APK_DATA:
			apk_data_type_set(ts, buf[1]);
			break;

		case APK_EARPHONE:
			apk_earphone_switch(ts, buf[1]);
			break;

		case APK_GESTURE:
			apk_gesture_debug(ts, buf[1]);
			break;

		case APK_INFO: /* read only, do nothing in write.*/
			break;

		case APK_NOISE:
			apk_noise_switch(ts, buf[1]);
			break;

		case APK_PROXIMITY:
			apk_proximity_switch(ts, buf[1]);
			break;

		case APK_WATER:
			apk_water_switch(ts, buf[1]);
			break;

		case APK_DEBUG_MODE:
			apk_debug_switch(ts, buf[1]);
			break;

		case APK_GAME_MODE:
			apk_game_switch(ts, buf[1]);
			break;

		default:
			type = APK_NULL;
			break;
		}
	}

	ts->type_now = type;
	mutex_unlock(&ts->mutex);

write_exit:

	if (buf != NULL) {
		kfree(buf);
	}

	return ret;
}

DECLARE_PROC_OPS(proc_oplus_apk_fops, simple_open, oplus_apk_read, oplus_apk_write, NULL);

#endif /*end of CONFIG_OPLUS_TP_APK*/

/*proc/touchpanel/debug_info/baseline*/
static int tp_baseline_debug_read_func(struct seq_file *s, void *v)
{
	struct touchpanel_data *ts = s->private;
	struct debug_info_proc_operations *debug_info_ops;

	if (!ts) {
		return 0;
	}

	if(true != ts->bus_ready) {
		TS_TP_INFO("tp bus not ready, returnd");
		return 0;
	}

	debug_info_ops = (struct debug_info_proc_operations *)(ts->debug_info_ops);

	if (!debug_info_ops) {
		TP_INFO(ts->tp_index, "debug_info_ops==NULL");
		return 0;
	}

	if (!debug_info_ops->baseline_read
			&& !debug_info_ops->baseline_blackscreen_read) {
		seq_printf(s, "Not support baseline proc node\n");
		return 0;
	}

	if (tp_status(ts) == SLEEP_MODE) {
		seq_printf(s, "Not in resume over or gesture state\n");
		return 0;
	}

	/*the diff  is big than one page, so do twice.*/
	if (s->size <= (PAGE_SIZE * 2)) {
		s->count = s->size;
		return 0;
	}

	if (ts->int_mode == BANNABLE) {
		disable_irq_nosync(ts->irq);
	}

	mutex_lock(&ts->mutex);
	if (ts->int_mode == UNBANNABLE && !tp_irq_check(ts)) {
		seq_printf(s, "Irq not available\n");
		mutex_unlock(&ts->mutex);
		return 0;
	}

	if (ts->is_suspended && ts->gesture_enable) {
		if (debug_info_ops->baseline_blackscreen_read) {
			debug_info_ops->baseline_blackscreen_read(s, ts->chip_data);
		}

	} else {
		if (debug_info_ops->baseline_read) {
			debug_info_ops->baseline_read(s, ts->chip_data);
		}
	}
	if (!ts->screenshot_not_reset_support) {
		/*step6: return to normal mode*/
		ts->ts_ops->reset(ts->chip_data);
	}

	if (ts->is_suspended == 0) {
		tp_touch_btnkey_release(ts->tp_index);
	}

	operate_mode_switch(ts);

	mutex_unlock(&ts->mutex);

	if (ts->int_mode == BANNABLE) {
		enable_irq(ts->irq);
	}

	return 0;
}

static int data_baseline_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_baseline_debug_read_func, PDE_DATA(inode));
}

DECLARE_PROC_OPS(tp_baseline_data_proc_fops, data_baseline_open, seq_read, NULL, single_release);

/*proc/touchpanel/debug_info/pen_baseline*/
static int tp_pen_baseline_debug_read_func(struct seq_file *s, void *v)
{
	struct touchpanel_data *ts = s->private;
	struct debug_info_proc_operations *debug_info_ops;

	if (!ts) {
		return 0;
	}

	if(true != ts->bus_ready) {
		TS_TP_INFO("tp bus not ready, returnd");
		return 0;
	}

	debug_info_ops = (struct debug_info_proc_operations *)(ts->debug_info_ops);

	if (!debug_info_ops) {
		TP_INFO(ts->tp_index, "debug_info_ops==NULL");
		return 0;
	}

	if (!debug_info_ops->pen_baseline_read
			&& !debug_info_ops->baseline_blackscreen_read) {
		seq_printf(s, "Not support pen baseline proc node\n");
		return 0;
	}

	if (tp_status(ts) == SLEEP_MODE) {
		seq_printf(s, "Not in resume over or gesture state\n");
		return 0;
	}

	/*the diff  is big than one page, so do twice.*/
	if (s->size <= (PAGE_SIZE * 2)) {
		s->count = s->size;
		return 0;
	}

	if (ts->int_mode == BANNABLE) {
		disable_irq_nosync(ts->irq);
	}

	mutex_lock(&ts->mutex);
	if (ts->int_mode == UNBANNABLE && !tp_irq_check(ts)) {
		seq_printf(s, "Irq not available\n");
		mutex_unlock(&ts->mutex);
		return 0;
	}

	if (ts->is_suspended && ts->gesture_enable) {
		if (debug_info_ops->baseline_blackscreen_read) {
			debug_info_ops->baseline_blackscreen_read(s, ts->chip_data);
		}

	} else {
		if (debug_info_ops->pen_baseline_read) {
			debug_info_ops->pen_baseline_read(s, ts->chip_data);
		}
	}

	/*step6: return to normal mode*/
	ts->ts_ops->reset(ts->chip_data);

	if (ts->is_suspended == 0) {
		tp_touch_btnkey_release(ts->tp_index);
	}

	operate_mode_switch(ts);

	mutex_unlock(&ts->mutex);

	if (ts->int_mode == BANNABLE) {
		enable_irq(ts->irq);
	}

	return 0;
}

static int data_pen_baseline_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_pen_baseline_debug_read_func, PDE_DATA(inode));
}
DECLARE_PROC_OPS(tp_pen_baseline_data_proc_fops, data_pen_baseline_open, seq_read, NULL, single_release);

/*proc/touchpanel/debug_info/delta*/
static int tp_delta_debug_read_func(struct seq_file *s, void *v)
{
	struct touchpanel_data *ts = s->private;
	struct debug_info_proc_operations *debug_info_ops;

	if (!ts) {
		return 0;
	}

	if(true != ts->bus_ready) {
		TS_TP_INFO("tp bus not ready, returnd");
		return 0;
	}

	debug_info_ops = (struct debug_info_proc_operations *)ts->debug_info_ops;

	if (!debug_info_ops) {
		return 0;
	}

	if (!debug_info_ops->delta_read) {
		seq_printf(s, "Not support delta proc node\n");
		return 0;
	}

	if (tp_status(ts) == SLEEP_MODE) {
		seq_printf(s, "Not in resume over state\n");
		return 0;
	}

	/*the diff  is big than one page, so do twice.*/
	if (s->size <= (PAGE_SIZE * 2)) {
		s->count = s->size;
		return 0;
	}

	if (ts->int_mode == BANNABLE) {
		disable_irq_nosync(ts->irq);
	}

	mutex_lock(&ts->mutex);
	if (ts->int_mode == UNBANNABLE && !tp_irq_check(ts)) {
		seq_printf(s, "Irq not available\n");
		mutex_unlock(&ts->mutex);
		return 0;
	}
	debug_info_ops->delta_read(s, ts->chip_data);
	mutex_unlock(&ts->mutex);

	if (ts->int_mode == BANNABLE) {
		enable_irq(ts->irq);
	}

	return 0;
}

static int data_delta_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_delta_debug_read_func, PDE_DATA(inode));
}

DECLARE_PROC_OPS(tp_delta_data_proc_fops, data_delta_open, seq_read, NULL, single_release);

/*proc/touchpanel/debug_info/pen_delta*/
static int tp_pen_delta_debug_read_func(struct seq_file *s, void *v)
{
	struct touchpanel_data *ts = s->private;
	struct debug_info_proc_operations *debug_info_ops;

	if (!ts) {
		return 0;
	}

	if(true != ts->bus_ready) {
		TS_TP_INFO("tp bus not ready, returnd");
		return 0;
	}

	debug_info_ops = (struct debug_info_proc_operations *)ts->debug_info_ops;

	if (!debug_info_ops) {
		return 0;
	}

	if (!debug_info_ops->pen_delta_read) {
		seq_printf(s, "Not support pen delta proc node\n");
		return 0;
	}

	if (tp_status(ts) == SLEEP_MODE) {
		seq_printf(s, "Not in resume over state\n");
		return 0;
	}

	/*the diff  is big than one page, so do twice.*/
	if (s->size <= (PAGE_SIZE * 2)) {
		s->count = s->size;
		return 0;
	}

	if (ts->int_mode == BANNABLE) {
		disable_irq_nosync(ts->irq);
	}

	mutex_lock(&ts->mutex);
	if (ts->int_mode == UNBANNABLE && !tp_irq_check(ts)) {
		seq_printf(s, "Irq not available\n");
		mutex_unlock(&ts->mutex);
		return 0;
	}
	debug_info_ops->pen_delta_read(s, ts->chip_data);
	mutex_unlock(&ts->mutex);

	if (ts->int_mode == BANNABLE) {
		enable_irq(ts->irq);
	}

	return 0;
}

static int data_pen_delta_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_pen_delta_debug_read_func, PDE_DATA(inode));
}

DECLARE_PROC_OPS(tp_pen_delta_data_proc_fops, data_pen_delta_open, seq_read, NULL, single_release);

/*proc/touchpanel/debug_info/self_delta*/
static int tp_self_delta_debug_read_func(struct seq_file *s, void *v)
{
	struct touchpanel_data *ts = s->private;
	struct debug_info_proc_operations *debug_info_ops;

	if (!ts) {
		return 0;
	}

	if(true != ts->bus_ready) {
		TS_TP_INFO("tp bus not ready, returnd");
		return 0;
	}

	debug_info_ops = (struct debug_info_proc_operations *)ts->debug_info_ops;

	if (!debug_info_ops) {
		return 0;
	}

	if (!debug_info_ops->self_delta_read) {
		seq_printf(s, "Not support self_delta proc node\n");
		return 0;
	}

	if (tp_status(ts) == SLEEP_MODE) {
		seq_printf(s, "Not in resume over state\n");
		return 0;
	}

	if (ts->int_mode == BANNABLE) {
		disable_irq_nosync(ts->irq);
	}

	mutex_lock(&ts->mutex);
	if (ts->int_mode == UNBANNABLE && !tp_irq_check(ts)) {
		seq_printf(s, "Irq not available\n");
		mutex_unlock(&ts->mutex);
		return 0;
	}
	debug_info_ops->self_delta_read(s, ts->chip_data);
	mutex_unlock(&ts->mutex);

	if (ts->int_mode == BANNABLE) {
		enable_irq(ts->irq);
	}

	return 0;
}

static int data_self_delta_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_self_delta_debug_read_func, PDE_DATA(inode));
}

DECLARE_PROC_OPS(tp_self_delta_data_proc_fops, data_self_delta_open, seq_read, NULL, single_release);

/*proc/touchpanel/debug_info/self_raw*/
static int tp_self_raw_debug_read_func(struct seq_file *s, void *v)
{
	struct touchpanel_data *ts = s->private;
	struct debug_info_proc_operations *debug_info_ops;

	if (!ts) {
		return 0;
	}

	if(true != ts->bus_ready) {
		TS_TP_INFO("tp bus not ready, returnd");
		return 0;
	}

	debug_info_ops = (struct debug_info_proc_operations *)ts->debug_info_ops;

	if (!debug_info_ops) {
		return 0;
	}

	if (!debug_info_ops->self_raw_read) {
		seq_printf(s, "Not support self_raw proc node\n");
		return 0;
	}

	if (tp_status(ts) == SLEEP_MODE) {
		seq_printf(s, "Not in resume over state\n");
		return 0;
	}

	if (ts->int_mode == BANNABLE) {
		disable_irq_nosync(ts->irq);
	}

	mutex_lock(&ts->mutex);
	if (ts->int_mode == UNBANNABLE && !tp_irq_check(ts)) {
		seq_printf(s, "Irq not available\n");
		mutex_unlock(&ts->mutex);
		return 0;
	}
	debug_info_ops->self_raw_read(s, ts->chip_data);
	mutex_unlock(&ts->mutex);

	if (ts->int_mode == BANNABLE) {
		enable_irq(ts->irq);
	}

	return 0;
}

static int data_self_raw_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_self_raw_debug_read_func, PDE_DATA(inode));
}

DECLARE_PROC_OPS(tp_self_raw_data_proc_fops, data_self_raw_open, seq_read, NULL, single_release);

/*proc/touchpanel/debug_info/main_register*/
static int tp_main_register_read_func(struct seq_file *s, void *v)
{
	struct touchpanel_data *ts = s->private;
	struct debug_info_proc_operations *debug_info_ops;
#ifndef CONFIG_REMOVE_OPLUS_FUNCTION
	struct monitor_data *monitor_data = &ts->monitor_data;
#endif

	if (!ts) {
		return 0;
	}

	if(true != ts->bus_ready) {
		TS_TP_INFO("tp bus not ready, returnd");
		return 0;
	}

	debug_info_ops = (struct debug_info_proc_operations *)ts->debug_info_ops;

	if (!debug_info_ops) {
		return 0;
	}

	if (!debug_info_ops->main_register_read) {
		seq_printf(s, "Not support main_register proc node\n");
		return 0;
	}

	if (tp_status(ts) == SLEEP_MODE) {
		seq_printf(s, "Not in resume over state\n");
		return 0;
	}

	if (ts->int_mode == BANNABLE) {
		disable_irq_nosync(ts->irq);
	}

	mutex_lock(&ts->mutex);
	if (ts->int_mode == UNBANNABLE && !tp_irq_check(ts)) {
		seq_printf(s, "Irq not available\n");
		mutex_unlock(&ts->mutex);
		return 0;
	}
	seq_printf(s, "touch_count:%d\n", ts->touch_count);
	debug_info_ops->main_register_read(s, ts->chip_data);

	if (ts->kernel_grip_support) {
		seq_printf(s, "kernel_grip_info:\n");
		kernel_grip_print_func(s, ts->grip_info);
	}
#ifndef CONFIG_REMOVE_OPLUS_FUNCTION
	if (ts->health_monitor_support && tp_debug == 2) {
		if (monitor_data->fw_version) {
			memset(monitor_data->fw_version, 0, MAX_DEVICE_VERSION_LENGTH);
			strncpy(monitor_data->fw_version, ts->panel_data.manufacture_info.version,
				strlen(ts->panel_data.manufacture_info.version));
		}

		tp_healthinfo_read(s, monitor_data);
	}
#endif
	mutex_unlock(&ts->mutex);

	if (ts->int_mode == BANNABLE) {
		enable_irq(ts->irq);
	}

	return 0;
}

static int main_register_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_main_register_read_func, PDE_DATA(inode));
}

DECLARE_PROC_OPS(tp_main_register_proc_fops, main_register_open, seq_read, NULL, single_release);

/*proc/touchpanel/debug_info/reserve*/
static int tp_reserve_read_func(struct seq_file *s, void *v)
{
	struct touchpanel_data *ts = s->private;
	struct debug_info_proc_operations *debug_info_ops;

	if (!ts) {
		return 0;
	}

	if(true != ts->bus_ready) {
		TS_TP_INFO("tp bus not ready, returnd");
		return 0;
	}

	debug_info_ops = (struct debug_info_proc_operations *)ts->debug_info_ops;

	if (!debug_info_ops) {
		return 0;
	}

	if (!debug_info_ops->reserve_read) {
		seq_printf(s, "Not support main_register proc node\n");
		return 0;
	}

	if (tp_status(ts) == SLEEP_MODE) {
		seq_printf(s, "Not in resume over state\n");
		return 0;
	}

	if (ts->int_mode == BANNABLE) {
		disable_irq_nosync(ts->irq);
	}

	mutex_lock(&ts->mutex);
	if (ts->int_mode == UNBANNABLE && !tp_irq_check(ts)) {
		seq_printf(s, "Irq not available\n");
		mutex_unlock(&ts->mutex);
		return 0;
	}
	debug_info_ops->reserve_read(s, ts->chip_data);
	mutex_unlock(&ts->mutex);

	if (ts->int_mode == BANNABLE) {
		enable_irq(ts->irq);
	}

	return 0;
}

static int reserve_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_reserve_read_func, PDE_DATA(inode));
}

DECLARE_PROC_OPS(tp_reserve_proc_fops, reserve_open, seq_read, NULL, single_release);

/*proc/touchpanel/debug_info/data_limit*/
static int tp_limit_data_read_func(struct seq_file *s, void *v)
{
	struct touchpanel_data *ts = s->private;

	if (!ts) {
		return 0;
	}

	tp_limit_read(s, ts);

	return 0;
}

static int limit_data_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_limit_data_read_func, PDE_DATA(inode));
}

DECLARE_PROC_OPS(tp_limit_data_proc_fops, limit_data_open, seq_read, NULL, single_release);

/*proc/touchpanel/debug_info/data_record*/
static ssize_t tp_data_record_write_func(struct file *file,
				    const char __user *buffer, size_t count, loff_t *ppos)
{
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	struct debug_info_proc_operations *debug_info_ops = NULL;
	int value = 0;
	char buf[4] = {0};

	TPD_DETAIL("%s tp_limit_data write enter\n", __func__);

	if (!ts) {
		return count;
	}

	if (!ts->tp_data_record_support) {
		return count;
	}

	if (count > 2) {
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 2);

	if (kstrtoint(buf, 10, &value)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	debug_info_ops = (struct debug_info_proc_operations *)(ts->debug_info_ops);
	if (!debug_info_ops) {
		TS_TP_INFO("debug_info_ops == NULL");
		return 0;
	}
	if (!debug_info_ops->tp_data_record_write) {
		TS_TP_INFO("debug_info_ops->tp_data_record_write == NULL");
		return 0;
	}

	TPD_DETAIL("%s tp_limit_data write :%d\n", __func__, value);

	if (ts->int_mode == BANNABLE) {
		disable_irq_nosync(ts->irq);
	}
	mutex_lock(&ts->mutex);

	if (debug_info_ops->tp_data_record_write) {
		debug_info_ops->tp_data_record_write(ts->chip_data, value);
	}
	if (ts->health_monitor_support) {
		if (value) {
			tp_healthinfo_report(&ts->monitor_data, HEALTH_REPORT, "data_record_on");
		} else {
			tp_healthinfo_report(&ts->monitor_data, HEALTH_REPORT, "data_record_off");
		}
	}
	mutex_unlock(&ts->mutex);
	if (ts->int_mode == BANNABLE) {
		enable_irq(ts->irq);
	}

	return count;
}

static int tp_data_record_read_func(struct seq_file *s, void *v)
{
	struct touchpanel_data *ts = s->private;

	if (!ts) {
		return 0;
	}
	return 0;
}

static int data_record_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_data_record_read_func, PDE_DATA(inode));
}

DECLARE_PROC_OPS(tp_data_record_proc_fops, data_record_open, seq_read, tp_data_record_write_func, single_release);

/*proc/touchpanel/debug_info/abs_doze*/
static int tp_abs_doze_read_func(struct seq_file *s, void *v)
{
	struct touchpanel_data *ts = s->private;
	struct debug_info_proc_operations *debug_info_ops;

	if (!ts) {
		return 0;
	}

	if(true != ts->bus_ready) {
		TS_TP_INFO("tp bus not ready, returnd");
		return 0;
	}

	debug_info_ops = (struct debug_info_proc_operations *)ts->debug_info_ops;

	if (!debug_info_ops) {
		return 0;
	}

	if (!debug_info_ops->abs_doze_read) {
		seq_printf(s, "Not support main_register proc node\n");
		return 0;
	}

	if (tp_status(ts) == SLEEP_MODE) {
		seq_printf(s, "Not in resume over state\n");
		return 0;
	}

	if (ts->int_mode == BANNABLE) {
		disable_irq_nosync(ts->irq);
	}

	mutex_lock(&ts->mutex);
	if (ts->int_mode == UNBANNABLE && !tp_irq_check(ts)) {
		seq_printf(s, "Irq not available\n");
		mutex_unlock(&ts->mutex);
		return 0;
	}
	debug_info_ops->abs_doze_read(s, ts->chip_data);
	mutex_unlock(&ts->mutex);

	if (ts->int_mode == BANNABLE) {
		enable_irq(ts->irq);
	}

	return 0;
}

static int abs_doze_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_abs_doze_read_func, PDE_DATA(inode));
}

DECLARE_PROC_OPS(tp_abs_doze_proc_fops, abs_doze_open, seq_read, NULL, single_release);

/*proc/touchpanel/debug_info/ic_reset*/
static ssize_t tp_inject_wdt_reset_write_func(struct file *file,
				    const char __user *buffer, size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[5] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		TPD_INFO("%s: ts is NULL\n", __func__);
		return count;
	}

	if (!ts->ts_ops || !ts->ts_ops->inject_wdt_reset || !ts->chip_data) {
		TS_TP_INFO("%s: ts->ts_ops or ts->ts_ops->inject_wdt_reset or ts->chip_data is NULL\n", __func__);
		return count;
	}

	if (ts->is_suspended) {
		TS_TP_INFO("%s: is_suspended, exit\n", __func__);
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 4);

	if (kstrtoint(buf, 10, &value)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	TP_INFO(ts->tp_index, "%s: inject watchdog reset value=%d\n", __func__, value);

	if (value) {
		mutex_lock(&ts->mutex);
		ts->ts_ops->inject_wdt_reset(ts->chip_data, value);
		mutex_unlock(&ts->mutex);
	}

	return count;
}

static int tp_inject_wdt_reset_read_func(struct seq_file *s, void *v)
{
	struct touchpanel_data *ts = s->private;

	if (!ts) {
		return 0;
	}
	return 0;
}

static int inject_wdt_reset_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_inject_wdt_reset_read_func, PDE_DATA(inode));
}

DECLARE_PROC_OPS(tp_inject_wdt_reset_proc_fops, inject_wdt_reset_open, seq_read, tp_inject_wdt_reset_write_func, single_release);

/*proc/touchpanel/debug_info/snr*/
static ssize_t proc_snr_write(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	int tmp = 0, i = 0;
	char buffer[4] = {0};

	if (!ts) {
		return count;
	}

	if (!ts->snr_read_support) {
		return count;
	}

	if (count > 2) {
		return count;
	}

	if (copy_from_user(buffer, buf, count)) {
		TS_TP_INFO("%s: read proc input error.\n", __func__);
		return count;
	}

	if (1 == sscanf(buffer, "%d", &tmp)) {
		for (i = 0; i < 10; i++) {
			ts->snr[i].doing = !!tmp;
		}
	} else {
		TPD_DEBUG("invalid content: '%s', length = %zd\n", buf, count);
	}

	TS_TP_INFO("%s: snr write doing = %d.\n", __func__, ts->snr[0].doing);
	return count;
}

#define SNR_NODE_READ_TIMES 100
static int tp_baseline_snr_read_func(struct seq_file *s, void *v)
{
	struct touchpanel_data *ts = s->private;
	struct debug_info_proc_operations *debug_info_ops = NULL;

	if (!ts)
		return 0;

	if (!ts->snr_read_support) {
		return 0;
	}

	if(true != ts->bus_ready) {
		TS_TP_INFO("tp bus not ready, returnd");
		return 0;
	}

	debug_info_ops = (struct debug_info_proc_operations *)(ts->debug_info_ops);
	if (!debug_info_ops) {
		TS_TP_INFO("debug_info_ops == NULL");
		return 0;
	}
	if (!debug_info_ops->delta_snr_read) {
		seq_printf(s, "Not support baseline snr proc node\n");
		return 0;
	}
	if (tp_status(ts) == SLEEP_MODE) {
		seq_printf(s, "Not in resume over or gesture state\n");
		return 0;
	}

	if (s->size <= (PAGE_SIZE * 2)) {
		s->count = s->size;
		return 0;
	}

	if (ts->int_mode == BANNABLE) {
		disable_irq_nosync(ts->irq);
	}
	mutex_lock(&ts->mutex);
	if (debug_info_ops->delta_snr_read) {
		debug_info_ops->delta_snr_read(s, ts->chip_data, SNR_NODE_READ_TIMES);
	}

	mutex_unlock(&ts->mutex);
	if (ts->int_mode == BANNABLE) {
		enable_irq(ts->irq);
	}
	return 0;
}

static int proc_snr_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_baseline_snr_read_func, PDE_DATA(inode));
}

DECLARE_PROC_OPS(proc_snr_ops, proc_snr_open, seq_read, proc_snr_write, NULL);

void tp_freq_hop_work(struct work_struct *work)
{
	struct touchpanel_data *ts = container_of(work, struct touchpanel_data,
				     freq_hop_info.freq_hop_work.work);

	if (!ts->is_suspended) {
		TP_INFO(ts->tp_index, "trigger frequency hopping~~~~\n");

		if (!ts->ts_ops->freq_hop_trigger) {
			TP_INFO(ts->tp_index, "%s:not support ts_ops->freq_hop_trigger callback\n",
				__func__);
			return;
		}

		ts->ts_ops->freq_hop_trigger(ts->chip_data);
	}

	if (ts->freq_hop_info.freq_hop_simulating) {
		TP_INFO(ts->tp_index, "queue_delayed_work again\n");
		queue_delayed_work(ts->freq_hop_info.freq_hop_workqueue,
				   &ts->freq_hop_info.freq_hop_work, ts->freq_hop_info.freq_hop_freq * HZ);
	}
}

static void tp_freq_hop_simulate(struct touchpanel_data *ts, int freq_hop_freq)
{
	TP_INFO(ts->tp_index, "%s is called.\n", __func__);
	ts->freq_hop_info.freq_hop_freq = freq_hop_freq;

	if (ts->freq_hop_info.freq_hop_simulating && !freq_hop_freq) {
		ts->freq_hop_info.freq_hop_simulating = false;

	} else {
		if (!ts->freq_hop_info.freq_hop_simulating) {
			ts->freq_hop_info.freq_hop_simulating = true;
			queue_delayed_work(ts->freq_hop_info.freq_hop_workqueue,
					   &ts->freq_hop_info.freq_hop_work, ts->freq_hop_info.freq_hop_freq * HZ);
		}
	}
}

/*proc/touchpanel/debug_info/freq_hop_simulate*/
static ssize_t proc_freq_hop_write(struct file *file, const char __user *buffer,
				   size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[5] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		TPD_INFO("%s: ts is NULL\n", __func__);
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 4);

	if (kstrtoint(buf, 16, &value)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	ts->freq_hop_info.freq_hop_freq = value;

	TP_INFO(ts->tp_index, "%s: freq_hop_simulate value=0x%x\n", __func__, value);

	if (!ts->is_suspended) {
		mutex_lock(&ts->mutex);
		tp_freq_hop_simulate(ts, value);
		mutex_unlock(&ts->mutex);

	} else {
		TP_INFO(ts->tp_index, "%s: freq_hop_simulate is_suspended.\n", __func__);
		ts->freq_hop_info.freq_hop_freq = value;
	}

	return count;
}

static ssize_t proc_freq_hop_read(struct file *file, char __user *buffer,
				  size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		snprintf(page, PAGESIZE - 1, "%d\n", -1); /*no support*/

	} else {
		/*support*/
		snprintf(page, PAGESIZE - 1, "%d\n", ts->freq_hop_info.freq_hop_freq);
	}

	ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));
	return ret;
}

DECLARE_PROC_OPS(proc_freq_hop_fops, simple_open, proc_freq_hop_read, proc_freq_hop_write, NULL);


/*proc/touchpanel/debug_info/force_water_mode*/
static ssize_t proc_force_water_mode_write(struct file *file, const char __user *buffer,
				   size_t count, loff_t *ppos)
{
	int value = 0;
	char buf[5] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		TPD_INFO("%s: ts is NULL\n", __func__);
		return count;
	}

	if (!ts->ts_ops) {
		TS_TP_INFO("%s: ts->ts_ops is NULL\n", __func__);
		return count;
	}

	if (!ts->ts_ops->force_water_mode) {
		TS_TP_INFO("%s: ts->ts_ops->force_water_mode is NULL\n", __func__);
		return count;
	}

	if (ts->is_suspended) {
		TS_TP_INFO("%s: is_suspended, exit\n", __func__);
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 4);

	if (kstrtoint(buf, 10, &value)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	ts->in_force_water_mode = !!value;

	TP_INFO(ts->tp_index, "%s: in_force_water_mode value=%d\n", __func__, value);

	ts->ts_ops->force_water_mode(ts->chip_data, ts->in_force_water_mode);

	return count;
}

static ssize_t proc_force_water_mode_read(struct file *file, char __user *buffer,
				  size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		snprintf(page, PAGESIZE - 1, "%d\n", -1); /*no support*/

	} else {
		/*support*/
		if(ts->ts_ops->get_water_mode != NULL) {
			ts->ts_ops->get_water_mode(ts->chip_data);
			TP_INFO(ts->tp_index, "%s:  water_mode 0x%x", __func__, ts->water_mode);
			snprintf(page, PAGESIZE - 1, "%d\n", ts->water_mode);
		}
	}

	ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));
	return ret;
}

DECLARE_PROC_OPS(proc_force_water_mode_fops, simple_open,
		   proc_force_water_mode_read, proc_force_water_mode_write, NULL);

static ssize_t proc_pocket_prevent_mode_write(struct file *file, const char __user *buffer,
				  size_t count, loff_t *ppos)
{
	int ret = 0;
	int value = 0;
	char buf[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		TPD_INFO("%s: ts is NULL\n", __func__);
		return count;
	}

	if (!ts->ts_ops->mode_switch) {
		TS_TP_INFO("not support ts_ops->mode_switch callback\n");
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 4);

	if (kstrtoint(buf, 10, &value)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	ts->pocket_prevent_mode = !!value;

	TP_INFO(ts->tp_index, "%s: pocket_prevent_mode value=%d\n", __func__, value);

	mutex_lock(&ts->mutex);
	ret = ts->ts_ops->mode_switch(ts->chip_data, MODE_GLOVE, (ts->glove_enable)&&(!ts->pocket_prevent_mode));
	if (ret < 0) {
		TS_TP_INFO("%s, Touchpanel operate mode switch failed\n", __func__);
	}
	mutex_unlock(&ts->mutex);

	return count;
}

static ssize_t proc_pocket_prevent_mode_read(struct file *file, char __user *buffer,
				  size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		snprintf(page, PAGESIZE - 1, "%d\n", -1); /*no support*/
		TPD_INFO("ts is null.\n");
		ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));
		return ret;
	} else {
		/*support*/
		snprintf(page, PAGESIZE - 1, "%d\n", ts->pocket_prevent_mode);
		ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));
		return ret;
	}
}

DECLARE_PROC_OPS(proc_pocket_prevent_mode, simple_open,
		  proc_pocket_prevent_mode_read, proc_pocket_prevent_mode_write, NULL);

/*proc/touchpanel/glove_mode_enable*/
static ssize_t proc_glove_mode_write(struct file *file, const char __user *buffer,
				  size_t count, loff_t *ppos)
{
	int ret = 0;
	int value = 0;
	char buf[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		TPD_INFO("%s: ts is NULL\n", __func__);
		return count;
	}

	if (!ts->ts_ops->mode_switch) {
		TS_TP_INFO("not support ts_ops->mode_switch callback\n");
		return count;
	}

	if (ts->is_suspended) {
		TS_TP_INFO("%s: is_suspended, exit\n", __func__);
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 4);

	if (kstrtoint(buf, 10, &value)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	ts->glove_enable = !!value;

	TP_INFO(ts->tp_index, "%s: glove_enable value=%d\n", __func__, value);

	mutex_lock(&ts->mutex);
	ret = ts->ts_ops->mode_switch(ts->chip_data, MODE_GLOVE, (ts->glove_enable) && (!ts->pocket_prevent_mode));
	if (ret < 0) {
		TS_TP_INFO("%s, Touchpanel operate mode switch failed\n", __func__);
	}
	mutex_unlock(&ts->mutex);

	return count;
}

static ssize_t proc_glove_mode_read(struct file *file, char __user *buffer,
				 size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		snprintf(page, PAGESIZE - 1, "%d\n", -1); /*no support*/
		TPD_INFO("glove mode enable is no support \n");
		ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));
		return ret;
	} else {
		/*support*/
		snprintf(page, PAGESIZE - 1, "%d:%lld\n", ts->glove_enable, ts->monitor_data.glove_enter_count);
		ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));
		return ret;
	}
}

DECLARE_PROC_OPS(proc_glove_mode, simple_open,
		  proc_glove_mode_read, proc_glove_mode_write, NULL);

/*proc/touchpanel/rainstorm_mode_enable*/
static ssize_t proc_rainstorm_mode_write(struct file *file, const char __user *buffer,
				  size_t count, loff_t *ppos)
{
	int ret = 0;
	int value = 0;
	size_t copy_len = 0;
	char buf[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts || !ts->ts_ops) {
		TPD_INFO("%s: ts is NULL\n", __func__);
		return count;
	}

	if (!ts->ts_ops->mode_switch) {
		TS_TP_INFO("not support ts_ops->mode_switch callback\n");
		return count;
	}

	copy_len = min(count, (size_t)3);
	if (tp_copy_from_user(buf, sizeof(buf), buffer, copy_len, copy_len)) {
        TS_TP_INFO("%s: read proc input error.\n", __func__);
		return count;
	}
	buf[copy_len] = '\0';

	if (kstrtoint(buf, 10, &value)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	mutex_lock(&ts->mutex);

	ts->rainstorm_enable = !!value;
	TP_INFO(ts->tp_index, "%s: rainstorm_enable value=%d\n", __func__, value);

	if (ts->is_suspended) {
		TS_TP_INFO("%s: is_suspended, exit\n", __func__);
		mutex_unlock(&ts->mutex);
		return count;
	}

	ret = ts->ts_ops->mode_switch(ts->chip_data, MODE_RAINSTORM, ts->rainstorm_enable);
	if (ret < 0) {
		TS_TP_INFO("%s, Touchpanel operate rainstorm mode switch failed\n", __func__);
	}
	mutex_unlock(&ts->mutex);

	return count;
}

static ssize_t proc_rainstorm_mode_read(struct file *file, char __user *buffer,
				 size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		snprintf(page, PAGESIZE - 1, "%d\n", -1); /*no support*/
		TPD_INFO("rainstorm mode enable is no support \n");
		ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));
		return ret;
	} else {
		/*support*/
		mutex_lock(&ts->mutex);
		snprintf(page, PAGESIZE - 1, "%d\n", ts->rainstorm_enable);
		ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));
		mutex_unlock(&ts->mutex);
		return ret;
	}
}

DECLARE_PROC_OPS(proc_rainstorm_mode, simple_open,
		  proc_rainstorm_mode_read, proc_rainstorm_mode_write, NULL);

/*proc/touchpanel/leather_cover_enable*/
static ssize_t proc_leather_cover_enable_write(struct file *file, const char __user *buffer,
				  size_t count, loff_t *ppos)
{
	int ret = 0;
	int value = 0;
	char buf[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		TPD_INFO("%s: ts is NULL\n", __func__);
		return count;
	}

	if (!ts->ts_ops->mode_switch) {
		TS_TP_INFO("not support ts_ops->mode_switch callback\n");
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 4);

	if (kstrtoint(buf, 10, &value)) {
		TP_INFO(ts->tp_index, "%s: kstrtoint error\n", __func__);
		return count;
	}

	ts->leather_cover_enable = !!value;

	TP_INFO(ts->tp_index, "%s: leather_cover_enable value=%d\n", __func__, value);

	mutex_lock(&ts->mutex);
	ret = ts->ts_ops->mode_switch(ts->chip_data, MODE_LEATHER_COVER, ts->leather_cover_enable);
	if (ret < 0) {
		TS_TP_INFO("%s, Touchpanel operate mode switch failed\n", __func__);
	}
	mutex_unlock(&ts->mutex);

	return count;
}

static ssize_t proc_leather_cover_enable_read(struct file *file, char __user *buffer,
				 size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		snprintf(page, PAGESIZE - 1, "%d\n", -1); /*no support*/
		TPD_INFO("leather_cover enable is no support \n");
		ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));
		return ret;
	} else {
		/*support*/
		snprintf(page, PAGESIZE - 1, "%d\n", ts->leather_cover_enable);
		TS_TP_INFO("leather_cover enable is: %d\n", ts->leather_cover_enable);
		ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));
		return ret;
	}
}

DECLARE_PROC_OPS(leather_cover_enable, simple_open,
		  proc_leather_cover_enable_read, proc_leather_cover_enable_write, NULL);

static ssize_t proc_probe_status_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *ppos)
{
	int tmp = 0;
	char buf[4] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		TPD_INFO("%s: ts is NULL\n", __func__);
		return count;
	}
	tp_copy_from_user(buf, sizeof(buf), buffer, count, 2);

	if (kstrtoint(buf, 10, &tmp)) {
		TPD_INFO("%s: kstrtoint error\n", __func__);
		return count;
	}
	ts->is_update_log = !!tmp;
	return count;
}

static ssize_t proc_probe_status_read(struct file *file, char __user *buffer,
							size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	if (!ts) {
		snprintf(page, PAGESIZE - 1, "%d", 0); /*no support*/
	} else {
		/*support*/
		snprintf(page, PAGESIZE - 1, "%d", ts->is_update_log);
	}
	ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));
	return ret;
}
DECLARE_PROC_OPS(proc_probe_status_fops, simple_open, proc_probe_status_read, proc_probe_status_write, NULL);

#ifndef CONFIG_REMOVE_OPLUS_FUNCTION
/*proc/touchpanel/debug_info/health_monitor*/
static int tp_health_monitor_read_func(struct seq_file *s, void *v)
{
	struct touchpanel_data *ts = s->private;
	struct monitor_data *monitor_data = &ts->monitor_data;

	mutex_lock(&ts->mutex);

	if (monitor_data->fw_version) {
		memset(monitor_data->fw_version, 0, MAX_DEVICE_VERSION_LENGTH);
		strncpy(monitor_data->fw_version, ts->panel_data.manufacture_info.version,
			strlen(ts->panel_data.manufacture_info.version));
	}

	tp_healthinfo_read(s, monitor_data);

	mutex_unlock(&ts->mutex);
	return 0;
}

static ssize_t health_monitor_control(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	struct monitor_data *monitor_data = &ts->monitor_data;
	char buffer[4] = {0};
	int tmp = 0;

	mutex_lock(&ts->mutex);
	if (count > 2) {
		goto EXIT;
	}
	if (copy_from_user(buffer, buf, count)) {
		TPD_INFO("%s: read proc input error.\n", __func__);
		goto EXIT;
	}

	if (1 == sscanf(buffer, "%d", &tmp) && tmp == 0) {
		tp_healthinfo_clear(monitor_data);
	} else {
		TPD_INFO("invalid operation\n");
	}

EXIT:
	mutex_unlock(&ts->mutex);
	return count;
}

static int health_monitor_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_health_monitor_read_func, PDE_DATA(inode));
}

DECLARE_PROC_OPS(tp_health_monitor_proc_fops, health_monitor_open, seq_read, health_monitor_control, single_release);

#if IS_ENABLED(CONFIG_TOUCHPANEL_FAULT_INJECT_ENABLE)
/*proc/touchpanel/debug_info/health_simulate_trigger*/
static ssize_t proc_health_simulate_trigger_read(struct file *file, char __user *buffer,
				     size_t count, loff_t *ppos)
{
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	uint8_t ret = 0;
	char page[PAGESIZE] = {0};

	if (!ts || !ts->health_monitor_support) {
		return count;
	}

	TS_TP_INFO("%s: health_simulate_trigger = %d.\n", __func__, ts->monitor_data.health_simulate_trigger);
	snprintf(page, PAGESIZE - 1, "%d", ts->monitor_data.health_simulate_trigger);
	ret = simple_read_from_buffer(buffer, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_health_simulate_trigger_write(struct file *file,
				      const char __user *buffer, size_t count, loff_t *ppos)
{
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));

	int tmp = 0;
	char buf[4] = {0};

	if (!ts || !ts->health_monitor_support) {
		return count;
	}

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 4);

	if (kstrtoint(buf, 10, &tmp)) {
		TS_TP_INFO("%s: kstrtoint error\n", __func__);
		return count;
	}

	ts->monitor_data.health_simulate_trigger = tmp;

	return count;
}

DECLARE_PROC_OPS(proc_health_simulate_trigger_ops, simple_open,
		   proc_health_simulate_trigger_read, proc_health_simulate_trigger_write, NULL);
#endif
#endif

/*******Part5:Register node Function  Area********************/

typedef struct {
	char *name;
	umode_t mode;
	struct proc_dir_entry *node;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,10,0)
	const struct proc_ops *fops;
#else
	const struct file_operations *fops;
#endif
	void *data;
	bool is_created;/*proc node is creater or not*/
	bool is_support;/*feature is supported or not*/
} tp_proc_node;

int init_probe_status_proc(struct touchpanel_data *ts)
{
	int ret = 0;
	int i = 0;
	char name[TP_NAME_SIZE_MAX] = {0};
	tp_proc_node tp_proc_node[] = {
		{"probe_status", 0666, NULL, &proc_probe_status_fops, ts, false, true},
	};

	if (ts == NULL) {
		ret = -ENOMEM;
		return 0;
	}

	if (ts->tp_index == 0) {
		snprintf(name, TP_NAME_SIZE_MAX, "%s", TPD_DEVICE);

	} else {
		snprintf(name, TP_NAME_SIZE_MAX, "%s%d", TPD_DEVICE, ts->tp_index);
	}
	name[TP_NAME_SIZE_MAX - 1] = '\0';

	ts->prEntry_tp = proc_mkdir(name, NULL);

	if (ts->prEntry_tp == NULL) {
		ret = -ENOMEM;
		TP_INFO(ts->tp_index, "%s: Couldn't create TP proc entry\n", __func__);
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(tp_proc_node); i++) {
		if (tp_proc_node[i].is_support) {
			tp_proc_node[i].node = proc_create_data(tp_proc_node[i].name,
								tp_proc_node[i].mode,
								ts->prEntry_tp, tp_proc_node[i].fops, tp_proc_node[i].data);

			if (tp_proc_node[i].node == NULL) {
				tp_proc_node[i].is_created = false;
				TP_INFO(ts->tp_index, "%s: Couldn't create proc/touchpanel/%s\n", __func__,
					tp_proc_node[i].name);
				ret = -ENODEV;

			} else {
				tp_proc_node[i].is_created = true;
			}
		}
	}

	return ret;
}

/*proc/touchpanel/debug_info*/
static int init_debug_info_proc(struct touchpanel_data *ts)
{
	int ret = 0;
	int i = 0;
	struct proc_dir_entry *prEntry_debug_info = NULL;

	tp_proc_node proc_debug_node[] = {
		{"data_record", 0666, NULL, &tp_data_record_proc_fops, ts, false, ts->tp_data_record_support},/* show data record interface*/
		{"data_limit", 0666, NULL, &tp_limit_data_proc_fops, ts, false, true},/* show limit data interface*/
		{"baseline", 0666, NULL, &tp_baseline_data_proc_fops, ts, false, true},/* show baseline data interface*/
		{"pen_baseline", 0666, NULL, &tp_pen_baseline_data_proc_fops, ts, false, ts->pen_support},/* show pen baseline data interface*/
		{"delta", 0666, NULL, &tp_delta_data_proc_fops, ts, false, true},/* show delta interface*/
		{"pen_delta", 0666, NULL, &tp_pen_delta_data_proc_fops, ts, false, ts->pen_support},/* show pen delta interface*/
		{"self_delta", 0666, NULL, &tp_self_delta_data_proc_fops, ts, false, true},/* show self delta interface*/
		{"self_raw", 0666, NULL, &tp_self_raw_data_proc_fops, ts, false, true},/* show self_raw interface*/
		{"main_register", 0666, NULL, &tp_main_register_proc_fops, ts, false, true},/* show main_register interface*/
		{"reserve", 0666, NULL, &tp_reserve_proc_fops, ts, false, true},/* show reserve interface*/
		{"abs_doze", 0666, NULL, &tp_abs_doze_proc_fops, ts, false, true},/* show abs_doze interface*/
		{
			"snr", 0666, NULL, &proc_snr_ops, ts, false,
			ts->snr_read_support
		},/* show abs_doze interface*/
		{"ic_reset", 0666, NULL, &tp_inject_wdt_reset_proc_fops, ts, false, true},/* inject watchdog reset*/
	};

	TP_INFO(ts->tp_index, "%s entry\n", __func__);

	/*proc/touchpanel/debug_info*/
	prEntry_debug_info = proc_mkdir("debug_info", ts->prEntry_tp);

	if (prEntry_debug_info == NULL) {
		TP_INFO(ts->tp_index, "%s: Couldn't create debug_info proc entry\n", __func__);
		return -ENOMEM;
	}

	ts->prEntry_debug_tp = prEntry_debug_info;

	for (i = 0; i < ARRAY_SIZE(proc_debug_node); i++) {
		if (proc_debug_node[i].is_support) {
			proc_debug_node[i].node = proc_create_data(proc_debug_node[i].name,
						  proc_debug_node[i].mode,
						  prEntry_debug_info, proc_debug_node[i].fops, proc_debug_node[i].data);

			if (proc_debug_node[i].node == NULL) {
				proc_debug_node[i].is_created = false;
				TP_INFO(ts->tp_index, "%s: Couldn't create proc/debug_info/%s\n", __func__,
					proc_debug_node[i].name);
				ret = -ENODEV;

			} else {
				proc_debug_node[i].is_created = true;
			}
		}
	}

	return ret;
}
static int init_debug_info_proc_part2(struct touchpanel_data *ts, struct proc_dir_entry *prEntry_debug_tp)
{
	int ret = 0;
	int i = 0;

	tp_proc_node proc_debug_node_part2[] = {
#ifndef CONFIG_REMOVE_OPLUS_FUNCTION
		{
			"health_monitor", 0666, NULL, &tp_health_monitor_proc_fops, ts, false,
			ts->health_monitor_support
		},
#if IS_ENABLED(CONFIG_TOUCHPANEL_FAULT_INJECT_ENABLE)
		{
			"health_simulate_trigger", 0666, NULL, &proc_health_simulate_trigger_ops, ts, false,
			ts->health_monitor_support
		},
#endif
#endif
		{
			"freq_hop_simulate_support", 0666, NULL, &proc_freq_hop_fops, ts, false,
			ts->freq_hop_simulate_support
		},
		{
			"force_water_mode", 0666, NULL, &proc_force_water_mode_fops, ts, false,
			ts->ts_ops->force_water_mode
		},
		{"hardware_control", 0666, NULL, &proc_hardware_control_fops, ts, false, true},
		{"report_rate_test", 0666, NULL, &proc_report_rate_test_fops, ts, false, true},
	};

	if (prEntry_debug_tp == NULL) {
		TP_INFO(ts->tp_index, "%s: Couldn't create debug_info proc entry\n", __func__);
		return -ENOMEM;
	}

	for (i = 0; i < ARRAY_SIZE(proc_debug_node_part2); i++) {
		if (proc_debug_node_part2[i].is_support) {
			proc_debug_node_part2[i].node = proc_create_data(proc_debug_node_part2[i].name,
						  proc_debug_node_part2[i].mode,
						  prEntry_debug_tp, proc_debug_node_part2[i].fops, proc_debug_node_part2[i].data);

			if (proc_debug_node_part2[i].node == NULL) {
				proc_debug_node_part2[i].is_created = false;
				TP_INFO(ts->tp_index, "%s: Couldn't create proc/debug_info/%s\n", __func__,
					proc_debug_node_part2[i].name);
				ret = -ENODEV;

			} else {
				proc_debug_node_part2[i].is_created = true;
			}
		}
	}

	return ret;
}

/**
 * init_touchpanel_proc - Using for create proc interface
 * @ts: touchpanel_data struct using for common driver
 *
 * we need to set touchpanel_data struct as private_data to those file_inode
 * Returning zero(success) or negative errno(failed)
 */
int init_touchpanel_proc_part2(struct touchpanel_data *ts, struct proc_dir_entry *prEntry_tp)
{
	int ret = 0;
	int i = 0;
	tp_proc_node tp_proc_node_part2[] = {
		{
			"report_rate_white_list", 0666, NULL, &proc_rate_white_list_fops, ts, false,
			ts->report_rate_white_list_support
		},
		{
			"charge_detect", 0666, NULL, &proc_switch_usb_state_fops, ts, false,
			ts->charger_pump_support
		},
		{
			"disable_touch_event", 0666, NULL, &proc_disable_touch_event_fops, ts, false,
			ts->disable_touch_event_support
		},
		{
			"wireless_charge_detect", 0666, NULL, &proc_wireless_charge_detect_fops, ts, false,
			ts->wireless_charger_support
		},
		{
			"headset_detect", 0666, NULL, &proc_headset_detect_fops, ts, false,
			ts->headset_pump_support
		},
		{
			"tp_aging_test", 0666, NULL, &proc_aging_test_ops, ts, false,
			true
		},
		{
			"smooth_level", 0666, NULL, &proc_smooth_level_fops, ts, false,
			ts->smooth_level_array_support
		},
		{
			"sensitive_level", 0666, NULL, &proc_sensitive_level_fops, ts, false,
			ts->sensitive_level_array_support
		},
		{
			"diaphragm_touch", 0666, NULL, &proc_diaphragm_touch_level_fops, ts, false,
			ts->diaphragm_touch_support
		},
		{
			"force_water_mode", 0666, NULL, &proc_force_water_mode_fops, ts, false,
			ts->ts_ops->force_water_mode
		},
		{
			"double_tap_enable_indep", 0666, NULL, &proc_gesture_control_indep_fops, ts, false,
			ts->black_gesture_indep_support
		},
		{
			"calibration", 0666, NULL, &proc_calibrate_fops, ts, false,
			ts->auto_test_need_cal_support
		},
		{
			"calibration_status", 0666, NULL, &proc_cal_status_fops, ts, false,
			ts->auto_test_need_cal_support
		},
		{
			"touch_scene_default_para", 0666, NULL, &touch_scene_default_para_fops, ts, false,
			ts->tp_scene_para_switch_support
		},
		{
			"touch_scene_para", 0666, NULL, &proc_touch_scene_para_fops, ts, false,
			ts->tp_scene_para_switch_support
		},
		/* proc/touchpanel/oplus_apk. Add the new test node for debug and apk. By zhangping 20190402 start*/
#ifdef CONFIG_OPLUS_TP_APK
		{"oplus_apk", 0666, NULL, &proc_oplus_apk_fops, ts, false, true},
#endif /* end of CONFIG_OPLUS_TP_APK*/
	};

	for (i = 0; i < ARRAY_SIZE(tp_proc_node_part2); i++) {
		if (tp_proc_node_part2[i].is_support) {
			tp_proc_node_part2[i].node = proc_create_data(tp_proc_node_part2[i].name,
								tp_proc_node_part2[i].mode,
								prEntry_tp, tp_proc_node_part2[i].fops, tp_proc_node_part2[i].data);

			if (tp_proc_node_part2[i].node == NULL) {
				tp_proc_node_part2[i].is_created = false;
				TP_INFO(ts->tp_index, "%s: Couldn't create proc/debug_info/%s\n", __func__,
					tp_proc_node_part2[i].name);
				ret = -ENODEV;

			} else {
				tp_proc_node_part2[i].is_created = true;
			}
		}
	}

	return ret;
}

int init_touchpanel_proc_part3(struct touchpanel_data *ts, struct proc_dir_entry *prEntry_tp)
{
	int ret = 0;
	int i = 0;

	tp_proc_node tp_proc_node_part3[] = {
		/* proc/touchpanel/oplus_apk. Add the new test node for debug and apk. By zhangping 20190402 start*/
#ifdef CONFIG_OPLUS_TP_APK
		{"oplus_apk", 0666, NULL, &proc_oplus_apk_fops, ts, false, true},
#endif /* end of CONFIG_OPLUS_TP_APK*/
		{
			"pencil_connected", 0666, NULL, &proc_pencil_connect_fops, ts, false,
			ts->pen_support
		},
		{
			"pencil_control", 0666, NULL, &proc_pencil_control_fops, ts, false,
			ts->pen_support
		},
		{"pencil_opp", 0666, NULL, &proc_pencil_opp_fops, ts, false, ts->pen_support},
		{"gesture_support", 0666, NULL, &tp_gesture_support_fops, ts, false, true},
		{"palm_to_sleep", 0666, NULL, &tp_palm_to_sleep_fops, ts, false, ts->palm_to_sleep_support},
		{"palm_to_sleep_support", 0666, NULL, &tp_palm_to_sleep_support_fops, ts, false, ts->palm_to_sleep_support},
		{"waterproof", 0666, NULL, &tp_waterproof_fops, ts, false, ts->waterproof_support},
		{
			"glove_mode_enable", 0666, NULL, &proc_glove_mode, ts, false,
			ts->glove_mode_v2_support
		},
		{"set_idle_freq_mode", 0666, NULL, &proc_set_idle_freq_mode_ops, ts, false, ts->idle_freq_support},
		{
			"rainstorm_mode", 0666, NULL, &proc_rainstorm_mode, ts, false, ts->rainstorm_mode_v2_support
		},
		{
			"pocket_prevent_mode", 0666, NULL, &proc_pocket_prevent_mode, ts, false, ts->glove_mode_v2_support
		},
		{
			"leather_cover_enable", 0666, NULL, &leather_cover_enable, ts, false,
			ts->glove_mode_support
		},
		{
			"leather_cover_enable", 0666, NULL, &leather_cover_enable, ts, false,
			ts->leather_cover_mode_support
		},
		{
			"fw_edge_limit_switch", 0666, NULL, &touch_edge_limit_switch_fops, ts, false,
			ts->fw_edge_limit_support
		},
		{
			"click_sensitive_level", 0666, NULL, &proc_click_sensitive_level_fops, ts, false,
			ts->click_sensitive_level_array_support
		},
		{
			"report_rate", 0666, NULL, &touch_report_rate_fops, ts, false,
			ts->report_rate_v2_support
		},
		{"fp_grip_enable", 0666, NULL, &fp_grip_support_ops, ts, false, ts->fp_grip_support},
		{"fp_unlock_status", 0666, NULL, &fp_unlock_status_ops, ts, false, ts->fp_unlock_status_support},
	};


	for (i = 0; i < ARRAY_SIZE(tp_proc_node_part3); i++) {
		if (tp_proc_node_part3[i].is_support) {
			tp_proc_node_part3[i].node = proc_create_data(tp_proc_node_part3[i].name,
								tp_proc_node_part3[i].mode,
								prEntry_tp, tp_proc_node_part3[i].fops, tp_proc_node_part3[i].data);

			if (tp_proc_node_part3[i].node == NULL) {
				tp_proc_node_part3[i].is_created = false;
				TP_INFO(ts->tp_index, "%s: Couldn't create proc/debug_info/%s\n", __func__,
					tp_proc_node_part3[i].name);
				ret = -ENODEV;

			} else {
				tp_proc_node_part3[i].is_created = true;
			}
		}
	}
	return ret;
}

int init_touchpanel_proc(struct touchpanel_data *ts)
{
	int ret = 0;
	int i = 0;
	struct proc_dir_entry *prEntry_tmp = NULL;
	char name[TP_NAME_SIZE_MAX];

	tp_proc_node tp_proc_node[] = {
		{
			"oplus_optimized_time", 0666, NULL, &proc_optimized_time_fops, ts, false,
			ts->optimized_show_support
		},
		{"tp_index", 0666, NULL, &proc_tp_index_ops, ts, false, true},
		{"debug_level", 0644, NULL, &proc_debug_level_ops, ts, false, true},
		{
			"double_tap_enable", 0666, NULL, &proc_gesture_control_fops, ts, false,
			ts->black_gesture_support
		},
		{
			"coordinate", 0444, NULL, &proc_coordinate_fops, ts, false,
			ts->black_gesture_support
		},
		{
			"game_switch_enable", 0666, NULL, &proc_game_switch_fops, ts, false,
			ts->game_switch_support
		},
		{"irq_depth", 0666, NULL, &proc_get_irq_depth_fops, ts, false, true},
		{
			"oplus_tp_noise_modetest", 0664, NULL, &proc_noise_modetest_fops, ts, false,
			ts->noise_modetest_support
		},
		{"tp_fw_update", 0666, NULL, &proc_fw_update_ops, ts, false, true},
		{"oplus_register_info", 0664, NULL, &proc_register_info_fops, ts, false, true},
		{
			"incell_panel", 0664, NULL, &proc_incell_panel_fops, ts, false,
			ts->is_incell_panel
		},
		{
			"fd_enable", 0666, NULL, &tp_fd_enable_fops, ts, false,
			ts->face_detect_support
		},
		{
			"event_num", 0666, NULL, &tp_event_num_fops, ts, false,
			ts->face_detect_support
		},
		{
			"fd_touch_count", 0666, NULL, &fd_touch_num_fops, ts, false,
			ts->face_detect_support
		},
		{
			"fp_enable", 0666, NULL, &tp_fp_enable_fops, ts, false,
			ts->fingerprint_underscreen_support
		},
		{
			"baseline_test", 0666, NULL, &tp_auto_test_proc_fops, ts, false, true
		},
		{
			"black_screen_test", 0666, NULL, &proc_black_screen_test_fops, ts, false,
			ts->gesture_test_support
		},
		{
			"baseline_result", 0666, NULL, &tp_auto_test_result_fops, ts, false, true
		},
		{
			"black_screen_result", 0666, NULL, &proc_black_screen_result_fops, ts, false,
			ts->gesture_test_support
		},
		{
			"oplus_tp_direction", 0666, NULL, &touch_dir_proc_fops, ts, false,
			ts->fw_edge_limit_support
		},
		{"communicate_test", 0666, NULL, &tp_communicate_test_ops, ts, false, true},
		{"aiunit_game_info", 0666, NULL, &proc_aiunit_game_info_ops, ts, false,
			ts->aiunit_game_info_support
		},
	};

	TP_INFO(ts->tp_index, "%s entry\n", __func__);

	/*proc files-step1:/proc/devinfo/tp  (touchpanel device info)*/
#ifndef CONFIG_REMOVE_OPLUS_FUNCTION

	if (ts->tp_index == 0) {
		snprintf(name, TP_NAME_SIZE_MAX, "%s", "tp");

	} else {
		snprintf(name, TP_NAME_SIZE_MAX, "%s%d", "tp", ts->tp_index);
	}

	if (ts->fw_update_app_support) {
		register_devinfo(name, &ts->panel_data.manufacture_info);

	} else {
		TP_INFO(ts->tp_index, "register_devinfo not defined!\n");
		ret = register_device_proc(name, ts->panel_data.manufacture_info.version,
				     ts->panel_data.manufacture_info.manufacture);
		if (ret) {
			pr_err("register_manufacture_devinfo fail\n");
		}
	}

#endif

	/*proc files-step2:/proc/touchpanel*/
	if (ts->prEntry_tp == NULL) {
		ret = -ENOMEM;
		TP_INFO(ts->tp_index, "%s: Couldn't create TP proc entry\n", __func__);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(tp_proc_node); i++) {
		if (tp_proc_node[i].is_support) {
			tp_proc_node[i].node = proc_create_data(tp_proc_node[i].name,
								tp_proc_node[i].mode,
								ts->prEntry_tp, tp_proc_node[i].fops, tp_proc_node[i].data);

			if (tp_proc_node[i].node == NULL) {
				tp_proc_node[i].is_created = false;
				TP_INFO(ts->tp_index, "%s: Couldn't create proc/debug_info/%s\n", __func__,
					tp_proc_node[i].name);
				ret = -ENODEV;

			} else {
				tp_proc_node[i].is_created = true;
			}
		}
	}

	init_touchpanel_proc_part3(ts, ts->prEntry_tp);
	init_touchpanel_proc_part2(ts, ts->prEntry_tp);

	/*create debug_info node*/
	init_debug_info_proc(ts);
	init_debug_info_proc_part2(ts, ts->prEntry_debug_tp);

	/*create kernel grip proc file*/
	if (ts->kernel_grip_support) {
		init_kernel_grip_proc(ts->prEntry_tp, ts->grip_info);
		prEntry_tmp = proc_create_data("kernel_grip_default_para",
					       0664,
					       ts->prEntry_tp,
					       &tp_grip_default_para_fops,
					       ts);
		if (prEntry_tmp == NULL) {
			TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
		}
	}

	/*int aoto test data*/
	ts->com_test_data.raw_cap_restriction = 100;

	return ret;
}

void remove_touchpanel_proc(struct touchpanel_data *ts)
{
	char name[TP_NAME_SIZE_MAX];

	if (!ts) {
		TPD_INFO("%s: ts is null.\n", __func__);
		return;
	}
	if (ts->tp_index == 0) {
		snprintf(name, TP_NAME_SIZE_MAX, "%s", TPD_DEVICE);

	} else {
		snprintf(name, TP_NAME_SIZE_MAX, "%s%d", TPD_DEVICE, ts->tp_index);
	}

	if (ts->prEntry_tp) {
		remove_proc_subtree(name, NULL);
	}
}
