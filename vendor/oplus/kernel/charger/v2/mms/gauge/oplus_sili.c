// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2024 Oplus. All rights reserved.
 */


#define pr_fmt(fmt) "[OPLUS_SILI]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/of_platform.h>
#include <linux/iio/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/list.h>
#include <linux/power_supply.h>
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/system/boot_mode.h>
#include <soc/oplus/device_info.h>
#include <soc/oplus/system/oplus_project.h>
#endif
#include <oplus_chg_module.h>
#include <oplus_chg_ic.h>
#include <oplus_chg_comm.h>
#include <oplus_chg_voter.h>
#include <oplus_mms.h>
#include <oplus_chg_monitor.h>
#include <oplus_mms_wired.h>
#include <oplus_mms_gauge.h>
#include <oplus_chg_vooc.h>
#include <oplus_batt_bal.h>
#include <oplus_parallel.h>
#include <oplus_chg_wls.h>
#include <linux/ktime.h>
#include <linux/sched/clock.h>
#include "oplus_gauge_common.h"

#ifndef CONFIG_OPLUS_CHARGER_MTK
#include <linux/soc/qcom/smem.h>
#endif

static int oplus_gauge_get_last_cc(struct oplus_mms *mms);

#define GAUGE_PARALLEL_IC_NUM_MIN 2
static bool is_support_parallel(struct oplus_mms_gauge *chip)
{
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return false;
	}

	if (chip->child_num >= GAUGE_PARALLEL_IC_NUM_MIN)
		return true;
	else
		return false;
}

static int oplus_mms_gauge_push_vbat_uv(struct oplus_mms_gauge *chip)
{
	struct mms_msg *msg;
	int rc;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, GAUGE_ITEM_VBAT_UV);
	if (msg == NULL) {
		chg_err("alloc vbat uv msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->gauge_topic, msg);
	if (rc < 0) {
		chg_err("publish vbat uv msg error, rc=%d\n", rc);
		kfree(msg);
	}
	chg_info(" [%d, %d]\n", chip->deep_spec.config.uv_thr, chip->deep_spec.config.count_thr);

	return rc;
}

#define GAUGE_INVALID_DEEP_COUNT_CALI	10
#define GAUGE_INVALID_DEEP_DICHG_COUNT	10
int oplus_gauge_show_deep_dischg_count(struct oplus_mms *topic)
{
	struct oplus_mms_gauge *chip;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return 0;
	}
	chip = oplus_mms_get_drvdata(topic);

	if (!chip  || !chip->deep_spec.support)
		return GAUGE_INVALID_DEEP_DICHG_COUNT;

	if (is_support_parallel(chip))
		return chip->deep_spec.counts > chip->deep_spec.sub_counts ? chip->deep_spec.counts : chip->deep_spec.sub_counts;

	return chip->deep_spec.counts;
}

static int oplus_gauge_get_deep_dischg_count(struct oplus_mms_gauge *chip, struct oplus_chg_ic_dev *ic)
{
	int rc, temp = GAUGE_INVALID_DEEP_DICHG_COUNT;

	if (!chip  || !chip->deep_spec.support || !ic)
		return GAUGE_INVALID_DEEP_DICHG_COUNT;

	rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_DEEP_DISCHG_COUNT, &temp);
	if (rc < 0) {
		if (rc != -ENOTSUPP)
			chg_err(" get batt deep dischg count error, rc=%d\n", rc);
		return GAUGE_INVALID_DEEP_DICHG_COUNT;
	}

	return temp;
}

#define OPLUS_TERM_VOLT_1_2_DELTA_MAX_MV    100
#define OPLUS_TERM_VOLT_1_3_DELTA_MAX_MV    300
#define OPLUS_TERM_VOLT_1_2_DEFAULT_DELTA_MV  25
#define OPLUS_TERM_VOLT_1_3_DEFAULT_DELTA_MV  150

static int oplus_mms_gauge_set_three_level_term_volt(struct oplus_mms_gauge *chip,
			struct oplus_chg_ic_dev *ic, int volt_mv)
{
	int func_rc = -ENOTSUPP;
	unsigned char data[32] = {0};
	struct gauge_three_level_term_volt_cfg *volt_cfg = NULL;
	int delta_level_1_2_mv = 0;
	int delta_level_1_3_mv = 0;

	/* update the param based on the DTS config.*/
	volt_cfg = &chip->three_level_term_volt_cfg;
	if (volt_cfg->term_volt_size == 0)
		return -ENOTSUPP;
	func_rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_THREE_LEVEL_TERM_VOLT,
				    data, OPLUS_GAUGE_THREE_LEVEL_TERM_VOLT_LEN);
	if (func_rc != 0)
		return func_rc;

	chg_info("get term_vol:[%x,%x,%x,%x,%x,%x]\n",
		 data[0], data[1], data[2], data[3], data[4], data[5]);

	/*
	 * The 18 bytes three leve term volt param:
	 * byte[0]: the low 8 bit of Term Voltage
	 * byte[1]: the high 8 bit of Term voltage
	 * byte[2]: the low 8 bit of Term Voltage_2
	 * byte[3]: the high 8 bit of Term voltage_2
	 * Byte[4]: the low 8 bit of Term Voltage_3
	 * byte[5]: the high 8 bit of Term voltage_3
	 * byte[6]: the hold time of Term Voltage
	 * byte[7]: the hold time of Term voltage_2
	 * byte[8]: the hold time of Term Voltage_3
	 * byte[9]: the time_to_drop_per1%
	 * byte[10]: the time_to_drop_per1%_2
	 * byte[11]: the time_to_drop_per1%_3
	 * byte[12]: the recover low 8 bit of Term Voltage
	 * byte[13]: the recover high 8 bit of Term voltage
	 * byte[14]: the recover low 8 bit of Term Voltage_2
	 * byte[15]: the recover high 8 bit of Term voltage_2
	 * byte[16]: the recover hold time of Term Voltage
	 * byte[17]: the recover hold time of Term voltage_2
	 */

	/* term_volt_1 */
	data[0] = (volt_mv & 0xff);
	data[1] = ((volt_mv & 0xff00) >> 8);

	/* term_volt_2*/
	delta_level_1_2_mv = (volt_cfg->term_volt - volt_cfg->term_volt_2);
	if (delta_level_1_2_mv > OPLUS_TERM_VOLT_1_2_DELTA_MAX_MV)
		delta_level_1_2_mv = OPLUS_TERM_VOLT_1_2_DELTA_MAX_MV;
	if (delta_level_1_2_mv == 0)
		delta_level_1_2_mv = OPLUS_TERM_VOLT_1_2_DEFAULT_DELTA_MV;

	if (delta_level_1_2_mv > 0) {
		data[2] = ((volt_mv - delta_level_1_2_mv) & 0xff);
		data[3] = (((volt_mv - delta_level_1_2_mv) & 0xff00) >> 8);
	}

	delta_level_1_3_mv = (volt_cfg->term_volt - volt_cfg->term_volt_3);
	if (delta_level_1_3_mv > OPLUS_TERM_VOLT_1_3_DELTA_MAX_MV)
		delta_level_1_3_mv = OPLUS_TERM_VOLT_1_3_DELTA_MAX_MV;
	if (delta_level_1_3_mv == 0)
		delta_level_1_3_mv = OPLUS_TERM_VOLT_1_3_DEFAULT_DELTA_MV;

	/* term_volt_3*/
	if (delta_level_1_3_mv > 0) {
		data[4] = ((volt_mv - delta_level_1_3_mv) & 0xff);
		data[5] = (((volt_mv - delta_level_1_3_mv) & 0xff00) >> 8);
	}
	func_rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_SET_THREE_LEVEL_TERM_VOLT,
						data, OPLUS_GAUGE_THREE_LEVEL_TERM_VOLT_LEN);
	chg_info("set term_vol:[%x,%x,%x,%x,%x,%x], delta=[%d,%d], func_rc=%d\n",
			  data[0], data[1], data[2], data[3], data[4], data[5],
			  delta_level_1_2_mv, delta_level_1_3_mv, func_rc);

	return func_rc;
}

static int oplus_mms_gauge_get_three_level_term_volt(struct oplus_mms_gauge *chip,
			struct oplus_chg_ic_dev *ic, int *volt_mv)
{
	int func_rc = -ENOTSUPP;
	unsigned char data[32] = {0};

	func_rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_THREE_LEVEL_TERM_VOLT,
					data, OPLUS_GAUGE_THREE_LEVEL_TERM_VOLT_LEN);
	if (func_rc != 0)
		return func_rc;

	*volt_mv = ((data[1] << 8) + data[0]);
	chg_info("get term_vol:[%x,%x,%x,%x,%x,%x], reg_term_volt[%d]\n",
		data[0], data[1], data[2], data[3], data[4], data[5], *volt_mv);
	return func_rc;
}

#define OPLUS_TERM_VOLT_RETRY_MAX    3
static int oplus_mms_gauge_update_three_level_term_volt(struct oplus_mms_gauge *chip,
			struct oplus_chg_ic_dev *ic, int volt_mv)
{
	int func_rc = -ENOTSUPP;
	struct gauge_three_level_term_volt_cfg *volt_cfg = NULL;
	int reg_term_volt = 0;
	int retry_count = 0;

	/* update the param based on the DTS config.*/
	volt_cfg = &chip->three_level_term_volt_cfg;
	if (volt_cfg->term_volt_size == 0)
		return -ENOTSUPP;

	do {
		func_rc = oplus_mms_gauge_set_three_level_term_volt(chip, ic, volt_mv);
		if (func_rc == -ENOTSUPP) {
			break;
		} else if (func_rc == 0) {
			/* read back to check if the term volt is updated success */
			func_rc = oplus_mms_gauge_get_three_level_term_volt(chip, ic, &reg_term_volt);
			if ((func_rc == 0) && (volt_mv == reg_term_volt)) {
				chg_info("update the three level term volt success!\n");
				break;
			} else {
				chg_info("volt_mv[%d], reg_term_volt[%d], retry_count[%d] failed!\n",
					  volt_mv, reg_term_volt, retry_count);
				msleep(10);
				continue;
			}
		} else {
			msleep(10);
		}
		retry_count++;
	} while (retry_count < OPLUS_TERM_VOLT_RETRY_MAX);

	if ((func_rc == 0) && (volt_mv == reg_term_volt))
		return func_rc;
	else {
		chg_info("update the three level term volt failed!\n");
		func_rc = -EFAULT;
	}

	return func_rc;
}

int oplus_mms_gauge_set_deep_term_volt(struct oplus_mms *mms, int volt_mv)
{
	int rc = 0;
	int i;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (!chip || !chip->deep_spec.support)
		return -EINVAL;

	if (mms == chip->gauge_topic) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_SET_DEEP_TERM_VOLT, volt_mv);
			if (rc < 0) {
				chg_err("gauge[%d](%s): can't set gauge deep term volt, rc=%d\n", i, ic->manu_name, rc);
				continue;
			}
			rc = oplus_mms_gauge_update_three_level_term_volt(chip, ic, volt_mv);
			if (rc < 0 && (rc != -ENOTSUPP))
				chg_err("gauge[%d](%s): can't update gauge term volt, rc=%d\n",
					i, ic->manu_name, rc);
			else if (rc == -ENOTSUPP)
				rc = 0;
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (mms != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_SET_DEEP_TERM_VOLT, volt_mv);
			if (rc < 0)
				chg_err("gauge[%d](%s): can't set gauge deep term volt, rc=%d\n", i, ic->manu_name, rc);
			rc = oplus_mms_gauge_update_three_level_term_volt(chip, ic, volt_mv);
			if (rc < 0 && (rc != -ENOTSUPP))
				chg_err("gauge[%d](%s): can't update three level gauge term volt, rc=%d\n",
					i, ic->manu_name, rc);
			else if (rc == -ENOTSUPP)
				rc = 0;

			return rc;
		}
	}

	return rc;
}

static int oplus_gauge_get_deep_term_volt(struct oplus_mms_gauge *chip)
{
	int rc = 0;
	int volt_mv = -EINVAL;

	if (!chip || !chip->deep_spec.support)
		return volt_mv;

	rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_DEEP_TERM_VOLT, &volt_mv);
	if (rc < 0)
		chg_err("get batt deep term volt error, rc=%d, volt_mv=%d\n", rc, volt_mv);
	return volt_mv;
}

static void oplus_gauge_update_three_level_volt_data(struct oplus_mms_gauge *chip,
	int volt_mv, unsigned char *data)
{
	struct gauge_three_level_term_volt_cfg *volt_cfg = NULL;
	int delta_level_1_2_mv = 0;
	int delta_level_1_3_mv = 0;

	volt_cfg = &chip->three_level_term_volt_cfg;
	/* term_volt_1 */
	data[0] = (volt_mv & 0xff);
	data[1] = ((volt_mv & 0xff00) >> 8);

	/* term_volt_2*/
	delta_level_1_2_mv = (volt_cfg->term_volt - volt_cfg->term_volt_2);
	if (delta_level_1_2_mv > OPLUS_TERM_VOLT_1_2_DELTA_MAX_MV)
	    delta_level_1_2_mv = OPLUS_TERM_VOLT_1_2_DELTA_MAX_MV;
	if (delta_level_1_2_mv == 0)
	    delta_level_1_2_mv = OPLUS_TERM_VOLT_1_2_DEFAULT_DELTA_MV;

	if (delta_level_1_2_mv > 0) {
	    data[2] = ((volt_mv - delta_level_1_2_mv) & 0xff);
	    data[3] = (((volt_mv - delta_level_1_2_mv) & 0xff00) >> 8);
	}

	delta_level_1_3_mv = (volt_cfg->term_volt - volt_cfg->term_volt_3);
	if (delta_level_1_3_mv > OPLUS_TERM_VOLT_1_3_DELTA_MAX_MV)
	    delta_level_1_3_mv = OPLUS_TERM_VOLT_1_3_DELTA_MAX_MV;
	if (delta_level_1_3_mv == 0)
	    delta_level_1_3_mv = OPLUS_TERM_VOLT_1_3_DEFAULT_DELTA_MV;

	/* term_volt_3*/
	if (delta_level_1_3_mv > 0) {
	    data[4] = ((volt_mv - delta_level_1_3_mv) & 0xff);
	    data[5] = (((volt_mv - delta_level_1_3_mv) & 0xff00) >> 8);
	}
}

static void oplus_gauge_update_three_level_ht_data(struct oplus_mms_gauge *chip,
	unsigned char *data)
{
	struct gauge_three_level_term_volt_cfg *volt_cfg = NULL;

	volt_cfg = &chip->three_level_term_volt_cfg;
	data[6] = (volt_cfg->hold_time > 0) ? volt_cfg->hold_time : 0;
	data[7] = (volt_cfg->hold_time_2 > 0) ? volt_cfg->hold_time_2 : 0;
	data[8] = (volt_cfg->hold_time_3 > 0) ? volt_cfg->hold_time_3 : 0;
	data[9] = (volt_cfg->time_to_drop_per1 > 0) ? volt_cfg->time_to_drop_per1 : 0;
	data[10] = (volt_cfg->time_to_drop_per1_2 > 0) ? volt_cfg->time_to_drop_per1_2 : 0;
	data[11] = (volt_cfg->time_to_drop_per1_3 > 0) ? volt_cfg->time_to_drop_per1_3 : 0;

	if (volt_cfg->recover_term_volt) {
		data[12] = volt_cfg->recover_term_volt & 0xff;
		data[13] = ((volt_cfg->recover_term_volt & 0xff00) >> 8);
	}
	if (volt_cfg->recover_term_volt_2) {
		data[14] = volt_cfg->recover_term_volt_2 & 0xff;
		data[15] = ((volt_cfg->recover_term_volt_2 & 0xff00) >> 8);
	}
	data[16] = (volt_cfg->recover_hold_time_of_term_voltage > 0) ?
			volt_cfg->recover_hold_time_of_term_voltage : 0;
	data[17] = (volt_cfg->recover_hold_time_of_term_voltage_2 > 0) ?
			volt_cfg->recover_hold_time_of_term_voltage_2 : 0;
}

#define VALID_TERM_VOLT 2000
static void oplus_gauge_update_three_level_ht_and_term_volt(struct oplus_mms_gauge *chip)
{
	int func_rc = -ENOTSUPP;
	unsigned char data[32] = {0};
	struct gauge_three_level_term_volt_cfg *volt_cfg = NULL;
	int current_volt = INVALID_MIN_VOLTAGE;
	int reg_term_volt = 0;

	if (!chip || !chip->deep_spec.support)
		return;

	func_rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_THREE_LEVEL_TERM_VOLT,
			data, OPLUS_GAUGE_THREE_LEVEL_TERM_VOLT_LEN);
	if (func_rc != 0)
		return;

	chg_info("term_vol:[%x,%x,%x,%x,%x,%x],holdtime[%x,%x,%x],time_to_drop[%x,%x,%x]," \
		 "recover_voltage[%x,%x,%x,%x],recover holdtime[%x,%x]\n",
		 data[0], data[1], data[2], data[3], data[4], data[5],
		 data[6], data[7], data[8],
		 data[9], data[10], data[11],
		 data[12], data[13], data[14], data[15],
		 data[16], data[17]);

	/* update the param based on the DTS config.*/
	volt_cfg = &chip->three_level_term_volt_cfg;

	/*
	 * The 18 bytes three leve term volt param:
	 * byte[0]: the low 8 bit of Term Voltage
	 * byte[1]: the high 8 bit of Term voltage
	 * byte[2]: the low 8 bit of Term Voltage_2
	 * byte[3]: the high 8 bit of Term voltage_2
	 * Byte[4]: the low 8 bit of Term Voltage_3
	 * byte[5]: the high 8 bit of Term voltage_3
	 * byte[6]: the hold time of Term Voltage
	 * byte[7]: the hold time of Term voltage_2
	 * byte[8]: the hold time of Term Voltage_3
	 * byte[9]: the time_to_drop_per1%
	 * byte[10]: the time_to_drop_per1%_2
	 * byte[11]: the time_to_drop_per1%_3
	 * byte[12]: the recover low 8 bit of Term Voltage
	 * byte[13]: the recover high 8 bit of Term voltage
	 * byte[14]: the recover low 8 bit of Term Voltage_2
	 * byte[15]: the recover high 8 bit of Term voltage_2
	 * byte[16]: the recover hold time of Term Voltage
	 * byte[17]: the recover hold time of Term voltage_2
	 */
	func_rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_DEEP_TERM_VOLT, &current_volt);
	if (func_rc < 0)
		chg_err("get batt deep term volt error, rc=%d\n", func_rc);

	reg_term_volt = (data[1] << 8) + data[0];
	chg_info("get term_volt = [%d, %d, %d], reg_term_volt[%d]\n", current_volt,
		 chip->deep_spec.config.term_voltage,
		 get_effective_result(chip->gauge_term_voltage_votable),
		 reg_term_volt);
	if ((volt_cfg->term_volt > current_volt) &&
	    (reg_term_volt != volt_cfg->term_volt)) {
		chg_info("set term_volt = %d\n", volt_cfg->term_volt);
		vote(chip->gauge_term_voltage_votable, READY_VOTER, true, volt_cfg->term_volt, false);
		return;
	}

	if (current_volt != reg_term_volt && current_volt > VALID_TERM_VOLT)
		oplus_gauge_update_three_level_volt_data(chip, current_volt, data);

	oplus_gauge_update_three_level_ht_data(chip, data);

	chg_info("update term_volt[%x,%x,%x,%x,%x,%x], holdtime[%x,%x,%x],time_to_drop[%x,%x,%x]," \
		 "recover_voltage[%x,%x,%x,%x],recover holdtime[%x,%x]\n",
		 data[0], data[1], data[2], data[3], data[4], data[5],
		 data[6], data[7], data[8],
		 data[9], data[10], data[11],
		 data[12], data[13], data[14], data[15], data[16], data[17]);

	if (chip->three_level_term_volt_cfg.term_volt) {
		func_rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_SET_THREE_LEVEL_TERM_VOLT,
					data, OPLUS_GAUGE_THREE_LEVEL_TERM_VOLT_LEN);
		chg_info(" func_rc = %d \n", func_rc);
	}
}

static int oplus_gauge_set_last_cc(struct oplus_mms *mms, int cc)
{
	int rc = 0;
	int i;
	struct oplus_chg_ic_dev *ic;
	struct oplus_mms_gauge *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (!chip || !chip->deep_spec.support)
		return -ENOTSUPP;

	if (mms == chip->gauge_topic) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_SET_LAST_CC, cc);
			if (rc < 0) {
				chg_err("gauge[%d](%s): can't set last cc, rc=%d\n", i, ic->manu_name, rc);
				continue;
			}
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (mms != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_SET_LAST_CC, cc);
			if (rc < 0)
				chg_err("gauge[%d](%s): can't set last cc, rc=%d\n", i, ic->manu_name, rc);
			return rc;
		}
	}

	return rc;
}

static int oplus_gauge_nvram_stress_test(struct oplus_mms_gauge *chip,
		struct oplus_mms *topic)
{
	int rc = 0;
	int last_cc = 0;
	int input_cc = 0;
	int temp = 0;

	if (NULL == chip || NULL == topic)
		return -EINVAL;

	chip->nvram_test.sum_cnt++;

	/* update to 0 when the test is completed! */
	if (chip->nvram_test.sum_cnt >= chip->nvram_test.input_count ||
	    chip->nvram_test.sum_cnt >= GAUGE_NVRAM_TEST_MAX_COUNT)
		last_cc = 0;
	else
		last_cc = chip->nvram_test.sum_cnt;

	/* +1 to update the last cc */
	input_cc = last_cc + 1;
	rc = oplus_gauge_set_last_cc(topic, input_cc);
	if (rc != 0) {
		chip->nvram_test.fail_cnt++;
		chg_err("set cc[%d] failed, sum_cnt[%d], fail_cnt[%d] \n",
			input_cc, chip->nvram_test.sum_cnt, chip->nvram_test.fail_cnt);
	} else {
		mdelay(10);
		temp = oplus_gauge_get_last_cc(topic);
		if (temp == input_cc) {
			chip->nvram_test.suc_cnt++;
			chg_info("sum_cnt[%d],suc_cnt[%d],write cc[%d] success\n",
				chip->nvram_test.sum_cnt, chip->nvram_test.suc_cnt, input_cc);
		} else {
			chip->nvram_test.fail_cnt++;
			chg_err("set cc %d failed, sum_cnt[%d], fail_cnt[%d]\n",
				input_cc, chip->nvram_test.sum_cnt,
				chip->nvram_test.fail_cnt);
		}
	}

	return rc;
}

static int oplus_gauge_term_volt_stress_test(struct oplus_mms_gauge *chip,
		struct oplus_mms *topic, int volt_mv)
{
	int rc = 0;
	int temp = 0;
	unsigned char data[32] = {0};

	chip->nvram_test.term_volt_sum_cnt++;
	rc = oplus_mms_gauge_set_deep_term_volt(topic, volt_mv);
	if (rc != 0) {
		chip->nvram_test.term_volt_fail_cnt++;
		chg_err("term_volt_test: sum_cnt[%d], fail_cnt[%d], write volt_mv[%d] failed, rc=%d.\n",
			chip->nvram_test.term_volt_sum_cnt,
			chip->nvram_test.term_volt_fail_cnt, volt_mv, rc);
	} else {
		oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_DEEP_TERM_VOLT, &temp);
		if (volt_mv != temp) {
			chip->nvram_test.term_volt_fail_cnt++;
			chg_err("term_volt_test: sum_cnt[%d], fail_cnt[%d], write volt_mv[%d] failed, rc=%d.\n",
				 chip->nvram_test.term_volt_sum_cnt,
				 chip->nvram_test.term_volt_fail_cnt, volt_mv, rc);
		} else {
			rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_THREE_LEVEL_TERM_VOLT,
					data, OPLUS_GAUGE_THREE_LEVEL_TERM_VOLT_LEN);
			if (rc == -ENOTSUPP) {
				chip->nvram_test.term_volt_suc_cnt++;
				chg_info("term_volt_test: sum_cnt[%d], fail_cnt[%d], write volt_mv[%d] success.\n",
					 chip->nvram_test.term_volt_sum_cnt,
					 chip->nvram_test.term_volt_fail_cnt, volt_mv);
				return 0;
			}

			temp = ((data[1] << 8) + data[0]);
			if ((rc != 0) || (volt_mv != temp)) {
				chip->nvram_test.term_volt_fail_cnt++;
				chg_err("term_volt_test: sum_cnt[%d], fail_cnt[%d], write volt_mv[%d] failed," \
						"read back term_volt = %d mV, rc = 0x%x.\n",
						chip->nvram_test.term_volt_sum_cnt,
						chip->nvram_test.term_volt_fail_cnt,
						volt_mv, temp, rc);
			} else {
				chip->nvram_test.term_volt_suc_cnt++;
				chg_info("term_volt_test: sum_cnt[%d], fail_cnt[%d], write volt_mv[%d] success.\n",
					chip->nvram_test.term_volt_sum_cnt,
					chip->nvram_test.term_volt_fail_cnt, volt_mv);
			}
		}
	}

	return rc;
}

#define OPLUS_GAUGE_TEST_START_VOLT_MV    2900
#define OPLUS_GAUGE_TEST_DELTA_VOLT_MV    300
void oplus_gauge_term_volt_stress_test_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_gauge *chip = container_of(dwork, struct oplus_mms_gauge,
				gauge_term_volt_stress_test_work);
	int volt_mv = OPLUS_GAUGE_TEST_START_VOLT_MV;

	/* every time, volt_mv add 1mV until add to 3200 */
	volt_mv += (chip->nvram_test.term_volt_sum_cnt % OPLUS_GAUGE_TEST_DELTA_VOLT_MV);
	chg_info("volt_mv[%d]mV, term_volt_sum_cnt[%d]\n",
		 volt_mv, chip->nvram_test.term_volt_sum_cnt);

	oplus_gauge_term_volt_stress_test(chip, chip->gauge_topic, volt_mv);
	if ((chip->nvram_test.term_volt_sum_cnt <= chip->nvram_test.input_count) &&
		(chip->nvram_test.input_state == BATTERY_START_TEST_TYPE)) {
		schedule_delayed_work(&chip->gauge_term_volt_stress_test_work,
			msecs_to_jiffies(chip->nvram_test.interval_ms));
	} else {
		chip->nvram_test.term_volt_test_state = BATTERY_STOP_TEST_TYPE;
		chg_info("term_volt test complet\n");
	}
}

void oplus_gauge_nvram_stress_test_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_gauge *chip = container_of(dwork, struct oplus_mms_gauge,
				gauge_nvram_stress_test_work);

	oplus_gauge_nvram_stress_test(chip, chip->gauge_topic);
	if ((chip->nvram_test.sum_cnt < chip->nvram_test.input_count) &&
		(chip->nvram_test.input_state == BATTERY_START_TEST_TYPE)) {
		chip->nvram_test.test_state = BATTERY_START_TEST_TYPE;
		schedule_delayed_work(&chip->gauge_nvram_stress_test_work,
			msecs_to_jiffies(chip->nvram_test.interval_ms));
	} else {
		chip->nvram_test.test_state = BATTERY_STOP_TEST_TYPE;
	}
}

static int __oplus_gauge_read_stress_test(struct oplus_chg_ic_dev *ic)
{
	int rc = 0;
	int result = 0;
	int temp = 0;

	rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_LAST_CC, &temp);
	if (rc < 0 && (rc != -ENOTSUPP)) {
		chg_err("gauge(%s): can't get last cc error, rc=%d\n", ic->manu_name, rc);
		result |= rc;
	}
	mdelay(10);

	rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_MAX, &temp);
	if (rc < 0 && (rc != -ENOTSUPP))
		chg_err("gauge(%s): can't get max voltage error, rc=%d\n", ic->manu_name, rc);
	mdelay(10);
	result |= rc;

	rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_CURR, &temp);
	if (rc < 0 && (rc != -ENOTSUPP))
		chg_err("gauge(%s): can't get curr error, rc=%d\n", ic->manu_name, rc);
	mdelay(10);
	result |= rc;

	rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_TEMP, &temp);
	if (rc < 0 && (rc != -ENOTSUPP))
		chg_err("gauge(%s): can't get temp error, rc=%d\n", ic->manu_name, rc);
	mdelay(10);
	result |= rc;

	rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC, &temp);
	if (rc < 0 && (rc != -ENOTSUPP))
		chg_err("gauge(%s): can't get soc error, rc=%d\n", ic->manu_name, rc);
	mdelay(10);
	result |= rc;

	rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_FCC, &temp);
	if (rc < 0 && (rc != -ENOTSUPP))
		chg_err("gauge(%s): can't get fcc error, rc=%d\n", ic->manu_name, rc);
	mdelay(10);
	result |= rc;

	rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_RM, &temp);
	if (rc < 0 && (rc != -ENOTSUPP))
		chg_err("gauge(%s): can't get rm error, rc=%d\n", ic->manu_name, rc);
	mdelay(10);
	result |= rc;

	rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_SOH, &temp);
	if (rc < 0 && (rc != -ENOTSUPP))
		chg_err("gauge(%s): can't get soh error, rc=%d\n", ic->manu_name, rc);
	mdelay(10);
	result |= rc;

	rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_QMAX, 0, &temp);
	if (rc < 0 && (rc != -ENOTSUPP))
		chg_err("gauge(%s): can't get QMAX error, rc=%d\n", ic->manu_name, rc);
	mdelay(10);
	result |= rc;

	rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_CC, &temp);
	if (rc < 0 && (rc != -ENOTSUPP))
		chg_err("gauge(%s): can't get BATT_CC error, rc=%d\n", ic->manu_name, rc);
	result |= rc;

	return result;
}

static int oplus_gauge_read_stress_test(struct oplus_mms_gauge *chip,
		struct oplus_mms *mms)
{
	int rc = 0;
	int i;
	struct oplus_chg_ic_dev *ic;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}

	if (!chip->deep_spec.support)
		return -ENOTSUPP;

	if (mms == chip->gauge_topic) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = __oplus_gauge_read_stress_test(ic);
			if (rc < 0) {
				chg_err("read failed, rc = %d \n", rc);
				break;
			}
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (mms != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = __oplus_gauge_read_stress_test(ic);
			if (rc < 0) {
				chg_err("read failed, rc = %d \n", rc);
				break;
			}
		}
	}

	return rc;
}

void oplus_gauge_read_stress_test_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_gauge *chip = container_of(dwork, struct oplus_mms_gauge,
				gauge_stress_read_test_work);
	int rc = 0;

	chip->nvram_test.read_sum_cnt++;
	rc = oplus_gauge_read_stress_test(chip, chip->gauge_topic);
	if ((rc != 0) && (rc != -ENOTSUPP))
		chip->nvram_test.read_fail_cnt++;
	else
		chip->nvram_test.read_suc_cnt =
			chip->nvram_test.read_sum_cnt - chip->nvram_test.read_fail_cnt;

	chg_info("read_test:rc=%d,sum_cnt[%d],fail_cnt[%d],suc_cnt[%d],input_count[%d],input_state[%d]\n",
			rc, chip->nvram_test.read_sum_cnt,
			chip->nvram_test.read_fail_cnt,
			chip->nvram_test.read_suc_cnt,
			chip->nvram_test.input_count,
			chip->nvram_test.input_state);
	if ((chip->nvram_test.read_sum_cnt < chip->nvram_test.input_count) &&
		(chip->nvram_test.input_state == BATTERY_START_TEST_TYPE)) {
		schedule_delayed_work(&chip->gauge_stress_read_test_work,
				msecs_to_jiffies(chip->nvram_test.interval_ms));
	} else {
		chg_info("read test complete! \n");
		chip->nvram_test.read_test_state = BATTERY_STOP_TEST_TYPE;
	}
}

int oplus_gauge_start_stress_read_test(struct oplus_mms *topic,
		int input_count, int interval_ms)
{
	struct oplus_mms_gauge *chip;

	if (NULL == topic) {
		chg_err("topic is NULL\n");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(topic);
	if (NULL == chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	chg_info("input_count = %d, interval_ms = %d \n",
		input_count, interval_ms);
	chip->nvram_test.interval_ms = interval_ms;
	chip->nvram_test.input_count = input_count;
	if (interval_ms <= 0)
		chip->nvram_test.interval_ms = GAUGE_NVRAM_TEST_DEFAULT_INTERVAL_MS;

	/* when the input count is no more than 0, stop the test. */
	if (input_count > GAUGE_NVRAM_TEST_MAX_COUNT) {
		chip->nvram_test.input_count = GAUGE_NVRAM_TEST_MAX_COUNT;
		chip->nvram_test.input_state = BATTERY_START_TEST_TYPE;
	} else if (input_count <= 0) {
		chip->nvram_test.input_state = BATTERY_STOP_TEST_TYPE;
		chip->nvram_test.input_count = 0;
	} else {
		chip->nvram_test.input_state = BATTERY_START_TEST_TYPE;
	}

	if (chip->nvram_test.input_state == BATTERY_START_TEST_TYPE) {
		chip->nvram_test.read_test_state = BATTERY_START_TEST_TYPE;
		schedule_delayed_work(&chip->gauge_stress_read_test_work,
				msecs_to_jiffies(chip->nvram_test.interval_ms));
	} else {
		cancel_delayed_work_sync(&chip->gauge_stress_read_test_work);
		chip->nvram_test.read_test_state = BATTERY_STOP_TEST_TYPE;
	}

	return 0;
}

int oplus_gauge_start_nvram_stress_test(struct oplus_mms *topic,
		int input_count, int interval_ms)
{
	struct oplus_mms_gauge *chip;

	if (NULL == topic) {
		chg_err("topic is NULL\n");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(topic);
	if (NULL == chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	chg_info("input_count = %d, interval_ms = %d ms \n",
		input_count, interval_ms);

	/* when the input count is no more than 0, stop the test. */
	if (input_count > GAUGE_NVRAM_TEST_MAX_COUNT) {
		chip->nvram_test.input_count = GAUGE_NVRAM_TEST_MAX_COUNT;
		chip->nvram_test.input_state = BATTERY_START_TEST_TYPE;
	} else if (input_count <= 0) {
		chip->nvram_test.input_state = BATTERY_STOP_TEST_TYPE;
		chip->nvram_test.input_count = 0;
	} else {
		chip->nvram_test.input_state = BATTERY_START_TEST_TYPE;
	}

	if (interval_ms <= 0)
		chip->nvram_test.interval_ms = GAUGE_NVRAM_TEST_DEFAULT_INTERVAL_MS;

	if (chip->nvram_test.input_state == BATTERY_START_TEST_TYPE) {
		schedule_delayed_work(&chip->gauge_nvram_stress_test_work,
			msecs_to_jiffies(chip->nvram_test.interval_ms));
	} else {
		chip->nvram_test.test_state = BATTERY_STOP_TEST_TYPE;
		cancel_delayed_work_sync(&chip->gauge_nvram_stress_test_work);
		chg_info("quit the nvram test and reset the last cc to default 1. \n");
		oplus_gauge_set_last_cc(topic, 1);
	}
	return 0;
}

int oplus_gauge_start_term_volt_stress_test(struct oplus_mms *topic,
		int input_count, int interval_ms)
{
	struct oplus_mms_gauge *chip;

	if (NULL == topic) {
		chg_err("topic is NULL\n");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(topic);
	if (NULL == chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	chg_info("input_count = %d, interval_ms = %d ms \n",
		input_count, interval_ms);

	if (chip->nvram_test.input_state == BATTERY_START_TEST_TYPE) {
		chip->nvram_test.term_volt_test_state = BATTERY_START_TEST_TYPE;
		schedule_delayed_work(&chip->gauge_term_volt_stress_test_work,
				msecs_to_jiffies(chip->nvram_test.interval_ms));
	} else {
		cancel_delayed_work_sync(&chip->gauge_term_volt_stress_test_work);
		chg_info("quit the nvram test and reset the term volt to default 2900 mV. \n");
		oplus_gauge_term_volt_stress_test(chip, chip->gauge_topic, 2900);
		chip->nvram_test.term_volt_test_state = BATTERY_STOP_TEST_TYPE;
	}
	return 0;
}

int oplus_gauge_get_nvram_stress_test(struct oplus_mms *topic,
		struct oplus_gauge_nvram_stress_test *data)
{
	struct oplus_mms_gauge *chip;

	if (NULL == topic) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(topic);
	if (chip)
		memcpy((u8 *)data, &chip->nvram_test, sizeof(struct oplus_gauge_nvram_stress_test));

	return 0;
}

int oplus_gauge_get_three_level_term_volt(struct oplus_mms *topic, int term_volt[])
{
	struct oplus_mms_gauge *chip;
	int rc = 0;
	char buf[OPLUS_GAUGE_THREE_LEVEL_TERM_VOLT_LEN] = { 0 };

	if (NULL == topic) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(topic);
	rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_THREE_LEVEL_TERM_VOLT,
			buf, OPLUS_GAUGE_THREE_LEVEL_TERM_VOLT_LEN);
	term_volt[0] = ((buf[1] << 8) + buf[0]);
	term_volt[1] = ((buf[3] << 8) + buf[2]);
	term_volt[2] = ((buf[5] << 8) + buf[4]);
	chg_info("three level term volt=[%d, %d, %d], rc = 0x%x.\n",
			term_volt[0], term_volt[1], term_volt[2], rc);

	return rc;
}

int oplus_gauge_set_cuv_state(struct oplus_mms *mms, int state)
{
	int rc = 0;
	int i;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (!chip->deep_spec.support)
		return -ENOTSUPP;

	if (mms != chip->gauge_topic) {
		chg_err("mms is not gauge_topic\n");
		return -EINVAL;
	}

	for (i = 0; i < chip->child_num; i++) {
		ic = chip->child_list[i].ic_dev;
		rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_SET_CUV_STATE, state);
		if (rc == -ENOTSUPP) {
			chg_debug("gauge[%d](%s): not support!\n", i, ic->manu_name);
			break;
		}

		if (rc < 0) {
			chg_err("gauge[%d](%s): can't set gauge cuv state, rc=%d\n", i, ic->manu_name, rc);
			continue;
		}
	}

	return rc;
}

int oplus_gauge_get_cuv_state(struct oplus_mms *mms, int *state)
{
	int rc = 0;
	struct oplus_mms_gauge *chip;

	if (mms == NULL) {
		chg_err("mms is NULL\n");
		return -EINVAL;
	}

	if (state == NULL) {
		chg_err("state is NULL\n");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (!chip) {
		chg_err("chip is NULL.\n");
		return -EINVAL;
	}

	if (!chip->deep_spec.support) {
		chg_debug("deep_spec.support is false.\n");
		return -ENOTSUPP;
	}

	if (mms != chip->gauge_topic) {
		chg_err("mms is not gauge_topic\n");
		return -EINVAL;
	}

	rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_CUV_STATE, state);
	if (rc < 0 && rc != -ENOTSUPP)
		chg_err("get batt cuv state error, rc=%d, *state=%d\n", rc, *state);

	return rc;
}

static int oplus_gauge_get_last_cc(struct oplus_mms *mms)
{
	int rc = 0;
	struct oplus_mms_gauge *chip;
	int cc = -EINVAL;
	int i;
	struct oplus_chg_ic_dev *ic;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (!chip  || !chip->deep_spec.support)
		return -EINVAL;

	if (mms == chip->gauge_topic) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_LAST_CC, &cc);
			if (rc < 0) {
				chg_err("gauge[%d](%s): can't get last cc error, rc=%d\n", i, ic->manu_name, rc);
				continue;
			}
			break;
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (mms != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_LAST_CC, &cc);
			if (rc < 0)
				chg_err("gauge[%d](%s): can't set last cc, rc=%d\n", i, ic->manu_name, rc);
			return cc;
		}
	}

	return cc;
}

int oplus_gauge_get_deep_count_cali(struct oplus_mms *topic)
{
	int rc = -GAUGE_INVALID_DEEP_COUNT_CALI;
	struct oplus_mms_gauge *chip;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return rc;
	}
	chip = oplus_mms_get_drvdata(topic);

	if (!chip  || !chip->deep_spec.support)
		return rc;

	return chip->deep_spec.config.count_cali;
}

#define GAUGE_LOW_ABNORMAL_TEMP (-200)
static int oplus_gauge_get_deep_dischg_temperature(struct oplus_mms_gauge *chip, int type)
{
	int gauge_temp = GAUGE_LOW_ABNORMAL_TEMP;
	union mms_msg_data data = { 0 };
	int rc;

	switch (type) {
	case STRATEGY_USE_BATT_TEMP:
		rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_TEMP,
					     &data, true);
		if (rc < 0) {
			chg_err("can't get battery temp, rc=%d\n", rc);
			return GAUGE_LOW_ABNORMAL_TEMP;
		}
		gauge_temp = data.intval;
		break;
	case STRATEGY_USE_SHELL_TEMP:
		rc = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_SHELL_TEMP,
					     &data, false);
		if (rc < 0) {
			chg_err("can't get shell temp, rc=%d\n", rc);
			return GAUGE_LOW_ABNORMAL_TEMP;
		}
		gauge_temp = data.intval;
		break;
	default:
		break;
		chg_err("not support temp type, type=%d\n", type);
	}
	return gauge_temp;
}

static void oplus_gauge_ddbc_temp_thr_init(struct oplus_mms_gauge *chip)
{
	int i;

	if (!chip)
		return;

	for (i = 0; i < DDB_CURVE_TEMP_WARM; i++) {
		chip->deep_spec.ddbc_tbatt.range[i] = chip->deep_spec.ddbc_tdefault.range[i];
	}
}

int oplus_gauge_ddbc_get_temp_region(struct oplus_mms_gauge *chip)
{
	int i, gauge_temp;
	int temp_region = DDB_CURVE_TEMP_WARM;

	gauge_temp = oplus_gauge_get_deep_dischg_temperature(chip, chip->deep_spec.ddbc_tbatt.temp_type);

	for (i = 0; i < DDB_CURVE_TEMP_WARM; i++) {
		if (gauge_temp < chip->deep_spec.ddbc_tbatt.range[i]) {
			temp_region = i;
			break;
		}
	}
	return temp_region;
}

static void oplus_gauge_ddrc_temp_thr_init(struct oplus_mms_gauge *chip)
{
	int i;
	int temp_range;

	if (!chip)
		return;

	if (chip->deep_spec.ddrc_strategy_v2)
		temp_range = chip->deep_spec.ddrc_temp_num;
	else
		temp_range = DDB_CURVE_TEMP_WARM;

	for (i = 0; i < temp_range; i++) {
		chip->deep_spec.ddrc_tbatt.range[i] = chip->deep_spec.ddrc_tdefault.range[i];
	}
}


int oplus_gauge_ddrc_get_temp_region(struct oplus_mms_gauge *chip)
{
	int i, gauge_temp;
	int temp_region, temp_range;

	if (chip->deep_spec.ddrc_strategy_v2)
		temp_range = chip->deep_spec.ddrc_temp_num;
	else
		temp_range = DDB_CURVE_TEMP_WARM;
	temp_region = temp_range;

	gauge_temp = oplus_gauge_get_deep_dischg_temperature(chip, chip->deep_spec.ddrc_tbatt.temp_type);

	for (i = 0; i < temp_range; i++) {
		if (gauge_temp < chip->deep_spec.ddrc_tbatt.range[i]) {
			temp_region = i;
			break;
		}
	}
	return temp_region;
}

static void oplus_gauge_ddrc_temp_thr_update(struct oplus_mms_gauge *chip,
				     int now, int pre)
{
	int temp_range;

	if (!chip)
		return;

	if (chip->deep_spec.ddrc_strategy_v2)
		temp_range = chip->deep_spec.ddrc_temp_num;
	else
		temp_range = DDB_CURVE_TEMP_WARM;

	if ((pre > now) && (now >= DDB_CURVE_TEMP_COLD) && (now < temp_range)) {
		chip->deep_spec.ddrc_tbatt.range[now] = chip->deep_spec.ddrc_tdefault.range[now] + BATT_TEMP_HYST;

		chg_info("now=%d, pre=%d, p[%d]update thr[%d] to %d\n", now, pre,
			chip->deep_spec.ddrc_tbatt.index_p, now, chip->deep_spec.ddrc_tbatt.range[now]);
	} else if ((pre < now) && (now >= DDB_CURVE_TEMP_COLD) && (now <= temp_range)) {
		chip->deep_spec.ddrc_tbatt.range[now - 1] =
			chip->deep_spec.ddrc_tdefault.range[now - 1] - BATT_TEMP_HYST;

		chg_info("now=%d, pre = %d, p[%d]update thr[%d] to %d\n", now, pre,
			chip->deep_spec.ddrc_tbatt.index_p, now - 1, chip->deep_spec.ddrc_tbatt.range[now - 1]);
	}
}

static void oplus_gauge_ddbc_temp_thr_update(struct oplus_mms_gauge *chip,
				     enum ddb_temp_region now, enum ddb_temp_region pre)
{
	if (!chip)
		return;

	if ((pre > now) && (now >= DDB_CURVE_TEMP_COLD) && (now < DDB_CURVE_TEMP_WARM)) {
		chip->deep_spec.ddbc_tbatt.range[now] =
			chip->deep_spec.ddbc_tdefault.range[now] + BATT_TEMP_HYST;

		chg_info("now=%d, pre=%d, update thr[%d] to %d\n",
			  now, pre, now, chip->deep_spec.ddbc_tbatt.range[now]);
	} else if ((pre < now) && (now >= DDB_CURVE_TEMP_COLD) && (now <= DDB_CURVE_TEMP_WARM)) {
		chip->deep_spec.ddbc_tbatt.range[now - 1] =
			chip->deep_spec.ddbc_tdefault.range[now - 1] - BATT_TEMP_HYST;

		chg_info("now=%d, pre=%d, update thr[%d] to %d\n",
			  now, pre, now - 1, chip->deep_spec.ddbc_tbatt.range[now - 1]);
	}
}

#define DEEP_RATIO_HYST	10
void oplus_gauge_get_ratio_value(struct oplus_mms *mms)
{
	union mms_msg_data data = { 0 };
	int *cc = 0, *ratio = 0, counts = 0;
	int rc = 0;
	int gauge_type;
	int soh_cc;
	struct oplus_mms_gauge *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (!chip || !chip->deep_spec.support)
		return;

	if (chip->sub_gauge && __ffs(chip->sub_gauge) < GAUGE_IC_NUM_MAX &&
	    mms == chip->gauge_topic_parallel[__ffs(chip->sub_gauge)]) {
		cc = &chip->deep_spec.sub_cc;
		ratio = &chip->deep_spec.sub_ratio;
		counts = chip->deep_spec.sub_counts;
	} else {
		cc = &chip->deep_spec.cc;
		ratio = &chip->deep_spec.ratio;
		counts = chip->deep_spec.counts;
	}

	rc = oplus_mms_get_item_data(mms, GAUGE_ITEM_CC, &data, true);
	if (rc < 0) {
		chg_err("can't get cc, rc=%d\n", rc);
		*cc = 0;
	} else {
		*cc = data.intval;
	}

	/* If it's platform gauge project, use soh from oplus_gauge_get_dec_cv_soh to replace cc */
	gauge_type = oplus_get_gauge_type();
	chg_info("gauge_type=%d\n", gauge_type);
	if (gauge_type == GAUGE_TYPE_PLATFORM) {
		soh_cc = oplus_gauge_get_dec_cv_soh(mms);
		chg_info("soh_cc=%d\n", soh_cc);
		if (soh_cc >= 0) {
			*cc = soh_cc;
		}
	}

	if (*cc <= 0 || *cc >= INVALID_CC_VALUE) {
		if (!counts)
			*ratio = 100;
		else
			*ratio = counts * 10;

	} else {
		*ratio = counts * 10 / *cc;
	}
}

void oplus_gauge_update_ddrc_data(struct oplus_mms *mms, int *index_cc, int *sub_index_cc)
{
	struct oplus_mms_gauge *chip;
	int *cc = 0, counts = 0, index_cc_main = 0, index_cc_sub = 0;
	int rc;
	struct ddrc_temp_curves *ddrc_curve;
	char *voter;
	int vterm_temp, vshut_temp;

	chip = oplus_mms_get_drvdata(mms);
	if (!chip || !chip->deep_spec.support || IS_ERR_OR_NULL(chip->ddrc_strategy))
		return;

	cc = &chip->deep_spec.cc;
	voter = DEEP_COUNT_VOTER;
	oplus_chg_strategy_init(chip->ddrc_strategy[0]);
	rc = oplus_chg_strategy_get_metadata(chip->ddrc_strategy[0], &chip->ddrc_curve);
	ddrc_curve = &chip->ddrc_curve;
	if (rc < 0 || !ddrc_curve->data || ddrc_curve->num < 1)
		return;

	if (*cc <= 0 || *cc >= INVALID_CC_VALUE) {
		index_cc_main = 0;
		chip->deep_spec.config.count_thr = 0;
	} else {
		for (index_cc_main = ddrc_curve->num - 1; index_cc_main > 0; index_cc_main--) {
			counts = ddrc_curve->data[index_cc_main].count < chip->deep_spec.config.count_cali ?
				0 : (ddrc_curve->data[index_cc_main].count - chip->deep_spec.config.count_cali);
			if (*cc >= counts) {
				chip->deep_spec.config.count_thr = counts;
				break;
			}
		}
	}
	vterm_temp = ddrc_curve->data[index_cc_main].vbat1;
	vshut_temp = ddrc_curve->data[index_cc_main].vbat0;
	vote(chip->target_term_voltage_votable, voter, true, vterm_temp, false);
	vote(chip->target_shutdown_voltage_votable, voter, true, vshut_temp, false);

	if (chip->sub_gauge && __ffs(chip->sub_gauge) < GAUGE_IC_NUM_MAX) {
		cc = &chip->deep_spec.sub_cc;
		voter = SUB_DEEP_COUNT_VOTER;
		if (chip->ddrc_num > __ffs(chip->sub_gauge)) {
			oplus_chg_strategy_init(chip->ddrc_strategy[__ffs(chip->sub_gauge)]);
			rc = oplus_chg_strategy_get_metadata(chip->ddrc_strategy[__ffs(chip->sub_gauge)],
				&chip->ddrc_curve_sub);
			ddrc_curve = &chip->ddrc_curve_sub;
		} else {
			chg_err("can't get ddrc_curve\n");
			return;
		}
		if (rc < 0 || !ddrc_curve->data || ddrc_curve->num < 1)
			return;
		if (*cc <= 0 || *cc >= INVALID_CC_VALUE) {
			index_cc_sub = 0;
			chip->deep_spec.config.count_thr = 0;
		} else {
			for (index_cc_sub = ddrc_curve->num - 1; index_cc_sub > 0; index_cc_sub--) {
				counts = ddrc_curve->data[index_cc_sub].count < chip->deep_spec.config.count_cali ?
					0 : (ddrc_curve->data[index_cc_sub].count - chip->deep_spec.config.count_cali);
				if (*cc >= counts) {
					chip->deep_spec.config.count_thr = counts;
					break;
				}
			}
		}
		vterm_temp = ddrc_curve->data[index_cc_sub].vbat1;
		vshut_temp = ddrc_curve->data[index_cc_sub].vbat0;
		vote(chip->target_term_voltage_votable, voter, true, vterm_temp, false);
		vote(chip->target_shutdown_voltage_votable, voter, true, vshut_temp, false);
	}

	*index_cc = index_cc_main;
	*sub_index_cc = index_cc_sub;
}

#define DEEP_DISCHG_UPDATE_CC_DELTA 5
void oplus_gauge_get_ddrc_status(struct oplus_mms *mms)
{
	struct oplus_mms_gauge *chip;
	int current_volt = 0, current_shut = 0, vterm = 0, vshut = 0;
	int update_vterm = 0, update_vshut = 0;
	int *cc = 0, index_cc = 0, sub_index_cc = 0, last_cc = 0, sub_last_cc = 0;
	struct deep_track_info *deep_info;
	struct ddrc_temp_curves *ddrc_curve;
	char *voter;
	bool *step_status;

	if (mms == NULL) {
		chg_err("mms is NULL\n");
		return;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (!chip || !chip->deep_spec.support || IS_ERR_OR_NULL(chip->ddrc_strategy))
		return;

	if (chip->sub_gauge) {
		oplus_gauge_get_ratio_value(chip->gauge_topic_parallel[chip->main_gauge]);
		oplus_gauge_get_ratio_value(chip->gauge_topic_parallel[__ffs(chip->sub_gauge)]);
	} else {
		oplus_gauge_get_ratio_value(mms);
	}

	cc = &chip->deep_spec.cc;
	deep_info = &chip->deep_info;
	voter = DEEP_COUNT_VOTER;
	step_status = &chip->deep_spec.config.step_status;
	ddrc_curve = &chip->ddrc_curve;

	oplus_gauge_update_ddrc_data(mms, &index_cc, &sub_index_cc);
	vterm = get_effective_result(chip->target_term_voltage_votable);
	vshut = get_effective_result(chip->target_shutdown_voltage_votable);
	current_volt = oplus_gauge_get_deep_term_volt(chip);
	current_shut = get_client_vote(chip->gauge_shutdown_voltage_votable, DEEP_COUNT_VOTER);
	if (current_shut <= 0)
		current_shut = vshut + current_volt - vterm;

	memset(deep_info, 0, sizeof(struct deep_track_info));
	deep_info->index += scnprintf(deep_info->msg, DEEP_INFO_LEN, "$$track_reason@@vote$$temp_p@@%d"
		"$$temp_n@@%d$$ratio_p@@%d$$ratio_n@@%d$$vstep@@%d$$vterm_final@@%d$$term_now@@%d$$index@@%d",
		chip->deep_spec.ddrc_tbatt.index_p, chip->deep_spec.ddrc_tbatt.index_n, chip->deep_spec.config.index_r,
		chip->ddrc_curve.index_r, chip->deep_spec.config.volt_step, vterm, current_volt, index_cc);

	if (current_volt != vterm) {
		if (chip->sub_gauge) {
			last_cc = oplus_gauge_get_last_cc(chip->gauge_topic_parallel[chip->main_gauge]);
			sub_last_cc = oplus_gauge_get_last_cc(chip->gauge_topic_parallel[__ffs(chip->sub_gauge)]);
		} else {
			last_cc = oplus_gauge_get_last_cc(mms);
		}
		if (last_cc <= 0 || *step_status || *cc < last_cc || vterm < current_volt ||
		    (chip->sub_gauge && (sub_last_cc <= 0 || chip->deep_spec.sub_cc < sub_last_cc))) {
			update_vterm = vterm;
			update_vshut = vshut;
			if (chip->sub_gauge) {
				oplus_gauge_set_last_cc(chip->gauge_topic_parallel[chip->main_gauge], *cc);
				oplus_gauge_set_last_cc(chip->gauge_topic_parallel[__ffs(chip->sub_gauge)], chip->deep_spec.sub_cc);
				vote(chip->gauge_term_voltage_votable, SUB_DEEP_COUNT_VOTER, true,
					chip->ddrc_curve_sub.data[sub_index_cc].vbat1, false);
				vote(chip->gauge_shutdown_voltage_votable, SUB_DEEP_COUNT_VOTER,
					true, chip->ddrc_curve_sub.data[sub_index_cc].vbat0, false);
			} else {
				oplus_gauge_set_last_cc(mms, *cc);
			}
			vote(chip->gauge_term_voltage_votable, voter, true,
				ddrc_curve->data[index_cc].vbat1, false);
			vote(chip->gauge_shutdown_voltage_votable, SUPER_ENDURANCE_MODE_VOTER,
				!chip->super_endurance_mode_status, chip->deep_spec.config.term_voltage, false);
			vote(chip->gauge_shutdown_voltage_votable, voter,
				true, ddrc_curve->data[index_cc].vbat0, false);
		} else if ((*cc >= last_cc + DEEP_DISCHG_UPDATE_CC_DELTA)) {
			if (vterm > current_volt)
				update_vterm = (vterm - current_volt) > chip->deep_spec.config.volt_step ?
				(current_volt + chip->deep_spec.config.volt_step) : vterm;
			else
				update_vterm = (current_volt - vterm) > chip->deep_spec.config.volt_step ?
				(current_volt - chip->deep_spec.config.volt_step) : vterm;


			if (vshut > current_shut)
				update_vshut = (vshut - current_shut) > chip->deep_spec.config.volt_step ?
				(current_shut + chip->deep_spec.config.volt_step) : vshut;
			else
				update_vshut = (current_shut - vshut) > chip->deep_spec.config.volt_step ?
				(current_shut - chip->deep_spec.config.volt_step) : vshut;

			if (update_vterm == vterm)
				update_vshut = vshut;
			if (chip->sub_gauge) {
				oplus_gauge_set_last_cc(chip->gauge_topic_parallel[chip->main_gauge], *cc);
				oplus_gauge_set_last_cc(chip->gauge_topic_parallel[__ffs(chip->sub_gauge)],
					chip->deep_spec.sub_cc);
				vote(chip->gauge_term_voltage_votable, SUB_DEEP_COUNT_VOTER, true, update_vterm, false);
				vote(chip->gauge_shutdown_voltage_votable, SUB_DEEP_COUNT_VOTER,
					true, update_vshut, false);
			} else {
				oplus_gauge_set_last_cc(mms, *cc);
			}
			vote(chip->gauge_term_voltage_votable, voter, true, update_vterm, false);
			vote(chip->gauge_shutdown_voltage_votable, SUPER_ENDURANCE_MODE_VOTER,
				!chip->super_endurance_mode_status, chip->deep_spec.config.term_voltage, false);
			vote(chip->gauge_shutdown_voltage_votable, voter,
				true, update_vshut, false);
		} else {
			if (chip->sub_gauge) {
				vote(chip->gauge_term_voltage_votable, SUB_DEEP_COUNT_VOTER,
				true, current_volt, false);
				vote(chip->gauge_shutdown_voltage_votable, SUB_DEEP_COUNT_VOTER, true, current_shut, false);
			}
			vote(chip->gauge_term_voltage_votable, voter,
				true, current_volt, false);
			vote(chip->gauge_shutdown_voltage_votable, SUPER_ENDURANCE_MODE_VOTER,
				!chip->super_endurance_mode_status, chip->deep_spec.config.term_voltage, false);
			vote(chip->gauge_shutdown_voltage_votable, voter, true, current_shut, false);
		}
	} else {
		if (chip->sub_gauge) {
			vote(chip->gauge_term_voltage_votable, SUB_DEEP_COUNT_VOTER, true,
			vterm, false);
			vote(chip->gauge_shutdown_voltage_votable, SUB_DEEP_COUNT_VOTER, true,
				vshut, false);
		}
		vote(chip->gauge_term_voltage_votable, voter, true,
			vterm, false);
		vote(chip->gauge_shutdown_voltage_votable, SUPER_ENDURANCE_MODE_VOTER,
			!chip->super_endurance_mode_status, chip->deep_spec.config.term_voltage, false);
		vote(chip->gauge_shutdown_voltage_votable, voter, true,
			vshut, false);
	}
	if (chip->sub_gauge)
		chg_info(" [%d, %d][%d, %d, %d, %d] [%d, %d, %d, %d, %d] [%d, %d, %d, %d]\n",
			update_vterm, update_vshut,
			vterm, current_volt, vshut, current_shut,
			last_cc, *cc, chip->deep_spec.counts, chip->deep_spec.ratio, *step_status,
			sub_last_cc, chip->deep_spec.sub_cc, chip->deep_spec.sub_counts, chip->deep_spec.sub_ratio);
	else
		chg_info(" [%d, %d][%d, %d, %d, %d] [%d, %d, %d, %d, %d]\n", update_vterm, update_vshut,
			vterm, current_volt, vshut, current_shut, last_cc, *cc,
			chip->deep_spec.counts, chip->deep_spec.ratio, *step_status);

	chip->deep_spec.config.index_r = ddrc_curve->index_r;
	chip->deep_spec.config.index_t = ddrc_curve->index_t;
	*step_status = false;
}

void oplus_gauge_set_deep_dischg_count(struct oplus_mms *mms, int count)
{
	int rc = 0;
	int i;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (!chip  || !chip->deep_spec.support || count < 0)
		return;

	if (mms == chip->gauge_topic) {
		chip->deep_spec.counts = count;
		chip->deep_spec.sub_counts = count;
	} else if (chip->sub_gauge && __ffs(chip->sub_gauge) < GAUGE_IC_NUM_MAX &&
		   mms == chip->gauge_topic_parallel[__ffs(chip->sub_gauge)]) {
		chip->deep_spec.sub_counts = count;
	} else {
		chip->deep_spec.counts = count;
	}

	if (mms == chip->gauge_topic) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_SET_DEEP_DISCHG_COUNT, count);
			if (rc < 0) {
				chg_err("gauge[%d](%s): can't set gauge dischg count, rc=%d\n", i, ic->manu_name, rc);
				continue;
			}
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (mms != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_SET_DEEP_DISCHG_COUNT, count);
			if (rc < 0)
				chg_err("gauge[%d](%s): can't  set gauge dischg count, rc=%d\n", i, ic->manu_name, rc);
			return;
		}
	}
}

void oplus_gauge_set_deep_count_cali(struct oplus_mms *topic, int val)
{
	struct oplus_mms_gauge *chip;
	bool charging;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return;
	}
	chip = oplus_mms_get_drvdata(topic);

	if (!chip  || !chip->deep_spec.support || val < 0)
		return;
	charging = chip->wired_online || chip->wls_online;
	chip->deep_spec.config.count_cali = val;
	if (!charging) {
		chip->deep_spec.config.step_status = true;
		oplus_gauge_get_ddrc_status(chip->gauge_topic);
	}
	chg_info(" val = %d\n", val);
}

void oplus_gauge_set_deep_dischg_ratio_thr(struct oplus_mms *topic, int ratio)
{
	struct oplus_mms_gauge *chip;
	bool charging;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return;
	}
	chip = oplus_mms_get_drvdata(topic);

	if (!chip  || !chip->deep_spec.support || ratio < 0 || ratio > 100) {
		chg_err("ratio(%d) invalid\n", ratio);
		return;
	}
	charging = chip->wired_online || chip->wls_online;
	chip->deep_spec.config.ratio_default = ratio;
	chip->deep_spec.config.ratio_shake = chip->deep_spec.config.ratio_default;
	chip->deep_spec.config.ratio_status = false;
	if (!charging) {
		chip->deep_spec.config.step_status = true;
		oplus_gauge_get_ddrc_status(chip->gauge_topic);
	}
	chg_info(" chip->deep_spec.config.ratio_default = %d\n", chip->deep_spec.config.ratio_default);
}

#define GAUGE_INVALID_DEEP_COUNT_RATIO_THR	10
int oplus_gauge_get_deep_dischg_ratio_thr(struct oplus_mms *topic)
{
	int rc = -GAUGE_INVALID_DEEP_COUNT_RATIO_THR;
	struct oplus_mms_gauge *chip;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return rc;
	}
	chip = oplus_mms_get_drvdata(topic);

	if (!chip  || !chip->deep_spec.support)
		return rc;

	return chip->deep_spec.config.ratio_default;
}

static int oplus_gauge_get_batt_id_info(struct oplus_mms_gauge *chip)
{
	int rc, temp = GPIO_STATUS_NOT_SUPPORT;

	if (!chip)
		return GPIO_STATUS_NOT_SUPPORT;


	rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATTID_INFO, &temp);
	if (rc < 0) {
		if (rc != -ENOTSUPP)
			chg_err(" get battid info error, rc=%d\n", rc);
		return GPIO_STATUS_NOT_SUPPORT;
	}

	return temp;
}

static int oplus_gauge_get_batt_id_match_info(struct oplus_mms_gauge *chip)
{
	int rc, temp = ID_MATCH_IGNORE;

	if (!chip)
		return ID_MATCH_IGNORE;

	rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATTID_MATCH_INFO, &temp);
	if (rc < 0) {
		if (rc != -ENOTSUPP)
			chg_err(" get battid match info error, rc=%d\n", rc);
		return ID_MATCH_IGNORE;
	}

	return temp;
}

static void  oplus_mms_gauge_set_sili_spare_power_enable(struct oplus_mms *mms)
{
	int rc = 0;
	int i;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return;
	}

	chip = oplus_mms_get_drvdata(mms);

	if (mms == chip->gauge_topic) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_SET_SILI_SPARE_POWER);
			if (rc < 0) {
				chg_err("gauge[%d](%s): can't set gauge sili spare power, rc=%d\n", i, ic->manu_name, rc);
				break;
			}
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (mms != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_SET_SILI_SPARE_POWER);
			if (rc < 0)
				chg_err("gauge[%d](%s): can't set set gauge sili spare power, rc=%d\n", i, ic->manu_name, rc);
			break;
		}
	}

	if (!rc)
		chip->deep_spec.spare_power_enable = true;
}

static void  oplus_mms_gauge_set_sili_ic_alg_cfg(struct oplus_mms *mms, int cfg)
{
	int rc = 0;
	int i;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return;
	}

	chip = oplus_mms_get_drvdata(mms);

	if (mms == chip->gauge_topic) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_SET_SILI_IC_ALG_CFG, cfg);
			if (rc < 0) {
				chg_err("gauge[%d](%s): can't set sili ic alg cfg, rc=%d\n", i, ic->manu_name, rc);
				continue;
			}
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (mms != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_SET_SILI_IC_ALG_CFG, cfg);
			if (rc < 0)
				chg_err("gauge[%d](%s): can't set sili ic alg cfg, rc=%d\n", i, ic->manu_name, rc);
			return;
		}
	}
}

static void  oplus_mms_gauge_set_sili_ic_alg_term_volt(
			struct oplus_mms *mms, int volt)
{
	int rc = 0;
	int i;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return;
	}

	chip = oplus_mms_get_drvdata(mms);

	if (mms == chip->gauge_topic) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_SET_SILI_IC_ALG_TERM_VOLT, volt);
			if (rc < 0) {
				chg_err("gauge[%d](%s): can't set gauge sili ic alg term volt, rc=%d\n", i, ic->manu_name, rc);
				continue;
			}
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (mms != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_SET_SILI_IC_ALG_TERM_VOLT, volt);
			if (rc < 0)
				chg_err("gauge[%d](%s): can't set gauge sili ic alg term volt, rc=%d\n", i, ic->manu_name, rc);
			return;
		}
	}
}

static int oplus_gauge_get_sili_simulate_term_volt(struct oplus_mms_gauge *chip, int *volt)
{
	int rc = 0;

	if (!chip || !volt)
		return -EINVAL;

	rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_SILI_SIMULATE_TERM_VOLT, volt);

	return rc;
}

static int oplus_gauge_get_sili_ic_alg_dsg_enable(struct oplus_mms_gauge *chip, bool *dsg_enable)
{
	int rc = 0;

	if (!chip || !dsg_enable)
		return -EINVAL;

	rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_SILI_IC_ALG_DSG_ENABLE, dsg_enable);

	return rc;
}

static void oplus_mms_gauge_get_sili_ic_alg_term_volt(struct oplus_mms *mms, int *volt)
{
	int temp_volt = 0;
	int max_volt = 0;
	int rc = 0;
	int i;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;

	if (mms == NULL || volt == NULL) {
		chg_err("mms or volt is NULL");
		return;
	}

	chip = oplus_mms_get_drvdata(mms);

	if (mms == chip->gauge_topic) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_SILI_IC_ALG_TERM_VOLT, &temp_volt);
			if (rc < 0) {
				chg_err("gauge[%d](%s): can't get gauge sili ic alg term volt, rc=%d\n", i, ic->manu_name, rc);
				continue;
			}
			if (temp_volt > max_volt)
				max_volt = temp_volt;
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (mms != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_SILI_IC_ALG_TERM_VOLT, &max_volt);
			if (rc < 0)
				chg_err("gauge[%d](%s):  can't get gauge sili ic alg term volt, rc=%d\n", i, ic->manu_name, rc);
			return;
		}
	}
	*volt = max_volt;
}

int oplus_gauge_get_sili_alg_application_info(struct oplus_mms *mms, u8 *info, int len)
{
	int rc = 0;
	int i;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);

	if (mms == chip->gauge_topic) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_SILI_ALG_APPLICATION_INFO, info, len);
			if (rc < 0) {
				chg_err("gauge[%d](%s): can't get gauge sili alg application, rc=%d\n", i, ic->manu_name, rc);
				continue;
			}
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (mms != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_SILI_ALG_APPLICATION_INFO, info, len);
			if (rc < 0)
				chg_err("gauge[%d](%s): can'tget gauge sili alg application, rc=%d\n", i, ic->manu_name, rc);
			return rc;
		}
	}

	return rc;
}

int oplus_gauge_get_sili_alg_lifetime_info(struct oplus_mms *mms, u8 *info, int len)
{
	int rc = 0;
	int i;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);

	if (mms == chip->gauge_topic) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_SILI_LIFETIME_INFO, info, len);
			if (rc < 0) {
				chg_err("gauge[%d](%s): can't get gauge sili lifetime info, rc=%d\n", i, ic->manu_name, rc);
				continue;
			}
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (mms != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_SILI_LIFETIME_INFO, info, len);
			if (rc < 0)
				chg_err("gauge[%d](%s): can'tget gauge sili lifetime info, rc=%d\n", i, ic->manu_name, rc);
			return rc;
		}
	}

	return rc;
}

static void oplus_gauge_init_sili_status(struct oplus_mms_gauge *chip)
{
	int byb_match_status = 0, batt_match_status = 0;
	int bybid = 0, batt_id = 0;

	if (!chip)
		return;

	byb_match_status = oplus_wired_get_byb_id_match_info(chip->wired_topic);

	batt_match_status = oplus_gauge_get_batt_id_match_info(chip);
	if ((byb_match_status == ID_NOT_MATCH) && (batt_match_status == ID_MATCH_SILI))
		chip->deep_spec.sili_err = true;
	else
		chip->deep_spec.sili_err = false;

	bybid = oplus_wired_get_byb_id_info(chip->wired_topic);
	batt_id = oplus_gauge_get_batt_id_info(chip);

	scnprintf(deep_id_info, DUMP_INFO_LEN, "$$deep_support@@%d$$byb_id@@%d$$batt_id@@%d$$sili_err@@%d$$counts@@%d$$uv_thr@@%d",
		chip->deep_spec.support, bybid, batt_id, chip->deep_spec.sili_err, chip->deep_spec.counts, chip->deep_spec.config.uv_thr);

	chg_info(" [%d, %d, %d, %d, %d, %d]\n", byb_match_status, batt_match_status, bybid, batt_id,
		chip->deep_spec.sili_err, chip->deep_spec.support);
}

#define BAT_TYPE_MESSAGE_LEN       36
#define OPLUS_SILICON_TYPE_TAG     "silicon"
#define OPLUS_GRAPHITE_TYPE_TAG    "graphite"
#define OPLUS_BATT_TYPE_TAG        "battery_type="

static char __oplus_chg_cmdline[BAT_TYPE_MESSAGE_LEN];
static char *oplus_chg_cmdline = __oplus_chg_cmdline;

static const char *oplus_battype_get_cmdline(void)
{
	struct device_node * of_chosen = NULL;
	char *bat_type = NULL;

	if (__oplus_chg_cmdline[0] != 0)
		return oplus_chg_cmdline;

	of_chosen = of_find_node_by_path("/chosen");
	if (of_chosen) {
		bat_type = (char *)of_get_property(
					of_chosen, "bat_type", NULL);
		if (!bat_type)
			chg_err("failed to get bat_type\n");
		else {
			strcpy(__oplus_chg_cmdline, bat_type);
			chg_debug("bat_type: %s\n", bat_type);
		}
	} else {
		chg_err("failed to get /chosen \n");
	}

	return oplus_chg_cmdline;
}

int oplus_gauge_get_battery_type_str(char *type)
{
#ifdef CONFIG_OPLUS_CHARGER_MTK
	struct device_node *node;
	char *str = NULL;
	const char *cmd_line = NULL;

	if (!type)
		return -ENOTSUPP;

	node = of_find_node_by_path("/soc/oplus_chg_core");
	if (node == NULL)
		return -ENOTSUPP;
	if (!of_property_read_bool(node, "oplus,battery_type_by_cmdline"))
		return -ENOTSUPP;

	cmd_line = oplus_battype_get_cmdline();
	if (NULL == cmd_line) {
		chg_debug("oplus_battype_get_cmdline: cmdline is NULL!!!\n");
		return -ENOTSUPP;
	}

	str = strstr(cmd_line, OPLUS_SILICON_TYPE_TAG);
	if (str == NULL) {
		/* check the graphite battery type again. */
		str = strstr(cmd_line, OPLUS_GRAPHITE_TYPE_TAG);
		if (str == NULL) {
			chg_err("get battery type is not supported!!!\n");
			return -ENOTSUPP;
		}
	}
	chg_debug("current battery type %s\n", str);

	snprintf(type, OPLUS_BATTERY_TYPE_LEN, "%s", str);
	return 0;
#else
	size_t smem_size;
	static oplus_ap_feature_data *smem_data;
	struct device_node *node;
	char *str = NULL;
	const char *cmd_line = NULL;

	if (!type)
		return -ENOTSUPP;

	node = of_find_node_by_path("/soc/oplus_chg_core");
	if (node == NULL)
		return -ENOTSUPP;
	if (of_property_read_bool(node, "oplus,battery_type_by_smem")) {
		if (!smem_data) {
			smem_data = (oplus_ap_feature_data *)qcom_smem_get(QCOM_SMEM_HOST_ANY, SMEM_OPLUS_CHG, &smem_size);
			if (IS_ERR_OR_NULL(smem_data)) {
				chg_err("unable to acquire smem oplus chg entry\n");
				return -EINVAL;
			}
			if (smem_data->size != sizeof(oplus_ap_feature_data)) {
				chg_err("size invalid %d %zu\n", smem_data->size, sizeof(oplus_ap_feature_data));
				return -EINVAL;
			}
			chg_info("current battery type str = %s\n", smem_data->battery_type_str);
		}

		snprintf(type, OPLUS_BATTERY_TYPE_LEN, "%s", smem_data->battery_type_str);
	} else if (of_property_read_bool(node, "oplus,battery_type_by_cmdline")) {
		cmd_line = oplus_battype_get_cmdline();
		if (NULL == cmd_line) {
			chg_debug("oplus_battype_get_cmdline: cmdline is NULL!!!\n");
			return -ENOTSUPP;
		}

		str = strstr(cmd_line, OPLUS_BATT_TYPE_TAG);
		if (str == NULL) {
			chg_err("get battery type is not supported!!!\n");
			return -ENOTSUPP;
		}
		str += strlen(OPLUS_BATT_TYPE_TAG);
		chg_debug("current battery type %s\n", str);

		scnprintf(type, OPLUS_BATTERY_TYPE_LEN, "%s", str);
	} else {
		return -ENOTSUPP;
	}
	return 0;
#endif
}



void oplus_mms_gauge_update_super_endurance_mode_status_work(struct work_struct *work)
{
	struct oplus_mms_gauge *chip =
		container_of(work, struct oplus_mms_gauge, update_super_endurance_mode_status_work);

	if (!chip->deep_spec.support)
		return;
	if (chip->deep_spec.sili_ic_alg_dsg_enable)
		oplus_mms_gauge_update_sili_ic_alg_term_volt(chip, true);

	vote(chip->gauge_shutdown_voltage_votable, SUPER_ENDURANCE_MODE_VOTER,
			!chip->super_endurance_mode_status, chip->deep_spec.config.term_voltage, false);
}

void oplus_mms_gauge_update_sili_ic_alg_term_volt(
	struct oplus_mms_gauge *chip, bool force)
{
	int alg_term_volt = 0;
	static bool first_record = true;

	mutex_lock(&chip->deep_spec.lock);
	if (!chip->deep_spec.sili_ic_alg_dsg_enable)
		goto not_handle;

	/* update just for uisoc < 15% */
	if (chip->ui_soc >= 15) {
		first_record = true;
		goto not_handle;
	}

	if (!chip->deep_spec.config.term_voltage ||
	    !is_client_vote_enabled(chip->gauge_term_voltage_votable, READY_VOTER)) {
		goto not_handle;
	}

	oplus_mms_gauge_get_sili_ic_alg_term_volt(chip->gauge_topic, &alg_term_volt);
	if (alg_term_volt && (force || first_record || !chip->deep_spec.sili_ic_alg_term_volt ||
	    chip->deep_spec.sili_ic_alg_term_volt > alg_term_volt)) {
		chip->deep_spec.sili_ic_alg_term_volt = alg_term_volt;
		oplus_mms_gauge_set_sili_ic_alg_term_volt(chip->gauge_topic, chip->deep_spec.sili_ic_alg_term_volt);
		if (!chip->super_endurance_mode_status)
			chip->deep_spec.config.uv_thr = alg_term_volt;
		else
			chip->deep_spec.config.uv_thr = alg_term_volt - GAUGE_TERM_VOLT_EFFECT_GAP_MV(100);
		chg_info("uv_thr=%d\n", chip->deep_spec.config.uv_thr);
		oplus_mms_gauge_push_vbat_uv(chip);
	}
	first_record = false;
not_handle:
	mutex_unlock(&chip->deep_spec.lock);
}

void oplus_mms_gauge_update_sili_ic_alg_cfg_work(struct work_struct *work)
{
	int rc;
	int alg_cfg;
	union mms_msg_data data = { 0 };
	struct oplus_mms_gauge *chip =
		container_of(work, struct oplus_mms_gauge, update_sili_ic_alg_cfg_work);

	if (!chip->deep_spec.support || !chip->deep_spec.sili_ic_alg_support) {
		chg_err("not support\n");
		return;
	}

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SILI_IC_ALG_CFG, &data, false);
	if (rc < 0)
		return;

	alg_cfg = data.intval;
	mutex_lock(&chip->deep_spec.lock);
	oplus_mms_gauge_set_sili_ic_alg_cfg(chip->gauge_topic, alg_cfg);
	oplus_gauge_get_sili_ic_alg_dsg_enable(chip, &chip->deep_spec.sili_ic_alg_dsg_enable);
	if (!chip->deep_spec.sili_ic_alg_dsg_enable) {
		vote(chip->gauge_term_voltage_votable, READY_VOTER, false, 0, true);
		vote(chip->gauge_shutdown_voltage_votable, SUPER_ENDURANCE_MODE_VOTER,
			!chip->super_endurance_mode_status, chip->deep_spec.config.term_voltage, false);
		vote(chip->gauge_shutdown_voltage_votable, READY_VOTER, false, 0, false);
		chip->deep_spec.sili_ic_alg_term_volt = 0;
		oplus_mms_gauge_set_sili_ic_alg_term_volt(chip->gauge_topic, chip->deep_spec.sili_ic_alg_term_volt);
	} else {
		vote(chip->gauge_shutdown_voltage_votable, READY_VOTER, true, INVALID_MAX_VOLTAGE, false);
		vote(chip->gauge_term_voltage_votable, READY_VOTER, true, INVALID_MAX_VOLTAGE, false);
	}
	mutex_unlock(&chip->deep_spec.lock);
	chg_info("alg_cfg=0x%x, sili_ic_alg_enable=%d\n", alg_cfg, chip->deep_spec.sili_ic_alg_dsg_enable);
}

void oplus_mms_gauge_update_sili_spare_power_enable_work(struct work_struct *work)
{
	int soc;
	int temp;
	bool enable;
	union mms_msg_data data = { 0 };
	struct oplus_mms_gauge *chip =
		container_of(work, struct oplus_mms_gauge, update_sili_spare_power_enable_work);

	if (!chip->deep_spec.support || !chip->deep_spec.spare_power_support || !chip->deep_spec.sili_ic_alg_dsg_enable) {
		chg_err("not support\n");
		return;
	}

	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SPARE_POWER_ENABLE, &data, false);
	if (!!data.intval)
		enable = true;
	else
		enable = false;

	chg_info("enable=%d\n", enable);
	if (!is_support_parallel(chip)) {
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_TEMP, &data, false);
		temp = data.intval;
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOC, &data, false);
		soc = data.intval;
	} else {
		oplus_mms_get_item_data(chip->gauge_topic_parallel[chip->main_gauge], GAUGE_ITEM_TEMP, &data, false);
		temp = data.intval;
		oplus_mms_get_item_data(chip->gauge_topic_parallel[chip->main_gauge], GAUGE_ITEM_SOC, &data, false);
		soc = data.intval;
	}

	chip->deep_spec.spare_power_enable = false;
	/* support for battery temp at 25C-40C and uisoc <= 5% and real soc > 0 */
	if (temp > 250 && temp < 400 && chip->ui_soc <= 5 && soc) {
		oplus_mms_gauge_set_sili_spare_power_enable(chip->gauge_topic);
		cancel_delayed_work(&chip->sili_spare_power_effect_check_work);
		schedule_delayed_work(&chip->sili_spare_power_effect_check_work, msecs_to_jiffies(2000));
	}
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SPARE_POWER_ENABLE, &data, true);
}

void oplus_mms_gauge_sili_spare_power_effect_check_work(struct work_struct *work)
{
	int alg_term_volt = 0;
	int spare_power_term_volt;
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_gauge *chip =
		container_of(dwork, struct oplus_mms_gauge, sili_spare_power_effect_check_work);

	spare_power_term_volt = chip->deep_spec.config.spare_power_term_voltage;
	oplus_mms_gauge_get_sili_ic_alg_term_volt(chip->gauge_topic, &alg_term_volt);
	chg_info("ic_alg_term_volt=%d\n", alg_term_volt);

	if (abs(spare_power_term_volt - alg_term_volt) < GAUGE_TERM_VOLT_EFFECT_GAP_MV(20)) {
		chg_info("spare power set success\n");
	} else {
		oplus_mms_gauge_set_sili_spare_power_enable(chip->gauge_topic);
		schedule_delayed_work(&chip->sili_spare_power_effect_check_work, msecs_to_jiffies(2000));
	}
}

void oplus_mms_gauge_sili_term_volt_effect_check_work(struct work_struct *work)
{
	int rc;
	int simulate_volt = 0;
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_gauge *chip =
		container_of(dwork, struct oplus_mms_gauge, sili_term_volt_effect_check_work);

	if (chip->deep_spec.sili_ic_alg_dsg_enable) {
		chg_err("dsg enable, not need check\n");
		return;
	}

	rc = oplus_gauge_get_sili_simulate_term_volt(chip, &simulate_volt);
	if (rc < 0)
		return;

	chg_info("expect term voltage=%d, simulate volt=%d\n", chip->deep_spec.config.term_voltage, simulate_volt);
	if (abs(chip->deep_spec.config.term_voltage - simulate_volt) < GAUGE_TERM_VOLT_EFFECT_GAP_MV(20))
		chg_info("self-developed expect term voltage set success\n");
	else
		oplus_mms_gauge_set_deep_term_volt(chip->gauge_topic, chip->deep_spec.config.term_voltage);
}

#define DDRC_NAME_LENGTH 32
static int oplus_voocphy_parse_ddrc_strategy(struct oplus_mms_gauge *chip)
{
	struct device_node *startegy_node;
	int i, length;
	char ddrc_name[DDRC_NAME_LENGTH] = {0};
	int rc = 0;
	struct device_node *child_node;
	struct deep_dischg_limits *config = &chip->deep_spec.config;
	u32 data;
	struct device_node *node = oplus_get_node_by_type(chip->dev->of_node);

	chip->ddrc_num = 0;
	for_each_child_of_node(node, child_node) {
		if (strncmp(child_node->name, "ddrc_strategy", strlen("ddrc_strategy")) == 0)
			chip->ddrc_num++;
	}
	if (!chip->ddrc_num)
		return 0;
	chip->ddrc_strategy = devm_kzalloc(chip->dev, sizeof(struct oplus_chg_strategy *) * chip->ddrc_num, GFP_KERNEL);
	rc = of_property_read_string(node, "deep_spec,ddrc_strategy_name", (const char **)&config->ddrc_strategy_name);
	if (rc < 0) {
		chg_err("oplus,ddrc_strategy_name reading failed, rc=%d\n", rc);
		config->ddrc_strategy_name = "ddrc_curve";
	}
	if (strncmp(config->ddrc_strategy_name, "ddrc_curve_v2", strlen("ddrc_curve_v2")) == 0)
		chip->deep_spec.ddrc_strategy_v2 = true;
	else
		chip->deep_spec.ddrc_strategy_v2 = false;

	if (chip->deep_spec.ddrc_strategy_v2) {
		if ((of_property_read_u32(node, "oplus,ddrc_temp_switch", &chip->deep_spec.ddrc_temp_switch)) < 0) {
			chip->deep_spec.ddrc_temp_switch = 3;
			chg_info("ddrc_temp_switch use default val\n");
		}
		chg_info("ddrc_temp_switch %d\n", chip->deep_spec.ddrc_temp_switch);
	}

	startegy_node = of_get_child_by_name(node, "ddrc_strategy");
	if (!startegy_node) {
		chg_err("ddrc_strategy not found\n");
		return -EINVAL;
	} else {
		chip->ddrc_strategy[0] = oplus_chg_strategy_alloc_by_node(config->ddrc_strategy_name, startegy_node);
		if (IS_ERR_OR_NULL(chip->ddrc_strategy[0])) {
			chg_err("alloc deep dischg ratio startegy error, rc=%ld\n", PTR_ERR(chip->ddrc_strategy[0]));
			oplus_chg_strategy_release(chip->ddrc_strategy[0]);
			chip->ddrc_strategy[0] = NULL;
			return -EINVAL;
		} else {
			chg_info("ddrc_strategy 0 alloc success\n");
		}
		rc = of_property_read_u32(startegy_node, "oplus,temp_type", &data);
		if (rc < 0) {
			chg_err("ddrc_tbatt oplus,temp_type reading failed, rc=%d\n", rc);
			chip->deep_spec.ddrc_tbatt.temp_type = STRATEGY_USE_SHELL_TEMP;
		} else {
			chip->deep_spec.ddrc_tbatt.temp_type = (int)data;
		}

		rc = of_property_count_elems_of_size(startegy_node, "oplus,temp_range", sizeof(u32));
		if (rc < 0) {
			chg_err("Count ratio_temp_range failed, rc=%d\n", rc);
			oplus_chg_strategy_release(chip->ddrc_strategy[0]);
			chip->ddrc_strategy[0] = NULL;
			return -ENODEV;
		}
		length = rc;
		chip->deep_spec.ddrc_temp_num = length;
		chip->deep_spec.ddrc_tbatt.range = devm_kzalloc(chip->dev, sizeof(int) * length, GFP_KERNEL);
		rc = of_property_read_u32_array(startegy_node, "oplus,temp_range",
								(u32 *)chip->deep_spec.ddrc_tbatt.range, length);
		if (rc < 0) {
			chg_err("ddrc_tbatt get oplus,temp_range property error, rc=%d\n", rc);
		}
		chip->deep_spec.ddrc_tdefault.range = devm_kzalloc(chip->dev, sizeof(int) * length, GFP_KERNEL);
		for (i = 0; i < length; i++)
			chip->deep_spec.ddrc_tdefault.range[i] = chip->deep_spec.ddrc_tbatt.range[i];
	}
	for (i = 1; i < chip->ddrc_num && rc >= 0; i++) {
		snprintf(ddrc_name, sizeof(ddrc_name), "ddrc_strategy_%d", i);
		startegy_node = of_get_child_by_name(node, ddrc_name);
		if (!startegy_node) {
			rc = -EINVAL;
			break;
		} else {
			chip->ddrc_strategy[i] = oplus_chg_strategy_alloc_by_node(config->ddrc_strategy_name, startegy_node);
			if (IS_ERR_OR_NULL(chip->ddrc_strategy[i])) {
				chg_err("alloc deep dischg ratio startegy %d error, rc=%ld\n", i, PTR_ERR(chip->ddrc_strategy[i]));
				oplus_chg_strategy_release(chip->ddrc_strategy[i]);
				chip->ddrc_strategy[i] = NULL;
				rc = -EINVAL;
				break;
			} else {
				chg_info("ddrc_strategy %d alloc success\n", i);
			}
		}
	}

	chg_info("parse ddrc_strategy succ num:%d\n", chip->ddrc_num);

	return rc;
}

#define OPLUS_TERM_VOLT_MAX_LEVEL       3
static int oplus_gauge_parse_three_level_term_volt_strategy(struct oplus_mms_gauge *chip)
{
	int rc = 0;
	int length = 0;
	struct device_node *node;
	u32 data[OPLUS_TERM_VOLT_MAX_LEVEL] = { 0 };

	node = oplus_get_node_by_type(chip->dev->of_node);
	if (NULL == node)
		return -EINVAL;

	rc = of_property_count_elems_of_size(node, "oplus_spec,term_volt", sizeof(u32));
	if (rc < 0) {
		chg_err("Count three level term_volt failed, rc=%d\n", rc);
	} else {
		length = rc;
		if (length <= OPLUS_TERM_VOLT_MAX_LEVEL) {
			rc = of_property_read_u32_array(node, "oplus_spec,term_volt", data, length);
			if (rc == 0) {
				chip->three_level_term_volt_cfg.term_volt = (data[0] & 0xffff);
				chip->three_level_term_volt_cfg.term_volt_2 = (data[1] & 0xffff);
				chip->three_level_term_volt_cfg.term_volt_3 = (data[2] & 0xffff);
			}
		}
		chip->three_level_term_volt_cfg.term_volt_size = length;
	}

	memset(data, 0, OPLUS_TERM_VOLT_MAX_LEVEL* sizeof(u32));
	rc = of_property_count_elems_of_size(node, "oplus_spec,term_volt_hold_time", sizeof(u32));
	if (rc < 0) {
		chg_err("Count three level term_volt_hold_time failed, rc=%d\n", rc);
	} else {
		length = rc;
		if (length <= OPLUS_TERM_VOLT_MAX_LEVEL) {
			rc = of_property_read_u32_array(node, "oplus_spec,term_volt_hold_time",
				data, length);
			if (rc == 0) {
				chip->three_level_term_volt_cfg.hold_time = (data[0] & 0xff);
				chip->three_level_term_volt_cfg.hold_time_2 = (data[1] & 0xff);
				chip->three_level_term_volt_cfg.hold_time_3 = (data[2] & 0xff);
			}
		}
	}

	memset(data, 0, OPLUS_TERM_VOLT_MAX_LEVEL* sizeof(u32));
	rc = of_property_count_elems_of_size(node, "oplus_spec,time_to_drop_per1", sizeof(u32));
	if (rc < 0) {
		chg_err("Count three level time_to_drop_per1 failed, rc=%d\n", rc);
	} else {
		length = rc;
		if (length <= OPLUS_TERM_VOLT_MAX_LEVEL) {
			rc = of_property_read_u32_array(node, "oplus_spec,time_to_drop_per1", data, length);
			if (rc == 0) {
				chip->three_level_term_volt_cfg.time_to_drop_per1 = (data[0] & 0xff);
				chip->three_level_term_volt_cfg.time_to_drop_per1_2 = (data[1] & 0xff);
				chip->three_level_term_volt_cfg.time_to_drop_per1_3 = (data[2] & 0xff);
			}
		}
	}

	memset(data, 0, OPLUS_TERM_VOLT_MAX_LEVEL* sizeof(u32));
	rc = of_property_count_elems_of_size(node, "oplus_spec,recover_term_volt", sizeof(u32));
	if (rc < 0) {
		chg_err("Count three level recover_term_volt failed, rc=%d\n", rc);
	} else {
		length = rc;
		if (length <= OPLUS_TERM_VOLT_MAX_LEVEL) {
			rc = of_property_read_u32_array(node, "oplus_spec,recover_term_volt", data, length);
			if (rc == 0) {
				chip->three_level_term_volt_cfg.recover_term_volt = (data[0] & 0xffff);
				chip->three_level_term_volt_cfg.recover_term_volt_2 = (data[1] & 0xffff);
			}
		}
	}

	memset(data, 0, OPLUS_TERM_VOLT_MAX_LEVEL* sizeof(u32));
	rc = of_property_count_elems_of_size(node, "oplus_spec,recover_term_volt_hold_time", sizeof(u32));
	if (rc < 0) {
		chg_err("Count three level recover_term_volt_hold_time failed, rc=%d\n", rc);
	} else {
		length = rc;
		if (length <= OPLUS_TERM_VOLT_MAX_LEVEL) {
			rc = of_property_read_u32_array(node, "oplus_spec,recover_term_volt_hold_time", data, length);
			if (rc == 0) {
				chip->three_level_term_volt_cfg.recover_hold_time_of_term_voltage = (data[0] & 0xff);
				chip->three_level_term_volt_cfg.recover_hold_time_of_term_voltage_2 = (data[1] & 0xff);
			}
		}
	}

	chg_info("length=%d, term_volt=[%d, %d, %d], holdtime=[%d, %d, %d], time_to_drop_per1=[%d,%d,%d]," \
		 "recover_term_volt[%d,%d], recover_hold_time[%d, %d]\n",
		 length, chip->three_level_term_volt_cfg.term_volt,
		 chip->three_level_term_volt_cfg.term_volt_2,
		 chip->three_level_term_volt_cfg.term_volt_3,
		 chip->three_level_term_volt_cfg.hold_time,
		 chip->three_level_term_volt_cfg.hold_time_2,
		 chip->three_level_term_volt_cfg.hold_time_3,
		 chip->three_level_term_volt_cfg.time_to_drop_per1,
		 chip->three_level_term_volt_cfg.time_to_drop_per1_2,
		 chip->three_level_term_volt_cfg.time_to_drop_per1_3,
		 chip->three_level_term_volt_cfg.recover_term_volt,
		 chip->three_level_term_volt_cfg.recover_term_volt_2,
		 chip->three_level_term_volt_cfg.recover_hold_time_of_term_voltage,
		 chip->three_level_term_volt_cfg.recover_hold_time_of_term_voltage_2);

	return rc;
}

int oplus_gauge_parse_deep_spec(struct oplus_mms_gauge *chip)
{
	struct device_node *node;
	int i = 0, rc = 0, length;
	u32 data;
	struct device_node *curves_node;

	if (!chip)
		return -ENODEV;

	node = oplus_get_node_by_type(chip->dev->of_node);

	rc = of_property_count_elems_of_size(node, "deep_spec,term_coeff", sizeof(u32));
	if (rc < 0) {
		chg_err("Count deep spec term_coeff failed, rc=%d\n", rc);
	} else {
		length = rc;
		if (length % DDT_COEFF_SIZE == 0 &&
		    length / DDT_COEFF_SIZE <= DDC_CURVE_MAX) {
			rc = of_property_read_u32_array(node, "deep_spec,term_coeff", (u32 *)chip->deep_spec.term_coeff,
							length);
			chip->deep_spec.term_coeff_size = length / DDT_COEFF_SIZE;
		}
	}
	rc = of_property_read_u32(node, "deep_spec,uv_thr",
			&chip->deep_spec.config.uv_thr);
	if (rc < 0)
		chip->deep_spec.config.uv_thr = 3000;

	rc = of_property_read_u32(node, "deep_spec,count_cali",
			&chip->deep_spec.config.count_cali);
	if (rc < 0)
		chip->deep_spec.config.count_cali = 0;

	rc = of_property_read_u32(node, "deep_spec,count_thr",
			&chip->deep_spec.config.count_thr);
	if (rc < 0)
		chip->deep_spec.config.count_thr = 1;

	rc = of_property_read_u32(node, "deep_spec,spare_power_term_voltage",
			&chip->deep_spec.config.spare_power_term_voltage);
	if (rc < 0)
		chip->deep_spec.config.spare_power_term_voltage = 2700;

	rc = of_property_read_u32(node, "deep_spec,vbat_soc",
			&chip->deep_spec.config.soc);
	if (rc < 0)
		chip->deep_spec.config.soc = 10;

	rc = of_property_read_u32(node, "deep_spec,curr_max_ma",
			&chip->deep_spec.config.curr_max_ma);
	if (rc < 0)
		chip->deep_spec.config.curr_max_ma = 10900;

	rc = of_property_read_u32(node, "deep_spec,curr_limit_ma",
			&chip->deep_spec.config.curr_limit_ma);
	if (rc < 0)
		chip->deep_spec.config.curr_limit_ma = 9100;

	rc = of_property_count_elems_of_size(node, "deep_spec,curr_limit_ratio", sizeof(u32));
	if (rc < 0) {
		chg_err("Count deep spec curr_limit_ratio curve failed, rc=%d\n", rc);
	} else {
		length = rc;
		rc = of_property_read_u32_array(node, "deep_spec,curr_limit_ratio",
			(u32 *)chip->deep_spec.limit_curr_curves.limits, length);
		chip->deep_spec.limit_curr_curves.nums = length / 3;
	}
	chg_err("chip->deep_spec.limit_curr_curves.nums = %d\n",
		chip->deep_spec.limit_curr_curves.nums);

	chip->deep_spec.support = of_property_read_bool(node, "deep_spec,support");
	chip->deep_spec.spare_power_support = of_property_read_bool(node, "deep_spec,spare_power_support");
	chip->deep_spec.sili_ic_alg_support = of_property_read_bool(node, "deep_spec,sili_ic_alg_support");

	rc = of_property_read_u32(node, "deep_spec,ratio_thr",
				&chip->deep_spec.config.ratio_default);
	if (rc < 0)
		chip->deep_spec.config.ratio_default = 30;
	chip->deep_spec.config.ratio_shake = chip->deep_spec.config.ratio_default;
	chip->deep_spec.config.sub_ratio_shake = chip->deep_spec.config.ratio_default;

	rc = of_property_count_elems_of_size(node, "deep_spec,count_step", sizeof(u32));
	if (rc < 0) {
		chg_err("Count deep spec count_step curve failed, rc=%d\n", rc);
	} else {
		length = rc;
		rc = of_property_read_u32_array(node, "deep_spec,count_step",
								(u32 *)chip->deep_spec.step_curves.limits, length);
		chip->deep_spec.step_curves.nums = length / 3;
	}

	if (chip->deep_spec.sili_ic_alg_support) {
		rc = of_property_count_elems_of_size(node, "deep_spec,sili_alg_cfg_list", sizeof(u32));
		if (rc > 0 && rc <= SILI_CFG_TYPE_MAX) {
			chip->deep_spec.sili_ic_alg_cfg.nums = rc;
			of_property_read_u32_array(node, "deep_spec,sili_alg_cfg_list",
								(u32 *)chip->deep_spec.sili_ic_alg_cfg.list, rc);
		}
	}

	rc = of_property_read_u32(node, "deep_spec,volt_step", &chip->deep_spec.config.volt_step);
	if (rc < 0)
		chip->deep_spec.config.volt_step = 100;

	curves_node = of_get_child_by_name(node, "deep_spec,ddbc_curve");
	if (!curves_node) {
		chg_err("Can not find deep_spec,ddbc_curve node\n");
		rc = -ENODEV;
		goto ddbc_strategy_err;
	}
	rc = of_property_read_u32(curves_node, "oplus,temp_type", &data);
	if (rc < 0) {
		chg_err("oplus,temp_type reading failed, rc=%d\n", rc);
		chip->deep_spec.ddbc_tbatt.temp_type = STRATEGY_USE_SHELL_TEMP;
	} else {
		chip->deep_spec.ddbc_tbatt.temp_type = (int)data;
	}

	rc = of_property_count_elems_of_size(curves_node, "oplus,temp_range", sizeof(u32));
	if (rc < 0) {
		chg_err("Count temp_range failed, rc=%d\n", rc);
		rc = -ENODEV;
		goto ddbc_strategy_err;
	}
	length = rc;
	chip->deep_spec.ddbc_tbatt.range = devm_kzalloc(chip->dev, sizeof(int) * length, GFP_KERNEL);
	rc = of_property_read_u32_array(curves_node, "oplus,temp_range",
							(u32 *)chip->deep_spec.ddbc_tbatt.range, length);
	if (rc < 0) {
		chg_err("get oplus,temp_range property error, rc=%d\n", rc);
		rc = -ENODEV;
		goto ddbc_strategy_err;
	}
	chip->deep_spec.ddbc_tdefault.range = devm_kzalloc(chip->dev, sizeof(int) * length, GFP_KERNEL);
	for (i = 0; i < DDB_CURVE_TEMP_WARM; i++)
		chip->deep_spec.ddbc_tdefault.range[i] = chip->deep_spec.ddbc_tbatt.range[i];

	for (i = 0; i <= DDB_CURVE_TEMP_WARM; i++) {
		rc = of_property_count_elems_of_size(curves_node, ddbc_curve_range_name[i], sizeof(u32));
		if (rc < 0) {
			chg_err("Count ddbc_curve_range_name %s failed, rc=%d\n",
				ddbc_curve_range_name[i], rc);
			rc = -ENODEV;
			goto ddbc_strategy_err;
		}
		length = rc;
		rc = of_property_read_u32_array(curves_node, ddbc_curve_range_name[i],
						(u32 *)chip->deep_spec.batt_curves[i].limits, length);
		if (rc < 0) {
			chg_err("parse chip->deep_spec.batt_curves[%d].limits failed, rc=%d\n", i, rc);
		}
		chip->deep_spec.batt_curves[i].nums = length / 3;
	}

	oplus_voocphy_parse_ddrc_strategy(chip);
	oplus_gauge_parse_three_level_term_volt_strategy(chip);

ddbc_strategy_err:

	return rc;
}


int oplus_mms_gauge_sili_ic_alg_cfg_init(struct oplus_mms_gauge *chip)
{
	int i;
	int rc = 0;
	int alg_cfg = 0;

	if (!chip->deep_spec.sili_ic_alg_support)
		return -EINVAL;

	for (i = 0; i < chip->deep_spec.sili_ic_alg_cfg.nums; i++)
		alg_cfg |= BIT(chip->deep_spec.sili_ic_alg_cfg.list[i]);

	oplus_mms_gauge_set_sili_ic_alg_cfg(chip->gauge_topic, alg_cfg);
	oplus_gauge_get_sili_ic_alg_dsg_enable(chip, &chip->deep_spec.sili_ic_alg_dsg_enable);
	chg_info("alg_cfg:0x%x, sili_ic_alg_enable:%d\n", alg_cfg, chip->deep_spec.sili_ic_alg_dsg_enable);

	if (chip->deep_spec.sili_ic_alg_dsg_enable) {
		vote(chip->gauge_shutdown_voltage_votable, READY_VOTER, true, INVALID_MAX_VOLTAGE, false);
		vote(chip->gauge_term_voltage_votable, READY_VOTER, true, INVALID_MAX_VOLTAGE, false);
		chg_info("sili_ic_alg_dsg_enable end\n");
	}
	return rc;
}


int oplus_mms_gauge_update_sili_ic_alg_term_volt_data(
					struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int volt = 0;
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
	rc = oplus_chg_ic_func(chip->gauge_ic,
		OPLUS_IC_FUNC_GAUGE_GET_SILI_IC_ALG_TERM_VOLT, &volt);
	if (rc < 0) {
		chg_err("get sili ic alg term volt error, rc=%d\n", rc);
		return -EINVAL;
	}

	data->intval = volt;
	return 0;
}

int oplus_mms_sub_gauge_update_sili_ic_alg_term_volt(
					struct oplus_mms *mms, union mms_msg_data *data)
{
	int rc = -1;
	int i;
	int volt = 0;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);

	for (i = 0; i < chip->child_num; i++) {
		if (mms != chip->gauge_topic_parallel[i])
			continue;
		ic = chip->gauge_ic_comb[i];
		rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_SILI_IC_ALG_TERM_VOLT, &volt);
		if (rc < 0)
			chg_err("gauge[%d](%s): can't get gauge sili ci alg term volt, rc=%d\n", i, ic->manu_name, rc);
		break;
	}

	if (rc == 0)
		data->intval = volt;

	return rc;
}

int oplus_mms_gauge_update_sili_ic_alg_dsg_enable(
					struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->deep_spec.sili_ic_alg_dsg_enable;
	return 0;
}

static int mms_gauge_debug_track = 0;
module_param(mms_gauge_debug_track, int, 0644);
MODULE_PARM_DESC(mms_gauge_debug_track, "debug track");
#define TRACK_UPLOAD_COUNT_MAX 3
#define TRACK_LOCAL_T_NS_TO_S_THD 1000000000
#define TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD (24 * 3600)
typedef struct {
	int count;
	int pre_time;
} UploadInfo;

static UploadInfo upload_info[2] = {0};

static bool oplus_mms_gauge_allow_upload_deep_dischg(struct oplus_mms_gauge *chip, bool main_batt)
{
	int curr_time;
	UploadInfo *info;

	if (!chip)
		return false;

	curr_time = local_clock() / TRACK_LOCAL_T_NS_TO_S_THD;
	info = &upload_info[main_batt ? 0 : 1];

	if (curr_time - info->pre_time > TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD)
		info->count = 0;

	if (info->count >= TRACK_UPLOAD_COUNT_MAX)
		return false;

	info->pre_time = curr_time;
	return true;
}

static int oplus_mms_gauge_upload_deep_dischg(struct oplus_mms_gauge *chip, char *deep_msg, bool main_batt)
{
	struct oplus_mms *err_topic;
	struct mms_msg *msg;
	int rc, item, idx;
	const char *batt_type;

	if (!chip)
		return -ENODEV;

	err_topic = oplus_mms_get_by_name("error");
	if (!err_topic) {
		chg_err("error topic not found\n");
		return -ENODEV;
	}

	if (main_batt) {
		item = ERR_ITEM_DEEP_DISCHG_INFO;
		batt_type = "main";
		idx = 0;
	} else {
		item = ERR_ITEM_SUB_DEEP_DISCHG_INFO;
		batt_type = "sub";
		idx = 1;
	}

	msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, item, deep_msg);
	if (!msg) {
		chg_err("alloc %s battery error msg failed\n", batt_type);
		return -ENOMEM;
	}

	rc = oplus_mms_publish_msg(err_topic, msg);
	if (rc < 0) {
		chg_err("publish %s deep dischg msg error, rc=%d\n", batt_type, rc);
		kfree(msg);
	}

	upload_info[idx].count++;

	return rc;
}

static void oplus_gauge_update_deep_dischg(struct oplus_mms_gauge *chip)
{
	union mms_msg_data data = { 0 };
	unsigned long update_delay = 0;
	static int cnts = 0;
	int ui_soc, vbat_min_mv, batt_temp, ibat_ma;
	int rc, i, iterm, vterm, ctime;
	bool charging, low_curr = false, track_check = false;
	int step = 1;
	int temp_region = DDB_CURVE_TEMP_WARM;

	charging = chip->wired_online || chip->wls_online;
	if (charging) {
		cnts = 0;
		return;
	}

	ui_soc = chip->ui_soc;
	rc = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_UI_SOC, &data,
					true);
	if (rc < 0) {
		chg_err("can't get ui_soc, rc=%d\n", rc);
		chip->ui_soc = 0;
	} else {
		chip->ui_soc = data.intval;
	}
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MIN, &data,
				     false);
	if (rc < 0) {
		chg_err("can't get vbat_min, rc=%d\n", rc);
		vbat_min_mv = 0;
	} else {
		vbat_min_mv = data.intval;
	}
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_TEMP, &data,
				     false);
	if (rc < 0) {
		chg_err("can't get batt_temp, rc=%d\n", rc);
		batt_temp = 0;
	} else {
		batt_temp = data.intval;
	}
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CURR, &data,
				     false);
	if (rc < 0) {
		chg_err("can't get ibat_ma, rc=%d\n", rc);
		ibat_ma = 0;
	} else {
		ibat_ma = data.intval;
	}

	if (chip->deep_spec.step_curves.nums) {
		for (i = chip->deep_spec.step_curves.nums - 1; i > 0; i--) {
			if (batt_temp >= chip->deep_spec.step_curves.limits[i].temp)
				break;
		}
		step = chip->deep_spec.step_curves.limits[i].step;
	}

	temp_region = chip->deep_spec.ddbc_tbatt.index_n;
	for (i = 0; i < chip->deep_spec.batt_curves[temp_region].nums; i++) {
		iterm = chip->deep_spec.batt_curves[temp_region].limits[i].iterm;
		vterm = chip->deep_spec.batt_curves[temp_region].limits[i].vterm;
		ctime = chip->deep_spec.batt_curves[temp_region].limits[i].ctime;
		if ((ibat_ma <= iterm) && (vbat_min_mv <= vterm)) {
			low_curr = true;
			break;
		}
	}

	if (low_curr) {
		if (++cnts >= ctime) {
			cnts = 0;
			chip->deep_spec.counts += step;
			track_check = true;
			if (is_support_parallel(chip))
				oplus_gauge_set_deep_dischg_count(chip->gauge_topic_parallel[chip->main_gauge],
								  chip->deep_spec.counts);
			else
				oplus_gauge_set_deep_dischg_count(chip->gauge_topic, chip->deep_spec.counts);
			oplus_gauge_get_ratio_value(chip->gauge_topic);
		} else {
			update_delay = msecs_to_jiffies(5000);
		}
	} else {
		cnts = 0;
		update_delay = msecs_to_jiffies(5000);
	}

	if (track_check || mms_gauge_debug_track) {
		track_check = false;
		mms_gauge_debug_track = 0;
		memset(&(chip->deep_info), 0, sizeof(chip->deep_info));
		chip->deep_info.index += scnprintf(chip->deep_info.msg, DEEP_INFO_LEN,
			"$$track_reason@@deep_dischg$$trange@@%d", temp_region);
		schedule_delayed_work(&chip->deep_track_work, 0);
	}

	if (update_delay > 0)
		schedule_delayed_work(&chip->deep_dischg_work, update_delay);
}

static void oplus_gauge_update_sub_deep_dischg(struct oplus_mms_gauge *chip)
{
	union mms_msg_data data = { 0 };
	unsigned long update_delay = 0;
	static int sub_cnts = 0;
	int sub_vbat_mv, sub_batt_temp, sub_ibat_ma;
	int rc, i, iterm, vterm, sub_ctime;
	bool charging, track_check = false;
	int sub_step = 1;
	bool sub_low_curr = false;
	int temp_region = DDB_CURVE_TEMP_WARM;

	charging = chip->wired_online || chip->wls_online;
	if (charging) {
		sub_cnts = 0;
		return;
	}

	rc = oplus_mms_get_item_data(chip->gauge_topic_parallel[__ffs(chip->sub_gauge)],
				     GAUGE_ITEM_VOL_MIN, &data, false);
	if (rc < 0) {
		chg_err("can't get vbat_min, rc=%d\n", rc);
		sub_vbat_mv = 0;
	} else {
		sub_vbat_mv = data.intval;
	}
	rc = oplus_mms_get_item_data(chip->gauge_topic_parallel[__ffs(chip->sub_gauge)],
				     GAUGE_ITEM_TEMP, &data, false);
	if (rc < 0) {
		chg_err("can't get batt_temp, rc=%d\n", rc);
		sub_batt_temp = 0;
	} else {
		sub_batt_temp = data.intval;
	}
	rc = oplus_mms_get_item_data(chip->gauge_topic_parallel[__ffs(chip->sub_gauge)],
				     GAUGE_ITEM_CURR, &data, false);
	if (rc < 0) {
		chg_err("can't get ibat_ma, rc=%d\n", rc);
		sub_ibat_ma = 0;
	} else {
		sub_ibat_ma = data.intval;
	}

	if (chip->deep_spec.step_curves.nums) {
		for (i = chip->deep_spec.step_curves.nums - 1; i > 0; i--) {
			if (sub_batt_temp >= chip->deep_spec.step_curves.limits[i].temp)
				break;
		}
		sub_step = chip->deep_spec.step_curves.limits[i].step;
	}

	temp_region = chip->deep_spec.ddbc_tbatt.index_n;
	for (i = 0; i < chip->deep_spec.batt_curves[temp_region].nums; i++) {
		iterm = chip->deep_spec.batt_curves[temp_region].limits[i].iterm;
		vterm = chip->deep_spec.batt_curves[temp_region].limits[i].vterm;
		sub_ctime = chip->deep_spec.batt_curves[temp_region].limits[i].ctime;
		if ((sub_ibat_ma <= iterm) && (sub_vbat_mv <= vterm)) {
			sub_low_curr = true;
			break;
		}
	}

	if (sub_low_curr) {
		if (++sub_cnts >= sub_ctime) {
			sub_cnts = 0;
			chip->deep_spec.sub_counts += sub_step;
			track_check = true;
			oplus_gauge_set_deep_dischg_count(chip->gauge_topic_parallel[__ffs(chip->sub_gauge)],
				chip->deep_spec.sub_counts);
			oplus_gauge_get_ratio_value(chip->gauge_topic_parallel[__ffs(chip->sub_gauge)]);
		} else {
			update_delay = msecs_to_jiffies(5000);
		}
	} else {
		sub_cnts = 0;
		update_delay = msecs_to_jiffies(5000);
	}


	if (track_check || mms_gauge_debug_track) {
		track_check = false;
		mms_gauge_debug_track = 0;
		memset(&(chip->deep_info), 0, sizeof(chip->deep_info));
		chip->deep_info.index += scnprintf(chip->deep_info.msg, DEEP_INFO_LEN,
			"$$track_reason@@deep_dischg$$trange@@%d", temp_region);
		schedule_delayed_work(&chip->sub_deep_track_work, 0);
	}

	if (update_delay > 0)
		schedule_delayed_work(&chip->sub_deep_dischg_work, update_delay);
}

void oplus_gauge_deep_dischg_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_gauge *chip =
		container_of(dwork, struct oplus_mms_gauge, deep_dischg_work);

	oplus_gauge_update_deep_dischg(chip);
}

void oplus_gauge_sub_deep_dischg_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_gauge *chip =
		container_of(dwork, struct oplus_mms_gauge, sub_deep_dischg_work);

	oplus_gauge_update_sub_deep_dischg(chip);
}

void oplus_gauge_deep_ratio_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_gauge *chip =
		container_of(dwork, struct oplus_mms_gauge, deep_ratio_work);
	int step_status_switch;

	if (chip->deep_spec.ddrc_strategy_v2)
		step_status_switch = chip->deep_spec.ddrc_temp_switch;
	else
		step_status_switch = DDB_CURVE_TEMP_NORMAL;
	mutex_lock(&chip->deep_spec.lock);
	if ((chip->deep_spec.ddrc_tbatt.index_n >= step_status_switch &&
		chip->deep_spec.ddrc_tbatt.index_p < step_status_switch) ||
		(chip->deep_spec.ddrc_tbatt.index_n < step_status_switch &&
		chip->deep_spec.ddrc_tbatt.index_p >= step_status_switch)) {
		chip->deep_spec.config.step_status = true;
		oplus_gauge_get_ddrc_status(chip->gauge_topic);
		chip->deep_spec.ddrc_tbatt.index_p = chip->deep_spec.ddrc_tbatt.index_n;
	} else if (chip->deep_spec.ddrc_tbatt.index_n >= step_status_switch) {
		oplus_gauge_get_ddrc_status(chip->gauge_topic);
		chip->deep_spec.ddrc_tbatt.index_p = chip->deep_spec.ddrc_tbatt.index_n;
	}
	mutex_unlock(&chip->deep_spec.lock);
	schedule_delayed_work(&chip->deep_track_work, 0);
	if (chip->sub_gauge)
		schedule_delayed_work(&chip->sub_deep_track_work, 0);
}

void oplus_gauge_deep_temp_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_gauge *chip =
		container_of(dwork, struct oplus_mms_gauge, deep_temp_work);
	int tratio_now = DDB_CURVE_TEMP_COLD, tcurve_now = DDB_CURVE_TEMP_COLD;
	int tratio_pre, tcurve_pre;
	bool charging;
	int step_status_switch;
#define TEMP_CNTS 3

	if (!chip->deep_spec.support)
		return;

	if (chip->deep_spec.ddrc_strategy_v2)
		step_status_switch = chip->deep_spec.ddrc_temp_switch;
	else
		step_status_switch = DDB_CURVE_TEMP_NORMAL;

	charging = chip->wired_online || chip->wls_online;
	tratio_now = oplus_gauge_ddrc_get_temp_region(chip);
	tcurve_now = oplus_gauge_ddbc_get_temp_region(chip);
	tratio_pre = chip->deep_spec.ddrc_tbatt.index_n;
	tcurve_pre = chip->deep_spec.ddbc_tbatt.index_n;

	mutex_lock(&chip->deep_spec.lock);
	if (tratio_pre != tratio_now) {
		chip->deep_spec.cnts.ratio++;
		if (chip->deep_spec.cnts.ratio >= TEMP_CNTS) {
			chip->deep_spec.ddrc_tbatt.index_n = tratio_now;
			oplus_gauge_ddrc_temp_thr_init(chip);
			oplus_gauge_ddrc_temp_thr_update(chip, tratio_now, tratio_pre);
		}
	} else {
		chip->deep_spec.cnts.ratio = 0;
		if (chip->deep_spec.ddrc_tbatt.index_n < step_status_switch &&
			chip->deep_spec.ddrc_tbatt.index_p > tratio_now &&
			!charging && (chip->ui_soc < chip->deep_spec.config.soc)) {
			chip->deep_spec.config.step_status = true;
			oplus_gauge_get_ddrc_status(chip->gauge_topic);
			chip->deep_spec.ddrc_tbatt.index_p = tratio_now;
		}
	}

	if (tcurve_pre != tcurve_now) {
		chip->deep_spec.cnts.dischg++;
		if (chip->deep_spec.cnts.dischg >= TEMP_CNTS) {
			oplus_gauge_ddbc_temp_thr_init(chip);
			oplus_gauge_ddbc_temp_thr_update(chip, tcurve_now, tcurve_pre);
			chip->deep_spec.ddbc_tbatt.index_n = tcurve_now;
		}
	} else {
		chip->deep_spec.cnts.dischg = 0;
	}
	mutex_unlock(&chip->deep_spec.lock);

	schedule_delayed_work(&chip->deep_temp_work, msecs_to_jiffies(5000));
}

void oplus_gauge_deep_dischg_check(struct oplus_mms_gauge *chip)
{
	union mms_msg_data data = { 0 };
	bool charging;

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_UI_SOC, &data, false);
	chip->ui_soc = data.intval;

	if (!chip->deep_spec.support)
		return;
	charging = chip->wired_online || chip->wls_online;

	if (!charging && (chip->ui_soc >= chip->deep_spec.config.soc)) {
		schedule_delayed_work(&chip->deep_dischg_work, 0);
		if (chip->sub_gauge)
			schedule_delayed_work(&chip->sub_deep_dischg_work, 0);
		schedule_delayed_work(&chip->deep_ratio_work, 0);
	} else {
		cancel_delayed_work(&chip->deep_dischg_work);
		if (chip->sub_gauge)
			cancel_delayed_work(&chip->sub_deep_dischg_work);
	}
}

void oplus_gauge_deep_id_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_gauge *chip =
		container_of(dwork, struct oplus_mms_gauge, deep_id_work);

	oplus_chg_ic_creat_err_msg(chip->child_list[chip->main_gauge].ic_dev, OPLUS_IC_ERR_BATT_ID, 0, deep_id_info);
	oplus_chg_ic_virq_trigger(chip->child_list[chip->main_gauge].ic_dev, OPLUS_IC_VIRQ_ERR);
}

void oplus_gauge_deep_track_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_gauge *chip =
		container_of(dwork, struct oplus_mms_gauge, deep_track_work);

	int vbat_min_mv, batt_temp, ibat_ma, term_volt;
	int bybid = 0, batt_id = 0;
	int rc;
	union mms_msg_data data = { 0 };
	int dischg_counts, cc, ratio;

	if (!oplus_mms_gauge_allow_upload_deep_dischg(chip, true))
		goto end;

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MIN, &data, false);
	if (rc < 0) {
		chg_err("can't get vbat_min, rc=%d\n", rc);
		vbat_min_mv = 0;
	} else {
		vbat_min_mv = data.intval;
	}
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_TEMP, &data,
				     false);
	if (rc < 0) {
		chg_err("can't get batt_temp, rc=%d\n", rc);
		batt_temp = 0;
	} else {
		batt_temp = data.intval;
	}
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CURR, &data,
				     false);
	if (rc < 0) {
		chg_err("can't get ibat_ma, rc=%d\n", rc);
		ibat_ma = 0;
	} else {
		ibat_ma = data.intval;
	}
	term_volt = get_effective_result(chip->gauge_term_voltage_votable);

	bybid = oplus_wired_get_byb_id_info(chip->wired_topic);
	batt_id = oplus_gauge_get_batt_id_info(chip);
	if (!chip->sub_gauge) {
		chip->deep_info.index += scnprintf(&(chip->deep_info.msg[chip->deep_info.index]),
			DEEP_INFO_LEN - chip->deep_info.index, "$$dischg_counts@@%d$$count_thr@@%d$$count_cali@@%d$$cc@@%d"
			"$$ratio@@%d$$vbat_uv@@%d$$vterm@@%d$$vbat_min@@%d$$tbat@@%d$$ui_soc@@%d$$ibat_ma@@%d$$bybid@@%d"
			"$$batt_id@@%d$$sili_err@@%d", chip->deep_spec.counts, chip->deep_spec.config.count_thr,
			chip->deep_spec.config.count_cali, chip->deep_spec.cc, chip->deep_spec.ratio,
			chip->deep_spec.config.uv_thr, term_volt, vbat_min_mv, batt_temp, chip->ui_soc,
			ibat_ma, bybid, batt_id, chip->deep_spec.sili_err);
	} else {
		if (get_effective_client(chip->target_term_voltage_votable) &&
		    strncmp(get_effective_client(chip->target_term_voltage_votable),
		    SUB_DEEP_COUNT_VOTER, strlen(SUB_DEEP_COUNT_VOTER)) == 0) {
			dischg_counts = chip->deep_spec.sub_counts;
			cc = chip->deep_spec.sub_cc;
			ratio = chip->deep_spec.sub_ratio;
		} else {
			dischg_counts = chip->deep_spec.counts;
			cc = chip->deep_spec.cc;
			ratio = chip->deep_spec.ratio;
		}
		chip->deep_info.index += scnprintf(&(chip->deep_info.msg[chip->deep_info.index]),
			DEEP_INFO_LEN - chip->deep_info.index, "$$dischg_counts@@%d$$count_thr@@%d$$count_cali@@%d$$cc@@%d"
			"$$ratio@@%d$$vbat_uv@@%d$$vterm@@%d$$vbat_min@@%d$$tbat@@%d$$ui_soc@@%d$$ibat_ma@@%d$$bybid@@%d"
			"$$batt_id@@%d$$sili_err@@%d$$main_dischg_counts@@%d$$sub_dischg_counts@@%d$$main_cc@@%d$$sub_cc@@%d"
			"$$main_ratio@@%d$$sub_ratio@@%d",
			dischg_counts, chip->deep_spec.config.count_thr,
			chip->deep_spec.config.count_cali, cc, ratio,
			chip->deep_spec.config.uv_thr, term_volt, vbat_min_mv, batt_temp, chip->ui_soc,
			ibat_ma, bybid, batt_id, chip->deep_spec.sili_err, chip->deep_spec.counts,
			chip->deep_spec.sub_counts, chip->deep_spec.cc, chip->deep_spec.sub_cc,
			chip->deep_spec.ratio, chip->deep_spec.sub_ratio);
	}
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_REG_INFO, &data, true);
	if (rc == 0 && data.strval && strlen(data.strval)) {
		chg_err("[main_gauge_reg_info] %s", data.strval);
		chip->deep_info.index += scnprintf(&(chip->deep_info.msg[chip->deep_info.index]),
			DEEP_INFO_LEN - chip->deep_info.index, "$$maingaugeinfo@@%s", data.strval);
	}

	oplus_mms_gauge_upload_deep_dischg(chip, chip->deep_info.msg, 1);
end:
	memset(&(chip->deep_info), 0, sizeof(chip->deep_info));
}

void oplus_gauge_sub_deep_track_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_gauge *chip =
		container_of(dwork, struct oplus_mms_gauge, sub_deep_track_work);

	int vbat_min_mv, batt_temp, ibat_ma, term_volt;
	int bybid = 0, batt_id = 0;
	int rc;
	union mms_msg_data data = { 0 };
	int dischg_counts, cc, ratio;

	if (!oplus_mms_gauge_allow_upload_deep_dischg(chip, false))
		goto end;

	rc = oplus_mms_get_item_data(chip->gauge_topic_parallel[__ffs(chip->sub_gauge)],
		GAUGE_ITEM_VOL_MIN, &data, false);
	if (rc < 0) {
		chg_err("can't get vbat_min, rc=%d\n", rc);
		vbat_min_mv = 0;
	} else {
		vbat_min_mv = data.intval;
	}
	rc = oplus_mms_get_item_data(chip->gauge_topic_parallel[__ffs(chip->sub_gauge)],
		GAUGE_ITEM_TEMP, &data, false);
	if (rc < 0) {
		chg_err("can't get batt_temp, rc=%d\n", rc);
		batt_temp = 0;
	} else {
		batt_temp = data.intval;
	}
	rc = oplus_mms_get_item_data(chip->gauge_topic_parallel[__ffs(chip->sub_gauge)],
		GAUGE_ITEM_CURR, &data, false);
	if (rc < 0) {
		chg_err("can't get ibat_ma, rc=%d\n", rc);
		ibat_ma = 0;
	} else {
		ibat_ma = data.intval;
	}
	term_volt = get_effective_result(chip->gauge_term_voltage_votable);

	if (get_effective_client(chip->target_term_voltage_votable) &&
	    strncmp(get_effective_client(chip->target_term_voltage_votable),
	    SUB_DEEP_COUNT_VOTER, strlen(SUB_DEEP_COUNT_VOTER)) == 0) {
		dischg_counts = chip->deep_spec.sub_counts;
		cc = chip->deep_spec.sub_cc;
		ratio = chip->deep_spec.sub_ratio;
	} else {
		dischg_counts = chip->deep_spec.counts;
		cc = chip->deep_spec.cc;
		ratio = chip->deep_spec.ratio;
	}
	chip->sub_deep_info.index += scnprintf(&(chip->sub_deep_info.msg[chip->sub_deep_info.index]),
		DEEP_INFO_LEN - chip->sub_deep_info.index, "$$dischg_counts@@%d$$count_thr@@%d$$count_cali@@%d$$cc@@%d$$sub_ratio@@%d"
		"$$vbat_uv@@%d$$vterm@@%d$$vbat_min@@%d$$tbat@@%d$$ui_soc@@%d$$ibat_ma@@%d$$bybid@@%d$$batt_id@@%d$$sili_err@@%d"
		"$$main_dischg_counts@@%d$$sub_dischg_counts@@%d$$main_cc@@%d$$sub_cc@@%d$$main_ratio@@%d$$sub_ratio@@%d",
		dischg_counts, chip->deep_spec.config.count_thr, chip->deep_spec.config.count_cali, cc,
		ratio, chip->deep_spec.config.uv_thr, term_volt, vbat_min_mv, batt_temp,
		chip->ui_soc, ibat_ma, bybid, batt_id, chip->deep_spec.sili_err, chip->deep_spec.counts,
		chip->deep_spec.sub_counts, chip->deep_spec.cc, chip->deep_spec.sub_cc,
		chip->deep_spec.ratio, chip->deep_spec.sub_ratio);

	rc = oplus_mms_get_item_data(chip->gauge_topic_parallel[__ffs(chip->sub_gauge)],
		GAUGE_ITEM_REG_INFO, &data, true);
	if (rc == 0 && data.strval && strlen(data.strval)) {
		chg_err("[sub_gauge_reg_info] %s", data.strval);
		chip->sub_deep_info.index += scnprintf(&(chip->sub_deep_info.msg[chip->sub_deep_info.index]),
			DEEP_INFO_LEN - chip->sub_deep_info.index, "$$subgaugeinfo@@%s", data.strval);
	}

	oplus_mms_gauge_upload_deep_dischg(chip, chip->sub_deep_info.msg, 0);
end:
	memset(&(chip->sub_deep_info), 0, sizeof(chip->sub_deep_info));
}


int oplus_mms_gauge_update_spare_power_enable(
					struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->deep_spec.spare_power_enable;
	return 0;
}


int oplus_mms_gauge_update_vbat_uv(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);
	data->intval = chip->deep_spec.config.uv_thr;
	chg_info("[%d, %d]\n", chip->deep_spec.config.uv_thr, chip->deep_spec.config.count_thr);
	return 0;
}

int oplus_mms_gauge_update_ratio_trange(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	if (!chip  || !chip->deep_spec.support)
		return 0;

	data->intval = chip->deep_spec.ddrc_tbatt.index_n;

	return 0;
}

int oplus_mms_gauge_update_ratio_limit_curr(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int i = 0, limit_curr = 0;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	if (!chip  || !chip->deep_spec.support)
		return -EINVAL;

	chg_info("ratio:%d ; cc:%d", chip->deep_spec.ratio, chip->deep_spec.cc);
	if (chip->deep_spec.limit_curr_curves.nums) {
		for (i = 0; i < chip->deep_spec.limit_curr_curves.nums; i++) {
			if ((chip->deep_spec.ratio <= chip->deep_spec.limit_curr_curves.limits[i].ratio) &&
				(chip->deep_spec.cc <= chip->deep_spec.limit_curr_curves.limits[i].cc)) {
				limit_curr = chip->deep_spec.config.curr_max_ma;
				break;
			} else {
				limit_curr = chip->deep_spec.config.curr_limit_ma;
			}
		}
	} else {
		return -EINVAL;
	}

	data->intval = limit_curr;
	return 0;
}

int oplus_mms_gauge_update_deep_ratio(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	if (strcmp("gauge", mms->desc->name) == 0) {
		data->intval = chip->deep_spec.ratio;
	} else {
		data->intval = chip->deep_spec.sub_ratio;
	}

	return 0;
}

int oplus_mms_gauge_get_si_prop(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int rc = 0;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	data->intval = chip->deep_spec.support;
	return rc;
}

void oplus_mms_gauge_sili_init(struct oplus_mms_gauge *chip)
{
	int step_status_switch;
	int temp_range;

	if (!chip->deep_spec.support)
		return;

	if (chip->deep_spec.ddrc_strategy_v2) {
		step_status_switch = chip->deep_spec.ddrc_temp_switch;
		temp_range = chip->deep_spec.ddrc_temp_num;
	} else {
		step_status_switch = DDB_CURVE_TEMP_NORMAL;
		temp_range = DDB_CURVE_TEMP_WARM;
	}

	vote(chip->gauge_shutdown_voltage_votable, READY_VOTER, true, INVALID_MAX_VOLTAGE, false);
	vote(chip->gauge_term_voltage_votable, READY_VOTER, true, INVALID_MAX_VOLTAGE, false);
	vote(chip->gauge_shutdown_voltage_votable, SPEC_VOTER, true, chip->deep_spec.config.uv_thr, false);

	chip->deep_spec.counts = oplus_gauge_get_deep_dischg_count(chip, chip->gauge_ic);
	if (chip->sub_gauge)
		chip->deep_spec.sub_counts = oplus_gauge_get_deep_dischg_count(chip, chip->gauge_ic_comb[__ffs(chip->sub_gauge)]);
	chip->deep_spec.ddrc_tbatt.index_n = oplus_gauge_ddrc_get_temp_region(chip);
	if (chip->deep_spec.ddrc_tbatt.index_n < step_status_switch)
		chip->deep_spec.ddrc_tbatt.index_p = temp_range;
	else
		chip->deep_spec.ddrc_tbatt.index_p = chip->deep_spec.ddrc_tbatt.index_n;
	chip->deep_spec.ddbc_tbatt.index_n = oplus_gauge_ddbc_get_temp_region(chip);
	oplus_gauge_get_ddrc_status(chip->gauge_topic);

	vote(chip->gauge_term_voltage_votable, READY_VOTER, false, 0, false);
	oplus_gauge_update_three_level_ht_and_term_volt(chip);
	vote(chip->gauge_shutdown_voltage_votable, SUPER_ENDURANCE_MODE_VOTER,
		!chip->super_endurance_mode_status, chip->deep_spec.config.term_voltage, false);
	vote(chip->gauge_shutdown_voltage_votable, READY_VOTER, false, 0, false);

	schedule_delayed_work(&chip->deep_temp_work, msecs_to_jiffies(5000));
}

void oplus_mms_gauge_deep_dischg_init(struct oplus_mms_gauge *chip)
{
	oplus_gauge_deep_dischg_check(chip);
	oplus_gauge_init_sili_status(chip);
	if (chip->deep_spec.support)
		schedule_delayed_work(&chip->deep_id_work, PUSH_DELAY_MS);
}


int oplus_gauge_shutdown_voltage_vote_callback(struct votable *votable, void *data, int volt, const char *client,
						      bool step)
{
	struct oplus_mms_gauge *chip = data;

	if (!chip->deep_spec.support)
		return 0;

	if (volt >= INVALID_MAX_VOLTAGE || volt <= INVALID_MIN_VOLTAGE) {
		chg_info("volt %d invalid, client %s\n", volt, client);
		return 0;
	}

	chg_info("shutdown voltage vote client %s, volt = %d\n", client, volt);
	chip->deep_spec.config.uv_thr = volt;
	return oplus_mms_gauge_push_vbat_uv(chip);
}

int oplus_target_shutdown_voltage_vote_callback(struct votable *votable, void *data, int volt, const char *client,
						      bool step)
{
	struct oplus_mms_gauge *chip = data;

	if (!chip->deep_spec.support)
		return 0;

	if (volt >= INVALID_MAX_VOLTAGE || volt <= INVALID_MIN_VOLTAGE) {
		chg_info("volt %d invalid, client %s\n", volt, client);
		return 0;
	}

	chg_info("target shutdown voltage vote client %s, volt = %d\n", client, volt);
	chip->deep_spec.config.target_uv_thr = volt;
	return 0;
}

static int oplus_mms_gauge_push_fcc_coeff(struct oplus_mms_gauge *chip, int coeff)
{
	struct mms_msg *msg;
	int rc;

	if (!chip->deep_spec.support)
		return 0;

	msg = oplus_mms_alloc_int_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, GAUGE_ITEM_FCC_COEFF, coeff);
	if (msg == NULL) {
		chg_err("alloc battery subboard msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->gauge_topic, msg);
	if (rc < 0) {
		chg_err("publish fcc coeff, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static int oplus_mms_gauge_push_soh_coeff(struct oplus_mms_gauge *chip, int coeff)
{
	struct mms_msg *msg;
	int rc;

	if (!chip->deep_spec.support)
		return 0;

	msg = oplus_mms_alloc_int_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, GAUGE_ITEM_SOH_COEFF, coeff);
	if (msg == NULL) {
		chg_err("alloc battery subboard msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->gauge_topic, msg);
	if (rc < 0) {
		chg_err("publish soh coeff, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

#define DEEP_TERM_VOLT_UPDATE_DELAY_MS 2000
#define DEEP_TERM_VOLT_UPDATE_RETRY_COUNT 5
void oplus_mms_gauge_set_deep_term_volt_work(struct work_struct *work)
{
	int current_volt = 0;
	static int retry_count = 0;
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_gauge *chip =
		container_of(dwork, struct oplus_mms_gauge, set_deep_term_volt_work);

	if (chip == NULL || !chip->deep_spec.support)
		return;

	oplus_mms_gauge_set_deep_term_volt(chip->gauge_topic, chip->deep_spec.config.term_voltage);

	current_volt = oplus_gauge_get_deep_term_volt(chip);
	chg_info("current_volt: %d, term_voltage: %d\n", current_volt, chip->deep_spec.config.term_voltage);

	if (current_volt != chip->deep_spec.config.term_voltage) {
		if (retry_count < DEEP_TERM_VOLT_UPDATE_RETRY_COUNT) {
			retry_count++;
			cancel_delayed_work(&chip->set_deep_term_volt_work);
			schedule_delayed_work(&chip->set_deep_term_volt_work, msecs_to_jiffies(DEEP_TERM_VOLT_UPDATE_DELAY_MS));
		} else {
			retry_count = 0;
			chg_err("deep term voltage update failed, retry count: %d\n", retry_count);
		}
	} else {
		retry_count = 0;
		chg_info("deep term voltage update success, current_volt: %d, term_voltage: %d\n", current_volt, chip->deep_spec.config.term_voltage);
	}

	return;
}

#define DEEP_DISCHG_UPDATE_VOLT_DELTA 100

int oplus_gauge_term_voltage_vote_callback(struct votable *votable, void *data, int volt, const char *client,
						  bool step)
{
	struct oplus_mms_gauge *chip = data;
	int current_volt = 0;
	int gauge_type = 0;
	int i = 0;

	if (!chip->deep_spec.support)
		return 0;

	if (volt >= INVALID_MAX_VOLTAGE || volt <= INVALID_MIN_VOLTAGE) {
		chg_info("volt %d invalid, client %s\n", volt, client);
		return 0;
	}

	current_volt = oplus_gauge_get_deep_term_volt(chip);
	gauge_type = oplus_get_gauge_type();

	for (i = chip->deep_spec.term_coeff_size - 1; i >= 0; i--) {
		if (volt >= chip->deep_spec.term_coeff[i].term_voltage) {
			chip->deep_spec.config.current_fcc_coeff = chip->deep_spec.term_coeff[i].fcc_coeff;
			chip->deep_spec.config.current_soh_coeff = chip->deep_spec.term_coeff[i].soh_coeff;
			break;
		}
	}

	oplus_mms_gauge_push_fcc_coeff(chip, chip->deep_spec.config.current_fcc_coeff);
	oplus_mms_gauge_push_soh_coeff(chip, chip->deep_spec.config.current_soh_coeff);
	chg_info("term voltage vote client %s, volt = %d\n", client, volt);
	chip->deep_spec.config.term_voltage = volt;

	if (gauge_type == GAUGE_TYPE_PLATFORM) {
		cancel_delayed_work_sync(&chip->set_deep_term_volt_work);
		schedule_delayed_work(&chip->set_deep_term_volt_work, msecs_to_jiffies(0));
	} else if (current_volt != volt || step) {
		oplus_mms_gauge_set_deep_term_volt(chip->gauge_topic, volt);
		cancel_delayed_work(&chip->sili_term_volt_effect_check_work);
		schedule_delayed_work(&chip->sili_term_volt_effect_check_work, msecs_to_jiffies(2000));
	}
	return 0;
}

int oplus_target_term_voltage_vote_callback(struct votable *votable, void *data, int volt, const char *client,
						  bool step)
{
	struct oplus_mms_gauge *chip = data;

	if (!chip->deep_spec.support)
		return 0;

	if (volt >= INVALID_MAX_VOLTAGE || volt <= INVALID_MIN_VOLTAGE) {
		chg_info("volt %d invalid, client %s\n", volt, client);
		return 0;
	}

	chg_info("target term voltage vote client %s, volt = %d\n", client, volt);
	chip->deep_spec.config.target_term_voltage = volt;
	return 0;
}

