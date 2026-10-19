// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2023 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[MAXIM]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/time.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/pinctrl/consumer.h>
#include <oplus_chg_ic.h>
#include <oplus_chg_module.h>
#include <oplus_chg_comm.h>
#include <oplus_mms.h>
#include <oplus_mms_gauge.h>
#include <oplus_mms_wired.h>
#include <asm/setup.h>
#include "oplus_ds28e30/1wire_protocol.h"
#include "oplus_ds28e30/ds28e30.h"

static char __oplus_chg_cmdline[COMMAND_LINE_SIZE];
static char *oplus_chg_cmdline = __oplus_chg_cmdline;

#define BATT_SN_NUM_LEN				12
#define BATT_NUM_MAX				10
#define MAX_SN_NUM_SIZE		BATT_NUM_MAX * BATT_SN_NUM_LEN

#define AUTH_MESSAGE_LEN			20
#define OPLUS_MAXIM_AUTH_TAG		"maxim_auth="
#define OPLUS_MAXIM_AUTH_SUCCESS	"maxim_auth=TRUE"
#define OPLUS_MAXIM_AUTH_FAILED		"maxim_auth=FALSE"
#define OPLUS_MAXIM_MAX_RETRY		15

struct oplus_maxim_gauge_chip {
	bool support_maxim_in_lk;
	bool maxim_in_kernel_init_ok;
	struct device *dev;
	struct oplus_chg_ic_dev *ic_dev;
	bool authenticate_result;
	bool boot_lk_auth_result;
	bool boot_kernel_auth_result;
	struct pinctrl *pinctrl;
	struct pinctrl_state *maxim_active;
	int data_gpio;
	struct onewire_gpio_data gpio_info;
	unsigned char sn_num[BATT_NUM_MAX][BATT_SN_NUM_LEN];
	int batt_info_num;
	struct delayed_work maxim_err_track_work;
	bool support_maxim_in_kernel;
};

static const char *oplus_maxim_get_cmdline(void)
{
	struct device_node * of_chosen = NULL;
	char *maxim_auth = NULL;

	if (__oplus_chg_cmdline[0] != 0)
		return oplus_chg_cmdline;

	of_chosen = of_find_node_by_path("/chosen");
	if (of_chosen) {
		maxim_auth = (char *)of_get_property(
					of_chosen, "maxim_auth", NULL);
		if (!maxim_auth)
			chg_err("%s: failed to get maxim_auth\n", __func__);
		else {
			strcpy(__oplus_chg_cmdline, maxim_auth);
			chg_info("%s: maxim_auth: %s\n", __func__, maxim_auth);
		}
	} else {
		chg_err("%s: failed to get /chosen \n", __func__);
	}

	return oplus_chg_cmdline;
}

static bool oplus_maxim_check_auth_msg(void)
{
	bool ret = false;
	char *str = NULL;

	if (NULL == oplus_maxim_get_cmdline()) {
		chg_err("oplus_chg_check_auth_msg: cmdline is NULL!!!\n");
		return false;
	}

	str = strstr(oplus_maxim_get_cmdline(), OPLUS_MAXIM_AUTH_TAG);
	if (str == NULL) {
		chg_err("oplus_chg_check_auth_msg: Asynchronous authentication is not supported!!!\n");
		return false;
	}

	pr_info("oplus_chg_check_auth_msg: %s\n", str);
	if (0 == memcmp(str, OPLUS_MAXIM_AUTH_SUCCESS, sizeof(OPLUS_MAXIM_AUTH_SUCCESS))) {
		ret = true;
		pr_info("oplus_chg_check_auth_msg: %s\n", OPLUS_MAXIM_AUTH_SUCCESS);
	} else {
		ret = false;
		pr_info("oplus_chg_check_auth_msg: %s\n", OPLUS_MAXIM_AUTH_FAILED);
	}

	return ret;
}

static int oplus_maxim_parse_dt(struct oplus_maxim_gauge_chip *chip)
{
	int rc, len, i, j;
	struct device_node *node = oplus_get_node_by_child_gauge(chip->dev->of_node);
	unsigned char sn_num_total[MAX_SN_NUM_SIZE] = {0};

	chip->maxim_in_kernel_init_ok = false;
	chip->support_maxim_in_lk = of_property_read_bool(node, "support_encryption_in_lk");
	chg_info("support_maxim_in_lk: %d\n", chip->support_maxim_in_lk);

	chip->support_maxim_in_kernel = of_property_read_bool(node, "support_encryption_in_kernel");
	chg_info("support_maxim_in_kernel: %d\n", chip->support_maxim_in_kernel);

	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		chg_err("get pinctrl fail\n");
		return -ENODEV;
	}
	chip->maxim_active = pinctrl_lookup_state(chip->pinctrl, "maxim_active");
	if (IS_ERR_OR_NULL(chip->maxim_active)) {
		chg_err(": %d Failed to get the maxim_active pinctrl handle\n", __LINE__);
		return -ENODEV;
	} else {
		pinctrl_select_state(chip->pinctrl, chip->maxim_active);
		chg_err(": %d set maxim_active pinctrl handle\n", __LINE__);
	}
	chip->data_gpio = of_get_named_gpio(node, "data-gpio", 0);
	if (chip->data_gpio < 0) {
		chg_err("maxim data_gpio not specified\n");
		return -1;
	} else {
		chg_err("chip->data_gpio %d\n", chip->data_gpio);
		if (gpio_is_valid(chip->data_gpio)) {
			rc = gpio_request(chip->data_gpio, "maxim-data-gpio");
			if (rc) {
				chg_err("unable to request gpio [%d]\n", chip->data_gpio);
				return -1;
			}
		} else {
			chg_err("maxim data_gpio invalid\n");
			return -1;
		}
	}

	rc = of_property_read_u32(node, "gpio-addr-set-out", &chip->gpio_info.onewire_gpio_cfg_addr_out);
	if (rc) {
		chg_err("maxim get gpio-addr-set-out failed %d\n", rc);
		return -1;
	} else {
		chg_info("cfg_addr_out 0x%x\n", chip->gpio_info.onewire_gpio_cfg_addr_out);
	}
	rc = of_property_read_u32(node, "gpio-addr-set-in", &chip->gpio_info.onewire_gpio_cfg_addr_in);
	if (rc) {
		chg_err("maxim get gpio-addr-set-in failed %d\n", rc);
		return -1;
	} else {
		chg_info("cfg_addr_in 0x%x\n", chip->gpio_info.onewire_gpio_cfg_addr_in);
	}
	rc = of_property_read_u32(node, "gpio-addr-level_high", &chip->gpio_info.onewire_gpio_level_addr_high);
	if (rc) {
		chg_err("maxim get gpio-addr-level_high failed %d\n", rc);
		return -1;
	} else {
		chg_info("gpio_level_addr_high 0x%x\n", chip->gpio_info.onewire_gpio_level_addr_high);
	}
	rc = of_property_read_u32(node, "gpio-addr-level_low", &chip->gpio_info.onewire_gpio_level_addr_low);
	if (rc) {
		chg_err("maxim get gpio-addr-level_low failed %d\n", rc);
		return -1;
	} else {
		chg_info("gpio_level_addr_low 0x%x\n", chip->gpio_info.onewire_gpio_level_addr_low);
	}
	rc = of_property_read_u32(node, "gpio-addr-data-in", &chip->gpio_info.onewire_gpio_in_addr);
	if (rc) {
		chg_err("maxim get gpio-addr-data-in failed %d\n", rc);
		return -1;
	} else {
		chg_info("gpio_in_addr 0x%x\n", chip->gpio_info.onewire_gpio_in_addr);
	}

	rc = of_property_read_u32(node, "gpio-addr-offset", &chip->gpio_info.gpio_addr_offset);
	if (rc) {
		chg_err("maxim get gpio-addr-offset failed %d\n", rc);
		return -1;
	} else {
		chg_info("gpio-addr-offset 0x%x\n", chip->gpio_info.gpio_addr_offset);
	}

	rc = of_property_read_u32(node, "gpio-set-out-val", &chip->gpio_info.onewire_gpio_cfg_out_val);
	if (rc) {
		chip->gpio_info.onewire_gpio_cfg_out_val = (0x1 << chip->gpio_info.gpio_addr_offset);
	}
	chg_err("gpio-set-out-val 0x%x\n", chip->gpio_info.onewire_gpio_cfg_out_val);

	rc = of_property_read_u32(node, "gpio-set-in-val", &chip->gpio_info.onewire_gpio_cfg_in_val);
	if (rc) {
		chip->gpio_info.onewire_gpio_cfg_in_val = (0x1 << chip->gpio_info.gpio_addr_offset);
	}
	chg_err("gpio-set-in-val 0x%x\n", chip->gpio_info.onewire_gpio_cfg_in_val);

	rc = of_property_read_u32(node, "gpio_level_high_val", &chip->gpio_info.onewire_gpio_level_high_val);
	if (rc) {
		chip->gpio_info.onewire_gpio_level_high_val = (0x1 << chip->gpio_info.gpio_addr_offset);
	}
	chg_err("gpio_level_high_val 0x%x\n", chip->gpio_info.onewire_gpio_level_high_val);

	rc = of_property_read_u32(node, "gpio_level_low_val", &chip->gpio_info.onewire_gpio_level_low_val);
	if (rc) {
		chip->gpio_info.onewire_gpio_level_low_val = (0x1 << chip->gpio_info.gpio_addr_offset);
	}
	chg_err("gpio_level_low_val 0x%x\n", chip->gpio_info.onewire_gpio_level_low_val);

	rc = of_property_read_u32(node, "write_begin_low_level_time", &chip->gpio_info.write_begin_low_level_time);
	if (rc) {
		chip->gpio_info.write_begin_low_level_time = 1000;
	}
	chg_info("write_begin_low_level_time %d\n", chip->gpio_info.write_begin_low_level_time);

	rc = of_property_read_u32(node, "write_relese_ic_time", &chip->gpio_info.write_relese_ic_time);
	if (rc) {
		chip->gpio_info.write_relese_ic_time = 5;
	}
	chg_info("write_relese_ic_time %d\n", chip->gpio_info.write_relese_ic_time);

	chip->gpio_info.maxim_romid_crc_support = of_property_read_bool(node, "oplus,maxim_romid_crc_support");
	chg_info("maxim_romid_crc_support %d\n", chip->gpio_info.maxim_romid_crc_support);

	rc = of_property_read_u32(node, "maxim_trl_ndelay", &chip->gpio_info.maxim_trl_ndelay);
	if (rc) {
		chip->gpio_info.maxim_trl_ndelay = 0;
	}
	chg_info("maxim_trl_ndelay %d\n", chip->gpio_info.maxim_trl_ndelay);

	chip->maxim_in_kernel_init_ok = true;
	chg_info("maxim_in_kernel_init_ok: %d\n", chip->maxim_in_kernel_init_ok);

	len = of_property_count_u8_elems(node, "oplus,batt_info");
	if (len < 0 || len > MAX_SN_NUM_SIZE) {
		chg_info("Count oplus,batt_info failed, rc = %d\n", len);
		return -1;
	}

	rc = of_property_read_u8_array(node, "oplus,batt_info", sn_num_total,
		len > MAX_SN_NUM_SIZE ? MAX_SN_NUM_SIZE : len);
	if (rc) {
		chg_err("maxim get oplus,batt_info failed %d\n", rc);
		return -1;
	}

	chip->batt_info_num = len / BATT_SN_NUM_LEN;
	for (i = 0; i < chip->batt_info_num; i++) {
		memcpy(chip->sn_num[i], &sn_num_total[i * BATT_SN_NUM_LEN], BATT_SN_NUM_LEN);
		chg_info("parse oplus,batt_info, batt_info_%d: \n", i);
		for (j = 0; j < BATT_SN_NUM_LEN; j++) {
			chg_info(" %x", chip->sn_num[i][j]);
		}
	}

	return 0;
}

static int oplus_maxim_guage_init(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -EAGAIN;
	}

	ic_dev->online = true;

	chg_info("oplus_maxim_guage_init, ic_dev->online = %d\n", ic_dev->online);
	return 0;
}

static int oplus_maxim_guage_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (!ic_dev->online)
		return 0;

	ic_dev->online = false;
	chg_info("oplus_maxim_guage_exit, ic_dev->online = %d\n", ic_dev->online);

	return 0;
}

static DEFINE_MUTEX(init_mutex);
static struct oplus_maxim_gauge_chip *maxim_chip;
static int onewire_init_gpio_info(struct oplus_maxim_gauge_chip *maxim_chip)
{
	int ret;
	mutex_lock(&init_mutex);

	maxim_chip->gpio_info.gpio_cfg_out_reg = devm_ioremap(maxim_chip->dev,
			maxim_chip->gpio_info.onewire_gpio_cfg_addr_out, 0x4);
	maxim_chip->gpio_info.gpio_cfg_in_reg = devm_ioremap(maxim_chip->dev,
			maxim_chip->gpio_info.onewire_gpio_cfg_addr_in, 0x4);
	maxim_chip->gpio_info.gpio_out_high_reg = devm_ioremap(maxim_chip->dev,
			maxim_chip->gpio_info.onewire_gpio_level_addr_high, 0x4);
	maxim_chip->gpio_info.gpio_out_low_reg = devm_ioremap(maxim_chip->dev,
			maxim_chip->gpio_info.onewire_gpio_level_addr_low, 0x4);
	maxim_chip->gpio_info.gpio_in_reg = devm_ioremap(maxim_chip->dev,
			maxim_chip->gpio_info.onewire_gpio_in_addr, 0x4);

	ret = onewire_init(&maxim_chip->gpio_info);
	mutex_unlock(&init_mutex);
	return ret;
}

static int oplus_maxim_guage_get_batt_auth(struct oplus_chg_ic_dev *ic_dev, bool *pass)
{
	struct oplus_maxim_gauge_chip *chip;
	bool flag;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (chip == NULL) {
		chg_err("maxim chip is NULL");
		return -ENODEV;
	}

	if (chip->authenticate_result == false && chip->maxim_in_kernel_init_ok) {
		onewire_set_gpio_config_out();
		flag = authenticate_ds28e30(chip->sn_num, chip->batt_info_num, 0);
		if (flag == true) {
			chg_info("%s: re Authenticated flag %d succ\n", __func__, flag);
			chip->authenticate_result = true;
		}
		onewire_set_gpio_config_in();
	}
	*pass = chip->authenticate_result;
	chg_err("%s auth = %d\n", __func__, *pass);

	return 0;
}

#define OPLUS_MAXIM_MAX_SOH_RETRY		5
static int oplus_maxim_guage_set_batt_histsoh_data(struct oplus_chg_ic_dev *ic_dev, const int *buf)
{
	struct oplus_maxim_gauge_chip *chip;
	bool flag = true;
	int i;
	int ret;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (buf == NULL) {
		chg_err("oplus_chg_ic_dev buf is NULL");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (chip == NULL) {
		chg_err("maxim chip is NULL");
		return -ENODEV;
	}

	if (chip->authenticate_result == true && chip->maxim_in_kernel_init_ok &&
		chip->support_maxim_in_kernel == true) {
		ret = onewire_init_gpio_info(maxim_chip);
		if (ret < 0) {
			chg_err("onewire_init failed, ret=%d\n", ret);
			return -ENODEV;
		}

		for (i = 0; i < OPLUS_MAXIM_MAX_SOH_RETRY; i++) {
			flag = set_historic_soh_data_ds28e30(buf);
			if (flag == true) {
				chg_info(": re set_historic_soh_data_ds28e30 flag %d succ\n", flag);
				break;
			}
		}
		onewire_set_gpio_config_in();
	}

	return 0;
}

static int oplus_maxim_guage_get_batt_histsoh_data(struct oplus_chg_ic_dev *ic_dev, int *buf, int len)
{
	struct oplus_maxim_gauge_chip *chip;
	bool flag;
	int i;
	int ret;

	if (ic_dev == NULL || buf == NULL) {
		chg_err(" oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (chip == NULL) {
		chg_err(" maxim chip is NULL");
		return -ENODEV;
	}

	if (chip->authenticate_result == true && chip->maxim_in_kernel_init_ok &&
		chip->support_maxim_in_kernel == true) {
		ret = onewire_init_gpio_info(maxim_chip);
		if (ret < 0) {
			chg_err(" onewire_init failed, ret=%d\n", ret);
			return -ENODEV;
		}

		for (i = 0; i < OPLUS_MAXIM_MAX_SOH_RETRY; i++) {
			flag = get_historic_soh_data_ds28e30(buf, len);
			if (flag == true) {
				chg_info(": re get_historic_soh_data_ds28e30 flag %d succ\n", flag);
				break;
			}
		}
		onewire_set_gpio_config_in();
	}

	return 0;
}

static int oplus_maxim_guage_get_batt_sn(struct oplus_chg_ic_dev *ic_dev, char *buf, int len)
{
	struct oplus_maxim_gauge_chip *chip;
	bool flag;
	int i;
	int ret;

	if (ic_dev == NULL || buf == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (chip == NULL) {
		chg_err("maxim chip is NULL");
		return -ENODEV;
	}

	if (chip->authenticate_result == true && chip->support_maxim_in_kernel == true &&
		chip->maxim_in_kernel_init_ok) {
		ret = onewire_init_gpio_info(maxim_chip);
		if (ret < 0) {
			chg_err(" onewire_init failed, ret=%d\n", ret);
			return -ENODEV;
		}

		for (i = 0; i < OPLUS_MAXIM_MAX_SOH_RETRY; i++) {
			flag = get_sn_ds28e30(buf);
			if (flag == true) {
				chg_info(": re get_sn_ds28e30 flag %d succ\n", flag);
				break;
			}
		}
		onewire_set_gpio_config_in();
	}
	chg_info(" get_batt_sn = %s\n", buf);

	return 0;
}

#define MAX_RETRY_CNT 3
static int oplus_maxim_gauge_get_battinfo_sn(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	struct oplus_maxim_gauge_chip *chip;
	int sn_len = 0;
	int rc;
	int retry_cnt = MAX_RETRY_CNT;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (buf == NULL || len < MAXIM_BATT_SN_LEN)
		return -EINVAL;

	if (!chip->maxim_in_kernel_init_ok)
		return -ENODEV;

	rc = onewire_init_gpio_info(chip);
	if (rc < 0) {
		chg_err("onewire_init_gpio_info failed\n");
		return -ENODEV;
	}

	while (!get_batt_sn_ds28e30(buf, &sn_len) && retry_cnt > 0)
		retry_cnt--;

	if (retry_cnt == 0) {
		chg_err("get battery sn failed\n");
		return -EINVAL;
	}

	onewire_set_gpio_config_in();

	return sn_len;
}

static void *oplus_chg_get_func(struct oplus_chg_ic_dev *ic_dev,
				enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT,
					       oplus_maxim_guage_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT,
					       oplus_maxim_guage_exit);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_AUTH:
		func = OPLUS_CHG_IC_FUNC_CHECK(
			OPLUS_IC_FUNC_GAUGE_GET_BATT_AUTH,
			oplus_maxim_guage_get_batt_auth);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_HMAC:
		func = OPLUS_CHG_IC_FUNC_CHECK(
			OPLUS_IC_FUNC_GAUGE_GET_BATT_HMAC,
			oplus_maxim_guage_get_batt_auth);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_BATT_HISTSOH_DATA:
		func = OPLUS_CHG_IC_FUNC_CHECK(
			OPLUS_IC_FUNC_GAUGE_SET_BATT_HISTSOH_DATA,
			oplus_maxim_guage_set_batt_histsoh_data);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_HISTSOH_DATA:
		func = OPLUS_CHG_IC_FUNC_CHECK(
			OPLUS_IC_FUNC_GAUGE_GET_BATT_HISTSOH_DATA,
			oplus_maxim_guage_get_batt_histsoh_data);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_IC_SN:
		func = OPLUS_CHG_IC_FUNC_CHECK(
			OPLUS_IC_FUNC_GAUGE_GET_BATT_IC_SN,
			oplus_maxim_guage_get_batt_sn);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_SN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_SN,
			oplus_maxim_gauge_get_battinfo_sn);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}
struct oplus_chg_ic_virq maxim_guage_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
	{ .virq_id = OPLUS_IC_VIRQ_RESUME },
};

static void oplus_chg_maxim_err_track_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_maxim_gauge_chip *chip = container_of(dwork,
		struct oplus_maxim_gauge_chip, maxim_err_track_work);

	if (chip->support_maxim_in_lk && !chip->boot_lk_auth_result) {
		oplus_chg_ic_creat_err_msg(
				chip->ic_dev, OPLUS_IC_ERR_GAUGE, 0,
				"batt auth lk err[%d]",
				chip->boot_lk_auth_result);
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_ERR);
	}
	if (!chip->boot_lk_auth_result &&!chip->boot_kernel_auth_result) {
		oplus_chg_ic_creat_err_msg(
				chip->ic_dev, OPLUS_IC_ERR_GAUGE, 0,
				"batt auth kernel err[%d]",
				chip->boot_kernel_auth_result);
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_ERR);
	}
}

static int oplus_maxim_probe(struct platform_device *pdev)
{
	int ret;
	int retry = 0;
	bool flag;
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	struct oplus_chg_ic_cfg ic_cfg = { 0 };

	chg_info("%s: entery\n", __func__);
	maxim_chip = devm_kzalloc(&pdev->dev, sizeof(struct oplus_maxim_gauge_chip), GFP_KERNEL);
	if (!maxim_chip) {
		chg_err("Failed to allocate memory\n");
		return -ENOMEM;
	}
	maxim_chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, maxim_chip);
	oplus_maxim_parse_dt(maxim_chip);
	INIT_DELAYED_WORK(&maxim_chip->maxim_err_track_work, oplus_chg_maxim_err_track_work);
	maxim_chip->authenticate_result = false;
	maxim_chip->boot_lk_auth_result = false;
	maxim_chip->boot_kernel_auth_result = false;
	flag = oplus_maxim_check_auth_msg();

	chg_info("oplus_maxim_probe support_maxim_in_lk=%d,support_in_kernel=%d,oplus_maxim_check_auth_msg=%d\n",
		maxim_chip->support_maxim_in_lk, maxim_chip->support_maxim_in_kernel, flag);

	if (maxim_chip->support_maxim_in_lk && flag) {
		chg_info("%s get lk auth success .\n", __func__);
		ret = true;

		/*LK already auth success, Kernel not auth again.*/
		maxim_chip->authenticate_result = true;
		maxim_chip->boot_lk_auth_result = true;
	} else {
		chg_info("%s: lk auth  failed\n", __func__);
		if (maxim_chip->maxim_in_kernel_init_ok == false) {
			chg_err("%s: not support kernel auth\n", __func__);
			return 0;
		}

		ret = onewire_init_gpio_info(maxim_chip);
		if (ret < 0) {
			chg_err("onewire_init failed, ret=%d\n", ret);
			maxim_chip->maxim_in_kernel_init_ok = false;
		} else {
			while (retry < OPLUS_MAXIM_MAX_RETRY) {
				mdelay(100);
				flag = authenticate_ds28e30(maxim_chip->sn_num, maxim_chip->batt_info_num, 0);
				if (flag == true) {
					chg_info("%s: Authenticated flag %d succ\n", __func__, flag);
					maxim_chip->authenticate_result = true;
					maxim_chip->boot_kernel_auth_result = true;
					break;
				}
				retry++;
			}
			onewire_set_gpio_config_in();
		}
		chg_info("%s: Authenticated flag retry =  %d\n", __func__, retry);
	}
	ret = of_property_read_u32(maxim_chip->dev->of_node, "oplus,ic_type", &ic_type);
	if (ret < 0) {
		chg_err("can't get ic type, ret=%d\n", ret);
		goto error;
	}
	ret = of_property_read_u32(maxim_chip->dev->of_node, "oplus,ic_index", &ic_index);
	if (ret < 0) {
		chg_err("can't get ic index, ret=%d\n", ret);
		goto error;
	}
	ic_cfg.name = maxim_chip->dev->of_node->name;
	snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "gauge-maxim");
	ic_cfg.type = ic_type;
	ic_cfg.index = ic_index;
	ic_cfg.get_func = oplus_chg_get_func;
	ic_cfg.virq_data = maxim_guage_virq_table;
	ic_cfg.virq_num = ARRAY_SIZE(maxim_guage_virq_table);
	ic_cfg.of_node = maxim_chip->dev->of_node;
	maxim_chip->ic_dev = devm_oplus_chg_ic_register(maxim_chip->dev, &ic_cfg);
	if (!maxim_chip->ic_dev) {
		ret = -ENODEV;
		chg_err("register %s error\n", maxim_chip->dev->of_node->name);
		goto error;
	}
	schedule_delayed_work(&maxim_chip->maxim_err_track_work, msecs_to_jiffies(10000));
	chg_info("register %s, oplus_maxim_probe sucess\n", maxim_chip->dev->of_node->name);
	return 0;

error:
	if (maxim_chip)
		devm_kfree(&pdev->dev, maxim_chip);

	chg_err("oplus_maxim_probe fail :%d\n", ret);
	return ret;
}

static void oplus_maxim_shutdown(struct platform_device *pdev)
{
	return;
}

static const struct of_device_id of_oplus_maxim_match[] = {
	{ .compatible = "oplus-maxim", },
	{},
};
MODULE_DEVICE_TABLE(of, of_oplus_maxim_match);

static struct platform_driver oplus_maxim_driver = {
	.probe		= oplus_maxim_probe,
	.shutdown	= oplus_maxim_shutdown,
	.driver		= {
		.name	= "oplus-maxim",
		.of_match_table = of_oplus_maxim_match,
	},
};

static __init int oplus_maxim_driver_init(void)
{
	int ret;
	chg_info("%s: start\n", __func__);
	ret = platform_driver_register(&oplus_maxim_driver);
	return ret;
}

static __exit void oplus_maxim_driver_exit(void)
{
	platform_driver_unregister(&oplus_maxim_driver);
}

oplus_chg_module_register(oplus_maxim_driver);

MODULE_DESCRIPTION("oplus maxim driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:oplus-maxim");
