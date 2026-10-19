// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021-2021 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[VIRTUAL_BUCK]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/slab.h>
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
#include <linux/regmap.h>
#include <linux/list.h>
#include <linux/pinctrl/consumer.h>
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/system/boot_mode.h>
#include <soc/oplus/device_info.h>
#include <soc/oplus/system/oplus_project.h>
#endif
#include <oplus_chg_module.h>
#include <oplus_chg_ic.h>
#include <linux/nvmem-consumer.h>
#include <oplus_chg_monitor.h>

#include "test-kit.h"

#include <oplus_chg_comm.h>
#if IS_ENABLED(CONFIG_HORAE_FLASH_LED_THERMAL)
#include <linux/thermal.h>
#include <soc/oplus/horae_flash_led_temp.h>
#define NTC_SUBBOARD_TEMP_COUNT  292
#endif

#define DISCONNECT			0
#define STANDARD_TYPEC_DEV_CONNECT	BIT(0)
#define OTG_DEV_CONNECT			BIT(1)

struct oplus_virtual_buck_child {
	struct oplus_chg_ic_dev *ic_dev;
	int index;
	int current_ratio;
	enum oplus_chg_ic_func *funcs;
	int func_num;
	enum oplus_chg_ic_virq_id *virqs;
	int virq_num;
};

struct oplus_vc_misc_gpio {
	struct pinctrl		*pinctrl;
	struct mutex		pinctrl_mutex;

	int			ship_gpio;
	struct pinctrl_state	*ship_active;
	struct pinctrl_state	*ship_sleep;

	int			vchg_trig_gpio;
	struct pinctrl_state	*vchg_trig_default;

	int			ccdetect_gpio;
	struct pinctrl_state	*ccdetect_active;
	struct pinctrl_state	*ccdetect_sleep;

	int dischg_gpio;
	bool is_dischg_gpio_request;
	struct pinctrl_state *dischg_enable;
	struct pinctrl_state *dischg_disable;
};

#if IS_ENABLED(CONFIG_HORAE_FLASH_LED_THERMAL)
struct oplus_ntc_switch {
	bool ntc_switch_usbtemp_l;
	bool ntc_switch_usbtemp_r;
	bool ntc_switch_subboard;
	int subboard_temp_table[NTC_SUBBOARD_TEMP_COUNT];
};
#endif

struct oplus_virtual_buck_ic {
	struct device *dev;
	struct oplus_chg_ic_dev *ic_dev;
	bool online;
	enum oplus_chg_ic_connect_type connect_type;
	int child_num;
	struct oplus_virtual_buck_child *child_list;
	struct delayed_work upload_device_id_work;

#if IS_ENABLED(CONFIG_HORAE_FLASH_LED_THERMAL)
	struct oplus_ntc_switch ntc_switch_gpio;
#endif

	/* parallel charge */
	int main_charger;
	int aux_charger_en_thr_ma;

	struct iio_channel *usbtemp_adc_l;
	struct iio_channel *usbtemp_adc_r;

	struct oplus_vc_misc_gpio misc_gpio;

	int ccdetect_irq;

	bool otg_switch;
	struct nvmem_cell	*soc_backup_nvmem;
	int usbtemp_conversion_ratio;

#if IS_ENABLED(CONFIG_OPLUS_CHG_TEST_KIT)
	struct test_feature *uart_gpio_test;
	struct test_feature *typec_port_test;
#endif
	struct oplus_mms *error_topic;
	struct mms_subscribe *error_subs;
	struct work_struct buck_fpga_rst_work;
	struct work_struct buck_i2c_rst_work;
	int fpga_reset_status;
	int i2c_reset_status;
};

#if IS_ENABLED(CONFIG_OPLUS_CHG_TEST_KIT)
enum situations_type {
	SITUATION_DEFAULT = 0,
	SITUATION_IDLE,
	SITUATION_OTG,
	SITUATION_CHARGING
};

struct test_kit_typec_port_info g_typec_port_info[] = {
	{
		.name = "typec_port_idle",
		.case_num = 1,
		.status = TYPEC_PORT_ROLE_SNK,
		.situation = SITUATION_IDLE,
	},
	{}
};

const struct test_feature_cfg g_typec_port_test_cfg = {
	.name = "typec_port_test",
	.test_info = (void *)g_typec_port_info,
	.test_func = test_kit_typec_port_test,
};

#if IS_ENABLED(CONFIG_HORAE_FLASH_LED_THERMAL)
static int oplus_ntc_convert(struct oplus_virtual_buck_ic *chip, int volt)
{
	int i;
	int size;

	size = ARRAY_SIZE(chip->ntc_switch_gpio.subboard_temp_table) - 1;
	if (size <= 0)
		return NTC_TEMP_DEFAULT;

	for (i = 1; i <= size; (i = i + 2)) {
		if (i >= size || chip->ntc_switch_gpio.subboard_temp_table[i] <= volt)
			break;
	}
	if (i >= size)
		i = size;
	i--;

	return chip->ntc_switch_gpio.subboard_temp_table[i] * NTC_TEMP_CONVERSION;
}

static int oplus_get_subboard_temp(struct thermal_zone_device *tz, int *temp)
{
	int subboard_volt = 0;
	int rc;
	bool ret;
	struct oplus_virtual_buck_ic *chip;

	chip = (struct oplus_virtual_buck_ic *)tz->devdata;
	if (chip) {
		ret = oplus_set_ntc_switch_lock(SUBBOARD, true);
		if (ret) {
			mdelay(10);
			rc = iio_read_channel_processed(chip->usbtemp_adc_l, &subboard_volt);
			ret = oplus_set_ntc_switch_lock(SUBBOARD, false);
			if (rc >= 0) {
				subboard_volt /= NTC_TEMP_CONVERSION;
				*temp = oplus_ntc_convert(chip, subboard_volt);
			} else {
				*temp = NTC_TEMP_DEFAULT;
				chg_info("iio_read fail\n");
			}
		} else {
			*temp = NTC_TEMP_DEFAULT;
			chg_info("ntc switch fail\n");
		}
	} else {
		*temp = NTC_TEMP_DEFAULT;
	}
	chg_info("subboard_volt:%d, temp:%d,\n", subboard_volt, *temp);
	return 0;
}

static struct thermal_zone_device_ops subboard_temp_ops = {
	.get_temp = oplus_get_subboard_temp,
};

static void register_subboard_thermal(struct oplus_virtual_buck_ic *chip)
{
	int ret = 0;
	struct thermal_zone_device *tz_dev;

	if (IS_ERR_OR_NULL(&chip->ntc_switch_gpio))
		return;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
	tz_dev = thermal_tripless_zone_device_register("subboard_temp", chip, &subboard_temp_ops, NULL);
#else
	tz_dev = thermal_zone_device_register("subboard_temp", 0, 0, chip, &subboard_temp_ops, NULL, 0, 0);
#endif
	if (IS_ERR(tz_dev)) {
		chg_err("subboard_temp register fail");
		return;
	}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
	ret = thermal_zone_device_enable(tz_dev);
	if (ret) {
		thermal_zone_device_unregister(tz_dev);
		return;
	}
#endif
	chg_info("register_subboard_thermal success\n");
}

static void oplus_ntc_switch_init(struct oplus_virtual_buck_ic *chip)
{
	int rc;

	if (!chip) {
		chg_err("chip not ready!\n");
		return;
	}
	chip->ntc_switch_gpio.ntc_switch_usbtemp_l = of_property_read_bool(chip->dev->of_node, "oplus,ntc_switch_usbtemp_l");
	chip->ntc_switch_gpio.ntc_switch_usbtemp_r = of_property_read_bool(chip->dev->of_node, "oplus,ntc_switch_usbtemp_r");
	chip->ntc_switch_gpio.ntc_switch_subboard = of_property_read_bool(chip->dev->of_node, "oplus,ntc_switch_subboard");
	if (chip->ntc_switch_gpio.ntc_switch_subboard) {
		rc = read_signed_data_from_node(chip->dev->of_node, "chg_ntc_para", (s32 *)chip->ntc_switch_gpio.subboard_temp_table,
						NTC_SUBBOARD_TEMP_COUNT);
		if (rc >= 0)
			register_subboard_thermal(chip);
	}
}
#endif

bool test_kit_typec_port_check(void *info, char *buf, size_t len, size_t *use_size)
{
	struct test_kit_typec_port_info *typec_port_info = NULL;
	struct oplus_virtual_buck_ic *vb = NULL;
	enum oplus_chg_typec_port_role_type typec_mode;
	int situation;
	int otg_status;
	bool present;
	bool pass = true;
	int rc;

	if (info == NULL) {
		chg_err("[TYPEC-PORT-CHECK]: info is NULL\n");
		return false;
	}
	typec_port_info = info;
	vb = typec_port_info->private_data;

	if (buf == NULL) {
		chg_err("[TYPEC-PORT-CHECK]: buf is NULL\n");
		return false;
	}
	*use_size = 0;

	rc = oplus_chg_ic_func(vb->ic_dev,
			       OPLUS_IC_FUNC_BUCK_INPUT_PRESENT,
			       &present);
	if (rc < 0) {
		chg_err("can't input present status, rc=%d\n", rc);
		*use_size += snprintf(buf + *use_size, len - *use_size,
			"[%s][typec check case: %d] can't input present status, rc=%d\n",
			typec_port_info->name, typec_port_info->case_num, rc);
		return false;
	}
	rc = oplus_chg_ic_func(vb->ic_dev,
			       OPLUS_IC_FUNC_GET_OTG_ONLINE_STATUS,
			       &otg_status);
	if (rc < 0) {
		chg_err("can't get otg status, rc=%d\n", rc);
		*use_size += snprintf(buf + *use_size, len - *use_size,
			"[%s][typec check case: %d] can't get otg status, rc=%d\n",
			typec_port_info->name, typec_port_info->case_num, rc);
		return false;
	}

	if (present)
		situation = SITUATION_CHARGING;
	else if (otg_status != DISCONNECT)
		situation = SITUATION_OTG;
	else
		situation = SITUATION_IDLE;

	if (situation != typec_port_info->situation) {
		*use_size += snprintf(buf + *use_size, len - *use_size,
			"[%s][typec check case: %d],situation expected: %d, actually: %d\n",
			typec_port_info->name, typec_port_info->case_num,
			typec_port_info->situation, situation);
		pass = false;
	}

	rc = oplus_chg_ic_func(vb->ic_dev,
			       OPLUS_IC_FUNC_GET_TYPEC_ROLE,
			       &typec_mode);
	if (rc < 0) {
		chg_err("can't get typec mode, rc=%d\n", rc);
		*use_size += snprintf(buf + *use_size, len - *use_size,
			"[%s][typec check case: %d] can't get typec mode, rc=%d\n",
			typec_port_info->name, typec_port_info->case_num, rc);
		return false;
	}

	if (typec_mode != typec_port_info->status) {
		*use_size += snprintf(buf + *use_size, len - *use_size,
			"[%s][typec check case: %d],typec mode error, expected: %d, actually: %u\n",
			typec_port_info->name, typec_port_info->case_num,
			typec_port_info->status, typec_mode);
		pass = false;
	}

	return pass;
}

#define UART_TX_INFO_INDEX	0
#define UART_RX_INFO_INDEX	1
struct test_kit_soc_gpio_info g_uart_gpio_info[] = {
	{
		.name = "uart_tx",
		.is_out = false,
		.is_high = false,
		.func = 0,
		.pull = 0,
		.drive = 2,
	},
	{
		.name = "uart_rx",
		.is_out = false,
		.is_high = false,
		.func = 0,
		.pull = 0,
		.drive = 2,
	},
	{}
};

const struct test_feature_cfg g_uart_gpio_test_cfg = {
	.name = "uart_gpio_test",
	.test_info = (void *)g_uart_gpio_info,
#if IS_ENABLED(CONFIG_OPLUS_CHARGER_MTK)
	.test_func = test_kit_mtk_soc_gpio_test,
#else
	.test_func = test_kit_qcom_soc_gpio_test,
#endif
};

static int oplus_virtual_buck_test_kit_init(struct oplus_virtual_buck_ic *chip)
{
	chip->uart_gpio_test = test_feature_register(&g_uart_gpio_test_cfg, chip);
	if (IS_ERR_OR_NULL(chip->uart_gpio_test))
		chg_err("uart_gpio_test register error");
	test_kit_reg_typec_port_check(test_kit_typec_port_check);
	chip->typec_port_test = test_feature_register(&g_typec_port_test_cfg, chip);
	if (IS_ERR_OR_NULL(chip->typec_port_test))
		chg_err("typec_port_test register error");
	g_typec_port_info[0].private_data = chip;

	return 0;
}

static void oplus_virtual_buck_test_kit_exit(struct oplus_virtual_buck_ic *chip)
{
	if (!IS_ERR_OR_NULL(chip->typec_port_test))
		test_feature_unregister(chip->typec_port_test);
	if (!IS_ERR_OR_NULL(chip->uart_gpio_test))
		test_feature_unregister(chip->uart_gpio_test);
	test_kit_unreg_typec_port_check();
}
#endif /* CONFIG_OPLUS_CHG_TEST_KIT */

static int oplus_chg_vb_set_typec_mode(struct oplus_chg_ic_dev *ic_dev,
				       enum oplus_chg_typec_port_role_type mode);
static int oplus_vb_virq_register(struct oplus_virtual_buck_ic *chip);

static inline bool func_is_support(struct oplus_virtual_buck_child *ic,
				   enum oplus_chg_ic_func func_id)
{
	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
	case OPLUS_IC_FUNC_EXIT:
	/*case OPLUS_IC_FUNC_BUCK_GET_BYB_STATUS:
	case OPLUS_IC_FUNC_BUCK_GET_BYB_VOUT:
	case OPLUS_IC_FUNC_BUCK_SET_BYB_VOUT:*/
		return true; /* must support */
	default:
		break;
	}

	if (ic->func_num > 0)
		return oplus_chg_ic_func_check_support_by_table(ic->funcs, ic->func_num, func_id);
	else
		return false;
}

#define BYB_INFO_LEN	1024
#define CHILD_BYB_INFO_LEN	256
static int oplus_chg_vb_get_byb_status_info(struct oplus_chg_ic_dev *ic_dev, char *buf)
{
	struct oplus_virtual_buck_ic *chip;
	int i;
	int index = 0;
	int rc = 0;
	int len = BYB_INFO_LEN;
	char child_buff[CHILD_BYB_INFO_LEN] = {0};

	if (buf == NULL) {
		chg_err("buf is NULL\n");
		return false;
	}

	if (ic_dev == NULL) {
		chg_err("feature ic_dev is NULL\n");
		index += scnprintf(buf + index, len - index, "ic_dev_is_NULL");
		return false;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (chip == NULL) {
		chg_err("feature chip is NULL\n");
		index += scnprintf(buf + index, len - index, "chip_is_NULL");
		return false;
	}

	for (i = 0; i < chip->child_num; i++) {
		if (!func_is_support(&chip->child_list[i], OPLUS_IC_FUNC_BUCK_GET_BYB_STATUS)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(chip->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_GET_BYB_STATUS, child_buff);
		if (rc < 0) {
			chg_err("child ic[%d] get byb status error, rc=%d\n", i, rc);
			continue;
		}
		index += scnprintf(buf + index, len - index, "%s", child_buff);
	}

	index += scnprintf(buf + index, len - index, "\n");
	chg_info("get_byb_status:%d %s", index, buf);
	return index;
}

static int oplus_chg_vb_get_byb_vout_info(struct oplus_chg_ic_dev *ic_dev, char *buf)
{
	struct oplus_virtual_buck_ic *chip;
	int i;
	int index = 0;
	int rc = 0;
	int len = BYB_INFO_LEN;
	char child_buff[CHILD_BYB_INFO_LEN] = {0};

	if (buf == NULL) {
		chg_err("buf is NULL\n");
		return false;
	}

	if (ic_dev == NULL) {
		chg_err("feature ic_dev is NULL\n");
		index += scnprintf(buf + index, len - index, "ic_dev_is_NULL");
		return false;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (chip == NULL) {
		chg_err("feature chip is NULL\n");
		index += scnprintf(buf + index, len - index, "chip_is_NULL");
		return false;
	}

	for (i = 0; i < chip->child_num; i++) {
		if (!func_is_support(&chip->child_list[i], OPLUS_IC_FUNC_BUCK_GET_BYB_VOUT)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(chip->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_GET_BYB_VOUT, child_buff);
		if (rc < 0) {
			chg_err("child ic[%d] get byb status error, rc=%d\n", i, rc);
			continue;
		}
		index += scnprintf(buf + index, len - index, "%s", child_buff);
	}

	index += scnprintf(buf + index, len - index, "\n");
	chg_info("get_byb_vout:%d,%s", index, buf);
	return index;
}

static int oplus_chg_vb_set_byb_vout_info(struct oplus_chg_ic_dev *ic_dev, int vout)
{
	struct oplus_virtual_buck_ic *chip;
	int i;
	int index = 0;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("feature ic_dev is NULL\n");
		return false;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (chip == NULL) {
		chg_err("feature chip is NULL\n");
		return false;
	}

	for (i = 0; i < chip->child_num; i++) {
		if (!func_is_support(&chip->child_list[i], OPLUS_IC_FUNC_BUCK_SET_BYB_VOUT)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(chip->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_SET_BYB_VOUT, vout);
		if (rc < 0) {
			chg_err("child ic[%d] get byb status error, rc=%d\n", i, rc);
			continue;
		}
	}

	chg_info("set_byb_vout is %d,%d\n", index, vout);
	return index;
}

static inline bool virq_is_support(struct oplus_virtual_buck_child *ic,
				   enum oplus_chg_ic_virq_id virq_id)
{
	switch (virq_id) {
	case OPLUS_IC_VIRQ_ERR:
	case OPLUS_IC_VIRQ_ONLINE:
	case OPLUS_IC_VIRQ_OFFLINE:
		return true; /* must support */
	default:
		break;
	}

	if (ic->virq_num > 0)
		return oplus_chg_ic_virq_check_support_by_table(ic->virqs, ic->virq_num, virq_id);
	else
		return false;
}

irqreturn_t oplus_vc_ccdetect_change_handler(int irq, void *data)
{
	struct oplus_virtual_buck_ic *chip = data;

	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_CC_DETECT);
	return IRQ_HANDLED;
}

#ifndef CONFIG_OPLUS_CHARGER_MTK
static int oplus_vc_usbtemp_l_gpio_init(struct oplus_virtual_buck_ic *chip)
{
	struct pinctrl_state *usbtemp_l_gpio_default = NULL;
	int rc;

	chip->misc_gpio.pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->misc_gpio.pinctrl)) {
		chg_err("get pinctrl fail\n");
		return -EINVAL;
	}

	usbtemp_l_gpio_default = pinctrl_lookup_state(chip->misc_gpio.pinctrl, "usbtemp_l_gpio_default");
	if (IS_ERR_OR_NULL(usbtemp_l_gpio_default)) {
		chg_err("get usbtemp_l_gpio_default error\n");
		return -EINVAL;
	}

	rc = pinctrl_select_state(chip->misc_gpio.pinctrl, usbtemp_l_gpio_default);
	if (rc < 0)
		chg_err("set usbtemp_l_gpio to default error\n");

	return rc;
}

static int oplus_vc_usbtemp_r_gpio_init(struct oplus_virtual_buck_ic *chip)
{
	struct pinctrl_state *usbtemp_r_gpio_default = NULL;
	int rc;

	chip->misc_gpio.pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->misc_gpio.pinctrl)) {
		chg_err("get pinctrl fail\n");
		return -EINVAL;
	}

	usbtemp_r_gpio_default = pinctrl_lookup_state(chip->misc_gpio.pinctrl, "usbtemp_r_gpio_default");
	if (IS_ERR_OR_NULL(usbtemp_r_gpio_default)) {
		chg_err("get usbtemp_r_gpio_default error\n");
		return -EINVAL;
	}

	rc = pinctrl_select_state(chip->misc_gpio.pinctrl, usbtemp_r_gpio_default);
	if (rc < 0)
		chg_err("set usbtemp_r_gpio to default error\n");

	return rc;
}
#endif

static bool oplus_vc_usbtemp_check_is_support(struct oplus_virtual_buck_ic *chip)
{
	int i;

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if (get_eng_version() == AGING) {
		chg_err("AGING mode, disable usbtemp\n");
		return false;
	}
#endif

	if(gpio_is_valid(chip->misc_gpio.dischg_gpio))
		return true;
	for (i = 0; i < chip->child_num; i++) {
		if (func_is_support(&chip->child_list[i], OPLUS_IC_FUNC_SET_USB_DISCHG_ENABLE))
			return true;
	}

	chg_info("dischg return false\n");

	return false;
}

static int oplus_vc_dischg_gpio_init(struct oplus_virtual_buck_ic *chip)
{
	int rc;

	chip->misc_gpio.pinctrl = devm_pinctrl_get(chip->dev);

	if (IS_ERR_OR_NULL(chip->misc_gpio.pinctrl)) {
		chg_err("get dischg_pinctrl fail\n");
		return -EINVAL;
	}

	chip->misc_gpio.dischg_enable = pinctrl_lookup_state(chip->misc_gpio.pinctrl, "dischg_enable");
	if (IS_ERR_OR_NULL(chip->misc_gpio.dischg_enable)) {
		chg_err("get dischg_enable fail\n");
		return -EINVAL;
	}

	chip->misc_gpio.dischg_disable = pinctrl_lookup_state(chip->misc_gpio.pinctrl, "dischg_disable");
	if (IS_ERR_OR_NULL(chip->misc_gpio.dischg_disable)) {
		chg_err("get dischg_disable fail\n");
		return -EINVAL;
	}

	rc = pinctrl_select_state(chip->misc_gpio.pinctrl, chip->misc_gpio.dischg_disable);
	if (rc < 0)
		chg_err("set dischg disable error\n");

	return rc;
}

static int oplus_vc_usbtemp_adc_init(struct oplus_virtual_buck_ic *chip)
{
	struct device_node *node = chip->dev->of_node;
	int rc = 0;

	chip->misc_gpio.dischg_gpio = of_get_named_gpio(node, "oplus,dischg-gpio", 0);
	if (!gpio_is_valid(chip->misc_gpio.dischg_gpio)) {
		chg_info("Couldn't read oplus,dischg-gpio, rc=%d\n",
			chip->misc_gpio.dischg_gpio);
	} else {
		if (oplus_vc_usbtemp_check_is_support(chip) == true) {
			rc = gpio_request(chip->misc_gpio.dischg_gpio, "dischg-gpio");
			if (rc) {
				chg_err("unable to request dischg-gpio:%d\n", chip->misc_gpio.dischg_gpio);
				return rc;
			}
			chip->misc_gpio.is_dischg_gpio_request = true;
			rc = oplus_vc_dischg_gpio_init(chip);
			if (rc) {
				chg_err("unable to init dischg-gpio:%d\n", chip->misc_gpio.dischg_gpio);
				goto free_dischg_gpio;
			}
		}
	}

#ifndef CONFIG_OPLUS_CHARGER_MTK
	rc = oplus_vc_usbtemp_l_gpio_init(chip);
	if (rc < 0) {
		chg_err("usbtemp_l_gpio init error, rc=%d\n", rc);
		goto free_dischg_gpio;
	}
	rc = oplus_vc_usbtemp_r_gpio_init(chip);
	if (rc < 0) {
		chg_err("usbtemp_r_gpio init error, rc=%d\n", rc);
		goto free_dischg_gpio;
	}
#endif

	return 0;

free_dischg_gpio:
	if (oplus_vc_usbtemp_check_is_support(chip)) {
		gpio_free(chip->misc_gpio.dischg_gpio);
		chip->misc_gpio.is_dischg_gpio_request = false;
	}
	return rc;
}

static int oplus_vc_usbtemp_iio_init(struct oplus_virtual_buck_ic *chip)
{
	struct device_node *node = chip->dev->of_node;
	int rc = 0;

	rc = of_property_match_string(node, "io-channel-names", "usb_temp_adc_l");
	if (rc >= 0) {
		chip->usbtemp_adc_l = iio_channel_get(chip->dev, "usb_temp_adc_l");
		if (IS_ERR(chip->usbtemp_adc_l)) {
			rc = PTR_ERR(chip->usbtemp_adc_l);
			chg_err("usb_temp_adc_l get error, rc=%d\n", rc);
			chip->usbtemp_adc_l = NULL;
			return rc;
		}
	} else {
		chg_err("usb_temp_adc_l not found\n");
	}

	rc = of_property_match_string(node, "io-channel-names", "usb_temp_adc_r");
	if (rc >= 0) {
		chip->usbtemp_adc_r = iio_channel_get(chip->dev, "usb_temp_adc_r");
		if (IS_ERR(chip->usbtemp_adc_r)) {
			rc = PTR_ERR(chip->usbtemp_adc_r);
			chg_err("usb_temp_adc_r get error, rc=%d\n", rc);
			chip->usbtemp_adc_r = NULL;
			return rc;
		}
	} else {
		chg_err("usb_temp_adc_r not found\n");
	}

	rc = of_property_read_u32(node, "oplus,usbtemp_conversion_ratio", &chip->usbtemp_conversion_ratio);
	if (rc)
		chip->usbtemp_conversion_ratio = 18;

	chg_info("oplus,usbtemp_conversion_ratio=%d\n", chip->usbtemp_conversion_ratio);

	return 0;
}

static int oplus_vc_ship_gpio_init(struct oplus_virtual_buck_ic *chip)
{
	int rc;

	chip->misc_gpio.pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->misc_gpio.pinctrl)) {
		chg_err("get pinctrl fail\n");
		return -EPROBE_DEFER;
	}

	chip->misc_gpio.ship_active = pinctrl_lookup_state(chip->misc_gpio.pinctrl, "ship_active");
	if (IS_ERR_OR_NULL(chip->misc_gpio.ship_active)) {
		chg_err("get ship_active fail\n");
		return -EINVAL;
	}

	chip->misc_gpio.ship_sleep = pinctrl_lookup_state(chip->misc_gpio.pinctrl, "ship_sleep");
	if (IS_ERR_OR_NULL(chip->misc_gpio.ship_sleep)) {
		chg_err("get ship_sleep fail\n");
		return -EINVAL;
	}

	rc = pinctrl_select_state(chip->misc_gpio.pinctrl, chip->misc_gpio.ship_sleep);
	if (rc < 0)
		chg_err("set ship gpio sleep error, rc=%d\n", rc);

	return rc;
}

static int oplus_vc_ccdetect_gpio_init(struct oplus_virtual_buck_ic *chip)
{
	int rc;

	chip->misc_gpio.pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->misc_gpio.pinctrl)) {
		chg_err("get pinctrl fail\n");
		return -EPROBE_DEFER;
	}

	chip->misc_gpio.ccdetect_active = pinctrl_lookup_state(
		chip->misc_gpio.pinctrl, "ccdetect_active");
	if (IS_ERR_OR_NULL(chip->misc_gpio.ccdetect_active)) {
		chg_err("get ccdetect_active fail\n");
		return -EINVAL;
	}

	chip->misc_gpio.ccdetect_sleep = pinctrl_lookup_state(
		chip->misc_gpio.pinctrl, "ccdetect_sleep");
	if (IS_ERR_OR_NULL(chip->misc_gpio.ccdetect_sleep)) {
		chg_err("get ccdetect_sleep fail\n");
		return -EINVAL;
	}

	if (gpio_is_valid(chip->misc_gpio.ccdetect_gpio)) {
		gpio_direction_input(chip->misc_gpio.ccdetect_gpio);
	}

	rc = pinctrl_select_state(chip->misc_gpio.pinctrl, chip->misc_gpio.ccdetect_active);
	if (rc < 0)
		chg_err("set ccdetect gpio active error, rc=%d\n", rc);

	return rc;
}

bool oplus_vc_ccdetect_gpio_support(struct oplus_virtual_buck_ic *chip)
{
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	int boot_mode = get_boot_mode();

	/* HW engineer requirement */
	if (boot_mode == MSM_BOOT_MODE__RF ||
	    boot_mode == MSM_BOOT_MODE__WLAN ||
	    boot_mode == MSM_BOOT_MODE__FACTORY)
		return false;
#endif

	if (gpio_is_valid(chip->misc_gpio.ccdetect_gpio))
		return true;

	return false;
}

static int oplus_ccdetect_enable(struct oplus_virtual_buck_ic *chip, bool en)
{
	int rc;

	rc = oplus_chg_vb_set_typec_mode(chip->ic_dev,
		en ? TYPEC_PORT_ROLE_TRY_SNK : TYPEC_PORT_ROLE_SNK);
	if (rc < 0)
		chg_err("%s ccdetect error, rc=%d\n", en ? "enable" : "disable", rc);

	return rc;
}

static int oplus_vc_ccdetect_before_irq_register(struct oplus_virtual_buck_ic *chip)
{
	int level = 1;

	level = gpio_get_value(chip->misc_gpio.ccdetect_gpio);
	usleep_range(2000, 2100);
	if (level != gpio_get_value(chip->misc_gpio.ccdetect_gpio)) {
		chg_err("ccdetect_gpio is unstable, try again...\n");
		usleep_range(10000, 11000);
		level = gpio_get_value(chip->misc_gpio.ccdetect_gpio);
	}

	if (level <= 0) {
		oplus_ccdetect_enable(chip, true);
	}

	return 0;
}

static int oplus_vc_ccdetect_irq_register(struct oplus_virtual_buck_ic *chip)
{
	int rc = 0;

	rc = devm_request_threaded_irq(chip->dev, chip->ccdetect_irq,
			NULL, oplus_vc_ccdetect_change_handler, IRQF_TRIGGER_FALLING
			| IRQF_TRIGGER_RISING | IRQF_ONESHOT, "ccdetect-change", chip);
	if (rc < 0) {
		chg_err("Unable to request ccdetect-change irq: %d\n", rc);
		return rc;
	}

	rc = enable_irq_wake(chip->ccdetect_irq);
	if (rc != 0) {
		chg_err("enable_irq_wake: ccdetect_irq failed %d\n", rc);
		devm_free_irq(chip->dev, chip->ccdetect_irq, chip);
	}

	return rc;
}

static void oplus_vc_upload_device_id_subscribe_error_topic(
	struct oplus_mms *topic, void *prv_data)
{
	struct oplus_virtual_buck_ic *chip = prv_data;

	/* delay 50ms upload for error topic init stop publish finish */
	schedule_delayed_work(&chip->upload_device_id_work, msecs_to_jiffies(50));
}

static void oplus_vc_upload_device_id_work(struct work_struct *work)
{
	int i;
	int rc;
	struct mms_msg *msg;
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_virtual_buck_ic *chip = container_of(dwork,
		struct oplus_virtual_buck_ic, upload_device_id_work);
	struct oplus_mms *err_topic = oplus_mms_get_by_name("error");

	if (!err_topic)
		return;

	for (i = 0; i < chip->child_num; i++) {
		if (!func_is_support(&chip->child_list[i], OPLUS_IC_FUNC_BUCK_INPUT_PRESENT))
			continue;

		if (chip->child_list[i].ic_dev) {
			msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_DEVICE_ID,
				"buck_ic=%s", chip->child_list[i].ic_dev->name);
			if (msg == NULL) {
				chg_err("alloc device id msg error\n");
				return;
			}

			rc = oplus_mms_publish_msg_sync(err_topic, msg);
			if (rc < 0) {
				chg_err("publish device id msg error, rc=%d\n", rc);
				kfree(msg);
			}
			break;
		}
	}
}

static void oplus_vc_upload_device_id(struct oplus_virtual_buck_ic *chip)
{
	oplus_mms_wait_topic("error",
		oplus_vc_upload_device_id_subscribe_error_topic, chip);
}

static int oplus_vc_input_pg_gpio_init(struct oplus_virtual_buck_ic *chip)
{
	struct pinctrl *input_pg_pinctrl = NULL;
	struct pinctrl_state *input_pg_default = NULL;

	input_pg_pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(input_pg_pinctrl)) {
		chg_err("get input_pg_pinctrl fail\n");
		return -EINVAL;
	}

	input_pg_default = pinctrl_lookup_state(input_pg_pinctrl, "input_pg_default");
	if (IS_ERR_OR_NULL(input_pg_default)) {
		chg_err("get input_pg_default fail\n");
		return -EINVAL;
	}

	pinctrl_select_state(input_pg_pinctrl, input_pg_default);

	return 0;
}

#if defined(OPLUS_FEATURE_POWERINFO_FTM) && defined(CONFIG_OPLUS_POWERINFO_FTM)
extern bool ext_boot_with_console(void);
#endif
static int oplus_vc_chg_2uart_pinctrl_init(struct oplus_virtual_buck_ic *chip)
{
	struct pinctrl		*chg_2uart_pinctrl;
	struct pinctrl_state	*chg_2uart_active;
	struct pinctrl_state	*chg_2uart_sleep;
#if IS_ENABLED(CONFIG_OPLUS_CHG_TEST_KIT)
	struct device_node *node = chip->dev->of_node;
	struct gpio_chip *gpio_chip;
	int uart_tx, uart_rx;
#endif /* CONFIG_OPLUS_CHG_TEST_KIT */

#if IS_ENABLED(CONFIG_OPLUS_CHG_TEST_KIT)
	uart_tx = of_get_named_gpio(node, "oplus,uart_tx-gpio", 0);
	if (gpio_is_valid(uart_tx)) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0))
		gpio_chip = gpio_to_chip(uart_tx);
#else
		gpio_chip = gpiod_to_chip(gpio_to_desc(uart_tx));
#endif
		g_uart_gpio_info[UART_TX_INFO_INDEX].chip = gpio_chip;
		if (gpio_chip != NULL)
			g_uart_gpio_info[UART_TX_INFO_INDEX].num = uart_tx - gpio_chip->base;
	}
	uart_rx = of_get_named_gpio(node, "oplus,uart_rx-gpio", 0);
	if (gpio_is_valid(uart_rx)) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0))
		gpio_chip = gpio_to_chip(uart_rx);
#else
		gpio_chip = gpiod_to_chip(gpio_to_desc(uart_rx));
#endif
		g_uart_gpio_info[UART_RX_INFO_INDEX].chip = gpio_chip;
		if (gpio_chip != NULL)
			g_uart_gpio_info[UART_RX_INFO_INDEX].num = uart_rx - gpio_chip->base;
	}
#endif /* CONFIG_OPLUS_CHG_TEST_KIT */

	chg_2uart_pinctrl = devm_pinctrl_get(chip->dev);

	if (IS_ERR_OR_NULL(chg_2uart_pinctrl)) {
		chg_err("get 2uart chg_2uart_pinctrl fail\n");
		return -EINVAL;
	}

	chg_2uart_active = pinctrl_lookup_state(chg_2uart_pinctrl, "chg_qupv3_se3_2uart_active");
	if (IS_ERR_OR_NULL(chg_2uart_active)) {
		chg_err("get chg_qupv3_se3_2uart_active fail\n");
		return -EINVAL;
	}

	chg_2uart_sleep = pinctrl_lookup_state(chg_2uart_pinctrl, "chg_qupv3_se3_2uart_sleep");
	if (IS_ERR_OR_NULL(chg_2uart_sleep)) {
		chg_err("get chg_qupv3_se3_2uart_sleep fail\n");
		return -EINVAL;
	}

#if defined(OPLUS_FEATURE_POWERINFO_FTM) && defined(CONFIG_OPLUS_POWERINFO_FTM)
	if (!ext_boot_with_console()) {
		chg_err("set chg_qupv3_se3_2uart_sleep\n");
		pinctrl_select_state(chg_2uart_pinctrl, chg_2uart_sleep);
	}
#endif

	return 0;
}

static int oplus_vc_misc_init(struct oplus_virtual_buck_ic *chip)
{
	struct device_node *node = chip->dev->of_node;
	int rc = 0;

	chip->misc_gpio.ship_gpio = of_get_named_gpio(node, "oplus,ship-gpio", 0);
	if (!gpio_is_valid(chip->misc_gpio.ship_gpio)) {
		chg_err("Couldn't read oplus,ship-gpio, rc=%d\n",
			chip->misc_gpio.ship_gpio);
	} else {
		rc = gpio_request(chip->misc_gpio.ship_gpio, "ship-gpio");
		if (rc) {
			chg_err("unable to request ship-gpio:%d\n", chip->misc_gpio.ship_gpio);
			return rc;
		}
		rc = oplus_vc_ship_gpio_init(chip);
		if (rc) {
			chg_err("unable to init ship-gpio:%d\n", chip->misc_gpio.ship_gpio);
			goto free_ship_gpio;
		}
		chg_info("init ship-gpio level[%d]\n", gpio_get_value(chip->misc_gpio.ship_gpio));
	}

	chip->misc_gpio.ccdetect_gpio = of_get_named_gpio(node, "oplus,ccdetect-gpio", 0);
	if (!gpio_is_valid(chip->misc_gpio.ccdetect_gpio)) {
		chg_err("Couldn't read oplus,ccdetect-gpio, rc=%d\n",
			chip->misc_gpio.ccdetect_gpio);
	} else {
		if (oplus_vc_ccdetect_gpio_support(chip)) {
			rc = gpio_request(chip->misc_gpio.ccdetect_gpio, "ccdetect-gpio");
			if (rc) {
				chg_err("unable to request ccdetect-gpio:%d\n", chip->misc_gpio.ccdetect_gpio);
				goto free_vchg_trig_gpio;
			} else {
				rc = oplus_vc_ccdetect_gpio_init(chip);
				if (rc) {
					chg_err("unable to init ccdetect-gpio:%d\n", chip->misc_gpio.ccdetect_gpio);
					goto free_ccdetect_gpio;
				}
				chip->ccdetect_irq = gpio_to_irq(chip->misc_gpio.ccdetect_gpio);
			}
		}
	}

	oplus_vc_input_pg_gpio_init(chip);
	oplus_vc_chg_2uart_pinctrl_init(chip);

	return 0;

free_ccdetect_gpio:
	if (oplus_vc_ccdetect_gpio_support(chip))
		gpio_free(chip->misc_gpio.ccdetect_gpio);
free_vchg_trig_gpio:
	if (gpio_is_valid(chip->misc_gpio.vchg_trig_gpio))
		gpio_free(chip->misc_gpio.vchg_trig_gpio);
free_ship_gpio:
	if (gpio_is_valid(chip->misc_gpio.ship_gpio))
		gpio_free(chip->misc_gpio.ship_gpio);

	return rc;
}

static int oplus_vc_child_funcs_init(struct oplus_virtual_buck_ic *chip, int child_num)
{
	struct device_node *node = chip->dev->of_node;
	struct device_node *func_node = NULL;
	int i, m;
	int rc = 0;

	for (i = 0; i < child_num; i++) {
		func_node = of_parse_phandle(node, "oplus,buck_ic_func_group", i);
		if (func_node == NULL) {
			chg_err("can't get ic[%d] function group\n", i);
			rc = -ENODATA;
			goto err;
		}
		rc = of_property_count_elems_of_size(func_node, "functions",
						     sizeof(u32));
		if (rc < 0) {
			chg_err("can't get ic[%d] functions size, rc=%d\n", i, rc);
			goto err;
		}
		chip->child_list[i].func_num = rc;
		chip->child_list[i].funcs =
			devm_kzalloc(chip->dev,
				     sizeof(enum oplus_chg_ic_func) *
					     chip->child_list[i].func_num,
				     GFP_KERNEL);
		if (chip->child_list[i].funcs == NULL) {
			rc = -ENOMEM;
			chg_err("alloc child ic funcs memory error\n");
			goto err;
		}
		rc = of_property_read_u32_array(
			func_node, "functions",
			(u32 *)chip->child_list[i].funcs,
			chip->child_list[i].func_num);
		if (rc) {
			i++;
			chg_err("can't get ic[%d] functions, rc=%d\n", i, rc);
			goto err;
		}
		(void)oplus_chg_ic_func_table_sort(
			chip->child_list[i].funcs,
			chip->child_list[i].func_num);
	}

	return 0;

err:
	for (m = i; m > 0; m--)
		devm_kfree(chip->dev, chip->child_list[m - 1].funcs);
	return rc;
}

static int oplus_vc_child_virqs_init(struct oplus_virtual_buck_ic *chip, int child_num)
{
	struct device_node *node = chip->dev->of_node;
	struct device_node *virq_node = NULL;
	int i, m;
	int rc = 0;

	for (i = 0; i < child_num; i++) {
		virq_node = of_parse_phandle(node, "oplus,buck_ic_func_group", i);
		if (virq_node == NULL) {
			chg_err("can't get ic[%d] function group\n", i);
			rc = -ENODATA;
			goto err;
		}
		rc = of_property_count_elems_of_size(virq_node, "virqs",
						     sizeof(u32));
		if (rc <= 0) {
			chip->child_list[i].virq_num = 0;
			chip->child_list[i].virqs = NULL;
			continue;
		}
		chip->child_list[i].virq_num = rc;
		chip->child_list[i].virqs =
			devm_kzalloc(chip->dev,
				     sizeof(enum oplus_chg_ic_func) *
					     chip->child_list[i].virq_num,
				     GFP_KERNEL);
		if (chip->child_list[i].virqs == NULL) {
			rc = -ENOMEM;
			chg_err("alloc child ic virqs memory error\n");
			goto err;
		}
		rc = of_property_read_u32_array(
			virq_node, "virqs",
			(u32 *)chip->child_list[i].virqs,
			chip->child_list[i].virq_num);
		if (rc) {
			i++;
			chg_err("can't get ic[%d] virqs, rc=%d\n", i, rc);
			goto err;
		}
		(void)oplus_chg_ic_irq_table_sort(chip->child_list[i].virqs, chip->child_list[i].virq_num);
	}

	return 0;

err:
	for (m = i; m > 0; m--) {
		if (chip->child_list[m - 1].virqs != NULL)
			devm_kfree(chip->dev, chip->child_list[m - 1].virqs);
	}
	return rc;
}

static int oplus_vc_child_init(struct oplus_virtual_buck_ic *chip)
{
	struct device_node *node = chip->dev->of_node;
	int i;
	int rc = 0;

	rc = of_property_read_u32(node, "oplus,buck_ic_connect",
				  &chip->connect_type);
	if (rc < 0) {
		chg_err("can't get buck ic connect type, rc=%d\n", rc);
		return rc;
	}
	rc = of_property_read_u32(node, "oplus,main_charger", &chip->main_charger);
	if (rc < 0) {
		chg_err("can't get main charger index, rc=%d\n", rc);
		return rc;
	}
	rc = of_property_count_elems_of_size(node, "oplus,buck_ic",
					     sizeof(u32));
	if (rc < 0) {
		chg_err("can't get buck ic number, rc=%d\n", rc);
		return rc;
	}
	chip->child_num = rc;
	chip->child_list = devm_kzalloc(
		chip->dev,
		sizeof(struct oplus_virtual_buck_child) * chip->child_num,
		GFP_KERNEL);
	if (chip->child_list == NULL) {
		rc = -ENOMEM;
		chg_err("alloc child ic memory error\n");
		return rc;
	}

	for (i = 0; i < chip->child_num; i++) {
		chip->child_list[i].ic_dev = of_get_oplus_chg_ic(node, "oplus,buck_ic", i);
		if (chip->child_list[i].ic_dev == NULL) {
			chg_err("not find buck ic %d\n", i);
			rc = -EAGAIN;
			goto read_property_err;
		}
		rc = of_property_read_u32_index(
			node, "oplus,buck_ic_current_ratio", i,
			&chip->child_list[i].current_ratio);
		if (rc < 0) {
			chg_err("can't read ic[%d] current ratio, rc=%d\n", i,
			       rc);
			goto read_property_err;
		}
	}

	rc = oplus_vc_child_funcs_init(chip, chip->child_num);
	if (rc < 0)
		goto child_funcs_init_err;
	rc = oplus_vc_child_virqs_init(chip, chip->child_num);
	if (rc < 0)
		goto child_virqs_init_err;

	return 0;

child_virqs_init_err:
	for (i = 0; i < chip->child_num; i++)
		devm_kfree(chip->dev, chip->child_list[i].funcs);
child_funcs_init_err:
read_property_err:
	for (; i >=0; i--)
		chip->child_list[i].ic_dev = NULL;
	devm_kfree(chip->dev, chip->child_list);
	chip->child_list = NULL;
	return rc;
}

static int oplus_chg_vb_init(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_buck_ic *chip;
	int i, m;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (ic_dev->online)
		return 0;

	rc = oplus_vc_child_init(chip);
	if (rc < 0) {
		chg_err("child list init error, rc=%d\n", rc);
		goto child_list_init_err;
	}

	rc = oplus_vb_virq_register(chip);
	if (rc < 0) {
		chg_err("virq register error, rc=%d\n", rc);
		goto virq_register_err;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < chip->child_num; i++) {
		rc = oplus_chg_ic_func(chip->child_list[i].ic_dev, OPLUS_IC_FUNC_INIT);
		if (rc < 0) {
			chg_err("child ic[%d] init error, rc=%d\n", i, rc);
			goto child_init_err;
		}
		oplus_chg_ic_set_parent(chip->child_list[i].ic_dev, ic_dev);
		if (!func_is_support(&chip->child_list[i],
				     OPLUS_IC_FUNC_BUCK_SET_ICL) ||
		    chip->child_list[i].current_ratio == 0) {
			if (func_is_support(&chip->child_list[i],
					    OPLUS_IC_FUNC_BUCK_INPUT_SUSPEND))
				oplus_chg_ic_func(
					chip->child_list[i].ic_dev,
					OPLUS_IC_FUNC_BUCK_INPUT_SUSPEND, true);
		}
		if (!func_is_support(&chip->child_list[i],
				     OPLUS_IC_FUNC_BUCK_SET_FCC) ||
		    chip->child_list[i].current_ratio == 0) {
			if (func_is_support(&chip->child_list[i],
					    OPLUS_IC_FUNC_BUCK_OUTPUT_SUSPEND))
				oplus_chg_ic_func(
					chip->child_list[i].ic_dev,
					OPLUS_IC_FUNC_BUCK_OUTPUT_SUSPEND,
					true);
		}
	}

	if (oplus_vc_ccdetect_gpio_support(chip)) {
		rc = oplus_vc_ccdetect_before_irq_register(chip);
		if (rc < 0)
			goto child_init_err;
		rc = oplus_vc_ccdetect_irq_register(chip);
		if (rc < 0)
			goto child_init_err;
	}
	oplus_vc_upload_device_id(chip);
	ic_dev->online = true;

	return 0;

child_init_err:
	for (m = i + 1; m > 0; m--)
		oplus_chg_ic_func(chip->child_list[m - 1].ic_dev, OPLUS_IC_FUNC_EXIT);
virq_register_err:
child_list_init_err:

	return rc;
}

static int oplus_chg_vb_exit(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_buck_ic *chip;
	int i;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (!ic_dev->online)
		return 0;

	ic_dev->online = false;

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	devm_free_irq(chip->dev, chip->ccdetect_irq, chip);
	for (i = 0; i < chip->child_num; i++) {
		rc = oplus_chg_ic_func(chip->child_list[i].ic_dev, OPLUS_IC_FUNC_EXIT);
		if (rc < 0)
			chg_err("child ic[%d] exit error, rc=%d\n", i, rc);
	}
	for (i = 0; i < chip->child_num; i++) {
		if (virq_is_support(&chip->child_list[i], OPLUS_IC_VIRQ_ERR)) {
			oplus_chg_ic_virq_release(chip->child_list[i].ic_dev,
						  OPLUS_IC_VIRQ_ERR, chip);
		}
		if (virq_is_support(&chip->child_list[i], OPLUS_IC_VIRQ_CC_DETECT)) {
			oplus_chg_ic_virq_release(chip->child_list[i].ic_dev,
						  OPLUS_IC_VIRQ_CC_DETECT, chip);
		}
		if (virq_is_support(&chip->child_list[i], OPLUS_IC_VIRQ_PLUGIN)) {
			oplus_chg_ic_virq_release(chip->child_list[i].ic_dev,
						  OPLUS_IC_VIRQ_PLUGIN, chip);
		}
		if (virq_is_support(&chip->child_list[i], OPLUS_IC_VIRQ_POWER_ROLE_STATUS)) {
			oplus_chg_ic_virq_release(chip->child_list[i].ic_dev,
						  OPLUS_IC_VIRQ_POWER_ROLE_STATUS, chip);
		}
		if (virq_is_support(&chip->child_list[i], OPLUS_IC_VIRQ_CC_CHANGED)) {
			oplus_chg_ic_virq_release(chip->child_list[i].ic_dev,
						  OPLUS_IC_VIRQ_CC_CHANGED, chip);
		}
	}
	for (i = 0; i < chip->child_num; i++) {
		if (chip->child_list[i].virqs != NULL)
			devm_kfree(chip->dev, chip->child_list[i].virqs);
	}
	for (i = 0; i < chip->child_num; i++)
		devm_kfree(chip->dev, chip->child_list[i].funcs);

	chg_info("vb_exit unregister success\n");

	return 0;
}

static int oplus_chg_vb_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_REG_DUMP);
		if (rc < 0 && rc != -ENOTSUPP)
			chg_err("child ic[%d] exit error, rc=%d\n", i, rc);
	}

	return 0;
}

static int oplus_chg_vb_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	struct oplus_virtual_buck_ic *vb;
	int i, index;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	index = 0;
	for (i = 0; i < vb->child_num; i++) {
		if (index >= len)
			return len;
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_SMT_TEST, buf + index,
				       len - index);
		if (rc < 0) {
			if (rc != -ENOTSUPP) {
				chg_err("child ic[%d] smt test error, rc=%d\n",
					i, rc);
				rc = snprintf(buf + index, len - index,
					"[%s]-[%s]:%d\n",
					vb->child_list[i].ic_dev->manu_name,
					"FUNC_ERR", rc);
			} else {
				rc = 0;
			}
		} else {
			if ((rc > 0) && buf[index + rc - 1] != '\n') {
				buf[index + rc] = '\n';
				index++;
			}
		}
		index += rc;
	}

	return index;
}

static int oplus_chg_vb_input_present(struct oplus_chg_ic_dev *ic_dev, bool *present)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_INPUT_PRESENT)) {
				rc = -ENOTSUPP;
				continue;
			}
			if (!vb->child_list[i].ic_dev) {
				chg_err("ic dev is NULL\n");
			}
			rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_INPUT_PRESENT, present);
			if (rc < 0) {
				chg_err("child ic[%d] get input present status error, rc=%d\n", i, rc);
				continue;
			}
			break;
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			if (i != 0)
				continue;
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_INPUT_PRESENT)) {
				chg_err("for serial connection, first IC must support this function\n");
				return -EINVAL;
			}
			rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_INPUT_PRESENT, present);
			if (rc < 0)
				chg_err("child ic[%d] get input present status error, rc=%d\n", i, rc);
			break;
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return rc;
}

static int oplus_chg_vb_input_suspend(struct oplus_chg_ic_dev *ic_dev, bool suspend)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;
	bool suspend_temp;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_INPUT_SUSPEND))
				continue;
			if (vb->child_list[i].current_ratio == 0)
				suspend_temp = true;
			else
				suspend_temp = suspend;
			rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_INPUT_SUSPEND, suspend_temp);
			if (rc < 0) {
				chg_err("child ic[%d] input %s error, rc=%d\n", i, suspend_temp ? "suspend" : "unsuspend", rc);
				return rc;
			}
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_INPUT_SUSPEND)) {
				chg_err("for serial connection, all ICs must support this function\n");
				return -EINVAL;
			}
			rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_INPUT_SUSPEND, suspend);
			if (rc < 0) {
				chg_err("child ic[%d] input %s error, rc=%d\n", i, suspend ? "suspend" : "unsuspend", rc);
				return rc;
			}
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return 0;
}

static int oplus_chg_vb_input_is_suspend(struct oplus_chg_ic_dev *ic_dev, bool *suspend)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;
	bool suspend_temp = true;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	*suspend = true;

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_INPUT_IS_SUSPEND))
				continue;
			rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_INPUT_IS_SUSPEND, &suspend_temp);
			if (rc < 0) {
				chg_err("child ic[%d] get input suspend status error, rc=%d\n", i, rc);
				return rc;
			}
			*suspend &= suspend_temp;
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_INPUT_IS_SUSPEND)) {
				chg_err("for serial connection, all ICs must support this function\n");
				return -EINVAL;
			}
			rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_INPUT_IS_SUSPEND, suspend);
			if (rc < 0) {
				chg_err("child ic[%d] input %s error, rc=%d\n", i, suspend ? "suspend" : "unsuspend", rc);
				return rc;
			}
			if (*suspend)
				return 0;
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return 0;
}

static int oplus_chg_vb_output_suspend(struct oplus_chg_ic_dev *ic_dev, bool suspend)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;
	bool suspend_temp;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_OUTPUT_SUSPEND))
				continue;
			if (vb->child_list[i].current_ratio == 0)
				suspend_temp = true;
			else
				suspend_temp = suspend;
			rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_OUTPUT_SUSPEND, suspend_temp);
			if (rc < 0) {
				chg_err("child ic[%d] output %s error, rc=%d\n", i, suspend_temp ? "suspend" : "unsuspend", rc);
				return rc;
			}
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_OUTPUT_SUSPEND)) {
				chg_err("for serial connection, all ICs must support this function\n");
				return -EINVAL;
			}
			rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_OUTPUT_SUSPEND, suspend);
			if (rc < 0) {
				chg_err("child ic[%d] output %s error, rc=%d\n", i, suspend ? "suspend" : "unsuspend", rc);
				return rc;
			}
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return 0;
}

static int oplus_chg_vb_output_is_suspend(struct oplus_chg_ic_dev *ic_dev, bool *suspend)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;
	bool suspend_temp = true;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	*suspend = true;

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_OUTPUT_IS_SUSPEND))
				continue;
			rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_OUTPUT_IS_SUSPEND, &suspend_temp);
			if (rc < 0) {
				chg_err("child ic[%d] get output suspend status error, rc=%d\n", i, rc);
				return rc;
			}
			*suspend &= suspend_temp;
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_OUTPUT_IS_SUSPEND)) {
				chg_err("for serial connection, all ICs must support this function\n");
				return -EINVAL;
			}
			rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_OUTPUT_IS_SUSPEND, suspend);
			if (rc < 0) {
				chg_err("child ic[%d] output %s error, rc=%d\n", i, suspend ? "suspend" : "unsuspend", rc);
				return rc;
			}
			if (*suspend)
				return 0;
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return 0;
}

static int oplus_chg_vb_set_icl(struct oplus_chg_ic_dev *ic_dev, bool vooc_mode,
				bool step, int icl_ma)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_SET_ICL))
				continue;
			rc = oplus_chg_ic_func(
				vb->child_list[i].ic_dev,
				OPLUS_IC_FUNC_BUCK_SET_ICL, vooc_mode, step,
				icl_ma * vb->child_list[i].current_ratio / 100);
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_SET_ICL)) {
				chg_err("for serial connection, all ICs must support this function\n");
				return -EINVAL;
			}
			return -EINVAL; /* TODO */
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
		if (rc < 0) {
			chg_err("child ic[%d] set icl error, rc=%d\n", i, rc);
			return rc;
		}
	}

	return 0;
}

static int oplus_chg_vb_get_icl(struct oplus_chg_ic_dev *ic_dev, int *icl_ma)
{
	struct oplus_virtual_buck_ic *vb;
	int temp_icl_ma;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	*icl_ma = 0;
	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (!func_is_support(&vb->child_list[i],
					     OPLUS_IC_FUNC_BUCK_GET_ICL))
				continue;
			rc = oplus_chg_ic_func(
				vb->child_list[i].ic_dev,
				OPLUS_IC_FUNC_BUCK_GET_ICL,
				&temp_icl_ma);
			if (rc < 0) {
				chg_err("child ic[%d] get icl error, rc=%d\n", i, rc);
				return rc;
			}
			*icl_ma += temp_icl_ma;
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_GET_ICL)) {
				chg_err("for serial connection, all ICs must support this function\n");
				return -EINVAL;
			}
			if (i == 0) {
				rc = oplus_chg_ic_func(
					vb->child_list[i].ic_dev,
					OPLUS_IC_FUNC_BUCK_GET_ICL, icl_ma);
				if (rc < 0) {
					chg_err("child ic[%d] get icl error, rc=%d\n", i, rc);
					return rc;
				}
				continue;
			}
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return 0;
}

static int oplus_chg_vb_set_fcc(struct oplus_chg_ic_dev *ic_dev, int fcc_ma)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_SET_FCC))
				continue;
			rc = oplus_chg_ic_func(
				vb->child_list[i].ic_dev,
				OPLUS_IC_FUNC_BUCK_SET_FCC,
				fcc_ma * vb->child_list[i].current_ratio / 100);
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_SET_FCC)) {
				chg_err("for serial connection, all ICs must support this function\n");
				return -EINVAL;
			}
			return -EINVAL; /* TODO */
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
		if (rc < 0) {
			chg_err("child ic[%d] set fcc error, rc=%d\n", i, rc);
			return rc;
		}
	}

	return 0;
}

static int oplus_chg_vb_set_fv(struct oplus_chg_ic_dev *ic_dev, int fv_mv)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (vb->child_list[i].current_ratio == 0)
				continue;
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			if (i < vb->child_num - 1)
				continue;
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
		if (!func_is_support(&vb->child_list[i],
				     OPLUS_IC_FUNC_BUCK_SET_FV))
			return -ENOTSUPP;
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_BUCK_SET_FV, fv_mv);
		if (rc < 0) {
			chg_err("child ic[%d] set fcc error, rc=%d\n", i, rc);
			return rc;
		}
	}

	return 0;
}

static int oplus_chg_vb_set_iterm(struct oplus_chg_ic_dev *ic_dev, int iterm_ma)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (vb->child_list[i].current_ratio == 0)
				continue;
			if (i != vb->main_charger)
				continue;
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_SET_ITERM)) {
				chg_err("main charger must support this function\n");
				return -EINVAL;
			}
			/*
			 * TODO:
			 * When the charging current is less than a certain
			 * threshold, all auxiliary charger should be turned
			 * off, and only the cut-off current should be set for
			 * the main charger.
			 */
			rc = oplus_chg_ic_func(
				vb->child_list[i].ic_dev,
				OPLUS_IC_FUNC_BUCK_SET_ITERM,
				iterm_ma);
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			return -EINVAL; /* TODO */
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
		if (rc < 0) {
			chg_err("child ic[%d] set fcc error, rc=%d\n", i, rc);
			return rc;
		}
	}

	return 0;
}

static int oplus_chg_vb_set_rechg_vol(struct oplus_chg_ic_dev *ic_dev, int vol_mv)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (vb->child_list[i].current_ratio == 0)
				continue;
			if (i != vb->main_charger)
				continue;
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_SET_RECHG_VOL)) {
				chg_err("main charger must support this function\n");
				return -EINVAL;
			}
			/*
			 * TODO:
			 * Pre-charging is only allowed by the main charger.
			 */
			rc = oplus_chg_ic_func(
				vb->child_list[i].ic_dev,
				OPLUS_IC_FUNC_BUCK_SET_RECHG_VOL,
				vol_mv);
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			return -EINVAL; /* TODO */
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
		if (rc < 0) {
			chg_err("child ic[%d] set fcc error, rc=%d\n", i, rc);
			return rc;
		}
	}

	return 0;
}

static int oplus_chg_vb_get_input_curr(struct oplus_chg_ic_dev *ic_dev, int *curr_ma)
{
	struct oplus_virtual_buck_ic *vb;
	int curr_temp;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	*curr_ma = 0;
	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (!func_is_support(&vb->child_list[i],
					     OPLUS_IC_FUNC_BUCK_GET_INPUT_CURR))
				continue;
			rc = oplus_chg_ic_func(
				vb->child_list[i].ic_dev,
				OPLUS_IC_FUNC_BUCK_GET_INPUT_CURR,
				&curr_temp);
			if (rc < 0) {
				chg_err("child ic[%d] get intput current error, rc=%d\n", i, rc);
				return rc;
			}
			*curr_ma += curr_temp;
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_GET_INPUT_CURR)) {
				chg_err("for serial connection, all ICs must support this function\n");
				return -EINVAL;
			}
			if (i == 0) {
				rc = oplus_chg_ic_func(
					vb->child_list[i].ic_dev,
					OPLUS_IC_FUNC_BUCK_GET_INPUT_CURR, curr_ma);
				if (rc < 0) {
					chg_err("child ic[%d] get intput current error, rc=%d\n", i, rc);
					return rc;
				}
				continue;
			}
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return rc;
}

static int oplus_chg_vb_get_input_vol(struct oplus_chg_ic_dev *ic_dev, int *vol_mv)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	*vol_mv = 0;
	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_GET_INPUT_VOL)) {
				rc = -ENOTSUPP;
				continue;
			}
			rc = oplus_chg_ic_func(
				vb->child_list[i].ic_dev,
				OPLUS_IC_FUNC_BUCK_GET_INPUT_VOL,
				vol_mv);
			if (rc < 0)
				chg_err("child ic[%d] get input voltage error, rc=%d\n", i, rc);
			else
				return 0;
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			if (i != 0)
				continue;
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_GET_INPUT_VOL)) {
				chg_err("for serial connection, first IC must support this function\n");
				return -EINVAL;
			}
			rc = oplus_chg_ic_func(
				vb->child_list[i].ic_dev,
				OPLUS_IC_FUNC_BUCK_GET_INPUT_VOL,
				vol_mv);
			if (rc < 0) {
				chg_err("child ic[%d] input voltage, rc=%d\n", i, rc);
				return rc;
			}
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return rc;
}

static int oplus_chg_vb_otg_boost_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_OTG_BOOST_ENABLE)) {
				rc = -ENOTSUPP;
				continue;
			}
			rc = oplus_chg_ic_func(
				vb->child_list[i].ic_dev,
				OPLUS_IC_FUNC_OTG_BOOST_ENABLE,
				en);
			if (rc < 0)
				chg_err("child ic[%d] set otg boost %s error, rc=%d\n", i, en ? "enable" : "disable", rc);
			else
				return 0;
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			if (i != 0)
				continue;
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_OTG_BOOST_ENABLE)) {
				chg_err("for serial connection, first IC must support this function\n");
				return -EINVAL;
			}
			rc = oplus_chg_ic_func(
				vb->child_list[i].ic_dev,
				OPLUS_IC_FUNC_OTG_BOOST_ENABLE,
				en);
			if (rc < 0) {
				chg_err("child ic[%d] set otg boost %s error, rc=%d\n", i, en ? "enable" : "disable", rc);
				return rc;
			}
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return rc;
}

static int oplus_chg_vb_set_otg_boost_vol(struct oplus_chg_ic_dev *ic_dev, int vol_mv)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_SET_OTG_BOOST_VOL)) {
				rc = -ENOTSUPP;
				continue;
			}
			rc = oplus_chg_ic_func(
				vb->child_list[i].ic_dev,
				OPLUS_IC_FUNC_SET_OTG_BOOST_VOL,
				vol_mv);
			if (rc < 0)
				chg_err("child ic[%d] set otg boost vol error, rc=%d\n", i, rc);
			else
				return 0;
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			if (i != 0)
				continue;
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_SET_OTG_BOOST_VOL)) {
				chg_err("for serial connection, first IC must support this function\n");
				return -EINVAL;
			}
			rc = oplus_chg_ic_func(
				vb->child_list[i].ic_dev,
				OPLUS_IC_FUNC_SET_OTG_BOOST_VOL,
				vol_mv);
			if (rc < 0) {
				chg_err("child ic[%d] set otg boost vol error, rc=%d\n", i, rc);
				return rc;
			}
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return rc;
}

static int oplus_chg_vb_set_otg_boost_curr_limit(struct oplus_chg_ic_dev *ic_dev, int curr_ma)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_SET_OTG_BOOST_CURR_LIMIT)) {
				rc = -ENOTSUPP;
				continue;
			}
			rc = oplus_chg_ic_func(
				vb->child_list[i].ic_dev,
				OPLUS_IC_FUNC_SET_OTG_BOOST_CURR_LIMIT,
				curr_ma);
			if (rc < 0)
				chg_err("child ic[%d] set otg boost curr limit error, rc=%d\n", i, rc);
			else
				return 0;
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			if (i != 0)
				continue;
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_SET_OTG_BOOST_CURR_LIMIT)) {
				chg_err("for serial connection, first IC must support this function\n");
				return -EINVAL;
			}
			rc = oplus_chg_ic_func(
				vb->child_list[i].ic_dev,
				OPLUS_IC_FUNC_SET_OTG_BOOST_CURR_LIMIT,
				curr_ma);
			if (rc < 0) {
				chg_err("child ic[%d] set otg boost error, rc=%d\n", i, rc);
				return rc;
			}
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return rc;
}

static int oplus_chg_vb_aicl_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	/* TODO */

	return rc;
}

static int oplus_chg_vb_aicl_rerun(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_AICL_RERUN)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_AICL_RERUN);
		if (rc < 0)
			chg_err("child ic[%d] rerun aicl error, rc=%d\n", i, rc);
		else
			return 0;
	}

	return rc;
}

static int oplus_chg_vb_aicl_reset(struct oplus_chg_ic_dev *ic_dev)
{
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	/* TODO */

	return rc;
}

static int oplus_chg_vb_get_cc_orientation(struct oplus_chg_ic_dev *ic_dev, int *orientation)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_GET_CC_ORIENTATION)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(
			vb->child_list[i].ic_dev,
			OPLUS_IC_FUNC_BUCK_GET_CC_ORIENTATION,
			orientation);
		if (rc < 0)
			chg_err("child ic[%d] get cc orientation error, rc=%d\n", i, rc);
		else
			return 0;
	}

	return rc;
}

#define VOLTAGE_3600MV  3600
#define HWDETECT_DONE_INTERVAL 200
#define HWDETECT_DONE_MAX_INTERVAL 5
static void oplus_audio_hwdetect_init_done(struct oplus_virtual_buck_ic *chip, int *detected)
{
	int i;
	int rc = 0;
	static bool first_check_high = false, first_check = false;
	static bool first_check_low = false, hwdetect_check_done = false;
	int vol_mv = 0;
	static unsigned long hwdetect_done_max_jiffies = 0, hwdetect_done_jiffies = 0;

	if (!first_check) {
		hwdetect_done_max_jiffies = jiffies +
			   (unsigned long)(HWDETECT_DONE_MAX_INTERVAL * HZ);
		first_check = true;
		if (*detected == 0)
			first_check_low = true;
	}
	if (first_check_low || *detected || hwdetect_check_done)
		return;
	if (time_is_before_jiffies(hwdetect_done_max_jiffies)) {
		hwdetect_check_done = true;
		return;
	}
	for (i = 0; i < chip->child_num; i++) {
		if (!func_is_support(&chip->child_list[i], OPLUS_IC_FUNC_BUCK_GET_INPUT_VOL)) {
			vol_mv = 0;
			continue;
		}
		rc = oplus_chg_ic_func(
			chip->child_list[i].ic_dev,
			OPLUS_IC_FUNC_BUCK_GET_INPUT_VOL,
			&vol_mv);
		if (rc < 0) {
			chg_err("child ic[%d] get hw detect error, rc=%d\n", i, rc);
			return;
		}
	}

	if (!first_check_high) {
		first_check_high = true;
		hwdetect_done_jiffies = jiffies + msecs_to_jiffies(HWDETECT_DONE_INTERVAL);
	}
	if (time_is_after_jiffies(hwdetect_done_jiffies) && vol_mv > VOLTAGE_3600MV)
		*detected = 1;
	else
		hwdetect_check_done = true;

	chg_info("hw_detect=%d, vol = %d, first_check_high = %d, hwdetect_check_done = %d\n",
		*detected, vol_mv, first_check_high, hwdetect_check_done);

}

static int oplus_chg_vb_get_hw_detect(struct oplus_chg_ic_dev *ic_dev, int *detected, bool recheck)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	if (oplus_vc_ccdetect_gpio_support(vb)) {
		*detected = !gpio_get_value(vb->misc_gpio.ccdetect_gpio);
		oplus_audio_hwdetect_init_done(vb, detected);
		chg_info("hw_detect=%d\n", *detected);
		return 0;
	}

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_GET_HW_DETECT)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(
			vb->child_list[i].ic_dev,
			OPLUS_IC_FUNC_BUCK_GET_HW_DETECT,
			detected, recheck);
		if (rc < 0)
			chg_err("child ic[%d] get hw detect error, rc=%d\n", i, rc);
		else
			return 0;
	}

	return rc;
}

static int oplus_chg_vb_get_charger_type(struct oplus_chg_ic_dev *ic_dev, int *type)
{
	struct oplus_virtual_buck_ic *vb;
	int chg_type;
	int i;
	int rc = 0;
	bool init = false;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_GET_CHARGER_TYPE)) {
			if (!init)
				rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(
			vb->child_list[i].ic_dev,
			OPLUS_IC_FUNC_BUCK_GET_CHARGER_TYPE,
			&chg_type);
		if (rc < 0) {
			chg_err("child ic[%d] get charger type error, rc=%d\n", i, rc);
			if (init)
				rc = 0;
			continue;
		} else {
			/* Priority support for PD adapters */
			if (chg_type == OPLUS_CHG_USB_TYPE_PD ||
			    chg_type == OPLUS_CHG_USB_TYPE_PD_DRP ||
			    chg_type == OPLUS_CHG_USB_TYPE_PD_PPS ||
			    chg_type == OPLUS_CHG_USB_TYPE_PD_SDP) {
				*type = chg_type;
				return 0;
			}
			if (!init)
				*type = chg_type;
			init = true;
			rc = 0;
		}
	}

	return rc;
}

static int oplus_chg_vb_rerun_bc12(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_RERUN_BC12)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(
			vb->child_list[i].ic_dev,
			OPLUS_IC_FUNC_BUCK_RERUN_BC12);
		if (rc < 0)
			chg_err("child ic[%d] rerun bc1.2 error, rc=%d\n", i, rc);
		else
			return 0;
	}

	return rc;
}

static int oplus_chg_vb_qc_detect_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_QC_DETECT_ENABLE)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(
			vb->child_list[i].ic_dev,
			OPLUS_IC_FUNC_BUCK_QC_DETECT_ENABLE,
			en);
		if (rc < 0)
			chg_err("child ic[%d] %s qc detect error, rc=%d\n", i, en ? "enable" : "disable", rc);
		else
			return 0;
	}

	return rc;
}

#define PWM_COUNT	5
static int oplus_chg_vb_shipmod_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chg_info("shipmode:%d\n", en);
	vb = oplus_chg_ic_get_drvdata(ic_dev);
	if (gpio_is_valid(vb->misc_gpio.ship_gpio)) {
		chg_info("select gpio ship mode control\n");
		pinctrl_select_state(vb->misc_gpio.pinctrl, vb->misc_gpio.ship_sleep);
		for (i = 0; i < PWM_COUNT; i++) {
			pinctrl_select_state(vb->misc_gpio.pinctrl, vb->misc_gpio.ship_active);
			mdelay(3);
			pinctrl_select_state(vb->misc_gpio.pinctrl, vb->misc_gpio.ship_sleep);
			mdelay(3);
		}
		chg_info("power off after 15s\n");
		return 0;
	}

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_SHIPMODE_ENABLE)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(
			vb->child_list[i].ic_dev,
			OPLUS_IC_FUNC_BUCK_SHIPMODE_ENABLE,
			en);

		if (rc < 0)
			chg_err("child ic[%d] %s shipmod error, rc=%d\n", i, en ? "enable" : "disable", rc);
		else
			chg_info("child ic[%d] %s shipmod success, rc=%d\n", i, en ? "enable" : "disable", rc);
	}

	return rc;
}

static int oplus_chg_vb_set_qc_config(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_qc_version version, int vol_mv)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_SET_QC_CONFIG)) {
			rc = (rc == 0) ? -ENOTSUPP : rc;
			continue;
		}
		rc = oplus_chg_ic_func(
			vb->child_list[i].ic_dev,
			OPLUS_IC_FUNC_BUCK_SET_QC_CONFIG,
			version, vol_mv);
		if (rc < 0)
			chg_err("child ic[%d] set qc to %dmV error, rc=%d\n", i, vol_mv, rc);
		else
			return 0;
	}

	return rc;
}

static int oplus_chg_vb_set_pd_config(struct oplus_chg_ic_dev *ic_dev, u32 pdo)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;
	int status = -ENOTSUPP;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_SET_PD_CONFIG)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(
			vb->child_list[i].ic_dev,
			OPLUS_IC_FUNC_BUCK_SET_PD_CONFIG,
			pdo);
		if (rc < 0) {
			chg_err("child ic[%d] set pdo(=0x%08x) error, rc=%d\n", i, pdo, rc);
			return rc;
		} else {
			status = 0;
		}
	}
	if (status == -ENOTSUPP && rc == -ENOTSUPP)
		return rc;
	else
		return 0;
}

static int oplus_chg_vb_wls_boost_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_WLS_BOOST_ENABLE)) {
				rc = -ENOTSUPP;
				continue;
			}
			rc = oplus_chg_ic_func(
				vb->child_list[i].ic_dev,
				OPLUS_IC_FUNC_WLS_BOOST_ENABLE,
				en);
			if (rc < 0)
				chg_err("child ic[%d] set wls boost %s error, rc=%d\n", i, en ? "enable" : "disable", rc);
			else
				return 0;
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			if (i != 0)
				continue;
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_WLS_BOOST_ENABLE)) {
				chg_err("for serial connection, first IC must support this function\n");
				return -EINVAL;
			}
			rc = oplus_chg_ic_func(
				vb->child_list[i].ic_dev,
				OPLUS_IC_FUNC_WLS_BOOST_ENABLE,
				en);
			if (rc < 0) {
				chg_err("child ic[%d] set wls boost %s error, rc=%d\n", i, en ? "enable" : "disable", rc);
				return rc;
			}
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return rc;
}

static int oplus_chg_vb_set_wls_boost_vol(struct oplus_chg_ic_dev *ic_dev, int vol_mv)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_SET_WLS_BOOST_VOL)) {
				rc = -ENOTSUPP;
				continue;
			}
			rc = oplus_chg_ic_func(
				vb->child_list[i].ic_dev,
				OPLUS_IC_FUNC_SET_WLS_BOOST_VOL,
				vol_mv);
			if (rc < 0)
				chg_err("child ic[%d] set wls boost vol error, rc=%d\n", i, rc);
			else
				return 0;
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			if (i != 0)
				continue;
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_SET_WLS_BOOST_VOL)) {
				chg_err("for serial connection, first IC must support this function\n");
				return -EINVAL;
			}
			rc = oplus_chg_ic_func(
				vb->child_list[i].ic_dev,
				OPLUS_IC_FUNC_SET_WLS_BOOST_VOL,
				vol_mv);
			if (rc < 0) {
				chg_err("child ic[%d] set wls boost vol error, rc=%d\n", i, rc);
				return rc;
			}
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return rc;
}

static int oplus_chg_vb_set_wls_boost_curr_limit(struct oplus_chg_ic_dev *ic_dev, int curr_ma)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_SET_WLS_BOOST_CURR_LIMIT)) {
				rc = -ENOTSUPP;
				continue;
			}
			rc = oplus_chg_ic_func(
				vb->child_list[i].ic_dev,
				OPLUS_IC_FUNC_SET_WLS_BOOST_CURR_LIMIT,
				curr_ma);
			if (rc < 0)
				chg_err("child ic[%d] set wls boost curr limit error, rc=%d\n", i, rc);
			else
				return 0;
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			if (i != 0)
				continue;
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_SET_WLS_BOOST_CURR_LIMIT)) {
				chg_err("for serial connection, first IC must support this function\n");
				return -EINVAL;
			}
			rc = oplus_chg_ic_func(
				vb->child_list[i].ic_dev,
				OPLUS_IC_FUNC_SET_WLS_BOOST_CURR_LIMIT,
				curr_ma);
			if (rc < 0) {
				chg_err("child ic[%d] set wls boost error, rc=%d\n", i, rc);
				return rc;
			}
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return rc;
}

static int oplus_chg_vb_gauge_update(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_GAUGE_UPDATE)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_GAUGE_UPDATE);
		if (rc < 0)
			chg_err("child ic[%d] gauge update error, rc=%d\n", i, rc);
		else
			return 0;
	}

	return rc;
}

static int oplus_chg_vb_voocphy_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_VOOCPHY_ENABLE)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_VOOCPHY_ENABLE, en);
		if (rc < 0)
			chg_err("child ic[%d] voocphy %s error, rc=%d\n", i, en ? "enable" : "disable", rc);

		return rc;
	}

	return rc;
}

static int oplus_chg_vb_voocphy_reset_again(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_VOOCPHY_RESET_AGAIN)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_VOOCPHY_RESET_AGAIN);
		if (rc < 0)
			chg_err("child ic[%d] voocphy reset again error, rc=%d\n", i, rc);

		return rc;
	}

	return rc;
}

static int oplus_chg_vb_get_charger_cycle(struct oplus_chg_ic_dev *ic_dev, int *cycle)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_GET_CHARGER_CYCLE)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_GET_CHARGER_CYCLE, cycle);
		if (rc < 0) {
			if (rc != -ENOTSUPP)
				chg_err("child ic[%d] get charger cycle error, rc=%d\n", i, rc);
		} else {
			return 0;
		}
	}

	return rc;
}

#define DATA_MASK	0x7F
#define DATA_IS_VALID	0x80
#define SOC_MAX		100
#define SOC_MIN		0

static int oplus_common_get_shutdown_soc(struct oplus_virtual_buck_ic *chip, int *get_soc)
{
	u8 *buf = NULL;
	size_t len;
	u8 soc = 0;
	int ret = 0;

	if (chip == NULL) {
		chg_err("oplus_virtual_buck_ic is NULL");
		return -ENODEV;
	}

	if (chip->soc_backup_nvmem) {
		buf = (u8 *)nvmem_cell_read(chip->soc_backup_nvmem, &len);
		if (IS_ERR_OR_NULL(buf)) {
			chg_err("failed to read nvmem cell\n");
			ret = PTR_ERR(buf);
			goto get_soc_end;
		}

		if (len <= 0 || len > sizeof(soc)) {
			chg_err("nvmem cell length out of range %lu\n", len);
			ret = -EINVAL;
			goto get_soc_end;
		}

		memcpy(&soc, buf, min(len, sizeof(soc)));
		if (((soc & DATA_IS_VALID) == 0)
				|| (soc & DATA_MASK) > SOC_MAX) {
			chg_err("soc 0x%02x invalid\n", soc);
			ret = -EINVAL;
			goto get_soc_end;
		}

		soc = soc & DATA_MASK;
		if (soc == SOC_MIN)
			soc = soc+1;

		*get_soc = soc;
		chg_info("get shutdown soc %d\n", *get_soc);
	}

get_soc_end:
	if (buf != NULL)
		kfree(buf);
	return ret;
}

static int oplus_common_backup_soc(struct oplus_virtual_buck_ic *chip, int soc)
{
	u8 back_soc = (u8)soc;
	int rc = 0;

	if (chip == NULL) {
		chg_err("oplus_virtual_buck_ic is NULL");
		return 0;
	}

	if (back_soc > SOC_MAX || back_soc < SOC_MIN) {
		chg_err("soc invalid\n");
		return -EINVAL;
	}

	if (chip->soc_backup_nvmem) {
		back_soc = (back_soc & DATA_MASK) | DATA_IS_VALID;
		rc = nvmem_cell_write(chip->soc_backup_nvmem, &back_soc, sizeof(back_soc));
		if (rc < 0)
			chg_err("store soc fail, rc=%d\n", rc);
		else
			chg_info("save soc %d sucess back_soc %x\n", soc, back_soc);
	}

	return rc;
}

static int oplus_chg_vb_get_shutdown_soc(struct oplus_chg_ic_dev *ic_dev, int *soc)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_GET_SHUTDOWN_SOC)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_GET_SHUTDOWN_SOC, soc);
		if (rc < 0 && rc != -ENOTSUPP)
			chg_err("child ic[%d] get shutdown soc error, rc=%d\n", i, rc);

		return rc;
	}

	if (rc == -ENOTSUPP && vb->soc_backup_nvmem) {
		rc = oplus_common_get_shutdown_soc(vb, soc);
	}

	return rc;
}

static int oplus_chg_vb_backup_soc(struct oplus_chg_ic_dev *ic_dev, int soc)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BACKUP_SOC)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_BACKUP_SOC, soc);
		if (rc < 0 && rc != -ENOTSUPP)
			chg_err("child ic[%d] backup soc error, rc=%d\n", i, rc);

		return rc;
	}

	if (rc == -ENOTSUPP && vb->soc_backup_nvmem) {
		rc = oplus_common_backup_soc(vb, soc);
	}

	return rc;
}

static int oplus_chg_vb_get_vbus_collapse_status(struct oplus_chg_ic_dev *ic_dev, bool *collapse)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_GET_VBUS_COLLAPSE_STATUS)) {
				rc = -ENOTSUPP;
				continue;
			}
			rc = oplus_chg_ic_func(
				vb->child_list[i].ic_dev,
				OPLUS_IC_FUNC_BUCK_GET_VBUS_COLLAPSE_STATUS,
				collapse);
			if (rc < 0)
				chg_err("child ic[%d] get vbus collapse status error, rc=%d\n", i, rc);
			else
				return 0;
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			if (i != 0)
				continue;
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_GET_VBUS_COLLAPSE_STATUS)) {
				chg_err("for serial connection, first IC must support this function\n");
				return -EINVAL;
			}
			rc = oplus_chg_ic_func(
				vb->child_list[i].ic_dev,
				OPLUS_IC_FUNC_BUCK_GET_VBUS_COLLAPSE_STATUS,
				collapse);
			if (rc < 0) {
				chg_err("child ic[%d] get vbus collapse status error, rc=%d\n", i, rc);
				return rc;
			}
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return rc;
}

#define USBTEMP_DEFAULT_VOLT_VALUE_MV 950

#if IS_ENABLED(CONFIG_HORAE_FLASH_LED_THERMAL)
static int oplus_chg_switch_usb_temp(struct oplus_virtual_buck_ic *chip, enum ntc_switch_type usbtemp_type, int *vol_val)
{
	int rc = 0;
	int usbtemp_volt;

	oplus_set_ntc_switch_lock(usbtemp_type, true);
	mdelay(10);

	switch (usbtemp_type) {
	case USBTEMP_L:
		rc = iio_read_channel_processed(chip->usbtemp_adc_l, &usbtemp_volt);
		break;
	case USBTEMP_R:
		rc = iio_read_channel_processed(chip->usbtemp_adc_r, &usbtemp_volt);
		break;
	default:
		usbtemp_volt = USBTEMP_DEFAULT_VOLT_VALUE_MV;
		break;
	}

	*vol_val = usbtemp_volt;
	oplus_set_ntc_switch_lock(usbtemp_type, false);
	return rc;
}
#endif

static int oplus_chg_vb_get_usb_temp_volt(struct oplus_chg_ic_dev *ic_dev, int *vol_l, int *vol_r)
{
	struct oplus_virtual_buck_ic *vb;
	static int usbtemp_volt_l_pre = USBTEMP_DEFAULT_VOLT_VALUE_MV;
	static int usbtemp_volt_r_pre = USBTEMP_DEFAULT_VOLT_VALUE_MV;
	int usbtemp_volt = 0;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		if (vol_l != NULL)
			*vol_l = usbtemp_volt_l_pre;
		if (vol_r != NULL)
			*vol_r = usbtemp_volt_r_pre;
		return 0;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	if (vol_l == NULL)
		goto usbtemp_next;

	if (IS_ERR_OR_NULL(vb->usbtemp_adc_l)) {
		*vol_l = usbtemp_volt_l_pre;
		goto usbtemp_next;
	}

#if IS_ENABLED(CONFIG_HORAE_FLASH_LED_THERMAL)
	if (vb->ntc_switch_gpio.ntc_switch_usbtemp_l)
		rc = oplus_chg_switch_usb_temp(vb, USBTEMP_L, &usbtemp_volt);
	else
		rc = iio_read_channel_processed(vb->usbtemp_adc_l, &usbtemp_volt);
#else
	rc = iio_read_channel_processed(vb->usbtemp_adc_l, &usbtemp_volt);
#endif

	if (rc < 0) {
		chg_err("usbtemp_volt_l read error\n");
		*vol_l = usbtemp_volt_l_pre;
		goto usbtemp_next;
	}

#ifndef CONFIG_OPLUS_CHARGER_MTK
	usbtemp_volt = vb->usbtemp_conversion_ratio * usbtemp_volt / 10000;
#endif
	if (usbtemp_volt > USBTEMP_DEFAULT_VOLT_VALUE_MV) {
		usbtemp_volt = USBTEMP_DEFAULT_VOLT_VALUE_MV;
	}

	*vol_l = usbtemp_volt;
	usbtemp_volt_l_pre = usbtemp_volt;

usbtemp_next:
	if (vol_r == NULL)
		return 0;

	usbtemp_volt = 0;
	if (IS_ERR_OR_NULL(vb->usbtemp_adc_r)) {
		*vol_r = usbtemp_volt_r_pre;
		return 0;
	}

#if IS_ENABLED(CONFIG_HORAE_FLASH_LED_THERMAL)
	if (vb->ntc_switch_gpio.ntc_switch_usbtemp_r)
		rc = oplus_chg_switch_usb_temp(vb, USBTEMP_R, &usbtemp_volt);
	else
		rc = iio_read_channel_processed(vb->usbtemp_adc_r, &usbtemp_volt);
#else
	rc = iio_read_channel_processed(vb->usbtemp_adc_r, &usbtemp_volt);
#endif

	if (rc < 0) {
		chg_err("usbtemp_volt_r read error\n");
		*vol_r = usbtemp_volt_r_pre;
		return 0;
	}

#ifndef CONFIG_OPLUS_CHARGER_MTK
	usbtemp_volt = vb->usbtemp_conversion_ratio * usbtemp_volt / 10000;
#endif
	if (usbtemp_volt > USBTEMP_DEFAULT_VOLT_VALUE_MV) {
		usbtemp_volt = USBTEMP_DEFAULT_VOLT_VALUE_MV;
	}

	*vol_r = usbtemp_volt;
	usbtemp_volt_r_pre = usbtemp_volt;

	return 0;
}

static int oplus_chg_vb_usb_temp_check_is_support(struct oplus_chg_ic_dev *ic_dev, bool *support)
{
	struct oplus_virtual_buck_ic *vb;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	*support = oplus_vc_usbtemp_check_is_support(vb);

	return 0;
}


static int oplus_chg_vb_get_typec_role(struct oplus_chg_ic_dev *ic_dev,
				       enum oplus_chg_typec_port_role_type *mode)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_GET_TYPEC_ROLE)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_GET_TYPEC_ROLE,
				       mode);
		if (rc < 0 && rc != -ENOTSUPP)
			chg_err("child ic[%d] get typec mode error, rc=%d\n", i, rc);
		return rc;
	}

	return rc;
}

static int oplus_chg_vb_get_power_role(struct oplus_chg_ic_dev *ic_dev,
				       int  *power_role)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = -ENOTSUPP;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_GET_POWER_ROLE)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_BUCK_GET_POWER_ROLE,
				       power_role);
		if (rc < 0) {
			if (rc != -ENOTSUPP)
				chg_err("child ic[%d] get power role error, rc=%d\n", i, rc);
			continue;
		}
		return 0;
	}
	if (rc == -ENOTSUPP)
		chg_err("no child ic support get power role function\n");
	return rc;
}

static int oplus_chg_vb_get_vdm_info(struct oplus_chg_ic_dev *ic_dev,
				     u32 *data, int *cnt)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = -ENOTSUPP;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_GET_VDM_INFO)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_BUCK_GET_VDM_INFO,
				       data, cnt);
		if (rc < 0) {
			if (rc != -ENOTSUPP)
				chg_err("child ic[%d] get vdm info error, rc=%d\n", i, rc);
			continue;
		}
		return 0;
	}
	if (rc == -ENOTSUPP)
		chg_err("no child ic support get vdm info function\n");
	return rc;
}

static int oplus_chg_vb_send_get_sink_cap(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = -ENOTSUPP;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	vb = oplus_chg_ic_get_drvdata(ic_dev);
	if (vb == NULL) {
		chg_err("oplus_virtual_buck_ic is NULL");
		return -ENODEV;
	}

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_SEND_GET_SINK_CAP)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_BUCK_SEND_GET_SINK_CAP);
		if (rc < 0) {
			if (rc != -ENOTSUPP)
				chg_err("child ic[%d] send get sink cap error, rc=%d\n", i, rc);
			continue;
		}
		return 0;
	}
	if (rc == -ENOTSUPP)
		chg_err("no child ic support get sink cap function\n");
	return rc;
}

static int oplus_chg_vb_get_typec_mode(struct oplus_chg_ic_dev *ic_dev,
				       enum oplus_chg_typec_port_role_type *mode)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_GET_TYPEC_MODE)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_GET_TYPEC_MODE,
				       mode);
		if (rc < 0 && rc != -ENOTSUPP)
			chg_err("child ic[%d] get typec mode error, rc=%d\n", i, rc);
		return rc;
	}

	return rc;
}

static int oplus_chg_vb_set_typec_mode(struct oplus_chg_ic_dev *ic_dev,
				       enum oplus_chg_typec_port_role_type mode)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_SET_TYPEC_MODE)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_SET_TYPEC_MODE,
				       mode);
		if (rc < 0)
			chg_err("child ic[%d] set typec mode(=%d) error, rc=%d\n", i, mode, rc);
		return rc;
	}

	return rc;
}

static int oplus_chg_vb_set_usb_dischg_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_SET_USB_DISCHG_ENABLE)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
					OPLUS_IC_FUNC_SET_USB_DISCHG_ENABLE,
					en);
		if (rc < 0)
			chg_err("child ic[%d] usbtemp disable en(=%d) error, rc=%d\n", i, en, rc);
		return rc;
	}

	if (IS_ERR_OR_NULL(vb->misc_gpio.pinctrl) ||
	    IS_ERR_OR_NULL(vb->misc_gpio.dischg_disable) ||
	    IS_ERR_OR_NULL(vb->misc_gpio.dischg_enable)) {
		chg_err("pinctrl, dischg_disable or dischg_enable is NULL\n");
		return -EINVAL;
	}

	rc = pinctrl_select_state(vb->misc_gpio.pinctrl,
				  en ? vb->misc_gpio.dischg_enable :
					     vb->misc_gpio.dischg_disable);
	if (rc < 0)
		chg_err("can't %s usb dischg, rc=%d\n", en ? "enable" : "disable", rc);

	return rc;
}

static int oplus_chg_vb_get_usb_dischg_status(struct oplus_chg_ic_dev *ic_dev, bool *en)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_GET_USB_DISCHG_STATUS)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
					OPLUS_IC_FUNC_GET_USB_DISCHG_STATUS,
					en);
		if (rc < 0)
			chg_err("child ic[%d] get usb dischg status error, rc=%d\n", i, rc);
		return rc;
	}

	if(!gpio_is_valid(vb->misc_gpio.dischg_gpio))
		return -ENOTSUPP;
	*en = !!gpio_get_value(vb->misc_gpio.dischg_gpio);

	return 0;
}

static int oplus_chg_vb_set_otg_switch_status(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int detect;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_SET_OTG_SWITCH_STATUS)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_SET_OTG_SWITCH_STATUS,
				       en);
		if (rc < 0)
			chg_err("child ic[%d] %s otg switch error, rc=%d\n", i, en ? "enable" : "disable", rc);
		else
			vb->otg_switch = en;

		return rc;
	}

	if (rc == -ENOTSUPP) {
		rc = oplus_chg_vb_get_hw_detect(ic_dev, &detect, false);
		if (rc < 0 && rc != -ENOTSUPP) {
			chg_err("can't get hw detect status, rc=%d\n", rc);
			return rc;
		}
		if (detect && rc != -ENOTSUPP) {
			chg_err("hw_detect=1, shouldn't set");
			vb->otg_switch = en;
			return 0;
		}

		rc = oplus_ccdetect_enable(vb, en);
		if (rc < 0)
			chg_err("%s ccdetect error, rc=%d\n", en ? "enable" : "disable", rc);
		else
			vb->otg_switch = en;
	}

	return rc;
}

static int oplus_chg_vb_get_otg_switch_status(struct oplus_chg_ic_dev *ic_dev, bool *en)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_GET_OTG_SWITCH_STATUS)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_GET_OTG_SWITCH_STATUS,
				       en);
		if (rc < 0)
			chg_err("child ic[%d] get otg switch status error, rc=%d\n", i, rc);
		return rc;
	}

	if (rc == -ENOTSUPP) {
		*en = vb->otg_switch;
		return 0;
	}

	return rc;
}

static int oplus_chg_vb_get_otg_online_status(struct oplus_chg_ic_dev *ic_dev, int *status)
{
	struct oplus_virtual_buck_ic *vb;
	int online = 0;
	enum oplus_chg_typec_port_role_type typec_mode;
	int hw_detect = 0;
	int level;
	bool typec_otg;
	bool support_hw_detect;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	if (oplus_vc_ccdetect_gpio_support(vb)) {
		level = gpio_get_value(vb->misc_gpio.ccdetect_gpio);
		if (level != gpio_get_value(vb->misc_gpio.ccdetect_gpio)) {
			chg_info("ccdetect_gpio is unstable, try again...\n");
			usleep_range(5000, 5100);
			level = gpio_get_value(vb->misc_gpio.ccdetect_gpio);
		}
		online = (level == 1) ? DISCONNECT : STANDARD_TYPEC_DEV_CONNECT;
	} else {
		/* The error returned may be that this function is not supported */
		rc = oplus_chg_vb_get_hw_detect(ic_dev, &hw_detect, false);
		if (rc < 0) {
			if (rc != -ENOTSUPP) {
				chg_err("get hw detect status error, rc=%d\n",
					rc);
				return rc;
			}
		} else {
			online = (hw_detect == 1) ? STANDARD_TYPEC_DEV_CONNECT : DISCONNECT;
		}
	}

	if (rc != -ENOTSUPP)
		support_hw_detect = true;
	else
		support_hw_detect = false;

	rc = oplus_chg_vb_get_typec_mode(ic_dev, &typec_mode);
	if (rc < 0) {
		if (rc != -ENOTSUPP) {
			chg_err("get typec mode error, rc=%d\n", rc);
			return rc;
		}
		typec_mode = TYPEC_PORT_ROLE_INVALID;
	}

	typec_otg = (typec_mode == TYPEC_PORT_ROLE_SRC);
	if (support_hw_detect) {
		online = online | (typec_otg ? OTG_DEV_CONNECT : DISCONNECT);
	} else {
		online = online | (typec_otg ? STANDARD_TYPEC_DEV_CONNECT : DISCONNECT);
	}
	*status = online;

	return rc;
}

static int oplus_chg_vb_cc_detect_happened(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	vb = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_CC_DETECT_HAPPENED)) {
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_CC_DETECT_HAPPENED);
		if (rc < 0)
			chg_err("child ic[%d] send cc detect happend error, rc=%d\n", i, rc);
	}

	return rc;
}

static int oplus_chg_vb_curr_drop(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	vb = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_CURR_DROP))
				continue;
			if (vb->child_list[i].current_ratio == 0)
				continue;
			rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
					       OPLUS_IC_FUNC_BUCK_CURR_DROP);
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			if (i != 0)
				continue; /* only need to process the first IC */
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_CURR_DROP)) {
				return 0;
			}
			rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
					       OPLUS_IC_FUNC_BUCK_CURR_DROP);
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
		if (rc < 0) {
			chg_err("child ic[%d] set curr drop error, rc=%d\n", i, rc);
			return rc;
		}
	}

	return rc;
}

static int oplus_chg_vb_wdt_enable(struct oplus_chg_ic_dev *ic_dev, bool enable)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	vb = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_WDT_ENABLE))
			continue;
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_BUCK_WDT_ENABLE,
				       enable);
		if (rc < 0) {
			chg_err("child ic[%d] %s wdt error, rc=%d\n", i, enable ? "enable" : "disable", rc);
			return rc;
		}
	}

	return rc;
}

static int oplus_chg_vb_kick_wdt(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;
	int err = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	vb = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i],
				     OPLUS_IC_FUNC_BUCK_KICK_WDT))
			continue;
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_BUCK_KICK_WDT);
		if (rc < 0) {
			chg_err("child ic[%d] kick wdt error, rc=%d\n", i, rc);
			err = rc;
		}
	}

	return err;
}

static int oplus_chg_vb_set_aicl_point(struct oplus_chg_ic_dev *ic_dev, int vbatt_mv)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;
	int err = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i],
				     OPLUS_IC_FUNC_BUCK_SET_AICL_POINT))
			continue;
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_BUCK_SET_AICL_POINT, vbatt_mv);
		if (rc < 0) {
			chg_err("child ic[%d] set aicl point error, rc=%d\n", i, rc);
			err = rc;
		}
	}

	return err;
}

static int oplus_chg_vb_set_vindpm(struct oplus_chg_ic_dev *ic_dev, int vol_mv)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;
	int err = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i],
				     OPLUS_IC_FUNC_BUCK_SET_VINDPM))
			continue;
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_BUCK_SET_VINDPM, vol_mv);
		if (rc < 0) {
			chg_err("child ic[%d] set vindpm error, rc=%d\n", i, rc);
			err = rc;
		}
	}

	return err;
}

static int oplus_chg_vb_hardware_init(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	vb = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i],  OPLUS_IC_FUNC_BUCK_HARDWARE_INIT))
			continue;
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				        OPLUS_IC_FUNC_BUCK_HARDWARE_INIT);
		if (rc < 0)
			chg_err("child ic[%d] set hardware init error, rc=%d\n", i, rc);
	}

	return rc;
}


static int oplus_chg_vb_set_curr_level(struct oplus_chg_ic_dev *ic_dev, int cool_down)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	vb = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_VOOCPHY_SET_CURR_LEVEL))
			continue;
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_VOOCPHY_SET_CURR_LEVEL,
				       cool_down);
		if (rc < 0) {
			chg_err("child ic[%d] set current level to %d error, rc=%d\n", i, cool_down, rc);
			return rc;
		}
	}

	return rc;
}

static int oplus_chg_vb_set_match_temp(struct oplus_chg_ic_dev *ic_dev, int match_temp)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	vb = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_VOOCPHY_SET_MATCH_TEMP))
			continue;
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_VOOCPHY_SET_MATCH_TEMP,
				       match_temp);
		if (rc < 0) {
			chg_err("child ic[%d] set match temp to %d error, rc=%d\n", i, match_temp, rc);
			return rc;
		}
	}

	return rc;
}

static int oplus_chg_vb_get_otg_enable(struct oplus_chg_ic_dev *ic_dev, bool *enable)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	vb = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_GET_OTG_ENABLE)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_GET_OTG_ENABLE,
				       enable);
		if (rc < 0) {
			chg_err("child ic[%d] can't get otg enable status, rc=%d\n", i, rc);
			return rc;
		}
		break;
	}

	return rc;
}

static int oplus_chg_vb_get_source_pdo(struct oplus_chg_ic_dev *ic_dev, u32 *data, int *num)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	vb = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_GET_SOURCE_PDO)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_GET_SOURCE_PDO,
				       data, num);
		if (rc < 0) {
			chg_err("child ic[%d] can't get pdo volt status, rc=%d\n", i, rc);
			return rc;
		}
		break;
	}

	return rc;
}

static int oplus_chg_vb_get_charger_vol_max(struct oplus_chg_ic_dev *ic_dev, int *vol)
{
	struct oplus_virtual_buck_ic *vb;
	int i, vol_tmp;
	int rc = 0;
	bool init = false;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	vb = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_GET_CHARGER_VOL_MAX)) {
			if (!init)
				rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_GET_CHARGER_VOL_MAX, &vol_tmp);
		if (rc < 0) {
			chg_err("child ic[%d] get charger vol max error, rc=%d\n", i, rc);
			return rc;
		}

		rc = 0;
		if (!init) {
			*vol = vol_tmp;
			init = true;
		} else {
			if (vol_tmp < *vol)
				*vol = vol_tmp;
		}
	}

	return rc;
}

static int oplus_chg_vb_get_charger_vol_min(struct oplus_chg_ic_dev *ic_dev, int *vol)
{
	struct oplus_virtual_buck_ic *vb;
	int i, vol_tmp;
	int rc = 0;
	bool init = false;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	vb = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_GET_CHARGER_VOL_MIN)) {
			if (!init)
				rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_GET_CHARGER_VOL_MIN, &vol_tmp);
		if (rc < 0) {
			chg_err("child ic[%d] get charger vol min error, rc=%d\n", i, rc);
			return rc;
		}

		rc = 0;
		if (!init) {
			*vol = vol_tmp;
			init = true;
		} else {
			if (vol_tmp > *vol)
				*vol = vol_tmp;
		}
	}

	return rc;
}

static int oplus_chg_vb_get_charger_curr_max(struct oplus_chg_ic_dev *ic_dev, int *curr)
{
	struct oplus_virtual_buck_ic *vb;
	int i, curr_tmp;
	int rc = 0;
	bool init = false;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	vb = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_GET_CHARGER_CURR_MAX)) {
			if (!init)
				rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_GET_CHARGER_CURR_MAX, &curr_tmp);
		if (rc < 0) {
			chg_err("child ic[%d] get charger curr max error, rc=%d\n", i, rc);
			return rc;
		}

		rc = 0;
		if (!init) {
			*curr = curr_tmp;
			init = true;
		} else {
			if (curr_tmp < *curr)
				*curr = curr_tmp;
		}
	}

	return rc;
}

static int oplus_chg_vb_bc12_completed(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	vb = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_BC12_COMPLETED))
			continue;
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_BUCK_BC12_COMPLETED);
		if (rc < 0)
			chg_err("child ic[%d] set bc1.2 completed error, rc=%d\n", i, rc);
	}

	return rc;
}

static int oplus_chg_vb_disable_vbus(struct oplus_chg_ic_dev *ic_dev, bool en, bool delay)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i],
				     OPLUS_IC_FUNC_DISABLE_VBUS))
			continue;
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_DISABLE_VBUS, en, delay);
		if (rc < 0)
			chg_err("child ic[%d] %s vbus error, rc=%d\n", i,
				en ? "disable" : "enable", rc);
	}

	return rc;
}

static int oplus_chg_vb_is_oplus_svid(struct oplus_chg_ic_dev *ic_dev,
				      bool *oplus_svid)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	vb = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i],
				     OPLUS_IC_FUNC_IS_OPLUS_SVID)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_IS_OPLUS_SVID, oplus_svid);
		if (rc < 0)
			chg_err("child ic[%d] can't get oplus_svid status, rc=%d\n",
				i, rc);
		break;
	}

	return rc;
}

static int oplus_chg_vb_get_data_role(struct oplus_chg_ic_dev *ic_dev,
				      int *role)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	vb = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i],
				     OPLUS_IC_FUNC_GET_DATA_ROLE)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_GET_DATA_ROLE, role);
		if (rc < 0)
			chg_err("child ic[%d] can't get typec data role, rc=%d\n",
				i, rc);
		break;
	}

	return rc;
}

static int oplus_chg_vb_get_typec_state(struct oplus_chg_ic_dev *ic_dev,
				      int *state)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	vb = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i],
				     OPLUS_IC_FUNC_BUCK_GET_TYPEC_STATE)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_BUCK_GET_TYPEC_STATE, state);
		if (rc < 0)
			chg_err("child ic[%d] can't get typec state, rc=%d\n",
				i, rc);
		break;
	}

	return rc;
}

static int oplus_chg_vb_get_usb_btb_temp(struct oplus_chg_ic_dev *ic_dev,
					 int *usb_btb_temp)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	vb = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i],
				     OPLUS_IC_FUNC_BUCK_GET_USB_BTB_TEMP)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_BUCK_GET_USB_BTB_TEMP,
				       usb_btb_temp);
		if (rc < 0)
			chg_err("child ic[%d] can't get usb btb temp, rc=%d\n",
				i, rc);
		break;
	}

	return rc;
}

static int oplus_chg_vb_get_vbat_pwr(struct oplus_chg_ic_dev *ic_dev,
					  int *vbat_pwr)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	vb = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i],
				     OPLUS_IC_FUNC_BUCK_GET_VBAT_PWR)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_BUCK_GET_VBAT_PWR,
				       vbat_pwr);
		if (rc < 0)
			chg_err("child ic[%d] can't get vbat pwr, rc=%d\n",
				i, rc);
		break;
	}

	return rc;
}

static int oplus_chg_vb_get_batt_btb_temp(struct oplus_chg_ic_dev *ic_dev,
					  int *batt_btb_temp)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	vb = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i],
				     OPLUS_IC_FUNC_BUCK_GET_BATT_BTB_TEMP)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_BUCK_GET_BATT_BTB_TEMP,
				       batt_btb_temp);
		if (rc < 0)
			chg_err("child ic[%d] can't get batt btb temp, rc=%d\n",
				i, rc);
		break;
	}

	return rc;
}

static int oplus_chg_vb_get_shaft_btb_temp(struct oplus_chg_ic_dev *ic_dev,
					  int *shaft_btb_tbat)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	vb = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_GET_SHAFT_BATT_BTB_TEMP)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_GET_SHAFT_BATT_BTB_TEMP, shaft_btb_tbat);
		if (rc < 0)
			chg_err("child ic[%d] can't get shaft btb status, rc=%d\n", i, rc);
		break;
	}

	return rc;
}

int oplus_chg_vb_set_shfat_btb_over_status(struct oplus_chg_ic_dev *ic_dev, bool is_shaft_btb_over)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_PUSH_SHAFT_BATT_BTB_OVER)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_PUSH_SHAFT_BATT_BTB_OVER,
			is_shaft_btb_over);
		if (rc < 0) {
			chg_err("child ic[%d] set shfat btb status error, rc=%d\n", i, rc);
			return rc;
		}
	}

	return 0;
}

static int oplus_chg_vb_get_fv(struct oplus_chg_ic_dev *ic_dev,
					  int *fv_ma)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	vb = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i],
				     OPLUS_IC_FUNC_BUCK_GET_FV)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_BUCK_GET_FV,
				       fv_ma);
		if (rc < 0)
			chg_err("child ic[%d] can't get fv, rc=%d\n",
				i, rc);
		break;
	}

	return rc;
}

static int oplus_chg_vb_wls_input_suspend(struct oplus_chg_ic_dev *ic_dev, bool suspend)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;
	bool suspend_temp;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_WLS_INPUT_SUSPEND))
				continue;
			if (vb->child_list[i].current_ratio == 0)
				suspend_temp = true;
			else
				suspend_temp = suspend;
			rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				OPLUS_IC_FUNC_BUCK_WLS_INPUT_SUSPEND, suspend_temp);
			if (rc < 0) {
				chg_err("child ic[%d] wls input %s error, rc=%d\n",
					i, suspend_temp ? "suspend" : "unsuspend", rc);
				return rc;
			}
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_WLS_INPUT_SUSPEND)) {
				chg_err("for serial connection, all ICs must support this function\n");
				return -EINVAL;
			}
			rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_WLS_INPUT_SUSPEND, suspend);
			if (rc < 0) {
				chg_err("child ic[%d] wls input %s error, rc=%d\n",
					i, suspend ? "suspend" : "unsuspend", rc);
				return rc;
			}
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return 0;
}

static int oplus_chg_vb_set_wls_icl(struct oplus_chg_ic_dev *ic_dev, int icl_ma)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_SET_WLS_ICL))
				continue;
			rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_SET_WLS_ICL,
				icl_ma * vb->child_list[i].current_ratio / 100);
			if (rc < 0) {
				chg_err("child ic[%d] set wls icl error, rc=%d\n", i, rc);
				return rc;
			}
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_SET_WLS_ICL)) {
				chg_err("for serial connection, all ICs must support this function\n");
				return -EINVAL;
			}
			return -EINVAL; /* TODO */
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return 0;
}

static int oplus_chg_vb_get_wls_icl(struct oplus_chg_ic_dev *ic_dev, int *icl_ma)
{
	struct oplus_virtual_buck_ic *vb;
	int temp_icl_ma;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	*icl_ma = 0;
	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_GET_WLS_ICL))
				continue;
			rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_GET_WLS_ICL, &temp_icl_ma);
			if (rc < 0) {
				chg_err("child ic[%d] get wls icl error, rc=%d\n", i, rc);
				return rc;
			}
			*icl_ma += temp_icl_ma;
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_GET_WLS_ICL)) {
				chg_err("for serial connection, all ICs must support this function\n");
				return -EINVAL;
			}
			if (i == 0) {
				rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
					OPLUS_IC_FUNC_BUCK_GET_WLS_ICL, icl_ma);
				if (rc < 0) {
					chg_err("child ic[%d] get wls icl error, rc=%d\n", i, rc);
					return rc;
				}
				continue;
			}
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return 0;
}

static int oplus_chg_vb_get_wls_input_curr(struct oplus_chg_ic_dev *ic_dev, int *curr_ma)
{
	struct oplus_virtual_buck_ic *vb;
	int curr_temp;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	*curr_ma = 0;
	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_GET_WLS_INPUT_CURR))
				continue;
			rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				OPLUS_IC_FUNC_BUCK_GET_WLS_INPUT_CURR, &curr_temp);
			if (rc < 0) {
				chg_err("child ic[%d] get wls intput current error, rc=%d\n", i, rc);
				return rc;
			}
			*curr_ma += curr_temp;
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_GET_WLS_INPUT_CURR)) {
				chg_err("for serial connection, all ICs must support this function\n");
				return -EINVAL;
			}
			if (i == 0) {
				rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
					OPLUS_IC_FUNC_BUCK_GET_WLS_INPUT_CURR, curr_ma);
				if (rc < 0) {
					chg_err("child ic[%d] get wls intput current error, rc=%d\n", i, rc);
					return rc;
				}
				continue;
			}
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return rc;
}

static int oplus_chg_vb_get_wls_input_vol(struct oplus_chg_ic_dev *ic_dev, int *vol_mv)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	*vol_mv = 0;
	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_GET_WLS_INPUT_VOL)) {
				rc = -ENOTSUPP;
				continue;
			}
			rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_GET_WLS_INPUT_VOL, vol_mv);
			if (rc < 0)
				chg_err("child ic[%d] get wls input voltage error, rc=%d\n", i, rc);
			else
				return 0;
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			if (i != 0)
				continue;
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_GET_WLS_INPUT_VOL)) {
				chg_err("for serial connection, first IC must support this function\n");
				return -EINVAL;
			}
			rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_GET_WLS_INPUT_VOL, vol_mv);
			if (rc < 0) {
				chg_err("child ic[%d] wls input voltage, rc=%d\n", i, rc);
				return rc;
			}
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return rc;
}

static int oplus_chg_vb_wls_aicl_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_WLS_AICL_ENABLE)) {
				rc = -ENOTSUPP;
				continue;
			}
			rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_WLS_AICL_ENABLE, en);
			if (rc < 0)
				chg_err("child ic[%d] wls aicl %s error, rc=%d\n", i, en ? "enable" : "disable", rc);
			else
				return 0;
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			if (i != 0)
				continue;
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_WLS_AICL_ENABLE)) {
				chg_err("for serial connection, first IC must support this function\n");
				return -EINVAL;
			}
			rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_WLS_AICL_ENABLE, en);
			if (rc < 0) {
				chg_err("child ic[%d] wls aicl %s error, rc=%d\n", i, en ? "enable" : "disable", rc);
				return rc;
			}
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return rc;
}

static int oplus_chg_vb_set_usb_drv(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_DIS_INSERT_DETECT)) {
				rc = -ENOTSUPP;
				continue;
			}
			rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_DIS_INSERT_DETECT, en);
			if (rc < 0)
				chg_err("child ic[%d] usb drv %s error, rc=%d\n", i, en ? "enable" : "disable", rc);
			else
				return 0;
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			if (i != 0)
				continue;
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_DIS_INSERT_DETECT)) {
				chg_err("for serial connection, first IC must support this function\n");
				return -EINVAL;
			}
			rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_DIS_INSERT_DETECT, en);
			if (rc < 0) {
				chg_err("child ic[%d] usb drv %s error, rc=%d\n", i, en ? "enable" : "disable", rc);
				return rc;
			}
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return rc;
}

static int oplus_chg_vb_wls_aicl_rerun(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (vb->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_WLS_AICL_RERUN)) {
				rc = -ENOTSUPP;
				continue;
			}
			if (vb->child_list[i].current_ratio == 0)
				continue;
			rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_WLS_AICL_RERUN);
			if (rc < 0)
				chg_err("child ic[%d] wls aicl rerun error, rc=%d\n", i, rc);
			else
				return 0;
		} else if (vb->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			if (i != 0)
				continue;
			if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_WLS_AICL_RERUN)) {
				chg_err("for serial connection, first IC must support this function\n");
				return -EINVAL;
			}
			rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_WLS_AICL_RERUN);
			if (rc < 0) {
				chg_err("child ic[%d] wls aicl rerun error, rc=%d\n", i, rc);
				return rc;
			}
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return rc;
}

static int oplus_chg_vb_get_byb_id_info(struct oplus_chg_ic_dev *ic_dev, int *count)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	*count = 0;
	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_GET_BYBID_INFO)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_GET_BYBID_INFO, count);
		if (rc < 0)
			chg_err("child ic[%d] get bybid info error, rc=%d\n", i, rc);
		else
			return 0;
	}

	return rc;
}

static int oplus_chg_vb_get_byb_id_match_info(struct oplus_chg_ic_dev *ic_dev, int *count)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	*count = 0;
	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_GET_BYBID_MATCH_INFO)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_GET_BYBID_MATCH_INFO, count);
		if (rc < 0)
			chg_err("child ic[%d] get bybid info error, rc=%d\n", i, rc);
		else
			return 0;
	}

	return rc;
}

static int oplus_chg_vb_fpga_rst(struct oplus_chg_ic_dev *ic_dev, int type)
{
	struct oplus_virtual_buck_ic *chip;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip || !chip->child_list) {
		chg_err("chip or chip->child_list is NULL");
		return -ENODEV;
	}

	for (i = 0; i < chip->child_num; i++) {
		if (!func_is_support(&chip->child_list[i],
				     OPLUS_IC_FUNC_BUCK_FPGA_RST)) {
			rc = (rc == 0) ? -ENOTSUPP : rc;
			chg_err("unsupport, i=%d rc=%d\n", i, rc);
			continue;
		}

		rc = oplus_chg_ic_func(chip->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_BUCK_FPGA_RST, type);
		if (rc < 0)
			chg_err("child ic[%d] OPLUS_IC_FUNC_BUCK_FPGA_RST error, rc=%d\n", i, rc);
		break;
	}

	return rc;
}

static int oplus_chg_vb_get_usb_aicl_enhance(struct oplus_chg_ic_dev *ic_dev, bool *enable)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	vb = oplus_chg_ic_get_drvdata(ic_dev);
	if (!vb || !vb->child_list) {
		chg_err("chip or chip->child_list is NULL");
		return -ENODEV;
	}

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_GET_USB_AICL_ENHANCE)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_BUCK_GET_USB_AICL_ENHANCE,
				       enable);
		if (rc < 0) {
			chg_err("child ic[%d] can't get otg enable status, rc=%d\n", i, rc);
			return rc;
		}
		break;
	}

	return rc;
}

static int oplus_chg_vb_get_lpd_info(struct oplus_chg_ic_dev *ic_dev, u32 *buffer, u32 flag)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL || buffer == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_GET_LPD_INFO)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(
			vb->child_list[i].ic_dev,
			OPLUS_IC_FUNC_BUCK_GET_LPD_INFO,
			buffer, flag);
		if (rc < 0)
			chg_err("child ic[%d] get lpd info error, rc=%d\n", i, rc);
		else
			return 0;
	}

	return rc;
}

int oplus_chg_vb_set_flash_mode(struct oplus_chg_ic_dev *ic_dev, bool flash_mode)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_SET_FLASH_MODE)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(
			vb->child_list[i].ic_dev,
			OPLUS_IC_FUNC_BUCK_SET_FLASH_MODE,
			flash_mode);
		if (rc < 0)
			chg_err("child ic[%d] set flash mode error, rc=%d\n", i, rc);
		else
			return 0;
	}

	return rc;
}

static int oplus_chg_vb_set_chg_path(struct oplus_chg_ic_dev *ic_dev, int path)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_SET_CHG_PATH)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_SET_CHG_PATH, path);
		if (rc < 0) {
			chg_err("child ic[%d] set chg path error, rc=%d\n", i, rc);
			return rc;
		}
	}

	return rc;
}

static int oplus_chg_vb_get_chg_path_status(struct oplus_chg_ic_dev *ic_dev, int *status)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	*status = 0;
	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_GET_CHG_PATH_STATUS)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_GET_CHG_PATH_STATUS, status);
		if (rc < 0)
			chg_err("child ic[%d] get chg path status error, rc=%d\n", i, rc);
		else
			return 0;
	}

	return rc;
}

static int oplus_chg_vb_iterm_check(struct oplus_chg_ic_dev *ic_dev, bool check)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_ITEM_CHECK)) {
			chg_err("for serial connection, all ICs must support this function\n");
			return -EINVAL;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_ITEM_CHECK, check);
		if (rc < 0) {
			chg_err("child ic[%d] iterm check %d error, rc=%d\n", i, check, rc);
			return rc;
		}
	}

	return 0;
}

static int oplus_chg_vb_set_power_mos(struct oplus_chg_ic_dev *ic_dev, bool enable)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);

	if (!vb || !vb->child_list) {
		chg_err("vb or child_list is NULL");
		return -ENODEV;
	}

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_SET_POWER_MOS_ENABLE)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_SET_POWER_MOS_ENABLE, enable);
		if (rc < 0) {
			chg_err("child ic[%d] iterm check %d error, rc=%d\n", i, enable, rc);
			return rc;
		}
	}

	return 0;
}

static int oplus_chg_vb_get_power_mos_status(struct oplus_chg_ic_dev *ic_dev, bool *enable)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	*enable = 0;
	vb = oplus_chg_ic_get_drvdata(ic_dev);

	if (!vb || !vb->child_list) {
		chg_err("vb or child_list is NULL");
		return -ENODEV;
	}

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_GET_POWER_MOS_ENABLE)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_GET_POWER_MOS_ENABLE, enable);
		if (rc < 0)
			chg_err("child ic[%d] get supplementary power mos status error, rc=%d\n", i, rc);
		else
			return 0;
	}

	return rc;
}

int oplus_chg_vb_set_adsp_ovp(struct oplus_chg_ic_dev *ic_dev, bool enable)
{
	struct oplus_virtual_buck_ic *chip;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (chip == NULL) {
		chg_err("feature chip is NULL\n");
		return -ENODEV;
	}
	if (chip->child_list == NULL) {
		chg_err("child_list is NULL\n");
		return -ENODEV;
	}

	for (i = 0; i < chip->child_num; i++) {
		if (!func_is_support(&chip->child_list[i], OPLUS_IC_FUNC_BUCK_SET_OVP_FORCED)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(
			chip->child_list[i].ic_dev,
			OPLUS_IC_FUNC_BUCK_SET_OVP_FORCED,
			enable);
		if (rc < 0)
			chg_err("child ic[%d] set OVP forced to %d error, rc=%d\n", i, enable, rc);
		else
			return 0;
	}

	return rc;
}

static int oplus_set_usb_dpdm_ovp_disable(struct oplus_chg_ic_dev *ic_dev, bool disable)
{
	struct oplus_virtual_buck_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	if (!vb || !vb->child_list) {
		chg_err("vb or child_list is NULL");
		return -ENODEV;
	}

	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_BUCK_SET_DPDM_OVP_DISABLE)) {
			rc = -ENOTSUPP;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_SET_DPDM_OVP_DISABLE, disable);
		if (rc < 0)
			chg_err("child ic[%d] OPLUS_IC_FUNC_BUCK_SET_DPDM_OVP_DISABLE error, rc=%d\n", i, rc);
		else
			return 0;
	}
	return rc;
}

static void *oplus_chg_vb_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, oplus_chg_vb_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, oplus_chg_vb_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP, oplus_chg_vb_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST, oplus_chg_vb_smt_test);
		break;
	case OPLUS_IC_FUNC_BUCK_INPUT_PRESENT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_INPUT_PRESENT, oplus_chg_vb_input_present);
		break;
	case OPLUS_IC_FUNC_BUCK_INPUT_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_INPUT_SUSPEND, oplus_chg_vb_input_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_INPUT_IS_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_INPUT_IS_SUSPEND, oplus_chg_vb_input_is_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_OUTPUT_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_OUTPUT_SUSPEND, oplus_chg_vb_output_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_OUTPUT_IS_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_OUTPUT_IS_SUSPEND, oplus_chg_vb_output_is_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_ICL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_ICL, oplus_chg_vb_set_icl);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_ICL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_ICL, oplus_chg_vb_get_icl);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_FCC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_FCC, oplus_chg_vb_set_fcc);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_FV:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_FV, oplus_chg_vb_set_fv);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_ITERM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_ITERM, oplus_chg_vb_set_iterm);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_RECHG_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_RECHG_VOL, oplus_chg_vb_set_rechg_vol);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_INPUT_CURR:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_INPUT_CURR, oplus_chg_vb_get_input_curr);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_INPUT_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_INPUT_VOL, oplus_chg_vb_get_input_vol);
		break;
	case OPLUS_IC_FUNC_OTG_BOOST_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_OTG_BOOST_ENABLE, oplus_chg_vb_otg_boost_enable);
		break;
	case OPLUS_IC_FUNC_SET_OTG_BOOST_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_OTG_BOOST_VOL, oplus_chg_vb_set_otg_boost_vol);
		break;
	case OPLUS_IC_FUNC_SET_OTG_BOOST_CURR_LIMIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_OTG_BOOST_CURR_LIMIT, oplus_chg_vb_set_otg_boost_curr_limit);
		break;
	case OPLUS_IC_FUNC_BUCK_AICL_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_AICL_ENABLE, oplus_chg_vb_aicl_enable);
		break;
	case OPLUS_IC_FUNC_BUCK_AICL_RERUN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_AICL_RERUN, oplus_chg_vb_aicl_rerun);
		break;
	case OPLUS_IC_FUNC_BUCK_AICL_RESET:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_AICL_RESET, oplus_chg_vb_aicl_reset);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_CC_ORIENTATION:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_CC_ORIENTATION, oplus_chg_vb_get_cc_orientation);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_HW_DETECT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_HW_DETECT, oplus_chg_vb_get_hw_detect);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_CHARGER_TYPE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_CHARGER_TYPE, oplus_chg_vb_get_charger_type);
		break;
	case OPLUS_IC_FUNC_BUCK_RERUN_BC12:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_RERUN_BC12, oplus_chg_vb_rerun_bc12);
		break;
	case OPLUS_IC_FUNC_BUCK_QC_DETECT_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_QC_DETECT_ENABLE, oplus_chg_vb_qc_detect_enable);
		break;
	case OPLUS_IC_FUNC_BUCK_SHIPMODE_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SHIPMODE_ENABLE, oplus_chg_vb_shipmod_enable);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_QC_CONFIG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_QC_CONFIG, oplus_chg_vb_set_qc_config);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_PD_CONFIG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_PD_CONFIG, oplus_chg_vb_set_pd_config);
		break;
	case OPLUS_IC_FUNC_WLS_BOOST_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_WLS_BOOST_ENABLE, oplus_chg_vb_wls_boost_enable);
		break;
	case OPLUS_IC_FUNC_SET_WLS_BOOST_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_WLS_BOOST_VOL, oplus_chg_vb_set_wls_boost_vol);
		break;
	case OPLUS_IC_FUNC_SET_WLS_BOOST_CURR_LIMIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_WLS_BOOST_CURR_LIMIT, oplus_chg_vb_set_wls_boost_curr_limit);
		break;
	case OPLUS_IC_FUNC_GAUGE_UPDATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_UPDATE, oplus_chg_vb_gauge_update);
		break;
	case OPLUS_IC_FUNC_VOOCPHY_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_VOOCPHY_ENABLE, oplus_chg_vb_voocphy_enable);
		break;
	case OPLUS_IC_FUNC_VOOCPHY_RESET_AGAIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_VOOCPHY_RESET_AGAIN, oplus_chg_vb_voocphy_reset_again);
		break;
	case OPLUS_IC_FUNC_GET_CHARGER_CYCLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_CHARGER_CYCLE, oplus_chg_vb_get_charger_cycle);
		break;
	case OPLUS_IC_FUNC_GET_SHUTDOWN_SOC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_SHUTDOWN_SOC, oplus_chg_vb_get_shutdown_soc);
		break;
	case OPLUS_IC_FUNC_BACKUP_SOC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BACKUP_SOC, oplus_chg_vb_backup_soc);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_VBUS_COLLAPSE_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_VBUS_COLLAPSE_STATUS, oplus_chg_vb_get_vbus_collapse_status);
		break;
	case OPLUS_IC_FUNC_GET_USB_TEMP_VOLT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_USB_TEMP_VOLT, oplus_chg_vb_get_usb_temp_volt);
		break;
	case OPLUS_IC_FUNC_USB_TEMP_CHECK_IS_SUPPORT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_USB_TEMP_CHECK_IS_SUPPORT, oplus_chg_vb_usb_temp_check_is_support);
		break;
	case OPLUS_IC_FUNC_GET_TYPEC_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_TYPEC_MODE, oplus_chg_vb_get_typec_mode);
		break;
	case OPLUS_IC_FUNC_SET_TYPEC_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_TYPEC_MODE, oplus_chg_vb_set_typec_mode);
		break;
	case OPLUS_IC_FUNC_SET_USB_DISCHG_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_USB_DISCHG_ENABLE, oplus_chg_vb_set_usb_dischg_enable);
		break;
	case OPLUS_IC_FUNC_GET_USB_DISCHG_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_USB_DISCHG_STATUS, oplus_chg_vb_get_usb_dischg_status);
		break;
	case OPLUS_IC_FUNC_SET_OTG_SWITCH_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_OTG_SWITCH_STATUS, oplus_chg_vb_set_otg_switch_status);
		break;
	case OPLUS_IC_FUNC_GET_OTG_SWITCH_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_OTG_SWITCH_STATUS, oplus_chg_vb_get_otg_switch_status);
		break;
	case OPLUS_IC_FUNC_GET_OTG_ONLINE_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_OTG_ONLINE_STATUS, oplus_chg_vb_get_otg_online_status);
		break;
	case OPLUS_IC_FUNC_CC_DETECT_HAPPENED:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CC_DETECT_HAPPENED, oplus_chg_vb_cc_detect_happened);
		break;
	case OPLUS_IC_FUNC_BUCK_CURR_DROP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_CURR_DROP, oplus_chg_vb_curr_drop);
		break;
	case OPLUS_IC_FUNC_BUCK_WDT_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_WDT_ENABLE, oplus_chg_vb_wdt_enable);
		break;
	case OPLUS_IC_FUNC_BUCK_KICK_WDT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_KICK_WDT, oplus_chg_vb_kick_wdt);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_AICL_POINT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_AICL_POINT, oplus_chg_vb_set_aicl_point);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_VINDPM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_VINDPM, oplus_chg_vb_set_vindpm);
		break;
	case OPLUS_IC_FUNC_VOOCPHY_SET_CURR_LEVEL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_VOOCPHY_SET_CURR_LEVEL, oplus_chg_vb_set_curr_level);
		break;
	case OPLUS_IC_FUNC_VOOCPHY_SET_MATCH_TEMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_VOOCPHY_SET_MATCH_TEMP, oplus_chg_vb_set_match_temp);
		break;
	case OPLUS_IC_FUNC_GET_OTG_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_OTG_ENABLE, oplus_chg_vb_get_otg_enable);
		break;
	case OPLUS_IC_FUNC_GET_SOURCE_PDO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_SOURCE_PDO, oplus_chg_vb_get_source_pdo);
		break;
	case OPLUS_IC_FUNC_GET_CHARGER_VOL_MAX:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_CHARGER_VOL_MAX, oplus_chg_vb_get_charger_vol_max);
		break;
	case OPLUS_IC_FUNC_GET_CHARGER_VOL_MIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_CHARGER_VOL_MIN, oplus_chg_vb_get_charger_vol_min);
		break;
	case OPLUS_IC_FUNC_GET_CHARGER_CURR_MAX:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_CHARGER_CURR_MAX, oplus_chg_vb_get_charger_curr_max);
		break;
	case OPLUS_IC_FUNC_BUCK_BC12_COMPLETED:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_BC12_COMPLETED, oplus_chg_vb_bc12_completed);
		break;
	case OPLUS_IC_FUNC_DISABLE_VBUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_DISABLE_VBUS, oplus_chg_vb_disable_vbus);
		break;
	case OPLUS_IC_FUNC_IS_OPLUS_SVID:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_IS_OPLUS_SVID,
					       oplus_chg_vb_is_oplus_svid);
		break;
	case OPLUS_IC_FUNC_BUCK_HARDWARE_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_HARDWARE_INIT, oplus_chg_vb_hardware_init);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_TYPEC_STATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_TYPEC_STATE, oplus_chg_vb_get_typec_state);
		break;
	case OPLUS_IC_FUNC_GET_DATA_ROLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_DATA_ROLE,
					       oplus_chg_vb_get_data_role);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_USB_BTB_TEMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_USB_BTB_TEMP,
					       oplus_chg_vb_get_usb_btb_temp);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_BATT_BTB_TEMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_BATT_BTB_TEMP,
					       oplus_chg_vb_get_batt_btb_temp);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_VBAT_PWR:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_VBAT_PWR,
					       oplus_chg_vb_get_vbat_pwr);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_FV:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_FV, oplus_chg_vb_get_fv);
		break;

	case OPLUS_IC_FUNC_BUCK_WLS_INPUT_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_WLS_INPUT_SUSPEND, oplus_chg_vb_wls_input_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_WLS_ICL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_WLS_ICL, oplus_chg_vb_set_wls_icl);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_WLS_ICL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_WLS_ICL, oplus_chg_vb_get_wls_icl);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_WLS_INPUT_CURR:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_WLS_INPUT_CURR, oplus_chg_vb_get_wls_input_curr);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_WLS_INPUT_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_WLS_INPUT_VOL, oplus_chg_vb_get_wls_input_vol);
		break;
	case OPLUS_IC_FUNC_BUCK_WLS_AICL_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_WLS_AICL_ENABLE, oplus_chg_vb_wls_aicl_enable);
		break;
	case OPLUS_IC_FUNC_BUCK_WLS_AICL_RERUN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_WLS_AICL_RERUN, oplus_chg_vb_wls_aicl_rerun);
		break;
	case OPLUS_IC_FUNC_GET_TYPEC_ROLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_TYPEC_ROLE, oplus_chg_vb_get_typec_role);
		break;
	case OPLUS_IC_FUNC_BUCK_DIS_INSERT_DETECT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_DIS_INSERT_DETECT, oplus_chg_vb_set_usb_drv);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_BYBID_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_BYBID_INFO, oplus_chg_vb_get_byb_id_info);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_BYBID_MATCH_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_BYBID_MATCH_INFO, oplus_chg_vb_get_byb_id_match_info);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_BYB_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_BYB_STATUS, oplus_chg_vb_get_byb_status_info);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_BYB_VOUT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_BYB_VOUT, oplus_chg_vb_get_byb_vout_info);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_BYB_VOUT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_BYB_VOUT, oplus_chg_vb_set_byb_vout_info);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_USB_AICL_ENHANCE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_USB_AICL_ENHANCE, oplus_chg_vb_get_usb_aicl_enhance);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_LPD_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_LPD_INFO, oplus_chg_vb_get_lpd_info);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_FLASH_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_FLASH_MODE, oplus_chg_vb_set_flash_mode);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_CHG_PATH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_CHG_PATH, oplus_chg_vb_set_chg_path);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_CHG_PATH_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_CHG_PATH_STATUS, oplus_chg_vb_get_chg_path_status);
		break;
	case OPLUS_IC_FUNC_BUCK_ITEM_CHECK:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_ITEM_CHECK, oplus_chg_vb_iterm_check);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_SHAFT_BATT_BTB_TEMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_SHAFT_BATT_BTB_TEMP,
					       oplus_chg_vb_get_shaft_btb_temp);
		break;
	case OPLUS_IC_FUNC_BUCK_PUSH_SHAFT_BATT_BTB_OVER:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_PUSH_SHAFT_BATT_BTB_OVER,
					       oplus_chg_vb_set_shfat_btb_over_status);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_POWER_ROLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_POWER_ROLE, oplus_chg_vb_get_power_role);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_POWER_MOS_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_POWER_MOS_ENABLE, oplus_chg_vb_set_power_mos);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_POWER_MOS_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_POWER_MOS_ENABLE, oplus_chg_vb_get_power_mos_status);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_OVP_FORCED:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_OVP_FORCED, oplus_chg_vb_set_adsp_ovp);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_DPDM_OVP_DISABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_DPDM_OVP_DISABLE, oplus_set_usb_dpdm_ovp_disable);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_VDM_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_VDM_INFO, oplus_chg_vb_get_vdm_info);
		break;
	case OPLUS_IC_FUNC_BUCK_SEND_GET_SINK_CAP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SEND_GET_SINK_CAP, oplus_chg_vb_send_get_sink_cap);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

static void oplus_vb_err_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_buck_ic *chip = virq_data;

	oplus_chg_ic_move_err_msg(chip->ic_dev, ic_dev);
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_ERR);
}

static void oplus_vb_cc_detect_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_buck_ic *chip = virq_data;

	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_CC_DETECT);
}

static void oplus_vb_plugin_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_buck_ic *chip = virq_data;

	if (virq_data != NULL)
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_PLUGIN);
	else
		chg_info("virtual buck plugin virq_data null\n");
}

static void oplus_vb_power_role_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_buck_ic *chip = virq_data;

	if (virq_data != NULL)
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_POWER_ROLE_STATUS);
	else
		chg_info("virtual buck power role virq_data null\n");
}

static void oplus_vb_cc_changed_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_buck_ic *chip = virq_data;

	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_CC_CHANGED);
}

static void oplus_vb_suspend_check_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_buck_ic *chip = virq_data;
	int i;

	for (i = 0; i < chip->child_num; i++) {
		if (chip->child_list[i].ic_dev != ic_dev)
			continue;
		if (!func_is_support(&chip->child_list[i], OPLUS_IC_FUNC_BUCK_SET_ICL) ||
		    chip->child_list[i].current_ratio == 0) {
			oplus_chg_ic_func(chip->child_list[i].ic_dev, OPLUS_IC_FUNC_BUCK_INPUT_SUSPEND, true);
			/* oplus_chg_unsuspend_plat_pmic(NULL); TODO */
		}
	}
}

static void oplus_vb_chg_type_change_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_buck_ic *chip = virq_data;

	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);
}

static void oplus_vb_svid_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_buck_ic *chip = virq_data;

	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_SVID);
}

static void oplus_vb_otg_enable_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_buck_ic *chip = virq_data;

	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_OTG_ENABLE);
}

static void oplus_vb_power_changed_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_buck_ic *chip = virq_data;

	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_POWER_CHANGED);
}


static void oplus_vb_voltage_change_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_buck_ic *chip = virq_data;

	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_VOLTAGE_CHANGED);
}

static void oplus_vb_current_change_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_buck_ic *chip = virq_data;

	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_CURRENT_CHANGED);
}

static void oplus_vb_bc12_completed_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_buck_ic *chip = virq_data;

	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_BC12_COMPLETED);
}

static void oplus_vb_data_role_changed_handler(struct oplus_chg_ic_dev *ic_dev,
					       void *virq_data)
{
	struct oplus_virtual_buck_ic *chip = virq_data;

	oplus_chg_ic_virq_trigger(chip->ic_dev,
				  OPLUS_IC_VIRQ_DATA_ROLE_CHANGED);
}

static void oplus_vb_pd_completed_notify_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_buck_ic *chip = virq_data;

	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_PD_COMPLETED);
}

static void oplus_vb_typec_state_notify_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_buck_ic *chip = virq_data;

	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_TYPEC_STATE);
}

struct oplus_chg_ic_virq oplus_vb_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_CC_DETECT },
	{ .virq_id = OPLUS_IC_VIRQ_PLUGIN },
	{ .virq_id = OPLUS_IC_VIRQ_CC_CHANGED },
	{ .virq_id = OPLUS_IC_VIRQ_SUSPEND_CHECK },
	{ .virq_id = OPLUS_IC_VIRQ_CHG_TYPE_CHANGE },
	{ .virq_id = OPLUS_IC_VIRQ_SVID },
	{ .virq_id = OPLUS_IC_VIRQ_OTG_ENABLE },
	{ .virq_id = OPLUS_IC_VIRQ_VOLTAGE_CHANGED },
	{ .virq_id = OPLUS_IC_VIRQ_CURRENT_CHANGED },
	{ .virq_id = OPLUS_IC_VIRQ_BC12_COMPLETED },
	{ .virq_id = OPLUS_IC_VIRQ_DATA_ROLE_CHANGED },
	{ .virq_id = OPLUS_IC_VIRQ_TYPEC_STATE},
	{ .virq_id = OPLUS_IC_VIRQ_PD_COMPLETED},
	{.virq_id = OPLUS_IC_VIRQ_POWER_ROLE_STATUS},
	{.virq_id = OPLUS_IC_VIRQ_POWER_CHANGED},
};

static int oplus_vb_virq_register(struct oplus_virtual_buck_ic *chip)
{
	int i, rc;

	for (i = 0; i < chip->child_num; i++) {
		if (virq_is_support(&chip->child_list[i], OPLUS_IC_VIRQ_ERR)) {
			rc = oplus_chg_ic_virq_register(chip->child_list[i].ic_dev,
				OPLUS_IC_VIRQ_ERR, oplus_vb_err_handler, chip);
			if (rc < 0)
				chg_err("register OPLUS_IC_VIRQ_ERR error, rc=%d", rc);
		}
		if (virq_is_support(&chip->child_list[i], OPLUS_IC_VIRQ_CC_DETECT)) {
			rc = oplus_chg_ic_virq_register(chip->child_list[i].ic_dev,
				OPLUS_IC_VIRQ_CC_DETECT, oplus_vb_cc_detect_handler, chip);
			if (rc < 0)
				chg_err("register OPLUS_IC_VIRQ_CC_DETECT error, rc=%d", rc);
		}
		if (virq_is_support(&chip->child_list[i], OPLUS_IC_VIRQ_PLUGIN)) {
			rc = oplus_chg_ic_virq_register(chip->child_list[i].ic_dev,
				OPLUS_IC_VIRQ_PLUGIN, oplus_vb_plugin_handler, chip);
			if (rc < 0)
				chg_err("register OPLUS_IC_VIRQ_PLUGIN error, rc=%d", rc);
		}
		if (virq_is_support(&chip->child_list[i], OPLUS_IC_VIRQ_POWER_ROLE_STATUS)) {
			rc = oplus_chg_ic_virq_register(chip->child_list[i].ic_dev,
				OPLUS_IC_VIRQ_POWER_ROLE_STATUS, oplus_vb_power_role_handler, chip);
			if (rc < 0)
				chg_err("register OPLUS_IC_VIRQ_POWER_ROLE_STATUS error, rc=%d", rc);
		}
		if (virq_is_support(&chip->child_list[i], OPLUS_IC_VIRQ_CC_CHANGED)) {
			rc = oplus_chg_ic_virq_register(chip->child_list[i].ic_dev,
				OPLUS_IC_VIRQ_CC_CHANGED, oplus_vb_cc_changed_handler, chip);
			if (rc < 0)
				chg_err("register OPLUS_IC_VIRQ_CC_CHANGED error, rc=%d", rc);
		}
		if (virq_is_support(&chip->child_list[i], OPLUS_IC_VIRQ_SUSPEND_CHECK)) {
			rc = oplus_chg_ic_virq_register(chip->child_list[i].ic_dev,
				OPLUS_IC_VIRQ_SUSPEND_CHECK, oplus_vb_suspend_check_handler, chip);
			if (rc < 0)
				chg_err("register OPLUS_IC_VIRQ_SUSPEND_CHECK error, rc=%d", rc);
		}
		if (virq_is_support(&chip->child_list[i], OPLUS_IC_VIRQ_CHG_TYPE_CHANGE)) {
			rc = oplus_chg_ic_virq_register(chip->child_list[i].ic_dev,
				OPLUS_IC_VIRQ_CHG_TYPE_CHANGE, oplus_vb_chg_type_change_handler, chip);
			if (rc < 0)
				chg_err("register OPLUS_IC_VIRQ_CHG_TYPE_CHANGE error, rc=%d", rc);
		}
		if (virq_is_support(&chip->child_list[i], OPLUS_IC_VIRQ_SVID)) {
			rc = oplus_chg_ic_virq_register(chip->child_list[i].ic_dev,
				OPLUS_IC_VIRQ_SVID, oplus_vb_svid_handler, chip);
			if (rc < 0)
				chg_err("register OPLUS_IC_VIRQ_SVID error, rc=%d", rc);
		}
		if (virq_is_support(&chip->child_list[i], OPLUS_IC_VIRQ_OTG_ENABLE)) {
			rc = oplus_chg_ic_virq_register(chip->child_list[i].ic_dev,
				OPLUS_IC_VIRQ_OTG_ENABLE, oplus_vb_otg_enable_handler, chip);
			if (rc < 0)
				chg_err("register OPLUS_IC_VIRQ_OTG_ENABLE error, rc=%d", rc);
		}
		if (virq_is_support(&chip->child_list[i], OPLUS_IC_VIRQ_POWER_CHANGED)) {
			rc = oplus_chg_ic_virq_register(chip->child_list[i].ic_dev,
				OPLUS_IC_VIRQ_POWER_CHANGED, oplus_vb_power_changed_handler, chip);
			if (rc < 0)
				chg_err("register OPLUS_IC_VIRQ_POWER_CHANGED error, rc=%d", rc);
		}
		if (virq_is_support(&chip->child_list[i], OPLUS_IC_VIRQ_VOLTAGE_CHANGED)) {
			rc = oplus_chg_ic_virq_register(chip->child_list[i].ic_dev,
				OPLUS_IC_VIRQ_VOLTAGE_CHANGED, oplus_vb_voltage_change_handler, chip);
			if (rc < 0)
				chg_err("register OPLUS_IC_VIRQ_VOLTAGE_CHANGED error, rc=%d", rc);
		}
		if (virq_is_support(&chip->child_list[i], OPLUS_IC_VIRQ_CURRENT_CHANGED)) {
			rc = oplus_chg_ic_virq_register(chip->child_list[i].ic_dev,
				OPLUS_IC_VIRQ_CURRENT_CHANGED, oplus_vb_current_change_handler, chip);
			if (rc < 0)
				chg_err("register OPLUS_IC_VIRQ_CURRENT_CHANGED error, rc=%d", rc);
		}
		if (virq_is_support(&chip->child_list[i], OPLUS_IC_VIRQ_BC12_COMPLETED)) {
			rc = oplus_chg_ic_virq_register(chip->child_list[i].ic_dev,
				OPLUS_IC_VIRQ_BC12_COMPLETED, oplus_vb_bc12_completed_handler, chip);
			if (rc < 0)
				chg_err("register OPLUS_IC_VIRQ_BC12_COMPLETED error, rc=%d", rc);
		}
		if (virq_is_support(&chip->child_list[i],
				    OPLUS_IC_VIRQ_DATA_ROLE_CHANGED)) {
			rc = oplus_chg_ic_virq_register(
				chip->child_list[i].ic_dev,
				OPLUS_IC_VIRQ_DATA_ROLE_CHANGED,
				oplus_vb_data_role_changed_handler, chip);
			if (rc < 0)
				chg_err("register OPLUS_IC_VIRQ_DATA_ROLE_CHANGED error, rc=%d",
					rc);
		}
		if (virq_is_support(&chip->child_list[i], OPLUS_IC_VIRQ_TYPEC_STATE)) {
			rc = oplus_chg_ic_virq_register(chip->child_list[i].ic_dev,
				OPLUS_IC_VIRQ_TYPEC_STATE, oplus_vb_typec_state_notify_handler, chip);
			if (rc < 0)
				chg_err("register OPLUS_IC_VIRQ_TYPEC_STATE error, rc=%d", rc);
		}
		if (virq_is_support(&chip->child_list[i], OPLUS_IC_VIRQ_PD_COMPLETED)) {
			rc = oplus_chg_ic_virq_register(chip->child_list[i].ic_dev,
				OPLUS_IC_VIRQ_PD_COMPLETED, oplus_vb_pd_completed_notify_handler, chip);
			if (rc < 0)
				chg_err("register OPLUS_IC_VIRQ_PD_COMPLETED error, rc=%d", rc);
		}
	}
	chg_info("vb_virq register success\n");

	return 0;
}

static void oplus_buck_fpga_rst_work(struct work_struct *work)
{
	struct oplus_virtual_buck_ic *chip = container_of(
		work, struct oplus_virtual_buck_ic, buck_fpga_rst_work);

	oplus_chg_vb_fpga_rst(chip->ic_dev, chip->fpga_reset_status);
}

static void oplus_buck_i2c_rst_work(struct work_struct *work)
{
	struct oplus_virtual_buck_ic *chip = container_of(
		work, struct oplus_virtual_buck_ic, buck_i2c_rst_work);

	oplus_chg_vb_fpga_rst(chip->ic_dev, chip->i2c_reset_status);
}

static void oplus_virtual_buck_error_subs_callback(struct mms_subscribe *subs,
					   enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_virtual_buck_ic *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case ERR_ITEM_FPGA_RESET:
			oplus_mms_get_item_data(chip->error_topic, ERR_ITEM_FPGA_RESET, &data,
						false);
			chip->fpga_reset_status = data.intval;
			chg_err("ERR_ITEM_FPGA_RESET, reset_status=%d\n", chip->fpga_reset_status);
			schedule_work(&chip->buck_fpga_rst_work);
			break;
		case ERR_ITEM_I2C_RESET:
			oplus_mms_get_item_data(chip->error_topic, ERR_ITEM_I2C_RESET, &data,
						false);
			chip->i2c_reset_status = data.intval;
			chg_err("ERR_ITEM_I2C_RESET, reset_status=%d\n", chip->i2c_reset_status);
			schedule_work(&chip->buck_i2c_rst_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_virtual_buck_subscribe_error_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_virtual_buck_ic *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->error_topic = topic;
	chip->error_subs =
		oplus_mms_subscribe(chip->error_topic, chip,
				    oplus_virtual_buck_error_subs_callback, "%s", chip->ic_dev->manu_name);
	if (IS_ERR_OR_NULL(chip->error_subs)) {
		chg_err("subscribe error topic error, rc=%ld\n",
			PTR_ERR(chip->error_subs));
		return;
	}

	oplus_mms_get_item_data(chip->error_topic, ERR_ITEM_FPGA_RESET, &data, true);
	chip->fpga_reset_status = data.intval;
	if (chip->fpga_reset_status == FPGA_RESET_START || chip->fpga_reset_status == FPGA_RESET_END)
		schedule_work(&chip->buck_fpga_rst_work);

	oplus_mms_get_item_data(chip->error_topic, ERR_ITEM_I2C_RESET, &data, true);
	chip->i2c_reset_status = data.intval;
	if (chip->i2c_reset_status == GUAGE_I2C_RST_START || chip->i2c_reset_status == GUAGE_I2C_RST_END)
		schedule_work(&chip->buck_i2c_rst_work);
}

static int oplus_virtual_buck_probe(struct platform_device *pdev)
{
	struct oplus_virtual_buck_ic *chip;
	struct device_node *node = pdev->dev.of_node;
	struct oplus_chg_ic_cfg ic_cfg = { 0 };
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	int rc = 0;

	chip = devm_kzalloc(&pdev->dev, sizeof(struct oplus_virtual_buck_ic),
			    GFP_KERNEL);
	if (chip == NULL) {
		chg_err("alloc memory error\n");
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);
	INIT_WORK(&chip->buck_fpga_rst_work, oplus_buck_fpga_rst_work);
	INIT_WORK(&chip->buck_i2c_rst_work, oplus_buck_i2c_rst_work);
	INIT_DELAYED_WORK(&chip->upload_device_id_work, oplus_vc_upload_device_id_work);

	chip->misc_gpio.is_dischg_gpio_request = false;
	rc = oplus_vc_usbtemp_adc_init(chip);
	if (rc < 0) {
		chg_err("usbtemp adc init error, rc=%d\n", rc);
	} else {
		rc = oplus_vc_usbtemp_iio_init(chip);
		if (rc < 0) {
			chg_err("usbtemp iio init error, rc=%d\n", rc);
			if (rc == -EPROBE_DEFER)
				goto iio_init_err;
		}
	}
	rc = oplus_vc_misc_init(chip);
	if (rc < 0) {
		chg_err("misc init error, rc=%d\n", rc);
		goto misic_init_err;
	}

	rc = of_property_read_u32(node, "oplus,ic_type", &ic_type);
	if (rc < 0) {
		chg_err("can't get ic type, rc=%d\n", rc);
		goto reg_ic_err;
	}
	rc = of_property_read_u32(node, "oplus,ic_index", &ic_index);
	if (rc < 0) {
		chg_err("can't get ic index, rc=%d\n", rc);
		goto reg_ic_err;
	}
	if (of_find_property(node, "nvmem-cells", NULL)) {
		chip->soc_backup_nvmem = devm_nvmem_cell_get(chip->dev,
						"oplus_soc_backup");
		if (IS_ERR(chip->soc_backup_nvmem)) {
			rc = PTR_ERR(chip->soc_backup_nvmem);
			if (rc != -EPROBE_DEFER)
				chg_err("Failed to get nvmem-cells, rc=%d\n", rc);
		}
	} else {
		chip->soc_backup_nvmem = NULL;
		chg_err("soc_backup_nvmem get fail\n");
	}

	ic_cfg.name = node->name;
	ic_cfg.index = ic_index;
	snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "buck-virtual");
	snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
	ic_cfg.type = ic_type;
	ic_cfg.get_func = oplus_chg_vb_get_func;
	ic_cfg.virq_data = oplus_vb_virq_table;
	ic_cfg.virq_num = ARRAY_SIZE(oplus_vb_virq_table);
	ic_cfg.of_node = node;
	chip->ic_dev = devm_oplus_chg_ic_register(chip->dev, &ic_cfg);
	if (!chip->ic_dev) {
		rc = -ENODEV;
		chg_err("register %s error\n", node->name);
		goto reg_ic_err;
	}

#if IS_ENABLED(CONFIG_HORAE_FLASH_LED_THERMAL)
	oplus_ntc_switch_init(chip);
#endif

#if IS_ENABLED(CONFIG_OPLUS_CHG_TEST_KIT)
	oplus_virtual_buck_test_kit_init(chip);
#endif
	oplus_mms_wait_topic("error", oplus_virtual_buck_subscribe_error_topic, chip);

	chg_err("probe success\n");
	return 0;

reg_ic_err:
	if (oplus_vc_ccdetect_gpio_support(chip))
		gpio_free(chip->misc_gpio.ccdetect_gpio);
	if (gpio_is_valid(chip->misc_gpio.vchg_trig_gpio))
		gpio_free(chip->misc_gpio.vchg_trig_gpio);
	if (gpio_is_valid(chip->misc_gpio.ship_gpio))
		gpio_free(chip->misc_gpio.ship_gpio);
misic_init_err:
iio_init_err:
	if (gpio_is_valid(chip->misc_gpio.dischg_gpio) && chip->misc_gpio.is_dischg_gpio_request)
		gpio_free(chip->misc_gpio.dischg_gpio);

	chip->misc_gpio.is_dischg_gpio_request = false;
	devm_kfree(&pdev->dev, chip);
	platform_set_drvdata(pdev, NULL);

	chg_err("probe error\n");
	return rc;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void oplus_virtual_buck_remove(struct platform_device *pdev)
#else
static int oplus_virtual_buck_remove(struct platform_device *pdev)
#endif
{
	struct oplus_virtual_buck_ic *chip = platform_get_drvdata(pdev);
#if IS_ENABLED(CONFIG_HORAE_FLASH_LED_THERMAL)
	struct thermal_zone_device *tzd;
#endif

	if (chip == NULL) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
		return -ENODEV;
#else
		return;
#endif
	}

#if IS_ENABLED(CONFIG_HORAE_FLASH_LED_THERMAL)
	if (chip->ntc_switch_gpio.ntc_switch_subboard) {
		tzd = thermal_zone_get_zone_by_name("subboard_temp");
		if (!IS_ERR_OR_NULL(tzd))
			thermal_zone_device_unregister(tzd);
	}
#endif

	if (chip->ic_dev->online)
		oplus_chg_vb_exit(chip->ic_dev);
#if IS_ENABLED(CONFIG_OPLUS_CHG_TEST_KIT)
	oplus_virtual_buck_test_kit_exit(chip);
#endif
	devm_oplus_chg_ic_unregister(&pdev->dev, chip->ic_dev);
	if (oplus_vc_ccdetect_gpio_support(chip))
		gpio_free(chip->misc_gpio.ccdetect_gpio);
	if (gpio_is_valid(chip->misc_gpio.vchg_trig_gpio))
		gpio_free(chip->misc_gpio.vchg_trig_gpio);
	if (gpio_is_valid(chip->misc_gpio.ship_gpio))
		gpio_free(chip->misc_gpio.ship_gpio);
	if (gpio_is_valid(chip->misc_gpio.dischg_gpio) && chip->misc_gpio.is_dischg_gpio_request)
		gpio_free(chip->misc_gpio.dischg_gpio);
	chip->misc_gpio.is_dischg_gpio_request = false;
	devm_kfree(&pdev->dev, chip->child_list);
	devm_kfree(&pdev->dev, chip);
	platform_set_drvdata(pdev, NULL);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return 0;
#endif
}

static const struct of_device_id oplus_virtual_buck_match[] = {
	{ .compatible = "oplus,virtual_buck" },
	{},
};

static struct platform_driver oplus_virtual_buck_driver = {
	.driver		= {
		.name = "oplus-virtual_buck",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(oplus_virtual_buck_match),
	},
	.probe		= oplus_virtual_buck_probe,
	.remove		= oplus_virtual_buck_remove,
};

static __init int oplus_virtual_buck_init(void)
{
	return platform_driver_register(&oplus_virtual_buck_driver);
}

static __exit void oplus_virtual_buck_exit(void)
{
	platform_driver_unregister(&oplus_virtual_buck_driver);
}

oplus_chg_module_register(oplus_virtual_buck);
