// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023-2023 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[VIRTUAL_UFCS]([%s][%d]): " fmt, __func__, __LINE__

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
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/system/boot_mode.h>
#include <soc/oplus/device_info.h>
#include <soc/oplus/system/oplus_project.h>
#endif
#include <oplus_chg_module.h>
#include <oplus_chg_ic.h>
#include <ufcs_class.h>
#include <oplus_chg_monitor.h>

struct oplus_virtual_ufcs_ic {
	struct device *dev;
	struct oplus_chg_ic_dev *ic_dev;
	struct ufcs_dev *ufcs;
	bool online;
	struct delayed_work upload_device_id_work;
};

static int oplus_chg_ufcs_init(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	ic_dev->online = true;

	return 0;
}

static int oplus_chg_ufcs_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (!ic_dev->online)
		return 0;

	ic_dev->online = false;

	chg_info("unregister success\n");

	return 0;
}

static int oplus_chg_ufcs_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	return 0;
}

static int oplus_chg_ufcs_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	return 0;
}

static int oplus_chg_ufcs_handshake(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_ufcs_ic *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	rc = ufcs_handshake(chip->ufcs);

	return rc;
}

static int oplus_chg_ufcs_pdo_set(struct oplus_chg_ic_dev *ic_dev, int vol_mv, int curr_ma)
{
	struct oplus_virtual_ufcs_ic *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	rc = ufcs_intf_pdo_set(chip->ufcs, vol_mv, curr_ma);

	return rc;
}

static int oplus_chg_ufcs_hard_reset(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_ufcs_ic *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	rc = ufcs_intf_source_hard_reset(chip->ufcs);

	return rc;
}

static int oplus_chg_ufcs_exit_ufcs_mode(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_ufcs_ic *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	rc = ufcs_force_exit(chip->ufcs);

	return rc;
}

static int oplus_chg_ufcs_config_wd(struct oplus_chg_ic_dev *ic_dev, u16 time_ms)
{
	struct oplus_virtual_ufcs_ic *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	rc = ufcs_intf_config_watchdog(chip->ufcs, time_ms);

	return rc;
}

static int oplus_chg_ufcs_get_dev_info(struct oplus_chg_ic_dev *ic_dev, u64 *dev_info)
{
	struct oplus_virtual_ufcs_ic *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (dev_info == NULL) {
		chg_err("dev_info buf is NULL");
		return -EINVAL;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	rc = ufcs_intf_get_device_info(chip->ufcs, dev_info);

	return rc;
}

static int oplus_chg_ufcs_get_err_info(struct oplus_chg_ic_dev *ic_dev, u64 *err_info)
{
	struct oplus_virtual_ufcs_ic *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (err_info == NULL) {
		chg_err("err_info buf is NULL");
		return -EINVAL;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	rc = ufcs_intf_get_error_info(chip->ufcs, err_info);

	return rc;
}

static int oplus_chg_ufcs_get_src_info(struct oplus_chg_ic_dev *ic_dev, u64 *src_info)
{
	struct oplus_virtual_ufcs_ic *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (src_info == NULL) {
		chg_err("src_info buf is NULL");
		return -EINVAL;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	rc = ufcs_intf_get_source_info(chip->ufcs, src_info);

	return rc;
}

static int oplus_chg_ufcs_get_cable_info(struct oplus_chg_ic_dev *ic_dev, u64 *cable_info)
{
	struct oplus_virtual_ufcs_ic *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (cable_info == NULL) {
		chg_err("cable_info buf is NULL");
		return -EINVAL;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	rc = ufcs_intf_get_cable_info(chip->ufcs, cable_info);

	return rc;
}

static int oplus_chg_ufcs_get_pdo_info(struct oplus_chg_ic_dev *ic_dev, u64 *pdo, int num)
{
	struct oplus_virtual_ufcs_ic *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (pdo == NULL) {
		chg_err("pdo buf is NULL");
		return -EINVAL;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	rc = ufcs_intf_get_pdo_info(chip->ufcs, pdo, num);

	return rc;
}

static int oplus_chg_ufcs_verify_adapter(struct oplus_chg_ic_dev *ic_dev,
	u8 key_index, u8 *auth_data, u8 data_len)
{
	struct oplus_virtual_ufcs_ic *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	rc = ufcs_intf_verify_adapter(chip->ufcs, key_index, auth_data, data_len);

	return rc;
}

static int oplus_chg_ufcs_get_power_change_info(
	struct oplus_chg_ic_dev *ic_dev, u32 *pwr_change_info, int num)
{
	struct oplus_virtual_ufcs_ic *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (pwr_change_info == NULL) {
		chg_err("pwr_change_info is NULL");
		return -EINVAL;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	rc = ufcs_intf_get_power_change_info(chip->ufcs, pwr_change_info, num);

	return rc;
}

static int oplus_chg_ufcs_get_emark_info(struct oplus_chg_ic_dev *ic_dev, u64 *info)
{
	struct oplus_virtual_ufcs_ic *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (info == NULL) {
		chg_err("info is NULL");
		return -EINVAL;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	rc = ufcs_intf_get_emark_info(chip->ufcs, info);

	return rc;
}

static int oplus_chg_ufcs_is_test_mode(struct oplus_chg_ic_dev *ic_dev, bool *en)
{
	struct oplus_virtual_ufcs_ic *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (en == NULL) {
		chg_err("en is NULL");
		return -EINVAL;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*en = ufcs_is_test_mode(chip->ufcs);

	return 0;
}

static int oplus_chg_ufcs_is_vol_acc_test_mode(struct oplus_chg_ic_dev *ic_dev, bool *en)
{
	struct oplus_virtual_ufcs_ic *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (en == NULL) {
		chg_err("en is NULL");
		return -EINVAL;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*en = ufcs_is_vol_acc_test_mode(chip->ufcs);

	return 0;
}

static int oplus_chg_ufcs_reset_dpdm(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_ufcs_ic *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	rc = ufcs_reset_dpdm(chip->ufcs);

	return rc;
}

static int oplus_chg_ufcs_get_power_info_ext(
	struct oplus_chg_ic_dev *ic_dev, u64 *info, int num)
{
	struct oplus_virtual_ufcs_ic *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (info == NULL) {
		chg_err("info is NULL");
		return -EINVAL;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	rc = ufcs_intf_get_power_info_ext(chip->ufcs, info, num);

	return rc;
}

static void *oplus_chg_ufcs_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}
	if (!oplus_chg_ic_func_is_support(ic_dev, func_id)) {
		chg_info("%s: this func(=%d) is not supported\n",  ic_dev->name, func_id);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT,
			oplus_chg_ufcs_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT,
			oplus_chg_ufcs_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP,
			oplus_chg_ufcs_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST,
			oplus_chg_ufcs_smt_test);
		break;
	case OPLUS_IC_FUNC_UFCS_HANDSHAKE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_HANDSHAKE,
			oplus_chg_ufcs_handshake);
		break;
	case OPLUS_IC_FUNC_UFCS_PDO_SET:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_PDO_SET,
			oplus_chg_ufcs_pdo_set);
		break;
	case OPLUS_IC_FUNC_UFCS_HARD_RESET:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_HARD_RESET,
			oplus_chg_ufcs_hard_reset);
		break;
	case OPLUS_IC_FUNC_UFCS_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_EXIT,
			oplus_chg_ufcs_exit_ufcs_mode);
		break;
	case OPLUS_IC_FUNC_UFCS_CONFIG_WD:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_CONFIG_WD,
			oplus_chg_ufcs_config_wd);
		break;
	case OPLUS_IC_FUNC_UFCS_GET_DEV_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_GET_DEV_INFO,
			oplus_chg_ufcs_get_dev_info);
		break;
	case OPLUS_IC_FUNC_UFCS_GET_ERR_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_GET_ERR_INFO,
			oplus_chg_ufcs_get_err_info);
		break;
	case OPLUS_IC_FUNC_UFCS_GET_SRC_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_GET_SRC_INFO,
			oplus_chg_ufcs_get_src_info);
		break;
	case OPLUS_IC_FUNC_UFCS_GET_CABLE_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_GET_CABLE_INFO,
			oplus_chg_ufcs_get_cable_info);
		break;
	case OPLUS_IC_FUNC_UFCS_GET_PDO_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_GET_PDO_INFO,
			oplus_chg_ufcs_get_pdo_info);
		break;
	case OPLUS_IC_FUNC_UFCS_VERIFY_ADAPTER:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_VERIFY_ADAPTER,
			oplus_chg_ufcs_verify_adapter);
		break;
	case OPLUS_IC_FUNC_UFCS_GET_POWER_CHANGE_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_GET_POWER_CHANGE_INFO,
			oplus_chg_ufcs_get_power_change_info);
		break;
	case OPLUS_IC_FUNC_UFCS_GET_EMARK_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_GET_EMARK_INFO,
			oplus_chg_ufcs_get_emark_info);
		break;
	case OPLUS_IC_FUNC_UFCS_GET_POWER_INFO_EXT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_GET_POWER_INFO_EXT,
			oplus_chg_ufcs_get_power_info_ext);
		break;
	case OPLUS_IC_FUNC_UFCS_IS_TEST_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_IS_TEST_MODE,
			oplus_chg_ufcs_is_test_mode);
		break;
	case OPLUS_IC_FUNC_UFCS_IS_VOL_ACC_TEST_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_IS_VOL_ACC_TEST_MODE,
			oplus_chg_ufcs_is_vol_acc_test_mode);
		break;
	case OPLUS_IC_FUNC_UFCS_RESET_DPDM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_UFCS_RESET_DPDM,
			oplus_chg_ufcs_reset_dpdm);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

static void oplus_vufcs_upload_device_id_subscribe_error_topic(
	struct oplus_mms *topic, void *prv_data)
{
	struct oplus_virtual_ufcs_ic *chip = prv_data;

	/* delay 50ms upload for error topic init stop publish finish */
	schedule_delayed_work(&chip->upload_device_id_work, msecs_to_jiffies(50));
}

static void oplus_vufcs_upload_device_id_work(struct work_struct *work)
{
	int rc;
	struct mms_msg *msg;
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_virtual_ufcs_ic *chip = container_of(dwork,
		struct oplus_virtual_ufcs_ic, upload_device_id_work);
	struct oplus_mms *err_topic = oplus_mms_get_by_name("error");

	if (!err_topic)
		return;

	if (!chip->ufcs || !chip->ufcs->dev.parent) {
		chg_err("dev not ready\n");
		return;
	}

	msg = oplus_mms_alloc_str_msg(
		MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_DEVICE_ID, "ufcs_ic=%s",
		chip->ufcs->dev.parent->of_node->name);
	if (msg == NULL) {
		chg_err("alloc device id msg error\n");
		return;
	}

	rc = oplus_mms_publish_msg_sync(err_topic, msg);
	if (rc < 0) {
		chg_err("publish device id msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_vufcs_upload_device_id(struct oplus_virtual_ufcs_ic *chip)
{
	oplus_mms_wait_topic("error",
		oplus_vufcs_upload_device_id_subscribe_error_topic, chip);
}

struct oplus_chg_ic_virq oplus_ufcs_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
};

static int oplus_virtual_ufcs_probe(struct platform_device *pdev)
{
	struct oplus_virtual_ufcs_ic *chip;
	struct device_node *node = pdev->dev.of_node;
	struct oplus_chg_ic_cfg ic_cfg = { 0 };
	int rc = 0;

	chip = devm_kzalloc(&pdev->dev, sizeof(struct oplus_virtual_ufcs_ic),
			    GFP_KERNEL);
	if (chip == NULL) {
		chg_err("alloc memory error\n");
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);
	chip->ufcs = ufcs_get_ufcs_device();
	if (chip->ufcs == NULL) {
		chg_err("ufcs device not found, retry");
		rc = -EPROBE_DEFER;
		goto ufcs_dev_err;
	}

	INIT_DELAYED_WORK(&chip->upload_device_id_work, oplus_vufcs_upload_device_id_work);

	snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "ufcs-virtual");
	snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x01");
	ic_cfg.get_func = oplus_chg_ufcs_get_func;
	ic_cfg.virq_data = oplus_ufcs_virq_table;
	ic_cfg.virq_num = ARRAY_SIZE(oplus_ufcs_virq_table);
	chip->ic_dev = devm_oplus_chg_ic_register(chip->dev, &ic_cfg);
	if (!chip->ic_dev) {
		rc = -ENODEV;
		chg_err("register %s error\n", node->name);
		goto reg_ic_err;
	}
	oplus_chg_ufcs_init(chip->ic_dev);
	oplus_vufcs_upload_device_id(chip);

	chg_err("probe success\n");
	return 0;

reg_ic_err:
	chip->ufcs = NULL;
ufcs_dev_err:
	devm_kfree(&pdev->dev, chip);
	platform_set_drvdata(pdev, NULL);

	chg_err("probe error, rc=%d\n", rc);
	return rc;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void oplus_virtual_ufcs_remove(struct platform_device *pdev)
#else
static int oplus_virtual_ufcs_remove(struct platform_device *pdev)
#endif
{
	struct oplus_virtual_ufcs_ic *chip = platform_get_drvdata(pdev);

	if (chip == NULL) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
		return -ENODEV;
#else
		return;
#endif
	}

	cancel_delayed_work(&chip->upload_device_id_work);

	if (chip->ic_dev->online)
		oplus_chg_ufcs_exit(chip->ic_dev);

	devm_oplus_chg_ic_unregister(&pdev->dev, chip->ic_dev);
	devm_kfree(&pdev->dev, chip);
	platform_set_drvdata(pdev, NULL);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return 0;
#endif
}

static const struct of_device_id oplus_virtual_ufcs_match[] = {
	{ .compatible = "oplus,virtual_ufcs" },
	{},
};

static struct platform_driver oplus_virtual_ufcs_driver = {
	.driver		= {
		.name = "oplus-virtual_ufcs",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(oplus_virtual_ufcs_match),
	},
	.probe		= oplus_virtual_ufcs_probe,
	.remove		= oplus_virtual_ufcs_remove,
};

static __init int oplus_virtual_ufcs_init(void)
{
#if __and(IS_BUILTIN(CONFIG_OPLUS_CHG), IS_BUILTIN(CONFIG_OPLUS_CHG_V2))
	struct device_node *node;

	node = of_find_node_by_path("/soc/oplus_chg_core");
	if (node == NULL)
		return 0;
	if (!of_property_read_bool(node, "oplus,chg_framework_v2"))
		return 0;
#endif /* CONFIG_OPLUS_CHG_V2 */
	return platform_driver_register(&oplus_virtual_ufcs_driver);
}

static __exit void oplus_virtual_ufcs_exit(void)
{
#if __and(IS_BUILTIN(CONFIG_OPLUS_CHG), IS_BUILTIN(CONFIG_OPLUS_CHG_V2))
	struct device_node *node;

	node = of_find_node_by_path("/soc/oplus_chg_core");
	if (node == NULL)
		return;
	if (!of_property_read_bool(node, "oplus,chg_framework_v2"))
		return;
#endif /* CONFIG_OPLUS_CHG_V2 */
	platform_driver_unregister(&oplus_virtual_ufcs_driver);
}

oplus_chg_module_register(oplus_virtual_ufcs);
