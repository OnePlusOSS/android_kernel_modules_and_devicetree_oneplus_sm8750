/***********************************************************
** Copyright (C), 2008-2025 Oplus. All rights reserved.
** File: oplus_sc8527.c
** Description: regcp ic
** Date: 2025-11-01
** -----------Revision History: -------------------------------
** <author>        <data>    <version >       <desc>
****************************************************************/

#define pr_fmt(fmt) "[sc8527]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/err.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/pinctrl/consumer.h>
#include <linux/sched/clock.h>
#include <linux/regmap.h>
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/system/oplus_project.h>
#endif
#include <soc/oplus/device_info.h>
#include <ufcs_class.h>
#include <oplus_chg_ic.h>
#include <oplus_chg_module.h>
#include <oplus_chg.h>
#include <oplus_mms_wired.h>
#include <oplus_mms.h>
#include <oplus_impedance_check.h>
#include "../oplus_voocphy.h"
#include "../chglib/oplus_chglib.h"
#include "oplus_sc8527.h"
#include "../monitor/oplus_chg_track.h"
#include <oplus_chg_vooc.h>
#include <oplus_chg_monitor.h>
#include <oplus_chg_voter.h>
#include <oplus_mms_gauge.h>

#define ERR_MSG_BUF	PAGE_SIZE

struct sc8527_device {
	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;
	struct oplus_voocphy_manager *voocphy;
	struct ufcs_dev *ufcs;

	struct oplus_chg_ic_dev *cp_ic;
	struct oplus_chg_ic_dev *boost_ic;
	struct oplus_chg_ic_dev *buck_ic;
	struct oplus_impedance_node *input_imp_node;
	struct oplus_impedance_node *output_imp_node;

	int auto_mode_gpio;
	struct pinctrl *pinctrl;
	struct pinctrl_state *auto_mode_enable;
	struct pinctrl_state *auto_mode_disable;

	struct mutex i2c_rw_lock;
	struct mutex chip_lock;
	atomic_t suspended;
	atomic_t i2c_err_count;
	struct wakeup_source *chip_ws;

	int ovp_reg;
	int ocp_reg;
	int ic_sc8527;
	int reg0d_val;

	bool ufcs_enable;
	bool voocphy_enable;

	enum oplus_cp_work_mode cp_work_mode;

	bool rested;
	bool error_reported;

	bool use_ufcs_phy;
	bool use_vooc_phy;
	bool vac_support;
	bool ship_mode_en;
	u8 ufcs_reg_dump[SC8527_FLAG_NUM];
	struct work_struct check_reg_work;
	struct work_struct power_on_mode_switch_work;
	struct delayed_work cp_uvp_enable_work;

	struct oplus_mms *wired_topic;
	struct mms_subscribe *wired_subs;
	bool wired_present;

	struct votable *work_mode_votable;
};

static int sc8527_check_register_and_upload_track(struct sc8527_device *chip);
static void sc8527_boost_ic_reg_set(struct sc8527_device *chip);
static int sc8527_set_work_mode(struct oplus_chg_ic_dev *dev, int mode);
#define BOOST_I2C_ERROR_VOTER	"BOOST_I2C_ERROR_VOTER"
#define I2C_RETRY_MAX	3

static bool is_work_mode_votable_available(struct sc8527_device *chip);

static void sc8527_handle_i2c_error_after_retry(struct sc8527_device *chip, int happen)
{
	static int last_happen = -1;
	if (!chip)
		return;

	if (!happen && is_work_mode_votable_available(chip)) {
		vote(chip->work_mode_votable, BOOST_I2C_ERROR_VOTER, happen, happen, false);
		return;
	}

	if (last_happen == happen) {
		return;
	}

	last_happen = happen;

	if (is_work_mode_votable_available(chip)) {
		vote(chip->work_mode_votable, BOOST_I2C_ERROR_VOTER, happen, happen, false);
		chg_err("sc8527 I2C set Mode :%d via votable\n", happen);
	} else {
		sc8527_set_work_mode(chip->boost_ic, happen);
		chg_err("work_mode_votable not available, sc8527 I2C set Mode :%d\n", happen);
	}
}

static enum oplus_cp_work_mode g_cp_support_work_mode[] = {
	CP_WORK_MODE_BYPASS,
};

#define I2C_ERR_MAX	10
static void sc8527_i2c_error(struct sc8527_device *chip, bool happen)
{
	struct vphy_chip *v_chip = NULL;
	struct oplus_voocphy_manager *voocphy = chip->voocphy;

	if (!voocphy)
		return;

	if (happen) {
		voocphy->voocphy_iic_err = true;
		voocphy->voocphy_iic_err_num++;
		if (voocphy->voocphy_iic_err_num >= I2C_ERR_MAX) {
			v_chip = oplus_chglib_get_vphy_chip(voocphy->dev);
			if (v_chip)
				oplus_chglib_creat_i2c_err(v_chip->dev); /* CP I2C error */
		}
	} else {
		voocphy->voocphy_iic_err_num = 0;
	}
	return;
}

static int sc8527_publish_ic_err_msg(int type, int sub_type, const char *format, ...)
{
	va_list args;
	char *buf;
	int rc;
	struct mms_msg *topic_msg;
	struct oplus_mms *err_topic = oplus_mms_get_by_name("error");

	if (!err_topic)
		return -ENODEV;

	buf = kzalloc(ERR_MSG_BUF, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	va_start(args, format);
	vsnprintf(buf, ERR_MSG_BUF, format, args);
	va_end(args);

	topic_msg =
		oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, ERR_ITEM_IC,
		"[%s]-[%d]-[%d]:%s", "sc8527", type, sub_type, buf);
	kfree(buf);
	if (topic_msg == NULL) {
		chg_err("alloc topic msg error\n");
		return -ENOMEM;
	}

	rc = oplus_mms_publish_msg_sync(err_topic, topic_msg);
	if (rc < 0) {
		chg_err("publish error topic msg error, rc=%d\n", rc);
		kfree(topic_msg);
	}

	return rc;
}

static void sc8527_upload_i2c_err_info(struct sc8527_device *chip, bool read, s32 *err_info)
{
	char *buf;
	size_t index = 0;

	buf = kzalloc(ERR_MSG_BUF, GFP_KERNEL);
	if (buf == NULL)
		return;

	index += scnprintf(buf + index, ERR_MSG_BUF - index,
		"$$i2c_type@@%s$$err_reg@@0x%x$$err_reason@@%d", read ? "read" : "write", err_info[0], err_info[1]);
	if (index > 0)
		buf[index - 1] = 0;

	sc8527_publish_ic_err_msg(OPLUS_IC_ERR_I2C, 0, "%s", buf);
	kfree(buf);
}

static bool sc8527_check_work_mode_support(enum oplus_cp_work_mode mode)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(g_cp_support_work_mode); i++) {
		if (g_cp_support_work_mode[i] == mode)
			return true;
	}
	return false;
}

static int __sc8527_read_byte(struct i2c_client *client, u8 reg, u8 *data)
{
	s32 ret;
	int retry = I2C_RETRY_MAX;
	struct oplus_voocphy_manager *chip = i2c_get_clientdata(client);
	s32 err_info[2] = { 0 };

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		while (retry > 0) {
			usleep_range(5000, 5000);
			ret = i2c_smbus_read_byte_data(client, reg);
			if (ret < 0)
				retry--;
			else
				break;
		}
	}

	if (ret < 0) {
		sc8527_i2c_error(chip->priv_data, true);
		chg_err("i2c read fail after %d retries: can't read from reg 0x%02X\n", I2C_RETRY_MAX, reg);
		err_info[0] = reg;
		err_info[1] = ret;
		sc8527_upload_i2c_err_info(chip->priv_data, true, err_info);
		sc8527_handle_i2c_error_after_retry(chip->priv_data, 1);
		return ret;
	}
	sc8527_handle_i2c_error_after_retry(chip->priv_data, 0);

	*data = (u8) ret;

	return 0;
}

static int __sc8527_write_byte(struct i2c_client *client, int reg, u8 val)
{
	s32 ret;
	int retry = I2C_RETRY_MAX;
	struct oplus_voocphy_manager *chip = i2c_get_clientdata(client);
	s32 err_info[2] = { 0 };

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		while (retry > 0) {
			usleep_range(5000, 5000);
			ret = i2c_smbus_write_byte_data(client, reg, val);
			if (ret < 0)
				retry--;
			else
				break;
		}
	}

	if (ret < 0) {
		sc8527_i2c_error(chip->priv_data, true);
		chg_err("i2c write fail after %d retries: can't write 0x%02X to reg 0x%02X: %d\n",
		       I2C_RETRY_MAX, val, reg, ret);
		err_info[0] = reg;
		err_info[1] = ret;
		sc8527_upload_i2c_err_info(chip->priv_data, false, err_info);
		sc8527_handle_i2c_error_after_retry(chip->priv_data, 1);
		return ret;
	}
	sc8527_handle_i2c_error_after_retry(chip->priv_data, 0);
	return 0;
}

static int sc8527_read_byte(struct sc8527_device *chip, u8 reg, u8 *data)
{
	int ret;

	if (chip == NULL) {
		chg_err("sc8527 chip is NULL\n");
		return -ENODEV;
	}

	mutex_lock(&chip->i2c_rw_lock);
	ret = __sc8527_read_byte(chip->client, reg, data);
	mutex_unlock(&chip->i2c_rw_lock);

	return ret;
}

static int sc8527_write_byte(struct sc8527_device *chip, u8 reg, u8 data)
{
	int ret;

	if (chip == NULL) {
		chg_err("sc8527 chip is NULL\n");
		return -ENODEV;
	}

	mutex_lock(&chip->i2c_rw_lock);
	ret = __sc8527_write_byte(chip->client, reg, data);
	mutex_unlock(&chip->i2c_rw_lock);

	return ret;
}

static int sc8527_update_bits(struct i2c_client *client, u8 reg,
						      u8 mask, u8 data)
{
	struct sc8527_device *chip;
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);
	int ret;
	u8 tmp;

	if (voocphy == NULL) {
		chg_err("voocphy is NULL\n");
		return -ENODEV;
	}
	chip = voocphy->priv_data;
	if (chip == NULL) {
		chg_err("sc8527 chip is NULL\n");
		return -ENODEV;
	}

	mutex_lock(&chip->i2c_rw_lock);
	ret = __sc8527_read_byte(client, reg, &tmp);
	if (ret) {
		chg_err("failed to read %02X register, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __sc8527_write_byte(client, reg, tmp);
	if (ret)
		chg_err("failed to write %02X register, ret=%d\n", reg, ret);
out:
	mutex_unlock(&chip->i2c_rw_lock);
	chg_err("write 0x%02X : 0x%02X\n", reg, tmp);
	return ret;
}

static s32 sc8527_read_word(struct i2c_client *client, u8 reg)
{
	s32 ret;
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);
	struct sc8527_device *chip;

	if (voocphy == NULL) {
		chg_err("voocphy is NULL\n");
		return -ENODEV;
	}
	chip = voocphy->priv_data;
	if (chip == NULL) {
		chg_err("sc8527 chip is NULL\n");
		return -ENODEV;
	}

	mutex_lock(&chip->i2c_rw_lock);
	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0) {
		int retry = I2C_RETRY_MAX;
		while (retry > 0) {
			usleep_range(5000, 5000);
			ret = i2c_smbus_read_word_data(client, reg);
			if (ret < 0)
				retry--;
			else
				break;
		}
	}

	if (ret < 0) {
		sc8527_i2c_error(voocphy->priv_data, true);
		chg_err("i2c read word fail after %d retries: can't read reg:0x%02X \n", I2C_RETRY_MAX, reg);
		sc8527_handle_i2c_error_after_retry(chip, 1);
		mutex_unlock(&chip->i2c_rw_lock);
		return ret;
	}
	sc8527_handle_i2c_error_after_retry(chip, 0);
	mutex_unlock(&chip->i2c_rw_lock);

	return ret;
}

static s32 sc8527_write_word(struct i2c_client *client, u8 reg, u16 val)
{
	s32 ret;
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);
	struct sc8527_device *chip;

	if (voocphy == NULL) {
		chg_err("voocphy is NULL\n");
		return -ENODEV;
	}
	chip = voocphy->priv_data;
	if (chip == NULL) {
		chg_err("sc8527 chip is NULL\n");
		return -ENODEV;
	}

	mutex_lock(&chip->i2c_rw_lock);
	ret = i2c_smbus_write_word_data(client, reg, val);
	if (ret < 0) {
		int retry = I2C_RETRY_MAX;
		while (retry > 0) {
			usleep_range(5000, 5000);
			ret = i2c_smbus_write_word_data(client, reg, val);
			if (ret < 0)
				retry--;
			else
				break;
		}
	}

	if (ret < 0) {
		sc8527_i2c_error(voocphy->priv_data, true);
		chg_err("i2c write word fail after %d retries: can't write 0x%02X to reg:0x%02X\n", I2C_RETRY_MAX, val, reg);
		sc8527_handle_i2c_error_after_retry(chip, 1);
		mutex_unlock(&chip->i2c_rw_lock);
		return ret;
	}
	sc8527_handle_i2c_error_after_retry(chip, 0);
	mutex_unlock(&chip->i2c_rw_lock);

	return 0;
}

static int sc8527_read_i2c_nonblock(struct i2c_client *client, u8 reg, u8 length, u8 *returnData)
{
	int rc = 0;
	int retry = I2C_RETRY_MAX;
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);

	rc = i2c_smbus_read_i2c_block_data(client, reg, length, returnData);
	if (rc < 0) {
		while (retry > 0) {
			usleep_range(5000, 5000);
			rc = i2c_smbus_read_i2c_block_data(client, reg, length, returnData);
			if (rc < 0)
				retry--;
			else
				break;
		}
	}

	if (rc < 0) {
		if (voocphy && voocphy->priv_data) {
			sc8527_i2c_error(voocphy->priv_data, true);
			sc8527_handle_i2c_error_after_retry(voocphy->priv_data, 1);
		}
		chg_err("read err after %d retries, rc = %d,\n", I2C_RETRY_MAX, rc);
	} else {
		if (voocphy && voocphy->priv_data) {
			sc8527_i2c_error(voocphy->priv_data, false);
			sc8527_handle_i2c_error_after_retry(voocphy->priv_data, 0);
		}
	}

	return rc;
}

static int sc8527_read_i2c_block(struct i2c_client *client, u8 reg, u8 length, u8 *returnData)
{
	struct sc8527_device *chip;
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);
	int rc = 0;
	int retry;

	if (voocphy == NULL) {
		chg_err("voocphy is NULL\n");
		return -ENODEV;
	}
	chip = voocphy->priv_data;
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return -ENODEV;
	}

	mutex_lock(&chip->i2c_rw_lock);
	rc = i2c_smbus_read_i2c_block_data(client, reg, length, returnData);

	if (rc < 0) {
		retry = I2C_RETRY_MAX;
		while (retry > 0) {
			usleep_range(5000, 5000);
			rc = i2c_smbus_read_i2c_block_data(client, reg, length, returnData);
			if (rc < 0)
				retry--;
			else
				break;
		}
	}

	if (rc < 0) {
		sc8527_i2c_error(chip, true);
		chg_err("read err after %d retries, rc = %d,\n", I2C_RETRY_MAX, rc);
		sc8527_handle_i2c_error_after_retry(chip, 1);
	} else {
		sc8527_i2c_error(chip, false);
		sc8527_handle_i2c_error_after_retry(chip, 0);
	}
	mutex_unlock(&chip->i2c_rw_lock);

	return rc;
}

static int sc8527_write_data(struct sc8527_device *chip, u8 addr,
			      u16 length, u8 *data)
{
	u8 *buf;
	int rc = 0;

	buf = kzalloc(length + 1, GFP_KERNEL);
	if (!buf) {
		chg_err("alloc memorry for i2c buffer error\n");
		return -ENOMEM;
	}

	buf[0] = addr & 0xff;
	memcpy(&buf[1], data, length);

	mutex_lock(&chip->i2c_rw_lock);
	rc = i2c_master_send(chip->client, buf, length + 1);
	if (rc < length + 1) {
		int retry = I2C_RETRY_MAX;
		while (retry > 0 && rc < length + 1) {
			usleep_range(5000, 5000);
			rc = i2c_master_send(chip->client, buf, length + 1);
			if (rc < length + 1)
				retry--;
			else
				break;
		}
	}

	if (rc < length + 1) {
		chg_err("write 0x%04x error after %d retries, ret = %d \n", addr, I2C_RETRY_MAX, rc);
		sc8527_handle_i2c_error_after_retry(chip, 1);
		mutex_unlock(&chip->i2c_rw_lock);
		kfree(buf);
		rc = rc < 0 ? rc : -EIO;
		return rc;
	}
	sc8527_handle_i2c_error_after_retry(chip, 0);
	mutex_unlock(&chip->i2c_rw_lock);
	kfree(buf);
	return rc;
}

#define I2C_MSG_LEN	2
static int sc8527_read_data(struct sc8527_device *chip, u8 addr, u8 *buf, int len)
{
	int rc = 0;
	struct i2c_msg msg[I2C_MSG_LEN] = {0};

	if (!chip)
		return -EINVAL;

	msg[0].addr = chip->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &addr;

	msg[1].addr = chip->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = buf;

	mutex_lock(&chip->i2c_rw_lock);
	rc = i2c_transfer(chip->client->adapter, msg, I2C_MSG_LEN);
	if (rc < 0) {
		int retry = I2C_RETRY_MAX;
		while (retry > 0) {
			usleep_range(5000, 5000);
			rc = i2c_transfer(chip->client->adapter, msg, I2C_MSG_LEN);
			if (rc < 0)
				retry--;
			else
				break;
		}
	}

	if (rc < 0) {
		chg_err("read 0x%02x error after %d retries, rc=%d\n", addr, I2C_RETRY_MAX, rc);
		sc8527_i2c_error(chip, true);
		sc8527_handle_i2c_error_after_retry(chip, 1);
		mutex_unlock(&chip->i2c_rw_lock);
		return rc;
	}
	sc8527_i2c_error(chip, false);
	sc8527_handle_i2c_error_after_retry(chip, 0);
	mutex_unlock(&chip->i2c_rw_lock);
	return 0;
}

static int sc8527_set_predata(struct oplus_voocphy_manager *chip, u16 val)
{
	s32 ret;
	if (!chip) {
		chg_err("chip is null\n");
		return -1;
	}

	/* predata */
	ret = sc8527_write_word(chip->client, SC8527_REG_2A, val);
	if (ret < 0) {
		chg_err("failed to write predata(%d)\n", ret);
		return ret;
	}

	chg_info("write predata 0x%0x\n", val);

	return ret;
}

static int sc8527_set_txbuff(struct oplus_voocphy_manager *chip, u16 val)
{
	s32 ret;

	if (!chip) {
		chg_err("chip is null\n");
		return -1;
	}

	/* txbuff */
	ret = sc8527_write_word(chip->client, SC8527_REG_25, val);
	if (ret < 0) {
		chg_err("failed to  write txbuff(%d)\n", ret);
		return ret;
	}

	return ret;
}

static int sc8527_get_adapter_info(struct oplus_voocphy_manager *chip)
{
	s32 data;

	if (!chip) {
		chg_err("chip is null\n");
		return -1;
	}

	data = sc8527_read_word(chip->client, SC8527_REG_27);

	if (data < 0) {
		chg_err("failed to get adapter info(%d)\n", data);
		return data;
	}

	VOOCPHY_DATA16_SPLIT(data, chip->voocphy_rx_buff, chip->vooc_flag);
	chg_info("data: 0x%0x, vooc_flag: 0x%0x, vooc_rxdata: 0x%0x\n",
		 data, chip->vooc_flag, chip->voocphy_rx_buff);

	return 0;
}

static void sc8527_update_data(struct oplus_voocphy_manager *chip)
{
	/* interrupt_flag */
	/* sc8527_read_byte(chip->priv_data, SC8527_REG_09, &data); */
	chip->interrupt_flag = 0;
	chip->cp_vsys = 0;
	chip->cp_vbat = 0;
	chip->cp_ichg = 0;
	chip->cp_vbus = 0;
}

static int sc8527_cp_vbus(struct oplus_voocphy_manager *chip)
{
	int vbus;
	struct oplus_mms *wired_topic;
	union mms_msg_data data = { 0 };

	wired_topic = oplus_mms_get_by_name("wired");
	if (!wired_topic)
		return 0;

	oplus_mms_get_item_data(wired_topic, WIRED_ITEM_VBUS, &data, true);
	vbus = data.intval;

	return vbus;
}

static int sc8527_get_cp_ichg(struct oplus_voocphy_manager *chip)
{
	return 0;
}

int sc8527_get_cp_vbat(struct oplus_voocphy_manager *chip)
{
	return 0;
}

static int sc8527_reg_reset(struct i2c_client *client, bool enable)
{
	int ret = 0;

	/* TODO*/

	return ret;
}

static int sc8527_get_chg_enable(struct oplus_voocphy_manager *chip, u8 *data)
{
	int ret = 0;

	if (!chip) {
		chg_err("chip is null\n");
		return -1;
	}

	ret = sc8527_read_byte(chip->priv_data, SC8527_REG_02, data);
	if (ret < 0) {
		chg_err("failed get chg enable status(%d)\n", ret);
		return ret;
	}

	*data = *data & SC8527_CHG_EN_MASK;

	return ret;
}

static int sc8527_set_uvp_disable(struct oplus_voocphy_manager *chip, bool disable)
{
	int ret = 0;

	if (!chip) {
		chg_err("chip is null\n");
		return -1;
	}

	ret = sc8527_update_bits(chip->client, SC8527_REG_3A,
				 SC8527_VAC2V2X_UVP_ENABLE_MASK,
				 disable ? SC8527_VAC2V2X_UVP_DISABLE :
					   SC8527_VAC2V2X_UVP_ENABLE);
	if (ret < 0) {
		chg_err("set reg_0x3a bit6 failed(%d)\n", ret);
		return ret;
	}

	return ret;
}

static void sc8527_cp_uvp_enable_work_func(struct work_struct *work)
{
	struct sc8527_device *dev =
		container_of(work, struct sc8527_device, cp_uvp_enable_work.work);
	int ret;

	if (!dev || !dev->voocphy) {
		chg_err("chip or voocphy is null\n");
		return;
	}

	ret = sc8527_set_uvp_disable(dev->voocphy, false);
	if (ret < 0)
		chg_err("enable uvp failed(%d)\n", ret);
}

static int sc8527_set_chg_enable(struct oplus_voocphy_manager *chip, bool enable)
{
	struct sc8527_device *dev;
	int ret = 0;

	if (!chip) {
		chg_err("chip is null\n");
		return -1;
	}

	dev = chip->priv_data;
	if (dev == NULL) {
		chg_err("sc8527 device is NULL\n");
		return -ENODEV;
	}

	cancel_delayed_work_sync(&dev->cp_uvp_enable_work);
	if (enable) {
		ret = sc8527_set_uvp_disable(chip, true);
		if (ret < 0)
			return ret;
	}

	/* vac range disable */
	sc8527_write_byte(chip->priv_data, SC8527_REG_02, 0x7a);

	if (enable)
		ret = sc8527_write_byte(chip->priv_data, SC8527_REG_02, ENABLE_MOS); /* enable mos */
	else
		ret = sc8527_write_byte(chip->priv_data, SC8527_REG_02, DISENABLE_MOS); /* disable mos */

	if (ret < 0) {
		chg_err("failed to set chg enable(%d)\n", ret);
		return ret;
	}
	if (enable)
		schedule_delayed_work(&dev->cp_uvp_enable_work, msecs_to_jiffies(1000));

	chg_err("%s\n", enable ? "enable" : "false");

	return ret;
}


static int sc8527_get_adc_enable(struct oplus_voocphy_manager *chip, u8 *data)
{
	return 0;
}

static int sc8527_set_adc_enable(struct oplus_voocphy_manager *chip, bool enable)
{
	return 0;
}

static u8 sc8527_get_vbus_status(struct oplus_voocphy_manager *chip)
{
	int ret = 0;
	u8 value = 0;

	if (!chip) {
		chg_err("chip is null\n");
		return -1;
	}

	ret = sc8527_read_byte(chip->priv_data, SC8527_REG_0B, &value);
	if (ret < 0) {
		chg_err("failed to get vbus status(%d)\n", ret);
		return ret;
	}

	value = (value & VBUS_INRANGE_STATUS_MASK) >> VBUS_INRANGE_STATUS_SHIFT;
	chg_info("vbus status: %d\n", value);

	return value;
}

static int sc8527_get_voocphy_enable(struct oplus_voocphy_manager *chip, u8 *data)
{
	int ret = 0;

	if (!chip) {
		chg_err("chip is null\n");
		return -1;
	}

	ret = sc8527_read_byte(chip->priv_data, SC8527_REG_2B, data);
	if (ret < 0)
		chg_err("failed to get voocphy enable status(%d)\n", ret);

	return ret;
}

static void sc8527_dump_reg_in_err_issue(struct oplus_voocphy_manager *chip)
{
	if (!chip) {
		chg_err("chip is null\n");
		return;
	}

	chg_info("SC8527_REG_09 -->09~0E[0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x]\n",
		 chip->reg_dump[9], chip->reg_dump[10], chip->reg_dump[11],
		 chip->reg_dump[12], chip->reg_dump[13], chip->reg_dump[14]);

	return;
}

static u8 sc8527_get_int_value(struct oplus_voocphy_manager *chip)
{
	int ret = 0;
	u8 int_column[6];

	if (!chip) {
		chg_err("chip is null\n");
		return -1;
	}

	ret = sc8527_read_i2c_block(chip->client, SC8527_REG_09, 6, int_column);
	if (ret < 0) {
		sc8527_i2c_error(chip->priv_data, true);
		chg_err("read SC8527_REG_09 6 bytes failed(%d)\n", ret);
		memset(chip->int_column, 0, sizeof(chip->int_column));
		/* set int_column[1]=1, otherwise the dcdc chg can't be enabled*/
		chip->int_column[1] = BIT(0);
		return chip->int_column[1];
	}

	memcpy(chip->int_column, int_column, sizeof(chip->int_column));
	chg_info("SC8527_REG_09 -->09~0E[0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x]\n",
		 chip->int_column[0], chip->int_column[1], chip->int_column[2],
		 chip->int_column[3], chip->int_column[4], chip->int_column[5]);

	return int_column[1];
}

static u8 sc8527_get_chg_auto_mode(struct oplus_voocphy_manager *chip)
{
	int ret = 0;
	u8 value = 0;

	if (!chip) {
		chg_err("chip is null\n");
		return -1;
	}
	ret = sc8527_read_byte(chip->priv_data, SC8527_REG_15, &value);
	value = value & SC8527_CHG_MODE_MASK;
	chg_info("get auto mode value = %d\n", value);

	return value;
}

static int sc8527_set_chg_auto_mode(struct oplus_voocphy_manager *chip, bool enable)
{
	int ret = 0;

	if (!chip) {
		chg_err("chip is null\n");
		return -1;
	}

	chg_info("enable = %d\n", enable);

	if (enable && (sc8527_get_chg_auto_mode(chip) == SC8527_CHG_FIX_MODE))
		ret = sc8527_update_bits(chip->client, SC8527_REG_15,
					 SC8527_CHG_MODE_MASK,
					 SC8527_CHG_AUTO_MODE);
	else if (!enable && (sc8527_get_chg_auto_mode(chip) == SC8527_CHG_AUTO_MODE))
		ret = sc8527_update_bits(chip->client, SC8527_REG_15,
					 SC8527_CHG_MODE_MASK,
					 SC8527_CHG_FIX_MODE);
	if (ret < 0)
		chg_err("failed to %s auto mode\n", enable ? "enable" : "disable");

	return ret;
}

static void sc8527_set_pd_svooc_config(struct oplus_voocphy_manager *chip, bool enable)
{
	if (!chip) {
		chg_err("chip is null\n");
		return;
	}

	sc8527_write_byte(chip->priv_data, SC8527_REG_04, 0x36); /* WD:1000ms */
	sc8527_write_byte(chip->priv_data, SC8527_REG_2C, 0xd1); /* Loose_det=1 */

	chg_info("pd svooc \n");
}

static bool sc8527_get_pd_svooc_config(struct oplus_voocphy_manager *chip)
{
	if (!chip) {
		chg_err("Failed\n");
		return false;
	}
	chg_info("no ucp flag,return true\n");

	return true;
}

void sc8527_send_handshake(struct oplus_voocphy_manager *chip)
{
	/* enable voocphy and handshake */
	sc8527_write_byte(chip->priv_data, SC8527_REG_24, 0x81);
}


static int sc8527_reset_voocphy(struct oplus_voocphy_manager *chip)
{
	struct sc8527_device *dev;

	dev = chip->priv_data;
	if (dev == NULL) {
		chg_err("sc8527 chip is NULL\n");
		return -ENODEV;
	}
	cancel_delayed_work_sync(&dev->cp_uvp_enable_work);
	/* turn off mos */
	sc8527_write_byte(chip->priv_data, SC8527_REG_02, 0x78);
	/* clear tx data */
	sc8527_write_byte(chip->priv_data, SC8527_REG_25, 0x00);
	sc8527_write_byte(chip->priv_data, SC8527_REG_26, 0x00);
	/* disable vooc phy irq */
	sc8527_write_byte(chip->priv_data, SC8527_REG_29, 0x7F); /* mask all flag */
	sc8527_write_byte(chip->priv_data, SC8527_REG_10, 0x79); /* disable irq */
	/* set D+ HiZ */
	sc8527_write_byte(chip->priv_data, SC8527_REG_20, 0xc0);

	/* select big bang mode */
	/* disable vooc */
	sc8527_write_byte(chip->priv_data, SC8527_REG_24, 0x00);
	sc8527_write_byte(chip->priv_data, SC8527_REG_04, 0x06); /* disable wdt */

	/* set predata */
	/* sc8527_write_word(chip->client, SC8527_REG_2A, 0x0); */
	sc8527_write_byte(chip->priv_data, SC8527_REG_24, 0x02); /* reset voocphy */
	msleep(1);
	/* sc8527_set_predata(0x0); */
	dev->voocphy_enable = false;
	chg_err("oplus_vooc_reset_voocphy done");

	return VOOCPHY_SUCCESS;
}


static int sc8527_reactive_voocphy(struct oplus_voocphy_manager *chip)
{
	/* set predata 0 */
	sc8527_write_word(chip->client, SC8527_REG_2A, 0x0);

	/* dpdm */
	sc8527_write_byte(chip->priv_data, SC8527_REG_20, 0x21);
	sc8527_write_byte(chip->priv_data, SC8527_REG_21, 0x80);
	sc8527_write_byte(chip->priv_data, SC8527_REG_2C, 0xD1);

	/* clear tx data */
	sc8527_write_byte(chip->priv_data, SC8527_REG_25, 0x00);
	sc8527_write_byte(chip->priv_data, SC8527_REG_26, 0x00);

	/* vooc */
	sc8527_write_byte(chip->priv_data, SC8527_REG_29, 0x05); /* diable vooc int flag mask */
	sc8527_send_handshake(chip);

	chg_info("oplus_vooc_reactive_voocphy done");

	return VOOCPHY_SUCCESS;
}

static int sc8527_init_device(struct sc8527_device *chip)
{
	sc8527_reg_reset(chip->client, true);
	msleep(10);
	sc8527_write_byte(chip, SC8527_REG_24, 0x00); /* VOOC_CTRL:disable,no handshake */
	chg_info("\n");

	return 0;
}

static int sc8527_init_vooc(struct oplus_voocphy_manager *chip)
{
	struct sc8527_device *dev;

	dev = chip->priv_data;
	chg_info("start init vooc\n");

	dev->voocphy_enable = true;
	sc8527_write_byte(chip->priv_data, SC8527_REG_24, 0x02); /* reset voocphy */
	sc8527_write_word(chip->client, SC8527_REG_2A, 0x00); /* reset predata 00 */
	msleep(1);
	if (oplus_chg_get_fcs_support_flags()) {
		sc8527_write_byte(chip->priv_data, SC8527_REG_20, 0x31);
		msleep(30);
	}
	sc8527_write_byte(chip->priv_data, SC8527_REG_20, 0x21); /* VOOC_CTRL:disable,no handshake */
	sc8527_write_byte(chip->priv_data, SC8527_REG_21, 0x80); /* VOOC_CTRL:disable,no handshake */
	sc8527_write_byte(chip->priv_data, SC8527_REG_2C, 0xD1); /* VOOC_CTRL:disable,no handshake */
	sc8527_write_byte(chip->priv_data, SC8527_REG_29, 0x25); /* mask sedseq flag,rx start,tx done */
	/* sc8527_vac_inrange_enable(chip, SC8527_VAC_INRANGE_DISABLE); */

	return 0;
}

static void sc8527_hardware_init(struct oplus_voocphy_manager *chip)
{
	chg_info("\n");
	sc8527_reg_reset(chip->client, true);
	msleep(10);
	sc8527_write_byte(chip->priv_data, SC8527_REG_01, 0x5e); /* disable v1x_scp */
	sc8527_write_byte(chip->priv_data, SC8527_REG_06, 0x85); /* enable  audio mode */
	sc8527_write_byte(chip->priv_data, SC8527_REG_08, 0xA6); /* REF_SKIP_R 40mv */
	sc8527_write_byte(chip->priv_data, SC8527_REG_29, 0x05); /* Masked Pulse_filtered, RX_Start,Tx_Done,soft intflag */
	sc8527_write_byte(chip->priv_data, SC8527_REG_10, 0x79); /* Masked Pulse_filtered, RX_Start,Tx_Done */
	sc8527_write_byte(chip->priv_data, SC8527_REG_03, 0xFF); /* set rvs and fwd ocp */
	sc8527_write_byte(chip->priv_data, SC8527_REG_12, 0x10); /* set OCP trigger time to 10us */
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if (get_eng_version() == HIGH_TEMP_AGING)
		sc8527_write_byte(chip->priv_data, SC8527_REG_0F, 0x06); /* mask 100/120 TDIE alarm */
#endif
	if (chip->v2x_volt_full_open_low) {
		sc8527_update_bits(chip->client, SC8527_REG_00, 0xc, 0); /* if device power off voltage is lower than 3V, need change 8527 full open thr */
	}
}

static int sc8527_dump_registers(struct oplus_voocphy_manager *chip)
{
	int ret = 0;
	u8 int_column[7];

	if (!chip) {
		chg_err("chip is null\n");
		return -1;
	}

	ret = sc8527_read_i2c_block(chip->client, SC8527_REG_00, 7, int_column);
	if (ret < 0) {
		chg_err("read SC8527_REG_00 7 bytes failed(%d)\n", ret);
		return ret;
	}
	chg_info("[00~06][0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x]\n",
		 int_column[0], int_column[1], int_column[2],
		 int_column[3], int_column[4], int_column[5], int_column[6]);

	ret = sc8527_read_i2c_block(chip->client, SC8527_REG_07, 7, int_column);
	if (ret < 0) {
		chg_err("read SC8527_REG_07 7 bytes failed[%d]\n", ret);
		return ret;
	}
	chg_info("[07~0D][0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x]\n",
		 int_column[0], int_column[1], int_column[2],
		 int_column[3], int_column[4], int_column[5], int_column[6]);

	ret = sc8527_read_i2c_block(chip->client, SC8527_REG_0E, 7, int_column);
	if (ret < 0) {
		chg_err("read SC8527_REG_0E 7 bytes failed[%d]\n", ret);
		return ret;
	}
	chg_info("[0E~14][0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x]\n",
		 int_column[0], int_column[1], int_column[2],
		 int_column[3], int_column[4], int_column[5], int_column[6]);

	return ret;
}

static int sc8527_svooc_hw_setting(struct oplus_voocphy_manager *chip)
{
	sc8527_write_byte(chip->priv_data, SC8527_REG_04, 0x36); /* WD:1000ms */
	sc8527_write_byte(chip->priv_data, SC8527_REG_2C, 0xd1); /* Loose_det=1 */

	return 0;
}

static int volt_curr_to_bits(int voltage_mv, int offset, int step)
{
	if (voltage_mv < offset)
		return 0;
	return (voltage_mv - offset) / step;
}

int sc8527_set_cv_volt(struct oplus_chg_ic_dev *ic_dev, int voltage_mv)
{
	struct sc8527_device *chip;
	int bits = volt_curr_to_bits(voltage_mv, V1X_CV_OFFSET, STEP_50MV);
	int bits_shift = bits << SC8527_CV_VOLT_SHIFT;
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);
	bits_shift = bits_shift > SC8527_CV_VOLT_MASK ? SC8527_CV_VOLT_MASK : bits_shift;
	chg_err("cv_volt:%d, bits:%d, bits_shift:%d\n", voltage_mv, bits, bits_shift);
	return sc8527_update_bits(chip->client, SC8527_REG_16, SC8527_CV_VOLT_MASK, bits_shift);
}

int sc8527_set_v1x_th_cv2cp_l(struct sc8527_device *chip, int voltage_mv)
{
	int bits = volt_curr_to_bits(voltage_mv, V1X_l_TH_OFFSET, STEP_50MV);
	int bits_shift = bits << SC8527_V1X_TH_CV2CP_L_SHIFT;

	bits_shift = bits_shift > SC8527_V1X_TH_CV2CP_L_MASK ? SC8527_V1X_TH_CV2CP_L_MASK : bits_shift;
	chg_err("cv2cp_l:%d, bits:%d, bits_shift:%d\n", voltage_mv, bits, bits_shift);

	return sc8527_update_bits(chip->client, SC8527_REG_16, SC8527_V1X_TH_CV2CP_L_MASK, bits_shift);
}

int sc8527_set_v1x_th_cv2cp_h(struct sc8527_device *chip, int voltage_mv)
{
	int bits = volt_curr_to_bits(voltage_mv, V1X_TH_OFFSET, STEP_50MV);
	int bits_shift = bits << SC8527_V1X_TH_CV2CP_H_SHIFT;

	bits_shift = bits_shift > SC8527_V1X_TH_CV2CP_H_MASK ? SC8527_V1X_TH_CV2CP_H_MASK : bits_shift;
	chg_err("cv2cp_h:%d, bits:%d, bits_shift:%d\n", voltage_mv, bits, bits_shift);

	return sc8527_update_bits(chip->client, SC8527_REG_17, SC8527_V1X_TH_CV2CP_H_MASK, bits_shift);
}

int sc8527_set_v1x_th_cp2cv(struct sc8527_device *chip, int voltage_mv)
{
	int bits = volt_curr_to_bits(voltage_mv, V1X_TH_OFFSET, STEP_50MV);
	int bits_shift = bits << SC8527_V1X_TH_CV2CP_SHIFT;

	bits_shift = bits_shift > SC8527_V1X_TH_CV2CP_MASK ? SC8527_V1X_TH_CV2CP_MASK : bits_shift;
	chg_err("cp2cv:%d, bits:%d, bits_shift:%d\n", voltage_mv, bits, bits_shift);

	return sc8527_update_bits(chip->client, SC8527_REG_17, SC8527_V1X_TH_CV2CP_MASK, bits_shift);
}

static int sc8527_get_in_cv_mode(struct oplus_chg_ic_dev *ic_dev, bool *cv_mode)
{
	int ret = 0;
	u8 value = 0;
	struct sc8527_device *chip;
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	if (!chip) {
		chg_err("chip is null\n");
		return -ENODEV;
	}
	ret = sc8527_read_byte(chip, SC8527_REG_1A, &value);
	value = value & SC8527_CONV_MODE_STAT_MASK;
	chg_info("get cv mode value = %d\n", value);
	if (ret < 0) {
		chg_err("i2c error:%d\n", ret);
		*cv_mode = false;
		return ret;
	}
	if (value == SC8527_CV_MODE || value == SC8527_BYP_MODE)
		*cv_mode = true;

	return 0;
}

static int sc8527_vooc_hw_setting(struct oplus_voocphy_manager *chip)
{
	sc8527_write_byte(chip->priv_data, SC8527_REG_04, 0x46); /* WD:5000ms */
	sc8527_write_byte(chip->priv_data, SC8527_REG_2C, 0xd1); /* Loose_det=1 */

	return 0;
}

static int sc8527_5v2a_hw_setting(struct oplus_voocphy_manager *chip)
{
	sc8527_write_byte(chip->priv_data, SC8527_REG_02, 0x78); /* disable acdrv,close mos */
	sc8527_write_byte(chip->priv_data, SC8527_REG_04, 0x06); /* dsiable wdt */
	sc8527_write_byte(chip->priv_data, SC8527_REG_24, 0x00); /* close voocphy */

	return 0;
}

static int sc8527_pdqc_hw_setting(struct oplus_voocphy_manager *chip)
{
	sc8527_write_byte(chip->priv_data, SC8527_REG_02, 0x78); /* disable acdrv,close mos */
	sc8527_write_byte(chip->priv_data, SC8527_REG_04, 0x06); /* dsiable wdt */
	sc8527_write_byte(chip->priv_data, SC8527_REG_24, 0x00); /* close voocphy */

	return 0;
}

static int sc8527_hw_setting(struct oplus_voocphy_manager *chip, int reason)
{
	if (!chip) {
		chg_err("chip is null exit\n");
		return -1;
	}
	chg_info("reason = %d\n", reason);
	switch (reason) {
	case SETTING_REASON_PROBE:
	case SETTING_REASON_RESET:
		sc8527_init_device(chip->priv_data);
		chg_info("SETTING_REASON_RESET OR PROBE\n");
		break;
	case SETTING_REASON_SVOOC:
		sc8527_svooc_hw_setting(chip);
		chg_info("SETTING_REASON_SVOOC\n");
		break;
	case SETTING_REASON_VOOC:
		sc8527_vooc_hw_setting(chip);
		chg_info("SETTING_REASON_VOOC\n");
		break;
	case SETTING_REASON_5V2A:
		sc8527_5v2a_hw_setting(chip);
		chg_info("SETTING_REASON_5V2A\n");
		break;
	case SETTING_REASON_PDQC:
		sc8527_pdqc_hw_setting(chip);
		chg_info("SETTING_REASON_PDQC\n");
		break;
	default:
		chg_info("do nothing\n");
		break;
	}
	return 0;
}

static bool sc8527_check_cp_int_happened(struct oplus_voocphy_manager *chip,
					 bool *dump_reg, bool *send_info)
{
	int i = 0;

	if (((bidirect_int_flag[0].mask & chip->int_column_pre[1]) == 0) && bidirect_int_flag[i].mark_except) {
		*dump_reg = true;
		*send_info = true;
		memcpy(&chip->reg_dump[9], chip->int_column_pre, sizeof(chip->int_column_pre));
		chg_info("cp int happened %s\n", bidirect_int_flag[i].except_info);
		return true;
	}

	for (i = 1; i < 7; i++) {
		if ((bidirect_int_flag[i].mask & chip->int_column_pre[3]) && bidirect_int_flag[i].mark_except) {
			*dump_reg = true;
			*send_info = true;
			memcpy(&chip->reg_dump[9], chip->int_column_pre, sizeof(chip->int_column_pre));
			chg_info("cp int happened %s\n", bidirect_int_flag[i].except_info);
			return true;
		}
	}

	for (i = 7; i < BIDIRECT_IRQ_EVNET_NUM; i++) {
		if ((bidirect_int_flag[i].mask & chip->int_column_pre[5]) && bidirect_int_flag[i].mark_except) {
			*dump_reg = true;
			*send_info = true;
			memcpy(&chip->reg_dump[9], chip->int_column_pre, sizeof(chip->int_column_pre));
			chg_info("cp int happened %s\n", bidirect_int_flag[i].except_info);
			return true;
		}
	}

	return false;
}

#define TRACK_LOCAL_T_NS_TO_S_THD		1000000000
#define TRACK_UPLOAD_COUNT_MAX			10
#define TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD	(24 * 3600)
#define REASON_LENGTH_MAX			1024
#define ERR_LENGTH_MAX				64
#define DUMP_LENGTH_MAX				512

static int sc8527_track_get_local_time_s(void)
{
	int local_time_s;

	local_time_s = local_clock() / TRACK_LOCAL_T_NS_TO_S_THD;
	return local_time_s;
}

static int sc8527_track_upload_cp_err_info_simple(struct sc8527_device *chip, int err_flag)
{
	int index = 0;
	int curr_time;
	static int upload_count = 0;
	static int pre_upload_time = 0;
	char temp_str[REASON_LENGTH_MAX] = {0};
	struct oplus_mms *err_topic;
	struct mms_msg *msg = NULL;
	int rc = 0;

	if (NULL == chip) {
		chg_err("chip is NULL");
		return -EINVAL;
	}

	err_topic = oplus_mms_get_by_name("error");
	if (!err_topic) {
		chg_err("error topic not found\n");
		return -EINVAL;
	}

	curr_time = sc8527_track_get_local_time_s();
	if (curr_time - pre_upload_time > TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD)
		upload_count = 0;

	if (upload_count > TRACK_UPLOAD_COUNT_MAX) {
		chg_info("cp_err_uploading upload_count = %d > max %d, should return\n",
			 upload_count, TRACK_UPLOAD_COUNT_MAX);
		return 0;
	}

	upload_count++;
	pre_upload_time = sc8527_track_get_local_time_s();

	index += scnprintf(&(temp_str[index]), REASON_LENGTH_MAX - index, "$$device_id@@%s", "sc8527");
	index += scnprintf(&(temp_str[index]),
		REASON_LENGTH_MAX - index, "$$err_scene@@sc8527_chip_work_err");

	index += scnprintf(&(temp_str[index]),
		REASON_LENGTH_MAX - index,
		"$$err_reason@@%02x", err_flag);
	index += scnprintf(&(temp_str[index]),
		REASON_LENGTH_MAX - index,
		"$$err_position@@%s", "main");

	msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
		ERR_ITEM_ERR_PHY_CP_INFO, temp_str);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -EINVAL;
	}
	rc = oplus_mms_publish_msg_sync(err_topic, msg);
	if (rc < 0) {
		chg_err("publish msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return 0;
}

static int sc8527_check_register_and_upload_track(struct sc8527_device *chip)
{
	int ret = 0;
	uint8_t reg09_val = 0;
	uint8_t reg0a_val = 0;
	uint8_t reg0b_val = 0;
	uint8_t reg0c_val = 0;
	uint8_t reg0d_val = 0;
	uint8_t reg0e_val = 0;
	uint8_t reg1a_val = 0;
	uint8_t reg1b_val = 0;
	unsigned int err_flag = 0;

	if (!chip)
		return -EINVAL;

	/* Read register 0x09 (STATUS1) */
	ret = sc8527_read_byte(chip, SC8527_REG_09, &reg09_val);
	if (ret < 0) {
		chg_err("Failed to read register 0x09: %d\n", ret);
		return ret;
	}

	/* Read register 0x0A (STATUS2) */
	ret = sc8527_read_byte(chip, SC8527_REG_0A, &reg0a_val);
	if (ret < 0) {
		chg_err("Failed to read register 0x0A: %d\n", ret);
		return ret;
	}

	/* Read register 0x0B (STATUS3) */
	ret = sc8527_read_byte(chip, SC8527_REG_0B, &reg0b_val);
	if (ret < 0) {
		chg_err("Failed to read register 0x0B: %d\n", ret);
		return ret;
	}

	/* Read register 0x0C (FLAG1) */
	ret = sc8527_read_byte(chip, SC8527_REG_0C, &reg0c_val);
	if (ret < 0) {
		chg_err("Failed to read register 0x0C: %d\n", ret);
		return ret;
	}

	/* Read register 0x0D (FLAG2) */
	ret = sc8527_read_byte(chip, SC8527_REG_0D, &reg0d_val);
	if (ret < 0) {
		chg_err("Failed to read register 0x0D: %d\n", ret);
		return ret;
	}

	/* Read register 0x0E (FLAG3) */
	ret = sc8527_read_byte(chip, SC8527_REG_0E, &reg0e_val);
	if (ret < 0) {
		chg_err("Failed to read register 0x0E: %d\n", ret);
		return ret;
	}

	/* Read register 0x1A (CONV_STAT1) */
	ret = sc8527_read_byte(chip, SC8527_REG_1A, &reg1a_val);
	if (ret < 0) {
		chg_err("Failed to read register 0x1A: %d\n", ret);
		return ret;
	}

	/* Read register 0x1B (CONV_STAT2) */
	ret = sc8527_read_byte(chip, SC8527_REG_1B, &reg1b_val);
	if (ret < 0) {
		chg_err("Failed to read register 0x1B: %d\n", ret);
		return ret;
	}

	/* Check register values against standard values */
	/* Register 09h (STATUS1): should be 0x00 */
	if (reg09_val != SC8527_STATUS1_STANDARD_VAL) {
		err_flag |= BIT(0);
		chg_err("Register 0x09 (STATUS1) abnormal: expected 0x%02x, got 0x%02x\n",
			SC8527_STATUS1_STANDARD_VAL, reg09_val);
	}

	/* Register 0Ah (STATUS2): low 3 bits (SC_EN_STAT, REG_EN_STAT, CP_SWITCHING_STAT) must be 111 */
	if ((reg0a_val & SC8527_STATUS2_LOW_3BITS_MASK) != SC8527_STATUS2_LOW_3BITS_STANDARD_VAL) {
		err_flag |= BIT(1);
		chg_err("Register 0x0A (STATUS2) abnormal: low 3 bits should be 0x%02x, got 0x%02x\n",
			SC8527_STATUS2_LOW_3BITS_STANDARD_VAL, reg0a_val);
	}

	/* Register 0Bh (STATUS3): any value, no check */

	/* Register 0Ch (FLAG1): should be 0x00 */
	if (reg0c_val != SC8527_FLAG1_STANDARD_VAL) {
		err_flag |= BIT(2);
		chg_err("Register 0x0C (FLAG1) abnormal: expected 0x%02x, got 0x%02x\n",
			SC8527_FLAG1_STANDARD_VAL, reg0c_val);
	}

	/* Register 0Dh (FLAG2): V2X_UVLO_FLG(bit7), V1X_SCP_FLG(bit6), CONV_OCP_FLG(bit2) should be 0 */
	if ((reg0d_val & SC8527_FLAG2_CHECK_MASK) != SC8527_FLAG2_CHECK_STANDARD_VAL) {
		err_flag |= BIT(3);
		chip->reg0d_val = reg0d_val;
		chg_err("Register 0x0D (FLAG2) abnormal: V2X_UVLO_FLG/V1X_SCP_FLG/CONV_OCP_FLG should be 0, got 0x%02x\n",
			reg0d_val);
	}

	/* Register 0Eh (FLAG3): should be 0x00 */
	if (reg0e_val != SC8527_FLAG3_STANDARD_VAL) {
		err_flag |= BIT(4);
		chg_err("Register 0x0E (FLAG3) abnormal: expected 0x%02x, got 0x%02x\n",
			SC8527_FLAG3_STANDARD_VAL, reg0e_val);
	}

	/* Register 1Ah (CONV_STAT1): any value, no check */

	/* Register 1Bh (CONV_STAT2): IIND_LIM_FLAG(bit0) should be 0 */
	if ((reg1b_val & SC8527_CONV_STAT2_IIND_LIM_FLAG_MASK) != 0x00) {
		err_flag |= BIT(5);
		chg_err("Register 0x1B (CONV_STAT2) abnormal: IIND_LIM_FLAG should be 0, got 0x%02x\n",
			reg1b_val);
	}

	chg_err("%s: reg09=0x%02x, reg0a=0x%02x, reg0b=0x%02x, reg0c=0x%02x, reg0d=0x%02x, reg0e=0x%02x, reg1a=0x%02x, reg1b=0x%02x, err_flag=0x%x\n",
		__func__, reg09_val, reg0a_val, reg0b_val, reg0c_val, reg0d_val, reg0e_val, reg1a_val, reg1b_val, err_flag);

	if (err_flag != 0) {
		sc8527_track_upload_cp_err_info_simple(chip, err_flag);
	}

	return err_flag;
}

static void sc8527_check_register_work(struct work_struct *work)
{
	struct sc8527_device *chip = container_of(work, struct sc8527_device, check_reg_work);

	if (!chip)
		return;

	sc8527_check_register_and_upload_track(chip);
}

static void sc8527_wired_subs_callback(struct mms_subscribe *subs,
				       enum mms_msg_type type, u32 id, bool sync)
{
	struct sc8527_device *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	if (!chip)
		return;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WIRED_ITEM_PRESENT:
			oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_PRESENT,
						&data, false);
			chip->wired_present = !!data.intval;
			chg_info("wired_present changed to %d\n", chip->wired_present);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void sc8527_subscribe_wired_topic(struct oplus_mms *topic, void *prv_data)
{
	struct sc8527_device *chip = prv_data;
	union mms_msg_data data = { 0 };

	if (!chip)
		return;

	chip->wired_topic = topic;
	chip->wired_subs = oplus_mms_subscribe(chip->wired_topic, chip,
					       sc8527_wired_subs_callback, "sc8527");
	if (IS_ERR_OR_NULL(chip->wired_subs)) {
		chg_err("subscribe wired topic error, rc=%ld\n",
			PTR_ERR(chip->wired_subs));
		return;
	}

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_PRESENT, &data, true);
	chip->wired_present = !!data.intval;
	chg_info("initial wired_present = %d\n", chip->wired_present);
}

static bool is_work_mode_votable_available(struct sc8527_device *chip)
{
	if (!chip->work_mode_votable)
		chip->work_mode_votable = find_votable("BOOST_WORK_MODE");
	return !!chip->work_mode_votable;
}

static void sc8527_power_on_mode_switch_work(struct work_struct *work)
{
	struct sc8527_device *chip = container_of(work, struct sc8527_device, power_on_mode_switch_work);
	bool v1x_scp_flg = false;
	bool conv_ocp_flg = false;
	bool has_ocp_event = false;
	int ret = 0;

	if (!chip || !chip->boost_ic) {
		chg_err("chip or boost_ic is NULL\n");
		return;
	}

	chg_info("start power on mode switch work\n");

	/* Step 1: check reg0d_val */
	v1x_scp_flg = !!(chip->reg0d_val & SC8527_FLAG2_V1X_SCP_FLG);
	conv_ocp_flg = !!(chip->reg0d_val & SC8527_FLAG2_CONV_OCP_FLG);
	has_ocp_event = v1x_scp_flg || conv_ocp_flg;
	if (has_ocp_event) {
		chg_err("Overcurrent event detected: V1X_SCP_FLG=%d, CONV_OCP_FLG=%d, reg0d=0x%02x\n",
				v1x_scp_flg, conv_ocp_flg, chip->reg0d_val);
		if (is_work_mode_votable_available(chip)) {
			vote(chip->work_mode_votable, BOOST_ERROR_VOTER, true, 1, false);
		}
		chg_info("Power on with Force CP Mode completed\n");
		return;
	}

	/* Step 2: REG_RST */
	ret = sc8527_update_bits(chip->client, SC8527_REG_06,
				  SC8527_REG_RESET_MASK,
				  SC8527_RESET_REG << SC8527_REG_RESET_SHIFT);
	if (ret < 0) {
		chg_err("failed to reset register, ret=%d\n", ret);
	}
	msleep(10);

	/* Step 3: init registers */
	sc8527_boost_ic_reg_set(chip);
	sc8527_update_bits(chip->client, SC8527_REG_1F, 0x01, 0x01);
	sc8527_update_bits(chip->client, SC8527_REG_1A, 0xc0, 0xc0);


	/* Step 4: try automode */
	chg_info("Attempting to enter Auto Mode\n");
	ret = oplus_chg_ic_func(chip->boost_ic, OPLUS_IC_FUNC_BOOST_SET_WORK_MODE, 0);

	chg_info("power on mode switch work completed\n");
}

static int sc8527_get_int_reg_info(struct oplus_voocphy_manager *chip,
				   char *dump_info, int len)
{
	int index = 0;

	if (!chip || !dump_info)
		return 0;

	index += snprintf(&(dump_info[index]), len - index,
			  "REG_09~REG_0E:[0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x]",
			  chip->int_column_pre[0], chip->int_column_pre[1], chip->int_column_pre[2],
			  chip->int_column_pre[3], chip->int_column_pre[4], chip->int_column_pre[5]);

	return index;
}

static int sc8527_get_cp_error_type(struct oplus_voocphy_manager *chip,
				    int *err_type)
{
	int i = 0;

	if (NULL == chip || NULL == err_type) {
		chg_err("chip ir err_type is NULL\n");
		return -EINVAL;
	}

	if (((bidirect_int_flag[0].mask & chip->int_column_pre[1]) == 0) &&
	    bidirect_int_flag[i].mark_except) {
		*err_type = i + 1;
		return 0;
	}

	for (i = 1; i < 7; i++) {
		if ((bidirect_int_flag[i].mask & chip->int_column_pre[3]) &&
		    bidirect_int_flag[i].mark_except) {
			*err_type = i + 1;
			return 0;
		}
	}

	for (i = 7; i < BIDIRECT_IRQ_EVNET_NUM; i++) {
		if ((bidirect_int_flag[i].mask & chip->int_column_pre[5]) &&
		    bidirect_int_flag[i].mark_except) {
			*err_type = i + 1;
			return 0;
		}
	}

	return 1; /* not found error type */
}

static int sc8527_track_upload_cp_err_info(struct oplus_voocphy_manager *chip,
					   int err_type)
{
	int index = 0;
	int curr_time;
	static int upload_count = 0;
	static int pre_upload_time = 0;
	char temp_str[REASON_LENGTH_MAX] = {0};
	struct oplus_mms *err_topic;
	struct mms_msg *msg = NULL;
	int rc = 0;
	char err_reason[ERR_LENGTH_MAX] = {0};
	char dump_info[DUMP_LENGTH_MAX] = {0};

	if (NULL == chip) {
		chg_err("chip is NULL");
		return -EINVAL;
	}

	if (err_type <= TRACK_BIDIRECT_CP_ERR_DEFAULT) {
		chg_err("err_type is invalid");
		return -EINVAL;
	}

	err_topic = oplus_mms_get_by_name("error");
	if (!err_topic) {
		chg_err("error topic not found\n");
		return -EINVAL;
	}

	curr_time = sc8527_track_get_local_time_s();
	if (curr_time - pre_upload_time > TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD)
		upload_count = 0;

	chg_info("err_type = %d\n", err_type);
	if (upload_count > TRACK_UPLOAD_COUNT_MAX) {
		chg_info("cp_err_uploading upload_count = %d > max %d, should return\n",
			 upload_count, TRACK_UPLOAD_COUNT_MAX);
		return 0;
	}

	upload_count++;
	pre_upload_time = sc8527_track_get_local_time_s();

	index += snprintf(&(temp_str[index]), REASON_LENGTH_MAX - index, "$$device_id@@%s", "sc8527");
	index += snprintf(&(temp_str[index]), REASON_LENGTH_MAX - index, "$$err_scene@@%s",
			  OPLUS_CHG_TRACK_SCENE_BIDIRECT_CP_ERR);

	oplus_chg_track_get_bidirect_cp_err_reason(err_type, err_reason, sizeof(err_reason));
	index += snprintf(&(temp_str[index]), REASON_LENGTH_MAX - index,
			  "$$err_reason@@%s", err_reason);

	sc8527_get_int_reg_info(chip, dump_info, sizeof(dump_info));
	index += snprintf(&(temp_str[index]), REASON_LENGTH_MAX - index,
			  "$$reg_info@@%s", dump_info);

	msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				      ERR_ITEM_BIDIRECT_CP_INFO, temp_str);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -EINVAL;
	}
	rc = oplus_mms_publish_msg_sync(err_topic, msg);
	if (rc < 0) {
		chg_err("publish msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return 0;
}

static ssize_t sc8527_show_registers(struct device *dev,
								     struct device_attribute *attr, char *buf)
{
	struct oplus_voocphy_manager *chip = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[700];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "sc8527");
	for (addr = 0x0; addr <= 0x50; addr++) {
		ret = sc8527_read_byte(chip->priv_data, addr, &val);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,
			               "Reg[%.2X] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t sc8527_store_register(struct device *dev,
								     struct device_attribute *attr, const char *buf, size_t count)
{
	struct oplus_voocphy_manager *chip = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	chg_err("write 0x%02X : 0x%02X\n", reg, val);
	if (ret == 2 && reg <= 0x50)
		sc8527_write_byte(chip->priv_data, (unsigned char)reg, (unsigned char)val);
	sc8527_check_register_and_upload_track(chip->priv_data);

	return count;
}


static DEVICE_ATTR(registers, 0660, sc8527_show_registers, sc8527_store_register);

static void sc8527_create_device_node(struct device *dev)
{
	device_create_file(dev, &dev_attr_registers);
}


static struct of_device_id sc8527_charger_match_table[] = {
	{
		.compatible = "sc,sc8527-master",
	},
	{},
};

static void register_voocphy_devinfo(void)
{
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	int ret = 0;
	char *version;
	char *manufacture;

	version = "sc8527";
	manufacture = "MP";

	ret = register_device_proc("voocphy", version, manufacture);
	if (ret)
		chg_err("register_voocphy_devinfo fail\n");
#endif
}

static struct oplus_voocphy_operations oplus_sc8527_ops = {
	.hardware_init		= sc8527_hardware_init,
	.hw_setting		= sc8527_hw_setting,
	.init_vooc		= sc8527_init_vooc,
	.set_predata		= sc8527_set_predata,
	.set_txbuff		= sc8527_set_txbuff,
	.get_adapter_info	= sc8527_get_adapter_info,
	.update_data		= sc8527_update_data,
	.get_chg_enable		= sc8527_get_chg_enable,
	.set_chg_enable		= sc8527_set_chg_enable,
	.reset_voocphy		= sc8527_reset_voocphy,
	.reactive_voocphy	= sc8527_reactive_voocphy,
	.send_handshake		= sc8527_send_handshake,
	.get_cp_vbat		= sc8527_get_cp_vbat,
	.get_int_value		= sc8527_get_int_value,
	.get_adc_enable		= sc8527_get_adc_enable,
	.set_adc_enable		= sc8527_set_adc_enable,
	.get_ichg		= sc8527_get_cp_ichg,
	.set_pd_svooc_config	= sc8527_set_pd_svooc_config,
	.get_pd_svooc_config	= sc8527_get_pd_svooc_config,
	.get_vbus_status	= sc8527_get_vbus_status,
	.set_chg_auto_mode	= sc8527_set_chg_auto_mode,
	.get_voocphy_enable	= sc8527_get_voocphy_enable,
	.dump_voocphy_reg	= sc8527_dump_reg_in_err_issue,
	.check_cp_int_happened	= sc8527_check_cp_int_happened,
	.upload_cp_error	= sc8527_track_upload_cp_err_info,
	.get_cp_error_type	= sc8527_get_cp_error_type,
};

static int sc8527_retrieve_reg_flags(struct sc8527_device *chip)
{
	unsigned int err_flag = 0;
	int rc = 0;
	u8 flag_buf[SC8527_FLAG_NUM] = { 0 };

	rc = sc8527_read_data(chip, SC8527_ADDR_GENERAL_INT_FLAG1, flag_buf,
			       SC8527_FLAG_NUM);
	if (rc < 0) {
		chg_err("failed to read flag register\n");
		return -EBUSY;
	}
	memcpy(chip->ufcs_reg_dump, flag_buf, SC8527_FLAG_NUM);

	if (flag_buf[0] & SC8527_FLAG_ACK_RECEIVE_TIMEOUT)
		err_flag |= BIT(UFCS_RECV_ERR_ACK_TIMEOUT);
	if (flag_buf[0] & SC8527_FLAG_MSG_TRANS_FAIL)
		err_flag |= BIT(UFCS_RECV_ERR_TRANS_FAIL);
	if (flag_buf[0] & SC8527_FLAG_RX_OVERFLOW)
		err_flag |= BIT(UFCS_COMM_ERR_RX_OVERFLOW);
	if (flag_buf[0] & SC8527_FLAG_DATA_READY)
		err_flag |= BIT(UFCS_RECV_ERR_DATA_READY);
	if (flag_buf[0] & SC8527_FLAG_SENT_PACKET_COMPLETE)
		err_flag |= BIT(UFCS_RECV_ERR_SENT_CMP);

	if (flag_buf[1] & SC8527_FLAG_HARD_RESET)
		err_flag |= BIT(UFCS_HW_ERR_HARD_RESET);
	if (flag_buf[1] & SC8527_FLAG_CRC_ERROR)
		err_flag |= BIT(UFCS_COMM_ERR_CRC_ERR);
	if (flag_buf[1] & SC8527_FLAG_STOP_ERROR)
		err_flag |= BIT(UFCS_COMM_ERR_STOP_ERR);
	if (flag_buf[1] & SC8527_FLAG_START_FAIL)
		err_flag |= BIT(UFCS_COMM_ERR_START_FAIL);
	if (flag_buf[1] & SC8527_FLAG_LENGTH_ERROR)
		err_flag |= BIT(UFCS_COMM_ERR_RX_LEN_ERR);
	if (flag_buf[1] & SC8527_FLAG_DATA_BYTE_TIMEOUT)
		err_flag |= BIT(UFCS_COMM_ERR_BYTE_TIMEOUT);
	if (flag_buf[1] & SC8527_FLAG_TRAINING_BYTE_ERROR)
		err_flag |= BIT(UFCS_COMM_ERR_TRAINING_ERR);
	if (flag_buf[1] & SC8527_FLAG_BAUD_RATE_ERROR)
		err_flag |= BIT(UFCS_COMM_ERR_BAUD_RATE_ERR);

	if (flag_buf[2] & SC8527_FLAG_BAUD_RATE_CHANGE)
		err_flag |= BIT(UFCS_COMM_ERR_BAUD_RATE_CHANGE);
	if (flag_buf[2] & SC8527_FLAG_BUS_CONFLICT)
		err_flag |= BIT(UFCS_COMM_ERR_BUS_CONFLICT);
	chip->ufcs->err_flag_save = err_flag;

	if (chip->ufcs->handshake_state == UFCS_HS_WAIT) {
		if ((flag_buf[0] & SC8527_FLAG_HANDSHAKE_SUCCESS) &&
		    !(flag_buf[0] & SC8527_FLAG_HANDSHAKE_FAIL)) {
			chip->ufcs->handshake_state = UFCS_HS_SUCCESS;
		 } else if (flag_buf[0] & SC8527_FLAG_HANDSHAKE_FAIL) {
			chip->ufcs->handshake_state = UFCS_HS_FAIL;
		 }
	}
	chg_info("[0x%x, 0x%x, 0x%x], err_flag=0x%x\n", flag_buf[0], flag_buf[1], flag_buf[2],
		 err_flag);

	return ufcs_set_error_flag(chip->ufcs, err_flag);
}

static int sc8527_ufcs_init(struct ufcs_dev *ufcs)
{
	return 0;
}

static int sc8527_ufcs_write_msg(struct ufcs_dev *ufcs, unsigned char *buf, int len)
{
	struct sc8527_device *chip = ufcs->drv_data;
	int rc;

	rc = sc8527_write_byte(chip, SC8527_ADDR_TX_LENGTH, len);
	if (rc < 0) {
		chg_err("write tx buf len error, rc=%d\n", rc);
		return rc;
	}
	rc = sc8527_write_data(chip, SC8527_ADDR_TX_BUFFER0, len, buf);
	if (rc < 0) {
		chg_err("write tx buf error, rc=%d\n", rc);
		return rc;
	}
	rc = sc8527_update_bits(chip->client, SC8527_ADDR_UFCS_CTRL1,
		SC8527_MASK_SND_CMP, SC8527_CMD_SND_CMP);
	if (rc < 0) {
		chg_err("write tx buf send cmd error, rc=%d\n", rc);
		return rc;
	}

	return rc;
}

static int sc8527_ufcs_read_msg(struct ufcs_dev *ufcs, unsigned char *buf, int len)
{
	struct sc8527_device *chip = ufcs->drv_data;
	u8 rx_buf_len;
	int rc;

	rc = sc8527_read_byte(chip, SC8527_ADDR_RX_LENGTH, &rx_buf_len);
	if (rc < 0) {
		chg_err("can't read rx buf len, rc=%d\n", rc);
		return rc;
	}
	if (rx_buf_len > len) {
		chg_err("rx_buf_len = %d, limit to %d\n", rx_buf_len, len);
		rx_buf_len = len;
	}

	rc = sc8527_read_data(chip, SC8527_ADDR_RX_BUFFER0, buf, rx_buf_len);
	if (rc < 0) {
		chg_err("can't read rx buf, rc=%d\n", rc);
		return rc;
	}

	return (int)rx_buf_len;
}

static int sc8527_ufcs_handshake(struct ufcs_dev *ufcs)
{
	struct sc8527_device *chip = ufcs->drv_data;
	int rc;

	chg_info("ufcs handshake\n");
	rc = sc8527_update_bits(chip->client, SC8527_ADDR_UFCS_CTRL1,
				    SC8527_MASK_EN_HANDSHAKE,
				    SC8527_CMD_EN_HANDSHAKE);
	if (rc < 0)
		chg_err("send handshake error, rc=%d\n", rc);

	return rc;
}

static int sc8527_ufcs_source_hard_reset(struct ufcs_dev *ufcs)
{
	struct sc8527_device *chip = ufcs->drv_data;
	int rc;
	int retry_count = 0;

retry:
	retry_count++;
	if (retry_count > UFCS_HARDRESET_RETRY_CNTS) {
		chg_err("send hard reset, retry count over!\n");
		return -EBUSY;
	}

	rc = sc8527_update_bits(chip->client, SC8527_ADDR_UFCS_CTRL1,
				SEND_SOURCE_HARDRESET,
				SEND_SOURCE_HARDRESET);
	if (rc < 0) {
		chg_err("I2c send handshake error\n");
		goto retry;
	}

	msleep(100);
	return 0;
}

static int sc8527_ufcs_cable_hard_reset(struct ufcs_dev *ufcs)
{
	return 0;
}

static int sc8527_ufcs_set_baud_rate(struct ufcs_dev *ufcs, enum ufcs_baud_rate baud)
{
	struct sc8527_device *chip = ufcs->drv_data;
	int rc;

	rc = sc8527_update_bits(chip->client, SC8527_ADDR_UFCS_CTRL1,
				FLAG_BAUD_RATE_VALUE,
				(baud << FLAG_BAUD_NUM_SHIFT));
	if (rc < 0)
		chg_err("set baud rate error, rc=%d\n", rc);

	return rc;
}

static int sc8527_ufcs_enable(struct ufcs_dev *ufcs)
{
	struct sc8527_device *chip = ufcs->drv_data;
	u8 addr_buf[SC8527_ENABLE_REG_NUM] = { SC8527_ADDR_DPDM_CTRL,
						SC8527_ADDR_UFCS_CTRL1,
						SC8527_ADDR_UFCS_CTRL2,
						SC8527_ADDR_UFCS_INT_MASK1,
						SC8527_ADDR_UFCS_INT_MASK2 };
	u8 cmd_buf[SC8527_ENABLE_REG_NUM] = {
		SC8527_CMD_DPDM_EN,	  SC8527_CMD_EN_CHIP,
		SC8527_CMD_CLR_TX_RX,    SC8527_CMD_MASK_ACK_TIMEOUT,
		SC8527_MASK_TRANING_BYTE_ERROR
	};
	int i;
	int rc;

	chip->rested = true;
	sc8527_reg_reset(chip->client, true);
	msleep(10);
	sc8527_init_device(chip);
	for (i = 0; i < SC8527_ENABLE_REG_NUM; i++) {
		rc = sc8527_write_byte(chip, addr_buf[i], cmd_buf[i]);
		if (rc < 0) {
			chg_err("write i2c failed!\n");
			return rc;
		}
	}
	chip->ufcs_enable = true;
	sc8527_write_byte(chip, SC8527_REG_CC, 0x00);/* hardreset signal to 1900us */

/*	rc = sc8527_write_byte(chip, SC8527_REG_09, SC8527_WATCHDOG_5S);
	if (rc < 0) {
		chg_err("failed to set sc8527_ufcs_enable (%d)\n", rc);
		return rc;
	}
*/
	return 0;
}

static int sc8527_ufcs_disable(struct ufcs_dev *ufcs)
{
	struct sc8527_device *chip = ufcs->drv_data;
	int rc;

	chip->ufcs_enable = false;
	chip->rested = false;
	rc = sc8527_write_byte(chip, SC8527_ADDR_UFCS_CTRL1,
				SC8527_CMD_DIS_CHIP);
	if (rc < 0) {
		chg_err("write i2c failed\n");
		return rc;
	}

/*	rc = sc8527_write_byte(chip, SC8527_REG_09, SC8527_WATCHDOG_DIS);
	if (rc < 0) {
		chg_err("failed to set sc8527_ufcs_disable (%d)\n", rc);
		return rc;
	}
*/
	return 0;
}
/*
static int sc8527_get_wdt_reg_by_time(unsigned int time_ms, u8 *reg)
{
	if (time_ms == 0) {
		*reg = SC8527_WATCHDOG_DIS;
	} else if (time_ms <= 200) {
		*reg = BIT(0);
	} else if (time_ms <= 500) {
		*reg = BIT(1);
	} else if (time_ms <= 1000) {
		*reg = BIT(1) | BIT(0);
	} else if (time_ms <= 5000) {
		*reg = BIT(2);
	} else if (time_ms <= 30000) {
		*reg = BIT(2) | BIT(0);
	} else {
		chg_err("sc8527 watchdog not support %dms(>30s)\n", time_ms);
		return -EINVAL;
	}

	return 0;
}
*/
static int sc8527_ufcs_cp_watchdog_config(struct ufcs_dev *ufcs, unsigned int time_ms)
{
/*	struct sc8527_device *chip = ufcs->drv_data;
	u8 en = 0;
	int rc = 0;

	rc = sc8527_get_wdt_reg_by_time(time_ms, &en);
	if (rc < 0)
		return rc;

	if (en == SC8527_WATCHDOG_DIS) {
		rc = sc8527_update_bits(chip->client, SC8527_REG_09,
		                         SC8527_WATCHDOG_MASK, SC8527_WATCHDOG_DIS);
		chg_info("watchdog_config disable (%d) ok!\n", en);
	} else {
		rc = sc8527_update_bits(chip->client, SC8527_REG_09,
		                         SC8527_WATCHDOG_MASK, en);
		chg_info("watchdog_config set (%d) ok!\n", en);
	}
	if (rc < 0) {
		chg_err("failed to cp_watchdog_config(%d)\n", rc);
		return rc;
	}
*/
	return 0;
}

#define SC8527_DP_20K_PD_EN_MASK BIT(4)
#define SC8527_DM_20K_PD_EN_MASK BIT(5)
static int sc8527_cp_reset_dpdm(struct ufcs_dev *ufcs)
{
	struct sc8527_device *chip = ufcs->drv_data;
	int rc = 0;
	int ufcs_disable_rc = 0;
	static bool first = true;
	u8 value = 0;
	bool ufcs_enable_success = false;

	if (!first && !oplus_chg_get_fcs_support_flags())
		return 0;
	first = false;

	if (chip == NULL) {
		chg_err("sc8527 chip is NULL\n");
		return -ENODEV;
	}

	rc = sc8527_read_byte(chip, SC8527_ADDR_UFCS_CTRL1, &value);
	if (rc < 0) {
		chg_err("[%s] read SC8527_ADDR_UFCS_CTRL1 error, rc=%d\n",
			chip->dev->of_node->name, rc);
		goto reset_dpdm_err;
	}

	if (!(value & SC8527_CMD_EN_CHIP)) {
		rc = sc8527_ufcs_enable(chip->ufcs);
		if (rc < 0) {
			chg_err("sc8527_ufcs_enable failed, rc=%d\n", rc);
			goto reset_dpdm_err;
		}
		ufcs_enable_success = true;
	}

	rc = sc8527_read_byte(chip, SC8527_REG_20, &value);
	if (rc < 0) {
		chg_err("[%s] read SC8527_REG_20 error, rc=%d\n", chip->dev->of_node->name, rc);
		goto reset_dpdm_err;
	}

	rc = sc8527_write_byte(chip, SC8527_REG_20,
		value | (SC8527_DP_20K_PD_EN_MASK | SC8527_DM_20K_PD_EN_MASK));
	if(rc < 0) {
		chg_err("dpdm pull down failed, rc=%d\n", rc);
		goto reset_dpdm_err;
	}
	chg_info("dpdm pull down\n");

	msleep(30);

	rc = sc8527_write_byte(chip, SC8527_REG_20,
		value & ~(SC8527_DP_20K_PD_EN_MASK | SC8527_DM_20K_PD_EN_MASK));
	if(rc < 0) {
		chg_err("dpdm pull up failed, rc=%d\n", rc);
		goto reset_dpdm_err;
	}
	chg_info("dpdm pull up\n");

reset_dpdm_err:
	if (ufcs_enable_success) {
		ufcs_disable_rc = sc8527_ufcs_disable(chip->ufcs);
		if (ufcs_disable_rc < 0)
			chg_err("sc8527_ufcs_disable failed, ufcs_disable_rc=%d\n",
				ufcs_disable_rc);
		if (rc >= 0)
			rc = ufcs_disable_rc;
	}

	return rc;
}

static struct ufcs_dev_ops ufcs_ops = {
	.init = sc8527_ufcs_init,
	.write_msg = sc8527_ufcs_write_msg,
	.read_msg = sc8527_ufcs_read_msg,
	.handshake = sc8527_ufcs_handshake,
	.source_hard_reset = sc8527_ufcs_source_hard_reset,
	.cable_hard_reset = sc8527_ufcs_cable_hard_reset,
	.set_baud_rate = sc8527_ufcs_set_baud_rate,
	.enable = sc8527_ufcs_enable,
	.disable = sc8527_ufcs_disable,
	.watchdog_config = sc8527_ufcs_cp_watchdog_config,
	.reset_dpdm = sc8527_cp_reset_dpdm,
};


static void sc8527_ufcs_event_handler(struct sc8527_device *chip)
{
	/* set awake */
	sc8527_retrieve_reg_flags(chip);
	ufcs_msg_handler(chip->ufcs);
}
static bool sc8527_is_volatile_reg(struct device *dev, unsigned int reg)
{
	return true;
}
static struct regmap_config sc8527_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = SC8527_MAX_REG,
	.cache_type = REGCACHE_RBTREE,
	.volatile_reg = sc8527_is_volatile_reg,
};

static struct ufcs_config sc8527_ufcs_config = {
	.check_crc = false,
	.reply_ack = false,
	.msg_resend = false,
	.handshake_hard_retry = true,
	.ic_vendor_id = SC8527_VENDOR_ID,
};

void sc8527_check_fault_info(struct sc8527_device *chip)
{
	int rc = 0;
	u8 buf[3] = { 0 };

	rc = sc8527_read_data(chip, 0x0c, buf, 3);
	if (rc < 0) {
		chg_err("failed to read fault register\n");
		return;
	}
	chg_info("%s 0x0c=0x%02x, 0x0d=0x%02x, 0x0e=0x%02x\n", chip->dev->of_node->name, buf[0], buf[1], buf[2]);
}

static irqreturn_t sc8527_interrupt_handler(int irq, void *dev_id)
{
	struct sc8527_device *chip = dev_id;
	struct oplus_voocphy_manager *voocphy = chip->voocphy;


	if (chip->use_ufcs_phy && chip->ufcs_enable) {
		sc8527_ufcs_event_handler(chip);
		return IRQ_HANDLED;
	} else if (chip->use_vooc_phy && chip->voocphy_enable) {
		return oplus_voocphy_interrupt_handler(voocphy);
	}

	schedule_work(&chip->check_reg_work);
	return IRQ_HANDLED;
}

static int sc8527_parse_dt(struct oplus_voocphy_manager *chip)
{
	struct device_node *node = chip->dev->of_node;

	chip->v2x_volt_full_open_low = of_property_read_bool(node, "oplus,v2x_volt_full_open_low");

	return 0;
}

static int sc8527_gpio_init(struct sc8527_device *chip)
{
	int rc;
	struct oplus_voocphy_manager *voocphy = chip->voocphy;
	struct device_node *node = voocphy->dev->of_node;

	voocphy->irq_gpio = of_get_named_gpio(node, "oplus,irq_gpio", 0);
	if (!gpio_is_valid(voocphy->irq_gpio)) {
		voocphy->irq_gpio = of_get_named_gpio(node, "oplus_spec,irq_gpio", 0);
		if (!gpio_is_valid(voocphy->irq_gpio)) {
			chg_err("irq_gpio not specified, rc=%d\n", voocphy->irq_gpio);
			return voocphy->irq_gpio;
		}
	}
	rc = gpio_request(voocphy->irq_gpio, "irq_gpio");
	if (rc) {
		chg_err("unable to request gpio[%d]\n", voocphy->irq_gpio);
		return rc;
	}
	chg_info("irq_gpio = %d\n", voocphy->irq_gpio);

	voocphy->irq = gpio_to_irq(voocphy->irq_gpio);
	voocphy->pinctrl = devm_pinctrl_get(voocphy->dev);
	if (IS_ERR_OR_NULL(voocphy->pinctrl)) {
		chg_err("get pinctrl fail\n");
		return -EINVAL;
	}

	voocphy->charging_inter_active =
	    pinctrl_lookup_state(voocphy->pinctrl, "charging_inter_active");
	if (IS_ERR_OR_NULL(voocphy->charging_inter_active)) {
		chg_err("failed to get the pinctrl state(%d)\n", __LINE__);
		return -EINVAL;
	}

	voocphy->charging_inter_sleep =
	    pinctrl_lookup_state(voocphy->pinctrl, "charging_inter_sleep");
	if (IS_ERR_OR_NULL(voocphy->charging_inter_sleep)) {
		chg_err("Failed to get the pinctrl state(%d)\n", __LINE__);
		return -EINVAL;
	}

	gpio_direction_input(voocphy->irq_gpio);
	pinctrl_select_state(voocphy->pinctrl, voocphy->charging_inter_active); /* no_PULL */
	rc = gpio_get_value(voocphy->irq_gpio);
	chg_info("irq_gpio = %d, irq_gpio_stat = %d\n", voocphy->irq_gpio, rc);

	chip->auto_mode_gpio = of_get_named_gpio(node, "oplus,auto_mode_gpio", 0);
	if (!gpio_is_valid(chip->auto_mode_gpio)) {
		chg_err("auto_mode_gpio[%d] unvalid\n", chip->auto_mode_gpio);
		return rc;
	}
	rc = gpio_request(chip->auto_mode_gpio, "auto_mode_gpio");
	if (rc) {
		chg_err("unable to request gpio[%d]\n", chip->auto_mode_gpio);
		return rc;
	}
	chg_info("auto_mode_gpio = %d\n", chip->auto_mode_gpio);

	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		chg_err("get reg_en pinctrl fail\n");
		return -EINVAL;
	}
	chip->auto_mode_enable =
	    pinctrl_lookup_state(chip->pinctrl, "auto_mode_enable");
	if (IS_ERR_OR_NULL(chip->auto_mode_enable)) {
		chg_err("failed to get the auto_mode_enable pinctrl state(%d)\n", __LINE__);
		return -EINVAL;
	}

	chip->auto_mode_disable =
	    pinctrl_lookup_state(chip->pinctrl, "auto_mode_disable");
	if (IS_ERR_OR_NULL(chip->auto_mode_disable)) {
		chg_err("failed to get the auto_mode_disable pinctrl state(%d)\n", __LINE__);
		return -EINVAL;
	}

	rc = pinctrl_select_state(chip->pinctrl, chip->auto_mode_enable);
	chg_info("set auto_mode enable %s, gpio_val:%d\n", rc < 0 ? "fail" : "success",
		gpio_get_value(chip->auto_mode_gpio));

	return 0;
}

static int sc8527_gpio_register(struct sc8527_device *chip)
{
	struct oplus_voocphy_manager *voocphy = chip->voocphy;
	struct irq_desc *desc;
	struct cpumask current_mask;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
	cpumask_var_t cpu_highcap_mask;
#endif
	int ret;

	ret = sc8527_gpio_init(chip);
	if (ret < 0) {
		chg_err("failed to gpio init(%d)\n", ret);
		return ret;
	}

	if (voocphy->irq) {
		ret = request_threaded_irq(voocphy->irq, NULL,
					   sc8527_interrupt_handler,
					   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					   "voocphy_irq", chip);
		if (ret < 0) {
			chg_err("request irq for irq=%d failed, ret =%d\n",
				voocphy->irq, ret);
			return ret;
		}
		enable_irq_wake(voocphy->irq);
		chg_debug("request irq ok\n");
	}

	desc = irq_to_desc(voocphy->irq);
	if (desc == NULL) {
		free_irq(voocphy->irq, voocphy);
		chg_err("desc null\n");
		return ret;
	}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
	update_highcap_mask(cpu_highcap_mask);
	cpumask_and(&current_mask, cpu_online_mask, cpu_highcap_mask);
#else
	cpumask_setall(&current_mask);
	cpumask_and(&current_mask, cpu_online_mask, &current_mask);
#endif
	ret = set_cpus_allowed_ptr(desc->action->thread, &current_mask);

	return 0;
}
static int sc8527_cp_init(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	ic_dev->online = true;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_ONLINE);

	return 0;
}

static int sc8527_cp_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	ic_dev->online = false;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_OFFLINE);

	return 0;
}

static int sc8527_cp_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	struct sc8527_device *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	sc8527_dump_reg_in_err_issue(chip->voocphy);
	return 0;
}

static int sc8527_cp_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	return 0;
}

static int sc8527_cp_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	return 0;
}

static int sc8527_cp_hw_init(struct oplus_chg_ic_dev *ic_dev)
{
	struct sc8527_device *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	if (chip->rested)
		return 0;

	sc8527_hardware_init(chip->voocphy);
	return 0;
}

static int sc8527_cp_set_work_mode(struct oplus_chg_ic_dev *ic_dev, enum oplus_cp_work_mode mode)
{
	struct sc8527_device *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);
	rc = sc8527_svooc_hw_setting(chip->voocphy);
	if (rc < 0)
		chg_err("set work mode to %d error\n", mode);

	return rc;
}

static int sc8527_cp_get_work_mode(struct oplus_chg_ic_dev *ic_dev, enum oplus_cp_work_mode *mode)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	*mode = CP_WORK_MODE_BYPASS;

	return 0;
}

static int sc8527_cp_check_work_mode_support(struct oplus_chg_ic_dev *ic_dev, enum oplus_cp_work_mode mode)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	return sc8527_check_work_mode_support(mode);
}

static int sc8527_cp_set_iin(struct oplus_chg_ic_dev *ic_dev, int iin)
{
	return 0;
}

static int sc8527_cp_get_vin(struct oplus_chg_ic_dev *ic_dev, int *vin)
{
	struct sc8527_device *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	rc = sc8527_cp_vbus(chip->voocphy);
	if (rc < 0) {
		chg_err("can't get cp vin, rc=%d\n", rc);
		return rc;
	}
	*vin = rc;

	return 0;
}

static int sc8527_cp_get_vout(struct oplus_chg_ic_dev *ic_dev, int *vout)
{
	struct sc8527_device *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	rc = sc8527_get_cp_vbat(chip->voocphy);
	if (rc < 0) {
		chg_err("can't get cp vout, rc=%d\n", rc);
		return rc;
	}
	*vout = rc;

	return 0;
}

static int sc8527_cp_get_iout(struct oplus_chg_ic_dev *ic_dev, int *iout)
{
	struct sc8527_device *chip;
	int iin;
	bool working;
	enum oplus_cp_work_mode work_mode;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	/*
	 * There is an exception in the iout adc of sc8537a, which is obtained
	 * indirectly through iin
	 */
	rc = oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_CP_GET_WORK_STATUS, &working);
	if (rc < 0)
		return rc;
	if (!working) {
		*iout = 0;
		return 0;
	}
	rc = oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_CP_GET_IIN, &iin);
	if (rc < 0)
		return rc;
	rc = oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_CP_GET_WORK_MODE, &work_mode);
	if (rc < 0)
		return rc;
	switch (work_mode) {
	case CP_WORK_MODE_BYPASS:
		*iout = iin;
		break;
	case CP_WORK_MODE_2_TO_1:
		*iout = iin * 2;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sc8527_cp_get_vac(struct oplus_chg_ic_dev *ic_dev, int *vac)
{
	struct sc8527_device *chip;
	/* u8 data_block[2] = { 0 }; */
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (!chip->vac_support)
		return -ENOTSUPP;

	/* rc = i2c_smbus_read_i2c_block_data(chip->client, SC8527_REG_17, 2, data_block); */
	if (rc < 0) {
		/* sc8527_i2c_error(chip->voocphy, true); */
		chg_err("sc8527 read vac error, rc=%d\n", rc);
		return rc;
	} else {
		/* sc8527_i2c_error(chip->voocphy, false); */
	}

	/* *vac = (((data_block[0] & SC8527_VAC_POL_H_MASK) << 8) | data_block[1]) * SC8527_VAC_ADC_LSB; */

	return 0;
}

static int sc8527_cp_set_work_start(struct oplus_chg_ic_dev *ic_dev, bool start)
{
	struct sc8527_device *chip;
	int rc = 0;
	u8 data = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (NULL == chip || NULL == chip->voocphy)
		return -ENODEV;

	sc8527_read_byte(chip, SC8527_REG_02, &data);
	chg_info("%s work %s, data = 0x%x\n", chip->dev->of_node->name, start ? "start" : "stop", data);

	if (start && data == DISENABLE_MOS) {
		rc = sc8527_set_chg_enable(chip->voocphy, start);
		sc8527_write_byte(chip, SC8527_REG_04, 0x36); /* WD:1000ms */
	} else if (!start) {
		rc = sc8527_set_chg_enable(chip->voocphy, start);
		sc8527_write_byte(chip, SC8527_REG_04, 0x06); /* dsiable wdt */
	}

	if (rc < 0)
		return rc;

	oplus_imp_node_set_active(chip->input_imp_node, start);
	/*oplus_imp_node_set_active(chip->output_imp_node, start); */

	return 0;
}

static int sc8527_cp_get_work_status(struct oplus_chg_ic_dev *ic_dev, bool *start)
{
	struct sc8527_device *chip;
	u8 data;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	rc = sc8527_get_chg_enable(chip->voocphy, &data);
	if (rc < 0) {
		chg_err("read SC8527_chg_enable error, rc=%d\n", rc);
		return rc;
	}

	*start = data & 1;

	return 0;
}

static void *sc8527_cp_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, sc8527_cp_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, sc8527_cp_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP, sc8527_cp_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST, sc8527_cp_smt_test);
		break;
	case OPLUS_IC_FUNC_CP_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_ENABLE, sc8527_cp_enable);
		break;
	case OPLUS_IC_FUNC_CP_HW_INTI:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_HW_INTI, sc8527_cp_hw_init);
		break;
	case OPLUS_IC_FUNC_CP_SET_WORK_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_WORK_MODE, sc8527_cp_set_work_mode);
		break;
	case OPLUS_IC_FUNC_CP_GET_WORK_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_WORK_MODE, sc8527_cp_get_work_mode);
		break;
	case OPLUS_IC_FUNC_CP_CHECK_WORK_MODE_SUPPORT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_CHECK_WORK_MODE_SUPPORT,
			sc8527_cp_check_work_mode_support);
		break;
	case OPLUS_IC_FUNC_CP_SET_IIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_IIN, sc8527_cp_set_iin);
		break;
	case OPLUS_IC_FUNC_CP_GET_VIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_VIN, sc8527_cp_get_vin);
		break;
	case OPLUS_IC_FUNC_CP_GET_VOUT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_VOUT, sc8527_cp_get_vout);
		break;
	case OPLUS_IC_FUNC_CP_GET_IOUT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_IOUT, sc8527_cp_get_iout);
		break;
	case OPLUS_IC_FUNC_CP_GET_VAC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_VAC, sc8527_cp_get_vac);
		break;
	case OPLUS_IC_FUNC_CP_SET_WORK_START:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_WORK_START, sc8527_cp_set_work_start);
		break;
	case OPLUS_IC_FUNC_CP_GET_WORK_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_WORK_STATUS, sc8527_cp_get_work_status);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq sc8527_cp_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
};

static int sc8527_boost_init(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	ic_dev->online = true;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_ONLINE);
	return 0;
}

static int sc8527_boost_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	ic_dev->online = false;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_OFFLINE);
	return 0;
}

static int sc8527_boost_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	return 0;
}

#define AUTO_MODE_RETRY_MAX	3
#define AUTO_MODE_DELAY_MS	2

static int sc8527_set_work_mode(struct oplus_chg_ic_dev *dev, int mode)
{
	struct sc8527_device *chip;
	struct oplus_voocphy_manager *voocphy;
	int rc = 0;
	uint8_t reg0a_val = 0;
	uint8_t reg1a_val = 0;
	uint8_t conv_mode_stat = 0;
	int retry_count = 0;
	bool auto_mode_ok = false;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(dev);

	if (IS_ERR_OR_NULL(chip->pinctrl) ||
	    IS_ERR_OR_NULL(chip->auto_mode_enable) ||
	    IS_ERR_OR_NULL(chip->auto_mode_disable)) {
		chg_err("auto_mode pinctrl error\n");
		return -ENODEV;
	}

	voocphy = chip->voocphy;
	if (!voocphy) {
		chg_err("voocphy is NULL\n");
		return -ENODEV;
	}

	if (mode == 1) {
		rc = pinctrl_select_state(chip->pinctrl, chip->auto_mode_disable);
		chg_info("set force cp mode %s, gpio_val:%d\n", rc < 0 ? "fail" : "success",
			 gpio_get_value(chip->auto_mode_gpio));
		return rc;
	}

	chg_info("Attempting to enter Auto Mode\n");

	for (retry_count = 0; retry_count < AUTO_MODE_RETRY_MAX; retry_count++) {
		if (retry_count > 0) {
			chg_info("Retry %d: Pull REG_EN low then high to re-enter Auto Mode\n", retry_count);
			rc = pinctrl_select_state(chip->pinctrl, chip->auto_mode_disable);
			if (rc < 0) {
				chg_err("failed to disable auto mode, ret=%d\n", rc);
			}
			msleep(10);
		}

		rc = pinctrl_select_state(chip->pinctrl, chip->auto_mode_enable);
		if (rc < 0) {
			chg_err("failed to enable auto mode pinctrl, ret=%d\n", rc);
			continue;
		}

		msleep(AUTO_MODE_DELAY_MS);

		rc = sc8527_read_byte(chip, SC8527_REG_0A, &reg0a_val);
		if (rc < 0) {
			chg_err("failed to read register 0x0A, ret=%d\n", rc);
			continue;
		}

		rc = sc8527_read_byte(chip, SC8527_REG_1A, &reg1a_val);
		if (rc < 0) {
			chg_err("failed to read register 0x1A, ret=%d\n", rc);
			continue;
		}

		conv_mode_stat = (reg1a_val & SC8527_CONV_MODE_STAT_MASK) >> SC8527_CONV_MODE_STAT_SHIFT;

		if ((reg0a_val & SC8527_STATUS2_REG_EN_STAT) &&
		    (conv_mode_stat != 0)) {
			auto_mode_ok = true;
			chg_info("Auto Mode is normal: REG_EN_STAT=1, CONV_MODE_STAT=0x%02x, gpio_val:%d\n",
				 conv_mode_stat, gpio_get_value(chip->auto_mode_gpio));
			break;
		} else {
			chg_err("Auto Mode abnormal: REG_EN_STAT=%d, CONV_MODE_STAT=0x%02x, reg0a=0x%02x, reg1a=0x%02x\n",
				!!(reg0a_val & SC8527_STATUS2_REG_EN_STAT), conv_mode_stat, reg0a_val, reg1a_val);
		}
	}

	if (!auto_mode_ok) {
		chg_err("Auto Mode entry failed after %d retries, use Force CP Mode\n", AUTO_MODE_RETRY_MAX);
		rc = pinctrl_select_state(chip->pinctrl, chip->auto_mode_disable);
		if (rc < 0) {
			chg_err("failed to set Force CP Mode, ret=%d\n", rc);
		}
		sc8527_check_register_and_upload_track(chip);
		chg_info("Power on with Force CP Mode completed\n");
		return -1;
	}

	return 0;
}

int sc8527_set_q2_ocp_enable(struct sc8527_device *chip, int enable)
{
	int bits_mask;
	int bits;
	if (enable) {
		bits = (1 << SC8527_Q2OCP_SHIFT);
	} else {
		bits = (0 << SC8527_Q2OCP_SHIFT);
	}
	bits_mask = SC8527_Q2OCP_MASK;

	chg_err("enable:%d, bits:%d, reg:%d\n", enable, bits, SC8527_REG_3A);

	return sc8527_update_bits(chip->client, SC8527_REG_3A, bits_mask, bits);
}

int sc8527_set_iind_limt(struct sc8527_device *chip, int curr_ma)
{
	int bits = volt_curr_to_bits(curr_ma, IIND_LIM_TH_OFFSET, STEP_1000MA);
	int bits_shift = bits << SC8527_IIND_LIM_SHIFT;
	int bits_mask = SC8527_IIND_LIM_MASK;

	chg_err("iind_limt:%d, bits:%d, bits_shift:%d\n", curr_ma, bits, bits_shift);

	return sc8527_update_bits(chip->client, SC8527_REG_1C, bits_mask, bits_shift);
}

int sc8527_set_clamp_mode(struct sc8527_device *chip, int enable)
{
	int bits_mask;
	int bits;
	if (enable) {
		bits = (SC8527_SNS_CLAMP_ENABLE << SC8527_SNS_CLAMP_SHIFT);
	} else {
		bits = (SC8527_SNS_CLAMP_DISABLE << SC8527_SNS_CLAMP_SHIFT);
	}
	bits_mask = SC8527_SNS_CLAMP_MASK;

	chg_err("enable:%d, bits:%d, reg:%d\n", enable, bits, SC8527_REG_38);

	return sc8527_update_bits(chip->client, SC8527_REG_38, bits_mask, bits);
}

int sc8527_set_mtk_pre_uv_mode(struct sc8527_device *chip, int voltage_mv)
{
	int bits = volt_curr_to_bits(voltage_mv, K_CHANGE_TH_OFFSET, STEP_200MV);
	int bits_shift = (bits << SC8527_K_CHANGE_SHIFT) | (SC8527_BCL_MTK_PRE_UV_MODE << SC8527_BCL_MODE_SHIFT);
	int bits_mask = SC8527_K_CHANGE_MASK | SC8527_BCL_MODE_MASK;

	chg_err("K_CHANGE:%d, bits:%d, bits_shift:%d\n", voltage_mv, bits, bits_shift);

	return sc8527_update_bits(chip->client, SC8527_REG_37, bits_mask, bits_shift);
}

static int sc8527_boost_set_bcl_rate(struct oplus_chg_ic_dev *ic_dev, int rate)
{
	struct sc8527_device *chip;
	int bits = 0;
	int bits_mask = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);
#ifdef CONFIG_OPLUS_CHARGER_MTK
	chg_err("MTK!force return!\n");
	return 0;
#endif
	if (rate <= SC8527_SNS_RATIO_500_THR) {
		bits = (SC8527_SNS_RATIO_550 << SC8527_SNS_RATIO_SHIFT) | (SC8527_BCL_CP_MODE << SC8527_BCL_MODE_SHIFT);
	} else if (rate <= SC8527_SNS_RATIO_550_THR) {
		bits = (SC8527_SNS_RATIO_550 << SC8527_SNS_RATIO_SHIFT) | (SC8527_BCL_AUTO_SNS_RATIO_MODE << SC8527_BCL_MODE_SHIFT);
	} else if (rate <= SC8527_SNS_RATIO_575_THR) {
		bits = (SC8527_SNS_RATIO_575 << SC8527_SNS_RATIO_SHIFT) | (SC8527_BCL_AUTO_SNS_RATIO_MODE << SC8527_BCL_MODE_SHIFT);
	} else if (rate <= SC8527_SNS_RATIO_600_THR) {
		bits = (SC8527_SNS_RATIO_600 << SC8527_SNS_RATIO_SHIFT) | (SC8527_BCL_AUTO_SNS_RATIO_MODE << SC8527_BCL_MODE_SHIFT);
	} else {
		bits = (SC8527_SNS_RATIO_625 << SC8527_SNS_RATIO_SHIFT) | (SC8527_BCL_AUTO_SNS_RATIO_MODE << SC8527_BCL_MODE_SHIFT);
	}
	/* set rate */
	bits_mask = SC8527_SNS_RATIO_MASK | SC8527_BCL_MODE_MASK;
	sc8527_update_bits(chip->client, SC8527_REG_37, bits_mask, bits);
	chg_err("rate:%d, bits:%d, reg:%d\n", rate, bits, SC8527_REG_37);

	if (rate <= SC8527_SNS_RATIO_500_THR) {
		sc8527_set_clamp_mode(chip, 1);
	} else {
		sc8527_set_clamp_mode(chip, 0);
	}
	return 0;
}

static void *sc8527_boost_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, sc8527_boost_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, sc8527_boost_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP, sc8527_boost_reg_dump);
		break;
	case OPLUS_IC_FUNC_BOOST_SET_CV:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BOOST_SET_CV, sc8527_set_cv_volt);
		break;
	case OPLUS_IC_FUNC_BOOST_SET_WORK_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BOOST_SET_WORK_MODE, sc8527_set_work_mode);
		break;
	case OPLUS_IC_FUNC_BOOST_SET_BCL_RATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BOOST_SET_BCL_RATE, sc8527_boost_set_bcl_rate);
		break;
	case OPLUS_IC_FUNC_BOOST_GET_IN_CV_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BOOST_GET_IN_CV_MODE, sc8527_get_in_cv_mode);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq sc8527_boost_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
};

static int oplus_sc8527_shipmode_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct sc8527_device *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);
	chip->ship_mode_en = en;
	chg_err("en:%d\n", chip->ship_mode_en);

	return 0;
}

static int oplus_chg_sc8527_init(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	ic_dev->online = true;
	return 0;
}

static int oplus_chg_sc8527_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	ic_dev->online = false;
	return 0;
}

static void *sc8527_buck_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, oplus_chg_sc8527_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, oplus_chg_sc8527_exit);
		break;
	case OPLUS_IC_FUNC_BUCK_SHIPMODE_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SHIPMODE_ENABLE, oplus_sc8527_shipmode_enable);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq sc8527_buck_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
};
static int sc8527_ic_register(struct sc8527_device *chip)
{
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	struct device_node *child;
	struct oplus_chg_ic_dev *ic_dev = NULL;
	struct oplus_chg_ic_cfg ic_cfg;
	int rc;

	for_each_child_of_node(chip->dev->of_node, child) {
		rc = of_property_read_u32(child, "oplus,ic_type", &ic_type);
		if (rc < 0)
			continue;
		rc = of_property_read_u32(child, "oplus,ic_index", &ic_index);
		if (rc < 0)
			continue;
		ic_cfg.name = child->name;
		ic_cfg.index = ic_index;
		ic_cfg.type = ic_type;
		ic_cfg.priv_data = chip;
		ic_cfg.of_node = child;
		switch (ic_type) {
		case OPLUS_CHG_IC_CP:
			/* (void)sc8527_init_imp_node(chip, child); */
			snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "cp-sc8527:%d", ic_index);
			snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
			ic_cfg.get_func = sc8527_cp_get_func;
			ic_cfg.virq_data = sc8527_cp_virq_table;
			ic_cfg.virq_num = ARRAY_SIZE(sc8527_cp_virq_table);
			break;
		case OPLUS_CHG_IC_VIRTUAL_BOOST:
			snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "boost-sc8527:%d", ic_index);
			snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
			ic_cfg.get_func = sc8527_boost_get_func;
			ic_cfg.virq_data = sc8527_boost_virq_table;
			ic_cfg.virq_num = ARRAY_SIZE(sc8527_boost_virq_table);
			break;
		case OPLUS_CHG_IC_BUCK:
			snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "buck-sc8527");
			snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
			ic_cfg.get_func = sc8527_buck_get_func;
			ic_cfg.virq_data = sc8527_buck_virq_table;
			ic_cfg.virq_num = ARRAY_SIZE(sc8527_buck_virq_table);
			break;
		default:
			chg_err("not support ic_type(=%d), OPLUS_CHG_IC_VIRTUAL_BOOST:%d\n", ic_type, OPLUS_CHG_IC_VIRTUAL_BOOST);
			continue;
		}
		ic_dev = devm_oplus_chg_ic_register(chip->dev, &ic_cfg);
		if (!ic_dev) {
			rc = -ENODEV;
			chg_err("register %s error\n", child->name);
			continue;
		}
		chg_info("register %s\n", child->name);

		switch (ic_dev->type) {
		case OPLUS_CHG_IC_CP:
			chip->cp_work_mode = CP_WORK_MODE_UNKNOWN;
			chip->cp_ic = ic_dev;
			break;
		case OPLUS_CHG_IC_VIRTUAL_BOOST:
			chip->boost_ic = ic_dev;
			break;
		case OPLUS_CHG_IC_BUCK:
			chip->buck_ic = ic_dev;
			break;
		default:
			chg_err("not support ic_type(=%d)\n", ic_dev->type);
			continue;
		}

		of_platform_populate(child, NULL, NULL, chip->dev);
	}

	return 0;
}

static bool sc8527_check_device_is_exist(struct i2c_client *client)
{
	u8 reg[2] = { 0 };
	int max_count = 5;
	int ret = -1;

	if (NULL == client) {
		chg_err("client is null");
		return false;
	}

        /* check if the ic is ok by read register */
	while (max_count--) {
		ret = sc8527_read_i2c_nonblock(client, 0x1c, 2, reg);
		if (ret < 0) {
			chg_err("count = %d read REG_1C 2 bytes failed(%d)\n", max_count, ret);
			msleep(10);
			continue;
		} else {
			break;
		}
	}

	if (ret >= 0) {
		chg_info(" CHIP_REV:0x%x, OTP_REV:0x%x", reg[0], reg[1]);
		return true;
	} else {
		chg_err("device maybe not exist, ret = %d!", ret);
		return false;
	}
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0))
static int sc8527_charger_probe(struct i2c_client *client)
#else
static int sc8527_charger_probe(struct i2c_client *client,
                                const struct i2c_device_id *id)
#endif
{
	int ret;
	struct sc8527_device *chip;
	struct oplus_voocphy_manager *voocphy;

	chg_info("start\n");
	if (!sc8527_check_device_is_exist(client)) {
		chg_err("sc8527 device is not exsit");
	}

	chip = devm_kzalloc(&client->dev, sizeof(struct sc8527_device), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "alloc sc8527 device buf error\n");
		return -ENOMEM;
	}

	voocphy = devm_kzalloc(&client->dev, sizeof(struct oplus_voocphy_manager), GFP_KERNEL);
	if (voocphy == NULL) {
		chg_err("alloc voocphy buf error\n");
		ret = -ENOMEM;
		goto chg_err;
	}

	chip->client = client;
	chip->dev = &client->dev;
	voocphy->client = client;
	voocphy->dev = &client->dev;
	voocphy->priv_data = chip;
	chip->voocphy = voocphy;
	mutex_init(&chip->i2c_rw_lock);
	mutex_init(&chip->chip_lock);
	INIT_WORK(&chip->check_reg_work, sc8527_check_register_work);
	INIT_WORK(&chip->power_on_mode_switch_work, sc8527_power_on_mode_switch_work);
	INIT_DELAYED_WORK(&chip->cp_uvp_enable_work, sc8527_cp_uvp_enable_work_func);
	i2c_set_clientdata(client, voocphy);

	sc8527_create_device_node(&(client->dev));
	voocphy->ops = &oplus_sc8527_ops;
	sc8527_dump_registers(voocphy);
	chip->use_vooc_phy = of_property_read_bool(chip->dev->of_node, "oplus,use_vooc_phy");
	chip->use_ufcs_phy = of_property_read_bool(chip->dev->of_node, "oplus,use_ufcs_phy");
	chg_err("use_ufcs_phy:%d,ufcs_enable:%d,use_vooc_phy:%d,voocphy_enable:%d\n",
	chip->use_ufcs_phy, chip->ufcs_enable, chip->use_vooc_phy, chip->voocphy_enable);
	sc8527_parse_dt(voocphy);

	chip->regmap = devm_regmap_init_i2c(client, &sc8527_regmap_config);
	if (chip->regmap == NULL) {
		chg_err("failed to devm_regmap_init_i2c!");
	}
	ret = oplus_register_voocphy(voocphy);
	if (ret < 0) {
		chg_err("failed to register voocphy, ret = %d", ret);
		goto reg_voocphy_err;
	}
	if (chip->use_ufcs_phy) {
		chip->ufcs = ufcs_device_register(chip->dev, &ufcs_ops, chip, &sc8527_ufcs_config);
		if (IS_ERR_OR_NULL(chip->ufcs)) {
			chg_err("ufcs device register error\n");
			ret = -ENODEV;
			goto reg_ufcs_err;
		}
	}

	ret = sc8527_gpio_register(chip);
	if (ret < 0) {
		chg_err("irq register error, rc=%d\n", ret);
		goto reg_irq_err;
	}

	ret = sc8527_ic_register(chip);
	if (ret < 0) {
		chg_err("cp ic register error\n");
		goto cp_reg_err;
	}

	chip->ufcs_enable = false;
	chip->voocphy_enable = false;
	chip->ship_mode_en = 0;
	sc8527_cp_init(chip->cp_ic);

	oplus_mms_wait_topic("wired", sc8527_subscribe_wired_topic, chip);
	if (sc8527_check_register_and_upload_track(chip) && !chip->wired_present)
		schedule_work(&chip->power_on_mode_switch_work);

	sc8527_dump_registers(voocphy);
	register_voocphy_devinfo();
	sc8527_set_cv_volt(chip->boost_ic, 3500);
	sc8527_set_v1x_th_cv2cp_l(chip, 300);
	sc8527_set_v1x_th_cv2cp_h(chip, 200);
	sc8527_set_v1x_th_cp2cv(chip, 200);
	//sc8527_set_q2_ocp_enable(chip, 1);
	sc8527_set_iind_limt(chip, IIND_LIM_DEFAULT_TH);
#ifdef CONFIG_OPLUS_CHARGER_MTK
	sc8527_set_mtk_pre_uv_mode(chip, K_CHANGE_DEFAULT_TH);
	sc8527_set_clamp_mode(chip, 0);
#else
	sc8527_boost_set_bcl_rate(chip->boost_ic, SC8527_SNS_RATIO_550_THR);
#endif
	sc8527_update_bits(chip->client, SC8527_REG_1F, 0x01, 0x01);
	sc8527_update_bits(chip->client, SC8527_REG_1A, 0xc0, 0xc0);

	chg_info("sc8527(%s) probe successfully\n", chip->dev->of_node && chip->dev->of_node->name ? chip->dev->of_node->name : "null");

	return 0;

cp_reg_err:
	free_irq(voocphy->irq, voocphy);
reg_irq_err:
	gpio_free(voocphy->irq_gpio);
reg_ufcs_err:
	if (chip->use_ufcs_phy)
		ufcs_device_unregister(chip->ufcs);
reg_voocphy_err:
	devm_kfree(&client->dev, voocphy);
chg_err:
	devm_kfree(&client->dev, chip);
	return ret;
}

static void sc8527_release_chip_resources(struct sc8527_device *chip)
{
	if (!chip)
		return;

	if (chip->cp_ic)
		sc8527_cp_exit(chip->cp_ic);

	if (chip->boost_ic)
		sc8527_boost_exit(chip->boost_ic);

	if (chip->buck_ic)
		oplus_chg_sc8527_exit(chip->buck_ic);

	if (chip->use_ufcs_phy && !IS_ERR_OR_NULL(chip->ufcs))
		ufcs_device_unregister(chip->ufcs);

	if (gpio_is_valid(chip->auto_mode_gpio))
		gpio_free(chip->auto_mode_gpio);
}

static void sc8527_release_irq_resources(struct oplus_voocphy_manager *voocphy)
{
	if (IS_ERR_OR_NULL(voocphy))
		return;

	if (voocphy->irq > 0) {
		disable_irq_wake(voocphy->irq);
		free_irq(voocphy->irq, voocphy);
	}

	if (gpio_is_valid(voocphy->irq_gpio))
		gpio_free(voocphy->irq_gpio);
}

static void sc8527_charger_remove(struct i2c_client *client)
{
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);
	struct sc8527_device *chip = NULL;

	if (IS_ERR_OR_NULL(voocphy))
		return;

	chip = voocphy->priv_data;
	chg_info("enter\n");

	cancel_delayed_work_sync(&chip->cp_uvp_enable_work);

	sc8527_release_chip_resources(chip);
	sc8527_release_irq_resources(voocphy);

	device_remove_file(&client->dev, &dev_attr_registers);
}

static void sc8527_charger_shutdown(struct i2c_client *client)
{
	struct oplus_voocphy_manager *voocphy_chip = i2c_get_clientdata(client);
	struct sc8527_device *chip = voocphy_chip->priv_data;
	int bits = 0;
	int bits_mask = 0;

	sc8527_update_bits(client, SC8527_REG_06, SC8527_REG_RESET_MASK,
			   SC8527_RESET_REG << SC8527_REG_RESET_SHIFT);
	msleep(10);
	/* disable v1x_scp */
	sc8527_write_byte(chip, SC8527_REG_01, 0x5e);
	/* REF_SKIP_R 40mv */
	sc8527_write_byte(chip, SC8527_REG_08, 0xA6);
	/* Masked Pulse_filtered, RX_Start,Tx_Done,bit soft intflag */
	sc8527_write_byte(chip, SC8527_REG_29, 0x05);
	if (chip->ship_mode_en != 0) {
		bits = (1 << SC8527_SHIPMODE_EN_ALLOW_SHIF);
		bits_mask = SC8527_SHIPMODE_EN_ALLOW_MASK;
		sc8527_update_bits(client, SC8527_REG_1F, bits_mask, bits);
		chg_err("en:0x%02X, reg:0x%02X, bits:0x%02X, bits_mask:0x%02X\n", chip->ship_mode_en, SC8527_REG_1F, bits, bits_mask);

		bits = (SC8527_SHIPMODE_EN << SC8527_SHIPMODE_EN_SHIFT) | (1 << SC8527_SHIPMODE_EN_DLY_SHIFT);
		bits_mask = SC8527_SHIPMODE_EN_MASK | SC8527_SHIPMODE_EN_DLY_MASK;
		sc8527_update_bits(client, SC8527_REG_1E, bits_mask, bits);
		chg_err("en:0x%02X, reg:0x%02X, bits:0x%02X, bits_mask:0x%02X\n", chip->ship_mode_en, SC8527_REG_1E, bits, bits_mask);
		msleep(1000);
	}
	return;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
static int oplus_sc8527_pm_resume(struct device *dev)
{
	struct oplus_voocphy_manager *chip = dev_get_drvdata(dev);
	struct sc8527_device *sc8527_chip;

	sc8527_chip = chip->priv_data;
	sc8527_set_v1x_th_cp2cv(sc8527_chip, 200);
	return 0;
}

static int oplus_sc8527_pm_suspend(struct device *dev)
{
	struct oplus_voocphy_manager *chip = dev_get_drvdata(dev);
	struct sc8527_device *sc8527_chip;

	sc8527_chip = chip->priv_data;
	sc8527_set_v1x_th_cp2cv(sc8527_chip, 100);
	return 0;
}

static const struct dev_pm_ops oplus_sc8527_pm_ops = {
	.resume		= oplus_sc8527_pm_resume,
	.suspend	= oplus_sc8527_pm_suspend,
};
#endif

static const struct i2c_device_id sc8527_charger_id[] = {
	{"sc8527-master", 0},
	{},
};

static struct i2c_driver sc8527_charger_driver = {
	.driver		= {
		.name	= "sc8527-charger",
		.owner	= THIS_MODULE,
		.of_match_table = sc8527_charger_match_table,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
		.pm 	= &oplus_sc8527_pm_ops,
#endif
	},
	.id_table	= sc8527_charger_id,

	.probe		= sc8527_charger_probe,
	.remove		= sc8527_charger_remove,
	.shutdown	= sc8527_charger_shutdown,
};

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
static int __init sc8527_subsys_init(void)
{
	int ret = 0;
	chg_debug(" init start\n");

	if (i2c_add_driver(&sc8527_charger_driver) != 0) {
		chg_err(" failed to register sc8527 i2c driver.\n");
	} else {
		chg_debug(" Success to register sc8527 i2c driver.\n");
	}

	return ret;
}

subsys_initcall(sc8527_subsys_init);
#else
static int sc8527_subsys_init(void)
{
	int ret = 0;
	chg_debug(" init start\n");

	if (i2c_add_driver(&sc8527_charger_driver) != 0) {
		chg_err(" failed to register sc8527 i2c driver.\n");
	} else {
		chg_debug(" Success to register sc8527 i2c driver.\n");
	}

	return ret;
}

static void sc8527_subsys_exit(void)
{
	i2c_del_driver(&sc8527_charger_driver);
}
oplus_chg_module_register(sc8527_subsys);
#endif /*LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)*/

MODULE_DESCRIPTION("SC SC8527 Charge Pump Driver");
MODULE_LICENSE("GPL v2");
