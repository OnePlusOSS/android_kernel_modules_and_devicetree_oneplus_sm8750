// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022-2023 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[SC8547D]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/list.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/err.h>
#include <linux/proc_fs.h>
#include <linux/pinctrl/consumer.h>

#include <trace/events/sched.h>
#include <linux/ktime.h>
#include <uapi/linux/sched/types.h>
#include <linux/pm_qos.h>
#include <linux/version.h>
#include <linux/time.h>
#include <linux/sched/clock.h>
#include <linux/rtc.h>
#include <linux/device.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/system/oplus_project.h>
#endif
#include <ufcs_class.h>

#include <oplus_chg_ic.h>
#include <oplus_chg_module.h>
#include <oplus_chg.h>
#include <oplus_mms.h>
#include <oplus_mms_gauge.h>
#include <oplus_impedance_check.h>
#include <oplus_chg_monitor.h>
#include <oplus_chg_voter.h>
#include "../voocphy/oplus_voocphy.h"

#include "../voocphy/phy/oplus_sc8547.h"
#include "oplus_hal_sc8547d.h"
#include <oplus_mms_wired.h>
#include <oplus_chg_vooc.h>

#define DEFUALT_VBUS_LOW 100
#define DEFUALT_VBUS_HIGH 200
#define I2C_ERR_NUM 10
#define MAIN_I2C_ERROR (1 << 0)
#define SLAVE_I2C_ERROR (1 << 1)
#define OPLUS_SC8547D_UCP_100MS 0x30
#define OPLUS_SC8547D_UCP_5MS 0x10
#define IRQ_EVENT_SC8547D_NUM 14

struct sc8547d_device {
	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;
	struct oplus_voocphy_manager *voocphy;
	struct oplus_voocphy_manager *voocphy_mg;
	struct ufcs_dev *ufcs;

	struct oplus_chg_ic_dev *cp_ic;
	struct oplus_impedance_node *input_imp_node;
	struct oplus_impedance_node *output_imp_node;

	struct oplus_mms *err_topic;
	struct mms_subscribe *err_subs;

	struct votable *disable_votable;

	struct oplus_mms *wired_topic;
	struct mms_subscribe *wired_subs;
	struct mutex i2c_rw_lock;
	struct mutex chip_lock;
	struct mutex adc_freeze_lock;
	atomic_t suspended;
	atomic_t i2c_err_count;
	struct wakeup_source *chip_ws;

	int ovp_reg;
	int ocp_reg;
	u8 vbus_ovp_reg;

	bool ufcs_enable;
	bool voocphy_enable;

	enum oplus_cp_work_mode cp_work_mode;

	bool rested;
	bool error_reported;
	bool use_ufcs_phy;
	bool use_vooc_phy;
	bool use_slave_cp;
	bool vac_support;
	bool ic_sc8547d;
	bool otg_connected;
	bool always_otg_en;
	bool work_start;
	bool chg_enabled;

	struct work_struct ufcs_regdump_work;
	struct work_struct cp_regdump_work;
	struct work_struct otg_enabled_work;
	struct work_struct ic_offline_work;
	struct delayed_work track_cp_switching_work;
	u8 ufcs_reg_dump[SC8547D_FLAG_NUM];
	int cp_reg_track[SC8547D_TRACK_NUM];
};

static enum oplus_cp_work_mode g_cp_support_work_mode[] = {
	CP_WORK_MODE_BYPASS,
	CP_WORK_MODE_2_TO_1,
};

static struct intinfo sc8547d_int_flag[IRQ_EVENT_SC8547D_NUM] = {
	{CP_SC8547D_TSD_FLAG_MASK, "ERR_CP_TSD", 4},
	{CP_SC8547D_SS_TIMEOUT_FLAG_MASK, "ERR_CP_SS_TIMEOUT", 12},
	{CP_SC8547D_PIN_DIAG_FALL_FLAG_MASK, "ERR_CP_PIN_DIAG_FALL", 1},
	{CP_SC8547D_WD_TIMEOUT_FLAG_MASK, "ERR_CP_WDT", 13},
	{CP_SC8547D_PMID2OUT_UVP_FLAG_MASK, "ERR_CP_PMID2OUT_UVP", 11},
	{CP_SC8547D_PMID2OUT_OVP_FLAG_MASK, "ERR_CP_PMID2OUT_OVP", 10},
	{CP_SC8547D_VOUT_OVP_FLAG_MASK, "ERR_CP_VOUT_OVP", 9},
	{CP_SC8547D_VBAT_OVP_FLAG_MASK, "ERR_CP_VBAT_OVP", 6},
	{CP_SC8547D_IBAT_OCP_FLAG_MASK, "ERR_CP_IBAT_OCP", 14},
	{CP_SC8547D_VBUS_OVP_FLAG_MASK, "ERR_CP_VBUS_OVP", 7},
	{CP_SC8547D_IBUS_OCP_FLAG_MASK, "ERR_CP_IBUS_OCP", 8},
};

static int sc8547_voocphy_get_chg_enable(struct oplus_voocphy_manager *chip, u8 *data);
static void sc8547_slave_update_data(struct oplus_voocphy_manager *voocphy_mg);
static void sc8547_upload_i2c_err_info(struct sc8547d_device *chip, bool read, s32 *err_info);
static u8 sc8547_voocphy_get_int_value(struct oplus_voocphy_manager *chip);

static int oplus_chg_get_vooc_charging(void)
{
	int vooc_charging_status = 0;
	struct oplus_mms *vooc_topic;
	union mms_msg_data data = { 0 };
	int rc;

	vooc_topic = oplus_mms_get_by_name("vooc");
	if (!vooc_topic)
		return 0;

	rc = oplus_mms_get_item_data(vooc_topic, VOOC_ITEM_VOOC_CHARGING, &data, true);
	if (!rc)
		vooc_charging_status = data.intval;

	return vooc_charging_status;
}

#define ERR_MSG_BUF	PAGE_SIZE
static void sc8547_i2c_error(struct sc8547d_device *chip, bool happen, bool read, s32 *err_info)
{
	if (happen) {
		if (chip->error_reported)
			return;
		if (atomic_read(&chip->i2c_err_count) < I2C_ERR_NUM) {
			atomic_inc(&chip->i2c_err_count);
			return;
		}
		chip->error_reported = true;
		vote(chip->disable_votable, IIC_VOTER, true, 1, false);
		sc8547_upload_i2c_err_info(chip, read, err_info);
	} else {
		vote(chip->disable_votable, IIC_VOTER, false, 0, false);
		chip->error_reported = false;
		atomic_set(&chip->i2c_err_count, 0);
	}
}

/************************************************************************/
static int __sc8547_read_byte(struct i2c_client *client, u8 reg, u8 *data)
{
	s32 ret;
	s32 err_info[2] = { 0 };
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);
	struct sc8547d_device *chip = voocphy->priv_data;

	if (chip == NULL) {
		chg_err("sc8547d chip is NULL\n");
		return -ENODEV;
	}

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		err_info[0] = reg;
		err_info[1] = ret;
		sc8547_i2c_error(chip, true, true, err_info);
		chg_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}
	sc8547_i2c_error(chip, false, true, err_info);
	*data = (u8)ret;

	return 0;
}

static int __sc8547_write_byte(struct i2c_client *client, u8 reg, u8 val)
{
	s32 ret;
	s32 err_info[2] = { 0 };
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);
	struct sc8547d_device *chip = voocphy->priv_data;

	if (chip == NULL) {
		chg_err("sc8547d chip is NULL\n");
		return -ENODEV;
	}

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		err_info[0] = reg;
		err_info[1] = ret;
		sc8547_i2c_error(chip, true, false, err_info);
		chg_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
			val, reg, ret);
		return ret;
	}
	sc8547_i2c_error(chip, false, false, err_info);
	return 0;
}

static int sc8547_read_byte(struct i2c_client *client, u8 reg, u8 *data)
{
	struct sc8547d_device *chip;
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);
	int ret;

	if (voocphy == NULL) {
		chg_err("voocphy is NULL\n");
		return -ENODEV;
	}
	chip = voocphy->priv_data;
	if (chip == NULL) {
		chg_err("sc8547d chip is NULL\n");
		return -ENODEV;
	}

	mutex_lock(&chip->i2c_rw_lock);
	ret = __sc8547_read_byte(client, reg, data);
	mutex_unlock(&chip->i2c_rw_lock);

	return ret;
}

static int sc8547_write_byte(struct i2c_client *client, u8 reg, u8 data)
{
	struct sc8547d_device *chip;
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);
	int ret;

	if (voocphy == NULL) {
		chg_err("voocphy is NULL\n");
		return -ENODEV;
	}
	chip = voocphy->priv_data;
	if (chip == NULL) {
		chg_err("sc8547d chip is NULL\n");
		return -ENODEV;
	}

	mutex_lock(&chip->i2c_rw_lock);
	ret = __sc8547_write_byte(client, reg, data);
	mutex_unlock(&chip->i2c_rw_lock);

	return ret;
}

static int sc8547_update_bits(struct i2c_client *client, u8 reg, u8 mask,
			      u8 data)
{
	struct sc8547d_device *chip;
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);
	int ret;
	u8 tmp;

	if (voocphy == NULL) {
		chg_err("voocphy is NULL\n");
		return -ENODEV;
	}
	chip = voocphy->priv_data;
	if (chip == NULL) {
		chg_err("sc8547d chip is NULL\n");
		return -ENODEV;
	}

	mutex_lock(&chip->i2c_rw_lock);
	ret = __sc8547_read_byte(client, reg, &tmp);
	if (ret) {
		chg_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __sc8547_write_byte(client, reg, tmp);
	if (ret)
		chg_err("Failed: reg=%02X, ret=%d\n", reg, ret);
out:
	mutex_unlock(&chip->i2c_rw_lock);
	return ret;
}

static s32 sc8547_read_word(struct i2c_client *client, u8 reg)
{
	s32 ret;
	s32 err_info[2] = { 0 };
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);
	struct sc8547d_device *chip;

	if (voocphy == NULL) {
		chg_err("voocphy is NULL\n");
		return -ENODEV;
	}
	chip = voocphy->priv_data;
	if (chip == NULL) {
		chg_err("sc8547d chip is NULL\n");
		return -ENODEV;
	}

	mutex_lock(&chip->i2c_rw_lock);
	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0) {
		err_info[0] = reg;
		err_info[1] = ret;
		sc8547_i2c_error(chip, true, true, err_info);
		chg_err("i2c read word fail: can't read reg:0x%02X \n", reg);
		mutex_unlock(&chip->i2c_rw_lock);
		return ret;
	}
	sc8547_i2c_error(chip, false, true, err_info);
	mutex_unlock(&chip->i2c_rw_lock);
	return ret;
}

static s32 sc8547_write_word(struct i2c_client *client, u8 reg, u16 val)
{
	s32 ret;
	s32 err_info[2] = { 0 };
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);
	struct sc8547d_device *chip;

	if (voocphy == NULL) {
		chg_err("voocphy is NULL\n");
		return -ENODEV;
	}
	chip = voocphy->priv_data;
	if (chip == NULL) {
		chg_err("sc8547d chip is NULL\n");
		return -ENODEV;
	}

	mutex_lock(&chip->i2c_rw_lock);
	ret = i2c_smbus_write_word_data(client, reg, val);
	if (ret < 0) {
		err_info[0] = reg;
		err_info[1] = ret;
		sc8547_i2c_error(chip, true, false, err_info);
		chg_err("i2c write word fail: can't write 0x%02X to reg:0x%02X \n", val, reg);
		mutex_unlock(&chip->i2c_rw_lock);
		return ret;
	}
	sc8547_i2c_error(chip, false, false, err_info);
	mutex_unlock(&chip->i2c_rw_lock);
	return 0;
}

static void sc8547d_awake_init(struct sc8547d_device *chip)
{
	chip->chip_ws = wakeup_source_register(chip->dev, "sc8547d_ws");
}

static void sc8547d_awake_deinit(struct sc8547d_device *chip)
{
	if (!chip || !chip->chip_ws) {
		chg_err("chip is NULL\n");
		return;
	}
	wakeup_source_unregister(chip->chip_ws);
	chip->chip_ws = NULL;
}

__maybe_unused static void sc8547d_set_awake(struct sc8547d_device *chip, bool awake)
{
	static bool pm_flag = false;

	if (awake && !pm_flag) {
		pm_flag = true;
		__pm_stay_awake(chip->chip_ws);
	} else if (!awake && pm_flag) {
		__pm_relax(chip->chip_ws);
		pm_flag = false;
	}
}

static int sc8547d_read_byte(struct sc8547d_device *chip, u8 addr, u8 *data)
{
	u8 addr_buf = addr & 0xff;
	int rc = 0;

	mutex_lock(&chip->i2c_rw_lock);
	rc = i2c_smbus_read_i2c_block_data(chip->client, addr_buf, 1, data);
	if (rc < 0) {
		chg_err("read 0x%04x error, rc = %d \n", addr, rc);
		goto error;
	}
	mutex_unlock(&chip->i2c_rw_lock);
	return 0;
error:
	mutex_unlock(&chip->i2c_rw_lock);
	return rc;
}

#define I2C_SMBUS_BLOCK_MAX	32
static int sc8547d_read_data(struct sc8547d_device *chip, u8 addr, u8 *buf, int len)
{
	u8 addr_buf = addr & 0xff;
	int rc = 0;

	mutex_lock(&chip->i2c_rw_lock);
	if (len <= I2C_SMBUS_BLOCK_MAX) {
		rc = i2c_smbus_read_i2c_block_data(chip->client, addr_buf, len, buf);
		if (rc < 0) {
			chg_err("read 0x%04x error, rc=%d\n", addr, rc);
			goto error;
		}
	} else {
		rc = i2c_master_send(chip->client, &addr_buf, 1);
		if (rc < 1) {
			chg_err("write 0x%04x error, rc=%d\n", addr, rc);
			rc = rc < 0 ? rc : -EIO;
			goto error;
		}

		rc = i2c_master_recv(chip->client, buf, len);
		if (rc < len) {
			chg_err("read 0x%04x error, rc=%d\n", addr, rc);
			rc = rc < 0 ? rc : -EIO;
			goto error;
		}
	}

	mutex_unlock(&chip->i2c_rw_lock);
	return rc;
error:
	mutex_unlock(&chip->i2c_rw_lock);
	return rc;
}

static int sc8547d_write_byte(struct sc8547d_device *chip, u8 addr, u8 data)
{
	u8 buf[2] = { addr & 0xff, data };
	int rc = 0;

	mutex_lock(&chip->i2c_rw_lock);
	rc = i2c_master_send(chip->client, buf, 2);
	if (rc < 2) {
		chg_err("write 0x%04x error, rc = %d \n", addr, rc);
		mutex_unlock(&chip->i2c_rw_lock);
		rc = rc < 0 ? rc : -EIO;
		return rc;
	}
	mutex_unlock(&chip->i2c_rw_lock);
	return 0;
}

static int sc8547d_write_data(struct sc8547d_device *chip, u8 addr,
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
		chg_err("write 0x%04x error, ret = %d \n", addr, rc);
		mutex_unlock(&chip->i2c_rw_lock);
		kfree(buf);
		rc = rc < 0 ? rc : -EIO;
		return rc;
	}
	mutex_unlock(&chip->i2c_rw_lock);
	kfree(buf);
	return rc;
}

static int sc8547d_write_bit_mask(struct sc8547d_device *chip, u8 reg,
				  u8 mask, u8 data)
{
	u8 temp = 0;
	int rc = 0;

	mutex_lock(&chip->i2c_rw_lock);
	rc = __sc8547_read_byte(chip->client, reg, &temp);
	if (rc) {
		chg_err("read failed: reg=%02X, rc=%d\n", reg, rc);
		goto out;
	}

	temp = (data & mask) | (temp & (~mask));

	rc = __sc8547_write_byte(chip->client, reg, temp);
	if (rc)
		chg_err("write failed: reg=%02X, rc=%d\n", reg, rc);
out:
	mutex_unlock(&chip->i2c_rw_lock);
	return rc;
}

static int sc8547_voocphy_set_predata(struct oplus_voocphy_manager *chip, u16 val)
{
	s32 ret;
	if (!chip) {
		chg_err("failed: chip is null\n");
		return -1;
	}

	ret = sc8547_write_word(chip->client, SC8547_REG_31, val);
	if (ret < 0) {
		chg_err("failed: write predata\n");
		return -1;
	}
	chg_info("write predata 0x%0x\n", val);
	return ret;
}

static int sc8547_voocphy_set_txbuff(struct oplus_voocphy_manager *chip, u16 val)
{
	s32 ret;
	if (!chip) {
		chg_err("failed: chip is null\n");
		return -1;
	}

	ret = sc8547_write_word(chip->client, SC8547_REG_2C, val);
	if (ret < 0) {
		chg_err("write txbuff\n");
		return -1;
	}

	return ret;
}

static int sc8547_voocphy_get_adapter_info(struct oplus_voocphy_manager *chip)
{
	s32 data;
	if (!chip) {
		chg_err("chip is null\n");
		return -1;
	}

	data = sc8547_read_word(chip->client, SC8547_REG_2E);

	if (data < 0) {
		chg_err("sc8547_read_word faile\n");
		return -1;
	}

	VOOCPHY_DATA16_SPLIT(data, chip->voocphy_rx_buff, chip->vooc_flag);
	chg_info("data: 0x%0x, vooc_flag: 0x%0x, vooc_rxdata: 0x%0x\n", data,
		chip->vooc_flag, chip->voocphy_rx_buff);

	return 0;
}

static void sc8547_voocphy_update_data(struct oplus_voocphy_manager *chip)
{
	u8 data_block[10] = { 0 };
	s32 ret = 0;
	s32 err_info[2] = { 0 };
	struct sc8547d_device *dev;

	dev = chip->priv_data;
	if (dev == NULL) {
		chg_err("sc8547d chip is NULL\n");
		return;
	}

	chip->interrupt_flag = sc8547_voocphy_get_int_value(chip);

	mutex_lock(&dev->adc_freeze_lock);
	sc8547_update_bits(dev->client, SC8547_REG_11, SC8547_ADC_FREEZE_MASK, SC8547_ADC_FREEZE_MASK);
	ret = sc8547d_read_data(dev, SC8547_REG_13, data_block, 10);
	sc8547_update_bits(dev->client, SC8547_REG_11, SC8547_ADC_FREEZE_MASK, 0);
	mutex_unlock(&dev->adc_freeze_lock);
	if (ret < 0) {
		err_info[0] = SC8547_REG_13;
		err_info[1] = ret;
		sc8547_i2c_error(dev, true, true, err_info);
		chg_err("sc8547_update_data read adc error \n");
	} else {
		sc8547_i2c_error(dev, false, true, err_info);
	}
	chip->cp_ichg = (((data_block[0] & SC8547_IBUS_POL_H_MASK) << 8) | data_block[1]) * SC8547_IBUS_ADC_LSB;
	chip->cp_vbus = (((data_block[2] & SC8547_VBUS_POL_H_MASK) << 8) | data_block[3]) * SC8547_VBUS_ADC_LSB;
	chip->cp_vac = (((data_block[4] & SC8547_VAC_POL_H_MASK) << 8) | data_block[5]) * SC8547_VAC_ADC_LSB;
	chip->cp_vsys = (((data_block[6] & SC8547_VOUT_POL_H_MASK) << 8) | data_block[7]) * SC8547_VOUT_ADC_LSB;
	chip->cp_vbat = (((data_block[8] & SC8547_VBAT_POL_H_MASK) << 8) | data_block[9]) * SC8547_VBAT_ADC_LSB;

	chip->master_cp_ichg = chip->cp_ichg;

	chg_info("cp_ichg = %d cp_vbus = %d, cp_vsys = %d cp_vbat = %d cp_vac = %d int_flag = %d,"
		"[6/9/D/F]=[0x%x, 0x%x][0x%x, 0x%x]", chip->cp_ichg, chip->cp_vbus, chip->cp_vsys,
		chip->cp_vbat, chip->cp_vac, chip->interrupt_flag, chip->int_column[0],
		chip->int_column[1], chip->int_column[2], chip->int_column[3]);

}

static int sc8547_voocphy_get_cp_ichg(struct oplus_voocphy_manager *voocphy)
{
	u8 data_block[2] = { 0 };
	int cp_ichg = 0;
	u8 cp_enable = 0;
	s32 ret = 0;
	s32 err_info[2] = { 0 };
	struct sc8547d_device *chip;

	chip = voocphy->priv_data;
	if (chip == NULL) {
		chg_err("sc8547d chip is NULL\n");
		return -ENODEV;
	}

	if (oplus_chg_get_vooc_charging())
		return voocphy->master_cp_ichg;

	sc8547_voocphy_get_chg_enable(voocphy, &cp_enable);
	if (cp_enable == 0)
		return 0;
	/*parse data_block for improving time of interrupt*/
	mutex_lock(&chip->adc_freeze_lock);
	sc8547_update_bits(chip->client, SC8547_REG_11, SC8547_ADC_FREEZE_MASK, SC8547_ADC_FREEZE_MASK);
	ret = sc8547d_read_data(chip, SC8547_REG_13, data_block, 2);
	sc8547_update_bits(chip->client, SC8547_REG_11, SC8547_ADC_FREEZE_MASK, 0);
	mutex_unlock(&chip->adc_freeze_lock);
	if (ret < 0) {
		err_info[0] = SC8547_REG_13;
		err_info[1] = ret;
		sc8547_i2c_error(chip, true, true, err_info);
		chg_err("sc8547 read ichg error \n");
	} else {
		sc8547_i2c_error(chip, false, true, err_info);
	}

	cp_ichg = (((data_block[0] & SC8547_IBUS_POL_H_MASK) << 8) | data_block[1]) * SC8547_IBUS_ADC_LSB;
	chg_info("%s cp_ichg:%d\n", chip->dev->of_node->name, cp_ichg);

	return cp_ichg;
}

static int sc8547_get_cp_ichg(struct sc8547d_device *chip)
{
	u8 data_block[2] = { 0 };
	int cp_ichg = 0;
	u8 cp_enable = 0;
	s32 ret = 0;

	sc8547_voocphy_get_chg_enable(chip->voocphy, &cp_enable);

	if (cp_enable == 0)
		return 0;
	/*parse data_block for improving time of interrupt*/
	mutex_lock(&chip->adc_freeze_lock);
	sc8547_update_bits(chip->client, SC8547_REG_11, SC8547_ADC_FREEZE_MASK, SC8547_ADC_FREEZE_MASK);
	ret = sc8547d_read_data(chip, SC8547_REG_13, data_block, 2);
	sc8547_update_bits(chip->client, SC8547_REG_11, SC8547_ADC_FREEZE_MASK, 0);
	mutex_unlock(&chip->adc_freeze_lock);
	if (ret < 0) {
		chg_err("sc8547 read ichg error \n");
		return 0;
	}

	cp_ichg = (((data_block[0] & SC8547_IBUS_POL_H_MASK) << 8) | data_block[1]) * SC8547_IBUS_ADC_LSB;

	return cp_ichg;
}

static int sc8547_voocphy_get_cp_vbat(struct oplus_voocphy_manager *voocphy)
{
	u8 data_block[2] = { 0 };
	s32 ret = 0;
	s32 err_info[2] = { 0 };
	struct sc8547d_device *chip;

	chip = voocphy->priv_data;
	if (chip == NULL) {
		chg_err("sc8547d chip is NULL\n");
		return -ENODEV;
	}

	if (oplus_chg_get_vooc_charging())
		return voocphy->cp_vbat;

	/*parse data_block for improving time of interrupt*/
	mutex_lock(&chip->adc_freeze_lock);
	sc8547_update_bits(chip->client, SC8547_REG_11, SC8547_ADC_FREEZE_MASK, SC8547_ADC_FREEZE_MASK);
	ret = sc8547d_read_data(chip, SC8547_REG_1B, data_block, 2);
	sc8547_update_bits(chip->client, SC8547_REG_11, SC8547_ADC_FREEZE_MASK, 0);
	mutex_unlock(&chip->adc_freeze_lock);
	if (ret < 0) {
		err_info[0] = SC8547_REG_1B;
		err_info[1] = ret;
		sc8547_i2c_error(chip, true, true, err_info);
		chg_err("sc8547 read vbat error \n");
	} else {
		sc8547_i2c_error(chip, false, true, err_info);
	}

	voocphy->cp_vbat = (((data_block[0] & SC8547_VBAT_POL_H_MASK) << 8) | data_block[1]) * SC8547_VBAT_ADC_LSB;

	return voocphy->cp_vbat;
}

static int sc8547_get_cp_vbat(struct sc8547d_device *chip)
{
	u8 data_block[2] = { 0 };
	s32 ret = 0;

	/*parse data_block for improving time of interrupt*/
	mutex_lock(&chip->adc_freeze_lock);
	sc8547_update_bits(chip->client, SC8547_REG_11, SC8547_ADC_FREEZE_MASK, SC8547_ADC_FREEZE_MASK);
	ret = sc8547d_read_data(chip, SC8547_REG_1B, data_block, 2);
	sc8547_update_bits(chip->client, SC8547_REG_11, SC8547_ADC_FREEZE_MASK, 0);
	mutex_unlock(&chip->adc_freeze_lock);
	if (ret < 0) {
		chg_err("sc8547 read vbat error \n");
		return 0;
	}

	return (((data_block[0] & SC8547_VBAT_POL_H_MASK) << 8) | data_block[1]) * SC8547_VBAT_ADC_LSB;
}

static int sc8547_voocphy_get_cp_vbus(struct oplus_voocphy_manager *voocphy)
{
	u8 data_block[2] = { 0 };
	s32 ret = 0;
	s32 err_info[2] = { 0 };
	struct sc8547d_device *chip;

	chip = voocphy->priv_data;
	if (chip == NULL) {
		chg_err("sc8547d chip is NULL\n");
		return -ENODEV;
	}

	if (oplus_chg_get_vooc_charging())
		return voocphy->cp_vbus;

	/* parse data_block for improving time of interrupt */
	mutex_lock(&chip->adc_freeze_lock);
	sc8547_update_bits(chip->client, SC8547_REG_11, SC8547_ADC_FREEZE_MASK, SC8547_ADC_FREEZE_MASK);
	ret = sc8547d_read_data(chip, SC8547_REG_15, data_block, 2);
	sc8547_update_bits(chip->client, SC8547_REG_11, SC8547_ADC_FREEZE_MASK, 0);
	mutex_unlock(&chip->adc_freeze_lock);
	if (ret < 0) {
		err_info[0] = SC8547_REG_15;
		err_info[1] = ret;
		sc8547_i2c_error(chip, true, true, err_info);
		chg_err("sc8547 read vbat error \n");
	} else {
		sc8547_i2c_error(chip, false, true, err_info);
	}

	voocphy->cp_vbus = (((data_block[0] & SC8547_VBUS_POL_H_MASK) << 8) | data_block[1]) * SC8547_VBUS_ADC_LSB;

	return voocphy->cp_vbus;
}

static int sc8547_get_cp_vbus(struct sc8547d_device *chip)
{
	u8 data_block[2] = { 0 };
	s32 ret = 0;

	/* parse data_block for improving time of interrupt */
	mutex_lock(&chip->adc_freeze_lock);
	sc8547_update_bits(chip->client, SC8547_REG_11, SC8547_ADC_FREEZE_MASK, SC8547_ADC_FREEZE_MASK);
	ret = sc8547d_read_data(chip, SC8547_REG_15, data_block, 2);
	sc8547_update_bits(chip->client, SC8547_REG_11, SC8547_ADC_FREEZE_MASK, 0);
	mutex_unlock(&chip->adc_freeze_lock);
	if (ret < 0) {
		chg_err("sc8547 read vbat error \n");
		return 0;
	}

	return (((data_block[0] & SC8547_VBUS_POL_H_MASK) << 8) | data_block[1]) * SC8547_VBUS_ADC_LSB;
}

/*********************************************************************/
static int sc8547_reg_reset(struct sc8547d_device *chip, bool enable)
{
	int ret;
	u8 val;
	if (enable)
		val = SC8547_RESET_REG;
	else
		val = SC8547_NO_REG_RESET;

	val <<= SC8547_REG_RESET_SHIFT;

	ret = sc8547_update_bits(chip->client, SC8547_REG_07,
				 SC8547_REG_RESET_MASK, val);

	return ret;
}

static bool sc8547d_hw_version_check(struct sc8547d_device *chip)
{
	int ret;
	u8 val;
	ret = sc8547_read_byte(chip->client, SC8547_REG_36, &val);
	if (val == SC8547D_DEVICE_ID) {
		chip->ic_sc8547d = true;
		return true;
	} else {
		chip->ic_sc8547d = false;
		return false;
	}
}

static int sc8547_voocphy_get_chg_enable(struct oplus_voocphy_manager *chip, u8 *data)
{
	int ret = 0;

	if (!chip) {
		chg_err("Failed\n");
		return -1;
	}

	ret = sc8547_read_byte(chip->client, SC8547_REG_07, data);
	if (ret < 0) {
		chg_err("SC8547_REG_07\n");
		return -1;
	}

	*data = *data >> 7;

	return ret;
}

static int sc8547_voocphy_get_voocphy_enable(
	struct oplus_voocphy_manager *chip, u8 *data)
{
	int ret = 0;

	if (!chip) {
		chg_err("Failed\n");
		return -1;
	}

	ret = sc8547_read_byte(chip->client, SC8547_REG_2B, data);
	if (ret < 0) {
		chg_err("SC8547_REG_2B\n");
		return -1;
	}

	return ret;
}

static void sc8547_voocphy_dump_reg_in_err_issue(struct oplus_voocphy_manager *voocphy)
{
	return;
}

static int sc8547_voocphy_get_adc_enable(struct oplus_voocphy_manager *chip, u8 *data)
{
	int ret = 0;

	if (!chip) {
		chg_err("Failed\n");
		return -1;
	}

	ret = sc8547_read_byte(chip->client, SC8547_REG_11, data);
	if (ret < 0) {
		chg_err("SC8547_REG_11\n");
		return -1;
	}

	*data = *data >> 7;

	return ret;
}

static u8 sc8547_voocphy_get_int_value(struct oplus_voocphy_manager *chip)
{
	int ret = 0;
	u8 data_block[10] = { 0 };
	struct sc8547d_device *dev;

	dev = chip->priv_data;
	if (dev == NULL) {
		chg_err("sc8547d chip is NULL\n");
		return -ENODEV;
	}

	ret = sc8547d_read_data(dev, SC8547_REG_06, data_block, 10);
	if (ret < 0) {
		chg_err(" read SC8547_REG_06 failed\n");
		return -1;
	}

	chip->int_column[0] = data_block[0];
	chip->int_column[1] = data_block[3];
	chip->int_column[2] = data_block[7];
	chip->int_column[3] = data_block[9];
	memcpy(chip->int_column_pre, chip->int_column, sizeof(chip->int_column));
	memcpy(chip->reg_dump, data_block, sizeof(data_block));

	return chip->int_column[3];
}

#define CP_SWITCHING_WORK_DELAY_MS	1000
static int sc8547_voocphy_set_chg_enable(struct oplus_voocphy_manager *chip,
				 bool enable)
{
	int rc = 0;
	struct sc8547d_device *sc8547d_chip;

	if (!chip) {
		chg_err("Failed\n");
		return -1;
	}
	sc8547d_chip = chip->priv_data;
	chg_info("enable=%d, cp_reg_track[4]=%d\n", enable, sc8547d_chip->cp_reg_track[4]);
	if (enable) {
		rc = sc8547_write_byte(chip->client, SC8547_REG_07, 0x80);
		schedule_delayed_work(&sc8547d_chip->track_cp_switching_work,
			msecs_to_jiffies(CP_SWITCHING_WORK_DELAY_MS));
	} else {
		rc = sc8547_write_byte(chip->client, SC8547_REG_07, 0x0);
		cancel_delayed_work(&sc8547d_chip->track_cp_switching_work);
	}

	return rc;
}

static int sc8547_set_chg_enable(struct sc8547d_device *chip, bool enable)
{
	if (enable) {
		if (chip->use_slave_cp)
			return sc8547_write_byte(chip->client, SC8547_REG_07, 0x81); /* FSW=600kHz */
		else
			return sc8547_write_byte(chip->client, SC8547_REG_07, 0x80); /* FSW=500kHz */
	} else {
		if (chip->use_slave_cp)
			return sc8547_write_byte(chip->client, SC8547_REG_07, 0x01);
		else
			return sc8547_write_byte(chip->client, SC8547_REG_07, 0x00);
	}
}

static void sc8547_voocphy_set_pd_svooc_config(
	struct oplus_voocphy_manager *chip, bool enable)
{
	int ret = 0;
	u8 reg_data = 0;
	if (!chip) {
		chg_err("Failed\n");
		return;
	}

	if (enable) {
		reg_data = 0xa0 | (chip->ocp_reg & 0xf);
		sc8547_write_byte(chip->client, SC8547_REG_05, reg_data);
		sc8547_write_byte(chip->client, SC8547_REG_09, 0x13);
	} else {
		reg_data = 0x20 | (chip->ocp_reg & 0xf);
		sc8547_write_byte(chip->client, SC8547_REG_05, reg_data);
	}

	ret = sc8547_read_byte(chip->client, SC8547_REG_05, &reg_data);
	if (ret < 0) {
		chg_err("SC8547_REG_05\n");
		return;
	}
	chg_err("pd_svooc config SC8547_REG_05 = 0x%x\n", reg_data);
}

static bool sc8547_voocphy_get_pd_svooc_config(struct oplus_voocphy_manager *chip)
{
	int ret = 0;
	u8 data = 0;

	if (!chip) {
		chg_err("Failed\n");
		return false;
	}

	ret = sc8547_read_byte(chip->client, SC8547_REG_05, &data);
	if (ret < 0) {
		chg_err("SC8547_REG_05\n");
		return false;
	}

	chg_err("SC8547_REG_05 = 0x%0x\n", data);

	data = data >> 7;
	if (data == 1)
		return true;
	else
		return false;
}

static int sc8547_voocphy_set_adc_enable(struct oplus_voocphy_manager *chip,
				 bool enable)
{
	if (!chip) {
		chg_err("Failed\n");
		return -1;
	}

	if (enable)
		return sc8547_update_bits(chip->client, SC8547_REG_11, SC8547_ADC_EN_MASK, SC8547_ADC_EN_MASK);
	else
		return sc8547_write_byte(chip->client, SC8547_REG_11, 0x00);
}

static int sc8547_set_adc_enable(struct sc8547d_device *chip, bool enable)
{
	int ret = 0;

	if (!chip) {
		chg_err("Failed\n");
		return -1;
	}

	if (enable)
		ret = sc8547_update_bits(chip->client, SC8547_REG_11, SC8547_ADC_EN_MASK, SC8547_ADC_EN_MASK);
	else
		ret = sc8547_write_byte(chip->client, SC8547_REG_11, 0x00);

	chg_debug("enable = %d. ret = %d\n", enable, ret);

	return ret;
}

static void sc8547_voocphy_send_handshake(struct oplus_voocphy_manager *chip)
{
	sc8547_write_byte(chip->client, SC8547_REG_2B, 0x81);
}

static int sc8547d_cp_set_sstimeout_ucp_enable(struct oplus_chg_ic_dev *ic_dev, bool enable);
static int sc8547d_voocphy_set_sstimeout_ucp_enable(struct oplus_voocphy_manager *chip, bool enable)
{
	int rc = 0;
	struct sc8547d_device *dev;
	dev = chip->priv_data;

	if (dev == NULL) {
		chg_err("sc8547d chip is NULL\n");
		return -ENODEV;
	}
	if (!chip->fcl_support && !chip->svooc_pcc_strategy)
		return -EINVAL;

	rc = sc8547d_cp_set_sstimeout_ucp_enable(dev->cp_ic, enable);
	return rc;
}

static int sc8547_voocphy_reset_voocphy(struct oplus_voocphy_manager *chip)
{
	u8 reg_data;
	struct sc8547d_device *dev;

	dev = chip->priv_data;
	if (dev == NULL) {
		chg_err("sc8547d chip is NULL\n");
		return -ENODEV;
	}

	cancel_delayed_work(&dev->track_cp_switching_work);

	/* turn off mos */
	sc8547_write_byte(chip->client, SC8547_REG_07, 0x0);
	if(dev->ic_sc8547d) {
		/* sc8547b need disable WDT when exit charging, to avoid after WDT time out.
		IF time out, sc8547b will trigger interrupt frequently.
		in addition, sc8547 and sc8547b WDT will disable when disable CP */
		sc8547_update_bits(chip->client, SC8547_REG_09,
				   SC8547_WATCHDOG_MASK, SC8547_WATCHDOG_DIS);
	}
	/* hwic config with plugout */
	reg_data = 0x20 | (chip->ovp_reg & 0x1f);
	sc8547_write_byte(chip->client, SC8547_REG_00, reg_data);
	sc8547_write_byte(chip->client, SC8547_REG_02, 0x01);
	sc8547_write_byte(chip->client, SC8547_REG_04, dev->vbus_ovp_reg);
	reg_data = 0x20 | (chip->ocp_reg & 0xf); /* ibus_ucp time: 50ms */

	sc8547_write_byte(chip->client, SC8547_REG_05, reg_data);
	sc8547_write_byte(chip->client, SC8547_REG_11, 0x00);

	/* clear tx data */
	sc8547_write_byte(chip->client, SC8547_REG_2C, 0x00);
	sc8547_write_byte(chip->client, SC8547_REG_2D, 0x00);

	/* disable vooc phy irq */
	sc8547_write_byte(chip->client, SC8547_REG_30, 0x7f);

	/* set D+ HiZ */
	sc8547_write_byte(chip->client, SC8547_REG_21, 0xc0);

	/* select big bang mode */

	/* disable vooc */
	sc8547_write_byte(chip->client, SC8547_REG_2B, 0x00);

	/* set predata */
	sc8547_write_word(chip->client, SC8547_REG_31, 0x0);

	/* mask insert irq */
	sc8547_write_byte(chip->client, SC8547_REG_10, 0x02);
	dev->voocphy_enable = false;
	sc8547d_voocphy_set_sstimeout_ucp_enable(chip, true);
	chg_err("oplus_vooc_reset_voocphy done");

	return VOOCPHY_SUCCESS;
}

static int sc8547_voocphy_reactive_voocphy(struct oplus_voocphy_manager *chip)
{
	u8 value;

	/*set predata to avoid cmd of adjust current(0x01)return error, add voocphy bit0 hold time to 800us*/
	sc8547_write_word(chip->client, SC8547_REG_31, 0x0);
	sc8547_read_byte(chip->client, SC8547_REG_34, &value);
	value = value | (3 << 5);
	sc8547_write_byte(chip->client, SC8547_REG_34, value);

	/*dpdm*/
	sc8547_write_byte(chip->client, SC8547_REG_21, 0x21);
	sc8547_write_byte(chip->client, SC8547_REG_22, 0x00);
	sc8547_write_byte(chip->client, SC8547_REG_33, 0xD1);

	/*clear tx data*/
	sc8547_write_byte(chip->client, SC8547_REG_2C, 0x00);
	sc8547_write_byte(chip->client, SC8547_REG_2D, 0x00);

	/*vooc*/
	sc8547_write_byte(chip->client, SC8547_REG_30, 0x05);
	sc8547_voocphy_send_handshake(chip);

	chg_info("oplus_vooc_reactive_voocphy done");

	return VOOCPHY_SUCCESS;
}

static int sc8547_init_device(struct sc8547d_device *chip)
{
	u8 reg_data;

	sc8547_write_byte(chip->client, SC8547_REG_11, 0x0); /* ADC_CTRL:disable */
	sc8547_write_byte(chip->client, SC8547_REG_02, 0x01);
	sc8547_write_byte(chip->client, SC8547_REG_04, chip->vbus_ovp_reg); /* VBUS_OVP:10 2:1 or 1:1V */
	reg_data = 0x20 | (chip->ovp_reg & 0x1f);
	sc8547_write_byte(chip->client, SC8547_REG_00, reg_data); /* VBAT_OVP:4.65V */
	reg_data = 0x20 | (chip->ocp_reg & 0xf);
	sc8547_write_byte(chip->client, SC8547_REG_05, reg_data); /* IBUS_OCP_UCP:3.6A, UCP_TIME: 50ms */
	sc8547_write_byte(chip->client, SC8547_REG_01, 0xbf);
	sc8547_write_byte(chip->client, SC8547_REG_2B, 0x00); /* OOC_CTRL:disable */
	sc8547_write_byte(chip->client, SC8547_REG_34, 0x60);
	sc8547_update_bits(chip->client, SC8547_REG_09, SC8547_IBUS_UCP_RISE_MASK_MASK,
			   (1 << SC8547_IBUS_UCP_RISE_MASK_SHIFT));
	sc8547_write_byte(chip->client, SC8547_REG_10, 0x02); /* mask insert irq */
	if (chip->always_otg_en || chip->otg_connected)
		sc8547_update_bits(chip->client, SC8547D_ADDR_OTG_EN, SC8547D_OTG_EN_MASK, 0x04);
	return 0;
}

static int sc8547_voocphy_init_vooc(struct oplus_voocphy_manager *voocphy)
{
	u8 value;
	struct sc8547d_device *chip;

	chg_info(" >>>>start init vooc\n");
	chip = voocphy->priv_data;
	if (chip == NULL) {
		chg_err("sc8547d chip is NULL\n");
		return -ENODEV;
	}

	chip->voocphy_enable = true;
	sc8547_reg_reset(chip, true);
	sc8547_init_device(chip);

	if (chip->use_vooc_phy) {
		chip->voocphy_enable = true;
		/*to avoid cmd of adjust current(0x01)return error, add voocphy bit0 hold time to 800us */
		/*SET PREDATA */
		sc8547_write_word(chip->client, SC8547_REG_31, 0x0);
		/*sc8547_set_predata(0x0);*/
		sc8547_read_byte(chip->client, SC8547_REG_34, &value);
		value = value | (1 << 6);
		sc8547_write_byte(chip->client, SC8547_REG_34, value);
		chg_err("read value %d\n", value);

		/*dpdm */
		if (oplus_chg_get_fcs_support_flags()) {
			sc8547_write_byte(chip->client, SC8547_REG_21, 0x31);
			msleep(30);
		}
		sc8547_write_byte(chip->client, SC8547_REG_21, 0x21);
		sc8547_write_byte(chip->client, SC8547_REG_22, 0x00);
		sc8547_write_byte(chip->client, SC8547_REG_33, 0xD1);

		/*vooc */
		sc8547_write_byte(chip->client, SC8547_REG_30, 0x05);
	}
	return 0;
}

static int sc8547_svooc_hw_setting(struct sc8547d_device *chip, bool wdt_cfg)
{
	u8 reg_data;
	sc8547_write_byte(chip->client, SC8547_REG_02, 0x01); /*VAC_OVP:12v*/
	sc8547_write_byte(chip->client, SC8547_REG_04, chip->vbus_ovp_reg); /*VBUS_OVP:10v*/
	reg_data = 0x20 | (chip->ocp_reg & 0xf);
	sc8547_write_byte(chip->client, SC8547_REG_05, reg_data); /*IBUS_OCP_UCP:3.6A, UCP_TIME:50ms */
	if (wdt_cfg)
		sc8547_write_byte(chip->client, SC8547_REG_09, 0x13); /*WD:1000ms*/
	else
		sc8547_update_bits(chip->client, SC8547_REG_09, 0x90, 0x10); /* 2:1, IBUS_UCP_RISE_MASK */
	sc8547_update_bits(chip->client, SC8547_REG_11, SC8547_ADC_EN_MASK, SC8547_ADC_EN_MASK); /*ADC_CTRL:ADC_EN*/
	sc8547_write_byte(chip->client, SC8547_REG_0D, 0x70);
	/*sc8547_write_byte(chip->client, SC8547_REG_2B, 0x81);*/ /*VOOC_CTRL,send handshake*/

	sc8547_write_byte(chip->client, SC8547_REG_33, 0xd1); /*Loose_det=1*/
	sc8547_write_byte(chip->client, SC8547_REG_34, 0x60);
	sc8547d_voocphy_set_sstimeout_ucp_enable(chip->voocphy, false);
	return 0;
}

static int sc8547_vooc_hw_setting(struct sc8547d_device *chip, bool wdt_cfg)
{
	sc8547_write_byte(chip->client, SC8547_REG_02, 0x07); /*VAC_OVP:*/
	sc8547_write_byte(chip->client, SC8547_REG_04, 0x64); /*VBUS_OVP:11V*/
	sc8547_write_byte(chip->client, SC8547_REG_05, 0x2c); /*IBUS_OCP_UCP:*/
	if (wdt_cfg)
		sc8547_write_byte(chip->client, SC8547_REG_09, 0x93); /*WD:1000ms*/
	else
		sc8547_update_bits(chip->client, SC8547_REG_09, 0x90, 0x90); /* 1:1, IBUS_UCP_RISE_MASK */
	sc8547_update_bits(chip->client, SC8547_REG_11, SC8547_ADC_EN_MASK, SC8547_ADC_EN_MASK); /*ADC_CTRL:ADC_EN*/
	sc8547_write_byte(chip->client, SC8547_REG_33, 0xd1); /*Loose_det*/
	sc8547_write_byte(chip->client, SC8547_REG_34, 0x60);
	if (chip->use_slave_cp)
		sc8547_write_byte(chip->client, SC8547_REG_3C, 0x60); /* Disable VBUS_IN_RANGE */
	sc8547d_voocphy_set_sstimeout_ucp_enable(chip->voocphy, false);
	return 0;
}

static int sc8547_5v2a_hw_setting(struct sc8547d_device *chip)
{
	sc8547_write_byte(chip->client, SC8547_REG_02, 0x07); /*VAC_OVP:*/
	sc8547_write_byte(chip->client, SC8547_REG_04, chip->vbus_ovp_reg); /*VBUS_OVP:*/
	sc8547_write_byte(chip->client, SC8547_REG_07, 0x0);
	sc8547_write_byte(chip->client, SC8547_REG_09, 0x10); /*WD:DISABLE*/

	sc8547_write_byte(chip->client, SC8547_REG_11, 0x00); /*ADC_CTRL:*/
	sc8547_write_byte(chip->client, SC8547_REG_2B, 0x00); /*VOOC_CTRL*/
	return 0;
}

static int sc8547_pdqc_hw_setting(struct sc8547d_device *chip)
{
	sc8547_write_byte(chip->client, SC8547_REG_00, 0x36); /*VAC_OV:*/
	sc8547_write_byte(chip->client, SC8547_REG_02, 0x01); /*VAC_OVP:*/
	sc8547_write_byte(chip->client, SC8547_REG_04, chip->vbus_ovp_reg); /*VBUS_OVP*/
	sc8547_write_byte(chip->client, SC8547_REG_07, 0x0);
	sc8547_write_byte(chip->client, SC8547_REG_09, 0x10); /*WD:DISABLE*/
	sc8547_write_byte(chip->client, SC8547_REG_11, 0x00); /*ADC_CTRL*/
	sc8547_write_byte(chip->client, SC8547_REG_2B, 0x00); /*VOOC_CTRL*/
	chg_err("sc8547_pdqc_hw_setting done");
	return 0;
}

static int sc8547_voocphy_hw_setting(struct oplus_voocphy_manager *voocphy, int reason)
{
	struct sc8547d_device *chip;

	if (!voocphy) {
		chg_err("voocphy is null exit\n");
		return -EINVAL;
	}

	chip = voocphy->priv_data;
	if (chip == NULL) {
		chg_err("sc8547d chip is NULL\n");
		return -ENODEV;
	}

	switch (reason) {
	case SETTING_REASON_PROBE:
	case SETTING_REASON_RESET:
		sc8547_init_device(chip);
		chg_info("SETTING_REASON_RESET OR PROBE\n");
		break;
	case SETTING_REASON_SVOOC:
		sc8547_svooc_hw_setting(chip, true);
		chg_info("SETTING_REASON_SVOOC\n");
		break;
	case SETTING_REASON_VOOC:
		sc8547_vooc_hw_setting(chip, true);
		chg_info("SETTING_REASON_VOOC\n");
		break;
	case SETTING_REASON_5V2A:
		sc8547_5v2a_hw_setting(chip);
		chg_info("SETTING_REASON_5V2A\n");
		break;
	case SETTING_REASON_PDQC:
		sc8547_pdqc_hw_setting(chip);
		chg_info("SETTING_REASON_PDQC\n");
		break;
	default:
		chg_err("do nothing\n");
		break;
	}
	return 0;
}

static int sc8547d_cp_manual_enable(struct oplus_voocphy_manager *chip, bool en)
{
	int rc;
	struct sc8547d_device *dev;

	if (!chip) {
		chg_err("Failed\n");
		return -ENODEV;
	}

	dev = chip->priv_data;
	if ((dev == NULL) || (dev->client == NULL)) {
		chg_err("sc8547d chip is NULL\n");
		return -ENODEV;
	}

	if (en) {
		rc = sc8547_update_bits(dev->client, SC8547_REG_0C, SC8547_IBUS_REG_EN_MASK, 0x80);
		if (rc < 0) {
			chg_err("Failed to enable manual mode, ret=%d\n", rc);
			return rc;
		}
		chg_info("CTRL7:force Manual_en");
	} else {
		rc = sc8547_update_bits(dev->client, SC8547_REG_0C, SC8547_IBUS_REG_EN_MASK, 0x00);
		if (rc < 0) {
			chg_err("Failed to disable manual mode, ret=%d\n", rc);
			return rc;
		}
		chg_info("CTRL7:force Manual_disable");
	}

	return 0;
}

static void sc8547_dual_chan_buck_set_ucp(struct oplus_voocphy_manager *chip, int ucp_value)
{
	u8 value;

	if (ucp_value) {
		/* to avoid cp ucp cause buck chg, change ucp as 100ms */
		sc8547_read_byte(chip->client, SC8547_REG_05, &value);
		value = value | OPLUS_SC8547D_UCP_100MS;
		chg_info("set 05 reg = 0x%x\n", value);
		sc8547_write_byte(chip->client, SC8547_REG_05, value);
	} else {
		sc8547_read_byte(chip->client, SC8547_REG_05, &value);
		value = (value & (~OPLUS_SC8547D_UCP_100MS)) | OPLUS_SC8547D_UCP_5MS;
		chg_info("set 05 reg = 0x%x\n", value);
		sc8547_write_byte(chip->client, SC8547_REG_05, value);
	}
}

static void sc8547_hardware_init(struct sc8547d_device *chip)
{
	sc8547_reg_reset(chip, true);
	sc8547_init_device(chip);
}

static void sc8547_voocphy_hardware_init(struct oplus_voocphy_manager *voocphy)
{
	struct sc8547d_device *chip;

	chip = voocphy->priv_data;
	if (chip == NULL) {
		chg_err("sc8547d chip is NULL\n");
		return;
	}

	sc8547_reg_reset(chip, true);
	sc8547_init_device(chip);
}


static bool sc8547_voocphy_check_cp_int_status(
	struct oplus_voocphy_manager *chip, int *err_type, bool check)
{
	int i = 0;
	int level = IRQ_EVENT_SC8547D_NUM;
	struct sc8547d_device *chip_device;
	bool dump_err = false;

	if (NULL == chip || NULL == err_type) {
		chg_err("chip ir err_type is NULL\n");
		return false;
	}

	chip_device = chip->priv_data;
	if (!chip_device) {
		chg_err("sc8547d_device is NULL\n");
		return false;
	}

	for (i = 0; i < 3; i++) {
		if ((chip_device->cp_reg_track[0] & sc8547d_int_flag[i].mask) ||
		    (sc8547d_int_flag[i].mask & chip->int_column_pre[0])) {
			chg_info("cp int 6 happened [0x%x, 0x%x][%d, %d, %d]\n",
				chip->int_column_pre[0], chip_device->cp_reg_track[0],
				i, sc8547d_int_flag[i].level, level);
			dump_err = true;
			if (check)
				goto check_out;
			if (sc8547d_int_flag[i].level < level) {
				level = sc8547d_int_flag[i].level;
				*err_type = i + 2;
			}
			chip_device->cp_reg_track[0] &= ~sc8547d_int_flag[i].mask;
		}
	}

	for (i = 3; i < 4; i++) {
		if ((chip_device->cp_reg_track[1] & sc8547d_int_flag[i].mask) ||
		    (sc8547d_int_flag[i].mask & chip->int_column_pre[1])) {
			chg_info("cp int 9 happened [0x%x, 0x%x][%d, %d, %d]\n", chip->int_column_pre[1],
				chip_device->cp_reg_track[1], i, sc8547d_int_flag[i].level, level);
			dump_err = true;
			if (check)
				goto check_out;
			if (sc8547d_int_flag[i].level < level) {
				level = sc8547d_int_flag[i].level;
				*err_type = i + 2;
			}
			chip_device->cp_reg_track[1] &= ~sc8547d_int_flag[i].mask;
		}
	}

	for (i = 4; i < 6; i++) {
		if ((chip_device->cp_reg_track[2] & sc8547d_int_flag[i].mask) ||
		    (sc8547d_int_flag[i].mask & chip->int_column_pre[2])) {
			chg_info("cp int D happened [0x%x, 0x%x][%d, %d, %d]\n", chip->int_column_pre[2],
				chip_device->cp_reg_track[2], i, sc8547d_int_flag[i].level, level);
			dump_err = true;
			if (check)
				goto check_out;
			if (sc8547d_int_flag[i].level < level) {
				level = sc8547d_int_flag[i].level;
				*err_type = i + 2;
			}
			chip_device->cp_reg_track[2] &= ~sc8547d_int_flag[i].mask;
		}
	}

	for (i = 6; i < 11; i++) {
		if ((chip_device->cp_reg_track[3] & sc8547d_int_flag[i].mask) ||
		    (sc8547d_int_flag[i].mask & chip->int_column_pre[3])) {
			chg_info("cp int F happened [0x%x, 0x%x][%d, %d, %d]\n", chip->int_column_pre[3],
				chip_device->cp_reg_track[3], i, sc8547d_int_flag[i].level, level);
			dump_err = true;
			if (check)
				goto check_out;
			if (sc8547d_int_flag[i].level < level) {
				level = sc8547d_int_flag[i].level;
				*err_type = i + 2;
			}
			chip_device->cp_reg_track[3] &= ~sc8547d_int_flag[i].mask;
		}
	}

check_out:
	return dump_err;
}

static bool sc8547_voocphy_check_cp_int_happened(
	struct oplus_voocphy_manager *chip, bool *dump_reg, bool *send_info)
{
	int err_type = 0;
	bool err_happened = false;

	err_happened = sc8547_voocphy_check_cp_int_status(chip, &err_type, true);

	return err_happened;
}

static int sc8547_get_cp_error_type(struct oplus_voocphy_manager *chip, int *err_type)
{
	bool err_happened = false;

	if (NULL == chip || NULL == err_type) {
		chg_err("chip or err_type is NULL\n");
		return -EINVAL;
	}

	err_happened = sc8547_voocphy_check_cp_int_status(chip, err_type, false);
	if (err_happened && *err_type > 0) {
		chg_info(" [%d, %d]\n", err_happened, *err_type);
		return 0;
	} else {
		return 1;
	}
}

#define TRACK_LOCAL_T_NS_TO_S_THD		1000000000
#define TRACK_UPLOAD_COUNT_MAX			10
#define TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD	(24 * 3600)
#define REASON_LENGTH_MAX			1024
#define ERR_LENGTH_MAX				64
#define DUMP_LENGTH_MAX				512
static int sc8547_track_get_local_time_s(void)
{
	int local_time_s;

	local_time_s = local_clock() / TRACK_LOCAL_T_NS_TO_S_THD;
	return local_time_s;
}

static int sc8547_get_int_reg_info(struct oplus_voocphy_manager *chip, char *dump_info, int len)
{
	int index = 0;

	if (!chip || !dump_info)
		return 0;

	index += scnprintf(&(dump_info[index]), len - index,
		"6/9/D/F:[0x%02x, 0x%02x, 0x%02x, 0x%02x]6~F master[0x%02x 0x%02x 0x%02x 0x%02x 0x%02x]"
		"[0x%02x 0x%02x 0x%02x 0x%02x 0x%02x]",
		chip->int_column_pre[0], chip->int_column_pre[1], chip->int_column_pre[2], chip->int_column_pre[3],
		chip->reg_dump[0], chip->reg_dump[1], chip->reg_dump[2], chip->reg_dump[3], chip->reg_dump[4],
		chip->reg_dump[5], chip->reg_dump[6], chip->reg_dump[7], chip->reg_dump[8], chip->reg_dump[9]);

	return index;
}

static int sc8547_track_upload_cp_err_info(struct oplus_voocphy_manager *chip, int err_type)
{
	int index = 0;
	int curr_time;
	static int upload_count = 0;
	static int pre_upload_time = 0;
	char temp_str[REASON_LENGTH_MAX] = {0};
	struct oplus_mms *err_topic;
	struct mms_msg *msg = NULL;
	int rc = 0;
	char dump_info[DUMP_LENGTH_MAX] = {0};

	if (NULL == chip) {
		chg_err("chip is NULL");
		return -EINVAL;
	}

	if (err_type <= TRACK_CP_ERR_DEFAULT) {
		chg_err("err_type is invalid");
		return -EINVAL;
	}

	err_topic = oplus_mms_get_by_name("error");
	if (!err_topic) {
		chg_err("error topic not found\n");
		return -EINVAL;
	}

	curr_time = sc8547_track_get_local_time_s();
	if (curr_time - pre_upload_time > TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD)
		upload_count = 0;

	if (upload_count > TRACK_UPLOAD_COUNT_MAX) {
		chg_info("cp_err_uploading upload_count = %d > max %d, should return\n",
			 upload_count, TRACK_UPLOAD_COUNT_MAX);
		return 0;
	}

	upload_count++;
	pre_upload_time = sc8547_track_get_local_time_s();

	index += scnprintf(&(temp_str[index]), REASON_LENGTH_MAX - index, "$$device_id@@%s", "sc8547d");
	index += scnprintf(&(temp_str[index]),
		REASON_LENGTH_MAX - index, "$$err_scene@@sc8547d_cp_work_err");

	index += scnprintf(&(temp_str[index]),
		REASON_LENGTH_MAX - index,
		"$$err_reason@@%s", track_cp_device_error_str(err_type));
	index += scnprintf(&(temp_str[index]),
		REASON_LENGTH_MAX - index,
		"$$err_position@@%s", "main");

	sc8547_get_int_reg_info(chip, dump_info, sizeof(dump_info));
	index += scnprintf(&(temp_str[index]),
		REASON_LENGTH_MAX - index,
		"$$reg_info@@%s", dump_info);

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

static int sc8547_publish_ic_err_msg(int type, int sub_type, const char *format, ...)
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
		"[%s]-[%d]-[%d]:%s", "sc8547d", type, sub_type, buf);
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

static void sc8547_upload_i2c_err_info(struct sc8547d_device *chip, bool read, s32 *err_info)
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

	sc8547_publish_ic_err_msg(OPLUS_IC_ERR_I2C, 0, "%s", buf);
	kfree(buf);
}

static ssize_t sc8547_show_registers(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct oplus_voocphy_manager *chip = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[300];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "sc8547");
	for (addr = 0x0; addr <= 0x3C; addr++) {
		if ((addr < 0x24) || (addr > 0x2B && addr < 0x33) ||
		    addr == 0x36 || addr == 0x3C) {
			ret = sc8547_read_byte(chip->client, addr, &val);
			if (ret == 0) {
				len = snprintf(tmpbuf, PAGE_SIZE - idx,
				               "Reg[%.2X] = 0x%.2x\n", addr, val);
				memcpy(&buf[idx], tmpbuf, len);
				idx += len;
			}
		}
	}

	return idx;
}

static ssize_t sc8547_store_register(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct oplus_voocphy_manager *chip = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg <= 0x3C)
		sc8547_write_byte(chip->client, (unsigned char)reg,
				  (unsigned char)val);

	return count;
}

static DEVICE_ATTR(registers, 0660, sc8547_show_registers,
		   sc8547_store_register);

static ssize_t sc8547d_track_reg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct oplus_voocphy_manager *voocphy_msg = dev_get_drvdata(dev);
	struct sc8547d_device *chip;

	if (!buf) {
		chg_err("buf is NULL\n");
		return -EINVAL;
	}

	if (!voocphy_msg) {
		chg_err("voocphy_msg is NULL\n");
		return -EINVAL;
	}
	chip = voocphy_msg->priv_data;
	if (!chip) {
		chg_err("sc8547d_device is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "6/9/d/f/en[0x%x, 0x%x, 0x%x, 0x%x, 0x%x]\n", chip->cp_reg_track[0],
		chip->cp_reg_track[1], chip->cp_reg_track[2], chip->cp_reg_track[3], chip->cp_reg_track[4]);
}

static ssize_t sc8547d_track_reg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct oplus_voocphy_manager *voocphy_msg = dev_get_drvdata(dev);
	int track_buf[SC8547D_TRACK_NUM] = { 0 };
	struct sc8547d_device *chip;

	if (!buf) {
		chg_err("buf is NULL\n");
		return -EINVAL;
	}

	if (!voocphy_msg) {
		chg_err("voocphy_msg is NULL\n");
		return -EINVAL;
	}
	chip = voocphy_msg->priv_data;
	if (!chip) {
		chg_err("sc8547d_device is NULL\n");
		return -EINVAL;
	}

	if (sscanf(buf, "%x,%x,%x,%x,%x", &track_buf[0], &track_buf[1], &track_buf[2], &track_buf[3], &track_buf[4]) != 5) {
		chg_err("invalid buff %s\n", buf);
		return -EINVAL;
	}

	if (track_buf[0] >= 0xff || track_buf[1] >= 0xff || track_buf[2] >= 0xff || track_buf[3] >= 0xff) {
		chg_err("6/9/d/f/en[0x%x, 0x%x, 0x%x, 0x%x, 0x%x] invalid\n", track_buf[0], track_buf[1], track_buf[2],
			track_buf[3], track_buf[4]);
		return -EINVAL;
	}

	memcpy(chip->cp_reg_track, track_buf, sizeof(track_buf));

	chg_info("6/9/d/f/en[0x%x, 0x%x, 0x%x, 0x%x, 0x%x]\n", chip->cp_reg_track[0], chip->cp_reg_track[1],
		chip->cp_reg_track[2], chip->cp_reg_track[3], chip->cp_reg_track[4]);

	return count;
}
static DEVICE_ATTR(track_reg, 0660, sc8547d_track_reg_show, sc8547d_track_reg_store);

static int sc8547d_read_flags(struct sc8547d_device *chip)
{
	unsigned int err_flag = 0;
	int rc = 0;
	u8 flag_buf[SC8547D_FLAG_NUM] = { 0 };

	rc = sc8547d_read_data(chip, SC8547D_ADDR_GENERAL_INT_FLAG1, flag_buf,
			       SC8547D_FLAG_NUM);
	if (rc < 0) {
		chg_err("failed to read flag register\n");
		return -EBUSY;
	}
	memcpy(chip->ufcs_reg_dump, flag_buf, SC8547D_FLAG_NUM);

	if (flag_buf[0] & SC8547D_FLAG_ACK_RECEIVE_TIMEOUT)
		err_flag |= BIT(UFCS_RECV_ERR_ACK_TIMEOUT);
	if (flag_buf[0] & SC8547D_FLAG_MSG_TRANS_FAIL)
		err_flag |= BIT(UFCS_RECV_ERR_TRANS_FAIL);
	if (flag_buf[0] & SC8547D_FLAG_RX_OVERFLOW)
		err_flag |= BIT(UFCS_COMM_ERR_RX_OVERFLOW);
	if (flag_buf[0] & SC8547D_FLAG_DATA_READY)
		err_flag |= BIT(UFCS_RECV_ERR_DATA_READY);
	if (flag_buf[0] & SC8547D_FLAG_SENT_PACKET_COMPLETE)
		err_flag |= BIT(UFCS_RECV_ERR_SENT_CMP);

	if (flag_buf[1] & SC8547D_FLAG_HARD_RESET)
		err_flag |= BIT(UFCS_HW_ERR_HARD_RESET);
	if (flag_buf[1] & SC8547D_FLAG_CRC_ERROR)
		err_flag |= BIT(UFCS_COMM_ERR_CRC_ERR);
	if (flag_buf[1] & SC8547D_FLAG_STOP_ERROR)
		err_flag |= BIT(UFCS_COMM_ERR_STOP_ERR);
	if (flag_buf[1] & SC8547D_FLAG_START_FAIL)
		err_flag |= BIT(UFCS_COMM_ERR_START_FAIL);
	if (flag_buf[1] & SC8547D_FLAG_LENGTH_ERROR)
		err_flag |= BIT(UFCS_COMM_ERR_RX_LEN_ERR);
	if (flag_buf[1] & SC8547D_FLAG_DATA_BYTE_TIMEOUT)
		err_flag |= BIT(UFCS_COMM_ERR_BYTE_TIMEOUT);
	if (flag_buf[1] & SC8547D_FLAG_TRAINING_BYTE_ERROR)
		err_flag |= BIT(UFCS_COMM_ERR_TRAINING_ERR);
	if (flag_buf[1] & SC8547D_FLAG_BAUD_RATE_ERROR)
		err_flag |= BIT(UFCS_COMM_ERR_BAUD_RATE_ERR);

	if (flag_buf[2] & SC8547D_FLAG_BAUD_RATE_CHANGE)
		err_flag |= BIT(UFCS_COMM_ERR_BAUD_RATE_CHANGE);
	if (flag_buf[2] & SC8547D_FLAG_BUS_CONFLICT)
		err_flag |= BIT(UFCS_COMM_ERR_BUS_CONFLICT);
	chip->ufcs->err_flag_save = err_flag;

	if (chip->ufcs->handshake_state == UFCS_HS_WAIT) {
		if ((flag_buf[0] & SC8547D_FLAG_HANDSHAKE_SUCCESS) &&
		    !(flag_buf[0] & SC8547D_FLAG_HANDSHAKE_FAIL)) {
			chip->ufcs->handshake_state = UFCS_HS_SUCCESS;
		 } else if (flag_buf[0] & SC8547D_FLAG_HANDSHAKE_FAIL) {
			chip->ufcs->handshake_state = UFCS_HS_FAIL;
		 }
	}
	chg_info("[0x%x, 0x%x, 0x%x], err_flag=0x%x\n", flag_buf[0], flag_buf[1], flag_buf[2],
		 err_flag);

	return ufcs_set_error_flag(chip->ufcs, err_flag);
}

static int sc8547d_ufcs_init(struct ufcs_dev *ufcs)
{
	return 0;
}

static int sc8547d_ufcs_write_msg(struct ufcs_dev *ufcs, unsigned char *buf, int len)
{
	struct sc8547d_device *chip = ufcs->drv_data;
	int rc;

	rc = sc8547d_write_byte(chip, SC8547D_ADDR_TX_LENGTH, len);
	if (rc < 0) {
		chg_err("write tx buf len error, rc=%d\n", rc);
		return rc;
	}
	rc = sc8547d_write_data(chip, SC8547D_ADDR_TX_BUFFER0, len, buf);
	if (rc < 0) {
		chg_err("write tx buf error, rc=%d\n", rc);
		return rc;
	}
	rc = sc8547d_write_bit_mask(chip, SC8547D_ADDR_UFCS_CTRL1,
		SC8547D_MASK_SND_CMP, SC8547D_CMD_SND_CMP);
	if (rc < 0) {
		chg_err("write tx buf send cmd error, rc=%d\n", rc);
		return rc;
	}

	return rc;
}

static int sc8547d_ufcs_read_msg(struct ufcs_dev *ufcs, unsigned char *buf, int len)
{
	struct sc8547d_device *chip = ufcs->drv_data;
	u8 rx_buf_len;
	int rc;

	rc = sc8547d_read_byte(chip, SC8547D_ADDR_RX_LENGTH, &rx_buf_len);
	if (rc < 0) {
		chg_err("can't read rx buf len, rc=%d\n", rc);
		return rc;
	}
	if (rx_buf_len > len) {
		chg_err("rx_buf_len = %d, limit to %d\n", rx_buf_len, len);
		rx_buf_len = len;
	}

	rc = sc8547d_read_data(chip, SC8547D_ADDR_RX_BUFFER0, buf, rx_buf_len);
	if (rc < 0) {
		chg_err("can't read rx buf, rc=%d\n", rc);
		return rc;
	}

	return (int)rx_buf_len;
}

static int sc8547d_ufcs_handshake(struct ufcs_dev *ufcs)
{
	struct sc8547d_device *chip = ufcs->drv_data;
	int rc;

	chg_info("ufcs handshake\n");
	rc = sc8547d_write_bit_mask(chip, SC8547D_ADDR_UFCS_CTRL1,
				    SC8547D_MASK_EN_HANDSHAKE,
				    SC8547D_CMD_EN_HANDSHAKE);
	if (rc < 0)
		chg_err("send handshake error, rc=%d\n", rc);

	return rc;
}

static int sc8547d_ufcs_source_hard_reset(struct ufcs_dev *ufcs)
{
	struct sc8547d_device *chip = ufcs->drv_data;
	int rc;
	int retry_count = 0;

retry:
	retry_count++;
	if (retry_count > UFCS_HARDRESET_RETRY_CNTS) {
		chg_err("send hard reset, retry count over!\n");
		return -EBUSY;
	}

	rc = sc8547d_write_bit_mask(chip, SC8547D_ADDR_UFCS_CTRL1,
				SEND_SOURCE_HARDRESET,
				SEND_SOURCE_HARDRESET);
	if (rc < 0) {
		chg_err("I2c send handshake error\n");
		goto retry;
	}

	msleep(100);
	return 0;
}

static int sc8547d_ufcs_cable_hard_reset(struct ufcs_dev *ufcs)
{
	return 0;
}

static int sc8547d_ufcs_set_baud_rate(struct ufcs_dev *ufcs, enum ufcs_baud_rate baud)
{
	struct sc8547d_device *chip = ufcs->drv_data;
	int rc;

	rc = sc8547d_write_bit_mask(chip, SC8547D_ADDR_UFCS_CTRL1,
				FLAG_BAUD_RATE_VALUE,
				(baud << FLAG_BAUD_NUM_SHIFT));
	if (rc < 0)
		chg_err("set baud rate error, rc=%d\n", rc);

	return rc;
}

static int sc8547d_ufcs_enable(struct ufcs_dev *ufcs)
{
	struct sc8547d_device *chip = ufcs->drv_data;
	u8 addr_buf[SC8547D_ENABLE_REG_NUM] = { SC8547D_ADDR_DPDM_CTRL,
						SC8547D_ADDR_UFCS_CTRL1,
						SC8547D_ADDR_UFCS_CTRL2,
						SC8547D_ADDR_UFCS_INT_MASK1,
						SC8547D_ADDR_UFCS_INT_MASK2 };
	u8 cmd_buf[SC8547D_ENABLE_REG_NUM] = {
		SC8547D_CMD_DPDM_EN,	  SC8547D_CMD_EN_CHIP,
		SC8547D_CMD_CLR_TX_RX,    SC8547D_CMD_MASK_ACK_TIMEOUT,
		SC8547D_MASK_TRANING_BYTE_ERROR
	};
	int i, rc;

	chip->rested = true;
	sc8547_reg_reset(chip, true);
	msleep(10);
	sc8547_init_device(chip);
	for (i = 0; i < SC8547D_ENABLE_REG_NUM; i++) {
		rc = sc8547d_write_byte(chip, addr_buf[i], cmd_buf[i]);
		if (rc < 0) {
			chg_err("write i2c failed!\n");
			return rc;
		}
	}
	chip->ufcs_enable = true;
	/* hardreset signal to 1900us ; ack timeout to 10ms + rx_FRAME_TIME */
	sc8547_write_byte(chip->client, SC8547_REG_CC, 0x80);

	rc = sc8547_write_byte(chip->client, SC8547_REG_09, SC8547_WATCHDOG_5S); /* WD:5000ms */
	if (rc < 0) {
		chg_err("failed to set sc8547d_ufcs_enable (%d)\n", rc);
		return rc;
	}

	return 0;
}

static int sc8547d_ufcs_disable(struct ufcs_dev *ufcs)
{
	struct sc8547d_device *chip = ufcs->drv_data;
	int rc;

	chip->ufcs_enable = false;
	chip->rested = false;
	rc = sc8547d_write_byte(chip, SC8547D_ADDR_UFCS_CTRL1,
				SC8547D_CMD_DIS_CHIP);
	if (rc < 0) {
		chg_err("write i2c failed\n");
		return rc;
	}

	rc = sc8547_write_byte(chip->client, SC8547_REG_09, SC8547_WATCHDOG_DIS); /* dsiable wdt */
	if (rc < 0) {
		chg_err("failed to set sc8547d_ufcs_disable (%d)\n", rc);
		return rc;
	}

	return 0;
}

static int sc8547d_get_wdt_reg_by_time(unsigned int time_ms, u8 *reg)
{
	if (time_ms == 0) {
		*reg = SC8547_WATCHDOG_DIS;
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
		chg_err("sc8547 watchdog not support %dms(>30s)\n", time_ms);
		return -EINVAL;
	}

	return 0;
}

static int sc8547d_ufcs_cp_watchdog_config(struct ufcs_dev *ufcs, unsigned int time_ms)
{
	struct sc8547d_device *chip = ufcs->drv_data;
	u8 en = 0;
	int rc = 0;

	rc = sc8547d_get_wdt_reg_by_time(time_ms, &en);
	if (rc < 0)
		return rc;

	if (en == SC8547_WATCHDOG_DIS) {
		rc = sc8547_update_bits(chip->client, SC8547_REG_09,
		                         SC8547_WATCHDOG_MASK, SC8547_WATCHDOG_DIS); /* dsiable wdt */
		chg_info("watchdog_config disable (%d) ok!\n", en);
	} else {
		rc = sc8547_update_bits(chip->client, SC8547_REG_09,
		                         SC8547_WATCHDOG_MASK, en);
		chg_info("watchdog_config set (%d) ok!\n", en);
	}
	if (rc < 0) {
		chg_err("failed to cp_watchdog_config(%d)\n", rc);
		return rc;
	}

	return 0;
}

static u8 sc8547d_get_vbus_status(struct sc8547d_device *dev)
{
	int ret = 0;
	u8 value = 0;
	u8 vbus_status;

	if (!dev) {
		chg_err("dev is null\n");
		return VOOC_VBUS_INVALID;
	}

	ret = sc8547_read_byte(dev->client, SC8547_REG_06, &value);
	if (ret < 0) {
		chg_err("failed to get vbus status(%d)\n", ret);
		return VOOC_VBUS_INVALID;
	}

	if (value & SC8547_VBUS_ERRORHI_STAT_MASK)
		vbus_status = VOOC_VBUS_HIGH;
	else if (value & SC8547_VBUS_ERRORLO_STAT_MASK)
		vbus_status = VOOC_VBUS_LOW;
	else
		vbus_status = VOOC_VBUS_NORMAL;

	chg_info("reg06=0x%02x, vbus status: %d\n", value, vbus_status);

	return vbus_status;
}

static u8 sc8547d_voocphy_get_vbus_status(struct oplus_voocphy_manager *chip)
{
	struct sc8547d_device *dev = chip->priv_data;
	if (dev == NULL) {
		chg_err("sc8547d chip is NULL\n");
		return VOOC_VBUS_INVALID;
	}

	return sc8547d_get_vbus_status(dev);
}

static void sc8547_create_device_node(struct device *dev)
{
	int ret;

	ret = device_create_file(dev, &dev_attr_registers);
	if (ret && ret != -EEXIST)
		chg_err("create registers node failed, rc=%d\n", ret);

	ret = device_create_file(dev, &dev_attr_track_reg);
	if (ret && ret != -EEXIST)
		chg_err("create track_reg node failed, rc=%d\n", ret);
}

static void sc8547_remove_device_node(struct device *dev)
{
	if (!dev) {
		chg_err("device is NULL\n");
		return;
	}

	device_remove_file(dev, &dev_attr_registers);
	device_remove_file(dev, &dev_attr_track_reg);
}

static struct oplus_voocphy_operations oplus_sc8547_ops = {
	.hardware_init = sc8547_voocphy_hardware_init,
	.hw_setting = sc8547_voocphy_hw_setting,
	.init_vooc = sc8547_voocphy_init_vooc,
	.set_predata = sc8547_voocphy_set_predata,
	.set_txbuff = sc8547_voocphy_set_txbuff,
	.get_adapter_info = sc8547_voocphy_get_adapter_info,
	.update_data = sc8547_voocphy_update_data,
	.get_chg_enable = sc8547_voocphy_get_chg_enable,
	.set_chg_enable = sc8547_voocphy_set_chg_enable,
	.reset_voocphy = sc8547_voocphy_reset_voocphy,
	.reactive_voocphy = sc8547_voocphy_reactive_voocphy,
	.send_handshake = sc8547_voocphy_send_handshake,
	.get_cp_vbat = sc8547_voocphy_get_cp_vbat,
	.get_cp_vbus = sc8547_voocphy_get_cp_vbus,
	.get_int_value = sc8547_voocphy_get_int_value,
	.get_adc_enable = sc8547_voocphy_get_adc_enable,
	.set_adc_enable = sc8547_voocphy_set_adc_enable,
	.get_ichg = sc8547_voocphy_get_cp_ichg,
	.set_pd_svooc_config = sc8547_voocphy_set_pd_svooc_config,
	.get_pd_svooc_config = sc8547_voocphy_get_pd_svooc_config,
	.get_voocphy_enable = sc8547_voocphy_get_voocphy_enable,
	.dump_voocphy_reg = sc8547_voocphy_dump_reg_in_err_issue,
	.check_cp_int_happened = sc8547_voocphy_check_cp_int_happened,
	.dual_chan_buck_set_ucp = sc8547_dual_chan_buck_set_ucp,
	.upload_cp_error = sc8547_track_upload_cp_err_info,
	.get_cp_error_type = sc8547_get_cp_error_type,
	.set_sstimeout_ucp_enable = sc8547d_voocphy_set_sstimeout_ucp_enable,
	.get_vbus_status = sc8547d_voocphy_get_vbus_status,
	.set_usb_dischg_enable = sc8547d_cp_manual_enable,
};

static int sc8547_slave_hw_setting(struct oplus_voocphy_manager *voocphy_mg, int reason)
{
	struct oplus_voocphy_manager *voocphy;

	if (!voocphy_mg) {
		chg_err("voocphy_mg is null exit\n");
		return -EINVAL;
	}

	voocphy = i2c_get_clientdata(voocphy_mg->slave_client);
	return sc8547_voocphy_hw_setting(voocphy, reason);
}

static int sc8547_slave_init_vooc(struct oplus_voocphy_manager *voocphy_mg)
{
	struct oplus_voocphy_manager *voocphy;

	if (!voocphy_mg) {
		chg_err("voocphy_mg is null exit\n");
		return -EINVAL;
	}

	voocphy = i2c_get_clientdata(voocphy_mg->slave_client);
	return sc8547_voocphy_init_vooc(voocphy);
}

static void sc8547_slave_update_data(struct oplus_voocphy_manager *voocphy_mg)
{
	u8 data_block[2] = {0};
	u8 int_flag = 0;
	s32 ret = 0;
	s32 err_info[2] = { 0 };
	struct oplus_voocphy_manager *voocphy;
	struct sc8547d_device *chip;

	if (!voocphy_mg) {
		chg_err("voocphy_mg is null exit\n");
		return;
	}

	if (!voocphy_mg->slave_client) {
		chg_err("voocphy_mg->slave_client is null\n");
		return;
	}

	voocphy = i2c_get_clientdata(voocphy_mg->slave_client);
	if (!voocphy) {
		chg_err("voocphy is null exit\n");
		return;
	}

	chip = voocphy->priv_data;
	if (chip == NULL) {
		chg_err("sc8547d chip is NULL\n");
		return;
	}

	/*int_flag*/
	sc8547_read_byte(chip->client, SC8547_REG_0F, &int_flag);

	/*parse data_block for improving time of interrupt*/
	mutex_lock(&chip->adc_freeze_lock);
	sc8547_update_bits(chip->client, SC8547_REG_11, SC8547_ADC_FREEZE_MASK, SC8547_ADC_FREEZE_MASK);
	ret = sc8547d_read_data(chip, SC8547_REG_13, data_block, 2);
	sc8547_update_bits(chip->client, SC8547_REG_11, SC8547_ADC_FREEZE_MASK, 0);
	mutex_unlock(&chip->adc_freeze_lock);
	if (ret < 0) {
		err_info[0] = SC8547_REG_13;
		err_info[1] = ret;
		sc8547_i2c_error(chip, true, true, err_info);
		chg_err("sc8547_update_data read vsys vbat error \n");
	} else {
		sc8547_i2c_error(chip, false, true, err_info);
	}

	voocphy_mg->slave_cp_ichg = (((data_block[0] & SC8547_IBUS_POL_H_MASK) << 8) | data_block[1]) * SC8547_IBUS_ADC_LSB;
	chg_info("slave cp_ichg = %d int_flag = %d", voocphy_mg->slave_cp_ichg, int_flag);
}

static int sc8547_slave_get_chg_enable(struct oplus_voocphy_manager *voocphy_mg, u8 *data)
{
	struct oplus_voocphy_manager *voocphy;

	if (!voocphy_mg) {
		chg_err("voocphy_mg is null exit\n");
		return -EINVAL;
	}

	voocphy = i2c_get_clientdata(voocphy_mg->slave_client);
	return sc8547_voocphy_get_chg_enable(voocphy, data);
}

static int sc8547_slave_set_chg_enable(struct oplus_voocphy_manager *voocphy_mg, bool enable)
{
	int rc = 0;
	struct oplus_voocphy_manager *voocphy;
	struct sc8547d_device *chip;
	u8 value = (SC8547_CHG_ENABLE << SC8547_CHG_EN_SHIFT) | SC8547_FSW_SET_550KHZ;

	if (!voocphy_mg) {
		chg_err("voocphy_mg is null exit\n");
		return -EINVAL;
	}

	voocphy = i2c_get_clientdata(voocphy_mg->slave_client);
	if (!voocphy) {
		chg_err("voocphy is null exit\n");
		return -EINVAL;
	}
	chip = voocphy->priv_data;
	if (chip == NULL) {
		chg_err("sc8547d chip is NULL\n");
		return -ENODEV;
	}

	if (enable) {
		if (chip->ic_sc8547d)
			value = (SC8547_CHG_ENABLE << SC8547_CHG_EN_SHIFT) | SC8547_FSW_SET_350KHZ;
		else
			value = (SC8547_CHG_ENABLE << SC8547_CHG_EN_SHIFT) | SC8547_FSW_SET_550KHZ;
	} else {
		if (chip->ic_sc8547d)
			value = (SC8547_CHG_DISABLE << SC8547_CHG_EN_SHIFT) | SC8547_FSW_SET_350KHZ;
		else
			value = (SC8547_CHG_DISABLE << SC8547_CHG_EN_SHIFT) | SC8547_FSW_SET_550KHZ;
	}
	chip->chg_enabled = enable;
	rc = sc8547_write_byte(voocphy_mg->slave_client, SC8547_REG_07, value);
	chg_err(" enable  = %d, value = 0x%x!\n", enable, value);

	return rc;
}

static int sc8547_slave_get_ichg(struct oplus_voocphy_manager *voocphy_mg)
{
	struct oplus_voocphy_manager *voocphy;
	struct sc8547d_device *chip;

	if (!voocphy_mg) {
		chg_err("voocphy_mg is null exit\n");
		return -EINVAL;
	}

	voocphy = i2c_get_clientdata(voocphy_mg->slave_client);
	if (!voocphy) {
		chg_err("voocphy is null exit\n");
		return -EINVAL;
	}
	chip = voocphy->priv_data;
	if (chip == NULL) {
		chg_err("sc8547d chip is NULL\n");
		return -ENODEV;
	}

	if (oplus_chg_get_vooc_charging()) {
		if (chip->chg_enabled)
			return voocphy_mg->slave_cp_ichg;
		else
			return 0;
	}

	return sc8547_voocphy_get_cp_ichg(voocphy);
}

static int sc8547_slave_reset_voocphy(struct oplus_voocphy_manager *voocphy_mg)
{
	struct oplus_voocphy_manager *voocphy;
	struct sc8547d_device *chip;

	if (!voocphy_mg) {
		chg_err("voocphy_mg is null exit\n");
		return -EINVAL;
	}

	voocphy = i2c_get_clientdata(voocphy_mg->slave_client);
	chip = voocphy->priv_data;
	if (chip == NULL) {
		chg_err("sc8547d chip is NULL\n");
		return -ENODEV;
	}

	sc8547_slave_set_chg_enable(voocphy_mg, false);
	if(chip->ic_sc8547d) {
		/* sc8547b need disable WDT when exit charging, to avoid after WDT time out.
		IF time out, sc8547b will trigger interrupt frequently.
		in addition, sc8547 and sc8547b WDT will disable when disable CP */
		sc8547_update_bits(voocphy_mg->slave_client, SC8547_REG_09,
				SC8547_WATCHDOG_MASK, SC8547_WATCHDOG_DIS);
	}
	sc8547_slave_hw_setting(voocphy_mg, SETTING_REASON_RESET);

	return VOOCPHY_SUCCESS;
}

static int sc8547_slave_get_adc_enable(struct oplus_voocphy_manager *voocphy_mg, u8 *data)
{
	struct oplus_voocphy_manager *voocphy;

	if (!voocphy_mg) {
		chg_err("voocphy_mg is null exit\n");
		return -EINVAL;
	}

	voocphy = i2c_get_clientdata(voocphy_mg->slave_client);
	return sc8547_voocphy_get_adc_enable(voocphy, data);
}

static int sc8547_slave_set_adc_enable(struct oplus_voocphy_manager *voocphy_mg, bool enable)
{
	struct oplus_voocphy_manager *voocphy;

	if (!voocphy_mg) {
		chg_err("voocphy_mg is null exit\n");
		return -EINVAL;
	}

	voocphy = i2c_get_clientdata(voocphy_mg->slave_client);
	return sc8547_voocphy_set_adc_enable(voocphy, enable);
}

static int sc8547_slave_get_cp_status(struct oplus_voocphy_manager *voocphy_mg)
{
	u8 data_reg06, data_reg07;
	int ret_reg06, ret_reg07;

	if (!voocphy_mg) {
		chg_err("voocphy_mg is null exit\n");
		return -EINVAL;
	}

	ret_reg06 = sc8547_read_byte(voocphy_mg->slave_client, SC8547_REG_06, &data_reg06);
	ret_reg07 = sc8547_read_byte(voocphy_mg->slave_client, SC8547_REG_07, &data_reg07);

	if (ret_reg06 < 0 || ret_reg07 < 0) {
		chg_err("SC8547_REG_06 or SC8547_REG_07 err\n");
		return 0;
	}
	data_reg06 = data_reg06 & SC8547_CP_SWITCHING_STAT_MASK;
	data_reg06 = data_reg06 >> 2;
	data_reg07 = data_reg07 >> 7;

	chg_err("reg06 = %d reg07 = %d\n", data_reg06, data_reg07);

	if (data_reg06 == 1 && data_reg07 == 1) {
		return 1;
	} else {
		return 0;
	}
}

static int sc8547_slave_get_voocphy_enable(struct oplus_voocphy_manager *voocphy_mg, u8 *data)
{
	struct oplus_voocphy_manager *voocphy;

	if (!voocphy_mg) {
		chg_err("voocphy_mg is null exit\n");
		return -EINVAL;
	}

	voocphy = i2c_get_clientdata(voocphy_mg->slave_client);
	return sc8547_voocphy_get_voocphy_enable(voocphy, data);
}

static void sc8547_slave_dump_reg_in_err_issue(struct oplus_voocphy_manager *voocphy_mg)
{
	struct oplus_voocphy_manager *voocphy;

	if (!voocphy_mg) {
		chg_err("voocphy_mg is null exit\n");
		return;
	}

	voocphy = i2c_get_clientdata(voocphy_mg->slave_client);
	sc8547_voocphy_dump_reg_in_err_issue(voocphy);
}

static int sc8547_slave_reset_ovp(struct oplus_voocphy_manager *voocphy_mg)
{
	if (voocphy_mg->ovp_ctrl_cpindex == SLAVE_CP_ID) {
		sc8547_write_byte(voocphy_mg->slave_client, SC8547_REG_02, 0x01);
		pr_err("sc8547_slave_reset_ovp VAC_OVP to 6.5v\n");
	}
	return 0;
}

static struct oplus_voocphy_operations oplus_sc8547_slave_ops = {
	.hw_setting		= sc8547_slave_hw_setting,
	.init_vooc		= sc8547_slave_init_vooc,
	.update_data		= sc8547_slave_update_data,
	.get_chg_enable		= sc8547_slave_get_chg_enable,
	.set_chg_enable		= sc8547_slave_set_chg_enable,
	.get_ichg		= sc8547_slave_get_ichg,
	.reset_voocphy      	= sc8547_slave_reset_voocphy,
	.get_adc_enable		= sc8547_slave_get_adc_enable,
	.set_adc_enable		= sc8547_slave_set_adc_enable,
	.get_cp_status 		= sc8547_slave_get_cp_status,
	.get_voocphy_enable 	= sc8547_slave_get_voocphy_enable,
	.dump_voocphy_reg	= sc8547_slave_dump_reg_in_err_issue,
	.reset_voocphy_ovp	= sc8547_slave_reset_ovp,
};

static int sc8547d_retrieve_flags(struct ufcs_dev *ufcs)
{
	int rc = 0;
	int err_type = 0;
	struct sc8547d_device *chip = ufcs->drv_data;
	struct oplus_voocphy_manager *voocphy = chip->voocphy;

	rc = sc8547_voocphy_get_int_value(voocphy);
	rc = sc8547_get_cp_error_type(voocphy, &err_type);
	if (rc == 0)
		rc = sc8547_track_upload_cp_err_info(voocphy, err_type);

	return rc;
}

#define SC8547D_DP_20K_PD_EN_MASK BIT(4)
#define SC8547D_DM_20K_PD_EN_MASK BIT(5)
static int sc8547d_cp_reset_dpdm(struct ufcs_dev *ufcs)
{
	struct sc8547d_device *chip = ufcs->drv_data;
	int rc = 0;
	int ufcs_disable_rc = 0;
	static bool first = true;
	u8 value = 0;
	bool ufcs_enable_success = false;

	if (!first && !oplus_chg_get_fcs_support_flags())
		return 0;
	first = false;

	if (chip == NULL) {
		chg_err("sc8547d chip is NULL\n");
		return -ENODEV;
	}

	// SC8547D_ADDR_UFCS_CTRL1 = 0x40
	rc = sc8547_read_byte(chip->client, SC8547D_ADDR_UFCS_CTRL1, &value);
	if (rc < 0) {
		chg_err("[%s] read SC8547D_ADDR_UFCS_CTRL1 error, rc=%d\n",
			chip->dev->of_node->name, rc);
		goto reset_dpdm_err;
	}

	if (!(value & SC8547D_CMD_EN_CHIP)) {
		rc = sc8547d_ufcs_enable(chip->ufcs);
		if (rc < 0) {
			chg_err("sc8547d_ufcs_enable failed, rc=%d\n", rc);
			goto reset_dpdm_err;
		}
		ufcs_enable_success = true;
	}

	rc = sc8547_read_byte(chip->client, SC8547_REG_21, &value);
	if (rc < 0) {
		chg_err("[%s] read SC8547_REG_21 error, rc=%d\n", chip->dev->of_node->name, rc);
		goto reset_dpdm_err;
	}

	rc = sc8547_write_byte(chip->client, SC8547_REG_21,
		value | (SC8547D_DP_20K_PD_EN_MASK | SC8547D_DM_20K_PD_EN_MASK));
	if(rc < 0) {
		chg_err("dpdm pull down failed, rc=%d\n", rc);
		goto reset_dpdm_err;
	}
	chg_info("dpdm pull down\n");

	msleep(30);

	rc = sc8547_write_byte(chip->client, SC8547_REG_21,
		value & ~(SC8547D_DP_20K_PD_EN_MASK | SC8547D_DM_20K_PD_EN_MASK));
	if(rc < 0) {
		chg_err("dpdm pull up failed, rc=%d\n", rc);
		goto reset_dpdm_err;
	}
	chg_info("dpdm pull up\n");

reset_dpdm_err:
	if (ufcs_enable_success) {
		ufcs_disable_rc = sc8547d_ufcs_disable(chip->ufcs);
		if (ufcs_disable_rc < 0)
			chg_err("sc8547d_ufcs_disable failed, ufcs_disable_rc=%d\n",
				ufcs_disable_rc);
		if (rc >= 0)
			rc = ufcs_disable_rc;
	}

	return rc;
}

static struct ufcs_dev_ops ufcs_ops = {
	.init = sc8547d_ufcs_init,
	.write_msg = sc8547d_ufcs_write_msg,
	.read_msg = sc8547d_ufcs_read_msg,
	.handshake = sc8547d_ufcs_handshake,
	.source_hard_reset = sc8547d_ufcs_source_hard_reset,
	.cable_hard_reset = sc8547d_ufcs_cable_hard_reset,
	.set_baud_rate = sc8547d_ufcs_set_baud_rate,
	.enable = sc8547d_ufcs_enable,
	.disable = sc8547d_ufcs_disable,
	.watchdog_config = sc8547d_ufcs_cp_watchdog_config,
	.retrieve_flags = sc8547d_retrieve_flags,
	.reset_dpdm = sc8547d_cp_reset_dpdm,
};

static int sc8547_charger_choose(struct sc8547d_device *chip)
{
	int ret;

	if (!oplus_voocphy_chip_is_null()) {
		chg_err("oplus_voocphy_chip already exists!");
		return 0;
	} else {
		ret = i2c_smbus_read_byte_data(chip->client, 0x07);
		chg_info("0x07 = %d\n", ret);
		if (ret < 0) {
			chg_err("i2c communication fail");
			return -EPROBE_DEFER;
		}

		return 1;
	}
}

static int sc8547_slave_charger_choose(struct sc8547d_device *chip)
{
	int ret;

	if (oplus_voocphy_chip_is_null()) {
		pr_err("oplus_voocphy_chip null, will do after master cp init!");
		return -EPROBE_DEFER;
	} else {
		ret = i2c_smbus_read_byte_data(chip->client, 0x07);
		pr_err("0x07 = %d\n", ret);
		if (ret < 0) {
			pr_err("i2c communication fail");
			if (oplus_voocphy_slave_chip_is_null()) {
				return -EPROBE_DEFER;
			} else {
				chg_err("not use sc8547d slave");
				return ret;
			}
		}
		else
			return 1;
	}
}

static void sc8547d_ufcs_event_handler(struct sc8547d_device *chip)
{
	/* set awake */
	sc8547d_read_flags(chip);
	ufcs_msg_handler(chip->ufcs);
}

static irqreturn_t sc8547_interrupt_handler(int irq, void *dev_id)
{
	struct sc8547d_device *chip = dev_id;
	struct oplus_voocphy_manager *voocphy = chip->voocphy;

	if (chip->use_ufcs_phy && chip->ufcs_enable) {
		sc8547d_ufcs_event_handler(chip);
		return IRQ_HANDLED;
	} else if (chip->use_vooc_phy && chip->voocphy_enable) {
		return oplus_voocphy_interrupt_handler(voocphy);
	}

	return IRQ_HANDLED;
}

static int sc8547_irq_gpio_init(struct oplus_voocphy_manager *chip)
{
	int rc;
	struct device_node *node = chip->dev->of_node;

	chip->irq_gpio = of_get_named_gpio(node, "oplus,irq_gpio", 0);
	if (!gpio_is_valid(chip->irq_gpio)) {
		chip->irq_gpio = of_get_named_gpio(node, "oplus_spec,irq_gpio", 0);
		if (!gpio_is_valid(chip->irq_gpio)) {
			chg_err("irq_gpio not specified, rc=%d\n", chip->irq_gpio);
			return chip->irq_gpio;
		}
	}
	rc = gpio_request(chip->irq_gpio, "irq_gpio");
	if (rc) {
		chg_err("unable to request gpio[%d]\n", chip->irq_gpio);
		return rc;
	}
	chg_info("irq_gpio = %d\n", chip->irq_gpio);

	chip->irq = gpio_to_irq(chip->irq_gpio);
	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		chg_err("get pinctrl fail\n");
		return -EINVAL;
	}

	chip->charging_inter_active =
	    pinctrl_lookup_state(chip->pinctrl, "charging_inter_active");
	if (IS_ERR_OR_NULL(chip->charging_inter_active)) {
		chg_err("failed to get the pinctrl state(%d)\n", __LINE__);
		return -EINVAL;
	}

	chip->charging_inter_sleep =
	    pinctrl_lookup_state(chip->pinctrl, "charging_inter_sleep");
	if (IS_ERR_OR_NULL(chip->charging_inter_sleep)) {
		chg_err("Failed to get the pinctrl state(%d)\n", __LINE__);
		return -EINVAL;
	}

	gpio_direction_input(chip->irq_gpio);
	pinctrl_select_state(chip->pinctrl, chip->charging_inter_active); /* no_PULL */
	rc = gpio_get_value(chip->irq_gpio);
	chg_info("irq_gpio = %d, irq_gpio_stat = %d\n", chip->irq_gpio, rc);

	return 0;
}

static int sc8547_irq_register(struct sc8547d_device *chip)
{
	struct oplus_voocphy_manager *voocphy = chip->voocphy;
	struct irq_desc *desc;
	struct cpumask current_mask;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
	cpumask_var_t cpu_highcap_mask;
#endif
	int ret;

	ret = sc8547_irq_gpio_init(voocphy);
	if (ret < 0) {
		chg_err("failed to irq gpio init(%d)\n", ret);
		return ret;
	}

	if (voocphy->irq) {
		ret = request_threaded_irq(voocphy->irq, NULL,
					   sc8547_interrupt_handler,
					   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					   "voocphy_irq", chip);
		if (ret < 0) {
			chg_err("request irq for irq=%d failed, ret =%d\n",
				voocphy->irq, ret);
			return ret;
		}
		chg_debug("request irq ok\n");
	}

	desc = irq_to_desc(voocphy->irq);
	if (desc == NULL) {
		free_irq(voocphy->irq, chip);
		chg_err("%s desc null\n", __func__);
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

static bool sc8547d_is_volatile_reg(struct device *dev, unsigned int reg)
{
	return true;
}
static void sc8547d_otg_enabled_work(struct work_struct *work)
{
	struct sc8547d_device *chip =
		container_of(work, struct sc8547d_device, otg_enabled_work);
	union mms_msg_data data = { 0 };

	oplus_mms_get_item_data(chip->wired_topic,  WIRED_ITEM_OTG_ENABLE, &data, false);
	chg_info("otg enable_value = %d\n", data.intval);
	if (data.intval == 0) {
		chip->otg_connected = false;
		if (!chip->always_otg_en)
			sc8547_update_bits(chip->client, SC8547D_ADDR_OTG_EN, SC8547D_OTG_EN_MASK, 0x0);
	}
}
static void sc8547d_ufcs_regdump_work(struct work_struct *work)
{
	struct sc8547d_device *chip =
		container_of(work, struct sc8547d_device, ufcs_regdump_work);
	struct mms_msg *topic_msg;
	char *buf;
	int rc;
	int i;
	size_t index = 0;

	buf = kzalloc(ERR_MSG_BUF, GFP_KERNEL);
	if (buf == NULL)
		return;

	for (i = 0; i < SC8547D_FLAG_NUM; i++)
		index += snprintf(buf + index, ERR_MSG_BUF, "0x%04x=%02x,",
			(SC8547D_ADDR_GENERAL_INT_FLAG1 + i), chip->ufcs_reg_dump[i]);
	if (index > 0)
		buf[index - 1] = 0;

	topic_msg =
		oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, ERR_ITEM_IC,
					"[%s]-[%d]-[%d]:$$reg_info@@%s",
					"ufcs-sc8547d",
					OPLUS_IC_ERR_UFCS, UFCS_ERR_REG_DUMP,
					buf);
	kfree(buf);
	if (topic_msg == NULL) {
		chg_err("alloc topic msg error\n");
		return;
	}

	rc = oplus_mms_publish_msg(chip->err_topic, topic_msg);
	if (rc < 0) {
		chg_err("publish error topic msg error, rc=%d\n", rc);
		kfree(topic_msg);
	}
}

#define SC8547D_CP_STATUS_REG_MAX	4
static u8 g_sc8547d_cp_status_reg[SC8547D_CP_STATUS_REG_MAX] = {
	0x06, 0x09, 0x0e, 0x0f
};

static void sc8547d_cp_regdump_work(struct work_struct *work)
{
	struct sc8547d_device *chip =
		container_of(work, struct sc8547d_device, cp_regdump_work);
	char *buf;
	int i;
	size_t index = 0;
	u8 data;
	int rc;

	buf = kzalloc(ERR_MSG_BUF, GFP_KERNEL);
	if (buf == NULL)
		return;

	for (i = 0; i < SC8547D_CP_STATUS_REG_MAX; i++) {
		data = 0;
		rc = sc8547d_read_data(chip, g_sc8547d_cp_status_reg[i], &data, 1);
		if (rc < 0) {
			chg_err("can't read 0x%02x buf, rc=%d\n", g_sc8547d_cp_status_reg[i], rc);
			continue;
		}
		index += snprintf(buf + index, ERR_MSG_BUF, "0x%02x=%02x,",
			g_sc8547d_cp_status_reg[i], data);
	}
	if (index > 0)
		buf[index - 1] = 0;

	chg_info(": %s\n", buf);
	kfree(buf);
}

static void sc8547d_err_subs_callback(struct mms_subscribe *subs,
				     enum mms_msg_type type, u32 id, bool sync)
{
	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case ERR_ITEM_UFCS:
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void sc8547d_subscribe_error_topic(struct oplus_mms *topic, void *prv_data)
{
	struct sc8547d_device *chip = prv_data;

	chip->err_topic = topic;
	chip->err_subs =
		oplus_mms_subscribe(chip->err_topic, chip,
				    sc8547d_err_subs_callback, "sc8547d");
	if (IS_ERR_OR_NULL(chip->err_subs)) {
		chg_err("subscribe error topic error, rc=%ld\n",
			PTR_ERR(chip->err_subs));
		return;
	}
}

static struct regmap_config sc8547d_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = SC8547D_MAX_REG,
	.cache_type = REGCACHE_RBTREE,
	.volatile_reg = sc8547d_is_volatile_reg,
};

static struct ufcs_config sc8547d_ufcs_config = {
	.check_crc = false,
	.reply_ack = false,
	.msg_resend = false,
	.handshake_hard_retry = true,
	.ic_vendor_id = SC8547D_VENDOR_ID,
};

static bool sc8547d_check_work_mode_support(enum oplus_cp_work_mode mode)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(g_cp_support_work_mode); i++) {
		if (g_cp_support_work_mode[i] == mode)
			return true;
	}
	return false;
}

static int sc8547d_cp_init(struct oplus_chg_ic_dev *ic_dev)
{
	struct sc8547d_device *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	chg_info("%s init\n", ic_dev->manu_name);

	vote(chip->disable_votable, OCP_VOTER, false, 0, false);
	vote(chip->disable_votable, UCP_VOTER, false, 0, false);
	vote(chip->disable_votable, IIC_VOTER, false, 0, false);
	vote(chip->disable_votable, PARENT_VOTER, false, 0, false);
	chip->error_reported = false;
	atomic_set(&chip->i2c_err_count, 0);

	return 0;
}

static int sc8547d_cp_exit(struct oplus_chg_ic_dev *ic_dev)
{
	struct sc8547d_device *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	chg_info("%s exit\n", ic_dev->manu_name);
	vote(chip->disable_votable, PARENT_VOTER, true, 1, false);
	schedule_work(&chip->cp_regdump_work);

	return 0;
}

static int sc8547d_cp_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	struct sc8547d_device *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	sc8547_voocphy_dump_reg_in_err_issue(chip->voocphy);
	return 0;
}

static int sc8547d_cp_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	return 0;
}

static int sc8547d_cp_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct sc8547d_device *chip;
	int rc;
	u8 data;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	if (chip->ufcs_enable) {
		chg_info("ufcs_enable, return!!!\n");
		return 0;
	}

	if (en) {
		sc8547_reg_reset(chip, true);
		sc8547_init_device(chip);
		sc8547_write_byte(chip->client, SC8547_REG_02, 0x01); /*VAC_OVP:12V*/
		sc8547_write_byte(chip->client, SC8547_REG_04, 0x71); /*VBUS_OVP:12V*/
		sc8547_write_byte(chip->client, SC8547_REG_05, 0xcf); /*IBUS_OCP:disable/5.7A,disable UCP*/
		sc8547_update_bits(chip->client, SC8547_REG_08, SC8547_SS_TIMEOUT_SET_MASK, SC8547_SS_TIMEOUT_DISABLE);
		sc8547_write_byte(chip->client, SC8547_REG_09, 0x10); /*WD:disable*/
		sc8547_update_bits(chip->client, SC8547_REG_11, SC8547_ADC_EN_MASK, SC8547_ADC_EN_MASK); /*ADC_CTRL:enable ADC*/
		sc8547_write_byte(chip->client, SC8547_REG_0D, 0x70); /*PMID2OUT_OVP_UVP:500mV,-100mV*/
		sc8547_write_byte(chip->client, SC8547_REG_0C, 0x02); /*CTRL4:force VAC_OK*/
		if (chip->use_slave_cp)
			sc8547_write_byte(chip->client, SC8547_REG_3C, 0x60); /* Disable VBUS_IN_RANGE */
		/*need 50ms delay between force VAC_OK and chg_enable*/
		msleep(50);
	}
	chg_info("[%s] set cp %s...\n", chip->dev->of_node->name, en ? "enable" : "disable");
	rc = sc8547_set_chg_enable(chip, en);
	if (rc < 0) {
		chg_err("[%s] set cp %s fail\n", chip->dev->of_node->name, en ? "enable" : "disable");
		return rc;
	}
	if (en) {
		msleep(500);
		sc8547_read_byte(chip->client, SC8547_REG_06, &data);
		chg_info("[%s] <~WPC~> cp status:0x%x.\n", chip->dev->of_node->name, data);
		if ((data & SC8547_CP_SWITCHING_STAT_MASK) != SC8547_CP_SWITCHING_STAT_MASK)
			return -EAGAIN;
	}

	return 0;
}

static int sc8547d_cp_wd_enable(struct oplus_chg_ic_dev *ic_dev, int timeout_ms)
{
	struct sc8547d_device *chip;
	u8 reg_val = 0;
	int ret = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	chg_info("[%s] set watchdog timeout to %dms\n", chip->dev->of_node->name, timeout_ms);

	ret = sc8547d_get_wdt_reg_by_time(timeout_ms, &reg_val);
	if (ret < 0)
		return ret;

	ret = sc8547_update_bits(chip->client, SC8547_REG_09,
				SC8547_WATCHDOG_MASK, reg_val);
	chg_info("[%s] set watchdog reg to 0x%02x ok!\n", chip->dev->of_node->name, reg_val);

	if (ret < 0) {
		chg_err("[%s] failed to set watchdog reg to 0x%02x, ret=%d\n",
			chip->dev->of_node->name, reg_val, ret);
		return ret;
	}

	return 0;
}

static int sc8547d_cp_hw_init(struct oplus_chg_ic_dev *ic_dev)
{
	struct sc8547d_device *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	if (chip->rested)
		return 0;

	sc8547_hardware_init(chip);
	chip->work_start = false;
	return 0;
}

static int sc8547d_cp_set_work_mode(struct oplus_chg_ic_dev *ic_dev, enum oplus_cp_work_mode mode)
{
	struct sc8547d_device *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	if (!sc8547d_check_work_mode_support(mode)) {
		chg_err("not supported work mode, mode=%d\n", mode);
		return -EINVAL;
	}

	if (mode == CP_WORK_MODE_BYPASS)
		rc = sc8547_vooc_hw_setting(chip, false);
	else
		rc = sc8547_svooc_hw_setting(chip, false);

	if (rc < 0)
		chg_err("[%s] set work mode to %d error\n", chip->dev->of_node->name, mode);

	return rc;
}

static int sc8547d_cp_get_work_mode(struct oplus_chg_ic_dev *ic_dev, enum oplus_cp_work_mode *mode)
{
	struct sc8547d_device *chip;
	u8 data;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	rc = sc8547_read_byte(chip->client, SC8547_REG_09, &data);
	if (rc < 0) {
		chg_err("[%s] read SC8547_REG_09 error, rc=%d\n", chip->dev->of_node->name, rc);
		return rc;
	}

	if (data & BIT(7))
		*mode = CP_WORK_MODE_BYPASS;
	else
		*mode = CP_WORK_MODE_2_TO_1;

	return 0;
}

static int sc8547d_cp_check_work_mode_support(struct oplus_chg_ic_dev *ic_dev, enum oplus_cp_work_mode mode)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	return sc8547d_check_work_mode_support(mode);
}

static int sc8547d_cp_set_iin(struct oplus_chg_ic_dev *ic_dev, int iin)
{
	return 0;
}

static int sc8547d_cp_get_vin(struct oplus_chg_ic_dev *ic_dev, int *vin)
{
	struct sc8547d_device *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	rc = sc8547_get_cp_vbus(chip);
	if (rc < 0) {
		chg_err("[%s] can't get cp vin, rc=%d\n", chip->dev->of_node->name, rc);
		return rc;
	}
	*vin = rc;
	return 0;
}

static int sc8547d_cp_get_iin(struct oplus_chg_ic_dev *ic_dev, int *iin)
{
	struct sc8547d_device *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	rc = sc8547_get_cp_ichg(chip);
	if (rc < 0) {
		chg_err("[%s] can't get cp iin, rc=%d\n", chip->dev->of_node->name, rc);
		return rc;
	}
	*iin = rc;

	return 0;
}

static int sc8547d_cp_get_vout(struct oplus_chg_ic_dev *ic_dev, int *vout)
{
	struct sc8547d_device *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	rc = sc8547_get_cp_vbat(chip);
	if (rc < 0) {
		chg_err("[%s] can't get cp vout, rc=%d\n", chip->dev->of_node->name, rc);
		return rc;
	}
	*vout = rc;

	return 0;
}

static int sc8547d_cp_get_iout(struct oplus_chg_ic_dev *ic_dev, int *iout)
{
	struct sc8547d_device *chip;
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

static int sc8547d_cp_get_vac(struct oplus_chg_ic_dev *ic_dev, int *vac)
{
	struct sc8547d_device *chip;
	u8 data_block[2] = { 0 };
	s32 err_info[2] = { 0 };
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (!chip->vac_support)
		return -ENOTSUPP;

	mutex_lock(&chip->adc_freeze_lock);
	sc8547_update_bits(chip->client, SC8547_REG_11, SC8547_ADC_FREEZE_MASK, SC8547_ADC_FREEZE_MASK);
	rc = sc8547d_read_data(chip, SC8547_REG_17, data_block, 2);
	sc8547_update_bits(chip->client, SC8547_REG_11, SC8547_ADC_FREEZE_MASK, 0);
	mutex_unlock(&chip->adc_freeze_lock);
	if (rc < 0) {
		err_info[0] = SC8547_REG_17;
		err_info[1] = rc;
		sc8547_i2c_error(chip, true, true, err_info);
		chg_err("[%s] sc8547 read vac error, rc=%d\n", chip->dev->of_node->name, rc);
		return rc;
	} else {
		sc8547_i2c_error(chip, false, true, err_info);
	}

	*vac = (((data_block[0] & SC8547_VAC_POL_H_MASK) << 8) | data_block[1]) * SC8547_VAC_ADC_LSB;

	return 0;
}

static int sc8547d_cp_set_work_start(struct oplus_chg_ic_dev *ic_dev, bool start)
{
	struct sc8547d_device *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	chg_info("%s work %s\n", chip->dev->of_node->name, start ? "start" : "stop");

	rc = sc8547_set_chg_enable(chip, start);
	if (rc < 0)
		return rc;
	chip->work_start = start;
	oplus_imp_node_set_active(chip->input_imp_node, start);
	oplus_imp_node_set_active(chip->output_imp_node, start);

	if (start)
		schedule_delayed_work(&chip->track_cp_switching_work, msecs_to_jiffies(CP_SWITCHING_WORK_DELAY_MS));
	else
		cancel_delayed_work(&chip->track_cp_switching_work);

	return 0;
}

static int sc8547d_cp_set_ucp_disable(struct oplus_chg_ic_dev *ic_dev, bool disable)
{
	struct sc8547d_device *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	chg_info("%s %s\n", chip->dev->of_node->name, disable ? "disable" : "enable");
	if (disable)
		sc8547_update_bits(chip->client, SC8547_REG_05, SC8547_IBUS_UCP_DIS_MASK,
				   SC8547_IBUS_UCP_DISABLE << SC8547_IBUS_UCP_DIS_SHIFT);
	else
		sc8547_update_bits(chip->client, SC8547_REG_05, SC8547_IBUS_UCP_DIS_MASK,
				   SC8547_IBUS_UCP_ENABLE << SC8547_IBUS_UCP_DIS_SHIFT);

	return 0;
}

static int sc8547d_cp_set_sstimeout_ucp_enable(struct oplus_chg_ic_dev *ic_dev, bool enable)
{
	struct sc8547d_device *chip;
	int ret;
	u8 reg_data;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	if (chip->use_slave_cp)
		return 0;

	ret = sc8547_read_byte(chip->client, SC8547_REG_05, &reg_data);
	if ((enable && !(reg_data >> SC8547_IBUS_UCP_DIS_SHIFT)) || (!enable && (reg_data >> SC8547_IBUS_UCP_DIS_SHIFT)))
		return 0;

	if (enable && (reg_data >> SC8547_IBUS_UCP_DIS_SHIFT)) {
		ret = sc8547_update_bits(chip->client, SC8547_REG_05, SC8547_IBUS_UCP_DIS_MASK,
			SC8547_IBUS_UCP_ENABLE << SC8547_IBUS_UCP_DIS_SHIFT);
		ret |= sc8547_update_bits(chip->client, SC8547_REG_08, SC8547_SS_TIMEOUT_SET_MASK,
			SC8547_SS_TIMEOUT_81920MS << SC8547_SS_TIMEOUT_SET_SHIFT);/* ucp ss_time enable*/
	} else {
		ret = sc8547_update_bits(chip->client, SC8547_REG_05, SC8547_IBUS_UCP_DIS_MASK,
			SC8547_IBUS_UCP_DISABLE << SC8547_IBUS_UCP_DIS_SHIFT);
		ret |= sc8547_update_bits(chip->client, SC8547_REG_08, SC8547_SS_TIMEOUT_SET_MASK,
			SC8547_SS_TIMEOUT_DISABLE << SC8547_SS_TIMEOUT_SET_SHIFT);/* ucp ss_time disable*/
	}
	sc8547_read_byte(chip->client, SC8547_REG_05, &reg_data);

	chg_info("%s %s SC8547_REG_05 = 0x%0x\n", chip->dev->of_node->name, enable ? "enable" : "disable", reg_data);

	return 0;
}

static int sc8547d_cp_get_work_status(struct oplus_chg_ic_dev *ic_dev, bool *start)
{
	struct sc8547d_device *chip;
	u8 data;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	rc = sc8547_read_byte(chip->client, SC8547_REG_06, &data);
	if (rc < 0) {
		chg_err("[%s] read SC8547_REG_06 error, rc=%d\n", chip->dev->of_node->name, rc);
		return rc;
	}

	*start = (data & SC8547_CP_SWITCHING_STAT_MASK) ? true : false;

	return 0;
}


static int sc8547d_cp_get_adc_enable(struct  oplus_chg_ic_dev *ic_dev, bool *status)
{
	int ret = 0;
	u8 data;
	struct sc8547d_device *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (!chip) {
		chg_err("Failed\n");
		return -ENODEV;
	}

	ret = sc8547_read_byte(chip->client, SC8547_REG_11, &data);
	if (ret < 0) {
		chg_err("read SC8547D_REG_11 failed, ret = %d\n", ret);
		return ret;
	}

	*status = (data >> 7) ? true : false;

	return ret;
}

static int sc8547d_cp_adc_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct sc8547d_device *chip;
	int ret = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	chg_debug("en = %d \n", en);
	ret = sc8547_set_adc_enable(chip, en);
	if (ret < 0)
		chg_err("adc enable config failed\n");

	return ret;
}

static int sc8547d_cp_watchdog_reset(struct oplus_chg_ic_dev *ic_dev)
{
	struct sc8547d_device *chip;
	u8 data;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	rc = sc8547_read_byte(chip->client, SC8547_REG_07, &data);
	if (rc < 0)
		return rc;

	return 0;
}

static void *sc8547d_cp_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
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
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, sc8547d_cp_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, sc8547d_cp_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP, sc8547d_cp_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST, sc8547d_cp_smt_test);
		break;
	case OPLUS_IC_FUNC_CP_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_ENABLE, sc8547d_cp_enable);
		break;
	case OPLUS_IC_FUNC_CP_WATCHDOG_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_WATCHDOG_ENABLE, sc8547d_cp_wd_enable);
		break;
	case OPLUS_IC_FUNC_CP_HW_INTI:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_HW_INTI, sc8547d_cp_hw_init);
		break;
	case OPLUS_IC_FUNC_CP_SET_WORK_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_WORK_MODE, sc8547d_cp_set_work_mode);
		break;
	case OPLUS_IC_FUNC_CP_GET_WORK_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_WORK_MODE, sc8547d_cp_get_work_mode);
		break;
	case OPLUS_IC_FUNC_CP_CHECK_WORK_MODE_SUPPORT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_CHECK_WORK_MODE_SUPPORT,
			sc8547d_cp_check_work_mode_support);
		break;
	case OPLUS_IC_FUNC_CP_SET_IIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_IIN, sc8547d_cp_set_iin);
		break;
	case OPLUS_IC_FUNC_CP_GET_VIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_VIN, sc8547d_cp_get_vin);
		break;
	case OPLUS_IC_FUNC_CP_GET_IIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_IIN, sc8547d_cp_get_iin);
		break;
	case OPLUS_IC_FUNC_CP_GET_VOUT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_VOUT, sc8547d_cp_get_vout);
		break;
	case OPLUS_IC_FUNC_CP_GET_IOUT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_IOUT, sc8547d_cp_get_iout);
		break;
	case OPLUS_IC_FUNC_CP_GET_VAC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_VAC, sc8547d_cp_get_vac);
		break;
	case OPLUS_IC_FUNC_CP_SET_WORK_START:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_WORK_START, sc8547d_cp_set_work_start);
		break;
	case OPLUS_IC_FUNC_CP_GET_WORK_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_WORK_STATUS, sc8547d_cp_get_work_status);
		break;
	case OPLUS_IC_FUNC_CP_SET_ADC_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_ADC_ENABLE, sc8547d_cp_adc_enable);
		break;
	case OPLUS_IC_FUNC_CP_GET_ADC_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_ADC_ENABLE, sc8547d_cp_get_adc_enable);
		break;
	case OPLUS_IC_FUNC_CP_SET_UCP_DISABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_UCP_DISABLE, sc8547d_cp_set_ucp_disable);
		break;
	case OPLUS_IC_FUNC_CP_WATCHDOG_RESET:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_WATCHDOG_RESET, sc8547d_cp_watchdog_reset);
		break;
	case OPLUS_IC_FUNC_CP_SET_SSTIMEOUT_UCP_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_SSTIMEOUT_UCP_ENABLE,
			sc8547d_cp_set_sstimeout_ucp_enable);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq sc8547d_cp_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
};

static int sc8547d_get_input_node_impedance(void *data)
{
	struct sc8547d_device *chip;
	int vac, vin, iin;
	int r_mohm;
	int rc;

	if (data == NULL)
		return -EINVAL;
	chip = data;

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_GET_VIN, &vin);
	if (rc < 0) {
		chg_err("can't read cp vin, rc=%d\n", rc);
		return rc;
	}
	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_GET_IIN, &iin);
	if (rc < 0) {
		chg_err("can't read cp iin, rc=%d\n", rc);
		return rc;
	}
	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_GET_VAC, &vac);
	if (rc < 0 && rc != -ENOTSUPP) {
		chg_err("can't read cp vac, rc=%d\n", rc);
		return rc;
	} else if (rc == -ENOTSUPP) {
		/* If the current IC does not support it, try to get it from the parent IC */
		rc = oplus_chg_ic_func(chip->cp_ic->parent, OPLUS_IC_FUNC_CP_GET_VAC, &vac);
		if (rc < 0) {
			chg_err("can't read parent cp vac, rc=%d\n", rc);
			return rc;
		}
	}

	r_mohm = (vac - vin) * 1000 / iin;
	if (r_mohm < 0) {
		chg_err("input_node: r_mohm=%d\n", r_mohm);
		r_mohm = 0;
	}

	return r_mohm;
}

static int sc8547d_get_output_node_impedance(void *data)
{
	struct sc8547d_device *chip;
	struct oplus_mms *gauge_topic;
	union mms_msg_data mms_data = { 0 };
	int vout, iout, vbat;
	int r_mohm;
	int rc;

	if (data == NULL)
		return -EINVAL;
	chip = data;

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_GET_VOUT, &vout);
	if (rc < 0) {
		chg_err("can't read cp vout, rc=%d\n", rc);
		return rc;
	}
	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_GET_IOUT, &iout);
	if (rc < 0) {
		chg_err("can't read cp iout, rc=%d\n", rc);
		return rc;
	}

	gauge_topic = oplus_mms_get_by_name("gauge");
	if (gauge_topic == NULL) {
		chg_err("gauge topic not found\n");
		return -ENODEV;
	}
	rc = oplus_mms_get_item_data(gauge_topic, GAUGE_ITEM_VOL_MAX, &mms_data, false);
	if (rc < 0) {
		chg_err("can't get vbat, rc=%d\n", rc);
		return rc;
	}
	vbat = mms_data.intval;

	r_mohm = (vout - vbat * oplus_gauge_get_batt_num()) * 1000 / iout;
	if (r_mohm < 0) {
		chg_err("output_node: r_mohm=%d\n", r_mohm);
		r_mohm = 0;
	}

	return r_mohm;
}

static int sc8547d_init_imp_node(struct sc8547d_device *chip, struct device_node *of_node)
{
	struct device_node *imp_node;
	struct device_node *child;
	const char *name;
	int rc;

	imp_node = of_get_child_by_name(of_node, "oplus,impedance_node");
	if (imp_node == NULL)
		return 0;

	for_each_child_of_node(imp_node, child) {
		rc = of_property_read_string(child, "node_name", &name);
		if (rc < 0) {
			chg_err("can't read %s node_name, rc=%d\n", child->name, rc);
			continue;
		}
		if (!strcmp(name, "cp_input")) {
			chip->input_imp_node =
				oplus_imp_node_register(child, chip->dev, chip, sc8547d_get_input_node_impedance);
			if (IS_ERR_OR_NULL(chip->input_imp_node)) {
				chg_err("%s register error, rc=%ld\n", child->name, PTR_ERR(chip->input_imp_node));
				chip->input_imp_node = NULL;
				continue;
			}
		} else if (!strcmp(name, "cp_output")) {
			chip->output_imp_node =
				oplus_imp_node_register(child, chip->dev, chip, sc8547d_get_output_node_impedance);
			if (IS_ERR_OR_NULL(chip->output_imp_node)) {
				chg_err("%s register error, rc=%ld\n", child->name, PTR_ERR(chip->output_imp_node));
				chip->output_imp_node = NULL;
				continue;
			}
		} else {
			chg_err("unknown node_name: %s\n", name);
		}
	}

	return 0;
}

static void sc8547d_ic_offline_work(struct work_struct *work)
{
	struct sc8547d_device *chip =
		container_of(work, struct sc8547d_device, ic_offline_work);

	if (!chip->work_start)
		return;
	sc8547d_cp_set_work_start(chip->cp_ic, false);
	sc8547d_cp_enable(chip->cp_ic, false);
}

static void sc8547d_track_cp_switching_work(struct work_struct *work)
{
	bool start = false;
	struct delayed_work *dwork = to_delayed_work(work);
	struct sc8547d_device *chip =
		container_of(dwork, struct sc8547d_device, track_cp_switching_work);
	struct oplus_voocphy_manager *voocphy = chip->voocphy;

	sc8547d_cp_get_work_status(chip->cp_ic, &start);
	chg_info("start=%d, cp_reg_track[4]=%d\n", start, chip->cp_reg_track[4]);
	if (start && !chip->cp_reg_track[4])
		return;

	chip->cp_reg_track[4] = 0;

	sc8547_track_upload_cp_err_info(voocphy, TRACK_CP_ERR_CP_EN_FAIL);
}

static int sc8547d_disable_vote_callback(struct votable *votable, void *data,
					 int disable, const char *client, bool step)
{
	struct sc8547d_device *chip = data;
	bool online;

	if (disable < 0)
		online = true;
	else
		online = !disable;
	chg_info("[%s]%s set sc8547d online to %s\n", votable_name(votable),
		 client, online ? "true" : "false");

	if (chip->cp_ic ==NULL)
		return -ENODEV;
	if (chip->cp_ic->online == online)
		return 0;

	chip->cp_ic->online = online;
	if (online) {
		oplus_chg_ic_virq_trigger(chip->cp_ic, OPLUS_IC_VIRQ_ONLINE);
	} else {
		oplus_chg_ic_virq_trigger(chip->cp_ic, OPLUS_IC_VIRQ_OFFLINE);
		schedule_work(&chip->ic_offline_work);
	}

	return 0;
}

#define CP_IC_VOTEABLE_NAME_MAX	128
static int sc8547d_ic_register(struct sc8547d_device *chip)
{
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	struct device_node *child;
	struct oplus_chg_ic_dev *ic_dev = NULL;
	struct oplus_chg_ic_cfg ic_cfg;
	char votable_name[CP_IC_VOTEABLE_NAME_MAX] = { 0 };
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
			(void)sc8547d_init_imp_node(chip, child);
			snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "cp-sc8547d:%d", ic_index);
			snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
			ic_cfg.get_func = sc8547d_cp_get_func;
			ic_cfg.virq_data = sc8547d_cp_virq_table;
			ic_cfg.virq_num = ARRAY_SIZE(sc8547d_cp_virq_table);
			if (chip->disable_votable != NULL)
				break;

			snprintf(votable_name, CP_IC_VOTEABLE_NAME_MAX - 1,
				 "SC8547D_DISABLE:%d", ic_index);
			chip->disable_votable = create_votable(votable_name,
				VOTE_SET_ANY, sc8547d_disable_vote_callback, chip);
			if (IS_ERR(chip->disable_votable)) {
				rc = PTR_ERR(chip->disable_votable);
				chg_err("creat disable_votable error, rc=%d\n", rc);
				chip->disable_votable = NULL;
				return rc;
			}
			break;
		default:
			chg_err("not support ic_type(=%d)\n", ic_type);
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
		default:
			chg_err("not support ic_type(=%d)\n", ic_dev->type);
			continue;
		}

		of_platform_populate(child, NULL, NULL, chip->dev);
	}

	return 0;
}

static void sc8547d_wired_topic_callback(struct mms_subscribe *subs, enum mms_msg_type type,
					u32 id, bool sync)
{
	struct sc8547d_device *chip = subs->priv_data;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WIRED_ITEM_PRE_OTG_ENABLE:
			chip->otg_connected = true;
			sc8547_update_bits(chip->client, SC8547D_ADDR_OTG_EN, SC8547D_OTG_EN_MASK, 0x04);
			break;
		case WIRED_ITEM_OTG_ENABLE:
			schedule_work(&chip->otg_enabled_work);
			break;
		default:
			break;
			}
		break;
	default:
		break;
	}
}

static void sc8547d_subscribe_wired_topic(struct oplus_mms *topic, void *prv_data)
{
	struct sc8547d_device *chip = prv_data;

	chip->wired_topic = topic;
	chip->wired_subs = oplus_mms_subscribe(chip->wired_topic, chip,
						  sc8547d_wired_topic_callback, chip->cp_ic->manu_name);
	if (IS_ERR_OR_NULL(chip->wired_subs)) {
		chg_err("subscribe wired topic error, rc=%ld\n", PTR_ERR(chip->wired_subs));
		return;
	}
}

static int sc8547d_parse_dt(struct sc8547d_device *chip)
{
	int rc;
	struct device_node *node = chip->dev->of_node;

	rc = of_property_read_u32(node, "ovp_reg", &chip->ovp_reg);
	if (rc)
		chip->ovp_reg = 0xe;
	chg_err("ovp_reg=0x%02x\n", chip->ovp_reg);

	rc = of_property_read_u32(node, "ocp_reg", &chip->ocp_reg);
	if (rc)
		chip->ocp_reg = 0x8;
	chg_err("ocp_reg=0x%02x\n", chip->ocp_reg);

	rc = of_property_read_u8(node, "vbus_ovp_reg", &chip->vbus_ovp_reg);
	if (rc)
		chip->vbus_ovp_reg = 0x50; /* VBUS_OVP 10V */
	chg_err("vbus_ovp_reg=0x%02x\n", chip->vbus_ovp_reg);

	chip->use_vooc_phy = of_property_read_bool(chip->dev->of_node, "oplus,use_vooc_phy");
	chip->use_ufcs_phy = of_property_read_bool(chip->dev->of_node, "oplus,use_ufcs_phy");
	chip->use_slave_cp = of_property_read_bool(chip->dev->of_node, "oplus,use_slave_cp");
	chip->vac_support = of_property_read_bool(chip->dev->of_node, "oplus,vac_support");
	chip->always_otg_en = of_property_read_bool(chip->dev->of_node, "oplus,always_otg_en");
	chg_info("use_vooc_phy=%d, use_ufcs_phy=%d, use_slave_cp=%d, vac_support=%d, always_otg_en=%d\n",
		 chip->use_vooc_phy, chip->use_ufcs_phy, chip->use_slave_cp, chip->vac_support,
		 chip->always_otg_en);
	return 0;
}

static void sc8547d_chgen_default(struct sc8547d_device *chip)
{
	sc8547_update_bits(chip->client, SC8547_REG_07, SC8547_CHG_EN_MASK, 0x00);
	return;
}

static void sc8547_mms_wait_topic(struct sc8547d_device *chip)
{
	if (chip->use_ufcs_phy)
		oplus_mms_wait_topic("error", sc8547d_subscribe_error_topic, chip);
	oplus_mms_wait_topic("wired", sc8547d_subscribe_wired_topic, chip);
}

static void sc8547d_init_default(struct sc8547d_device *chip)
{
	chip->ufcs_enable = false;
	chip->voocphy_enable = false;
	vote(chip->disable_votable, DEF_VOTER, false, 0, false);

	sc8547_mms_wait_topic(chip);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0))
static int sc8547d_driver_probe(struct i2c_client *client)
#else
static int sc8547d_driver_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
#endif
{
	struct sc8547d_device *chip;
	struct oplus_voocphy_manager *voocphy;
	struct device_node *node = oplus_get_node_by_child_gauge(client->dev.of_node);
	int rc;

	if (node && of_property_read_bool(node, "skip_cp_probe")) {
		chg_info("non-use cp, skip cp driver probe\n");
		return -ENODEV;
	}

	chip = devm_kzalloc(&client->dev, sizeof(struct sc8547d_device), GFP_KERNEL);
	if (chip == NULL) {
		chg_err("alloc sc8547d device buf error\n");
		return -ENOMEM;
	}

	voocphy = devm_kzalloc(&client->dev, sizeof(struct oplus_voocphy_manager), GFP_KERNEL);
	if (voocphy == NULL) {
		chg_err("alloc voocphy buf error\n");
		rc = -ENOMEM;
		goto alloc_voocphy_mg_err;
	}

	chip->client = client;
	chip->dev = &client->dev;
	voocphy->client = client;
	voocphy->dev = &client->dev;
	voocphy->priv_data = chip;
	chip->voocphy = voocphy;
	mutex_init(&chip->i2c_rw_lock);
	mutex_init(&chip->chip_lock);
	mutex_init(&chip->adc_freeze_lock);
	i2c_set_clientdata(client, voocphy);
	INIT_WORK(&chip->ufcs_regdump_work, sc8547d_ufcs_regdump_work);
	INIT_WORK(&chip->cp_regdump_work, sc8547d_cp_regdump_work);
	INIT_WORK(&chip->otg_enabled_work, sc8547d_otg_enabled_work);
	INIT_WORK(&chip->ic_offline_work, sc8547d_ic_offline_work);
	INIT_DELAYED_WORK(&chip->track_cp_switching_work, sc8547d_track_cp_switching_work);

	rc = sc8547d_parse_dt(chip);
	if (rc < 0)
		goto parse_dt_err;

	chip->regmap = devm_regmap_init_i2c(client, &sc8547d_regmap_config);
	if (chip->regmap == NULL) {
		rc = -ENODEV;
		goto regmap_init_err;
	}

	sc8547d_awake_init(chip);
	sc8547_create_device_node(&(client->dev));
	sc8547d_chgen_default(chip);

	if (chip->use_vooc_phy) {
		rc = sc8547_charger_choose(chip);
		if (rc <= 0) {
			chg_err("choose error, rc=%d\n", rc);
			goto choose_err;
		}
		if (!sc8547d_hw_version_check(chip)) {
			chg_err("not sc8547d\n");
		}

		voocphy->ops = &oplus_sc8547_ops;
		chip->voocphy_mg = voocphy;
		rc = oplus_register_voocphy(voocphy);
		if (rc < 0) {
			chg_err("failed to register voocphy, ret = %d", rc);
			goto reg_voocphy_err;
		}
	} else if (chip->use_slave_cp) {
		rc = sc8547_slave_charger_choose(chip);
		if (rc <= 0) {
			chg_err("slave cp choose error, rc=%d\n", rc);
			goto choose_err;
		}
		if (!sc8547d_hw_version_check(chip)) {
			chg_err("not sc8547d\n");
		}
		voocphy->slave_client = client;
		voocphy->slave_dev = &client->dev;
		voocphy->slave_ops = &oplus_sc8547_slave_ops;
		oplus_voocphy_get_chip(&chip->voocphy_mg);
		oplus_voocphy_slave_init(voocphy);
	}

	if (chip->use_ufcs_phy) {
		if (!sc8547d_hw_version_check(chip)) {
			chg_err("sc8547 dont use ufcs\n");
			chip->use_ufcs_phy = false;
			goto skip_ufcs_reg;
		}
		chip->ufcs = ufcs_device_register(chip->dev, &ufcs_ops, chip, &sc8547d_ufcs_config);
		if (IS_ERR_OR_NULL(chip->ufcs)) {
			rc = PTR_ERR(chip->ufcs);
			chg_err("ufcs device register error, rc=%d\n", rc);
			if (rc == -EINTR) {
				chip->use_ufcs_phy = false;
				chip->ufcs = NULL;
				goto skip_ufcs_reg;
			} else {
				rc = -ENODEV;
				goto reg_ufcs_err;
			}
		}
	}

skip_ufcs_reg:
	rc = sc8547_irq_register(chip);
	if (rc < 0) {
		if (chip->use_vooc_phy || chip->use_ufcs_phy) {
			chg_err("irq register error\n");
			goto irq_reg_err;
		}
	}

	rc = sc8547d_ic_register(chip);
	if (rc < 0) {
		chg_err("cp ic register error\n");
		goto cp_reg_err;
	}

	sc8547d_init_default(chip);
	chg_info("sc8547d(%s) probe successfully\n", chip->dev->of_node->name);

	return 0;

cp_reg_err:
	destroy_votable(chip->disable_votable);
	if (chip->input_imp_node != NULL)
		oplus_imp_node_unregister(chip->dev, chip->input_imp_node);
	if (chip->output_imp_node != NULL)
		oplus_imp_node_unregister(chip->dev, chip->output_imp_node);
irq_reg_err:
	if (chip->use_ufcs_phy)
		ufcs_device_unregister(chip->ufcs);
reg_ufcs_err:
reg_voocphy_err:
choose_err:
	sc8547d_awake_deinit(chip);
	sc8547_remove_device_node(&(client->dev));
regmap_init_err:
parse_dt_err:
	devm_kfree(&client->dev, voocphy);
alloc_voocphy_mg_err:
	devm_kfree(&client->dev, chip);
	return rc;
}

static void sc8547d_shutdown(struct i2c_client *client)
{
	int ret;
	u8 value = 0;

	ret = sc8547_read_byte(client, SC8547_REG_0C, &value);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to read REG_0C: %d\n", ret);
	} else if ((value & 0x80) != 0) {
		value = 0;
		ret = sc8547_update_bits(client, SC8547_REG_0C, SC8547_IBUS_REG_EN_MASK, value);
		if (ret < 0) {
			dev_err(&client->dev, "Failed to update SC8547_REG_0C: %d\n", ret);
		}
	}
	sc8547_update_bits(client, SC8547_REG_07, SC8547_CHG_EN_MASK, 0x00);
	sc8547_write_byte(client, SC8547_REG_11, 0x00);
	sc8547_write_byte(client, SC8547_REG_21, 0x00);
	return;
}

static const struct of_device_id sc8547d_match[] = {
	{.compatible = "oplus,sc8547d" },
	{},
};

static const struct i2c_device_id sc8547d_id[] = {
	{ "oplus,sc8547d", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sc8547d_id);

static struct i2c_driver sc8547d_i2c_driver = {
	.driver =
		{
			.name = "sc8547d",
			.owner = THIS_MODULE,
			.of_match_table = sc8547d_match,
		},
	.probe = sc8547d_driver_probe,
	.id_table = sc8547d_id,
	.shutdown = sc8547d_shutdown,
};

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
static int __init sc8547d_i2c_driver_init(void)
{
	int ret = 0;
	chg_info(" init start\n");

	if (i2c_add_driver(&sc8547d_i2c_driver) != 0) {
		chg_err(" failed to register sc8574a i2c driver.\n");
	} else {
		chg_err(" Success to register sc8574a i2c driver.\n");
	}

	return ret;
}

subsys_initcall(sc8547d_i2c_driver_init);
#else
static __init int sc8547d_i2c_driver_init(void)
{
	return i2c_add_driver(&sc8547d_i2c_driver);
}

static __exit void sc8547d_i2c_driver_exit(void)
{
	i2c_del_driver(&sc8547d_i2c_driver);
}

oplus_chg_module_register(sc8547d_i2c_driver);
#endif /*LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)*/

MODULE_DESCRIPTION("SC SC8547D VOOCPHY&UFCS Driver");
MODULE_LICENSE("GPL v2");
