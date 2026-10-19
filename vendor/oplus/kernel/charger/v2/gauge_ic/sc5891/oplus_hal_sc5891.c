// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2023 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[SC5891]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/version.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/usb/phy.h>
#include <linux/acpi.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/kthread.h>
#include <linux/sched/prio.h>
#include <uapi/linux/sched/types.h>
#include <linux/platform_device.h>
#include <linux/random.h>
#include <linux/errno.h>
#include <linux/pinctrl/consumer.h>
#include <linux/debugfs.h>
#include <linux/workqueue.h>

#include <oplus_chg.h>
#include <oplus_mms.h>
#include <oplus_chg_module.h>
#include <oplus_chg_ic.h>
#include "uecc_lib/uecc_wrapper.h"
#include "oplus_hal_sc5891.h"
#include <oplus_chg_monitor.h>
#include "../../monitor/oplus_chg_track.h"
#include <oplus_mms_gauge.h>

#define SC5891_PRIKEY_INDEX			0
#define SC5891_CMD_PARAM_LOCK			(0x00)
#define SC5891_CMD_PARAM_UNLOCK			(0x5a)
#define SC5891_CMD_READ_NORMAL_DELAY_MS		150
#define SC5891_CMD_READ_LONG_DELAY_MS		350
#define SC5891_RETRY_MAX			3
#define SC5891_PRIKEY_DEFAULT_INDEX		0
#define SC5891_PULL_DOWN_I2C_DURATION_MS	2000
#define SC5891_SHUTDOWN_WAIT_TIME_MS		5

#define ECO_BATT_SN_OFFSET			6
#define BATT_SN_LEN				12
#define FULL_BATT_SN_LEN			25
#define BATT_SN_MAX				5
#define MAX_SN_SIZE				BATT_SN_MAX * BATT_SN_LEN

struct sc5891_device {
	struct device *dev;
	struct oplus_chg_ic_dev *ic_dev;
	struct i2c_client *i2c;
	struct mutex cmd_rw_lock;
	struct mutex pinctrl_lock;
	struct mutex flow_lock;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_default;
	struct pinctrl_state *pinctrl_output_low;
	struct work_struct gauge_update_work;
	struct delayed_work hardware_init_work;

	struct wakeup_source *rw_wake_lock;
	atomic_t pm_suspended;

	struct oplus_mms *gauge_topic;
	struct mms_subscribe *gauge_subs;

	union sc5891_romid romid;
	uint8_t prikey[SC5891_INFO_LEN_PRIVATE_KEY];
	uint8_t pubkey[SC5891_INFO_LEN_PUBLIC_KEY];
	uint8_t cert[SC5891_INFO_LEN_CERT];
	struct battery_manufacture_info battinfo;
	struct delayed_work get_manu_battinfo_work;
	bool support_eco_design;
	bool cert_valid;
	bool hardware_init_ok;
	bool prikey_ok;
	int prikey_index;
	int sc5891_fault_mitigation;
	int batt_info_num;
	uint8_t dt_batt_info[BATT_SN_MAX][BATT_SN_LEN + 1];
	uint8_t batt_info[BATT_SN_LEN + 1];
	uint8_t full_batt_sn[FULL_BATT_SN_LEN + 1];
	bool sn_match;

	/*debugfs*/
	struct dentry *debugfs_sc5891;
	uint32_t debug_sc5891_cmd;
	uint32_t debug_sc5891_cmd_err;
	bool debug_sc5891_i2c_err;
	bool debug_sc5891_dump_log;
};

enum {
	SC5891_DATA_ID_BATT_SN = 0,
	SC5891_DATA_ID_MANU_DATE,                       // Add manufacture date data ID
	SC5891_DATA_ID_FIRST_USAGE_DATE,                // Add first usage date data ID
	SC5891_DATA_ID_UI_CYCLE_COUNT,                  // Add UI cycle count data ID
	SC5891_DATA_ID_UI_SOH,                          // Add UI SOH data ID
	SC5891_DATA_ID_USED_FLAG,                       // Add used flag data ID
	SC5891_DATA_ID_TERM_VOLT,
	SC5891_DATA_ID_LAST_CC,
	SC5891_DATA_ID_DEEP_DISCHG_COUNT,
	SC5891_DATA_ID_HISTSOH_CHECKSUM,
	SC5891_DATA_ID_HISTSOH_PARAM_NUM,
	SC5891_DATA_ID_HISTSOH_UI_SOH,
	SC5891_DATA_ID_HISTSOH_BATT_SOH,
	SC5891_DATA_ID_HISTSOH_UI_SOH_CC,
	SC5891_DATA_ID_HISTSOH_BATT_CC,
	SC5891_DATA_ID_HISTSOH_QMAX,
	SC5891_DATA_ID_HISTSOH,
	SC5891_DATA_ID_SILI_VCT,
	SC5891_DATA_ID_MAX,
};

struct sc5891_data_cfg {
	uint8_t page_id;
	uint8_t page_num;
	uint8_t byte_id;
	uint8_t data_len;	/*not include checksum*/
	bool need_checksum;	/*crc-8*/
};

/* page_id, page_num, byte_id, data_len, checksum */
const static struct sc5891_data_cfg sc5891_data_cfg_table[] = {
	[SC5891_DATA_ID_BATT_SN]		= {1, 2, 0, 25, false},
	[SC5891_DATA_ID_HISTSOH_CHECKSUM]	= {4, 1, 0, 1, false},
	[SC5891_DATA_ID_HISTSOH_PARAM_NUM]	= {4, 1, 1, 1, false},
	[SC5891_DATA_ID_HISTSOH_UI_SOH]		= {4, 1, 2, 1, false},
	[SC5891_DATA_ID_HISTSOH_BATT_SOH]	= {4, 1, 3, 1, false},
	[SC5891_DATA_ID_HISTSOH_UI_SOH_CC]	= {4, 1, 4, 2, false},
	[SC5891_DATA_ID_HISTSOH_BATT_CC]	= {4, 1, 6, 2, false},
	[SC5891_DATA_ID_HISTSOH_QMAX]		= {4, 1, 8, 2, false},
	[SC5891_DATA_ID_HISTSOH]		= {4, 1, 0, 10, false},
	[SC5891_DATA_ID_TERM_VOLT]		= {5, 1, 3, 2, true},
	[SC5891_DATA_ID_LAST_CC]		= {5, 1, 6, 2, true},
	[SC5891_DATA_ID_DEEP_DISCHG_COUNT]	= {5, 1, 0, 2, true},
	[SC5891_DATA_ID_SILI_VCT]		= {5, 1, 9, 2, true},
	/* Add ECO design related data configuration */
	[SC5891_DATA_ID_MANU_DATE]        = {3, 1, 10, 2, false},   // Manufacture date, 2 bytes
	[SC5891_DATA_ID_FIRST_USAGE_DATE] = {3, 1, 0, 2, true},     // First usage date, 2 bytes
	[SC5891_DATA_ID_UI_CYCLE_COUNT]   = {3, 1, 5, 2, true},     // UI cycle count, 2 bytes
	[SC5891_DATA_ID_UI_SOH]           = {3, 1, 3, 1, true},     // UI SOH, 1 byte
	[SC5891_DATA_ID_USED_FLAG]        = {3, 1, 8, 1, true},     // Used flag, 1 byte
};
static int sc5891_ic_get_romid(struct sc5891_device *chip, uint64_t *id);
#define SC5891_PINCTRL_AVOID_RETRY_MAX	3
static void sc5891_pinctrl_avoid(struct sc5891_device *chip, bool locked)
{
	int rc = 0;
	int i;
	if (chip->sc5891_fault_mitigation == 1) {
		if (locked) {
			mutex_lock(&chip->pinctrl_lock);
			pinctrl_select_state(chip->pinctrl, chip->pinctrl_default);
			mutex_unlock(&chip->pinctrl_lock);
			for (i = 0; i < SC5891_PINCTRL_AVOID_RETRY_MAX; i++) {
				rc = sc5891_ic_get_romid(chip, &chip->romid.value);
				if (rc < 0) {
					chg_err("get romid failed\n");
					continue;
				}
				break;
			}
		} else {
			mutex_lock(&chip->pinctrl_lock);
			pinctrl_select_state(chip->pinctrl, chip->pinctrl_output_low);
			mutex_unlock(&chip->pinctrl_lock);
		}
	}
}

static int sc5891_get_current_time_s(void)
{
	struct timespec ts;

	getnstimeofday(&ts);
	return ts.tv_sec;
}

static int sc5891_i2c_write(struct sc5891_device *chip, const struct sc5891_tansfer_data *data)
{
	int rc = 0;
	struct i2c_msg msg = {
		.addr = chip->i2c->addr,
		.flags = 0,
		.len = data->len + 3,	/*length(1byte) + crc-16(2byte)*/
		.buf = (uint8_t *)data,
	};

	if (atomic_read(&chip->pm_suspended) == 1) {
		chg_err("in suspend");
		return -EAGAIN;
	}

	rc = i2c_transfer(chip->i2c->adapter, &msg, 1);
	if (rc < 0)
		chg_err("i2c write failed, rc = %d\n", rc);

	return rc;
}

static int sc5891_i2c_read(struct sc5891_device *chip, struct sc5891_tansfer_data *data)
{
	int rc = 0;
	struct i2c_msg msg = {
		.addr = chip->i2c->addr,
		.flags = I2C_M_RD,
		.len = data->expect_len + 3,	/*length(1byte) + crc-16(2byte)*/
		.buf = (uint8_t *)data,
	};
	memset(data, 0x00, sizeof(struct sc5891_tansfer_data));

	if (atomic_read(&chip->pm_suspended) == 1) {
		chg_err("in suspend");
		return -EAGAIN;
	}

	rc = i2c_transfer(chip->i2c->adapter, &msg, 1);
	if (rc < 0)
		chg_err("i2c read failed, rc = %d\n", rc);

	return rc;
}

static uint8_t do_crc8(uint8_t *data, uint32_t len)
{
	uint32_t i;
	uint8_t crc = 0x00;

	while(len--) {
		crc ^= *data++;
		for (i = 0; i < 8; ++i) {
			if (crc & 0x01)
				crc = (crc >> 1) ^ 0x8c;
			else
				crc = crc >> 1;
		}
	}
	return crc;
}

static uint16_t do_crc(uint8_t *data, uint32_t len)
{
	uint32_t i;
	uint16_t crc = 0xFFFF;

	while(len--) {
		crc ^= *data++;
		for (i = 0; i < 8; ++i) {
			if (crc & 1)
				crc = (crc >> 1) ^ 0xA001;
			else
				crc = (crc >> 1);
		}
	}
	return crc;
}

static const uint8_t sys_pub[SC5891_INFO_LEN_PUBLIC_KEY] = {0x47, 0xf4, 0xe8, 0x76,
	0x66, 0xb2, 0xfb, 0x9f, 0x4b, 0xd2, 0x0f, 0xc8, 0xb9, 0x83, 0x4e, 0x71,
	0xb4, 0x4f, 0x3e, 0xc6, 0x0c, 0x0d, 0xf2, 0xae, 0x64, 0x26, 0xf8, 0xb4,
	0xc8, 0x66, 0x5b, 0x94, 0x26, 0x2d, 0x41, 0x8f, 0x54, 0x14, 0x04, 0xe9,
	0x90, 0x50, 0xbc, 0x39, 0x1d, 0xb5, 0x38, 0xa2, 0xfa, 0x82, 0x97, 0xbe,
	0xf0, 0x9d, 0xea, 0x20, 0x8d, 0x30, 0x3b, 0x5c, 0xe3, 0x30, 0x3e, 0xc2};
static int sc5891_extern_verify_cert(uint8_t *chip_pub, const uint8_t *cert)
{
	return (verify_cert(sys_pub, chip_pub, cert) == 1 ? 0 : -EINVAL);
}

static void sc5891_dump(uint8_t* p, uint8_t len)
{
	uint8_t i;

	chg_info("dump data len %d \n", len);
	for (i = 0; i < len; ++i)
		chg_info("[%d] 0x%02x ", i, *(p + i));
}

#define TRACK_UPLOAD_COUNT_MAX			10
#define TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD	(24 * 3600)
#define MAX_STR_LEN				256
__printf(3, 4)
static void sc5891_upload_track(struct sc5891_device *chip, int sub_type, const char *format, ...)
{
	va_list args;
	int len;
	static int upload_count = 0;
	static int pre_upload_time = 0;
	int curr_time;
	char buf[MAX_STR_LEN] = {0};

	if (!chip->hardware_init_ok) {
		chg_err("track upload is not allowed\n");
		return;
	}

	va_start(args, format);
	len = vsnprintf(buf, MAX_STR_LEN - 1, format, args);
	va_end(args);

	curr_time = sc5891_get_current_time_s();
	if (curr_time - pre_upload_time > TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD)
		upload_count = 0;

	if (upload_count >= TRACK_UPLOAD_COUNT_MAX) {
		chg_err("exceed upload limit, skip upload\n");
		return;
	}

	pre_upload_time = curr_time;
	oplus_chg_ic_creat_err_msg(chip->ic_dev, OPLUS_IC_ERR_GAUGE, sub_type, buf);
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_ERR);
	upload_count++;
}

static int sc5891_hardware_reset(struct sc5891_device *chip)
{
	int rc = 0;

	if (chip == NULL)
		return -EINVAL;

	if (chip->pinctrl == NULL) {
		chip->pinctrl = devm_pinctrl_get(chip->dev);
		if (IS_ERR_OR_NULL(chip->pinctrl)) {
			chg_err("get pinctrl fail\n");
			return -ENODEV;
		}
	}

	mutex_lock(&chip->pinctrl_lock);
	rc = pinctrl_select_state(chip->pinctrl, chip->pinctrl_output_low);
	msleep(SC5891_PULL_DOWN_I2C_DURATION_MS);
	rc = pinctrl_select_state(chip->pinctrl, chip->pinctrl_default);
	mutex_unlock(&chip->pinctrl_lock);
	chg_info("ic reset finish");

	return rc;
}

static int sc5891_check_error_inject(struct sc5891_device *chip, struct sc5891_tansfer_data *send_data)
{
	int rc = 0;

	if (chip->debug_sc5891_i2c_err) {
		chg_info("debugfs inject i2c error\n");
		chip->debug_sc5891_i2c_err = false;
		rc = -SC5891_RESULT_I2C_FAIL;
	}
	if (chip->debug_sc5891_cmd_err && chip->debug_sc5891_cmd == send_data->cmd) {
		chg_info("debugfs inject error, cmd:0x%x err:0x%d\n", chip->debug_sc5891_cmd,
			 chip->debug_sc5891_cmd_err);
		rc = -chip->debug_sc5891_cmd_err;
		chip->debug_sc5891_cmd_err = 0;
		chip->debug_sc5891_cmd = 0;
	}

	return rc;
}

static int sc5891_get_cmd_wait_time(uint8_t cmd)
{
	if (cmd == SC5891_CMD_ECW || cmd == SC5891_CMD_ECDSA)
		return SC5891_CMD_READ_LONG_DELAY_MS;

	return SC5891_CMD_READ_NORMAL_DELAY_MS;
}

static void __sc5891_prepare_send_data(struct sc5891_device *chip, struct sc5891_tansfer_data *send_data)
{
	uint16_t crc = 0;

	crc = do_crc(&(send_data->cmd), send_data->len);
	send_data->data[send_data->len] = (crc >> 8) & 0xff;
	send_data->data[send_data->len - 1] = crc & 0xff;
	if (chip->debug_sc5891_dump_log)
		sc5891_dump((uint8_t *)send_data, send_data->len + 3);
}

static int sc5891_post_check_recv_data(struct sc5891_device *chip, struct sc5891_tansfer_data *recv_data)
{
	uint16_t crc = 0;
	int rc = 0;

	crc = do_crc(&(recv_data->cmd), recv_data->len);
	if (crc != ((recv_data->data[recv_data->len] << 8) | recv_data->data[recv_data->len - 1])) {
		chg_err("recv data crc error\n");
		rc = -SC5891_RESULT_SOFT_CRC_ERROR;
	}

	return rc;
}

static int sc5891_send_cmd(struct sc5891_device *chip, struct sc5891_tansfer_data *send_data,
			   struct sc5891_tansfer_data *recv_data)
{
	int rc = 0;
	uint8_t expect_recv_len;

	if (chip == NULL || send_data == NULL)
		return -ENOENT;

	__sc5891_prepare_send_data(chip, send_data);

	mutex_lock(&chip->cmd_rw_lock);
	rc = sc5891_check_error_inject(chip, send_data);
	if (rc == -SC5891_RESULT_I2C_FAIL)
		goto i2c_err;
	if (rc != 0)
		goto out;

	rc = sc5891_i2c_write(chip, send_data);
	if (rc < 0) {
		chg_err("write cmd failed %d\n", rc);
		goto i2c_err;
	}
	if (send_data->cmd == SC5891_CMD_SHUTDOWN) {
		rc = 0;
		msleep(SC5891_SHUTDOWN_WAIT_TIME_MS);
		goto out;
	}

	msleep(sc5891_get_cmd_wait_time(send_data->cmd));

	expect_recv_len = recv_data->expect_len;
	rc = sc5891_i2c_read(chip, recv_data);
	if (rc < 0) {
		chg_err("read result failed %d\n", rc);
		goto i2c_err;
	}

	if (chip->debug_sc5891_dump_log)
		sc5891_dump((uint8_t *)recv_data, recv_data->len + 3);

	if (recv_data->result != SC5891_RESULT_SUCCESS) {
		rc = -recv_data->result;
		goto out;
	}
	mutex_unlock(&chip->cmd_rw_lock);

	if (recv_data->len < 1 && recv_data->len != expect_recv_len) {
		chg_err("recv cmd %d len err %d expect_recv_len %d\n", send_data->cmd, recv_data->len, expect_recv_len);
		return -SC5891_RESULT_RECV_LEN_NOT_MATCH;
	}

	rc = sc5891_post_check_recv_data(chip, recv_data);
	return rc;

i2c_err:
	sc5891_upload_track(chip, SEC_IC_ERR_I2C, "$$romid@@0x%x%x$$cmd@@0x%x$$rc@@%d",
		chip->romid.high_half, chip->romid.low_half, send_data->cmd, rc);
	rc = -SC5891_RESULT_I2C_FAIL; /*avoid double tracking*/
	sc5891_hardware_reset(chip);
out:
	mutex_unlock(&chip->cmd_rw_lock);
	return rc;
}

static int sc5891_ic_get_romid(struct sc5891_device *chip, uint64_t *id)
{
	struct sc5891_tansfer_data send_data = {
		.len = SC5891_CMD_SEND_LEN_GET_ROMID,
		.cmd = SC5891_CMD_GET_ROMID,
	};
	struct sc5891_tansfer_data recv_data = {
		.expect_len = SC5891_EXPECT_RECV_LEN_GET_ROMID,
	};
	int rc = 0;
	uint8_t crc;
	union sc5891_romid *rom_id;

	rc = sc5891_send_cmd(chip, &send_data, &recv_data);
	if (rc < 0) {
		chg_err("get romid failed, cmd res = %d\n", rc);
		goto err_track;
	}

	rom_id = (union sc5891_romid *)recv_data.data;
	crc = do_crc8((uint8_t *)rom_id, 7);
	if (crc != rom_id->crc) {
		chg_err("recv data crc error\n");
		rc = -SC5891_RESULT_HARD_CRC_ERROR;
		goto err_track;
	}
	memmove((uint8_t *)id, rom_id, SC5891_INFO_LEN_ROMID);

	return 0;

err_track:
	if (rc != SC5891_RESULT_I2C_FAIL)
		sc5891_upload_track(chip, SEC_IC_ERR_ROMID_FAIL, "$$rc@@%d", rc);
	return rc;
}

static int sc5891_ic_get_public_key_cert(struct sc5891_device *chip, uint8_t *key, uint8_t *certificate)
{
	struct sc5891_tansfer_data send_data = {
		.len = SC5891_CMD_SEND_LEN_GET_CERT,
		.cmd = SC5891_CMD_GET_CERTIFICATION,
	};
	struct sc5891_tansfer_data recv_data = {
		.expect_len = SC5891_EXPECT_RECV_LEN_GET_CERT,
	};
	int rc = 0;

	rc = sc5891_send_cmd(chip, &send_data, &recv_data);
	if (rc < 0) {
		chg_err("get public key and certificate failed, cmd res = %d\n", rc);
		return rc;
	}

	memmove(key, recv_data.data, SC5891_INFO_LEN_PUBLIC_KEY);
	memmove(certificate, recv_data.data + SC5891_INFO_LEN_PUBLIC_KEY, SC5891_INFO_LEN_CERT);
	return 0;
}

static int sc5891_ic_ecdsa(struct sc5891_device *chip, bool *is_valid)
{
	struct sc5891_tansfer_data send_data = {
		.len = SC5891_CMD_SEND_LEN_ECDSA,
		.cmd = SC5891_CMD_ECDSA,
	};
	struct sc5891_tansfer_data recv_data = {
		.expect_len = SC5891_EXPECT_RECV_LEN_ECDSA,
	};
	int rc = 0;
	uint8_t sign[SC5891_INFO_LEN_SIGN] = {0};
	uint8_t challenge[SC5891_INFO_LEN_CHALLENGE] = {0};

	get_random_bytes(challenge, SC5891_INFO_LEN_CHALLENGE);
	memmove(send_data.data, challenge, SC5891_INFO_LEN_CHALLENGE);

	rc = sc5891_send_cmd(chip, &send_data, &recv_data);
	if (rc < 0) {
		chg_err("ecdsa failed, cmd res = %d\n", rc);
		*is_valid = false;
		goto err_track;
	}

	memmove(sign, recv_data.data, SC5891_INFO_LEN_SIGN);

	*is_valid = verify_sign(chip->pubkey, (uint8_t *)&chip->romid.value, challenge, sign);
	if (!*is_valid) {
		chg_err("ecdsa verify failed\n");
		rc = -SC5891_RESULT_ECDSA_FAIL;
		goto err_track;
	}
	return 0;
err_track:
	if (rc != SC5891_RESULT_I2C_FAIL)
		sc5891_upload_track(chip, SEC_IC_ERR_ECDSA_FAIL, "$$romid@@0x%x%x$$rc@@%d",
			chip->romid.high_half, chip->romid.low_half, rc);
	return rc;
}

static int sc5891_ic_get_protection(struct sc5891_device *chip, uint8_t page_num, uint8_t *status)
{
	struct sc5891_tansfer_data send_data = {
		.len = SC5891_CMD_SEND_LEN_GET_PROTECTION,
		.cmd = SC5891_CMD_PROTECTION_STATUS,
	};
	struct sc5891_tansfer_data recv_data = {
		.expect_len = SC5891_EXPECT_RECV_LEN_GET_PROTECTION,
	};
	int rc = 0;

	send_data.data[0] = page_num;

	rc = sc5891_send_cmd(chip, &send_data, &recv_data);
	if (rc < 0) {
		chg_err("get protection status failed, cmd res = %d\n", rc);
		return rc;
	}

	*status = recv_data.data[0];
	return 0;
}

static int sc5891_ic_set_protection(struct sc5891_device *chip, uint8_t page_num, uint8_t protect)
{
	struct sc5891_tansfer_data send_data = {
		.len = SC5891_CMD_SEND_LEN_SET_PROTECTION,
		.cmd = SC5891_CMD_SET_PROTECTION_TYPE,
	};
	struct sc5891_tansfer_data recv_data = {
		.expect_len = SC5891_EXPECT_RECV_LEN_SET_PROTECTION,
	};
	int rc;

	send_data.data[0] = page_num;
	send_data.data[1] = protect;

	rc = sc5891_send_cmd(chip, &send_data, &recv_data);
	if (rc < 0) {
		chg_err("set protection failed, cmd res = %d\n", rc);
		return rc;
	}

	return 0;
}

static int sc5891_ic_mem_read(struct sc5891_device *chip, uint8_t page_id, uint8_t *data)
{
	struct sc5891_tansfer_data send_data = {
		.len = SC5891_CMD_SEND_LEN_READ_MEMORY,
		.cmd = SC5891_CMD_READ_MEMORY,
	};
	struct sc5891_tansfer_data recv_data = {
		.expect_len = SC5891_EXPECT_RECV_LEN_READ_MEMORY,
	};
	int rc = 0;

	send_data.data[0] = page_id;

	rc = sc5891_send_cmd(chip, &send_data, &recv_data);
	if (rc < 0) {
		chg_err("read page[%d] fail, cmd res = %d\n", page_id, rc);
		goto err_track;
	}
	memmove(data, recv_data.data, SC5891_INFO_PAGE_SIZE);

	return 0;
err_track:
	if (rc != SC5891_RESULT_I2C_FAIL)
		sc5891_upload_track(chip, SEC_IC_ERR_MEM_R_FAIL, "$$romid@@0x%x%x$$page_id@@%d$$rc@@%d",
			chip->romid.high_half, chip->romid.low_half, page_id, rc);
	return rc;
}

static int sc5891_ic_mem_write(struct sc5891_device *chip, uint8_t page_id, uint8_t *data)
{
	struct sc5891_tansfer_data send_data = {
		.len = SC5891_CMD_SEND_LEN_WRITE_MEMORY,
		.cmd = SC5891_CMD_WRITE_MEMORY,
	};
	struct sc5891_tansfer_data recv_data = {
		.expect_len = SC5891_EXPECT_RECV_LEN_WRITE_MEMORY,
	};
	int rc = 0;

	send_data.data[0] = page_id;
	memmove(send_data.data + 1, data, SC5891_INFO_PAGE_SIZE);
	rc = sc5891_send_cmd(chip, &send_data, &recv_data);
	if(rc < 0) {
		chg_err("write page[%d] fail, cmd res = %d\n", page_id, rc);
		goto err_track;
	}
	return 0;
err_track:
	if (rc != SC5891_RESULT_I2C_FAIL)
		sc5891_upload_track(chip, SEC_IC_ERR_MEM_W_FAIL, "$$romid@@0x%x%x$$page_id@@%d$$rc@@%d",
			chip->romid.high_half, chip->romid.low_half, page_id, rc);
	return rc;
}

static int sc5891_ic_ecw(struct sc5891_device *chip, bool *valid)
{
	struct sc5891_tansfer_data send_data = {
		.len = SC5891_CMD_SEND_LEN_ECW,
		.cmd = SC5891_CMD_ECW,
	};
	struct sc5891_tansfer_data recv_data = {
		.expect_len = SC5891_EXPECT_RECV_LEN_ECW,
	};
	uint8_t sign[SC5891_INFO_LEN_CERT] = {0};
	uint8_t challenge[SC5891_INFO_LEN_CHALLENGE] = {0};
	int rc;

	if (!chip->prikey_ok) {
		chg_err("prikey is not ok\n");
		return -EINVAL;
	}

	get_random_bytes(challenge, SC5891_INFO_LEN_CHALLENGE);
	gen_sign(chip->prikey, (uint8_t *)&chip->romid.value, challenge, sign);

	memmove(send_data.data, challenge, SC5891_INFO_LEN_CHALLENGE);
	memmove(send_data.data + SC5891_INFO_LEN_CHALLENGE, sign, SC5891_INFO_LEN_CERT);

	rc = sc5891_send_cmd(chip, &send_data, &recv_data);
	if (rc < 0) {
		chg_err("ecw fail, cmd res = %d\n", rc);
		*valid = false;
		goto err_track;
	}
	*valid = true;
	return 0;

err_track:
	if (rc != SC5891_RESULT_I2C_FAIL)
		sc5891_upload_track(chip, SEC_IC_ERR_ECW_FAIL, "$$romid@@0x%x%x$$rc@@%d",
			chip->romid.high_half, chip->romid.low_half, rc);
	return rc;
}
__maybe_unused
static int sc5891_ic_mem_lock(struct sc5891_device *chip)
{
	struct sc5891_tansfer_data send_data = {
		.len = SC5891_CMD_SEND_LEN_LOCK,
		.cmd = SC5891_CMD_LOCK,
	};
	struct sc5891_tansfer_data recv_data = {
		.expect_len = SC5891_EXPECT_RECV_LEN_LOCK,
	};
	int rc = 0;

	send_data.data[0] = SC5891_CMD_PARAM_LOCK;
	rc = sc5891_send_cmd(chip, &send_data, &recv_data);
	if (rc < 0) {
		chg_err("mem lock fail, cmd res = %d\n", rc);
		return rc;
	}
	return 0;
}

static int sc5891_ic_mem_unlock(struct sc5891_device *chip)
{
	struct sc5891_tansfer_data send_data = {
		.len = SC5891_CMD_SEND_LEN_LOCK,
		.cmd = SC5891_CMD_LOCK,
	};
	struct sc5891_tansfer_data recv_data = {
		.expect_len = SC5891_EXPECT_RECV_LEN_LOCK,
	};
	int rc;

	send_data.data[0] = SC5891_CMD_PARAM_UNLOCK;
	rc = sc5891_send_cmd(chip, &send_data, &recv_data);
	if (rc < 0) {
		chg_err("mem unlock fail, cmd res = %d\n", rc);
		return rc;
	}
	return 0;
}

static int sc5891_ic_enter_shutdown(struct sc5891_device *chip)
{
	struct sc5891_tansfer_data send_data = {
		.len = SC5891_CMD_SEND_LEN_SHUTDOWN,
		.cmd = SC5891_CMD_SHUTDOWN,
	};
	int rc;

	rc = sc5891_send_cmd(chip, &send_data, NULL);
	if (rc < 0) {
		chg_err("enter shutdown fail, cmd res = %x\n", rc);
		return rc;
	}
	return 0;
}

static bool sc5891_verify_checksum(uint8_t *page_buf, const struct sc5891_data_cfg *cfg)
{
	uint8_t crc;

	if (page_buf == NULL || cfg == NULL) {
		chg_err("input is invalid\n");
		return false;
	}

	crc = do_crc8(page_buf + cfg->byte_id, cfg->data_len);
	if (crc != page_buf[cfg->byte_id + cfg->data_len]) {
		chg_err("checksum verify fail. cal_crc:%x, crc:%x\n",
			crc, page_buf[cfg->byte_id + cfg->data_len]);
		return false;
	}
	return true;
}

static int __sc5891_read_page(struct sc5891_device *chip, int page_id,
			      uint8_t *data)
{
	int rc;
	int retry_cnt = SC5891_RETRY_MAX;

	if (page_id <= 0 || page_id > SC5891_INFO_PAGE_NUM) {
		chg_err("page_id:%d is invalid\n", page_id);
		return -ENODATA;
	}

retry:
	rc = sc5891_ic_mem_read(chip, page_id, data);
	if (rc < 0) {
		chg_err("read page fail. rc = %d\n", rc);
		retry_cnt--;
		if (retry_cnt > 0) {
			chg_err("retry read page:%d, remaining %d times\n", page_id, retry_cnt);
			msleep(SC5891_CMD_READ_NORMAL_DELAY_MS);
			goto retry;
		}
		return rc;
	}

	return 0;
}

static int sc5891_read_data(struct sc5891_device *chip, int id,
			    uint8_t *data, int *len)
{
	const struct sc5891_data_cfg *cfg;
	uint8_t *page_buf = NULL;
	int rc = 0;
	int i = 0;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	if (id < 0 || id >= SC5891_DATA_ID_MAX) {
		chg_err("invalid data id:%d\n", id);
		return -EINVAL;
	}

	cfg = &sc5891_data_cfg_table[id];

	page_buf = kzalloc(cfg->page_num * SC5891_INFO_PAGE_SIZE, GFP_KERNEL);
	if (page_buf == NULL) {
		chg_err("alloc page_buf for %d fail\n", cfg->page_num);
		return -ENOMEM;
	}

	mutex_lock(&chip->flow_lock);
	__pm_stay_awake(chip->rw_wake_lock);
	sc5891_pinctrl_avoid(chip, true);
	for (i = 0; i < cfg->page_num; i++) {
		rc = __sc5891_read_page(chip, cfg->page_id + i, page_buf + i * SC5891_INFO_PAGE_SIZE);
		if (rc < 0) {
			sc5891_pinctrl_avoid(chip, false);
			__pm_relax(chip->rw_wake_lock);
			mutex_unlock(&chip->flow_lock);
			goto err;
		}
	}
	sc5891_ic_enter_shutdown(chip);
	sc5891_pinctrl_avoid(chip, false);
	__pm_relax(chip->rw_wake_lock);
	mutex_unlock(&chip->flow_lock);

	if (cfg->need_checksum && !sc5891_verify_checksum(page_buf, cfg)) {
		chg_err("checksum fail. rc = %d\n", rc);
		rc = -SC5891_RESULT_READ_CHECKSUM_FAIL;
		sc5891_upload_track(chip, SEC_IC_ERR_MEM_R_FAIL,
			"$$romid@@0x%x%x$$page_id@@%d$$rc@@%d", chip->romid.high_half,
			chip->romid.low_half, cfg->page_id, rc);
		goto err;
	}

	memmove(data, page_buf + cfg->byte_id, cfg->data_len);
	*len = cfg->data_len;
err:
	kfree(page_buf);
	return rc;
}

static int __sc5891_check_protection(struct sc5891_device *chip, int page_id)
{
	int rc;
	uint8_t protect_status;

	rc = sc5891_ic_get_protection(chip, page_id, &protect_status);
	if (rc < 0)
		return rc;

	if (protect_status == SC5891_PROTECT_STATUS_ECW)
		return 0;

	if (protect_status == SC5891_PROTECT_STATUS_WP) {
		chg_err("page[%d] is write protect\n", page_id);
		return -EINVAL;
	}

	rc = sc5891_ic_set_protection(chip, page_id, SC5891_PROTECT_STATUS_ECW);

	return rc;
}

static int __sc5891_write_page(struct sc5891_device *chip, int page_id,
			     uint8_t *data)
{
	int rc;
	int retry_cnt = SC5891_RETRY_MAX;

	if (page_id <= 0 || page_id > SC5891_INFO_PAGE_NUM) {
		chg_err("page_id:%d is invalid\n", page_id);
		return -ENODATA;
	}

retry:
	rc = sc5891_ic_mem_write(chip, page_id, data);
	if (rc < 0) {
		chg_err("write page fail. rc = %d\n", rc);
		retry_cnt--;
		if (retry_cnt > 0) {
			chg_err("retry write page:%d, remaining %d times\n", page_id, retry_cnt);
			msleep(SC5891_CMD_READ_NORMAL_DELAY_MS);
			goto retry;
		}
		return rc;
	}

	return 0;
}

static int __sc5891_update_data_in_one_page(struct sc5891_device *chip, int id, uint8_t *data)
{
	const struct sc5891_data_cfg *cfg;
	uint8_t *page_buf = NULL;
	uint8_t crc;
	int rc;
	bool valid;

	cfg = &sc5891_data_cfg_table[id];

	if (cfg->page_num != 1) {
		chg_err("page num is %d, wrong func called\n", cfg->page_num);
		return -EINVAL;
	}

	page_buf = kzalloc(SC5891_INFO_PAGE_SIZE, GFP_KERNEL);
	if (page_buf == NULL) {
		chg_err("alloc page mem error\n");
		return -ENOMEM;
	}

	rc = __sc5891_check_protection(chip, cfg->page_id);
	if (rc < 0)
		goto err;

	rc = __sc5891_read_page(chip, cfg->page_id, page_buf);
	if (rc < 0)
		goto err;

	memmove(page_buf + cfg->byte_id, data, cfg->data_len);

	if (cfg->need_checksum) {
		crc = do_crc8(data, cfg->data_len);
		page_buf[cfg->byte_id + cfg->data_len] = crc;
	}

	rc = sc5891_ic_ecw(chip, &valid);
	if (rc < 0)
		goto err;

	rc = __sc5891_write_page(chip, cfg->page_id, page_buf);

err:
	kfree(page_buf);
	return rc;
}

static int __sc5891_update_data_in_multi_pages(struct sc5891_device *chip, int id, uint8_t *data)
{
	const struct sc5891_data_cfg *cfg;
	uint8_t *page_buf = NULL;
	int writen_len = 0;
	int copy_len = 0;
	uint8_t crc;
	int i;
	int rc;
	int offset;
	bool valid;

	cfg = &sc5891_data_cfg_table[id];

	page_buf = kzalloc(SC5891_INFO_PAGE_SIZE, GFP_KERNEL);
	if (page_buf == NULL) {
		chg_err("alloc page mem error\n");
		return -ENOMEM;
	}

	for (i = 0; i < cfg->page_num; i++) {
		memset(page_buf, 0, SC5891_INFO_PAGE_SIZE);
		rc = __sc5891_check_protection(chip, cfg->page_id + i);
		if (rc < 0)
			goto err;

		rc = __sc5891_read_page(chip, cfg->page_id + i, page_buf);
		if (rc < 0)
			goto err;

		if (cfg->data_len - writen_len > SC5891_INFO_PAGE_SIZE) {
			copy_len = SC5891_INFO_PAGE_SIZE;
			offset = cfg->byte_id;
		} else {
			copy_len = cfg->data_len - writen_len;
			offset = 0;
		}
		memmove(page_buf + offset, data + writen_len, copy_len);
		writen_len += copy_len;

		if (cfg->need_checksum && writen_len == cfg->data_len) {
			crc = do_crc8(data, cfg->data_len);
			page_buf[offset + copy_len] = crc;
		}

		rc = sc5891_ic_ecw(chip, &valid);
		if (rc < 0)
			goto err;

		rc = __sc5891_write_page(chip, cfg->page_id + i, page_buf);
		if (rc < 0)
			goto err;
	}

err:
	kfree(page_buf);
	return rc;
}

static int sc5891_write_data(struct sc5891_device *chip, int id,
			     uint8_t *data)
{
	const struct sc5891_data_cfg *cfg;
	int rc = 0;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	if (id < 0 || id >= SC5891_DATA_ID_MAX) {
		chg_err("invalid data id:%d\n", id);
		return -EINVAL;
	}

	cfg = &sc5891_data_cfg_table[id];

	mutex_lock(&chip->flow_lock);
	__pm_stay_awake(chip->rw_wake_lock);
	sc5891_pinctrl_avoid(chip, true);
	rc = sc5891_ic_mem_unlock(chip);
	if (rc < 0) {
		chg_err("mem unlock failed, rc=%d\n", rc);
		goto err;
	}

	if (unlikely(cfg->page_num > 1))
		rc = __sc5891_update_data_in_multi_pages(chip, id, data);
	else
		rc = __sc5891_update_data_in_one_page(chip, id, data);

	if (rc < 0) {
		chg_err("write data id:%d failed, rc=%d\n", id, rc);
	}

err:
	sc5891_ic_enter_shutdown(chip);
	sc5891_pinctrl_avoid(chip, false);
	__pm_relax(chip->rw_wake_lock);
	mutex_unlock(&chip->flow_lock);
	return rc;
}

static int sc5891_write_int(struct sc5891_device *chip, int id, int data)
{
	return sc5891_write_data(chip, id, (uint8_t*)&data);
}

static int sc5891_read_int(struct sc5891_device *chip, int id, int *data)
{
	int tmp = 0;
	int rc;
	int len;

	rc = sc5891_read_data(chip, id, (uint8_t*)&tmp, &len);
	if (rc == 0)
		*data = tmp;
	return rc;
}

static int sc5891_init(struct oplus_chg_ic_dev *ic_dev)
{
	struct sc5891_device *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	ic_dev->online = false;
	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}
	chg_info("%s init\n", ic_dev->manu_name);

	ic_dev->online = chip->hardware_init_ok;

	return 0;
}

static int sc5891_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	ic_dev->online = false;

	chg_info("%s exit\n", ic_dev->manu_name);
	return 0;
}

static int sc5891_get_term_volt(struct oplus_chg_ic_dev *ic_dev, int *term_volt)
{
	int rc = 0;
	int data = 0;
	struct sc5891_device *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return -ENODEV;
	}

	rc = sc5891_read_int(chip, SC5891_DATA_ID_TERM_VOLT, &data);
	if (rc < 0) {
		chg_err("read term_volt err %d\n", rc);
		return rc;
	}

	*term_volt = data;
	chg_info("read term_volt: %d\n", data);

	return 0;
}

static int sc5891_get_sili_vct(struct oplus_chg_ic_dev *ic_dev, int *sili_vct)
{
	int rc = 0;
	int data = 0;
	struct sc5891_device *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return -ENODEV;
	}

	rc = sc5891_read_int(chip, SC5891_DATA_ID_SILI_VCT, &data);
	if (rc < 0) {
		chg_err("read sili_vct err %d\n", rc);
		return rc;
	}

	*sili_vct = data;
	chg_info("read sili_vct: %d\n", data);

	return 0;
}

static int sc5891_get_last_cc(struct oplus_chg_ic_dev *ic_dev, int *last_cc)
{
	int rc = 0;
	int data = 0;
	struct sc5891_device *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return -ENODEV;
	}

	rc = sc5891_read_int(chip, SC5891_DATA_ID_LAST_CC, &data);
	if (rc < 0) {
		chg_err("read last cc err %d\n", rc);
		return rc;
	}

	*last_cc = data;
	chg_info("read last cc: %d\n", data);

	return 0;
}

#define HISTSOH_DATA_SIZE 16
#define HISTSOH_PAGE_SIZE 16

/* Calculate checksum for historic soh data (consistent with getCheckSum in chg_exchange_soh_mesg.cpp) */
static uint8_t sc5891_calculate_checksum(int *data, int num)
{
	int i;
	uint8_t checksum = 0;

	if (data == NULL || num > HISTSOH_DATA_SIZE)
		return 0;

	for (i = 0; i < num; i++) {
		checksum += ((data[i] >> 8) & 0xFF) + (data[i] & 0xFF);
	}
	return checksum;
}

static int sc5891_get_ui_soh_cc(struct oplus_chg_ic_dev *ic_dev, int *ui_soh_cc)
{
	int rc = 0;
	struct sc5891_device *chip;
	u8 histsoh_data[HISTSOH_PAGE_SIZE] = {0};
	int get_len;
	int read_checksum = 0;
	int num = 0;
	int set_member[HISTSOH_DATA_SIZE] = {0};
	int base_index = 0;
	int i = 0;
	uint8_t cal_checksum = 0;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return -ENODEV;
	}

	if (ui_soh_cc == NULL) {
		chg_err("ui_soh_cc is NULL\n");
		return -EINVAL;
	}

	/* Read complete historic soh data block for checksum validation */
	rc = sc5891_read_data(chip, SC5891_DATA_ID_HISTSOH, histsoh_data, &get_len);
	if (rc < 0) {
		chg_err("read historic_soh_data err %d\n", rc);
		return rc;
	}

	/* Parse the basic fields */
	read_checksum = (int)histsoh_data[0];   /* Checksum (1 byte) */
	num = (int)histsoh_data[1];             /* Number of members (1 byte) */

	/* Check the validity of the number of members */
	if (num > HISTSOH_DATA_SIZE || num < 0) {
		chg_err("Invalid num: %d\n", num);
		return -EINVAL;
	}

	/* Parse initial two elements (1 byte each) */
	set_member[0] = (int)histsoh_data[2]; /* UI_SOH */
	set_member[1] = (int)histsoh_data[3]; /* BATT_SOH */

	/* Process subsequent elements (16-bit value composition) */
	for (i = 2; i < num && i < HISTSOH_DATA_SIZE; i++) {
		base_index = 4 + (i - 2) * 2; /* Calculate data offset */
		if (base_index + 1 >= HISTSOH_PAGE_SIZE) {
			chg_err("Data overflow at index %d\n", i);
			return -EINVAL;
		}
		/* Combine high/low bytes */
		set_member[i] = (histsoh_data[base_index] << 8) | histsoh_data[base_index + 1];
	}

	/* Calculate checksum */
	cal_checksum = sc5891_calculate_checksum(set_member, num);

	/* Validate checksum */
	if (cal_checksum != (uint8_t)read_checksum) {
		chg_err("checksum error: calc=0x%02x, read=0x%02x, num=%d\n",
			cal_checksum, read_checksum, num);
		return -EINVAL;
	}

	/* Extract UI_SOH_CC from parsed data (index 2 in set_member array) */
	if (num > 2) {
		*ui_soh_cc = set_member[2];
		chg_info("read ui_soh_cc: %d (checksum validated)\n", *ui_soh_cc);
	} else {
		chg_err("insufficient data, num=%d\n", num);
		return -EINVAL;
	}

	return 0;
}

#define BATT_SN_SIZE 25
static int sc5891_set_historic_soh_data(struct sc5891_device *chip, const int *set_soh_data)
{
	int rc;
	int i;
	u8 set_data[HISTSOH_PAGE_SIZE] = {0};

	if (set_soh_data == NULL) {
		chg_err("set_soh_data == NULL failed\n");
		return -EINVAL;
	}

	chg_info("write pack historic_soh_data eeprom 2 is:");
	for (i = 0; i < HISTSOH_DATA_SIZE; i++) {
		chg_info("%d,", set_soh_data[i]);
		set_data[i] = (u8)(set_soh_data[i] & 0x00FF);
	}

	rc = sc5891_write_data(chip, SC5891_DATA_ID_HISTSOH, set_data);
	if (rc < 0) {
		chg_err("write SC5891_DATA_ID_HISTSOH err %d\n", rc);
		return rc;
	}

	chg_info("write historic_soh_data success\n");
	return 0;
}

static int sc5891_get_historic_soh_data(struct sc5891_device *chip, int *output_str, int len)
{
	int i;
	u8 get_data[HISTSOH_PAGE_SIZE];
	int rc = 0;
	int get_len;

	len = len / sizeof(int);
	if (output_str == NULL || len > HISTSOH_PAGE_SIZE) {
		chg_err("NULL pointer detected or len %d > 16\n", len);
		return -EINVAL;
	}

	rc = sc5891_read_data(chip, SC5891_DATA_ID_HISTSOH, get_data, &get_len);
	if (rc < 0) {
		chg_err("read historic_soh_data err %d\n", rc);
		return rc;
	}

	chg_info("read pack historic_soh_data PG_USER_EEPROM_2 is:%d", len);
	for (i = 0; i < len && i < HISTSOH_PAGE_SIZE; i++) {
		output_str[i] = (int)get_data[i];
		chg_info("%d,", output_str[i]);
	}

	return 0;
}

#define OPLUS_MAXIM_MAX_SOH_RETRY		5
static int oplus_sc5891_set_batt_histsoh_data(struct oplus_chg_ic_dev *ic_dev, const int *buf)
{
	struct sc5891_device *chip;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (buf == NULL) {
		chg_err("oplus_chg_ic_dev buf is NULL");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (chip == NULL) {
		chg_err("maxim chip is NULL");
		return -ENODEV;
	}

	for (i = 0; i < OPLUS_MAXIM_MAX_SOH_RETRY; i++) {
		rc  = sc5891_set_historic_soh_data(chip, buf);
		if (rc == 0) {
			chg_info("set_historic_soh_data success at retry %d\n", i);
			break;
		}
		chg_err("set_historic_soh_data failed at retry %d, rc=%d\n", i, rc);
	}

	return rc;
}

static int oplus_sc5891_get_batt_histsoh_data(struct oplus_chg_ic_dev *ic_dev, int *buf, int len)
{
	struct sc5891_device *chip;
	int rc = 0;
	int i;

	if (ic_dev == NULL || buf == NULL) {
		chg_err(" oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (chip == NULL) {
		chg_err(" maxim chip is NULL");
		return -ENODEV;
	}

	for (i = 0; i < OPLUS_MAXIM_MAX_SOH_RETRY; i++) {
		rc = sc5891_get_historic_soh_data(chip, buf, len);
		if (rc >= 0) {
			chg_info("get_historic_soh_data success at retry %d\n", i);
			break;
		}
		chg_err("get_historic_soh_data failed at retry %d, rc=%d\n", i, rc);
	}

	return rc;
}

static int sc5891_get_deep_dischg_count(struct oplus_chg_ic_dev *ic_dev, int *deep_dischg_count)
{
	int rc = 0;
	int data = 0;
	struct sc5891_device *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return -ENODEV;
	}

	rc = sc5891_read_int(chip, SC5891_DATA_ID_DEEP_DISCHG_COUNT, &data);
	if (rc < 0) {
		chg_err("read deep_dischg_count err %d\n", rc);
		return rc;
	}

	*deep_dischg_count = data;
	chg_info("read deep_dischg_count: %d\n", data);

	return 0;
}

static int sc5891_hardware_init(struct sc5891_device *chip)
{
	int rc = 0;

	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	mutex_lock(&chip->flow_lock);

	if (READ_ONCE(chip->hardware_init_ok)) {
		mutex_unlock(&chip->flow_lock);
		return 0;
	}

	chg_info("sc5891_hardware_init");
	sc5891_pinctrl_avoid(chip, true);

	rc = sc5891_ic_get_romid(chip, &chip->romid.value);
	if (rc < 0) {
		chg_err("sc5891 get romid fail. rc = %d\n", rc);
		goto err;
	}

	if (chip->romid.ven_code != SC5891_ROMID_VENDOR ||
	    chip->romid.cid != SC5891_ROMID_CID) {
		chg_err("sc5891 verify romid fail:romid:0x%x%x", chip->romid.high_half, chip->romid.low_half);
		rc = -EINVAL;
		goto err;
	}

	rc = sc5891_ic_get_public_key_cert(chip, chip->pubkey, chip->cert);
	if (rc < 0) {
		chg_err("sc5891 get pubkey fail. rc = %d\n", rc);
		goto err;
	}

	sc5891_ic_enter_shutdown(chip);
	WRITE_ONCE(chip->hardware_init_ok, true);
err:
	sc5891_pinctrl_avoid(chip, false);
	mutex_unlock(&chip->flow_lock);
	return rc;
}

static int sc5891_get_romid(struct oplus_chg_ic_dev *ic_dev, uint8_t *romid, int *len)
{
	struct sc5891_device *chip;
	int rc = 0;

	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	if (!ic_dev->online) {
		chg_err("%s is offline\n", ic_dev->name);
		return -ENODEV;
	}

	mutex_lock(&chip->flow_lock);
	sc5891_pinctrl_avoid(chip, true);
	rc = sc5891_ic_get_romid(chip, &chip->romid.value);
	sc5891_ic_enter_shutdown(chip);
	sc5891_pinctrl_avoid(chip, false);
	mutex_unlock(&chip->flow_lock);

	if (rc < 0)
		return rc;

	memmove(romid, &chip->romid.value, sizeof(chip->romid.value));
	*len = sizeof(chip->romid.value);

	return 0;
}

static int sc5891_write_page(struct oplus_chg_ic_dev *ic_dev,
	int page_id, uint8_t *data, int len)
{
	struct sc5891_device *chip;
	bool valid;
	int rc = 0;

	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	if (!ic_dev->online) {
		chg_err("%s is offline\n", ic_dev->name);
		return -ENODEV;
	}

	/*page 1 and page 2 are used to store battery SN and can't be write*/
	if (page_id < 3 || page_id > SC5891_INFO_PAGE_NUM) {
		chg_err("page_id:%d is invalid\n", page_id);
		return -ENODATA;
	}

	if (len > SC5891_INFO_PAGE_SIZE) {
		chg_err("len:%d exceed page size %d, cut off\n", len, SC5891_INFO_PAGE_SIZE);
		len = SC5891_INFO_PAGE_SIZE;
	}

	mutex_lock(&chip->flow_lock);
	__pm_stay_awake(chip->rw_wake_lock);
	sc5891_pinctrl_avoid(chip, true);
	rc = sc5891_ic_mem_unlock(chip);
	if (rc < 0)
		goto err_out;

	rc = __sc5891_check_protection(chip, page_id);
	if (rc < 0)
		goto err_out;

	rc = sc5891_ic_ecw(chip, &valid);
	if (rc < 0)
		goto err_out;

	rc = __sc5891_write_page(chip, page_id, data);

err_out:
	sc5891_ic_enter_shutdown(chip);
	sc5891_pinctrl_avoid(chip, false);
	__pm_relax(chip->rw_wake_lock);
	mutex_unlock(&chip->flow_lock);
	return rc;
}

static int sc5891_read_page(struct oplus_chg_ic_dev *ic_dev,
	int page_id, uint8_t *data, int *len)
{
	struct sc5891_device *chip;
	int rc = 0;

	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	if (!ic_dev->online) {
		chg_err("%s is offline\n", ic_dev->name);
		return -ENODEV;
	}

	if (data == NULL || len == NULL) {
		chg_err("data or len is NULL\n");
		return -EINVAL;
	}
	mutex_lock(&chip->flow_lock);
	__pm_stay_awake(chip->rw_wake_lock);
	sc5891_pinctrl_avoid(chip, true);
	rc = __sc5891_read_page(chip, page_id, data);
	if (rc < 0)
		goto err_out;
	*len = SC5891_INFO_PAGE_SIZE;

err_out:
	sc5891_ic_enter_shutdown(chip);
	sc5891_pinctrl_avoid(chip, false);
	__pm_relax(chip->rw_wake_lock);
	mutex_unlock(&chip->flow_lock);
	return rc;
}

static int sc5891_ecdsa(struct oplus_chg_ic_dev *ic_dev, bool *valid)
{
	struct sc5891_device *chip;
	int rc;

	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (chip == NULL || valid == NULL) {
		chg_err("chip or val is NULL");
		return -ENODEV;
	}

	if (!ic_dev->online) {
		chg_err("%s is offline\n", ic_dev->name);
		return -ENODEV;
	}

	if (!READ_ONCE(chip->hardware_init_ok)) {
		rc = sc5891_hardware_init(chip);
		if (rc < 0)
			return rc;
	}

	mutex_lock(&chip->flow_lock);
	sc5891_pinctrl_avoid(chip, true);
	rc = sc5891_ic_ecdsa(chip, valid);
	sc5891_ic_enter_shutdown(chip);
	sc5891_pinctrl_avoid(chip, false);
	mutex_unlock(&chip->flow_lock);

	return rc;
}

static int sc5891_ecw(struct oplus_chg_ic_dev *ic_dev, bool *valid)
{
	struct sc5891_device *chip;
	int rc;

	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (chip == NULL || valid == NULL) {
		chg_err("chip or valid is NULL");
		return -ENODEV;
	}

	if (!ic_dev->online) {
		chg_err("%s is offline\n", ic_dev->name);
		return -ENODEV;
	}

	if (!READ_ONCE(chip->hardware_init_ok)) {
		rc = sc5891_hardware_init(chip);
		if (rc < 0) {
			chg_err("hardware_init err %d\n", rc);
			return rc;
		}
	}

	mutex_lock(&chip->flow_lock);
	sc5891_pinctrl_avoid(chip, true);
	rc = sc5891_ic_ecw(chip, valid);
	sc5891_ic_enter_shutdown(chip);
	sc5891_pinctrl_avoid(chip, false);
	mutex_unlock(&chip->flow_lock);

	return rc;
}

static int sc5891_get_batt_info_from_ic(struct sc5891_device *chip);
static int sc5891_get_manu_date_from_sn(struct sc5891_device *chip, u16 *raw_out);

static int oplus_sc5891_get_manu_date(struct oplus_chg_ic_dev *ic_dev, char *buf, int len)
{
	struct sc5891_device *chip;
	int rc;
	u16 manu_date;
	int date_len = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip || !buf || len < OPLUS_BATTINFO_DATE_SIZE)
		return -EINVAL;

	if (!chip->support_eco_design)
		return -ENOTSUPP;

	rc = sc5891_get_manu_date_from_sn(chip, &manu_date);
	if (rc < 0) {
		chg_err("manu_date from batt_sn failed, rc=%d\n", rc);
		return rc;
	}

	date_len = snprintf(buf, len, "%d-%02d-%02d",
					   (((manu_date >> 9) & 0x7F) + 1980),
					   (manu_date >> 5) & 0xF,
					   manu_date & 0x1F);
	return date_len;
}

static int oplus_sc5891_get_first_usage_date(struct oplus_chg_ic_dev *ic_dev, char *buf, int len)
{
	struct sc5891_device *chip;
	u8 data[2];
	int rc;
	u16 first_usage_date;
	int date_len = 0;
	int read_len = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip || !buf || len < OPLUS_BATTINFO_DATE_SIZE)
		return -EINVAL;

	if (!chip->support_eco_design)
		return -ENOTSUPP;

	/* Read first usage date using SC5891 format */
	rc = sc5891_read_data(chip, SC5891_DATA_ID_FIRST_USAGE_DATE, data, &read_len);
	if (rc < 0) {
		chg_err("read first_usage_date failed, rc=%d\n", rc);
		return rc;
	}

	/* Convert to 16-bit data */
	first_usage_date = (data[1] << 8) | data[0];

	date_len = snprintf(buf, len, "%d-%02d-%02d",
					   (((first_usage_date >> 9) & 0x7F) + 1980),
					   (first_usage_date >> 5) & 0xF,
					   first_usage_date & 0x1F);
	return date_len;
}

static int oplus_sc5891_set_first_usage_date(struct oplus_chg_ic_dev *ic_dev, const char *buf)
{
	struct sc5891_device *chip;
	int year = 0;
	int month = 0;
	int day = 0;
	int rc = 0;
	u16 date = 0;
	u8 data[2];

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip || !buf)
		return -EINVAL;

	if (!chip->support_eco_design)
		return -ENOTSUPP;

	rc = sscanf(buf, "%d-%d-%d", &year, &month, &day);
	if (rc != 3) {
		chg_err("invalid first usage date format, rc=%d\n", rc);
		return -EINVAL;
	}
	date = (((year - 1980) & 0x7F) << 9) | ((month & 0xF) << 5) | (day & 0x1F);

	/* Convert to byte array */
	data[0] = date & 0xFF;
	data[1] = (date >> 8) & 0xFF;

	chg_info("%d-%d-%d, date=0x%04x", year, month, day, date);

	/* Write first usage date using SC5891 format */
	rc = sc5891_write_data(chip, SC5891_DATA_ID_FIRST_USAGE_DATE, data);
	if (rc < 0) {
		chg_err("set first usage date fail rc = %d\n", rc);
		return rc;
	}

	chip->battinfo.first_usage_date = date;
	chg_info("set first usage date succ\n");

	return 0;
}

static int oplus_sc5891_get_ui_cycle_count(struct oplus_chg_ic_dev *ic_dev, u16 *ui_cycle_count)
{
	struct sc5891_device *chip;
	u8 data[2];
	int rc;
	int read_len = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip || !ui_cycle_count)
		return -EINVAL;

	if (!chip->support_eco_design)
		return -ENOTSUPP;

	/* Read UI cycle count using SC5891 format */
	rc = sc5891_read_data(chip, SC5891_DATA_ID_UI_CYCLE_COUNT, data, &read_len);
	if (rc < 0) {
		chg_err("read ui_cycle_count failed, rc=%d\n", rc);
		return rc;
	}

	*ui_cycle_count = (data[1] << 8) | data[0];
	return 0;
}

static int oplus_sc5891_set_ui_cycle_count(struct oplus_chg_ic_dev *ic_dev, u16 ui_cycle_count)
{
	struct sc5891_device *chip;
	int rc = 0;
	u8 data[2];

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip)
		return -EINVAL;

	if (!chip->support_eco_design)
		return -ENOTSUPP;

	/* Convert to byte array */
	data[0] = ui_cycle_count & 0xFF;
	data[1] = (ui_cycle_count >> 8) & 0xFF;

	/* Write UI cycle count using SC5891 format */
	rc = sc5891_write_data(chip, SC5891_DATA_ID_UI_CYCLE_COUNT, data);
	if (rc < 0) {
		chg_err("set ui cycle count %u failed, rc=%d", ui_cycle_count, rc);
		return rc;
	}

	chip->battinfo.ui_cycle_count = ui_cycle_count;
	chg_info("set ui cycle count %u succ", ui_cycle_count);

	return 0;
}

static int oplus_sc5891_get_ui_soh(struct oplus_chg_ic_dev *ic_dev, u8 *ui_soh)
{
	struct sc5891_device *chip;
	u8 data[1];
	int rc;
	int read_len = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip || !ui_soh)
		return -EINVAL;

	if (!chip->support_eco_design)
		return -ENOTSUPP;

	/* Read UI SOH using SC5891 format */
	rc = sc5891_read_data(chip, SC5891_DATA_ID_UI_SOH, data, &read_len);
	if (rc < 0) {
		chg_err("read ui_soh failed, rc=%d\n", rc);
		return rc;
	}

	*ui_soh = data[0];
	return 0;
}

static int oplus_sc5891_set_ui_soh(struct oplus_chg_ic_dev *ic_dev, u8 ui_soh)
{
	struct sc5891_device *chip;
	int rc = 0;
	u8 data[1];

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip)
		return -EINVAL;

	if (!chip->support_eco_design)
		return -ENOTSUPP;

	data[0] = ui_soh;

	/* Write UI SOH using SC5891 format */
	rc = sc5891_write_data(chip, SC5891_DATA_ID_UI_SOH, data);
	if (rc < 0) {
		chg_err("set ui soh %u failed, rc=%d", ui_soh, rc);
		return rc;
	}

	chip->battinfo.ui_soh = ui_soh;
	chg_info("set ui soh %u succ", ui_soh);

	return 0;
}

static int oplus_sc5891_get_used_flag(struct oplus_chg_ic_dev *ic_dev, u8 *used_flag)
{
	struct sc5891_device *chip;
	u8 data[1];
	int rc;
	int read_len = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip || !used_flag)
		return -EINVAL;

	if (!chip->support_eco_design)
		return -ENOTSUPP;

	/* Read used flag using SC5891 format */
	rc = sc5891_read_data(chip, SC5891_DATA_ID_USED_FLAG, data, &read_len);
	if (rc < 0) {
		chg_err("read used_flag failed, rc=%d\n", rc);
		return rc;
	}

	*used_flag = data[0];
	return 0;
}

static int oplus_sc5891_set_used_flag(struct oplus_chg_ic_dev *ic_dev, u8 used_flag)
{
	struct sc5891_device *chip;
	int rc = 0;
	u8 data[1];

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip)
		return -EINVAL;

	if (!chip->support_eco_design)
		return -ENOTSUPP;

	data[0] = used_flag;

	/* Write used flag using SC5891 format */
	rc = sc5891_write_data(chip, SC5891_DATA_ID_USED_FLAG, data);
	if (rc < 0) {
		chg_err("set used flag %u failed, rc=%d", used_flag, rc);
		return rc;
	}

	chip->battinfo.used_flag = used_flag;
	chg_info("set used flag %u succ", used_flag);

	return 0;
}

static void oplus_sc5891_get_manu_battinfo_work(struct work_struct *work)
{
	int rc = 0;
	u8 data[2];
	int read_len = 0;
	struct sc5891_device *chip = container_of(work, struct sc5891_device, get_manu_battinfo_work.work);

	if (!chip->support_eco_design) {
		chg_info("not support eco design\n");
		return;
	}

	rc = sc5891_get_manu_date_from_sn(chip, &chip->battinfo.manu_date);
	if (rc)
		chg_err("get manu_date from batt_sn fail rc = %d\n", rc);

	rc = sc5891_read_data(chip, SC5891_DATA_ID_FIRST_USAGE_DATE, data, &read_len);
	if (rc)
		chg_err("get first_usage_date fail rc = %d\n", rc);
	else
		chip->battinfo.first_usage_date = (data[1] << 8) | data[0];

	rc = sc5891_read_data(chip, SC5891_DATA_ID_UI_CYCLE_COUNT, data, &read_len);
	if (rc)
		chg_err("get ui_cycle_count fail rc = %d\n", rc);
	else
		chip->battinfo.ui_cycle_count = (data[1] << 8) | data[0];

	rc = sc5891_read_data(chip, SC5891_DATA_ID_UI_SOH, data, &read_len);
	if (rc)
		chg_err("get ui_soh fail rc = %d\n", rc);
	else
		chip->battinfo.ui_soh = data[0];

	rc = sc5891_read_data(chip, SC5891_DATA_ID_USED_FLAG, data, &read_len);
	if (rc)
		chg_err("get used_flag fail rc = %d\n", rc);
	else
		chip->battinfo.used_flag = data[0];

	chg_info("ECO design info: manu_date=0x%04x, first_usage_date=0x%04x, ui_cycle_count=%u, ui_soh=%u, used_flag=%u",
			 chip->battinfo.manu_date, chip->battinfo.first_usage_date,
			 chip->battinfo.ui_cycle_count, chip->battinfo.ui_soh, chip->battinfo.used_flag);
}

static int sc5891_enter_shutdown(struct oplus_chg_ic_dev *ic_dev, bool *valid)
{
	struct sc5891_device *chip;
	int rc;

	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (chip == NULL || valid == NULL) {
		chg_err("chip or valid is NULL");
		return -ENODEV;
	}

	if (!ic_dev->online) {
		chg_err("%s is offline\n", ic_dev->name);
		return -ENODEV;
	}

	mutex_lock(&chip->flow_lock);
	sc5891_pinctrl_avoid(chip, true);
	rc = sc5891_ic_enter_shutdown(chip);
	sc5891_pinctrl_avoid(chip, false);
	mutex_unlock(&chip->flow_lock);

	*valid = (rc == 0);
	return rc;
}

/* Full SN: X* + 12-digit material + 3-char prod date (YMD) + tail; date at index 14..16 */
#define SC5891_SN_PROD_DATE_OFF		14u

static int sc5891_sn_char_to_year(unsigned char c)
{
	if (c >= '0' && c <= '9')
		return 2020 + (c - '0');
	if (c >= 'A' && c <= 'Z')
		return 2030 + (c - 'A');
	return -1;
}

static int sc5891_sn_char_to_month(unsigned char c)
{
	if (c >= '1' && c <= '9')
		return c - '0';
	if (c >= 'A' && c <= 'C')
		return c - 'A' + 10;
	return -1;
}

static int sc5891_sn_char_to_day(unsigned char c)
{
	if (c >= '1' && c <= '9')
		return c - '0';
	/* 10–31 via base-32-ish alphabet; I/O omitted (confusable with 1/0) */
	if (c >= 'A' && c <= 'H')
		return c - 'A' + 10;
	if (c >= 'J' && c <= 'N')
		return c - 'J' + 18;
	if (c >= 'P' && c <= 'X')
		return c - 'P' + 23;
	return -1;
}

static int sc5891_manu_date_raw_from_full_sn(const char *sn, u16 *out)
{
	int year, month, day;

	if (!sn || !out)
		return -EINVAL;
	if (strnlen(sn, FULL_BATT_SN_LEN + 1) < SC5891_SN_PROD_DATE_OFF + 3)
		return -EINVAL;

	year = sc5891_sn_char_to_year(sn[SC5891_SN_PROD_DATE_OFF]);
	month = sc5891_sn_char_to_month(sn[SC5891_SN_PROD_DATE_OFF + 1]);
	day = sc5891_sn_char_to_day(sn[SC5891_SN_PROD_DATE_OFF + 2]);

	if (year < 1980 || year > 1980 + 127 || month < 1 || month > 12 ||
	    day < 1 || day > 31)
		return -EINVAL;

	*out = (u16)((((year - 1980) & 0x7F) << 9) | ((month & 0xF) << 5) |
		     (day & 0x1F));
	return 0;
}

static int sc5891_get_manu_date_from_sn(struct sc5891_device *chip, u16 *raw_out)
{
	int rc;

	rc = sc5891_get_batt_info_from_ic(chip);
	if (rc < 0)
		return rc;

	return sc5891_manu_date_raw_from_full_sn(chip->full_batt_sn, raw_out);
}

static int sc5891_get_batt_info_from_ic(struct sc5891_device *chip)
{
	const struct sc5891_data_cfg *cfg = &sc5891_data_cfg_table[SC5891_DATA_ID_BATT_SN];
	uint8_t *buf;
	int rc;
	int len;

	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	buf = kzalloc(cfg->data_len, GFP_KERNEL);
	if (buf == NULL) {
		chg_err("alloc page mem error\n");
		return -ENOMEM;
	}

	rc = sc5891_read_data(chip, SC5891_DATA_ID_BATT_SN, buf, &len);
	if (rc < 0) {
		chg_err("fail to read batt_sn, rc = %d\n", rc);
		kfree(buf);
		return rc;
	}

	memmove(chip->full_batt_sn, buf, FULL_BATT_SN_LEN);
	chip->full_batt_sn[FULL_BATT_SN_LEN] = '\0';
	/* batt sn for authentication starts from index 2 */
	memmove(chip->batt_info, buf + 2, BATT_SN_LEN);
	chip->batt_info[BATT_SN_LEN] = '\0';
	kfree(buf);
	return 0;
}

static bool sc5891_check_batt_info(struct sc5891_device *chip)
{
	int i;
	int rc;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return false;
	}

	rc = sc5891_get_batt_info_from_ic(chip);
	if (rc < 0) {
		chg_err("get batt_info from IC failed!\n");
		return false;
	}
	chg_info("batt_info in IC: %s", chip->batt_info);

	for (i = 0; i < chip->batt_info_num; ++i) {
		if (memcmp(chip->batt_info, chip->dt_batt_info[i], BATT_SN_LEN) == 0)
			return true;
	}

	return false;
}

static int sc5891_get_batt_hmac(struct oplus_chg_ic_dev *ic_dev, bool *hmac)
{
	struct sc5891_device *chip;

	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	chip->sn_match = sc5891_check_batt_info(chip);

	if (!chip->sn_match) {
		chg_err("batt_info not match.");
		*hmac = false;
		return 0;
	}

	if (!chip->cert_valid)
		sc5891_ecdsa(ic_dev, &chip->cert_valid);

	chg_info("batt_info match. hmac:%d", chip->cert_valid);
	*hmac = chip->cert_valid;

	return 0;
}

static int sc5891_set_prikey(struct oplus_chg_ic_dev *ic_dev, int index,
	uint8_t *prikey, int len)
{
	struct sc5891_device *chip;

	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	memmove(chip->prikey, prikey, len);
	chip->prikey_ok = true;
	chg_info("set prikey index success\n");
	return 0;
}

static int sc5891_get_prikey_index(struct oplus_chg_ic_dev *ic_dev, int *index)
{
	struct sc5891_device *chip;

	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	*index = chip->prikey_index;
	return 0;
}

static int sc5891_get_eco_batt_sn(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	struct sc5891_device *chip;
	int rc;

	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	if (buf == NULL || len < OPLUS_BATT_SERIAL_NUM_SIZE)
		return -EINVAL;

	if (chip->cert_valid) {
		memmove(buf, chip->full_batt_sn + ECO_BATT_SN_OFFSET, OPLUS_BATT_SERIAL_NUM_SIZE);
		buf[OPLUS_BATT_SERIAL_NUM_SIZE - 1] = '\0';
		return strnlen(buf, OPLUS_BATT_SERIAL_NUM_SIZE);
	}

	rc = sc5891_get_batt_info_from_ic(chip);
	if (rc < 0) {
		chg_err("get batt_info from ic failed, rc:%d", rc);
		return rc;
	}

	memmove(buf, chip->full_batt_sn + ECO_BATT_SN_OFFSET, OPLUS_BATT_SERIAL_NUM_SIZE);
	buf[OPLUS_BATT_SERIAL_NUM_SIZE - 1] = '\0';
	return strnlen(buf, OPLUS_BATT_SERIAL_NUM_SIZE);
}

static int sc5891_get_sn_match(struct oplus_chg_ic_dev *ic_dev, bool *match)
{
	struct sc5891_device *chip;

	chip = oplus_chg_ic_get_priv_data(ic_dev);

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	if (!chip->sn_match)
		chip->sn_match = sc5891_check_batt_info(chip);

	*match = chip->sn_match;
	return 0;
}

static int sc5891_get_full_batt_sn(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	struct sc5891_device *chip;
	int rc;

	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	if (buf == NULL || len < FULL_BATT_SN_LEN)
		return -EINVAL;

	if (chip->cert_valid) {
		memmove(buf, chip->full_batt_sn, FULL_BATT_SN_LEN);
		return 0;
	}

	rc = sc5891_get_batt_info_from_ic(chip);
	if (rc < 0) {
		chg_err("get batt_info from ic failed, rc:%d", rc);
		return rc;
	}

	memmove(buf, chip->full_batt_sn, FULL_BATT_SN_LEN);
	return 0;
}

#define SC5891_TERM_VOLT_UPPER_LIMIT 4500
static int sc5891_write_term_volt(struct oplus_chg_ic_dev *ic_dev, int term_volt)
{
	struct sc5891_device *chip;
	int current_term_volt = 0;
	int rc = 0;

	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return -ENODEV;
	}

	if (term_volt < 0 || term_volt > SC5891_TERM_VOLT_UPPER_LIMIT) {
		chg_err("term_volt: %d is invalid\n", term_volt);
		return -EINVAL;
	}

	rc = sc5891_get_term_volt(ic_dev, &current_term_volt);
	if (rc < 0) {
		chg_err("read term_volt failed, rc=%d, force write\n", rc);
	} else if (current_term_volt == term_volt) {
		chg_info("term_volt already set to %d, skip write\n", term_volt);
		return 0;
	}

	rc = sc5891_write_int(chip, SC5891_DATA_ID_TERM_VOLT, term_volt);
	if (rc < 0) {
		chg_err("write term_volt failed, rc=%d\n", rc);
		return rc;
	}
	chg_info("write term_volt: %d (previous: %d)\n", term_volt, current_term_volt);
	return 0;
}

#define SC5891_LAST_CC_UPPER_LIMIT 10000
static int sc5891_write_last_cc(struct oplus_chg_ic_dev *ic_dev, int last_cc)
{
	struct sc5891_device *chip;
	int current_last_cc = 0;
	int rc = 0;

	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return -ENODEV;
	}

	if (last_cc < 0 || last_cc > SC5891_LAST_CC_UPPER_LIMIT) {
		chg_err("last_cc: %d is invalid\n", last_cc);
		return -EINVAL;
	}

	rc = sc5891_get_last_cc(ic_dev, &current_last_cc);
	if (rc < 0) {
		chg_err("read last_cc failed, rc=%d, force write\n", rc);
	} else if (current_last_cc == last_cc) {
		chg_info("last_cc already set to %d, skip write\n", last_cc);
		return 0;
	}

	rc = sc5891_write_int(chip, SC5891_DATA_ID_LAST_CC, last_cc);
	if (rc < 0) {
		chg_err("write last_cc failed, rc=%d\n", rc);
		return rc;
	}
	chg_info("write last_cc: %d (previous: %d)\n", last_cc, current_last_cc);
	return 0;
}

#define SC5891_DEEP_COUNT_UPPER_LIMIT 50000
static int sc5891_write_deep_count(struct oplus_chg_ic_dev *ic_dev, int deep_count)
{
	struct sc5891_device *chip;
	int current_deep_count = 0;
	int rc = 0;

	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return -ENODEV;
	}

	if (deep_count < 0 || deep_count > SC5891_DEEP_COUNT_UPPER_LIMIT) {
		chg_err("deep_count: %d is invalid\n", deep_count);
		return -EINVAL;
	}

	rc = sc5891_get_deep_dischg_count(ic_dev, &current_deep_count);
	if (rc < 0) {
		chg_err("read deep_count failed, rc=%d, force write\n", rc);
	} else if (current_deep_count == deep_count) {
		chg_info("deep_count already set to %d, skip write\n", deep_count);
		return 0;
	}

	rc = sc5891_write_int(chip, SC5891_DATA_ID_DEEP_DISCHG_COUNT, deep_count);
	if (rc < 0) {
		chg_err("write deep_count failed, rc=%d\n", rc);
		return rc;
	}
	chg_info("write deep_count: %d (previous: %d)\n", deep_count, current_deep_count);
	return 0;
}

#define SC5891_SILI_VCT_UPPER_LIMIT 5000
static int sc5891_write_sili_vct(struct oplus_chg_ic_dev *ic_dev, int sili_vct)
{
	struct sc5891_device *chip;
	int current_sili_vct = 0;
	int rc = 0;

	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return -ENODEV;
	}

	if (sili_vct < 0 || sili_vct > SC5891_SILI_VCT_UPPER_LIMIT) {
		chg_err("sili_vct: %d is invalid\n", sili_vct);
		return -EINVAL;
	}

	rc = sc5891_get_sili_vct(ic_dev, &current_sili_vct);
	if (rc < 0) {
		chg_err("read sili_vct failed, rc=%d, force write\n", rc);
	} else if (current_sili_vct == sili_vct) {
		chg_info("sili_vct already set to %d, skip write\n", sili_vct);
		return 0;
	}

	rc = sc5891_write_int(chip, SC5891_DATA_ID_SILI_VCT, sili_vct);
	if (rc < 0) {
		chg_err("write sili_vct failed, rc=%d\n", rc);
		return rc;
	}
	chg_info("write sili_vct: %d (previous: %d)\n", sili_vct, current_sili_vct);
	return 0;
}

static void *sc5891_ic_get_func(struct oplus_chg_ic_dev *ic_dev,
				enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT))
		return NULL;

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT,
			sc5891_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT,
			sc5891_exit);
		break;
	case OPLUS_IC_FUNC_GAUGE_SEC_GET_ROMID:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SEC_GET_ROMID,
			sc5891_get_romid);
		break;
	case OPLUS_IC_FUNC_GAUGE_SEC_WRITE_PAGE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SEC_WRITE_PAGE,
			sc5891_write_page);
		break;
	case OPLUS_IC_FUNC_GAUGE_SEC_READ_PAGE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SEC_READ_PAGE,
			sc5891_read_page);
		break;
	case OPLUS_IC_FUNC_GAUGE_SEC_ECDSA:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SEC_ECDSA,
			sc5891_ecdsa);
		break;
	case OPLUS_IC_FUNC_GAUGE_SEC_ECW:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SEC_ECW,
			sc5891_ecw);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_MANU_DATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_MANU_DATE,
					       oplus_sc5891_get_manu_date);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_FIRST_USAGE_DATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_FIRST_USAGE_DATE,
					       oplus_sc5891_get_first_usage_date);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_FIRST_USAGE_DATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_FIRST_USAGE_DATE,
					       oplus_sc5891_set_first_usage_date);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_UI_CC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_UI_CC,
					       oplus_sc5891_get_ui_cycle_count);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_UI_CC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_UI_CC,
					       oplus_sc5891_set_ui_cycle_count);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_UI_SOH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_UI_SOH,
					       oplus_sc5891_get_ui_soh);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_UI_SOH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_UI_SOH,
					       oplus_sc5891_set_ui_soh);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_USED_FLAG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_USED_FLAG,
					       oplus_sc5891_get_used_flag);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_USED_FLAG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_USED_FLAG,
					       oplus_sc5891_set_used_flag);
		break;
	case OPLUS_IC_FUNC_GAUGE_SEC_SHUTDOWN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SEC_SHUTDOWN,
			sc5891_enter_shutdown);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEEP_TERM_VOLT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DEEP_TERM_VOLT,
							sc5891_get_term_volt);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_DEEP_TERM_VOLT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_DEEP_TERM_VOLT,
							sc5891_write_term_volt);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEEP_DISCHG_COUNT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DEEP_DISCHG_COUNT,
						  sc5891_get_deep_dischg_count);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_DEEP_DISCHG_COUNT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_DEEP_DISCHG_COUNT,
						  sc5891_write_deep_count);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_LAST_CC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_LAST_CC,
						  sc5891_write_last_cc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_LAST_CC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_LAST_CC,
					      sc5891_get_last_cc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEC_CV_SOH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DEC_CV_SOH,
				sc5891_get_ui_soh_cc);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_BATT_HISTSOH_DATA:
		func = OPLUS_CHG_IC_FUNC_CHECK(
			OPLUS_IC_FUNC_GAUGE_SET_BATT_HISTSOH_DATA,
			oplus_sc5891_set_batt_histsoh_data);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_HISTSOH_DATA:
		func = OPLUS_CHG_IC_FUNC_CHECK(
			OPLUS_IC_FUNC_GAUGE_GET_BATT_HISTSOH_DATA,
			oplus_sc5891_get_batt_histsoh_data);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_AUTH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_AUTH,
			sc5891_get_batt_hmac);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_HMAC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_HMAC,
			sc5891_get_batt_hmac);
		break;
	case OPLUS_IC_FUNC_GAUGE_SEC_SET_PRIKEY:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SEC_SET_PRIKEY,
			sc5891_set_prikey);
		break;
	case OPLUS_IC_FUNC_GAUGE_SEC_GET_PRIKEY_INDEX:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SEC_GET_PRIKEY_INDEX,
			sc5891_get_prikey_index);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_SN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_SN,
			sc5891_get_eco_batt_sn);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_VCT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_VCT, sc5891_write_sili_vct);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_VCT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_VCT, sc5891_get_sili_vct);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_SN_MATCH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_SN_MATCH,
			sc5891_get_sn_match);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_IC_SN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_IC_SN,
			sc5891_get_full_batt_sn);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}
	return func;
}

static struct oplus_chg_ic_virq sc5891_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
	{ .virq_id = OPLUS_IC_VIRQ_RESUME },
};

static void sc5891_parse_batt_info(struct sc5891_device *chip)
{
	unsigned char sn_total[MAX_SN_SIZE] = {0};
	int len;
	int rc;
	int i;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return;
	}

	len = of_property_count_u8_elems(chip->dev->of_node, "oplus,batt_info");
	if (len < 0 || len > MAX_SN_SIZE) {
		chg_info("Count oplus,batt_info failed, len = %d\n", len);
		return;
	}

	rc = of_property_read_u8_array(chip->dev->of_node, "oplus,batt_info", sn_total, len);
	if (rc) {
		chg_err("Get oplus,batt_info failed %d\n", rc);
		return;
	}

	chip->batt_info_num = len / BATT_SN_LEN;
	for (i = 0; i < chip->batt_info_num; i++) {
		memmove(chip->dt_batt_info[i], &sn_total[i * BATT_SN_LEN], BATT_SN_LEN);
		chip->dt_batt_info[i][BATT_SN_LEN] = '\0';
		chg_info("dt_batt_info_%d: %s\n", i, chip->dt_batt_info[i]);
	}
}

static void sc5891_ic_unregister(struct sc5891_device *chip)
{
	if (chip->ic_dev)
		devm_oplus_chg_ic_unregister(chip->dev, chip->ic_dev);
	chip->ic_dev = NULL;
}

static int sc5891_ic_register(struct sc5891_device *chip)
{
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	struct oplus_chg_ic_cfg ic_cfg;
	int rc;

	rc = of_property_read_u32(chip->dev->of_node, "oplus,ic_type", &ic_type);
	if (rc < 0)
		return -ENODEV;
	rc = of_property_read_u32(chip->dev->of_node, "oplus,ic_index", &ic_index);
	if (rc < 0)
		return -ENODEV;
	rc = of_property_read_u32(chip->dev->of_node, "oplus,prikey_index", &chip->prikey_index);
	if (rc < 0)
		chip->prikey_index = SC5891_PRIKEY_DEFAULT_INDEX;
	rc = of_property_read_u32(chip->dev->of_node, "oplus,sc5891_fault_mitigation", &chip->sc5891_fault_mitigation);
	if (rc < 0)
		chip->sc5891_fault_mitigation = 0;
	chg_info("sc5891_fault_mitigation: %d\n", chip->sc5891_fault_mitigation);
	sc5891_parse_batt_info(chip);

	ic_cfg.name = chip->dev->of_node->name;
	ic_cfg.index = ic_index;
	ic_cfg.type = ic_type;
	ic_cfg.priv_data = chip;
	ic_cfg.virq_data = sc5891_virq_table;
	ic_cfg.virq_num = ARRAY_SIZE(sc5891_virq_table);
	ic_cfg.of_node = chip->dev->of_node;
	if (ic_type != OPLUS_CHG_IC_GAUGE) {
		chg_err("not support ic_type(=%d)\n", ic_type);
		return -ENODEV;
	}

	snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "sec-sc5891");
	snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
	ic_cfg.get_func = sc5891_ic_get_func;
	chip->ic_dev = devm_oplus_chg_ic_register(chip->dev, &ic_cfg);
	if (!chip->ic_dev) {
		chg_err("register %s error\n", chip->dev->of_node->name);
		return -ENODEV;
	}
	chg_info("register %s\n", chip->dev->of_node->name);
	return 0;
}

static int sc5891_pinctrl_init(struct sc5891_device *chip)
{
	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		chg_err("get pinctrl fail\n");
		return -ENODEV;
	}
	chip->pinctrl_default = pinctrl_lookup_state(chip->pinctrl, "default");
	if (IS_ERR_OR_NULL(chip->pinctrl_default)) {
		chg_err("get default fail\n");
		return -ENODEV;
	}
	chip->pinctrl_output_low = pinctrl_lookup_state(chip->pinctrl, "output-low-state");
	if (IS_ERR_OR_NULL(chip->pinctrl_output_low)) {
		chg_err("get output-low-state fail\n");
		return -ENODEV;
	}

	return 0;
}

static void sc5891_pinctrl_release(struct sc5891_device *chip)
{
	mutex_lock(&chip->pinctrl_lock);
	if (chip != NULL)
		devm_pinctrl_put(chip->pinctrl);
	mutex_unlock(&chip->pinctrl_lock);
}

#define SC5891_UPLOAD_GAUGE_BUF_LEN 512
static void sc5891_upload_gauge_data(struct sc5891_device *chip)
{
	int page_start = 3;	/* first two page store batt_sn */
	int page_end = sc5891_data_cfg_table[SC5891_DATA_ID_MAX - 1].page_id;
	int i;
	int j;
	int rc;
	int offset = 0;
	int len;
	char *track_buf = NULL;
	uint8_t page_buf[SC5891_INFO_PAGE_SIZE] = {0};
	struct mms_msg *msg;
	struct oplus_mms *err_topic;

	if(chip == NULL) {
		chg_err("chip is NULL\n");
		return;
	}

	if (!chip->hardware_init_ok) {
		chg_err("track upload is not allowed\n");
		return;
	}

	track_buf = kzalloc(SC5891_UPLOAD_GAUGE_BUF_LEN, GFP_KERNEL);
	if (track_buf == NULL) {
		chg_err("memory alloc fail\n");
		goto err_out;
	}

	offset += scnprintf(track_buf + offset, SC5891_UPLOAD_GAUGE_BUF_LEN - offset, "$$data@@");
	for (i = page_start; i <= page_end; i++) {
		offset += scnprintf(track_buf + offset, SC5891_UPLOAD_GAUGE_BUF_LEN - offset, "[%d]", i);
		rc = sc5891_read_page(chip->ic_dev, i, page_buf, &len);
		if (rc < 0) {
			chg_err("read page[%d] fail\n", i);
			goto err_out;
		}
		for (j = 0; j < SC5891_INFO_PAGE_SIZE; j++)
			offset += scnprintf(track_buf + offset, SC5891_UPLOAD_GAUGE_BUF_LEN - offset,
				  "%02x", page_buf[j]);
	}
	track_buf[offset] = '\0';

	err_topic = oplus_mms_get_by_name("error");
	if (!err_topic) {
		chg_err("error topic not found\n");
		goto err_out;
	}

	msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_LOW, ERR_ITEM_SEC_IC_MEM_INFO, track_buf);
	if (msg == NULL) {
		chg_err("oplus_mms_alloc_str_msg fail\n");
		goto  err_out;
	}
	rc = oplus_mms_publish_msg_sync(err_topic, msg);
	if (rc < 0) {
		chg_err("publish error msg error, rc=%d,\n", rc);
		kfree(msg);
	}

err_out:
	kfree(track_buf);
}

static void sc5891_gauge_update_work(struct work_struct *work)
{
	struct sc5891_device *chip =
		container_of(work, struct sc5891_device, gauge_update_work);
	union mms_msg_data data = { 0 };
	static int last_cc = 0;

	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CC, &data, false);
	if (last_cc == data.intval)
		return;

	if (atomic_read(&chip->pm_suspended) == 1) {
		chg_err("in suspend");
		return;
	}

	last_cc = data.intval;
	sc5891_upload_gauge_data(chip);
}

static void sc5891_gauge_subs_callback(struct mms_subscribe *subs,
				       enum mms_msg_type type, u32 id, bool sync)
{
	struct sc5891_device *chip = subs->priv_data;

	switch (type) {
	case MSG_TYPE_TIMER:
		schedule_work(&chip->gauge_update_work);
		break;
	default:
		break;
	}
}

static void sc5891_subscribe_gauge_topic(struct oplus_mms *topic, void *priv_data)
{
	struct sc5891_device *chip = priv_data;

	chip->gauge_topic = topic;
	chip->gauge_subs =
		oplus_mms_subscribe(chip->gauge_topic, chip,
				    sc5891_gauge_subs_callback, "sc5891");
	if (IS_ERR_OR_NULL(chip->gauge_subs)) {
		chg_err("subscribe error topic error, rc=%ld\n",
			PTR_ERR(chip->gauge_subs));
		return;
	}
}

static void sc5891_debugfs_remove(struct sc5891_device *chip)
{
	debugfs_remove(chip->debugfs_sc5891);
}

static int sc5891_debugfs_init(struct sc5891_device *chip)
{
	struct dentry *debugfs_root;

	debugfs_root = oplus_chg_track_get_debugfs_root();
	if (debugfs_root == NULL) {
		return -ENOENT;
	}

	chip->debugfs_sc5891 = debugfs_create_dir("sc5891", debugfs_root);
	if (chip->debugfs_sc5891 == NULL) {
		chg_err("debugfs_create_dir failed\n");
		return -ENOENT;
	}

	chip->debug_sc5891_cmd = 0;
	chip->debug_sc5891_cmd_err = 0;
	chip->debug_sc5891_i2c_err = false;
	chip->debug_sc5891_dump_log = false;
	debugfs_create_u32("debug_sc5891_cmd", 0644, chip->debugfs_sc5891, &(chip->debug_sc5891_cmd));
	debugfs_create_u32("debug_sc5891_cmd_err", 0644, chip->debugfs_sc5891, &(chip->debug_sc5891_cmd_err));
	debugfs_create_bool("debug_sc5891_i2c_err", 0644, chip->debugfs_sc5891, &(chip->debug_sc5891_i2c_err));
	debugfs_create_bool("debug_sc5891_dump_log", 0644, chip->debugfs_sc5891, &(chip->debug_sc5891_dump_log));
	return 0;
}

static void sc5891_hardware_init_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct sc5891_device *chip =
		container_of(dwork, struct sc5891_device, hardware_init_work);
	int rc = 0;
	static int retry = OPLUS_CHG_IC_INIT_RETRY_MAX;

	rc = sc5891_hardware_init(chip);
	if (rc < 0) {
		if (retry > 0) {
			chg_err("hardware init retry, rc=%d\n", rc);
			retry--;
			schedule_delayed_work(&chip->hardware_init_work,
					      msecs_to_jiffies(5000));
		}
		return;
	}

	mutex_lock(&chip->flow_lock);
	sc5891_pinctrl_avoid(chip, true);
	rc = sc5891_extern_verify_cert(chip->pubkey, chip->cert);
	if (rc < 0)
		chg_err("cert verify failed\n");
	rc = sc5891_ic_ecdsa(chip, &chip->cert_valid);
	sc5891_ic_enter_shutdown(chip);
	sc5891_pinctrl_avoid(chip, false);
	mutex_unlock(&chip->flow_lock);
	chg_info("hardware init ok, ecdsa result:%d", chip->cert_valid);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0))
static int sc5891_probe(struct i2c_client *client)
#else
static int sc5891_probe(struct i2c_client *client,
const struct i2c_device_id *id)
#endif
{
	struct sc5891_device *chip;
	int rc;

	chg_info("sc5891 probe start.\n");
	chip = devm_kzalloc(&client->dev, sizeof(struct sc5891_device), GFP_KERNEL);
	if (!chip) {
		chg_err("kzalloc failed\n");
		return -ENOMEM;
	}

	chip->dev = &client->dev;
	chip->i2c = client;
	chip->hardware_init_ok = false;
	chip->cert_valid = false;
	i2c_set_clientdata(client, chip);
	mutex_init(&chip->cmd_rw_lock);
	mutex_init(&chip->pinctrl_lock);
	mutex_init(&chip->flow_lock);
	INIT_WORK(&chip->gauge_update_work, sc5891_gauge_update_work);
	INIT_DELAYED_WORK(&chip->hardware_init_work, sc5891_hardware_init_work);
	chip->rw_wake_lock = wakeup_source_register(chip->dev, "sc5891_rw_wakeup");
	rc = sc5891_pinctrl_init(chip);
	if (rc < 0) {
		chg_err("pinctrl init error, rc=%d\n", rc);
		goto error_exit;
	}

	rc = sc5891_ic_register(chip);
	if (rc < 0) {
		chg_err("ic register error, rc=%d\n", rc);
		goto error_exit;
	}

	rc = sc5891_debugfs_init(chip);
	if (rc < 0) {
		chg_err("debugfs init error, rc=%d\n", rc);
		goto debugfs_err;
	}

	schedule_delayed_work(&chip->hardware_init_work, 0);
	oplus_mms_wait_topic("gauge", sc5891_subscribe_gauge_topic, chip);
	chg_info("sc5891 probe succuessfully.\n");

	chip->support_eco_design = !!(oplus_chg_get_nvid_support_flags() & BIT(ECO_DESIGN_SUPPORT_REGION));
	INIT_DELAYED_WORK(&chip->get_manu_battinfo_work, oplus_sc5891_get_manu_battinfo_work);
	schedule_delayed_work(&chip->get_manu_battinfo_work, 0);
	return 0;

debugfs_err:
	sc5891_ic_unregister(chip);
error_exit:
	wakeup_source_unregister(chip->rw_wake_lock);
	devm_kfree(&client->dev, chip);
	return rc;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0))
static int sc5891_remove(struct i2c_client *client)
#else
static void sc5891_remove(struct i2c_client *client)
#endif
{
	struct sc5891_device *chip = i2c_get_clientdata(client);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0))
	if (chip == NULL)
		return -EINVAL;
#else
	if (chip == NULL)
		return;
#endif
	if (!IS_ERR_OR_NULL(chip->gauge_subs))
		oplus_mms_unsubscribe(chip->gauge_subs);
	sc5891_ic_enter_shutdown(chip);
	sc5891_debugfs_remove(chip);
	sc5891_ic_unregister(chip);
	sc5891_pinctrl_release(chip);
	wakeup_source_unregister(chip->rw_wake_lock);
	mutex_destroy(&chip->cmd_rw_lock);
	mutex_destroy(&chip->pinctrl_lock);
	mutex_destroy(&chip->flow_lock);
	devm_kfree(&client->dev, chip);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0))
	return 0;
#else
	return;
#endif
}

static void sc5891_shutdown(struct i2c_client *client)
{
	struct sc5891_device *chip = i2c_get_clientdata(client);
	bool valid;

	if (chip == NULL)
		return;

	sc5891_enter_shutdown(chip->ic_dev, &valid);
}

static const struct of_device_id sc5891_of_match[] = {
	{.compatible = "oplus,sc5891"},
	{},
};

static const struct i2c_device_id sc5891_id[] = {
	{"oplus,sc5891", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, sc5891_id);

static int sc5891_pm_suspend(struct device *dev_chip)
{
	struct i2c_client *client  = container_of(dev_chip, struct i2c_client, dev);
	struct sc5891_device *chip = i2c_get_clientdata(client);

	if (chip == NULL)
		return 0;

	atomic_set(&chip->pm_suspended, 1);
	return 0;
}

static int sc5891_pm_resume(struct device *dev_chip)
{
	struct i2c_client *client  = container_of(dev_chip, struct i2c_client, dev);
	struct sc5891_device *chip = i2c_get_clientdata(client);

	if (chip == NULL)
		return 0;

	atomic_set(&chip->pm_suspended, 0);
	return 0;
}

static const struct dev_pm_ops sc5891_pm_ops = {
	.resume = sc5891_pm_resume,
	.suspend = sc5891_pm_suspend,
};

static struct i2c_driver sc5891_i2c_driver = {
	.probe = sc5891_probe,
	.remove = sc5891_remove,
	.shutdown = sc5891_shutdown,
	.driver = {
		.name = "sc5891-driver",
		.of_match_table = of_match_ptr(sc5891_of_match),
		.pm = &sc5891_pm_ops,
	}
};

static __init int sc5891_driver_init(void)
{
	return i2c_add_driver(&sc5891_i2c_driver);
}

static __exit void sc5891_driver_exit(void)
{
	i2c_del_driver(&sc5891_i2c_driver);
}

oplus_chg_module_register(sc5891_driver);

MODULE_DESCRIPTION("SC5891 Driver");
MODULE_LICENSE("GPL v2");
