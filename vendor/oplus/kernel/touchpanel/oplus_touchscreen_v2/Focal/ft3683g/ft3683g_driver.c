// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/string.h>
#include <linux/err.h>
#include <linux/thermal.h>
#include "ft3683g_core.h"
#include "../../touchpanel_healthinfo/touchpanel_exception.h"

struct chip_data_ft3683g *g_fts_data = NULL;

/*******Part0:LOG TAG Declear********************/

#ifdef TPD_DEVICE
#undef TPD_DEVICE
#define TPD_DEVICE "focaltech-FT3683g"
#else
#define TPD_DEVICE "focaltech-FT3683g"
#endif
#define TPD_INFO(a, arg...)  pr_err("[TP]"TPD_DEVICE ": " a, ##arg)

#define FTS_REG_UPGRADE                             0xFC
#define FTS_UPGRADE_AA                              0xAA
#define FTS_UPGRADE_55                              0x55
#define FTS_DELAY_UPGRADE_AA                        10
#define FTS_DELAY_UPGRADE_RESET                     80
#define FTS_UPGRADE_LOOP                            10

#define FTS_ROMBOOT_CMD_SET_PRAM_ADDR               0xAD
#define FTS_ROMBOOT_CMD_SET_PRAM_ADDR_LEN           4
#define FTS_ROMBOOT_CMD_WRITE                       0xAE
#define FTS_ROMBOOT_CMD_START_APP                   0x08
#define FTS_DELAY_PRAMBOOT_START                    100
#define FTS_ROMBOOT_CMD_ECC                         0xCC
#define FTS_ROMBOOT_CMD_ECC_NEW_LEN                 7
#define FTS_ECC_FINISH_TIMEOUT                      100
#define FTS_ROMBOOT_CMD_ECC_FINISH                  0xCE
#define FTS_ROMBOOT_CMD_ECC_READ                    0xCD
#define FTS_PRAM_SADDR                              0x000000
#define FTS_DRAM_SADDR                              0xD00000
#define FTS_DELAY_READ_ID                           20

#define FTS_CMD_RESET                               0x07
#define FTS_CMD_START                               0x55
#define FTS_CMD_START_DELAY                         12
#define FTS_CMD_READ_ID                             0x90
#define FTS_CMD_DATA_LEN                            0x7A
#define FTS_CMD_ERASE_APP                           0x61
#define FTS_RETRIES_REASE                           50
#define FTS_RETRIES_DELAY_REASE                     400
#define FTS_REASE_APP_DELAY                         1350
#define FTS_CMD_ECC_INIT                            0x64
#define FTS_CMD_ECC_CAL                             0x65
#define FTS_RETRIES_ECC_CAL                         10
#define FTS_RETRIES_DELAY_ECC_CAL                   50
#define FTS_CMD_ECC_READ                            0x66
#define FTS_CMD_FLASH_STATUS                        0x6A
#define FTS_CMD_WRITE                               0xBF
#define FTS_CMD_SET_WFLASH_ADDR                     0xAB
#define FTS_CMD_SET_RFLASH_ADDR                     0xAC
#define FTS_RETRIES_WRITE                           100
#define FTS_RETRIES_DELAY_WRITE                     1
#define FTS_REG_RESET_REASON                        0xC4

#define FTS_CMD_GAME_AIUINIT_EN                     0xC9
#define FTS_CMD_GAME_AIUINIT                        0xCA

#define FTS_CMD_FLASH_STATUS_NOP                    0x0000
#define FTS_CMD_FLASH_STATUS_ECC_OK                 0xF055
#define FTS_CMD_FLASH_STATUS_ERASE_OK               0xF0AA
#define FTS_CMD_FLASH_STATUS_WRITE_OK               0x1000

#define POINT_REPORT_CHECK_WAIT_TIME                200    /* unit:ms */
#define PRC_INTR_INTERVALS                          100    /* unit:ms */

/*********************************************************
 *              proc/ftxxxx-debug                        *
 *********************************************************/
#define PROC_READ_REGISTER                      1
#define PROC_WRITE_REGISTER                     2
#define PROC_WRITE_DATA                         6
#define PROC_READ_DATA                          7
#define PROC_SET_TEST_FLAG                      8
#define PROC_HW_RESET                           11
#define PROC_READ_STATUS                        12
#define PROC_SET_BOOT_MODE                      13
#define PROC_ENTER_TEST_ENVIRONMENT             14
#define PROC_WRITE_DATA_DIRECT                  16
#define PROC_READ_DATA_DIRECT                   17
#define PROC_CONFIGURE                          18
#define PROC_CONFIGURE_INTR                     20
#define PROC_GET_DRIVER_INFO                    21
#define PROC_NAME                               "ftxxxx-debug"
#define PROC_BUF_SIZE                           256

#define AL2_FCS_COEF                ((1 << 15) + (1 << 10) + (1 << 3))

#define SET_REG(bit, val) do { \
	ts_data->ctrl_reg_state &= (~(0x03 << bit)); \
	ts_data->ctrl_reg_state |= ((val & 0x03) << bit);  \
} while (0)

enum GESTURE_ID {
	GESTURE_FINGER_PRINT_ERROR = 0x16,
	GESTURE_RIGHT2LEFT_SWIP = 0x20,
	GESTURE_LEFT2RIGHT_SWIP = 0x21,
	GESTURE_DOWN2UP_SWIP = 0x22,
	GESTURE_UP2DOWN_SWIP = 0x23,
	GESTURE_DOUBLE_TAP = 0x24,
	GESTURE_DOUBLE_SWIP = 0x25,
	GESTURE_RIGHT_VEE = 0x51,
	GESTURE_LEFT_VEE = 0x52,
	GESTURE_DOWN_VEE = 0x53,
	GESTURE_UP_VEE = 0x54,
	GESTURE_O_CLOCKWISE = 0x57,
	GESTURE_O_ANTICLOCK = 0x30,
	GESTURE_W = 0x31,
	GESTURE_M = 0x32,
	GESTURE_FINGER_PRINT = 0x26,
	GESTURE_SINGLE_TAP = 0x27,
	GESTURE_HEART_ANTICLOCK = 0x55,
	GESTURE_HEART_CLOCKWISE = 0x59,
};

static void focal_esd_check_enable(void *chip_data, bool enable);
static int fts_hw_reset(struct chip_data_ft3683g *ts_data, u32 delayms);




/*************************************************************
 *******************FocalTech SPI protocols*******************
 *************************************************************/

#define SPI_RETRY_NUMBER            3
#define CS_HIGH_DELAY               150 /* unit: us */
#define SPI_BUF_LENGTH              4096

#define DATA_CRC_EN                 0x20
#define WRITE_CMD                   0x00
#define READ_CMD                    (0x80 | DATA_CRC_EN)

#define SPI_DUMMY_BYTE              3
#define SPI_HEADER_LENGTH           6   /*CRC*/
static void fts_get_rawdata_snr(struct chip_data_ft3683g *ts_data);
static void fts_rate_white_list_ctrl(void *chip_data, int value);
static int fts_reset(void *chip_data);
/* spi interface */
static int fts_spi_transfer(struct spi_device *spi, u8 *tx_buf, u8 *rx_buf, u32 len)
{
	int ret = 0;
	struct spi_message msg;
	struct spi_transfer xfer = {
		.tx_buf = tx_buf,
		.rx_buf = rx_buf,
		.len    = len,
	};

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	ret = spi_sync(spi, &msg);
	if (ret) {
		TPD_INFO("spi_sync fail,ret:%d", ret);
		return ret;
	}

	return ret;
}

static void fts_spi_buf_show(u8 *data, int datalen)
{
	int i = 0;
	int count = 0;
	int size = 0;
	char *tmpbuf = NULL;

	if (!data || (datalen <= 0)) {
		TPD_INFO("data/datalen is invalid");
		return;
	}

	size = (datalen > 256) ? 256 : datalen;
	tmpbuf = kzalloc(1024, GFP_KERNEL);
	if (!tmpbuf) {
		TPD_INFO("tmpbuf zalloc fail");
		return;
	}

	for (i = 0; i < size; i++)
		count += snprintf(tmpbuf + count, 1024 - count, "%02X ", data[i]);

	TPD_INFO("%s", tmpbuf);
	if (tmpbuf) {
		kfree(tmpbuf);
		tmpbuf = NULL;
	}
}

static void crckermit(u8 *data, u32 len, u16 *crc_out)
{
	u32 i = 0;
	u32 j = 0;
	u16 crc = 0xFFFF;

	for (i = 0; i < len; i++) {
		crc ^= data[i];
		for (j = 0; j < 8; j++) {
			if (crc & 0x01)
				crc = (crc >> 1) ^ 0x8408;
			else
				crc = (crc >> 1);
		}
	}

	*crc_out = crc;
}

static int rdata_check(u8 *rdata, u32 rlen)
{
	u16 crc_calc = 0;
	u16 crc_read = 0;

	crckermit(rdata, rlen - 2, &crc_calc);
	crc_read = (u16)(rdata[rlen - 1] << 8) + rdata[rlen - 2];
	if (crc_calc != crc_read) {
		fts_spi_buf_show(rdata, rlen);
		return -EIO;
	}

	return 0;
}

int fts_write(u8 *writebuf, u32 writelen)
{
	int ret = 0;
	int ret_err = -1;
	int i = 0;
	struct chip_data_ft3683g *ts_data = g_fts_data;
	u8 *txbuf = NULL;
	u8 *rxbuf = NULL;
	u32 txlen = 0;
	u32 txlen_need = writelen + SPI_HEADER_LENGTH + SPI_DUMMY_BYTE;
	u32 datalen = writelen - 1;

	if (!ts_data || !ts_data->ft_spi) {
		TPD_INFO("ts_data/ft_spi is invalid");
		return -EINVAL;
	}

	if (!writebuf || !writelen) {
		TPD_INFO("writebuf/len is invalid");
		return -EINVAL;
	}

	mutex_lock(&ts_data->bus_lock);
	if (txlen_need > SPI_BUF_LENGTH) {
		txbuf = kzalloc(txlen_need, GFP_KERNEL);
		if (NULL == txbuf) {
			TPD_INFO("txbuf malloc fail");
			ret = -ENOMEM;
			goto err_write;
		}

		rxbuf = kzalloc(txlen_need, GFP_KERNEL);
		if (NULL == rxbuf) {
			TPD_INFO("rxbuf malloc fail");
			ret = -ENOMEM;
			goto err_write;
		}
	} else {
		txbuf = ts_data->bus_tx_buf;
		rxbuf = ts_data->bus_rx_buf;
		memset(txbuf, 0x0, SPI_BUF_LENGTH);
		memset(rxbuf, 0x0, SPI_BUF_LENGTH);
	}

	txbuf[txlen++] = writebuf[0];
	txbuf[txlen++] = WRITE_CMD;
	txbuf[txlen++] = (datalen >> 8) & 0xFF;
	txbuf[txlen++] = datalen & 0xFF;
	if (datalen > 0) {
		txlen = txlen + SPI_DUMMY_BYTE;
		memcpy(&txbuf[txlen], &writebuf[1], datalen);
		txlen = txlen + datalen;
	}

	for (i = 0; i < SPI_RETRY_NUMBER; i++) {
		ret = fts_spi_transfer(ts_data->ft_spi, txbuf, rxbuf, txlen);
		if ((0 == ret) && ((rxbuf[3] & 0xA0) == 0)) {
			break;
		} else {
			TPD_INFO("data write(addr:%x),status:%x,retry:%d,ret:%d",
			         writebuf[0], rxbuf[3], i, ret);
			ret = -EIO;
			udelay(CS_HIGH_DELAY);
		}
	}
	if (ret < 0) {
		TPD_INFO("data write(addr:%x) fail,status:%x,ret:%d",
		         writebuf[0], rxbuf[3], ret);
	}
	if (ts_data->monitor_data && ts_data->monitor_data->health_monitor_support
			   && (ret < 0 || ts_data->monitor_data->health_simulate_trigger)) {
		ts_data->monitor_data->bus_buf = writebuf;
		ts_data->monitor_data->bus_len = writelen;
		tp_healthinfo_report(ts_data->monitor_data, HEALTH_BUS,
			   ts_data->monitor_data->health_simulate_trigger ? &ret_err : &ret);
	}

err_write:
	if (txlen_need > SPI_BUF_LENGTH) {
		if (txbuf) {
			kfree(txbuf);
			txbuf = NULL;
		}

		if (rxbuf) {
			kfree(rxbuf);
			rxbuf = NULL;
		}
	}

	udelay(CS_HIGH_DELAY);
	mutex_unlock(&ts_data->bus_lock);
	return ret;
}

int fts_write_reg(u8 addr, u8 value)
{
	u8 writebuf[2] = { 0 };

	writebuf[0] = addr;
	writebuf[1] = value;
	return fts_write(writebuf, 2);
}

int fts_read(u8 *cmd, u32 cmdlen, u8 *data, u32 datalen)
{
	int ret = 0;
	int ret_err = -1;
	int i = 0;
	struct chip_data_ft3683g *ts_data = g_fts_data;
	u8 *txbuf = NULL;
	u8 *rxbuf = NULL;
	u32 txlen = 0;
	u32 txlen_need = datalen + SPI_HEADER_LENGTH + SPI_DUMMY_BYTE;
	u8 ctrl = READ_CMD;
	u32 dp = 0;

	if (!ts_data || !ts_data->ft_spi) {
		TPD_INFO("ts_data/ft_spi is invalid");
		return -EINVAL;
	}

	if (!cmd || !cmdlen || !data || !datalen) {
		TPD_INFO("cmd/cmdlen/data/datalen is invalid");
		return -EINVAL;
	}

	mutex_lock(&ts_data->bus_lock);
	if (txlen_need > SPI_BUF_LENGTH) {
		txbuf = kzalloc(txlen_need, GFP_KERNEL);
		if (NULL == txbuf) {
			TPD_INFO("txbuf malloc fail");
			ret = -ENOMEM;
			goto err_read;
		}

		rxbuf = kzalloc(txlen_need, GFP_KERNEL);
		if (NULL == rxbuf) {
			TPD_INFO("rxbuf malloc fail");
			ret = -ENOMEM;
			goto err_read;
		}
	} else {
		txbuf = ts_data->bus_tx_buf;
		rxbuf = ts_data->bus_rx_buf;
		memset(txbuf, 0x0, SPI_BUF_LENGTH);
		memset(rxbuf, 0x0, SPI_BUF_LENGTH);
	}

	txbuf[txlen++] = cmd[0];
	txbuf[txlen++] = ctrl;
	txbuf[txlen++] = (datalen >> 8) & 0xFF;
	txbuf[txlen++] = datalen & 0xFF;
	dp = txlen + SPI_DUMMY_BYTE;
	txlen = dp + datalen;
	if (ctrl & DATA_CRC_EN) {
		txlen = txlen + 2;
	}

	for (i = 0; i < SPI_RETRY_NUMBER; i++) {
		ret = fts_spi_transfer(ts_data->ft_spi, txbuf, rxbuf, txlen);
		if ((0 == ret) && ((rxbuf[3] & 0xA0) == 0)) {
			memcpy(data, &rxbuf[dp], datalen);
			/* crc check */
			if (ctrl & DATA_CRC_EN) {
				ret = rdata_check(&rxbuf[dp], txlen - dp);
				if (ret < 0) {
					TPD_INFO("data read(addr:%x) crc abnormal,retry:%d",
					         cmd[0], i);
					udelay(CS_HIGH_DELAY);
					continue;
				}
			}
			break;
		} else {
			TPD_INFO("data read(addr:%x) status:%x,retry:%d,ret:%d",
			         cmd[0], rxbuf[3], i, ret);
			ret = -EIO;
			udelay(CS_HIGH_DELAY);
		}
	}

	if (ret < 0) {
		TPD_INFO("data read(addr:%x) %s,status:%x,ret:%d", cmd[0],
		         (i >= SPI_RETRY_NUMBER) ? "crc abnormal" : "fail",
		         rxbuf[3], ret);
	}

	if (ts_data->monitor_data && ts_data->monitor_data->health_monitor_support
			   && (ret < 0 || ts_data->monitor_data->health_simulate_trigger)) {
		ts_data->monitor_data->bus_buf = cmd;
		ts_data->monitor_data->bus_len = cmdlen;
		tp_healthinfo_report(ts_data->monitor_data, HEALTH_BUS,
			   ts_data->monitor_data->health_simulate_trigger ? &ret_err : &ret);
	}

err_read:
	if (txlen_need > SPI_BUF_LENGTH) {
		if (txbuf) {
			kfree(txbuf);
			txbuf = NULL;
		}

		if (rxbuf) {
			kfree(rxbuf);
			rxbuf = NULL;
		}
	}

	udelay(CS_HIGH_DELAY);
	mutex_unlock(&ts_data->bus_lock);
	return ret;
}

int fts_read_reg(u8 addr, u8 *value)
{
	return fts_read(&addr, 1, value, 1);
}

static int fts_spi_transfer_direct(u8 *writebuf, u32 writelen, u8 *readbuf, u32 readlen)
{
	int ret = 0;
	struct chip_data_ft3683g *ts_data = g_fts_data;
	u8 *txbuf = NULL;
	u8 *rxbuf = NULL;
	bool read_cmd = (readbuf && readlen) ? 1 : 0;
	u32 txlen = (read_cmd) ? readlen : writelen;

	if (!writebuf || !writelen) {
		TPD_INFO("writebuf/len is invalid");
		return -EINVAL;
	}

	mutex_lock(&ts_data->bus_lock);
	if (txlen > SPI_BUF_LENGTH) {
		txbuf = kzalloc(txlen, GFP_KERNEL);
		if (NULL == txbuf) {
			TPD_INFO("txbuf malloc fail");
			ret = -ENOMEM;
			goto err_spi_dir;
		}

		rxbuf = kzalloc(txlen, GFP_KERNEL);
		if (NULL == rxbuf) {
			TPD_INFO("rxbuf malloc fail");
			ret = -ENOMEM;
			goto err_spi_dir;
		}
	} else {
		txbuf = ts_data->bus_tx_buf;
		rxbuf = ts_data->bus_rx_buf;
		memset(txbuf, 0x0, SPI_BUF_LENGTH);
		memset(rxbuf, 0x0, SPI_BUF_LENGTH);
	}

	memcpy(txbuf, writebuf, writelen);
	ret = fts_spi_transfer(ts_data->ft_spi, txbuf, rxbuf, txlen);
	if (ret < 0) {
		TPD_INFO("data read(addr:%x) fail,status:%x,ret:%d", txbuf[0], rxbuf[3], ret);
		goto err_spi_dir;
	}

	if (read_cmd) {
		memcpy(readbuf, rxbuf, txlen);
	}

	ret = 0;
err_spi_dir:
	if (txlen > SPI_BUF_LENGTH) {
		if (txbuf) {
			kfree(txbuf);
			txbuf = NULL;
		}

		if (rxbuf) {
			kfree(rxbuf);
			rxbuf = NULL;
		}
	}

	udelay(CS_HIGH_DELAY);
	mutex_unlock(&ts_data->bus_lock);
	return ret;
}

int fts_spi_write_direct(u8 *writebuf, u32 writelen)
{
	int ret = 0;
	u8 *readbuf = NULL;

	ret = fts_spi_transfer_direct(writebuf, writelen, readbuf, 0);
	if (ret < 0)
		return ret;
	else
		return 0;
}

int fts_spi_read_direct(u8 *writebuf, u32 writelen, u8 *readbuf, u32 readlen)
{
	int ret = 0;

	ret = fts_spi_transfer_direct(writebuf, writelen, readbuf, readlen);
	if (ret < 0)
		return ret;
	else
		return 0;
}

static int fts_bus_init(struct chip_data_ft3683g *ts_data)
{
	ts_data->bus_tx_buf = kzalloc(SPI_BUF_LENGTH, GFP_KERNEL);
	if (NULL == ts_data->bus_tx_buf) {
		TPD_INFO("failed to allocate memory for bus_tx_buf");
		return -ENOMEM;
	}

	ts_data->bus_rx_buf = kzalloc(SPI_BUF_LENGTH, GFP_KERNEL);
	if (NULL == ts_data->bus_rx_buf) {
		kfree(ts_data->bus_tx_buf);
		ts_data->bus_tx_buf = NULL;
		TPD_INFO("failed to allocate memory for bus_rx_buf");
		return -ENOMEM;
	}

	mutex_init(&ts_data->bus_lock);
	return 0;
}

static int fts_bus_exit(struct chip_data_ft3683g *ts_data)
{
	if (ts_data && ts_data->bus_tx_buf) {
		kfree(ts_data->bus_tx_buf);
		ts_data->bus_tx_buf = NULL;
	}

	if (ts_data && ts_data->bus_rx_buf) {
		kfree(ts_data->bus_rx_buf);
		ts_data->bus_rx_buf = NULL;
	}
	return 0;
}


static ssize_t fts_debug_write(struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
	u8 *writebuf = NULL;
	u8 tmpbuf[PROC_BUF_SIZE] = { 0 };
	int buflen = count;
	int writelen = 0;
	int ret = 0;
	char tmp[PROC_BUF_SIZE];
	struct chip_data_ft3683g *ts_data = PDE_DATA(file_inode(filp));
	struct ftxxxx_proc *proc = &ts_data->proc;

	if (buflen < 1) {
		TPD_INFO("apk proc wirte count(%d) fail", buflen);
		return -EINVAL;
	}

	if (buflen > PROC_BUF_SIZE) {
		writebuf = (u8 *)kzalloc(buflen * sizeof(u8), GFP_KERNEL);
		if (NULL == writebuf) {
			TPD_INFO("apk proc wirte buf zalloc fail");
			return -ENOMEM;
		}
	} else {
		writebuf = tmpbuf;
	}

	if (copy_from_user(writebuf, buff, buflen)) {
		TPD_INFO("[APK]: copy from user error!!");
		ret = -EFAULT;
		goto proc_write_err;
	}

	proc->opmode = writebuf[0];
	if (buflen == 1) {
		ret = buflen;
		goto proc_write_err;
	}

	switch (proc->opmode) {
	case PROC_SET_TEST_FLAG:
		TPD_INFO("[APK]: PROC_SET_TEST_FLAG = %x", writebuf[1]);
		if (writebuf[1] == 0) {
			focal_esd_check_enable(ts_data, true);
		} else {
			focal_esd_check_enable(ts_data, false);
		}
		break;

	case PROC_READ_REGISTER:
		proc->cmd[0] = writebuf[1];
		break;

	case PROC_WRITE_REGISTER:
		ret = fts_write_reg(writebuf[1], writebuf[2]);
		if (ret < 0) {
			TPD_INFO("PROC_WRITE_REGISTER write error");
			goto proc_write_err;
		}
		break;

	case PROC_READ_DATA:
		writelen = buflen - 1;
		if (writelen >= FTS_MAX_COMMMAND_LENGTH) {
			TPD_INFO("cmd(PROC_READ_DATA) length(%d) fail", writelen);
			goto proc_write_err;
		}
		memcpy(proc->cmd, writebuf + 1, writelen);
		proc->cmd_len = writelen;
		break;

	case PROC_WRITE_DATA:
		writelen = buflen - 1;
		ret = fts_write(writebuf + 1, writelen);
		if (ret < 0) {
			TPD_INFO("PROC_WRITE_DATA write error");
			goto proc_write_err;
		}
		break;

	case PROC_HW_RESET:
		if (buflen < PROC_BUF_SIZE) {
			snprintf(tmp, PROC_BUF_SIZE, "%s", writebuf + 1);
			tmp[buflen - 1] = '\0';
			if (strncmp(tmp, "focal_driver", 12) == 0) {
				TPD_INFO("APK execute HW Reset");
				fts_hw_reset(ts_data, 0);
			}
		}
		break;

	case PROC_READ_DATA_DIRECT:
		writelen = buflen - 1;
		if (writelen >= FTS_MAX_COMMMAND_LENGTH) {
			TPD_INFO("cmd(PROC_READ_DATA_DIRECT) length(%d) fail", writelen);
			goto proc_write_err;
		}
		memcpy(proc->cmd, writebuf + 1, writelen);
		proc->cmd_len = writelen;
		break;

	case PROC_WRITE_DATA_DIRECT:
		writelen = buflen - 1;
		ret = fts_spi_transfer_direct(writebuf + 1, writelen, NULL, 0);
		if (ret < 0) {
			TPD_INFO("PROC_WRITE_DATA_DIRECT write error");
			goto proc_write_err;
		}
		break;

	case PROC_CONFIGURE:
		ts_data->ft_spi->mode = writebuf[1];
		ts_data->ft_spi->bits_per_word = writebuf[2];
		ts_data->ft_spi->max_speed_hz = *(u32 *)(writebuf + 4);
		TPD_INFO("spi,mode=%d,bits=%d,speed=%d", ts_data->ft_spi->mode,
		         ts_data->ft_spi->bits_per_word, ts_data->ft_spi->max_speed_hz);
		ret = spi_setup(ts_data->ft_spi);
		if (ret) {
			TPD_INFO("spi setup fail");
			goto proc_write_err;
		}
		break;

	case PROC_CONFIGURE_INTR:
		if (writebuf[1] == 0)
			disable_irq_nosync(ts_data->ts->irq);
		else
			enable_irq(ts_data->ts->irq);
		break;

	default:
		break;
	}

	ret = buflen;
proc_write_err:
	if ((buflen > PROC_BUF_SIZE) && writebuf) {
		kfree(writebuf);
		writebuf = NULL;
	}

	return ret;
}

static ssize_t fts_debug_read(struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
	int ret = 0;
	int num_read_chars = 0;
	int buflen = count;
	u8 *readbuf = NULL;
	u8 tmpbuf[PROC_BUF_SIZE] = { 0 };
	struct chip_data_ft3683g *ts_data = PDE_DATA(file_inode(filp));
	struct ftxxxx_proc *proc = &ts_data->proc;

	if (buflen <= 0) {
		TPD_INFO("apk proc read count(%d) fail", buflen);
		return -EINVAL;
	}

	if (buflen > PROC_BUF_SIZE) {
		readbuf = (u8 *)kzalloc(buflen * sizeof(u8), GFP_KERNEL);
		if (NULL == readbuf) {
			TPD_INFO("apk proc wirte buf zalloc fail");
			return -ENOMEM;
		}
	} else {
		readbuf = tmpbuf;
	}

	switch (proc->opmode) {
	case PROC_READ_REGISTER:
		num_read_chars = 1;
		ret = fts_read_reg(proc->cmd[0], &readbuf[0]);
		if (ret < 0) {
			TPD_INFO("PROC_READ_REGISTER read error");
			goto proc_read_err;
		}
		break;

	case PROC_READ_DATA:
		num_read_chars = buflen;
		ret = fts_read(proc->cmd, proc->cmd_len, readbuf, num_read_chars);
		if (ret < 0) {
			TPD_INFO("PROC_READ_DATA read error");
			goto proc_read_err;
		}
		break;

	case PROC_READ_DATA_DIRECT:
		num_read_chars = buflen;
		ret = fts_spi_transfer_direct(proc->cmd, proc->cmd_len, readbuf, num_read_chars);
		if (ret < 0) {
			TPD_INFO("PROC_READ_DATA_DIRECT read error");
			goto proc_read_err;
		}
		break;

	case PROC_GET_DRIVER_INFO:
		if (buflen >= 64) {
			num_read_chars = buflen;
			readbuf[0] = 3;
			snprintf(&readbuf[32], buflen - 32, "Focaltech V3.4 20211214");
		}
		break;

	default:
		break;
	}

	ret = num_read_chars;
proc_read_err:
	if ((num_read_chars > 0) && copy_to_user(buff, readbuf, num_read_chars)) {
		TPD_INFO("copy to user error");
		ret = -EFAULT;
	}

	if ((buflen > PROC_BUF_SIZE) && readbuf) {
		kfree(readbuf);
		readbuf = NULL;
	}

	return ret;
}

DECLARE_PROC_OPS(fts_proc_fops, simple_open, fts_debug_read, fts_debug_write, NULL);

static int fts_ta_open(struct inode *inode, struct file *file)
{
	struct chip_data_ft3683g *ts_data = PDE_DATA(inode);

	if (ts_data->touch_analysis_support) {
		TPD_INFO("fts_ta open");
		ts_data->ta_buf = kzalloc(FTS_MAX_TOUCH_BUF, GFP_KERNEL);
		if (!ts_data->ta_buf) {
			TPD_INFO("kzalloc for ta_buf fails");
			return -ENOMEM;
		}
	}
	return 0;
}

static int fts_ta_release(struct inode *inode, struct file *file)
{
	struct chip_data_ft3683g *ts_data = PDE_DATA(inode);

	if (ts_data->touch_analysis_support) {
		TPD_INFO("fts_ta close");
		ts_data->ta_flag = 0;
		if (ts_data->ta_buf) {
			kfree(ts_data->ta_buf);
			ts_data->ta_buf = NULL;
		}
	}
	return 0;
}

static ssize_t fts_ta_read(struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
	int read_num = (int)count;
	struct chip_data_ft3683g *ts_data = PDE_DATA(file_inode(filp));

	if (!ts_data->touch_analysis_support || !ts_data->ta_buf) {
		TPD_INFO("touch_analysis is disabled, or ta_buf is NULL");
		return -EINVAL;
	}

	if (!(filp->f_flags & O_NONBLOCK)) {
		ts_data->ta_flag = 1;
		wait_event_interruptible(ts_data->ts_waitqueue, !ts_data->ta_flag);
	}

	read_num = (ts_data->ta_size < read_num) ? ts_data->ta_size : read_num;
	if ((read_num > 0) && (copy_to_user(buff, ts_data->ta_buf, read_num))) {
		TPD_INFO("copy to user error");
		return -EFAULT;
	}

	return read_num;
}

DECLARE_PROC_OPS(fts_procta_fops, fts_ta_open, fts_ta_read, NULL, fts_ta_release);

static int fts_create_apk_debug_channel(struct chip_data_ft3683g *ts_data)
{
	struct ftxxxx_proc *proc = &ts_data->proc;

	proc->proc_entry = proc_create_data(PROC_NAME, 0777, NULL, &fts_proc_fops, ts_data);
	if (NULL == proc->proc_entry) {
		TPD_INFO("create proc entry fail");
		return -ENOMEM;
	}

	ts_data->proc_ta.proc_entry = proc_create_data("fts_ta", 0777, NULL, \
	                              &fts_procta_fops, ts_data);
	if (!ts_data->proc_ta.proc_entry) {
		TPD_INFO("create proc_ta entry fail");
		return -ENOMEM;
	}
	TPD_INFO("Create proc entry success!");
	return 0;
}

static void fts_release_apk_debug_channel(struct chip_data_ft3683g *ts_data)
{
	struct ftxxxx_proc *proc = &ts_data->proc;

	if (proc->proc_entry) {
		proc_remove(proc->proc_entry);
	}

	if (ts_data->proc_ta.proc_entry)
		proc_remove(ts_data->proc_ta.proc_entry);
}


static ssize_t fts_prc_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct touchpanel_data *ts = dev_get_drvdata(dev);
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)ts->chip_data;

	mutex_lock(&ts->mutex);
	if (buf[0] == '1') {
		TPD_INFO("enable prc");
		ts_data->prc_support = 1;
	} else if (buf[0] == '0') {
		TPD_INFO("disable prc");
		cancel_delayed_work_sync(&ts_data->prc_work);
		ts_data->prc_support = 0;
	}
	mutex_unlock(&ts->mutex);

	return count;
}

static ssize_t fts_prc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	struct touchpanel_data *ts = dev_get_drvdata(dev);
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)ts->chip_data;

	count = snprintf(buf, PAGE_SIZE, "PRC: %s\n", \
	                 ts_data->prc_support ? "Enable" : "Disable");

	return count;
}


/* fts_touch_size node */
static ssize_t fts_touchsize_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count = 0;
	struct touchpanel_data *ts = dev_get_drvdata(dev);
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)ts->chip_data;

	count += snprintf(buf + count, PAGE_SIZE, "touch size:%d\n", ts_data->touch_size);

	return count;
}

static ssize_t fts_touchsize_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value = 0;
	struct touchpanel_data *ts = dev_get_drvdata(dev);
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)ts->chip_data;

	sscanf(buf, "%d", &value);
	if ((value > 2) && (value < FTS_MAX_TOUCH_BUF)) {
		TPD_INFO("touch size:%d->%d", ts_data->touch_size, value);
		ts_data->touch_size = value;
	} else
		TPD_INFO("touch size:%d invalid", value);

	return count;
}

/* fts_ta_mode node */
static ssize_t fts_tamode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count = 0;
	struct touchpanel_data *ts = dev_get_drvdata(dev);
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)ts->chip_data;

	count += snprintf(buf + count, PAGE_SIZE, "touch analysis:%s\n", \
	                  ts_data->touch_analysis_support ? "Enable" : "Disable");

	return count;
}

static ssize_t fts_tamode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value = 0;
	struct touchpanel_data *ts = dev_get_drvdata(dev);
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)ts->chip_data;

	sscanf(buf, "%d", &value);
	ts_data->touch_analysis_support = !!value;
	TPD_INFO("set touch analysis:%d", ts_data->touch_analysis_support);

	return count;
}

static DEVICE_ATTR(fts_prc, S_IRUGO | S_IWUSR, fts_prc_show, fts_prc_store);
static DEVICE_ATTR(fts_touch_size, S_IRUGO | S_IWUSR, fts_touchsize_show, fts_touchsize_store);
static DEVICE_ATTR(fts_ta_mode, S_IRUGO | S_IWUSR, fts_tamode_show, fts_tamode_store);

/* add your attr in here*/
static struct attribute *fts_attributes[] = {
	&dev_attr_fts_prc.attr,
	&dev_attr_fts_touch_size.attr,
	&dev_attr_fts_ta_mode.attr,
	NULL
};

static struct attribute_group fts_attribute_group = {
	.attrs = fts_attributes
};

static int fts_create_sysfs(struct chip_data_ft3683g *ts_data)
{
	int ret = 0;

	ret = sysfs_create_group(&ts_data->ts->dev->kobj, &fts_attribute_group);
	if (ret) {
		TPD_INFO("[EX]: sysfs_create_group() failed!!");
		sysfs_remove_group(&ts_data->ts->dev->kobj, &fts_attribute_group);
		return -ENOMEM;
	} else {
		TPD_INFO("[EX]: sysfs_create_group() succeeded!!");
	}

	return ret;
}

static int fts_remove_sysfs(struct chip_data_ft3683g *ts_data)
{
	sysfs_remove_group(&ts_data->ts->dev->kobj, &fts_attribute_group);
	return 0;
}




/*******Part1:Call Back Function implement*******/

static int fts_rstgpio_set(struct hw_resource *hw_res, bool on)
{
	if (gpio_is_valid(hw_res->reset_gpio)) {
		TPD_INFO("Set the reset_gpio \n");
		gpio_direction_output(hw_res->reset_gpio, on);

	} else {
		TPD_INFO("reset is invalid!!\n");
	}

	return 0;
}

static void fts_set_fp_error_report(void *chip_data, bool enable)
{
	int retval = 0;
	u8 regval = 0;

	TPD_INFO("%s: %s set fp error report.\n", __func__, enable ? "Enter" : "Exit");

	retval = fts_read_reg(FTS_REG_SET_FP_ERROR_REPORT, &regval);
	if(retval < 0) {
		TPD_INFO("Failed to get water mode config\n");
		return;
	}

	if(enable) {
		regval = regval | 0x80;
	} else {
		regval = regval & 0x7f;
	}

	retval = fts_write_reg(FTS_REG_SET_FP_ERROR_REPORT, regval);
	if(retval < 0) {
		TPD_INFO("Failed to set fp error report\n");
		return;
	}

	retval = fts_read_reg(FTS_REG_SET_FP_ERROR_REPORT, &regval);
	if(retval < 0) {
		TPD_INFO("Failed to get fp error report\n");
		return;
	}
	TPD_INFO("%s: now reg_val=0x%x", __func__, regval);
}


/*
 * return success: 0; fail : negative
 */
static int fts_hw_reset(struct chip_data_ft3683g *ts_data, u32 delayms)
{
	TPD_INFO("%s.\n", __func__);
	fts_write_reg(0xB6, 0x01);

	msleep(20);
	fts_rstgpio_set(ts_data->hw_res, false); /* reset gpio*/
	msleep(5);
	fts_rstgpio_set(ts_data->hw_res, true); /* reset gpio*/

	if (delayms) {
		msleep(delayms);
	}
	ts_data->is_ic_sleep = false;
	if (ts_data->fingerprint_error_report_support && tp_debug > LEVEL_BASIC) {
		fts_set_fp_error_report(ts_data, true);
	}
	return 0;
}
static int fts_power_control(void *chip_data, bool enable)
{
	int ret = 0;

	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;

	if (true == enable) {
		fts_rstgpio_set(ts_data->hw_res, false);
		msleep(1);
		ret = tp_powercontrol_avdd(ts_data->hw_res, true);

		if (ret) {
			return -1;
		}
		ret = tp_powercontrol_vddi(ts_data->hw_res, true);

		if (ret) {
			return -1;
		}
		msleep(POWEWRUP_TO_RESET_TIME);
		fts_rstgpio_set(ts_data->hw_res, true);
		msleep(RESET_TO_NORMAL_TIME);
		ts_data->is_ic_sleep = false;
	} else {
		ret = fts_write_reg(FTS_REG_PULSE_CONTROL, 0x01);
		msleep(2);
		fts_rstgpio_set(ts_data->hw_res, false);
		msleep(1);
		ret = tp_powercontrol_avdd(ts_data->hw_res, false);

		if (ret) {
			return -1;
		}
		ret = tp_powercontrol_vddi(ts_data->hw_res, false);

		if (ret) {
			return -1;
		}
	}

	return ret;
}

static int focal_dump_reg_state(void *chip_data, char *buf)
{
	int count = 0;
	u8 regvalue = 0;

	/*power mode 0:active 1:monitor 3:sleep*/
	fts_read_reg(FTS_REG_POWER_MODE, &regvalue);
	count += sprintf(buf + count, "Power Mode:0x%02x\n", regvalue);

	/*FW version*/
	fts_read_reg(FTS_REG_FW_VER, &regvalue);
	count += sprintf(buf + count, "FW Ver:0x%02x\n", regvalue);

	/*Vendor ID*/
	fts_read_reg(FTS_REG_VENDOR_ID, &regvalue);
	count += sprintf(buf + count, "Vendor ID:0x%02x\n", regvalue);

	/* 1 Gesture mode,0 Normal mode*/
	fts_read_reg(FTS_REG_GESTURE_EN, &regvalue);
	count += sprintf(buf + count, "Gesture Mode:0x%02x\n", regvalue);

	/* 3 charge in*/
	fts_read_reg(FTS_REG_CTRL, &regvalue);
	count += sprintf(buf + count, "Control stat:0x%02x\n", regvalue);

	/*Interrupt counter*/
	fts_read_reg(FTS_REG_INT_CNT, &regvalue);
	count += sprintf(buf + count, "INT count:0x%02x\n", regvalue);

	/*Flow work counter*/
	fts_read_reg(FTS_REG_FLOW_WORK_CNT, &regvalue);
	count += sprintf(buf + count, "ESD count:0x%02x\n", regvalue);

	return count;
}

static int focal_get_fw_version(void *chip_data)
{
	u8 fw_ver = 0;

	fts_read_reg(FTS_REG_FW_VER, &fw_ver);
	return (int)fw_ver;
}

static void focal_get_differ_version(void *chip_data)
{
	u8 differ_version = 0;
	int tx_num = 0;
	int rx_num = 0;
	u16 buf_len = 0;
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;

	tx_num = ts_data->hw_res->tx_num;
	rx_num = ts_data->hw_res->rx_num;
	if (tx_num > FTS_MAX_TX_NUM)
		tx_num = FTS_MAX_TX_NUM;
	if (rx_num > FTS_MAX_RX_NUM)
		rx_num = FTS_MAX_RX_NUM;

	buf_len = 154 + (tx_num * rx_num) * 2 + (tx_num + rx_num) * 4 + 8 * rx_num * 2;

	fts_read_reg(FTS_REG_DIFFER_VERSION, &differ_version);
	if (differ_version) {
		ts_data->buffer_len = buf_len;
		ts_data->tp_differ_version = FTS_DIFFER_VERSION_V2;
		TPD_INFO("ts tp differ version v2.\n");
	} else {
		ts_data->buffer_len = FTS_MAX_POINTS_SNR_LENGTH;
		ts_data->tp_differ_version = FTS_DIFFER_VERSION_V1;
		TPD_INFO("ts tp differ version v1.\n");
	}
}

static void focal_esd_check_enable(void *chip_data, bool enable)
{
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	ts_data->esd_check_enabled = enable;
}

static bool focal_get_esd_check_flag(void *chip_data)
{
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	return ts_data->esd_check_need_stop;
}

static int fts_esd_handle(void *chip_data)
{
	int ret = -1;
	int i = 0;
	static int flow_work_cnt_last = 0;
	static int err_cnt = 0;
	static int i2c_err = 0;
	u8 val = 0xFF;
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;

	if (!ts_data->esd_check_enabled) {
		goto NORMAL_END;
	}

	ret = fts_read_reg(0x00, &val);

	if ((ret & 0x70) == 0x40) { /*work in factory mode*/
		goto NORMAL_END;
	}

	for (i = 0; i < 3; i++) {
		ret = fts_read_reg(FTS_REG_CHIP_ID, &val);

		if (val != FTS_VAL_CHIP_ID) {
			TPD_INFO("%s: read chip_id failed!(ret:%x)\n", __func__, ret);
			msleep(10);
			i2c_err++;

		} else {
			i2c_err = 0;
			break;
		}
	}

	ret = fts_read_reg(FTS_REG_FLOW_WORK_CNT, &val);

	if (ret < 0) {
		TPD_INFO("%s: read FTS_REG_FLOW_WORK_CNT failed!\n", __func__);
		i2c_err++;
	}

	if (flow_work_cnt_last == val) {
		err_cnt++;

	} else {
		err_cnt = 0;
	}

	flow_work_cnt_last = ret;

	if ((err_cnt >= 5) || (i2c_err >= 3)) {
		TPD_INFO("esd check failed, start reset!\n");
		disable_irq_nosync(ts_data->ts->irq);
		tp_touch_btnkey_release(ts_data->tp_index);
		fts_reset(ts_data);
		enable_irq(ts_data->ts->irq);
		flow_work_cnt_last = 0;
		err_cnt = 0;
		i2c_err = 0;
	}

NORMAL_END:
	return 0;
}


static void fts_release_all_finger(struct touchpanel_data *ts)
{
#ifdef TYPE_B_PROTOCOL
	int i = 0;

	if (!ts->touch_count || !ts->irq_slot)
		return;

	mutex_lock(&ts->report_mutex);
	for (i = 0; i < ts->max_num; i++) {
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
	input_sync(ts->input_dev);
	mutex_unlock(&ts->report_mutex);
	TPD_INFO("fts_release_all_finger");
	ts->view_area_touched = 0; /*realse all touch point,must clear this flag*/
	ts->touch_count = 0;
	ts->irq_slot = 0;
#endif
}

static void fts_prc_func(struct work_struct *work)
{
	struct chip_data_ft3683g *ts_data = container_of(work,
	                                   struct chip_data_ft3683g, prc_work.work);
	unsigned long cur_jiffies = jiffies;
	unsigned long intr_timeout = msecs_to_jiffies(PRC_INTR_INTERVALS);

	if (ts_data->prc_support && !ts_data->ts->is_suspended) {
		intr_timeout += ts_data->intr_jiffies;
		if (time_after(cur_jiffies, intr_timeout)) {
			if (ts_data->ts->touch_count && ts_data->ts->irq_slot) {
				fts_release_all_finger(ts_data->ts);
				TPD_INFO("prc trigger interval:%dms",
				         jiffies_to_msecs(cur_jiffies - ts_data->intr_jiffies));
			}
			ts_data->prc_mode = 0;
		} else {
			queue_delayed_work(ts_data->ts_workqueue, &ts_data->prc_work,
			                   msecs_to_jiffies(POINT_REPORT_CHECK_WAIT_TIME));
			ts_data->prc_mode = 1;
		}
	} else {
		ts_data->prc_mode = 0;
	}
}

static void fts_prc_queue_work(struct chip_data_ft3683g *ts_data)
{
	ts_data->intr_jiffies = jiffies;
	if (ts_data->prc_support && !ts_data->prc_mode && !ts_data->ts->is_suspended) {
		queue_delayed_work(ts_data->ts_workqueue, &ts_data->prc_work,
		                   msecs_to_jiffies(POINT_REPORT_CHECK_WAIT_TIME));
		ts_data->prc_mode = 1;
	}
}



static int fts_point_report_check_init(struct chip_data_ft3683g *ts_data)
{
	TPD_INFO("point check init");

	if (ts_data->ts_workqueue) {
		INIT_DELAYED_WORK(&ts_data->prc_work, fts_prc_func);
	} else {
		TPD_INFO("fts workqueue is NULL, can't run point report check function");
		return -EINVAL;
	}

	ts_data->prc_support = 1;
	return 0;
}

static int fts_point_report_check_exit(struct chip_data_ft3683g *ts_data)
{
	TPD_INFO("point check exit");
	cancel_delayed_work_sync(&ts_data->prc_work);
	return 0;
}


static bool fts_fwupg_check_flash_status(struct chip_data_ft3683g *ts_data,
        u16 flash_status, int retries, int retries_delay)
{
	int ret = 0;
	int i = 0;
	u8 cmd = 0;
	u8 val[2] = { 0 };
	u16 read_status = 0;

	for (i = 0; i < retries; i++) {
		cmd = FTS_CMD_FLASH_STATUS;
		ret = fts_read(&cmd, 1, val, 2);
		read_status = (((u16)val[0]) << 8) + val[1];

		if (flash_status == read_status) {
			return true;
		}

		TPD_DEBUG("flash status fail,ok:%04x read:%04x, retries:%d", flash_status,
		          read_status, i);
		msleep(retries_delay);
	}

	TPD_INFO("flash status fail,ok:%04x read:%04x, retries:%d", flash_status,
	         read_status, i);
	return false;
}


static int fts_fwupg_enter_into_boot(struct chip_data_ft3683g *ts_data)
{
	int ret = 0;
	int i = 0;
	u8 cmd = 0;
	u8 id[2] = { 0 };

	do {
		/*reset to boot*/
		ret = fts_write_reg(FTS_REG_UPGRADE, FTS_UPGRADE_AA);

		if (ret < 0) {
			TPD_INFO("write FC=0xAA fail");
			return ret;
		}

		msleep(FTS_DELAY_UPGRADE_AA);

		ret = fts_write_reg(FTS_REG_UPGRADE, FTS_UPGRADE_55);

		if (ret < 0) {
			TPD_INFO("write FC=0x55 fail");
			return ret;
		}

		msleep(FTS_DELAY_UPGRADE_RESET);

		/*read boot id*/
		cmd = FTS_CMD_START;
		ret = fts_write(&cmd, 1);

		if (ret < 0) {
			TPD_INFO("write 0x55 fail");
			return ret;
		}

		cmd = FTS_CMD_READ_ID;
		ret = fts_read(&cmd, 1, id, 2);

		if (ret < 0) {
			TPD_INFO("read boot id fail");
			return ret;
		}

		TPD_INFO("read boot id:0x%02x%02x", id[0], id[1]);

		if ((id[0] == FTS_VAL_BT_ID) && (id[1] == FTS_VAL_BT_ID2)) {
			break;
		}
	} while (i++ < FTS_UPGRADE_LOOP);

	return 0;
}

static int fts_fwupg_erase(struct chip_data_ft3683g *ts_data, u32 delay)
{
	int ret = 0;
	u8 cmd = 0;
	bool flag = false;

	TPD_INFO("**********erase now**********");

	/*send to erase flash*/
	cmd = FTS_CMD_ERASE_APP;
	ret = fts_write(&cmd, 1);

	if (ret < 0) {
		TPD_INFO("send erase cmd fail");
		return ret;
	}

	msleep(delay);

	/* read status 0xF0AA: success */
	flag = fts_fwupg_check_flash_status(ts_data, FTS_CMD_FLASH_STATUS_ERASE_OK,
	                                    FTS_RETRIES_REASE, FTS_RETRIES_DELAY_REASE);

	if (!flag) {
		TPD_INFO("check ecc flash status fail");
		return -EIO;
	}

	return 0;
}

static int fts_flash_write_buf(struct chip_data_ft3683g *ts_data, u32 saddr,
                               u8 *buf, u32 len, u32 delay)
{
	int ret = 0;
	u32 i = 0;
	u32 j = 0;
	u32 packet_number = 0;
	u32 packet_len = 0;
	u32 addr = 0;
	u32 offset = 0;
	u32 remainder = 0;
	u32 cmdlen = 0;
	u8 packet_buf[BYTES_PER_TIME + 6] = { 0 };
	u8 cmd = 0;
	u8 val[2] = { 0 };
	u16 read_status = 0;
	u16 wr_ok = 0;

	TPD_INFO("**********write data to flash**********");
	TPD_INFO("data buf start addr=0x%x, len=0x%x", saddr, len);
	packet_number = len / BYTES_PER_TIME;
	remainder = len % BYTES_PER_TIME;

	if (remainder > 0) {
		packet_number++;
	}

	packet_len = BYTES_PER_TIME;
	TPD_INFO("write data, num:%d remainder:%d", packet_number, remainder);

	for (i = 0; i < packet_number; i++) {
		offset = i * BYTES_PER_TIME;
		addr = saddr + offset;

		/* last packet */
		if ((i == (packet_number - 1)) && remainder) {
			packet_len = remainder;
		}

		packet_buf[0] = FTS_CMD_SET_WFLASH_ADDR;
		packet_buf[1] = (addr >> 16) & 0xFF;
		packet_buf[2] = (addr >> 8) & 0xFF;
		packet_buf[3] = (addr) & 0xFF;
		ret = fts_write(packet_buf, 4);
		if (ret < 0) {
			TPD_INFO("set flash address fail");
			return ret;
		}

		packet_buf[0] = FTS_CMD_WRITE;
		cmdlen = 1;

		memcpy(&packet_buf[cmdlen], &buf[offset], packet_len);
		ret = fts_write(&packet_buf[0], packet_len + cmdlen);

		if (ret < 0) {
			TPD_INFO("app write fail");
			return ret;
		}

		mdelay(delay);

		/* read status */
		wr_ok = FTS_CMD_FLASH_STATUS_WRITE_OK + addr / packet_len;

		for (j = 0; j < FTS_RETRIES_WRITE; j++) {
			cmd = FTS_CMD_FLASH_STATUS;
			ret = fts_read(&cmd, 1, val, 2);
			read_status = (((u16)val[0]) << 8) + val[1];

			/* TPD_DEBUG("%x %x", wr_ok, read_status); */
			if (wr_ok == read_status) {
				break;
			}

			mdelay(FTS_RETRIES_DELAY_WRITE);
		}
	}

	return 0;
}

static int fts_fwupg_ecc_cal_host(u8 *buf, u32 len)
{
	u16 ecc = 0;
	u32 i = 0;
	u32 j = 0;

	for (i = 0; i < len; i += 2) {
		ecc ^= ((buf[i] << 8) | (buf[i + 1]));
		for (j = 0; j < 16; j ++) {
			if (ecc & 0x01)
				ecc = (u16)((ecc >> 1) ^ AL2_FCS_COEF);
			else
				ecc >>= 1;
		}
	}

	return (int)ecc;
}

static int fts_fwupg_ecc_cal_tp(struct chip_data_ft3683g *ts_data, u32 saddr, u32 len)
{
	int ret = 0;
	u8 wbuf[7] = { 0 };
	u8 val[2] = { 0 };
	int ecc = 0;
	bool bflag = false;

	TPD_INFO("**********read out checksum**********");
	/* check sum init */
	wbuf[0] = FTS_CMD_ECC_INIT;
	ret = fts_write(&wbuf[0], 1);

	if (ret < 0) {
		TPD_INFO("ecc init cmd write fail");
		return ret;
	}

	/* send commond to start checksum */
	wbuf[0] = FTS_CMD_ECC_CAL;
	wbuf[1] = (saddr >> 16) & 0xFF;
	wbuf[2] = (saddr >> 8) & 0xFF;
	wbuf[3] = (saddr);
	wbuf[4] = (len >> 16) & 0xFF;
	wbuf[5] = (len >> 8) & 0xFF;
	wbuf[6] = (len);
	TPD_INFO("ecc calc startaddr:0x%04x, len:%d", saddr, len);
	ret = fts_write(&wbuf[0], 7);

	if (ret < 0) {
		TPD_INFO("ecc calc cmd write fail");
		return ret;
	}

	msleep(len / 256);

	/* read status if check sum is finished */
	bflag = fts_fwupg_check_flash_status(ts_data, FTS_CMD_FLASH_STATUS_ECC_OK,
	                                     FTS_RETRIES_ECC_CAL,
	                                     FTS_RETRIES_DELAY_ECC_CAL);

	if (!bflag) {
		TPD_INFO("ecc flash status read fail");
		return -EIO;
	}

	/* read out check sum */
	wbuf[0] = FTS_CMD_ECC_READ;
	ret = fts_read(&wbuf[0], 1, val, 2);

	if (ret < 0) {
		TPD_INFO("ecc read cmd write fail");
		return ret;
	}

	ecc = (int)((u16)(val[0] << 8) + val[1]);

	return ecc;
}

static int fts_upgrade(struct chip_data_ft3683g *ts_data, u8 *buf, u32 len)
{
	int ret = 0;
	u32 start_addr = 0;
	u8 cmd[4] = { 0 };
	int ecc_in_host = 0;
	int ecc_in_tp = 0;
	struct monitor_data *monitor_data = ts_data->monitor_data;

	if (!buf) {
		TPD_INFO("fw_buf is invalid");
		return -EINVAL;
	}

	/* enter into upgrade environment */
	ret = fts_fwupg_enter_into_boot(ts_data);

	if (ret < 0 || (monitor_data && monitor_data->health_simulate_trigger)) {
		tp_healthinfo_report(monitor_data, HEALTH_FW_UPDATE, "Enter pramboot/bootloader failed");
		TPD_INFO("enter into pramboot/bootloader fail,ret=%d", ret);
		if (!monitor_data || !monitor_data->health_simulate_trigger) {
			goto fw_reset;
		}
	}

	cmd[0] = FTS_CMD_DATA_LEN;
	cmd[1] = (len >> 16) & 0xFF;
	cmd[2] = (len >> 8) & 0xFF;
	cmd[3] = (len) & 0xFF;
	ret = fts_write(&cmd[0], 4);

	if (ret < 0 || (monitor_data && monitor_data->health_simulate_trigger)) {
		tp_healthinfo_report(monitor_data, HEALTH_FW_UPDATE, "FTS_CMD_DATA_LEN failed");
		TPD_INFO("data len cmd write fail");
		if (!monitor_data || !monitor_data->health_simulate_trigger) {
			goto fw_reset;
		}
	}

	/*erase*/
	ret = fts_fwupg_erase(ts_data, FTS_REASE_APP_DELAY);

	if (ret < 0 || (monitor_data && monitor_data->health_simulate_trigger)) {
		tp_healthinfo_report(monitor_data, HEALTH_FW_UPDATE, "FTS_REASE_APP_DELAY failed");
		TPD_INFO("erase cmd write fail");
		if (!monitor_data || !monitor_data->health_simulate_trigger) {
			goto fw_reset;
		}
	}

	/* write app */
	start_addr = 0;
	ret = fts_flash_write_buf(ts_data, start_addr, buf, len, 1);

	if (ret < 0 || (monitor_data && monitor_data->health_simulate_trigger)) {
		tp_healthinfo_report(monitor_data, HEALTH_FW_UPDATE, "Flash Write failed");
		TPD_INFO("flash write fail");
		if (!monitor_data || !monitor_data->health_simulate_trigger) {
			goto fw_reset;
		}
	}

	ecc_in_host = fts_fwupg_ecc_cal_host(buf, len);
	ecc_in_tp = fts_fwupg_ecc_cal_tp(ts_data, start_addr, len);

	if (ecc_in_tp < 0 || (monitor_data && monitor_data->health_simulate_trigger)) {
		tp_healthinfo_report(monitor_data, HEALTH_FW_UPDATE, "ECC Read failed");
		TPD_INFO("ecc read fail");
		if (!monitor_data || !monitor_data->health_simulate_trigger) {
			goto fw_reset;
		}
	}

	TPD_INFO("ecc in tp:%x, host:%x", ecc_in_tp, ecc_in_host);

	if (ecc_in_tp != ecc_in_host || (monitor_data && monitor_data->health_simulate_trigger)) {
		tp_healthinfo_report(monitor_data, HEALTH_FW_UPDATE, "ECC Check failed");
		TPD_INFO("ecc check fail");
		if (!monitor_data || !monitor_data->health_simulate_trigger) {
			goto fw_reset;
		}
	}

	TPD_INFO("upgrade success, reset to normal boot");
	cmd[0] = FTS_CMD_RESET;
	ret = fts_write(&cmd[0], 1);

	if (ret < 0 || (monitor_data && monitor_data->health_simulate_trigger)) {
		tp_healthinfo_report(monitor_data, HEALTH_FW_UPDATE, "FTS_CMD_RESET failed");
		TPD_INFO("reset to normal boot fail");
	}

	msleep(200);
	return 0;

fw_reset:
	TPD_INFO("upgrade fail, reset to normal boot");
	cmd[0] = FTS_CMD_RESET;
	ret = fts_write(&cmd[0], 1);

	if (ret < 0 || (monitor_data && monitor_data->health_simulate_trigger)) {
		tp_healthinfo_report(monitor_data, HEALTH_FW_UPDATE, "FTS_CMD_RESET failed");
		TPD_INFO("reset to normal boot fail");
	}

	return -EIO;
}


static fw_check_state fts_fw_check(void *chip_data,
                                   struct resolution_info *resolution_info, struct panel_info *panel_data)
{
	u8 cmd = 0;
	u8 id[2] = { 0 };
	char dev_version[MAX_DEVICE_VERSION_LENGTH] = {0};
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;

	fts_read_reg(FTS_REG_CHIP_ID, &id[0]);
	fts_read_reg(FTS_REG_CHIP_ID2, &id[1]);

	if ((id[0] != FTS_VAL_CHIP_ID) || (id[1] != FTS_VAL_CHIP_ID2)) {
		cmd = 0x55;
		fts_write(&cmd, 1);
		msleep(12);
		cmd = 0x90;
		fts_read(&cmd, 1, id, 2);
		TPD_INFO("boot id:0x%02x%02x, fw abnormal", id[0], id[1]);
		return FW_ABNORMAL;
	}

	/*fw check normal need update tp_fw  && device info*/
	fts_read_reg(FTS_REG_FW_VER, &ts_data->fwver);
	panel_data->tp_fw = ts_data->fwver;
	TPD_INFO("FW VER:%d", panel_data->tp_fw);

	if (panel_data->manufacture_info.version) {
		sprintf(dev_version, "%04x", panel_data->tp_fw);
		strncpy(&(panel_data->manufacture_info.version[7]), dev_version, 5);
	}

	return FW_NORMAL;
}

int fts_reset_proc(int hdelayms)
{
	TPD_INFO("tp reset");
	fts_rstgpio_set(g_fts_data->hw_res, false); /* reset gpio*/
	msleep(5);
	fts_rstgpio_set(g_fts_data->hw_res, true); /* reset gpio*/

	if (hdelayms) {
		msleep(hdelayms);
	}

	return 0;
}

#define OFFSET_FW_DATA_FW_VER 0x010E
static fw_update_state fts_fw_update(void *chip_data, const struct firmware *fw,
                                     bool force)
{
	int ret = 0;
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	u8 *buf;
	u32 len = 0;

	if (!fw) {
		TPD_INFO("fw is null");
		return FW_UPDATE_ERROR;
	}

	buf = (u8 *)fw->data;
	len = (int)fw->size;

	if ((len < 0x120)) {
		TPD_INFO("fw_len(%d) is invalid", len);
		return FW_UPDATE_ERROR;
	}

	if (force || (buf[OFFSET_FW_DATA_FW_VER] != ts_data->fwver)) {
		TPD_INFO("Need update, force(%d)/fwver:Host(0x%02x),TP(0x%02x)", force,
		         buf[OFFSET_FW_DATA_FW_VER], ts_data->fwver);
		focal_esd_check_enable(ts_data, false);
		ret = fts_upgrade(ts_data, buf, len);
		focal_esd_check_enable(ts_data, true);

		if (ret < 0) {
			TPD_INFO("fw update fail");
			return FW_UPDATE_ERROR;
		}

		focal_get_differ_version(ts_data);
		return FW_UPDATE_SUCCESS;
	}

	return FW_NO_NEED_UPDATE;
}


static int fts_enter_factory_work_mode(struct chip_data_ft3683g *ts_data,
                                       u8 mode_val)
{
	int ret = 0;
	int retry = 20;
	u8 regval = 0;

	TPD_INFO("%s:enter %s mode", __func__, (mode_val == 0x40) ? "factory" : "work");
	ret = fts_write_reg(DEVIDE_MODE_ADDR, mode_val);

	if (ret < 0) {
		TPD_INFO("%s:write mode(val:0x%x) fail", __func__, mode_val);
		return ret;
	}

	while (--retry) {
		fts_read_reg(DEVIDE_MODE_ADDR, &regval);

		if (regval == mode_val) {
			break;
		}

		msleep(20);
	}

	if (!retry) {
		TPD_INFO("%s:enter mode(val:0x%x) timeout", __func__, mode_val);
		return -EIO;
	}

	msleep(FACTORY_TEST_DELAY);
	return 0;
}

static int fts_start_scan(struct chip_data_ft3683g *ts_data)
{
	int ret = 0;
	int retry = 50;
	u8 regval = 0;
	u8 scanval = FTS_FACTORY_MODE_VALUE | (1 << 7);

	TPD_INFO("%s: start to scan a frame", __func__);
	ret = fts_write_reg(DEVIDE_MODE_ADDR, scanval);

	if (ret < 0) {
		TPD_INFO("%s:start to scan a frame fail", __func__);
		return ret;
	}

	while (--retry) {
		fts_read_reg(DEVIDE_MODE_ADDR, &regval);

		if (regval == FTS_FACTORY_MODE_VALUE) {
			break;
		}

		msleep(20);
	}

	if (!retry) {
		TPD_INFO("%s:scan a frame timeout", __func__);
		return -EIO;
	}

	return 0;
}

static int fts_get_rawdata(struct chip_data_ft3683g *ts_data, int *raw,
                           bool is_diff)
{
	int ret = 0;
	int i = 0;
	int byte_num = ts_data->hw_res->tx_num * ts_data->hw_res->rx_num * 2;
	int size = 0;
	int packet_len = 0;
	int offset = 0;
	u8 raw_addr = 0;
	u8 regval = 0;
	u8 *buf = NULL;

	TPD_INFO("%s:call", __func__);
	/*kzalloc buffer*/
	buf = kzalloc(byte_num, GFP_KERNEL);

	if (!buf) {
		TPD_INFO("%s:kzalloc for raw byte buf fail", __func__);
		return -ENOMEM;
	}

	ret = fts_enter_factory_work_mode(ts_data, FTS_FACTORY_MODE_VALUE);

	if (ret < 0) {
		TPD_INFO("%s:enter factory mode fail", __func__);
		goto raw_err;
	}

	if (is_diff) {
		fts_read_reg(FACTORY_REG_DATA_SELECT, &regval);
		ret = fts_write_reg(FACTORY_REG_DATA_SELECT, 0x01);

		if (ret < 0) {
			TPD_INFO("%s:write 0x01 to reg0x06 fail", __func__);
			goto reg_restore;
		}
	}

	ret = fts_start_scan(ts_data);

	if (ret < 0) {
		TPD_INFO("%s:scan a frame fail", __func__);
		goto reg_restore;
	}

	ret = fts_write_reg(FACTORY_REG_LINE_ADDR, 0xAA);

	if (ret < 0) {
		TPD_INFO("%s:write 0xAA to reg0x01 fail", __func__);
		goto reg_restore;
	}

	raw_addr = FACTORY_REG_RAWDATA_ADDR_MC_SC;
	ret = fts_read(&raw_addr, 1, buf, MAX_PACKET_SIZE);
	size = byte_num - MAX_PACKET_SIZE;
	offset = MAX_PACKET_SIZE;

	while (size > 0) {
		if (size >= MAX_PACKET_SIZE) {
			packet_len = MAX_PACKET_SIZE;

		} else {
			packet_len = size;
		}

		ret = fts_read(&raw_addr, 1, buf + offset, packet_len);

		if (ret < 0) {
			TPD_INFO("%s:read raw data(packet:%d) fail", __func__,
			         offset / MAX_PACKET_SIZE);
			goto reg_restore;
		}

		size -= packet_len;
		offset += packet_len;
	}

	for (i = 0; i < byte_num; i = i + 2) {
		raw[i >> 1] = (int)(short)((buf[i] << 8) + buf[i + 1]);
	}

reg_restore:

	if (is_diff) {
		ret = fts_write_reg(FACTORY_REG_DATA_SELECT, regval);

		if (ret < 0) {
			TPD_INFO("%s:restore reg0x06 fail", __func__);
		}
	}

raw_err:
	kfree(buf);
	ret = fts_enter_factory_work_mode(ts_data, FTS_WORK_MODE_VALUE);

	if (ret < 0) {
		TPD_INFO("%s:enter work mode fail", __func__);
	}

	return ret;
}

static void fts_fa_fb_read(struct seq_file *s, void *chip_data, u8 base_cmd) {
	char *data_a = NULL;
	u8 base_a = base_cmd;
	u8 *buff;
	int ret = 0;
	int i = 0;
	int j = 0;

	data_a = kzalloc(5120, GFP_KERNEL);
	if (!data_a) {
		TPD_INFO("focal:%s:kmalloc error.", __func__);
		return;
	}

	buff = kzalloc(300, GFP_KERNEL);
	if (buff == NULL) {
		TPD_INFO("focal:%s:kmalloc error\n", __func__);
		goto end1;
	}

	fts_read(&base_a, 1, data_a, 5120);
	seq_printf(s, "%x 5k read start\n", base_a);
	for (i = 0; i < 64; i++) {
		ret = 0;
		for (j = 0; j < 80; j++) {
			ret += snprintf(buff + ret, 300 - ret, "%2x ", data_a[i * 80 + j]);
		}
		TPD_INFO("[%02d]%s\n", i, buff);
		seq_printf(s, "[%02d]%s\n", i, buff);
	}

	kfree(buff);
end1:
	kfree(data_a);
}

static void fts_delta_read(struct seq_file *s, void *chip_data)
{
	int ret = 0;
	int i = 0;
	int j = 0;
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	u8 *touch_buf = ts_data->snr_buf;
	u8 cmd = FTS_REG_POINTS;
	u8 last_differ_mode = 0;
	int buffer_len = 0;

	TPD_INFO("%s:start to read diff data", __func__);
	focal_esd_check_enable(ts_data, false);	 /*no allowed esd check*/

	ret = fts_write_reg(FTS_REG_POWER_MODE, 0x00);
	if (ret < 0) {
	   TPD_INFO("%s:set tp power_mode fail", __func__);
	}
	TPD_INFO("%s:set tp power_mode success", __func__);

	focal_get_differ_version(ts_data);
	if (ts_data->tp_differ_version == FTS_DIFFER_VERSION_V2) {
		fts_fa_fb_read(s, ts_data, 0xFB);
	}

	last_differ_mode = ts_data->differ_mode;
	ret = fts_write_reg(FTS_REG_WORK_MODE, FTS_REG_WORK_MODE_FINAL_DIFF_MODE);
	if (ret < 0) {
	   TPD_INFO("%s:open fastdiff fail", __func__);
	   goto raw_fail;
	}
	ts_data->differ_mode = FTS_REG_WORK_MODE_FINAL_DIFF_MODE;
	TPD_INFO("%s:open fastdiff test success", __func__);
	msleep(50);
	buffer_len = ts_data->buffer_len;

	for (j = 0; j < 10; j++) {
	   memset(touch_buf, 0xFF, buffer_len);
	   ret = fts_read(&cmd, 1, &touch_buf[0], buffer_len);
	   ts_data->snr_data_is_ready = 1;
	   fts_get_rawdata_snr(ts_data);
	   if (ts_data->snr_count != 255) {
		   TPD_INFO("%s:get rawdata suc,count:%u.", __func__, ts_data->snr_count);
		   break;
	   } else {
		   TPD_INFO("%s:get rawdata fail,count:%u,time:%d.", __func__, ts_data->snr_count, j);
	   }
	   msleep(10);
	}

	seq_printf(s, "mutual diff data:");
	for (i = 0; i < tx_num; i++) {
	   seq_printf(s, "\n[%2d]", i + 1);

	   for (j = 0; j < rx_num; j++) {
		   seq_printf(s, " %6d,", ts_data->diff_buf[i * rx_num + j]);
	   }
	}
	seq_printf(s, "\n");

	seq_printf(s, "sc_water diff data:\n");
	seq_printf(s, "[rx]");
	for (i = 0; i < rx_num; i++) {
	   seq_printf(s, " %6d,", ts_data->sc_water[i]);
	}
	seq_printf(s, "\n");

	seq_printf(s, "[tx]");
	for (i = 0; i < tx_num; i++) {
	   seq_printf(s, " %6d,", ts_data->sc_water[i + rx_num]);
	}
	seq_printf(s, "\n");

	seq_printf(s, "sc_nomal diff data:\n");
	seq_printf(s, "[rx]");
	for (i = 0; i < rx_num; i++) {
	   seq_printf(s, " %6d,", ts_data->sc_nomal[i]);
	}
	seq_printf(s, "\n");

	seq_printf(s, "[tx]");
	for (i = 0; i < tx_num; i++) {
	   seq_printf(s, " %6d,", ts_data->sc_nomal[i + rx_num]);
	}
	seq_printf(s, "\n");

raw_fail:
	fts_write_reg(FTS_REG_WORK_MODE, last_differ_mode);
	ts_data->differ_mode = last_differ_mode;
	msleep(30);
	focal_esd_check_enable(ts_data, true);
}

static void fts_self_delta_read(struct seq_file *s, void *chip_data)
{
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	int ret = 0;

	TPD_INFO("%s:start to read diff data", __func__);
	focal_esd_check_enable(ts_data, false);	 /*no allowed esd check*/

	ret = fts_write_reg(FTS_REG_POWER_MODE, 0x00);
	if (ret < 0) {
	   TPD_INFO("%s:set tp power_mode fail", __func__);
	}
	TPD_INFO("%s:set tp power_mode success", __func__);

	if (ts_data->tp_differ_version == FTS_DIFFER_VERSION_V2) {
		fts_fa_fb_read(s, ts_data, 0xFA);
	}
	msleep(30);
	focal_esd_check_enable(ts_data, true);
}

static void fts_baseline_read(struct seq_file *s, void *chip_data)
{
	int ret = 0;
	int i = 0;
	int j = 0;
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	int *raw = NULL;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;

	if (ts_data->tp_differ_version == FTS_DIFFER_VERSION_V2) {
		TPD_INFO("%s skip, no need.", __func__);
		return;
	}

	TPD_INFO("%s:start to read raw data", __func__);
	focal_esd_check_enable(ts_data, false);

	raw = kzalloc(tx_num * rx_num * sizeof(int), GFP_KERNEL);

	if (!raw) {
		seq_printf(s, "kzalloc for raw fail\n");
		goto raw_fail;
	}

	ret = fts_write_reg(FTS_REG_AUTOCLB_ADDR, 0x01);

	if (ret < 0) {
		TPD_INFO("%s, write 0x01 to reg 0xee failed \n", __func__);
	}

	ret = fts_get_rawdata(ts_data, raw, false);

	if (ret < 0) {
		seq_printf(s, "get raw data fail\n");
		goto raw_fail;
	}

	for (i = 0; i < tx_num; i++) {
		seq_printf(s, "\n[%2d]", i + 1);

		for (j = 0; j < rx_num; j++) {
			seq_printf(s, " %5d,", raw[i * rx_num + j]);
		}
	}

	seq_printf(s, "\n");

raw_fail:
	fts_write_reg(FTS_REG_AUTOCLB_ADDR, 0x00);
	focal_esd_check_enable(ts_data, true);
	kfree(raw);
}

static void fts_main_register_read(struct seq_file *s, void *chip_data)
{
	u8 regvalue = 0;
	u8 cmd = FTS_REG_FOD_INFO;
	u8 val[10] = { 0 };

	/*TP FW version*/
	fts_read_reg(FTS_REG_FW_VER, &regvalue);
	seq_printf(s, "TP FW Ver:0x%02x\n", regvalue);

	/*Vendor ID*/
	fts_read_reg(FTS_REG_VENDOR_ID, &regvalue);
	seq_printf(s, "Vendor ID:0x%02x\n", regvalue);

	/*Gesture enable*/
	fts_read_reg(FTS_REG_GESTURE_EN, &regvalue);
	seq_printf(s, "Gesture Mode:0x%02x\n", regvalue);

	/*Fod info*/
	memset(val, 0xFF, 10);
	fts_read(&cmd, 1, val, FTS_REG_FOD_INFO_LEN);
	seq_printf(s, "FOD_INFO:0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", val[0],
		  val[1], val[2], val[3], val[4], val[5], val[6], val[7], val[8]);

	/*charge in*/
	fts_read_reg(FTS_REG_CTRL, &regvalue);
	seq_printf(s, "Control state:0x%02x\n", regvalue);

	/*edge limit*/
	fts_read_reg(FTS_REG_EDGE_LIMIT, &regvalue);
	seq_printf(s, "edge Mode:0x%02x\n", regvalue);

	/*game mode*/
	fts_read_reg(FTS_REG_GAME_MODE_EN, &regvalue);
	seq_printf(s, "Game Mode:0x%02x\n", regvalue);

	/*FOD mode*/
	fts_read_reg(FTS_REG_FOD_EN, &regvalue);
	seq_printf(s, "FOD Mode:0x%02x\n", regvalue);

	/*Interrupt counter*/
	fts_read_reg(FTS_REG_INT_CNT, &regvalue);
	seq_printf(s, "INT count:0x%02x\n", regvalue);

	/*Flow work counter*/
	fts_read_reg(FTS_REG_FLOW_WORK_CNT, &regvalue);
	seq_printf(s, "ESD count:0x%02x\n", regvalue);

	/*Panel ID*/
	fts_read_reg(FTS_REG_MODULE_ID, &regvalue);
	seq_printf(s, "PANEL ID:0x%02x\n", regvalue);

	return;
}

static void fts_aiunit_game_info(void *chip_data)
{
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	u8 cmd[MAX_AIUNIT_SET_NUM * 10 + 1] = { 0 };
	int i = 0;
	u8 regvalue = 0;
	int ret = 0;

	if (ts_data == NULL) {
		return;
	}

	if (ts_data->ts->is_suspended) {
		return;
	}

	if (ts_data->ts->aiunit_game_enable) {
		ret = fts_write_reg(FTS_CMD_GAME_AIUINIT_EN, 1);
		if (ret < 0)
			TPD_INFO("%s, write 1 to reg 0xc9 failed \n", __func__);
		msleep(3);
		fts_read_reg(FTS_CMD_GAME_AIUINIT_EN, &regvalue);
		if (regvalue == 1) {
			TPD_INFO("%s: aiunit game info enter suc.\n", __func__);
		} else {
			TPD_INFO("%s: aiunit game info enter fail.\n", __func__);
		}
	} else {
		ret = fts_write_reg(FTS_CMD_GAME_AIUINIT_EN, 0);
		if (ret < 0)
			TPD_INFO("%s, write 0 to reg 0xc9 failed \n", __func__);
		msleep(3);
		fts_read_reg(FTS_CMD_GAME_AIUINIT_EN, &regvalue);
		if (regvalue == 0) {
			TPD_INFO("%s: aiunit game info exit suc.\n", __func__);
		} else {
			TPD_INFO("%s: aiunit game info exit fail.\n", __func__);
		}
	}

	cmd[0] = FTS_CMD_GAME_AIUINIT;
	for (i = 0; i < MAX_AIUNIT_SET_NUM; i++) {
		cmd[10 * i + 1] = ts_data->ts->tp_ic_aiunit_game_info[i].gametype;
		cmd[10 * i + 2] = ts_data->ts->tp_ic_aiunit_game_info[i].aiunit_game_type;
		cmd[10 * i + 3] = ts_data->ts->tp_ic_aiunit_game_info[i].left & 0xff;
		cmd[10 * i + 4] = (ts_data->ts->tp_ic_aiunit_game_info[i].left >> 8) & 0xff;
		cmd[10 * i + 5] = ts_data->ts->tp_ic_aiunit_game_info[i].top & 0xff;
		cmd[10 * i + 6] = (ts_data->ts->tp_ic_aiunit_game_info[i].top >> 8) & 0xff;
		cmd[10 * i + 7] = ts_data->ts->tp_ic_aiunit_game_info[i].right & 0xff;
		cmd[10 * i + 8] = (ts_data->ts->tp_ic_aiunit_game_info[i].right >> 8) & 0xff;
		cmd[10 * i + 9] = ts_data->ts->tp_ic_aiunit_game_info[i].bottom & 0xff;
		cmd[10 * i + 10] = (ts_data->ts->tp_ic_aiunit_game_info[i].bottom >> 8) & 0xff;
		TPD_INFO("type:%x,%x left:%x,%x top:%x,%x right:%x,%x bottom:%x,%x.", \
				cmd[10 * i + 1], cmd[10 * i + 2], \
				cmd[10 * i + 3], cmd[10 * i + 4], \
				cmd[10 * i + 5], cmd[10 * i + 6], \
				cmd[10 * i + 7], cmd[10 * i + 8], \
				cmd[10 * i + 9], cmd[10 * i + 10]);
	}

	ret = fts_write(&cmd[0], 10 * MAX_AIUNIT_SET_NUM + 1);
	if (ret < 0) {
		TPD_INFO("fts tp aiunit game write fail");
	}
}

static void fts_enable_gesture_mask(void *chip_data, uint32_t enable)
{
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	u8 gesture_config_D1 = 0xBF;
	u8 gesture_config_D2 = 0x07;
	u8 gesture_config_D6 = 0x3E;
	int state = ts_data->gesture_state;

	TPD_INFO("%s: enable gesture:%u.\n", __func__, enable);
	if (enable) {
			SET_GESTURE_BIT(state, DOU_TAP, gesture_config_D1, 4)
			SET_GESTURE_BIT(state, UP_VEE, gesture_config_D6, 4)
			SET_GESTURE_BIT(state, DOWN_VEE, gesture_config_D6, 3)
			SET_GESTURE_BIT(state, LEFT_VEE, gesture_config_D6, 2)
			SET_GESTURE_BIT(state, RIGHT_VEE, gesture_config_D6, 1)
			SET_GESTURE_BIT(state, CIRCLE_GESTURE, gesture_config_D2, 0)
			SET_GESTURE_BIT(state, DOU_SWIP, gesture_config_D1, 5)
			SET_GESTURE_BIT(state, LEFT2RIGHT_SWIP, gesture_config_D1, 1)
			SET_GESTURE_BIT(state, RIGHT2LEFT_SWIP, gesture_config_D1, 0)
			SET_GESTURE_BIT(state, UP2DOWN_SWIP, gesture_config_D1, 3)
			SET_GESTURE_BIT(state, DOWN2UP_SWIP, gesture_config_D1, 2)
			SET_GESTURE_BIT(state, M_GESTRUE, gesture_config_D2, 2)
			SET_GESTURE_BIT(state, W_GESTURE, gesture_config_D2, 1)
			SET_GESTURE_BIT(state, SINGLE_TAP, gesture_config_D1, 7)
			SET_GESTURE_BIT(state, HEART, gesture_config_D6, 5)
		} else {
			gesture_config_D1 = 0x0;
			gesture_config_D2 = 0x0;
			gesture_config_D6 = 0x0;
	}

	fts_write_reg(FTS_REG_GESTURE_CONFIG1, gesture_config_D1);
	fts_write_reg(FTS_REG_GESTURE_CONFIG2, gesture_config_D2);
	fts_write_reg(FTS_REG_GESTURE_CONFIG4, gesture_config_D6);
	msleep(1);
	TPD_INFO("%s: gesture config D1:%x, D2:%x, D6:%x\n", __func__, \
		gesture_config_D1, gesture_config_D2, gesture_config_D6);
}

static void fts_set_gesture_state(void *chip_data, int state)
{
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;

	TPD_INFO("%s:state:%d!\n", __func__, state);
	ts_data->gesture_state = state;
}


static int fts_enable_black_gesture(struct chip_data_ft3683g *ts_data,
                                    bool enable)
{
	int state = ts_data->gesture_state;
	u8 gesture_config_D1 = 0xBF;
	u8 gesture_config_D2 = 0x07;
	u8 gesture_config_D6 = 0x3E;
	u8 ucReadD0 = 0;
	int i = 0;

	TPD_INFO("MODE_GESTURE, write 0xD0=%d", enable);
	fts_write_reg(FTS_REG_GESTURE_EN, enable);
	for (i = 0; i < 3; i++) {
		fts_read_reg(FTS_REG_GESTURE_EN, &ucReadD0);
		if (ucReadD0 == enable) {
			TPD_INFO("MODE_GESTURE, Read 0xD0=%d (write success)", ucReadD0);
			break;
		} else {
		    fts_write_reg(FTS_REG_GESTURE_EN, enable);
		}
		msleep(10);
	}
	if (i >= 3) {
		TPD_INFO("MODE_GESTURE, Write 0xD0 failed after 3 retries!");
	}

	if (enable) {
			SET_GESTURE_BIT(state, DOU_TAP, gesture_config_D1, 4)
			SET_GESTURE_BIT(state, UP_VEE, gesture_config_D6, 4)
			SET_GESTURE_BIT(state, DOWN_VEE, gesture_config_D6, 3)
			SET_GESTURE_BIT(state, LEFT_VEE, gesture_config_D6, 2)
			SET_GESTURE_BIT(state, RIGHT_VEE, gesture_config_D6, 1)
			SET_GESTURE_BIT(state, CIRCLE_GESTURE, gesture_config_D2, 0)
			SET_GESTURE_BIT(state, DOU_SWIP, gesture_config_D1, 5)
			SET_GESTURE_BIT(state, LEFT2RIGHT_SWIP, gesture_config_D1, 1)
			SET_GESTURE_BIT(state, RIGHT2LEFT_SWIP, gesture_config_D1, 0)
			SET_GESTURE_BIT(state, UP2DOWN_SWIP, gesture_config_D1, 3)
			SET_GESTURE_BIT(state, DOWN2UP_SWIP, gesture_config_D1, 2)
			SET_GESTURE_BIT(state, M_GESTRUE, gesture_config_D2, 2)
			SET_GESTURE_BIT(state, W_GESTURE, gesture_config_D2, 1)
			SET_GESTURE_BIT(state, SINGLE_TAP, gesture_config_D1, 7)
			SET_GESTURE_BIT(state, HEART, gesture_config_D6, 5)
		} else {
			gesture_config_D1 = 0x0;
			gesture_config_D2 = 0x0;
			gesture_config_D6 = 0x0;
	}
	TPD_INFO("%s: gesture config D1:%x, D2:%x, D6:%x\n", __func__, \
		gesture_config_D1, gesture_config_D2, gesture_config_D6);

	fts_write_reg(FTS_REG_GESTURE_CONFIG1, gesture_config_D1);
	fts_write_reg(FTS_REG_GESTURE_CONFIG2, gesture_config_D2);
	fts_write_reg(FTS_REG_GESTURE_CONFIG4, gesture_config_D6);
	return 0;
}

static int fts_enable_edge_limit(struct chip_data_ft3683g *ts_data, int enable)
{
	u8 edge_mode = 0;

	/*0:Horizontal, 1:Vertical*/
	if (enable == VERTICAL_SCREEN) {
		edge_mode = 0;
		SET_REG(FTS_REG_EDGE_LIMIT_BIT, 0x00);

	} else if (enable == LANDSCAPE_SCREEN_90) {
		edge_mode = 1;
		SET_REG(FTS_REG_EDGE_LIMIT_BIT, 0x01);

	} else if (enable == LANDSCAPE_SCREEN_270) {
		edge_mode = 2;
		SET_REG(FTS_REG_EDGE_LIMIT_BIT, 0x02);
	}

	TPD_INFO("MODE_EDGE, write 0x8B|45=0x%x", ts_data->ctrl_reg_state);
	return fts_write_reg(FTS_REG_CTRL, ts_data->ctrl_reg_state);
}

static int fts_enable_charge_mode(struct chip_data_ft3683g *ts_data, bool enable)
{
	SET_REG(FTS_REG_CHARGER_MODE_EN_BIT, enable);
	TPD_INFO("MODE_CHARGE, write 0x8B|01=0x%x", ts_data->ctrl_reg_state);
	ts_data->charger_connected = enable;
	return fts_write_reg(FTS_REG_CTRL, ts_data->ctrl_reg_state);
}

static int fts_enable_game_mode(struct chip_data_ft3683g *ts_data, bool enable)
{
	struct chip_data_ft3683g *chip_data = (struct chip_data_ft3683g *)ts_data;
	struct touchpanel_data *ts = spi_get_drvdata(chip_data->ft_spi);
	int ret = 0;
	u8 regvalue = 0;
	int game_mode = FTS_NOT_GAME_MODE;
	int report_rate = FTS_120HZ_REPORT_RATE;
	TPD_INFO("MODE_GAME, write 0x8B%d", enable);

	if (ts->aiunit_game_info_support) {
		fts_write_reg(FTS_CMD_GAME_AIUINIT_EN, enable);
		TPD_INFO("%s: game aiuinit C9 set\n", __func__);
		msleep(1);
	}
	if (enable) {
		if (ts_data->extreme_game_report_rate) {
				TPD_INFO("%s:ts->noise_level:%d rate_ctrl_level:%d", __func__, ts->noise_level, ts->rate_ctrl_level);
				fts_read_reg(FTS_REG_CHARGER_MODE_EN, &regvalue);
				if(regvalue<= 1) {
				regvalue  = (regvalue) | (0x0C);
				ret = fts_write_reg(FTS_REG_CHARGER_MODE_EN, regvalue);
				}
			switch (ts->noise_level) {
			case INTELLIGENT_GAME_MODE:
				ts_data->extreme_game_flag = false;
				fts_rate_white_list_ctrl(ts_data, ts->rate_ctrl_level);
				break;

			case EXTREME_GAME_MODE:
				fts_rate_white_list_ctrl(ts_data, ts_data->extreme_game_report_rate);
				ts_data->extreme_game_flag = true;
				break;

			default:
				ts_data->extreme_game_flag = false;
				fts_rate_white_list_ctrl(ts_data, ts->rate_ctrl_level);
				break;
			}
			return ret;
		}
		if (ts_data->switch_game_rate_support) {/*ts_data->switch_game_rate_support*/
			switch (ts->noise_level) {
			case FTS_GET_RATE_120:
				game_mode = FTS_240HZ_GAME_MODE;
				report_rate = FTS_120HZ_REPORT_RATE;
				break;

			case FTS_GET_RATE_180:
				game_mode = FTS_240HZ_GAME_MODE;
				report_rate = FTS_180HZ_REPORT_RATE;
				break;

			case FTS_GET_RATE_240:
				game_mode = FTS_240HZ_GAME_MODE;
				report_rate = FTS_240HZ_REPORT_RATE;
				break;

			case FTS_GET_RATE_300:
				game_mode = FTS_360HZ_GAME_MODE;
				report_rate = FTS_360HZ_REPORT_RATE;
				break;

			case FTS_GET_RATE_600:
				game_mode = FTS_720HZ_GAME_MODE;
				report_rate = FTS_720HZ_REPORT_RATE;
				break;

			default:
				game_mode = FTS_240HZ_GAME_MODE;
				report_rate = FTS_240HZ_REPORT_RATE;
			break;
			}
			TPD_INFO("%s:set report_rate:%d", __func__, report_rate);
		} else {
			game_mode = FTS_240HZ_GAME_MODE;
			report_rate = FTS_240HZ_REPORT_RATE;
		}
	} else {
		if (ts_data->extreme_game_report_rate) {
			ts_data->extreme_game_flag = false;
			fts_read_reg(FTS_REG_CHARGER_MODE_EN, &regvalue);
			if(regvalue> 1) {
			regvalue  = (regvalue) & (0xF3);
			ret = fts_write_reg(FTS_REG_CHARGER_MODE_EN, regvalue);
			}
			fts_rate_white_list_ctrl(ts_data, ts->rate_ctrl_level);
			return ret;
		} else {
			game_mode = FTS_NOT_GAME_MODE;
			report_rate = FTS_120HZ_REPORT_RATE;
		}
	}
	SET_REG(FTS_REG_GAME_MODE_EN_BIT, game_mode);
	TPD_INFO("MODE_GAME, write 0x8B|23=0x%x, 0x88=%d", ts_data->ctrl_reg_state, report_rate);
	ret = fts_write_reg(FTS_REG_CTRL, ts_data->ctrl_reg_state);
	mdelay(15);
	ret = fts_write_reg(FTS_REG_REPORT_RATE, report_rate);
	return ret;
}

static int fts_set_idle_freq_mode(bool enable)
{
	int ret = 0;
	u8 regvalue = 0;

	TPD_INFO("%s: %s IDLE_GAME_GAME.\n", __func__, enable ? "Enter" : "Exit");

	if (enable) {
		ret = fts_write_reg(FTS_IDLE_FREQ_240, 0x03);
		if(ret < 0) {
			TPD_INFO("Failed to set idle 240 game mode config\n");
			return ret;
		}
	} else {
		ret = fts_write_reg(FTS_IDLE_FREQ_240, 0x06);
		if(ret < 0) {
			TPD_INFO("Failed to set idle 120 game mode config\n");
			return ret;
		}
	}
	ret = fts_read_reg(FTS_IDLE_FREQ_240, &regvalue);

	if(ret < 0) {
		TPD_INFO("Failed to get idle game mode config\n");
		return ret;
	}

	TPD_INFO("IDLE_GAME_GAME, write 0x89 is %d", regvalue);
	return ret;
}

static int fts_enable_headset_mode(struct chip_data_ft3683g *ts_data,
                                   bool enable)
{
	SET_REG(FTS_REG_HEADSET_MODE_EN_BIT, enable);
	TPD_INFO("MODE_HEADSET, write 0x8B|6=0x%x \n", enable);
	return fts_write_reg(FTS_REG_CTRL, ts_data->ctrl_reg_state);
}

static void fts_force_glove_mode(bool enable)
{
	int retval = 0;
	u8 regval = 0;

	TPD_INFO("%s: %s force glove mode.\n", __func__, enable ? "Enter" : "Exit");

	retval = fts_read_reg(FTS_REG_GLOVE_MODE_SWITCH, &regval);
	if(retval < 0) {
		TPD_INFO("Failed to get glove mode config\n");
		return;
	}
	TPD_INFO("%s: before edit glove mode reg_val=0x%x", __func__, regval);

	if(enable)
		retval = fts_write_reg(FTS_REG_GLOVE_MODE_SWITCH, 0x01);
	else
		retval = fts_write_reg(FTS_REG_GLOVE_MODE_SWITCH, 0x00);
	if(retval < 0) {
		TPD_INFO("Failed to set glove mode config\n");
		return;
	}

	retval = fts_read_reg(FTS_REG_GLOVE_MODE_SWITCH, &regval);
	if(retval < 0) {
		TPD_INFO("Failed to get glove mode config\n");
		return;
	}
	TPD_INFO("%s: after edit glove mode reg_val=0x%x", __func__, regval);
}

static int fts_mode_switch(void *chip_data, work_mode mode, int flag)
{
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	int ret = 0;

	switch (mode) {
	case MODE_NORMAL:
		TPD_INFO("MODE_NORMAL");
		break;

	case MODE_SLEEP:
		TPD_INFO("MODE_SLEEP, write 0xA5=3");
		if (ts_data->fod_info.fp_down) {
			TPD_INFO("fingerprint auto up");
			ts_data->ts->view_area_touched = 0;
			ts_data->fod_info.event_type = 0;
			ts_data->fod_info.fp_down = 0;
		}
		ret = fts_write_reg(FTS_REG_POWER_MODE, 0x03);

		if (ret < 0) {
			TPD_INFO("%s: enter into sleep failed.\n", __func__);
			goto mode_err;
		}
		ts_data->is_ic_sleep = true;
		break;

	case MODE_GESTURE:
		TPD_INFO("MODE_GESTURE, Melo, ts->is_suspended = %d \n",
		         ts_data->ts->is_suspended);

		if (ts_data->ts->is_suspended) {                             /* do not pull up reset when doing resume*/
			if (ts_data->is_ic_sleep == true) {
				fts_reset(ts_data);
			}
		}

		ret = fts_enable_black_gesture(ts_data, flag);

		if (ret < 0) {
			TPD_INFO("%s: enable gesture failed.\n", __func__);
			goto mode_err;
		}

		break;

	/*    case MODE_GLOVE:*/
	/*        break;*/

	case MODE_GLOVE:
		TPD_INFO("MODE_GLOVE, Melo, ts->glove_enable = %d ts->pocket_prevent_mode = %d flag = %d\n",
		         ts_data->ts->glove_enable, ts_data->ts->pocket_prevent_mode, flag);

		fts_force_glove_mode(flag);

		break;

	case MODE_EDGE:
		ret = fts_enable_edge_limit(ts_data, flag);

		if (ret < 0) {
			TPD_INFO("%s: enable edg limit failed.\n", __func__);
			goto mode_err;
		}

		break;

	case MODE_FACE_DETECT:
		break;

	case MODE_CHARGE:
		ret = fts_enable_charge_mode(ts_data, flag);

		if (ret < 0) {
			TPD_INFO("%s: enable charge mode failed.\n", __func__);
			goto mode_err;
		}

		break;

	case MODE_GAME:
		ret = fts_enable_game_mode(ts_data, flag);

		if (ret < 0) {
			TPD_INFO("%s: enable game mode failed.\n", __func__);
			goto mode_err;
		}

		break;

	case MODE_HEADSET:
		ret = fts_enable_headset_mode(ts_data, flag);

		if (ret < 0) {
			TPD_INFO("%s: enable headset mode failed.\n", __func__);
			goto mode_err;
		}

		break;

	default:
		TPD_INFO("%s: Wrong mode.\n", __func__);
		goto mode_err;
	}
	return 0;
mode_err:
	return ret;
}

static void fts_report_rate(void *chip_data, int value)
{
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	int ret = 0;
	int regvalue = 0;

	TPD_INFO("fts_report_rate_ctrl to  value: %d", value);
	if (ts_data == NULL) {
		return;
	}

	if (ts_data->ts->is_suspended) {
		return;
	}

	switch(value) {
	case REPORT_FPS_DEFAULT:
		regvalue = FTS_120HZ_REPORT_RATE;
		break;
	case REPORT_FPS_180HZ:
		regvalue = FTS_180HZ_REPORT_RATE;
		break;
	case REPORT_FPS_240HZ:
		regvalue = FTS_240HZ_REPORT_RATE;
		break;
	default:
		TPD_INFO("%s:default report rate = %d \n", __func__, value);
		return;
	}

	ret = fts_write_reg(FTS_REG_REPORT_RATE, regvalue);
	if (ret < 0) {
		TPD_INFO("write FTS_REG_REPORT_RATE fail");
		return;
	}
}
static int fts_send_temperature(void *chip_data, int temp, bool normal_mode);

#ifndef CONFIG_ARCH_QTI_VM
static int get_now_temp(struct chip_data_ft3683g *ts_data)
{
	struct touchpanel_data *ts = spi_get_drvdata(ts_data->ft_spi);
	int result = -40000;
	int ret = 0;

#ifdef CONFIG_TOUCHPANEL_TRUSTED_TOUCH
	if (atomic_read(&ts->trusted_touch_enabled) == 1) {
		TPD_INFO("%s: Trusted touch is already enabled, do not get temp\n", __func__);
		return ret;
	}
#endif

	if (ts->is_suspended) {
		TPD_INFO("%s : !ts->is_suspended\n", __func__);
		return ret;
	}

	ts->oplus_shell_themal = thermal_zone_get_zone_by_name("shell_back");

	if (IS_ERR(ts->oplus_shell_themal)) {
		TPD_INFO("%s Can't get shell_back\n", __func__);
		ts->oplus_shell_themal = NULL;
		ret = -1;
		return ret;
	}

	TPD_DEBUG("%s get shell_back ret:%d\n", __func__, ret);

	ret = thermal_zone_get_temp(ts->oplus_shell_themal, &result);
	if (ret < 0)
		TPD_INFO("%s can't thermal_zone_get_temp, ret=%d\n", __func__, ret);

	result = result / 1000;
	TPD_INFO("%s : temp is %d\n", __func__, result);

	if (result <= MAX_TEMPERATURE && result >= MIN_TEMPERATURE) {
		fts_send_temperature(ts->chip_data, result, true);
	} else {
		ts->monitor_data.abnormal_temperature_count++;
	}

	return ret;
}
#endif


/*
 * return success: 0; fail : negative
 */
static int fts_reset(void *chip_data)
{
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	int ret = 0;

	TPD_INFO("%s:call\n", __func__);
	fts_hw_reset(ts_data, RESET_TO_NORMAL_TIME);

	if (ts_data->ts->temperature_detect_shellback_support == true) {
#ifndef CONFIG_ARCH_QTI_VM
		get_now_temp(ts_data);
#endif
	}
	if (ts_data->tp_data_record_support) {
		if (ts_data->differ_mode == FTS_REG_WORK_MODE_SNR_MODE) {
			ret = fts_write_reg(FTS_REG_WORK_MODE, FTS_REG_WORK_MODE_SNR_MODE);
			if (ret < 0) {
				TPD_INFO("%s:open snr diff mode fail", __func__);
			}
			TPD_INFO("%s:open snr diff mode suc", __func__);
		} else if (ts_data->differ_mode == FTS_REG_WORK_MODE_FINAL_DIFF_MODE) {
			ret = fts_write_reg(FTS_REG_WORK_MODE, FTS_REG_WORK_MODE_FINAL_DIFF_MODE);
			if (ret < 0) {
				TPD_INFO("%s:open final diff mode fail", __func__);
			}
			TPD_INFO("%s:open final diff mode suc", __func__);
		}
	}
	return 0;
}

int ft3683g_rstpin_reset(void *chip_data)
{
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	TPD_INFO("%s:call\n", __func__);
	fts_hw_reset(ts_data, RESET_TO_FACTORY_TIME);

	return 0;
}
static int  fts_reset_gpio_control(void *chip_data, bool enable)
{
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	return fts_rstgpio_set(ts_data->hw_res, enable);
}

static int fts_get_vendor(void *chip_data, struct panel_info *panel_data)
{
	int len = 0;

	len = strlen(panel_data->fw_name);

	if ((len > 3) && (panel_data->fw_name[len - 3] == 'i') && \
	    (panel_data->fw_name[len - 2] == 'm')
	    && (panel_data->fw_name[len - 1] == 'g')) {
		TPD_INFO("tp_type = %d, panel_data->fw_name = %s\n", panel_data->tp_type,
		         panel_data->fw_name);
	}

	TPD_INFO("tp_type = %d, panel_data->fw_name = %s\n", panel_data->tp_type,
	         panel_data->fw_name);

	return 0;
}

static int fts_get_chip_info(void *chip_data)
{
	u8 cmd = 0;
	u8 id[2] = { 0 };

	fts_read_reg(FTS_REG_CHIP_ID, &id[0]);
	fts_read_reg(FTS_REG_CHIP_ID2, &id[1]);
	TPD_INFO("read chip id:0x%02x%02x", id[0], id[1]);

	if ((id[0] == FTS_VAL_CHIP_ID) && (id[1] == FTS_VAL_CHIP_ID2)) {
		return 0;
	}

	TPD_INFO("fw is invalid, need read boot id");
	cmd = 0x55;
	fts_write(&cmd, 1);
	msleep(12);
	cmd = 0x90;
	fts_read(&cmd, 1, id, 2);
	TPD_INFO("read boot id:0x%02x%02x", id[0], id[1]);

	if ((id[0] == FTS_VAL_BT_ID) && (id[1] == FTS_VAL_BT_ID2)) {
		return 0;
	}

	return 0;
}

static int fts_ftm_process(void *chip_data)
{
	int ret = 0;

	ret = fts_power_control(chip_data, true);
	if (ret < 0) {
		TPD_INFO("%s:power on fail", __func__);
		return ret;
	}

	ret = fts_mode_switch(chip_data, MODE_SLEEP, true);

	if (ret < 0) {
		TPD_INFO("%s:switch mode to MODE_SLEEP fail", __func__);
		return ret;
	}





	return 0;
}

static void fts_read_fod_info(struct chip_data_ft3683g *ts_data)
{
	int ret = 0;
	u8 cmd = FTS_REG_FOD_INFO;
	u8 val[FTS_REG_FOD_INFO_LEN] = { 0 };

	ret = fts_read(&cmd, 1, val, FTS_REG_FOD_INFO_LEN);

	if (ret < 0) {
		TPD_INFO("%s:read FOD info fail", __func__);
		return;
	}

	TPD_DEBUG("%s:FOD info buffer:%x %x %x %x %x %x %x %x %x", __func__, val[0],
	          val[1], val[2], val[3], val[4], val[5], val[6], val[7], val[8]);
	ts_data->fod_info.fp_id = val[0];
	ts_data->fod_info.event_type = val[1];

	if (val[8] == 0) {
		ts_data->fod_info.fp_down = 1;

	} else if (val[8] == 1) {
		ts_data->fod_info.fp_down = 0;
	}

	ts_data->fod_info.fp_area_rate = val[2];
	ts_data->fod_info.fp_x = (val[4] << 8) + val[5];
	ts_data->fod_info.fp_y = (val[6] << 8) + val[7];
}

static void fts_read_fod_error_info(struct chip_data_ft3683g *ts_data)
{
	int ret = 0;
	u8 cmd = FTS_REG_FOD_ERROR_INFO;
	u8 val[FTS_REG_FOD_ERROR_INFO_LEN] = { 0 };

	ret = fts_read(&cmd, 1, val, FTS_REG_FOD_ERROR_INFO_LEN);
	if (ret < 0) {
		TPD_INFO("%s:read FOD error info fail", __func__);
		return;
	}
	TPD_DEBUG("TP_FP_ERROR_REPORT:fingerprint error type:[%*ph]\n", FTS_REG_FOD_ERROR_INFO_LEN, val);
	switch (val[FTS_REG_FOD_ERROR_INFO_LEN - 1]) {
	case FTS_FINGERPRINT_AREA_NOT_MATCH:
		if (ts_data->monitor_data && ts_data->monitor_data->health_monitor_support) {
			tp_healthinfo_report(ts_data->monitor_data, HEALTH_REPORT, "fingerprint_area_not_match_count");
		}
		TPD_DEBUG("TP_FP_ERROR_REPORT:area size: 0x%x\n", val[12]);
		TPD_DEBUG("TP_FP_ERROR_REPORT:FINGERPRINT_AREA_NOT_MATCH\n");
		break;
	case FTS_ANOTHER_FINGER_ON_NON_FP_ZONE:
		if (ts_data->monitor_data && ts_data->monitor_data->health_monitor_support) {
			tp_healthinfo_report(ts_data->monitor_data, HEALTH_REPORT, "another_finger_on_non-fingerprint_zone_count");
		}
		TPD_DEBUG("TP_FP_ERROR_REPORT:x:0x%x,y:0x%x\n", (val[4] << 8) + val[5], (val[6] << 8) + val[7]);
		TPD_DEBUG("TP_FP_ERROR_REPORT:ANOTHER_FINGER_ON_NON_FP_ZONE\n");
		break;
	case FTS_FINGERPRINT_DOWN_BEFORE_FP_ENABLE:
		if (ts_data->monitor_data && ts_data->monitor_data->health_monitor_support) {
			tp_healthinfo_report(ts_data->monitor_data, HEALTH_REPORT, "fingerprint_down_before_fp_enable_count");
		}
		TPD_DEBUG("TP_FP_ERROR_REPORT:down time: %*ph\n", 4, val);
		TPD_DEBUG("TP_FP_ERROR_REPORT:FINGERPRINT_DOWN_BEFORE_FP_ENABLE\n");
		break;
	case FTS_FINGERPRINT_X_Y_NOT_MATCH:
		if (ts_data->monitor_data && ts_data->monitor_data->health_monitor_support) {
			tp_healthinfo_report(ts_data->monitor_data, HEALTH_REPORT, "fingerprint_x_y_not_match_count");
		}
		TPD_DEBUG("TP_FP_ERROR_REPORT:FINGERPRINT_X_Y_NOT_MATCH\n");
		break;
	case FTS_FINGERPRINT_OUT_MOVE_IN:
		if (ts_data->monitor_data && ts_data->monitor_data->health_monitor_support) {
			tp_healthinfo_report(ts_data->monitor_data, HEALTH_REPORT, "fingerprint_out_move_in_count");
		}
		TPD_DEBUG("TP_FP_ERROR_REPORT:FINGERPRINT_OUT_MOVE_IN\n");
		break;
	default:
		TPD_DEBUG("TP_FP_ERROR_REPORT:unknown fingerprint error type: 0x%x\n", val[FTS_REG_FOD_ERROR_INFO_LEN - 1]);
		break;
	}

	return;
}

static void fts_read_aod_info(struct chip_data_ft3683g *ts_data)
{
	int ret = 0;
	u8 cmd = FTS_REG_AOD_INFO;
	u8 val[FTS_REG_AOD_INFO_LEN] = { 0 };

	ret = fts_read(&cmd, 1, val, FTS_REG_AOD_INFO_LEN);

	if (ret < 0) {
		TPD_INFO("%s:read AOD info fail", __func__);
		return;
	}

	TPD_DEBUG("%s:AOD info buffer:%x %x %x %x %x %x", __func__, val[0],
		  val[1], val[2], val[3], val[4], val[5]);
	ts_data->aod_info.gesture_id = val[0];
	ts_data->aod_info.point_num = val[1];

	ts_data->aod_info.aod_x = (val[2] << 8) + val[3];
	ts_data->aod_info.aod_y = (val[4] << 8) + val[5];
}

static u8 fts_chip_get_reset_reason(struct chip_data_ft3683g *ts_data)
{
	int ret = 0;
	u8 reset_reason = 0;

	ret = fts_read_reg(FTS_REG_RESET_REASON, &reset_reason);
	TPD_INFO("reset_reason: %d", reset_reason);
	return reset_reason;
}

static u32 fts_u32_trigger_reason(void *chip_data, int gesture_enable,
                                  int is_suspended)
{
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	int ret = 0;
	u8 cmd = FTS_REG_POINTS;
	u32 result_event = 0;
	u8 *touch_buf = ts_data->touch_buf;
	u8 val = 0xFF;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int raw_num = tx_num * rx_num;
	int sc_num = tx_num + rx_num;
	int j = 0;
	int offect = 0;
	u8 reset_reason = 0;
	u16 buffer_len = 0;
	u32 click_level_index = 0;
	u32 click_level_value = 0;
	u8 mode_active = 0;

	fts_prc_queue_work(ts_data);

	if (!ts_data->differ_read_every_frame && (!ts_data->snr_is_reading)) {
		buffer_len = ts_data->touch_size;
	} else {
		buffer_len = ts_data->buffer_len;
	}
	memset(touch_buf, 0xFF, buffer_len);
	ret = fts_read(&cmd, 1, &touch_buf[0], buffer_len);
	if (gesture_enable && is_suspended) {
		ret = fts_read_reg(FTS_REG_GESTURE_EN, &val);
		if (ret < 0) {
			TPD_INFO("ft3683g_fts_read_reg  FTS_REG_GESTURE_EN error \n");
			return IRQ_IGNORE;
		}
		if (val == 0x01) {
			return IRQ_GESTURE;
		} else {
			TPD_INFO("gesture not enable in fw, don't process gesture");
			reset_reason = fts_chip_get_reset_reason(ts_data);
			if (reset_reason != FTS_RST_REASON_UNKNOWN && reset_reason != FTS_RST_REASON_FWUPDATE) {
				tp_exception_report(&ts_data->ts->exception_data, EXCEP_GESTURE, "gesture not enable", sizeof("gesture not enable"));
			}
		}
	}
	if (ret < 0) {
		TPD_INFO("read touch buffer fail");
		return IRQ_IGNORE;
	}
	ts_data->gesture_flag = 0;
	if (ts_data->ts->palm_to_sleep_enable && !ts_data->ts->is_suspended) {
		ret = fts_read_reg(FTS_REG_PALM_TO_SLEEP_STATUS, &val);
		if (ret < 0) {
			TPD_INFO("ft3683g_fts_read_reg  PALM_TO_SLEEP_STATUS  error \n");
		}

		if(val == 1) {
			result_event = IRQ_PALM;
			TPD_INFO("fts_enable_palm_to_sleep enable\n");
		}
	}
	if (!ts_data->snr_data_is_ready) {
		memcpy(ts_data->snr_buf, ts_data->touch_buf, buffer_len * sizeof(u8));
		ts_data->snr_data_is_ready = 1;
	}
	if (ts_data->differ_read_every_frame && ts_data->tp_data_record_support && ts_data->tp_differ_version == FTS_DIFFER_VERSION_V1) {
		ts_data->snr_count = touch_buf[135];
		offect = 136;
		for (j = 0; j < raw_num; j = j + 1) {
				ts_data->diff_buf[j] = (int)(short)((touch_buf[offect + 2*j] << 8) +
								(touch_buf[offect + 2*j + 1]));
		}

		offect += 2 * raw_num;
		for (j = 0; j < sc_num; j = j + 1) {
				ts_data->sc_water[j] = (int)(short)((touch_buf[offect + 2*j] << 8) +
								(touch_buf[offect + 2*j + 1]));
		}

		offect += 2 * sc_num + 8;
		for (j = 0; j < sc_num; j = j + 1) {
				ts_data->sc_nomal[j] = (int)(short)((touch_buf[offect + 2*j] << 8) +
								(touch_buf[offect + 2*j + 1]));
		}
	}

	if ((touch_buf[1] == 0xFF) && (touch_buf[2] == 0xFF) && (touch_buf[3] == 0xFF)) {
		TPD_INFO("Need recovery TP state");
		reset_reason = fts_chip_get_reset_reason(ts_data);
		if (reset_reason != FTS_RST_REASON_EXTERNAL){
			TPD_INFO("finished probe, report exception");
			tp_exception_report(&ts_data->ts->exception_data, EXCEP_TOUCH_IC_RESET, "fw_status_err", sizeof("fw_status_err"));
		}
		return IRQ_FW_AUTO_RESET;
	}

	/*glove mode*/
	TP_SPECIFIC_PRINT(ts_data->tp_index, ts_data->print_count_glove, "GloveMode:%d\n", (touch_buf[0]&0x40) ? 1 : 0);
	TP_DEBUG_RATELIMIT_PRINT(ts_data->tp_index, ts_data->print_count, "PalmMode:%d, WaterMode:%d\n", (touch_buf[0]&0x02) ? 1 : 0, (touch_buf[0]&0x01) ? 1 : 0);
	TP_DEBUG_RATELIMIT_PRINT(ts_data->tp_index, ts_data->print_count, "NoiseMode:%d, TemperatureMode:%d\n", (touch_buf[0]&0x10) ? 1 : 0, (touch_buf[0]&0x20) ? 1 : 0);
	mode_active = (touch_buf[0] & (0x40 | 0x02 | 0x01 | 0x10 | 0x20)) ? 1 : 0;
	if (mode_active) {
		click_level_index = 0;
		click_level_value = 0;
		TP_DEBUG_RATELIMIT_PRINT(ts_data->tp_index, ts_data->print_count,
			"ClickSensitiveLevel:%u, value:%u\n",
			click_level_index, click_level_value);
	} else {
		click_level_index = ts_data->ts->click_sensitive_level_chosen;
		if (click_level_index >= CLICK_SENSITIVE_LEVEL_NUM) {
			click_level_index = CLICK_SENSITIVE_LEVEL_NUM - 1;
		}
		click_level_value = ts_data->ts->click_sensitive_level_array[click_level_index];
		TP_DEBUG_RATELIMIT_PRINT(ts_data->tp_index, ts_data->print_count,
				"ClickSensitiveLevel:%u, value:%u\n",
				click_level_index, click_level_value);
	}
	if (buffer_len >= MAX_DIFF_L8) {
		TP_DEBUG_RATELIMIT_PRINT(ts_data->tp_index, ts_data->print_count, "ResetType:%d, downThd:%d, upThd:%d, idleThd:%d, maxDiff:%d\n",
			touch_buf[RESET_TYPE], touch_buf[DOWN_THD], touch_buf[UP_THD], touch_buf[IDLE_THD], ((touch_buf[MAX_DIFF_H8] << 8) + touch_buf[MAX_DIFF_L8]));
	}
	/*confirm need print debug info*/
	if (touch_buf[0] != ts_data->irq_type) {
		SET_BIT(result_event, IRQ_FW_HEALTH);
	}

	ts_data->irq_type = touch_buf[0];

	/*normal touch*/
	SET_BIT(result_event, IRQ_TOUCH);
	TPD_DEBUG("%s, fgerprint, is_suspended = %d, fp_en = %d, ", __func__,
	          is_suspended, ts_data->fp_en);
	TPD_DEBUG("%s, fgerprint, touched = %d, event_type = %d, fp_down = %d, fp_down_report = %d, ",
	          __func__, ts_data->ts->view_area_touched, ts_data->fod_info.event_type,
	          ts_data->fod_info.fp_down, ts_data->fod_info.fp_down_report);

	if (!is_suspended && ts_data->fp_en) {
		fts_read_fod_info(ts_data);

		if ((ts_data->fod_info.event_type == FTS_EVENT_FOD)
		    && (ts_data->fod_info.fp_down)) {
			if (!ts_data->fod_info.fp_down_report) {    /* 38, 1, 0*/
				ts_data->fod_info.fp_down_report = 1;
				SET_BIT(result_event, IRQ_FINGERPRINT);
				TPD_DEBUG("%s, fgerprint, set IRQ_FINGERPRINT when fger down but not reported! \n",
				          __func__);
				ts_data->fod_trigger = TYPE_FOD_TRIGGER;
			}

			/*            if (ts_data->fod_info.fp_down_report) {      38, 1, 1*/
			/*            }*/

		} else if ((ts_data->fod_info.event_type == FTS_EVENT_FOD)
		           && (!ts_data->fod_info.fp_down)) {
			if (ts_data->fod_info.fp_down_report) {     /* 38, 0, 1*/
				ts_data->fod_info.fp_down_report = 0;
				SET_BIT(result_event, IRQ_FINGERPRINT);
				TPD_DEBUG("%s, fgerprint, set IRQ_FINGERPRINT when fger up but still reported! \n",
				          __func__);
			}

			/*                if (!ts_data->fod_info.fp_down_report) {     38, 0, 0*/
			/*                }*/
		}
	}

	return result_event;
}

static void fts_print_differ(int *diff_buf, u32 diff_size, int rols, int cols)
{
	int ret = 0;
	int i, j;
	u8 *out_str;
	u32 out_size;

	if (diff_buf == NULL) {
		TPD_INFO("focal:%s:diff_buf is NULL\n", __func__);
		return;
	}

	out_size = (rols*cols)*5 + 1;
	out_str = kzalloc(out_size, GFP_KERNEL);
	if (out_str == NULL) {
		TPD_INFO("focal:%s:kmalloc error\n", __func__);
		return;
	}

	for (i = 0; i < rols; ++i) {
		ret = 0;
		for (j = 0; j < cols; ++j) {
			ret += snprintf(out_str+ret, out_size - ret, "%5d", diff_buf[i*cols + j]);
		}
		TPD_INFO("[%2d]%s\n", i, out_str);
	}

	kfree(out_str);
}

static void fts_print_differ_v2(struct chip_data_ft3683g *ts_data, int tx_num, int rx_num, char *point_buf, char *edge_buf, int point_num)
{
	int ret = 0;
	int i, j;
	u8 *out_str;
	u32 out_size;
	short data_16;
	int offset = 0;
	u8 *touch_buf = ts_data->touch_buf;
	u8 *data_8;

	out_size = rx_num * 6 + 7;
	out_str = kzalloc(out_size, GFP_KERNEL);
	if (out_str == NULL) {
		TPD_INFO("focal:%s:kmalloc error\n", __func__);
		return;
	}

	for (j = 0; j < rx_num; j++) {
		ret += snprintf(out_str + ret, out_size - ret, ", RX%02d", j);
	}
	TPD_INFO("    %s\n", out_str);

	offset += 154;

	data_8 = &touch_buf[offset];
	for (i = 0; i < tx_num; i++) {
		ret = 0;
		for (j = 0; j < rx_num; j++) {
			data_16 = (short)((data_8[0] << 8) + data_8[1]);
			ret += snprintf(out_str + ret, out_size - ret, ",%5d", data_16);
			data_8 += 2;
		}
		TPD_INFO("TX%02d%s\n", i, out_str);
	}

	ret = 0;
	for (j = 0; j < rx_num; j++) {
		data_16 = (short)((data_8[0] << 8) + data_8[1]);
		ret += snprintf(out_str + ret, out_size - ret, ",%5d", data_16);
		data_8 += 2;
	}
	TPD_INFO("W_RX%s\n", out_str);

	ret = 0;
	for (j = 0; j < tx_num; j++) {
		data_16 = (short)((data_8[0] << 8) + data_8[1]);
		ret += snprintf(out_str + ret, out_size - ret, ",%5d", data_16);
		data_8 += 2;
	}
	TPD_INFO("W_TX%s\n", out_str);

	ret = 0;
	for (j = 0; j < rx_num; j++) {
		data_16 = (short)((data_8[0] << 8) + data_8[1]);
		ret += snprintf(out_str + ret, out_size - ret, ",%5d", data_16);
		data_8 += 2;
	}
	TPD_INFO("N_RX%s\n", out_str);

	ret = 0;
	for (j = 0; j < tx_num; j++) {
		data_16 = (short)((data_8[0] << 8) + data_8[1]);
		ret += snprintf(out_str + ret, out_size - ret, ",%5d", data_16);
		data_8 += 2;
	}
	TPD_INFO("N_TX%s\n", out_str);

	ts_data->time_since_last_frame = (u32)((touch_buf[144] << 24) + (touch_buf[145] << 16) + (touch_buf[146] << 8) + touch_buf[147]);
	ts_data->frame_cnt = (u32)((touch_buf[148] << 24) + (touch_buf[149] << 16) + (touch_buf[150] << 8) + touch_buf[151]);
	ts_data->cur_noise = (u16)((touch_buf[142] << 8) + touch_buf[143]);
	TPD_INFO("frameCnt %u, charge %d, game %d, Tempe %d, BaseRefresh %d, RawErr %d, Freq %d, Noise %d, NoiseSta %u, DS %d, time %u, END.\n", \
			ts_data->frame_cnt, ts_data->ts->is_usb_checked, touch_buf[139], ts_data->tp_temperature, touch_buf[136], touch_buf[137], \
			touch_buf[138], ts_data->cur_noise, touch_buf[141], touch_buf[140], ts_data->time_since_last_frame);
	TPD_INFO("tpic: fingerNum %u, fID (%s), edge (%s) END.\n", point_num, point_buf, edge_buf);

	for (i = 0; i < 8; i++) {
		ret = 0;
		for (j = 0; j < rx_num; j++) {
			data_16 = (short)((data_8[0] << 8) + data_8[1]);
			ret += snprintf(out_str + ret, out_size - ret, ",%5d", data_16);
			data_8 += 2;
		}
		TPD_INFO("reg%d%s\n", i, out_str);
	}
	kfree(out_str);
}

static int fts_get_touch_points(void *chip_data, struct point_info *points,
                                int max_num)
{
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	int i = 0;
	int obj_attention = 0;
	int base = 0;
	int event_num = 0;
	u8 finger_num = 0;
	u8 pointid = 0;
	int base_prevent = 0;
	u8 event_flag = 0;
	u8 *touch_buf = ts_data->touch_buf;
	u8 touch_etype = 0;
	struct touchpanel_snr *snr = ts_data->ts->snr;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int pitch_x_width = 0;
	int pitch_y_width = 0;
	char point_buff[150] = {0};
	char edge_buff[200] = {0};
	char state_buff[3] = {0};
	int point_ret = 0;
	int edge_ret = 0;
	touch_etype = ((touch_buf[FTS_TOUCH_E_NUM] >> 4) & 0x0F);

	ts_data->ft3683_grip_v2_support = true;

	if (ts_data->snr_read_support) {
		for (i = 0; i < max_num; i++) {
			snr[i].point_status = 0;
		}
	}

	switch (touch_etype) {
	case TOUCH_DEFAULT:
			finger_num = touch_buf[1] & 0xFF;

			if (finger_num > max_num) {
				TPD_INFO("invalid point_num(%d),max_num(%d)", finger_num, max_num);
				return -EIO;
			}

			for (i = 0; i < max_num; i++) {
				base = 6 * i;
				pointid = (touch_buf[4 + base]) >> 4;

				if (pointid >= FTS_MAX_ID) {
					break;

				} else if (pointid >= max_num) {
					TPD_INFO("ID(%d) beyond max_num(%d)", pointid, max_num);
					return -EINVAL;
				}

				event_num++;
				if (!ts_data->high_resolution_support && !ts_data->high_resolution_support_x8) {
					points[pointid].x = ((touch_buf[2 + base] & 0x0F) << 8) + (touch_buf[3 + base] & 0xFF);
					points[pointid].y = ((touch_buf[4 + base] & 0x0F) << 8) + (touch_buf[5 + base] & 0xFF);
					points[pointid].touch_major = touch_buf[7 + base];
					points[pointid].width_major = touch_buf[7 + base];
					points[pointid].z =  touch_buf[7 + base];
					event_flag = (touch_buf[2 + base] >> 6);
				} else if (ts_data->high_resolution_support_x8) {
					points[pointid].x = (((touch_buf[2 + base] & 0x0F) << 11) +
					                     ((touch_buf[3 + base] & 0xFF) << 3) +
					                     ((touch_buf[6 + base] >> 5) & 0x07));
					points[pointid].y = (((touch_buf[4 + base] & 0x0F) << 11) +
					                     ((touch_buf[5 + base] & 0xFF) << 3) +
					                     ((touch_buf[6 + base] >> 2) & 0x07));
					points[pointid].touch_major = touch_buf[7 + base];
					points[pointid].width_major = touch_buf[7 + base];
					points[pointid].z =  touch_buf[7 + base];
					event_flag = (touch_buf[2 + base] >> 6);
				}

				points[pointid].status = 0;

				if ((event_flag == 0) || (event_flag == 2)) {
					points[pointid].status = 1;
					obj_attention |= (1 << pointid);

					if (finger_num == 0) {
						TPD_INFO("abnormal touch data from fw");
						return -EIO;
					}
				}
			}

			if (event_num == 0) {
				TPD_INFO("no touch point information");
				return -EIO;
			}
		break;

	case TOUCH_PROTOCOL_v2:

		event_num = touch_buf[FTS_TOUCH_E_NUM] & 0x0F;
		if (((!event_num) && (!ts_data->snr_is_reading) && (!ts_data->differ_read_every_frame)) || (event_num > max_num)) {
			TPD_INFO("invalid touch event num(%d)", event_num);
			return -EINVAL;
		}
		/*ts_data->touch_event_num = event_num;*/

		for (i = 0; i < event_num; i++) {
			base = FTS_ONE_TCH_LEN_V2 * i + 4;
			base_prevent = 4 * i;
			pointid = (touch_buf[FTS_TOUCH_OFF_ID_YH + base]) >> 4;
			if (pointid >= max_num) {
				TPD_INFO("touch point ID(%d) beyond max_touch_number(%d)",
						pointid, max_num);
					return -EINVAL;
				}

				/*points[i].id = pointid;*/
				event_flag = touch_buf[FTS_TOUCH_OFF_E_XH + base] >> 6;

				points[pointid].x = ((touch_buf[FTS_TOUCH_OFF_E_XH + base] & 0x0F) << 12) \
							  + ((touch_buf[FTS_TOUCH_OFF_XL + base] & 0xFF) << 4) \
							  + ((touch_buf[FTS_TOUCH_OFF_PRE + base] >> 4) & 0x0F);

				points[pointid].y = ((touch_buf[FTS_TOUCH_OFF_ID_YH + base] & 0x0F) << 12) \
							  + ((touch_buf[FTS_TOUCH_OFF_YL + base] & 0xFF) << 4) \
							  + (touch_buf[FTS_TOUCH_OFF_PRE + base] & 0x0F);

				/*points[pointid].x = points[pointid].x  / FTS_HI_RES_X_MAX;*/
				/*points[pointid].y = points[pointid].y  / FTS_HI_RES_X_MAX;*/
				points[pointid].touch_major = touch_buf[FTS_TOUCH_OFF_AREA + base];
				points[pointid].width_major = touch_buf[FTS_TOUCH_OFF_AREA + base];
				points[pointid].z = touch_buf[FTS_TOUCH_OFF_AREA + base];
				if (ts_data->ft3683_grip_v2_support) {
					if (pointid < 7) {
						points[pointid].tx_press = touch_buf[94 + base_prevent];
						points[pointid].rx_press = touch_buf[95 + base_prevent];
						points[pointid].tx_er = touch_buf[97 + base_prevent];
						points[pointid].rx_er = touch_buf[96 + base_prevent];
					} else {
						points[pointid].tx_press = 0;
						points[pointid].rx_press = 0;
						points[pointid].tx_er = 0;
						points[pointid].rx_er = 0;
					}
					TPD_DEBUG("grip info points[%d] tx_press=%d rx_press=%d tx_er=%d rx_er=%d",
							pointid, points[pointid].tx_press, points[pointid].rx_press,
							points[pointid].tx_er, points[pointid].rx_er);
				}

				if (points[pointid].touch_major <= 0) points[pointid].touch_major = 0x09;
				if (points[pointid].width_major <= 0) points[pointid].width_major = 0x09;

				pitch_x_width = ts_data->resolution_info->max_x / ts_data->hw_res->tx_num;
				pitch_y_width = ts_data->resolution_info->max_y / ts_data->hw_res->rx_num;

				if (ts_data->snr_read_support) {
					if (snr[pointid].doing && points[pointid].x && points[pointid].y) {
						snr[pointid].point_status = 1;
						snr[pointid].x = points[pointid].x;
						snr[pointid].y = points[pointid].y;
						snr[pointid].width_major = points[pointid].width_major;
						snr[pointid].channel_x = snr[pointid].x / pitch_x_width;
						snr[pointid].channel_y = snr[pointid].y / pitch_y_width;
						GET_LEN_BY_WIDTH_MAJOR(snr[pointid].width_major, &snr[pointid].area_len);
						TPD_DEBUG("snr%d: [%d %d, %d] {%d %d} len %d \n",
							pointid, snr[pointid].x, snr[pointid].y, snr[pointid].width_major,
							snr[pointid].channel_x, snr[pointid].channel_y, snr[pointid].area_len);
					}
				}

				points[pointid].status = 0;

				if ((event_flag == 0) || (event_flag == 2)) {
					points[pointid].status = 1;
					obj_attention |= (1 << pointid);

				}
			if (ts_data->differ_read_every_frame && (ts_data->tp_differ_version == FTS_DIFFER_VERSION_V2)) {
				if (event_flag == 0) {
					strcpy(state_buff, "E1");
				} else if (event_flag == 2) {
					strcpy(state_buff, "E2");
				} else {
					strcpy(state_buff, "E3");
				}
				if (i < event_num - 1) {
					point_ret += snprintf(point_buff + point_ret, 149 - point_ret, "%u %u %u %s,", pointid, points[pointid].x, points[pointid].y, state_buff);
					edge_ret += snprintf(edge_buff + edge_ret, 149 - edge_ret, "%u %u %u %u %u,", pointid, \
							points[pointid].tx_press, points[pointid].rx_press, points[pointid].tx_er, points[pointid].rx_er);
				} else {
					point_ret += snprintf(point_buff + point_ret, 149 - point_ret, "%u %u %u %s", pointid, points[pointid].x, points[pointid].y, state_buff);
					edge_ret += snprintf(edge_buff + edge_ret, 149 - edge_ret, "%u %u %u %u %u", pointid, \
							points[pointid].tx_press, points[pointid].rx_press, points[pointid].tx_er, points[pointid].rx_er);
				}
			}
		}
			break;
	default:
			break;
	}

	if (ts_data->differ_read_every_frame && (ts_data->tp_differ_version == FTS_DIFFER_VERSION_V1)) {
		TPD_INFO("mutual diff data count:%u\n", ts_data->snr_count);
		fts_print_differ(ts_data->diff_buf, tx_num*rx_num, tx_num, rx_num);
		TPD_INFO("sc_water diff data:");
		fts_print_differ(ts_data->sc_water, tx_num+rx_num, 1, tx_num + rx_num);
		TPD_INFO("sc_normal diff data:");
		fts_print_differ(ts_data->sc_nomal, tx_num*rx_num, 1, tx_num + rx_num);
	} else if (ts_data->differ_read_every_frame && (ts_data->tp_differ_version == FTS_DIFFER_VERSION_V2)) {
		TPD_INFO("tp differ print start.\n");
		fts_print_differ_v2(ts_data, tx_num, rx_num, point_buff, edge_buff, event_num);
	}

	if (event_num == 0) {
		return -EINVAL;
	}

	if (ts_data->touch_analysis_support && ts_data->ta_flag) {
		ts_data->ta_flag = 0;
		ts_data->ta_size = ts_data->touch_size;
		if (ts_data->ta_buf && ts_data->ta_size)
			memcpy(ts_data->ta_buf, ts_data->touch_buf, ts_data->ta_size);
		wake_up_interruptible(&ts_data->ts_waitqueue);
	}

	return obj_attention;
}

static void fts_health_report(void *chip_data, struct monitor_data *mon_data)
{
	int ret = 0;
	u8 val = 0;
	u8 cmd = 0;
	u8 health_buf[3] = {0};
	u8 ucMcFreVal[2] = {0};
	u8 waterlevel = 0;          /* water level: high 3 bits of 3rd byte, 0~7 */
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	char *freq_str = NULL;
	char *waterlevel_str = NULL;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int event_num = 0;
	char point_buff[FTS_POINTER_BUFFER_LEN] = {0};
	char edge_buff[FTS_EDG_BUFFER_LEN] = {0};

	if (IS_ERR_OR_NULL(ts_data) || IS_ERR_OR_NULL(ts_data->monitor_data)) {
		TPD_INFO("%s:NULL Pointer", __func__);
		return;
	}

	cmd = FTS_REG_POINTS;
	ret = fts_read(&cmd, 1, health_buf, sizeof(health_buf));
	if (ret < 0) {
		TPD_INFO("%s:read health register(0x01) fail", __func__);
		return;
	}
	val = health_buf[0];

	if (val & 0x01) {
		ts_data->water_mode = 1;
		TPD_INFO("%s:water flag =%d", __func__, ts_data->water_mode);
	}
	else {
		ts_data->water_mode = 0;
		TPD_INFO("%s:water flag error", __func__);
	}

	TPD_INFO("Health register(0x01):0x%x", val);
	if (((val & 0x01) && !ts_data->is_in_water)
	    || ts_data->monitor_data->health_simulate_trigger) {
		TPD_DETAIL("Health register(0x01):Water Shield");
		tp_healthinfo_report(mon_data, HEALTH_REPORT, HEALTH_REPORT_SHIELD_WATER);
		ts_data->is_in_water = true;
	}
	if ((val & 0x02)
	    || ts_data->monitor_data->health_simulate_trigger) {
		TPD_DETAIL("Health register(0x01):Palm Shield");
		tp_healthinfo_report(mon_data, HEALTH_REPORT, HEALTH_REPORT_SHIELD_PALM);
	}
	if ((val & 0x04)
	    || ts_data->monitor_data->health_simulate_trigger) {
		TPD_DETAIL("Health register(0x01):Freq Hopping");
		tp_healthinfo_report(mon_data, HEALTH_REPORT, HEALTH_REPORT_HOPPING);
	}
	if ((val & 0x08)
	    || ts_data->monitor_data->health_simulate_trigger) {
		TPD_DETAIL("Health register(0x01):Base Refresh");
		tp_healthinfo_report(mon_data, HEALTH_REPORT, HEALTH_REPORT_BASELINE_ERR);
		focal_get_differ_version(ts_data);
		cmd = FTS_REG_POINTS;
		ret = fts_read(&cmd, 1, ts_data->touch_buf, ts_data->buffer_len);
		if ((ts_data->tp_differ_version == FTS_DIFFER_VERSION_V2) && !ret) {
			TPD_INFO("baseline error ,tp differ print start.\n");
			fts_print_differ_v2(ts_data, tx_num, rx_num, point_buff, edge_buff, event_num);
		}
	}
	if ((val & 0x10)
	    || ts_data->monitor_data->health_simulate_trigger) {
		if (ts_data->charger_connected) {
			TPD_DETAIL("Health register(0x01):Big Noise in Charge");
			tp_healthinfo_report(mon_data, HEALTH_REPORT, HEALTH_REPORT_NOISE_CHARGE);
		} else {
			TPD_DETAIL("Health register(0x01):Big Noise");
			tp_healthinfo_report(mon_data, HEALTH_REPORT, HEALTH_REPORT_NOISE);
		}
	}
	if ((val & 0x20)
	    || ts_data->monitor_data->health_simulate_trigger) {
		TPD_DETAIL("Health register(0x01):Temperature");
		tp_healthinfo_report(mon_data, HEALTH_REPORT, HEALTH_REPORT_TEMP_DRIFT);
	}
	if ((val & 0x40) && (ts_data->glove_mode_flag == 0)) {
		TPD_INFO("Health register(0x01):GloveMode:1");
		ts_data->glove_mode_flag = 1;
		tp_healthinfo_report(mon_data, HEALTH_GLOVE, &ts_data->glove_mode_flag);
		ts_data->glove_mode_enable_count++;
	}
	if ((!(val & 0x40)) && (ts_data->glove_mode_flag == 1)) {
		TPD_INFO("Health register(0x01):GloveMode:0");
		ts_data->glove_mode_flag = 0;
		tp_healthinfo_report(mon_data, HEALTH_GLOVE, &ts_data->glove_mode_flag);
	}
	if ((val & 0x40)
	    || ts_data->monitor_data->health_simulate_trigger) {
		TPD_DETAIL("Health register(0x01):Chanel Fill");
		tp_healthinfo_report(mon_data, HEALTH_REPORT, HEALTH_REPORT_CHANEL_FILL);
	}
	if ((val & 0x80)
	    || ts_data->monitor_data->health_simulate_trigger) {
		if (!ts_data->fod_trigger) {
			TPD_DETAIL("Health register(0x01):FOD");
			ts_data->fod_trigger = TYPE_SMALL_FOD_TRIGGER;
		}
	}

	/* Water level health report (high 3 bits of 3rd byte, 0~7, report only when level changes) */
	/* bit7 Accumulated water at bottom, bit6 rainstorm, bit5 light rain */
	waterlevel = (health_buf[2] >> 5) & 0x07;
	if (waterlevel != ts_data->last_waterlevel) {
		TPD_INFO("Health register(0x01):water level:%u -> %u", ts_data->last_waterlevel, waterlevel);
		waterlevel_str = kzalloc(16, GFP_KERNEL);
		if (!waterlevel_str) {
			TPD_INFO("waterlevel_str kzalloc failed.\n");
		} else {
			snprintf(waterlevel_str, 16, "water_level_%u", waterlevel);
			tp_healthinfo_report(mon_data, HEALTH_REPORT, waterlevel_str);
			kfree(waterlevel_str);
			ts_data->last_waterlevel = waterlevel;
		}
	}

	/*ret = ft3681_fts_read_reg(FTS_REG_HEALTH_1, &val);
	TPD_INFO("Health register(0xFD):0x%x(water-flag:%d / noise-flag:%d)" / no-suitable-freq:%d)",
			val, (val & 0x01), (val & 0x02), ((val & 0x10) >> 4));*/
	/*if (val & 0x10 && !mon_data->no_suitable_freq) {
		mon_data->no_suitable_freq = true;
		tp_healthinfo_report(mon_data, HEALTH_REPORT, HEALTH_REPORT_NO_SUITABLE_FREQ);
	}*/
/* 	ret = fts_read_reg(FTS_REG_HEALTH_2, &val);
	TPD_INFO("Health register(0xFE):0x%x(work-freq:%u)", val, val); */
	cmd = FTS_REG_HEALTH_2;
	ret = fts_read(&cmd, 1, ucMcFreVal, 2);
	TPD_INFO("Health register(0xFE):0x%02x%02x", ucMcFreVal[0], ucMcFreVal[1]);
	if ((mon_data->work_freq && mon_data->work_freq != val)
	    || ts_data->monitor_data->health_simulate_trigger) {
		freq_str = kzalloc(10, GFP_KERNEL);
		if (!freq_str) {
			TPD_INFO("freq_str kzalloc failed.\n");
		} else {
			snprintf(freq_str, 10, "freq_%u", val);
			tp_healthinfo_report(mon_data, HEALTH_REPORT, freq_str);
			kfree(freq_str);
		}
	}
	mon_data->work_freq = val;

	/* baseline_negative*/
	cmd = FTS_REG_HEALTH_BASELINE;
	ret = fts_read_reg(cmd, &val);
	if (ret < 0) {
		TPD_INFO("read baseline_negative reg failed.\n");
		return;
	}
	/* bit4:1---baseline neg */
	if (val & 0x10) {
		TPD_INFO("Health register(0x01):baseline_negative:1");
		tp_healthinfo_report(mon_data, HEALTH_REPORT, HEALTH_REPORT_BASELINE_NEGATIVE);
		focal_get_differ_version(ts_data);
		cmd = FTS_REG_POINTS;
		ret = fts_read(&cmd, 1, ts_data->touch_buf, ts_data->buffer_len);
		if ((ts_data->tp_differ_version == FTS_DIFFER_VERSION_V2) && !ret) {
			TPD_INFO("baseline negative, tp differ print start.\n");
			fts_print_differ_v2(ts_data, tx_num, rx_num, point_buff, edge_buff, event_num);
			if (ts_data->ts->exception_upload_support) {
				tp_exception_report(&ts_data->ts->exception_data, EXCEP_BASELINE_ERR, "Baseline_negative", sizeof("Baseline_negative"));
			}
		}
	}
}

static int fts_get_gesture_info(void *chip_data, struct gesture_info *gesture)
{
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	int ret = 0;
	u8 cmd = FTS_REG_GESTURE_OUTPUT_ADDRESS;
	u8 buf[FTS_GESTURE_DATA_LEN] = { 0 };
	u8 gesture_id = 0;
	u8 point_num = 0;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int event_num = 0;
	char point_buff[150] = {0};
	char edge_buff[200] = {0};


	ret = fts_read(&cmd, 1, &buf[2], FTS_GESTURE_DATA_LEN - 2);

	if (ret < 0) {
		TPD_INFO("read gesture data fail");
		return ret;
	}

	gesture_id = buf[2];
	point_num = buf[3];
	TPD_INFO("gesture_id=%d, point_num=%d", gesture_id, point_num);

	if (gesture == NULL) {
		TPD_INFO("gesture == NULL, return\n\
			gesture->Point_start.x = %d\n;\
			gesture->Point_start.y = %d\n;\
			gesture->Point_end.x = %d\n;\
			gesture->Point_end.y = %d\n;\
			gesture->Point_1st.x = %d\n;\
			gesture->Point_1st.y = %d\n;\
			gesture->Point_2nd.x = %d\n;\
			gesture->Point_2nd.y = %d\n;\
			gesture->Point_3rd.x = %d\n;\
			gesture->Point_3rd.y = %d\n;\
			gesture->Point_4th.x = %d\n;\
			gesture->Point_4th.y = %d\n;"
			 , ((buf[4] << 8) + buf[5]), ((buf[6] << 8) + buf[7]), ((buf[8] << 8) + buf[9])
			 , ((buf[10] << 8) + buf[11]), ((buf[12] << 8) + buf[13]),
			 ((buf[14] << 8) + buf[15])
			 , ((buf[16] << 8) + buf[17]), ((buf[18] << 8) + buf[19]),
			 ((buf[20] << 8) + buf[21])
			 , ((buf[22] << 8) + buf[23]), ((buf[24] << 8) + buf[25]),
			 ((buf[26] << 8) + buf[27]));
		return ret;
	}

	switch (gesture_id) {
	case GESTURE_DOUBLE_TAP:
		gesture->gesture_type = DOU_TAP;
		break;

	case GESTURE_UP_VEE:
		gesture->gesture_type = UP_VEE;
		break;

	case GESTURE_DOWN_VEE:
		gesture->gesture_type = DOWN_VEE;
		break;

	case GESTURE_LEFT_VEE:
		gesture->gesture_type = LEFT_VEE;
		break;

	case GESTURE_RIGHT_VEE:
		gesture->gesture_type = RIGHT_VEE;
		break;

	case GESTURE_O_CLOCKWISE:
		gesture->clockwise = 1;
		gesture->gesture_type = CIRCLE_GESTURE;
		break;

	case GESTURE_O_ANTICLOCK:
		gesture->clockwise = 0;
		gesture->gesture_type = CIRCLE_GESTURE;
		break;

	case GESTURE_DOUBLE_SWIP:
		gesture->gesture_type = DOU_SWIP;
		break;

	case GESTURE_LEFT2RIGHT_SWIP:
		gesture->gesture_type = LEFT2RIGHT_SWIP;
		break;

	case GESTURE_RIGHT2LEFT_SWIP:
		gesture->gesture_type = RIGHT2LEFT_SWIP;
		break;

	case GESTURE_UP2DOWN_SWIP:
		gesture->gesture_type = UP2DOWN_SWIP;
		break;

	case GESTURE_DOWN2UP_SWIP:
		gesture->gesture_type = DOWN2UP_SWIP;
		break;

	case GESTURE_M:
		gesture->gesture_type = M_GESTRUE;
		break;

	case GESTURE_W:
		gesture->gesture_type = W_GESTURE;
		break;

	case GESTURE_HEART_CLOCKWISE:
		gesture->clockwise = 1;
		gesture->gesture_type = HEART;
		break;

	case GESTURE_HEART_ANTICLOCK:
		gesture->clockwise = 0;
		gesture->gesture_type = HEART;
		break;

	case GESTURE_FINGER_PRINT:
		fts_read_fod_info(ts_data);
		TPD_INFO("FOD event type:0x%x", ts_data->fod_info.event_type);
		TPD_DEBUG("%s, fgerprint, touched = %d, fp_down = %d, fp_down_report = %d, \n",
		          __func__, ts_data->ts->view_area_touched, ts_data->fod_info.fp_down,
		          ts_data->fod_info.fp_down_report);

		if (ts_data->fod_info.event_type == FTS_EVENT_FOD) {
			if (ts_data->fod_info.fp_down && !ts_data->fod_info.fp_down_report) {
				gesture->gesture_type = FINGER_PRINTDOWN;
				ts_data->fod_info.fp_down_report = 1;

			} else if (!ts_data->fod_info.fp_down && ts_data->fod_info.fp_down_report) {
				gesture->gesture_type = FRINGER_PRINTUP;
				ts_data->fod_info.fp_down_report = 0;
			}

			gesture->Point_start.x = ts_data->fod_info.fp_x;
			gesture->Point_start.y = ts_data->fod_info.fp_y;
			gesture->Point_end.x = ts_data->fod_info.fp_area_rate;
			gesture->Point_end.y = 0;
		}

		break;
	case GESTURE_FINGER_PRINT_ERROR:
		fts_read_fod_error_info(ts_data);
		gesture->gesture_type = UNKOWN_GESTURE;
		break;
	case GESTURE_SINGLE_TAP:
		if ((!ts_data->differ_read_every_frame) || ts_data->gesture_flag == 0) {
			ts_data->gesture_flag = 1;
			gesture->gesture_type = SINGLE_TAP;
		} else {
			TPD_INFO("gesture_flag = 1, gesture irq ignore.\n");
			gesture->gesture_type = UNKOWN_GESTURE;
		}
		break;

	default:
		gesture->gesture_type = UNKOWN_GESTURE;
	}

	if (gesture_id != GESTURE_SINGLE_TAP) {
		ts_data->gesture_flag = 0;
	}

	if (gesture->gesture_type == SINGLE_TAP || gesture->gesture_type == DOU_TAP) {
		fts_read_aod_info(ts_data);
		gesture->Point_start.x = ts_data->aod_info.aod_x;
		gesture->Point_start.y = ts_data->aod_info.aod_y;
		TPD_INFO("AOD event type:0x%x", ts_data->aod_info.gesture_id);
	}

	if ((gesture->gesture_type != FINGER_PRINTDOWN)
	    && (gesture->gesture_type != FRINGER_PRINTUP)
	    && (gesture->gesture_type != UNKOWN_GESTURE)) {
		gesture->Point_start.x = (u16)((buf[4] << 8) + buf[5]);
		gesture->Point_start.y = (u16)((buf[6] << 8) + buf[7]);
		gesture->Point_end.x = (u16)((buf[8] << 8) + buf[9]);
		gesture->Point_end.y = (u16)((buf[10] << 8) + buf[11]);
		gesture->Point_1st.x = (u16)((buf[12] << 8) + buf[13]);
		gesture->Point_1st.y = (u16)((buf[14] << 8) + buf[15]);
		gesture->Point_2nd.x = (u16)((buf[16] << 8) + buf[17]);
		gesture->Point_2nd.y = (u16)((buf[18] << 8) + buf[19]);
		gesture->Point_3rd.x = (u16)((buf[20] << 8) + buf[21]);
		gesture->Point_3rd.y = (u16)((buf[22] << 8) + buf[23]);
		gesture->Point_4th.x = (u16)((buf[24] << 8) + buf[25]);
		gesture->Point_4th.y = (u16)((buf[26] << 8) + buf[27]);
	}

	if (ts_data->differ_read_every_frame && (ts_data->tp_differ_version == FTS_DIFFER_VERSION_V1)) {
		TPD_INFO("mutual diff data count:%u\n", ts_data->snr_count);
		fts_print_differ(ts_data->diff_buf, tx_num*rx_num, tx_num, rx_num);
		TPD_INFO("sc_water diff data:");
		fts_print_differ(ts_data->sc_water, tx_num+rx_num, 1, tx_num + rx_num);
		TPD_INFO("sc_normal diff data:");
		fts_print_differ(ts_data->sc_nomal, tx_num*rx_num, 1, tx_num + rx_num);
	} else if (ts_data->differ_read_every_frame && (ts_data->tp_differ_version == FTS_DIFFER_VERSION_V2)) {
		TPD_INFO("tp differ print start.\n");
		fts_print_differ_v2(ts_data, tx_num, rx_num, point_buff, edge_buff, event_num);
	}
	return 0;
}

static void fts_enable_fingerprint_underscreen(void *chip_data, uint32_t enable)
{
	int ret = 0;
	u8 val = 0xFF;
	u8 ucReadCF = 0;
	int i = 0;
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;


	TPD_INFO("%s:enable=%d", __func__, enable);
	ret = fts_read_reg(FTS_REG_FOD_EN, &val);

	if (ret < 0) {
		TPD_INFO("%s: read FOD enable(%x) fail", __func__, FTS_REG_FOD_EN);
		return;
	}

	TPD_DEBUG("%s, fgerprint, touched = %d, event_type = %d, fp_down = %d. fp_down_report = %d \n",
	          __func__, ts_data->ts->view_area_touched, ts_data->fod_info.event_type,
	          ts_data->fod_info.fp_down, ts_data->fod_info.fp_down_report);

	if (enable) {
		val |= 0x02;
		ts_data->fp_en = 1;

		if ((!ts_data->ts->view_area_touched)
		    && (ts_data->fod_info.event_type != FTS_EVENT_FOD)
		    && (!ts_data->fod_info.fp_down)
		    && (ts_data->fod_info.fp_down_report)) {   /* notouch, !38, 0, 1*/
			ts_data->fod_info.fp_down_report = 0;
			TPD_DEBUG("%s, fgerprint, fp_down_report status abnormal (notouch, 38!, 0, 1), needed to be reseted! \n",
			          __func__);
		}

	} else {
		val &= 0xFD;
		ts_data->fp_en = 0;
		ts_data->fod_info.fp_down = 0;
		ts_data->fod_info.event_type = 0;
		/*        ts_data->fod_info.fp_down_report = 0;*/
	}

	TPD_INFO("%s:write %x=%x.", __func__, FTS_REG_FOD_EN, val);
	ret = fts_write_reg(FTS_REG_FOD_EN, val);
	for (i = 0; i < 3; i++) {
		fts_read_reg(FTS_REG_FOD_EN, &ucReadCF);
		if (ucReadCF == val) {
			TPD_INFO("MODE_GESTURE, Read 0xCF=%d (write success)", ucReadCF);
			break;
		} else {
		    ret = fts_write_reg(FTS_REG_FOD_EN, val);
		}
		msleep(10);
	}
	if (i >= 3) {
		TPD_INFO("MODE_GESTURE, Write 0xCF failed after 3 retries!");
	}

	if (ret < 0) {
		TPD_INFO("%s: write FOD enable(%x=%x) fail", __func__, FTS_REG_FOD_EN, val);
	}
}

static void fts_screenon_fingerprint_info(void *chip_data,
        struct fp_underscreen_info *fp_tpinfo)
{
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;

	memset(fp_tpinfo, 0, sizeof(struct fp_underscreen_info));
	TPD_INFO("FOD event type:0x%x", ts_data->fod_info.event_type);

	if (ts_data->fod_info.fp_down) {
		fp_tpinfo->touch_state = FINGERPRINT_DOWN_DETECT;

	} else {
		fp_tpinfo->touch_state = FINGERPRINT_UP_DETECT;
	}

	fp_tpinfo->area_rate = ts_data->fod_info.fp_area_rate;
	fp_tpinfo->x = ts_data->fod_info.fp_x;
	fp_tpinfo->y = ts_data->fod_info.fp_y;

	TPD_INFO("FOD Info:touch_state:%d,area_rate:%d,x:%d,y:%d[fp_down:%d]",
	         fp_tpinfo->touch_state, fp_tpinfo->area_rate, fp_tpinfo->x,
	         fp_tpinfo->y, ts_data->fod_info.fp_down);

	if (ts_data->fingerprint_error_report_support && fp_tpinfo->touch_state == FINGERPRINT_UP_DETECT) {
		fts_read_fod_error_info(ts_data);
	}
}

static void fts_register_info_read(void *chip_data, uint16_t register_addr,
                                   uint8_t *result, uint8_t length)
{
	u8 addr = (u8)register_addr;

	fts_read(&addr, 1, result, length);
}

static void fts_set_touch_direction(void *chip_data, uint8_t dir)
{
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	ts_data->touch_direction = dir;
}

static uint8_t fts_get_touch_direction(void *chip_data)
{
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	return ts_data->touch_direction;
}

static int fts_smooth_lv_set(void *chip_data, int level)
{
	TPD_INFO("set smooth lv to %d", level);

	return fts_write_reg(FTS_REG_SMOOTH_LEVEL, level);
}

static int fts_send_temperature(void *chip_data, int temp, bool normal_mode)
{
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	int ret = 0;

	ts_data->tp_temperature = temp;
	TPD_INFO("%s:temperature:%d!\n", __func__, ts_data->tp_temperature);

	if (!!normal_mode) {
		ret = fts_write_reg(FTS_REG_TEMPERATURE, ts_data->tp_temperature&0xFF);
		if (ret < 0) {
			TPD_INFO("%s:fts send temperature fail", __func__);
		}
		TPD_INFO("%s:fts send temperature:%d suc!", __func__, ts_data->tp_temperature);
	}

	return 0;
}

static void fts_force_water_mode(void *chip_data, bool enable)
{
	int retval = 0;
	u8 regval = 0;

	TPD_INFO("%s: %s force water mode.\n", __func__, enable ? "Enter" : "Exit");

	retval = fts_read_reg(FTS_REG_FREQUENCE_WATER_MODE, &regval);
	if(retval < 0) {
		TPD_INFO("Failed to get water mode config\n");
		return;
	}

	if(enable) {
		regval = regval | 0x02;
	} else {
		regval = regval & 0xfd;
	}

	retval = fts_write_reg(FTS_REG_FREQUENCE_WATER_MODE, regval);
	if(retval < 0) {
		TPD_INFO("Failed to set water mode config\n");
		return;
	}

	retval = fts_read_reg(FTS_REG_FREQUENCE_WATER_MODE, &regval);
	if(retval < 0) {
		TPD_INFO("Failed to get water mode config\n");
		return;
	}
	TPD_INFO("%s: now reg_val=0x%x", __func__, regval);
}

static void fts_inject_wdt_reset(void *chip_data, int value)
{
	int retval = 0;
	u8 regval = 0;

	if (!chip_data) {
		TPD_INFO("%s: chip_data is NULL\n", __func__);
		return;
	}

	TPD_INFO("%s: %s inject watchdog reset.\n", __func__, value ? "Enter" : "Exit");

	retval = fts_read_reg(FTS_REG_INJECT_WDT_RESET, &regval);
	if(retval < 0) {
		TPD_INFO("Failed to get watchdog reset config\n");
		return;
	}

	if (value) {
		regval = regval | 0x03;
	} else {
		regval = regval & 0xfc;
	}

	retval = fts_write_reg(FTS_REG_INJECT_WDT_RESET, regval);
	if(retval < 0) {
		TPD_INFO("Failed to write inject watchdog reset\n");
		return;
	}

	retval = fts_read_reg(FTS_REG_INJECT_WDT_RESET, &regval);
	if(retval < 0) {
		TPD_INFO("Failed to read inject watchdog reset\n");
		return;
	}
	TPD_INFO("%s: now reg_val=0x%x", __func__, regval);
}

static void fts_freq_hop_trigger(void *chip_data)
{
	int retval = 0;
	u8 regval = 0;
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;

	TPD_INFO("%s : send cmd to tigger frequency hopping here!!!\n", __func__);

	retval = fts_read_reg(FTS_REG_FREQUENCE_WATER_MODE, &regval);
	if(retval < 0) {
		TPD_INFO("Failed to get frequency hopping mode config\n");
		return;
	}

	TPD_INFO("%s : Hop to frequency : %d\n", __func__, ts_data->freq_point);

	retval = fts_write_reg(FTS_REG_FREQUENCE_WATER_MODE, 0x01);
	if(retval < 0) {
		TPD_INFO("Failed to hop frequency\n");
	}
	ts_data->freq_point = 1;
	retval = fts_read_reg(FTS_REG_FREQUENCE_WATER_MODE, &regval);
	if(retval < 0) {
		TPD_INFO("Failed to get frequency hopping mode config\n");
		return;
	}
	TPD_INFO("%s: now reg_val=0x%x", __func__, regval);
}


static int fts_refresh_switch(void *chip_data, int fps)
{
	TPD_INFO("lcd fps =%d", fps);
	return fts_write_reg(FTS_REG_REPORT_RATE,
			     (fps == 60 ? FTS_120HZ_REPORT_RATE : FTS_180HZ_REPORT_RATE));
}


static int fts_sensitive_lv_set(void *chip_data, int level)
{
	int ret = 0;

	TPD_INFO("set sensitive lv to %d", level);

	ret = fts_write_reg(FTS_REG_STABLE_DISTANCE_AFTER_N, level);
	if (ret < 0) {
		TPD_INFO("write FTS_REG_STABLE_DISTANCE_AFTER_N fail");
		return ret;
	}

	ret = fts_write_reg(FTS_REG_STABLE_DISTANCE, level);
	if (ret < 0) {
		TPD_INFO("write FTS_REG_STABLE_DISTANCE fail");
		return ret;
	}

	return 0;
}

static int fts_click_sensitive_lv_set(void *chip_data, int level)
{
	int ret = 0;
	u8 regval = 0;

	TPD_INFO("%s:click_sensitive_lv_set to %d", __func__, level);

	ret = fts_write_reg(FTS_REG_CLICK_SENSITIVE, level);
	if (ret < 0) {
		TPD_INFO("write FTS_REG_CLICK_SENSITIVE fail");
		return ret;
	}

	ret = fts_read_reg(FTS_REG_CLICK_SENSITIVE, &regval);
	if(ret < 0) {
		TPD_INFO("Failed to get click_sensitive_lv config\n");
		return ret;
	}
	TPD_INFO("%s: now click_sensitive_lv_set =0x%x", __func__, regval);

	return ret;
}

static int fts_set_high_frame_rate(void *chip_data, int level, int time)
{
	int ret = 0;

	TPD_INFO("set high_frame_rate to %d, keep %ds", level, time);

	if (level) {
		ret = fts_write_reg(FTS_REG_HIGH_FRAME_TIME, time);
	} else {
		ret = fts_write_reg(FTS_REG_HIGH_FRAME_TIME, 0);
	}

	return ret;
}

static void fts_get_rawdata_snr(struct chip_data_ft3683g *ts_data)
{
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int raw_num = tx_num * rx_num;
	int sc_num = tx_num + rx_num;
	int j = 0;
	u8 *touch_buf = ts_data->snr_buf;
	u8 *data_8 = NULL;

	for (j = 0; j < 10; j = j + 1) {
		if (ts_data->snr_data_is_ready) {
			break;
		} else {
			msleep(2);
			TPD_INFO("%s:fts_get_rawdata_snr not ready", __func__);
		}
	}

	if (ts_data->tp_differ_version == FTS_DIFFER_VERSION_V1) {
		ts_data->snr_count = touch_buf[135];
		data_8 = &touch_buf[136];
	} else if (ts_data->tp_differ_version == FTS_DIFFER_VERSION_V2) {
		ts_data->snr_count = touch_buf[151];
		data_8 = &touch_buf[154];
	}

	if (!data_8) {
		TPD_INFO("%s: Invalid differ version or data_8 not initialized", __func__);
		return;
	}

	for (j = 0; j < raw_num; j = j + 1) {
		ts_data->diff_buf[j] = (int)(short)((data_8[0] << 8) + data_8[1]);
		data_8 += 2;
	}

	for (j = 0; j < sc_num; j = j + 1) {
		ts_data->sc_water[j] = (int)(short)((data_8[0] << 8) + data_8[1]);
		data_8 += 2;
	}

	if (ts_data->tp_differ_version == FTS_DIFFER_VERSION_V1) {
		if (ts_data->differ_mode == FTS_REG_WORK_MODE_SNR_MODE) {
			data_8 += 40;
		} else if (ts_data->differ_mode == FTS_REG_WORK_MODE_FINAL_DIFF_MODE) {
			data_8 += 8;
		}
	}

	for (j = 0; j < sc_num; j = j + 1) {
		ts_data->sc_nomal[j] = (int)(short)((data_8[0] << 8) + data_8[1]);
		data_8 += 2;
	}
	ts_data->snr_data_is_ready = 0;
}

static void fts_tp_data_record_write(void *chip_data, int count)
{
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	int ret = 0;

	TPD_INFO("%s fts_tp_data_record_write:%d \n", __func__, count);
	if (!ts_data->tp_data_record_support) {
		TPD_INFO("data record not support! \n");
		return;
	}

	if (count < 0) {
		TPD_INFO("%s:count is error %d", __func__, count);
		return;
	}

	if (count) {
		ts_data->differ_read_every_frame = 1;
		focal_get_differ_version(ts_data);
		ts_data->differ_mode = FTS_REG_WORK_MODE_FINAL_DIFF_MODE;
		ret = fts_write_reg(FTS_REG_WORK_MODE, FTS_REG_WORK_MODE_FINAL_DIFF_MODE);
		if (ret < 0) {
			TPD_INFO("%s:open fastdiff fail", __func__);
		}
		TPD_INFO("%s:open fianl diff mode suc", __func__);
	} else {
		ts_data->differ_read_every_frame = 0;
		ts_data->differ_mode = FTS_REG_WORK_MODE_NORMAL_MODE;
		ret = fts_write_reg(FTS_REG_WORK_MODE, FTS_REG_WORK_MODE_NORMAL_MODE);
		if (ret < 0) {
			TPD_INFO("%s:close fastdiff fail", __func__);
		}
		TPD_INFO("%s:close fastdiff suc", __func__);
	}
}

static void fts_delta_snr_read(struct seq_file *s, void *chip_data, uint32_t count)
{
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	struct touchpanel_data *ts = spi_get_drvdata(ts_data->ft_spi);
	int rx_num = ts_data->hw_res->rx_num;
	struct touchpanel_snr *snr = ts_data->ts->snr;
	int j = 0;
	u8 snr_count = 0;
	int ret = 0;
	u32 real_count = 0;
	int data_reay = 0;
	int diff_data = 0;
	u32 i = 0;
	u8 last_differ_mode = 0;
	u8 snr_test_mode = 0;

	if (!ts_data->snr_read_support) {
		seq_printf(s, "snr read not support! \n");
		return;
	}

	if (!snr[0].doing) {
		seq_printf(s, "snr doing zero! \n");
		return;
	}

	focal_get_differ_version(ts_data);
	if (ts_data->tp_differ_version == FTS_DIFFER_VERSION_V1) {
		snr_test_mode = FTS_REG_WORK_MODE_SNR_MODE;
	} else {
		snr_test_mode = FTS_REG_WORK_MODE_FINAL_DIFF_MODE;
	}

	ret = fts_write_reg(FTS_REG_WORK_MODE, snr_test_mode);
	if (ret < 0) {
			TPD_INFO("%s:open fastdiff fail", __func__);
			return;
	}
	last_differ_mode = ts_data->differ_mode;
	ts_data->differ_mode = snr_test_mode;
	TPD_INFO("%s:open fastdiff test success", __func__);
	mutex_unlock(&ts->mutex);
	if (ts->int_mode == BANNABLE) {
		enable_irq(ts->irq);
	}
	msleep(2000);     /* wait for tp ic frequency hopping */
	ts_data->snr_is_reading = 1;
	ts_data->snr_data_is_ready = 0;
	msleep(50);

	for (i = 0; i < count; i++) {
		for (j = 0; j < 10; j++) {
			msleep(5);
			fts_get_rawdata_snr(ts_data);
			if (snr_count != ts_data->snr_count && ts_data->snr_count != 255 \
				&& ts_data->diff_buf[rx_num * snr[0].channel_x + snr[0].channel_y] > 100) {
				snr_count = ts_data->snr_count;
				data_reay = 1;
				TPD_INFO("%s:snr_count update,snr_count:%u,diff_buf:%d.", __func__, ts_data->snr_count, \
						ts_data->diff_buf[rx_num * snr[0].channel_x + snr[0].channel_y]);
				break;
			} else {
				data_reay = 0;
				TPD_INFO("%s:snr_count is same or error,data not ready,snr_count:%u,diff_buf:%d.", __func__, ts_data->snr_count, \
						ts_data->diff_buf[rx_num * snr[0].channel_x + snr[0].channel_y]);
			}
		}

		if (data_reay == 1) {
			for (j = 0; j < 10; j++) {
				if (snr[j].point_status) {
					diff_data = ts_data->diff_buf[rx_num * snr[j].channel_x + snr[j].channel_y];
					if (i && (snr[j].max != 0 || snr[j].min != 0)) {
						snr[j].max = diff_data > snr[j].max ? diff_data : snr[j].max;
						snr[j].min = diff_data < snr[j].min ? diff_data : snr[j].min;
					} else {
						snr[j].max = diff_data;
						snr[j].min = diff_data;
					}
					snr[j].sum += diff_data;
					TPD_INFO("%s:snr%d report sum %d += %d. now max=%d, min=%d \n", __func__, j, snr[j].sum, diff_data, snr[j].max, snr[j].min);
				}
			}
		real_count++;
		} else {
			TPD_INFO("%s:error,get snr data 10 times fail, error count is:%u", __func__, i);
		}
		TPD_INFO("%s:test set count is %u,now test count is %u.", __func__, count, i);
	}
	msleep(10);
	ts_data->snr_is_reading = 0;
	mutex_lock(&ts->mutex);
	ret = fts_write_reg(FTS_REG_WORK_MODE, last_differ_mode);
	if (ret < 0) {
			TPD_INFO("%s:close fastdiff fail", __func__);
			return;
	}
	ts_data->differ_mode = last_differ_mode;
	TPD_INFO("%s:close fastdiff test success", __func__);
	mutex_unlock(&ts->mutex);
	msleep(30);
	if (ts->int_mode == BANNABLE) {
		disable_irq_nosync(ts->irq);
	}
	mutex_lock(&ts->mutex);

	TPD_INFO("%s:test set count = %u, real test count = %u", __func__, count, real_count);

	if (real_count != 0) {
		for (i = 0; i < 10; i++) {
			if (snr[i].point_status) {
				seq_printf(s, "%d|%d|", snr[i].channel_x, snr[i].channel_y);
				snr[i].noise = snr[i].max - snr[i].min;
				seq_printf(s, "%d|", snr[i].max);
				seq_printf(s, "%d|", snr[i].min);
				seq_printf(s, "%d|", snr[i].sum/real_count);
				seq_printf(s, "%d\n", snr[i].noise);
				TPD_INFO("snr%d-cover [%d %d] %d %d %d %d\n", i, snr[i].channel_x, snr[i].channel_y, snr[i].max, snr[i].min, snr[i].sum, snr[i].noise);
				SNR_RESET(snr[i]);
				TPD_INFO("snr%d-cover [%d %d] %d %d %d %d\n", i, snr[i].channel_x, snr[i].channel_y, snr[i].max, snr[i].min, snr[i].sum, snr[i].noise);
			}
		}
	} else {
		seq_printf(s, "real_count = %u\n", real_count);
		TPD_INFO("%s:get snr data error,real count is error,real_count = %u\n", __func__, real_count);
		SNR_RESET(snr[i]);
	}
}

static void fts_rate_white_list_ctrl(void *chip_data, int value)
{
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	int ret = 0;
	int regvalue = 0;

	TPD_INFO("fts_rate_white_list_ctrl to  value: %d", value);
	if (ts_data == NULL) {
		return;
	}

	if (ts_data->ts->is_suspended) {
		return;
	}

	if (ts_data->extreme_game_flag) {
		return;
	}

	switch(value) {
	case 120:
		regvalue = 12;
		break;
	case 180:
		regvalue = 18;
		break;
	case 240:
		regvalue = 24;
		break;
	case 360:
		regvalue = 36;
		break;
	default:
		TPD_INFO("%s: report rate = %d, not support\n", __func__, value);
		return;
	}

	ret = fts_write_reg(FTS_REG_REPORT_RATE, regvalue);
	if (ret < 0) {
		TPD_INFO("write FTS_REG_REPORT_RATE fail");
		return;
	}
}

static int fts_diaphragm_touch_lv_set(void *chip_data, int value)
{
	int ret = 0;
	int regvalue = 0;

	TPD_INFO("fts_diaphragm_touch_lv_set to %d", value);

	switch(value) {
	case SMART_DEFAULT_MODE:
		regvalue = 0;
		break;
	case SMART_FILM_MODE:
		regvalue = 1;
		break;
	case SMART_WATERPROOF_MODE:
		regvalue = 2;
		break;
	case SMART_FILM_WATERPROOF_MODE:
		regvalue = 3;
		break;
	default:
		TPD_INFO("%s: report rate = %d, not support\n", __func__, value);
		return 0;
	}
	ret = fts_write_reg(FTS_REG_SMART_TOUCH_MODE_EN, regvalue);
	if (ret < 0) {
		TPD_INFO("write FTS_REG_SMART_TOUCH_MODE_EN fail");
		return 0;
	}

	return 0;
}

static int ft3683g_parse_dts(struct chip_data_ft3683g *ts_data, struct spi_device *spi)
{
	struct device *dev;
	struct device_node *np;
	struct device_node *chip_np;
	int rc = 0;

	dev = &spi->dev;
	np = dev->of_node;

	ts_data->high_resolution_support = of_property_read_bool(np, "high_resolution_support");
	ts_data->high_resolution_support_x8 = of_property_read_bool(np, "high_resolution_support_x8");
	TPD_INFO("%s:high_resolution_support is:%d %d\n", __func__, ts_data->high_resolution_support,
	         ts_data->high_resolution_support_x8);
	chip_np = of_get_child_by_name(np, "FT3683G");

	if (!chip_np) {
		ts_data->switch_game_rate_support = 0;
	} else {
		ts_data->switch_game_rate_support = of_property_read_bool(chip_np, "switch_report_rate");
		TPD_INFO("%s:switch_report_rate is:%d\n", __func__, ts_data->switch_game_rate_support);
		rc = of_property_read_u32(chip_np, "extreme_game_report_rate", &ts_data->extreme_game_report_rate);
		if (rc < 0) {
			/*default :0 disable feature*/
			ts_data->extreme_game_report_rate = 0;
		}
		ts_data->extreme_game_flag = false;
		TPD_INFO("extreme_game_report_rate %d\n", ts_data->extreme_game_report_rate);
	}


	return 0;
}

int fts_set_spi_max_speed(unsigned int speed, char mode)
{
	int rc;
	struct spi_device *spi = g_fts_data->ft_spi;

	if (mode) {
		spi->max_speed_hz = speed;
	} else {
		spi->max_speed_hz = g_fts_data->spi_speed;
	}

	rc = spi_setup(spi);
	if (rc) {
		TPD_INFO("%s: spi setup fail\n", __func__);
		return rc;
	}
	return rc;
}

static void fts_get_water_mode(void *chip_data)
{
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	struct touchpanel_data *ts = spi_get_drvdata(ts_data->ft_spi);
	TPD_INFO("%s: water flag %d!\n", __func__, ts_data->water_mode);
	if (ts_data->water_mode == 1) {
		ts->water_mode = 1;
	}
	else {
		ts->water_mode = 0;
	}
}

static void fts_get_glove_mode(void *chip_data, int *enable, int *count)
{
	int retval = 0;
	u8 regval = 0;
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;

	if (!ts_data || !enable || !count) {
		TPD_INFO("Failed to get glove mode config, null pointer");
		return;
	}

	retval = fts_read_reg(FTS_REG_GLOVE_MODE_SWITCH, &regval);
	if(retval < 0) {
		TPD_INFO("Failed to get glove mode config\n");
		return;
	}

	*count = ts_data->glove_mode_enable_count;
	*enable = regval;
}

static void fts_edge_limit_switch_write(void *chip_data, int value)
{
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	int ret = 0;
	int regvalue = 0;

	TPD_INFO("fts_edge_limit_switch_write to  value: %d", value);
	if (ts_data == NULL) {
		return;
	}

	if (ts_data->ts->is_suspended) {
		return;
	}

	if((value < 0) || value > 1) {
		TPD_INFO("%s: fts_edge_limit_switch_write value error\n", __func__);
		return;
	}
	regvalue = value;
	ret = fts_write_reg(FTS_REG_EDGE_LIMIT_SWITCH, regvalue);
	if (ret < 0) {
		TPD_INFO("write FTS_REG_REPORT_RATE fail");
		return;
	}
}

static struct oplus_touchpanel_operations fts_ops = {
	.power_control              = fts_power_control,
	.get_vendor                 = fts_get_vendor,
	.get_chip_info              = fts_get_chip_info,
	.fw_check                   = fts_fw_check,
	.mode_switch                = fts_mode_switch,
	.reset                      = fts_reset,
	.reset_gpio_control         = fts_reset_gpio_control,
	.fw_update                  = fts_fw_update,
	.trigger_reason             = fts_u32_trigger_reason,
	.get_touch_points           = fts_get_touch_points,
	.health_report              = fts_health_report,
	.get_gesture_info           = fts_get_gesture_info,
	.ftm_process                = fts_ftm_process,
	.enable_fingerprint         = fts_enable_fingerprint_underscreen,
	.screenon_fingerprint_info  = fts_screenon_fingerprint_info,
	.register_info_read         = fts_register_info_read,
	.set_touch_direction        = fts_set_touch_direction,
	.get_touch_direction        = fts_get_touch_direction,
	.esd_handle                 = fts_esd_handle,
	.tp_refresh_switch          = fts_refresh_switch,
	.smooth_lv_set              = fts_smooth_lv_set,
	.sensitive_lv_set           = fts_sensitive_lv_set,
	.click_sensitive_lv_set     = fts_click_sensitive_lv_set,
	.enable_gesture_mask        = fts_enable_gesture_mask,
	.set_gesture_state          = fts_set_gesture_state,
	.send_temperature           = fts_send_temperature,
	.freq_hop_trigger           = fts_freq_hop_trigger,
	.force_water_mode           = fts_force_water_mode,
	.set_fp_error_report        = fts_set_fp_error_report,
	.set_high_frame_rate        = fts_set_high_frame_rate,
	.rate_white_list_ctrl       = fts_rate_white_list_ctrl,
	.edge_limit_switch_write    = fts_edge_limit_switch_write,
	.report_rate                = fts_report_rate,
	.diaphragm_touch_lv_set         = fts_diaphragm_touch_lv_set,
	.get_water_mode            = fts_get_water_mode,
	.get_glove_mode            = fts_get_glove_mode,
	.aiunit_game_info          = fts_aiunit_game_info,
	.set_idle_freq_mode        = fts_set_idle_freq_mode,
	.inject_wdt_reset           = fts_inject_wdt_reset,
};

static struct focal_auto_test_operations ft3683g_test_ops = {
	.auto_test_preoperation = ft3683g_auto_preoperation,
	.test1 = ft3683g_noise_autotest,
	.test2 = ft3683g_rawdata_autotest,
	.test3 = ft3683g_uniformity_autotest,
	.test4 = ft3683g_scap_cb_autotest,
	.test5 = ft3683g_scap_rawdata_autotest,
	.test6 = ft3683g_short_test,
	.test7 = ft3683g_panel_differ_test,
	.test8 = ft3683g_membist_test,
	.test9 = ft3683g_cal_test,
	.test12 = ft3683g_rst_autotest,
	.auto_test_endoperation = ft3683g_auto_endoperation,
};

static struct engineer_test_operations ft3683g_engineer_test_ops = {
	.auto_test              = focal_auto_test,
};

static struct debug_info_proc_operations fts_debug_info_proc_ops = {
	.delta_read        = fts_delta_read,
	/*  .key_trigger_delta_read = fts_key_trigger_delta_read,*/
	.baseline_read = fts_baseline_read,
	.main_register_read = fts_main_register_read,
	.self_delta_read   = fts_self_delta_read,
	.delta_snr_read    = fts_delta_snr_read,
	.tp_data_record_write    = fts_tp_data_record_write,
};

struct focal_debug_func focal_debug_ops = {
	.esd_check_enable       = focal_esd_check_enable,
	.get_esd_check_flag     = focal_get_esd_check_flag,
	.get_fw_version         = focal_get_fw_version,
	.dump_reg_sate          = focal_dump_reg_state,
};

static int fts_tp_probe(struct spi_device *spi)
{
	struct chip_data_ft3683g *ts_data = NULL;
	struct touchpanel_data *ts = NULL;
	int ret = -1;
	u64 time_counter = 0;

	TPD_INFO("%s  is called\n", __func__);

	reset_healthinfo_time_counter(&time_counter);

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
	spi->cs_setup.value = 1;
	spi->cs_setup.unit = 0;
	spi->cs_hold.value = 1;
	spi->cs_hold.unit = 0;
	spi->cs_inactive.value = 1;
	spi->cs_inactive.unit = 0;
#endif /* end of LINUX_VERSION_CODE*/
#endif

	ret = spi_setup(spi);
	if (ret) {
		TPD_INFO("spi setup fail");
		return ret;
	}

	/*step1:Alloc chip_info*/
	ts_data = kzalloc(sizeof(struct chip_data_ft3683g), GFP_KERNEL);

	if (ts_data == NULL) {
		TPD_INFO("ts_data kzalloc error\n");
		ret = -ENOMEM;
		return ret;
	}

	memset(ts_data, 0, sizeof(*ts_data));
	ts_data->last_waterlevel = 0xFF;
	ts_data->spi_speed = spi->max_speed_hz;
	g_fts_data = ts_data;

	ts_data->ts_workqueue = create_singlethread_workqueue("fts_wq");
	if (!ts_data->ts_workqueue) {
		TPD_INFO("create fts workqueue fail");
	}
	init_waitqueue_head(&ts_data->ts_waitqueue);

	ret = fts_bus_init(ts_data);
	if (ret < 0) {
		TPD_INFO("bus init error\n");
		goto ts_malloc_failed;
	}

	ts_data->touch_buf = (u8 *)kzalloc(FTS_MAX_TOUCH_BUF, GFP_KERNEL);
	if (!ts_data->touch_buf) {
		TPD_INFO("failed to alloc memory for touch buf");
		ret = -ENOMEM;
		goto err_bus_init;
	}
	ts_data->touch_size = FTS_MAX_POINTS_LENGTH;
	ts_data->touch_analysis_support = 0;
	ts_data->ta_flag = 0;
	ts_data->ta_size = 0;

	fts_point_report_check_init(ts_data);

	/*step2:Alloc common ts*/
	ts = common_touch_data_alloc();

	if (ts == NULL) {
		TPD_INFO("ts kzalloc error\n");
		ret = -ENOMEM;
		goto err_report_buf;
	}

	memset(ts, 0, sizeof(*ts));

	/*step3:binding client && dev for easy operate*/
	ts_data->ft_spi = spi;
	ts_data->hw_res = &ts->hw_res;
	ts_data->ts = ts;
	ts->debug_info_ops = &fts_debug_info_proc_ops;
	ts->s_client = spi;
	ts->irq = spi->irq;
	ts->dev = &spi->dev;
	ts->chip_data = ts_data;
	ts->bus_type = TP_BUS_SPI;
	spi_set_drvdata(spi, ts);

	/*step4:file_operations callback binding*/
	ts->ts_ops = &fts_ops;
	ts->engineer_ops = &ft3683g_engineer_test_ops;
	ts->com_test_data.chip_test_ops = &ft3683g_test_ops;

	ts->private_data = &focal_debug_ops;
	ft3683g_parse_dts(ts_data, spi);

	ts_data->monitor_data = &ts->monitor_data;
	ts_data->resolution_info = &ts->resolution_info;
	/*step5:register common touch*/
	ret = register_common_touch_device(ts);

	if (ret < 0) {
		goto err_register_driver;
	}

	ts_data->is_ic_sleep = false;
	ts_data->snr_read_support = ts->snr_read_support;
	ts_data->tp_data_record_support = ts->tp_data_record_support;
	ts_data->differ_read_every_frame = 0;
	ts_data->snr_is_reading = 0;
	ts_data->snr_data_is_ready = 0;
	ts_data->differ_mode = FTS_REG_WORK_MODE_NORMAL_MODE;
	focal_get_differ_version(ts_data);

	/*step6:create focal apk debug files*/
	fts_create_apk_debug_channel(ts_data);
	fts_create_sysfs(ts_data);

	/*step7:Chip Related function*/
	focal_create_sysfs_spi(spi);

	ts_data->black_gesture_indep = ts->black_gesture_indep_support;
	ts_data->fingerprint_error_report_support = ts->fingerprint_error_report_support;
		if (ts->health_monitor_support) {
		tp_healthinfo_report(&ts->monitor_data, HEALTH_PROBE, &time_counter);
	}
	ts_data->probe_done = 1;
	TPD_INFO("%s, probe normal end\n", __func__);

	return 0;

err_register_driver:
	common_touch_data_free(ts);
	ts = NULL;

err_report_buf:
	kfree(ts_data->touch_buf);
	ts_data->touch_buf = NULL;

err_bus_init:
	kfree(ts_data->bus_tx_buf);
	ts_data->bus_tx_buf = NULL;
	kfree(ts_data->bus_rx_buf);
	ts_data->bus_rx_buf = NULL;

ts_malloc_failed:

	kfree(ts_data);
	ts_data = NULL;

	TPD_INFO("%s, probe error\n", __func__);

	return ret;
}

static void fts_spi_tp_shutdown(struct spi_device *spi)
{
	struct touchpanel_data *ts = spi_get_drvdata(spi);

	tp_shutdown(ts);

	TPD_INFO("%s fts_spi_tp_shutdown is call.\n", __func__);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
static void fts_tp_remove(struct spi_device *spi)
#else
static int fts_tp_remove(struct spi_device *spi)
#endif
{
	struct touchpanel_data *ts = spi_get_drvdata(spi);
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)ts->chip_data;

	TPD_INFO("%s is called\n", __func__);
	fts_point_report_check_exit(ts_data);
	fts_release_apk_debug_channel(ts_data);
	fts_remove_sysfs(ts_data);
	fts_bus_exit(ts_data);
	kfree(ts_data->touch_buf);
	ts_data->touch_buf = NULL;

	kfree(ts_data);
	ts_data = NULL;

	if (ts) {
		unregister_common_touch_device(ts);
		common_touch_data_free(ts);
	}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
#else
	return 0;
#endif
}

static int fts_spi_suspend(struct device *dev)
{
	struct touchpanel_data *ts = dev_get_drvdata(dev);

	TPD_INFO("%s: is called\n", __func__);
	tp_pm_suspend(ts);


	return 0;
}

static int fts_spi_resume(struct device *dev)
{
	struct touchpanel_data *ts = dev_get_drvdata(dev);

	TPD_INFO("%s is called\n", __func__);
	tp_pm_resume(ts);


	return 0;
}

static const struct spi_device_id tp_id[] = {
	{ TPD_DEVICE, 0 },
	{ "oplus,tp_noflash", 0 },
	{ }
};

static struct of_device_id tp_match_table[] = {
	{ .compatible = TPD_DEVICE, },
	{ .compatible = "oplus,tp_noflash", },
	{ }
};

static const struct dev_pm_ops tp_pm_ops = {
	.suspend = fts_spi_suspend,
	.resume = fts_spi_resume,
};

static struct spi_driver fts_ts_driver = {
	.probe          = fts_tp_probe,
	.remove         = fts_tp_remove,
	.id_table   = tp_id,
	.shutdown	= fts_spi_tp_shutdown,
	.driver         = {
		.name   = TPD_DEVICE,
		.of_match_table =  tp_match_table,
		.pm = &tp_pm_ops,
	},
};

static int __init tp_driver_init_ft3683g(void)
{
	TPD_INFO("%s is called\n", __func__);

	if (!tp_judge_ic_match(TPD_DEVICE)) {
		return 0;
	}

	if (spi_register_driver(&fts_ts_driver) != 0) {
		TPD_INFO("unable to add spi driver.\n");
		return 0;
	}

	return 0;
}

/* should never be called */
static void __exit tp_driver_exit_ft3683g(void)
{
	spi_unregister_driver(&fts_ts_driver);
	return;
}
#ifdef CONFIG_TOUCHPANEL_LATE_INIT
late_initcall(tp_driver_init_ft3683g);
#else
module_init(tp_driver_init_ft3683g);
#endif
module_exit(tp_driver_exit_ft3683g);

MODULE_DESCRIPTION("Touchscreen Ft3683G Driver");
MODULE_LICENSE("GPL");
