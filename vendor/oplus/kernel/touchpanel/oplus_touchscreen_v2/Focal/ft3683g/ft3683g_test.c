// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/delay.h>
#include "ft3683g_core.h"

/*******Part0:LOG TAG Declear********************/

#ifdef TPD_DEVICE
#undef TPD_DEVICE
#define TPD_DEVICE "ft3683g-test"
#else
#define TPD_DEVICE "ft3683g-test"
#endif
#define TPD_INFO(a, arg...)  pr_err("[TP]"TPD_DEVICE ": " a, ##arg)


#define FTS_TEST_FUNC_ENTER() do { \
	TPD_INFO("[FTS_TS][TEST]%s: Enter\n", __func__); \
} while (0)

#define FTS_TEST_FUNC_EXIT()  do { \
	TPD_INFO("[FTS_TS][TEST]%s: Exit(%d)\n", __func__, __LINE__); \
} while (0)


#define FTS_TEST_SAVE_INFO(fmt, args...) do { \
	if (g_fts_data->s) { \
		seq_printf(g_fts_data->s, fmt, ##args); \
	} \
	TPD_INFO(fmt, ##args); \
} while (0)

#define FTS_TEST_SAVE_ERR(fmt, args...)  do { \
	if (g_fts_data->s) { \
		seq_printf(g_fts_data->s, fmt, ##args); \
	} \
	TPD_INFO(fmt, ##args); \
} while (0)

#define SHORT_MIN_CA                    600

#define  SCAP_CB_ON_GCB_MIN             0
#define  SCAP_CB_ON_GCB_MAX             125
#define  SCAP_CB_OFF_GCB_MIN            0
#define  SCAP_CB_OFF_GCB_MAX            125


enum wp_type {
	WATER_PROOF_OFF = 0,
	WATER_PROOF_ON = 1,
	HIGH_SENSITIVITY = 2,
	HOV = 3,
	WATER_PROOF_ON_TX = 100,
	WATER_PROOF_ON_RX,
	WATER_PROOF_OFF_TX,
	WATER_PROOF_OFF_RX,
};

enum byte_mode {
	DATA_ONE_BYTE,
	DATA_TWO_BYTE,
};

enum normalize_type {
	NORMALIZE_OVERALL,
	NORMALIZE_AUTO,
};


#define MAX_LENGTH_TEST_NAME            64


static u8 soc_membist_test_cmd[][11] = {\
	{0x70,  0x55,  0xAA}, \
	{0x70,  0x07,  0xf8,  0x80, 0x49,  0x00,  0x00,  0x00,  0x50,  0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x82, 0xc3,  0x00,  0x00}, \
	{0x70,  0x06,  0xf9,  0x82, 0xc4,  0x00,  0x00}, \
	{0x70,  0x07,  0xf8,  0x80, 0x04,  0x00,  0x00,  0xaa,  0x55,  0x00,  0x00}, \
	{0x70,  0x07,  0xf8,  0x80, 0x02,  0x00,  0x00,  0xFF,  0xFF,  0x00,  0x00}, \
	{0x70,  0x06,  0xf9,  0x80, 0x02,  0x00,  0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x02,  0x00,  0x00,  0xFF,  0x00,  0x00,  0x00}, \
	{0x70,  0x07,  0xf8,  0x80, 0x04,  0x00,  0x00,  0xaa,  0x55 , 0x00,  0x00}, \
	{0x70,  0x07,  0xf8,  0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x82, 0xc8, 0x00, 0x00, 0xc5, 0x6a, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x82, 0xc3, 0x00, 0x00, 0x8e}, \
	{0x70,  0x07,  0xf8,  0x82, 0xc8, 0x00, 0x00, 0xc5, 0x6a, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x82, 0xc0, 0x00, 0x00, 0x05}, \
	{0x70,  0x07,  0xf8,  0x80, 0x4e, 0x00, 0x00, 0xfa, 0xf3, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x80, 0x4b, 0x00, 0x00, 0xfa, 0xf1, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x80, 0x4e, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x80, 0x4f, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0x00, 0x8d, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0xaa, 0x55, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0x01, 0x8d, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0x3d, 0x2c, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0x46, 0x91, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0x34, 0x90, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0x08, 0x03, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0x54, 0x93, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0x34, 0x90, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0xF8, 0x03, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0x32, 0x90, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0x4f, 0x90, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0x50, 0x90, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0x51, 0x90, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0x49, 0x90, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0x4a, 0x90, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0x4b, 0x10, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x01, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0x4c, 0x10, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x01, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0x4d, 0x10, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x04, 0x00, 0x00}, \
	{0x70,  0x06,  0xf9,  0x81, 0x01, 0x00, 0x00}, \
	{0x70,  0x07,  0xf8,  0x81, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
	{0x70,  0x0A,  0xF5}, \
};

static int fts_test_spi_write_direct(u8 *writebuf, u32 writelen)
{
	int ret = 0;
	ret = fts_spi_write_direct(writebuf, writelen);

	return ret;
}

static int fts_test_spi_read_direct(u8 *writebuf, u32 writelen, u8 *readbuf, u32 readlen)
{
	int ret = 0;
	ret = fts_spi_read_direct(writebuf, writelen, readbuf, readlen);

	return ret;
}

static void sys_delay(int ms)
{
	msleep(ms);
}

static int focal_abs(int value)
{
	if (value < 0) {
		value = 0 - value;
	}

	return value;
}

static void print_buffer(int *buffer, int length, int line_num)
{
	int i = 0;
	int j = 0;
	int tmpline = 0;
	char *tmpbuf = NULL;
	int tmplen = 0;
	int cnt = 0;

	if ((NULL == buffer) || (length <= 0)) {
		TPD_INFO("buffer/length(%d) fail", length);
		return;
	}

	tmpline = line_num ? line_num : length;
	tmplen = tmpline * 6 + 128;
	tmpbuf = kzalloc(tmplen, GFP_KERNEL);

	if (!tmpbuf) {
		TPD_INFO("%s, alloc failed \n", __func__);
		return;
	}

	for (i = 0; i < length; i = i + tmpline) {
		cnt = 0;

		for (j = 0; j < tmpline; j++) {
			cnt += snprintf(tmpbuf + cnt, tmplen - cnt, "%5d ", buffer[i + j]);

			if ((cnt >= tmplen) || ((i + j + 1) >= length)) {
				break;
			}
		}

		TPD_DEBUG("%s", tmpbuf);
	}

	if (tmpbuf) {
		kfree(tmpbuf);
		tmpbuf = NULL;
	}
}

#define NODE_MATCH      1
#define CHANNEL_MATCH   2
#define CHEN_MATCH      3
#define ONE_MATCH       4
static int ft3683g_output_data(int *buffer, struct chip_data_ft3683g *ts_data,
                       struct auto_testdata *focal_testdata, int limit_type)
{
	uint8_t data_buf[64];
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int i = 0;
	int num_each_line = 0;
	int data_volumn = 0;

	if (limit_type == NODE_MATCH) {
		num_each_line = rx_num;
		data_volumn = rx_num * tx_num;

	} else if (limit_type == CHANNEL_MATCH) {
		num_each_line = rx_num + tx_num;
		data_volumn = (rx_num + tx_num) * 2;
	} else if (limit_type == CHEN_MATCH) {
		num_each_line = rx_num;
		data_volumn = rx_num + tx_num;
	} else if (limit_type == ONE_MATCH) {
		num_each_line = rx_num;
		data_volumn = 1;
	}

	memset(data_buf, 0, sizeof(data_buf));

	for (i = 0; i < data_volumn; i += 1) {
		snprintf(data_buf, 64, "%d,", buffer[i]);
		tp_test_write(focal_testdata->fp, focal_testdata->length, data_buf,
		              strlen(data_buf), focal_testdata->pos);

		if (!((i + 1) % num_each_line) || (i == data_volumn - 1)) {
			snprintf(data_buf, 64, "\n");
			tp_test_write(focal_testdata->fp, focal_testdata->length, data_buf,
			              strlen(data_buf), focal_testdata->pos);
		}
	}

	return 0;
}

/********************************************************************
 * test read/write interface
 *******************************************************************/
static int fts_test_bus_read(u8 *cmd, int cmdlen, u8 *data, int datalen)
{
	int ret = 0;

	ret = fts_read(cmd, cmdlen, data, datalen);
	if (ret < 0)
		return ret;
	else
		return 0;
}

static int fts_test_bus_write(u8 *writebuf, int writelen)
{
	int ret = 0;

	ret = fts_write(writebuf, writelen);
	if (ret < 0)
		return ret;
	else
		return 0;
}

static int fts_test_read_reg(u8 addr, u8 *val)
{
	return fts_test_bus_read(&addr, 1, val, 1);
}

static int fts_test_write_reg(u8 addr, u8 val)
{
	int ret;
	u8 cmd[2] = {0};

	cmd[0] = addr;
	cmd[1] = val;
	ret = fts_test_bus_write(cmd, 2);

	return ret;
}

static int fts_test_read(u8 addr, u8 *readbuf, int readlen)
{
	int ret = 0;
	int i = 0;
	int packet_length = 0;
	int packet_num = 0;
	int packet_remainder = 0;
	int offset = 0;
	int byte_num = readlen;

	packet_num = byte_num / BYTES_PER_TIME;
	packet_remainder = byte_num % BYTES_PER_TIME;

	if (packet_remainder) {
		packet_num++;
	}

	if (byte_num < BYTES_PER_TIME) {
		packet_length = byte_num;

	} else {
		packet_length = BYTES_PER_TIME;
	}

	/* FTS_TEST_DBG("packet num:%d, remainder:%d", packet_num, packet_remainder); */

	ret = fts_test_bus_read(&addr, 1, &readbuf[offset], packet_length);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read buffer fail\n");
		return ret;
	}

	for (i = 1; i < packet_num; i++) {
		offset += packet_length;

		if ((i == (packet_num - 1)) && packet_remainder) {
			packet_length = packet_remainder;
		}


		ret = fts_test_bus_read(&addr, 1, &readbuf[offset], packet_length);

		if (ret < 0) {
			FTS_TEST_SAVE_ERR("read buffer fail\n");
			return ret;
		}
	}

	return 0;
}

/*static int fts_test_spi_write_direct(u8 *writebuf, u32 writelen)
{
     int ret = 0;
	 ret = fts_spi_write_direct(writebuf,writelen);

	 return ret;
}

static int fts_test_spi_read_direct(u8 *writebuf, u32 writelen, u8 *readbuf, u32 readlen)
{
	int ret = 0;
	ret = fts_spi_read_direct(writebuf,writelen,readbuf,readlen);

	return ret;

}*/

/*
 * wait_state_update - wait fw status update
 */
static int wait_state_update(u8 retval)
{
	int ret = 0;
	int times = 0;
	u8 addr = 0;
	u8 state = 0xFF;

	addr = FACTORY_REG_PARAM_UPDATE_STATE_TOUCH;
	while (times++ < FACTORY_TEST_RETRY) {
		sys_delay(FACTORY_TEST_DELAY);
		/* Wait register status update */
		state = 0xFF;
		ret = fts_test_read_reg(addr, &state);
		if ((ret >= 0) && (retval == state))
			break;
		else
			TPD_INFO("reg%x=%x,retry:%d", addr, state, times);
	}

	if (times >= FACTORY_TEST_RETRY) {
		FTS_TEST_SAVE_ERR("Wait State Update fail,reg%x=%x\n", addr, state);
		return -EIO;
	}

	return 0;
}


/*
 * read_mass_data - read rawdata/short test data
 * addr - register addr which read data from
 * byte_num - read data length, unit:byte
 * buf - save data
 *
 * return 0 if read data succuss, otherwise return error code
 */
static int read_mass_data(u8 addr, int byte_num, int *buf)
{
	int ret = 0;
	int i = 0;
	u8 *data = NULL;

	data = (u8 *)kzalloc(byte_num * sizeof(u8), GFP_KERNEL);

	if (NULL == data) {
		FTS_TEST_SAVE_ERR("mass data buffer malloc fail\n");
		return -ENOMEM;
	}

	/* read rawdata buffer */
	TPD_INFO("mass data len:%d", byte_num);
	ret = fts_test_read(addr, data, byte_num);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read mass data fail\n");
		goto read_massdata_err;
	}

	for (i = 0; i < byte_num; i = i + 2) {
		buf[i >> 1] = (int)(short)((data[i] << 8) + data[i + 1]);
	}

	ret = 0;
read_massdata_err:
	kfree(data);
	return ret;
}


/********************************************************************
 * test global function enter work/factory mode
 *******************************************************************/
static int enter_work_mode(void)
{
	int ret = 0;
	u8 mode = 0;
	int i = 0;
	int j = 0;

	TPD_INFO("%s +\n", __func__);
	ret = fts_test_read_reg(DEVIDE_MODE_ADDR, &mode);

	if ((ret >= 0) && (0x00 == mode)) {
		return 0;
	}

	for (i = 0; i < ENTER_WORK_FACTORY_RETRIES; i++) {
		ret = fts_test_write_reg(DEVIDE_MODE_ADDR, 0x00);

		if (ret >= 0) {
			sys_delay(FACTORY_TEST_DELAY);

			for (j = 0; j < 20; j++) {
				ret = fts_test_read_reg(DEVIDE_MODE_ADDR, &mode);

				if ((ret >= 0) && (0x00 == mode)) {
					TPD_INFO("enter work mode success");
					return 0;

				} else {
					sys_delay(FACTORY_TEST_DELAY);
				}
			}
		}

		sys_delay(50);
	}

	if (i >= ENTER_WORK_FACTORY_RETRIES) {
		FTS_TEST_SAVE_ERR("Enter work mode fail\n");
		return -EIO;
	}

	TPD_INFO("%s -\n", __func__);
	return 0;
}


static int fts_special_operation_for_samsung(struct chip_data_ft3683g *ts_data)
{
	int ret = 0;

	if (true ==
	    ts_data->use_panelfactory_limit) {                      /*only for firmware released to samsung factory*/
		ret = fts_test_write_reg(FTS_REG_SAMSUNG_SPECIFAL, 0x01);

		if (ret < 0) {
			FTS_TEST_SAVE_ERR("write FTS_REG_SAMSUNG_SPECIFAL fail at %s,ret=%d\n", __func__, ret);
			return -EIO;
		}
	}

	return ret;
}


#define FTS_FACTORY_MODE 0x40
static int enter_factory_mode(struct chip_data_ft3683g *ts_data)
{
	int ret = 0;
	u8 mode = 0;
	int i = 0;
	int j = 0;

	ret = fts_test_read_reg(DEVIDE_MODE_ADDR, &mode);

	if ((ret >= 0) && (FTS_FACTORY_MODE == mode)) {
		fts_special_operation_for_samsung(ts_data);
		return 0;
	}

	for (i = 0; i < ENTER_WORK_FACTORY_RETRIES; i++) {
		ret = fts_test_write_reg(DEVIDE_MODE_ADDR, 0x40);

		if (ret >= 0) {
			sys_delay(FACTORY_TEST_DELAY);

			for (j = 0; j < 20; j++) {
				ret = fts_test_read_reg(DEVIDE_MODE_ADDR, &mode);

				if ((ret >= 0) && (FTS_FACTORY_MODE == mode)) {
					TPD_INFO("enter factory mode success");
					sys_delay(200);
					fts_special_operation_for_samsung(ts_data);
					return 0;

				} else {
					sys_delay(FACTORY_TEST_DELAY);
				}
			}
		}

		sys_delay(50);
	}

	if (i >= ENTER_WORK_FACTORY_RETRIES) {
		FTS_TEST_SAVE_ERR("Enter factory mode fail\n");
		return -EIO;
	}

	fts_special_operation_for_samsung(ts_data);
	return 0;
}

static int get_channel_num(struct chip_data_ft3683g *ts_data)
{
	int ret = 0;
	u8 tx_num = 0;
	u8 rx_num = 0;

	ret = fts_test_read_reg(FACTORY_REG_CHX_NUM, &tx_num);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read tx_num register fail\n");
		return ret;
	}

	ret = fts_test_read_reg(FACTORY_REG_CHY_NUM, &rx_num);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read rx_num register fail\n");
		return ret;
	}

	if ((tx_num != ts_data->hw_res->tx_num)
	    || (rx_num != ts_data->hw_res->rx_num)) {
		FTS_TEST_SAVE_ERR("channel num check fail, tx_num:%d-%d, rx_num:%d-%d\n",
		                  tx_num, ts_data->hw_res->tx_num,
		                  rx_num, ts_data->hw_res->rx_num);
		return -EIO;
	}

	return 0;
}

static int read_rawdata(u8 off_addr, u8 off_val, u8 rawdata_addr, int byte_num,
                        int *data)
{
	int ret = 0;

	/* set line addr or rawdata start addr */
	ret = fts_test_write_reg(off_addr, off_val);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("wirte line/start addr fail\n");
		return ret;
	}

	ret = read_mass_data(rawdata_addr, byte_num, data);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read rawdata fail\n");
		return ret;
	}

	return 0;
}

static int start_scan(void)
{
	int ret = 0;
	u8 addr = 0;
	u8 val = 0;
	u8 finish_val = 0;
	int times = 0;

	addr = DEVIDE_MODE_ADDR;
	val = 0xC0;
	finish_val = 0x40;

	/* write register to start scan */
	ret = fts_test_write_reg(addr, val);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("write start scan mode fail\n");
		return ret;
	}

	/* Wait for the scan to complete */
	while (times++ < FACTORY_TEST_RETRY) {
		sys_delay(FACTORY_TEST_DELAY);

		ret = fts_test_read_reg(addr, &val);

		if ((ret >= 0) && (val == finish_val)) {
			break;

		} else {
			TPD_INFO("reg%x=%x,retry:%d", addr, val, times);
		}
	}

	if (times >= FACTORY_TEST_RETRY) {
		FTS_TEST_SAVE_ERR("scan timeout\n");
		return -EIO;
	}

	return 0;
}

/*
 * start_scan - start to scan a frame
 */
static int ft3683g_start_scan(int frame_num)
{
	int ret = 0;
	u8 addr = 0;
	u8 val = 0;
	u8 finish_val = 0;
	int times = 0;

	addr = DEVIDE_MODE_ADDR;
	val = 0xC0;
	finish_val = 0x40;

	/* write register to start scan */
	ret = fts_test_write_reg(addr, val);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("write start scan mode fail\n");
		return ret;
	}

	sys_delay(frame_num * FACTORY_TEST_DELAY / 2);
	/* Wait for the scan to complete */
	while (times++ < 100) {
		sys_delay(FACTORY_TEST_DELAY);

		ret = fts_test_read_reg(addr, &val);
		if ((ret >= 0) && (val == finish_val)) {
			break;
		} else
			TPD_INFO("reg%x=%x,retry:%d", addr, val, times);
	}

	if (times >= 100) {
		FTS_TEST_SAVE_ERR("scan timeout\n");
		return -EIO;
	}

	return 0;
}

static bool get_fw_wp(u8 wp_ch_sel, enum wp_type water_proof_type)
{
	bool fw_wp_state = false;

	switch (water_proof_type) {
	case WATER_PROOF_ON:
		/* bit5: 0-check in wp on, 1-not check */
		fw_wp_state = !(wp_ch_sel & 0x20);
		break;

	case WATER_PROOF_ON_TX:
		/* Bit6:  0-check Rx+Tx in wp mode  1-check one channel
		   Bit2:  0-check Tx in wp mode;  1-check Rx in wp mode
		*/
		fw_wp_state = (!(wp_ch_sel & 0x40) || !(wp_ch_sel & 0x04));
		break;

	case WATER_PROOF_ON_RX:
		fw_wp_state = (!(wp_ch_sel & 0x40) || (wp_ch_sel & 0x04));
		break;

	case WATER_PROOF_OFF:
		/* bit7: 0-check in wp off, 1-not check */
		fw_wp_state = !(wp_ch_sel & 0x80);
		break;

	case WATER_PROOF_OFF_TX:
		/* Bit1-0:  00-check Tx in non-wp mode
		            01-check Rx in non-wp mode
		            10:check Rx+Tx in non-wp mode
		*/
		fw_wp_state = ((0x0 == (wp_ch_sel & 0x03)) || (0x02 == (wp_ch_sel & 0x03)));
		break;

	case WATER_PROOF_OFF_RX:
		fw_wp_state = ((0x01 == (wp_ch_sel & 0x03)) || (0x02 == (wp_ch_sel & 0x03)));
		break;

	default:
		break;
	}

	return fw_wp_state;
}

static int get_cb_ft5662(int *cb_buf, int byte_num, bool is_cf)
{
	int ret = 0;
	int i = 0;
	u8 cb[SC_NUM_MAX] = { 0 };

	if (byte_num > SC_NUM_MAX) {
		FTS_TEST_SAVE_ERR("CB byte(%d)>max(%d)", byte_num, SC_NUM_MAX);
		return -EINVAL;
	}

	ret = fts_test_write_reg(FACTORY_REG_MC_SC_CB_H_ADDR_OFF, 0);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("write cb_h addr offset fail\n");
		return ret;
	}


	ret = fts_test_write_reg(FACTORY_REG_MC_SC_CB_ADDR_OFF, 0);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("write cb addr offset fail\n");
		return ret;
	}

	ret = fts_test_read(FACTORY_REG_MC_SC_CB_ADDR, cb, byte_num);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read cb fail\n");
		return ret;
	}

	for (i = 0; i < byte_num; i++) {
		if (is_cf)
			cb_buf[i] = (cb[i] & 0x80) ? -((cb[i] & 0x3F)) : (cb[i] & 0x3F);
		else
			cb_buf[i] = (cb[i] & 0x80) ? -((cb[i] & 0x7F)) : (cb[i] & 0x7F);
	}

	return 0;
}

static int get_rawdata_mc_sc(enum wp_type wp, int byte_num, int *data)
{
	int ret = 0;
	u8 val = 0;
	u8 addr = 0;
	u8 rawdata_addr = 0;

	addr = FACTORY_REG_LINE_ADDR;
	rawdata_addr = FACTORY_REG_RAWDATA_ADDR_MC_SC;

	if (WATER_PROOF_ON == wp) {
		val = 0xAC;

	} else if (WATER_PROOF_OFF == wp) {
		val = 0xAB;
	} else if (HIGH_SENSITIVITY == wp) {
		val = 0xA0;
	} else if (HOV == wp) {
		val = 0xA1;
		byte_num = 4 * 2;
	}

	ret = read_rawdata(addr, val, rawdata_addr, byte_num, data);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read rawdata fail\n");
		return ret;
	}

	return 0;
}

static bool compare_mc_sc(struct chip_data_ft3683g *ts_data, bool tx_check,
                          bool rx_check, int *data, int *min, int *max)
{
	int i = 0;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int channel_num = tx_num + rx_num;
	bool result = true;

	if (rx_check) {
		for (i = 0; i < rx_num; i++) {
			if (0 == ts_data->node_valid_sc[i]) {
				continue;
			}

			if ((data[i] < min[i]) || (data[i] > max[i])) {
				TPD_INFO("rx check ERR [%d]: [%d] > [%d] > [%d] \n", i, max[i], data[i],
				         min[i]);
				FTS_TEST_SAVE_ERR("test fail,rx%d=%5d,range=(%5d,%5d)\n",
				                  i + 1, data[i], min[i], max[i]);
				result = false;
			}
		}
	}

	if (tx_check) {
		for (i = rx_num; i < channel_num; i++) {
			if (0 == ts_data->node_valid_sc[i]) {
				continue;
			}

			if ((data[i] < min[i]) || (data[i] > max[i])) {
				TPD_INFO("tx check ERR [%d]: [%d] > [%d] > [%d] \n", i, max[i], data[i],
				         min[i]);
				FTS_TEST_SAVE_INFO("test fail,tx%d=%5d,range=(%5d,%5d)\n",
				                   i - rx_num + 1, data[i], min[i], max[i]);
				result = false;
			}
		}
	}

	return result;
}

static int short_get_adc_data_mc(u8 retval, int byte_num, int *adc_buf, u8 mode)
{
	int ret = 0;
	int i = 0;
	u8 short_state = 0;

	FTS_TEST_FUNC_ENTER();
	/* select short test mode & start test */
	ret = fts_test_write_reg(FACTROY_REG_SHORT2_TEST_EN, mode);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("write short test mode fail\n");
		goto test_err;
	}

	for (i = 0; i < FACTORY_TEST_RETRY; i++) {
		sys_delay(FACTORY_TEST_RETRY_DELAY);

		ret = fts_test_read_reg(FACTROY_REG_SHORT2_TEST_STATE, &short_state);

		if ((ret >= 0) && (retval == short_state)) {
			break;
		} else
			TPD_DEBUG("reg%x=%x,retry:%d",
			          FACTROY_REG_SHORT2_TEST_STATE, short_state, i);
	}

	if (i >= FACTORY_TEST_RETRY) {
		FTS_TEST_SAVE_ERR("short test timeout, ADC data not OK\n");
		ret = -EIO;
		goto test_err;
	}

	ret = read_mass_data(FACTORY_REG_SHORT2_ADDR_MC, byte_num, adc_buf);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("get short(adc) data fail\n");
	}

	/*    TPD_DEBUG("adc data:\n");*/
	/*    print_buffer(adc_buf, byte_num / 2, 0);*/
test_err:
	FTS_TEST_FUNC_EXIT();
	return ret;
}

static int short_test_ch_to_all(struct chip_data_ft3683g *ts_data,
                                int *res, u8 *ab_ch, bool *result)
{
	int ret = 0;
	int i = 0;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int min_ca = SHORT_MIN_CA;
	int ch_num = tx_num + rx_num;
	int byte_num = 0;
	u8 ab_ch_num = 0;
	int temp = 0;

	TPD_INFO("short test:channel to all other\n");
	/*get resistance data*/
	byte_num = ch_num * 2;
	ret = short_get_adc_data_mc(TEST_RETVAL_AA, byte_num, &res[0], \
	                            FACTROY_REG_SHORT2_CA);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("get weak short data fail,ret:%d\n", ret);
		return ret;
	}

	*result = true;

	for (i = 0; i < ch_num; i++) {
        temp = res[i];
        if ((212 - ((temp * 250 / 2047) + 40)) == 0) {
			res[i] = 50000;
			continue;
        }
        res[i] = focal_abs(((temp * 25 / 2047 + 4) * 2005) / (212 - ((temp * 250 / 2047) + 40)));
        if (res[i] < min_ca) {
			ab_ch_num++;
			ab_ch[ab_ch_num] = i;
			*result = false;
        }
	}

	if (ab_ch_num) {
		print_buffer(res, ch_num, ch_num);
		ab_ch[0] = ab_ch_num;
		printk("[FTS_TS]ab_ch:");
		for (i = 0; i < ab_ch_num + 1; i++) {
			printk("%2d ", ab_ch[i]);
		}
		printk("\n");
	}

	return 0;
}

static void fts_show_null_noise(int *null_noise, int rx_num, int fre_num)
{
	int i = 0;
	int cnt = 0;
	char tmpbuf[512] = { 0 };

	/*show noise*/
	TPD_INFO("null noise:%d", null_noise[0]);
	for (i = 0; i < (rx_num * fre_num); i = i + 1) {
		cnt += snprintf(tmpbuf + cnt, 512 - cnt, "%5d,", null_noise[i + 1]);
		if (((i + 1) % rx_num) == 0) {
			cnt = 0;
			TPD_INFO("%s", tmpbuf);
		}
	}
}

static void ft3683g_get_null_noise(struct chip_data_ft3683g *ts_data,
                                  struct auto_testdata *focal_testdata)
{
	int ret = 0;
	uint8_t data_buf[64];
	int *null_noise;
	int null_byte_num = 0;

	null_byte_num = ts_data->hw_res->rx_num * ts_data->fre_num + 1;
	TPD_INFO("null noise num:%d", null_byte_num);
	null_noise = kzalloc(null_byte_num * sizeof(int), GFP_KERNEL);
	if (!null_noise) {
		TPD_INFO("null_noise malloc fail");
		return;
	}

	null_byte_num = null_byte_num * 2;
	ret = read_rawdata(FACTORY_REG_LINE_ADDR, 0xB0, 0xCE, null_byte_num,
	                   null_noise);
	if (ret < 0) {
		TPD_INFO("read null noise fail\n");
	} else {
		snprintf(data_buf, 64, "%d,", null_noise[0]);
		ft3683g_output_data(&null_noise[0], ts_data, focal_testdata, ONE_MATCH);
		fts_show_null_noise(&null_noise[0], ts_data->hw_res->rx_num, ts_data->fre_num);
	}

	kfree(null_noise);
	null_noise = NULL;
}

static void ft3683g_autotest_populate_result_head(
	struct chip_data_ft3683g *ts_data, struct auto_testdata *p_testdata)
{
	uint8_t  data_buf[256];
	uint32_t buflen = 0;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int line_num = 0;

	FTS_TEST_FUNC_ENTER();

	/*header*/
	buflen = snprintf(data_buf, 256, "ECC, 85, 170, IC Name, %s, IC Code, %x\n",
	                  "FT3683G", 0);
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	buflen = snprintf(data_buf, 256, "TestItem Num, %d, ", 12);
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	line_num = 11;
	buflen = snprintf(data_buf, 256, "%s, %d, %d, %d, %d, %d, ", "Noise Test", 14,
	                  tx_num, rx_num, line_num, 1);
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	line_num += tx_num;
	buflen = snprintf(data_buf, 256, "%s, %d, %d, %d, %d, %d, ", "Null Noise", 41,
	                  1, 1, line_num, 1);
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	line_num += 1;
	buflen = snprintf(data_buf, 256, "%s, %d, %d, %d, %d, %d, ", "Rawdata Test", 7,
	                  tx_num, rx_num, line_num, 2);
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	line_num += tx_num;
	buflen = snprintf(data_buf, 256, "%s, %d, %d, %d, %d, %d, ",
	                  "Rawdata Uniformity Test", 16, tx_num, rx_num, line_num, 1);
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	line_num += tx_num;
	buflen = snprintf(data_buf, 256, "%s, %d, %d, %d, %d, %d, ",
	                  "Rawdata Uniformity Test", 16, tx_num, rx_num, line_num, 2);
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	line_num += tx_num;
	buflen = snprintf(data_buf, 256, "%s, %d, %d, %d, %d, %d, ", "SCAP CB Test", 9,
	                  2, rx_num, line_num, 1);
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	line_num += 2;
	buflen = snprintf(data_buf, 256, "%s, %d, %d, %d, %d, %d, ", "SCAP CB Test", 9,
	                  2, 1, line_num, 2);
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	line_num += 2;
	buflen = snprintf(data_buf, 256, "%s, %d, %d, %d, %d, %d, ", "SCAP CB Test", 9,
	                  2, rx_num, line_num, 3);
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	line_num += 2;
	buflen = snprintf(data_buf, 256, "%s, %d, %d, %d, %d, %d, ", "SCAP CB Test", 9,
	                  2, 1, line_num, 4);
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	line_num += 2;
	buflen = snprintf(data_buf, 256, "%s, %d, %d, %d, %d, %d, ",
	                  "SCAP Rawdata Test", 10, 2, rx_num, line_num, 1);
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	line_num += 2;
	buflen = snprintf(data_buf, 256, "%s, %d, %d, %d, %d, %d, ",
	                  "SCAP Rawdata Test", 10, 2, rx_num, line_num, 2);
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	line_num += 2;
	buflen = snprintf(data_buf, 256, "%s, %d, %d, %d, %d, %d, ", "Panel Differ Test", 20,
	                  tx_num, rx_num, line_num, 1);
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	buflen = snprintf(data_buf, 256, "\n\n\n\n\n\n\n\n\n");
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	FTS_TEST_FUNC_EXIT();
	return;
}


#define NUM_MODE 2
#define TEST_RESULT_NORMAL     0
#define TEST_RESULT_ABNORMAL  -1
int ft3683g_auto_preoperation(struct seq_file *s, void *chip_data,
                             struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info)
{
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	int node_num = ts_data->hw_res->tx_num * ts_data->hw_res->rx_num;
	int channel_num = ts_data->hw_res->tx_num + ts_data->hw_res->rx_num;


	ts_data->node_valid = (int *)kzalloc(node_num * sizeof(int), GFP_KERNEL);
	if (!ts_data->node_valid) {
		FTS_TEST_SAVE_ERR("kzalloc for node_valid fail\n");
		goto alloc_err;
	}

	ts_data->node_valid_sc = (int *)kzalloc(channel_num * sizeof(int), GFP_KERNEL);
	if (!ts_data->node_valid_sc) {
		FTS_TEST_SAVE_ERR("kzalloc for node_valid_sc fail\n");
		goto alloc_err;
	}

	ts_data->noise_rawdata = (int *)kzalloc(node_num * sizeof(int), GFP_KERNEL);

	if (!ts_data->noise_rawdata) {
		FTS_TEST_SAVE_ERR("kzalloc for noise_rawdata fail\n");
		goto alloc_err;
	}

	ts_data->rawdata = (int *)kzalloc(node_num * sizeof(int), GFP_KERNEL);

	if (!ts_data->rawdata) {
		FTS_TEST_SAVE_ERR("kzalloc for rawdata fail\n");
		goto alloc_err;
	}

	ts_data->panel_differ = (int *)kzalloc(node_num * sizeof(int), GFP_KERNEL);

	if (!ts_data->panel_differ) {
		FTS_TEST_SAVE_ERR("kzalloc for panel_differ fail\n");
		goto alloc_err;
	}

	ts_data->scap_cb = (int *)kzalloc(channel_num * NUM_MODE * sizeof(int),
	                                  GFP_KERNEL);

	if (!ts_data->scap_cb) {
		FTS_TEST_SAVE_ERR("kzalloc for scap_cb fail\n");
		goto alloc_err;
	}

	ts_data->scap_rawdata = (int *)kzalloc(channel_num * NUM_MODE * sizeof(int),
	                                       GFP_KERNEL);

	if (!ts_data->scap_rawdata) {
		FTS_TEST_SAVE_ERR("kzalloc for scap_rawdata fail\n");
		goto alloc_err;
	}

	ts_data->rawdata_linearity = kzalloc(node_num * 2 * sizeof(int), GFP_KERNEL);

	if (!ts_data->rawdata_linearity) {
		FTS_TEST_SAVE_ERR("ts_data->rawdata_linearity buffer malloc fail\n");
		goto alloc_err;
	}

	ft3683g_autotest_populate_result_head(ts_data, focal_testdata);
	fts_test_entry(ts_data, focal_testdata);

	return TEST_RESULT_NORMAL;

alloc_err:

	if (ts_data->node_valid) {
		kfree(ts_data->node_valid);
		ts_data->node_valid = NULL;
	}

	if (ts_data->node_valid_sc) {
		kfree(ts_data->node_valid_sc);
		ts_data->node_valid_sc = NULL;
	}

	if (ts_data->rawdata_linearity) {
		kfree(ts_data->rawdata_linearity);
		ts_data->rawdata_linearity = NULL;
	}

	if (ts_data->scap_rawdata) {
		kfree(ts_data->scap_rawdata);
		ts_data->scap_rawdata = NULL;
	}

	if (ts_data->scap_cb) {
		kfree(ts_data->scap_cb);
		ts_data->scap_cb = NULL;
	}

	if (ts_data->rawdata) {
		kfree(ts_data->rawdata);
		ts_data->rawdata = NULL;
	}

	if (ts_data->panel_differ) {
		kfree(ts_data->panel_differ);
		ts_data->panel_differ = NULL;
	}

	if (ts_data->noise_rawdata) {
		kfree(ts_data->noise_rawdata);
		ts_data->noise_rawdata = NULL;
	}

	return TEST_RESULT_ABNORMAL;
}


static int raw_cap_data_restriction(int val, int raw_cap_restriction)
{
	return val * raw_cap_restriction / 100;
}

int ft3683g_rawdata_autotest(struct seq_file *s, void *chip_data,
                            struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info)
{
	int ret = 0;
	int i = 0;
	u8 fre = 0;
	u8 reg06_val = 0;
	u8 rawdata_addr = 0;
	bool result = false;
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	int byte_num = 0;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int node_num = tx_num * rx_num;
	int rawdata = 0;

	FTS_TEST_FUNC_ENTER();
	FTS_TEST_SAVE_INFO("\n============ Test Item: Rawdata Test\n");

	if (!ts_data->fts_autotest_offset->fts_raw_data_P
	    || !ts_data->fts_autotest_offset->fts_raw_data_N) {
		TPD_INFO("fts_raw_data_P || fts_raw_data_N is NULL");
		return 0;
	}

	if (!ts_data->rawdata || !ts_data->node_valid) {
		FTS_TEST_SAVE_ERR("rawdata is null\n");
		ret = -EINVAL;
		goto test_err;
	}

	ret = enter_factory_mode(ts_data);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("failed to enter factory mode,ret=%d\n", ret);
		goto test_err;
	}

	/* save origin value */
	ret = fts_test_read_reg(FACTORY_REG_FRE_LIST, &fre);

	if (ret) {
		FTS_TEST_SAVE_ERR("read 0x0A fail,ret=%d\n", ret);
		goto test_err;
	}

	ret = fts_test_read_reg(FACTORY_REG_DATA_SELECT, &reg06_val);

	if (ret) {
		FTS_TEST_SAVE_ERR("read 0x06 error,ret=%d\n", ret);
		goto test_err;
	}

	/* set frequecy high */
	ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, 0x81);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("set frequecy fail,ret=%d\n", ret);
		goto restore_reg;
	}

	/* wait fw state update */
	ret = wait_state_update(TEST_RETVAL_AA);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("wait state update fail\n");
		goto restore_reg;
	}

	ret = fts_test_write_reg(0x06, 0);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("set 0x06 fail,ret=%d\n", ret);
		goto restore_reg;
	}

	/*********************GET RAWDATA*********************/
	for (i = 0; i < 3; i++) {
		/* lost 3 frames, in order to obtain stable data */
		/* start scanning */
		ret = start_scan();

		if (ret < 0) {
			FTS_TEST_SAVE_ERR("scan fail\n");
			continue;
		}

		/* read rawdata */
		rawdata_addr = FACTORY_REG_RAWDATA_ADDR_MC_SC;
		byte_num = node_num * 2;
		ret = read_rawdata(FACTORY_REG_LINE_ADDR, 0xAA, rawdata_addr, byte_num,
		                   ts_data->rawdata);

		if (ret < 0) {
			FTS_TEST_SAVE_ERR("read rawdata fail\n");
		}
	}

	if (ret < 0) {
		result = false;
		goto restore_reg;
	}

	ft3683g_output_data(ts_data->rawdata, ts_data, focal_testdata, NODE_MATCH);


	/* compare */
	result = true;

	for (i = 0; i < node_num; i++) {
		if (0 == ts_data->node_valid[i]) {
			continue;
		}
		rawdata = ts_data->rawdata[i];
		rawdata = raw_cap_data_restriction(rawdata, focal_testdata->raw_cap_restriction);
		if ((rawdata < ts_data->fts_autotest_offset->fts_raw_data_N[i])
		    || (rawdata > ts_data->fts_autotest_offset->fts_raw_data_P[i])) {
			TPD_INFO("raw data ERR [%d] restriction[%d]: [%d] > [%d] > [%d] \n", i,
			         ts_data->fts_autotest_offset->fts_raw_data_P[i], ts_data->rawdata[i], rawdata,
			         ts_data->fts_autotest_offset->fts_raw_data_N[i]);
			FTS_TEST_SAVE_ERR("test fail,node(%4d,%4d)=%5d,restriction[%d], range=(%5d,%5d)\n",
			                  i / rx_num + 1, i % rx_num + 1, ts_data->rawdata[i], rawdata,
			                  ts_data->fts_autotest_offset->fts_raw_data_N[i],
			                  ts_data->fts_autotest_offset->fts_raw_data_P[i]);
			result = false;
		}
	}

restore_reg:
	/* set the origin value */
	ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, fre);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("restore 0x0A fail,ret=%d\n", ret);
	}

	/* wait fw state update */
	ret = wait_state_update(TEST_RETVAL_AA);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("wait state update fail\n");
	}

	ret = fts_test_write_reg(FACTORY_REG_DATA_SELECT, reg06_val);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("restore 0x06 fail,ret=%d\n", ret);
	}

test_err:

	if (result) {
		FTS_TEST_SAVE_INFO("------ rawdata test PASS\n");
		ret = TEST_RESULT_NORMAL;
	} else {
		FTS_TEST_SAVE_INFO("------ rawdata test NG\n");
		ret = TEST_RESULT_ABNORMAL;
	}

	FTS_TEST_FUNC_EXIT();
	return ret;
}


int ft3683g_uniformity_autotest(struct seq_file *s, void *chip_data,
                               struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info)
{
	int ret = 0;
	int row = 0;
	int col = 1;
	int i = 0;
	int deviation = 0;
	int max = 0;
	int *rl_tmp = NULL;
	int offset = 0;
	int offset2 = 0;
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int node_num = tx_num * rx_num;
	bool result = false;
	bool result2 = false;

	FTS_TEST_FUNC_ENTER();
	FTS_TEST_SAVE_INFO("\n============ Test Item: Rawdata Unfiormity Test\n");

	if (!ts_data->fts_autotest_offset->fts_uniformity_data_P
	    || !ts_data->fts_autotest_offset->fts_uniformity_data_N) {
		TPD_INFO("fts_uniformity_data_P || fts_uniformity_data_N is NULL");
		return 0;
	}

	if (!ts_data->rawdata || !ts_data->rawdata_linearity || !ts_data->node_valid) {
		FTS_TEST_SAVE_ERR("rawdata/rawdata_linearity is null\n");
		ret = -EINVAL;
		goto test_err;
	}

	result = true;
	/*    FTS_TEST_SAVE_INFO("Check Tx Linearity\n");*/
	ts_data->rl_cnt = 0;
	rl_tmp = ts_data->rawdata_linearity + ts_data->rl_cnt;

	for (row = 0; row < tx_num; row++) {
		for (col = 0; col < rx_num - 1; col++) {
			offset = row * rx_num + col;
			offset2 = row * rx_num + col + 1;
			deviation = abs(ts_data->rawdata[offset] - ts_data->rawdata[offset2]);
			max = max(ts_data->rawdata[offset], ts_data->rawdata[offset2]);
			max = max ? max : 1;
			rl_tmp[offset] = 100 * deviation / max;
		}
	}

	ft3683g_output_data(rl_tmp, ts_data, focal_testdata, NODE_MATCH);

	/* compare */
	for (i = 0; i < node_num; i++) {
		if (0 == ts_data->node_valid[i]) {
			continue;
		}

		if ((rl_tmp[i] < ts_data->fts_autotest_offset->fts_uniformity_data_N[i])
		    || (rl_tmp[i] > ts_data->fts_autotest_offset->fts_uniformity_data_P[i])) {
			TPD_INFO("uniformity data ERR [%d]: [%d] > [%d] > [%d] \n", i,
			         ts_data->fts_autotest_offset->fts_uniformity_data_P[i], rl_tmp[i],
			         ts_data->fts_autotest_offset->fts_uniformity_data_N[i]);
			FTS_TEST_SAVE_ERR("test fail,node(%4d,%4d)=%5d,range=(%5d,%5d)\n",
			                  i / rx_num + 1, i % rx_num + 1, rl_tmp[i],
			                  ts_data->fts_autotest_offset->fts_uniformity_data_N[i],
			                  ts_data->fts_autotest_offset->fts_uniformity_data_P[i]);
			result = false;
		}
	}

	ts_data->rl_cnt += node_num;

	result2 = true;
	/*    FTS_TEST_SAVE_INFO("Check Rx Linearity\n");*/
	rl_tmp = ts_data->rawdata_linearity + ts_data->rl_cnt;

	for (row = 0; row < tx_num - 1; row++) {
		for (col = 0; col < rx_num; col++) {
			offset = row * rx_num + col;
			offset2 = (row + 1) * rx_num + col;
			deviation = abs(ts_data->rawdata[offset] - ts_data->rawdata[offset2]);
			max = max(ts_data->rawdata[offset], ts_data->rawdata[offset2]);
			max = max ? max : 1;
			rl_tmp[offset] = 100 * deviation / max;
		}
	}

	ft3683g_output_data(rl_tmp, ts_data, focal_testdata, NODE_MATCH);


	/* compare */
	for (i = 0; i < node_num; i++) {
		if (0 == ts_data->node_valid[i]) {
			continue;
		}

		if ((rl_tmp[i] < ts_data->fts_autotest_offset->fts_uniformity_data_N[i])
		    || (rl_tmp[i] > ts_data->fts_autotest_offset->fts_uniformity_data_P[i])) {
			TPD_INFO("uniformity data ERR [%d]: [%d] > [%d] > [%d] \n", i,
			         ts_data->fts_autotest_offset->fts_uniformity_data_P[i], rl_tmp[i],
			         ts_data->fts_autotest_offset->fts_uniformity_data_N[i]);
			FTS_TEST_SAVE_ERR("test fail,node(%4d,%4d)=%5d,range=(%5d,%5d)\n",
			                  i / rx_num + 1, i % rx_num + 1, rl_tmp[i],
			                  ts_data->fts_autotest_offset->fts_uniformity_data_N[i],
			                  ts_data->fts_autotest_offset->fts_uniformity_data_P[i]);
			result2 = false;
		}
	}

	ts_data->rl_cnt += node_num;

test_err:
	if (result && result2) {
		FTS_TEST_SAVE_INFO("------Rawdata Uniformity Test PASS\n");
		ret = TEST_RESULT_NORMAL;

	} else {
		FTS_TEST_SAVE_ERR("------Rawdata Uniformity Test NG\n");
		ret = TEST_RESULT_ABNORMAL;
	}

	FTS_TEST_FUNC_EXIT();
	return ret;
}



int ft3683g_scap_cb_autotest(struct seq_file *s, void *chip_data,
                            struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info)
{
	int ret = 0;
	bool tmp_result = false;
	bool tmp2_result = false;
	u8 wc_sel = 0;
	u8 sc_mode = 0;
	bool fw_wp_check = false;
	bool tx_check = false;
	bool rx_check = false;
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	int *scb_tmp = NULL;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int channel_num = tx_num + rx_num;
	int byte_num = channel_num;
	int scap_gcb_tx = 0;
	int scap_gcb_rx = 0;

	FTS_TEST_FUNC_ENTER();
	FTS_TEST_SAVE_INFO("\n============ Test Item: Scap CB Test\n");

	if (!ts_data->fts_autotest_offset->fts_scap_cb_data_P
	    || !ts_data->fts_autotest_offset->fts_scap_cb_data_N
	    || !ts_data->fts_autotest_offset->fts_scap_cb_data_waterproof_N
	    || !ts_data->fts_autotest_offset->fts_scap_cb_data_waterproof_P) {
		TPD_INFO("fts_scap_cb_data_P || fts_scap_cb_data_N || waterproof_N || waterproof_P  is NULL");
		return 0;
	}

	if (!ts_data->scap_cb || !ts_data->node_valid_sc) {
		FTS_TEST_SAVE_ERR("scap_cb is null\n");
		ret = -EINVAL;
		goto test_err;
	}

	ret = enter_factory_mode(ts_data);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("enter factory mode fail,ret=%d\n", ret);
		goto test_err;
	}

	/* get waterproof channel select */
	ret = fts_test_read_reg(FACTORY_REG_WC_SEL, &wc_sel);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read water_channel_sel fail,ret=%d\n", ret);
		goto test_err;
	}

	ret = fts_test_read_reg(FACTORY_REG_MC_SC_MODE, &sc_mode);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read sc_mode fail,ret=%d\n", ret);
		goto test_err;
	}

	/* water proof on check */
	ts_data->scb_cnt = 0;
	fw_wp_check = get_fw_wp(wc_sel, WATER_PROOF_ON);

	if (fw_wp_check) {
		scb_tmp = ts_data->scap_cb + ts_data->scb_cnt;
		/* 1:waterproof 0:non-waterproof */
		ret = fts_test_write_reg(FACTORY_REG_MC_SC_MODE, WATER_PROOF_ON);
		if (ret < 0) {
			FTS_TEST_SAVE_ERR("set mc_sc mode fail\n");
			goto restore_reg;
		}

		ret = wait_state_update(TEST_RETVAL_AA);
		if (ret < 0) {
			FTS_TEST_SAVE_ERR("wait state update fail\n");
			goto restore_reg;
		}

		ret = get_cb_ft5662(scb_tmp, byte_num, false);
		if (ret < 0) {
			FTS_TEST_SAVE_ERR("read sc_cb fail,ret=%d\n", ret);
			goto restore_reg;
		}

		ret = fts_test_read_reg(FACTROY_REG_SCAP_GCB_RX, (u8 *)&scap_gcb_rx);
		if (ret) {
			FTS_TEST_SAVE_ERR("read GCB_RX fail,ret=%d\n", ret);
			goto restore_reg;
		}

		ret = fts_test_read_reg(FACTROY_REG_SCAP_GCB_TX, (u8 *)&scap_gcb_tx);
		if (ret) {
			FTS_TEST_SAVE_ERR("read GCB_TX fail,ret=%d\n", ret);
			goto restore_reg;
		}
		TPD_INFO("GCB:RX=%d,TX=%d", scap_gcb_rx, scap_gcb_tx);

		/* compare */
		tx_check = get_fw_wp(wc_sel, WATER_PROOF_ON_TX);
		rx_check = get_fw_wp(wc_sel, WATER_PROOF_ON_RX);

		ft3683g_output_data(scb_tmp, ts_data, focal_testdata, CHEN_MATCH);
		ft3683g_output_data(&scap_gcb_rx, ts_data, focal_testdata, ONE_MATCH);
		ft3683g_output_data(&scap_gcb_tx, ts_data, focal_testdata, ONE_MATCH);

		tmp_result = compare_mc_sc(ts_data, tx_check, rx_check, scb_tmp,
		                           ts_data->fts_autotest_offset->fts_scap_cb_data_waterproof_N,
		                           ts_data->fts_autotest_offset->fts_scap_cb_data_waterproof_P);

		if (rx_check && ((scap_gcb_rx < SCAP_CB_ON_GCB_MIN) ||
		                 (scap_gcb_rx > SCAP_CB_ON_GCB_MAX))) {
			FTS_TEST_SAVE_ERR("test fail,gcb_rx:%5d,range=(%5d,%5d)\n",
			                  scap_gcb_rx, SCAP_CB_ON_GCB_MIN,
			                  SCAP_CB_ON_GCB_MAX);
			tmp_result = false;
		}

		if (tx_check && ((scap_gcb_tx < SCAP_CB_ON_GCB_MIN) ||
		                 (scap_gcb_tx > SCAP_CB_ON_GCB_MAX))) {
			FTS_TEST_SAVE_ERR("test fail,gcb_tx:%5d,range=(%5d,%5d)\n",
			                  scap_gcb_tx, SCAP_CB_ON_GCB_MIN,
			                  SCAP_CB_ON_GCB_MAX);
			tmp_result = false;
		}

		ts_data->scb_cnt += channel_num;

	} else {
		tmp_result = true;
	}

	/* water proof off check */
	fw_wp_check = get_fw_wp(wc_sel, WATER_PROOF_OFF);

	if (fw_wp_check) {
		scb_tmp = ts_data->scap_cb + ts_data->scb_cnt;
		/* 1:waterproof 0:non-waterproof */
		ret = fts_test_write_reg(FACTORY_REG_MC_SC_MODE, WATER_PROOF_OFF);
		if (ret < 0) {
			FTS_TEST_SAVE_ERR("set mc_sc mode fail\n");
			goto restore_reg;
		}

		ret = wait_state_update(TEST_RETVAL_AA);
		if (ret < 0) {
			FTS_TEST_SAVE_ERR("wait state update fail\n");
			goto restore_reg;
		}

		ret = get_cb_ft5662(scb_tmp, byte_num, false);
		if (ret < 0) {
			FTS_TEST_SAVE_ERR("read sc_cb fail,ret=%d\n", ret);
			goto restore_reg;
		}

		ret = fts_test_read_reg(FACTROY_REG_SCAP_GCB_RX, (u8 *)&scap_gcb_rx);
		if (ret) {
			FTS_TEST_SAVE_ERR("read GCB_RX fail,ret=%d\n", ret);
			goto restore_reg;
		}

		ret = fts_test_read_reg(FACTROY_REG_SCAP_GCB_TX, (u8 *)&scap_gcb_tx);
		if (ret) {
			FTS_TEST_SAVE_ERR("read GCB_TX fail,ret=%d\n", ret);
			goto restore_reg;
		}
		TPD_INFO("GCB:RX=%d,TX=%d", scap_gcb_rx, scap_gcb_tx);

		/* compare */
		tx_check = get_fw_wp(wc_sel, WATER_PROOF_OFF_TX);
		rx_check = get_fw_wp(wc_sel, WATER_PROOF_OFF_RX);

		ft3683g_output_data(scb_tmp, ts_data, focal_testdata, CHEN_MATCH);
		ft3683g_output_data(&scap_gcb_rx, ts_data, focal_testdata, ONE_MATCH);
		ft3683g_output_data(&scap_gcb_tx, ts_data, focal_testdata, ONE_MATCH);

		tmp2_result = compare_mc_sc(ts_data, tx_check, rx_check, scb_tmp,
		                            ts_data->fts_autotest_offset->fts_scap_cb_data_N,
		                            ts_data->fts_autotest_offset->fts_scap_cb_data_P);

		if (rx_check && ((scap_gcb_rx < SCAP_CB_OFF_GCB_MIN) ||
		                 (scap_gcb_rx > SCAP_CB_OFF_GCB_MAX))) {
			FTS_TEST_SAVE_ERR("test fail,gcb_rx:%5d,range=(%5d,%5d)\n",
			                  scap_gcb_rx, SCAP_CB_OFF_GCB_MIN,
			                  SCAP_CB_OFF_GCB_MAX);
			tmp2_result = false;
		}

		if (tx_check && ((scap_gcb_tx < SCAP_CB_OFF_GCB_MIN) ||
		                 (scap_gcb_tx > SCAP_CB_OFF_GCB_MAX))) {
			FTS_TEST_SAVE_ERR("test fail,gcb_tx:%5d,range=(%5d,%5d)\n",
			                  scap_gcb_tx, SCAP_CB_OFF_GCB_MIN,
			                  SCAP_CB_OFF_GCB_MAX);
			tmp2_result = false;
		}

		ts_data->scb_cnt += channel_num;

	} else {
		tmp2_result = true;
	}

restore_reg:
	ret = fts_test_write_reg(FACTORY_REG_MC_SC_MODE, sc_mode);

	if (ret) {
		FTS_TEST_SAVE_ERR("write sc mode fail,ret=%d\n", ret);
	}

	ret = wait_state_update(TEST_RETVAL_AA);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("wait state update fail\n");
	}

test_err:
	if (tmp_result && tmp2_result) {
		FTS_TEST_SAVE_INFO("------Scap CB (normal && waterproof) Test PASS\n");
		ret = TEST_RESULT_NORMAL;

	} else {
		if (tmp_result) {
			FTS_TEST_SAVE_ERR("------Scap CB Test (waterproof) NG\n");
		}

		if (tmp2_result) {
			FTS_TEST_SAVE_ERR("------Scap CB Test (normal) NG\n");
		}

		ret = TEST_RESULT_ABNORMAL;
	}

	FTS_TEST_FUNC_EXIT();
	return ret;
}


int ft3683g_scap_rawdata_autotest(struct seq_file *s, void *chip_data,
                                 struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info)
{
	int ret = 0;
	bool tmp_result = false;
	bool tmp2_result = false;
	u8 wc_sel = 0;
	bool fw_wp_check = false;
	bool tx_check = false;
	bool rx_check = false;
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	int *srawdata_tmp = NULL;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int channel_num = tx_num + rx_num;
	int byte_num = channel_num * 2;

	FTS_TEST_FUNC_ENTER();
	FTS_TEST_SAVE_INFO("\n============ Test Item: Scap Rawdata Test\n");

	if (!ts_data->fts_autotest_offset->fts_scap_raw_data_P
	    || !ts_data->fts_autotest_offset->fts_scap_raw_data_N
	    || !ts_data->fts_autotest_offset->fts_scap_raw_waterproof_data_N
	    || !ts_data->fts_autotest_offset->fts_scap_raw_waterproof_data_P) {
		TPD_INFO("fts_scap_raw_data_P || fts_scap_raw_data_N || raw_waterproof_data_N || raw_waterproof_data_P is NULL");
		return 0;
	}

	if (!ts_data->scap_rawdata || !ts_data->node_valid_sc) {
		FTS_TEST_SAVE_ERR("scap_rawdata is null\n");
		ret = -EINVAL;
		goto test_err;
	}

	ret = enter_factory_mode(ts_data);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("enter factory mode fail,ret=%d\n", ret);
		goto test_err;
	}

	/* get waterproof channel select */
	ret = fts_test_read_reg(FACTORY_REG_WC_SEL, &wc_sel);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read water_channel_sel fail,ret=%d\n", ret);
		goto test_err;
	}

	/* scan rawdata 2 times */
	ret = start_scan();

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("scan scap rawdata fail\n");
		goto test_err;
	}

	ret = start_scan();

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("scan scap rawdata fail\n");
		goto test_err;
	}

	/* water proof on check */
	ts_data->srawdata_cnt = 0;
	fw_wp_check = get_fw_wp(wc_sel, WATER_PROOF_ON);

	if (fw_wp_check) {
		srawdata_tmp = ts_data->scap_rawdata + ts_data->srawdata_cnt;
		ret = get_rawdata_mc_sc(WATER_PROOF_ON, byte_num, srawdata_tmp);

		if (ret < 0) {
			FTS_TEST_SAVE_ERR("get scap(WP_ON) rawdata fail\n");
			goto test_err;
		}

		/* compare */
		tx_check = get_fw_wp(wc_sel, WATER_PROOF_ON_TX);
		rx_check = get_fw_wp(wc_sel, WATER_PROOF_ON_RX);

		ft3683g_output_data(srawdata_tmp, ts_data, focal_testdata, CHEN_MATCH);

		tmp_result = compare_mc_sc(ts_data, tx_check, rx_check, srawdata_tmp,
		                           ts_data->fts_autotest_offset->fts_scap_raw_waterproof_data_N,
		                           ts_data->fts_autotest_offset->fts_scap_raw_waterproof_data_P);
		ts_data->srawdata_cnt += channel_num;

	} else {
		tmp_result = true;
	}

	/* water proof off check */
	fw_wp_check = get_fw_wp(wc_sel, WATER_PROOF_OFF);

	if (fw_wp_check) {
		srawdata_tmp = ts_data->scap_rawdata + ts_data->srawdata_cnt;
		ret = get_rawdata_mc_sc(WATER_PROOF_OFF, byte_num, srawdata_tmp);

		if (ret < 0) {
			FTS_TEST_SAVE_ERR("get scap(WP_OFF) rawdata fail\n");
			goto test_err;
		}

		/* compare */
		tx_check = get_fw_wp(wc_sel, WATER_PROOF_OFF_TX);
		rx_check = get_fw_wp(wc_sel, WATER_PROOF_OFF_RX);

		ft3683g_output_data(srawdata_tmp, ts_data, focal_testdata, CHEN_MATCH);

		tmp2_result = compare_mc_sc(ts_data, tx_check, rx_check, srawdata_tmp,
		                            ts_data->fts_autotest_offset->fts_scap_raw_data_N,
		                            ts_data->fts_autotest_offset->fts_scap_raw_data_P);
		ts_data->srawdata_cnt += channel_num;

	} else {
		tmp2_result = true;
	}

test_err:
	if (tmp_result && tmp2_result) {
		FTS_TEST_SAVE_INFO("------SCAP Rawdata Test PASS\n");
		ret = TEST_RESULT_NORMAL;

	} else {
		if (tmp_result) {
			FTS_TEST_SAVE_INFO("------SCAP Rawdata(WATER_PROOF_ON) Test NG\n");
		}

		if (tmp2_result) {
			FTS_TEST_SAVE_INFO("------SCAP Rawdata(NORMAL) Test NG\n");
		}

		ret = TEST_RESULT_ABNORMAL;
	}

	FTS_TEST_FUNC_EXIT();
	return ret;
}

int ft3683g_short_test(struct seq_file *s, void *chip_data,
                      struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info)
{
	int ret = 0;
	int adc[256] = { 0 };
	u8 ab_ch[256] = { 0 };
	bool ca_result = false;
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;

	FTS_TEST_FUNC_ENTER();
	FTS_TEST_SAVE_INFO("\n============ Test Item: Short Test\n");
	ret = enter_factory_mode(ts_data);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("enter factory mode fail,ret=%d\n", ret);
		goto test_err;
	}

	/* get short resistance and exceptional channel */
	ret = short_test_ch_to_all(ts_data, adc, ab_ch, &ca_result);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("short test of channel to all fails\n");
		goto test_err;
	}

test_err:
	if (ca_result) {
		FTS_TEST_SAVE_INFO("------Short test PASS\n");
		ret = TEST_RESULT_NORMAL;

	} else {
		FTS_TEST_SAVE_ERR("------Short Test NG\n");
		ret = TEST_RESULT_ABNORMAL;
	}

	FTS_TEST_FUNC_EXIT();
	return ret;
}

int ft3683g_rst_autotest(struct seq_file *s, void *chip_data,
                                  struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info)
{
	int ret = 0;
	u8 val = 0;
	u8 val2 = 0;
	u8 val3 = 0;
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;

	FTS_TEST_FUNC_ENTER();
	FTS_TEST_SAVE_INFO("\n============ Test Item: Reset Test\n");

	enter_work_mode();

	ret = fts_test_read_reg(FACTORY_FTS_RESET_TEST, &val);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("one: read report_rate error, ret=%d\n", ret);
		goto test_err;
	}

	val2 = val - 1;
	ret = fts_test_write_reg(FACTORY_FTS_RESET_TEST, val2);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("one: set report_rate fail, ret=%d\n", ret);
		goto test_err;
	}

	ft3683g_rstpin_reset((void*)ts_data);
	ret = fts_test_read_reg(FACTORY_FTS_RESET_TEST, &val3);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("one: read report_rate error, ret=%d\n", ret);
		goto test_err;
	}

	TPD_INFO("one: reset test: val = %d, val3 = %d", val, val3);

	ret = fts_test_read_reg(FACTORY_FTS_RESET_TEST, &val);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("two: read report_rate error, ret=%d\n", ret);
		goto test_err;
	}

	val2 = val - 1;
	ret = fts_test_write_reg(FACTORY_FTS_RESET_TEST, val2);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("two: set report_rate fail, ret=%d\n", ret);
		goto test_err;
	}

	ft3683g_rstpin_reset((void*)ts_data);
	ret = fts_test_read_reg(FACTORY_FTS_RESET_TEST, &val3);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("two: read report_rate error, ret=%d\n", ret);
		goto test_err;
	}

	TPD_INFO("two: reset test: val = %d, val3 = %d", val, val3);

	if (val3 != val) {
		FTS_TEST_SAVE_ERR("check reg to test rst failed.\n");
		ret = -1;
	}

test_err:
	if (!ret) {
		FTS_TEST_SAVE_INFO("------Reset Test PASS\n");
	} else {
		FTS_TEST_SAVE_INFO("------Reset Test NG\n");
	}

	FTS_TEST_FUNC_EXIT();
	return ret;
}

int ft3683g_panel_differ_test(struct seq_file *s, void *chip_data,
                             struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info)
{
	int ret = 0;
	int i = 0;
	u8 fre = 0;
	u8 g_cb = 0;
	u8 normalize = 0;
	u8 data_type = 0;
	u8 rawdata_addr = 0;
	bool result = false;
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	int byte_num = 0;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int node_num = tx_num * rx_num;

	FTS_TEST_FUNC_ENTER();
	FTS_TEST_SAVE_INFO("\n============ Test Item: Panel Differ Test\n");

	if (!ts_data->fts_autotest_offset->fts_panel_differ_data_P
	    || !ts_data->fts_autotest_offset->fts_panel_differ_data_N) {
		TPD_INFO("fts_panel_differ_data_P || fts_panel_differ_data_N is NULL");
		return 0;
	}

	if (!ts_data->panel_differ || !ts_data->node_valid) {
		FTS_TEST_SAVE_ERR("rawdata is null\n");
		ret = -EINVAL;
		goto test_err;
	}

	ret = enter_factory_mode(ts_data);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("failed to enter factory mode,ret=%d\n", ret);
		goto test_err;
	}

	/* save origin value */
	ret = fts_test_read_reg(FACTORY_REG_FRE_LIST, &fre);
	if (ret) {
		FTS_TEST_SAVE_ERR("read 0x0A fail,ret=%d\n", ret);
		goto test_err;
	}

	ret = fts_test_read_reg(FACTORY_REG_DATA_SELECT, &data_type);
	if (ret) {
		FTS_TEST_SAVE_ERR("read 0x5B fail,ret=%d\n", ret);
		goto test_err;
	}

	ret = fts_test_read_reg(FACTORY_REG_GCB, &g_cb);
	if (ret) {
		FTS_TEST_SAVE_ERR("read regBD fail,ret=%d\n", ret);
		goto test_err;
	}

	ret = fts_test_read_reg(FACTORY_REG_NORMALIZE, &normalize);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read normalize fail,ret=%d\n", ret);
		goto test_err;
	}

	/* set frequecy high */
	ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, 0x81);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("set frequecy fail,ret=%d\n", ret);
		goto restore_reg;
	}

	/* wait fw state update */
	ret = wait_state_update(TEST_RETVAL_AA);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("wait state update fail\n");
		goto restore_reg;
	}

	ret = fts_test_write_reg(FACTORY_REG_DATA_SELECT, 0x0);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("set raw type fail,ret=%d\n", ret);
		goto restore_reg;
	}

	ret = fts_test_write_reg(FACTORY_REG_GCB, 0x00);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("set fir fail,ret=%d\n", ret);
		goto restore_reg;
	}

	ret = wait_state_update(TEST_RETVAL_AA);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("wait state update fail\n");
		goto restore_reg;
	}

	/* set to overall normalize */
	ret = fts_test_write_reg(FACTORY_REG_NORMALIZE, 0x00);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("write normalize fail,ret=%d\n", ret);
		goto restore_reg;
	}

	/* get rawdata */
	for (i = 0; i < 3; i++) {
		/* lost 3 frames, in order to obtain stable data */
		/* start scanning */
		ret = start_scan();

		if (ret < 0) {
			FTS_TEST_SAVE_ERR("scan fail\n");
			continue;
		}

		/* read rawdata */
		rawdata_addr = FACTORY_REG_RAWDATA_ADDR_MC_SC;
		byte_num = node_num * 2;
		ret = read_rawdata(FACTORY_REG_LINE_ADDR, 0xAA, rawdata_addr, byte_num,
		                   ts_data->panel_differ);

		if (ret < 0) {
			FTS_TEST_SAVE_ERR("read rawdata fail\n");
		}
	}

	for (i = 0; i < node_num; i++) {
		ts_data->panel_differ[i] = ts_data->panel_differ[i] / 10;
	}
	ft3683g_output_data(ts_data->panel_differ, ts_data, focal_testdata, NODE_MATCH);


	/* compare */
	result = true;

	for (i = 0; i < node_num; i++) {
		if (0 == ts_data->node_valid[i]) {
			continue;
		}

		if ((ts_data->panel_differ[i] < ts_data->fts_autotest_offset->fts_panel_differ_data_N[i])
		    || (ts_data->panel_differ[i] > ts_data->fts_autotest_offset->fts_panel_differ_data_P[i])) {
			TPD_INFO("raw data ERR [%d]: [%d] > [%d] > [%d] \n", i,
			         ts_data->fts_autotest_offset->fts_panel_differ_data_P[i], ts_data->panel_differ[i],
			         ts_data->fts_autotest_offset->fts_panel_differ_data_N[i]);
			FTS_TEST_SAVE_ERR("test fail,node(%4d,%4d)=%5d,range=(%5d,%5d)\n",
			                  i / rx_num + 1, i % rx_num + 1, ts_data->panel_differ[i],
			                  ts_data->fts_autotest_offset->fts_panel_differ_data_N[i],
			                  ts_data->fts_autotest_offset->fts_panel_differ_data_P[i]);
			result = false;
		}
	}

restore_reg:
	/* set the origin value */
	ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, fre);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("restore 0x0A fail,ret=%d\n", ret);
	}

	/* wait fw state update */
	ret = wait_state_update(TEST_RETVAL_AA);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("wait state update fail\n");
	}

	ret = fts_test_write_reg(FACTORY_REG_DATA_SELECT, data_type);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("set raw type fail,ret=%d\n", ret);
	}

	ret = fts_test_write_reg(FACTORY_REG_GCB, g_cb);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("restore 0xBD fail,ret=%d\n", ret);
	}

	ret = wait_state_update(TEST_RETVAL_AA);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("wait state update fail\n");
	}

	ret = fts_test_write_reg(FACTORY_REG_NORMALIZE, normalize);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("restore normalize fail,ret=%d\n", ret);
	}

test_err:

	if (result) {
		FTS_TEST_SAVE_INFO("------Panel differ test PASS\n");
		ret = TEST_RESULT_NORMAL;
	} else {
		FTS_TEST_SAVE_INFO("------Panel differ test NG\n");
		ret = TEST_RESULT_ABNORMAL;
	}

	FTS_TEST_FUNC_EXIT();
	return ret;
}

int ft3683g_noise_autotest(struct seq_file *s, void *chip_data,
                          struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info)
{
	int ret = 0;
	int i = 0;
	u8 fre = 0;
	u8 reg06_val = 0;
	u8 reg0d_val = 0;
	u8 reg1a_val = 0;
	u8 reg1b_val = 0;
	u8 rawdata_addr = 0;
	bool result = false;
	int frame_num = 20;
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;
	int byte_num = 0;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int node_num = tx_num * rx_num;

	FTS_TEST_FUNC_ENTER();
	FTS_TEST_SAVE_INFO("\n============ Test Item: Noise Test\n");

	if (!ts_data->fts_autotest_offset->fts_noise_data_P
	    || !ts_data->fts_autotest_offset->fts_noise_data_N) {
		TPD_INFO("fts_noise_data_P || fts_noise_data_N is NULL");
		return 0;
	}

	if (!ts_data->noise_rawdata || !ts_data->node_valid) {
		FTS_TEST_SAVE_ERR("noise data is null\n");
		ret = -EINVAL;
		goto test_err;
	}

	ret = enter_factory_mode(ts_data);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("failed to enter factory mode,ret=%d\n", ret);
		goto test_err;
	}

	ret = fts_test_read_reg(FACTORY_REG_TOUCH_THR, &reg0d_val);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read reg0d fail,ret=%d\n", ret);
		goto test_err;
	}

	TPD_INFO("reg0d_val = [%d]\n", reg0d_val);

	/* save origin value */
	ret = fts_test_read_reg(FACTORY_REG_DATA_SELECT, &reg06_val);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read reg06 fail,ret=%d\n", ret);
		goto test_err;
	}

	TPD_INFO("reg06_val = [%d]\n", reg06_val);

	ret = fts_test_read_reg(FACTORY_REG_FRE_LIST, &fre);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read fre error,ret=%d\n", ret);
		goto test_err;
	}

	TPD_INFO("fre = [%d]\n", fre);

	ret = fts_test_read_reg(0x1A, &reg1a_val);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read 0x1A error,ret=%d\n", ret);
		goto test_err;
	}

	TPD_INFO("reg1a_val = [%d]\n", reg1a_val);

	ret = fts_test_read_reg(0x1B, &reg1b_val);
	if (ret < 0) {
        FTS_TEST_SAVE_ERR("read reg1b fail,ret=%d\n", ret);
        goto test_err;
	}

	ret = fts_test_write_reg(FACTORY_REG_DATA_SELECT, 0x01);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("set reg06 fail,ret=%d\n", ret);
		goto restore_reg;
	}

	msleep(10);

	ret = fts_test_write_reg(0x1A, 0x01);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("write 0x1A fail,ret=%d\n", ret);
		goto restore_reg;
	}

	ret = fts_test_write_reg(FACTORY_REG_FRAME_NUM, (frame_num >> 8) & 0xFF);
	ret = fts_test_write_reg(FACTORY_REG_FRAME_NUM + 1, frame_num & 0xFF);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("set frame fail,ret=%d\n", ret);
		goto restore_reg;
	}

	ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, 0x00);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("set fre fail,ret=%d\n", ret);
		goto restore_reg;
	}
	msleep(20);

	ret = wait_state_update(TEST_RETVAL_AA);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("wait state update fail\n");
		goto restore_reg;
	}

	ret = fts_test_write_reg(0x1B, 0x01);
	if (ret < 0) {
        FTS_TEST_SAVE_ERR("set scan mode fail,ret=%d\n", ret);
        return ret;
	}

	ft3683g_start_scan(frame_num);

	/* read rawdata */
	rawdata_addr = 0xCE;
	byte_num = node_num * 2;
	ret = read_rawdata(FACTORY_REG_LINE_ADDR, 0xAA, rawdata_addr, byte_num,
	                   ts_data->noise_rawdata);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read rawdata fail\n");
		result = false;
		goto restore_reg;
	}

	ft3683g_output_data(ts_data->noise_rawdata, ts_data, focal_testdata, NODE_MATCH);

	/* compare */
	result = true;

	if (ts_data->fts_autotest_offset->fts_noise_data_P
	    && ts_data->fts_autotest_offset->fts_noise_data_N) {
		for (i = 0; i < node_num; i++) {
			if (0 == ts_data->node_valid[i]) {
				continue;
			}

			if (ts_data->noise_rawdata[i] >
			    ts_data->fts_autotest_offset->fts_noise_data_P[i]) {
				TPD_INFO("noise data ERR [%d]: [%d] > [%d] > [%d] \n", i,
				         ts_data->fts_autotest_offset->fts_noise_data_P[i], ts_data->noise_rawdata[i],
				         ts_data->fts_autotest_offset->fts_noise_data_N[i]);
				FTS_TEST_SAVE_ERR("test fail,node(%4d,%4d)=%5d,range=(%5d,%5d)\n",
				                  i / rx_num + 1, i % rx_num + 1, ts_data->noise_rawdata[i],
				                  ts_data->fts_autotest_offset->fts_noise_data_N[i],
				                  ts_data->fts_autotest_offset->fts_noise_data_P[i]);
				result = false;
			}
		}

	} else {
		TPD_INFO("fts_raw_data_P || fts_raw_data_N is null \n");
		result = false;
	}

	ft3683g_get_null_noise(ts_data, focal_testdata);

restore_reg:

	/* set the origin value */
	ret = fts_test_write_reg(FACTORY_REG_DATA_SELECT, reg06_val);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("restore normalize fail,ret=%d\n", ret);
	}

	ret = wait_state_update(TEST_RETVAL_AA);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("wait state update fail\n");
	}

	ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, fre);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("restore 0x0A fail,ret=%d\n", ret);
	}

	ret = wait_state_update(TEST_RETVAL_AA);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("wait state update fail\n");
	}

	ret = fts_test_write_reg(0x1A, reg1a_val);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("restore 0x1A fail,ret=%d\n", ret);
	}

	ret = fts_test_write_reg(0x1B, reg1b_val);
	if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore reg1b fail,ret=%d\n", ret);
	}

test_err:

	if (result) {
		FTS_TEST_SAVE_INFO("------Noise Test PASS\n");
		ret = TEST_RESULT_NORMAL;

	} else {
		FTS_TEST_SAVE_INFO("------Noise Test NG\n");
		ret = TEST_RESULT_ABNORMAL;
	}

	FTS_TEST_FUNC_EXIT();
	return ret;
}

static int ft3683g_membist_write_cmd(int offset, int num)
{
	u8 state_cmd[] = {0x70, 0x06, 0xf9, 0x80, 0x4e, 0x00, 0x00};
	u8 addr_value = 0x71;
	int i = 0;
	int ret = 0;
	u8 cmd_read_data[5] = {0};
	int data = 0;
	int cmd_len = 0;

	for(i = offset; i < offset + num; i++) {
        cmd_len = 11;
		if(!memcpy(state_cmd, soc_membist_test_cmd[i], 7)) {
        cmd_len = 7;
		}
		ret = fts_test_spi_write_direct(soc_membist_test_cmd[i], cmd_len);
		if (ret < 0) {
			FTS_TEST_SAVE_ERR("write cmd fail,i = %d ,ret=%d\n", i, ret);
			return ret;
		}

		if(!memcpy(state_cmd, soc_membist_test_cmd[i], 7)) {
			ret = fts_test_spi_read_direct(&addr_value, 1, cmd_read_data, 4);
			if (ret < 0) {
			FTS_TEST_SAVE_ERR("sos set bist mode read fail ,ret=%d\n", ret);
			return ret;
			}
			data = (cmd_read_data[1] << 24) | (cmd_read_data[2] << 16) | (cmd_read_data[3] << 8) | cmd_read_data[4];
			if(data != 0x01000000) {
				FTS_TEST_SAVE_ERR("read data = %x", data);
				return -EINVAL;
			}
		}
	}
	return 0;
}

static int ft3683g_membist_read_cmd(u8 *cmd, u8 *data)
{
	int ret = 0;
	u8 addr_value = 0x71;
	u8 cmd_read_data[5] = {0};

	ret = fts_test_spi_write_direct(cmd, 7);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("sos set Burnin mode fail ,ret=%d\n", ret);
		return ret;
	}

	ret = fts_test_spi_read_direct(&addr_value, 1, cmd_read_data, 5);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("sos set bist mode read fail ,ret=%d\n", ret);
		return ret;
	}
	memcpy(data, cmd_read_data, 5);
	return 0;
}

static bool ft3683g_debug_mode_check(void)
{
	u8 cmd[] = {0x70, 0x06, 0xf9, 0x80, 0x0d, 0x00, 0x00};
	u8 addr_value = 0x71;
	u8 cmd_read_data[5] = {0};
	int ret = 0;
	u16 id = 0;

	ret = fts_test_spi_write_direct(cmd, 7);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("sos set Burnin mode fail ,ret=%d\n", ret);
		return false;
	}

	ret = fts_test_spi_read_direct(&addr_value, 1, cmd_read_data, 5);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("sos set bist mode read fail ,ret=%d\n", ret);
		return false;
	}
	FTS_TEST_SAVE_ERR("cmd_read_data=0x%02X 0x%02X 0x%02X 0x%02X\n", cmd_read_data[1], cmd_read_data[2], cmd_read_data[3], cmd_read_data[4]);
	id = (cmd_read_data[2] << 8) + cmd_read_data[1];
	if(id == 0x5008)
	return true;

	return false;
}

static bool ft3683g_enter_into_debug_mode(void)
{
	int i = 0;
	int ret = 0;
	u8 cmd[] = {0x70, 0x55, 0xAA};

	for(i = 0; i < 5; i++) {
		fts_reset_proc(0);
		mdelay(1);
		ret = fts_test_spi_write_direct(cmd, 3);
		if (ret < 0) {
			FTS_TEST_SAVE_ERR("write 0x55 0xAA fail ,ret=%d\n", ret);
			return false;
		}
		sys_delay(200);
		if(ft3683g_debug_mode_check()) {
			return true;
		}
	}
	return false;
}

int ft3683g_membist_test(struct seq_file *s, void *chip_data,
              struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info)
{
	int ret = 0;
	bool tmp_soc_result = true;
	u8 addr_value = 0x71;
	u8 cmd_read_data[5] = {0};
	u8 i = 0;
	int vdd12trim = 0;
	int osc104m_trim = 0;

	FTS_TEST_FUNC_ENTER();
	FTS_TEST_SAVE_INFO("\n============ Test Item: Mem Bist Test\n");

	fts_set_spi_max_speed(2000000, 1);
	if (!ft3683g_enter_into_debug_mode()) {
		FTS_TEST_SAVE_ERR("enter debug mode fail\n");
		tmp_soc_result = false;
		goto test_err;
	}


	ret = fts_test_spi_write_direct(soc_membist_test_cmd[1], 11);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("disable write protection fail ,ret=%d\n", ret);
		tmp_soc_result = false;
		goto test_err;
	}
	ret = fts_test_spi_write_direct(soc_membist_test_cmd[2], 7);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("enable clk fail ,ret=%d\n", ret);
		tmp_soc_result = false;
		goto test_err;
	}
	ret = fts_test_spi_read_direct(&addr_value, 1, cmd_read_data, 5);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("sos set bist mode read fail ,ret=%d\n", ret);
		tmp_soc_result = false;
		goto test_err;
	}
	FTS_TEST_SAVE_ERR("vdd12trim ,cmd_read_data=0x%02X 0x%02X 0x%02X 0x%02X\n", cmd_read_data[1], cmd_read_data[2], cmd_read_data[3], cmd_read_data[4]);
	vdd12trim = (cmd_read_data[4] << 24) | (cmd_read_data[3] << 16) | (cmd_read_data[2] << 8) | cmd_read_data[1];
	FTS_TEST_SAVE_ERR("vdd12trim = %x", vdd12trim);
	ret = fts_test_spi_write_direct(soc_membist_test_cmd[3], 7);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("sos set Burnin mode fail ,ret=%d\n", ret);
		tmp_soc_result = false;
		goto test_err;
	}
	ret = fts_test_spi_read_direct(&addr_value, 1, cmd_read_data, 5);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("sos set bist mode read fail ,ret=%d\n", ret);
		tmp_soc_result = false;
		goto test_err;
	}
	FTS_TEST_SAVE_ERR("osc104m_trim ,cmd_read_data=0x%02X 0x%02X 0x%02X 0x%02X\n", cmd_read_data[1], cmd_read_data[2], cmd_read_data[3], cmd_read_data[4]);
	osc104m_trim = (cmd_read_data[4] << 24) | (cmd_read_data[3] << 16) | (cmd_read_data[2] << 8) | cmd_read_data[1];
	FTS_TEST_SAVE_ERR("osc104m_trim = %x", osc104m_trim);

	for (i = 4;i < 6;i++) {
		ret = fts_test_spi_write_direct(soc_membist_test_cmd[i], sizeof(soc_membist_test_cmd[i]));
		if (ret < 0) {
		FTS_TEST_SAVE_ERR("sos set bist mode cmd[%d] fail ,ret=%d\n", i, ret);
		tmp_soc_result = false;
		goto test_err;
		}
	}
	ft3683g_membist_read_cmd(soc_membist_test_cmd[6], cmd_read_data);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("soc_membist_test_cmd[6] write fail ,ret=%d\n", ret);
		tmp_soc_result = false;
		goto test_err;
	}
	if ((cmd_read_data[1] == 0xFF) && (cmd_read_data[2] == 0xFF) && (cmd_read_data[3] == 0x00) && (cmd_read_data[4] == 0x00)) {
		FTS_TEST_SAVE_INFO("SOS Bist Mode Register success!!!\n");
	} else {
		tmp_soc_result = false;
		FTS_TEST_SAVE_ERR("membist NG!!!\n");
		FTS_TEST_SAVE_ERR("cmd_read_data=0x%02X 0x%02X 0x%02X 0x%02X\n", cmd_read_data[1], cmd_read_data[2], cmd_read_data[3], cmd_read_data[4]);
		goto test_err;
	}

	ret = ft3683g_membist_write_cmd(7, 4); /*4*/
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("sos set bist mode cmd[23] fail ,ret=%d\n", ret);
		tmp_soc_result = false;
		goto test_err;
	}

	/*ret = ft3683g_membist_write_cmd(10, 1); 4*/
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("sos set bist mode cmd[23] fail ,ret=%d\n", ret);
		tmp_soc_result = false;
		goto test_err;
	}


	if((vdd12trim & 0x07) != 0x05) {
		vdd12trim = (vdd12trim & 0xF0) + ((vdd12trim & 0x07) + 7);
		FTS_TEST_SAVE_ERR("vdd12trim = %x", vdd12trim);
	}
	soc_membist_test_cmd[11][7] = (vdd12trim & 0xFF);
	ret = fts_test_spi_write_direct(soc_membist_test_cmd[11], 11);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("set vdd12trim cmd[11] fail ,ret=%d\n", ret);
		tmp_soc_result = false;
		goto test_err;
	}

	ret = fts_test_spi_write_direct(soc_membist_test_cmd[12], 11);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("sos set bist mode cmd[12] fail ,ret=%d\n", ret);
		tmp_soc_result = false;
		goto test_err;
	}

	soc_membist_test_cmd[13][7] = (osc104m_trim & 0x007F);
	ret = fts_test_spi_write_direct(soc_membist_test_cmd[13], 11);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("sos set bist mode cmd[13] fail ,ret=%d\n", ret);
		tmp_soc_result = false;
		goto test_err;
	}

	ret = ft3683g_membist_write_cmd(14, 2);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("sos set bist mode cmd[14] fail ,ret=%d\n", ret);
		tmp_soc_result = false;
		goto test_err;
	}

	sys_delay(50);

	ret = fts_test_spi_write_direct(soc_membist_test_cmd[16], 7);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("sos set bist mode cmd[16] fail ,ret=%d\n", ret);
		tmp_soc_result = false;
		goto test_err;
	}
	ret = fts_test_spi_read_direct(&addr_value, 1, cmd_read_data, 4);
	if (ret < 0) {
	FTS_TEST_SAVE_ERR("sos set bist mode read fail ,ret=%d\n", ret);
		tmp_soc_result = false;
		goto test_err;
	}
	if ((cmd_read_data[1] == 0xE0) && (cmd_read_data[2] == 0x33) && (cmd_read_data[3] == 0x00) && (cmd_read_data[4] == 0x00)) {
		FTS_TEST_SAVE_INFO("SOS Bist Mode Register success!!!\n");
	} else {
		tmp_soc_result = false;
		FTS_TEST_SAVE_ERR("membist NG!!!\n");
		FTS_TEST_SAVE_ERR("5008j_membist_result0=0x%02X 0x%02X 0x%02X 0x%02X\n", cmd_read_data[1], cmd_read_data[2], cmd_read_data[3], cmd_read_data[4]);
		goto test_err;
	}

	ret = fts_test_spi_write_direct(soc_membist_test_cmd[17], 7);
	if (ret < 0) {
        FTS_TEST_SAVE_ERR("sos set bist mode cmd[17] fail ,ret=%d\n", ret);
        tmp_soc_result = false;
        goto test_err;
	}
	ret = fts_test_spi_read_direct(&addr_value, 1, cmd_read_data, 4);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("sos set bist mode read fail ,ret=%d\n", ret);
		tmp_soc_result = false;
		goto test_err;
	}
	if ((cmd_read_data[1] == 0x00) && (cmd_read_data[2] == 0x1F) && (cmd_read_data[3] == 0x00) && (cmd_read_data[4] == 0x00)) {
		FTS_TEST_SAVE_INFO("SOS Bist Mode Register success!!!\n");
	} else {
		tmp_soc_result = false;
		FTS_TEST_SAVE_ERR("membist NG!!!\n");
		FTS_TEST_SAVE_ERR("5008j_membist_result1=0x%02X 0x%02X 0x%02X 0x%02X\n", cmd_read_data[1], cmd_read_data[2], cmd_read_data[3], cmd_read_data[4]);
		goto test_err;
	}

	ret = ft3683g_membist_write_cmd(18, 30);
	if(ret < 0) {
		tmp_soc_result = false;
		goto test_err;
	}
	sys_delay(5);

	ret = ft3683g_membist_write_cmd(48, 42);
	if(ret < 0) {
		tmp_soc_result = false;
		goto test_err;
	}
	sys_delay(10);

	ret = ft3683g_membist_write_cmd(90, 5);
	if(ret < 0) {
		tmp_soc_result = false;
		goto test_err;
	}
	sys_delay(10);

	ret = fts_test_spi_write_direct(soc_membist_test_cmd[95], 7);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("sos set bist mode cmd[95] fail ,ret=%d\n", ret);
		tmp_soc_result = false;
		goto test_err;
	}

	addr_value = 0x71;
	ret = fts_test_spi_read_direct(&addr_value, 1, cmd_read_data, 4);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("sos set bist mode read fail ,ret=%d\n", ret);
		tmp_soc_result = false;
		goto test_err;
	}
	if ((cmd_read_data[1] == 0x55) && (cmd_read_data[2] == 0x55) && (cmd_read_data[3] == 0x00) && (cmd_read_data[4] == 0x00)) {
		FTS_TEST_SAVE_INFO("SOS Bist Mode Register success!!!\n");
	} else {
		tmp_soc_result = false;
		FTS_TEST_SAVE_ERR("membist NG!!!\n");
		FTS_TEST_SAVE_ERR("sos read bist mode reg,cmd_read_data=0x%02X 0x%02X 0x%02X 0x%02X\n",
						cmd_read_data[1], cmd_read_data[2], cmd_read_data[3], cmd_read_data[4]);
		goto test_err;
	}

	ret = ft3683g_membist_write_cmd(96, 6);
	if(ret < 0) {
		tmp_soc_result = false;
		goto test_err;
	}
	sys_delay(10);
	ret = fts_test_spi_write_direct(soc_membist_test_cmd[102], 7);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("sos set bist mode cmd[102] fail ,ret=%d\n", ret);
		tmp_soc_result = false;
		goto test_err;
	}

	addr_value = 0x71;
	ret = fts_test_spi_read_direct(&addr_value, 1, cmd_read_data, 4);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("sos set bist mode read fail ,ret=%d\n", ret);
		tmp_soc_result = false;
		goto test_err;
	}
	if ((cmd_read_data[1] == 0x55) && (cmd_read_data[2] == 0x45) && (cmd_read_data[3] == 0x00) && (cmd_read_data[4] == 0x00)) {
		FTS_TEST_SAVE_INFO("SOS Bist Mode Register success!!!\n");
	} else {
		tmp_soc_result = false;
		FTS_TEST_SAVE_ERR("membist NG!!!\n");
		FTS_TEST_SAVE_ERR("sos read bist mode reg ,cmd_read_data=0x%02X 0x%02X 0x%02X 0x%02X\n",
							cmd_read_data[1], cmd_read_data[2], cmd_read_data[3], cmd_read_data[4]);
		goto test_err;
	}


	ret = ft3683g_membist_write_cmd(103, 6);
	if(ret < 0) {
		tmp_soc_result = false;
		goto test_err;
	}
	sys_delay(10);
	ret = fts_test_spi_write_direct(soc_membist_test_cmd[109], 7);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("sos set bist mode cmd[109] fail ,ret=%d\n", ret);
		tmp_soc_result = false;
		goto test_err;
	}

	addr_value = 0x71;
	ret = fts_test_spi_read_direct(&addr_value, 1, cmd_read_data, 4);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("sos set bist mode read fail ,ret=%d\n", ret);
		tmp_soc_result = false;
		goto test_err;
	}
	if ((cmd_read_data[1] == 0x15) && (cmd_read_data[2] == 0x00) && (cmd_read_data[3] == 0x00) && (cmd_read_data[4] == 0x00)) {
		FTS_TEST_SAVE_INFO("SOS Bist Mode Register success!!!\n");
	} else {
		tmp_soc_result = false;
		FTS_TEST_SAVE_ERR("membist NG!!!\n");
		FTS_TEST_SAVE_ERR("sos read bist mode reg ,cmd_read_data=0x%02X 0x%02X 0x%02X 0x%02X\n",
					cmd_read_data[1], cmd_read_data[2], cmd_read_data[3], cmd_read_data[4]);
		goto test_err;
	}

	ret = ft3683g_membist_write_cmd(110, 2);
	if(ret < 0) {
		tmp_soc_result = false;
		goto test_err;
	}
	test_err:

	fts_reset_proc(200);
	fts_set_spi_max_speed(8000000, 0);

	/* result */
	if(tmp_soc_result) {
		ret = TEST_RESULT_NORMAL;
		FTS_TEST_SAVE_INFO("------MemBist Test PASS\n");
	} else {
		ret = TEST_RESULT_ABNORMAL;
		FTS_TEST_SAVE_ERR("------ MemBist Test NG\n");
	}

	FTS_TEST_FUNC_EXIT();
	return ret;
}

static u8 pb_cal_file_ft3683g[] = {
	#include "./CALCULATE_FT3683G_V01.h"
};

#define WRITE_PRAM_HARD_LEN 128
static int fts_ft3683g_write_pram_hard(u8 *data, int len)
{
	/*u8 cmd_1[] = {0x70, 0x55, 0xaa};*/
	u8 cmd_2[] = {0x70, 0x07, 0xf8, 0x00, 0x04, 0x00, 0x00, 0xaa, 0x55, 0x00, 0x00};
	u8 cmd_3[] = {0x70, 0x07, 0xf8, 0x00, 0x48, 0x00, 0x00, 0x01, 0x10, 0x00, 0x00};
	u8 cmd_4[] = {0x70, 0x05, 0xfa};
	u8 cmd_5[] = {0x70, 0x04, 0xfb};
	u8 cmd_6[] = {0x70, 0x07, 0xf8, 0x00, 0x07, 0x00, 0x00, 0x5A, 0x5A, 0x00, 0x00};
	u8 cmd_7[] = {0x70, 0x0a, 0xf5};

	u32 i = 0;
	u32 offset = 0;
	u32 addr = 0;
	u32 saddr = 0x2000;
	u32 remainder = 0;
	u32 packet_number;
	u32 packet_len = 0;
	u8 packet_buf[WRITE_PRAM_HARD_LEN + 12] = { 0 };
	u8 read_data_cmd[WRITE_PRAM_HARD_LEN + 12] = {0};
	int temp_len = 0;

	TPD_INFO("write pramboot to pram,pramboot len=%d", len);
	if (!data || (len < 0x120) || (len > (0x10000))) {
		TPD_INFO("buf/pramboot length(%d) fail", len);
		return -EINVAL;
	}

	if(!ft3683g_enter_into_debug_mode()) {
		FTS_TEST_SAVE_ERR("enter debug mode fail\n");
		return -EINVAL;
	}

	fts_spi_write_direct(cmd_2, sizeof(cmd_2));
	fts_spi_write_direct(cmd_3, sizeof(cmd_3));

	packet_number = len / WRITE_PRAM_HARD_LEN;
	remainder = len % WRITE_PRAM_HARD_LEN;
	if (remainder > 0)
	packet_number++;
	packet_len = WRITE_PRAM_HARD_LEN;

	for (i = 0; i < packet_number; i++) {
		offset = i * WRITE_PRAM_HARD_LEN;
		addr = saddr + (offset / 4);
		/* last packet */
		if ((i == (packet_number - 1)) && remainder) {
			packet_len = remainder;
		}

		temp_len = (packet_len / 4) - 1;
		memcpy(packet_buf, cmd_4, 3);
		packet_buf[3] = (addr >> 8) & 0xFF;
		packet_buf[4] = addr & 0xFF;
		packet_buf[5] = (temp_len >> 8) & 0xFF;
		packet_buf[6] = (temp_len) & 0xFF;
		memcpy(&packet_buf[7], &data[offset], packet_len);
		fts_spi_write_direct(packet_buf, packet_len + 7);

		memcpy(read_data_cmd, cmd_5, 3);
		read_data_cmd[3] = (addr >> 8) & 0xFF;
		read_data_cmd[4] = addr & 0xFF;
		read_data_cmd[5] = (temp_len >> 8) & 0xFF;
		read_data_cmd[6] = ((temp_len) & 0xFF);
		read_data_cmd[7] = 0x71;
		fts_spi_write_direct(read_data_cmd, 7);
		fts_spi_read_direct(&read_data_cmd[7], 1, &read_data_cmd[8], packet_len + 1);
		if(memcmp(&read_data_cmd[9], &data[offset], packet_len)) {
			TPD_INFO("write pram hard fail\n");
			return -EINVAL;
		}
	}
	fts_spi_write_direct(cmd_6, sizeof(cmd_6));
	fts_spi_write_direct(cmd_7, sizeof(cmd_7));
	msleep(200);
	return 0;
}

int ft3683g_cal_test(struct seq_file *s, void *chip_data,
			struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info)
{
	int ret = 0;
	u8 i = 0;
	u8 read_value = 0;
	u8 *pb_buf = pb_cal_file_ft3683g;
	u32 pb_len = sizeof(pb_cal_file_ft3683g);

	FTS_TEST_FUNC_ENTER();
	FTS_TEST_SAVE_INFO("\n============ Test Item: Cal Test\n");
	fts_set_spi_max_speed(2000000, 1);
	/*step1 write cal bin*/
	ret = fts_ft3683g_write_pram_hard(pb_buf, pb_len);
	if (ret < 0) {
		goto test_err;
	}

	/*step3 enable cal test*/
	ret = fts_test_write_reg(0xB6, 0x01);
	if (ret < 0) {
		TPD_INFO("enable cal test fail,ret=%d\n", ret);
		goto test_err;
	}

	/*step4 wait for cal test*/
	for (i = 0; i < 20 ;i++) {
		ret = fts_test_read_reg(0xB7, &read_value);
		if (ret < 0) {
			TPD_INFO("read 0xB7 fail,ret=%d,read_value = 0x%x\n", ret, read_value);
			goto test_err;
		}

		if ((read_value == 0x55) || (read_value == 0xAA)) {
			break;
		}
		sys_delay(10);
	}

	test_err:

	/*step5 result*/
	if(read_value == 0x55) {
		ret = TEST_RESULT_NORMAL;
		FTS_TEST_SAVE_ERR("------Cal test PASS!!!\n");
	} else {
		ret = TEST_RESULT_ABNORMAL;
		FTS_TEST_SAVE_ERR("------Cal test NG!!!\n");
		if (i >= 20) {
			FTS_TEST_SAVE_ERR("------Cal test timeout!!!\n");
		}
	}

	fts_reset_proc(200);
	fts_set_spi_max_speed(8000000, 0);
	FTS_TEST_FUNC_EXIT();
	return ret;
}

int ft3683g_auto_endoperation(struct seq_file *s, void *chip_data,
                             struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info)
{
	struct chip_data_ft3683g *ts_data = (struct chip_data_ft3683g *)chip_data;

	TPD_INFO("%s +\n", __func__);

	if (ts_data->fts_autotest_offset) {
		kfree(ts_data->fts_autotest_offset);
		ts_data->fts_autotest_offset = NULL;
	}

	if (ts_data->node_valid) {
		kfree(ts_data->node_valid);
		ts_data->node_valid = NULL;
	}

	if (ts_data->node_valid_sc) {
		kfree(ts_data->node_valid_sc);
		ts_data->node_valid_sc = NULL;
	}

	if (ts_data->rawdata_linearity) {
		kfree(ts_data->rawdata_linearity);
		ts_data->rawdata_linearity = NULL;
	}

	if (ts_data->scap_rawdata) {
		kfree(ts_data->scap_rawdata);
		ts_data->scap_rawdata = NULL;
	}

	if (ts_data->scap_cb) {
		kfree(ts_data->scap_cb);
		ts_data->scap_cb = NULL;
	}

	if (ts_data->rawdata) {
		kfree(ts_data->rawdata);
		ts_data->rawdata = NULL;
	}

	if (ts_data->panel_differ) {
		kfree(ts_data->panel_differ);
		ts_data->panel_differ = NULL;
	}

	if (ts_data->noise_rawdata) {
		kfree(ts_data->noise_rawdata);
		ts_data->noise_rawdata = NULL;
	}

	enter_work_mode();

	TPD_INFO("%s -\n", __func__);
	return 0;
}

#ifdef FTS_KIT
static int fts_get_threshold(struct chip_data_ft3683g *ts_data, struct auto_testdata *p_focal_testdata)
{
	int i = 0;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int node_num = tx_num * rx_num;
	int channel_num = tx_num + rx_num;
	const struct firmware *limit_fw = NULL;
	int offset = 32;

	limit_fw = (const struct firmware *) p_focal_testdata->fw;

	if (!limit_fw || !limit_fw->data || !limit_fw->size) {
		TPD_INFO("limit_fw is null");
		return -ENOMEM;
	}

	ts_data->fts_autotest_offset = kzalloc(sizeof(struct fts_autotest_offset), GFP_KERNEL);
	if (!ts_data->fts_autotest_offset) {
		TPD_INFO("fts_autotest_offset is null");
		return -ENOMEM;
	}

	ts_data->fts_autotest_offset->fts_noise_data_P = (int32_t *)(limit_fw->data + offset);
	offset += node_num * sizeof(int32_t);
	ts_data->fts_autotest_offset->fts_noise_data_N  = (int32_t *)(limit_fw->data + offset);
	offset += node_num * sizeof(int32_t);

	ts_data->fts_autotest_offset->fts_raw_data_P = (int32_t *)(limit_fw->data + offset);
	offset += node_num * sizeof(int32_t);
	ts_data->fts_autotest_offset->fts_raw_data_N  = (int32_t *)(limit_fw->data + offset);
	offset += node_num * sizeof(int32_t);

	ts_data->fts_autotest_offset->fts_uniformity_data_P = (int32_t *)(limit_fw->data + offset);
	offset += node_num * sizeof(int32_t);
	ts_data->fts_autotest_offset->fts_uniformity_data_N  = (int32_t *)(limit_fw->data + offset);
	offset += node_num * sizeof(int32_t);

	ts_data->fts_autotest_offset->fts_scap_cb_data_waterproof_P = (int32_t *)(limit_fw->data + offset);
	offset += channel_num * sizeof(int32_t);
	ts_data->fts_autotest_offset->fts_scap_cb_data_waterproof_N  = (int32_t *)(limit_fw->data + offset);
	offset += channel_num * sizeof(int32_t);

	ts_data->fts_autotest_offset->fts_scap_cb_data_P = (int32_t *)(limit_fw->data + offset);
	offset += channel_num * sizeof(int32_t);
	ts_data->fts_autotest_offset->fts_scap_cb_data_N  = (int32_t *)(limit_fw->data + offset);
	offset += channel_num * sizeof(int32_t);

	ts_data->fts_autotest_offset->fts_scap_raw_waterproof_data_P = (int32_t *)(limit_fw->data + offset);
	offset += channel_num * sizeof(int32_t);
	ts_data->fts_autotest_offset->fts_scap_raw_waterproof_data_N  = (int32_t *)(limit_fw->data + offset);
	offset += channel_num * sizeof(int32_t);

	ts_data->fts_autotest_offset->fts_scap_raw_data_P = (int32_t *)(limit_fw->data + offset);
	offset += channel_num * sizeof(int32_t);
	ts_data->fts_autotest_offset->fts_scap_raw_data_N  = (int32_t *)(limit_fw->data + offset);
	offset += channel_num * sizeof(int32_t);

	ts_data->fts_autotest_offset->fts_panel_differ_data_P = (int32_t *)(limit_fw->data + offset);
	offset += node_num * sizeof(int32_t);
	ts_data->fts_autotest_offset->fts_panel_differ_data_N  = (int32_t *)(limit_fw->data + offset);
	offset += node_num * sizeof(int32_t);

	for (i = 0; i < node_num; i++) {
		ts_data->fts_autotest_offset->fts_noise_data_P[i] = 300;
		ts_data->fts_autotest_offset->fts_noise_data_N[i] = 0;

		ts_data->fts_autotest_offset->fts_raw_data_P[i] = 40000;
		ts_data->fts_autotest_offset->fts_raw_data_N[i] = 0;

		ts_data->fts_autotest_offset->fts_panel_differ_data_P[i] = 40000;
		ts_data->fts_autotest_offset->fts_panel_differ_data_N[i] = 0;

		ts_data->fts_autotest_offset->fts_uniformity_data_P[i] = 100;
		ts_data->fts_autotest_offset->fts_uniformity_data_N[i] = 0;
	}

	for (i = 0; i < channel_num; i++) {
		ts_data->fts_autotest_offset->fts_scap_cb_data_waterproof_P[i] = 500;
		ts_data->fts_autotest_offset->fts_scap_cb_data_waterproof_N[i] = -500;
		ts_data->fts_autotest_offset->fts_scap_cb_data_P[i] = 500;
		ts_data->fts_autotest_offset->fts_scap_cb_data_N[i] = -500;

		ts_data->fts_autotest_offset->fts_scap_raw_waterproof_data_P[i] = 1000;
		ts_data->fts_autotest_offset->fts_scap_raw_waterproof_data_N[i] = -1000;
		ts_data->fts_autotest_offset->fts_scap_raw_data_P[i] = 1000;
		ts_data->fts_autotest_offset->fts_scap_raw_data_N[i] = -1000;
	}

	return 0;
}
#else
static int fts_get_threshold_from_img(struct chip_data_ft3683g *ts_data, struct auto_testdata *p_focal_testdata)
{
	int ret = 0;
	int i = 0;
	int item_cnt = 0;
	/*uint8_t * p_print = NULL;*/
	uint32_t *p_item_offset = NULL;
	struct auto_test_header *ph = NULL;
	struct auto_test_item_header *item_head = NULL;
	const struct firmware *limit_fw = NULL;
	u8 fwver = 0xFF;

	ret = fts_read_reg(FTS_REG_FW_VER, &fwver);

	if (fwver >= 0x20) {
		ts_data->use_panelfactory_limit = false;

	} else {
		ts_data->use_panelfactory_limit = true;
	}

	TPD_INFO("%s, use_panelfactory_limit = %d \n", __func__,
	         ts_data->use_panelfactory_limit);

	ts_data->fts_autotest_offset = kzalloc(sizeof(struct fts_autotest_offset),
	                                       GFP_KERNEL);

	limit_fw = (const struct firmware *) p_focal_testdata->fw;
	ph = (struct auto_test_header *)(limit_fw->data);

	p_item_offset = (uint32_t *)(limit_fw->data + 16);

	for (i = 0; i < 8 * sizeof(ph->test_item); i++) {
		if ((ph->test_item >> i) & 0x01) {
			item_cnt++;
		}
	}

	TPD_INFO("%s: total test item = %d \n", __func__, item_cnt);

	TPD_INFO("%s: populating fts_test_offset \n", __func__);

	for (i = 0; i < item_cnt; i++) {
		TPD_INFO("%s: i[%d] \n", __func__, i);
		item_head = (struct auto_test_item_header *)(limit_fw->data + p_item_offset[i]);

		if (item_head->item_limit_type == LIMIT_TYPE_NO_DATA) {
			TPD_INFO("[%d] incorrect item type: LIMIT_TYPE_NO_DATA\n", item_head->item_bit);

		} else if (item_head->item_limit_type == LIMIT_TYPE_TOP_FLOOR_DATA) {
			if (false == ts_data->use_panelfactory_limit) {
				TPD_INFO("test item bit [%d] \n", item_head->item_bit);

				if (item_head->item_bit == TYPE_NOISE_DATA) {
					ts_data->fts_autotest_offset->fts_noise_data_P = (int32_t *)(
					            limit_fw->data + item_head->top_limit_offset);
					ts_data->fts_autotest_offset->fts_noise_data_N = (int32_t *)(
					            limit_fw->data + item_head->floor_limit_offset);
					TPD_INFO("fts_noise_data_P = %p, fts_noise_data_N = %p \n",
					         ts_data->fts_autotest_offset->fts_noise_data_P,
					         ts_data->fts_autotest_offset->fts_noise_data_N);

				} else if (item_head->item_bit == TYPE_RAW_DATA) {
					ts_data->fts_autotest_offset->fts_raw_data_P = (int32_t *)(
					            limit_fw->data + item_head->top_limit_offset);
					ts_data->fts_autotest_offset->fts_raw_data_N = (int32_t *)(
					            limit_fw->data + item_head->floor_limit_offset);
					TPD_INFO("fts_raw_data_P = %p, fts_raw_data_N = %p \n",
					         ts_data->fts_autotest_offset->fts_raw_data_P,
					         ts_data->fts_autotest_offset->fts_raw_data_N);

				} else if (item_head->item_bit == TYPE_UNIFORMITY_DATA) {
					ts_data->fts_autotest_offset->fts_uniformity_data_P = (int32_t *)(
					            limit_fw->data + item_head->top_limit_offset);
					ts_data->fts_autotest_offset->fts_uniformity_data_N = (int32_t *)(
					            limit_fw->data + item_head->floor_limit_offset);
					TPD_INFO("fts_uniformity_data_P = %p, fts_uniformity_data_N = %p \n",
					         ts_data->fts_autotest_offset->fts_uniformity_data_P,
					         ts_data->fts_autotest_offset->fts_uniformity_data_N);

				} else if (item_head->item_bit == TYPE_PANEL_DIFFER_DATA) {
					ts_data->fts_autotest_offset->fts_panel_differ_data_P = (int32_t *)(
								limit_fw->data + item_head->top_limit_offset);
					ts_data->fts_autotest_offset->fts_panel_differ_data_N = (int32_t *)(
								limit_fw->data + item_head->floor_limit_offset);
					TPD_INFO("fts_panel_differ_data_P = %p, fts_panel_differ_data_N = %p \n",
						 ts_data->fts_autotest_offset->fts_panel_differ_data_P,
						 ts_data->fts_autotest_offset->fts_panel_differ_data_N);
				}

			} else if (true == ts_data->use_panelfactory_limit) {
				TPD_INFO("test item bit [%d] \n", item_head->item_bit);

				if (item_head->item_bit == TYPE_FACTORY_NOISE_DATA) {
					ts_data->fts_autotest_offset->fts_noise_data_P = (int32_t *)(
					            limit_fw->data + item_head->top_limit_offset);
					ts_data->fts_autotest_offset->fts_noise_data_N = (int32_t *)(
					            limit_fw->data + item_head->floor_limit_offset);
					TPD_INFO("fts_noise_data_P = %p, fts_noise_data_N = %p \n",
					         ts_data->fts_autotest_offset->fts_noise_data_P,
					         ts_data->fts_autotest_offset->fts_noise_data_P);

				} else if (item_head->item_bit == TYPE_FACTORY_RAW_DATA) {
					ts_data->fts_autotest_offset->fts_raw_data_P = (int32_t *)(
					            limit_fw->data + item_head->top_limit_offset);
					ts_data->fts_autotest_offset->fts_raw_data_N = (int32_t *)(
					            limit_fw->data + item_head->floor_limit_offset);
					TPD_INFO("fts_raw_data_P = %p, fts_raw_data_P = %p \n",
					         ts_data->fts_autotest_offset->fts_raw_data_P,
					         ts_data->fts_autotest_offset->fts_raw_data_N);

				} else if (item_head->item_bit == TYPE_FACTORY_UNIFORMITY_DATA) {
					ts_data->fts_autotest_offset->fts_uniformity_data_P = (int32_t *)(
					            limit_fw->data + item_head->top_limit_offset);
					ts_data->fts_autotest_offset->fts_uniformity_data_N = (int32_t *)(
					            limit_fw->data + item_head->floor_limit_offset);
					TPD_INFO("fts_uniformity_data_P = %p, fts_uniformity_data_P = %p \n",
					         ts_data->fts_autotest_offset->fts_uniformity_data_P,
					         ts_data->fts_autotest_offset->fts_uniformity_data_N);

				} else if (item_head->item_bit == TYPE_FACTORY_PANEL_DIFFER_DATA) {
					ts_data->fts_autotest_offset->fts_panel_differ_data_P = (int32_t *)(
								limit_fw->data + item_head->top_limit_offset);
					ts_data->fts_autotest_offset->fts_panel_differ_data_N = (int32_t *)(
								limit_fw->data + item_head->floor_limit_offset);
					TPD_INFO("fts_panel_differ_data_P = %p, fts_panel_differ_data_N = %p \n",
						 ts_data->fts_autotest_offset->fts_panel_differ_data_P,
						 ts_data->fts_autotest_offset->fts_panel_differ_data_N);
				}
			}

		} else if (item_head->item_limit_type == LIMIT_TYPE_TOP_FLOOR_RX_TX_DATA) {
			if (false == ts_data->use_panelfactory_limit) {
				TPD_INFO("test item bit [%d] \n", item_head->item_bit);

				if (item_head->item_bit == TYPE_SCAP_CB_DATA) {
					ts_data->fts_autotest_offset->fts_scap_cb_data_P = (int32_t *)(
					            limit_fw->data + item_head->top_limit_offset);
					ts_data->fts_autotest_offset->fts_scap_cb_data_N = (int32_t *)(
					            limit_fw->data + item_head->floor_limit_offset);
					TPD_INFO("fts_scap_cb_data_P = %p, fts_scap_cb_data_P = %p \n",
					         ts_data->fts_autotest_offset->fts_scap_cb_data_P,
					         ts_data->fts_autotest_offset->fts_scap_cb_data_N);

				} else if (item_head->item_bit == TYPE_SCAP_RAW_DATA) {
					ts_data->fts_autotest_offset->fts_scap_raw_data_P = (int32_t *)(
					            limit_fw->data + item_head->top_limit_offset);
					ts_data->fts_autotest_offset->fts_scap_raw_data_N = (int32_t *)(
					            limit_fw->data + item_head->floor_limit_offset);
					TPD_INFO("fts_scap_raw_data_P = %p, fts_scap_raw_data_N = %p \n",
					         ts_data->fts_autotest_offset->fts_scap_raw_data_P,
					         ts_data->fts_autotest_offset->fts_scap_raw_data_N);

				} else if (item_head->item_bit == TYPE_SCAP_CB_WATERPROOF_DATA) {
					ts_data->fts_autotest_offset->fts_scap_cb_data_waterproof_P = (int32_t *)(
					            limit_fw->data + item_head->top_limit_offset);
					ts_data->fts_autotest_offset->fts_scap_cb_data_waterproof_N = (int32_t *)(
					            limit_fw->data + item_head->floor_limit_offset);
					TPD_INFO("fts_scap_cb_data_waterproof_P = %p, fts_scap_cb_data_waterproof_N = %p \n",
					         ts_data->fts_autotest_offset->fts_scap_cb_data_waterproof_P,
					         ts_data->fts_autotest_offset->fts_scap_cb_data_waterproof_N);

				} else if (item_head->item_bit == TYPE_SCAP_RAW_WATERPROOF_DATA) {
					ts_data->fts_autotest_offset->fts_scap_raw_waterproof_data_P = (int32_t *)(
					            limit_fw->data + item_head->top_limit_offset);
					ts_data->fts_autotest_offset->fts_scap_raw_waterproof_data_N = (int32_t *)(
					            limit_fw->data + item_head->floor_limit_offset);
					TPD_INFO("fts_scap_raw_waterproof_data_P = %p, fts_scap_raw_waterproof_data_N = %p \n",
					         ts_data->fts_autotest_offset->fts_scap_raw_waterproof_data_P,
					         ts_data->fts_autotest_offset->fts_scap_raw_waterproof_data_N);
				}

			} else if (true == ts_data->use_panelfactory_limit) {
				TPD_INFO("test item bit [%d] \n", item_head->item_bit);

				if (item_head->item_bit == TYPE_FACTORY_SCAP_CB_DATA) {
					ts_data->fts_autotest_offset->fts_scap_cb_data_P = (int32_t *)(
					            limit_fw->data + item_head->top_limit_offset);
					ts_data->fts_autotest_offset->fts_scap_cb_data_N = (int32_t *)(
					            limit_fw->data + item_head->floor_limit_offset);

				} else if (item_head->item_bit == TYPE_FACTORY_SCAP_RAW_DATA) {
					ts_data->fts_autotest_offset->fts_scap_raw_data_P = (int32_t *)(
					            limit_fw->data + item_head->top_limit_offset);
					ts_data->fts_autotest_offset->fts_scap_raw_data_N = (int32_t *)(
					            limit_fw->data + item_head->floor_limit_offset);

				} else if (item_head->item_bit == TYPE_FACTORY_SCAP_CB_WATERPROOF_DATA) {
					ts_data->fts_autotest_offset->fts_scap_cb_data_waterproof_P = (int32_t *)(
					            limit_fw->data + item_head->top_limit_offset);
					ts_data->fts_autotest_offset->fts_scap_cb_data_waterproof_N = (int32_t *)(
					            limit_fw->data + item_head->floor_limit_offset);

				} else if (item_head->item_bit == TYPE_FACTORY_SCAP_RAW_WATERPROOF_DATA) {
					ts_data->fts_autotest_offset->fts_scap_raw_waterproof_data_P = (int32_t *)(
					            limit_fw->data + item_head->top_limit_offset);
					ts_data->fts_autotest_offset->fts_scap_raw_waterproof_data_N = (int32_t *)(
					            limit_fw->data + item_head->floor_limit_offset);
				}
			}

		} else {
			TPD_INFO("[%d] unknown item type \n", item_head->item_bit);
		}
	}

	ret = 0;
	return ret;
}
#endif

static void fts_print_threshold(struct chip_data_ft3683g *ts_data)
{
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int node_num = tx_num * rx_num;
	int channel_num = tx_num + rx_num;

	TPD_INFO("noise threshold max/min:");
	print_buffer(ts_data->fts_autotest_offset->fts_noise_data_P, node_num, rx_num);
	print_buffer(ts_data->fts_autotest_offset->fts_noise_data_N, node_num, rx_num);

	TPD_INFO("rawdata threshold max/min:");
	print_buffer(ts_data->fts_autotest_offset->fts_raw_data_P, node_num, rx_num);
	print_buffer(ts_data->fts_autotest_offset->fts_raw_data_N, node_num, rx_num);

	TPD_INFO("uniformity threshold max/min:");
	print_buffer(ts_data->fts_autotest_offset->fts_uniformity_data_P, node_num,
	             rx_num);
	print_buffer(ts_data->fts_autotest_offset->fts_uniformity_data_N, node_num,
	             rx_num);

	TPD_INFO("scap cb normal threshold max/min:");
	print_buffer(ts_data->fts_autotest_offset->fts_scap_cb_data_P, channel_num,
	             channel_num);
	print_buffer(ts_data->fts_autotest_offset->fts_scap_cb_data_N, channel_num,
	             channel_num);

	TPD_INFO("scap cb waterproof threshold max/min:");
	print_buffer(ts_data->fts_autotest_offset->fts_scap_cb_data_waterproof_P,
	             channel_num, channel_num);
	print_buffer(ts_data->fts_autotest_offset->fts_scap_cb_data_waterproof_N,
	             channel_num, channel_num);

	TPD_INFO("scap rawdata threshold max/min:");
	print_buffer(ts_data->fts_autotest_offset->fts_scap_raw_data_P, channel_num,
	             channel_num);
	print_buffer(ts_data->fts_autotest_offset->fts_scap_raw_data_N, channel_num,
	             channel_num);

	TPD_INFO("scap rawdata waterproof threshold max/min:");
	print_buffer(ts_data->fts_autotest_offset->fts_scap_raw_waterproof_data_P,
	             channel_num, channel_num);
	print_buffer(ts_data->fts_autotest_offset->fts_scap_raw_waterproof_data_N,
	             channel_num, channel_num);
}

int fts_test_entry(struct chip_data_ft3683g *ts_data,
                   struct auto_testdata *focal_testdata)
{
	int ret = 0;
	int i = 0;
	int node_num = ts_data->hw_res->tx_num * ts_data->hw_res->rx_num;
	int channel_num = ts_data->hw_res->tx_num + ts_data->hw_res->rx_num;

	FTS_TEST_FUNC_ENTER();
	FTS_TEST_SAVE_ERR("FW_VER:0x%02x, tx_num:%d, rx_num:%d\n", ts_data->fwver,
	                  ts_data->hw_res->tx_num, ts_data->hw_res->rx_num);

	/*set node valid*/
	for (i = 0; i < node_num; i++) {
		ts_data->node_valid[i] = 1;
	}

	for (i = 0; i < channel_num; i++) {
		ts_data->node_valid_sc[i] = 1;
	}

#ifdef FTS_KIT
	ret = fts_get_threshold(ts_data, focal_testdata);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("get threshold fail,ret=%d\n", ret);
		return 0xFF;
	}
#else
	ret = fts_get_threshold_from_img(ts_data, focal_testdata);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("get threshold from img fail,ret=%d\n", ret);
		return 0xFF;
	}
#endif

	fts_print_threshold(ts_data);

	ret = enter_factory_mode(ts_data);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("failed to enter factory mode,ret=%d\n", ret);
		ret = 0xFF;
		goto test_err;
	}

	ret = fts_test_read_reg(0x14, &ts_data->fre_num);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read fre_num fails");
		goto test_err;
	}
	TPD_INFO("fre_num:%d", ts_data->fre_num);

	ret = get_channel_num(ts_data);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("check channel num fail,ret=%d\n", ret);
		ret = 0xFF;
		goto test_err;
	}

	FTS_TEST_FUNC_EXIT();
	return ret;

test_err:
	enter_work_mode();
	FTS_TEST_FUNC_EXIT();
	return ret;
}
