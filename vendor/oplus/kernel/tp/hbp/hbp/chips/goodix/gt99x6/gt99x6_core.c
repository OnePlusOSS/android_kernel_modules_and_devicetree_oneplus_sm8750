#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#include <linux/mm.h>
#include <linux/version.h>
#include <linux/proc_fs.h>

#include "gt99x6_core.h"

#include "../../../hbp_core.h"
#include "../../../hbp_spi.h"
#include "../../../utils/debug.h"
#include "../../chips_healthinfo_report.h"

#define PLATFORM_DRIVER_NAME "gt99x6"
#define KO_VERSION            "v5.0.0.2"

/*****************************************************************************
* Static variabls
*****************************************************************************/
struct gt_core *g_gt;

#define SPI_TRANS_PREFIX_LEN    1
#define REGISTER_WIDTH          4
#define SPI_READ_DUMMY_LEN      4
#define SPI_READ_PREFIX_LEN  (SPI_TRANS_PREFIX_LEN + REGISTER_WIDTH + SPI_READ_DUMMY_LEN)
#define SPI_WRITE_PREFIX_LEN (SPI_TRANS_PREFIX_LEN + REGISTER_WIDTH)

/* flag */
#define SPI_WRITE_FLAG  			0xF0
#define SPI_READ_FLAG   			0xF1
#define MASK_8BIT					0xFF

#define MOVE_8BIT					8
#define MOVE_16BIT					16
#define MOVE_24BIT					24


/* length*/
#define GOODIX_MASK_ID_LEN				6
#define GOODIX_MAX_IC_INFO_LEN			300
#define GOODIX_PAGE_SIZE				(8 * 1024 + 16)
#define GOODIX_CMD_LEN_9897				6
#define GOODIX_FRAME_LEN_MAX			2048
#define GOODIX_FRAME_MAX			    2400

#define DEBUG_BUF_LEN					160
#define DEBUG_AFE_DATA_BUF_LEN			20
#define DEBUG_AFE_DATA_BUF_OFFSET		DEBUG_AFE_DATA_BUF_LEN

#define MAX_GT_IRQ_DATA_LENGTH           90       /*irq data(points,key,checksum) size read from irq*/

//addr
#define GOODIX_IC_INFO_ADDR		        0x10070
#define GOODIX_FRAME_ADDR_GT9916		0x10400
#define GOODIX_FRAME_ADDR_GT9966		0x1036C



#define GOODIX_CMD_LEN_9897				6

/* times*/
#define VERSION_INFO_READ_RETRY				5
#define SEND_COMMAND_RETRY				6
#define CHECK_COMMAND_RETRY				5

/* command ack info */
#define CMD_ACK_BUFFER_OVERFLOW				0x01
#define CMD_ACK_CHECKSUM_ERROR				0x02
#define CMD_ACK_BUSY					0x04
#define CMD_ACK_OK					0x80
#define CMD_ACK_IDLE					0xFF
#define CMD_ACK_UNKNOWN					0x00
#define CMD_ACK_ERROR					(-1)


/* others */
#define RESET_LOW_DELAY_US				2000
#define SEND_COMMAND_END_DELAY				10
/*others*/
#define WAIT_STATE					0
#define WAKEUP_STATE					1
#define GET_FRAME_BLOCK_MODE			        1
#define GET_FRAME_NONBLOCK_MODE				0
#define IRQ_ENABLE_FLAG					1
#define IRQ_DISABLE_FLAG				0
#define GESTURE_DOUBLE_CLICK				0
#define GESTURE_SINGLE_CLICK				1

#define RAWDATA_DISABLE					0
#define RAWDATA_ENABLE					1
#define TOUCH_DATA_DISABLE				0
#define TOUCH_DATA_ENABLE				1
#define SLEEP_MODE						0
#define GOODIX_BE_MODE  				0
#define GOODIX_LE_MODE  				1

static int goodix_spi_read(struct gt_core *ts_data, unsigned int addr, unsigned char *data, unsigned int len);
static int goodix_spi_write(struct gt_core *ts_data, unsigned int addr, unsigned char *data, unsigned int len);


#pragma pack(push, 1)
struct thp_goodix_cmd {
		union {
			struct {
				u8 state;
				u8 ack;
				u8 len;
				u8 cmd;
				u8 data[2];
				u16 checksum;
			};
		u8 buf[8];
	};
};

struct goodix_version_info {
	u8 rom_pid[6];               /* rom PID */
	u8 rom_vid[3];               /* Mask VID */
	u8 rom_vid_reserved;
	u8 patch_pid[8];              /* Patch PID */
	u8 patch_vid[4];              /* Patch VID */
	u8 patch_vid_reserved;
	u8 sensor_id;
	u8 reserved[2];
	u16 checksum;
};
#pragma pack(pop)

u16 checksum16_cmp(u8 *data, u32 size, int mode)
{
		u16 cal_checksum = 0;
		u16 checksum;
		u32 i;

		if (size < sizeof(u16)) {
			hbp_err("inval size %d", size);
			return 1;
		}

		for (i = 0; i < size - sizeof(u16); i++)
			cal_checksum += data[i];
		if (mode == GOODIX_BE_MODE)
			checksum = (data[size - sizeof(u16)] << MOVE_8BIT) +
						data[size - 1];
		else
			checksum = data[size - sizeof(u16)] +
						(data[size - 1] << MOVE_8BIT);

		return cal_checksum == checksum ? 0 : 1;
}

void gt_hw_power_off(struct gt_core *gt)
{
    hbp_dev_power_type_ctrl(gt, POWER_RESET, false);
    msleep(POWER_OFF_RESET_DELAY_MS);
    hbp_dev_power_type_ctrl(gt, POWER_AVDD, false);
    msleep(POWER_OFF_AVDD_DELAY_MS);
    hbp_dev_power_type_ctrl(gt, POWER_VDDI, false);
    msleep(POWER_OFF_VDDI_DELAY_MS);
    hbp_dev_power_type_ctrl(gt, POWER_BUS, false);
    msleep(POWER_OFF_BUS_DELAY_MS);
}

static int gt_chip_enable_hbp_mode(void *priv, bool en)
{
	return 0;
}

enum {
	FRAME_FLAG_RESET = 1,
	FRAME_FLAG_UPLINK
};

static int gt_read_frame_head(void *priv, u8 *flag)
{
	struct gt_core *ts_data = (struct gt_core *)priv;
	u8 frame_head[FRAME_HEAD_LEN];

	ts_data->frame_len = 0;
	goodix_spi_read(ts_data, ts_data->board_data.frame_addr, frame_head, sizeof(frame_head));
	if (frame_head[0] != 0x80) {
		hbp_err("invalid frame_head[0]:0x%02x\n", frame_head[0]);
		hbp_err("invalid frame_head, %*ph\n", 16, frame_head);
		hbp_dev_healthinfo_report(ts_data, CHIPS_REPORT_GOODIX_FRAME_INVALID_HEAD);
		return -1;
	}

	if (ts_data->protocol_type == 0) {
		if (checksum16_cmp(frame_head, 14, GOODIX_LE_MODE) == 0) {
			ts_data->protocol_type = FRAME_PROTOCOL_OLD;
		} else if (checksum16_cmp(frame_head, FRAME_HEAD_LEN, GOODIX_LE_MODE) == 0) {
			ts_data->protocol_type = FRAME_PROTOCOL_NEW;
		} else {
			hbp_err("frame head checksum error, %*ph\n", FRAME_HEAD_LEN, frame_head);
			hbp_dev_healthinfo_report(ts_data, CHIPS_REPORT_GOODIX_FRAME_CHECKSUM_FAIL);
			return -1;
		}
	}

	if (ts_data->protocol_type == FRAME_PROTOCOL_OLD) {
		if (checksum16_cmp(frame_head, 14, GOODIX_LE_MODE)) {
			hbp_err("frame head checksum error, %*ph\n", 14, frame_head);
			hbp_dev_healthinfo_report(ts_data, CHIPS_REPORT_GOODIX_FRAME_CHECKSUM_FAIL);
			return -1;
		}
		ts_data->frame_len = (frame_head[7] << 8) | frame_head[6];
		if (frame_head[9] & 0x10)
			*flag = FRAME_FLAG_UPLINK;
		else if (frame_head[9] & 0x80)
			*flag = FRAME_FLAG_RESET;
	} else if (ts_data->protocol_type == FRAME_PROTOCOL_NEW) {
		if (checksum16_cmp(frame_head, FRAME_HEAD_LEN, GOODIX_LE_MODE)) {
			hbp_err("frame head checksum error, %*ph\n", FRAME_HEAD_LEN, frame_head);
			hbp_dev_healthinfo_report(ts_data, CHIPS_REPORT_GOODIX_FRAME_CHECKSUM_FAIL);
			return -1;
		}
		ts_data->frame_len = (frame_head[4] << 8) | frame_head[3];
		if (frame_head[7] & 0x01)
			*flag = FRAME_FLAG_RESET;
	} else {
		hbp_err("unknown protocol type %d", ts_data->protocol_type);
		hbp_dev_healthinfo_report(ts_data, CHIPS_REPORT_GOODIX_FRAME_INVALID_LEN);
		return -1;
	}

	return 0;
}

static int gt_chip_get_irq_reason(void *priv, enum irq_reason *reason)
{
	struct gt_core *gt = (struct gt_core *)priv;
	u32 ges_addr = gt->board_data.ges_addr;
	u8 buf[sizeof(struct goodix_version_info)] = {0};
	struct goodix_version_info *fw_ver = (struct goodix_version_info *)buf;
	u8 flag = 0;
	int ret;

	*reason = IRQ_REASON_NORMAL;

	goodix_spi_read(gt, 0x10014, buf, sizeof(buf));

	if (memcmp(fw_ver->patch_pid, "GEST", 4) == 0) { // gesture mode
		goodix_spi_read(gt, ges_addr, buf, 1);
		if (buf[0] & 0x20) {
			*reason = IRQ_REASON_NORMAL; // run get gesture flow
		} else {
			*reason = IRQ_REASON_GESTURE_DIFF; // run get frame flow
			ret = gt_read_frame_head(gt, &flag);
			if (ret < 0) {
				return ret;
			}
			if (flag == FRAME_FLAG_UPLINK) {
				*reason = IRQ_REASON_UPLINK_REPORT;
				hbp_info("irq reason: uplink frame\n");
			}
		}
	} else { // normal mode
		ret = gt_read_frame_head(gt, &flag);
		if (ret < 0) {
			return ret;
		}
		if (flag == FRAME_FLAG_RESET) {
			goodix_spi_read(gt, ges_addr, buf, 8);
			if (buf[3] == 0x04) {
				hbp_dev_healthinfo_report(gt, CHIPS_REPORT_GOODIX_RESET_ERROR_FAIL);
			} else if (buf[3] == 0x08) {
				hbp_dev_healthinfo_report(gt, CHIPS_REPORT_GOODIX_RESET_WATCHDOG_FAIL);
			}
			if ((buf[0] == 0x40) && (buf[2] == 0x06) && (buf[3] != 0x01)) {
				hbp_err("tp reset occurred! type[%02x]", buf[3]);
				*reason = IRQ_REASON_RESET_WDT;
			}
			buf[0] = 0;
			goodix_spi_write(gt, ges_addr, buf, 1);
		} else if (flag == FRAME_FLAG_UPLINK) {
			*reason = IRQ_REASON_UPLINK_REPORT;
			hbp_info("irq reason: uplink frame\n");
		}
	}

	return 0;
}

#define GESTURE_DATA_HEAD_LEN				8
#define GESTURE_KEY_DATA_LEN				42
#define GESTURE_FP_ERROR_DATA_LEN			18
static void fhp_read_fod_error_info(u8* gesture_data, u32 data_len)
{
	if (gesture_data == NULL) {
		hbp_info("Error: null pointer input\n");
		return;
	}
	if (data_len < GESTURE_FP_ERROR_DATA_LEN) {
		hbp_info("Error: insufficient data length (%d), need at least GESTURE_FP_ERROR_DATA_LEN bytes\n", data_len);
		return;
	}
	switch (gesture_data[8]) {
	case FTS_FINGERPRINT_AREA_NOT_MATCH:
		hbp_info("TP_FP_ERROR_REPORT:area size: 0x%x\n", gesture_data[9]);
		hbp_info("TP_FP_ERROR_REPORT:FINGERPRINT_AREA_NOT_MATCH\n");
		break;
	case FTS_ANOTHER_FINGER_ON_NON_FP_ZONE:
		hbp_info("TP_FP_ERROR_REPORT:x:0x%x,y:0x%x\n", (gesture_data[11] << 8) + gesture_data[10], (gesture_data[13] << 8) + gesture_data[12]);
		hbp_info("TP_FP_ERROR_REPORT:ANOTHER_FINGER_ON_NON_FP_ZONE\n");
		break;
	case FTS_FINGERPRINT_X_Y_NOT_MATCH:
		hbp_info("TP_FP_ERROR_REPORT:x:0x%x,y:0x%x\n", (gesture_data[15] << 8) + gesture_data[14], (gesture_data[17] << 8) + gesture_data[16]);
		hbp_info("TP_FP_ERROR_REPORT:FINGERPRINT_X_Y_NOT_MATCH\n");
		break;
	case FTS_FINGERPRINT_OUT_MOVE_IN:
		hbp_info("TP_FP_ERROR_REPORT:FINGERPRINT_OUT_MOVE_IN\n");
		break;
	default:
		hbp_info("TP_FP_ERROR_REPORT:unknown fingerprint error type: 0x%02x\n",
			gesture_data[8]);
		break;
	}
}

static int gt_chip_get_gesture(void *priv, struct gesture_info *gesture)
{
	struct gt_core *gt = (struct gt_core *)priv;
	u32 ges_addr = gt->board_data.ges_addr;
	u8 temp_data[GESTURE_KEY_DATA_LEN] = {0};
	u8 clean_data = 0;
	u8 *ges_coor = &temp_data[GESTURE_DATA_HEAD_LEN];

	/* read gesture data */
	goodix_spi_read(gt, ges_addr, temp_data, sizeof(temp_data));
	if (temp_data[0] == 0) {
		hbp_err("invalid gesture head\n");
		//goto re_send_ges_cmd;
		return -1;
	}

	// clean sync flag
	goodix_spi_write(gt, ges_addr, &clean_data, 1);

	/* check gesture data */
	if (checksum16_cmp(temp_data, GESTURE_DATA_HEAD_LEN, GOODIX_LE_MODE)) {
		hbp_err("gesture data head check failed, %*ph\n", GESTURE_DATA_HEAD_LEN, temp_data);
		hbp_dev_healthinfo_report(gt, CHIPS_REPORT_GOODIX_GESTURE_CHECKSUM_FAIL);
		return -1;
	} else if (checksum16_cmp(&temp_data[GESTURE_DATA_HEAD_LEN],
			GESTURE_KEY_DATA_LEN - GESTURE_DATA_HEAD_LEN, GOODIX_LE_MODE)) {
		hbp_err("Gesture data checksum error, %*ph\n", (int)sizeof(temp_data), temp_data);
		hbp_dev_healthinfo_report(gt, CHIPS_REPORT_GOODIX_GESTURE_CHECKSUM_FAIL);
		return -1;
	}

	hbp_info("get gesture type:0x%02x\n", temp_data[4]);

	switch (temp_data[4]) {
	case 0xCC: //double tap
		hbp_info("get gesture event: Double tap\n");
		gesture->type = DoubleTap;
		break;
	case 0x63: // <
		hbp_info("get gesture event: <\n");
		gesture->type = RightVee;
		break;
	case 0x65: // E
		hbp_info("get gesture event: E\n");
		break;
	case 0x6D: // M
		hbp_info("get gesture event: M\n");
		gesture->type = Mgestrue;
		break;
	case 0x76: // V
		hbp_info("get gesture event: V\n");
		gesture->type = UpVee;
		break;
	case 0x5E: // ^
		hbp_info("get gesture event: ^\n");
		gesture->type = DownVee;
		break;
	case 0x3E: // >
		hbp_info("get gesture event: >\n");
		gesture->type = LeftVee;
		break;
	case 0x77: // W
		hbp_info("get gesture event: W\n");
		gesture->type = Wgestrue;
		break;
	case 0x40: // A
		hbp_info("get gesture event: A\n");
		break;
	case 0x66: // F
		hbp_info("get gesture event: F\n");
		break;
	case 0x6F: // O
		hbp_info("get gesture event: O\n");
		gesture->type = Circle;
		break;
	case 0xAA: // R2L
		hbp_info("get gesture event: right to left\n");
		gesture->type = Right2LeftSwip;
		break;
	case 0xBB: // L2R
		hbp_info("get gesture event: left to right\n");
		gesture->type = Left2RightSwip;
		break;
	case 0xBA: // UP
		hbp_info("get gesture event: up\n");
		gesture->type = Down2UpSwip;
		break;
	case 0xAB: // DOWN
		hbp_info("get gesture event: down\n");
		gesture->type = Up2DownSwip;
		break;
	case 0x46: // FP_DOWN
		hbp_info("get gesture event: finger print down\n");
		gesture->type = FingerprintDown;
		break;
	case 0x55: // FP_UP
		hbp_info("get gesture event: finger print up\n");
		gesture->type = FingerprintUp;
		break;
	case 0x4C: // single tap
		hbp_info("get gesture event: single tap\n");
		gesture->type = SingleTap;
		break;
	case 0x48: // double swip
		hbp_info("get gesture event: single tap\n");
		gesture->type = DoubleSwip;
		break;
	case GOODIX_COMPLEX_SMALL_AREA:
		hbp_info("get gesture event: fp_grip_small_area_cnt\n");
		hbp_dev_healthinfo_report(gt, "fp_grip_small_area_cnt");
		gesture->type = FP_GESTURE_HOLD;
		break;
	case GOODIX_SIMPLE_AREA:
		hbp_info("get gesture event: fp_grip_big_area_cnt\n");
		hbp_dev_healthinfo_report(gt, "fp_grip_big_area_cnt");
		gesture->type = FP_GESTURE_HOLD;
		break;
	case GOODIX_RELEASE_HOLD:
		hbp_info("get gesture event: fp_grip_release_cnt\n");
		hbp_dev_healthinfo_report(gt, "fp_grip_release_cnt");
		gesture->type = FP_GESTURE_RELEASE;
		break;
	case GOODIX_TOUCH_HOLD_EARLY_DOWN:
		hbp_info("get gesture event: fp_grip_early_down\n");
		hbp_dev_healthinfo_report(gt, "fp_grip_early_down");
		gesture->type = FingerprintEarlyDown;
		break;
	case GOODIX_FINGERPRINT_ERR_REPORT:
		fhp_read_fod_error_info(temp_data, sizeof(temp_data));
		gesture->type = UnknownGesture;
		break;
	case GOODIX_PENDETECT:
		hbp_info("get gesture event: pendetect\n");
		gesture->type = PenDetect;
		break;
	default:
		hbp_err("not support gesture type 0x%02x\n", temp_data[4]);
		break;
	}

	gesture->Point_start.x = le16_to_cpup((__le16 *)(ges_coor));
	gesture->Point_start.y = le16_to_cpup((__le16 *)(ges_coor + 2));
	gesture->Point_end.x = le16_to_cpup((__le16 *)(ges_coor + 4));
	gesture->Point_end.y = le16_to_cpup((__le16 *)(ges_coor + 6));
	gesture->Point_1st.x = le16_to_cpup((__le16 *)(ges_coor + 16));
	gesture->Point_1st.y = le16_to_cpup((__le16 *)(ges_coor + 18));
	gesture->Point_2nd.x = le16_to_cpup((__le16 *)(ges_coor + 20));
	gesture->Point_2nd.y = le16_to_cpup((__le16 *)(ges_coor + 22));
	gesture->Point_3rd.x = le16_to_cpup((__le16 *)(ges_coor + 24));
	gesture->Point_3rd.y = le16_to_cpup((__le16 *)(ges_coor + 26));
	gesture->Point_4th.x = le16_to_cpup((__le16 *)(ges_coor + 28));
	gesture->Point_4th.y = le16_to_cpup((__le16 *)(ges_coor + 30));

	return 0;
}
static int gt_chip_get_touch_points(void *priv, struct point_info *points)
{

	return 0;
}

/* static u8 gt_chip_get_reset_reason(void *priv)
{
	return 0;
} */

/* static void gt_chip_get_func_position(void *priv)
{

} */


/**
 * goodix_thp_spi_read- read device register through spi bus
 * @ts_data: pointer to device data
 * @addr: register address
 * @data: read buffer
 * @len: bytes to read
 * return: 0 - read ok, < 0 - spi transter error
 */
static int goodix_spi_read(struct gt_core *ts_data, unsigned int addr, unsigned char *data, unsigned int len)
{

	u8 *rx_buf = ts_data->bus_tx_buf;
	u8 *tx_buf = ts_data->bus_rx_buf;
	int ret = 0;

	if (len > PAGE_SIZE - SPI_READ_PREFIX_LEN) {
		hbp_err("spi len too long:%d",len);
		return -1;
	}
	mutex_lock(&ts_data->bus_mutex);
	/*spi_read tx_buf format: 0xF1 + addr(4bytes) + data*/
	tx_buf[0] = SPI_READ_FLAG;
	tx_buf[1] = (addr >> 24) & 0xFF;
	tx_buf[2] = (addr >> 16) & 0xFF;
	tx_buf[3] = (addr >> 8) & 0xFF;
	tx_buf[4] = addr & 0xFF;
	tx_buf[5] = 0xFF;
	tx_buf[6] = 0xFF;
	tx_buf[7] = 0xFF;

	ret = ts_data->bus_ops->spi_sync(ts_data->bus_ops, tx_buf, rx_buf, SPI_READ_PREFIX_LEN + len);
	if (ret < 0) {
		hbp_err("spi transfer error:%d",ret);
		hbp_dev_healthinfo_report(ts_data, CHIPS_REPORT_GOODIX_SPI_READ_FAIL);
		goto exit;
	}
	memcpy(data, &rx_buf[SPI_READ_PREFIX_LEN - 1], len);
exit:
	mutex_unlock(&ts_data->bus_mutex);
	return ret;
}

/**
 * goodix_spi_write- write device register through spi bus
 * @ts_data: pointer to device data
 * @addr: register address
 * @data: write buffer
 * @len: bytes to write
 * return: 0 - write ok; < 0 - spi transter error.
 */
static int goodix_spi_write(struct gt_core *ts_data, unsigned int addr, unsigned char *data, unsigned int len)
{
	u8 *tx_buf = ts_data->bus_tx_buf;
	u8 *rx_buf = ts_data->bus_rx_buf;
	int ret = 0;

	if (len > PAGE_SIZE - SPI_WRITE_PREFIX_LEN) {
		hbp_err("spi len too long:%d",len);
		return -1;
	}
	mutex_lock(&ts_data->bus_mutex);
	tx_buf[0] = SPI_WRITE_FLAG;
	tx_buf[1] = (addr >> 24) & 0xFF;
	tx_buf[2] = (addr >> 16) & 0xFF;
	tx_buf[3] = (addr >> 8) & 0xFF;
	tx_buf[4] = addr & 0xFF;

	memcpy(&tx_buf[SPI_WRITE_PREFIX_LEN], data, len);
	ret = ts_data->bus_ops->spi_sync(ts_data->bus_ops, tx_buf, rx_buf, len + SPI_WRITE_PREFIX_LEN);

	if (ret < 0) {
		hbp_err("spi transfer error:%d",ret);
		hbp_dev_healthinfo_report(ts_data, CHIPS_REPORT_GOODIX_SPI_WRITE_FAIL);
		goto exit;
	}
exit:
	mutex_unlock(&ts_data->bus_mutex);
	return ret;
}

static int gt_spi_sync(void *priv, char *tx, char *rx, int32_t len)
{
	struct gt_core *gt = (struct gt_core *)priv;

	return gt->bus_ops->spi_sync(gt->bus_ops, tx, rx, len);
}

static int gt_chip_get_frame(void *priv, u8 *raw, u32 rawsize)
{
	struct gt_core *ts_data = (struct gt_core *)priv;

	if (!raw) {
		hbp_err("get frame is error");
		return -1;
	}

	if (ts_data->protocol_type == 0) {
		hbp_err("protocol type is unknown");
		return -1;
	}

	if (ts_data->frame_len == 0 || ts_data->frame_len > rawsize) {
		hbp_err("invalid frame len:%d", ts_data->frame_len);
		return -1;
	}

	goodix_spi_read(ts_data, ts_data->board_data.frame_addr, raw, ts_data->frame_len);
	return 0;
}

struct dev_operations gt_ops = {
	.spi_sync = gt_spi_sync,
	.get_frame = gt_chip_get_frame,
	.get_gesture = gt_chip_get_gesture,
	.get_touch_points = gt_chip_get_touch_points,
	.get_irq_reason = gt_chip_get_irq_reason,
	.enable_hbp_mode = gt_chip_enable_hbp_mode,
};

/* hardware opeation funstions */
static const struct goodix_thp_hw_ops hw_spi_ops = {
	.read = goodix_spi_read,
	.write = goodix_spi_write,
};

static void gt_board_init(struct gt_core *gt, const char *ic_name)
{
	if (!ic_name) {
		hbp_err("ic_name error");
		return;
	}

	if (strcmp(ic_name, "gt9966") == 0) {
		hbp_info("Initializing board for GT9966");
		gt->board_data.chip_type = GT9966;
		gt->board_data.frame_addr = GOODIX_FRAME_ADDR_GT9966;
		gt->board_data.ges_addr = 0x10274;
		gt->board_data.cmd_addr = 0x10174;
	} else if (strcmp(ic_name, "gt9916") == 0) {
		hbp_info("Initializing board for GT9916");
		gt->board_data.chip_type = GT9916;
		gt->board_data.frame_addr = GOODIX_FRAME_ADDR_GT9916;
		gt->board_data.ges_addr = 0x10308;
		gt->board_data.cmd_addr = 0x10174;
	} else if (strcmp(ic_name, "gt9926") == 0) {
		hbp_info("Initializing board for GT9926");
		gt->board_data.chip_type = GT9926;
		gt->board_data.frame_addr = 0x10400;
		gt->board_data.ges_addr = 0x10308;
		gt->board_data.cmd_addr = 0x10174;
	} else if (strcmp(ic_name, "gt9976") == 0) {
		hbp_info("Initializing board for GT9976");
		gt->board_data.chip_type = GT9976;
		gt->board_data.frame_addr = 0x10474;
		gt->board_data.ges_addr = 0x10374;
		gt->board_data.cmd_addr = 0x10274;
	} else {
		hbp_err("unknown ic_name %s", ic_name);
	}

	hbp_info("board_data: chip_type=%d, frame_addr=0x%X, ges_addr=0x%X, cmd_addr=0x%X\n",
		gt->board_data.chip_type,
		gt->board_data.frame_addr,
		gt->board_data.ges_addr,
		gt->board_data.cmd_addr);
}

static int gt_dev_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct gt_core *gt;
	struct chip_info info;

	hbp_info("probe start, ko_version %s.\n", KO_VERSION);
	if (!match_from_cmdline(&pdev->dev, &info)) {
		return 0;
	}

	gt = kzalloc(sizeof(struct gt_core), GFP_KERNEL);
	if (!gt) {
		return -ENOMEM;
	}

	mutex_init(&gt->bus_mutex);
	gt->bus_rx_buf = kzalloc(GOODIX_PAGE_SIZE, GFP_KERNEL);
	gt->bus_tx_buf = kzalloc(GOODIX_PAGE_SIZE, GFP_KERNEL);
	if (!gt->bus_tx_buf || !gt->bus_rx_buf) {
		return -ENOMEM;
	}
	gt->hw_ops = &hw_spi_ops;

	gt_board_init(gt, info.ic_name);

	ret = hbp_register_devices(gt, &pdev->dev, &gt_ops, &info, &gt->bus_ops);
	if (ret < 0) {
		hbp_err("failed to register device:%s %d\n", info.vendor, ret);
		goto err_exit;
	}

	gt_tools_init(gt);
	platform_set_drvdata(pdev, gt);
	hbp_info("probe end, ic_name %s.\n", info.ic_name);
	return 0;

err_exit:
	if(gt) {
		kfree(gt->bus_rx_buf);
		kfree(gt->bus_tx_buf);
		kfree(gt);
	}
	return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void gt_dev_remove(struct platform_device *pdev)
#else
static int gt_dev_remove(struct platform_device *pdev)
#endif
{
	struct gt_core *gt = platform_get_drvdata(pdev);

	hbp_info("%s IN\n", __func__);
	gt_tools_exit(gt);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
#else
	return 0;
#endif
}

static void gt_dev_shutdown(struct platform_device *pdev)
{
	struct gt_core *gt = platform_get_drvdata(pdev);

	if (!gt) {
		hbp_err("gt_dev_shutdown: gt is null\n");
		return;
	}

	hbp_info("gt99x6 shutdown start\n");
	gt_hw_power_off(gt);
	hbp_info("gt99x6 shutdown end\n");
}

static const struct of_device_id gt_dt_match[] = {
	{.compatible = "goodix,gt9916", },
	{.compatible = "goodix,gt9966", },
	{.compatible = "goodix,gt9926", },
	{.compatible = "goodix,gt9976", },
	{},
};

static struct platform_driver gt_dev_driver = {
	.probe = gt_dev_probe,
	.remove = gt_dev_remove,
	.shutdown = gt_dev_shutdown,
	.driver = {
		.name = PLATFORM_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(gt_dt_match),
	},
};

static int __init gt_platform_init(void)
{
	return platform_driver_register(&gt_dev_driver);
}

static void __exit gt_platform_exit(void)
{
	platform_driver_unregister(&gt_dev_driver);
}

late_initcall(gt_platform_init);
module_exit(gt_platform_exit);

MODULE_DESCRIPTION("Goodix Driver");
MODULE_LICENSE("GPL v2");
