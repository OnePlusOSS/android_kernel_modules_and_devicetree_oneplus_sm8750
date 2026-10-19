#ifndef __LINUX_GT9916_CORE_H__
#define __LINUX_GT9916_CORE_H__
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/jiffies.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>

#define FRAME_HEAD_LEN				16
#define FRAME_PROTOCOL_OLD			1
#define FRAME_PROTOCOL_NEW			2

enum gt_chip_type {
	GT9966 = 0,
	GT9916,
	GT9926,
	GT9976
};


struct gt_board_data {
	enum gt_chip_type chip_type;
	unsigned int frame_addr;
	unsigned int ges_addr;
	unsigned int cmd_addr;
};

struct gt_core {
	struct mutex bus_mutex;
	u8 *bus_tx_buf;
	u8 *bus_rx_buf;
	struct bus_operations *bus_ops;
	const struct goodix_thp_hw_ops *hw_ops;
	struct gt_board_data board_data;
	struct miscdevice tool_misc_dev;
	char tool_misc_dev_name[32];
	u8 protocol_type;
	u32 frame_len;
};

enum gesture_id {
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

/* gesture type */
#define GOODIX_GESTURE_CMD_ENABLE        0xA6
#define GOODIX_GESTURE_CMD_DISABE        0xA7
#define GOODIX_ALL_GESTURE_ENABLE        0xFFFFFFFF
#define GOODIX_LEFT2RIGHT_SWIP           0xAA
#define GOODIX_RIGHT2LEFT_SWIP           0xBB
#define GOODIX_UP2DOWN_SWIP              0xAB
#define GOODIX_DOWN2UP_SWIP              0xBA
#define GOODIX_DOU_TAP                   0xCC
#define GOODIX_DOU_SWIP                  0x48
#define GOODIX_SINGLE_TAP                0x4C
#define GOODIX_PENDETECT                 0xDD
#define GOODIX_UP_VEE                    0x76
#define GOODIX_DOWN_VEE                  0x5E
#define GOODIX_LEFT_VEE                  0x3E
#define GOODIX_RIGHT_VEE                 0x63
#define GOODIX_CIRCLE_GESTURE            0x6F
#define GOODIX_M_GESTRUE                 0x6D
#define GOODIX_W_GESTURE                 0x77

/* gesture type for fingerprint start */
#define GOODIX_COMPLEX_SMALL_AREA        0x41
#define GOODIX_SIMPLE_AREA               0x42
#define GOODIX_RELEASE_HOLD              0x44
#define GOODIX_FINGERPRINT_ERR_REPORT   0x80
#define GOODIX_TOUCH_HOLD_EARLY_DOWN 0x81

/* gesture type for fingerprint end */

#define GTP_SENSOR_ID_DEFAULT            255
#define GTP_SENSOR_ID_ERR                0

enum _FTS_FP_ERROR_TYPE {
	FTS_FINGERPRINT_DOWN_BEFORE_FP_ENABLE = 0,
	FTS_FINGERPRINT_X_Y_NOT_MATCH = 0x08,
	FTS_FINGERPRINT_OUT_MOVE_IN = 0x04,
	FTS_ANOTHER_FINGER_ON_NON_FP_ZONE = 0x02,
	FTS_FINGERPRINT_AREA_NOT_MATCH = 0x01,
};
/* power off sequence delay (ms) */
#define POWER_OFF_RESET_DELAY_MS		 5
#define POWER_OFF_AVDD_DELAY_MS			 10
#define POWER_OFF_VDDI_DELAY_MS			 10
#define POWER_OFF_BUS_DELAY_MS			 20

struct goodix_thp_hw_ops {
	int (*read)(struct gt_core *ts_data, unsigned int addr, unsigned char *data, unsigned int len);
	int (*write)(struct gt_core *ts_data, unsigned int addr, unsigned char *data, unsigned int len);
	int (*send_cmd)(struct gt_core *ts_data, u8 cmd, u16 data);
	int (*reset)(struct gt_core *ts_data, u32 delay_ms);
};


/* communication interface */
//static int goodix_spi_read(struct gt_core *ts_data, unsigned int addr, unsigned char *data, unsigned int len);
//static int goodix_spi_write(struct gt_core *ts_data, unsigned int addr, unsigned char *data, unsigned int len);

int gt_tools_init(struct gt_core *core_data);
void gt_tools_exit(struct gt_core *core_data);


#endif /* __LINUX_GT9916_CORE_H__ */
