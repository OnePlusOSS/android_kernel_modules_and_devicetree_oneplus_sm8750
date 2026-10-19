#ifndef __HBP_CORE_H__
#define __HBP_CORE_H__

#include <linux/irqreturn.h>
#include <linux/firmware.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>

#include "hbp_report.h"
#include "hbp_notify.h"
#include "utils/debug.h"
#include "hbp_spi.h"
#include "hbp_tui.h"

#include "hbp_frame.h"
#include "hbp_power.h"
#include "hbp_exception.h"
#include "hbp_healthinfo.h"

#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY)
#include <linux/soc/qcom/panel_event_notifier.h>
#endif

#if defined PAGE_SIZE
#undef PAGE_SIZE
#define PAGE_SIZE 8192
#endif

#define MAX_DEVICES 2
#define MAX_FW_NAME_LEN (128)
#define MAX_TOUCH_POINTS 10
#define TOUCH_BIT_CHECK  0x3FF  //max support 10 point report.using for detect non-valid points

#define EARLY_RESUME_TIMEOUT 3000
#define STATE_SYNC_TIMEOUT 200

#define DRIVER_SYNC_TIMEOUT 50
#define DAEMON_ACK_TIMEOUT 1000

#define FP_FRAME_TIME 10

#define HBP_CORE "hbp_core"
#define HBP_STATAS "hbp-sts"
#define HBP_CTRL "hbp"

#define TOUCH_NAME "touchpanel"

#define HBP_CORE_GROUP 0xD5
#define HBP_CORE_DEVINFO			_IO(HBP_CORE_GROUP, 0x01)
#define HBP_CORE_GET_STATE			_IO(HBP_CORE_GROUP, 0x02)
#define HBP_CORE_STATE_ACK			_IO(HBP_CORE_GROUP, 0x03)
#define HBP_CORE_POWER_IN_SLEEP		_IO(HBP_CORE_GROUP, 0x04)
#define HBP_CORE_GET_GESTURE_COORD	_IO(HBP_CORE_GROUP, 0x05)

#define STATE_BIT_IFP_DOWN (1 << 0)

#define SMART_GESTURE_THRESHOLD 0x0A
#define SMART_GESTURE_LOW_VALUE 0x05

#define FP_GRIP_ENABLE           1
#define FP_GRIP_DISABLE_TIMEOUT  2
#define FP_GRIP_DISABLE          0

/* bit operation */
#define SET_BIT(data, flag) ((data) |= (flag))
#define CLR_BIT(data, flag) ((data) &= ~(flag))
#define CHK_BIT(data, flag) ((data) & (flag))

enum {
	DRV_NOREADY = 1,
	DRV_READY,
	DRV_INT,
	DRV_ACK,
};

union rez {
	struct {
		int32_t x;
		int32_t y;
	};
	struct {
		int32_t tx;
		int32_t rx;
	};
	struct {
		int32_t min;
		int32_t max;
	};
};

#define MAX_NAME_LEN		(64)

struct device_info {
	char project[MAX_NAME_LEN];
	int8_t panels_expect;
	int8_t panels_attached;
	struct {
		int id;
		char ic_name[MAX_NAME_LEN];
		char vendor[MAX_NAME_LEN];
		int max_x;
		int max_y;
	} device[MAX_DEVICES];
};

union usr_data {
	int64_t val;

	struct {
		void __user *tx;
		void __user *rx;
		int32_t len;
	} rw;

	struct {
		void __user *data;
		size_t size;
	} frame;

	struct {
		uint8_t state;
		int x;
		int y;
		int touch_early_down_flag;
		long is_touch_fp_area_Cnt;
		int tp_firmware_time;
	} ifp;

	struct power_sequeue sq[MAX_POWER_SEQ];

	struct {
		uint8_t mode;
		uint8_t bits_per_word;
		int speed;
	} spi_setup;

	struct {
		bool filmed;
		int level;
		bool trusty;
	} film;

	struct {
		void __user *info;
		size_t info_size;
	} health_info;
};

struct chip_info {
	const char *ic_name;
	const char *vendor;
};

struct panel_hw {
	int irq_gpio;  /*irq gpio*/

	/*for power contrl*/
	struct regulator *avdd_reg;  /*power avdd 2.8-3.3v*/
	union rez avdd_volt;

	struct regulator *vddi_reg;	 /*power vddi 1.8v*/
	union rez vddi_volt;

	union rez resolution;

	/*pinctrl*/
	struct pinctrl *pctrl;
	struct pinctrl_state *reset_active;
	struct pinctrl_state *reset_idle;
	struct pinctrl_state *avdd_on;
	struct pinctrl_state *avdd_off;
	struct pinctrl_state *vddi_on;
	struct pinctrl_state *vddi_off;
	struct pinctrl_state *resume;
	struct pinctrl_state *suspend;
};

/*suspend or resume state*/
enum status {
	TP_EARLY_SUSPEND = 1,
	TP_SUSPEND,
	TP_EARLY_RESUMED,
	TP_RESUMED
};

struct Coordinate {
	int x;
	int y;
};

enum gesture_type {
	UnknownGesture = 0,
	DoubleTap,
	UpVee,
	DownVee,
	LeftVee,
	RightVee,
	Circle,
	DoubleSwip,			// ||
	Left2RightSwip,
	Right2LeftSwip,
	Up2DownSwip,
	Down2UpSwip,
	Mgestrue,
	Wgestrue,
	FingerprintDown,
	FingerprintUp,
	SingleTap,
	Heart,
	PenDetect,
	SGesture,
	FingerprintEarlyDown,
	FP_GESTURE_HOLD,
	FP_GESTURE_RELEASE,
};

struct point_info {
	uint16_t x;
	uint16_t y;
	uint16_t z;
	uint8_t  width_major;
	uint8_t  touch_major;
	uint8_t  status;
	uint8_t  tx_press;
	uint8_t  rx_press;
	uint8_t  tx_er;
	uint8_t  rx_er;
};

struct gesture_info {
	enum gesture_type type;
	uint32_t clockwise;
	struct Coordinate Point_start;
	struct Coordinate Point_end;
	struct Coordinate Point_1st;
	struct Coordinate Point_2nd;
	struct Coordinate Point_3rd;
	struct Coordinate Point_4th;
	uint8_t id;
	int tp_firmware_time;
};

struct dev_operations {
	int (*spi_write)(void *priv, void *tx, int32_t len);
	int (*spi_read)(void *priv, char *rx, int32_t len);
	int (*spi_sync)(void *priv, char *tx, char *rx, int32_t len);
	int (*spi_set_para)(void *priv, uint8_t mode, uint8_t bits_per_word, int speed);
	int (*spi_get_para)(void *priv, uint8_t *mode, uint8_t *bits_per_word, int *speed);
	int (*get_frame)(void *priv, uint8_t *buf, uint32_t size);
	int (*get_gesture)(void *priv, struct gesture_info *gesture);
	int (*get_touch_points)(void *priv, struct point_info *points);
	int (*get_irq_reason)(void *priv, enum irq_reason *reason);
	int (*enable_hbp_mode)(void *priv, bool en);
};

struct debug_cfg {
	bool report_gesture_frm;
};

struct hbp_device {
	struct dev_operations *dev_ops;
	struct device *dev;
	struct device *bdev;  /*bus device, etc i2c or spi*/

	struct input_dev *i_dev;   /*touch input device*/
	struct input_dev *p_dev;   /*pencil input device*/

	int id;
	struct panel_hw hw;
	uint16_t state;
	int errReason;

	/*spi write or read buffer*/
	char *_wr;
	char *_rd;

	int irq;
	uint32_t irq_flags;
	bool irq_freed;
	bool irq_enabled;

	struct mutex mlock;
	struct mutex mreport;
	struct mutex mifp;

	/*stat*/
	struct wait_queue_head sts_event;
	int sts_ack;
	struct wait_queue_head drv_event;
	int drv_ack;

	/*fp*/
	int pre_fpstate;
	bool screenoff_ifp;

	int touch_early_down_flag;
	long is_touch_fp_area_cnt;
	ktime_t touch_fp_area_time;
	ktime_t fp_down_time;

	struct frame_queue frame_queue;

	/*callback from panel*/
	struct notifier_block fb_notif;

	void (*panel_cb)(hbp_panel_event event, struct hbp_device *hbp_dev);
	void *priv;

	wait_queue_head_t poll_wait;

#if IS_ENABLED(CONFIG_GH_ARM64_DRV)
	enum hbp_env env;
	struct hbp_vm_info *vm_info;
#endif

	struct debug_cfg debug;
	struct monitor_data monitor_data;

	bool up_status;
	int touch_report_num;
	int last_width_major;
	int last_touch_major;
	int irq_slot;           /*debug use, for print all finger's first touch log*/

	/*feature*/
	bool frame_insert_support;
	union touch_time top_irq_frame_tv;

	bool pen_support;
	bool create_with_power_on_support;
	char clk_name[16];
	struct clk *pen_ck;
	/* edge grip for fingerprint */
	bool fp_grip_support;
	atomic_t fp_grip_hold; /* shared by IRQ writer and resume reader, use atomic to avoid TOCTOU */
	int fp_grip_enable;
};

struct device_state {
	uint16_t id:2;
	uint16_t state:4;
	uint16_t sync:2;
	uint16_t value:8;
};

#define STATE_WAITQ		(1)
#define STATE_WAKEUP	(2)

#define ACK_WAITQ		(3)
#define ACK_WAKEUP		(4)

struct hbp_core {
	struct device *dev;

	const char *prj;
	struct hbp_device *devices[MAX_DEVICES];
	int active_id;

	int boot_mode;
	bool power_in_sleep;

	struct device_state states[MAX_DEVICES];
	struct wait_queue_head state_event;
	int state_st;

	struct wait_queue_head ack_event;
	int state_ack;

	struct class *cls;
	struct device *cdev;
	int major;

	struct mutex gesture_mtx;
	struct gesture_info gesture_info;
	struct device_info dev_info;
	struct wakeup_source *ws;

#if IS_ENABLED(CONFIG_GH_ARM64_DRV)
	struct hbp_vm_mem *tui_mem[2*MAX_DEVICES];
#endif

	bool in_hbp_mode;
	struct exception_data    exception_data; /*exception_data monitor data*/

	/* workqueue for state notify */
	struct workqueue_struct *state_notify_wq;
	struct work_struct state_notify_work;
	struct mutex state_notify_mtx;  /* protect state_notify_id, state_notify_event and states[] */
	int state_notify_id;
	hbp_panel_event state_notify_event;
};

extern int hbp_exception_report(hbp_excep_type excep_tpye,
		void *summary, unsigned int summary_size);
extern int hbp_register_devices(void *priv,
				struct device *dev,
				struct dev_operations *dev_ops,
				struct chip_info *chip,
				struct bus_operations **bus_ops);
extern int hbp_unregister_devices(void *priv);
extern bool match_from_cmdline(struct device *dev, struct chip_info *info);
extern void hbp_set_irq_wake(struct hbp_device *hbp_dev, bool wake);
extern void hbp_dev_power_type_ctrl(void *priv, enum power_type type, bool en);
extern void hbp_dev_healthinfo_report(void *priv, char *report);
/*
#if 1
request_firmware_select()
{
	hbp_err("empty function\n");
	return -ENODEV;
};

#endif
*/

#endif /*__HBP_CORE_H__*/
