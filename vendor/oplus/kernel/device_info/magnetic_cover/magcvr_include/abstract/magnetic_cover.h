#ifndef __MAGCVR_CORE_H__
#define __MAGCVR_CORE_H__

#include <linux/atomic.h>
#include <linux/ioctl.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/init.h>
#include <linux/hrtimer.h>
#include <linux/timer.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/uaccess.h>
#include <linux/syscalls.h>
#include <linux/input/mt.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/ktime.h>

#if IS_ENABLED(CONFIG_OPLUS_MAGCVR_NOTIFY)
#include "magtransfer/magcvr_notify.h"
#endif
#define NAME_UNKNOW 255

typedef enum magcvr_hardware_name {
	MAGCVR_MXM1120 = 0,
	MAGCVR_AK09973,
	MAGCVR_UNKNOW = NAME_UNKNOW,
} hardware_name;

#define MAX_SQRT_TIME                       1024
enum MAGCVR_STATUS_PARA {
	MAGNETIC_COVER_INPUT_FAR  = 0,
	MAGNETIC_COVER_INPUT_NEAR = 1,
	MAGNETIC_COVER_IIC_FAIL = 100,
	MAGNETIC_COVER_IC_FAIL  = 101,
};

#define MAGNETIC_COVER_INPUT_DEVICE "oplus,magnetic_cover"

static int v_adjust = 2000;
module_param(v_adjust, int, 0644);

/* power set*/
#define POWER_ENABLE  1
#define POWER_DISABLE 0

#define VDD_MIN_UV     (2700000)
#define VDD_MAX_UV     (3600000)
#define VDD_DEFAULT_UV (3000000)

#define VIO_MIN_UV     (1650000)
#define VIO_MAX_UV     (3600000)
#define VIO_DEFAULT_UV (1800000)

#define MAG_CVR_TAG         "magnetic_cover"
#define MAG_CVR_DEBUG_TAG   "magnetic_cover debug"

#define INPUT_BUF_SIZE       32

// #define MAGCVR_LOG_DEBUG_ON 1
bool debug_enable = 0;
int  magcvr_index = 0;

#define MAG_CVR_DEBUG(fmt, args...)\
	do{\
		if (debug_enable) \
		pr_err("[%s_%d][%s]DEBUG->"fmt, MAG_CVR_DEBUG_TAG, magcvr_index, __func__, ##args); \
	}while(0)

#define MAG_CVR_ERR(fmt, args...)\
	pr_err("[%s_%d][%s]ERR!!->"fmt, MAG_CVR_DEBUG_TAG, magcvr_index, __func__, ##args)

#define MAG_CVR_LOG(fmt, args...)\
	pr_err("[%s_%d][%s]->"fmt, MAG_CVR_TAG, magcvr_index, __func__, ##args)

// define name
#define M_3P0_VOLT    "vdd_3v0_volt"
#define M_3P0_NAME    "vdd_3v0"
#define M_3P0_DEFAULT 3008000
#define M_1P8_VOLT    "vcc_1v8_volt"
#define M_1P8_NAME    "vcc_1v8"
#define M_1P8_DEFAULT 1800000

#define M_VOLT_LOAD   200000

#define M_IRQ         "mag_irq_gpio"
#define M_DELAY_WORK  "magcvr_delaywork_func"
#define M_INTERRUPT   "magnetic_cover_irq"

#define M_DEF_SETP      50
#define M_SHORT_MAX     32767
#define m_int2short(value) ((value) >= M_SHORT_MAX ? M_SHORT_MAX : (value))
#define M_INT_MAX       2147483647
#define m_long2int(value) ((value) >= M_INT_MAX ? M_INT_MAX : (value))

#define M_FAR_THRESHOLD 150
#define M_FAR_NOISE     125
#define M_FAR_MAX_TH    2000

#define INIT_PROBE_ERR  255

#define GET_DATA_RETRY  5
#define GET_DATA_TIMN   50
#define NOISE_STEP      100
#define M_INDEX         0
#define MAGCVR_STATS_INDEX_MAX 2
#define MAG_CVR_SAMPLE_COUNT 10
#define DELAY_TIME      100

#define MAGCVR_CHECK_HRTIMER_MS        300000
#define MAGCVR_CHECK_HRTIMER_S         5
#define MS_TO_NS                       1000000
#define DEBOUNCE_TIME                  300
#define MAX_DEBOUNCE_CNT               6
#define CHECK_HRTIMER_PARAMS           4

#define DEBOUNCE_TIME_NAME       "debounce_time"
#define MAX_DEBOUNCE_CNT_NAME    "max_debounce_cnt"
#define DEBOUNCE_TIME_S_NAME     "debounce_time_s"
#define DEBOUNCE_TIME_MS_NAME    "debounce_timer_ms"

enum M_IRQ_TYPE {
	EDGE_DOWN = 2,
	EDGE_DOUBLE = 4,
	LOW_LEVEL = 8,
};

enum M_POS_NEG {
	M_POSITIVE =  0,
	M_NEGATIVE =  1,
	M_BILATERAL = 2,
};

// create proc
#define MAGCVR_NAME_SIZE_MAX 25
#define CHMOD                0666
#define PROC_MAGCVR          "magnetic_cover"
typedef struct {
	char *name;
	umode_t mode;
	struct proc_dir_entry *node;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,10,0)
	const struct proc_ops *fops;
#else
	const struct file_operations *fops;
#endif
	void *data;
	bool is_created;/*proc node is creater or not*/
	bool is_support;/*feature is supported or not*/
} magcvr_proc_node;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
#define PDE_DATA pde_data
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
#define DECLARE_PROC_OPS(name, open_func, read_func, write_func, release_func) \
    static const struct proc_ops name = { \
                        .proc_open    = open_func, \
                        .proc_write   = write_func, \
                        .proc_read    = read_func, \
                        .proc_release = release_func, \
                        .proc_lseek   = default_llseek, \
                        }
#else
#define CREATE_PROC_OPS(name, open_func, read_func, write_func, release_func) \
                        static const struct file_operations name = { \
                        .open  = open_func,      \
                        .write = write_func,     \
                        .read  = read_func,      \
                        .release = release_func, \
                        .owner = THIS_MODULE,    \
                        }
#endif

static inline void magcvr_kfree(void **mem)
{
    if (*mem != NULL) {
    kfree(*mem);
    *mem = NULL;
    }
}

// irq trigger tyupe

#define M_IRQ_TYPE_NONE           0x00000000    /* nspecified type */
#define M_IRQ_TYPE_EDGE_RISING    0x00000001    /* Rising edge trigger */
#define M_IRQ_TYPE_EDGE_FALLING   0x00000002    /* Falling edge trigger */
#define M_IRQ_TYPE_EDGE_BOTH      (M_IRQ_TYPE_EDGE_FALLING | M_IRQ_TYPE_EDGE_RISING)
#define M_IRQ_TYPE_LEVEL_HIGH     0x00000004    /* High-level trigger */
#define M_IRQ_TYPE_LEVEL_LOW      0x00000008    /* Low-level trigger */
#define M_IRQ_TYPE_SENSE_MASK     0x0000000f    /* Mask of the above */
#define M_IRQ_TYPE_PROBE          0x00000010    /* Probing in progress */

typedef enum magcvr_power_type {
    M_1P8 = 0,
    M_3P0 = 1,
} power_type;

#define POWER_ENABLE  1
#define POWER_DISABLE 0

// healthinfo start

#define ERR_MAG_REG_MAX_CNT 10
#define CAL_OFFSET_MAX_CNT  3
#define CHECK_SEPARATOR ":"

/********************************************************/
typedef enum healthinfo_opt {
    OPT_DEBUG_ON  = 0,
    OPT_IIC_READ  = 1,
    OPT_IIC_WRITE = 2,
    OPT_CHIP_ERR  = 3,
    OPT_NEAR_FAR  = 4,
    OPT_GET_TIME  = 5,
    OPT_CLEAR_ALL = 15, /* unsigned short,so max 2^15(32768)*/
} opt_define;

struct operation {
    int fault_injection_opt;
    char temp[128];
};

struct default_name {
    opt_define opt;
    char name[32];
};

struct default_name checkname[] = {
    {OPT_DEBUG_ON,    "debug_enable"},
    {OPT_IIC_READ,    "iic_read_check"},
    {OPT_IIC_WRITE,   "iic_write_check"},
    {OPT_CHIP_ERR,    "chip_ic_err"},
    {OPT_NEAR_FAR,    "debug_near_far"},
    {OPT_GET_TIME,    "get_data_time"},
    {OPT_CLEAR_ALL,   "now_clear_all"},
};
/********************************************************/

// healthinfo end

struct oplus_magnetic_cover_operations {
	int (*chip_init)(void *chip_data);
	int (*get_data)(void *chip_data, long *value);
	int (*update_threshold)(void *chip_data,
                            int position,
                            int high_thd,
                            int low_thd);
	int (*chip_esd_check)(void *chip_data, int chip_control);
	void (*dump_regsiter)(struct seq_file *s, void *chip_data);
};

struct notify_event_info {
	uint8_t state_type;
};

struct magnetic_cover_info {
	struct device     *magcvr_dev;
	struct i2c_client *iic_client;
	struct spi_device *spi_client;
	struct input_dev *input_dev;
	int irq;
	int chip_id;
	int position;
	int err_state;
	int last_position;
	int detect_step;
	int detect_offset;
	long cur_distance;
	int m_value;
	int far_threshold;
	int ori_far_threshold;
	int far_noise_th;
	int ori_far_noise_th;
	int negative_far_threshold;
	int negative_ori_far_threshold;
	int negative_far_noise_th;
	int negative_ori_far_noise_th;
	int far_max_th;
	int magcvr_pos_or_neg;
	int irq_type;
	int high_thd;
	int low_thd;
	bool magcvr_notify_support;
	bool magcvr_debounce_irq_support;
	void *chip_info;                                /*Chip Related data*/
	/* power */
	int irq_gpio;                                   /*irq GPIO num*/
	int reset_gpio;                                 /*Reset GPIO*/
	int enable_avdd_gpio;                           /*avdd enable GPIO*/
	int enable_vddi_gpio;                           /*vddi enable GPIO*/
	struct regulator *avdd;                         /*power avdd 2.8-3.3v*/
	struct regulator *vddi;                         /*power vddi 1.8v*/
	uint32_t vdd_volt;                              /*avdd specific volt*/
	uint32_t vddi_volt;                             /*vddi specific volt*/
	struct regulator *vdd_3v;
	struct regulator *vio_1p8;
	unsigned int      vdd_3v_volt;
	unsigned int      vio_1p8_volt;
	/* other config and operation */
	struct oplus_magnetic_cover_operations *mc_ops;  /*call_back function*/
	struct proc_dir_entry *prEntry_magcvr;
	struct work_struct magcvr_handle_wq;
	struct mutex mutex;
	struct notify_event_info *mag_notify;
	bool mag_debug;
	bool driver_start;
	bool same_position;
	struct wakeup_source *wakeup_source;
	// boot opt
	bool noise_set_threshold;
	bool update_first_position;
	// healthinfo
	int iic_read_err_cnt;
	int iic_read_fail;
	int iic_write_err_cnt;
	int iic_write_fail;
	int reg_err[ERR_MAG_REG_MAX_CNT];
	int cal_offset[CAL_OFFSET_MAX_CNT];
	int cal_offset_cnt;
	bool init_chip_failed;
	bool no_need_calibration;
	atomic_long_t stats_attach_wireless_cnt;
	atomic_long_t stats_attach_pen_cnt;
	atomic_long_t stats_detach_wireless_cnt;
	atomic_long_t stats_detach_pen_cnt;
	// fault injection opt
	unsigned short fault_injection_opt;
	int fault_injection_state;
	int magcvr_index;
	int near_key_code;
	int far_key_code;
	const char *pen_name;
	bool is_gpio_mode;
	u64 cur_timer;
	u64 last_timer;
	u64 debounce_time;
	u64 debounce_cnt;
	u64 max_debounce_cnt;
	u64 check_hrtimer_ms;
	u64 check_hrtimer_s;
	u64 timer_check;
	struct hrtimer check_hrtimer;
	ktime_t check_time;
};

struct magnetic_cover_info *alloc_for_magcvr(void);
int magcvr_core_init(struct magnetic_cover_info *magcvr_data);
int magcvr_set_threshold(struct magnetic_cover_info *magcvr_info);
int magcvr_set_position(struct magnetic_cover_info *magcvr_info);
void unregister_magcvr_core(struct magnetic_cover_info *magcvr_info);
int magcvr_set_position(struct magnetic_cover_info *magcvr_info);
int magcvr_set_threshold(struct magnetic_cover_info *magcvr_info);
int fault_injection_handle(struct magnetic_cover_info *magcvr_info, int opt);
int after_magcvr_core_init(struct magnetic_cover_info *magcvr_info);
void mag_call_notifier(int position);

#endif  /* __MAGCVR_CORE_H__ */
