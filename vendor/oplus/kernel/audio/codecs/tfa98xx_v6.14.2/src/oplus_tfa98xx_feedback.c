/******************************************************************************
** File: - oplus_tfa98xx_feedback.c
**
** Copyright (C), 2022-2024, OPLUS Mobile Comm Corp., Ltd
**
** Description:
**     Implementation of tfa98xx reg error or speaker r0 or f0 error feedback.
**
** Version: 1.0
** --------------------------- Revision History: ------------------------------
**      <author>                                       <date>                  <desc>
*******************************************************************************/

#define pr_fmt(fmt) "%s(): " fmt, __func__

#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "config.h"
#include "tfa98xx.h"
#include "tfa.h"
#include "tfa_internal.h"

#include "oplus_tfa98xx_feedback.h"

#include <soc/oplus/system/oplus_mm_kevent_fb.h>

#define SMARTPA_ERR_FB_VERSION             "1.0.3"
#define SPK_ERR_FB_VERSION                 "1.0.3"

#define OPLUS_AUDIO_EVENTID_SMARTPA_ERR    10041
#define OPLUS_AUDIO_EVENTID_SPK_ERR        10042

#define BYPASS_PA_ERR_FB_10041             0x01
#define BYPASS_SPK_ERR_FB_10042            0x02
#define TEST_PA_ERR_FB_10041               0x04
#define TEST_SPK_ERR_FB_10042              0x08

/*bit0~3 check status registers; bit16:check speaker status*/
#define CHECK_STATUS_REGS_MASKS            (0xF)
#define CHECK_SPEAKER_MASKS                (0x100)

enum {
	TFA_LIB_VERSION_SB28_VALUE = 0x81c0000, /* 8.28.0.0 */
	TFA_LIB_VERSION_SB35_VALUE = 0x8230400, /* 8.35.4.0 */
	TFA_LIB_VERSION_SB38_VALUE = 0x8260100, /* 8.38.1.0 */
};

/* enum for library version */
enum {
	TFA_LIB_VERSION_SB28 = 0,  /* 8.28.0.0 */
	TFA_LIB_VERSION_SB35,      /* 8.35.4.0 */
	TFA_LIB_VERSION_SB38,      /* 8.38.1.0 */
	TFA_LIB_VERSION_NUM,
	TFA_LIB_VERSION_INVALID = 0xff  /* invalid version */
};

#define RE_BASE_RANGE                      2500
#define RE_ERR_COUNT_FB                    3
#define RE_ALLOW_NORMAL_CNT                2
#define RE_ERR_TIME_S                      300

#define F0_OUT_OF_RANGE_COUNT_FB           3
#define F0_HOLE_BOLCK_COUNT_FB             5
#define F0_HOLE_BOLCK_TIME_FB              1800
#define F0_MIN                             150
#define F0_MAX                             1900
#define F0_BLOCK_HOLE_INC                  180
#define F0_ALLOW_NORMAL_CNT                2

#define RE_RECORD_CNT                      12
#define F0_RECORD_CNT                      12

#define CHECK_SPK_DELAY_TIME               (10) /* seconds */

/* this enum order must consistent with ready command data, such as: tfaCmdStereoReady_LP */
enum {
	POS_R0 = 0,
	POS_F0,
	POS_NUM
};

#define TFA_DATA_BYTES               3
#define TFA_OFFSET_BASE              3
#define PARAM_OFFSET(pos, id)        (TFA_OFFSET_BASE + ((id) * POS_NUM + (pos)) * TFA_DATA_BYTES)
#define GET_VALUE(pdata, offset)     ((pdata[offset] << 16) + (pdata[offset+1] << 8) + pdata[offset+2])

const int r0_factor[TFA_LIB_VERSION_NUM] = {65, 65, 65}; /* 65 ~ 0x10000 / 1000 */

#define TFA_GET_R0(pdata, id)        (GET_VALUE(pdata, PARAM_OFFSET(POS_R0, id)) / r0_factor[tfa_fb.lib_new])

const int f0_factor[TFA_LIB_VERSION_NUM] = {1, 1, 2048}; /* 1 ~ 0x1, 1 ~ 0x1, 2048 ~ 0x800 */

#define TFA_GET_F0(pdata, id)        (GET_VALUE(pdata, PARAM_OFFSET(POS_F0, id)) / f0_factor[tfa_fb.lib_new])

#define TFA_MAX_RESULT_LEN           (MAX_SPK_NUM * POS_NUM * TFA_DATA_BYTES + TFA_OFFSET_BASE)
#define TFA_RESULT_BUF_LEN           ((TFA_MAX_RESULT_LEN + 3) & (~3))
#define TFA_ONE_ALGO_MAX_RESULT_LEN  (2 * POS_NUM * TFA_DATA_BYTES + TFA_OFFSET_BASE)
#define TFA_CMD_READY_LEN(spkNum)    (((spkNum) * POS_NUM + 2) * TFA_DATA_BYTES)
#define TFA_DATA_NUM_OFFSET          5
#define TFA_CMD_HEAD_OFFSET          0

enum {
	ALL_OFF = 0,
	ALL_ON,
	SPK_OFF,
	PA_OFF,
	SPK_ON,
	PA_ON,
	MAX_ON_OFF
};

enum {
	MONO = 0,
	LEFT = MONO,
	RIGHT,
	THIRD,
	FOURTH,
	MAX_SPK_NUM
};

enum {
	READ_ALGORITHM_VERSION = 0,
	READ_DAMAGED_STATUS_1,
	SET_READY_CMD_1,
	READ_RESULT_FB_1,
	TWO_SPKS_MAX_STEPS = READ_RESULT_FB_1,
	READ_DAMAGED_STATUS_2,
	SET_READY_CMD_2,
	READ_RESULT_FB_2,
	FOUR_SPKS_MAX_STEPS
};

const uint32_t g_step_delay[FOUR_SPKS_MAX_STEPS] = {10000, 20, 20, 100, 20, 20, 100};

#define SetFdBuf(buf, arg, ...) \
	do { \
		int len = strlen(buf); \
		snprintf(buf + len, sizeof(buf) - len - 1, arg, ##__VA_ARGS__); \
	} while (0)

typedef struct {
	uint32_t max_re;
	uint32_t min_re;
	uint32_t err_cnt;
	uint32_t rd_id;
	uint32_t re[RE_RECORD_CNT];
	ktime_t tm;
} tfa_re_err_record_t;

typedef struct {
	uint32_t max_f0;
	uint32_t err_cnt;
	uint32_t rd_id;
	uint32_t f0[F0_RECORD_CNT];
	ktime_t tm;
} tfa_f0_err_record_t;

struct oplus_tfa98xx_feedback {
	uint32_t chk_flag;
	uint32_t control_fb;
	uint32_t vbatlow_cnt;
	uint32_t queue_work_flag;
	int pa_cnt;
	int cmd_step;
	uint32_t lib_version;
	uint32_t lib_new;
	uint32_t r0_cal[MAX_SPK_NUM];
	uint32_t f0_cal[MAX_SPK_NUM];
	uint32_t damage_flag;
	tfa_re_err_record_t rd_re[MAX_SPK_NUM];
	tfa_f0_err_record_t rd_f0[MAX_SPK_NUM];
	struct mutex *lock;
	ktime_t last_chk_reg;
	ktime_t last_chk_spk;
	struct list_head *plist;
};

static struct oplus_tfa98xx_feedback tfa_fb = {
	.chk_flag = 0,
	.control_fb = 0,
	.vbatlow_cnt = 0,
	.queue_work_flag = 0,
	.pa_cnt = 0,
	.cmd_step = 0,
	.lib_version = 0,
	.lib_new = TFA_LIB_VERSION_INVALID,
	.r0_cal = {0, 0, 0, 0},
	.f0_cal = {0, 0, 0, 0},
	.damage_flag = 0,
	.rd_re = {0},
	.rd_f0 = {0},
	.lock = NULL,
	.last_chk_reg = 0,
	.last_chk_spk = 0,
	.plist = NULL
};

const struct oplus_tfa98xx_check_info tfa98xx_check_info[] = {
	{
		.device_revision = 0x66,
		.reg_checked_count = 2,
		.reg_checked_addr = {
			TFA98XX_STATUS_FLAGS0,
			TFA98XX_STATUS_FLAGS1,
		},
		.reg_checked_mask = {
			TFA9865_STATUS_FLAGS0_MASK,
			TFA9865_STATUS_FLAGS1_MASK,
		},
		.expected_value = {
			TFA9865_STATUS_FLAGS0_EXPECT_VALUE,
			TFA9865_STATUS_FLAGS1_EXPECT_VALUE,
		},
	},
};

/* macro for record re error value */
#define F0_ERR_RECORD(rd_f0, val)                                  \
	do {                                                           \
		if (rd_f0.err_cnt == 0) {                                  \
			rd_f0.tm = ktime_get();                                \
		}                                                          \
		if (rd_f0.rd_id < F0_RECORD_CNT) {                         \
			rd_f0.f0[rd_f0.rd_id] = val;                           \
			rd_f0.rd_id++;                                         \
		} else {                                                   \
			rd_f0.rd_id = F0_RECORD_CNT;                           \
		}                                                          \
		rd_f0.err_cnt++;                                           \
		rd_f0.max_f0 = (rd_f0.max_f0 < val) ? val : rd_f0.max_f0;  \
	} while (0)

/* macro for record re error value */
#define RE_ERR_RECORD(rd_re, val)                                  \
	do {                                                           \
		if (rd_re.err_cnt == 0) {                                  \
			rd_re.tm = ktime_get();                                \
		}                                                          \
		if (rd_re.rd_id < RE_RECORD_CNT) {                         \
			rd_re.re[rd_re.rd_id] = val;                           \
			rd_re.rd_id++;                                         \
		} else {                                                   \
			rd_re.rd_id = RE_RECORD_CNT;                           \
		}                                                          \
		rd_re.err_cnt++;                                           \
		rd_re.max_re = (rd_re.max_re < val) ? val : rd_re.max_re;  \
		if ((rd_re.min_re == 0) || (rd_re.min_re > val)) {         \
			rd_re.min_re = val;                                    \
		}                                                          \
	} while (0)

#define ERROR_INFO_MAX_LEN                 32
#define REG_BITS  16
#define TFA9874_STATUS_NORMAL_VALUE    ((0x850F << REG_BITS) + 0x16)/*reg 0x13 high 16 bits and 0x10 low 16 bits*/
#define TFA9874_STATUS_CHECK_MASK      ((0x300 << REG_BITS) + 0x9C)/*reg 0x10 mask bit2~4, bit7, reg 0x13 mask bit8 , bit9 */
#define TFA9873_STATUS_NORMAL_VALUE    ((0x850F << REG_BITS) + 0x56) /*reg 0x13 high 16 bits and 0x10 low 16 bits*/
#define TFA9873_STATUS_CHECK_MASK      ((0x300 << REG_BITS) + 0x15C)/*reg 0x10 mask bit2~4, bit6, bit8, reg 0x13 mask bit8 , bit9*/

/* 2024/06/28, Add for smartpa vbatlow err check. */
#define VBAT_LOW_REG_BIT_MASK              0x10

struct check_status_err {
	int bit;
	uint32_t err_val;
	char info[ERROR_INFO_MAX_LEN];
};

/* TFA9865, currently only check below registers:
	TFA98XX_STATUS_FLAGS0,
	TFA98XX_STATUS_FLAGS1,
*/
static const struct check_status_err check_err_tfa986x[] = {
	/* register TFA98XX_STATUS_FLAGS0 check bits */
	{TFA9865_STATUS_FLAGS0_OTP_POS,             0, "OverTemperature"},
	{TFA9865_STATUS_FLAGS0_UVP_POS,             0, "VbatLow"},
	{TFA9865_STATUS_FLAGS0_OVP_POS,             0, "VbatHigh"},
	{TFA9865_STATUS_FLAGS0_OCP_POS,             1, "CurrentHigh"},

	/* register TFA98XX_STATUS_FLAGS1 check bits */
	{TFA9865_STATUS_FLAGS1_CLKS_POS + REG_BITS,  0, "UnstableClk"},
	{TFA9865_STATUS_FLAGS1_CLIPS_POS + REG_BITS,  1, "Clipping"},
};

static const struct check_status_err check_err_tfa9874[] = {
	/*register 0x10 check bits*/
	{2,             0, "OverTemperature"},
	{3,             1, "CurrentHigh"},
	{4,             0, "VbatLow"},
	{7,             1, "NoClock"},
	/*register 0x13 check bits*/
	{8 + REG_BITS,  0, "VbatHigh"},
	{9 + REG_BITS,  1, "Clipping"},
};

static const struct check_status_err check_err_tfa9873[] = {
	/*register 0x10 check bits*/
	{2,             0, "OverTemperature"},
	{3,             1, "CurrentHigh"},
	{4,             0, "VbatLow"},
	{6,             0, "UnstableClk"},
	{8,             1, "NoClock"},
	/*register 0x13 check bits*/
	{8 + REG_BITS,  0, "VbatHigh"},
	{9 + REG_BITS,  1, "Clipping"},
};

static const unsigned char tfa986x_fb_regs[] = {
	TFA9865_SYS_CONTROL0,
	TFA9865_SYS_CONTROL1,
	TFA9865_SYS_CONTROL2,
	TFA9865_CLOCK_CONTROL,
	TFA9865_STATUS_FLAGS2,
	TFA9865_BATTERY_VOLTAGE,
	TFA9865_TEMPERATURE,
};

static const unsigned char fb_regs[] = {0x00, 0x01, 0x02, 0x04, 0x05, 0x11, 0x14, 0x15, 0x16};

static const unsigned char valid_tfa_chip_id[] = {
	0x65,
	0x66,
	0x73,
	0x74,
};

int is_tfa_id_valid(struct tfa98xx *tfa98xx)
{
	int count = sizeof(valid_tfa_chip_id)/sizeof(valid_tfa_chip_id[0]);
	int i = 0;

	for (; i < count; i++) {
		if ((tfa98xx->rev & 0xFF) == valid_tfa_chip_id[i]) {
			return 1;
		}
	}

	return 0;
}

extern enum Tfa98xx_Error
tfa98xx_write_dsp(struct tfa_device *tfa, int num_bytes, const char *command_buffer);
extern enum Tfa98xx_Error
tfa98xx_read_dsp(struct tfa_device *tfa, int num_bytes, unsigned char *result_buffer);

inline bool is_param_valid(struct tfa98xx *tfa98xx)
{
	if ((tfa98xx == NULL) || (tfa98xx->tfa == NULL) || (tfa98xx->tfa98xx_wq == NULL)) {
		pr_err("input parameter is not available\n");
		return false;
	}

	if (is_tfa_id_valid(tfa98xx) == 0) {
		return false;
	}

	if (tfa98xx->tfa->dev_idx >= MAX_SPK_NUM) {
		pr_err("dev_idx = %d error\n", tfa98xx->tfa->dev_idx);
		return false;
	}

	return true;
}

static int tfa98xx_check_status_reg(struct tfa98xx *tfa98xx)
{
	uint64_t reg_val = 0;
	uint16_t reg10 = 0;
	uint16_t reg13 = 0;
	uint16_t reg_tmp = 0;
	int flag = 0;
	char fb_buf[MAX_PAYLOAD_DATASIZE] = {0};
	char info[MAX_PAYLOAD_DATASIZE] = {0};
	int offset = 0;
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;
	int i;

	int info_idx = 0;
	int checked_reg_count = 0;
	unsigned short status_reg_value[MAX_STATUS_REG_CHECKED_COUNT] = {0};

	const struct check_status_err *ptr_err_info = NULL;
	const unsigned char *ptr_fb_regs = NULL;
	int err_info_size = 0;
	int fb_reg_size = 0;

	if (((tfa98xx->rev & 0xFF) == 0x65) || ((tfa98xx->rev & 0xFF) == 0x66)) {
		reg_val = 0;
		ptr_err_info = check_err_tfa986x;
		err_info_size = ARRAY_SIZE(check_err_tfa986x);
		ptr_fb_regs = tfa986x_fb_regs;
		fb_reg_size = ARRAY_SIZE(tfa986x_fb_regs);

		pr_debug("%s, %d, err_info_size = %d", __func__, __LINE__, err_info_size);

		for (info_idx = 0; info_idx < sizeof(tfa98xx_check_info) / sizeof(tfa98xx_check_info[0]); info_idx++) {
			if (tfa98xx_check_info[info_idx].device_revision == (tfa98xx->rev & 0xFF))
				break;
		}

		if (info_idx == sizeof(tfa98xx_check_info) / sizeof(tfa98xx_check_info[0])) {
			pr_err("%s, %d, TFA98%#x info not found!", __func__, __LINE__, tfa98xx->rev & 0xFF);

			return -1;
		}

		checked_reg_count = min(tfa98xx_check_info[info_idx].reg_checked_count, MAX_STATUS_REG_CHECKED_COUNT);

		for (i = 0; i < checked_reg_count; i++) {
			err |= tfa98xx_read_register16(tfa98xx->tfa, tfa98xx_check_info[info_idx].reg_checked_addr[i], &status_reg_value[i]);

			pr_debug("%s, %d, i = %d, addr = %#x, value = 0x%04X, mask = 0x%04X",
				__func__, __LINE__,
				i, tfa98xx_check_info[info_idx].reg_checked_addr[i],
				status_reg_value[i], tfa98xx_check_info[info_idx].reg_checked_mask[i]);

			pr_debug("%s, %d, (value & mask) = 0x%04X, expect_value = 0x%04X",
				__func__, __LINE__,
				status_reg_value[i] & tfa98xx_check_info[info_idx].reg_checked_mask[i], tfa98xx_check_info[info_idx].expected_value[i]);
		}

		for (i = checked_reg_count - 1; i >= 0; i--) {
			reg_val = (reg_val << REG_BITS) | status_reg_value[i];
		}

		pr_debug("%s, %d, reg_val = 0x%llX", __func__, __LINE__, reg_val);

		if (Tfa98xx_Error_Ok == err) {
			if (tfa_fb.control_fb & TEST_PA_ERR_FB_10041) {
				reg_val = 0;
				pr_info("%s: just for test 10041, change reg_val=0x%llx", __func__, reg_val);
			}
			/* 2024/06/28, Add for smartpa vbatlow err check. */
			if (0 == (reg_val & (1 << TFA9865_STATUS_FLAGS0_UVP_POS))) {
				tfa_fb.vbatlow_cnt++;
				pr_info("%s: vbatlow_cnt=%u", __func__, tfa_fb.vbatlow_cnt);
			}
			flag = 0;

			for (i = 0; i < checked_reg_count; i++) {
				pr_debug("%s, %d, status_reg_value[%d] = 0x%04X, mask = 0x%04X",
					__func__, __LINE__,
					i, status_reg_value[i], tfa98xx_check_info[info_idx].reg_checked_mask[i]);

				pr_debug("%s, %d, status_reg_value[i] & tfa98xx_check_info[info_idx].reg_checked_mask[i]) = 0x%04X",
					__func__, __LINE__,
					status_reg_value[i] & tfa98xx_check_info[info_idx].reg_checked_mask[i]);

				if ((status_reg_value[i] & tfa98xx_check_info[info_idx].reg_checked_mask[i]) != tfa98xx_check_info[info_idx].expected_value[i]) {
					flag = 1;
					break;
				}
			}

			if (flag == 1) {
				SetFdBuf(info, "TFA98%X, SPK%x, ", (tfa98xx->rev & 0xFF), tfa98xx->tfa->dev_idx + 1);

				for (i = 0; i < checked_reg_count; i++) {
					SetFdBuf(info, "reg[0x%02x]=0x%04x, ", tfa98xx_check_info[info_idx].reg_checked_addr[i], status_reg_value[i]);
				}
			}

			for (i = 0; i < err_info_size; i++) {
				if (ptr_err_info[i].err_val == (1 & (reg_val >> ptr_err_info[i].bit))) {
					SetFdBuf(info, "%s,", ptr_err_info[i].info);
				}
			}

			/* read other registers */
			if (flag == 1) {
				SetFdBuf(info, "dump regs(");
				for (i = 0; i < fb_reg_size; i++) {
					err = tfa98xx_read_register16(tfa98xx->tfa, ptr_fb_regs[i], &reg_tmp);
					if (Tfa98xx_Error_Ok == err) {
						SetFdBuf(info, "%x=0x%04x,", ptr_fb_regs[i], reg_tmp);
					} else {
						break;
					}
				}
				SetFdBuf(info, "),");
			}
		} else {
			SetFdBuf(info, "TFA98%x SPK%d: failed to read status regs, error=%d,", \
				(tfa98xx->rev & 0xFF), tfa98xx->tfa->dev_idx + 1, err);
				tfa_fb.last_chk_reg = ktime_get();
		}
	} else {
		/* check status register 0x10 value */
		err = tfa98xx_read_register16(tfa98xx->tfa, 0x10, &reg10);
		if (Tfa98xx_Error_Ok == err) {
			err = tfa98xx_read_register16(tfa98xx->tfa, 0x13, &reg13);
		}
		pr_info("%s: read SPK%d status regs ret=%d, reg[0x10]=0x%x, reg[0x13]=0x%x", \
				__func__, tfa98xx->tfa->dev_idx + 1, err, reg10, reg13);

		if (Tfa98xx_Error_Ok == err) {
			reg_val = (reg13 << REG_BITS) + reg10;
			if (tfa_fb.control_fb & TEST_PA_ERR_FB_10041) {
				reg_val = 0;
				pr_info("%s: just for test 10041, change reg_val=0x%llx", __func__, reg_val);
			}
			/* 2024/06/28, Add for smartpa vbatlow err check. */
			if (0 == (reg_val & VBAT_LOW_REG_BIT_MASK)) {
				tfa_fb.vbatlow_cnt++;
				pr_info("%s: vbatlow_cnt=%u", __func__, tfa_fb.vbatlow_cnt);
			}
			flag = 0;
			if (((tfa98xx->rev & 0xff) == 0x74) &&
					((TFA9874_STATUS_NORMAL_VALUE&TFA9874_STATUS_CHECK_MASK) != (reg_val&TFA9874_STATUS_CHECK_MASK))) {
				SetFdBuf(info, "TFA9874 SPK%x:reg[0x10]=0x%x,reg[0x13]=0x%x,", tfa98xx->tfa->dev_idx + 1, reg10, reg13);
				for (i = 0; i < ARRAY_SIZE(check_err_tfa9874); i++) {
					if (check_err_tfa9874[i].err_val == (1 & (reg_val >> check_err_tfa9874[i].bit))) {
						SetFdBuf(info, "%s,", check_err_tfa9874[i].info);
					}
				}
				flag = 1;
			} else if (((tfa98xx->rev & 0xff) == 0x73) &&
					((TFA9873_STATUS_NORMAL_VALUE&TFA9873_STATUS_CHECK_MASK) != (reg_val&TFA9873_STATUS_CHECK_MASK))) {
				SetFdBuf(info, "TFA9873 SPK%x:reg[0x10]=0x%x,reg[0x13]=0x%x,", tfa98xx->tfa->dev_idx + 1, reg10, reg13);
				for (i = 0; i < ARRAY_SIZE(check_err_tfa9873); i++) {
					if (check_err_tfa9873[i].err_val == (1 & (reg_val >> check_err_tfa9873[i].bit))) {
						SetFdBuf(info, "%s,", check_err_tfa9873[i].info);
					}
				}
				flag = 1;
			}

			/* read other registers */
			if (flag == 1) {
				SetFdBuf(info, "dump regs(");
				for (i = 0; i < sizeof(fb_regs); i++) {
					err = tfa98xx_read_register16(tfa98xx->tfa, fb_regs[i], &reg_tmp);
					if (Tfa98xx_Error_Ok == err) {
						SetFdBuf(info, "%x=%x,", fb_regs[i], reg_tmp);
					} else {
						break;
					}
				}
				SetFdBuf(info, "),");
			}
		} else {
			SetFdBuf(info, "TFA98%x SPK%d: failed to read regs 0x10 and 0x13, error=%d,", \
					(tfa98xx->rev & 0xFF), tfa98xx->tfa->dev_idx + 1, err);
			tfa_fb.last_chk_reg = ktime_get();
		}
	}

	/* feedback the check error */
	offset = strlen(info);
	if ((offset > 0) && (offset < MM_KEVENT_MAX_PAYLOAD_SIZE)) {
		if (tfa_fb.control_fb & TEST_PA_ERR_FB_10041) {
			scnprintf(fb_buf, sizeof(fb_buf) - 1, "payload@@just for test 10041, ignore");
		} else {
			scnprintf(fb_buf, sizeof(fb_buf) - 1, "payload@@%s", info);
		}
		pr_err("%s: fb_buf=%s\n", __func__, fb_buf);
		mm_fb_audio_kevent_named(OPLUS_AUDIO_EVENTID_SMARTPA_ERR,
				MM_FB_KEY_RATELIMIT_5MIN, fb_buf);
	}

	return 0;
}

static int tfa98xx_set_check_feedback(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	int val = ucontrol->value.integer.value[0];

	switch (val) {
	case ALL_OFF:
		tfa_fb.chk_flag = 0;
		break;
	case ALL_ON:
		tfa_fb.chk_flag = CHECK_SPEAKER_MASKS + CHECK_STATUS_REGS_MASKS;
		break;
	case SPK_OFF:
		tfa_fb.chk_flag &= ~CHECK_SPEAKER_MASKS;
		break;
	case PA_OFF:
		tfa_fb.chk_flag &= ~CHECK_STATUS_REGS_MASKS;
		break;
	case SPK_ON:
		tfa_fb.chk_flag |= CHECK_SPEAKER_MASKS;
		break;
	case PA_ON:
		tfa_fb.chk_flag |= CHECK_STATUS_REGS_MASKS;
		break;
	default:
		pr_info("%s: unsupported set value = %d\n", __func__, val);
		break;
	}

	pr_debug("%s: set value = %d, tfa_fb.chk_flag = 0x%x\n", __func__, val, tfa_fb.chk_flag);
	return 1;
}

static int tfa98xx_get_check_feedback(struct snd_kcontrol *kcontrol,
						struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = tfa_fb.chk_flag;
	pr_info("%s: tfa_fb.chk_flag = 0x%x\n", __func__, tfa_fb.chk_flag);

	return 0;
}

static int tfa98xx_set_bypass_feedback(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	tfa_fb.control_fb = ucontrol->value.integer.value[0];
	pr_info("%s: set %u", __func__, tfa_fb.control_fb);
	return 0;
}

static int tfa98xx_get_bypass_feedback(struct snd_kcontrol *kcontrol,
						struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = tfa_fb.control_fb;
	pr_info("%s: get %u", __func__, tfa_fb.control_fb);
	return 0;
}

/* 2024/06/28, Add for smartpa vbatlow err check. */
static int tfa98xx_get_vbatlow_cnt(struct snd_kcontrol *kcontrol,
						struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_component_get_drvdata(component);

	if ((tfa_fb.chk_flag & CHECK_STATUS_REGS_MASKS) && (tfa98xx->dsp_init != TFA98XX_DSP_INIT_STOPPED) &&
			!(tfa_fb.control_fb & BYPASS_PA_ERR_FB_10041)) {
		mutex_lock(tfa_fb.lock);
		list_for_each_entry(tfa98xx, tfa_fb.plist, list) {
			tfa98xx_check_status_reg(tfa98xx);
		}
		mutex_unlock(tfa_fb.lock);
	}

	ucontrol->value.integer.value[0] = tfa_fb.vbatlow_cnt;
	pr_info("%s: vbatlow_cnt = %u", __func__, tfa_fb.vbatlow_cnt);
	tfa_fb.vbatlow_cnt = 0;

	return 0;
}

static char const *tfa98xx_check_feedback_text[] = {"Off", "On", "SPKOff", "PAOff", "SPKOn", "PAOn"};
static const struct soc_enum tfa98xx_check_feedback_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tfa98xx_check_feedback_text), tfa98xx_check_feedback_text);
const struct snd_kcontrol_new tfa98xx_check_feedback[] = {
	SOC_ENUM_EXT("TFA_CHECK_FEEDBACK", tfa98xx_check_feedback_enum,
			tfa98xx_get_check_feedback, tfa98xx_set_check_feedback),
	SOC_SINGLE_EXT("PA_BYPASS_FEEDBACK", SND_SOC_NOPM, 0, 0xff, 0,
			tfa98xx_get_bypass_feedback, tfa98xx_set_bypass_feedback),
	/* 2024/06/28, Add for smartpa vbatlow err check. */
	SOC_SINGLE_EXT("PA Vbatlow Count", SND_SOC_NOPM, 0, 0xFFFF, 0,
			tfa98xx_get_vbatlow_cnt, NULL),
};

static int tfa98xx_cmd_set(struct tfa98xx *tfa98xx, int8_t *pbuf, int16_t size)
{
	int err = -1;

	if ((pbuf == NULL) || (size <= 0)) {
		pr_err("input parameter is not available\n");
		return -ENODEV;
	}

	if (tfa98xx && tfa98xx->tfa->is_probus_device) {
		mutex_lock(&tfa98xx->dsp_lock);
		err = tfa98xx_write_dsp(tfa98xx->tfa, size, pbuf);
		mutex_unlock(&tfa98xx->dsp_lock);
	}

	if (err != Tfa98xx_Error_Ok) {
		pr_err("send data to adsp error, ret = %d\n", err);
	}

	return err;
}

static int tfa98xx_get_lib_version(struct tfa98xx *tfa98xx, unsigned int *pversion)
{
	int err = -1;
	unsigned char result[TFA_ONE_ALGO_MAX_RESULT_LEN] = {0};
	int8_t tfaCmdLibVer[] = {0x00, 0x80, 0xfe};
	unsigned char lib_ver[4]= {0};

	if (pversion == NULL) {
		pr_err("input parameter is not available\n");
		return -ENODEV;
	}

	if (tfa98xx && tfa98xx->tfa->is_probus_device) {
		mutex_lock(&tfa98xx->dsp_lock);
		err = tfa98xx_write_dsp(tfa98xx->tfa, sizeof(tfaCmdLibVer), tfaCmdLibVer);
		if (err == Tfa98xx_Error_Ok) {
			err = tfa98xx_read_dsp(tfa98xx->tfa, sizeof(result), result);
		}
		mutex_unlock(&tfa98xx->dsp_lock);
	}

	if (err != Tfa98xx_Error_Ok) {
		pr_err("send or recv data error, ret = %d\n", err);

		return err;
	}

	/* Split 3rd byte into two seperate ITF version fields (3rd field and 4th field) */
	lib_ver[0] = (result[0]);
	lib_ver[1] = (result[1]);
	if ((lib_ver[0] != 2) && (lib_ver[1] >= 33)) {
		lib_ver[3] = (result[2]) & 0x07;
		lib_ver[2] = (result[2] >> 3) & 0x1F;
	} else {
		lib_ver[3] = (result[2]) & 0x3f;
		lib_ver[2] = (result[2] >> 6) & 0x03;
	}
	*pversion = (lib_ver[0] << 24) + (lib_ver[1] << 16) + (lib_ver[2] << 8) + lib_ver[3];
	pr_info("tfa lib version is %d.%d.%d.%d, version=0x%x", \
			lib_ver[0], lib_ver[1], lib_ver[2], lib_ver[3], *pversion);

	return err;
}

static int tfa98xx_check_new_lib(struct tfa98xx *tfa98xx)
{
	unsigned int lib_ver = 0;

	if (tfa_fb.lib_new < TFA_LIB_VERSION_NUM){
		return Tfa98xx_Error_Ok;
	}

	if (Tfa98xx_Error_Ok == tfa98xx_get_lib_version(tfa98xx, &lib_ver)) {
		if (lib_ver != 0) {
			tfa_fb.lib_version = lib_ver;
			if (lib_ver <= TFA_LIB_VERSION_SB28_VALUE) { // 8.28.0.0
				tfa_fb.lib_new = TFA_LIB_VERSION_SB28;
			} else if (lib_ver <= TFA_LIB_VERSION_SB35_VALUE) { // 8.35.4.0
				tfa_fb.lib_new = TFA_LIB_VERSION_SB35;
			} else if (lib_ver <= TFA_LIB_VERSION_SB38_VALUE) { // 8.38.1.0
				tfa_fb.lib_new = TFA_LIB_VERSION_SB38;
			} else {
				pr_err("invalid lib_ver=%u, use default SB38", lib_ver);
				tfa_fb.lib_new = TFA_LIB_VERSION_SB38;
			}
			return Tfa98xx_Error_Ok;
		}
	}
	return -1;
}

static int tfa98xx_get_speaker_status(struct tfa98xx *tfa98xx, uint32_t algo_flag)
{
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;
	char buffer[6] = {0};

	if ((CHECK_SPEAKER_MASKS & tfa_fb.chk_flag) == 0) {
		return -1;
	}

	mutex_lock(&tfa98xx->dsp_lock);
	/*Get the GetStatusChange results*/
	err = tfa_dsp_cmd_id_write_read(tfa98xx->tfa, MODULE_FRAMEWORK,
			FW_PAR_ID_GET_STATUS_CHANGE, 6, (unsigned char *)buffer);
	mutex_unlock(&tfa98xx->dsp_lock);

	if (err == Tfa98xx_Error_Ok) {
		if (buffer[2] & 0x6) {
			if (1 == algo_flag) {
				if (buffer[2] & 0x2) {
					tfa_fb.damage_flag |= (1 << LEFT);
				}
				if ((tfa_fb.pa_cnt > 1) && (buffer[2] & 0x4)) {
					tfa_fb.damage_flag |= (1 << RIGHT);
				}
			} else if (2 == algo_flag) {
				if (buffer[2] & 0x2) {
					tfa_fb.damage_flag |= (1 << THIRD);
				}
				if ((tfa_fb.pa_cnt > 3) && (buffer[2] & 0x4)) {
					tfa_fb.damage_flag |= (1 << FOURTH);
				}
			} else {
				pr_err("not support algo_flag = %u", algo_flag);
			}
		}
	}
	pr_info("ret=%d, damage_flag=%d\n", err, tfa_fb.damage_flag);

	return err;
}

/* set cmd to protection algorithm to calcurate r0, f0 */
static int tfa98xx_set_ready_cmd(struct tfa98xx *tfa98xx, uint32_t algo_flag)
{
	int16_t spk_num = 0;
	int16_t cmdlen = 0;
	int8_t *pcmd = NULL;

	/*cmd BYTE0: first algorithm: 0x00, second algorithm: 0x10*/
	/*cmd BYTE5: need to change as accroding to get param num */
	/* lib_new: TFA_LIB_VERSION_SB28->SB28, TFA_LIB_VERSION_SB35->SB35, TFA_LIB_VERSION_SB38->SB38 */
	int8_t tfaCmdReady[TFA_LIB_VERSION_NUM][18] = {
		{/* SB28 */
			0x00, 0x80, 0x0b, 0x00, 0x00, 0x04,
			/*first speaker: R0, F0*/
			0x22, 0x00, 0x00,
			0x22, 0x00, 0x13,
			/*second speaker*/
			0x22, 0x00, 0x01,
			0x22, 0x00, 0x14
		},
		{/* SB35 */
			0x00, 0x80, 0x0b, 0x00, 0x00, 0x04,
			/*first speaker: R0, F0*/
			0x22, 0x00, 0x00,
			0x22, 0x00, 0x1b,
			/*second speaker*/
			0x22, 0x00, 0x01,
			0x22, 0x00, 0x1c
		},
		{/* SB38 */
			0x00, 0x80, 0x0b, 0x00, 0x00, 0x04,
			/*first speaker: R0, F0*/
			0x22, 0x00, 0x00,
			0x22, 0x00, 0x1d,
			/*second speaker*/
			0x22, 0x00, 0x01,
			0x22, 0x00, 0x1e
		}
	};

	if (tfa_fb.lib_new >= TFA_LIB_VERSION_NUM) {
		pr_err("invalid lib_new=%u, use newest version", tfa_fb.lib_new);
		tfa_fb.lib_new = TFA_LIB_VERSION_NUM - 1;
	}
	pcmd = tfaCmdReady[tfa_fb.lib_new];
	if (1 == algo_flag) {
		*(pcmd + TFA_CMD_HEAD_OFFSET) = 0x00;
		spk_num = (tfa_fb.pa_cnt == 1) ? 1 : 2;
	} else if (2 == algo_flag) {
		*(pcmd + TFA_CMD_HEAD_OFFSET) = 0x10;
		spk_num = (tfa_fb.pa_cnt == 3) ? 1 : 2;
	} else {
		pr_err("not support algo_flag = %u", algo_flag);
		return -EINVAL;
	}
	*(pcmd + TFA_DATA_NUM_OFFSET) = POS_NUM * spk_num;
	cmdlen = TFA_CMD_READY_LEN(spk_num);

	return tfa98xx_cmd_set(tfa98xx, pcmd, cmdlen);
}

static int tfa98xx_get_result(struct tfa98xx *tfa98xx, uint8_t *pbuf, int len, uint32_t algo_flag)
{
	uint8_t tfaCmdGet[] = {0x00, 0x80, 0x8b, 0x00};
	int err = Tfa98xx_Error_Ok;

	if (!pbuf || (0 == len)) {
		err = -EINVAL;
		pr_err("input param error");
		goto exit;
	}

	if (1 == algo_flag) {
		tfaCmdGet[TFA_CMD_HEAD_OFFSET] = 0x00;
	} else if (2 == algo_flag) {
		tfaCmdGet[TFA_CMD_HEAD_OFFSET] = 0x10;
	} else {
		err = -EINVAL;
		pr_err("not support algo_flag = %u", algo_flag);
		goto exit;
	}

	if (tfa98xx && tfa98xx->tfa->is_probus_device) {
		mutex_lock(&tfa98xx->dsp_lock);
		err = tfa98xx_write_dsp(tfa98xx->tfa, sizeof(tfaCmdGet), tfaCmdGet);
		if (err == Tfa98xx_Error_Ok) {
			err = tfa98xx_read_dsp(tfa98xx->tfa, len, pbuf);
		}
		mutex_unlock(&tfa98xx->dsp_lock);
	}

	if (err != Tfa98xx_Error_Ok) {
		pr_err("set or get adsp data error, ret = %d\n", err);
		goto exit;
	}

	if (0x00 == GET_VALUE(pbuf, 0)) {
		pr_err("get adsp data error\n");
		err = Tfa98xx_Error_DSP_not_running;
		goto exit;
	}

exit:
	return err;
}

void tfa98xx_check_f0(uint32_t f0, int ch)
{
	char fb_buf[MAX_PAYLOAD_DATASIZE] = {0};
	int i = 0;

	if ((ch < 0) || (ch >= MAX_SPK_NUM)) {
		pr_err("invalid speaker number: %d", ch + 1);
		return;
	}

	if ((f0 <= F0_MIN) || (f0 >= F0_MAX)) {
		F0_ERR_RECORD(tfa_fb.rd_f0[ch], f0);
		if (tfa_fb.rd_f0[ch].err_cnt >= F0_OUT_OF_RANGE_COUNT_FB) {
			SetFdBuf(fb_buf, "payload@@TFA98xx SPK%d:F0=%u,out of range(%u, %u)", \
				ch + 1, f0, F0_MIN, F0_MAX);
		}
	} else if ((tfa_fb.f0_cal[ch] != 0) && (f0 > (tfa_fb.f0_cal[ch] + F0_BLOCK_HOLE_INC))) {
		F0_ERR_RECORD(tfa_fb.rd_f0[ch], f0);
		if ((tfa_fb.rd_f0[ch].err_cnt >= F0_HOLE_BOLCK_COUNT_FB) &&
			(ktime_after(ktime_get(), ktime_add_ms(tfa_fb.rd_f0[ch].tm, F0_HOLE_BOLCK_TIME_FB * 1000)) ||
				(tfa_fb.rd_f0[ch].max_f0 >= F0_MAX))) {
			SetFdBuf(fb_buf, "payload@@TFA98xx SPK%d:F0=%u, larger than %u, maybe speaker hole blocked", \
				ch + 1, f0, (tfa_fb.f0_cal[ch] + F0_BLOCK_HOLE_INC));
		}
	} else if (tfa_fb.rd_f0[ch].rd_id > 0) {
		if (tfa_fb.rd_f0[ch].rd_id < (tfa_fb.rd_f0[ch].err_cnt + F0_ALLOW_NORMAL_CNT)) {
			if (tfa_fb.rd_f0[ch].rd_id < F0_RECORD_CNT) {
				tfa_fb.rd_f0[ch].f0[tfa_fb.rd_f0[ch].rd_id] = f0;
				tfa_fb.rd_f0[ch].rd_id++;
			} else {
				tfa_fb.rd_f0[ch].err_cnt--;
			}
		} else {
			memset(&tfa_fb.rd_f0[ch], 0, sizeof(tfa_fb.rd_f0[ch]));
		}
	}

	if (tfa_fb.rd_f0[ch].err_cnt > 0) {
		pr_err("TFA98xx SPK%d: record tm=%lld, now tm=%lld, err_cnt=%u, rd_id=%u, max_f0=%u",
			ch + 1, tfa_fb.rd_f0[ch].tm, ktime_get(), tfa_fb.rd_f0[ch].err_cnt, tfa_fb.rd_f0[ch].rd_id, tfa_fb.rd_f0[ch].max_f0);
	}

	if (strlen(fb_buf) > 0) {
		SetFdBuf(fb_buf, ",max_f0=%u,record f0=", tfa_fb.rd_f0[ch].max_f0);
		for (i = 0; i < tfa_fb.rd_f0[ch].rd_id; i++) {
			SetFdBuf(fb_buf, "%u,", tfa_fb.rd_f0[ch].f0[i]);
		}
		mm_fb_audio_kevent_named(OPLUS_AUDIO_EVENTID_SPK_ERR, MM_FB_KEY_RATELIMIT_1H, fb_buf);
		pr_err("%s", fb_buf);
		memset(&tfa_fb.rd_f0[ch], 0, sizeof(tfa_fb.rd_f0[ch]));
	}
}

void tfa98xx_check_re(uint32_t re, int ch)
{
	char fb_buf[MAX_PAYLOAD_DATASIZE] = {0};
	int i = 0;

	if ((ch < 0) || (ch >= MAX_SPK_NUM)) {
		pr_err("invalid speaker number: %d", ch + 1);
		return;
	}

	/* just for test */
	if (tfa_fb.control_fb & TEST_SPK_ERR_FB_10042) {
		re = 20000;
		pr_err("change re=%d, just for test 10042, ignore", re);
	}

	if (tfa_fb.damage_flag & (1 << ch)) {
		RE_ERR_RECORD(tfa_fb.rd_re[ch], re);
		if ((tfa_fb.rd_re[ch].err_cnt >= RE_ERR_COUNT_FB) &&
			ktime_after(ktime_get(), ktime_add_ms(tfa_fb.rd_re[ch].tm, RE_ERR_TIME_S * 1000))) {
			SetFdBuf(fb_buf, "payload@@TFA98xx SPK%d-detected-damaged", ch + 1);
		}
	} else if ((re < (tfa_fb.r0_cal[ch] - RE_BASE_RANGE)) || (re > (tfa_fb.r0_cal[ch] + RE_BASE_RANGE))) {
		RE_ERR_RECORD(tfa_fb.rd_re[ch], re);
		if ((tfa_fb.rd_re[ch].err_cnt >= RE_ERR_COUNT_FB) &&
			ktime_after(ktime_get(), ktime_add_ms(tfa_fb.rd_re[ch].tm, RE_ERR_TIME_S * 1000))) {
			SetFdBuf(fb_buf, "payload@@TFA98xx SPK%d:speaker R0 out of range(%u, %u), re=%u", \
					ch + 1, (tfa_fb.r0_cal[ch] - RE_BASE_RANGE), (tfa_fb.r0_cal[ch] + RE_BASE_RANGE), re);
		}
	} else {
		if ((tfa_fb.rd_re[ch].rd_id > 0) && (tfa_fb.rd_re[ch].rd_id < (tfa_fb.rd_re[ch].err_cnt + RE_ALLOW_NORMAL_CNT))) {
			if (tfa_fb.rd_re[ch].rd_id < RE_RECORD_CNT) {
				tfa_fb.rd_re[ch].re[tfa_fb.rd_re[ch].rd_id] = re;
				tfa_fb.rd_re[ch].rd_id++;
			} else {
				tfa_fb.rd_re[ch].err_cnt--;
			}
		} else if (tfa_fb.rd_re[ch].rd_id >= (tfa_fb.rd_re[ch].err_cnt + RE_ALLOW_NORMAL_CNT)) {
			memset(&tfa_fb.rd_re[ch], 0, sizeof(tfa_fb.rd_re[ch]));
		}
	}

	if (tfa_fb.rd_re[ch].err_cnt > 0) {
		pr_err("TFA98xx SPK%u: min_re=%d, max_re=%d, rd_id=%u, err_cnt=%u",
			ch + 1, tfa_fb.rd_re[ch].min_re, tfa_fb.rd_re[ch].max_re, tfa_fb.rd_re[ch].rd_id, tfa_fb.rd_re[ch].err_cnt);
	}

	if (strlen(fb_buf) > 0) {
		SetFdBuf(fb_buf, ",r0_cal=%u, min_re=%u,max_re=%u,record re=",
			tfa_fb.r0_cal[ch], tfa_fb.rd_re[ch].min_re, tfa_fb.rd_re[ch].max_re);
		for (i = 0; i < tfa_fb.rd_re[ch].rd_id; i++) {
			SetFdBuf(fb_buf, "%u,", tfa_fb.rd_re[ch].re[i]);
		}

		if (tfa_fb.control_fb & TEST_SPK_ERR_FB_10042) {
			mm_fb_audio_kevent_named(OPLUS_AUDIO_EVENTID_SPK_ERR, MM_FB_KEY_RATELIMIT_1H, "just for test 10042, ignore");
		} else {
			mm_fb_audio_kevent_named(OPLUS_AUDIO_EVENTID_SPK_ERR, MM_FB_KEY_RATELIMIT_1H, fb_buf);
		}
		pr_err("%s", fb_buf);
		memset(&tfa_fb.rd_re[ch], 0, sizeof(tfa_fb.rd_re[ch]));
	}
}

static int tfa98xx_check_result(uint8_t *pdata, uint32_t algo_flag)
{
	uint32_t re = 0;
	uint32_t fr = 0;
	int index = 0;

	if (!pdata) {
		pr_err("pdata is null");
		return -EINVAL;
	}
	if ((tfa_fb.pa_cnt <= MONO) || (tfa_fb.pa_cnt > MAX_SPK_NUM)) {
		pr_err("pa_cnt %d invalid", tfa_fb.pa_cnt);
		return -EINVAL;
	}

	if (algo_flag == 1) {
		index = 0;
	} else {
		index = 2;
	}
	for (; index < tfa_fb.pa_cnt; index++) {
		re = TFA_GET_R0(pdata, index);
		fr = TFA_GET_F0(pdata, index);
		pr_info("TFA98xx SPK%d: damage_flag=0x%x, re=%u, fr=%u",
			index + 1, tfa_fb.damage_flag, re, fr);

		/* for feedback test */
		if (tfa_fb.control_fb & TEST_SPK_ERR_FB_10042) {
			re = 20000;
			pr_info("just for test 10042, change re=%d", re);
		}

		tfa98xx_check_re(re, index);
		tfa98xx_check_f0(fr, index);
	}

	return Tfa98xx_Error_Ok;
}

static void tfa98xx_check_work(struct work_struct *work)
{
	struct tfa98xx *tfa98xx = NULL;
	struct tfa98xx *tfa98xx_1 = NULL;
	struct tfa98xx *tfa98xx_2 = NULL;
	uint8_t result[TFA_RESULT_BUF_LEN] = {0};
	int ret = Tfa98xx_Error_Ok;

	if (!(CHECK_SPEAKER_MASKS & tfa_fb.chk_flag) || !tfa_fb.plist || !tfa_fb.lock) {
		return;
	}

	mutex_lock(tfa_fb.lock);
	list_for_each_entry(tfa98xx, tfa_fb.plist, list) {
		if (!is_param_valid(tfa98xx)) {
			continue;
		}

		if (tfa98xx->tfa->dev_idx == 0) {
			tfa98xx_1 = tfa98xx;
		} else if ((tfa_fb.pa_cnt > 2) && (tfa_fb.pa_cnt <= 4) && (tfa98xx->tfa->dev_idx == 2)) {
			tfa98xx_2 = tfa98xx;
		}
	}
	mutex_unlock(tfa_fb.lock);

	if (NULL == tfa98xx_1) {
		pr_err("parameter is not available\n");
		return;
	}

	tfa98xx = container_of(work, struct tfa98xx, check_work.work);

	switch (tfa_fb.cmd_step) {
	case READ_ALGORITHM_VERSION:
		/*get algorithm library version if not get before*/
		ret = tfa98xx_check_new_lib(tfa98xx);
		break;
	case READ_DAMAGED_STATUS_1:
		/*get speaker hole blocked or damaged status */
		ret = tfa98xx_get_speaker_status(tfa98xx_1, 1);
		break;
	case SET_READY_CMD_1:
		/* set cmd to protection algorithm to calcurate r0, f0 */
		ret = tfa98xx_set_ready_cmd(tfa98xx_1, 1);
		break;
	case READ_RESULT_FB_1:
	/* set cmd to get r0, f0 */
		ret = tfa98xx_get_result(tfa98xx_1, result, sizeof(result), 1);
		if (ret == Tfa98xx_Error_Ok){
			tfa98xx_check_result(result, 1);
		}
		break;
	case READ_DAMAGED_STATUS_2:
		/*get speaker hole blocked or damaged status */
		if (tfa98xx_2) {
			ret = tfa98xx_get_speaker_status(tfa98xx_2, 2);
		}
		break;
	case SET_READY_CMD_2:
		/* set cmd to protection algorithm to calcurate r0, f0 */
		if (tfa98xx_2) {
			ret = tfa98xx_set_ready_cmd(tfa98xx_2, 2);
		}
		break;
	case READ_RESULT_FB_2:
		/* set cmd to get r0, f0 */
		if (tfa98xx_2) {
			ret = tfa98xx_get_result(tfa98xx_2, result, sizeof(result), 2);
			if (ret == Tfa98xx_Error_Ok) {
				tfa98xx_check_result(result, 2);
			}
		}
		break;
	default:
		ret = Tfa98xx_Error_Bad_Parameter;
		break;
	}

	if (Tfa98xx_Error_Ok == ret) {
		tfa_fb.cmd_step += 1;
	} else {
		pr_err("chk_flag=0x%x, step=%d, error ret=%d\n", tfa_fb.chk_flag, tfa_fb.cmd_step, ret);
		tfa_fb.cmd_step = 0;
	}

	if ((!tfa98xx_2 && tfa_fb.cmd_step > READ_RESULT_FB_1) || (tfa_fb.cmd_step > READ_RESULT_FB_2)) {
		tfa_fb.cmd_step = 0;
		tfa_fb.damage_flag = 0;
	}

	if (tfa_fb.chk_flag & CHECK_SPEAKER_MASKS) {
		queue_delayed_work(tfa98xx->tfa98xx_wq, &tfa98xx->check_work,
			msecs_to_jiffies(g_step_delay[tfa_fb.cmd_step]));
	}

	return;
}

void oplus_tfa98xx_record_r0_f0_range(int r0_cal, int32_t f0_cal, int dev_idx)
{
	if ((dev_idx >= MONO) && (dev_idx < MAX_SPK_NUM)) {
		if (r0_cal == tfa_fb.r0_cal[dev_idx]) {
			return;
		}

		tfa_fb.r0_cal[dev_idx] = r0_cal;
		tfa_fb.f0_cal[dev_idx] = f0_cal;

		pr_info("spk dev_idx=%d, r0_cal = %d, f0_cal = %d\n", dev_idx, r0_cal, f0_cal);
	} else {
		pr_info("unsupport dev_idx=%d\n", dev_idx);
	}
}

void oplus_tfa98xx_check_reg(struct tfa98xx *tfa98xx)
{
	uint32_t id = 0;

	if ((tfa_fb.chk_flag == 0) || (NULL == tfa_fb.lock) || (tfa_fb.control_fb & BYPASS_PA_ERR_FB_10041)) {
		return;
	}

	if (!is_param_valid(tfa98xx)) {
		pr_err("parameter is not available\n");
		return;
	}

	id = tfa98xx->tfa->dev_idx;
	if (((1 << id) & tfa_fb.chk_flag) == 0) {
		return;
	}
	tfa_fb.chk_flag = (~(1 << id)) & tfa_fb.chk_flag;

	if ((tfa_fb.last_chk_reg != 0) && \
			ktime_before(ktime_get(), ktime_add_ms(tfa_fb.last_chk_reg, MM_FB_KEY_RATELIMIT_5MIN))) {
		return;
	}
	mutex_lock(tfa_fb.lock);
	tfa98xx_check_status_reg(tfa98xx);
	mutex_unlock(tfa_fb.lock);
}

void oplus_tfa98xx_queue_check_work(struct tfa98xx *tfa98xx)
{
	if (((tfa_fb.chk_flag & CHECK_SPEAKER_MASKS) != 0) && (0 == tfa_fb.queue_work_flag) &&
		!(tfa_fb.control_fb & BYPASS_SPK_ERR_FB_10042)) {
		if (tfa98xx && tfa98xx->tfa98xx_wq && tfa98xx->check_work.work.func && (0 == tfa_fb.queue_work_flag)) {
			pr_info("queue delay work for check speaker\n");
			tfa_fb.cmd_step = 0;
			tfa_fb.damage_flag = 0;
			queue_delayed_work(tfa98xx->tfa98xx_wq, &tfa98xx->check_work, CHECK_SPK_DELAY_TIME * HZ);
			tfa_fb.queue_work_flag = 1;
		}
	}
}

void oplus_tfa98xx_exit_check_work(struct tfa98xx *tfa98xx)
{
	if (tfa98xx && tfa98xx->check_work.wq && (1 == tfa_fb.queue_work_flag)) {
		pr_info("cancel delay work for check speaker\n");
		tfa_fb.chk_flag = (~CHECK_SPEAKER_MASKS) & tfa_fb.chk_flag;
		tfa_fb.cmd_step = 0;
		cancel_delayed_work_sync(&tfa98xx->check_work);
		tfa_fb.queue_work_flag = 0;
	}
}

void oplus_tfa98xx_feedback_init(struct tfa98xx *tfa98xx,
		struct list_head *dev_list, struct mutex *lock, int count)
{
	if (tfa98xx && tfa98xx->codec && tfa98xx->tfa && dev_list && lock) {
		if ((count > tfa_fb.pa_cnt) && (count <= MAX_SPK_NUM)) {
			tfa_fb.pa_cnt = count;
			pr_info("update pa_cnt = %d\n", tfa_fb.pa_cnt);
		}

		if (NULL == tfa_fb.lock) {
			snd_soc_add_component_controls(tfa98xx->codec,
					   tfa98xx_check_feedback,
					   ARRAY_SIZE(tfa98xx_check_feedback));
			INIT_DELAYED_WORK(&tfa98xx->check_work, tfa98xx_check_work);
			tfa_fb.plist = dev_list;
			tfa_fb.lock = lock;

			pr_info("event_id=%u, version:%s\n", OPLUS_AUDIO_EVENTID_SMARTPA_ERR, SMARTPA_ERR_FB_VERSION);
			pr_info("event_id=%u, version:%s\n", OPLUS_AUDIO_EVENTID_SPK_ERR, SPK_ERR_FB_VERSION);
		} else {
			tfa98xx->check_work.work.func = NULL;
			tfa98xx->check_work.wq = NULL;
		}
	}
	if (tfa98xx && tfa98xx->tfa && tfa98xx->tfa->dev_idx < tfa_fb.pa_cnt) {
		if (tfa98xx->tfa->min_mohms > 0 && tfa98xx->tfa->max_mohms > 0) {
			tfa_fb.r0_cal[tfa98xx->tfa->dev_idx] =
				(tfa98xx->tfa->min_mohms + tfa98xx->tfa->max_mohms) / 2;
			pr_info("init r0_cal[%d] = %u (avg of min=%u, max=%u)\n",
				tfa98xx->tfa->dev_idx,
				tfa_fb.r0_cal[tfa98xx->tfa->dev_idx],
				tfa98xx->tfa->min_mohms,
				tfa98xx->tfa->max_mohms);
		}
	}
}

int oplus_need_check_calib_values(void)
{
	return (tfa_fb.chk_flag == (CHECK_SPEAKER_MASKS + (1 << tfa_fb.pa_cnt) - 1));
}

void oplus_tfa98xx_clear_check_flag(void)
{
	tfa_fb.chk_flag = 0;
	pr_info("set chk_flag = 0\n");
}
