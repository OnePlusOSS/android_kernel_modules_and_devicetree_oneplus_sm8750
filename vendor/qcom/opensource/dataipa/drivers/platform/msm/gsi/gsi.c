// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022, 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/msm_gsi.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/msi.h>
#include <linux/smp.h>
#include "gsi.h"
#include "gsi_emulation.h"
#include "gsihal.h"

#include <asm/arch_timer.h>
#include <linux/sched/clock.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <soc/qcom/minidump.h>

#define CREATE_TRACE_POINTS
#include "gsi_trace.h"

#define GSI_CMD_TIMEOUT (5*HZ)
#define GSI_FC_CMD_TIMEOUT (2*GSI_CMD_TIMEOUT)
#define GSI_START_CMD_TIMEOUT_MS 1000
#define GSI_CMD_POLL_CNT 5
#define GSI_STOP_CMD_TIMEOUT_MS 200
#define GSI_MAX_CH_LOW_WEIGHT 15
#define GSI_IRQ_STORM_THR 5
#define GSI_FC_MAX_TIMEOUT 5

#define GSI_STOP_CMD_POLL_CNT 4
#define GSI_STOP_IN_PROC_CMD_POLL_CNT 2

#define GSI_RESET_WA_MIN_SLEEP 1000
#define GSI_RESET_WA_MAX_SLEEP 2000
#define GSI_CHNL_STATE_MAX_RETRYCNT 10

#define GSI_STTS_REG_BITS 32
#define GSI_MSB_MASK 0xFFFFFFFF00000000ULL
#define GSI_LSB_MASK 0x00000000FFFFFFFFULL
#define GSI_MSB(num) ((u32)((num & GSI_MSB_MASK) >> 32))
#define GSI_LSB(num) ((u32)(num & GSI_LSB_MASK))

#define GSI_FC_NUM_WORDS_PER_CHNL_SHRAM		(20)
#define GSI_FC_STATE_INDEX_SHRAM			(7)
#define GSI_FC_PENDING_MASK					(0x00080000)

#define GSI_NTN3_PENDING_DB_AFTER_RB_MASK 18
#define GSI_NTN3_PENDING_DB_AFTER_RB_SHIFT 1
/* FOR_SEQ_HIGH channel scratch: (((8 * (pipe_id * ctx_size + offset_lines)) + 4) / 4) */
#define GSI_GSI_SHRAM_n_EP_FOR_SEQ_HIGH_N_GET(ep_id) (((8 * (ep_id * 10 + 9)) + 4) / 4)

#ifndef CONFIG_DEBUG_FS
void gsi_debugfs_init(void)
{
}
#endif

static const struct of_device_id msm_gsi_match[] = {
	{ .compatible = "qcom,msm_gsi", },
	{ },
};


#if defined(CONFIG_IPA_EMULATION)
static bool running_emulation = true;
#else
static bool running_emulation;
#endif

struct gsi_ctx *gsi_ctx;
EXPORT_SYMBOL_GPL(gsi_ctx);


static union __packed gsi_channel_scratch __gsi_update_mhi_channel_scratch(
	unsigned long chan_hdl, struct __packed gsi_mhi_channel_scratch mscr);

static void __gsi_config_type_irq(int ee, uint32_t mask, uint32_t val)
{
	uint32_t curr;

	curr = gsihal_read_reg_n(GSI_EE_n_CNTXT_TYPE_IRQ_MSK, ee);

	gsihal_write_reg_n(GSI_EE_n_CNTXT_TYPE_IRQ_MSK, ee,
		(curr & ~mask) | (val & mask));
}

static void __gsi_config_ch_irq(int ee, uint32_t mask, uint32_t val)
{
	uint32_t curr;

	curr = gsihal_read_reg_n(GSI_EE_n_CNTXT_SRC_GSI_CH_IRQ_MSK, ee);

	gsihal_write_reg_n(GSI_EE_n_CNTXT_SRC_GSI_CH_IRQ_MSK, ee,
		(curr & ~mask) | (val & mask));
}

static void __gsi_config_all_ch_irq(int ee, uint32_t mask, uint32_t val)
{
	uint32_t curr, k, max_k;

	max_k = gsihal_get_bit_map_array_size();
	for (k = 0; k < max_k; k++)
	{
		curr = gsihal_read_reg_nk(GSI_EE_n_CNTXT_SRC_GSI_CH_IRQ_MSK_k, ee, k);

		gsihal_write_reg_nk(GSI_EE_n_CNTXT_SRC_GSI_CH_IRQ_MSK_k, ee, k,
			(curr & ~mask) | (val & mask));
	}
}

static void __gsi_config_evt_irq(int ee, uint32_t mask, uint32_t val)
{
	uint32_t curr;

	curr = gsihal_read_reg_n(GSI_EE_n_CNTXT_SRC_EV_CH_IRQ_MSK, ee);

	gsihal_write_reg_n(GSI_EE_n_CNTXT_SRC_EV_CH_IRQ_MSK, ee,
		(curr & ~mask) | (val & mask));
}

static void __gsi_config_all_evt_irq(int ee, uint32_t mask, uint32_t val)
{
	uint32_t curr, k, max_k;

	max_k = gsihal_get_bit_map_array_size();
	for (k = 0; k < max_k; k++)
	{
		curr = gsihal_read_reg_nk(GSI_EE_n_CNTXT_SRC_EV_CH_IRQ_MSK_k, ee, k);

		gsihal_write_reg_nk(GSI_EE_n_CNTXT_SRC_EV_CH_IRQ_MSK_k, ee, k,
			(curr & ~mask) | (val & mask));
	}
}

static void __gsi_config_ieob_irq(int ee, uint32_t mask, uint32_t val)
{
	uint32_t curr;

	curr = gsihal_read_reg_n(GSI_EE_n_CNTXT_SRC_IEOB_IRQ_MSK, ee);

	gsihal_write_reg_n(GSI_EE_n_CNTXT_SRC_IEOB_IRQ_MSK, ee,
		(curr & ~mask) | (val & mask));

	GSIDBG("current IEO_IRQ_MSK: 0x%x, change to: 0x%x\n",
		curr, ((curr & ~mask) | (val & mask)));
}

static void __gsi_config_all_ieob_irq(int ee, uint32_t mask, uint32_t val)
{
	uint32_t curr, k, max_k;

	max_k = gsihal_get_bit_map_array_size();
	for (k = 0; k < max_k; k++)
	{
		curr = gsihal_read_reg_nk(GSI_EE_n_CNTXT_SRC_IEOB_IRQ_MSK_k, ee, k);

		gsihal_write_reg_nk(GSI_EE_n_CNTXT_SRC_IEOB_IRQ_MSK_k, ee, k,
			(curr & ~mask) | (val & mask));
		GSIDBG("current IEO_IRQ_MSK: 0x%x, change to: 0x%x\n",
			curr, ((curr & ~mask) | (val & mask)));
	}
}

static void __gsi_config_ieob_irq_k(int ee, uint32_t k, uint32_t mask, uint32_t val)
{
	uint32_t curr;

	curr = gsihal_read_reg_nk(GSI_EE_n_CNTXT_SRC_IEOB_IRQ_MSK_k, ee, k);

		gsihal_write_reg_nk(GSI_EE_n_CNTXT_SRC_IEOB_IRQ_MSK_k, ee, k,
			(curr & ~mask) | (val & mask));
		GSIDBG("current IEO_IRQ_MSK: 0x%x, change to: 0x%x\n",
			curr, ((curr & ~mask) | (val & mask)));
}

static void __gsi_config_glob_irq(int ee, uint32_t mask, uint32_t val)
{
	uint32_t curr;

	curr = gsihal_read_reg_n(GSI_EE_n_CNTXT_GLOB_IRQ_EN, ee);

	gsihal_write_reg_n(GSI_EE_n_CNTXT_GLOB_IRQ_EN, ee,
		(curr & ~mask) | (val & mask));
}

static void __gsi_config_gen_irq(int ee, uint32_t mask, uint32_t val)
{
	uint32_t curr;

	curr = gsihal_read_reg_n(GSI_EE_n_CNTXT_GSI_IRQ_EN, ee);

	gsihal_write_reg_n(GSI_EE_n_CNTXT_GSI_IRQ_EN, ee,
		(curr & ~mask) | (val & mask));
}

static void gsi_channel_state_change_wait(unsigned long chan_hdl,
	struct gsi_chan_ctx *ctx,
	uint32_t tm, enum gsi_ch_cmd_opcode op)
{
	int poll_cnt;
	int gsi_pending_intr;
	int res;
	struct gsihal_reg_ctx_type_irq type;
	struct gsihal_reg_ch_k_cntxt_0 ch_k_cntxt_0;
	int ee = gsi_ctx->per.ee;
	enum gsi_chan_state curr_state = GSI_CHAN_STATE_NOT_ALLOCATED;
	int stop_in_proc_retry = 0;
	int stop_retry = 0;

	/*
	 * Start polling the GSI channel for
	 * duration = tm * GSI_CMD_POLL_CNT.
	 * We need to do polling of gsi state for improving debugability
	 * of gsi hw state.
	 */

	for (poll_cnt = 0;
		poll_cnt < GSI_CMD_POLL_CNT;
		poll_cnt++) {
		res = wait_for_completion_timeout(&ctx->compl,
			msecs_to_jiffies(tm));

		/* Interrupt received, return */
		if (res != 0)
			return;

		gsihal_read_reg_n_fields(GSI_EE_n_CNTXT_TYPE_IRQ, ee, &type);
		if (gsi_ctx->per.ver >= GSI_VER_3_0) {
			gsi_pending_intr = gsihal_read_reg_nk(
				GSI_EE_n_CNTXT_SRC_GSI_CH_IRQ_k,
				ee, gsihal_get_ch_reg_idx(chan_hdl));
		} else {
			gsi_pending_intr = gsihal_read_reg_n(
				GSI_EE_n_CNTXT_SRC_GSI_CH_IRQ, ee);
		}

		if (gsi_ctx->per.ver == GSI_VER_1_0) {
			gsihal_read_reg_nk_fields(GSI_EE_n_GSI_CH_k_CNTXT_0,
				ee, chan_hdl, &ch_k_cntxt_0);
			curr_state = ch_k_cntxt_0.chstate;
		}

		/* Update the channel state only if interrupt was raised
		 * on particular channel and also checking global interrupt
		 * is raised for channel control.
		 */
		if ((type.ch_ctrl) &&
			(gsi_pending_intr & gsihal_get_ch_reg_mask(chan_hdl))) {
			/*
			 * Check channel state here in case the channel is
			 * already started but interrupt is not yet received.
			 */

			gsihal_read_reg_nk_fields(GSI_EE_n_GSI_CH_k_CNTXT_0,
				ee, chan_hdl, &ch_k_cntxt_0);
			curr_state = ch_k_cntxt_0.chstate;
		}

		if (op == GSI_CH_START) {
			if (curr_state == GSI_CHAN_STATE_STARTED ||
				curr_state == GSI_CHAN_STATE_FLOW_CONTROL) {
				ctx->state = curr_state;
				return;
			}
		}

		if (op == GSI_CH_STOP) {
			if (curr_state == GSI_CHAN_STATE_STOPPED)
				stop_retry++;
			else if (curr_state == GSI_CHAN_STATE_STOP_IN_PROC)
				stop_in_proc_retry++;
		}

		/* if interrupt marked reg after poll count reaching to max
		 * keep loop to continue reach max stop proc and max stop count.
		 */
		if (stop_retry == 1 || stop_in_proc_retry == 1)
			poll_cnt = 0;

		/* If stop channel retry reached to max count
		 * clear the pending interrupt, if channel already stopped.
		 */
		if (stop_retry == GSI_STOP_CMD_POLL_CNT) {
			if (gsi_ctx->per.ver >= GSI_VER_3_0) {
				gsihal_write_reg_nk(GSI_EE_n_CNTXT_SRC_GSI_CH_IRQ_CLR_k,
					ee, gsihal_get_ch_reg_idx(chan_hdl),
					gsi_pending_intr);
			}
			else {
				gsihal_write_reg_n(GSI_EE_n_CNTXT_SRC_GSI_CH_IRQ_CLR,
				ee,
				gsi_pending_intr);
			}
			ctx->state = curr_state;
			return;
		}

		/* If channel state stop in progress case no need
		 * to wait for long time.
		 */
		if (stop_in_proc_retry == GSI_STOP_IN_PROC_CMD_POLL_CNT) {
			ctx->state = curr_state;
			return;
		}

		GSIDBG("GSI wait on chan_hld=%lu irqtyp=%u state=%u intr=%u\n",
			chan_hdl,
			type.ch_ctrl,
			ctx->state,
			gsi_pending_intr);
	}

	GSIDBG("invalidating the channel state when timeout happens\n");
	ctx->state = curr_state;
}

static void gsi_handle_ch_ctrl(int ee)
{
	uint32_t ch;
	int i, k, max_k;
	uint32_t ch_hdl;
	struct gsihal_reg_ch_k_cntxt_0 ch_k_cntxt_0;
	struct gsi_chan_ctx *ctx;

	if (gsi_ctx->per.ver >= GSI_VER_3_0) {
		max_k = gsihal_get_bit_map_array_size();
		for (k = 0; k < max_k; k++) {
			ch = gsihal_read_reg_nk(GSI_EE_n_CNTXT_SRC_GSI_CH_IRQ_k, ee, k);
			gsihal_write_reg_nk(GSI_EE_n_CNTXT_SRC_GSI_CH_IRQ_CLR_k, ee, k, ch);

			GSIDBG("ch %x\n", ch);
			for (i = 0; i < GSI_STTS_REG_BITS; i++) {
				if ((1 << i) & ch) {
					ch_hdl = i + (GSI_STTS_REG_BITS * k);
					if (ch_hdl >= gsi_ctx->max_ch ||
						ch_hdl >= GSI_CHAN_MAX) {
						GSIERR("invalid channel %d\n",
							ch_hdl);
						break;
					}

					ctx = &gsi_ctx->chan[ch_hdl];
					gsihal_read_reg_nk_fields(GSI_EE_n_GSI_CH_k_CNTXT_0,
						ee, ch_hdl, &ch_k_cntxt_0);
					ctx->state = ch_k_cntxt_0.chstate;

					GSIDBG("ch %u state updated to %u\n",
						ch_hdl, ctx->state);
					complete(&ctx->compl);
					gsi_ctx->ch_dbg[ch_hdl].cmd_completed++;
				}
			}
		}
	} else {
		ch = gsihal_read_reg_n(GSI_EE_n_CNTXT_SRC_GSI_CH_IRQ, ee);
		gsihal_write_reg_n(GSI_EE_n_CNTXT_SRC_GSI_CH_IRQ_CLR, ee, ch);

		GSIDBG("ch %x\n", ch);
		for (i = 0; i < GSI_STTS_REG_BITS; i++) {
			if ((1 << i) & ch) {
				if (i >= gsi_ctx->max_ch ||
					i >= GSI_CHAN_MAX) {
					GSIERR("invalid channel %d\n", i);
					break;
				}

				ctx = &gsi_ctx->chan[i];
				gsihal_read_reg_nk_fields(GSI_EE_n_GSI_CH_k_CNTXT_0,
					ee, i, &ch_k_cntxt_0);
				ctx->state = ch_k_cntxt_0.chstate;

				GSIDBG("ch %u state updated to %u\n", i,
					ctx->state);
				complete(&ctx->compl);
				gsi_ctx->ch_dbg[i].cmd_completed++;
			}
		}
	}
}

static void gsi_handle_ev_ctrl(int ee)
{
	uint32_t ch;
	int i, k;
	uint32_t evt_hdl, max_k;
	struct gsi_evt_ctx *ctx;
	struct gsihal_reg_ev_ch_k_cntxt_0 ev_ch_k_cntxt_0;

	if (gsi_ctx->per.ver >= GSI_VER_3_0) {
		max_k = gsihal_get_bit_map_array_size();
		for (k = 0; k < max_k; k++) {
			ch = gsihal_read_reg_nk(GSI_EE_n_CNTXT_SRC_EV_CH_IRQ_k, ee, k);
			gsihal_write_reg_nk(GSI_EE_n_CNTXT_SRC_EV_CH_IRQ_CLR_k, ee, k, ch);

			GSIDBG("ev %x\n", ch);
			for (i = 0; i < GSI_STTS_REG_BITS; i++) {
				if ((1 << i) & ch) {
					evt_hdl = i + (GSI_STTS_REG_BITS * k);
					if (evt_hdl >= gsi_ctx->max_ev ||
						evt_hdl >= GSI_EVT_RING_MAX) {
						GSIERR("invalid event %d\n",
							evt_hdl);
						break;
					}

					ctx = &gsi_ctx->evtr[evt_hdl];
					gsihal_read_reg_nk_fields(GSI_EE_n_EV_CH_k_CNTXT_0,
						ee, evt_hdl, &ev_ch_k_cntxt_0);
					ctx->state = ev_ch_k_cntxt_0.chstate;

					GSIDBG("evt %u state updated to %u\n",
						evt_hdl, ctx->state);
					complete(&ctx->compl);
				}
			}
		}
	} else {
		ch = gsihal_read_reg_n(GSI_EE_n_CNTXT_SRC_EV_CH_IRQ, ee);
		gsihal_write_reg_n(GSI_EE_n_CNTXT_SRC_EV_CH_IRQ_CLR, ee, ch);

		GSIDBG("ev %x\n", ch);
		for (i = 0; i < GSI_STTS_REG_BITS; i++) {
			if ((1 << i) & ch) {
				if (i >= gsi_ctx->max_ev ||
					i >= GSI_EVT_RING_MAX) {
					GSIERR("invalid event %d\n", i);
					break;
				}

				ctx = &gsi_ctx->evtr[i];
				gsihal_read_reg_nk_fields(GSI_EE_n_EV_CH_k_CNTXT_0,
					ee, i, &ev_ch_k_cntxt_0);
				ctx->state = ev_ch_k_cntxt_0.chstate;

				GSIDBG("evt %u state updated to %u\n", i,
					ctx->state);
				complete(&ctx->compl);
			}
		}
	}
}

static void gsi_handle_glob_err(uint32_t err)
{
	struct gsi_log_err *log;
	struct gsi_chan_ctx *ch;
	struct gsi_evt_ctx *ev;
	struct gsi_chan_err_notify chan_notify;
	struct gsi_evt_err_notify evt_notify;
	struct gsi_per_notify per_notify;
	enum gsi_err_type err_type;
	struct gsihal_reg_ch_k_cntxt_0 ch_k_cntxt_0;

	log = (struct gsi_log_err *)&err;
	GSIERR("log err_type=%u ee=%u idx=%u\n", log->err_type, log->ee,
			log->virt_idx);
	GSIERR("code=%u arg1=%u arg2=%u arg3=%u\n", log->code, log->arg1,
			log->arg2, log->arg3);

	err_type = log->err_type;
	/*
	 * These are errors thrown by hardware. We need
	 * BUG_ON() to capture the hardware state right
	 * when it is unexpected.
	 */
	switch (err_type) {
	case GSI_ERR_TYPE_GLOB:
		per_notify.evt_id = GSI_PER_EVT_GLOB_ERROR;
		per_notify.user_data = gsi_ctx->per.user_data;
		per_notify.data.err_desc = err & 0xFFFF;
		gsi_ctx->per.notify_cb(&per_notify);
		break;
	case GSI_ERR_TYPE_CHAN:
		if (WARN_ON(log->virt_idx >= gsi_ctx->max_ch)) {
			GSIERR("Unexpected ch %d\n", log->virt_idx);
			return;
		}

		ch = &gsi_ctx->chan[log->virt_idx];
		chan_notify.chan_user_data = ch->props.chan_user_data;
		chan_notify.err_desc = err & 0xFFFF;
		if (log->code == GSI_INVALID_TRE_ERR) {
			if (log->ee != gsi_ctx->per.ee) {
				GSIERR("unexpected EE in event %d\n", log->ee);
				GSI_ASSERT();
			}

			gsihal_read_reg_nk_fields(GSI_EE_n_GSI_CH_k_CNTXT_0,
				gsi_ctx->per.ee, log->virt_idx, &ch_k_cntxt_0);
			ch->state = ch_k_cntxt_0.chstate;
			GSIDBG("ch %u state updated to %u\n", log->virt_idx,
					ch->state);
			ch->stats.invalid_tre_error++;
			if (ch->state == GSI_CHAN_STATE_ERROR) {
				GSIERR("Unexpected channel state %d\n",
					ch->state);
				GSI_ASSERT();
			}
			chan_notify.evt_id = GSI_CHAN_INVALID_TRE_ERR;
		} else if (log->code == GSI_OUT_OF_BUFFERS_ERR) {
			if (log->ee != gsi_ctx->per.ee) {
				GSIERR("unexpected EE in event %d\n", log->ee);
				GSI_ASSERT();
			}
			chan_notify.evt_id = GSI_CHAN_OUT_OF_BUFFERS_ERR;
		} else if (log->code == GSI_OUT_OF_RESOURCES_ERR) {
			if (log->ee != gsi_ctx->per.ee) {
				GSIERR("unexpected EE in event %d\n", log->ee);
				GSI_ASSERT();
			}
			chan_notify.evt_id = GSI_CHAN_OUT_OF_RESOURCES_ERR;
			complete(&ch->compl);
		} else if (log->code == GSI_UNSUPPORTED_INTER_EE_OP_ERR) {
			chan_notify.evt_id =
				GSI_CHAN_UNSUPPORTED_INTER_EE_OP_ERR;
		} else if (log->code == GSI_NON_ALLOCATED_EVT_ACCESS_ERR) {
			if (log->ee != gsi_ctx->per.ee) {
				GSIERR("unexpected EE in event %d\n", log->ee);
				GSI_ASSERT();
			}
			chan_notify.evt_id =
				GSI_CHAN_NON_ALLOCATED_EVT_ACCESS_ERR;
		} else if (log->code == GSI_HWO_1_ERR) {
			if (log->ee != gsi_ctx->per.ee) {
				GSIERR("unexpected EE in event %d\n", log->ee);
				GSI_ASSERT();
			}
			chan_notify.evt_id = GSI_CHAN_HWO_1_ERR;
		} else {
			GSIERR("unexpected event log code %d\n", log->code);
			GSI_ASSERT();
		}
		ch->props.err_cb(&chan_notify);
		break;
	case GSI_ERR_TYPE_EVT:
		if (WARN_ON(log->virt_idx >= gsi_ctx->max_ev)) {
			GSIERR("Unexpected ev %d\n", log->virt_idx);
			return;
		}

		ev = &gsi_ctx->evtr[log->virt_idx];
		evt_notify.user_data = ev->props.user_data;
		evt_notify.err_desc = err & 0xFFFF;
		if (log->code == GSI_OUT_OF_BUFFERS_ERR) {
			if (log->ee != gsi_ctx->per.ee) {
				GSIERR("unexpected EE in event %d\n", log->ee);
				GSI_ASSERT();
			}
			evt_notify.evt_id = GSI_EVT_OUT_OF_BUFFERS_ERR;
		} else if (log->code == GSI_OUT_OF_RESOURCES_ERR) {
			if (log->ee != gsi_ctx->per.ee) {
				GSIERR("unexpected EE in event %d\n", log->ee);
				GSI_ASSERT();
			}
			evt_notify.evt_id = GSI_EVT_OUT_OF_RESOURCES_ERR;
			complete(&ev->compl);
		} else if (log->code == GSI_UNSUPPORTED_INTER_EE_OP_ERR) {
			evt_notify.evt_id = GSI_EVT_UNSUPPORTED_INTER_EE_OP_ERR;
		} else if (log->code == GSI_EVT_RING_EMPTY_ERR) {
			if (log->ee != gsi_ctx->per.ee) {
				GSIERR("unexpected EE in event %d\n", log->ee);
				GSI_ASSERT();
			}
			evt_notify.evt_id = GSI_EVT_EVT_RING_EMPTY_ERR;
		} else {
			GSIERR("unexpected event log code %d\n", log->code);
			GSI_ASSERT();
		}
		ev->props.err_cb(&evt_notify);
		break;
	}
}

static void gsi_handle_gp_int1(void)
{
	complete(&gsi_ctx->gen_ee_cmd_compl);
}

static void gsi_handle_glob_ee(int ee)
{
	uint32_t val;
	uint32_t err;
	struct gsi_per_notify notify;
	uint32_t clr = ~0;
	struct gsihal_reg_cntxt_glob_irq_stts cntxt_glob_irq_stts;

	val = gsihal_read_reg_n_fields(GSI_EE_n_CNTXT_GLOB_IRQ_STTS,
		ee, &cntxt_glob_irq_stts);

	notify.user_data = gsi_ctx->per.user_data;

	if(cntxt_glob_irq_stts.error_int) {
		err = gsihal_read_reg_n(GSI_EE_n_ERROR_LOG, ee);
		if (gsi_ctx->per.ver >= GSI_VER_1_2)
			gsihal_write_reg_n(GSI_EE_n_ERROR_LOG, ee, 0);
		gsihal_write_reg_n(GSI_EE_n_ERROR_LOG_CLR, ee, clr);
		gsi_handle_glob_err(err);
	}

	if (cntxt_glob_irq_stts.gp_int1)
		gsi_handle_gp_int1();

	if (cntxt_glob_irq_stts.gp_int2) {
		notify.evt_id = GSI_PER_EVT_GLOB_GP2;
		gsi_ctx->per.notify_cb(&notify);
	}

	if (cntxt_glob_irq_stts.gp_int3) {
		notify.evt_id = GSI_PER_EVT_GLOB_GP3;
		gsi_ctx->per.notify_cb(&notify);
	}

	gsihal_write_reg_n(GSI_EE_n_CNTXT_GLOB_IRQ_CLR, ee, val);
}

static void gsi_incr_ring_wp(struct gsi_ring_ctx *ctx)
{
	ctx->wp_local += ctx->elem_sz;
	if (ctx->wp_local == ctx->end)
		ctx->wp_local = ctx->base;
}

static void gsi_incr_ring_rp(struct gsi_ring_ctx *ctx)
{
	ctx->rp_local += ctx->elem_sz;
	if (ctx->rp_local == ctx->end)
		ctx->rp_local = ctx->base;
}

uint16_t gsi_find_idx_from_addr(struct gsi_ring_ctx *ctx, uint64_t addr)
{
	WARN_ON(addr < ctx->base || addr >= ctx->end);
	return (uint32_t)(addr - ctx->base) / ctx->elem_sz;
}

static uint16_t gsi_get_complete_num(struct gsi_ring_ctx *ctx, uint64_t addr1,
		uint64_t addr2)
{
	uint32_t addr_diff;

	GSIDBG_LOW("gsi base addr 0x%llx end addr 0x%llx\n",
		ctx->base, ctx->end);

	if (addr1 < ctx->base || addr1 >= ctx->end) {
		GSIERR("address = 0x%llx not in range\n", addr1);
		GSI_ASSERT();
	}

	if (addr2 < ctx->base || addr2 >= ctx->end) {
		GSIERR("address = 0x%llx not in range\n", addr2);
		GSI_ASSERT();
	}

	addr_diff = (uint32_t)(addr2 - addr1);
	if (addr1 < addr2)
		return addr_diff / ctx->elem_sz;
	else
		return (addr_diff + ctx->len) / ctx->elem_sz;
}

static void gsi_process_chan(struct gsi_xfer_compl_evt *evt,
		struct gsi_chan_xfer_notify *notify, bool callback)
{
	uint32_t ch_id;
	struct gsi_chan_ctx *ch_ctx;
	uint16_t rp_idx;
	uint64_t rp;

	ch_id = evt->chid;
	if (WARN_ON(ch_id >= gsi_ctx->max_ch)) {
		GSIERR("Unexpected ch %d\n", ch_id);
		return;
	}

	ch_ctx = &gsi_ctx->chan[ch_id];
	if (WARN_ON(ch_ctx->props.prot != GSI_CHAN_PROT_GPI &&
		ch_ctx->props.prot != GSI_CHAN_PROT_GCI))
		return;

	if (evt->type != GSI_XFER_COMPL_TYPE_GCI) {
		rp = evt->xfer_ptr;

		if (ch_ctx->ring.rp_local != rp) {
			ch_ctx->stats.completed +=
				gsi_get_complete_num(&ch_ctx->ring,
				ch_ctx->ring.rp_local, rp);
			ch_ctx->ring.rp_local = rp;
		}

		/*
		 * Increment RP local only in polling context to avoid
		 * sys len mismatch.
		 */
		if (!callback || (ch_ctx->props.dir == GSI_CHAN_DIR_TO_GSI &&
			!ch_ctx->props.tx_poll))
			/* the element at RP is also processed */
			gsi_incr_ring_rp(&ch_ctx->ring);

		ch_ctx->ring.rp = ch_ctx->ring.rp_local;
		rp_idx = gsi_find_idx_from_addr(&ch_ctx->ring, rp);
		notify->veid = GSI_VEID_DEFAULT;
	} else {
		rp_idx = evt->cookie;
		notify->veid = evt->veid;
	}


	WARN_ON(!ch_ctx->user_data[rp_idx].valid);
	notify->xfer_user_data = ch_ctx->user_data[rp_idx].p;
	/*
	 * In suspend just before stopping the channel possible to receive
	 * the IEOB interrupt and xfer pointer will not be processed in this
	 * mode and moving channel poll mode. In resume after starting the
	 * channel will receive the IEOB interrupt and xfer pointer will be
	 * overwritten. To avoid this process all data in polling context.
	 */
	if (!callback || (ch_ctx->props.dir == GSI_CHAN_DIR_TO_GSI &&
		!ch_ctx->props.tx_poll)) {
		ch_ctx->stats.completed++;
		ch_ctx->user_data[rp_idx].valid = false;
	}

	notify->chan_user_data = ch_ctx->props.chan_user_data;
	notify->evt_id = evt->code;
	notify->bytes_xfered = evt->len;

	if (callback) {
		if (atomic_read(&ch_ctx->poll_mode)) {
			GSIERR("Calling client callback in polling mode\n");
			WARN_ON(1);
		}
		ch_ctx->props.xfer_cb(notify);
	}
}

static void gsi_process_evt_re(struct gsi_evt_ctx *ctx,
		struct gsi_chan_xfer_notify *notify, bool callback)
{
	struct gsi_xfer_compl_evt *evt;
	struct gsi_chan_ctx *ch_ctx;

	/*
	 * RMB before reading event ring shared b/w IPA h/w & driver
	 * ordering between IPA h/w store and CPU load.
	 */
	dma_rmb();
	evt = (struct gsi_xfer_compl_evt *)(ctx->ring.base_va +
			ctx->ring.rp_local - ctx->ring.base);
	gsi_process_chan(evt, notify, callback);
	/*
	 * Increment RP local only in polling context to avoid
	 * sys len mismatch.
	 */
	ch_ctx = &gsi_ctx->chan[evt->chid];
	if (callback && (ch_ctx->props.dir == GSI_CHAN_DIR_FROM_GSI ||
		ch_ctx->props.tx_poll))
		return;
	gsi_incr_ring_rp(&ctx->ring);
	/* recycle this element */
	gsi_incr_ring_wp(&ctx->ring);
	ctx->stats.completed++;
}

static void gsi_ring_evt_doorbell(struct gsi_evt_ctx *ctx)
{
	uint32_t val;

	ctx->ring.wp = ctx->ring.wp_local;
	val = GSI_LSB(ctx->ring.wp_local);
	gsihal_write_reg_nk(GSI_EE_n_EV_CH_k_DOORBELL_0,
		gsi_ctx->per.ee, ctx->id, val);
}

void gsi_ring_evt_doorbell_polling_mode(unsigned long chan_hdl) {
	struct gsi_evt_ctx *ctx;

	ctx = gsi_ctx->chan[chan_hdl].evtr;
	gsi_ring_evt_doorbell(ctx);
}
EXPORT_SYMBOL(gsi_ring_evt_doorbell_polling_mode);

static void gsi_ring_chan_doorbell(struct gsi_chan_ctx *ctx)
{
	uint32_t val;

	/*
	 * allocate new events for this channel first
	 * before submitting the new TREs.
	 * for TO_GSI channels the event ring doorbell is rang as part of
	 * interrupt handling.
	 */
	if (ctx->evtr && ctx->props.dir == GSI_CHAN_DIR_FROM_GSI)
		gsi_ring_evt_doorbell(ctx->evtr);
	ctx->ring.wp = ctx->ring.wp_local;

	val = GSI_LSB(ctx->ring.wp_local);
	gsihal_write_reg_nk(GSI_EE_n_GSI_CH_k_DOORBELL_0,
		gsi_ctx->per.ee, ctx->props.ch_id, val);
}

static bool check_channel_polling(struct gsi_evt_ctx* ctx) {
	/* For shared event rings both channels will be marked */
	return atomic_read(&ctx->chan[0]->poll_mode);
}

static void gsi_handle_ieob(int ee)
{
	uint32_t ch, evt_hdl;
	int i, k, max_k;
	uint64_t rp;
	struct gsi_evt_ctx *ctx;
	struct gsi_chan_xfer_notify notify;
	unsigned long flags;
	unsigned long cntr;
	uint32_t msk;
	bool empty;

	if (gsi_ctx->per.ver >= GSI_VER_3_0) {
		max_k = gsihal_get_bit_map_array_size();
		for (k = 0; k < max_k; k++) {
			ch = gsihal_read_reg_nk(GSI_EE_n_CNTXT_SRC_IEOB_IRQ_k, ee, k);
			msk = gsihal_read_reg_nk(GSI_EE_n_CNTXT_SRC_IEOB_IRQ_MSK_k, ee, k);
			gsihal_write_reg_nk(GSI_EE_n_CNTXT_SRC_IEOB_IRQ_CLR_k, ee, k, ch & msk);

			if (trace_gsi_qtimer_enabled())
			{
				uint64_t qtimer = 0;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
				qtimer = arch_timer_read_cntpct_el0();
#endif
				trace_gsi_qtimer(qtimer, false, 0, ch, msk);
			}

			for (i = 0; i < GSI_STTS_REG_BITS; i++) {
				if ((1 << i) & ch & msk) {
					evt_hdl = i + (GSI_STTS_REG_BITS * k);
					if (evt_hdl >= gsi_ctx->max_ev ||
					    evt_hdl >= GSI_EVT_RING_MAX) {
						GSIERR("invalid event %d\n",
						       evt_hdl);
						break;
					}
					ctx = &gsi_ctx->evtr[evt_hdl];

					/*
					 * Don't handle MSI interrupts, only handle IEOB
					 * IRQs
					 */
					if (ctx->props.intr == GSI_INTR_MSI)
						continue;

					if (ctx->props.intf !=
					    GSI_EVT_CHTYPE_GPI_EV) {
						GSIERR("Unexpected irq intf %d\n",
						       ctx->props.intf);
						GSI_ASSERT();
					}
					spin_lock_irqsave(&ctx->ring.slock,
							  flags);
check_again_v3_0:
					cntr = 0;
					empty = true;
					rp = ctx->props.gsi_read_event_ring_rp(
						&ctx->props, ctx->id, ee);
					rp |= ctx->ring.rp & GSI_MSB_MASK;

					ctx->ring.rp = rp;
					while (ctx->ring.rp_local != rp) {
						++cntr;
						if (check_channel_polling(ctx)) {
								cntr = 0;
								break;
						}
						gsi_process_evt_re(ctx, &notify,
								   true);
						empty = false;
					}
					if (!empty)
						gsi_ring_evt_doorbell(ctx);
					if (cntr != 0)
						goto check_again_v3_0;
					spin_unlock_irqrestore(&ctx->ring.slock,
							       flags);
				}
			}
		}
	} else {
		ch = gsihal_read_reg_n(GSI_EE_n_CNTXT_SRC_IEOB_IRQ, ee);
		msk = gsihal_read_reg_n(GSI_EE_n_CNTXT_SRC_IEOB_IRQ_MSK, ee);
		gsihal_write_reg_n(GSI_EE_n_CNTXT_SRC_IEOB_IRQ_CLR, ee, ch & msk);

		for (i = 0; i < GSI_STTS_REG_BITS; i++) {
			if ((1 << i) & ch & msk) {
				if (i >= gsi_ctx->max_ev ||
				    i >= GSI_EVT_RING_MAX) {
					GSIERR("invalid event %d\n", i);
					break;
				}
				ctx = &gsi_ctx->evtr[i];

				/*
				 * Don't handle MSI interrupts, only handle IEOB
				 * IRQs
				 */
				if (ctx->props.intr == GSI_INTR_MSI)
					continue;

				if (ctx->props.intf != GSI_EVT_CHTYPE_GPI_EV) {
					GSIERR("Unexpected irq intf %d\n",
					       ctx->props.intf);
					GSI_ASSERT();
				}
				spin_lock_irqsave(&ctx->ring.slock, flags);
			check_again:
				cntr = 0;
				empty = true;
				rp = ctx->props.gsi_read_event_ring_rp(
					&ctx->props, ctx->id, ee);
				rp |= ctx->ring.rp & GSI_MSB_MASK;

				ctx->ring.rp = rp;
				while (ctx->ring.rp_local != rp) {
					++cntr;
					if (check_channel_polling(ctx)) {
						cntr = 0;
						break;
					}
					gsi_process_evt_re(ctx, &notify, true);
					empty = false;
				}
				if (!empty)
					gsi_ring_evt_doorbell(ctx);
				if (cntr != 0)
					goto check_again;
				spin_unlock_irqrestore(&ctx->ring.slock, flags);
			}
		}
	}
}

static void gsi_handle_inter_ee_ch_ctrl(int ee)
{
	uint32_t ch, ch_hdl;
	int i, k, max_k;

	if (gsi_ctx->per.ver >= GSI_VER_3_0) {
		max_k = gsihal_get_bit_map_array_size();
		for (k = 0; k < max_k; k++) {
			ch = gsihal_read_reg_nk(GSI_INTER_EE_n_SRC_GSI_CH_IRQ_k, ee, k);
			gsihal_write_reg_nk(GSI_INTER_EE_n_SRC_GSI_CH_IRQ_k, ee, k, ch);

			for (i = 0; i < GSI_STTS_REG_BITS; i++) {
				if ((1 << i) & ch) {
					ch_hdl = i + (GSI_STTS_REG_BITS * k);
					/* not currently expected */
					GSIERR("ch %u was inter-EE changed\n", ch_hdl);
				}
			}
		}
	} else {
		ch = gsihal_read_reg_n(GSI_INTER_EE_n_SRC_GSI_CH_IRQ, ee);
		gsihal_write_reg_n(GSI_INTER_EE_n_SRC_GSI_CH_IRQ, ee, ch);

		for (i = 0; i < GSI_STTS_REG_BITS; i++) {
			if ((1 << i) & ch) {
				/* not currently expected */
				GSIERR("ch %u was inter-EE changed\n", i);
			}
		}
	}
}

static void gsi_handle_inter_ee_ev_ctrl(int ee)
{
	uint32_t ch, evt_hdl;
	int i, k, max_k;

	if (gsi_ctx->per.ver >= GSI_VER_3_0) {
		max_k = gsihal_get_bit_map_array_size();
		for (k = 0; k < max_k; k++) {
			ch = gsihal_read_reg_nk(GSI_INTER_EE_n_SRC_EV_CH_IRQ_k, ee, k);
			gsihal_write_reg_nk(GSI_INTER_EE_n_SRC_EV_CH_IRQ_CLR_k, ee, k, ch);

			for (i = 0; i < GSI_STTS_REG_BITS; i++) {
				if ((1 << i) & ch) {
					evt_hdl = i + (GSI_STTS_REG_BITS * k);
					/* not currently expected */
					GSIERR("evt %u was inter-EE changed\n",
					       evt_hdl);
				}
			}
		}
	} else {
		ch = gsihal_read_reg_n(GSI_INTER_EE_n_SRC_EV_CH_IRQ, ee);
		gsihal_write_reg_n(GSI_INTER_EE_n_SRC_EV_CH_IRQ_CLR, ee, ch);

		for (i = 0; i < GSI_STTS_REG_BITS; i++) {
			if ((1 << i) & ch) {
				/* not currently expected */
				GSIERR("evt %u was inter-EE changed\n", i);
			}
		}
	}
}

static void gsi_handle_general(int ee)
{
	uint32_t val;
	struct gsi_per_notify notify;
	struct gsihal_reg_cntxt_gsi_irq_stts gsi_irq_stts;

	val = gsihal_read_reg_n_fields(GSI_EE_n_CNTXT_GSI_IRQ_STTS,
		ee, &gsi_irq_stts);

	notify.user_data = gsi_ctx->per.user_data;

	if (gsi_irq_stts.gsi_mcs_stack_ovrflow)
		notify.evt_id = GSI_PER_EVT_GENERAL_MCS_STACK_OVERFLOW;

	if (gsi_irq_stts.gsi_cmd_fifo_ovrflow)
		notify.evt_id = GSI_PER_EVT_GENERAL_CMD_FIFO_OVERFLOW;

	if (gsi_irq_stts.gsi_bus_error)
		notify.evt_id = GSI_PER_EVT_GENERAL_BUS_ERROR;

	if (gsi_irq_stts.gsi_break_point)
		notify.evt_id = GSI_PER_EVT_GENERAL_BREAK_POINT;

	if (gsi_ctx->per.notify_cb)
		gsi_ctx->per.notify_cb(&notify);

	gsihal_write_reg_n(GSI_EE_n_CNTXT_GSI_IRQ_CLR, ee, val);
}

static void gsi_handle_irq(void)
{
	uint32_t type;
	int ee = gsi_ctx->per.ee;
	int index;
	struct gsihal_reg_ctx_type_irq ctx_type_irq;

	while (1) {
		if (!gsi_ctx->per.clk_status_cb())
			break;
		type = gsihal_read_reg_n_fields(GSI_EE_n_CNTXT_TYPE_IRQ,
			ee, &ctx_type_irq);

		if (!type)
			break;

		GSIDBG_LOW("type 0x%x\n", type);
		index = gsi_ctx->gsi_isr_cache_index;
		gsi_ctx->gsi_isr_cache[index].timestamp =
			sched_clock();
		gsi_ctx->gsi_isr_cache[index].qtimer =
			__arch_counter_get_cntvct();
		gsi_ctx->gsi_isr_cache[index].interrupt_type = type;
		gsi_ctx->gsi_isr_cache_index++;
		if (gsi_ctx->gsi_isr_cache_index == GSI_ISR_CACHE_MAX)
			gsi_ctx->gsi_isr_cache_index = 0;

		if(ctx_type_irq.ch_ctrl) {
			gsi_handle_ch_ctrl(ee);
			break;
		}

		if (ctx_type_irq.ev_ctrl) {
			gsi_handle_ev_ctrl(ee);
			break;
		}

		if (ctx_type_irq.glob_ee)
			gsi_handle_glob_ee(ee);

		if (ctx_type_irq.ieob)
			gsi_handle_ieob(ee);

		if (ctx_type_irq.inter_ee_ch_ctrl)
			gsi_handle_inter_ee_ch_ctrl(ee);

		if (ctx_type_irq.inter_ee_ev_ctrl)
			gsi_handle_inter_ee_ev_ctrl(ee);

		if (ctx_type_irq.general)
			gsi_handle_general(ee);

	}
}

static irqreturn_t gsi_isr(int irq, void *ctxt)
{
	if (gsi_ctx->per.req_clk_cb) {
		bool granted = false;

		gsi_ctx->per.req_clk_cb(gsi_ctx->per.user_data, &granted);
		if (granted) {
			gsi_handle_irq();
			gsi_ctx->per.rel_clk_cb(gsi_ctx->per.user_data);
		}
	} else if (!gsi_ctx->per.clk_status_cb()) {
	/* we only want to capture the gsi isr storm here */
		if (atomic_read(&gsi_ctx->num_unclock_irq) ==
			GSI_IRQ_STORM_THR)
			gsi_ctx->per.enable_clk_bug_on();
		atomic_inc(&gsi_ctx->num_unclock_irq);
		return IRQ_HANDLED;
	} else {
		atomic_set(&gsi_ctx->num_unclock_irq, 0);
		gsi_handle_irq();
	}
	return IRQ_HANDLED;
}

static irqreturn_t gsi_msi_isr(int irq, void *ctxt)
{
	int ee = gsi_ctx->per.ee;
	uint64_t rp;
	struct gsi_chan_xfer_notify notify;
	unsigned long flags;
	unsigned long cntr;
	bool empty;
	uint8_t evt;
	unsigned long msi;
	struct gsi_evt_ctx *evt_ctxt;

	/* Determine which event channel to handle */
	for (msi = 0; msi < gsi_ctx->msi.num; msi++) {
		if (gsi_ctx->msi.irq[msi] == irq)
			break;
	}

	evt = gsi_ctx->msi.evt[msi];
	evt_ctxt = &gsi_ctx->evtr[evt];

	if (trace_gsi_qtimer_enabled()) {
		uint64_t qtimer = 0;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
		qtimer = arch_timer_read_cntpct_el0();
#endif
		trace_gsi_qtimer(qtimer, true, evt, 0, 0);
	}

	if (evt_ctxt->props.intf != GSI_EVT_CHTYPE_GPI_EV) {
		GSIERR("Unexpected irq intf %d\n",
			evt_ctxt->props.intf);
		GSI_ASSERT();
	}

	/* Clearing IEOB irq if there are any genereated for MSI channel */
	gsihal_write_reg_nk(GSI_EE_n_CNTXT_SRC_IEOB_IRQ_CLR_k, ee,
		gsihal_get_ch_reg_idx(evt_ctxt->id),
		gsihal_get_ch_reg_mask(evt_ctxt->id));
	spin_lock_irqsave(&evt_ctxt->ring.slock, flags);
check_again:
	cntr = 0;
	empty = true;
	rp = evt_ctxt->props.gsi_read_event_ring_rp(&evt_ctxt->props,
			evt_ctxt->id, ee);
	rp |= evt_ctxt->ring.rp & 0xFFFFFFFF00000000;

	evt_ctxt->ring.rp = rp;
	while (evt_ctxt->ring.rp_local != rp) {
		++cntr;
		if (evt_ctxt->props.exclusive &&
			atomic_read(&evt_ctxt->chan[0]->poll_mode)) {
			cntr = 0;
			break;
		}
		gsi_process_evt_re(evt_ctxt, &notify, true);
		empty = false;
	}
	if (!empty)
		gsi_ring_evt_doorbell(evt_ctxt);
	if (cntr != 0)
		goto check_again;
	spin_unlock_irqrestore(&evt_ctxt->ring.slock, flags);
	return IRQ_HANDLED;
}

static uint32_t gsi_get_max_channels(enum gsi_ver ver)
{
	uint32_t max_ch = 0;
	struct gsihal_reg_hw_param hw_param;
	struct gsihal_reg_hw_param2 hw_param2;

	switch (ver) {
	case GSI_VER_ERR:
	case GSI_VER_MAX:
		GSIERR("GSI version is not supported %d\n", ver);
		WARN_ON(1);
		break;
	case GSI_VER_1_0:
		gsihal_read_reg_n_fields(GSI_EE_n_GSI_HW_PARAM,
			gsi_ctx->per.ee, &hw_param);
		max_ch = hw_param.gsi_ch_num;
		break;
	case GSI_VER_1_2:
		gsihal_read_reg_n_fields(GSI_EE_n_GSI_HW_PARAM_0,
			gsi_ctx->per.ee, &hw_param);
		max_ch = hw_param.gsi_ch_num;
		break;
	default:
		gsihal_read_reg_n_fields(GSI_EE_n_GSI_HW_PARAM_2,
			gsi_ctx->per.ee, &hw_param2);
		max_ch = hw_param2.gsi_num_ch_per_ee;
		break;
	}

	GSIDBG("max channels %d\n", max_ch);

	return max_ch;
}

static uint32_t gsi_get_max_event_rings(enum gsi_ver ver)
{
	uint32_t max_ev = 0;
	struct gsihal_reg_hw_param hw_param;
	struct gsihal_reg_hw_param2 hw_param2;
	struct gsihal_reg_hw_param4 hw_param4;

	switch (ver) {
	case GSI_VER_ERR:
	case GSI_VER_MAX:
		GSIERR("GSI version is not supported %d\n", ver);
		WARN_ON(1);
		break;
	case GSI_VER_1_0:
		gsihal_read_reg_n_fields(GSI_EE_n_GSI_HW_PARAM,
			gsi_ctx->per.ee, &hw_param);
		max_ev = hw_param.gsi_ev_ch_num;
		break;
	case GSI_VER_1_2:
		gsihal_read_reg_n_fields(GSI_EE_n_GSI_HW_PARAM_0,
			gsi_ctx->per.ee, &hw_param);
		max_ev = hw_param.gsi_ev_ch_num;
		break;
	case GSI_VER_3_0:
	case GSI_VER_5_2:
	case GSI_VER_5_5:
		gsihal_read_reg_n_fields(GSI_EE_n_GSI_HW_PARAM_4,
			gsi_ctx->per.ee, &hw_param4);
		max_ev = hw_param4.gsi_num_ev_per_ee;
		break;
	default:
		gsihal_read_reg_n_fields(GSI_EE_n_GSI_HW_PARAM_2,
			gsi_ctx->per.ee, &hw_param2);
		max_ev = hw_param2.gsi_num_ev_per_ee;
		break;
	}

	GSIDBG("max event rings %d\n", max_ev);

	return max_ev;
}
int gsi_complete_clk_grant(unsigned long dev_hdl)
{
	unsigned long flags;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (!gsi_ctx->per_registered) {
		GSIERR("no client registered\n");
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (dev_hdl != (uintptr_t)gsi_ctx) {
		GSIERR("bad params dev_hdl=0x%lx gsi_ctx=0x%pK\n", dev_hdl,
				gsi_ctx);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	spin_lock_irqsave(&gsi_ctx->slock, flags);
	gsi_handle_irq();
	gsi_ctx->per.rel_clk_cb(gsi_ctx->per.user_data);
	spin_unlock_irqrestore(&gsi_ctx->slock, flags);

	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_complete_clk_grant);

int gsi_map_base(phys_addr_t gsi_base_addr, u32 gsi_size, enum gsi_ver ver)
{
	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	gsi_ctx->base = devm_ioremap(
		gsi_ctx->dev, gsi_base_addr, gsi_size);

	if (!gsi_ctx->base) {
		GSIERR("failed to map access to GSI HW\n");
		return -GSI_STATUS_RES_ALLOC_FAILURE;
	}

	GSIDBG("GSI base(%pa) mapped to (%pK) with len (0x%x)\n",
		&gsi_base_addr,
		gsi_ctx->base,
		gsi_size);

	/* initialize HAL before accessing any register */
	gsihal_init(ver, gsi_ctx->base);

	return 0;
}
EXPORT_SYMBOL(gsi_map_base);

int gsi_unmap_base(void)
{
	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (!gsi_ctx->base) {
		GSIERR("access to GSI HW has not been mapped\n");
		return -GSI_STATUS_INVALID_PARAMS;
	}

	devm_iounmap(gsi_ctx->dev, gsi_ctx->base);

	gsi_ctx->base = NULL;

	return 0;
}
EXPORT_SYMBOL(gsi_unmap_base);

static void __gsi_msi_write_msg(struct msi_desc *desc, struct msi_msg *msg)
{
	u16 msi = 0;

	if (IS_ERR_OR_NULL(desc) || IS_ERR_OR_NULL(msg) || IS_ERR_OR_NULL(gsi_ctx))
		BUG();

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
	msi = desc->msi_index;
#else
	msi = desc->platform.msi_index;
#endif

	/* MSI should be valid and unallocated */
	if ((msi >= gsi_ctx->msi.num) || (test_bit(msi, gsi_ctx->msi.allocated)))
		BUG();

	/* Save the message for later use */
	memcpy(&gsi_ctx->msi.msg[msi], msg, sizeof(*msg));

	dev_notice(gsi_ctx->dev,
		"saved msi %u msg data %u addr 0x%08x%08x\n", msi,
		msg->data, msg->address_hi, msg->address_lo);

	/* Single MSI control is used. So MSI address will be same. */
	if (!gsi_ctx->msi_addr_set) {
		gsi_ctx->msi_addr = gsi_ctx->msi.msg[msi].address_hi;
		gsi_ctx->msi_addr = (gsi_ctx->msi_addr << 32) |
			gsi_ctx->msi.msg[msi].address_lo;
		gsi_ctx->msi_addr_set = true;
	}

	GSIDBG("saved msi %u msg data %u addr 0x%08x%08x, MSI:0x%llx\n", msi,
		msg->data, msg->address_hi, msg->address_lo, gsi_ctx->msi_addr);
}

static int __gsi_request_msi_irq(unsigned long msi)
{
	int result = 0;

	/* Ensure this is not already allocated */
	if (test_bit((int)msi, gsi_ctx->msi.allocated)) {
		GSIERR("MSI %lu already allocated\n", msi);
		return -GSI_STATUS_ERROR;
	}

	/* Request MSI IRQ
	 * NOTE: During the call to devm_request_irq, the
	 * __gsi_msi_write_msg callback is triggered.
	 */
	result = devm_request_irq(gsi_ctx->dev, gsi_ctx->msi.irq[msi],
			(irq_handler_t)gsi_msi_isr, IRQF_TRIGGER_NONE,
			"gsi_msi", gsi_ctx);

	if (result) {
		GSIERR("failed to register msi irq %u idx %lu\n",
			gsi_ctx->msi.irq[msi], msi);
		return -GSI_STATUS_ERROR;
	}

	set_bit(msi, gsi_ctx->msi.allocated);
	return result;
}

static int __gsi_allocate_msis(void)
{
	int result = 0;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 18, 0))
	struct msi_desc *desc = NULL;
#endif
	size_t size = 0;

	/* Allocate all MSIs */
	GSIDBG("gsi_ctx->dev = %p, gsi_ctx->msi.num = %d", gsi_ctx->dev, gsi_ctx->msi.num);
#if (KERNEL_VERSION(6, 8, 0) > LINUX_VERSION_CODE)
	result = platform_msi_domain_alloc_irqs(gsi_ctx->dev, gsi_ctx->msi.num,
			__gsi_msi_write_msg);
#else
	result = platform_device_msi_init_and_alloc_irqs(gsi_ctx->dev, gsi_ctx->msi.num,
			__gsi_msi_write_msg);
#endif
	if (result) {
		GSIERR("error allocating platform MSIs - %d\n", result);
		return -GSI_STATUS_ERROR;
	}
	GSIDBG("MSI allocating is succesful\n");

	/* Loop through the allocated MSIs and save the info, then
	 * request the IRQ.
	 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
	for (unsigned long msi = 0; msi < gsi_ctx->msi.num; msi++) {
		/* Save IRQ */
		gsi_ctx->msi.irq[msi] = msi_get_virq(gsi_ctx->dev, msi);
#else
	for_each_msi_entry(desc, gsi_ctx->dev) {
		unsigned long msi = desc->platform.msi_index;

		/* Ensure a valid index */
		if (msi >= gsi_ctx->msi.num) {
			GSIERR("error invalid MSI %lu\n", msi);
			result = -GSI_STATUS_ERROR;
			goto err_free_msis;
		}

		/* Save IRQ */
		gsi_ctx->msi.irq[msi] = desc->irq;
		GSIDBG("desc->irq =%d\n", desc->irq);
#endif
		/* Request the IRQ */
		if (__gsi_request_msi_irq(msi)) {
			GSIERR("error requesting IRQ for MSI %lu\n",
				msi);
			result = -GSI_STATUS_ERROR;
			goto err_free_msis;
		}
		GSIDBG("Requesting IRQ succesful\n");
	}

	return result;

err_free_msis:
	size = sizeof(unsigned long) * BITS_TO_LONGS(gsi_ctx->msi.num);
#if (KERNEL_VERSION(6, 8, 0) > LINUX_VERSION_CODE)
	platform_msi_domain_free_irqs(gsi_ctx->dev);
#else
	platform_device_msi_free_irqs_all(gsi_ctx->dev);
#endif
	memset(gsi_ctx->msi.allocated, 0, size);

	return result;
}

int gsi_register_device(struct gsi_per_props *props, unsigned long *dev_hdl)
{
	int res;
	int result = GSI_STATUS_SUCCESS;
	struct gsihal_reg_gsi_status gsi_status;
	struct gsihal_reg_gsi_ee_n_cntxt_gsi_irq gen_irq;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (!props || !dev_hdl) {
		GSIERR("bad params props=%pK dev_hdl=%pK\n", props, dev_hdl);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (props->ver <= GSI_VER_ERR || props->ver >= GSI_VER_MAX) {
		GSIERR("bad params gsi_ver=%d\n", props->ver);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (!props->notify_cb) {
		GSIERR("notify callback must be provided\n");
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (props->req_clk_cb && !props->rel_clk_cb) {
		GSIERR("rel callback  must be provided\n");
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (gsi_ctx->per_registered) {
		GSIERR("per already registered\n");
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	spin_lock_init(&gsi_ctx->slock);
	gsi_ctx->per = *props;
	if (props->intr == GSI_INTR_IRQ) {
		if (!props->irq) {
			GSIERR("bad irq specified %u\n", props->irq);
			return -GSI_STATUS_INVALID_PARAMS;
		}
		/*
		 * On a real UE, there are two separate interrupt
		 * vectors that get directed toward the GSI/IPA
		 * drivers.  They are handled by gsi_isr() and
		 * (ipa_isr() or ipa3_isr()) respectively.  In the
		 * emulation environment, this is not the case;
		 * instead, interrupt vectors are routed to the
		 * emualation hardware's interrupt controller, which
		 * in turn, forwards a single interrupt to the GSI/IPA
		 * driver.  When the new interrupt vector is received,
		 * the driver needs to probe the interrupt
		 * controller's registers so see if one, the other, or
		 * both interrupts have occurred.  Given the above, we
		 * now need to handle both situations, namely: the
		 * emulator's and the real UE.
		 */
		if (running_emulation) {
			/*
			 * New scheme involving the emulator's
			 * interrupt controller.
			 */
			res = devm_request_threaded_irq(
				gsi_ctx->dev,
				props->irq,
				/* top half handler to follow */
				emulator_hard_irq_isr,
				/* threaded bottom half handler to follow */
				emulator_soft_irq_isr,
				IRQF_SHARED,
				"emulator_intcntrlr",
				gsi_ctx);
		} else {
			/*
			 * Traditional scheme used on the real UE.
			 */
			res = devm_request_irq(gsi_ctx->dev, props->irq,
				gsi_isr,
				props->req_clk_cb ? IRQF_TRIGGER_RISING :
					IRQF_TRIGGER_HIGH,
				"gsi",
				gsi_ctx);
		}
		if (res) {
			GSIERR(
			 "failed to register isr for %u\n",
			 props->irq);
			return -GSI_STATUS_ERROR;
		}
		GSIDBG(
			"succeeded to register isr for %u\n",
			props->irq);

		res = enable_irq_wake(props->irq);
		if (res)
			GSIERR("failed to enable wake irq %u\n", props->irq);
		else
			GSIERR("GSI irq is wake enabled %u\n", props->irq);

	} else {
		GSIERR("do not support interrupt type %u\n", props->intr);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	/* If MSIs are enabled, make sure they are set up */
	if (gsi_ctx->msi.num) {
		if (__gsi_allocate_msis()) {
			GSIERR("failed to allocate MSIs\n");
			goto err_free_irq;
		}
	}

	/*
	 * If base not previously mapped via gsi_map_base(), map it
	 * now...
	 */
	if (!gsi_ctx->base) {
		res = gsi_map_base(props->phys_addr, props->size, props->ver);
		if (res) {
			result = res;
			goto err_free_msis;
		}
	}

	if (running_emulation) {
		GSIDBG("GSI SW ver register value 0x%x\n",
			gsihal_read_reg_n(GSI_EE_n_GSI_SW_VERSION, 0));
		gsi_ctx->intcntrlr_mem_size =
		    props->emulator_intcntrlr_size;
		gsi_ctx->intcntrlr_base =
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
		    devm_ioremap(
#else
		    devm_ioremap_nocache(
#endif
			gsi_ctx->dev,
			props->emulator_intcntrlr_addr,
			props->emulator_intcntrlr_size);
		if (!gsi_ctx->intcntrlr_base) {
			GSIERR(
			  "failed to remap emulator's interrupt controller HW\n");
			gsi_unmap_base();
			devm_free_irq(gsi_ctx->dev, props->irq, gsi_ctx);
			result = -GSI_STATUS_RES_ALLOC_FAILURE;
			goto err_iounmap;
		}

		GSIDBG(
		    "Emulator's interrupt controller base(%pa) mapped to (%pK) with len (0x%lx)\n",
		    &(props->emulator_intcntrlr_addr),
		    gsi_ctx->intcntrlr_base,
		    props->emulator_intcntrlr_size);

		gsi_ctx->intcntrlr_gsi_isr = gsi_isr;
		gsi_ctx->intcntrlr_client_isr =
		    props->emulator_intcntrlr_client_isr;
	}

	gsi_ctx->per_registered = true;
	mutex_init(&gsi_ctx->mlock);
	atomic_set(&gsi_ctx->num_chan, 0);
	atomic_set(&gsi_ctx->num_evt_ring, 0);
	gsi_ctx->max_ch = gsi_get_max_channels(gsi_ctx->per.ver);
	if (gsi_ctx->max_ch == 0) {
		gsi_unmap_base();
		if (running_emulation)
			devm_iounmap(gsi_ctx->dev, gsi_ctx->intcntrlr_base);
		gsi_ctx->base = gsi_ctx->intcntrlr_base = NULL;
		devm_free_irq(gsi_ctx->dev, props->irq, gsi_ctx);
		GSIERR("failed to get max channels\n");
		result = -GSI_STATUS_ERROR;
		goto err_iounmap;
	}
	gsi_ctx->max_ev = gsi_get_max_event_rings(gsi_ctx->per.ver);
	if (gsi_ctx->max_ev == 0) {
		gsi_unmap_base();
		if (running_emulation)
			devm_iounmap(gsi_ctx->dev, gsi_ctx->intcntrlr_base);
		gsi_ctx->base = gsi_ctx->intcntrlr_base = NULL;
		devm_free_irq(gsi_ctx->dev, props->irq, gsi_ctx);
		GSIERR("failed to get max event rings\n");
		result = -GSI_STATUS_ERROR;
		goto err_iounmap;
	}

	if (gsi_ctx->max_ev > GSI_EVT_RING_MAX) {
		GSIERR("max event rings are beyond absolute maximum\n");
		result = -GSI_STATUS_ERROR;
		goto err_iounmap;
	}

	if (props->mhi_er_id_limits_valid &&
	    props->mhi_er_id_limits[0] > (gsi_ctx->max_ev - 1)) {
		gsi_unmap_base();
		if (running_emulation)
			devm_iounmap(gsi_ctx->dev, gsi_ctx->intcntrlr_base);
		gsi_ctx->base = gsi_ctx->intcntrlr_base = NULL;
		devm_free_irq(gsi_ctx->dev, props->irq, gsi_ctx);
		GSIERR("MHI event ring start id %u is beyond max %u\n",
			props->mhi_er_id_limits[0], gsi_ctx->max_ev);
		result = -GSI_STATUS_ERROR;
		goto err_iounmap;
	}

	gsi_ctx->evt_bmap = ~((((unsigned long)1) << gsi_ctx->max_ev) - 1);

	/* exclude reserved mhi events */
	if (props->mhi_er_id_limits_valid)
		gsi_ctx->evt_bmap |=
			((1 << (props->mhi_er_id_limits[1] + 1)) - 1) ^
			((1 << (props->mhi_er_id_limits[0])) - 1);

	/*
	 * enable all interrupts but GSI_BREAK_POINT.
	 * Inter EE commands / interrupt are no supported.
	 */
	__gsi_config_type_irq(props->ee, ~0, ~0);
	if (gsi_ctx->per.ver >= GSI_VER_3_0) {
		__gsi_config_all_ch_irq(props->ee, ~0, ~0);
		__gsi_config_all_evt_irq(props->ee, ~0, ~0);
		__gsi_config_all_ieob_irq(props->ee, ~0, ~0);
	}
	else {
		__gsi_config_ch_irq(props->ee, ~0, ~0);
		__gsi_config_evt_irq(props->ee, ~0, ~0);
		__gsi_config_ieob_irq(props->ee, ~0, ~0);
	}
	__gsi_config_glob_irq(props->ee, ~0, ~0);

	/*
	 * Disabling global INT1 interrupt by default and enable it
	 * onlt when sending the generic command.
	 */
	__gsi_config_glob_irq(props->ee,
			gsihal_get_glob_irq_en_gp_int1_mask(), 0);

	gen_irq.gsi_mcs_stack_ovrflow = 1;
	gen_irq.gsi_cmd_fifo_ovrflow = 1;
	gen_irq.gsi_bus_error = 1;
	gen_irq.gsi_break_point = 0;
	gsihal_write_reg_n_fields(GSI_EE_n_CNTXT_GSI_IRQ_EN,
		gsi_ctx->per.ee, &gen_irq);

	gsihal_write_reg_n(GSI_EE_n_CNTXT_INTSET, gsi_ctx->per.ee, props->intr);
	/* set GSI_TOP_EE_n_CNTXT_MSI_BASE_LSB/MSB to 0 */
	if ((gsi_ctx->per.ver >= GSI_VER_2_0) &&
		(props->intr != GSI_INTR_MSI)) {
		gsihal_write_reg_n(
			GSI_EE_n_CNTXT_MSI_BASE_LSB, gsi_ctx->per.ee, 0);
		gsihal_write_reg_n(
			GSI_EE_n_CNTXT_MSI_BASE_MSB, gsi_ctx->per.ee, 0);
	}

	gsihal_read_reg_n_fields(GSI_EE_n_GSI_STATUS,
		gsi_ctx->per.ee, &gsi_status);
	if (gsi_status.enabled)
		gsi_ctx->enabled = true;
	else
		GSIERR("Manager EE has not enabled GSI, GSI un-usable\n");

	if (gsi_ctx->per.ver >= GSI_VER_1_2)
		gsihal_write_reg_n(GSI_EE_n_ERROR_LOG, gsi_ctx->per.ee, 0);

	if (running_emulation) {
		/*
		 * Set up the emulator's interrupt controller...
		 */
		res = setup_emulator_cntrlr(
		    gsi_ctx->intcntrlr_base, gsi_ctx->intcntrlr_mem_size);
		if (res != 0) {
			GSIERR("setup_emulator_cntrlr() failed\n");
			result = res;
			goto err_iounmap;
		}
	}

	*dev_hdl = (uintptr_t)gsi_ctx;
	gsi_ctx->gsi_isr_cache_index = 0;

	return result;
err_iounmap:
	gsi_unmap_base();
	if (running_emulation && gsi_ctx->intcntrlr_base != NULL)
		devm_iounmap(gsi_ctx->dev, gsi_ctx->intcntrlr_base);
	gsi_ctx->base = gsi_ctx->intcntrlr_base = NULL;

err_free_msis:
	if (gsi_ctx->msi.num) {
		size_t size =
			sizeof(unsigned long) * BITS_TO_LONGS(gsi_ctx->msi.num);
#if (KERNEL_VERSION(6, 8, 0) > LINUX_VERSION_CODE)
		platform_msi_domain_free_irqs(gsi_ctx->dev);
#else
		platform_device_msi_free_irqs_all(gsi_ctx->dev);
#endif
		memset(gsi_ctx->msi.allocated, 0, size);
	}

err_free_irq:
	devm_free_irq(gsi_ctx->dev, props->irq, gsi_ctx);

	return result;
}
EXPORT_SYMBOL(gsi_register_device);

int gsi_write_device_scratch(unsigned long dev_hdl,
		struct gsi_device_scratch *val)
{
	unsigned int max_usb_pkt_size = 0;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (!gsi_ctx->per_registered) {
		GSIERR("no client registered\n");
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (dev_hdl != (uintptr_t)gsi_ctx) {
		GSIERR("bad params dev_hdl=0x%lx gsi_ctx=0x%pK\n", dev_hdl,
				gsi_ctx);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (val->max_usb_pkt_size_valid &&
			val->max_usb_pkt_size != 1024 &&
			val->max_usb_pkt_size != 512 &&
			val->max_usb_pkt_size != 64) {
		GSIERR("bad USB max pkt size dev_hdl=0x%lx sz=%u\n", dev_hdl,
				val->max_usb_pkt_size);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	mutex_lock(&gsi_ctx->mlock);
	if (val->mhi_base_chan_idx_valid)
		gsi_ctx->scratch.word0.s.mhi_base_chan_idx =
			val->mhi_base_chan_idx;

	if (val->max_usb_pkt_size_valid) {
		max_usb_pkt_size = 2;
		if (val->max_usb_pkt_size > 64)
			max_usb_pkt_size =
				(val->max_usb_pkt_size == 1024) ? 1 : 0;
		gsi_ctx->scratch.word0.s.max_usb_pkt_size = max_usb_pkt_size;
	}

	gsihal_write_reg_n(GSI_EE_n_CNTXT_SCRATCH_0,
		gsi_ctx->per.ee, gsi_ctx->scratch.word0.val);
	mutex_unlock(&gsi_ctx->mlock);

	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_write_device_scratch);

int gsi_deregister_device(unsigned long dev_hdl, bool force)
{
	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (!gsi_ctx->per_registered) {
		GSIERR("no client registered\n");
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (dev_hdl != (uintptr_t)gsi_ctx) {
		GSIERR("bad params dev_hdl=0x%lx gsi_ctx=0x%pK\n", dev_hdl,
				gsi_ctx);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (!force && atomic_read(&gsi_ctx->num_chan)) {
		GSIERR("cannot deregister %u channels are still connected\n",
				atomic_read(&gsi_ctx->num_chan));
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	if (!force && atomic_read(&gsi_ctx->num_evt_ring)) {
		GSIERR("cannot deregister %u events are still connected\n",
				atomic_read(&gsi_ctx->num_evt_ring));
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	/* disable all interrupts */
	__gsi_config_type_irq(gsi_ctx->per.ee, ~0, 0);
	if (gsi_ctx->per.ver >= GSI_VER_3_0) {
		__gsi_config_all_ch_irq(gsi_ctx->per.ee, ~0, 0);
		__gsi_config_all_evt_irq(gsi_ctx->per.ee, ~0, 0);
		__gsi_config_all_ieob_irq(gsi_ctx->per.ee, ~0, 0);
	}
	else {
		__gsi_config_ch_irq(gsi_ctx->per.ee, ~0, 0);
		__gsi_config_evt_irq(gsi_ctx->per.ee, ~0, 0);
		__gsi_config_ieob_irq(gsi_ctx->per.ee, ~0, 0);
	}
	__gsi_config_glob_irq(gsi_ctx->per.ee, ~0, 0);
	__gsi_config_gen_irq(gsi_ctx->per.ee, ~0, 0);

	if (gsi_ctx->msi.num)
#if (KERNEL_VERSION(6, 8, 0) > LINUX_VERSION_CODE)
		platform_msi_domain_free_irqs(gsi_ctx->dev);
#else
		platform_device_msi_free_irqs_all(gsi_ctx->dev);
#endif

	devm_free_irq(gsi_ctx->dev, gsi_ctx->per.irq, gsi_ctx);
	gsihal_destroy();
	gsi_unmap_base();
	gsi_ctx->per_registered = false;
	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_deregister_device);

static void gsi_program_evt_ring_ctx(struct gsi_evt_ring_props *props,
		uint8_t evt_id, unsigned int ee)
{
	struct gsihal_reg_ev_ch_k_cntxt_0 ev_ch_k_cntxt_0;
	struct gsihal_reg_ev_ch_k_cntxt_1 ev_ch_k_cntxt_1;
	struct gsihal_reg_ev_ch_k_cntxt_2 ev_ch_k_cntxt_2;
	struct gsihal_reg_ev_ch_k_cntxt_3 ev_ch_k_cntxt_3;
	struct gsihal_reg_ev_ch_k_cntxt_8 ev_ch_k_cntxt_8;
	struct gsihal_reg_ev_ch_k_cntxt_9 ev_ch_k_cntxt_9;
	union gsihal_reg_ev_ch_k_cntxt_10 ev_ch_k_cntxt_10;
	union gsihal_reg_ev_ch_k_cntxt_11 ev_ch_k_cntxt_11;
	struct gsihal_reg_ev_ch_k_cntxt_12 ev_ch_k_cntxt_12;
	struct gsihal_reg_ev_ch_k_cntxt_13 ev_ch_k_cntxt_13;

	GSIDBG("intf=%u intr=%u re=%u\n", props->intf, props->intr,
			props->re_size);
	ev_ch_k_cntxt_0.chtype = props->intf;
	ev_ch_k_cntxt_0.intype = props->intr;
	ev_ch_k_cntxt_0.element_size = props->re_size;
	gsihal_write_reg_nk_fields(GSI_EE_n_EV_CH_k_CNTXT_0,
		ee, evt_id, &ev_ch_k_cntxt_0);

	ev_ch_k_cntxt_1.r_length = props->ring_len;
	gsihal_write_reg_nk_fields(GSI_EE_n_EV_CH_k_CNTXT_1,
		ee, evt_id,
		&ev_ch_k_cntxt_1);

	ev_ch_k_cntxt_2.r_base_addr_lsbs = GSI_LSB(props->ring_base_addr);
	gsihal_write_reg_nk_fields(GSI_EE_n_EV_CH_k_CNTXT_2,
		ee, evt_id,
		&ev_ch_k_cntxt_2);

	ev_ch_k_cntxt_3.r_base_addr_msbs = GSI_MSB(props->ring_base_addr);
	gsihal_write_reg_nk_fields(GSI_EE_n_EV_CH_k_CNTXT_3,
		ee, evt_id,
		&ev_ch_k_cntxt_3);

	ev_ch_k_cntxt_8.int_modt = props->int_modt;
	ev_ch_k_cntxt_8.int_modc = props->int_modc;
	gsihal_write_reg_nk_fields(GSI_EE_n_EV_CH_k_CNTXT_8,
		ee, evt_id,
		&ev_ch_k_cntxt_8);

	ev_ch_k_cntxt_9.intvec = props->intvec;
	gsihal_write_reg_nk_fields(GSI_EE_n_EV_CH_k_CNTXT_9,
		ee, evt_id,
		&ev_ch_k_cntxt_9);

	if(props->intf != GSI_EVT_CHTYPE_WDI3_V2_EV) {
		ev_ch_k_cntxt_10.msi_addr_lsb = GSI_LSB(props->msi_addr);
		gsihal_write_reg_nk_fields(GSI_EE_n_EV_CH_k_CNTXT_10,
			ee, evt_id,
			&ev_ch_k_cntxt_10);

		ev_ch_k_cntxt_11.msi_addr_msb = GSI_MSB(props->msi_addr);
		gsihal_write_reg_nk_fields(GSI_EE_n_EV_CH_k_CNTXT_11,
			ee, evt_id,
			&ev_ch_k_cntxt_11);


		ev_ch_k_cntxt_12.rp_update_addr_lsb = GSI_LSB(props->rp_update_addr);
		gsihal_write_reg_nk_fields(GSI_EE_n_EV_CH_k_CNTXT_12,
			ee, evt_id,
			&ev_ch_k_cntxt_12);

		ev_ch_k_cntxt_13.rp_update_addr_msb = GSI_MSB(props->rp_update_addr);
		gsihal_write_reg_nk_fields(GSI_EE_n_EV_CH_k_CNTXT_13,
			ee, evt_id,
			&ev_ch_k_cntxt_13);
	}
	else {
		ev_ch_k_cntxt_10.rp_addr_lsb = GSI_LSB(props->rp_update_addr);
		gsihal_write_reg_nk_fields(GSI_EE_n_EV_CH_k_CNTXT_10,
			ee, evt_id,
			&ev_ch_k_cntxt_10);

		ev_ch_k_cntxt_11.rp_addr_msb = GSI_MSB(props->rp_update_addr);
		gsihal_write_reg_nk_fields(GSI_EE_n_EV_CH_k_CNTXT_11,
			ee, evt_id,
			&ev_ch_k_cntxt_11);
	}


}

static void gsi_init_evt_ring(struct gsi_evt_ring_props *props,
		struct gsi_ring_ctx *ctx)
{
	ctx->base_va = (uintptr_t)props->ring_base_vaddr;
	ctx->base = props->ring_base_addr;
	ctx->wp = ctx->base;
	ctx->rp = ctx->base;
	ctx->wp_local = ctx->base;
	ctx->rp_local = ctx->base;
	ctx->len = props->ring_len;
	ctx->elem_sz = props->re_size;
	ctx->max_num_elem = ctx->len / ctx->elem_sz - 1;
	ctx->end = ctx->base + (ctx->max_num_elem + 1) * ctx->elem_sz;

	if (props->rp_update_vaddr)
		*(uint64_t *)(props->rp_update_vaddr) = ctx->rp_local;
}

static void gsi_prime_evt_ring(struct gsi_evt_ctx *ctx)
{
	unsigned long flags;
	struct gsihal_reg_gsi_ee_n_ev_ch_k_doorbell_1 db;

	spin_lock_irqsave(&ctx->ring.slock, flags);
	memset((void *)ctx->ring.base_va, 0, ctx->ring.len);
	ctx->ring.wp_local = ctx->ring.base +
		ctx->ring.max_num_elem * ctx->ring.elem_sz;

	/* write order MUST be MSB followed by LSB */
	db.write_ptr_msb = GSI_MSB(ctx->ring.wp_local);
	gsihal_write_reg_nk_fields(GSI_EE_n_EV_CH_k_DOORBELL_1,
		gsi_ctx->per.ee, ctx->id, &db);

	gsi_ring_evt_doorbell(ctx);
	spin_unlock_irqrestore(&ctx->ring.slock, flags);
}

static void gsi_prime_evt_ring_wdi(struct gsi_evt_ctx *ctx)
{
	unsigned long flags;

	spin_lock_irqsave(&ctx->ring.slock, flags);
	if (ctx->ring.base_va)
		memset((void *)ctx->ring.base_va, 0, ctx->ring.len);
	ctx->ring.wp_local = ctx->ring.base +
		((ctx->ring.max_num_elem + 2) * ctx->ring.elem_sz);
	gsi_ring_evt_doorbell(ctx);
	spin_unlock_irqrestore(&ctx->ring.slock, flags);
}

static int gsi_validate_evt_ring_props(struct gsi_evt_ring_props *props)
{
	uint64_t ra;

	if ((props->re_size == GSI_EVT_RING_RE_SIZE_4B &&
				props->ring_len % 4) ||
			(props->re_size == GSI_EVT_RING_RE_SIZE_8B &&
				 props->ring_len % 8) ||
			(props->re_size == GSI_EVT_RING_RE_SIZE_16B &&
				 props->ring_len % 16) ||
				(props->re_size == GSI_EVT_RING_RE_SIZE_32B &&
					props->ring_len % 32)) {
		GSIERR("bad params ring_len %u not a multiple of RE size %u\n",
				props->ring_len, props->re_size);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (!gsihal_check_ring_length_valid(props->ring_len, props->re_size))
		return -GSI_STATUS_INVALID_PARAMS;

	ra = props->ring_base_addr;
	do_div(ra, roundup_pow_of_two(props->ring_len));

	if (props->ring_base_addr != ra * roundup_pow_of_two(props->ring_len)) {
		GSIERR("bad params ring base not aligned 0x%llx align 0x%lx\n",
				props->ring_base_addr,
				roundup_pow_of_two(props->ring_len));
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (props->intf == GSI_EVT_CHTYPE_GPI_EV &&
			!props->ring_base_vaddr) {
		GSIERR("protocol %u requires ring base VA\n", props->intf);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (props->intf == GSI_EVT_CHTYPE_MHI_EV &&
			(!props->evchid_valid ||
			props->evchid > gsi_ctx->per.mhi_er_id_limits[1] ||
			props->evchid < gsi_ctx->per.mhi_er_id_limits[0])) {
		GSIERR("MHI requires evchid valid=%d val=%u\n",
				props->evchid_valid, props->evchid);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (props->intf != GSI_EVT_CHTYPE_MHI_EV &&
			props->evchid_valid) {
		GSIERR("protocol %u cannot specify evchid\n", props->intf);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (!props->err_cb) {
		GSIERR("err callback must be provided\n");
		return -GSI_STATUS_INVALID_PARAMS;
	}

	return GSI_STATUS_SUCCESS;
}

/**
 * gsi_cleanup_xfer_user_data: cleanup the user data array using callback passed
 *	by IPA driver. Need to do this in GSI since only GSI knows which TRE
 *	are being used or not. However, IPA is the one that does cleaning,
 *	therefore we pass a callback from IPA and call it using params from GSI
 *
 * @chan_hdl: hdl of the gsi channel user data array to be cleaned
 * @cleanup_cb: callback used to clean the user data array. takes 2 inputs
 *	@chan_user_data: ipa_sys_context of the gsi_channel
 *	@xfer_uder_data: user data array element (rx_pkt wrapper)
 *
 * Returns: 0 on success, negative on failure
 */
static int gsi_cleanup_xfer_user_data(unsigned long chan_hdl,
	void (*cleanup_cb)(void *chan_user_data, void *xfer_user_data))
{
	struct gsi_chan_ctx *ctx;
	uint64_t i;
	uint16_t rp_idx;

	ctx = &gsi_ctx->chan[chan_hdl];
	if (ctx->state != GSI_CHAN_STATE_ALLOCATED) {
		GSIERR("bad state %d\n", ctx->state);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	/* for coalescing, traverse the whole array */
	if (ctx->props.prot == GSI_CHAN_PROT_GCI) {
		size_t user_data_size =
			ctx->ring.max_num_elem + 1 + GSI_VEID_MAX;
		for (i = 0; i < user_data_size; i++) {
			if (ctx->user_data[i].valid)
				cleanup_cb(ctx->props.chan_user_data,
					ctx->user_data[i].p);
		}
	} else {
		/* for non-coalescing, clean between RP and WP */
		while (ctx->ring.rp_local != ctx->ring.wp_local) {
			rp_idx = gsi_find_idx_from_addr(&ctx->ring,
				ctx->ring.rp_local);
			WARN_ON(!ctx->user_data[rp_idx].valid);
			cleanup_cb(ctx->props.chan_user_data,
				ctx->user_data[rp_idx].p);
			gsi_incr_ring_rp(&ctx->ring);
		}
	}
	return 0;
}

/**
 * gsi_read_event_ring_rp_ddr - function returns the RP value of the event
 *      ring read from the ring context register.
 *
 * @props: Props structere of the event channel
 * @id: Event channel index
 * @ee: EE
 *
 * @Return pointer to the read pointer
 */
static inline uint64_t gsi_read_event_ring_rp_ddr(struct gsi_evt_ring_props* props,
	uint8_t id, int ee)
{
	return readl_relaxed(props->rp_update_vaddr);
}

/**
 * gsi_read_event_ring_rp_reg - function returns the RP value of the event ring
 *      read from the DDR.
 *
 * @props: Props structere of the event channel
 * @id: Event channel index
 * @ee: EE
 *
 * @Return pointer to the read pointer
 */
static inline uint64_t gsi_read_event_ring_rp_reg(struct gsi_evt_ring_props* props,
	uint8_t id, int ee)
{
	uint64_t rp;

	rp = gsihal_read_reg_nk(GSI_EE_n_EV_CH_k_CNTXT_4, ee, id);
	rp |= ((uint64_t)gsihal_read_reg_nk(GSI_EE_n_EV_CH_k_CNTXT_5, ee, id)) << 32;

	return rp;
}

static int __gsi_pair_msi(struct gsi_evt_ctx *ctx,
		struct gsi_evt_ring_props *props)
{
	int result = GSI_STATUS_SUCCESS;
	unsigned long msi = 0;

	if (IS_ERR_OR_NULL(ctx) || IS_ERR_OR_NULL(props) || IS_ERR_OR_NULL(gsi_ctx))
		BUG();

	/* Find the first unused MSI */
	msi = find_first_zero_bit(gsi_ctx->msi.used, gsi_ctx->msi.num);
	if (msi >= gsi_ctx->msi.num) {
		GSIERR("No free MSIs for evt %u\n", ctx->id);
		return -GSI_STATUS_ERROR;
	}

	/* Ensure it's been allocated */
	if (!test_bit((int)msi, gsi_ctx->msi.allocated)) {
		GSIDBG("MSI %lu not allocated\n", msi);
		return -GSI_STATUS_ERROR;
	}

	/* Save the event ID for later lookup */
	gsi_ctx->msi.evt[msi] = ctx->id;

	/* Add this event to the IRQ mask */
	set_bit((int)ctx->id, &gsi_ctx->msi.mask);

	props->intvec = gsi_ctx->msi.msg[msi].data;
	props->msi_addr = (uint64_t)gsi_ctx->msi.msg[msi].address_hi << 32 |
			(uint64_t)gsi_ctx->msi.msg[msi].address_lo;

	GSIDBG("props->intvec = %d, props->msi_addr = %llu\n", props->intvec, props->msi_addr);

	if (props->msi_addr == 0)
		BUG();

	/* Mark MSI as used */
	set_bit(msi, gsi_ctx->msi.used);

	return result;
}

int gsi_alloc_evt_ring(struct gsi_evt_ring_props *props, unsigned long dev_hdl,
		unsigned long *evt_ring_hdl)
{
	unsigned long evt_id;
	enum gsi_evt_ch_cmd_opcode op = GSI_EVT_ALLOCATE;
	struct gsihal_reg_ee_n_ev_ch_cmd ev_ch_cmd;
	struct gsi_evt_ctx *ctx;
	int res = 0;
	int ee;
	unsigned long flags;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (!props || !evt_ring_hdl || dev_hdl != (uintptr_t)gsi_ctx) {
		GSIERR("bad params props=%pK dev_hdl=0x%lx evt_ring_hdl=%pK\n",
				props, dev_hdl, evt_ring_hdl);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (gsi_validate_evt_ring_props(props)) {
		GSIERR("invalid params\n");
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (!props->evchid_valid) {
		mutex_lock(&gsi_ctx->mlock);
		evt_id = find_first_zero_bit(&gsi_ctx->evt_bmap,
				sizeof(unsigned long) * BITS_PER_BYTE);
		if (evt_id == sizeof(unsigned long) * BITS_PER_BYTE) {
			GSIERR("failed to alloc event ID\n");
			mutex_unlock(&gsi_ctx->mlock);
			return -GSI_STATUS_RES_ALLOC_FAILURE;
		}
		set_bit(evt_id, &gsi_ctx->evt_bmap);
		mutex_unlock(&gsi_ctx->mlock);
	} else {
		evt_id = props->evchid;
	}
	GSIDBG("Using %lu as virt evt id\n", evt_id);

	if (props->rp_update_addr != 0) {
		GSIDBG("Using DDR to read event RP for virt evt id: %lu\n",
			evt_id);
		props->gsi_read_event_ring_rp =
			gsi_read_event_ring_rp_ddr;
	}
	else {
		GSIDBG("Using CONTEXT reg to read event RP for virt evt id: %lu\n",
			evt_id);
		props->gsi_read_event_ring_rp =
			gsi_read_event_ring_rp_reg;
	}

	ctx = &gsi_ctx->evtr[evt_id];
	memset(ctx, 0, sizeof(*ctx));
	mutex_init(&ctx->mlock);
	init_completion(&ctx->compl);
	atomic_set(&ctx->chan_ref_cnt, 0);
	ctx->num_of_chan_allocated = 0;
	ctx->id = evt_id;

	mutex_lock(&gsi_ctx->mlock);
	/* Pair an MSI with this event if this is an MSI and GPI event channel
	 * NOTE: This modifies props, so must be before props are saved to ctx.
	 */
	if (props->intf == GSI_EVT_CHTYPE_GPI_EV &&
		props->intr == GSI_INTR_MSI) {
		if (__gsi_pair_msi(ctx, props)) {
			GSIERR("evt_id=%lu failed to pair MSI\n", evt_id);
			if (!props->evchid_valid)
				clear_bit(evt_id, &gsi_ctx->evt_bmap);
			mutex_unlock(&gsi_ctx->mlock);
			return -GSI_STATUS_NODEV;
		}
		GSIDBG("evt_id=%lu pair MSI succesful\n", evt_id);
	}
	ctx->props = *props;

	ee = gsi_ctx->per.ee;
	ev_ch_cmd.opcode = op;
	ev_ch_cmd.chid = evt_id;
	gsihal_write_reg_n_fields(GSI_EE_n_EV_CH_CMD, ee, &ev_ch_cmd);
	res = wait_for_completion_timeout(&ctx->compl, GSI_CMD_TIMEOUT);
	if (res == 0) {
		GSIERR("evt_id=%lu timed out\n", evt_id);
		if (!props->evchid_valid)
			clear_bit(evt_id, &gsi_ctx->evt_bmap);
		mutex_unlock(&gsi_ctx->mlock);
		return -GSI_STATUS_TIMED_OUT;
	}

	if (ctx->state != GSI_EVT_RING_STATE_ALLOCATED) {
		GSIERR("evt_id=%lu allocation failed state=%u\n",
				evt_id, ctx->state);
		if (!props->evchid_valid)
			clear_bit(evt_id, &gsi_ctx->evt_bmap);
		mutex_unlock(&gsi_ctx->mlock);
		return -GSI_STATUS_RES_ALLOC_FAILURE;
	}

	gsi_program_evt_ring_ctx(props, evt_id, gsi_ctx->per.ee);

	spin_lock_init(&ctx->ring.slock);
	gsi_init_evt_ring(props, &ctx->ring);

	ctx->id = evt_id;
	*evt_ring_hdl = evt_id;
	atomic_inc(&gsi_ctx->num_evt_ring);
	if (props->intf == GSI_EVT_CHTYPE_GPI_EV)
		gsi_prime_evt_ring(ctx);
	else if (props->intf == GSI_EVT_CHTYPE_WDI2_EV)
		gsi_prime_evt_ring_wdi(ctx);
	mutex_unlock(&gsi_ctx->mlock);

	spin_lock_irqsave(&gsi_ctx->slock, flags);
	if (gsi_ctx->per.ver >= GSI_VER_3_0) {
	gsihal_write_reg_nk(GSI_EE_n_CNTXT_SRC_IEOB_IRQ_CLR_k, ee,
		gsihal_get_ch_reg_idx(evt_id), gsihal_get_ch_reg_mask(evt_id));
	}
	else {
		gsihal_write_reg_n(GSI_EE_n_CNTXT_SRC_IEOB_IRQ_CLR, ee, 1 << evt_id);
	}

	/* enable ieob interrupts for GPI, enable MSI interrupts */
	if (gsi_ctx->per.ver >= GSI_VER_3_0) {
		if ((props->intf != GSI_EVT_CHTYPE_GPI_EV) &&
			(props->intr != GSI_INTR_MSI))
			__gsi_config_ieob_irq_k(gsi_ctx->per.ee, gsihal_get_ch_reg_idx(evt_id),
				gsihal_get_ch_reg_mask(evt_id),
				0);
		else
			__gsi_config_ieob_irq_k(gsi_ctx->per.ee, gsihal_get_ch_reg_idx(evt_id),
				gsihal_get_ch_reg_mask(evt_id),
				~0);
	}
	else {
		if ((props->intf != GSI_EVT_CHTYPE_GPI_EV) &&
			(props->intr != GSI_INTR_MSI))
			__gsi_config_ieob_irq(gsi_ctx->per.ee, 1 << evt_id, 0);
		else
			__gsi_config_ieob_irq(gsi_ctx->per.ee, 1 << ctx->id, ~0);
	}
	spin_unlock_irqrestore(&gsi_ctx->slock, flags);

	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_alloc_evt_ring);

static void __gsi_write_evt_ring_scratch(unsigned long evt_ring_hdl,
		union __packed gsi_evt_scratch val)
{
	gsihal_write_reg_nk(GSI_EE_n_EV_CH_k_SCRATCH_0,
		gsi_ctx->per.ee, evt_ring_hdl, val.data.word1);
	gsihal_write_reg_nk(GSI_EE_n_EV_CH_k_SCRATCH_1,
		gsi_ctx->per.ee, evt_ring_hdl, val.data.word2);
}

int gsi_write_evt_ring_scratch(unsigned long evt_ring_hdl,
		union __packed gsi_evt_scratch val)
{
	struct gsi_evt_ctx *ctx;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (evt_ring_hdl >= gsi_ctx->max_ev) {
		GSIERR("bad params evt_ring_hdl=%lu\n", evt_ring_hdl);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	ctx = &gsi_ctx->evtr[evt_ring_hdl];

	if (ctx->state != GSI_EVT_RING_STATE_ALLOCATED) {
		GSIERR("bad state %d\n",
				gsi_ctx->evtr[evt_ring_hdl].state);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	mutex_lock(&ctx->mlock);
	ctx->scratch = val;
	__gsi_write_evt_ring_scratch(evt_ring_hdl, val);
	mutex_unlock(&ctx->mlock);

	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_write_evt_ring_scratch);

int gsi_dealloc_evt_ring(unsigned long evt_ring_hdl)
{
	struct gsihal_reg_ee_n_ev_ch_cmd ev_ch_cmd;
	enum gsi_evt_ch_cmd_opcode op = GSI_EVT_DE_ALLOC;
	struct gsi_evt_ctx *ctx;
	int res = 0;
	u32 msi;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (evt_ring_hdl >= gsi_ctx->max_ev ||
			evt_ring_hdl >= GSI_EVT_RING_MAX) {
		GSIERR("bad params evt_ring_hdl=%lu\n", evt_ring_hdl);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	ctx = &gsi_ctx->evtr[evt_ring_hdl];

	if (atomic_read(&ctx->chan_ref_cnt)) {
		GSIERR("%d channels still using this event ring\n",
			atomic_read(&ctx->chan_ref_cnt));
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	if (ctx->state != GSI_EVT_RING_STATE_ALLOCATED) {
		GSIERR("bad state %d\n", ctx->state);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	/* Unpair the MSI */
	if (ctx->props.intf == GSI_EVT_CHTYPE_GPI_EV &&
		ctx->props.intr == GSI_INTR_MSI) {
		GSIERR("Interrupt dereg for msi_irq = %d\n", ctx->props.msi_irq);

		for (msi = 0; msi < gsi_ctx->msi.num; msi++) {
			if (gsi_ctx->msi.msg[msi].data == ctx->props.intvec) {
				mutex_lock(&gsi_ctx->mlock);
				clear_bit(msi, gsi_ctx->msi.used);
				gsi_ctx->msi.evt[msi] = 0;
				clear_bit(evt_ring_hdl, &gsi_ctx->msi.mask);
				mutex_unlock(&gsi_ctx->mlock);
			}
		}
	}

	mutex_lock(&gsi_ctx->mlock);
	reinit_completion(&ctx->compl);
	ev_ch_cmd.chid = evt_ring_hdl;
	ev_ch_cmd.opcode = op;
	gsihal_write_reg_n_fields(GSI_EE_n_EV_CH_CMD,
		gsi_ctx->per.ee, &ev_ch_cmd);
	res = wait_for_completion_timeout(&ctx->compl, GSI_CMD_TIMEOUT);
	if (res == 0) {
		GSIERR("evt_id=%lu timed out\n", evt_ring_hdl);
		mutex_unlock(&gsi_ctx->mlock);
		return -GSI_STATUS_TIMED_OUT;
	}

	if (ctx->state != GSI_EVT_RING_STATE_NOT_ALLOCATED) {
		GSIERR("evt_id=%lu unexpected state=%u\n", evt_ring_hdl,
				ctx->state);
		/*
		 * IPA Hardware returned GSI RING not allocated, which is
		 * unexpected hardware state.
		 */
		GSI_ASSERT();
	}
	mutex_unlock(&gsi_ctx->mlock);

	if (!ctx->props.evchid_valid) {
		mutex_lock(&gsi_ctx->mlock);
		clear_bit(evt_ring_hdl, &gsi_ctx->evt_bmap);
		mutex_unlock(&gsi_ctx->mlock);
	}
	atomic_dec(&gsi_ctx->num_evt_ring);

	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_dealloc_evt_ring);

int gsi_query_evt_ring_db_addr(unsigned long evt_ring_hdl,
		uint32_t *db_addr_wp_lsb, uint32_t *db_addr_wp_msb)
{
	struct gsi_evt_ctx *ctx;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (!db_addr_wp_msb || !db_addr_wp_lsb) {
		GSIERR("bad params msb=%pK lsb=%pK\n", db_addr_wp_msb,
				db_addr_wp_lsb);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (evt_ring_hdl >= gsi_ctx->max_ev) {
		GSIERR("bad params evt_ring_hdl=%lu\n", evt_ring_hdl);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	ctx = &gsi_ctx->evtr[evt_ring_hdl];

	if (ctx->state != GSI_EVT_RING_STATE_ALLOCATED) {
		GSIERR("bad state %d\n",
				gsi_ctx->evtr[evt_ring_hdl].state);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	*db_addr_wp_lsb = gsi_ctx->per.phys_addr + gsihal_get_reg_nk_ofst(
		GSI_EE_n_EV_CH_k_DOORBELL_0, gsi_ctx->per.ee, evt_ring_hdl);

	*db_addr_wp_msb = gsi_ctx->per.phys_addr + gsihal_get_reg_nk_ofst(
		GSI_EE_n_EV_CH_k_DOORBELL_1, gsi_ctx->per.ee, evt_ring_hdl);

	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_query_evt_ring_db_addr);

int gsi_ring_evt_ring_db(unsigned long evt_ring_hdl, uint64_t value)
{
	struct gsi_evt_ctx *ctx;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (evt_ring_hdl >= gsi_ctx->max_ev) {
		GSIERR("bad params evt_ring_hdl=%lu\n", evt_ring_hdl);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	ctx = &gsi_ctx->evtr[evt_ring_hdl];

	if (ctx->state != GSI_EVT_RING_STATE_ALLOCATED) {
		GSIERR("bad state %d\n",
				gsi_ctx->evtr[evt_ring_hdl].state);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	ctx->ring.wp_local = value;
	gsi_ring_evt_doorbell(ctx);

	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_ring_evt_ring_db);

int gsi_ring_ch_ring_db(unsigned long chan_hdl, uint64_t value)
{
	struct gsi_chan_ctx *ctx;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (chan_hdl >= gsi_ctx->max_ch) {
		GSIERR("bad chan_hdl=%lu\n", chan_hdl);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	ctx = &gsi_ctx->chan[chan_hdl];

	if (ctx->state != GSI_CHAN_STATE_STARTED) {
		GSIERR("bad state %d\n", ctx->state);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	ctx->ring.wp_local = value;

	/* write MSB first */
	gsihal_write_reg_nk(GSI_EE_n_GSI_CH_k_DOORBELL_1,
		gsi_ctx->per.ee, ctx->props.ch_id, GSI_MSB(ctx->ring.wp_local));

	gsi_ring_chan_doorbell(ctx);

	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_ring_ch_ring_db);

int gsi_reset_evt_ring(unsigned long evt_ring_hdl)
{
	struct gsihal_reg_ee_n_ev_ch_cmd ev_ch_cmd;
	enum gsi_evt_ch_cmd_opcode op = GSI_EVT_RESET;
	struct gsi_evt_ctx *ctx;
	int res;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (evt_ring_hdl >= gsi_ctx->max_ev) {
		GSIERR("bad params evt_ring_hdl=%lu\n", evt_ring_hdl);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	ctx = &gsi_ctx->evtr[evt_ring_hdl];

	if (ctx->state != GSI_EVT_RING_STATE_ALLOCATED) {
		GSIERR("bad state %d\n", ctx->state);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	mutex_lock(&gsi_ctx->mlock);
	reinit_completion(&ctx->compl);
	ev_ch_cmd.chid = evt_ring_hdl;
	ev_ch_cmd.opcode = op;
	gsihal_write_reg_n_fields(GSI_EE_n_EV_CH_CMD,
		gsi_ctx->per.ee, &ev_ch_cmd);
	res = wait_for_completion_timeout(&ctx->compl, GSI_CMD_TIMEOUT);
	if (res == 0) {
		GSIERR("evt_id=%lu timed out\n", evt_ring_hdl);
		mutex_unlock(&gsi_ctx->mlock);
		return -GSI_STATUS_TIMED_OUT;
	}

	if (ctx->state != GSI_EVT_RING_STATE_ALLOCATED) {
		GSIERR("evt_id=%lu unexpected state=%u\n", evt_ring_hdl,
				ctx->state);
		/*
		 * IPA Hardware returned GSI RING not allocated, which is
		 * unexpected. Indicates hardware instability.
		 */
		GSI_ASSERT();
	}

	gsi_program_evt_ring_ctx(&ctx->props, evt_ring_hdl, gsi_ctx->per.ee);
	gsi_init_evt_ring(&ctx->props, &ctx->ring);

	/* restore scratch */
	__gsi_write_evt_ring_scratch(evt_ring_hdl, ctx->scratch);

	if (ctx->props.intf == GSI_EVT_CHTYPE_GPI_EV)
		gsi_prime_evt_ring(ctx);
	if (ctx->props.intf == GSI_EVT_CHTYPE_WDI2_EV)
		gsi_prime_evt_ring_wdi(ctx);
	mutex_unlock(&gsi_ctx->mlock);

	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_reset_evt_ring);

int gsi_get_evt_ring_cfg(unsigned long evt_ring_hdl,
		struct gsi_evt_ring_props *props, union gsi_evt_scratch *scr)
{
	struct gsi_evt_ctx *ctx;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (!props || !scr) {
		GSIERR("bad params props=%pK scr=%pK\n", props, scr);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (evt_ring_hdl >= gsi_ctx->max_ev) {
		GSIERR("bad params evt_ring_hdl=%lu\n", evt_ring_hdl);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	ctx = &gsi_ctx->evtr[evt_ring_hdl];

	if (ctx->state == GSI_EVT_RING_STATE_NOT_ALLOCATED) {
		GSIERR("bad state %d\n", ctx->state);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	mutex_lock(&ctx->mlock);
	*props = ctx->props;
	*scr = ctx->scratch;
	mutex_unlock(&ctx->mlock);

	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_get_evt_ring_cfg);

int gsi_set_evt_ring_cfg(unsigned long evt_ring_hdl,
		struct gsi_evt_ring_props *props, union gsi_evt_scratch *scr)
{
	struct gsi_evt_ctx *ctx;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (!props || gsi_validate_evt_ring_props(props)) {
		GSIERR("bad params props=%pK\n", props);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (evt_ring_hdl >= gsi_ctx->max_ev) {
		GSIERR("bad params evt_ring_hdl=%lu\n", evt_ring_hdl);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	ctx = &gsi_ctx->evtr[evt_ring_hdl];

	if (ctx->state != GSI_EVT_RING_STATE_ALLOCATED) {
		GSIERR("bad state %d\n", ctx->state);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	if (ctx->props.exclusive != props->exclusive) {
		GSIERR("changing immutable fields not supported\n");
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	mutex_lock(&ctx->mlock);
	ctx->props = *props;
	if (scr)
		ctx->scratch = *scr;
	mutex_unlock(&ctx->mlock);

	return gsi_reset_evt_ring(evt_ring_hdl);
}
EXPORT_SYMBOL(gsi_set_evt_ring_cfg);

static void gsi_program_chan_ctx_qos(struct gsi_chan_props *props,
	unsigned int ee)
{
	struct gsihal_reg_gsi_ee_n_gsi_ch_k_qos ch_k_qos;

	ch_k_qos.wrr_weight = props->low_weight;
	ch_k_qos.max_prefetch = props->max_prefetch;
	ch_k_qos.use_db_eng = props->use_db_eng;

	if (gsi_ctx->per.ver >= GSI_VER_2_0) {
		if (gsi_ctx->per.ver < GSI_VER_2_5) {
			ch_k_qos.use_escape_buf_only = props->prefetch_mode;
		} else {
			ch_k_qos.prefetch_mode = props->prefetch_mode;
			ch_k_qos.empty_lvl_thrshold =
				props->empty_lvl_threshold;
			if (gsi_ctx->per.ver >= GSI_VER_2_9)
				ch_k_qos.db_in_bytes = props->db_in_bytes;
			if (gsi_ctx->per.ver >= GSI_VER_3_0)
				ch_k_qos.low_latency_en = props->low_latency_en;
		}
	}
	gsihal_write_reg_nk_fields(GSI_EE_n_GSI_CH_k_QOS,
		ee, props->ch_id, &ch_k_qos);
}

static void gsi_program_chan_ctx(struct gsi_chan_props *props, unsigned int ee,
		uint8_t erindex)
{
	struct gsihal_reg_ch_k_cntxt_0 ch_k_cntxt_0;
	struct gsihal_reg_ch_k_cntxt_1 ch_k_cntxt_1;

	switch (props->prot) {
	case GSI_CHAN_PROT_MHI:
	case GSI_CHAN_PROT_XHCI:
	case GSI_CHAN_PROT_GPI:
	case GSI_CHAN_PROT_XDCI:
	case GSI_CHAN_PROT_WDI2:
	case GSI_CHAN_PROT_WDI3:
	case GSI_CHAN_PROT_GCI:
	case GSI_CHAN_PROT_MHIP:
	case GSI_CHAN_PROT_WDI3_V2:
		ch_k_cntxt_0.chtype_protocol_msb = 0;
		break;
	case GSI_CHAN_PROT_AQC:
	case GSI_CHAN_PROT_11AD:
	case GSI_CHAN_PROT_RTK:
	case GSI_CHAN_PROT_QDSS:
	case GSI_CHAN_PROT_NTN:
		ch_k_cntxt_0.chtype_protocol_msb = 1;
		break;
	default:
		GSIERR("Unsupported protocol %d\n", props->prot);
		WARN_ON(1);
		return;
	}

	ch_k_cntxt_0.chtype_protocol = props->prot;
	ch_k_cntxt_0.chtype_dir = props->dir;
	if (gsi_ctx->per.ver >= GSI_VER_3_0) {
		ch_k_cntxt_1.erindex = erindex;
	} else {
		ch_k_cntxt_0.erindex = erindex;
	}
	ch_k_cntxt_0.element_size = props->re_size;
	gsihal_write_reg_nk_fields(GSI_EE_n_GSI_CH_k_CNTXT_0,
		ee, props->ch_id, &ch_k_cntxt_0);

	ch_k_cntxt_1.r_length = props->ring_len;
	gsihal_write_reg_nk_fields(GSI_EE_n_GSI_CH_k_CNTXT_1,
		ee, props->ch_id, &ch_k_cntxt_1);

	gsihal_write_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_2,
		ee, props->ch_id, GSI_LSB(props->ring_base_addr));
	gsihal_write_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_3,
		ee, props->ch_id, GSI_MSB(props->ring_base_addr));

	gsi_program_chan_ctx_qos(props, ee);
}

static void gsi_init_chan_ring(struct gsi_chan_props *props,
		struct gsi_ring_ctx *ctx)
{
	ctx->base_va = (uintptr_t)props->ring_base_vaddr;
	ctx->base = props->ring_base_addr;
	ctx->wp = ctx->base;
	ctx->rp = ctx->base;
	ctx->wp_local = ctx->base;
	ctx->rp_local = ctx->base;
	ctx->len = props->ring_len;
	ctx->elem_sz = props->re_size;
	ctx->max_num_elem = ctx->len / ctx->elem_sz - 1;
	ctx->end = ctx->base + (ctx->max_num_elem + 1) *
		ctx->elem_sz;
}

static int gsi_validate_channel_props(struct gsi_chan_props *props)
{
	uint64_t ra;
	uint64_t last;

	if (props->ch_id >= gsi_ctx->max_ch) {
		GSIERR("ch_id %u invalid\n", props->ch_id);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if ((props->re_size == GSI_CHAN_RE_SIZE_4B &&
				props->ring_len % 4) ||
			(props->re_size == GSI_CHAN_RE_SIZE_8B &&
				 props->ring_len % 8) ||
			(props->re_size == GSI_CHAN_RE_SIZE_16B &&
				 props->ring_len % 16) ||
			(props->re_size == GSI_CHAN_RE_SIZE_32B &&
				 props->ring_len % 32) ||
			(props->re_size == GSI_CHAN_RE_SIZE_64B &&
					props->ring_len % 64)) {
		GSIERR("bad params ring_len %u not a multiple of re size %u\n",
				props->ring_len, props->re_size);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (!gsihal_check_ring_length_valid(props->ring_len, props->re_size))
		return -GSI_STATUS_INVALID_PARAMS;

	ra = props->ring_base_addr;
	do_div(ra, roundup_pow_of_two(props->ring_len));

	if (props->ring_base_addr != ra * roundup_pow_of_two(props->ring_len)) {
		GSIERR("bad params ring base not aligned 0x%llx align 0x%lx\n",
				props->ring_base_addr,
				roundup_pow_of_two(props->ring_len));
		return -GSI_STATUS_INVALID_PARAMS;
	}

	last = props->ring_base_addr + props->ring_len - props->re_size;

	/* MSB should stay same within the ring */
	if ((props->ring_base_addr & 0xFFFFFFFF00000000ULL) !=
	    (last & 0xFFFFFFFF00000000ULL)) {
		GSIERR("MSB is not fixed on ring base 0x%llx size 0x%x\n",
			props->ring_base_addr,
			props->ring_len);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (props->prot == GSI_CHAN_PROT_GPI &&
			!props->ring_base_vaddr) {
		GSIERR("protocol %u requires ring base VA\n", props->prot);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (props->low_weight > GSI_MAX_CH_LOW_WEIGHT) {
		GSIERR("invalid channel low weight %u\n", props->low_weight);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (props->prot == GSI_CHAN_PROT_GPI && !props->xfer_cb) {
		GSIERR("xfer callback must be provided\n");
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (!props->err_cb) {
		GSIERR("err callback must be provided\n");
		return -GSI_STATUS_INVALID_PARAMS;
	}

	return GSI_STATUS_SUCCESS;
}

int gsi_alloc_channel(struct gsi_chan_props *props, unsigned long dev_hdl,
		unsigned long *chan_hdl)
{
	struct gsi_chan_ctx *ctx;
	int res;
	int ee;
	enum gsi_ch_cmd_opcode op = GSI_CH_ALLOCATE;
	uint8_t erindex;
	struct gsi_user_data *user_data;
	size_t user_data_size;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (!props || !chan_hdl || dev_hdl != (uintptr_t)gsi_ctx) {
		GSIERR("bad params props=%pK dev_hdl=0x%lx chan_hdl=%pK\n",
				props, dev_hdl, chan_hdl);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (gsi_validate_channel_props(props)) {
		GSIERR("bad params\n");
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (props->evt_ring_hdl != ~0) {
		if (props->evt_ring_hdl >= gsi_ctx->max_ev) {
			GSIERR("invalid evt ring=%lu\n", props->evt_ring_hdl);
			return -GSI_STATUS_INVALID_PARAMS;
		}

		if (atomic_read(
			&gsi_ctx->evtr[props->evt_ring_hdl].chan_ref_cnt) &&
			gsi_ctx->evtr[props->evt_ring_hdl].props.exclusive &&
			gsi_ctx->evtr[props->evt_ring_hdl].chan[0]->props.prot !=
			GSI_CHAN_PROT_GCI) {
			GSIERR("evt ring=%lu exclusively used by ch_hdl=%pK\n",
				props->evt_ring_hdl, chan_hdl);
			return -GSI_STATUS_UNSUPPORTED_OP;
		}
	}

	ctx = &gsi_ctx->chan[props->ch_id];
	if (ctx->allocated) {
		GSIERR("chan %d already allocated\n", props->ch_id);
		return -GSI_STATUS_NODEV;
	}
	memset(ctx, 0, sizeof(*ctx));

	/* For IPA offloaded WDI channels not required user_data pointer */
	if (props->prot != GSI_CHAN_PROT_WDI2 &&
		props->prot != GSI_CHAN_PROT_WDI3 &&
		props->prot != GSI_CHAN_PROT_WDI3_V2)
		user_data_size = props->ring_len / props->re_size;
	else
		user_data_size = props->re_size;
	/*
	 * GCI channels might have OOO event completions up to GSI_VEID_MAX.
	 * user_data needs to be large enough to accommodate those.
	 * TODO: increase user data size if GSI_VEID_MAX is not enough
	 */
	if (props->prot == GSI_CHAN_PROT_GCI)
		user_data_size += GSI_VEID_MAX;

	user_data = devm_kzalloc(gsi_ctx->dev,
		user_data_size * sizeof(*user_data),
		GFP_KERNEL);
	if (user_data == NULL) {
		GSIERR("context not allocated\n");
		return -GSI_STATUS_RES_ALLOC_FAILURE;
	}

	mutex_init(&ctx->mlock);
	init_completion(&ctx->compl);
	atomic_set(&ctx->poll_mode, GSI_CHAN_MODE_CALLBACK);
	ctx->props = *props;

	if (gsi_ctx->per.ver != GSI_VER_2_2) {
		struct gsihal_reg_ee_n_gsi_ch_cmd ch_cmd;

		mutex_lock(&gsi_ctx->mlock);
		ee = gsi_ctx->per.ee;
		gsi_ctx->ch_dbg[props->ch_id].ch_allocate++;
		ch_cmd.chid = props->ch_id;
		ch_cmd.opcode = op;
		gsihal_write_reg_n_fields(GSI_EE_n_GSI_CH_CMD, ee, &ch_cmd);
		res = wait_for_completion_timeout(&ctx->compl, GSI_CMD_TIMEOUT);
		if (res == 0) {
			GSIERR("chan_hdl=%u timed out\n", props->ch_id);
			mutex_unlock(&gsi_ctx->mlock);
			devm_kfree(gsi_ctx->dev, user_data);
			return -GSI_STATUS_TIMED_OUT;
		}
		if (ctx->state != GSI_CHAN_STATE_ALLOCATED) {
			GSIERR("chan_hdl=%u allocation failed state=%d\n",
					props->ch_id, ctx->state);
			mutex_unlock(&gsi_ctx->mlock);
			devm_kfree(gsi_ctx->dev, user_data);
			return -GSI_STATUS_RES_ALLOC_FAILURE;
		}
		mutex_unlock(&gsi_ctx->mlock);
	} else {
		mutex_lock(&gsi_ctx->mlock);
		ctx->state = GSI_CHAN_STATE_ALLOCATED;
		mutex_unlock(&gsi_ctx->mlock);
	}
	erindex = props->evt_ring_hdl != ~0 ? props->evt_ring_hdl :
		GSI_NO_EVT_ERINDEX;
	if (erindex != GSI_NO_EVT_ERINDEX && erindex >= GSI_EVT_RING_MAX) {
		GSIERR("invalid erindex %u\n", erindex);
		devm_kfree(gsi_ctx->dev, user_data);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (erindex < GSI_EVT_RING_MAX) {
		ctx->evtr = &gsi_ctx->evtr[erindex];
		if(ctx->evtr->num_of_chan_allocated
		   >= MAX_CHANNELS_SHARING_EVENT_RING) {
			GSIERR(
				"too many channels sharing the same event ring %u\n",
				erindex);
			GSI_ASSERT();
		}
		if (props->prot != GSI_CHAN_PROT_GCI) {
			atomic_inc(&ctx->evtr->chan_ref_cnt);
			if (ctx->evtr->props.exclusive) {
				if (atomic_read(&ctx->evtr->chan_ref_cnt) == 1)
					ctx->evtr->chan
					[ctx->evtr->num_of_chan_allocated++] = ctx;
			}
			else {
				ctx->evtr->chan[ctx->evtr->num_of_chan_allocated++]
					= ctx;
			}
		}
	}

	gsi_program_chan_ctx(props, gsi_ctx->per.ee, erindex);

	spin_lock_init(&ctx->ring.slock);
	gsi_init_chan_ring(props, &ctx->ring);
	if (!props->max_re_expected)
		ctx->props.max_re_expected = ctx->ring.max_num_elem;
	ctx->user_data = user_data;
	*chan_hdl = props->ch_id;
	ctx->allocated = true;
	ctx->stats.dp.last_timestamp = jiffies_to_msecs(jiffies);
	atomic_inc(&gsi_ctx->num_chan);

	if (props->prot == GSI_CHAN_PROT_GCI) {
		gsi_ctx->coal_info.ch_id = props->ch_id;
		gsi_ctx->coal_info.evchid = props->evt_ring_hdl;
	}

	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_alloc_channel);

static int gsi_alloc_ap_channel(unsigned int chan_hdl)
{
	struct gsi_chan_ctx *ctx;
	struct gsihal_reg_ee_n_gsi_ch_cmd ch_cmd;
	int res;
	int ee;
	enum gsi_ch_cmd_opcode op = GSI_CH_ALLOCATE;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	ctx = &gsi_ctx->chan[chan_hdl];
	if (ctx->allocated) {
		GSIERR("chan %d already allocated\n", chan_hdl);
		return -GSI_STATUS_NODEV;
	}

	memset(ctx, 0, sizeof(*ctx));

	mutex_init(&ctx->mlock);
	init_completion(&ctx->compl);
	atomic_set(&ctx->poll_mode, GSI_CHAN_MODE_CALLBACK);

	mutex_lock(&gsi_ctx->mlock);
	ee = gsi_ctx->per.ee;
	gsi_ctx->ch_dbg[chan_hdl].ch_allocate++;
	ch_cmd.chid = chan_hdl;
	ch_cmd.opcode = op;
	gsihal_write_reg_n_fields(GSI_EE_n_GSI_CH_CMD, ee, &ch_cmd);
	res = wait_for_completion_timeout(&ctx->compl, GSI_CMD_TIMEOUT);
	if (res == 0) {
		GSIERR("chan_hdl=%u timed out\n", chan_hdl);
		mutex_unlock(&gsi_ctx->mlock);
		return -GSI_STATUS_TIMED_OUT;
	}
	if (ctx->state != GSI_CHAN_STATE_ALLOCATED) {
		GSIERR("chan_hdl=%u allocation failed state=%d\n",
				chan_hdl, ctx->state);
		mutex_unlock(&gsi_ctx->mlock);
		return -GSI_STATUS_RES_ALLOC_FAILURE;
	}
	mutex_unlock(&gsi_ctx->mlock);

	return GSI_STATUS_SUCCESS;
}

static void __gsi_write_channel_scratch(unsigned long chan_hdl,
		union __packed gsi_channel_scratch val)
{
	gsihal_write_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_0,
		gsi_ctx->per.ee, chan_hdl, val.data.word1);
	gsihal_write_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_1,
		gsi_ctx->per.ee, chan_hdl, val.data.word2);
	gsihal_write_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_2,
		gsi_ctx->per.ee, chan_hdl, val.data.word3);
	gsihal_write_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_3,
		gsi_ctx->per.ee, chan_hdl, val.data.word4);
}

static void __gsi_write_wdi3_channel_scratch2_reg(unsigned long chan_hdl,
		union __packed gsi_wdi3_channel_scratch2_reg val)
{
	gsihal_write_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_2,
		gsi_ctx->per.ee, chan_hdl, val.data.word1);
}


int gsi_write_channel_scratch3_reg(unsigned long chan_hdl,
		union __packed gsi_wdi_channel_scratch3_reg val)
{
	struct gsi_chan_ctx *ctx;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (chan_hdl >= gsi_ctx->max_ch) {
		GSIERR("bad params chan_hdl=%lu\n", chan_hdl);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	ctx = &gsi_ctx->chan[chan_hdl];

	mutex_lock(&ctx->mlock);

	ctx->scratch.wdi.endp_metadatareg_offset =
				val.wdi.endp_metadatareg_offset;
	ctx->scratch.wdi.qmap_id = val.wdi.qmap_id;
	gsihal_write_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_3,
		gsi_ctx->per.ee, chan_hdl, val.data.word1);
	mutex_unlock(&ctx->mlock);
	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_write_channel_scratch3_reg);

int gsi_write_channel_scratch2_reg(unsigned long chan_hdl,
		union __packed gsi_wdi2_channel_scratch2_reg val)
{
	struct gsi_chan_ctx *ctx;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (chan_hdl >= gsi_ctx->max_ch) {
		GSIERR("bad params chan_hdl=%lu\n", chan_hdl);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	ctx = &gsi_ctx->chan[chan_hdl];

	mutex_lock(&ctx->mlock);

	ctx->scratch.wdi2_new.endp_metadatareg_offset =
				val.wdi.endp_metadatareg_offset;
	ctx->scratch.wdi2_new.qmap_id = val.wdi.qmap_id;
	val.wdi.update_ri_moderation_threshold =
		ctx->scratch.wdi2_new.update_ri_moderation_threshold;
	gsihal_write_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_2,
		gsi_ctx->per.ee, chan_hdl, val.data.word1);
	mutex_unlock(&ctx->mlock);
	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_write_channel_scratch2_reg);

static void __gsi_read_channel_scratch(unsigned long chan_hdl,
		union __packed gsi_channel_scratch * val)
{
	val->data.word1 = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_0,
		gsi_ctx->per.ee, chan_hdl);
	val->data.word2 = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_1,
		gsi_ctx->per.ee, chan_hdl);
	val->data.word3 = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_2,
		gsi_ctx->per.ee, chan_hdl);
	val->data.word4 = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_3,
		gsi_ctx->per.ee, chan_hdl);
}

static void __gsi_read_wdi3_channel_scratch2_reg(unsigned long chan_hdl,
		union __packed gsi_wdi3_channel_scratch2_reg * val)
{
	val->data.word1 = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_2,
		gsi_ctx->per.ee, chan_hdl);

}

int gsi_write_channel_scratch(unsigned long chan_hdl,
		union __packed gsi_channel_scratch val)
{
	struct gsi_chan_ctx *ctx;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (chan_hdl >= gsi_ctx->max_ch) {
		GSIERR("bad params chan_hdl=%lu\n", chan_hdl);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (gsi_ctx->chan[chan_hdl].state != GSI_CHAN_STATE_ALLOCATED &&
		gsi_ctx->chan[chan_hdl].state != GSI_CHAN_STATE_STOPPED) {
		GSIERR("bad state %d\n",
				gsi_ctx->chan[chan_hdl].state);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	ctx = &gsi_ctx->chan[chan_hdl];

	mutex_lock(&ctx->mlock);
	ctx->scratch = val;
	__gsi_write_channel_scratch(chan_hdl, val);
	mutex_unlock(&ctx->mlock);

	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_write_channel_scratch);

int gsi_write_wdi3_channel_scratch2_reg(unsigned long chan_hdl,
		union __packed gsi_wdi3_channel_scratch2_reg val)
{
	struct gsi_chan_ctx *ctx;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (chan_hdl >= gsi_ctx->max_ch) {
		GSIERR("bad params chan_hdl=%lu\n", chan_hdl);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (gsi_ctx->chan[chan_hdl].state != GSI_CHAN_STATE_ALLOCATED &&
		gsi_ctx->chan[chan_hdl].state != GSI_CHAN_STATE_STARTED &&
		gsi_ctx->chan[chan_hdl].state != GSI_CHAN_STATE_STOPPED) {
		GSIERR("bad state %d\n",
				gsi_ctx->chan[chan_hdl].state);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	ctx = &gsi_ctx->chan[chan_hdl];

	mutex_lock(&ctx->mlock);
	ctx->scratch.data.word3 = val.data.word1;
	__gsi_write_wdi3_channel_scratch2_reg(chan_hdl, val);
	mutex_unlock(&ctx->mlock);

	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_write_wdi3_channel_scratch2_reg);


int gsi_read_channel_scratch(unsigned long chan_hdl,
		union __packed gsi_channel_scratch *val)
{
	struct gsi_chan_ctx *ctx;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (chan_hdl >= gsi_ctx->max_ch) {
		GSIERR("bad params chan_hdl=%lu\n", chan_hdl);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (gsi_ctx->chan[chan_hdl].state != GSI_CHAN_STATE_ALLOCATED &&
		gsi_ctx->chan[chan_hdl].state != GSI_CHAN_STATE_STARTED &&
		gsi_ctx->chan[chan_hdl].state != GSI_CHAN_STATE_STOPPED) {
		GSIERR("bad state %d\n",
				gsi_ctx->chan[chan_hdl].state);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	ctx = &gsi_ctx->chan[chan_hdl];

	mutex_lock(&ctx->mlock);
	__gsi_read_channel_scratch(chan_hdl, val);
	mutex_unlock(&ctx->mlock);

	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_read_channel_scratch);

int gsi_read_wdi3_channel_scratch2_reg(unsigned long chan_hdl,
		union __packed gsi_wdi3_channel_scratch2_reg * val)
{
	struct gsi_chan_ctx *ctx;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (chan_hdl >= gsi_ctx->max_ch) {
		GSIERR("bad params chan_hdl=%lu\n", chan_hdl);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (gsi_ctx->chan[chan_hdl].state != GSI_CHAN_STATE_ALLOCATED &&
		gsi_ctx->chan[chan_hdl].state != GSI_CHAN_STATE_STARTED &&
		gsi_ctx->chan[chan_hdl].state != GSI_CHAN_STATE_STOPPED) {
		GSIERR("bad state %d\n",
				gsi_ctx->chan[chan_hdl].state);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	ctx = &gsi_ctx->chan[chan_hdl];

	mutex_lock(&ctx->mlock);
	__gsi_read_wdi3_channel_scratch2_reg(chan_hdl, val);
	mutex_unlock(&ctx->mlock);

	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_read_wdi3_channel_scratch2_reg);


int gsi_update_mhi_channel_scratch(unsigned long chan_hdl,
		struct __packed gsi_mhi_channel_scratch mscr)
{
	struct gsi_chan_ctx *ctx;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (chan_hdl >= gsi_ctx->max_ch) {
		GSIERR("bad params chan_hdl=%lu\n", chan_hdl);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (gsi_ctx->chan[chan_hdl].state != GSI_CHAN_STATE_ALLOCATED &&
		gsi_ctx->chan[chan_hdl].state != GSI_CHAN_STATE_STOPPED) {
		GSIERR("bad state %d\n",
				gsi_ctx->chan[chan_hdl].state);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	ctx = &gsi_ctx->chan[chan_hdl];

	mutex_lock(&ctx->mlock);
	ctx->scratch = __gsi_update_mhi_channel_scratch(chan_hdl, mscr);
	mutex_unlock(&ctx->mlock);

	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_update_mhi_channel_scratch);

int gsi_query_channel_db_addr(unsigned long chan_hdl,
		uint32_t *db_addr_wp_lsb, uint32_t *db_addr_wp_msb)
{
	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (!db_addr_wp_msb || !db_addr_wp_lsb) {
		GSIERR("bad params msb=%pK lsb=%pK\n", db_addr_wp_msb,
				db_addr_wp_lsb);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (chan_hdl >= gsi_ctx->max_ch) {
		GSIERR("bad params chan_hdl=%lu\n", chan_hdl);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (gsi_ctx->chan[chan_hdl].state == GSI_CHAN_STATE_NOT_ALLOCATED) {
		GSIERR("bad state %d\n",
				gsi_ctx->chan[chan_hdl].state);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	*db_addr_wp_lsb = gsi_ctx->per.phys_addr +
		gsihal_get_reg_nk_ofst(GSI_EE_n_GSI_CH_k_DOORBELL_0,
			gsi_ctx->per.ee, chan_hdl);
	*db_addr_wp_msb = gsi_ctx->per.phys_addr +
		gsihal_get_reg_nk_ofst(GSI_EE_n_GSI_CH_k_DOORBELL_1,
			gsi_ctx->per.ee, chan_hdl);

	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_query_channel_db_addr);

int gsi_pending_irq_type(void)
{
	int ee = gsi_ctx->per.ee;

	return gsihal_read_reg_n(GSI_EE_n_CNTXT_TYPE_IRQ, ee);
}
EXPORT_SYMBOL(gsi_pending_irq_type);

int gsi_start_channel(unsigned long chan_hdl)
{
	enum gsi_ch_cmd_opcode op = GSI_CH_START;
	uint32_t val;
	struct gsihal_reg_ee_n_gsi_ch_cmd ch_cmd;
	struct gsi_chan_ctx *ctx;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (chan_hdl >= gsi_ctx->max_ch) {
		GSIERR("bad params chan_hdl=%lu\n", chan_hdl);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	ctx = &gsi_ctx->chan[chan_hdl];

	if (ctx->state == GSI_CHAN_STATE_STARTED) {
		GSIDBG("chan_hdl=%lu already in started state\n", chan_hdl);
		return GSI_STATUS_SUCCESS;
	}

	if (ctx->state != GSI_CHAN_STATE_ALLOCATED &&
		ctx->state != GSI_CHAN_STATE_STOP_IN_PROC &&
		ctx->state != GSI_CHAN_STATE_STOPPED) {
		GSIERR("bad state %d\n", ctx->state);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	mutex_lock(&gsi_ctx->mlock);
	reinit_completion(&ctx->compl);

	/* check if INTSET is in IRQ mode for GPI channel */
	val = gsihal_read_reg_n(GSI_EE_n_CNTXT_INTSET, gsi_ctx->per.ee);
	if (ctx->evtr &&
		ctx->evtr->props.intf == GSI_EVT_CHTYPE_GPI_EV &&
		val != GSI_INTR_IRQ) {
		GSIERR("GSI_EE_n_CNTXT_INTSET %d\n", val);
		BUG();
	}

	gsi_ctx->ch_dbg[chan_hdl].ch_start++;
	ch_cmd.chid = chan_hdl;
	ch_cmd.opcode = op;
	gsihal_write_reg_n_fields(GSI_EE_n_GSI_CH_CMD,
		gsi_ctx->per.ee, &ch_cmd);
	GSIDBG("GSI Channel Start, waiting for completion\n");
	gsi_channel_state_change_wait(chan_hdl,
		ctx,
		GSI_START_CMD_TIMEOUT_MS, op);

	if (ctx->state != GSI_CHAN_STATE_STARTED &&
		ctx->state != GSI_CHAN_STATE_FLOW_CONTROL) {
		/*
		 * Hardware returned unexpected status, unexpected
		 * hardware state.
		 */
		GSIERR("chan=%lu timed out, unexpected state=%u\n",
			chan_hdl, ctx->state);
		gsi_dump_ch_info(chan_hdl);
		GSI_ASSERT();
	}

	GSIDBG("GSI Channel=%lu Start success\n", chan_hdl);

	/* write order MUST be MSB followed by LSB */
	gsihal_write_reg_nk(GSI_EE_n_GSI_CH_k_DOORBELL_1,
		gsi_ctx->per.ee, ctx->props.ch_id, GSI_MSB(ctx->ring.wp_local));

	mutex_unlock(&gsi_ctx->mlock);

	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_start_channel);

void gsi_dump_ch_info(unsigned long chan_hdl)
{
	uint32_t val;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return;
	}

	if (chan_hdl >= gsi_ctx->max_ch) {
		GSIDBG("invalid chan id %lu\n", chan_hdl);
		return;
	}

	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_0,
		gsi_ctx->per.ee, chan_hdl);
	GSIERR("CH%2lu CTX0  0x%x\n", chan_hdl, val);
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_1,
		gsi_ctx->per.ee, chan_hdl);
	GSIERR("CH%2lu CTX1  0x%x\n", chan_hdl, val);
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_2,
		gsi_ctx->per.ee, chan_hdl);
	GSIERR("CH%2lu CTX2  0x%x\n", chan_hdl, val);
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_3,
		gsi_ctx->per.ee, chan_hdl);
	GSIERR("CH%2lu CTX3  0x%x\n", chan_hdl, val);
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_4,
		gsi_ctx->per.ee, chan_hdl);
	GSIERR("CH%2lu CTX4  0x%x\n", chan_hdl, val);
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_5,
		gsi_ctx->per.ee, chan_hdl);
	GSIERR("CH%2lu CTX5  0x%x\n", chan_hdl, val);
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_6,
		gsi_ctx->per.ee, chan_hdl);
	GSIERR("CH%2lu CTX6  0x%x\n", chan_hdl, val);
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_7,
		gsi_ctx->per.ee, chan_hdl);
	GSIERR("CH%2lu CTX7  0x%x\n", chan_hdl, val);
	if (gsi_ctx->per.ver >= GSI_VER_3_0) {
		val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_8,
			gsi_ctx->per.ee, chan_hdl);
		GSIERR("CH%2lu CTX8  0x%x\n", chan_hdl, val);
	}
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_RE_FETCH_READ_PTR,
		gsi_ctx->per.ee, chan_hdl);
	GSIERR("CH%2lu REFRP 0x%x\n", chan_hdl, val);
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_RE_FETCH_WRITE_PTR,
		gsi_ctx->per.ee, chan_hdl);
	GSIERR("CH%2lu REFWP 0x%x\n", chan_hdl, val);
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_QOS,
		gsi_ctx->per.ee, chan_hdl);
	GSIERR("CH%2lu QOS   0x%x\n", chan_hdl, val);
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_0,
		gsi_ctx->per.ee, chan_hdl);
	GSIERR("CH%2lu SCR0  0x%x\n", chan_hdl, val);
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_1,
		gsi_ctx->per.ee, chan_hdl);
	GSIERR("CH%2lu SCR1  0x%x\n", chan_hdl, val);
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_2,
		gsi_ctx->per.ee, chan_hdl);
	GSIERR("CH%2lu SCR2  0x%x\n", chan_hdl, val);
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_3,
		gsi_ctx->per.ee, chan_hdl);
	GSIERR("CH%2lu SCR3  0x%x\n", chan_hdl, val);
	if (gsi_ctx->per.ver >= GSI_VER_3_0) {
		val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_4,
			gsi_ctx->per.ee, chan_hdl);
		GSIERR("CH%2lu SCR4  0x%x\n", chan_hdl, val);
		val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_5,
			gsi_ctx->per.ee, chan_hdl);
		GSIERR("CH%2lu SCR5  0x%x\n", chan_hdl, val);
		val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_6,
			gsi_ctx->per.ee, chan_hdl);
		GSIERR("CH%2lu SCR6  0x%x\n", chan_hdl, val);
		val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_7,
			gsi_ctx->per.ee, chan_hdl);
		GSIERR("CH%2lu SCR7  0x%x\n", chan_hdl, val);
		val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_8,
			gsi_ctx->per.ee, chan_hdl);
		GSIERR("CH%2lu SCR8  0x%x\n", chan_hdl, val);
		val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_9,
			gsi_ctx->per.ee, chan_hdl);
		GSIERR("CH%2lu SCR9  0x%x\n", chan_hdl, val);
	}

	return;
}
EXPORT_SYMBOL(gsi_dump_ch_info);

int gsi_stop_channel(unsigned long chan_hdl)
{
	enum gsi_ch_cmd_opcode op = GSI_CH_STOP;
	int res;
	uint32_t val;
	struct gsihal_reg_ee_n_gsi_ch_cmd ch_cmd;
	struct gsi_chan_ctx *ctx;
	unsigned long flags;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (chan_hdl >= gsi_ctx->max_ch) {
		GSIERR("bad params chan_hdl=%lu\n", chan_hdl);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	ctx = &gsi_ctx->chan[chan_hdl];

	if (ctx->state == GSI_CHAN_STATE_STOPPED) {
		GSIDBG("chan_hdl=%lu already stopped\n", chan_hdl);
		return GSI_STATUS_SUCCESS;
	}

	if (ctx->state != GSI_CHAN_STATE_STARTED &&
		ctx->state != GSI_CHAN_STATE_STOP_IN_PROC &&
		ctx->state != GSI_CHAN_STATE_ERROR) {
		GSIERR("bad state %d\n", ctx->state);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	mutex_lock(&gsi_ctx->mlock);
	reinit_completion(&ctx->compl);

	/* check if INTSET is in IRQ mode for GPI channel */
	val = gsihal_read_reg_n(GSI_EE_n_CNTXT_INTSET, gsi_ctx->per.ee);
	if (ctx->evtr &&
		ctx->evtr->props.intf == GSI_EVT_CHTYPE_GPI_EV &&
		val != GSI_INTR_IRQ) {
		GSIERR("GSI_EE_n_CNTXT_INTSET %d\n", val);
		BUG();
	}

	gsi_ctx->ch_dbg[chan_hdl].ch_stop++;
	ch_cmd.chid = chan_hdl;
	ch_cmd.opcode = op;
	gsihal_write_reg_n_fields(GSI_EE_n_GSI_CH_CMD,
		gsi_ctx->per.ee, &ch_cmd);

	GSIDBG("GSI Channel Stop, waiting for completion: 0x%x\n", val);
	gsi_channel_state_change_wait(chan_hdl,
		ctx,
		GSI_STOP_CMD_TIMEOUT_MS, op);

	if (ctx->state != GSI_CHAN_STATE_STOPPED &&
		ctx->state != GSI_CHAN_STATE_STOP_IN_PROC) {
		GSIERR("chan=%lu unexpected state=%u\n", chan_hdl, ctx->state);
		gsi_dump_ch_info(chan_hdl);
		res = -GSI_STATUS_BAD_STATE;
		BUG();
		goto free_lock;
	}

	if (ctx->state == GSI_CHAN_STATE_STOP_IN_PROC) {
		GSIERR("chan=%lu busy try again\n", chan_hdl);
		res = -GSI_STATUS_AGAIN;
		goto free_lock;
	}

	/* If channel is stopped succesfully and has an event with IRQ type MSI
		- clear IEOB */
	if (ctx->evtr && ctx->evtr->props.intr == GSI_INTR_MSI) {
		spin_lock_irqsave(&ctx->evtr->ring.slock, flags);
		if (gsi_ctx->per.ver >= GSI_VER_3_0) {
			gsihal_write_reg_nk(GSI_EE_n_CNTXT_SRC_IEOB_IRQ_CLR_k,
				gsi_ctx->per.ee, gsihal_get_ch_reg_idx(ctx->evtr->id),
				gsihal_get_ch_reg_mask(ctx->evtr->id));
		} else {
			gsihal_write_reg_n(GSI_EE_n_CNTXT_SRC_IEOB_IRQ_CLR,
				gsi_ctx->per.ee, 1 << ctx->evtr->id);
		}
		spin_unlock_irqrestore(&ctx->evtr->ring.slock, flags);
	}

	res = GSI_STATUS_SUCCESS;

free_lock:
	mutex_unlock(&gsi_ctx->mlock);
	return res;
}
EXPORT_SYMBOL(gsi_stop_channel);

int gsi_stop_db_channel(unsigned long chan_hdl)
{
	enum gsi_ch_cmd_opcode op = GSI_CH_DB_STOP;
	int res;
	struct gsihal_reg_ee_n_gsi_ch_cmd ch_cmd;
	struct gsi_chan_ctx *ctx;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (chan_hdl >= gsi_ctx->max_ch) {
		GSIERR("bad params chan_hdl=%lu\n", chan_hdl);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	ctx = &gsi_ctx->chan[chan_hdl];

	if (ctx->state == GSI_CHAN_STATE_STOPPED) {
		GSIDBG("chan_hdl=%lu already stopped\n", chan_hdl);
		return GSI_STATUS_SUCCESS;
	}

	if (ctx->state != GSI_CHAN_STATE_STARTED &&
		ctx->state != GSI_CHAN_STATE_STOP_IN_PROC) {
		GSIERR("bad state %d\n", ctx->state);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	mutex_lock(&gsi_ctx->mlock);
	reinit_completion(&ctx->compl);

	gsi_ctx->ch_dbg[chan_hdl].ch_db_stop++;
	ch_cmd.chid = chan_hdl;
	ch_cmd.opcode = op;
	gsihal_write_reg_n_fields(GSI_EE_n_GSI_CH_CMD,
		gsi_ctx->per.ee, &ch_cmd);
	res = wait_for_completion_timeout(&ctx->compl,
			msecs_to_jiffies(GSI_STOP_CMD_TIMEOUT_MS));
	if (res == 0) {
		GSIERR("chan_hdl=%lu timed out\n", chan_hdl);
		res = -GSI_STATUS_TIMED_OUT;
		goto free_lock;
	}

	if (ctx->state != GSI_CHAN_STATE_STOPPED &&
		ctx->state != GSI_CHAN_STATE_STOP_IN_PROC) {
		GSIERR("chan=%lu unexpected state=%u\n", chan_hdl, ctx->state);
		res = -GSI_STATUS_BAD_STATE;
		goto free_lock;
	}

	if (ctx->state == GSI_CHAN_STATE_STOP_IN_PROC) {
		GSIERR("chan=%lu busy try again\n", chan_hdl);
		res = -GSI_STATUS_AGAIN;
		goto free_lock;
	}

	res = GSI_STATUS_SUCCESS;

free_lock:
	mutex_unlock(&gsi_ctx->mlock);
	return res;
}
EXPORT_SYMBOL(gsi_stop_db_channel);

int gsi_reset_channel(unsigned long chan_hdl)
{
	enum gsi_ch_cmd_opcode op = GSI_CH_RESET;
	int res;
	struct gsihal_reg_ee_n_gsi_ch_cmd ch_cmd;
	struct gsi_chan_ctx *ctx;
	bool reset_done = false;
	uint32_t retry_cnt = 0;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (chan_hdl >= gsi_ctx->max_ch) {
		GSIERR("bad params chan_hdl=%lu\n", chan_hdl);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	ctx = &gsi_ctx->chan[chan_hdl];

	/*
	 * In WDI3 case, if SAP enabled but no client connected,
	 * GSI will be in allocated state. When SAP disabled,
	 * gsi_reset_channel will be called and reset is needed.
	 */
	if (ctx->state != GSI_CHAN_STATE_STOPPED &&
		ctx->state != GSI_CHAN_STATE_ALLOCATED) {
		GSIERR("bad state %d\n", ctx->state);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	mutex_lock(&gsi_ctx->mlock);

reset:
	reinit_completion(&ctx->compl);
	gsi_ctx->ch_dbg[chan_hdl].ch_reset++;
	ch_cmd.chid = chan_hdl;
	ch_cmd.opcode = op;
	gsihal_write_reg_n_fields(GSI_EE_n_GSI_CH_CMD,
		gsi_ctx->per.ee, &ch_cmd);
	res = wait_for_completion_timeout(&ctx->compl, GSI_CMD_TIMEOUT);
	if (res == 0) {
		GSIERR("chan_hdl=%lu timed out\n", chan_hdl);
		mutex_unlock(&gsi_ctx->mlock);
		return -GSI_STATUS_TIMED_OUT;
	}

revrfy_chnlstate:
	if (ctx->state != GSI_CHAN_STATE_ALLOCATED) {
		GSIERR("chan_hdl=%lu unexpected state=%u\n", chan_hdl,
				ctx->state);
		/* GSI register update state not sync with gsi channel
		 * context state not sync, need to wait for 1ms to sync.
		 */
		retry_cnt++;
		if (retry_cnt <= GSI_CHNL_STATE_MAX_RETRYCNT) {
			usleep_range(GSI_RESET_WA_MIN_SLEEP,
				GSI_RESET_WA_MAX_SLEEP);
			goto revrfy_chnlstate;
		}
		/*
		 * Hardware returned incorrect state, unexpected
		 * hardware state.
		 */
		GSI_ASSERT();
	}

	/* Hardware issue fixed from GSI 2.0 and no need for the WA */
	if (gsi_ctx->per.ver >= GSI_VER_2_0)
		reset_done = true;

	/* workaround: reset GSI producers again */
	if (ctx->props.dir == GSI_CHAN_DIR_FROM_GSI && !reset_done) {
		usleep_range(GSI_RESET_WA_MIN_SLEEP, GSI_RESET_WA_MAX_SLEEP);
		reset_done = true;
		goto reset;
	}

	if (ctx->props.cleanup_cb)
		gsi_cleanup_xfer_user_data(chan_hdl, ctx->props.cleanup_cb);

	gsi_program_chan_ctx(&ctx->props, gsi_ctx->per.ee,
			ctx->evtr ? ctx->evtr->id : GSI_NO_EVT_ERINDEX);
	gsi_init_chan_ring(&ctx->props, &ctx->ring);

	/* restore scratch */
	__gsi_write_channel_scratch(chan_hdl, ctx->scratch);

	mutex_unlock(&gsi_ctx->mlock);

	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_reset_channel);

int gsi_dealloc_channel(unsigned long chan_hdl)
{
	enum gsi_ch_cmd_opcode op = GSI_CH_DE_ALLOC;
	int res;
	struct gsihal_reg_ee_n_gsi_ch_cmd ch_cmd;
	struct gsi_chan_ctx *ctx;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (chan_hdl >= gsi_ctx->max_ch) {
		GSIERR("bad params chan_hdl=%lu\n", chan_hdl);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	ctx = &gsi_ctx->chan[chan_hdl];

	if (ctx->state != GSI_CHAN_STATE_ALLOCATED) {
		GSIERR("bad state %d\n", ctx->state);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	/*In GSI_VER_2_2 version deallocation channel not supported*/
	if (gsi_ctx->per.ver != GSI_VER_2_2) {
		mutex_lock(&gsi_ctx->mlock);
		reinit_completion(&ctx->compl);

		gsi_ctx->ch_dbg[chan_hdl].ch_de_alloc++;
		ch_cmd.chid = chan_hdl;
		ch_cmd.opcode = op;
		gsihal_write_reg_n_fields(GSI_EE_n_GSI_CH_CMD,
			gsi_ctx->per.ee, &ch_cmd);
		res = wait_for_completion_timeout(&ctx->compl, GSI_CMD_TIMEOUT);
		if (res == 0) {
			GSIERR("chan_hdl=%lu timed out\n", chan_hdl);
			mutex_unlock(&gsi_ctx->mlock);
			return -GSI_STATUS_TIMED_OUT;
		}
		if (ctx->state != GSI_CHAN_STATE_NOT_ALLOCATED) {
			GSIERR("chan_hdl=%lu unexpected state=%u\n", chan_hdl,
					ctx->state);
			/* Hardware returned incorrect value */
			GSI_ASSERT();
		}

		mutex_unlock(&gsi_ctx->mlock);
	} else {
		mutex_lock(&gsi_ctx->mlock);
		GSIDBG("In GSI_VER_2_2 channel deallocation not supported\n");
		ctx->state = GSI_CHAN_STATE_NOT_ALLOCATED;
		GSIDBG("chan_hdl=%lu Channel state = %u\n", chan_hdl,
								ctx->state);
		mutex_unlock(&gsi_ctx->mlock);
	}
	devm_kfree(gsi_ctx->dev, ctx->user_data);
	ctx->allocated = false;
	if (ctx->evtr && (ctx->props.prot != GSI_CHAN_PROT_GCI)) {
		atomic_dec(&ctx->evtr->chan_ref_cnt);
		ctx->evtr->num_of_chan_allocated--;
	}
	atomic_dec(&gsi_ctx->num_chan);

	if (ctx->props.prot == GSI_CHAN_PROT_GCI) {
		gsi_ctx->coal_info.ch_id = GSI_CHAN_MAX;
		gsi_ctx->coal_info.evchid = GSI_EVT_RING_MAX;
	}
	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_dealloc_channel);

void gsi_update_ch_dp_stats(struct gsi_chan_ctx *ctx, uint16_t used)
{
	unsigned long now = jiffies_to_msecs(jiffies);
	unsigned long elapsed;

	if (used == 0) {
		elapsed = now - ctx->stats.dp.last_timestamp;
		if (ctx->stats.dp.empty_time < elapsed)
			ctx->stats.dp.empty_time = elapsed;
	}

	if (used <= ctx->props.max_re_expected / 3)
		++ctx->stats.dp.ch_below_lo;
	else if (used <= 2 * ctx->props.max_re_expected / 3)
		++ctx->stats.dp.ch_below_hi;
	else
		++ctx->stats.dp.ch_above_hi;
	ctx->stats.dp.last_timestamp = now;
}

static void __gsi_query_channel_free_re(struct gsi_chan_ctx *ctx,
		uint16_t *num_free_re)
{
	uint16_t start;
	uint16_t end;
	uint64_t rp;
	int ee = gsi_ctx->per.ee;
	uint16_t used;

	WARN_ON(ctx->props.prot != GSI_CHAN_PROT_GPI);

	if (!ctx->evtr) {
		rp = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_4,
			ee, ctx->props.ch_id);
		rp |= ctx->ring.rp & GSI_MSB_MASK;
		ctx->ring.rp = rp;
	} else {
		rp = ctx->ring.rp_local;
	}

	start = gsi_find_idx_from_addr(&ctx->ring, rp);
	end = gsi_find_idx_from_addr(&ctx->ring, ctx->ring.wp_local);

	if (end >= start)
		used = end - start;
	else
		used = ctx->ring.max_num_elem + 1 - (start - end);

	*num_free_re = ctx->ring.max_num_elem - used;
}

int gsi_query_channel_info(unsigned long chan_hdl,
		struct gsi_chan_info *info)
{
	struct gsi_chan_ctx *ctx;
	spinlock_t *slock;
	unsigned long flags;
	uint64_t rp;
	uint64_t wp;
	int ee;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (chan_hdl >= gsi_ctx->max_ch || !info) {
		GSIERR("bad params chan_hdl=%lu info=%pK\n", chan_hdl, info);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	ctx = &gsi_ctx->chan[chan_hdl];
	if (ctx->evtr) {
		slock = &ctx->evtr->ring.slock;
		info->evt_valid = true;
	} else {
		slock = &ctx->ring.slock;
		info->evt_valid = false;
	}

	spin_lock_irqsave(slock, flags);

	ee = gsi_ctx->per.ee;
	rp = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_4,
		ee, ctx->props.ch_id);
	rp |= ((uint64_t)gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_5,
		ee, ctx->props.ch_id)) << 32;
	ctx->ring.rp = rp;
	info->rp = rp;

	wp = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_6,
		ee, ctx->props.ch_id);
	wp |= ((uint64_t)gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_7,
		ee, ctx->props.ch_id)) << 32;
	ctx->ring.wp = wp;
	info->wp = wp;

	if (info->evt_valid) {
		rp = gsihal_read_reg_nk(GSI_EE_n_EV_CH_k_CNTXT_4,
			ee, ctx->evtr->id);
		rp |= ((uint64_t)gsihal_read_reg_nk(GSI_EE_n_EV_CH_k_CNTXT_5,
			ee, ctx->evtr->id)) << 32;
		info->evt_rp = rp;

		wp = gsihal_read_reg_nk(GSI_EE_n_EV_CH_k_CNTXT_6,
			ee, ctx->evtr->id);
		wp |= ((uint64_t)gsihal_read_reg_nk(GSI_EE_n_EV_CH_k_CNTXT_7,
			ee, ctx->evtr->id)) << 32;
		info->evt_wp = wp;
	}

	spin_unlock_irqrestore(slock, flags);

	GSIDBG("ch=%lu RP=0x%llx WP=0x%llx ev_valid=%d ERP=0x%llx EWP=0x%llx\n",
			chan_hdl, info->rp, info->wp,
			info->evt_valid, info->evt_rp, info->evt_wp);

	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_query_channel_info);

int gsi_is_teth_channel_empty(unsigned long chan_hdl, bool *is_empty)
{
	uint32_t rp;
	uint32_t wp;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (chan_hdl >= gsi_ctx->max_ch || !is_empty) {
		GSIERR("bad params chan_hdl=%lu is_empty=%pK\n",
				chan_hdl, is_empty);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	rp = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_RE_FETCH_READ_PTR,
			gsi_ctx->per.ee, chan_hdl);
	wp = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_RE_FETCH_WRITE_PTR,
			gsi_ctx->per.ee, chan_hdl);
	if (rp == wp) {
		GSIDBG_LOW("Teth channel empty\n");
		*is_empty = true;
	} else {
		GSIDBG("Teth channel not empty ch=%lu rp = 0x%x wp = 0x%x\n",
				chan_hdl, rp, wp);
		*is_empty = false;
	}

	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL_GPL(gsi_is_teth_channel_empty);

int gsi_is_channel_empty(unsigned long chan_hdl, bool *is_empty)
{
	struct gsi_chan_ctx *ctx;
	struct gsi_evt_ctx *ev_ctx;
	spinlock_t *slock;
	unsigned long flags;
	uint64_t rp;
	uint64_t wp;
	uint64_t rp_local;
	int ee;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (chan_hdl >= gsi_ctx->max_ch || !is_empty) {
		GSIERR("bad params chan_hdl=%lu is_empty=%pK\n",
				chan_hdl, is_empty);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	ctx = &gsi_ctx->chan[chan_hdl];
	ee = gsi_ctx->per.ee;

	if (ctx->props.prot != GSI_CHAN_PROT_GPI &&
		ctx->props.prot != GSI_CHAN_PROT_GCI) {
		GSIERR("op not supported for protocol %u\n", ctx->props.prot);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	if (ctx->evtr)
		slock = &ctx->evtr->ring.slock;
	else
		slock = &ctx->ring.slock;

	spin_lock_irqsave(slock, flags);

	if (ctx->props.dir == GSI_CHAN_DIR_FROM_GSI && ctx->evtr) {
		ev_ctx = &gsi_ctx->evtr[ctx->evtr->id];
		/* Read the event ring rp from DDR to avoid mismatch */
		rp = ev_ctx->props.gsi_read_event_ring_rp(&ev_ctx->props,
					ev_ctx->id, ee);

		rp |= ctx->evtr->ring.rp & GSI_MSB_MASK;
		ctx->evtr->ring.rp = rp;

		wp = gsihal_read_reg_nk(GSI_EE_n_EV_CH_k_CNTXT_6,
			ee, ctx->evtr->id);
		wp |= ctx->evtr->ring.wp & GSI_MSB_MASK;
		ctx->evtr->ring.wp = wp;

		rp_local = ctx->evtr->ring.rp_local;
	} else {
		rp = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_4,
			ee, ctx->props.ch_id);
		rp |= ctx->ring.rp & GSI_MSB_MASK;
		ctx->ring.rp = rp;

		wp = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_6,
			ee, ctx->props.ch_id);
		wp |= ctx->ring.wp & GSI_MSB_MASK;
		ctx->ring.wp = wp;

		rp_local = ctx->ring.rp_local;
	}

	if (ctx->props.dir == GSI_CHAN_DIR_FROM_GSI)
		*is_empty = (rp_local == rp) ? true : false;
	else
		*is_empty = (wp == rp) ? true : false;

	spin_unlock_irqrestore(slock, flags);

	if (ctx->props.dir == GSI_CHAN_DIR_FROM_GSI && ctx->evtr)
		GSIDBG("ch=%ld ev=%d RP=0x%llx WP=0x%llx RP_LOCAL=0x%llx\n",
			chan_hdl, ctx->evtr->id, rp, wp, rp_local);
	else
		GSIDBG("ch=%lu RP=0x%llx WP=0x%llx RP_LOCAL=0x%llx\n",
			chan_hdl, rp, wp, rp_local);

	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_is_channel_empty);

bool gsi_is_event_pending(unsigned long chan_hdl) {
	struct gsi_chan_ctx *ctx;
	uint64_t rp;
	uint64_t rp_local;
	int ee;

	if (chan_hdl >= gsi_ctx->max_ch) {
		GSIERR("bad params chan_hdl=%lu\n", chan_hdl);
		return false;
	}

	ctx = &gsi_ctx->chan[chan_hdl];
	ee = gsi_ctx->per.ee;

	/* read only, updating will be handled in NAPI context if needed */
	rp = ctx->evtr->props.gsi_read_event_ring_rp(
		&ctx->evtr->props, ctx->evtr->id, ee);
	rp |= ctx->evtr->ring.rp & GSI_MSB_MASK;
	rp_local = ctx->evtr->ring.rp_local;

	return rp != rp_local;
}
EXPORT_SYMBOL(gsi_is_event_pending);

int __gsi_get_gci_cookie(struct gsi_chan_ctx *ctx, uint16_t idx)
{
	int i;
	int end;

	if (!ctx->user_data[idx].valid) {
		ctx->user_data[idx].valid = true;
		return idx;
	}

	/*
	 * at this point we need to find an "escape buffer" for the cookie
	 * as the userdata in this spot is in use. This happens if the TRE at
	 * idx is not completed yet and it is getting reused by a new TRE.
	 */
	ctx->stats.userdata_in_use++;
	end = ctx->ring.max_num_elem + 1;
	for (i = 0; i < GSI_VEID_MAX; i++) {
		if (!ctx->user_data[end + i].valid) {
			ctx->user_data[end + i].valid = true;
			return end + i;
		}
	}

	/* Go over original userdata when escape buffer is full (costly) */
	GSIDBG("escape buffer is full\n");
	for (i = 0; i < end; i++) {
		if (!ctx->user_data[i].valid) {
			ctx->user_data[i].valid = true;
			return i;
		}
	}

	/* Everything is full (possibly a stall) */
	GSIERR("both userdata array and escape buffer is full\n");
	BUG();
	return 0xFFFF;
}

int __gsi_populate_gci_tre(struct gsi_chan_ctx *ctx,
	struct gsi_xfer_elem *xfer)
{
	struct gsi_gci_tre gci_tre;
	struct gsi_gci_tre *tre_gci_ptr;
	uint16_t idx;

	memset(&gci_tre, 0, sizeof(gci_tre));
	if (xfer->addr & 0xFFFFFF0000000000) {
		GSIERR("chan_hdl=%u add too large=%llx\n",
			ctx->props.ch_id, xfer->addr);
		return -EINVAL;
	}

	if (xfer->type != GSI_XFER_ELEM_DATA) {
		GSIERR("chan_hdl=%u bad RE type=%u\n", ctx->props.ch_id,
			xfer->type);
		return -EINVAL;
	}

	idx = gsi_find_idx_from_addr(&ctx->ring, ctx->ring.wp_local);
	tre_gci_ptr = (struct gsi_gci_tre *)(ctx->ring.base_va +
		idx * ctx->ring.elem_sz);

	gci_tre.buffer_ptr = xfer->addr;
	gci_tre.buf_len = xfer->len;
	gci_tre.re_type = GSI_RE_COAL;
	gci_tre.cookie = __gsi_get_gci_cookie(ctx, idx);
	if (gci_tre.cookie > (ctx->ring.max_num_elem + GSI_VEID_MAX))
		return -EPERM;

	/* write the TRE to ring */
	*tre_gci_ptr = gci_tre;
	ctx->user_data[gci_tre.cookie].p = xfer->xfer_user_data;

	return 0;
}

int __gsi_populate_tre(struct gsi_chan_ctx *ctx,
	struct gsi_xfer_elem *xfer)
{
	struct gsi_tre tre;
	struct gsi_tre *tre_ptr;
	uint16_t idx;

	memset(&tre, 0, sizeof(tre));
	tre.buffer_ptr = xfer->addr;
	tre.buf_len = xfer->len;
	if (xfer->type == GSI_XFER_ELEM_DATA) {
		tre.re_type = GSI_RE_XFER;
	} else if (xfer->type == GSI_XFER_ELEM_IMME_CMD) {
		tre.re_type = GSI_RE_IMMD_CMD;
	} else if (xfer->type == GSI_XFER_ELEM_NOP) {
		tre.re_type = GSI_RE_NOP;
	} else {
		GSIERR("chan_hdl=%u bad RE type=%u\n", ctx->props.ch_id,
			xfer->type);
		return -EINVAL;
	}

	tre.bei = (xfer->flags & GSI_XFER_FLAG_BEI) ? 1 : 0;
	tre.ieot = (xfer->flags & GSI_XFER_FLAG_EOT) ? 1 : 0;
	tre.ieob = (xfer->flags & GSI_XFER_FLAG_EOB) ? 1 : 0;
	tre.chain = (xfer->flags & GSI_XFER_FLAG_CHAIN) ? 1 : 0;

	if (unlikely(ctx->state  == GSI_CHAN_STATE_NOT_ALLOCATED)) {
		GSIERR("bad state %d\n", ctx->state);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	idx = gsi_find_idx_from_addr(&ctx->ring, ctx->ring.wp_local);
	tre_ptr = (struct gsi_tre *)(ctx->ring.base_va +
		idx * ctx->ring.elem_sz);

	/* write the TRE to ring */
	*tre_ptr = tre;
	ctx->user_data[idx].valid = true;
	ctx->user_data[idx].p = xfer->xfer_user_data;

	return 0;
}

int gsi_queue_xfer(unsigned long chan_hdl, uint16_t num_xfers,
		struct gsi_xfer_elem *xfer, bool ring_db)
{
	struct gsi_chan_ctx *ctx;
	uint16_t free;
	uint64_t wp_rollback;
	int i;
	spinlock_t *slock;
	unsigned long flags;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (chan_hdl >= gsi_ctx->max_ch || (num_xfers && !xfer)) {
		GSIERR("bad params chan_hdl=%lu num_xfers=%u xfer=%pK\n",
				chan_hdl, num_xfers, xfer);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (unlikely(gsi_ctx->chan[chan_hdl].state
				 == GSI_CHAN_STATE_NOT_ALLOCATED)) {
		GSIERR("bad state %d\n",
			   gsi_ctx->chan[chan_hdl].state);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}


	ctx = &gsi_ctx->chan[chan_hdl];

	if (ctx->props.prot != GSI_CHAN_PROT_GPI &&
			ctx->props.prot != GSI_CHAN_PROT_GCI) {
		GSIERR("op not supported for protocol %u\n", ctx->props.prot);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	if (ctx->evtr)
		slock = &ctx->evtr->ring.slock;
	else
		slock = &ctx->ring.slock;

	spin_lock_irqsave(slock, flags);

	/* allow only ring doorbell */
	if (!num_xfers)
		goto ring_doorbell;

	/*
	 * for GCI channels the responsibility is on the caller to make sure
	 * there is enough room in the TRE.
	 */
	if (ctx->props.prot != GSI_CHAN_PROT_GCI) {
		__gsi_query_channel_free_re(ctx, &free);
		if (num_xfers > free) {
			GSIERR_RL("chan_hdl=%lu num_xfers=%u free=%u\n",
				chan_hdl, num_xfers, free);
			spin_unlock_irqrestore(slock, flags);
			return -GSI_STATUS_RING_INSUFFICIENT_SPACE;
		}
	}

	wp_rollback = ctx->ring.wp_local;
	for (i = 0; i < num_xfers; i++) {
		if (ctx->props.prot == GSI_CHAN_PROT_GCI) {
			if (__gsi_populate_gci_tre(ctx, &xfer[i]))
				break;
		} else {
			if (__gsi_populate_tre(ctx, &xfer[i]))
				break;
		}
		gsi_incr_ring_wp(&ctx->ring);
	}

	if (i != num_xfers) {
		/* reject all the xfers */
		ctx->ring.wp_local = wp_rollback;
		spin_unlock_irqrestore(slock, flags);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	ctx->stats.queued += num_xfers;

ring_doorbell:
	if (ring_db) {
		/* ensure TRE is set before ringing doorbell */
		wmb();
		gsi_ring_chan_doorbell(ctx);
	}

	spin_unlock_irqrestore(slock, flags);

	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_queue_xfer);

int gsi_start_xfer(unsigned long chan_hdl)
{
	struct gsi_chan_ctx *ctx;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (chan_hdl >= gsi_ctx->max_ch) {
		GSIERR("bad params chan_hdl=%lu\n", chan_hdl);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	ctx = &gsi_ctx->chan[chan_hdl];

	if (ctx->props.prot != GSI_CHAN_PROT_GPI &&
		ctx->props.prot != GSI_CHAN_PROT_GCI) {
		GSIERR("op not supported for protocol %u\n", ctx->props.prot);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	if (ctx->state == GSI_CHAN_STATE_NOT_ALLOCATED) {
		GSIERR("bad state %d\n", ctx->state);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	if (ctx->ring.wp == ctx->ring.wp_local)
		return GSI_STATUS_SUCCESS;

	gsi_ring_chan_doorbell(ctx);

	return GSI_STATUS_SUCCESS;
};
EXPORT_SYMBOL(gsi_start_xfer);

int gsi_poll_channel(unsigned long chan_hdl,
		struct gsi_chan_xfer_notify *notify)
{
	int unused_var;

	return gsi_poll_n_channel(chan_hdl, notify, 1, &unused_var);
}
EXPORT_SYMBOL(gsi_poll_channel);

int gsi_poll_n_channel(unsigned long chan_hdl,
		struct gsi_chan_xfer_notify *notify,
		int expected_num, int *actual_num)
{
	struct gsi_chan_ctx *ctx;
	uint64_t rp;
	int ee;
	int i;
	unsigned long flags;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (chan_hdl >= gsi_ctx->max_ch || !notify ||
	    !actual_num || expected_num <= 0) {
		GSIERR("bad params chan_hdl=%lu notify=%pK\n",
			chan_hdl, notify);
		GSIERR("actual_num=%pK expected_num=%d\n",
			actual_num, expected_num);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	ctx = &gsi_ctx->chan[chan_hdl];
	ee = gsi_ctx->per.ee;

	if (ctx->props.prot != GSI_CHAN_PROT_GPI &&
		ctx->props.prot != GSI_CHAN_PROT_GCI) {
		GSIERR("op not supported for protocol %u\n", ctx->props.prot);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	/* Before going to poll packet make sure it was in allocated state */
	if (unlikely(ctx->state  == GSI_CHAN_STATE_NOT_ALLOCATED)) {
		GSIERR("bad state %d\n", ctx->state);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	if (!ctx->evtr) {
		GSIERR("no event ring associated chan_hdl=%lu\n", chan_hdl);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	spin_lock_irqsave(&ctx->evtr->ring.slock, flags);
	if (ctx->evtr->ring.rp == ctx->evtr->ring.rp_local) {
		/* update rp to see of we have anything new to process */
		rp = ctx->evtr->props.gsi_read_event_ring_rp(
			&ctx->evtr->props, ctx->evtr->id, ee);
		rp |= ctx->evtr->ring.rp & GSI_MSB_MASK;

		ctx->evtr->ring.rp = rp;
		/* read gsi event ring rp again if last read is empty */
		if (rp == ctx->evtr->ring.rp_local) {
			/* event ring is empty */
			if (gsi_ctx->per.ver >= GSI_VER_3_0) {
				gsihal_write_reg_nk(GSI_EE_n_CNTXT_SRC_IEOB_IRQ_CLR_k,
					ee, gsihal_get_ch_reg_idx(ctx->evtr->id),
				gsihal_get_ch_reg_mask(ctx->evtr->id));
			}
			else {
				gsihal_write_reg_n(GSI_EE_n_CNTXT_SRC_IEOB_IRQ_CLR,
					ee, 1 << ctx->evtr->id);
			}
			/* do another read to close a small window */
			__iowmb();
			rp = ctx->evtr->props.gsi_read_event_ring_rp(
				&ctx->evtr->props, ctx->evtr->id, ee);
			rp |= ctx->evtr->ring.rp & GSI_MSB_MASK;
			ctx->evtr->ring.rp = rp;
			if (rp == ctx->evtr->ring.rp_local) {
				spin_unlock_irqrestore(
					&ctx->evtr->ring.slock,
					flags);
				ctx->stats.poll_empty++;
				return GSI_STATUS_POLL_EMPTY;
			}
		}
	}

	*actual_num = gsi_get_complete_num(&ctx->evtr->ring,
			ctx->evtr->ring.rp_local, ctx->evtr->ring.rp);

	if (*actual_num > expected_num)
		*actual_num = expected_num;

	for (i = 0; i < *actual_num; i++)
		gsi_process_evt_re(ctx->evtr, notify + i, false);

	spin_unlock_irqrestore(&ctx->evtr->ring.slock, flags);
	ctx->stats.poll_ok++;

	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_poll_n_channel);

int gsi_config_channel_mode(unsigned long chan_hdl, enum gsi_chan_mode mode)
{
	struct gsi_chan_ctx *ctx, *coal_ctx;
	enum gsi_chan_mode curr;
	unsigned long flags;
	enum gsi_chan_mode chan_mode;
	int i;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (chan_hdl >= gsi_ctx->max_ch) {
		GSIERR("bad params chan_hdl=%lu mode=%u\n", chan_hdl, mode);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	ctx = &gsi_ctx->chan[chan_hdl];

	if (ctx->props.prot != GSI_CHAN_PROT_GPI &&
		ctx->props.prot != GSI_CHAN_PROT_GCI) {
		GSIERR("op not supported for protocol %u\n", ctx->props.prot);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	if (!ctx->evtr) {
		GSIERR("cannot configure mode on chan_hdl=%lu\n",
				chan_hdl);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	if (atomic_read(&ctx->poll_mode))
		curr = GSI_CHAN_MODE_POLL;
	else
		curr = GSI_CHAN_MODE_CALLBACK;

	if (mode == curr) {
		GSIDBG("already in requested mode %u chan_hdl=%lu\n",
				curr, chan_hdl);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}
	spin_lock_irqsave(&gsi_ctx->slock, flags);
	if (curr == GSI_CHAN_MODE_CALLBACK &&
			mode == GSI_CHAN_MODE_POLL) {
		if (gsi_ctx->per.ver >= GSI_VER_3_0) {
			if (ctx->evtr->props.intr != GSI_INTR_MSI) {
				__gsi_config_ieob_irq_k(gsi_ctx->per.ee,
				gsihal_get_ch_reg_idx(ctx->evtr->id),
				gsihal_get_ch_reg_mask(ctx->evtr->id),
				0);
			}
		}
		else {
			__gsi_config_ieob_irq(gsi_ctx->per.ee, 1 << ctx->evtr->id, 0);
		}
		if (gsi_ctx->per.ver >= GSI_VER_3_0) {
			gsihal_write_reg_nk(GSI_EE_n_CNTXT_SRC_IEOB_IRQ_CLR_k,
				gsi_ctx->per.ee, gsihal_get_ch_reg_idx(ctx->evtr->id),
				gsihal_get_ch_reg_mask(ctx->evtr->id));
		}
		else {
			gsihal_write_reg_n(GSI_EE_n_CNTXT_SRC_IEOB_IRQ_CLR,
				gsi_ctx->per.ee, 1 << ctx->evtr->id);
		}
		atomic_set(&ctx->poll_mode, mode);
		for(i = 0; i < ctx->evtr->num_of_chan_allocated; i++) {
			atomic_set(&ctx->evtr->chan[i]->poll_mode, mode);
		}
		if ((ctx->props.prot == GSI_CHAN_PROT_GCI) && *ctx->evtr->chan) {
			atomic_set(&ctx->evtr->chan[0]->poll_mode, mode);
		} else if (gsi_ctx->coal_info.evchid == ctx->evtr->id) {
			coal_ctx = &gsi_ctx->chan[gsi_ctx->coal_info.ch_id];
			if (coal_ctx != NULL)
				atomic_set(&coal_ctx->poll_mode, mode);
		}

		GSIDBG("set gsi_ctx evtr_id %d to %d mode\n",
			ctx->evtr->id, mode);
		ctx->stats.callback_to_poll++;
	}

	if (curr == GSI_CHAN_MODE_POLL &&
			mode == GSI_CHAN_MODE_CALLBACK) {
		atomic_set(&ctx->poll_mode, mode);
		for(i = 0; i < ctx->evtr->num_of_chan_allocated; i++) {
			atomic_set(&ctx->evtr->chan[i]->poll_mode, mode);
		}
		if ((ctx->props.prot == GSI_CHAN_PROT_GCI) && *ctx->evtr->chan) {
			atomic_set(&ctx->evtr->chan[0]->poll_mode, mode);
		} else if (gsi_ctx->coal_info.evchid == ctx->evtr->id) {
			coal_ctx = &gsi_ctx->chan[gsi_ctx->coal_info.ch_id];
			if (coal_ctx != NULL)
				atomic_set(&coal_ctx->poll_mode, mode);
		}
		if (gsi_ctx->per.ver >= GSI_VER_3_0) {
			if (ctx->evtr->props.intr != GSI_INTR_MSI) {
				__gsi_config_ieob_irq_k(gsi_ctx->per.ee,
				gsihal_get_ch_reg_idx(ctx->evtr->id),
				gsihal_get_ch_reg_mask(ctx->evtr->id),
				~0);
			}
		}
		else {
			__gsi_config_ieob_irq(gsi_ctx->per.ee, 1 << ctx->evtr->id, ~0);
		}
		GSIDBG("set gsi_ctx evtr_id %d to %d mode\n",
			ctx->evtr->id, mode);

		/*
		 * In GSI 2.2 and 2.5 there is a limitation that can lead
		 * to losing an interrupt. For these versions an
		 * explicit check is needed after enabling the interrupt
		 */
		if ((gsi_ctx->per.ver == GSI_VER_2_2 ||
		    gsi_ctx->per.ver == GSI_VER_2_5) &&
			!gsi_ctx->per.skip_ieob_mask_wa) {
			u32 src = gsihal_read_reg_n(
				GSI_EE_n_CNTXT_SRC_IEOB_IRQ,
				gsi_ctx->per.ee);
			if (src & (1 << ctx->evtr->id)) {
				if (gsi_ctx->per.ver >= GSI_VER_3_0) {
					__gsi_config_ieob_irq_k(gsi_ctx->per.ee,
						gsihal_get_ch_reg_idx(ctx->evtr->id),
						gsihal_get_ch_reg_mask(ctx->evtr->id),
						0);
					gsihal_write_reg_nk(
						GSI_EE_n_CNTXT_SRC_IEOB_IRQ_CLR_k,
						gsi_ctx->per.ee,
						gsihal_get_ch_reg_idx(ctx->evtr->id),
						gsihal_get_ch_reg_mask(ctx->evtr->id));
				}
				else {
					__gsi_config_ieob_irq(gsi_ctx->per.ee, 1 << 
						ctx->evtr->id, 0);
					gsihal_write_reg_n(
						GSI_EE_n_CNTXT_SRC_IEOB_IRQ_CLR,
						gsi_ctx->per.ee,
						1 << ctx->evtr->id);
				}
				spin_unlock_irqrestore(&gsi_ctx->slock, flags);
				spin_lock_irqsave(&ctx->evtr->ring.slock,
									flags);
				chan_mode = atomic_xchg(&ctx->poll_mode,
						GSI_CHAN_MODE_POLL);
				spin_unlock_irqrestore(
					&ctx->evtr->ring.slock, flags);
				ctx->stats.poll_pending_irq++;
				GSIDBG("IEOB WA pnd cnt = %ld prvmode = %d\n",
						ctx->stats.poll_pending_irq,
						chan_mode);
				if (chan_mode == GSI_CHAN_MODE_POLL)
					return GSI_STATUS_SUCCESS;
				else
					return -GSI_STATUS_PENDING_IRQ;
			}
		}
		ctx->stats.poll_to_callback++;
	}
	spin_unlock_irqrestore(&gsi_ctx->slock, flags);
	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_config_channel_mode);

int gsi_get_channel_cfg(unsigned long chan_hdl, struct gsi_chan_props *props,
		union gsi_channel_scratch *scr)
{
	struct gsi_chan_ctx *ctx;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (!props || !scr) {
		GSIERR("bad params props=%pK scr=%pK\n", props, scr);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (chan_hdl >= gsi_ctx->max_ch) {
		GSIERR("bad params chan_hdl=%lu\n", chan_hdl);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	ctx = &gsi_ctx->chan[chan_hdl];

	if (ctx->state == GSI_CHAN_STATE_NOT_ALLOCATED) {
		GSIERR("bad state %d\n", ctx->state);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	mutex_lock(&ctx->mlock);
	*props = ctx->props;
	*scr = ctx->scratch;
	mutex_unlock(&ctx->mlock);

	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_get_channel_cfg);

int gsi_set_channel_cfg(unsigned long chan_hdl, struct gsi_chan_props *props,
		union gsi_channel_scratch *scr)
{
	struct gsi_chan_ctx *ctx;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (!props || gsi_validate_channel_props(props)) {
		GSIERR("bad params props=%pK\n", props);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (chan_hdl >= gsi_ctx->max_ch) {
		GSIERR("bad params chan_hdl=%lu\n", chan_hdl);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	ctx = &gsi_ctx->chan[chan_hdl];

	if (ctx->state != GSI_CHAN_STATE_ALLOCATED) {
		GSIERR("bad state %d\n", ctx->state);
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	if (ctx->props.ch_id != props->ch_id ||
		ctx->props.evt_ring_hdl != props->evt_ring_hdl) {
		GSIERR("changing immutable fields not supported\n");
		return -GSI_STATUS_UNSUPPORTED_OP;
	}

	mutex_lock(&ctx->mlock);
	ctx->props = *props;
	if (scr)
		ctx->scratch = *scr;
	gsi_program_chan_ctx(&ctx->props, gsi_ctx->per.ee,
			ctx->evtr ? ctx->evtr->id : GSI_NO_EVT_ERINDEX);
	gsi_init_chan_ring(&ctx->props, &ctx->ring);

	/* restore scratch */
	__gsi_write_channel_scratch(chan_hdl, ctx->scratch);
	mutex_unlock(&ctx->mlock);

	return GSI_STATUS_SUCCESS;
}
EXPORT_SYMBOL(gsi_set_channel_cfg);

static void gsi_configure_ieps(enum gsi_ver ver)
{
	gsihal_write_reg(GSI_GSI_IRAM_PTR_CH_CMD, 1);
	gsihal_write_reg(GSI_GSI_IRAM_PTR_CH_DB, 2);
	gsihal_write_reg(GSI_GSI_IRAM_PTR_CH_DIS_COMP, 3);
	gsihal_write_reg(GSI_GSI_IRAM_PTR_CH_EMPTY, 4);
	gsihal_write_reg(GSI_GSI_IRAM_PTR_EE_GENERIC_CMD, 5);
	gsihal_write_reg(GSI_GSI_IRAM_PTR_EVENT_GEN_COMP, 6);
	gsihal_write_reg(GSI_GSI_IRAM_PTR_INT_MOD_STOPPED, 7);
	gsihal_write_reg(GSI_GSI_IRAM_PTR_PERIPH_IF_TLV_IN_0, 8);
	gsihal_write_reg(GSI_GSI_IRAM_PTR_PERIPH_IF_TLV_IN_2, 9);
	gsihal_write_reg(GSI_GSI_IRAM_PTR_PERIPH_IF_TLV_IN_1, 10);
	gsihal_write_reg(GSI_GSI_IRAM_PTR_NEW_RE, 11);
	gsihal_write_reg(GSI_GSI_IRAM_PTR_READ_ENG_COMP, 12);
	gsihal_write_reg(GSI_GSI_IRAM_PTR_TIMER_EXPIRED, 13);
	gsihal_write_reg(GSI_GSI_IRAM_PTR_EV_DB, 14);
	gsihal_write_reg(GSI_GSI_IRAM_PTR_UC_GP_INT, 15);
	gsihal_write_reg(GSI_GSI_IRAM_PTR_WRITE_ENG_COMP, 16);

	if (ver >= GSI_VER_2_5)
		gsihal_write_reg(
			GSI_GSI_IRAM_PTR_TLV_CH_NOT_FULL,
			17);

	if (ver >= GSI_VER_2_11)
		gsihal_write_reg(
			GSI_GSI_IRAM_PTR_MSI_DB,
			18);
	if (ver >= GSI_VER_3_0)
		gsihal_write_reg(
			GSI_GSI_IRAM_PTR_INT_NOTIFY_MCS,
			19);
}

static void gsi_configure_bck_prs_matrix(void)
{
	/*
	 * For now, these are default values. In the future, GSI FW image will
	 * produce optimized back-pressure values based on the FW image.
	 */
	gsihal_write_reg(GSI_IC_DISABLE_CHNL_BCK_PRS_LSB, 0xfffffffe);
	gsihal_write_reg(GSI_IC_DISABLE_CHNL_BCK_PRS_MSB, 0xffffffff);
	gsihal_write_reg(GSI_IC_GEN_EVNT_BCK_PRS_LSB, 0xffffffbf);
	gsihal_write_reg(GSI_IC_GEN_EVNT_BCK_PRS_MSB, 0xffffffff);
	gsihal_write_reg(GSI_IC_GEN_INT_BCK_PRS_LSB, 0xffffefff);
	gsihal_write_reg(GSI_IC_GEN_INT_BCK_PRS_MSB, 0xffffffff);
	gsihal_write_reg(GSI_IC_STOP_INT_MOD_BCK_PRS_LSB, 0xffffefff);
	gsihal_write_reg(GSI_IC_STOP_INT_MOD_BCK_PRS_MSB, 0xffffffff);
	gsihal_write_reg(GSI_IC_PROCESS_DESC_BCK_PRS_LSB, 0x00000000);
	gsihal_write_reg(GSI_IC_PROCESS_DESC_BCK_PRS_MSB, 0x00000000);
	gsihal_write_reg(GSI_IC_TLV_STOP_BCK_PRS_LSB, 0xf9ffffff);
	gsihal_write_reg(GSI_IC_TLV_STOP_BCK_PRS_MSB, 0xffffffff);
	gsihal_write_reg(GSI_IC_TLV_RESET_BCK_PRS_LSB, 0xf9ffffff);
	gsihal_write_reg(GSI_IC_TLV_RESET_BCK_PRS_MSB, 0xffffffff);
	gsihal_write_reg(GSI_IC_RGSTR_TIMER_BCK_PRS_LSB, 0xffffffff);
	gsihal_write_reg(GSI_IC_RGSTR_TIMER_BCK_PRS_MSB, 0xfffffffe);
	gsihal_write_reg(GSI_IC_READ_BCK_PRS_LSB, 0xffffffff);
	gsihal_write_reg(GSI_IC_READ_BCK_PRS_MSB, 0xffffefff);
	gsihal_write_reg(GSI_IC_WRITE_BCK_PRS_LSB, 0xffffffff);
	gsihal_write_reg(GSI_IC_WRITE_BCK_PRS_MSB, 0xffffdfff);
	gsihal_write_reg(GSI_IC_UCONTROLLER_GPR_BCK_PRS_LSB, 0xffffffff);
	gsihal_write_reg(GSI_IC_UCONTROLLER_GPR_BCK_PRS_MSB, 0xff03ffff);
}

int gsi_configure_regs(phys_addr_t per_base_addr, enum gsi_ver ver)
{
	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (!gsi_ctx->base) {
		GSIERR("access to GSI HW has not been mapped\n");
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (ver <= GSI_VER_ERR || ver >= GSI_VER_MAX) {
		GSIERR("Incorrect version %d\n", ver);
		return -GSI_STATUS_ERROR;
	}

	gsihal_write_reg(GSI_GSI_PERIPH_BASE_ADDR_MSB, 0);
	gsihal_write_reg(GSI_GSI_PERIPH_BASE_ADDR_LSB, per_base_addr);
	gsi_configure_bck_prs_matrix();
	gsi_configure_ieps(ver);

	return 0;
}
EXPORT_SYMBOL(gsi_configure_regs);

int gsi_enable_fw(phys_addr_t gsi_base_addr, u32 gsi_size, enum gsi_ver ver)
{
	struct gsihal_reg_gsi_cfg gsi_cfg;

	if (ver <= GSI_VER_ERR || ver >= GSI_VER_MAX) {
		GSIERR("Incorrect version %d\n", ver);
		return -GSI_STATUS_ERROR;
	}

	/* Enable the MCS and set to x2 clocks */
	gsi_cfg.gsi_enable = 1;
	gsi_cfg.double_mcs_clk_freq = 1;
	gsi_cfg.uc_is_mcs = 0;
	gsi_cfg.gsi_pwr_clps = 0;
	gsi_cfg.bp_mtrix_disable = 0;
	if (ver >= GSI_VER_1_2) {
		gsihal_write_reg(GSI_GSI_MCS_CFG, 1);

		gsi_cfg.mcs_enable = 0;

	} else {
		gsi_cfg.mcs_enable = 1;
	}

	/* GSI frequency is peripheral frequency divided by 3 (2+1) */
	if (ver >= GSI_VER_2_5)
		gsi_cfg.sleep_clk_div = 2;
	gsihal_write_reg_fields(GSI_GSI_CFG, &gsi_cfg);

	return 0;

}
EXPORT_SYMBOL(gsi_enable_fw);

void gsi_get_inst_ram_offset_and_size(unsigned long *base_offset,
		unsigned long *size, enum gsi_ver ver)
{
	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return;
	}

	if (size)
		*size = gsihal_get_inst_ram_size();

	if (base_offset) {
		*base_offset = gsihal_get_reg_n_ofst(GSI_GSI_INST_RAM_n, 0);
	}
}
EXPORT_SYMBOL(gsi_get_inst_ram_offset_and_size);

/*
 * Dumping the Debug registers for halt issue debugging.
 */
static void gsi_dump_halt_debug_reg(unsigned int chan_idx, unsigned int ee)
{
	struct gsihal_reg_ch_k_cntxt_0 ch_k_cntxt_0;

	GSIERR("DEBUG_PC_FOR_DEBUG = 0x%x\n",
		gsihal_read_reg(GSI_EE_n_GSI_DEBUG_PC_FOR_DEBUG));

	GSIERR("GSI_DEBUG_BUSY_REG 0x%x\n",
		gsihal_read_reg(GSI_EE_n_GSI_DEBUG_BUSY_REG));

	GSIERR("GSI_EE_n_CNTXT_GLOB_IRQ_EN_OFFS = 0x%x\n",
			gsihal_read_reg_n(GSI_EE_n_CNTXT_GLOB_IRQ_EN, gsi_ctx->per.ee));

	GSIERR("GSI_EE_n_CNTXT_GLOB_IRQ_STTS_OFFS IRQ type = 0x%x\n",
		gsihal_read_reg_n(GSI_EE_n_CNTXT_GLOB_IRQ_EN, gsi_ctx->per.ee));

	GSIERR("GSI_EE_n_CNTXT_SCRATCH_0_OFFS = 0x%x\n",
		 gsihal_read_reg_n(GSI_EE_n_CNTXT_SCRATCH_0, gsi_ctx->per.ee));
	if (gsi_ctx->per.ver >= GSI_VER_2_9)
		GSIERR("GSI_EE_n_GSI_CH_k_SCRATCH_4 = 0x%x\n",
			gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_4, ee, chan_idx));

	gsihal_read_reg_nk_fields(GSI_EE_n_GSI_CH_k_CNTXT_0, ee, chan_idx, &ch_k_cntxt_0);
	GSIERR("Q6 channel [%d] state =  %d\n", chan_idx, ch_k_cntxt_0.chstate);
}

int gsi_halt_channel_ee(unsigned int chan_idx, unsigned int ee, int *code)
{
	enum gsi_generic_ee_cmd_opcode op = GSI_GEN_EE_CMD_HALT_CHANNEL;
	struct gsihal_reg_gsi_ee_generic_cmd cmd;
	int res;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (chan_idx >= gsi_ctx->max_ch || !code) {
		GSIERR("bad params chan_idx=%d\n", chan_idx);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	mutex_lock(&gsi_ctx->mlock);
	__gsi_config_glob_irq(gsi_ctx->per.ee,
			gsihal_get_glob_irq_en_gp_int1_mask(), ~0);
	reinit_completion(&gsi_ctx->gen_ee_cmd_compl);

	/* invalidate the response */
	gsi_ctx->scratch.word0.val = gsihal_read_reg_n(
		GSI_EE_n_CNTXT_SCRATCH_0, gsi_ctx->per.ee);
	gsi_ctx->scratch.word0.s.generic_ee_cmd_return_code = 0;
	gsihal_write_reg_n(GSI_EE_n_CNTXT_SCRATCH_0,
		gsi_ctx->per.ee, gsi_ctx->scratch.word0.val);

	gsi_ctx->gen_ee_cmd_dbg.halt_channel++;
	cmd.opcode = op;
	cmd.virt_chan_idx = chan_idx;
	cmd.ee = ee;
	gsihal_write_reg_n_fields(GSI_EE_n_GSI_EE_GENERIC_CMD, gsi_ctx->per.ee, &cmd);
	res = wait_for_completion_timeout(&gsi_ctx->gen_ee_cmd_compl,
		msecs_to_jiffies(GSI_CMD_TIMEOUT));
	if (res == 0) {
		GSIERR("chan_idx=%u ee=%u timed out\n", chan_idx, ee);
		res = -GSI_STATUS_TIMED_OUT;
		goto free_lock;
	}

	gsi_ctx->scratch.word0.val = gsihal_read_reg_n(GSI_EE_n_CNTXT_SCRATCH_0,
		gsi_ctx->per.ee);
	if (gsi_ctx->scratch.word0.s.generic_ee_cmd_return_code ==
		GSI_GEN_EE_CMD_RETURN_CODE_RETRY) {
		GSIDBG("chan_idx=%u ee=%u busy try again\n", chan_idx, ee);
		*code = GSI_GEN_EE_CMD_RETURN_CODE_RETRY;
		res = -GSI_STATUS_AGAIN;
		goto free_lock;
	}
	if (gsi_ctx->scratch.word0.s.generic_ee_cmd_return_code == 0) {
		GSIERR("No response received\n");
		gsi_dump_halt_debug_reg(chan_idx, ee);
		usleep_range(GSI_RESET_WA_MIN_SLEEP, GSI_RESET_WA_MAX_SLEEP);
		GSIERR("Reading after usleep scratch 0 reg\n");
		gsi_ctx->scratch.word0.val = gsihal_read_reg_n(GSI_EE_n_CNTXT_SCRATCH_0,
				 gsi_ctx->per.ee);
		if (gsi_ctx->scratch.word0.s.generic_ee_cmd_return_code == 0) {
			GSIERR("No response received second attempt\n");
			gsi_dump_halt_debug_reg(chan_idx, ee);
			res = -GSI_STATUS_ERROR;
			goto free_lock;
		}
	}

	res = GSI_STATUS_SUCCESS;
	*code = gsi_ctx->scratch.word0.s.generic_ee_cmd_return_code;
free_lock:
	__gsi_config_glob_irq(gsi_ctx->per.ee,
		gsihal_get_glob_irq_en_gp_int1_mask(), 0);
	mutex_unlock(&gsi_ctx->mlock);

	return res;
}
EXPORT_SYMBOL(gsi_halt_channel_ee);

int gsi_alloc_channel_ee(unsigned int chan_idx, unsigned int ee, int *code)
{
	enum gsi_generic_ee_cmd_opcode op = GSI_GEN_EE_CMD_ALLOC_CHANNEL;
	struct gsi_chan_ctx *ctx;
	struct gsihal_reg_gsi_ee_generic_cmd cmd;
	int res;

	if (chan_idx >= gsi_ctx->max_ch || !code) {
		GSIERR("bad params chan_idx=%d\n", chan_idx);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	if (ee == 0)
		return gsi_alloc_ap_channel(chan_idx);

	mutex_lock(&gsi_ctx->mlock);
	__gsi_config_glob_irq(gsi_ctx->per.ee,
			gsihal_get_glob_irq_en_gp_int1_mask(), ~0);
	reinit_completion(&gsi_ctx->gen_ee_cmd_compl);

	/* invalidate the response */
	gsi_ctx->scratch.word0.val = gsihal_read_reg_n(GSI_EE_n_CNTXT_SCRATCH_0,
		gsi_ctx->per.ee);
	gsi_ctx->scratch.word0.s.generic_ee_cmd_return_code = 0;
	gsihal_write_reg_n(GSI_EE_n_CNTXT_SCRATCH_0,
		gsi_ctx->per.ee, gsi_ctx->scratch.word0.val);

	cmd.opcode = op;
	cmd.virt_chan_idx = chan_idx;
	cmd.ee = ee;
	gsihal_write_reg_n_fields(
		GSI_EE_n_GSI_EE_GENERIC_CMD, gsi_ctx->per.ee, &cmd);
	res = wait_for_completion_timeout(&gsi_ctx->gen_ee_cmd_compl,
		msecs_to_jiffies(GSI_CMD_TIMEOUT));
	if (res == 0) {
		GSIERR("chan_idx=%u ee=%u timed out\n", chan_idx, ee);
		res = -GSI_STATUS_TIMED_OUT;
		goto free_lock;
	}

	gsi_ctx->scratch.word0.val = gsihal_read_reg_n(GSI_EE_n_CNTXT_SCRATCH_0,
		gsi_ctx->per.ee);
	if (gsi_ctx->scratch.word0.s.generic_ee_cmd_return_code ==
		GSI_GEN_EE_CMD_RETURN_CODE_OUT_OF_RESOURCES) {
		GSIDBG("chan_idx=%u ee=%u out of resources\n", chan_idx, ee);
		*code = GSI_GEN_EE_CMD_RETURN_CODE_OUT_OF_RESOURCES;
		res = -GSI_STATUS_RES_ALLOC_FAILURE;
		goto free_lock;
	}
	if (gsi_ctx->scratch.word0.s.generic_ee_cmd_return_code == 0) {
		GSIERR("No response received\n");
		res = -GSI_STATUS_ERROR;
		goto free_lock;
	}
	if (ee == 0) {
		ctx = &gsi_ctx->chan[chan_idx];
		gsi_ctx->ch_dbg[chan_idx].ch_allocate++;
	}
	res = GSI_STATUS_SUCCESS;
	*code = gsi_ctx->scratch.word0.s.generic_ee_cmd_return_code;
free_lock:
	__gsi_config_glob_irq(gsi_ctx->per.ee,
		gsihal_get_glob_irq_en_gp_int1_mask(), 0);
	mutex_unlock(&gsi_ctx->mlock);

	return res;
}
EXPORT_SYMBOL(gsi_alloc_channel_ee);

int gsi_enable_flow_control_ee(unsigned int chan_idx, unsigned int ee,
	int *code)
{
	enum gsi_generic_ee_cmd_opcode op = GSI_GEN_EE_CMD_ENABLE_FLOW_CHANNEL;
	struct gsihal_reg_ch_k_cntxt_0 ch_k_cntxt_0;
	struct gsihal_reg_gsi_ee_generic_cmd cmd;
	enum gsi_chan_state curr_state = GSI_CHAN_STATE_NOT_ALLOCATED;
	int res;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (chan_idx >= gsi_ctx->max_ch || !code) {
		GSIERR("bad params chan_idx=%d\n", chan_idx);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	mutex_lock(&gsi_ctx->mlock);
	__gsi_config_glob_irq(gsi_ctx->per.ee,
			gsihal_get_glob_irq_en_gp_int1_mask(), ~0);
	reinit_completion(&gsi_ctx->gen_ee_cmd_compl);

	/* invalidate the response */
	gsi_ctx->scratch.word0.val = gsihal_read_reg_n(GSI_EE_n_CNTXT_SCRATCH_0,
		gsi_ctx->per.ee);
	gsi_ctx->scratch.word0.s.generic_ee_cmd_return_code = 0;
	gsihal_write_reg_n(GSI_EE_n_CNTXT_SCRATCH_0,
		gsi_ctx->per.ee, gsi_ctx->scratch.word0.val);

	gsi_ctx->gen_ee_cmd_dbg.flow_ctrl_channel++;
	cmd.opcode = op;
	cmd.virt_chan_idx = chan_idx;
	cmd.ee = ee;
	gsihal_write_reg_n_fields(
		GSI_EE_n_GSI_EE_GENERIC_CMD, gsi_ctx->per.ee, &cmd);

	res = wait_for_completion_timeout(&gsi_ctx->gen_ee_cmd_compl,
		msecs_to_jiffies(GSI_CMD_TIMEOUT));
	if (res == 0) {
		GSIERR("chan_idx=%u ee=%u timed out\n", chan_idx, ee);
		res = -GSI_STATUS_TIMED_OUT;
		goto free_lock;
	}

	gsi_ctx->scratch.word0.val = gsihal_read_reg_n(GSI_EE_n_CNTXT_SCRATCH_0,
		gsi_ctx->per.ee);
	if (gsi_ctx->scratch.word0.s.generic_ee_cmd_return_code ==
		GSI_GEN_EE_CMD_RETURN_CODE_CHANNEL_NOT_RUNNING) {
		GSIDBG("chan_idx=%u ee=%u not in correct state\n",
			chan_idx, ee);
		*code = GSI_GEN_EE_CMD_RETURN_CODE_CHANNEL_NOT_RUNNING;
		res = -GSI_STATUS_RES_ALLOC_FAILURE;
		goto free_lock;
	} else if (gsi_ctx->scratch.word0.s.generic_ee_cmd_return_code ==
		GSI_GEN_EE_CMD_RETURN_CODE_INCORRECT_CHANNEL_TYPE ||
		gsi_ctx->scratch.word0.s.generic_ee_cmd_return_code ==
		GSI_GEN_EE_CMD_RETURN_CODE_INCORRECT_CHANNEL_INDEX) {
		GSIERR("chan_idx=%u ee=%u not in correct state\n",
			chan_idx, ee);
		GSI_ASSERT();
	}
	if (gsi_ctx->scratch.word0.s.generic_ee_cmd_return_code == 0) {
		GSIERR("No response received\n");
		res = -GSI_STATUS_ERROR;
		goto free_lock;
	}

	/*Reading current channel state*/
	gsihal_read_reg_nk_fields(GSI_EE_n_GSI_CH_k_CNTXT_0,
		gsi_ctx->per.ee, chan_idx, &ch_k_cntxt_0);
	curr_state = ch_k_cntxt_0.chstate;
	if (curr_state == GSI_CHAN_STATE_FLOW_CONTROL) {
		GSIDBG("ch %u state updated to %u\n", chan_idx, curr_state);
		res = GSI_STATUS_SUCCESS;
	} else {
		GSIERR("ch %u state updated to %u incorrect state\n",
			chan_idx, curr_state);
		res = -GSI_STATUS_ERROR;
	}
	*code = gsi_ctx->scratch.word0.s.generic_ee_cmd_return_code;
free_lock:
	__gsi_config_glob_irq(gsi_ctx->per.ee,
		gsihal_get_glob_irq_en_gp_int1_mask(), 0);
	mutex_unlock(&gsi_ctx->mlock);

	return res;
}
EXPORT_SYMBOL(gsi_enable_flow_control_ee);

int gsi_flow_control_ee(unsigned int chan_idx, int ep_id, unsigned int ee,
				bool enable, bool prmy_scnd_fc, int *code)
{
	struct gsihal_reg_gsi_ee_generic_cmd cmd;
	enum gsi_generic_ee_cmd_opcode op = enable ?
					GSI_GEN_EE_CMD_ENABLE_FLOW_CHANNEL :
					GSI_GEN_EE_CMD_DISABLE_FLOW_CHANNEL;
	int res;
	int wait_due_pending = 0;
	uint32_t fc_pending = 0;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (chan_idx >= gsi_ctx->max_ch || !code) {
		GSIERR("bad params chan_idx=%d\n", chan_idx);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	GSIDBG("GSI flow control opcode=%d, ch_id=%d\n", op, chan_idx);

	mutex_lock(&gsi_ctx->mlock);
	__gsi_config_glob_irq(gsi_ctx->per.ee,
			gsihal_get_glob_irq_en_gp_int1_mask(), ~0);
	reinit_completion(&gsi_ctx->gen_ee_cmd_compl);

	/* invalidate the response */
	gsi_ctx->scratch.word0.val = gsihal_read_reg_n(GSI_EE_n_CNTXT_SCRATCH_0,
                gsi_ctx->per.ee);

	gsi_ctx->scratch.word0.s.generic_ee_cmd_return_code = 0;
	gsihal_write_reg_n(GSI_EE_n_CNTXT_SCRATCH_0,
                gsi_ctx->per.ee, gsi_ctx->scratch.word0.val);

	gsi_ctx->gen_ee_cmd_dbg.flow_ctrl_channel++;
	cmd.opcode = op;
	cmd.virt_chan_idx = chan_idx;
	cmd.ee = ee;
	cmd.prmy_scnd_fc = prmy_scnd_fc;
	gsihal_write_reg_n_fields(
		GSI_EE_n_GSI_EE_GENERIC_CMD, gsi_ctx->per.ee, &cmd);

wait_again:
	fc_pending = gsihal_read_reg_n(GSI_GSI_SHRAM_n,
		(ep_id * GSI_FC_NUM_WORDS_PER_CHNL_SHRAM) + GSI_FC_STATE_INDEX_SHRAM) &
		GSI_FC_PENDING_MASK;
	res = wait_for_completion_timeout(&gsi_ctx->gen_ee_cmd_compl,
		msecs_to_jiffies(GSI_FC_CMD_TIMEOUT));
	if (res == 0) {
		GSIERR("chan_idx=%u ee=%u timed out\n", chan_idx, ee);
		if (op == GSI_GEN_EE_CMD_ENABLE_FLOW_CHANNEL &&
			wait_due_pending < GSI_FC_MAX_TIMEOUT &&
			fc_pending) {
			wait_due_pending++;
			goto wait_again;
		}
		GSIERR("GSI_EE_n_CNTXT_GLOB_IRQ_EN_OFFS = 0x%x\n",
			gsihal_read_reg_n(GSI_EE_n_CNTXT_GLOB_IRQ_EN, gsi_ctx->per.ee));
		GSIERR("GSI_EE_n_CNTXT_GLOB_IRQ_STTS_OFFS IRQ type = 0x%x\n",
			gsihal_read_reg_n(GSI_EE_n_CNTXT_GLOB_IRQ_STTS, gsi_ctx->per.ee));
	}

	gsi_ctx->scratch.word0.val = gsihal_read_reg_n(GSI_EE_n_CNTXT_SCRATCH_0,
					gsi_ctx->per.ee);

	GSIDBG(
		"Flow control command response GENERIC_CMD_RESPONSE_CODE = %u, val = %u\n",
		gsi_ctx->scratch.word0.s.generic_ee_cmd_return_code,
		gsi_ctx->scratch.word0.val);

	if (gsi_ctx->scratch.word0.s.generic_ee_cmd_return_code ==
		GSI_GEN_EE_CMD_RETURN_CODE_CHANNEL_NOT_RUNNING) {
		GSIDBG("chan_idx=%u ee=%u not in correct state\n",
							chan_idx, ee);
		*code = GSI_GEN_EE_CMD_RETURN_CODE_CHANNEL_NOT_RUNNING;
		res = -GSI_STATUS_RES_ALLOC_FAILURE;
		goto free_lock;
	} else if (gsi_ctx->scratch.word0.s.generic_ee_cmd_return_code ==
			GSI_GEN_EE_CMD_RETURN_CODE_INCORRECT_CHANNEL_TYPE) {
		GSIERR("chan_idx=%u ee=%u not in correct state\n",
				chan_idx, ee);
		GSI_ASSERT();
	} else if (gsi_ctx->scratch.word0.s.generic_ee_cmd_return_code ==
			GSI_GEN_EE_CMD_RETURN_CODE_INCORRECT_CHANNEL_INDEX) {
		GSIERR("Channel ID = %u ee = %u not allocated\n", chan_idx, ee);
	}

	if (gsi_ctx->scratch.word0.s.generic_ee_cmd_return_code == 0) {
		GSIERR("No response received\n");
		res = -GSI_STATUS_ERROR;
		GSI_ASSERT();
		goto free_lock;
	}

	*code = gsi_ctx->scratch.word0.s.generic_ee_cmd_return_code;
	res = GSI_STATUS_SUCCESS;
free_lock:
	__gsi_config_glob_irq(gsi_ctx->per.ee,
		gsihal_get_glob_irq_en_gp_int1_mask(), 0);
	mutex_unlock(&gsi_ctx->mlock);

	return res;
}
EXPORT_SYMBOL(gsi_flow_control_ee);

int gsi_query_flow_control_state_ee(unsigned int chan_idx, unsigned int ee,
						bool prmy_scnd_fc, int *code)
{
	struct gsihal_reg_gsi_ee_generic_cmd cmd;
	enum gsi_generic_ee_cmd_opcode op = GSI_GEN_EE_CMD_QUERY_FLOW_CHANNEL;
	int res;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (chan_idx >= gsi_ctx->max_ch || !code) {
		GSIERR("bad params chan_idx=%d\n", chan_idx);
		return -GSI_STATUS_INVALID_PARAMS;
	}

	mutex_lock(&gsi_ctx->mlock);
	__gsi_config_glob_irq(gsi_ctx->per.ee,
			gsihal_get_glob_irq_en_gp_int1_mask(), ~0);
	reinit_completion(&gsi_ctx->gen_ee_cmd_compl);

	/* invalidate the response */
	gsi_ctx->scratch.word0.val = gsihal_read_reg_n(GSI_EE_n_CNTXT_SCRATCH_0,
							gsi_ctx->per.ee);
	gsi_ctx->scratch.word0.s.generic_ee_cmd_return_code = 0;
	gsihal_write_reg_n(GSI_EE_n_CNTXT_SCRATCH_0,
                gsi_ctx->per.ee, gsi_ctx->scratch.word0.val);

	gsi_ctx->gen_ee_cmd_dbg.flow_ctrl_channel++;
	cmd.opcode = op;
	cmd.virt_chan_idx = chan_idx;
	cmd.ee = ee;
	cmd.prmy_scnd_fc = prmy_scnd_fc;
	gsihal_write_reg_n_fields(
			GSI_EE_n_GSI_EE_GENERIC_CMD, gsi_ctx->per.ee, &cmd);

	res = wait_for_completion_timeout(&gsi_ctx->gen_ee_cmd_compl,
		msecs_to_jiffies(GSI_CMD_TIMEOUT));
	if (res == 0) {
		GSIERR("chan_idx=%u ee=%u timed out\n", chan_idx, ee);
		res = -GSI_STATUS_TIMED_OUT;
		goto free_lock;
	}

	gsi_ctx->scratch.word0.val = gsihal_read_reg_n(GSI_EE_n_CNTXT_SCRATCH_0,
					gsi_ctx->per.ee);

	*code = gsi_ctx->scratch.word0.s.generic_ee_cmd_return_val;

	if (prmy_scnd_fc)
		res = (gsi_ctx->scratch.word0.s.generic_ee_cmd_return_val ==
		GSI_GEN_EE_CMD_RETURN_VAL_FLOW_CONTROL_SECONDARY)?
				GSI_STATUS_SUCCESS:-GSI_STATUS_ERROR;
	else
		res = (gsi_ctx->scratch.word0.s.generic_ee_cmd_return_val ==
		GSI_GEN_EE_CMD_RETURN_VAL_FLOW_CONTROL_PRIMARY)?
				GSI_STATUS_SUCCESS:-GSI_STATUS_ERROR;

free_lock:
	__gsi_config_glob_irq(gsi_ctx->per.ee,
		gsihal_get_glob_irq_en_gp_int1_mask(), 0);
	mutex_unlock(&gsi_ctx->mlock);

	return res;
}
EXPORT_SYMBOL(gsi_query_flow_control_state_ee);


int gsi_map_virtual_ch_to_per_ep(u32 ee, u32 chan_num, u32 per_ep_index)
{
	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return -GSI_STATUS_NODEV;
	}

	if (!gsi_ctx->base) {
		GSIERR("access to GSI HW has not been mapped\n");
		return -GSI_STATUS_INVALID_PARAMS;
	}

	gsihal_write_reg_nk(GSI_MAP_EE_n_CH_k_VP_TABLE,
		ee, chan_num, per_ep_index);

	return 0;
}
EXPORT_SYMBOL(gsi_map_virtual_ch_to_per_ep);

void gsi_wdi3_write_evt_ring_db(unsigned long evt_ring_hdl,
	uint32_t db_addr_low, uint32_t db_addr_high)
{
	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return;
	}

	if (gsi_ctx->per.ver >= GSI_VER_2_9) {
		gsihal_write_reg_nk(GSI_EE_n_EV_CH_k_CNTXT_10,
			gsi_ctx->per.ee, evt_ring_hdl, db_addr_low);

		gsihal_write_reg_nk(GSI_EE_n_EV_CH_k_CNTXT_11,
			gsi_ctx->per.ee, evt_ring_hdl, db_addr_high);
	} else {
		gsihal_write_reg_nk(GSI_EE_n_EV_CH_k_CNTXT_12,
			gsi_ctx->per.ee, evt_ring_hdl, db_addr_low);

		gsihal_write_reg_nk(GSI_EE_n_EV_CH_k_CNTXT_13,
			gsi_ctx->per.ee, evt_ring_hdl, db_addr_high);
	}
}
EXPORT_SYMBOL(gsi_wdi3_write_evt_ring_db);

int gsi_get_refetch_reg(unsigned long chan_hdl, bool is_rp)
{
	if (is_rp) {
		return gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_RE_FETCH_READ_PTR,
			gsi_ctx->per.ee, chan_hdl);
	} else {
		return gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_RE_FETCH_WRITE_PTR,
			gsi_ctx->per.ee, chan_hdl);
	}
}
EXPORT_SYMBOL(gsi_get_refetch_reg);

/*
 * ; +------------------------------------------------------+
 * ; | NTN3 Rx Channel Scratch                              |
 * ; +-------------+--------------------------------+-------+
 * ; | 32-bit word | Field                          | Bits  |
 * ; +-------------+--------------------------------+-------+
 * ; | 4           | NTN_PENDING_DB_AFTER_ROLLBACK  | 18-18 |
 * ; +-------------+--------------------------------+-------+
 * ; | 5           | NTN_MSI_DB_INDEX_VALUE         | 0-31  |
 * ; +-------------+--------------------------------+-------+
 * ; | 6           | NTN_RX_CHAIN_COUNTER           | 0-31  |
 * ; +-------------+--------------------------------+-------+
 * ; | 7           | NTN_RX_ERR_COUNTER             | 0-31  |
 * ; +-------------+--------------------------------+-------+
 * ; | 8           | NTN_ACCUMULATED_TRES_HANDLED   | 0-31  |
 * ; +-------------+--------------------------------+-------+
 * ; | 9           | NTN_ROLLBACKS_COUNTER          | 0-31  |
 * ; +-------------+--------------------------------+-------+
 * ; | FOR_SEQ_HIGH| NTN_MSI_DB_COUNT               | 0-31  |
 * ; +-------------+--------------------------------+-------+
 *
 * ; +------------------------------------------------------+
 * ; | NTN3 Tx Channel Scratch                              |
 * ; +-------------+--------------------------------+-------+
 * ; | 32-bit word | Field                          | Bits  |
 * ; +-------------+--------------------------------+-------+
 * ; | 4           | NTN_PENDING_DB_AFTER_ROLLBACK  | 18-18 |
 * ; +-------------+--------------------------------+-------+
 * ; | 5           | NTN_MSI_DB_INDEX_VALUE         | 0-31  |
 * ; +-------------+--------------------------------+-------+
 * ; | 6           | TX_DERR_COUNTER                | 0-31  |
 * ; +-------------+--------------------------------+-------+
 * ; | 7           | NTN_TX_OOB_COUNTER             | 0-31  |
 * ; +-------------+--------------------------------+-------+
 * ; | 8           | NTN_ACCUMULATED_TRES_HANDLED   | 0-31  |
 * ; +-------------+--------------------------------+-------+
 * ; | 9           | NTN_ROLLBACKS_COUNTER          | 0-31  |
 * ; +-------------+--------------------------------+-------+
 * ; | FOR_SEQ_HIGH| NTN_MSI_DB_COUNT               | 0-31  |
 * ; +-------------+--------------------------------+-------+
 */
int gsi_ntn3_client_stats_get(unsigned ep_id, int scratch_id, unsigned chan_hdl)
{
	switch (scratch_id) {
	case -1:
		return gsihal_read_reg_n(GSI_GSI_SHRAM_n, GSI_GSI_SHRAM_n_EP_FOR_SEQ_HIGH_N_GET(ep_id));
	case 4:
		return (gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_4, gsi_ctx->per.ee,
			chan_hdl) >> GSI_NTN3_PENDING_DB_AFTER_RB_MASK) &
			GSI_NTN3_PENDING_DB_AFTER_RB_SHIFT;
		break;
	case 5:
		return gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_5, gsi_ctx->per.ee, chan_hdl);
		break;
	case 6:
		return gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_6, gsi_ctx->per.ee, chan_hdl);
		break;
	case 7:
		return gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_7, gsi_ctx->per.ee, chan_hdl);
		break;
	case 8:
		return gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_8, gsi_ctx->per.ee, chan_hdl);
		break;
	case 9:
		return gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_9, gsi_ctx->per.ee, chan_hdl);
		break;
	default:
		GSIERR("invalid scratch id %d\n", scratch_id);
		return 0;
	}
	return 0;
}
EXPORT_SYMBOL(gsi_ntn3_client_stats_get);

int gsi_get_drop_stats(unsigned long ep_id, int scratch_id,
	unsigned long chan_hdl)
{
#define GSI_RTK_ERR_STATS_MASK 0xFFFF
#define GSI_NTN_ERR_STATS_MASK 0xFFFFFFFF
#define GSI_AQC_RX_STATUS_MASK 0x1FFF
#define GSI_AQC_RX_STATUS_SHIFT 0
#define GSI_AQC_RDM_ERR_MASK 0x1FFF0000
#define GSI_AQC_RDM_ERR_SHIFT 16

	uint16_t rx_status;
	uint16_t rdm_err;
	uint32_t val;

	/* on newer versions we can read the ch scratch directly from reg */
	if (gsi_ctx->per.ver >= GSI_VER_3_0) {
		switch (scratch_id) {
		case 5:
			return gsihal_read_reg_nk(
				GSI_EE_n_GSI_CH_k_SCRATCH_5,
				gsi_ctx->per.ee,
				chan_hdl) & GSI_RTK_ERR_STATS_MASK;
			break;
		case 6:
			return gsihal_read_reg_nk(
				GSI_EE_n_GSI_CH_k_SCRATCH_6,
				gsi_ctx->per.ee,
				chan_hdl) & GSI_NTN_ERR_STATS_MASK;
			break;
		case 7:
			val = gsihal_read_reg_nk(
				GSI_EE_n_GSI_CH_k_SCRATCH_7,
				gsi_ctx->per.ee,
				chan_hdl);
			rx_status = (val & GSI_AQC_RX_STATUS_MASK)
				>> GSI_AQC_RX_STATUS_SHIFT;
			rdm_err = (val & GSI_AQC_RDM_ERR_MASK)
				>> (GSI_AQC_RDM_ERR_SHIFT);
			return rx_status + rdm_err;
			break;
		default:
			GSIERR("invalid scratch id %d\n", scratch_id);
			return 0;
		}

	/* on older versions we need to read the scratch from SHRAM */
	} else {
		/* RTK use scratch 5 */
		if (scratch_id == 5) {
			/*
			 * each channel context is 6 lines of 8 bytes, but n in
			 * SHRAM_n is in 4 bytes offsets, so multiplying ep_id
			 * by 6*2=12 will give the beginning of the required
			 * channel context, and then need to add 7 since the
			 * channel context layout has the ring rbase (8 bytes)
			 * + channel scratch 0-4 (20 bytes) so adding
			 * additional 28/4 = 7 to get to scratch 5 of the
			 * required channel.
			 */
			return gsihal_read_reg_n(
				GSI_GSI_SHRAM_n,
				ep_id * 12 + 7) & GSI_RTK_ERR_STATS_MASK;
		}
	}
	return 0;
}
EXPORT_SYMBOL(gsi_get_drop_stats);

int gsi_get_wp(unsigned long chan_hdl)
{
	return gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_6, gsi_ctx->per.ee,
		chan_hdl);
}
EXPORT_SYMBOL(gsi_get_wp);

void gsi_wdi3_dump_register(unsigned long chan_hdl)
{
	uint32_t val;

	if (!gsi_ctx) {
		pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
		return;
	}
	GSIDBG("reg dump ch id %ld\n", chan_hdl);
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_0,
		gsi_ctx->per.ee, chan_hdl);
	GSIDBG("GSI_EE_n_GSI_CH_k_CNTXT_0 0x%x\n", val);
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_1,
		gsi_ctx->per.ee, chan_hdl);
	GSIDBG("GSI_EE_n_GSI_CH_k_CNTXT_1 0x%x\n", val);
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_2,
		gsi_ctx->per.ee, chan_hdl);
	GSIDBG("GSI_EE_n_GSI_CH_k_CNTXT_2 0x%x\n", val);
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_3,
		gsi_ctx->per.ee, chan_hdl);
	GSIDBG("GSI_EE_n_GSI_CH_k_CNTXT_3 0x%x\n", val);
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_4,
		gsi_ctx->per.ee, chan_hdl);
	GSIDBG("GSI_EE_n_GSI_CH_k_CNTXT_4 0x%x\n", val);
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_5,
		gsi_ctx->per.ee, chan_hdl);
	GSIDBG("GSI_EE_n_GSI_CH_k_CNTXT_5 0x%x\n", val);
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_6,
		gsi_ctx->per.ee, chan_hdl);
	GSIDBG("GSI_EE_n_GSI_CH_k_CNTXT_6 0x%x\n", val);
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_7,
		gsi_ctx->per.ee, chan_hdl);
	GSIDBG("GSI_EE_n_GSI_CH_k_CNTXT_7 0x%x\n", val);

	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_RE_FETCH_READ_PTR,
		gsi_ctx->per.ee, chan_hdl);
	GSIDBG("GSI_EE_n_GSI_CH_k_RE_FETCH_READ_PTR 0x%x\n", val);
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_RE_FETCH_WRITE_PTR,
		gsi_ctx->per.ee, chan_hdl);
	GSIDBG("GSI_EE_n_GSI_CH_k_RE_FETCH_WRITE_PTR 0x%x\n", val);
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_QOS,
		gsi_ctx->per.ee, chan_hdl);
	GSIDBG("GSI_EE_n_GSI_CH_k_QOS 0x%x\n", val);
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_0,
		gsi_ctx->per.ee, chan_hdl);
	GSIDBG("GSI_EE_n_GSI_CH_k_SCRATCH_0 0x%x\n", val);
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_1,
		gsi_ctx->per.ee, chan_hdl);
	GSIDBG("GSI_EE_n_GSI_CH_k_SCRATCH_1 0x%x\n", val);
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_2,
		gsi_ctx->per.ee, chan_hdl);
	GSIDBG("GSI_EE_n_GSI_CH_k_SCRATCH_2 0x%x\n", val);
	val = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_3,
		gsi_ctx->per.ee, chan_hdl);
	GSIDBG("GSI_EE_n_GSI_CH_k_SCRATCH_3 0x%x\n", val);
}
EXPORT_SYMBOL(gsi_wdi3_dump_register);

int gsi_query_msi_addr(unsigned long chan_hdl, phys_addr_t *addr)
{
        if (!gsi_ctx) {
                pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
                return -GSI_STATUS_NODEV;
        }

        if (chan_hdl >= gsi_ctx->max_ch) {
                GSIERR("bad params chan_hdl=%lu\n", chan_hdl);
                return -GSI_STATUS_INVALID_PARAMS;
        }

        if (gsi_ctx->chan[chan_hdl].state == GSI_CHAN_STATE_NOT_ALLOCATED) {
                GSIERR("bad state %d\n",
                        gsi_ctx->chan[chan_hdl].state);
                return -GSI_STATUS_UNSUPPORTED_OP;
        }

        *addr = (phys_addr_t)(gsi_ctx->per.phys_addr +
                gsihal_get_reg_nk_ofst(GSI_EE_n_GSI_CH_k_CNTXT_8,
                        gsi_ctx->per.ee, chan_hdl));

        return 0;
}
EXPORT_SYMBOL(gsi_query_msi_addr);

int gsi_query_device_msi_addr(u64 *addr)
{
    if (!gsi_ctx) {
            pr_err("%s:%d gsi context not allocated\n", __func__, __LINE__);
            return -GSI_STATUS_NODEV;
    }

	if (gsi_ctx->msi_addr_set)
		*addr = gsi_ctx->msi_addr;
	else
		*addr = 0;

	GSIDBG("Device MSI Addr: 0x%llx", *addr);
    return 0;
}
EXPORT_SYMBOL(gsi_query_device_msi_addr);


uint64_t gsi_read_event_ring_wp(int evtr_id, int ee)
{
	uint64_t wp;

	wp = gsihal_read_reg_nk(GSI_EE_n_EV_CH_k_CNTXT_6,
			ee, evtr_id);
	wp |= ((uint64_t)gsihal_read_reg_nk(GSI_EE_n_EV_CH_k_CNTXT_7,
			ee, evtr_id)) << 32;

	return wp;
}
EXPORT_SYMBOL(gsi_read_event_ring_wp);

uint64_t gsi_read_event_ring_bp(int evt_hdl)
{
	return gsi_ctx->evtr[evt_hdl].ring.base;
}
EXPORT_SYMBOL(gsi_read_event_ring_bp);

uint64_t gsi_get_evt_ring_rp(int evt_hdl)
{
	return gsi_ctx->evtr[evt_hdl].props.gsi_read_event_ring_rp(
		&gsi_ctx->evtr[evt_hdl].props, evt_hdl, gsi_ctx->per.ee);
}
EXPORT_SYMBOL(gsi_get_evt_ring_rp);

uint64_t gsi_read_chan_ring_rp(int chan_id, int ee)
{
	uint64_t rp;

	rp = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_4,
			ee, chan_id);
	rp |= ((uint64_t)gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_5,
			ee, chan_id)) << 32;

	return rp;
}
EXPORT_SYMBOL(gsi_read_chan_ring_rp);

uint64_t gsi_read_chan_ring_wp(int chan_id, int ee)
{
	uint64_t wp;

	wp = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_6,
			ee, chan_id);
	wp |= ((uint64_t)gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_CNTXT_7,
			ee, chan_id)) << 32;

	return wp;
}
EXPORT_SYMBOL(gsi_read_chan_ring_wp);

uint64_t gsi_read_chan_ring_bp(int chan_hdl)
{
	return gsi_ctx->chan[chan_hdl].ring.base;
}
EXPORT_SYMBOL(gsi_read_chan_ring_bp);

uint64_t gsi_read_chan_ring_re_fetch_wp(int chan_id, int ee)
{
	uint64_t wp;

	wp = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_RE_FETCH_WRITE_PTR,
			ee, chan_id);

	return wp;
}
EXPORT_SYMBOL(gsi_read_chan_ring_re_fetch_wp);

enum gsi_chan_prot gsi_get_chan_prot_type(int chan_hdl)
{
	return gsi_ctx->chan[chan_hdl].props.prot;
}
EXPORT_SYMBOL(gsi_get_chan_prot_type);

enum gsi_chan_state gsi_get_chan_state(int chan_hdl)
{
	return gsi_ctx->chan[chan_hdl].state;
}
EXPORT_SYMBOL(gsi_get_chan_state);

int gsi_get_chan_poll_mode(int chan_hdl)
{
	return atomic_read(&gsi_ctx->chan[chan_hdl].poll_mode);
}
EXPORT_SYMBOL(gsi_get_chan_poll_mode);

uint32_t gsi_get_ring_len(int chan_hdl)
{
	return gsi_ctx->chan[chan_hdl].ring.len;
}
EXPORT_SYMBOL(gsi_get_ring_len);

uint8_t gsi_get_chan_props_db_in_bytes(int chan_hdl)
{
	return gsi_ctx->chan[chan_hdl].props.db_in_bytes;
}
EXPORT_SYMBOL(gsi_get_chan_props_db_in_bytes);

int gsi_get_peripheral_ee(void)
{
	return gsi_ctx->per.ee;
}
EXPORT_SYMBOL(gsi_get_peripheral_ee);

uint32_t gsi_get_chan_stop_stm(int chan_id, int ee)
{
	uint32_t ch_scratch;
	ch_scratch = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_4, ee, chan_id);
	/* Only bits 28 - 31 for STM */
	return ((ch_scratch & 0xF0000000) >> 24);
}
EXPORT_SYMBOL(gsi_get_chan_stop_stm);

enum gsi_evt_ring_elem_size gsi_get_evt_ring_re_size(int evt_hdl)
{
	return gsi_ctx->evtr[evt_hdl].props.re_size;
}
EXPORT_SYMBOL(gsi_get_evt_ring_re_size);

uint32_t gsi_get_evt_ring_len(int evt_hdl)
{
	return gsi_ctx->evtr[evt_hdl].ring.len;
}
EXPORT_SYMBOL(gsi_get_evt_ring_len);

void gsi_update_almst_empty_thrshold(unsigned long chan_hdl, unsigned short threshold)
{
	gsihal_write_reg_nk(GSI_EE_n_CH_k_CH_ALMST_EMPTY_THRSHOLD,
		gsi_ctx->per.ee, chan_hdl, threshold);
}
EXPORT_SYMBOL(gsi_update_almst_empty_thrshold);

static union __packed gsi_channel_scratch __gsi_update_mhi_channel_scratch(
	unsigned long chan_hdl, struct __packed gsi_mhi_channel_scratch mscr)
{
	union __packed gsi_channel_scratch scr;

	/* below sequence is not atomic. assumption is sequencer specific fields
	 * will remain unchanged across this sequence
	 */

	/* READ */
	scr.data.word1 = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_0,
		gsi_ctx->per.ee, chan_hdl);
	scr.data.word2 = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_1,
		gsi_ctx->per.ee, chan_hdl);
	scr.data.word3 = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_2,
		gsi_ctx->per.ee, chan_hdl);
	scr.data.word4 = gsihal_read_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_3,
		gsi_ctx->per.ee, chan_hdl);

	/* UPDATE */
	scr.mhi.polling_mode = mscr.polling_mode;

	if (gsi_ctx->per.ver < GSI_VER_2_5) {
		scr.mhi.max_outstanding_tre = mscr.max_outstanding_tre;
		scr.mhi.outstanding_threshold = mscr.outstanding_threshold;
	}

	/* WRITE */
	gsihal_write_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_0,
		gsi_ctx->per.ee, chan_hdl, scr.data.word1);
	gsihal_write_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_1,
		gsi_ctx->per.ee, chan_hdl, scr.data.word2);
	gsihal_write_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_2,
		gsi_ctx->per.ee, chan_hdl, scr.data.word3);
	gsihal_write_reg_nk(GSI_EE_n_GSI_CH_k_SCRATCH_3,
		gsi_ctx->per.ee, chan_hdl, scr.data.word4);

	return scr;
}
/**
 * gsi_get_hw_profiling_stats() - Query GSI HW profiling stats
 * @stats:	[out] stats blob from client populated by driver
 *
 * Returns:	0 on success, negative on failure
 *
 */
int gsi_get_hw_profiling_stats(struct gsi_hw_profiling_data *stats)
{
	if (stats == NULL) {
		GSIERR("bad parms NULL stats == NULL\n");
		return -EINVAL;
	}

	stats->bp_cnt = (u64)gsihal_read_reg(
						GSI_GSI_MCS_PROFILING_BP_CNT_LSB) +
						((u64)gsihal_read_reg(
						GSI_GSI_MCS_PROFILING_BP_CNT_MSB) << 32);
	stats->bp_and_pending_cnt = (u64)gsihal_read_reg(
						GSI_GSI_MCS_PROFILING_BP_AND_PENDING_CNT_LSB) +
						((u64)gsihal_read_reg(
						GSI_GSI_MCS_PROFILING_BP_AND_PENDING_CNT_MSB) << 32);
	stats->mcs_busy_cnt = (u64)gsihal_read_reg(
						GSI_GSI_MCS_PROFILING_MCS_BUSY_CNT_LSB) +
						((u64)gsihal_read_reg(
						GSI_GSI_MCS_PROFILING_MCS_BUSY_CNT_MSB) << 32);
	stats->mcs_idle_cnt = (u64)gsihal_read_reg(
						GSI_GSI_MCS_PROFILING_MCS_IDLE_CNT_LSB) +
						((u64)gsihal_read_reg(
						GSI_GSI_MCS_PROFILING_MCS_IDLE_CNT_MSB) << 32);

	return 0;
}

/**
 * gsi_get_fw_version() - Query GSI FW version
 * @ver:	[out] ver blob from client populated by driver
 *
 * Returns:	0 on success, negative on failure
 *
 */
int gsi_get_fw_version(struct gsi_fw_version *ver)
{
	u32 raw = 0;

	if (ver == NULL) {
		GSIERR("bad parms: ver == NULL\n");
		return -EINVAL;
	}

	if (gsi_ctx->per.ver < GSI_VER_3_0)
		raw = gsihal_read_reg_n(GSI_GSI_INST_RAM_n,
			GSI_INST_RAM_FW_VER_OFFSET);
	else
		raw = gsihal_read_reg_n(GSI_GSI_INST_RAM_n,
			GSI_INST_RAM_FW_VER_GSI_3_0_OFFSET);

	ver->hw = (raw & GSI_INST_RAM_FW_VER_HW_MASK) >>
				GSI_INST_RAM_FW_VER_HW_SHIFT;
	ver->flavor = (raw & GSI_INST_RAM_FW_VER_FLAVOR_MASK) >>
					GSI_INST_RAM_FW_VER_FLAVOR_SHIFT;
	ver->fw = (raw & GSI_INST_RAM_FW_VER_FW_MASK) >>
				GSI_INST_RAM_FW_VER_FW_SHIFT;

	return 0;
}

#if IS_ENABLED(CONFIG_QCOM_VA_MINIDUMP)
static int qcom_va_md_gsi_notif_handler(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	struct va_md_entry entry;

	strlcpy(entry.owner, "gsi_mini", sizeof(entry.owner));
	entry.vaddr = (unsigned long)gsi_ctx;
	entry.size = sizeof(struct gsi_ctx);
	qcom_va_md_add_region(&entry);
	return NOTIFY_OK;
}

static struct notifier_block qcom_va_md_gsi_notif_blk = {
        .notifier_call = qcom_va_md_gsi_notif_handler,
        .priority = INT_MAX,
};
#endif

static int msm_gsi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int result;

	pr_debug("gsi_probe\n");
	gsi_ctx = devm_kzalloc(dev, sizeof(*gsi_ctx), GFP_KERNEL);
	if (!gsi_ctx) {
		dev_err(dev, "failed to allocated gsi context\n");
		return -ENOMEM;
	}

	gsi_ctx->ipc_logbuf = ipc_log_context_create(GSI_IPC_LOG_PAGES,
		"gsi", MINIDUMP_MASK);
	if (gsi_ctx->ipc_logbuf == NULL)
		GSIERR("failed to create IPC log, continue...\n");

	result = of_property_read_u32(pdev->dev.of_node, "qcom,num-msi",
			&gsi_ctx->msi.num);
	if (result)
		GSIERR("No MSIs configured\n");
	else {
		if (gsi_ctx->msi.num > GSI_MAX_NUM_MSI) {
			GSIERR("Num MSIs %u larger than max %u, normalizing\n",
				gsi_ctx->msi.num,
				GSI_MAX_NUM_MSI);
			gsi_ctx->msi.num = GSI_MAX_NUM_MSI;
		} else GSIDBG("Num MSIs=%u\n", gsi_ctx->msi.num);
	}

	gsi_ctx->dev = dev;
	init_completion(&gsi_ctx->gen_ee_cmd_compl);
	gsi_debugfs_init();

#if IS_ENABLED(CONFIG_QCOM_VA_MINIDUMP)
	result = qcom_va_md_register("gsi_mini", &qcom_va_md_gsi_notif_blk);

	if(result)
		GSIERR("gsi mini qcom_va_md_register failed = %d\n", result);
	else
		GSIDBG("gsi mini qcom_va_md_register success\n");
#endif

	return 0;
}

static struct platform_driver msm_gsi_driver = {
	.probe          = msm_gsi_probe,
	.driver		= {
		.name	= "gsi",
		.of_match_table = msm_gsi_match,
	},
};

static struct platform_device *pdev;

/**
 * Module Init.
 */
static int __init gsi_init(void)
{
	int ret;

	pr_debug("%s\n", __func__);

	ret = platform_driver_register(&msm_gsi_driver);
	if (ret < 0)
		goto out;

	if (running_emulation) {
		pdev = platform_device_register_simple("gsi", -1, NULL, 0);
		if (IS_ERR(pdev)) {
			ret = PTR_ERR(pdev);
			platform_driver_unregister(&msm_gsi_driver);
			goto out;
		}
	}

out:
	return ret;
}
arch_initcall(gsi_init);

/*
 * Module exit.
 */
static void __exit gsi_exit(void)
{
	if (running_emulation && pdev)
		platform_device_unregister(pdev);
	platform_driver_unregister(&msm_gsi_driver);
}
module_exit(gsi_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Generic Software Interface (GSI)");