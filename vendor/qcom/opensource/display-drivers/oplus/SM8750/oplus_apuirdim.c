/***************************************************************
** Copyright (C), 2025, OPLUS Mobile Comm Corp., Ltd
** File : oplus_apuirdim.c
** Description : oplus_apuirdim implement
** Version : 2.0
** Date : 2025/08/13
** Author : Display
***************************************************************/
#include "oplus_apuirdim.h"
#include <linux/pinctrl/consumer.h>
#include "oplus_adfr.h"
#include "oplus_display_device_ioctl.h"
#include "oplus_display_utils.h"
#include "oplus_display_pwm.h"
#include "oplus_display_ext.h"
#include "dsi_display.h"
#include "sde_trace.h"
#include "sde_encoder_phys.h"
#include "oplus_display_panel_cmd.h"
#include "oplus_display_effect.h"
#include <uapi/linux/sched/types.h>
enum oplus_apuir_log_level {
	OPLUS_APUIR_LOG_LEVEL_NONE = 0,
	OPLUS_APUIR_LOG_LEVEL_ERR = 1,
	OPLUS_APUIR_LOG_LEVEL_WARN = 2,
	OPLUS_APUIR_LOG_LEVEL_INFO = 3,
	OPLUS_APUIR_LOG_LEVEL_DEBUG = 4,
};
/* for nvt start */
#define APUIR_DS_READ_LENGTH 75
#define APUIR_BUF_STR_LEN (APUIR_DS_READ_LENGTH * 3 + 1)
#define APUIR_DS_READ_ROW 1
#define APUIR_OFF_FRAME_COUNT 2
u8 apuirregs_loading[APUIR_DS_READ_LENGTH] = {0};
u8 apuirregs_loading_mode1[APUIR_DS_READ_LENGTH] = {0};
u8 apuirregs_loading_mode2[APUIR_DS_READ_LENGTH] = {0};
u8 apuirregs_loading_modeoff[APUIR_DS_READ_LENGTH] = {0};
/* for nvt end */

unsigned int oplus_apuir_log_level = OPLUS_APUIR_LOG_LEVEL_INFO;
EXPORT_SYMBOL(oplus_apuir_log_level);
unsigned int oplus_apuir_display_id = 0;
EXPORT_SYMBOL(oplus_apuir_display_id);
uint32_t m_apuirdim_ds = 0;
bool m_apuirdim_ds_update = false;
struct workqueue_struct *apuir_setcmd_wq;
static struct work_struct apuir_setcmd_work;
static bool apuir_work_inited;
static enum dsi_cmd_set_type mAPuirType = DSI_CMD_APUIR_ON;
int off_framecount = 0;
static int first_loading = 0;
u32* m_up800nit_ds_list = NULL;
u32* m_less800nit_ds_list = NULL;
int m_up800nit_ds_list_count = 0;
int m_less800nit_ds_list_count = 0;
u32 m_ds1x0_real = 0, m_ds1x3_real = 0, m_ds2x0_real = 0;
u32 m_ds21x0_real = 0, m_ds21x3_real = 0;
int apuir_enable = 0;

void oplus_apuir_setenable(int enable) {
	apuir_enable = enable;
	APUIR_INFO("apuir_enable:%d\n", apuir_enable);
}

int oplus_get_apuir_enable(void) {
	return apuir_enable;
}

void oplus_apuir_init(void *dsi_panel)
{
	struct dsi_panel *panel = dsi_panel;
	if (!panel) {
		APUIR_ERR("Invalid params\n");
		return;
	}
	if (!strcmp(panel->type, "primary")) {
		apuir_setcmd_wq = create_singlethread_workqueue("apuir_setcmd0");
	} else if (!strcmp(panel->type, "secondary")) {
		apuir_setcmd_wq = create_singlethread_workqueue("apuir_setcmd1");
	}
	INIT_WORK(&apuir_setcmd_work, oplus_apuir_setcmd_work_handler);
	apuir_work_inited = true;
}

static void oplus_apuir_ensure_workqueue(struct dsi_panel *panel)
{
	const char *type = (panel && panel->type) ? panel->type : "unknown";

	if (!apuir_work_inited) {
		INIT_WORK(&apuir_setcmd_work, oplus_apuir_setcmd_work_handler);
		apuir_work_inited = true;
	}

	if (!apuir_setcmd_wq) {
		if (!strcmp(type, "primary"))
			apuir_setcmd_wq = create_singlethread_workqueue("apuir_setcmd0");
		else if (!strcmp(type, "secondary"))
			apuir_setcmd_wq = create_singlethread_workqueue("apuir_setcmd1");
		else
			apuir_setcmd_wq = create_singlethread_workqueue("apuir_setcmd");
	}
}
/* -------------------- aod -------------------- */
void oplus_apuir_setcmd_work_handler(struct work_struct *work_item)
{
	int rc = 0;
	struct dsi_display *display = oplus_display_get_current_display();
	struct dsi_panel *panel = NULL;

	APUIR_DEBUG("start\n");

	if (!display || !display->panel || !display->panel->cur_mode) {
		APUIR_ERR("invalid display or panel params\n");
		return;
	}
	/* uir cmd are available in power on */
	if (display->panel->power_mode != SDE_MODE_DPMS_ON) {
		APUIR_ERR("should not send uir cmd when power mode is %u\n", display->panel->power_mode);
		return;
	}

	panel = display->panel;
	SDE_ATRACE_BEGIN("oplus_apuir_setcmd_work_handler");
	SDE_ATRACE_BEGIN("oplus_apuir_set_cmd_replace");
	oplus_panel_cmd_reg_replace_specific_row(display->panel, display->panel->cur_mode, mAPuirType, apuirregs_loading, APUIR_DS_READ_LENGTH, APUIR_DS_READ_ROW);
	SDE_ATRACE_END("oplus_apuir_set_cmd_replace");

	mutex_lock(&panel->panel_lock);
	/* uir cmd are available in power on */
	if (display->panel->power_mode != SDE_MODE_DPMS_ON) {
		APUIR_ERR("Should not send uir cmd when power mode is %u\n", display->panel->power_mode);
		mutex_unlock(&panel->panel_lock);
		SDE_ATRACE_END("oplus_apuir_setcmd_work_handler");
		return;
	}
	SDE_ATRACE_BEGIN("cmdset");
	rc = dsi_panel_tx_cmd_set(display->panel, mAPuirType, false);
	SDE_ATRACE_END("cmdset");
	if (rc) {
		APUIR_ERR("[%s] failed to send DSI_CMD_POST_ON_BACKLIGHT cmd, rc=%d\n", display->name, rc);
	}
	mutex_unlock(&panel->panel_lock);
	SDE_ATRACE_END("oplus_apuir_setcmd_work_handler");

	APUIR_DEBUG("end\n");

	return;
}

bool oplus_apuir_get_uir_state(void)
{
	uint32_t aplds = m_apuirdim_ds & 0xfff;
	if ((m_apuirdim_ds != 0 && aplds == 0 && off_framecount == APUIR_OFF_FRAME_COUNT) || m_apuirdim_ds == 0) {
		APUIR_INFO("oplus_apuir_get_uir_state return false aplds %d m_apuirdim_ds = %x off_framecount = %d\n", aplds, m_apuirdim_ds, off_framecount);
		return false;
	} else {
		APUIR_INFO("oplus_apuir_get_uir_state return true aplds %d m_apuirdim_ds = %x off_framecount = %d\n", aplds, m_apuirdim_ds, off_framecount);
		return true;
	}
}

int oplus_apuir_set_ds(void *sde_enc_v)
{
	struct sde_encoder_virt *sde_enc = (struct sde_encoder_virt *)sde_enc_v;
	struct sde_encoder_phys *phys = NULL;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	u32 propval;

	APUIR_DEBUG("start\n");
	if (!oplus_get_apuir_enable()) {
		return 0;
	}

	if (!sde_enc) {
		APUIR_ERR("invalid sde_encoder_virt parameters\n");
		return 0;
	}

	phys = sde_enc->phys_encs[0];
	if (!phys || !phys->connector) {
		APUIR_ERR("invalid sde_encoder_phys parameters\n");
		return 0;
	}

	c_conn = to_sde_connector(phys->connector);
	if (!c_conn) {
		APUIR_ERR("invalid sde_connector parameters\n");
		return 0;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI)
		return 0;

	display = c_conn->display;
	if (!display || !display->panel || !display->panel->cur_mode) {
		APUIR_ERR("invalid display or panel params\n");
		return -EINVAL;
	}
	propval = sde_connector_get_property(c_conn->base.state, CONNECTOR_PROP_UIR_DS);
	if ((m_apuirdim_ds != propval || (off_framecount > 0 && off_framecount < APUIR_OFF_FRAME_COUNT))) {
		APUIR_INFO("apuirdriver:m_apuirdim_ds:0x%08x %d \n", propval, propval);
		m_apuirdim_ds = propval;
		m_apuirdim_ds_update = true;
		if (m_apuirdim_ds != 0) {
			oplus_apuir_set_cmd(display, m_apuirdim_ds);
		}
	}
	return 0;
}

/*for nvt start*/
static void exchangeregs(int ds, u8* apuirregs, int index0, int index1, int index2, int index3) {
	int temp1 = 0, temp2 = 1, temp3 = 0, temp4 = 0;
	int dsh = 0, dsl = 0;

	dsh = (ds >> 8) & 0xFF;
	dsl = ds & 0xFF;

	temp1 = apuirregs[index0];
	temp2 = apuirregs[index1];
	temp3 = apuirregs[index2];
	temp4 = apuirregs[index3];

	temp1 = (temp1 & 0x0F) | (dsh << 4);
	temp2 = (temp2 & 0xF0) | (dsh & 0x0F);
	temp3 = dsl;
	temp4 = dsl;

	apuirregs[index0] = temp1;
	apuirregs[index1] = temp2;
	apuirregs[index2] = temp3;
	apuirregs[index3] = temp4;
}

void oplus_apuir_set_up800nit_ds_list(int count, u32* list) {
	m_up800nit_ds_list_count = count;
	if (m_up800nit_ds_list_count != 3 || !list) {
		APUIR_ERR("oplus_apuir_set_up800nit_ds_list %d != 3 or list is null\n", m_up800nit_ds_list_count);
		return;
	}
	m_up800nit_ds_list = list;
	m_ds1x0_real = m_up800nit_ds_list[0];
	m_ds1x3_real = m_up800nit_ds_list[1];
	m_ds2x0_real = m_up800nit_ds_list[2];
	APUIR_INFO("oplus_apuir_set_up800nit_ds_list %d %d %d\n", m_up800nit_ds_list[0], m_up800nit_ds_list[1], m_up800nit_ds_list[2]);
}

void oplus_apuir_set_less800nit_ds_list(int count, u32* list) {
	m_less800nit_ds_list_count  = count;
	if (m_less800nit_ds_list_count != 2 || !list) {
		APUIR_ERR("oplus_apuir_set_less800nit_ds_list %d != 2 or list is null\n", m_less800nit_ds_list_count);
		return;
	}
	m_less800nit_ds_list = list;
	m_ds21x0_real = m_less800nit_ds_list[0];
	m_ds21x3_real = m_less800nit_ds_list[1];
	APUIR_INFO("oplus_apuir_set_less800nit_ds_list %d %d\n", m_less800nit_ds_list[0], m_less800nit_ds_list[1]);
}

static void transfer_ds(u32* aplds, u32* oprds) {
	/* hwc ds setting */
	u32 DS1X0_predic = 2801;
	u32 DS2X0_predic = 3839;
	u32 aplratio = 0, oprratio = 0;
	u32 temp = *aplds, temp1 = *oprds;

	if (*aplds != 0) {
		aplratio = (*aplds - DS1X0_predic) * 100000 / (DS2X0_predic - DS1X0_predic);
		*aplds = aplratio * (m_ds2x0_real - m_ds1x0_real) / 100000 + m_ds1x0_real;
	}
	if (*oprds != 0) {
		oprratio = (*oprds - DS1X0_predic) * 100000 / (DS2X0_predic - DS1X0_predic);
		*oprds = oprratio * (m_ds2x0_real - m_ds1x0_real) / 100000 + m_ds1x0_real;
	}
	APUIR_INFO("apuirdriver aplratio %d aplds(%d->%d) oprratio %d oprds(%d->%d)\n", aplratio, temp, *aplds, oprratio, temp1, *oprds);
}

void oplus_apuir_set_cmd(void *dsi_display, unsigned int ds)
{
	struct dsi_display *display = dsi_display;
	struct dsi_panel *panel = NULL;
	int seed_mode = 0;
	u8* apuirregs = apuirregs_loading;
	u32 aplds = ds & 0xfff, oprds = ds >> 12;
	u32 modepose = 73;
	u32 apl_lpose = 60, apl_hpose = 61;
	u32 aplmode = 0x11, oprmode = 0x17;
	int index0 = 40, index1 = 48, index2 = 47, index3 = 49;
	/* int index0 = 14, index1 = 17, index2 = 16, index3 = 18; */
	int index4 = 37, index5 = 40, index6 = 39, index7 = 41;
	/* int index4 = 11, index5 = 14, index6 = 13, index7 = 15; */
	/* ds */
	u32 ds_min = m_ds1x0_real, ds_max = m_ds2x0_real;
	/* ds2 */
	u32 ds2_min = m_ds21x0_real, ds2_max = m_ds21x3_real;
	u32 ratio = 0;
	u32 ds2 = 0;
	int ret = 0;

	enum dsi_cmd_set_type loading1type = DSI_CMD_LOADING_EFFECT_MODE1;
	enum dsi_cmd_set_type loading2type = DSI_CMD_LOADING_EFFECT_MODE2;
	enum dsi_cmd_set_type loadingofftype = DSI_CMD_LOADING_EFFECT_OFF;

	if (!dsi_display) {
		APUIR_ERR("NULL dsi_display");
		return;
	}
	panel = display->panel;
	if (!dsi_panel_initialized(panel)) {
		ADFR_DEBUG("should not send cmd sets if panel is not initialized\n");
		return;
	}
	if (!m_apuirdim_ds_update) {
		APUIR_INFO("update skipped");
		return;
	}
	m_apuirdim_ds_update = false;
	if (!display->panel || !display->panel->cur_mode || !display->panel->cur_mode->priv_info) {
		APUIR_ERR("invalid panel params\n");
		return;
	}

	/* uir cmd are available in power on */
	if (display->panel->power_mode != SDE_MODE_DPMS_ON) {
		APUIR_ERR("should not send uir cmd when power mode is %u\n", display->panel->power_mode);
		return;
	}

	SDE_ATRACE_BEGIN("oplus_apuir_set_cmd");
	transfer_ds(&aplds, &oprds);
	/*for nvt start*/
	if (!first_loading) {
		APUIR_INFO("apuirdriver loading apuir cmds\n");
		ret = oplus_panel_cmd_reg_read_specific_row(panel, panel->cur_mode, loading1type, apuirregs_loading_mode1, APUIR_DS_READ_LENGTH, APUIR_DS_READ_ROW);
		if (ret <0) {
			APUIR_ERR("apuirdriver loading apuirregs_loading_mode1 error\n");
			return;
		}
		ret = oplus_panel_cmd_reg_read_specific_row(panel, panel->cur_mode, loading2type, apuirregs_loading_mode2, APUIR_DS_READ_LENGTH, APUIR_DS_READ_ROW);
		if (ret <0) {
			APUIR_ERR("apuirdriver loading apuirregs_loading_mode2 error\n");
			return;
		}
		ret = oplus_panel_cmd_reg_read_specific_row(panel, panel->cur_mode, loadingofftype, apuirregs_loading_modeoff, APUIR_DS_READ_LENGTH, APUIR_DS_READ_ROW);
		if (ret <0) {
			APUIR_ERR("apuirdriver loading apuirregs_loading_modeoff error\n");
			return;
		}
		first_loading = 1;
	}
	/*for nvt end*/

	seed_mode = __oplus_get_seed_mode();
	switch (seed_mode) {
	case PANEL_LOADING_EFFECT_MODE1:
		memcpy(apuirregs, apuirregs_loading_mode1, sizeof(apuirregs_loading_mode1));
		break;
	case PANEL_LOADING_EFFECT_MODE2:
		memcpy(apuirregs, apuirregs_loading_mode2, sizeof(apuirregs_loading_mode2));
		break;
	case PANEL_LOADING_EFFECT_OFF:
		memcpy(apuirregs, apuirregs_loading_modeoff, sizeof(apuirregs_loading_modeoff));
		break;
	default:
		memcpy(apuirregs, apuirregs_loading_mode1, sizeof(apuirregs_loading_mode1));
		break;
	}

	if (aplds > 0) {
		if (aplds > m_ds1x3_real) {
			ds2 = ds2_max;
			APUIR_DEBUG("apuirdriver aplds > 0 ds2 = %d 0x%x\n", ds2, ds2);
		} else {
			ratio = (aplds - ds_min) * 100000 / (ds_max - ds_min);
			ds2 = ratio * (ds2_max - ds2_min) / 30000 + ds2_min;
			APUIR_DEBUG("apuirdriver aplds > 0 ds2 = %d 0x%x ratio = %d / 100000\n", ds2, ds2, ratio);
		}
		mAPuirType = DSI_CMD_APUIR_ON;
		exchangeregs(ds2, apuirregs, index4, index5, index6, index7);
	} else if (off_framecount < APUIR_OFF_FRAME_COUNT - 1) {
		if (oprds > m_ds1x3_real) {
			ds2 = ds2_max;
			APUIR_DEBUG("apuirdriver aplds == 0 ds2 = %d 0x%x\n", ds2, ds2);
		} else {
			ratio = (oprds - ds_min) * 100000 / (ds_max - ds_min);
			ds2 = ratio * (ds2_max - ds2_min) / 30000 + ds2_min;
			APUIR_DEBUG("apuirdriver aplds == 0 ds2 = %d 0x%x ratio = %d / 100000\n", ds2, ds2, ratio);
		}
		exchangeregs(ds2, apuirregs, index4, index5, index6, index7);
	}

	if (aplds == 0) {
		off_framecount++;
		if (off_framecount >= 1 && off_framecount < APUIR_OFF_FRAME_COUNT) {
			exchangeregs(oprds, apuirregs, index0, index1, index2, index3);
			mAPuirType = DSI_CMD_APUIR_MIDDLE_OFF;
			apuirregs[modepose] = oprmode;
			apuirregs[apl_lpose] = 0xFF;
			apuirregs[apl_hpose] = 0xFF;
		} else if (off_framecount == APUIR_OFF_FRAME_COUNT) {
			mAPuirType = DSI_CMD_APUIR_OFF;
		}
	} else {
		off_framecount = 0;
		mAPuirType = DSI_CMD_APUIR_ON;
		apuirregs[modepose] = aplmode;
		apuirregs[apl_lpose] = 0x3F;
		apuirregs[apl_hpose] = 0x33;
		exchangeregs(aplds, apuirregs, index0, index1, index2, index3);
	}

	APUIR_INFO("apuirdriver replace off_framecount %d DSI_CMD_APUIR_ON %d DSI_CMD_APUIR_OFF %d mAPuirType = %d seed_mode = %d regs len=%d\n",
			off_framecount, DSI_CMD_APUIR_ON, DSI_CMD_APUIR_OFF, mAPuirType, seed_mode, APUIR_DS_READ_LENGTH);
	oplus_apuir_ensure_workqueue(display->panel);
	if (unlikely(!apuir_setcmd_wq)) {
		APUIR_WARN("workqueue not ready, calling handler directly\n");
		oplus_apuir_setcmd_work_handler(&apuir_setcmd_work);
	} else {
		queue_work(apuir_setcmd_wq, &apuir_setcmd_work);
	}
	SDE_ATRACE_END("oplus_apuir_set_cmd");
}
/*for nvt end*/

