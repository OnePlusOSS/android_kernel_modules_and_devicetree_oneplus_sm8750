// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
 */

#define pr_fmt(fmt)	"[drm:%s:%d] " fmt, __func__, __LINE__
#include <drm/msm_drm_pp.h>
#include "sde_hw_mdss.h"
#include "sde_hwio.h"
#include "sde_hw_catalog.h"
#include "sde_hw_dspp.h"
#include "sde_hw_color_processing.h"
#include "sde_dbg.h"
#include "sde_ad4.h"
#include "sde_hw_rc.h"
#include "sde_kms.h"
#include "sde_aiqe_common.h"
#include "sde_hw_color_proc_aiqe_v1.h"

#define DSPP_VALID_START_OFF 0x800

static struct sde_dspp_cfg *_dspp_offset(enum sde_dspp dspp,
		struct sde_mdss_cfg *m,
		void __iomem *addr,
		struct sde_hw_blk_reg_map *b)
{
	int i;

	if (!m || !addr || !b)
		return ERR_PTR(-EINVAL);

	for (i = 0; i < m->dspp_count; i++) {
		if (dspp == m->dspp[i].id) {
			b->base_off = addr;
			b->blk_off = m->dspp[i].base;
			b->length = m->dspp[i].len;
			b->hw_rev = m->hw_rev;
			b->log_mask = SDE_DBG_MASK_DSPP;
			return &m->dspp[i];
		}
	}

	return ERR_PTR(-EINVAL);
}

static void dspp_igc(struct sde_hw_dspp *c)
{
	int ret = 0;

	if (c->cap->sblk->igc.version == SDE_COLOR_PROCESS_VER(0x3, 0x1)) {
		ret = reg_dmav1_init_dspp_op_v4(SDE_DSPP_IGC, c);
		if (!ret)
			c->ops.setup_igc = reg_dmav1_setup_dspp_igcv31;
		else
			c->ops.setup_igc = sde_setup_dspp_igcv3;
	} else if (c->cap->sblk->igc.version ==
			SDE_COLOR_PROCESS_VER(0x4, 0x0)) {
		c->ops.setup_igc = NULL;
		ret = reg_dmav2_init_dspp_op_v4(SDE_DSPP_IGC, c);
		if (!ret)
			c->ops.setup_igc = reg_dmav2_setup_dspp_igcv4;
	} else if (c->cap->sblk->igc.version ==
			SDE_COLOR_PROCESS_VER(0x5, 0x0)) {
		c->ops.setup_igc = NULL;
		ret = reg_dmav2_init_dspp_op_v4(SDE_DSPP_IGC, c);
		if (!ret)
			c->ops.setup_igc = reg_dmav2_setup_dspp_igcv5;
	}
}

static void dspp_pcc(struct sde_hw_dspp *c)
{
	int ret = 0;

	if (c->cap->sblk->pcc.version == SDE_COLOR_PROCESS_VER(0x1, 0x7)) {
		c->ops.setup_pcc = sde_setup_dspp_pcc_v1_7;
	} else if (c->cap->sblk->pcc.version ==
			SDE_COLOR_PROCESS_VER(0x4, 0x0)) {
		ret = reg_dmav1_init_dspp_op_v4(SDE_DSPP_PCC, c);
		if (!ret)
			c->ops.setup_pcc = reg_dmav1_setup_dspp_pccv4;
		else
			c->ops.setup_pcc = sde_setup_dspp_pccv4;
	} else if (c->cap->sblk->pcc.version ==
			SDE_COLOR_PROCESS_VER(0x5, 0x0) ||
			c->cap->sblk->pcc.version ==
			SDE_COLOR_PROCESS_VER(0x6, 0x0)) {
		ret = reg_dmav1_init_dspp_op_v4(SDE_DSPP_PCC, c);
		if (!ret)
			c->ops.setup_pcc = reg_dmav1_setup_dspp_pccv5;
		else
			c->ops.setup_pcc = NULL;
	}
}

static void dspp_gc(struct sde_hw_dspp *c)
{
	int ret = 0;

	if (c->cap->sblk->gc.version == SDE_COLOR_PROCESS_VER(0x1, 8)) {
		ret = reg_dmav1_init_dspp_op_v4(SDE_DSPP_GC, c);
		if (!ret)
			c->ops.setup_gc = reg_dmav1_setup_dspp_gcv18;
		/**
		 * programming for v18 through ahb is same as v17,
		 * hence assign v17 function
		 */
		else
			c->ops.setup_gc = sde_setup_dspp_gc_v1_7;
	} else if (c->cap->sblk->gc.version == SDE_COLOR_PROCESS_VER(0x2, 0x0)) {
		ret = reg_dmav1_init_dspp_op_v4(SDE_DSPP_GC, c);
		if (!ret)
			c->ops.setup_gc = reg_dmav1_setup_dspp_gcv2;
		else
			c->ops.setup_gc = NULL;
	}
}

static void dspp_hsic(struct sde_hw_dspp *c)
{
	int ret = 0;

	if (c->cap->sblk->hsic.version == SDE_COLOR_PROCESS_VER(0x1, 0x7)) {
		ret = reg_dmav1_init_dspp_op_v4(SDE_DSPP_HSIC, c);
		if (!ret)
			c->ops.setup_pa_hsic = reg_dmav1_setup_dspp_pa_hsicv17;
		else
			c->ops.setup_pa_hsic = sde_setup_dspp_pa_hsic_v17;
	}
}

static void dspp_memcolor(struct sde_hw_dspp *c)
{
	int ret = 0;

	if (c->cap->sblk->memcolor.version == SDE_COLOR_PROCESS_VER(0x1, 0x7)) {
		ret = reg_dmav1_init_dspp_op_v4(SDE_DSPP_MEMCOLOR, c);
		if (!ret) {
			c->ops.setup_pa_memcol_skin =
				reg_dmav1_setup_dspp_memcol_skinv17;
			c->ops.setup_pa_memcol_sky =
				reg_dmav1_setup_dspp_memcol_skyv17;
			c->ops.setup_pa_memcol_foliage =
				reg_dmav1_setup_dspp_memcol_folv17;
			c->ops.setup_pa_memcol_prot =
				reg_dmav1_setup_dspp_memcol_protv17;
		} else {
			c->ops.setup_pa_memcol_skin =
				sde_setup_dspp_memcol_skin_v17;
			c->ops.setup_pa_memcol_sky =
				sde_setup_dspp_memcol_sky_v17;
			c->ops.setup_pa_memcol_foliage =
				sde_setup_dspp_memcol_foliage_v17;
			c->ops.setup_pa_memcol_prot =
				sde_setup_dspp_memcol_prot_v17;
		}
	}
}

static void dspp_sixzone(struct sde_hw_dspp *c)
{
	int ret = 0;

	if (c->cap->sblk->sixzone.version == SDE_COLOR_PROCESS_VER(0x1, 0x7)) {
		ret = reg_dmav1_init_dspp_op_v4(SDE_DSPP_SIXZONE, c);
		if (!ret)
			c->ops.setup_sixzone = reg_dmav1_setup_dspp_sixzonev17;
		else
			c->ops.setup_sixzone = sde_setup_dspp_sixzone_v17;
	} else if (c->cap->sblk->sixzone.version ==
			SDE_COLOR_PROCESS_VER(0x2, 0x0)) {
		c->ops.setup_sixzone = NULL;
		ret = reg_dmav2_init_dspp_op_v4(SDE_DSPP_SIXZONE, c);
		if (!ret)
			c->ops.setup_sixzone = reg_dmav2_setup_dspp_sixzonev2;
	}
}

static void dspp_gamut(struct sde_hw_dspp *c)
{
	int ret = 0;

	if (c->cap->sblk->gamut.version == SDE_COLOR_PROCESS_VER(0x4, 0)) {
		ret = reg_dmav1_init_dspp_op_v4(SDE_DSPP_GAMUT, c);
		if (!ret)
			c->ops.setup_gamut = reg_dmav1_setup_dspp_3d_gamutv4;
		else
			c->ops.setup_gamut = sde_setup_dspp_3d_gamutv4;
	} else if (c->cap->sblk->gamut.version ==
			SDE_COLOR_PROCESS_VER(0x4, 1)) {
		ret = reg_dmav1_init_dspp_op_v4(SDE_DSPP_GAMUT, c);
		if (!ret)
			c->ops.setup_gamut = reg_dmav1_setup_dspp_3d_gamutv41;
		else
			c->ops.setup_gamut = sde_setup_dspp_3d_gamutv41;
	} else if (c->cap->sblk->gamut.version ==
			SDE_COLOR_PROCESS_VER(0x4, 2)) {
		ret = reg_dmav1_init_dspp_op_v4(SDE_DSPP_GAMUT, c);
		c->ops.setup_gamut = NULL;
		if (!ret)
			c->ops.setup_gamut = reg_dmav1_setup_dspp_3d_gamutv42;
	} else if (c->cap->sblk->gamut.version ==
			SDE_COLOR_PROCESS_VER(0x4, 3)) {
		c->ops.setup_gamut = NULL;
		ret = reg_dmav2_init_dspp_op_v4(SDE_DSPP_GAMUT, c);
		if (!ret)
			c->ops.setup_gamut = reg_dmav2_setup_dspp_3d_gamutv43;
	}
}

static void dspp_dither(struct sde_hw_dspp *c)
{
	if (c->cap->sblk->dither.version == SDE_COLOR_PROCESS_VER(0x1, 0x7))
		c->ops.setup_pa_dither = sde_setup_dspp_dither_v1_7;
}

static void dspp_hist(struct sde_hw_dspp *c)
{
	if (c->cap->sblk->hist.version == (SDE_COLOR_PROCESS_VER(0x1, 0x7))) {
		c->ops.setup_histogram = sde_setup_dspp_hist_v1_7;
		c->ops.read_histogram = sde_read_dspp_hist_v1_7;
		c->ops.lock_histogram = sde_lock_dspp_hist_v1_7;
	}
}

static void dspp_vlut(struct sde_hw_dspp *c)
{
	int ret = 0;

	if (c->cap->sblk->vlut.version == (SDE_COLOR_PROCESS_VER(0x1, 0x7))) {
		c->ops.setup_vlut = sde_setup_dspp_pa_vlut_v1_7;
	} else if (c->cap->sblk->vlut.version ==
			(SDE_COLOR_PROCESS_VER(0x1, 0x8))) {
		ret = reg_dmav1_init_dspp_op_v4(SDE_DSPP_VLUT, c);
		if (!ret)
			c->ops.setup_vlut = reg_dmav1_setup_dspp_vlutv18;
		else
			c->ops.setup_vlut = sde_setup_dspp_pa_vlut_v1_8;
	}
}

static void dspp_ad(struct sde_hw_dspp *c)
{
	if (c->cap->sblk->ad.version == SDE_COLOR_PROCESS_VER(4, 0)) {
		c->ops.setup_ad = sde_setup_dspp_ad4;
		c->ops.ad_read_intr_resp = sde_read_intr_resp_ad4;
		c->ops.validate_ad = sde_validate_dspp_ad4;
	}
}

static void dspp_ltm(struct sde_hw_dspp *c)
{
	int ret = 0;

	c->ops.validate_ltm_roi = NULL;

	if (c->cap->sblk->ltm.version == SDE_COLOR_PROCESS_VER(0x1, 0x0) ||
		c->cap->sblk->ltm.version == SDE_COLOR_PROCESS_VER(0x1, 0x1) ||
		c->cap->sblk->ltm.version == SDE_COLOR_PROCESS_VER(0x1, 0x2) ||
		c->cap->sblk->ltm.version == SDE_COLOR_PROCESS_VER(0x1, 0x3)) {
		ret = reg_dmav1_init_ltm_op_v6(SDE_LTM_INIT, c);
		if (!ret)
			ret = reg_dmav1_init_ltm_op_v6(SDE_LTM_ROI, c);
		if (!ret)
			ret = reg_dmav1_init_ltm_op_v6(SDE_LTM_VLUT, c);

		if (!ret) {
			if (c->cap->sblk->ltm.version ==
				SDE_COLOR_PROCESS_VER(0x1, 0x2) ||
				c->cap->sblk->ltm.version ==
				SDE_COLOR_PROCESS_VER(0x1, 0x3)) {
				c->ops.setup_ltm_vlut =
					reg_dmav1_setup_ltm_vlutv1_2;
				c->ops.setup_ltm_hist_ctrl =
					sde_setup_dspp_ltm_hist_ctrlv1_2;
				c->ops.clear_ltm_merge_mode =
					sde_ltm_clear_merge_modev1_2;
			} else {
				c->ops.setup_ltm_vlut =
					reg_dmav1_setup_ltm_vlutv1;
				c->ops.setup_ltm_hist_ctrl =
					sde_setup_dspp_ltm_hist_ctrlv1;
				c->ops.clear_ltm_merge_mode =
					sde_ltm_clear_merge_mode;
			}

			if (c->cap->sblk->ltm.version ==
					SDE_COLOR_PROCESS_VER(0x1, 0x3)) {
				c->ops.setup_ltm_roi = reg_dmav1_setup_ltm_roiv1_3;
				c->ops.validate_ltm_roi = sde_validate_ltm_roiv1_3;
			} else {
				c->ops.setup_ltm_roi = reg_dmav1_setup_ltm_roiv1;
			}

			c->ops.setup_ltm_init = reg_dmav1_setup_ltm_initv1;
			c->ops.setup_ltm_thresh = sde_setup_dspp_ltm_threshv1;
			c->ops.setup_ltm_hist_buffer =
				sde_setup_dspp_ltm_hist_bufferv1;
			c->ops.ltm_read_intr_status = sde_ltm_read_intr_status;
		} else {
			c->ops.setup_ltm_init = NULL;
			c->ops.setup_ltm_roi = NULL;
			c->ops.setup_ltm_vlut = NULL;
			c->ops.setup_ltm_thresh = NULL;
			c->ops.setup_ltm_hist_ctrl = NULL;
			c->ops.setup_ltm_hist_buffer = NULL;
			c->ops.ltm_read_intr_status = NULL;
			c->ops.clear_ltm_merge_mode = NULL;
		}
		if (!ret && (c->cap->sblk->ltm.version ==
			SDE_COLOR_PROCESS_VER(0x1, 0x1) ||
			c->cap->sblk->ltm.version ==
			SDE_COLOR_PROCESS_VER(0x1, 0x2) ||
			c->cap->sblk->ltm.version ==
			SDE_COLOR_PROCESS_VER(0x1, 0x3)))
			c->ltm_checksum_support = true;
		else
			c->ltm_checksum_support = false;
	}
}

static void dspp_rc(struct sde_hw_dspp *c)
{
	int ret = 0;

	if (!c) {
		SDE_ERROR("invalid arguments\n");
		return;
	}

	if (c->cap->sblk->rc.version == SDE_COLOR_PROCESS_VER(0x1, 0x0) ||
			c->cap->sblk->rc.version == SDE_COLOR_PROCESS_VER(0x1, 0x1)) {
		ret = sde_hw_rc_init(c);
		if (ret) {
			SDE_ERROR("rc init failed, ret %d\n", ret);
			return;
		}

		ret = reg_dmav1_init_dspp_op_v4(SDE_DSPP_RC, c);
		if (!ret) {
			c->ops.setup_rc_mask = reg_dmav1_setup_rc_mask_configv1;
			c->ops.setup_rc_pu_roi = reg_dmav1_setup_rc_pu_configv1;

		} else {
			c->ops.setup_rc_mask = sde_hw_rc_setup_mask;
			c->ops.setup_rc_pu_roi = sde_hw_rc_setup_pu_roi;
		}

		c->ops.validate_rc_mask = sde_hw_rc_check_mask;
		c->ops.validate_rc_pu_roi = sde_hw_rc_check_pu_roi;
	}
}

static void dspp_spr(struct sde_hw_dspp *c)
{
	int ret = 0;

	if (!c) {
		SDE_ERROR("invalid arguments\n");
		return;
	}

	c->ops.validate_spr_init_config = NULL;
	c->ops.validate_spr_udc_config = NULL;
	c->ops.setup_spr_init_config = NULL;
	c->ops.setup_spr_udc_config = NULL;
	c->ops.setup_spr_pu_config = NULL;
	c->ops.read_spr_opr_value = NULL;

	if (c->cap->sblk->spr.version == SDE_COLOR_PROCESS_VER(0x1, 0x0)) {
		ret = reg_dmav2_init_spr_op_v1(SDE_SPR_INIT, c);
		if (ret) {
			SDE_ERROR("regdma init failed for spr, ret %d\n", ret);
			return;
		}

		c->ops.setup_spr_init_config = reg_dmav1_setup_spr_init_cfgv1;
		c->ops.setup_spr_pu_config = reg_dmav1_setup_spr_pu_cfgv1;
		c->ops.read_spr_opr_value = sde_spr_read_opr_value;
	} else if (c->cap->sblk->spr.version == SDE_COLOR_PROCESS_VER(0x2, 0x0)) {
		ret = reg_dmav2_init_spr_op_v1(SDE_SPR_INIT, c);
		if (ret) {
			SDE_ERROR("regdma init failed for spr, ret %d\n", ret);
			return;
		}

		ret = reg_dmav2_init_spr_op_v1(SDE_SPR_UDC, c);
		if (ret) {
			SDE_ERROR("regdma init failed for spr udc, ret %d\n", ret);
			return;
		}

		c->ops.validate_spr_init_config = sde_spr_check_init_cfg;
		c->ops.validate_spr_udc_config = sde_spr_check_udc_cfg;
		c->ops.setup_spr_init_config = reg_dmav1_setup_spr_init_cfgv2;
		c->ops.setup_spr_udc_config = reg_dmav1_setup_spr_udc_cfgv2;
		c->ops.setup_spr_pu_config = reg_dmav1_setup_spr_pu_cfgv2;
		c->ops.read_spr_opr_value = sde_spr_read_opr_value;
	}
}

static void dspp_demura(struct sde_hw_dspp *c)
{
	int ret;
	c->ops.setup_demura_cfg = NULL;
	c->ops.setup_demura_backlight_cfg = NULL;
	c->ops.setup_demura_cfg0_param2 = NULL;

	if (c->cap->sblk->demura.version == SDE_COLOR_PROCESS_VER(0x1, 0x0)) {
		ret = reg_dmav1_init_dspp_op_v4(SDE_DSPP_DEMURA, c);
		if (!ret)
			ret = reg_dmav1_init_dspp_op_v4(SDE_DSPP_DEMURA_CFG0_PARAM2, c);

		if (!ret) {
			c->ops.setup_demura_cfg = reg_dmav1_setup_demurav1;
			c->ops.setup_demura_backlight_cfg =
					sde_demura_backlight_cfg;
			c->ops.demura_read_plane_status =
					sde_demura_read_plane_status;
			c->ops.setup_demura_pu_config = sde_demura_pu_cfg;
			c->ops.setup_demura_cfg0_param2 = reg_dmav1_setup_demura_cfg0_param2;
		}
	} else if (c->cap->sblk->demura.version == SDE_COLOR_PROCESS_VER(0x2, 0x0)) {
		ret = reg_dmav1_init_dspp_op_v4(SDE_DSPP_DEMURA, c);
		if (!ret)
			ret = reg_dmav1_init_dspp_op_v4(SDE_DSPP_DEMURA_CFG0_PARAM2, c);
		if (!ret) {
			c->ops.setup_demura_cfg = reg_dmav1_setup_demurav2;
			c->ops.setup_demura_backlight_cfg =
					sde_demura_backlight_cfg;
			c->ops.demura_read_plane_status =
					sde_demura_read_plane_status;
			c->ops.setup_demura_pu_config = sde_demura_pu_cfg;
			c->ops.setup_demura_cfg0_param2 = reg_dmav1_setup_demura_cfg0_param2;
		} else {
			SDE_ERROR("Regdma init dspp op failed for DemuraV2\n");
		}
	} else if (c->cap->sblk->demura.version == SDE_COLOR_PROCESS_VER(0x3, 0x0)) {
		ret = reg_dmav1_init_dspp_op_v4(SDE_DSPP_DEMURA, c);
		if (!ret)
			ret = reg_dmav1_init_dspp_op_v4(SDE_DSPP_DEMURA_CFG0_PARAM2, c);
		if (!ret) {
			c->ops.setup_demura_cfg = reg_dmav1_setup_demurav3;
			c->ops.setup_demura_backlight_cfg = sde_demura_backlight_cfg;
			c->ops.demura_read_plane_status = sde_demura_read_plane_status;
			c->ops.setup_demura_pu_config = sde_demura_pu_cfg;
			c->ops.setup_demura_cfg0_param2 = reg_dmav1_setup_demura_cfg0_param2;
		} else {
			SDE_ERROR("Regdma init dspp op failed for Demura v3\n");
		}
	}
}

static void dspp_aiqe(struct sde_hw_dspp *c)
{
	int ret = 0;

	if (!c) {
		SDE_ERROR("invalid arguments\n");
		return;
	}

	c->ops.setup_mdnie = NULL;
	c->ops.setup_mdnie_art = NULL;
	c->ops.setup_copr = NULL;
	c->ops.read_copr_status = NULL;
	c->ops.reset_mdnie_art = NULL;
	c->ops.setup_mdnie_psr = NULL;
	c->ops.validate_aiqe_ssrc_data = NULL;
	c->ops.setup_aiqe_ssrc_config = NULL;
	c->ops.setup_aiqe_ssrc_data = NULL;
	c->ops.setup_aiqe_abc = NULL;


	if (!c->sde_kms || !c->sde_kms->catalog)
		return;

	if (!c->sde_kms->catalog->ssip_allowed) {
		SDE_INFO("ssip_allowed = %d\n", c->sde_kms->catalog->ssip_allowed);
		return;
	}

	if (c->cap->sblk->aiqe.version == SDE_COLOR_PROCESS_VER(0x1, 0x0)) {
		ret = reg_dmav1_init_dspp_op_v4(SDE_DSPP_AIQE, c);
		if (!ret) {
			if (c->cap->sblk->aiqe.mdnie_supported) {
				c->ops.setup_mdnie = reg_dmav1_setup_mdnie_v1;
				c->ops.setup_mdnie_art = sde_setup_mdnie_art_v1;
				c->ops.reset_mdnie_art = sde_reset_mdnie_art;
				c->ops.setup_mdnie_psr = sde_setup_mdnie_psr;
			}

			if (c->cap->sblk->aiqe.ssrc_supported) {
				c->ops.validate_aiqe_ssrc_data = sde_validate_aiqe_ssrc_data_v1;
				c->ops.setup_aiqe_ssrc_config =
						reg_dmav1_setup_aiqe_ssrc_config_v1;
				c->ops.setup_aiqe_ssrc_data = reg_dmav1_setup_aiqe_ssrc_data_v1;
			}

			if (c->cap->sblk->aiqe.copr_supported) {
				c->ops.setup_copr = sde_setup_copr_v1;
				c->ops.read_copr_status = sde_read_copr_status;
			}

			if (c->cap->sblk->aiqe.abc_supported)
				c->ops.setup_aiqe_abc = sde_setup_aiqe_abc_v1;
		}
	}
}

static void dspp_ai_scaler(struct sde_hw_dspp *c)
{
	if (!c) {
		SDE_ERROR("invalid arguments\n");
		return;
	}

	c->ops.setup_ai_scaler = NULL;
	c->ops.check_ai_scaler = NULL;

	if (!c->sde_kms || !c->sde_kms->catalog)
		return;

	if (!c->sde_kms->catalog->ssip_allowed) {
		SDE_INFO("ssip_allowed = %d\n", c->sde_kms->catalog->ssip_allowed);
		return;
	}

	if (c->cap->sblk->ai_scaler.version == SDE_COLOR_PROCESS_VER(0x1, 0x0)) {
		if (c->cap->sblk->ai_scaler.ai_scaler_supported) {
			c->ops.check_ai_scaler = sde_check_ai_scaler_v1;
			c->ops.setup_ai_scaler = sde_setup_ai_scaler_v1;
		}
	}
}

static void (*dspp_blocks[SDE_DSPP_MAX])(struct sde_hw_dspp *c);

static void _init_dspp_ops(void)
{
	dspp_blocks[SDE_DSPP_IGC] = dspp_igc;
	dspp_blocks[SDE_DSPP_PCC] = dspp_pcc;
	dspp_blocks[SDE_DSPP_GC] = dspp_gc;
	dspp_blocks[SDE_DSPP_HSIC] = dspp_hsic;
	dspp_blocks[SDE_DSPP_MEMCOLOR] = dspp_memcolor;
	dspp_blocks[SDE_DSPP_SIXZONE] = dspp_sixzone;
	dspp_blocks[SDE_DSPP_GAMUT] = dspp_gamut;
	dspp_blocks[SDE_DSPP_DITHER] = dspp_dither;
	dspp_blocks[SDE_DSPP_HIST] = dspp_hist;
	dspp_blocks[SDE_DSPP_VLUT] = dspp_vlut;
	dspp_blocks[SDE_DSPP_AD] = dspp_ad;
	dspp_blocks[SDE_DSPP_LTM] = dspp_ltm;
	dspp_blocks[SDE_DSPP_RC] = dspp_rc;
	dspp_blocks[SDE_DSPP_SPR] = dspp_spr;
	dspp_blocks[SDE_DSPP_DEMURA] = dspp_demura;
	dspp_blocks[SDE_DSPP_AIQE] = dspp_aiqe;
	dspp_blocks[SDE_DSPP_AI_SCALER] = dspp_ai_scaler;
}

static void _setup_dspp_ops(struct sde_hw_dspp *c, unsigned long features)
{
	int i = 0;

	if (!c->cap->sblk)
		return;

	for (i = 0; i < SDE_DSPP_MAX; i++) {
		if (!test_bit(i, &features))
			continue;
		if (dspp_blocks[i])
			dspp_blocks[i](c);
	}
}

struct sde_hw_blk_reg_map *sde_hw_dspp_init(enum sde_dspp idx,
			void __iomem *addr,
			struct sde_mdss_cfg *m,
			struct sde_kms *sde_kms)
{
	struct sde_hw_dspp *c;
	struct sde_dspp_cfg *cfg;
	char buf[256];

	if (!addr || !m || !sde_kms)
		return ERR_PTR(-EINVAL);

	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return ERR_PTR(-ENOMEM);

	cfg = _dspp_offset(idx, m, addr, &c->hw);
	if (IS_ERR_OR_NULL(cfg)) {
		kfree(c);
		return ERR_PTR(-EINVAL);
	}

	/* Populate DSPP Top HW block */
	c->hw_top.base_off = addr;
	c->hw_top.blk_off = m->dspp_top.base;
	c->hw_top.length = m->dspp_top.len;
	c->hw_top.hw_rev = m->hw_rev;
	c->hw_top.log_mask = SDE_DBG_MASK_DSPP;
	c->dpu_idx = sde_kms->dev->primary->index;
	c->sde_kms = sde_kms;

	/* Assign ops */
	c->idx = idx;
	c->cap = cfg;
	_init_dspp_ops();
	_setup_dspp_ops(c, c->cap->features);

	sde_dbg_reg_register_dump_range(SDE_DBG_NAME, cfg->name,
			c->hw.blk_off + DSPP_VALID_START_OFF,
			c->hw.blk_off + c->hw.length, c->hw.xin_id);

	if ((cfg->sblk->ltm.id == SDE_DSPP_LTM) && cfg->sblk->ltm.base) {
		sde_dbg_reg_register_dump_range(SDE_DBG_NAME, "LTM",
				c->hw.blk_off + cfg->sblk->ltm.base,
				c->hw.blk_off + cfg->sblk->ltm.base + 0xC4,
				c->hw.xin_id);
	}

	if ((cfg->sblk->rc.id == SDE_DSPP_RC) && cfg->sblk->rc.base) {
		snprintf(buf, ARRAY_SIZE(buf), "%s_%d", "rc", c->idx - DSPP_0);
		sde_dbg_reg_register_dump_range(SDE_DBG_NAME, buf,
				c->hw.blk_off + cfg->sblk->rc.base,
				c->hw.blk_off + cfg->sblk->rc.base +
				cfg->sblk->rc.len, c->hw.xin_id);
	}

	if ((cfg->sblk->spr.id == SDE_DSPP_SPR) && cfg->sblk->spr.base) {
		snprintf(buf, ARRAY_SIZE(buf), "%s_%d", "spr", c->idx - DSPP_0);
		sde_dbg_reg_register_dump_range(SDE_DBG_NAME, buf,
				c->hw.blk_off + cfg->sblk->spr.base,
				c->hw.blk_off + cfg->sblk->spr.base +
				cfg->sblk->spr.len, c->hw.xin_id);
	}

	if ((cfg->sblk->demura.id == SDE_DSPP_DEMURA) &&
			cfg->sblk->demura.base) {
		snprintf(buf, ARRAY_SIZE(buf), "%s_%d", "demura",
				c->idx - DSPP_0);
		sde_dbg_reg_register_dump_range(SDE_DBG_NAME, buf,
				c->hw.blk_off + cfg->sblk->demura.base,
				c->hw.blk_off + cfg->sblk->demura.base +
				cfg->sblk->demura.len, c->hw.xin_id);
	}

	if (c->sde_kms->catalog && c->sde_kms->catalog->ssip_allowed) {
		if (cfg->sblk->aiqe.id == SDE_DSPP_AIQE && cfg->sblk->aiqe.base
				&& cfg->sblk->aiqe.base != 0xffffffff) {
			snprintf(buf, ARRAY_SIZE(buf), "%s_%d", "aiqe", c->idx - DSPP_0);
			sde_dbg_reg_register_dump_range(SDE_DBG_NAME, buf,
					c->hw.blk_off + cfg->sblk->aiqe.base,
					c->hw.blk_off + cfg->sblk->aiqe.base +
					cfg->sblk->aiqe.len, c->hw.xin_id);
		}

		if ((cfg->sblk->ai_scaler.id == SDE_DSPP_AI_SCALER) &&
				cfg->sblk->ai_scaler.base
				&& cfg->sblk->ai_scaler.base != 0xffffffff) {
			snprintf(buf, ARRAY_SIZE(buf), "%s_%d", "ai_scaler",
					c->idx - DSPP_0);
			sde_dbg_reg_register_dump_range(SDE_DBG_NAME, buf,
					c->hw.blk_off + cfg->sblk->ai_scaler.base,
					c->hw.blk_off + cfg->sblk->ai_scaler.base +
					cfg->sblk->ai_scaler.len, c->hw.xin_id);
		}
	}

	return &c->hw;
}

void sde_hw_dspp_destroy(struct sde_hw_blk_reg_map *hw)
{
	struct sde_hw_dspp *dspp;

	if (hw) {
		dspp = to_sde_hw_dspp(hw);
		reg_dmav1_deinit_dspp_ops(dspp);
		reg_dmav1_deinit_ltm_ops(dspp);
		kfree(dspp);
	}
}