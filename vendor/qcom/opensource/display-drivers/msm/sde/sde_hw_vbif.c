// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021-2022, 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
 */
#include <linux/iopoll.h>

#include "sde_hwio.h"
#include "sde_hw_catalog.h"
#include "sde_hw_vbif.h"
#include "sde_dbg.h"

#define VBIF_VERSION			0x0000
#define VBIF_CLKON			0x0004
#define VBIF_CLK_FORCE_CTRL0		0x0008
#define VBIF_CLK_FORCE_CTRL1		0x000C
#define VBIF_QOS_REMAP_00		0x0020
#define VBIF_QOS_REMAP_01		0x0024
#define VBIF_QOS_REMAP_10		0x0028
#define VBIF_QOS_REMAP_11		0x002C
#define VBIF_WRITE_GATHER_EN		0x00AC
#define VBIF_IN_RD_LIM_CONF0		0x00B0
#define VBIF_IN_RD_LIM_CONF1		0x00B4
#define VBIF_IN_RD_LIM_CONF2		0x00B8
#define VBIF_IN_WR_LIM_CONF0		0x00C0
#define VBIF_IN_WR_LIM_CONF1		0x00C4
#define VBIF_IN_WR_LIM_CONF2		0x00C8
#define VBIF_OUT_RD_LIM_CONF0		0x00D0
#define VBIF_OUT_WR_LIM_CONF0		0x00D4
#define VBIF_OUT_AXI_AMEMTYPE_CONF0	0x0160
#define VBIF_OUT_AXI_AMEMTYPE_CONF1	0x0164
#define VBIF_OUT_AXI_ASHARED		0x0170
#define VBIF_OUT_AXI_AINNERSHARED	0x0174
#define VBIF_XIN_PND_ERR		0x0190
#define VBIF_XIN_SRC_ERR		0x0194
#define VBIF_XIN_CLR_ERR		0x019C
#define VBIF_XIN_HALT_CTRL0		0x0200
#define VBIF_XIN_HALT_CTRL1		0x0204
#define VBIF_AXI_HALT_CTRL0		0x0208
#define VBIF_AXI_HALT_CTRL1		0x020c
#define VBIF_XINL_QOS_RP_REMAP_000	0x0550
#define VBIF_XINL_QOS_LVL_REMAP_000	0x0590

static void sde_hw_clear_errors(struct sde_hw_vbif *vbif,
		u32 *pnd_errors, u32 *src_errors)
{
	struct sde_hw_blk_reg_map *c;
	u32 pnd, src;

	if (!vbif)
		return;
	c = &vbif->hw;
	pnd = SDE_REG_READ(c, VBIF_XIN_PND_ERR);
	src = SDE_REG_READ(c, VBIF_XIN_SRC_ERR);

	if (pnd_errors)
		*pnd_errors = pnd;
	if (src_errors)
		*src_errors = src;

	SDE_REG_WRITE(c, VBIF_XIN_CLR_ERR, pnd | src);
}

static void sde_hw_set_mem_type(struct sde_hw_vbif *vbif,
		u32 xin_id, u32 value)
{
	struct sde_hw_blk_reg_map *c;
	u32 reg_off;
	u32 bit_off;
	u32 reg_val;

	/*
	 * Assume 4 bits per bit field, 8 fields per 32-bit register so
	 * 16 bit fields maximum across two registers
	 */
	if (!vbif || xin_id >= MAX_XIN_COUNT)
		return;

	c = &vbif->hw;

	/* enable cacheable */
	if (xin_id >= 8) {
		xin_id -= 8;
		reg_off = VBIF_OUT_AXI_AMEMTYPE_CONF1;
	} else {
		reg_off = VBIF_OUT_AXI_AMEMTYPE_CONF0;
	}
	bit_off = (xin_id & 0x7) * 4;
	reg_val = SDE_REG_READ(c, reg_off);
	reg_val &= ~(0x7 << bit_off);
	reg_val |= (value & 0x7) << bit_off;
	SDE_REG_WRITE(c, reg_off, reg_val);
}

static void sde_hw_set_mem_type_v1(struct sde_hw_vbif *vbif,
		u32 xin_id, u32 value)
{
	struct sde_hw_blk_reg_map *c;
	u32 reg_val;

	if (!vbif || xin_id >= MAX_XIN_COUNT)
		return;

	sde_hw_set_mem_type(vbif, xin_id, value);

	c = &vbif->hw;

	/* disable outer shareable */
	reg_val = SDE_REG_READ(c, VBIF_OUT_AXI_ASHARED);
	reg_val &= ~BIT(xin_id);
	SDE_REG_WRITE(c, VBIF_OUT_AXI_ASHARED, 0);

	/* disable inner shareable */
	reg_val = SDE_REG_READ(c, VBIF_OUT_AXI_AINNERSHARED);
	reg_val &= ~BIT(xin_id);
	SDE_REG_WRITE(c, VBIF_OUT_AXI_AINNERSHARED, 0);
}

static void sde_hw_set_limit_conf(struct sde_hw_vbif *vbif,
		u32 xin_id, bool rd, u32 limit)
{
	struct sde_hw_blk_reg_map *c = &vbif->hw;
	u32 reg_val;
	u32 reg_off;
	u32 bit_off;

	if (rd)
		reg_off = VBIF_IN_RD_LIM_CONF0;
	else
		reg_off = VBIF_IN_WR_LIM_CONF0;

	reg_off += (xin_id / 4) * 4;
	bit_off = (xin_id % 4) * 8;
	reg_val = SDE_REG_READ(c, reg_off);
	reg_val &= ~(0xFF << bit_off);
	reg_val |= (limit) << bit_off;
	SDE_REG_WRITE(c, reg_off, reg_val);
}

static u32 sde_hw_get_limit_conf(struct sde_hw_vbif *vbif,
		u32 xin_id, bool rd)
{
	struct sde_hw_blk_reg_map *c = &vbif->hw;
	u32 reg_val;
	u32 reg_off;
	u32 bit_off;
	u32 limit;

	if (rd)
		reg_off = VBIF_IN_RD_LIM_CONF0;
	else
		reg_off = VBIF_IN_WR_LIM_CONF0;

	reg_off += (xin_id / 4) * 4;
	bit_off = (xin_id % 4) * 8;
	reg_val = SDE_REG_READ(c, reg_off);
	limit = (reg_val >> bit_off) & 0xFF;

	return limit;
}

static void sde_hw_set_xin_halt(struct sde_hw_vbif *vbif,
		u32 xin_id, bool enable)
{
	struct sde_hw_blk_reg_map *c = &vbif->hw;
	u32 reg_val;

	reg_val = SDE_REG_READ(c, VBIF_XIN_HALT_CTRL0);

	if (enable)
		reg_val |= BIT(xin_id);
	else
		reg_val &= ~BIT(xin_id);

	SDE_REG_WRITE(c, VBIF_XIN_HALT_CTRL0, reg_val);
	wmb(); /* make sure that xin client halted */
}

static bool sde_hw_get_xin_halt_status(struct sde_hw_vbif *vbif,
		u32 xin_id)
{
	struct sde_hw_blk_reg_map *c = &vbif->hw;
	u32 reg_val;

	reg_val = SDE_REG_READ(c, VBIF_XIN_HALT_CTRL1);

	return ((reg_val >> 16) & BIT(xin_id)) ? true : false;
}

static void sde_hw_set_axi_halt(struct sde_hw_vbif *vbif)
{
	struct sde_hw_blk_reg_map *c = &vbif->hw;

	SDE_REG_WRITE(c, VBIF_CLKON, BIT(0));
	SDE_REG_WRITE(c, VBIF_AXI_HALT_CTRL0, BIT(0));
	wmb(); /* make sure that axi transactions are halted */
}

static int sde_hw_get_axi_halt_status(struct sde_hw_vbif *vbif)
{
	struct sde_hw_blk_reg_map *c = &vbif->hw;
	int ctrl = 0, ret = 0;
	u32 val;

	ret = read_poll_timeout(sde_reg_read, ctrl, (ctrl & BIT(0)),
			100, 4000, false, c, VBIF_AXI_HALT_CTRL1);

	/* check AXI port 0 & 1 status on error */
	if (ret) {
		val = SDE_REG_READ(c, VBIF_AXI_HALT_CTRL1);
		ret = (val & (BIT(4) | BIT(5))) ? 0 : ret;
	}

	return ret;
}

static void sde_hw_set_qos_remap(struct sde_hw_vbif *vbif,
		u32 xin_id, u32 level, u32 rp_remap, u32 lvl_remap)
{
	struct sde_hw_blk_reg_map *c;
	u32 reg_val, reg_val_lvl, mask, reg_high, reg_shift;

	if (!vbif)
		return;

	c = &vbif->hw;

	reg_high = ((xin_id & 0x8) >> 3) * 4 + (level * 8);
	reg_shift = (xin_id & 0x7) * 4;

	reg_val = SDE_REG_READ(c, VBIF_XINL_QOS_RP_REMAP_000 + reg_high);
	reg_val_lvl = SDE_REG_READ(c, VBIF_XINL_QOS_LVL_REMAP_000 + reg_high);

	mask = 0x7 << reg_shift;

	reg_val &= ~mask;
	reg_val |= (rp_remap << reg_shift) & mask;

	reg_val_lvl &= ~mask;
	reg_val_lvl |= (lvl_remap << reg_shift) & mask;

	SDE_REG_WRITE(c, VBIF_XINL_QOS_RP_REMAP_000 + reg_high, reg_val);
	SDE_REG_WRITE(c, VBIF_XINL_QOS_LVL_REMAP_000 + reg_high, reg_val_lvl);
}

static void sde_hw_set_write_gather_en(struct sde_hw_vbif *vbif, u32 xin_id)
{
	struct sde_hw_blk_reg_map *c;
	u32 reg_val;

	if (!vbif || xin_id >= MAX_XIN_COUNT)
		return;

	c = &vbif->hw;

	reg_val = SDE_REG_READ(c, VBIF_WRITE_GATHER_EN);
	reg_val |= BIT(xin_id);
	SDE_REG_WRITE(c, VBIF_WRITE_GATHER_EN, reg_val);
}

static void _setup_vbif_ops(const struct sde_mdss_cfg *m,
		struct sde_hw_vbif_ops *ops, unsigned long cap)
{
	ops->set_limit_conf = sde_hw_set_limit_conf;
	ops->get_limit_conf = sde_hw_get_limit_conf;
	ops->set_axi_halt = sde_hw_set_axi_halt;
	ops->get_axi_halt_status = sde_hw_get_axi_halt_status;
	ops->set_xin_halt = sde_hw_set_xin_halt;
	ops->get_xin_halt_status = sde_hw_get_xin_halt_status;
	if (test_bit(SDE_VBIF_QOS_REMAP, &cap))
		ops->set_qos_remap = sde_hw_set_qos_remap;
	if (test_bit(SDE_VBIF_DISABLE_SHAREABLE, &cap))
		ops->set_mem_type = sde_hw_set_mem_type_v1;
	else
		ops->set_mem_type = sde_hw_set_mem_type;
	ops->clear_errors = sde_hw_clear_errors;
	ops->set_write_gather_en = sde_hw_set_write_gather_en;
}

static const struct sde_vbif_cfg *_top_offset(enum sde_vbif vbif,
		const struct sde_mdss_cfg *m,
		void __iomem *addr,
		struct sde_hw_blk_reg_map *b)
{
	int i;

	for (i = 0; i < m->vbif_count; i++) {
		if (vbif == m->vbif[i].id) {
			b->base_off = addr;
			b->blk_off = m->vbif[i].base;
			b->length = m->vbif[i].len;
			b->hw_rev = m->hw_rev;
			b->log_mask = SDE_DBG_MASK_VBIF;
			return &m->vbif[i];
		}
	}

	return ERR_PTR(-EINVAL);
}

struct sde_hw_vbif *sde_hw_vbif_init(enum sde_vbif idx,
		void __iomem *addr,
		const struct sde_mdss_cfg *m)
{
	struct sde_hw_vbif *c;
	const struct sde_vbif_cfg *cfg;

	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return ERR_PTR(-ENOMEM);

	cfg = _top_offset(idx, m, addr, &c->hw);
	if (IS_ERR_OR_NULL(cfg)) {
		kfree(c);
		return ERR_PTR(-EINVAL);
	}

	/*
	 * Assign ops
	 */
	c->idx = idx;
	c->cap = cfg;
	_setup_vbif_ops(m, &c->ops, c->cap->features);

	/* no need to register sub-range in sde dbg, dump entire vbif io base */

	mutex_init(&c->mutex);

	return c;
}

void sde_hw_vbif_destroy(struct sde_hw_vbif *vbif)
{
	if (vbif)
		mutex_destroy(&vbif->mutex);
	kfree(vbif);
}
