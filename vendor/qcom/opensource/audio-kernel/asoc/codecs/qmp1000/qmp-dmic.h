/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _QMP_DMIC_H
#define _QMP_DMIC_H

#include <linux/regmap.h>
#include <linux/soundwire/sdw_registers.h>
#include <bindings/audio-codec-port-types.h>

#define ZERO 0
/* QMP SDCA Control - function number */
#define FUNC_NUM_BERL_DIG 0x00
#define FUNC_NUM_SMP_MIC1 0x01
#define FUNC_NUM_SMP_MIC2 0x02

/* QMP SDCA entity */
#define QMP_SDCA_ENT_ENT0   0x00
#define QMP_SDCA_ENT_IT11   0x01
#define QMP_SDCA_ENT_PDE11  0x02
#define QMP_SDCA_ENT_SMPU   0x08
#define QMP_SDCA_ENT_ITCS   0x03
#define QMP_SDCA_ENT_DATAPATH_REG2   0x0b

/* QMP SDCA control */
#define QMP_SDCA_CTL_PDE_REQ_PS    0x01
#define QMP_SDCA_CTL_IT_USAGE      0x04
#define QMP_SDCA_CTL_FUNC_STAT     0x10
#define QMP_SDCA_CTL_PDE_ACT_PS    0x10
#define QMP_SDCA_CTL_FUNC_ACT      0x11
#define QMP_SDCA_CTL_CLOCK_VALID   0x02
#define QMP_SDCA_CTL_SENS_ADJ      0x36
#define QMP_SDCA_CTL_LP_SENS_ADJ   0x37

/* QMP SDCA control number */
#define QMP_SDCA_CTL_NUM0  0x00
#define QMP_SDCA_CTL_NUM1  0x01
#define QMP_SDCA_CTL_NUM2  0x02
#define QMP_SDCA_CTL_NUM3  0x03

static const struct reg_default qmp_sdca_dmic_reg_defaults[] = {
	{ SDW_SDCA_CTL(FUNC_NUM_BERL_DIG, QMP_SDCA_ENT_DATAPATH_REG2, QMP_SDCA_CTL_SENS_ADJ,
			QMP_SDCA_CTL_NUM0), 0x01 },
	{ SDW_SDCA_CTL(FUNC_NUM_BERL_DIG, QMP_SDCA_ENT_DATAPATH_REG2, QMP_SDCA_CTL_SENS_ADJ,
			QMP_SDCA_CTL_NUM1), 0x00 },
	{ SDW_SDCA_CTL(FUNC_NUM_BERL_DIG, QMP_SDCA_ENT_DATAPATH_REG2, QMP_SDCA_CTL_SENS_ADJ,
			QMP_SDCA_CTL_NUM2), 0x01 },
	{ SDW_SDCA_CTL(FUNC_NUM_BERL_DIG, QMP_SDCA_ENT_DATAPATH_REG2, QMP_SDCA_CTL_SENS_ADJ,
			QMP_SDCA_CTL_NUM3), 0x00 },
	{ SDW_SDCA_CTL(FUNC_NUM_BERL_DIG, QMP_SDCA_ENT_DATAPATH_REG2, QMP_SDCA_CTL_LP_SENS_ADJ,
			QMP_SDCA_CTL_NUM0), 0x00 },
	{ SDW_SDCA_CTL(FUNC_NUM_SMP_MIC1, QMP_SDCA_ENT_IT11, QMP_SDCA_CTL_IT_USAGE,
			QMP_SDCA_CTL_NUM0), 0x01 },
	{ SDW_SDCA_CTL(FUNC_NUM_SMP_MIC1, QMP_SDCA_ENT_PDE11, QMP_SDCA_CTL_PDE_REQ_PS,
			QMP_SDCA_CTL_NUM0), 0x03 },
	{ SDW_SDCA_CTL(FUNC_NUM_SMP_MIC1, QMP_SDCA_ENT_ENT0, QMP_SDCA_CTL_FUNC_STAT,
			QMP_SDCA_CTL_NUM0), 0x67 },
	{ SDW_SDCA_CTL(FUNC_NUM_SMP_MIC1, QMP_SDCA_ENT_ENT0, QMP_SDCA_CTL_FUNC_ACT,
			QMP_SDCA_CTL_NUM0), 0x00 },
	{ SDW_SDCA_CTL(FUNC_NUM_SMP_MIC1, QMP_SDCA_ENT_PDE11, QMP_SDCA_CTL_PDE_ACT_PS,
			QMP_SDCA_CTL_NUM0), 0x03 },
	{ SDW_SDCA_CTL(FUNC_NUM_SMP_MIC1, QMP_SDCA_ENT_ITCS, QMP_SDCA_CTL_CLOCK_VALID,
			QMP_SDCA_CTL_NUM0), 0x00 },
	{ SDW_SDCA_CTL(FUNC_NUM_SMP_MIC2, QMP_SDCA_ENT_IT11, QMP_SDCA_CTL_IT_USAGE,
			QMP_SDCA_CTL_NUM0), 0x01 },
	{ SDW_SDCA_CTL(FUNC_NUM_SMP_MIC2, QMP_SDCA_ENT_PDE11, QMP_SDCA_CTL_PDE_REQ_PS,
			QMP_SDCA_CTL_NUM0), 0x03 },
	{ SDW_SDCA_CTL(FUNC_NUM_SMP_MIC2, QMP_SDCA_ENT_ENT0, QMP_SDCA_CTL_FUNC_STAT,
			QMP_SDCA_CTL_NUM0), 0x67 },
	{ SDW_SDCA_CTL(FUNC_NUM_SMP_MIC2, QMP_SDCA_ENT_ENT0, QMP_SDCA_CTL_FUNC_ACT,
			QMP_SDCA_CTL_NUM0), 0x00 },
	{ SDW_SDCA_CTL(FUNC_NUM_SMP_MIC2, QMP_SDCA_ENT_SMPU, QMP_SDCA_CTL_FUNC_STAT,
			QMP_SDCA_CTL_NUM0), 0x00 },
	{ SDW_SDCA_CTL(FUNC_NUM_SMP_MIC2, QMP_SDCA_ENT_SMPU, QMP_SDCA_CTL_FUNC_ACT,
			QMP_SDCA_CTL_NUM0), 0x00 },
	{ SDW_SDCA_CTL(FUNC_NUM_SMP_MIC2, QMP_SDCA_ENT_PDE11, QMP_SDCA_CTL_PDE_ACT_PS,
			QMP_SDCA_CTL_NUM0), 0x03 },
	{ SDW_SDCA_CTL(FUNC_NUM_SMP_MIC2, QMP_SDCA_ENT_ITCS, QMP_SDCA_CTL_CLOCK_VALID,
			QMP_SDCA_CTL_NUM0), 0x00 },
};

#endif /* __QMP_DMIC_H__ */