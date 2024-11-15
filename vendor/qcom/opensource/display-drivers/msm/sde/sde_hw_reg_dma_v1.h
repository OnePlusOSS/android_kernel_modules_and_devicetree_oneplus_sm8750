/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 */
#ifndef _SDE_HW_REG_DMA_V1_H
#define _SDE_HW_REG_DMA_V1_H

#include "sde_reg_dma.h"

/**
 * init_v1() - initialize the reg dma v1 driver by installing v1 ops
 * @reg_dma - reg_dma hw info structure exposing capabilities.
 * @dpu_idx: dpu index
 */
int init_v1(struct sde_hw_reg_dma *reg_dma, u32 dpu_idx);

/**
 * init_v11() - initialize the reg dma v11 driver by installing v11 ops
 * @reg_dma - reg_dma hw info structure exposing capabilities.
 * @dpu_idx: dpu index
 */
int init_v11(struct sde_hw_reg_dma *reg_dma, u32 dpu_idx);

/**
 * init_v12() - initialize the reg dma v12 driver by installing v12 ops
 * @reg_dma - reg_dma hw info structure exposing capabilities.
 * @dpu_idx: dpu index
 */
int init_v12(struct sde_hw_reg_dma *reg_dma, u32 dpu_idx);

/**
 * init_v2() - initialize the reg dma v2 driver by installing v2 ops
 * @reg_dma - reg_dma hw info structure exposing capabilities.
 * @dpu_idx: dpu index
 */
int init_v2(struct sde_hw_reg_dma *reg_dma, u32 dpu_idx);

/**
 * init_v3() - initialize the reg dma v3 driver by installing v2 ops
 * @reg_dma - reg_dma hw info structure exposing capabilities.
 * @dpu_idx: dpu index
 */
int init_v3(struct sde_hw_reg_dma *reg_dma, u32 dpu_idx);

/**
 * deinit_v1() - free up any resources allocated during the v1 reg dma init
 * @dpu_idx: dpu index
 */
void deinit_v1(u32 dpu_idx);
#endif /* _SDE_HW_REG_DMA_V1_H */
