// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "cam_debug_util.h"
#include "cam_presil_hw_access.h"

bool cam_presil_mode_enabled(void)
{
	return false;
}

bool cam_presil_subscribe_device_irq(int irq_num,
	irq_handler_t irq_handler, void *irq_priv_data, const char *irq_name)
{
	return true;
}

bool cam_presil_unsubscribe_device_irq(int irq_num)
{
	return true;
}

int cam_presil_register_read(void *addr, uint32_t *val)
{
	return CAM_PRESIL_SUCCESS;
}

int cam_presil_register_write(void *addr, uint32_t value, uint32_t flags)
{
	return CAM_PRESIL_SUCCESS;
}

int cam_presil_send_buffer(uint64_t dma_buf_uint, int mmu_hdl, uint32_t offset,
	uint32_t size, uint32_t addr32, uintptr_t cpu_vaddr, bool hfi_skip)
{
	return CAM_PRESIL_SUCCESS;
}

int cam_presil_retrieve_buffer(uint64_t dma_buf_uint, int mmu_hdl,
	uint32_t offset, uint32_t size, uint32_t addr32, uintptr_t cpu_vaddr, bool hfi_skip)
{
	return CAM_PRESIL_SUCCESS;
}

int cam_presil_readl_poll_timeout(void __iomem *mem_address, uint32_t val,
	int max_try_count, int interval_msec)
{
	return CAM_PRESIL_SUCCESS;
}

int cam_presil_hfi_write_cmd(void *hfi_cmd, uint32_t cmdlen, uint32_t client_id)
{
	return CAM_PRESIL_SUCCESS;
}

int cam_presil_hfi_read_message(uint32_t *pmsg, uint8_t q_id,
	uint32_t *words_read, uint32_t client_id)
{
	return CAM_PRESIL_SUCCESS;
}

int cam_presil_send_event(uint32_t event_id, uint32_t value)
{
	return CAM_PRESIL_SUCCESS;
}

int cam_presil_enqueue_presil_irq_tasklet(CAM_PRESIL_IRQ_HANDLER_BOTTOM_HALF bh_handler,
	void *handler_priv,
	void *payload)
{
	return CAM_PRESIL_SUCCESS;
}
