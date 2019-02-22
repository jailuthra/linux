/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VXD PVDEC Private header file
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _VXD_PVDEC_PRIV_H
#define _VXD_PVDEC_PRIV_H

#include <linux/interrupt.h>

#include "img_dec_common.h"
#include "vxd_pvdec_regs.h"
#include "vxd_dec.h"

struct vxd_boot_poll_params {
	unsigned int msleep_cycles;
};

struct vxd_ena_params {
	struct vxd_boot_poll_params boot_poll;

	size_t fw_buf_size;
	u32 fw_buf_virt_addr; /*
			       * VXD's MMU virtual address of a firmware
			       * buffer.
			       */
	u32 ptd; /* Shifted physical address of PTD */

	/* Required for firmware upload via registers. */
	struct {
		const u8 *buf; /* Firmware blob buffer */

	} regs_data;

	struct {
		unsigned secure:1;  /* Secure flow indicator. */
		unsigned wait_dbg_fifo:1; /*
					   * Indicates that fw shall use
					   * blocking mode when putting logs
					   * into debug fifo
					   */
	};

	/* Structure containing memory staller configuration */
	struct {
		u32 *data;          /* Configuration data array */
		u8 size;            /* Configuration size in dwords */

	} mem_staller;

	u32 fwwdt_ms;      /* Firmware software watchdog timeout value */

	u32 crc; /* HW signatures to be enabled by firmware */
	u32 rendec_addr; /* VXD's virtual address of a rendec buffer */
	u16 rendec_size; /* Size of a rendec buffer in 4K pages */
};

int vxd_pvdec_init(const struct device *dev, void __iomem *reg_base);

int vxd_pvdec_ena(const struct device *dev, void __iomem *reg_base,
		  struct vxd_ena_params *ena_params, struct vxd_fw_hdr *hdr,
		  u32 *freq_khz);

int vxd_pvdec_dis(const struct device *dev, void __iomem *reg_base);

int vxd_pvdec_mmu_flush(const struct device *dev, void __iomem *reg_base);

int vxd_pvdec_send_msg(const struct device *dev, void __iomem *reg_base,
		       u32 *msg, size_t msg_size, u16 msg_id,
		       struct vxd_dev *ctx);

int vxd_pvdec_pend_msg_info(const struct device *dev, void __iomem *reg_base,
			    size_t *size, u16 *msg_id, bool *not_last_msg);

int vxd_pvdec_recv_msg(const struct device *dev, void __iomem *reg_base,
		       u32 *buf, size_t buf_size, struct vxd_dev *ctx);

int vxd_pvdec_check_fw_status(const struct device *dev, void __iomem *reg_base);

size_t vxd_pvdec_peek_mtx_fifo(const struct device *dev,
			       void __iomem *reg_base);

size_t vxd_pvdec_read_mtx_fifo(const struct device *dev, void __iomem *reg_base,
			       u32 *buf, size_t size);

irqreturn_t vxd_pvdec_clear_int(void __iomem *reg_base, u32 *irq_status);

int vxd_pvdec_check_irq(const struct device *dev, void __iomem *reg_base,
			u32 irq_status);

int vxd_pvdec_msg_fit(const struct device *dev, void __iomem *reg_base,
		      size_t msg_size);

void vxd_pvdec_get_state(const struct device *dev, void __iomem *reg_base,
			 u32 num_pipes, struct vxd_hw_state *state);

int vxd_pvdec_get_props(const struct device *dev, void __iomem *reg_base,
			struct vxd_core_props *props);

size_t vxd_pvdec_get_dbg_fifo_size(void __iomem *reg_base);

int vxd_pvdec_dump_mtx_ram(const struct device *dev, void __iomem *reg_base,
			   u32 addr, u32 count, u32 *buf);

int vxd_pvdec_dump_mtx_status(const struct device *dev, void __iomem *reg_base,
			      u32 *array, u32 array_size);

#endif /* _VXD_PVDEC_PRIV_H */
