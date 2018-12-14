/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Low-level VXD interface component
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

#ifndef _VXD_BUF_H
#define _VXD_BUF_H

/*
 * struct vxdio_ddbufinfo - contains information about virtual address
 * @buf_size: the size of the buffer (in bytes).
 * @cpu_virt: the cpu virtual address  (mapped into the local cpu mmu)
 * @dev_virt: device virtual address (pages mapped into IMG H/W mmu)
 * @hndl_memory: handle to device mmu mapping
 * @buff_id: buffer id used in communication with interface
 * @is_internal: true, if the buffer is allocated internally
 * @ref_count: reference count (number of users)
 * @kmstr_id: stream id
 * @core_id: core id
 */
struct vxdio_ddbufinfo {
	u32 buf_size;
	void *cpu_virt;
	u32 dev_virt;
	void *hndl_memory;
	u32 buff_id;
	u32 is_internal;
	u32 ref_count;
	u32 kmstr_id;
	u32 core_id;
};

#endif /* _VXD_BUF_H */
