/* SPDX-License-Identifier: GPL-2.0 */
/*
 * V-DEC MMU Definitions
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

#ifndef _VXD_MMU_DEF_H_
#define _VXD_MMU_DEF_H_

/*
 * This type defines MMU variant.
 */
enum mmu_etype {
	MMU_TYPE_NONE = 0,
	MMU_TYPE_32BIT,
	MMU_TYPE_36BIT,
	MMU_TYPE_40BIT,
};

/*
 * This type defines the MMU heaps.
 * @0:	Heap for untiled video buffers
 * @1:	Heap for bitstream buffers
 * @2:	Heap for Stream buffers
 * @3:	Number of heaps
 */
enum mmu_eheap_id {
	MMU_HEAP_IMAGE_BUFFERS_UNTILED = 0x00,
	MMU_HEAP_BITSTREAM_BUFFERS,
	MMU_HEAP_STREAM_BUFFERS,
	MMU_HEAP_MAX,
};

#endif /* _VXD_MMU_DEFS_H_ */
