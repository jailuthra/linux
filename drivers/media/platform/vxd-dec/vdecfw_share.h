/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VXD DEC SYSDEV and UI Interface header
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
#ifndef _VDECFW_SHARE_H_
#define _VDECFW_SHARE_H_

/*
 * NOTE :
 * vdecfw_share_macros.h has been mearged into this file.
 */
/*
 * This macro sets alignment for a field structure.
 * Parameters :
 * a - alignment value
 * t - field type
 * n - field name
 */
#define IMG_ALIGN_FIELD(a, t, n) t n  __attribute__ ((aligned(a)))

/* END of vdecfw_share_macros.h */

/*
 * Field alignments in shared data structures
 */
/* Default field alignment */
#define VDECFW_SHARE_DEFAULT_ALIGNMENT	4
/* 64-bit field alignment */
#define VDECFW_SHARE_64BIT_ALIGNMENT	8
/* Pointer field alignment */
#define VDECFW_SHARE_PTR_ALIGNMENT	4

#endif /* _VDECFW_SHARE_H_ */
