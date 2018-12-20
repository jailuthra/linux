/* SPDX-License-Identifier: GPL-2.0 */
/*
 * IMG PVDEC pixel Registers
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

#ifndef _MEM_IO_H
#define _MEM_IO_H

#include <linux/types.h>
#include "reg_io2.h"

#define RND_TO_WORDS(size) ((((size) + 3) / 4) * 4)

#define MEMIO_CHECK_ALIGNMENT(vpmem)        \
		IMG_ASSERT((vpmem))

#define MEMIO_READ_FIELD(vpmem, field) \
	((((*((field##_TYPE *)(((uintptr_t)(vpmem)) + field##_OFFSET))) & \
			field##_MASK) >> field##_SHIFT))

#define MEMIO_READ_TABLE_FIELD(vpmem, field, tabidx) \
	((((*((field##_TYPE *)(((uintptr_t)(vpmem)) + field##_OFFSET + \
	(field##_STRIDE * (tabidx))))) & field##_MASK) >> field##_SHIFT)) \

#define MEMIO_READ_REPEATED_FIELD(vpmem, field, repidx) ({ \
	typeof(val) __repidx = repidx; \
	((((*((field##_TYPE *)(((uintptr_t)(vpmem)) + field##_OFFSET))) & \
	(field##_MASK >> ((__repidx) * field##_SIZE))) >> \
	(field##_SHIFT - ((__repidx) * field##_SIZE)))); }) \

#define MEMIO_READ_TABLE_REPEATED_FIELD(vpmem, field, tabidx, repidx) ({ \
	typeof(val) __repidx = repidx; \
	((((*((field##_TYPE *)(((uintptr_t)(vpmem)) + field##_OFFSET + \
	(field##_STRIDE * (tabidx))))) & (field##_MASK >> \
	((__repidx) * field##_SIZE))) >> (field##_SHIFT - \
	((__repidx) * field##_SIZE)))); }) \

#define MEMIO_WRITE_FIELD(vpmem, field, value) \
	do { \
		typeof(vpmem) __vpmem = vpmem; \
		MEMIO_CHECK_ALIGNMENT(__vpmem); \
		(*((field##_TYPE *)(((uintptr_t)(__vpmem)) + \
		field##_OFFSET))) = \
		(field##_TYPE)(((*((field##_TYPE *)(((uintptr_t)(__vpmem)) + \
		field##_OFFSET))) & ~(field##_TYPE)field##_MASK) | \
		(field##_TYPE)(((value) << field##_SHIFT) & field##_MASK)); \
	} while (0) \

#define MEMIO_WRITE_FIELD_LITE(vpmem, field, value) \
	do { \
		typeof(vpmem) __vpmem = vpmem; \
		MEMIO_CHECK_ALIGNMENT(__vpmem); \
		(*((field##_TYPE *)(((uintptr_t)(__vpmem)) + \
		field##_OFFSET))) = \
		((*((field##_TYPE *)(((uintptr_t)(__vpmem)) + \
		field##_OFFSET))) |\
		(field##_TYPE) (((value) << field##_SHIFT))); \
	} while (0) \

#define MEMIO_WRITE_TABLE_FIELD(vpmem, field, tabidx, value) \
	do { \
		typeof(vpmem) __vpmem = vpmem; \
		typeof(tabidx) __tabidx = tabidx; \
		MEMIO_CHECK_ALIGNMENT(__vpmem); \
		IMG_ASSERT(((__tabidx) < field##_NO_ENTRIES) || \
		(field##_NO_ENTRIES == 0)); \
		(*((field##_TYPE *)(((uintptr_t)(__vpmem)) + field##_OFFSET + \
		(field##_STRIDE * (__tabidx))))) =  \
		((*((field##_TYPE *)(((uintptr_t)(__vpmem)) + field##_OFFSET + \
		(field##_STRIDE * (__tabidx))))) & \
		(field##_TYPE)~field##_MASK) | \
		(field##_TYPE)(((value) << field##_SHIFT) & field##_MASK); \
	} while (0) \

#define MEMIO_WRITE_REPEATED_FIELD(vpmem, field, repidx, value) \
	do { \
		typeof(vpmem) __vpmem = vpmem; \
		typeof(repidx) __repidx = repidx; \
		MEMIO_CHECK_ALIGNMENT(__vpmem); \
		IMG_ASSERT((__repidx) < field##_NO_REPS); \
		(*((field##_TYPE *)(((uintptr_t)(__vpmem)) + \
		field##_OFFSET))) = \
		((*((field##_TYPE *)(((uintptr_t)(__vpmem)) + \
		field##_OFFSET))) & \
		(field##_TYPE)~(field##_MASK >> ((__repidx) * field##_SIZE)) | \
		(field##_TYPE)(((value) << (field##_SHIFT - \
		((__repidx) * field##_SIZE))) & (field##_MASK >> \
		((__repidx) * field##_SIZE)))); \
	} while (0) \

#define MEMIO_WRITE_TABLE_REPEATED_FIELD(vpmem, field, tabidx, repidx, value) \
	do { \
		typeof(vpmem) __vpmem = vpmem; \
		typeof(tabidx) __tabidx = tabidx; \
		typeof(repidx) __repidx = repidx; \
		MEMIO_CHECK_ALIGNMENT(__vpmem); \
		IMG_ASSERT(((__tabidx) < field##_NO_ENTRIES) || \
		(field##_NO_ENTRIES == 0)); \
		IMG_ASSERT((__repidx) < field##_NO_REPS); \
		(*((field##_TYPE *)(((uintptr_t)(__vpmem)) + field##_OFFSET + \
		(field##_STRIDE * (__tabidx))))) = \
		((*((field##_TYPE *)(((uintptr_t)(__vpmem)) + field##_OFFSET + \
		(field##_STRIDE * (__tabidx))))) & \
		(field##_TYPE)~(field##_MASK >> \
		((__repidx) * field##_SIZE))) | (field##_TYPE)(((value) << \
		(field##_SHIFT - ((__repidx) * field##_SIZE))) & \
		(field##_MASK >> ((__repidx) * field##_SIZE))); \
	} while (0) \

#endif /* _MEM_IO_H */
