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

#ifndef _VXD_PROPS_H
#define _VXD_PROPS_H

#include "vdec_defs.h"
#include "vxd_mmu_defs.h"

#define VDEC_MAX_PIXEL_PIPES 2

#define VXD_MAX_CORES    1
#define VER_STR_LEN      64

#define CORE_REVISION(maj, min, maint) \
	((((maj) & 0xff) << 16) | (((min) & 0xff) << 8) | (((maint) & 0xff)))
#define MAJOR_REVISION(rev)	(((rev) >> 16) & 0xff)
#define MINOR_REVISION(rev)	(((rev) >> 8) & 0xff)
#define MAINT_REVISION(rev)	((rev) & 0xff)

#define FROM_REV(maj, min, maint) ({ \
		typeof(maj) __maj = maj; \
		typeof(min) __min = min; \
		(((maj_rev) > (__maj)) || \
		(((maj_rev) == (__maj)) && ((min_rev) > (__min))) || \
		(((maj_rev) == (__maj)) && ((min_rev) == (__min)) && \
		((int)(maint_rev) >= (maint)))); })

#define BEFORE_REV(maj, min, maint) ({ \
	typeof(maj) __maj = maj; \
	typeof(min) __min = min; \
	(((int)(maj_rev) < (__maj)) || \
	(((maj_rev) == (__maj)) && ((int)(min_rev) < (__min))) || \
	(((maj_rev) == (__maj)) && ((min_rev) == (__min)) && \
	((int)(maint_rev) < (maint)))); })

#define AT_REV(maj, min, maint)	(((maj_rev) == (u32)maj) && \
	((min_rev) == (u32)min) && ((maint_rev) == (u32)maint))

struct vxd_vidstd_props {
	enum vdec_vid_std vidstd;
	u32      core_rev;
	u32      min_width;
	u32      min_height;
	u32      max_width;
	u32      max_height;
	u32      max_macroblocks;
	u32      max_luma_bitdepth;
	u32      max_chroma_bitdepth;
	enum pixel_fmt_idc max_chroma_format;
};

struct vxd_coreprops {
	char aversion[VER_STR_LEN];
	u8 mpeg2[VDEC_MAX_PIXEL_PIPES];
	u8 mpeg4[VDEC_MAX_PIXEL_PIPES];
	u8 h264[VDEC_MAX_PIXEL_PIPES];
	u8 vc1[VDEC_MAX_PIXEL_PIPES];
	u8 avs[VDEC_MAX_PIXEL_PIPES];
	u8 real[VDEC_MAX_PIXEL_PIPES];
	u8 jpeg[VDEC_MAX_PIXEL_PIPES];
	u8 vp6[VDEC_MAX_PIXEL_PIPES];
	u8 vp8[VDEC_MAX_PIXEL_PIPES];
	u8 hevc[VDEC_MAX_PIXEL_PIPES];
	u8 rotation_support[VDEC_MAX_PIXEL_PIPES];
	u8 scaling_support[VDEC_MAX_PIXEL_PIPES];
	u8 hd_support;
	u32 num_streams[VDEC_MAX_PIXEL_PIPES];
	u32 num_entropy_pipes;
	u32 num_pixel_pipes;
	struct vxd_vidstd_props vidstd_props[VDEC_STD_MAX];
	enum mmu_etype mmu_type;
	u8 mmu_support_stride_per_context;
	u8 mmu_support_secure;
	/* Range extensions supported by hw -> used only by hevc */
	u8 hevc_range_ext[VDEC_MAX_PIXEL_PIPES];
};

#endif /* _VXD_PROPS_H */
