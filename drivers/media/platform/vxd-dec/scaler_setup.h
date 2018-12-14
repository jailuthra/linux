/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VXD DEC constants calculation and scalling coefficients
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
#ifndef _SCALER_SETUP_H
#define _SCALER_SETUP_H

#define LOWP				11
#define HIGHP				14

#define FIXED(a, digits)		((int)((a) * (1 << (digits))))

struct scaler_params {
	u32  vert_pitch;
	u32  vert_startpos;
	u32  vert_pitch_chroma;
	u32  vert_startpos_chroma;
	u32  horz_pitch;
	u32  horz_startpos;
	u32  horz_pitch_chroma;
	u32  horz_startpos_chroma;
	u8   fixed_point_shift;
};

struct scaler_filter {
	u8 bhoriz_bilinear;
	u8 bvert_bilinear;
};

struct scaler_pitch {
	int horiz_luma;
	int vert_luma;
	int horiz_chroma;
	int vert_chroma;
};

struct scaler_config {
	enum vdec_vid_std vidstd;
	const struct vxd_coreprops *coreprops;
	struct pixel_pixinfo *in_pixel_info;
	const struct pixel_pixinfo *out_pixel_info;
	u8 bfield_coded;
	u8 bseparate_chroma_planes;
	u32 recon_width;
	u32 recon_height;
	u32 mb_width;
	u32 mb_height;
	u32 scale_width;
	u32 scale_height;
};

#endif /* _SCALER_SETUP_H */
