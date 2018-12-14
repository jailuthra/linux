/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VXD Decoder device driver utility header
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

#ifndef __VDECDD_UTILS_H__
#define __VDECDD_UTILS_H__

#include "img_errors.h"
#include "vdecdd_defs.h"

/* The picture buffer alignment (in bytes) for VXD. */
#define VDEC_VXD_PICTBUF_ALIGNMENT		(64)
/* The buffer alignment (in bytes) for VXD. */
#define VDEC_VXD_BUF_ALIGNMENT			(4096)
/* The extended stride alignment for VXD.  */
#define VDEC_VXD_EXT_STRIDE_ALIGNMENT_DEFAULT	(64)
/* Macroblock dimension (width and height) in pixels. */
#define VDEC_MB_DIMENSION			(16)

static inline u32 vdec_size_min(u32 a, u32 b)
{
	return(a <= b ? a : b);
}

static inline u8 vdec_size_lt(struct vdec_pict_size sa,
			      struct vdec_pict_size sb)
{
	return((sa.width < sb.width && sa.height <= sb.height) ||
	       (sa.width <= sb.width && sa.height < sb.height));
}

static inline u8 vdec_size_ge(struct vdec_pict_size sa,
			      struct vdec_pict_size sb)
{
	return(sa.width >= sb.width && sa.height >= sb.height);
}

static inline u8 vdec_size_ne(struct vdec_pict_size sa,
			      struct vdec_pict_size sb)
{
	return(sa.width != sb.width || sa.height != sb.height);
}

static inline u8 vdec_size_nz(struct vdec_pict_size sa)
{
	return(sa.width != 0 && sa.height != 0);
}

int vdecddutils_free_strunit(struct vdecdd_str_unit *str_unit);

int vdecddutils_create_strunit(struct vdecdd_str_unit **str_unit_handle,
			       struct lst_t *bs_list);

int vdecddutils_ref_pict_get_maxnum
	(const struct vdec_str_configdata *str_cfg_data,
	 const struct vdec_comsequ_hdrinfo *comseq_hdr_info,
	 u32 *num_picts);

int vdecddutils_get_minrequired_numpicts
	(const struct vdec_str_configdata *str_cfg_data,
	 const struct vdec_comsequ_hdrinfo *comseq_hdr_info,
	 const struct vdec_str_opconfig *op_cfg,
	 u32 *num_picts);

int vdecddutils_pictbuf_getconfig
	(const struct vdec_str_configdata *str_cfg_data,
	 const struct vdec_pict_rend_config *pict_rend_cfg,
	 const struct vdec_str_opconfig *str_opcfg,
	 struct vdec_pict_bufconfig *pict_bufcfg);

int vdecddutils_pictbuf_getinfo
	(const struct vdec_str_configdata *str_cfg_data,
	 const struct vdec_pict_rend_config *pict_rend_cfg,
	 const struct vdec_str_opconfig *str_opcfg,
	 struct vdec_pict_rendinfo *pict_rend_info);

int vdecddutils_convert_buffer_config
	(const struct vdec_str_configdata *str_cfg_data,
	 const struct vdec_pict_bufconfig *pict_bufcfg,
	 struct vdec_pict_rendinfo *pict_rend_info);

int vdecddutils_get_display_region
	(const struct vdec_pict_size *coded_size,
	 const struct vdec_rect *orig_disp_region,
	 struct vdec_rect *disp_region);

void vdecddutils_buf_vxd_adjust_size(u32 *buf_size);

#endif
