/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VXD Decoder CORE and V4L2 Node Interface header
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

#ifndef __CORE_H__
#define __CORE_H__

#include <linux/scatterlist.h>
#include <linux/types.h>
#include "decoder.h"

int core_initialise(void *dev_handle, u32 internal_heap_id,
		    void *cb);

int core_deinitialise(void);

int core_supported_features(struct vdec_features *features);

int core_stream_create(void *vxd_dec_ctx_arg,
		       const struct vdec_str_configdata *str_cfgdata,
		       u32 *res_str_id);

int core_stream_destroy(u32 res_str_id);

int core_stream_play(u32 res_str_id);

int core_stream_stop(u32 res_str_id);

int core_stream_map_buf(u32 res_str_id, enum vdec_buf_type buf_type,
			struct vdec_buf_info *buf_info, u32 *buf_map_id);

int core_stream_map_buf_sg(u32 res_str_id,
			   enum vdec_buf_type buf_type,
			   struct vdec_buf_info *buf_info,
			   struct sg_table *sgt, u32 *buf_map_id);

int core_stream_unmap_buf(u32 buf_map_id);

int core_stream_unmap_buf_sg(u32 buf_map_id);

int core_stream_submit_unit(u32 res_str_id,
			    struct vdecdd_str_unit *str_unit);

int core_stream_fill_pictbuf(u32 buf_map_id);

/* This function to be called before stream play */
int core_stream_set_output_config(u32 res_str_id,
				  struct vdec_str_opconfig *str_opcfg,
				  struct vdec_pict_bufconfig *pict_bufcg);

int core_stream_flush(u32 res_str_id, u8 discard_refs);

int core_stream_release_bufs(u32 res_str_id,
			     enum vdec_buf_type buf_type);

int core_stream_get_status(u32 res_str_id,
			   struct vdecdd_decstr_status *str_status);

#endif
