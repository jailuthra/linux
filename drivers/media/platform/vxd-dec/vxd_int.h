/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VXD DEC Common low level core interface component
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
#ifndef _VXD_INT_H
#define _VXD_INT_H

#include "fw_interface.h"
#include "scaler_setup.h"
#include "vdecdd_defs.h"
#include "vdecfw_shared.h"
#include "vdec_defs.h"
#include "vxd_ext.h"
#include "vxd_props.h"

/*
 * Size of buffer used for batching messages
 */
#define BATCH_MSG_BUFFER_SIZE           (8 * 4096)

#define INTRA_BUF_SIZE                  (1024 * 32)
#define AUX_LINE_BUFFER_SIZE            (512 * 1024)

#define MAX_PICTURE_WIDTH               (4096)
#define MAX_PICTURE_HEIGHT              (4096)

 /*
  * this macro returns the host address of device buffer.
  */
#define GET_HOST_ADDR(buf) ((buf)->dev_virt)

#define GET_HOST_ADDR_OFFSET(buf, offset) (((buf)->dev_virt) + (offset))

/*
 * The extended stride alignment for VXD.
 */
#define VDEC_VXD_EXT_STRIDE_ALIGNMENT_DEFAULT  (64)

struct vxd_buffers {
	struct vdecdd_ddpict_buf *recon_pict;
	struct vdecdd_ddpict_buf *alt_pict;
	struct vxdio_ddbufinfo *intra_bufinfo;
	struct vxdio_ddbufinfo *auxline_bufinfo;
	struct vxdio_ddbufinfo *err_pict_bufinfo;
	u32 intra_bufsize_per_pipe;
	u32 auxline_bufsize_per_pipe;
	struct vxdio_ddbufinfo *msb_bufinfo;
	u8 btwopass;
};

struct pvdec_core_rev {
	u32 maj_rev;
	u32 min_rev;
	u32 maint_rev;
	u32 int_rev;
};

/*
 * this has all that it needs to translate a Stream Unit for a picture
 * into a transaction.
 */
int vxd_set_altpictcmds(const struct vdecdd_str_unit *str_unit,
			const struct vdec_str_configdata *str_configdata,
			const struct vdec_str_opconfig *output_config,
			const struct vxd_coreprops *coreprops,
			const struct vxd_buffers *buffers,
			u32 *pict_cmds);

/*
 * this has all that it needs to translate a Stream Unit for
 * a picture into a transaction.
 */
int vxd_set_reconpictcmds(const struct vdecdd_str_unit *str_unit,
			  const struct vdec_str_configdata *str_configdata,
			  const struct vdec_str_opconfig *output_config,
			  const struct vxd_coreprops *coreprops,
			  const struct vxd_buffers *buffers,
			  u32 *pict_cmds);

int vxd_getscalercmds(const struct scaler_config *scaler_config,
		      const struct scaler_pitch *pitch,
		      const struct scaler_filter *filter,
		      const struct pixel_pixinfo *out_loop_pixel_info,
		      struct scaler_params *params,
		      u32 *pict_cmds);

/*
 * this creates value of MSVDX_CMDS_CODED_PICTURE_SIZE register.
 */
u32 vxd_get_codedpicsize(u16  width_min1, u16 height_min1);

/*
 * return HW codec mode based on video standard.
 */
u8 vxd_get_codedmode(enum vdec_vid_std vidstd);

/*
 * translates core properties to the form of the struct vxd_coreprops struct.
 */
int vxd_get_coreproperties(void *hndl_coreproperties,
			   struct vxd_coreprops *vxd_coreprops);

/*
 * translates picture attributes to the form of the VXD_sPictAttrs struct.
 */
int vxd_get_pictattrs(u32 flags, struct vxd_pict_attrs *pict_attrs);

/*
 * translates message attributes to the form of the VXD_eMsgAttr struct.
 */
int vxd_get_msgerrattr(u32 flags, enum vxd_msg_attr *msg_attr);

/*
 * sets a message flag.
 */
int vxd_set_msgflag(enum vxd_msg_flag input_flag, u32 *flags);

#endif /* _VXD_INT_H */
