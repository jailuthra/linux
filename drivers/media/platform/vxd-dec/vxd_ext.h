/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VXD DEC Low-level device interface component
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

#ifndef _VXD_EXT_H
#define _VXD_EXT_H

#define VLR_COMPLETION_COMMS_AREA_SIZE             476

/* Word Size of buffer used to pass messages between LISR and HISR */
#define VXD_SIZE_MSG_BUFFER   (1 * 1024)

/* This structure describes macroblock coordinates. */
struct vxd_mb_coords {
	u32 x;
	u32 y;
};

/* This structure contains firmware and decoding pipe state information. */
struct vxd_pipestate {
	u8 is_pipe_present;
	u8 cur_codec;
	u32 acheck_point[VDECFW_CHECKPOINT_MAX];
	u32 firmware_action;
	u32 fe_slices;
	u32 be_slices;
	u32 fe_errored_slices;
	u32 be_errored_slices;
	u32 be_mbs_dropped;
	u32 be_mbs_recovered;
	struct vxd_mb_coords fe_mb;
	struct vxd_mb_coords be_mb;
};

/* This structure contains firmware and decoder core state information. */
struct vxd_firmware_state {
	u32 fw_step;
	struct vxd_pipestate pipe_state[VDECFW_MAX_DP];
};

/* This structure contains the video decoder device state. */
struct vxd_states {
	struct vxd_firmware_state fw_state;
};

struct vxd_pict_attrs {
	u32 dwrfired;
	u32 mmufault;
	u32 deverror;
};

/* This type defines the message attributes. */
enum vxd_msg_attr {
	VXD_MSG_ATTR_NONE     = 0,
	VXD_MSG_ATTR_DECODED  = 1,
	VXD_MSG_ATTR_FATAL    = 2,
	VXD_MSG_ATTR_CANCELED = 3,
};

enum vxd_msg_flag {
	VXD_MSG_FLAG_DROP     = 0,
	VXD_MSG_FLAG_EXCL     = 1,
};

#endif /* VXD_EXT_H */
