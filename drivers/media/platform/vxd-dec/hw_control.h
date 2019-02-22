/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VXD DEC Hardware control implementation
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

#ifndef _HW_CONTROL_H
#define _HW_CONTROL_H

#include "bspp.h"
#include "decoder.h"
#include "fw_interface.h"
#include "img_dec_common.h"
#include "img_errors.h"
#include "lst.h"
#include "mem_io.h"
#include "vdecdd_defs.h"
#include "vdecfw_shared.h"
#include "vxd_buf.h"
#include "vxd_ext.h"
#include "vxd_props.h"

/* Size of additional buffers needed for each HEVC picture */
#ifdef HAS_HEVC

/* Empirically defined */
#define MEM_TO_REG_BUF_SIZE 0x2000

/*
 * Max. no. of slices found in stream db: approx. 2200,
 * set MAX_SLICES to 2368 to get buffer size page aligned
 */
#define MAX_SLICES 2368
#define SLICE_PARAMS_SIZE 64
#define SLICE_PARAMS_BUF_SIZE (MAX_SLICES * SLICE_PARAMS_SIZE)

/*
 * Size of buffer for "above params" structure, sufficient for stream of width 8192
 * 192 * (8192/64) == 0x6000, see "above_param_size" in TRM
 */
#define ABOVE_PARAMS_BUF_SIZE 0x6000
#endif

enum hwctrl_msgid {
	HWCTRL_MSGID_BATCH = 0,
	HWCTRL_MSGID_FRAGMENT = 1,
	CORE_MSGID_MAX,
};

struct hwctrl_to_kernel_msg {
	u32 msg_size;
	u32 km_str_id;
	u32 flags;
	u8 *msg_hdr;
};

struct hwctrl_batch_msgdata {
	struct vxdio_ddbufinfo *batchmsg_bufinfo;
	struct vxdio_ddbufinfo *pvdec_fwctx;
	u32 ctrl_alloc_bytes;
	u32 operating_mode;
	u32 transaction_id;
	u32 tile_cfg;
	u32 genc_id;
	u32 mb_load;
	u32 size_delimited_mode;
};

struct hwctrl_fragment_msgdata {
	struct vxdio_ddbufinfo *batchmsg_bufinfo;
	u32 ctrl_alloc_offset;
	u32 ctrl_alloc_bytes;
};

struct hwctrl_msgdata {
	u32 km_str_id;
	struct hwctrl_batch_msgdata batch_msgdata;
	struct hwctrl_fragment_msgdata fragment_msgdata;
};

/*
 * This structure contains MSVDX Message information.
 */
struct hwctrl_msgstatus {
	u8 control_fence_id[VDECFW_MSGID_CONTROL_TYPES];
	u8 decode_fence_id[VDECFW_MSGID_DECODE_TYPES];
	u8 completion_fence_id[VDECFW_MSGID_COMPLETION_TYPES];
};

/*
 * this structure contains the HWCTRL Core state.
 */
struct hwctrl_state {
	struct vxd_states core_state;
	struct hwctrl_msgstatus fwmsg_status;
	struct hwctrl_msgstatus hostmsg_status;
};

int hwctrl_picture_submit_fragment(void *hndl_hwctx,
				   struct dec_pict_fragment  *pict_fragment,
				   struct dec_decpict *decpict,
				   void *vxd_dec_ctx);

int hwctrl_process_msg(void *hndl_hwct, u32 msg_flags, u32 *msg,
		       struct dec_decpict **decpict);

int hwctrl_getcore_cached_status(void *hndl_hwctx, struct hwctrl_state *state);

int hwctrl_get_core_status(void *hndl_hwctx, struct hwctrl_state *state);

int hwctrl_is_on_seq_replay(void *hndl_hwctx);

int hwctrl_picture_submitbatch(void *hndl_hwctx, struct dec_decpict  *decpict,
			       void *vxd_dec_ctx);

int hwctrl_getpicpend_pictlist(void *hndl_hwctx, u32 transaction_id,
			       struct dec_decpict  **decpict);

int hwctrl_peekheadpiclist(void *hndl_hwctx, struct dec_decpict **decpict);

int hwctrl_getdecodedpicture(void *hndl_hwctx, struct dec_decpict **decpict);

int hwctrl_removefrom_piclist(void *hndl_hwctx, struct dec_decpict  *decpict);

int hwctrl_getregsoffset(void *hndl_hwctx,
			 struct decoder_regsoffsets *regs_offsets);

int hwctrl_initialise(void *dec_core, void *comp_int_userdata,
		      const struct vdecdd_dd_devconfig  *dd_devconfig,
		      struct vxd_coreprops *core_props, void **hndl_hwctx);

int hwctrl_deinitialise(void *hndl_hwctx);

#endif /* _HW_CONTROL_H */
