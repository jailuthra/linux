/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VXD Decoder device driver header definitions
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

#ifndef __VDECDD_DEFS_H__
#define __VDECDD_DEFS_H__

#include "bspp.h"
#include "lst.h"
#include "vdec_defs.h"
#include "vdecfw_shared.h"
#include "vxd_buf.h"
#include "vxd_mmu_defs.h"

/* RMAN type for streams.          */
#define VDECDD_STREAM_TYPE_ID	(0xB0B00001)

/* RMAN type for buffer mappings.  */
#define VDECDD_BUFMAP_TYPE_ID	(0xB0B00002)

/*
 * This type contains core feature flags.
 * @brief  Core Feature Flags
 */
enum vdecdd_core_feature_flags {
	VDECDD_COREFEATURE_MPEG2	= (1 << 0),
	VDECDD_COREFEATURE_MPEG4	= (1 << 1),
	VDECDD_COREFEATURE_H264		= (1 << 2),
	VDECDD_COREFEATURE_VC1		= (1 << 3),
	VDECDD_COREFEATURE_AVS		= (1 << 4),
	VDECDD_COREFEATURE_REAL		= (1 << 5),
	VDECDD_COREFEATURE_JPEG		= (1 << 6),
	VDECDD_COREFEATURE_VP6		= (1 << 7),
	VDECDD_COREFEATURE_VP8		= (1 << 8),
	VDECDD_COREFEATURE_HEVC		= (1 << 9),
	VDECDD_COREFEATURE_HD_DECODE	= (1 << 10),
	VDECDD_COREFEATURE_ROTATION	= (1 << 11),
	VDECDD_COREFEATURE_SCALING	= (1 << 12)
};

/*
 * This structure contains configuration relating to a device.
 * @brief  Device Configuration
 */
struct vdecdd_dd_devconfig {
	u32	num_slots_per_pipe;
};

/*
 * This structure contains the Decoder decoding picture status.
 * @brief Decoder Decoding Picture Status
 */
struct vdecdd_dec_pict_status {
	u32	transaction_id;
	enum vdecfw_progresscheckpoint	fw_cp;
	enum vdecfw_progresscheckpoint	fehw_cp;
	enum vdecfw_progresscheckpoint	behw_cp;
	u32	dmac_status;
	u32	fe_mb_x;
	u32	fe_mb_y;
	u32	be_mb_x;
	u32	be_mb_y;
	u8	fw_control_msg[VDECFW_MSGID_CONTROL_TYPES];
	u8	fw_decode_msg[VDECFW_MSGID_DECODE_TYPES];
	u8	fw_completion_msg[VDECFW_MSGID_COMPLETION_TYPES];
	u8	host_control_msg[VDECFW_MSGID_CONTROL_TYPES];
	u8	host_decode_msg[VDECFW_MSGID_DECODE_TYPES];
	u8	host_completion_msg[VDECFW_MSGID_COMPLETION_TYPES];
};

/*
 * This structure contains the Decoder decoding picture status.
 * @brief  Core Status
 */
struct vdecdd_core_status {
	u32	mtx_pc;
	u32	mtx_pcx;
	u32	mtx_enable;
	u32	mtx_st_bits;
	u32	mtx_fault0;
	u32	mtx_a0_stack_ptr;
	u32	mtx_a0_frame_ptr;
	u32	dma_setup[3];
	u32	dma_count[3];
	u32	dma_peripheral_addr[3];
};

/*
 * This structure contains the Decoder component stream status.
 * @brief   Decoder Component Stream Status
 */
struct vdecdd_decstr_status {
	u32	num_pict_decoding;
	struct vdecdd_dec_pict_status	dec_pict_st[VDECFW_MAX_NUM_PICTURES];
	u32	num_pict_decoded;
	u32	decoded_picts[VDECFW_MAX_NUM_PICTURES];
	u32	features;
	struct vdecdd_core_status	core_st;
	u32	display_pics;
	u32	release_pics;
	u32	next_display_items[VDECFW_MAX_NUM_PICTURES];
	u32	next_display_item_parent[VDECFW_MAX_NUM_PICTURES];
	u32	next_release_items[VDECFW_MAX_NUM_PICTURES];
	u32	next_release_item_parent[VDECFW_MAX_NUM_PICTURES];
	u32	flds_as_frm_decodes;
	u32	total_pict_decoded;
	u32	total_pict_displayed;
	u32	total_pict_finished;
};

/*
 * This structure contains the device context.
 * @brief  VDECDD Device Context
 */
struct vdecdd_dddev_context {
	void	*dev_handle;
	void	*dec_context;
	u32	internal_heap_id;
	void	*res_buck_handle;
};

/*
 * This type defines the stream state.
 * @brief  VDEDD Stream State
 */
enum vdecdd_ddstr_state {
	VDECDD_STRSTATE_STOPPED = 0x00,
	VDECDD_STRSTATE_PLAYING,
	VDECDD_STRSTATE_STOPPING,
	VDECDD_STRSTATE_MAX
};

/*
 * This structure contains the mapped output buffer configuration.
 * @brief  VDECDD Mapped Output Buffer Configuration
 */
struct vdecdd_mapbuf_info {
	u32	buf_size;
	u32	num_buf;
	u8	byte_interleave;
};

struct vdecdd_ddstr_ctx;

/*
 * This structure contains the map info.
 * @brief  VDECDD Map Info
 */
struct vdecdd_ddbuf_mapinfo {
	void	**link; /* to be part of single linked list */
	void	*res_handle;
	u32	buf_id;
	u32	buf_map_id;
	struct vdecdd_ddstr_ctx	*ddstr_context;
	void			*buf_cb_param;
	enum vdec_buf_type	buf_type;
	enum mmu_eheap_id	mmuheap_id;
	struct vxdio_ddbufinfo	ddbuf_info;
};

/*
 * This structure contains the information about the picture buffer
 * and its structure.
 * @brief  VDECDD Picture Buffer Info
 */
struct vdecdd_ddpict_buf {
	struct vdecdd_ddbuf_mapinfo	*pict_buf;
	struct vdec_pict_rendinfo	rend_info;
	struct vdec_pict_bufconfig	buf_config;
};

/*
 * This structure contains the stream context.
 * @brief  VDECDD Stream Context
 */
struct vdecdd_ddstr_ctx {
	void	**link; /* to be part of single linked list */
	u32	res_str_id;
	u32	km_str_id;
	void	*res_buck_handle;
	void	*res_handle;
	struct vdecdd_dddev_context	*dd_dev_context;
	struct vdec_str_configdata	str_config_data;
	u8	preempt;
	enum vdecdd_ddstr_state	dd_str_state;
	void			*mmu_str_handle;
	void			*dec_ctx;
	enum vdec_play_mode	play_mode;
	struct vdec_comsequ_hdrinfo	comseq_hdr_info;
	struct vdecdd_ddpict_buf	disp_pict_buf;
	struct vdecdd_mapbuf_info	map_buf_info;
	struct vdec_str_opconfig	opconfig;
	u8				str_op_configured;
	struct vdec_comsequ_hdrinfo	prev_comseq_hdr_info;
	struct bspp_pict_hdr_info	prev_pict_hdr_info;
};

/*
 * This type defines the stream unit type.
 * @brief  Stream Unit Type
 */
enum vdecdd_str_unit_type {
	VDECDD_STRUNIT_ANONYMOUS = 0,
	VDECDD_STRUNIT_SEQUENCE_START,
	VDECDD_STRUNIT_CLOSED_GOP,
	VDECDD_STRUNIT_SEQUENCE_END,
	VDECDD_STRUNIT_PICTURE_PORTENT,
	VDECDD_STRUNIT_PICTURE_START,
	VDECDD_STRUNIT_PICTURE_FRAGMENT,
	VDECDD_STRUNIT_PICTURE_END,
	VDECDD_STRUNIT_FENCE,
	VDECDD_STRUNIT_STOP,
	VDECDD_STRUNIT_MAX
};

/*
 * This structure contains a front end stream unit.
 * @brief   Front End Stream Unit
 */
struct vdecdd_str_unit {
	void				*lst_padding;
	enum vdecdd_str_unit_type	str_unit_type;
	void				*str_unit_handle;
	u32				err_flags;
	struct lst_t			bstr_seg_list;
	void				*dd_data;
	struct bspp_sequ_hdr_info	*seq_hdr_info;
	u32				seq_hdr_id;
	u8				closed_gop;
	u8				eop;
	struct bspp_pict_hdr_info	*pict_hdr_info;
	void				*dd_pict_data;
	u8				last_pict_in_seq;
	void				*str_unit_tag;
	u8				decode;
	u32				features;
	u8				eos;
};

/*
 * This structure contains a set of picture resources required at all the
 * interim stages of decoding it as it flows around the internals. It originates
 * in the plant.
 * @brief   Picture Resources (stream)
 */
struct vdecdd_pict_resint {
	void	**link; /* to be part of single linked list */
	struct vdecdd_ddbuf_mapinfo	*mb_param_buf;
	u32				ref_cnt;

#ifdef HAS_HEVC
	/* GENC fragment buffer */
	struct vdecdd_ddbuf_mapinfo	*genc_fragment_buf;
#endif
#if defined(HAS_HEVC) || defined(ERROR_CONCEALMENT)
	/* Sequence resources (GENC buffers) */
	struct vdecdd_seq_resint	*seq_resint;
#endif
};

/*
 * This structure contains the supplementary information for decoded picture.
 * @brief   Event Callback Information
 */
struct vdecdd_pict_sup_data {
	void	*raw_vui_data;
	void	*raw_sei_list_first_fld;
	void	*raw_sei_list_second_fld;
	u8	merged_flds;
	union {
		struct vdecdd_h264_pict_supl_data {
			u8	nal_ref_idc;
			u16	frame_num;
		} h264_pict_supl_data;
#ifdef HAS_HEVC
		struct vdecdd_hevc_pict_supl_data {
			u32 pic_order_cnt;
		} hevc_pict_supl_data;
#endif
	};
};

/*
 * This structure contains a set of resources representing a picture
 * at all the stages of processing it.
 * @brief   Picture
 */
struct vdecdd_picture {
	u32				pict_id;
	u8				last_pict_in_seq;
	struct vdecdd_pict_resint	*pict_res_int;
	struct vdecdd_ddpict_buf	disp_pict_buf;
	struct vdec_str_opconfig	op_config;
	struct vdec_dec_pict_info	*dec_pict_info;
	struct vdecdd_pict_sup_data	dec_pict_sup_data;
	struct vdec_dec_pict_auxinfo	dec_pict_aux_info;
};

/*
 * This structure contains the information required to check Decoder support
 * Only pointers that are non-null will be used in validation.
 * @brief  VDECDD Support Check Information
 */
struct vdecdd_supp_check {
	/* Inputs */
	const struct vdec_comsequ_hdrinfo	*comseq_hdrinfo;
	const struct vdec_str_opconfig		*op_cfg;
	const struct vdecdd_ddpict_buf		*disp_pictbuf;
	const struct bspp_pict_hdr_info		*pict_hdrinfo;
	u8					non_cfg_req;

	/* Outputs */
	struct vdec_unsupp_flags		unsupp_flags;
	u32					features;
};

/*
 * This type defines unsupported stream configuration features.
 * @brief  Unsupported Stream Configuration Flags
 */
enum vdec_unsupp_strcfg {
	VDECDD_UNSUPPORTED_STRCONFIG_STD	= (1 << 0),
	VDECDD_UNSUPPORTED_STRCONFIG_BSTRFORMAT	= (1 << 1)
};

/*
 * This type defines unsupported output configuration features.
 * @brief  Unsupported Output Configuration Flags
 */
enum vdec_unsupp_opcfg {
	VDECDD_UNSUPPORTED_OUTPUTCONFIG_ROTATION		= (1 << 0),
	VDECDD_UNSUPPORTED_OUTPUTCONFIG_ROTATION_WITH_FIELDS	= (1 << 1),
	VDECDD_UNSUPPORTED_OUTPUTCONFIG_ROTATION_WITH_SCALING	= (1 << 2),
	VDECDD_UNSUPPORTED_OUTPUTCONFIG_SCALING			= (1 << 3),
	VDECDD_UNSUPPORTED_OUTPUTCONFIG_UP_DOWNSAMPLING		= (1 << 4),
	VDECDD_UNSUPPORTED_OUTPUTCONFIG_SCALING_WITH_OOLD	= (1 << 5),
	VDECDD_UNSUPPORTED_OUTPUTCONFIG_SCALING_MONOCHROME	= (1 << 6),
	VDECDD_UNSUPPORTED_OUTPUTCONFIG_SCALING_SIZE		= (1 << 7),
	VDECDD_UNSUPPORTED_OUTPUTCONFIG_X_WITH_JPEG		= (1 << 8),
	VDECDD_UNSUPPORTED_OUTPUTCONFIG_SCALESIZE		= (1 << 9),
	VDECDD_UNSUPPORTED_OUTPUTCONFIG_PIXFORMAT		= (1 << 10),
	VDECDD_UNSUPPORTED_OUTPUTCONFIG_ROTATION_WITH_HIGH_COLOUR	=
								(1 << 11),
	VDECDD_UNSUPPORTED_OUTPUTCONFIG_DOWNSAMPLING_WITH_ROTATION	=
								(1 << 12),
	VDECDD_UNSUPPORTED_OUTPUTCONFIG_ROTATION_WITH_10BIT_PACKED	=
								(1 << 13),
};

/*
 * This type defines unsupported output configuration features.
 * @brief  Unsupported Output Configuration Flags
 */
enum vdec_unsupp_op_bufcfg {
	VDECDD_UNSUPPORTED_OUTPUTBUFCONFIG_EXTENDED_STRIDE	= (1 << 0),
	VDECDD_UNSUPPORTED_OUTPUTBUFCONFIG_64BYTE_STRIDE	= (1 << 1),
	VDECDD_UNSUPPORTED_OUTPUTBUFCONFIG_FIXED_STRIDE		= (1 << 2),
	VDECDD_UNSUPPORTED_OUTPUTBUFCONFIG_PICTURE_SIZE		= (1 << 3),
	VDECDD_UNSUPPORTED_OUTPUTBUFCONFIG_BUFFER_SIZE		= (1 << 4),
	VDECDD_UNSUPPORTED_OUTPUTBUFCONFIG_Y_SIZE		= (1 << 5),
	VDECDD_UNSUPPORTED_OUTPUTBUFCONFIG_UV_SIZE		= (1 << 6),
	VDECDD_UNSUPPORTED_OUTPUTBUFCONFIG_Y_STRIDE		= (1 << 7),
	VDECDD_UNSUPPORTED_OUTPUTBUFCONFIG_UV_STRIDE		= (1 << 8),
};

/*
 * This type defines unsupported sequence header features.
 * @brief  Unsupported Sequence Header Flags
 */
enum vdec_unsupp_seqhdr {
	VDECDD_UNSUPPORTED_SEQUHDR_PIXFORMAT_BIT_DEPTH	= (1 << 0),
	VDECDD_UNSUPPORTED_SEQUHDR_SCALING		= (1 << 1),
	VDECDD_UNSUPPORTED_SEQUHDR_PIXEL_FORMAT		= (1 << 2),
	VDECDD_UNSUPPORTED_SEQUHDR_NUM_OF_VIEWS		= (1 << 3),
	VDECDD_UNSUPPORTED_SEQUHDR_CODED_HEIGHT		= (1 << 4),
	VDECDD_UNSUPPORTED_SEQUHDR_SEP_COLOUR_PLANE	= (1 << 5),
	VDECDD_UNSUPPORTED_SEQUHDR_SIZE			= (1 << 6),
};

/*
 * This type defines unsupported picture header features.
 * @brief  Unsupported Picture Header Flags
 */
enum vdec_unsupp_picthdr {
	VDECDD_UNSUPPORTED_PICTHDR_UPSCALING		= (1 << 0),
	VDECDD_UNSUPPORTED_PICTHDR_OVERSIZED_SGM	= (1 << 1),
	VDECDD_UNSUPPORTED_PICTHDR_DISCONTINUOUS_MBS	= (1 << 2),
	VDECDD_UNSUPPORTED_PICTHDR_RESOLUTION		= (1 << 3),
	VDECDD_UNSUPPORTED_PICTHDR_SCALING_ORIGINALSIZE	= (1 << 4),
	VDECDD_UNSUPPORTED_PICTHDR_SCALING_SIZE		= (1 << 5),
	VDECDD_UNSUPPORTED_PICTHDR_HEVC_RANGE_EXT	= (1 << 6),
};

/*
 * This type defines the Bitstream Segment type.
 * @brief  Bitstream segment type
 */
enum vdecdd_bstr_segtype {
	VDECDD_BSSEG_LASTINBUFF		= (1 << 0),
	VDECDD_BSSEG_SKIP		= (1 << 1),
	VDECDD_BSSEG_INSERTSCP		= (1 << 2),
	VDECDD_BSSEG_INSERT_STARTCODE	= (1 << 3) | VDECDD_BSSEG_INSERTSCP,
};

struct vdecdd_seq_resint {
	void **link;

#ifdef HAS_HEVC
	u16				genc_buf_id;
	struct vdecdd_ddbuf_mapinfo	*genc_buffers[4];    /* GENC buffers */
	struct vdecdd_ddbuf_mapinfo	*intra_buffer;    /* GENC buffers */
	struct vdecdd_ddbuf_mapinfo	*aux_buffer;    /* GENC buffers */
#endif
	struct vdecdd_ddbuf_mapinfo *err_pict_buf; /* Pointer to "Error Recovery Frame Store" buffer. */

	/*
	 * Ref. counter (number of users) of sequence resources
	 * NOTE: Internal buffer reference counters are not used
	 * for buffers allocated as sequence resources.
	 */
	u32 ref_count;
};

#endif
