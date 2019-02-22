/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VXD Decoder Component header
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

#ifndef __DECODER_H__
#define __DECODER_H__

#include "bspp.h"
#include "dq.h"
#include "lst.h"
#include "vdecdd_defs.h"
#include "vdec_defs.h"
#include "vxd_buf.h"
#include "vxd_ext.h"
#include "vxd_props.h"
#include "hevcfw_data.h"

#define MAX_CONCURRENT_STREAMS 16

enum dec_pict_states {
	DECODER_PICTURE_STATE_TO_DECODE = 0,
	DECODER_PICTURE_STATE_DECODED,
	DECODER_PICTURE_STATE_TO_DISCARD,
	DECODER_PICTURE_STATE_MAX,
};

enum dec_res_type {
	DECODER_RESTYPE_TRANSACTION = 0,
	DECODER_RESTYPE_HDR,
	DECODER_RESTYPE_BATCH_MSG,
#ifdef HAS_HEVC
	DECODER_RESTYPE_PVDEC_BUF,
#endif
	DECODER_RESTYPE_MAX,
};

enum dec_core_query_type {
	DECODER_CORE_GET_RES_LIMIT = 0,
};

/*
 * @Function              pfnRefPicGetMaxNum
 * @Description
 * This is the prototype for functions calculating the maximum number
 * of reference pictures required per video standard.
 *
 * @Input    psComSequHdrInfo  : A pointer to the common VSH information
 * structure.
 *
 * @Output   pui32MaxRefPicNum :  A pointer used to return the maximum number
 *                               of reference frames required.
 *
 * @Return   IMG_RESULT : This function returns either IMG_SUCCESS or
 * an error code.
 */
typedef int32_t (*ref_pic_get_maximum)
	(const struct vdec_comsequ_hdrinfo *comseq_hdr_info,
	 u32 *max_ref_pict_num);

typedef int (*strunit_processed_cb)(void *handle, int cb_type, void *item);

typedef int (*core_gen_cb)(void *handle, int query, void *item);

struct dec_ctx;

/*
 * This structure contains the core context.
 * @brief  Decoder Core Context
 */
struct dec_core_ctx {
	void		**link; /* to be part of single linked list */
	struct dec_ctx	*dec_ctx;
	u8		enumerated;
	u8		master;
	u8		configured;
	u32		core_features;
	u32		pipe_features[VDEC_MAX_PIXEL_PIPES];
	struct vxd_coreprops	core_props;
	void			*resources;
	void			*hw_ctx;
	u32			cum_pics;
	u8			busy;
};

struct dec_ctx {
	u8		inited;
	void		*user_data;
	const struct vdecdd_dd_devconfig	*dev_cfg;
	u32				num_pipes;
	struct dec_core_ctx		*dec_core_ctx;
	struct lst_t			str_list;
	void				*mmu_dev_handle;
	void				*dev_handle;
	struct vxdio_ddbufinfo		ptd_buf_info;
	u8				sup_stds[VDEC_STD_MAX];
	u32				internal_heap_id;
	u32				str_cnt;
};

/*
 * This structure contains the device decode resource (used for decoding and
 * held for subsequent decoding).
 * @brief  Decoder Device Resource
 */
struct dec_pictdec_res {
	void	**link; /* to be part of single linked list */
	u32	transaction_id;
	struct vxdio_ddbufinfo	fw_ctx_buf;
	struct vxdio_ddbufinfo	h264_sgm_buf;
	u32			ref_cnt;
};

struct dec_decpict;

/*
 *
 * This structure contains the stream context.
 * @brief  Decoder Stream Context
 */
struct dec_str_ctx {
	void	**link; /* to be part of single linked list */
	int	km_str_id;
	struct vdec_str_configdata	config;
	struct dec_ctx			*decctx;
	void				*vxd_dec_ctx;
	void				*usr_int_data;
	void				*mmu_str_handle;
	void				*pict_idgen;
	struct lst_t			pend_strunit_list;
	struct dq_linkage_t		str_decd_pict_list;
	u32				num_ref_res;
	struct lst_t			ref_res_lst;
	u32				num_dec_res;
	struct lst_t			dec_res_lst;
	u32				avail_pipes;
	u32				avail_slots;
	struct vdecdd_decstr_status	dec_str_st;
	struct vxdio_ddbufinfo		pvdec_fw_ctx_buf;
	u32				last_fe_transaction_id;
	u32				next_dec_pict_id;
	u32				next_pict_id_expected;
	struct dec_pictdec_res		*cur_fe_pict_dec_res;
	struct dec_pictdec_res		*prev_fe_pict_dec_res;
	struct dec_pictdec_res		*last_be_pict_dec_res;
	struct dec_decpict		*cur_pict;
	void				*resources;
	strunit_processed_cb		str_processed_cb;
	core_gen_cb			core_query_cb;
};

/*
 * Resource Structure for DECODER_sDdResourceInfo to be used with pools
 */
struct res_resinfo {
	void			**link; /* to be part of single linked list */
	void			*res;
	struct vxdio_ddbufinfo	*ddbuf_info;
};

struct vdecdd_ddstr_ctx;

/*
 * This structure contains the Decoded attributes
 * @brief Decoded attributes
 */
struct dec_pict_attrs {
	u8		first_fld_rcvd;
	u32		fe_err;
	u32		no_be_wdt;
	u32		mbs_dropped;
	u32		mbs_recovered;
	struct vxd_pict_attrs	pict_attrs;
};

/*
 * This union contains firmware contexts. Used to allocate buffers for firmware
 * context.
 */
union dec_fw_contexts {
	struct h264fw_context_data	h264_context;
#ifdef HAS_HEVC
	struct hevcfw_ctx_data	hevc_context;
#endif
};

/*
 * for debug
 */
struct dec_fwmsg {
	void			**link;
	struct dec_pict_attrs	pict_attrs;
	struct vdec_pict_hwcrc	pict_hwcrc;
};

/*
 * This structure contains the stream decode resource (persistent for
 * longer than decoding).
 * @brief  Decoder Stream Resource
 */
struct dec_pictref_res {
	void			**link; /* to be part of single linked list */
	struct vxdio_ddbufinfo	fw_ctrlbuf;
	u32			ref_cnt;
};

/*
 * This structure defines the decode picture.
 * @brief  Decoder Picture
 */
struct dec_decpict {
	void				**link;
	u32				transaction_id;
	void				*dec_str_ctx;
	u8				twopass;
	u8				first_fld_rcvd;
	struct res_resinfo		*transaction_info;
	struct res_resinfo		*hdr_info;
#ifdef HAS_HEVC
	struct res_resinfo		*pvdec_info;
	u32				temporal_out_addr;
#endif
	struct vdecdd_ddpict_buf	*recon_pict;
	struct vdecdd_ddpict_buf	*alt_pict;
	struct res_resinfo		*batch_msginfo;
	struct vxdio_ddbufinfo		*intra_bufinfo;
	struct vxdio_ddbufinfo		*auxline_bufinfo;
	struct vxdio_ddbufinfo		*vlc_tables_bufinfo;
	struct vxdio_ddbufinfo		*vlc_idx_tables_bufinfo;
	struct vxdio_ddbufinfo		*start_code_bufinfo;
	struct dec_fwmsg		*first_fld_fwmsg;
	struct dec_fwmsg		*second_fld_fwmsg;
	struct bspp_pict_hdr_info	*pict_hdr_info;
	struct dec_pictdec_res		*cur_pict_dec_res;
	struct dec_pictdec_res		*prev_pict_dec_res;
	struct dec_pictref_res		*pict_ref_res;
	struct lst_t			dec_pict_seg_list;
	struct lst_t			fragment_list;
	u8				eop_found;
	u32				operating_op;
	u16				genc_id;
	struct vdecdd_ddbuf_mapinfo	**genc_bufs;
	struct vdecdd_ddbuf_mapinfo	*genc_fragment_buf;
	u32				ctrl_alloc_bytes;
	u32				ctrl_alloc_offset;
	enum dec_pict_states		state;
	struct vxdio_ddbufinfo		*str_pvdec_fw_ctxbuf;
};

/*
 *
 * This structure defines the decode picture reference.
 * @brief  Decoder Picture Reference
 */
struct dec_str_unit {
	void			**link; /* to be part of single linked list */
	struct dec_decpict	*dec_pict;
	struct vdecdd_str_unit	*str_unit;
};

/*
 * This structure defines the decoded picture.
 * @brief  Decoded Picture
 */
struct dec_decoded_pict {
	struct dq_linkage_t	link; /* to be part of double linked list */
	u32			transaction_id;
	u8			processed;
	u8			process_failed;
	u8			force_display;
	u8			displayed;
	u8			merged;
	u32			disp_idx;
	u32			rel_idx;
	struct vdecdd_picture	*pict;
	struct dec_fwmsg	*first_fld_fwmsg;
	struct dec_fwmsg	*second_fld_fwmsg;
	struct dec_pictref_res	*pict_ref_res;
};

struct dec_pict_fragment {
	void	**link; /* to be part of single linked list */
	/* Control allocation size in bytes */
	u32	ctrl_alloc_bytes;
	/* Control allocation offset in bytes */
	u32	ctrl_alloc_offset;
};

/*
 * This structure contains the pointer to the picture segment.
 * All the segments could be added to the list in struct dec_decpict,
 * but because list items cannot belong to more than one list this wrapper
 * is used which is added in the list sDecPictSegList inside struct dec_decpict
 * @brief  Decoder Picture Segment
 */
struct dec_decpict_seg {
	void			**link; /* to be part of single linked list */
	struct bspp_bitstr_seg	*bstr_seg;
	u8			internal_seg;
};

struct decoder_regsoffsets {
	u32	vdmc_cmd_offset;
	u32	vec_offset;
	u32	entropy_offset;
	u32	vec_be_regs_offset;
	u32	vdec_be_codec_regs_offset;
};

int decoder_initialise(void *init_usr_data, u32 internal_heap_id,
		       struct vdecdd_dd_devconfig *dd_devcfg, u32 *num_pipes,
		       void **dec_ctx);

int decoder_deinitialise(void *dec_ctx);

int decoder_supported_features(void *dec_ctx, struct vdec_features *features);

int decoder_stream_destroy(void *dec_str_ctx, u8  abort);

int decoder_stream_create(void *dec_ctx, struct vdec_str_configdata str_cfg,
			  u32 kmstr_id, void **mmu_str_handle,
			  void *vxd_dec_ctx, void *str_usr_int_data,
			  void **dec_str_ctx, void *decoder_cb, void *query_cb);

int decoder_stream_prepare_ctx(void *dec_str_ctx, u8 flush_dpb);

int decoder_stream_process_unit(void *dec_str_ctx,
				struct vdecdd_str_unit *str_unit);

int decoder_get_load(void *dec_str_ctx, u32 *avail_slots);

int
decoder_check_support(void *dec_ctx,
		      const struct vdec_str_configdata *str_cfg,
		      const struct vdec_str_opconfig *op_cfg,
		      const struct vdecdd_ddpict_buf *disp_pictbuf,
		      const struct vdec_pict_rendinfo *req_pict_rendinfo,
		      const struct vdec_comsequ_hdrinfo *comseq_hdrinfo,
		      const struct bspp_pict_hdr_info *pict_hdrinfo,
		      const struct vdec_comsequ_hdrinfo *prev_comseq_hdrinfo,
		      const struct bspp_pict_hdr_info *prev_pict_hdrinfo,
		      u8 non_cfg_req, struct vdec_unsupp_flags *unsupported,
		      u32 *features);

u8 decoder_is_stream_idle(void *dec_str_ctx);

int decoder_stream_flush(void *dec_str_ctx, u8 discard_refs);

int decoder_stream_release_buffers(void *dec_str_ctx);

int decoder_stream_get_status(void *dec_str_ctx,
			      struct vdecdd_decstr_status *dec_str_st);

int decoder_service_firmware_response(void *dec_str_ctx_arg, u32 *msg,
				      u32 msg_size, u32 msg_flags);

#endif
