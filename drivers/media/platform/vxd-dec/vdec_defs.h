/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VXD Decoder common header
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

#ifndef __VDEC_DEFS_H__
#define __VDEC_DEFS_H__

#include "img_mem.h"
#include "img_pixfmts.h"
#include "pixel_api.h"
#include "vdecfw_shared.h"

#define VDEC_MAX_PANSCAN_WINDOWS	4
#define VDEC_MB_DIMENSION		(16)

#define MAX_PICS_IN_SYSTEM	(64)
#define SEQUENCE_SLOTS		(64)
#define PPS_SLOTS		(64)
/* Only for HEVC */
#define VPS_SLOTS		(16)
#define MAX_VPSS		(MAX_PICS_IN_SYSTEM + VPS_SLOTS)
#define MAX_SEQUENCES		(MAX_PICS_IN_SYSTEM + SEQUENCE_SLOTS)
#define MAX_PPSS		(MAX_PICS_IN_SYSTEM + PPS_SLOTS)

#define VDEC_H264_MAXIMUMVALUEOFCPB_CNT	32
#define VDEC_H264_MAX_SLICE_GROUPMBS	65536
#define VDEC_H264_MVC_MAX_VIEWS		(H264FW_MAX_NUM_VIEWS)
#define VDEC_H264_MVC_MAX_REFS		(H264FW_MAX_NUM_MVC_REFS)
#define VDEC_H264_MVC_MAX_LEVELS	(16UL)
#define VDEC_H264_MVC_MAX_APP_OP_TID	(16UL)
#define VDEC_H264_MVC_MAX_TARGET_VIEW	(8UL)

#define VDEC_H264_MVC_REF_LIST_ANCHOR_L0	0
#define VDEC_H264_MVC_REF_LIST_ANCHOR_L1	1
#define VDEC_H264_MVC_REF_LIST_NON_ANCHOR_L0	0
#define VDEC_H264_MVC_REF_LIST_NON_ANCHOR_L1	1

#define VDEC_ASSERT(expected) ({WARN_ON(!(expected)); 0; })

/*
 * This type defines the video standard.
 * @brief  VDEC Video Standards
 */
enum vdec_vid_std {
	VDEC_STD_UNDEFINED = 0,
	VDEC_STD_MPEG2,
	VDEC_STD_MPEG4,
	VDEC_STD_H263,
	VDEC_STD_H264,
	VDEC_STD_VC1,
	VDEC_STD_AVS,
	VDEC_STD_REAL,
	VDEC_STD_JPEG,
	VDEC_STD_VP6,
	VDEC_STD_VP8,
	VDEC_STD_SORENSON,
	VDEC_STD_HEVC,
	VDEC_STD_MAX
};

/*
 * This type defines the bitstream format. Should be done at the
 * start of decoding.
 * @brief  VDEC Bitstream Format
 */
enum vdec_bstr_format {
	VDEC_BSTRFORMAT_UNDEFINED = 0,
	VDEC_BSTRFORMAT_ELEMENTARY,
	VDEC_BSTRFORMAT_DEMUX_BYTESTREAM,
	VDEC_BSTRFORMAT_DEMUX_SIZEDELIMITED,
	VDEC_BSTRFORMAT_MAX
};

/*
 * This type defines the Type of payload. Could change with every buffer.
 * @brief  VDEC Bitstream Element Type
 */
enum vdec_bstr_element_type {
	VDEC_BSTRELEMENT_UNDEFINED = 0,
	VDEC_BSTRELEMENT_UNSPECIFIED,
	VDEC_BSTRELEMENT_CODEC_CONFIG,
	VDEC_BSTRELEMENT_PICTURE_DATA,
	VDEC_BSTRELEMENT_MAX,
};

/*
 * This structure contains the stream configuration details.
 * @brief  VDEC Stream Configuration Information
 */
struct vdec_str_configdata {
	enum vdec_vid_std	vid_std;
	enum vdec_bstr_format	bstr_format;
	u32			user_str_id;
	u8			update_yuv;
	u8		bandwidth_efficient;
	u8		disable_mvc;
	u8		full_scan;
	u8		immediate_decode;
	u8		intra_frame_closed_gop;
};

/*
 * This type defines the buffer type categories.
 * @brief  Buffer Types
 */
enum vdec_buf_type {
	VDEC_BUFTYPE_BITSTREAM,
	VDEC_BUFTYPE_PICTURE,
	VDEC_BUFTYPE_ALL,
	VDEC_BUFTYPE_MAX
};

/*
 * This structure contains information related to a picture plane.
 * @brief  Picture Plane Information
 */
struct vdec_plane_info {
	u32	offset;
	u32	stride;
	u32	size;
};

/*
 * This structure describes the VDEC picture dimensions.
 * @brief  VDEC Picture Size
 */
struct vdec_pict_size {
	u32	width;
	u32	height;
};

/*
 * This enumeration defines the colour plane indices.
 * @brief  Colour Plane Indices
 */
enum vdec_color_planes {
	VDEC_PLANE_VIDEO_Y	= 0,
	VDEC_PLANE_VIDEO_YUV	= 0,
	VDEC_PLANE_VIDEO_U	= 1,
	VDEC_PLANE_VIDEO_UV	= 1,
	VDEC_PLANE_VIDEO_V	= 2,
	VDEC_PLANE_VIDEO_A	= 3,
	VDEC_PLANE_LIGHT_R	= 0,
	VDEC_PLANE_LIGHT_G	= 1,
	VDEC_PLANE_LIGHT_B	= 2,
	VDEC_PLANE_INK_C	= 0,
	VDEC_PLANE_INK_M	= 1,
	VDEC_PLANE_INK_Y	= 2,
	VDEC_PLANE_INK_K	= 3,
	VDEC_PLANE_MAX		= 4
};

/*
 * This structure describes the rendered region of a picture buffer (i.e. where
 * the image data is written.
 * @brief  Picture Buffer Render Information
 */
struct vdec_pict_rendinfo {
	u32			rendered_size;
	struct vdec_plane_info	plane_info[VDEC_PLANE_MAX];
	u32			stride_alignment;
	struct vdec_pict_size	rend_pict_size;
};

/*
 * This structure contains information required to configure the picture
 * buffers
 * @brief  Picture Buffer Configuration
 */
struct vdec_pict_bufconfig {
	u32	coded_width;
	u32	coded_height;
	enum img_pixfmt	pixel_fmt;
	u32		stride[IMG_MAX_NUM_PLANES];
	u32		stride_alignment;
	u8		byte_interleave;
	u32		buf_size;
	u8		packed;
	u32		chroma_offset[IMG_MAX_NUM_PLANES];
};

/*
 * This structure describes the VDEC Display Rectangle.
 * @brief  VDEC Display Rectangle
 */
struct vdec_rect {
	u32	top_offset;
	u32	left_offset;
	u32	width;
	u32	height;
};

/*
 * This structure contains the Color Space Description that may be present
 * in SequenceDisplayExtn(MPEG2), VUI parameters(H264), Visual Object(MPEG4)
 * for the application to use.
 * @brief  Stream Color Space Properties
 */
struct vdec_color_space_desc {
	u8	is_present;
	u8	color_primaries;
	u8	transfer_characteristics;
	u8	matrix_coefficients;
};

/*
 * This structure contains common (standard agnostic) sequence header
 * information, which is required for image buffer allocation and display.
 * @brief  Sequence Header Information (common)
 */
struct vdec_comsequ_hdrinfo {
	u32		codec_profile;
	u32		codec_level;
	u32		bitrate;
	long		frame_rate;
	u32		frame_rate_num;
	u32		frame_rate_den;
	u32		aspect_ratio_num;
	u32		aspect_ratio_den;
	u8		interlaced_frames;
	struct pixel_pixinfo	pixel_info;
	struct vdec_pict_size	max_frame_size;
	struct vdec_pict_size	frame_size;
	u8			field_codec_mblocks;
	u32			min_pict_buf_num;
	u8			picture_reordering;
	u8			post_processing;
	struct vdec_rect	orig_display_region;
	struct vdec_rect	raw_display_region;
	u32			num_views;
	u32			max_reorder_picts;
	u8			separate_chroma_planes;
	u8			not_dpb_flush;
	struct vdec_color_space_desc	color_space_info;
};

/*
 * This structure contains the standard specific codec configuration
 * @brief Codec configuration
 */
struct vdec_codec_config {
	u32	default_height;
	u32	default_width;
};

/*
 * This structure describes the decoded picture attributes (relative to the
 * encoded, where necessary, e.g. rotation angle).
 * @brief  Stream Output Configuration
 */
struct vdec_str_opconfig {
	struct pixel_pixinfo	pixel_info;
	u8			force_oold;
};

/*
 * This type defines the "play" mode.
 * @brief  Play Mode
 */
enum vdec_play_mode {
	VDEC_PLAYMODE_PARSE_ONLY,
	VDEC_PLAYMODE_NORMAL_DECODE,
	VDEC_PLAYMODE_MAX
};

/*
 * This type defines the bitstream processing error info.
 * @brief  Bitstream Processing Error Info
 */
struct vdec_bstr_err_info {
	u32	sequence_err;
	u32	picture_err;
	u32	other_err;
};

/*
 * This structure describes the VDEC Pan Scan Window.
 * @brief  VDEC Pan Scan Window
 */
struct vdec_window {
	u32	ui32topoffset;
	u32	ui32leftoffset;
	u32	ui32width;
	u32	ui32height;
};

/*
 * This structure contains the VDEC picture display properties.
 * @brief  VDEC Picture Display Properties
 */
struct vdec_pict_disp_info {
	struct vdec_rect	enc_disp_region;
	struct vdec_rect	disp_region;
	struct vdec_rect	raw_disp_region;
	u8			top_fld_first;
	u8			out_top_fld_first;
	u32			max_frm_repeat;
	u32			repeat_first_fld;
	u32			num_pan_scan_windows;
	struct vdec_window	pan_scan_windows[VDEC_MAX_PANSCAN_WINDOWS];
};

/*
 * This structure contains VXD hardware signatures.
 * @brief  VXD Hardware signatures
 */
struct vdec_pict_hwcrc {
	u8	first_fld_rcvd;
	u32	crc_vdmc_pix_recon;
	u32	vdeb_sysmem_wrdata;
};

struct vdec_features {
	u8	valid;
	u8	mpeg2;
	u8	mpeg4;
	u8	h264;
	u8	vc1;
	u8	avs;
	u8	real;
	u8	jpeg;
	u8	vp6;
	u8	vp8;
	u8	hevc;
	u8	hd;
	u8	rotation;
	u8	scaling;
	u8	scaling_oold;
	u8	scaling_extnd_strides;
};

/*
 * This type defines the auxiliary info for picture queued for decoding.
 * @brief  Auxiliary Decoding Picture Info
 */
struct vdec_dec_pict_auxinfo {
	u32	seq_hdr_id;
	u32	pps_id;
	u32	second_pps_id;
	u8	not_decoded;
};

/*
 * This type defines the decoded picture state.
 * @brief  Decoded Picture State
 */
enum vdec_pict_state {
	VDEC_PICT_STATE_NOT_DECODED,
	VDEC_PICT_STATE_DECODED,
	VDEC_PICT_STATE_TERMINATED,
	VDEC_PICT_STATE_MAX
};

/*
 * This type defines the container for various picture tags.
 * @brief  Picture Tag Container
 */
struct vdec_pict_tag_container {
	enum img_buffer_type		pict_type;
	u64				pict_tag_param;
	u64				sideband_info;
	struct vdec_pict_hwcrc		pict_hwcrc;
};

/*
 * This structure describes raw bitstream data chunk.
 * @brief  Raw Bitstream Data Chunk
 */
struct vdec_raw_bstr_data {
	u32				size;
	u32				bit_offset;
	u8				*data;
	struct vdec_raw_bstr_data	*next;
};

/*
 * This type defines the supplementary picture data.
 * @brief  Supplementary Picture Data
 */
struct vdec_pict_supl_data {
	struct vdec_raw_bstr_data	*raw_vui_data;
	struct vdec_raw_bstr_data	*raw_sei_list_first_fld;
	struct vdec_raw_bstr_data	*raw_sei_list_second_fld;
	union {
		struct h264_pict_supl_data {
			u8	nal_ref_idc;
			u16	frame_num;
		} data;
	};
};

/*
 * This structure contains decoded picture information for display.
 * @brief  Decoded Picture Information
 */
struct vdec_dec_pict_info {
	enum vdec_pict_state	pict_state;
	enum img_buffer_type	buf_type;
	u8			interlaced_flds;
	u32			err_flags;
	u32			err_level;
	struct vdec_pict_tag_container	first_fld_tag_container;
	struct vdec_pict_tag_container	second_fld_tag_container;
	struct vdec_str_opconfig	op_config;
	struct vdec_pict_rendinfo	rend_info;
	struct vdec_pict_disp_info	disp_info;
	u32				last_in_seq;
	u32				decode_id;
	u32				id_for_hwcrc_chk;
	u16				view_id;
	u32				timestamp;
	struct vdec_pict_supl_data  pict_supl_data;
};

struct vdec_pict_rend_config {
	struct vdec_pict_size	coded_pict_size;
	u8			packed;
	u8			byte_interleave;
	u32			stride_alignment;
};

/*
 * This structure contains unsupported feature flags.
 * @brief  Unsupported Feature Flags
 */
struct vdec_unsupp_flags {
	u32 str_cfg;
	u32 str_opcfg;
	u32 op_bufcfg;
	u32 seq_hdr;
	u32 pict_hdr;
};

/*
 * This type defines the error , error in parsing, error in decoding etc.
 * @brief  VDEC parsing/decoding error  Information
 */
enum vdec_error_type {
	VDEC_ERROR_NONE			= (0),
	VDEC_ERROR_SR_ERROR		= (1 << 0),
	VDEC_ERROR_FEHW_TIMEOUT		= (1 << 1),
	VDEC_ERROR_FEHW_DECODE		= (1 << 2),
	VDEC_ERROR_BEHW_TIMEOUT		= (1 << 3),
	VDEC_ERROR_SERVICE_TIMER_EXPIRY	= (1 << 4),
	VDEC_ERROR_MISSING_REFERENCES	= (1 << 5),
	VDEC_ERROR_MMU_FAULT		= (1 << 6),
	VDEC_ERROR_DEVICE		= (1 << 7),
	VDEC_ERROR_CORRUPTED_REFERENCE	= (1 << 8),
	VDEC_ERROR_MMCO			= (1 << 9),
	VDEC_ERROR_MBS_DROPPED		= (1 << 10),
	VDEC_ERROR_MAX			= (1 << 11),
};

/*
 * This structure contains information relating to a buffer.
 * @brief  Buffer Information
 */
struct vdec_buf_info {
	void	*cpu_linear_addr;
	u32	buf_id;
	struct vdec_pict_bufconfig	pictbuf_cfg;
	int	fd;
	/* The following are fields used internally within VDEC... */
	u32	buf_size;
	enum sys_emem_attrib	mem_attrib;
	void	*buf_alloc_handle;
	void	*buf_map_handle;
};
#endif
