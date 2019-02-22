/* SPDX-License-Identifier: GPL-2.0 */
/*
 * h.264 secure data unit parsing API.
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
#ifndef __H264SECUREPARSER_H__
#define __H264SECUREPARSER_H__

#include "bspp_int.h"
#include "vdec_defs.h"

/*
 * enum h264_nalunittype
 * @Description Contains H264 NAL unit types
 */
enum h264_nalunittype {
	H264_NALTYPE_UNSPECIFIED = 0,
	H264_NALTYPE_SLICE = 1,
	H264_NALTYPE_SLICE_PARTITION_A = 2,
	H264_NALTYPE_SLICE_PARTITION_B = 3,
	H264_NALTYPE_SLICE_PARTITION_C = 4,
	H264_NALTYPE_IDR_SLICE = 5,
	H264_NALTYPE_SUPPLEMENTAL_ENHANCEMENT_INFO = 6,
	H264_NALTYPE_SEQUENCE_PARAMETER_SET = 7,
	H264_NALTYPE_PICTURE_PARAMETER_SET = 8,
	H264_NALTYPE_ACCESS_UNIT_DELIMITER = 9,
	H264_NALTYPE_END_OF_SEQUENCE = 10,
	H264_NALTYPE_END_OF_STREAM = 11,
	H264_NALTYPE_FILLER_DATA = 12,
	H264_NALTYPE_SEQUENCE_PARAMETER_SET_EXTENSION = 13,
	H264_NALTYPE_SLICE_PREFIX = 14,
	H264_NALTYPE_SUBSET_SPS = 15,
	H264_NALTYPE_AUXILIARY_SLICE = 19,
	H264_NALTYPE_SLICE_SCALABLE = 20,
	H264_NALTYPE_SLICE_IDR_SCALABLE = 21,
	H264_NALTYPE_MAX = 31,
};

/*
 * struct bspp_h264_sps_info
 * @Description	H264 SPS parsed information
 */
struct bspp_h264_sps_info {
	u32 profile_idc;
	u32 constraint_set_flags;
	u32 level_idc;
	u8 seq_parameter_set_id;
	u8 chroma_format_idc;
	s32 separate_colour_plane_flag;
	u32 bit_depth_luma_minus8;
	u32 bit_depth_chroma_minus8;
	u8 qpprime_y_zero_transform_bypass_flag;
	s32 seq_scaling_matrix_present_flag;
	u8 seq_scaling_list_present_flag[12];
	u32 log2_max_frame_num_minus4;
	u32 pic_order_cnt_type;
	u32 log2_max_pic_order_cnt_lsb_minus4;
	s32 delta_pic_order_always_zero_flag;
	s32 offset_for_non_ref_pic;
	s32 offset_for_top_to_bottom_field;
	u32 num_ref_frames_in_pic_order_cnt_cycle;
	u32 *offset_for_ref_frame;
	u32 max_num_ref_frames;
	s32 gaps_in_frame_num_value_allowed_flag;
	u32 pic_width_in_mbs_minus1;
	u32 pic_height_in_map_units_minus1;
	s32 frame_mbs_only_flag;
	s32 mb_adaptive_frame_field_flag;
	s32 direct_8x8_inference_flag;
	s32 frame_cropping_flag;
	u32 frame_crop_left_offset;
	u32 frame_crop_right_offset;
	u32 frame_crop_top_offset;
	u32 frame_crop_bottom_offset;
	s32 vui_parameters_present_flag;
	/* mvc_vui_parameters_present_flag;   UNUSED */
	s32 bmvcvuiparameterpresentflag;
	/*
	 * scaling lists are derived from both SPS and PPS information
	 * but will change whenever the PPS changes
	 * The derived set of tables are associated here with the PPS
	 * NB: These are in H.264 order
	 */
	/* derived from SPS and PPS - 8 bit each */
	u8 *scllst4x4seq;
	/* derived from SPS and PPS - 8 bit each */
	u8 *scllst8x8seq;
	/* This is not direct parsed data, though it is extracted */
	u8 usedefaultscalingmatrixflag_seq[12];
};

struct bspp_h264_hrdparam_info {
	u8     cpb_cnt_minus1;
	u8     bit_rate_scale;
	u8     cpb_size_scale;
	u32	*bit_rate_value_minus1;
	u32	*cpb_size_value_minus1;
	u8	*cbr_flag;
	u8	initial_cpb_removal_delay_length_minus1;
	u8	cpb_removal_delay_length_minus1;
	u8	dpb_output_delay_length_minus1;
	u8	time_offset_length;
};

struct bspp_h264_vui_info {
	u8                aspect_ratio_info_present_flag;
	u32              aspect_ratio_idc;
	u32              sar_width;
	u32              sar_height;
	u8                overscan_info_present_flag;
	u8                overscan_appropriate_flag;
	u8                video_signal_type_present_flag;
	u32              video_format;
	u8                video_full_range_flag;
	u8                colour_description_present_flag;
	u32              colour_primaries;
	u32              transfer_characteristics;
	u32              matrix_coefficients;
	u8                chroma_location_info_present_flag;
	u32              chroma_sample_loc_type_top_field;
	u32              chroma_sample_loc_type_bottom_field;
	u8                timing_info_present_flag;
	u32              num_units_in_tick;
	u32              time_scale;
	u8                fixed_frame_rate_flag;
	u8                nal_hrd_parameters_present_flag;
	struct bspp_h264_hrdparam_info  nal_hrd_parameters;
	u8                vcl_hrd_parameters_present_flag;
	struct bspp_h264_hrdparam_info  vcl_hrd_parameters;
	u8                low_delay_hrd_flag;
	u8                pic_struct_present_flag;
	u8                bitstream_restriction_flag;
	u8                motion_vectors_over_pic_boundaries_flag;
	u32              max_bytes_per_pic_denom;
	u32              max_bits_per_mb_denom;
	u32              log2_max_mv_length_vertical;
	u32              log2_max_mv_length_horizontal;
	u32              num_reorder_frames;
	u32              max_dec_frame_buffering;
};

/*
 * struct bspp_h264_seq_hdr_info
 * @Description	Contains everything parsed from the Sequence Header.
 */
struct bspp_h264_seq_hdr_info {
	/* Video sequence header information */
	struct bspp_h264_sps_info sps_info;
	struct bspp_h264_vui_info         vui_info;        /* VUI sequence header information.            */
};

/*
 * struct bspp_h264_ppssgm_info
 * @Description	This structure contains H264 PPS parse data.
 */
struct bspp_h264_ppssgm_info {
	u8 *slice_group_id;
	u16 slicegroupidnum;
};

/*
 * struct bspp_h264_pps_info
 * @Description	This structure contains H264 PPS parse data.
 */
struct bspp_h264_pps_info {
	/* pic_parameter_set_id: defines the PPS ID of the current PPS */
	s32 pps_id;
	/* seq_parameter_set_id: defines the SPS that current PPS points to */
	s32 seq_parameter_set_id;
	s32 entropy_coding_mode_flag;
	s32 pic_order_present_flag;
	u8 num_slice_groups_minus1;
	u8 slice_group_map_type;
	u16 run_length_minus1[8];
	u16 top_left[8];
	u16 bottom_right[8];
	s32 slice_group_change_direction_flag;
	u16 slice_group_change_rate_minus1;
	u16 pic_size_in_map_unit;
	struct bspp_h264_ppssgm_info h264_ppssgm_info;
	u8 num_ref_idx_lx_active_minus1[H264FW_MAX_REFPIC_LISTS];
	s32 weighted_pred_flag;
	u8 weighted_bipred_idc;
	s32 pic_init_qp_minus26;
	s32 pic_init_qs_minus26;
	s32 chroma_qp_index_offset;
	s32 deblocking_filter_control_present_flag;
	s32 constrained_intra_pred_flag;
	s32 redundant_pic_cnt_present_flag;
	s32 transform_8x8_mode_flag;
	s32 pic_scaling_matrix_present_flag;
	u8 pic_scaling_list_present_flag[12];
	s32 second_chroma_qp_index_offset;

	/*
	 * scaling lists are derived from both SPS and PPS information
	 * but will change whenever the PPS changes
	 * The derived set of tables are associated here with the PPS
	 * NB: These are in H.264 order
	 */
	/* derived from SPS and PPS - 8 bit each */
	u8 *scllst4x4pic;
	/* derived from SPS and PPS - 8 bit each */
	u8 *scllst8x8pic;
	/* This is not direct parsed data, though it is extracted */
	u8 usedefaultscalingmatrixflag_pic[12];
};

/*
 * enum bspp_h264_slice_type
 * @Description	contains H264 slice types
 */
enum bspp_h264_slice_type {
	P_SLICE = 0,
	B_SLICE,
	I_SLICE,
	SP_SLICE,
	SI_SLICE
};

/*
 * struct bspp_h264_slice_hdr_info
 * @Description This structure contains H264 slice header information
 */
struct bspp_h264_slice_hdr_info {
	u16 first_mb_in_slice;
	enum bspp_h264_slice_type slice_type;

	/* data to ID new picture */
	u32 pps_id;
	u32 frame_num;
	u8 colour_plane_id;
	u8 field_pic_flag;
	u8 bottom_field_flag;
	u32 idr_pic_id;
	u32 pic_order_cnt_lsb;
	s32 delta_pic_order_cnt_bottom;
	s32 delta_pic_order_cnt[2];
	u32 redundant_pic_cnt;

	/* Things we need to read out when doing In Secure */
	u8 num_ref_idx_active_override_flag;
	u8 num_ref_idx_lx_active_minus1[2];
	u16 slice_group_change_cycle;
};

/*
 * @Function	bspp_h264_set_parser_config
 * @Description	Sets the parser configuration
 */
s32 bspp_h264_set_parser_config(enum vdec_bstr_format bstr_format,
				struct bspp_vid_std_features *pvidstd_features,
				struct bspp_swsr_ctx *pswsr_ctx,
				struct bspp_parser_callbacks *pparser_callbacks,
				struct bspp_inter_pict_data *pinterpict_data);

/*
 * @Function	bspp_h264_determine_unittype
 * @Description	This function determines the BSPP unit type based on the
 *		provided bitstream (H264 specific) unit type
 */
void bspp_h264_determine_unittype(u8 bitstream_unittype,
				  s32 disable_mvc,
				  enum bspp_unit_type *pbsppunittype);

#endif /*__H264SECUREPARSER_H__ */
