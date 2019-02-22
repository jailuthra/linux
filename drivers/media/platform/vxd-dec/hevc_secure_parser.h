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
#ifndef __HEVCSECUREPARSER_H__
#define __HEVCSECUREPARSER_H__

#include "bspp_int.h"

#define HEVC_MAX_NUM_PROFILE_IDC	(32)
#define HEVC_MAX_NUM_SUBLAYERS		(7)
#define HEVC_MAX_VPS_OP_SETS_PLUS1	(1024)
#define HEVC_MAX_VPS_NUH_RESERVED_ZERO_LAYER_ID_PLUS1	(1)
#define HEVC_MAX_NUM_REF_PICS		(16)
#define HEVC_MAX_NUM_ST_REF_PIC_SETS	(65)
#define HEVC_MAX_NUM_LT_REF_PICS	(32)
#define HEVC_MAX_NUM_REF_IDX_ACTIVE	(15)
#define HEVC_LEVEL_IDC_MIN		(30)
#define HEVC_LEVEL_IDC_MAX		(186)
#define HEVC_1_0_PROFILE_IDC_MAX	(3)
#define HEVC_MAX_CPB_COUNT		(32)
#define HEVC_MIN_CODED_UNIT_SIZE	(8)

/* hevc scaling lists (all values are maximum possible ones) */
#define HEVC_SCALING_LIST_NUM_SIZES	(4)
#define HEVC_SCALING_LIST_NUM_MATRICES	(6)
#define HEVC_SCALING_LIST_MATRIX_SIZE	(64)

#define HEVC_MAX_TILE_COLS		(20)
#define HEVC_MAX_TILE_ROWS		(22)

#define HEVC_EXTENDED_SAR		(255)

#define HEVC_MAX_CHROMA_QP		(6)

enum hevc_nalunittype {
	HEVC_NALTYPE_TRAIL_N = 0,
	HEVC_NALTYPE_TRAIL_R = 1,
	HEVC_NALTYPE_TSA_N = 2,
	HEVC_NALTYPE_TSA_R = 3,
	HEVC_NALTYPE_STSA_N = 4,
	HEVC_NALTYPE_STSA_R = 5,
	HEVC_NALTYPE_RADL_N = 6,
	HEVC_NALTYPE_RADL_R = 7,
	HEVC_NALTYPE_RASL_N = 8,
	HEVC_NALTYPE_RASL_R = 9,
	HEVC_NALTYPE_RSV_VCL_N10 = 10,
	HEVC_NALTYPE_RSV_VCL_R11 = 11,
	HEVC_NALTYPE_RSV_VCL_N12 = 12,
	HEVC_NALTYPE_RSV_VCL_R13 = 13,
	HEVC_NALTYPE_RSV_VCL_N14 = 14,
	HEVC_NALTYPE_RSV_VCL_R15 = 15,
	HEVC_NALTYPE_BLA_W_LP = 16,
	HEVC_NALTYPE_BLA_W_RADL = 17,
	HEVC_NALTYPE_BLA_N_LP  = 18,
	HEVC_NALTYPE_IDR_W_RADL = 19,
	HEVC_NALTYPE_IDR_N_LP = 20,
	HEVC_NALTYPE_CRA = 21,
	HEVC_NALTYPE_RSV_IRAP_VCL22 = 22,
	HEVC_NALTYPE_RSV_IRAP_VCL23 = 23,
	HEVC_NALTYPE_VPS = 32,
	HEVC_NALTYPE_SPS = 33,
	HEVC_NALTYPE_PPS = 34,
	HEVC_NALTYPE_AUD = 35,
	HEVC_NALTYPE_EOS = 36,
	HEVC_NALTYPE_EOB = 37,
	HEVC_NALTYPE_FD = 38,
	HEVC_NALTYPE_PREFIX_SEI = 39,
	HEVC_NALTYPE_SUFFIX_SEI = 40
};

enum bspp_hevcslicetype {
	HEVC_SLICE_B = 0,
	HEVC_SLICE_P = 1,
	HEVC_SLICE_I = 2
};

/* HEVC NAL unit header */
struct bspp_hevcnalheader {
	u8 nal_unit_type;
	u8 nuh_layer_id;
	u8 nuh_temporal_id_plus1;
};

/* HEVC video profile_tier_level */
struct bspp_hevc_profile_tierlevel {
	u8 general_profile_space;
	u8 general_tier_flag;
	u8 general_profile_idc;
	u8 general_profile_compatibility_flag[HEVC_MAX_NUM_PROFILE_IDC];
	u8 general_progressive_source_flag;
	u8 general_interlaced_source_flag;
	u8 general_non_packed_constraint_flag;
	u8 general_frame_only_constraint_flag;
	u8 general_max_12bit_constraint_flag;
	u8 general_max_10bit_constraint_flag;
	u8 general_max_8bit_constraint_flag;
	u8 general_max_422chroma_constraint_flag;
	u8 general_max_420chroma_constraint_flag;
	u8 general_max_monochrome_constraint_flag;
	u8 general_intra_constraint_flag;
	u8 general_one_picture_only_constraint_flag;
	u8 general_lower_bit_rate_constraint_flag;
	u8 general_level_idc;
	u8 sub_layer_profile_present_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	u8 sub_layer_level_present_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	u8 sub_layer_profile_space[HEVC_MAX_NUM_SUBLAYERS - 1];
	u8 sub_layer_tier_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	u8 sub_layer_profile_idc[HEVC_MAX_NUM_SUBLAYERS - 1];
	u8 sub_layer_profile_compatibility_flag[HEVC_MAX_NUM_SUBLAYERS - 1][HEVC_MAX_NUM_PROFILE_IDC];
	u8 sub_layer_progressive_source_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	u8 sub_layer_interlaced_source_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	u8 sub_layer_non_packed_constraint_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	u8 sub_layer_frame_only_constraint_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	u8 sub_layer_max_12bit_constraint_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	u8 sub_layer_max_10bit_constraint_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	u8 sub_layer_max_8bit_constraint_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	u8 sub_layer_max_422chroma_constraint_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	u8 sub_layer_max_420chroma_constraint_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	u8 sub_layer_max_monochrome_constraint_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	u8 sub_layer_intra_constraint_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	u8 sub_layer_one_picture_only_constraint_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	u8 sub_layer_lower_bit_rate_constraint_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	u8 sub_layer_level_idc[HEVC_MAX_NUM_SUBLAYERS - 1];
};

/* HEVC sub layer HRD parameters */
struct bspp_hevc_sublayer_hrd_parameters {
	u8 bit_rate_value_minus1[HEVC_MAX_CPB_COUNT];
	u8 cpb_size_value_minus1[HEVC_MAX_CPB_COUNT];
	u8 cpb_size_du_value_minus1[HEVC_MAX_CPB_COUNT];
	u8 bit_rate_du_value_minus1[HEVC_MAX_CPB_COUNT];
	u8 cbr_flag[HEVC_MAX_CPB_COUNT];
};

/* HEVC HRD parameters */
struct bspp_hevc_hrd_parameters {
	u8 nal_hrd_parameters_present_flag;
	u8 vcl_hrd_parameters_present_flag;
	u8 sub_pic_hrd_params_present_flag;
	u8 tick_divisor_minus2;
	u8 du_cpb_removal_delay_increment_length_minus1;
	u8 sub_pic_cpb_params_in_pic_timing_sei_flag;
	u8 dpb_output_delay_du_length_minus1;
	u8 bit_rate_scale;
	u8 cpb_size_scale;
	u8 cpb_size_du_scale;
	u8 initial_cpb_removal_delay_length_minus1;
	u8 au_cpb_removal_delay_length_minus1;
	u8 dpb_output_delay_length_minus1;
	u8 fixed_pic_rate_general_flag[HEVC_MAX_NUM_SUBLAYERS];
	u8 fixed_pic_rate_within_cvs_flag[HEVC_MAX_NUM_SUBLAYERS];
	u8 elemental_duration_in_tc_minus1[HEVC_MAX_NUM_SUBLAYERS];
	u8 low_delay_hrd_flag[HEVC_MAX_NUM_SUBLAYERS];
	u8 cpb_cnt_minus1[HEVC_MAX_NUM_SUBLAYERS];
	struct bspp_hevc_sublayer_hrd_parameters sublayhrdparams[HEVC_MAX_NUM_SUBLAYERS];
};

/* HEVC video parameter set */
struct bspp_hevc_vps {
	u8 is_different;
	u8 is_sent;
	u8 is_available;
	u8 vps_video_parameter_set_id;
	u8 vps_reserved_three_2bits;
	u8 vps_max_layers_minus1;
	u8 vps_max_sub_layers_minus1;
	u8 vps_temporal_id_nesting_flag;
	u16 vps_reserved_0xffff_16bits;
	struct bspp_hevc_profile_tierlevel profiletierlevel;
	u8 vps_max_dec_pic_buffering_minus1[HEVC_MAX_NUM_SUBLAYERS];
	u8 vps_max_num_reorder_pics[HEVC_MAX_NUM_SUBLAYERS];
	u8 vps_max_latency_increase_plus1[HEVC_MAX_NUM_SUBLAYERS];
	u8 vps_sub_layer_ordering_info_present_flag;
	u8 vps_max_layer_id;
	u8 vps_num_layer_sets_minus1;
	u8 layer_id_included_flag[HEVC_MAX_VPS_OP_SETS_PLUS1][HEVC_MAX_VPS_NUH_RESERVED_ZERO_LAYER_ID_PLUS1];
	u8 vps_timing_info_present_flag;
	u32 vps_num_units_in_tick;
	u32 vps_time_scale;
	u8 vps_poc_proportional_to_timing_flag;
	u8 vps_num_ticks_poc_diff_one_minus1;
	u8 vps_num_hrd_parameters;
	u8 *hrd_layer_set_idx;
	u8 *cprms_present_flag;
	u8 vps_extension_flag;
	u8 vps_extension_data_flag;
};

/* HEVC scaling lists */
struct bspp_hevc_scalinglist_data {
	u8 dccoeffs[HEVC_SCALING_LIST_NUM_SIZES - 2][HEVC_SCALING_LIST_NUM_MATRICES];
	u8 lists[HEVC_SCALING_LIST_NUM_SIZES][HEVC_SCALING_LIST_NUM_MATRICES][HEVC_SCALING_LIST_MATRIX_SIZE];
};

/* HEVC short term reference picture set */
struct bspp_hevc_shortterm_refpicset {
	u8 num_negative_pics;
	u8 num_positive_pics;
	short int delta_poc_s0[HEVC_MAX_NUM_REF_PICS];
	short int delta_poc_s1[HEVC_MAX_NUM_REF_PICS];
	u8 used_bycurr_pic_s0[HEVC_MAX_NUM_REF_PICS];
	u8 used_bycurr_pic_s1[HEVC_MAX_NUM_REF_PICS];
	u8 num_delta_pocs;
};

/* HEVC video usability information */
struct bspp_hevc_vui_params {
	u8 aspect_ratio_info_present_flag;
	u8 aspect_ratio_idc;
	u16 sar_width;
	u16 sar_height;
	u8 overscan_info_present_flag;
	u8 overscan_appropriate_flag;
	u8 video_signal_type_present_flag;
	u8 video_format;
	u8 video_full_range_flag;
	u8 colour_description_present_flag;
	u8 colour_primaries;
	u8 transfer_characteristics;
	u8 matrix_coeffs;
	u8 chroma_loc_info_present_flag;
	u8 chroma_sample_loc_type_top_field;
	u8 chroma_sample_loc_type_bottom_field;
	u8 neutral_chroma_indication_flag;
	u8 field_seq_flag;
	u8 frame_field_info_present_flag;
	u8 default_display_window_flag;
	u16 def_disp_win_left_offset;
	u16 def_disp_win_right_offset;
	u16 def_disp_win_top_offset;
	u16 def_disp_win_bottom_offset;
	u8 vui_timing_info_present_flag;
	u32 vui_num_units_in_tick;
	u32 vui_time_scale;
	u8 vui_poc_proportional_to_timing_flag;
	u32 vui_num_ticks_poc_diff_one_minus1;
	u8 vui_hrd_parameters_present_flag;
	struct bspp_hevc_hrd_parameters vui_hrd_params;
	u8 bitstream_restriction_flag;
	u8 tiles_fixed_structure_flag;
	u8 motion_vectors_over_pic_boundaries_flag;
	u8 restricted_ref_pic_lists_flag;
	u16 min_spatial_segmentation_idc;
	u8 max_bytes_per_pic_denom;
	u8 max_bits_per_min_cu_denom;
	u8 log2_max_mv_length_horizontal;
	u8 log2_max_mv_length_vertical;
};

/* HEVC sps range extensions */
struct bspp_hevc_sps_range_exts {
	u8 transform_skip_rotation_enabled_flag;
	u8 transform_skip_context_enabled_flag;
	u8 implicit_rdpcm_enabled_flag;
	u8 explicit_rdpcm_enabled_flag;
	u8 extended_precision_processing_flag;
	u8 intra_smoothing_disabled_flag;
	u8 high_precision_offsets_enabled_flag;
	u8 persistent_rice_adaptation_enabled_flag;
	u8 cabac_bypass_alignment_enabled_flag;
};

/* HEVC sequence parameter set */
struct bspp_hevc_sps {
	u8 is_different;
	u8 is_sent;
	u8 is_available;
	u8 sps_video_parameter_set_id;
	u8 sps_max_sub_layers_minus1;
	u8 sps_temporal_id_nesting_flag;
	struct bspp_hevc_profile_tierlevel profile_tier_level;
	u8 sps_seq_parameter_set_id;
	u8 chroma_format_idc;
	u8 separate_colour_plane_flag;
	u32 pic_width_in_luma_samples;
	u32 pic_height_in_luma_samples;
	u8 conformance_window_flag;
	u16 conf_win_left_offset;
	u16 conf_win_right_offset;
	u16 conf_win_top_offset;
	u16 conf_win_bottom_offset;
	u8 bit_depth_luma_minus8;
	u8 bit_depth_chroma_minus8;
	u8 log2_max_pic_order_cnt_lsb_minus4;
	u8 sps_sub_layer_ordering_info_present_flag;
	u8 sps_max_dec_pic_buffering_minus1[HEVC_MAX_NUM_SUBLAYERS];
	u8 sps_max_num_reorder_pics[HEVC_MAX_NUM_SUBLAYERS];
	u32 sps_max_latency_increase_plus1[HEVC_MAX_NUM_SUBLAYERS];
	u8 log2_min_luma_coding_block_size_minus3;
	u8 log2_diff_max_min_luma_coding_block_size;
	u8 log2_min_transform_block_size_minus2;
	u8 log2_diff_max_min_transform_block_size;
	u8 max_transform_hierarchy_depth_inter;
	u8 max_transform_hierarchy_depth_intra;
	u8 scaling_list_enabled_flag;
	u8 sps_scaling_list_data_present_flag;
	struct bspp_hevc_scalinglist_data scalinglist_data;
	u8 amp_enabled_flag;
	u8 sample_adaptive_offset_enabled_flag;
	u8 pcm_enabled_flag;
	u8 pcm_sample_bit_depth_luma_minus1;
	u8 pcm_sample_bit_depth_chroma_minus1;
	u8 log2_min_pcm_luma_coding_block_size_minus3;
	u8 log2_diff_max_min_pcm_luma_coding_block_size;
	u8 pcm_loop_filter_disabled_flag;
	u8 num_short_term_ref_pic_sets;
	struct bspp_hevc_shortterm_refpicset rps_list[HEVC_MAX_NUM_ST_REF_PIC_SETS];
	u8 long_term_ref_pics_present_flag;
	u8 num_long_term_ref_pics_sps;
	u16 lt_ref_pic_poc_lsb_sps[HEVC_MAX_NUM_LT_REF_PICS];
	u8 used_by_curr_pic_lt_sps_flag[HEVC_MAX_NUM_LT_REF_PICS];
	u8 sps_temporal_mvp_enabled_flag;
	u8 strong_intra_smoothing_enabled_flag;
	u8 vui_parameters_present_flag;
	struct bspp_hevc_vui_params vui_params;
	u8 sps_extension_present_flag;
	u8 sps_range_extensions_flag;
	struct bspp_hevc_sps_range_exts range_exts;
	u8 sps_extension_7bits;
	u8 sps_extension_data_flag;
	/* derived elements */
	u8 sub_width_c;
	u8 sub_height_c;
	u8 ctb_log2size_y;
	u8 ctb_size_y;
	u32 pic_width_in_ctbs_y;
	u32 pic_height_in_ctbs_y;
	u32 pic_size_in_ctbs_y;
	int max_pic_order_cnt_lsb;
	u32 sps_max_latency_pictures[HEVC_MAX_NUM_SUBLAYERS];
	 /* raw vui data as extracted from bitstream. */
	struct bspp_raw_bitstream_data *vui_raw_data;
};

/*
 * This structure contains HEVC sequence header information (VPS, SPS, VUI)
 * contains everything parsed from the video/sequence header.
 */
struct bspp_hevc_sequ_hdr_info {
	struct bspp_hevc_vps vps;
	struct bspp_hevc_sps sps;
};

/* HEVC pps range extensions */
struct bspp_hevc_pps_range_exts {
	u8 log2_max_transform_skip_block_size_minus2;
	u8 cross_component_prediction_enabled_flag;
	u8 chroma_qp_offset_list_enabled_flag;
	u8 diff_cu_chroma_qp_offset_depth;
	u8 chroma_qp_offset_list_len_minus1;
	char cb_qp_offset_list[HEVC_MAX_CHROMA_QP];
	char cr_qp_offset_list[HEVC_MAX_CHROMA_QP];
	u8 log2_sao_offset_scale_luma;
	u8 log2_sao_offset_scale_chroma;
};

/* HEVC picture parameter set */
struct bspp_hevc_pps {
	u8 is_available;
	u8 is_param_copied;
	u8 pps_pic_parameter_set_id;
	u8 pps_seq_parameter_set_id;
	u8 dependent_slice_segments_enabled_flag;
	u8 output_flag_present_flag;
	u8 num_extra_slice_header_bits;
	u8 sign_data_hiding_enabled_flag;
	u8 cabac_init_present_flag;
	u8 num_ref_idx_l0_default_active_minus1;
	u8 num_ref_idx_l1_default_active_minus1;
	char init_qp_minus26;
	u8 constrained_intra_pred_flag;
	u8 transform_skip_enabled_flag;
	u8 cu_qp_delta_enabled_flag;
	u8 diff_cu_qp_delta_depth;
	int pps_cb_qp_offset;
	int pps_cr_qp_offset;
	u8 pps_slice_chroma_qp_offsets_present_flag;
	u8 weighted_pred_flag;
	u8 weighted_bipred_flag;
	u8 transquant_bypass_enabled_flag;
	u8 tiles_enabled_flag;
	u8 entropy_coding_sync_enabled_flag;
	u8 num_tile_columns_minus1;
	u8 num_tile_rows_minus1;
	u8 uniform_spacing_flag;
	u8 column_width_minus1[HEVC_MAX_TILE_COLS];
	u8 row_height_minus1[HEVC_MAX_TILE_ROWS];
	u8 loop_filter_across_tiles_enabled_flag;
	u8 pps_loop_filter_across_slices_enabled_flag;
	u8 deblocking_filter_control_present_flag;
	u8 deblocking_filter_override_enabled_flag;
	u8 pps_deblocking_filter_disabled_flag;
	char pps_beta_offset_div2;
	char pps_tc_offset_div2;
	u8 pps_scaling_list_data_present_flag;
	struct bspp_hevc_scalinglist_data scaling_list;
	u8 lists_modification_present_flag;
	u8 log2_parallel_merge_level_minus2;
	u8 slice_segment_header_extension_present_flag;
	u8 pps_extension_present_flag;
	u8 pps_range_extensions_flag;
	struct bspp_hevc_pps_range_exts range_exts;
	u8 pps_extension_7bits;
	u8 pps_extension_data_flag;
	/* derived elements */
	u16 col_bd[HEVC_MAX_TILE_COLS + 1];
	u16 row_bd[HEVC_MAX_TILE_ROWS + 1];
	/* PVDEC derived elements */
	u32 max_tile_height_in_ctbs_y;
};

/* HEVC slice segment header */
struct bspp_hevc_slice_segment_header {
	u8 bslice_is_idr;
	u8 first_slice_segment_in_pic_flag;
	u8 no_output_of_prior_pics_flag;
	u8 slice_pic_parameter_set_id;
	u8 dependent_slice_segment_flag;
	u32 slice_segment_address;
};

/*
 * @Function   bspp_hevc_set_parser_config
 * sets the parser configuration.
 */
int bspp_hevc_set_parser_config(enum vdec_bstr_format bstr_format,
				struct bspp_vid_std_features *pvidstd_features,
				struct bspp_swsr_ctx *pswsr_ctx,
				struct bspp_parser_callbacks *pparser_callbacks,
				struct bspp_inter_pict_data *pinterpict_data);

void bspp_hevc_determine_unittype(u8 bitstream_unittype,
				  s32 disable_mvc,
				  enum bspp_unit_type *bspp_unittype);

#endif /*__H264SECUREPARSER_H__ */
