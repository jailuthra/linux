// SPDX-License-Identifier: GPL-2.0
/*
 * VXD Decoder Core component function implementations
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "core.h"
#include "decoder.h"
#include "img_errors.h"
#include "img_pixfmts.h"
#include "lst.h"
#include "resource.h"
#include "rman_api.h"
#include "vdecdd_utils.h"
#include "vdec_mmu_wrapper.h"
#include "vxd_dec.h"

/*
 * This enum defines resource availability masks.
 * @brief  Resource Availability
 */
enum core_availability {
	CORE_AVAIL_PICTBUF	= (1 << 0),
	CORE_AVAIL_PICTRES	= (1 << 1),
	CORE_AVAIL_CORE		= (1 << 2),
	CORE_AVAIL_MAX
};

struct core_mbparam_alloc_info {
	u8	alloc_mbparam_bufs;
	u32	mbparam_size;
	u32	overalloc_mbnum;
};

static struct core_mbparam_alloc_info mbparam_allocinfo[VDEC_STD_MAX - 1] = {
    /*                AllocFlag    MBParamSize    Overalloc     */
    /* MPEG2    */  { true,    0xc8,          0             },
    /* MPEG4    */  { true,    0xc8,          0             },
    /* H263     */  { true,    0xc8,          0             },
    /* H264     */  { true,    0x80,          0             },
    /* VC1      */  { true,    0x80,          (4096 * 2) / 0x80 },
    /* AVS      */  { true,    0x80,          0             },
    /* REAL     */  { true,    0x80,          0             },
    /* JPEG     */  { false,   0x00,          0             },
    /* VP6      */  { true,    0x80,          0             },
    /* VP8      */  { true,    0x80,          0             },
    /* SORENSON */  { true,    0xc8,          0             },
    /* HEVC     */  { true,    0x40,          0             },
};

struct core_pict_resinfo {
	u32	pict_res_num;
	u32	mb_param_bufsize;
	u8	is_valid;
};

struct vxdio_mempool {
	u32			mem_heap_id;
	enum sys_emem_attrib	mem_attrib;
};

static u32 global_avail_slots;
static u8 is_core_initialized;

struct core_stream_ctx;

/*
 * This structure contains the core Context.
 * @brief  core Context
 */
struct core_context {
	struct vdecdd_dddev_context	*dev_ctx;
	/* List of stream context structures */
	struct lst_t	core_str_ctx;
	vxd_cb		vxd_str_processed_cb;
};

/* Global Core Context */
static struct core_context *global_core_ctx;

/*
 * This structure contains the core Stream Context.
 * @brief  core Stream Context
 */
struct core_stream_context {
	void	**link; /* to be part of single linked list */
	struct core_context	*core_ctx;
	struct vdecdd_ddstr_ctx	*dd_str_ctx;
	struct vxd_dec_ctx	*vxd_dec_context;
	/* List of picture buffers */
	struct lst_t		pict_buf_list;
	/* List of auxiliary picture resources */
	struct lst_t		pict_res_list;
	/* List of sequence header information */
	struct lst_t		seq_hdr_list;
	/* Queue of stream units to be processed */
	struct lst_t		str_unit_list;
	struct vdec_comsequ_hdrinfo	comseq_hdr_info;
	u8	opcfg_set;
	/* Picture buffer layout to use for decoding. */
	struct vdecdd_ddpict_buf	disp_pict_buf;
	struct vdec_str_opconfig	op_cfg;
	u8	new_seq;
	u8	new_op_cfg;
	u8	no_prev_refs_used;
	u32	avail_slots;
	u32	res_avail;
	u32	mbparams_bufsize;
	u8	stopped;
	struct core_pict_resinfo	pict_resinfo;
	/* Reconstructed picture buffer */
	struct vdecdd_ddpict_buf	recon_pictbuf;
	/* Coded picture size of last reconfiguration */
	struct vdec_pict_size		coded_pict_size;
};

static void core_fw_response_cb(int res_str_id, u32 *msg, u32 msg_size,
				u32 msg_flags)
{
	struct core_stream_context *core_str_ctx;
	int ret;

	/* extract core_str_ctx and dec_core_ctx from res_str_id */
	VDEC_ASSERT(res_str_id);

	/* Get access to stream context.. */
	ret = rman_get_resource(res_str_id, VDECDD_STREAM_TYPE_ID,
				(void **)&core_str_ctx, NULL);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		pr_err("could not extract core_str_context\n");

	ret =
	decoder_service_firmware_response(core_str_ctx->dd_str_ctx->dec_ctx,
					  msg, msg_size, msg_flags);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		pr_err("decoder_service_firmware_response failed\n");
}

/*
 * @Function CORE_Initialise
 */
int core_initialise(void *dev_handle, u32 int_heap_id, void *vxd_cb_ptr)
{
	struct vdecdd_dd_devconfig	dev_cfg_local;
	u32	num_pipes_local;
	int	ret;

	if (is_core_initialized)
		return IMG_ERROR_INVALID_PARAMETERS;

	is_core_initialized = true;

	global_core_ctx = kzalloc(sizeof(*global_core_ctx), GFP_KERNEL);
	if (!global_core_ctx) {
		is_core_initialized = false;
		return IMG_ERROR_OUT_OF_MEMORY;
	}

	global_core_ctx->dev_ctx = kzalloc(sizeof(*global_core_ctx->dev_ctx),
					   GFP_KERNEL);
	if (!global_core_ctx->dev_ctx) {
		kfree(global_core_ctx);
		global_core_ctx = NULL;
		is_core_initialized = false;
		return IMG_ERROR_OUT_OF_MEMORY;
	}

	/* Initialise device context. */
	global_core_ctx->dev_ctx->dev_handle = dev_handle; /* v4L2 dev handle */
	global_core_ctx->vxd_str_processed_cb = (vxd_cb)vxd_cb_ptr;

	ret = decoder_initialise(global_core_ctx->dev_ctx, int_heap_id,
				 &dev_cfg_local, &num_pipes_local,
				 &global_core_ctx->dev_ctx->dec_context);
	if (ret != IMG_SUCCESS)
		goto decoder_init_error;

	global_core_ctx->dev_ctx->internal_heap_id = int_heap_id;

	/* Dump codec config */
	pr_info("Decode slots/core:  %d", dev_cfg_local.num_slots_per_pipe);

	lst_init(&global_core_ctx->core_str_ctx);

	/* Ensure the resource manager is initialised.. */
	ret = rman_initialise();
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		goto create_bucket_error;

	/* Create resource bucket.. */
	ret = rman_create_bucket(&global_core_ctx->dev_ctx->res_buck_handle);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		goto create_bucket_error;

	return IMG_SUCCESS;

create_bucket_error:
	decoder_deinitialise(global_core_ctx->dev_ctx->dec_context);

decoder_init_error:
	kfree(global_core_ctx->dev_ctx);
	global_core_ctx->dev_ctx = NULL;

	return ret;
}

/*
 * @Function    core_check_decoder_support
 * @Description
 * This function determines whether Decoder supports bitstream and
 * configuration.
 */
static int
core_check_decoder_support(const struct vdecdd_dddev_context *dd_dev_ctx,
			   const struct vdec_str_configdata  *str_cfg_data,
			   const struct vdec_comsequ_hdrinfo *prev_seq_hdrinfo,
			   const struct bspp_pict_hdr_info *prev_pict_hdrinfo,
			   const struct vdecdd_mapbuf_info *map_bufinfo,
			   struct vdecdd_supp_check *supp_check)
{
	int ret;
	struct vdec_unsupp_flags  unsupported;
	struct vdec_pict_rendinfo disp_pict_rend_info;

	memset(&disp_pict_rend_info, 0, sizeof(struct vdec_pict_rendinfo));

	/*
	 * If output picture buffer information is provided create another
	 * with properties required by bitstream so that it can be compared.
	 */
	if (supp_check->disp_pictbuf) {
		struct vdec_pict_rend_config pict_rend_cfg;

		memset(&pict_rend_cfg, 0, sizeof(pict_rend_cfg));

		/*
		 * Cannot validate the display picture buffer layout without
		 * knowing the pixel format required for the output and the
		 * sequence information.
		 */
		if (supp_check->comseq_hdrinfo && supp_check->op_cfg) {
			pict_rend_cfg.coded_pict_size =
			supp_check->comseq_hdrinfo->max_frame_size;

			pict_rend_cfg.byte_interleave =
			supp_check->disp_pictbuf->buf_config.byte_interleave;

			pict_rend_cfg.packed =
			supp_check->disp_pictbuf->buf_config.packed;

			pict_rend_cfg.stride_alignment =
			supp_check->disp_pictbuf->buf_config.stride_alignment;

			/*
			 * Recalculate render picture layout based upon
			 * sequence and output config.
			 */
			vdecddutils_pictbuf_getinfo(str_cfg_data,
						    &pict_rend_cfg,
						    supp_check->op_cfg,
						    &disp_pict_rend_info);
		}
	}
	/* Check that the decoder supports the picture. */
	ret = decoder_check_support(dd_dev_ctx->dec_context, str_cfg_data,
				    supp_check->op_cfg,
				    supp_check->disp_pictbuf,
				    (disp_pict_rend_info.rendered_size) ?
				    &disp_pict_rend_info : NULL,
				    supp_check->comseq_hdrinfo,
				    supp_check->pict_hdrinfo,
				    prev_seq_hdrinfo,
				    prev_pict_hdrinfo,
				    supp_check->non_cfg_req, &unsupported,
				    &supp_check->features);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS) {
		if (ret == IMG_ERROR_NOT_SUPPORTED)
			supp_check->unsupp_flags = unsupported;
	}

	return ret;
}

/*
 * @Function  core_supported_features
 */
int core_supported_features(struct vdec_features *features)
{
	struct vdecdd_dddev_context *dd_dev_ctx;

	VDEC_ASSERT(global_core_ctx);

	dd_dev_ctx = global_core_ctx->dev_ctx;
	VDEC_ASSERT(dd_dev_ctx);
	if (!dd_dev_ctx)
		return IMG_ERROR_NOT_INITIALISED;

	return decoder_supported_features(dd_dev_ctx->dec_context, features);
}

/*
 * @Function core_stream_stop
 */
int core_stream_stop(u32 res_str_id)
{
	int  ret = IMG_SUCCESS;
	struct vdecdd_str_unit *stop_unit;
	struct vdecdd_ddstr_ctx *ddstr_ctx;
	struct core_stream_context *core_str_ctx;

	/*
	 * Stream based messages without a device context
	 * must have a stream ID.
	 */
	VDEC_ASSERT(res_str_id);

	if (res_str_id == 0) {
		pr_err("Invalid params passed to %s\n", __func__);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	/* Get access to stream context.. */
	ret = rman_get_resource(res_str_id, VDECDD_STREAM_TYPE_ID,
				(void **)&core_str_ctx, NULL);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	VDEC_ASSERT(core_str_ctx);

	ddstr_ctx = core_str_ctx->dd_str_ctx;

	/* Validate input arguments */
	VDEC_ASSERT(ddstr_ctx);

	/*
	 * Disregard this stop request if the stream is currently
	 * stopped or being stopped.
	 */
	if (ddstr_ctx->dd_str_state == VDECDD_STRSTATE_PLAYING) {
		vdecddutils_create_strunit(&stop_unit, NULL);
		if (!stop_unit) {
			pr_err("Failed to allocate memory for stop unit\n");
			return IMG_ERROR_OUT_OF_MEMORY;
		}
		memset(stop_unit, 0, sizeof(*stop_unit));

		stop_unit->str_unit_type = VDECDD_STRUNIT_STOP;
		stop_unit->str_unit_tag = NULL;
		stop_unit->decode = false;

		/*
		 * Since the stop is now to be passed to the decoder signal
		 * that we're stopping.
		 */
		ddstr_ctx->dd_str_state = VDECDD_STRSTATE_STOPPING;
		decoder_stream_process_unit(ddstr_ctx->dec_ctx, stop_unit);
		core_str_ctx->stopped = true;
	}

	return ret;
}

/*
 * @Function              core_is_stream_idle
 */
static u8 core_is_stream_idle(struct vdecdd_ddstr_ctx *dd_str_ctx)
{
	u8 is_stream_idle;

	is_stream_idle = decoder_is_stream_idle(dd_str_ctx->dec_ctx);

	return is_stream_idle;
}

/*
 * @Function              core_stream_destroy
 */
int core_stream_destroy(u32 res_str_id)
{
	struct vdecdd_ddstr_ctx *ddstr_ctx;
	struct core_stream_context *core_str_ctx;
	int ret;

	/*
	 * Stream based messages without a device context
	 * must have a stream ID.
	 */
	VDEC_ASSERT(res_str_id);

	if (res_str_id == 0) {
		pr_err("Invalid params passed to %s\n", __func__);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	/* Get access to stream context.. */
	ret = rman_get_resource(res_str_id, VDECDD_STREAM_TYPE_ID,
				(void **)&core_str_ctx, NULL);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	VDEC_ASSERT(core_str_ctx);

	ddstr_ctx = core_str_ctx->dd_str_ctx;

	/* Validate input arguments */
	VDEC_ASSERT(ddstr_ctx);

	ret = core_stream_stop(res_str_id);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	/* Destroy stream if idle otherwise wait and do it later */
	if (core_is_stream_idle(ddstr_ctx))
		rman_free_resource(ddstr_ctx->res_handle);

	/* Return success.. */
	return IMG_SUCCESS;
}

static int
core_picture_attach_resources(struct core_stream_context *core_str_ctx,
			      struct vdecdd_str_unit *str_unit, u8 check)
{
	u32 ret = IMG_SUCCESS;

	/*
	 * Take sequence header from cache.
	 * Note: sequence header id must be set in PICTURE_START unit
	 */
	str_unit->seq_hdr_info =
		resource_list_getbyid(&core_str_ctx->seq_hdr_list,
				      str_unit->seq_hdr_id);

	/* Check is not needed e.g. when freeing resources at stream destroy */
	if (check && !str_unit->seq_hdr_info) {
		pr_err("[USERSID=0x%08X] Sequence header not available for current picture while attaching",
		       core_str_ctx->dd_str_ctx->str_config_data.user_str_id);
		ret = IMG_ERROR_NOT_SUPPORTED;
	}

	return ret;
}

/*
 * @Function core_handle_processed_unit
 */
static int core_handle_processed_unit(struct core_stream_context *c_str_ctx,
				      struct vdecdd_str_unit *str_unit)
{
	struct bspp_bitstr_seg *bstr_seg;
	struct vdecdd_ddstr_ctx *dd_str_ctx = c_str_ctx->dd_str_ctx;
	int	ret;
	struct core_context *g_ctx = global_core_ctx;

	/* check for type of the unit */
	switch (str_unit->str_unit_type) {
	case VDECDD_STRUNIT_SEQUENCE_START:
		/* nothing to be done as sps is maintained till it changes */
	break;

	case VDECDD_STRUNIT_PICTURE_START:
		/* Loop over bit stream segments.. */
		bstr_seg = (struct bspp_bitstr_seg *)
				lst_removehead(&str_unit->bstr_seg_list);
		while (bstr_seg && (bstr_seg->bstr_seg_flag &
		       VDECDD_BSSEG_LASTINBUFF) && (dd_str_ctx->dd_str_state !=
		       VDECDD_STRSTATE_STOPPED)) {
			struct vdecdd_ddbuf_mapinfo *ddbuf_map_info;
			/* Get access to map info context.. */
			ret = rman_get_resource(bstr_seg->bufmap_id,
						VDECDD_BUFMAP_TYPE_ID,
						(void **)&ddbuf_map_info,
						NULL);
			VDEC_ASSERT(ret == IMG_SUCCESS);
			if (ret != IMG_SUCCESS)
				return ret;
			lst_add(&c_str_ctx->vxd_dec_context->seg_list,
				bstr_seg);
			g_ctx->vxd_str_processed_cb(c_str_ctx->vxd_dec_context,
						    VXD_CB_STRUNIT_PROCESSED,
						    bstr_seg->bufmap_id);

			/* Get next segment. */
			bstr_seg = (struct bspp_bitstr_seg *)
				lst_removehead(&str_unit->bstr_seg_list);
		}
	break;

	case VDECDD_STRUNIT_PICTURE_END:
		g_ctx->vxd_str_processed_cb(c_str_ctx->vxd_dec_context,
					    VXD_CB_PICT_END, 0xFFFF);
	break;

	case VDECDD_STRUNIT_STOP:
		/*
		 * Signal that the stream has been stopped in the
		 * device driver.
		 */
		dd_str_ctx->dd_str_state = VDECDD_STRSTATE_STOPPED;

	break;

	default:
		pr_err("Invalid stream unit type passed\n");
		return IMG_ERROR_GENERIC_FAILURE;
	}

	pr_info("[SID=0x%08X] [UTYPE=0x%08X] PROCESSED",
		dd_str_ctx->res_str_id,
		str_unit->str_unit_type);

	/* Return success.. */
	return IMG_SUCCESS;
}

static int
core_handle_decoded_picture(struct core_stream_context *core_str_ctx,
			    struct vdecdd_picture *picture, u32 type)
{
	/* Pick the client image buffer. */
	struct vdecdd_ddbuf_mapinfo *pictbuf_mapinfo =
				picture->disp_pict_buf.pict_buf;
	VDEC_ASSERT(pictbuf_mapinfo);
	if (!pictbuf_mapinfo)
		return IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE;

	global_core_ctx->vxd_str_processed_cb(core_str_ctx->vxd_dec_context,
					      type,
					      pictbuf_mapinfo->buf_map_id);
	return IMG_SUCCESS;
}

static int core_stream_processed_cb(void *handle, int cb_type, void *cb_item)
{
	int ret;
	struct core_stream_context *core_str_ctx =
		(struct core_stream_context *)handle;
	VDEC_ASSERT(core_str_ctx);
	if (!core_str_ctx) {
		pr_err("NULL handle passed to core callback\n");
		return IMG_ERROR_GENERIC_FAILURE;
	}

	/* Based on callback type, retrieve the item */
	switch (cb_type) {
	case VXD_CB_STRUNIT_PROCESSED:
		{
		struct vdecdd_str_unit *str_unit =
			(struct vdecdd_str_unit *)cb_item;
		VDEC_ASSERT(str_unit);
		if (!str_unit) {
			pr_err("NULL item passed to core callback type STRUNIT_PROCESSED\n");
			return IMG_ERROR_GENERIC_FAILURE;
		}
		ret = core_handle_processed_unit(core_str_ctx, str_unit);
		if (ret != IMG_SUCCESS) {
			pr_err("core_handle_processed_unit returned error\n");
			return ret;
		}
		break;
		}

	case VXD_CB_PICT_DECODED:
	case VXD_CB_PICT_DISPLAY:
	case VXD_CB_PICT_RELEASE:
		{
		struct vdecdd_picture *picture =
			(struct vdecdd_picture *)cb_item;
		if (!picture) {
			pr_err("NULL item passed to core callback type PICTURE_DECODED\n");
			return IMG_ERROR_GENERIC_FAILURE;
		}
		ret = core_handle_decoded_picture(core_str_ctx, picture,
						  cb_type);
		break;
		}

	case VXD_CB_STR_END:
		global_core_ctx->vxd_str_processed_cb(core_str_ctx->vxd_dec_context,
						      cb_type, 0);
		ret = IMG_SUCCESS;

		break;

	default:
		return 0;
	}

	return ret;
}

static int core_decoder_queries(void *handle, int query, void *item)
{
	struct core_stream_context *core_str_ctx =
		(struct core_stream_context *)handle;
	VDEC_ASSERT(core_str_ctx);
	if (!core_str_ctx) {
		pr_err("NULL handle passed to %s callback\n", __func__);
		return IMG_ERROR_GENERIC_FAILURE;
	}

	switch (query) {
	case DECODER_CORE_GET_RES_LIMIT:
		{
		u32 num_img_bufs;
		u32 num_res;

		num_img_bufs = resource_list_getnum(&core_str_ctx->pict_buf_list);

		/* Return the number of internal resources. */
		num_res = core_str_ctx->pict_resinfo.pict_res_num;

		/* Return the minimum of the two. */
		*((u32 *)item) = vdec_size_min(num_img_bufs, num_res);
		}
		break;

	default:
		return IMG_ERROR_GENERIC_FAILURE;
	}
	return IMG_SUCCESS;
}

static int
core_free_common_picture_resource(struct core_stream_context *core_str_ctx,
				  struct vdecdd_pict_resint *pict_resint)
{
	int ret = IMG_SUCCESS;

	if (pict_resint->mb_param_buf &&
	    pict_resint->mb_param_buf->ddbuf_info.hndl_memory) {
		pr_info("mmu_free for buff_id[%d]\n",
			pict_resint->mb_param_buf->ddbuf_info.buff_id);
		ret = mmu_free_mem(core_str_ctx->dd_str_ctx->mmu_str_handle,
				   &pict_resint->mb_param_buf->ddbuf_info);
		if (ret != IMG_SUCCESS)
			pr_err("MMU_Free for MBParam buffer failed with error %u",
			       ret);
	}
	return ret;
}

/*
 * @Function  core_fn_free_stream
 */
static void core_fn_free_stream(void *param)
{
	int	ret;
	struct vdecdd_ddstr_ctx	*dd_str_context;
	struct vdecdd_dddev_context *dd_dev_ctx;
	struct core_stream_context *core_str_ctx;
	struct vdecdd_pict_resint *pict_resint;

	/* Validate input arguments */
	VDEC_ASSERT(param);

	core_str_ctx = (struct core_stream_context *)param;

	dd_str_context = core_str_ctx->dd_str_ctx;

	VDEC_ASSERT(dd_str_context);
	if (!dd_str_context)
		return;

	dd_dev_ctx = dd_str_context->dd_dev_context;
	VDEC_ASSERT(dd_dev_ctx);

	/* Destroy stream in the Decoder. */
	if (dd_str_context->dec_ctx) {
		ret = decoder_stream_destroy(dd_str_context->dec_ctx,
					     false);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		dd_str_context->dec_ctx = NULL;
	}

	/* Remove any "active" picture resources allocated for this stream. */
	pict_resint = resource_list_removehead(&core_str_ctx->pict_res_list);

	while (pict_resint) {
		ret = core_free_common_picture_resource(core_str_ctx,
							pict_resint);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			return;

		pict_resint =
			resource_list_removehead(&core_str_ctx->pict_res_list);
	}

	/* Destroy the MMU context for this stream. */
	if (dd_str_context->mmu_str_handle) {
		ret = mmu_stream_destroy(dd_str_context->mmu_str_handle);

		VDEC_ASSERT(ret == IMG_SUCCESS);
		dd_str_context->mmu_str_handle = NULL;
	}

	/* Destroy the stream resources. */
	if (dd_str_context->res_buck_handle) {
		rman_destroy_bucket(dd_str_context->res_buck_handle);
		dd_str_context->res_buck_handle = NULL;
	}

	/* Free stream context. */
	kfree(dd_str_context);
}

/*
 * @Function core_fn_free_stream_unit
 */
static int core_fn_free_stream_unit(struct vdecdd_str_unit *str_unit,
				    void *param)
{
	struct core_stream_context *core_str_ctx =
		(struct core_stream_context *)param;
	u32 ret = IMG_SUCCESS;

	/* Attach picture resources where required. */
	if (str_unit->str_unit_type == VDECDD_STRUNIT_PICTURE_START)
		/*
		 * Do not force attachment because the resources can be
		 * unattached yet, e.g. in case of not yet processed picture
		 * units
		 */
		ret = core_picture_attach_resources(core_str_ctx, str_unit,
						    false);

	str_unit->decode = false;

	return ret;
}

/*
 * @Function  core_is_unsupported
 */
static u8 core_is_unsupported(struct vdec_unsupp_flags *unsupp_flags)
{
	u8 unsupported = false;

	if (unsupp_flags->str_cfg ||
	    unsupp_flags->seq_hdr ||
	    unsupp_flags->pict_hdr ||
	    unsupp_flags->str_opcfg ||
	    unsupp_flags->op_bufcfg)
		unsupported = true;

	return unsupported;
}

int core_stream_create(void *vxd_dec_ctx_arg,
		       const struct vdec_str_configdata *str_cfg_data,
		       u32 *res_str_id)
{
	int ret;
	struct vdecdd_ddstr_ctx	*dd_str_context;
	struct vdecdd_supp_check supp_check;
	struct vdecdd_dddev_context *dd_dev_ctx;
	struct core_stream_context *core_str_ctx;

	/* Validate input arguments */
	VDEC_ASSERT(str_cfg_data);
	VDEC_ASSERT(res_str_id);

	VDEC_ASSERT(global_core_ctx);
	dd_dev_ctx = global_core_ctx->dev_ctx;

	VDEC_ASSERT(dd_dev_ctx);
	if (!dd_dev_ctx)
		return IMG_ERROR_NOT_INITIALISED;

	/* Allocate Core Stream Context */
	core_str_ctx = kzalloc(sizeof(*core_str_ctx), GFP_KERNEL);
	if (!core_str_ctx)
		return IMG_ERROR_OUT_OF_MEMORY;

	core_str_ctx->core_ctx = global_core_ctx;
	core_str_ctx->vxd_dec_context = (struct vxd_dec_ctx *)vxd_dec_ctx_arg;
	/* register callback for firmware response */
	core_str_ctx->vxd_dec_context->cb = (decode_cb)core_fw_response_cb;

	lst_init(&core_str_ctx->pict_buf_list);
	lst_init(&core_str_ctx->pict_res_list);
	lst_init(&core_str_ctx->seq_hdr_list);
	lst_init(&core_str_ctx->str_unit_list);

	/* Allocate device stream context.. */
	dd_str_context = kzalloc(sizeof(*dd_str_context), GFP_KERNEL);
	VDEC_ASSERT(dd_str_context);
	if (!dd_str_context) {
		kfree(core_str_ctx);
		core_str_ctx = NULL;
		return IMG_ERROR_OUT_OF_MEMORY;
	}

	dd_str_context->dd_dev_context = dd_dev_ctx;
	core_str_ctx->dd_str_ctx = dd_str_context;

	/* Check stream configuration. */
	memset(&supp_check, 0x0, sizeof(supp_check));
	ret = core_check_decoder_support(dd_dev_ctx, str_cfg_data, NULL, NULL,
					 NULL, &supp_check);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		goto error;

	if (core_is_unsupported(&supp_check.unsupp_flags)) {
		ret = IMG_ERROR_NOT_SUPPORTED;
		goto error;
	}

	/* Create a bucket for the resources.. */
	ret = rman_create_bucket(&dd_str_context->res_buck_handle);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		goto error;

	/* Register the stream as a device resource.. */
	ret = rman_register_resource(dd_dev_ctx->res_buck_handle,
				     VDECDD_STREAM_TYPE_ID,
				     core_fn_free_stream, core_str_ctx,
				     &dd_str_context->res_handle,
				     &dd_str_context->res_str_id);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		goto error;

	/* Create unique Stream Id */
	dd_str_context->km_str_id = core_str_ctx->vxd_dec_context->stream.id;

	/*
	 * Create stream in the Decoder.
	 * NOTE: this must take place first since it creates the MMU context.
	 */
	ret = decoder_stream_create(dd_dev_ctx->dec_context, *str_cfg_data,
				    dd_str_context->km_str_id,
				    &dd_str_context->mmu_str_handle,
				    core_str_ctx->vxd_dec_context,
				    core_str_ctx, &dd_str_context->dec_ctx,
				    (void *)core_stream_processed_cb, (void *)core_decoder_queries);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		goto error;

	/* Setup stream context.. */
	dd_str_context->str_config_data	= *str_cfg_data;
	dd_str_context->dd_str_state = VDECDD_STRSTATE_STOPPED;

	pr_info("[SID=0x%08X] New stream created [USERSID=0x%08X]",
		dd_str_context->res_str_id, str_cfg_data->user_str_id);

	*res_str_id = dd_str_context->res_str_id;

	lst_add(&global_core_ctx->core_str_ctx, core_str_ctx);

	/* Return success.. */
	return IMG_SUCCESS;

error:
	if (dd_str_context->res_handle)
		rman_free_resource(dd_str_context->res_handle);
	else
		core_fn_free_stream(core_str_ctx);

	return ret;
}

static int
core_get_resource_availability(struct core_stream_context *core_str_ctx)
{
	u32 avail = ~0;

	if (resource_list_getnumavail(&core_str_ctx->pict_buf_list) == 0)
		avail &= ~CORE_AVAIL_PICTBUF;

	if (resource_list_getnumavail(&core_str_ctx->pict_res_list) == 0)
		avail &= ~CORE_AVAIL_PICTRES;

	if (global_avail_slots == 0)
		avail &= ~CORE_AVAIL_CORE;

	return avail;
}

static int
core_stream_set_pictbuf_config(struct vdecdd_ddstr_ctx *dd_str_ctx,
			       struct vdec_pict_bufconfig *pictbuf_cfg)
{
	int  ret;

	/* Validate input arguments */
	VDEC_ASSERT(dd_str_ctx);
	VDEC_ASSERT(pictbuf_cfg);

	/*
	 * If there are no buffers mapped or the configuration is not set
	 * (only done when reconfiguring output) then calculate the output
	 * picture buffer layout.
	 */
	if (dd_str_ctx->map_buf_info.num_buf == 0 ||
	    dd_str_ctx->disp_pict_buf.buf_config.buf_size == 0) {
		struct vdecdd_supp_check supp_check;
		struct vdecdd_ddpict_buf disp_pictbuf;

		memset(&disp_pictbuf, 0, sizeof(disp_pictbuf));

		disp_pictbuf.buf_config = *pictbuf_cfg;

		/*
		 * Ensure that the external picture buffer information
		 * is compatible with the hardware and convert to internal
		 * driver representation.
		 */
		ret =
		vdecddutils_convert_buffer_config(&dd_str_ctx->str_config_data,
						  &disp_pictbuf.buf_config,
						  &disp_pictbuf.rend_info);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			goto error;

		/*
		 * Provide the current state for validation against the new
		 * buffer configuration.
		 */
		memset(&supp_check, 0, sizeof(supp_check));
		supp_check.disp_pictbuf = &disp_pictbuf;

		if (dd_str_ctx->comseq_hdr_info.max_frame_size.width)
			supp_check.comseq_hdrinfo =
						&dd_str_ctx->comseq_hdr_info;

		if (dd_str_ctx->str_op_configured)
			supp_check.op_cfg = &dd_str_ctx->opconfig;

		ret =
		core_check_decoder_support(dd_str_ctx->dd_dev_context,
					   &dd_str_ctx->str_config_data,
					   &dd_str_ctx->prev_comseq_hdr_info,
					   &dd_str_ctx->prev_pict_hdr_info,
					   &dd_str_ctx->map_buf_info,
					   &supp_check);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			goto error;

		if (core_is_unsupported(&supp_check.unsupp_flags)) {
			ret = IMG_ERROR_NOT_SUPPORTED;
			goto error;
		}

		dd_str_ctx->disp_pict_buf = disp_pictbuf;
	} else {
		/*
		 * Check configuration of buffer matches that for stream
		 * including any picture buffers that are already mapped.
		 */
		if (memcmp(pictbuf_cfg, &dd_str_ctx->disp_pict_buf.buf_config,
			   sizeof(*pictbuf_cfg))) {
			/*
			 * Configuration of output buffer doesn't match the
			 * rest.
			 */
			pr_err("[SID=0x%08X] All output buffers must have the same properties.",
			       dd_str_ctx->res_str_id);
			ret = IMG_ERROR_INVALID_PARAMETERS;
			goto error;
		}
	}

	/* Return success.. */
	return IMG_SUCCESS;

error:
	return ret;
}

int
core_stream_set_output_config(u32 res_str_id,
			      struct vdec_str_opconfig *str_opcfg,
			      struct vdec_pict_bufconfig *pict_bufcfg_handle)
{
	struct vdecdd_supp_check       supp_check;
	struct vdec_pict_bufconfig     pict_buf_cfg;
	struct vdec_pict_rendinfo      disp_pict_rend_info;
	int              ret;

	struct vdecdd_ddstr_ctx	*dd_str_context;
	struct core_stream_context	*core_str_ctx;

	/*
	 * Stream based messages without a device context
	 * must have a stream ID.
	 */
	VDEC_ASSERT(res_str_id);

	/* Get access to stream context */
	ret = rman_get_resource(res_str_id, VDECDD_STREAM_TYPE_ID,
				(void **)&core_str_ctx, NULL);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	dd_str_context = core_str_ctx->dd_str_ctx;

	VDEC_ASSERT(dd_str_context);
	VDEC_ASSERT(str_opcfg);

	memset(&supp_check, 0, sizeof(supp_check));
	if (core_str_ctx->new_seq)
		supp_check.comseq_hdrinfo = &dd_str_context->comseq_hdr_info;
	else
		supp_check.comseq_hdrinfo = NULL;

	supp_check.op_cfg = str_opcfg;

	/*
	 * Validate stream output configuration against display
	 * buffer properties if no new picture buffer configuration
	 * is provided.
	 */
	if (!pict_bufcfg_handle) {
		VDEC_ASSERT(dd_str_context->disp_pict_buf.rend_info.rendered_size);
		supp_check.disp_pictbuf = &dd_str_context->disp_pict_buf;
	}

	/* Validate output configuration. */
	ret = core_check_decoder_support(dd_str_context->dd_dev_context,
					 &dd_str_context->str_config_data,
					 &dd_str_context->prev_comseq_hdr_info,
					 &dd_str_context->prev_pict_hdr_info,
					 &dd_str_context->map_buf_info,
					 &supp_check);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return IMG_SUCCESS;

	if (core_is_unsupported(&supp_check.unsupp_flags))
		return IMG_ERROR_NOT_SUPPORTED;

	/* Update the stream output configuration. */
	dd_str_context->opconfig = *str_opcfg;

	/* Mark output as configured. */
	dd_str_context->str_op_configured = true;

	if (pict_bufcfg_handle) {
		/*
		 * Clear/invalidate the latest picture buffer configuration
		 * since it is easier to reuse the set function to calculate
		 * for this new output configuration than to determine
		 * compatibility. Keep a copy beforehand just in case the new
		 * configuration is invalid.
		 */
		if (dd_str_context->disp_pict_buf.rend_info.rendered_size !=
		    0) {
			pict_buf_cfg =
				dd_str_context->disp_pict_buf.buf_config;
			disp_pict_rend_info =
				dd_str_context->disp_pict_buf.rend_info;

			memset(&dd_str_context->disp_pict_buf.buf_config, 0,
			       sizeof(dd_str_context->disp_pict_buf.buf_config));
			memset(&dd_str_context->disp_pict_buf.rend_info, 0,
			       sizeof(dd_str_context->disp_pict_buf.rend_info));
		}

		/*
		 * Recalculate the picture buffer internal layout from the
		 * externalconfiguration. These settings provided by the
		 * allocator should be adhered to since the display process
		 * will expect the decoder to use them.
		 * If the configuration is invalid we need to leave the
		 * decoder state as it was before.
		 */
		ret = core_stream_set_pictbuf_config(dd_str_context,
						     pict_bufcfg_handle);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS &&
		    dd_str_context->disp_pict_buf.rend_info.rendered_size !=
		    0) {
			/* Restore old picture buffer configuration */
			dd_str_context->disp_pict_buf.buf_config =
							pict_buf_cfg;
			dd_str_context->disp_pict_buf.rend_info =
							disp_pict_rend_info;
			return ret;
		}
	} else if (core_is_unsupported(&supp_check.unsupp_flags)) {
		return IMG_ERROR_NOT_SUPPORTED;
	}

	/* Return success.. */
	return ret;
}

/*
 * @Function core_stream_play
 */
int core_stream_play(u32 res_str_id)
{
	int ret;
	struct vdecdd_ddstr_ctx *dd_str_context;
	struct core_stream_context *core_str_ctx;
	/* Picture buffer layout to use for decoding. */
	struct vdecdd_ddpict_buf *disp_pict_buf;
	struct vdec_str_opconfig *op_cfg;
	struct vdecdd_supp_check supp_check;

	/*
	 * Stream based messages without a device context
	 * must have a stream ID.
	 */
	VDEC_ASSERT(res_str_id);

	/* Get access to stream context.. */
	ret = rman_get_resource(res_str_id, VDECDD_STREAM_TYPE_ID,
				(void **)&core_str_ctx, NULL);

	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	dd_str_context = core_str_ctx->dd_str_ctx;

	VDEC_ASSERT(dd_str_context);

	/* Ensure we are stopped. */
	VDEC_ASSERT(dd_str_context->dd_str_state == VDECDD_STRSTATE_STOPPED);

	/* Set "playing". */
	dd_str_context->dd_str_state = VDECDD_STRSTATE_PLAYING;

	/* set that is it not yet in closed GOP */
	core_str_ctx->no_prev_refs_used = true;

	disp_pict_buf = dd_str_context->disp_pict_buf.rend_info.rendered_size ?
			&dd_str_context->disp_pict_buf : NULL;
	op_cfg = dd_str_context->str_op_configured ?
		&dd_str_context->opconfig : NULL;

	if (disp_pict_buf && op_cfg) {
		VDEC_ASSERT(!disp_pict_buf->pict_buf);

		if (memcmp(&core_str_ctx->op_cfg, op_cfg,
			   sizeof(core_str_ctx->op_cfg)) ||
			   memcmp(&core_str_ctx->disp_pict_buf,
				  disp_pict_buf,
				  sizeof(core_str_ctx->disp_pict_buf)))
			core_str_ctx->new_op_cfg = true;

		core_str_ctx->disp_pict_buf = *disp_pict_buf;
		core_str_ctx->op_cfg = *op_cfg;

		core_str_ctx->opcfg_set = true;
	} else {
		core_str_ctx->opcfg_set = false;
		/* Must not be decoding without output configuration */
		VDEC_ASSERT(0);
	}

	memset(&supp_check, 0, sizeof(supp_check));

	if (vdec_size_nz(core_str_ctx->comseq_hdr_info.max_frame_size))
		supp_check.comseq_hdrinfo = &core_str_ctx->comseq_hdr_info;

	if (core_str_ctx->opcfg_set) {
		supp_check.op_cfg = &core_str_ctx->op_cfg;
		supp_check.disp_pictbuf = &core_str_ctx->disp_pict_buf;
	}
	supp_check.non_cfg_req = true;
	ret = core_check_decoder_support(dd_str_context->dd_dev_context,
					 &dd_str_context->str_config_data,
					 &dd_str_context->prev_comseq_hdr_info,
					 &dd_str_context->prev_pict_hdr_info,
					 &dd_str_context->map_buf_info,
					 &supp_check);
	if (ret != IMG_SUCCESS)
		return ret;

	/* Return success.. */
	return IMG_SUCCESS;
}

/*
 * @Function  core_deinitialise
 */
int core_deinitialise(void)
{
	struct vdecdd_dddev_context *dd_dev_ctx;
	int ret;

	dd_dev_ctx = global_core_ctx->dev_ctx;
	VDEC_ASSERT(dd_dev_ctx);
	if (!dd_dev_ctx)
		return IMG_ERROR_NOT_INITIALISED;

	ret = decoder_deinitialise(dd_dev_ctx->dec_context);
	VDEC_ASSERT(ret == IMG_SUCCESS);

	/* Free context resources.. */
	rman_destroy_bucket(dd_dev_ctx->res_buck_handle);

	kfree(dd_dev_ctx);

	global_core_ctx->dev_ctx = NULL;

	return IMG_SUCCESS;
}

static int core_get_mb_num(u32 width, u32 height)
{
	/*
	 * Calculate the number of MBs needed for current video
	 * sequence settings.
	 */
	u32 width_mb  = ALIGN(width, VDEC_MB_DIMENSION) / VDEC_MB_DIMENSION;
	u32 height_mb = ALIGN(height, 2 * VDEC_MB_DIMENSION) /
		VDEC_MB_DIMENSION;

	return width_mb * height_mb;
}

static int
core_common_bufs_getsize(struct core_stream_context *core_str_ctx,
			 const struct vdec_comsequ_hdrinfo *comseq_hdrinfo,
			 struct vdec_pict_size *max_pict_size,
			 u32 *mbparam_bufsize, u8 *res_needed)
{
	enum vdec_vid_std vid_std =
		core_str_ctx->dd_str_ctx->str_config_data.vid_std;
	u32 std_idx = vid_std - 1;
	u32 mb_num = 0;

	if (core_str_ctx->dd_str_ctx->str_config_data.vid_std >= VDEC_STD_MAX)
		return IMG_ERROR_GENERIC_FAILURE;

	/* Reset the MB parameters buffer size. */
	*mbparam_bufsize = 0;

	if (mbparam_allocinfo[std_idx].alloc_mbparam_bufs) {
		*res_needed = true;

		/*
		 * Calculate the number of MBs needed for current video
		 * sequence settings.
		 */
		mb_num =
		core_get_mb_num(max_pict_size->width, max_pict_size->height);

		/* Calculate the final number of MBs needed. */
		mb_num += mbparam_allocinfo[std_idx].overalloc_mbnum;

		/* Calculate the MB params buffer size. */
		*mbparam_bufsize = mb_num *
				   mbparam_allocinfo[std_idx].mbparam_size;

		/* Adjust the buffer size for MSVDX. */
		vdecddutils_buf_vxd_adjust_size(mbparam_bufsize);

		if (comseq_hdrinfo->separate_chroma_planes)
			*mbparam_bufsize *= 3;
	}

	return IMG_SUCCESS;
}

/*
 * @Function core_pict_res_getinfo
 */
static int
core_pict_res_getinfo(struct core_stream_context *core_str_ctx,
		      const struct vdec_comsequ_hdrinfo *comseq_hdrinfo,
		      const struct vdec_str_opconfig *op_cfg,
		      const struct vdecdd_ddpict_buf *disp_pictbuf,
		      struct core_pict_resinfo *pict_resinfo)
{
	struct vdec_pict_size coded_pict_size;
	struct dec_ctx *decctx;
	u8 res_needed = false;
	int ret;

	/* Reset the picture resource info. */
	memset(pict_resinfo, 0, sizeof(*pict_resinfo));

	coded_pict_size = comseq_hdrinfo->max_frame_size;

	core_common_bufs_getsize(core_str_ctx, comseq_hdrinfo,
				 &coded_pict_size,
				 &pict_resinfo->mb_param_bufsize, &res_needed);

	/* If any picture resources are needed... */
	if (res_needed) {
		/* Get the number of resources required. */
		ret =
		vdecddutils_get_minrequired_numpicts(&core_str_ctx->dd_str_ctx->str_config_data,
						     comseq_hdrinfo, op_cfg,
						     &pict_resinfo->pict_res_num);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			return ret;

		decctx =
			(struct dec_ctx *)global_core_ctx->dev_ctx->dec_context;

		pict_resinfo->pict_res_num +=
		decctx->num_pipes * decctx->dev_cfg->num_slots_per_pipe - 1;
	}

	return IMG_SUCCESS;
}

static int core_alloc_resbuf(struct vdecdd_ddbuf_mapinfo **buf_handle,
			     u32 size, void *mmu_handle,
			     struct vxdio_mempool mem_pool)
{
	int ret;
	struct vdecdd_ddbuf_mapinfo *buf;

	*buf_handle = kzalloc(sizeof(**buf_handle), GFP_KERNEL);
	buf = *buf_handle;
	VDEC_ASSERT(buf);
	if (buf) {
		buf->mmuheap_id = MMU_HEAP_STREAM_BUFFERS;
		pr_info("%s:%d calling MMU_StreamMalloc", __func__, __LINE__);
		ret = mmu_stream_alloc(mmu_handle, buf->mmuheap_id,
				       mem_pool.mem_heap_id,
				       mem_pool.mem_attrib, size,
				       DEV_MMU_PAGE_SIZE,
				       &buf->ddbuf_info);

		VDEC_ASSERT(ret == IMG_SUCCESS);
	} else {
		ret = IMG_ERROR_OUT_OF_MEMORY;
	}
	return ret;
}

/*
 * @Function core_stream_resource_create
 */
static int
core_stream_resource_create(struct core_stream_context *core_str_ctx,
			    u8 closed_gop, u32 mem_heap_id,
			    const struct vdec_comsequ_hdrinfo *comseq_hdrinfo,
			    const struct vdec_str_opconfig *op_cfg,
			    const struct vdecdd_ddpict_buf *disp_pict_buf)
{
	struct vdecdd_pict_resint *pict_resint = NULL;
	int ret = IMG_SUCCESS;
	struct vxdio_mempool mem_pool;
	/* allocate mb_param_bufs */
	int cnt;

	mem_pool.mem_heap_id = mem_heap_id;
	mem_pool.mem_attrib = SYS_MEMATTRIB_UNCACHED |
				SYS_MEMATTRIB_WRITECOMBINE |
				SYS_MEMATTRIB_INTERNAL;

	/*
	 * Clear the reconstructed picture buffer layout if the previous
	 * references are no longer used. Only under these circumstances
	 * should the bitstream resolution change.
	 */
	if (closed_gop) {
		memset(&core_str_ctx->recon_pictbuf.rend_info, 0,
		       sizeof(core_str_ctx->recon_pictbuf.rend_info));
		memset(&core_str_ctx->coded_pict_size, 0,
		       sizeof(core_str_ctx->coded_pict_size));
	} else {
		if (vdec_size_ne(core_str_ctx->coded_pict_size,
				 comseq_hdrinfo->max_frame_size)) {
			VDEC_ASSERT(false);
			pr_err("Coded picture size changed within the closed GOP (i.e. mismatched references)");
		}
	}

	/*
	 * Set the reconstructed buffer properties if they
	 * may have been changed.
	 */
	if (core_str_ctx->recon_pictbuf.rend_info.rendered_size == 0) {
		core_str_ctx->recon_pictbuf.rend_info =
					disp_pict_buf->rend_info;
		core_str_ctx->recon_pictbuf.buf_config =
					disp_pict_buf->buf_config;
		core_str_ctx->coded_pict_size = comseq_hdrinfo->max_frame_size;
	} else {
		if (memcmp(&disp_pict_buf->rend_info,
			   &core_str_ctx->recon_pictbuf.rend_info,
			   sizeof(core_str_ctx->recon_pictbuf.rend_info))) {
			/*
			 * Reconstructed picture buffer information has changed
			 * during a closed GOP.
			 */
			VDEC_ASSERT("Reconstructed picture buffer information cannot change within a GOP" ==
				    NULL);
			pr_err("Reconstructed picture buffer information cannot change within a GOP.");
			return IMG_ERROR_GENERIC_FAILURE;
		}
	}

	/* Logic to create only one time */
	if (!core_str_ctx->pict_resinfo.is_valid) {
		/*
		 * When demand for picture resources reduces (in quantity) the
		 * extra buffers are still retained. Preserve the existing
		 * count in case the demand increases again, at which time
		 * these residual buffers won't need to be reallocated.
		 */
		ret = core_pict_res_getinfo(core_str_ctx, comseq_hdrinfo,
					    op_cfg, disp_pict_buf,
					    &core_str_ctx->pict_resinfo);
		if (ret != IMG_SUCCESS) {
			pr_err("core_pict_res_getinfo failed[%d]\n", ret);
			return ret;
		}

		for (cnt = 0; cnt < core_str_ctx->pict_resinfo.pict_res_num;
		     cnt++) {
			pict_resint = kzalloc(sizeof(*pict_resint), GFP_KERNEL);
			if (!pict_resint)
				return IMG_ERROR_OUT_OF_MEMORY;

			if (core_str_ctx->pict_resinfo.mb_param_bufsize > 0) {
				ret =
				core_alloc_resbuf(&pict_resint->mb_param_buf,
						  core_str_ctx->pict_resinfo.mb_param_bufsize,
						  core_str_ctx->dd_str_ctx->mmu_str_handle,
						  mem_pool);
				if (ret !=  IMG_SUCCESS) {
					pr_info("core_alloc_resbuf failed[%d]\n",
						ret);
					return ret;
				}
			}
			/* Add the internal picture resources to the list. */
			ret = resource_list_add(&core_str_ctx->pict_res_list,
						pict_resint, 0,
						&pict_resint->ref_cnt);
		}
		core_str_ctx->pict_resinfo.is_valid = true;
	}

	return IMG_SUCCESS;
}

static int
core_reconfigure_recon_pictbufs(struct core_stream_context *core_str_ctx,
				u8 no_references)
{
	struct vdecdd_ddstr_ctx *dd_str_ctx;
	int ret;

	dd_str_ctx = core_str_ctx->dd_str_ctx;
	VDEC_ASSERT(dd_str_ctx->str_op_configured);

	/* Re-configure the internal picture buffers now that none are held. */
	ret =
	core_stream_resource_create(core_str_ctx, no_references,
				    dd_str_ctx->dd_dev_context->internal_heap_id,
				    &dd_str_ctx->comseq_hdr_info,
				    &dd_str_ctx->opconfig,
				    &dd_str_ctx->disp_pict_buf);
	return ret;
}

/*
 * @Function              core_picture_prepare
 */
static int core_picture_prepare(struct core_stream_context *core_str_ctx,
				struct vdecdd_str_unit *str_unit)
{
	int ret = IMG_SUCCESS;
	struct vdecdd_picture *pict_local = NULL;
	u32 avail = 0;
	u8 need_pict_res;

	/*
	 * For normal decode, setup picture data.
	 * Preallocate the picture structure.
	 */
	pict_local = kzalloc(sizeof(*pict_local), GFP_KERNEL);
	if (!pict_local)
		return IMG_ERROR_OUT_OF_MEMORY;

	/* Determine whether the picture can be decoded. */
	ret = decoder_get_load(core_str_ctx->dd_str_ctx->dec_ctx,
			       &global_avail_slots);
	if (ret != IMG_SUCCESS) {
		pr_err("No resources avaialable to decode this picture");
		ret = IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE;
		goto unwind;
	}

	/*
	 * Load and availability is cached in stream context simply
	 * for status reporting.
	 */
	avail = core_get_resource_availability(core_str_ctx);

	if ((avail & CORE_AVAIL_CORE) == 0) {
		/* Return straight away if the core is not available */
		ret = IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE;
		goto unwind;
	}

	if (core_str_ctx->new_op_cfg || core_str_ctx->new_seq) {
		/*
		 * Reconstructed buffers should be checked for reconfiguration
		 * under these conditions:
		 *     1. New output configuration,
		 *     2. New sequence.
		 * Core can decide to reset the reconstructed buffer properties
		 * if there are no previous reference pictures used
		 * (i.e. at a closed GOP). This code must go here because we
		 * may not stop when new sequence is found or references become
		 * unused.
		 */
		ret =
		core_reconfigure_recon_pictbufs(core_str_ctx,
						core_str_ctx->no_prev_refs_used);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			goto unwind;
	}

	/* Update the display information for this picture. */
	ret =
	vdecddutils_get_display_region(&str_unit->pict_hdr_info->coded_frame_size,
				       &str_unit->pict_hdr_info->disp_info.enc_disp_region,
				       &str_unit->pict_hdr_info->disp_info.disp_region);
	if (ret != IMG_SUCCESS)
		goto unwind;

	/* Clear internal state */
	core_str_ctx->new_seq = false;
	core_str_ctx->new_op_cfg = false;
	core_str_ctx->no_prev_refs_used = false;

	/*
	 * Recalculate this since we might have just created
	 * internal resources.
	 */
	core_str_ctx->res_avail = core_get_resource_availability(core_str_ctx);

	/*
	 * If picture resources were needed for this stream, picture resources
	 * list wouldn't be empty
	 */
	need_pict_res = !lst_empty(&core_str_ctx->pict_res_list);
	/* If there are resources available */
	if ((core_str_ctx->res_avail & CORE_AVAIL_PICTBUF) &&
	    (!need_pict_res ||
	     (core_str_ctx->res_avail & CORE_AVAIL_PICTRES))) {
		/* Pick internal picture resources. */
		if (need_pict_res) {
			pict_local->pict_res_int =
			resource_list_get_avail(&core_str_ctx->pict_res_list);

			VDEC_ASSERT(pict_local->pict_res_int);
			if (!pict_local->pict_res_int) {
				ret = IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE;
				goto unwind;
			}
		}

		/* Pick the client image buffer. */
		pict_local->disp_pict_buf.pict_buf =
			resource_list_get_avail(&core_str_ctx->pict_buf_list);
		VDEC_ASSERT(pict_local->disp_pict_buf.pict_buf);
		if (!pict_local->disp_pict_buf.pict_buf) {
			ret = IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE;
			goto unwind;
		}
	} else {
		/* Need resources to process picture start. */
		ret = IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE;
		goto unwind;
	}

	/* Ensure that the buffer contains layout information. */
	pict_local->disp_pict_buf.rend_info =
			core_str_ctx->disp_pict_buf.rend_info;
	pict_local->disp_pict_buf.buf_config =
			core_str_ctx->disp_pict_buf.buf_config;
	pict_local->op_config = core_str_ctx->op_cfg;
	pict_local->last_pict_in_seq = str_unit->last_pict_in_seq;

	str_unit->dd_pict_data = pict_local;

	/* Indicate that all necessary resources are now available. */
	if (core_str_ctx->res_avail != ~0) {
		pr_info("LAST AVAIL: 0x%08X\n", core_str_ctx->res_avail);
		core_str_ctx->res_avail = ~0;
	}

	/* dump decoder internal resource addresses */
	if (pict_local->pict_res_int) {
		if (pict_local->pict_res_int->mb_param_buf) {
			pr_info("[USERSID=0x%08X] MB parameter buffer device virtual address: 0x%08X",
				core_str_ctx->dd_str_ctx->str_config_data.user_str_id,
				pict_local->pict_res_int->mb_param_buf->ddbuf_info.dev_virt);
		}

		if (core_str_ctx->comseq_hdr_info.separate_chroma_planes) {
			pr_info("[USERSID=0x%08X] Display picture virtual address: LUMA 0x%08X, CHROMA 0x%08X, CHROMA2 0x%08X",
				core_str_ctx->dd_str_ctx->str_config_data.user_str_id,
				pict_local->disp_pict_buf.pict_buf->ddbuf_info.dev_virt,
				pict_local->disp_pict_buf.pict_buf->ddbuf_info.dev_virt +
				pict_local->disp_pict_buf.rend_info.plane_info[VDEC_PLANE_VIDEO_U].offset,
				pict_local->disp_pict_buf.pict_buf->ddbuf_info.dev_virt +
				pict_local->disp_pict_buf.rend_info.plane_info[VDEC_PLANE_VIDEO_V].offset);
		} else {
			pr_info("[USERSID=0x%08X] Display picture virtual address: LUMA 0x%08X, CHROMA 0x%08X",
				core_str_ctx->dd_str_ctx->str_config_data.user_str_id,
				pict_local->disp_pict_buf.pict_buf->ddbuf_info.dev_virt,
				pict_local->disp_pict_buf.pict_buf->ddbuf_info.dev_virt +
				pict_local->disp_pict_buf.rend_info.plane_info[VDEC_PLANE_VIDEO_UV].offset);
		}
	}

	ret = core_picture_attach_resources(core_str_ctx, str_unit, true);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		goto unwind;

	return IMG_SUCCESS;

unwind:
	if (pict_local && pict_local->pict_res_int) {
		resource_item_return(&pict_local->pict_res_int->ref_cnt);
		pict_local->pict_res_int = NULL;
	}
	if (pict_local && pict_local->disp_pict_buf.pict_buf) {
		resource_item_return(&pict_local->disp_pict_buf.pict_buf->ddbuf_info.ref_count);
		pict_local->disp_pict_buf.pict_buf = NULL;
	}
	kfree(pict_local);
	return ret;
}

/*
 * @Function              core_validate_new_sequence
 */
static int
core_validate_new_sequence(struct core_stream_context *core_str_ctx,
			   const struct vdec_comsequ_hdrinfo *comseq_hdrinfo)
{
	int ret;
	struct vdecdd_supp_check supp_check;
	struct vdecdd_ddstr_ctx *dd_str_ctx;
	u32 num_req_bufs_prev, num_req_bufs_cur;
	struct vdecdd_mapbuf_info mapbuf_info;

	memset(&supp_check, 0, sizeof(supp_check));

	/*
	 * Omit picture header from this setup since we can'supp_check
	 * validate this here.
	 */
	supp_check.comseq_hdrinfo = comseq_hdrinfo;

	if (core_str_ctx->opcfg_set) {
		supp_check.op_cfg = &core_str_ctx->op_cfg;
		supp_check.disp_pictbuf = &core_str_ctx->disp_pict_buf;

		ret =
		vdecddutils_get_minrequired_numpicts(&core_str_ctx->dd_str_ctx->str_config_data,
						     &core_str_ctx->comseq_hdr_info,
						     &core_str_ctx->op_cfg,
						     &num_req_bufs_prev);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			return ret;

		ret =
		vdecddutils_get_minrequired_numpicts(&core_str_ctx->dd_str_ctx->str_config_data,
						     comseq_hdrinfo,
						     &core_str_ctx->op_cfg,
						     &num_req_bufs_cur);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			return ret;
	}

	/* Check if the output configuration is compatible with new VSH. */
	dd_str_ctx = core_str_ctx->dd_str_ctx;
	mapbuf_info = dd_str_ctx->map_buf_info;

	/* Check the compatibility of the bitstream data and configuration */
	supp_check.non_cfg_req = true;
	ret = core_check_decoder_support(dd_str_ctx->dd_dev_context,
					 &dd_str_ctx->str_config_data,
					 &dd_str_ctx->prev_comseq_hdr_info,
					 &dd_str_ctx->prev_pict_hdr_info,
					 &mapbuf_info, &supp_check);
	if (ret != IMG_SUCCESS)
		return ret;

	core_str_ctx->new_seq = true;

	return IMG_SUCCESS;
}

static int
core_validate_new_picture(struct core_stream_context *core_str_ctx,
			  const struct bspp_pict_hdr_info *pict_hdrinfo,
			  u32 *features)
{
	int ret;
	struct vdecdd_supp_check supp_check;
	struct vdecdd_ddstr_ctx *dd_str_ctx;
	struct vdecdd_mapbuf_info mapbuf_info;

	memset(&supp_check, 0, sizeof(supp_check));
	supp_check.comseq_hdrinfo = &core_str_ctx->comseq_hdr_info;
	supp_check.pict_hdrinfo = pict_hdrinfo;

	/*
	 * They cannot become invalid during a sequence.
	 * However, output configuration may signal something that
	 * changes compatibility on a closed GOP within a sequence
	 * (e.g. resolution may significantly decrease
	 * in a GOP and scaling wouldn't be supported). This resolution shift
	 * would not be signalled in the sequence header
	 * (since that is the maximum) but only
	 * found now when validating the first picture in the GOP.
	 */
	if (core_str_ctx->opcfg_set)
		supp_check.op_cfg = &core_str_ctx->op_cfg;

	/*
	 * Check if the new picture is compatible with the
	 * current driver state.
	 */
	dd_str_ctx = core_str_ctx->dd_str_ctx;
	mapbuf_info = dd_str_ctx->map_buf_info;

	/* Check the compatibility of the bitstream data and configuration */
	supp_check.non_cfg_req = true;
	ret = core_check_decoder_support(dd_str_ctx->dd_dev_context,
					 &dd_str_ctx->str_config_data,
					 &dd_str_ctx->prev_comseq_hdr_info,
					 &dd_str_ctx->prev_pict_hdr_info,
					 &mapbuf_info, &supp_check);

	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	if (supp_check.unsupp_flags.str_opcfg ||
	    supp_check.unsupp_flags.pict_hdr)
		return IMG_ERROR_NOT_SUPPORTED;

	/*
	 * Clear the reconfiguration flags unless triggered by
	 * unsupported output config.
	 */
	*features = supp_check.features;

	return IMG_SUCCESS;
}

/*
 * @Function core_stream_submit_unit
 */
int core_stream_submit_unit(u32 res_str_id, struct vdecdd_str_unit *str_unit)
{
	int ret;
	u8 process_str_unit = true;

	struct vdecdd_ddstr_ctx	*dd_str_context;
	struct core_stream_context *core_str_ctx;

	/*
	 * Stream based messages without a device context
	 * must have a stream ID.
	 */
	VDEC_ASSERT(res_str_id);
	VDEC_ASSERT(str_unit);

	if (res_str_id == 0 || !str_unit) {
		pr_err("Invalid params passed to %s\n", __func__);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	/* Get access to stream context.. */
	ret = rman_get_resource(res_str_id, VDECDD_STREAM_TYPE_ID,
				(void **)&core_str_ctx, NULL);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	VDEC_ASSERT(core_str_ctx);
	dd_str_context = core_str_ctx->dd_str_ctx;
	VDEC_ASSERT(dd_str_context);

	ret = resource_list_add(&core_str_ctx->str_unit_list, str_unit, 0,
				NULL);
	VDEC_ASSERT(ret == IMG_SUCCESS);

	switch (str_unit->str_unit_type) {
	case VDECDD_STRUNIT_SEQUENCE_START:
		if (str_unit->seq_hdr_info) {
			/* Add sequence header to cache. */
			ret =
			resource_list_replace(&core_str_ctx->seq_hdr_list,
					      str_unit->seq_hdr_info,
					      str_unit->seq_hdr_info->sequ_hdr_id,
					      &str_unit->seq_hdr_info->ref_count,
					      NULL, NULL);

			if (ret != IMG_SUCCESS)
				pr_err("[USERSID=0x%08X] Failed to replace resource",
				       res_str_id);
		} else {
			/* ...or take from cache. */
			str_unit->seq_hdr_info =
			resource_list_getbyid(&core_str_ctx->seq_hdr_list,
					      str_unit->seq_hdr_id);
		}

		VDEC_ASSERT(str_unit->seq_hdr_info);
		if (!str_unit->seq_hdr_info) {
			pr_err("Sequence header information not available for current picture");
			break;
		}
		/*
		 * Check that this latest sequence header information is
		 * compatible with current state and then if no errors store
		 * as current.
		 */
		core_str_ctx->comseq_hdr_info =
			str_unit->seq_hdr_info->com_sequ_hdr_info;

		ret =
		core_validate_new_sequence(core_str_ctx,
					   &str_unit->seq_hdr_info->com_sequ_hdr_info);
		if (ret != IMG_SUCCESS)
			return ret;

		dd_str_context->prev_comseq_hdr_info =
					dd_str_context->comseq_hdr_info;
		dd_str_context->comseq_hdr_info =
				str_unit->seq_hdr_info->com_sequ_hdr_info;

		pr_info("[SID=0x%08X] VSH: Maximum Frame Resolution [%dx%d]",
			dd_str_context->res_str_id,
			dd_str_context->comseq_hdr_info.max_frame_size.width,
			dd_str_context->comseq_hdr_info.max_frame_size.height);

		break;

	case VDECDD_STRUNIT_PICTURE_START:
		/*
		 * Check that the picture configuration is compatible
		 * with the current state.
		 */
		ret = core_validate_new_picture(core_str_ctx,
						str_unit->pict_hdr_info,
						&str_unit->features);
		if (ret != IMG_SUCCESS) {
			if (ret == IMG_ERROR_NOT_SUPPORTED) {
				/*
				 * Do not process stream unit since there is
				 * something unsupported.
				 */
				process_str_unit = false;
				break;
			}
		}

		/* Prepare picture for decoding. */
		ret = core_picture_prepare(core_str_ctx, str_unit);
		if (ret != IMG_SUCCESS)
			if (ret == IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE ||
			    ret == IMG_ERROR_NOT_SUPPORTED)
				/*
				 * Do not process stream unit since there is
				 * something unsupported or resources are not
				 * available.
				 */
				process_str_unit = false;
		break;

	default:
		/*
		 * Sequence/picture headers should only be attached to
		 * corresponding units.
		 */
		VDEC_ASSERT(!str_unit->seq_hdr_info);
		VDEC_ASSERT(!str_unit->pict_hdr_info);
		break;
	}

	if (process_str_unit) {
		/* Submit stream unit to the decoder for processing. */
		str_unit->decode = true;
		ret = decoder_stream_process_unit(dd_str_context->dec_ctx,
						  str_unit);
	} else {
		ret = IMG_ERROR_GENERIC_FAILURE;
	}

	return ret;
}

/*
 * @Function  core_stream_fill_pictbuf
 */
int core_stream_fill_pictbuf(u32 buf_map_id)
{
	int ret;
	struct vdecdd_ddbuf_mapinfo *ddbuf_map_info;
	struct vdecdd_ddstr_ctx *dd_str_ctx;
	struct core_stream_context *core_str_ctx;

	/* Get access to map info context.. */
	ret = rman_get_resource(buf_map_id, VDECDD_BUFMAP_TYPE_ID,
				(void **)&ddbuf_map_info, NULL);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	dd_str_ctx = ddbuf_map_info->ddstr_context;

	/* Get access to stream context.. */
	ret = rman_get_resource(dd_str_ctx->res_str_id, VDECDD_STREAM_TYPE_ID,
				(void **)&core_str_ctx, NULL);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	/* Check buffer type. */
	VDEC_ASSERT(ddbuf_map_info->buf_type == VDEC_BUFTYPE_PICTURE);

	/* Add the image buffer to the list */
	ret = resource_list_add(&core_str_ctx->pict_buf_list, ddbuf_map_info,
				0, &ddbuf_map_info->ddbuf_info.ref_count);

	return ret;
}

/*
 * @Function              core_fn_free_mapped
 */
static void core_fn_free_mapped(void *param)
{
	struct vdecdd_ddbuf_mapinfo *ddbuf_map_info =
		(struct vdecdd_ddbuf_mapinfo *)param;

	/* Validate input arguments */
	VDEC_ASSERT(param);

	/* Do not free the MMU mapping. It is handled by talmmu code. */
	kfree(ddbuf_map_info);
}

/*
 * @Function              core_stream_map_buf
 */
int core_stream_map_buf(u32 res_str_id, enum vdec_buf_type buf_type,
			struct vdec_buf_info *buf_info, u32 *buf_map_id)
{
	int ret;
	struct vdecdd_ddstr_ctx *dd_str_ctx;
	struct core_stream_context *core_str_ctx;
	struct vdecdd_ddbuf_mapinfo *ddbuf_map_info;

	/*
	 * Stream based messages without a device context
	 * must have a stream ID.
	 */
	VDEC_ASSERT(res_str_id);
	VDEC_ASSERT(buf_type < VDEC_BUFTYPE_MAX);
	VDEC_ASSERT(buf_info);
	VDEC_ASSERT(buf_map_id);

	/* Get access to stream context.. */
	ret = rman_get_resource(res_str_id, VDECDD_STREAM_TYPE_ID,
				(void **)&core_str_ctx, NULL);

	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	dd_str_ctx = core_str_ctx->dd_str_ctx;

	VDEC_ASSERT(dd_str_ctx);

	/* Allocate an active stream unit.. */
	ddbuf_map_info = kzalloc(sizeof(*ddbuf_map_info), GFP_KERNEL);
	VDEC_ASSERT(ddbuf_map_info);

	if (!ddbuf_map_info) {
		pr_err("[SID=0x%08X] Failed to allocate memory for DD buffer map information",
		       dd_str_ctx->res_str_id);

		return IMG_ERROR_OUT_OF_MEMORY;
	}
	memset(ddbuf_map_info, 0, sizeof(*ddbuf_map_info));

	/* Save the stream context etc. */
	ddbuf_map_info->ddstr_context	= dd_str_ctx;
	ddbuf_map_info->buf_type	= buf_type;

	pr_info("%s:%d vdec2plus: vxd map buff id %d", __func__, __LINE__,
		buf_info->buf_id);
	ddbuf_map_info->buf_id = buf_info->buf_id;

	/* Register the allocation as a stream resource.. */
	ret = rman_register_resource(dd_str_ctx->res_buck_handle,
				     VDECDD_BUFMAP_TYPE_ID,
				     core_fn_free_mapped,
				     ddbuf_map_info,
				     &ddbuf_map_info->res_handle,
				     buf_map_id);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		goto error;

	ddbuf_map_info->buf_map_id = *buf_map_id;

	if (buf_type == VDEC_BUFTYPE_PICTURE) {
		if (dd_str_ctx->map_buf_info.num_buf == 0) {
			dd_str_ctx->map_buf_info.buf_size = buf_info->buf_size;
			dd_str_ctx->map_buf_info.byte_interleave =
					buf_info->pictbuf_cfg.byte_interleave;

			pr_info("[SID=0x%08X] Mapped Buffer size: %d (bytes)",
				dd_str_ctx->res_str_id, buf_info->buf_size);
		} else {
			/*
			 * Same byte interleaved setting should be used.
			 * Convert to actual bools by comparing with zero.
			 */
			if (buf_info->pictbuf_cfg.byte_interleave !=
			    dd_str_ctx->map_buf_info.byte_interleave) {
				pr_err("[SID=0x%08X] Buffer cannot be mapped since its byte interleave value (%s) is not the same as buffers already mapped (%s)",
				       dd_str_ctx->res_str_id,
				       buf_info->pictbuf_cfg.byte_interleave ?
				       "ON" : "OFF",
				       dd_str_ctx->map_buf_info.byte_interleave ?
				       "ON" : "OFF");
				ret = IMG_ERROR_INVALID_PARAMETERS;
				goto error;
			}
		}

		/* Configure the buffer.. */
		ret = core_stream_set_pictbuf_config(dd_str_ctx,
						     &buf_info->pictbuf_cfg);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			goto error;
	}

	/* Map heap from VDEC to MMU. */
	switch (buf_type) {
	case VDEC_BUFTYPE_BITSTREAM:
		ddbuf_map_info->mmuheap_id = MMU_HEAP_BITSTREAM_BUFFERS;
	break;

	case VDEC_BUFTYPE_PICTURE:
		mmu_get_heap(buf_info->pictbuf_cfg.stride[VDEC_PLANE_VIDEO_Y],
			     &ddbuf_map_info->mmuheap_id);
	break;

	default:
		VDEC_ASSERT(false);
	}

	/* Map this buffer into the MMU. */
	pr_info("----- %s:%d calling MMU_StreamMapExt", __func__, __LINE__);
	ret = mmu_stream_map_ext(dd_str_ctx->mmu_str_handle,
				 (enum mmu_eheap_id)ddbuf_map_info->mmuheap_id,
				 ddbuf_map_info->buf_id,
				 buf_info->buf_size, DEV_MMU_PAGE_SIZE,
				 buf_info->mem_attrib,
				 buf_info->cpu_linear_addr,
				 &ddbuf_map_info->ddbuf_info);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		goto error;

	if (buf_type == VDEC_BUFTYPE_PICTURE)
		dd_str_ctx->map_buf_info.num_buf++;

	/*
	 * Initialise the reference count to indicate that the client
	 * still holds the buffer.
	 */
	ddbuf_map_info->ddbuf_info.ref_count = 1;

	/* Return success.. */
	return IMG_SUCCESS;

error:
	if (ddbuf_map_info) {
		if (ddbuf_map_info->res_handle)
			rman_free_resource(ddbuf_map_info->res_handle);
		else
			kfree(ddbuf_map_info);
	}

	return ret;
}

/*
 * @Function              core_stream_map_buf_sg
 */
int core_stream_map_buf_sg(u32 res_str_id, enum vdec_buf_type buf_type,
			   struct vdec_buf_info *buf_info,
			   struct sg_table *sgt, u32 *buf_map_id)
{
	int ret;
	struct vdecdd_ddstr_ctx *dd_str_ctx;
	struct core_stream_context *core_str_ctx;
	struct vdecdd_ddbuf_mapinfo *ddbuf_map_info;

	/*
	 * Resource stream ID cannot be zero. If zero just warning and
	 * proceeding further will break the code. Return IMG_ERROR_INVALID_ID.
	 */
	if (res_str_id <= 0)
		return IMG_ERROR_INVALID_ID;

	VDEC_ASSERT(buf_type < VDEC_BUFTYPE_MAX);
	VDEC_ASSERT(buf_info);
	VDEC_ASSERT(buf_map_id);

	/* Get access to stream context.. */
	ret = rman_get_resource(res_str_id, VDECDD_STREAM_TYPE_ID,
				(void **)&core_str_ctx, NULL);

	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	dd_str_ctx = core_str_ctx->dd_str_ctx;

	VDEC_ASSERT(dd_str_ctx);

	/* Allocate an active stream unit.. */
	ddbuf_map_info = kzalloc(sizeof(*ddbuf_map_info), GFP_KERNEL);
	VDEC_ASSERT(ddbuf_map_info);

	if (!ddbuf_map_info) {
		pr_err("[SID=0x%08X] Failed to allocate memory for DD buffer map information",
		       dd_str_ctx->res_str_id);

		return IMG_ERROR_OUT_OF_MEMORY;
	}

	/* Save the stream context etc. */
	ddbuf_map_info->ddstr_context = dd_str_ctx;
	ddbuf_map_info->buf_type = buf_type;

	pr_info("%s:%d vdec2plus: vxd map buff id %d", __func__, __LINE__,
		buf_info->buf_id);
	ddbuf_map_info->buf_id = buf_info->buf_id;

	/* Register the allocation as a stream resource.. */
	ret = rman_register_resource(dd_str_ctx->res_buck_handle,
				     VDECDD_BUFMAP_TYPE_ID,
				     core_fn_free_mapped, ddbuf_map_info,
				     &ddbuf_map_info->res_handle, buf_map_id);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		goto error;

	ddbuf_map_info->buf_map_id = *buf_map_id;

	if (buf_type == VDEC_BUFTYPE_PICTURE) {
		if (dd_str_ctx->map_buf_info.num_buf == 0) {
			dd_str_ctx->map_buf_info.buf_size = buf_info->buf_size;

			dd_str_ctx->map_buf_info.byte_interleave =
					buf_info->pictbuf_cfg.byte_interleave;

			pr_info("[SID=0x%08X] Mapped Buffer size: %d (bytes)",
				dd_str_ctx->res_str_id, buf_info->buf_size);
		} else {
			/*
			 * Same byte interleaved setting should be used.
			 * Convert to actual bools by comparing with zero.
			 */
			if (buf_info->pictbuf_cfg.byte_interleave !=
			    dd_str_ctx->map_buf_info.byte_interleave) {
				pr_err("[SID=0x%08X] Buffer cannot be mapped since its byte interleave value (%s) is not the same as buffers already mapped (%s)",
				       dd_str_ctx->res_str_id,
				       buf_info->pictbuf_cfg.byte_interleave ?
				       "ON" : "OFF",
				       dd_str_ctx->map_buf_info.byte_interleave ?
				       "ON" : "OFF");
				ret = IMG_ERROR_INVALID_PARAMETERS;
				goto error;
			}
		}

		/* Configure the buffer.. */
		ret = core_stream_set_pictbuf_config(dd_str_ctx,
						     &buf_info->pictbuf_cfg);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			goto error;
	}

	/* Map heap from VDEC to MMU. */
	switch (buf_type) {
	case VDEC_BUFTYPE_BITSTREAM:
		ddbuf_map_info->mmuheap_id = MMU_HEAP_BITSTREAM_BUFFERS;
	break;

	case VDEC_BUFTYPE_PICTURE:
		mmu_get_heap(buf_info->pictbuf_cfg.stride[VDEC_PLANE_VIDEO_Y],
			     &ddbuf_map_info->mmuheap_id);
	break;

	default:
		VDEC_ASSERT(false);
	}

	/* Map this buffer into the MMU. */
	pr_info("----- %s:%d calling MMU_StreamMapExt_sg", __func__, __LINE__);
	ret =
	mmu_stream_map_ext_sg(dd_str_ctx->mmu_str_handle,
			      (enum mmu_eheap_id)ddbuf_map_info->mmuheap_id,
			      sgt, buf_info->buf_size, DEV_MMU_PAGE_SIZE,
			      buf_info->mem_attrib, buf_info->cpu_linear_addr,
			      &ddbuf_map_info->ddbuf_info,
			      &ddbuf_map_info->buf_id);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		goto error;

	if (buf_type == VDEC_BUFTYPE_PICTURE)
		dd_str_ctx->map_buf_info.num_buf++;

	/*
	 * Initialise the reference count to indicate that the client
	 * still holds the buffer.
	 */
	ddbuf_map_info->ddbuf_info.ref_count = 1;

	/* Return success.. */
	return IMG_SUCCESS;

error:
	if (ddbuf_map_info) {
		if (ddbuf_map_info->res_handle)
			rman_free_resource(ddbuf_map_info->res_handle);
		else
			kfree(ddbuf_map_info);
	}

	return ret;
}

/*
 * @Function  core_stream_unmap_buf
 */
int core_stream_unmap_buf(u32 buf_map_id)
{
	int ret;
	struct vdecdd_ddbuf_mapinfo *ddbuf_map_info;
	struct vdecdd_ddstr_ctx	*dd_str_ctx;
	struct core_stream_context *core_str_ctx;

	/* Get access to map info context.. */
	ret = rman_get_resource(buf_map_id, VDECDD_BUFMAP_TYPE_ID,
				(void **)&ddbuf_map_info, NULL);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	dd_str_ctx = ddbuf_map_info->ddstr_context;
	VDEC_ASSERT(dd_str_ctx);

	/* Get access to stream context.. */
	ret = rman_get_resource(dd_str_ctx->res_str_id, VDECDD_STREAM_TYPE_ID,
				(void **)&core_str_ctx, NULL);
	VDEC_ASSERT(core_str_ctx);

	pr_info("UNMAP: PM [0x%p] --> VM [0x%08X - 0x%08X] (%d bytes)",
		ddbuf_map_info->ddbuf_info.cpu_virt,
		ddbuf_map_info->ddbuf_info.dev_virt,
		ddbuf_map_info->ddbuf_info.dev_virt +
		ddbuf_map_info->ddbuf_info.buf_size,
		ddbuf_map_info->ddbuf_info.buf_size);

	/* Buffer should only be held by the client. */
	VDEC_ASSERT(ddbuf_map_info->ddbuf_info.ref_count == 1);
	if (ddbuf_map_info->ddbuf_info.ref_count != 1)
		return IMG_ERROR_MEMORY_IN_USE;

	ddbuf_map_info->ddbuf_info.ref_count = 0;
	if (ddbuf_map_info->buf_type == VDEC_BUFTYPE_PICTURE) {
		/* Remove this picture buffer from pictbuf list */
		ret = resource_list_remove(&core_str_ctx->pict_buf_list,
					   ddbuf_map_info);

		VDEC_ASSERT(ret == IMG_SUCCESS ||
			    ret == IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE);
		if (ret != IMG_SUCCESS &&
		    ret != IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE)
			return ret;

		ddbuf_map_info->ddstr_context->map_buf_info.num_buf--;

		/* Clear some state if there are no more mapped buffers. */
		if (dd_str_ctx->map_buf_info.num_buf == 0) {
			dd_str_ctx->map_buf_info.buf_size = 0;
			dd_str_ctx->map_buf_info.byte_interleave = false;
		}
	}

	/* Unmap this buffer from the MMU. */
	ret = mmu_free_mem(dd_str_ctx->mmu_str_handle,
			   &ddbuf_map_info->ddbuf_info);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	/* Free buffer map info. */
	rman_free_resource(ddbuf_map_info->res_handle);

	/* Return success.. */
	return IMG_SUCCESS;
}

/*
 * @Function              core_stream_unmap_buf_sg
 */
int core_stream_unmap_buf_sg(u32 buf_map_id)
{
	int ret;
	struct vdecdd_ddbuf_mapinfo *ddbuf_map_info;
	struct vdecdd_ddstr_ctx	*dd_str_ctx;
	struct core_stream_context *core_str_ctx;

	/* Get access to map info context.. */
	ret = rman_get_resource(buf_map_id, VDECDD_BUFMAP_TYPE_ID,
				(void **)&ddbuf_map_info, NULL);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	dd_str_ctx = ddbuf_map_info->ddstr_context;
	VDEC_ASSERT(dd_str_ctx);

	/* Get access to stream context.. */
	ret = rman_get_resource(dd_str_ctx->res_str_id, VDECDD_STREAM_TYPE_ID,
				(void **)&core_str_ctx, NULL);
	VDEC_ASSERT(core_str_ctx);

	pr_info("UNMAP: PM [0x%p] --> VM [0x%08X - 0x%08X] (%d bytes)",
		ddbuf_map_info->ddbuf_info.cpu_virt,
		ddbuf_map_info->ddbuf_info.dev_virt,
		ddbuf_map_info->ddbuf_info.dev_virt +
		ddbuf_map_info->ddbuf_info.buf_size,
		ddbuf_map_info->ddbuf_info.buf_size);

	/* Buffer should only be held by the client. */
	VDEC_ASSERT(ddbuf_map_info->ddbuf_info.ref_count == 1);
	if (ddbuf_map_info->ddbuf_info.ref_count != 1)
		return IMG_ERROR_MEMORY_IN_USE;

	ddbuf_map_info->ddbuf_info.ref_count = 0;

	if (ddbuf_map_info->buf_type == VDEC_BUFTYPE_PICTURE) {
		/* Remove this picture buffer from pictbuf list */
		ret = resource_list_remove(&core_str_ctx->pict_buf_list,
					   ddbuf_map_info);

		VDEC_ASSERT(ret == IMG_SUCCESS ||
			    ret == IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE);
		if (ret != IMG_SUCCESS &&
		    ret != IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE)
			return ret;

		ddbuf_map_info->ddstr_context->map_buf_info.num_buf--;

		/*
		 * Clear some state if there are no more
		 * mapped buffers.
		 */
		if (dd_str_ctx->map_buf_info.num_buf == 0) {
			dd_str_ctx->map_buf_info.buf_size = 0;
			dd_str_ctx->map_buf_info.byte_interleave = false;
		}
	}

	/* Unmap this buffer from the MMU. */
	ret = mmu_free_mem_sg(dd_str_ctx->mmu_str_handle,
			      &ddbuf_map_info->ddbuf_info);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	/* Free buffer map info. */
	rman_free_resource(ddbuf_map_info->res_handle);

	/* Return success.. */
	return IMG_SUCCESS;
}

/*
 * @Function              core_stream_flush
 */
int core_stream_flush(u32 res_str_id, u8 discard_refs)
{
	struct vdecdd_ddstr_ctx *dd_str_ctx;
	struct core_stream_context *core_str_ctx;
	int  ret;

	/* Get access to stream context.. */
	ret = rman_get_resource(res_str_id, VDECDD_STREAM_TYPE_ID,
				(void **)&core_str_ctx, NULL);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	dd_str_ctx = core_str_ctx->dd_str_ctx;

	VDEC_ASSERT(dd_str_ctx);
	VDEC_ASSERT(dd_str_ctx->dd_str_state == VDECDD_STRSTATE_STOPPED);

	/*
	 * If unsupported sequence is found, we need to do additional
	 * check for DPB flush condition
	 */
	if (!dd_str_ctx->comseq_hdr_info.not_dpb_flush) {
		ret = decoder_stream_flush(dd_str_ctx->dec_ctx, discard_refs);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			return ret;
	}

	/* Return success.. */
	return IMG_SUCCESS;
}

int core_stream_release_bufs(u32 res_str_id, enum vdec_buf_type buf_type)
{
	int ret;
	struct core_stream_context *core_str_ctx;
	struct vdecdd_ddstr_ctx *dd_str_ctx;

	/* Get access to stream context.. */
	ret = rman_get_resource(res_str_id, VDECDD_STREAM_TYPE_ID,
				(void **)&core_str_ctx, NULL);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	dd_str_ctx = core_str_ctx->dd_str_ctx;

	VDEC_ASSERT(dd_str_ctx);
	VDEC_ASSERT(buf_type < VDEC_BUFTYPE_MAX);

	switch (buf_type) {
	case VDEC_BUFTYPE_PICTURE:
		{
		/* Empty all the decoded picture related buffer lists. */
		ret = resource_list_empty(&core_str_ctx->pict_buf_list, true,
					  NULL, NULL);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		break;
		}

	case VDEC_BUFTYPE_BITSTREAM:
		{
		/* Empty the stream unit queue. */
		ret =
		resource_list_empty(&core_str_ctx->str_unit_list, false,
				    (resource_pfn_freeitem)core_fn_free_stream_unit,
				    core_str_ctx);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		break;
		}

	case VDEC_BUFTYPE_ALL:
		{
		/* Empty all the decoded picture related buffer lists. */
		ret = resource_list_empty(&core_str_ctx->pict_buf_list, true, NULL, NULL);
		VDEC_ASSERT(ret == IMG_SUCCESS);

		/* Empty the stream unit queue. */
		ret =
		resource_list_empty(&core_str_ctx->str_unit_list, false,
				    (resource_pfn_freeitem)core_fn_free_stream_unit,
				    core_str_ctx);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		break;
		}

	default:
		{
		ret = IMG_ERROR_INVALID_PARAMETERS;
		VDEC_ASSERT(false);
		break;
		}
	}

	if (buf_type == VDEC_BUFTYPE_PICTURE || buf_type == VDEC_BUFTYPE_ALL) {
		ret = decoder_stream_release_buffers(dd_str_ctx->dec_ctx);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			return ret;
	}

	/* Return success.. */
	return IMG_SUCCESS;
}

int core_stream_get_status(u32 res_str_id,
			   struct vdecdd_decstr_status *str_st)
{
	int ret;
	struct core_stream_context *core_str_ctx;
	struct vdecdd_ddstr_ctx *dd_str_ctx;

	/* Get access to stream context.. */
	ret = rman_get_resource(res_str_id, VDECDD_STREAM_TYPE_ID,
				(void **)&core_str_ctx, NULL);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	dd_str_ctx = core_str_ctx->dd_str_ctx;

	VDEC_ASSERT(dd_str_ctx);
	VDEC_ASSERT(str_st);

	ret = decoder_stream_get_status(dd_str_ctx->dec_ctx, str_st);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	/* Return success.. */
	return IMG_SUCCESS;
}

