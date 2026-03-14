// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 Renesas Electronics Corp.
 * Copyright 2025 Niklas Söderlund <niklas.soderlund@ragnatech.se>
 */

#include <media/v4l2-isp.h>
#include <media/videobuf2-v4l2.h>

#include "rppx1.h"

#define RPPX1_PARAMS_BLOCK_INFO(block, data) \
	[RPPX1_PARAMS_BLOCK_TYPE_ ## block] = { \
		.size = sizeof(struct rppx1_params_ ## data ## _config), \
	}

static const struct v4l2_isp_params_block_type_info
rppx1_ext_params_blocks_info[] = {
	RPPX1_PARAMS_BLOCK_INFO(BLS, bls),
	RPPX1_PARAMS_BLOCK_INFO(AWB_GAIN, awb_gain),
	RPPX1_PARAMS_BLOCK_INFO(FLT, flt),
	RPPX1_PARAMS_BLOCK_INFO(BDM, bdm),
	RPPX1_PARAMS_BLOCK_INFO(CTK, ctk),
	RPPX1_PARAMS_BLOCK_INFO(GOC, goc),
	RPPX1_PARAMS_BLOCK_INFO(LSC, lsc),
	RPPX1_PARAMS_BLOCK_INFO(AWB_MEAS, awb_meas),
	RPPX1_PARAMS_BLOCK_INFO(HST_MEAS, hst),
	RPPX1_PARAMS_BLOCK_INFO(AEC_MEAS, aec),
};

int rppx1_params(struct rppx1 *rpp, struct vb2_buffer *vb, size_t max_size,
		 rppx1_reg_write write, void *priv)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct v4l2_isp_params_buffer *cfg;
	size_t block_offset;
	int ret;

	ret = v4l2_isp_params_validate_buffer_size(rpp->dev, vb, max_size);
	if (ret)
		return ret;

	cfg = vb2_plane_vaddr(&vbuf->vb2_buf, 0);

	ret = v4l2_isp_params_validate_buffer(rpp->dev, vb,
					      (struct v4l2_isp_params_buffer *)cfg,
					      rppx1_ext_params_blocks_info,
					      ARRAY_SIZE(rppx1_ext_params_blocks_info));
	if (ret)
		return ret;

	/* Walk the list of parameter blocks and process them. */
	block_offset = 0;
	while (block_offset < cfg->data_size) {
		const union rppx1_params_block *block =
			(const union rppx1_params_block *)&cfg->data[block_offset];
		struct rpp_module *module;
		int ret;

		block_offset += block->header.size;

		switch (block->header.type) {
		case RPPX1_PARAMS_BLOCK_TYPE_BLS:
			module = &rpp->pre1.bls;
			break;
		case RPPX1_PARAMS_BLOCK_TYPE_AWB_GAIN:
			module = &rpp->pre1.awbg;
			break;
		case RPPX1_PARAMS_BLOCK_TYPE_FLT:
		case RPPX1_PARAMS_BLOCK_TYPE_BDM:
			/* Both types handled by the same block. */
			module = &rpp->post.db;
			break;
		case RPPX1_PARAMS_BLOCK_TYPE_CTK:
			module = &rpp->post.ccor;
			break;
		case RPPX1_PARAMS_BLOCK_TYPE_GOC:
			module = &rpp->hv.ga;
			break;
		case RPPX1_PARAMS_BLOCK_TYPE_LSC:
			module = &rpp->pre1.lsc;
			break;
		case RPPX1_PARAMS_BLOCK_TYPE_AWB_MEAS:
			module = &rpp->post.wbmeas;
			break;
		case RPPX1_PARAMS_BLOCK_TYPE_HST_MEAS:
			module = &rpp->post.hist;
			break;
		case RPPX1_PARAMS_BLOCK_TYPE_AEC_MEAS:
			module = &rpp->pre1.exm;
			break;
		default:
			module = NULL;
			break;
		}

		if (!module) {
			pr_warn("Not handled RPPX1 block type: 0x%04x\n", block->header.type);
			continue;
		}

		ret = rpp_module_call(module, fill_params, block, write, priv);
		if (ret) {
			pr_err("Error processing RPPX1 block type: 0x%04x\n", block->header.type);
			return ret;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(rppx1_params);
