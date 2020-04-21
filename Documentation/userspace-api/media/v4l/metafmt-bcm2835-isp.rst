.. SPDX-License-Identifier: GPL-2.0
.. c:namespace:: V4L

***********************************************************************************
V4L2_META_FMT_BCM2835_ISP_PARAMS ('BCMP'), V4L2_META_FMT_BCM2835_ISP_STATS ('BSTA')
***********************************************************************************

.. _v4l2-meta-fmt-bcm2835-isp-params:

BCM2835 ISP parameters
======================

The ISP parameters are configured by queuing buffers to the "bcm2835-isp-params"
metadata output node. Parameter buffers use the V4L2 extensible parameters
format described in :ref:`v4l2-isp`. Userspace assembles one or more parameter
blocks in the data area of :c:type:`v4l2_isp_params_buffer` and submits the
buffer to the driver.

Each block begins with a :c:type:`v4l2_isp_params_block_header` and embeds the
block-specific payload. The header ``type`` must be set to a value from
:c:type:`bcm2835_isp_param_block_type`, ``size`` must match the block size, and
``flags`` can be used to enable or disable the block.

Example: enqueue two parameter blocks (black level and gamma)
-------------------------------------------------------------

.. code-block:: c

	struct v4l2_isp_buffer *params =
		(struct v4l2_isp_buffer *)buffer;

	params->version = BCM2835_ISP_PARAM_BUFFER_V1;
	params->data_size = 0;

	void *data = (void *)params->data;

	struct bcm2835_isp_params_black_level *black =
		(struct bcm2835_isp_params_black_level *)data;

	black->header.type = BCM2835_ISP_PARAM_BLOCK_BLACK_LEVEL;
	black->header.flags |= V4L2_ISP_PARAMS_FL_BLOCK_ENABLE;
	black->header.size = sizeof(*black);

	black->black_level.enabled = 1;
	black->black_level.black_level_r = 64;
	black->black_level.black_level_g = 64;
	black->black_level.black_level_b = 64;

	data += sizeof(*black);
	params->data_size += sizeof(*black);

	struct bcm2835_isp_params_gamma *gamma =
		(struct bcm2835_isp_params_gamma *)data;

	gamma->header.type = BCM2835_ISP_PARAM_BLOCK_GAMMA;
	gamma->header.flags |= V4L2_ISP_PARAMS_FL_BLOCK_ENABLE;
	gamma->header.size = sizeof(*gamma);

	gamma->gamma.enabled = 1;
	/* Fill gamma->gamma.x[] and gamma->gamma.y[] here */

	params->data_size += sizeof(*gamma);

The total payload size must not exceed :c:macro:`BCM2835_ISP_PARAMS_MAX_SIZE`.
The driver applies parameter buffers on the next frame boundary.

.. _v4l2-meta-fmt-bcm2835-isp-stats:

BCM2835 ISP Statistics
======================

The BCM2835 ISP hardware calculate image statistics for an input Bayer frame.
These statistics are obtained from the "bcm2835-isp-stats2" device node
using the :c:type:`v4l2_meta_format` interface. They are formatted as described
by the :c:type:`bcm2835_isp_stats` structure below.

Example: read AWB region sums
-----------------------------

.. code-block:: c

	#define DEFAULT_AWB_REGIONS_X 16
	#define DEFAULT_AWB_REGIONS_Y 12
	#define AWB_REGIONS (DEFAULT_AWB_REGIONS_X * DEFAULT_AWB_REGIONS_Y)

	const struct bcm2835_isp_stats *stats =
		(const struct bcm2835_isp_stats *)buf;

	for (unsigned int i = 0; i < AWB_REGIONS; i++) {
		const struct bcm2835_isp_stats_region *r = &stats->awb_stats[i];

		printf("awb_stats[%u]: r %llu g %llu b %llu (counted %u)\n",
		       i, r->r_sum, r->g_sum, r->b_sum, r->counted);
	}

BCM2835 ISP uAPI data types
===========================

.. kernel-doc:: include/uapi/linux/media/broadcom/bcm2835-isp.h
   :functions: bcm2835_isp_awb_gains bcm2835_isp_ccm bcm2835_isp_gain_format
                bcm2835_isp_digital_gain bcm2835_isp_lens_shading
                bcm2835_isp_black_level bcm2835_isp_geq bcm2835_isp_gamma
                bcm2835_isp_denoise bcm2835_isp_sharpen bcm2835_isp_dpc_mode
                bcm2835_isp_dpc bcm2835_isp_rational bcm2835_isp_cdn

.. kernel-doc:: include/uapi/linux/media/broadcom/bcm2835-isp.h
   :functions: bcm2835_isp_stats_hist bcm2835_isp_stats_region
	             bcm2835_isp_stats_focus bcm2835_isp_stats
