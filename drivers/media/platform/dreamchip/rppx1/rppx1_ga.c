// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 Renesas Electronics Corp.
 * Copyright 2025 Niklas Söderlund <niklas.soderlund@ragnatech.se>
 */

#include "rpp_module.h"

#define GAMMA_OUT_VERSION_REG			0x0000

#define GAMMA_OUT_ENABLE_REG			0x0004
#define GAMMA_OUT_ENABLE_GAMMA_OUT_EN		BIT(0)

#define GAMMA_OUT_MODE_REG			0x0008
#define GAMMA_OUT_MODE_GAMMA_OUT_EQU_SEGM	BIT(0)

#define GAMMA_OUT_Y_REG_NUM			17
#define GAMMA_OUT_Y_REG(n)			(0x000c + (4 * (n)))

static int rppx1_ga_probe(struct rpp_module *mod)
{
	/* Version check. */
	switch (rpp_module_read(mod, GAMMA_OUT_VERSION_REG)) {
	case 1:
		mod->info.ga.colorbits = 12;
		break;
	case 2:
		mod->info.ga.colorbits = 24;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int rppx1_ga_start(struct rpp_module *mod,
			  const struct v4l2_mbus_framefmt *fmt)
{
	/* Disable stage. */
	rpp_module_write(mod, GAMMA_OUT_ENABLE_REG, 0);

	return 0;
}

static int
rppx1_ga_fill_params(struct rpp_module *mod,
		     const union rppx1_params_block *block,
		     rppx1_reg_write write, void *priv)
{
	const struct rppx1_params_goc_config *cfg = &block->goc;

	/* If the modules is disabled, simply bypass it. */
	if (cfg->header.flags & V4L2_ISP_PARAMS_FL_BLOCK_DISABLE) {
		write(priv, mod->base + GAMMA_OUT_ENABLE_REG, 0);
		return 0;
	}

	write(priv, mod->base + GAMMA_OUT_MODE_REG,
	      cfg->mode ? GAMMA_OUT_ENABLE_GAMMA_OUT_EN : 0);

	/*
	 * The native params are 24-bit while the RPP can be 12 or 24 bit.
	 * Figure out how much we need to adjust the input values.
	 */
	const unsigned int shift = 24 - mod->info.ga.colorbits;

	for (unsigned int i = 0; i < RPPX1_GAMMA_OUT_MAX_SAMPLES; i++)
		write(priv, mod->base + GAMMA_OUT_Y_REG(i),
		      cfg->gamma_y[i] >> shift);

	/* Enable module. */
	write(priv, mod->base + GAMMA_OUT_ENABLE_REG,
	      GAMMA_OUT_ENABLE_GAMMA_OUT_EN);

	return 0;
}

const struct rpp_module_ops rppx1_ga_ops = {
	.probe = rppx1_ga_probe,
	.start = rppx1_ga_start,
	.fill_params = rppx1_ga_fill_params,
};
