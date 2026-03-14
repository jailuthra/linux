// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 Renesas Electronics Corp.
 * Copyright 2025 Niklas Söderlund <niklas.soderlund@ragnatech.se>
 */

#include "rpp_module.h"

#define FILT_VERSION_REG		0x0000

#define DEMOSAIC_REG			0x0004
#define DEMOSAIC_DEMOSAIC_BYPASS	BIT(16)
#define DEMOSAIC_DEMOSAIC_TH_MASK	GENMASK(15, 0)

#define FILT_MODE_REG			0x0008
#define FILT_MODE_FILT_LP_SELECT_MASK	GENMASK(11, 8)
#define FILT_MODE_FILT_CHR_H_MODE_MASK	GENMASK(7, 6)
#define FILT_MODE_FILT_CHR_V_MODE_MASK	GENMASK(5, 4)
#define FILT_MODE_FILT_MODE		BIT(1)
#define FILT_MODE_FILT_ENABLE		BIT(0)

#define FILT_THRESH_BL0_REG		0x000c
#define FILT_THRESH_BL1_REG		0x0010
#define FILT_THRESH_SH0_REG		0x0014
#define FILT_THRESH_SH1_REG		0x0018
#define FILT_LUM_WEIGHT_REG		0x001c
#define FILT_FAC_SH1_REG		0x0020
#define FILT_FAC_SH0_REG		0x0024
#define FILT_FAC_MID_REG		0x0028
#define FILT_FAC_BL0_REG		0x002c
#define FILT_FAC_BL1_REG		0x0030

static int rppx1_db_probe(struct rpp_module *mod)
{
	/* Version check. */
	if (rpp_module_read(mod, FILT_VERSION_REG) != 5)
		return -EINVAL;

	return 0;
}

static int
rppx1_db_fill_params_flt(struct rpp_module *mod,
			 const union rppx1_params_block *block,
			 rppx1_reg_write write, void *priv)
{
	const struct rppx1_params_flt_config *cfg = &block->flt;

	/* If the modules is disabled, simply bypass it. */
	if (cfg->header.flags & V4L2_ISP_PARAMS_FL_BLOCK_DISABLE) {
		write(priv, mod->base + FILT_MODE_REG, 0);
		return 0;
	}

	/* Native values are at RPP 18-bit precision. */
	write(priv, mod->base + FILT_THRESH_BL0_REG, cfg->thresh_bl0);
	write(priv, mod->base + FILT_THRESH_BL0_REG, cfg->thresh_bl1);
	write(priv, mod->base + FILT_THRESH_SH0_REG, cfg->thresh_sh0);
	write(priv, mod->base + FILT_THRESH_SH1_REG, cfg->thresh_sh1);

	/* Native values are at RPP 8-bit precision. */
	write(priv, mod->base + FILT_FAC_BL0_REG, cfg->fac_bl0);
	write(priv, mod->base + FILT_FAC_BL1_REG, cfg->fac_bl1);
	write(priv, mod->base + FILT_FAC_MID_REG, cfg->fac_mid);
	write(priv, mod->base + FILT_FAC_SH0_REG, cfg->fac_sh0);
	write(priv, mod->base + FILT_FAC_SH1_REG, cfg->fac_sh1);

	/*
	 * The lum_weight field is provided in RPP register format:
	 *
	 * 31		unused
	 * 30:28	lum_weight_gain
	 * 27:24	unused
	 * 23:12	lum_weight_kink
	 * 11:0		lum_weight_min
	 */
	write(priv, mod->base + FILT_LUM_WEIGHT_REG, cfg->lum_weight);

	write(priv, mod->base + FILT_MODE_REG,
	      (cfg->chr_v_mode << 4) |
	      (cfg->chr_h_mode << 6) |
	      (cfg->grn_stage1 << 8) |
	      (cfg->mode ? FILT_MODE_FILT_MODE : 0) |
	      FILT_MODE_FILT_ENABLE);

	return 0;
}

static int
rppx1_db_fill_params_bdm(struct rpp_module *mod,
			 const union rppx1_params_block *block,
			 rppx1_reg_write write, void *priv)
{
	const struct rppx1_params_bdm_config *cfg = &block->bdm;

	/* If the modules is disabled, simply bypass it. */
	if (cfg->header.flags & V4L2_ISP_PARAMS_FL_BLOCK_DISABLE) {
		write(priv, mod->base + DEMOSAIC_REG, 0x400);
		return 0;
	}

	/* Native threshold is at RPP 16-bit precision. */
	write(priv, mod->base + DEMOSAIC_REG, cfg->demosaic_th);

	return 0;
}

static int
rppx1_db_fill_params(struct rpp_module *mod,
		     const union rppx1_params_block *block,
		     rppx1_reg_write write, void *priv)
{
	switch (block->header.type) {
	case RPPX1_PARAMS_BLOCK_TYPE_FLT:
		return rppx1_db_fill_params_flt(mod, block, write, priv);
	case RPPX1_PARAMS_BLOCK_TYPE_BDM:
		return rppx1_db_fill_params_bdm(mod, block, write, priv);
	}

	return -EINVAL;
}

const struct rpp_module_ops rppx1_db_ops = {
	.probe = rppx1_db_probe,
	.fill_params = rppx1_db_fill_params,
};
