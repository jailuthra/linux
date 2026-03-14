// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 Renesas Electronics Corp.
 * Copyright 2025 Niklas Söderlund <niklas.soderlund@ragnatech.se>
 */

#include "rppx1.h"

void rppx1_stats_fill_isr(struct rppx1 *rpp, u32 isc, void *buf)
{
	struct rppx1_stat_buffer *stats = buf;

	stats->meas_type = 0;

	if (isc & RPPX1_IRQ_ID_POST_AWB_MEAS)
		if (!rpp_module_call(&rpp->post.wbmeas, fill_stats, &stats->params))
			stats->meas_type |= RPPX1_STAT_AWB;

	if (isc & RPPX1_IRQ_ID_POST_HIST_MEAS)
		if (!rpp_module_call(&rpp->post.hist, fill_stats, &stats->params))
			stats->meas_type |= RPPX1_STAT_HIST;

	if (isc & RPPX1_IRQ_ID_PRE1_EXM) {
		if (!rpp_module_call(&rpp->pre1.exm, fill_stats, &stats->params))
			stats->meas_type |= RPPX1_STAT_AUTOEXP;

		rpp_module_call(&rpp->pre1.bls, fill_stats, &stats->params);
	}
}
EXPORT_SYMBOL_GPL(rppx1_stats_fill_isr);
