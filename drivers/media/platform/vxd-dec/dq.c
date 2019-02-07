// SPDX-License-Identifier: GPL-2.0
/*
 * Utility module for doubly linked queues.
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
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "dq.h"
#include "vdec_defs.h"

void dq_init(struct dq_linkage_t *queue)
{
	queue->fwd = (struct dq_linkage_t *)queue;
	queue->back = (struct dq_linkage_t *)queue;
}

void dq_addhead(struct dq_linkage_t *queue, void *item)
{
	VDEC_ASSERT(((struct dq_linkage_t *)queue)->back);
	VDEC_ASSERT(((struct dq_linkage_t *)queue)->fwd);

	if (!((struct dq_linkage_t *)queue)->back ||
	    !((struct dq_linkage_t *)queue)->fwd)
		return;

	((struct dq_linkage_t *)item)->back = (struct dq_linkage_t *)queue;
	((struct dq_linkage_t *)item)->fwd =
					((struct dq_linkage_t *)queue)->fwd;
	((struct dq_linkage_t *)queue)->fwd->back = (struct dq_linkage_t *)item;
	((struct dq_linkage_t *)queue)->fwd = (struct dq_linkage_t *)item;
}

void dq_addtail(struct dq_linkage_t *queue, void *item)
{
	VDEC_ASSERT(((struct dq_linkage_t *)queue)->back);
	VDEC_ASSERT(((struct dq_linkage_t *)queue)->fwd);

	if (!((struct dq_linkage_t *)queue)->back ||
	    !((struct dq_linkage_t *)queue)->fwd)
		return;

	((struct dq_linkage_t *)item)->fwd = (struct dq_linkage_t *)queue;
	((struct dq_linkage_t *)item)->back =
					((struct dq_linkage_t *)queue)->back;
	((struct dq_linkage_t *)queue)->back->fwd = (struct dq_linkage_t *)item;
	((struct dq_linkage_t *)queue)->back = (struct dq_linkage_t *)item;
}

int dq_empty(struct dq_linkage_t *queue)
{
	VDEC_ASSERT(((struct dq_linkage_t *)queue)->back);
	VDEC_ASSERT(((struct dq_linkage_t *)queue)->fwd);

	if (!((struct dq_linkage_t *)queue)->back ||
	    !((struct dq_linkage_t *)queue)->fwd)
		return 1;

	return ((queue)->fwd == (struct dq_linkage_t *)(queue));
}

void *dq_first(struct dq_linkage_t *queue)
{
	struct dq_linkage_t *temp = queue->fwd;

	VDEC_ASSERT(((struct dq_linkage_t *)queue)->back);
	VDEC_ASSERT(((struct dq_linkage_t *)queue)->fwd);

	if (!((struct dq_linkage_t *)queue)->back ||
	    !((struct dq_linkage_t *)queue)->fwd)
		return NULL;

	return temp == (struct dq_linkage_t *)queue ? NULL : temp;
}

void *dq_last(struct dq_linkage_t *queue)
{
	struct dq_linkage_t *temp = queue->back;

	VDEC_ASSERT(((struct dq_linkage_t *)queue)->back);
	VDEC_ASSERT(((struct dq_linkage_t *)queue)->fwd);

	if (!((struct dq_linkage_t *)queue)->back ||
	    !((struct dq_linkage_t *)queue)->fwd)
		return NULL;

	return temp == (struct dq_linkage_t *)queue ? NULL : temp;
}

void *dq_next(void *item)
{
	VDEC_ASSERT(((struct dq_linkage_t *)item)->back);
	VDEC_ASSERT(((struct dq_linkage_t *)item)->fwd);

	if (!((struct dq_linkage_t *)item)->back ||
	    !((struct dq_linkage_t *)item)->fwd)
		return NULL;

	return ((struct dq_linkage_t *)item)->fwd;
}

void *dq_previous(void *item)
{
	VDEC_ASSERT(((struct dq_linkage_t *)item)->back);
	VDEC_ASSERT(((struct dq_linkage_t *)item)->fwd);

	if (!((struct dq_linkage_t *)item)->back ||
	    !((struct dq_linkage_t *)item)->fwd)
		return NULL;

	return ((struct dq_linkage_t *)item)->back;
}

void dq_remove(void *item)
{
	VDEC_ASSERT(((struct dq_linkage_t *)item)->back);
	VDEC_ASSERT(((struct dq_linkage_t *)item)->fwd);

	if (!((struct dq_linkage_t *)item)->back ||
	    !((struct dq_linkage_t *)item)->fwd)
		return;

	((struct dq_linkage_t *)item)->fwd->back =
					((struct dq_linkage_t *)item)->back;
	((struct dq_linkage_t *)item)->back->fwd =
					((struct dq_linkage_t *)item)->fwd;

	/* make item linkages safe for "orphan" removes */
	((struct dq_linkage_t *)item)->fwd = item;
	((struct dq_linkage_t *)item)->back = item;
}

void *dq_removehead(struct dq_linkage_t *queue)
{
	struct dq_linkage_t *temp;

	VDEC_ASSERT(((struct dq_linkage_t *)queue)->back);
	VDEC_ASSERT(((struct dq_linkage_t *)queue)->fwd);

	if (!((struct dq_linkage_t *)queue)->back ||
	    !((struct dq_linkage_t *)queue)->fwd)
		return NULL;

	if ((queue)->fwd == (struct dq_linkage_t *)(queue))
		return NULL;

	temp = ((struct dq_linkage_t *)queue)->fwd;
	temp->fwd->back = temp->back;
	temp->back->fwd = temp->fwd;

	/* make item linkages safe for "orphan" removes */
	temp->fwd = temp;
	temp->back = temp;
	return temp;
}

void *dq_removetail(struct dq_linkage_t *queue)
{
	struct dq_linkage_t *temp;

	VDEC_ASSERT(((struct dq_linkage_t *)queue)->back);
	VDEC_ASSERT(((struct dq_linkage_t *)queue)->fwd);

	if (!((struct dq_linkage_t *)queue)->back ||
	    !((struct dq_linkage_t *)queue)->fwd)
		return NULL;

	if ((queue)->fwd == (struct dq_linkage_t *)(queue))
		return NULL;

	temp = ((struct dq_linkage_t *)queue)->back;
	temp->fwd->back = temp->back;
	temp->back->fwd = temp->fwd;

	/* make item linkages safe for "orphan" removes */
	temp->fwd = temp;
	temp->back = temp;

	return temp;
}

void dq_addbefore(void *successor, void *item)
{
	VDEC_ASSERT(((struct dq_linkage_t *)successor)->back);
	VDEC_ASSERT(((struct dq_linkage_t *)successor)->fwd);

	if (!((struct dq_linkage_t *)successor)->back ||
	    !((struct dq_linkage_t *)successor)->fwd)
		return;

	((struct dq_linkage_t *)item)->fwd = (struct dq_linkage_t *)successor;
	((struct dq_linkage_t *)item)->back =
				((struct dq_linkage_t *)successor)->back;
	((struct dq_linkage_t *)item)->back->fwd = (struct dq_linkage_t *)item;
	((struct dq_linkage_t *)successor)->back = (struct dq_linkage_t *)item;
}

void dq_addafter(void *predecessor, void *item)
{
	VDEC_ASSERT(((struct dq_linkage_t *)predecessor)->back);
	VDEC_ASSERT(((struct dq_linkage_t *)predecessor)->fwd);

	if (!((struct dq_linkage_t *)predecessor)->back ||
	    !((struct dq_linkage_t *)predecessor)->fwd)
		return;

	((struct dq_linkage_t *)item)->fwd =
				((struct dq_linkage_t *)predecessor)->fwd;
	((struct dq_linkage_t *)item)->back =
					(struct dq_linkage_t *)predecessor;
	((struct dq_linkage_t *)item)->fwd->back = (struct dq_linkage_t *)item;
	((struct dq_linkage_t *)predecessor)->fwd = (struct dq_linkage_t *)item;
}

void dq_move(struct dq_linkage_t *from, struct dq_linkage_t *to)
{
	VDEC_ASSERT(((struct dq_linkage_t *)from)->back);
	VDEC_ASSERT(((struct dq_linkage_t *)from)->fwd);
	VDEC_ASSERT(((struct dq_linkage_t *)to)->back);
	VDEC_ASSERT(((struct dq_linkage_t *)to)->fwd);

	if (!((struct dq_linkage_t *)from)->back ||
	    !((struct dq_linkage_t *)from)->fwd ||
	    !((struct dq_linkage_t *)to)->back ||
	    !((struct dq_linkage_t *)to)->fwd)
		return;

	if ((from)->fwd == (struct dq_linkage_t *)(from)) {
		dq_init(to);
	} else {
		*to = *from;
		to->fwd->back = (struct dq_linkage_t *)to;
		to->back->fwd = (struct dq_linkage_t *)to;
		dq_init(from);
	}
}
