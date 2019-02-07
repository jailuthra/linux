/* SPDX-License-Identifier: GPL-2.0 */
/*
 * List processing primitives.
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
#ifndef __LIST_H__
#define __LIST_H__

struct lst_t {
	void **first;
	void **last;
};

void lst_add(struct lst_t *list, void *item);
void lst_addhead(struct lst_t *list, void *item);
int  lst_empty(struct lst_t *list);
void *lst_first(struct lst_t *list);
void lst_init(struct lst_t *list);
void *lst_last(struct lst_t *list);
void *lst_next(void *item);
void *lst_remove(struct lst_t *list, void *item);
void *lst_removehead(struct lst_t *list);
int lst_check(struct lst_t *list, void *item);

#endif /* __LIST_H__ */
