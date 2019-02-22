/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VXD DEC SYSDEV and UI Interface header
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

#ifndef _VXD_RESOURCE_H
#define _VXD_RESOURCE_H

typedef int (*resource_pfn_freeitem)(void *item, void *free_cb_param);

int resource_item_use(u32 *refcnt);

int resource_item_return(u32 *refcnt);

int resource_item_release(u32 *refcnt);

int resource_item_isavailable(u32 *refcnt);

int resource_list_add(struct lst_t *list, void *item, u32 id, u32 *refcnt);

void *resource_list_pickhead(struct lst_t *list);

int resource_list_remove(struct lst_t *list, void *item);

void *resource_list_removehead(struct lst_t *list);

int resource_list_remove_nextavail(struct lst_t *list,
				   resource_pfn_freeitem fn_freeitem,
				   void *free_cb_param);

void *resource_list_get_avail(struct lst_t *list);

void *resource_list_reuseitem(struct lst_t *list, void *item);

void *resource_list_getbyid(struct lst_t *list, u32 id);

int resource_list_getnumavail(struct lst_t *list);

int resource_list_getnum(struct lst_t *list);

int resource_list_replace(struct lst_t *list, void *item, u32 id, u32 *refcnt,
			  resource_pfn_freeitem fn_freeitem,
			  void *free_cb_param);

int resource_list_empty(struct lst_t *list, u32 release_item,
			resource_pfn_freeitem fn_freeitem,
			void *free_cb_param);

int resource_getnumpict(struct lst_t *list);

#endif /* _VXD_RESOURCE_H */
