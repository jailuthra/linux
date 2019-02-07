/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This component is used to track decoder resources,
 * and share them across other components.
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

#ifndef __RMAN_API_H__
#define __RMAN_API_H__

#include <linux/types.h>
#include "img_errors.h"
#include "lst.h"

#define	RMAN_ALL_TYPES		(0xFFFFFFFF)
#define	RMAN_TYPE_P1		(0xFFFFFFFE)
#define	RMAN_TYPE_P2		(0xFFFFFFFE)
#define	RMAN_TYPE_P3		(0xFFFFFFFE)
#define RMAN_STICKY		(0xFFFFFFFD)

int rman_initialise(void);

void rman_deinitialise(void);

int rman_create_bucket(void **res_handle);

void rman_destroy_bucket(void *res_handle);

void *rman_get_global_bucket(void);

typedef void (*rman_fn_free) (void *param);

int rman_register_resource(void *res_handle, u32 type_id, rman_fn_free fn_free,
			   void *param, void **res_handle_ptr,
			   u32 *res_id);

typedef int (*rman_fn_alloc) (void *alloc_info, void **param);

int rman_get_named_resource(char *res_name, rman_fn_alloc fn_alloc,
			    void *alloc_info, void *res_bucket_handle,
			    u32 type_id, rman_fn_free fn_free,
			    void **param, void **res_handle, u32 *res_id);

u32 rman_get_resource_id(void *res_handle);

int rman_get_resource(u32 res_id, u32 type_id, void **param,
		      void **res_handle);

void rman_free_resource(void *res_handle);

void rman_lock_resource(void *res_handle);

void rman_unlock_resource(void *res_hanle);

void rman_free_resources(void *res_bucket_handle, u32 type_id);

#endif
