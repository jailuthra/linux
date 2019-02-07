/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Object Pool Memory Allocator header
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
#ifndef _pool_h_
#define _pool_h_

struct pool;

/*
 * @Function	pool_create
 * @Description
 * Create an sObject pool
 * @Input	name : Name of sObject pool for diagnostic purposes
 * @Input	obj_size : size of each sObject in the pool in bytes
 * @Output	pool : Will contain NULL or sObject pool handle
 * @Return	IMG_SUCCESS or an error code.
 */
int pool_create(const char * const name,
		u32 obj_size,
		struct pool ** const pool);

/*
 * @Function	pool_delete
 * @Description
 * Delete an sObject pool. All psObjects allocated from the pool must
 * be free'd with pool_free() before deleting the sObject pool.
 * @Input	pool : Object Pool pointer
 * @Return IMG_SUCCESS or an error code.
 */
int pool_delete(struct pool * const pool);

/*
 * @Function	pool_alloc
 * @Description
 * Allocate an Object from an Object pool.
 * @Input	pool : Object Pool
 * @Output	obj_hdnl : Pointer containing the handle to the
 * object created or IMG_NULL
 * @Return    IMG_SUCCESS or an error code.
 */
int pool_alloc(struct pool * const pool,
	       void ** const obj_hdnl);

/*
 * @Function	pool_free
 * @Description
 * Free an sObject previously allocated from an sObject pool.
 * @Input	pool : Object Pool pointer.
 * @Output	obj_hdnl : Handle to the object to be freed.
 * @Return	IMG_SUCCESS or an error code.
 */
int pool_free(struct pool * const pool,
	      void * const obj_hdnl);

#endif /* _pool_h_ */
