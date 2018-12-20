/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Self scaling hash tables.
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
#ifndef _HASH_H_
#define _HASH_H_

struct hash;

/*
 * @Function	VID_HASH_Initialise
 * @Description
 * To initialise the hash module.
 * @Input	 None
 * @Return	IMG_SUCCESS or an error code.
 */
int vid_hash_initialise(void);

/*
 * @Function	VID_HASH_Finalise
 * @Description
 * To finalise the hash module. All allocated hash tables should
 * be deleted before calling this function.
 * @Input	None
 * @Return	IMG_SUCCESS or an error code.
 */
int vid_hash_finalise(void);

/*
 * @Function	VID_HASH_Create
 * @Description
 * Create a self scaling hash table.
 * @Input	initial_size : Initial and minimum size of the hash table.
 * @Output	hash : Hash table handle or NULL.
 * @Return	IMG_SUCCESS or an error code.
 */
int vid_hash_create(u32 initial_size,
		    struct hash ** const hash_hndl);

/*
 * @Function	VID_HASH_Delete
 * @Description
 * To delete a hash table, all entries in the table should be
 * removed before calling this function.
 * @Input	hash : Hash table pointer
 * @Return	IMG_SUCCESS or an error code.
 */
int vid_hash_delete(struct hash * const ps_hash);

/*
 * @Function	VID_HASH_Insert
 * @Description
 * To insert a key value pair into a hash table.
 * @Input	ps_hash : Hash table pointer
 * @Input	key : Key value
 * @Input	value : The value associated with the key.
 * @Return	IMG_SUCCESS or an error code.
 */
int vid_hash_insert(struct hash * const ps_hash,
		    u64 key,
		    u64 value);

/*
 * @Function	VID_HASH_Remove
 * @Description
 * To remove a key value pair from a hash table
 * @Input	ps_hash : Hash table pointer
 * @Input	key : Key value
 * @Input	result : 0 if the key is missing or the value
 *		associated with the key.
 * @Return	IMG_SUCCESS or an error code.
 */
int vid_hash_remove(struct hash * const ps_hash,
		    u64 key,
		    u64 * const result);

#endif /* _HASH_H_ */

