// SPDX-License-Identifier: GPL-2.0
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
#include <linux/slab.h>

#include "hash.h"
#include "img_errors.h"
#include "pool.h"

#define FALSE 0
#define TRUE 1

/* pool of struct hash objects */
static struct pool *global_hashpool;

/* pool of struct bucket objects */
static struct pool *global_bucketpool;

static int global_initialized = FALSE;

/* Each entry in a hash table is placed into a bucket */
struct bucket {
	struct bucket *next;
	u64 key;
	u64 value;
};

struct hash {
	struct bucket **table;
	u32 size;
	u32 count;
	u32 minimum_size;
};

/*
 * @Function	hash_func
 * @Description
 * Hash function intended for hashing addresses.
 * @Input	Vale : The key to hash.
 * @Input	size : The size of the hash table
 * @Return	hash : The hash value.
 */
static u32 hash_func(u64 vale,
		     u32 size)
{
	u32 hash = (u32)(vale);

	hash += (hash << 12);
	hash ^= (hash >> 22);
	hash += (hash << 4);
	hash ^= (hash >> 9);
	hash += (hash << 10);
	hash ^= (hash >> 2);
	hash += (hash << 7);
	hash ^= (hash >> 12);
	hash &= (size - 1);
	return hash;
}

/*
 * @Function	hash_chain_insert
 * @Description
 * Hash function intended for hashing addresses.
 * @Input	bucket : The bucket
 * @Input	table : The hash table
 * @Input	size : The size of the hash table
 * @Return	IMG_SUCCESS or an error code.
 */
static int hash_chain_insert(struct bucket *bucket,
			     struct bucket **table,
			     u32 size)
{
	u32 idx;
	u32 result = IMG_ERROR_FATAL;

	if (!bucket || !table || !size) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		return result;
	}

	idx = hash_func(bucket->key, size);

	if (idx < size) {
		result = IMG_SUCCESS;
		bucket->next = table[idx];
		table[idx] = bucket;
	}

	return result;
}

/*
 * @Function	hash_rehash
 * @Description
 * Iterate over every entry in an old hash table and rehash into the new table.
 * @Input	old_table : The old hash table
 * @Input	old_size : The size of the old hash table
 * @Input	new_table : The new hash table
 * @Input	new_sz : The size of the new hash table
 * @Return	IMG_SUCCESS or an error code.
 */
static int hash_rehash(struct bucket **old_table,
		       u32 old_size,
		       struct bucket **new_table,
		       u32 new_sz)
{
	u32 idx;
	u32 result = IMG_ERROR_FATAL;

	if (!old_table || !new_table) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		return result;
	}

	for (idx = 0; idx < old_size; idx++) {
		struct bucket *bucket;
		struct bucket *nex_bucket;

		bucket = old_table[idx];
		while (bucket) {
			nex_bucket = bucket->next;
			result = hash_chain_insert(bucket, new_table, new_sz);
			if (result != IMG_SUCCESS) {
				result = IMG_ERROR_UNEXPECTED_STATE;
				return result;
			}
			bucket = nex_bucket;
		}
	}
	result = IMG_SUCCESS;

	return result;
}

/*
 * @Function	hash_resize
 * @Description
 * Attempt to resize a hash table, failure to allocate a new larger hash table
 * is not considered a hard failure. We simply continue and allow the table to
 * fill up, the effect is to allow hash chains to become longer.
 * @Input	hash_arg : Pointer to the hash table
 * @Input	new_sz : The size of the new hash table
 * @Return	IMG_SUCCESS or an error code.
 */
static int hash_resize(struct hash *hash_arg,
		       u32 new_sz)
{
	u32 malloc_sz = 0;
	u32 result = IMG_ERROR_FATAL;

	if (!hash_arg) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		return result;
	}

	if (new_sz != hash_arg->size) {
		struct bucket **new_bkt_table;

		malloc_sz = (sizeof(struct bucket *) * new_sz);
		new_bkt_table = kmalloc(malloc_sz, GFP_KERNEL);

		if (!new_bkt_table) {
			result = IMG_ERROR_MALLOC_FAILED;
			return result;
		}
		if (new_bkt_table) {
			u32 idx;

			for (idx = 0; idx < new_sz; idx++)
				new_bkt_table[idx] = NULL;

			result = hash_rehash(hash_arg->table,
					     hash_arg->size,
					     new_bkt_table,
					     new_sz);

			if (result != IMG_SUCCESS) {
				kfree(new_bkt_table);
				new_bkt_table = NULL;
				result = IMG_ERROR_UNEXPECTED_STATE;
				return result;
			}

			if (!hash_arg->table)
				kfree(hash_arg->table);
			hash_arg->table = new_bkt_table;
			hash_arg->size = new_sz;
		}
	}
	result = IMG_SUCCESS;

	return result;
}

static u32 private_max(u32 a, u32 b)
{
	u32 ret = (a > b) ? a : b;
	return ret;
}

/*
 * @Function	vid_hash_initialise
 * @Description
 * To initialise the hash module.
 * @Input	None
 * @Return	IMG_SUCCESS or an error code.
 */
int vid_hash_initialise(void)
{
	u32 result = IMG_ERROR_ALREADY_COMPLETE;

	if (!global_initialized) {
		if (global_hashpool || global_bucketpool) {
			result = IMG_ERROR_UNEXPECTED_STATE;
			return result;
		}

		result = pool_create("img-hash",
				     sizeof(struct hash),
				     &global_hashpool);

		if (result != IMG_SUCCESS) {
			result = IMG_ERROR_UNEXPECTED_STATE;
			return result;
		}

		result = pool_create("img-sBucket",
				     sizeof(struct bucket),
				     &global_bucketpool);
		if (result != IMG_SUCCESS) {
			if (global_bucketpool) {
				result = pool_delete(global_bucketpool);
				global_bucketpool = NULL;
			}
			result = IMG_ERROR_UNEXPECTED_STATE;
			return result;
		}
		global_initialized = TRUE;
		result = IMG_SUCCESS;
	}
	return result;
}

/*
 * @Function	vid_hash_finalise
 * @Description
 * To finalise the hash module. All allocated hash tables should
 * be deleted before calling this function.
 * @Input	None
 * @Return	IMG_SUCCESS or an error code.
 */
int vid_hash_finalise(void)
{
	u32 result = IMG_ERROR_FATAL;

	if (global_initialized) {
		if (global_hashpool) {
			result = pool_delete(global_hashpool);
			if (result != IMG_SUCCESS)
				return result;

			global_hashpool = NULL;
		}

		if (global_bucketpool) {
			result = pool_delete(global_bucketpool);
			if (result != IMG_SUCCESS)
				return result;

			global_bucketpool = NULL;
		}
		global_initialized = FALSE;
		result = IMG_SUCCESS;
	}

	return result;
}

/*
 * @Function	vid_hash_create
 * @Description
 * Create a self scaling hash table.
 * @Input	initial_size : Initial and minimum size of the hash table.
 * @Output	hash_arg : Will countin the hash table handle or NULL.
 * @Return	IMG_SUCCESS or an error code.
 */
int vid_hash_create(u32 initial_size,
		    struct hash ** const hash_arg)
{
	u32 idx;
	u32 tbl_sz = 0;
	u32 result = IMG_ERROR_FATAL;
	struct hash *local_hash = NULL;

	if (!hash_arg) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		return result;
	}

	if (global_initialized) {
		pool_alloc(global_hashpool, ((void **)&local_hash));
		if (!local_hash) {
			result = IMG_ERROR_UNEXPECTED_STATE;
			*hash_arg = NULL;
			return result;
		}

		local_hash->count = 0;
		local_hash->size = initial_size;
		local_hash->minimum_size = initial_size;

		tbl_sz = (sizeof(struct bucket *) * local_hash->size);
		local_hash->table = kmalloc(tbl_sz, GFP_KERNEL);
		if (!local_hash->table) {
			result = pool_free(global_hashpool, local_hash);
			if (result != IMG_SUCCESS)
				result = IMG_ERROR_UNEXPECTED_STATE;
			result |= IMG_ERROR_MALLOC_FAILED;
			*hash_arg = NULL;
			return result;
		}

		for (idx = 0; idx < local_hash->size; idx++)
			local_hash->table[idx] = NULL;

		*hash_arg = local_hash;
		result = IMG_SUCCESS;
	}
	return result;
}

/*
 * @Function	vid_hash_delete
 * @Description
 * To delete a hash table, all entries in the table should be
 * removed before calling this function.
 * @Input	hash_arg : Hash table pointer
 * @Return	IMG_SUCCESS or an error code.
 */
int vid_hash_delete(struct hash * const hash_arg)
{
	u32 result = IMG_ERROR_FATAL;

	if (!hash_arg) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		return result;
	}

	if (global_initialized) {
		if (hash_arg->count != 0) {
			result = IMG_ERROR_UNEXPECTED_STATE;
			return result;
		}

		kfree(hash_arg->table);
		hash_arg->table = NULL;

		result = pool_free(global_hashpool, hash_arg);
		if (result != IMG_SUCCESS) {
			result = IMG_ERROR_UNEXPECTED_STATE;
			return result;
		}
	}
	return result;
}

/*
 * @Function	vid_hash_insert
 * @Description
 * To insert a key value pair into a hash table.
 * @Input	hash_arg : Hash table pointer
 * @Input	key : Key value
 * @Input	value : The value associated with the key.
 * @Return	IMG_SUCCESS or an error code.
 */
int vid_hash_insert(struct hash * const hash_arg,
		    u64 key,
		    u64 value)
{
	struct bucket *ps_bucket = NULL;
	u32 result = IMG_ERROR_FATAL;

	if (!hash_arg) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		return result;
	}

	if (global_initialized) {
		result = pool_alloc(global_bucketpool, ((void **)&ps_bucket));
		if (result != IMG_SUCCESS || !ps_bucket) {
			result = IMG_ERROR_UNEXPECTED_STATE;
			return result;
		}
		ps_bucket->next = NULL;
		ps_bucket->key = key;
		ps_bucket->value = value;

		result = hash_chain_insert(ps_bucket,
					   hash_arg->table,
					   hash_arg->size);

		if (result != IMG_SUCCESS) {
			pool_free(global_bucketpool, ((void **)&ps_bucket));
			result = IMG_ERROR_UNEXPECTED_STATE;
			return result;
		}

		hash_arg->count++;

		/* check if we need to think about re-balancing */
		if ((hash_arg->count << 1) > hash_arg->size) {
			result = hash_resize(hash_arg, (hash_arg->size << 1));
			if (result != IMG_SUCCESS) {
				result = IMG_ERROR_UNEXPECTED_STATE;
				return result;
			}
		}
		result = IMG_SUCCESS;
	}
	return result;
}

/*
 * @Function	vid_hash_remove
 * @Description
 * To remove a key value pair from a hash table
 * @Input	hash_arg : Hash table pointer
 * @Input	key : Key value
 * @Input	ret_result : 0 if the key is missing or the value
 *		associated with the key.
 * @Return	IMG_SUCCESS or an error code.
 */
int vid_hash_remove(struct hash * const hash_arg,
		    u64 key,
		    u64 * const ret_result)
{
	u32 idx;
	u32 tmp1 = 0;
	u32 tmp2 = 0;
	u32 result = IMG_ERROR_FATAL;
	struct bucket **bucket = NULL;

	if (!hash_arg) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		return result;
	}

	idx = hash_func(key, hash_arg->size);

	for (bucket = &hash_arg->table[idx]; (*bucket) != NULL;
		bucket = &((*bucket)->next)) {
		if ((*bucket)->key == key) {
			struct bucket *ps_bucket = (*bucket);

			u64 value = ps_bucket->value;

			*bucket = ps_bucket->next;
			result = pool_free(global_bucketpool, ps_bucket);

			hash_arg->count--;

			/* check if we need to think about re-balencing */
			if (hash_arg->size > (hash_arg->count << 2) &&
			    hash_arg->size > hash_arg->minimum_size) {
				tmp1 = (hash_arg->size >> 1);
				tmp2 = hash_arg->minimum_size;
				result = hash_resize(hash_arg,
						     private_max(tmp1, tmp2));
			}
			*ret_result = value;
			result = IMG_SUCCESS;
			break;
		}
	}
	return result;
}
