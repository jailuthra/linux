/* SPDX-License-Identifier: GPL-2.0 */
/*
 * IMG DEC Memory Manager header file
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

#ifndef _IMG_DEC_MEM_MGR_H
#define _IMG_DEC_MEM_MGR_H

#include <linux/idr.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/stddef.h>
#include <linux/types.h>
#include <stdarg.h>

/* buffer ids (per memory context) */
#define MEM_MAN_MIN_BUFFER 1
#define MEM_MAN_MAX_BUFFER 16384

enum mem_attr {
	MEM_ATTR_CACHED        = 0x00000001,
	MEM_ATTR_UNCACHED      = 0x00000002,
	MEM_ATTR_WRITECOMBINE  = 0x00000004,
	MEM_ATTR_SECURE        = 0x00000010,
};

enum mmu_callback_type {
	MMU_CALLBACK_MAP = 1,
	MMU_CALLBACK_UNMAP,
};

enum heap_type {
	MEM_HEAP_TYPE_UNIFIED = 1,
};

union heap_options {
	struct {
		gfp_t gfp_type; /* pool and flags for buffer allocations */
	} unified;
};

/*
 * struct heap_config - contains heap configuration structure
 * @type: enumeration of heap_type
 * @options: pool and flags for buffer allocations, eg GFP_KERNEL
 * @to_dev_addr: function pointer for retrieving device addr
 */
struct heap_config {
	enum heap_type type;
	union heap_options options;
	phys_addr_t (*to_dev_addr)(union heap_options *opts, phys_addr_t addr);
};

/*
 * struct mmu_heap - typedef for mmu_heap
 * @virt_addr_start: start of the device virtual address
 * @alloc_atom: atom allocation in bytes
 * @size: total size of the heap in bytes
 */
struct mmu_heap {
	uintptr_t virt_addr_start;
	size_t alloc_atom;
	size_t size;
};

/*
 * struct mem_ctx - the memory context
 * @buffers: idr list of buffers
 * @mmu_ctxs: contains linked lists of struct mmu_ctx
 * @mem_man_entry: the entry list for dev_mem_main:mem_ctxs linked list
 */
struct mem_ctx {
	struct idr buffers;
	struct list_head mmu_ctxs;
	struct list_head mem_man_entry;
};

/*
 * struct mmu_ctx_mapping - the mmu context mapping information
 * @mmu_ctx: pointer to the mmu_ctx to which this mmu mapping information
 *	     belongs
 * @buffer: pointer to the buffer which this mmu_ctx_mapping is for
 * @map: pointer to the mmu_map which this mmu_ctx_mapping belongs
 * @virt_addr: Virtual address
 * @mmu_ctx_entry: the entry list for mmu_ctx:mapping linked list.
 * @buffer_entry: the entry list for buffer:mappings linked list.
 */
struct mmu_ctx_mapping {
	struct mmu_ctx *mmu_ctx;
	struct buffer *buffer;
	struct mmu_map *map;
	u32 virt_addr;
	struct list_head mmu_ctx_entry;
	struct list_head buffer_entry;
};

/*
 * struct mmu_ctx - the mmu context information - one per stream
 * @device: pointer to the device
 * @mmu_config_addr_width: the address width for the mmu config
 * @mem_ctx: pointer to mem_ctx where this mmu_ctx belongs to
 * @heap: pointer to struct heap to where this mem_ctx belongs to
 * @mmu_dir: pointer to the mmu_directory this mmu_ctx belongs to
 * @mappings: contains linked list of struct mmu_ctx_mapping
 * @mem_ctx_entry: the entry list for mem_ctx:mmu_ctxs
 * @callback_fn: pointer to function callback
 * @callback_data: pointer to the callback data
 */
struct mmu_ctx {
	struct device *device;
	u32 mmu_config_addr_width;
	struct mem_ctx *mem_ctx;
	struct heap *heap;
	struct mmu_directory *mmu_dir;
	struct list_head mappings;
	struct list_head mem_ctx_entry;
	void (*callback_fn)(enum mmu_callback_type type, int buff_id,
			    void *data);
	void *callback_data;
};

/*
 * struct buffer - the mmu context information - one per stream
 * @id: buffer identification
 * @request_size: request size for the allocation
 * @actual_size: size aligned with the PAGE_SIZE allocation
 * @device: pointer to the device
 * @mem_ctx: pointer to struct mem_ctx to where this buffer belongs to
 * @heap: pointer to struct heap to where this buffer belongs to
 * @mappings: contains linked lists of struct mmu_ctx_mapping
 * @kptr: pointer to virtual mapping for the buffer object into kernel address
 *	  space
 * @priv: pointer to priv data used for scaterlist table info
 */
struct buffer {
	int id; /* Generated in <mem_ctx:buffers> */
	size_t request_size;
	size_t actual_size;
	struct device *device;
	struct mem_ctx *mem_ctx;
	struct heap *heap;
	struct list_head mappings; /* contains <struct mmu_ctx_mapping> */
	void *kptr;
	void *priv;
};

struct heap_ops {
	int (*alloc)(struct device *device, struct heap *heap,
		     size_t size, enum mem_attr attr,
		     struct buffer *buffer);
	void (*free)(struct heap *heap, struct buffer *buffer);
	int (*map_km)(struct heap *heap, struct buffer *buffer);
	int (*get_sg_table)(struct heap *heap, struct buffer *buffer,
			    struct sg_table **sg_table);
	void (*sync_cpu_to_dev)(struct heap *heap, struct buffer *buffer);
	void (*sync_dev_to_cpu)(struct heap *heap, struct buffer *buffer);
	void (*destroy)(struct heap *heap);
};

struct heap {
	int id; /* Generated in <mem_man:heaps> */
	enum heap_type type;
	struct heap_ops *ops;
	union heap_options options;
	phys_addr_t (*to_dev_addr)(union heap_options *opts, phys_addr_t addr);
	void *priv;
};

int img_mem_init(struct device *dev);
void img_mem_exit(void);

int img_mem_create_ctx(struct mem_ctx **new_ctx);
void img_mem_destroy_ctx(struct mem_ctx *ctx);

int img_mem_import(struct device *device, struct mem_ctx *ctx,
		   size_t size, enum mem_attr attr, int *buf_id);

int img_mem_alloc(struct device *device, struct mem_ctx *ctx, int heap_id,
		  size_t size, enum mem_attr attributes, int *buf_id);
void img_mem_free(struct mem_ctx *ctx, int buff_id);

void img_mem_free_bufid(struct mem_ctx *ctx, int buf_id);

int img_mem_map_km(struct mem_ctx *ctx, int buf_id);
void *img_mem_get_kptr(struct mem_ctx *ctx, int buff_id);

int img_mem_sync_cpu_to_device(struct mem_ctx *ctx, int buf_id);
int img_mem_sync_device_to_cpu(struct mem_ctx *ctx, int buf_id);

int img_mmu_ctx_create(struct device *device, u32 mmu_config_addr_width,
		       struct mem_ctx *mem_ctx, int heap_id,
		       void (*callback_fn)(enum mmu_callback_type type,
					   int buff_id, void *data),
		       void *callback_data, struct mmu_ctx **mmu_ctx);
void img_mmu_ctx_destroy(struct mmu_ctx *ctx);

int img_mmu_map(struct mmu_ctx *mmu_ctx, struct mem_ctx *mem_ctx,
		int buff_id, u32 virt_addr, unsigned int map_flags);
int img_mmu_map_sg(struct mmu_ctx *mmu_ctx, struct mem_ctx *mem_ctx,
		   int buff_id, struct sg_table *sgt, u32 virt_addr,
		   unsigned int map_flags);
int img_mmu_unmap(struct mmu_ctx *mmu_ctx, struct mem_ctx *mem_ctx,
		  int buff_id);

int img_mmu_get_ptd(const struct mmu_ctx *ctx, unsigned int *ptd);

int img_mem_add_heap(const struct heap_config *heap_cfg, int *heap_id);
void img_mem_del_heap(int heap_id);

/* Heap operation related function */
int img_mem_unified_init(const struct heap_config *config,
			 struct heap *heap);

#endif /* _IMG_DEC_MEM_MGR */
