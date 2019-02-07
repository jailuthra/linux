// SPDX-License-Identifier: GPL-2.0
/*
 * IMG DEC Memory Manager
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

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/idr.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#include "imgmmu.h"
#include "img_mem_man.h"

#define VXD_MMU_SHIFT 8 /* assume 40-bit MMU */
/* heaps ids (global) */
#define MIN_HEAP 1
#define MAX_HEAP 16

/*
 * struct dev_mem_man - the device memory management
 * @heaps: idr list of heap for the device memory manager
 * @mem_ctxs: contains lists of mem_ctx
 * @mutex: mutex for this device
 */
struct mem_man {
	struct device *dev;
	struct idr heaps;
	struct list_head mem_ctxs;
	struct mutex mutex; /* mutex for this device */
};

static struct mem_man mem_man_data;

/*
 * struct mmu_page - the mmu page information for the buffer
 * @buffer: buffer pointer for the particular mmu_page
 * @page_cfg: mmu page configuration of physical and virtual addr
 * @addr_shift: address shifting information
 */
struct mmu_page {
	struct buffer *buffer;
	struct mmu_page_cfg page_cfg;
	u32 addr_shift;
};

static int trace_physical_pages;

static void _img_mem_free(struct buffer *buffer);
static void _img_mmu_unmap(struct mmu_ctx_mapping *mapping);
static void _img_mmu_ctx_destroy(struct mmu_ctx *ctx);

static char *get_heap_name(enum heap_type type)
{
	switch (type) {
	case MEM_HEAP_TYPE_UNIFIED:
		return "unified";
	default:
		return "unknown";
	}
}

int img_mem_add_heap(const struct heap_config *heap_cfg, int *heap_id)
{
	struct mem_man *mem_man = &mem_man_data;
	struct heap *heap;
	int (*init_fn)(const struct heap_config *heap_cfg, struct heap *heap);
	int ret;

	switch (heap_cfg->type) {
	case MEM_HEAP_TYPE_UNIFIED:
		init_fn = img_mem_unified_init;
		break;
	default:
		dev_err(mem_man->dev, "%s: heap type %d unknown\n", __func__,
			heap_cfg->type);
		return -EINVAL;
	}

	heap = kmalloc(sizeof(*heap), GFP_KERNEL);
	if (!heap)
		return -ENOMEM;

	ret = mutex_lock_interruptible(&mem_man->mutex);
	if (ret)
		goto lock_failed;

	ret = idr_alloc(&mem_man->heaps, heap, MIN_HEAP, MAX_HEAP, GFP_KERNEL);
	if (ret < 0) {
		dev_err(mem_man->dev, "%s: idr_alloc failed\n", __func__);
		goto alloc_id_failed;
	}

	heap->id = ret;
	heap->type = heap_cfg->type;
	heap->options = heap_cfg->options;
	heap->to_dev_addr = heap_cfg->to_dev_addr;
	heap->priv = NULL;

	ret = init_fn(heap_cfg, heap);
	if (ret) {
		dev_err(mem_man->dev, "%s: heap init failed\n", __func__);
		goto heap_init_failed;
	}

	*heap_id = heap->id;
	mutex_unlock(&mem_man->mutex);

	dev_info(mem_man->dev, "%s created heap %d type %d (%s)\n",
		 __func__, *heap_id, heap_cfg->type, get_heap_name(heap->type));
	return 0;

heap_init_failed:
	idr_remove(&mem_man->heaps, heap->id);
alloc_id_failed:
	mutex_unlock(&mem_man->mutex);
lock_failed:
	kfree(heap);
	return ret;
}

static void _img_mem_del_heap(struct heap *heap)
{
	struct mem_man *mem_man = &mem_man_data;

	if (heap->ops->destroy)
		heap->ops->destroy(heap);

	idr_remove(&mem_man->heaps, heap->id);
}

void img_mem_del_heap(int heap_id)
{
	struct mem_man *mem_man = &mem_man_data;
	struct heap *heap;

	mutex_lock(&mem_man->mutex);

	heap = idr_find(&mem_man->heaps, heap_id);
	if (!heap) {
		dev_warn(mem_man->dev, "%s heap %d not found!\n", __func__,
			 heap_id);
		mutex_unlock(&mem_man->mutex);
		return;
	}

	_img_mem_del_heap(heap);

	mutex_unlock(&mem_man->mutex);

	kfree(heap);
}

int img_mem_create_ctx(struct mem_ctx **new_ctx)
{
	struct mem_man *mem_man = &mem_man_data;
	struct mem_ctx *ctx;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	idr_init(&ctx->buffers);
	INIT_LIST_HEAD(&ctx->mmu_ctxs);

	mutex_lock(&mem_man->mutex);
	list_add(&ctx->mem_man_entry, &mem_man->mem_ctxs);
	mutex_unlock(&mem_man->mutex);

	*new_ctx = ctx;
	return 0;
}

static void _img_mem_destroy_ctx(struct mem_ctx *ctx)
{
	struct mem_man *mem_man = &mem_man_data;
	struct buffer *buffer;
	int buff_id;

	/* free derelict mmu contexts */
	while (!list_empty(&ctx->mmu_ctxs)) {
		struct mmu_ctx *mc;

		mc = list_first_entry(&ctx->mmu_ctxs,
				      struct mmu_ctx, mem_ctx_entry);
		dev_warn(mem_man->dev, "%s: found derelict mmu context %p\n",
			 __func__, mc);
		_img_mmu_ctx_destroy(mc);
		kfree(mc);
	}

	/* free derelict buffers */
	buff_id = MEM_MAN_MIN_BUFFER;
	buffer = idr_get_next(&ctx->buffers, &buff_id);
	while (buffer) {
		dev_warn(mem_man->dev, "%s: found derelict buffer %d\n",
			 __func__, buff_id);
		_img_mem_free(buffer);
		kfree(buffer);
		buff_id = MEM_MAN_MIN_BUFFER;
		buffer = idr_get_next(&ctx->buffers, &buff_id);
	}

	idr_destroy(&ctx->buffers);
	list_del(&ctx->mem_man_entry);
}

void img_mem_destroy_ctx(struct mem_ctx *ctx)
{
	struct mem_man *mem_man = &mem_man_data;

	mutex_lock(&mem_man->mutex);
	_img_mem_destroy_ctx(ctx);
	mutex_unlock(&mem_man->mutex);

	kfree(ctx);
}

static int _img_mem_alloc(struct device *device, struct mem_ctx *ctx,
			  struct heap *heap, size_t size,
			  enum mem_attr attr, struct buffer **buffer_new)
{
	struct buffer *buffer;
	int ret;

	if (size == 0) {
		dev_err(device, "%s: buffer size is zero\n", __func__);
		return -EINVAL;
	}

	if (!heap->ops || !heap->ops->alloc) {
		dev_err(device, "%s: no alloc function in heap %d!\n",
			__func__, heap->id);
		return -EINVAL;
	}

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	ret = idr_alloc(&ctx->buffers, buffer,
			MEM_MAN_MIN_BUFFER, MEM_MAN_MAX_BUFFER,
			GFP_KERNEL);
	if (ret < 0) {
		dev_err(device, "%s: idr_alloc failed\n", __func__);
		goto idr_alloc_failed;
	}

	buffer->id = ret;
	buffer->request_size = size;
	buffer->actual_size = ((size + PAGE_SIZE - 1) / PAGE_SIZE) * PAGE_SIZE;
	buffer->device = device;
	buffer->mem_ctx = ctx;
	buffer->heap = heap;
	INIT_LIST_HEAD(&buffer->mappings);
	buffer->kptr = NULL;
	buffer->priv = NULL;

	ret = heap->ops->alloc(device, heap, buffer->actual_size, attr,
		buffer);
	if (ret) {
		dev_err(device, "%s: heap %d alloc failed\n", __func__,
			heap->id);
		goto heap_alloc_failed;
	}

	*buffer_new = buffer;

	dev_dbg(device, "%s heap %p ctx %p created buffer %d (%p) actual_size %zu\n",
		__func__, heap, ctx, buffer->id, buffer, buffer->actual_size);
	return 0;

heap_alloc_failed:
	idr_remove(&ctx->buffers, buffer->id);
idr_alloc_failed:
	kfree(buffer);
	return ret;
}

int img_mem_alloc(struct device *device, struct mem_ctx *ctx, int heap_id,
		  size_t size, enum mem_attr attr, int *buf_id)
{
	struct mem_man *mem_man = &mem_man_data;
	struct heap *heap;
	struct buffer *buffer;
	int ret;

	dev_dbg(device, "%s heap %d ctx %p size %zu\n", __func__, heap_id,
		ctx, size);

	ret = mutex_lock_interruptible(&mem_man->mutex);
	if (ret)
		return ret;

	heap = idr_find(&mem_man->heaps, heap_id);
	if (!heap) {
		dev_err(device, "%s: heap id %d not found\n", __func__,
			heap_id);
		mutex_unlock(&mem_man->mutex);
		return -EINVAL;
	}

	ret = _img_mem_alloc(device, ctx, heap, size, attr, &buffer);
	if (ret) {
		mutex_unlock(&mem_man->mutex);
		return ret;
	}

	*buf_id = buffer->id;
	mutex_unlock(&mem_man->mutex);

	dev_dbg(device, "%s heap %d ctx %p created buffer %d (%p) size %zu\n",
		__func__, heap_id, ctx, *buf_id, buffer, size);
	return ret;
}

static int _img_mem_import(struct device *device, struct mem_ctx *ctx,
			   size_t size, enum mem_attr attr, struct buffer **buffer_new)
{
	struct buffer *buffer;
	int ret;

	if (size == 0) {
		dev_err(device, "%s: buffer size is zero\n", __func__);
		return -EINVAL;
	}

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	ret = idr_alloc(&ctx->buffers, buffer,
			MEM_MAN_MIN_BUFFER, MEM_MAN_MAX_BUFFER,
			GFP_KERNEL);
	if (ret < 0) {
		dev_err(device, "%s: idr_alloc failed\n", __func__);
		goto idr_alloc_failed;
	}

	buffer->id = ret;
	buffer->request_size = size;
	buffer->actual_size = ((size + PAGE_SIZE - 1) / PAGE_SIZE) * PAGE_SIZE;
	buffer->device = device;
	buffer->mem_ctx = ctx;
	buffer->heap = NULL;
	INIT_LIST_HEAD(&buffer->mappings);
	buffer->kptr = NULL;
	buffer->priv = NULL;

	*buffer_new = buffer;

	dev_dbg(device, "%s ctx %p created buffer %d (%p) actual_size %zu\n",
		__func__, ctx, buffer->id, buffer, buffer->actual_size);
	return 0;

idr_alloc_failed:
	kfree(buffer);
	return ret;
}

int img_mem_import(struct device *device, struct mem_ctx *ctx,
		   size_t size, enum mem_attr attr, int *buf_id)
{
	struct mem_man *mem_man = &mem_man_data;
	struct buffer *buffer;
	int ret;

	dev_dbg(device, "%s ctx %p size %zu\n", __func__, ctx, size);

	ret = mutex_lock_interruptible(&mem_man->mutex);
	if (ret)
		return ret;

	ret = _img_mem_import(device, ctx, size, attr, &buffer);
	if (ret) {
		mutex_unlock(&mem_man->mutex);
		return ret;
	}

	*buf_id = buffer->id;
	mutex_unlock(&mem_man->mutex);

	dev_dbg(device, "%s ctx %p created buffer %d (%p) size %zu\n",
		__func__, ctx, *buf_id, buffer, size);
	return ret;
}

static void _img_mem_free(struct buffer *buffer)
{
	struct device *dev = buffer->device;
	struct heap *heap = buffer->heap;
	struct mem_ctx *ctx = buffer->mem_ctx;

	if (!heap->ops || !heap->ops->free) {
		dev_err(dev, "%s: no free function in heap %d!\n",
			__func__, heap->id);
		return;
	}

	while (!list_empty(&buffer->mappings)) {
		struct mmu_ctx_mapping *map;

		map = list_first_entry(&buffer->mappings,
				       struct mmu_ctx_mapping, buffer_entry);
		dev_warn(dev, "%s: found mapping for buffer %d (size %zu)\n",
			 __func__, map->buffer->id, map->buffer->actual_size);

		_img_mmu_unmap(map);

		kfree(map);
	}

	heap->ops->free(heap, buffer);

	idr_remove(&ctx->buffers, buffer->id);
}

void img_mem_free(struct mem_ctx *ctx, int buff_id)
{
	struct mem_man *mem_man = &mem_man_data;
	struct buffer *buffer;

	mutex_lock(&mem_man->mutex);

	buffer = idr_find(&ctx->buffers, buff_id);
	if (!buffer) {
		dev_err(mem_man->dev, "%s: buffer id %d not found\n",
			__func__, buff_id);
		mutex_unlock(&mem_man->mutex);
		return;
	}

	_img_mem_free(buffer);

	mutex_unlock(&mem_man->mutex);

	kfree(buffer);
}

void img_mem_free_bufid(struct mem_ctx *ctx, int buff_id)
{
	struct mem_man *mem_man = &mem_man_data;
	struct buffer *buffer;

	mutex_lock(&mem_man->mutex);

	buffer = idr_find(&ctx->buffers, buff_id);
	if (!buffer) {
		dev_err(mem_man->dev, "%s: buffer id %d not found\n",
			__func__, buff_id);
		mutex_unlock(&mem_man->mutex);
		return;
	}

	idr_remove(&ctx->buffers, buffer->id);

	mutex_unlock(&mem_man->mutex);

	kfree(buffer);
}

static int _img_mem_map_km(struct buffer *buffer)
{
	struct device *dev = buffer->device;
	struct heap *heap = buffer->heap;

	if (!heap->ops || !heap->ops->map_km) {
		dev_err(dev, "%s: no map_km in heap %d!\n", __func__, heap->id);
		return -EINVAL;
	}

	return heap->ops->map_km(heap, buffer);
}

int img_mem_map_km(struct mem_ctx *ctx, int buff_id)
{
	struct mem_man *mem_man = &mem_man_data;
	struct buffer *buffer;
	int ret;

	mutex_lock(&mem_man->mutex);
	buffer = idr_find(&ctx->buffers, buff_id);
	if (!buffer) {
		dev_err(mem_man->dev, "%s: buffer id %d not found\n",
			__func__, buff_id);
		mutex_unlock(&mem_man->mutex);
		return -EINVAL;
	}

	ret = _img_mem_map_km(buffer);

	mutex_unlock(&mem_man->mutex);

	return ret;
}

void *img_mem_get_kptr(struct mem_ctx *ctx, int buff_id)
{
	struct mem_man *mem_man = &mem_man_data;
	struct buffer *buffer;
	void *kptr;

	mutex_lock(&mem_man->mutex);
	buffer = idr_find(&ctx->buffers, buff_id);
	if (!buffer) {
		dev_err(mem_man->dev, "%s: buffer id %d not found\n", __func__,
			buff_id);
		mutex_unlock(&mem_man->mutex);
		return NULL;
	}
	kptr = buffer->kptr;
	mutex_unlock(&mem_man->mutex);
	return kptr;
}

static void _img_mem_sync_cpu_to_device(struct buffer *buffer)
{
	struct heap *heap = buffer->heap;

	if (heap->ops && heap->ops->sync_cpu_to_dev)
		heap->ops->sync_cpu_to_dev(heap, buffer);

	/* sync to device memory */
	mb();
}

int img_mem_sync_cpu_to_device(struct mem_ctx *ctx, int buff_id)
{
	struct mem_man *mem_man = &mem_man_data;
	struct buffer *buffer;

	mutex_lock(&mem_man->mutex);
	buffer = idr_find(&ctx->buffers, buff_id);
	if (!buffer) {
		dev_err(mem_man->dev, "%s: buffer id %d not found\n", __func__,
			buff_id);
		mutex_unlock(&mem_man->mutex);
		return -EINVAL;
	}

	_img_mem_sync_cpu_to_device(buffer);

	mutex_unlock(&mem_man->mutex);
	return 0;
}

static void _img_mem_sync_device_to_cpu(struct buffer *buffer)
{
	struct heap *heap = buffer->heap;

	if (heap->ops && heap->ops->sync_dev_to_cpu)
		heap->ops->sync_dev_to_cpu(heap, buffer);
}

int img_mem_sync_device_to_cpu(struct mem_ctx *ctx, int buff_id)
{
	struct mem_man *mem_man = &mem_man_data;
	struct buffer *buffer;

	mutex_lock(&mem_man->mutex);
	buffer = idr_find(&ctx->buffers, buff_id);
	if (!buffer) {
		dev_err(mem_man->dev, "%s: buffer id %d not found\n", __func__,
			buff_id);
		mutex_unlock(&mem_man->mutex);
		return -EINVAL;
	}

	_img_mem_sync_device_to_cpu(buffer);

	mutex_unlock(&mem_man->mutex);
	return 0;
}

static struct mmu_page_cfg *mmu_page_alloc(void *arg)
{
	struct mem_man *mem_man = &mem_man_data;
	struct mmu_ctx *mmu_ctx = arg;
	struct mmu_page *page;
	struct buffer *buffer;
	struct heap *heap;
	int ret;

	dev_dbg(mmu_ctx->device, "%s:%d arg %p\n", __func__, __LINE__, arg);

	WARN_ON(!mutex_is_locked(&mem_man->mutex));

	page = kzalloc(sizeof(*page), GFP_KERNEL);
	if (!page)
		return NULL;

	ret = _img_mem_alloc(mmu_ctx->device, mmu_ctx->mem_ctx,
			     mmu_ctx->heap, PAGE_SIZE, 0, &buffer);
	if (ret) {
		dev_err(mmu_ctx->device, "%s: img_mem_alloc failed (%d)\n",
			__func__, ret);
		goto free_page;
	}

	ret = _img_mem_map_km(buffer);
	if (ret) {
		dev_err(mmu_ctx->device, "%s: img_mem_map_km failed (%d)\n",
			__func__, ret);
		goto free_buffer;
	}

	page->addr_shift = mmu_ctx->mmu_config_addr_width - 32;
	page->buffer = buffer;
	page->page_cfg.cpu_virt_addr = (uintptr_t)buffer->kptr;

	heap = buffer->heap;
	if (heap->ops && heap->ops->get_sg_table) {
		struct sg_table *sgt;

		ret = heap->ops->get_sg_table(heap, buffer, &sgt);
		if (ret) {
			dev_err(mmu_ctx->device,
				"%s: heap %d buffer %d no sg_table!\n",
				__func__, heap->id, buffer->id);
			ret = -EINVAL;
			goto free_buffer;
		}
		page->page_cfg.phys_addr = sg_phys(sgt->sgl);
	} else {
		dev_err(mmu_ctx->device, "%s: heap %d buffer %d no get_sg!\n",
			__func__, heap->id, buffer->id);
		ret = -EINVAL;
		goto free_buffer;
	}

	dev_dbg(mmu_ctx->device, "%s:%d virt addr %#lx\n", __func__, __LINE__,
		page->page_cfg.cpu_virt_addr);
	dev_dbg(mmu_ctx->device, "%s:%d phys addr %#llx\n", __func__, __LINE__,
		page->page_cfg.phys_addr);
	return &page->page_cfg;

free_buffer:
	_img_mem_free(buffer);
	kfree(buffer);
free_page:
	kfree(page);
	return NULL;
}

static void mmu_page_free(struct mmu_page_cfg *arg)
{
	struct mem_man *mem_man = &mem_man_data;
	struct mmu_page *page;

	page = container_of(arg, struct mmu_page, page_cfg);

	WARN_ON(!mutex_is_locked(&mem_man->mutex));

	_img_mem_free(page->buffer);
	kfree(page->buffer);
	kfree(page);
}

static void mmu_page_write(struct mmu_page_cfg *page_cfg,
			   unsigned int offset, u64 addr,
			   unsigned int flags)
{
	u32 *mem = (u32 *)page_cfg->cpu_virt_addr;
	struct mmu_page *mmu_page;
	struct heap *heap;

	if (trace_physical_pages)
		pr_debug("%s: off %#x addr %#llx flags %#x\n",
			 __func__, offset, addr, flags);

	mmu_page = container_of(page_cfg, struct mmu_page, page_cfg);
	heap = mmu_page->buffer->heap;

	/* skip translation when flags are zero, assuming address is invalid */
	if (flags && heap->to_dev_addr)
		addr = heap->to_dev_addr(&heap->options, addr);
	addr >>= mmu_page->addr_shift;

	mem[offset] = addr | flags;
}

static void mmu_update_page(struct mmu_page_cfg *arg)
{
	struct mem_man *mem_man = &mem_man_data;
	struct mmu_page *page;

	if (trace_physical_pages)
		pr_debug("%s\n", __func__);

	page = container_of(arg, struct mmu_page, page_cfg);

	WARN_ON(!mutex_is_locked(&mem_man->mutex));

	_img_mem_sync_cpu_to_device(page->buffer);
}

int img_mmu_ctx_create(struct device *device, u32 mmu_config_addr_width,
		       struct mem_ctx *mem_ctx, int heap_id,
		       void (*callback_fn)(enum mmu_callback_type type,
					   int buff_id, void *data),
		       void *callback_data, struct mmu_ctx **mmu_ctx)
{
	struct mem_man *mem_man = &mem_man_data;

	static struct mmu_info mmu_functions = {
		.pfn_page_alloc = mmu_page_alloc,
		.pfn_page_free = mmu_page_free,
		.pfn_page_write = mmu_page_write,
		.pfn_page_update = mmu_update_page,
	};
	struct mmu_ctx *ctx;
	int ret;

	if (mmu_config_addr_width < 32) {
		dev_err(device,
			"%s: invalid addr_width (%d) must be >= 32 !\n",
			__func__, mmu_config_addr_width);
		return -EINVAL;
	}

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->device = device;
	ctx->mem_ctx = mem_ctx;
	ctx->mmu_config_addr_width = mmu_config_addr_width;

	mutex_lock(&mem_man->mutex);

	ctx->heap = idr_find(&mem_man->heaps, heap_id);
	if (!ctx->heap) {
		dev_err(device, "%s: invalid heap_id (%d)!\n", __func__,
			heap_id);
		mutex_unlock(&mem_man->mutex);
		kfree(ctx);
		return -EINVAL;
	}

	mmu_functions.alloc_ctx = ctx;
	ctx->mmu_dir = mmu_create_directory(&mmu_functions);
	if (IS_ERR(ctx->mmu_dir)) {
		ret = PTR_ERR(ctx->mmu_dir);
		dev_err(device, "%s: directory create failed (%d)!\n", __func__,
			ret);
		ctx->mmu_dir = NULL;
		mutex_unlock(&mem_man->mutex);
		kfree(ctx);
		return ret;
	}

	list_add(&ctx->mem_ctx_entry, &mem_ctx->mmu_ctxs);
	INIT_LIST_HEAD(&ctx->mappings);

	ctx->callback_fn = callback_fn;
	ctx->callback_data = callback_data;

	*mmu_ctx = ctx;

	mutex_unlock(&mem_man->mutex);

	return 0;
}

static void _img_mmu_ctx_destroy(struct mmu_ctx *ctx)
{
	struct mem_man *mem_man = &mem_man_data;
	int ret;

	while (!list_empty(&ctx->mappings)) {
		struct mmu_ctx_mapping *map;

		map = list_first_entry(&ctx->mappings,
				       struct mmu_ctx_mapping, mmu_ctx_entry);
		dev_info(ctx->device,
			 "%s: found mapped buffer %d (size %zu)\n",
			 __func__, map->buffer->id, map->buffer->request_size);

		_img_mmu_unmap(map);

		kfree(map);
	}

	ret = mmu_destroy_directory(ctx->mmu_dir);
	if (ret)
		dev_err(mem_man->dev, "mmu_destroy_directory failed (%d)!\n",
			ret);
	list_del(&ctx->mem_ctx_entry);
}

void img_mmu_ctx_destroy(struct mmu_ctx *ctx)
{
	struct mem_man *mem_man = &mem_man_data;

	mutex_lock(&mem_man->mutex);
	_img_mmu_ctx_destroy(ctx);
	mutex_unlock(&mem_man->mutex);

	kfree(ctx);
}

int img_mmu_map_sg(struct mmu_ctx *mmu_ctx, struct mem_ctx *mem_ctx,
		   int buff_id, struct sg_table *sgt, u32 virt_addr,
		   unsigned int map_flags)
{
	struct mem_man *mem_man = &mem_man_data;
	struct mmu_ctx_mapping *mapping;
	struct mmu_heap_alloc heap_alloc;
	struct buffer *buffer;
	int ret = 0;

	dev_dbg(mmu_ctx->device, "%s sgt %p virt_addr %#x\n", __func__,
		sgt, virt_addr);

	mapping = kzalloc(sizeof(*mapping), GFP_KERNEL);
	if (!mapping)
		return -ENOMEM;

	mutex_lock(&mem_man->mutex);
	buffer = idr_find(&mem_ctx->buffers, buff_id);
	if (!buffer) {
		dev_err(mmu_ctx->device, "%s: buffer id %d not found\n",
			__func__, buff_id);
		ret = -EINVAL;
		goto error;
	}
	dev_dbg(mmu_ctx->device, "%s buffer %d 0x%p size %zu virt_addr %#x\n",
		__func__, buff_id, buffer, buffer->request_size, virt_addr);

	heap_alloc.virt_addr = virt_addr;
	heap_alloc.alloc_size = buffer->actual_size;

	mapping->mmu_ctx = mmu_ctx;
	mapping->buffer = buffer;
	mapping->virt_addr = virt_addr;

	if (sgt) {
		mapping->map = mmu_directory_map_sg(mmu_ctx->mmu_dir, sgt->sgl,
						    &heap_alloc, map_flags);
		if (IS_ERR(mapping->map)) {
			ret = PTR_ERR(mapping->map);
			mapping->map = NULL;
		}
	} else {
		dev_err(mmu_ctx->device, "%s: buffer %d no get_sg!\n",
			__func__, buffer->id);
		ret = -EINVAL;
		goto error;
	}
	if (ret) {
		dev_err(mmu_ctx->device, "mmu_directory_map_sg failed (%d)!\n",
			ret);
		goto error;
	}

	list_add(&mapping->mmu_ctx_entry, &mmu_ctx->mappings);
	list_add(&mapping->buffer_entry, &mapping->buffer->mappings);

	if (mmu_ctx->callback_fn)
		mmu_ctx->callback_fn(MMU_CALLBACK_MAP, buffer->id,
				     mmu_ctx->callback_data);

	mutex_unlock(&mem_man->mutex);
	return 0;

error:
	mutex_unlock(&mem_man->mutex);
	kfree(mapping);
	return ret;
}

int img_mmu_map(struct mmu_ctx *mmu_ctx, struct mem_ctx *mem_ctx,
		int buff_id, u32 virt_addr, unsigned int map_flags)
{
	struct mem_man *mem_man = &mem_man_data;
	struct mmu_ctx_mapping *mapping;
	struct mmu_heap_alloc heap_alloc;
	struct buffer *buffer;
	struct heap *heap;
	int ret;

	dev_dbg(mmu_ctx->device, "%s buffer %d virt_addr %#x\n", __func__,
		buff_id, virt_addr);

	mapping = kzalloc(sizeof(*mapping), GFP_KERNEL);
	if (!mapping)
		return -ENOMEM;

	mutex_lock(&mem_man->mutex);
	buffer = idr_find(&mem_ctx->buffers, buff_id);
	if (!buffer) {
		dev_err(mmu_ctx->device, "%s: buffer id %d not found\n",
			__func__, buff_id);
		ret = -EINVAL;
		goto error;
	}
	dev_dbg(mmu_ctx->device, "%s buffer %d 0x%p size %zu virt_addr %#x\n",
		__func__, buff_id, buffer, buffer->request_size, virt_addr);

	heap_alloc.virt_addr = virt_addr;
	heap_alloc.alloc_size = buffer->actual_size;

	mapping->mmu_ctx = mmu_ctx;
	mapping->buffer = buffer;
	mapping->virt_addr = virt_addr;

	heap = buffer->heap;
	if (heap->ops && heap->ops->get_sg_table) {
		struct sg_table *sgt;

		ret = heap->ops->get_sg_table(heap, buffer, &sgt);
		if (ret) {
			dev_err(mmu_ctx->device,
				"%s: heap %d buffer %d no sg_table!\n",
				__func__, heap->id, buffer->id);
			goto error;
		}

		mapping->map = mmu_directory_map_sg(mmu_ctx->mmu_dir, sgt->sgl,
						    &heap_alloc, map_flags);
		if (IS_ERR(mapping->map)) {
			ret = PTR_ERR(mapping->map);
			mapping->map = NULL;
		}
	} else {
		dev_err(mmu_ctx->device, "%s: heap %d buffer %d no get_sg!\n",
			__func__, heap->id, buffer->id);
		ret = -EINVAL;
		goto error;
	}
	if (ret) {
		dev_err(mmu_ctx->device, "mmu_directory_map failed (%d)!\n",
			ret);
		goto error;
	}

	list_add(&mapping->mmu_ctx_entry, &mmu_ctx->mappings);
	list_add(&mapping->buffer_entry, &mapping->buffer->mappings);

	if (mmu_ctx->callback_fn)
		mmu_ctx->callback_fn(MMU_CALLBACK_MAP, buffer->id,
				     mmu_ctx->callback_data);

	mutex_unlock(&mem_man->mutex);
	return 0;

error:
	mutex_unlock(&mem_man->mutex);
	kfree(mapping);
	return ret;
}

static void _img_mmu_unmap(struct mmu_ctx_mapping *mapping)
{
	struct mmu_ctx *ctx = mapping->mmu_ctx;
	int res;

	dev_dbg(ctx->device, "%s:%d mapping %p buffer %d\n", __func__,
		__LINE__, mapping, mapping->buffer->id);

	res = mmu_directory_unmap(mapping->map);
	if (res)
		dev_warn(ctx->device, "mmu_directory_unmap failed (%d)!\n",
			 res);

	list_del(&mapping->mmu_ctx_entry);
	list_del(&mapping->buffer_entry);

	if (ctx->callback_fn)
		ctx->callback_fn(MMU_CALLBACK_UNMAP, mapping->buffer->id,
				 ctx->callback_data);
}

int img_mmu_unmap(struct mmu_ctx *mmu_ctx, struct mem_ctx *mem_ctx,
		  int buff_id)
{
	struct mem_man *mem_man = &mem_man_data;
	struct mmu_ctx_mapping *mapping;
	struct list_head *lst;

	dev_dbg(mmu_ctx->device, "%s:%d buffer %d\n", __func__, __LINE__,
		buff_id);

	mutex_lock(&mem_man->mutex);

	mapping = NULL;
	list_for_each(lst, &mmu_ctx->mappings) {
		struct mmu_ctx_mapping *m;

		m = list_entry(lst, struct mmu_ctx_mapping, mmu_ctx_entry);
		if (m->buffer->id == buff_id) {
			mapping = m;
			break;
		}
	}

	if (!mapping) {
		dev_err(mmu_ctx->device, "%s: buffer id %d not found\n",
			__func__, buff_id);
		mutex_unlock(&mem_man->mutex);
		return -EINVAL;
	}

	_img_mmu_unmap(mapping);

	mutex_unlock(&mem_man->mutex);
	kfree(mapping);
	return 0;
}

int img_mmu_get_ptd(const struct mmu_ctx *ctx, unsigned int *ptd)
{
	struct mem_man *mem_man = &mem_man_data;
	struct mmu_page_cfg *page_cfg;
	phys_addr_t addr;

	mutex_lock(&mem_man->mutex);

	page_cfg = mmu_directory_get_page(ctx->mmu_dir);
	if (!page_cfg) {
		mutex_unlock(&mem_man->mutex);
		return -EINVAL;
	}

	addr = page_cfg->phys_addr;
	if (ctx->heap->to_dev_addr)
		addr = ctx->heap->to_dev_addr(&ctx->heap->options, addr);

	mutex_unlock(&mem_man->mutex);

	*ptd = (unsigned int)(addr >>= VXD_MMU_SHIFT);

	dev_dbg(ctx->device, "%s: addr %#llx ptd %#x\n", __func__,
		page_cfg->phys_addr, *ptd);
	return 0;
}

/*
 * Initialisation
 */
int img_mem_init(struct device *dev)
{
	struct mem_man *mem_man = &mem_man_data;

	mem_man->dev = dev;
	idr_init(&mem_man->heaps);
	INIT_LIST_HEAD(&mem_man->mem_ctxs);
	mutex_init(&mem_man->mutex);

	return 0;
}

void img_mem_exit(void)
{
	struct mem_man *mem_man = &mem_man_data;
	struct heap *heap;
	int heap_id;

	/* keeps mutex checks (WARN_ON) happy, this will never actually wait */
	mutex_lock(&mem_man->mutex);

	while (!list_empty(&mem_man->mem_ctxs)) {
		struct mem_ctx *mc;

		mc = list_first_entry(&mem_man->mem_ctxs,
				      struct mem_ctx, mem_man_entry);
		dev_warn(mem_man->dev, "%s derelict memory context %p!\n",
			 __func__, mc);
		_img_mem_destroy_ctx(mc);
		kfree(mc);
	}

	heap_id = MIN_HEAP;
	heap = idr_get_next(&mem_man->heaps, &heap_id);
	while (heap) {
		dev_warn(mem_man->dev, "%s derelict heap %d!\n", __func__,
			 heap_id);
		_img_mem_del_heap(heap);
		kfree(heap);
		heap_id = MIN_HEAP;
		heap = idr_get_next(&mem_man->heaps, &heap_id);
	}
	idr_destroy(&mem_man->heaps);

	mutex_unlock(&mem_man->mutex);

	mutex_destroy(&mem_man->mutex);
}
