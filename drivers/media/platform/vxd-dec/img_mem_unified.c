// SPDX-License-Identifier: GPL-2.0
/*
 * IMG DEC Memory Manager for unified memory
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

#include <linux/dma-mapping.h>
#include <linux/gfp.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include "img_mem_man.h"

static int trace_physical_pages;

static int unified_alloc(struct device *device, struct heap *heap,
			 size_t size, enum mem_attr attr,
			 struct buffer *buffer)
{
	struct sg_table *sgt;
	struct scatterlist *sgl;
	int pages;
	int ret;

	dev_dbg(device, "%s:%d buffer %d (0x%p)\n", __func__, __LINE__,
		buffer->id, buffer);

	sgt = kmalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return -ENOMEM;

	pages = (size + PAGE_SIZE - 1) / PAGE_SIZE;

	ret = sg_alloc_table(sgt, pages, GFP_KERNEL);
	if (ret)
		goto sg_alloc_table_failed;

	sgl = sgt->sgl;
	while (sgl) {
		struct page *page;
		dma_addr_t dma_addr;

		page = alloc_page(heap->options.unified.gfp_type);
		if (!page) {
			dev_err(device, "%s alloc_page failed!\n", __func__);
			ret = -ENOMEM;
			goto alloc_page_failed;
		}
		if (trace_physical_pages)
			dev_dbg(device, "%s:%d phys %#llx size %lu page_address %p\n",
				__func__, __LINE__,
				(unsigned long long)page_to_phys(page),
				PAGE_SIZE, page_address(page));

		/*
		 * dma_map_page() is probably going to fail if alloc flags are
		 * GFP_HIGHMEM, since it is not mapped to CPU. Hopefully, this
		 * will never happen because memory of this sort cannot be used
		 * for DMA anyway. To check if this is the case, build with
		 * debug, set trace_physical_pages=1 and check if page_address
		 * printed above is NULL
		 */
		dma_addr = dma_map_page(device, page, 0, PAGE_SIZE,
					DMA_BIDIRECTIONAL);
		if (dma_mapping_error(device, dma_addr)) {
			__free_page(page);
			dev_err(device, "%s dma_map_page failed!\n", __func__);
			ret = -EIO;
			goto alloc_page_failed;
		}
		dma_unmap_page(device, dma_addr, PAGE_SIZE, DMA_BIDIRECTIONAL);

		sg_set_page(sgl, page, PAGE_SIZE, 0);

		sgl = sg_next(sgl);
	}

	buffer->priv = sgt;
	return 0;

alloc_page_failed:
	sgl = sgt->sgl;
	while (sgl) {
		struct page *page = sg_page(sgl);

		if (page)
			__free_page(page);

		sgl = sg_next(sgl);
	}
	sg_free_table(sgt);
sg_alloc_table_failed:
	kfree(sgt);
	return ret;
}

static void unified_free(struct heap *heap, struct buffer *buffer)
{
	struct device *dev = buffer->device;
	struct sg_table *sgt = buffer->priv;
	struct scatterlist *sgl;

	dev_dbg(dev, "%s:%d buffer %d (0x%p)\n", __func__, __LINE__,
		buffer->id, buffer);

	if (buffer->kptr) {
		dev_dbg(dev, "%s vunmap 0x%p\n", __func__, buffer->kptr);
		dma_unmap_sg(dev, sgt->sgl, sgt->orig_nents, DMA_FROM_DEVICE);
		vunmap(buffer->kptr);
	}

	sgl = sgt->sgl;
	while (sgl) {
		__free_page(sg_page(sgl));
		sgl = sg_next(sgl);
	}
	sg_free_table(sgt);
	kfree(sgt);
}

static int unified_map_km(struct heap *heap, struct buffer *buffer)
{
	struct device *dev = buffer->device;
	struct sg_table *sgt = buffer->priv;
	struct scatterlist *sgl = sgt->sgl;
	unsigned int num_pages = sg_nents(sgl);
	struct page **pages;
	pgprot_t prot;
	int ret;
	int i;

	dev_dbg(dev, "%s:%d buffer %d (0x%p)\n", __func__, __LINE__,
		buffer->id, buffer);

	if (buffer->kptr) {
		dev_warn(dev, "%s called for already mapped buffer %d\n",
			 __func__, buffer->id);
		return 0;
	}

	pages = kmalloc_array(num_pages, sizeof(struct page *), GFP_KERNEL);
	if (!pages)
		return -ENOMEM;

	prot = PAGE_KERNEL;
	prot = pgprot_writecombine(prot);

	i = 0;
	while (sgl) {
		pages[i++] = sg_page(sgl);
		sgl = sg_next(sgl);
	}

	buffer->kptr = vmap(pages, num_pages, VM_MAP, prot);
	kfree(pages);
	if (!buffer->kptr) {
		dev_err(dev, "%s vmap failed!\n", __func__);
		return -EFAULT;
	}

	ret = dma_map_sg(dev, sgt->sgl, sgt->orig_nents, DMA_FROM_DEVICE);

	if (ret <= 0) {
		dev_err(dev, "%s dma_map_sg failed!\n", __func__);
		vunmap(buffer->kptr);
		return -EFAULT;
	}
	dev_dbg(dev, "%s:%d buffer %d orig_nents %d nents %d\n", __func__,
		__LINE__, buffer->id, sgt->orig_nents, ret);
	sgt->nents = ret;

	dev_dbg(dev, "%s:%d buffer %d vmap to 0x%p\n", __func__, __LINE__,
		buffer->id, buffer->kptr);

	return 0;
}

static int unified_get_sg_table(struct heap *heap, struct buffer *buffer,
				struct sg_table **sg_table)
{
	*sg_table = buffer->priv;
	return 0;
}

static void unified_sync_cpu_to_dev(struct heap *heap, struct buffer *buffer)
{
	struct device *dev = buffer->device;
	struct sg_table *sgt = buffer->priv;

	if (!buffer->kptr)
		return;

	dev_dbg(dev, "%s:%d buffer %d (0x%p)\n", __func__, __LINE__,
		buffer->id, buffer);

	dma_sync_sg_for_device(dev, sgt->sgl, sgt->orig_nents, DMA_TO_DEVICE);
}

static void unified_sync_dev_to_cpu(struct heap *heap, struct buffer *buffer)
{
	struct device *dev = buffer->device;
	struct sg_table *sgt = buffer->priv;

	if (!buffer->kptr)
		return;

	dev_dbg(dev, "%s:%d buffer %d (0x%p)\n", __func__, __LINE__,
		buffer->id, buffer);

	dma_sync_sg_for_cpu(dev, sgt->sgl, sgt->orig_nents, DMA_TO_DEVICE);
}

static void unified_heap_destroy(struct heap *heap)
{
}

static struct heap_ops unified_heap_ops = {
	.alloc = unified_alloc,
	.free = unified_free,
	.map_km = unified_map_km,
	.get_sg_table = unified_get_sg_table,
	.sync_cpu_to_dev = unified_sync_cpu_to_dev,
	.sync_dev_to_cpu = unified_sync_dev_to_cpu,
	.destroy = unified_heap_destroy,
};

int img_mem_unified_init(const struct heap_config *heap_cfg,
			 struct heap *heap)
{
	heap->ops = &unified_heap_ops;
	return 0;
}
