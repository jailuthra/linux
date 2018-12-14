/* SPDX-License-Identifier: GPL-2.0 */
/*
 * TAL MMU Extensions.
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
#include "addr_alloc.h"
#include "ra.h"

#ifndef __TALMMU_API_H__
#define __TALMMU_API_H__

#define	TALMMU_MAX_DEVICE_HEAPS	(32)
#define	TALMMU_MAX_TEMPLATES	(32)

/* MMU type */
enum talmmu_mmu_type {
	/* 4kb pages and 32-bit address range */
	TALMMU_MMUTYPE_4K_PAGES_32BIT_ADDR = 0x1,
	/* variable size pages and 32-bit address */
	TALMMU_MMUTYPE_VAR_PAGES_32BIT_ADDR,
	/* 4kb pages and 36-bit address range */
	TALMMU_MMUTYPE_4K_PAGES_36BIT_ADDR,
	/* 4kb pages and 40-bit address range */
	TALMMU_MMUTYPE_4K_PAGES_40BIT_ADDR,
	/* variable size pages and 40-bit address range */
	TALMMU_MMUTYPE_VP_40BIT
};

/* Device flags */
enum talmmu_dev_flags {
	TALMMU_DEVFLAGS_NONE = 0x0,
};

/* Heap type */
enum talmmu_heap_type {
	TALMMU_HEAP_SHARED_EXPORTED,
	TALMMU_HEAP_PERCONTEXT,
};

/* Heap flags */
enum talmmu_eheapflags {
	TALMMU_HEAPFLAGS_NONE = 0x0,
	TALMMU_HEAPFLAGS_SET_CACHE_CONSISTENCY = 0x00000001,
	TALMMU_HEAPFLAGS_128BYTE_INTERLEAVE = 0x00000002,
	TALMMU_HEAPFLAGS_256BYTE_INTERLEAVE = 0x00000004
};

/* Contains the device memory information */
struct talmmu_devmem_info {
	/* device id */
	u32 device_id;
	/* mmu type */
	enum talmmu_mmu_type mmu_type;
	/* Device flags - bit flags that can be combined */
	enum talmmu_dev_flags dev_flags;
	/* Name of the memory space for page directory allocations */
	char *pagedir_memspace_name;
	/* Name of the memory space for page table allocations */
	char *pagetable_memspace_name;
	/* Page size in bytes */
	u32 page_size;
	/* PTD alignment, must be multiple of Page size */
	u32 ptd_alignment;
};

struct talmmu_heap_info {
	/* heap id */
	u32 heap_id;
	/* heap type */
	enum talmmu_heap_type heap_type;
	/* heap flags - bit flags that can be combined */
	enum talmmu_eheapflags heap_flags;
	/* Name of the memory space for memory allocations */
	char *memspace_name;
	/* Base device virtual address */
	u32 basedev_virtaddr;
	/* size in bytes */
	u32 size;
};

/* Device memory template information */
struct talmmu_dm_tmpl {
	/* list */
	struct lst_t list;
	/* Copy of device memory info structure */
	struct talmmu_devmem_info devmem_info;
	/* Memory space ID for PTD allocations */
	void *ptd_memspace_hndl;
	/* Memory space ID for Page Table allocations */
	void *ptentry_memspace_hndl;
	/* number of heaps */
	u32 num_heaps;
	/* Array of heap pointers */
	struct talmmu_devmem_heap *devmem_heap[TALMMU_MAX_DEVICE_HEAPS];
	/* Number of active contexts */
	u32 num_ctxs;
	/* List of device memory context created from this template */
	struct lst_t devmem_ctx_list;
	/* Number of bits to shift right to obtain page number */
	u32 page_num_shift;
	/* Mask to extract byte-within-page */
	u32 byte_in_pagemask;
	/* Heap alignment */
	u32 heap_alignment;
	/* Page table entries/page */
	u32 pagetable_entries_perpage;
	/* Number of bits to shift right to obtain page table number */
	u32 pagetable_num_shift;
	/* Mask to extract index-within-page-table */
	u32 index_in_pagetable_mask;
	/* Number of bits to shift right to obtain page dir number */
	u32 pagedir_num_shift;
};

/* Device memory heap information */
struct talmmu_devmem_heap {
	/* list item */
	struct lst_t list;
	/* Copy of the heap info structure */
	struct talmmu_heap_info heap_info;
	/* Pointer to the device memory template */
	struct talmmu_dm_tmpl *devmem_template;
	/* true if device virtual address offset allocated externally by user */
	u32 ext_dev_virtaddr;
	/* list of memory allocations */
	struct lst_t memory_list;
	/* Memory space ID for memory allocations */
	void *memspace_hndl;
	/* Address context structure */
	struct addr_context ctx;
	/* Regions structure */
	struct addr_region regions;
	/* Indicates whther MMU must be flushed */
	u32 page_changes;
	/* No. of pages in this heap */
	u32 num_pages;
	/* No. of page tables in this heap */
	u32 num_pagetables;
	/* Pointer to an array of page table structures */
	struct talmmu_pagetable **pagetable;
	/* size of heap guard band */
	u32 guardband;
};

struct talmmu_devmem_ctx {
	/* list item */
	struct lst_t list;
	/* Pointer to device template */
	struct talmmu_dm_tmpl *devmem_template;
	/* No. of heaps */
	u32 num_heaps;
	/* Array of heap pointers */
	struct talmmu_devmem_heap *devmem_heap[TALMMU_MAX_DEVICE_HEAPS];
	/* The MMU context id */
	u32 mmu_ctx_id;
	/* Pointer to the memory that represents Page directory */
	u32 *pagedir;
};

struct talmmu_memory {
	/* list item */
	struct lst_t list;
	/* Heap from which memory was allocated */
	struct talmmu_devmem_heap *devmem_heap;
	/* Context through which memory was allocated */
	struct talmmu_devmem_ctx *devmem_ctx;
	/* size */
	u32 size;
	/* alignment */
	u32 alignment;
	/* device virtual offset of allocation */
	u32 dev_virtoffset;
	/* true if device virtual address offset allocated externally by user */
	u32 ext_dev_virtaddr;
};

enum talmmu_heap_option_id {
	/* Add guard band to all mallocs */
	TALMMU_HEAP_OPT_ADD_GUARD_BAND,
	TALMMU_HEAP_OPT_SET_MEM_ATTRIB,
	TALMMU_HEAP_OPT_SET_MEM_POOL,

	/* Placeholder */
	TALMMU_NO_OF_OPTIONS

};

struct talmmu_guardband_options {
	u32 guardband;
};

union talmmu_heap_options {
	/* Guardband parameters */
	struct talmmu_guardband_options guardband_opt;
};

int talmmu_init(void);
int talmmu_deinit(void);
int talmmu_devmem_template_create(struct talmmu_devmem_info *devmem_info,
				  void **devmem_template_hndl);
int talmmu_devmem_heap_add(void *devmem_tmplt_hndl,
			   struct talmmu_heap_info *heap_info_arg);
int talmmu_devmem_template_destroy(void *devmem_tmplt_hndl);
int talmmu_devmem_ctx_create(void *devmem_tmplt_hndl,
			     u32 mmu_ctx_id,
			     void **devmem_ctx_hndl);
int talmmu_devmem_ctx_destroy(void *devmem_ctx_hndl);
int talmmu_get_heap_handle(u32 hid,
			   void *devmem_ctx_hndl,
			   void **devmem_heap_hndl);
int talmmu_devmem_heap_empty(void *devmem_heap_hndl);
int talmmu_devmem_heap_options(void *devmem_heap_hndl,
			       enum talmmu_heap_option_id heap_opt_id,
			       union talmmu_heap_options heap_options);
int talmmu_devmem_addr_alloc(void *devmem_ctx_hndl,
			     void *devmem_heap_hndl,
			     u32 size,
			     u32 align,
			     void **mem_hndl);
int talmmu_devmem_addr_free(void *mem_hndl);
int talmmu_get_dev_virt_addr(void *mem_hndl,
			     u32 *dev_virt);

#endif /* __TALMMU_API_H__ */
