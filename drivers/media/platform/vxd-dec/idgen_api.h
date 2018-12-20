/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ID generation manager API.
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
#ifndef __IDGENAPI_H__
#define __IDGENAPI_H__

#include "img_errors.h"

/*
 * This function is used to create Id generation context.
 * NOTE: Should only be called once to setup the context structure.
 * NOTE: The client is responsible for providing thread/process safe locks on
 * the context structure to maintain coherence.
 */
int idgen_createcontext(unsigned int maxid, unsigned int blksize,
			int incid, void **idgenhandle);

/*
 * This function is used to destroy an Id generation context.  This function
 * discards any handle blocks associated with the context.
 * NOTE: The client is responsible for providing thread/process safe locks on
 * the context structure to maintain coherence.
 */
int idgen_destroycontext(void *idgenhandle);

/*
 * This function is used to associate a handle with an Id.
 * NOTE: The client is responsible for providing thread/process safe locks on
 * the context structure to maintain coherency.
 */
int idgen_allocid(void *idgenhandle, void *handle, unsigned int *id);

/*
 * This function is used to free an Id.
 * NOTE: The client is responsible for providing thread/process safe locks on
 * the context structure to maintain coherency.
 */
int idgen_freeid(void *idgenhandle, unsigned int id);

/*
 * This function is used to get the handle associated with an Id.
 * NOTE: The client is responsible for providing thread/process safe locks on
 * the context structure to maintain coherency.
 */
int idgen_gethandle(void *idgenhandle, unsigned int id, void **handle);
#endif /* __IDGENAPI_H__ */
