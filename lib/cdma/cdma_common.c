/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2026
 */

#include "cdma_common.h"

#include <errno.h>
#include <stddef.h>

#ifdef RTE_ENABLE_DBCHECKER
#include <rte_dbchecker.h>
#endif

int
cdma_common_copy_iova_prepare(uint64_t src_iova, uint64_t dst_iova,
	uint32_t len, uint64_t *src_prog_iova, uint64_t *dst_prog_iova)
{
	if (src_prog_iova == NULL || dst_prog_iova == NULL)
		return -EINVAL;

	*src_prog_iova = src_iova;
	*dst_prog_iova = dst_iova;

#ifdef RTE_ENABLE_DBCHECKER
	{
		dma_addr_t src_prog;
		dma_addr_t dst_prog;

		/* CDMA reads src / writes dst — same dma direction naming as igb tx/rx. */
		src_prog = dbchecker_alloc_mtdt((dma_addr_t)src_iova, len,
			DMA_TO_DEVICE, CDMA_DBCHECKER_DEV_ID);
		dst_prog = dbchecker_alloc_mtdt((dma_addr_t)dst_iova, len,
			DMA_FROM_DEVICE, CDMA_DBCHECKER_DEV_ID);
		if (src_prog == (dma_addr_t)-1 || dst_prog == (dma_addr_t)-1) {
			if (src_prog != (dma_addr_t)-1)
				dbchecker_free_mtdt(src_prog);
			if (dst_prog != (dma_addr_t)-1)
				dbchecker_free_mtdt(dst_prog);
			return -ENOMEM;
		}
		*src_prog_iova = (uint64_t)src_prog;
		*dst_prog_iova = (uint64_t)dst_prog;
	}
#else
	(void)len;
#endif
	return 0;
}

void
cdma_common_copy_iova_finish(uint64_t src_prog_iova, uint64_t dst_prog_iova)
{
#ifdef RTE_ENABLE_DBCHECKER
	dbchecker_free_mtdt((dma_addr_t)src_prog_iova);
	dbchecker_free_mtdt((dma_addr_t)dst_prog_iova);
#else
	(void)src_prog_iova;
	(void)dst_prog_iova;
#endif
}
