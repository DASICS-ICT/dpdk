/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2026
 */

#include "cdma_common.h"

#include <errno.h>
#include <stddef.h>

#ifdef RTE_ENABLE_DBCHECKER
#include <rte_dbchecker.h>

static unsigned int cdma_dbchecker_refcnt;
#endif

/*
 * Mirror igb_ethdev: first user calls dbchecker_module_init_hook(), last user
 * calls err_handler + dbchecker_module_exit_hook().
 */
int
cdma_common_dbchecker_acquire(void)
{
#ifdef RTE_ENABLE_DBCHECKER
	if (cdma_dbchecker_refcnt++ == 0) {
		int ret = dbchecker_module_init_hook();

		if (ret) {
			cdma_dbchecker_refcnt--;
			return ret;
		}
	}
#endif
	return 0;
}

void
cdma_common_dbchecker_release(void)
{
#ifdef RTE_ENABLE_DBCHECKER
	if (--cdma_dbchecker_refcnt == 0) {
		(void)dbchecker_err_handler();
		dbchecker_module_exit_hook();
	}
#endif
}

int
cdma_common_copy_iova_prepare(int use_dbchecker, uint16_t dbchecker_dev_id,
	uint64_t src_iova, uint64_t dst_iova, uint32_t len,
	uint64_t *src_prog_iova, uint64_t *dst_prog_iova)
{
	if (src_prog_iova == NULL || dst_prog_iova == NULL)
		return -EINVAL;

	*src_prog_iova = src_iova;
	*dst_prog_iova = dst_iova;

	if (!use_dbchecker)
		return 0;

#ifndef RTE_ENABLE_DBCHECKER
	(void)dbchecker_dev_id;
	(void)len;
	return 0;
#else
	{
		dma_addr_t src_prog;
		dma_addr_t dst_prog;

		/* CDMA reads src / writes dst — same dma direction naming as igb tx/rx. */
		src_prog = dbchecker_alloc_mtdt((dma_addr_t)src_iova, len,
			DMA_TO_DEVICE, dbchecker_dev_id);
		dst_prog = dbchecker_alloc_mtdt((dma_addr_t)dst_iova, len,
			DMA_FROM_DEVICE, dbchecker_dev_id);
		if (src_prog == (dma_addr_t)-1 || dst_prog == (dma_addr_t)-1) {
			if (src_prog != (dma_addr_t)-1)
				dbchecker_free_mtdt(src_prog);
			if (dst_prog != (dma_addr_t)-1)
				dbchecker_free_mtdt(dst_prog);
			return -ENOMEM;
		}
		*src_prog_iova = (uint64_t)src_prog;
		*dst_prog_iova = (uint64_t)dst_prog;
		return 0;
	}
#endif
}

void
cdma_common_copy_iova_finish(int use_dbchecker,
	uint64_t src_prog_iova, uint64_t dst_prog_iova)
{
	if (!use_dbchecker)
		return;

#ifndef RTE_ENABLE_DBCHECKER
	(void)src_prog_iova;
	(void)dst_prog_iova;
	return;
#else
	dbchecker_free_mtdt((dma_addr_t)src_prog_iova);
	dbchecker_free_mtdt((dma_addr_t)dst_prog_iova);
#endif
}
