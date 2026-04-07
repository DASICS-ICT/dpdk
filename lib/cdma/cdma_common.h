/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2026
 *
 * Internal helpers shared by CDMA (dbchecker copy IOVA wrap).
 */

#ifndef CDMA_COMMON_H
#define CDMA_COMMON_H

#include <stdint.h>

/* dbchecker device ID used for all CDMA transfers. */
#define CDMA_DBCHECKER_DEV_ID 1

int cdma_common_copy_iova_prepare(uint64_t src_iova, uint64_t dst_iova,
	uint32_t len, uint64_t *src_prog_iova, uint64_t *dst_prog_iova);
void cdma_common_copy_iova_finish(uint64_t src_prog_iova,
	uint64_t dst_prog_iova);

#endif /* CDMA_COMMON_H */
