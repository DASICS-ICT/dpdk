/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2026
 *
 * Internal helpers shared by CDMA (dbchecker refcnt + copy IOVA wrap).
 */

#ifndef CDMA_COMMON_H
#define CDMA_COMMON_H

#include <stdint.h>

int cdma_common_dbchecker_acquire(void);
void cdma_common_dbchecker_release(void);

int cdma_common_copy_iova_prepare(int use_dbchecker, uint16_t dbchecker_dev_id,
	uint64_t src_iova, uint64_t dst_iova, uint32_t len,
	uint64_t *src_prog_iova, uint64_t *dst_prog_iova);
void cdma_common_copy_iova_finish(int use_dbchecker,
	uint64_t src_prog_iova, uint64_t dst_prog_iova);

#endif /* CDMA_COMMON_H */
