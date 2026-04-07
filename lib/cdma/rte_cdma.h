/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2026
 */

#ifndef RTE_CDMA_H
#define RTE_CDMA_H

#include <stddef.h>
#include <stdint.h>

#define CDMA_DEFAULT_PLATFORM_BUS_DEVICES_PATH "/sys/bus/platform/devices"
#define CDMA_DEFAULT_DEVICE_NAME "80800000.dma"
#define CDMA_DEFAULT_REGION_INDEX 0u
#define CDMA_DEFAULT_TIMEOUT_CYCLES 1000000u

#define CDMA_DEV_INITIALIZER { \
	.regs = NULL, \
	.regs_size = 0, \
	.dev_fd = -1, \
	.bus_devices_path = NULL, \
	.device_name = NULL, \
	.region_index = 0, \
	.timeout_cycles = 0, \
	.debug_log = 0, \
	.dbchecker_dev_id = 0, \
}

struct cdma_params {
	const char *bus_devices_path;
	const char *device_name;
	uint32_t region_index;
	uint32_t timeout_cycles;
	int debug_log;
	/** Passed to dbchecker_alloc_mtdt when built with -Denable_dbchecker=true; ignored otherwise. */
	uint16_t dbchecker_dev_id;
};

struct cdma_dev {
	volatile void *regs;
	size_t regs_size;
	int dev_fd;
	const char *bus_devices_path;
	const char *device_name;
	uint32_t region_index;
	uint32_t timeout_cycles;
	int debug_log;
	uint16_t dbchecker_dev_id;
};

void cdma_default_params(struct cdma_params *params);
int cdma_open(struct cdma_dev *dev, const struct cdma_params *params);
int cdma_reset(struct cdma_dev *dev);
int cdma_copy(struct cdma_dev *dev, uint64_t src_iova, uint64_t dst_iova,
	uint32_t len);
void cdma_close(struct cdma_dev *dev);

/*
 * Usage (after rte_eal_init()):
 *   deps += ['cdma']
 *   #include <rte_cdma.h>
 * Pass IOVA from DPDK-managed memory (e.g. rte_malloc).
 * With RTE_ENABLE_DBCHECKER, cdma_copy wraps src/dst through dbchecker metadata
 * (set dbchecker_dev_id if non-zero; initialize dbchecker separately).
 */

#endif /* RTE_CDMA_H */
