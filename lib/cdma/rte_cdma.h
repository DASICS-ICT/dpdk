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
}

struct cdma_params {
	const char *bus_devices_path;
	const char *device_name;
	uint32_t region_index;
	uint32_t timeout_cycles;
	int debug_log;
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
};

typedef int (*cdma_pre_submit_fn)(void *arg);
typedef int (*cdma_post_complete_fn)(void *arg);

void cdma_default_params(struct cdma_params *params);
int cdma_open(struct cdma_dev *dev, const struct cdma_params *params);
int cdma_reset(struct cdma_dev *dev);
int cdma_copy(struct cdma_dev *dev, uint64_t src_iova, uint64_t dst_iova,
	uint32_t len);
int cdma_copy_with_pre_submit(struct cdma_dev *dev, uint64_t src_iova,
	uint64_t dst_iova, uint32_t len, cdma_pre_submit_fn pre_submit,
	void *pre_submit_arg);
int cdma_copy_with_hooks(struct cdma_dev *dev, uint64_t src_iova,
	uint64_t dst_iova, uint32_t len, cdma_pre_submit_fn pre_submit,
	void *pre_submit_arg, cdma_post_complete_fn post_complete,
	void *post_complete_arg);
void cdma_close(struct cdma_dev *dev);

/*
 * Usage (after rte_eal_init()):
 *   deps += ['cdma']
 *   #include <rte_cdma.h>
 * Pass IOVA from DPDK-managed memory (e.g. rte_malloc).
 * dbchecker support is controlled entirely by the build flag -Denable_dbchecker=true;
 * no runtime switch is needed.
 * cdma_copy_with_pre_submit() invokes its callback after dbchecker metadata is
 * installed but before the CDMA registers are programmed. It is intended for
 * cache-state preparation and other measurement-only hooks.
 * cdma_copy_with_hooks() additionally invokes post_complete after the CDMA is
 * idle but before dbchecker metadata is released. This permits a measurement
 * window that excludes the CPU-side metadata cleanup.
 */

#endif /* RTE_CDMA_H */
