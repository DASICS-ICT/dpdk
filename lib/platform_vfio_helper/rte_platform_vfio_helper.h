/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2026
 */

#ifndef RTE_PLATFORM_VFIO_HELPER_H
#define RTE_PLATFORM_VFIO_HELPER_H

#include <stddef.h>
#include <stdint.h>

/** Default sysfs path for Linux platform bus devices. */
#define RTE_PLATFORM_VFIO_DEFAULT_BUS_DEVICES_PATH "/sys/bus/platform/devices"

struct rte_platform_vfio_params {
	const char *bus_devices_path;
	const char *device_name;
	uint32_t region_index;
	int debug_log;
};

struct rte_platform_vfio_device {
	volatile void *regs;
	size_t regs_size;
	int dev_fd;
	const char *bus_devices_path;
	const char *device_name;
	uint32_t region_index;
	int debug_log;
};

/** Zero-initialized struct has dev_fd==0; prefer this or memset + dev_fd=-1 before open. */
#define RTE_PLATFORM_VFIO_DEVICE_INITIALIZER { \
	.regs = NULL, \
	.regs_size = 0, \
	.dev_fd = -1, \
	.bus_devices_path = NULL, \
	.device_name = NULL, \
	.region_index = 0, \
	.debug_log = 0, \
}

void rte_platform_vfio_default_params(struct rte_platform_vfio_params *params);
int rte_platform_vfio_open(struct rte_platform_vfio_device *dev,
	const struct rte_platform_vfio_params *params);
void rte_platform_vfio_close(struct rte_platform_vfio_device *dev);

#endif /* RTE_PLATFORM_VFIO_HELPER_H */
