/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2026
 */

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <eal_export.h>
#include <rte_vfio.h>

#include "rte_platform_vfio_helper.h"

static void
vfio_helper_log(int enabled, const char *fmt, ...)
{
	va_list ap;

	if (!enabled)
		return;
	va_start(ap, fmt);
	printf("[platform_vfio] ");
	vprintf(fmt, ap);
	printf("\n");
	fflush(stdout);
	va_end(ap);
}

static void
resolve_params(const struct rte_platform_vfio_params *params,
	struct rte_platform_vfio_params *cfg)
{
	rte_platform_vfio_default_params(cfg);
	if (params == NULL)
		return;
	if (params->bus_devices_path != NULL)
		cfg->bus_devices_path = params->bus_devices_path;
	if (params->device_name != NULL)
		cfg->device_name = params->device_name;
	if (params->region_index != 0)
		cfg->region_index = params->region_index;
	cfg->debug_log = params->debug_log;
}

RTE_EXPORT_SYMBOL(rte_platform_vfio_default_params)
void
rte_platform_vfio_default_params(struct rte_platform_vfio_params *params)
{
	if (params == NULL)
		return;
	params->bus_devices_path = RTE_PLATFORM_VFIO_DEFAULT_BUS_DEVICES_PATH;
	params->device_name = NULL;
	params->region_index = 0;
	params->debug_log = 0;
}

RTE_EXPORT_SYMBOL(rte_platform_vfio_open)
int
rte_platform_vfio_open(struct rte_platform_vfio_device *dev,
	const struct rte_platform_vfio_params *params)
{
	struct rte_platform_vfio_params cfg;
	struct vfio_device_info dev_info = { .argsz = sizeof(dev_info) };
	struct vfio_region_info reg_info = { .argsz = sizeof(reg_info) };
	void *mapped = MAP_FAILED;
	int dev_fd = -1;
	int rc;

	if (dev == NULL)
		return -EINVAL;

	/*
	 * Repeat open without close: mapped regs indicate an active handle.
	 * Do not use "dev_fd >= 0" before init: a zero-initialized struct has
	 * dev_fd==0, which is a valid fd on Linux and would falsely return
	 * -EBUSY (errno 16).
	 */
	if (dev->regs != NULL)
		return -EBUSY;

	memset(dev, 0, sizeof(*dev));
	dev->dev_fd = -1;

	resolve_params(params, &cfg);
	if (cfg.device_name == NULL) {
		fprintf(stderr, "platform_vfio: device_name is required\n");
		return -EINVAL;
	}

	reg_info.index = cfg.region_index;

	vfio_helper_log(cfg.debug_log, "opening %s from %s",
		cfg.device_name, cfg.bus_devices_path);

	rc = rte_vfio_setup_device(cfg.bus_devices_path, cfg.device_name,
		&dev_fd, &dev_info);
	if (rc) {
		fprintf(stderr, "rte_vfio_setup_device(%s) failed: %d\n",
			cfg.device_name, rc);
		return rc;
	}

	if (dev_info.num_regions <= cfg.region_index) {
		fprintf(stderr, "Device %s has only %u regions, region %u unavailable\n",
			cfg.device_name, dev_info.num_regions, cfg.region_index);
		rc = -ENODEV;
		goto err_release;
	}

	rc = ioctl(dev_fd, VFIO_DEVICE_GET_REGION_INFO, &reg_info);
	if (rc) {
		fprintf(stderr, "VFIO_DEVICE_GET_REGION_INFO failed: %s\n",
			strerror(errno));
		rc = -errno;
		goto err_release;
	}

	if (!(reg_info.flags & VFIO_REGION_INFO_FLAG_MMAP)) {
		fprintf(stderr, "Region %u does not support mmap (flags=0x%x)\n",
			cfg.region_index, reg_info.flags);
		rc = -ENOTSUP;
		goto err_release;
	}

	mapped = mmap(NULL, reg_info.size, PROT_READ | PROT_WRITE, MAP_SHARED,
		dev_fd, reg_info.offset);
	if (mapped == MAP_FAILED) {
		fprintf(stderr, "mmap region %u failed: %s\n",
			cfg.region_index, strerror(errno));
		rc = -errno;
		goto err_release;
	}

	dev->regs = (volatile void *)mapped;
	dev->regs_size = reg_info.size;
	dev->dev_fd = dev_fd;
	dev->bus_devices_path = cfg.bus_devices_path;
	dev->device_name = cfg.device_name;
	dev->region_index = cfg.region_index;
	dev->debug_log = cfg.debug_log;

	vfio_helper_log(dev->debug_log,
		"mapped region %u at %p (size=0x%zx)",
		dev->region_index, (const void *)(uintptr_t)dev->regs,
		dev->regs_size);

	return 0;

err_release:
	rte_vfio_release_device(cfg.bus_devices_path, cfg.device_name, dev_fd);
	memset(dev, 0, sizeof(*dev));
	dev->dev_fd = -1;
	return rc;
}

RTE_EXPORT_SYMBOL(rte_platform_vfio_close)
void
rte_platform_vfio_close(struct rte_platform_vfio_device *dev)
{
	if (dev == NULL)
		return;

	if (dev->regs != NULL) {
		munmap((void *)(uintptr_t)dev->regs, dev->regs_size);
		dev->regs = NULL;
		dev->regs_size = 0;
	}

	if (dev->dev_fd >= 0) {
		rte_vfio_release_device(dev->bus_devices_path,
			dev->device_name, dev->dev_fd);
		dev->dev_fd = -1;
	}

	dev->bus_devices_path = NULL;
	dev->device_name = NULL;
	dev->region_index = 0;
	dev->debug_log = 0;
}
