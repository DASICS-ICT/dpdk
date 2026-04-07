/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2026
 */

#include "cdma_hw.h"
#include "rte_cdma.h"

#include <errno.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <eal_export.h>
#include <rte_io.h>
#include <rte_platform_vfio_helper.h>

#define CDMA_RESET_RETRIES 50
#define CDMA_RESET_POLL_US 1000

static inline uint32_t
cdma_reg_read(volatile void *base, uint32_t off)
{
	return rte_read32((volatile void *)((uintptr_t)base + off));
}

static inline void
cdma_reg_write(volatile void *base, uint32_t off, uint32_t val)
{
	rte_write32(val, (volatile void *)((uintptr_t)base + off));
}

static void
cdma_log(int enabled, const char *fmt, ...)
{
	va_list ap;

	if (!enabled)
		return;

	va_start(ap, fmt);
	printf("[CDMA] ");
	vprintf(fmt, ap);
	printf("\n");
	fflush(stdout);
	va_end(ap);
}

static void
cdma_resolve_params(const struct cdma_params *params, struct cdma_params *cfg)
{
	cdma_default_params(cfg);
	if (params == NULL)
		return;

	if (params->bus_devices_path != NULL)
		cfg->bus_devices_path = params->bus_devices_path;
	if (params->device_name != NULL)
		cfg->device_name = params->device_name;
	if (params->region_index != 0)
		cfg->region_index = params->region_index;
	if (params->timeout_cycles != 0)
		cfg->timeout_cycles = params->timeout_cycles;

	cfg->debug_log = params->debug_log;
}

static int
cdma_wait_idle(const struct cdma_dev *dev, uint32_t timeout_cycles)
{
	uint32_t i;

	for (i = 0; i < timeout_cycles; i++) {
		uint32_t sr = cdma_reg_read(dev->regs, XAXICDMA_SR_OFFSET);

		if (sr & XAXICDMA_SR_ERR_ALL_MASK) {
			fprintf(stderr, "CDMA error: SR=0x%08x\n", sr);
			return -EIO;
		}
		if (sr & XAXICDMA_SR_IDLE_MASK)
			return 0;
	}

	return -ETIMEDOUT;
}

RTE_EXPORT_SYMBOL(cdma_default_params)
void
cdma_default_params(struct cdma_params *params)
{
	if (params == NULL)
		return;

	params->bus_devices_path = CDMA_DEFAULT_PLATFORM_BUS_DEVICES_PATH;
	params->device_name = CDMA_DEFAULT_DEVICE_NAME;
	params->region_index = CDMA_DEFAULT_REGION_INDEX;
	params->timeout_cycles = CDMA_DEFAULT_TIMEOUT_CYCLES;
	params->debug_log = 0;
}

RTE_EXPORT_SYMBOL(cdma_open)
int
cdma_open(struct cdma_dev *dev, const struct cdma_params *params)
{
	struct cdma_params cfg;
	struct rte_platform_vfio_device vfio = RTE_PLATFORM_VFIO_DEVICE_INITIALIZER;
	struct rte_platform_vfio_params vparams;
	int rc;

	if (dev == NULL)
		return -EINVAL;

	if (dev->regs != NULL)
		return -EBUSY;

	if (dev->dev_fd >= 0 &&
	    (dev->regs_size != 0 || dev->bus_devices_path != NULL ||
	     dev->device_name != NULL))
		return -EBUSY;

	memset(dev, 0, sizeof(*dev));
	dev->dev_fd = -1;

	cdma_resolve_params(params, &cfg);

	rte_platform_vfio_default_params(&vparams);
	vparams.bus_devices_path = cfg.bus_devices_path;
	vparams.device_name = cfg.device_name;
	vparams.region_index = cfg.region_index;
	vparams.debug_log = cfg.debug_log;

	rc = rte_platform_vfio_open(&vfio, &vparams);
	if (rc)
		return rc;

	dev->regs = vfio.regs;
	dev->regs_size = vfio.regs_size;
	dev->dev_fd = vfio.dev_fd;
	dev->bus_devices_path = vfio.bus_devices_path;
	dev->device_name = vfio.device_name;
	dev->region_index = vfio.region_index;
	dev->timeout_cycles = cfg.timeout_cycles;
	dev->debug_log = cfg.debug_log;

	cdma_log(dev->debug_log,
		"mapped region %u at %p (size=0x%zx)",
		dev->region_index, (const void *)(uintptr_t)dev->regs,
		dev->regs_size);

	return 0;
}

RTE_EXPORT_SYMBOL(cdma_reset)
int
cdma_reset(struct cdma_dev *dev)
{
	int tries = CDMA_RESET_RETRIES;

	if (dev == NULL || dev->regs == NULL)
		return -EINVAL;

	cdma_log(dev->debug_log, "resetting %s", dev->device_name);
	cdma_reg_write(dev->regs, XAXICDMA_CR_OFFSET, XAXICDMA_CR_RESET_MASK);

	while (tries-- > 0) {
		uint32_t cr = cdma_reg_read(dev->regs, XAXICDMA_CR_OFFSET);

		if ((cr & XAXICDMA_CR_RESET_MASK) == 0)
			return 0;
		usleep(CDMA_RESET_POLL_US);
	}

	fprintf(stderr, "CDMA reset timed out\n");
	return -ETIMEDOUT;
}

RTE_EXPORT_SYMBOL(cdma_copy)
int
cdma_copy(struct cdma_dev *dev, uint64_t src_iova, uint64_t dst_iova,
	uint32_t len)
{
	uint32_t cr;
	int rc;

	if (dev == NULL || dev->regs == NULL || len == 0)
		return -EINVAL;

	cr = cdma_reg_read(dev->regs, XAXICDMA_CR_OFFSET);
	if (cr & XAXICDMA_CR_SGMODE_MASK) {
		fprintf(stderr, "CDMA is in SG mode, cannot do simple transfer\n");
		return -EINVAL;
	}

	cdma_log(dev->debug_log,
		"copy src=0x%016" PRIx64 " dst=0x%016" PRIx64 " len=%u",
		src_iova, dst_iova, len);

	cdma_reg_write(dev->regs, XAXICDMA_SR_OFFSET,
		XAXICDMA_SR_ERR_ALL_MASK | XAXICDMA_XR_IRQ_ALL_MASK);

	rc = cdma_wait_idle(dev, dev->timeout_cycles);
	if (rc) {
		fprintf(stderr, "CDMA engine not idle before programming: %d\n", rc);
		return rc;
	}

	cdma_reg_write(dev->regs, XAXICDMA_SRCADDR_OFFSET,
		(uint32_t)(src_iova & 0xFFFFFFFFu));
	cdma_reg_write(dev->regs, XAXICDMA_SRCADDR_MSB_OFFSET,
		(uint32_t)(src_iova >> 32));
	cdma_reg_write(dev->regs, XAXICDMA_DSTADDR_OFFSET,
		(uint32_t)(dst_iova & 0xFFFFFFFFu));
	cdma_reg_write(dev->regs, XAXICDMA_DSTADDR_MSB_OFFSET,
		(uint32_t)(dst_iova >> 32));
	cdma_reg_write(dev->regs, XAXICDMA_BTT_OFFSET, len);

	return cdma_wait_idle(dev, dev->timeout_cycles);
}

RTE_EXPORT_SYMBOL(cdma_close)
void
cdma_close(struct cdma_dev *dev)
{
	struct rte_platform_vfio_device vfio;

	if (dev == NULL)
		return;

	memset(&vfio, 0, sizeof(vfio));
	vfio.regs = dev->regs;
	vfio.regs_size = dev->regs_size;
	vfio.dev_fd = dev->dev_fd;
	vfio.bus_devices_path = dev->bus_devices_path;
	vfio.device_name = dev->device_name;
	vfio.region_index = dev->region_index;
	vfio.debug_log = dev->debug_log;

	rte_platform_vfio_close(&vfio);

	dev->regs = NULL;
	dev->regs_size = 0;
	dev->dev_fd = -1;
	dev->bus_devices_path = NULL;
	dev->device_name = NULL;
	dev->region_index = 0;
	dev->timeout_cycles = 0;
	dev->debug_log = 0;
}
