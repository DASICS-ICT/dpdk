/* SPDX-License-Identifier: BSD-3-Clause
 * AXI CDMA demo over VFIO-platform with DPDK hugepage memory.
 *
 * Usage:
 *   dpdk-cdma_demo [EAL options] -- [-l <len>]
 *
 * The CDMA device (80800000.dma) must be bound to vfio-platform beforehand.
 */

#include <errno.h>
#include <getopt.h>
#include <inttypes.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <rte_common.h>
#include <rte_eal.h>
#include <rte_errno.h>
#include <rte_io.h>
#include <rte_malloc.h>
#include <rte_memory.h>
#include <rte_vfio.h>

/* ---------- Platform device parameters ---------- */

#define PLATFORM_BUS_DEVICES_PATH "/sys/bus/platform/devices"
#define CDMA_DEVICE_NAME          "80800000.dma"

/* ---------- AXI CDMA register layout (Xilinx xaxicdma_hw.h) ---------- */

#define XAXICDMA_CR_OFFSET          0x00  /* Control */
#define XAXICDMA_SR_OFFSET          0x04  /* Status */
#define XAXICDMA_SRCADDR_OFFSET     0x18  /* Source address low */
#define XAXICDMA_SRCADDR_MSB_OFFSET 0x1C  /* Source address high */
#define XAXICDMA_DSTADDR_OFFSET     0x20  /* Destination address low */
#define XAXICDMA_DSTADDR_MSB_OFFSET 0x24  /* Destination address high */
#define XAXICDMA_BTT_OFFSET         0x28  /* Bytes to transfer (kicks xfer) */

#define XAXICDMA_CR_RESET_MASK      0x00000004
#define XAXICDMA_CR_SGMODE_MASK     0x00000008

#define XAXICDMA_SR_IDLE_MASK       0x00000002
#define XAXICDMA_SR_ERR_ALL_MASK    0x00000770

#define XAXICDMA_XR_IRQ_ALL_MASK    0x00007000

/* ---------- Tunables ---------- */

#define DEFAULT_XFER_LEN   4096     /* bytes */
#define RESET_TIMEOUT_US   50000    /* 50 ms */
#define POLL_TIMEOUT_CYCLES 1000000

/* ---------- Debug helpers ---------- */

/* Flush after every diagnostic print so we always see the last message
 * even if the board hangs immediately afterwards. */
#define DBG(fmt, ...) do { \
	printf("[DBG] " fmt "\n", ##__VA_ARGS__); \
	fflush(stdout); \
} while (0)

static void
dump_low_bytes(const char *label, const void *buf, uint32_t len)
{
	const uint8_t *bytes = buf;
	uint32_t dump_len = RTE_MIN(len, 64u);

	printf("%s (low %uB):", label, dump_len);
	for (uint32_t i = 0; i < dump_len; i++) {
		if ((i % 16) == 0)
			printf("\n  %04x:", i);
		printf(" %02x", bytes[i]);
	}
	printf("\n");
	fflush(stdout);
}

static void
sigbus_handler(int sig)
{
	(void)sig;
	const char msg[] = "\n[FATAL] SIGBUS caught — register access caused bus error, board may hang\n";
	/* write() is async-signal-safe, printf is not */
	write(STDERR_FILENO, msg, sizeof(msg) - 1);
	_exit(128 + sig);
}

/* ---------- MMIO helpers (with barriers) ---------- */

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

/* ---------- CDMA operations ---------- */

static int
cdma_reset(volatile void *regs)
{
	int tries = 50;

	DBG("cdma_reset: writing CR_RESET...");
	cdma_reg_write(regs, XAXICDMA_CR_OFFSET, XAXICDMA_CR_RESET_MASK);
	DBG("cdma_reset: CR_RESET written, polling for clear...");

	while (tries-- > 0) {
		uint32_t cr = cdma_reg_read(regs, XAXICDMA_CR_OFFSET);
		DBG("cdma_reset: CR=0x%08x (try %d)", cr, 50 - tries);
		if ((cr & XAXICDMA_CR_RESET_MASK) == 0)
			return 0;
		usleep(1000);
	}

	fprintf(stderr, "CDMA reset timed out\n");
	return -ETIMEDOUT;
}

static int
cdma_wait_idle(volatile void *regs, int timeout_cycles)
{
	int i;

	for (i = 0; i < timeout_cycles; i++) {
		uint32_t sr = cdma_reg_read(regs, XAXICDMA_SR_OFFSET);

		if (sr & XAXICDMA_SR_ERR_ALL_MASK) {
			fprintf(stderr, "CDMA error: SR=0x%08x\n", sr);
			return -EIO;
		}
		if (sr & XAXICDMA_SR_IDLE_MASK)
			return 0;
	}

	return -ETIMEDOUT;
}

static int
cdma_simple_transfer(volatile void *regs, uint64_t src_addr, uint64_t dst_addr,
		     uint32_t length, int timeout_cycles)
{
	uint32_t cr;
	int rc;

	if (length == 0)
		return -EINVAL;

	DBG("transfer: reading CR...");
	cr = cdma_reg_read(regs, XAXICDMA_CR_OFFSET);
	DBG("transfer: CR=0x%08x", cr);
	if (cr & XAXICDMA_CR_SGMODE_MASK) {
		fprintf(stderr, "CDMA is in SG mode, cannot do simple transfer\n");
		return -EINVAL;
	}

	/* Clear sticky error/irq bits */
	DBG("transfer: clearing SR error bits...");
	cdma_reg_write(regs, XAXICDMA_SR_OFFSET,
		       XAXICDMA_SR_ERR_ALL_MASK | XAXICDMA_XR_IRQ_ALL_MASK);

	/* Wait until engine is idle before programming */
	DBG("transfer: waiting for idle before programming...");
	rc = cdma_wait_idle(regs, timeout_cycles);
	if (rc) {
		fprintf(stderr, "transfer: engine not idle before programming: %d\n", rc);
		return rc;
	}
	DBG("transfer: engine idle, programming addresses...");

	/* Program source / destination / length; writing BTT kicks the xfer */
	DBG("transfer: src=0x%016" PRIx64 " dst=0x%016" PRIx64 " len=%u",
	    src_addr, dst_addr, length);
	cdma_reg_write(regs, XAXICDMA_SRCADDR_OFFSET,
		       (uint32_t)(src_addr & 0xFFFFFFFFu));
	cdma_reg_write(regs, XAXICDMA_SRCADDR_MSB_OFFSET,
		       (uint32_t)(src_addr >> 32));
	cdma_reg_write(regs, XAXICDMA_DSTADDR_OFFSET,
		       (uint32_t)(dst_addr & 0xFFFFFFFFu));
	cdma_reg_write(regs, XAXICDMA_DSTADDR_MSB_OFFSET,
		       (uint32_t)(dst_addr >> 32));
	DBG("transfer: addresses written, writing BTT to kick transfer...");
	cdma_reg_write(regs, XAXICDMA_BTT_OFFSET, length);
	DBG("transfer: BTT written, polling for completion...");

	return cdma_wait_idle(regs, timeout_cycles);
}

/* ---------- VFIO device setup ---------- */

static int
vfio_map_cdma_regs(volatile void **regs_out, size_t *regs_size_out, int *dev_fd_out)
{
	struct vfio_device_info dev_info = { .argsz = sizeof(dev_info) };
	struct vfio_region_info reg_info = { .argsz = sizeof(reg_info), .index = 0 };
	int dev_fd = -1;
	void *mapped;
	int rc;

	DBG("vfio_setup: calling rte_vfio_setup_device...");
	rc = rte_vfio_setup_device(PLATFORM_BUS_DEVICES_PATH, CDMA_DEVICE_NAME,
				   &dev_fd, &dev_info);
	if (rc) {
		fprintf(stderr, "rte_vfio_setup_device(%s) failed: %d\n",
			CDMA_DEVICE_NAME, rc);
		return rc;
	}

	printf("VFIO device %s opened: fd=%d, num_regions=%u, num_irqs=%u\n",
	       CDMA_DEVICE_NAME, dev_fd, dev_info.num_regions, dev_info.num_irqs);
	fflush(stdout);

	if (dev_info.num_regions == 0) {
		fprintf(stderr, "Device has no MMIO regions\n");
		rc = -ENODEV;
		goto err;
	}

	rc = ioctl(dev_fd, VFIO_DEVICE_GET_REGION_INFO, &reg_info);
	if (rc) {
		fprintf(stderr, "VFIO_DEVICE_GET_REGION_INFO failed: %s\n",
			strerror(errno));
		rc = -errno;
		goto err;
	}

	printf("Region 0: size=0x%llx, offset=0x%llx, flags=0x%x\n",
	       (unsigned long long)reg_info.size,
	       (unsigned long long)reg_info.offset,
	       reg_info.flags);
	fflush(stdout);

	if (!(reg_info.flags & VFIO_REGION_INFO_FLAG_MMAP)) {
		fprintf(stderr, "Region 0 does not support mmap (flags=0x%x)\n",
			reg_info.flags);
		rc = -ENOTSUP;
		goto err;
	}

	DBG("vfio_setup: calling mmap(size=0x%llx, offset=0x%llx)...",
	    (unsigned long long)reg_info.size, (unsigned long long)reg_info.offset);
	mapped = mmap(NULL, reg_info.size, PROT_READ | PROT_WRITE, MAP_SHARED,
		      dev_fd, reg_info.offset);
	if (mapped == MAP_FAILED) {
		fprintf(stderr, "mmap region 0 failed: %s\n", strerror(errno));
		rc = -errno;
		goto err;
	}
	DBG("vfio_setup: mmap succeeded at %p", mapped);

	*regs_out = (volatile void *)mapped;
	*regs_size_out = reg_info.size;
	*dev_fd_out = dev_fd;
	return 0;

err:
	rte_vfio_release_device(PLATFORM_BUS_DEVICES_PATH, CDMA_DEVICE_NAME, dev_fd);
	return rc;
}

/* ---------- Main ---------- */

static void
usage(const char *prog)
{
	printf("Usage: %s [EAL options] -- [-l <transfer_length>]\n", prog);
}

int
main(int argc, char **argv)
{
	volatile void *regs = NULL;
	size_t regs_size = 0;
	int dev_fd = -1;
	uint32_t xfer_len = DEFAULT_XFER_LEN;
	void *src_buf = NULL, *dst_buf = NULL;
	rte_iova_t src_iova, dst_iova;
	int rc, opt;

	/* Install SIGBUS handler so we get a message instead of silent death */
	signal(SIGBUS, sigbus_handler);

	/* EAL init */
	rc = rte_eal_init(argc, argv);
	if (rc < 0)
		rte_exit(EXIT_FAILURE, "rte_eal_init failed: %s\n",
			 rte_strerror(rte_errno));
	argc -= rc;
	argv += rc;

	/* Parse app-specific options */
	while ((opt = getopt(argc, argv, "l:h")) != -1) {
		switch (opt) {
		case 'l':
			xfer_len = (uint32_t)strtoul(optarg, NULL, 0);
			break;
		case 'h':
		default:
			usage(argv[0]);
			return opt == 'h' ? 0 : 1;
		}
	}

	if (xfer_len == 0) {
		fprintf(stderr, "Transfer length must be > 0\n");
		return 1;
	}

	printf("=== AXI CDMA DPDK Demo ===\n");
	printf("Device: %s, transfer length: %u bytes\n", CDMA_DEVICE_NAME, xfer_len);
	fflush(stdout);

	/* 1. Map CDMA registers via VFIO */
	rc = vfio_map_cdma_regs(&regs, &regs_size, &dev_fd);
	if (rc) {
		fprintf(stderr, "Failed to map CDMA registers: %d\n", rc);
		goto out;
	}
	DBG("CDMA registers mapped at %p (size 0x%zx)",
	    (const void *)(uintptr_t)regs, regs_size);

	/* 2. Probe: try reading SR before touching anything else */
	DBG("Probing: reading SR register...");
	{
		uint32_t sr = cdma_reg_read(regs, XAXICDMA_SR_OFFSET);
		DBG("Probe OK: SR=0x%08x (idle=%d, err=0x%03x)",
		    sr, !!(sr & XAXICDMA_SR_IDLE_MASK),
		    (sr & XAXICDMA_SR_ERR_ALL_MASK) >> 4);
	}

	/* 3. Reset CDMA engine */
	rc = cdma_reset(regs);
	if (rc) {
		fprintf(stderr, "CDMA reset failed: %d\n", rc);
		goto out;
	}
	DBG("CDMA engine reset OK");

	/* 4. Allocate DMA buffers from hugepage memory */
	src_buf = rte_malloc("cdma_src", xfer_len, RTE_CACHE_LINE_SIZE);
	dst_buf = rte_malloc("cdma_dst", xfer_len, RTE_CACHE_LINE_SIZE);
	if (!src_buf || !dst_buf) {
		fprintf(stderr, "rte_malloc failed for DMA buffers\n");
		rc = -ENOMEM;
		goto out;
	}

	src_iova = rte_mem_virt2iova(src_buf);
	dst_iova = rte_mem_virt2iova(dst_buf);
	if (src_iova == RTE_BAD_IOVA || dst_iova == RTE_BAD_IOVA) {
		fprintf(stderr, "Failed to get IOVA for DMA buffers\n");
		rc = -EFAULT;
		goto out;
	}

	DBG("src: va=%p iova=0x%" PRIx64, src_buf, (uint64_t)src_iova);
	DBG("dst: va=%p iova=0x%" PRIx64, dst_buf, (uint64_t)dst_iova);

	/* 5. Fill source, clear destination */
	for (uint32_t i = 0; i < xfer_len; i++)
		((uint8_t *)src_buf)[i] = (uint8_t)(i & 0xFF);
	memset(dst_buf, 0, xfer_len);
	dump_low_bytes("SRC before transfer", src_buf, xfer_len);
	dump_low_bytes("DST before transfer", dst_buf, xfer_len);

	/* 6. Perform DMA transfer */
	DBG("Starting CDMA transfer...");
	rc = cdma_simple_transfer(regs, (uint64_t)src_iova, (uint64_t)dst_iova,
				  xfer_len, POLL_TIMEOUT_CYCLES);
	if (rc) {
		fprintf(stderr, "CDMA transfer failed: %d\n", rc);
		goto out;
	}
	DBG("CDMA transfer completed");
	dump_low_bytes("DST after transfer", dst_buf, xfer_len);

	/* 7. Verify */
	if (memcmp(src_buf, dst_buf, xfer_len) == 0) {
		printf("PASS: data verified OK (%u bytes)\n", xfer_len);
	} else {
		fprintf(stderr, "FAIL: data mismatch!\n");
		/* Print first few mismatches */
		int shown = 0;
		for (uint32_t i = 0; i < xfer_len && shown < 8; i++) {
			if (((uint8_t *)src_buf)[i] != ((uint8_t *)dst_buf)[i]) {
				fprintf(stderr, "  offset %u: src=0x%02x dst=0x%02x\n",
					i, ((uint8_t *)src_buf)[i],
					((uint8_t *)dst_buf)[i]);
				shown++;
			}
		}
		rc = -1;
	}

out:
	rte_free(src_buf);
	rte_free(dst_buf);
	if (regs)
		munmap((void *)(uintptr_t)regs, regs_size);
	if (dev_fd >= 0)
		rte_vfio_release_device(PLATFORM_BUS_DEVICES_PATH,
					CDMA_DEVICE_NAME, dev_fd);
	rte_eal_cleanup();
	return rc ? 1 : 0;
}
