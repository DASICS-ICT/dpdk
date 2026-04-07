/* SPDX-License-Identifier: BSD-3-Clause
 * AXI CDMA demo over vfio-platform with DPDK hugepage memory.
 *
 * Usage:
 *   dpdk-cdma_demo [EAL options] -- [-d <device>] [-l <len>] [-t <timeout>]
 *
 * The CDMA platform device must be bound to vfio-platform beforehand.
 *
 * When DPDK is built with -Denable_dbchecker=true, librte_cdma links dbchecker;
 * cdma_copy then registers src/dst with dbchecker if it is initialized and enabled.
 */

#include <getopt.h>
#include <inttypes.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <rte_common.h>
#include <rte_eal.h>
#include <rte_errno.h>
#include <rte_malloc.h>
#include <rte_memory.h>

#include <rte_cdma.h>

#define DEFAULT_XFER_LEN 4096

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
	const char msg[] =
		"\n[FATAL] SIGBUS caught - register access caused bus error, board may hang\n";

	(void)sig;
	write(STDERR_FILENO, msg, sizeof(msg) - 1);
	_exit(128 + sig);
}

static void
usage(const char *prog)
{
	printf("Usage: %s [EAL options] -- "
	       "[-d <device>] [-l <transfer_length>] [-t <timeout_cycles>]\n",
	       prog);
}

int
main(int argc, char **argv)
{
	struct cdma_dev dev = CDMA_DEV_INITIALIZER;
	struct cdma_params params;
	void *src_buf = NULL;
	void *dst_buf = NULL;
	rte_iova_t src_iova;
	rte_iova_t dst_iova;
	uint32_t xfer_len = DEFAULT_XFER_LEN;
	int rc;
	int opt;

	signal(SIGBUS, sigbus_handler);

	cdma_default_params(&params);
	params.debug_log = 1;

	rc = rte_eal_init(argc, argv);
	if (rc < 0)
		rte_exit(EXIT_FAILURE, "rte_eal_init failed: %s\n",
			 rte_strerror(rte_errno));
	argc -= rc;
	argv += rc;

	while ((opt = getopt(argc, argv, "d:l:t:h")) != -1) {
		switch (opt) {
		case 'd':
			params.device_name = optarg;
			break;
		case 'l':
			xfer_len = (uint32_t)strtoul(optarg, NULL, 0);
			break;
		case 't':
			params.timeout_cycles = (uint32_t)strtoul(optarg, NULL, 0);
			break;
		case 'h':
		default:
			usage(argv[0]);
			rte_eal_cleanup();
			return opt == 'h' ? 0 : 1;
		}
	}

	if (params.timeout_cycles == 0)
		params.timeout_cycles = CDMA_DEFAULT_TIMEOUT_CYCLES;

	if (xfer_len == 0) {
		fprintf(stderr, "Transfer length must be > 0\n");
		rte_eal_cleanup();
		return 1;
	}

	printf("=== AXI CDMA DPDK Demo ===\n");
	printf("Device: %s, transfer length: %u bytes, timeout cycles: %u\n",
		params.device_name, xfer_len, params.timeout_cycles);
	fflush(stdout);

	rc = cdma_open(&dev, &params);
	if (rc) {
		fprintf(stderr, "Failed to open CDMA device: %d\n", rc);
		goto out;
	}

	rc = cdma_reset(&dev);
	if (rc) {
		fprintf(stderr, "CDMA reset failed: %d\n", rc);
		goto out;
	}

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

	printf("src: va=%p iova=0x%" PRIx64 "\n", src_buf, (uint64_t)src_iova);
	printf("dst: va=%p iova=0x%" PRIx64 "\n", dst_buf, (uint64_t)dst_iova);
	fflush(stdout);

	for (uint32_t i = 0; i < xfer_len; i++)
		((uint8_t *)src_buf)[i] = (uint8_t)(i & 0xFF);
	memset(dst_buf, 0, xfer_len);
	dump_low_bytes("SRC before transfer", src_buf, xfer_len);
	dump_low_bytes("DST before transfer", dst_buf, xfer_len);

	rc = cdma_copy(&dev, (uint64_t)src_iova, (uint64_t)dst_iova, xfer_len);
	if (rc) {
		fprintf(stderr, "CDMA transfer failed: %d\n", rc);
		goto out;
	}

	dump_low_bytes("DST after transfer", dst_buf, xfer_len);

	if (memcmp(src_buf, dst_buf, xfer_len) == 0) {
		printf("PASS: data verified OK (%u bytes)\n", xfer_len);
	} else {
		int shown = 0;

		fprintf(stderr, "FAIL: data mismatch!\n");
		for (uint32_t i = 0; i < xfer_len && shown < 8; i++) {
			if (((uint8_t *)src_buf)[i] != ((uint8_t *)dst_buf)[i]) {
				fprintf(stderr,
					"  offset %u: src=0x%02x dst=0x%02x\n",
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
	cdma_close(&dev);
	rte_eal_cleanup();
	return rc ? 1 : 0;
}
