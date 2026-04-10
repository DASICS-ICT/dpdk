/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2026
 */

#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <rte_cdma.h>
#include <rte_common.h>
#include <rte_errno.h>
#include <rte_log.h>
#include <rte_malloc.h>
#include <rte_memory.h>

#include "cdma_poc.h"
#include "ipsec.h"

#define CDMA_POC_XFER_ALIGN 4u
#define CDMA_POC_SPY_BUF_LEN (MAX_KEY_SIZE + CDMA_POC_XFER_ALIGN)

struct cdma_poc_stats {
	uint32_t inspected;
	uint32_t matched;
	uint32_t failed;
};

static void
cdma_poc_dump_key_hex(const char *title, const uint8_t *buf, uint16_t len)
{
	uint16_t i, row, row_end;
	char line[16 * 3 + 8];
	int pos;

	if (len == 0) {
		RTE_LOG(INFO, IPSEC, "%s: (empty)\n", title);
		return;
	}

	for (row = 0; row < len; row = row_end) {
		row_end = (uint16_t)RTE_MIN((uint32_t)row + 16u, (uint32_t)len);
		pos = 0;
		for (i = row; i < row_end; i++)
			pos += snprintf(line + pos, sizeof(line) - (size_t)pos,
				"%s%02x", (i > row) ? ":" : "", buf[i]);
		RTE_LOG(INFO, IPSEC, "%s [%u..%" PRIu16 "): %s\n", title, row,
			(uint16_t)(row_end - 1u), line);
	}
}

static void
cdma_poc_log_mismatch(const uint8_t *expected, const uint8_t *actual,
	uint16_t len, uint32_t *first_bad_off)
{
	uint32_t i;

	*first_bad_off = 0;
	for (i = 0; i < len; i++) {
		if (expected[i] != actual[i]) {
			*first_bad_off = i;
			return;
		}
	}
}

static int
cdma_poc_copy_and_check(struct cdma_dev *dev, void *spy_buf, rte_iova_t spy_iova,
	const uint8_t *key, uint16_t key_len, uint32_t socket_id,
	const char *direction, uint32_t sa_idx, const char *key_name,
	struct cdma_poc_stats *stats)
{
	const uint8_t *actual;
	rte_iova_t aligned_key_iova;
	rte_iova_t key_iova;
	uint32_t key_off;
	uint32_t xfer_len;
	uint32_t first_bad_off;
	int rc;

	if (key_len == 0)
		return 0;

	stats->inspected++;
	key_iova = rte_mem_virt2iova((void *)(uintptr_t)key);
	if (key_iova == RTE_BAD_IOVA) {
		stats->failed++;
		RTE_LOG(ERR, IPSEC,
			"[CDMA PoC] Scenario 1 failed to resolve IOVA: socket=%u dir=%s sa=%u key=%s len=%" PRIu16 "\n",
			socket_id, direction, sa_idx, key_name, key_len);
		return -EFAULT;
	}

	aligned_key_iova = RTE_ALIGN_FLOOR(key_iova, CDMA_POC_XFER_ALIGN);
	key_off = (uint32_t)(key_iova - aligned_key_iova);
	xfer_len = RTE_ALIGN_CEIL(key_off + key_len, CDMA_POC_XFER_ALIGN);
	actual = (const uint8_t *)spy_buf + key_off;

	memset(spy_buf, 0, xfer_len);
	rc = cdma_copy(dev, aligned_key_iova, spy_iova, xfer_len);
	if (rc != 0) {
		stats->failed++;
		RTE_LOG(ERR, IPSEC,
			"[CDMA PoC] Scenario 1 CDMA copy failed: socket=%u dir=%s sa=%u key=%s len=%" PRIu16 " aligned_src=0x%" PRIx64 " raw_src=0x%" PRIx64 " xfer_len=%" PRIu32 " rc=%d\n",
			socket_id, direction, sa_idx, key_name, key_len,
			(uint64_t)aligned_key_iova, (uint64_t)key_iova, xfer_len, rc);
		return rc;
	}

	cdma_poc_dump_key_hex(
		"[CDMA PoC] Scenario 1 key (CPU read)", key, key_len);
	cdma_poc_dump_key_hex(
		"[CDMA PoC] Scenario 1 key (CDMA read)", actual, key_len);

	if (memcmp(key, actual, key_len) == 0) {
		stats->matched++;
		RTE_LOG(INFO, IPSEC,
			"[CDMA PoC] Scenario 1 MATCH: socket=%u dir=%s sa=%u key=%s len=%" PRIu16 " iova=0x%" PRIx64 " aligned_src=0x%" PRIx64 " off=%" PRIu32 " xfer_len=%" PRIu32 "\n",
			socket_id, direction, sa_idx, key_name, key_len,
			(uint64_t)key_iova, (uint64_t)aligned_key_iova,
			key_off, xfer_len);
		return 0;
	}

	stats->failed++;
	cdma_poc_log_mismatch(key, actual, key_len, &first_bad_off);
	RTE_LOG(ERR, IPSEC,
		"[CDMA PoC] Scenario 1 mismatch: socket=%u dir=%s sa=%u key=%s len=%" PRIu16 " first_bad_off=%" PRIu32 " expected=0x%02x actual=0x%02x\n",
		socket_id, direction, sa_idx, key_name, key_len, first_bad_off,
		key[first_bad_off], actual[first_bad_off]);
	return -EIO;
}

static int
cdma_poc_inspect_sa_ctx(struct cdma_dev *dev, void *spy_buf, rte_iova_t spy_iova,
	uint32_t socket_id, const char *direction, const struct sa_ctx *sa_ctx,
	struct cdma_poc_stats *stats)
{
	uint32_t sa_idx;
	int rc;

	if (sa_ctx == NULL)
		return 0;

	for (sa_idx = 0; sa_idx < sa_ctx->nb_sa; sa_idx++) {
		const struct ipsec_sa *sa = &sa_ctx->sa[sa_idx];

		rc = cdma_poc_copy_and_check(dev, spy_buf, spy_iova,
			sa->cipher_key, sa->cipher_key_len, socket_id,
			direction, sa_idx, "cipher", stats);
		if (rc != 0)
			return rc;

		rc = cdma_poc_copy_and_check(dev, spy_buf, spy_iova,
			sa->auth_key, sa->auth_key_len, socket_id,
			direction, sa_idx, "auth", stats);
		if (rc != 0)
			return rc;
	}

	return 0;
}

int
cdma_poc_run_gateway_case_study(struct socket_ctx *ctx, uint32_t nb_socket_ctx)
{
	struct cdma_dev dev = CDMA_DEV_INITIALIZER;
	struct cdma_params params;
	struct cdma_poc_stats stats = {0};
	void *spy_buf;
	rte_iova_t spy_iova;
	uint32_t socket_id;
	int rc;

	if (ctx == NULL)
		return -EINVAL;

	cdma_default_params(&params);
	spy_buf = rte_zmalloc("cdma_poc_gateway_spy", CDMA_POC_SPY_BUF_LEN,
		RTE_CACHE_LINE_SIZE);
	if (spy_buf == NULL)
		return -ENOMEM;

	spy_iova = rte_mem_virt2iova(spy_buf);
	if (spy_iova == RTE_BAD_IOVA) {
		rte_free(spy_buf);
		return -EFAULT;
	}

	RTE_LOG(INFO, IPSEC,
		"[CDMA PoC] Scenario 1: IPsec Gateway Case Study\n");

	rc = cdma_open(&dev, &params);
	if (rc != 0) {
		rte_free(spy_buf);
		return rc;
	}

	rc = cdma_reset(&dev);
	if (rc != 0) {
		cdma_close(&dev);
		rte_free(spy_buf);
		return rc;
	}

	for (socket_id = 0; socket_id < nb_socket_ctx; socket_id++) {
		rc = cdma_poc_inspect_sa_ctx(&dev, spy_buf, spy_iova, socket_id,
			"in", ctx[socket_id].sa_in, &stats);
		if (rc != 0)
			goto out;

		rc = cdma_poc_inspect_sa_ctx(&dev, spy_buf, spy_iova, socket_id,
			"out", ctx[socket_id].sa_out, &stats);
		if (rc != 0)
			goto out;
	}

	if (stats.inspected == 0) {
		RTE_LOG(WARNING, IPSEC,
			"[CDMA PoC] Scenario 1 found no non-empty cipher/auth keys to inspect\n");
	} else {
		RTE_LOG(INFO, IPSEC,
			"[CDMA PoC] Scenario 1 summary: inspected=%" PRIu32 " matched=%" PRIu32 " failed=%" PRIu32 "\n",
			stats.inspected, stats.matched, stats.failed);
	}

out:
	cdma_close(&dev);
	rte_free(spy_buf);
	return rc;
}
