/* SPDX-License-Identifier: BSD-3-Clause
 * DBChecker m_axi_dbte latency benchmark using the FPGA perf counters.
 */

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include <rte_cdma.h>
#include <rte_common.h>
#include <rte_dbchecker.h>
#include <rte_eal.h>
#include <rte_errno.h>
#include <rte_io.h>
#include <rte_malloc.h>
#include <rte_memory.h>

#define DEFAULT_ITERATIONS       200U
#define DEFAULT_WARMUP           4U
#define DEFAULT_XFER_LEN         16U
#define DEFAULT_L2_SWEEP_BYTES   (128U * 1024U)
#define DEFAULT_COLD_SWEEP_BYTES (8U * 1024U * 1024U)
#define DEFAULT_CLOCK_MHZ        250.0
#define CACHE_LINE_BYTES         64U

#define PS_APM_DDR_PHYS          UINT64_C(0xFD0B0000)
#define PS_APM_CCI_PHYS          UINT64_C(0xFD490000)
#define PS_APM_MAP_BYTES         0x1000U
#define PS_APM_MAX_UIO           32U

#define PS_APM_MSR0_OFFSET       0x0044U
#define PS_APM_MC0_OFFSET        0x0100U
#define PS_APM_MC_STRIDE         0x0010U
#define PS_APM_CTL_OFFSET        0x0300U
#define PS_APM_CTL_RD_END_FIRST  0x00000080U
#define PS_APM_CTL_RD_START_ACCEPT 0x00000040U
#define PS_APM_CTL_RESET         0x00000002U
#define PS_APM_CTL_ENABLE        0x00000001U

#define PS_APM_METRIC_WRITE_TX   0U
#define PS_APM_METRIC_READ_TX    1U
#define PS_APM_METRIC_WRITE_BYTES 2U
#define PS_APM_METRIC_READ_BYTES 3U
#define PS_APM_METRIC_READ_LATENCY 5U
#define PS_APM_METRIC_WRITE_LATENCY 6U
#define PS_APM_METRIC_READ_MIN   14U
#define PS_APM_METRIC_READ_MAX   15U

enum cache_mode {
	CACHE_MODE_L1,
	CACHE_MODE_L2,
	CACHE_MODE_COLD,
	CACHE_MODE_ALL,
};

struct app_config {
	const char *device_name;
	enum cache_mode mode;
	uint32_t refill_bytes;
	uint32_t iterations;
	uint32_t warmup;
	uint32_t xfer_len;
	size_t l2_sweep_bytes;
	size_t cold_sweep_bytes;
	double clock_mhz;
	int verbose;
	int ps_apm;
};

struct axi_apm {
	const char *name;
	uint64_t phys_addr;
	volatile uint8_t *regs;
	uint32_t control_base;
	uint32_t saved_control;
	uint32_t saved_msr[3];
	uint32_t num_counters;
	uint32_t num_msr;
	int fd;
	int state_saved;
};

struct ps_apm {
	struct axi_apm ddr;
	struct axi_apm cci;
	int available;
};

struct ps_apm_sample {
	uint32_t cci_read_tx;
	uint32_t cci_read_bytes;
	uint32_t cci_read_latency;
	uint32_t cci_read_min;
	uint32_t cci_read_max;
	uint32_t cci_write_tx;
	uint32_t cci_write_bytes;
	uint32_t cci_write_latency;
	uint32_t ddr_read_tx[6];
	uint32_t ddr_cci_read_bytes[2];
	uint32_t ddr_cci_read_latency[2];
};

struct sweep_ctx {
	volatile uint8_t *buffer;
	size_t bytes;
	volatile uint64_t sink;
};

struct measurement_ctx {
	struct sweep_ctx sweep;
	struct ps_apm *apm;
	struct ps_apm_sample *apm_sample;
	volatile uint8_t *src;
	size_t src_bytes;
	volatile uint64_t sink;
	int apm_started;
};

struct latency_sample {
	double cycles_per_miss;
	uint32_t hit;
	uint32_t miss;
	uint32_t penalty_cycles;
	struct ps_apm_sample apm;
};

static void
sigbus_handler(int sig)
{
	const char msg[] =
		"\n[FATAL] SIGBUS while accessing CDMA/DBChecker registers\n";

	(void)sig;
	write(STDERR_FILENO, msg, sizeof(msg) - 1);
	_exit(128 + SIGBUS);
}

static const char *
mode_name(enum cache_mode mode)
{
	switch (mode) {
	case CACHE_MODE_L1:
		return "l1-hot";
	case CACHE_MODE_L2:
		return "l2-hot";
	case CACHE_MODE_COLD:
		return "cold";
	case CACHE_MODE_ALL:
		return "all";
	}
	return "unknown";
}

static int
parse_mode(const char *text, enum cache_mode *mode)
{
	if (strcmp(text, "l1") == 0 || strcmp(text, "l1-hot") == 0)
		*mode = CACHE_MODE_L1;
	else if (strcmp(text, "l2") == 0 || strcmp(text, "l2-hot") == 0)
		*mode = CACHE_MODE_L2;
	else if (strcmp(text, "cold") == 0)
		*mode = CACHE_MODE_COLD;
	else if (strcmp(text, "all") == 0)
		*mode = CACHE_MODE_ALL;
	else
		return -EINVAL;
	return 0;
}

static int
parse_u32(const char *text, uint32_t *value)
{
	char *end = NULL;
	unsigned long parsed;

	errno = 0;
	parsed = strtoul(text, &end, 0);
	if (errno != 0 || end == text || *end != '\0' || parsed > UINT32_MAX)
		return -EINVAL;
	*value = (uint32_t)parsed;
	return 0;
}

static int
parse_size(const char *text, size_t *value)
{
	char *end = NULL;
	unsigned long long parsed;
	unsigned long long scale = 1;

	errno = 0;
	parsed = strtoull(text, &end, 0);
	if (errno != 0 || end == text)
		return -EINVAL;
	if (*end == 'K' || *end == 'k') {
		scale = 1024ULL;
		end++;
	} else if (*end == 'M' || *end == 'm') {
		scale = 1024ULL * 1024ULL;
		end++;
	}
	if (*end != '\0' || parsed > SIZE_MAX / scale)
		return -EINVAL;
	*value = (size_t)(parsed * scale);
	return 0;
}

static inline uint32_t
axi_apm_read(const struct axi_apm *apm, uint32_t offset)
{
	return rte_read32((volatile void *)(apm->regs + offset));
}

static inline void
axi_apm_write(const struct axi_apm *apm, uint32_t offset, uint32_t value)
{
	rte_write32(value, (volatile void *)(apm->regs + offset));
}

static int
axi_apm_open(struct axi_apm *apm)
{
	char path[128];
	char dev_path[32];
	uint64_t addr;
	unsigned int i;
	FILE *fp;
	void *mapping;

	apm->fd = -1;
	apm->regs = NULL;
	for (i = 0; i < PS_APM_MAX_UIO; i++) {
		snprintf(path, sizeof(path),
			"/sys/class/uio/uio%u/maps/map0/addr", i);
		fp = fopen(path, "r");
		if (fp == NULL)
			continue;
		if (fscanf(fp, "0x%" SCNx64, &addr) != 1)
			addr = UINT64_MAX;
		fclose(fp);
		if (addr != apm->phys_addr)
			continue;

		snprintf(dev_path, sizeof(dev_path), "/dev/uio%u", i);
		apm->fd = open(dev_path, O_RDWR | O_CLOEXEC);
		if (apm->fd < 0)
			return -errno;
		mapping = mmap(NULL, PS_APM_MAP_BYTES, PROT_READ | PROT_WRITE,
			MAP_SHARED, apm->fd, 0);
		if (mapping == MAP_FAILED) {
			int rc = -errno;

			close(apm->fd);
			apm->fd = -1;
			return rc;
		}
		apm->regs = mapping;
		return 0;
	}
	return -ENODEV;
}

static void
axi_apm_close(struct axi_apm *apm)
{
	uint32_t i;

	if (apm->regs != NULL && apm->state_saved) {
		/* Leave selector/control ownership as it was before this process. */
		axi_apm_write(apm, PS_APM_CTL_OFFSET,
			apm->saved_control & ~PS_APM_CTL_ENABLE);
		for (i = 0; i < apm->num_msr; i++)
			axi_apm_write(apm, PS_APM_MSR0_OFFSET + i * 4U,
				apm->saved_msr[i]);
		axi_apm_write(apm, PS_APM_CTL_OFFSET, apm->saved_control);
	}
	if (apm->regs != NULL)
		munmap((void *)(uintptr_t)apm->regs, PS_APM_MAP_BYTES);
	if (apm->fd >= 0)
		close(apm->fd);
	apm->regs = NULL;
	apm->fd = -1;
}

static void
axi_apm_set_metric(const struct axi_apm *apm, uint32_t counter,
	uint32_t slot, uint32_t metric)
{
	uint32_t offset = PS_APM_MSR0_OFFSET + (counter / 4U) * 4U;
	uint32_t shift = (counter % 4U) * 8U;
	uint32_t selector = ((slot & 0x7U) << 5) | (metric & 0x1FU);
	uint32_t value = axi_apm_read(apm, offset);

	value &= ~(UINT32_C(0xFF) << shift);
	value |= selector << shift;
	axi_apm_write(apm, offset, value);
}

static uint32_t
axi_apm_counter(const struct axi_apm *apm, uint32_t counter)
{
	return axi_apm_read(apm,
		PS_APM_MC0_OFFSET + counter * PS_APM_MC_STRIDE);
}

static void
axi_apm_reset(const struct axi_apm *apm)
{
	axi_apm_write(apm, PS_APM_CTL_OFFSET,
		apm->control_base | PS_APM_CTL_RESET);
	axi_apm_write(apm, PS_APM_CTL_OFFSET, apm->control_base);
}

static void
axi_apm_enable(const struct axi_apm *apm)
{
	axi_apm_write(apm, PS_APM_CTL_OFFSET,
		apm->control_base | PS_APM_CTL_ENABLE);
}

static void
axi_apm_disable(const struct axi_apm *apm)
{
	axi_apm_write(apm, PS_APM_CTL_OFFSET, apm->control_base);
}

static int
ps_apm_open(struct ps_apm *apm)
{
	static const uint8_t cci_metrics[8] = {
		PS_APM_METRIC_READ_TX,
		PS_APM_METRIC_READ_BYTES,
		PS_APM_METRIC_READ_LATENCY,
		PS_APM_METRIC_READ_MIN,
		PS_APM_METRIC_READ_MAX,
		PS_APM_METRIC_WRITE_TX,
		PS_APM_METRIC_WRITE_BYTES,
		PS_APM_METRIC_WRITE_LATENCY,
	};
	uint32_t i;
	int rc;

	memset(apm, 0, sizeof(*apm));
	apm->ddr.name = "DDR_APM";
	apm->ddr.phys_addr = PS_APM_DDR_PHYS;
	apm->ddr.num_counters = 10;
	apm->ddr.fd = -1;
	apm->cci.name = "CCI_APM";
	apm->cci.phys_addr = PS_APM_CCI_PHYS;
	apm->cci.num_counters = 8;
	apm->cci.fd = -1;

	rc = axi_apm_open(&apm->ddr);
	if (rc)
		return rc;
	rc = axi_apm_open(&apm->cci);
	if (rc) {
		axi_apm_close(&apm->ddr);
		return rc;
	}
	apm->ddr.num_msr = (apm->ddr.num_counters + 3U) / 4U;
	apm->cci.num_msr = (apm->cci.num_counters + 3U) / 4U;
	apm->ddr.saved_control = axi_apm_read(&apm->ddr,
		PS_APM_CTL_OFFSET);
	apm->cci.saved_control = axi_apm_read(&apm->cci,
		PS_APM_CTL_OFFSET);
	for (i = 0; i < apm->ddr.num_msr; i++)
		apm->ddr.saved_msr[i] = axi_apm_read(&apm->ddr,
			PS_APM_MSR0_OFFSET + i * 4U);
	for (i = 0; i < apm->cci.num_msr; i++)
		apm->cci.saved_msr[i] = axi_apm_read(&apm->cci,
			PS_APM_MSR0_OFFSET + i * 4U);
	apm->ddr.state_saved = 1;
	apm->cci.state_saved = 1;

	/* Count AR acceptance through the first R beat; DBTE reads one beat. */
	apm->ddr.control_base = apm->ddr.saved_control &
		~(PS_APM_CTL_ENABLE | PS_APM_CTL_RESET);
	apm->cci.control_base = apm->cci.saved_control &
		~(PS_APM_CTL_ENABLE | PS_APM_CTL_RESET);
	apm->ddr.control_base |= PS_APM_CTL_RD_START_ACCEPT |
		PS_APM_CTL_RD_END_FIRST;
	apm->cci.control_base |= PS_APM_CTL_RD_START_ACCEPT |
		PS_APM_CTL_RD_END_FIRST;
	axi_apm_disable(&apm->ddr);
	axi_apm_disable(&apm->cci);

	for (i = 0; i < RTE_DIM(cci_metrics); i++)
		axi_apm_set_metric(&apm->cci, i, 0, cci_metrics[i]);
	for (i = 0; i < 6; i++)
		axi_apm_set_metric(&apm->ddr, i, i,
			PS_APM_METRIC_READ_TX);
	axi_apm_set_metric(&apm->ddr, 6, 1, PS_APM_METRIC_READ_BYTES);
	axi_apm_set_metric(&apm->ddr, 7, 2, PS_APM_METRIC_READ_BYTES);
	axi_apm_set_metric(&apm->ddr, 8, 1, PS_APM_METRIC_READ_LATENCY);
	axi_apm_set_metric(&apm->ddr, 9, 2, PS_APM_METRIC_READ_LATENCY);

	axi_apm_reset(&apm->ddr);
	axi_apm_reset(&apm->cci);
	apm->available = 1;
	return 0;
}

static void
ps_apm_close(struct ps_apm *apm)
{
	if (apm->available) {
		axi_apm_disable(&apm->cci);
		axi_apm_disable(&apm->ddr);
	}
	axi_apm_close(&apm->cci);
	axi_apm_close(&apm->ddr);
	apm->available = 0;
}

static void
ps_apm_start(struct ps_apm *apm)
{
	/* Start the downstream DDR monitor before its upstream CCI monitor. */
	axi_apm_reset(&apm->ddr);
	axi_apm_reset(&apm->cci);
	axi_apm_enable(&apm->ddr);
	axi_apm_enable(&apm->cci);
	rte_io_wmb();
}

static void
ps_apm_stop(struct ps_apm *apm, struct ps_apm_sample *sample)
{
	uint32_t i;

	rte_io_rmb();
	/* Stop upstream first, then downstream, and only then read counters. */
	axi_apm_disable(&apm->cci);
	axi_apm_disable(&apm->ddr);
	rte_io_rmb();

	memset(sample, 0, sizeof(*sample));
	sample->cci_read_tx = axi_apm_counter(&apm->cci, 0);
	sample->cci_read_bytes = axi_apm_counter(&apm->cci, 1);
	sample->cci_read_latency = axi_apm_counter(&apm->cci, 2);
	sample->cci_read_min = axi_apm_counter(&apm->cci, 3);
	sample->cci_read_max = axi_apm_counter(&apm->cci, 4);
	sample->cci_write_tx = axi_apm_counter(&apm->cci, 5);
	sample->cci_write_bytes = axi_apm_counter(&apm->cci, 6);
	sample->cci_write_latency = axi_apm_counter(&apm->cci, 7);
	for (i = 0; i < 6; i++)
		sample->ddr_read_tx[i] = axi_apm_counter(&apm->ddr, i);
	sample->ddr_cci_read_bytes[0] = axi_apm_counter(&apm->ddr, 6);
	sample->ddr_cci_read_bytes[1] = axi_apm_counter(&apm->ddr, 7);
	sample->ddr_cci_read_latency[0] = axi_apm_counter(&apm->ddr, 8);
	sample->ddr_cci_read_latency[1] = axi_apm_counter(&apm->ddr, 9);
}

static void
ps_apm_calibrate(struct ps_apm *apm, volatile void *cdma_regs)
{
	struct ps_apm_sample empty;
	struct ps_apm_sample mmio;
	volatile uint32_t sink = 0;
	uint64_t empty_ddr = 0;
	uint64_t mmio_ddr = 0;
	uint32_t i;

	ps_apm_start(apm);
	ps_apm_stop(apm, &empty);
	ps_apm_start(apm);
	for (i = 0; i < 8; i++)
		sink ^= rte_read32((volatile void *)
			((uintptr_t)cdma_regs + 0x04U));
	ps_apm_stop(apm, &mmio);
	for (i = 0; i < 6; i++) {
		empty_ddr += empty.ddr_read_tx[i];
		mmio_ddr += mmio.ddr_read_tx[i];
	}
	printf("  PS APM calibration: empty{cci_rd=%" PRIu32
	       ",ddr_rd=%" PRIu64 "} 8xCDMA-MMIO-read{cci_rd=%" PRIu32
	       ",cci_bytes=%" PRIu32 ",ddr_rd=%" PRIu64 "}\n",
		empty.cci_read_tx, empty_ddr, mmio.cci_read_tx,
		mmio.cci_read_bytes, mmio_ddr);
	if (mmio.cci_read_tx == empty.cci_read_tx + 8U &&
		mmio.cci_read_bytes == empty.cci_read_bytes + 128U) {
		printf("  CCI APM calibration identifies CPU/CDMA MMIO reads;"
		       " its per-copy count is not DBTE cache-hit telemetry.\n");
	}
	(void)sink;
}

static void
usage(const char *prog)
{
	printf("Usage: %s [EAL options] -- [options]\n", prog);
	printf("  -d, --device NAME       CDMA platform device (default: %s)\n",
		CDMA_DEFAULT_DEVICE_NAME);
	printf("  -m, --mode MODE         l1, l2, cold, or all (default: all)\n");
	printf("  -n, --iterations N      measured copies per mode (default: %u)\n",
		DEFAULT_ITERATIONS);
	printf("  -w, --warmup N          unmeasured copies before tests (default: %u)\n",
		DEFAULT_WARMUP);
	printf("  -l, --length BYTES      CDMA transfer length (default: %u)\n",
		DEFAULT_XFER_LEN);
	printf("      --l2-sweep SIZE     L1 eviction sweep, K/M suffix allowed (default: 128K)\n");
	printf("      --cold-sweep SIZE   L2 eviction sweep, K/M suffix allowed (default: 8M)\n");
	printf("      --clock-mhz MHZ     DBChecker clock (default: %.0f)\n",
		DEFAULT_CLOCK_MHZ);
	printf("      --refill-bytes N    DBTE refill size: 16 or 64 (default: 64)\n");
	printf("      --no-ps-apm         disable CCI/DDR PS AXI monitor telemetry\n");
	printf("  -v, --verbose           print every counter sample\n");
}

static int
sweep_cache(void *arg)
{
	struct sweep_ctx *ctx = arg;
	uint64_t sum = ctx->sink;
	size_t i;

	for (i = 0; i < ctx->bytes; i += CACHE_LINE_BYTES)
		sum += ctx->buffer[i];
	ctx->sink = sum;

	/* Order CPU metadata stores and cache traffic before the CDMA MMIO start. */
	rte_io_wmb();
	return 0;
}

static int
measurement_pre_submit(void *arg)
{
	struct measurement_ctx *ctx = arg;
	uint64_t sum = ctx->sink;
	size_t i;
	int rc;

	if (ctx->sweep.bytes != 0) {
		rc = sweep_cache(&ctx->sweep);
		if (rc)
			return rc;
	}

	/* Keep the CDMA data read cache-hot so downstream reads are DBTE-led. */
	for (i = 0; i < ctx->src_bytes; i += CACHE_LINE_BYTES)
		sum += ctx->src[i];
	ctx->sink = sum;
	rte_io_wmb();

	if (ctx->apm != NULL && ctx->apm_sample != NULL &&
		ctx->apm->available) {
		ps_apm_start(ctx->apm);
		ctx->apm_started = 1;
	}
	return 0;
}

static int
measurement_post_complete(void *arg)
{
	struct measurement_ctx *ctx = arg;

	if (ctx->apm_started) {
		ps_apm_stop(ctx->apm, ctx->apm_sample);
		ctx->apm_started = 0;
	}
	return 0;
}

static int
compare_double(const void *lhs, const void *rhs)
{
	double a = *(const double *)lhs;
	double b = *(const double *)rhs;

	return (a > b) - (a < b);
}

static double
percentile(const double *sorted, uint32_t count, uint32_t percent)
{
	uint64_t index;

	if (count == 0)
		return 0.0;
	index = ((uint64_t)(count - 1) * percent + 50) / 100;
	return sorted[index];
}

static int
ensure_dbchecker(int *initialized_here)
{
	struct dbchecker_perf_stats stats;
	int rc;

	*initialized_here = 0;
	rc = dbchecker_perf_read(&stats);
	if (rc == 0)
		return 0;
	if (rc != -ENODEV)
		return rc;

	rc = dbchecker_init();
	if (rc == 0)
		*initialized_here = 1;
	return rc;
}

static void
dump_dbchecker_diagnostics(const char *where)
{
	struct dbchecker_perf_stats perf = { 0 };
	struct dbchecker_refill_stats refill = { 0 };
	bool line64 = false;

	if (dbchecker_perf_read(&perf) == 0 &&
		dbchecker_refill_stats_read(&refill) == 0 &&
		dbchecker_refill_mode_get(&line64) == 0) {
		fprintf(stderr,
			"DBChecker diagnostics (%s): mode=%uB hit=%" PRIu32
			" miss=%" PRIu32 " penalty=%" PRIu32
			" refill_bytes=%" PRIu32 " hist=[%u,%u,%u,%u]"
			" diff_wait=%" PRIu32 " rob_full=%" PRIu32 "\n",
			where, line64 ? 64U : 16U, perf.hit, perf.miss,
			perf.penalty_cycles, refill.bytes,
			refill.served_hist[0], refill.served_hist[1],
			refill.served_hist[2], refill.served_hist[3],
			refill.different_line_wait_cycles,
			refill.rob_full_cycles);
	}
	dbchecker_err_handler();
}

static int
run_copy(struct cdma_dev *dev, void *src, void *dst, rte_iova_t src_iova,
	rte_iova_t dst_iova, uint32_t len, uint32_t sequence,
	struct sweep_ctx *sweep, struct ps_apm *apm,
	struct latency_sample *sample)
{
	struct dbchecker_perf_stats perf;
	struct measurement_ctx measurement = {
		.sweep = {
			.buffer = sweep == NULL ? NULL : sweep->buffer,
			.bytes = sweep == NULL ? 0 : sweep->bytes,
			.sink = sweep == NULL ? 0 : sweep->sink,
		},
		.apm = apm,
		.apm_sample = sample == NULL ? NULL : &sample->apm,
		.src = src,
		.src_bytes = len,
		.sink = 0,
		.apm_started = 0,
	};
	uint8_t *src_bytes = src;
	uint32_t i;
	int rc;

	for (i = 0; i < len; i++)
		src_bytes[i] = (uint8_t)(sequence + i * 17U);
	memset(dst, 0, len);
	rte_io_wmb();

	rc = dbchecker_perf_reset();
	if (rc)
		return rc;
	if (sample != NULL)
		memset(&sample->apm, 0, sizeof(sample->apm));

	rc = cdma_copy_with_hooks(dev, (uint64_t)src_iova,
		(uint64_t)dst_iova, len, measurement_pre_submit, &measurement,
		measurement_post_complete, &measurement);
	if (sweep != NULL)
		sweep->sink = measurement.sweep.sink;
	if (rc) {
		dump_dbchecker_diagnostics("CDMA failure");
		return rc;
	}

	rc = dbchecker_perf_read(&perf);
	if (rc)
		return rc;
	if (memcmp(src, dst, len) != 0) {
		fprintf(stderr,
			"Data mismatch: src[0..3]=%02x %02x %02x %02x"
			" dst[0..3]=%02x %02x %02x %02x\n",
			((uint8_t *)src)[0], ((uint8_t *)src)[1],
			((uint8_t *)src)[2], ((uint8_t *)src)[3],
			((uint8_t *)dst)[0], ((uint8_t *)dst)[1],
			((uint8_t *)dst)[2], ((uint8_t *)dst)[3]);
		dump_dbchecker_diagnostics("data mismatch");
		return -EIO;
	}

	if (sample != NULL) {
		sample->hit = perf.hit;
		sample->miss = perf.miss;
		sample->penalty_cycles = perf.penalty_cycles;
		sample->cycles_per_miss = perf.miss == 0 ? 0.0 :
			(double)perf.penalty_cycles / perf.miss;
	}
	return 0;
}

static int
run_mode(struct cdma_dev *dev, const struct app_config *cfg,
	enum cache_mode mode, void *src, void *dst, rte_iova_t src_iova,
	rte_iova_t dst_iova, volatile uint8_t *scrub, struct ps_apm *apm)
{
	struct latency_sample *samples;
	struct sweep_ctx sweep = {
		.buffer = scrub,
		.bytes = 0,
		.sink = 0,
	};
	double *cycles;
	uint64_t total_penalty = 0;
	uint64_t total_miss = 0;
	uint64_t total_hit = 0;
	uint64_t total_cci_read_tx = 0;
	uint64_t total_cci_read_bytes = 0;
	uint64_t total_cci_read_latency = 0;
	uint64_t total_ddr_read_tx[6] = { 0 };
	uint64_t total_ddr_cci_read_bytes[2] = { 0 };
	uint64_t total_ddr_cci_read_latency[2] = { 0 };
	uint32_t ddr_cci_tx_hist[4] = { 0 };
	uint32_t anomalous = 0;
	uint32_t i;
	double weighted_cycles;
	double ns_per_cycle = 1000.0 / cfg->clock_mhz;
	int rc = 0;

	if (mode == CACHE_MODE_L2)
		sweep.bytes = cfg->l2_sweep_bytes;
	else if (mode == CACHE_MODE_COLD)
		sweep.bytes = cfg->cold_sweep_bytes;

	samples = calloc(cfg->iterations, sizeof(*samples));
	cycles = calloc(cfg->iterations, sizeof(*cycles));
	if (samples == NULL || cycles == NULL) {
		rc = -ENOMEM;
		goto out;
	}

	for (i = 0; i < cfg->iterations; i++) {
		rc = run_copy(dev, src, dst, src_iova, dst_iova,
			cfg->xfer_len, i + ((uint32_t)mode << 24),
			sweep.bytes == 0 ? NULL : &sweep, apm, &samples[i]);
		if (rc) {
			fprintf(stderr, "%s: iteration %u failed: %s (%d)\n",
				mode_name(mode), i, strerror(-rc), rc);
			goto out;
		}

		total_hit += samples[i].hit;
		total_miss += samples[i].miss;
		total_penalty += samples[i].penalty_cycles;
		if (apm != NULL && apm->available) {
			uint32_t ddr_cci_tx = samples[i].apm.ddr_read_tx[1] +
				samples[i].apm.ddr_read_tx[2];
			uint32_t slot;

			total_cci_read_tx += samples[i].apm.cci_read_tx;
			total_cci_read_bytes += samples[i].apm.cci_read_bytes;
			total_cci_read_latency += samples[i].apm.cci_read_latency;
			for (slot = 0; slot < 6; slot++)
				total_ddr_read_tx[slot] +=
					samples[i].apm.ddr_read_tx[slot];
			for (slot = 0; slot < 2; slot++) {
				total_ddr_cci_read_bytes[slot] +=
					samples[i].apm.ddr_cci_read_bytes[slot];
				total_ddr_cci_read_latency[slot] +=
					samples[i].apm.ddr_cci_read_latency[slot];
			}
			if (ddr_cci_tx <= 2)
				ddr_cci_tx_hist[0]++;
			else if (ddr_cci_tx == 3)
				ddr_cci_tx_hist[1]++;
			else if (ddr_cci_tx == 4)
				ddr_cci_tx_hist[2]++;
			else
				ddr_cci_tx_hist[3]++;
		}
		cycles[i] = samples[i].cycles_per_miss;
		if (samples[i].miss != 2 || samples[i].hit != 0)
			anomalous++;
		if (cfg->verbose) {
			printf("  %s[%u]: hit=%" PRIu32 " miss=%" PRIu32
			       " penalty=%" PRIu32 " cycles/miss=%.2f",
				mode_name(mode), i, samples[i].hit, samples[i].miss,
				samples[i].penalty_cycles,
				samples[i].cycles_per_miss);
			if (apm != NULL && apm->available) {
				printf(" cci_rd=%" PRIu32
				       " ddr_rd=[%" PRIu32 ",%" PRIu32
				       ",%" PRIu32 ",%" PRIu32 ",%" PRIu32
				       ",%" PRIu32 "]",
					samples[i].apm.cci_read_tx,
					samples[i].apm.ddr_read_tx[0],
					samples[i].apm.ddr_read_tx[1],
					samples[i].apm.ddr_read_tx[2],
					samples[i].apm.ddr_read_tx[3],
					samples[i].apm.ddr_read_tx[4],
					samples[i].apm.ddr_read_tx[5]);
			}
			printf("\n");
		}
	}

	qsort(cycles, cfg->iterations, sizeof(*cycles), compare_double);
	weighted_cycles = total_miss == 0 ? 0.0 :
		(double)total_penalty / total_miss;

	printf("\n[%s]\n", mode_name(mode));
	printf("  samples=%u anomalous=%u total_hit=%" PRIu64
	       " total_miss=%" PRIu64 " total_penalty=%" PRIu64 "\n",
		cfg->iterations, anomalous, total_hit, total_miss, total_penalty);
	printf("  weighted mean: %.2f cycles/miss = %.2f ns\n",
		weighted_cycles, weighted_cycles * ns_per_cycle);
	printf("  per-copy mean distribution (two DBTE refills/copy):\n");
	printf("    min=%6.2f ns  p50=%6.2f ns  p95=%6.2f ns  p99=%6.2f ns  max=%6.2f ns\n",
		cycles[0] * ns_per_cycle,
		percentile(cycles, cfg->iterations, 50) * ns_per_cycle,
		percentile(cycles, cfg->iterations, 95) * ns_per_cycle,
		percentile(cycles, cfg->iterations, 99) * ns_per_cycle,
		cycles[cfg->iterations - 1] * ns_per_cycle);
	if (weighted_cycles >= 1.0) {
		printf("  approximate AR->R when AR has no wait: %.2f ns"
		       " (penalty minus one WB cycle)\n",
			(weighted_cycles - 1.0) * ns_per_cycle);
	}
	if (anomalous != 0) {
		printf("  WARNING: expected hit=0 and miss=2 per copy;"
		       " other DBChecker traffic may be present.\n");
	}
	if (apm != NULL && apm->available) {
		uint64_t ddr_cci_tx = total_ddr_read_tx[1] +
			total_ddr_read_tx[2];

		printf("  PS AXI monitors (post-conditioning -> CDMA complete):\n");
		printf("    CCI APM slot 0: read_tx=%" PRIu64 " (%.3f/copy)"
		       " read_bytes=%" PRIu64,
			total_cci_read_tx,
			(double)total_cci_read_tx / cfg->iterations,
			total_cci_read_bytes);
		if (total_cci_read_tx != 0)
			printf(" avg_read_latency=%.2f APM cycles",
				(double)total_cci_read_latency / total_cci_read_tx);
		printf("\n");
		printf("    DDR read_tx slots[0..5]=[%" PRIu64 ",%" PRIu64
		       ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64
		       "]\n",
			total_ddr_read_tx[0], total_ddr_read_tx[1],
			total_ddr_read_tx[2], total_ddr_read_tx[3],
			total_ddr_read_tx[4], total_ddr_read_tx[5]);
		printf("    DDR CCI ports (slots 1+2): read_tx=%" PRIu64
		       " (%.3f/copy) windows{<=2=%u,3=%u,4=%u,>=5=%u}\n",
			ddr_cci_tx, (double)ddr_cci_tx / cfg->iterations,
			ddr_cci_tx_hist[0], ddr_cci_tx_hist[1],
			ddr_cci_tx_hist[2], ddr_cci_tx_hist[3]);
		printf("    DDR slots 1/2: read_bytes=[%" PRIu64 ",%" PRIu64
		       "] total_read_latency=[%" PRIu64 ",%" PRIu64
		       "] APM cycles\n",
			total_ddr_cci_read_bytes[0],
			total_ddr_cci_read_bytes[1],
			total_ddr_cci_read_latency[0],
			total_ddr_cci_read_latency[1]);
	}

out:
	free(cycles);
	free(samples);
	return rc;
}

int
main(int argc, char **argv)
{
	static const struct option long_options[] = {
		{ "device", required_argument, NULL, 'd' },
		{ "mode", required_argument, NULL, 'm' },
		{ "iterations", required_argument, NULL, 'n' },
		{ "warmup", required_argument, NULL, 'w' },
		{ "length", required_argument, NULL, 'l' },
		{ "l2-sweep", required_argument, NULL, 1000 },
		{ "cold-sweep", required_argument, NULL, 1001 },
		{ "clock-mhz", required_argument, NULL, 1002 },
		{ "no-ps-apm", no_argument, NULL, 1003 },
		{ "refill-bytes", required_argument, NULL, 1004 },
		{ "verbose", no_argument, NULL, 'v' },
		{ "help", no_argument, NULL, 'h' },
		{ NULL, 0, NULL, 0 },
	};
	struct app_config cfg = {
		.device_name = CDMA_DEFAULT_DEVICE_NAME,
		.mode = CACHE_MODE_ALL,
		.refill_bytes = 64,
		.iterations = DEFAULT_ITERATIONS,
		.warmup = DEFAULT_WARMUP,
		.xfer_len = DEFAULT_XFER_LEN,
		.l2_sweep_bytes = DEFAULT_L2_SWEEP_BYTES,
		.cold_sweep_bytes = DEFAULT_COLD_SWEEP_BYTES,
		.clock_mhz = DEFAULT_CLOCK_MHZ,
		.verbose = 0,
		.ps_apm = 1,
	};
	struct cdma_dev dev = CDMA_DEV_INITIALIZER;
	struct cdma_params cdma_params;
	struct ps_apm ps_apm;
	struct ps_apm *apm = NULL;
	volatile uint8_t *scrub = NULL;
	rte_iova_t src_iova;
	rte_iova_t dst_iova;
	void *src = NULL;
	void *dst = NULL;
	size_t scrub_bytes;
	size_t scrub_off;
	uint32_t i;
	int dbchecker_initialized_here = 0;
	int eal_rc;
	int rc = 0;
	int opt;

	signal(SIGBUS, sigbus_handler);

	eal_rc = rte_eal_init(argc, argv);
	if (eal_rc < 0)
		rte_exit(EXIT_FAILURE, "rte_eal_init failed: %s\n",
			rte_strerror(rte_errno));
	argc -= eal_rc;
	argv += eal_rc;

	while ((opt = getopt_long(argc, argv, "d:m:n:w:l:vh",
			long_options, NULL)) != -1) {
		switch (opt) {
		case 'd':
			cfg.device_name = optarg;
			break;
		case 'm':
			if (parse_mode(optarg, &cfg.mode) != 0) {
				fprintf(stderr, "Invalid mode: %s\n", optarg);
				rc = -EINVAL;
				goto out_eal;
			}
			break;
		case 'n':
			if (parse_u32(optarg, &cfg.iterations) != 0 ||
				cfg.iterations == 0) {
				fprintf(stderr, "Invalid iteration count: %s\n", optarg);
				rc = -EINVAL;
				goto out_eal;
			}
			break;
		case 'w':
			if (parse_u32(optarg, &cfg.warmup) != 0) {
				fprintf(stderr, "Invalid warmup count: %s\n", optarg);
				rc = -EINVAL;
				goto out_eal;
			}
			break;
		case 'l':
			if (parse_u32(optarg, &cfg.xfer_len) != 0 ||
				cfg.xfer_len == 0) {
				fprintf(stderr, "Invalid transfer length: %s\n", optarg);
				rc = -EINVAL;
				goto out_eal;
			}
			break;
		case 1000:
			if (parse_size(optarg, &cfg.l2_sweep_bytes) != 0) {
				fprintf(stderr, "Invalid L2 sweep size: %s\n", optarg);
				rc = -EINVAL;
				goto out_eal;
			}
			break;
		case 1001:
			if (parse_size(optarg, &cfg.cold_sweep_bytes) != 0) {
				fprintf(stderr, "Invalid cold sweep size: %s\n", optarg);
				rc = -EINVAL;
				goto out_eal;
			}
			break;
		case 1002: {
			char *end = NULL;

			cfg.clock_mhz = strtod(optarg, &end);
			if (end == optarg || *end != '\0' || cfg.clock_mhz <= 0.0) {
				fprintf(stderr, "Invalid clock: %s\n", optarg);
				rc = -EINVAL;
				goto out_eal;
			}
			break;
		}
		case 1003:
			cfg.ps_apm = 0;
			break;
		case 1004:
			if (parse_u32(optarg, &cfg.refill_bytes) != 0 ||
				(cfg.refill_bytes != 16 && cfg.refill_bytes != 64)) {
				fprintf(stderr, "Invalid refill size: %s\n", optarg);
				rc = -EINVAL;
				goto out_eal;
			}
			break;
		case 'v':
			cfg.verbose = 1;
			break;
		case 'h':
			usage(argv[0]);
			goto out_eal;
		default:
			usage(argv[0]);
			rc = -EINVAL;
			goto out_eal;
		}
	}

	cdma_default_params(&cdma_params);
	cdma_params.device_name = cfg.device_name;
	rc = cdma_open(&dev, &cdma_params);
	if (rc) {
		fprintf(stderr, "Failed to open CDMA device %s: %s (%d)\n",
			cfg.device_name, strerror(-rc), rc);
		goto out_eal;
	}
	rc = cdma_reset(&dev);
	if (rc) {
		fprintf(stderr, "CDMA reset failed: %s (%d)\n", strerror(-rc), rc);
		goto out_cdma;
	}

	rc = ensure_dbchecker(&dbchecker_initialized_here);
	if (rc) {
		fprintf(stderr, "Failed to initialize DBChecker: %s (%d)\n",
			strerror(-rc), rc);
		goto out_cdma;
	}
	dbchecker_en_set(DBCHECKER_DISABLE_MASK);
	rc = dbchecker_refill_mode_set(cfg.refill_bytes == 64);
	if (rc) {
		fprintf(stderr, "Failed to select %uB refill: %s (%d)\n",
			cfg.refill_bytes, strerror(-rc), rc);
		goto out_buffers;
	}
	dbchecker_en_set(DBCHECKER_ENABLE_MASK);
	if (cfg.ps_apm) {
		rc = ps_apm_open(&ps_apm);
		if (rc) {
			fprintf(stderr,
				"WARNING: PS CCI/DDR APM unavailable: %s (%d);"
				" continuing without PS telemetry\n",
				strerror(-rc), rc);
			rc = 0;
		} else {
			apm = &ps_apm;
		}
	}

	src = rte_malloc("dbte_latency_src", cfg.xfer_len,
		RTE_CACHE_LINE_SIZE);
	dst = rte_malloc("dbte_latency_dst", cfg.xfer_len,
		RTE_CACHE_LINE_SIZE);
	scrub_bytes = RTE_MAX(cfg.l2_sweep_bytes, cfg.cold_sweep_bytes);
	if (scrub_bytes != 0)
		scrub = rte_malloc("dbte_latency_scrub", scrub_bytes,
			RTE_CACHE_LINE_SIZE);
	if (src == NULL || dst == NULL || (scrub_bytes != 0 && scrub == NULL)) {
		fprintf(stderr, "Failed to allocate benchmark buffers\n");
		rc = -ENOMEM;
		goto out_buffers;
	}

	src_iova = rte_mem_virt2iova(src);
	dst_iova = rte_mem_virt2iova(dst);
	if (src_iova == RTE_BAD_IOVA || dst_iova == RTE_BAD_IOVA) {
		fprintf(stderr, "Failed to resolve source/destination IOVA\n");
		rc = -EFAULT;
		goto out_buffers;
	}

	for (scrub_off = 0; scrub_off < scrub_bytes;
			scrub_off += CACHE_LINE_BYTES)
		scrub[scrub_off] = (uint8_t)(scrub_off / CACHE_LINE_BYTES);
	rte_io_wmb();

	printf("DBChecker m_axi_dbte latency benchmark\n");
	printf("  CDMA=%s src_iova=0x%" PRIx64 " dst_iova=0x%" PRIx64 "\n",
		cfg.device_name, (uint64_t)src_iova, (uint64_t)dst_iova);
	printf("  mode=%s refill=%uB iterations=%u warmup=%u length=%u clock=%.3f MHz\n",
		mode_name(cfg.mode), cfg.refill_bytes, cfg.iterations, cfg.warmup,
		cfg.xfer_len, cfg.clock_mhz);
	printf("  l2_sweep=%zu bytes cold_sweep=%zu bytes\n",
		cfg.l2_sweep_bytes, cfg.cold_sweep_bytes);
	printf("  Counter interval: DBTE miss request -> refill response;"
	       " one copy should cause two no-cache DBTE misses.\n");
	if (apm != NULL) {
		printf("  PS APM enabled: DDR_APM=0x%" PRIx64
		       " CCI_APM=0x%" PRIx64 "\n",
			apm->ddr.phys_addr, apm->cci.phys_addr);
		printf("  PS APM window excludes cache sweep and metadata cleanup;"
		       " CDMA source is re-touched before counting.\n");
		ps_apm_calibrate(apm, dev.regs);
	}

	for (i = 0; i < cfg.warmup; i++) {
		rc = run_copy(&dev, src, dst, src_iova, dst_iova,
			cfg.xfer_len, 0x80000000U + i, NULL, NULL, NULL);
		if (rc) {
			fprintf(stderr, "Warmup %u failed: %s (%d)\n",
				i, strerror(-rc), rc);
			goto out_buffers;
		}
	}

	if (cfg.mode == CACHE_MODE_ALL || cfg.mode == CACHE_MODE_L1) {
		rc = run_mode(&dev, &cfg, CACHE_MODE_L1, src, dst,
			src_iova, dst_iova, scrub, apm);
		if (rc)
			goto out_buffers;
	}
	if (cfg.mode == CACHE_MODE_ALL || cfg.mode == CACHE_MODE_L2) {
		rc = run_mode(&dev, &cfg, CACHE_MODE_L2, src, dst,
			src_iova, dst_iova, scrub, apm);
		if (rc)
			goto out_buffers;
	}
	if (cfg.mode == CACHE_MODE_ALL || cfg.mode == CACHE_MODE_COLD) {
		rc = run_mode(&dev, &cfg, CACHE_MODE_COLD, src, dst,
			src_iova, dst_iova, scrub, apm);
		if (rc)
			goto out_buffers;
	}

out_buffers:
	rte_free((void *)(uintptr_t)scrub);
	rte_free(dst);
	rte_free(src);
	if (apm != NULL)
		ps_apm_close(apm);
	if (dbchecker_initialized_here)
		dbchecker_exit();
out_cdma:
	cdma_close(&dev);
out_eal:
	rte_eal_cleanup();
	return rc == 0 ? 0 : 1;
}
