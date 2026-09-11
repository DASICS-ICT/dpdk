/* dbchecker.h - Userspace DBChecker public definitions */
#ifndef LIB_DBCHECKER_DBCHECKER_H
#define LIB_DBCHECKER_DBCHECKER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <rte_malloc.h>

/* Minimal local replacements for kernel types used by original API */
typedef uint64_t dma_addr_t;
enum dma_data_direction {
    DMA_BIDIRECTIONAL = 0,
    DMA_FROM_DEVICE = 1,
    DMA_TO_DEVICE = 2
};

#define DBCHECKER_DEFAULT_PLATFORM_BUS_DEVICES_PATH "/sys/bus/platform/devices"
/** Default sysfs device name (adjust to match the board, e.g. 81000000.dbchecker). */
#define DBCHECKER_DEFAULT_DEVICE_NAME "81000000.dbchecker"
#define DBCHECKER_DEFAULT_REGION_INDEX 0u

#define DBCHECKER_EN_OFFSET          0x00U
#define DBCHECKER_CMD_OFFSET         0x04U
#define DBCHECKER_DBTE_MB_LO_OFFSET  0x08U
#define DBCHECKER_DBTE_MB_HI_OFFSET  0x0CU
#define DBCHECKER_ERR_ADDR_LO_OFFSET 0x10U
#define DBCHECKER_ERR_ADDR_HI_OFFSET 0x14U
#define DBCHECKER_ERR_INFO_OFFSET    0x18U
#define DBCHECKER_ERR_CNT_OFFSET     0x1CU
#define DBCHECKER_PERF_HIT_OFFSET     0x20U
#define DBCHECKER_PERF_MISS_OFFSET    0x24U
#define DBCHECKER_PERF_PENALTY_OFFSET 0x28U
#define DBCHECKER_REFILL_CFG_OFFSET    0x2CU
#define DBCHECKER_REFILL_HIST_1_OFFSET 0x40U
#define DBCHECKER_REFILL_HIST_2_OFFSET 0x44U
#define DBCHECKER_REFILL_HIST_3_OFFSET 0x48U
#define DBCHECKER_REFILL_HIST_4P_OFFSET 0x4CU
#define DBCHECKER_DIFF_LINE_WAIT_OFFSET 0x50U
#define DBCHECKER_ROB_FULL_OFFSET       0x54U
#define DBCHECKER_REFILL_BYTES_OFFSET   0x58U
#define DBCHECKER_AUTO_REL_STATUS_OFFSET 0x30U
#define DBCHECKER_AUTO_REL_PERF_OFFSET   0x34U

#define MAX_DBTE_TABLE_SIZE (1U << 16)

enum dbchecker_cmd_op {
  DBCHECKER_OP_FREE,
  DBCHECKER_OP_CLEAR,
};

enum dbchecker_rw_mode {
    DBCHECKER_RWMODE_INVALID,
    DBCHECKER_RWMODE_RO,
    DBCHECKER_RWMODE_WO,
    DBCHECKER_RWMODE_RW
};

static const enum dbchecker_rw_mode dma_to_db_map[] = {
    [DMA_BIDIRECTIONAL] = DBCHECKER_RWMODE_RW, /* Index 0 -> Value 3 */
    [DMA_FROM_DEVICE]   = DBCHECKER_RWMODE_WO, /* Index 1 -> Value 2 */
    [DMA_TO_DEVICE]     = DBCHECKER_RWMODE_RO  /* Index 2 -> Value 1 */
};

typedef union {
    struct {
        uint64_t lo_bnd    : 48;
        uint64_t up_bnd_lo : 16;

        uint64_t up_bnd_hi : 32;
        uint64_t dev_id    : 5;
        uint64_t wr        : 2;
        uint64_t v         : 1;
        uint64_t no_cache : 1;  // auto-release for determinate-length transfers
        uint64_t reserved  : 19;
        uint64_t index_off : 4;
    } __attribute__((packed));

    struct {
        uint64_t raw0;
        uint64_t raw1;
    };
} dbchecker_mtdt_u;

typedef union {
    struct {
        uint32_t imm : 30;
        uint32_t op  : 1;
        uint32_t v   : 1;
    };
    uint32_t raw;
} __attribute__((packed, aligned(4))) dbchecker_cmd_u;

#define DBCHECKER_ENABLE_MASK 0x3UL /* bypass device 31 by default */
#define DBCHECKER_DISABLE_MASK 0x0UL
#define UNTRUST_DEV_ID 0x0U

#define DBCHECKER_DEBUG 0

#define DBCHECKER_DEBUG_LOG(fmt, args...) \
        do { \
                if (DBCHECKER_DEBUG) \
                        printf(fmt, ##args); \
        } while (0)

struct dbchecker_params {
	const char *bus_devices_path;
	const char *device_name;
	uint32_t region_index;
	int debug_log;
};

/** Aggregate DBTE lookup counters maintained by the FPGA. */
struct dbchecker_perf_stats {
	uint32_t hit;
	uint32_t miss;
	uint32_t penalty_cycles;
};

struct dbchecker_refill_stats {
	uint32_t served_hist[4];
	uint32_t different_line_wait_cycles;
	uint32_t rob_full_cycles;
	uint32_t bytes;
};

void dbchecker_default_params(struct dbchecker_params *params);
int dbchecker_init_with_params(const struct dbchecker_params *params);

/* Public API */
int dbchecker_init(void);
void dbchecker_exit(void);
int dbchecker_command(uint32_t cmd);
void dbchecker_en_set(uint32_t dev_mask);
uint32_t dbchecker_en_get(void);
int dbchecker_perf_read(struct dbchecker_perf_stats *stats);
int dbchecker_perf_reset(void);
int dbchecker_refill_mode_set(bool line64);
int dbchecker_refill_mode_get(bool *line64);
int dbchecker_refill_stats_read(struct dbchecker_refill_stats *stats);
dma_addr_t dbchecker_alloc_mtdt(dma_addr_t addr, size_t size, enum dma_data_direction dir, uint16_t dev_id, bool no_cache);
dma_addr_t dbchecker_free_mtdt(dma_addr_t addr);
void dbchecker_free_all_mtdt(void);
int dbchecker_err_handler(void);
int dbchecker_module_init_hook(void);
void dbchecker_module_exit_hook(void);
dma_addr_t dbchecker_alloc_mtdt_generic(dma_addr_t addr,
    size_t size, enum dma_data_direction dir, uint16_t dev_id, bool no_cache);
dma_addr_t dbchecker_free_mtdt_generic(dma_addr_t addr);

#endif /* LIB_DBCHECKER_DBCHECKER_H */
