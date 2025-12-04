/* dbchecker.h - Userspace DBChecker public definitions */
#ifndef LIB_DBCHECKER_DBCHECKER_H
#define LIB_DBCHECKER_DBCHECKER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <rte_mbuf.h>
#include <rte_memzone.h>

/* Minimal local replacements for kernel types used by original API */
typedef uint64_t dma_addr_t;
enum dma_data_direction {
    DMA_BIDIRECTIONAL = 0,
    DMA_FROM_DEVICE = 1,
    DMA_TO_DEVICE = 2
};

#define DBCHECKER_BASE_ADDR 0x40000000ULL
#define DBCHECKER_REG_SIZE 4 /* 32bit */
#define DBCHECKER_REG_NUM 10 /* 10 registers */

#define DBCHECKER_EN_OFFSET          0x00U
#define DBCHECKER_CMD_OFFSET         0x04U
#define DBCHECKER_DBTE_MB_LO_OFFSET  0x08U
#define DBCHECKER_DBTE_MB_HI_OFFSET  0x0CU
#define DBCHECKER_ERR_ADDR_LO_OFFSET 0x10U
#define DBCHECKER_ERR_ADDR_HI_OFFSET 0x14U
#define DBCHECKER_ERR_INFO_OFFSET    0x18U
#define DBCHECKER_ERR_CNT_OFFSET     0x1CU

#define MAX_DBTE_TABLE_SIZE 65535

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

struct dbchecker_mtdt {
    // --- 第一个 64位 字 (Word 0) ---
    // 0-31 位
    uint64_t index_off : 4;
    uint64_t researved : 20;
    uint64_t valid     : 1;
    uint64_t wr        : 2;
    uint64_t dev_id    : 5;
    
    // 32-63 位 (up_bnd 的低 32 位)
    uint64_t up_bnd_low : 32; 

    // --- 第二个 64位 字 (Word 1) ---
    // 64-79 位 (up_bnd 的高 16 位)
    uint64_t up_bnd_high : 16;
    
    // 80-127 位 (lo_bnd 正好 48 位)
    uint64_t lo_bnd      : 48;
} __attribute__((packed));

struct dbchecker_cmd {
  uint32_t v        : 1;  /* valid bit */
  uint32_t op       : 1;  /* 0: free, 1: clear_cnt */
  uint32_t reserved : 13;
  uint32_t clr      : 1;  /* 0: clear the specific mtdt; 1: clear all */
  uint32_t index    : 16; /* index of the mtdt to be cleaned */
} __attribute__((packed));

#define DBCHECKER_ENABLE_MASK 0x80000000UL /* bypass device 31 by default */
#define DBCHECKER_DISABLE_MASK 0x0UL
#define UNTRUST_DEV_ID 0x0U

// you do not know whether the ptr is signed if count dbte_index from 0
static uint16_t dbte_alloc_id = 1;

static uint8_t dbchecker_enable = 0;

#define DBCHECKER_DEBUG 0

#define DBCHECKER_DEBUG_LOG(fmt, args...) \
        do { \
                if (DBCHECKER_DEBUG) \
                        printf(fmt, ##args); \
        } while (0)

/* Public API */
int dbchecker_init(const char *dev);
void dbchecker_exit(void);
int dbchecker_command(struct dbchecker_cmd *cmd);
void dbchecker_en_set(uint32_t dev_mask);
uint32_t dbchecker_en_get(void);
dma_addr_t dbchecker_alloc_mtdt(dma_addr_t addr, size_t size, enum dma_data_direction dir);
dma_addr_t dbchecker_free_mtdt(dma_addr_t addr);
int dbchecker_activate_mtdt(dma_addr_t addr, enum dma_data_direction dir);
int dbchecker_deactivate_mtdt(dma_addr_t addr);
void dbchecker_free_all_mtdt(void);
int dbchecker_activate_mtdt_hook(struct rte_mbuf *m, enum dma_data_direction dir);
int dbchecker_deactivate_mtdt_hook(struct rte_mbuf *m);
int dbchecker_err_handler(void);
int dbchecker_module_init_hook(void);
void dbchecker_module_exit_hook(void);
void dbchecker_alloc_mtdt_hook(struct rte_mbuf *m);
void dbchecker_free_mtdt_hook(struct rte_mbuf *m);
void dbchecker_dma_zone_alloc_hook(const struct rte_memzone *mz);
void dbchecker_dma_zone_free_hook(const struct rte_memzone *mz);

#endif /* LIB_DBCHECKER_DBCHECKER_H */
