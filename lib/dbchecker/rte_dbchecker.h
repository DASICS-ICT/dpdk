/* dbchecker.h - Userspace DBChecker public definitions */
#ifndef LIB_DBCHECKER_DBCHECKER_H
#define LIB_DBCHECKER_DBCHECKER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <rte_mbuf.h>
#include <rte_memzone.h>
#include <rte_malloc.h>

/* Minimal local replacements for kernel types used by original API */
typedef uint64_t dma_addr_t;
enum dma_data_direction {
    DMA_BIDIRECTIONAL = 0,
    DMA_FROM_DEVICE = 1,
    DMA_TO_DEVICE = 2
};

#define DBCHECKER_BASE_ADDR 0x40000000ULL
#define DBCHECKER_MAP_SIZE  0x20000ULL

#define DBCHECKER_EN_OFFSET          0x00U
#define DBCHECKER_ERR_ADDR_LO_OFFSET 0x04U
#define DBCHECKER_ERR_ADDR_HI_OFFSET 0x08U
#define DBCHECKER_ERR_INFO_OFFSET    0x0CU
#define DBCHECKER_ERR_CNT_OFFSET     0x10U
#define DBCHECKER_CLR_ERR_OFFSET     0x14U
#define DBCHECKER_MTDT_SRAM_OFFSET   0x20U

#define MAX_DBTE_TABLE_SIZE 4096

enum dbchecker_rw_mode {
    DBCHECKER_RWMODE_INVALID,
    DBCHECKER_RWMODE_RO,
    DBCHECKER_RWMODE_WO,
    DBCHECKER_RWMODE_RW
};

// we assume that we use little-endian system
struct dbchecker_mtdt {
    uint64_t lo_bnd    : 48;
    uint64_t up_bnd_lo : 16;
    uint64_t up_bnd_hi : 32;
    uint64_t index     : 12;
    uint64_t wr        : 2; // 00: invalid, 01: RO, 10: WO, 11: RW
    uint64_t v         : 1;
    uint64_t rsvd      : 17;
} __attribute__((packed));

#define DBCHECKER_ENABLE_MASK 0x1UL /* bypass device 31 by default */
#define DBCHECKER_DISABLE_MASK 0x0UL

#define DBCHECKER_DEBUG 0

#define DBCHECKER_DEBUG_LOG(fmt, args...) \
        do { \
                if (DBCHECKER_DEBUG) \
                        printf(fmt, ##args); \
        } while (0)

/* Public API */
int dbchecker_init(const char *dev);
void dbchecker_exit(void);
void dbchecker_en_set(uint32_t dev_mask);
uint32_t dbchecker_en_get(void);
dma_addr_t dbchecker_alloc_mtdt(dma_addr_t addr, size_t size, enum dma_data_direction dir);
dma_addr_t dbchecker_free_mtdt(dma_addr_t addr);
int dbchecker_err_handler(void);
int dbchecker_module_init_hook(void);
void dbchecker_module_exit_hook(void);
void dbchecker_alloc_mtdt_hook(struct rte_mbuf *m, enum dma_data_direction dir);
void dbchecker_free_mtdt_hook(struct rte_mbuf *m);
void dbchecker_dma_zone_alloc_hook(const struct rte_memzone *mz);
void dbchecker_dma_zone_free_hook(const struct rte_memzone *mz);

#endif /* LIB_DBCHECKER_DBCHECKER_H */
