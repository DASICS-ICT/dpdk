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
#define DBCHECKER_REG_SIZE 8 /* 64bit */
#define DBCHECKER_REG_NUM 10 /* 10 registers */

#define DBCHECKER_EN_OFFSET         0x00U
#define DBCHECKER_CMD_OFFSET        0x08U
#define DBCHECKER_MTDT_LO_OFFSET    0x10U
#define DBCHECKER_MTDT_HI_OFFSET    0x18U
#define DBCHECKER_RES_OFFSET        0x20U
#define DBCHECKER_KEYL_OFFSET       0x28U
#define DBCHECKER_KEYH_OFFSET       0x30U
#define DBCHECKER_ERR_CNT_OFFSET    0x38U
#define DBCHECKER_ERR_INFO_OFFSET   0x40U
#define DBCHECKER_ERR_MTDT_OFFSET   0x48U

#define MAX_DBTE_TABLE_SIZE 4096

enum dbchecker_cmd_op {
  DBCHECKER_OP_FREE,
  DBCHECKER_OP_ALLOC,
  DBCHECKER_OP_CLEAR,
  DBCHECKER_OP_SWITCH
};

enum dbchecker_cmd_status {
    DBCHECKER_CMD_INVALID,
    DBCHECKER_CMD_REQUEST,
    DBCHECKER_CMD_DONE,
    DBCHECKER_CMD_ERROR
};

enum dbchecker_rw_mode {
    DBCHECKER_RWMODE_INVALID,
    DBCHECKER_RWMODE_RO,
    DBCHECKER_RWMODE_WO,
    DBCHECKER_RWMODE_RW
};

struct dbchecker_mtdt {
  uint8_t  wr     : 2; /* 0: INVALID, 1: RO, 2: WO, 3: RW */
  uint8_t  dev    : 5; /* device id */
  uint32_t id     : 25;
  uint64_t up_bnd : 48;
  uint64_t lo_bnd : 48;
};

struct dbchecker_cmd {
  uint8_t  status : 2; /* 00: inv, 01: req, 10: done, 11: err */
  uint8_t  op     : 2; /* 00: free, 01: alloc, 10: clear, 11: switch */
  uint8_t  pad    : 8;
  uint64_t imm    : 52;
  struct dbchecker_mtdt mtdt; /* only used for alloc cmd and switch cmd */
};

struct dbchecker_en_ctrl {
  uint16_t byp_dev_bm : 16;
  uint64_t padding    : 42;
  bool func_en;
  bool intr_en;
  bool intr_clr;
  bool stall_mode;
  bool err_byp;
  bool err_rpt;
};

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
void dbchecker_en_set(struct dbchecker_en_ctrl *ctrl);
uint32_t dbchecker_en_get(void);
dma_addr_t dbchecker_alloc_mtdt(dma_addr_t addr, size_t size, enum dma_data_direction dir);
dma_addr_t dbchecker_free_mtdt(dma_addr_t addr);
int dbchecker_err_handler(void);
int dbchecker_module_init_hook(void);
void dbchecker_module_exit_hook(void);
void dbchecker_alloc_mtdt_hook(struct rte_mbuf *m);
void dbchecker_free_mtdt_hook(struct rte_mbuf *m);
void dbchecker_dma_zone_alloc_hook(const struct rte_memzone *mz);
void dbchecker_dma_zone_free_hook(const struct rte_memzone *mz);

#endif /* LIB_DBCHECKER_DBCHECKER_H */
