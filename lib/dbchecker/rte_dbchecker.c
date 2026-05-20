/*
 * User-space DBChecker for DPDK: MMIO via vfio-platform (rte_platform_vfio_helper).
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>

#include <eal_export.h>
#include <rte_platform_vfio_helper.h>
#include <rte_io.h>

#include "rte_dbchecker.h"
static uint16_t dbte_alloc_id = 0;
static uint8_t dbchecker_enable = 0;
static struct rte_platform_vfio_device dbchecker_vfio = RTE_PLATFORM_VFIO_DEVICE_INITIALIZER;
static struct dbchecker_mtdt *dbte_table;
static volatile struct dbchecker_mtdt *dbte_table_sram;

/* helpers: pread/pwrite wrappers */
/*
 * Find a UIO device by its name (the 'name' file under /sys/class/uio/uioX/name).
 * If found, write the device node path (e.g. /dev/uio3) into out_dev (len bytes)
 * and return 0. Return -1 if not found or on error.
 */

// tool functions to read/write uio mmio registers


// int dbchecker_command(struct dbchecker_cmd *cmd){
//     uint32_t hw_cmd =   ((uint32_t)(cmd->v & 0x1UL) << 31) |
//                         ((uint32_t)(cmd->op & 0x3UL) << 29) |
//                         ((uint32_t)(cmd->status & 0x1UL) << 28) |
//                         (cmd->imm & 0xFFFFFFFUL);
//     if (cmd->op == DBCHECKER_OP_ALLOC) {
//         uio_write32(DBCHECKER_MTDT_0_OFFSET, (uint32_t)(cmd->mtdt->lo_bnd & 0xFFFFFFFFUL));
//         uio_write32(DBCHECKER_MTDT_1_OFFSET, 
//             (uint32_t)(((cmd->mtdt->lo_bnd >> 32) & 0xFFFFUL) | ((cmd->mtdt->up_bnd_lo & 0xFFFFUL) << 16)));
//         uio_write32(DBCHECKER_MTDT_2_OFFSET, (uint32_t)(cmd->mtdt->up_bnd_hi & 0xFFFFFFFFUL));
//     }
//     rte_wmb();
//     uio_write32(DBCHECKER_CMD_OFFSET, hw_cmd);
//     return 0;
// }

/*
 * dbte_alloc_id layout: [ group (12 bits) | offset (4 bits) ]
 */
static inline uint16_t dbte_next_id(uint16_t id)
{
	uint16_t idx = (id + 1) & 0xFFFU;
	return idx;
}

static inline uint32_t
dbchecker_reg_read32(uint32_t off)
{
	if (unlikely(dbchecker_vfio.regs == NULL))
		return 0;
	return rte_read32((volatile void *)((uintptr_t)dbchecker_vfio.regs + off));
}

static inline void
dbchecker_reg_write32(uint32_t off, uint32_t v)
{
	if (unlikely(dbchecker_vfio.regs == NULL))
		return;
	rte_write32(v, (volatile void *)((uintptr_t)dbchecker_vfio.regs + off));
}
void dbchecker_en_set(uint32_t dev_mask){
    dbchecker_reg_write32(DBCHECKER_EN_OFFSET, dev_mask);
}

uint32_t dbchecker_en_get(void){
    return dbchecker_reg_read32(DBCHECKER_EN_OFFSET);
}


RTE_EXPORT_SYMBOL(dbchecker_alloc_mtdt)
dma_addr_t dbchecker_alloc_mtdt(dma_addr_t addr, size_t size, enum dma_data_direction dir){
    //if (!(dbchecker_en_get() & 0xFFFFFFFF))
    //    return addr; // not enabled

    // use global flag to avoid mmio
    if (!dbchecker_enable) {
        if (dbchecker_init(NULL) < 0)
            return (dma_addr_t)-1;
    }

    struct dbchecker_mtdt mtdt;
    memset(&mtdt, 0, sizeof(mtdt));
    mtdt.wr = (dir == DMA_BIDIRECTIONAL) ? DBCHECKER_RWMODE_RW :
         (dir == DMA_FROM_DEVICE) ? DBCHECKER_RWMODE_WO :
         (dir == DMA_TO_DEVICE) ? DBCHECKER_RWMODE_RO :
          DBCHECKER_RWMODE_INVALID;

    dma_addr_t alloc_addr = (dma_addr_t)-1;
    mtdt.lo_bnd = addr & 0xFFFFFFFFFFFFULL;
    mtdt.up_bnd_lo = (uint16_t)((addr + size) & 0xFFFFULL);
    mtdt.up_bnd_hi = (uint32_t)(((addr + size) >> 16) & 0xFFFFFFFFUL);

    /* Find a free slot starting at current counter. The counter encodes
     * [group:12 | offset:4] and increments group first then offset.
     * If the current slot is occupied (v != 0), walk forward until a
     * slot with v == 0 is found or we wrap back to start -> failure.
     */
    uint16_t start = dbte_alloc_id;
    uint16_t idx = start;
    bool found = false;
    do {
        if (dbte_table[idx].v == 0) {
            found = true;
            break;
        }
        idx = dbte_next_id(idx);
    } while (idx != start);
    
    if (!found) {
        printf("DBCHECKER: alloc failed, table full (start idx %u)\n", start);
        return alloc_addr; /* -1 */
    }
    mtdt.v = 1;
    mtdt.index = idx;

    /* store copy of mtdt at found index and advance allocation cursor to next position */
    // RTE_ASSERT(dbte_table[idx].v == 0);
    dbte_table[idx] = mtdt;
    // RTE_ASSERT(dbte_table[idx].v == 1);
    // RTE_ASSERT(dbte_table_sram[idx].v == 0);
    dbte_table_sram[idx] = mtdt;
    rte_io_wmb();
    // RTE_ASSERT(dbte_table_sram[idx].v == 1);
    
    alloc_addr = (addr & 0xFFFFFFFFFFFFULL) | ((uint64_t)idx << 48);

    dbte_alloc_id = dbte_next_id(idx);
    // DBCHECKER_DEBUG_LOG("DBCHECKER: alloc addr: 0x%llx, save metadata idx %zu\n",
    //     (unsigned long long)alloc_addr, idx);
    // printf("DBCHECKER: alloc addr: 0x%llx, lo_bnd 0x%llx, up_bnd_hi 0x%llx, up_bnd_lo 0x%llx\n",
    //      (unsigned long long)alloc_addr, (unsigned long long)mtdt.lo_bnd, 
    //      (unsigned long long)mtdt.up_bnd_hi, (unsigned long long)mtdt.up_bnd_lo);
    return alloc_addr;
}


RTE_EXPORT_SYMBOL(dbchecker_free_mtdt)
dma_addr_t dbchecker_free_mtdt(dma_addr_t addr){
    //if (!(dbchecker_en_get() & 0xFFFFFFFF)) 
    //    return addr; // dbchecker not enabled

    // use global flag to avoid mmio
    if (!dbchecker_enable)
        return addr; // dbchecker not enabled

    uint16_t index = (uint16_t)((addr >> 48) & 0xFFFUL);
    //printf("dbchecker_free_mtdt index %x\n", index);

    if (index >= MAX_DBTE_TABLE_SIZE) {
        printf("DBCHECKER Error: free mtdt failed, index %u out of bounds (Max %u)\n", 
           index, MAX_DBTE_TABLE_SIZE);
        return (dma_addr_t)-1; 
    }

    dbte_table[index].v = 0;
    dbte_table_sram[index].v = 0;
    rte_io_wmb();
    // assert(dbte_table[index].v == 0);
    // assert(dbte_table_sram[index].v == 0);
    return addr & 0xFFFFFFFFFFFFULL; // orig addr
}

// void dbchecker_free_all_mtdt(void){
//     //printf("dbchecker_free_all_mtdt\n");    
//     //printf("free dbte table\n");
//     //memset(dbte_table, 0, sizeof(struct dbchecker_mtdt) * MAX_DBTE_TABLE_SIZE);
//     struct dbchecker_cmd free_cmd;
//     memset(&free_cmd, 0, sizeof(free_cmd));
//     free_cmd.v = 1;
//     free_cmd.op = DBCHECKER_OP_FREE;
//     free_cmd.imm = 1 << 12; // clear all
//     dbchecker_command(&free_cmd);
//     //printf("submit clear all cmd\n");
// }


RTE_EXPORT_SYMBOL(dbchecker_err_handler)
int dbchecker_err_handler(void){
    uint32_t cnt = dbchecker_reg_read32(DBCHECKER_ERR_CNT_OFFSET);
    uint32_t info = dbchecker_reg_read32(DBCHECKER_ERR_INFO_OFFSET);
    uint32_t addr_lo = dbchecker_reg_read32(DBCHECKER_ERR_ADDR_LO_OFFSET);
    uint32_t addr_hi = dbchecker_reg_read32(DBCHECKER_ERR_ADDR_HI_OFFSET);
    uint64_t addr = ((uint64_t)addr_hi << 32) | addr_lo;
    if (cnt & ~0xF){
        fprintf(stderr, "DBCHECKER: error detected!\n");
        fprintf(stderr, "DBCHECKER: error count: 0x%llx, index: 0x%llx, addr: 0x%llx\n",
            (unsigned long long)cnt, (unsigned long long)info, (unsigned long long)addr);
        fprintf(stderr, "DBCHECKER: error mtdt: lo_bnd=0x%llx, up_bnd_lo=0x%llx, up_bnd_hi=0x%llx, v=%lx, wr=%lx\n",
            (unsigned long long)dbte_table[info].lo_bnd,
            (unsigned long long)dbte_table[info].up_bnd_lo,
            (unsigned long long)dbte_table[info].up_bnd_hi,
            (unsigned long)dbte_table[info].v,
            (unsigned long)dbte_table[info].wr);
        dbchecker_reg_write32(DBCHECKER_CLR_ERR_OFFSET, 0x1UL); // clear error
        return -1;
    }
    return 0;
}

// int dbchecker_activate_mtdt(dma_addr_t addr, enum dma_data_direction dir){
//     if (!dbchecker_enable)
//         return 0; // dbchecker not enabled

//     uint16_t index = (uint16_t)((addr >> 48) & 0xFFFFUL);
//     dbte_table[index].wr = (dir == DMA_BIDIRECTIONAL) ? DBCHECKER_RWMODE_RW :
//               (dir == DMA_FROM_DEVICE) ? DBCHECKER_RWMODE_WO :
//               (dir == DMA_TO_DEVICE) ? DBCHECKER_RWMODE_RO :
//                DBCHECKER_RWMODE_INVALID;
//     dbte_table[index].v = 1;
//     return 0;
// }

// int dbchecker_activate_mtdt_hook(struct rte_mbuf *m, enum dma_data_direction dir){
//     if (!m || uio_map == NULL)
//         return -1;
//     if (!RTE_MBUF_DIRECT(m))
//         return -1;
//     dma_addr_t base = (dma_addr_t)rte_mbuf_iova_get(m);
//     return dbchecker_activate_mtdt(base, dir);
// }

// int dbchecker_deactivate_mtdt(dma_addr_t addr){
//     if (!dbchecker_enable) 
//         return 0; // dbchecker not enabled

//     uint16_t index = (uint16_t)((addr >> 48) & 0xFFFFUL);
//     dbte_table[index].v = 0;
//     struct dbchecker_cmd free_cmd;
//     memset(&free_cmd, 0, sizeof(free_cmd));
//     free_cmd.v  = 1;
//     free_cmd.op = DBCHECKER_OP_FREE;
//     free_cmd.imm = index;
//     //printf("deactivate index %x\n", index);
//     return dbchecker_command(&free_cmd);
// }

// int dbchecker_deactivate_mtdt_hook(struct rte_mbuf *m){
//     if (!m || uio_map == NULL)
//         return -1;
//     if (!RTE_MBUF_DIRECT(m))
//         return -1;
//     dma_addr_t base = (dma_addr_t)rte_mbuf_iova_get(m);
//     return dbchecker_deactivate_mtdt(base);
// }

// static void *err_thread_fn(void *arg)
// {
//     (void)arg;
//     while (err_thread_running) {
//         dbchecker_err_handler();
//         usleep(1000 * 1000); /* 1000 ms */
//     }
//     return NULL;
// }


/* Initialize user-space DBChecker (open VFIO device) */
int dbchecker_init(const char *dev)
{
	const char *device_name = dev ? dev : "81000000.dbchecker";
	struct rte_platform_vfio_params vparams;

	if (dbchecker_vfio.regs != NULL)
		return -1;

	rte_platform_vfio_default_params(&vparams);
	vparams.device_name = device_name;

	if (rte_platform_vfio_open(&dbchecker_vfio, &vparams) != 0)
		return -1;

	dbte_table = (struct dbchecker_mtdt*)rte_zmalloc("dbte_table",
		MAX_DBTE_TABLE_SIZE * sizeof(struct dbchecker_mtdt), 0);
	if (!dbte_table) {
		printf("DBChecker: alloc dbte table failed\n");
		rte_platform_vfio_close(&dbchecker_vfio);
		return -1;
	}

	dbte_table_sram = (volatile struct dbchecker_mtdt*)((char*)dbchecker_vfio.regs + DBCHECKER_MTDT_SRAM_OFFSET);
	for (int i = 0; i < MAX_DBTE_TABLE_SIZE; i++) {
		dbte_table_sram[i] = (struct dbchecker_mtdt){0};
	}

	dbchecker_en_set(DBCHECKER_ENABLE_MASK);
	dbchecker_enable = 1;
	printf("DBCHECKER (userspace): init, mtdt size %d using %s\n",
		(int)sizeof(struct dbchecker_mtdt), device_name);
	return 0;
}

/* Cleanup user-space DBChecker */
void dbchecker_exit(void)
{
	if (dbchecker_vfio.regs != NULL) {
		dbchecker_en_set(DBCHECKER_DISABLE_MASK);
		rte_platform_vfio_close(&dbchecker_vfio);
	}
	if (dbte_table) {
		rte_free(dbte_table);
		dbte_table = NULL;
	}
	dbte_table_sram = NULL;
	dbchecker_enable = 0;
	printf("DBCHECKER (userspace): exit\n");
}

RTE_EXPORT_SYMBOL(dbchecker_module_init_hook)
int dbchecker_module_init_hook(void)
{
    /* use default device discovery behavior */
    return dbchecker_init(NULL);
}

RTE_EXPORT_SYMBOL(dbchecker_module_exit_hook)
void dbchecker_module_exit_hook(void)
{
    dbchecker_exit();
}

dma_addr_t dbchecker_alloc_mtdt_generic(dma_addr_t addr, size_t size, enum dma_data_direction dir){
    if (!dbchecker_enable) {
        if (dbchecker_init(NULL) < 0)
            return (dma_addr_t)-1;
    }

    struct dbchecker_mtdt mtdt;
    memset(&mtdt, 0, sizeof(mtdt));
    mtdt.wr = (dir == DMA_BIDIRECTIONAL) ? DBCHECKER_RWMODE_RW :
         (dir == DMA_FROM_DEVICE) ? DBCHECKER_RWMODE_WO :
         (dir == DMA_TO_DEVICE) ? DBCHECKER_RWMODE_RO :
          DBCHECKER_RWMODE_INVALID;

    dma_addr_t alloc_addr = (dma_addr_t)-1;
    mtdt.lo_bnd = addr & 0xFFFFFFFFFFFFULL;
    mtdt.up_bnd_lo = (uint16_t)((addr + size) & 0xFFFFULL);
    mtdt.up_bnd_hi = (uint32_t)(((addr + size) >> 16) & 0xFFFFFFFFUL);

    uint16_t start = dbte_alloc_id;
    uint16_t idx = start;
    bool found = false;
    do {
        if (dbte_table[idx].v == 0) {
            found = true;
            break;
        }
        idx = dbte_next_id(idx);
    } while (idx != start);

    if (!found) {
        printf("DBCHECKER: alloc failed, table full (start idx %u)\n", start);
        rte_exit( EXIT_FAILURE, "DBCHECKER: alloc failed, table full\n");
        return alloc_addr;
    }
    mtdt.v = 1;
    mtdt.index = idx;

    dbte_table[idx] = mtdt;
    dbte_table_sram[idx] = mtdt;
    rte_wmb();

    alloc_addr = (addr & 0xFFFFFFFFFFFFULL) | ((uint64_t)idx << 48);
    dbte_alloc_id = dbte_next_id(idx);
    return alloc_addr;
}

dma_addr_t dbchecker_free_mtdt_generic(dma_addr_t addr){
    if (!dbchecker_enable)
        return addr;

    uint16_t index = (uint16_t)((addr >> 48) & 0xFFFUL);

    if (index >= MAX_DBTE_TABLE_SIZE) {
        printf("DBCHECKER Error: free mtdt failed, index %u out of bounds (Max %u)\n",
           index, MAX_DBTE_TABLE_SIZE);
        return (dma_addr_t)-1;
    }

    dbte_table[index].v = 0;
    dbte_table_sram[index].v = 0;
    rte_wmb();
    return addr & 0xFFFFFFFFFFFFULL;
}

/* If this file is compiled into a library for DPDK user applications,
 * they can call dbchecker_init()/dbchecker_exit() to manage the device.
 */

/* keep legacy wrapper symbols removed in favor of consistent *_hook names */

/* keep simple buildability: an example main when compiled standalone */
#ifdef DBCHECKER_STANDALONE
int main(int argc, char **argv)
{
    const char *dev = NULL;
    if (argc > 1) dev = argv[1];
    if (dbchecker_init(dev) != 0) return 1;
    printf("Press Enter to exit...\n");
    getchar();
    dbchecker_exit();
    return 0;
}
#endif
/* end of userspace implementation */