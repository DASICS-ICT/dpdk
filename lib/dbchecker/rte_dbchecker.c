/*
 * User-space DBChecker driver for DPDK-like environment.
 * This file replaces the previous kernel module implementation.
 * It accesses device registers via /dev/uio0 using pread/pwrite.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/uio.h>
#include <inttypes.h>
#include <dirent.h>
#include <limits.h>
#include <sys/mman.h>

/* public declarations and definitions */
#include "rte_dbchecker.h"

static uint16_t dbte_alloc_id = 0;
static uint8_t dbchecker_enable = 0;
static char uio_device[256] = "/dev/uio0";
static int uio_fd = -1;
/* mmap'ed region base and size */
static void *uio_map = NULL;
static size_t uio_map_size = 0;
static struct dbchecker_mtdt *dbte_table;

/* helpers: pread/pwrite wrappers */
/*
 * Find a UIO device by its name (the 'name' file under /sys/class/uio/uioX/name).
 * If found, write the device node path (e.g. /dev/uio3) into out_dev (len bytes)
 * and return 0. Return -1 if not found or on error.
 */
static int find_uio_device_by_name(const char *target_name, char *out_dev, size_t len)
{
    const char *sys_uio = "/sys/class/uio";
    DIR *d = opendir(sys_uio);
    if (!d) return -1;
    struct dirent *ent;
    while ((ent = readdir(d)) != NULL) {
        /* accept entries starting with "uio" (e.g. uio0, uio1, ...) */
        if (strncmp(ent->d_name, "uio", 3) != 0)
            continue;
        char name_path[256];
        snprintf(name_path, sizeof(name_path), "%s/%s/name", sys_uio, ent->d_name);
        FILE *f = fopen(name_path, "r");
        if (!f) continue;
        char buf[128];
        if (fgets(buf, sizeof(buf), f) != NULL) {
            /* strip newline */
            size_t bl = strlen(buf);
            if (bl && buf[bl-1] == '\n') buf[bl-1] = '\0';
            if (strcmp(buf, target_name) == 0) {
                /* found, compose /dev/uioX */
                snprintf(out_dev, len, "/dev/%s", ent->d_name);
                fclose(f);
                closedir(d);
                return 0;
            }
        }
        fclose(f);
    }
    closedir(d);
    return -1;
}

// tool functions to read/write uio mmio registers

static inline uint32_t uio_read32(off_t offset)
{
    if (unlikely(!uio_map)) return 0;
    volatile uint32_t *p = (volatile uint32_t *)((char *)uio_map + offset);
    uint32_t v = *p;
    return v;
}

/*
 * dbte_alloc_id layout: [ group (12 bits) | offset (4 bits) ]
 * Increment order: increment group first (0..4095), then offset (0..15).
 */
static inline uint16_t dbte_next_id(uint16_t id)
{
    // uint16_t offset = id & 0xFULL;
    // uint16_t group = id >> 4;
    // group++;
    // if (group > 0xFFF) {
    //     group = 0;
    //     offset = (offset + 1) & 0xFULL; /* wrap offset mod 16 */
    // }
    // return (uint16_t)((group << 4) | (offset & 0xF));
    uint16_t idx = (id + 1) & 0xFFF;
    return idx;
}

static inline void uio_write32(off_t offset, uint32_t v)
{
    if (unlikely(!uio_map)) return;
    volatile uint32_t *p = (volatile uint32_t *)((char *)uio_map + offset);
    *p = v;
}

/* write 64 as lo/hi 32 at offset and offset+4 */
static inline void uio_write64_lo_hi(uint64_t v, off_t offset)
{
    if (unlikely(!uio_map)) return;
    volatile uint32_t *plo = (volatile uint32_t *)((char *)uio_map + offset);
    volatile uint32_t *phi = (volatile uint32_t *)((char *)uio_map + offset + 4);
    *plo = (uint32_t)(v & 0xFFFFFFFFULL);
    *phi = (uint32_t)((v >> 32) & 0xFFFFFFFFULL);
}

/* read 64 from lo/hi 32 at offset and offset+4 */
static inline uint64_t uio_read64_lo_hi(off_t offset)
{
    if (unlikely(!uio_map)) return 0;
    volatile uint32_t *plo = (volatile uint32_t *)((char *)uio_map + offset);
    volatile uint32_t *phi = (volatile uint32_t *)((char *)uio_map + offset + 4);
    uint32_t lo = *plo;
    uint32_t hi = *phi;
    return ((uint64_t)hi << 32) | lo;
}

int dbchecker_command(struct dbchecker_cmd *cmd){
    uint32_t hw_cmd =   ((uint32_t)(cmd->v & 0x1UL) << 31) |
                        ((uint32_t)(cmd->op & 0x3UL) << 29) |
                        ((uint32_t)(cmd->status & 0x1UL) << 28) |
                        (cmd->imm & 0xFFFFFFFUL);
    if (cmd->op == DBCHECKER_OP_ALLOC) {
        uio_write32(DBCHECKER_MTDT_0_OFFSET, (uint32_t)(cmd->mtdt->lo_bnd & 0xFFFFFFFFUL));
        uio_write32(DBCHECKER_MTDT_1_OFFSET, 
            (uint32_t)(((cmd->mtdt->lo_bnd >> 32) & 0xFFFFUL) | ((cmd->mtdt->up_bnd_lo & 0xFFFFUL) << 16)));
        uio_write32(DBCHECKER_MTDT_2_OFFSET, (uint32_t)(cmd->mtdt->up_bnd_hi & 0xFFFFFFFFUL));
    }
    rte_wmb();
    uio_write32(DBCHECKER_CMD_OFFSET, hw_cmd);
    return 0;
}

void dbchecker_en_set(uint32_t dev_mask){
    uio_write32(DBCHECKER_EN_OFFSET, dev_mask);
}

uint32_t dbchecker_en_get(void){
    return uio_read32(DBCHECKER_EN_OFFSET);
}


dma_addr_t dbchecker_alloc_mtdt(dma_addr_t addr, size_t size, enum dma_data_direction dir){
    //if (!(dbchecker_en_get() & 0xFFFFFFFF))
    //    return addr; // not enabled

    // use global flag to avoid mmio
    if (!dbchecker_enable)
        return addr; // not enabled

    struct dbchecker_mtdt mtdt;
    struct dbchecker_cmd alloc_cmd;
    memset(&alloc_cmd, 0, sizeof(alloc_cmd));
    uint8_t wr;
    wr = (dir == DMA_BIDIRECTIONAL) ? DBCHECKER_RWMODE_RW :
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
    mtdt.imm = idx;
    alloc_cmd.mtdt = &mtdt;
    alloc_cmd.v = 1;
    alloc_cmd.op = DBCHECKER_OP_ALLOC;
    alloc_cmd.status = 0;
    alloc_cmd.imm = ((uint32_t)idx & 0xFFFUL) |
              ((uint32_t)(wr & 0x3UL) << 12);

    /* store copy of mtdt at found index and advance allocation cursor to next position */
    dbte_table[idx] = mtdt;
    //printf("alloc dbte_table [%d].v = %x\n", idx, dbte_table[idx].v);
    dbchecker_command(&alloc_cmd);
    /* construct returned iova with table index in high bits as previous design */
    alloc_addr = (addr & 0xFFFFFFFFFFFFULL) | ((uint64_t)idx << 48);

    dbte_alloc_id = dbte_next_id(idx);
    // DBCHECKER_DEBUG_LOG("DBCHECKER: alloc addr: 0x%llx, save metadata idx %zu\n",
    //     (unsigned long long)alloc_addr, idx);
    // printf("DBCHECKER: alloc addr: 0x%llx, lo_bnd 0x%llx, up_bnd_hi 0x%llx, up_bnd_lo 0x%llx\n",
    //      (unsigned long long)alloc_addr, (unsigned long long)mtdt.lo_bnd, 
    //      (unsigned long long)mtdt.up_bnd_hi, (unsigned long long)mtdt.up_bnd_lo);
    return alloc_addr;
}


dma_addr_t dbchecker_free_mtdt(dma_addr_t addr){
    //if (!(dbchecker_en_get() & 0xFFFFFFFF)) 
    //    return addr; // dbchecker not enabled

    // use global flag to avoid mmio
    if (!dbchecker_enable)
        return addr; // dbchecker not enabled

    uint16_t index = (uint16_t)((addr >> 48) & 0xFFFUL);
    //printf("dbchecker_free_mtdt\n");

    if (index >= MAX_DBTE_TABLE_SIZE) {
        printf("DBCHECKER Error: free mtdt failed, index %u out of bounds (Max %u)\n", 
           index, MAX_DBTE_TABLE_SIZE);
        return (dma_addr_t)-1; 
    }

    dbte_table[index].v = 0;
    //printf("free dbte_table [%d].v = %x\n", index, dbte_table[index].v);
    struct dbchecker_cmd free_cmd;
    memset(&free_cmd, 0, sizeof(free_cmd));
    free_cmd.v  = 1;
    free_cmd.op = DBCHECKER_OP_FREE;
    free_cmd.imm = index;
    //printf("DBCHECKER: free mtdt index %x\n", (uint32_t)index);
    dbchecker_command(&free_cmd);
    // DBCHECKER_DEBUG_LOG("DBCHECKER: free addr: 0x%llx\n", (unsigned long long)addr);
    
    return addr & 0xFFFFFFFFFFFFULL; // orig addr
}

void dbchecker_free_all_mtdt(void){
    //printf("dbchecker_free_all_mtdt\n");    
    //printf("free dbte table\n");
    //memset(dbte_table, 0, sizeof(struct dbchecker_mtdt) * MAX_DBTE_TABLE_SIZE);
    struct dbchecker_cmd free_cmd;
    memset(&free_cmd, 0, sizeof(free_cmd));
    free_cmd.v = 1;
    free_cmd.op = DBCHECKER_OP_FREE;
    free_cmd.imm = 1 << 12; // clear all
    dbchecker_command(&free_cmd);
    //printf("submit clear all cmd\n");
}


int dbchecker_err_handler(void){
    uint32_t cnt = uio_read32(DBCHECKER_ERR_CNT_OFFSET);
    uint32_t info = uio_read32(DBCHECKER_ERR_INFO_OFFSET);
    uint32_t addr_lo = uio_read32(DBCHECKER_ERR_ADDR_LO_OFFSET);
    uint32_t addr_hi = uio_read32(DBCHECKER_ERR_ADDR_HI_OFFSET);
    uint64_t addr = ((uint64_t)addr_hi << 32) | addr_lo;
    if (cnt & ~0xF){
        fprintf(stderr, "DBCHECKER: error detected!\n");
        fprintf(stderr, "DBCHECKER: error count: 0x%llx, index: 0x%llx, addr: 0x%llx\n",
            (unsigned long long)cnt, (unsigned long long)info, (unsigned long long)addr);
        fprintf(stderr, "DBCHECKER: error mtdt: lo_bnd=0x%llx, up_bnd_lo=0x%llx, up_bnd_hi=0x%llx, v=%lx, op=%lx, status=%lx, imm=%lx\n",
            (unsigned long long)dbte_table[info].lo_bnd,
            (unsigned long long)dbte_table[info].up_bnd_lo,
            (unsigned long long)dbte_table[info].up_bnd_hi,
            (unsigned long)dbte_table[info].v,
            (unsigned long)dbte_table[info].op,
            (unsigned long)dbte_table[info].status,
            (unsigned long)dbte_table[info].imm);
        struct dbchecker_cmd err_cmd;
        memset(&err_cmd, 0, sizeof(err_cmd));
        err_cmd.v  = 1;
        err_cmd.op = DBCHECKER_OP_CLEAR;
        dbchecker_command(&err_cmd);
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

/* Initialize user-space DBChecker (open UIO, start poll thread) */
int dbchecker_init(const char *dev)
{
    rte_spinlock_init(&my_lock);
    if (dev) {
        /* copy provided device path into buffer */
        strncpy(uio_device, dev, sizeof(uio_device) - 1);
        uio_device[sizeof(uio_device) - 1] = '\0';
    } else {
        /* try to locate device by name "dbchecker_uio" */
        char found[256];
        if (find_uio_device_by_name("dbchecker_uio", found, sizeof(found)) == 0) {
            strncpy(uio_device, found, sizeof(uio_device) - 1);
            uio_device[sizeof(uio_device) - 1] = '\0';
        }
    }
    uio_fd = open(uio_device, O_RDWR);
    if (uio_fd < 0) {
        fprintf(stderr, "Failed to open %s: %s\n", uio_device, strerror(errno));
        return -1;
    }

    /* determine mmap size from sysfs if possible: /sys/class/uio/<uioX>/maps/map0/size */
    {
        char *b = strrchr(uio_device, '/');
        const char *uioname = b ? b + 1 : uio_device; /* e.g. uio0 */
        char size_path[256];
        snprintf(size_path, sizeof(size_path), "/sys/class/uio/%s/maps/map0/size", uioname);
        FILE *f = fopen(size_path, "r");
        if (f) {
            char buf[64];
            if (fgets(buf, sizeof(buf), f) != NULL) {
                /* parse hex or decimal */
                unsigned long long v = strtoull(buf, NULL, 0);
                if (v > 0) uio_map_size = (size_t)v;
            }
            fclose(f);
        }
    }
    if (uio_map_size == 0) {
        /* fallback to mapping size sufficient for registers */
        uio_map_size = (size_t)(DBCHECKER_REG_NUM * DBCHECKER_REG_SIZE);
    }

    uio_map = mmap(NULL, uio_map_size, PROT_READ | PROT_WRITE, MAP_SHARED, uio_fd, 0);
    if (uio_map == MAP_FAILED) {
        perror("mmap");
        close(uio_fd);
        uio_fd = -1;
        uio_map = NULL;
        return -1;
    }

    dbte_table = (struct dbchecker_mtdt*)rte_zmalloc("dbte_table", 
        sizeof(struct dbchecker_mtdt) * MAX_DBTE_TABLE_SIZE,
        RTE_CACHE_LINE_SIZE
    );
    if (!dbte_table){
        printf("DBChecker: alloc dbte table failed\n");
        return -1;
    }

    printf("dbchecker: cmd size %d, mtdt size %d\n", (int)sizeof(struct dbchecker_cmd), (int)sizeof(struct dbchecker_mtdt));

    dbchecker_en_set(DBCHECKER_ENABLE_MASK);
    dbchecker_enable = 1;
    printf("DBCHECKER (userspace): init, using %s\n", uio_device);
    return 0;
}

/* Cleanup user-space DBChecker */
void dbchecker_exit(void)
{
    // if (err_thread_running) {
    //     err_thread_running = false;
    //     pthread_join(err_thread, NULL);
    // }
    dbchecker_free_all_mtdt();
    dbchecker_en_set(DBCHECKER_DISABLE_MASK);
    if (uio_map) {
        munmap(uio_map, uio_map_size);
        uio_map = NULL;
        uio_map_size = 0;
    }
    if (uio_fd >= 0) close(uio_fd);
    dbchecker_enable = 0;
    printf("DBCHECKER (userspace): exit\n");
}

int dbchecker_module_init_hook(void)
{
    /* use default device discovery behavior */
    return dbchecker_init(NULL);
}

void dbchecker_module_exit_hook(void)
{
    dbchecker_exit();
}

void dbchecker_alloc_mtdt_hook(struct rte_mbuf *m, enum dma_data_direction dir)
{
    if (!m || uio_map == NULL)
        return;
    if (!RTE_MBUF_DIRECT(m))
        return;
    dma_addr_t base = (dma_addr_t)rte_mbuf_iova_get(m);
    size_t len = (size_t)m->buf_len;
    if (base == 0 || len == 0)
        return;
    /* If IOVA already appears translated (DBChecker encodes table index
     * in high bits), avoid double-allocating metadata for the same buffer.
     * This makes the hook idempotent when it is accidentally called twice.
     */
    if (((uint64_t)base >> 52) != 0) {
        DBCHECKER_DEBUG_LOG("dbchecker_alloc_mtdt_hook: mbuf iova already translated 0x%llx, skipping\n",
            (unsigned long long)base);
        return;
    }

    dma_addr_t new_iova = dbchecker_alloc_mtdt(base, len, dir);
    if (new_iova != (dma_addr_t)-1) {
        rte_mbuf_iova_set(m, new_iova);
        DBCHECKER_DEBUG_LOG("dbchecker_alloc_mtdt_hook: updated mbuf iova 0x%llx -> 0x%llx\n",
            (unsigned long long)base, (unsigned long long)new_iova);
    }
}

void dbchecker_free_mtdt_hook(struct rte_mbuf *m)
{
    if (!m || uio_map == NULL)
        return;
    if (!RTE_MBUF_DIRECT(m))
        return;
    dma_addr_t iova = (dma_addr_t)rte_mbuf_iova_get(m);
    if (iova == 0)
        return;
    /* If IOVA does not contain DBChecker translation bits, skip free.
     * This avoids double-freeing metadata if the hook was called more than once.
     */
    // if (((uint64_t)iova >> 52) == 0) {
    //     DBCHECKER_DEBUG_LOG("dbchecker_free_mtdt_hook: mbuf iova not translated 0x%llx, skipping\n",
    //         (unsigned long long)iova);
    //     return;
    // }

    rte_mbuf_iova_set(m, dbchecker_free_mtdt(iova));
    DBCHECKER_DEBUG_LOG("dbchecker_free_mtdt_hook: freed mbuf iova 0x%llx\n",
        (unsigned long long)iova);
}

/* Hooks for memzone allocations/freeing used by ethdev dma-zone helpers. */
void dbchecker_dma_zone_alloc_hook(const struct rte_memzone *mz)
{
    if (!mz || uio_map == NULL)
        return;
    dma_addr_t base = (dma_addr_t)mz->iova;
    size_t len = mz->len;
    if (base == 0 || len == 0)
        return;
    dma_addr_t new_iova = dbchecker_alloc_mtdt(base, len, DMA_BIDIRECTIONAL);
    if (new_iova != (dma_addr_t)-1) {
        /* update memzone iova so drivers program the device with the
         * address that has associated MTDT metadata. Cast away const to
         * update the internal memzone descriptor. */
        struct rte_memzone *mz_nc = (struct rte_memzone *)(uintptr_t)mz;
        mz_nc->iova = (rte_iova_t)new_iova;
        //dbchecker_activate_mtdt(new_iova, DMA_BIDIRECTIONAL);
        DBCHECKER_DEBUG_LOG("dbchecker_dma_zone_alloc_hook: updated memzone '%s' iova 0x%llx -> 0x%llx\n",
            mz->name, (unsigned long long)base, (unsigned long long)new_iova);
    }
}

void dbchecker_dma_zone_free_hook(const struct rte_memzone *mz)
{
    if (!mz || uio_map == NULL || !mz->iova)
        return;
    struct rte_memzone *mz_nc = (struct rte_memzone *)(uintptr_t)mz;
    mz_nc->iova = dbchecker_free_mtdt((dma_addr_t)mz_nc->iova);
    DBCHECKER_DEBUG_LOG("dbchecker_dma_zone_free_hook: freed memzone '%s' iova 0x%llx\n",
        mz_nc->name, (unsigned long long)mz_nc->iova);
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