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
#include <rte_io.h>
#include <rte_malloc.h>
#include <rte_memory.h>
#include <rte_platform_vfio_helper.h>

#include "rte_dbchecker.h"

static uint16_t dbte_alloc_id;
static uint8_t dbchecker_enable;
static struct rte_platform_vfio_device dbchecker_vfio = RTE_PLATFORM_VFIO_DEVICE_INITIALIZER;
static dbchecker_mtdt_u *dbte_table;

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

/*
 * dbte_alloc_id layout: [ group (12 bits) | offset (4 bits) ]
 */
static inline uint16_t
dbte_next_id(uint16_t id)
{
	uint16_t offset = id & 0xFULL;
	uint16_t group = id >> 4;

	group++;
	if (group > 0xFFF) {
		group = 0;
		offset = (offset + 1) & 0xFULL;
	}
	return (uint16_t)((group << 4) | (offset & 0xF));
}

int
dbchecker_command(uint32_t cmd)
{
	dbchecker_reg_write32(DBCHECKER_CMD_OFFSET, cmd);
	return 0;
}

void
dbchecker_en_set(uint32_t dev_mask)
{
	dbchecker_reg_write32(DBCHECKER_EN_OFFSET, dev_mask);
}

uint32_t
dbchecker_en_get(void)
{
	return dbchecker_reg_read32(DBCHECKER_EN_OFFSET);
}

RTE_EXPORT_SYMBOL(dbchecker_alloc_mtdt)
dma_addr_t
dbchecker_alloc_mtdt(dma_addr_t addr, size_t size, enum dma_data_direction dir, uint16_t dev_id, bool auto_rel)
{
	if (!dbchecker_enable) {
		if (dbchecker_init() < 0)
			return (dma_addr_t)-1;
	}
	dbchecker_mtdt_u mtdt;
	if (likely(dir <= DMA_TO_DEVICE))
		mtdt.wr = dma_to_db_map[dir];
	else
		mtdt.wr = DBCHECKER_RWMODE_INVALID;

	dma_addr_t alloc_addr = (dma_addr_t)-1;

	mtdt.lo_bnd = addr & 0xFFFFFFFFFFFFULL;
	mtdt.up_bnd_lo = (uint16_t)((addr + size) & 0xFFFFULL);
	mtdt.up_bnd_hi = (uint32_t)(((addr + size) >> 16) & 0xFFFFFFFFUL);
	mtdt.dev_id = dev_id & 0x1F;
	mtdt.auto_rel_en = auto_rel ? 1 : 0;

	uint16_t start = dbte_alloc_id;
	uint16_t idx = start;
	bool found = false;
	const uint64_t v_mask = (1ULL << 39);

	do {
		if (idx != 0 && (dbte_table[idx].raw1 & v_mask) == 0) {
			found = true;
			break;
		}
		idx = dbte_next_id(idx);
	} while (idx != start);

	if (unlikely(!found)) {
		printf("DBCHECKER: alloc failed, table full (start idx %u)\n", start);
		return alloc_addr;
	}

	mtdt.index_off = (idx & 0xFUL);
	mtdt.v = 1;

	alloc_addr = (addr & 0xFFFFFFFFFFFFULL) | ((uint64_t)idx << 48);

	dbte_table[idx].raw0 = mtdt.raw0;
	rte_wmb();
	dbte_table[idx].raw1 = mtdt.raw1;
	dbte_alloc_id = dbte_next_id(idx);
	// printf("DBCHECKER: alloc mtdt idx %u for addr 0x%lx size 0x%zx (use iova 0x%lx)\n",
	// 	idx, (unsigned long long)addr, size, (unsigned long long)alloc_addr);
	return alloc_addr;
}

RTE_EXPORT_SYMBOL(dbchecker_free_mtdt)
dma_addr_t
dbchecker_free_mtdt(dma_addr_t addr)
{
	if (!dbchecker_enable)
		return addr;

	uint16_t index = (uint16_t)((addr >> 48) & 0xFFFFUL);

	dbchecker_mtdt_u mtdt;

	mtdt.raw1 = dbte_table[index].raw1;
	mtdt.v = 0;

	dbte_table[index].raw1 = mtdt.raw1;
	rte_wmb();
	if (!mtdt.auto_rel_en) {
		dbchecker_cmd_u free_cmd = {
			.imm = index,
			.op  = DBCHECKER_OP_FREE,
			.v   = 1
		};
		dbchecker_command(free_cmd.raw);
	}

	return addr & 0xFFFFFFFFFFFFULL;
}

void
dbchecker_free_all_mtdt(void)
{
	dbchecker_cmd_u free_cmd = {
		.imm = 1 << 16,
		.op  = DBCHECKER_OP_FREE,
		.v   = 1
	};
	dbchecker_command(free_cmd.raw);
}

RTE_EXPORT_SYMBOL(dbchecker_err_handler)
int
dbchecker_err_handler(void)
{
	if (dbchecker_vfio.regs == NULL || !dbchecker_enable)
		return 0;

	uint32_t cnt = dbchecker_reg_read32(DBCHECKER_ERR_CNT_OFFSET);
	uint32_t info = dbchecker_reg_read32(DBCHECKER_ERR_INFO_OFFSET);
	uint32_t addr_lo = dbchecker_reg_read32(DBCHECKER_ERR_ADDR_LO_OFFSET);
	uint32_t addr_hi = dbchecker_reg_read32(DBCHECKER_ERR_ADDR_HI_OFFSET);
	uint64_t addr = ((uint64_t)addr_hi << 32) | addr_lo;
	uint16_t index = (uint16_t)((addr >> 48) & 0xFFFFUL);

	if (cnt & ~0xF) {
		fprintf(stderr, "DBCHECKER: error detected!\n");
		fprintf(stderr, "DBCHECKER: error count: 0x%llx, info: 0x%llx, addr: 0x%llx\n",
			(unsigned long long)cnt, (unsigned long long)info, (unsigned long long)addr);
		if (index < MAX_DBTE_TABLE_SIZE && dbte_table != NULL)
			fprintf(stderr, "DBCHECKER: error mtdt raw0 : 0x%llx, raw1: 0x%llx\n",
				(unsigned long long)dbte_table[index].raw0,
				(unsigned long long)dbte_table[index].raw1);
		fprintf(stderr, "DBCHECKER: error type: %s\n",
			((cnt >> 4)  & 0x7F) ? "cross boundary violation" :
			((cnt >> 11) & 0x7F) ? "write-read violation" :
			((cnt >> 18) & 0x7F) ? "invalid metadata" :
			((cnt >> 25) & 0x7F) ? "device mismatch" : "unknown");
		dbchecker_cmd_u err_cmd = {
			.op  = DBCHECKER_OP_CLEAR,
			.v   = 1
		};
		dbchecker_command(err_cmd.raw);
		return -1;
	}
	return 0;
}

static void
dbchecker_resolve_params(const struct dbchecker_params *params, struct dbchecker_params *cfg)
{
	dbchecker_default_params(cfg);
	if (params == NULL)
		return;
	if (params->bus_devices_path != NULL)
		cfg->bus_devices_path = params->bus_devices_path;
	if (params->device_name != NULL)
		cfg->device_name = params->device_name;
	if (params->region_index != 0)
		cfg->region_index = params->region_index;
	cfg->debug_log = params->debug_log;
}

RTE_EXPORT_SYMBOL(dbchecker_default_params)
void
dbchecker_default_params(struct dbchecker_params *params)
{
	if (params == NULL)
		return;
	params->bus_devices_path = DBCHECKER_DEFAULT_PLATFORM_BUS_DEVICES_PATH;
	params->device_name = DBCHECKER_DEFAULT_DEVICE_NAME;
	params->region_index = DBCHECKER_DEFAULT_REGION_INDEX;
	params->debug_log = 0;
}

RTE_EXPORT_SYMBOL(dbchecker_init_with_params)
int
dbchecker_init_with_params(const struct dbchecker_params *params)
{
	struct dbchecker_params cfg;
	struct rte_platform_vfio_params vparams;
	int ret;

	if (dbchecker_vfio.regs != NULL)
		return -EBUSY;

	dbchecker_resolve_params(params, &cfg);

	rte_platform_vfio_default_params(&vparams);
	vparams.bus_devices_path = cfg.bus_devices_path;
	vparams.device_name = cfg.device_name;
	vparams.region_index = cfg.region_index;
	vparams.debug_log = cfg.debug_log;

	ret = rte_platform_vfio_open(&dbchecker_vfio, &vparams);
	if (ret)
		return ret;

	dbte_table = (dbchecker_mtdt_u *)rte_zmalloc("dbte_table",
		sizeof(dbchecker_mtdt_u) * MAX_DBTE_TABLE_SIZE,
		RTE_CACHE_LINE_SIZE);
	if (!dbte_table) {
		printf("DBChecker: alloc dbte table failed\n");
		rte_platform_vfio_close(&dbchecker_vfio);
		return -ENOMEM;
	}

	uint64_t dbte_table_phys = rte_mem_virt2iova((const void *)dbte_table);

	DBCHECKER_DEBUG_LOG("DBChecker: dbte_table_iova = %lx\n", dbte_table_phys);
	dbchecker_reg_write32(DBCHECKER_DBTE_MB_LO_OFFSET,
		(uint32_t)(dbte_table_phys & 0xFFFFFFFFUL));
	dbchecker_reg_write32(DBCHECKER_DBTE_MB_HI_OFFSET,
		(uint32_t)(dbte_table_phys >> 32));
	dbchecker_en_set(DBCHECKER_ENABLE_MASK);
	dbchecker_enable = 1;
	printf("DBCHECKER (userspace): init vfio-platform device %s\n", cfg.device_name);
	return 0;
}

int
dbchecker_init(void)
{
	struct dbchecker_params params;

	dbchecker_default_params(&params);
	return dbchecker_init_with_params(&params);
}

void
dbchecker_exit(void)
{
	dbchecker_free_all_mtdt();
	dbchecker_en_set(DBCHECKER_DISABLE_MASK);

	if (dbte_table != NULL) {
		rte_free(dbte_table);
		dbte_table = NULL;
	}

	rte_platform_vfio_close(&dbchecker_vfio);
	dbchecker_enable = 0;
	printf("DBCHECKER (userspace): exit\n");
}

RTE_EXPORT_SYMBOL(dbchecker_module_init_hook)
int
dbchecker_module_init_hook(void)
{
	return dbchecker_init();
}

RTE_EXPORT_SYMBOL(dbchecker_module_exit_hook)
void
dbchecker_module_exit_hook(void)
{
	dbchecker_exit();
}

dma_addr_t
dbchecker_alloc_mtdt_generic(dma_addr_t addr, size_t size,
	enum dma_data_direction dir, uint16_t dev_id, bool auto_rel)
{
	if (!dbchecker_enable) {
		if (dbchecker_init() < 0)
			return (dma_addr_t)-1;
	}

	dbchecker_mtdt_u mtdt;
	if (likely(dir <= DMA_TO_DEVICE))
		mtdt.wr = dma_to_db_map[dir];
	else
		mtdt.wr = DBCHECKER_RWMODE_INVALID;

	dma_addr_t alloc_addr = (dma_addr_t)-1;

	mtdt.lo_bnd = addr & 0xFFFFFFFFFFFFULL;
	mtdt.up_bnd_lo = (uint16_t)((addr + size) & 0xFFFFULL);
	mtdt.up_bnd_hi = (uint32_t)(((addr + size) >> 16) & 0xFFFFFFFFUL);
	mtdt.dev_id = dev_id;
	mtdt.auto_rel_en = auto_rel ? 1 : 0;

	uint16_t start = dbte_alloc_id;
	uint16_t idx = start;
	bool found = false;
	const uint64_t v_mask = (1ULL << 39);

	do {
		if (idx != 0 && (dbte_table[idx].raw1 & v_mask) == 0) {
			found = true;
			break;
		}
		idx = dbte_next_id(idx);
	} while (idx != start);

	if (unlikely(!found)) {
		printf("DBCHECKER: alloc failed, table full (start idx %u)\n", start);
		return alloc_addr;
	}

	mtdt.index_off = (idx & 0xFUL);
	mtdt.v = 1;

	alloc_addr = (addr & 0xFFFFFFFFFFFFULL) | ((uint64_t)idx << 48);

	dbte_table[idx].raw0 = mtdt.raw0;
	rte_wmb();
	dbte_table[idx].raw1 = mtdt.raw1;
	dbte_alloc_id = dbte_next_id(idx);
	return alloc_addr;
}

dma_addr_t
dbchecker_free_mtdt_generic(dma_addr_t addr)
{
	if (!dbchecker_enable)
		return addr;

	uint16_t index = (uint16_t)((addr >> 48) & 0xFFFFUL);

	if (index >= MAX_DBTE_TABLE_SIZE) {
		printf("DBCHECKER Error: free mtdt failed, index %u out of bounds (Max %u)\n",
			index, MAX_DBTE_TABLE_SIZE);
		return (dma_addr_t)-1;
	}

	dbchecker_mtdt_u mtdt;

	mtdt.raw1 = dbte_table[index].raw1;
	mtdt.v = 0;

	dbte_table[index].raw1 = mtdt.raw1;
	rte_wmb();
	dbchecker_cmd_u free_cmd = {
		.imm = index,
		.op  = DBCHECKER_OP_FREE,
		.v   = 1
	};
	dbchecker_command(free_cmd.raw);

	return addr & 0xFFFFFFFFFFFFULL;
}
