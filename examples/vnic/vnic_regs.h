#ifndef VNIC_REGS_H
#define VNIC_REGS_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#define VNIC_REGS_PHYS          0x11000000ULL
#define VNIC_REG_RX_DESC        0x0ULL
#define VNIC_REG_RX_DATA        0x8ULL
#define VNIC_REG_TX_DESC        0x10ULL
#define VNIC_REG_TX_DATA        0x18ULL
#define VNIC_REG_RX_TAIL        0x20ULL
#define VNIC_REG_RX_HEAD        0x28ULL
#define VNIC_REG_TX_TAIL        0x30ULL
#define VNIC_REG_TX_HEAD        0x38ULL
#define VNIC_REG_STATUS         0x40ULL
#define VNIC_REGS_SIZE          0x100ULL

#define VNIC_DESC_SIZE          0x20ULL
#define VNIC_DATA_SIZE          0x1000ULL
#define VNIC_MAX_QUEUE_DEPTH    0x1000ULL

#define VNIC_RESV_MEM_PHYS     0xf8000000ULL
#define VNIC_RESV_MEM_SIZE     0x8000000ULL
#define VNIC_RESV_RX_DESC      0x0ULL
#define VNIC_RESV_RX_DATA      VNIC_RESV_RX_DESC + VNIC_DESC_SIZE * VNIC_MAX_QUEUE_DEPTH
#define VNIC_RESV_TX_DESC      VNIC_RESV_RX_DATA + VNIC_DATA_SIZE * VNIC_MAX_QUEUE_DEPTH
#define VNIC_RESV_TX_DATA      VNIC_RESV_TX_DESC + VNIC_DESC_SIZE * VNIC_MAX_QUEUE_DEPTH

#define VNIC_STATUS_IDLE 0x0
#define VNIC_STATUS_BUSY 0x80000000ULL

#define DEBUG 0
#define VNIC_DEBUG(fmt, args...) \
	do { \
		if (DEBUG) \
			printf(fmt, ##args); \
	} while (0)

static volatile void *vnic_regs_virt;
static volatile void *vnic_resv_mem_virt;
static volatile void *vnic_rx_desc_base;
static volatile void *vnic_rx_data_base;
static volatile void *vnic_tx_desc_base;
static volatile void *vnic_tx_data_base;
static volatile void *vnic_rx_tail;
static volatile void *vnic_rx_head;
static volatile void *vnic_tx_tail;
static volatile void *vnic_tx_head;
static volatile void *vnic_status;

static int reg_fd;
static int resv_mem_fd;

static int vnic_init_buf(void)
{
	resv_mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (resv_mem_fd < 0) {
		perror("Failed to open /dev/mem for reserved memory");
		return -1;
	}

	vnic_resv_mem_virt = mmap(NULL, VNIC_RESV_MEM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, resv_mem_fd, VNIC_RESV_MEM_PHYS);
	if (vnic_resv_mem_virt == MAP_FAILED) {
		perror("Failed to mmap reserved memory");
		close(resv_mem_fd);
		return -1;
	}

	memset((void *)vnic_resv_mem_virt, 0, VNIC_RESV_MEM_SIZE);

	vnic_rx_desc_base = vnic_resv_mem_virt + VNIC_RESV_RX_DESC;
	vnic_rx_data_base = vnic_resv_mem_virt + VNIC_RESV_RX_DATA;
	vnic_tx_desc_base = vnic_resv_mem_virt + VNIC_RESV_TX_DESC;
	vnic_tx_data_base = vnic_resv_mem_virt + VNIC_RESV_TX_DATA;

	return 0;
}

static int vnic_init_regs(void)
{
	reg_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (reg_fd < 0) {
		perror("Failed to open /dev/mem");
		return -1;
	}

	vnic_regs_virt = mmap(NULL, VNIC_REGS_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, reg_fd, VNIC_REGS_PHYS);
	if (vnic_regs_virt == MAP_FAILED) {
		perror("Failed to mmap /dev/mem");
		close(reg_fd);
		return -1;
	}

	memset((void *)vnic_regs_virt, 0, VNIC_REGS_SIZE);
	vnic_write_reg64(VNIC_REG_RX_DESC, VNIC_RESV_MEM_PHYS + VNIC_RESV_RX_DESC);
	vnic_write_reg64(VNIC_REG_RX_DATA, VNIC_RESV_MEM_PHYS + VNIC_RESV_RX_DATA);
	vnic_write_reg64(VNIC_REG_TX_DESC, VNIC_RESV_MEM_PHYS + VNIC_RESV_TX_DESC);
	vnic_write_reg64(VNIC_REG_TX_DATA, VNIC_RESV_MEM_PHYS + VNIC_RESV_TX_DATA);

	return 0;
}

static void vnic_flush_cache(void)
{

	uint64_t cache_block_size = 64;
	uint64_t cache_sets = 128;
	uint64_t cache_size = 32768;
	uint64_t cache_associativity = cache_size / (cache_block_size * cache_sets);
	uint8_t *buffer = malloc(2 * cache_size);
	if (!buffer) {
		return;
	}
	uint8_t *v_buffer = buffer;
	int set, way;
	uint8_t *ptr;
	for (set = 0; set < cache_sets; set++) {
		for (way = 0; way < cache_associativity; way++) {
			// 计算当前目标地址：基址 + 块偏移 + 组偏移 + 路偏移
			ptr = v_buffer + (set * cache_block_size) + (way * cache_sets * cache_block_size);
			*ptr = 1;
		}
	}
	free(buffer);
}

void vnic_cleanup_regs(void)
{
	munmap((void *)vnic_regs_virt, VNIC_REGS_SIZE);
	close(reg_fd);
	close(resv_mem_fd);
}

uint64_t vnic_read_reg64(uint32_t offset)
{
	uint64_t val;
	//vnic_flush_cache();
	val = *(volatile uint64_t *)((void *)vnic_regs_virt + offset);
	return val;
}

void vnic_write_reg64(uint32_t offset, uint64_t val)
{
	*(volatile uint64_t *)((void *)vnic_regs_virt + offset) = val;
	//vnic_flush_cache();
}

uint32_t vnic_read_reg32(uint32_t offset)
{
	uint32_t val;
	//vnic_flush_cache();
	val = *(volatile uint32_t *)((void *)vnic_regs_virt + offset);
	return val;
}

void vnic_write_reg32(uint32_t offset, uint32_t val)
{
	*(volatile uint32_t *)((void *)vnic_regs_virt + offset) = val;
	//vnic_flush_cache();
}


void vnic_dump_regs(void)
{
	VNIC_DEBUG("VNIC RX DESC Base: 0x%llx\n", vnic_read_reg64(VNIC_REG_RX_DESC));
	VNIC_DEBUG("VNIC RX DATA Base: 0x%llx\n", vnic_read_reg64(VNIC_REG_RX_DATA));
	VNIC_DEBUG("VNIC TX DESC Base: 0x%llx\n", vnic_read_reg64(VNIC_REG_TX_DESC));
	VNIC_DEBUG("VNIC TX	DATA Base: 0x%llx\n", vnic_read_reg64(VNIC_REG_TX_DATA));
	VNIC_DEBUG("VNIC RX Tail: 0x%llx\n", vnic_read_reg64(VNIC_REG_RX_TAIL));
	VNIC_DEBUG("VNIC RX Head: 0x%llx\n", vnic_read_reg64(VNIC_REG_RX_HEAD));
	VNIC_DEBUG("VNIC TX Tail: 0x%llx\n", vnic_read_reg64(VNIC_REG_TX_TAIL));
	VNIC_DEBUG("VNIC TX Head: 0x%llx\n", vnic_read_reg64(VNIC_REG_TX_HEAD));
	VNIC_DEBUG("VNIC Status: 0x%llx\n", vnic_read_reg64(VNIC_REG_STATUS));

}

#endif /* VNIC_REGS_H */