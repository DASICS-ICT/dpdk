#ifndef VNIC_REGS_H
#define VNIC_REGS_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stddef.h>
#include <inttypes.h>

#ifdef RTE_ENABLE_DBCHECKER
    #include <rte_dbchecker.h>
#endif

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

#define VNIC_DEV_ID 0x1UL

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

static int reg_fd;
static int resv_mem_fd;

static void vnic_cleanup_regs(void)
{
	munmap((void *)(uintptr_t)vnic_regs_virt, VNIC_REGS_SIZE);
	close(reg_fd);
	close(resv_mem_fd);
}

static uint64_t vnic_read_reg64(uint32_t offset)
{
	uint64_t val;
	val = *(volatile uint64_t *)((volatile char *)vnic_regs_virt + offset);
	return val;
}

static void vnic_write_reg64(uint32_t offset, uint64_t val)
{
	*(volatile uint64_t *)((volatile char *)vnic_regs_virt + offset) = val;
}

// static uint32_t vnic_read_reg32(uint32_t offset)
// {
// 	uint32_t val;
// 	val = *(volatile uint32_t *)((volatile char *)vnic_regs_virt + offset);
// 	return val;
// }

static void vnic_write_reg32(uint32_t offset, uint32_t val)
{
	*(volatile uint32_t *)((volatile char *)vnic_regs_virt + offset) = val;
}

/**
 * @brief Safe memset function for I/O memory
 * 
 * @param dst  Destination address (volatile void*), usually mapped virtual address
 * @param c    Value to set (truncated to unsigned char)
 * @param n    Number of bytes (size_t)
 */
static inline void io_memset(volatile void *dst, int c, size_t n)
{
    volatile uint8_t *u8_ptr = (volatile uint8_t *)dst;
    uint8_t val8 = (uint8_t)c;
    
    // 1. Try to perform 32-bit optimized access
    // Use 32-bit writes only if the address is 4-byte aligned and length >= 4.
    // This prevents unaligned access faults on architectures like ARM.
    if (((uintptr_t)u8_ptr & 0x3) == 0 && n >= 4) {
        
        // Expand the 8-bit value to 32-bit (e.g., 0xAB -> 0xABABABAB)
        uint32_t val32 = (uint32_t)val8 | 
                         ((uint32_t)val8 << 8) | 
                         ((uint32_t)val8 << 16) | 
                         ((uint32_t)val8 << 24);
        
        volatile uint32_t *u32_ptr = (volatile uint32_t *)dst;
        size_t u32_count = n / 4;

        // Core loop: Write with 32-bit width
        while (u32_count--) {
            *u32_ptr++ = val32;
        }

        // Update pointer and remaining length to handle the "tail"
        u8_ptr = (volatile uint8_t *)u32_ptr;
        n %= 4;
    }

    // 2. Handle remaining bytes (or handle unaligned start addresses)
    // Note: If the hardware strictly forbids byte access, this part might fail
    // for the tail, but typically VNIC memory sizes are multiples of 4.
    while (n--) {
        *u8_ptr++ = val8;
    }
}

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

	io_memset(vnic_resv_mem_virt, 0, VNIC_RESV_MEM_SIZE);

	vnic_rx_desc_base = (volatile void*)((volatile char*)vnic_resv_mem_virt + VNIC_RESV_RX_DESC);
	vnic_rx_data_base = (volatile void*)((volatile char*)vnic_resv_mem_virt + VNIC_RESV_RX_DATA);
	vnic_tx_desc_base = (volatile void*)((volatile char*)vnic_resv_mem_virt + VNIC_RESV_TX_DESC);
	vnic_tx_data_base = (volatile void*)((volatile char*)vnic_resv_mem_virt + VNIC_RESV_TX_DATA);

	return 0;
}

static void vnic_dump_regs(void)
{
	VNIC_DEBUG("VNIC RX DESC Base: 0x%llx\n", (unsigned long long)vnic_read_reg64(VNIC_REG_RX_DESC));
	VNIC_DEBUG("VNIC RX DATA Base: 0x%llx\n", (unsigned long long)vnic_read_reg64(VNIC_REG_RX_DATA));
	VNIC_DEBUG("VNIC TX DESC Base: 0x%llx\n", (unsigned long long)vnic_read_reg64(VNIC_REG_TX_DESC));
	VNIC_DEBUG("VNIC TX DATA Base: 0x%llx\n", (unsigned long long)vnic_read_reg64(VNIC_REG_TX_DATA));
	VNIC_DEBUG("VNIC RX Tail: 0x%llx\n", (unsigned long long)vnic_read_reg64(VNIC_REG_RX_TAIL));
	VNIC_DEBUG("VNIC RX Head: 0x%llx\n", (unsigned long long)vnic_read_reg64(VNIC_REG_RX_HEAD));
	VNIC_DEBUG("VNIC TX Tail: 0x%llx\n", (unsigned long long)vnic_read_reg64(VNIC_REG_TX_TAIL));
	VNIC_DEBUG("VNIC TX Head: 0x%llx\n", (unsigned long long)vnic_read_reg64(VNIC_REG_TX_HEAD));
	VNIC_DEBUG("VNIC Status: 0x%llx\n", (unsigned long long)vnic_read_reg64(VNIC_REG_STATUS));
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

	io_memset(vnic_regs_virt, 0, VNIC_REGS_SIZE);
	#ifndef RTE_ENABLE_DBCHECKER
		vnic_write_reg64(VNIC_REG_RX_DESC, VNIC_RESV_MEM_PHYS + VNIC_RESV_RX_DESC);
		vnic_write_reg64(VNIC_REG_RX_DATA, VNIC_RESV_MEM_PHYS + VNIC_RESV_RX_DATA);
		vnic_write_reg64(VNIC_REG_TX_DESC, VNIC_RESV_MEM_PHYS + VNIC_RESV_TX_DESC);
		vnic_write_reg64(VNIC_REG_TX_DATA, VNIC_RESV_MEM_PHYS + VNIC_RESV_TX_DATA);
	#else
		dma_addr_t rxq_phys = 
			dbchecker_alloc_mtdt((dma_addr_t)(VNIC_RESV_MEM_PHYS + VNIC_RESV_RX_DESC),
					VNIC_DESC_SIZE * VNIC_MAX_QUEUE_DEPTH, DMA_BIDIRECTIONAL, VNIC_DEV_ID);
			dbchecker_activate_mtdt(rxq_phys, DMA_BIDIRECTIONAL, VNIC_DEV_ID, false);
		dma_addr_t txq_phys = 
			dbchecker_alloc_mtdt((dma_addr_t)(VNIC_RESV_MEM_PHYS + VNIC_RESV_TX_DESC),
					VNIC_DESC_SIZE * VNIC_MAX_QUEUE_DEPTH, DMA_BIDIRECTIONAL, VNIC_DEV_ID);
			dbchecker_activate_mtdt(txq_phys, DMA_BIDIRECTIONAL, VNIC_DEV_ID, false);
		vnic_write_reg64(VNIC_REG_RX_DESC, rxq_phys);
		vnic_write_reg64(VNIC_REG_TX_DESC, txq_phys);
	#endif
	vnic_dump_regs();
	return 0;
}

#endif /* VNIC_REGS_H */