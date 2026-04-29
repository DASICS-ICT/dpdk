#ifndef VNIC_RXTX_H
#define VNIC_RXTX_H

#include <rte_mempool.h>
#include <rte_malloc.h>
#include <rte_mbuf.h>
#include <rte_eal.h>
#include <rte_memory.h>
#include <rte_byteorder.h>
#include <rte_common.h>

#include "vnic_regs.h"

#define VNIC_MAX_QUEUE_DEPTH 0x1000ULL

#define VNIC_DESC_STATUS_DONE       0x80000000ULL
#define VNIC_DESC_STATUS_VALID      0x40000000ULL

struct vnic_rxtx_desc {
	uint64_t buf;    // Buffer address
	uint64_t len;    // Length of the received packet
	uint64_t id;   // RX/TX identifier
	uint32_t status; // Status flags
	uint32_t op;  
	// 0x0: be a good guy!
	// 0x1: cross the boundary;
	// 0x2: read (write) a write (read) only buffer
	// 0x3: use after free
};

enum vnic_desc_op {
	VNIC_DESC_OP_GOOD = 0,
	VNIC_DESC_OP_CROSS_BOUNDARY = 1,
	VNIC_DESC_OP_WO_RO_VIOLATION = 2,
	VNIC_DESC_OP_USE_AFTER_FREE = 3,
	VNIC_DESC_OP_DEV_SPOOF = 4,
};

struct vnic_rxtx_desc_queue {
	uint64_t tail;
	uint64_t head;
	uint64_t last_head;
};


static struct vnic_rxtx_desc_queue* vnic_create_rxtx_queue(void)
{
	struct vnic_rxtx_desc_queue *queue = rte_zmalloc(NULL, sizeof(struct vnic_rxtx_desc_queue), 0);
	if (queue == NULL) {
		printf("Failed to allocate RX/TX queue structure\n");
		return NULL;
	}

	queue->tail = 0;
	queue->head = 0;
	queue->last_head = 0;

	return queue;
}


/*
static void vnic_configure_rxtx_queues(struct vnic_rxtx_desc_queue *rxq, struct vnic_rxtx_desc_queue *txq)
{
	vnic_write_reg64(VNIC_RX_OFFSET, rxq->phys_addr);
	vnic_write_reg64(VNIC_TX_OFFSET, txq->phys_addr);
}
*/


static void vnic_free_rxtx_queues(struct vnic_rxtx_desc_queue *queue)
{
	if (queue)
		rte_free(queue);
}


static uint64_t vnic_tx_burst(struct vnic_rxtx_desc_queue *txq, uint64_t nb_pkts, uint64_t pkt_size) 
{
	uint64_t i;
	uint64_t nb_tx = 0;
	uint64_t next_tail;
	uint64_t last_head;
	//struct rte_mbuf *mbuf;
	volatile struct vnic_rxtx_desc *tx_desc;

	last_head = txq->last_head;

	for (i = 0; i < nb_pkts; i++) {
		// 2. 检查队列空间是否足够
		next_tail = (txq->tail + 1) % VNIC_MAX_QUEUE_DEPTH;
		if (next_tail == last_head) {
			printf("TX queue full, stopping at %" PRIu64 " packets\n", i);
			return i;
		}

		// 3. 准备发送描述符
		tx_desc = (volatile struct vnic_rxtx_desc *)((volatile char*)vnic_tx_desc_base + txq->tail * VNIC_DESC_SIZE);
		uint64_t orig_buf = (uint64_t)(VNIC_RESV_MEM_PHYS + VNIC_RESV_TX_DATA + txq->tail * VNIC_DATA_SIZE);
		#ifndef RTE_ENABLE_DBCHECKER
			tx_desc->buf = orig_buf;
		#else
			tx_desc->buf = dbchecker_alloc_mtdt(orig_buf, pkt_size, DMA_TO_DEVICE, VNIC_DEV_ID, true);
			dbchecker_activate_mtdt(tx_desc->buf, DMA_TO_DEVICE, VNIC_DEV_ID, false);
		#endif
		tx_desc->len = pkt_size;          // 实际数据包长度
		tx_desc->op = VNIC_DESC_OP_GOOD;
		tx_desc->status = VNIC_DESC_STATUS_VALID; // 标记有效
		tx_desc->id = txq->tail;
		txq->tail = next_tail;
		nb_tx++;
		VNIC_DEBUG("submit tx req 0x%llx buf=0x%llx len=0x%llx id=0x%llx status=0x%x\n",
		      (unsigned long long)(VNIC_RESV_MEM_PHYS + VNIC_RESV_TX_DESC + next_tail * VNIC_DESC_SIZE), 
					(unsigned long long)tx_desc->buf, 
					(unsigned long long)tx_desc->len, 
					(unsigned long long)tx_desc->id, tx_desc->status);

	}
    
    // 6. 通知FPGA有新的数据包
    rte_wmb();
    if (nb_tx > 0) {
        vnic_write_reg64(VNIC_REG_TX_TAIL, txq->tail); // 更新FPGA尾指针寄存器
    }

    return nb_tx;
}

static uint64_t vnic_tx_poc(struct vnic_rxtx_desc_queue *txq, enum vnic_desc_op op) 
{
	int i;
	uint64_t nb_tx = 0;
	uint64_t next_tail;
	uint64_t last_head;
	//struct rte_mbuf *mbuf;
	volatile struct vnic_rxtx_desc *tx_desc;

	last_head = txq->last_head;

	for (i = 0; i < 1; i++) {
		next_tail = (txq->tail + 1) % VNIC_MAX_QUEUE_DEPTH;
		if (next_tail == last_head) {
			printf("TX queue full, stopping at %d packets\n", i);
			return i;
		}

		tx_desc = (volatile struct vnic_rxtx_desc *)((volatile char*)vnic_tx_desc_base + txq->tail * VNIC_DESC_SIZE);
		uint64_t orig_buf = (uint64_t)(VNIC_RESV_MEM_PHYS + VNIC_RESV_TX_DATA + txq->tail * VNIC_DATA_SIZE);
		volatile void *data = (volatile void *)((volatile char*)vnic_tx_data_base + txq->tail * VNIC_DATA_SIZE);
		io_memset(data, 'A'+ op, 128);
		tx_desc->len = 64;
		tx_desc->op = op;
		tx_desc->status = VNIC_DESC_STATUS_VALID;
		tx_desc->id = txq->tail;
		txq->tail = next_tail;

		#ifndef RTE_ENABLE_DBCHECKER
			tx_desc->buf = orig_buf;
		#else
			tx_desc->buf = dbchecker_alloc_mtdt(orig_buf, 64, DMA_TO_DEVICE, op == VNIC_DESC_OP_DEV_SPOOF ? 0xf : VNIC_DEV_ID, true);
			dbchecker_activate_mtdt(tx_desc->buf, DMA_TO_DEVICE, tx_desc->op == VNIC_DESC_OP_DEV_SPOOF ? 0xf : VNIC_DEV_ID, false);
			if (tx_desc->op == VNIC_DESC_OP_USE_AFTER_FREE) {
				//printf("free before use tx desc buf\n");
				dbchecker_free_mtdt(tx_desc->buf);
			}
		#endif

		// printf("tx req buf=0x%llx data=0x%llx len=0x%llx op=%d\n", 
		// 	(unsigned long long)tx_desc->buf, *(volatile unsigned long long *)data, (unsigned long long)tx_desc->len, op);
		nb_tx++;
		VNIC_DEBUG("submit tx req 0x%llx buf=0x%llx len=0x%llx id=0x%llx status=0x%x\n",
		      (unsigned long long)(VNIC_RESV_MEM_PHYS + VNIC_RESV_TX_DESC + next_tail * VNIC_DESC_SIZE), 
					(unsigned long long)tx_desc->buf, 
					(unsigned long long)tx_desc->len, 
					(unsigned long long)tx_desc->id, tx_desc->status);

	}
    
    // 6. 通知FPGA有新的数据包
    rte_wmb();
    if (nb_tx > 0) {
        vnic_write_reg64(VNIC_REG_TX_TAIL, txq->tail); // 更新FPGA尾指针寄存器
    }

    return nb_tx;
}

static uint64_t vnic_process_tx_completion(struct vnic_rxtx_desc_queue *txq) 
{
	// 1. 读取FPGA更新后的头指针
	uint64_t new_head = vnic_read_reg64(VNIC_REG_TX_HEAD);
	uint64_t last_head = txq->last_head;
	volatile struct vnic_rxtx_desc *tx_desc;
	uint64_t nb_tx_cmpl = 0;

	// 2. 处理所有已完成的数据包
	while (last_head != new_head) {
		//VNIC_DEBUG("last head: %d, new head: %lu\n", last_head, new_head);
		tx_desc = (volatile struct vnic_rxtx_desc *)((volatile char*)vnic_tx_desc_base + last_head * VNIC_DESC_SIZE);

		// 3. 检查完成状态
		//printf("tx status: %x\n", tx_desc->status);
		//if (tx_desc->status & VNIC_DESC_STATUS_DONE) {
			// 5. 重置描述符状态
			tx_desc->status = 0;
			#ifdef RTE_ENABLE_DBCHECKER
				dbchecker_free_mtdt(tx_desc->buf);
			#endif
			// 6. 移动到下一个描述符
			last_head = (last_head + 1) % VNIC_MAX_QUEUE_DEPTH;
			nb_tx_cmpl++;
			//printf("cmpl tx req buf=0x%lx len=%lu id=%lu status=0x%x\n",
			//	tx_desc->buf, tx_desc->len, tx_desc->id, tx_desc->status);

		//}
	}

	// 7. 更新本地头指针
	txq->last_head = last_head;
	return nb_tx_cmpl;
}

static uint64_t vnic_rx_burst(struct vnic_rxtx_desc_queue *rxq, uint64_t nb_pkts, uint64_t pkt_size) 
{
	uint64_t i;
	uint64_t nb_rx = 0;
	uint64_t next_tail;
	uint64_t last_head;
	//struct rte_mbuf *mbuf;
	volatile struct vnic_rxtx_desc *rx_desc;

	last_head = rxq->last_head;

	for (i = 0; i < nb_pkts; i++) {
		// 2. 检查队列空间是否足够
		next_tail = (rxq->tail + 1) % VNIC_MAX_QUEUE_DEPTH;
		if (next_tail == last_head) {
			printf("RX queue full, stopping at %" PRIx64 " packets\n", i);
			return i;
		}

		// 3. 准备发送描述符
		rx_desc = (volatile struct vnic_rxtx_desc *)((volatile char*)vnic_rx_desc_base + rxq->tail * VNIC_DESC_SIZE);
		uint64_t orig_buf = (uint64_t)(VNIC_RESV_MEM_PHYS + VNIC_RESV_RX_DATA + rxq->tail * VNIC_DATA_SIZE);
		#ifndef RTE_ENABLE_DBCHECKER
			rx_desc->buf = orig_buf;
		#else
			rx_desc->buf = dbchecker_alloc_mtdt(orig_buf, pkt_size, DMA_FROM_DEVICE, VNIC_DEV_ID, true);
			dbchecker_activate_mtdt(rx_desc->buf, DMA_FROM_DEVICE, VNIC_DEV_ID, false);
		#endif
		rx_desc->len = pkt_size;          // 实际数据包长度
		rx_desc->op = VNIC_DESC_OP_GOOD;
		rx_desc->status = VNIC_DESC_STATUS_VALID; // 标记有效
		rx_desc->id = rxq->tail;
		rxq->tail = next_tail;
		nb_rx++;
		VNIC_DEBUG("submit rx req 0x%llx buf=0x%llx len=0x%llx id=0x%llx status=0x%x\n",
		      (unsigned long long)(VNIC_RESV_MEM_PHYS + VNIC_RESV_RX_DESC + next_tail * VNIC_DESC_SIZE), 
					(unsigned long long)rx_desc->buf, 
					(unsigned long long)rx_desc->len, 
					(unsigned long long)rx_desc->id, rx_desc->status);

	}
    
    // 6. 通知FPGA有新的数据包
    rte_wmb();
    if (nb_rx > 0) {
        vnic_write_reg64(VNIC_REG_RX_TAIL, rxq->tail); // 更新FPGA尾指针寄存器
    }

    return nb_rx;
}

// static uint64_t vnic_rx_poc(struct vnic_rxtx_desc_queue *rxq, enum vnic_desc_op op) 
// {
// 	int i;
// 	uint64_t nb_rx = 0;
// 	uint64_t next_tail;
// 	uint64_t last_head;
// 	//struct rte_mbuf *mbuf;
// 	volatile struct vnic_rxtx_desc *rx_desc;

// 	last_head = rxq->last_head;

// 	for (i = 0; i < 1; i++) {
// 		// 2. 检查队列空间是否足够
// 		next_tail = (rxq->tail + 1) % VNIC_MAX_QUEUE_DEPTH;
// 		if (next_tail == last_head) {
// 			printf("RX queue full, stopping at %d packets\n", i);
// 			return i;
// 		}

// 		// 3. 准备发送描述符
// 		rx_desc = (volatile struct vnic_rxtx_desc *)((volatile char*)vnic_rx_desc_base + rxq->tail * VNIC_DESC_SIZE);
// 		uint64_t orig_buf = (uint64_t)(VNIC_RESV_MEM_PHYS + VNIC_RESV_RX_DATA + rxq->tail * VNIC_DATA_SIZE);
// 		volatile void *data = (volatile void *)((volatile char*)vnic_rx_data_base + rxq->tail * VNIC_DATA_SIZE);
// 		io_memset(data, 'a'+ op, 128);
// 		rx_desc->len = 64;
// 		rx_desc->op = op;
// 		rx_desc->status = VNIC_DESC_STATUS_VALID;
// 		rx_desc->id = rxq->tail;
// 		rxq->tail = next_tail;

// 		#ifndef RTE_ENABLE_DBCHECKER
// 			rx_desc->buf = orig_buf;
// 		#else
// 			rx_desc->buf = dbchecker_alloc_mtdt(orig_buf, 64, DMA_FROM_DEVICE);
// 			dbchecker_activate_mtdt(rx_desc->buf, DMA_FROM_DEVICE, VNIC_DEV_ID, false);
// 			if (rx_desc->op == VNIC_DESC_OP_USE_AFTER_FREE) {
// 				printf("free before use rx desc buf\n");
// 				dbchecker_free_mtdt(rx_desc->buf);
// 			}
// 			#endif
		
// 		nb_rx++;
// 		VNIC_DEBUG("submit rx req %llx buf=0x%llx len=%llx id=%llx status=0x%x\n",
// 		      (unsigned long long)(VNIC_RESV_MEM_PHYS + VNIC_RESV_RX_DESC + next_tail * VNIC_DESC_SIZE), 
// 					(unsigned long long)rx_desc->buf, 
// 					(unsigned long long)rx_desc->len, 
// 					(unsigned long long)rx_desc->id, rx_desc->status);

// 	}
    
//     // 6. 通知FPGA有新的数据包
//     rte_wmb();
//     if (nb_rx > 0) {
//         vnic_write_reg64(VNIC_REG_RX_TAIL, rxq->tail); // 更新FPGA尾指针寄存器
//     }

//     return nb_rx;
// }

static uint64_t vnic_process_rx_completion(struct vnic_rxtx_desc_queue *rxq) 
{
	// 1. 读取FPGA更新后的头指针
	uint64_t new_head = vnic_read_reg64(VNIC_REG_RX_HEAD);
	uint64_t last_head = rxq->last_head;
	volatile struct vnic_rxtx_desc *rx_desc;
	uint64_t nb_rx_cmpl = 0;

	// 2. 处理所有已完成的数据包
	while (last_head != new_head) {
		//VNIC_DEBUG("last head: %d, new head: %lu\n", last_head, new_head);
		rx_desc = (volatile struct vnic_rxtx_desc *)((volatile char*)vnic_rx_desc_base + last_head * VNIC_DESC_SIZE);

		// 3. 检查完成状态
		//printf("tx status: %x\n", tx_desc->status);
		//if (tx_desc->status & VNIC_DESC_STATUS_DONE) {
			// 5. 重置描述符状态
			rx_desc->status = 0;
			#ifdef RTE_ENABLE_DBCHECKER
				dbchecker_free_mtdt(rx_desc->buf);
			#endif
			// 6. 移动到下一个描述符
			last_head = (last_head + 1) % VNIC_MAX_QUEUE_DEPTH;
			nb_rx_cmpl++;
			//printf("cmpl tx req buf=0x%lx len=%lu id=%lu status=0x%x\n",
			//	tx_desc->buf, tx_desc->len, tx_desc->id, tx_desc->status);

		//}
	}

	// 7. 更新本地头指针
	rxq->last_head = last_head;
	return nb_rx_cmpl;
}


#endif /* VNIC_RXTX_H */