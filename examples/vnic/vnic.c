#include "vnic_rxtx.h"
#include <rte_mbuf.h>
#include <rte_cycles.h>
#include <math.h>

#define NUM_MBUFS 8192
#define MBUF_CACHE_SIZE 250
#define DEFAULT_BURST_SIZE 1
#define DEFAULT_PKT_SIZE 64

uint64_t burst_size = DEFAULT_BURST_SIZE;
uint64_t pkt_size = DEFAULT_PKT_SIZE;
int is_tx_thpt = 0;
int is_tx_latency = 0;
int is_rx_thpt = 0;
int is_fwd = 0;

struct perf_stats {
	uint64_t total_bytes;      // 总传输字节数
	uint64_t total_packets;    // 总传输包数
	uint64_t start_tsc;        // 开始时间戳计数器
	uint64_t end_tsc;          // 结束时间戳计数器
	double duration_sec;       // 持续时间(秒)
	double bandwidth_mbps;     // 带宽(Gbps)
	double packets_per_sec;    // 包/秒
};

void parse_args(int argc, char** argv) {
	for (int i = 0; i < argc; i++) {
		if (strcmp(argv[i], "--burst") == 0 && i + 1 < argc) {
			burst_size = atoi(argv[i + 1]);
		}
		if (strcmp(argv[i], "--size") == 0 && i + 1 < argc) {
			pkt_size = atoi(argv[i + 1]);
		}
		if (strcmp(argv[i], "--tx-thpt") == 0) {
			is_tx_thpt = 1;
		}
		if (strcmp(argv[i], "--rx-thpt") == 0) {
			is_rx_thpt = 1;
		}
		if (strcmp(argv[i], "--tx-lat") == 0) {
			is_tx_latency = 1;
		}
		if (strcmp(argv[i], "--fwd") == 0) {
			is_fwd = 1;
		}
	}
}

void vnic_test_rx_throughput(struct vnic_rxtx_desc_queue *rxq, uint64_t nb_pkts, uint64_t pkt_size)
{
	uint64_t nb, nb_cmpl = 0;
	struct perf_stats *stats = malloc(sizeof(struct perf_stats));
	memset(stats, 0, sizeof(struct perf_stats));
	stats->start_tsc = rte_rdtsc();

	nb = vnic_rx_burst(rxq, burst_size, pkt_size);
	VNIC_DEBUG("submit %lx packets for rx\n", nb);
	do {
		nb_cmpl += vnic_process_rx_completion(rxq);
	} while (nb_cmpl != nb);

	stats->end_tsc = rte_rdtsc();
	stats->total_packets = nb_cmpl;
	stats->total_bytes = nb_cmpl * pkt_size;
	stats->duration_sec = (double)(stats->end_tsc - stats->start_tsc) / rte_get_tsc_hz();
	stats->bandwidth_mbps = (double)(stats->total_bytes * 8) / (stats->duration_sec * 1e6);
	stats->packets_per_sec = (double)stats->total_packets / stats->duration_sec;

	printf("\n===== RX Throughput Statistics =====\n");
	printf("Total packets: %" PRIu64 "\n", stats->total_packets);
	printf("Total bytes: %" PRIu64 " bytes\n", stats->total_bytes);
	printf("Duration: %.6f seconds\n", stats->duration_sec);
	printf("Bandwidth: %.3f mbps\n", stats->bandwidth_mbps);
	printf("Throughput: %.2f packets/sec\n", stats->packets_per_sec);
	printf("===================================\n");
}

void vnic_test_tx_throughput(struct vnic_rxtx_desc_queue *txq, uint64_t nb_pkts, uint64_t pkt_size)
{
	uint64_t nb, nb_cmpl = 0;
	struct perf_stats *stats = malloc(sizeof(struct perf_stats));
	memset(stats, 0, sizeof(struct perf_stats));
	stats->start_tsc = rte_rdtsc();

	nb = vnic_tx_burst(txq, nb_pkts, pkt_size);
	VNIC_DEBUG("submit %lx packets for tx\n", nb);
	do {
		nb_cmpl += vnic_process_tx_completion(txq);
	} while (nb_cmpl != nb);

	stats->end_tsc = rte_rdtsc();
	stats->total_packets = nb_cmpl;
	stats->total_bytes = nb_cmpl * pkt_size;
	stats->duration_sec = (double)(stats->end_tsc - stats->start_tsc) / rte_get_tsc_hz();
	stats->bandwidth_mbps = (double)(stats->total_bytes * 8) / (stats->duration_sec * 1e6);
	stats->packets_per_sec = (double)stats->total_packets / stats->duration_sec;

	printf("\n===== TX Throughput Statistics =====\n");
	printf("Total packets: %" PRIu64 "\n", stats->total_packets);
	printf("Total bytes: %" PRIu64 " bytes\n", stats->total_bytes);
	printf("Duration: %.6f seconds\n", stats->duration_sec);
	printf("Bandwidth: %.3f mbps\n", stats->bandwidth_mbps);
	printf("Throughput: %.2f packets/sec\n", stats->packets_per_sec);
	printf("===================================\n");
}

int compare_double(const void *a, const void *b) {
    double da = *(const double *)a;
    double db = *(const double *)b;
    return (da > db) - (da < db);
}

double calculate_percentile(double *data, int count, double percentile) {
    if (count == 0) return 0.0;
    
    // 计算百分位数位置（使用线性插值法）
    double position = (percentile / 100.0) * (count - 1);
    int index = (int)position;
    double fraction = position - index;
    
    // 确保索引在有效范围内
    if (index < 0) index = 0;
    if (index >= count - 1) return data[count - 1];
    
    // 线性插值计算百分位数
    return data[index] + fraction * (data[index + 1] - data[index]);
}

void vnic_test_tx_latency(struct vnic_rxtx_desc_queue *txq)
{
	int i;
	int rep = 100000;
	int nb, nb_cmpl = 0;
	double latency[rep];
	double time = 0;
	for (i = 0; i < rep; i++) {
		time = rte_rdtsc();
		
		nb = vnic_tx_burst(txq, 1, 16);
		VNIC_DEBUG("submit %lx packets for tx\n", nb);
		do {
			nb_cmpl = 0;
			nb_cmpl += vnic_process_tx_completion(txq);
		} while (nb_cmpl != nb);

		latency[i] = (rte_rdtsc() - time) / rte_get_tsc_hz();

	}
	qsort(latency, rep, sizeof(double), compare_double);

	double percentiles[] = {5, 10, 20, 30, 40, 50, 60, 70, 80, 90, 95, 99, 99.5, 99.9};
	int num_percentiles = sizeof(percentiles) / sizeof(percentiles[0]);

	for (i = 0; i < num_percentiles; i++) {
        double value = calculate_percentile(latency, rep, percentiles[i]);
        printf("P%-5.1f: %.6f\n", percentiles[i], value);
    }
}

void vnic_test_fwd(struct vnic_rxtx_desc_queue *rxq, struct vnic_rxtx_desc_queue *txq, uint64_t nb_pkts, uint64_t pkt_size)
{
	printf("starting rx/tx/fwd test\n");
	uint64_t nb_rx, nb_rx_cmpl = 0, nb_tx_cmpl = 0;
	uint64_t next_tail, last_head;
	uint64_t fwd_size = 32;
	uint64_t to_process_pkts = nb_pkts;
	int i;
	struct perf_stats *stats = malloc(sizeof(struct perf_stats));
	volatile struct vnic_rxtx_desc *tx_desc;
	last_head = txq->last_head;
	memset(stats, 0, sizeof(struct perf_stats));
	stats->start_tsc = rte_rdtsc();

	while (to_process_pkts > 0) {
		//printf("to_process_pkts %lx, fwd_size %lx\n", to_process_pkts, fwd_size);
		if (to_process_pkts < fwd_size)
			fwd_size = to_process_pkts;

		nb_rx = vnic_rx_burst(rxq, fwd_size, pkt_size);
		//printf("submit %lx packets for rx\n", nb_rx);
		do {
			nb_rx_cmpl += vnic_process_rx_completion(rxq);
		} while (nb_rx_cmpl != nb_rx);
		//printf("rx completed %lx packets\n", nb_rx_cmpl);
		for (i = 0; i < fwd_size; i++) {
			next_tail = (txq->tail + 1) % VNIC_MAX_QUEUE_DEPTH;
			if (next_tail == last_head) {
				//printf("TX queue full, stopping at %d packets\n", i);
				break;
			}
			//printf("prepare tx descs\n");
			tx_desc = (volatile struct vnic_rxtx_desc *)(vnic_resv_mem_virt + VNIC_RESV_TX_DESC + txq->tail * VNIC_DESC_SIZE);
			tx_desc->buf = (uint64_t)(VNIC_RESV_MEM_PHYS + VNIC_RESV_RX_DATA + (rxq->tail - nb_rx_cmpl) * VNIC_DATA_SIZE);
			tx_desc->len = pkt_size;
			tx_desc->status = VNIC_DESC_STATUS_VALID;
			tx_desc->id = txq->tail;
			txq->tail = next_tail;
		}
		vnic_write_reg64(VNIC_REG_TX_TAIL, txq->tail);
		//printf("submit %lx packets for tx, new tail %lx\n", fwd_size, txq->tail);
		do {
			nb_tx_cmpl += vnic_process_tx_completion(txq);
		} while (nb_tx_cmpl != fwd_size);
		
		to_process_pkts -= fwd_size;
		nb_rx_cmpl = 0;
		nb_tx_cmpl = 0;		
	}
	stats->end_tsc = rte_rdtsc();
	stats->total_packets = nb_pkts * 2;
	stats->total_bytes = nb_pkts * pkt_size * 2;
	stats->duration_sec = (double)(stats->end_tsc - stats->start_tsc) / rte_get_tsc_hz();
	stats->bandwidth_mbps = (double)(stats->total_bytes * 8) / (stats->duration_sec * 1e6);
	stats->packets_per_sec = (double)stats->total_packets / stats->duration_sec;
	printf("\n===== FWD Throughput Statistics =====\n");
	printf("Total packets: %" PRIu64 "\n", stats->total_packets);
	printf("Total bytes: %" PRIu64 " bytes\n", stats->total_bytes);
	printf("Duration: %.6f seconds\n", stats->duration_sec);
	printf("Bandwidth: %.3f mbps\n", stats->bandwidth_mbps);
	printf("Throughput: %.2f packets/sec\n", stats->packets_per_sec);
	printf("===================================\n");
}


int main(int argc, char *argv[])
{
	int ret = 0;
	int i;
	struct rte_mempool *mbuf_pool;
	struct vnic_rxtx_queue *rxq, *txq;
	uint64_t nb = 0, nb_cmpl = 0;

	ret = rte_eal_init(argc, argv);
	if (ret < 0) {
		rte_exit(EXIT_FAILURE, "Error with EAL initialization\n");
	}

	parse_args(argc, argv);
	ret = vnic_init_buf();
	if (ret < 0) {
		rte_exit(EXIT_FAILURE, "Error with VNIC buffer initialization\n");
	}

	ret = vnic_init_regs();
	if (ret < 0) {
		rte_exit(EXIT_FAILURE, "Error with VNIC register initialization\n");
	}

	rxq = vnic_create_rxtx_queue();
	txq = vnic_create_rxtx_queue();

	if (rxq == NULL || txq == NULL) {
		rte_exit(EXIT_FAILURE, "Error with VNIC RX/TX queue initialization\n");
	}

	//vnic_dump_regs();

	vnic_write_reg32(VNIC_REG_STATUS, VNIC_STATUS_BUSY);
	rte_wmb();

	// reset timer
	VNIC_DEBUG("is_rx_thpt %d, is_tx_thpt %d, is_tx_latency %d\n", 
		is_rx_thpt, is_tx_thpt, is_tx_latency);
	// RX main
	if (is_rx_thpt)
		vnic_test_rx_throughput(rxq, burst_size, pkt_size);

	// TX main
	if (is_tx_thpt)
		vnic_test_tx_throughput(txq, burst_size, pkt_size);

	// TX latency test
	if (is_tx_latency)
		vnic_test_tx_latency(txq);

	if (is_fwd)
		vnic_test_fwd(rxq, txq, burst_size, pkt_size);

	vnic_cleanup_regs();
	vnic_free_rxtx_queues(rxq);
	vnic_free_rxtx_queues(txq);

	rte_eal_cleanup();

	return 0;
}
