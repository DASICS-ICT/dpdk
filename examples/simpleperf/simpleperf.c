// simpleperf: DPDK-25 ethdev bandwidth tester

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
// simpleperf: DPDK-25 ethdev bandwidth tester (clean version)

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>

#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_mempool.h>
#include <rte_mbuf.h>
#include <rte_cycles.h>
#include <rte_ether.h>

#define NUM_MBUFS  16384
#define MBUF_CACHE_SIZE 250
#define DEFAULT_BURST_SIZE 128
#define DEFAULT_PKT_SIZE 64
#define DEFAULT_SECONDS 3600

static uint16_t g_port_id = 0;
static uint16_t g_queue_id = 0;
static uint32_t g_burst = DEFAULT_BURST_SIZE;
static uint32_t g_pkt_size = DEFAULT_PKT_SIZE;
static uint32_t g_seconds = DEFAULT_SECONDS;
static int g_mode_tx = 1; /* default TX */
static struct rte_ether_addr g_dst_mac;
static int g_have_dst_mac = 0;

struct perf_stats {
    uint64_t total_bytes;
    uint64_t total_packets;
    uint64_t start_tsc;
    uint64_t end_tsc;
};

static void usage(const char *prg)
{
    printf("Usage: %s [EAL args] -- [--tx|--rx] [--port N] [--queue Q] [--burst B] [--size S] [--seconds T] [--dst-mac xx:xx:xx:xx:xx:xx]\n", prg);
}

static int parse_mac(const char *s, struct rte_ether_addr *mac)
{
    unsigned int b[6];
    if (sscanf(s, "%x:%x:%x:%x:%x:%x", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6)
        return -1;
    for (int i = 0; i < 6; i++)
        mac->addr_bytes[i] = (uint8_t)b[i];
    return 0;
}

static void parse_app_args(int argc, char **argv)
{
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--tx") == 0) g_mode_tx = 1;
        else if (strcmp(argv[i], "--rx") == 0) g_mode_tx = 0;
        else if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) g_port_id = (uint16_t)atoi(argv[++i]);
        else if (strcmp(argv[i], "--queue") == 0 && i + 1 < argc) g_queue_id = (uint16_t)atoi(argv[++i]);
        else if (strcmp(argv[i], "--burst") == 0 && i + 1 < argc) g_burst = (uint32_t)atoi(argv[++i]);
        else if (strcmp(argv[i], "--size") == 0 && i + 1 < argc) g_pkt_size = (uint32_t)atoi(argv[++i]);
        else if (strcmp(argv[i], "--seconds") == 0 && i + 1 < argc) g_seconds = (uint32_t)atoi(argv[++i]);
        else if (strcmp(argv[i], "--dst-mac") == 0 && i + 1 < argc) {
            if (parse_mac(argv[++i], &g_dst_mac) == 0)
                g_have_dst_mac = 1;
        }
    }
}

static void print_stats(const char *title, struct perf_stats *s, uint32_t pkt_size)
{
    double hz = (double)rte_get_tsc_hz();
    double seconds = (double)(s->end_tsc - s->start_tsc) / hz;
    double mbps = (double)s->total_bytes * 8.0 / (seconds * 1e6);
    double pps = (double)s->total_packets / seconds;
    printf("\n===== %s =====\n", title);
    printf("Total packets: %" PRIu64 "\n", s->total_packets);
    printf("Total bytes:   %" PRIu64 "\n", s->total_bytes);
    printf("Duration:      %.6f s\n", seconds);
    printf("Bandwidth:     %.3f Mbps\n", mbps);
    printf("Throughput:    %.2f pkt/s\n", pps);
    printf("Pkt size:      %u bytes\n", pkt_size);
    printf("============================\n");
}

static int port_init(uint16_t port, struct rte_mempool *mp)
{
    struct rte_eth_conf port_conf = {0};
    uint16_t rx_rings = 1, tx_rings = 1;
    int ret;

    ret = rte_eth_dev_configure(port, rx_rings, tx_rings, &port_conf);
    if (ret < 0) return ret;

    uint16_t nb_rxd = 1024, nb_txd = 1024;
    ret = rte_eth_dev_adjust_nb_rx_tx_desc(port, &nb_rxd, &nb_txd);
    if (ret < 0) return ret;

    for (uint16_t q = 0; q < rx_rings; q++) {
        ret = rte_eth_rx_queue_setup(port, q, nb_rxd, rte_eth_dev_socket_id(port), NULL, mp);
        if (ret < 0) return ret;
    }
    for (uint16_t q = 0; q < tx_rings; q++) {
        ret = rte_eth_tx_queue_setup(port, q, nb_txd, rte_eth_dev_socket_id(port), NULL);
        if (ret < 0) return ret;
    }

    ret = rte_eth_dev_start(port);
    if (ret < 0) return ret;

    rte_eth_promiscuous_enable(port);
    return 0;
}

static void do_tx(struct rte_mempool *mp)
{
    struct perf_stats st = {0};
    struct rte_mbuf **bufs = NULL;
    uint32_t burst = g_burst > 1024 ? 1024 : g_burst;
    uint32_t frame_len = g_pkt_size < 64 ? 64 : g_pkt_size;

    bufs = malloc(sizeof(struct rte_mbuf *) * burst);
    if (!bufs) rte_exit(EXIT_FAILURE, "malloc failed\n");

    struct rte_ether_addr src_mac;
    rte_eth_macaddr_get(g_port_id, &src_mac);
    if (!g_have_dst_mac) memset(&g_dst_mac, 0xFF, sizeof(g_dst_mac));

    uint64_t start = rte_rdtsc();
    uint64_t deadline = start + (uint64_t)g_seconds * rte_get_tsc_hz();
    st.start_tsc = start;

    while (rte_rdtsc() < deadline) {
        if (rte_pktmbuf_alloc_bulk(mp, bufs, burst) != 0)
            continue;
        /* build frames */
        for (uint32_t i = 0; i < burst; i++) {
            struct rte_mbuf *m = bufs[i];
            struct rte_ether_hdr *eth = rte_pktmbuf_mtod(m, struct rte_ether_hdr *);
            rte_ether_addr_copy(&g_dst_mac, &eth->dst_addr);
            rte_ether_addr_copy(&src_mac, &eth->src_addr);
            eth->ether_type = rte_cpu_to_be_16(RTE_ETHER_TYPE_IPV4);
            /* ensure pkt data len */
            uint32_t payload = frame_len - sizeof(*eth);
            if (rte_pktmbuf_pkt_len(m) < frame_len) {
                if (rte_pktmbuf_append(m, payload) == NULL) {
                    rte_pktmbuf_free(m);
                    bufs[i] = NULL;
                    continue;
                }
            }
            rte_pktmbuf_data_len(m) = frame_len; /* best-effort set for accounting */
        }
        /* compact valid bufs */
        uint32_t valid = 0;
        for (uint32_t i = 0; i < burst; i++) if (bufs[i]) bufs[valid++] = bufs[i];
        if (valid == 0) continue;

        uint32_t sent = 0;
        while (sent < valid) {
            uint16_t n = rte_eth_tx_burst(g_port_id, g_queue_id, &bufs[sent], valid - sent);
            if (n == 0) break;
            sent += n;
        }
        for (uint32_t i = sent; i < valid; i++) rte_pktmbuf_free(bufs[i]);
        st.total_packets += sent;
        st.total_bytes += (uint64_t)sent * frame_len;
    }
    st.end_tsc = rte_rdtsc();
    print_stats("TX Throughput Statistics", &st, frame_len);
    free(bufs);
}

static void do_rx(void)
{
    struct perf_stats st = {0};
    struct rte_mbuf *bufs[1024];
    uint32_t burst = g_burst > 1024 ? 1024 : g_burst;

    uint64_t start = rte_rdtsc();
    uint64_t deadline = start + (uint64_t)g_seconds * rte_get_tsc_hz();
    st.start_tsc = start;

    while (rte_rdtsc() < deadline) {
        uint16_t nb = rte_eth_rx_burst(g_port_id, g_queue_id, bufs, burst);
        if (nb == 0) continue;
        for (uint16_t i = 0; i < nb; i++) {
            st.total_packets++;
            st.total_bytes += rte_pktmbuf_pkt_len(bufs[i]);
            rte_pktmbuf_free(bufs[i]);
        }
    }
    st.end_tsc = rte_rdtsc();
    print_stats("RX Throughput Statistics", &st, 0);
}

int main(int argc, char **argv)
{
    int ret = rte_eal_init(argc, argv);
    if (ret < 0) rte_exit(EXIT_FAILURE, "EAL init failed\n");

    /* adjust argv/argc for app args */
    argc -= ret;
    argv += ret;

    if (argc > 0 && strcmp(argv[0], "--") == 0) {
        argc--;
        argv++;
    }

    parse_app_args(argc, argv);

    uint16_t nb_ports = rte_eth_dev_count_avail();
    if (nb_ports == 0) rte_exit(EXIT_FAILURE, "No Ethernet ports - bye\n");
    if (g_port_id >= nb_ports) rte_exit(EXIT_FAILURE, "Invalid port id %u\n", g_port_id);

    struct rte_mempool *mp = rte_pktmbuf_pool_create("MBUF_POOL", NUM_MBUFS,
        MBUF_CACHE_SIZE, 0, RTE_MBUF_DEFAULT_BUF_SIZE, rte_socket_id());
    if (mp == NULL) rte_exit(EXIT_FAILURE, "Cannot create mbuf pool\n");

    if (port_init(g_port_id, mp) < 0) rte_exit(EXIT_FAILURE, "Cannot init port %u\n", g_port_id);

    printf("Starting %s test on port %u queue %u burst=%u size=%u s=%u\n",
        g_mode_tx ? "TX" : "RX", g_port_id, g_queue_id, g_burst, g_pkt_size, g_seconds);

    if (g_mode_tx)
        do_tx(mp);
    else
        do_rx();

    rte_eth_dev_stop(g_port_id);
    rte_eth_dev_close(g_port_id);
    rte_eal_cleanup();
    return 0;
}