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
#include <signal.h>
#include <rte_atomic.h>
#include <rte_ring.h>

extern int dbchecker_err_handler(void) __attribute__((weak));

#define NUM_MBUFS 16384
#define MBUF_CACHE_SIZE 250
#define DEFAULT_BURST_SIZE 64
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

/* shared stats for multi-core mode */
static volatile uint64_t g_total_packets = 0;
static volatile uint64_t g_total_bytes = 0;
static volatile uint64_t g_start_tsc = 0;
static volatile uint64_t g_end_tsc = 0;
static volatile int g_worker_done = 0;
static struct rte_mempool *g_mp = NULL; /* global mempool for main core */
/* packet template prepared by main to minimize per-packet construction in tx_worker */
static uint8_t *g_template = NULL;
static uint32_t g_frame_len = 0;


static void usage(const char *prg)
{
    printf("Usage: %s [EAL args] -- [--tx|--rx] [--port N] [--queue Q] [--burst B] [--size S] [--seconds T] [--interval-ms T] [--dst-mac xx:xx:xx:xx:xx:xx]\n", prg);
}

static volatile sig_atomic_t g_stop;

static void
signal_handler(int signum)
{
    (void)signum;
    g_stop = 1;
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
        else {
            usage(argv[0]);
            rte_exit(EXIT_FAILURE, "Invalid argument: %s\n", argv[i]);
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

    uint16_t nb_rxd = 512, nb_txd = 512;
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

/* worker versions for multi-core mode (no printing) */
static int tx_worker(void *arg)
{
    (void)arg;
    struct rte_mbuf *bufs[512];
    struct rte_mbuf *bad_bufs[512];
    uint32_t burst = g_burst > 512 ? 512 : g_burst;

    uint64_t tsc_hz = rte_get_tsc_hz();
    uint64_t deadline = 0;
    int started = 0;
    uint64_t local_packets = 0;
    uint64_t local_bytes = 0;

    /* tx_worker now performs mbuf alloc/fill/send/free locally to preserve
     * single-threaded alloc/free ownership for mbufs. The main thread only
     * prepares a template buffer (`g_template`) to minimize per-packet work.
     */
    while (!g_stop) {
        /* try to allocate a bulk of mbufs; on failure briefly sleep */
        if (rte_pktmbuf_alloc_bulk(g_mp, bufs, burst) != 0) {
            continue;
        }

        /* fill payload from template; track any failure to append */
        uint32_t valid = 0;
        uint32_t bad = 0;
        for (uint32_t i = 0; i < burst; i++) {
            struct rte_mbuf *m = bufs[i];
            char *pkt = (char *)rte_pktmbuf_append(m, g_frame_len);
            if (pkt == NULL) {
                bad_bufs[bad++] = m;
                continue;
            }
            /* copy prebuilt template (ethernet header + payload) */
            rte_memcpy(pkt, g_template, g_frame_len);
            bufs[valid++] = m;
        }

        if (valid == 0) continue;

        /* send as many as possible; tx_burst may return partial sends */
        uint16_t sent = 0;
        while (sent < (uint16_t)valid) {
            uint16_t n = rte_eth_tx_burst(g_port_id, g_queue_id, &bufs[sent], (uint16_t)(valid - sent));
            if (n == 0) break;
            if (!started) {
                uint64_t t = rte_rdtsc();
                started = 1;
                if (g_start_tsc == 0) g_start_tsc = t;
                deadline = g_start_tsc + (uint64_t)g_seconds * tsc_hz;
            }
            sent += n;
            local_packets += n;
            local_bytes += (uint64_t)n * g_frame_len;
        }


        /* free any mbufs that failed to be appended */
        if (bad > 0) rte_pktmbuf_free_bulk(bad_bufs, bad);
        /* free any unsent mbufs locally (tx_worker owns alloc/free) */
        if (sent < (uint16_t)valid) {
            rte_pktmbuf_free_bulk(&bufs[sent], (uint16_t)(valid - sent));
        }

        if (started && rte_rdtsc() >= deadline)
            break;
    }
    g_end_tsc = rte_rdtsc();
    /* publish local counters to globals once (no atomics) */
    g_total_packets = local_packets;
    g_total_bytes = local_bytes;
    g_worker_done = 1;
    return 0;
}

static int rx_worker(void *arg)
{
    (void)arg;
    struct rte_mbuf *bufs[512];
    uint32_t burst = g_burst > 512 ? 512 : g_burst;
    uint64_t tsc_hz = rte_get_tsc_hz();
    uint64_t deadline = 0;
    int started = 0;
    uint64_t local_packets = 0;
    uint64_t local_bytes = 0;

    while (!g_stop) {
        uint16_t nb = rte_eth_rx_burst(g_port_id, g_queue_id, bufs, burst);
        if (nb == 0) {
            continue;
        }
        if (!started) {
            uint64_t t = rte_rdtsc();
            started = 1;
            if (g_start_tsc == 0) g_start_tsc = t;
            deadline = g_start_tsc + (uint64_t)g_seconds * tsc_hz;
        }
        /* count and free received mbufs locally to ensure RX-side alloc/free on same thread */
        uint64_t totlen = 0;
        for (uint16_t i = 0; i < nb; i++) totlen += rte_pktmbuf_pkt_len(bufs[i]);
        local_packets += nb;
        local_bytes += totlen;
        /* free received mbufs in bulk */
        rte_pktmbuf_free_bulk(bufs, nb);
        if (started && rte_rdtsc() >= deadline)
            break;
    }
    g_end_tsc = rte_rdtsc();
    /* publish RX counters to globals (no atomics) */
    g_total_packets = local_packets;
    g_total_bytes = local_bytes;
    g_worker_done = 1;
    return 0;
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

    /* install signal handler to allow printing stats on Ctrl-C */
    g_stop = 0;
    signal(SIGINT, signal_handler);

    uint16_t nb_ports = rte_eth_dev_count_avail();
    if (nb_ports == 0) rte_exit(EXIT_FAILURE, "No Ethernet ports - bye\n");
    if (g_port_id >= nb_ports) rte_exit(EXIT_FAILURE, "Invalid port id %u\n", g_port_id);

    struct rte_mempool *mp = rte_pktmbuf_pool_create("MBUF_POOL", NUM_MBUFS,
        MBUF_CACHE_SIZE, 0, RTE_MBUF_DEFAULT_BUF_SIZE, rte_socket_id());
    if (mp == NULL) rte_exit(EXIT_FAILURE, "Cannot create mbuf pool\n");

    /* store global mempool; rings removed since tx_worker now allocs/frees mbufs */
    g_mp = mp;

    if (port_init(g_port_id, mp) < 0) rte_exit(EXIT_FAILURE, "Cannot init port %u\n", g_port_id);

    /* wait for link up (try up to 30 seconds) to increase chance first tx_burst succeeds) */
    {
        struct rte_eth_link link;
        unsigned int wait_secs = 0;
        memset(&link, 0, sizeof(link));
        printf("Waiting for link to come up on port %u...\n", g_port_id);
        while (wait_secs < 30) {
            rte_eth_link_get_nowait(g_port_id, &link);
            if (link.link_status == RTE_ETH_LINK_UP) {
                printf("Port %u Link Up - speed %u Mbps - %s\n",
                       g_port_id, (unsigned)link.link_speed,
                       (link.link_duplex == RTE_ETH_LINK_FULL_DUPLEX) ? "full-duplex" : "half-duplex");
                break;
            }
            sleep(1);
            wait_secs++;
        }
        if (link.link_status != RTE_ETH_LINK_UP)
            printf("Port %u: link not up after %u seconds, continuing anyway\n", g_port_id, wait_secs);
    }

    printf("Starting %s test on port %u queue %u burst=%u size=%u s=%u...  ",
        g_mode_tx ? "TX" : "RX", g_port_id, g_queue_id, g_burst, g_pkt_size, g_seconds);

    /* initialize shared counters (published by worker at end) */
    g_total_packets = 0;
    g_total_bytes = 0;
    g_worker_done = 0;
    g_start_tsc = 0;
    g_end_tsc = 0;
    const char spinner[] = "|/-\\";
    size_t spinner_idx = 0;
    unsigned int lcore_count = rte_lcore_count();
    if (lcore_count >= 2) {
        /* multi-core: launch worker on a secondary lcore and use main core as printer */
        unsigned int worker_lcore = rte_get_next_lcore(-1, 1, 0);
        int err;
        if (g_mode_tx)
            err = rte_eal_remote_launch(tx_worker, mp, worker_lcore);
        else
            err = rte_eal_remote_launch(rx_worker, mp, worker_lcore);
        if (err) rte_exit(EXIT_FAILURE, "Failed to launch worker on lcore %u\n", worker_lcore);

        struct rte_ether_addr src_mac;
        if (g_mode_tx) {
            rte_eth_macaddr_get(g_port_id, &src_mac);
            /* prepare a template packet (ether header + zeroed payload) so
             * tx_worker can quickly memcpy it into newly allocated mbufs.
             */
            g_frame_len = g_pkt_size < 64 ? 64 : g_pkt_size;
            g_template = malloc(g_frame_len);
            if (g_template == NULL) rte_exit(EXIT_FAILURE, "Failed to allocate template buffer\n");
            if (!g_have_dst_mac) memset(&g_dst_mac, 0xFF, sizeof(g_dst_mac));
            struct rte_ether_hdr *eth = (struct rte_ether_hdr *)g_template;
            rte_ether_addr_copy(&g_dst_mac, &eth->dst_addr);
            rte_ether_addr_copy(&src_mac, &eth->src_addr);
            eth->ether_type = rte_cpu_to_be_16(RTE_ETHER_TYPE_IPV4);
            if (g_frame_len > sizeof(struct rte_ether_hdr))
                memset(g_template + sizeof(struct rte_ether_hdr), 0, g_frame_len - sizeof(struct rte_ether_hdr));
        }

        /* Lightweight main loop: produce TX mbufs (if TX) and show a spinner
         * to indicate liveness. Avoid per-interval expensive computations and atomics.
         */
        while (!g_stop && g_worker_done == 0) {
                /* (RX mode) rx_worker frees its own mbufs; tx_worker now handles alloc/fill/send/free */
                /* only overwrite last char */
                putchar('\b');
                putchar(spinner[spinner_idx % (sizeof(spinner)-1)]);
                fflush(stdout);
                spinner_idx++;
        }

        /* wait for worker to finish if not already */
        rte_eal_wait_lcore(worker_lcore);
        if (dbchecker_err_handler) dbchecker_err_handler();
        /* no ring draining needed: tx_worker frees unsent mbufs locally */
        /* rx_worker performed counting and freeing itself; no remaining rx_ring drain needed */

        /* final summary print */
        struct perf_stats s = {0};
        s.total_packets = (uint64_t)g_total_packets;
        s.total_bytes = (uint64_t)g_total_bytes;
        s.start_tsc = g_start_tsc ? g_start_tsc : rte_rdtsc();
        s.end_tsc = g_end_tsc ? g_end_tsc : rte_rdtsc();
        /* clear spinner char and print final stats */
        printf("\r ");
        printf("\n");
        print_stats(g_mode_tx ? "TX Throughput Statistics" : "RX Throughput Statistics", &s, g_mode_tx ? g_pkt_size : 0);
        if (g_template) {
            free(g_template);
            g_template = NULL;
        }
    }
    else printf("Not enough lcores for multi-core mode (need at least 2)\n");
    
    rte_eth_dev_stop(g_port_id);
    rte_eth_dev_close(g_port_id);
    rte_eal_cleanup();
    return 0;
}