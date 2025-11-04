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
static uint32_t g_print_interval_ms = 1000; /* printing interval in milliseconds */
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
static rte_atomic64_t g_total_packets;
static rte_atomic64_t g_total_bytes;
static volatile uint64_t g_start_tsc = 0;
static volatile uint64_t g_end_tsc = 0;
static rte_atomic64_t g_worker_done;

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
        else if (strcmp(argv[i], "--interval-ms") == 0 && i + 1 < argc) {
            g_print_interval_ms = (uint32_t)atoi(argv[++i]);
            if (g_print_interval_ms == 0) g_print_interval_ms = 1000;
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

/* print instantaneous stats for the last interval (delta values) */
static void print_instant_stats(const char *title, uint64_t delta_bytes, uint64_t delta_packets, double interval_s, uint32_t pkt_size)
{
    double mbps = interval_s > 0.0 ? (double)delta_bytes * 8.0 / (interval_s * 1e6) : 0.0;
    double pps = interval_s > 0.0 ? (double)delta_packets / interval_s : 0.0;
    printf("\n===== %s (instant) =====\n", title);
    printf("Bytes (this interval): %" PRIu64 "\n", delta_bytes);
    printf("Packets (this interval): %" PRIu64 "\n", delta_packets);
    printf("Interval:      %.6f s\n", interval_s);
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

    uint64_t tsc_hz = rte_get_tsc_hz();
    uint64_t now;
    uint64_t next_print = 0; /* set when transmission starts */
    uint64_t deadline = 0; /* set when transmission starts */
    int started = 0; /* becomes 1 when first tx_burst returns >0 */
    uint64_t last_print_tsc = 0;
    uint64_t last_total_packets = 0;
    uint64_t last_total_bytes = 0;
    st.start_tsc = 0;

    for (;;) {
        now = rte_rdtsc();
        if (g_stop) {
            /* print stats on Ctrl-C */
            st.end_tsc = rte_rdtsc();
            printf("\nReceived SIGINT, printing final TX stats:\n");
            print_stats("TX Throughput Statistics", &st, frame_len);
            break;
        }
        /* if transmission has started, handle per-second printing and deadline */
        if (started) {
            if (now >= next_print) {
                /* ensure we don't miss multiple seconds */
                next_print += tsc_hz * ((now - next_print) / tsc_hz + 1);
                printf("\033[2J\033[H"); /* clear screen and move cursor home */
                fflush(stdout);
                double interval_s = (double)(now - last_print_tsc) / (double)tsc_hz;
                uint64_t delta_packets = st.total_packets - last_total_packets;
                uint64_t delta_bytes = st.total_bytes - last_total_bytes;
                print_instant_stats("TX Live Statistics", delta_bytes, delta_packets, interval_s, frame_len);
                last_print_tsc = now;
                last_total_packets = st.total_packets;
                last_total_bytes = st.total_bytes;
            }
            if (now >= deadline)
                break;
        }

        if (rte_pktmbuf_alloc_bulk(mp, bufs, burst) != 0) {
            /* allocation failed; if not started, just spin and retry */
            continue;
        }
        /* build frames: append payload/headroom and populate header */
        for (uint32_t i = 0; i < burst; i++) {
            struct rte_mbuf *m = bufs[i];
            /* append the full frame length and get pointer to start */
            char *pkt = (char *)rte_pktmbuf_append(m, frame_len);
            if (pkt == NULL) {
                rte_pktmbuf_free(m);
                bufs[i] = NULL;
                continue;
            }
            /* populate ethernet header at packet start */
            struct rte_ether_hdr *eth = (struct rte_ether_hdr *)pkt;
            rte_ether_addr_copy(&g_dst_mac, &eth->dst_addr);
            rte_ether_addr_copy(&src_mac, &eth->src_addr);
            eth->ether_type = rte_cpu_to_be_16(RTE_ETHER_TYPE_IPV4);
        }
        /* compact valid bufs */
        uint32_t valid = 0;
        for (uint32_t i = 0; i < burst; i++) if (bufs[i]) bufs[valid++] = bufs[i];
        if (valid == 0) continue;

        uint32_t sent = 0;
        while (sent < valid) {
            uint16_t n = rte_eth_tx_burst(g_port_id, g_queue_id, &bufs[sent], valid - sent);
            if (n == 0)
                break;
            /* on first successful tx, mark start and initialize timers */
            if (!started) {
                uint64_t t = rte_rdtsc();
                started = 1;
                st.start_tsc = t;
                deadline = st.start_tsc + (uint64_t)g_seconds * tsc_hz;
                next_print = st.start_tsc + tsc_hz;
                last_print_tsc = st.start_tsc;
                last_total_packets = 0;
                last_total_bytes = 0;
            }
            /* count only after successful tx_burst */
            st.total_packets += n;
            st.total_bytes += (uint64_t)n * frame_len;
            /* update shared counters for multi-core printer if enabled */
            rte_atomic64_add(&g_total_packets, n);
            rte_atomic64_add(&g_total_bytes, (int64_t)((uint64_t)n * frame_len));
            /* set global start timestamp once */
            rte_atomic64_cmpset((volatile uint64_t *)&g_start_tsc, 0, st.start_tsc);
            sent += n;
        }
        for (uint32_t i = sent; i < valid; i++) rte_pktmbuf_free(bufs[i]);
    }
    if (!g_stop) {
        st.end_tsc = rte_rdtsc();
        printf("\033[2J\033[H");
        fflush(stdout);
        print_stats("TX Throughput Statistics", &st, frame_len);
    }
    free(bufs);
}

static void do_rx(void)
{
    struct perf_stats st = {0};
    struct rte_mbuf *bufs[1024];
    uint32_t burst = g_burst > 1024 ? 1024 : g_burst;

    uint64_t tsc_hz = rte_get_tsc_hz();
    uint64_t now;
    uint64_t next_print = 0; /* set when reception starts */
    uint64_t deadline = 0; /* set when reception starts */
    int started = 0; /* becomes 1 when first rx_burst returns >0 */
    st.start_tsc = 0;
    uint64_t last_print_tsc = 0;
    uint64_t last_total_packets = 0;
    uint64_t last_total_bytes = 0;

    for (;;) {
        now = rte_rdtsc();
        /* if reception has started, handle per-second printing and deadline */
        if (g_stop) {
            st.end_tsc = rte_rdtsc();
            printf("\nReceived SIGINT, printing final RX stats:\n");
            print_stats("RX Throughput Statistics", &st, 0);
            break;
        }
        if (started) {
            if (now >= next_print) {
                next_print += tsc_hz * ((now - next_print) / tsc_hz + 1);
                printf("\033[2J\033[H");
                fflush(stdout);
                double interval_s = (double)(now - last_print_tsc) / (double)tsc_hz;
                uint64_t delta_packets = st.total_packets - last_total_packets;
                uint64_t delta_bytes = st.total_bytes - last_total_bytes;
                print_instant_stats("RX Live Statistics", delta_bytes, delta_packets, interval_s, 0);
                last_print_tsc = now;
                last_total_packets = st.total_packets;
                last_total_bytes = st.total_bytes;
            }
            if (now >= deadline)
                break;
        }

        uint16_t nb = rte_eth_rx_burst(g_port_id, g_queue_id, bufs, burst);
        if (nb == 0) {
            /* no packets this iteration */
            continue;
        }
        /* on first successful rx, mark start and initialize timers */
        if (!started) {
            started = 1;
            st.start_tsc = now;
            deadline = st.start_tsc + (uint64_t)g_seconds * tsc_hz;
            next_print = st.start_tsc + tsc_hz;
            last_print_tsc = st.start_tsc;
            last_total_packets = 0;
            last_total_bytes = 0;
        }
        for (uint16_t i = 0; i < nb; i++) {
            uint64_t len = rte_pktmbuf_pkt_len(bufs[i]);
            st.total_packets++;
            st.total_bytes += len;
            /* update shared counters for multi-core printer if enabled */
            rte_atomic64_add(&g_total_packets, 1);
            rte_atomic64_add(&g_total_bytes, (int64_t)len);
            rte_pktmbuf_free(bufs[i]);
        }
    }
    if (!g_stop) {
        st.end_tsc = rte_rdtsc();
        printf("\033[2J\033[H");
        fflush(stdout);
        print_stats("RX Throughput Statistics", &st, 0);
    }
}

/* worker versions for multi-core mode (no printing) */
static int tx_worker(void *arg)
{
    struct rte_mempool *mp = arg;
    struct rte_mbuf **bufs = NULL;
    uint32_t burst = g_burst > 1024 ? 1024 : g_burst;
    uint32_t frame_len = g_pkt_size < 64 ? 64 : g_pkt_size;
    struct rte_ether_addr src_mac;

    bufs = malloc(sizeof(struct rte_mbuf *) * burst);
    if (!bufs) rte_exit(EXIT_FAILURE, "malloc failed\n");

    rte_eth_macaddr_get(g_port_id, &src_mac);
    if (!g_have_dst_mac) memset(&g_dst_mac, 0xFF, sizeof(g_dst_mac));

    uint64_t tsc_hz = rte_get_tsc_hz();
    uint64_t deadline = 0;
    int started = 0;

    while (!g_stop) {
        if (rte_pktmbuf_alloc_bulk(mp, bufs, burst) != 0)
            continue;
        for (uint32_t i = 0; i < burst; i++) {
            struct rte_mbuf *m = bufs[i];
            char *pkt = (char *)rte_pktmbuf_append(m, frame_len);
            if (pkt == NULL) {
                rte_pktmbuf_free(m);
                bufs[i] = NULL;
                continue;
            }
            struct rte_ether_hdr *eth = (struct rte_ether_hdr *)pkt;
            rte_ether_addr_copy(&g_dst_mac, &eth->dst_addr);
            rte_ether_addr_copy(&src_mac, &eth->src_addr);
            eth->ether_type = rte_cpu_to_be_16(RTE_ETHER_TYPE_IPV4);
        }
        uint32_t valid = 0;
        for (uint32_t i = 0; i < burst; i++) if (bufs[i]) bufs[valid++] = bufs[i];
        if (valid == 0) continue;

        uint32_t sent = 0;
        while (sent < valid) {
            uint16_t n = rte_eth_tx_burst(g_port_id, g_queue_id, &bufs[sent], valid - sent);
            if (n == 0) break;
            if (!started) {
                uint64_t t = rte_rdtsc();
                started = 1;
                rte_atomic64_cmpset((volatile uint64_t *)&g_start_tsc, 0, t);
                deadline = g_start_tsc + (uint64_t)g_seconds * tsc_hz;
            }
            rte_atomic64_add(&g_total_packets, n);
            rte_atomic64_add(&g_total_bytes, (int64_t)((uint64_t)n * frame_len));
            sent += n;
        }
        for (uint32_t i = sent; i < valid; i++) rte_pktmbuf_free(bufs[i]);
        if (started && rte_rdtsc() >= deadline)
            break;
    }
    g_end_tsc = rte_rdtsc();
    rte_atomic64_set(&g_worker_done, 1);
    free(bufs);
    return 0;
}

static int rx_worker(void *arg)
{
    (void)arg;
    struct rte_mbuf *bufs[1024];
    uint32_t burst = g_burst > 1024 ? 1024 : g_burst;
    uint64_t tsc_hz = rte_get_tsc_hz();
    uint64_t deadline = 0;
    int started = 0;

    while (!g_stop) {
        uint16_t nb = rte_eth_rx_burst(g_port_id, g_queue_id, bufs, burst);
        if (nb == 0) continue;
        if (!started) {
            uint64_t t = rte_rdtsc();
            started = 1;
            rte_atomic64_cmpset((volatile uint64_t *)&g_start_tsc, 0, t);
            deadline = g_start_tsc + (uint64_t)g_seconds * tsc_hz;
        }
        for (uint16_t i = 0; i < nb; i++) {
            uint64_t len = rte_pktmbuf_pkt_len(bufs[i]);
            rte_atomic64_add(&g_total_packets, 1);
            rte_atomic64_add(&g_total_bytes, (int64_t)len);
            rte_pktmbuf_free(bufs[i]);
        }
        if (started && rte_rdtsc() >= deadline)
            break;
    }
    g_end_tsc = rte_rdtsc();
    rte_atomic64_set(&g_worker_done, 1);
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

    printf("Starting %s test on port %u queue %u burst=%u size=%u s=%u\n",
        g_mode_tx ? "TX" : "RX", g_port_id, g_queue_id, g_burst, g_pkt_size, g_seconds);

    /* initialize shared atomics */
    rte_atomic64_init(&g_total_packets);
    rte_atomic64_init(&g_total_bytes);
    rte_atomic64_init(&g_worker_done);
    rte_atomic64_set(&g_total_packets, 0);
    rte_atomic64_set(&g_total_bytes, 0);
    rte_atomic64_set(&g_worker_done, 0);
    g_start_tsc = 0;
    g_end_tsc = 0;

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

        /* printer loop on main lcore */
        uint64_t hz = rte_get_tsc_hz();
        uint64_t interval_tsc = (uint64_t)g_print_interval_ms * hz / 1000ULL;
        uint64_t last_tsc = rte_rdtsc();
        uint64_t next_print = last_tsc + interval_tsc;
        uint64_t last_packets = 0, last_bytes = 0;
        double interval_s = (double)g_print_interval_ms / 1000.0;

        /* printer loop: prints every g_print_interval_ms regardless of worker activity */
        while (!g_stop && rte_atomic64_read(&g_worker_done) == 0) {
            uint64_t now = rte_rdtsc();
            if (now >= next_print) {
                uint64_t total_packets = (uint64_t)rte_atomic64_read(&g_total_packets);
                uint64_t total_bytes = (uint64_t)rte_atomic64_read(&g_total_bytes);
                uint64_t delta_packets = total_packets - last_packets;
                uint64_t delta_bytes = total_bytes - last_bytes;
                printf("\033[2J\033[H");
                fflush(stdout);
                print_instant_stats(g_mode_tx ? "TX Live Statistics" : "RX Live Statistics",
                                   delta_bytes, delta_packets, interval_s, g_mode_tx ? g_pkt_size : 0);
                last_tsc = now;
                last_packets = total_packets;
                last_bytes = total_bytes;
                next_print += interval_tsc;
            }
            rte_delay_us_sleep(1000);
        }

        /* wait for worker to finish if not already */
        rte_eal_wait_lcore(worker_lcore);

        /* final summary print */
        struct perf_stats s = {0};
        s.total_packets = (uint64_t)rte_atomic64_read(&g_total_packets);
        s.total_bytes = (uint64_t)rte_atomic64_read(&g_total_bytes);
        s.start_tsc = g_start_tsc ? g_start_tsc : rte_rdtsc();
        s.end_tsc = g_end_tsc ? g_end_tsc : rte_rdtsc();
        print_stats(g_mode_tx ? "TX Throughput Statistics" : "RX Throughput Statistics", &s, g_mode_tx ? g_pkt_size : 0);
    } else {
        /* single-core: run in current thread (existing behaviour) */
        if (g_mode_tx)
            do_tx(mp);
        else
            do_rx();
    }

    rte_eth_dev_stop(g_port_id);
    rte_eth_dev_close(g_port_id);
    rte_eal_cleanup();
    return 0;
}