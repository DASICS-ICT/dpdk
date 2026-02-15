// simpleperf: DPDK-25 ethdev bandwidth tester (clean version)

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_mempool.h>
#include <rte_mbuf.h>
#include <rte_cycles.h>
#include <rte_ether.h>
#include <rte_random.h>
#include <rte_pause.h>
#include <signal.h>
#include <rte_atomic.h>
#include <rte_ring.h>

#ifdef RTE_ENABLE_DBCHECKER
    #include <rte_dbchecker.h>
#endif

//#define TEST_ACT_CPUTIME

#define DEFAULT_NUM_MBUFS 4096
#define MBUF_CACHE_SIZE 256
#define DEFAULT_BURST_SIZE 32
#define DEFAULT_PKT_SIZE 64
#define DEFAULT_SECONDS 3600
#define DEV_ID 0x0U

enum traffic_profile {
    PROFILE_CONST = 0,
    PROFILE_POISSON,
    PROFILE_ONOFF
};

struct size_profile {
    int bimodal_enabled;
    uint32_t small_sz;
    uint32_t big_sz;
    double prob_big; /* 0.0-1.0 */
};

struct onoff_profile {
    uint64_t on_cycles;
    uint64_t off_cycles;
    int on; /* 1:on, 0:off */
    uint64_t state_end_tsc;
};

struct token_bucket {
    double tokens;       /* bytes currently available */
    double bytes_per_tsc;/* bytes generated per TSC */
    double capacity;     /* max bucket size in bytes */
    uint64_t last_tsc;
};

static uint16_t g_port_id = 0;
static uint16_t g_queue_id = 0;
static uint32_t g_burst = DEFAULT_BURST_SIZE;
static uint32_t g_pkt_size = DEFAULT_PKT_SIZE;
static uint32_t g_seconds = DEFAULT_SECONDS;
static int g_mode_tx = 1; /* default TX */
static struct rte_ether_addr g_dst_mac;
static int g_have_dst_mac = 0;
static uint32_t g_num_mbufs = DEFAULT_NUM_MBUFS; 
static uint64_t g_activate_cpu_time = 0;
static enum traffic_profile g_profile = PROFILE_CONST;
static double g_rate_bps = 0.0; /* byte-based rate limit (B/s), 0 => unlimited */
static struct size_profile g_size_prof = {0, DEFAULT_PKT_SIZE, DEFAULT_PKT_SIZE, 0.0};
static struct onoff_profile g_onoff = {0, 0, 1, 0};
static uint32_t g_seed = 1;
static uint8_t g_eth_hdr_template[sizeof(struct rte_ether_hdr)];
static uint64_t g_rng_state = 1;

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

static void usage(const char *prg)
{
    printf("Usage: %s [EAL args] -- [--tx|--rx] [--port N] [--queue Q] [--burst B] [--size S] [--seconds T] [--mbufs N]\n", prg);
    printf("        [--dst-mac xx:xx:xx:xx:xx:xx] [--profile const|poisson|onoff] [--rate-bps N]\n");
    printf("        [--size-bimodal small,big,prob] [--onoff on_ms,off_ms] [--seed N]\n");
    printf("  --mbufs N: Set number of mbufs (range: 4096-65536, default: %d)\n", DEFAULT_NUM_MBUFS);
    printf("  --profile: Traffic pattern. const (default), poisson (random IAT), onoff (burst/silent)\n");
    printf("  --rate-bps: Target bytes per second (0=unlimited)\n");
    printf("  --size-bimodal: Enable two-size mix, e.g. 64,1500,0.3 (30%% large)\n");
    printf("  --onoff: On/off durations in ms, e.g. 200,100\n");
    printf("  --seed: RNG seed for reproducibility\n");
}

static volatile sig_atomic_t g_stop;

static void
signal_handler(int signum)
{
    (void)signum;
    g_stop = 1;
}

static inline uint64_t fast_rand64(void)
{
    /* xorshift64* (single-core use) */
    uint64_t x = g_rng_state;
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    g_rng_state = x;
    return x * 0x2545F4914F6CDD1DULL;
}

static inline double rand_uniform(void)
{
    uint64_t r = fast_rand64();
    return r / (double)UINT64_MAX;
}

static inline uint64_t sample_exp_cycles(double lambda, double tsc_hz)
{
    /* lambda in events per second; returns cycles until next event */
    double u = rand_uniform();
    if (u < 1e-12) u = 1e-12; /* avoid log(0) */
    double iat = -log(u) / lambda; /* seconds */
    return (uint64_t)(iat * tsc_hz);
}

static inline void tb_init(struct token_bucket *tb, double rate_bps, double tsc_hz)
{
    double rate_Bps = rate_bps; /* bytes per second */
    tb->tokens = rate_Bps;
    tb->capacity = rate_Bps; /* allow burst of ~1s worth of traffic */
    tb->bytes_per_tsc = rate_Bps / tsc_hz;
    tb->last_tsc = rte_rdtsc();
}

static inline void tb_refill(struct token_bucket *tb, uint64_t now)
{
    uint64_t dt = now - tb->last_tsc;
    double add = dt * tb->bytes_per_tsc;
    tb->tokens = tb->tokens + add;
    if (tb->tokens > tb->capacity) tb->tokens = tb->capacity;
    tb->last_tsc = now;
}

static inline void tb_wait(struct token_bucket *tb, double need_tokens)
{
    if (tb->bytes_per_tsc <= 0.0) return;
    uint64_t now = rte_rdtsc();
    tb_refill(tb, now);
    while (tb->tokens < need_tokens) {
        rte_pause();
        now = rte_rdtsc();
        tb_refill(tb, now);
    }
    tb->tokens -= need_tokens;
}

static uint32_t pick_pkt_size(void)
{
    if (!g_size_prof.bimodal_enabled)
        return g_pkt_size;
    double u = rand_uniform();
    if (u < g_size_prof.prob_big)
        return g_size_prof.big_sz;
    return g_size_prof.small_sz;
}

static inline double expected_frame_len(void)
{
    /* include min Ethernet frame length of 64B (no preamble/IFG accounted) */
    /* return bits instead of bytes */
    if (!g_size_prof.bimodal_enabled) {
        uint32_t len = (g_pkt_size < 64 ? 64 : g_pkt_size) << 3;
        return (double)len;
    }

    double small = (double)((g_size_prof.small_sz < 64 ? 64 : g_size_prof.small_sz) << 3);
    double big = (double)((g_size_prof.big_sz < 64 ? 64 : g_size_prof.big_sz) << 3);
    return (double)(small * (1.0 - g_size_prof.prob_big) + big * g_size_prof.prob_big);
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
        else if (strcmp(argv[i], "--mbufs") == 0 && i + 1 < argc) {
            int val = atoi(argv[++i]);
            if (val < 4096 || val > 65536) {
                rte_exit(EXIT_FAILURE, "Invalid mbufs count: %d. Range must be 4096-65536\n", val);
            }
            g_num_mbufs = (uint32_t)val;
        }
        else if (strcmp(argv[i], "--profile") == 0 && i + 1 < argc) {
            const char *p = argv[++i];
            if (strcmp(p, "const") == 0) g_profile = PROFILE_CONST;
            else if (strcmp(p, "poisson") == 0) g_profile = PROFILE_POISSON;
            else if (strcmp(p, "onoff") == 0) g_profile = PROFILE_ONOFF;
            else rte_exit(EXIT_FAILURE, "Invalid profile: %s\n", p);
        }
        else if (strcmp(argv[i], "--rate-bps") == 0 && i + 1 < argc) {
            g_rate_bps = atof(argv[++i]);
            if (g_rate_bps < 0.0) g_rate_bps = 0.0;
        }
        else if (strcmp(argv[i], "--size-bimodal") == 0 && i + 1 < argc) {
            const char *s = argv[++i];
            /* format: small,big,prob */
            int small = 0, big = 0;
            double prob = 0.0;
            if (sscanf(s, "%d,%d,%lf", &small, &big, &prob) != 3)
                rte_exit(EXIT_FAILURE, "Invalid --size-bimodal format (expect small,big,prob)\n");
            if (small <= 0 || big <= 0 || prob < 0.0 || prob > 1.0)
                rte_exit(EXIT_FAILURE, "Invalid --size-bimodal values\n");
            g_size_prof.bimodal_enabled = 1;
            g_size_prof.small_sz = (uint32_t)small;
            g_size_prof.big_sz = (uint32_t)big;
            g_size_prof.prob_big = prob;
        }
        else if (strcmp(argv[i], "--onoff") == 0 && i + 1 < argc) {
            const char *s = argv[++i];
            int on_ms = 0, off_ms = 0;
            if (sscanf(s, "%d,%d", &on_ms, &off_ms) != 2)
                rte_exit(EXIT_FAILURE, "Invalid --onoff format (expect on_ms,off_ms)\n");
            if (on_ms < 0 || off_ms < 0)
                rte_exit(EXIT_FAILURE, "Invalid --onoff values\n");
            g_onoff.on_cycles = (uint64_t)on_ms * rte_get_tsc_hz() / 1000ULL;
            g_onoff.off_cycles = (uint64_t)off_ms * rte_get_tsc_hz() / 1000ULL;
        }
        else if (strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
            g_seed = (uint32_t)atoi(argv[++i]);
        }
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

static void print_stats(uint16_t port, struct perf_stats *s)
{
    struct rte_eth_stats rs;
    int rc = rte_eth_stats_get(port, &rs);
    if (rc < 0) {
        printf("[port %u]: rte_eth_stats_get failed: %s (%d)\n",
               port, rte_strerror(-rc), rc);
        return;
    }
    double hz = (double)rte_get_tsc_hz();
    double seconds = (double)(s->end_tsc - s->start_tsc) / hz;
    double mbps = (double)(rs.ibytes + rs.obytes) * 8.0 / (seconds * 1e6);
    double pps = (double)(rs.ipackets + rs.opackets) / seconds;

    printf("ipackets=%" PRIu64 "  ibytes=%" PRIu64 "\n", rs.ipackets, rs.ibytes);
    printf("opackets=%" PRIu64 "  obytes=%" PRIu64 "\n", rs.opackets, rs.obytes);
    printf("ierrors=%" PRIu64 "   oerrors=%" PRIu64 "\n", rs.ierrors, rs.oerrors);
    printf("imissed=%" PRIu64 "\n", rs.imissed);
    printf("Duration:      %.6f s\n", seconds);
    printf("Activate CPU Time: %.6f s\n", (double)g_activate_cpu_time / hz);
    printf("Bandwidth:     %.3f Mbps\n", mbps);
    printf("Throughput:    %.2f pkt/s\n", pps);
    if (g_size_prof.bimodal_enabled) {
        printf("Pkt size mix:  %uB @%.2f  %uB @%.2f\n",
               g_size_prof.small_sz, 1.0 - g_size_prof.prob_big,
               g_size_prof.big_sz, g_size_prof.prob_big);
    } else {
        printf("Pkt size:      %u bytes\n", g_pkt_size);
    }
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
    rte_eth_stats_reset(port);
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

    double tsc_hz = (double)rte_get_tsc_hz();
    uint64_t deadline = 0;
    int rv = 0;
    struct token_bucket tb = {0};
    if (g_profile == PROFILE_CONST && g_rate_bps > 0.0)
        tb_init(&tb, g_rate_bps, tsc_hz);

    uint64_t t = rte_rdtsc();
    if (g_start_tsc == 0) g_start_tsc = t;
    deadline = g_start_tsc + (uint64_t)g_seconds * (uint64_t)tsc_hz;

    uint64_t next_event_tsc = g_start_tsc;
    double lambda_pps = 0.0;
    if (g_profile == PROFILE_POISSON && g_rate_bps > 0.0) {
        double avg_len = expected_frame_len();
        if (avg_len > 0.0)
            lambda_pps = g_rate_bps / avg_len; /* convert B/s to pps using expected frame length */
        if (lambda_pps > 0.0)
            next_event_tsc = g_start_tsc + sample_exp_cycles(lambda_pps, tsc_hz);
    }

    if (g_profile == PROFILE_ONOFF) {
        g_onoff.on = 1;
        if (g_onoff.on_cycles > 0)
            g_onoff.state_end_tsc = g_start_tsc + g_onoff.on_cycles;
    }

    while (!g_stop) {
        uint64_t now = rte_rdtsc();
        uint32_t send_cnt = burst;

        /* on-off gating */
        if (g_profile == PROFILE_ONOFF && (g_onoff.on_cycles || g_onoff.off_cycles)) {
            if (g_onoff.on && g_onoff.on_cycles > 0 && now >= g_onoff.state_end_tsc) {
                g_onoff.on = 0;
                if (g_onoff.off_cycles > 0)
                    g_onoff.state_end_tsc = now + g_onoff.off_cycles;
            } else if (!g_onoff.on && g_onoff.off_cycles > 0 && now >= g_onoff.state_end_tsc) {
                g_onoff.on = 1;
                if (g_onoff.on_cycles > 0)
                    g_onoff.state_end_tsc = now + g_onoff.on_cycles;
            }
            if (!g_onoff.on) {
                rte_pause();
                if (now >= deadline) break;
                continue;
            }
        }

        if (g_profile == PROFILE_POISSON && lambda_pps > 0.0) {
            if (now < next_event_tsc) {
                rte_pause();
                if (now >= deadline) break;
                continue;
            }
            /* accumulate how many Poisson events have arrived; cap by burst */
            uint32_t due = 0;
            while (due < burst && now >= next_event_tsc) {
                due++;
                next_event_tsc += sample_exp_cycles(lambda_pps, tsc_hz);
            }
            if (due == 0) {
                if (now >= deadline) break;
                continue;
            }
            send_cnt = due;
        }

        do {
            rv = rte_pktmbuf_alloc_bulk(g_mp, bufs, send_cnt);
        } while (rv != 0 && !g_stop);
        if (rv != 0) break;

        uint32_t valid = 0;
        uint32_t bad = 0;
        uint64_t batch_bytes = 0;
        for (uint32_t i = 0; i < send_cnt; i++) {
            uint32_t pkt_sz = pick_pkt_size();
            uint32_t frame_len = pkt_sz < 64 ? 64 : pkt_sz;
            struct rte_mbuf *m = bufs[i];
            char *pkt = (char *)rte_pktmbuf_append(m, frame_len);
            if (pkt == NULL) {
                bad_bufs[bad++] = m;
                continue;
            }
            rte_memcpy(pkt, g_eth_hdr_template, sizeof(struct rte_ether_hdr));
            bufs[valid++] = m;
            batch_bytes += (uint64_t)frame_len;
        }
        if (bad > 0) rte_pktmbuf_free_bulk(bad_bufs, bad);
        if (valid == 0) {
            if (now >= deadline) break;
            continue;
        }

        if (g_profile == PROFILE_CONST && g_rate_bps > 0.0)
            tb_wait(&tb, (double)batch_bytes);

        #ifdef RTE_ENABLE_DBCHECKER
            #ifdef TEST_ACT_CPUTIME
                uint64_t start = rte_rdtsc();
            #endif
            for (uint16_t i = 0; i < valid; i++) {
                dbchecker_activate_mtdt_hook(bufs[i], DMA_TO_DEVICE, DEV_ID, false);
            }
            #ifdef TEST_ACT_CPUTIME
                g_activate_cpu_time += (rte_rdtsc() - start);
            #endif
        #endif
        uint16_t sent = 0;
        while (sent < (uint16_t)valid) {
            uint16_t n = rte_eth_tx_burst(g_port_id, g_queue_id, &bufs[sent], valid - sent);
            if (n == 0) continue;
            sent += n;
        }

        if (rte_rdtsc() >= deadline)
            break;
    }
    g_end_tsc = rte_rdtsc();
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

    uint64_t t = rte_rdtsc();
    if (g_start_tsc == 0) g_start_tsc = t;
    deadline = g_start_tsc + (uint64_t)g_seconds * tsc_hz;

    while (!g_stop) {
        uint16_t nb = rte_eth_rx_burst(g_port_id, g_queue_id, bufs, burst);
        if (nb == 0) {
            goto rx_round_done;
        }

        /* free received mbufs in bulk */
        rte_pktmbuf_free_bulk(bufs, nb);
        #ifdef RTE_ENABLE_DBCHECKER
            for (uint16_t i = 0; i < nb; i++) {
                dbchecker_deactivate_mtdt_hook(bufs[i]);
            }
        #endif
rx_round_done:
        if (rte_rdtsc() >= deadline)
            break;
    }

    g_end_tsc = rte_rdtsc();
    /* publish RX counters to globals (no atomics) */
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

    rte_srand(g_seed);
    g_rng_state = g_seed ? g_seed : 1ULL;

    /* install signal handler to allow printing stats on Ctrl-C */
    g_stop = 0;
    signal(SIGINT, signal_handler);

    uint16_t nb_ports = rte_eth_dev_count_avail();
    if (nb_ports == 0) rte_exit(EXIT_FAILURE, "No Ethernet ports - bye\n");
    if (g_port_id >= nb_ports) rte_exit(EXIT_FAILURE, "Invalid port id %u\n", g_port_id);

    #ifdef RTE_ENABLE_DBCHECKER
        dbchecker_module_init_hook();
    #endif

    printf("Creating mbuf pool with %u mbufs...\n", g_num_mbufs);
    struct rte_mempool *mp = rte_pktmbuf_pool_create("MBUF_POOL", g_num_mbufs,
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
            if (!rte_eth_link_get_nowait(g_port_id, &link) && link.link_status == RTE_ETH_LINK_UP) {
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

    printf("Starting %s test on port %u queue %u burst=%u size=%u mbufs=%u s=%u... \n",
        g_mode_tx ? "TX" : "RX", g_port_id, g_queue_id, g_burst, g_pkt_size, g_num_mbufs, g_seconds);

    /* initialize shared counters (published by worker at end) */
    g_worker_done = 0;
    g_start_tsc = 0;
    g_end_tsc = 0;
    /* single-core mode: run worker loop directly on main core */
    if (g_mode_tx) {
        struct rte_ether_addr src_mac;
        rte_eth_macaddr_get(g_port_id, &src_mac);
        if (!g_have_dst_mac) memset(&g_dst_mac, 0xFF, sizeof(g_dst_mac));
        struct rte_ether_hdr *eth = (struct rte_ether_hdr *)g_eth_hdr_template;
        rte_ether_addr_copy(&g_dst_mac, &eth->dst_addr);
        rte_ether_addr_copy(&src_mac, &eth->src_addr);
        eth->ether_type = rte_cpu_to_be_16(RTE_ETHER_TYPE_IPV4);

        /* call tx_worker directly on the main core */
        tx_worker(NULL);
    } else {
        /* RX mode: perform rx_worker logic on main core */
        rx_worker(NULL);
    }

    /* final summary print */
    struct perf_stats s = {0};
    s.start_tsc = g_start_tsc ? g_start_tsc : rte_rdtsc();
    s.end_tsc = g_end_tsc ? g_end_tsc : rte_rdtsc();
    printf("\n==== %s ETH stats port=%u ====\n", g_mode_tx ? "TX" : "RX", g_port_id);
    print_stats(g_port_id, &s);
    #ifdef RTE_ENABLE_DBCHECKER
        dbchecker_err_handler();
        dbchecker_module_exit_hook();
    #endif
    rte_eth_dev_stop(g_port_id);
    rte_eth_dev_close(g_port_id);
    rte_eal_cleanup();
    return 0;
}