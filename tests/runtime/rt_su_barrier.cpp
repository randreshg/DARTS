/*
 * darts-rt-su-barrier -- a whole-machine barrier under NUMA_PAIRED with
 * SIBLING_THEN_ANY stealing and background node/shared-pool traffic.
 *
 * Every round is one Runtime::run() of a RootTP whose parent is a
 * long-lived sentinel; the RootTP place<>s one HostTP per SU, and each
 * HostTP adds one unplaced codelet per worker of its SU to its own SU's
 * codelet queue, so the SU's workers (TP scheduler + micro schedulers) take
 * exactly one each and all of them meet in one spin barrier. Scoped
 * stealing must never draw from another SU's codelet queue or placed
 * closures, or a worker would take a second barrier codelet and the barrier
 * would deadlock (watchdog, exit 3).
 *
 * Background traffic: every barrier codelet, before it enters the barrier,
 * adds one node-scoped codelet (setPlacedNode) and one shared codelet
 * (setPlacedShared); those compete with the next round's barrier codelets.
 *
 *   A  200 rounds: every worker is a barrier participant every round, each
 *      barrier codelet on its HostTP's SU
 *   B  background: every node codelet fired on its node and every shared
 *      codelet on {su, sibling(su)}; all drained after the last round
 *   C  placedSteals() == 0
 *   D  ~Runtime with nothing outstanding
 *
 * Exit 3 on a barrier deadlock; skipped (exit 4) unless the host has two LLC
 * clusters per NUMA node.
 */
#include "rt_test_util.h"

#include <cstring>
#include <time.h>

using namespace darts;
using namespace rt_test;

enum { ROUNDS = 200 };

static PairedShape      g_s;
static unsigned         NSU = 0, NNODE = 0, UNITS = 0, PARTS = 0;
static std::vector<int> g_cpuCluster;
static std::vector<int> g_clusterNode;

static int clusterOf(int cpu)
{
    if(cpu < 0 || cpu >= (int)g_cpuCluster.size()) return -1;
    return g_cpuCluster[cpu];
}

static inline void cpuRelax(void)
{
#if defined(__x86_64__) || defined(__i386__)
    __asm__ __volatile__("pause" ::: "memory");
#endif
}

static void spinUs(long us)
{
    struct timespec t0, t;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    do { clock_gettime(CLOCK_MONOTONIC, &t); }
    while((t.tv_sec - t0.tv_sec) * 1000000000L + (t.tv_nsec - t0.tv_nsec) < us * 1000L);
}

/* PARTS-participant sense-reversing spin barrier with no timeout: a missing
 * participant is a deadlock that the watchdog turns into exit 3. */
static std::atomic<int> g_arrive(0), g_gen(0);

static void barrierAll(void)
{
    const int gen = g_gen.load(std::memory_order_acquire);
    if(g_arrive.fetch_add(1, std::memory_order_acq_rel) + 1 == (int)PARTS)
    {
        g_arrive.store(0, std::memory_order_relaxed);
        g_gen.fetch_add(1, std::memory_order_acq_rel);
        return;
    }
    while(g_gen.load(std::memory_order_acquire) == gen) cpuRelax();
}

static int                    g_round = 0;
static std::atomic<int>       g_entered(0), g_remaining(0);
static std::atomic<long long> g_offSU(0), g_badRounds(0);

/* Background codelets. */
static std::atomic<long long> g_bgFires(0), g_bgOff(0), g_bgPushed(0);

class BgCd : public Codelet
{
public:
    unsigned target_;
    bool     node_;
    BgCd(void) : Codelet(), target_(0), node_(false) { }
    virtual void fire(void)
    {
        spinUs(2);
        int cl = clusterOf(sched_getcpu());
        bool ok;
        if(node_) ok = cl >= 0 && g_clusterNode[cl] == (int)target_;
        else      ok = cl == (int)target_ || cl == (int)(target_ ^ 1u);
        if(!ok) g_bgOff.fetch_add(1);
        g_bgFires.fetch_add(1);
    }
};

static std::vector<BgCd> g_bg;

class HostCodelet : public Codelet
{
public:
    unsigned su_, k_;
    HostCodelet(void) : Codelet(), su_(0), k_(0) { }
    void arm(ThreadedProcedure * tp, unsigned su, unsigned k)
    {
        initCodelet(0, 0, tp, 0);
        su_ = su; k_ = k;
    }
    virtual void fire(void)
    {
        if(clusterOf(sched_getcpu()) != (int)su_) g_offSU.fetch_add(1);
        g_entered.fetch_add(1);
        /* background traffic: one node codelet and one shared codelet */
        const size_t slot = ((size_t)g_round * PARTS + su_ * UNITS + k_) * 2;
        BgCd & n = g_bg[slot];
        n.initCodelet(0, 0, NULL, 0);
        n.node_ = true;
        n.target_ = (unsigned)((su_ + k_ + (unsigned)g_round) % NNODE);
        n.setPlacedNode(n.target_);
        BgCd & s = g_bg[slot + 1];
        s.initCodelet(0, 0, NULL, 0);
        s.node_ = false;
        s.target_ = (unsigned)((su_ * 7 + k_ + (unsigned)g_round) % NSU);
        s.setPlacedShared(s.target_);
        g_bgPushed.fetch_add(2);
        n.add();
        s.add();
        barrierAll();
        if(g_remaining.fetch_sub(1) == 1)
            Runtime::finalSignal.decDep();
    }
};

class HostTP : public ThreadedProcedure
{
public:
    explicit HostTP(unsigned su) : ThreadedProcedure(), cl_(UNITS)
    {
        for(unsigned k = 0; k < UNITS; ++k)
        {
            cl_[k].arm(this, su, k);
            add(&cl_[k]);
        }
    }
private:
    std::vector<HostCodelet> cl_;
};

class RootTP : public ThreadedProcedure
{
public:
    RootTP(void) : ThreadedProcedure()
    {
        for(unsigned c = 0; c < NSU; ++c)
            place<HostTP, unsigned>(c, this, c);
    }
};

int main(void)
{
    if(int rc = skip_unless_paired(g_s, "su-barrier")) return rc;
    hwloc::AbstractMachine am(true);
    NSU = g_s.nsu; NNODE = g_s.nnode; UNITS = g_s.mcPerTp + 1; PARTS = NSU * UNITS;
    g_clusterNode.resize(NSU);
    for(unsigned c = 0; c < NSU; ++c) g_clusterNode[c] = (int)am.numaNodeOfCluster(c);
    g_cpuCluster = cluster_of_cpu_table();
    g_bg.resize((size_t)ROUNDS * PARTS * 2);
    std::printf("su-barrier: %d rounds x %u participants, NUMA_PAIRED, SIBLING_THEN_ANY, "
                "background node+shared traffic\n", ROUNDS, PARTS);
    start_watchdog(60);

    ThreadAffinity affin(UNITS - 1, NSU, NUMA_PAIRED, TPDYNAMIC, MCDYNAMIC, true);
    affin.setStealScope(STEAL_SIBLING_THEN_ANY);
    if(!affin.generateMask()) { std::printf("REFUSE: NUMA_PAIRED generateMask() == false\n"); return EXIT_REFUSE; }
    require_cpus(affin);
    Runtime * rt = new Runtime(&affin);
    if(!rt->constructionOk() || !rt->numaPaired())
    {
        std::printf("REFUSE: constructionOk=%d numaPaired=%d\n", rt->constructionOk(), rt->numaPaired());
        return EXIT_REFUSE;
    }
    ThreadedProcedure * sentinel = new ThreadedProcedure();
    const uint64_t st0   = TPScheduler::placedSteals();

    /* A */
    {
        RowScope r("A", "whole-machine barrier");
        for(g_round = 0; g_round < ROUNDS; ++g_round)
        {
            g_entered.store(0);
            g_remaining.store(PARTS);
            rt->run(new tpClosure(&TPFactory<RootTP>, sentinel));
            if(g_entered.load() != (int)PARTS) g_badRounds.fetch_add(1);
            progress("A");
        }
        check(g_badRounds.load() == 0, "every worker participated every round");
        check(g_offSU.load() == 0, "every barrier codelet fired on its HostTP's SU");
        char d[200];
        std::snprintf(d, sizeof(d), "%u/%u barrier x %d rounds, NUMA_PAIRED + SIBLING_THEN_ANY + bg traffic",
                      PARTS, PARTS, ROUNDS);
        r.setDetail(d);
        r.setRounds(ROUNDS);
        r.setOffTarget(g_offSU.load());
    }

    /* B: drain the background traffic (the micro schedulers keep running). */
    {
        RowScope r("B", "background node/shared codelets");
        for(int i = 0; i < 1000 && g_bgFires.load() < g_bgPushed.load(); ++i) { usleep(10000); progress("B"); }
        check(g_bgFires.load() == g_bgPushed.load(), "every background codelet fired");
        check(g_bgPushed.load() == (long long)ROUNDS * PARTS * 2, "two background codelets per participant");
        check(g_bgOff.load() == 0, "node codelets on their node, shared codelets on {su, sibling}");
        char d[200];
        std::snprintf(d, sizeof(d), "background pushed=%lld fired=%lld off=%lld",
                      g_bgPushed.load(), g_bgFires.load(), g_bgOff.load());
        r.setDetail(d);
        r.setRounds(ROUNDS);
        r.setOffTarget(g_bgOff.load());
    }

    /* C */
    {
        RowScope r("C", "counters");
        check(TPScheduler::placedSteals() == st0, "placedSteals == 0");
        char d[200];
        std::snprintf(d, sizeof(d), "placedSteals=%llu nodePulls=%llu siblingPulls=%llu",
                      (unsigned long long)(TPScheduler::placedSteals() - st0),
                      (unsigned long long)total_node_pulls(),
                      (unsigned long long)total_sibling_pulls());
        r.setDetail(d);
    }

    /* D */
    {
        RowScope r("D", "~Runtime with nothing outstanding returned");
        progress("shutdown");
        delete rt;
        delete sentinel;
        r.setRounds(1);
    }

    stop_watchdog();
    return print_rows("DARTS-RT-SU-BARRIER") ? EXIT_CHECK : EXIT_PASS;
}
