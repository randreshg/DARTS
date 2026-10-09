/*
 * darts-rt-node-place -- node-scoped codelet placement (NUMA_PAIRED).
 *
 * Codelet::setPlacedNode(n) sends a ready codelet to the node pool of NUMA
 * node n (TPScheduler::pushCodeletToNode), which both SUs of that node pull
 * from (TPScheduler::popCodelet). The codelet's placed_ stays
 * UNPLACED_CLUSTER, so isPlaced() is false and no per-SU path sees it.
 *
 *   A  nodes x 20000 setPlacedNode(n) codelets, 5 us spin each: every fire is
 *      on node n, each SU gets more than 5 % of its node's fires
 *   B  the SUs' node-pull counters grew, placedSteals() did not
 *   C  isPlaced() == false, placeScope() == SCOPE_NODE, placedNode() == n for
 *      every node codelet; setPlacedCluster resets the scope to SCOPE_SU
 *   D  refusals under NUMA_PAIRED: pushCodeletToNode(nodes),
 *      pushCodeletShared(SUs) and pushTPNode(nodes) return false and are
 *      counted; an add() of a codelet placed on node 99 is refused
 *      (directedEnqueueFailed) and never fires
 *   E  ~Runtime with nothing outstanding
 *
 * Skipped (exit 4) unless the host has two LLC clusters per NUMA node.
 */
#include "rt_test_util.h"

#include <cstring>
#include <time.h>

using namespace darts;
using namespace rt_test;

enum { MAX_SU = 1024, PER_NODE = 20000, BATCH_PER_NODE = 2000 };

static PairedShape      g_s;
static unsigned         NSU = 0, NNODE = 0, TOTAL = 0;
static std::vector<int> g_cpuCluster;
static int              g_clusterNode[MAX_SU];

static int clusterOf(int cpu)
{
    if(cpu < 0 || cpu >= (int)g_cpuCluster.size()) return -1;
    return g_cpuCluster[cpu];
}

static void spinUs(long us)
{
    struct timespec t0, t;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    do { clock_gettime(CLOCK_MONOTONIC, &t); }
    while((t.tv_sec - t0.tv_sec) * 1000000000L + (t.tv_nsec - t0.tv_nsec) < us * 1000L);
}

static std::atomic<int>       g_left(0);
static std::atomic<long long> g_off(0), g_fires(0), g_badScope(0);
static std::atomic<long long> g_suFires[MAX_SU];

class NodeCd : public Codelet
{
public:
    unsigned node_;
    NodeCd(void) : Codelet(), node_(0) { }
    virtual void fire(void)
    {
        spinUs(5);
        int cl = clusterOf(sched_getcpu());
        if(cl < 0 || cl >= (int)NSU || g_clusterNode[cl] != (int)node_) g_off.fetch_add(1);
        if(cl >= 0 && cl < (int)NSU) g_suFires[cl].fetch_add(1);
        g_fires.fetch_add(1);
        if((g_fires.load() & 1023) == 0) progress();
        if(g_left.fetch_sub(1) == 1)
            Runtime::finalSignal.decDep();
    }
};

static std::vector<NodeCd> g_cds;
static int g_batch = 0;

/* One batch: BATCH_PER_NODE codelets per node, interleaved over the nodes,
 * added from the TP scheduler thread that expands this closure. */
class BatchTP : public ThreadedProcedure
{
public:
    BatchTP(void) : ThreadedProcedure()
    {
        const size_t base = (size_t)g_batch * BATCH_PER_NODE * NNODE;
        for(size_t i = 0; i < (size_t)BATCH_PER_NODE * NNODE; ++i)
        {
            NodeCd & cd = g_cds[base + i];
            const unsigned n = (unsigned)(i % NNODE);
            cd.initCodelet(0, 0, NULL, 0);
            cd.setPlacedNode(n);
            cd.node_ = n;
            if(cd.isPlaced() || cd.placeScope() != SCOPE_NODE || cd.placedNode() != n
               || cd.placedCluster() != Codelet::UNPLACED_CLUSTER)
                g_badScope.fetch_add(1);
            cd.add();
        }
    }
};

/* D: refusals, from a DARTS thread (this constructor runs on a TP scheduler). */
static bool g_rNode = true, g_rShared = true, g_rTP = true;
static std::atomic<int> g_refusedFires(0);

class NeverCd : public Codelet
{
public:
    NeverCd(void) : Codelet() { }
    virtual void fire(void) { g_refusedFires.fetch_add(1); }
};
static NeverCd g_never[3];

class NopTP : public ThreadedProcedure
{
public:
    NopTP(void) : ThreadedProcedure() { }
};

class RefuseTP : public ThreadedProcedure
{
public:
    RefuseTP(void) : ThreadedProcedure()
    {
        g_never[0].initCodelet(0, 0, NULL, 0);
        g_never[1].initCodelet(0, 0, NULL, 0);
        g_rNode   = TPScheduler::pushCodeletToNode(NNODE, &g_never[0]);
        g_rShared = TPScheduler::pushCodeletShared(NSU, &g_never[1]);
        tpClosure * c = new tpClosure(&TPFactory<NopTP>, NULL);
        g_rTP = TPScheduler::pushTPNode(NNODE, c);
        if(!g_rTP) delete c;
        g_never[2].initCodelet(0, 0, NULL, 0);
        g_never[2].setPlacedNode(99);
        g_never[2].add();
        Runtime::finalSignal.decDep();
    }
};

int main(void)
{
    if(int rc = skip_unless_paired(g_s, "node-place")) return rc;
    hwloc::AbstractMachine am(true);
    NSU = g_s.nsu; NNODE = g_s.nnode; TOTAL = NNODE * PER_NODE;
    if(NSU > MAX_SU) { std::printf("node-place: SKIP -- more than %d SUs\n", MAX_SU); return EXIT_REFUSE; }
    for(unsigned c = 0; c < NSU; ++c) g_clusterNode[c] = (int)am.numaNodeOfCluster(c);
    for(unsigned c = 0; c < NSU; ++c) g_suFires[c].store(0);
    g_cpuCluster = cluster_of_cpu_table();
    g_cds.resize(TOTAL);
    std::printf("node-place: %u node codelets (%d per node) on %u nodes, 5 us each\n",
                TOTAL, PER_NODE, NNODE);
    start_watchdog(60);

    ThreadAffinity affin(g_s.mcPerTp, NSU, NUMA_PAIRED, TPDYNAMIC, MCDYNAMIC, true);
    if(!affin.generateMask()) { std::printf("REFUSE: NUMA_PAIRED generateMask() == false\n"); return EXIT_REFUSE; }
    require_cpus(affin);
    Runtime * rt = new Runtime(&affin);
    if(!rt->constructionOk() || !rt->numaPaired())
    {
        std::printf("REFUSE: constructionOk=%d numaPaired=%d\n", rt->constructionOk(), rt->numaPaired());
        return EXIT_REFUSE;
    }
    ThreadedProcedure * sentinel = new ThreadedProcedure();

    const uint64_t np0   = total_node_pulls();
    const uint64_t st0   = TPScheduler::placedSteals();

    /* A + C */
    {
        RowScope r("A", "nodes x 20000 setPlacedNode(n) codelets fire on node n");
        const int batches = PER_NODE / BATCH_PER_NODE;
        for(g_batch = 0; g_batch < batches; ++g_batch)
        {
            g_left.store(BATCH_PER_NODE * NNODE);
            rt->run(new tpClosure(&TPFactory<BatchTP>, sentinel));
            progress("A");
        }
        long long off = g_off.load();
        check(g_fires.load() == (long long)TOTAL, "every node codelet fired exactly once");
        check(off == 0, "every fire on the codelet's node");
        long long minShareBad = 0;
        char d[512]; int p = std::snprintf(d, sizeof(d), "fires=%lld SU share %%:", g_fires.load());
        for(unsigned n = 0; n < NNODE; ++n)
        {
            long long a = 0, b = 0;
            unsigned sa = NSU, sb = NSU;
            for(unsigned c = 0; c < NSU; ++c)
                if(g_clusterNode[c] == (int)n) { if(sa == NSU) sa = c; else sb = c; }
            if(sa < NSU) a = g_suFires[sa].load();
            if(sb < NSU) b = g_suFires[sb].load();
            long long tot = a + b;
            bool ok = sb < NSU && tot > 0 && a * 20 > tot && b * 20 > tot;
            if(!ok) ++minShareBad;
            if(p < (int)sizeof(d) - 24)
                p += std::snprintf(d + p, sizeof(d) - p, " %lld/%lld",
                                   tot ? a * 100 / tot : 0, tot ? b * 100 / tot : 0);
        }
        check(minShareBad == 0, "each SU gets more than 5% of its node's fires");
        r.setDetail(std::string("nodes x 20000 setPlacedNode(n): ") + d);
        r.setRounds(batches);
        r.setOffTarget(off);
    }

    /* B */
    {
        RowScope r("B", "node-pull counters");
        const uint64_t np = total_node_pulls() - np0;
        check(np > 0, "node pulls > 0");
        check(np <= (uint64_t)TOTAL, "node pulls <= node codelets");
        check(TPScheduler::placedSteals() == st0, "placedSteals == 0");
        char d[200];
        std::snprintf(d, sizeof(d), "nodePulls=%llu placedSteals=%llu",
                      (unsigned long long)np,
                      (unsigned long long)(TPScheduler::placedSteals() - st0));
        r.setDetail(d);
    }

    /* C */
    {
        RowScope r("C", "isPlaced()==false, placeScope()==SCOPE_NODE, placedNode()==n");
        check(g_badScope.load() == 0, "every node codelet reported scope NODE and isPlaced()==false");
        NodeCd x;
        x.setPlacedNode(3);
        x.setPlacedCluster(5);
        check(x.placeScope() == SCOPE_SU && x.isPlaced() && x.placedCluster() == 5,
              "setPlacedCluster resets the scope to SCOPE_SU");
        x.setPlacedShared(7);
        check(x.placeScope() == SCOPE_SU_SHARED && x.placedCluster() == 7, "setPlacedShared");
        x.clearPlacedCluster();
        check(x.placeScope() == SCOPE_SU && !x.isPlaced(), "clearPlacedCluster");
        r.setRounds(TOTAL);
        r.setOffTarget(g_badScope.load());
    }

    /* D */
    {
        RowScope r("D", "refusals: node N, shared SU 2N, TP node N, add() on node 99");
        const uint64_t ref0 = TPScheduler::nodeGroupRefused();
        rt->run(new tpClosure(&TPFactory<RefuseTP>, sentinel));
        usleep(20000);
        check(!g_rNode, "pushCodeletToNode(nodes) refused");
        check(!g_rShared, "pushCodeletShared(SUs) refused");
        check(!g_rTP, "pushTPNode(nodes) refused");
        check(g_never[2].directedEnqueueFailed(), "add() on node 99 -> directedEnqueueFailed()");
        check(TPScheduler::nodeGroupRefused() - ref0 == 4, "four refusals counted");
        check(g_refusedFires.load() == 0, "no refused codelet fired");
        char d[160];
        std::snprintf(d, sizeof(d), "refusals: node N, shared SU 2N, TP node N, add() on node 99; counted=%llu",
                      (unsigned long long)(TPScheduler::nodeGroupRefused() - ref0));
        r.setDetail(d);
        r.setRounds(4);
    }

    /* E */
    {
        RowScope r("E", "~Runtime with nothing outstanding returned");
        progress("shutdown");
        delete rt;
        delete sentinel;
        r.setRounds(1);
    }

    stop_watchdog();
    return print_rows("DARTS-RT-NODE-PLACE") ? EXIT_CHECK : EXIT_PASS;
}
