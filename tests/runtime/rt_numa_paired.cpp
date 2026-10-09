/*
 * darts-rt-numa-paired -- the opt-in NUMA_PAIRED layout.
 *
 * NUMA_PAIRED puts one TP scheduler on each last-level-cache cluster (SU),
 * its micro schedulers on the other units of the same cluster, and adds a
 * node-group layer: SU = cluster = TP scheduler index, node =
 * AbstractMachine::numaNodeOfCluster(SU), two SUs per node.
 *
 *   A  mask: TP scheduler i on unit 0 of cluster i, its micro schedulers on
 *      units 1..mcPerTp of cluster i; getSuPerNode() == 2,
 *      nodeOfTps(i) == numaNodeOfCluster(i); steal scope defaults to
 *      STEAL_LEGACY and place<> is sticky by default
 *   B  Runtime: constructionOk(), numaPaired(), numNodes(), nodeOfTPS(i);
 *      every SU is joined to its sibling and node
 *   C  ROUNDS rounds of one setPlacedCluster(su) codelet per SU: every fire
 *      runs on a cpu of that SU (the directed SU placement is unchanged
 *      under the node layer)
 *   D  refusals: mcPerTp too large, numTPS != 2 x nodes and LLC = false give
 *      generateMask() == false and leave no node table; SPREAD and COMPACT
 *      never get a node group
 *   E  ~Runtime returns
 *
 * Skipped (exit 4) unless the host has two LLC clusters per NUMA node and
 * this process may use all their cpus.
 */
#include "rt_test_util.h"

using namespace darts;
using namespace rt_test;

enum { ROUNDS = 1000 };

static PairedShape g_s;
static std::vector<int> g_cpuCluster;

static int clusterOf(int cpu)
{
    if(cpu < 0 || cpu >= (int)g_cpuCluster.size()) return -1;
    return g_cpuCluster[cpu];
}

static std::atomic<int>       g_left(0);
static std::atomic<long long> g_off(0), g_fires(0);

class SuCodelet : public Codelet
{
public:
    unsigned su_;
    SuCodelet(void) : Codelet(), su_(0) { }
    virtual void fire(void)
    {
        if(clusterOf(sched_getcpu()) != (int)su_) g_off.fetch_add(1);
        g_fires.fetch_add(1);
        progress();
        if(g_left.fetch_sub(1) == 1)
            Runtime::finalSignal.decDep();
    }
};

static std::vector<SuCodelet> g_cd;

class RoundTP : public ThreadedProcedure
{
public:
    RoundTP(void) : ThreadedProcedure()
    {
        for(unsigned su = 0; su < g_s.nsu; ++su)
        {
            g_cd[su].initCodelet(0, 0, NULL, SHORTWAIT);
            g_cd[su].setPlacedCluster(su);
            g_cd[su].su_ = su;
            g_cd[su].add();
        }
    }
};

int main(void)
{
    if(int rc = skip_unless_paired(g_s, "numa-paired")) return rc;
    hwloc::AbstractMachine am(true);
    std::printf("numa-paired: llc_clusters=%u numa_nodes=%u mc_per_tp=%u\n",
                g_s.nsu, g_s.nnode, g_s.mcPerTp);
    g_cpuCluster = cluster_of_cpu_table();
    g_cd.resize(g_s.nsu);
    start_watchdog(60);

    /* A */
    ThreadAffinity affin(g_s.mcPerTp, g_s.nsu, NUMA_PAIRED, TPDYNAMIC, MCDYNAMIC, true);
    {
        RowScope r("A", "NUMA_PAIRED mask: one SU per LLC cluster, 2 SU per node");
        check(affin.generateMask(), "NUMA_PAIRED generateMask() == true");
        long long bad = 0;
        AffinityMask * tp = affin.getTPMask();
        AffinityMask * mc = affin.getMCMask();
        for(unsigned i = 0; i < g_s.nsu; ++i)
        {
            if(tp->clusterID[i] != i || tp->unitID[i] != 0) ++bad;
            for(unsigned j = 0; j < g_s.mcPerTp; ++j)
                if(mc->clusterID[i * g_s.mcPerTp + j] != i || mc->unitID[i * g_s.mcPerTp + j] != j + 1) ++bad;
            if(affin.nodeOfTps(i) != (int)am.numaNodeOfCluster(i)) ++bad;
        }
        check(bad == 0, "TP/MC masks and nodeOfTps(i) == numaNodeOfCluster(i)");
        check(affin.getSuPerNode() == 2, "getSuPerNode() == 2");
        check(affin.getStealScope() == STEAL_LEGACY, "steal scope defaults to STEAL_LEGACY");
        check(affin.stickyPlacement(), "place<> is sticky by default under NUMA_PAIRED");
        r.setRounds(g_s.nsu);
        r.setOffTarget(bad);
    }

    /* D (before the Runtime: nothing here may depend on it) */
    {
        RowScope r("D", "refusals leave no node table; SPREAD/COMPACT have none");
        ThreadAffinity big(g_s.mcPerTp + 1, g_s.nsu, NUMA_PAIRED, TPDYNAMIC, MCDYNAMIC, true);
        check(!big.generateMask(), "mcPerTp = units per cluster refused");
        check(big.getSuPerNode() == 0 && big.nodeOfTps(0) == -1, "mcPerTp refusal clamps nothing");
        ThreadAffinity half(g_s.mcPerTp, g_s.nnode, NUMA_PAIRED, TPDYNAMIC, MCDYNAMIC, true);
        check(!half.generateMask(), "numTPS != 2 x nodes refused");
        ThreadAffinity nollc(g_s.mcPerTp, g_s.nsu, NUMA_PAIRED, TPDYNAMIC, MCDYNAMIC, false);
        check(!nollc.generateMask(), "LLC = false refused");
        check(!nollc.stickyPlacement(), "a refused mask is not sticky by default");
        ThreadAffinity compact(g_s.mcPerTp, g_s.nsu, COMPACT, TPDYNAMIC, MCDYNAMIC, true);
        check(compact.generateMask() && compact.getSuPerNode() == 0 && !compact.stickyPlacement(),
              "COMPACT: no node group, place<> not sticky");
        ThreadAffinity spread(g_s.mcPerTp, g_s.nsu, SPREAD, TPDYNAMIC, MCDYNAMIC, true);
        check(spread.generateMask() && spread.getSuPerNode() == 0, "SPREAD: no node group");
        r.setRounds(5);
    }

    /* B */
    require_cpus(affin);
    ThreadedProcedure * sentinel = 0;
    Runtime * rt = 0;
    {
        RowScope r("B", "Runtime(NUMA_PAIRED): node table, siblings, node pools");
        rt = new Runtime(&affin);
        check(rt->constructionOk(), "constructionOk()");
        check(rt->numaPaired(), "numaPaired()");
        check(rt->numNodes() == g_s.nnode, "numNodes() == NUMA nodes");
        long long bad = 0;
        TPScheduler * me = myThread.threadTPsched;
        for(unsigned i = 0; i < g_s.nsu; ++i)
        {
            if(rt->nodeOfTPS(i) != (unsigned)am.numaNodeOfCluster(i)) ++bad;
            TPScheduler * s = static_cast<TPScheduler *>(me->getRuntimeTPSched(i));
            if(!s->hasNodeGroup() || s->getNode() != i / 2 || !s->getSibling()
               || s->getSibling()->getClusterIndex() != (i ^ 1u) || s->getClusterIndex() != i
               || !s->stickyPlacement())
                ++bad;
        }
        check(bad == 0, "nodeOfTPS(i), node, sibling and stickiness of every SU");
        check(clusterOf(sched_getcpu()) == 0, "the constructing thread runs on SU 0");
        r.setRounds(g_s.nsu);
        r.setOffTarget(bad);
        if(!rt->constructionOk())
        {
            r.finish(false);
            print_rows("DARTS-RT-NUMA-PAIRED");
            return EXIT_REFUSE;
        }
        sentinel = new ThreadedProcedure();
    }

    /* C */
    {
        RowScope r("C", "setPlacedCluster(su) codelets fire on their SU");
        const uint64_t st0 = TPScheduler::placedSteals();
        for(int round = 0; round < ROUNDS; ++round)
        {
            g_left.store(g_s.nsu);
            rt->run(new tpClosure(&TPFactory<RoundTP>, sentinel));
            progress("C");
        }
        long long off = g_off.load();
        check(g_fires.load() == (long long)ROUNDS * g_s.nsu, "every codelet fired exactly once per round");
        check(off == 0, "every placed codelet fired on a cpu of its SU");
        check(TPScheduler::placedSteals() == st0, "placedSteals unchanged");
        char d[160];
        std::snprintf(d, sizeof(d), "setPlacedCluster(su) codelets fire on their SU, fires=%lld",
                      g_fires.load());
        r.setDetail(d);
        r.setRounds(ROUNDS);
        r.setOffTarget(off);
    }

    /* E */
    {
        RowScope r("E", "~Runtime returned");
        progress("shutdown");
        delete rt;
        delete sentinel;
        r.setRounds(1);
    }

    stop_watchdog();
    return print_rows("DARTS-RT-NUMA-PAIRED") ? EXIT_CHECK : EXIT_PASS;
}
