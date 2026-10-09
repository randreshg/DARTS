/*
 * darts-rt-sibling-steal -- scoped stealing under NUMA_PAIRED.
 *
 * Each scenario runs in its own forked child with its own Runtime, because
 * the steal scope is fixed at construction (ThreadAffinity::setStealScope).
 * Shared codelets: setPlacedShared(2k) codelets go to SU 2k's shared_ pool;
 * SU 2k pulls them, and its sibling 2k+1 pulls them only under SIBLING or
 * SIBLING_THEN_ANY. Closures: a LoaderTP placed on SU 2k invoke<>s work
 * closures into SU 2k's ready_; who expands them depends on the scope.
 *
 *   S1 SIBLING: shared codelets on SU 2k fire only on {2k, 2k+1}; sibling
 *      fires > 0 and the SUs' sibling-pull counters grew
 *   S2 SIBLING closure arm: closures are expanded only on 2k or 2k+1
 *   A1 SIBLING_THEN_ANY: cross-node expansions are allowed (and observed);
 *      shared codelets still stay on {2k, 2k+1}
 *   N1 NONE: shared codelets fire only on 2k (no sibling pull)
 *   N2 NONE: closures in ready_ are never stolen (expanded only on 2k)
 *   L1 LEGACY under NUMA_PAIRED: as today -- all closures complete through the
 *      legacy random steal, no sibling pull, placedSteals == 0
 *   R1 push refusal: under COMPACT (no node group), pushCodeletShared /
 *      pushCodeletToNode / pushTPNode and an add() of a shared codelet return
 *      false (refused) and are counted; a non-DARTS thread is refused too
 *
 * Skipped (exit 4) unless the host has two LLC clusters per NUMA node and at
 * least two NUMA nodes.
 */
#include "rt_test_util.h"

#include <cstring>
#include <time.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/wait.h>

using namespace darts;
using namespace rt_test;

enum { NSH = 1000, NCL = 300 };

static PairedShape      g_s;
static unsigned         NSU = 0, NNODE = 0;
static std::vector<int> g_cpuCluster;
static std::vector<int> g_clusterNode;

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

/* Result of one scenario, written by the child into shared memory. */
struct Res
{
    int       exitCode, constructionOk, done;
    long long shFires, shOff, shOnHome, shOnSibling;
    long long clExp, clOnHome, clOnSibling, clOff, clCrossNode;
    unsigned long long siblingSuPulls, placedSteals;
    int       refuseOk;
    unsigned long long refusedCounted;
};
static Res * g_res = 0;

/* ------------------------------------------------------------------ */
/* Child-side state. */

static std::atomic<int>       g_left(0);
static std::atomic<long long> g_shFires(0), g_shOff(0), g_shOnHome(0), g_shOnSib(0);
static std::atomic<long long> g_clExp(0), g_clOnHome(0), g_clOnSib(0), g_clOff(0), g_clCross(0);
static bool                   g_allowSibling = false;

static void finishOne(void)
{
    if(g_left.fetch_sub(1) == 1)
        Runtime::finalSignal.decDep();
}

class SharedCd : public Codelet
{
public:
    unsigned home_;
    SharedCd(void) : Codelet(), home_(0) { }
    virtual void fire(void)
    {
        spinUs(20);
        int cl = clusterOf(sched_getcpu());
        if(cl == (int)home_) g_shOnHome.fetch_add(1);
        else if(cl == (int)(home_ ^ 1u))
        {
            g_shOnSib.fetch_add(1);
            if(!g_allowSibling) g_shOff.fetch_add(1);
        }
        else g_shOff.fetch_add(1);
        g_shFires.fetch_add(1);
        if((g_shFires.load() & 255) == 0) progress();
        finishOne();
    }
};

static std::vector<SharedCd> g_sh;     /* [node * NSH + i] */

class SharedRootTP : public ThreadedProcedure
{
public:
    SharedRootTP(void) : ThreadedProcedure()
    {
        for(unsigned i = 0; i < NSH; ++i)
            for(unsigned k = 0; k < NNODE; ++k)
            {
                SharedCd & cd = g_sh[k * NSH + i];
                cd.initCodelet(0, 0, NULL, 0);
                cd.home_ = 2 * k;
                cd.setPlacedShared(2 * k);
                cd.add();
            }
    }
};

/* A closure's TP constructor runs on the TP scheduler that expanded it. */
class WorkTP : public ThreadedProcedure
{
public:
    explicit WorkTP(unsigned home) : ThreadedProcedure()
    {
        spinUs(50);
        int cl = clusterOf(sched_getcpu());
        g_clExp.fetch_add(1);
        if(cl == (int)home) g_clOnHome.fetch_add(1);
        else if(cl == (int)(home ^ 1u)) g_clOnSib.fetch_add(1);
        else g_clOff.fetch_add(1);
        if(cl < 0 || g_clusterNode[cl] != g_clusterNode[home]) g_clCross.fetch_add(1);
        if((g_clExp.load() & 63) == 0) progress();
        finishOne();
    }
};

class LoaderTP : public ThreadedProcedure
{
public:
    explicit LoaderTP(unsigned su) : ThreadedProcedure()
    {
        for(unsigned i = 0; i < NCL; ++i)
            invoke<WorkTP, unsigned>(this, su);      /* -> ready_ of this SU */
    }
};

class ClosureRootTP : public ThreadedProcedure
{
public:
    ClosureRootTP(void) : ThreadedProcedure()
    {
        for(unsigned k = 0; k < NNODE; ++k)
            place<LoaderTP, unsigned>(2 * k, this, 2 * k);
    }
};

/* R1 */
class NeverCd : public Codelet
{
public:
    NeverCd(void) : Codelet() { }
    virtual void fire(void) { g_shOff.fetch_add(1); }
};
static NeverCd g_never[4];

class NopTP : public ThreadedProcedure
{
public:
    NopTP(void) : ThreadedProcedure() { }
};

static bool g_r[4] = { true, true, true, true };

class RefuseTP : public ThreadedProcedure
{
public:
    RefuseTP(void) : ThreadedProcedure()
    {
        for(int i = 0; i < 4; ++i) g_never[i].initCodelet(0, 0, NULL, 0);
        g_r[0] = TPScheduler::pushCodeletShared(1, &g_never[0]);
        g_r[1] = TPScheduler::pushCodeletToNode(0, &g_never[1]);
        tpClosure * c = new tpClosure(&TPFactory<NopTP>, NULL);
        g_r[2] = TPScheduler::pushTPNode(0, c);
        if(!g_r[2]) delete c;
        g_never[2].setPlacedShared(1);
        g_never[2].add();
        g_r[3] = !g_never[2].directedEnqueueFailed();
        Runtime::finalSignal.decDep();
    }
};

static void * offThread(void * p)
{
    *(bool *)p = TPScheduler::pushCodeletShared(0, &g_never[3]);
    return 0;
}

/* ------------------------------------------------------------------ */

enum Arm { ARM_SHARED = 1, ARM_CLOSURE = 2, ARM_REFUSE = 4 };

static int childMain(AffinityMode mode, StealScope scope, int arms)
{
    start_watchdog(60);
    ThreadAffinity affin(g_s.mcPerTp, NSU, mode, TPDYNAMIC, MCDYNAMIC, true);
    affin.setStealScope(scope);
    if(!affin.generateMask()) { std::printf("REFUSE: generateMask() == false\n"); return EXIT_REFUSE; }
    require_cpus(affin);
    g_allowSibling = (scope == STEAL_SIBLING || scope == STEAL_SIBLING_THEN_ANY);
    Runtime * rt = new Runtime(&affin);
    g_res->constructionOk = rt->constructionOk() ? 1 : 0;
    if(!rt->constructionOk()) { std::printf("REFUSE: constructionOk() == false\n"); return EXIT_REFUSE; }
    ThreadedProcedure * sentinel = new ThreadedProcedure();
    const uint64_t sp0 = total_sibling_pulls();
    const uint64_t st0 = TPScheduler::placedSteals();

    if(arms & ARM_SHARED)
    {
        g_left.store(NNODE * NSH);
        rt->run(new tpClosure(&TPFactory<SharedRootTP>, sentinel));
        progress("shared");
    }
    if(arms & ARM_CLOSURE)
    {
        g_left.store(NNODE * NCL);
        rt->run(new tpClosure(&TPFactory<ClosureRootTP>, sentinel));
        progress("closure");
    }
    if(arms & ARM_REFUSE)
    {
        const uint64_t ref0 = TPScheduler::nodeGroupRefused();
        rt->run(new tpClosure(&TPFactory<RefuseTP>, sentinel));
        bool off = true;
        pthread_t t;
        pthread_create(&t, 0, offThread, &off);
        pthread_join(t, 0);
        usleep(20000);
        g_res->refuseOk = (!g_r[0] && !g_r[1] && !g_r[2] && !g_r[3] && !off) ? 1 : 0;
        g_res->refusedCounted = TPScheduler::nodeGroupRefused() - ref0;
    }
    /* Let the workers that are still destroying TPs settle. */
    usleep(20000);

    g_res->shFires = g_shFires.load();   g_res->shOff = g_shOff.load();
    g_res->shOnHome = g_shOnHome.load(); g_res->shOnSibling = g_shOnSib.load();
    g_res->clExp = g_clExp.load();       g_res->clOnHome = g_clOnHome.load();
    g_res->clOnSibling = g_clOnSib.load(); g_res->clOff = g_clOff.load();
    g_res->clCrossNode = g_clCross.load();
    g_res->siblingSuPulls = total_sibling_pulls() - sp0;
    g_res->placedSteals   = TPScheduler::placedSteals() - st0;

    progress("shutdown");
    delete rt;
    delete sentinel;
    g_res->done = 1;
    stop_watchdog();
    return EXIT_PASS;
}

static int childStatus(pid_t pid)
{
    int st = 0;
    if(waitpid(pid, &st, 0) != pid) return -1;
    if(WIFEXITED(st)) return WEXITSTATUS(st);
    return 1000 + (WIFSIGNALED(st) ? WTERMSIG(st) : 0);
}

static int g_refuseExit = 0;

/* Run one scenario in a forked child; returns false when the child failed. */
static bool runScenario(const char * name, AffinityMode mode, StealScope scope, int arms, Res & out)
{
    std::memset(g_res, 0, sizeof(Res));
    std::fflush(stdout);
    pid_t pid = fork();
    if(pid == 0)
    {
        int rc = childMain(mode, scope, arms);
        std::fflush(stdout);
        _exit(rc);
    }
    int rc = childStatus(pid);
    out = *g_res;
    out.exitCode = rc;
    std::printf("  scenario %-18s exit=%d shared fires=%lld home=%lld sibling=%lld off=%lld | "
                "closures=%lld home=%lld sibling=%lld off=%lld crossNode=%lld | siblingSuPulls=%llu "
                "placedSteals=%llu\n",
                name, rc, out.shFires, out.shOnHome, out.shOnSibling, out.shOff, out.clExp,
                out.clOnHome, out.clOnSibling, out.clOff, out.clCrossNode, out.siblingSuPulls,
                out.placedSteals);
    std::fflush(stdout);
    if(rc == EXIT_REFUSE) g_refuseExit = 1;
    return rc == 0 && out.done;
}

static std::string fmt(const char * f, long long a, long long b, long long c)
{
    char buf[200];
    std::snprintf(buf, sizeof(buf), f, a, b, c);
    return buf;
}

int main(void)
{
    if(int rc = skip_unless_paired(g_s, "sibling-steal")) return rc;
    if(g_s.nnode < 2) { std::printf("sibling-steal: SKIP -- needs two NUMA nodes\n"); return EXIT_REFUSE; }
    hwloc::AbstractMachine am(true);
    NSU = g_s.nsu; NNODE = g_s.nnode;
    g_clusterNode.resize(NSU);
    for(unsigned c = 0; c < NSU; ++c) g_clusterNode[c] = (int)am.numaNodeOfCluster(c);
    g_sh.resize((size_t)NNODE * NSH);
    g_cpuCluster = cluster_of_cpu_table();
    std::printf("sibling-steal: %d shared codelets x %u even SUs (20 us), %d closures x %u even SUs "
                "(50 us)\n", NSH, NNODE, NCL, NNODE);
    g_res = (Res *)mmap(0, sizeof(Res), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    if(g_res == MAP_FAILED) { std::printf("FAIL: mmap\n"); return EXIT_CHECK; }
    const long long nSh = (long long)NNODE * NSH, nCl = (long long)NNODE * NCL;
    Res r;

    /* SIBLING */
    {
        bool ok = runScenario("SIBLING", NUMA_PAIRED, STEAL_SIBLING, ARM_SHARED | ARM_CLOSURE, r);
        row("S1", fmt("SIBLING shared: home=%lld sibling=%lld off=%lld", r.shOnHome, r.shOnSibling, r.shOff),
            nSh, r.shOff, nSh - r.shFires,
            ok && r.shFires == nSh && r.shOff == 0 && r.shOnSibling > 0 && r.siblingSuPulls > 0);
        row("S2", fmt("SIBLING closures: home=%lld sibling=%lld off=%lld", r.clOnHome, r.clOnSibling, r.clOff),
            nCl, r.clOff, nCl - r.clExp,
            ok && r.clExp == nCl && r.clOff == 0 && r.placedSteals == 0);
    }
    /* SIBLING_THEN_ANY */
    {
        bool ok = runScenario("SIBLING_THEN_ANY", NUMA_PAIRED, STEAL_SIBLING_THEN_ANY,
                              ARM_SHARED | ARM_CLOSURE, r);
        row("A1", fmt("SIBLING_THEN_ANY: closures crossNode=%lld (allowed); shared off=%lld sibling=%lld",
                      r.clCrossNode, r.shOff, r.shOnSibling),
            nCl, r.shOff, (nCl - r.clExp) + (nSh - r.shFires),
            ok && r.clExp == nCl && r.shFires == nSh && r.shOff == 0 && r.clCrossNode > 0
               && r.placedSteals == 0);
    }
    /* NONE */
    {
        bool ok = runScenario("NONE", NUMA_PAIRED, STEAL_NONE, ARM_SHARED | ARM_CLOSURE, r);
        row("N1", fmt("NONE shared: home=%lld sibling=%lld off=%lld", r.shOnHome, r.shOnSibling, r.shOff),
            nSh, r.shOff, nSh - r.shFires,
            ok && r.shFires == nSh && r.shOnHome == nSh && r.shOnSibling == 0 && r.siblingSuPulls == 0);
        row("N2", fmt("NONE closures never stolen: home=%lld sibling=%lld off=%lld", r.clOnHome,
                      r.clOnSibling, r.clOff),
            nCl, r.clOnSibling + r.clOff, nCl - r.clExp, ok && r.clExp == nCl && r.clOnHome == nCl);
    }
    /* LEGACY under NUMA_PAIRED */
    {
        bool ok = runScenario("LEGACY", NUMA_PAIRED, STEAL_LEGACY, ARM_SHARED | ARM_CLOSURE, r);
        row("L1", fmt("LEGACY under NUMA_PAIRED: closures home=%lld elsewhere=%lld; shared sibling=%lld",
                      r.clOnHome, r.clOnSibling + r.clOff, r.shOnSibling),
            nCl, 0, (nCl - r.clExp) + (nSh - r.shFires),
            ok && r.clExp == nCl && r.shFires == nSh && r.shOnHome == nSh && r.siblingSuPulls == 0
               && r.placedSteals == 0);
    }
    /* Push refusal without a node group */
    {
        bool ok = runScenario("COMPACT-refuse", COMPACT, STEAL_LEGACY, ARM_REFUSE, r);
        row("R1", fmt("COMPACT: shared/node/TP pushes + add() + non-DARTS refused; counted=%lld",
                      (long long)r.refusedCounted, 0, 0),
            5, 0, r.shOff, ok && r.refuseOk && r.refusedCounted == 5 && r.shOff == 0);
    }

    long long fails = print_rows("DARTS-RT-SIBLING-STEAL");
    if(fails && g_refuseExit) return EXIT_REFUSE;
    return fails ? EXIT_CHECK : EXIT_PASS;
}
