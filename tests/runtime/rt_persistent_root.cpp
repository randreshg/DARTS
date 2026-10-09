/*
 * darts-rt-persistent-root -- placed root entry and persistent-root helpers.
 *
 * Runtime::runPlaced(tps, root) is Runtime::run() with the root closure
 * pushed to TP scheduler `tps` (pushTPPlaced) instead of TPS 0.
 * PersistentRoot.h: PersistentRootTP (self-hold, release_self), FinalOnce
 * (one finalSignal.decDep per run), BodyDrain (host waits for bodies).
 *
 *   A  one runPlaced(R, root) call (R = 5, or the last scheduler on a smaller
 *      machine): the root (sentinel parent) expands on TPS R and chains 1000 tokens x 48 layers through per-token child TPs
 *      (each layer a 1 us spin codelet). Exactly 48,000 fires, run() entered
 *      once, the final codelet's ordering log [drain exit, release_self,
 *      signal] checked inside it, wait_zero returns on the host after run(),
 *      every destructor exactly once, the root deleted after release_self, a
 *      second signal() returns false
 *   B  after A: runPlaced(numTPS) is refused without running, and a second
 *      plain run() behaves normally; one of its codelets calls the already
 *      fired FinalOnce of A (false, and no decDep: run() does not return
 *      before the chain's own end)
 *   C  a gap with no ready codelets of the root, while the join hold sits on
 *      another TP, does not delete a PersistentRootTP; control: the same gap
 *      deletes a plain root
 *   D  helpers without a runtime: wait_zero waits for an outstanding body,
 *      FinalOnce::reset re-arms
 *   E  ~Runtime returns with nothing outstanding
 *
 * The runtime is COMPACT over the LLC clusters (one TP scheduler and its
 * micro schedulers per cluster). Skipped (exit 4) on a host with fewer than
 * two LLC clusters, clusters of different sizes, or when the allocation does
 * not hold the mask's cpus.
 */
#include "rt_test_util.h"
#include "PersistentRoot.h"

#include <chrono>
#include <cstring>

using namespace darts;
using namespace rt_test;

enum { NTOK = 1000, NLAYER = 48, BCHAIN = 100 };

static unsigned ROOT_TPS = 5;
static int      g_rootClusterWant = -1;

static std::vector<int> g_cpuCluster;

static int clusterOf(int cpu)
{
    if(cpu < 0 || cpu >= (int)g_cpuCluster.size()) return -1;
    return g_cpuCluster[cpu];
}

static void spin_ns(long long ns)
{
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    while(std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::steady_clock::now() - t0).count() < ns) { }
}

/* ------------------------------------------------------------------ */
/* Event log: a global sequence, so "after" is a total order. */

enum Ev { EV_DRAIN_EXIT = 1, EV_RELEASE = 2, EV_SIGNAL = 3, EV_ROOT_DTOR = 4 };

static std::atomic<long long> g_seq(0);
static std::atomic<long long> g_seqRelease(0), g_seqRootDtor(0);

static const int LOGMAX = 16;
static int g_log[LOGMAX];
static std::atomic<int> g_logN(0);
static void logEv(int e)
{
    int i = g_logN.fetch_add(1);
    if(i < LOGMAX) g_log[i] = e;
}

/* ------------------------------------------------------------------ */
/* Row A */

static FinalOnce g_finalA;
static BodyDrain g_drainA;
static std::atomic<long long> g_firesA(0), g_rootCtors(0), g_rootDtors(0);
static std::atomic<int>  g_rootTps(-1), g_rootCluster(-1);
static std::atomic<int>  g_orderOk(0);
static std::atomic<int>  g_tokCtor[NTOK], g_tokDtor[NTOK];
static PersistentRootTP * g_rootA = 0;

class TokenTP;

class LayerCodelet : public Codelet
{
public:
    int tok_, layer_;
    LayerCodelet * next_;
    LayerCodelet(void) : Codelet(), tok_(0), layer_(0), next_(0) { }
    virtual void fire(void);
};

class TokenTP : public ThreadedProcedure
{
public:
    int tok_;
    LayerCodelet cd_[NLAYER];
    TokenTP(int t) : ThreadedProcedure(), tok_(t)
    {
        g_tokCtor[t].fetch_add(1);
        for(int l = NLAYER - 1; l >= 0; --l)
        {
            cd_[l].initCodelet(l == 0 ? 0 : 1, l == 0 ? 0 : 1, this, SHORTWAIT);
            cd_[l].tok_ = t;
            cd_[l].layer_ = l;
            cd_[l].next_ = (l + 1 < NLAYER) ? &cd_[l + 1] : 0;
        }
        add(&cd_[0]);
    }
    virtual ~TokenTP() { g_tokDtor[tok_].fetch_add(1); }
};

void LayerCodelet::fire(void)
{
    g_drainA.enter();
    spin_ns(1000);
    g_firesA.fetch_add(1);
    if((g_firesA.load(std::memory_order_relaxed) & 1023) == 0) progress("A");
    if(next_)
    {
        LayerCodelet * n = next_;
        g_drainA.exit();
        n->decDep();
        return;
    }
    if(tok_ + 1 < NTOK)
    {
        /* The child chain hangs off the root; this token TP still holds a
         * reference on the root while it spawns. */
        invoke<TokenTP>(getTP()->parentTP_, tok_ + 1);
        g_drainA.exit();
        return;
    }
    /* Final codelet: exit the body, drop holds (release_self), then signal
     * as the last action. */
    PersistentRootTP * root = g_rootA;
    g_drainA.exit();
    logEv(EV_DRAIN_EXIT);
    g_seqRelease.store(g_seq.fetch_add(1) + 1);
    root->release_self();
    logEv(EV_RELEASE);
    logEv(EV_SIGNAL);
    int n = g_logN.load();
    bool ok = n >= 3 && g_log[n - 3] == EV_DRAIN_EXIT && g_log[n - 2] == EV_RELEASE
              && g_log[n - 1] == EV_SIGNAL;
    g_orderOk.store(ok ? 1 : -1);
    g_finalA.signal();          /* last action: nothing of this codelet/TP follows */
}

class RootA : public PersistentRootTP
{
public:
    RootA(void) : PersistentRootTP()
    {
        g_rootCtors.fetch_add(1);
        g_rootTps.store(myThread.threadTPsched ? (int)myThread.threadTPsched->getClusterIndex() : -1);
        g_rootCluster.store(clusterOf(sched_getcpu()));
        g_rootA = this;
        invoke<TokenTP>(this, 0);
    }
    virtual ~RootA()
    {
        g_seqRootDtor.store(g_seq.fetch_add(1) + 1);
        logEv(EV_ROOT_DTOR);
        g_rootDtors.fetch_add(1);
    }
};

/* ------------------------------------------------------------------ */
/* Row B: a plain chain under a second run(). */

static std::atomic<long long> g_firesB(0);
static std::atomic<int> g_secondSignal(-1);

class ChainCodelet : public Codelet
{
public:
    int idx_;
    ChainCodelet * next_;
    ChainCodelet(void) : Codelet(), idx_(0), next_(0) { }
    virtual void fire(void)
    {
        if(idx_ == 0)
            g_secondSignal.store(g_finalA.signal() ? 1 : 0);
        spin_ns(100000);       /* 100 us x 100: an early decDep would return run() ~10 ms early */
        g_firesB.fetch_add(1);
        progress("B");
        if(next_) { next_->decDep(); return; }
        Runtime::finalSignal.decDep();
    }
};

class ChainTP : public ThreadedProcedure
{
public:
    ChainCodelet cd_[BCHAIN];
    ChainTP(void) : ThreadedProcedure()
    {
        for(int i = BCHAIN - 1; i >= 0; --i)
        {
            cd_[i].initCodelet(i == 0 ? 0 : 1, i == 0 ? 0 : 1, this, SHORTWAIT);
            cd_[i].idx_ = i;
            cd_[i].next_ = (i + 1 < BCHAIN) ? &cd_[i + 1] : 0;
        }
        add(&cd_[0]);
    }
};

/* ------------------------------------------------------------------ */
/* Row C: the gap. */

static const long long GAP_US = 50000;
static FinalOnce g_finalC;
static std::atomic<long long> g_rootCDtors(0), g_joinDtors(0);
static std::atomic<long long> g_dtorsAtJoin(-1);
static std::atomic<int> g_persistentC(0);
static ThreadedProcedure * g_rootC = 0;
static ThreadedProcedure * g_sentinel = 0;

class JoinTP;

class JoinCodelet : public Codelet
{
public:
    JoinCodelet(void) : Codelet() { }
    virtual void fire(void)
    {
        g_dtorsAtJoin.store(g_rootCDtors.load());
        if(g_persistentC.load())
        {
            g_seqRelease.store(g_seq.fetch_add(1) + 1);
            static_cast<PersistentRootTP*>(g_rootC)->release_self();
        }
        g_finalC.signal();      /* last action */
    }
};

class TimerCodelet : public Codelet
{
public:
    JoinCodelet * join_;
    TimerCodelet(void) : Codelet(), join_(0) { }
    virtual void fire(void)
    {
        /* No ready codelet anywhere during this sleep; the root holds only
         * its self-hold, the join hold sits on JoinTP. */
        usleep((useconds_t)GAP_US);
        progress("C");
        join_->decDep();
    }
};

class JoinTP : public ThreadedProcedure
{
public:
    JoinCodelet  join_;
    TimerCodelet timer_;
    JoinTP(void) : ThreadedProcedure()
    {
        join_.initCodelet(1, 1, this, SHORTWAIT);
        timer_.initCodelet(0, 0, this, SHORTWAIT);
        timer_.join_ = &join_;
        add(&timer_);
    }
    virtual ~JoinTP() { g_joinDtors.fetch_add(1); }
};

class KickCodelet : public Codelet
{
public:
    KickCodelet(void) : Codelet() { }
    virtual void fire(void)
    {
        /* The join TP is a child of the sentinel, not of the root. */
        invoke<JoinTP>(g_sentinel);
    }
};

class RootCPersistent : public PersistentRootTP
{
public:
    KickCodelet kick_;
    RootCPersistent(void) : PersistentRootTP()
    {
        g_rootC = this;
        kick_.initCodelet(0, 0, this, SHORTWAIT);
        add(&kick_);
    }
    virtual ~RootCPersistent()
    {
        g_seqRootDtor.store(g_seq.fetch_add(1) + 1);
        g_rootCDtors.fetch_add(1);
    }
};

class RootCPlain : public ThreadedProcedure
{
public:
    KickCodelet kick_;
    RootCPlain(void) : ThreadedProcedure()
    {
        g_rootC = this;
        kick_.initCodelet(0, 0, this, SHORTWAIT);
        add(&kick_);
    }
    virtual ~RootCPlain() { g_rootCDtors.fetch_add(1); }
};

/* Wait (bounded) for a worker-side counter: the last decRef of a root may
 * run on a worker after run() has returned on the host. */
static bool waitFor(const std::atomic<long long> & c, long long want, int ms)
{
    for(int i = 0; i < ms; ++i)
    {
        if(c.load() == want) return true;
        usleep(1000);
    }
    return c.load() == want;
}

static bool waitAllTokDtors(int ms)
{
    for(int i = 0; i <= ms; ++i)
    {
        bool all = true;
        for(int t = 0; t < NTOK && all; ++t) all = g_tokDtor[t].load() >= 1;
        if(all) return true;
        usleep(1000);
    }
    return false;
}

int main(void)
{
    hwloc::AbstractMachine am(true);
    const unsigned nsu = (unsigned)am.getNbClusters();
    const unsigned units = nsu ? (unsigned)am.getClusterMap()[0].getNbUnits() : 0;
    std::printf("persistent-root: llc_clusters=%u units_per_cluster=%u numa_nodes=%zu\n",
                nsu, units, am.getNbNumaNodes());
    bool uniform = nsu >= 2 && units >= 2;
    for(unsigned c = 0; uniform && c < nsu; ++c)
        uniform = am.getClusterMap()[c].getNbUnits() == units;
    if(!uniform)
    {
        std::printf("SKIP: needs at least two LLC clusters of equal size (>= 2 units)\n");
        return EXIT_REFUSE;
    }
    if(ROOT_TPS >= nsu) ROOT_TPS = nsu - 1;
    g_cpuCluster = cluster_of_cpu_table();
    for(int t = 0; t < NTOK; ++t) { g_tokCtor[t].store(0); g_tokDtor[t].store(0); }

    /* D (no runtime) */
    {
        RowScope r("D", "BodyDrain::wait_zero waits for a body; FinalOnce reset");
        BodyDrain d;
        d.wait_zero();                      /* zero: returns at once */
        d.enter();
        std::atomic<int> exited(0);
        std::thread t([&]() { usleep(30000); exited.store(1); d.exit(); });
        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
        d.wait_zero();
        long long waitedUs = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t0).count();
        check(exited.load() == 1, "wait_zero returned only after exit()");
        check(waitedUs >= 20000, "wait_zero waited for the outstanding body");
        t.join();
        r.setRounds(1);
    }

    start_watchdog(60);
    ThreadAffinity affin(units - 1, nsu, COMPACT, TPDYNAMIC, MCDYNAMIC, true);
    if(!affin.generateMask())
    {
        std::printf("SKIP: COMPACT one-scheduler-per-cluster mask refused\n");
        return EXIT_REFUSE;
    }
    require_cpus(affin);
    g_rootClusterWant = (int)affin.getTPMask()->clusterID[ROOT_TPS];
    Runtime * rt = new Runtime(&affin);
    if(!rt->constructionOk() || rt->getNumTPS() != nsu)
    {
        std::printf("REFUSE: Runtime constructionOk=%d numTPS=%u\n",
                    rt->constructionOk() ? 1 : 0, rt->getNumTPS());
        return EXIT_REFUSE;
    }
    ThreadedProcedure * sentinel = new ThreadedProcedure();
    g_sentinel = sentinel;

    /* A */
    {
        RowScope r("A", "runPlaced(R): 1000 tok x 48 layers, persistent root");
        g_finalA.reset();
        int runCalls = 0;
        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
        ++runCalls;
        bool ran = rt->runPlaced(ROOT_TPS, new tpClosure(&TPFactory<RootA>, sentinel));
        long long runMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0).count();
        g_drainA.wait_zero();
        bool waited = true;                 /* reached: wait_zero returned on the host */
        check(ran, "runPlaced(R) accepted the root");
        check(runCalls == 1, "run entered once");
        check(waited, "wait_zero returned on the host after runPlaced()");
        check(g_firesA.load() == (long long)NTOK * NLAYER, "exactly 48,000 fires");
        check(g_rootCtors.load() == 1, "root expanded once");
        check(g_rootTps.load() == (int)ROOT_TPS, "root expanded on TPS R (scheduler index)");
        check(g_rootCluster.load() == g_rootClusterWant, "root expanded on a cpu of TPS R's cluster");
        check(g_orderOk.load() == 1, "ordering log [drain exit, release_self, signal] inside the final codelet");
        check(g_finalA.fired(), "FinalOnce fired");
        bool rootGone = waitFor(g_rootDtors, 1, 5000);
        check(rootGone && g_rootDtors.load() == 1, "root destructor exactly once");
        check(g_seqRootDtor.load() > g_seqRelease.load() && g_seqRelease.load() > 0,
              "root deleted after release_self");
        check(waitAllTokDtors(5000), "every token TP destroyed");
        long long badTok = 0;
        for(int t = 0; t < NTOK; ++t)
            if(g_tokCtor[t].load() != 1 || g_tokDtor[t].load() != 1) ++badTok;
        check(badTok == 0, "every token TP constructed and destroyed exactly once");
        check(g_rootDtors.load() == 1, "root destructor still exactly once");
        check(!g_finalA.signal(), "a second signal() returns false");
        char d[200];
        std::snprintf(d, sizeof(d), "runPlaced(R): fires=%lld root_tps=%d run_ms=%lld tok_dtor_bad=%lld",
                      g_firesA.load(), g_rootTps.load(), runMs, badTok);
        r.setDetail(d);
        r.setRounds(NTOK);
        r.setOffTarget(g_rootTps.load() == (int)ROOT_TPS ? 0 : 1);
        progress("A done");
    }

    /* B */
    {
        RowScope r("B", "second run(): normal; runPlaced(numTPS) refused");
        tpClosure * c = new tpClosure(&TPFactory<ChainTP>, sentinel);
        check(!rt->runPlaced(rt->getNumTPS(), c), "runPlaced(numTPS) refused");
        delete c;
        check(!rt->runPlaced(ROOT_TPS, 0), "runPlaced(NULL) refused");
        rt->run(new tpClosure(&TPFactory<ChainTP>, sentinel));
        long long firesAtReturn = g_firesB.load();
        check(g_secondSignal.load() == 0, "fired FinalOnce: second signal() returns false inside a run");
        check(firesAtReturn == BCHAIN, "run() returned only after the chain's own final (no decDep from FinalOnce)");
        char d[160];
        std::snprintf(d, sizeof(d), "second run(): fires_at_return=%lld/%d second_signal=%d",
                      firesAtReturn, (int)BCHAIN, g_secondSignal.load());
        r.setDetail(d);
        r.setRounds(1);
        progress("B done");
    }

    /* C */
    {
        RowScope r("C", "gap without ready codelets keeps the persistent root");
        /* Control: a plain root is deleted in the same gap. */
        g_persistentC.store(0);
        g_rootCDtors.store(0); g_dtorsAtJoin.store(-1);
        g_finalC.reset();
        check(rt->runPlaced(ROOT_TPS, new tpClosure(&TPFactory<RootCPlain>, sentinel)), "control runPlaced");
        long long ctrlAtJoin = g_dtorsAtJoin.load();
        check(ctrlAtJoin == 1, "control: plain root deleted during the gap");
        check(waitFor(g_joinDtors, 1, 5000), "control: join TP destroyed");

        /* Persistent root. */
        g_persistentC.store(1);
        g_rootCDtors.store(0); g_dtorsAtJoin.store(-1);
        g_seqRelease.store(0); g_seqRootDtor.store(0);
        g_finalC.reset();
        check(rt->runPlaced(ROOT_TPS, new tpClosure(&TPFactory<RootCPersistent>, sentinel)), "persistent runPlaced");
        long long persAtJoin = g_dtorsAtJoin.load();
        check(persAtJoin == 0, "persistent root alive at the join after the gap");
        check(waitFor(g_rootCDtors, 1, 5000), "persistent root deleted exactly once after release_self");
        check(g_seqRootDtor.load() > g_seqRelease.load() && g_seqRelease.load() > 0,
              "persistent root deleted after release_self");
        check(waitFor(g_joinDtors, 2, 5000), "join TP destroyed");
        char d[160];
        std::snprintf(d, sizeof(d), "gap %lld us: root dtors at join plain=%lld persistent=%lld",
                      GAP_US, ctrlAtJoin, persAtJoin);
        r.setDetail(d);
        r.setRounds(2);
        progress("C done");
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
    return print_rows("DARTS-RT-PERSISTENT-ROOT") ? EXIT_CHECK : EXIT_PASS;
}
