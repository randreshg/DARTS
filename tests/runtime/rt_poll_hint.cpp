/*
 * darts-rt-poll-hint -- the idle-poll non-empty hint.
 *
 * Every idle worker of a TBB=OFF DARTS build polls pthread-mutex deques: a
 * micro scheduler calls its TP scheduler's popCodelet() (codelets_, then
 * under a node group the shared pool, the node pool and the sibling's
 * shared pool), and the TP scheduler adds popTP() (node closure pool, ready_)
 * and steal(). With short idle back-off those empty-pool lock round trips
 * contend with the pushes that carry real work. The hint is an atomic count
 * per pool, raised before the push and lowered after a pop that returned an
 * item, so a count of 0 proves the pool held nothing at that instant and the
 * poll skips the lock (TPScheduler::setIdlePollHint).
 *
 * pthread_mutex_lock and usleep are wrapped at link time (-Wl,--wrap), so the
 * rows count real lock acquisitions and idle back-off sleeps.
 *
 *   A  in-runtime stress, NUMA_PAIRED + STEAL_SIBLING, default hint (on): one
 *      producer codelet per SU releases 2000 items per round in small bursts
 *      to every queue scope (local add, pushCodeletTo, setPlacedShared,
 *      setPlacedNode) while the other workers idle-poll; every item fires
 *      exactly once, 20 rounds
 *   B  the same under COMPACT with the hint opted in (local add and
 *      pushCodeletTo only: COMPACT has no shared or node pools)
 *   C  scheduler-level stress, hint on: standalone TPDynamic schedulers in a
 *      node group (STEAL_SIBLING); 8 producer threads push 800000 codelets
 *      (codelets_, shared, node pool) and 200000 closures (ready_, node
 *      closure pool) in bursts while 64 poller threads run the micro
 *      scheduler idle poll (popCodelet) and the TP scheduler idle poll
 *      (popTP, steal, popCodelet); every item is popped exactly once and
 *      every hint count returns to 0; 10 rounds, fresh schedulers each
 *   M  micro benchmark, 64 threads x 200000 idle polls on empty standalone
 *      schedulers: lock acquisitions and ns per poll, hint off vs on, for the
 *      micro scheduler and the TP scheduler idle poll
 *   I  idle runtimes (every worker idle, 1 s): lock acquisitions per idle
 *      back-off sleep for NUMA_PAIRED off / default and COMPACT default / on
 *   L  default-off: ThreadAffinity::idlePollHint() is false for SPREAD,
 *      COMPACT and a refused or not yet generated NUMA_PAIRED mask, true for
 *      an accepted NUMA_PAIRED mask, and the setter overrides both ways; a
 *      scheduler without setIdlePollHint() keeps every count at 0 and takes
 *      exactly the old locks
 *
 * Runtimes run in forked children (one Runtime per process). Skipped (exit
 * 4) unless the host has two LLC clusters per NUMA node.
 */
#include "rt_test_util.h"

#include <cstring>
#include <time.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>

using namespace darts;
using namespace rt_test;

/* ------------------------------------------------------------------ */
/* pthread_mutex_lock counter (linked with -Wl,--wrap=pthread_mutex_lock). */

enum { MAXCPU = 1024, PAD = 8 };
static std::atomic<unsigned long long> g_lockCount[MAXCPU * PAD];
static __thread unsigned long long t_locks = 0;

extern "C" int __real_pthread_mutex_lock(pthread_mutex_t * m);
extern "C" int __wrap_pthread_mutex_lock(pthread_mutex_t * m)
{
    int cpu = sched_getcpu();
    if(cpu < 0 || cpu >= MAXCPU) cpu = 0;
    g_lockCount[(size_t)cpu * PAD].fetch_add(1, std::memory_order_relaxed);
    ++t_locks;
    return __real_pthread_mutex_lock(m);
}

static unsigned long long totalLocks(void)
{
    unsigned long long s = 0;
    for(size_t c = 0; c < MAXCPU; ++c) s += g_lockCount[c * PAD].load(std::memory_order_relaxed);
    return s;
}

/* usleep counter (linked with -Wl,--wrap=usleep): the idle back-off sleeps
 * of the runtime's policy loops. The measuring thread marks itself. */
static std::atomic<unsigned long long> g_sleeps(0);
static __thread int t_notWorker = 0;

extern "C" int __real_usleep(useconds_t us);
extern "C" int __wrap_usleep(useconds_t us)
{
    if(!t_notWorker) g_sleeps.fetch_add(1, std::memory_order_relaxed);
    return __real_usleep(us);
}

static unsigned long long backoffSleeps(void) { return g_sleeps.load(std::memory_order_relaxed); }

enum { MAX_SU = 256, MAX_NODE = 128 };
static PairedShape g_s;
static unsigned    NSU = 0, NNODE = 0;

static void spinUs(long us)
{
    struct timespec t0, t;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    do { clock_gettime(CLOCK_MONOTONIC, &t); }
    while((t.tv_sec - t0.tv_sec) * 1000000000L + (t.tv_nsec - t0.tv_nsec) < us * 1000L);
}

static double nowSec(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (double)t.tv_sec + 1e-9 * (double)t.tv_nsec;
}

/* ------------------------------------------------------------------ */
/* Results shared with the forked children. */

struct IdleRes
{
    int    done, refused, hint, hintAgree;
    unsigned long long locks, sleeps;
    double secs;
    int    workers;
};

struct StressRes
{
    int       done, refused, hint;
    long long items, fired, dup, missing, rounds;
    unsigned long long pending;
};

enum HintArg { HINT_DEFAULT = 0, HINT_ON = 1, HINT_OFF = 2 };

static const char * hintName(HintArg h)
{
    return h == HINT_ON ? "on" : (h == HINT_OFF ? "off" : "default");
}

/* Builds the affinity of a child runtime; _exit(4) when refused. */
static void makeAffinity(ThreadAffinity & affin, bool paired, HintArg hint, int * refused)
{
    if(paired) affin.setStealScope(STEAL_SIBLING);
    if(hint == HINT_ON)  affin.setIdlePollHint(true);
    if(hint == HINT_OFF) affin.setIdlePollHint(false);
    if(!affin.generateMask()) { *refused = 1; _exit(EXIT_REFUSE); }
    if(first_disallowed_cpu(affinity_cpus(affin)) != -2) { *refused = 1; _exit(EXIT_REFUSE); }
}

/* The idle-poll hint state of every TP scheduler, read from the main thread
 * (TP scheduler 0's thread, outside run()). 1 / 0 when all agree, -1 if not. */
static int runtimeHint(Runtime * rt)
{
    TPScheduler * mine = myThread.threadTPsched;
    if(!mine) return -2;
    int h = mine->idlePollHint() ? 1 : 0;
    for(unsigned i = 0; i < rt->getNumTPS(); ++i)
    {
        TPScheduler * t = static_cast<TPScheduler *>(mine->getRuntimeTPSched(i));
        if(!t || (t->idlePollHint() ? 1 : 0) != h) return -1;
    }
    return h;
}

static unsigned long long runtimePending(Runtime * rt)
{
    TPScheduler * mine = myThread.threadTPsched;
    unsigned long long p = 0;
    for(unsigned i = 0; mine && i < rt->getNumTPS(); ++i)
        p += static_cast<TPScheduler *>(mine->getRuntimeTPSched(i))->idlePollHintPending();
    return p;
}

/* ------------------------------------------------------------------ */
/* I: idle lock rate. */

static void idleChild(bool paired, HintArg hint, IdleRes * r)
{
    t_notWorker = 1;
    ThreadAffinity affin(g_s.mcPerTp, NSU, paired ? NUMA_PAIRED : COMPACT, TPDYNAMIC, MCDYNAMIC, true);
    makeAffinity(affin, paired, hint, &r->refused);
    Runtime * rt = new Runtime(&affin);
    if(!rt->constructionOk() || rt->numaPaired() != paired) { r->refused = 1; _exit(EXIT_REFUSE); }
    r->hint = runtimeHint(rt);
    r->workers = (int)(rt->getNumTPS() * (1 + rt->getNumMCS())) - 1;  /* TPS 0 is this (sleeping) thread */
    usleep(200 * 1000);   /* every worker is in its policy loop */
    const unsigned long long l0 = totalLocks(), s0 = backoffSleeps();
    const double t0 = nowSec();
    usleep(1000 * 1000);
    const double t1 = nowSec();
    const unsigned long long l1 = totalLocks(), s1 = backoffSleeps();
    r->locks = l1 - l0;
    r->sleeps = s1 - s0;
    r->secs = t1 - t0;
    r->done = 1;
    std::fflush(stdout);
    _exit(EXIT_PASS);   /* no ~Runtime: the parent only needs the counts */
}

/* ------------------------------------------------------------------ */
/* A / B: in-runtime stress. */

enum { PER_PRODUCER = 2000, ROUNDS = 20, BURST = 8 };
static int PRODUCERS = 0, ITEMS = 0;

static std::atomic<int> g_left(0);
static std::atomic<int> * g_fire = 0;   /* per item fire count, per round */
static bool g_paired = true;

class ItemCd : public Codelet
{
public:
    int id_;
    ItemCd(void) : Codelet(), id_(0) { }
    virtual void fire(void)
    {
        g_fire[id_].fetch_add(1);
        if(g_left.fetch_sub(1) == 1)
            Runtime::finalSignal.decDep();
    }
};

static ItemCd * g_items = 0;

class ProducerCd : public Codelet
{
public:
    unsigned su_;
    ProducerCd(void) : Codelet(), su_(0) { }
    virtual void fire(void)
    {
        uint64_t x = 0x9E3779B97F4A7C15ULL * (su_ + 1) + (uint64_t)g_left.load();
        const int base = (int)su_ * PER_PRODUCER;
        for(int i = 0; i < PER_PRODUCER; ++i)
        {
            ItemCd & it = g_items[base + i];
            it.initCodelet(0, 0, NULL, 0);
            it.id_ = base + i;
            x ^= x << 13; x ^= x >> 7; x ^= x << 17;
            switch(g_paired ? i % 4 : i % 2)
            {
            case 0: break;                                              /* local add -> codelets_ */
            case 1: it.setPlacedCluster((uint32_t)(x % NSU)); break;   /* pushCodeletTo */
            case 2: it.setPlacedShared((uint32_t)(x % NSU)); break;    /* shared_ */
            case 3: it.setPlacedNode((uint32_t)(x % NNODE)); break;    /* node pool */
            }
            it.add();
            /* Small bursts: let the pools drain to empty between them, so
             * the hint keeps crossing 0 while pollers race the pushes. */
            if((i % BURST) == BURST - 1) spinUs((long)(x % 4));
        }
        if(g_left.fetch_sub(1) == 1)
            Runtime::finalSignal.decDep();
    }
};

static ProducerCd g_prod[MAX_SU];

class StartTP : public ThreadedProcedure
{
public:
    StartTP(void) : ThreadedProcedure()
    {
        for(unsigned s = 0; s < (unsigned)PRODUCERS; ++s)
        {
            g_prod[s].initCodelet(0, 0, NULL, 0);
            g_prod[s].su_ = s;
            g_prod[s].setPlacedCluster(s);
            g_prod[s].add();
        }
    }
};

static void stressChild(bool paired, HintArg hint, StressRes * r)
{
    start_watchdog(60);
    g_paired = paired;
    ThreadAffinity affin(g_s.mcPerTp, NSU, paired ? NUMA_PAIRED : COMPACT, TPDYNAMIC, MCDYNAMIC, true);
    makeAffinity(affin, paired, hint, &r->refused);
    Runtime * rt = new Runtime(&affin);
    if(!rt->constructionOk() || rt->numaPaired() != paired) { r->refused = 1; _exit(EXIT_REFUSE); }
    r->hint = runtimeHint(rt);
    g_items = new ItemCd[ITEMS];
    g_fire = new std::atomic<int>[ITEMS];
    ThreadedProcedure * sentinel = new ThreadedProcedure();
    long long fired = 0, dup = 0, missing = 0;
    for(int round = 0; round < ROUNDS; ++round)
    {
        for(int i = 0; i < ITEMS; ++i) g_fire[i].store(0);
        g_left.store(ITEMS + PRODUCERS);
        rt->run(new tpClosure(&TPFactory<StartTP>, sentinel));
        for(int i = 0; i < ITEMS; ++i)
        {
            const int f = g_fire[i].load();
            fired += f;
            if(f == 0) ++missing;
            if(f > 1) dup += f - 1;
        }
        r->rounds = round + 1;
        progress("A");
    }
    usleep(20 * 1000);
    r->pending = runtimePending(rt);
    r->items = (long long)ITEMS * ROUNDS;
    r->fired = fired;
    r->dup = dup;
    r->missing = missing;
    r->done = 1;
    stop_watchdog();
    std::fflush(stdout);
    _exit(EXIT_PASS);
}

/* ------------------------------------------------------------------ */
/* C / M / L: standalone schedulers (no Runtime, no workers). */

class Tok : public Codelet
{
public:
    Tok(void) : Codelet() { }
    virtual void fire(void) { }
};

struct Group
{
    TPScheduler *          sched[MAX_SU];
    dartsPool<Codelet*>    nodeCd[MAX_NODE];
    dartsPool<tpClosure*>  nodeTP[MAX_NODE];
    PollHintCount          nodeCdN[MAX_NODE];
    PollHintCount          nodeTPN[MAX_NODE];

    /* nodeGroup: setNodeGroup like Runtime::linkTPSched (SU s on node s/2,
     * sibling s^1); hint: setIdlePollHint like linkTPSched. */
    Group(bool nodeGroup, bool hint, StealScope scope)
    {
        for(unsigned s = 0; s < NSU; ++s)
        {
            sched[s] = TPScheduler::create(TPDYNAMIC);
            sched[s]->setID(s * 4);
            sched[s]->setClusterIndex(s);
            sched[s]->setNumPeers(NSU);
        }
        for(unsigned s = 0; s < NSU; ++s)
            for(unsigned j = 0; j < NSU; ++j)
                sched[s]->addPeer(sched[j], j);
        for(unsigned s = 0; s < NSU; ++s)
        {
            const unsigned n = s / 2;
            if(nodeGroup)
                sched[s]->setNodeGroup(n, sched[s ^ 1], &nodeCd[n], &nodeTP[n], scope);
            if(hint)
                sched[s]->setIdlePollHint(nodeGroup ? &nodeCdN[n] : NULL, nodeGroup ? &nodeTPN[n] : NULL);
        }
    }
    ~Group() { for(unsigned s = 0; s < NSU; ++s) delete sched[s]; }

    unsigned long long pending(void) const
    {
        unsigned long long p = 0;
        for(unsigned s = 0; s < NSU; ++s) p += sched[s]->idlePollHintPending();
        for(unsigned n = 0; n < NNODE; ++n) p += Atomics::load(nodeCdN[n].n) + Atomics::load(nodeTPN[n].n);
        return p;
    }
};

/* C */
enum { C_PRODUCERS = 8, C_POLLERS = 64, C_CD_PER_PROD = 100000, C_TP_PER_PROD = 25000,
       C_CD = C_PRODUCERS * C_CD_PER_PROD, C_TP = C_PRODUCERS * C_TP_PER_PROD };

struct StressC
{
    Group *                  g;
    Tok *                    toks;
    tpClosure *              clos;
    std::atomic<unsigned char> * cdSeen;
    std::atomic<unsigned char> * tpSeen;
    std::atomic<long long>   popped, dup, foreign;
    std::atomic<int>         producersLeft;
    std::atomic<int>         stop;
};

static StressC * g_c = 0;

static void * cProducer(void * arg)
{
    const unsigned p = (unsigned)(uintptr_t)arg;
    uint64_t x = 0xD1B54A32D192ED03ULL * (p + 1);
    const unsigned cd0 = p * C_CD_PER_PROD, tp0 = p * C_TP_PER_PROD;
    unsigned ci = 0, ti = 0;
    while(ci < (unsigned)C_CD_PER_PROD || ti < (unsigned)C_TP_PER_PROD)
    {
        for(int b = 0; b < 8; ++b)
        {
            x ^= x << 13; x ^= x >> 7; x ^= x << 17;
            const unsigned su = (unsigned)(x % NSU);
            TPScheduler * t = g_c->g->sched[su];
            const unsigned kind = (unsigned)((x >> 8) % 5);
            if(kind < 3 && ci < (unsigned)C_CD_PER_PROD)
            {
                Codelet * cd = &g_c->toks[cd0 + ci++];
                bool ok = kind == 0 ? t->pushCodelet(cd) : (kind == 1 ? t->pushShared(cd) : t->pushNodeCodelet(cd));
                check(ok, "standalone codelet push");
            }
            else if(ti < (unsigned)C_TP_PER_PROD)
            {
                tpClosure * c = &g_c->clos[tp0 + ti++];
                bool ok = (kind & 1) ? t->pushTP(c) : t->pushNodeTP(c);
                check(ok, "standalone closure push");
            }
        }
        if((x & 7) == 0) spinUs((long)((x >> 3) % 6));
    }
    g_c->producersLeft.fetch_sub(1);
    return 0;
}

static void notePopCd(Codelet * c)
{
    const ptrdiff_t i = static_cast<Tok *>(c) - g_c->toks;
    if(i < 0 || i >= C_CD) { g_c->foreign.fetch_add(1); return; }
    if(g_c->cdSeen[i].fetch_add(1)) g_c->dup.fetch_add(1);
    g_c->popped.fetch_add(1);
}

static void notePopTp(tpClosure * c)
{
    const ptrdiff_t i = c - g_c->clos;
    if(i < 0 || i >= C_TP) { g_c->foreign.fetch_add(1); return; }
    if(g_c->tpSeen[i].fetch_add(1)) g_c->dup.fetch_add(1);
    g_c->popped.fetch_add(1);
}

static void * cPoller(void * arg)
{
    const unsigned k = (unsigned)(uintptr_t)arg;
    TPScheduler * t = g_c->g->sched[(k / 4) % NSU];
    const bool tps = (k % 4) == 0;
    long long idle = 0;
    while(!g_c->stop.load(std::memory_order_relaxed))
    {
        bool got = false;
        if(tps)
        {
            tpClosure * c = t->popTP();
            if(!c) c = t->steal();
            if(c) { notePopTp(c); got = true; }
        }
        if(Codelet * cd = t->popCodelet()) { notePopCd(cd); got = true; }
        if(!got && ((++idle) & 63) == 0) usleep(1);
    }
    return 0;
}

struct CRound { long long popped, dup, missing, foreign; unsigned long long pending; double secs; };

static CRound stressRound(void)
{
    Group g(true, true, STEAL_SIBLING);
    StressC c;
    c.g = &g;
    c.toks = new Tok[C_CD];
    c.clos = new tpClosure[C_TP];
    c.cdSeen = new std::atomic<unsigned char>[C_CD];
    c.tpSeen = new std::atomic<unsigned char>[C_TP];
    for(int i = 0; i < C_CD; ++i) c.cdSeen[i].store(0);
    for(int i = 0; i < C_TP; ++i) c.tpSeen[i].store(0);
    c.popped.store(0); c.dup.store(0); c.foreign.store(0);
    c.producersLeft.store(C_PRODUCERS); c.stop.store(0);
    g_c = &c;
    pthread_t prod[C_PRODUCERS], poll[C_POLLERS];
    for(unsigned k = 0; k < C_POLLERS; ++k) pthread_create(&poll[k], 0, cPoller, (void *)(uintptr_t)k);
    const double t0 = nowSec();
    for(unsigned p = 0; p < C_PRODUCERS; ++p) pthread_create(&prod[p], 0, cProducer, (void *)(uintptr_t)p);
    const long long total = (long long)C_CD + C_TP;
    long long last = -1;
    while(c.popped.load() < total || c.producersLeft.load() > 0)
    {
        usleep(10 * 1000);
        long long now = c.popped.load();
        if(now != last) { last = now; progress("C"); }
        if(nowSec() - t0 > 120) break;
    }
    const double secs = nowSec() - t0;
    c.stop.store(1);
    for(unsigned p = 0; p < C_PRODUCERS; ++p) pthread_join(prod[p], 0);
    for(unsigned k = 0; k < C_POLLERS; ++k) pthread_join(poll[k], 0);
    long long missing = 0;
    for(int i = 0; i < C_CD; ++i) if(!c.cdSeen[i].load()) ++missing;
    for(int i = 0; i < C_TP; ++i) if(!c.tpSeen[i].load()) ++missing;
    CRound res;
    res.popped = c.popped.load(); res.dup = c.dup.load(); res.missing = missing;
    res.foreign = c.foreign.load(); res.pending = g.pending(); res.secs = secs;
    g_c = 0;
    delete [] c.toks; delete [] c.clos; delete [] c.cdSeen; delete [] c.tpSeen;
    return res;
}

enum { C_ROUNDS = 10 };

static void rowC(void)
{
    RowScope r("C", "scheduler-level stress");
    const long long total = (long long)C_CD + C_TP;
    long long popped = 0, dup = 0, missing = 0, foreign = 0;
    unsigned long long pend = 0;
    double secs = 0;
    for(int k = 0; k < C_ROUNDS; ++k)
    {
        const CRound x = stressRound();
        check(x.popped == total, "popped == pushed");
        popped += x.popped; dup += x.dup; missing += x.missing; foreign += x.foreign;
        pend += x.pending; secs += x.secs;
    }
    check(dup == 0, "no item popped twice");
    check(missing == 0, "no item lost");
    check(foreign == 0, "no foreign pointer");
    check(pend == 0, "every hint count back to 0");
    char d[256];
    std::snprintf(d, sizeof(d), "%d x (8 producers, 64 pollers, %d codelets + %d closures): popped=%lld dup=%lld missing=%lld pending=%llu %.2fs",
                  C_ROUNDS, C_CD, C_TP, popped, dup, missing, pend, secs);
    r.setDetail(d);
    r.setRounds((long long)C_ROUNDS * total);
    r.setOffTarget(dup + missing);
}

/* M / L: idle-poll micro benchmark on empty standalone schedulers. */
enum { M_THREADS = 64, M_POLLS = 200000 };

struct PollStat { double locks, ns; };

struct BenchArg
{
    Group *              g;
    unsigned             k;
    bool                 tpsRole;
    std::atomic<int> *   go;
    unsigned long long   locks;
    double               ns;
};

static void * benchThread(void * a)
{
    BenchArg * b = static_cast<BenchArg *>(a);
    TPScheduler * t = b->g->sched[(b->k / 4) % NSU];
    while(!b->go->load()) { }
    const unsigned long long l0 = t_locks;
    const double t0 = nowSec();
    unsigned long long got = 0;
    for(int i = 0; i < M_POLLS; ++i)
    {
        if(b->tpsRole)
        {
            if(tpClosure * c = t->popTP()) { got++; (void)c; }
            else if(tpClosure * c2 = t->steal()) { got++; (void)c2; }
        }
        if(t->popCodelet()) got++;
    }
    b->ns = (nowSec() - t0) * 1e9 / M_POLLS;
    b->locks = t_locks - l0;
    check(got == 0, "benchmark pools are empty");
    return 0;
}

/* Every thread runs the same role: tpsRole = the TPS idle poll (popTP, steal,
 * popCodelet), else the MC idle poll (popCodelet). */
static PollStat bench(Group & g, bool tpsRole)
{
    std::atomic<int> go(0);
    BenchArg args[M_THREADS];
    pthread_t th[M_THREADS];
    for(unsigned k = 0; k < M_THREADS; ++k)
    {
        args[k].g = &g; args[k].k = k; args[k].tpsRole = tpsRole; args[k].go = &go;
        args[k].locks = 0; args[k].ns = 0;
        pthread_create(&th[k], 0, benchThread, &args[k]);
    }
    go.store(1);
    unsigned long long locks = 0;
    double ns = 0;
    for(unsigned k = 0; k < M_THREADS; ++k)
    {
        pthread_join(th[k], 0);
        locks += args[k].locks;
        ns += args[k].ns;
    }
    progress("M");
    PollStat s;
    s.locks = (double)locks / ((double)M_THREADS * M_POLLS);
    s.ns = ns / M_THREADS;
    return s;
}

static void rowM(void)
{
    RowScope r("M", "micro benchmark");
    Group off(true, false, STEAL_SIBLING), on(true, true, STEAL_SIBLING);
    const PollStat mcOff = bench(off, false), mcOn = bench(on, false);
    const PollStat tpOff = bench(off, true),  tpOn = bench(on, true);
    /* Micro scheduler idle poll under a node group + SIBLING: codelets_,
     * shared, node pool, sibling shared = 4 locks. The TP scheduler idle poll
     * adds popTP (node closure pool, ready_) and steal() (sibling ready_) =
     * 7. */
    check(mcOff.locks == 4.0, "MC idle poll, hint off: exactly 4 locks");
    check(tpOff.locks == 7.0, "TPS idle poll, hint off: exactly 7 locks");
    check(mcOn.locks == 0.0, "MC idle poll, hint on: 0 locks");
    check(tpOn.locks == 0.0, "TPS idle poll, hint on: 0 locks");
    char d[256];
    std::snprintf(d, sizeof(d), "64 thr x %d polls: MC off %.2f locks %.0f ns, on %.2f locks %.0f ns; TPS off %.2f locks %.0f ns, on %.2f locks %.0f ns",
                  M_POLLS, mcOff.locks, mcOff.ns, mcOn.locks, mcOn.ns, tpOff.locks, tpOff.ns, tpOn.locks, tpOn.ns);
    r.setDetail(d);
    r.setRounds((long long)M_THREADS * M_POLLS * 4);
}

static void rowL(void)
{
    RowScope r("L", "default-off");
    {
        ThreadAffinity a(g_s.mcPerTp, NSU, SPREAD, TPDYNAMIC, MCDYNAMIC);
        check(!a.idlePollHint(), "SPREAD: hint off");
    }
    {
        ThreadAffinity a(g_s.mcPerTp, NSU, COMPACT, TPDYNAMIC, MCDYNAMIC, true);
        check(!a.idlePollHint(), "COMPACT (not generated): hint off");
        check(a.generateMask(), "COMPACT generateMask");
        check(!a.idlePollHint(), "COMPACT: hint off");
        a.setIdlePollHint(true);
        check(a.idlePollHint(), "COMPACT + setIdlePollHint(true): on");
    }
    {
        ThreadAffinity a(g_s.mcPerTp, NSU, NUMA_PAIRED, TPDYNAMIC, MCDYNAMIC, true);
        check(!a.idlePollHint(), "NUMA_PAIRED before generateMask: off");
        check(a.generateMask(), "NUMA_PAIRED generateMask");
        check(a.idlePollHint(), "NUMA_PAIRED accepted: on");
        a.setIdlePollHint(false);
        check(!a.idlePollHint(), "NUMA_PAIRED + setIdlePollHint(false): off");
    }
    {
        /* A refused NUMA_PAIRED shape (half the schedulers) stays off. */
        ThreadAffinity a(g_s.mcPerTp, NNODE, NUMA_PAIRED, TPDYNAMIC, MCDYNAMIC, true);
        (void)a.generateMask();
        check(!a.idlePollHint(), "refused NUMA_PAIRED: off");
    }
    /* Legacy scheduler (no node group, no hint): counts never move and the
     * idle poll takes exactly the legacy lock (codelets_ = 1 for MC). */
    Group legacy(false, false, STEAL_LEGACY);
    Tok tk[64];
    for(int i = 0; i < 64; ++i) check(legacy.sched[i % NSU]->pushCodelet(&tk[i]), "legacy push");
    long long n = 0;
    for(unsigned s = 0; s < NSU; ++s) while(legacy.sched[s]->popCodelet()) ++n;
    check(n == 64, "legacy push/pop round trip");
    check(legacy.pending() == 0, "legacy: hint counts stay 0");
    for(unsigned s = 0; s < NSU; ++s) check(!legacy.sched[s]->idlePollHint(), "legacy scheduler: hint off");
    const PollStat mc = bench(legacy, false);
    check(mc.locks == 1.0, "legacy micro scheduler idle poll: exactly 1 lock (codelets_)");
    char d[200];
    std::snprintf(d, sizeof(d), "affinity defaults + overrides; legacy sched: counts 0, MC idle poll %.2f locks %.0f ns", mc.locks, mc.ns);
    r.setDetail(d);
    r.setRounds(64);
}

/* ------------------------------------------------------------------ */

template <class T>
static T * sharedRes(void)
{
    void * p = mmap(0, sizeof(T), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    if(p == MAP_FAILED) { std::printf("mmap failed\n"); std::exit(EXIT_CHECK); }
    std::memset(p, 0, sizeof(T));
    return static_cast<T *>(p);
}

/* fork + wait; the child never returns. Returns the child's exit code. */
template <class F>
static int inChild(F f, int timeoutS)
{
    std::fflush(stdout);
    pid_t pid = fork();
    if(pid == 0) { f(); _exit(EXIT_CHECK); }
    if(pid < 0) return -1;
    int st = 0;
    const double t0 = nowSec();
    for(;;)
    {
        pid_t w = waitpid(pid, &st, WNOHANG);
        if(w == pid) break;
        if(nowSec() - t0 > timeoutS) { kill(pid, SIGKILL); waitpid(pid, &st, 0); return EXIT_WATCHDOG; }
        usleep(20 * 1000);
    }
    return WIFEXITED(st) ? WEXITSTATUS(st) : 128 + (WIFSIGNALED(st) ? WTERMSIG(st) : 0);
}

struct IdleRun { IdleRes * r; bool paired; HintArg hint; void operator()() const { idleChild(paired, hint, r); } };
struct StressRun { StressRes * r; bool paired; HintArg hint; void operator()() const { stressChild(paired, hint, r); } };

static bool g_refused = false;

static IdleRes * runIdle(bool paired, HintArg hint)
{
    IdleRes * r = sharedRes<IdleRes>();
    IdleRun f = { r, paired, hint };
    int ec = inChild(f, 120);
    if(ec == EXIT_REFUSE || r->refused) g_refused = true;
    check(ec == EXIT_PASS && r->done, "idle child exited 0 with results");
    return r;
}

static double locksPerPoll(const IdleRes * r)
{
    return r->sleeps ? (double)r->locks / (double)r->sleeps : -1.0;
}

static double locksPerSecWorker(const IdleRes * r)
{
    return (r->secs > 0 && r->workers > 0) ? (double)r->locks / r->secs / r->workers : 0.0;
}

static void rowStress(const char * id, bool paired, HintArg hint)
{
    RowScope r(id, "in-runtime stress");
    StressRes * s = sharedRes<StressRes>();
    StressRun f = { s, paired, hint };
    int ec = inChild(f, 300);
    if(ec == EXIT_REFUSE || s->refused) g_refused = true;
    check(ec == EXIT_PASS && s->done, "stress child exited 0");
    check(s->hint == 1, "every TP scheduler has the hint on");
    check(s->fired == s->items, "fires == items");
    check(s->dup == 0, "no item fired twice");
    check(s->missing == 0, "no item lost");
    check(s->pending == 0, "every SU hint count back to 0");
    char d[240];
    std::snprintf(d, sizeof(d), "%s hint=%s(%d) %s: items=%lld fired=%lld dup=%lld missing=%lld pending=%llu",
                  paired ? "NUMA_PAIRED+SIBLING" : "COMPACT", hintName(hint), s->hint,
                  paired ? "4 scopes" : "local+pushCodeletTo", s->items, s->fired, s->dup, s->missing, s->pending);
    r.setDetail(d);
    r.setRounds(s->rounds);
    r.setOffTarget(s->dup + s->missing);
}

int main(void)
{
    if(int rc = skip_unless_paired(g_s, "poll-hint")) return rc;
    NSU = g_s.nsu; NNODE = g_s.nnode;
    if(NSU > MAX_SU || NNODE > MAX_NODE) { std::printf("poll-hint: SKIP -- too many SUs\n"); return EXIT_REFUSE; }
    PRODUCERS = (int)NSU; ITEMS = PRODUCERS * PER_PRODUCER;
    std::printf("poll-hint: llc_clusters=%u numa_nodes=%u\n", NSU, NNODE);

    /* Children first: the parent has no threads while it forks. */
    rowStress("A", true, HINT_DEFAULT);
    rowStress("B", false, HINT_ON);

    /* I */
    {
        RowScope r("I", "idle runtimes");
        IdleRes * pOff = runIdle(true, HINT_OFF);
        IdleRes * pOn  = runIdle(true, HINT_DEFAULT);
        IdleRes * cDef = runIdle(false, HINT_DEFAULT);
        IdleRes * cOn  = runIdle(false, HINT_ON);
        check(pOff->hint == 0 && cDef->hint == 0, "NUMA_PAIRED off and COMPACT default: hint off on every SU");
        check(pOn->hint == 1 && cOn->hint == 1, "NUMA_PAIRED default and COMPACT on: hint on on every SU");
        check(pOff->sleeps > 0 && pOn->sleeps > 0 && cDef->sleeps > 0 && cOn->sleeps > 0, "idle workers slept");
        check(locksPerPoll(pOff) >= 1.0, "NUMA_PAIRED off: >= 1 lock per idle poll");
        check(locksPerPoll(cDef) >= 1.0, "COMPACT default (no hint): >= 1 lock per idle poll");
        check(pOn->locks * 1000 <= pOn->sleeps, "NUMA_PAIRED default: <= 0.001 locks per idle poll");
        check(cOn->locks * 1000 <= cOn->sleeps, "COMPACT on: <= 0.001 locks per idle poll");
        char d[400];
        std::snprintf(d, sizeof(d),
                      "idle workers 1 s, locks/poll (locks/s/worker, polls/s/worker): NUMA_PAIRED off %.3f (%.0f, %.0f) default %.3f (%.0f, %.0f); "
                      "COMPACT default %.3f (%.0f, %.0f) on %.3f (%.0f, %.0f)",
                      locksPerPoll(pOff), locksPerSecWorker(pOff), pOff->sleeps / pOff->secs / pOff->workers,
                      locksPerPoll(pOn), locksPerSecWorker(pOn), pOn->sleeps / pOn->secs / pOn->workers,
                      locksPerPoll(cDef), locksPerSecWorker(cDef), cDef->sleeps / cDef->secs / cDef->workers,
                      locksPerPoll(cOn), locksPerSecWorker(cOn), cOn->sleeps / cOn->secs / cOn->workers);
        r.setDetail(d);
        r.setRounds((long long)(pOff->sleeps + pOn->sleeps + cDef->sleeps + cOn->sleeps));
    }

    start_watchdog(120);
    rowC();
    rowM();
    rowL();
    stop_watchdog();

    if(g_refused) { std::printf("SKIP: a child refused the topology or pinning\n"); return EXIT_REFUSE; }
    return print_rows("DARTS-RT-POLL-HINT") ? EXIT_CHECK : EXIT_PASS;
}
