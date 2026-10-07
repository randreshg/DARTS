/*
 * darts-rt-wake -- event wake of resident TPs and the create path on a
 * NUMA_PAIRED Runtime.
 *
 * Every TP is a 1-stage resident TP: the entry re-arms, marks RUNNING, takes
 * the payload by generation, marks IDLE and then performs its release (a
 * pending-count decrement) as the last statement.
 *
 *   A  up to 8 routers (one per node) x 100k wakes to 128 TPs over all SUs, one
 *      global generation counter: fires == accepted (WOKEN_OK), every wake
 *      accepted, BUSY_PROTOCOL (contended) or DUPLICATE (stale), never
 *      MISSING; every accepted entry release was enqueued (tryDecDep)
 *   B  a duplicate wake in the same generation is counted and never fires:
 *      while WOKEN/RUNNING, and again after IDLE; a stale (older) generation
 *      likewise; a duplicate mailbox post is refused; the next generation
 *      fires
 *   C  create path: 500 keys through pushCodeletTo(su, create) (create_ns:
 *      issue -> inserted), 500 keys created inline by the router
 *      (create_inline_ns), then one wake per key (wake_ns: lookup_and_wake
 *      -> entry fire); every created TP fires once per wake
 *   D  two creators race per key (200 keys, the two SUs of the last node):
 *      one INSERTED, one
 *      EXISTS + drop_hold, exactly one destructor for the loser
 *   E  magic_ poison check at every fire, all rows; teardown destroys
 *      every TP exactly once; ~Runtime returns
 *
 * Exit 3 on a stall; skipped (exit 4) unless the host has two LLC clusters
 * per NUMA node and this process may use all their cpus.
 */
#include <atomic>
#include <thread>
#include <mutex>
#include <algorithm>
#include <functional>
#include <time.h>
#include "ResidentRuntime.h"
#include "rt_test_util.h"

using namespace darts;
using namespace rt_test;

enum { MAX_SU = 256, MAX_ROUTER = 8, WAKES_A = 100000, NTP_A = 128, NKEY_C = 1000,
       NKEY_D = 200 };

static PairedShape g_s;
static unsigned    NSU = 0, NNODE = 0, NROUTER = 0;

static const uint32_t ALIVE = 0xa11fe5edu, DEAD = 0xdeadbeefu;

static inline uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static std::atomic<long long> g_ctor(0), g_dtor(0), g_poison(0), g_rearmFail(0), g_stateErr(0),
                              g_mboxErr(0), g_fires(0);
static std::atomic<long long> g_pending(0);

static void release_pending(void)
{
    if(g_pending.fetch_sub(1) == 1)
        Runtime::finalSignal.decDep();
}

struct Payload { uint64_t gen; uint64_t t0; };

class WakeTP;

class EntryCodelet : public Codelet
{
public:
    WakeTP * tp_;
    EntryCodelet(WakeTP * tp, unsigned su) : Codelet(1, 1, (ThreadedProcedure *)0, SHORTWAIT), tp_(tp)
    { setPlacedCluster(su); }
    virtual void fire(void);
};

class WakeTP : public ResidentTP
{
public:
    volatile uint32_t     magic_;
    WakeMailbox<Payload>  mbox_;
    std::atomic<long long> fires_;
    uint64_t              wakeNs_;
    EntryCodelet          entry_;
    WakeTP(ResidentAnchor * a, unsigned su)
        : ResidentTP(a, su, 4096), magic_(ALIVE), fires_(0), wakeNs_(0), entry_(this, su)
    {
        entry_.setTP(this);
        g_ctor.fetch_add(1);
    }
    virtual ~WakeTP() { magic_ = DEAD; g_dtor.fetch_add(1); }
    virtual Codelet & entry() { return entry_; }
};

void EntryCodelet::fire(void)
{
    WakeTP * t = tp_;
    if(!rearm()) g_rearmFail.fetch_add(1);
    if(t->magic_ != ALIVE) { g_poison.fetch_add(1); return; }
    if(!t->mark_running()) g_stateErr.fetch_add(1);
    Payload p;
    if(!t->mbox_.take(t->generation(), p)) g_mboxErr.fetch_add(1);
    else if(p.t0) t->wakeNs_ = now_ns() - p.t0;
    t->fires_.fetch_add(1);
    g_fires.fetch_add(1);
    if(!t->mark_idle()) g_stateErr.fetch_add(1);
    release_pending();                          /* last statement */
}

static ResidentKey key(uint16_t l, uint16_t e, uint16_t s)
{
    ResidentKey k; k.group = l; k.item = e; k.shard = s; return k;
}

static ResidentAnchor * g_anchor = 0;
static ResidentTable *  g_tab[MAX_SU];

/* lookup_and_wake + post + unpin + release. g_pending is raised before the
 * release so the fire can never drive it to zero early. */
static ResidentTable::WakeResult wake(ResidentTable & tab, const ResidentKey & k, uint64_t gen,
                                      uint64_t t0, std::atomic<long long> * notEnqueued)
{
    ResidentTP * tp = 0;
    ResidentTable::WakeResult r = tab.lookup_and_wake(k, gen, &tp);
    if(r != ResidentTable::WOKEN_OK) return r;
    WakeTP * w = static_cast<WakeTP *>(tp);
    Payload p; p.gen = gen; p.t0 = t0;
    if(!w->mbox_.post(gen, p)) g_mboxErr.fetch_add(1);
    tab.unpin(k, tp);
    g_pending.fetch_add(1);
    if(!w->entry().tryDecDep() && notEnqueued) notEnqueued->fetch_add(1);
    return r;
}

/* ------------------------------------------------------------------ */
/* Row A */

static std::atomic<uint64_t> g_gen(0);
static std::atomic<long long> g_accepted(0), g_busy(0), g_dup(0), g_missing(0), g_notEnq(0);

class RouterA : public Codelet
{
public:
    unsigned id_;
    RouterA(void) : Codelet(1, 1, NULL, SHORTWAIT), id_(0) { }
    virtual void fire(void)
    {
        uint64_t x = 0x9e3779b97f4a7c15ULL * (id_ + 1);
        long long acc = 0, busy = 0, dup = 0, miss = 0;
        for(int i = 0; i < WAKES_A; ++i)
        {
            x ^= x << 13; x ^= x >> 7; x ^= x << 17;
            const unsigned tpi = (unsigned)(x % NTP_A);
            const uint64_t gen = g_gen.fetch_add(1) + 1;
            ResidentTable::WakeResult r = wake(*g_tab[tpi % NSU], key(0, (uint16_t)tpi, 0), gen, 0, &g_notEnq);
            if(r == ResidentTable::WOKEN_OK) ++acc;
            else if(r == ResidentTable::BUSY_PROTOCOL) ++busy;
            else if(r == ResidentTable::DUPLICATE) ++dup;
            else ++miss;
            if((i & 0xfff) == 0) progress("A");
        }
        g_accepted.fetch_add(acc); g_busy.fetch_add(busy); g_dup.fetch_add(dup); g_missing.fetch_add(miss);
        release_pending();
    }
};

static RouterA g_routerA[MAX_ROUTER];

class RootA : public ThreadedProcedure
{
public:
    RootA(void) : ThreadedProcedure()
    {
        for(unsigned i = 0; i < NROUTER; ++i) g_routerA[i].decDep();
    }
};

/* ------------------------------------------------------------------ */
/* Row B (one codelet on SU 0 driving a TP on SU 5) */

static WakeTP * g_tpB = 0;
static std::atomic<long long> g_dupB(0), g_staleB(0), g_okB(0), g_errB(0), g_firesAfterDup(0);
static std::atomic<int> g_dupPostRefused(0);

static bool wait_fires(WakeTP * t, long long n, int ms)
{
    for(int i = 0; i < ms * 10; ++i)
    {
        if(t->fires_.load() >= n && t->state() == ResidentTP::IDLE) return true;
        usleep(100);
    }
    return false;
}

class DriverB : public Codelet
{
public:
    DriverB(void) : Codelet(1, 1, NULL, SHORTWAIT) { }
    virtual void fire(void)
    {
        ResidentTable & tab = *g_tab[5];
        const ResidentKey k = key(1, 5, 0);
        ResidentTable::WakeResult r;
        r = wake(tab, k, 10, 0, 0);
        if(r == ResidentTable::WOKEN_OK) g_okB.fetch_add(1); else g_errB.fetch_add(1);
        /* same generation while WOKEN/RUNNING (or already IDLE): counted, no fire */
        r = wake(tab, k, 10, 0, 0);
        if(r == ResidentTable::DUPLICATE) g_dupB.fetch_add(1); else g_errB.fetch_add(1);
        Payload p; p.gen = 10; p.t0 = 0;
        if(!g_tpB->mbox_.post(10, p)) g_dupPostRefused.fetch_add(1);
        if(!wait_fires(g_tpB, 1, 5000)) g_errB.fetch_add(1);
        progress("B");
        r = wake(tab, k, 10, 0, 0);                 /* again after IDLE */
        if(r == ResidentTable::DUPLICATE) g_dupB.fetch_add(1); else g_errB.fetch_add(1);
        r = wake(tab, k, 9, 0, 0);                  /* stale */
        if(r == ResidentTable::DUPLICATE) g_staleB.fetch_add(1); else g_errB.fetch_add(1);
        usleep(20000);
        g_firesAfterDup.store(g_tpB->fires_.load());
        r = wake(tab, k, 11, 0, 0);                 /* the next generation fires */
        if(r == ResidentTable::WOKEN_OK) g_okB.fetch_add(1); else g_errB.fetch_add(1);
        if(!wait_fires(g_tpB, 2, 5000)) g_errB.fetch_add(1);
        release_pending();
    }
};

static DriverB g_driverB;

class RootB : public ThreadedProcedure
{
public:
    RootB(void) : ThreadedProcedure() { g_driverB.decDep(); }
};

/* ------------------------------------------------------------------ */
/* Row C */

struct CreateSlot;
class CreateC : public Codelet
{
public:
    CreateSlot * s_;
    CreateC(void) : Codelet(0, 0, NULL, SHORTWAIT), s_(0) { }
    virtual void fire(void);
};

struct CreateSlot
{
    ResidentKey k;
    unsigned    su;
    uint64_t    t0, createNs;
    WakeTP *    tp;
    CreateC     cd;
};

static CreateSlot g_slot[NKEY_C];
static std::atomic<long long> g_createdC(0), g_errC(0);

void CreateC::fire(void)
{
    CreateSlot * s = s_;
    WakeTP * tp = new WakeTP(g_anchor, s->su);
    if(g_tab[s->su]->insert(s->k, tp) != ResidentTable::INSERTED) { tp->drop_hold(); g_errC.fetch_add(1); return; }
    s->createNs = now_ns() - s->t0;
    s->tp = tp;
    if(wake(*g_tab[s->su], s->k, 1, 0, &g_notEnq) != ResidentTable::WOKEN_OK) g_errC.fetch_add(1);
    g_createdC.fetch_add(1);
}

class DriverC : public Codelet
{
public:
    DriverC(void) : Codelet(1, 1, NULL, SHORTWAIT) { }
    virtual void fire(void)
    {
        /* create: half through pushCodeletTo(su, create), half inline */
        for(int i = 0; i < NKEY_C; ++i)
        {
            CreateSlot & s = g_slot[i];
            ResidentTP * none = 0;
            if(g_tab[s.su]->lookup_and_wake(s.k, 1, &none) != ResidentTable::MISSING) g_errC.fetch_add(1);
            if(i < NKEY_C / 2)
            {
                s.t0 = now_ns();
                if(!TPScheduler::pushCodeletTo(s.su, &s.cd)) g_errC.fetch_add(1);
            }
            else
            {
                s.t0 = now_ns();
                WakeTP * tp = new WakeTP(g_anchor, s.su);
                if(g_tab[s.su]->insert(s.k, tp) != ResidentTable::INSERTED) { tp->drop_hold(); g_errC.fetch_add(1); continue; }
                s.createNs = now_ns() - s.t0;
                s.tp = tp;
                if(wake(*g_tab[s.su], s.k, 1, 0, &g_notEnq) != ResidentTable::WOKEN_OK) g_errC.fetch_add(1);
                g_createdC.fetch_add(1);
            }
            if((i & 0x3f) == 0) progress("C create");
        }
        for(int w = 0; w < 50000 && g_createdC.load() < NKEY_C; ++w) usleep(100);
        for(int i = 0; i < NKEY_C; ++i)
            if(!g_slot[i].tp || !wait_fires(g_slot[i].tp, 1, 5000)) { g_errC.fetch_add(1); }
        progress("C wake");
        /* one timed wake per key */
        for(int i = 0; i < NKEY_C; ++i)
        {
            CreateSlot & s = g_slot[i];
            if(wake(*g_tab[s.su], s.k, 2, now_ns(), &g_notEnq) != ResidentTable::WOKEN_OK) g_errC.fetch_add(1);
            if(s.tp && !wait_fires(s.tp, 2, 5000)) g_errC.fetch_add(1);   /* one wake in flight at a time */
        }
        release_pending();
    }
};

static DriverC g_driverC;

class RootC : public ThreadedProcedure
{
public:
    RootC(void) : ThreadedProcedure() { g_driverC.decDep(); }
};

/* ------------------------------------------------------------------ */
/* Row D */

class RaceCreate : public Codelet
{
public:
    unsigned key_, su_;
    RaceCreate(void) : Codelet(0, 0, NULL, SHORTWAIT), key_(0), su_(0) { }
    virtual void fire(void);
};

static RaceCreate g_raceD[2][NKEY_D];
static std::atomic<long long> g_insD(0), g_existsD(0), g_otherD(0), g_dtorAtExists(0);

void RaceCreate::fire(void)
{
    const ResidentKey k = key(3, (uint16_t)key_, 0);
    WakeTP * tp = new WakeTP(g_anchor, 7);
    ResidentTable::InsertResult r = g_tab[7]->insert(k, tp);
    if(r == ResidentTable::INSERTED) g_insD.fetch_add(1);
    else if(r == ResidentTable::EXISTS)
    {
        long long d0 = g_dtor.load();
        tp->drop_hold();                        /* never scheduled: deleted here */
        if(g_dtor.load() > d0) g_dtorAtExists.fetch_add(1);
        g_existsD.fetch_add(1);
    }
    else { tp->drop_hold(); g_otherD.fetch_add(1); }
    release_pending();
}

class RootD : public ThreadedProcedure
{
public:
    RootD(void) : ThreadedProcedure()
    {
        for(int i = 0; i < NKEY_D; ++i)
        {
            TPScheduler::pushCodeletTo(NSU - 2, &g_raceD[0][i]);
            TPScheduler::pushCodeletTo(NSU - 1, &g_raceD[1][i]);
        }
    }
};

/* ------------------------------------------------------------------ */

static uint64_t median(std::vector<uint64_t> v)
{
    if(v.empty()) return 0;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

int main(void)
{
    if(int rc = skip_unless_paired(g_s, "wake")) return rc;
    NSU = g_s.nsu; NNODE = g_s.nnode;
    NROUTER = NNODE < (unsigned)MAX_ROUTER ? NNODE : (unsigned)MAX_ROUTER;
    if(NSU > MAX_SU) { std::printf("wake: SKIP -- more than %d SUs\n", MAX_SU); return EXIT_REFUSE; }
    std::printf("wake: llc_clusters=%u numa_nodes=%u routers=%u\n", NSU, NNODE, NROUTER);
    start_watchdog(60);

    ThreadAffinity affin(g_s.mcPerTp, NSU, NUMA_PAIRED, TPDYNAMIC, MCDYNAMIC, true);
    if(!affin.generateMask())
    {
        std::printf("SKIP: NUMA_PAIRED generateMask() failed\n");
        return EXIT_REFUSE;
    }
    require_cpus(affin);
    Runtime * rt = new Runtime(&affin);
    if(!rt->constructionOk())
    {
        std::printf("SKIP: Runtime constructionOk() == false\n");
        return EXIT_REFUSE;
    }
    ThreadedProcedure * sentinel = new ThreadedProcedure();
    g_anchor = new ResidentAnchor();
    for(unsigned su = 0; su < NSU; ++su) g_tab[su] = new ResidentTable(1ULL << 40, NULL);

    /* A */
    {
        RowScope r("A", "routers x 100k wakes to 128 TPs: fires == accepted");
        std::vector<WakeTP *> tps;
        for(unsigned i = 0; i < NTP_A; ++i)
        {
            WakeTP * tp = new WakeTP(g_anchor, i % NSU);
            tps.push_back(tp);
            check(g_tab[i % NSU]->insert(key(0, (uint16_t)i, 0), tp) == ResidentTable::INSERTED, "insert");
        }
        for(unsigned i = 0; i < NROUTER; ++i)
        {
            g_routerA[i].id_ = i;
            g_routerA[i].setPlacedCluster(2 * i);
        }
        g_pending.store(NROUTER);
        g_fires.store(0);
        rt->run(new tpClosure(&TPFactory<RootA>, sentinel));
        const long long total = (long long)NROUTER * WAKES_A;
        check(g_fires.load() == g_accepted.load(), "fires == accepted");
        check(g_accepted.load() + g_busy.load() + g_dup.load() == total, "accepted + busy + stale == wakes");
        check(g_missing.load() == 0, "no MISSING");
        check(g_notEnq.load() == 0, "every accepted release enqueued (tryDecDep == true)");
        check(g_accepted.load() > total / 4, "most wakes accepted");
        long long perTp = 0;
        for(size_t i = 0; i < tps.size(); ++i) perTp += tps[i]->fires_.load();
        check(perTp == g_accepted.load(), "per-TP fires sum to accepted");
        char d[200];
        std::snprintf(d, sizeof(d), "8x100k wakes -> 128 TPs: accepted=%lld fires=%lld busy=%lld stale=%lld",
                      g_accepted.load(), g_fires.load(), g_busy.load(), g_dup.load());
        r.setDetail(d);
        r.setRounds(total);
        r.setOffTarget(g_accepted.load() - g_fires.load());
    }

    /* B */
    {
        RowScope r("B", "duplicate wake in one generation: counted, no fire");
        g_tpB = new WakeTP(g_anchor, 5);
        check(g_tab[5]->insert(key(1, 5, 0), g_tpB) == ResidentTable::INSERTED, "insert");
        g_driverB.setPlacedCluster(0);
        g_pending.store(1);
        size_t dupBefore = g_tab[5]->duplicates();
        rt->run(new tpClosure(&TPFactory<RootB>, sentinel));
        check(g_errB.load() == 0, "no unexpected wake result or timeout");
        check(g_okB.load() == 2, "generations 10 and 11 woken");
        check(g_dupB.load() == 2 && g_staleB.load() == 1, "2 duplicates (WOKEN/RUNNING and IDLE) + 1 stale counted");
        check(g_tab[5]->duplicates() - dupBefore == 3, "table duplicates() counted 3");
        check(g_dupPostRefused.load() == 1, "duplicate mailbox post refused");
        check(g_firesAfterDup.load() == 1, "no fire from a duplicate");
        check(g_tpB->fires_.load() == 2, "exactly 2 fires");
        char d[160];
        std::snprintf(d, sizeof(d), "duplicate/stale wakes: dup=%lld stale=%lld fires=%lld",
                      g_dupB.load(), g_staleB.load(), g_tpB->fires_.load());
        r.setDetail(d);
        r.setRounds(5);
    }

    /* C */
    {
        RowScope r("C", "create path: pushCodeletTo(su, create) and inline, wake timing");
        for(int i = 0; i < NKEY_C; ++i)
        {
            CreateSlot & s = g_slot[i];
            s.su = (unsigned)(i % NSU);
            s.k = key(2, (uint16_t)(i % 128), (uint16_t)(i / 128));
            s.t0 = s.createNs = 0; s.tp = 0;
            s.cd.s_ = &s;
        }
        g_driverC.setPlacedCluster(0);
        g_pending.store(1);
        rt->run(new tpClosure(&TPFactory<RootC>, sentinel));
        std::vector<uint64_t> cPush, cInline, wk;
        long long fires = 0;
        for(int i = 0; i < NKEY_C; ++i)
        {
            CreateSlot & s = g_slot[i];
            (i < NKEY_C / 2 ? cPush : cInline).push_back(s.createNs);
            if(s.tp) { wk.push_back(s.tp->wakeNs_); fires += s.tp->fires_.load(); }
        }
        check(g_errC.load() == 0, "no create/wake error");
        check(g_createdC.load() == NKEY_C, "1000 TPs created and inserted");
        check(fires == 2LL * NKEY_C, "each created TP fired once per wake (2 wakes)");
        char d[200];
        std::snprintf(d, sizeof(d), "create_ns=%llu create_inline_ns=%llu wake_ns=%llu (medians, n=%d/%d/%d)",
                      (unsigned long long)median(cPush), (unsigned long long)median(cInline),
                      (unsigned long long)median(wk), (int)cPush.size(), (int)cInline.size(), (int)wk.size());
        r.setDetail(d);
        r.setRounds(NKEY_C);
        std::printf("timing: create_ns=%llu create_inline_ns=%llu wake_ns=%llu\n",
                    (unsigned long long)median(cPush), (unsigned long long)median(cInline),
                    (unsigned long long)median(wk));
    }

    /* D */
    {
        RowScope r("D", "two creators per key: INSERTED + EXISTS/drop_hold, one destructor");
        for(int i = 0; i < NKEY_D; ++i)
            for(int j = 0; j < 2; ++j) { g_raceD[j][i].key_ = i; g_raceD[j][i].su_ = 6 + j; }
        g_pending.store(2 * NKEY_D);
        rt->run(new tpClosure(&TPFactory<RootD>, sentinel));
        check(g_insD.load() == NKEY_D && g_existsD.load() == NKEY_D, "one INSERTED and one EXISTS per key");
        check(g_otherD.load() == 0, "no NO_ROOM");
        check(g_dtorAtExists.load() == NKEY_D, "EXISTS + drop_hold destroyed the loser at once");
        r.setRounds(NKEY_D);
    }

    /* E */
    {
        RowScope r("E", "poison check, teardown: one destructor per TP, ~Runtime");
        for(unsigned su = 0; su < NSU; ++su) g_tab[su]->teardown();
        for(int i = 0; i < 2000 && g_dtor.load() < g_ctor.load(); ++i) usleep(1000);
        check(g_poison.load() == 0, "0 fires on a destroyed TP");
        check(g_rearmFail.load() == 0, "every rearm at fire start succeeded");
        check(g_stateErr.load() == 0, "every mark_running/mark_idle succeeded");
        check(g_mboxErr.load() == 0, "every mailbox post/take matched");
        check(g_dtor.load() == g_ctor.load(), "every TP built was destroyed exactly once");
        progress("shutdown");
        for(unsigned su = 0; su < NSU; ++su) delete g_tab[su];
        delete rt;
        delete g_anchor;
        delete sentinel;
        char d[160];
        std::snprintf(d, sizeof(d), "built=%lld destroyed=%lld poison=%lld", g_ctor.load(), g_dtor.load(),
                      g_poison.load());
        r.setDetail(d);
        r.setRounds(g_ctor.load());
    }

    stop_watchdog();
    return print_rows("DARTS-RT-WAKE") ? EXIT_CHECK : EXIT_PASS;
}
