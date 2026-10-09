/*
 * darts-rt-resident -- resident TPs re-armed across wakes, the eviction /
 * wake interleaving and the idle ordering on a NUMA_PAIRED Runtime.
 *
 * Every TP is a 3-stage resident TP (entry -> s2 -> s3, each placed on its
 * SU). Each stage re-arms itself first; the entry takes the wake payload
 * from the mailbox by generation and marks RUNNING; s3 loads the join from
 * the TP, marks IDLE and releases the join as its last statement. Wakes go
 * only through ResidentTable::lookup_and_wake.
 *
 *   A  every SU x 64 resident TPs, 10,000 wake rounds (one router per SU,
 *      re-armed, woken by its 64 TPs): every stage fires 10,000 times per
 *      TP, no destructor during the rounds, scheduler references drained
 *      after run(), teardown runs exactly one destructor per TP
 *   B  up to 8 lanes (one per NUMA node) x 1000 iterations racing evict
 *      (sibling SU) against
 *      lookup_and_wake (owner SU). evict returns EVICTED, BUSY or NOT_FOUND. A MISSING/EVICTING wake goes to the create
 *      path (pushCodeletTo(owner, create), insert, wake). Every wake fires
 *      or is routed to create, lost wakes == 0, and every TP ever built is
 *      destroyed exactly once
 *   C  one chain per SU x 1e5 back-to-back steps, each waking the same key as
 *      soon as the previous step's s3 released it: BUSY_PROTOCOL == 0
 *   D  magic_ poison check at every fire of every row: 0 poisoned fires
 *   E  ~Runtime with nothing outstanding returned
 *
 * Exit 3 on a stall; skipped (exit 4) unless the host has two LLC clusters
 * per NUMA node and this process may use all their cpus.
 */
#include <atomic>
#include <thread>
#include <mutex>
#include <functional>
#include <time.h>
#include "ResidentRuntime.h"
#include "rt_test_util.h"

using namespace darts;
using namespace rt_test;

enum { MAX_SU = 256, NTP_A = 64, ROUNDS_A = 10000, MAX_LANES_B = 8, ITERS_B = 1000,
       STEPS_C = 100000, MAX_TPS = 1 << 16 };

static PairedShape g_s;
static unsigned    NSU = 0, NNODE = 0, LANES_B = 0;

static const uint32_t ALIVE = 0xa11fe5edu, DEAD = 0xdeadbeefu;

static std::atomic<long long> g_ctor(0), g_dtor(0), g_poison(0), g_rearmFail(0), g_stateErr(0),
                              g_mboxErr(0), g_fire[3];
static std::atomic<int> g_dtorById[MAX_TPS];
static std::atomic<int> g_left(0);

static void finish_one(void)
{
    if(g_left.fetch_sub(1) == 1)
        Runtime::finalSignal.decDep();
}

struct Payload { uint64_t gen; Codelet * join; };

class StageTP;

class Stage : public Codelet
{
public:
    StageTP * tp_;
    int       idx_;
    Stage(StageTP * tp, int idx);
    virtual void fire(void);
};

class StageTP : public ResidentTP
{
public:
    volatile uint32_t     magic_;
    int                   id_;
    WakeMailbox<Payload>  mbox_;
    Codelet *             join_;
    std::atomic<long long> entries_;
    Stage                 s1_, s2_, s3_;

    StageTP(ResidentAnchor * a, unsigned su, size_t bytes)
        : ResidentTP(a, su, bytes), magic_(ALIVE), id_((int)g_ctor.fetch_add(1)), join_(0), entries_(0),
          s1_(this, 0), s2_(this, 1), s3_(this, 2) { }
    virtual ~StageTP()
    {
        magic_ = DEAD;
        if(id_ >= 0 && id_ < MAX_TPS) g_dtorById[id_].fetch_add(1);
        g_dtor.fetch_add(1);
    }
    virtual Codelet & entry() { return s1_; }
};

Stage::Stage(StageTP * tp, int idx) : Codelet(1, 1, tp, SHORTWAIT), tp_(tp), idx_(idx)
{
    setPlacedCluster(tp->su());
}

void Stage::fire(void)
{
    StageTP * t = tp_;
    if(!rearm()) g_rearmFail.fetch_add(1);           /* rule 1: re-arm first */
    if(t->magic_ != ALIVE) { g_poison.fetch_add(1); return; }
    g_fire[idx_].fetch_add(1);
    if(idx_ == 0)
    {
        if(!t->mark_running()) g_stateErr.fetch_add(1);
        Payload p;
        if(!t->mbox_.take(t->generation(), p) || p.gen != t->generation()) { g_mboxErr.fetch_add(1); }
        t->join_ = p.join;
        t->entries_.fetch_add(1);
        t->s2_.decDep();
        return;
    }
    if(idx_ == 1)
    {
        t->s3_.decDep();
        return;
    }
    Codelet * j = t->join_;                         /* load before the TP may be re-woken */
    if(!t->mark_idle()) g_stateErr.fetch_add(1);     /* rule 2: idle before the release */
    if(j) j->decDep();                               /* rule 3: last statement */
}

/* Post, unpin, then release the entry. unpin may precede the release: a
 * WOKEN TP is not evictable, so the memo hold keeps it alive. */
static bool wake(ResidentTable & tab, const ResidentKey & k, uint64_t gen, Codelet * join,
                 ResidentTable::WakeResult * res)
{
    ResidentTP * tp = 0;
    ResidentTable::WakeResult r = tab.lookup_and_wake(k, gen, &tp);
    if(res) *res = r;
    if(r != ResidentTable::WOKEN_OK) return false;
    StageTP * st = static_cast<StageTP *>(tp);
    Payload p; p.gen = gen; p.join = join;
    if(!st->mbox_.post(gen, p)) g_mboxErr.fetch_add(1);
    tab.unpin(k, tp);
    st->entry().decDep();
    return true;
}

static ResidentKey key(uint16_t l, uint16_t e, uint16_t s)
{
    ResidentKey k; k.group = l; k.item = e; k.shard = s; return k;
}

static ResidentAnchor * g_anchor = 0;
static ResidentTable *  g_tab[MAX_SU];

/* Waits (host) until every TP in the tables holds only its memo reference:
 * scheduler epilogues after the last fire may still be running when run()
 * returns. */
static bool drain_refs(const std::vector<StageTP *> & tps, int ms)
{
    for(int i = 0; i < ms * 10; ++i)
    {
        bool ok = true;
        for(size_t k = 0; k < tps.size() && ok; ++k) ok = tps[k]->ref_count() == 1;
        if(ok) return true;
        usleep(100);
    }
    return false;
}

/* ------------------------------------------------------------------ */
/* Row A */

static std::atomic<long long> g_wakeErrA(0);

class RouterA : public Codelet
{
public:
    unsigned su_;
    uint64_t round_;
    RouterA(void) : Codelet(1, NTP_A, NULL, SHORTWAIT), su_(0), round_(0) { }
    virtual void fire(void)
    {
        if(!rearm()) g_rearmFail.fetch_add(1);
        const uint64_t gen = ++round_;
        const unsigned su = su_;
        if(gen > (uint64_t)ROUNDS_A) { finish_one(); return; }
        if((gen & 0xff) == 0) progress("A");
        ResidentTable & tab = *g_tab[su];
        for(unsigned i = 0; i < NTP_A; ++i)
            if(!wake(tab, key(0, (uint16_t)i, (uint16_t)su), gen, this, 0))
            {
                g_wakeErrA.fetch_add(1);
                decDep();                       /* keep the round count honest */
            }
        /* nothing after the last release: locals only */
    }
};

static RouterA g_routerA[MAX_SU];

class RootA : public ThreadedProcedure
{
public:
    RootA(void) : ThreadedProcedure()
    {
        for(unsigned su = 0; su < NSU; ++su) g_routerA[su].decDep();
    }
};

/* ------------------------------------------------------------------ */
/* Row B */

struct LaneB;

class StarterB : public Codelet
{ public: LaneB * l_; StarterB() : Codelet(1, 2, NULL, SHORTWAIT), l_(0) { } virtual void fire(void); };
class EvictB : public Codelet
{ public: LaneB * l_; EvictB() : Codelet(1, 1, NULL, SHORTWAIT), l_(0) { } virtual void fire(void); };
class WakeB : public Codelet
{ public: LaneB * l_; WakeB() : Codelet(1, 1, NULL, SHORTWAIT), l_(0) { } virtual void fire(void); };
class CreateB : public Codelet
{ public: LaneB * l_; CreateB() : Codelet(0, 0, NULL, SHORTWAIT), l_(0) { } virtual void fire(void); };

struct LaneB
{
    unsigned    owner, evictor;
    ResidentKey k;
    uint64_t    iter;
    StarterB    start;
    EvictB      ev;
    WakeB       wk;
    CreateB     cr;
    std::atomic<long long> evicted, busy, notFound, woken, routed, created, exists, protocol;
};

static LaneB g_lane[MAX_LANES_B];

void StarterB::fire(void)
{
    LaneB * l = l_;
    if(!rearm()) g_rearmFail.fetch_add(1);
    const uint64_t it = ++l->iter;
    if(it > (uint64_t)ITERS_B) { finish_one(); return; }
    if((it & 0x3f) == 0) progress("B");
    l->ev.decDep();
    l->wk.decDep();                             /* last: the starter re-fires only after both */
}

void EvictB::fire(void)
{
    LaneB * l = l_;
    if(!rearm()) g_rearmFail.fetch_add(1);
    ResidentTable::EvictResult r = g_tab[l->owner]->evict(l->k);
    if(r == ResidentTable::EVICTED) l->evicted.fetch_add(1);
    else if(r == ResidentTable::BUSY) l->busy.fetch_add(1);
    else l->notFound.fetch_add(1);              /* evicted earlier, not yet recreated */
    l->start.decDep();
}

void WakeB::fire(void)
{
    LaneB * l = l_;
    if(!rearm()) g_rearmFail.fetch_add(1);
    ResidentTable::WakeResult r;
    if(wake(*g_tab[l->owner], l->k, l->iter, &l->start, &r)) { l->woken.fetch_add(1); return; }
    if(r == ResidentTable::MISSING || r == ResidentTable::EVICTING)
    {
        l->routed.fetch_add(1);
        if(!TPScheduler::pushCodeletTo(l->owner, &l->cr)) { l->protocol.fetch_add(1); l->start.decDep(); }
        return;
    }
    l->protocol.fetch_add(1);                   /* BUSY_PROTOCOL / DUPLICATE: report, keep going */
    l->start.decDep();
}

void CreateB::fire(void)
{
    LaneB * l = l_;
    ResidentTable & tab = *g_tab[l->owner];
    for(int attempt = 0; attempt < 4; ++attempt)
    {
        StageTP * tp = new StageTP(g_anchor, l->owner, 4096);
        ResidentTable::InsertResult ir = tab.insert(l->k, tp);
        if(ir == ResidentTable::INSERTED) l->created.fetch_add(1);
        else { tp->drop_hold(); if(ir == ResidentTable::EXISTS) l->exists.fetch_add(1); else tab.make_room(4096, 0); }
        ResidentTable::WakeResult r;
        if(wake(tab, l->k, l->iter, &l->start, &r)) return;
        if(r != ResidentTable::MISSING && r != ResidentTable::EVICTING) break;
    }
    l->protocol.fetch_add(1);
    l->start.decDep();
}

class RootB : public ThreadedProcedure
{
public:
    RootB(void) : ThreadedProcedure()
    {
        for(unsigned i = 0; i < LANES_B; ++i) g_lane[i].start.decDep();
    }
};

/* ------------------------------------------------------------------ */
/* Row C */

static std::atomic<long long> g_busyC(0), g_otherC(0), g_stepsC(0);

class StepC : public Codelet
{
public:
    unsigned su_;
    uint64_t step_;
    StepC(void) : Codelet(1, 1, NULL, SHORTWAIT), su_(0), step_(0) { }
    virtual void fire(void)
    {
        if(!rearm()) g_rearmFail.fetch_add(1);
        const uint64_t gen = ++step_;
        const unsigned su = su_;
        if(gen > (uint64_t)STEPS_C) { finish_one(); return; }
        if((gen & 0x3ff) == 0) progress("C");
        g_stepsC.fetch_add(1);
        ResidentTable::WakeResult r;
        if(wake(*g_tab[su], key(1, 0, (uint16_t)su), gen, this, &r)) return;
        if(r == ResidentTable::BUSY_PROTOCOL) g_busyC.fetch_add(1); else g_otherC.fetch_add(1);
        decDep();                               /* report and take the next step */
    }
};

static StepC g_stepC[MAX_SU];

class RootC : public ThreadedProcedure
{
public:
    RootC(void) : ThreadedProcedure()
    {
        for(unsigned su = 0; su < NSU; ++su) g_stepC[su].decDep();
    }
};

/* ------------------------------------------------------------------ */

int main(void)
{
    if(int rc = skip_unless_paired(g_s, "resident")) return rc;
    NSU = g_s.nsu; NNODE = g_s.nnode;
    LANES_B = NNODE < (unsigned)MAX_LANES_B ? NNODE : (unsigned)MAX_LANES_B;
    if(NSU > MAX_SU) { std::printf("resident: SKIP -- more than %d SUs\n", MAX_SU); return EXIT_REFUSE; }
    std::printf("resident: llc_clusters=%u numa_nodes=%u\n", NSU, NNODE);
    start_watchdog(60);
    for(int i = 0; i < 3; ++i) g_fire[i].store(0);
    for(int i = 0; i < MAX_TPS; ++i) g_dtorById[i].store(0);

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
        RowScope r("A", "every SU x 64 resident 3-stage TPs, 10,000 wake rounds");
        std::vector<StageTP *> tps;
        for(unsigned su = 0; su < NSU; ++su)
        {
            g_routerA[su].su_ = su;
            g_routerA[su].setPlacedCluster(su);
            for(unsigned i = 0; i < NTP_A; ++i)
            {
                StageTP * tp = new StageTP(g_anchor, su, 1 << 20);
                tps.push_back(tp);
                check(g_tab[su]->insert(key(0, (uint16_t)i, (uint16_t)su), tp) == ResidentTable::INSERTED,
                      "insert resident TP");
            }
        }
        long long d0 = g_dtor.load();
        g_left.store(NSU);
        rt->run(new tpClosure(&TPFactory<RootA>, sentinel));
        long long dRun = g_dtor.load() - d0;
        const long long expect = (long long)NSU * NTP_A * ROUNDS_A;
        check(dRun == 0, "no destructor during the rounds");
        check(g_fire[0].load() == expect && g_fire[1].load() == expect && g_fire[2].load() == expect,
              "every stage fired 10,000 times per TP");
        long long badEntries = 0;
        for(size_t i = 0; i < tps.size(); ++i)
            if(tps[i]->entries_.load() != ROUNDS_A || tps[i]->generation() != (uint64_t)ROUNDS_A
               || tps[i]->state() != ResidentTP::IDLE) ++badEntries;
        check(badEntries == 0, "each TP: 10,000 entries, generation 10,000, IDLE");
        check(g_wakeErrA.load() == 0, "every lookup_and_wake was WOKEN_OK");
        check(drain_refs(tps, 2000), "scheduler references drained: ref_ == 1 (memo hold) on every TP");
        long long d1 = g_dtor.load();
        size_t dropped = 0;
        for(unsigned su = 0; su < NSU; ++su) dropped += g_tab[su]->teardown();
        check(dropped == tps.size() && g_dtor.load() - d1 == (long long)tps.size(),
              "teardown: exactly one destructor per TP");
        char d[200];
        std::snprintf(d, sizeof(d), "%u SU x 64 resident 3-stage TPs, 10,000 rounds, fires=%lld/%lld/%lld", NSU,
                      g_fire[0].load(), g_fire[1].load(), g_fire[2].load());
        r.setDetail(d);
        r.setRounds(ROUNDS_A);
        r.setOffTarget(expect - g_fire[2].load());
    }

    /* B */
    {
        RowScope r("B", "evict vs lookup_and_wake race, lanes x 1000 iterations");
        for(int i = 0; i < 3; ++i) g_fire[i].store(0);
        long long c0 = g_ctor.load();
        for(unsigned i = 0; i < LANES_B; ++i)
        {
            LaneB & l = g_lane[i];
            l.owner = 2 * i; l.evictor = 2 * i + 1;
            l.k = key(2, (uint16_t)i, 0);
            l.iter = 0;
            l.start.l_ = &l; l.ev.l_ = &l; l.wk.l_ = &l; l.cr.l_ = &l;
            l.start.setPlacedCluster(l.owner);
            l.ev.setPlacedCluster(l.evictor);
            l.wk.setPlacedCluster(l.owner);
            l.cr.setPlacedCluster(l.owner);
            l.evicted = 0; l.busy = 0; l.notFound = 0; l.woken = 0; l.routed = 0; l.created = 0; l.exists = 0; l.protocol = 0;
        }
        g_left.store(LANES_B);
        rt->run(new tpClosure(&TPFactory<RootB>, sentinel));
        long long ev = 0, busy = 0, nf = 0, woken = 0, routed = 0, created = 0, prot = 0;
        for(unsigned i = 0; i < LANES_B; ++i)
        {
            LaneB & l = g_lane[i];
            ev += l.evicted; busy += l.busy; nf += l.notFound; woken += l.woken; routed += l.routed;
            created += l.created; prot += l.protocol;
            check(l.evicted + l.busy + l.notFound == ITERS_B, "lane: every evict EVICTED, BUSY or NOT_FOUND");
            check(l.woken + l.routed == ITERS_B, "lane: every wake woken or routed to create");
            check(l.created >= l.routed, "lane: every routed wake created a TP");
        }
        const long long total = (long long)LANES_B * ITERS_B;
        long long lost = total - g_fire[0].load();
        check(prot == 0, "no protocol errors");
        check(lost == 0, "lost wakes == 0 (entry fires == iterations)");
        check(g_fire[2].load() == total, "every woken TP completed");
        check(ev > 0 && busy > 0, "both race outcomes observed (EVICTED and BUSY)");
        /* Drop what is still resident, then wait for the scheduler epilogues
         * that may still own the last reference of an evicted TP. */
        for(unsigned su = 0; su < NSU; ++su) g_tab[su]->teardown();
        for(int i = 0; i < 2000 && g_dtor.load() < g_ctor.load(); ++i) usleep(1000);
        long long once = 0, notOnce = 0;
        for(long long id = c0; id < g_ctor.load() && id < MAX_TPS; ++id)
            (g_dtorById[id].load() == 1 ? once : notOnce)++;
        check(g_ctor.load() < MAX_TPS, "TP ids within the tracking table");
        check(notOnce == 0, "exactly one destructor per TP built in row B");
        check(once == g_ctor.load() - c0 && once >= created, "every TP built in row B was destroyed");
        char d[240];
        std::snprintf(d, sizeof(d), "race %ux1000: evicted=%lld busy=%lld not_found=%lld woken=%lld routed=%lld created=%lld lost=%lld", LANES_B,
                      ev, busy, nf, woken, routed, created, lost);
        r.setDetail(d);
        r.setRounds(total);
        r.setOffTarget(lost);
    }

    /* C */
    {
        RowScope r("C", "back-to-back steps on one key, BUSY_PROTOCOL == 0");
        for(int i = 0; i < 3; ++i) g_fire[i].store(0);
        std::vector<StageTP *> tps;
        for(unsigned su = 0; su < NSU; ++su)
        {
            g_stepC[su].su_ = su;
            g_stepC[su].setPlacedCluster(su);
            StageTP * tp = new StageTP(g_anchor, su, 4096);
            tps.push_back(tp);
            g_tab[su]->insert(key(1, 0, (uint16_t)su), tp);
        }
        g_left.store(NSU);
        rt->run(new tpClosure(&TPFactory<RootC>, sentinel));
        const long long total = (long long)NSU * STEPS_C;
        check(g_busyC.load() == 0, "BUSY_PROTOCOL == 0");
        check(g_otherC.load() == 0, "no MISSING/DUPLICATE results");
        check(g_fire[2].load() == total, "every step completed");
        check(drain_refs(tps, 2000), "scheduler references drained");
        size_t dropped = 0;
        for(unsigned su = 0; su < NSU; ++su) dropped += g_tab[su]->teardown();
        check(dropped == tps.size(), "teardown drops one TP per SU");
        char d[200];
        std::snprintf(d, sizeof(d), "%u chains x 1e5 steps: busy_protocol=%lld steps=%lld", NSU,
                      g_busyC.load(), g_stepsC.load());
        r.setDetail(d);
        r.setRounds(total);
        r.setOffTarget(g_busyC.load());
    }

    /* D */
    {
        RowScope r("D", "magic_ poison check at every fire, all rows");
        check(g_poison.load() == 0, "0 fires on a destroyed TP");
        check(g_rearmFail.load() == 0, "every rearm at fire start succeeded");
        check(g_stateErr.load() == 0, "every mark_running/mark_idle transition succeeded");
        check(g_mboxErr.load() == 0, "every mailbox post/take matched its generation");
        check(g_dtor.load() == g_ctor.load(), "every TP built was destroyed");
        char d[160];
        std::snprintf(d, sizeof(d), "poison=%lld rearm_fail=%lld state_err=%lld mbox_err=%lld",
                      g_poison.load(), g_rearmFail.load(), g_stateErr.load(), g_mboxErr.load());
        r.setDetail(d);
        r.setRounds(g_ctor.load());
        r.setOffTarget(g_poison.load());
    }

    /* E */
    {
        RowScope r("E", "~Runtime with nothing outstanding returned");
        progress("shutdown");
        for(unsigned su = 0; su < NSU; ++su) delete g_tab[su];
        delete rt;
        delete g_anchor;
        delete sentinel;
        r.setRounds(1);
    }

    stop_watchdog();
    return print_rows("DARTS-RT-RESIDENT") ? EXIT_CHECK : EXIT_PASS;
}
