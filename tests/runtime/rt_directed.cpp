/*
 * darts-rt-directed -- directed codelet placement (Codelet::setPlacedCluster,
 * TPScheduler::pushCodeletTo) on the default Runtime of any host.
 *
 *   A  ROUNDS rounds of one placed codelet per TP scheduler, enabled through
 *      Codelet::add, ThreadedProcedure::add and a dependence release
 *      (decDep from a codelet on another scheduler): every fire runs under
 *      the named TP scheduler
 *   B  a codelet placed on an index past the scheduler table is refused: it
 *      never fires, its status is DIRECTED_ENQUEUE_FAILED, the failure hook
 *      runs once with the target, directedRefused() grows by one and the
 *      owning TP is still destroyed exactly once
 *   C  pushCodeletTo from a thread that belongs to no DARTS scheduler is
 *      refused and counted
 *   D  ~Runtime returns
 */
#include "rt_test_util.h"

#include <cstring>

using namespace darts;
using namespace rt_test;

enum { ROUNDS = 500 };

static std::atomic<long long> g_left(0), g_fires(0), g_off(0);
static std::atomic<long long> g_hook(0), g_failedFires(0), g_tpDtor(0);
static std::atomic<unsigned long long> g_hookTarget(0);

static bool runningOn(unsigned target)
{
    TPScheduler * me = myThread.threadTPsched;
    return me && target < me->getNumTPSched() && me->getRuntimeTPSched(target) == me;
}

static void fired(unsigned target)
{
    if(!runningOn(target)) g_off.fetch_add(1);
    g_fires.fetch_add(1);
    progress();
    if(g_left.fetch_sub(1) == 1)
        Runtime::finalSignal.decDep();
}

/* A: Codelet::add of TP-less placed codelets. */
class LooseCodelet : public Codelet
{
public:
    unsigned target_;
    LooseCodelet(void) : Codelet(), target_(0) { }
    virtual void fire(void) { fired(target_); }
};

static std::vector<LooseCodelet> g_loose;

class LooseRoundTP : public ThreadedProcedure
{
public:
    LooseRoundTP(void) : ThreadedProcedure()
    {
        for(unsigned i = 0; i < g_loose.size(); ++i)
        {
            g_loose[i].initCodelet(0, 0, NULL, SHORTWAIT);
            g_loose[i].setPlacedCluster(i);
            g_loose[i].target_ = i;
            g_loose[i].add();
        }
    }
};

/* A: ThreadedProcedure::add of ready codelets and a dependence release. */
class OwnedCodelet : public Codelet
{
public:
    unsigned target_;
    Codelet * release_;
    OwnedCodelet(uint32_t dep, ThreadedProcedure * tp, unsigned target, Codelet * release)
        : Codelet(dep, dep, tp, SHORTWAIT), target_(target), release_(release) { }
    virtual void fire(void)
    {
        Codelet * r = release_;
        fired(target_);
        if(r) r->decDep();
    }
};

class OwnedRoundTP : public ThreadedProcedure
{
public:
    std::vector<OwnedCodelet *> cds_;
    OwnedRoundTP(unsigned n) : ThreadedProcedure()
    {
        /* For every scheduler i: a dependent codelet placed on i, released by
         * a ready codelet placed on (i + 1) % n. */
        for(unsigned i = 0; i < n; ++i)
            cds_.push_back(new OwnedCodelet(1, this, i, NULL));
        for(unsigned i = 0; i < n; ++i)
            cds_.push_back(new OwnedCodelet(0, this, (i + 1) % n, cds_[i]));
        for(unsigned i = 0; i < 2 * n; ++i)
            cds_[i]->setPlacedCluster(cds_[i]->target_);
        for(unsigned i = n; i < 2 * n; ++i)
            add(cds_[i]);
    }
    ~OwnedRoundTP(void)
    {
        for(size_t i = 0; i < cds_.size(); ++i) delete cds_[i];
    }
};

/* B: one refused codelet next to one that ends the run. */
class RefusedCodelet : public Codelet
{
public:
    RefusedCodelet(ThreadedProcedure * tp) : Codelet(0, 0, tp, SHORTWAIT) { }
    virtual void fire(void) { g_failedFires.fetch_add(1); }
    virtual void onDirectedEnqueueFailure(uint64_t target)
    {
        g_hook.fetch_add(1);
        g_hookTarget.store(target);
    }
};

class DoneCodelet : public Codelet
{
public:
    DoneCodelet(ThreadedProcedure * tp) : Codelet(0, 0, tp, SHORTWAIT) { }
    virtual void fire(void) { progress(); Runtime::finalSignal.decDep(); }
};

class RefuseTP : public ThreadedProcedure
{
public:
    RefusedCodelet bad_;
    DoneCodelet    done_;
    RefuseTP(unsigned badTarget) : ThreadedProcedure(), bad_(this), done_(this)
    {
        bad_.setPlacedCluster(badTarget);
        add(&bad_);
        add(&done_);
    }
    ~RefuseTP(void) { g_tpDtor.fetch_add(1); }
};

class NopCodelet : public Codelet
{
public:
    NopCodelet(void) : Codelet(0, 0, NULL, SHORTWAIT) { }
    virtual void fire(void) { }
};

int main(void)
{
    start_watchdog(60);
    Runtime * rt = new Runtime();
    const unsigned n = rt->getNumTPS();
    std::printf("directed: tp_schedulers=%u mc_per_tp=%u\n", n, rt->getNumMCS());
    ThreadedProcedure * sentinel = new ThreadedProcedure();

    /* A */
    {
        RowScope r("A", "placed codelets fire under their TP scheduler");
        g_loose.resize(n);
        for(int round = 0; round < ROUNDS; ++round)
        {
            g_left.store(n);
            rt->run(new tpClosure(&TPFactory<LooseRoundTP>, sentinel));
            g_left.store(2 * n);
            rt->run(new tpClosure1<unsigned>(&TPFactory<OwnedRoundTP, unsigned>, sentinel, n));
            progress("A");
        }
        check(g_fires.load() == (long long)ROUNDS * 3 * n, "every placed codelet fired once per round");
        check(g_off.load() == 0, "every placed codelet fired on its TP scheduler");
        char d[160];
        std::snprintf(d, sizeof(d), "placed codelets fire under their TP scheduler (%lld fires)",
                      g_fires.load());
        r.setDetail(d);
        r.setRounds(ROUNDS);
        r.setOffTarget(g_off.load());
    }

    /* B */
    {
        RowScope r("B", "placement past the scheduler table is refused, not rerouted");
        const uint64_t refused0 = TPScheduler::directedRefused();
        rt->run(new tpClosure1<unsigned>(&TPFactory<RefuseTP, unsigned>, sentinel, n));
        for(int i = 0; i < 1000 && g_tpDtor.load() == 0; ++i) usleep(1000);
        check(g_hook.load() == 1, "failure hook ran once");
        check(g_hookTarget.load() == n, "failure hook got the requested target");
        check(g_failedFires.load() == 0, "refused codelet never fired");
        check(TPScheduler::directedRefused() == refused0 + 1, "directedRefused() grew by one");
        check(g_tpDtor.load() == 1, "owning TP destroyed exactly once");
        r.setRounds(1);
    }

    /* C */
    {
        RowScope r("C", "pushCodeletTo from a non-DARTS thread is refused");
        const uint64_t refused0 = TPScheduler::directedRefused();
        NopCodelet cd;
        bool pushed = true;
        std::thread t([&]() { pushed = TPScheduler::pushCodeletTo(0, &cd); });
        t.join();
        check(!pushed, "push refused");
        check(TPScheduler::directedRefused() == refused0 + 1, "refusal counted");
        r.setRounds(1);
    }

    /* D */
    {
        RowScope r("D", "~Runtime returned");
        progress("shutdown");
        delete rt;
        delete sentinel;
        r.setRounds(1);
    }

    stop_watchdog();
    return print_rows("DARTS-RT-DIRECTED") ? EXIT_CHECK : EXIT_PASS;
}
