/*
 * darts-rt-sticky-place -- opt-in sticky place<> (ThreadAffinity::
 * setStickyPlacement) on a small COMPACT runtime of any host.
 *
 *   A  default: stickyPlacement() is off on every scheduler and place<> still
 *      delivers every closure (expansions may be stolen; their number is
 *      only reported)
 *   B  setStickyPlacement(true): N closures place<>d on TP scheduler 0 while
 *      the other schedulers are idle and stealing all expand on scheduler 0;
 *      placedSteals() does not move
 */
#include <chrono>

#include "rt_test_util.h"

using namespace darts;
using namespace rt_test;

enum { NTPS = 4, MCPERTP = 1, N = 4000 };

static std::atomic<long long> g_left(0), g_expanded(0), g_off(0);

static unsigned myIndex(void)
{
    TPScheduler * me = myThread.threadTPsched;
    return me ? me->getClusterIndex() : ~0u;
}

static void spinUs(unsigned us)
{
    std::chrono::steady_clock::time_point end =
        std::chrono::steady_clock::now() + std::chrono::microseconds(us);
    while(std::chrono::steady_clock::now() < end) { }
}

class DoneCodelet : public Codelet
{
public:
    DoneCodelet(ThreadedProcedure * tp) : Codelet(0, 0, tp, SHORTWAIT) { }
    virtual void fire(void)
    {
        progress();
        if(g_left.fetch_sub(1) == 1)
            Runtime::finalSignal.decDep();
    }
};

/* Expanded from a place<>d closure: records where the expansion ran. */
class LeafTP : public ThreadedProcedure
{
public:
    DoneCodelet done_;
    LeafTP(unsigned target) : ThreadedProcedure(), done_(this)
    {
        g_expanded.fetch_add(1);
        if(myIndex() != target) g_off.fetch_add(1);
        spinUs(5);
        add(&done_);
    }
};

class RootTP : public ThreadedProcedure
{
public:
    RootTP(void) : ThreadedProcedure()
    {
        for(unsigned i = 0; i < N; ++i)
            place<LeafTP, unsigned>(0, this, 0u);
    }
};

static long long runRound(Runtime * rt, ThreadedProcedure * sentinel)
{
    g_left.store(N);
    g_expanded.store(0);
    g_off.store(0);
    rt->run(new tpClosure(&TPFactory<RootTP>, sentinel));
    check(g_expanded.load() == N, "every placed closure expanded once");
    return g_off.load();
}

/* From the host thread, which runs TP scheduler 0. */
static bool allSticky(bool want)
{
    TPScheduler * me = myThread.threadTPsched;
    if(!me) return false;
    for(unsigned i = 0; i < me->getNumTPSched(); ++i)
        if(static_cast<TPScheduler *>(me->getRuntimeTPSched(i))->stickyPlacement() != want)
            return false;
    return true;
}

int main(void)
{
    start_watchdog(60);
    hwloc::AbstractMachine am(false);
    if(am.getTotalNbUnits() < NTPS * (1 + MCPERTP))
    {
        std::printf("SKIP: needs %d cpus\n", NTPS * (1 + MCPERTP));
        return EXIT_REFUSE;
    }

    /* A */
    {
        ThreadAffinity affin(MCPERTP, NTPS, COMPACT, TPDYNAMIC, MCDYNAMIC, false);
        check(affin.generateMask(), "COMPACT mask");
        require_cpus(affin);
        RowScope r("A", "default: place<> not sticky, every closure delivered");
        check(!affin.stickyPlacement(), "ThreadAffinity::stickyPlacement() defaults to false");
        Runtime * rt = new Runtime(&affin);
        ThreadedProcedure * sentinel = new ThreadedProcedure();
        check(allSticky(false), "no scheduler is sticky");
        long long off = runRound(rt, sentinel);
        char d[160];
        std::snprintf(d, sizeof(d), "default: place<> not sticky, every closure delivered (%lld stolen)", off);
        r.setDetail(d);
        r.setRounds(N);
        progress("A");
        delete rt;
        delete sentinel;
    }

    /* B */
    {
        ThreadAffinity affin(MCPERTP, NTPS, COMPACT, TPDYNAMIC, MCDYNAMIC, false);
        check(affin.generateMask(), "COMPACT mask");
        affin.setStickyPlacement(true);
        Runtime * rt = new Runtime(&affin);
        ThreadedProcedure * sentinel = new ThreadedProcedure();
        {
            RowScope r("B", "sticky: place<>(0) always expands on scheduler 0");
            check(allSticky(true), "every scheduler is sticky");
            const uint64_t st0 = TPScheduler::placedSteals();
            long long off = 0;
            for(int round = 0; round < 5; ++round)
            {
                off += runRound(rt, sentinel);
                progress("B");
            }
            check(off == 0, "no placed closure expanded elsewhere");
            check(TPScheduler::placedSteals() == st0, "placedSteals() unchanged");
            r.setRounds(5 * N);
            r.setOffTarget(off);
        }
        delete rt;
        delete sentinel;
    }

    stop_watchdog();
    return print_rows("DARTS-RT-STICKY-PLACE") ? EXIT_CHECK : EXIT_PASS;
}
