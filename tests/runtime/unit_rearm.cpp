/*
 * darts-unit-rearm -- SyncSlot::rearm and the Codelet re-arm API. No
 * Runtime: nothing here is scheduled.
 *
 *   A  SyncSlot::rearm: from 0 it succeeds and restores reset_; from a
 *      non-zero counter it fails and changes nothing; resetCounter unchanged
 *   B  8 threads x 1e6 rounds of decCounter + rearm on one SyncSlot(8, 8):
 *      per round exactly one decCounter returns true and its rearm succeeds,
 *      so the count of true returns == rounds
 *   C  8 threads race rearm on one fired slot, 1e5 times: exactly one wins
 *      per round (a CAS, not a store)
 *   D  Codelet::rearm clears DIRECTED_ENQUEUE_FAILED back to the original
 *      status, re-arms the counter, and refuses (status untouched) while the
 *      counter is non-zero
 *   E  origStatus_ is recorded by the 4-argument constructor and by
 *      initCodelet
 *   F  tryDecDep: false while dependencies remain; an explicitly placed
 *      codelet released off any DARTS thread is refused (false, status
 *      DIRECTED_ENQUEUE_FAILED), and decDep() keeps doing the same
 *
 * Exit 0 only when every row PASSes.
 */
#include "rt_test_util.h"

using namespace darts;
using namespace rt_test;

class NopCodelet : public Codelet
{
public:
    NopCodelet(uint32_t dep, uint32_t res, uint32_t stat) : Codelet(dep, res, NULL, stat) { }
    NopCodelet(void) : Codelet() { }
    virtual void fire(void) { }
};

enum { NTHREAD = 8, ROUNDS_B = 1000000, ROUNDS_C = 100000 };

int main(void)
{
    start_watchdog(100);

    /* A */
    {
        RowScope r("A", "SyncSlot::rearm: 0 -> reset_, non-zero refused");
        SyncSlot s(1, 1);
        check(!s.rearm(), "rearm at counter 1 refused");
        check(s.getCounter() == 1, "refused rearm leaves counter 1");
        check(s.decCounter(), "decCounter 1 -> 0 returns true");
        check(s.getCounter() == 0, "counter 0 after the release");
        check(s.rearm(), "rearm at 0 succeeds");
        check(s.getCounter() == 1, "rearm restores reset_ (1)");
        check(!s.rearm(), "second rearm refused");
        SyncSlot t(3, 5);
        check(!t.rearm(), "rearm at counter 3 refused");
        check(t.decCounter() == false && t.decCounter() == false && t.decCounter() == true,
              "3 releases, the last returns true");
        check(t.rearm() && t.getCounter() == 5, "rearm restores reset_ (5), not the first dep");
        t.resetCounter();
        check(t.getCounter() == 5, "resetCounter unchanged");
        r.setRounds(1);
    }

    /* B */
    {
        RowScope r("B", "8 threads x 1e6 rounds decCounter+rearm, true returns == rounds");
        SyncSlot s(NTHREAD, NTHREAD);
        std::atomic<long long> round(0), trues(0), rearmFail(0);
        std::vector<std::thread> th;
        for(int i = 0; i < NTHREAD; ++i)
            th.push_back(std::thread([&]() {
                for(long long k = 0; k < ROUNDS_B; ++k)
                {
                    while(round.load(std::memory_order_acquire) < k) { }
                    if(s.decCounter())
                    {
                        trues.fetch_add(1);
                        if(!s.rearm()) rearmFail.fetch_add(1);
                        round.fetch_add(1, std::memory_order_release);
                    }
                    if((k & 0xffff) == 0) progress("B");
                }
            }));
        for(size_t i = 0; i < th.size(); ++i) th[i].join();
        check(trues.load() == ROUNDS_B, "count of true decCounter returns == rounds");
        check(rearmFail.load() == 0, "every rearm after the releasing decCounter succeeded");
        check(s.getCounter() == NTHREAD, "slot ends re-armed at reset_");
        char d[128];
        std::snprintf(d, sizeof(d), "8 threads x 1e6 rounds decCounter+rearm, trues=%lld", trues.load());
        r.setDetail(d);
        r.setRounds(ROUNDS_B);
        r.setOffTarget(ROUNDS_B - trues.load());
    }

    /* C */
    {
        RowScope r("C", "8 threads race rearm on a fired slot: one winner per round");
        SyncSlot s(0, 1);
        std::atomic<long long> round(0), arrived(0), wins(0), bad(0);
        std::vector<std::thread> th;
        for(int i = 0; i < NTHREAD; ++i)
            th.push_back(std::thread([&]() {
                for(long long k = 0; k < ROUNDS_C; ++k)
                {
                    while(round.load(std::memory_order_acquire) < k) { }
                    if(s.rearm()) wins.fetch_add(1);
                    /* the last arrival checks the round and fires the slot again */
                    if(arrived.fetch_add(1) + 1 == (long long)NTHREAD * (k + 1))
                    {
                        if(wins.load() != k + 1) bad.fetch_add(1);
                        if(!s.decCounter()) bad.fetch_add(1);
                        round.fetch_add(1, std::memory_order_release);
                    }
                    if((k & 0xfff) == 0) progress("C");
                }
            }));
        for(size_t i = 0; i < th.size(); ++i) th[i].join();
        check(wins.load() == ROUNDS_C, "exactly one rearm per round succeeded");
        check(bad.load() == 0, "no round had zero or two winners");
        r.setRounds(ROUNDS_C);
        r.setOffTarget(bad.load());
    }

    /* D */
    {
        RowScope r("D", "Codelet::rearm clears DIRECTED_ENQUEUE_FAILED");
        NopCodelet c(1, 1, MEMORY);
        c.notifyDirectedEnqueueFailure(7);
        check(c.directedEnqueueFailed(), "notifyDirectedEnqueueFailure sets the terminal status");
        check(!c.rearm(), "rearm refused while the counter is 1");
        check(c.getStatus() == Codelet::DIRECTED_ENQUEUE_FAILED, "refused rearm leaves the status");
        check(c.getSyncSlot()->decCounter(), "release the slot to 0");
        check(c.rearm(), "rearm at 0 succeeds");
        check(c.getStatus() == MEMORY, "status restored to the original (MEMORY)");
        check(!c.directedEnqueueFailed(), "directedEnqueueFailed() false after rearm");
        check(c.getCounter() == 1, "counter re-armed to reset_");
        /* a status set by the user (not the failure marker) is left alone */
        c.setStatus(LOCAL);
        check(c.getSyncSlot()->decCounter() && c.rearm(), "second cycle re-arms");
        check(c.getStatus() == LOCAL, "rearm restores only DIRECTED_ENQUEUE_FAILED");
        r.setRounds(2);
    }

    /* E */
    {
        RowScope r("E", "origStatus_ from the constructor and from initCodelet");
        NopCodelet a(0, 1, LOCAL);
        check(a.getOrigStatus() == LOCAL, "4-argument constructor records LOCAL");
        NopCodelet b;
        check(b.getOrigStatus() == NIL, "default constructor records NIL");
        b.initCodelet(0, 1, NULL, MEMORY);
        check(b.getOrigStatus() == MEMORY, "initCodelet records MEMORY");
        b.notifyDirectedEnqueueFailure(3);
        check(b.rearm() && b.getStatus() == MEMORY, "initCodelet-built codelet restores MEMORY");
        a.notifyDirectedEnqueueFailure(3);
        check(a.rearm() && a.getStatus() == LOCAL, "constructor-built codelet restores LOCAL");
        r.setRounds(2);
    }

    /* F: off any DARTS thread pushCodeletTo refuses, so the placed path is
     * observable without a Runtime. */
    {
        RowScope r("F", "tryDecDep: not-ready false; refused placed enqueue false");
        NopCodelet c(2, 2, SHORTWAIT);
        c.setPlacedCluster(3);
        check(!c.tryDecDep(), "first of two releases: not enqueued");
        check(!c.directedEnqueueFailed(), "no failure before the slot is ready");
        check(!c.tryDecDep(), "ready but refused off-runtime: not enqueued");
        check(c.directedEnqueueFailed(), "refusal recorded as DIRECTED_ENQUEUE_FAILED");
        check(!c.tryDecDep(), "a third release on a zero counter does nothing");
        check(c.rearm() && c.getStatus() == SHORTWAIT && c.getCounter() == 2,
              "rearm restores the status and the counter");
        c.decDep();
        c.decDep();
        check(c.directedEnqueueFailed(), "virtual decDep() keeps the same behaviour");
        r.setRounds(2);
    }

    stop_watchdog();
    return print_rows("DARTS-UNIT-REARM") ? EXIT_CHECK : EXIT_PASS;
}
