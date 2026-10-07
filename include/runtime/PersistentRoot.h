/*
 * PersistentRoot.h -- helpers for a root TP that outlives gaps in its own
 * work, so one Runtime::run()/runPlaced() can drive a long sequence of
 * steps instead of re-entering the runtime per step. Header-only, C++11,
 * opt-in: nothing in the runtime uses these types.
 *
 *   PersistentRootTP  a ThreadedProcedure that takes one self-hold (incRef)
 *                     in its constructor, so a gap with no ready codelet and
 *                     no live child cannot drop its reference count to zero
 *                     and delete it. release_self() drops that hold.
 *   FinalOnce         Runtime::finalSignal.decDep() at most once per run:
 *                     CAS 0 -> 1, then decDep. reset() re-arms it (host
 *                     side, before the run).
 *   BodyDrain         counts bodies in flight; the host waits for zero
 *                     after the run returns.
 *
 * Launch: Runtime::runPlaced(tps, new tpClosure(&TPFactory<MyRoot>, sentinel))
 * with a caller-owned sentinel parent TP, so the root is a child (it is
 * deleted on its last reference) and the scheduler's delete rules apply.
 *
 * Termination protocol:
 *   1. The final codelet drops all of its holds (child joins, resident pins,
 *      ...) and calls release_self() on the root.
 *   2. It then calls FinalOnce::signal() as its LAST action: no member of the
 *      codelet, its TP or the root is touched afterwards, and every
 *      BodyDrain::exit() of the run happens before the signal. The root may
 *      be deleted by release_self() itself or later by the scheduler's
 *      post-fire decRef of the final codelet's TP, possibly after run()
 *      has returned on the host.
 *   3. The host thread, after runPlaced() returns: BodyDrain::wait_zero(),
 *      then application teardown, then the summary.
 *   - wait_zero() must never be called from inside a DARTS body (codelet
 *     fire or TP constructor): it would wait for itself.
 */
#ifndef DARTS_PERSISTENT_ROOT_H
#define DARTS_PERSISTENT_ROOT_H

/* std headers that use thread_local go before the DARTS headers
 * (threadlocal.h #defines thread_local). */
#include <atomic>
#include <thread>
#include <cstdint>

#include "ThreadedProcedure.h"
#include "Runtime.h"

namespace darts
{
    /* At most one finalSignal.decDep() per run. */
    class FinalOnce
    {
    public:
        FinalOnce(void) : fired_(0) { }
        /* Host side, before the run (no run may be using it). */
        void reset(void) { fired_.store(0, std::memory_order_release); }
        /* True for the one call that won the CAS 0 -> 1 and decremented the
         * final signal; false (and no decDep) for every later call. */
        bool signal(void)
        {
            uint32_t expected = 0;
            if(!fired_.compare_exchange_strong(expected, 1U, std::memory_order_acq_rel,
                                               std::memory_order_acquire))
                return false;
            Runtime::finalSignal.decDep();
            return true;
        }
        /* Diagnostic: whether signal() has fired since the last reset(). */
        bool fired(void) const { return fired_.load(std::memory_order_acquire) != 0; }
    private:
        FinalOnce(const FinalOnce &);
        FinalOnce & operator=(const FinalOnce &);
        std::atomic<uint32_t> fired_;
    };

    /* Bodies in flight. enter()/exit() inside DARTS bodies; wait_zero() on
     * the host only, after runPlaced() returns. */
    class BodyDrain
    {
    public:
        BodyDrain(void) : n_(0) { }
        void enter(void) { n_.fetch_add(1, std::memory_order_acq_rel); }
        void exit(void)  { n_.fetch_sub(1, std::memory_order_acq_rel); }
        void wait_zero(void) const
        {
            while(n_.load(std::memory_order_acquire) != 0)
                std::this_thread::yield();
        }
    private:
        BodyDrain(const BodyDrain &);
        BodyDrain & operator=(const BodyDrain &);
        std::atomic<int64_t> n_;
    };

    /* A root TP holding a reference on itself until release_self(). */
    class PersistentRootTP : public ThreadedProcedure
    {
    public:
        PersistentRootTP(void) : ThreadedProcedure() { incRef(); }
        /* Drop the self-hold; deletes the root when it was the last
         * reference. Call at most once; touch nothing of the root after. */
        void release_self(void)
        {
            if(decRef())
                delete this;
        }
    };
} // namespace darts

#endif /* DARTS_PERSISTENT_ROOT_H */
