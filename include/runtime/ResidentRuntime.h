/*
 * Resident threaded procedures, event wake and a per-SU resident table.
 * Header-only, C++11.
 *
 * Opt-in: nothing in the runtime includes this header or creates a
 * ResidentTP.
 *
 * A program that runs the same small graph again and again (one per step,
 * keyed by what it computes) can keep its TPs alive between invocations and
 * wake them with a payload instead of creating, running and destroying a TP
 * per step. Each SU keeps a ResidentTable of its live TPs under a memory
 * budget; a pluggable eviction policy makes room.
 *
 * A resident TP stays alive between invocations. The memo table holds the
 * TP's base reference (ref_ == 1 from ThreadedProcedure's constructor; no
 * extra incRef), so the scheduler's per-fire decRef never frees it. Its
 * codelets re-arm their sync slots and the TP idles until the next wake.
 * Eviction drops the memo hold.
 *
 * Include order: this header includes its std headers first. Include it (or
 * <atomic>, <mutex>, <thread>) before darts.h, because
 * include/threadlocal/threadlocal.h #defines thread_local.
 *
 * State machine of a ResidentTP (state_):
 *
 *     IDLE --lookup_and_wake--> WOKEN --mark_running--> RUNNING
 *       ^                                                  |
 *       +------------------------mark_idle-----------------+
 *     IDLE --evict / make_room (under the table lock)--> EVICTING (erased)
 *
 * IDLE -> WOKEN and IDLE -> EVICTING both happen under the table's mutex, so
 * a wake and an eviction of the same key are totally ordered.
 *
 * Generations: every accepted wake of a key carries a generation strictly
 * greater than the last accepted one (start at 1; 0 is never accepted). A
 * wake whose generation is not newer is DUPLICATE (duplicate or stale) and
 * changes nothing. A recreated TP starts over at 0.
 *
 * Protocol rules:
 *  1. Re-arm at the START of a codelet's own fire(): Codelet::rearm().
 *  2. mark_idle() (and any per-invocation bookkeeping) comes BEFORE the
 *     final successor or join release.
 *  3. That release is the LAST statement of the body; load whatever it
 *     needs (e.g. the join pointer) into locals before mark_idle(). No member
 *     of the TP or of the codelet may be touched after it: the TP may already
 *     be woken again, or evicted.
 *  4. Wake only through ResidentTable::lookup_and_wake. MISSING or EVICTING
 *     -> the create path (construct, insert, then lookup_and_wake again).
 *     BUSY_PROTOCOL (the TP is WOKEN or RUNNING) is a protocol error: report
 *     it and fail the run. DUPLICATE is a duplicate or stale wake: count it,
 *     do not fire.
 *     After WOKEN_OK: post the payload to the TP's mailbox, then release the
 *     entry codelet (decDep/tryDecDep) and unpin. unpin may come before the
 *     release: a WOKEN TP is not evictable, so the memo hold keeps it alive.
 *  5. Build resident TPs directly (new). With TPFactory, take the hold in the
 *     constructor (incRef), since TPFactory drops the base reference.
 *  6. Never wake through Codelet::add(&other).
 *  7. Never decDep an unplaced codelet from a non-DARTS thread.
 *  8. Never parent a resident TP to a per-step TP: its parent is a
 *     ResidentAnchor, which children never decrement or delete.
 *  9. Objects shared by several resident TPs (e.g. a join they all release)
 *     must not be owned by an evictable TP.
 * 10. Exactly one producer posts to a TP's mailbox per generation (the one
 *     whose lookup_and_wake returned WOKEN_OK).
 */
#ifndef DARTS_RESIDENT_RUNTIME_H
#define DARTS_RESIDENT_RUNTIME_H

#include <atomic>
#include <mutex>
#include <thread>
#include <functional>
#include <unordered_map>
#include <utility>
#include <vector>
#include <stddef.h>
#include <stdint.h>

#include "ThreadedProcedure.h"
#include "Codelet.h"
#include "Atomics.h"

namespace darts {

/* The parent of every resident TP. Its parentTP_ is NULL, so a child's
 * destructor never decrements or deletes it (ThreadedProcedure.cpp), and the
 * scheduler's checkParent() is true for its children (they are released by
 * reference count). The application owns and deletes the anchor. */
class ResidentAnchor : public ThreadedProcedure
{
public:
    ResidentAnchor(void) : ThreadedProcedure() { parentTP_ = NULL; }
};

/* What a resident TP computes, as three application-defined 16-bit fields
 * (for example a stage, an item and a shard of the item). */
struct ResidentKey
{
    uint16_t group, item, shard;
    bool operator==(const ResidentKey & o) const
    {
        return group == o.group && item == o.item && shard == o.shard;
    }
};

struct ResidentKeyHash
{
    size_t operator()(const ResidentKey & k) const
    {
        uint64_t x = ((uint64_t)k.group << 32) | ((uint64_t)k.item << 16) | (uint64_t)k.shard;
        x ^= x >> 33; x *= 0xff51afd7ed558ccdULL; x ^= x >> 33;   /* murmur3 finaliser */
        return (size_t)x;
    }
};

class ResidentTable;

class ResidentTP : public ThreadedProcedure
{
public:
    enum State : uint32_t { IDLE = 0, WOKEN = 1, RUNNING = 2, EVICTING = 3 };

    /* parentTP_ = anchor (non-NULL). The memo hold is the base ref_ == 1:
     * no extra incRef. `su` is the SU the derived class places its codelets
     * on; `bytes` is what the TP charges against that SU's budget. */
    ResidentTP(ResidentAnchor * anchor, unsigned su, size_t bytes)
        : ThreadedProcedure(), state_(IDLE), gen_(0), su_(su), bytes_(bytes)
    {
        parentTP_ = anchor;
    }
    virtual ~ResidentTP() {}

    /* WOKEN -> RUNNING (entry fire). false if the TP was not WOKEN. */
    bool mark_running(void)
    {
        uint32_t s = WOKEN;
        return state_.compare_exchange_strong(s, (uint32_t)RUNNING, std::memory_order_acq_rel);
    }
    /* RUNNING -> IDLE. Must precede the final release (rule 2). false if the
     * TP was not RUNNING. */
    bool mark_idle(void)
    {
        uint32_t s = RUNNING;
        return state_.compare_exchange_strong(s, (uint32_t)IDLE, std::memory_order_acq_rel);
    }
    /* Drops the memo hold (a TP the table refused, or one just erased). */
    void drop_hold(void)
    {
        if(decRef())
            delete this;
    }

    unsigned su(void) const { return su_; }
    size_t   bytes(void) const { return bytes_; }
    /* The generation of the last accepted wake (0 before any). */
    uint64_t generation(void) const { return gen_.load(std::memory_order_acquire); }
    uint32_t state(void) const { return state_.load(std::memory_order_acquire); }
    /* Diagnostic: the current reference count. */
    unsigned ref_count(void) const { return Atomics::load(const_cast<unsigned int &>(ref_)); }

    virtual Codelet & entry(void) = 0;

    /* Releases one wake pin's reference with the rule of Codelet.cpp
     * (releaseDirectedReference): delete on the last reference only for a
     * TP with a parent. */
    static void unpin(ResidentTP * tp)
    {
        if(!tp)
            return;
        const bool deleteTP = tp->checkParent();
        if(tp->decRef() && deleteTP)
            delete tp;
    }

private:
    friend class ResidentTable;
    std::atomic<uint32_t> state_;
    std::atomic<uint64_t> gen_;
    unsigned              su_;
    size_t                bytes_;
};

/* One payload slot per TP, tagged by generation. post() is called by the one
 * producer whose wake was accepted (rule 10); take() by the woken body. The
 * release store / acquire load order the payload; the entry's enqueue
 * (a locked scheduler queue) also orders post() before the fire. */
template<class Payload>
class WakeMailbox
{
public:
    WakeMailbox(void) : gen_(0), dups_(0), payload_() {}

    /* false (counted) for generation 0 or a generation not newer than the
     * last post: a duplicate or stale wake. */
    bool post(uint64_t gen, const Payload & p)
    {
        if(gen == 0 || gen <= gen_.load(std::memory_order_acquire))
        {
            dups_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        payload_ = p;
        gen_.store(gen, std::memory_order_release);
        return true;
    }

    /* Copies the payload posted for exactly `gen`. */
    bool take(uint64_t gen, Payload & out) const
    {
        if(gen == 0 || gen_.load(std::memory_order_acquire) != gen)
            return false;
        out = payload_;
        return true;
    }

    uint64_t generation(void) const { return gen_.load(std::memory_order_acquire); }
    uint64_t duplicates(void) const { return dups_.load(std::memory_order_relaxed); }

private:
    std::atomic<uint64_t> gen_;
    std::atomic<uint64_t> dups_;
    Payload               payload_;
};

/* Eviction policy hook. The table calls every method with its mutex held:
 * a policy must not call back into the table. victim() picks one key for
 * which evictable(key) is true and must not change its own state; the table
 * calls erased() for each key it actually removes. */
struct EvictionPolicyIface
{
    virtual ~EvictionPolicyIface() {}
    virtual void touched(const ResidentKey & key, uint64_t tick) = 0;
    virtual bool victim(const std::function<bool(const ResidentKey &)> & evictable, ResidentKey & out) = 0;
    virtual void erased(const ResidentKey & key) = 0;
};

/* One per SU. Every operation holds mu_. Holds are dropped outside the lock.
 * `tick` passed to EvictionPolicyIface::touched is a per-table counter that
 * increases on every insert and every accepted wake. */
class ResidentTable
{
public:
    ResidentTable(uint64_t budget_bytes, EvictionPolicyIface * policy)
        : budget_(budget_bytes), policy_(policy), used_(0), tick_(0), evictions_(0), bytesFreed_(0),
          wakes_(0), misses_(0), busy_(0), dups_(0), unpinErrors_(0), noRoom_(0) {}

    /* Drops every remaining hold (teardown()). Host only, after the run. */
    ~ResidentTable() { teardown(); }

    enum WakeResult { WOKEN_OK, MISSING, EVICTING, BUSY_PROTOCOL, DUPLICATE };

    /* find; generation check; CAS IDLE -> WOKEN; pins++; incRef. On WOKEN_OK
     * *out is the TP (pinned: call unpin(key, tp) once). Otherwise *out is
     * left unchanged and nothing changes but a counter. */
    WakeResult lookup_and_wake(const ResidentKey & key, uint64_t gen, ResidentTP ** out)
    {
        std::lock_guard<std::mutex> lock(mu_);
        Map::iterator it = map_.find(key);
        if(it == map_.end())
        {
            ++misses_;
            return MISSING;
        }
        ResidentTP * tp = it->second.tp;
        if(gen == 0 || gen <= tp->gen_.load(std::memory_order_relaxed))
        {
            ++dups_;
            return DUPLICATE;
        }
        uint32_t s = ResidentTP::IDLE;
        if(!tp->state_.compare_exchange_strong(s, (uint32_t)ResidentTP::WOKEN, std::memory_order_acq_rel))
        {
            if(s == ResidentTP::EVICTING)
                return EVICTING;
            ++busy_;
            return BUSY_PROTOCOL;
        }
        tp->gen_.store(gen, std::memory_order_release);
        ++it->second.pins;
        tp->incRef();
        ++wakes_;
        if(policy_)
            policy_->touched(key, ++tick_);
        *out = tp;
        return WOKEN_OK;
    }

    /* pins--; then ResidentTP::unpin(tp) outside the lock. A key/TP that does
     * not match a pinned entry is counted (unpin_errors()) and the reference
     * is still released. */
    void unpin(const ResidentKey & key, ResidentTP * tp)
    {
        {
            std::lock_guard<std::mutex> lock(mu_);
            Map::iterator it = map_.find(key);
            if(it != map_.end() && it->second.tp == tp && it->second.pins > 0)
                --it->second.pins;
            else
                ++unpinErrors_;
        }
        ResidentTP::unpin(tp);
    }

    enum InsertResult { INSERTED, EXISTS, NO_ROOM };

    /* On INSERTED the table owns the TP's memo hold. On EXISTS / NO_ROOM the
     * caller drop_hold()s its new TP. NO_ROOM does not evict: call make_room
     * and retry. */
    InsertResult insert(const ResidentKey & key, ResidentTP * tp)
    {
        std::lock_guard<std::mutex> lock(mu_);
        if(map_.find(key) != map_.end())
            return EXISTS;
        if(tp->bytes() > budget_ || used_ > budget_ - tp->bytes())
        {
            ++noRoom_;
            return NO_ROOM;
        }
        Entry e; e.tp = tp; e.pins = 0;
        map_.insert(std::make_pair(key, e));
        used_ += tp->bytes();
        if(policy_)
            policy_->touched(key, ++tick_);
        return INSERTED;
    }

    enum EvictResult { EVICTED, BUSY, NOT_FOUND };

    /* BUSY if pins > 0 or state != IDLE; else CAS IDLE -> EVICTING, erase,
     * and drop_hold outside the lock. The TP is deleted there, or by the
     * scheduler's decRef after a fire that is still finishing. */
    EvictResult evict(const ResidentKey & key)
    {
        ResidentTP * tp = 0;
        {
            std::lock_guard<std::mutex> lock(mu_);
            Map::iterator it = map_.find(key);
            if(it == map_.end())
                return NOT_FOUND;
            if(!evictable_locked(it->second))
                return BUSY;
            tp = erase_locked(it);
        }
        tp->drop_hold();
        return EVICTED;
    }

    /* Evicts policy victims until `bytes` more fit. Returns false, evicting
     * nothing, when bytes > budget, when there is no policy, or when the
     * evictable entries cannot free enough (e.g. all BUSY). The victims are
     * chosen first and evicted only when they suffice; the loop is bounded
     * by the table size. Evicted keys are appended to *evicted (may be NULL). */
    bool make_room(uint64_t bytes, std::vector<ResidentKey> * evicted)
    {
        std::vector<ResidentTP *> drop;
        {
            std::lock_guard<std::mutex> lock(mu_);
            if(bytes > budget_)
                return false;
            if(used_ <= budget_ - bytes)
                return true;
            if(!policy_)
                return false;
            const uint64_t need = used_ - (budget_ - bytes);
            std::vector<ResidentKey> chosen;
            uint64_t freed = 0;
            std::function<bool(const ResidentKey &)> ok = [this, &chosen](const ResidentKey & k) -> bool {
                Map::const_iterator it = map_.find(k);
                if(it == map_.end() || !evictable_locked(it->second))
                    return false;
                for(size_t i = 0; i < chosen.size(); ++i)
                    if(chosen[i] == k)
                        return false;
                return true;
            };
            for(size_t n = 0, cap = map_.size(); freed < need && n < cap; ++n)
            {
                ResidentKey k;
                if(!policy_->victim(ok, k) || !ok(k))
                    return false;
                chosen.push_back(k);
                freed += map_.find(k)->second.tp->bytes();
            }
            if(freed < need)
                return false;
            for(size_t i = 0; i < chosen.size(); ++i)
            {
                drop.push_back(erase_locked(map_.find(chosen[i])));
                if(evicted)
                    evicted->push_back(chosen[i]);
            }
        }
        for(size_t i = 0; i < drop.size(); ++i)
            drop[i]->drop_hold();
        return true;
    }

    /* Host only, after the run (after BodyDrain::wait_zero()): erases every
     * entry and drops its memo hold, whatever its state. Not counted as
     * evictions. Returns the number of holds dropped. */
    size_t teardown(void)
    {
        std::vector<ResidentTP *> drop;
        {
            std::lock_guard<std::mutex> lock(mu_);
            for(Map::iterator it = map_.begin(); it != map_.end(); ++it)
            {
                drop.push_back(it->second.tp);
                used_ -= it->second.tp->bytes();
                if(policy_)
                    policy_->erased(it->first);
            }
            map_.clear();
        }
        for(size_t i = 0; i < drop.size(); ++i)
            drop[i]->drop_hold();
        return drop.size();
    }

    uint64_t used(void) const        { std::lock_guard<std::mutex> l(mu_); return used_; }
    uint64_t budget(void) const      { return budget_; }
    uint64_t evictions(void) const   { std::lock_guard<std::mutex> l(mu_); return evictions_; }
    uint64_t bytes_freed(void) const { std::lock_guard<std::mutex> l(mu_); return bytesFreed_; }
    size_t   size(void) const        { std::lock_guard<std::mutex> l(mu_); return map_.size(); }
    uint64_t wakes(void) const       { std::lock_guard<std::mutex> l(mu_); return wakes_; }
    uint64_t misses(void) const      { std::lock_guard<std::mutex> l(mu_); return misses_; }
    uint64_t busy_protocol(void) const { std::lock_guard<std::mutex> l(mu_); return busy_; }
    uint64_t duplicates(void) const  { std::lock_guard<std::mutex> l(mu_); return dups_; }
    uint64_t unpin_errors(void) const { std::lock_guard<std::mutex> l(mu_); return unpinErrors_; }
    uint64_t no_room(void) const     { std::lock_guard<std::mutex> l(mu_); return noRoom_; }

private:
    struct Entry { ResidentTP * tp; uint32_t pins; };
    typedef std::unordered_map<ResidentKey, Entry, ResidentKeyHash> Map;

    ResidentTable(const ResidentTable &);
    ResidentTable & operator=(const ResidentTable &);

    static bool evictable_locked(const Entry & e)
    {
        return e.pins == 0 && e.tp->state_.load(std::memory_order_acquire) == ResidentTP::IDLE;
    }

    /* CAS IDLE -> EVICTING (it cannot fail under mu_ for an evictable entry:
     * only lookup_and_wake leaves IDLE, and it holds mu_), erase, account. */
    ResidentTP * erase_locked(Map::iterator it)
    {
        ResidentTP * tp = it->second.tp;
        uint32_t s = ResidentTP::IDLE;
        (void)tp->state_.compare_exchange_strong(s, (uint32_t)ResidentTP::EVICTING, std::memory_order_acq_rel);
        used_ -= tp->bytes();
        ++evictions_;
        bytesFreed_ += tp->bytes();
        if(policy_)
            policy_->erased(it->first);
        map_.erase(it);
        return tp;
    }

    mutable std::mutex     mu_;
    Map                    map_;
    const uint64_t         budget_;
    EvictionPolicyIface *  policy_;
    uint64_t               used_, tick_, evictions_, bytesFreed_;
    uint64_t               wakes_, misses_, busy_, dups_, unpinErrors_, noRoom_;
};

} // namespace darts

#endif
