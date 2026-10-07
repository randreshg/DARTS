/*
 * darts-unit-resident-table -- ResidentTable, ResidentTP holds and the wake
 * mailbox on never-scheduled TPs (no Runtime).
 *
 *   A  construct + insert: ref_ == 1 (the memo hold is the base reference,
 *      no extra incRef), state IDLE, used() == bytes
 *   B  lookup_and_wake: MISSING; WOKEN_OK pins and takes one reference;
 *      a second wake while WOKEN or RUNNING is BUSY_PROTOCOL; a wake with a
 *      generation <= the last accepted one is DUPLICATE (no pin, no ref);
 *      mark_running/mark_idle transitions; unpin returns ref_ to 1
 *   C  evict: NOT_FOUND; while pinned -> BUSY; unpinned but not IDLE ->
 *      BUSY; IDLE -> EVICTED with exactly one destructor, used() back,
 *      evictions/bytes_freed counted
 *   D  EXISTS and NO_ROOM: the table keeps its own TP; the caller's
 *      drop_hold() runs exactly one destructor
 *   E  LRU order (test policy): the least recently touched IDLE entry goes
 *   F  LFU order (test policy): the least frequently touched IDLE entry goes
 *   G  make_room refuses bytes > budget, an all-BUSY table, a table with no
 *      policy, and evicts nothing when it refuses
 *   H  make_room frees exactly enough (two victims for a two-TP request) and
 *      reports the evicted keys
 *   I  teardown drops every hold: one destructor per TP, used() == 0
 *   J  WakeMailbox: post/take by generation, duplicate and stale posts
 *      refused and counted
 *   K  ResidentKey equality and hash spread (48 x 128 x 16 keys)
 *
 * Exit 0 only when every row PASSes.
 */
/* std headers (and ResidentRuntime.h, which includes its own first) go
 * before darts.h: threadlocal.h #defines thread_local. */
#include <functional>
#include <map>
#include <unordered_set>
#include "ResidentRuntime.h"
#include "rt_test_util.h"

using namespace darts;
using namespace rt_test;

static std::atomic<long long> g_dtors(0);

class NopCodelet : public Codelet
{
public:
    NopCodelet(ThreadedProcedure * tp) : Codelet(1, 1, tp, SHORTWAIT) { }
    virtual void fire(void) { }
};

class TestTP : public ResidentTP
{
public:
    NopCodelet entry_;
    TestTP(ResidentAnchor * a, unsigned su, size_t bytes)
        : ResidentTP(a, su, bytes), entry_(this) { entry_.setPlacedCluster(su); }
    virtual ~TestTP() { g_dtors.fetch_add(1); }
    virtual Codelet & entry() { return entry_; }
};

static ResidentKey key(uint16_t l, uint16_t e, uint16_t s = 0)
{
    ResidentKey k; k.group = l; k.item = e; k.shard = s; return k;
}

/* Reference LRU: victim = the evictable key touched longest ago. */
class TestLru : public EvictionPolicyIface
{
public:
    std::map<std::pair<uint32_t, uint16_t>, uint64_t> last_;
    static std::pair<uint32_t, uint16_t> id(const ResidentKey & k)
    { return std::make_pair(((uint32_t)k.group << 16) | k.item, k.shard); }
    virtual void touched(const ResidentKey & k, uint64_t tick) { last_[id(k)] = tick; }
    virtual bool victim(const std::function<bool(const ResidentKey &)> & evictable, ResidentKey & out)
    {
        bool found = false; uint64_t best = 0;
        for(std::map<std::pair<uint32_t, uint16_t>, uint64_t>::iterator it = last_.begin(); it != last_.end(); ++it)
        {
            ResidentKey k = key((uint16_t)(it->first.first >> 16), (uint16_t)(it->first.first & 0xffff), it->first.second);
            if(!evictable(k)) continue;
            if(!found || it->second < best) { found = true; best = it->second; out = k; }
        }
        return found;
    }
    virtual void erased(const ResidentKey & k) { last_.erase(id(k)); }
};

/* Reference LFU: victim = the evictable key touched fewest times. */
class TestLfu : public TestLru
{
public:
    std::map<std::pair<uint32_t, uint16_t>, uint64_t> count_;
    virtual void touched(const ResidentKey & k, uint64_t tick) { TestLru::touched(k, tick); count_[id(k)]++; }
    virtual bool victim(const std::function<bool(const ResidentKey &)> & evictable, ResidentKey & out)
    {
        bool found = false; uint64_t best = 0;
        for(std::map<std::pair<uint32_t, uint16_t>, uint64_t>::iterator it = count_.begin(); it != count_.end(); ++it)
        {
            ResidentKey k = key((uint16_t)(it->first.first >> 16), (uint16_t)(it->first.first & 0xffff), it->first.second);
            if(!evictable(k)) continue;
            if(!found || it->second < best) { found = true; best = it->second; out = k; }
        }
        return found;
    }
    virtual void erased(const ResidentKey & k) { TestLru::erased(k); count_.erase(id(k)); }
};

/* One full wake cycle on a never-scheduled TP: what the router and the
 * codelets would do, minus the scheduler. */
static bool cycle(ResidentTable & t, const ResidentKey & k, uint64_t gen)
{
    ResidentTP * tp = 0;
    if(t.lookup_and_wake(k, gen, &tp) != ResidentTable::WOKEN_OK || !tp) return false;
    bool ok = tp->mark_running();
    ok = tp->mark_idle() && ok;
    t.unpin(k, tp);
    return ok;
}

int main(void)
{
    start_watchdog(60);
    ResidentAnchor anchor;

    /* A */
    {
        RowScope r("A", "construct + insert: ref_ == 1, IDLE, used == bytes");
        long long d0 = g_dtors.load();
        ResidentTable t(1000, NULL);
        TestTP * tp = new TestTP(&anchor, 3, 100);
        check(tp->ref_count() == 1, "ref_ == 1 after construction (memo hold, no extra incRef)");
        check(tp->parentTP_ == &anchor, "parentTP_ == anchor");
        check(anchor.parentTP_ == NULL, "the anchor has no parent");
        check(tp->su() == 3 && tp->bytes() == 100, "su() and bytes()");
        check(tp->state() == ResidentTP::IDLE, "constructed IDLE");
        check(tp->generation() == 0, "generation() == 0 before any wake");
        check(t.insert(key(1, 2), tp) == ResidentTable::INSERTED, "insert -> INSERTED");
        check(tp->ref_count() == 1, "ref_ == 1 after insert");
        check(t.used() == 100 && t.size() == 1, "used() == 100, size() == 1");
        check(t.budget() == 1000, "budget() == 1000");
        check(t.teardown() == 1, "teardown drops one hold");
        check(g_dtors.load() == d0 + 1, "exactly one destructor");
        r.setRounds(1);
    }

    /* B */
    {
        RowScope r("B", "lookup_and_wake: MISSING/WOKEN_OK/BUSY_PROTOCOL/DUPLICATE");
        long long d0 = g_dtors.load();
        ResidentTable t(1000, NULL);
        TestTP * tp = new TestTP(&anchor, 0, 10);
        ResidentKey k = key(0, 5);
        ResidentTP * out = 0;
        check(t.lookup_and_wake(k, 1, &out) == ResidentTable::MISSING && out == 0, "empty table -> MISSING");
        check(t.insert(k, tp) == ResidentTable::INSERTED, "insert");
        check(t.lookup_and_wake(k, 1, &out) == ResidentTable::WOKEN_OK && out == tp, "IDLE -> WOKEN_OK");
        check(tp->state() == ResidentTP::WOKEN, "state WOKEN");
        check(tp->generation() == 1, "generation() == 1");
        check(tp->ref_count() == 2, "the wake pin holds one reference");
        ResidentTP * out2 = 0;
        check(t.lookup_and_wake(k, 2, &out2) == ResidentTable::BUSY_PROTOCOL && out2 == 0, "WOKEN -> BUSY_PROTOCOL");
        check(t.lookup_and_wake(k, 1, &out2) == ResidentTable::DUPLICATE, "same generation while WOKEN -> DUPLICATE");
        check(!tp->mark_idle(), "mark_idle from WOKEN refused");
        check(tp->mark_running(), "WOKEN -> RUNNING");
        check(!tp->mark_running(), "second mark_running refused");
        check(t.lookup_and_wake(k, 2, &out2) == ResidentTable::BUSY_PROTOCOL, "RUNNING -> BUSY_PROTOCOL");
        check(tp->mark_idle(), "RUNNING -> IDLE");
        t.unpin(k, tp);
        check(tp->ref_count() == 1, "unpin returns ref_ to 1");
        check(t.lookup_and_wake(k, 1, &out2) == ResidentTable::DUPLICATE && tp->state() == ResidentTP::IDLE,
              "replayed generation after idle -> DUPLICATE, state untouched");
        check(t.lookup_and_wake(k, 0, &out2) == ResidentTable::DUPLICATE, "generation 0 is never accepted");
        check(tp->ref_count() == 1, "DUPLICATE took no reference");
        check(cycle(t, k, 2), "generation 2 accepted after idle");
        check(t.busy_protocol() == 2 && t.duplicates() == 3, "busy_protocol() == 2, duplicates() == 3");
        check(t.wakes() == 2 && t.misses() == 1, "wakes() == 2, misses() == 1");
        check(t.unpin_errors() == 0, "no unpin errors");
        t.teardown();
        check(g_dtors.load() == d0 + 1, "exactly one destructor");
        r.setRounds(1);
    }

    /* C */
    {
        RowScope r("C", "evict: NOT_FOUND, pinned BUSY, not-IDLE BUSY, IDLE EVICTED");
        long long d0 = g_dtors.load();
        ResidentTable t(1000, NULL);
        TestTP * tp = new TestTP(&anchor, 0, 64);
        ResidentKey k = key(4, 4, 1);
        check(t.evict(k) == ResidentTable::NOT_FOUND, "evict on empty table -> NOT_FOUND");
        t.insert(k, tp);
        ResidentTP * out = 0;
        check(t.lookup_and_wake(k, 1, &out) == ResidentTable::WOKEN_OK, "wake");
        check(t.evict(k) == ResidentTable::BUSY, "pinned (WOKEN) -> BUSY");
        tp->mark_running();
        t.unpin(k, tp);
        check(t.evict(k) == ResidentTable::BUSY, "unpinned but RUNNING -> BUSY");
        check(g_dtors.load() == d0, "no destructor while busy");
        tp->mark_idle();
        check(t.evict(k) == ResidentTable::EVICTED, "IDLE -> EVICTED");
        check(g_dtors.load() == d0 + 1, "exactly one destructor on eviction");
        check(t.used() == 0 && t.size() == 0, "used() == 0, size() == 0");
        check(t.evictions() == 1 && t.bytes_freed() == 64, "evictions() == 1, bytes_freed() == 64");
        check(t.evict(k) == ResidentTable::NOT_FOUND, "second evict -> NOT_FOUND");
        check(t.lookup_and_wake(k, 2, &out) == ResidentTable::MISSING, "evicted key -> MISSING (create path)");
        r.setRounds(1);
    }

    /* D */
    {
        RowScope r("D", "EXISTS / NO_ROOM + drop_hold -> exactly one destructor each");
        long long d0 = g_dtors.load();
        ResidentTable t(150, NULL);
        TestTP * a = new TestTP(&anchor, 0, 100);
        TestTP * b = new TestTP(&anchor, 0, 100);
        TestTP * c = new TestTP(&anchor, 0, 100);
        check(t.insert(key(1, 1), a) == ResidentTable::INSERTED, "first insert");
        check(t.insert(key(1, 1), b) == ResidentTable::EXISTS, "same key -> EXISTS");
        b->drop_hold();
        check(g_dtors.load() == d0 + 1, "EXISTS + drop_hold -> one destructor");
        check(t.insert(key(1, 2), c) == ResidentTable::NO_ROOM, "over budget -> NO_ROOM");
        c->drop_hold();
        check(g_dtors.load() == d0 + 2, "NO_ROOM + drop_hold -> one destructor");
        check(t.used() == 100 && t.size() == 1, "table unchanged: used() == 100, size() == 1");
        check(t.no_room() == 1, "no_room() == 1");
        ResidentTP * out = 0;
        check(t.lookup_and_wake(key(1, 1), 1, &out) == ResidentTable::WOKEN_OK && out == a, "the table kept its own TP");
        a->mark_running(); a->mark_idle(); t.unpin(key(1, 1), a);
        t.teardown();
        check(g_dtors.load() == d0 + 3, "teardown -> the third destructor");
        r.setRounds(1);
    }

    /* E */
    {
        RowScope r("E", "LRU order");
        TestLru lru;
        ResidentTable t(300, &lru);
        for(uint16_t e = 0; e < 3; ++e) t.insert(key(0, e), new TestTP(&anchor, 0, 100));
        check(cycle(t, key(0, 0), 1), "touch e0");            /* order now e1, e2, e0 */
        std::vector<ResidentKey> ev;
        check(t.make_room(100, &ev), "make_room(100)");
        check(ev.size() == 1 && ev[0] == key(0, 1), "LRU evicts e1 first");
        check(cycle(t, key(0, 2), 1), "touch e2");            /* order now e0, e2 */
        ev.clear();
        check(t.make_room(200, &ev) && ev.size() == 1 && ev[0] == key(0, 0), "then e0");
        check(t.used() == 100, "used() == 100");
        t.teardown();
        r.setRounds(2);
    }

    /* F */
    {
        RowScope r("F", "LFU order");
        TestLfu lfu;
        ResidentTable t(300, &lfu);
        for(uint16_t e = 0; e < 3; ++e) t.insert(key(2, e), new TestTP(&anchor, 0, 100));
        check(cycle(t, key(2, 0), 1) && cycle(t, key(2, 0), 2) && cycle(t, key(2, 0), 3), "e0 x3");
        check(cycle(t, key(2, 1), 1), "e1 x1");
        check(cycle(t, key(2, 2), 1) && cycle(t, key(2, 2), 2), "e2 x2");
        std::vector<ResidentKey> ev;
        check(t.make_room(100, &ev) && ev.size() == 1 && ev[0] == key(2, 1), "LFU evicts e1 (1 touch)");
        ev.clear();
        check(t.make_room(200, &ev) && ev.size() == 1 && ev[0] == key(2, 2), "then e2 (2 touches)");
        t.teardown();
        r.setRounds(2);
    }

    /* G */
    {
        RowScope r("G", "make_room refuses infeasible / all-BUSY / no policy, evicts nothing");
        long long d0 = g_dtors.load();
        TestLru lru;
        ResidentTable t(200, &lru);
        TestTP * a = new TestTP(&anchor, 0, 100);
        TestTP * b = new TestTP(&anchor, 0, 100);
        t.insert(key(3, 0), a); t.insert(key(3, 1), b);
        std::vector<ResidentKey> ev;
        check(!t.make_room(201, &ev) && ev.empty(), "bytes > budget refused");
        ResidentTP * o1 = 0, * o2 = 0;
        t.lookup_and_wake(key(3, 0), 1, &o1);
        t.lookup_and_wake(key(3, 1), 1, &o2);
        check(!t.make_room(100, &ev) && ev.empty(), "all pinned (BUSY) refused");
        a->mark_running(); t.unpin(key(3, 0), a);
        check(!t.make_room(100, &ev) && ev.empty(), "unpinned but RUNNING still refused");
        a->mark_idle();
        check(!t.make_room(200, &ev) && ev.empty(), "one evictable of two needed: refused, nothing evicted");
        check(t.size() == 2 && t.used() == 200 && t.evictions() == 0, "nothing evicted");
        check(t.make_room(0, &ev) && ev.empty(), "make_room(0) on a full table is satisfied, evicts nothing");
        b->mark_running(); b->mark_idle(); t.unpin(key(3, 1), b);
        ResidentTable none(100, NULL);
        none.insert(key(3, 9), new TestTP(&anchor, 0, 100));
        check(!none.make_room(50, &ev) && ev.empty(), "no policy -> refused");
        check(none.make_room(0, &ev), "make_room(0) on a full budget is satisfied");
        none.teardown();
        t.teardown();
        check(g_dtors.load() == d0 + 3, "one destructor per TP at teardown");
        r.setRounds(1);
    }

    /* H */
    {
        RowScope r("H", "make_room frees exactly enough and reports keys");
        long long d0 = g_dtors.load();
        TestLru lru;
        ResidentTable t(400, &lru);
        for(uint16_t e = 0; e < 4; ++e) t.insert(key(5, e), new TestTP(&anchor, 0, 100));
        std::vector<ResidentKey> ev;
        check(t.make_room(150, &ev), "make_room(150) on a full 400/400 table");
        check(ev.size() == 2 && ev[0] == key(5, 0) && ev[1] == key(5, 1), "two LRU victims e0, e1");
        check(t.used() == 200 && t.evictions() == 2 && t.bytes_freed() == 200, "used 200, 2 evictions, 200 B freed");
        check(g_dtors.load() == d0 + 2, "two destructors");
        check(t.make_room(200, &ev) && ev.size() == 2, "200 more already fits: no eviction");
        t.teardown();
        r.setRounds(1);
    }

    /* I */
    {
        RowScope r("I", "teardown: one destructor per TP, used() == 0");
        long long d0 = g_dtors.load();
        ResidentTable * t = new ResidentTable(1 << 20, NULL);
        for(uint16_t e = 0; e < 128; ++e) t->insert(key(7, e), new TestTP(&anchor, e % 16, 1000));
        check(t->size() == 128 && t->used() == 128000, "128 entries, 128000 B");
        check(t->teardown() == 128, "teardown returns 128");
        check(t->used() == 0 && t->size() == 0, "empty after teardown");
        check(g_dtors.load() == d0 + 128, "128 destructors");
        for(uint16_t e = 0; e < 4; ++e) t->insert(key(8, e), new TestTP(&anchor, 0, 10));
        delete t;
        check(g_dtors.load() == d0 + 132, "the table destructor tears down the rest");
        r.setRounds(1);
    }

    /* J */
    {
        RowScope r("J", "WakeMailbox: generation-tagged post/take, duplicates refused");
        struct Payload { int tokens; float w; };
        WakeMailbox<Payload> m;
        Payload p = { 3, 0.5f }, q = { 0, 0.0f };
        check(!m.take(1, q), "nothing to take before a post");
        check(m.post(1, p), "post gen 1");
        check(!m.post(1, p), "duplicate post gen 1 refused");
        check(m.take(1, q) && q.tokens == 3 && q.w == 0.5f, "take gen 1");
        check(!m.take(2, q), "take gen 2 before its post refused");
        p.tokens = 9;
        check(m.post(3, p), "post gen 3");
        check(!m.post(2, p), "stale post gen 2 refused");
        check(!m.post(0, p), "gen 0 refused");
        check(m.take(3, q) && q.tokens == 9, "take gen 3");
        check(!m.take(1, q), "old generation no longer taken");
        check(m.duplicates() == 3, "duplicates() == 3");
        check(m.generation() == 3, "generation() == 3");
        r.setRounds(1);
    }

    /* K */
    {
        RowScope r("K", "ResidentKey equality and hash spread");
        std::unordered_set<size_t> hashes;
        ResidentKeyHash h;
        long long n = 0;
        for(uint16_t l = 0; l < 48; ++l)
            for(uint16_t e = 0; e < 128; ++e)
                for(uint16_t s = 0; s < 16; ++s) { hashes.insert(h(key(l, e, s))); ++n; }
        check((long long)hashes.size() == n, "48 x 128 x 16 keys hash distinctly");
        check(key(1, 2, 3) == key(1, 2, 3) && !(key(1, 2, 3) == key(1, 2, 4)), "operator==");
        r.setRounds(n);
    }

    stop_watchdog();
    return print_rows("DARTS-UNIT-RESIDENT-TABLE") ? EXIT_CHECK : EXIT_PASS;
}
