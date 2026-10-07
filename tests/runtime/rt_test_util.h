/*
 * rt_test_util.h -- shared harness of the runtime tests in tests/runtime.
 * Header-only, C++11.
 *
 *   check(cond, what)      records a failed sub-check (never assert(): DARTS
 *                          builds with -O3 -DNDEBUG) and returns cond
 *   Row / Table            one result row per test; RowScope counts the
 *                          check() failures that happen while it is open
 *   print_rows(...)        a markdown result table followed by a final
 *                          "<NAME> PASS|FAIL" line
 *   start_watchdog(s)      _exit(3) when progress() is not bumped for s seconds
 *   cluster_of_cpu_table() os cpu -> LLC cluster, AbstractMachine(true)
 *   require_cpus(affin)    exit(4) unless sched_getaffinity(0) holds every cpu
 *                          of the affinity's TPS and MC masks; call it BEFORE
 *                          `new Runtime`
 *
 * Exit codes: 0 pass, 1 check failed, 2 usage, 3 watchdog, 4 skipped (the
 * host or the allocation does not have the shape the test needs; ctest
 * reports it as skipped).
 */
#ifndef DARTS_RT_TEST_UTIL_H
#define DARTS_RT_TEST_UTIL_H

/* std headers that use thread_local go before darts.h:
 * include/threadlocal/threadlocal.h #defines thread_local to __thread. */
#include <atomic>
#include <thread>
#include <mutex>

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <string>
#include <vector>
#include <sched.h>
#include <pthread.h>
#include <unistd.h>

#include "darts.h"
#include "AbstractMachine.h"

namespace rt_test
{

enum ExitCode { EXIT_PASS = 0, EXIT_CHECK = 1, EXIT_USAGE = 2, EXIT_WATCHDOG = 3,
                EXIT_REFUSE = 4 };

/* ------------------------------------------------------------------ */
/* check(): process-wide failure counter. */

inline std::atomic<long long> & check_failures()
{
    static std::atomic<long long> n(0);
    return n;
}

inline bool check(bool cond, const char * what)
{
    if(!cond)
    {
        check_failures().fetch_add(1);
        std::printf("  CHECK FAILED: %s\n", what);
        std::fflush(stdout);
    }
    return cond;
}

inline bool check(bool cond, const std::string & what) { return check(cond, what.c_str()); }

/* ------------------------------------------------------------------ */
/* Result rows. */

struct Row
{
    std::string test, detail;
    long long   rounds, offTarget, badFires;
    bool        pass;
};

struct Table
{
    std::vector<Row> rows;

    void add(const std::string & test, const std::string & detail, long long rounds,
             long long offTarget, long long badFires, bool pass)
    {
        Row r; r.test = test; r.detail = detail; r.rounds = rounds;
        r.offTarget = offTarget; r.badFires = badFires; r.pass = pass;
        rows.push_back(r);
        std::printf("  %-4s %-52s rounds=%-6lld off_target=%-5lld bad_fires=%-4lld %s\n",
                    test.c_str(), detail.c_str(), rounds, offTarget, badFires,
                    pass ? "PASS" : "FAIL");
        std::fflush(stdout);
    }

    long long failed() const
    {
        long long n = 0;
        for(size_t i = 0; i < rows.size(); ++i) if(!rows[i].pass) ++n;
        return n;
    }
};

inline Table & rows()
{
    static Table t;
    return t;
}

inline void row(const std::string & test, const std::string & detail, long long rounds,
                long long offTarget, long long badFires, bool pass)
{
    rows().add(test, detail, rounds, offTarget, badFires, pass);
}

/* A row whose verdict includes every check() failure while it is open:
 * badFires = failed sub-checks, pass = (no failed sub-check) && ok. */
class RowScope
{
public:
    RowScope(Table & t, const std::string & test, const std::string & detail)
        : t_(t), test_(test), detail_(detail), start_(check_failures().load()),
          rounds_(0), offTarget_(0), done_(false) {}
    RowScope(const std::string & test, const std::string & detail)
        : t_(rows()), test_(test), detail_(detail), start_(check_failures().load()),
          rounds_(0), offTarget_(0), done_(false) {}
    ~RowScope() { finish(true); }

    void setRounds(long long r)    { rounds_ = r; }
    void setOffTarget(long long o) { offTarget_ = o; }
    void setDetail(const std::string & d) { detail_ = d; }

    bool finish(bool ok)
    {
        if(done_) return false;
        done_ = true;
        long long bad = check_failures().load() - start_;
        bool pass = ok && bad == 0;
        t_.add(test_, detail_, rounds_, offTarget_, bad, pass);
        return pass;
    }

private:
    Table &     t_;
    std::string test_, detail_;
    long long   start_, rounds_, offTarget_;
    bool        done_;
};

/* Prints the markdown table and "<name> PASS|FAIL"; returns the number of
 * failed rows (0 = pass). An empty table is a FAIL. */
inline long long print_rows(const Table & t, const char * name, FILE * out = stdout)
{
    long long fails = t.failed();
    bool pass = fails == 0 && !t.rows.empty();
    std::fprintf(out, "\n| test | detail | rounds | off-target | bad fires | result |\n|---|---|---|---|---|---|\n");
    for(size_t i = 0; i < t.rows.size(); ++i)
    {
        const Row & r = t.rows[i];
        std::fprintf(out, "| %s | %s | %lld | %lld | %lld | %s |\n", r.test.c_str(), r.detail.c_str(),
                     r.rounds, r.offTarget, r.badFires, r.pass ? "PASS" : "FAIL");
    }
    std::fprintf(out, "\n%zu rows, %lld failed\n%s %s\n", t.rows.size(), fails, name,
                 pass ? "PASS" : "FAIL");
    std::fflush(out);
    return pass ? 0 : (fails ? fails : 1);
}

inline long long print_rows(const char * name) { return print_rows(rows(), name); }

/* ------------------------------------------------------------------ */
/* Watchdog: a test that stops making progress fails with _exit(3) instead
 * of hanging the job. Start it before the Runtime so the thread inherits
 * the unpinned mask. */

struct WatchdogState
{
    std::atomic<long long> progress;
    std::atomic<int>       stop;
    int                    seconds;
    const char *           phase;
    pthread_t              thread;
    bool                   running;
    WatchdogState() : progress(0), stop(0), seconds(60), phase("start"), thread(), running(false) {}
};

inline WatchdogState & watchdog_state()
{
    static WatchdogState s;
    return s;
}

inline void progress(const char * phase = 0)
{
    WatchdogState & s = watchdog_state();
    if(phase) s.phase = phase;
    s.progress.fetch_add(1);
}

inline void * watchdog_main(void *)
{
    WatchdogState & s = watchdog_state();
    long long last = s.progress.load();
    long long idleMs = 0;
    while(!s.stop.load())
    {
        usleep(100 * 1000);
        long long now = s.progress.load();
        if(now != last) { last = now; idleMs = 0; continue; }
        idleMs += 100;
        if(idleMs >= (long long)s.seconds * 1000)
        {
            std::printf("FAIL: watchdog -- no progress for %d s in phase %s\n",
                        s.seconds, s.phase ? s.phase : "?");
            std::fflush(stdout);
            _exit(EXIT_WATCHDOG);
        }
    }
    return 0;
}

inline bool start_watchdog(int seconds)
{
    WatchdogState & s = watchdog_state();
    if(s.running) return true;
    s.seconds = seconds > 0 ? seconds : 1;
    s.stop.store(0);
    if(pthread_create(&s.thread, 0, watchdog_main, 0) != 0) return false;
    s.running = true;
    return true;
}

inline void stop_watchdog()
{
    WatchdogState & s = watchdog_state();
    if(!s.running) return;
    s.stop.store(1);
    pthread_join(s.thread, 0);
    s.running = false;
}

/* ------------------------------------------------------------------ */
/* Topology. */

/* os cpu -> LLC cluster index (-1 = not in the map), from the same
 * AbstractMachine(useLLC = true) the Runtime uses. */
inline std::vector<int> cluster_of_cpu_table()
{
    darts::hwloc::AbstractMachine am(true);
    darts::hwloc::Cluster * map = am.getClusterMap();
    std::vector<int> t;
    for(size_t c = 0; c < am.getNbClusters(); ++c)
    {
        darts::hwloc::Unit * units = map[c].getUnits();
        for(uint64_t u = 0; u < map[c].getNbUnits(); ++u)
        {
            int id = (int)units[u].getId();
            if(id < 0) continue;
            if(id >= (int)t.size()) t.resize(id + 1, -1);
            t[id] = (int)c;
        }
    }
    return t;
}

/* The os cpus a generated ThreadAffinity will pin to, mapped exactly as
 * Runtime.cpp does: clusterMap[clusterID].getUnits()[unitID].getId().
 * A mask entry outside the map yields -1. */
inline std::vector<int> affinity_cpus(const darts::ThreadAffinity & affin)
{
    darts::ThreadAffinity & a = const_cast<darts::ThreadAffinity &>(affin);
    darts::hwloc::AbstractMachine am(a.getLLC());
    darts::hwloc::Cluster * map = am.getClusterMap();
    std::vector<int> cpus;
    darts::AffinityMask * masks[2] = { a.getTPMask(), a.getMCMask() };
    for(int m = 0; m < 2; ++m)
        for(unsigned i = 0; i < masks[m]->size; ++i)
        {
            unsigned c = masks[m]->clusterID[i], u = masks[m]->unitID[i];
            if(c >= am.getNbClusters() || u >= map[c].getNbUnits()) { cpus.push_back(-1); continue; }
            cpus.push_back((int)map[c].getUnits()[u].getId());
        }
    return cpus;
}

/* First cpu of the list that sched_getaffinity(0) does not allow, or -2 when
 * every cpu is allowed. -1 entries (unmappable) are reported as refused. */
inline int first_disallowed_cpu(const std::vector<int> & cpus)
{
    cpu_set_t set;
    CPU_ZERO(&set);
    if(sched_getaffinity(0, sizeof(set), &set) != 0) return cpus.empty() ? -2 : cpus[0];
    for(size_t i = 0; i < cpus.size(); ++i)
        if(cpus[i] < 0 || cpus[i] >= CPU_SETSIZE || !CPU_ISSET(cpus[i], &set)) return cpus[i];
    return -2;
}

inline void require_cpu_list(const std::vector<int> & cpus)
{
    int bad = first_disallowed_cpu(cpus);
    if(bad != -2)
    {
        std::printf("SKIP: cpu %d is not in this process's affinity mask\n", bad);
        std::fflush(stdout);
        std::exit(EXIT_REFUSE);
    }
}

/* Call before `new Runtime(&affin)` (and after generateMask()). */
inline void require_cpus(const darts::ThreadAffinity & affin)
{
    require_cpu_list(affinity_cpus(affin));
}

/* ------------------------------------------------------------------ */
/* NUMA_PAIRED shape: two LLC clusters per NUMA node, clusters 2k and 2k+1
 * on node k. nsu = clusters, nnode = NUMA nodes, mcPerTp = units of the
 * smallest cluster - 1 (every unit of an SU busy). False (the caller skips
 * with EXIT_REFUSE) on any other host. */

struct PairedShape
{
    unsigned nsu, nnode, mcPerTp;
    PairedShape() : nsu(0), nnode(0), mcPerTp(0) {}
};

inline bool paired_shape(PairedShape & s)
{
    darts::hwloc::AbstractMachine am(true);
    s.nnode = (unsigned)am.getNbNumaNodes();
    s.nsu   = (unsigned)am.getNbClusters();
    if(s.nnode < 1 || s.nsu != 2 * s.nnode) return false;
    uint64_t minUnits = ~(uint64_t)0;
    for(unsigned c = 0; c < s.nsu; ++c)
    {
        if(am.numaNodeOfCluster(c) != c / 2) return false;
        if(am.getClusterMap()[c].getNbUnits() < minUnits) minUnits = am.getClusterMap()[c].getNbUnits();
    }
    if(minUnits < 2) return false;
    s.mcPerTp = (unsigned)(minUnits - 1);
    return true;
}

inline int skip_unless_paired(PairedShape & s, const char * name)
{
    if(paired_shape(s)) return 0;
    std::printf("%s: SKIP -- needs exactly two LLC clusters per NUMA node "
                "(clusters 2k, 2k+1 on node k) with at least two units each\n", name);
    return EXIT_REFUSE;
}

/* Sums of the per-SU pull counters, read from the host thread (which runs
 * TP scheduler 0 after the Runtime constructor). */
inline uint64_t total_node_pulls()
{
    darts::TPScheduler * me = darts::myThread.threadTPsched;
    uint64_t n = 0;
    for(unsigned i = 0; me && i < me->getNumTPSched(); ++i)
        n += static_cast<darts::TPScheduler *>(me->getRuntimeTPSched(i))->nodePulls();
    return n;
}

inline uint64_t total_sibling_pulls()
{
    darts::TPScheduler * me = darts::myThread.threadTPsched;
    uint64_t n = 0;
    for(unsigned i = 0; me && i < me->getNumTPSched(); ++i)
        n += static_cast<darts::TPScheduler *>(me->getRuntimeTPSched(i))->siblingPulls();
    return n;
}

} // namespace rt_test

#endif
