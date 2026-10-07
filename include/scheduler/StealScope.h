/*
 * StealScope -- how far a TP scheduler of a NUMA_PAIRED Runtime looks for
 * work that is not its own.
 *
 *   STEAL_LEGACY            the existing random-peer closure steal; the
 *                           default, and the only scope a runtime without a
 *                           node group ever has
 *   STEAL_NONE              no closure steal and no sibling codelet pull
 *   STEAL_SIBLING           closures: the oldest stealable closure of the
 *                           sibling SU (same NUMA node); codelets: the
 *                           sibling SU's shared pool
 *   STEAL_SIBLING_THEN_ANY  as SIBLING, then one random victim among the
 *                           other SUs' stealable closures
 *
 * No scope ever takes another SU's own codelet queue or placed closures, so
 * codelets placed with setPlacedCluster() and sticky closures still run
 * where they were placed.
 */
#ifndef DARTS_STEAL_SCOPE_H
#define DARTS_STEAL_SCOPE_H

namespace darts
{
    enum StealScope { STEAL_LEGACY = 0, STEAL_NONE = 1, STEAL_SIBLING = 2, STEAL_SIBLING_THEN_ANY = 3 };
}

#endif /* DARTS_STEAL_SCOPE_H */
