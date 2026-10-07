/*
 * darts-unit-numa-topology -- AbstractMachine::getNbNumaNodes() and
 * numaNodeOfCluster() against hwloc's own NUMA cpusets, for the socket and
 * the LLC cluster maps. No Runtime is started.
 *
 *   A  getNbNumaNodes() >= 1 and equals hwloc's NUMANODE count (1 if none)
 *   B  LLC clusters: every unit of cluster c is in the cpuset of NUMA node
 *      numaNodeOfCluster(c); socket clusters (which may span several nodes):
 *      that node holds at least one unit of c (when the topology has NUMA
 *      objects)
 *   C  every NUMA node that has cpus owns at least one LLC cluster
 *   D  an out-of-range cluster answers 0
 */
#include "rt_test_util.h"

#include <hwloc.h>

using namespace darts;
using namespace rt_test;

static void checkMap(bool llc, hwloc_topology_t topo, int hwNodes)
{
    hwloc::AbstractMachine am(llc);
    const size_t nodes = am.getNbNumaNodes();
    char d[160];
    {
        std::snprintf(d, sizeof(d), "%s: getNbNumaNodes() == %zu", llc ? "LLC" : "socket", nodes);
        RowScope r(llc ? "A.llc" : "A.sock", d);
        check(nodes >= 1, "at least one node");
        check(nodes == (size_t)(hwNodes > 0 ? hwNodes : 1), "matches hwloc's NUMANODE count");
        r.setRounds(1);
    }
    long long bad = 0, units = 0;
    std::vector<int> owned(nodes, 0);
    for(size_t c = 0; c < am.getNbClusters(); ++c)
    {
        const uint64_t n = am.numaNodeOfCluster(c);
        if(n >= nodes) { ++bad; continue; }
        owned[n]++;
        if(hwNodes <= 0) continue;
        hwloc_obj_t node = hwloc_get_obj_by_type(topo, HWLOC_OBJ_NUMANODE, (unsigned)n);
        hwloc::Cluster & cl = am.getClusterMap()[c];
        uint64_t in = 0;
        for(uint64_t u = 0; u < cl.getNbUnits(); ++u, ++units)
            if(node && node->cpuset && hwloc_bitmap_isset(node->cpuset, (unsigned)cl.getUnits()[u].getId()))
                ++in;
        if(llc ? in != cl.getNbUnits() : in == 0)
            ++bad;
    }
    {
        std::snprintf(d, sizeof(d), "%s: %zu clusters (%lld units) on their reported node",
                      llc ? "LLC" : "socket", am.getNbClusters(), units);
        RowScope r(llc ? "B.llc" : "B.sock", d);
        check(bad == 0, llc ? "every unit is on its cluster's NUMA node"
                            : "the reported node holds part of the cluster");
        r.setRounds(am.getNbClusters());
        r.setOffTarget(bad);
    }
    if(llc && hwNodes > 0)
    {
        RowScope r("C", "every NUMA node with cpus owns an LLC cluster");
        for(size_t n = 0; n < nodes; ++n)
        {
            hwloc_obj_t node = hwloc_get_obj_by_type(topo, HWLOC_OBJ_NUMANODE, (unsigned)n);
            if(node && node->cpuset && !hwloc_bitmap_iszero(node->cpuset))
                check(owned[n] > 0, "node owns a cluster");
        }
        r.setRounds(nodes);
    }
    if(llc)
    {
        RowScope r("D", "out-of-range cluster answers 0");
        check(am.numaNodeOfCluster(am.getNbClusters()) == 0, "numaNodeOfCluster(nClusters) == 0");
        r.setRounds(1);
    }
}

int main(void)
{
    hwloc_topology_t topo;
    hwloc_topology_init(&topo);
    hwloc_topology_load(topo);
    const int hwNodes = hwloc_get_nbobjs_by_type(topo, HWLOC_OBJ_NUMANODE);
    {
        hwloc::AbstractMachine am(true);
        std::printf("numa-topology: numa_nodes=%d llc_clusters=%zu units=%zu cluster->node:",
                    hwNodes, am.getNbClusters(), am.getTotalNbUnits());
        for(size_t c = 0; c < am.getNbClusters(); ++c)
            std::printf(" %llu", (unsigned long long)am.numaNodeOfCluster(c));
        std::printf("\n");
    }
    checkMap(false, topo, hwNodes);
    checkMap(true, topo, hwNodes);
    hwloc_topology_destroy(topo);
    return print_rows("DARTS-UNIT-NUMA-TOPOLOGY") ? EXIT_CHECK : EXIT_PASS;
}
