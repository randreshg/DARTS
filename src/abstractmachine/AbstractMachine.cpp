/* 
 * Copyright (c) 2011-2014, University of Delaware
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <AbstractMachine.h>

typedef hwloc_obj_t obj_t;

void 
darts :: hwloc :: AbstractMachine :: discoverTopologyWithLLC(void)
{
    /* The cluster level is the first cache object below the first package
     * (the last-level cache). Every cache object at that depth that holds
     * at least one usable PU becomes one cluster, numbered in order, so the
     * map stays dense and in bounds when the process may use only part of
     * the machine (a cgroup or batch allocation) and the PU count per
     * cache differs. A topology without a package or a cache object falls
     * back to the per-package map of discoverTopology(). */
    hwloc_obj_t o = hwloc_get_obj_by_type(_topology,HWLOC_OBJ_SOCKET,0);
    hwloc_obj_t obj;
    for (obj = o ? o->first_child : 0;
            obj && !hwloc_obj_type_is_cache(obj->type);
            obj = obj->first_child)
        ;
    if (!obj) {
        discoverTopology();
        return;
    }

    const int depth = obj->depth;
    const unsigned nbCaches = hwloc_get_nbobjs_by_depth(_topology, depth);
    _clusterMap = new Cluster[nbCaches ? nbCaches : 1];
    _nbClusters = 0;
    for (unsigned k = 0; k < nbCaches; ++k) {
        o = hwloc_get_obj_by_depth(_topology, depth, k);
        int nUnits = o ? hwloc_get_nbobjs_inside_cpuset_by_type(_topology,o->cpuset,HWLOC_OBJ_PU) : 0;
        if (nUnits <= 0)
            continue;
        Unit *units  = new Unit[nUnits];
        for (int i = 0; i < nUnits; ++i) {
            hwloc_obj_t t = hwloc_get_obj_inside_cpuset_by_type(_topology,o->cpuset,HWLOC_OBJ_PU,i);
            Unit hwu(_nbClusters,t->logical_index,t->os_index);
            units[i] = hwu; // simple shallow copy
        }
        Cluster cluster(_nbClusters,_nbClusters,nUnits,units);
        _clusterMap[_nbClusters++] = cluster; // simple shallow copy
    }
    if (_nbClusters == 0) {
        delete [] _clusterMap;
        _clusterMap = 0;
        discoverTopology();
    }
}

void 
darts :: hwloc :: AbstractMachine :: discoverTopology(void)
{
    _nbClusters   = hwloc_get_nbobjs_by_type(_topology,HWLOC_OBJ_SOCKET);
    _clusterMap   = new Cluster[_nbClusters];
    hwloc_obj_t o = hwloc_get_obj_by_type(_topology,HWLOC_OBJ_SOCKET,0);
    // TODO Refactor this code and the previous function's code into a single one
    for (; o; o = o->next_cousin)  {
        int           nUnits = hwloc_get_nbobjs_inside_cpuset_by_type(_topology,o->cpuset,HWLOC_OBJ_PU);
        Unit *units  = new Unit[nUnits];
        for (int i = 0; i < nUnits; ++i) {
            hwloc_obj_t t = hwloc_get_obj_inside_cpuset_by_type(_topology,o->cpuset,HWLOC_OBJ_PU,i);
            Unit hwu(o->logical_index,t->logical_index,t->os_index);
            units[i] = hwu; // simple shallow copy
        }
        Cluster cluster(o->logical_index,o->logical_index,nUnits,units);
        _clusterMap[o->logical_index] = cluster; // simple shallow copy
    }
}


