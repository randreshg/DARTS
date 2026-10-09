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


#pragma once
#include <cassert>
#include "TPScheduler.h"
#include "MicroScheduler.h"
#include "AbstractMachine.h"
#include "CodeletFinal.h"
#include "Thread.h"
#include "threadlocal.h"
#include "Affinity.h"

namespace darts
{
    class Runtime;
    
    struct tpRuntimeArgs
    {
        Runtime * rt;
        unsigned int threadId;
        unsigned int clusterId;
        unsigned int tpSched;
    };
    
    struct mcRuntimeArgs
    {
        Runtime * rt;
        unsigned int threadId;
        unsigned int clusterId;
        unsigned int unitId;
        unsigned int mcSched;
    };

    class Runtime
    {
    private:
        hwloc::AbstractMachine AbsMac;
        hwloc::Cluster * clusterMap;
        unsigned int numTPSched_;
        unsigned int numMCSched_;
        unsigned int numThreads_;
        TPScheduler ** TPSched_;
        MScheduler  ** MCSched_;
        Thread * localThreads_;
        tpRuntimeArgs * tpargs_;
        mcRuntimeArgs * mcargs_;
	ThreadAffinity * affinity_;
	bool papi_;
        volatile unsigned int tpcount_;
        volatile unsigned int mccount_;
        volatile unsigned int fullcount_;
        volatile bool spin_;
        /* NUMA_PAIRED only (0 / NULL / true on every other runtime):
         * constructionOk_ is false when a pin failed or DARTS_AFFINITY
         * overrode the layout; tpsNode_[i] = NUMA node of TP scheduler i;
         * one node codelet pool and one node closure pool per node; the
         * constructing thread (TP scheduler 0) is pinned and its previous
         * mask restored by the destructor. */
        bool constructionOk_;
        unsigned numNodes_;
        unsigned * tpsNode_;
        dartsPool<Codelet*> * nodeCodelets_;
        dartsPool<tpClosure*> * nodeTPs_;
        /* Idle-poll hint counters of the node pools: one per node, only
         * when the hint is on and a node group exists. */
        PollHintCount * nodeCodeletsN_;
        PollHintCount * nodeTPsN_;
        dartsCpuMask callerMaskSaved_;
        bool callerMaskValid_;
    public:
        static CodeletFinal finalSignal;
        Runtime(unsigned int maxCluster = -1, unsigned int maxWorker = -1);
        Runtime(ThreadAffinity * affinity);
        void run(tpClosure * tpToStart);
        /* run() with the root closure pushed to TP scheduler `tps` as a
         * placed closure (pushTPPlaced: never stolen) instead of TP
         * scheduler 0's stealable queue; TP scheduler 0 still runs its policy
         * on the calling thread until finalSignal. Returns false, without
         * running and without taking ownership of `root`, when root is NULL,
         * tps is out of range or the push is refused. */
        bool runPlaced(unsigned tps, tpClosure * root);
        ~Runtime(void);
        unsigned int getNumTPS(void) {return numTPSched_;}
        unsigned int getNumMCS(void) {return numMCSched_;}
        TPScheduler * newTPSched(unsigned int id, unsigned int type) { return TPSched_[id] = TPScheduler::create(type); }
        MScheduler  * newMCSched(unsigned int id, unsigned int type) { return MCSched_[id] = MScheduler::create(type); }
        void decTP(void)   {         Atomics::fetchSub(tpcount_,   1U); }
        void decMC(void)   {         Atomics::fetchSub(mccount_,   1U); }
        void decFull(void) { if( 1== Atomics::fetchSub(fullcount_, 1U) ) spin_=false; }
        ThreadAffinity * getAffinity(void) { return affinity_; }
        bool usePapi(void) { return papi_; }
        bool checkTP(void) { return (tpcount_); }
        bool checkMC(void) { return (mccount_); }
        void spin(void) const { 
            useconds_t usecs = 10, range = 10;
            while(spin_) {
                usleep(usecs);
                usecs += range;
            }
        }

        /* False when a NUMA_PAIRED runtime could not be placed as requested
         * (a pin failed, or DARTS_AFFINITY overrode the layout); the caller
         * should not trust placement-dependent results. Always true for the
         * other layouts. */
        bool constructionOk(void) const { return constructionOk_; }
        /* Node-group layer: numNodes() is 0 and numaPaired() false unless
         * the Runtime was built from an accepted NUMA_PAIRED mask;
         * nodeOfTPS(i) is ~0u without a node group or out of range. */
        unsigned numNodes(void) const { return numNodes_; }
        unsigned nodeOfTPS(unsigned i) const { return (tpsNode_ && i < numTPSched_) ? tpsNode_[i] : ~0u; }
        bool     numaPaired(void) const { return numNodes_ != 0; }

        void linkTPSched(void);
        void linkMCSched(void);       
    };
    
} // namespace darts
