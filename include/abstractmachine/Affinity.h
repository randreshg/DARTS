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


#ifndef AFFINITY_H
#define	AFFINITY_H
#include <string.h>
#include <iostream>
#include "StealScope.h"
#ifdef COUNT
#include <papi.h>
#endif
#define NUMEVENTS 5

namespace darts
{

    struct AffinityMask
    {
        unsigned int size;
        unsigned int * clusterID;
        unsigned int * unitID;
        AffinityMask(unsigned int num):
        size(num)
        {
            clusterID = new unsigned int[size];
            unitID = new unsigned int[size];
        }
        ~AffinityMask(void)
        {
            delete [] clusterID;
            delete [] unitID;
        }
    };
    
    /* NUMA_PAIRED (opt-in): one TP scheduler per last-level-cache cluster
     * (SU), its mcPerTp micro schedulers on units 1..mcPerTp of the same
     * cluster, on a host with exactly two such clusters per NUMA node, plus
     * a node-group layer: SU 2k and 2k+1 are the two SUs of NUMA node k and
     * share a node codelet pool and a node closure pool, and each SU has a
     * shared codelet pool its sibling may pull from (see StealScope).
     * generateMask() refuses any other machine shape. SPREAD and COMPACT
     * are unchanged. */
    enum AffinityMode { SPREAD = 0, COMPACT= 1, NUMA_PAIRED = 2 };
    
    class ThreadAffinity
    {
    private:
      
	bool papi;
#ifdef COUNT
	/* PAPI event selection; only read by the COUNT build. */
	bool L1;
	bool L2;
	bool FpIdle;
	bool Vector;
	bool Stall;
#endif
	
	bool llc;
        unsigned int mcPerTp;
        unsigned int numTPS;
        unsigned int numMCS;
        unsigned int TPpolicy;
        unsigned int MCpolicy;
	int * eventSet;
	long long * eventCounter;
        AffinityMode mode;
        AffinityMask TPMask;
        AffinityMask MCMask;
        /* place<> stickiness: -1 = not set, 0 = off, 1 = on. */
        int stickyPlacement_;
        /* NUMA_PAIRED only: 2 once generateMask() accepted the layout, else
         * 0; tpsNode_[i] = NUMA node of TP scheduler i (NULL until then). */
        unsigned suPerNode_;
        int * tpsNode_;
        StealScope stealScope_;
        /* Idle-poll hint: -1 = not set, 0 = off, 1 = on. */
        int idlePollHint_;
    public:
        ThreadAffinity(unsigned int mcpertp, unsigned int numbase, AffinityMode choice, unsigned int tpSched = 0, unsigned int mcSched = 0, bool LLC = false):
        papi(false),
#ifdef COUNT
        L1(false),
        L2(false),
        FpIdle(false),
        Vector(false),
        Stall(false),
#endif
        llc(LLC),
        mcPerTp(mcpertp),
        numTPS(numbase), numMCS(numbase*(mcpertp)),
        TPpolicy(tpSched), MCpolicy(mcSched),
        eventSet(new int[numTPS+numMCS]),
        eventCounter(new long long[(numTPS+numMCS)*NUMEVENTS]),
        mode(choice),
        TPMask(numTPS), MCMask(numMCS),
        stickyPlacement_(-1),
        suPerNode_(0), tpsNode_(NULL), stealScope_(STEAL_LEGACY),
        idlePollHint_(-1)
	{
#ifdef COUNT
	  for(unsigned int i=0;i<numTPS+numMCS;i++)
	  {
	    eventSet[i] = PAPI_NULL;
	  }
#endif
	}
        
        ~ThreadAffinity(void)
	{
	  delete [] eventCounter;
	  delete [] eventSet;
	  delete [] tpsNode_;
	}
        
        bool 		getLLC(void)    { return llc; }
        unsigned int   getNumTPS(void) { return numTPS;  }
        unsigned int   getNumMCS(void) { return numMCS; }
        unsigned int   getNumMcPerTp(void) { return mcPerTp; }
        AffinityMask * getTPMask(void) { return &TPMask; }
        AffinityMask * getMCMask(void) { return &MCMask; }
        unsigned int getTPpolicy(void) { return TPpolicy; }
        unsigned int getMCpolicy(void) { return MCpolicy; }
        AffinityMode getMode(void) const { return mode; }
        /* Make place<> closures sticky: they go to the target scheduler's
         * placed pool, expand only there and are never stolen. Unset, it is
         * on exactly for an accepted NUMA_PAIRED mask. Read once by the
         * Runtime constructor. */
        void setStickyPlacement(bool on) { stickyPlacement_ = on ? 1 : 0; }
        bool stickyPlacement(void) const
        {
            if(stickyPlacement_ >= 0)
                return stickyPlacement_ == 1;
            return mode == NUMA_PAIRED && suPerNode_ == 2;
        }
        /* Node-group layer (NUMA_PAIRED). getSuPerNode() is 0 for every other
         * mode and for a refused mask; nodeOfTps(i) is -1 without a node
         * group or out of range. */
        unsigned getSuPerNode(void) const { return suPerNode_; }
        int nodeOfTps(unsigned i) const { return (tpsNode_ && i < numTPS) ? tpsNode_[i] : -1; }
        /* Steal scope of a NUMA_PAIRED Runtime (default STEAL_LEGACY). Set it
         * before constructing the Runtime. */
        void setStealScope(StealScope s) { stealScope_ = s; }
        StealScope getStealScope(void) const { return stealScope_; }
        /* Idle-poll non-empty hint (TPScheduler::setIdlePollHint): an idle
         * poll skips the lock of a pool whose atomic count is 0. Unset, it
         * is on exactly for an accepted NUMA_PAIRED mask and off for every
         * other mode; setIdlePollHint(true) opts SPREAD/COMPACT in and
         * setIdlePollHint(false) keeps NUMA_PAIRED on the locked polls.
         * Read once by the Runtime constructor. */
        void setIdlePollHint(bool on) { idlePollHint_ = on ? 1 : 0; }
        bool idlePollHint(void) const
        {
            if(idlePollHint_ >= 0)
                return idlePollHint_ == 1;
            return mode == NUMA_PAIRED && suPerNode_ == 2;
        }
        bool generateMask(void);
        void printMask(void);
	bool usePapi(void) { return papi; }
	void initPapi(bool l1, bool l2, bool fpidle, bool vector, bool stall);
	bool threadInitPapi(int threadId);
	void startCounters(int threadId);
	void writeCounters(int threadId);
	long long readCounter(int threadId, int offset);
	void incrementCounters(int threadId);
    };

}

#endif	/* AFFINITY_H */

