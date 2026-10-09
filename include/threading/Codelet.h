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
#include <stdint.h>
#include "SyncSlot.h"
#include "codeletDefines.h"

namespace darts
{

    //This is a forward declaration since there is a circular dependence
    class ThreadedProcedure;    

    /* Where a ready codelet is enqueued:
     *   SCOPE_SU         the default: placed_ names a TP scheduler
     *                    (setPlacedCluster) or is UNPLACED_CLUSTER (the
     *                    releasing scheduler)
     *   SCOPE_SU_SHARED  placed_ names an SU whose shared pool receives it;
     *                    the SU's sibling may pull it (setPlacedShared)
     *   SCOPE_NODE       placedNode_ names a NUMA node whose node pool
     *                    receives it; either SU of the node may fire it
     *                    (setPlacedNode); placed_ stays UNPLACED_CLUSTER
     * The last two need a NUMA_PAIRED Runtime; without one the push is
     * refused exactly like a refused directed push. */
    enum PlaceScope { SCOPE_SU = 0, SCOPE_SU_SHARED = 1, SCOPE_NODE = 2 };
    /*
		 * Class: Codelet
		 * The codelet class is a virutal class. Use this class to instantiate codelets
		 * and provide a funct method to execute.
		 * 
		 * See Also:
		 * <ThreadedProcedure>
		 * <ABCScheduler>
    */

    class Codelet
    {
    private:
				/*
				 * Variable: status_
				 * The status of the codelet TODO: Explicit?
				*/
        volatile uint32_t status_;
        /* Optional owner TP scheduler (setPlacedCluster). Set before a
         * dependence can be released, so it is immutable while an
         * invocation is in flight. */
        uint32_t placed_;

    protected:
                                /*
				 * Variable: sync_
				 * The codelets counter
				*/
        SyncSlot sync_;
				/*
				 * Variable: myTP_
				 * Pointer to TP frame/context
				*/
        ThreadedProcedure * myTP_;

    private:
        /* Placement scope and node (see PlaceScope). */
        uint32_t placedNode_;
        uint8_t  placeScope_;
        /* The status given to the constructor or initCodelet; rearm()
         * restores it after a DIRECTED_ENQUEUE_FAILED. */
        uint32_t origStatus_;

    public:
        static const uint32_t UNPLACED_CLUSTER = 0xffffffffu;
        /* Status recorded when an explicitly placed ready codelet could not
         * be enqueued on its owner (see onDirectedEnqueueFailure). */
        static const uint32_t DIRECTED_ENQUEUE_FAILED = 0xfffffffeu;
        /**
				 * Constructor: Codelet(uint32_t dep, uint32_t res, ThreadedProcedure * theTp, uint32_t stat);
				 * 
				 * Parameters:
				 *	dep - The dependence counter of the codelet
				 *	res - The value of the dependence counter if the codelet has to be reset
				 *	theTp - The TP the codelet belongs to
				 *	stat - Locality parameter (TODO: has to be explicited)
         */
        Codelet(uint32_t dep, uint32_t res, ThreadedProcedure * theTp=NULL, uint32_t stat=SHORTWAIT);
        Codelet(void);

        //Destructors
        virtual ~Codelet() {}

				/**
				 * Method: initCodelet
         * Implements a delayed Codelet init. We may need them for an array of codelets
				 * when the constructor is not called
				 * 
				 * Parameters:
				 *	dep - The dependence counter of the codelet
				 *	res - The value of the dependence counter if the codelet has to be reset
				 *	theTp - The TP the codelet belongs to
				 *	stat - Locality parameter (TODO: has to be explicited)
         */
        void initCodelet(uint32_t dep, uint32_t res, ThreadedProcedure * theTp, uint32_t stat);

				/**
				 * Method: decDep
         * Decrements the dependence counter of the codelet
         */
        virtual void decDep (void);

        /**
         * Method: tryDecDep
         * The body of decDep. Returns true exactly when this call made the
         * codelet ready and its enqueue was accepted; false when dependences
         * remain, the counter was already zero, or a placed enqueue was
         * refused.
         */
        bool tryDecDep (void);

        /**
         * Method: rearm
         * Re-arms a fired codelet for its next invocation: SyncSlot::rearm()
         * (counter 0 -> reset, CAS) and, on success, a status of
         * DIRECTED_ENQUEUE_FAILED back to the original status. Returns false,
         * changing nothing, while the counter is non-zero. A codelet that is
         * fired repeatedly calls it first thing in fire().
         */
        bool rearm (void);

        /* The status recorded by the constructor or initCodelet. */
        uint32_t getOrigStatus (void) const { return origStatus_; }
        
				/**
				 * Method: resetCodelet
         * Resets the codelet
         */
        void resetCodelet (void);

        /**
				 * Method: resetCodelet
         * Returns:
				 * Whether or not the codelet counter is 0. If it is, all the dependences have been satisfied and
				 * the codelet can now be fired
         */
        bool codeletReady (void);

        /**
				 * Method: setStatus
				 * Sets the status of the codelet (TODO: Has to be explained)
         */
        void setStatus (uint32_t stat);

        /**
				 * Method: getStatus
				 * Gets the status of the codelet (TODO: Has to be explained)
         */
        uint32_t getStatus (void) const;  

        /* Directed placement. A placed codelet is enqueued, when it becomes
         * ready, on TP scheduler `cluster` (an index into the Runtime's
         * scheduler table, the same index place<> uses) instead of on the
         * scheduler that released it. That queue is never drawn by TP
         * stealing, so the codelet fires on the named scheduler. An
         * unplaced codelet (the default) keeps the existing behaviour. */
        void     setPlacedCluster(uint32_t cluster) { placed_ = cluster; placeScope_ = SCOPE_SU; }
        void     clearPlacedCluster(void) { placed_ = UNPLACED_CLUSTER; placeScope_ = SCOPE_SU; }
        uint32_t placedCluster(void) const { return placed_; }
        bool     isPlaced(void) const { return placed_ != UNPLACED_CLUSTER; }
        /* Node scope (NUMA_PAIRED): the node pool of NUMA node `node`.
         * isPlaced() stays false. */
        void     setPlacedNode(uint32_t node) { placed_ = UNPLACED_CLUSTER; placedNode_ = node; placeScope_ = SCOPE_NODE; }
        /* Shared scope (NUMA_PAIRED): the shared pool of SU `su`. */
        void     setPlacedShared(uint32_t su) { placed_ = su; placeScope_ = SCOPE_SU_SHARED; }
        PlaceScope placeScope(void) const { return (PlaceScope)placeScope_; }
        uint32_t placedNode(void) const { return placedNode_; }

        /* A placed codelet whose enqueue is refused (index out of range,
         * released from a thread that belongs to no DARTS scheduler, or a
         * failing queue) is never run somewhere else: its status becomes
         * DIRECTED_ENQUEUE_FAILED, onDirectedEnqueueFailure() is called, and
         * the TP reference taken for the enqueue is rolled back. The hook may
         * release references it owns but must not delete this codelet or
         * its TP. */
        bool directedEnqueueFailed(void) const;
        void notifyDirectedEnqueueFailure(uint64_t target);
        virtual void onDirectedEnqueueFailure(uint64_t target) { (void)target; }

				/**
				 * Method: getTP
				 * Returns:
				 * The parent ThreadedProcedure pointer
         */
        ThreadedProcedure * getTP(void);
				
				/**
				 * Method: setTP
				 * Sets the parent ThreadedProcedure pointer
         */
        void setTP(ThreadedProcedure * aTP);
				
				/**
				 * Method: casStatus
				 * TODO description
         */
        bool casStatus(uint32_t oldval, uint32_t newval);

				/**
				 * Method: Do
				 * Wrapper to push codelets to the parent scheduler
         */
        void add(Codelet * aCodelet);
        void add() { add(this); }
				
        /**
				 * Method: getCounter
				 * Returns:
				 * The dependence counter of the codelet
         */
        uint32_t getCounter(void) const;
        
        SyncSlot * getSyncSlot(void);
        
				/**
				 * Method: funct
				 * This is the code of the codelet to be executed.
         */
        virtual void fire(void) = 0;
                 
        #ifdef TRACE
        void * returnFunct(void);
        #endif
    };    
} // namespace darts
