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
#include "Scheduler.h"
#include "tpClosure.h"
#include "ThreadedProcedure.h"
#include <vector>
#include <stdlib.h>
#include "dartsPool.h"
#include "Atomics.h"
#include "StealScope.h"

#ifdef TRACE
#include "getClock.h"
#endif

namespace darts
{
    enum TPSCHED {TPPUSHFULL      = 0, 
                  TPROUNDROBIN    = 1, 
                  TPSTATIC        = 2,
                  TPDYNAMIC       = 3,
                  TPWORKPUSH      = 4};


    class TPScheduler : public Scheduler
    {
    private:
        size_t numberOfPeers;
        TPScheduler** peers_;
        std::vector<Scheduler*> children_;
        
    protected:
        /* Stealable TP closures (invoke<>, pushTP). */
        dartsPool<tpClosure*> ready_;
        /* Placed TP closures (pushTPPlaced): this scheduler is their
         * destination and steal() never draws from here. */
        dartsPool<tpClosure*> placed_;
        dartsPool<Codelet*> codelets_;
        /* Node-group layer (NUMA_PAIRED only). NULL / 0 / STEAL_LEGACY
         * unless the Runtime called setNodeGroup(), so a runtime without a
         * node group never touches any of it except the one nodeCodelets_
         * test after an empty codelets_ pop. shared_ holds this SU's
         * setPlacedShared() codelets; nodeCodelets_ / nodeTPs_ are the pools
         * of this SU's NUMA node, shared with the sibling SU. */
        dartsPool<Codelet*> shared_;
        TPScheduler * sibling_;
        dartsPool<Codelet*> * nodeCodelets_;
        dartsPool<tpClosure*> * nodeTPs_;
        unsigned node_;
        StealScope stealScope_;
        uint64_t rng_;
        /* Codelets this SU took from its node pool / its sibling's shared
         * pool (only counted with a node group). */
        volatile uint64_t nodePulls_;
        volatile uint64_t siblingPulls_;

    public:
        
        TPScheduler(void):
        numberOfPeers(0),
        peers_(NULL),
        sibling_(NULL),
        nodeCodelets_(NULL),
        nodeTPs_(NULL),
        node_(0),
        stealScope_(STEAL_LEGACY),
        rng_(0),
        nodePulls_(0),
        siblingPulls_(0),
        placedCount_(0),
        clusterIndex_(0),
        stickyPlacement_(false)
        {
            
        }

        /* Join this SU to its node group. Called once by the Runtime of a
         * NUMA_PAIRED layout, before any worker starts. Seeds the xorshift64
         * state of the SIBLING_THEN_ANY victim draw from the SU index. */
        void setNodeGroup(unsigned node, TPScheduler * sibling,
                          dartsPool<Codelet*> * nodeCodelets,
                          dartsPool<tpClosure*> * nodeTPs, StealScope scope)
        {
            node_         = node;
            sibling_      = sibling;
            nodeCodelets_ = nodeCodelets;
            nodeTPs_      = nodeTPs;
            stealScope_   = scope;
            rng_          = 0x9E3779B97F4A7C15ULL * ((uint64_t)clusterIndex_ + 1);
        }
        bool       hasNodeGroup(void) const  { return nodeCodelets_ != NULL; }
        unsigned   getNode(void) const       { return node_; }
        StealScope getStealScope(void) const { return stealScope_; }
        TPScheduler * getSibling(void) const { return sibling_; }
        uint64_t   nodePulls(void)           { return Atomics::load(nodePulls_); }
        uint64_t   siblingPulls(void)        { return Atomics::load(siblingPulls_); }

        /* This scheduler's index in the Runtime's scheduler table, i.e. the
         * index place<> and pushCodeletTo() resolve a target to. getID() is
         * the global thread id and differs from it whenever there are micro
         * schedulers. Set by the Runtime when the scheduler is created. */
        void     setClusterIndex(unsigned idx) { clusterIndex_ = idx; }
        unsigned getClusterIndex(void) const   { return clusterIndex_; }

        /* Whether place<> pushes to placed_ (sticky, never stolen) instead
         * of the stealable ready_ pool. Off by default, which keeps the
         * existing place<> behaviour; the Runtime sets it from
         * ThreadAffinity::stickyPlacement() before any worker starts. */
        void setStickyPlacement(bool on)  { stickyPlacement_ = on; }
        bool stickyPlacement(void) const  { return stickyPlacement_; }
        
	~TPScheduler(void){}

        Scheduler *
        getSubScheduler(size_t pos) const
        {
            if(children_.size()>pos)
                return children_.at(pos);
            return 0;
        }

        void
        setSubScheduler(Scheduler * aSub)
        {
            children_.push_back(aSub);
        }

        size_t 
        getNumSub(void) const
        {
            return children_.size();
        }
        
        void addPeer(TPScheduler * toAdd, size_t pos)
        {
            peers_[pos] = toAdd;
        }

        void
        setNumPeers(size_t numPeers)
        {
            numberOfPeers = numPeers;
            if(!peers_)
                peers_ = new TPScheduler*[numPeers];
        }
                
        size_t 
        getNumPeers(void) const
        {
            return numberOfPeers;
        }

        TPScheduler * 
        getPeer(size_t pos) const
        {
            if(pos<numberOfPeers)
                return peers_[pos];
            return 0;
        }
        
        /* Steal a closure from a random peer's stealable pool. A placed
         * closure is never in that pool; placedSteals() counts any that
         * would be returned anyway, so the invariant is measured. */
        tpClosure *
        steal(void)
        {
            if(stealScope_ == STEAL_LEGACY)
            {
            if(numberOfPeers)
            {
                uint64_t random = rand() % numberOfPeers;
                if(random!=getID())
                {
                    tpClosure * stolen = peers_[random]->popTPStealable();
                    if(stolen && stolen->sticky)
                        Atomics::fetchAdd(placedSteals_, (uint64_t)1);
                    return stolen;
                }
            }
            return NULL;
            }
            if(stealScope_ == STEAL_NONE)
                return NULL;
            if(sibling_)
            {
                tpClosure * c = sibling_->popTPStealableHead();
                if(c)
                {
                    if(c->sticky)
                        Atomics::fetchAdd(placedSteals_, (uint64_t)1);
                    return c;
                }
            }
            if(stealScope_ == STEAL_SIBLING_THEN_ANY)
                return stealAnyScoped();
            return NULL;
        }  

        /* SIBLING_THEN_ANY second stage: one xorshift64 victim drawn
         * uniformly from the SUs other than this one and its sibling (no
         * draw is wasted on itself). Stealable closures only. */
        tpClosure *
        stealAnyScoped(void)
        {
            const size_t self = clusterIndex_;
            const size_t sib  = sibling_ ? sibling_->getClusterIndex() : self;
            const size_t lo   = (self < sib) ? self : sib;
            const size_t hi   = (self < sib) ? sib : self;
            const size_t excluded = (lo == hi) ? 1 : 2;
            if(numberOfPeers <= excluded)
                return NULL;
            rng_ ^= rng_ << 13;
            rng_ ^= rng_ >> 7;
            rng_ ^= rng_ << 17;
            size_t victim = (size_t)(rng_ % (uint64_t)(numberOfPeers - excluded));
            if(victim >= lo) ++victim;
            if(excluded == 2 && victim >= hi) ++victim;
            tpClosure * stolen = peers_[victim]->popTPStealable();
            if(stolen && stolen->sticky)
                Atomics::fetchAdd(placedSteals_, (uint64_t)1);
            return stolen;
        }

        /* Placed closures ever returned by steal(); stays 0. */
        static uint64_t placedSteals(void) { return Atomics::load(placedSteals_); }
        
        virtual void policy(void) = 0;
                
        virtual bool 
        pushTP(tpClosure * TPtoPush)
        {
            return ready_.push(TPtoPush);
        }
        
        /* Push a closure that must expand on this scheduler. */
        virtual bool
        pushTPPlaced(tpClosure * TPtoPush)
        {
            TPtoPush->sticky = true;
            Atomics::fetchAdd(placedCount_, (uint64_t)1);
            if(placed_.push(TPtoPush))
                return true;
            Atomics::fetchSub(placedCount_, (uint64_t)1);
            return false;
        }

        /* The push place<> uses: sticky when stickyPlacement() is on,
         * otherwise the scheduler's ordinary pushTP(). */
        bool
        placeTP(tpClosure * TPtoPush)
        {
            return stickyPlacement_ ? pushTPPlaced(TPtoPush) : pushTP(TPtoPush);
        }

        /* Own-policy pop: placed work first, then stealable work.
         * placedCount_ is only a hint that keeps the common no-placed-work
         * case at one load instead of a pool lock; correctness comes from
         * the separate pools. */
        virtual tpClosure * 
        popTP(void)
        {
            if(Atomics::load(placedCount_))
            {
                tpClosure * placed = placed_.pop();
                if(placed)
                {
                    Atomics::fetchSub(placedCount_, (uint64_t)1);
                    return placed;
                }
            }
            /* Node-placed closures (placeNode<>), shared with the sibling
             * SU. NULL without a node group. */
            if(nodeTPs_)
                if(tpClosure * c = nodeTPs_->pop())
                    return c;
            return ready_.pop();
        }

        /* Steal-path pop: stealable work only. */
        virtual tpClosure *
        popTPStealable(void)
        {
            return ready_.pop();
        }

        /* Sibling steal: the oldest stealable closure (the owner pops the
         * newest). */
        tpClosure *
        popTPStealableHead(void)
        {
            return ready_.popHead();
        }
        
        virtual bool 
        pushCodelet(Codelet * CodeletToPush)
        {
            return codelets_.push(CodeletToPush);
        }
        
        /* Own queue first: without a node group that is the whole function
         * plus one predictable branch. With one: this SU's shared pool, then
         * the node pool, then (SIBLING scopes) the sibling SU's shared pool.
         * Never another SU's own queue. */
        virtual Codelet * 
        popCodelet(void)
        {
            Codelet * c = codelets_.pop();
            if(c || !nodeCodelets_)
                return c;
            if((c = shared_.pop()))
                return c;
            if((c = nodeCodelets_->pop()))
            {
                Atomics::fetchAdd(nodePulls_, (uint64_t)1);
                return c;
            }
            if((stealScope_ == STEAL_SIBLING || stealScope_ == STEAL_SIBLING_THEN_ANY)
               && sibling_ && (c = sibling_->shared_.popHead()))
            {
                Atomics::fetchAdd(siblingPulls_, (uint64_t)1);
                return c;
            }
            return NULL;
        }

        /* Enqueue a ready codelet on TP scheduler `cluster`, an index into
         * the Runtime's scheduler table (the index place<> uses). The index
         * is bounds checked, not wrapped. Refused, and counted in
         * directedRefused(), when the index is out of range, when the
         * calling thread belongs to no DARTS scheduler, or when the queue
         * push fails; the caller keeps ownership of the codelet. */
        static bool pushCodeletTo(uint64_t cluster, Codelet * cd);
        static uint64_t directedRefused(void) { return Atomics::load(directedRefused_); }

        /* Node-group pushes (NUMA_PAIRED). Each is bounds checked, refused
         * from a thread that belongs to no DARTS scheduler, and refused when
         * the target has no node group (any other runtime); every refusal
         * returns false, leaves ownership with the caller and is counted in
         * nodeGroupRefused().
         *   pushCodeletToNode  the codelet pool of NUMA node `node`
         *   pushCodeletShared  the shared pool of SU `su`
         *   pushTPNode         the closure pool of NUMA node `node` (not
         *                      sticky: either SU of the node expands it, and
         *                      steal() never draws from it) */
        static bool pushCodeletToNode(uint64_t node, Codelet * cd);
        static bool pushCodeletShared(uint64_t su, Codelet * cd);
        static bool pushTPNode(uint64_t node, tpClosure * closure);
        static uint64_t nodeGroupRefused(void) { return Atomics::load(nodeGroupRefused_); }
        
        static TPScheduler * create(unsigned int type);

    private:
        volatile uint64_t placedCount_;
        unsigned          clusterIndex_;
        bool              stickyPlacement_;
        static volatile uint64_t placedSteals_;
        static volatile uint64_t directedRefused_;
        static volatile uint64_t nodeGroupRefused_;
    };
}

