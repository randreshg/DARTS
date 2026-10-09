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


/*The all methods are separated because there is a circular dependency
 between codelet and threaded procedure classes*/

//Don't Mess with the order of these includes
#include "codeletDefines.h"
#include "SyncSlot.h"
#include "Codelet.h"
#include "ThreadedProcedure.h"
#include "threadlocal.h"
#include "MSchedPolicy.h"
#include "TPScheduler.h"
#include <cassert>

namespace darts
{
    const uint32_t Codelet::UNPLACED_CLUSTER;
    const uint32_t Codelet::DIRECTED_ENQUEUE_FAILED;

    /* A ready codelet holds one temporary reference on its TP until its
     * enqueue is accepted. A refused directed enqueue rolls it back with the
     * scheduler's delete-on-last-reference rule for child TPs; a root TP is
     * owned by its creator and is only decremented. */
    static void releaseDirectedReference(ThreadedProcedure * tp)
    {
        if(!tp)
            return;
        const bool deleteTP = tp->checkParent();
        if(tp->decRef() && deleteTP)
            delete tp;
    }
    
    Codelet::Codelet(uint32_t dep, uint32_t res, ThreadedProcedure * theTp, uint32_t stat):
    status_(stat),
    placed_(UNPLACED_CLUSTER),
    sync_(dep,res),
    myTP_(theTp),
    placedNode_(0),
    placeScope_(SCOPE_SU),
    origStatus_(stat)
    {
    }

    Codelet::Codelet(void):
    status_(NIL),
    placed_(UNPLACED_CLUSTER),
    sync_(0U,0U),
    myTP_(0),
    placedNode_(0),
    placeScope_(SCOPE_SU),
    origStatus_(NIL) { }

    /* The node-group scopes. Returns whether the push was accepted and the
     * target to report on refusal. */
    static bool pushScoped(Codelet * cd, uint64_t * target)
    {
        if(cd->placeScope() == SCOPE_NODE)
        {
            *target = cd->placedNode();
            return TPScheduler::pushCodeletToNode(cd->placedNode(), cd);
        }
        *target = cd->placedCluster();
        return TPScheduler::pushCodeletShared(cd->placedCluster(), cd);
    }

    void
    Codelet::initCodelet(uint32_t dep, uint32_t res, ThreadedProcedure * theTp, uint32_t stat)
    {
        sync_.initSyncSlot(dep,res);
        status_ = stat ;
        origStatus_ = stat;
        myTP_ = theTp;
    }

    void
    Codelet::decDep(void)
    {
        (void)tryDecDep();
    }

    bool
    Codelet::tryDecDep(void)
    {
        if(sync_.decCounter())
        {
            ThreadedProcedure * tp = myTP_;
            if(tp)
                tp->incRef();
            if(placeScope_ != SCOPE_SU)
            {
                uint64_t target = 0;
                if(pushScoped(this, &target))
                    return true;
                notifyDirectedEnqueueFailure(target);
                releaseDirectedReference(tp);
                return false;
            }
            if(placed_ != UNPLACED_CLUSTER)
            {
                if(TPScheduler::pushCodeletTo(placed_, this))
                    return true;
                /* Running a placed codelet on the releasing scheduler would
                 * break its placement: report the failure instead. */
                notifyDirectedEnqueueFailure(placed_);
                releaseDirectedReference(tp);
                return false;
            }
            if(myThread.threadMCsched)
            {
                if(myThread.threadMCsched->getLocal())
                {
                        if(myThread.threadMCsched->pushLocal(this))
                                return true;
                }
            }
            myThread.threadTPsched->pushCodelet(this);
            return true;
        }
        return false;
    }

    bool
    Codelet::rearm(void)
    {
        if(!sync_.rearm())
            return false;
        /* Only the terminal failure marker is cleared; a status the owner
         * set on purpose stays. */
        (void)Atomics::boolcompareAndSwap(status_, DIRECTED_ENQUEUE_FAILED, origStatus_);
        return true;
    }

    bool
    Codelet::directedEnqueueFailed(void) const
    {
        return Atomics::load(status_) == DIRECTED_ENQUEUE_FAILED;
    }

    void
    Codelet::notifyDirectedEnqueueFailure(uint64_t target)
    {
        (void)Atomics::swap(status_, DIRECTED_ENQUEUE_FAILED);
        onDirectedEnqueueFailure(target);
    }

    void 
    Codelet::resetCodelet(void)
    {
        sync_.resetCounter();
        //The check is just in case we are reseting the final codelet
        //if(sync_.ready())
        //{
            //myTP_->incRef();
           //myThread.threadTPsched->placeCodelet(this);
        //}
    }

    bool 
    Codelet::codeletReady(void)
    {
        return sync_.ready();
    }

    void 
    Codelet::setStatus(uint32_t stat)
    {
        status_ = stat;
    }

    uint32_t Codelet::getStatus(void) const{
        return status_;
    }
    
    bool
    Codelet::casStatus(uint32_t oldval, uint32_t newval )
    {
        return Atomics::boolcompareAndSwap(status_,oldval,newval);
    }

    ThreadedProcedure * 
    Codelet::getTP(void)
    {
        return myTP_;
    }

    void 
    Codelet::setTP(ThreadedProcedure * aTP)
    {
        myTP_ = aTP;
    }

    void
    Codelet::add(Codelet * aCodelet)
    {
        if(aCodelet->codeletReady())
        {
            if(aCodelet->placeScope() != SCOPE_SU)
            {
                /* The reference is taken on the added codelet's own TP,
                 * which is the one the scheduler releases after its fire. */
                ThreadedProcedure * own = aCodelet->getTP();
                if(own)
                    own->incRef();
                uint64_t target = 0;
                if(pushScoped(aCodelet, &target))
                    return;
                aCodelet->notifyDirectedEnqueueFailure(target);
                releaseDirectedReference(own);
                return;
            }
            if(aCodelet->isPlaced())
            {
                ThreadedProcedure * tp = myTP_;
                if(tp)
                    tp->incRef();
                if(TPScheduler::pushCodeletTo(aCodelet->placedCluster(), aCodelet))
                    return;
                aCodelet->notifyDirectedEnqueueFailure(aCodelet->placedCluster());
                releaseDirectedReference(tp);
                return;
            }
            /* A codelet without a TP (initCodelet(..., NULL, ...)) has no
             * reference to take; the scheduler skips the release for it. */
            if(myTP_)
                myTP_->incRef();
            myThread.threadTPsched->pushCodelet(aCodelet);
        }
    }

    uint32_t 
    Codelet::getCounter(void) const{
        return sync_.getCounter();
    }
    
    SyncSlot *
    Codelet::getSyncSlot(void)
    {
        return &sync_;
    }

    
    #ifdef TRACE
    void *
    Codelet::returnFunct(void)
    {
        void (Codelet::*f)(void) = &Codelet::fire; 
        return (void*) f;
    }
    #endif
    
} // namespace darts
