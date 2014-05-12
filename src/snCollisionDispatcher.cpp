/****************************************************************************/
/*Copyright (c) 2014, Florent DEVILLE.                                      */
/*All rights reserved.                                                      */
/*                                                                          */
/*Redistribution and use in source and binary forms, with or without        */
/*modification, are permitted provided that the following conditions        */
/*are met:                                                                  */
/*                                                                          */
/* - Redistributions of source code must retain the above copyright         */
/*notice, this list of conditions and the following disclaimer.             */
/* - Redistributions in binary form must reproduce the above                */
/*copyright notice, this list of conditions and the following               */
/*disclaimer in the documentation and/or other materials provided           */
/*with the distribution.                                                    */
/* - The names of its contributors cannot be used to endorse or promote     */
/*products derived from this software without specific prior written        */
/*permission.                                                               */
/* - The source code cannot be used for commercial purposes without         */
/*its contributors' permission.                                             */
/*                                                                          */
/*THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       */
/*"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         */
/*LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         */
/*FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE            */
/*COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,       */
/*INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,      */
/*BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;          */
/*LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER          */
/*CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT        */
/*LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN         */
/*ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           */
/*POSSIBILITY OF SUCH DAMAGE.                                               */
/****************************************************************************/

#include "snCollisionDispatcher.h"
#include "snActorPairManager.h"
#include "snActorPair.h"

#include <thread>
using std::thread;

#include <Windows.h>

namespace Supernova
{
	snCollisionDispatcher::snCollisionDispatcher()
	{}
	
	snCollisionDispatcher::~snCollisionDispatcher()
	{}

	void snCollisionDispatcher::initialize(snScene* _scene, snCollisionCallback _callback, unsigned int _threadCount)
	{
		m_scene = _scene;
		m_callback = _callback;
		m_threadCount = _threadCount;
	}

	void snCollisionDispatcher::dispatch(const snActorPairManager* _pcs) const
	{
		unsigned int pairCount = _pcs->getActivePairsCount();

		//make the pairCount a multiple of threadCount
		int modulo = pairCount % m_threadCount;

		//compute the number of pair to comute per threads
		int pairPerThread = (pairCount + modulo) / m_threadCount;

		//create threads
		thread** runningThread = new thread*[m_threadCount - 1];

		for (unsigned int threadId = 0; threadId < m_threadCount-1; ++threadId)
		{
			unsigned int startId = (threadId + 1) * pairPerThread;
			unsigned int endId = (threadId + 2) * pairPerThread;
			if (endId > pairCount)
				endId = pairCount;
			runningThread[threadId] = new thread(&snCollisionDispatcher::run, this, _pcs, startId, endId);
			SetThreadAffinityMask(runningThread[threadId]->native_handle(), 1 << (threadId + 1));
			
		}

		//run in the current thread
		unsigned int endId = pairPerThread;
		if (endId > pairCount)
			endId = pairCount;
		run(_pcs, 0, endId);

		//wait for all the thread to finish
		for (unsigned int threadId = 1; threadId < m_threadCount; ++threadId)
			runningThread[threadId - 1]->join();

		//delet the threads
		for (unsigned int threadId = 1; threadId < m_threadCount; ++threadId)
		{
			delete runningThread[threadId - 1];
		}
		delete runningThread;
	}

	void snCollisionDispatcher::run(const snActorPairManager* _pcs, unsigned int _startId, unsigned int _endId) const
	{
		const vector<snActorPair*>& pairs = _pcs->getPairs();
		for (unsigned int i = _startId; i != _endId; ++i)
		{
			(m_scene->*m_callback)(pairs[i]->m_first, pairs[i]->m_second);
		}
	}
}