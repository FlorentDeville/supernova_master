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

#include "snSweepManager.h"
#include "snRigidbody.h"
#include "snActorPair.h"

#ifdef SN_DEBUGGER
#include "snDebugger.h"
#endif //idef SN_DEBUGGER

using namespace Supernova::Vector;

namespace Supernova
{
	//Default constructor
	snSweepManager::snSweepManager() : m_sortedActors(), m_axis(0), m_deferredAxis(0), m_callback(0), m_sum(), m_squaredSum(), m_scene(0)
	{}

	//Default destructor
	snSweepManager::~snSweepManager()
	{}

	//Clear the list of actors
	void snSweepManager::clearList()
	{
		m_sortedActors.clear();
	}

	//Set the callback function to call when a possibly colliding pair of actors is found.
	void snSweepManager::setCallback(snScene* _scene, snPairFoundCallback _callback)
	{
		m_scene = _scene;
		m_callback = _callback;
	}

	//Add the actor to the list of actors to use in the sweep and prune process.
	void snSweepManager::addActor(snRigidbody* _actor)
	{
		m_sortedActors.push_back(_actor);
	}

	void snSweepManager::removeActor(snRigidbody const * const _actor)
	{
		for (list<snRigidbody*>::iterator i = m_sortedActors.begin(); i != m_sortedActors.end(); ++i)
		{
			if ((*i) == _actor)
			{
				m_sortedActors.erase(i);
				return;
			}
		}
	}

	//Sort the list using actor's aabbs.
	void snSweepManager::preBroadPhase()
	{
		m_axis = m_deferredAxis;
		//sort the list in ascending order
		m_sortedActors.sort([this](const snRigidbody* _a, const snRigidbody* _b)
		{
			return snVec4GetById(_a->getBoundingVolume()->m_min, m_axis) < snVec4GetById(_b->getBoundingVolume()->m_min, m_axis);
		});

		m_sum = snVec4Set(0, 0, 0, 0);
		m_squaredSum = snVec4Set(0, 0, 0, 0);

#ifdef SN_DEBUGGER
		//Number of pair found 
		m_collisionPairFound = 0;
#endif //ifdef SN_DEBUGGER

	}

	//Check for possibily colliding pair of actors and call the callback function everytime a pair is found.
	void snSweepManager::broadPhase()
	{
		vector<snActorPair> pcs;

		//loop through each actor in the scene using the sweep list
		for (list<snRigidbody*>::iterator i = m_sortedActors.begin(); i != m_sortedActors.end(); ++i)
		{
			if (!(*i)->getIsActive())
				continue;

			//compute aabb center point
			snVec center = ((*i)->getBoundingVolume()->m_max + (*i)->getBoundingVolume()->m_min) * 0.5f;

			//compute sum and sum squared to compute variance later
			m_sum = m_sum + center;
			m_squaredSum = m_squaredSum + (center * center);

			//test collision against all other actors
			list<snRigidbody*>::iterator j = i;
			++j;
			while (j != m_sortedActors.end())
			{
				if (!(*j)->getIsActive())
				{
					++j;
					continue;
				}

				if((!(*i)->isAwake() && !(*j)->isAwake()) //Do not check sleeping bodies
					|| ((*i)->isStatic() && !(*j)->isAwake()) //Do not check a static and a sleeping
					|| (!(*i)->isAwake() && (*j)->isStatic()) //Do not check a sleeping and a static.
					) 
				{
					++j;
					continue;
				}

				//check if the collision detection is enabled between the two actors
				if (!snRigidbody::isCollisionDetectionEnabled(*i, *j))
				{
					++j;
					continue;
				}

				//check if the tested bounding volume(j) is too far to the current bounding volume (i)
				if (snVec4GetById((*j)->getBoundingVolume()->m_min, m_axis) > snVec4GetById((*i)->getBoundingVolume()->m_max, m_axis))
					break;

				if (AABBOverlap((*i)->getBoundingVolume(), (*j)->getBoundingVolume()))
				{
					//a pair is found, call the callback
					//(m_scene->*m_callback)(*i, *j);
					snActorPair pair;
					pair.m_first = *i;
					pair.m_second = *j;
					pcs.push_back(pair);

#ifdef SN_DEBUGGER
					//Number of pair found 
					++m_collisionPairFound;
#endif //ifdef SN_DEBUGGER
				}
				++j;
			}
		}

#ifdef SN_DEBUGGER
		//Number of pair found 
		DEBUGGER->setWatchExpression(L"Collision Pair Found", std::to_wstring(m_collisionPairFound));
#endif //ifdef SN_DEBUGGER

		//Collision detection through the possibly colliding set.
		for(vector<snActorPair>::const_iterator i = pcs.cbegin(); i != pcs.cend(); ++i)
		{
			(m_scene->*m_callback)(i->m_first, i->m_second);
		}
	}

	//Compute which axis to use to sort the actors.
	void snSweepManager::postBroadPhase()
	{
		//compute variance (no need to divide by the number of elements)
		snVec v = m_squaredSum - (m_sum * m_sum);
		snVec4Absolute(v);

		//update the axis to sort to take the axis with the greatest variance.
		m_deferredAxis = 0;
		if (snVec4GetById(v, 1) > snVec4GetById(v, 0)) m_deferredAxis = 1;
		if (snVec4GetById(v, 2) > snVec4GetById(v, m_deferredAxis)) m_deferredAxis = 2;
	}

	//Find all the possibly colliding actors for a given aabb.
	void snSweepManager::getPossiblyCollidingActor(const snAABB& _aabb, vector<snRigidbody*>& _pca) const
	{
		//loop through each actor in the scene using the sweep list
		for (list<snRigidbody*>::const_iterator i = m_sortedActors.begin(); i != m_sortedActors.end(); ++i)
		{
			//ignore inactive actors
			if (!(*i)->getIsActive())
				continue;

			//check if the aabb is before the actor
			if (snVec4GetById(_aabb.m_max, m_axis) < snVec4GetById((*i)->getBoundingVolume()->m_min, m_axis))
				continue;

			//check if theaabb is too far to the current bounding volume (i)
			if (snVec4GetById(_aabb.m_min, m_axis) > snVec4GetById((*i)->getBoundingVolume()->m_max, m_axis))
				break;

			//Check if the bounding boxes overlap
			if (AABBOverlap(&_aabb, (*i)->getBoundingVolume()))
			{
				_pca.push_back(*i);
			}
		}
	}
}