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

#include "snCollision.h"
#include "snColliderBox.h"
#include "snColliderSphere.h"
#include "snColliderPlan.h"

#include "snCollisionResult.h"
#include "snColliderContainer.h"
#include "snICollider.h"
#include "snIActor.h"
#include "snMath.h"
#include "snFeatureClipping.h"
#include <assert.h>

using namespace Supernova::Vector;

namespace Supernova
{
	snCollision::snCollision()
	{
		//initialize the collision query map
		m_collisionQueryMap.insert(snCollisionQueryMapElement(SN_COLLISION_KEY(snEColliderBox, snEColliderBox), &Supernova::snCollision::queryTestCollisionBoxVersusBox));
		m_collisionQueryMap.insert(snCollisionQueryMapElement(SN_COLLISION_KEY(snEColliderSphere, snEColliderSphere), &Supernova::snCollision::queryTestCollisionSphereVersusSphere));
		m_collisionQueryMap.insert(snCollisionQueryMapElement(SN_COLLISION_KEY(snEColliderBox, snEColliderSphere), &Supernova::snCollision::queryTestCollisionBoxVersusSphere));
		m_collisionQueryMap.insert(snCollisionQueryMapElement(SN_COLLISION_KEY(snEColliderBox, snEColliderPlan), &Supernova::snCollision::queryTestCollisionBoxVersusPlan));
		m_collisionQueryMap.insert(snCollisionQueryMapElement(SN_COLLISION_KEY(snEColliderSphere, snEColliderPlan), &Supernova::snCollision::queryTestCollisionSphereVersusPlan));
	}

	snCollision::~snCollision(){}

	void snCollision::queryTestCollision(snIActor* _a1, snIActor* _a2, snCollisionResult* _results, unsigned int _maxResultCount, unsigned int* _resultsCount) const
	{
		std::vector<snColliderContainer*>& listColliders1 = _a1->getColliders();
		std::vector<snColliderContainer*>& listColliders2 = _a2->getColliders();

		*_resultsCount = 0;
		for (vector<snColliderContainer*>::const_iterator c1 = listColliders1.cbegin(); c1 != listColliders1.cend(); ++c1)
		{
			for (vector<snColliderContainer*>::const_iterator c2 = listColliders2.cbegin(); c2 != listColliders2.cend(); ++c2)
			{
				//pouahhhh it's ugly!!!! let's check first the AABBs
				_results[*_resultsCount] = invokeQueryTestCollision((*c1)->m_collider, (*c2)->m_collider);
				if (_results[*_resultsCount].m_collision)
				{
					++(*_resultsCount);

					//If we already found _resultsCount collision, then we can't store the next collision anymore so return.
					if (*_resultsCount >= _maxResultCount)
						return;
				}
			}
		}

	}

	snCollisionResult snCollision::invokeQueryTestCollision(const snICollider* const _c1, const snICollider* const _c2) const
	{
		unsigned short key = SN_COLLISION_KEY(_c1->getTypeOfCollider(), _c2->getTypeOfCollider());
		snCollisionQueryMap::const_iterator i = m_collisionQueryMap.find(key);
		
		if (i != m_collisionQueryMap.cend())
		{
			snQueryTestCollisionFunction func = i->second;
			return func(_c1, _c2);
		}
		else
		{
			key = SN_COLLISION_KEY(_c2->getTypeOfCollider(), _c1->getTypeOfCollider());
			i = m_collisionQueryMap.find(key);
			snQueryTestCollisionFunction func = i->second;
			snCollisionResult res = func(_c2, _c1);
			res.m_normal = -res.m_normal;
			return res;
		}
			
	}

	snCollisionResult snCollision::queryTestCollisionBoxVersusBox(const snICollider* const _c1, const snICollider* const _c2)
	{
		const snColliderBox* _b1 = static_cast<const snColliderBox*>(_c1);
		const snColliderBox* _b2 = static_cast<const snColliderBox*>(_c2);

		snCollisionResult res;

		const snVec* s1Normals;
		const snVec* s2Normals;

		s1Normals = _b1->getWorldNormal();
		s2Normals = _b2->getWorldNormal();

		snVec smallestOverlapNormal;
		float smallestOverlap = SN_FLOAT_MAX;		

		//compute collider overlap using the normals
		const float NORMAL_COUNT = 3;
		for (int i = 0; i < NORMAL_COUNT; ++i)
		{
			bool overlapRes = true;		
			overlapRes = computeBoxesOverlap(*_b1, *_b2, s1Normals[i], smallestOverlapNormal, smallestOverlap);
			if (!overlapRes) return res;		
			overlapRes = computeBoxesOverlap(*_b1, *_b2, s2Normals[i], smallestOverlapNormal, smallestOverlap);
			if (!overlapRes) return res;		
		}	

		//compute overlap for the cross product of the normals
		for (int i = 0; i < NORMAL_COUNT; ++i)
		{
			for (int j = i; j < NORMAL_COUNT; ++j)
			{
				snVec cross = snVec3Cross(s1Normals[i], s2Normals[j]);
				if (snVec3SquaredNorme(cross) != 1.f) continue;
				bool overlapRes = computeBoxesOverlap(*_b1, *_b2, cross, smallestOverlapNormal, smallestOverlap);
				if (!overlapRes) return res;
			}
		}

		//there is a collision so find the collision patch
		res.m_collision = true;
		res.m_normal = smallestOverlapNormal;

		assert(smallestOverlap > 0.f);

		//find collision patch using clipping algorithm.
		snFeatureClipping clipping;
		bool clippingResult = clipping.findContactPatch(*_b1, *_b2, smallestOverlapNormal, res.m_contacts, res.m_penetrations);
		res.m_collision = clippingResult;
		return res;
	}

	snCollisionResult snCollision::queryTestCollisionSphereVersusSphere(const snICollider* const _c1, const snICollider* const _c2)
	{
		const snColliderSphere* _s1 = static_cast<const snColliderSphere*>(_c1);
		const snColliderSphere* _s2 = static_cast<const snColliderSphere*>(_c2);

		snCollisionResult res;

		//Get the vector between the sphere's origins.
		snVec vecDistance = _s1->getWorldOrigin() - _s2->getWorldOrigin();

		//Calculate the minimum distance between the spheres
		float squaredMinDistance = _s1->getRadius() + _s2->getRadius();
		squaredMinDistance *= squaredMinDistance;

		//If the distance between the centers is inferior to the sum or radius then collision.
		if (squaredMinDistance > snVec3SquaredNorme(vecDistance))
		{
			res.m_collision = true;
			res.m_normal = vecDistance;
			snVec3Normalize(res.m_normal);
			snVec4SetW(res.m_normal, 0);
			res.m_penetrations.push_back(sqrtf(squaredMinDistance) - snVec3Norme(vecDistance));

		}
		

		return res;
	}

	snCollisionResult snCollision::queryTestCollisionBoxVersusSphere(const snICollider* const _c1, const snICollider* const _c2)
	{
		const snColliderBox* _box = static_cast<const snColliderBox*>(_c1);
		const snColliderSphere* _sphere = static_cast<const snColliderSphere*>(_c2);

		snCollisionResult res;

		//Get the closest point to the box.
		snVec closestPoint = _box->getClosestPoint(_sphere->getWorldOrigin());

		//Get the distance (squared) between the closest point and the sphere center.
		snVec closestPointToSphere = closestPoint - _sphere->getWorldOrigin();
		float squaredDistanceClosestPoint = snVec3SquaredNorme(closestPointToSphere);
		float squaredRadius = _sphere->getRadius() * _sphere->getRadius();

		//If the closest point distance is bigger than the radius then no collision (all distances are squared).
		if (squaredDistanceClosestPoint >= squaredRadius)
			return res;

		res.m_collision = true;
		res.m_normal = closestPointToSphere;
		snVec3Normalize(res.m_normal);
		snVec4SetW(res.m_normal, 0);
		snVec contactPoint = _sphere->getWorldOrigin() + res.m_normal * _sphere->getRadius();
		res.m_contacts.push_back(contactPoint);
		res.m_penetrations.push_back(snVec3Norme(contactPoint - closestPoint));
		return res;
	}

	snCollisionResult snCollision::queryTestCollisionBoxVersusPlan(const snICollider* const _c1, const snICollider* const /*_c2*/)
	{
		const snColliderBox* _box = static_cast<const snColliderBox*>(_c1);
		const snColliderPlan* _plan = static_cast<const snColliderPlan*>(_c1);

		//get the distance between the box center and the plan
		float boxDistance = fabsf(_plan->getDistance(_box->getWorldOrigin()));
		
		//get the extends
		snVec extends = _box->getSize() * 0.5f;

		//get the box normals
		const snVec* s1Normals = _box->getWorldNormal();

		//compute the minimum distance between the box and the plan
		float minDistance = snVec4GetX(extends) * fabsf(snVec3Dot(s1Normals[0], _plan->getWorldNormal())) +
			snVec4GetY(extends) * fabsf(snVec3Dot(s1Normals[1], _plan->getWorldNormal())) +
			snVec4GetZ(extends) * fabsf(snVec3Dot(s1Normals[2], _plan->getWorldNormal()));
		
		//compare the real distance to the min distance
		float overlap = boxDistance - minDistance;

		snCollisionResult res;
		if (overlap < 0)
		{
			res.m_collision = true;
			res.m_normal = _plan->getWorldNormal();
			res.m_penetrations.push_back(fabs(overlap));
		}
		else
			res.m_collision = false;
		
		return res;
	}

	snCollisionResult snCollision::queryTestCollisionSphereVersusPlan(const snICollider* const _c1, const snICollider* const /*_c2*/)
	{
		const snColliderSphere* _sphere = static_cast<const snColliderSphere*>(_c1);
		const snColliderPlan* _plan = static_cast<const snColliderPlan*>(_c1);

		snCollisionResult res;

		//get the distance between the sphere center and the plan
		float distance = _plan->getDistance(_sphere->getWorldOrigin());

		if (distance > _sphere->getRadius())
			return res;
		else
		{
			res.m_collision = true;
			res.m_normal = _plan->getWorldNormal();
			res.m_penetrations.push_back(_sphere->getRadius() - distance);
			return res;
		}
	}

	bool snCollision::computeBoxesOverlap(const snColliderBox& _c1, const snColliderBox& _c2, const snVec& _axis, snVec& _separatingAxis, float& _overlap)
	{
		float minS1, minS2, maxS1, maxS2;

		_c1.computeProjection(_axis, minS2, maxS2);
		_c2.computeProjection(_axis, minS1, maxS1);

		float diff1 = maxS1 - minS2;
 		float diff2 = maxS2 - minS1;
		
		//no collision
		if (diff1 <= 0.f || diff2 <= 0.f)
			return false;

		//collision, get the minimum overlap with the axis
		if (diff1 < _overlap)
		{
			_overlap = diff1;
			_separatingAxis = _axis;
		}
		if (diff2 < _overlap)
		{
			_overlap = diff2;
			_separatingAxis = _axis * -1;
		}
		return true;
	}
}