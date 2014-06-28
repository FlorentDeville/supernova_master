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
#include "snOBB.h"
#include "snCapsule.h"
#include "snSphere.h"
#include "snColliderPlan.h"

#include "snCollisionResult.h"
#include "snColliderContainer.h"
#include "snICollider.h"
#include "snIActor.h"
#include "snMath.h"
#include "snFeatureClipping.h"
#include <assert.h>

#include "snSAT.h"
#include "snGJK.h"

using namespace Supernova::Vector;

namespace Supernova
{
	snCollision::snCollision()
	{
		//initialize the collision query map
		m_collisionQueryMap.insert(snCollisionQueryMapElement(SN_COLLISION_KEY(snEColliderBox, snEColliderBox), &Supernova::snCollision::queryTestCollisionOBBVersusOBB));
		m_collisionQueryMap.insert(snCollisionQueryMapElement(SN_COLLISION_KEY(snEColliderSphere, snEColliderSphere), &Supernova::snCollision::queryTestCollisionSphereVersusSphere));
		m_collisionQueryMap.insert(snCollisionQueryMapElement(SN_COLLISION_KEY(snEColliderBox, snEColliderSphere), &Supernova::snCollision::queryTestCollisionBoxVersusSphere));
		m_collisionQueryMap.insert(snCollisionQueryMapElement(SN_COLLISION_KEY(snEColliderBox, snEColliderPlan), &Supernova::snCollision::queryTestCollisionBoxVersusPlan));
		m_collisionQueryMap.insert(snCollisionQueryMapElement(SN_COLLISION_KEY(snEColliderSphere, snEColliderPlan), &Supernova::snCollision::queryTestCollisionSphereVersusPlan));
		m_collisionQueryMap.insert(snCollisionQueryMapElement(SN_COLLISION_KEY(snEColliderCapsule, snEColliderSphere), &Supernova::snCollision::queryTestCollisionCapsuleVersusSphere));
		m_collisionQueryMap.insert(snCollisionQueryMapElement(SN_COLLISION_KEY(snEColliderCapsule, snEColliderBox), &Supernova::snCollision::queryTestCollisionCapsuleVersusOBB));
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

	bool snCollision::queryTestCollision(const snICollider* const _c1, const snICollider* const _c2) const
	{
		snCollisionResult res = invokeQueryTestCollision(_c1, _c2);
		return res.m_collision;
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

	snCollisionResult snCollision::queryTestCollisionOBBVersusOBB(const snICollider* const _c1, const snICollider* const _c2)
	{
		snCollisionResult res;
		const snOBB* _b1 = static_cast<const snOBB*>(_c1);
		const snOBB* _b2 = static_cast<const snOBB*>(_c2);
		if (!snSAT::queryIntersection<snOBB, snOBB>(*_b1, *_b2, res.m_normal))
			return res;

		//there is a collision so find the collision patch
		res.m_collision = true;

		//find collision patch using clipping algorithm.
		snFeatureClipping clipping;
		bool clippingResult = clipping.findContactPatch(*_b1, *_b2, res.m_normal, res.m_contacts, res.m_penetrations);
		res.m_collision = clippingResult;
		return res;
	}

	snCollisionResult snCollision::queryTestCollisionSphereVersusSphere(const snICollider* const _c1, const snICollider* const _c2)
	{
		const snSphere* _s1 = static_cast<const snSphere*>(_c1);
		const snSphere* _s2 = static_cast<const snSphere*>(_c2);

		snCollisionResult res;

		//Get the vector between the sphere's origins.
		snVec vecDistance = _s1->getCenter() - _s2->getCenter();

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
		const snOBB* _box = static_cast<const snOBB*>(_c1);
		const snSphere* _sphere = static_cast<const snSphere*>(_c2);

		snCollisionResult res;

		//Get the closest point to the box.
		snVec closestPoint = _box->getClosestPoint(_sphere->getCenter());

		//Get the distance (squared) between the closest point and the sphere center.
		snVec closestPointToSphere = closestPoint - _sphere->getCenter();
		float squaredDistanceClosestPoint = snVec3SquaredNorme(closestPointToSphere);
		float squaredRadius = _sphere->getRadius() * _sphere->getRadius();

		//If the closest point distance is bigger than the radius then no collision (all distances are squared).
		if (squaredDistanceClosestPoint >= squaredRadius)
			return res;

		res.m_collision = true;
		res.m_normal = closestPointToSphere;
		snVec3Normalize(res.m_normal);
		snVec4SetW(res.m_normal, 0);
		snVec contactPoint = _sphere->getCenter() + res.m_normal * _sphere->getRadius();
		res.m_contacts.push_back(contactPoint);
		res.m_penetrations.push_back(snVec3Norme(contactPoint - closestPoint));
		return res;
	}

	snCollisionResult snCollision::queryTestCollisionBoxVersusPlan(const snICollider* const _c1, const snICollider* const /*_c2*/)
	{
		const snOBB* _box = static_cast<const snOBB*>(_c1);
		const snColliderPlan* _plan = static_cast<const snColliderPlan*>(_c1);

		//get the distance between the box center and the plan
		float boxDistance = fabsf(_plan->getDistance(_box->getPosition()));
		
		//get the extends
		snVec extends = _box->getExtends();

		//get the box normals
		const int OOB_NORMAL_COUNT = 3;
		snVec s1Normals[OOB_NORMAL_COUNT];
		_box->getUniqueNormals(s1Normals, OOB_NORMAL_COUNT);

		//compute the minimum distance between the box and the plan
		snVec dot0 = snVec4GetAbsolute(snVec3Dot(s1Normals[0], _plan->getWorldNormal()));
		snVec dot1 = snVec4GetAbsolute(snVec3Dot(s1Normals[1], _plan->getWorldNormal()));
		snVec dot2 = snVec4GetAbsolute(snVec3Dot(s1Normals[2], _plan->getWorldNormal()));

		snVec dot = _mm_shuffle_ps(dot0, dot1, _MM_SHUFFLE(3, 2, 1, 0));
		dot = _mm_shuffle_ps(dot, dot2, _MM_SHUFFLE(3, 2, 1, 0));
		snVec minDistance = dot * extends;

		/*snVec minDistance = snVec4GetX(extends) * snVec4GetAbsolute(snVec3Dot(s1Normals[0], _plan->getWorldNormal())) +
			snVec4GetY(extends) * snVec4GetAbsolute(snVec3Dot(s1Normals[1], _plan->getWorldNormal())) +
			snVec4GetZ(extends) * snVec4GetAbsolute(snVec3Dot(s1Normals[2], _plan->getWorldNormal()));*/
		
		//compare the real distance to the min distance
		float overlap = boxDistance - snVec4GetX(minDistance);

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
		const snSphere* _sphere = static_cast<const snSphere*>(_c1);
		const snColliderPlan* _plan = static_cast<const snColliderPlan*>(_c1);

		snCollisionResult res;

		//get the distance between the sphere center and the plan
		float distance = _plan->getDistance(_sphere->getCenter());

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

	snCollisionResult snCollision::queryTestCollisionCapsuleVersusSphere(const snICollider* const _c1, const snICollider* const _c2)
	{
		const snCapsule* _capsule = static_cast<const snCapsule*>(_c1);
		const snSphere* _sphere = static_cast<const snSphere*>(_c2);
	
		//Get the closest point on the capsule
		snVec center = _sphere->getCenter() - _capsule->getSecondEndPoint(); //vector from the second endpoint to the sphere's center
		snVec dir = _capsule->getFirstEndPoint() - _capsule->getSecondEndPoint(); //vector from the second endpoint to the first endpoint.
		float length = snVec3Norme(dir); //compute the distance between the two endpoints !!!!!!!!!!!!!COULD BE CACHED
		snVec3Normalize(dir); //normalized the direction

		//Compute the distance of the closest point along the capsule axis and clamp it.
		snVec dot = snVec3Dot(dir, center);
		snVec closestPointLength = clampComponents(dot, 0, length);

		//Closest point is the closest point to the sphere on the capsule's axis.
		snVec closestPoint = _capsule->getSecondEndPoint() + (dir * closestPointLength);

		//Check the distance between the sphere's center and the closest point
		float sqDistance = snVec3SquaredNorme(closestPoint - _sphere->getCenter());
		float sqMinDistance = _sphere->getRadius() + _capsule->getRadius();
		sqMinDistance *= sqMinDistance;

		snCollisionResult res;
		if (sqDistance <= sqMinDistance)
		{
			res.m_collision = true;
			res.m_normal = _sphere->getCenter() - closestPoint;
			snVec3Normalize(res.m_normal);
			res.m_contacts.push_back(closestPoint + res.m_normal * _capsule->getRadius());
			res.m_penetrations.push_back(sqrt(sqMinDistance) - sqrt(sqDistance)); //TODO : figure out how to avoid two sqrt!
		}
		

		return res;
	}

	snCollisionResult snCollision::queryTestCollisionCapsuleVersusOBB(const snICollider* const _c1, const snICollider* const _c2)
	{
		const snCapsule* _capsule = static_cast<const snCapsule*>(_c1);
		const snOBB* _obb = static_cast<const snOBB*>(_c2);

		snCollisionResult res;
		res.m_collision = snGJK::GJKIntersect<snCapsule, snOBB>(*_capsule, *_obb);
		return res;
	}
}