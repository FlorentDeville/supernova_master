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

#include "snICollider.h"
#include "snActor.h"

#include "snMath.h"
#include "snFeatureClipping.h"
#include <assert.h>

namespace Supernova
{
	snCollision::snCollision()
	{
		//initialize the collision query map
		m_collisionQueryMap.insert(snCollisionQueryMapElement(SN_COLLISION_KEY(snEColliderBox, snEColliderBox), &Supernova::snCollision::queryTestCollisionBoxVersusBox_V2));
		m_collisionQueryMap.insert(snCollisionQueryMapElement(SN_COLLISION_KEY(snEColliderSphere, snEColliderSphere), &Supernova::snCollision::queryTestCollisionSphereVersusSphere));
		m_collisionQueryMap.insert(snCollisionQueryMapElement(SN_COLLISION_KEY(snEColliderBox, snEColliderSphere), &Supernova::snCollision::queryTestCollisionBoxVersusSphere));
		m_collisionQueryMap.insert(snCollisionQueryMapElement(SN_COLLISION_KEY(snEColliderBox, snEColliderPlan), &Supernova::snCollision::queryTestCollisionBoxVersusPlan));
		m_collisionQueryMap.insert(snCollisionQueryMapElement(SN_COLLISION_KEY(snEColliderSphere, snEColliderPlan), &Supernova::snCollision::queryTestCollisionSphereVersusPlan));
	}

	snCollision::~snCollision(){}

	snCollisionResult snCollision::queryTestCollision(snActor* _a1, snActor* _a2) const
	{
		std::vector<snICollider*>& c1 = _a1->getColliders();
		std::vector<snICollider*>& c2 = _a2->getColliders();

		return invokeQueryTestCollision(c1[0], _a1->getPosition(), _a1->getInverseOrientationMatrix(), c2[0], _a2->getPosition(), _a2->getInverseOrientationMatrix());
	}

	snCollisionResult snCollision::invokeQueryTestCollision(const snICollider* const _c1, const snVector4f& _p1, const snMatrix44f& _invR1,
		const snICollider* const _c2, const snVector4f& _p2, const snMatrix44f& _invR2) const
	{
		unsigned short key = SN_COLLISION_KEY(_c1->getTypeOfCollider(), _c2->getTypeOfCollider());
		snCollisionQueryMap::const_iterator i = m_collisionQueryMap.find(key);
		
		if (i != m_collisionQueryMap.cend())
		{
			snQueryTestCollisionFunction func = i->second;
			return func(_c1, _p1, _invR1, _c2, _p2, _invR2);
		}
		else
		{
			key = SN_COLLISION_KEY(_c2->getTypeOfCollider(), _c1->getTypeOfCollider());
			i = m_collisionQueryMap.find(key);
			snQueryTestCollisionFunction func = i->second;
			return func(_c1, _p1, _invR1, _c2, _p2, _invR2);
		}
			
	}

	snCollisionResult snCollision::queryTestCollisionBoxVersusBox(const snICollider* const _c1, const snVector4f& _p1, const snMatrix44f& _invR1,
		const snICollider* const _c2, const snVector4f& _p2, const snMatrix44f& _invR2)
	{
		const snColliderBox* _b1 = static_cast<const snColliderBox*>(_c1);
		const snColliderBox* _b2 = static_cast<const snColliderBox*>(_c2);

		snCollisionResult res;

		const snVector4f* s1Normals;
		const snVector4f* s2Normals;

		s1Normals = _b1->getWorldNormal();
		s2Normals = _b2->getWorldNormal();

		snVector4f smallestOverlapNormal;
		float smallestOverlap = SN_FLOAT_MAX;		

		//compute collider overlap using the normals
		const float NORMAL_COUNT = 3;
		for (int i = 0; i < NORMAL_COUNT; ++i)
		{
			bool overlapRes = true;		
			overlapRes = computeOverlap(*_b1, *_b2, s1Normals[i], smallestOverlapNormal, smallestOverlap);
			if (!overlapRes) return res;		
			overlapRes = computeOverlap(*_b1, *_b2, s2Normals[i], smallestOverlapNormal, smallestOverlap);
			if (!overlapRes) return res;		
		}	

		//compute overlap for the cross product of the normals
		for (int i = 0; i < NORMAL_COUNT; ++i)
		{
			for (int j = i; j < NORMAL_COUNT; ++j)
			{
				snVector4f cross = s1Normals[i].cross(s2Normals[j]);
				if (cross.squareNorme() != 1.f) continue;
				bool overlapRes = computeOverlap(*_b1, *_b2, cross, smallestOverlapNormal, smallestOverlap);
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

	snCollisionResult snCollision::queryTestCollisionBoxVersusBox_V2(const snICollider* const _c1, const snVector4f& _p1, const snMatrix44f& _invR1,
		const snICollider* const _c2, const snVector4f& _p2, const snMatrix44f& _invR2)
	{
		const snColliderBox* _b1 = static_cast<const snColliderBox*>(_c1);
		const snColliderBox* _b2 = static_cast<const snColliderBox*>(_c2);
		const snVector4f* s1Normals = _b1->getWorldNormal();
		const snVector4f* s2Normals = _b2->getWorldNormal();
		snVector4f smallestOverlapNormal;
		float smallestOverlap = SN_FLOAT_MAX;

		snCollisionResult res;

		snVector4f ea = _b1->getSize() * 0.5f;
		snVector4f eb = _b2->getSize() * 0.5f;

		//compute rotation matrix expressing b in a's coordinate frame.
		snMatrix44f R;
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				R[i][j] = s1Normals[i].dot(s2Normals[j]);
			}
		}

		//compute translation vector t
		snVector4f ds = _p2 - _p1;
		//Bring translation into a's coordinate frame
		snVector4f t(ds.dot(s1Normals[0]), ds.dot(s1Normals[1]), ds.dot(s1Normals[2]), 0);

		//compute common subexpressions
		snMatrix44f absR;
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				absR[i][j] = fabs(R[i][j]) + 0.0001f;
			}
		}


		for (int i = 0; i < 3; ++i)
		{
			//test axis A0, A1, A2
			float ra = ea[i];
			float rb = eb.dot(absR[i]);

			//compute overlap
			float overlap = ra + rb - fabs(t[i]);

			//no overlap, it means the current tested axis is a separate axis so there is no collision.
			if (overlap <= 0.f)
				return res;

			if (smallestOverlap > overlap)
			{
				int s = sign(s1Normals[i].dot(ds));
				smallestOverlapNormal = s1Normals[i] * -s;
				smallestOverlap = overlap;
			}
		}

		//test axis B0, B1, B2
		for (int i = 0; i < 3; ++i)
		{
			snVector4f absRColI(absR[0][i], absR[1][i], absR[2][i], 0);
			float ra = ea.dot(absRColI);
			float rb = eb[i];

			snVector4f RColI(R[0][i], R[1][i], R[2][i], 0);
			float at = t.dot(RColI);

			float overlap = ra + rb - fabs(at);
			if (overlap <= 0.f)
				return res;

			if (smallestOverlap > overlap)
			{
				int s = sign(s2Normals[i].dot(ds));
				smallestOverlapNormal = s2Normals[i] * -s;
				smallestOverlap = overlap;
			}
		}

		//A0 x B0
		snVector4f cross = s1Normals[0].cross(s2Normals[0]);
		if (cross.squareNorme() == 1.f)
		{
			float tDotL = fabsf(t[2] * R[1][0] - t[1] * R[2][0]);
			float ra = ea[1] * absR[2][0] + ea[2] * absR[1][0];
			float rb = eb[1] * absR[0][2] + eb[2] * absR[0][1];

			float overlap = ra + rb - tDotL;
			if (overlap <= 0.f)
				return res;

			if (smallestOverlap > overlap)
			{
				int s = sign(cross.dot(ds));
				smallestOverlapNormal = cross * -s;
				smallestOverlap = overlap;
			}
		}

		//A0 x B1
		cross = s1Normals[0].cross(s2Normals[1]);
		if (cross.squareNorme() == 1.f)
		{
			float tDotL = fabsf(t[2] * R[1][1] - t[1] * R[2][1]);
			float ra = ea[1] * absR[2][1] + ea[2] * absR[1][1];
			float rb = eb[0] * absR[0][2] + eb[2] * absR[0][0];

			float overlap = ra + rb - tDotL;
			if (overlap <= 0.f)
				return res;

			if (smallestOverlap > overlap)
			{
				int s = sign(cross.dot(ds));
				smallestOverlapNormal = cross * -s;
				smallestOverlap = overlap;
			}
		}

		//A0 x B2
		cross = s1Normals[0].cross(s2Normals[2]);
		if (cross.squareNorme() == 1.f)
		{
			float tDotL = fabsf(t[2] * R[1][2] - t[1] * R[2][2]);
			float ra = ea[1] * absR[2][2] + ea[2] * absR[1][2];
			float rb = eb[0] * absR[0][1] + eb[1] * absR[0][0];

			float overlap = ra + rb - tDotL;
			if (overlap <= 0.f)
				return res;

			if (smallestOverlap > overlap)
			{
				int s = sign(cross.dot(ds));
				smallestOverlapNormal = cross * -s;
				smallestOverlap = overlap;
			}
		}

		//A1 x B0
		cross = s1Normals[1].cross(s2Normals[0]);
		if (cross.squareNorme() == 1.f)
		{
			float tDotL = fabsf(t[0] * R[2][0] - t[2] * R[0][0]);
			float ra = ea[0] * absR[2][0] + ea[2] * absR[0][0];
			float rb = eb[1] * absR[1][2] + eb[2] * absR[1][1];

			float overlap = ra + rb - tDotL;
			if (overlap <= 0.f)
				return res;

			if (smallestOverlap > overlap)
			{
				int s = sign(cross.dot(ds));
				smallestOverlapNormal = cross * -s;
				smallestOverlap = overlap;
			}
		}

		//A1 x B1
		cross = s1Normals[1].cross(s2Normals[1]);
		if (cross.squareNorme() == 1.f)
		{
			float tDotL = fabsf(t[0] * R[2][1] - t[2] * R[0][1]);
			float ra = ea[0] * absR[2][1] + ea[2] * absR[0][1];
			float rb = eb[0] * absR[1][2] + eb[2] * absR[1][1];

			float overlap = ra + rb - tDotL;
			if (overlap <= 0.f)
				return res;

			if (smallestOverlap > overlap)
			{
				int s = sign(cross.dot(ds));
				smallestOverlapNormal = cross * -s;
				smallestOverlap = overlap;
			}
		}

		//A1 x B2
		cross = s1Normals[1].cross(s2Normals[2]);
		if (cross.squareNorme() == 1.f)
		{
			float tDotL = fabsf(t[0] * R[2][2] - t[2] * R[0][2]);
			float ra = ea[0] * absR[2][2] + ea[2] * absR[0][2];
			float rb = eb[0] * absR[1][1] + eb[1] * absR[1][0];

			float overlap = ra + rb - tDotL;
			if (overlap <= 0.f)
				return res;

			if (smallestOverlap > overlap)
			{
				int s = sign(cross.dot(ds));
				smallestOverlapNormal = cross * -s;
				smallestOverlap = overlap;
			}
		}

		//A2 x B0
		cross = s1Normals[2].cross(s2Normals[0]);
		if (cross.squareNorme() == 1.f)
		{
			float tDotL = fabsf(t[1] * R[0][0] - t[0] * R[1][0]);
			float ra = ea[0] * absR[1][0] + ea[1] * absR[0][0];
			float rb = eb[1] * absR[2][2] + eb[2] * absR[2][1];

			float overlap = ra + rb - tDotL;
			if (overlap <= 0.f)
				return res;

			if (smallestOverlap > overlap)
			{
				int s = sign(cross.dot(ds));
				smallestOverlapNormal = cross * -s;
				smallestOverlap = overlap;
			}
		}

		//A2 x B1
		cross = s1Normals[2].cross(s2Normals[1]);
		if (cross.squareNorme() == 1.f)
		{
			float tDotL = fabsf(t[1] * R[0][1] - t[0] * R[1][1]);
			float ra = ea[0] * absR[1][1] + ea[1] * absR[0][1];
			float rb = eb[0] * absR[2][2] + eb[2] * absR[2][0];

			float overlap = ra + rb - tDotL;
			if (overlap <= 0.f)
				return res;

			if (smallestOverlap > overlap)
			{
				int s = sign(cross.dot(ds));
				smallestOverlapNormal = cross * -s;
				smallestOverlap = overlap;
			}
		}

		//A2 x B2
		cross = s1Normals[2].cross(s2Normals[2]);
		if (cross.squareNorme() == 1.f)
		{
			float tDotL = fabsf(t[1] * R[0][2] - t[0] * R[1][2]);
			float ra = ea[0] * absR[1][2] + ea[1] * absR[0][2];
			float rb = eb[0] * absR[2][1] + eb[1] * absR[2][0];

			float overlap = ra + rb - tDotL;
			if (overlap <= 0.f)
				return res;

			if (smallestOverlap > overlap)
			{
				int s = sign(cross.dot(ds));
				smallestOverlapNormal = cross * -s;
				smallestOverlap = overlap;
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

	snCollisionResult snCollision::queryTestCollisionSphereVersusSphere(const snICollider* const _c1, const snVector4f& _p1, const snMatrix44f& _invR1,
		const snICollider* const _c2, const snVector4f& _p2, const snMatrix44f& _invR2)
	{
		const snColliderSphere* _s1 = static_cast<const snColliderSphere*>(_c1);
		const snColliderSphere* _s2 = static_cast<const snColliderSphere*>(_c2);

		snCollisionResult res;

		//Get the vector between the sphere's origins.
		snVector4f vecDistance = _s1->getWorldOrigin() - _s2->getWorldOrigin();

		//Calculate the minimum distance between the spheres
		float squaredMinDistance = _s1->getRadius() + _s2->getRadius();
		squaredMinDistance *= squaredMinDistance;

		//If the distance between the centers is inferior to the sum or radius then collision.
		if (squaredMinDistance > vecDistance.squareNorme())
		{
			res.m_collision = true;
			res.m_normal = vecDistance;
			res.m_normal.normalize();
			res.m_normal.VEC4FW = 0;

			res.m_penetrations.push_back(sqrtf(squaredMinDistance) - vecDistance.norme());

		}
		

		return res;
	}

	snCollisionResult snCollision::queryTestCollisionBoxVersusSphere(const snICollider* const _c1, const snVector4f& _p1, const snMatrix44f& _invR1,
		const snICollider* const _c2, const snVector4f& _p2, const snMatrix44f& _invR2)
	{
		const snColliderBox* _box = static_cast<const snColliderBox*>(_c1);
		const snColliderSphere* _sphere = static_cast<const snColliderSphere*>(_c2);

		snCollisionResult res;

		//Get the closest point to the box.
		snVector4f closestPoint = _box->getClosestPoint(_sphere->getWorldOrigin());

		//Get the distance (squared) between the closest point and the sphere center.
		float squaredDistanceClosestPoint = (_sphere->getWorldOrigin() - closestPoint).squareNorme();
		float squaredRadius = _sphere->getRadius() * _sphere->getRadius();

		//If the closest point distance is bigger than the radius then no collision (all distances are squared).
		if (squaredDistanceClosestPoint > squaredRadius)
			return res;

		res.m_collision = true;
		res.m_normal = (_sphere->getWorldOrigin() - closestPoint);
		res.m_normal.normalize();
		res.m_normal.VEC4FW = 0;
		res.m_penetrations.push_back(-sqrtf(squaredDistanceClosestPoint) + sqrtf(squaredRadius));
		res.m_contacts.push_back(_sphere->getWorldOrigin() - res.m_normal * _sphere->getRadius());
		return res;
	}

	snCollisionResult snCollision::queryTestCollisionBoxVersusPlan(const snICollider* const _c1, const snVector4f& _p1, const snMatrix44f& _invR1,
		const snICollider* const _c2, const snVector4f& _p2, const snMatrix44f& _invR2)
	{
		const snColliderBox* _box = static_cast<const snColliderBox*>(_c1);
		const snColliderPlan* _plan = static_cast<const snColliderPlan*>(_c1);

		//get the distance between the box center and the plan
		float boxDistance = fabsf(_plan->getDistance(_box->getWorldOrigin()));
		
		//get the extends
		snVector4f extends = _box->getSize() * 0.5f;

		//get the box normals
		const snVector4f* s1Normals = _box->getWorldNormal();

		//compute the minimum distance between the box and the plan
		float minDistance = extends.VEC4FX * fabsf(s1Normals[0].dot(_plan->getWorldNormal())) +
			extends.VEC4FY * fabsf(s1Normals[1].dot(_plan->getWorldNormal())) +
			extends.VEC4FZ * fabsf(s1Normals[2].dot(_plan->getWorldNormal()));
		
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

	snCollisionResult snCollision::queryTestCollisionSphereVersusPlan(const snICollider* const _c1, const snVector4f& _p1, const snMatrix44f& _invR1,
		const snICollider* const _c2, const snVector4f& _p2, const snMatrix44f& _invR2)
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

	bool snCollision::computeOverlap(const snICollider& _c1, const snICollider& _c2, const snVector4f& _axis, snVector4f& _separatingAxis, float& _overlap)
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