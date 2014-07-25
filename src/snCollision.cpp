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

#ifdef _DEBUG
	//#define SANITY_GJK
	#include "snLogger.h"
#endif //ifdef _DEBUG

#include "snCollision.h"
#include "snOBB.h"
#include "snCapsule.h"
#include "snSphere.h"
#include "snHeightMap.h"

#include "snCollisionResult.h"
#include "snColliderContainer.h"
#include "snICollider.h"
#include "snIActor.h"
#include "snMath.h"
#include "snFeatureClipping.h"
#include <assert.h>

#include "snSAT.h"
#include "snGJK.h"
#include "snEPA.h"
#include "snEPASimplex.h"

#include "snClosestPoint.h"

using namespace Supernova::Vector;

namespace Supernova
{
	snCollision::snCollision()
	{
		//initialize the collision query map
		m_collisionQueryMap.insert(snCollisionQueryMapElement(SN_COLLISION_KEY(snEColliderBox, snEColliderBox), &Supernova::snCollision::queryTestCollisionOBBVersusOBB));
		m_collisionQueryMap.insert(snCollisionQueryMapElement(SN_COLLISION_KEY(snEColliderSphere, snEColliderSphere), &Supernova::snCollision::queryTestCollisionSphereVersusSphere));
		m_collisionQueryMap.insert(snCollisionQueryMapElement(SN_COLLISION_KEY(snEColliderBox, snEColliderSphere), &Supernova::snCollision::queryTestCollisionBoxVersusSphere));
		m_collisionQueryMap.insert(snCollisionQueryMapElement(SN_COLLISION_KEY(snEColliderCapsule, snEColliderSphere), &Supernova::snCollision::queryTestCollisionCapsuleVersusSphere));
		m_collisionQueryMap.insert(snCollisionQueryMapElement(SN_COLLISION_KEY(snEColliderSphere, snEColliderHeightMap), &Supernova::snCollision::queryTestCollisionSphereVersusHeightMap));
		m_collisionQueryMap.insert(snCollisionQueryMapElement(SN_COLLISION_KEY(snEColliderBox, snEColliderHeightMap), &Supernova::snCollision::queryTestCollisionOBBVersusHeightMap));
		m_collisionQueryMap.insert(snCollisionQueryMapElement(SN_COLLISION_KEY(snEColliderCapsule, snEColliderHeightMap), &Supernova::snCollision::queryTestCollisionCapsuleVersusHeightMap));
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
		//first look for a specific algorithm in the jump table
		unsigned short key = SN_COLLISION_KEY(_c1->getTypeOfCollider(), _c2->getTypeOfCollider());
		snCollisionQueryMap::const_iterator i = m_collisionQueryMap.find(key);
		
		if (i != m_collisionQueryMap.cend())
		{
			snQueryTestCollisionFunction func = i->second;
			return func(_c1, _c2);
		}

		key = SN_COLLISION_KEY(_c2->getTypeOfCollider(), _c1->getTypeOfCollider());
		i = m_collisionQueryMap.find(key);
		if (i != m_collisionQueryMap.cend())
		{
			snQueryTestCollisionFunction func = i->second;
			snCollisionResult res = func(_c2, _c1);
			res.m_normal = -res.m_normal;
			return res;
		}

		//No specific algorithm found, use the default GJK.
		return queryTestCollisionGJK(_c1, _c2);
			
	}

	snCollisionResult snCollision::queryTestCollisionOBBVersusOBB(const snICollider* const _c1, const snICollider* const _c2)
	{
		snCollisionResult res;

		snSAT sat;
		bool resSAT = sat.queryIntersection(*_c1, *_c2, res.m_normal);

#ifdef SANITY_GJK
		const snOBB* _b1 = static_cast<const snOBB*>(_c1);
		const snOBB* _b2 = static_cast<const snOBB*>(_c2);
		snVec simplex[4];
		unsigned int simplexSize;
		bool resGJK = snGJK::gjkIntersect(*_b1, *_b2, simplex, simplexSize);

		if (resSAT != resGJK)
		{
			snVec normals[3];
			LOGGER->logWarn("");
			LOGGER->logWarn("A difference was detected between SAT and GJK.");
			LOGGER->logWarn("START DUMP");
			LOGGER->logWarn(" - OBB 1 :");
			LOGGER->logWarn("      pos = " + LOGGER->toString(_b1->getPosition()));
			LOGGER->logWarn("      extends = " + LOGGER->toString(_b1->getExtends()));
			_b1->getUniqueNormals(normals, 3);
			LOGGER->logWarn("      nx = " + LOGGER->toString(normals[0]));
			LOGGER->logWarn("      ny = " + LOGGER->toString(normals[1]));
			LOGGER->logWarn("      nz = " + LOGGER->toString(normals[2]));

			LOGGER->logWarn(" - OBB 2 :");
			LOGGER->logWarn("      pos = " + LOGGER->toString(_b2->getPosition()));
			LOGGER->logWarn("      extends = " + LOGGER->toString(_b2->getExtends()));
			_b2->getUniqueNormals(normals, 3);
			LOGGER->logWarn("      nx = " + LOGGER->toString(normals[0]));
			LOGGER->logWarn("      ny = " + LOGGER->toString(normals[1]));
			LOGGER->logWarn("      nz = " + LOGGER->toString(normals[2]));
			string value = resSAT ? "true" : "false";
			LOGGER->logWarn("SAT result : " + value);
			value = resGJK ? "true" : "false";
			LOGGER->logWarn("GJK result : " + value);
			LOGGER->logWarn("STOP DUMP");
			LOGGER->logWarn("");

			assert(false);
		}
#endif //ifdef _DEBUG

		if (!resSAT)
			return res;

		//there is a collision so find the collision patch
		res.m_collision = true;

		//find collision patch using clipping algorithm.
		snFeatureClipping clipping;
		bool clippingResult = clipping.findContactPatch(*_c1, *_c2, res.m_normal, res.m_contacts, res.m_penetrations);
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

			res.m_contacts.push_back(_s2->getCenter() + res.m_normal * _s2->getRadius());
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

	snCollisionResult snCollision::queryTestCollisionSphereVersusHeightMap(const snICollider* const _c1, const snICollider* const _c2)
	{
		snCollisionResult res;

		//Cast colliders
		const snSphere* const sphere = static_cast<const snSphere* const>(_c1);
		const snHeightMap* const heightMap = static_cast<const snHeightMap* const>(_c2);

		//find triangles ids overlapping the sphere
		const unsigned int MAX_TRIANGLE_COUNT = 10;
		unsigned int triangleId[MAX_TRIANGLE_COUNT];

		//Get the triangles ids
		snAABB sphereBB;
		sphere->computeAABB(&sphereBB);
		unsigned int triangleCount = heightMap->getOverlapTriangles(sphereBB, triangleId, MAX_TRIANGLE_COUNT);
		if (triangleCount == -1)
			return res;

		float sqRadius = sphere->getRadius() * sphere->getRadius();

		//For each triangle, find the closest point
		for (unsigned int i = 0; i < triangleCount; ++i)
		{
			//get the triangle vertices
			snVec vertices[3];
			heightMap->getTriangle(triangleId[i], vertices);

			//find the closest point of the triangle to the sphere's center
			snVec closestPoint = snClosestPoint::PointTriangle(sphere->getCenter(), vertices[0], vertices[1], vertices[2]);

			//compare distance
			snVec direction = sphere->getCenter() - closestPoint;
			float sqDistance = snVec3SquaredNorme(direction);
			if (sqDistance > sqRadius) // no collision
				continue;

			//Collision
			res.m_contacts.push_back(closestPoint);
			res.m_penetrations.push_back(sqrtf(sqDistance) - sphere->getRadius());
		}

		//no collision found
		if (res.m_contacts.size() == 0)
			return res;

		//compute the normal (temporary)
		snVec normal = snVec4Set(0);
		for (unsigned int i = 0; i < res.m_contacts.size(); ++i)
		{
			snVec dir = sphere->getCenter() - res.m_contacts[i];
			snVec3Normalize(dir);

			normal = normal + dir;
		}
		res.m_normal = normal * (1.f / res.m_contacts.size());
		res.m_collision = true;
		return res;
	}

	snCollisionResult snCollision::queryTestCollisionOBBVersusHeightMap(const snICollider* const _c1, const snICollider* const _c2)
	{
		snCollisionResult res;

		snAABB boundingVolume;
		_c1->computeAABB(&boundingVolume);

		//Get the overlapped triangles
		const snHeightMap* const heightMap = static_cast<const snHeightMap* const>(_c2);
		const int MAX_TRIANGLE = 10;
		unsigned int trianglesId[MAX_TRIANGLE];
		int triangleCount = heightMap->getOverlapTriangles(boundingVolume, trianglesId, MAX_TRIANGLE);

		//no triangle => no collision
		if (triangleCount <= 0)
			return res;

		//loop through each triangles
		unsigned int patchCount = 0;
		for (int i = 0; i < triangleCount; ++i)
		{
			//get the triangle information
			snVec triangleNormal = heightMap->getNormal(trianglesId[i]);

			snVec triangleVertices[3];
			heightMap->getTriangle(trianglesId[i], triangleVertices);

			//get the triangle offset along it's normal
			float d = snVec4GetX(snVec3Dot(triangleNormal, triangleVertices[0]));

			//check if the box overlap along the triangle normal
			float min, max;
			_c1->projectToAxis(triangleNormal, min, max);
			min -= d;
			max -= d;

			//no overlap
			if (min * max > 0)
				continue;

			//compute the patch
			snVec obbFeature[4];
			unsigned int obbFeatureSize;
			unsigned int obbFeatureId;
			_c1->getClosestFeature(-triangleNormal, obbFeature, obbFeatureSize, obbFeatureId);
			snVec obbFeatureNormal = _c1->getFeatureNormal(obbFeatureId);

			snFeatureClipping clipping;
			snVecVector patch;
			std::vector<float> penetration;
			if (!clipping.findContactPatch(obbFeature, obbFeatureSize, obbFeatureNormal, triangleVertices, 3, triangleNormal, triangleNormal, patch, penetration))
				continue;

			//save result
			++patchCount;
			res.m_collision = true;
			res.m_normal = res.m_normal + triangleNormal;
			//reserve enough space
			res.m_contacts.reserve(res.m_contacts.size() + patch.size());
			res.m_penetrations.reserve(res.m_penetrations.size() + penetration.size());

			for (unsigned int j = 0; j < patch.size(); ++j)
			{
				//check if the points we try to add already exists
				bool add = true;
				for (unsigned int k = 0; k < res.m_contacts.size(); ++k)
				{
					float delta = snVec3SquaredNorme(patch[j] - res.m_contacts[k]);
					const float EPSILON = 0.01f;
					if (delta <= EPSILON)
					{
						add = false;
						break;
					}
				}

				if (add)
				{
					res.m_contacts.push_back(patch[j]);
					res.m_penetrations.push_back(penetration[j]);
				}
				
			}
		}

		if (!res.m_collision)
			return res;

		res.m_normal = res.m_normal * (1.f / patchCount);
		return res;

	}

	snCollisionResult snCollision::queryTestCollisionCapsuleVersusHeightMap(const snICollider* const _c1, const snICollider* const _c2)
	{
		snCollisionResult res;
		res.m_normal = snVec4Set(0, 0, 0, 0);

		//Cast colliders
		const snCapsule* const capsule = static_cast<const snCapsule* const>(_c1);
		const snHeightMap* const heightMap = static_cast<const snHeightMap* const>(_c2);

		//find triangles ids overlapping the sphere
		const unsigned int MAX_TRIANGLE_COUNT = 10;
		unsigned int triangleId[MAX_TRIANGLE_COUNT];

		//Get the triangles ids
		snAABB capsuleBB;
		capsule->computeAABB(&capsuleBB);
		unsigned int triangleCount = heightMap->getOverlapTriangles(capsuleBB, triangleId, MAX_TRIANGLE_COUNT);
		if (triangleCount == -1)
			return res;

		float sqRadius = capsule->getRadius() * capsule->getRadius();
		unsigned int patchCount = 0;

		//For each triangle, find the closest point
		for (unsigned int i = 0; i < triangleCount; ++i)
		{
			//get the triangle information
			snVec triangle[3];
			heightMap->getTriangle(triangleId[i], triangle);

			//Check if the cylinder overlap the triangle on the Y axis
			float minY;
			float maxY;
			minY = snVec4GetY(triangle[0]);
			maxY = snVec4GetY(triangle[0]);
			for (unsigned int t = 1; t < 3; ++t)
			{
				float y = snVec4GetY(triangle[t]);
				if (y < minY) minY = y;
				if (y > maxY) maxY = y;
			}

			if (snVec4GetY(capsuleBB.m_max) <= minY || snVec4GetY(capsuleBB.m_min) >= maxY)
				continue;

			//store results
			snVecVector patch;
			std::vector<float> penetration;
			snVec collisionNormal = heightMap->getNormal(triangleId[i]);

			//Look for the closest point between the first end point and the triangle
			snVec closestPointOnTriangle = snClosestPoint::PointTriangle(capsule->getFirstEndPoint(), triangle[0], triangle[1], triangle[2]);

			//Look for the closest point to the line segment
			snVec closestPointOnCapsuleAxis = snClosestPoint::PointLineSegment(closestPointOnTriangle, capsule->getFirstEndPoint(), capsule->getSecondEndPoint());

			//Compute the length between those two points
			snVec diff = closestPointOnCapsuleAxis - closestPointOnTriangle;
			bool collisionFound = false;
			if (snVec3SquaredNorme(diff) < sqRadius) //Collision!!!
			{
				patch.push_back(closestPointOnTriangle);
				float length = snVec3Norme(diff);
				penetration.push_back(capsule->getRadius() - length);
				collisionFound = true;
			}

			//Look for the closest pont between the second end point and the triangle
			closestPointOnTriangle = snClosestPoint::PointTriangle(capsule->getSecondEndPoint(), triangle[0], triangle[1], triangle[2]);

			//Look for the closest point to the line segment
			closestPointOnCapsuleAxis = snClosestPoint::PointLineSegment(closestPointOnTriangle, capsule->getFirstEndPoint(), capsule->getSecondEndPoint());

			//Compute the length between those two points
			diff = closestPointOnCapsuleAxis - closestPointOnTriangle;
			if (snVec3SquaredNorme(diff) < sqRadius) //Collision!!!
			{
				patch.push_back(closestPointOnTriangle);
				float length = snVec3Norme(diff);
				penetration.push_back(capsule->getRadius() - length);

				collisionFound = true;
			}

			if (!collisionFound)
				continue;

			//save result
			++patchCount;
			res.m_collision = true;
			res.m_normal = res.m_normal + collisionNormal;

			//reserve enough space
			res.m_contacts.reserve(res.m_contacts.size() + patch.size());
			res.m_penetrations.reserve(res.m_penetrations.size() + penetration.size());

			for (unsigned int j = 0; j < patch.size(); ++j)
			{
				//check if the points we try to add already exists
				bool add = true;
				for (unsigned int k = 0; k < res.m_contacts.size(); ++k)
				{
					float delta = snVec3SquaredNorme(patch[j] - res.m_contacts[k]);
					const float EPSILON = 0.01f;
					if (delta <= EPSILON)
					{
						add = false;
						break;
					}
				}

				if (add)
				{
					res.m_contacts.push_back(patch[j]);
					res.m_penetrations.push_back(penetration[j]);
				}

			}
		}

		if (!res.m_collision)
		{
			return res;
		}

		res.m_normal = res.m_normal * (1.f / patchCount);

		return res;
	}

	snCollisionResult snCollision::queryTestCollisionGJK(const snICollider* const _c1, const snICollider* const _c2)
	{
		snCollisionResult res;
		res.m_collision = false;

		//GJK to check for collision
		snVec gjkSimplex[4];
		unsigned int simplexSize = 0;
		if (!snGJK::gjkIntersect(*_c1, *_c2, gjkSimplex, simplexSize))
			return res;

		//EPa to get the collision normal
		snEPA epa;
		snSimplex simplex;

		if (!epa.prepareSimplex(gjkSimplex, simplexSize, *_c1, *_c2, simplex))
			return res;

		if (!epa.execute(simplex, *_c1, *_c2, res.m_normal))
			return res;

		//Feature clipping to get the contact patch
		snFeatureClipping clipping;
		res.m_collision = clipping.findContactPatch(*_c1, *_c2, res.m_normal, res.m_contacts, res.m_penetrations);

		return res;
	}


}