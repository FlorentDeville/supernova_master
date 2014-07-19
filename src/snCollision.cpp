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
#include "snEPA.h"
#include "snEPASimplex.h"

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
		const snOBB* _b1 = static_cast<const snOBB*>(_c1);
		const snOBB* _b2 = static_cast<const snOBB*>(_c2);
		bool resSAT = snSAT::queryIntersection<snOBB, snOBB>(*_b1, *_b2, res.m_normal);

#ifdef SANITY_GJK
		bool resGJK = snGJK::gjkIntersect<snOBB, snOBB>(*_b1, *_b2);

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

	snCollisionResult snCollision::queryTestCollisionGJK(const snICollider* const _c1, const snICollider* const _c2)
	{
		snCollisionResult res;
		res.m_collision = false;

		snVec gjkSimplex[4];
		unsigned int simplexSize = 0;
		if (!snGJK::gjkIntersect(*_c1, *_c2, gjkSimplex, simplexSize))
			return res;

		//Add points to the simplex
		snEPA epa;
		snSimplex simplex;
		/*for (unsigned int i = 0; i < simplexSize; ++i)
			simplex.addVertex(gjkSimplex[i]);*/
		
		if (simplexSize == 1) //This is just a contact point, ignore it.
		{
			return res;
		}
		else if (simplexSize == 2) //A line where the origin lies
		{
			// The simplex returned by GJK is a line segment d containing the origin.
			// We add two additional support points to construct a hexahedron (two tetrahedron
			// glued together with triangle faces. The idea is to compute three different vectors
			// v1, v2 and v3 that are orthogonal to the segment d. The three vectors are relatively
			// rotated of 120 degree around the d segment. The the three new points to
			// construct the polytope are the three support points in those three directions
			// v1, v2 and v3.

			// Direction of the segment
			snVec d = gjkSimplex[1] - gjkSimplex[0];
			snVec3Normalize(d);

			// Choose the coordinate axis from the minimal absolute component of the vector d
			unsigned int minAxis = snVec3GetMinAxis(d);

			// Compute sin(60)
			const double sin60 = sqrt(3.0) * 0.5;

			// Create a rotation quaternion to rotate the vector v1 to get the vectors
			// v2 and v3
			snVec rotationQuat = d * sin60;// snVec4Set(d.getX() * sin60, d.getY() * sin60, d.getZ() * sin60, 0.5);
			snVec4SetW(rotationQuat, 0.5f);

			// Construct the corresponding rotation matrix
			snMatrix44f rotationMat;// = rotationQuat.getMatrix();
			rotationMat.createRotationFromQuaternion(rotationQuat);

			// Compute the vector v1, v2, v3
			snVec v1 = snVec3Cross(d, snVec4Set(minAxis == 0, minAxis == 1, minAxis == 2, 0));// d.cross(Vector3D(minAxis == 0, minAxis == 1, minAxis == 2));
			snVec v2 = snMatrixTransform3(v1, rotationMat);// rotationMat * v1;
			snVec v3 = snMatrixTransform3(v2, rotationMat);// rotationMat * v2;
			snVec3Normalize(v1);
			snVec3Normalize(v2);
			snVec3Normalize(v3);
			// Compute the support point in the direction of v1
			
			/*suppPointsA[2] = boundingVolume1->getSupportPoint(v1, OBJECT_MARGIN);
			suppPointsB[2] = boundingVolume2->getSupportPoint(v1.getOpposite(), OBJECT_MARGIN);
			points[2] = suppPointsA[2] - suppPointsB[2];*/
			snVec newPoint2 = _c1->support(v1) - _c2->support(-v1);

			// Compute the support point in the direction of v2
			/*suppPointsA[3] = boundingVolume1->getSupportPoint(v2, OBJECT_MARGIN);
			suppPointsB[3] = boundingVolume2->getSupportPoint(v2.getOpposite(), OBJECT_MARGIN);
			points[3] = suppPointsA[3] - suppPointsB[3];*/
			snVec newPoint3 = _c1->support(v2) - _c2->support(-v2);

			// Compute the support point in the direction of v3
	/*		suppPointsA[4] = boundingVolume1->getSupportPoint(v3, OBJECT_MARGIN);
			suppPointsB[4] = boundingVolume2->getSupportPoint(v3.getOpposite(), OBJECT_MARGIN);
			points[4] = suppPointsA[4] - suppPointsB[4];*/
			snVec newPoint4 = _c1->support(v3) - _c2->support(-v3);

			// Now we have an hexahedron (two tetrahedron glued together). We can simply keep the
			// tetrahedron that contains the origin in order that the initial polytope of the
			// EPA algorithm is a tetrahedron, which is simpler to deal with.

			// If the origin is in the tetrahedron of points 0, 2, 3, 4
			//if (isOriginInTetrahedron(points[0], points[2], points[3], points[4]) == 0) 
			unsigned int wrongId = 0;
			if (epa.isValidStartSimplex(gjkSimplex[0], newPoint2, newPoint3, newPoint4, wrongId))
			{
				// We use the point 4 instead of point 1 for the initial tetrahedron
				simplex.addVertex(gjkSimplex[0]);
				simplex.addVertex(newPoint4);
				simplex.addVertex(newPoint2);
				simplex.addVertex(newPoint3);
				/*suppPointsA[1] = suppPointsA[4];
				suppPointsB[1] = suppPointsB[4];
				points[1] = points[4];*/
			}
			//else if (isOriginInTetrahedron(points[1], points[2], points[3], points[4]) == 0) 
			else if (epa.isValidStartSimplex(gjkSimplex[1], newPoint2, newPoint3, newPoint4, wrongId))
			{  // If the origin is in the tetrahedron of points 1, 2, 3, 4
				// We use the point 4 instead of point 0 for the initial tetrahedron
				simplex.addVertex(newPoint4);
				simplex.addVertex(gjkSimplex[1]);
				simplex.addVertex(newPoint2);
				simplex.addVertex(newPoint3);

				/*suppPointsA[0] = suppPointsA[0];
				suppPointsB[0] = suppPointsB[0];
				points[0] = points[0];*/
			}
			else 
			{
				// The origin is not in the initial polytope
				return res;
			}

			snEPATriangle* t0 = simplex.addTriangle(0, 2, 1);
			snEPATriangle* t1 = simplex.addTriangle(0, 1, 3);
			snEPATriangle* t2 = simplex.addTriangle(1, 2, 3);
			snEPATriangle* t3 = simplex.addTriangle(2, 0, 3);

			simplex.addLink(t0, 0, t3, 0);
			simplex.addLink(t0, 1, t2, 0);
			simplex.addLink(t0, 2, t1, 0);
			simplex.addLink(t1, 1, t2, 2);
			simplex.addLink(t2, 1, t3, 2);
			simplex.addLink(t3, 1, t1, 2);
			// The polytope contains now 4 vertices
			//nbVertices = 4;

		}
		else if (simplexSize == 3)
		{

			//Compute the triangle normal and find 2 points on both sides of the triangle.
			//Create a simplex from that.

			// Compute the normal of the triangle
			snVec v1 = gjkSimplex[1] - gjkSimplex[0];
			snVec v2 = gjkSimplex[2] - gjkSimplex[0];
			snVec n = snVec3Cross(v1, v2);
			snVec3Normalize(n);

			for (unsigned int i = 0; i < simplexSize; ++i)
				simplex.addVertex(gjkSimplex[i]);

			// Compute the two new vertices to obtain a hexahedron
			int id4 = simplex.addVertex(_c1->support(n) - _c2->support(-n));
			int id5 = simplex.addVertex(_c1->support(-n) - _c2->support(n));

			//Create the triangles
			snEPATriangle* t0 = simplex.addTriangle(0, 1, id4);
			snEPATriangle* t1 = simplex.addTriangle(0, id4, 2);
			snEPATriangle* t2 = simplex.addTriangle(1, 2, id4);

			snEPATriangle* t3 = simplex.addTriangle(1, 0, id5);
			snEPATriangle* t4 = simplex.addTriangle(id5, 0, 2);
			snEPATriangle* t5 = simplex.addTriangle(1, id5, 2);

			//Create the links
			simplex.addLink(t0, 1, t2, 2);
			simplex.addLink(t0, 2, t1, 0);
			simplex.addLink(t1, 1, t2, 1);

			simplex.addLink(t3, 2, t5, 0);
			simplex.addLink(t5, 1, t4, 2);
			simplex.addLink(t4, 0, t3, 1);

			simplex.addLink(t2, 0, t5, 2);
			simplex.addLink(t1, 2, t4, 1);
			simplex.addLink(t0, 0, t3, 0);
		}
		else if (simplexSize == 4)
		{
			for (unsigned int i = 0; i < simplexSize; ++i)
				simplex.addVertex(gjkSimplex[i]);

			snEPATriangle* t0 = simplex.addTriangle(0, 2, 1);
			snEPATriangle* t1 = simplex.addTriangle(0, 1, 3);
			snEPATriangle* t2 = simplex.addTriangle(1, 2, 3);
			snEPATriangle* t3 = simplex.addTriangle(2, 0, 3);

			simplex.addLink(t0, 0, t3, 0);
			simplex.addLink(t0, 1, t2, 0);
			simplex.addLink(t0, 2, t1, 0);
			simplex.addLink(t1, 1, t2, 2);
			simplex.addLink(t2, 1, t3, 2);
			simplex.addLink(t3, 1, t1, 2);
		}
		else
		{
			throw; // simplex size not handled
		}
		
		float depth = 0;
		if (!epa.execute(simplex, *_c1, *_c2, res.m_normal, depth))
			return res;

		snFeatureClipping clipping;
		res.m_collision = clipping.findContactPatch(*_c1, *_c2, res.m_normal, res.m_contacts, res.m_penetrations);

		return res;
	}


}