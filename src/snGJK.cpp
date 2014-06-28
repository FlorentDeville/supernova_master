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

#include "snGJK.h"
#include "snVec.h"

#include "snTypes.h"
#include "snMath.h"
#include "snCollisionResult.h"
#include "snSimplex.h"

#include "snIGJKCollider.h"

#include <assert.h>

using namespace Supernova::Vector;

#define SN_SAME_DIRECTION(_v1, _v2) snVec4GetX(snVec3Dot(_v1, _v2)) > 0

namespace Supernova
{
	snGJK::snGJK(){}

	snGJK::~snGJK(){}

	snCollisionResult snGJK::queryIntersection(const snIGJKCollider& _c1, const snIGJKCollider& _c2) const
	{
		snCollisionResult res;
		res.m_collision = false;

		snVec simplex[4];

		//vector of point making the resulting simplex.
		int simplexCount = 0;

		//compute first direction from collider 1 to collider 2
		snVec direction = snVec4Set(1, 0, 0, 0);//_c2.getWorldOrigin() - _c1.getWorldOrigin();

		//compute first point of the simplex
		simplex[simplexCount] = support(_c1, _c2, direction);
		++simplexCount;

		//compute direction in opposite direction
		direction = direction * -1;
		
		//while not over
		bool over = false;
		while (!over)
		{
			//compute another point
			snVec newPoint = support(_c1, _c2, direction);

			//check if we passed the origin
			if (snVec4GetX(snVec3Dot(newPoint, direction)) <= 0)
				return res;

			//add the new point to the simplex
			simplex[simplexCount] = newPoint;
			++simplexCount;

			//check if the simplex enclose the origin.
			over = doSimplex(simplex, simplexCount, direction);
		}

		////create a simplex for the EPA
		//snSimplex epaSimplex;
		//epaSimplex.addVertex(simplex[0]);
		//epaSimplex.addVertex(simplex[1]);
		//epaSimplex.addVertex(simplex[2]);
		//epaSimplex.addVertex(simplex[3]);

		//epaSimplex.addTriangle(0, 1, 2);
		//epaSimplex.addTriangle(0, 3, 1);
		//epaSimplex.addTriangle(1, 3, 2);
		//epaSimplex.addTriangle(0, 2, 3);
		//
		////res.m_normal = expandPolytope(epaSimplex, _c1, _c2);

		//snSimplex epaSimplex2;
		//epaSimplex2.addVertex(simplex[0]);
		//epaSimplex2.addVertex(simplex[1]);
		//epaSimplex2.addVertex(simplex[2]);
		//epaSimplex2.addVertex(simplex[3]);

		//epaSimplex2.addTriangle(0, 1, 2);
		//epaSimplex2.addTriangle(0, 3, 1);
		//epaSimplex2.addTriangle(1, 3, 2);
		//epaSimplex2.addTriangle(0, 2, 3);
		//snVec tempNormal;
		//res.m_collision = expandPolytopeV2(epaSimplex2, _c1, _c2, tempNormal);
		///*if (!(res.m_normal == tempNormal))
		//	int a = 0;*/

		//res.m_normal = tempNormal;
		
		snVec3Normalize(res.m_normal);

		//compute the collision patch
		//res.m_collision = m_clipping.findContactPatch(_c1, _c2, res.m_normal, res.m_contacts, res.m_penetrations);

		return res;
	}

	snVec snGJK::support(const snIGJKCollider& _c1, const snIGJKCollider& _c2, const snVec& _direction) const
	{
		float t;
		snVec p1 = _c1.support(_direction, t);
		snVec p2 = _c2.support(-_direction, t);

		return p1 - p2;
	}

	bool snGJK::doSimplex(snVec* const _simplex, int& _simplexCount, snVec& _direction) const
	{
		switch (_simplexCount)
		{
		case 2:
			return checkOneSimplex(_simplex, _simplexCount, _direction);

		case 3:
			return checkTwoSimplex(_simplex, _simplexCount, _direction);

		case 4:
			return checkThreeSimplex(_simplex, _simplexCount, _direction);

		default:
			return false;
		}
	}

	bool snGJK::checkOneSimplex(snVec* const _s, int& _simplexCount, snVec& _direction) const
	{
		//Three possible cases : 
		// 1) closest to _s[0]
		// 2) closest to _s[1]
		// 3) closest to the line
		//As we come from _s[0], case 1 is impossible. Only case 2 and 3 are possible

		//_simplex[1] is A. It is the last point added.
		//_simplex[0] is B. It is the first point entered in the simplex.
		snVec AB = _s[0] - _s[1];
		snVec AO = VEC_ZERO - _s[1];

		if (SN_SAME_DIRECTION(AB, AO)) //Case 3)
			_direction = snVec3Cross(snVec3Cross(AB, AO), AB);
		else
		{
			//Case 2)
			_direction = AO;
			--_simplexCount;
		}

		return false;
	}

	bool snGJK::checkTwoSimplex(snVec* const _simplex, int& _simplexCount, snVec& _direction) const
	{
		//A = _simplex[2]. The last point entered in the simplex.
		//B = _simplex[1].
		//C = _simplex[0].
		snVec AO = snVec4Set(0, 0, 0, 1) - _simplex[2];

		snVec AB = _simplex[1] - _simplex[2];
		snVec AC = _simplex[0] - _simplex[2];

		snVec ABC = snVec3Cross(AB, AC);

		snVec E2 = snVec3Cross(ABC, AC);

		if (SN_SAME_DIRECTION(E2, AO))
		{
			if (SN_SAME_DIRECTION(AC, AO))
			{
				//keep [C, A]
				_simplex[1] = _simplex[2];
				--_simplexCount;
				_direction = snVec3Cross(snVec3Cross(AC, AO), AC);
			}
			else
			{
				if (SN_SAME_DIRECTION(AB, AO))
				{
					//Keep [B, A]
					_simplex[0] = _simplex[1];
					_simplex[1] = _simplex[2];
					--_simplexCount;
					_direction = snVec3Cross(snVec3Cross(AB, AO), AB);
				}
				else
				{
					//keep [A]
					_simplex[0] = _simplex[2];
					_simplexCount = 1;
					_direction = AO;
				}
			}
		}
		else
		{
			snVec E1 = snVec3Cross(AB, ABC);
			if (SN_SAME_DIRECTION(E1, AO))
			{
				if (SN_SAME_DIRECTION(AB, AO))
				{
					//Keep [B, A]
					_simplex[0] = _simplex[1];
					_simplex[1] = _simplex[2];
					--_simplexCount;
					_direction = snVec3Cross(snVec3Cross(AB, AO), AB);
				}
				else
				{
					//keep [A]
					_simplex[0] = _simplex[2];
					_simplexCount = 1;
					_direction = AO;
				}
			}
			else
			{
				if (SN_SAME_DIRECTION(ABC, AO))
				{
					_direction = ABC;
				}
				else
				{
					/*snVec temp = _simplex[0];
					_simplex[0] = _simplex[1];
					_simplex[1] = temp;*/
					_direction = ABC * -1;
				}
			}
		}

		return false;
	}

	bool snGJK::checkThreeSimplex(snVec* const _simplex, int& _simplexCount, snVec& _direction) const
	{
		//A = _simplex[3]. The last point inserted. B = _simplex[2], C = _simplex[1], D = _simplex[0]
		snVec AO = snVec4Set(0, 0, 0, 1) - _simplex[3];
		snVec AD = _simplex[0] - _simplex[3];
		snVec AC = _simplex[1] - _simplex[3];
		snVec AB = _simplex[2] - _simplex[3];

		snVec ABC = snVec3Cross(AB, AC);
		snVec ACD = snVec3Cross(AC, AD);
		snVec ADB = snVec3Cross(AD, AB);

		//check on what side of the triangle the opposite point is
		int BSideOnACD = sign(snVec4GetX(snVec3Dot(ACD, AB)));
		int CSideOnADB = sign(snVec4GetX(snVec3Dot(ADB, AC)));
		int DSideOnABC = sign(snVec4GetX(snVec3Dot(ABC, AD)));

		//check if the origin is on the same side as a point relative to a triangle
		bool ABSameAsOrigin = sign(snVec4GetX(snVec3Dot(ACD, AO))) == BSideOnACD;
		bool ACSameAsOrigin = sign(snVec4GetX(snVec3Dot(ADB, AO))) == CSideOnADB;
		bool ADSameAsOrigin = sign(snVec4GetX(snVec3Dot(ABC, AO))) == DSideOnABC;

		if (ABSameAsOrigin && ACSameAsOrigin && ADSameAsOrigin) // the origin is inside the tetrahedron
			return true;
		else if (!ABSameAsOrigin) //the point B is not in the direction of the origin
		{
			//remove B and point direction to the other side of ACD
			_simplex[2] = _simplex[3];
			--_simplexCount;
			_direction = ACD * -(float)BSideOnACD;
		}
		else if (!ACSameAsOrigin) //the point C is not in the direction of the origin
		{
			//remove C and point direction to the other side of ADB
			_simplex[1] = _simplex[2];
			_simplex[2] = _simplex[3];
			--_simplexCount;
			_direction = ADB * -(float)CSideOnADB;
		}
		else // !ADSameAsOrigin : the point D is not in the direction of the origin
		{
			//remove D and point direction to the other side of ABC
			_simplex[0] = _simplex[1];
			_simplex[1] = _simplex[2];
			_simplex[2] = _simplex[3];
			--_simplexCount;
			_direction = ABC * -(float)DSideOnABC;
		}

		//check now the triangle.
		return checkTwoSimplex(_simplex, _simplexCount, _direction);
	}

	snVec snGJK::expandPolytope(snSimplex& _simplex, const snIGJKCollider& _c1, const snIGJKCollider& _c2) const
	{
		bool loopOver = false;
		while (!loopOver)
		{
			//find the closest triangle to the origin
			int closestTriangleId = -1;
			snVec normal;
			float distance = 0;
			_simplex.computeTriangleClosestToOrigin(closestTriangleId, normal, distance);

			//expand the polytope
			snVec revNormal = normal *-1;
			snVec newVertex = support(_c1, _c2, revNormal);

			//check if we are closer to the origin
			float newPointDistance = snVec4GetX(snVec3Dot(revNormal, newVertex));

			const float EPA_TOLERANCE = 0.01f;
			if ((newPointDistance - distance) < EPA_TOLERANCE)
				return normal;

			_simplex.expand(newVertex, closestTriangleId);
		}

		return snVec();
	}

	bool snGJK::expandPolytopeV2(snSimplex& _simplex, const snIGJKCollider& _c1, const snIGJKCollider& _c2, snVec& _normal) const
	{
		bool loopOver = false;
		while (!loopOver)
		{
			//find the closest triangle to the origin
			int closestTriangleId = -1;
			snVec triangleNormal;
			float distance = 0;
			_simplex.computeTriangleClosestToOrigin(closestTriangleId, triangleNormal, distance);

			////find the closest point to the origin
			//snVec revNormal;
			//_simplex.computeClosestPointToOriginInTriangle(closestTriangleId, revNormal);
			//distance = revNormal.norme();
			//const float MIN_PENETRATION = 0.0001f;
			//if (distance <= MIN_PENETRATION)
			//{
			//	revNormal = triangleNormal * -1;
			//	/*_normal = triangleNormal;
			//	return true;*/
			//}
			snVec revNormal = triangleNormal * -1;
			//find a new point for the simplex.
			snVec newVertex = support(_c1, _c2, revNormal);

			//check if we are closer to the origin
			float newPointDistance = snVec4GetX(snVec3Dot(revNormal, newVertex));

			snVec triangle[3];
			_simplex.getTriangle(closestTriangleId, triangle[0], triangle[1], triangle[2]);
			const float EPA_TOLERANCE = 0.01f;
			if ((newPointDistance - distance) < EPA_TOLERANCE ||
				newVertex == triangle[0] ||
				newVertex == triangle[1] ||
				newVertex == triangle[2])
			{
				snVec3Normalize(revNormal);
				_normal = revNormal * -1;
				return true;
			}			

			//add the new vertex
			int newVertexId = _simplex.addVertex(newVertex);
			int id0, id1, id2;
			_simplex.getTriangle(closestTriangleId, id0, id1, id2);

			//split the first edge
			{
				snVec ve1 = _simplex.computeClosestPointForSegment(triangle[0], triangle[1], snVec4Set(0, 0, 0, 1));
				snVec we1 = support(_c1, _c2, ve1);
				if (!(snVec3Dot(ve1, we1) == snVec3Dot(ve1, ve1)))
				{
					int edgeVertexId = _simplex.addVertex(we1);
					_simplex.addTriangle(id0, edgeVertexId, newVertexId);
					_simplex.addTriangle(edgeVertexId, id1, newVertexId);
				}
				else
				{
					_simplex.addTriangle(id0, id1, newVertexId);
				}
			}
			//split the second edge
			{
				snVec ve1 = _simplex.computeClosestPointForSegment(triangle[1], triangle[2], snVec4Set(0, 0, 0, 1));
				snVec we1 = support(_c1, _c2, ve1);
				if (!(snVec3Dot(ve1, we1) == snVec3Dot(ve1, ve1)))
				{
					int edgeVertexId = _simplex.addVertex(we1);
					_simplex.addTriangle(id1, edgeVertexId, newVertexId);
					_simplex.addTriangle(edgeVertexId, id2, newVertexId);
				}
				else
				{
					_simplex.addTriangle(id1, id2, newVertexId);
				}
			}
			//split the third edge
			{
				snVec ve1 = _simplex.computeClosestPointForSegment(triangle[2], triangle[0], snVec4Set(0, 0, 0, 1));
				snVec we1 = support(_c1, _c2, ve1);
				if (!(snVec3Dot(ve1, we1) == snVec3Dot(ve1, ve1)))
				{
					int edgeVertexId = _simplex.addVertex(we1);
					_simplex.addTriangle(id2, edgeVertexId, newVertexId);
					_simplex.addTriangle(edgeVertexId, id0, newVertexId);
				}
				else
				{
					_simplex.addTriangle(id2, id0, newVertexId);
				}
			}
			_simplex.setTriangleValidity(closestTriangleId, false);
			//_simplex.expand(newVertex, closestTriangleId);
		}
	}

	snVec snGJK::UpdateSimplex(snVec* _s, int& _n)
	{
		if (_n == 2)
		{
			return UpdateTwoSimplex(_s, _n);
		}
		else if (_n == 3)
		{
			return UpdateThreeSimplex(_s, _n);
		}
		else
		{
			return UpdateFourSimplex(_s, _n);
		}
	}

	snVec snGJK::UpdateTwoSimplex(snVec* _s, int& _n)
	{
		// Four voronoi regions that the origin could be in:
		// 0) closest to vertex s[0].
		// 1) closest to vertex s[1].
		// 2) closest to line segment s[0]->s[1]. XX
		// 3) contained in the line segment s[0]->s[1], and our search is over and the algorithm is now finished. XX

		// By construction of the simplex, the cases 0) and 1) can never occur. Then only the cases marked with XX need to be checked.
#ifdef MATH_ASSERT_CORRECTNESS
		// Sanity-check that the above reasoning is valid by testing each voronoi region and assert()ing that the ones we assume never to
		// happen never will.
		float d0 = s[0].DistanceSq(vec::zero);
		float d1 = s[1].DistanceSq(vec::zero);
		float d2 = LineSegment(s[0], s[1]).DistanceSq(vec::zero);
		assert2(d2 <= d0, d2, d0);
		assert2(d2 <= d1, d2, d1);
		// Cannot be in case 0: the step 0 -> 1 must have been toward the zero direction:
		assert(Dot(s[1] - s[0], -s[0]) >= 0.f);
		// Cannot be in case 1: the zero direction cannot be in the voronoi region of vertex s[1].
		assert(Dot(s[1] - s[0], -s[1]) <= 0.f);
#endif

		snVec d01 = _s[1] - _s[0];
		snVec newSearchDir = snVec3Cross(d01, snVec3Cross(d01, _s[1]));
		if (snVec3SquaredNorme(newSearchDir) > 1e-7f)
			return newSearchDir; // Case 2)
		else
		{
			// Case 3)
			_n = 0;
			return VEC_ZERO;//vec::zero;
		}
	}

	snVec snGJK::UpdateThreeSimplex(snVec* _s, int& _n)
	{
		// Nine voronoi regions:
		// 0) closest to vertex s[0].
		// 1) closest to vertex s[1].
		// 2) closest to vertex s[2].
		// 3) closest to edge s[0]->s[1].
		// 4) closest to edge s[1]->s[2].  XX
		// 5) closest to edge s[0]->s[2].  XX
		// 6) closest to the triangle s[0]->s[1]->s[2], in the positive side.  XX
		// 7) closest to the triangle s[0]->s[1]->s[2], in the negative side.  XX
		// 8) contained in the triangle s[0]->s[1]->s[2], and our search is over and the algorithm is now finished.  XX

		// By construction of the simplex, the origin must always be in a voronoi region that includes the point s[2], since that
		// was the last added point. But it cannot be the case 2), since previous search took us deepest towards the direction s[1]->s[2],
		// and case 2) implies we should have been able to go even deeper in that direction, or that the origin is not included in the convex shape,
		// a case which has been checked for already before. Therefore the cases 0)-3) can never occur. Only the cases marked with XX need to be checked.
#ifdef MATH_ASSERT_CORRECTNESS
		// Sanity-check that the above reasoning is valid by testing each voronoi region and assert()ing that the ones we assume never to
		// happen never will.
		float d[7];
		d[0] = _s[0].DistanceSq(vec::zero);
		d[1] = _s[1].DistanceSq(vec::zero);
		d[2] = _s[2].DistanceSq(vec::zero);
		d[3] = LineSegment(s[0], s[1]).DistanceSq(vec::zero);
		d[4] = LineSegment(s[1], s[2]).DistanceSq(vec::zero);
		d[5] = LineSegment(s[2], s[0]).DistanceSq(vec::zero);
		d[6] = Triangle(s[0], s[1], s[2]).DistanceSq(vec::zero);

		bool isContainedInTriangle = (d[6] <= 1e-3f); // Are we in case 8)?
		float dist = FLOAT_INF;
		int minDistIndex = -1;
		for (int i = 4; i < 7; ++i)
		if (d[i] < dist)
		{
			dist = d[i];
			minDistIndex = i;
		}

		assert4(isContainedInTriangle || dist <= d[0] + 1e-4f, d[0], dist, isContainedInTriangle, minDistIndex);
		assert4(isContainedInTriangle || dist <= d[1] + 1e-4f, d[1], dist, isContainedInTriangle, minDistIndex);
		assert4(isContainedInTriangle || dist <= d[2] + 1e-4f, d[2], dist, isContainedInTriangle, minDistIndex);
		assert4(isContainedInTriangle || dist <= d[3] + 1e-4f, d[3], dist, isContainedInTriangle, minDistIndex);
#endif
		snVec d12 = _s[2] - _s[1];
		snVec d02 = _s[2] - _s[0];
		snVec triNormal = snVec3Cross(d02, d12);

		snVec e12 = snVec3Cross(d12, triNormal);
		float t12 = snVec4GetX(snVec3Dot(_s[1], e12)); //make a static 0 vector and compare the dot to the zero vector.
		if (t12 < 0.f)
		{
			// Case 4: Edge 1->2 is closest.
#ifdef MATH_ASSERT_CORRECTNESS
			assert4(d[4] <= dist + 1e-3f * Max(1.f, d[4], dist), d[4], dist, isContainedInTriangle, minDistIndex);
#endif
			snVec newDir = snVec3Cross(d12, snVec3Cross(d12, _s[1]));
			_s[0] = _s[1];
			_s[1] = _s[2];
			_n = 2;
			return newDir;
		}
		snVec e02 = snVec3Cross(triNormal, d02);
		float t02 = snVec4GetX(snVec3Dot(_s[0], e02));
		if (t02 < 0.f)
		{
			// Case 5: Edge 0->2 is closest.
#ifdef MATH_ASSERT_CORRECTNESS
			assert4(d[5] <= dist + 1e-3f * Max(1.f, d[5], dist), d[5], dist, isContainedInTriangle, minDistIndex);
#endif
			snVec newDir = snVec3Cross(d02, snVec3Cross(d02, _s[0]));
			_s[1] = _s[2];
			_n = 2;
			return newDir;
		}
		// Cases 6)-8):
#ifdef MATH_ASSERT_CORRECTNESS
		assert4(d[6] <= dist + 1e-3f * Max(1.f, d[6], dist), d[6], dist, isContainedInTriangle, minDistIndex);
#endif
		float scaledSignedDistToTriangle = snVec4GetX(snVec3Dot(triNormal, _s[2]));
		float distSq = scaledSignedDistToTriangle*scaledSignedDistToTriangle;
		float scaledEpsilonSq = 1e-6f * snVec3SquaredNorme(triNormal);

		if (distSq > scaledEpsilonSq)
		{
			// The origin is sufficiently far away from the triangle.
			if (scaledSignedDistToTriangle <= 0.f)
				return triNormal; // Case 6)
			else
			{
				// Case 7) Swap s[0] and s[1] so that the normal of Triangle(s[0],s[1],s[2]).PlaneCCW() will always point towards the new search direction.
				std::swap(_s[0], _s[1]);
				return -triNormal;
			}
		}
		else
		{
			// Case 8) The origin lies directly inside the triangle. For robustness, terminate the search here immediately with success.
			_n = 0;
			return VEC_ZERO;//vec::zero;
		}
	}

	snVec snGJK::UpdateFourSimplex(snVec* _s, int& _n)
	{
		// A tetrahedron defines fifteen voronoi regions:
		//  0) closest to vertex s[0].
		//  1) closest to vertex s[1].
		//  2) closest to vertex s[2].
		//  3) closest to vertex s[3].
		//  4) closest to edge s[0]->s[1].
		//  5) closest to edge s[0]->s[2].
		//  6) closest to edge s[0]->s[3].  XX
		//  7) closest to edge s[1]->s[2].
		//  8) closest to edge s[1]->s[3].  XX
		//  9) closest to edge s[2]->s[3].  XX
		// 10) closest to the triangle s[0]->s[1]->s[2], in the outfacing side.
		// 11) closest to the triangle s[0]->s[1]->s[3], in the outfacing side. XX
		// 12) closest to the triangle s[0]->s[2]->s[3], in the outfacing side. XX
		// 13) closest to the triangle s[1]->s[2]->s[3], in the outfacing side. XX
		// 14) contained inside the tetrahedron simplex, and our search is over and the algorithm is now finished. XX

		// By construction of the simplex, the origin must always be in a voronoi region that includes the point s[3], since that
		// was the last added point. But it cannot be the case 3), since previous search took us deepest towards the direction s[2]->s[3],
		// and case 3) implies we should have been able to go even deeper in that direction, or that the origin is not included in the convex shape,
		// a case which has been checked for already before. Therefore the cases 0)-5), 7) and 10) can never occur and
		// we only need to check cases 6), 8), 9), 11), 12), 13) and 14), marked with XX.

#ifdef MATH_ASSERT_CORRECTNESS
		// Sanity-check that the above reasoning is valid by testing each voronoi region and assert()ing that the ones we assume never to
		// happen never will.
		float d[14];
		d[0] = s[0].DistanceSq(vec::zero);
		d[1] = s[1].DistanceSq(vec::zero);
		d[2] = s[2].DistanceSq(vec::zero);
		d[3] = s[3].DistanceSq(vec::zero);
		d[4] = LineSegment(s[0], s[1]).DistanceSq(vec::zero);
		d[5] = LineSegment(s[0], s[2]).DistanceSq(vec::zero);
		d[6] = LineSegment(s[0], s[3]).DistanceSq(vec::zero);
		d[7] = LineSegment(s[1], s[2]).DistanceSq(vec::zero);
		d[8] = LineSegment(s[1], s[3]).DistanceSq(vec::zero);
		d[9] = LineSegment(s[2], s[3]).DistanceSq(vec::zero);
		d[10] = Triangle(s[0], s[1], s[2]).DistanceSq(vec::zero);
		d[11] = Triangle(s[0], s[1], s[3]).DistanceSq(vec::zero);
		d[12] = Triangle(s[0], s[2], s[3]).DistanceSq(vec::zero);
		d[13] = Triangle(s[1], s[2], s[3]).DistanceSq(vec::zero);

		vec Tri013Normal = Cross(s[1] - s[0], s[3] - s[0]);
		vec Tri023Normal = Cross(s[3] - s[0], s[2] - s[0]);
		vec Tri123Normal = Cross(s[2] - s[1], s[3] - s[1]);
		vec Tri012Normal = Cross(s[2] - s[0], s[1] - s[0]);
		assert(Dot(Tri012Normal, s[3] - s[0]) <= 0.f);
		float InTri012 = Dot(-s[0], Tri012Normal);
		float InTri013 = Dot(-s[3], Tri013Normal);
		float InTri023 = Dot(-s[3], Tri023Normal);
		float InTri123 = Dot(-s[3], Tri123Normal);
		bool insideSimplex = InTri012 <= 0.f && InTri013 <= 0.f && InTri023 <= 0.f && InTri123 <= 0.f;

		float dist = FLOAT_INF;
		int minDistIndex = -1;
		for (int i = 6; i < 14; ++i)
		if (i == 6 || i == 8 || i == 9 || i == 11 || i == 12 || i == 13 || i == 14)
		if (d[i] < dist)
		{
			dist = d[i];
			minDistIndex = i;
		}
		assert4(insideSimplex || dist <= d[0] + 1e-4f * Max(1.f, d[0], dist), d[0], dist, insideSimplex, minDistIndex);
		assert4(insideSimplex || dist <= d[1] + 1e-4f * Max(1.f, d[1], dist), d[1], dist, insideSimplex, minDistIndex);
		assert4(insideSimplex || dist <= d[2] + 1e-4f * Max(1.f, d[2], dist), d[2], dist, insideSimplex, minDistIndex);
		assert4(insideSimplex || dist <= d[4] + 1e-4f * Max(1.f, d[4], dist), d[4], dist, insideSimplex, minDistIndex);
		assert4(insideSimplex || dist <= d[5] + 1e-4f * Max(1.f, d[5], dist), d[5], dist, insideSimplex, minDistIndex);
		assert4(insideSimplex || dist <= d[7] + 1e-4f * Max(1.f, d[7], dist), d[7], dist, insideSimplex, minDistIndex);
		assert4(insideSimplex || dist <= d[10] + 1e-4f * Max(1.f, d[10], dist), d[10], dist, insideSimplex, minDistIndex);
#endif

		snVec d01 = _s[1] - _s[0];
		snVec d02 = _s[2] - _s[0];
		snVec d03 = _s[3] - _s[0];
		snVec tri013Normal = snVec3Cross(d01, d03); // Normal of triangle 0->1->3 pointing outwards from the simplex.
		snVec tri023Normal = snVec3Cross(d03, d02); // Normal of triangle 0->2->3 pointing outwards from the simplex.
		assert(snVec4GetX(snVec3Dot(tri013Normal, d02)) <= 0.f);
		assert(snVec4GetX(snVec3Dot(tri023Normal, d01)) <= 0.f);

		snVec e03_1 = snVec3Cross(tri013Normal, d03); // The normal of edge 0->3 on triangle 013.
		snVec e03_2 = snVec3Cross(d03, tri023Normal); // The normal of edge 0->3 on triangle 023.
		float inE03_1 = snVec4GetX(snVec3Dot(e03_1, _s[3]));
		float inE03_2 = snVec4GetX(snVec3Dot(e03_2, _s[3]));
		if (inE03_1 <= 0.f && inE03_2 <= 0.f)
		{
			// Case 6) Edge 0->3 is closest. Simplex degenerates to a line segment.
#ifdef MATH_ASSERT_CORRECTNESS
			assert4(!insideSimplex && d[6] <= dist + 1e-3f * Max(1.f, d[6], dist), d[6], dist, insideSimplex, minDistIndex);
#endif
			snVec newDir = snVec3Cross(d03, snVec3Cross(d03, _s[3]));
			_s[1] = _s[3];
			_n = 2;
			return newDir;
		}

		snVec d12 = _s[2] - _s[1];
		snVec d13 = _s[3] - _s[1];
		snVec tri123Normal = snVec3Cross(d12, d13);
		assert(snVec4GetX(snVec3Dot(tri123Normal, -d02)) <= 0.f);
		snVec e13_0 = snVec3Cross(d13, tri013Normal);
		snVec e13_2 = snVec3Cross(tri123Normal, d13);
		float inE13_0 = snVec4GetX(snVec3Dot(e13_0, _s[3]));
		float inE13_2 = snVec4GetX(snVec3Dot(e13_2, _s[3]));
		if (inE13_0 <= 0.f && inE13_2 <= 0.f)
		{
			// Case 8) Edge 1->3 is closest. Simplex degenerates to a line segment.
#ifdef MATH_ASSERT_CORRECTNESS
			assert4(!insideSimplex && d[8] <= dist + 1e-3f * Max(1.f, d[8], dist), d[8], dist, insideSimplex, minDistIndex);
#endif
			snVec newDir = snVec3Cross(d13, snVec3Cross(d13, _s[3]));
			_s[0] = _s[1];
			_s[1] = _s[3];
			_n = 2;
			return newDir;
		}

		snVec d23 = _s[3] - _s[2];
		snVec e23_0 = snVec3Cross(tri023Normal, d23);
		snVec e23_1 = snVec3Cross(d23, tri123Normal);
		float inE23_0 = snVec4GetX(snVec3Dot(e23_0, _s[3]));
		float inE23_1 = snVec4GetX(snVec3Dot(e23_1, _s[3]));
		if (inE23_0 <= 0.f && inE23_1 <= 0.f)
		{
			// Case 9) Edge 2->3 is closest. Simplex degenerates to a line segment.
#ifdef MATH_ASSERT_CORRECTNESS
			assert4(!insideSimplex && d[9] <= dist + 1e-3f * Max(1.f, d[9], dist), d[9], dist, insideSimplex, minDistIndex);
#endif
			snVec newDir = snVec3Cross(d23, snVec3Cross(d23, _s[3]));
			_s[0] = _s[2];
			_s[1] = _s[3];
			_n = 2;
			return newDir;
		}

		float inTri013 = snVec4GetX(snVec3Dot(_s[3], tri013Normal));
		if (inTri013 < 0.f && inE13_0 >= 0.f && inE03_1 >= 0.f)
		{
			// Case 11) Triangle 0->1->3 is closest.
#ifdef MATH_ASSERT_CORRECTNESS
			assert4(!insideSimplex && d[11] <= dist + 1e-3f * Max(1.f, d[11], dist), d[11], dist, insideSimplex, minDistIndex);
#endif
			_s[2] = _s[3];
			_n = 3;
			return tri013Normal;
		}
		float inTri023 = snVec4GetX(snVec3Dot(_s[3], tri023Normal));
		if (inTri023 < 0.f && inE23_0 >= 0.f && inE03_2 >= 0.f)
		{
			// Case 12) Triangle 0->2->3 is closest.
#ifdef MATH_ASSERT_CORRECTNESS
			assert4(!insideSimplex && d[12] <= dist + 1e-3f * Max(1.f, d[12], dist), d[12], dist, insideSimplex, minDistIndex);
#endif
			_s[1] = _s[0];
			_s[0] = _s[2];
			_s[2] = _s[3];
			_n = 3;
			return tri023Normal;
		}
		float inTri123 = snVec4GetX(snVec3Dot(_s[3], tri123Normal));
		if (inTri123 < 0.f && inE13_2 >= 0.f && inE23_1 >= 0.f)
		{
			// Case 13) Triangle 1->2->3 is closest.
#ifdef MATH_ASSERT_CORRECTNESS
			assert4(!insideSimplex && d[13] <= dist + 1e-3f * Max(1.f, d[13], dist), d[13], dist, insideSimplex, minDistIndex);
#endif
			_s[0] = _s[1];
			_s[1] = _s[2];
			_s[2] = _s[3];
			_n = 3;
			return tri123Normal;
		}

		// Case 14) Not in the voronoi region of any triangle or edge. The origin is contained in the simplex, the search is finished.
		_n = 0;
		return VEC_ZERO;//vec::zero;
	}

}