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
#include "snEPASimplex.h"

#include "snIGJKCollider.h"

#include <assert.h>

using namespace Supernova::Vector;

#define SN_SAME_DIRECTION(_v1, _v2) (snVec4GetX(snVec3Dot(_v1, _v2)) > 0)

#ifdef _DEBUG
	#define SANITY_CHECK
#endif

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

	snVec snGJK::updateSimplex(snVec* _s, int& _n)
	{
		if (_n == 2)
		{
			return updateTwoSimplex(_s, _n);
		}
		else if (_n == 3)
		{
			return updateThreeSimplex(_s, _n);
		}
		else
		{
			return updateFourSimplex(_s, _n);
		}
	}

	snVec snGJK::updateTwoSimplex(snVec* _s, int& _n)
	{
		//Case 0 : closest to _s[0].
		//Case 1 : closest to _s[1].
		//Case 2 : closest to line 01.
		//Case 3 : on the line 01 => intersection.

		//Case 0 : We come from _s[0] and we added _s[1] so _s[0] cannot be the closest point to the origin.
		//Case 1 : The loop already checks if we passed the origin so _s[1] cannot be the closest point.
		//We are left with case 2 and 3.

		snVec d01 = _s[1] - _s[0];

#ifdef SANITY_CHECK

		snVec dir = d01;
		snVec3Normalize(dir);

		float dot = snVec4GetX(snVec3Dot(dir, -_s[0]));

		//if dot < 0.f then case 0
		assert(dot >= 0.f);

		//if dot > length then cas 1
		float length = snVec3Norme(d01);
		assert(dot <= length);
#endif

		snVec newSearchDir = snVec3Cross(d01, snVec3Cross(d01, _s[1]));
		if (snVec3SquaredNorme(newSearchDir) > 1e-7f)
		{
			//Case 3
			return newSearchDir;
		}
		else
		{
			//If the the origin is on the line d01 and _s[1] are colinear. Their cross product is 0. This is case 3
			_n = 0;
			return VEC_ZERO;//vec::zero;
		}
	}

	snVec snGJK::updateThreeSimplex(snVec* _s, int& _n)
	{
		//Case 0 : closest to _s[0].
		//Case 1 : closest to _s[1].
		//Case 2 : closest to _s[2].
		//Case 3 : closest to edge 01.
		//Case 4 : closest to edge 12.
		//Case 5 : closest to edge 02.
		//Case 6 : closest to the triangle 012, in the positive side.
		//Case 7 : closest to the triangle 012, in the negative side.
		//Case 8 : contained inside the triangle 012 => intersection.

		//Case 0 : already checked during the previous iteration.
		//Case 1 : already checked during the previous iteration.
		//Case 2 : this is the last point added to the simplex and it passed the origin or else we would have exited.
		//Case 3 : can't be here as _s[2] made us move closer to the origin.

		snVec d12 = _s[2] - _s[1];
		snVec d02 = _s[2] - _s[0];
		snVec triNormal = snVec3Cross(d02, d12);

#ifdef SANITY_CHECK

		snVec d01 = _s[1] - _s[0];
		snVec e01 = snVec3Cross(d01, triNormal);
		
		assert(! (!SN_SAME_DIRECTION(d01, -_s[0]) && !SN_SAME_DIRECTION(d02, -_s[0]) )); //Case 0
		assert(! (!SN_SAME_DIRECTION(-d01, -_s[1]) && !SN_SAME_DIRECTION(d12, -_s[1]) )); //Case 1
		assert(! (!SN_SAME_DIRECTION(-d02, -_s[2]) && !SN_SAME_DIRECTION(-d12, -_s[2]) )); //Case 2
		assert(! (SN_SAME_DIRECTION(d01, -_s[0]) && SN_SAME_DIRECTION(-d01, -_s[1]) && SN_SAME_DIRECTION(e01, -_s[1]) )); //Case 3
		
		//float d[7];
		//d[0] = _s[0].DistanceSq(vec::zero);
		//d[1] = _s[1].DistanceSq(vec::zero);
		//d[2] = _s[2].DistanceSq(vec::zero);
		//d[3] = LineSegment(s[0], s[1]).DistanceSq(vec::zero);
		//d[4] = LineSegment(s[1], s[2]).DistanceSq(vec::zero);
		//d[5] = LineSegment(s[2], s[0]).DistanceSq(vec::zero);
		//d[6] = Triangle(s[0], s[1], s[2]).DistanceSq(vec::zero);

		//bool isContainedInTriangle = (d[6] <= 1e-3f); // Are we in case 8)?
		//float dist = FLOAT_INF;
		//int minDistIndex = -1;
		//for (int i = 4; i < 7; ++i)
		//if (d[i] < dist)
		//{
		//	dist = d[i];
		//	minDistIndex = i;
		//}

		//assert4(isContainedInTriangle || dist <= d[0] + 1e-4f, d[0], dist, isContainedInTriangle, minDistIndex);
		//assert4(isContainedInTriangle || dist <= d[1] + 1e-4f, d[1], dist, isContainedInTriangle, minDistIndex);
		//assert4(isContainedInTriangle || dist <= d[2] + 1e-4f, d[2], dist, isContainedInTriangle, minDistIndex);
		//assert4(isContainedInTriangle || dist <= d[3] + 1e-4f, d[3], dist, isContainedInTriangle, minDistIndex);
#endif
		

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

	snVec snGJK::updateFourSimplex(snVec* _s, int& _n)
	{
		//Case 0 : closest to vertex _s[0].
		//Case 1 : closest to vertex _s[1].
		//Case 2 : closest to vertex _s[2].
		//Case 3 : closest to vertex _s[3].
		//Case 4 : closest to edge 01.
		//Case 5 : closest to edge 02.
		//Case 6 : closest to edge 03.  XX
		//Case 7 : closest to edge 12.
		//Case 8 : closest to edge 13.  XX
		//Case 9 : closest to edge 23.  XX
		//Case 10 : closest to the triangle 012, in the outfacing side.
		//Case 11 : closest to the triangle 013, in the outfacing side. XX
		//Case 12 : closest to the triangle 023, in the outfacing side. XX
		//Case 13 : closest to the triangle 123, in the outfacing side. XX
		//Case 14 : contained inside the tetrahedron simplex => intersection. XX

		//Case 0 : It can't be, that's were we come from.
		//Case 1 : It can't be, that's were we come from.
		//Case 2 : It can't be, that's were we come from.
		//Case 3 : Already checked in the main loop of the algorithm.
		//Case 4 : It can't be, that's were we come from.
		//Case 5 : It can't be, that's were we come from.
		//Case 7 : It can't be, that's were we come from.
		//Case 10 : It can't be, that's were we come from.
		//Case to test : 6, 8, 9, 11, 12, 13


		//Reminder about triangles:
		//Triangle are defined anticlockwise. It means for the triangle 012, the outward normal is 01 X 02.
		//
		//To find the normal of an edge orthogonal to a triangle we need a vector defining the edge and the triangle outward normal.
		//If for the triangle, the edge is define clockwise then the normal is tri_normal X edge. 
		//If the edge is anticlockwise then the normal is edge X tri_normal.
#ifdef SANITY_CHECK
		snVec san_d01 = _s[1] - _s[0];
		snVec san_d03 = _s[3] - _s[0];
		snVec san_d02 = _s[2] - _s[0];
		assert(! (!SN_SAME_DIRECTION(san_d01, -_s[0]) && !SN_SAME_DIRECTION(san_d02, -_s[0]) && !SN_SAME_DIRECTION(san_d03, -_s[0]) )); //Case 0

		snVec san_d12 = _s[2] - _s[1];
		snVec san_d13 = _s[3] - _s[1];
		assert(! (!SN_SAME_DIRECTION(-san_d01, -_s[1]) && !SN_SAME_DIRECTION(san_d13, -_s[1]) && !SN_SAME_DIRECTION(san_d02, -_s[1]) )); //Case 1

		snVec san_d23 = _s[3] - _s[2];
		assert(!(!SN_SAME_DIRECTION(-san_d12, -_s[2]) && !SN_SAME_DIRECTION(san_d23, -_s[2]) && !SN_SAME_DIRECTION(-san_d02, -_s[2]))); //Case 2

		assert(!(!SN_SAME_DIRECTION(-san_d23, -_s[3]) && !SN_SAME_DIRECTION(-san_d13, -_s[3]) && !SN_SAME_DIRECTION(-san_d03, -_s[3]))); //Case 3

		snVec san_tri013Normal = snVec3Cross(san_d01, san_d03);
		snVec san_tri012Normal = snVec3Cross(san_d02, san_d01);
		snVec san_e01_1 = snVec3Cross(san_d01, san_tri013Normal);
		snVec san_e01_2 = snVec3Cross(san_tri012Normal, san_d01);
		assert(! (SN_SAME_DIRECTION(san_e01_1, -_s[0]) && SN_SAME_DIRECTION(san_e01_2, -_s[0]))); //Case 4

		snVec san_tri023Normal = snVec3Cross(san_d03, san_d02);
		snVec san_e02_1 = snVec3Cross(san_tri023Normal, san_d02);
		snVec san_e02_2 = snVec3Cross(san_d02, san_tri012Normal);
		assert(!(SN_SAME_DIRECTION(san_e02_1, -_s[0]) && SN_SAME_DIRECTION(san_e02_2, -_s[0]))); //Case 5

		snVec san_tri123Normal = snVec3Cross(san_d23, -san_d12);
		snVec san_e12_1 = snVec3Cross(san_d12, san_tri123Normal);
		snVec san_e12_2 = snVec3Cross(san_tri012Normal, san_d12);
		assert(!(SN_SAME_DIRECTION(san_e12_1, -_s[1]) && SN_SAME_DIRECTION(san_e12_2, -_s[1]))); //Case 7

		assert(! SN_SAME_DIRECTION(san_tri012Normal, -_s[0]));//Case 10
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