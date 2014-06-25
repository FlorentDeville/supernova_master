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
#include "snVec.inl"

#include "snTypes.h"
#include "snMath.h"
#include "snCollisionResult.h"
#include "snSimplex.h"

#include "snIGJKCollider.h"

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
		snVec p1 = _c1.gjkSupport(_direction);
		snVec p2 = _c2.gjkSupport(-_direction);

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

	bool snGJK::checkOneSimplex(snVec* const _simplex, int& _simplexCount, snVec& _direction) const
	{
		//_simplex[1] is A. It is the last point added.
		//_simplex[0] is B. It is the first point entered in the simplex.
		snVec AB = _simplex[0] - _simplex[1];
		snVec AO = snVec4Set(0, 0, 0, 1) - _simplex[1];

		if (SN_SAME_DIRECTION(AB, AO))
			_direction = snVec3Cross(snVec3Cross(AB, AO), AB);
		else
		{
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
}