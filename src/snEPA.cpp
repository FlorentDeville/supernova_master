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
#include "snEPA.h"

#include "snEPASimplex.h"
#include "snICollider.h"
using namespace Supernova::Vector;

#include "snClosestPoint.h"

#include <assert.h>

namespace Supernova
{
	bool snEPA::execute(snSimplex& _simplex, const snICollider& _c1, const snICollider& _c2, snVec& _normal) const
	{
		snEPATriangle* closestTriangle = 0;

		bool loopOver = false;
		while (!loopOver)
		{
			//Get the closest triangle to the origin
			closestTriangle = _simplex.getClosestTriangleToOrigin();
			if (closestTriangle == 0)
				return false;

			//Get the closest distance to the origin
			snVec closestPoint = closestTriangle->getClosestPoint();
			float distance = closestTriangle->getSqDistance();

			//Get a point in the same direction as the outward normal
			snVec direction = closestPoint;
			snVec3Normalize(direction);
			snVec newVertex = _c1.support(direction) - _c2.support(-direction);

			//Check if we are closer to the origin
			float newDistance = snVec4GetX(snVec3Dot(newVertex, closestPoint));

			//If the new distance is not bigger than the closest triangle, then finish
			const float EPA_TOLERANCE = 0.01f;
			if ((newDistance - distance) < EPA_TOLERANCE)
			{
				//If the distance is inferior to contact epsilon, consider it as no collision
				const float CONTACT_EPSILON = 10e-5f;
				if (distance <= CONTACT_EPSILON)
					return false;

				break;
			}

			//Expand the simplex
			unsigned int vertexId = _simplex.addVertex(newVertex);
			if (!closestTriangle->quickHull(_simplex, vertexId))
				return false;
		}

		//Compute the normal and depth using the closest triangle.
		_normal = -closestTriangle->getClosestPoint();
		snVec3Normalize(_normal);
		return true;
	}

	bool snEPA::prepareSimplex(snVec * const _inSimplex, unsigned int _inSimplexSize, const snICollider& _c1, const snICollider& _c2, snSimplex& _outSimplex) const
	{
		if (_inSimplexSize == 1)
		{
			//Ignore the collision when the simplex is a point.
			return false;
		}
		else if (_inSimplexSize == 2)
		{
			return prepareSimplex2(_inSimplex, _c1, _c2, _outSimplex);
		}
		else if (_inSimplexSize == 3)
		{
			return prepareSimplex3(_inSimplex, _c1, _c2, _outSimplex);
		}
		else if (_inSimplexSize == 4)
		{
			return prepareSimplex4(_inSimplex, _outSimplex);
		}
		else
		{
			throw; // simplex size not handled
		}
	}

	bool snEPA::prepareSimplex2(snVec * const _inSimplex, const snICollider& _c1, const snICollider& _c2, snSimplex& _outSimplex) const
	{
		//GJK returned a line. 
		//Let's find three points around that line to make a simplex containing the origin.

		// Direction of the segment
		snVec d = _inSimplex[1] - _inSimplex[0];
		snVec3Normalize(d);

		// Choose the coordinate axis from the minimal absolute component of the vector d
		unsigned int minAxis = snVec3GetMinAxis(d);

		// sin(60) = sqrt(3) / 2
		const float sin60 = sqrt(3.f) * 0.5f;

		// Create a rotation quaternion to rotate the direction vector of 120 degrees.
		// R = (sin(theta/2) * x, sin(theta/2) * y, sin(theta/2)*z, cos(theta/2))
		//In our case sin(theta/2) = sin(60).
		snVec rotationQuat = d * sin60;
		snVec4SetW(rotationQuat, 0.5f);
		snMatrix44f rotationMat;
		rotationMat.createRotationFromQuaternion(rotationQuat);

		// Compute the vector v1, v2, v3
		snVec v1 = snVec3Cross(d, snVec4Set(minAxis == 0, minAxis == 1, minAxis == 2, 0));
		snVec v2 = snMatrixTransform3(v1, rotationMat);
		snVec v3 = snMatrixTransform3(v2, rotationMat);
		snVec3Normalize(v1);
		snVec3Normalize(v2);
		snVec3Normalize(v3);

		// Compute new support points.
		snVec newPoint2 = _c1.support(v1) - _c2.support(-v1);
		snVec newPoint3 = _c1.support(v2) - _c2.support(-v2);
		snVec newPoint4 = _c1.support(v3) - _c2.support(-v3);

		//Check which tetrahedron contains the origin
		if (isValidStartSimplex(_inSimplex[0], newPoint2, newPoint3, newPoint4))
		{
			//Use the point 4 instead of point 1
			_outSimplex.addVertex(_inSimplex[0]);
			_outSimplex.addVertex(newPoint4);
			_outSimplex.addVertex(newPoint2);
			_outSimplex.addVertex(newPoint3);
		}
		else if (isValidStartSimplex(_inSimplex[1], newPoint2, newPoint3, newPoint4))
		{
			//Use the point 4 instead of point 0
			_outSimplex.addVertex(newPoint4);
			_outSimplex.addVertex(_inSimplex[1]);
			_outSimplex.addVertex(newPoint2);
			_outSimplex.addVertex(newPoint3);
		}
		else
		{
			// The origin is not in the initial polytope
			return false;
		}

		//Create the simplex
		snEPATriangle* t0 = _outSimplex.addTriangle(0, 2, 1);
		snEPATriangle* t1 = _outSimplex.addTriangle(0, 1, 3);
		snEPATriangle* t2 = _outSimplex.addTriangle(1, 2, 3);
		snEPATriangle* t3 = _outSimplex.addTriangle(2, 0, 3);

		_outSimplex.addLink(t0, 0, t3, 0);
		_outSimplex.addLink(t0, 1, t2, 0);
		_outSimplex.addLink(t0, 2, t1, 0);
		_outSimplex.addLink(t1, 1, t2, 2);
		_outSimplex.addLink(t2, 1, t3, 2);
		_outSimplex.addLink(t3, 1, t1, 2);

		return true;
	}

	bool snEPA::prepareSimplex3(snVec * const _inSimplex, const snICollider& _c1, const snICollider& _c2, snSimplex& _outSimplex) const
	{
		//GJK returned a triangle.
		//Find two points, each one on a side of the triangle and make a simplex out of that.

		//Compute the normal of the triangle
		snVec v1 = _inSimplex[1] - _inSimplex[0];
		snVec v2 = _inSimplex[2] - _inSimplex[0];
		snVec n = snVec3Cross(v1, v2);
		snVec3Normalize(n);

		//Add existing points to the simplex
		for (unsigned int i = 0; i < 3; ++i)
			_outSimplex.addVertex(_inSimplex[i]);

		// Compute the two new vertices to obtain a hexahedron
		int id4 = _outSimplex.addVertex(_c1.support(n) - _c2.support(-n));
		int id5 = _outSimplex.addVertex(_c1.support(-n) - _c2.support(n));

		//Create the triangles
		snEPATriangle* t0 = _outSimplex.addTriangle(0, 1, id4);
		snEPATriangle* t1 = _outSimplex.addTriangle(0, id4, 2);
		snEPATriangle* t2 = _outSimplex.addTriangle(1, 2, id4);

		snEPATriangle* t3 = _outSimplex.addTriangle(1, 0, id5);
		snEPATriangle* t4 = _outSimplex.addTriangle(id5, 0, 2);
		snEPATriangle* t5 = _outSimplex.addTriangle(1, id5, 2);

		//Create the links
		_outSimplex.addLink(t0, 1, t2, 2);
		_outSimplex.addLink(t0, 2, t1, 0);
		_outSimplex.addLink(t1, 1, t2, 1);

		_outSimplex.addLink(t3, 2, t5, 0);
		_outSimplex.addLink(t5, 1, t4, 2);
		_outSimplex.addLink(t4, 0, t3, 1);

		_outSimplex.addLink(t2, 0, t5, 2);
		_outSimplex.addLink(t1, 2, t4, 1);
		_outSimplex.addLink(t0, 0, t3, 0);

		return true;
	}

	bool snEPA::prepareSimplex4(snVec * const _inSimplex, snSimplex& _outSimplex) const
	{
		for (unsigned int i = 0; i < 4; ++i)
			_outSimplex.addVertex(_inSimplex[i]);

		snEPATriangle* t0 = _outSimplex.addTriangle(0, 2, 1);
		snEPATriangle* t1 = _outSimplex.addTriangle(0, 1, 3);
		snEPATriangle* t2 = _outSimplex.addTriangle(1, 2, 3);
		snEPATriangle* t3 = _outSimplex.addTriangle(2, 0, 3);

		_outSimplex.addLink(t0, 0, t3, 0);
		_outSimplex.addLink(t0, 1, t2, 0);
		_outSimplex.addLink(t0, 2, t1, 0);
		_outSimplex.addLink(t1, 1, t2, 2);
		_outSimplex.addLink(t2, 1, t3, 2);
		_outSimplex.addLink(t3, 1, t1, 2);

		return true;
	}

	bool snEPA::isValidStartSimplex(const snVec& _p1, const snVec& _p2, const snVec& _p3, const snVec& _p4) const
	{
		//Check triangle 123
		snVec normal1 = snVec3Cross(_p2 - _p1, _p3 - _p1);
		if (snVec3Inferior(VEC_ZERO, snVec3Dot(normal1, _p1)) == snVec3Inferior(VEC_ZERO, snVec3Dot(normal1, _p4)))
		{
			return false;
		}

		//Check triangle 234
		snVec normal2 = snVec3Cross(_p4 - _p2, _p3 - _p2);
		if (snVec3Inferior(VEC_ZERO, snVec3Dot(normal2, _p2)) == snVec3Inferior(VEC_ZERO, snVec3Dot(normal2, _p1)))
		{
			return false;
		}

		//Check triangle 134
		snVec normal3 = snVec3Cross(_p4 - _p3, _p1 - _p3);
		if (snVec3Inferior(VEC_ZERO, snVec3Dot(normal3, _p3)) == snVec3Inferior(VEC_ZERO, snVec3Dot(normal3, _p2)))
		{
			return false;
		}

		//Check triangle 124
		snVec normal4 = snVec3Cross(_p2 - _p4, _p1 - _p4);
		if (snVec3Inferior(VEC_ZERO, snVec3Dot(normal4, _p4)) == snVec3Inferior(VEC_ZERO, snVec3Dot(normal4, _p3)))
		{
			return false;
		}

		return true;
	}
}