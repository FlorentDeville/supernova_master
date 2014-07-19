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
	bool snEPA::execute(snSimplex& _simplex, const snICollider& _c1, const snICollider& _c2, snVec& _normal, float& _depth) const
	{
		snEPATriangle* closestTriangle = 0;

		bool loopOver = false;
		while (!loopOver)
		{
			//get the closest triangle to the origin
			closestTriangle = _simplex.getClosestTriangleToOrigin();
			if (closestTriangle == 0)
				return false;

			//get the closest distance to the origin
			snVec closestPoint = closestTriangle->getClosestPoint();
			float distance = closestTriangle->getSqDistance();

			//get a point in the same direction as the outward normal
			snVec direction = closestPoint;
			snVec3Normalize(direction);
			snVec newVertex = _c1.support(direction) - _c2.support(-direction);

			//check if we are closer to the origin
			float newDistance = snVec4GetX(snVec3Dot(newVertex, closestPoint));

			//If the new distance is not bigger than the closest triangle, then finish
			const float EPA_TOLERANCE = 0.01f;
			if ((newDistance - distance) < EPA_TOLERANCE)
			{
				//if the distance is inferior to contact epsilon, consider it as no collision
				const float CONTACT_EPSILON = 10e-5f;
				if (distance <= CONTACT_EPSILON)
					return false;

				break;
			}

			//expand the simplex
			unsigned int vertexId = _simplex.addVertex(newVertex);
			if (!closestTriangle->quickHull(_simplex, vertexId))
				break;
		}

		//compute the normal and depth using the closest triangle.
		_normal = -closestTriangle->getClosestPoint();
		snVec3Normalize(_normal);
		_depth = sqrtf(closestTriangle->getSqDistance());
		return true;
	}

	// Decide if the origin is in the tetrahedron
	// Return 0 if the origin is in the tetrahedron and return the number (1,2,3 or 4) of
	// the vertex that is wrong if the origin is not in the tetrahedron
	bool snEPA::isValidStartSimplex(const snVec& _p1, const snVec& _p2, const snVec& _p3, const snVec& _p4, unsigned int _wrongVertexId) const
	{
		// Check vertex 1
		snVec normal1 = snVec3Cross(_p2 - _p1, _p3 - _p1);//(p2 - p1).cross(p3 - p1);
		//if (normal1.dot(p1) > 0.0 == normal1.dot(p4) > 0.0) 
		if (snVec3Inferior(VEC_ZERO,snVec3Dot(normal1, _p1)) == snVec3Inferior(VEC_ZERO, snVec3Dot(normal1, _p4)))
		{
			_wrongVertexId = 4;
			return false;
			//return 4;
		}

		// Check vertex 2
		snVec normal2 = snVec3Cross(_p4 - _p2, _p3 - _p2); //(p4 - p2).cross(p3 - p2);
		//if (normal2.dot(p2) > 0.0 == normal2.dot(p1) > 0.0) 
		if (snVec3Inferior(VEC_ZERO, snVec3Dot(normal2, _p2)) == snVec3Inferior(VEC_ZERO, snVec3Dot(normal2, _p1)))
		{
			_wrongVertexId = 1;
			return false;
			//return 1;
		}

		// Check vertex 3
		snVec normal3 = snVec3Cross(_p4 - _p3, _p1 - _p3); //(p4 - p3).cross(p1 - p3);
		//if (normal3.dot(p3) > 0.0 == normal3.dot(p2) > 0.0) 
		if (snVec3Inferior(VEC_ZERO, snVec3Dot(normal3, _p3)) == snVec3Inferior(VEC_ZERO, snVec3Dot(normal3, _p2)))
		{
			_wrongVertexId = 2;
			return false;
			//return 2;
		}

		// Check vertex 4
		snVec normal4 = snVec3Cross(_p2 - _p4, _p1 - _p4); //(p2 - p4).cross(p1 - p4);
		//if (normal4.dot(p4) > 0.0 == normal4.dot(p3) > 0.0) 
		if (snVec3Inferior(VEC_ZERO, snVec3Dot(normal4, _p4)) == snVec3Inferior(VEC_ZERO, snVec3Dot(normal4, _p3)))
		{
			_wrongVertexId = 3;
			return false;
			//return 3;
		}

		// The origin is in the tetrahedron, we return 0
		return true;
	}

}