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
#include "snIGJKCollider.h"
using namespace Supernova::Vector;

#include "snClosestPoint.h"

#include <assert.h>

namespace Supernova
{
	bool snEPA::execute(snSimplex& _simplex, const snIGJKCollider& _c1, const snIGJKCollider& _c2, snVec& _normal, float& _depth) const
	{
		bool loopOver = false;
		while (!loopOver)
		{
			//get the closest triangle to the origin
			snEPATriangle* closestTriangle = _simplex.getClosestTriangleToOrigin();
			assert(closestTriangle != 0);

			//get the closest distance to the origin
			snVec closestPoint = closestTriangle->getClosestPoint();
			float distance = closestTriangle->getSqDistance();

			//get a point in the same direction as the outward normal
			float dist0 = 0;
			float dist1 = 0;
			snVec newVertex = _c1.support(closestPoint, dist0) - _c2.support(-closestPoint, dist1);

			//check if we are closer to the origin
			float newDistance = dist0 + dist1;

			//If the new distance is not bigger than the closest triangle, then finish
			const float EPA_TOLERANCE = 0.01f;
			if ((newDistance - distance) < EPA_TOLERANCE)
			{
				_normal = -closestTriangle->getClosestPoint();
				snVec3Normalize(_normal);
				_depth = distance;
				return true;
			}

			//expand the simplex
			unsigned int vertexId = _simplex.addVertex(newVertex);
			closestTriangle->quickHull(_simplex, vertexId);
		}

		return false;
	}
}