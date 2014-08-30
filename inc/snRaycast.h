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
#ifndef SN_RAYCAST_H
#define SN_RAYCAST_H

#include "snVec.h"

namespace Supernova
{
	class snRay;
	struct snAABB;
	class snHeightMap;

	class snRaycast
	{
	public:

		//Test for intersection between a ray and a AABB.
		// _ray : the ray to use to check for intersection
		// _aabb : axis aligned bounding box.
		// _hit : if there is an intersection, it contains the intersection point.
		// return : true for intersection. False otherwise.
		static bool RayAABB(const snRay& _ray, const snAABB& _aabb, snVec& _hit);

		//Test for intersection between a ray and a heightmap.
		// _ray : the ray to check for intersection.
		// _hmap : the heightmap to check for intersection.
		// _hit : Contains the intersection point.
		// return : True for intersection. False otherwise.
		static bool RayHeightmap(const snRay& _ray, const snHeightMap& _hmap, snVec& _hit);

		//Test for intersection between a ray and a triangle.
		// _ray the ray to check for intersection.
		// _v1 : first vertex of the triangle.
		// _v2 : second vertex of the triangle.
		// _v3 : third vertex of the triangle.
		// _hit : Contains the intersection point.
		// return : True for intersection. False otherwise.
		static bool RayTriangle(const snRay& _ray, const snVec& _v1, const snVec& _v2, const snVec& _v3, snVec& _hit);
	};
}

#endif //ifndef SN_RAYCAST_H