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
#include "snRaycast.h"

#include "snRay.h"
#include "snAABB.h"
#include "snHeightMap.h"
#include "snMath.h"

using namespace Supernova::Vector;

#define IR(x)	((unsigned int&)x)

namespace Supernova
{
	bool snRaycast::RayAABB(const snRay& _ray, const snAABB& _aabb, snVec& _hit)
	{
		bool inside = true;
		snVec MinB = _aabb.m_min;
		snVec MaxB = _aabb.m_max;
		snVec MaxT = snVec4Set(-1);

		// Find candidate planes.
		for(unsigned int i=0;i<3;i++)
		{
			if(_ray.m_origin.m128_f32[i] < MinB.m128_f32[i])
			{
				_hit.m128_f32[i] = MinB.m128_f32[i];
				inside = false;

				// Calculate T distances to candidate planes
				if(IR(_ray.m_direction.m128_f32[i]))
					MaxT.m128_f32[i] = (MinB.m128_f32[i] - _ray.m_origin.m128_f32[i]) / _ray.m_direction.m128_f32[i];
			}
			else if(_ray.m_origin.m128_f32[i] > MaxB.m128_f32[i])
			{
				_hit.m128_f32[i] = MaxB.m128_f32[i];
				inside = false;

				// Calculate T distances to candidate planes
				if(IR(_ray.m_direction.m128_f32[i]))
					MaxT.m128_f32[i] = (MaxB.m128_f32[i] - _ray.m_origin.m128_f32[i]) / _ray.m_direction.m128_f32[i];
			}
		}

		// Ray origin inside bounding box
		if(inside)
		{
			_hit = _ray.m_origin;
			return true;
		}

		// Get largest of the maxT's for final choice of intersection
		unsigned int whichPlane = 0;
		if(MaxT.m128_f32[1] > MaxT.m128_f32[whichPlane])	
			whichPlane = 1;
		if(MaxT.m128_f32[2] > MaxT.m128_f32[whichPlane])	
			whichPlane = 2;

		// Check final candidate actually inside box
		if(IR(MaxT.m128_f32[whichPlane])&0x80000000) 
			return false;

		for(unsigned int i=0;i<3;i++)
		{
			if(i!=whichPlane)
			{
				_hit.m128_f32[i] = _ray.m_origin.m128_f32[i] + MaxT.m128_f32[whichPlane] * _ray.m_direction.m128_f32[i];
	//#ifdef RAYAABB_EPSILON
		//		if(coord.m[i] < MinB.m[i] - RAYAABB_EPSILON || coord.m[i] > MaxB.m[i] + RAYAABB_EPSILON)	return false;
	//#else
				if(_hit.m128_f32[i] < MinB.m128_f32[i] || _hit.m128_f32[i] > MaxB.m128_f32[i])
					return false;
	//#endif
			}
		}
		return true;
	}

	bool snRaycast::RayHeightmap(const snRay& _ray, const snHeightMap& _hmap, snVec& _hit)
	{
		//Check ray vs AABB
		const snAABB& bb = _hmap.getBoundingVolume();

		snVec hit;
		bool res = RayAABB(_ray, bb, hit);
		if(!res)
		{
			return false;
		}

		//Do not delete. Check all the quads one by one. Helpfull to debug.
		/*unsigned int rawResX = 512;
		unsigned int rawResY = 512;
		{
			float minSqDistance = SN_FLOAT_MAX;
			res = false;
			for(unsigned int x = 0; x < _hmap.getWidth(); ++x)
			{
				for(unsigned int y = 0; y < _hmap.getLength(); ++y)
				{
					snVec hit;
					if(RayHeightmapQuad(_ray, _hmap, x, y, hit))
					{
						float sqDistance = snVec3SquaredNorme(_ray.m_origin - hit);
						if(sqDistance < minSqDistance)
						{
							_hit = hit;
							minSqDistance = sqDistance;
							rawResX = x;
							rawResY = y;

						}
					}
				}
			}
		}*/

		snVec secondPoint = _ray.m_origin + _ray.m_direction;
		float sign = _ray.m_origin.m128_f32[VEC_ID_X] <= secondPoint.m128_f32[VEC_ID_X] ? 1.f : -1.f;

		//Compute the slope of the line
		float dx = secondPoint.m128_f32[VEC_ID_X] - _ray.m_origin.m128_f32[VEC_ID_X];
		float dy = secondPoint.m128_f32[VEC_ID_Z] - _ray.m_origin.m128_f32[VEC_ID_Z];

		//Get the first quad
		unsigned int x, y;
		res = _hmap.getOverlapQuad(_ray.m_origin, x, y);
		if(!res)
		{
			return false;
		}

		//Those are special cases
		const float DELTA_EPSILON = 0.05f;
		float absDX = abs(dx);
		float absDY = abs(dy);
		if(absDX < DELTA_EPSILON && absDY < DELTA_EPSILON) // there is only one quad
		{
			return RayHeightmapQuad(_ray, _hmap, x, y, _hit);
		}
		else if(absDX < DELTA_EPSILON) //vertical line
		{
			int inc = dy > 0 ? 1 : -1;
			while(_hmap.isValidQuad(x, y))
			{
				//Check for intersection with the quad
				if(RayHeightmapQuad(_ray, _hmap, x, y, _hit))
				{
					return true;
				}

				//Compute next value
				y += inc;
			}

			return false;
		}
		else if(absDY < DELTA_EPSILON) //dy == 0 horizontal line
		{
			int inc = dx > 0 ? 1 : -1;
			while(_hmap.isValidQuad(x, y))
			{
				//Check for intersection with the quad
				if(RayHeightmapQuad(_ray, _hmap, x, y, _hit))
				{
					return true;
				}

				//Compute next value
				x += inc;
			}

			return false;
		}

		//Now we are on generic cases.
		//Look for the next quad
		float m = dy / dx;
		unsigned int offset = 0;

		if(abs(m) <= 1)
		{
			dx = sign * _hmap.getQuadSize();
			dy = m * sign * _hmap.getQuadSize();
			offset = 1;
		}
		else
		{
			dx = (1.f / abs(m)) * _hmap.getQuadSize() * sign;
			dy = m < 0 ? -_hmap.getQuadSize() : _hmap.getQuadSize();
			dy *= sign;
			offset = 0;
		}

		float fx = _ray.m_origin.m128_f32[VEC_ID_X] - bb.m_min.m128_f32[VEC_ID_X];
		float fy = _ray.m_origin.m128_f32[VEC_ID_Z] - bb.m_min.m128_f32[VEC_ID_Z];
		unsigned int previousX = x;
		unsigned int previousY = y;

		while(_hmap.isValidQuad(x, y))
		{
			if(previousX != x && previousY != y) //Going in diagonal
			{
				if(_hmap.isValidQuad(x, previousY))
				{
					if(RayHeightmapQuad(_ray, _hmap, x, previousY, _hit))
					{
						return true;
					}
				}
				if(_hmap.isValidQuad(previousX, y))
				{
					if(RayHeightmapQuad(_ray, _hmap, previousX, y, _hit))
					{
						return true;
					}
				}

				if(_hmap.isValidQuad(previousX, previousY))
				{
					if(RayHeightmapQuad(_ray, _hmap, previousX, previousY, _hit))
					{
						return true;
					}
				}
			}

			//Check for intersection with the quad
			if(RayHeightmapQuad(_ray, _hmap, x, y, _hit))
			{
				return true;
			}
			
			//save previous values
			previousX = x;
			previousY = y;

			//Compute next quad
			fx += dx;
			fy += dy;

			x = (unsigned int)(fx / _hmap.getQuadSize());
			y = (unsigned int)(fy / _hmap.getQuadSize());
		}

		return false;
	}

	bool snRaycast::RayTriangle(const snRay& _ray, const snVec& _v1, const snVec& _v2, const snVec& _v3, snVec& _hit)
	{
		//Implementation of the Moller Trumbore intersection algorithm
		//http://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm

		//Find vectors for two edges sharing V1
		snVec e1 = _v2 - _v1;
		snVec e2 = _v3 - _v1;
		
		//Begin calculating determinant - also used to calculate u parameter
		snVec P = snVec3Cross(_ray.m_direction, e2);

		//if determinant is near zero, ray lies in plane of triangle
		float det = snVec4GetX(snVec3Dot(e1, P));

		//NOT CULLING
		const float EPSILON = 0.000001f;
		if(det > -EPSILON && det < EPSILON) 
		{
			return false;
		}
		float inv_det = 1.f / det;
 
		//calculate distance from V1 to ray origin
		snVec T = _ray.m_origin - _v1;
 
		//Calculate u parameter and test bound
		float u = snVec4GetX(snVec3Dot(T, P)) * inv_det;

		//The intersection lies outside of the triangle
		if(u < 0.f || u > 1.f) 
		{
			return false;
		}

		//Prepare to test v parameter
		snVec Q = snVec3Cross(T, e1);
 
		//Calculate V parameter and test bound
		float v = snVec4GetX(snVec3Dot(_ray.m_direction, Q)) * inv_det;

		//The intersection lies outside of the triangle
		if(v < 0.f || u + v  > 1.f) 
		{
			return false;
		}
 
		float t = snVec4GetX(snVec3Dot(e2, Q)) * inv_det;
 
		if(t > EPSILON) //ray intersection
		{ 
			_hit = _ray.m_origin + _ray.m_direction * t;
			return true;
		}
 
		// No hit, no win
		return false;
	}

	bool snRaycast::RayHeightmapQuad(const snRay& _ray, const snHeightMap& _hmap, unsigned int _x, unsigned int _y, snVec& _hit)
	{
		const unsigned int TRIANGLE_COUNT = 2;
		unsigned int triangleIds[TRIANGLE_COUNT];
		_hmap.getTriangleIds(_x, _y, triangleIds);

		for(unsigned int i = 0; i < TRIANGLE_COUNT; ++i)
		{
			snVec vertices[3];
			_hmap.getTriangle(triangleIds[i], vertices);

			//Check for intersection between the ray and the triangle
			bool res = RayTriangle(_ray, vertices[0], vertices[1], vertices[2], _hit);
			if(res) //Return the first intersection
			{
				return true;
			}
		}

		return false;
	}
}