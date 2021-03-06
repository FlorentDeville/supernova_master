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

#ifndef SN_HEIGHT_MAP_H
#define SN_HEIGHT_MAP_H

#include "snICollider.h"
#include "snAABB.h"

namespace Supernova
{
	//Height map.
	//Triangles indices start from the bottom left corner to the right top corner. For each quad, the top left triangle id is 
	// equal to the bottom left id + m_width
	//		   _____ _____ _____ _____ 
	//		  |9   /|11  /|13  /|15  /|
	//	row 1 |  / 8|  /10|  /12|  /14|		width = 4
	//	      |/____|/____|/____|/____|		length = 2
	//		  |1   /|3   /|5   /|7   /|
	//	row 0 |  / 0|  / 2|  / 4|  / 6|
	//	      |/____|/____|/____|/____|
	//
	//		   col0   col1  col2  col3
	//
	class SN_ALIGN snHeightMap : public snICollider
	{
	private:
		//Size of a single quad
		float m_quadSize;

		//Number of quads on the x axis
		unsigned int m_width;

		//Number of quads on the z axis
		unsigned int m_length;

		//The bounding volume of the height map
		snAABB m_boundingVolume;

	public:
		snHeightMap(const snVec& _min, const snVec& _max, float _quadSize, unsigned int _width, unsigned int _length);

		virtual ~snHeightMap();

#pragma region snICollider Interface
		//////////////////////////////////////////////////////////////////////
		// ICollider Interface												//
		//////////////////////////////////////////////////////////////////////

		//Initialize the height map. Doesn't do anything
		void initialize();

		//Update the collider using the transform.
		//The height map is only static so this method doesn't do anything.
		void updateFromTransform();

		//Compute the inertia tensor in a local frame.
		//The height map can only be used in a static actor so this method return a zero matrix.
		void computeLocalInertiaTensor(float _mass, snMatrix44f& _inertiaTensor) const;

		//Compute the bounding volume for this collider.
		//The height map is static so it actually get the AABB and doesn't compute it.
		void computeAABB(snAABB * const _boundingVolume) const;

#pragma endregion

		//Fill in the array _triangle with the snVec making the triangle in position _x and _y in the height map.
		/*virtual void getTriangle(unsigned int _x, unsigned int _y, snVec* _triangle) const = 0;*/
		virtual void getTriangle(unsigned int _id, snVec* _triangle) const = 0;

		//Return the normal of the triangle in position _x and _y.
		virtual snVec getNormal(unsigned int _triangleId) const = 0;

		//Return the ids of all the triangles overlaping the AABB.
		int getOverlapTriangles(const snAABB& _bounding, unsigned int* const _ids, unsigned int _maxTriangles) const;

		//Return the ids of the triangles overlaping a point. It returns a maximum of two triangles.
		// _point : the point to check.
		// _ids : an array of triangle ids.
		// _maxTriangles : the maximum number of element the array can contains.
		// return : the number of triangles found.
		int getOverlapTriangles(const snVec& _point, unsigned int* const _ids, unsigned int _maxTriangles) const;

		//Return the AABB making the bounding volume of the height map.
		snAABB const & getBoundingVolume() const;

		//Return the size of the quads making the height map.
		float getQuadSize() const;

		//Return the number of quads per row.
		unsigned int getLength() const;

		//Return the number of quads per column.
		unsigned int getWidth() const;

		//Find the coordinates of the quads overlapping a given point. This function finds the quads directly above or under a point.
		// _point : the point to use to find a quad.
		// _x : if a quad is found, get the x coordinate of the quad.
		// _y : if a quad is found, get the y coordinate of the quad.
		// return : True if a quad is found. False otherwise.
		bool getOverlapQuad(const snVec& _point, unsigned int& _x, unsigned int& _y) const;

		//Retrieve the ids of triangles making a quad.
		// _x : x coordinate of the quad.
		// _y : y coordinate of the quad.
		// _ids : an array of minimum 2 unsigned int containing the ids of the triangle making the quad.
		// remarks : There is no check for out of bounds coordinate.
		void getTriangleIds(unsigned int _x, unsigned int y, unsigned int* const _ids) const;

		//Check if the coordinates correpsond to a quads of the height map.
		// _x : the x coordinate of a quad.
		// _y : the y coordinate of a quad.
		// return : True if the coordinate correspond to a quad. False otherwise.
		bool isValidQuad(unsigned int _x, unsigned int _y) const;
	};
}

#endif //ifndef SN_HEIGHT_MAP_H