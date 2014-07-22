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

#ifndef TERRAIN_COLLIDER_H
#define TERRAIN_COLLIDER_H

#include "snHeightMap.h"
using namespace Supernova;


namespace Devil
{
	class GfxEntityHeightMap;

	class TerrainCollider : public snHeightMap
	{
	private:
		//Vertex buffer
		snVec* m_vertices;

		//Normal of triangles
		snVec* m_triangleNormal;

		//Index buffer
		unsigned long* m_indices;

		//Number of vertex
		unsigned int m_vertexCount;

		//Number of index
		unsigned int m_indexCount;

	public:
		TerrainCollider(const snVec& _min, const snVec& _max, float _quadSize, unsigned int _width, unsigned int _length,
			float* _heights);

		~TerrainCollider();

		//Fill in the array _triangle with the snVec making the triangle in position _x and _y in the height map.
		void getTriangle(unsigned int _id, snVec* _triangle) const;

		//Return the normal of the triangle in position _x and _y.
		snVec getNormal(unsigned int _triangleId) const;
	};
}

#endif //ifndef TERRAIN_COLLIDER_H