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

#include "TerrainCollider.h"

//#include "GfxEntityHeightMap.h"
using namespace Supernova::Vector;

//#include <VertexTypes.h>
//using namespace DirectX;

namespace Devil
{
	TerrainCollider::TerrainCollider(const snVec& _min, const snVec& _max, float _quadSize, unsigned int _width, unsigned int _length, 
		float* _heights) 
		: snHeightMap(_min, _max, _quadSize, _width, _length)
	{
		//compute how many vertices we have
		m_vertexCount = (_width + 1) * (_length + 1);
		m_vertices = (snVec*)_aligned_malloc(sizeof(snVec)* m_vertexCount, 16);

		//fill in the array of vertices
		int vertexId = 0;
		for (unsigned int row = 1; row <= _length+1; ++row) //loop through each row
		{
			snVec lowerLeftCorner = _min;
			snVec4SetY(lowerLeftCorner, 0);
			snVec offsetZ = lowerLeftCorner + snVec4Set(0, 0, _quadSize, 0) * (float)(row-1);

			unsigned int offsetVertexId = row * (_width + 3);
			for (unsigned int column = 1; column <= _width+1; ++column) //loop through each column
			{
				m_vertices[vertexId] = offsetZ + snVec4Set(_quadSize, 0, 0, 0) * (float)(column-1) + snVec4Set(0, _heights[offsetVertexId+column], 0, 0);
				++vertexId;
			}
		}

		//compute how many triangle and indices we need
		unsigned int triangleCount = _width * _length * 2;
		m_indexCount = triangleCount * 3;
		m_triangleNormal = (snVec*)_aligned_malloc(sizeof(snVec)* triangleCount, 16);

		//create the index buffer
		unsigned int indexId = 0;
		unsigned int triangleId = 0;
		m_indices = new unsigned long[m_indexCount];
		unsigned int vertexPerRow = _width + 1;
		for (unsigned int row = 0; row < _length; ++row) //loop through each row
		{
			unsigned int offsetId = row * vertexPerRow;
			for (unsigned int column = 0; column < _width; ++column)//loop through each column
			{
				//lower right triangle
				m_indices[indexId] = offsetId + column;
				m_indices[indexId + 1] = offsetId + column + 1;
				m_indices[indexId + 2] = offsetId + column + vertexPerRow + 1;

				//compute normals
				snVec v0 = m_vertices[m_indices[indexId]];
				snVec v1 = m_vertices[m_indices[indexId + 1]];
				snVec v2 = m_vertices[m_indices[indexId + 2]];

				snVec normal = snVec3Cross(v0 - v1, v2 - v1);
				snVec3Normalize(normal);
				m_triangleNormal[triangleId++] = normal;

				//upper left triangle
				indexId += 3;

				m_indices[indexId] = offsetId + column;
				m_indices[indexId + 1] = offsetId + column + vertexPerRow + 1;
				m_indices[indexId + 2] = offsetId + column + vertexPerRow;

				//compute normals
				v0 = m_vertices[m_indices[indexId]];
				v1 = m_vertices[m_indices[indexId + 1]];
				v2 = m_vertices[m_indices[indexId + 2]];

				normal = snVec3Cross(v0 - v1, v2 - v1);
				snVec3Normalize(normal);
				m_triangleNormal[triangleId++] = normal;

				indexId += 3;
			}
		}
	}

	TerrainCollider::~TerrainCollider()
	{
		_aligned_free(m_vertices);
		_aligned_free(m_triangleNormal);
		delete[] m_indices;
	}

	//Fill in the array _triangle with the snVec making the triangle in position _x and _y in the height map.
	void TerrainCollider::getTriangle(unsigned int _id, snVec* _triangle) const
	{
		unsigned int offset = _id * 3;
		_triangle[0] = m_vertices[m_indices[offset]];
		_triangle[1] = m_vertices[m_indices[offset + 1]];
		_triangle[2] = m_vertices[m_indices[offset + 2]];

		return;
	}

	//Return the normal of the triangle in position _x and _y.
	snVec TerrainCollider::getNormal(unsigned int _triangleId) const
	{
		return m_triangleNormal[_triangleId];
	}
}