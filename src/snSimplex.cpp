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

#include "snSimplex.h"
#include "snMath.h"

namespace Supernova
{
	snSimplex::snSimplex() : m_vertexBuffer(), m_indexBuffer(), m_triangleCount(0){}

	snSimplex::~snSimplex(){}

	int snSimplex::addVertex(const snVector4f& _v)
	{
		int id = m_vertexBuffer.size();
		m_vertexBuffer.push_back(_v);
		return id;
	}

	int snSimplex::addTriangle(int _vertexId1, int _vertexId2, int _vertexId3)
	{
		m_indexBuffer.push_back(_vertexId1);
		m_indexBuffer.push_back(_vertexId2);
		m_indexBuffer.push_back(_vertexId3);
		return m_triangleCount++;
	}

	void snSimplex::expand(const snVector4f& _v, int _triangleId)
	{
		//add the vertex
		int idNewVertex = addVertex(_v);

		//get the first index buffer of the existing triangle to expand
		int indexId = _triangleId * 3;

		//create two new triangles N 0 1
		m_indexBuffer.push_back(idNewVertex);
		m_indexBuffer.push_back(m_indexBuffer[indexId]);
		m_indexBuffer.push_back(m_indexBuffer[indexId + 1]);

		//N 1 2
		m_indexBuffer.push_back(idNewVertex);
		m_indexBuffer.push_back(m_indexBuffer[indexId + 1]);
		m_indexBuffer.push_back(m_indexBuffer[indexId + 2]);

		//update the existing triangle 0 N 2
		m_indexBuffer[indexId + 1] = idNewVertex;

		////N10
		//m_indexBuffer.push_back(idNewVertex);
		//m_indexBuffer.push_back(m_indexBuffer[indexId + 1]);
		//m_indexBuffer.push_back(m_indexBuffer[indexId]);

		////N21
		//m_indexBuffer.push_back(idNewVertex);
		//m_indexBuffer.push_back(m_indexBuffer[indexId + 2]);
		//m_indexBuffer.push_back(m_indexBuffer[indexId + 1]);

		////2N0
		//m_indexBuffer[indexId + 1] = m_indexBuffer[indexId + 2];
		//m_indexBuffer[indexId + 2] = idNewVertex;

		m_triangleCount += 2;
	}

	int snSimplex::getTriangleCount() const
	{
		return m_triangleCount;
	}

	void snSimplex::getTriangle(int _triangleId, snVector4f& _v1, snVector4f& _v2, snVector4f& _v3) const
	{
		int indexBufferId = _triangleId * 3;

		_v1 = m_vertexBuffer[m_indexBuffer[indexBufferId]];
		_v2 = m_vertexBuffer[m_indexBuffer[indexBufferId + 1]];
		_v3 = m_vertexBuffer[m_indexBuffer[indexBufferId + 2]];
	}

	void snSimplex::computeTriangleClosestToOrigin(int& _triangleId, snVector4f& _normal, float& _distance) const
	{
		_distance = SN_FLOAT_MAX;

		//loop through each triangle to find the closest triangle to the origin
		for (int triangleId = 0; triangleId < m_triangleCount; ++triangleId)
		{
			//get the triangle
			snVector4f vertices[3];
			getTriangle(triangleId, vertices[0], vertices[1], vertices[2]);

			//compute the triangle normal going to the origin
			snVector4f e1 = vertices[0] - vertices[2];
			snVector4f e2 = vertices[1] - vertices[2];
			snVector4f n = e1.cross(e2);
			n.normalize();

			//compute the distance to the origin
			snVector4f AO = snVector4f(0, 0, 0, 1) - vertices[0];
			float distance = n.dot(AO);

			int side = sign(distance);
			distance *= side;
			n = n * side;

			//the distance is greater
			if (distance >= _distance)
				continue;

			//the distance is smaller so save the triangle
			_distance = distance;
			_triangleId = triangleId;
			_normal = n;
		}
	}
}