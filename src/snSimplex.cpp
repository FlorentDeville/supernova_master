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
	snSimplex::snSimplex() : m_vertexBuffer(), m_indexBuffer(), m_triangleCount(0), m_validTriangle(){}

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
		m_validTriangle.push_back(true);
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
		m_validTriangle.push_back(true);

		//N 1 2
		m_indexBuffer.push_back(idNewVertex);
		m_indexBuffer.push_back(m_indexBuffer[indexId + 1]);
		m_indexBuffer.push_back(m_indexBuffer[indexId + 2]);
		m_validTriangle.push_back(true);

		//update the existing triangle 0 N 2
		m_indexBuffer[indexId + 1] = idNewVertex;
		m_validTriangle[_triangleId] = true;
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

	void snSimplex::getTriangle(int _triangleId, int& _id1, int& _id2, int& _id3) const
	{
		int indexBufferId = _triangleId * 3;

		_id1 = m_indexBuffer[indexBufferId];
		_id2 = m_indexBuffer[indexBufferId + 1];
		_id3 = m_indexBuffer[indexBufferId + 2];
	}

	void snSimplex::computeTriangleClosestToOrigin(int& _triangleId, snVector4f& _normal, float& _distance) const
	{
		_distance = SN_FLOAT_MAX;

		//loop through each triangle to find the closest triangle to the origin
		for (int triangleId = 0; triangleId < m_triangleCount; ++triangleId)
		{
			//this triangle can't be the closest to the origin
			if (!m_validTriangle[triangleId])
				continue;

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
			{
				m_validTriangle[triangleId] = false;
				continue;
			}
			
			computeClosestPointToOriginInTriangle(triangleId, AO);
			distance = AO.norme();
			if (distance >= _distance)
			{
				m_validTriangle[triangleId] = false;
				continue;
			}

			//the distance is smaller so save the triangle
			_distance = distance;
			_triangleId = triangleId;

			if (_distance <= 0.0001f)
				_normal = n;
			else
			{
				AO.normalize();
				_normal = AO * -1;
			}
		}
	}

	void snSimplex::computeClosestPointToOriginInTriangle(int _triangleId, snVector4f& _closestPoint) const
	{
		int index = _triangleId * 3;
		snVector4f a = m_vertexBuffer[m_indexBuffer[index]];
		snVector4f b = m_vertexBuffer[m_indexBuffer[index + 1]];
		snVector4f c = m_vertexBuffer[m_indexBuffer[index + 2]];

		snVector4f ab = b - a;
		snVector4f ac = c - a;
		snVector4f ap = snVector4f(0, 0, 0, 1) - a;
		float d1 = ab.dot(ap);
		float d2 = ac.dot(ap);
		if (d1 <= 0.f && d2 <= 0.f)
		{
			_closestPoint = a;
			return;
		}

		snVector4f bp = snVector4f(0, 0, 0, 1) - b;
		float d3 = ab.dot(bp);
		float d4 = ac.dot(bp);
		if (d3 >= 0.f && d4 <= d3)
		{
			_closestPoint = b;
			return;
		}

		float vc = d1 * d4 - d3 * d2;
		if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f)
		{
			float v = d1 / (d1 - d3);
			_closestPoint = a + ab * v;
			return;
		}

		snVector4f cp = snVector4f(0, 0, 0, 1) - c;
		float d5 = ab.dot(cp);
		float d6 = ac.dot(cp);
		if (d6 >= 0.f && d5 <= d6)
		{
			_closestPoint = c;
			return;
		}

		float vb = d5 * d2 - d1 * d6;
		if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f)
		{
			float w = d2 / (d2 - d6);
			_closestPoint = a + ac * w;
			return;
		}

		float va = d3 * d6 - d5 * d4;
		if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f)
		{
			float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
			_closestPoint = b + (c - b) * w;
			return;
		}

		float denom = 1.f / (va + vb + vc);
		float v = vb * denom;
		float w = vc * denom;
		_closestPoint = a + ab * v + ac * w;
	}

	snVector4f snSimplex::computeClosestPointForSegment(const snVector4f& _e1, const snVector4f& _e2, const snVector4f& _p) const
	{
		snVector4f edge = _e2 - _e1;
		float length = edge.norme();
		edge.normalize();

		snVector4f e1p = _p - _e1;

		float distance = edge.dot(e1p);
		distance = clamp(distance, 0.f, length);

		return _e1 + (edge * distance);
	}

	void snSimplex::setTriangleValidity(int _triangleId, bool _isValid)
	{
		m_validTriangle[_triangleId] = _isValid;
	}

	void* snSimplex::operator new(size_t _count)
	{
		return _aligned_malloc(_count, SN_ALIGN_SIZE);
	}

	void snSimplex::operator delete(void* _p)
	{
		_aligned_free(_p);
	}
}