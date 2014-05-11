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

using namespace Supernova::Vector;

namespace Supernova
{
	snSimplex::snSimplex() : m_vertexBuffer(), m_indexBuffer(), m_triangleCount(0), m_validTriangle(){}

	snSimplex::~snSimplex(){}

	int snSimplex::addVertex(const snVec& _v)
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

	void snSimplex::expand(const snVec& _v, int _triangleId)
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

	void snSimplex::getTriangle(int _triangleId, snVec& _v1, snVec& _v2, snVec& _v3) const
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

	void snSimplex::computeTriangleClosestToOrigin(int& _triangleId, snVec& _normal, float& _distance) const
	{
		_distance = SN_FLOAT_MAX;

		//loop through each triangle to find the closest triangle to the origin
		for (int triangleId = 0; triangleId < m_triangleCount; ++triangleId)
		{
			//this triangle can't be the closest to the origin
			if (!m_validTriangle[triangleId])
				continue;

			//get the triangle
			snVec vertices[3];
			getTriangle(triangleId, vertices[0], vertices[1], vertices[2]);

			//compute the triangle normal going to the origin
			snVec e1 = vertices[0] - vertices[2];
			snVec e2 = vertices[1] - vertices[2];
			snVec n = snVec3Cross(e1, e2);
			snVec3Normalize(n);

			//compute the distance to the origin
			snVec AO = snVec4Set(0, 0, 0, 1) - vertices[0];
			float distance = snVec3Dot(n, AO);

			int side = sign(distance);
			distance *= side;
			n = n * (float)side;

			//the distance is greater
			if (distance >= _distance)
			{
				m_validTriangle[triangleId] = false;
				continue;
			}
			
			computeClosestPointToOriginInTriangle(triangleId, AO);
			distance = snVec3Norme(AO);
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
				snVec3Normalize(AO);
				_normal = AO * -1;
			}
		}
	}

	void snSimplex::computeClosestPointToOriginInTriangle(int _triangleId, snVec& _closestPoint) const
	{
		int index = _triangleId * 3;
		snVec a = m_vertexBuffer[m_indexBuffer[index]];
		snVec b = m_vertexBuffer[m_indexBuffer[index + 1]];
		snVec c = m_vertexBuffer[m_indexBuffer[index + 2]];

		snVec ab = b - a;
		snVec ac = c - a;
		snVec ap = snVec4Set(0, 0, 0, 1) - a;
		float d1 = snVec3Dot(ab, ap);
		float d2 = snVec3Dot(ac, ap);
		if (d1 <= 0.f && d2 <= 0.f)
		{
			_closestPoint = a;
			return;
		}

		snVec bp = snVec4Set(0, 0, 0, 1) - b;
		float d3 = snVec3Dot(ab, bp);
		float d4 = snVec3Dot(ac, bp);
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

		snVec cp = snVec4Set(0, 0, 0, 1) - c;
		float d5 = snVec3Dot(ab, cp);
		float d6 = snVec3Dot(ac, cp);
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

	snVec snSimplex::computeClosestPointForSegment(const snVec& _e1, const snVec& _e2, const snVec& _p) const
	{
		snVec edge = _e2 - _e1;
		float length = snVec3Norme(edge);
		snVec3Normalize(edge);

		snVec e1p = _p - _e1;

		float distance = snVec3Dot(edge, e1p);
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