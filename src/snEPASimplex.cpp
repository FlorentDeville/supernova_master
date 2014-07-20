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

#include "snEPASimplex.h"
#include "snMath.h"

#include <malloc.h>
#include <assert.h>
#include <new>
using namespace Supernova::Vector;

namespace Supernova
{
	snSimplex::snSimplex()
	{
		m_trianglesCount = 0;
		m_vertexCount = 0;
	}

	snSimplex::~snSimplex(){}

	snVec snSimplex::getVertex(unsigned int _id) const
	{
		return m_vertexBuffer[_id];
	}

	unsigned int snSimplex::getVertexCount() const
	{
		return m_vertexCount;
	}

	snEPATriangle* snSimplex::getTriangle(unsigned int _id)
	{
		return (&m_triangles[_id]);
	}

	unsigned int snSimplex::getTriangleCount() const
	{
		return m_trianglesCount;
	}

	unsigned int snSimplex::addVertex(const snVec& _vertex)
	{
		assert(m_vertexCount < MAX_VERTEX_COUNT);
		m_vertexBuffer[m_vertexCount] = _vertex;
		return m_vertexCount++;
	}

	snEPATriangle* snSimplex::addTriangle(unsigned int _vertexId0, unsigned int _vertexId1, unsigned int _vertexId2)
	{
		if (m_trianglesCount >= MAX_TRIANGLE_COUNT)
			return 0;

		snEPATriangle* newTriangle = &m_triangles[m_trianglesCount++];

		::new (newTriangle)snEPATriangle(*this, _vertexId0, _vertexId1, _vertexId2);
		if (!newTriangle->computeClosestPointToOrigin(*this))
		{
			--m_trianglesCount;
			return 0;
		}
		
		return newTriangle;
	}

	bool snSimplex::addLink(snEPATriangle* triangle1, unsigned int _edgeId1, snEPATriangle* triangle2, unsigned int _edgeId2)
	{
		if (!triangle1->setAdjacentEdge(triangle2, _edgeId2, _edgeId1))
			return false;

		if (!triangle2->setAdjacentEdge(triangle1, _edgeId1, _edgeId2))
			return false;

		return true;
	}

	snEPATriangle* snSimplex::getClosestTriangleToOrigin()
	{
		snEPATriangle* closest = 0;
		float sqDistance = SN_FLOAT_MAX;

		for (unsigned int i = 0; i < m_trianglesCount; ++i)
		{
			snEPATriangle& tri = m_triangles[i];

			//Ignore obsolete triangle
			if (tri.getIsObsolete())
				continue;

			float triDistance = tri.getSqDistance();
			
			if (!(triDistance >= 0.f))
				continue;

			if (triDistance < sqDistance)
			{
				closest = &tri;
				sqDistance = triDistance;
			}
		}

		return closest;
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