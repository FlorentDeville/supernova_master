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
#include "snEPATriangle.h"
#include "snEPASimplex.h"
#include "snClosestPoint.h"

#include <assert.h>
#include <new>

using namespace Supernova::Vector;

namespace Supernova
{
	snEPATriangle::snEPATriangle()
	{
		m_verticesId[0] = 0;
		m_verticesId[1] = 0;
		m_verticesId[2] = 0;
		m_isObsolete = false;
	}

	snEPATriangle::snEPATriangle(unsigned int _idVertex0, unsigned int _idVertex1, unsigned int _idVertex2)
	{
		m_verticesId[0] = _idVertex0;
		m_verticesId[1] = _idVertex1;
		m_verticesId[2] = _idVertex2;
		m_isObsolete = false;
	}

	snEPATriangle::~snEPATriangle()
	{}

	unsigned int snEPATriangle::getVertexId(unsigned int _id) const
	{
		assert(_id >= 0 && _id <= 2);
		return m_verticesId[_id];
	}

	bool snEPATriangle::getIsObsolete() const
	{
		return m_isObsolete;
	}

	snEPAEdge& snEPATriangle::getAdjacentEdge(unsigned int _id)
	{
		return m_adjacentEdges[_id];
	}

	float snEPATriangle::getSqDistance() const
	{
		return m_sqDistance;
	}

	snVec snEPATriangle::getClosestPoint() const
	{
		return m_closestPoint;
	}

	bool snEPATriangle::setAdjacentEdge(const snEPAEdge& _adjacent, unsigned int _id)
	{
		//Check if the vertices match between the two triangles
		bool valid = (m_verticesId[_id] == _adjacent.getEndVertexId()) && (m_verticesId[(_id + 1) % 3] == _adjacent.getStartVertexId());
		assert(valid);
		if (!valid)
			return false;

		//Set the edge
		m_adjacentEdges[_id] = _adjacent;
		return true;
	}

	bool snEPATriangle::setAdjacentEdge(snEPATriangle* _triangle, unsigned int _adjacentEdgeId, unsigned int _edgeId)
	{
		//Check if the vertices match between the two triangles
		bool valid = (m_verticesId[_edgeId] == _triangle->getVertexId((_adjacentEdgeId + 1) % 3)) 
			&& (m_verticesId[(_edgeId + 1) % 3] == _triangle->getVertexId(_adjacentEdgeId));

		if (!valid)
			return false;

		snEPAEdge* edge = &m_adjacentEdges[_edgeId];
		new (edge)snEPAEdge(_triangle, _adjacentEdgeId);
		return true;
	}

	void snEPATriangle::setIsObsolete(bool _isObsolete)
	{
		m_isObsolete = _isObsolete;
	}

	bool snEPATriangle::isVisibleFromVertex(const snSimplex& _simplex, unsigned int _id)
	{
		snVec closestToVert = _simplex.getVertex(_id) - m_closestPoint;
		return snVec4GetX(snVec3Dot(m_closestPoint, closestToVert)) > 0.0;
	}

	bool snEPATriangle::quickHull(snSimplex& _simplex, unsigned int _id)
	{
		//the new vertex is visible from the triangle (it's a precondition).
		setIsObsolete(true);

		//Save the number of triangle in the simplex
		unsigned int savedTriangleCount = _simplex.getTriangleCount();

		//Recursive call to apply quickhull to other triangles
		if (!m_adjacentEdges[0].quickHull(_simplex, _id))
			return false;

		if (!m_adjacentEdges[1].quickHull(_simplex, _id))
			return false;

		if (!m_adjacentEdges[2].quickHull(_simplex, _id))
			return false;

		unsigned int j = _simplex.getTriangleCount() - 1;
		for (unsigned int i = savedTriangleCount; i != _simplex.getTriangleCount(); j = i++)
		{
			//make the other halflinks
			snEPATriangle* newTriangle = _simplex.getTriangle(i);
			snEPATriangle* existingTriangle = newTriangle->getAdjacentEdge(1).getOwner();
			if (!existingTriangle->setAdjacentEdge(newTriangle, 1, newTriangle->getAdjacentEdge(1).getId()))
				return false;

			//make the two other links
			if (!_simplex.addLink(newTriangle, 0, _simplex.getTriangle(j), 2))
				return false;
		}

		return true;
	}

	bool snEPATriangle::computeClosestPointToOrigin(const snSimplex& _simplex)
	{
		m_closestPoint = snClosestPoint::PointTriangle(VEC_ZERO, _simplex.getVertex(m_verticesId[0]), _simplex.getVertex(m_verticesId[1]),
			_simplex.getVertex(m_verticesId[2]));
		m_sqDistance = snVec4GetX(snVec3Dot(m_closestPoint, m_closestPoint));

		return true;
	}
}