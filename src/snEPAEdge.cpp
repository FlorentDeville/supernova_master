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
#include "snEPAEdge.h"

#include "snEPATriangle.h"
#include "snEPASimplex.h"

namespace Supernova
{
	snEPAEdge::snEPAEdge() : m_owner(0), m_id(0)
	{}

	snEPAEdge::snEPAEdge(snEPATriangle* _owner, unsigned int _id) : m_owner(_owner), m_id(_id)
	{}

	snEPAEdge::~snEPAEdge()
	{}

	snEPATriangle* snEPAEdge::getOwner() const
	{
		return m_owner;
	}

	unsigned int snEPAEdge::getId() const
	{
		return m_id;
	}

	unsigned int snEPAEdge::getStartVertexId() const
	{
		return m_owner->getVertexId(m_id);
	}

	unsigned int snEPAEdge::getEndVertexId() const
	{
		return m_owner->getVertexId((m_id + 1) % 3);
	}

	bool snEPAEdge::quickHull(snSimplex& _simplex, unsigned int _id)
	{
		//ignore obsolete triangles.
		if (m_owner->getIsObsolete())
			return true;

		if (m_owner->isVisibleFromVertex(_simplex, _id))
		{
			//If the current triangle is visible then it needs to be removed from the simplex
			m_owner->setIsObsolete(true);

			//check the other edges
			if (!m_owner->getAdjacentEdge((m_id + 1) % 3).quickHull(_simplex, _id))
				return false;

			if (!m_owner->getAdjacentEdge((m_id + 2) % 3).quickHull(_simplex, _id))
				return false;
		}
		else
		{
			//This edge is a part of the horizon. Create a new triangle.
			snEPATriangle* newTriangle = _simplex.addTriangle(_id, getEndVertexId(), getStartVertexId());
			if (newTriangle == 0)
				return false;;

			//Link the edge of the new triangle to the edge of the current triangle
			newTriangle->setAdjacentEdge(m_owner, m_id, 1);
		}

		return true;
	}
}