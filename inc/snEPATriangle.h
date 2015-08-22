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
#ifndef SN_EPA_TRIANGLE
#define SN_EPA_TRIANGLE

#include "snEPAEdge.h"
#include "snVec.h"

namespace Supernova
{
	class snSimplex;

	//Represent a triangle in the EPA simplex.
	//The triangle is defined by :
	//  - 3 vertex ids. Those are the id of the vertices in the vertex buffer.
	//  - 3 adjacent edges. Each edge is linked to an adjacent edge of another triangle. The edge with id i start from vertex i 
	//			and ends to (i+1)%3.
	class SN_ALIGN snEPATriangle
	{
	private:

		//Closest point to the origin
		snVec m_closestPoint;

		//The squared distance to the origin
		float m_sqDistance;

		//Array of ids of the three vertices making this triangle
		unsigned int m_verticesId[3];

		//Array of adjacent edges. Each adjacent edges belong to another triangle.
		//The edge with id i links vertices i and (i+1)%3
		snEPAEdge m_adjacentEdges[3];
		
		//Flag to indicate if this triangle has to be considered a part of the simplex.
		bool m_isObsolete;

	public:
		snEPATriangle();

		snEPATriangle(unsigned int _idVertex0, unsigned int _idVertex1, unsigned int _idVertex2);

		~snEPATriangle();

		//Return the vertex buffer id of the vertex _id in the triangle.
		unsigned int getVertexId(unsigned int _id) const;

		//True if the triangle is not a part of the simplex. False otherwise.
		bool getIsObsolete() const; 

		//Return an edge identified by its id.
		snEPAEdge& getAdjacentEdge(unsigned int _id);

		//Return the squared distance of the triangle to the origin
		float getSqDistance() const;

		//Return the closest point to the origin
		snVec getClosestPoint() const;

		//Set the edge _adjacent adjacents to the edge with id _id of the current triangle.
		//Return true if the edge was added. 
		//Return false if the edges are not adjacent (vertices ids of _adjacent and the edge _id do not match).
		bool setAdjacentEdge(const snEPAEdge& _adjacent, unsigned int _id);

		//Set the adjacent edge of _trangle with id _adjacentEdgeId as an adjacent edge of the edge of the current triangle with id _edgeId
		bool setAdjacentEdge(snEPATriangle* _triangle, unsigned int _adjacentEdgeId, unsigned int _edgeId);

		//Set the obsolete flag of the triangle.
		void setIsObsolete(bool _isObsolete);

		//Return true if the point _v is visible from the current triangle. Return false otherwise.
		bool isVisibleFromVertex(const snSimplex& _simplex, unsigned int _id);

		//Insert the new point in the simplex in the current triangle and make sure the simplex stays convex.
		bool quickHull(snSimplex& _simplex, unsigned int _id);

		bool computeClosestPointToOrigin(const snSimplex& _simplex);
	};
}

#endif //ifndef SN_EPA_TRIANGLE