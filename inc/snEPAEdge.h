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
#ifndef SN_EPA_EDGE_H
#define SN_EPA_EDGE_H

namespace Supernova
{
	class snEPATriangle;
	class snSimplex;

	//Represent an edge in a triangle.
	//This edge is defined by a pointer to the triangle owner of the edge and an id. 
	//The id is between 0 and 2. An edge with id i links vertices i and (i+1)%3.
	class snEPAEdge
	{
	private:
		//Owner of this edge
		snEPATriangle* m_owner;

		//Id of this edge in the owner triangle.
		//The edge with id i linked the vertices i and (i+1)%3
		unsigned int m_id;

	public:
		snEPAEdge();

		snEPAEdge(snEPATriangle* _owner, unsigned int _id);

		~snEPAEdge();

		//Return a pointer to the EPATriangle owner of this edge
		snEPATriangle* getOwner() const;

		//Return the id of the current edge in its triangle.
		unsigned int getId() const;

		//Return the vertex buffer id of this edge's start point.
		unsigned int getStartVertexId() const;

		//Return the vertex buffer id of this edge's end point
		unsigned int getEndVertexId() const;

		//Check if this edge is part of the horizon of the vertex and update the simplex.
		void quickHull(snSimplex& _simplex, unsigned int _id);
	};
}

#endif //ifndef SN_EPA_EDGE_H