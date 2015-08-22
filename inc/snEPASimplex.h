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

#ifndef SN_SIMPLEX_H
#define SN_SIMPLEX_H

#include "snEPATriangle.h"
#include "snVec.h"

namespace Supernova
{
	const unsigned int MAX_TRIANGLE_COUNT = 200;
	const unsigned int MAX_VERTEX_COUNT = 150;

	//Represent a simplex as produced by the GJK algorithm.
	class SN_ALIGN snSimplex
	{
	private:

		//List of vertices making the simplex
		snVec m_vertexBuffer[MAX_VERTEX_COUNT];

		//List of triangles.
		snEPATriangle m_triangles[MAX_TRIANGLE_COUNT];

		//Numbero of triangles currently in the simplex
		unsigned int m_trianglesCount;

		//Number of vertices currently in the simplex.
		unsigned int m_vertexCount;

	public:

		//Constructor
		snSimplex();

		//Destructor
		virtual ~snSimplex();

		//Return the vertex with id _id.
		snVec getVertex(unsigned int _id) const;

		//Return the number of vertices in the simplex
		unsigned int getVertexCount() const;

		//Return the triangle with id _id
		snEPATriangle* getTriangle(unsigned int _id);

		//Return the number of triangle in the simplex
		unsigned int getTriangleCount() const;

		//Add a new vertex to the vertex buffer and return the id of the new vertex.
		unsigned int addVertex(const snVec& _vertex);

		//Add a triangle to the simplex maed with the three vertices with ids _vertexId0, _vertexId1, _vertexId2
		snEPATriangle* addTriangle(unsigned int _vertexId0, unsigned int _vertexId1, unsigned int _vertexId2);

		//Link two edge1 of triangle1 to edge2 of triangle2.
		bool addLink(snEPATriangle* triangle1, unsigned int _edgeId1, snEPATriangle* triangle2, unsigned int _edgeId2);

		//Return a pointer to the closest triangle to the origin
		snEPATriangle* getClosestTriangleToOrigin();

		//Overridden new operator to create scene with correct alignement.
		void* operator new(size_t _count);

		//Overridden delete operator to delete using the correct alignement.
		void operator delete(void* _p);
	};
}
#endif