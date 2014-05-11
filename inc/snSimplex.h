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

#include "snTypes.h"

namespace Supernova
{
	//Represent a simplex as produced by the GJK algorithm.
	class SN_ALIGN snSimplex
	{
	private:
		//List of vertices making the simplex
		snVecVector m_vertexBuffer;

		//List of index making the triangles of the simplex.
		vector<int> m_indexBuffer;

		//The number of triangles in the simplex.
		int m_triangleCount;

		//Get or set if a triangle is valid to be the closest to the origin.
		mutable vector<bool> m_validTriangle;

	public:

		//Constructor
		snSimplex();

		//Destructor
		virtual ~snSimplex();

		//Add a vertex to the simplex and return its id.
		int addVertex(const snVec& _v);

		//Add a triangle to the simplex and return its id
		int addTriangle(int _vertexId1, int _vertexId2, int _vertexId3);

		//Expand a triangle identify with it's id with a new point. 
		void expand(const snVec& _v, int _triangleId);

		//Get the number of triangle in the simplex
		int getTriangleCount() const;

		//Get the vertices making a triangle identify by its id.
		void getTriangle(int _triangleId, snVec& _v1, snVec& _v2, snVec& _v3) const;

		void getTriangle(int _triangleId, int& _id1, int& _id2, int& _id3) const;

		//Compute what triangle is the closest to the origin and return its id and normal. The normal points to the origin.
		void computeTriangleClosestToOrigin(int& _triangleId, snVec& _normal, float& _distance) const;

		//Compute the closest point to the origin of a given triangle.
		void computeClosestPointToOriginInTriangle(int _triangleId, snVec& _closestPoint) const;

		snVec computeClosestPointForSegment(const snVec& _e1, const snVec& e2, const snVec& _p) const;

		void setTriangleValidity(int _triangleId, bool _isValid);

		//Overridden new operator to create scene with correct alignement.
		void* operator new(size_t _count);

		//Overridden delete operator to delete using the correct alignement.
		void operator delete(void* _p);
	};
}
#endif