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

#ifndef SN_GJK_H
#define SN_GJK_H

#include "snFeatureClipping.h"

namespace Supernova
{
	class snICollider;
	class snCollisionResult;
	class snSimplex;
	class snIGJKCollider;

	//Provide functionnalty to check collision between colliders using GJK algorithm.
	class snGJK
	{
	private:
		snFeatureClipping m_clipping;

	public:
		//Default constructor
		snGJK();

		//Default destructor
		virtual ~snGJK();

		//Check if two colliders are intersecting.
		snCollisionResult queryIntersection(const snIGJKCollider& _c1, const snIGJKCollider& _c2) const;

	private:
		//Compute the point from the minkowski difference in a particular direction for two colliders
		snVec support(const snIGJKCollider& _c1, const snIGJKCollider& _c2, const snVec& _direction) const;

		//Check if a simplex contains the origin. If the simplex does not contain the origin, the simplex and the direction are updated.
		bool doSimplex(snVec* const _simplex, int& _simplexCount, snVec& _direction) const;

		//Check a simplex made of two points (a line).
		bool checkOneSimplex(snVec* const _simplex, int& _simplexCount, snVec& _direction) const;

		//Check a simplex made of three points (a triangle)
		bool checkTwoSimplex(snVec* const _simplex, int& _simplexCount, snVec& _direction) const;

		//Check a simplex made of four points (tetrahedron).
		bool checkThreeSimplex(snVec* const _simplex, int& _simplexCount, snVec& _direction) const;

		//Expand a simplex and return the collision normal.
		snVec expandPolytope(snSimplex& _simplex, const snIGJKCollider& _c1, const snIGJKCollider& _c2) const;

		bool expandPolytopeV2(snSimplex& _simplex, const snIGJKCollider& _c1, const snIGJKCollider& _c2, snVec& _normal) const;
	};
}

#endif //SN_GJK_H