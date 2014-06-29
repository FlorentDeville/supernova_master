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
		//The maximum number of iteration for the algorithm.
		static const int MAX_ITERATION = 50;

		snFeatureClipping m_clipping;

	public:
		//Default constructor
		snGJK();

		//Default destructor
		virtual ~snGJK();

		//Check if two colliders are intersecting.
		snCollisionResult queryIntersection(const snIGJKCollider& _c1, const snIGJKCollider& _c2) const;

		template<class T, class U> static bool gjkIntersect(const T& _a, const U& _b);

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

		static snVec updateSimplex(snVec* _s, int& _n);

		static snVec updateTwoSimplex(snVec* _s, int& _n);
		
		static snVec updateThreeSimplex(snVec* _s, int& _n);

		static snVec updateFourSimplex(snVec* _s, int& _n);
	};

	template<class T, class U> bool snGJK::gjkIntersect(const T& _a, const U& _b)
	{
		snVec support[4];
		
		//Start with an arbitrary point in the Minkowski set shape.
		support[0] = _a.anyPoint() - _b.anyPoint();

		//Go straight to the origin
		snVec d = -support[0];

		//Check if the first support point is the origin
		if (snVec3SquaredNorme(d) < 1e-7f)
			return true;

		//Start to iterate
		int n = 1; 
		int i = MAX_ITERATION;
		while (--i >= 0)
		{
			//Normalize the direction and compute the support point.
			snVec3Normalize(d);
			float maxS, minS;
			snVec newSupport = _a.support(d, maxS) - _b.support(-d, minS);

			//If this new support point did not passed the origin then the Minkowski difference does not contain the origin so no collision.
			if (minS + maxS < 0.f)
				return false;

			//Update the simplex
			support[n] = newSupport;
			++n;
			d = updateSimplex(support, n);

			//If the origin lies inside the simplex then intersection
			if (n == 0) 
				return true;
		}
		
		//We reached the maximum number of iteration so exit and report no intersection.
		return false;
	}
}

#endif //SN_GJK_H