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
#define SUPPORT(dir) (_a.support(dir, maxS) - _b.support(-dir, minS));

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

		template<class T, class U> static bool GJKIntersect(const T& _a, const U& _b);

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

		static snVec UpdateSimplex(snVec* _s, int& _n);

		static snVec UpdateTwoSimplex(snVec* _s, int& _n);
		
		static snVec UpdateThreeSimplex(snVec* _s, int& _n);

		static snVec UpdateFourSimplex(snVec* _s, int& _n);
	};

	template<class T, class U> bool snGJK::GJKIntersect(const T& _a, const U& _b)
	{
		snVec support[4];
		float maxS, minS;
		// Start with an arbitrary point in the Minkowski set shape.
		support[0] = _a.anyPoint() - _b.anyPoint();
		snVec d = -support[0]; // First search direction is straight toward the origin from the found point.
		if (snVec3SquaredNorme(d) < 1e-7f) // Robustness check: Test if the first arbitrary point we guessed produced the zero vector we are looking for!
			return true;
		int n = 1; // Stores the current number of points in the search simplex.
		int nIterations = 50; // Robustness check: Limit the maximum number of iterations to perform to avoid infinite loop if types A or B are buggy!
		while (nIterations-- > 0)
		{
			// Compute the extreme point to the direction d in the Minkowski set shape.
			snVec3Normalize(d);
			snVec newSupport = SUPPORT(d);
#ifdef MATH_VEC_IS_FLOAT4
			assume(newSupport.w == 0.f);
#endif
			// If the most extreme point in that search direction did not walk past the origin, then the origin cannot be contained in the Minkowski
			// convex shape, and the two convex objects a and b do not share a common point - no intersection!
			if (minS + maxS < 0.f)
				return false;
			// Add the newly evaluated point to the search simplex.
			support[n++] = newSupport;
			// Examine the current simplex, prune a redundant part of it, and produce the next search direction.
			d = UpdateSimplex(support, n);
			if (n == 0) // Was the origin contained in the current simplex? If so, then the convex shapes a and b do share a common point - intersection!
				return true;
		}
		//assume2(false && "GJK intersection test did not converge to a result!", a.SerializeToString(), b.SerializeToString());
		return false; // Report no intersection.
	}
}

#endif //SN_GJK_H