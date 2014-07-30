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

		//Check if two collider intersects
		// _a : first collider.
		// _b : second collider.
		// _simplex : (out parameter) array of 4 snVec containing the result simplex.
		// _simplexSize : (out parameter) the number of vertices in the result simplex.
		// result : True if the two colliders intersect. False otherwise.
		static bool intersect(const snICollider& _a, const snICollider& _b, snVec* const _simplex, unsigned int& _simplexSize);

		//Compute the closest points between two colliders.
		// _a : first collider.
		// _b : second collider.
		// _distance : the distance between the two colliders. It's the distance between the two closest points.
		// return : true if the distance was found. False otherwise.
		static bool distance(const snICollider& _a, const snICollider& _b, float& _distance);

	private:

		static snVec updateSimplex(snVec* _s, int& _n);

		static snVec updateTwoSimplex(snVec* _s, int& _n);
		
		static snVec updateThreeSimplex(snVec* _s, int& _n);

		static snVec updateFourSimplex(snVec* _s, int& _n);
	};
}

#endif //SN_GJK_H