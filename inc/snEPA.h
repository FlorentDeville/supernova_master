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

#ifndef SN_EPA_H
#define SN_EPA_H

#include "snVec.h"

namespace Supernova
{
	class snSimplex;
	class snICollider;

	//Take a simplex from GJK and execute the EPA algorithm on it to get the collision normal
	class snEPA
	{
	public:

		//Apply EPA to the simplex to find the collision normal.
		bool execute(snSimplex& _simplex, const snICollider& _c1, const snICollider& _c2, snVec& _normal) const;

		//Take a raw simplex (vector of snVec) coming from GJK and convert it into an snSimplex of four points or more that the
		//function execute can use.
		bool prepareSimplex(snVec * const _inSimplex, unsigned int _inSimplexSize, const snICollider& _c1, const snICollider& _c2, snSimplex& _outSimplex) const;

	private:

		//Make a simplex EPA can use from the gjk simplex with two points
		bool prepareSimplex2(snVec * const _inSimplex, const snICollider& _c1, const snICollider& _c2, snSimplex& _outSimplex) const;

		//Make a simplex EPA can use from the gjk simplex with three points
		bool prepareSimplex3(snVec * const _inSimplex, const snICollider& _c1, const snICollider& _c2, snSimplex& _outSimplex) const;

		//Make a simplex EPA can use from the gjk simplex with four points
		bool prepareSimplex4(snVec * const _inSimplex, snSimplex& _outSimplex) const;

		//Return true if the origin is inside the tetrahedrion defined by four points.
		bool isValidStartSimplex(const snVec& _p1, const snVec& _p2, const snVec& _p3, const snVec& _p4) const;
	};
}
#endif //ifdnef SN_EPA_H