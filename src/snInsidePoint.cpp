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
#include "snInsidePoint.h"

using namespace Supernova::Vector;

namespace Supernova
{
	bool snInsidePoint::isInsideTriangle(const snVec& _a, const snVec& _b, const snVec& _c, const snVec& _p)
	{
		snVec v0 = _b - _a;
		snVec v1 = _c - _a;
		snVec v2 = _p - _a;

		float d00 = snVec4GetX(snVec3Dot(v0, v0));
		float d01 = snVec4GetX(snVec3Dot(v0, v1));
		float d11 = snVec4GetX(snVec3Dot(v1, v1));
		float d20 = snVec4GetX(snVec3Dot(v2, v0));
		float d21 = snVec4GetX(snVec3Dot(v2, v1));

		float oneOverDenom = 1.f / (d00 * d11 - d01 * d01);

		float v = (d11 * d20 - d01 * d21) * oneOverDenom;
		float w = (d00 * d21 - d01 * d20) * oneOverDenom;
		float vPlusw = v + w;

		if(v >= 0 && v <= 1 && w >= 0 && w <= 1 && vPlusw <= 1)
			return true;

		return false;
	}
}