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
#include "snClosestPoint.h"
#include "snMath.h"

using namespace Supernova::Vector;

namespace Supernova
{
	void snClosestPoint::SegmentSegment(const snVec& _p1, const snVec& _q1, const snVec& _p2, const snVec& _q2,
		float& s, float& t, snVec& _c1, snVec& _c2)
	{
		snVec d1 = _q1 - _p1; //direction of segment s1
		snVec d2 = _q2 - _p2; //direction of segment s2
		snVec r = _p1 - _p2;

		snVec a = snVec3Dot(d1, d1); //squared length of s1
		snVec e = snVec3Dot(d2, d2); //squared length od s2
		snVec f = snVec3Dot(d2, r);

		const float EPSILON = 1e-7f;
		const snVec EPSILON_VEC = snVec4Set(EPSILON);

		//check for degenerate cases
		if (snVec3Inferior(a, EPSILON_VEC) && snVec3Inferior(e, EPSILON_VEC))
		{
			s = t = 0;
			_c1 = _p1;
			_c2 = _p2;
			return;
		}

		if (snVec3Inferior(a, EPSILON_VEC)) //only s1 is degenerate
		{
			s = 0;
			t = snVec4GetX(f / e);
			t = clamp(t, 0, 1);
		}
		else //s1 is not degenerate
		{
			snVec c = snVec3Dot(d1, r);
			if (snVec3Inferior(e, EPSILON_VEC)) //s2 is degenerate
			{
				t = 0;
				s = clamp(snVec4GetX(-c / a), 0, 1);
			}
			else //no degenerate segment
			{
				snVec b = snVec3Dot(d1, d2);
				snVec denom = (a * e) - (b * b);

				if (!(denom == VEC_ZERO)) //s1 and s2 are not colinear
				{
					s = clamp(snVec4GetX((b * f - c * e) / denom), 0, 1);
				}
				else //colinear
				{
					s = 0;
				}

				t = snVec4GetX((b * s + f) / e);

				if (t < 0)
				{
					t = 0;
					s = clamp(snVec4GetX(-c / a), 0, 1);
				}
				else if (t > 1)
				{
					t = 1;
					s = clamp(snVec4GetX((b - c) / a), 0, 1);
				}
			}
		}

		_c1 = _p1 + d1 * s;
		_c2 = _p2 + d2 * t;
	}

	snVec snClosestPoint::PointTriangle(const snVec& _p, const snVec& _a, const snVec& _b, const snVec& _c)
	{
		//Check if p is in vertex region outside a
		snVec ab = _b - _a;
		snVec ac = _c - _a;
		snVec ap = _p - _a;

		float d1 = snVec4GetX(snVec3Dot(ab, ap));
		float d2 = snVec4GetX(snVec3Dot(ac, ap));

		//Barycentric coordiante (1, 0, 0)
		if (d1 <= 0.f && d2 <= 0.f)
			return _a;

		//Check if p in vertex region outside b
		snVec bp = _p - _b;
		float d3 = snVec4GetX(snVec3Dot(ab, bp));
		float d4 = snVec4GetX(snVec3Dot(ac, bp));
		if (d3 >= 0.f && d4 <= d3) //Barycentric coordinates (0, 1, 0)
			return _b;

		//Check if p is in region AB
		float vc = d1 * d4 - d3 * d2;
		if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f)
		{
			float v = d1 / (d1 - d3);
			return _a + v * ab; //Barycentric coordinates (1-v, v, 0)
		}

		//Check if p is in region outside c
		snVec cp = _p - _c;
		float d5 = snVec4GetX(snVec3Dot(ab, cp));
		float d6 = snVec4GetX(snVec3Dot(ac, cp));
		if (d6 >= 0.f && d5 <= d6) //Barycentric coordinates (0, 0, 1)
			return _c;

		//Check if p is in edge region of AC
		float vb = d5 * d2 - d1 * d6;
		if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f) //Barycentric coordinates (1-w, 0, w)
		{
			float w = d2 / (d2 - d6);
			return _a + w * ac;
		}

		//Check if p is in edge region of bc.
		float va = d3 * d6 - d5 * d4;
		if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f)//Barycentric coordinates (0, 1-w, w)
		{
			float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
			return _b + w * (_c - _b);
		}

		//p inside face region. Computer Q through its barycentric coordinates (u, v, w)
		float denom = 1.f / (va + vb + vc);
		float v = vb * denom;
		float w = vc * denom;
		return _a + ab * v + ac * w;
	}
}