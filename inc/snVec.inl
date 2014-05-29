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

#ifndef SN_VEC_INL
#define SN_VEC_INL

#include "snVec.h"

#include <math.h>
#include <xmmintrin.h>
#include <emmintrin.h>

#define VEC_ID_X 0
#define VEC_ID_Y 1
#define VEC_ID_Z 2
#define VEC_ID_W 3

namespace Supernova
{
	namespace Vector
	{
		const snVec VEC_ZERO = snVec4Set(0, 0, 0, 0);

		//Mask where the only bit is 1 is the most significant bit. It's the sign bit in a 32 bit float.
		const snVec VEC_SIGNMASK = _mm_castsi128_ps(_mm_set1_epi32(0x80000000));

		snVec snVec4Set(float _x, float _y, float _z, float _w)
		{
			return _mm_set_ps(_w, _z, _y, _x);
		}

		snVec operator*(const snVec& _a, const snVec& _b)
		{
			return _mm_mul_ps(_a, _b);
		}

		snVec operator*(const snVec& _a, float _f)
		{
			snVec f = snVec4Set(_f, _f, _f, _f);
			return _a * f;
		}

		snVec operator*(float _f, const snVec& _a)
		{
			snVec f = snVec4Set(_f, _f, _f, _f);
			return _a * f;
		}

		snVec operator+(const snVec& _a, const snVec& _b)
		{
			return _mm_add_ps(_a, _b);
		}

		snVec operator-(const snVec& _a, const snVec& _b)
		{
			return _mm_sub_ps(_a, _b);
		}

		snVec operator-(const snVec& _a)
		{
			//To negate a float, we need to switch the bit sign.
			//Using XOR operator with the bit mask will do so as the bit mask as a 1 only for the sign bit.
			return _mm_xor_ps(_a, VEC_SIGNMASK);
		}

		bool operator == (const snVec& _a, const snVec& _b)
		{
			__m128 vcmp = _mm_cmpeq_ps(_a, _b);
			__m128i ivcmp = _mm_castps_si128(vcmp);
			int vmask = _mm_movemask_epi8(ivcmp);
			return (vmask == 0xffff);
		}

		float snVec3Dot(const snVec& _a, const snVec& _b)
		{
			snVec mul = _a * _b;
			return mul.m128_f32[VEC_ID_X] + mul.m128_f32[VEC_ID_Y] + mul.m128_f32[VEC_ID_Z];
		}

		float snVec4Dot(const snVec& _a, const snVec& _b)
		{
			snVec mul = _a * _b;
			return mul.m128_f32[VEC_ID_X] + mul.m128_f32[VEC_ID_Y] + mul.m128_f32[VEC_ID_Z] + mul.m128_f32[VEC_ID_W];
		}

		/*Cross product between _v1 and _v2. The W coordinate will be 0.*/
		snVec snVec3Cross(const snVec& _v1, const snVec& _v2)
		{
			__m128 v1Left = _mm_shuffle_ps(_v1, _v1, _MM_SHUFFLE(VEC_ID_W, VEC_ID_X, VEC_ID_Z, VEC_ID_Y));
			__m128 v1Right = _mm_shuffle_ps(_v1, _v1, _MM_SHUFFLE(VEC_ID_W, VEC_ID_Y, VEC_ID_X, VEC_ID_Z));

			__m128 v2Left = _mm_shuffle_ps(_v2, _v2, _MM_SHUFFLE(VEC_ID_W, VEC_ID_Y, VEC_ID_X, VEC_ID_Z));
			__m128 v2Right = _mm_shuffle_ps(_v2, _v2, _MM_SHUFFLE(VEC_ID_W, VEC_ID_X, VEC_ID_Z, VEC_ID_Y));

			__m128 Left = _mm_mul_ps(v1Left, v2Left);
			__m128 Right = _mm_mul_ps(v1Right, v2Right);

			return _mm_sub_ps(Left, Right);
		}

		/*Return the squared length of the vector.*/
		float snVec3SquaredNorme(const snVec& _a)
		{
			return snVec3Dot(_a, _a);
		}

		/*Calculate the length of the vector.*/
		float snVec3Norme(const snVec& _a)
		{
			return sqrtf(snVec3SquaredNorme(_a));
		}

		/*Normalize the vector. Its direction remain the same but its length is set to 1.*/
		void snVec3Normalize(snVec& _a)
		{
			float n = snVec3Norme(_a);
			if (n == 0) return;

			float invNorme = 1.f / n;
			snVec div = snVec4Set(invNorme, invNorme, invNorme, 1);
			_a = _a * div;
		}

		snVec snVec4GetAbsolute(const snVec& _a)
		{
			return _mm_andnot_ps(VEC_SIGNMASK, _a);
		}

		void snVec4Absolute(snVec& _a)
		{
			_a = snVec4GetAbsolute(_a);
		}

		float snVec4GetById(const snVec& _a, unsigned int _id)
		{
			return _a.m128_f32[_id];
		}

		float snVec4GetX(const snVec& _v)
		{
			return snVec4GetById(_v, VEC_ID_X);
		}

		float snVec4GetY(const snVec& _v)
		{
			return snVec4GetById(_v, VEC_ID_Y);
		}

		float snVec4GetZ(const snVec& _v)
		{
			return snVec4GetById(_v, VEC_ID_Z);
		}

		float snVec4GetW(const snVec& _v)
		{
			return snVec4GetById(_v, VEC_ID_W);
		}

		void snVec4SetY(snVec& _v, float _y)
		{
			_v.m128_f32[VEC_ID_Y] = _y;
		}

		void snVec4SetZ(snVec& _v, float _z)
		{
			_v.m128_f32[VEC_ID_Z] = _z;
		}

		void snVec4SetW(snVec& _v, float _w)
		{
			_v.m128_f32[VEC_ID_W] = _w;
		}
	}
}

#endif //ifndef SN_VEC_INL