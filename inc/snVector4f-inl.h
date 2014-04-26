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

#ifndef SN_VECTOR4F_INL
#define SN_VECTOR4F_INL

#include "snVector4f.h"
#include "snMath.h"

namespace Supernova
{
	inline snVector4f::snVector4f(__m128 _Value)
	{
		m_vec = _Value;
	}

	inline snVector4f::snVector4f()
	{
		m_vec = _mm_setzero_ps();
	}

	inline snVector4f::snVector4f(float _X, float _Y, float _Z, float _W)
	{
		m_vec = _mm_set_ps(_W, _Z, _Y, _X);
	}

	inline snVector4f::~snVector4f(){}

	inline snVector4f snVector4f::operator + (const snVector4f& _Other) const
	{
		snVector4f ret(_mm_add_ps(m_vec, _Other.m_vec));
		return ret;
	}

	inline snVector4f snVector4f::operator - (const snVector4f& _Other)const
	{
		snVector4f ret(_mm_sub_ps(m_vec, _Other.m_vec));
		return ret;
	}

	inline snVector4f snVector4f::operator-() const
	{
		//To negate a float, we need to switch the bit sign.
		//Using XOR operator with the bit mask will do so as the bit mask as a 1 only for the sign bit.
		return _mm_xor_ps(m_vec, SIGNMASK);
	}

	inline snVector4f snVector4f::operator*(const snVector4f& _Other) const
	{
		snVector4f ret(_mm_mul_ps(m_vec, _Other.m_vec));
		return ret;
	}

	inline snVector4f snVector4f::operator*(float _Other) const
	{
		__m128 factor = _mm_set_ps1(_Other);
		snVector4f ret(_mm_mul_ps(factor, m_vec));
		return ret;
	}

	inline snVector4f snVector4f::operator*(int _other) const
	{
		__m128 factor = _mm_set_ps1((float)_other);
		snVector4f ret(_mm_mul_ps(factor, m_vec));
		return ret;
	}

	inline float snVector4f::dot4(const snVector4f& _other) const
	{
		__m128 mul = _mm_mul_ps(m_vec, _other.m_vec);
		return mul.m128_f32[VEC4F_ID_X] + mul.m128_f32[VEC4F_ID_Y] + mul.m128_f32[VEC4F_ID_Z] + mul.m128_f32[VEC4F_ID_W];
	}

	inline float snVector4f::dot(const snVector4f& _other) const
	{
		__m128 mul = _mm_mul_ps(m_vec, _other.m_vec);
		return mul.m128_f32[VEC4F_ID_X] + mul.m128_f32[VEC4F_ID_Y] + mul.m128_f32[VEC4F_ID_Z];
	}

	/*Dot product between _v1 and _v2. The W coordinate is ignored.*/
	inline float snVector4f::dot(const snVector4f& _V1, const snVector4f& _V2)
	{
		__m128 mul = _mm_mul_ps(_V1.m_vec, _V2.m_vec);
		return mul.m128_f32[VEC4F_ID_X] + mul.m128_f32[VEC4F_ID_Y] + mul.m128_f32[VEC4F_ID_Z];
	}

	inline snVector4f snVector4f::cross(const snVector4f& _other) const
	{
		__m128 v1Left = _mm_shuffle_ps(m_vec, m_vec, _MM_SHUFFLE(VEC4F_ID_W, VEC4F_ID_X, VEC4F_ID_Z, VEC4F_ID_Y));
		__m128 v1Right = _mm_shuffle_ps(m_vec, m_vec, _MM_SHUFFLE(VEC4F_ID_W, VEC4F_ID_Y, VEC4F_ID_X, VEC4F_ID_Z));

		__m128 v2Left = _mm_shuffle_ps(_other.m_vec, _other.m_vec, _MM_SHUFFLE(VEC4F_ID_W, VEC4F_ID_Y, VEC4F_ID_X, VEC4F_ID_Z));
		__m128 v2Right = _mm_shuffle_ps(_other.m_vec, _other.m_vec, _MM_SHUFFLE(VEC4F_ID_W, VEC4F_ID_X, VEC4F_ID_Z, VEC4F_ID_Y));

		__m128 Left = _mm_mul_ps(v1Left, v2Left);
		__m128 Right = _mm_mul_ps(v1Right, v2Right);

		return snVector4f(_mm_sub_ps(Left, Right));
	}

	/*Cross product between _v1 and _v2. The W coordinate will be 0.*/
	inline snVector4f snVector4f::cross(const snVector4f& _V1, const snVector4f& _V2)
	{
		__m128 v1Left = _mm_shuffle_ps(_V1.m_vec, _V1.m_vec, _MM_SHUFFLE(VEC4F_ID_W, VEC4F_ID_X, VEC4F_ID_Z, VEC4F_ID_Y));
		__m128 v1Right = _mm_shuffle_ps(_V1.m_vec, _V1.m_vec, _MM_SHUFFLE(VEC4F_ID_W, VEC4F_ID_Y, VEC4F_ID_X, VEC4F_ID_Z));

		__m128 v2Left = _mm_shuffle_ps(_V2.m_vec, _V2.m_vec, _MM_SHUFFLE(VEC4F_ID_W, VEC4F_ID_Y, VEC4F_ID_X, VEC4F_ID_Z));
		__m128 v2Right = _mm_shuffle_ps(_V2.m_vec, _V2.m_vec, _MM_SHUFFLE(VEC4F_ID_W, VEC4F_ID_X, VEC4F_ID_Z, VEC4F_ID_Y));

		__m128 Left = _mm_mul_ps(v1Left, v2Left);
		__m128 Right = _mm_mul_ps(v1Right, v2Right);

		return snVector4f(_mm_sub_ps(Left, Right));
	}

	/*Calculate the length of the vector.*/
	inline float snVector4f::norme() const
	{
		return sqrtf(squareNorme());
	}

	/*Return the squared length of the vector.*/
	inline float snVector4f::squareNorme() const
	{
		__m128 square = _mm_mul_ps(m_vec, m_vec);
		return square.m128_f32[VEC4F_ID_X] + square.m128_f32[VEC4F_ID_Y] + square.m128_f32[VEC4F_ID_Z];
	}

	/*Normalize the vector. Its direction remain the same but its length is set to 1.*/
	inline void snVector4f::normalize()
	{
		float n = norme();
		if (n == 0) return;
		__m128 div = _mm_set_ps(1, n, n, n);
		m_vec = _mm_div_ps(m_vec, div);
	}

	/*Set the vector to the length provided in parameter.*/
	inline void snVector4f::setLength(float _length)
	{
		normalize();
		__m128 l = _mm_set_ps(1, _length, _length, _length);
		m_vec = _mm_mul_ps(m_vec, l);
	}

	inline void snVector4f::mirror(const snVector4f& _incoming, const snVector4f& _normal, snVector4f& _out)
	{
		_out = (_normal * 2 * (-_incoming.dot(_normal))) + _incoming;
	}

	inline float snVector4f::getX() const
	{
		return VEC4FX;
	}

	inline float snVector4f::getY() const
	{
		return VEC4FY;
	}

	inline float snVector4f::getZ() const
	{
		return VEC4FZ;
	}

	inline float snVector4f::getW() const
	{
		return VEC4FW;
	}

	inline void snVector4f::setX(float _x)
	{
		VEC4FX = _x;
	}

	inline void snVector4f::setY(float _y)
	{
		VEC4FY = _y;
	}

	inline void snVector4f::setZ(float _z)
	{
		VEC4FZ = _z;
	}

	inline void snVector4f::setW(float _w)
	{
		VEC4FW = _w;
	}

	inline void snVector4f::set(float _X, float _Y, float _Z, float _W)
	{
		m_vec = _mm_set_ps(_W, _Z, _Y, _X);
	}

	inline bool snVector4f::operator == (const snVector4f& _other) const
	{
		__m128 vcmp = _mm_cmpeq_ps(m_vec, _other.m_vec);
		__m128i ivcmp = _mm_castps_si128(vcmp);
		int vmask = _mm_movemask_epi8(ivcmp);
		return (vmask == 0xffff);
	}

	inline void snVector4f::absolute()
	{
		m_vec = _mm_andnot_ps(SIGNMASK, m_vec);
	}

	inline snVector4f snVector4f::getAbsolute()
	{
		return _mm_andnot_ps(SIGNMASK, m_vec);
	}

	inline float& snVector4f::operator[](int id)
	{
		return m_vec.m128_f32[id];
	}

	inline float snVector4f::operator[](int id) const
	{
		return m_vec.m128_f32[id];
	}

	inline snVector4f snVector4f::cosInterpolation(const snVector4f& _start, const snVector4f& _end, float _t)
	{
		float cosineT = (1.f - cosf(_t * SN_PI)) * 0.5f;
		return (_start * (1 - cosineT) + _end * cosineT);
	}
}

#endif //ifndef SN_VECTOR4F_INL