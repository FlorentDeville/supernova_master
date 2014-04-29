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

#ifndef SN_VECTOR4F_H
#define SN_VECTOR4F_H

#include <xmmintrin.h>
#include <emmintrin.h>
#include <math.h>

#define VEC4F_ID_X 0
#define VEC4F_ID_Y 1
#define VEC4F_ID_Z 2
#define VEC4F_ID_W 3

#define VEC4FX m_vec.m128_f32[VEC4F_ID_X]
#define VEC4FY m_vec.m128_f32[VEC4F_ID_Y]
#define VEC4FZ m_vec.m128_f32[VEC4F_ID_Z]
#define VEC4FW m_vec.m128_f32[VEC4F_ID_W]

#include "snGlobals.h"

//Return true if two vectors point to the same direction.
#define SN_SAME_DIRECTION(_v1, _v2) _v1.dot(_v2) > 0

namespace Supernova
{
	//Represent a 4 floating point vector implemented with simd vector.
	class SN_ALIGN snVector4f
	{
	public:
		__m128 m_vec;

		//Bit mask where the only bit to 1 is the sign bit. In a 32 bits float, the bit sign is the most significant bit so
		//this mask contains 4 floating point with a binary value 0x80000000.
		static const __m128 SIGNMASK;

		//A vector with the four parameter set to 0
		static const snVector4f m_zero;

	public:
		snVector4f(__m128 _Value);

		snVector4f();

		snVector4f(float _X, float _Y, float _Z, float _W);

		~snVector4f();

		snVector4f operator+(const snVector4f& _Other) const;

		snVector4f operator-(const snVector4f& _Other) const;

		snVector4f operator-() const;

		snVector4f operator*(const snVector4f& _Other) const;
		
		snVector4f operator*(float _Other) const;
		
		snVector4f operator*(int _other) const;
		
		float dot4(const snVector4f& _other) const;
		
		float dot(const snVector4f& _other) const;
		
		/*Dot product between _v1 and _v2. The W coordinate is ignored.*/
		static float dot(const snVector4f& _V1, const snVector4f& _V2);
		
		snVector4f cross(const snVector4f& _other) const;
		
		/*Cross product between _v1 and _v2. The W coordinate will be 0.*/
		static snVector4f cross(const snVector4f& _V1, const snVector4f& _V2);
		
		/*Calculate the length of the vector.*/
		float norme() const;
	
		/*Return the squared length of the vector.*/
		float squareNorme() const;
		
		/*Normalize the vector. Its direction remain the same but its length is set to 1.*/
		void normalize();
		
		/*Set the vector to the length provided in parameter.*/
		void setLength(float _length);
		
		void mirror(const snVector4f& _incoming, const snVector4f& _normal, snVector4f& _out);
		
		float getX() const;
		
		float getY() const;
		
		float getZ() const;
		
		float getW() const;
		
		void setX(float _x);
		
		void setY(float _y);
		
		void setZ(float _z);
		
		void setW(float _w);
		
		void set(float _X, float _Y, float _Z, float _W);
		
		bool operator == (const snVector4f& _other) const;
		
		void absolute();
		
		snVector4f getAbsolute();
		
		float& operator[](int id);
		
		float operator[](int id) const;

		//Compute a cosine interpolation between two vectors given a parameter t between 0 and 1
		static snVector4f cosInterpolation(const snVector4f& _start, const snVector4f& _end, float _t);

		//Compute catmull rom interpolation.
		static snVector4f catmullRomInterpolation(const snVector4f& _y0, const snVector4f& _y1, const snVector4f& _y2,
			const snVector4f& _y3, float _t);
		
	};
}

#endif // SNVECTOR4F_H