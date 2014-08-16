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

#ifndef SN_VEC_H
#define SN_VEC_H

union __m128;

#include "snGlobals.h"

namespace Supernova
{
	typedef __m128 snVec;

	namespace Vector
	{
		SN_INLINE snVec snVec4Set(float _x, float _y, float _z, float _w);

		SN_INLINE snVec snVec4Set(float _value);

		//Set the x, y and z component to _value. Set to w component to 0.
		SN_INLINE snVec snVec3Set(float _value);

		SN_INLINE snVec operator*(const snVec& _a, const snVec& _b);

		SN_INLINE snVec operator*(const snVec& _a, float _f);

		SN_INLINE snVec operator*(float _f, const snVec& _a);

		SN_INLINE snVec operator/(const snVec& _a, const snVec& _b);

		SN_INLINE snVec operator+(const snVec& _a, const snVec& _b);

		SN_INLINE snVec operator-(const snVec& _a, const snVec& _b);

		SN_INLINE snVec operator-(const snVec& _a);

		SN_INLINE bool operator == (const snVec& _a, const snVec& _b);

		SN_INLINE snVec snVec3Dot(const snVec& _a, const snVec& _b);

		SN_INLINE float snVec4Dot(const snVec& _a, const snVec& _b);

		/*Cross product between _v1 and _v2. The W coordinate will be 0.*/
		SN_INLINE snVec snVec3Cross(const snVec& _v1, const snVec& _v2);

		/*Return the squared length of the vector.*/
		SN_INLINE float snVec3SquaredNorme(const snVec& _a);

		/*Calculate the length of the vector.*/
		SN_INLINE float snVec3Norme(const snVec& _a);

		/*Normalize the vector. Its direction remain the same but its length is set to 1.*/
		SN_INLINE void snVec3Normalize(snVec& _a);

		SN_INLINE snVec snVec4GetInverse(const snVec& _v);

		SN_INLINE snVec snVec4GetAbsolute(const snVec& _a);

		SN_INLINE void snVec4Absolute(snVec& _a);

		SN_INLINE float snVec4GetById(const snVec& _a, unsigned int _id);

		SN_INLINE float snVec4GetX(const snVec& _v);

		SN_INLINE float snVec4GetY(const snVec& _v);

		SN_INLINE float snVec4GetZ(const snVec& _v);

		SN_INLINE float snVec4GetW(const snVec& _v);

		SN_INLINE void snVec4SetX(snVec& _v, float _x);

		SN_INLINE void snVec4SetY(snVec& _v, float _y);

		SN_INLINE void snVec4SetZ(snVec& _v, float _z);

		SN_INLINE void snVec4SetW(snVec& _v, float _w);

		//Return a vector containing the maximum value of _v1 and _v2
		SN_INLINE snVec snVec4GetMax(const snVec& _v1, const snVec& _v2);

		//Reurn a vector where all four values are equal to the maximum float of _v
		SN_INLINE snVec snVec4GetMax(const snVec& _v);

		//Return a vector containing the minimum value of _v1 and _v2
		SN_INLINE snVec snVec4GetMin(const snVec& _v1, const snVec& _v2);

		//Reurn a vector where all four values are equal to the minimum float of _v
		SN_INLINE snVec snVec4GetMin(const snVec& _v);

		//Clamp each component of _v using each component of _min as minimum value and each component of _max as maximum value
		SN_INLINE snVec snVec4Clamp(const snVec& _v, const snVec& _min, const snVec& _max);

		//Compare each member of _a to each member of _b and return true if all of them are inferiors.
		SN_INLINE bool snVec3Inferior(const snVec& _a, const snVec& _b);

		SN_INLINE bool snVec3SuperiorOrEqual(const snVec& _a, const snVec& _b);

		//Return the id of the minimum axis
		SN_INLINE unsigned int snVec3GetMinAxis(const snVec& _v);
	}
}

#include "snVec.inl"

#endif //ifndef SN_VEC_H