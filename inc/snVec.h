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

namespace Supernova
{
	typedef __m128 snVec;

	namespace Vector
	{
		inline snVec snVec4Set(float _x, float _y, float _z, float _w);

		inline snVec snVec4Set(float _value);

		inline snVec operator*(const snVec& _a, const snVec& _b);

		inline snVec operator*(const snVec& _a, float _f);

		inline snVec operator*(float _f, const snVec& _a);

		inline snVec operator+(const snVec& _a, const snVec& _b);

		inline snVec operator-(const snVec& _a, const snVec& _b);

		inline snVec operator-(const snVec& _a);

		inline bool operator == (const snVec& _a, const snVec& _b);

		inline float snVec3Dot(const snVec& _a, const snVec& _b);

		//inline snVec snVec3Dot(const snVec& _a, const snVec& _b);

		inline float snVec4Dot(const snVec& _a, const snVec& _b);

		/*Cross product between _v1 and _v2. The W coordinate will be 0.*/
		inline snVec snVec3Cross(const snVec& _v1, const snVec& _v2);

		/*Return the squared length of the vector.*/
		inline float snVec3SquaredNorme(const snVec& _a);

		/*Calculate the length of the vector.*/
		inline float snVec3Norme(const snVec& _a);

		/*Normalize the vector. Its direction remain the same but its length is set to 1.*/
		inline void snVec3Normalize(snVec& _a);

		inline snVec snVec4GetInverse(const snVec& _v);

		inline snVec snVec4GetAbsolute(const snVec& _a);

		inline void snVec4Absolute(snVec& _a);

		inline float snVec4GetById(const snVec& _a, unsigned int _id);

		inline float snVec4GetX(const snVec& _v);

		inline float snVec4GetY(const snVec& _v);

		inline float snVec4GetZ(const snVec& _v);

		inline float snVec4GetW(const snVec& _v);

		inline void snVec4SetX(snVec& _v, float _x);

		inline void snVec4SetY(snVec& _v, float _y);

		inline void snVec4SetZ(snVec& _v, float _z);

		inline void snVec4SetW(snVec& _v, float _w);

		//Return a vector containing the maximum value of _v1 and _v2
		inline snVec snVec4GetMax(const snVec& _v1, const snVec& _v2);

		//Reurn a vector where all four values are equal to the maximum float of _v
		inline snVec snVec4GetMax(const snVec& _v);

		//Return a vector containing the minimum value of _v1 and _v2
		inline snVec snVec4GetMin(const snVec& _v1, const snVec& _v2);

		//Reurn a vector where all four values are equal to the minimum float of _v
		inline snVec snVec4GetMin(const snVec& _v);

		//Clamp each component of _v using each component of _min as minimum value and each component of _max as maximum value
		inline snVec snVec4Clamp(const snVec& _v, const snVec& _min, const snVec& _max);
	}
}

#endif //ifndef SN_VEC_H