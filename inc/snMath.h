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

#ifndef SN_MATH_H
#define SN_MATH_H

#include <cfloat>

//The maximum number a floating point variable can store.
#define SN_FLOAT_MAX FLT_MAX

//The smallest absolute number a floating point variable can store
#define SN_FLOAT_MIN FLT_MIN

#define SN_PI 3.1415926f

#include "snVec.inl"

namespace Supernova
{
	class snMatrix44f;

	//Clamp the value between min and max.
	float clamp(float _value, float _min, float _max);

	//Clamp a vector componentwise.
	snVec clampComponents(const snVec& _v, float _min, float _max);

	//Return true if the value is between the min and max value included.
	bool isInRange(float _value, float _min, float _max);

	//Return 1 if the float is positive, -1 if negative
	int sign(float _value);

	//compute an orthonormal basis for the vector _a. This is a code snippet from Erin Catto's blog.
	void computeBasis(const snVec& _a, snVec& _b, snVec& _c);

	//compute a frenet matrix from a catmull rom interpolation
	void computeFrenetFromCatmullRom(const snVec& _a0, const snVec& _a1, const snVec& _a2, const snVec& _a3,
		float _t, snMatrix44f& _frenet);

	snVec cosInterpolation(const snVec& _start, const snVec& _end, float _t);

	//Transform an array of struct of 4 vector into a structure of array of 4 elements
	void arrayOfStructToStructOfArray(const snVec* const _arrayOfStruct, snVec* const _structOfArray);

	//Linear interpolation between two snVec
	snVec lerp(const snVec& _start, const snVec& _end, float _t);

}
#endif //SN_MATH_H