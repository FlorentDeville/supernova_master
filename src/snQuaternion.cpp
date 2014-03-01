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

#include "snQuaternion.h"

namespace Supernova
{
	void snQuaternionMultiply(const snVector4f& _q1, const snVector4f& _q2, snVector4f& _result)
	{
		snVector4f q1Crossq2 = _q1.cross(_q2);
		float q1Dotq2 = _q1.dot(_q2);
		float resultW = _q1.getW() * _q2.getW() - q1Dotq2;

		_result = (_q2 * _q1.getW()) + (_q1 * _q2.getW()) + q1Crossq2;
		_result.setW(resultW);
	}

	snVector4f snQuaternionFromEuler(float _x, float _y, float _z)
	{
		float halfX = _x * 0.5f;
		float halfY = _y * 0.5f;
		float halfZ = _z * 0.5f;

		snVector4f rotationX = snVector4f(sinf(halfX), 0, 0, cosf(halfX));
		snVector4f rotationY = snVector4f(0, sinf(halfY), 0, cosf(halfY));
		snVector4f rotationZ = snVector4f(0, 0, sinf(halfZ), cosf(halfZ));

		snVector4f res;
		snQuaternionMultiply(rotationX, rotationY, res);
		snQuaternionMultiply(res, rotationZ, res);

		return res;
	}

	void snQuaternionNormalize(const snVector4f& _q, snVector4f& _n)
	{
		float oneOverNorme = 1.f / sqrt(_q.dot4(_q));
		_n = _q * oneOverNorme;
	}
}