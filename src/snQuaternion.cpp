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
#include "snMatrix44f.h"

using namespace Supernova::Vector;

namespace Supernova
{
	void snQuaternionMultiply(const snVec& _q1, const snVec& _q2, snVec& _result)
	{
		snVec q1Crossq2 = snVec3Cross(_q1, _q2);
		float q1Dotq2 = snVec4GetX(snVec3Dot(_q1, _q2));
		float resultW = snVec4GetW(_q1) * snVec4GetW(_q2) - q1Dotq2;

		_result = (_q2 * snVec4GetW(_q1)) + (_q1 * snVec4GetW(_q2)) + q1Crossq2;
		snVec4SetW(_result, resultW);
	}

	snVec snQuaternionFromEuler(float _x, float _y, float _z)
	{
		//This sucks and could be improved incredibly calculating the cos and sin with simd but I don't have time right now.
		float halfX = _x * 0.5f;
		float halfY = _y * 0.5f;
		float halfZ = _z * 0.5f;

		snVec rotationX = snVec4Set(sinf(halfX), 0, 0, cosf(halfX));
		snVec rotationY = snVec4Set(0, sinf(halfY), 0, cosf(halfY));
		snVec rotationZ = snVec4Set(0, 0, sinf(halfZ), cosf(halfZ));

		snVec res;
		snQuaternionMultiply(rotationX, rotationY, res);
		snQuaternionMultiply(res, rotationZ, res);

		return res;
	}

	snVec snQuaternionFromEuler(const snVec& _euler)
	{
		//Ok this is really bad but right now I don't have the time to make a real version.
		return snQuaternionFromEuler(snVec4GetX(_euler), snVec4GetY(_euler), snVec4GetZ(_euler));
	}

	void snQuaternionNormalize(const snVec& _q, snVec& _n)
	{
		float oneOverNorme = 1.f / sqrt(snVec4Dot(_q, _q));
		_n = _q * oneOverNorme;
	}

	snVec snQuaternionFromMatrix(const snMatrix44f& _m)
	{
		
		float qw, qx, qy, qz;

		float m12 = snVec4GetZ(_m.m_r[1]);
		float m21 = snVec4GetY(_m.m_r[2]);
		float m20 = snVec4GetX(_m.m_r[2]);
		float m02 = snVec4GetZ(_m.m_r[0]);
		float m01 = snVec4GetY(_m.m_r[0]);
		float m10 = snVec4GetX(_m.m_r[1]);

		float m00 = snVec4GetX(_m.m_r[0]);
		float m11 = snVec4GetY(_m.m_r[1]);
		float m22 = snVec4GetZ(_m.m_r[2]);

		float tr = m00 + m11 + m22;
		if (tr > 0) 
		{
			float S = sqrtf(tr + 1.f) * 2; // S=4*qw 
			qw = 0.25f * S;
			qx = (m12 - m21) / S;
			qy = (m20 - m02) / S;
			qz = (m01 - m10) / S;
		}
		else if ((m00 > m11)&(m00 > m22)) 
		{
			float S = sqrtf(1.f + m00 - m11 - m22) * 2; // S=4*qx 
			qw = (m12 - m21) / S;
			qx = 0.25f * S;
			qy = (m10 + m01) / S;
			qz = (m20 + m02) / S;
		}
		else if (m11 > m22) 
		{
			float S = sqrtf(1.f + m11 - m00 - m22) * 2; // S=4*qy
			qw = (m20 - m02) / S;
			qx = (m10 + m01) / S;
			qy = 0.25f * S;
			qz = (m21 + m12) / S;
		}
		else 
		{
			float S = sqrtf(1.f + m22 - m00 - m11) * 2; // S=4*qz
			qw = (m10 - m01) / S;
			qx = (m02 + m20) / S;
			qy = (m12 + m21) / S;
			qz = 0.25f * S;
		}

		return snVec4Set(qx, qy, qz, qw);
	}
}