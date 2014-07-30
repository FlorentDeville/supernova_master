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

#include "snMatrix44f.h"

#include <cstring>

using namespace Supernova::Vector;

namespace Supernova
{
	//Initialize the zero matrix.
	const snMatrix44f snMatrix44f::m_zero;

	/*Default constructor*/
	snMatrix44f::snMatrix44f()
	{
		for (int i = 0; i < 4; i++)
			m_r[i] = _mm_setzero_ps();

	}

	/*Destructor*/
	snMatrix44f::~snMatrix44f(){}

	/*Set the matrix to the identity*/
	void snMatrix44f::identity()
	{
		m_r[0] = _mm_set_ps(0, 0, 0, 1);
		m_r[1] = _mm_set_ps(0, 0, 1, 0);
		m_r[2] = _mm_set_ps(0, 1, 0, 0);
		m_r[3] = _mm_set_ps(1, 0, 0, 0);
	}

	/*Calculate the inverse matrix*/
	snMatrix44f snMatrix44f::inverse() const
	{
		//the inverse set to the identity
		snMatrix44f I;
		I.identity();

		//the entry matrix
		snMatrix44f A;
		A = *this;

		//iterate through each line
		for (int k = 0; k < 4; k++)
		{
			//inverse line if a[k][k] = 0;
			if (A.m_r[k].m128_f32[k] == 0)
			{
				int i = 0;
				bool over = false;
				while (i < 4 && !over)
				{
					if (i == k)
					{
						i++;
						continue;
					}

					if (A.m_r[i].m128_f32[k] != 0)
					{
						over = true;

						//inverse row i and k
						for (int h = 0; h < 4; h++)
						{
							float temp = A.m_r[i].m128_f32[h];
							A.m_r[i].m128_f32[h] = A.m_r[k].m128_f32[h];
							A.m_r[k].m128_f32[h] = temp;

							temp = I.m_r[i].m128_f32[h];
							I.m_r[i].m128_f32[h] = I.m_r[k].m128_f32[h];
							I.m_r[k].m128_f32[h] = temp;
						}
					}
					i++;
				}
				//not inversible
				if (!over)
					return I;
			}

			//simplify the line to put a 1 in a[k][k]
			if (A.m_r[k].m128_f32[k] != 1)
			{
				float coeff = 1.f / A.m_r[k].m128_f32[k];
				for (int h = 0; h < 4; h++)
				{
					A.m_r[k].m128_f32[h] *= coeff;
					I.m_r[k].m128_f32[h] *= coeff;
				}
			}

			//pivot
			for (int i = 0; i < 4; i++)
			{
				if (i == k)
					continue;

				float coeff = A.m_r[i].m128_f32[k];

				for (int h = 0; h < 4; h++)
				{
					A.m_r[i].m128_f32[h] = A.m_r[i].m128_f32[h] - coeff * A.m_r[k].m128_f32[h];
					I.m_r[i].m128_f32[h] = I.m_r[i].m128_f32[h] - coeff * I.m_r[k].m128_f32[h];
				}
			}
		}

		return I;
	}

	/*addition*/
	snMatrix44f snMatrix44f::operator+(const snMatrix44f& m)const
	{
		snMatrix44f sum;
		for (int i = 0; i < ROW_COUNT; i++)
			sum[i] = m_r[i] + m[i];

		return sum;
	}

	/*substraction*/
	snMatrix44f snMatrix44f::operator-(const snMatrix44f& m)const
	{
		snMatrix44f sum;
		for (int i = 0; i < ROW_COUNT; i++)
			sum[i] = m_r[i] - m[i];

		return sum;
	}

	/*add and assign*/
	void snMatrix44f::operator+=(const snMatrix44f& m)
	{
		for (int i = 0; i < ROW_COUNT; i++)
			m_r[i] = m_r[i] + m[i];
	}

	/*sub and assign*/
	void snMatrix44f::operator-=(const snMatrix44f& m)
	{
		for (int i = 0; i < ROW_COUNT; i++)
			m_r[i] = m_r[i] - m[i];
	}

	/*assign*/
	void snMatrix44f::operator=(const snMatrix44f& m)const
	{
		//copy array to array
		memcpy((void*)m_r, m.m_r, sizeof(float)* 16);
	}

	/*create a translation matrix*/
	void snMatrix44f::createTranslation(float x, float y, float z)
	{
		identity();
		m_r[3] = snVec4Set(x, y, z, 1);
	}

	/*Create a translation matrix from a vector*/
	void snMatrix44f::createTranslation(const snVec& v)
	{
		identity();
		m_r[3] = v;
	}

	/*create a rotation matrix around X axis*/
	void snMatrix44f::createRotationX(float radiusAngle)
	{
		identity();
		float c = cos(radiusAngle);
		float s = sin(radiusAngle);
		m_r[1] = snVec4Set(0, c, s, 0);
		m_r[2] = snVec4Set(0, -s, c, 0);
	}

	/*create a rotation matrix around Y axis*/
	void snMatrix44f::createRotationY(float radiusAngle)
	{
		identity();
		float c = cos(radiusAngle);
		float s = sin(radiusAngle);
		m_r[0] = snVec4Set(c, 0, -s, 0);
		m_r[2] = snVec4Set(s, 0, c, 0);
	}

	/*create a rotation matrix around Z axis*/
	void snMatrix44f::createRotationZ(float radiusAngle)
	{
		identity();
		float c = cos(radiusAngle);
		float s = sin(radiusAngle);

		m_r[0] = snVec4Set(c, s, 0, 0);
		m_r[1] = snVec4Set(-s, c, 0, 0);
	}

	/*create a rotation matrix a specific axis*/
	void snMatrix44f::createRotation(const snVec& axis, float angle)
	{
		float c = cos(angle);
		float s = sin(angle);

		snVec axisSquared = axis * axis;
		snVec axisTimesS = axis * s;
		snVec axisTimesOneMinusC = axis * (1 - c);

		m_r[0] = snVec4Set(snVec4GetX(axisSquared) + (1 - snVec4GetX(axisSquared)) * c,
			snVec4GetX(axis) * snVec4GetY(axisTimesOneMinusC) - snVec4GetZ(axisTimesS),
			snVec4GetX(axis) * snVec4GetZ(axisTimesOneMinusC)+ snVec4GetY(axisTimesS),
			0);

		m_r[1] = snVec4Set(snVec4GetX(axis) * snVec4GetY(axisTimesOneMinusC)+ snVec4GetZ(axisTimesS),
			snVec4GetY(axisSquared) + (1 - snVec4GetY(axisSquared)) * c,
			snVec4GetY(axis) * snVec4GetZ(axisTimesOneMinusC) - snVec4GetX(axisTimesS),
			0);

		m_r[2] = snVec4Set(snVec4GetX(axis) * snVec4GetZ(axisTimesOneMinusC) - snVec4GetY(axisTimesS),
			snVec4GetY(axis) * snVec4GetZ(axisTimesOneMinusC) + snVec4GetX(axisTimesS),
			snVec4GetZ(axisSquared) + (1 - snVec4GetZ(axisSquared)) * c,
			0);

		m_r[3] = snVec4Set(0, 0, 0, 1);
	}

	/*create a non uniform scale matrix*/
	void snMatrix44f::createScale(const snVec& s)
	{
		identity();

		m_r[0].m128_f32[0] = snVec4GetX(s);
		m_r[1].m128_f32[1] = snVec4GetY(s);
		m_r[2].m128_f32[2] = snVec4GetZ(s);
	}

	/*create a uniform scale matrix*/
	void snMatrix44f::createScale(float s)
	{
		identity();

		m_r[0].m128_f32[0] = s;
		m_r[1].m128_f32[1] = s;
		m_r[2].m128_f32[2] = s;
	}

	/*fill the matrix using opengl model of matrix*/
	void snMatrix44f::getOpenGLMatrix(double* matrix)
	{
		matrix[0] = snVec4GetX(m_r[0]);
		matrix[1] = snVec4GetY(m_r[0]);
		matrix[2] = snVec4GetZ(m_r[0]);
		matrix[3] = snVec4GetW(m_r[0]);

		matrix[4] = snVec4GetX(m_r[1]);
		matrix[5] = snVec4GetY(m_r[1]);
		matrix[6] = snVec4GetZ(m_r[1]);
		matrix[7] = snVec4GetW(m_r[1]);

		matrix[8] = snVec4GetX(m_r[2]);
		matrix[9] = snVec4GetY(m_r[2]);
		matrix[10] = snVec4GetZ(m_r[2]);
		matrix[11] = snVec4GetW(m_r[2]);

		matrix[12] = snVec4GetX(m_r[3]);
		matrix[13] = snVec4GetY(m_r[3]);
		matrix[14] = snVec4GetZ(m_r[3]);
		matrix[15] = snVec4GetW(m_r[3]);

	}

	//calculate frenet matrix
	void snMatrix44f::createFrenet(const snVec& position, const snVec& direction, const snVec& up)
	{

		snVec normal = snVec3Cross(direction, up);
		normal = normal * -1;
		snVec3Normalize(normal);

		snVec binormal = snVec3Cross(direction, normal);
		snVec3Normalize(binormal);

		m_r[0] = normal;
		m_r[1] = binormal;
		m_r[2] = direction;
		m_r[3] = position;

		m_r[3].m128_f32[3] = 0;
		m_r[4] = snVec4Set(0, 0, 0, 1);
	}

	//build a matrix using three vectors as columns
	void snMatrix44f::buildColumn(const snVec& c1, const snVec& c2, const snVec& c3)
	{
		m_r[0] = snVec4Set(snVec4GetX(c1), snVec4GetX(c2), snVec4GetX(c3), 0);
		m_r[1] = snVec4Set(snVec4GetY(c1), snVec4GetY(c2), snVec4GetY(c3), 0);
		m_r[2] = snVec4Set(snVec4GetZ(c1), snVec4GetZ(c2), snVec4GetZ(c3), 0);
		m_r[3] = snVec4Set(0, 0, 0, 1);
	}

	//calculate the determinant
	float snMatrix44f::det()const
	{
		float detM0 = snVec4GetY(m_r[1]) * snVec4GetZ(m_r[2]) * snVec4GetW(m_r[3]) +
			snVec4GetZ(m_r[1]) * snVec4GetW(m_r[2]) * snVec4GetY(m_r[3]) +
			snVec4GetW(m_r[1]) * snVec4GetY(m_r[2]) * snVec4GetZ(m_r[3]) -
			snVec4GetW(m_r[1]) * snVec4GetZ(m_r[2]) * snVec4GetY(m_r[3]) -
			snVec4GetW(m_r[2]) * snVec4GetZ(m_r[3]) * snVec4GetY(m_r[1]) -
			snVec4GetW(m_r[3]) * snVec4GetZ(m_r[1]) * snVec4GetY(m_r[2]);

		float detM1 = snVec4GetX(m_r[1]) * snVec4GetZ(m_r[2]) * snVec4GetW(m_r[3]) +
			snVec4GetX(m_r[2]) * snVec4GetY(m_r[3]) * snVec4GetW(m_r[1]) +
			snVec4GetZ(m_r[1]) * snVec4GetW(m_r[2]) * snVec4GetX(m_r[3]) -
			snVec4GetW(m_r[1]) * snVec4GetZ(m_r[2]) * snVec4GetX(m_r[3]) -
			snVec4GetW(m_r[2]) * snVec4GetY(m_r[3]) * snVec4GetX(m_r[1]) -
			snVec4GetW(m_r[3]) * snVec4GetZ(m_r[1]) * snVec4GetX(m_r[2]);

		float detM2 = snVec4GetX(m_r[1]) * snVec4GetY(m_r[2]) * snVec4GetW(m_r[3]) +
			snVec4GetX(m_r[2]) * snVec4GetY(m_r[3]) * snVec4GetW(m_r[1]) +
			snVec4GetY(m_r[1]) * snVec4GetW(m_r[2]) * snVec4GetX(m_r[3]) -
			snVec4GetW(m_r[1]) * snVec4GetY(m_r[2]) * snVec4GetX(m_r[3]) -
			snVec4GetW(m_r[2]) * snVec4GetY(m_r[3]) * snVec4GetX(m_r[1]) -
			snVec4GetW(m_r[3]) * snVec4GetY(m_r[1]) * snVec4GetX(m_r[2]);

		float detM3 = snVec4GetX(m_r[1]) * snVec4GetY(m_r[2]) * snVec4GetZ(m_r[3]) +
			snVec4GetX(m_r[2]) * snVec4GetY(m_r[3]) * snVec4GetZ(m_r[1]) +
			snVec4GetY(m_r[1]) * snVec4GetZ(m_r[2]) * snVec4GetX(m_r[3]) -
			snVec4GetZ(m_r[1]) * snVec4GetY(m_r[2]) * snVec4GetX(m_r[3]) -
			snVec4GetZ(m_r[2]) * snVec4GetY(m_r[3]) * snVec4GetX(m_r[1]) -
			snVec4GetZ(m_r[3]) * snVec4GetY(m_r[1]) * snVec4GetX(m_r[2]);

		return snVec4GetX(m_r[0]) * detM0 - snVec4GetY(m_r[0]) * detM1 + snVec4GetZ(m_r[0]) * detM2 - snVec4GetW(m_r[0]) * detM3;
	}

	void snMatrix44f::transpose(snMatrix44f& _transpose) const
	{
		// Let's say we start with this matrix:
		// r0 = 00 01 02 03
		// r1 = 10 11 12 13
		// r2 = 20 21 22 23
		// r3 = 30 31 32 33
		//
		// Then we compute shuffle 1 and 2 like this:
		// s1 = 00 01 10 11 //shuffle(r0, r1, shuffle(1, 0, 1, 0))
		// s2 = 20 21 30 31 //shuffle(r2, r3, shuffle(1, 0, 1, 0))
		//
		// From this we can compute the first two rows of the transpose:
		// transpose row 1 = 00 10 20 30 //shuffle(s1, s2, shuffle(2, 0, 2, 0))
		// transpose row 2 = 01 11 21 31 //shuffle(s1, s2, shuffle(3, 1, 3, 1))
		//
		// We repeat the process to get transpose row 2 and 3:
		// s1 = 02 03 12 13 //shuffle(r0, r1, shuffle(3, 2, 3, 2))
		// s2 = 22 23 32 33 //shuffle(r2, r3, shuffle(3, 2, 3, 2))
		// transpose row 2 = 02 12 22 32 //shuffle(s1, s2, shuffle(2, 0, 2, 0))
		// transpose row 3 = 03 13 23 33 //shuffle(s1, s2, shuffle(3, 1, 3, 1))

		__m128 s1 = _mm_shuffle_ps(m_r[0], m_r[1], _MM_SHUFFLE(1, 0, 1, 0));
		__m128 s2 = _mm_shuffle_ps(m_r[2], m_r[3], _MM_SHUFFLE(1, 0, 1, 0));

		_transpose.m_r[0] = _mm_shuffle_ps(s1, s2, _MM_SHUFFLE(2, 0, 2, 0));
		_transpose.m_r[1] = _mm_shuffle_ps(s1, s2, _MM_SHUFFLE(3, 1, 3, 1));

		s1 = _mm_shuffle_ps(m_r[0], m_r[1], _MM_SHUFFLE(3, 2, 3, 2));
		s2 = _mm_shuffle_ps(m_r[2], m_r[3], _MM_SHUFFLE(3, 2, 3, 2));

		_transpose.m_r[2] = _mm_shuffle_ps(s1, s2, _MM_SHUFFLE(2, 0, 2, 0));
		_transpose.m_r[3] = _mm_shuffle_ps(s1, s2, _MM_SHUFFLE(3, 1, 3, 1));
	}

	void snMatrix44f::createRotationFromQuaternion(const snVec& _q)
	{
		const __m128 Constant1110 = { 1.0f, 1.0f, 1.0f, 0.0f };
		const __m128 Mask = _mm_castsi128_ps(_mm_set_epi32(0, -1, -1, -1));
		
		__m128 Q0 = _mm_add_ps(_q, _q);
		__m128 Q1 = _mm_mul_ps(_q, Q0);

		__m128 V0 = _mm_shuffle_ps(Q1, Q1, _MM_SHUFFLE(3, 0, 0, 1));
		V0 = _mm_and_ps(V0, Mask);
		__m128 V1 = _mm_shuffle_ps(Q1, Q1, _MM_SHUFFLE(3, 1, 2, 2));
		V1 = _mm_and_ps(V1, Mask);
		__m128 R0 = _mm_sub_ps(Constant1110, V0);
		R0 = _mm_sub_ps(R0, V1);

		V0 = _mm_shuffle_ps(_q, _q, _MM_SHUFFLE(3, 1, 0, 0));
		V1 = _mm_shuffle_ps(Q0, Q0, _MM_SHUFFLE(3, 2, 1, 2));
		V0 = _mm_mul_ps(V0, V1);

		V1 = _mm_shuffle_ps(_q, _q, _MM_SHUFFLE(3, 3, 3, 3));
		__m128 V2 = _mm_shuffle_ps(Q0, Q0, _MM_SHUFFLE(3, 0, 2, 1));
		V1 = _mm_mul_ps(V1, V2);

		__m128 R1 = _mm_add_ps(V0, V1);
		__m128 R2 = _mm_sub_ps(V0, V1);

		V0 = _mm_shuffle_ps(R1, R2, _MM_SHUFFLE(1, 0, 2, 1));
		V0 = _mm_shuffle_ps(V0, V0, _MM_SHUFFLE(1, 3, 2, 0));
		V1 = _mm_shuffle_ps(R1, R2, _MM_SHUFFLE(2, 2, 0, 0));
		V1 = _mm_shuffle_ps(V1, V1, _MM_SHUFFLE(2, 0, 2, 0));

		Q1 = _mm_shuffle_ps(R0, V0, _MM_SHUFFLE(1, 0, 3, 0));
		Q1 = _mm_shuffle_ps(Q1, Q1, _MM_SHUFFLE(1, 3, 2, 0));

		m_r[0] = Q1;

		Q1 = _mm_shuffle_ps(R0, V0, _MM_SHUFFLE(3, 2, 3, 1));
		Q1 = _mm_shuffle_ps(Q1, Q1, _MM_SHUFFLE(1, 3, 0, 2));
		m_r[1] = Q1;

		Q1 = _mm_shuffle_ps(V1, R0, _MM_SHUFFLE(3, 2, 1, 0));
		m_r[2] = Q1;
		m_r[3] = snVec4Set(0, 0, 0, 1);
	}

	snVec snMatrix44f::getScale() const
	{
		return snVec4Set(snVec3Norme(m_r[0]), snVec3Norme(m_r[1]), snVec3Norme(m_r[2]), 0);
	}

	snVec snMatrixTransform3(const snVec& _v, const snMatrix44f& _m)
	{
		//compute v.[i] * m[i] = r[i] the res = sum(r[i])

		__m128 vTemp = _mm_shuffle_ps(_v, _v, _MM_SHUFFLE(0, 0, 0, 0));
		__m128 res = _mm_mul_ps(vTemp, _m.m_r[0]);

		vTemp = _mm_shuffle_ps(_v, _v, _MM_SHUFFLE(1, 1, 1, 1));
		__m128 mul = _mm_mul_ps(vTemp, _m.m_r[1]);
		res = _mm_add_ps(res, mul);

		vTemp = _mm_shuffle_ps(_v, _v, _MM_SHUFFLE(2, 2, 2, 2));
		mul = _mm_mul_ps(vTemp, _m.m_r[2]);
		res = _mm_add_ps(res, mul);

		return res;
	}

	snVec snMatrixTransform4(const snVec& _v, const snMatrix44f& _m)
	{
		//compute v.[i] * m[i] = r[i] the res = sum(r[i])
		__m128 vTemp = _mm_shuffle_ps(_v, _v, _MM_SHUFFLE(0, 0, 0, 0));
		__m128 res = _mm_mul_ps(vTemp, _m[0]);

		vTemp = _mm_shuffle_ps(_v, _v, _MM_SHUFFLE(1, 1, 1, 1));
		__m128 mul = _mm_mul_ps(vTemp, _m[1]);
		res = _mm_add_ps(res, mul);

		vTemp = _mm_shuffle_ps(_v, _v, _MM_SHUFFLE(2, 2, 2, 2));
		mul = _mm_mul_ps(vTemp, _m[2]);
		res = _mm_add_ps(res, mul);

		vTemp = _mm_shuffle_ps(_v, _v, _MM_SHUFFLE(3, 3, 3, 3));
		mul = _mm_mul_ps(vTemp, _m[3]);
		res = _mm_add_ps(res, mul);

		return res;
	}

	snVec snMatrixTransform3(const snMatrix44f& _m, const snVec& _v)
	{
		//compute first matrix column time X
		snVec x = snVec3Dot(_v, _m[0]);
		snVec y = snVec3Dot(_v, _m[1]);
		snVec z = snVec3Dot(_v, _m[2]);

		snVec shuffle = _mm_shuffle_ps(x, y, _MM_SHUFFLE(VEC_ID_X, VEC_ID_X, VEC_ID_X, VEC_ID_X));
		return _mm_shuffle_ps(shuffle, z, _MM_SHUFFLE(VEC_ID_W, VEC_ID_X, VEC_ID_Z, VEC_ID_X));
	}

	void snMatrixMultiply4(const snMatrix44f& _m1, const snMatrix44f& _m2, snMatrix44f& _res)
	{
		_res[0] = snMatrixTransform4(_m1[0], _m2);
		_res[1] = snMatrixTransform4(_m1[1], _m2);
		_res[2] = snMatrixTransform4(_m1[2], _m2);
		_res[3] = snMatrixTransform4(_m1[3], _m2);
	}

	void snMatrixMultiply3(const snMatrix44f& _m1, const snMatrix44f& _m2, snMatrix44f& _res)
	{
		_res[0] = snMatrixTransform4(_m1[0], _m2);
		_res[1] = snMatrixTransform4(_m1[1], _m2);
		_res[2] = snMatrixTransform4(_m1[2], _m2);
		_res[3] = snVec4Set(0, 0, 0, 1);
	}

	snVec snMatrixGetTranslation(const snMatrix44f& _transform)
	{
		return _transform.m_r[3];
	}

	void snMatrixCreateTransform(const snMatrix44f& _rotation, const snVec& _translation, snMatrix44f& _transform)
	{
		_transform.m_r[0] = _rotation.m_r[0];
		_transform.m_r[1] = _rotation.m_r[1];
		_transform.m_r[2] = _rotation.m_r[2];
		_transform.m_r[3] = _translation;
	}
}