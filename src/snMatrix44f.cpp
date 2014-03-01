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


#include <iostream>
using std::cout;
using std::endl;

#include <math.h>
#include "snVector4f.h"

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
			if (A.m_r[k].m_vec.m128_f32[k] == 0)
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

					if (A.m_r[i].m_vec.m128_f32[k] != 0)
					{
						over = true;

						//inverse row i and k
						for (int h = 0; h < 4; h++)
						{
							float temp = A.m_r[i].m_vec.m128_f32[h];
							A.m_r[i].m_vec.m128_f32[h] = A.m_r[k].m_vec.m128_f32[h];
							A.m_r[k].m_vec.m128_f32[h] = temp;

							temp = I.m_r[i].m_vec.m128_f32[h];
							I.m_r[i].m_vec.m128_f32[h] = I.m_r[k].m_vec.m128_f32[h];
							I.m_r[k].m_vec.m128_f32[h] = temp;
						}
					}
					i++;
				}
				//not inversible
				if (!over)
					return I;
			}

			//simplify the line to put a 1 in a[k][k]
			if (A.m_r[k].m_vec.m128_f32[k] != 1)
			{
				float coeff = 1.f / A.m_r[k].m_vec.m128_f32[k];
				for (int h = 0; h < 4; h++)
				{
					A.m_r[k].m_vec.m128_f32[h] *= coeff;
					I.m_r[k].m_vec.m128_f32[h] *= coeff;
				}
			}

			//pivot
			for (int i = 0; i < 4; i++)
			{
				if (i == k)
					continue;

				float coeff = A.m_r[i].m_vec.m128_f32[k];

				for (int h = 0; h < 4; h++)
				{
					A.m_r[i].m_vec.m128_f32[h] = A.m_r[i].m_vec.m128_f32[h] - coeff * A.m_r[k].m_vec.m128_f32[h];
					I.m_r[i].m_vec.m128_f32[h] = I.m_r[i].m_vec.m128_f32[h] - coeff * I.m_r[k].m_vec.m128_f32[h];
				}
			}
		}

		return I;
	}

	/*Display in std out*/
	void snMatrix44f::display()const
	{
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
				cout << m_r[i].m_vec.m128_f32[j] << " ";

			cout << endl;
		}
	}

	/*addition*/
	snMatrix44f snMatrix44f::operator+(const snMatrix44f& m)const
	{
		snMatrix44f sum;
		for (int i = 0; i < 4; i++)
			sum.m_r[i] = m_r[i] + m.m_r[i];

		return sum;
	}

	/*substraction*/
	snMatrix44f snMatrix44f::operator-(const snMatrix44f& m)const
	{
		snMatrix44f sum;
		for (int i = 0; i < 4; i++)
			sum.m_r[i] = m_r[i] - m.m_r[i];

		return sum;
	}

	/*multiplication*/
	snMatrix44f snMatrix44f::operator*(const snMatrix44f& m)const
	{
		snMatrix44f res;

		//for each row of the first matrix
		for (int i = 0; i < 4; i++) //row
		{
			//for each column of the second matrix
			for (int j = 0; j < 4; j++) //column
			{
				//create a vector containing the column of the second matrix
				snVector4f col = snVector4f(m.m_r[0].m_vec.m128_f32[j], m.m_r[1].m_vec.m128_f32[j],
					m.m_r[2].m_vec.m128_f32[j], m.m_r[3].m_vec.m128_f32[j]);

				//compute the value of res[i][j]
				res.m_r[i].m_vec.m128_f32[j] = m_r[i].dot4(col);
			}
		}

		return res;
	}

	/*Multiplication with a vector*/
	snVector4f snMatrix44f::operator*(const snVector4f& v)const
	{
		snVector4f col0(m_r[0].getX(), m_r[1].getX(), m_r[2].getX(), m_r[3].getX());
		snVector4f col1(m_r[0].getY(), m_r[1].getY(), m_r[2].getY(), m_r[3].getY());
		snVector4f col2(m_r[0].getZ(), m_r[1].getZ(), m_r[2].getZ(), m_r[3].getZ());
		snVector4f col3(m_r[0].getW(), m_r[1].getW(), m_r[2].getW(), m_r[3].getW());

		snVector4f res(v.dot4(col0), v.dot4(col1), v.dot4(col2), v.dot4(col3));

		return res;
	}

	/*add and assign*/
	void snMatrix44f::operator+=(const snMatrix44f& m)
	{
		for (int i = 0; i < 4; i++)
			m_r[i] = m_r[i] + m.m_r[i];
	}

	/*sub and assign*/
	void snMatrix44f::operator-=(const snMatrix44f& m)
	{
		for (int i = 0; i < 4; i++)
			m_r[i] = m_r[i] - m.m_r[i];
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
		m_r[3] = snVector4f(x, y, z, 1);
	}

	/*Create a translation matrix from a vector*/
	void snMatrix44f::createTranslation(const snVector4f& v)
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
		m_r[1] = snVector4f(0, c, s, 0);
		m_r[2] = snVector4f(0, -s, c, 0);
	}

	/*create a rotation matrix around Y axis*/
	void snMatrix44f::createRotationY(float radiusAngle)
	{
		identity();
		float c = cos(radiusAngle);
		float s = sin(radiusAngle);
		m_r[0] = snVector4f(c, 0, -s, 0);
		m_r[2] = snVector4f(s, 0, c, 0);
	}

	/*create a rotation matrix around Z axis*/
	void snMatrix44f::createRotationZ(float radiusAngle)
	{
		identity();
		float c = cos(radiusAngle);
		float s = sin(radiusAngle);

		m_r[0] = snVector4f(c, s, 0, 0);
		m_r[1] = snVector4f(-s, c, 0, 0);
	}

	/*create a rotation matrix a specific axis*/
	void snMatrix44f::createRotation(const snVector4f& axis, float angle)
	{
		float c = cos(angle);
		float s = sin(angle);

		snVector4f axisSquared = axis * axis;
		snVector4f axisTimesS = axis * s;
		snVector4f axisTimesOneMinusC = axis * (1 - c);

		m_r[0] = snVector4f(axisSquared.getX() + (1 - axisSquared.getX()) * c,
			axis.getX() * axisTimesOneMinusC.getY() - axisTimesS.getZ(),
			axis.getX() * axisTimesOneMinusC.getZ() + axisTimesS.getY(),
			0);

		m_r[1] = snVector4f(axis.VEC4FX * axisTimesOneMinusC.getY() + axisTimesS.getZ(),
			axisSquared.getY() + (1 - axisSquared.getY()) * c,
			axis.VEC4FY * axisTimesOneMinusC.getZ() - axisTimesS.getX(),
			0);

		m_r[2] = snVector4f(axis.VEC4FX * axisTimesOneMinusC.getZ() - axisTimesS.getY(),
			axis.VEC4FY * axisTimesOneMinusC.getZ() + axisTimesS.getX(),
			axisSquared.getZ() + (1 - axisSquared.getZ()) * c,
			0);

		m_r[3] = snVector4f(0, 0, 0, 1);
	}

	/*create a non uniform scale matrix*/
	void snMatrix44f::createScale(const snVector4f& s)
	{
		identity();

		m_r[0].m_vec.m128_f32[0] = s.getX();
		m_r[1].m_vec.m128_f32[1] = s.getY();
		m_r[2].m_vec.m128_f32[2] = s.getZ();
	}

	/*create a uniform scale matrix*/
	void snMatrix44f::createScale(float s)
	{
		identity();

		m_r[0].m_vec.m128_f32[0] = s;
		m_r[1].m_vec.m128_f32[1] = s;
		m_r[2].m_vec.m128_f32[2] = s;
	}

	/*fill the matrix using opengl model of matrix*/
	void snMatrix44f::getOpenGLMatrix(double* matrix)
	{
		matrix[0] = m_r[0].getX();
		matrix[1] = m_r[0].getY();
		matrix[2] = m_r[0].getZ();
		matrix[3] = m_r[0].getW();

		matrix[4] = m_r[1].getX();
		matrix[5] = m_r[1].getY();
		matrix[6] = m_r[1].getZ();
		matrix[7] = m_r[1].getW();

		matrix[8] = m_r[2].getX();
		matrix[9] = m_r[2].getY();
		matrix[10] = m_r[2].getZ();
		matrix[11] = m_r[2].getW();

		matrix[12] = m_r[3].getX();
		matrix[13] = m_r[3].getY();
		matrix[14] = m_r[3].getZ();
		matrix[15] = m_r[3].getW();

	}

	//calculate frenet matrix
	void snMatrix44f::createFrenet(const snVector4f& position, const snVector4f& direction, const snVector4f& up)
	{

		snVector4f normal = snVector4f::cross(direction, up);
		normal = normal * -1;
		normal.normalize();

		snVector4f binormal = snVector4f::cross(direction, normal);
		binormal.normalize();

		m_r[0] = normal;
		m_r[1] = binormal;
		m_r[2] = direction;
		m_r[3] = position;

		m_r[3].m_vec.m128_f32[3] = 0;
		m_r[4] = snVector4f(0, 0, 0, 1);
	}

	//build a matrix using three vectors as columns
	void snMatrix44f::buildColumn(const snVector4f& c1, const snVector4f& c2, const snVector4f& c3)
	{
		m_r[0] = snVector4f(c1.getX(), c2.getX(), c3.getX(), 0);
		m_r[1] = snVector4f(c1.getY(), c2.getY(), c3.getY(), 0);
		m_r[2] = snVector4f(c1.getZ(), c2.getZ(), c3.getZ(), 0);
		m_r[3] = snVector4f(0, 0, 0, 1);
	}

	//calculate the determinant
	float snMatrix44f::det()const
	{
		float detM0 = m_r[1].getY() * m_r[2].getZ() * m_r[3].getW() +
			m_r[1].getZ() * m_r[2].getW() * m_r[3].getY() +
			m_r[1].getW() * m_r[2].getY() * m_r[3].getZ() -
			m_r[1].getW() * m_r[2].getZ() * m_r[3].getY() -
			m_r[2].getW() * m_r[3].getZ() * m_r[1].getY() -
			m_r[3].getW() * m_r[1].getZ() * m_r[2].getY();

		float detM1 = m_r[1].getX() * m_r[2].getZ() * m_r[3].getW() +
			m_r[2].getX() * m_r[3].getY() * m_r[1].getW() +
			m_r[1].getZ() * m_r[2].getW() * m_r[3].getX() -
			m_r[1].getW() * m_r[2].getZ() * m_r[3].getX() -
			m_r[2].getW() * m_r[3].getY() * m_r[1].getX() -
			m_r[3].getW() * m_r[1].getZ() * m_r[2].getX();

		float detM2 = m_r[1].getX() * m_r[2].getY() * m_r[3].getW() +
			m_r[2].getX() * m_r[3].getY() * m_r[1].getW() +
			m_r[1].getY() * m_r[2].getW() * m_r[3].getX() -
			m_r[1].getW() * m_r[2].getY() * m_r[3].getX() -
			m_r[2].getW() * m_r[3].getY() * m_r[1].getX() -
			m_r[3].getW() * m_r[1].getY() * m_r[2].getX();

		float detM3 = m_r[1].getX() * m_r[2].getY() * m_r[3].getZ() +
			m_r[2].getX() * m_r[3].getY() * m_r[1].getZ() +
			m_r[1].getY() * m_r[2].getZ() * m_r[3].getX() -
			m_r[1].getZ() * m_r[2].getY() * m_r[3].getX() -
			m_r[2].getZ() * m_r[3].getY() * m_r[1].getX() -
			m_r[3].getZ() * m_r[1].getY() * m_r[2].getX();

		return m_r[0].getX() * detM0 - m_r[0].getY() * detM1 + m_r[0].getZ() * detM2 - m_r[0].getW() * detM3;
	}

	void snMatrix44f::transpose(snMatrix44f& _transpose) const
	{
		_transpose.m_r[0] = snVector4f(m_r[0].getX(), m_r[1].getX(), m_r[2].getX(), m_r[3].getX());
		_transpose.m_r[1] = snVector4f(m_r[0].getY(), m_r[1].getY(), m_r[2].getY(), m_r[3].getY());
		_transpose.m_r[2] = snVector4f(m_r[0].getZ(), m_r[1].getZ(), m_r[2].getZ(), m_r[3].getZ());
		_transpose.m_r[3] = snVector4f(m_r[0].getW(), m_r[1].getW(), m_r[2].getW(), m_r[3].getW());
	}

	void snMatrix44f::createRotationFromQuaternion(const snVector4f& _q)
	{
		const __m128 Constant1110 = { 1.0f, 1.0f, 1.0f, 0.0f };
		const __m128 Mask = _mm_castsi128_ps(_mm_set_epi32(0, -1, -1, -1));
		
		__m128 Q0 = _mm_add_ps(_q.m_vec, _q.m_vec);
		__m128 Q1 = _mm_mul_ps(_q.m_vec, Q0);

		__m128 V0 = _mm_shuffle_ps(Q1, Q1, _MM_SHUFFLE(3, 0, 0, 1));
		V0 = _mm_and_ps(V0, Mask);
		__m128 V1 = _mm_shuffle_ps(Q1, Q1, _MM_SHUFFLE(3, 1, 2, 2));
		V1 = _mm_and_ps(V1, Mask);
		__m128 R0 = _mm_sub_ps(Constant1110, V0);
		R0 = _mm_sub_ps(R0, V1);

		V0 = _mm_shuffle_ps(_q.m_vec, _q.m_vec, _MM_SHUFFLE(3, 1, 0, 0));
		V1 = _mm_shuffle_ps(Q0, Q0, _MM_SHUFFLE(3, 2, 1, 2));
		V0 = _mm_mul_ps(V0, V1);

		V1 = _mm_shuffle_ps(_q.m_vec, _q.m_vec, _MM_SHUFFLE(3, 3, 3, 3));
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
		m_r[3] = { 0, 0, 0, 1 };
	}

	snVector4f snMatrixTransform3(const snVector4f& _v, const snMatrix44f& _m)
	{
		//compute v.[i] * m[i] = r[i] the res = sum(r[i])

		__m128 vTemp = _mm_shuffle_ps(_v.m_vec, _v.m_vec, _MM_SHUFFLE(0, 0, 0, 0));
		__m128 res = _mm_mul_ps(vTemp, _m.m_r[0].m_vec);

		vTemp = _mm_shuffle_ps(_v.m_vec, _v.m_vec, _MM_SHUFFLE(1, 1, 1, 1));
		__m128 mul = _mm_mul_ps(vTemp, _m.m_r[1].m_vec);
		res = _mm_add_ps(res, mul);

		vTemp = _mm_shuffle_ps(_v.m_vec, _v.m_vec, _MM_SHUFFLE(2, 2, 2, 2));
		mul = _mm_mul_ps(vTemp, _m.m_r[2].m_vec);
		res = _mm_add_ps(res, mul);

		return res;
	}
}