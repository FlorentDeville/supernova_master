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

#ifndef SN_MATRIX44F_H
#define SN_MATRIX44F_H

#include <xmmintrin.h>
#include "snGlobals.h"
#include "snVector4f-inl.h"

namespace Supernova
{

	/*Represent a 4x4 matrix*/
	class SN_ALIGN snMatrix44f
	{
	public:

		static const int ROW_COUNT = 4;

		//array representing the matrix. Each element is a row so the representation is row major.
		snVector4f m_r[ROW_COUNT];

		static const snMatrix44f m_zero;

	public:
		/*Default constructor*/
		snMatrix44f();

		/*Destructor*/
		~snMatrix44f();

		/*Display in std out*/
		void display()const;

		/*Set the matrix to the identity*/
		void identity();

		/*Calculate the inverse matrix*/
		snMatrix44f inverse() const;

		/*addition*/
		snMatrix44f operator+(const snMatrix44f& m)const;

		/*substraction*/
		snMatrix44f operator-(const snMatrix44f& m)const;

		/*add and assign*/
		void operator+=(const snMatrix44f& m);

		/*sub and assign*/
		void operator-=(const snMatrix44f& m);

		/*assign*/
		void operator=(const snMatrix44f& m)const;

		/*Get the row with the index _id*/
		inline snVector4f& operator[](int _id);

		/*Get the row with the index _id*/
		inline snVector4f operator[](int _id) const;

		/*create a translation matrix*/
		void createTranslation(float x, float y, float z);

		/*Create a translation matrix from a vector*/
		void createTranslation(const snVector4f& v);

		/*create a rotation matrix around X axis*/
		void createRotationX(float radiusAngle);

		/*create a rotation matrix around Y axis*/
		void createRotationY(float radiusAngle);

		/*create a rotation matrix around Z axis*/
		void createRotationZ(float radiusAngle);

		/*create a rotation matrix a specific axis*/
		void createRotation(const snVector4f& axis, float angle);

		/*create a non uniform scale matrix*/
		void createScale(const snVector4f& s);

		/*create a uniform scale matrix*/
		void createScale(float s);

		//convert the matrix to an opengl matrix (column major)
		void getOpenGLMatrix(double* matrix);

		//calculate frenet matrix
		void createFrenet(const snVector4f& position, const snVector4f& direction, const snVector4f& up);

		//build a matrix using three vectors as columns
		void buildColumn(const snVector4f& c1, const snVector4f& c2, const snVector4f& c3);

		//calculate the determinant
		float det()const;

		//compute the transpose matrix
		void transpose(snMatrix44f& _transpose) const;

		//Compute the rotation matrix from a quaternion
		void createRotationFromQuaternion(const snVector4f& _q);
	};

	snVector4f& snMatrix44f::operator[](int _id)
	{
		return m_r[_id];
	}

	snVector4f snMatrix44f::operator[](int _id) const
	{
		return m_r[_id];
	}

	//Compute the product between a row vector and a matrix. The fourth element of the vector and the fourth line of the matrix are ignored.
	snVector4f snMatrixTransform3(const snVector4f& _v, const snMatrix44f& _m);

	//Compute the product between a row vector and a matrix.
	snVector4f snMatrixTransform4(const snVector4f& _v, const snMatrix44f& _m);

	//Compute the product between a matrix and a column vector. The fourth element of the vector and the fourth line of the matrix are ignored.
	snVector4f snMatrixTransform3(const snMatrix44f& _m, const snVector4f& _v);

	//Multiply two matrices _m1 and _m2 and set the result in _res.
	void snMatrixMultiply4(const snMatrix44f& _m1, const snMatrix44f& _m2, snMatrix44f& _res);

	//Multiply two matrices _m1 and _m2 and set the result in _res. The matrices are considered to be 3x3 so the last row and column are ignored.
	void snMatrixMultiply3(const snMatrix44f& _m1, const snMatrix44f& _m2, snMatrix44f& _res);
}

#endif // ifndef SN_MATRIX44F_H