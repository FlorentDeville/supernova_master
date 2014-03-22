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
#include "snVector4f.h"

namespace Supernova
{

	/*Represent a 4x4 matrix*/
	class SN_ALIGN snMatrix44f
	{
	public:

		//array representing the matrix. Each element is a row so the representation is row major.
		snVector4f m_r[4];

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

		/*multiplication*/
		snMatrix44f operator*(const snMatrix44f& m)const;

		/*Multiplication with a vector*/
		snVector4f operator*(const snVector4f& v)const;

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

	//Compute the product between a vector and a matrix. The fourth element of the vector and the fourth line of the matrix are ignored.
	snVector4f snMatrixTransform3(const snVector4f& _v, const snMatrix44f& _m);
}

#endif // SN_MATRIX44F_H