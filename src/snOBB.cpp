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

//Warning because m_box is in the initialization list. From the MSDN, the warning can be disabled.
#pragma warning( disable : 4351)

#include "snOBB.h"

#include <assert.h>
#include <list>
using std::list;

#include "snMath.h"
#include "snAABB.h"
#include "snMatrix44f.h"
#include "snTransform.h"

using namespace Supernova::Vector;

namespace Supernova
{
	snOBB::snOBB(const snVec& _extends) : snICollider(), m_box(), m_pos(snVec4Set(0)), m_extends(_extends)
	{
		m_typeOfCollider = snEColliderBox;
	}

	snOBB::~snOBB()
	{}

	void snOBB::initialize()
	{
		computeVertices();
	}

	snVec snOBB::getPosition() const
	{
		return m_pos;
	}

	snVec snOBB::getExtends() const
	{
		return m_extends;
	}

	void snOBB::computeVertices()
	{
		float halfX = snVec4GetX(m_extends);
		float halfY = snVec4GetY(m_extends);
		float halfZ = snVec4GetZ(m_extends);

		m_box[0] = snVec4Set(halfX, halfY, -halfZ, 1);
		m_box[1] = snVec4Set(halfX, -halfY, -halfZ, 1);
		m_box[2] = snVec4Set(-halfX, -halfY, -halfZ, 1);
		m_box[3] = snVec4Set(-halfX, halfY, -halfZ, 1);

		m_box[4] = snVec4Set(halfX, halfY, halfZ, 1);
		m_box[5] = snVec4Set(halfX, -halfY, halfZ, 1);
		m_box[6] = snVec4Set(-halfX, -halfY, halfZ, 1);
		m_box[7] = snVec4Set(-halfX, halfY, halfZ, 1);

		//front
		m_idFaces[0] = 3; // { 0, 1, 2, 3,    4, 5, 6, 7,    0, 4, 5, 1,   7, 3, 2, 6,   7, 4, 0, 3,   2, 6, 5, 1 };
		m_idFaces[1] = 2;
		m_idFaces[2] = 1;
		m_idFaces[3] = 0;

		//right
		m_idFaces[4] = 1;
		m_idFaces[5] = 5;
		m_idFaces[6] = 4;
		m_idFaces[7] = 0;

		//up
		m_idFaces[8] = 3;
		m_idFaces[9] = 0;
		m_idFaces[10] = 4;
		m_idFaces[11] = 7;

		//down
		m_idFaces[12] = 1;
		m_idFaces[13] = 2;
		m_idFaces[14] = 6;
		m_idFaces[15] = 5;

		//back
		m_idFaces[16] = 7;
		m_idFaces[17] = 4;
		m_idFaces[18] = 5;
		m_idFaces[19] = 6;

		//left
		m_idFaces[20] = 6;
		m_idFaces[21] = 2;
		m_idFaces[22] = 3;
		m_idFaces[23] = 7;
	}

	void snOBB::updateFromTransform()
	{
		for (int i = 0; i < 8; ++i)
			m_worldBox[i] = snMatrixTransform4(m_box[i], m_transform.getLocalToWorld());

		//world origin is the last row.
		m_pos = m_transform.getPosition();

		//world normals are just the rows of the transform matrix
		m_normals[0] = m_transform.getRight();
		m_normals[1] = m_transform.getUp();
		m_normals[2] = m_transform.getForward();
	}

	void snOBB::computeLocalInertiaTensor(float _mass, snMatrix44f& _inertiaTensor) const
	{
		snVec squaredSize = m_extends * m_extends * 4;// size * size = (extends * 2) * (extends * 2) = extends * extends * 4;

		float massOverTwelve = _mass / 12.f;
		_inertiaTensor.m_r[0] = snVec4Set(massOverTwelve * (snVec4GetY(squaredSize) + snVec4GetZ(squaredSize)), 0, 0, 0);
		_inertiaTensor.m_r[1] = snVec4Set(0, massOverTwelve * (snVec4GetX(squaredSize) + snVec4GetZ(squaredSize)), 0, 0);
		_inertiaTensor.m_r[2] = snVec4Set(0, 0, massOverTwelve * (snVec4GetX(squaredSize) + snVec4GetY(squaredSize)), 0);
		_inertiaTensor.m_r[3] = snVec4Set(0, 0, 0, 1);
	}

	void snOBB::projectToAxis(const snVec& _direction, float& _min, float& _max) const
	{
		//////////////////////////////////////////////////////////////////////
		// The basic idea would be to compute the dot product of every point. Instead, convert the array of vector into 
		// a vector of array (or array of struct to struct of array).
		// So instead of having points this way:
		// P1x P1y P1z P1w (vector1)
		// P2x P2y P2z P2w (vector2)
		// P3x P3y P3z P3w (vector3)
		// P4x P4y P4z P4w (vector4)
		//
		// I turn them this way :
		// P1x P2x P3x P4x (vector1)
		// P1y P2y P3y P4y (vector2)
		// P1z P2z P3z P4z (vector3)
		// P1w P2w P3w P4w (vector4)
		// 
		// and direction:
		// dx dx dx dx (d1)
		// dy dy dy dy (d2)
		// dz dz dz dz (d3)
		// dw dw dw dw (d4)
		//
		// the dot products become
		// dot = vector1 * d1 + vector2 * d2 + vector3 * d3
		///////////////////////////////////////////////////////////////////////////

		snVec temp[4] = { _direction, _direction, _direction, _direction };
		snVec dir[4];
		arrayOfStructToStructOfArray(temp, dir);

		snVec structOfArray1[4];
		snVec structOfArray2[4];
		arrayOfStructToStructOfArray(m_worldBox, structOfArray1);
		arrayOfStructToStructOfArray(m_worldBox + 4, structOfArray2);

		snVec dot1 = (structOfArray1[0] * dir[0]) +
			(structOfArray1[1] * dir[1]) +
			(structOfArray1[2] * dir[2]);

		snVec dot2 = (structOfArray2[0] * dir[0]) +
			(structOfArray2[1] * dir[1]) +
			(structOfArray2[2] * dir[2]);

		//Dot1 and Dot2 contains the results of the dot products between the points and the direction.
		//Find the maximum
		snVec compare = snVec4GetMax(dot1, dot2);
		compare = snVec4GetMax(compare);
		_max = snVec4GetX(compare);

		//Find the minimum
		compare = snVec4GetMin(dot1, dot2);
		compare = snVec4GetMin(compare);
		_min = snVec4GetX(compare);
	}

	int snOBB::getUniqueNormals(snVec* _arrayNormals, int _arraySize) const
	{
		//not enough space to get the normals
		if (_arraySize < 3)
			return -1;

		for (int i = 0; i < 3; ++i)
			_arrayNormals[i] = m_normals[i];
		
		return 3;
	}

	snVec snOBB::support(const snVec& _direction) const
	{
		int dotX = sign(snVec4GetX(snVec3Dot(_direction, m_normals[0])));
		int dotY = sign(snVec4GetX(snVec3Dot(_direction, m_normals[1])));
		int dotZ = sign(snVec4GetX(snVec3Dot(_direction, m_normals[2])));

		return m_pos +
			m_normals[0] * (float)dotX * snVec4GetX(m_extends) +
			m_normals[1] * (float)dotY * snVec4GetY(m_extends) +
			m_normals[2] * (float)dotZ * snVec4GetZ(m_extends);
	}

	snVec snOBB::anyPoint() const
	{
		return m_worldBox[0];
	}

	snVec snOBB::getClosestPoint(const snVec& _v) const
	{
		snVec ret = snVec4Set(0, 0, 0, 0);

		//Vector from the center of the box to the point
		snVec dir = _v - m_pos;

		//Loop through each normals
		for (int i = 0; i < 3; ++i)
		{
			//Projection of the point to the normal
			snVec dot = snVec3Dot(dir, m_normals[i]);
			float extend = snVec4GetById(m_extends, i);
			dot = clampComponents(dot, -extend, extend);
			ret = ret + m_normals[i] * dot;
		}

		return ret + m_pos;
	}

	void snOBB::getClosestFeature(const snVec& _n, snVec* const _polygon, unsigned int& _count, unsigned int& _featureId) const
	{
		unsigned int fID = 0;
		{
			float maxDot = -SN_FLOAT_MAX;
			int normalId = -1;
			int side = 0;

			//loop through each obb normal to find the one the closest to the normal in argument
			for (int i = 0; i < 3; ++i)
			{
				float dot = snVec4GetX(snVec3Dot(_n, m_normals[i] * (1.f / snVec4GetById(m_extends, i))));

				if (dot > maxDot && dot > 0) //if _n is going to the same direction as the normal
				{
					maxDot = dot;
					normalId = i;
					side = 1;
				}
				else if (-dot > maxDot && -dot > 0) //_n is going to the same direction as -normal
				{
					maxDot = -dot;
					normalId = i;
					side = -1;
				}

			}

			if (normalId == 0 && side == 1)
				_featureId = 1;
			else if (normalId == 0 && side == -1)
				_featureId = 5;
			else if (normalId == 1 && side == 1)
				_featureId = 2;
			else if (normalId == 1 && side == -1)
				_featureId = 3;
			else if (normalId == 2 && side == 1)
				_featureId = 4;
			else
				_featureId = 0;

			fID = _featureId;
			unsigned int indexId = fID * 4;
			_polygon[0] = m_worldBox[m_idFaces[indexId]];
			_polygon[1] = m_worldBox[m_idFaces[indexId + 1]];
			_polygon[2] = m_worldBox[m_idFaces[indexId + 2]];
			_polygon[3] = m_worldBox[m_idFaces[indexId + 3]];
			_count = 4;
		}
	}

	void snOBB::getVertexFeatureIds(const snVec* _vertices, const snVec& _n, unsigned int* _ids) const
	{
		float maxDot = -SN_FLOAT_MAX;
		int normalId = -1;
		int side = 0;

		//loop through each obb normal to find the one the closest to the normal in argument
		for (int i = 0; i < 3; ++i)
		{
			float dot = snVec4GetX(snVec3Dot(_n, m_normals[i] * (1.f / snVec4GetById(m_extends, i))));

			if (dot > maxDot && dot > 0) //if _n is going to the same direction as the normal
			{
				maxDot = dot;
				normalId = i;
				side = 1;
			}
			else if (-dot > maxDot && -dot > 0) //_n is going to the same direction as -normal
			{
				maxDot = -dot;
				normalId = i;
				side = -1;
			}

		}

		//Get the face id
		unsigned int faceId = 0;
		if (normalId == 0 && side == 1)
			faceId = 1;
		else if (normalId == 0 && side == -1)
			faceId = 5;
		else if (normalId == 1 && side == 1)
			faceId = 2;
		else if (normalId == 1 && side == -1)
			faceId = 3;
		else if (normalId == 2 && side == 1)
			faceId = 4;
		else
			faceId = 0;

		//Get the vertices ids
		unsigned int indexId = faceId * 4;
		unsigned int verticesId[4];
		verticesId[0] = m_idFaces[indexId];
		verticesId[1] = m_idFaces[indexId + 1];
		verticesId[2] = m_idFaces[indexId + 2];
		verticesId[3] = m_idFaces[indexId + 3];

		list<unsigned int> inputListIds;
		for(unsigned int i = 0; i < 4; ++i)
			inputListIds.push_back(i);

		//map each vertex in parameter to an id
		for(unsigned int i : verticesId)
		{
			list<unsigned int>::iterator closest;
			float minDistance = SN_FLOAT_MAX;
			for(list<unsigned int>::iterator input = inputListIds.begin(); input != inputListIds.end(); ++input)
			{
				float sqDistance = snVec3SquaredNorme(m_worldBox[i] - _vertices[*input]);
				if(sqDistance < minDistance)
				{
					minDistance = sqDistance;
					closest = input;
				}
			}

			//save results
			_ids[*closest] = i + 3;
			inputListIds.erase(closest);
		}

	}

	snVec snOBB::getFeatureNormal(unsigned int _featureId) const
	{
		switch (_featureId)
		{
		case 0: //front
			return m_normals[2] * -1;
			break;

		case 1: //right
			return m_normals[0];
			break;

		case 2: //up
			return m_normals[1];
			break;

		case 3: //bottom
			return m_normals[1] * -1;
			break;

		case 4: //back
			return m_normals[2];
			break;

		case 5: //left
			return m_normals[0] * -1;
			break;

		default:
			assert(false);
			return snVec4Set(0, 0, 0, 0);
			break;

		}
	}

	void snOBB::computeAABB(snAABB * const _boundingVolume) const
	{
		_boundingVolume->m_max = m_worldBox[0];
		_boundingVolume->m_min = m_worldBox[0];

		const __m128 BITWISE_NEG_FLAG = _mm_castsi128_ps(_mm_set1_epi32(0xffffffff));
		for (int i = 1; i < 8; ++i)
		{
			//get the maximum
			__m128 compare = _mm_cmpgt_ps(m_worldBox[i], _boundingVolume->m_max);
			__m128 ncompare = _mm_xor_ps(compare, BITWISE_NEG_FLAG);

			__m128 mask1 = _mm_and_ps(m_worldBox[i], compare);
			__m128 mask2 = _mm_and_ps(_boundingVolume->m_max, ncompare);
			_boundingVolume->m_max = _mm_or_ps(mask1, mask2);

			//get the minimum
			compare = _mm_cmplt_ps(m_worldBox[i], _boundingVolume->m_min);
			ncompare = _mm_xor_ps(compare, BITWISE_NEG_FLAG);

			mask1 = _mm_and_ps(m_worldBox[i], compare);
			mask2 = _mm_and_ps(_boundingVolume->m_min, ncompare);
			_boundingVolume->m_min = _mm_or_ps(mask1, mask2);
		}
	}
}