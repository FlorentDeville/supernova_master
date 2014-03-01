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

#include "snColliderBox.h"

#include <assert.h>

#include "snCollisionResult.h"
#include "snMath.h"
#include "snCollision.h"

namespace Supernova
{
	snColliderBox::snColliderBox() : snICollider(), m_size(), m_box()
	{
	}

	snColliderBox::~snColliderBox()
	{}

	void snColliderBox::initialize()
	{
		computeVertices();
	}

	const snVector4f& snColliderBox::getSize() const
	{
		return m_size;
	}

	void snColliderBox::setSize(const snVector4f& _size)
	{
		m_size = _size;
	}

	void snColliderBox::computeVertices()
	{
		float halfX = m_size.VEC4FX * 0.5f;
		float halfY = m_size.VEC4FY * 0.5f;
		float halfZ = m_size.VEC4FZ * 0.5f;

		m_box[0] = m_origin + snVector4f(halfX, halfY, -halfZ, 0);
		m_box[1] = m_origin + snVector4f(halfX, -halfY, -halfZ, 0);
		m_box[2] = m_origin + snVector4f(-halfX, -halfY, -halfZ, 0);
		m_box[3] = m_origin + snVector4f(-halfX, halfY, -halfZ, 0);

		m_box[4] = m_origin + snVector4f(halfX, halfY, halfZ, 0);
		m_box[5] = m_origin + snVector4f(halfX, -halfY, halfZ, 0);
		m_box[6] = m_origin + snVector4f(-halfX, -halfY, halfZ, 0);
		m_box[7] = m_origin + snVector4f(-halfX, halfY, halfZ, 0);

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
		m_idFaces[13] = 5;
		m_idFaces[14] = 6;
		m_idFaces[15] = 2;

		//back
		m_idFaces[16] = 7;
		m_idFaces[17] = 6;
		m_idFaces[18] = 5;
		m_idFaces[19] = 4;

		//left
		m_idFaces[20] = 6;
		m_idFaces[21] = 2;
		m_idFaces[22] = 3;
		m_idFaces[23] = 7;

		//adjacent of the front face
		m_facesAdjacent[0][0] = 1;
		m_facesAdjacent[0][1] = 2;
		m_facesAdjacent[0][2] = 3;
		m_facesAdjacent[0][3] = 5;

		//adjacent of the right face
		m_facesAdjacent[1][0] = 0;
		m_facesAdjacent[1][1] = 2;
		m_facesAdjacent[1][2] = 4;
		m_facesAdjacent[1][3] = 3;

		//adjacent of the up face
		m_facesAdjacent[2][0] = 0;
		m_facesAdjacent[2][1] = 1;
		m_facesAdjacent[2][2] = 4;
		m_facesAdjacent[2][3] = 5;

		//adjacent of the bottom face
		m_facesAdjacent[3][0] = 0;
		m_facesAdjacent[3][1] = 1;
		m_facesAdjacent[3][2] = 4;
		m_facesAdjacent[3][3] = 5;

		//adjacent of the back face
		m_facesAdjacent[4][0] = 1;
		m_facesAdjacent[4][1] = 2;
		m_facesAdjacent[4][2] = 3;
		m_facesAdjacent[4][3] = 5;

		//adjacent of the left face
		m_facesAdjacent[5][0] = 0;
		m_facesAdjacent[5][1] = 2;
		m_facesAdjacent[5][2] = 4;
		m_facesAdjacent[5][3] = 3;

	}

	void snColliderBox::setWorldTransform(const snMatrix44f& _transform)
	{
		for (int i = 0; i < 8; ++i)
			m_worldBox[i] = _transform * m_box[i];

		m_worldOrigin = _transform * m_origin;

		m_worldNormals[0] = _transform * snVector4f(1, 0, 0, 0); //right - x
		m_worldNormals[1] = _transform * snVector4f(0, 1, 0, 0); //up - y
		m_worldNormals[2] = _transform * snVector4f(0, 0, 1, 0); //back - z
		m_worldNormals[0].normalize();
		m_worldNormals[1].normalize();
		m_worldNormals[2].normalize();
	}

	void snColliderBox::computeLocalInertiaTensor(float _mass, snMatrix44f& _inertiaTensor) const
	{
		snVector4f squaredSize = m_size * m_size;

		float massOverTwelve = _mass / 12.f;
		_inertiaTensor.m_r[0] = snVector4f(massOverTwelve * (squaredSize.VEC4FY + squaredSize.VEC4FZ), 0, 0, 0);
		_inertiaTensor.m_r[1] = snVector4f(0, massOverTwelve * (squaredSize.VEC4FX + squaredSize.VEC4FZ), 0, 0);
		_inertiaTensor.m_r[2] = snVector4f(0, 0, massOverTwelve * (squaredSize.VEC4FX + squaredSize.VEC4FY), 0);
		_inertiaTensor.m_r[3] = snVector4f(0, 0, 0, 1);
	}

	snCollisionResult snColliderBox::queryTestCollision(const snICollider& _collider) const
	{
		return _collider.queryTestCollision(*this);
	}

	snCollisionResult snColliderBox::queryTestCollision(const snColliderBox& _box) const
	{
		return snCollision::queryTestCollision(*this, _box);
	}

	snCollisionResult snColliderBox::queryTestCollision(const snColliderPlan& _plan) const
	{
		return snCollision::queryTestCollision(*this, _plan);
	}

	snCollisionResult snColliderBox::queryTestCollision(const snColliderSphere& _sphere) const
	{
		return snCollision::queryTestCollision(*this, _sphere);
	}

	snVector4f snColliderBox::getFarthestPointInDirection(const snVector4f& _direction) const
	{
		float maxDotProduct = -SN_FLOAT_MAX;
		int id = -1;

		//check every point
		for (int i = 0; i < VERTEX_COUNT; ++i)
		{
			float dot = _direction.dot(m_worldBox[i]);
			if (dot > maxDotProduct)
			{
				maxDotProduct = dot;
				id = i;
			}
		}

		assert(id != -1);
		return m_worldBox[id];
	}

	const snVector4f* snColliderBox::getWorldVertices() const
	{
		return m_worldBox;
	}

	const snVector4f* snColliderBox::getWorldNormal() const
	{
		return m_worldNormals;
	}

	snVector4f snColliderBox::getClosestPoint(const snVector4f& _v) const
	{
		snVector4f ret;

		//Vector from the center of the box to the point
		snVector4f dir = _v - m_worldOrigin;

		//Get box normals
		const snVector4f* normals;
		normals = getWorldNormal();

		//Loop through each normals
		for (int i = 0; i < 3; ++i)
		{
			//Projection of the point to the normal
			float dot = dir.dot(normals[i]);

			float halfExtend = m_size.m_vec.m128_f32[i] * 0.5f;
			if (dot > halfExtend) dot = halfExtend;
			else if (dot < -halfExtend) dot = -halfExtend;

			ret = ret + normals[i] * dot;
		}

		return ret + m_worldOrigin;
	}

	void snColliderBox::getClosestPolygonProjected(const snVector4f& _n, snVector4f* const _polygon, int& _faceId) const
	{
		//find the closest point projected onto the normal
		float min = SN_FLOAT_MAX;
		int minId = -1;
		for (int i = 0; i < VERTEX_COUNT; ++i)
		{
			float d = _n.dot(m_worldBox[i]);
			if (d < min)
			{
				min = d;
				minId = i;
			}
		}

		//loop through each polygon
		float minPoly = SN_FLOAT_MAX;
		int minPolyId = 0;
		for (int polyId = 0; polyId < INDEX_COUNT; polyId += 4)
		{
			//check if the polygon contains the closest vertex
			if (m_idFaces[polyId] != minId &&
				m_idFaces[polyId + 1] != minId &&
				m_idFaces[polyId + 2] != minId &&
				m_idFaces[polyId + 3] != minId)
				continue;

			//make the sum of dot product
			float sumOfDot = _n.dot(m_worldBox[m_idFaces[polyId]]) + _n.dot(m_worldBox[m_idFaces[polyId + 1]]) +
				_n.dot(m_worldBox[m_idFaces[polyId + 2]]) + _n.dot(m_worldBox[m_idFaces[polyId + 3]]);

			if (sumOfDot < minPoly)
			{
				minPoly = sumOfDot;
				minPolyId = polyId;
			}
		}

		_faceId = int(minPolyId * 0.25f);
		_polygon[0] = m_worldBox[m_idFaces[minPolyId]];
		_polygon[1] = m_worldBox[m_idFaces[minPolyId + 1]];
		_polygon[2] = m_worldBox[m_idFaces[minPolyId + 2]];
		_polygon[3] = m_worldBox[m_idFaces[minPolyId + 3]];
	}

	snVector4f snColliderBox::getWorldNormalOfFace(int _faceId) const
	{
		switch (_faceId)
		{
		case 0: //front
			return m_worldNormals[2] * -1;
			break;

		case 1: //right
			return m_worldNormals[0];
			break;

		case 2: //up
			return m_worldNormals[1];
			break;

		case 3: //bottom
			return m_worldNormals[1] * -1;
			break;

		case 4: //back
			return m_worldNormals[2];
			break;

		case 5: //left
			return m_worldNormals[0] * -1;
			break;

		default:
			assert(false);
			return snVector4f();
			break;

		}
	}

	const int* snColliderBox::getAdjacentFaces(int _faceId) const
	{
		return m_facesAdjacent[_faceId];
	}

	snVector4f snColliderBox::getWorldVertexOfFace(int _faceId) const
	{
		return m_worldBox[m_idFaces[_faceId * 4]];
	}
}