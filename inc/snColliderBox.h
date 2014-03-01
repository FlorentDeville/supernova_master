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

#ifndef SN_COLLIDER_BOX_H
#define SN_COLLIDER_BOX_H

#include "snICollider.h"

namespace Supernova
{
	class snColliderBox : public snICollider
	{
	private:
		static const int VERTEX_COUNT = 8;
		static const int INDEX_COUNT = 24;
		static const int FACE_COUNT = 6;

		//size of the box: width, height, depth.
		snVector4f m_size;

		//Vertices composing the box.
		snVector4f m_box[VERTEX_COUNT];

		/*List of indices of vertices making each face of the box. Widing is anticlockwise.*/
		int m_idFaces[INDEX_COUNT];

		//Store for each face the id of adjacent faces.
		int m_facesAdjacent[FACE_COUNT][4];

		//Vertices composing the box in world coordinates.
		snVector4f m_worldBox[VERTEX_COUNT];

		//Normals expressed in wolrd coordinates.
		snVector4f m_worldNormals[3];

	public:
		//Constructor. You should not create collider yourself. Use snActor::createColliderBox instead. If you create a collider yourself
		// it is your responsability to delete it.
		snColliderBox();

		//Destructor.
		~snColliderBox();

		//Initialize the collider. Should be called once all the parameters of the collider are set.
		void initialize();

		void setWorldTransform(const snMatrix44f& _transform);

		//Compute the inertia tensor in a local frame.
		void computeLocalInertiaTensor(float _mass, snMatrix44f& _inertiaTensor) const;

		//Check collision between a collider given in parameter and this collider (in that order).
		snCollisionResult queryTestCollision(const snICollider& _collider) const;

		//Check collision between this collider and a box collider.
		snCollisionResult queryTestCollision(const snColliderBox& _box) const;

		//Check collision between this collider and a plan collider.
		snCollisionResult queryTestCollision(const snColliderPlan& _plan) const;

		//Check collision between this collider and a sphere collider.
		snCollisionResult queryTestCollision(const snColliderSphere& _sphere) const;

		//Return the farthest point in the direction provided by the _direction vector. It does not need to be normalized.
		snVector4f getFarthestPointInDirection(const snVector4f& _direction) const;

		const snVector4f& getSize() const;

		void setSize(const snVector4f& _size);

		const snVector4f* getWorldVertices() const;

		/*Return an array of three vectors containing normals along x, y and z axis (in that order) and in world coordinates.*/
		const snVector4f* getWorldNormal() const;

		/*Calculate the closest point to the box of the point given in parameter.*/
		snVector4f getClosestPoint(const snVector4f&) const;

		//get the closest polygon projected onto the normal.
		void getClosestPolygonProjected(const snVector4f& _n, snVector4f* const _polygon, int& _count) const;

		//return the normal in workd coordinate of the face
		snVector4f getWorldNormalOfFace(int _faceId) const;

		const int* getAdjacentFaces(int _faceId) const;

		snVector4f getWorldVertexOfFace(int _faceId) const;

	private:
		void computeVertices();

		
	};
}

#endif // SN_COLLIDER_BOX_H