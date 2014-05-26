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

#ifndef SN_ICOLLIDER_H
#define SN_ICOLLIDER_H

#include "snMatrix44f.h"

namespace Supernova
{
	//forward declarations of derived types.
	class snCollisionResult;
	class snColliderBox;
	class snColliderSphere;
	class snColliderPlan;
	struct snAABB;

	//Type enum to identify the kind of collider
	enum snEColliderType : unsigned char
	{
		snEColliderUnknown,
		snEColliderBox,
		snEColliderPlan,
		snEColliderSphere
	};

	class snICollider
	{

	protected:

		/*Origin of the shape in world coordinate.*/
		snVec m_worldOrigin;

		//The type of collider
		snEColliderType m_typeOfCollider;

	public:
		snICollider(){}

		virtual ~snICollider(){}

		//Initialize the collider. Should be called once all the parameters of the collider are set.
		virtual void initialize() = 0;

		//Move the collider using the given transform matrix.
		virtual void setWorldTransform(const snMatrix44f& _transform) = 0;

		//Compute the inertia tensor in a local frame.
		virtual void computeLocalInertiaTensor(float _mass, snMatrix44f& _inertiaTensor) const = 0;

		//Return the farthest point in the direction provided by the _direction vector. It does not need to be normalized.
		virtual snVec getFarthestPointInDirection(const snVec& /*_direction*/) const { return Supernova::Vector::snVec4Set(0, 0, 0, 1); };

		//Project the collider along an axis and return the min and max value. The direction has to be expressed in world coordinate.
		virtual void computeProjection(const snVec& _direction, float& _min, float& _max) const = 0;

		//Get the closest polygon projected onto the normal.
		virtual void getClosestPolygonProjected(const snVec& /*_n*/, snVec* const /*_polygon*/, int& /*_count*/) const{};

		//Get the ids of all the adjacent faces.
		virtual const int* getAdjacentFaces(int /*_faceId*/) const { return 0; };

		//Return the normal in world coordinate of the face
		virtual snVec getWorldNormalOfFace(int /*_faceId*/) const { return Supernova::Vector::snVec4Set(0, 0, 0, 1); };

		virtual snVec getWorldVertexOfFace(int /*_faceId*/) const { return Supernova::Vector::snVec4Set(0, 0, 0, 1); };

		snEColliderType getTypeOfCollider() const { return m_typeOfCollider; }

		const snVec& getWorldOrigin() const { return m_worldOrigin; }

		//Compute the bounding volume for this collider
		virtual void computeAABB(snAABB * const /*_boundingVolume*/) const {};

		void* operator new(size_t _count)
		{
			return _aligned_malloc(_count, SN_ALIGN_SIZE);
		}

		void operator delete(void* _p)
		{
			_aligned_free(_p);
		}
	};
}

#endif // SN_ICOLLIDER_H