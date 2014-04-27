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

#include "snIActor.h"
#include "snICollider.h"

#include <assert.h>

namespace Supernova
{

	snIActor:: ~snIActor()
	{
		for (vector<snICollider*>::iterator i = m_colliders.begin(); i != m_colliders.end(); ++i)
			delete *i;

		m_colliders.clear();
	}

	//Add a collider to the actor
	void snIActor::addCollider(snICollider* _collider)
	{
		m_colliders.push_back(_collider);
	}

	//Get the name of the actor
	string snIActor::getName() const
	{
		return m_name;
	}

	//return the list of colliders
	vector<snICollider*>& snIActor::getColliders()
	{
		return m_colliders;
	}

	//Return the position of the actor.
	snVector4f snIActor::getPosition() const
	{
		return m_x;
	}
	//Return the orientation represented as a quaternion
	snVector4f snIActor::getOrientationQuaternion()
	{
		return m_q;
	}


	//Return the orientation represented as a matrix.
	const snMatrix44f& snIActor::getOrientationMatrix() const
	{
		return m_R;
	}

	//Return the inverse of orientation matrix.
	const snMatrix44f& snIActor::getInverseOrientationMatrix() const
	{
		return m_invR;
	}

	//Get the maximum depth another actor can penetrate into this actor
	float snIActor::getSkinDepth() const
	{
		return m_skinDepth;
	}

	//Return a pointer to the AABB.
	const snAABB* snIActor::getBoundingVolume() const
	{
		return &m_boundingVolume;
	}

	//Return the physic material.
	snPhysicMaterial& snIActor::getPhysicMaterial()
	{
		return m_material;
	}

	//Return the type of actor
	snActorType snIActor::getActorType() const
	{
		return m_typeOfActor;
	}

	//Set the name of the actor
	void snIActor::setName(const string& _name)
	{
		m_name = _name;
	}

	//Set the maximum depth another actor can penetrate into this actor
	void snIActor::setSkinDepth(float _skinDepth)
	{
		m_skinDepth = _skinDepth;
	}

	void snIActor::setOnCollisionCallback(OnCollisionCallback _callback)
	{
		m_collisionCallback = _callback;
	}

	void snIActor::OnCollision(snIActor* const _other)
	{
		assert(m_collisionCallback != 0);

		m_collisionCallback(this, _other);
	}

	//Allocate an actor with the correct alignement
	void* snIActor::operator new(size_t _count)
	{
		return _aligned_malloc(_count, SN_ALIGN_SIZE);
	}

	//Free the memory allocated for an actor
	void snIActor::operator delete(void* _p)
	{
		_aligned_free(_p);
	}

	//Add a collision flag to the actor
	void snIActor::addCollisionFlag(snCollisionFlag _flag)
	{
		m_collisionFlag = m_collisionFlag | _flag;
	}

	//Remove the collision flag of the actor
	void snIActor::removeCollisionFlag(snCollisionFlag _flag)
	{
		m_collisionFlag = m_collisionFlag & ~_flag;
	}

	//Set the collision flag
	void snIActor::setCollisionFlag(snCollisionFlag _flag)
	{
		m_collisionFlag = _flag;
	}

	//Return the collision flag
	unsigned char snIActor::getCollisionFlag()
	{
		return m_collisionFlag;
	}

	//Check if a collision flag is enabled.
	bool snIActor::isEnabledCollisionFlag(snCollisionFlag _flag)
	{
		return (m_collisionFlag & _flag) != 0;
	}

	//Compute the bounding volume based on the colliders
	void snIActor::computeBoundingVolume()
	{
		assert(m_colliders.size() == 1);

		m_colliders[0]->computeAABB(&m_boundingVolume);
	}
}