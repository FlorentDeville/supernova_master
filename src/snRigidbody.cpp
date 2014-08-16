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

#include "snRigidbody.h"
#include "snICollider.h"

#include <assert.h>

using namespace Supernova::Vector;

namespace Supernova
{
	snRigidbody::snRigidbody() : snObject(), m_linearDamping(0.01f), m_angularDamping(0.f)
	{
		m_name = "default";
		m_centerOfMass = snVec4Set(0, 0, 0, 1);
		m_worldCenterOfMass = snVec4Set(0, 0, 0, 1);

		m_transform.setLocalPosition(snVec4Set(0, 0, 0, 1));
		m_transform.setLocalOrientation(snVec4Set(0, 0, 0, 1));

		m_skinDepth = 0.025f;

		m_w = snVec4Set(0, 0, 0, 0);
		m_v = snVec4Set(0, 0, 0, 0);
		m_isKinematic = false;

		m_collisionFlag = snCollisionFlag::CF_NO_FLAG;
		m_collisionCallback = 0;
		m_isActive = true;
	}

	snRigidbody::~snRigidbody()
	{
		for (vector<snICollider*>::iterator i = m_colliders.begin(); i != m_colliders.end(); ++i)
			delete *i;

		m_colliders.clear();
	}

	//Add a collider to the actor
	void snRigidbody::addCollider(snICollider* _collider)
	{
		m_colliders.push_back(_collider);
		snTransformAddLink(&m_transform, &_collider->getTransform());
	}

	bool snRigidbody::isStatic() const
	{
		return !m_isKinematic && m_mass == 0;
	}

	bool snRigidbody::isKinematic() const
	{
		return m_isKinematic;
	}

	bool snRigidbody::isDynamic() const
	{
		return !m_isKinematic && m_mass != 0;
	}


#pragma region Getter

	//Get the name of the actor
	string snRigidbody::getName() const
	{
		return m_name;
	}

	//Return if the actor is active or not.
	bool snRigidbody::getIsActive() const
	{
		return m_isActive;
	}

	//return the list of colliders
	vector<snICollider*>& snRigidbody::getColliders()
	{
		return m_colliders;
	}

	//Return the mass. It returns 0 in case of a static or kinematic body
	float snRigidbody::getMass() const
	{
		return m_mass;
	}

	//Return the inverse of the mass
	float snRigidbody::getInvMass() const
	{
		return m_invMass;
	}

	//Return the inverse of the inertia expressed in world coordinate
	const snMatrix44f& snRigidbody::getInvWorldInertia() const
	{
		return m_invWorldInertia;
	}

	//Return the center of mass expressed in local coordinate system.
	snVec snRigidbody::getCenterOfMass() const
	{
		return m_centerOfMass;
	}

	//Return the center of mass expressed in world coordinate system.
	snVec snRigidbody::getWorldCenterOfMass() const
	{
		return m_worldCenterOfMass;
	}

	//Return the position of the actor.
	snVec snRigidbody::getPosition() const
	{
		return m_transform.getPosition();
	}

	//Return the transform of the current actor.
	const snTransform& snRigidbody::getTransform() const
	{
		return m_transform;
	}

	snTransform& snRigidbody::getTransform()
	{
		return m_transform;
	}

	//Return the linear velocity
	snVec snRigidbody::getLinearVelocity() const
	{
		return m_v;
	}

	//Return the angular velocity
	snVec snRigidbody::getAngularVelocity() const
	{
		return m_w;
	}

	//Return the orientation represented as a quaternion
	snVec snRigidbody::getOrientationQuaternion()
	{
		return m_transform.getOrientation();
	}

	//Get the maximum depth another actor can penetrate into this actor
	float snRigidbody::getSkinDepth() const
	{
		return m_skinDepth;
	}

	//Return a pointer to the AABB.
	const snAABB* snRigidbody::getBoundingVolume() const
	{
		return &m_boundingVolume;
	}

	//Return the physic material.
	snPhysicMaterial& snRigidbody::getPhysicMaterial()
	{
		return m_material;
	}

	//Return the linear damping.
	float snRigidbody::getLinearDampingCoeff() const
	{
		return m_linearDamping;
	}

	//Return the angular damping
	float snRigidbody::getAngularDampingCoeff() const
	{
		return m_angularDamping;
	}

#pragma endregion

#pragma region Setter

	//Set the name of the actor
	void snRigidbody::setName(const string& _name)
	{
		m_name = _name;
	}

	//Set if the actor is active or not.
	void snRigidbody::setIsActive(bool _isActive)
	{
		m_isActive = _isActive;
	}

	//Set the maximum depth another actor can penetrate into this actor
	void snRigidbody::setSkinDepth(float _skinDepth)
	{
		m_skinDepth = _skinDepth;
	}

	//Set the linear velocity
	void snRigidbody::setLinearVelocity(const snVec& _linearVelocity)
	{
		m_v = _linearVelocity;
	}

	//Set the angular velocity
	void snRigidbody::setAngularVelocity(const snVec& _angularVelocity)
	{
		m_w = _angularVelocity;
	}

	//Set the position of the actor
	void snRigidbody::setPosition(const snVec& _position)
	{
		m_transform.setLocalPosition(_position);
	}

	//Set the orientation of the actor
	void snRigidbody::setOrientation(const snVec& _orientation)
	{
		m_transform.setLocalOrientation(_orientation);
	}

	//Set the collision callback
	void snRigidbody::setOnCollisionCallback(OnCollisionCallback _callback)
	{
		m_collisionCallback = _callback;
	}

	//Set if the actor is kinematic
	void snRigidbody::setIsKinematic(bool _isKinematic)
	{
		m_isKinematic = _isKinematic;
		if (m_isKinematic)
		{
			m_mass = 0;
			m_invMass = 0;
			m_invInertia = snMatrix44f::m_zero;
		}
	}

	//Set the linear damping coefficient
	void snRigidbody::setLinearDampingCoeff(float _linearDamping)
	{
		m_linearDamping = _linearDamping;
	}

	//Set the angular damping coefficient
	void snRigidbody::setAngularDampingCoeff(float _angularDamping)
	{
		m_angularDamping = _angularDamping;
	}

	void snRigidbody::setKinematicPosition(const snVec& _position)
	{
		assert(m_isKinematic);
		setPosition(_position);

		//compute colliders in world coordinate
		updateCollidersAndAABB();
	}

	void snRigidbody::setKinematicTransform(const snVec& _position, const snVec& _orientation)
	{
		assert(m_isKinematic);

		setPosition(_position);
		setOrientation(_orientation);

		updateCollidersAndAABB();
	}

#pragma endregion

#pragma region Allocation

	//Allocate an actor with the correct alignement
	void* snRigidbody::operator new(size_t _count)
	{
		return _aligned_malloc(_count, SN_ALIGN_SIZE);
	}

	//Free the memory allocated for an actor
	void snRigidbody::operator delete(void* _p)
	{
		_aligned_free(_p);
	}

#pragma endregion

#pragma region Collision Flags

	//Add a collision flag to the actor
	void snRigidbody::addCollisionFlag(snCollisionFlag _flag)
	{
		m_collisionFlag = m_collisionFlag | _flag;
	}

	//Remove the collision flag of the actor
	void snRigidbody::removeCollisionFlag(snCollisionFlag _flag)
	{
		m_collisionFlag = m_collisionFlag & ~_flag;
	}

	//Set the collision flag
	void snRigidbody::setCollisionFlag(snCollisionFlag _flag)
	{
		m_collisionFlag = _flag;
	}

	//Return the collision flag
	unsigned char snRigidbody::getCollisionFlag()
	{
		return m_collisionFlag;
	}

	//Check if a collision flag is enabled.
	bool snRigidbody::isEnabledCollisionFlag(snCollisionFlag _flag)
	{
		return (m_collisionFlag & _flag) != 0;
	}

#pragma endregion

	//Initialize the actor so it is ready to be used in the scene. It has to be called and must be called after all the parameters of
	// the actor and its colliders are set.
	void snRigidbody::initialize()
	{
		//initialize colliders
		for (vector<snICollider*>::iterator i = m_colliders.begin(); i != m_colliders.end(); ++i)
		{
			(*i)->initialize();
			(*i)->updateFromTransform();
			m_centerOfMass = m_centerOfMass + (*i)->getTransform().getLocalPosition();
		}

		snVec4SetW(m_centerOfMass, 1);

		computeWorldCenterOfMass();

		//compute colliders in world coordinate
		updateCollidersAndAABB();
	}

	void snRigidbody::initializeStatic(const snVec& _p, const snVec& _q)
	{
		m_transform.setLocalPosition(_p);
		m_transform.setLocalOrientation(_q);

		m_skinDepth = 0.025f;
		m_mass = 0;
		m_invMass = 0;
		m_invInertia = snMatrix44f::m_zero;

		m_isActive = true;
	}

	void snRigidbody::onCollision(snRigidbody* const _other)
	{
		if (m_collisionCallback!= 0)
			m_collisionCallback(this, _other);
	}

	//Check if we can make a collision detection test based on the type of actors
	bool snRigidbody::isCollisionDetectionEnabled(const snRigidbody* const _a, const snRigidbody* const _b)
	{
		//No colision detection between two statics or a kinematic and a static.
		if((_a->isStatic() && _b->isStatic())
			|| (_a->isStatic() && _b->isKinematic())
			|| (_a->isKinematic() && _b->isStatic()))
			return false;

		return true;
	}

	//Compute the angular speed of the actor
	float snRigidbody::computeAngularSpeed() const
	{
		return snVec3Norme(m_w);
	}

	//Compute the linear speed of the actor
	float snRigidbody::computeLinearSpeed() const
	{
		return snVec3Norme(m_v);
	}

	//Compute the bounding volume based on the colliders
	void snRigidbody::computeBoundingVolume()
	{
		vector<snICollider*>::const_iterator collider = m_colliders.cbegin();
		(*collider)->computeAABB(&m_boundingVolume);
		++collider;

		//loop through all the colliders and make the AABB for the entire actor
		for(;collider != m_colliders.cend(); ++collider)
		{
			snAABB aabb;
			(*collider)->computeAABB(&aabb);
			mergeAABB(m_boundingVolume, aabb, m_boundingVolume);
		}
	}

	//Compute the inverse of the inertia tensor expressed in world coordinates
	void snRigidbody::computeInvWorldInertia()
	{
		//The inverse world inertia tensor is R * I-1 * RT with R being the orientation matrix this is why
		// the last row is set to (0, 0, 0, 1).
		snMatrix44f localToWorld = m_transform.getLocalToWorld();
		localToWorld.m_r[3] = snVec4Set(0, 0, 0, 1);

		snMatrix44f localToWorldTranspose;
		localToWorld.transpose(localToWorldTranspose);

		snMatrix44f WInvJ;
		snMatrixMultiply3(localToWorld, m_invInertia, WInvJ);
		snMatrixMultiply3(WInvJ, localToWorldTranspose, m_invWorldInertia);
	}

	void snRigidbody::computeWorldCenterOfMass()
	{
		m_worldCenterOfMass = snMatrixTransform4(m_centerOfMass, m_transform.getLocalToWorld());
	}

	//Update the colliders based on the current position and orientation
	void snRigidbody::updateCollidersAndAABB()
	{
		for (vector<snICollider*>::iterator i = m_colliders.begin(); i != m_colliders.end(); ++i)
		{
			(*i)->updateFromTransform();
		}

		computeBoundingVolume();
	}

	//Set the mass to the actor and update its inertia
	void snRigidbody::updateMassAndInertia(float _mass)
	{
		//if the actor is kinematic, there is no need to update the mass and inertia.
		assert(!m_isKinematic);

		m_mass = _mass;
		m_invMass = 1.f / _mass;

		//compute inertia of the collider
		snMatrix44f globalIntertia;
		for (vector<snICollider*>::const_iterator i = m_colliders.cbegin(); i != m_colliders.cend(); ++i)
		{
			snMatrix44f inertia;
			(*i)->computeLocalInertiaTensor(m_mass, inertia);

			//add the inertia of the collider to the global collider
			globalIntertia = globalIntertia + inertia;
		}

		//compute the inverse inertia tensor
		m_invInertia = globalIntertia.inverse();

		computeInvWorldInertia();
	}
}