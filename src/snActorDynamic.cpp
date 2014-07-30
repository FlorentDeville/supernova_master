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

#include "snActorDynamic.h"
#include "snICollider.h"
#include "snQuaternion.h"
#include "snColliderContainer.h"

#include <assert.h>

using namespace Supernova::Vector;

namespace Supernova
{
	snActorDynamic::snActorDynamic() : m_linearDamping(0.01f), m_angularDamping(0.f)
	{
		m_name = "default";
		m_centerOfMass = snVec4Set(0, 0, 0, 1);
		m_worldCenterOfMass = snVec4Set(0, 0, 0, 1);

		m_transform.setPosition(snVec4Set(0, 0, 0, 1));
		m_transform.setOrientation(snVec4Set(0, 0, 0, 1));

		m_skinDepth = 0.025f;

		m_w = snVec4Set(0, 0, 0, 0);
		m_v = snVec4Set(0, 0, 0, 0);
		m_isKinematic = false;
		m_typeOfActor = snActorType::snActorTypeDynamic;

		m_collisionFlag = snCollisionFlag::CF_NO_FLAG;
		m_collisionCallback = 0;
		m_isActive = true;
	}

	snActorDynamic::~snActorDynamic()
	{}

	float snActorDynamic::getMass() const
	{
		return m_mass;
	}

	//Return the inverse of the mass
	float snActorDynamic::getInvMass() const
	{
		return m_invMass;
	}

	//Return the inverse of the inertia expressed in world coordinate
	const snMatrix44f& snActorDynamic::getInvWorldInertia() const
	{
		return m_invWorldInertia;
	}

	//Return the linear velocity
	snVec snActorDynamic::getLinearVelocity() const
	{
		return m_v;
	}

	//Return the angular velocity
	snVec snActorDynamic::getAngularVelocity() const
	{
		return m_w;
	}

	//Return the linear damping coefficient
	float snActorDynamic::getLinearDampingCoeff() const
	{
		return m_linearDamping;
	}

	//Return the angular damping coefficient
	float snActorDynamic::getAngularDampingCoeff() const
	{
		return m_angularDamping;
	}

	//Set the linear velocity
	void snActorDynamic::setLinearVelocity(const snVec& _linearVelocity)
	{
		m_v = _linearVelocity;
	}

	//Set the angular velocity
	void snActorDynamic::setAngularVelocity(const snVec& _angularVelocity)
	{
		m_w = _angularVelocity;
	}

	//Set the linear damping coefficient
	void snActorDynamic::setLinearDampingCoeff(float _linearDamping)
	{
		m_linearDamping = _linearDamping;
	}

	//Set the angular damping coefficient
	void snActorDynamic::setAngularDampingCoeff(float _angularDamping)
	{
		m_angularDamping = _angularDamping;
	}

	//Set the position of the actor
	void snActorDynamic::setPosition(const snVec& _position)
	{
		m_transform.setPosition(_position);
	}

	//Set the orientation of the actor
	void snActorDynamic::setOrientation(const snVec& _orientation)
	{
		m_transform.setOrientation(_orientation);
	}

	//Set if the actor is kinematic
	void snActorDynamic::setIsKinematic(bool _isKinematic)
	{
		m_isKinematic = _isKinematic;
		if (m_isKinematic)
		{
			m_mass = 0;
			m_invMass = 0;
			m_invInertia = snMatrix44f::m_zero;
			m_typeOfActor = snActorType::snActorTypeKinematic;
		}
		else
			m_typeOfActor = snActorType::snActorTypeDynamic;
	}

	void snActorDynamic::setKinematicPosition(const snVec& _position)
	{
		assert(m_isKinematic);
		setPosition(_position);

		//compute colliders in world coordinate
		updateCollidersAndAABB();
	}

	void snActorDynamic::setKinematicTransform(const snVec& _position, const snVec& _orientation)
	{
		assert(m_isKinematic);

		setPosition(_position);
		setOrientation(_orientation);

		updateCollidersAndAABB();
	}

	//Set the mass to the actor and update its inertia
	void snActorDynamic::updateMassAndInertia(float _mass)
	{
		//if the actor is kinematic, there is no need to update the mass and inertia.
		assert(!m_isKinematic);

		m_mass = _mass;
		m_invMass = 1.f / _mass;

		//compute inertia of the collider
		snMatrix44f globalIntertia;
		for (vector<snColliderContainer*>::const_iterator i = m_colliders.cbegin(); i != m_colliders.cend(); ++i)
		{
			snMatrix44f inertia;
			(*i)->m_collider->computeLocalInertiaTensor(m_mass, inertia);

			//add the inertia of the collider to the global collider
			globalIntertia = globalIntertia + inertia;
		}

		//compute the inverse inertia tensor
		m_invInertia = globalIntertia.inverse();

		computeInvWorldInertia();
	}

	//Initialize the actor so it is ready to be used in the scene. It has to be called and must be called after all the parameters of
	// the actor and its colliders are set.
	void snActorDynamic::initialize()
	{
		//initialize colliders
		for (vector<snColliderContainer*>::iterator i = m_colliders.begin(); i != m_colliders.end(); ++i)
		{
			(*i)->m_collider->initialize();
			m_centerOfMass = m_centerOfMass + (*i)->m_localTransform.getPosition();//snMatrixGetTranslation((*i)->m_localTransform);
		}

		snVec4SetW(m_centerOfMass, 1);

		computeWorldCenterOfMass();

		//compute colliders in world coordinate
		updateCollidersAndAABB();
	}

	//Compute the angular speed of the actor
	float snActorDynamic::computeAngularSpeed() const
	{
		return snVec3Norme(m_w);
	}

	//Compute the linear speed of the actor
	float snActorDynamic::computeLinearSpeed() const
	{
		return snVec3Norme(m_v);
	}

	//Compute the inverse of the inertia tensor expressed in world coordinates
	void snActorDynamic::computeInvWorldInertia()
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

	void snActorDynamic::computeWorldCenterOfMass()
	{
		m_worldCenterOfMass = snMatrixTransform4(m_centerOfMass, m_transform.getLocalToWorld());
	}

	//Update the colliders based on the current position and orientation
	void snActorDynamic::updateCollidersAndAABB()
	{
		for (vector<snColliderContainer*>::iterator i = m_colliders.begin(); i != m_colliders.end(); ++i)
		{
			snTransform result;
			snTransformMultiply((*i)->m_localTransform, m_transform, result);
			(*i)->m_collider->setTransform(result);
		}

		computeBoundingVolume();
	}

	//Move the actor forward in time using _dt as a time step.
	//_linearSpeed2Limit and _angularSpeed2Limit are the squared speed below which the velocities will be set to 0.
	void snActorDynamic::integrate(float _dt, float _linearSpeed2Limit, float _angularSpeed2Limit)
	{
		//apply damping
		m_v = m_v * (1 - m_linearDamping * _dt);
		m_w = m_w * (1 - m_angularDamping * _dt);
		
		//if the linear speed is too small, set it to 0.
		float sqSpeed = snVec3SquaredNorme(m_v);
		if (sqSpeed < _linearSpeed2Limit)
			m_v = snVec4Set(0);

		//if the angular speed is too small, set it to 0.
		sqSpeed = snVec3SquaredNorme(m_w);
		if (sqSpeed < _angularSpeed2Limit)
			m_w = snVec4Set(0);

		//calculate position using euler integration
		snVec previousPosition = m_transform.getPosition();
		m_transform.setPosition(previousPosition + m_v * _dt);

		//calculate velocity as quaternion using dq/dt = 0.5 * w * q
		snVec q = m_transform.getOrientation();
		snVec qw = snQuaternionMultiply(m_w, q);
		qw = qw * 0.5f;

		//calculate orientation using euler integration
		q = q + (qw * _dt);
		snQuaternionNormalize(q, q);
		m_transform.setOrientation(q);

		//set new state
		computeInvWorldInertia();
		computeWorldCenterOfMass();

		//compute colliders in world coordinate
		updateCollidersAndAABB();
	}
}