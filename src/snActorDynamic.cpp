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

#include <assert.h>

namespace Supernova
{
	snActorDynamic::snActorDynamic() : m_linearDamping(0.01f), m_angularDamping(0.f)
	{
		m_name = "default";
		m_x = snVector4f();
		m_q = snVector4f(0, 0, 0, 1);
		m_skinDepth = 0.025f;
		m_R.identity();
		m_invR.identity();

		m_w = snVector4f(0, 0, 0, 0);
		m_isKinematic = false;
		m_typeOfActor = snActorType::snActorTypeDynamic;

		m_collisionFlag = snCollisionFlag::CF_NO_FLAG;
		m_collisionCallback = 0;
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
	snVector4f snActorDynamic::getLinearVelocity() const
	{
		return m_v;
	}

	//Return the angular velocity
	snVector4f snActorDynamic::getAngularVelocity() const
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
	void snActorDynamic::setLinearVelocity(const snVector4f& _linearVelocity)
	{
		m_v = _linearVelocity;
	}

	//Set the angular velocity
	void snActorDynamic::setAngularVelocity(const snVector4f& _angularVelocity)
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
	void snActorDynamic::setPosition(const snVector4f& _position)
	{
		m_x = _position;
	}

	//Set the orientation of the actor
	void snActorDynamic::setOrientation(const snVector4f& _orientation)
	{
		m_q = _orientation;
		m_R.createRotationFromQuaternion(m_q);
		m_invR = m_R.inverse();
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

	void snActorDynamic::setKinematicPosition(const snVector4f& _position)
	{
		assert(m_isKinematic);
		setPosition(_position);

		//compute colliders in world coordinate
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

	//Initialize the actor so it is ready to be used in the scene. It has to be called and must be called after all the parameters of
	// the actor and its colliders are set.
	void snActorDynamic::initialize()
	{
		//initialize colliders
		for (vector<snICollider*>::iterator i = m_colliders.begin(); i != m_colliders.end(); ++i)
		{
			(*i)->initialize();
		}

		//compute colliders in world coordinate
		updateCollidersAndAABB();
	}

	//Compute the angular speed of the actor
	float snActorDynamic::computeAngularSpeed() const
	{
		return m_w.norme();
	}

	//Compute the linear speed of the actor
	float snActorDynamic::computeLinearSpeed() const
	{
		return m_v.norme();
	}

	//Compute the inverse of the inertia tensor expressed in world coordinates
	void snActorDynamic::computeInvWorldInertia()
	{
		snMatrix44f RT;
		m_R.transpose(RT);

		snMatrix44f WInvJ;
		snMatrixMultiply3(m_R, m_invInertia, WInvJ);
		snMatrixMultiply3(WInvJ, RT, m_invWorldInertia);
	}

	//Update the colliders based on the current position and orientation
	void snActorDynamic::updateCollidersAndAABB()
	{
		snMatrix44f translation;
		translation.createTranslation(m_x);

		snMatrix44f transform;
		snMatrixMultiply4(m_R, translation, transform);

		for (vector<snICollider*>::iterator i = m_colliders.begin(); i != m_colliders.end(); ++i)
			(*i)->setWorldTransform(transform);

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
		float sqSpeed = m_v.squareNorme();
		if (sqSpeed < _linearSpeed2Limit)
			m_v = snVector4f();

		//if the angular speed is too small, set it to 0.
		sqSpeed = m_w.squareNorme();
		if (sqSpeed < _angularSpeed2Limit)
			m_w = snVector4f();

		//calculate position using euler integration
		m_x = m_x + m_v * _dt;

		//calculate velocity as quaternion using dq/dt = 0.5 * w * q
		snVector4f qw;
		snQuaternionMultiply(m_w, m_q, qw);
		qw = qw * 0.5f;

		//calculate orientation using euler integration
		m_q = m_q + (qw * _dt);
		snQuaternionNormalize(m_q, m_q);

		//compute orientation as a matrix
		m_R.createRotationFromQuaternion(m_q);

		//set new state
		computeInvWorldInertia();

		//compute colliders in world coordinate
		updateCollidersAndAABB();
	}
}