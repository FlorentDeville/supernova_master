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

#include "snActorStatic.h"
#include "snICollider.h"
#include "snColliderContainer.h"

using namespace Supernova::Vector;

namespace Supernova
{

	snActorStatic::snActorStatic()
	{
		init(snVec4Set(0, 0, 0, 1), snVec4Set(0, 0, 0, 1));
	}

	snActorStatic::snActorStatic(const snVec& _position)
	{
		init(_position, snVec4Set(0, 0, 0, 1));
	}

	snActorStatic::snActorStatic(const snVec& _position, const snVec& _orientation)
	{
		init(_position, _orientation);
	}

	snActorStatic::~snActorStatic()
	{

	}

	float snActorStatic::getMass() const
	{
		return 0;
	}

	//Return the inverse of the mass
	float snActorStatic::getInvMass() const
	{
		return 0;
	}

	//Return the inverse of the inertia expressed in world coordinate
	const snMatrix44f& snActorStatic::getInvWorldInertia() const
	{
		return snMatrix44f::m_zero;
	}

	//Return the linear velocity
	snVec snActorStatic::getLinearVelocity() const
	{
		return VEC_ZERO;
	}

	//Return the angular velocity
	snVec snActorStatic::getAngularVelocity() const
	{
		return VEC_ZERO;
	}

	//Set the linear velocity
	void snActorStatic::setLinearVelocity(const snVec& /*_linearVelocity*/)
	{
		return;
	}

	//Set the angular velocity
	void snActorStatic::setAngularVelocity(const snVec& /*_angularVelocity*/)
	{
		return;
	}

	//Move the actor forward in time using _dt as a time step.
	//_linearSpeed2Limit and _angularSpeed2Limit are the squared speed below which the velocities will be set to 0.
	//A static actor cannot move so this function doesn't do anyhthing.
	void snActorStatic::integrate(float /*_dt*/, float /*_linearSpeed2Limit*/, float /*_angularSpeed2Limit*/)
	{
		return;
	}

	void snActorStatic::initialize()
	{
		//loop through each colliders to initialize them
		for (vector<snColliderContainer*>::iterator i = m_colliders.begin(); i != m_colliders.end(); ++i)
		{
			(*i)->m_collider->initialize();
			(*i)->m_collider->setTransform(m_transform);				
		}

		//compute the AABB
		computeBoundingVolume();
	}

	void snActorStatic::init(const snVec& _position, const snVec& _orientation)
	{
		m_name = "default";

		m_transform.setPosition(_position);
		m_transform.setOrientation(_orientation);

		m_R.createRotationFromQuaternion(_orientation);
		m_invR = m_R.inverse();

		m_skinDepth = 0.025f;
		m_typeOfActor = snActorType::snActorTypeStatic;
		m_collisionFlag = snCollisionFlag::CF_NO_FLAG;
		m_collisionCallback = 0;
		m_isActive = true;
		m_centerOfMass = snVec4Set(0, 0, 0, 1);
		m_worldCenterOfMass = snVec4Set(0, 0, 0, 1);
	}
}