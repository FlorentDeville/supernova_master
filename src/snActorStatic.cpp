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

namespace Supernova
{

	snActorStatic::snActorStatic()
	{
		init(snVector4f(), snVector4f(0, 0, 0, 1));
	}

	snActorStatic::snActorStatic(const snVector4f& _position)
	{
		init(_position, snVector4f(0, 0, 0, 1));
	}

	snActorStatic::snActorStatic(const snVector4f& _position, const snVector4f& _orientation)
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
	snVector4f snActorStatic::getLinearVelocity() const
	{
		return snVector4f::m_zero;
	}

	//Return the angular velocity
	snVector4f snActorStatic::getAngularVelocity() const
	{
		return snVector4f::m_zero;
	}

	//Set the linear velocity
	void snActorStatic::setLinearVelocity(const snVector4f& /*_linearVelocity*/)
	{
		return;
	}

	//Set the angular velocity
	void snActorStatic::setAngularVelocity(const snVector4f& /*_angularVelocity*/)
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
		//create the transform matrix
		snMatrix44f translation;
		translation.createTranslation(m_x);
		snMatrix44f transform;
		snMatrixMultiply4(m_R, translation, transform);

		//loop through each colliders to initialize them
		for (vector<snICollider*>::iterator i = m_colliders.begin(); i != m_colliders.end(); ++i)
		{
			(*i)->initialize();
			(*i)->setWorldTransform(transform);				
		}

		//compute the AABB
		computeBoundingVolume();
	}

	void snActorStatic::init(const snVector4f& _position, const snVector4f& _orientation)
	{
		m_name = "default";

		m_x = _position;

		m_q = _orientation;
		m_R.createRotationFromQuaternion(m_q);
		m_invR = m_R.inverse();

		m_skinDepth = 0.025f;
		m_typeOfActor = snActorType::snActorTypeStatic;
		m_collisionFlag = snCollisionFlag::CF_NO_FLAG;
		m_collisionCallback = 0;
	}
}