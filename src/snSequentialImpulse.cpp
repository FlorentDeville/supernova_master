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

#include "snSequentialImpulse.h"

#include "snMath.h"

#include "snActor.h"

#include <algorithm>    
using std::max;
using std::min;

namespace Supernova
{
	snSequentialImpulse::snSequentialImpulse()
	{
		m_iterationCount = 10;
	}


	snSequentialImpulse::~snSequentialImpulse()
	{
	}

	
	int snSequentialImpulse::getIterationCount() const
	{
		return m_iterationCount;
	}

	void snSequentialImpulse::setIterationCount(int _iterationCount)
	{
		m_iterationCount = _iterationCount;
	}

	void snSequentialImpulse::solve(snContactPointVector& _contacts) const
	{
		for (int iteration = 0; iteration < m_iterationCount; ++iteration)
		{
			for (snContactPointVectorIterator it = _contacts.begin(); it != _contacts.end(); ++it)
			{
				runIteration(*it);
			}
		}
	}

	void snSequentialImpulse::runIteration(snContactPoint& _contact) const
	{
		resolveNonPenetrationConstraint(_contact);

		resolveFrictionConstraint(_contact);
	}

	void snSequentialImpulse::resolveNonPenetrationConstraint(snContactPoint& _contact) const
	{
		snActor* bodyA = _contact.m_bodies[0];
		snActor* bodyB = _contact.m_bodies[1];

		//compute relative velocity between the two colliding bodies
		snVector4f deltaLinVel = bodyB->getLinearVelocity() - bodyA->getLinearVelocity();
		float dv = _contact.m_normal.dot(deltaLinVel) +
			_contact.m_rBCrossN.dot(bodyB->getAngularVelocity()) - _contact.m_rACrossN.dot(bodyA->getAngularVelocity());

		//compute lagrange multiplier
		float lambda = (_contact.m_bias - dv) * _contact.m_normalEffectiveMass;

		//clamp lambda
		float oldAccLambda = _contact.m_accumulatedImpulseMag;
		_contact.m_accumulatedImpulseMag += lambda;
		_contact.m_accumulatedImpulseMag = clamp(_contact.m_accumulatedImpulseMag, -SN_FLOAT_MAX, 0);
		lambda = _contact.m_accumulatedImpulseMag - oldAccLambda;

		//compute the impulse
		snVector4f impulse = _contact.m_normal * lambda;

		//apply the normal impulse
		bodyA->setLinearVelocity(bodyA->getLinearVelocity() - (impulse * bodyA->getInvMass()));
		bodyB->setLinearVelocity(bodyB->getLinearVelocity() + (impulse * bodyB->getInvMass()));

		bodyA->setAngularVelocity(bodyA->getAngularVelocity() - _contact.m_raCrossNInvI * lambda);
		bodyB->setAngularVelocity(bodyB->getAngularVelocity() + _contact.m_rbCrossNInvI * lambda);
	}


	void snSequentialImpulse::resolveFrictionConstraint(snContactPoint& _contact) const
	{
		float clampingValue = _contact.m_frictionCoefficient * _contact.m_accumulatedImpulseMag;

		snActor* a1 = _contact.m_bodies[0];
		snActor* a2 = _contact.m_bodies[1];

		//compute relative velocity
		snVector4f dv = a2->getLinearVelocity() + a2->getAngularVelocity().cross(_contact.m_rb)
			- a1->getLinearVelocity() - a1->getAngularVelocity().cross(_contact.m_ra);

		//compute and clamp the impulse along the first tangent
		float lambda = -dv.dot(_contact.m_tangent1) * _contact.m_tangent1EffectiveMass;
		float tempLambda = _contact.m_accumulatedFrictionImpulse1;
		_contact.m_accumulatedFrictionImpulse1 = clamp(_contact.m_accumulatedFrictionImpulse1 + lambda, clampingValue, -clampingValue);
		lambda = _contact.m_accumulatedFrictionImpulse1 - tempLambda;


		snVector4f impulse = _contact.m_tangent1 * lambda;

		//apply the impulse
		a1->setLinearVelocity(a1->getLinearVelocity() - impulse * a1->getInvMass());
		a2->setLinearVelocity(a2->getLinearVelocity() + impulse * a2->getInvMass());

		a1->setAngularVelocity(a1->getAngularVelocity() - _contact.m_raCrossT1InvI * lambda);
		a2->setAngularVelocity(a2->getAngularVelocity() + _contact.m_rbCrossT1InvI * lambda);

		//compute and clamp the impulse along the second tangent
		lambda = -dv.dot(_contact.m_tangent2) * _contact.m_tangent2EffectiveMass;
		tempLambda = _contact.m_accumulatedFrictionImpulse2;
		_contact.m_accumulatedFrictionImpulse2 = clamp(_contact.m_accumulatedFrictionImpulse2 + lambda, clampingValue, -clampingValue);
		lambda = _contact.m_accumulatedFrictionImpulse2 - tempLambda;


		impulse = _contact.m_tangent2 * lambda;

		//apply the impulse
		a1->setLinearVelocity(a1->getLinearVelocity() - impulse * a1->getInvMass());
		a2->setLinearVelocity(a2->getLinearVelocity() + impulse * a2->getInvMass());

		a1->setAngularVelocity(a1->getAngularVelocity() - _contact.m_raCrossT2InvI * lambda);
		a2->setAngularVelocity(a2->getAngularVelocity() + _contact.m_rbCrossT2InvI * lambda);
	}
}