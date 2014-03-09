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

#include "snFrictionConstraint.h"
#include "snMath.h"
#include "snNonPenetrationConstraint.h"

namespace Supernova
{
	snFrictionConstraint::snFrictionConstraint()
		:snIConstraint()
	{
	}

	snFrictionConstraint::~snFrictionConstraint(){}

	void snFrictionConstraint::initialize(snActor* const _body1, snActor* const _body2, snNonPenetrationConstraint const * _npConstraint)
	{
		m_bodies[0] = _body1;
		m_bodies[1] = _body2;
		m_npConstraint = _npConstraint;
	}

	void snFrictionConstraint::prepare()
	{
		m_accumulatedImpulseMagnitude = 0;
		m_secondAccumulatedImpulseMagnitude = 0;

		//Compute the friction coefficient as the average of frictions of the two objects.
		m_frictionCoefficient = (m_bodies[0]->getPhysicMaterial().m_friction + m_bodies[1]->getPhysicMaterial().m_friction) * 0.5f;

		//compute tangent vectors
		computeBasis(m_npConstraint->getNormal(), m_tangent[0], m_tangent[1]);

		float sumInvMass = m_bodies[0]->getInvMass() + m_bodies[1]->getInvMass();

		//compute the effective mass along the first tangent vector
		snVector4f tempA = m_npConstraint->getRadius()[0].cross(m_tangent[0]);
		m_rCrossT0InvI[0] = snMatrixTransform3(tempA, m_bodies[0]->getInvWorldInertia());
		snVector4f tempB = m_npConstraint->getRadius()[1].cross(m_tangent[0]);
		m_rCrossT0InvI[1] = snMatrixTransform3(tempB, m_bodies[1]->getInvWorldInertia());
		tempA.setW(0);
		tempB.setW(0);
		m_effectiveMass = 1.f / (sumInvMass +
			(m_rCrossT0InvI[0].cross(m_npConstraint->getRadius()[0]) + m_rCrossT0InvI[1].cross(m_npConstraint->getRadius()[1])).dot(m_tangent[0]));

		//compute the effective mass along the second tangent vector
		tempA = m_npConstraint->getRadius()[0].cross(m_tangent[1]);
		m_rCrossT1InvI[0] = snMatrixTransform3(tempA, m_bodies[0]->getInvWorldInertia());
		tempB = m_npConstraint->getRadius()[1].cross(m_tangent[1]);
		m_rCrossT1InvI[1] = snMatrixTransform3(tempB, m_bodies[1]->getInvWorldInertia());

		m_secondEffectiveMass = 1.f / (sumInvMass +
			(m_rCrossT1InvI[0].cross(m_npConstraint->getRadius()[0]) + m_rCrossT1InvI[1].cross(m_npConstraint->getRadius()[1])).dot(m_tangent[1]));

	}

	void snFrictionConstraint::resolve()
	{
		float clampingValue = m_frictionCoefficient * m_npConstraint->getAccumulatedImpulseMagnitude();

		//compute relative velocity
		snVector4f dv = m_bodies[1]->getLinearVelocity() + m_bodies[1]->getAngularVelocity().cross(m_npConstraint->getRadius()[1])
			- m_bodies[0]->getLinearVelocity() - m_bodies[0]->getAngularVelocity().cross(m_npConstraint->getRadius()[0]);

		//compute lagrangian for the fist tangent
		float lagrangian = -dv.dot(m_tangent[0]) * m_effectiveMass;

		//clamp the lagrangian
		float tempLambda = m_accumulatedImpulseMagnitude;
		m_accumulatedImpulseMagnitude = clamp(m_accumulatedImpulseMagnitude + lagrangian, clampingValue, -clampingValue);
		lagrangian = m_accumulatedImpulseMagnitude - tempLambda;


		snVector4f impulse = m_tangent[0] * lagrangian;

		//apply the impulse
		m_bodies[0]->setLinearVelocity(m_bodies[0]->getLinearVelocity() - impulse * m_bodies[0]->getInvMass());
		m_bodies[1]->setLinearVelocity(m_bodies[1]->getLinearVelocity() + impulse * m_bodies[1]->getInvMass());

		m_bodies[0]->setAngularVelocity(m_bodies[0]->getAngularVelocity() - m_rCrossT0InvI[0] * lagrangian);
		m_bodies[1]->setAngularVelocity(m_bodies[1]->getAngularVelocity() + m_rCrossT0InvI[1] * lagrangian);

		//compute the relative velocity
		dv = m_bodies[1]->getLinearVelocity() + m_bodies[1]->getAngularVelocity().cross(m_npConstraint->getRadius()[1])
			- m_bodies[0]->getLinearVelocity() - m_bodies[0]->getAngularVelocity().cross(m_npConstraint->getRadius()[0]);

		//compute and clamp the impulse along the second tangent
		lagrangian = -dv.dot(m_tangent[1]) * m_secondEffectiveMass;
		tempLambda = m_secondAccumulatedImpulseMagnitude;
		m_secondAccumulatedImpulseMagnitude = clamp(m_secondAccumulatedImpulseMagnitude + lagrangian, clampingValue, -clampingValue);
		lagrangian = m_secondAccumulatedImpulseMagnitude - tempLambda;


		impulse = m_tangent[1] * lagrangian;

		//apply the impulse
		m_bodies[0]->setLinearVelocity(m_bodies[0]->getLinearVelocity() - impulse * m_bodies[0]->getInvMass());
		m_bodies[1]->setLinearVelocity(m_bodies[1]->getLinearVelocity() + impulse * m_bodies[1]->getInvMass());

		m_bodies[0]->setAngularVelocity(m_bodies[0]->getAngularVelocity() - m_rCrossT1InvI[0] * lagrangian);
		m_bodies[1]->setAngularVelocity(m_bodies[1]->getAngularVelocity() + m_rCrossT1InvI[1] * lagrangian);
	}
}