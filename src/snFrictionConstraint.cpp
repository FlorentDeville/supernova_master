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
#include "snContactConstraint.h"

#include "snVec.inl"

using namespace Supernova::Vector;

namespace Supernova
{
	snFrictionConstraint::snFrictionConstraint()
		:snIConstraint(), m_effectiveMass(snVec4Set(0)), m_accumulatedImpulseMagnitude(snVec4Set(0))
	{
	}

	snFrictionConstraint::~snFrictionConstraint(){}

	void snFrictionConstraint::initialize(snIActor* const _body1, snIActor* const _body2, snContactConstraint const * _npConstraint)
	{
		m_bodies[0] = _body1;
		m_bodies[1] = _body2;
		m_npConstraint = _npConstraint;
	}

	void snFrictionConstraint::prepare(float /*_dt*/)
	{
		m_accumulatedImpulseMagnitude = snVec4Set(0);
		m_secondAccumulatedImpulseMagnitude = snVec4Set(0);

		//Compute the friction coefficient as the average of frictions of the two objects.
		m_frictionCoefficient = (m_bodies[0]->getPhysicMaterial().m_friction + m_bodies[1]->getPhysicMaterial().m_friction) * 0.5f;

		//compute tangent vectors
		computeBasis(m_npConstraint->getNormal(), m_tangent[0], m_tangent[1]);

		snVec sumInvMass = snVec3Set(m_bodies[0]->getInvMass() + m_bodies[1]->getInvMass());

		//compute the effective mass along the first tangent vector
		snVec tempA = snVec3Cross(m_npConstraint->getRadius()[0], m_tangent[0]);
		m_rCrossT0InvI[0] = snMatrixTransform3(tempA, m_bodies[0]->getInvWorldInertia());
		snVec tempB = snVec3Cross(m_npConstraint->getRadius()[1], m_tangent[0]);
		m_rCrossT0InvI[1] = snMatrixTransform3(tempB, m_bodies[1]->getInvWorldInertia());
		m_effectiveMass =(sumInvMass +
			snVec3Dot(snVec3Cross(m_rCrossT0InvI[0], m_npConstraint->getRadius()[0]) + snVec3Cross(m_rCrossT0InvI[1], m_npConstraint->getRadius()[1]), m_tangent[0]));
		m_effectiveMass = snVec4GetInverse(m_effectiveMass);

		//compute the effective mass along the second tangent vector
		tempA = snVec3Cross(m_npConstraint->getRadius()[0], m_tangent[1]);
		m_rCrossT1InvI[0] = snMatrixTransform3(tempA, m_bodies[0]->getInvWorldInertia());
		tempB = snVec3Cross(m_npConstraint->getRadius()[1], m_tangent[1]);
		m_rCrossT1InvI[1] = snMatrixTransform3(tempB, m_bodies[1]->getInvWorldInertia());

		m_secondEffectiveMass =(sumInvMass +
			snVec3Dot(snVec3Cross(m_rCrossT1InvI[0], m_npConstraint->getRadius()[0]) + snVec3Cross(m_rCrossT1InvI[1], m_npConstraint->getRadius()[1]), m_tangent[1]));
		m_secondEffectiveMass = snVec4GetInverse(m_secondEffectiveMass);
	}

	void snFrictionConstraint::resolve()
	{
		snVec clampingValue = m_frictionCoefficient * m_npConstraint->getAccumulatedImpulseMagnitude();

		//compute relative velocity
		snVec linVel1 = m_bodies[1]->getLinearVelocity();
		snVec angVel1 = snVec3Cross(m_bodies[1]->getAngularVelocity(), m_npConstraint->getRadius()[1]);
		snVec linVel0 = m_bodies[0]->getLinearVelocity();
		snVec angVel0 = snVec3Cross(m_bodies[0]->getAngularVelocity(), m_npConstraint->getRadius()[0]);

		snVec dv = linVel1 + angVel1 - linVel0 - angVel0;

		//compute lagrangian for the fist tangent
		snVec lagrangian = -snVec3Dot(dv, m_tangent[0]) * m_effectiveMass;

		//clamp the lagrangian
		snVec tempLambda = m_accumulatedImpulseMagnitude;
		m_accumulatedImpulseMagnitude = snVec4Clamp(m_accumulatedImpulseMagnitude + lagrangian, clampingValue, -clampingValue);
		lagrangian = m_accumulatedImpulseMagnitude - tempLambda;


		snVec impulse = m_tangent[0] * lagrangian;

		//apply the impulse
		m_bodies[0]->setLinearVelocity(m_bodies[0]->getLinearVelocity() - impulse * m_bodies[0]->getInvMass());
		m_bodies[1]->setLinearVelocity(m_bodies[1]->getLinearVelocity() + impulse * m_bodies[1]->getInvMass());

		m_bodies[0]->setAngularVelocity(m_bodies[0]->getAngularVelocity() - m_rCrossT0InvI[0] * lagrangian);
		m_bodies[1]->setAngularVelocity(m_bodies[1]->getAngularVelocity() + m_rCrossT0InvI[1] * lagrangian);

		//compute the relative velocity
		snVec vec1 = m_bodies[1]->getLinearVelocity();
		snVec vec2 = snVec3Cross(m_bodies[1]->getAngularVelocity(), m_npConstraint->getRadius()[1]);
		snVec vec3 = m_bodies[0]->getLinearVelocity();
		snVec vec4 = snVec3Cross(m_bodies[0]->getAngularVelocity(), m_npConstraint->getRadius()[0]);

		dv = vec1 + vec2 - vec3 - vec4;

		//compute and clamp the impulse along the second tangent
		lagrangian = -snVec3Dot(dv, m_tangent[1]) * m_secondEffectiveMass;
		tempLambda = m_secondAccumulatedImpulseMagnitude;
		m_secondAccumulatedImpulseMagnitude = snVec4Clamp(m_secondAccumulatedImpulseMagnitude + lagrangian, clampingValue, -clampingValue);
		lagrangian = m_secondAccumulatedImpulseMagnitude - tempLambda;


		impulse = m_tangent[1] * lagrangian;

		//apply the impulse
		m_bodies[0]->setLinearVelocity(m_bodies[0]->getLinearVelocity() - impulse * m_bodies[0]->getInvMass());
		m_bodies[1]->setLinearVelocity(m_bodies[1]->getLinearVelocity() + impulse * m_bodies[1]->getInvMass());

		m_bodies[0]->setAngularVelocity(m_bodies[0]->getAngularVelocity() - m_rCrossT1InvI[0] * lagrangian);
		m_bodies[1]->setAngularVelocity(m_bodies[1]->getAngularVelocity() + m_rCrossT1InvI[1] * lagrangian);
	}
}