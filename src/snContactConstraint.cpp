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

#include "snContactConstraint.h"
#include "snIActor.h"
#include "snMath.h"
#include "snScene.h"

#include <algorithm>    
using std::max;

using namespace Supernova::Vector;

namespace Supernova
{

	/// <summary>
	/// Initializes a new instance of the <see cref="snContactConstraint"/> class.
	/// </summary>
	snContactConstraint::snContactConstraint() : snIConstraint(), m_effectiveMass(snVec4Set(0))
	{
	}

	/// <summary>
	/// Finalizes an instance of the <see cref="snContactConstraint"/> class.
	/// </summary>
	snContactConstraint::~snContactConstraint()
	{}

	/// <summary>
	/// Initializes the specified _body1.
	/// </summary>
	/// <param name="_body1">The _body1.</param>
	/// <param name="_body2">The _body2.</param>
	/// <param name="_normal">The _normal.</param>
	/// <param name="_collisionPoint">The _collision point.</param>
	/// <param name="_penetrationDepth">The _penetration depth.</param>
	/// <param name="_scene">The _scene.</param>
	void snContactConstraint::initialize(snIActor* const _body1, snIActor* const _body2, const snVec& _normal, const snVec& _collisionPoint, float _penetrationDepth,
		snScene const * _scene)
	{
		m_bodies[0] = _body1;
		m_bodies[1] = _body2;

		m_collisionPoint = _collisionPoint;
		m_penetrationDepth = _penetrationDepth;
		m_normal = _normal;
		m_scene = _scene;
	}

	/// <summary>
	/// Prepares the specified _DT.
	/// </summary>
	/// <param name="_dt">The _DT.</param>
	void snContactConstraint::prepare(float _dt)
	{
		m_accumulatedImpulseMagnitude = snVec4Set(0);

		//world center of mass = position + center of mass
		//so radius = point - world center of mass = point - position - center of mass
		m_radius[0] = m_collisionPoint - m_bodies[0]->getWorldCenterOfMass();
		m_radius[1] = m_collisionPoint - m_bodies[1]->getWorldCenterOfMass();

		//compute r cross n
		m_rCrossN[0] = snVec3Cross(m_radius[0], m_normal);
		m_rCrossN[1] = snVec3Cross(m_radius[1], m_normal);

		//Compute the effective mass for the non penetration constraint	
		snVec rCrossNInvI = snMatrixTransform3(m_rCrossN[0], m_bodies[0]->getInvWorldInertia()); // (r X N) I-1
		snVec tempA = snVec3Cross(rCrossNInvI, m_radius[0]); // [(r X N)I-1] X r

		rCrossNInvI = snMatrixTransform3(m_rCrossN[1], m_bodies[1]->getInvWorldInertia());
		snVec tempB = snVec3Cross(rCrossNInvI, m_radius[1]);

		snVec sumInvMass = snVec4Set(m_bodies[0]->getInvMass() + m_bodies[1]->getInvMass());

		// 1/ ( 1/ma + 1/mb + ( [(ra X n)Ia-1] X ra + [(rb X n)Ib-1] X rb) . N)
		m_effectiveMass = sumInvMass + snVec3Dot(tempA + tempB, m_normal);
		m_effectiveMass = snVec4GetInverse(m_effectiveMass);

		//compute relative velocity
		snVec v0 = m_bodies[0]->getLinearVelocity() + snVec3Cross(m_bodies[0]->getAngularVelocity(), m_radius[0]);
		snVec v1 = m_bodies[1]->getLinearVelocity() + snVec3Cross(m_bodies[1]->getAngularVelocity(), m_radius[1]);
		snVec relVel = snVec3Dot(m_normal, v1 - v0);

		//compute the resitution coefficient as the average of the coeff of the actors
		float restitution = (m_bodies[0]->getPhysicMaterial().m_restitution + m_bodies[1]->getPhysicMaterial().m_restitution) * 0.5f;

		//compute the velocity correction
		float error = m_scene->getContactConstraintBeta() / _dt * max<float>(0, m_penetrationDepth - m_bodies[0]->getSkinDepth() - m_bodies[1]->getSkinDepth());
		snVec vecError = snVec3Set(-error);
		m_velocityBias = vecError - (restitution * relVel);

		//compute invI * (r x N)T
		m_invI_rCrossN[0] = snMatrixTransform3(m_bodies[0]->getInvWorldInertia(), m_rCrossN[0]);
		m_invI_rCrossN[1] = snMatrixTransform3(m_bodies[1]->getInvWorldInertia(), m_rCrossN[1]);

		{
			m_frictionAccumulatedImpulse[0] = snVec4Set(0);
			m_frictionAccumulatedImpulse[1] = snVec4Set(0);

			//Compute the friction coefficient as the average of frictions of the two objects.
			m_frictionCoefficient = (m_bodies[0]->getPhysicMaterial().m_friction + m_bodies[1]->getPhysicMaterial().m_friction) * 0.5f;

			//compute tangent vectors
			computeBasis(m_normal, m_tangent[0], m_tangent[1]);

			snVec sumInvMass = snVec3Set(m_bodies[0]->getInvMass() + m_bodies[1]->getInvMass());

			//compute the effective mass along the first tangent vector
			snVec tempA = snVec3Cross(m_radius[0], m_tangent[0]);
			m_rCrossT0InvI[0] = snMatrixTransform3(tempA, m_bodies[0]->getInvWorldInertia());
			snVec tempB = snVec3Cross(m_radius[1], m_tangent[0]);
			m_rCrossT0InvI[1] = snMatrixTransform3(tempB, m_bodies[1]->getInvWorldInertia());
			m_frictionEffectiveMass[0] = (sumInvMass +
				snVec3Dot(snVec3Cross(m_rCrossT0InvI[0], m_radius[0]) + snVec3Cross(m_rCrossT0InvI[1], m_radius[1]), m_tangent[0]));
			m_frictionEffectiveMass[0] = snVec4GetInverse(m_frictionEffectiveMass[0]);

			//compute the effective mass along the second tangent vector
			tempA = snVec3Cross(m_radius[0], m_tangent[1]);
			m_rCrossT1InvI[0] = snMatrixTransform3(tempA, m_bodies[0]->getInvWorldInertia());
			tempB = snVec3Cross(m_radius[1], m_tangent[1]);
			m_rCrossT1InvI[1] = snMatrixTransform3(tempB, m_bodies[1]->getInvWorldInertia());

			m_frictionEffectiveMass[1] = (sumInvMass +
				snVec3Dot(snVec3Cross(m_rCrossT1InvI[0], m_radius[0]) + snVec3Cross(m_rCrossT1InvI[1], m_radius[1]), m_tangent[1]));
			m_frictionEffectiveMass[1] = snVec4GetInverse(m_frictionEffectiveMass[1]);
		}
	}

	/// <summary>
	/// Resolves this instance.
	/// </summary>
	void snContactConstraint::resolve()
	{
		//compute relative velocity between the two colliding bodies
		snVec deltaLinVel = m_bodies[1]->getLinearVelocity() - m_bodies[0]->getLinearVelocity();
		snVec dv = snVec3Dot(m_normal, deltaLinVel) +
			snVec3Dot(m_rCrossN[1], m_bodies[1]->getAngularVelocity()) - snVec3Dot(m_rCrossN[0], m_bodies[0]->getAngularVelocity());

		//compute lagrange multiplier
		snVec lagrangian = (m_velocityBias - dv) * m_effectiveMass;

		//clamp lambda
		snVec oldAccLambda = m_accumulatedImpulseMagnitude;
		m_accumulatedImpulseMagnitude = clampComponents(m_accumulatedImpulseMagnitude + lagrangian, -SN_FLOAT_MAX, 0);
		lagrangian = m_accumulatedImpulseMagnitude - oldAccLambda;

		//compute the impulse
		snVec impulse = m_normal * lagrangian;

		//compute the new linear velocity
		m_bodies[0]->setLinearVelocity(m_bodies[0]->getLinearVelocity() - (impulse * m_bodies[0]->getInvMass()));
		m_bodies[1]->setLinearVelocity(m_bodies[1]->getLinearVelocity() + (impulse * m_bodies[1]->getInvMass()));

		//compute the new angular velocity
		m_bodies[0]->setAngularVelocity(m_bodies[0]->getAngularVelocity() - m_invI_rCrossN[0] * lagrangian);
		m_bodies[1]->setAngularVelocity(m_bodies[1]->getAngularVelocity() + m_invI_rCrossN[1] * lagrangian);

		{
			snVec clampingValue = m_frictionCoefficient * m_accumulatedImpulseMagnitude;

			//compute relative velocity
			snVec linVel1 = m_bodies[1]->getLinearVelocity();
			snVec angVel1 = snVec3Cross(m_bodies[1]->getAngularVelocity(), m_radius[1]);
			snVec linVel0 = m_bodies[0]->getLinearVelocity();
			snVec angVel0 = snVec3Cross(m_bodies[0]->getAngularVelocity(), m_radius[0]);

			snVec dv = linVel1 + angVel1 - linVel0 - angVel0;

			//compute lagrangian for the fist tangent
			snVec lagrangian = -snVec3Dot(dv, m_tangent[0]) * m_frictionEffectiveMass[0];

			//clamp the lagrangian
			snVec tempLambda = m_frictionAccumulatedImpulse[0];
			m_frictionAccumulatedImpulse[0] = snVec4Clamp(m_frictionAccumulatedImpulse[0] + lagrangian, clampingValue, -clampingValue);
			lagrangian = m_frictionAccumulatedImpulse[0] - tempLambda;

			snVec impulse = m_tangent[0] * lagrangian;

			//apply the impulse
			m_bodies[0]->setLinearVelocity(m_bodies[0]->getLinearVelocity() - impulse * m_bodies[0]->getInvMass());
			m_bodies[1]->setLinearVelocity(m_bodies[1]->getLinearVelocity() + impulse * m_bodies[1]->getInvMass());

			m_bodies[0]->setAngularVelocity(m_bodies[0]->getAngularVelocity() - m_rCrossT0InvI[0] * lagrangian);
			m_bodies[1]->setAngularVelocity(m_bodies[1]->getAngularVelocity() + m_rCrossT0InvI[1] * lagrangian);

			//compute the relative velocity
			snVec vec1 = m_bodies[1]->getLinearVelocity();
			snVec vec2 = snVec3Cross(m_bodies[1]->getAngularVelocity(), m_radius[1]);
			snVec vec3 = m_bodies[0]->getLinearVelocity();
			snVec vec4 = snVec3Cross(m_bodies[0]->getAngularVelocity(), m_radius[0]);

			dv = vec1 + vec2 - vec3 - vec4;

			//compute and clamp the impulse along the second tangent
			lagrangian = -snVec3Dot(dv, m_tangent[1]) * m_frictionEffectiveMass[1];
			tempLambda = m_frictionAccumulatedImpulse[1];
			m_frictionAccumulatedImpulse[1] = snVec4Clamp(m_frictionAccumulatedImpulse[1] + lagrangian, clampingValue, -clampingValue);
			lagrangian = m_frictionAccumulatedImpulse[1] - tempLambda;


			impulse = m_tangent[1] * lagrangian;

			//apply the impulse
			m_bodies[0]->setLinearVelocity(m_bodies[0]->getLinearVelocity() - impulse * m_bodies[0]->getInvMass());
			m_bodies[1]->setLinearVelocity(m_bodies[1]->getLinearVelocity() + impulse * m_bodies[1]->getInvMass());

			m_bodies[0]->setAngularVelocity(m_bodies[0]->getAngularVelocity() - m_rCrossT1InvI[0] * lagrangian);
			m_bodies[1]->setAngularVelocity(m_bodies[1]->getAngularVelocity() + m_rCrossT1InvI[1] * lagrangian);
		}
	}

	/// <summary>
	/// Gets the normal.
	/// </summary>
	/// <returns></returns>

	snVec const & snContactConstraint::getNormal() const
	{
		return m_normal;
	}

	/// <summary>
	/// Gets the radius.
	/// </summary>
	/// <returns></returns>
	snVec const * snContactConstraint::getRadius() const
	{
		return m_radius;
	}

	snVec snContactConstraint::getAccumulatedImpulseMagnitude() const
	{
		return m_accumulatedImpulseMagnitude;
	}
}