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

namespace Supernova
{
	snContactConstraint::snContactConstraint() : snIConstraint(), m_effectiveMass(0)
	{
	}

	snContactConstraint::~snContactConstraint()
	{}

	void snContactConstraint::initialize(snIActor* const _body1, snIActor* const _body2, const snVector4f& _normal, const snVector4f& _collisionPoint, float _penetrationDepth,
		snScene const * _scene)
	{
		m_bodies[0] = _body1;
		m_bodies[1] = _body2;

		m_collisionPoint = _collisionPoint;
		m_penetrationDepth = _penetrationDepth;
		m_normal = _normal;
		m_scene = _scene;
	}

	void snContactConstraint::prepare(float _dt)
	{
		m_accumulatedImpulseMagnitude = 0;

		m_radius[0] = m_collisionPoint - m_bodies[0]->getPosition();
		m_radius[1] = m_collisionPoint - m_bodies[1]->getPosition();

		//compute r cross n
		m_rCrossN[0] = m_radius[0].cross(m_normal);
		m_rCrossN[1] = m_radius[1].cross(m_normal);

		//Compute the effective mass for the non penetration constraint
		// (r X N) I-1
		m_rCrossNInvI[0] = snMatrixTransform3(m_rCrossN[0], m_bodies[0]->getInvWorldInertia());
		m_rCrossNInvI[1] = snMatrixTransform3(m_rCrossN[1], m_bodies[1]->getInvWorldInertia());

		// [(r X N)I-1] X r
		snVector4f tempA = m_rCrossNInvI[0].cross(m_radius[0]);
		snVector4f tempB = m_rCrossNInvI[1].cross(m_radius[1]);

		float sumInvMass = m_bodies[0]->getInvMass() + m_bodies[1]->getInvMass();

		// 1/ ( 1/ma + 1/mb + ( [(ra X n)Ia-1] X ra + [(rb X n)Ib-1] X rb) . N)
		m_effectiveMass = 1.f / (sumInvMass + (tempA + tempB).dot(m_normal));

		//compute relative velocity
		snVector4f v0 = m_bodies[0]->getLinearVelocity() + m_bodies[0]->getAngularVelocity().cross(m_radius[0]);
		snVector4f v1 = m_bodies[1]->getLinearVelocity() + m_bodies[1]->getAngularVelocity().cross(m_radius[1]);
		float relVel = m_normal.dot(v1 - v0);

		//compute the resitution coefficient as the average of the coeff of the actors
		float restitution = (m_bodies[0]->getPhysicMaterial().m_restitution + m_bodies[1]->getPhysicMaterial().m_restitution) * 0.5f;

		//the amount of correction to apply per frame.
		const float beta = 0.25f;

		//compute the velocity correction
		float error = beta / _dt * max<float>(0, m_penetrationDepth - m_bodies[0]->getSkinDepth() - m_bodies[1]->getSkinDepth());
		m_velocityBias = -restitution * relVel - error;

	}

	void snContactConstraint::resolve()
	{
		//compute relative velocity between the two colliding bodies
		snVector4f deltaLinVel = m_bodies[1]->getLinearVelocity() - m_bodies[0]->getLinearVelocity();
		float dv = m_normal.dot(deltaLinVel) +
			m_rCrossN[1].dot(m_bodies[1]->getAngularVelocity()) - m_rCrossN[0].dot(m_bodies[0]->getAngularVelocity());

		//compute lagrange multiplier
		float lagrangian = (m_velocityBias - dv) * m_effectiveMass;

		//clamp lambda
		float oldAccLambda = m_accumulatedImpulseMagnitude;
		m_accumulatedImpulseMagnitude = clamp(m_accumulatedImpulseMagnitude + lagrangian, -SN_FLOAT_MAX, 0);
		lagrangian = m_accumulatedImpulseMagnitude - oldAccLambda;

		//compute the impulse
		snVector4f impulse = m_normal * lagrangian;

		//compute the new linear velocity
		m_bodies[0]->setLinearVelocity(m_bodies[0]->getLinearVelocity() - (impulse * m_bodies[0]->getInvMass()));
		m_bodies[1]->setLinearVelocity(m_bodies[1]->getLinearVelocity() + (impulse * m_bodies[1]->getInvMass()));

		//compute the new angular velocity
		m_bodies[0]->setAngularVelocity(m_bodies[0]->getAngularVelocity() - m_rCrossNInvI[0] * lagrangian);
		m_bodies[1]->setAngularVelocity(m_bodies[1]->getAngularVelocity() + m_rCrossNInvI[1] * lagrangian);
	}

	snVector4f const & snContactConstraint::getNormal() const
	{
		return m_normal;
	}

	snVector4f const * snContactConstraint::getRadius() const
	{
		return m_radius;
	}
}