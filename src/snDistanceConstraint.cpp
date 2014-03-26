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

#include "snDistanceConstraint.h"
#include "snActor.h"
#include "snMath.h"

namespace Supernova
{
	snDistanceConstraint::snDistanceConstraint(snActor* const _body1, const snVector4f& _offsetBody1, snActor* const _body2,
		const snVector4f& _offsetBody2, float _distance) : snIConstraint(), m_effectiveMass(0)
	{
		m_bodies[0] = _body1;
		m_localOffset[0] = _offsetBody1;

		m_bodies[1] = _body2;
		m_localOffset[1] = _offsetBody2;

		m_distance = _distance;
	}

	snDistanceConstraint::~snDistanceConstraint()
	{}

	void snDistanceConstraint::prepare()
	{
		//compute the jacobian which in our case is J = (pa - pb) / ||pa - pb||
		m_normalizeddp = m_bodies[1]->getPosition() - m_bodies[0]->getPosition();
		m_normalizeddp.normalize();

		//compute the offsets in the world coordinates
		m_worldOffset[0] = snMatrixTransform3(m_localOffset[0], m_bodies[0]->getOrientationMatrix()) + m_bodies[0]->getPosition();
		m_worldOffset[1] = snMatrixTransform3(m_localOffset[1], m_bodies[1]->getOrientationMatrix()) + m_bodies[1]->getPosition();

		//compute radius
		m_radius[0] = m_worldOffset[0] - m_bodies[0]->getPosition(); // so this is local offset * orientation ???
		m_radius[1] = m_worldOffset[1] - m_bodies[1]->getPosition(); // so this is local offset * orientation ???

		//compute the effective mass : 1 / (ma-1 + mb-1 + ((ra X U)Ia-1 X ra + (rb X U)Ib-1 X rb).U)
		m_rCrossDirection[0] = m_radius[0].cross(m_normalizeddp);
		m_rCrossDirection[1] = m_radius[1].cross(m_normalizeddp);

		m_rCrossUInvI[0] = snMatrixTransform3(m_rCrossDirection[0], m_bodies[0]->getInvWorldInertia());
		m_rCrossUInvI[1] = snMatrixTransform3(m_rCrossDirection[1], m_bodies[1]->getInvWorldInertia());

		m_effectiveMass = 1.f / (m_bodies[0]->getInvMass() + m_bodies[1]->getInvMass() + 
			m_normalizeddp.dot(m_rCrossUInvI[0].cross(m_radius[0]) + m_rCrossUInvI[1].cross(m_radius[1])));

	}

	void snDistanceConstraint::resolve()
	{
		//compute the jacobian times the relative velocity.
		float JV = m_normalizeddp.dot(m_bodies[1]->getLinearVelocity() - m_bodies[0]->getLinearVelocity())
			+ m_bodies[1]->getAngularVelocity().dot(m_rCrossDirection[1]) - m_bodies[0]->getAngularVelocity().dot(m_rCrossDirection[0]);

		//compute lagrangian
		float lagrangian = JV * m_effectiveMass;

		//clamp lambda
		float oldAccLambda = m_accumulatedImpulseMagnitude;
		m_accumulatedImpulseMagnitude += lagrangian;
		m_accumulatedImpulseMagnitude = clamp(m_accumulatedImpulseMagnitude, -SN_FLOAT_MAX, 0);
		lagrangian = m_accumulatedImpulseMagnitude - oldAccLambda;

		//compute impulse
		snVector4f impulse = m_normalizeddp * lagrangian;

		//compute linear velocity
		m_bodies[0]->setLinearVelocity(m_bodies[0]->getLinearVelocity() - impulse * m_bodies[0]->getInvMass());
		m_bodies[1]->setLinearVelocity(m_bodies[1]->getLinearVelocity() + impulse * m_bodies[1]->getInvMass());

		//compute angular velocity
		m_bodies[0]->setAngularVelocity(m_bodies[0]->getAngularVelocity() - m_rCrossUInvI[0] * lagrangian);
		m_bodies[1]->setAngularVelocity(m_bodies[1]->getAngularVelocity() - m_rCrossUInvI[1] * lagrangian);
	}
}