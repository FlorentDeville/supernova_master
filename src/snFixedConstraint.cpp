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

#include "snFixedConstraint.h"
#include "snIActor.h"
#include "snMath.h"

namespace Supernova
{
	snFixedConstraint::snFixedConstraint(snIActor* const _actor, const snVector4f& _fixedPoint, float _distance)
		: snIConstraint(), m_actor(_actor), m_fixedPoint(_fixedPoint), m_distance(_distance)
	{
		
	}

	snFixedConstraint::~snFixedConstraint(){}

	void snFixedConstraint::prepare(float _dt)
	{
		m_accumulatedImpulseMagnitude = 0;

		//compute the offset between the object position and the fixed point
		m_offset = m_fixedPoint - m_actor->getPosition();

		//Compute the r skew matrix
		m_R[0] = snVector4f(0, m_offset.getZ(), -m_offset.getY(), 0);
		m_R[1] = snVector4f(-m_offset.getZ(), 0, m_offset.getX(), 0);
		m_R[2] = snVector4f(m_offset.getY(), -m_offset.getX(), 0, 0);
		m_R[3] = snVector4f(0, 0, 0, 1);
		snMatrix44f RT;
		m_R.transpose(RT);

		//compute the effective mass
		snMatrix44f RI;
		snMatrixMultiply3(m_R, m_actor->getInvWorldInertia(), RI);

		snMatrix44f RIR;
		snMatrixMultiply3(RI, RT, RIR);

		snMatrix44f invM;
		invM[0] = snVector4f(m_actor->getInvMass(), 0, 0, 0);
		invM[1] = snVector4f(0, m_actor->getInvMass(), 0, 0);
		invM[2] = snVector4f(0, 0, m_actor->getInvMass(), 0);
		invM[3] = snVector4f(0, 0, 0, 0);
		m_effectiveMass = invM + RIR;
		m_effectiveMass = m_effectiveMass.inverse();

		//compute I-1RT
		snMatrixMultiply3(m_actor->getInvWorldInertia(), RT, m_invIRT);

		//compute velocity bias (baumgarte stabilization)
		m_normalizedOffset = m_offset;
		m_normalizedOffset.normalize();
		float beta = 0.1f;

		snVector4f deltaOffset = (m_normalizedOffset * m_distance) - m_offset;
		m_bias = deltaOffset * (beta / _dt);
	}

	void snFixedConstraint::resolve()
	{
		//compute JV
		snVector4f Rw = m_actor->getAngularVelocity().cross(m_offset);
		snVector4f JV = m_actor->getLinearVelocity() + Rw;

		//compute lagrangian
		snVector4f lagrangian = snMatrixTransform3(-JV - m_bias, m_effectiveMass);

		//compute the corrective velocity
		snVector4f dv = lagrangian * m_actor->getInvMass();
		snVector4f dw = snMatrixTransform3(m_invIRT, lagrangian);
		m_actor->setLinearVelocity(m_actor->getLinearVelocity() + dv);
		m_actor->setAngularVelocity(m_actor->getAngularVelocity() + dw);
	}

	snVector4f snFixedConstraint::getFixedPosition() const
	{
		return m_fixedPoint;
	}

	float snFixedConstraint::getDistance() const
	{
		return m_distance;
	}

	const snIActor* snFixedConstraint::getActor() const
	{
		return m_actor;
	}
}