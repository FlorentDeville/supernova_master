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
#include "snRigidbody.h"
#include "snMath.h"

using namespace Supernova::Vector;

namespace Supernova
{
	snFixedConstraint::snFixedConstraint(snRigidbody* const _actor, const snVec& _fixedPoint, float _distance)
		: snIConstraint(), m_actor(_actor), m_fixedPoint(_fixedPoint), m_distance(_distance), m_accumulatedImpulseMagnitude(0)
	{
		
	}

	snFixedConstraint::~snFixedConstraint(){}

	void snFixedConstraint::prepare(float _dt)
	{
		m_accumulatedImpulseMagnitude = 0;

		//compute the offset between the object position and the fixed point
		m_offset = m_fixedPoint - m_actor->getWorldCenterOfMass();

		//Compute the r skew matrix
		m_R[0] = snVec4Set(0, snVec4GetZ(m_offset), -snVec4GetY(m_offset), 0);
		m_R[1] = snVec4Set(-snVec4GetZ(m_offset), 0, snVec4GetX(m_offset), 0);
		m_R[2] = snVec4Set(snVec4GetY(m_offset), -snVec4GetX(m_offset), 0, 0);
		m_R[3] = snVec4Set(0, 0, 0, 1);
		snMatrix44f RT;
		m_R.transpose(RT);

		//compute the effective mass
		snMatrix44f RI;
		snMatrixMultiply3(m_R, m_actor->getInvWorldInertia(), RI);

		snMatrix44f RIR;
		snMatrixMultiply3(RI, RT, RIR);

		snMatrix44f invM;
		invM[0] = snVec4Set(m_actor->getInvMass(), 0, 0, 0);
		invM[1] = snVec4Set(0, m_actor->getInvMass(), 0, 0);
		invM[2] = snVec4Set(0, 0, m_actor->getInvMass(), 0);
		invM[3] = snVec4Set(0, 0, 0, 0);
		m_effectiveMass = invM + RIR;
		m_effectiveMass = m_effectiveMass.inverse();

		//compute I-1RT
		snMatrixMultiply3(m_actor->getInvWorldInertia(), RT, m_invIRT);

		//compute velocity bias (baumgarte stabilization)
		m_normalizedOffset = m_offset;
		snVec3Normalize(m_normalizedOffset);
		float beta = 0.1f;

		snVec deltaOffset = (m_normalizedOffset * m_distance) - m_offset;
		m_bias = deltaOffset * (beta / _dt);
	}

	void snFixedConstraint::resolve()
	{
		//compute JV
		snVec Rw = snVec3Cross(m_actor->getAngularVelocity(), m_offset);
		snVec JV = m_actor->getLinearVelocity() + Rw;

		//compute lagrangian
		snVec lagrangian = snMatrixTransform3(-JV - m_bias, m_effectiveMass);

		//compute the corrective velocity
		snVec dv = lagrangian * m_actor->getInvMass();
		snVec dw = snMatrixTransform3(m_invIRT, lagrangian);
		m_actor->setLinearVelocity(m_actor->getLinearVelocity() + dv);
		m_actor->setAngularVelocity(m_actor->getAngularVelocity() + dw);
	}

	snVec snFixedConstraint::getFixedPosition() const
	{
		return m_fixedPoint;
	}

	float snFixedConstraint::getDistance() const
	{
		return m_distance;
	}

	const snRigidbody* snFixedConstraint::getActor() const
	{
		return m_actor;
	}
}