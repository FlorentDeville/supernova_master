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

#include "snPointToPointConstraint.h"
#include "snActor.h"
#include "snMath.h"

namespace Supernova
{
	snPointToPointConstraint::snPointToPointConstraint(snActor* const _bodyA, const snVector4f& _pivotA, snActor* const _bodyB,
		const snVector4f& _pivotB)
		: snIConstraint()
	{
		m_bodies[0] = _bodyA;
		m_pivot[0] = _pivotA;

		m_bodies[1] = _bodyB;
		m_pivot[1] = _pivotB;
	}

	snPointToPointConstraint::~snPointToPointConstraint()
	{}

	void snPointToPointConstraint::prepare()
	{
		const int ACTOR_COUNT = 2;
		snMatrix44f invMass[2];

		snMatrix44f RinvIRT[2];

		for (int i = 0; i < ACTOR_COUNT; ++i)
		{
			//compute the offset in world coordinates.
			snMatrix44f transform = m_bodies[i]->getOrientationMatrix();
			transform[3] = m_bodies[i]->getPosition();
			m_worldPivot[i] = snMatrixTransform4(m_pivot[i], transform);

			//compute the offset
			m_offset[i] = m_worldPivot[i] - m_bodies[i]->getPosition();

			//compte the skew matrix
			m_R[i][0] = snVector4f(0, m_offset[i].getZ(), -m_offset[i].getY(), 0);
			m_R[i][1] = snVector4f(-m_offset[i].getZ(), 0, m_offset[i].getX(), 0);
			m_R[i][2] = snVector4f(m_offset[i].getY(), -m_offset[i].getX(), 0, 0);
			m_R[i][3] = snVector4f(0, 0, 0, 1);

			//compute inverse mass matrix
			invMass[i][0] = snVector4f(m_bodies[i]->getInvMass(), 0, 0, 0);
			invMass[i][1] = snVector4f(0, m_bodies[i]->getInvMass(), 0, 0);
			invMass[i][2] = snVector4f(0, 0, m_bodies[i]->getInvMass(), 0);
			invMass[i][3] = snVector4f(0, 0, 0, 1);

			//compute R * I-1 * RT
			snMatrix44f RT;
			m_R[i].transpose(RT);

			snMatrix44f RInvI;
			snMatrixMultiply3(m_R[i], m_bodies[i]->getInvWorldInertia(), RInvI);
			snMatrixMultiply3(RInvI, RT, RinvIRT[i]);

			//compute I-1 * R
			snMatrixMultiply3(m_bodies[i]->getInvWorldInertia(), m_R[i], m_InvIR[i]);
		}

		//compute the effective mass and then its inverse.
		snMatrix44f effectiveMass = invMass[0] + RinvIRT[0] + invMass[1] + RinvIRT[1];
		effectiveMass[3] = snVector4f(0, 0, 0, 1);
		m_invEffectiveMass = effectiveMass.inverse();
	}

	void snPointToPointConstraint::resolve()
	{
		//compute the jacobian times the relative velocity.
		snVector4f JV = m_bodies[0]->getLinearVelocity() + m_bodies[0]->getAngularVelocity().cross(m_worldPivot[0]) -
			m_bodies[1]->getLinearVelocity() - m_bodies[1]->getAngularVelocity().cross(m_worldPivot[1]);

		/*JV = m_bodies[1]->getLinearVelocity() - m_bodies[0]->getLinearVelocity() + m_bodies[0]->getAngularVelocity().cross(m_worldPivot[0])
			- m_bodies[1]->getAngularVelocity().cross(m_worldPivot[1]);*/

		//compute lagrangian
		snVector4f lagrangian = snMatrixTransform3(JV, m_invEffectiveMass);

		////clamp lambda
		//float oldAccLambda = m_accumulatedImpulseMagnitude;
		//m_accumulatedImpulseMagnitude += lagrangian;
		//m_accumulatedImpulseMagnitude = clamp(m_accumulatedImpulseMagnitude, -SN_FLOAT_MAX, 0);
		//lagrangian = m_accumulatedImpulseMagnitude - oldAccLambda;

		//compute linear velocity
		m_bodies[0]->setLinearVelocity(m_bodies[0]->getLinearVelocity() - lagrangian * m_bodies[0]->getInvMass());
		m_bodies[1]->setLinearVelocity(m_bodies[1]->getLinearVelocity() + lagrangian * m_bodies[1]->getInvMass());

		//compute angular velocity
		snMatrix44f RT[2];
		m_R[0].transpose(RT[0]);
		m_R[1].transpose(RT[1]);

		snMatrix44f InvIRT[2];
		snMatrixMultiply3(m_bodies[0]->getInvWorldInertia(), RT[0], InvIRT[0]);
		snMatrixMultiply3(m_bodies[1]->getInvWorldInertia(), RT[1], InvIRT[1]);

		snVector4f dw[2];
		dw[0] = snMatrixTransform3(InvIRT[0], lagrangian);
		dw[1] = snMatrixTransform3(InvIRT[1], lagrangian);
		m_bodies[0]->setAngularVelocity(m_bodies[0]->getAngularVelocity() - dw[0]);
		m_bodies[1]->setAngularVelocity(m_bodies[1]->getAngularVelocity() + dw[1]);
	}
}