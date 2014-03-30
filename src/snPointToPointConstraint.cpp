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
		m_actors[0] = _bodyA;
		m_pivot[0] = _pivotA;

		m_actors[1] = _bodyB;
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
			snMatrix44f transform = m_actors[i]->getOrientationMatrix();
			transform[3] = m_actors[i]->getPosition();
			m_worldPivot[i] = snMatrixTransform4(m_pivot[i], transform);

			//compute the offset
			m_offset[i] = m_worldPivot[i] - m_actors[i]->getPosition();

			//compte the skew matrix
			m_R[i][0] = snVector4f(0, m_offset[i].getZ(), -m_offset[i].getY(), 0);
			m_R[i][1] = snVector4f(-m_offset[i].getZ(), 0, m_offset[i].getX(), 0);
			m_R[i][2] = snVector4f(m_offset[i].getY(), -m_offset[i].getX(), 0, 0);
			m_R[i][3] = snVector4f(0, 0, 0, 1);

			//compute inverse mass matrix
			invMass[i][0] = snVector4f(m_actors[i]->getInvMass(), 0, 0, 0);
			invMass[i][1] = snVector4f(0, m_actors[i]->getInvMass(), 0, 0);
			invMass[i][2] = snVector4f(0, 0, m_actors[i]->getInvMass(), 0);
			invMass[i][3] = snVector4f(0, 0, 0, 1);

			//compute R * I-1 * RT
			snMatrix44f RT;
			m_R[i].transpose(RT);

			snMatrix44f RInvI;
			snMatrixMultiply3(m_R[i], m_actors[i]->getInvWorldInertia(), RInvI);
			snMatrixMultiply3(RInvI, RT, RinvIRT[i]);

			//compute I-1 * R
			snMatrixMultiply3(m_actors[i]->getInvWorldInertia(), RT, m_InvIRT[i]);
		}

		//compute the effective mass and then its inverse.
		snMatrix44f KMatrix = invMass[0] + RinvIRT[0] + invMass[1] + RinvIRT[1];
		KMatrix[3] = snVector4f(0, 0, 0, 1);
		m_invEffectiveMass = KMatrix.inverse();

		//compute baumgarte stabilization
		float beta = 0.1f;
		float dt = 0.016f;
		snVector4f error = m_worldPivot[0] - m_worldPivot[1];
		m_bias = error * (beta / dt);
		//m_bias = snVector4f(0, 0, 0, 0);
	}

	void snPointToPointConstraint::resolve()
	{
		//compute the jacobian times the relative velocity.
		snVector4f JV = m_actors[0]->getLinearVelocity() + m_actors[0]->getAngularVelocity().cross(m_offset[0]) -
			m_actors[1]->getLinearVelocity() - m_actors[1]->getAngularVelocity().cross(m_offset[1]);

		//compute lagrangian
		snVector4f lagrangian = snMatrixTransform3(-JV - m_bias, m_invEffectiveMass);

		//compute linear velocity
		m_actors[0]->setLinearVelocity(m_actors[0]->getLinearVelocity() + lagrangian * m_actors[0]->getInvMass());
		m_actors[1]->setLinearVelocity(m_actors[1]->getLinearVelocity() - lagrangian * m_actors[1]->getInvMass());

		m_actors[0]->setAngularVelocity(m_actors[0]->getAngularVelocity() + snMatrixTransform3(m_InvIRT[0], lagrangian));
		m_actors[1]->setAngularVelocity(m_actors[1]->getAngularVelocity() - snMatrixTransform3(m_InvIRT[1], lagrangian));
	}

	snActor const * const * snPointToPointConstraint::getActors() const
	{
		return m_actors;
	}

	snVector4f const * snPointToPointConstraint::getWPivot() const
	{
		return m_worldPivot;
	}

	snVector4f const* snPointToPointConstraint::getOffset() const
	{
		return m_offset;
	}
}