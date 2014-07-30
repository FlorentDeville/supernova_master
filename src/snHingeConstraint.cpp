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

#include "snHingeConstraint.h"
#include "snIActor.h"
#include "snMath.h"
using namespace Supernova::Vector;

namespace Supernova
{
	snHingeConstraint::snHingeConstraint(snIActor* _actor, const snVec& _axis, const snVec& _anchor) : m_actor(_actor), m_axis(_axis),
		m_anchor(_anchor)
	{
		snMatrix44f invTransform = m_actor->getTransform().getLocalToWorld().inverse();
		m_localAxis = snMatrixTransform3(m_axis, invTransform);
	}

	snHingeConstraint::~snHingeConstraint(){}

	void snHingeConstraint::prepare(float /*_dt*/)
	{
		//compute the skew matrix
		m_radius = m_anchor - m_actor->getPosition();
		m_r[0] = snVec4Set(0, snVec4GetZ(m_radius), -snVec4GetY(m_radius), 0);
		m_r[1] = snVec4Set(-snVec4GetZ(m_radius), 0, snVec4GetX(m_radius), 0);
		m_r[2] = snVec4Set(snVec4GetY(m_radius), -snVec4GetX(m_radius), 0, 0);
		m_r[3] = snVec4Set(0, 0, 0, 1);

		//compute inverse mass matrix
		snMatrix44f invMass;
		invMass[0] = snVec4Set(m_actor->getInvMass(), 0, 0, 0);
		invMass[1] = snVec4Set(0, m_actor->getInvMass(), 0, 0);
		invMass[2] = snVec4Set(0, 0, m_actor->getInvMass(), 0);
		invMass[3] = snVec4Set(0, 0, 0, 1);

		snMatrix44f rI;
		snMatrixMultiply3(m_r, m_actor->getInvWorldInertia(), rI);

		snMatrix44f rT;
		m_r.transpose(rT);
		snMatrix44f rInvIrT;
		snMatrixMultiply3(rI, rT, rInvIrT);

		snMatrix44f KTrans = invMass + rInvIrT;
		m_KTrans = KTrans.inverse();

		snMatrixMultiply3(m_invIRT, m_actor->getInvWorldInertia(), rT);

		//Rotation constraints

		//expressed the local axis in world coordinate
		snVec worldAxis = snMatrixTransform3(m_localAxis, m_actor->getTransform().getLocalToWorld());

		//compute a base from the world axis
		snVec b, c;
		computeBasis(worldAxis, b, c);
		m_bCrossA = snVec3Cross(b, worldAxis);
		m_cCrossA = snVec3Cross(c, worldAxis);

		snVec bCrossAInvI = snMatrixTransform3(m_bCrossA, m_actor->getInvWorldInertia());
		m_KRot1 = snVec3Dot(bCrossAInvI, m_bCrossA);
		m_invIBCrossA = snMatrixTransform3(m_actor->getInvWorldInertia(), m_bCrossA);

		snVec cCrossAInvI = snMatrixTransform3(m_cCrossA, m_actor->getInvWorldInertia());
		m_KRot2 = snVec3Dot(cCrossAInvI, m_cCrossA);
		m_invICCrossA = snMatrixTransform3(m_actor->getInvWorldInertia(), m_cCrossA);

		//baumgarte stabilization
		/*float beta = 0.1f;
		float ds = m_initialDistanceToAnchor - snVec3Norme(m_radius);
		m_biasTrans = beta / _dt * (m_anchor * ds);*/
	}

	void snHingeConstraint::resolve()
	{
		//solve translation constraint
		snVec JV = m_actor->getLinearVelocity() + snVec3Cross(m_actor->getAngularVelocity(), m_radius);
		snVec lagrangian = snMatrixTransform3(-JV, m_KTrans);

		m_actor->setLinearVelocity(m_actor->getLinearVelocity() + m_actor->getInvMass() * lagrangian);
		m_actor->setAngularVelocity(m_actor->getAngularVelocity() + snMatrixTransform3(m_invIRT, lagrangian));

		//solve first rotation constraint
		snVec JRot1 = snVec3Dot(m_bCrossA, m_actor->getAngularVelocity());
		snVec lagrangianRot1 = -JRot1 / m_KRot1;
		snVec4SetW(lagrangianRot1, 0);
		m_actor->setAngularVelocity(m_actor->getAngularVelocity() + m_invIBCrossA * lagrangianRot1);

		//solve second rotation constraint
		snVec JRot2 = snVec3Dot(m_cCrossA, m_actor->getAngularVelocity());
		snVec lagrangianRot2 = -JRot2 / m_KRot2;
		snVec4SetW(lagrangianRot2, 0);
		m_actor->setAngularVelocity(m_actor->getAngularVelocity() + m_invICCrossA * lagrangianRot2);
	}
}