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

#include "snContactPoint.h"
#include "snActor.h"
#include "snScene.h"
#include "snMath.h"

#include <algorithm>
using std::max;

namespace Supernova
{
	void snContactPoint::initialize(snActor* _body0, snActor* _body1, const snVector4f& _normal, float _penetrationDepth, 
		const snVector4f& _contact)
	{
		m_bodies[0] = _body0;
		m_bodies[1] = _body1;
		m_normal = _normal;
		m_penetration = _penetrationDepth;
		m_point = _contact;
	}

	void snContactPoint::prepare(snScene const * _scene, float _dt)
	{
		//compute relative velocity
		m_ra = m_point - m_bodies[0]->getPosition();
		snVector4f v1 = m_bodies[0]->getLinearVelocity() + m_bodies[0]->getAngularVelocity().cross(m_ra);
		m_rb = m_point - m_bodies[1]->getPosition();
		snVector4f v2 = m_bodies[1]->getLinearVelocity() + m_bodies[1]->getAngularVelocity().cross(m_rb);
		snVector4f relVel = v2 - v1;
		m_preImpRelSpeed = m_normal.dot(relVel);
		m_accumulatedImpulseMag = 0;
		m_accumulatedFrictionImpulse1 = 0;
		m_accumulatedFrictionImpulse2 = 0;

		//compute r cross n
		m_rACrossN = m_ra.cross(m_normal);
		m_rBCrossN = m_rb.cross(m_normal);

		//Compute the effective mass for the non penetration constraint
		// (r X N) I-1
		m_raCrossNInvI = snMatrixTransform3(m_rACrossN, m_bodies[0]->getInvWorldInertia());
		m_rbCrossNInvI = snMatrixTransform3(m_rBCrossN, m_bodies[1]->getInvWorldInertia());

		// [(r X N)I-1] X r
		snVector4f tempA = m_raCrossNInvI.cross(m_ra);
		snVector4f tempB = m_rbCrossNInvI.cross(m_rb);

		float sumInvMass = m_bodies[0]->getInvMass() + m_bodies[1]->getInvMass();

		// 1/ ( 1/ma + 1/mb + ( [(ra X n)Ia-1] X ra + [(rb X n)Ib-1] X rb) . N)
		m_normalEffectiveMass = 1.f / (sumInvMass + (tempA + tempB).dot(m_normal));

		//compute the bias
		float restitution = (m_bodies[0]->getPhysicMaterial().m_restitution + m_bodies[1]->getPhysicMaterial().m_restitution) * 0.5f;
		m_bias = -restitution * m_preImpRelSpeed - _scene->getBeta() / _dt * max<float>(0, m_penetration - _scene->getMaxSlop());

		//compute tangent vectors. They make an orthonormal basis with the normal
		computeBasis(m_normal, m_tangent1, m_tangent2);

		//compute the effective mass along the first tangent vector
		tempA = m_ra.cross(m_tangent1);
		m_raCrossT1InvI = snMatrixTransform3(tempA, m_bodies[0]->getInvWorldInertia());
		tempB = m_rb.cross(m_tangent1);
		m_rbCrossT1InvI = snMatrixTransform3(tempB, m_bodies[1]->getInvWorldInertia());
		//tempA.setW(0);
		//tempB.setW(0);
		m_tangent1EffectiveMass = 1.f / (sumInvMass +
			(m_raCrossT1InvI.cross(m_ra) + m_rbCrossT1InvI.cross(m_rb)).dot(m_tangent1));

		//compute the effective mass along the second tangent vector
		tempA = m_ra.cross(m_tangent2);
		m_raCrossT2InvI = snMatrixTransform3(tempA, m_bodies[0]->getInvWorldInertia());
		tempB = m_rb.cross(m_tangent2);
		m_rbCrossT2InvI = snMatrixTransform3(tempB, m_bodies[1]->getInvWorldInertia());
		tempA.setW(0);
		tempB.setW(0);
		m_tangent2EffectiveMass = 1.f / (sumInvMass +
			(m_raCrossT2InvI.cross(m_ra) + m_rbCrossT2InvI.cross(m_rb)).dot(m_tangent2));

		//Compute the friction coefficient as the average of frictions of the two objects.
		m_frictionCoefficient = (m_bodies[0]->getPhysicMaterial().m_friction + m_bodies[1]->getPhysicMaterial().m_friction) * 0.5f;
	}
}