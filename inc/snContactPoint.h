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

#ifndef SN_CONTACT_POINT_H
#define SN_CONTACT_POINT_H

#include "snVector4f.h"
#include "snGlobals.h"

namespace Supernova
{
	class snActor;

	//Represents a contact point between two bodies. It contains all the precalculation necessary for the sequential impulse solver.
	class SN_ALIGN snContactPoint
	{
	public:

		//Collision normal going from the second body to the first one.
		snVector4f m_normal;

		//Collision point
		snVector4f m_point;

		//Penetration distance.
		float m_penetration;

		//Pointers to the two colliding bodies.
		snActor* m_bodies[2];

		//Relative speed between the two bodies before any impulse correction.
		float m_preImpRelSpeed;

		//The total amount of impulse.
		float m_accumulatedImpulseMag;

		//vector between the collision points and the bodies position
		snVector4f m_ra, m_rb;

		//cross product between the collision point and the collision normal
		snVector4f m_rACrossN, m_rBCrossN;

		//(r X N) I-1
		snVector4f m_raCrossNInvI, m_rbCrossNInvI;

		//(r X t) I-1
		snVector4f m_raCrossT1InvI, m_rbCrossT1InvI, m_raCrossT2InvI, m_rbCrossT2InvI;

		//The effective mass for the non penetration constraint. Its expression is 1 / (J * M-1 * JT)
		float m_normalEffectiveMass;

		//The effective mass for the friction constraint along the first tangent vector. Its expression is 1 / (J * M-1 * JT)
		float m_tangent1EffectiveMass;

		//The effective mass for the friction constraint along the second tangent vector. Its expression is 1 / (J * M-1 * JT)
		float m_tangent2EffectiveMass;

		//The bias for the non penetration constraint. Its expression is : restitution * dv + beta / dt * max(0, penetration - max_slop)
		float m_bias;

		//First tangent.
		snVector4f m_tangent1;

		//Second tangent, orthogonal to the first one.
		snVector4f m_tangent2;

		//The accumulated impulse on the first tangent vector.
		float m_accumulatedFrictionImpulse1;

		//The accumulated impulse on the second tangent vector.
		float m_accumulatedFrictionImpulse2;

		//The friction coefficient. It is the average of the frictions of the two bodies.
		float m_frictionCoefficient;

	public:
		snContactPoint(){};
		~snContactPoint(){};
	};
}

#endif //SN_CONTACT_POINT_H