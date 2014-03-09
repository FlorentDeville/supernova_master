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

#ifndef SN_FRICTION_CONSTRAINT_H
#define SN_FRICTION_CONSTRAINT_H

#include "snIConstraint.h"
#include "snActor.h"

namespace Supernova
{
	class snNonPenetrationConstraint;

	//Friction constraint clamped using a corresponding non penetration constraint.
	class SN_ALIGN snFrictionConstraint : public snIConstraint
	{
	protected:

		//The two bodies involved in this constraints.
		snActor* m_bodies[2];

		//coefficient used to compute the friction.
		float m_frictionCoefficient;

		//Corresponding non penetration constraint used to clamp the friction
		snNonPenetrationConstraint const * m_npConstraint;

		//The two tangent vector along which the friction is applied.
		snVector4f m_tangent[2];

		//(r X T0)I-1
		snVector4f m_rCrossT0InvI[2];

		//(r X T1)I-1
		snVector4f m_rCrossT1InvI[2];

		//The effective mass for the second tangent vector.
		float m_secondEffectiveMass;

		//The accumulated impulse for the second tangent vector.
		float m_secondAccumulatedImpulseMagnitude;

	public:
		snFrictionConstraint();

		virtual ~snFrictionConstraint();

		//Give to the constraint the basic information it needs.
		void initialize(snActor* const _body1, snActor* const _body2, snNonPenetrationConstraint const * _npConstraint);

		void prepare();

		void resolve();
	};
}

#endif //SN_FRICTION_CONSTRAINT_H