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

#ifndef SN_FIXED_CONSTRAINT_H
#define SN_FIXED_CONSTRAINT_H

#include "snIConstraint.h"
#include "snMatrix44f.h"

namespace Supernova
{
	class snActor;

	//Represent a constraint of a body staying at the same distance of a point in space.
	class snFixedConstraint : public snIConstraint
	{
	protected:

		//Actor this constraint has to be applied to.
		snActor* m_actor;

		//Point in space. The actor has to stay at the same distance to this point.
		snVector4f m_fixedPoint;

		//Skew matrix used to compute the cross product r X w
		snMatrix44f m_R;

		//The mass expressed in the constraint frame of reference. It is equal to 1 / (J * M-1 * JT).
		snMatrix44f m_effectiveMass;

		//The distance between the fixed point and the actor's center of mass.
		float m_distance;

		//The time step. Necessary to compute baumgarte stabilization.
		float m_dt;

		//Velocity bias computed using baumgarte stabilization.
		snVector4f m_bias;

		//I-1 * R
		snMatrix44f m_invIR;

		//Normalized vector going from the actor's center of mass to the constraint's fixed point.
		snVector4f m_normalizedOffset;

	public:
		snFixedConstraint(snActor* const _actor, const snVector4f& _fixedPoint, float _dt);
		~snFixedConstraint();

		void prepare();

		void resolve();

	};
}

#endif //ifndef SN_FIXED_CONSTRAINT_H