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
	class snIActor;

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	//Represent a constraint of a body staying at the same distance of a point in space.
	//
	//Position Constraint : C = x + r - p with : 
	//			- x as the position of the actor.
	//			- r as the vector from the actor origin to the pivot point (expressed in world coordinate).
	//			- p as the pivot point (expressed in world coordinate).
	//
	//Velocity Constraint : dC/dt = v + Rs * w with :
	//			- v as the linear velocity of the actor.
	//			- Rs as the skew symmetric matrices used to compute a cross product. Rs * w = w X r
	//			- w as the angular velocity of the actor.
	//
	//Jacobian : J = [E Rs] with : 
	//			- E as the identity matrix.
	//			- Rs as the skew symmetric matrices used to compute a cross product. Rs * w = w X r
	//
	//K Matrix : K = M-1 + Rs * I-1 * RsT with :
	//			- M-1 as the inverse masss matrix.
	//			- Rs as the skew symmetric matrices used to compute a cross product. Rs * w = w X r
	//			- I-1 as the inverse inertia tensor expressed in world coordinate.
	//			- RsT as the transpose of Rs.
	//
	//Linear velocity : v = v + m-1 * l with :
	//			- v as linear velocity of the actor.
	//			- m-1 as the inverse of the mass.
	//			- l as the lagrangian.
	//
	//Angular velocity : w + I-1 * RsT * l with :
	//			- w as the angular velocity.
	//			- I-1 as the inverse inertia tensor expressed in world coordinate.
	//			- RsT as the transpose of Rs.
	//			- l as the lagrangian
	//
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	class snFixedConstraint : public snIConstraint
	{
	protected:

		//Actor this constraint has to be applied to.
		snIActor* m_actor;

		//Point in space. The actor has to stay at the same distance to this point.
		snVec m_fixedPoint;

		//Skew matrix used to compute the cross product r X w
		snMatrix44f m_R;

		//The mass expressed in the constraint frame of reference. It is equal to 1 / (J * M-1 * JT).
		snMatrix44f m_effectiveMass;

		//The distance between the fixed point and the actor's center of mass.
		float m_distance;

		//Velocity bias computed using baumgarte stabilization.
		snVec m_bias;

		//I-1 * RT
		snMatrix44f m_invIRT;

		//Vector from the actor origin to the fixed point.
		snVec m_offset;

		//Normalized vector going from the actor's center of mass to the constraint's fixed point.
		snVec m_normalizedOffset;

	public:
		snFixedConstraint(snIActor* const _actor, const snVec& _fixedPoint, float _distance);
		~snFixedConstraint();

		void prepare(float _dt);

		void resolve();

		//Return the world coordinate of the fixed point.
		snVec getFixedPosition() const;

		//Return the distance between the fixed point and the actor.
		float getDistance() const;

		const snIActor* getActor() const;

	};
}

#endif //ifndef SN_FIXED_CONSTRAINT_H