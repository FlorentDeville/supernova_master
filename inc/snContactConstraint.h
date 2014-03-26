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

#ifndef SN_NON_PENETRATION_CONSTRAINT_H
#define SN_NON_PENETRATION_CONSTRAINT_H

#include "snIConstraint.h"

namespace Supernova
{
	class snActor;
	class snScene;

	//Constraint to prevent two actors from penetrating each others.
	class SN_ALIGN snContactConstraint : public snIConstraint
	{
	protected:

		//The two bodies who must repsect the constraint.
		snActor* m_bodies[2];

		//Collision normal going from the second body to the first one.
		snVector4f m_normal;

		//The collision point between the two bodies expressed in world coordinates.
		snVector4f m_collisionPoint;

		//Penetration depth of the two actors.
		float m_penetrationDepth;

		//collision point - center of mass
		snVector4f m_radius[2];

		//r X N
		snVector4f m_rCrossN[2];

		//(r X N)I-1
		snVector4f m_rCrossNInvI[2];

		//Bias velocity
		float m_velocityBias;

		//The mass expressed in the constraint frame of reference. It is equal to 1 / (J * M-1 * JT).
		float m_effectiveMass;

		//Scene containing this constraints.
		snScene const * m_scene;

		//Delta time used to integrate the current step.
		float m_dt;

	public:
		snContactConstraint();

		virtual ~snContactConstraint();

		//Give to the constraints the basic information it needs.
		void initialize(snActor* const _body1, snActor* const _body2, const snVector4f& _normal, const snVector4f& _collisionPoint, float _penetrationDepth,
			snScene const * _scene, float _dt);

		void prepare();

		void resolve();

		//Return the collision normal
		snVector4f const & getNormal() const;

		//Return an array of two vector containing the radius of each actor.
		snVector4f const * getRadius() const;
	};
}

#endif //SN_NON_PENETRATION_CONSTRAINT_H