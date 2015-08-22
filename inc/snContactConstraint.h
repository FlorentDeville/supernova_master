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
#include "snVec.h"

namespace Supernova
{
	class snRigidbody;
	class snScene;

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	//Constraint to prevent two actors from penetrating each others.
	//
	//Position Constraint: C = (pa - pb).n = with pa = x1 + r1 and pb = x2 + r2:
	//						pa, pb respectively the collision point of the first and second body.
	//						xa, xb respectively the position of the first and second body.
	//						ra, rb respectively the vector from the center of mass to the collision point of the first and second body.
	//						n the collision contact of the first body.
	//This constraint means the collision points of the two rigid-bodies has to be the same.
	//
	//Velocity Constraint : dC/dt = va.n + (ra x n).wa - vb.n - (rb x n).wb with :
	//		- va, vb as the linear velocities of body a and b
	//		- wa, wb as the angular velocities of body a and b.
	//
	//Jacobian : J = [ n (ra x n) -n -(rb x n)]
	//
	//K Matrix : K = J * M-1 * JT = ma-1 + mb-1 + ((ra x n)Ia-1 x ra + (rb x n)Ib-1 x rb).n with : 
	//		- ma-1, mb-1 as the inverse mass matrices for body a and b.
	//		- Ia-1, Ib-1 as the inverse world inertia tensor for body a and b.
	//
	//Linear Velocity : 
	//			Va = Va + ma-1 * n * l and Vb = Vb - mb-1 * n * l with :
	//				- l as the lagrangian.
	//
	//Angular Velocity :
	//			wa = wa + Ia-1 * (ra x n)T * l and wb = wb - Ib-1 * (rb x n)T * l with : 
	//				- l as the lagrangian.
	//
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	class SN_ALIGN snContactConstraint : public snIConstraint
	{
	protected:

		//The two bodies who must repsect the constraint.
		snRigidbody* m_bodies[2];

		//Collision normal going from the second body to the first one.
		snVec m_normal;

		//The collision point between the two bodies expressed in world coordinates.
		snVec m_collisionPoint;

		//collision point - center of mass
		snVec m_radius[2];

		//r X N
		snVec m_rCrossN[2];

		//I-1(r X N)T
		snVec m_invI_rCrossN[2];

		//Bias velocity. The value is duplicated in each component.
		snVec m_velocityBias;

		//The mass expressed in the constraint frame of reference. It is equal to 1 / (J * M-1 * JT).
		//The value is duplicated in each component. W = 0
		snVec m_effectiveMass;

		//The accumulated impulse. The value is duplicated in each component.
		snVec m_accumulatedImpulseMagnitude;

		//The two tangent vector along which the friction is applied.
		snVec m_tangent[2];

		//(r X T0)I-1
		snVec m_rCrossT0InvI[2];

		//(r X T1)I-1
		snVec m_rCrossT1InvI[2];

		//The mass expressed in the constraint frame of reference. It is equal to 1 / (J * M-1 * JT).
		snVec m_frictionEffectiveMass[2];

		//The accumulated impulse for the tangent vectors.
		snVec m_frictionAccumulatedImpulse[2];

		//Penetration depth of the two actors.
		float m_penetrationDepth;

		//coefficient used to compute the friction.
		float m_frictionCoefficient;

		//Scene containing this constraints.
		snScene const * m_scene;

	public:
		snContactConstraint();

		virtual ~snContactConstraint();

		//Give to the constraints the basic information it needs.
		void initialize(snRigidbody* const _body1, snRigidbody* const _body2, const snVec& _normal, const snVec& _collisionPoint, float _penetrationDepth,
			snScene const * _scene);

		void prepare(float _dt);

		void resolve();

		//Return an array of the two snRigidbodies making this constraint.
		snRigidbody * const * const getBodies() const;

		//Return the number of bodies in this constraint
		unsigned int getBodiesCount() const;
	};
}

#endif //SN_NON_PENETRATION_CONSTRAINT_H