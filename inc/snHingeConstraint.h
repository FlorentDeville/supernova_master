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

#ifndef SN_HINGE_CONSTRAINT_H
#define SN_HINGE_CONSTRAINT_H

#include "snIConstraint.h"
#include "snMatrix44f.h"

namespace Supernova
{
	class snRigidbody;

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	//Represent a constraint for a single body. It only lets the bodies to rotate about an axis. It is a combination of a 
	//translation constraint and two rotional constraints.
	//There is no baumgarte stabilization as there is no drfting, or at least I didn't see any in the test case.
	//
	// TRANSLATIONAL CONSTRAINT
	//Position Constraint: C = x + r where x is the position of the actor, r the vector from the the actor to the anchor point.
	//
	//Velocity Constraint : dC/dt = v + Rw with :
	//		- v as the linear velocities of the body
	//		- w as the angular velocities of the body
	//		- R as the skew symmetric matrix of r. Rw = w x r
	//
	//Jacobian : J = [ E R ]
	//
	//K Matrix : K = M-1 + R * I-1 * RT with : 
	//		- M-1 as the inverse mass matrix.
	//		- I-1 as the inverse world inertia tensor.
	//		- RT as the transpose of R.
	//
	//Linear Velocity : 
	//			V = V + M-1 * l with :
	//				- V as the linear velocity.
	//				- l as the lagrangian.
	//
	//Angular Velocity :
	//			w = w + I-1 * RT * l:
	//
	// FIRST ROTATIONAL CONSTRAINT
	//Position Constraint: C = a.b where a is the axis of rotation (the inital one) and b one of the two tangent vectors of 
	// the rotation axis as seen by the body. This constraint means that a and b must stay orthogonal.
	//
	//Velocity Constraint : dC/dt = (b x a).w
	//
	//Jacobian : J = [ 0 (b x a)]
	//
	//K Matrix : (b x a) * I-1 * (b x a)T
	//
	//Linear Velocity : this constraint does not change linear velocity
	//
	//Angular Velocity : 
	//				w = w + I-1 * (b x a)T * l
	//
	// SECOND ROTATIONAL CONSTRAINT
	//Position Constraint : C = a.c where c is the second tangent vectors.
	//
	//Velocity Constraint : dC/dt = (c x a).w
	//
	//Jacobian : J = [ 0 (c x a)]
	//
	//K Matrix : (c x a) * I-1 * (c x a)T
	//
	//Linear Velocity : this constraint does not change linear velocity
	//
	//Angular Velocity : 
	//				w = w + I-1 * (c x a)T * l
	//
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	class snHingeConstraint : public snIConstraint
	{
	private:
		//The actor constrained
		snRigidbody* m_actor;

		//The rotation axis expressed in world coordinate
		snVec m_axis;

		//The rotation axis expressed in the actor's local coordinates.
		snVec m_localAxis;

		//Than anchor point through which goes the rotation axis
		snVec m_anchor;

		//Vector from the actor to the anchor.
		snVec m_radius;

		//The skew symmetric matrix of the radius
		snMatrix44f m_r;

		//Mass matrix for the translation part of the constraint
		snMatrix44f m_KTrans;

		//I-1 * rT
		snMatrix44f m_invIRT;

		//b x a and c x a
		snVec m_bCrossA, m_cCrossA;

		//K matrices for the first and second rotational constraints
		snVec m_KRot1, m_KRot2;

		//I-1 * (b x a) and I-1 * (c x a)
		snVec m_invIBCrossA, m_invICCrossA;

		//Bias value for the translational constraint and the two rotationals constraints.
		//snVec m_biasTrans, m_biasRot1, m_biasRot2;

		//float m_initialDistanceToAnchor;

	public:

		snHingeConstraint(snRigidbody* _actor, const snVec& _axis, const snVec& _anchor);

		~snHingeConstraint();

		void prepare(float _dt);
		
		void resolve();
	};
}

#endif //ifndef SN_HINGE_CONSTRAINT_H