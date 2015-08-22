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

#ifndef SN_POINT_TO_POINT_CONSTRAINT_H
#define SN_POINT_TO_POINT_CONSTRAINT_H

#include "snIConstraint.h"
#include "snMatrix44f.h"

namespace Supernova
{
	class snRigidbody;
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	//Represent a constraint between two bodies. It only lets the bodies to rotate about a common point.
	//
	//Position Constraint: C = pa - pb = 0 where pa and pb are the pivot point expressed in world coordinate for body a and b.
	//It means those two points as to be the same everytime.
	//It is otherwise expressed : C = xa + Ra * ra - xb - Rb * rb with : 
	//		- xa, xb as the position of the actors
	//		- Ra, Rb as the orientation matrix of the actors
	//		- ra, rb as the vector from the actor position to the pivot point (in world coordinate). It is called m_offset in this class.
	//
	//Velocity Constraint : dC/dt = va + Rsa * wa - xb - Rsb * wb with :
	//		- va, vb as the linear velocities of body a and b
	//		- Rsa, Rsb as the skew symmetric matrices used to compute a cross product. Rsa * wa = wa X ra
	//		- wa, wb as the angular velocities of body a and b.
	//
	//Jacobian : J = [ 1 Rsa -1 -Rsb]
	//
	//K Matrix : K = Ma-1 + Rsa * Ia-1 * RsaT + Mb-1 + Rsb * Ib-1 * RsbT with : 
	//		- Ma-1, Mb-1 as the inverse mass matrices for body a and b.
	//		- Ia-1, Ib-1 as the inverse world inertia tensor for body a and b.
	//		- Rsa, Rsb as the skew symmetric matrices used to compute a cross product. They are the same as in the velocity constraint.
	//		- RsaT, RsbT as the transpose of respectively Rsa and Rsb.
	//
	//Linear Velocity : 
	//			Va = Va + Ma-1 * l and Vb = Vb - Mb-1 * l with :
	//				- Va, Vb as the linear velocities of actor a and b.
	//				- Ma-1, Mb-1 as the inverse mass matrices for actor a and b.
	//				- l as the lagrangian.
	//
	//Angular Velocity :
	//			wa = wa + Ia-1 * RsaT * l and wb = wb - Ib-1 * RsbT with : 
	//				- wa, wb as the angular velocities ofactor a and b.
	//				- Ia-1, Ib-1 as the inverse world inertia tensors for actor a and b.
	//				- RsaT, RsbT as the transpose of respectively Rsa and Rsb.
	//				- l as the lagrangian.
	//
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	class SN_ALIGN snPointToPointConstraint : public snIConstraint
	{
	private:
		//The two bodies which must respect the constraint.
		snRigidbody* m_actors[2];

		//Offset to the center of mass of the bodies. They must be expressed in local coordinates of the bodies.
		snVec m_pivot[2];

		//Offset to the center of mass expressed in world coordinates.
		snVec m_worldPivot[2];

		//Vector from the constraint point to the center of mass.
		snVec m_offset[2];

		//Skew matrix used to compute the cross product r X w
		snMatrix44f m_R[2];

		//I-1 * RT
		snMatrix44f m_InvIRT[2];

		//Inverse of the mass expressed in the constraint frame of reference. It is equal to 1 / (J * M-1 * JT).
		snMatrix44f m_invEffectiveMass;

		//Baumgarte stabilization
		snVec m_bias;

	public:
		//Constructor for a Point To Point Constraint.
		//_bodyA and _bodyB are const pointers to snActor objects.
		//_pivotA is the pivot point around which _bodyA can rotate. It is expressed in _bodyA local coordinate.
		//_pivotB is the pivot point around which _bodyB can rotate. It is expressed in _bodyB local coordinate.
		snPointToPointConstraint(snRigidbody* const _actorA, const snVec& _pivotA, snRigidbody* const _actorB, const snVec& _pivotB);

		virtual ~snPointToPointConstraint();

		void prepare(float _dt);

		void resolve();

		//Return an array of two pointers to snActors
		snRigidbody const * const * getActors() const;

		//Return an array of vectors containing the pivot points in world coordinate for each actor of the constraint.
		snVec const * getWPivot() const;

		//Return an array of vectors constaining the offset of the pivot. The vector goes from the actor origin to the pivot.
		snVec const* getOffset() const;

		//Return an array of the two snRigidbodies making this constraint.
		snRigidbody * const * const getBodies() const;

		//Return the number of bodies in this constraint
		unsigned int getBodiesCount() const;
	};
}

#endif //ifndef SN_POINT_TO_POINT_CONSTRAINT_H