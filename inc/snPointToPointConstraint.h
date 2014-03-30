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
#include "snActor.h"

namespace Supernova
{
	//Represent a constraint between two bodies. It forces the two bodies to remain at the same distance.
	class SN_ALIGN snPointToPointConstraint : public snIConstraint
	{
	private:
		//The two bodies which must respect the constraint.
		snActor* m_bodies[2];

		//Offset to the center of mass of the bodies. They must be expressed in local coordinates of the bodies.
		snVector4f m_pivot[2];

		//Offset to the center of mass expressed in world coordinates.
		snVector4f m_worldPivot[2];

		//Vector from the constraint point to the center of mass.
		snVector4f m_offset[2];

		//Skew matrix used to compute the cross product r X w
		snMatrix44f m_R[2];

		//I-1 * R
		snMatrix44f m_InvIR[2];

		////Constraint distance between the two bodies.
		//float m_distance;

		////Normalized vector from the second body to the first one
		//snVector4f m_normalizeddp;

		////radius X normalized dp
		//snVector4f m_rCrossDirection[2];

		////(radius X normalized dp) * I-1
		//snVector4f m_rCrossUInvI[2];

		//Inverse of the mass expressed in the constraint frame of reference. It is equal to 1 / (J * M-1 * JT).
		snMatrix44f m_invEffectiveMass;

	public:

		snPointToPointConstraint(snActor* const _bodyA, const snVector4f& _pivotA, snActor* const _bodyB, const snVector4f& _pivotB);

		virtual ~snPointToPointConstraint();

		void prepare();

		void resolve();
	};
}

#endif //ifndef SN_POINT_TO_POINT_CONSTRAINT_H