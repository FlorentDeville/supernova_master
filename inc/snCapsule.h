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
#ifndef SN_CAPSULE_H
#define SN_CAPSULE_H

#include "snICollider.h"
#include "snIGJKCollider.h"

#include "snVec.h"
namespace Supernova
{
	class SN_ALIGN snCapsule :
		public snICollider,
		public snIGJKCollider
	{
	private:
		//First end point of the capsule
		snVec m_a;

		//Second end point of the capsule
		snVec m_b;

		//Radius of the capsule
		float m_radius;

	public:
		//Construct a capsule where the axis goes along the y axis and the origin is in the middle of the segment.
		// It means the first endpoint a is (0, -_length * 0.5, 0, 1) and the second endpoint 
		// b is (0, length * 0.5, 0, 1).
		snCapsule(float _length, float _radius);

		//Construct a capsule using _a and _b respectively as first and second endpoint
		snCapsule(const snVec& _a, const snVec& _b, float _radius);

		//Return the first endpoint making the capsule.
		snVec getFirstEndPoint() const;

		//Return the second end point making the capsule
		snVec getSecondEndPoint() const;

		//Return the radius of the capsule
		float getRadius() const;

		//////////////////////////////////////////////////////////////////////
		// snICollider Interface											//
		//////////////////////////////////////////////////////////////////////

		//Initialize the collider. Should be called once all the parameters of the collider are set.
		void initialize();

		//Move the collider using the given transform matrix.
		void setTransform(const snMatrix44f& _transform);

		//Compute the inertia tensor in the local frame.
		void computeLocalInertiaTensor(float _mass, snMatrix44f& _inertiaTensor) const;

		//Compute the bounding volume for this collider
		void computeAABB(snAABB * const _boundingVolume) const;

		//////////////////////////////////////////////////////////////////////
		// GJK Interface													//
		//////////////////////////////////////////////////////////////////////

		//Return the farthest point in the direction provided by the _direction vector. It does not need to be normalized.
		snVec support(const snVec& _direction, float& _distance) const;

		//Return any point making the collider
		snVec anyPoint() const;
	};
}
#endif //ifndef SN_CAPSULE_H