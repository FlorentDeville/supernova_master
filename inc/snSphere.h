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

#ifndef SN_SPHERE_H
#define SN_SPHERE_H

#include "snICollider.h"

namespace Supernova
{
	class snSphere : 
		public snICollider
	{
	private:

		//Get the position of the center of the sphere
		snVec m_center;

		/*Sphere radius.*/
		float m_radius;

	public:
		snSphere(float _radius);

		snVec getCenter() const;

		float getRadius() const;

		//////////////////////////////////////////////////////////////////////
		// snICollider Interface											//
		//////////////////////////////////////////////////////////////////////
		void updateFromTransform();

		void initialize();

		void computeLocalInertiaTensor(float _mass, snMatrix44f& _inertiaTensor) const;

		void computeAABB(snAABB * const _boundingVolume) const;

		//////////////////////////////////////////////////////////////////////
		// Interface for GJK algorithm										//
		//////////////////////////////////////////////////////////////////////

		//Return the farthest point in the direction provided as parameter.
		snVec support(const snVec& _direction) const;

		//Return any point making the collider
		snVec anyPoint() const;
	};
}

#endif //ifndef SN_SPHERE_H