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

#ifndef SN_ACTOR_STATIC_H
#define SN_ACTOR_STATIC_H

#include "snIActor.h"

namespace Supernova
{
	class SN_ALIGN snActorStatic : public snIActor
	{
	public:
		snActorStatic(snObjectId _id, const snVec& _position);

		snActorStatic(snObjectId _id, const snVec& _position, const snVec& _orientation);

		~snActorStatic();

#pragma region Virtual Getter

		//Return 0
		float getMass() const;

		//Return the inverse of the mass
		float getInvMass() const;

		//Return the inverse of the inertia expressed in world coordinate
		const snMatrix44f& getInvWorldInertia() const;

		//Return the linear velocity
		snVec getLinearVelocity() const;

		//Return the angular velocity
		snVec getAngularVelocity() const;

#pragma endregion

#pragma region Virtual Setter

		//Set the linear velocity
		void setLinearVelocity(const snVec& _linearVelocity);

		//Set the angular velocity
		void setAngularVelocity(const snVec& _angularVelocity);

#pragma endregion

		//Move the actor forward in time using _dt as a time step.
		//_linearSpeed2Limit and _angularSpeed2Limit are the squared speed below which the velocities will be set to 0.
		//A static actor cannot move so this function doesn't do anyhthing.
		void integrate(float _dt, float _linearSpeed2Limit, float _angularSpeed2Limit);

		void initialize();

	private:

		//Initialize the actor with default values.
		//This function must be called in the constructor.
		void init(const snVec& _position, const snVec& _orientation);
	};
}
#endif //ifndef SN_ACTOR_STATIC_H