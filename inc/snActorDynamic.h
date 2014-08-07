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
#ifndef SN_ACTOR_DYNAMIC_H
#define SN_ACTOR_DYNAMIC_H

#include "snIActor.h"

namespace Supernova
{
	class SN_ALIGN snActorDynamic : public snIActor
	{
	private:

		//The total mass of the actor
		float m_mass;

		//Inverse of the total mass
		float m_invMass;

		//Inverse of the inertia tensor expressed in local coordiantes
		snMatrix44f m_invInertia;

		//Inverse of the inertia tensor expressed in world coordinates.
		snMatrix44f m_invWorldInertia;

		//Flag to indicate if this actor is kinematic or not.
		bool m_isKinematic;

		//Linear velocity
		snVec m_v;

		//angular velocity
		snVec m_w;

		//Linear damping coefficient
		float m_linearDamping;

		//Angular damping coefficient
		float m_angularDamping;

	public:

		snActorDynamic();
		~snActorDynamic();

#pragma region Virtual Getter

		//Return the mass
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

#pragma region Getter

		//Return the linear damping coefficient
		float getLinearDampingCoeff() const;

		//Return the angular damping coefficient
		float getAngularDampingCoeff() const;

#pragma endregion

#pragma region Virtual Setter

		//Set the linear velocity
		void setLinearVelocity(const snVec& _linearVelocity);

		//Set the angular velocity
		void setAngularVelocity(const snVec& _angularVelocity);

#pragma endregion

#pragma region Setter

		//Set the linear damping coefficient
		void setLinearDampingCoeff(float _linearDamping);

		//Set the angular damping coefficient
		void setAngularDampingCoeff(float _angularDamping);

		//Set the position of the actor
		void setPosition(const snVec& _position);

		//Set the orientation of the actor
		void setOrientation(const snVec& _orientation);

		//Set if the actor is kinematic
		void setIsKinematic(bool _isKinematic);

		//Set the position of a kinematic actor
		void setKinematicPosition(const snVec& _position);

		//Set the position and orientation of a kinematic actor.
		void setKinematicTransform(const snVec& _position, const snVec& _orientation);

#pragma endregion

		//Set the mass to the actor and update its inertia
		void updateMassAndInertia(float _mass);

		//Initialize the actor so it is ready to be used in the scene. It has to be called and must be called after all the parameters of
		// the actor and its colliders are set.
		void initialize();

		//Compute the angular speed of the actor
		float computeAngularSpeed() const;

		//Compute the linear speed of the actor
		float computeLinearSpeed() const;

	private:

		//Compute the inverse of the inertia tensor expressed in world coordinates
		void computeInvWorldInertia();

		//Compute the center of mass expressed in world coordinate.
		void computeWorldCenterOfMass();

		//Update the colliders based on the current position and orientation
		void updateCollidersAndAABB();

		//Move the actor forward in time using _dt as a time step.
		//_linearSpeed2Limit and _angularSpeed2Limit are the squared speed below which the velocities will be set to 0.
		void integrate(float _dt, float _linearSpeed2Limit, float _angularSpeed2Limit);
	};
}
#endif //ifndef SN_ACTOR_DYNAMIC_H