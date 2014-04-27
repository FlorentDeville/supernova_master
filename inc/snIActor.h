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

#ifndef SN_I_ACTOR_H
#define SN_I_ACTOR_H

#include <string>
using std::string;

#include <vector>
using std::vector;

#include "snMatrix44f.h"
#include "snPhysicMaterial.h"
#include "snAABB.h"
#include "snCollisionFlag.h"

namespace Supernova
{
	class snIActor;
	//Define the signature of a callback function called when a collision is detected
	typedef void(*OnCollisionCallback)(snIActor* const, snIActor* const);

	//Forward declarations
	class snICollider;

	enum snActorType : unsigned char
	{
		snActorTypeStatic,
		snActorTypeKinematic,
		snActorTypeDynamic
	};

	//Interface for static and dynamic actor
	class snIActor
	{
	protected:
	
#pragma region Protected Variables

		//name of the actor
		string m_name;

		//list of collider defining collision geometry
		vector<snICollider*> m_colliders;

		//position of the actor
		snVector4f m_x;

		//orientation represented as a quaternion
		snVector4f m_q;

		//orientation represented as a matrix
		snMatrix44f m_R;

		//Inverse of the orientation matrix
		snMatrix44f m_invR;

		//Defines the behavior of the object : friction and restitution
		snPhysicMaterial m_material;

		//AABB bounding volume containing all colliders.
		snAABB m_boundingVolume;

		//Represent the maximum depth another actor can penetrate into the current actor.
		float m_skinDepth;

		//Define the type of actor
		snActorType m_typeOfActor;

		//Flags defining behavior of the actor when a collision is detected
		unsigned char m_collisionFlag;

		//Function to call when a collision is detected on this actor. The collision flag CF_CONTACT_CALLBACK needs to be set in order
		// to the callback to be called.
		OnCollisionCallback m_collisionCallback;

#pragma endregion

	public:

		virtual ~snIActor();

		//Add a collider to the actor
		void addCollider(snICollider* _collider);

#pragma region Getter

		//Get the name of the actor
		string getName() const;

		//return the list of colliders
		vector<snICollider*>& getColliders();

		//Return the mass. It returns 0 in case of a static or kinematic body
		virtual float getMass() const = 0;

		//Return the inverse of the mass
		virtual float getInvMass() const = 0;

		//Return the inverse of the inertia expressed in world coordinate
		virtual const snMatrix44f& getInvWorldInertia()const = 0;

		//Return the position of the actor.
		snVector4f getPosition() const;

		//Return the linear velocity
		virtual snVector4f getLinearVelocity() const = 0;

		//Return the angular velocity
		virtual snVector4f getAngularVelocity() const = 0;

		//Return the orientation represented as a quaternion
		snVector4f getOrientationQuaternion();

		//Return the orientation represented as a matrix.
		const snMatrix44f& getOrientationMatrix() const;

		//Return the inverse of orientation matrix.
		const snMatrix44f& getInverseOrientationMatrix() const;

		//Get the maximum depth another actor can penetrate into this actor
		float getSkinDepth() const;

		//Return a pointer to the AABB.
		const snAABB* getBoundingVolume() const;

		//Return the physic material.
		snPhysicMaterial& getPhysicMaterial();

		//Return the type of actor
		snActorType getActorType() const;
#pragma endregion

#pragma region Setter

		//Set the name of the actor
		void setName(const string& _name);

		//Set the maximum depth another actor can penetrate into this actor
		void setSkinDepth(float _skinDepth);

		//Set the linear velocity
		virtual void setLinearVelocity(const snVector4f& _linearVelocity) = 0;

		//Set the angular velocity
		virtual void setAngularVelocity(const snVector4f& _angularVelocity) = 0;

		//Set the collision callback
		void setOnCollisionCallback(OnCollisionCallback _callback);

#pragma endregion

#pragma region Allocation

		//Allocate an actor with the correct alignement
		void* operator new(size_t _count);

		//Free the memory allocated for an actor
		void operator delete(void* _p);

#pragma endregion

#pragma region Collision Flags

		//Add a collision flag to the actor
		void addCollisionFlag(snCollisionFlag _flag);

		//Remove the collision flag of the actor
		void removeCollisionFlag(snCollisionFlag _flag);

		//Set the collision flag
		void setCollisionFlag(snCollisionFlag _flag);

		//Return the collision flag
		unsigned char getCollisionFlag();

		//Check if a collision flag is enabled.
		bool isEnabledCollisionFlag(snCollisionFlag _flag);

#pragma endregion

		//Move the actor forward in time using _dt as a time step.
		//_linearSpeed2Limit and _angularSpeed2Limit are the squared speed below which the velocities will be set to 0.
		virtual void integrate(float _dt, float _linearSpeed2Limit, float _angularSpeed2Limit) = 0;

		//Initialize the actor so it is ready to be used in the scene. It has to be called and must be called after all the parameters of
		// the actor and its colliders are set.
		virtual void initialize() = 0;

		void OnCollision(snIActor* const _other);

	protected :
		//Compute the bounding volume based on the colliders
		void computeBoundingVolume();

	};
}
#endif //ifndef SN_I_ACTOR_H