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

#ifndef SN_RIGIDBODY_H
#define SN_RIGIDBODY_H

#include <string>
using std::string;

#include <vector>
using std::vector;

#include "snObject.h"
#include "snVec.h"
#include "snPhysicMaterial.h"
#include "snAABB.h"
#include "snTransform.h"
#include "snCollisionFlag.h"

namespace Supernova
{
	class snICollider;

	class snRigidbody;

	//Define the signature of a callback function called when a collision is detected
	typedef void(*OnCollisionCallback)(snRigidbody* const, snRigidbody* const);

	class snRigidbody : public snObject
	{
	private:
	
#pragma region Protected Variables

		//name of the actor
		string m_name;

		//Flag to indicate if this actor is active or not.
		bool m_isActive;

		//list of collider defining collision geometry
		vector<snICollider*> m_colliders;

		//Position of the center of mass expressed in local coordinate system.
		snVec m_centerOfMass;

		//Position of the center of mass expressed in world coordinate system.
		snVec m_worldCenterOfMass;

		//Position, orientation and scaling of the current actor.
		snTransform m_transform;

		//Defines the behavior of the object : friction and restitution
		snPhysicMaterial m_material;

		//AABB bounding volume containing all colliders.
		snAABB m_boundingVolume;

		//Represent the maximum depth another actor can penetrate into the current actor.
		float m_skinDepth;

		//Flags defining behavior of the actor when a collision is detected
		unsigned char m_collisionFlag;

		//Function to call when a collision is detected on this actor. The collision flag CF_CONTACT_CALLBACK needs to be set in order
		// to the callback to be called.
		OnCollisionCallback m_collisionCallback;

		//The total mass of the actor
		float m_mass;

		//Inverse of the total mass
		float m_invMass;

		//Linear damping coefficient
		float m_linearDamping;

		//Angular damping coefficient
		float m_angularDamping;

		//Inverse of the inertia tensor expressed in local coordiantes
		snMatrix44f m_invInertia;

		//Inverse of the inertia tensor expressed in world coordinates.
		snMatrix44f m_invWorldInertia;

		//Linear velocity
		snVec m_v;

		//angular velocity
		snVec m_w;

		//Amount of time the rigidbody has been sleeping while in active state.
		float m_preSleepingTime;

		//Flag ot indicate if the rigidbody is awake or sleeping. A sleeping body is not checked for collisions against
		//other sleeping bodies and is not simulated. The contact constraint between two sleeping bodies are cached
		//in the contact constraint manager.
		bool m_isAwake;

		//Flag to indicate if this actor is kinematic or not.
		bool m_isKinematic;

#pragma endregion

	public:

		snRigidbody();

		virtual ~snRigidbody();

		//Add a collider to the actor
		void addCollider(snICollider* _collider);

		bool isStatic() const;

		bool isKinematic() const;

		bool isDynamic() const;

#pragma region Getter

		//Get the name of the actor
		string getName() const;

		//Return if the actor is active or not.
		bool getIsActive() const;

		//return the list of colliders
		vector<snICollider*>& getColliders();

		//Return the mass. It returns 0 in case of a static or kinematic body
		float getMass() const;

		//Return the inverse of the mass
		float getInvMass() const;

		//Return the inverse of the inertia expressed in world coordinate
		const snMatrix44f& getInvWorldInertia() const;

		//Return the center of mass expressed in local coordinate system.
		snVec getCenterOfMass() const;

		//Return the center of mass expressed in world coordinate system.
		snVec getWorldCenterOfMass() const;

		//Return the position of the actor.
		snVec getPosition() const;

		//Return the transform of the current actor.
		const snTransform& getTransform() const;

		//Return the transform of the current rigidbody.
		snTransform& getTransform();

		//Return the linear velocity
		snVec getLinearVelocity() const;

		//Return the angular velocity
		snVec getAngularVelocity() const;

		//Return the orientation represented as a quaternion
		snVec getOrientationQuaternion();

		//Get the maximum depth another actor can penetrate into this actor
		float getSkinDepth() const;

		//Return a pointer to the AABB.
		const snAABB* getBoundingVolume() const;

		//Return the physic material.
		snPhysicMaterial& getPhysicMaterial();

		//Return the linear damping.
		float getLinearDampingCoeff() const;

		//Return the angular damping
		float getAngularDampingCoeff() const;

		//Get if the rigidbody is sleeping.
		bool isAwake() const;

#pragma endregion

#pragma region Setter

		//Set the name of the actor
		void setName(const string& _name);

		//Set if the actor is active or not.
		void setIsActive(bool _isActive);

		//Set the maximum depth another actor can penetrate into this actor
		void setSkinDepth(float _skinDepth);

		//Set the linear velocity
		void setLinearVelocity(const snVec& _linearVelocity);

		//Set the angular velocity
		void setAngularVelocity(const snVec& _angularVelocity);

		//Set the position of the actor
		void setPosition(const snVec& _position);

		//Set the orientation of the actor
		void setOrientation(const snVec& _orientation);

		//Set the collision callback
		void setOnCollisionCallback(OnCollisionCallback _callback);

		//Set if the actor is kinematic
		void setIsKinematic(bool _isKinematic);

		//Set the linear damping coefficient
		void setLinearDampingCoeff(float _linearDamping);

		//Set the angular damping coefficient
		void setAngularDampingCoeff(float _angularDamping);

		//Set the position and orientation of a kinematic actor.
		void setKinematicTransform(const snVec& _position, const snVec& _orientation);

		//Set the awake flag of the rigidbody.
		void setAwake(bool _isAwake);		

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

		//Initialize the rigidbody as a dynamic body. It has to be called once and must be called after all the parameters of
		//the actor and its colliders are set.
		void initializeDynamic();

		//Initialize the rigidbody as  static body. It has to be called once after all the parameter of the body are set.
		// _p : position of the body.
		// _q : orientation of the body.
		void initializeStatic(const snVec& _p, const snVec& _q);

		void onCollision(snRigidbody* const _other);

		//Check if we can make a collision detection test based on the type of actors
		static bool isCollisionDetectionEnabled(const snRigidbody* const _a, const snRigidbody* const _b);

		//Compute the angular speed of the actor
		float computeAngularSpeed() const;

		//Compute the linear speed of the actor
		float computeLinearSpeed() const;

		//Compute the inverse of the inertia tensor expressed in world coordinates
		void computeInvWorldInertia();

		//Compute the center of mass expressed in world coordinate.
		void computeWorldCenterOfMass();

		//Update the colliders based on the current position and orientation
		void updateCollidersAndAABB();

		//Set the mass to the actor and update its inertia
		void updateMassAndInertia(float _mass);

		//Depending of the current velocity of the body, turn into sleeping state.
		// _dt : the time step of the physics engine.
		// _period : the time to wait before going to sleep in seconds.
		void updateSleepingState(float _dt, float _period);
	
	protected :
		//Compute the bounding volume based on the colliders
		void computeBoundingVolume();
	};
}

#endif //ifndef SN_RIGIDBODY_H