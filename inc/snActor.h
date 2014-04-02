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

#ifndef SN_ACTOR_H
#define SN_ACTOR_H

#include "snTypes.h"
#include "snMatrix44f.h"
#include "snPhysicMaterial.h"


#include <vector>
#include <string>
using std::string;



namespace Supernova
{
	class snICollider;
	class snColliderBox;

	class SN_ALIGN snActor
	{
	private:

		//name of the actor
		string m_name;

		//mass of the actor
		float m_mass;

		//inverse of the mass
		float m_massInv;
		
		//inertia tensor expressed in local coordinate
		snMatrix44f m_inertiaTensor;

		//Inverse of the inertia tensor in local coordinate
		snMatrix44f m_invInertiaTensor;

		//Inertia tensor expressed in world coordinates.
		snMatrix44f m_worldInertiaTensor;

		//Inverse of the inertia tensor expressed in world coordinates
		snMatrix44f m_invWorldInertiaTensor;

		//list of collider making this actor
		std::vector<snICollider*> m_colliders;

		//Indicates if this actor is kinematic or not
		bool m_isKinematic;

		//Indicates if this actor is static or not. A static actor is checked for collisions against other actor but it is not simulated.
		bool m_isStatic;

		//position
		snVector4f m_x;

		//linear velocity
		snVector4f m_v;

		//orientation quaternion
		snVector4f m_q;

		//angular velocity
		snVector4f m_w;

		//orientation matrix
		snMatrix44f m_R;

		//Inverse of the orientation matrix
		snMatrix44f m_invR;

		//Various parameters to define the behavior of the object (friction, bouncing...)
		snPhysicMaterial m_material;

		//Coefficient used to compute the linear damping force which is equal to -m_linearDamping * m_linearVelocity.
		float m_linearDampingCoeff;

		//Coefficient used to compute the amgular damping torque which is equal to -m_angularDamping * m_angularVelocity.
		float m_angularDampingCoeff;

	public:

		//Constructor. Actors should be created using snScene::createActor. If you create an actor yourself it is your responsability
		//to delete it.
		snActor();

		//Constructor to let you set if the actor is static or not. Actors should be created using snScene::createActor.
		explicit snActor(bool _isStatic);

		virtual ~snActor();

		//Has to be called once all the parameters and colliders are iniialized.
		bool initialize();

		string getName() const;

		void setName(const string& _name);

		float getMass() const;

		void setMass(float);

		float getInvMass()const;

		const snMatrix44f& getInertiaTensor() const;

		const snMatrix44f getInvInertiaTensor() const;

		void computeWInertiaTensor();

		void computeWInvInertiaTensor();

		const snMatrix44f& getWorldInertia()const;

		const snMatrix44f& getInvWorldInertia()const;

		bool getIsKinematic()const;

		void setIsKinematic(bool);

		//Return if th current actor is static.
		bool getIsStatic() const;

		snVector4f getPosition()const;

		void setPosition(const snVector4f&);

		snVector4f getLinearVelocity()const;

		void setLinearVelocity(const snVector4f& _v);

		snVector4f getAngularVelocity() const;

		void setAngularVelocity(const snVector4f& _w);

		snVector4f getOrientationQuaternion() const;

		void setOrientationQuaternion(const snVector4f& _q);

		const snMatrix44f& getOrientationMatrix() const;

		void setOrientationMatrix(const snMatrix44f& _R);

		const snMatrix44f& getInverseOrientationMatrix() const;

		void setLinearDampingCoeff(float _linearDamping);

		void setAngularDampingCoeff(float _angularDamping);

		float getLinearDampingCoeff() const;

		float getAngularDampingCoeff() const;

		//Create a collider box and add it to the actor. Return its id.
		void createColliderBox(snColliderBox** _box, int& _colliderId);

		//Delete the collider.
		void deleteCollider(unsigned int _colliderId);

		//Add collider to the actor
		int attachCollider(snICollider* _collider);

		//remove the collider from the actor
		void removeCollider(unsigned int _colliderId);

		//Return the collider identified by its id. Return 0 if the collider can't be found.
		snICollider* getCollider(unsigned int _colliderId);

		std::vector<snICollider*>& getColliders();

		bool updateCollider(const snVector4f& x, const snMatrix44f& _R);

		//Return the physic material.
		snPhysicMaterial& getPhysicMaterial();

		void* operator new(size_t _count);

		void operator delete(void* _p);

	private:

		void computeInertiaTensor();
	};
}
#endif //ACTOR_H