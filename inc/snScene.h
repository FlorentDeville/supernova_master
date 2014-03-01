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

#ifndef SN_SCENE_H
#define SN_SCENE_H

#include <vector>
using std::vector;
using std::string;

#include "AlignmentAllocator.h"
#include "snSequentialImpulse.h"
#include "snTypes.h"
#include "snGJK.h"

#ifdef _DEBUG
namespace Devil
{
	class EntityCollisionPoint;
}
#endif

namespace Supernova
{
	class snActor;
	class CollisionResult;

	//The physic scene to simulate. HUGE WIP.
	class SN_ALIGN snScene
	{
	private:

		//Singleton
		//static snScene* m_instance;

		//Name of the scene.
		string m_name;

		//List of actors in the scene.
		vector<snActor*> m_actors;

		//List of contacts found during the latest collision detection steps
		snContactPointVector m_contactsPoints;

		//Solver for contact points using sequential impulse.
		snSequentialImpulse m_sequentialImpulseSolver;

		snGJK m_GJK;

		//Coefficient of the penetration. Used to compute the bias velocity.
		float m_beta;

		//Maximum penetration authorized between two actors.
		float m_maxSlop;

		//Vector representing the gravity in the scene.
		snVector4f m_gravity;

		//Threshold under wich the linear speed is ignored and becomes 0.
		float m_linearSquaredSpeedThreshold;

		//Threshold under wich the angular speed is ignored and becomes 0.
		float m_angularSquaredSpeedThreshold;

	public:
		//Constructor. Scenes should be created using snFactory::createScene. If you create a scene yourself it is your responsability
		//to delete it.
		snScene();

		//Destructor
		virtual ~snScene();

		//Create a new actor and add it to the scene.
		void createActor(snActor** _newActor, int& _actorId);

		//Delete an actor
		void deleteActor(int _actorId);

		//Add the actor to the scene.
		int attachActor(snActor* _actor);

		//Remove the actor from the scene
		void removeActor(int _actorId);

		//Get an actor from its id. Returns 0 if the actor can't be found.
		snActor* getActor(int _actorId);

		//Delete all actors from the physics scene.
		void clearScene();

		//Simulate the scene for a given elapsed time step.
		void update(float _dt);

		//Return the list of contact points for the last iteration of the engine.
		const snContactPointVector& getContactsPoints() const;

		//Return a reference to the sequential impulse solver.
		snSequentialImpulse& getSolver();

		//Set the coefficient of penetration.
		void setBeta(float _beta);

		//Set the maximum authorized penetration between two actors.
		void setMaxSlop(float _maxSlop);

		//Set the gravity to apply in the scene.
		void setGravity(const snVector4f& _gravity);

		//Set the threshold under wich the linear velocity is set to 0
		void setLinearSquaredSpeedThreshold(float _linearSquaredSpeedThreshold);

		//Set the threshold under wich the angular velocity is set to 0
		void setAngularSquaredSpeedThreshold(float _angularSquaredSpeedThreshold);

		//Overridden new operator to create scene with correct alignement.
		void* operator new(size_t _count);

		//Overridden delete operator to delete using the correct alignement.
		void operator delete(void* _p);

	private:
		//Check collisions and make a list of contact points.
		void getContactPoints(snContactPointVector& _contacts, float _dt);

		//Apply forces and compute linear and angular velocities
		void applyForces(float _dt);

		//Update linear and angular position of all actors.
		void updatePosition(float _dt);

		//Resolve all contact points using sequential impulse algorithm.
		void sequentialImpulseSIMD(snContactPointVector& _contacts) const;
	};
}

#endif //SN_SCENE_H