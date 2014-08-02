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

#include <list>
using std::list;

#include "AlignmentAllocator.h"
#include "snObject.h"
#include "snTypes.h"
#include "snGJK.h"
#include "snCollision.h"
#include "snContactConstraintManager.h"
#include "snCollisionMode.h"
#include "snSweepManager.h"
#include "snActorPairManager.h"
#include "snCollisionDispatcher.h"

#ifdef _DEBUG
namespace Devil
{
	class EntityCollisionPoint;
}
#endif

namespace Supernova
{
	class snIActor;
	class snActorStatic;
	class snActorDynamic;
	class CollisionResult;
	class snIConstraint;
	class snPointToPointConstraint;
	class snFixedConstraint;
	class snHingeConstraint;

	//The physic scene to simulate. HUGE WIP.
	class SN_ALIGN snScene : public snObject
	{
	private:

		//Name of the scene.
		string m_name;

		//List of actors in the scene.
		vector<snIActor*> m_actors;

		//List of constraints in the scene.
		vector<snIConstraint*> m_constraints;

		//List of constraints created by the collision detection system.
		snContactConstraintManager m_contactConstraintManager;

		//The list of collision points gathered during the previous update of the scene.
		snVecVector m_collisionPoints;

		//Provide access to the collision query functions.
		static snCollision m_collisionService;

		snGJK m_GJK;

		//Vector representing the gravity in the scene.
		snVec m_gravity;

		//Threshold under wich the linear speed is ignored and becomes 0.
		float m_linearSquaredSpeedThreshold;

		//Threshold under wich the angular speed is ignored and becomes 0.
		float m_angularSquaredSpeedThreshold;

		//Number of iteration to execute to solve the constraints.
		int m_solverIterationCount;

		//Manager handling the sweep and prune broad phase algorithm
		snSweepManager m_sweepAndPrune;

		//Manager storing pairs of actors representing the possibly coliding set (pcs).
		snActorPairManager m_pcs;

		//Collision dispatcher for multithreaded collision detection
		snCollisionDispatcher m_dispatcher;

		//The type of collision to use
		snCollisionMode m_collisionMode;

		//The amount of correction (baumgarte stabilization) to apply per frame for every contact constraints.
		float m_contactConstraintBeta;

	public:
		//Constructor. Scenes should be created using snFactory::createScene. If you create a scene yourself it is your responsability
		//to delete it.
		snScene(snObjectId _id);

		//Destructor
		virtual ~snScene();

		//Create a new actor and add it to the scene.
		// return : a pointer to the newly created dynamic actor.
		snActorDynamic* createActorDynamic();

		//Create a new static actor and add it to the scene.
		// _position : the position of the static actor.
		// _orientation : the orientation as a quaternion of the static actor.
		// return : a pointer to the newly created static actor.
		snActorStatic* createActorStatic(const snVec& _position, const snVec& _orientation);

		//Delete an actor
		void deleteActor(snIActor* const _actor);

		//Add the actor to the scene.
		int attachActor(snIActor* const _actor);

		//Remove the actor from the scene
		void removeActor(snIActor const * const _actor);

		//Get a constraint from its id.
		snIConstraint* getConstraint(unsigned int _constraintId);

		//Create a distance constraint between two actors and return the id of the constraint
		snPointToPointConstraint* createPointToPointConstraint(snIActor* const _body1, const snVec& _offset1, snIActor* const _body2, const snVec& _offset2);

		snFixedConstraint* createFixedConstraint(snIActor* const _actor, const snVec& _fixedPoint, float _distance);

		snHingeConstraint* createHingeConstraint(snIActor* _actor, const snVec& _axis, const snVec& _anchor);

		//Delete all actors from the physics scene.
		void clearScene();

		//Simulate the scene for a given elapsed time step.
		void update(float _dt);

		//Return the list of collision results for the last iteration of the engine.
		const snVecVector& getCollisionPoints() const;

		//Get the type of collision used.
		snCollisionMode getCollisionMode() const;

		//Return the beta factor for the contact constraints. It is the amount of correction applied per frame.
		float getContactConstraintBeta() const;

		//Set the gravity to apply in the scene.
		void setGravity(const snVec& _gravity);

		//Set the threshold under wich the linear velocity is set to 0
		void setLinearSquaredSpeedThreshold(float _linearSquaredSpeedThreshold);

		//Set the threshold under wich the angular velocity is set to 0
		void setAngularSquaredSpeedThreshold(float _angularSquaredSpeedThreshold);

		//Set the number of iteration to execute to resolve constraints.
		void setSolverIterationCount(int _solverIterationCount);

		//Set the collision mode to use in the scene
		void setCollisionMode(snCollisionMode _collisionMode);

		//Set the beta factor for contact constraints. It is the amount of correction to apply per frame. It should be between 0 and 1 and
		//preferably low. The default value is 0.25.
		void setContactConstraintBeta(float _beta);

		//Overridden new operator to create scene with correct alignement.
		void* operator new(size_t _count);

		//Overridden delete operator to delete using the correct alignement.
		void operator delete(void* _p);

		//Make a sphere cast test
		bool sphereCast(const snVec& _center, float _radius, const snVec& _direction, float _length);

		bool shapeCast(snICollider& _collider, const snTransform& _origin, const snVec& _direction, float _length, float& _distance) const;

	private:
		//Apply forces and compute linear and angular velocities
		void applyForces(float _dt);

		//Update linear and angular position of all actors.
		void updatePosition(float _dt);

		//Resolve the constraint provided in the array and the constraints stored in the scene.
		void resolveAllConstraints();

		//Prepare all the constraints. It must be called before resolving them.
		void prepareConstraints(float _dt);

		//Check collisions using brute force algorithm (no broad phase) in the scene and fill in an array of collision constraints.
		void computeNaiveCollisions();

		//Check collisions using sweep and prune broad phase and create collision constraints.
		void singleThreadedBroadPhase();

		//Broad phase for the multithreaded collision detection model.
		void multiThreadedBroadPhase();

		//Narrow phase for the multithreaded collision detection model.
		void multiThreadedNarrowPhase();

		//Compute tcollision detection between two actors and create the corresponding collision constraints.
		void computeCollisionDetection(snIActor* _a, snIActor* _b);

		//Store a pair of actor into the PCS.
		void storeActorPair(snIActor* _a, snIActor* _b);
	};
}

#endif //SN_SCENE_H