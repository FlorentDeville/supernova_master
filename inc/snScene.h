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

#include "snWorld.h"
#include "AlignmentAllocator.h"
#include "snObject.h"
#include "snTypes.h"
#include "snGJK.h"
#include "snCollision.h"
#include "snContactConstraintManager.h"
#include "snCollisionMode.h"
#include "snSweepManager.h"
#include "snActorPairManager.h"
#include "snFrictionMode.h"

#ifdef _DEBUG
namespace Devil
{
	class EntityCollisionPoint;
}
#endif

namespace Supernova
{
	class snRigidbody;
	class CollisionResult;
	class snIConstraint;
	class snPointToPointConstraint;
	class snFixedConstraint;
	class snHingeConstraint;
	class snRay;

	//The physic scene to simulate. HUGE WIP.
	class SN_ALIGN snScene : public snObject
	{
	private:

		//Name of the scene.
		string m_name;

		//List of actors in the scene.
		vector<snRigidbody*> m_actors;

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

		//The type of collision to use
		snCollisionMode m_collisionMode;

		//The amount of correction (baumgarte stabilization) to apply per frame for every contact constraints.
		float m_contactConstraintBeta;

		//The friction mode to use.
		snFrictionMode m_frictionMode;

		//Time to wait before an body can go into sleeping period.
		float m_sleepingPeriod;

	public:
		//Constructor. Scenes should be created using snWorld::createScene. This constructor should be hidden.
		snScene();

		//Delete everything inside the scene (actors, colliders and constraints) and delete the entry for that scene in the
		//look up table.
		virtual ~snScene();

		//Add a rigidbody to the scene.
		// _actor : handle to a rigidbody.
		void attachActor(snhRigidbody _actor);

		//Remove a rigidbody from the scene.
		// _actor : a handle to the rigidbody to remove.
		void removeActor(snhRigidbody _actor);

		//Delete a rigidbody and remove it from the scene.
		// _actor : a handle to the rigidbody to delete.
		// remarks : all handles to the rigidbody will become invalid.
		void deleteActor(snhRigidbody _actor);

		//Get a constraint from its id.
		snIConstraint* getConstraint(unsigned int _constraintId);

		//Create a distance constraint between two actors and return the id of the constraint
		snPointToPointConstraint* createPointToPointConstraint(snRigidbody* const _body1, const snVec& _offset1, snRigidbody* const _body2, const snVec& _offset2);

		snFixedConstraint* createFixedConstraint(snRigidbody* const _actor, const snVec& _fixedPoint, float _distance);

		snHingeConstraint* createHingeConstraint(snRigidbody* _actor, const snVec& _axis, const snVec& _anchor);

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

		//Return the friction mode to use in the scene.
		snFrictionMode getFrictionMode() const;

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

		//Set the friction mode. It can be changed dynamically.
		void setFrictionMode(snFrictionMode _mode);

		//Set the time to wait before an object can go to sleep in seconds.
		// _dt : the time to wait in seconds.
		void setSleepingPeriod(float _dt);

		//Overridden new operator to create scene with correct alignement.
		void* operator new(size_t _count);

		//Overridden delete operator to delete using the correct alignement.
		void operator delete(void* _p);

		//Set the linear velocity of a kinematic rigidbody
		// _rb : a kinematic rigidbody.
		// _linVel : the new linear velocity of the kinematic rigidbody.
		void setKinematicRigidbodyLinearVelocity(snRigidbody* _rb, const snVec& _linVel);

		//Make a raycast test against the entire scene.
		// _ray : the ray to use for the raycast.
		// _hit : closest hit point if any is found.
		// return : true if a hit point is found. False otherwise.
		bool raycast(const snRay& _ray, snVec& _hit) const;

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
		void computeCollisionDetection(snRigidbody* _a, snRigidbody* _b);

		//Store a pair of actor into the PCS.
		void storeActorPair(snRigidbody* _a, snRigidbody* _b);

		//Add an actor to the scene.
		// _actor : a pointer to an actor.
		void attachActorByPointer(snRigidbody* const _actor);

		//Remove an actor from the scene.
		// _actor : a pointer to the actor to remove.
		void removeActorByPointer(snRigidbody const * const _actor);

		//Compute the next position and orientation of a rigidbody base on the time elapsed since the previous state
		// and the its current velocity.
		// _rb : a pointer to a rigibdbody.
		// _dt : time step for the integration.
		void integrate(snRigidbody* _rb, float _dt);

		//Awake the rigid bodies linked to the body _rg through a constraint and apply the same process
		//recursively to the newly awaken rigid bodies.
		void awakeRigidbodiesLinkedByConstraints(snRigidbody* _rb) const;
	};
}

#endif //SN_SCENE_H