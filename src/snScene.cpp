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

#include "snScene.h"

#include "snRigidbody.h"
#include "snActorPair.h"
#include "snICollider.h"
#include "snCollision.h"
#include "snCollisionResult.h"
#include "snMath.h"
#include "snQuaternion.h"
#include "snFeatureClipping.h"

#include "snFixedConstraint.h"
#include "snPointToPointConstraint.h"
#include "snContactConstraint.h"
#include "snHingeConstraint.h"

#include "snSphere.h"
#include "snCapsule.h"

#include "snTimer.h"

#ifdef SN_DEBUGGER
#include "snDebugger.h"
#endif //ifdef SN_DEBUGGER

#ifdef _DEBUG
#include <Windows.h>
#include <string>
#endif // ifdef _DEBUG

#include <algorithm>    
using std::max;

#include <assert.h>

using namespace Supernova::Vector;

namespace Supernova
{

	snCollision snScene::m_collisionService;

	snScene::snScene() : snObject(), m_linearSquaredSpeedThreshold(0.005f),
		m_angularSquaredSpeedThreshold(0.001f), m_solverIterationCount(10),
		m_collisionMode(snECollisionMode_ST_SweepAndPrune), m_contactConstraintBeta(0.25f), m_sweepAndPrune()
	{
		m_gravity = snVec4Set(0, -9.81f, 0, 0);
		m_sweepAndPrune.setCallback(this, &snScene::computeCollisionDetection);
		m_frictionMode = snFrictionMode::SN_FRICTION_ONE_DIRECTION;
		m_sleepingPeriod = 0.5f;
	}

	snScene::~snScene()
	{
		clearScene();
	}

	void snScene::attachActor(snhRigidbody _actor)
	{
		attachActorByPointer(_actor.getPtr());
	}

	void snScene::removeActor(snhRigidbody _actor)
	{
		removeActorByPointer(_actor.getPtr());
	}

	void snScene::deleteActor(snhRigidbody _actor)
	{
		//Remove the actor
		removeActor(_actor);

		//delete the actor
		snRigidbody* ptr = _actor.getPtr();
		if (ptr == 0)
			return;

		delete ptr;
	}

	snIConstraint* snScene::getConstraint(unsigned int _constraintId)
	{
		assert(_constraintId < m_constraints.size());

		return m_constraints[_constraintId];
	}

	snPointToPointConstraint* snScene::createPointToPointConstraint(snRigidbody* const _body1, const snVec& _offset1, snRigidbody* const _body2, 
		const snVec& _offset2)
	{
		snPointToPointConstraint* constraint = new snPointToPointConstraint(_body1, _offset1, _body2, _offset2);
		m_constraints.push_back(constraint);
		return constraint;
	}

	snFixedConstraint* snScene::createFixedConstraint(snRigidbody* const _actor, const snVec& _fixedPoint, float _distance)
	{
		snFixedConstraint* constraint = new snFixedConstraint(_actor, _fixedPoint, _distance);
		m_constraints.push_back(constraint);
		return constraint;
	}

	snHingeConstraint* snScene::createHingeConstraint(snRigidbody* _actor, const snVec& _axis, const snVec& _anchor)
	{
		snHingeConstraint* constraint = new snHingeConstraint(_actor, _axis, _anchor);
		m_constraints.push_back(constraint);
		return constraint;
	}

	void snScene::clearScene()
	{
		for (vector<snRigidbody*>::iterator i = m_actors.begin(); i != m_actors.end(); ++i)
		{
			if ((*i) == 0)
				continue;

			delete *i;
		}
		m_actors.clear();

		for (vector<snIConstraint*>::iterator i = m_constraints.begin(); i != m_constraints.end(); ++i)
		{
			if ((*i) == 0)
				continue;

			delete *i;
		}
		m_constraints.clear();

		m_sweepAndPrune.clearList();
	}

	void snScene::update(float _deltaTime)
	{
#ifdef SN_DEBUGGER
		long long startTimer = snTimer::getCurrentTick();
#endif //ifdef SN_DEBUGGER

		//compute collision points
		switch (m_collisionMode)
		{
		case snCollisionMode::snECollisionModeBruteForce:
			computeNaiveCollisions();
			break;

		case snCollisionMode::snECollisionMode_ST_SweepAndPrune:
			singleThreadedBroadPhase();
			break;

		case snCollisionMode::snECollisionMode_MT_SweepAndPrune:
			multiThreadedBroadPhase();
			multiThreadedNarrowPhase();
			break;
		}

#ifdef SN_DEBUGGER
		float durationMS = snTimer::convertElapsedTickCountInMilliSeconds(snTimer::getElapsedTickCount(startTimer));
		DEBUGGER->setWatchExpression(L"Collision Detection (ms)", std::to_wstring(durationMS));
#endif //ifdef SN_DEBUGGER

#ifdef SN_DEBUGGER
		startTimer = snTimer::getCurrentTick();
#endif //ifdef SN_DEBUGGER

		//The constraints must be prepared before applying forces.
		//applyForces updates the velocities with candidates velocities and we can't prepare constraints with candidate velocities.
		prepareConstraints(_deltaTime);

		//apply forces and compute candidate velocities for actors.
		applyForces(_deltaTime);

		//apply impulses
		resolveAllConstraints();

#ifdef SN_DEBUGGER
		durationMS = snTimer::convertElapsedTickCountInMilliSeconds(snTimer::getElapsedTickCount(startTimer));
		DEBUGGER->setWatchExpression(L"Constraint Solver (ms)", std::to_wstring(durationMS));
#endif //ifdef SN_DEBUGGER

		//update positions
		updatePosition(_deltaTime);	
	}

	const snVecVector& snScene::getCollisionPoints() const
	{
		return m_collisionPoints;
	}

	snCollisionMode snScene::getCollisionMode() const
	{
		return m_collisionMode;
	}

	float snScene::getContactConstraintBeta() const
	{
		return m_contactConstraintBeta;
	}

	snFrictionMode snScene::getFrictionMode() const
	{
		return m_frictionMode;
	}

	void snScene::setGravity(const snVec& _gravity)
	{
		m_gravity = _gravity;
	}

	void snScene::setLinearSquaredSpeedThreshold(float _linearSquaredSpeedThreshold)
	{
		m_linearSquaredSpeedThreshold = _linearSquaredSpeedThreshold;
	}

	void snScene::setAngularSquaredSpeedThreshold(float _angularSquaredSpeedThreshold)
	{
		m_angularSquaredSpeedThreshold = _angularSquaredSpeedThreshold;
	}

	void snScene::setSolverIterationCount(int _solverIterationCount)
	{
		m_solverIterationCount = _solverIterationCount;

#ifdef SN_DEBUGGER
		DEBUGGER->setWatchExpression(L"Solver Iteration Count", std::to_wstring(m_solverIterationCount));
#endif

	}

	void snScene::setCollisionMode(snCollisionMode _collisionMode)
	{
		m_collisionMode = _collisionMode;

		switch (m_collisionMode)
		{
		case snCollisionMode::snECollisionModeBruteForce:
			break;

		case snCollisionMode::snECollisionMode_ST_SweepAndPrune:
			m_sweepAndPrune.setCallback(this, &snScene::computeCollisionDetection);
			break;

		case snCollisionMode::snECollisionMode_MT_SweepAndPrune:
			m_sweepAndPrune.setCallback(this, &snScene::storeActorPair);
			break;
		}

	}

	void snScene::setContactConstraintBeta(float _beta)
	{
		m_contactConstraintBeta = _beta;
	}

	void snScene::setFrictionMode(snFrictionMode _mode)
	{
		m_frictionMode = _mode;
	}

	void snScene::setSleepingPeriod(float _dt)
	{
		m_sleepingPeriod = _dt;
	}

	void* snScene::operator new(size_t _count)
	{
		return _aligned_malloc(_count, SN_ALIGN_SIZE);
	}

	void snScene::operator delete(void* _p)
	{
		_aligned_free(_p);
	}

	bool snScene::sphereCast(const snVec& _center, float _radius, const snVec& _direction, float _length)
	{
		//make a capsule
		snCapsule capsule(_center, _center + _direction * _length, _radius);
		snAABB bb; 
		capsule.computeAABB(&bb);

		//Make the list of possibly colliding actors using the sweep and prune list
		vector<snRigidbody*> pca;
		m_sweepAndPrune.getPossiblyCollidingActor(bb, pca);

		//For each pca, check for collision
		bool collision = false;
		for (vector<snRigidbody*>::iterator i = pca.begin(); i != pca.end(); ++i)
		{
			if ((*i)->getName() == "ball")
				continue;

			vector<snICollider*>& colliders = (*i)->getColliders();
			for (vector<snICollider*>::iterator c = colliders.begin(); c != colliders.end(); ++c)
			{
				//make collision test
				snCollisionResult res = m_collisionService.queryTestCollision(&capsule, (*c));

				//If a collision is detected, return true.
				if (res.m_collision)
				{
					collision = true;
					break;
				}
			}
		}
		
		return collision;
		
	}

	bool snScene::shapeCast(snICollider& _collider, const snTransform& _origin, const snVec& _direction, float _length, float& _distance) const
	{
		bool result = false;
		_distance = _length;

		snVec _startPosition = _origin.getPosition();
		snTransform castTransform = _origin;

		/*while (true)
		{*/
			//Move the collider to the correct position.
		_collider.updateFromTransform();

			//Get the bounding volume
			snAABB bb;
			_collider.computeAABB(&bb);

			//Make the list of possibly colliding actors using the sweep and prune list
			vector<snRigidbody*> pca;
			m_sweepAndPrune.getPossiblyCollidingActor(bb, pca);

			//Loop through the possibly colliding actor
			for (vector<snRigidbody*>::const_iterator i = pca.begin(); i != pca.end(); ++i)
			{
				if ((*i)->getName() == "ball")
					continue;

				vector<snICollider*>& colliders = (*i)->getColliders();
				for (vector<snICollider*>::const_iterator c = colliders.begin(); c != colliders.end(); ++c)
				{
					//make collision test
					snCollisionResult collisionTestResult = m_collisionService.queryTestCollision(&_collider, (*c));
					if (collisionTestResult.m_collision) //ignore colliding shapes
					{
						continue;
					}

					//Make distance test
					snGJK gjk;
					float currentDistance = -1;
					bool distanceResult = gjk.distance(_collider, **c, currentDistance);
					if (!distanceResult) //The distance could not be found
					{
						continue;
					}

					if (currentDistance < _distance)
					{
						result = true;
						_distance = currentDistance;
					}
					
				}
			}
		//}
			return result;

	}

	void snScene::setKinematicRigidbodyLinearVelocity(snRigidbody* _rb, const snVec& _linVel)
	{
		assert(_rb->isKinematic());

		_rb->setLinearVelocity(_linVel);

		if(!_rb->isAwake())
		{
			_rb->isAwake();
			m_contactConstraintManager.awakeConstraint(_rb);
		}
	}

	void snScene::applyForces(float _dt)
	{
		for (vector<snRigidbody*>::iterator i = m_actors.begin(); i != m_actors.end(); ++i)
		{
			if ((*i) == 0 || !(*i)->getIsActive() || !(*i)->isAwake())
				continue;

			//apply gravity only if the inverse of the mass is not zero.
			//If it is zero, then the actor is static or kinematic so no gravity as to be applied.
			if ((*i)->getInvMass() != 0)
				(*i)->setLinearVelocity((*i)->getLinearVelocity() + (m_gravity * _dt));
		}
	}

	void snScene::updatePosition(float _dt)
	{
		for (vector<snRigidbody*>::iterator i = m_actors.begin(); i != m_actors.end(); ++i)
		{
			if ((*i) == 0 || !(*i)->getIsActive())
				continue;

			integrate(*i, _dt);
		}
	}

	void snScene::resolveAllConstraints()
	{
		//resolve the constraints
		for (int i = 0; i < m_solverIterationCount; ++i)
		{
			//resolve the contact constraints
			m_contactConstraintManager.resolveActiveConstraints();

			for (vector<snIConstraint*>::iterator constraint = m_constraints.begin(); constraint != m_constraints.end(); ++constraint)
				(*constraint)->resolve();
		}
	}

	void snScene::prepareConstraints(float _dt)
	{
		//prepare the collision constraints
		m_contactConstraintManager.prepareActiveConstraint(_dt);

		//prepare the scene constraints
		for (vector<snIConstraint*>::iterator constraint = m_constraints.begin(); constraint != m_constraints.end(); ++constraint)
			(*constraint)->prepare(_dt);
	}

	void snScene::computeNaiveCollisions()
	{
		m_collisionPoints.clear();

		if (m_actors.size() <= 1)
			return;

		m_contactConstraintManager.preBroadPhase();

#ifdef SN_DEBUGGER
		int collisionQueriesCount = 0;
#endif // ifdef SN_DEBUGGER

		for (std::vector<snRigidbody*>::iterator i = m_actors.begin(); i != m_actors.end() - 1; ++i)
		{
			if ((*i) == 0 || !(*i)->getIsActive())
				continue;

			for (std::vector<snRigidbody*>::iterator j = i + 1; j != m_actors.end(); ++j)
			{
				if ((*j) == 0 || !(*j)->getIsActive())
					continue;

				//check if the collision detection is enabled between the two actors
				if (!snRigidbody::isCollisionDetectionEnabled(*i, *j))
					continue;

#ifdef SN_DEBUGGER
				++collisionQueriesCount;
#endif //ifdef SN_DEBUGGER
				computeCollisionDetection(*i, *j);
			}

			//deactivate all unused constraints
			m_contactConstraintManager.postBroadPhase();
		}

#ifdef SN_DEBUGGER
		DEBUGGER->setWatchExpression(L"Collision Queries Count", std::to_wstring(collisionQueriesCount));
#endif //ifdef SN_DEBUGGER
	}

	void snScene::singleThreadedBroadPhase()
	{
		//Clear the list of collision points
		m_collisionPoints.clear();

		//Prepare for the broad phase. First prepare the constraint manager then prepare the sweep and prune manager
		m_contactConstraintManager.preBroadPhase();
		m_sweepAndPrune.preBroadPhase();

		//Apply the broad phase
		m_sweepAndPrune.broadPhase();

		//Post broad phase for the constraints manager and the sweep and prune manager.
		m_contactConstraintManager.postBroadPhase();
		m_sweepAndPrune.postBroadPhase();
	}

	void snScene::multiThreadedBroadPhase()
	{
		//Clear the list of collision points
		m_collisionPoints.clear();

		//Prepare for the broad phase.
		m_contactConstraintManager.preBroadPhase();
		m_pcs.preBroadPhase();
		m_sweepAndPrune.preBroadPhase();

		//Apply the broad phase
		m_sweepAndPrune.broadPhase();

		//Post broad phase
		m_contactConstraintManager.postBroadPhase();
		m_pcs.postBroadPhase();
		m_sweepAndPrune.postBroadPhase();
	}

	//Narrow phase for the multithreaded collision detection model.
	void snScene::multiThreadedNarrowPhase()
	{
		//dispatch to threads which will run the collision detection.
	}

	void snScene::computeCollisionDetection(snRigidbody* _a, snRigidbody* _b)
	{
		//query collision between actor _a and _b.
		const unsigned int MAX_COL_RES = 10;
		snCollisionResult colRes[MAX_COL_RES];
		unsigned int colResCount = 0;
		m_collisionService.queryTestCollision(_a, _b, colRes, MAX_COL_RES, &colResCount);

		//no collision, leave
		if (colResCount == 0)
			return;

		//check for collision callbacks
		if (_a->isEnabledCollisionFlag(snCollisionFlag::CF_CONTACT_CALLBACK))
			_a->onCollision(_b);
		if (_b->isEnabledCollisionFlag(snCollisionFlag::CF_CONTACT_CALLBACK))
			_b->onCollision(_a);

		//check if a collision response is needed
		if (_a->isEnabledCollisionFlag(snCollisionFlag::CF_NO_CONTACT_RESPONSE) ||
			_b->isEnabledCollisionFlag(snCollisionFlag::CF_NO_CONTACT_RESPONSE))
		{
			return;
		}

		//Awake the bodies
		if(!_a->isAwake())
		{
			_a->setAwake(true);
			m_contactConstraintManager.awakeConstraint(_a);
			awakeRigidbodiesLinkedByConstraints(_a);
		}
		if(!_b->isAwake())
		{
			_b->setAwake(true);
			m_contactConstraintManager.awakeConstraint(_b);
			awakeRigidbodiesLinkedByConstraints(_b);
		}

		//make the collision constraints from the collision results
		for (unsigned int colResId = 0; colResId < colResCount; ++colResId)
		{
			snCollisionResult* singleRes = colRes + colResId;

			vector<float>::const_iterator penetrationIterator = singleRes->m_penetrations.cbegin();
			for (snVecVectorConstIterator point = singleRes->m_contacts.cbegin(); point != singleRes->m_contacts.cend(); ++point, ++penetrationIterator)
			{
				m_collisionPoints.push_back(*point);

				//if a constraints already exists, take it and reuse it or else create it.
				snContactConstraint* contactConstraint = m_contactConstraintManager.getAvailableConstraint();

				//initialize and activate the constraints
				contactConstraint->initialize(_a, _b, singleRes->m_normal, *point, *penetrationIterator, this);
				contactConstraint->setIsActive(true);
			}
		}
	}

	void snScene::storeActorPair(snRigidbody* _a, snRigidbody* _b)
	{
		snActorPair* pair = m_pcs.getAvailablePair();
		pair->m_first = _a;
		pair->m_second = _b;
	}

	void snScene::attachActorByPointer(snRigidbody* const _actor)
	{
		if (_actor == 0)
			return;

		//try to add it to the vector
		bool actorAdded = false;
		for (unsigned int i = 0; i < m_actors.size(); ++i)
		{
			if (m_actors[i] != 0)
				continue;

			m_actors[i] = _actor;
			actorAdded = true;
			break;
		}

		//no existing spot found so push back
		if (!actorAdded)
		{
			m_actors.push_back(_actor);
		}

		//Add the actor to the broad phase.
		m_sweepAndPrune.addActor(_actor);
	}

	void snScene::removeActorByPointer(snRigidbody const * const _actor)
	{
		if (_actor == 0)
			return;

		//Remove the actor from the broad phase
		m_sweepAndPrune.removeActor(_actor);

		//Remove the actor from the scene
		for (vector<snRigidbody*>::iterator i = m_actors.begin(); i != m_actors.end(); ++i)
		{
			if ((*i) == _actor)
			{
				(*i) = 0;
			}
		}
	}

	void snScene::integrate(snRigidbody* const _rb, float _dt)
	{
		if(_rb->isStatic() || !_rb->isAwake())
			return;

		//apply damping
		snVec linVel = _rb->getLinearVelocity() * (1 - _rb->getLinearDampingCoeff() * _dt);
		snVec angVel = _rb->getAngularVelocity() * (1 - _rb->getAngularDampingCoeff() * _dt);
		
		//if the linear speed is too small, set it to 0.
		float sqSpeed = snVec3SquaredNorme(linVel);
		if (sqSpeed < m_linearSquaredSpeedThreshold)
			linVel = snVec4Set(0);

		//if the angular speed is too small, set it to 0.
		sqSpeed = snVec3SquaredNorme(angVel);
		if (sqSpeed < m_angularSquaredSpeedThreshold)
			angVel = snVec4Set(0);

		
		//calculate position using euler integration
		snVec previousPosition = _rb->getTransform().getPosition();
		_rb->getTransform().setLocalPosition(previousPosition + linVel * _dt);
		
		//calculate velocity as quaternion using dq/dt = 0.5 * w * q
		snVec q = _rb->getTransform().getOrientation();
		snVec qw = snQuaternionMultiply(angVel, q);
		qw = qw * 0.5f;

		//calculate orientation using euler integration
		q = q + (qw * _dt);
		snQuaternionNormalize(q, q);
		_rb->getTransform().setLocalOrientation(q);

		//Set the velocities.
		_rb->setLinearVelocity(linVel);
		_rb->setAngularVelocity(angVel);

		//set new state
		_rb->computeInvWorldInertia();
		_rb->computeWorldCenterOfMass();

		//compute colliders in world coordinate
		_rb->updateCollidersAndAABB();
		
		//update the sleeping state
		_rb->updateSleepingState(_dt, m_sleepingPeriod);

		if(!_rb->isAwake())
		{
			m_contactConstraintManager.setSleepingBody(_rb);
		}
	}

	void snScene::awakeRigidbodiesLinkedByConstraints(snRigidbody* _rb) const
	{
		vector<snRigidbody*> toAwake;

		for(vector<snIConstraint*>::const_iterator i = m_constraints.cbegin(); i != m_constraints.cend(); ++i)
		{
			snRigidbody * const* const bodies = (*i)->getBodies();
			unsigned int count = (*i)->getBodiesCount();

			//Check if the constraint has the body _rb
			bool isLinked = false;
			for(unsigned int bodyId = 0; bodyId < count; ++bodyId)
			{
				if(bodies[bodyId] == _rb)
				{
					isLinked = true;
					break;
				}
			}

			if(!isLinked)
			{
				continue;
			}

			//Activate all the other bodies if necessary
			for(unsigned int bodyId = 0; bodyId < count; ++bodyId)
			{
				if(bodies[bodyId] == _rb)
				{
					continue;
				}

				if(bodies[bodyId]->isAwake())
				{
					continue;
				}

				bodies[bodyId]->setAwake(true);
				toAwake.push_back(bodies[bodyId]);
			}
		}

		//Recurvisely wake up all the linked bodies.
		for(vector<snRigidbody*>::iterator iter = toAwake.begin(); iter != toAwake.end(); ++iter)
		{
			awakeRigidbodiesLinkedByConstraints(*iter);
		}
	}
}