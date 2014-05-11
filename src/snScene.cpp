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

#include "snActorDynamic.h"
#include "snActorStatic.h"
#include "snICollider.h"
#include "snCollision.h"
#include "snCollisionResult.h"
#include "snMath.h"
#include "snQuaternion.h"
#include "snFeatureClipping.h"

#include "snFixedConstraint.h"
#include "snPointToPointConstraint.h"
#include "snContactConstraint.h"
#include "snFrictionConstraint.h"

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

	snScene::snScene() :m_linearSquaredSpeedThreshold(0.005f),
		m_angularSquaredSpeedThreshold(0.001f), m_solverIterationCount(10),
		m_collisionMode(snECollisionModeSweepAndPrune), m_contactConstraintBeta(0.25f), m_sweepAndPrune()
	{
		m_gravity = snVec4Set(0, -9.81f, 0, 0),
		m_sweepAndPrune.setCallback(this, &snScene::computeCollisionDetection);
	}

	snScene::~snScene()
	{
		clearScene();
	}

	void snScene::createActorDynamic(snActorDynamic** _newActor, int& _actorId)
	{
		//create the actor
		*_newActor = new snActorDynamic();

		_actorId = attachActor(*_newActor);
	}

	void snScene::createActorStatic(snActorStatic** _newActor, int& _actorId, const snVec& _position, const snVec& _orientation)
	{
		//create the actor
		*_newActor = new snActorStatic(_position, _orientation);

		//attach the actor to the current scene
		_actorId = attachActor(*_newActor);
	}

	void snScene::deleteActor(unsigned int _actorId)
	{
		//id out of range
		if (_actorId >= m_actors.size())
			return;

		//actor already deleted
		if (m_actors[_actorId] == 0)
			return;

		//delete actor
		delete m_actors[_actorId];
		m_actors[_actorId] = 0;

	}

	int snScene::attachActor(snIActor* _actor)
	{
		//try to add it to the vector
		for (unsigned int i = 0; i < m_actors.size(); ++i)
		{
			if (m_actors[i] != 0)
				continue;

			m_actors[i] = _actor;
			return i;
		}

		//no existing spot found so push back
		int actorId = m_actors.size();
		m_actors.push_back(_actor);
		m_sweepAndPrune.addActor(_actor);
		return actorId;
	}

	void snScene::removeActor(unsigned int _actorId)
	{
		if (m_actors.size() <= _actorId)
			return;

		m_actors[_actorId] = 0;
	}

	snIActor* snScene::getActor(unsigned int _actorId)
	{
		//id out of range
		if (_actorId >= m_actors.size())
			return 0;

		return m_actors[_actorId];
	}

	snIConstraint* snScene::getConstraint(unsigned int _constraintId)
	{
		assert(_constraintId < m_constraints.size());

		return m_constraints[_constraintId];
	}

	snPointToPointConstraint* snScene::createPointToPointConstraint(snIActor* const _body1, const snVec& _offset1, snIActor* const _body2, 
		const snVec& _offset2)
	{
		snPointToPointConstraint* constraint = new snPointToPointConstraint(_body1, _offset1, _body2, _offset2);
		m_constraints.push_back(constraint);
		return constraint;
	}

	snFixedConstraint* snScene::createFixedConstraint(snIActor* const _actor, const snVec& _fixedPoint, float _distance)
	{
		snFixedConstraint* constraint = new snFixedConstraint(_actor, _fixedPoint, _distance);
		m_constraints.push_back(constraint);
		return constraint;
	}

	void snScene::clearScene()
	{
		for (vector<snIActor*>::iterator i = m_actors.begin(); i != m_actors.end(); ++i)
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

		case snCollisionMode::snECollisionModeSweepAndPrune:
			computeBroadPhaseCollisions();
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

#ifdef SN_DEBUGGER
		switch (m_collisionMode)
		{
		case snCollisionMode::snECollisionModeBruteForce:
			DEBUGGER->setWatchExpression(L"Collision Mode", L"Brute Force");
			break;

		case snCollisionMode::snECollisionModeSweepAndPrune:
			DEBUGGER->setWatchExpression(L"Collision Mode", L"Sweep And Prune");
			break;
		}
#endif //ifdef SN_DEBUGGER
	}

	void snScene::setContactConstraintBeta(float _beta)
	{
		m_contactConstraintBeta = _beta;
	}

	void* snScene::operator new(size_t _count)
	{
		return _aligned_malloc(_count, SN_ALIGN_SIZE);
	}

	void snScene::operator delete(void* _p)
	{
		_aligned_free(_p);
	}

	void snScene::applyForces(float _dt)
	{
		for (vector<snIActor*>::iterator i = m_actors.begin(); i != m_actors.end(); ++i)
		{
			if (!(*i)->getIsActive())
				continue;

			//apply gravity only if the inverse of the mass is not zero.
			//If it is zero, then the actor is static or kinematic so no gravity as to be applied.
			if ((*i)->getInvMass() != 0)
				(*i)->setLinearVelocity((*i)->getLinearVelocity() + (m_gravity * _dt));
		}
	}

	void snScene::updatePosition(float _dt)
	{
		for (vector<snIActor*>::iterator i = m_actors.begin(); i != m_actors.end(); ++i)
		{
			if (!(*i)->getIsActive())
				continue;

			(*i)->integrate(_dt, m_linearSquaredSpeedThreshold, m_angularSquaredSpeedThreshold);
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

		for (std::vector<snIActor*>::iterator i = m_actors.begin(); i != m_actors.end() - 1; ++i)
		{
			if (!(*i)->getIsActive())
				continue;

			for (std::vector<snIActor*>::iterator j = i + 1; j != m_actors.end(); ++j)
			{
				if (!(*j)->getIsActive())
					continue;

				//check if the collision detection is enabled between the two actors
				if (!isCollisionDetectionEnabled(*i, *j))
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

	void snScene::computeBroadPhaseCollisions()
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

	void snScene::computeCollisionDetection(snIActor* _a, snIActor* _b)
	{
		snCollisionResult res = m_collisionService.queryTestCollision(_a, _b);

		//no collision, leave
		if (!res.m_collision)
			return;

		//check for collision callbacks
		if (_a->isEnabledCollisionFlag(snCollisionFlag::CF_CONTACT_CALLBACK))
			_a->OnCollision(_b);
		if (_b->isEnabledCollisionFlag(snCollisionFlag::CF_CONTACT_CALLBACK))
			_b->OnCollision(_a);

		//check if a collision response is needed
		if (_a->isEnabledCollisionFlag(snCollisionFlag::CF_NO_CONTACT_RESPONSE) ||
			_b->isEnabledCollisionFlag(snCollisionFlag::CF_NO_CONTACT_RESPONSE))
			return;

		

		//make the collision constraints from the collision results
		vector<float>::const_iterator penetrationIterator = res.m_penetrations.cbegin();
		for (snVecVectorConstIterator point = res.m_contacts.cbegin(); point != res.m_contacts.cend(); ++point, ++penetrationIterator)
		{
			m_collisionPoints.push_back(*point);

			//if a constraints already exists, take it and reuse it or else create it.
			snContactConstraint* npConstraint = 0;
			snFrictionConstraint* fConstraint = 0;
			m_contactConstraintManager.getAvailableConstraints(&npConstraint, &fConstraint);

			//initialize and activate the constraints
			npConstraint->initialize(_a, _b, res.m_normal, *point, *penetrationIterator, this);
			fConstraint->initialize(_a, _b, npConstraint);
			npConstraint->setIsActive(true);
			fConstraint->setIsActive(true);
		}
	}

	//Check if the collision detection is enabled between the two actors
	bool snScene::isCollisionDetectionEnabled(const snIActor* const _a, const snIActor* const _b)
	{
		//No colision detection between two statics or a kinematic and a static.
		if ((_a->getActorType() == snActorType::snActorTypeStatic && _b->getActorType() == snActorType::snActorTypeStatic) ||
			(_a->getActorType() == snActorType::snActorTypeStatic && _b->getActorType() == snActorType::snActorTypeKinematic) ||
			(_a->getActorType() == snActorType::snActorTypeKinematic && _b->getActorType() == snActorType::snActorTypeStatic))
			return false;

		return true;
	}
}