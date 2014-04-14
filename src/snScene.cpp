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

#include "snActor.h"
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


namespace Supernova
{

	snCollision snScene::m_collisionService;

	snScene::snScene() : m_gravity(0, -9.81f, 0, 0), m_linearSquaredSpeedThreshold(0.005f),
		m_angularSquaredSpeedThreshold(0.001f), m_solverIterationCount(10),
		m_sweepList(), m_sweepAxis(0), m_collisionMode(snECollisionModeSweepAndPrune)
	{
	}

	snScene::~snScene()
	{
		clearScene();
	}

	void snScene::createActor(snActor** _newActor, int& _actorId)
	{
		//create the actor
		*_newActor = new snActor();

		_actorId = attachActor(*_newActor);
	}

	void snScene::createStaticActor(snActor** _newActor, int& _actorId)
	{
		//create the actor
		*_newActor = new snActor(true);

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

	int snScene::attachActor(snActor* _actor)
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
		m_sweepList.push_back(_actor);
		return actorId;
	}

	void snScene::removeActor(unsigned int _actorId)
	{
		if (m_actors.size() <= _actorId)
			return;

		m_actors[_actorId] = 0;
	}

	snActor* snScene::getActor(unsigned int _actorId)
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

	snPointToPointConstraint* snScene::createPointToPointConstraint(snActor* const _body1, const snVector4f& _offset1, snActor* const _body2, 
		const snVector4f& _offset2)
	{
		snPointToPointConstraint* constraint = new snPointToPointConstraint(_body1, _offset1, _body2, _offset2);
		m_constraints.push_back(constraint);
		return constraint;
	}

	snFixedConstraint* snScene::createFixedConstraint(snActor* const _actor, const snVector4f& _fixedPoint, float _distance)
	{
		snFixedConstraint* constraint = new snFixedConstraint(_actor, _fixedPoint, _distance);
		m_constraints.push_back(constraint);
		return constraint;
	}

	void snScene::clearScene()
	{
		for (vector<snActor*>::iterator i = m_actors.begin(); i != m_actors.end(); ++i)
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

		m_sweepList.clear();
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

		//The constraints must be prepared before applying forces.
		//applyForces updates the velocities with candidates velocities and we can't prepare constraints with candidate velocities.
		prepareConstraints(_deltaTime);

#ifdef SN_DEBUGGER
		startTimer = snTimer::getCurrentTick();
#endif //ifdef SN_DEBUGGER

		//apply forces and compute candidate velocities for actors.
		applyForces(_deltaTime);

#ifdef SN_DEBUGGER
		durationMS = snTimer::convertElapsedTickCountInMilliSeconds(snTimer::getElapsedTickCount(startTimer));
		DEBUGGER->setWatchExpression(L"Constraint Solver (ms)", std::to_wstring(durationMS));
#endif //ifdef SN_DEBUGGER

		//apply impulses
		resolveAllConstraints();

		//update positions
		updatePosition(_deltaTime);	
	}

	const snVector4fVector& snScene::getCollisionPoints() const
	{
		return m_collisionPoints;
	}

	snCollisionMode snScene::getCollisionMode() const
	{
		return m_collisionMode;
	}

	void snScene::setGravity(const snVector4f& _gravity)
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
		for (vector<snActor*>::iterator i = m_actors.begin(); i != m_actors.end(); ++i)
		{
			//calculate weight
			snVector4f W = m_gravity * (*i)->getMass();

			//calculate linear velocity
			snVector4f v = (*i)->getLinearVelocity() + (W * _dt * (*i)->getInvMass());
	
			//set state
			(*i)->setLinearVelocity(v);
		}
	}

	void snScene::updatePosition(float _dt)
	{
		for (vector<snActor*>::iterator i = m_actors.begin(); i != m_actors.end(); ++i)
		{
			//do not simulate static actors.
			if ((*i)->getIsStatic())
				continue;

			//apply damping
			(*i)->setLinearVelocity((*i)->getLinearVelocity() * (1 - (*i)->getLinearDampingCoeff() * _dt));
			(*i)->setAngularVelocity((*i)->getAngularVelocity() * (1 - (*i)->getAngularDampingCoeff() * _dt));

			//if the linear speed is too small, set it to 0.
			float sqSpeed = (*i)->getLinearVelocity().squareNorme();
			if (sqSpeed < m_linearSquaredSpeedThreshold)
				(*i)->setLinearVelocity(snVector4f());

			//if the angular speed is too small, set it to 0.
			sqSpeed = (*i)->getAngularVelocity().squareNorme();
			if (sqSpeed < m_angularSquaredSpeedThreshold)
				(*i)->setAngularVelocity(snVector4f());

			//calculate position
			snVector4f x = (*i)->getPosition() + (*i)->getLinearVelocity() * _dt;

			//calculate velocity as quaternion using dq/dt = 0.5 * w * q
			snVector4f qw;
			snQuaternionMultiply((*i)->getAngularVelocity(), (*i)->getOrientationQuaternion(), qw);
			qw = qw * 0.5f;

			//calculate orientation
			snVector4f q = (*i)->getOrientationQuaternion() + (qw * _dt);
			snQuaternionNormalize(q, q);

			//compute orientation as a matrix
			snMatrix44f snR;
			snR.createRotationFromQuaternion(q);

			//set new state
			(*i)->setPosition(x);
			(*i)->setOrientationQuaternion(q);
			(*i)->setOrientationMatrix(snR);

			//compute inertia in world coordinate
			(*i)->computeWInertiaTensor();
			(*i)->computeWInvInertiaTensor();

			//compute colliser in world coordinate
			(*i)->updateCollider(x, snR);
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

		for (std::vector<snActor*>::iterator i = m_actors.begin(); i != m_actors.end() - 1; ++i)
		{
			for (std::vector<snActor*>::iterator j = i + 1; j != m_actors.end(); ++j)
			{

				//do not check collision between static actors.
				if ((*i)->getIsStatic() && (*j)->getIsStatic())
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
		m_collisionPoints.clear();
		m_contactConstraintManager.preBroadPhase();

#ifdef SN_DEBUGGER
		long long startTimer = snTimer::getCurrentTick();
#endif //ifdef SN_DEBUGGER

		//sort the list in ascending order
		m_sweepList.sort([this](const snActor* _a, const snActor* _b)
		{
			return _a->getBoundingVolume()->m_min[m_sweepAxis] < _b->getBoundingVolume()->m_min[m_sweepAxis];	
		});

#ifdef SN_DEBUGGER
		float durationMS = snTimer::convertElapsedTickCountInMilliSeconds(snTimer::getElapsedTickCount(startTimer));
		DEBUGGER->setWatchExpression(L"Broad Phase - Sort (ms)", std::to_wstring(durationMS));
#endif

		//sum and squared sum of the AABB center
		snVector4f s, s2;

#ifdef 	SN_DEBUGGER
		int collisionQueriesCount = 0;
#endif //ifdef SN_DEBUGGER

#ifdef SN_DEBUGGER
		startTimer = snTimer::getCurrentTick();
#endif //ifdef SN_DEBUGGER

		//loop through each actor in the scene using the sweep list
		for (list<snActor*>::iterator i = m_sweepList.begin(); i != m_sweepList.end(); ++i)
		{
			//compute aabb center point
			snVector4f center = ((*i)->getBoundingVolume()->m_max + (*i)->getBoundingVolume()->m_min) * 0.5f;

			//compute sum and sum squared to compute variance later
			s = s + center;
			s2 = s2 + (center * center);

			//test collision against all other actors
			list<snActor*>::iterator j = i;
			++j;
			while (j != m_sweepList.end())
			{
				//check if the tested bounding volume(j) is too far to the current bounding volume (i)
				if ((*j)->getBoundingVolume()->m_min[m_sweepAxis] > (*i)->getBoundingVolume()->m_max[m_sweepAxis])
					break;

				if (AABBOverlap((*i)->getBoundingVolume(), (*j)->getBoundingVolume()))
				{
#ifdef 	SN_DEBUGGER
					++collisionQueriesCount;
#endif //ifdef SN_DEBUGGER
					computeCollisionDetection(*i, *j);
				}
				++j;
			}
		}

		m_contactConstraintManager.postBroadPhase();

#ifdef SN_DEBUGGER
		durationMS = snTimer::convertElapsedTickCountInMilliSeconds(snTimer::getElapsedTickCount(startTimer));
		DEBUGGER->setWatchExpression(L"Broad Phase - Collision (ms)", std::to_wstring(durationMS));
#endif
		
#ifdef SN_DEBUGGER
		startTimer = snTimer::getCurrentTick();
#endif //ifdef SN_DEBUGGER

		//compute variance
		snVector4f v = s2 - (s * s);
		v.absolute();

		//update the axis to sort to take the axis with the greatest variance.
		m_sweepAxis = 0;
		if (v[1] > v[0]) m_sweepAxis = 1;
		if (v[2] > v[m_sweepAxis]) m_sweepAxis = 2;

#ifdef SN_DEBUGGER
		durationMS = snTimer::convertElapsedTickCountInMilliSeconds(snTimer::getElapsedTickCount(startTimer));
		DEBUGGER->setWatchExpression(L"Broad Phase - Compute Variance (ms)", std::to_wstring(durationMS));
#endif

#ifdef SN_DEBUGGER
		DEBUGGER->setWatchExpression(L"Collision Queries Count", std::to_wstring(collisionQueriesCount));
		DEBUGGER->setWatchExpression(L"Sort Variance", std::to_wstring(v[0]) + L"," + std::to_wstring(v[1]) + L"," + std::to_wstring(v[2]));
		DEBUGGER->setWatchExpression(L"Sort Axis", std::to_wstring(m_sweepAxis));
#endif //idef SN_DEBUGGER
	}

	void snScene::computeCollisionDetection(snActor* _a, snActor* _b)
	{
		snCollisionResult res = m_collisionService.queryTestCollision(_a, _b);

		//no collision, leave
		if (!res.m_collision)
			return;

		//make the collision constraints from the collision results
		vector<float>::const_iterator penetrationIterator = res.m_penetrations.cbegin();
		for (snVector4fVectorConstIterator point = res.m_contacts.cbegin(); point != res.m_contacts.cend(); ++point, ++penetrationIterator)
		{
			m_collisionPoints.push_back(*point);

			//if a constraints already exists, take it and reuse it or else create it.
			snContactConstraint* npConstraint = static_cast<snContactConstraint*>(m_contactConstraintManager.getAvailableConstraint());
			snFrictionConstraint* fConstraint = static_cast<snFrictionConstraint*>(m_contactConstraintManager.getAvailableConstraint());

			//initialize and activate the constraints
			npConstraint->initialize(_a, _b, res.m_normal, *point, *penetrationIterator, this);
			fConstraint->initialize(_a, _b, npConstraint);
			npConstraint->setIsActive(true);
			fConstraint->setIsActive(true);
		}
	}
}