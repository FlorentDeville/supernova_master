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

#include "snDistanceConstraint.h"
#include "snNonPenetrationConstraint.h"
#include "snFrictionConstraint.h"

#ifdef _DEBUG
//#include "World.h"
//#include "EntityCollisionPoint.h"
//using namespace Devil;

#include <Windows.h>
#include <string>
#endif // _DEBUG

#include <algorithm>    
using std::max;


namespace Supernova
{
	snScene::snScene() : m_beta(0.25f), m_maxSlop(0.05f), m_gravity(0, -9.81f, 0, 0), m_linearSquaredSpeedThreshold(0.005f),
		m_angularSquaredSpeedThreshold(0.001f), m_solverIterationCount(10)
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

	int snScene::createDistanceConstraint(snActor* const _body1, const snVector4f& _offset1, snActor* const _body2, const snVector4f& _offset2, 
		float _distance)
	{
		snDistanceConstraint* constraint = new snDistanceConstraint(_body1, _offset1, _body2, _offset2, _distance);
		int id = m_constraints.size();
		m_constraints.push_back(constraint);
		return id;
	}

	void snScene::clearScene()
	{
		for (vector<snActor*>::iterator i = m_actors.begin(); i != m_actors.end(); ++i)
			delete *i;
		m_actors.clear();

		for (vector<snIConstraint*>::iterator i = m_constraints.begin(); i != m_constraints.end(); ++i)
			delete *i;
		m_constraints.clear();

		for (vector<snIConstraint*>::iterator i = m_collisionConstraints.begin(); i != m_collisionConstraints.end(); ++i)
			delete *i;
		m_collisionConstraints.clear();
	}

	void snScene::update(float _deltaTime)
	{
		//compute collision points
		computeCollisions(_deltaTime);

		//The constraints must be prepared before applying forces.
		//applyForces updates the velocities with candidates velocities and we can't prepare constraints with candidate velocities.
		prepareConstraints();

		//apply forces and compute candidate velocities for actors.
		applyForces(_deltaTime);

		//apply impulses
		resolveAllConstraints();

		//update positions
		updatePosition(_deltaTime);	
	}

	const snVector4fVector& snScene::getCollisionPoints() const
	{
		return m_collisionPoints;
	}

	void snScene::setBeta(float _beta)
	{
		m_beta = _beta;
	}

	float snScene::getBeta() const
	{
		return m_beta;
	}

	void snScene::setMaxSlop(float _maxSlop)
	{
		m_maxSlop = _maxSlop;
	}

	float snScene::getMaxSlop() const
	{
		return m_maxSlop;
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
			
			//calculate angular velocity
			//XMVECTOR w = (*i)->getAngularVelocity() + XMVector4Transform((*i)->getTorque(), (*i)->getInvWorldInertia()) * _dt;
			
			//set state
			(*i)->setLinearVelocity(v);
			//(*i)->setAngularVelocity(w);
		}
	}

	void snScene::updatePosition(float _dt)
	{
		for (vector<snActor*>::iterator i = m_actors.begin(); i != m_actors.end(); ++i)
		{
			//do not simulate static actors.
			if ((*i)->getIsStatic())
				continue;

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
			for (vector<snIConstraint*>::iterator constraint = m_collisionConstraints.begin(); constraint != m_collisionConstraints.end(); ++constraint)
			{
				if ((*constraint)->getIsActive())
					(*constraint)->resolve();
			}

			for (vector<snIConstraint*>::iterator constraint = m_constraints.begin(); constraint != m_constraints.end(); ++constraint)
				(*constraint)->resolve();
		}
	}

	void snScene::prepareConstraints()
	{
		//prepare the collision constraints
		for (vector<snIConstraint*>::iterator constraint = m_collisionConstraints.begin(); constraint != m_collisionConstraints.end(); ++constraint)
			(*constraint)->prepare();

		//prepare the scene constraints
		for (vector<snIConstraint*>::iterator constraint = m_constraints.begin(); constraint != m_constraints.end(); ++constraint)
			(*constraint)->prepare();
	}

	void snScene::computeCollisions(float _dt)
	{
#if _DEBUG
			LARGE_INTEGER frequency;
			QueryPerformanceFrequency(&frequency);

			float tickPerMilliseconds = (float)frequency.QuadPart * 0.001f;
			float tickPerMicroseconds = tickPerMilliseconds * 0.001f;
#endif
			m_collisionPoints.clear();

			if (m_actors.size() <= 1)
				return;

			unsigned int currentConstraintId = 0;

			for (std::vector<snActor*>::iterator i = m_actors.begin(); i != m_actors.end() - 1; ++i)
			{
				for (std::vector<snActor*>::iterator j = i + 1; j != m_actors.end(); ++j)
				{
					//do not check collision between twe static actors.
					if ((*i)->getIsStatic() && (*j)->getIsStatic())
						continue;

					//check collisions SAT
#if _DEBUG
					LARGE_INTEGER startSAT;
					QueryPerformanceCounter(&startSAT);
#endif
					snCollisionResult res = snCollision::queryTestCollision((*i), (*j));
					//snCollisionResult res = m_GJK.queryIntersection(*(*i)->getCollider(0), *(*j)->getCollider(0));
#if _DEBUG
					LARGE_INTEGER endSAT;
					QueryPerformanceCounter(&endSAT);
					LONGLONG SATDuration = endSAT.QuadPart - startSAT.QuadPart;


					//check collision GJK
					LARGE_INTEGER startGJK;
					QueryPerformanceCounter(&startGJK);
					snVector4f simplex[4];
					//snCollisionResult _res = m_GJK.queryIntersection(*(*i)->getCollider(0), *(*j)->getCollider(0));
					LARGE_INTEGER endGJK;
					QueryPerformanceCounter(&endGJK);
					LONGLONG GJKDuration = endGJK.QuadPart - startGJK.QuadPart;
#endif
					//no collision, continue
					if (!res.m_collision)
						continue;

					//make the contact points from the collision results
					vector<float>::const_iterator penetrationIterator = res.m_penetrations.cbegin();
					for (snVector4fVectorConstIterator point = res.m_contacts.cbegin(); point != res.m_contacts.cend(); ++point, ++penetrationIterator)
					{
						m_collisionPoints.push_back(*point);

						//if a constraints already exists, take it and reuse it or else create it.
						snNonPenetrationConstraint* npConstraint = 0;
						snFrictionConstraint* fConstraint = 0;
						if (currentConstraintId < m_collisionConstraints.size())
						{
							npConstraint = static_cast<snNonPenetrationConstraint*>(m_collisionConstraints[currentConstraintId]);
							fConstraint = static_cast<snFrictionConstraint*>(m_collisionConstraints[++currentConstraintId]);
							++currentConstraintId;
						}
						else
						{
							npConstraint = new snNonPenetrationConstraint();
							m_collisionConstraints.push_back(npConstraint);

							fConstraint = new snFrictionConstraint();
							m_collisionConstraints.push_back(fConstraint);
							currentConstraintId += 2;
						}

						//initialize and activate the constraints
						npConstraint->initialize(*i, *j, res.m_normal, *point, *penetrationIterator, this, _dt);
						fConstraint->initialize(*i, *j, npConstraint);
						npConstraint->setIsActive(true);
						fConstraint->setIsActive(true);
					}
				}
			}

			//deactivate all unused constraints
			for (unsigned int i = currentConstraintId; i < m_collisionConstraints.size(); ++i)
				m_collisionConstraints[i]->setIsActive(false);
		}

}