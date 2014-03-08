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
		m_angularSquaredSpeedThreshold(0.001f)
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

	void snScene::clearScene()
	{
		for (vector<snActor*>::iterator i = m_actors.begin(); i != m_actors.end(); ++i)
			delete *i;
		m_actors.clear();
	}

	void snScene::update(float _deltaTime)
	{
		//compute collision points
		m_contactsPoints.clear();
		getContactPoints(m_contactsPoints, _deltaTime);

		//apply forces
		applyForces(_deltaTime);

		//apply impulses
		sequentialImpulseSIMD(m_contactsPoints);

		//update positions
		updatePosition(_deltaTime);	
	}

	const snContactPointVector& snScene::getContactsPoints() const
	{
		return m_contactsPoints;
	}

	snSequentialImpulse& snScene::getSolver()
	{
		return m_sequentialImpulseSolver;
	}

	void snScene::setBeta(float _beta)
	{
		m_beta = _beta;
	}

	void snScene::setMaxSlop(float _maxSlop)
	{
		m_maxSlop = _maxSlop;
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

	void* snScene::operator new(size_t _count)
	{
		return _aligned_malloc(_count, SN_ALIGN_SIZE);
	}

	void snScene::operator delete(void* _p)
	{
		_aligned_free(_p);
	}

	void snScene::getContactPoints(snContactPointVector& _contacts, float _dt)
	{
#if _DEBUG
		LARGE_INTEGER frequency;
		QueryPerformanceFrequency(&frequency);

		float tickPerMilliseconds = (float)frequency.QuadPart * 0.001f;
		float tickPerMicroseconds = tickPerMilliseconds * 0.001f;
#endif

		_contacts.clear();

		if (m_actors.size() <= 1)
			return;

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

					snContactPoint NewPoint;
					NewPoint.m_point = *point;
					NewPoint.m_penetration = *penetrationIterator;
					NewPoint.m_normal = res.m_normal;
					NewPoint.m_bodies[0] = *i;
					NewPoint.m_bodies[1] = *j;

					//compute relative velocity
					NewPoint.m_ra = *point - (*i)->getPosition();
					snVector4f v1 = (*i)->getLinearVelocity() + (*i)->getAngularVelocity().cross(NewPoint.m_ra);
					NewPoint.m_rb = *point - (*j)->getPosition();
					snVector4f v2 = (*j)->getLinearVelocity() + (*j)->getAngularVelocity().cross(NewPoint.m_rb);
					snVector4f relVel = v2 - v1;
					NewPoint.m_preImpRelSpeed = NewPoint.m_normal.dot(relVel);
					NewPoint.m_accumulatedImpulseMag = 0;
					NewPoint.m_accumulatedFrictionImpulse1 = 0;
					NewPoint.m_accumulatedFrictionImpulse2 = 0;

					//compute r cross n
					NewPoint.m_rACrossN = NewPoint.m_ra.cross(NewPoint.m_normal);
					NewPoint.m_rBCrossN = NewPoint.m_rb.cross(NewPoint.m_normal);

					//Compute the effective mass for the non penetration constraint
					// (r X N) I-1
					NewPoint.m_raCrossNInvI = snMatrixTransform3(NewPoint.m_rACrossN, (*i)->getInvWorldInertia()); 
					NewPoint.m_rbCrossNInvI = snMatrixTransform3(NewPoint.m_rBCrossN, (*j)->getInvWorldInertia());

					// [(r X N)I-1] X r
					snVector4f tempA = NewPoint.m_raCrossNInvI.cross(NewPoint.m_ra);
					snVector4f tempB = NewPoint.m_rbCrossNInvI.cross(NewPoint.m_rb);

					float sumInvMass = (*i)->getInvMass() + (*j)->getInvMass();

					// 1/ ( 1/ma + 1/mb + ( [(ra X n)Ia-1] X ra + [(rb X n)Ib-1] X rb) . N)
					NewPoint.m_normalEffectiveMass = 1.f / (sumInvMass +(tempA + tempB).dot(NewPoint.m_normal));

					//compute the bias
					float restitution = ((*i)->getPhysicMaterial().m_restitution + (*j)->getPhysicMaterial().m_restitution) * 0.5f;
					NewPoint.m_bias = - restitution * NewPoint.m_preImpRelSpeed - m_beta / _dt * max<float>(0, NewPoint.m_penetration - m_maxSlop);

					//compute tangent vectors. They make an orthonormal basis with the normal
					computeBasis(NewPoint.m_normal, NewPoint.m_tangent1, NewPoint.m_tangent2);

					//compute the effective mass along the first tangent vector
					tempA = NewPoint.m_ra.cross(NewPoint.m_tangent1);
					NewPoint.m_raCrossT1InvI = snMatrixTransform3(tempA, (*i)->getInvWorldInertia());
					tempB = NewPoint.m_rb.cross(NewPoint.m_tangent1);
					NewPoint.m_rbCrossT1InvI = snMatrixTransform3(tempB, (*j)->getInvWorldInertia());
					tempA.setW(0);
					tempB.setW(0);
					NewPoint.m_tangent1EffectiveMass = 1.f / ( sumInvMass + 
						(NewPoint.m_raCrossT1InvI.cross(NewPoint.m_ra) + NewPoint.m_rbCrossT1InvI.cross(NewPoint.m_rb)).dot(NewPoint.m_tangent1));

					//compute the effective mass along the second tangent vector
					tempA = NewPoint.m_ra.cross(NewPoint.m_tangent2);
					NewPoint.m_raCrossT2InvI = snMatrixTransform3(tempA, (*i)->getInvWorldInertia());
					tempB = NewPoint.m_rb.cross(NewPoint.m_tangent2);
					NewPoint.m_rbCrossT2InvI = snMatrixTransform3(tempB, (*j)->getInvWorldInertia());
					tempA.setW(0);
					tempB.setW(0);
					NewPoint.m_tangent2EffectiveMass = 1.f / (sumInvMass +
						(NewPoint.m_raCrossT2InvI.cross(NewPoint.m_ra) + NewPoint.m_rbCrossT2InvI.cross(NewPoint.m_rb)).dot(NewPoint.m_tangent2));

					//Compute the friction coefficient as the average of frictions of the two objects.
					NewPoint.m_frictionCoefficient = ((*i)->getPhysicMaterial().m_friction + (*j)->getPhysicMaterial().m_friction) * 0.5f;
					_contacts.push_back(NewPoint);
				}
			}
		}
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

	void snScene::sequentialImpulseSIMD(snContactPointVector& _contacts) const
	{
		m_sequentialImpulseSolver.solve(_contacts);
	}
}