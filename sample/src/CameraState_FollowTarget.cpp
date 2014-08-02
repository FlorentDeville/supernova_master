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

#include "CameraState_FollowTarget.h"
#include "World.h"
#include "IWorldEntity.h"
#include "EntityCamera.h"

#include <DirectXMath.h>
using namespace DirectX;

#include "snIActor.h"
using Supernova::snIActor;

#include "snWorld.h"
#include "snScene.h"
#include "snDebugger.h"
#include "snSphere.h"
using namespace Supernova;

namespace Devil
{
	CameraState_FollowTarget::CameraState_FollowTarget(EntityCamera* _camera, IWorldEntity* _target, float _distance, float _height)
		:m_camera(_camera), m_target(_target), m_distance(_distance), m_height(_height){}

	CameraState_FollowTarget::~CameraState_FollowTarget(){}

	void CameraState_FollowTarget::enter()
	{
		m_target = (IWorldEntity*)WORLD->getMonkeyBall();

		//Use the target velocity as forward vector
		float speed = Supernova::Vector::snVec3Norme(m_target->getActor()->getLinearVelocity());
		if (speed > 0.1f)
		{
			m_forward = m_target->getActor()->getLinearVelocity();
			Supernova::Vector::snVec4SetY(m_forward, 0);
			Supernova::Vector::snVec3Normalize(m_forward);
		}
		else //if the target doesn't move, use the X vector as forward vector
		{
			m_forward = XMVectorSet(1, 0, 0, 0);
		}

		m_positionDamper.setCurrentValue(m_camera->getPosition());
		m_positionDamper.setSpeed(100.f);

		m_lookAtDamper.setCurrentValue(m_camera->getLookAt());
		m_lookAtDamper.setSpeed(100.f);
	}

	void CameraState_FollowTarget::execute()
	{
		XMVECTOR targetPosition = m_target->getActor()->getPosition();

		//Use the target velocity as forward vector
		float speed = Supernova::Vector::snVec3Norme(m_target->getActor()->getLinearVelocity());
		if (speed > 0.1f)
		{
			snVec newForward = m_target->getActor()->getLinearVelocity();
			Supernova::Vector::snVec4SetY(newForward, 0);
			Supernova::Vector::snVec3Normalize(newForward);

			if (Supernova::Vector::snVec3Norme(newForward) != 0)
			{
				m_forward = newForward;
			}
		}

		XMVECTOR up = XMVectorSet(0, 1, 0, 0);

		XMVECTOR cameraPosition = targetPosition - (m_forward * m_distance) + (up * m_height);

		snVec dir = cameraPosition - targetPosition;
		float length = Vector::snVec3Norme(dir);
		Vector::snVec3Normalize(dir);

		snSphere s(1);
		snTransform sphereOrigin(targetPosition);
		float distance = 0;
		bool res = false;//SUPERNOVA->getScene(0)->shapeCast(s, sphereOrigin, dir, length, distance);//SUPERNOVA->getScene(0)->sphereCast(targetPosition, 0.5f, dir, length);
		if (res)
		{
			DEBUGGER->setWatchExpression(L"Sphere Cast", std::to_wstring(distance));
		}
		else
		{
			DEBUGGER->setWatchExpression(L"Sphere Cast", L"false");
		}

		m_positionDamper.setIdealValue(cameraPosition);
		cameraPosition = m_positionDamper.computeValue(WORLD->getDeltaTime());

		m_camera->setPosition(cameraPosition);

		m_lookAtDamper.setIdealValue(targetPosition);
		m_camera->setLookAt(m_lookAtDamper.computeValue(WORLD->getDeltaTime()));
		m_camera->setUp(up);
	}

	void CameraState_FollowTarget::exit(){}

	void* CameraState_FollowTarget::operator new(size_t _count)
	{
		return _aligned_malloc(_count, 16);
	}

	void CameraState_FollowTarget::operator delete(void* _p)
	{
		_aligned_free(_p);
	}
}