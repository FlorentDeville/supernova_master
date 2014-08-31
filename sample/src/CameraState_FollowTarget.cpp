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

#include "snRigidbody.h"
using Supernova::snRigidbody;

#include "snWorld.h"
#include "snScene.h"
#include "snRay.h"
#include "snDebugger.h"
#include "snSphere.h"
using namespace Supernova;

#include "Input.h"

namespace Devil
{
	CameraState_FollowTarget::CameraState_FollowTarget(EntityCamera* _camera)
		:m_camera(_camera), m_target(0), m_distance(0), m_height(0), m_angleY(0){}

	CameraState_FollowTarget::~CameraState_FollowTarget(){}

	void CameraState_FollowTarget::enter()
	{
		m_forward = XMVectorSet(0, 0, 1, 0);
		m_angleY = 0;

		m_positionDamper.setCurrentValue(m_camera->getPosition());
		m_positionDamper.setSpeed(1000.f);

		m_lookAtDamper.setCurrentValue(m_camera->getLookAt());
		m_lookAtDamper.setSpeed(1000.f);
	}

	void CameraState_FollowTarget::execute()
	{
		//Update the rotation
		float amount = INPUT->getMessage(Devil::Input::InputMessage::TURN_SIDEWAYS);
		float angularSpeed = 0.1f;
		m_angleY += amount * angularSpeed;
		const float TWO_PI = 3.14157f * 2;
		if(m_angleY > TWO_PI)
		{
			m_angleY -= TWO_PI;
		}
		else if(m_angleY < -TWO_PI)
		{
			m_angleY += TWO_PI;
		}

		amount = INPUT->getMessage(Devil::Input::InputMessage::TURN_UP_AND_DOWN);
		m_height -= amount;

		//Rotate the forward vector
		snMatrix44f forwardRotation;
		forwardRotation.createRotationY(m_angleY);
		snVec forward = snMatrixTransform3(Vector::snVec4Set(0, 0, 1, 0), forwardRotation);
		XMVECTOR up = XMVectorSet(0, 1, 0, 0);

		//Compute the position
		XMVECTOR cameraPosition = m_target->getPosition() - (forward * m_distance) + (up * m_height);
		m_positionDamper.setIdealValue(cameraPosition);
		cameraPosition = m_positionDamper.computeValue(WORLD->getDeltaTime());

		//Check for obstacles
		snVec rayStart = m_target->getPosition() + Vector::snVec4Set(0, 1, 0, 0) * 4;
		snVec dir = cameraPosition - rayStart;
		float sqCameraDistance = Vector::snVec3SquaredNorme(dir);
		Vector::snVec3Normalize(dir);

		snRay ray(rayStart, dir);
		snVec hit;
		const float cameraRadius = 2.f;
		if(WORLD->getPhysicsScene()->raycast(ray, hit))
		{
			float sqObstacleDistance = Vector::snVec3SquaredNorme(rayStart - hit);
			if(sqObstacleDistance < sqCameraDistance)
			{
				cameraPosition = hit + (Vector::snVec4Set(0, 1, 0, 0) - dir) * cameraRadius;
			}
		}

		
		//Set final camera position
		m_camera->setPosition(cameraPosition);

		//Comput the lookat
		m_lookAtDamper.setIdealValue(m_target->getPosition());
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

	void CameraState_FollowTarget::setup(IWorldEntity const * const _target, float _distance, float _height)
	{
		m_target = _target;
		m_distance = _distance;
		m_height = _height;
	}
}