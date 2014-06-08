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

namespace Devil
{
	CameraState_FollowTarget::CameraState_FollowTarget(EntityCamera* _camera, IWorldEntity* _target, float _distance, float _height)
		:m_camera(_camera), m_target(_target), m_distance(_distance), m_height(_height){}

	CameraState_FollowTarget::~CameraState_FollowTarget(){}

	void CameraState_FollowTarget::enter()
	{
		m_target = (IWorldEntity*)WORLD->getMonkeyBall();
	}

	void CameraState_FollowTarget::execute()
	{
		XMVECTOR targetPosition = m_target->getActor()->getPosition();
		XMVECTOR forward = XMVectorSet(1, 0, 0, 0);
		XMVECTOR up = XMVectorSet(0, 1, 0, 0);

		XMVECTOR cameraPosition = targetPosition - (forward * m_distance) + (up * m_height);
		m_camera->setPosition(cameraPosition);
		m_camera->setLookAt(targetPosition);
		m_camera->setUp(up);
	}

	void CameraState_FollowTarget::exit(){}
}