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
#include "EntityCamera.h"
#include "Graphics.h"
#include "Camera.h"

#include "World.h"
#include "Input.h"

#include "snVec.inl"

using namespace DirectX;

#include "CameraState_FreeCamera.h"
#include "CameraState_FollowTarget.h"
#include "CameraState.h"

namespace Devil
{

	EntityCamera::EntityCamera()
	{
	}


	EntityCamera::~EntityCamera()
	{
	}

	bool EntityCamera::initialize(const XMVECTOR& _position, const XMVECTOR& _lookAt, const XMVECTOR& _up)
	{
		m_gfxCamera = GRAPHICS->getCamera();
		m_position = _position;
		m_lookAt = _lookAt;
		m_up = _up;

		//create the fsm
		CameraState_FreeCamera* stateFreeCamera = new CameraState_FreeCamera(this);
		m_fsmRunner.addState(CAMERA_STATE_FREE_CAMERA, stateFreeCamera);
		
		CameraState_FollowTarget* stateFollowTarget = new CameraState_FollowTarget(this, (IWorldEntity*)WORLD->getMonkeyBall(), 80, 40);
		m_fsmRunner.addState(CAMERA_STATE_FOLLOW_TARGET, stateFollowTarget);

		//m_fsmRunner.setImmediateState(CAMERA_STATE_FREE_CAMERA);
		m_fsmRunner.setImmediateState(CAMERA_STATE_FOLLOW_TARGET);
		return true;
	}

	void EntityCamera::update()
	{
		m_fsmRunner.update();
	}

	void EntityCamera::render()
	{
		m_gfxCamera->Render(m_position, m_lookAt, m_up);
	}

	void EntityCamera::setLookAt(const XMVECTOR& _lookAt)
	{
		m_lookAt = _lookAt;
	}

	void EntityCamera::setUp(const XMVECTOR& _up)
	{
		m_up = _up;
	}

	const XMVECTOR& EntityCamera::getLookAt() const
	{
		return m_lookAt;
	}

	const XMVECTOR& EntityCamera::getUp() const
	{
		return m_up;
	}
}