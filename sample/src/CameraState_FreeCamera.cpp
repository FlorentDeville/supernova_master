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
#include "CameraState_FreeCamera.h"

#include "snVec.h"
#include <DirectXMath.h>
using namespace DirectX;

#include "Input.h"
#include "EntityCamera.h"

namespace Devil
{
	CameraState_FreeCamera::CameraState_FreeCamera(EntityCamera* _camera) : m_camera(_camera){}

	CameraState_FreeCamera::~CameraState_FreeCamera(){}

	void CameraState_FreeCamera::enter(){}

	void CameraState_FreeCamera::execute()
	{
		const float linearCameraSpeed = 10.75f;
		const float angularCameraSpeed = 0.025f;

		//forward vector
		XMVECTOR forward = m_camera->getLookAt() - m_camera->getPosition();
		forward = XMVector3Normalize(forward);
		forward = XMVectorSetW(forward, 0);

		//left vector
		XMVECTOR left = XMVector3Cross(forward, m_camera->getUp());
		left = XMVectorSetW(left, 0);

		//up vector
		XMVECTOR up = XMVector3Cross(left, forward);
		up = XMVectorSetW(up, 0);

		const int WHEEL_SPEED = 4;
		float amount = INPUT->getMessage(Devil::Input::InputMessage::MOVE_FORWARD);
		if (amount != 0)
		{
			forward = forward * linearCameraSpeed * WHEEL_SPEED * amount;
			m_camera->setPosition(m_camera->getPosition() + forward);
			m_camera->setLookAt(m_camera->getLookAt() + forward);
		}

		amount = INPUT->getMessage(Devil::Input::InputMessage::MOVE_UP_AND_DOWN);
		if (amount != 0)
		{
			XMVECTOR offset = up * linearCameraSpeed * amount;
			m_camera->setPosition(m_camera->getPosition() + offset);
			m_camera->setLookAt(m_camera->getLookAt() + offset);
		}

		amount = INPUT->getMessage(Devil::Input::InputMessage::MOVE_SIDEWAY);
		if (amount != 0)
		{
			left = left * linearCameraSpeed * amount;
			m_camera->setPosition(m_camera->getPosition() - left);
			m_camera->setLookAt(m_camera->getLookAt() - left);
		}

		amount = INPUT->getMessage(Devil::Input::InputMessage::TURN_SIDEWAYS);
		if (amount != 0)
		{
			XMMATRIX rotation = XMMatrixRotationAxis(up, angularCameraSpeed * amount);
			forward = XMVector3Transform(forward, rotation);
			m_camera->setLookAt(m_camera->getPosition() + forward);
		}

		left = XMVector3Cross(forward, m_camera->getUp());

		amount = INPUT->getMessage(Devil::Input::InputMessage::TURN_UP_AND_DOWN);
		if (amount != 0)
		{
			XMMATRIX rotation = XMMatrixRotationAxis(left, angularCameraSpeed * amount);
			forward = XMVector3Transform(forward, rotation);
			m_camera->setLookAt(m_camera->getPosition() + forward);
		}

		XMVECTOR l = m_camera->getLookAt();
		l = XMVectorSetW(l, 1);
		m_camera->setLookAt(l);
	}

	void CameraState_FreeCamera::exit(){}
}