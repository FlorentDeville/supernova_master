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

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <XInput.h>
#pragma comment(lib,"xinput9_1_0.lib")

#include "X360Controller.h"

namespace Devil
{
	namespace Input
	{
		namespace Device
		{
			X360Controller::X360Controller() : m_cooldownToggleCollisionPoint(200), m_cooldownShoot(200), m_cooldownRenderMode(200),
				m_cooldownToggleWatchWindow(200)
			{
				for (int i = 0; i < MESSAGE_COUNT; ++i)
				{
					m_messages.push_back(0);
				}
			}

			X360Controller::~X360Controller(){}

			void X360Controller::update()
			{
				//Get the state of the controller
				XINPUT_STATE controllerState;
				DWORD res = XInputGetState(0, &controllerState);
				m_messageCount = 0;
				//Check if the controller is connected
				if (res != ERROR_SUCCESS)
				{
					m_isConnected = false;
					return;
				}
				else
					m_isConnected = true;

				const float INPUT_DEADZONE = (0.24f * FLOAT(0x7FFF)); // Default to 24% of the +/- 32767 range.
					
				//Left stick, left/right
				if (controllerState.Gamepad.sThumbLX > INPUT_DEADZONE ||
					controllerState.Gamepad.sThumbLX < -INPUT_DEADZONE)
				{
					float value = controllerState.Gamepad.sThumbLX / float(0x7FFF);
					m_messages[InputMessage::MOVE_SIDEWAY] = value;
					++m_messageCount;
				}
				else
				{
					m_messages[InputMessage::MOVE_SIDEWAY] = 0;
				}

				//Left stick, up/down
				if (controllerState.Gamepad.sThumbLY > INPUT_DEADZONE ||
					controllerState.Gamepad.sThumbLY < -INPUT_DEADZONE)
				{
					float value = controllerState.Gamepad.sThumbLY / float(0x7FFF);
					m_messages[InputMessage::MOVE_UP_AND_DOWN] = value;
					++m_messageCount;
				}
				else
				{
					m_messages[InputMessage::MOVE_UP_AND_DOWN] = 0;
				}

				//Right stick, left/right
				if (controllerState.Gamepad.sThumbRX > INPUT_DEADZONE ||
					controllerState.Gamepad.sThumbRX < -INPUT_DEADZONE)
				{
					float value = controllerState.Gamepad.sThumbRX / float(0x7FFF);
					m_messages[InputMessage::TURN_SIDEWAYS] = value;
					++m_messageCount;
				}
				else
				{
					m_messages[InputMessage::TURN_SIDEWAYS] = 0;
				}

				//Right stick, left/right
				if (controllerState.Gamepad.sThumbRY > INPUT_DEADZONE ||
					controllerState.Gamepad.sThumbRY < -INPUT_DEADZONE)
				{
					float value = controllerState.Gamepad.sThumbRY / float(0x7FFF);
					m_messages[InputMessage::TURN_UP_AND_DOWN] = value;
					++m_messageCount;
				}
				else
				{
					m_messages[InputMessage::TURN_UP_AND_DOWN] = 0;
				}
	
				if (controllerState.Gamepad.bLeftTrigger > 0)
				{
					m_messages[InputMessage::MOVE_FORWARD] = -(float)controllerState.Gamepad.bLeftTrigger / 255.f;
					++m_messageCount;
				}
				else if (controllerState.Gamepad.bRightTrigger > 0)
				{
					m_messages[InputMessage::MOVE_FORWARD] = (float)controllerState.Gamepad.bRightTrigger / 255.f;
					++m_messageCount;
				}
				else
				{
					m_messages[InputMessage::MOVE_FORWARD] = 0;
				}

				WORD wButtons = controllerState.Gamepad.wButtons;

				if ((wButtons & XINPUT_GAMEPAD_A) && m_cooldownToggleCollisionPoint.cooldownElapsed())
				{
					m_messages[InputMessage::TOGGLE_COLLISION_POINTS] = 1;
					++m_messageCount;
				}
				else
				{
					m_messages[InputMessage::TOGGLE_COLLISION_POINTS] = 0;
				}

				if ((wButtons & XINPUT_GAMEPAD_X) && m_cooldownShoot.cooldownElapsed())
				{
					m_messages[InputMessage::SHOOT] = 1;
					++m_messageCount;
				}
				else
				{
					m_messages[InputMessage::SHOOT] = 0;
				}

				if ((wButtons & XINPUT_GAMEPAD_B) && m_cooldownRenderMode.cooldownElapsed())
				{
					m_messages[InputMessage::TOGGLE_RENDER_MODE] = 1;
					++m_messageCount;
				}
				else
				{
					m_messages[InputMessage::TOGGLE_RENDER_MODE] = 0;
				}

				if ((wButtons & XINPUT_GAMEPAD_Y) && m_cooldownToggleWatchWindow.cooldownElapsed())
				{
					m_messages[InputMessage::TOGGLE_WATCH_WINDOW] = 1;
					++m_messageCount;
				}
				else
				{
					m_messages[InputMessage::TOGGLE_WATCH_WINDOW] = 0;
				}
			}
		}
	}
}