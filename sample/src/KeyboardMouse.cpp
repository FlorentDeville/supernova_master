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

#include "KeyboardMouse.h"

#include <Windows.h>
#include <Windowsx.h>

#include "snTimer.h"
using namespace Supernova;

namespace Devil
{
	namespace Input
	{
		namespace Device
		{
			KeyboardMouse::KeyboardMouse() : m_cooldownToggleCollisionPoint(200), m_cooldownShoot(200), m_cooldownRenderMode(200),
				m_cooldownToggleWatchWindow(200), m_mouseWheelState(0)
			{
				for (int i = 0; i < MESSAGE_COUNT; ++i)
				{
					m_messages.push_back(0);
				}

				m_isConnected = true;
				m_mouseLeftButtonDown = false;
			}

			KeyboardMouse::~KeyboardMouse()
			{}

			void KeyboardMouse::update()
			{
				m_messageCount = 0;

				if ((GetKeyState('W') & 0x8000) && m_cooldownToggleWatchWindow.cooldownElapsed())
				{
					m_messages[InputMessage::TOGGLE_WATCH_WINDOW] = 1;
					++m_messageCount;
				}
				else
				{
					m_messages[InputMessage::TOGGLE_WATCH_WINDOW] = 0;
				}

				if ((GetKeyState('R') & 0x8000) && m_cooldownRenderMode.cooldownElapsed())
				{
					m_messages[InputMessage::TOGGLE_RENDER_MODE] = 1;
					++m_messageCount;
				}
				else
				{
					m_messages[InputMessage::TOGGLE_RENDER_MODE] = 0;
				}

				if ((GetKeyState('C') & 0x8000) && m_cooldownToggleCollisionPoint.cooldownElapsed())
				{
					m_messages[InputMessage::TOGGLE_COLLISION_POINTS] = 1;
					++m_messageCount;
				}
				else
				{
					m_messages[InputMessage::TOGGLE_COLLISION_POINTS] = 0;
				}

				if ((GetKeyState(' ') & 0x8000) && m_cooldownShoot.cooldownElapsed())
				{
					m_messages[InputMessage::SHOOT] = 1;
					++m_messageCount;
				}
				else
				{
					m_messages[InputMessage::SHOOT] = 0;
				}

				//Move up/down
				if (GetKeyState('Z') & 0x8000)
				{
					m_messages[InputMessage::MOVE_UP_AND_DOWN] = 1;
					++m_messageCount;
				}
				else if (GetKeyState('S') & 0x8000)
				{
					m_messages[InputMessage::MOVE_UP_AND_DOWN] = -1;
					++m_messageCount;
				}
				else
				{
					m_messages[InputMessage::MOVE_UP_AND_DOWN] = 0;
				}

				//Move left/right
				if (GetKeyState('Q') & 0x8000)
				{
					m_messages[InputMessage::MOVE_SIDEWAY] = -1;
					++m_messageCount;
				}
				else if (GetKeyState('D') & 0x8000)
				{
					m_messages[InputMessage::MOVE_SIDEWAY] = 1;
					++m_messageCount;
				}
				else
				{
					m_messages[InputMessage::MOVE_SIDEWAY] = 0;
				}

				//left button down
				if (GetKeyState(VK_LBUTTON) & 0x8000)
				{
					DWORD position = GetMessagePos();
					short x = GET_X_LPARAM(position);
					short y = GET_Y_LPARAM(position);

					//First time we press the left button so initialize the position
					if (!m_mouseLeftButtonDown)
					{
						m_mouseLeftButtonDown = true;
						m_mouseLeftButtonDownX = x;
						m_mouseLeftButtonDownY = y;
					}
				
					//turn camera left or right
					if (x > m_mouseLeftButtonDownX)
					{
						m_messages[InputMessage::TURN_SIDEWAYS] = 1;
						++m_messageCount;
					}
					else if (x < m_mouseLeftButtonDownX)
					{
						m_messages[InputMessage::TURN_SIDEWAYS] = -1;
						++m_messageCount;
					}
					else
						m_messages[InputMessage::TURN_SIDEWAYS] = 0;

					//turn camera up or down
					if (y > m_mouseLeftButtonDownY)
					{
						m_messages[InputMessage::TURN_UP_AND_DOWN] = -1;
						++m_messageCount;
					}
					else if (y < m_mouseLeftButtonDownY)
					{
						m_messages[InputMessage::TURN_UP_AND_DOWN] = 1;
						++m_messageCount;
					}
					else
						m_messages[InputMessage::TURN_UP_AND_DOWN] = 0;

					m_mouseLeftButtonDownX = x;
					m_mouseLeftButtonDownY = y;
				}
				else
				{
					m_mouseLeftButtonDown = false;
				}

				//Move forward/backward
				if (m_mouseWheelState > 0)
				{
					m_messages[InputMessage::MOVE_FORWARD] = 1;
					++m_messageCount;
				}
				else if (m_mouseWheelState < 0)
				{
					m_messages[InputMessage::MOVE_FORWARD] = -1;
					++m_messageCount;
				}
				else
				{
					m_messages[InputMessage::MOVE_FORWARD] = 0;
				}
				m_mouseWheelState = 0;
			}

			void KeyboardMouse::setMouseWheelState(float _state)
			{
				m_mouseWheelState = _state;
			}
		}
	}
}