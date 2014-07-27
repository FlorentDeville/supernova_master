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

#ifndef KEYBOARD_MOUSE_H
#define KEYBOARD_MOUSE_H

#include "IDevice.h"
#include "CooldownManager.h"

namespace Devil
{
	namespace Input
	{
		namespace Device
		{
			class KeyboardMouse : public IDevice
			{
			private:
				//Flag to indicate if the mouse left button is down
				bool m_mouseLeftButtonDown;

				//The position of the mouse on the x axis the first time the left button was down.
				int m_mouseLeftButtonDownX;

				//The position of the mouse on the y axis the first time the left button was down.
				int m_mouseLeftButtonDownY;

				//The state of the mouse wheel.
				float m_mouseWheelState;

				//Handle the cooldown for the toggle collision point message
				CooldownManager m_cooldownToggleCollisionPoint;

				//Handle the cooldown for shooting message
				CooldownManager m_cooldownShoot;

				//Handle the cooldown for toggling the render mode.
				CooldownManager m_cooldownRenderMode;

				//Handle the cooldown for toggling the watch window.
				CooldownManager m_cooldownToggleWatchWindow;

			public:
				KeyboardMouse();

				virtual ~KeyboardMouse();

				//Update the current state of the device.
				void update();

				//Set the state of the mouse wheel
				void setMouseWheelState(float _state);
			};
		}
	}
}
#endif //ifndef KEYBOARD_MOUSE_H