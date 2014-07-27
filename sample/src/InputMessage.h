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

#ifndef INPUT_MESSAGE_H
#define INPUT_MESSAGE_H

namespace Devil
{
	namespace Input
	{
		//List of the message the input component is able to handle.
		enum InputMessage
		{
			//Order to move the camera forward. A parameter between -1 and 1 is sent with it.
			MOVE_FORWARD,

			//Strafe to the left or to the right. A parameter between -1 and 1 is sent with it.
			MOVE_SIDEWAY,

			//Move the camera along the Y axis (up and down). A parameter between -1 and 1 is sent with it.
			MOVE_UP_AND_DOWN,

			//Rotate the camera around the local X axis. A parameter between -1 and 1 is sent.
			TURN_UP_AND_DOWN,

			//Rotate the camera around the local Y axis. A parameter between -1 and 1 is sent.
			TURN_SIDEWAYS,

			//Shox/hode collision points
			TOGGLE_COLLISION_POINTS,

			//Shoot a rigid body
			SHOOT,

			//Change the rendering mode.
			TOGGLE_RENDER_MODE,

			//Show/Hide the watch window
			TOGGLE_WATCH_WINDOW,

			//Number of messages available
			MESSAGE_COUNT
		};
	}
}

#endif //ifndef INPUT_MESSAGE_H