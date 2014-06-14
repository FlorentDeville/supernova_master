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

#ifndef FSM_H
#define FSM_H

#include <map>
using std::map;

namespace Devil
{
	namespace FSM
	{
		class IState;

		class FSMRunner
		{
		private:
			//Map of states
			map<unsigned int, IState*> m_states;

			//The state the fsm is currently on
			unsigned int m_currentState;

			//The next state to apply.
			unsigned int m_deferredState;

			//Indicate if the fsm has to exit from the current state and goes to the deferred state.
			bool m_hasToExitState;

			//Indicate if the fsm has to considered the current state as a newly entered state.
			bool m_hasToEnterState;

		public:
			FSMRunner();
			~FSMRunner();

			//Add a state to the fsm.
			void addState(unsigned int _id, IState* _state);

			//Change the current state immediately.
			void setImmediateState(unsigned int _id);

			//Change the current state to the value given as parameter when the next update will be called.
			void setDeferredState(unsigned int _id);

			//Set the current state  to the state _id without exiting from any previous state.
			void setInitialState(unsigned int _id);

			void update();
		};
	}
}

#endif //ifndef FSM_H