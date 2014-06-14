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

#include "FSMRunner.h"
#include "IState.h"

namespace Devil
{
	namespace FSM
	{
		FSMRunner::FSMRunner() : m_currentState(0), m_states(), m_hasToExitState(false), m_hasToEnterState(true), m_deferredState(0){}

		FSMRunner::~FSMRunner()
		{
			for (map<unsigned int, IState*>::iterator i = m_states.begin(); i != m_states.end(); ++i)
			{
				if (i->second != 0)
				{
					delete i->second;
					i->second = 0;
				}
			}
		}

		//Add a state to the fsm.
		void FSMRunner::addState(unsigned int _id, IState* _state)
		{
			m_states[_id] = _state;
			_state->setFSM(this);
		}

		//Change the current state immediately.
		void FSMRunner::setImmediateState(unsigned int _id)
		{
			m_deferredState = _id;
			m_currentState = _id;
			m_hasToExitState = true;
		}

		//Change the current state to the value given as parameter when the next update will be called.
		void FSMRunner::setDeferredState(unsigned int _id)
		{
			m_deferredState = _id;
			m_hasToExitState = true;
		}

		void FSMRunner::setInitialState(unsigned int _id)
		{
			m_deferredState = _id;
			m_currentState = _id;
		}

		void FSMRunner::update()
		{
			if (m_hasToEnterState)
			{
				m_states[m_currentState]->enter();
				m_states[m_currentState]->execute();

				m_hasToEnterState = false;
			}
			else
			{
				m_states[m_currentState]->execute();
			}

			if (m_hasToExitState)
			{
				m_states[m_currentState]->exit();
				m_currentState = m_deferredState;
				m_hasToExitState = false;
				m_hasToEnterState = true;
			}
		}
	}
}