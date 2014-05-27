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

#ifndef SN_DEBUGGER_H
#define SN_DEBUGGER_H

#include <map>
using std::map;

#include <string>
using std::wstring;

#include "snVec.inl"

namespace Supernova
{
	//Singleton storing debugging information accessible from the outside of the engine.
	//In order to Supernova to register information, SN_DEBUGGER needs to be defined.
	class snDebugger
	{
	private:
		//Static and unique instance of the current class. It is a singleton.
		static snDebugger* m_instance;

		//Store debugging information. The key is the name of the information, the value is its value.
		//It's a kind of equivalent to the watch window in visual studio.
		map<wstring, wstring> m_watch;

	public:

		//Return the unique and global instance of the current class.
		static snDebugger* getInstance();

		//Delete the singleton and clean up everything it has allocated.
		void shutdown();

		//Set a value in the watch.
		void setWatchExpression(wstring _name, wstring _value);

		//Set a snVec object into the watcher.
		void setWatchExpression(wstring _name, const snVec& _value);

		//Returns the map containing all the watched values.
		const map<wstring, wstring>& getWatch() const;

		//Clear the content of the watch
		void clearWatch();

	private:
		//Default constructor
		snDebugger();

		//Destructor
		virtual ~snDebugger();
	};

	//Gives access to the debugger
#define DEBUGGER snDebugger::getInstance()

}
#endif //ifndef SN_DEBUGGER_H