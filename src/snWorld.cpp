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
#include "snWorld.h"
#include "snScene.h"

#ifdef SN_DEBUGGER
#include "snDebugger.h"
#endif //ifdef SN_DEBUGGER

namespace Supernova
{
	snWorld* snWorld::m_instance = 0;

	snWorld::snWorld()
	{
		m_key = 0;
	}

	snWorld::~snWorld()
	{
		//Delete all the element in the look up table.
		while (m_lookUpTable.size() != 0)
		{
			map<snObjectId, snObject*>::iterator firstElement = m_lookUpTable.begin();
			if (firstElement->second != 0)
			{
				delete firstElement->second;
			}
			else 
			{
				//This is weird, all destructor of snObject should remove themselves from the look up table 
				//so we shouldn't have a null element here. Remove anyway.

				m_lookUpTable.erase(firstElement);
			}
		}
	}

	snWorld* snWorld::getInstance()
	{
		if (m_instance == 0)
			m_instance = new snWorld();

		return m_instance;
	}

	bool snWorld::initialize()
	{
		return true;
	}

	bool snWorld::clean()
	{
		if (m_instance != 0)
		{
			delete m_instance;
			m_instance = 0;
		}

#ifdef SN_DEBUGGER
		//close the debugger
		DEBUGGER->shutdown();
#endif //ifdef SN_DEBUGGER

		return true;
	}

	void* snWorld::getObject(snObjectId _id) const
	{
		map<snObjectId, snObject*>::const_iterator i = m_lookUpTable.find(_id);
		if (i == m_lookUpTable.end())
			return 0;

		return i->second;
	}

	snhScene snWorld::createScene()
	{
		//Create the new scene.
		snScene* newScene = new snScene(m_key);

		//Add the scene to the look up table.
		//Use the insert version. Its not as clear as the [] operator but its faster.
		if (m_lookUpTable.size() == 0)
			m_lookUpTable.insert(m_lookUpTable.begin(), std::pair<snObjectId, snObject*>(m_key, newScene));
		else
			m_lookUpTable.insert(--m_lookUpTable.end(), std::pair<snObjectId, snObject*>(m_key, newScene));

		//Return the handle
		snhScene handle(m_key);

		//Increase the key. This is not thread safe!!!!!
		++m_key;

		return handle;
	}

	void snWorld::removeObject(snObjectId _id)
	{
		m_lookUpTable.erase(_id);
	}

	void snWorld::deleteScene(snhScene _hScene)
	{
		//Get the underlying pointer
		snScene* scene = _hScene.getPtr();
		if (scene == 0)
			return;

		//Delete the object
		delete scene;
	}

	void snWorld::deleteScene(snObjectId _id)
	{
		snScene* scene = static_cast<snScene*>(getObject(_id));
		if (scene == 0)
			return;

		delete scene;
	}

	void* snWorld::operator new(size_t _count)
	{
		return _aligned_malloc(_count, SN_ALIGN_SIZE);
	}

	void snWorld::operator delete(void* _p)
	{
		_aligned_free(_p);
	}
}