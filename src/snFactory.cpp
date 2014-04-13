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
#include "snFactory.h"
#include "snScene.h"

#ifdef SN_DEBUGGER
#include "snDebugger.h"
#endif //ifdef SN_DEBUGGER

namespace Supernova
{
	snFactory* snFactory::m_instance = 0;

	snFactory::snFactory()
	{

	}

	snFactory::~snFactory()
	{
		deleteAllScenes();
	}

	snFactory* snFactory::getInstance()
	{
		if (m_instance == 0)
			m_instance = new snFactory();

		return m_instance;
	}

	bool snFactory::initialize()
	{
		return true;
	}

	bool snFactory::clean()
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

	void snFactory::createScene(snScene** _newScene, int& _sceneId)
	{
		//create the new scene
		*_newScene = new snScene();

		//try to find an empty spot to store the pointer
		for (unsigned int i = 0; i < m_scenes.size(); ++i)
		{
			if (m_scenes[i] != 0)
				continue;

			m_scenes[i] = *_newScene;
			_sceneId = i;
			return;
		}

		//no empty spot found so push back
		_sceneId = m_scenes.size();
		m_scenes.push_back(*_newScene);
		
		return;
	}

	void snFactory::deleteScene(unsigned int _sceneId)
	{
		//the id is out of range
		if (_sceneId >= m_scenes.size())
			return;

		//get the scene
		snScene* scene = m_scenes[_sceneId];

		//the scene is already deleted
		if (scene == 0)
			return;

		//delete the scene
		delete scene;
		m_scenes[_sceneId] = 0;
	}

	void snFactory::deleteAllScenes()
	{
		for (vector<snScene*>::iterator i = m_scenes.begin(); i != m_scenes.end(); ++i)
		{
			if ((*i) == 0)
				continue;

			delete *i;
			*i = 0;
		}

		m_scenes.clear();
	}

	void snFactory::updateAllScenes(float _dt)
	{
		for (vector<snScene*>::iterator i = m_scenes.begin(); i != m_scenes.end(); ++i)
		{
			if ((*i) == 0)
				continue;

			(*i)->update(_dt);
		}
	}

	snScene* snFactory::getScene(unsigned int _sceneId)
	{
		if (_sceneId >= m_scenes.size())
			return 0;

		return m_scenes[_sceneId];
	}

	void* snFactory::operator new(size_t _count)
	{
		return _aligned_malloc(_count, SN_ALIGN_SIZE);
	}

	void snFactory::operator delete(void* _p)
	{
		_aligned_free(_p);
	}
}