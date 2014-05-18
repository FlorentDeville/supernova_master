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

#include "WorldHUD.h"
#include "Graphics.h"
#include "snFactory.h"
#include "snScene.h"
#include "Input.h"

#ifdef SN_DEBUGGER
#include "snDebugger.h"
#endif //ifdef SN_DEBUGGER

namespace Devil
{
	WorldHUD::WorldHUD() : m_showWatcher(true)
	{

	}

	WorldHUD::~WorldHUD()
	{

	}

	void WorldHUD::update(){}

	void WorldHUD::render(){}

	void WorldHUD::spriteRender()
	{
		const float SCALE = 0.4f;
		const int LINE_HEIGHT = 17;
		float height = 1;
		GRAPHICS->writeText(L"SUPERNOVA", XMFLOAT2(1, height), 0.5);
		height += LINE_HEIGHT;
		GRAPHICS->writeText(m_sceneName, XMFLOAT2(1, height), 0.5);
		height += LINE_HEIGHT * 2;
		GRAPHICS->writeText(L"Graphics FPS : " + m_graphicsFPS, XMFLOAT2(1, height), 0.5);
		height += LINE_HEIGHT;
		GRAPHICS->writeText(L"Physics FPS : " + m_physicsFPS, XMFLOAT2(1, height), 0.5);
		height += LINE_HEIGHT * 2;

		//display debugging information
#ifdef SN_DEBUGGER

		if (INPUT->isKeyDown('W'))
		{
			m_showWatcher = !m_showWatcher;
			INPUT->keyUp('W');
		}
		
		if (m_showWatcher)
		{

			GRAPHICS->writeText(L"======WATCH======", XMFLOAT2(1, height), SCALE);
			height += LINE_HEIGHT;
			const map<wstring, wstring>& watch = DEBUGGER->getWatch();
			for (map<wstring, wstring>::const_iterator i = watch.cbegin(); i != watch.cend(); ++i)
			{
				GRAPHICS->writeText(i->first + L" : " + i->second, XMFLOAT2(1, height), SCALE);
				height += LINE_HEIGHT;
			}

			switch (SUPERNOVA->getScene(0)->getCollisionMode())
			{
			case snCollisionMode::snECollisionModeBruteForce:
				GRAPHICS->writeText(L"Collision Mode : Brute Force", XMFLOAT2(1, height), SCALE);
				break;

			case snCollisionMode::snECollisionMode_ST_SweepAndPrune:
				GRAPHICS->writeText(L"Collision Mode : Single Threaded Broad Phase", XMFLOAT2(1, height), SCALE);
				break;

			case snCollisionMode::snECollisionMode_MT_SweepAndPrune:
				GRAPHICS->writeText(L"Collision Mode : Multi Threaded Broad Phase", XMFLOAT2(1, height), SCALE);
				break;
			}
		}
#endif //ifdef SN_DEBUGGER

		//display controls
		height = 1;
		float offset = 300;
		GRAPHICS->writeText(L"F1 to F6 to switch scene", XMFLOAT2(GRAPHICS->getScreenWidth() - offset, height), 0.5);
		height += LINE_HEIGHT;
		GRAPHICS->writeText(L"Z, Q, S, D to move around", XMFLOAT2(GRAPHICS->getScreenWidth() - offset, height), 0.5);
		height += LINE_HEIGHT;
		GRAPHICS->writeText(L"Mouse to rotate", XMFLOAT2(GRAPHICS->getScreenWidth() - offset, height), 0.5);
		height += LINE_HEIGHT;
		GRAPHICS->writeText(L"Mouse wheel to zoom in/out", XMFLOAT2(GRAPHICS->getScreenWidth() - offset, height), 0.5);
		height += LINE_HEIGHT;
		GRAPHICS->writeText(L"SPACE to shoot a cube", XMFLOAT2(GRAPHICS->getScreenWidth() - offset, height), 0.5);
		height += LINE_HEIGHT;
		GRAPHICS->writeText(L"C, V to show/hide collision points", XMFLOAT2(GRAPHICS->getScreenWidth() - offset, height), 0.5);
		height += LINE_HEIGHT;
		GRAPHICS->writeText(L"1, 2 to switch collision mode", XMFLOAT2(GRAPHICS->getScreenWidth() - offset, height), 0.5);
		height += LINE_HEIGHT;
		GRAPHICS->writeText(L"W to show/hide watcher", XMFLOAT2(GRAPHICS->getScreenWidth() - offset, height), 0.5);
	}

	void WorldHUD::setSceneName(const wstring _sceneName)
	{
		m_sceneName = _sceneName;
	}

	void WorldHUD::setGraphicsFPS(int _graphicsFPS)
	{
		m_graphicsFPS = std::to_wstring(_graphicsFPS);
	}

	void WorldHUD::setPhysicsFPS(int _physicsFPS)
	{
		m_physicsFPS = std::to_wstring(_physicsFPS);
	}
}