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

#ifndef SCENE_MANAGER_H
#define SCENE_MANAGER_H

#include <malloc.h>

#include "snCollisionMode.h"

namespace Supernova
{
	class snScene;
}

#include "snVec.h"
using Supernova::snVec;
using Supernova::snScene;
using Supernova::snCollisionMode;

#include <DirectXMath.h>
using DirectX::XMFLOAT4;

#include <string>
using std::wstring;

namespace Devil
{
	class IComponent;
	class EntityComposite;

	class SceneManager
	{
	private:
		static SceneManager* m_instance;

		//The collision mode to use when creating a scene.
		snCollisionMode m_collisionMode;

		//Array of colors
		XMFLOAT4 m_colors[5];

		IComponent* m_dominoHammerBlockerPath;

	public:
		static SceneManager* getInstance();

		bool initialize();
		void shutdown();

		void update();

		//Scene with simple boxes interactions.
		void createBasicTest();

		//Box Stacking
		void createStacking();

		//Constraints (rope)
		void createConstraints();

		//Stack
		void createTower();

		void createSceneFriction();

		void createSceneDamping();

		//Show difference between static, dynamic and kinematic
		void createSceneActorsType();

		void createSceneDomino();

		void createSceneComposite();

		void createSceneMonkeyBall();

		void createSceneGJK();

		void createSceneTerrain();

		void setCollisionMode(snCollisionMode _collisionMode);
		
		void createGround(snScene* const _scene, float _restitution, float _friction);

		void activateDominoSceneHammerBlocker();

	private:
		SceneManager();
		virtual ~SceneManager();

		void clearScene() const;

		snVec createTowerLevel(snScene* const _scene, const snVec& _origin) const;

		//Create the bisic actor of the world.
		void createSandbox(const std::wstring& _sceneName);

		EntityComposite* createWheel(const snVec& _position, const snVec& _orientation, const XMFLOAT4& _color, float _length);
	};

#define SCENEMGR SceneManager::getInstance()

}

#endif //ifndef SCENE_MANAGER_H