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

#include "SceneManager.h"
#include <assert.h>


#include "World.h"
#include "WorldHUD.h"
#include "Input.h"
#include "Graphics.h"

#include "snFactory.h"
#include "snScene.h"
#include "snActor.h"
#include "snColliderBox.h"
#include "snQuaternion.h"

#include "EntityBox.h"

using namespace Supernova;

namespace Devil
{

	SceneManager* SceneManager::m_instance = 0;

	SceneManager* SceneManager::getInstance()
	{
		if (m_instance == 0)
			m_instance = new SceneManager();

		return m_instance;
	}

	bool SceneManager::initialize()
	{
		return true;
	}

	void SceneManager::shutdown()
	{
		assert(m_instance != 0);

		delete m_instance;
		m_instance = 0;
	}

	void SceneManager::update()
	{
		if (INPUT->isKeyDown(112))//F1
		{
			clearScene();
			createScene1();
			INPUT->keyUp(112);
		}
		else if (INPUT->isKeyDown(113))//F2
		{
			clearScene();
			createScene2();
			INPUT->keyUp(113);
		}
		else if (INPUT->isKeyDown(114))//F3
		{
			clearScene();
			createScene3();
			INPUT->keyUp(114);
		}
		else if (INPUT->isKeyDown(115))//F4
		{
			clearScene();
			createScene4();
			INPUT->keyUp(115);
		}
		else if (INPUT->isKeyDown(116))//F5
		{
			clearScene();
			createTower();
			INPUT->keyUp(116);
		}
		else if (INPUT->isKeyDown(117))//F6
		{
			clearScene();
			createSceneFriction();
			INPUT->keyUp(117);
		}
	}

	void SceneManager::createScene1()
	{
		WORLD->initialize();
		WorldHUD* HUD = WORLD->createHUD();
		HUD->setSceneName(L"Scene 1 : Basic Test");

		GRAPHICS->setClearScreenColor(Colors::DarkGray);
		int sceneId = -1;
		snScene* m_physicScene = 0;
		SUPERNOVA->createScene(&m_physicScene, sceneId);
		m_physicScene->setGravity(snVector4f(0, -9.81f * 2, 0, 0));
		m_physicScene->setCollisionMode(m_collisionMode);

		m_physicScene->setAngularSquaredSpeedThreshold(0.0001f);
		m_physicScene->setLinearSquaredSpeedThreshold(0.005f);
		int solverIterationCount = 5;
		m_physicScene->setSolverIterationCount(solverIterationCount);
		HUD->setSolverIterationCount(solverIterationCount);

		//create camera
		XMVECTOR cameraPosition = XMVectorSet(25, 30, 50, 1);
		XMVECTOR cameraLookAt = XMVectorSet(15, 7, 0, 1);
		XMVECTOR cameraUp = XMVectorSet(0, 1, 0, 0);
		WORLD->createCamera(cameraPosition, cameraLookAt, cameraUp);

		WORLD->deactivateCollisionPoint();
		XMFLOAT4 color1(0.8f, 1, 1, 1);
		XMFLOAT4 color2(0.93f, 0.68f, 1, 1);
		XMFLOAT4 color3(1, 0.8f, 0.678f, 1);
		XMFLOAT4 color4(0.89f, 0.71f, 0.75f, 1);
		XMFLOAT4 color5(0.96f, 0.48f, 0.63f, 1);
		float groundHeight = 0;
		//ground
		{
			float width = 100;
			float height = 2;
			float depth = 100;
			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);
			act->setName("ground");
			act->setMass(100);
			act->setPosition(snVector4f(0, 0, 0, 1));
			act->setIsKinematic(true);
			act->getPhysicMaterial().m_restitution = 0.f;
			act->getPhysicMaterial().m_friction = 0.f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);
			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
			kinematicBox->setActor(act);

			groundHeight += height * 0.5f;
		}
		//return;
		//first block on the ground
		float blockOneHeight = 0;
		{
			float width = 10;
			float height = 7;
			float depth = 10;

			snVector4f pos(0, groundHeight + height * 0.5f, 0, 1);

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);
			_ASSERTE(_CrtCheckMemory());
			act->setName("base");
			act->setMass(200);
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0.f;
			act->getPhysicMaterial().m_friction = 0.25f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);
			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), color1);
			_ASSERTE(_CrtCheckMemory());
			box->setActor(act);

			blockOneHeight = pos.getY() + height * 0.5f;
		}
		//return;
		//platform
		float platformHeight = 0;
		{
			float width = 15;
			float height = 0.5;
			float depth = 3;
			snVector4f pos(width * 0.5f, blockOneHeight + height * 0.5f + 0, 0, 1);

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);
			act->setName("platform");
			act->setMass(100);
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0.f;
			act->getPhysicMaterial().m_friction = 1.f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);
			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), color2);
			box->setActor(act);

			platformHeight = pos.getY() + height * 0.5f;
		}
		//return;
		//second block
		float blockTwoHeight = 0;
		{
			float width = 3;
			float height = 3;
			float depth = 3;
			snVector4f pos(2, platformHeight + height * 0.5f, 0, 1);

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);
			act->setName("two");
			act->setMass(100);
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0.f;
			act->getPhysicMaterial().m_friction = 1.f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);
			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), color3);
			box->setActor(act);

			blockTwoHeight = pos.getY() + height * 0.5f;
		}
		//return;
		float blockThreeHeight = 0;
		{
			float width = 3;
			float height = 3;
			float depth = 3;
			snVector4f pos(2, blockTwoHeight + height * 0.5f, 0, 1);

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);
			act->setName("three");
			act->setMass(100);
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0.f;
			act->getPhysicMaterial().m_friction = 1.f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);
			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), color5);
			box->setActor(act);

			blockThreeHeight = pos.getY() + height * 0.5f;
		}

		//return;
		//dynamic
		{
			float width = 3;
			float height = 3;
			float depth = 3;
			snVector4f pos(11, platformHeight + height * 0.5f + 25, 0, 1);

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);
			act->setName("dymnamic");
			act->setMass(500);
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0.f;
			act->getPhysicMaterial().m_friction = 1.f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);
			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), color4);
			box->setActor(act);

			blockTwoHeight = pos.getY() + height * 0.5f;
		}
	}

	void SceneManager::createScene2()
	{
		WORLD->initialize();

		WorldHUD* HUD = WORLD->createHUD();
		HUD->setSceneName(L"Scene 2 : Stacking");

		GRAPHICS->setClearScreenColor(Colors::DarkGray);

		//create the camera.
		XMVECTOR cameraPosition = XMVectorSet(70, 50, 100, 1);
		XMVECTOR cameraLookAt = XMVectorSet(15, 15, 0, 1);
		XMVECTOR cameraUp = XMVectorSet(0, 1, 0, 0);
		WORLD->createCamera(cameraPosition, cameraLookAt, cameraUp);

		int sceneId = -1;
		snScene* m_physicScene = 0;
		SUPERNOVA->createScene(&m_physicScene, sceneId);
		m_physicScene->setGravity(snVector4f(0, -9.81f * 2, 0, 0));
		m_physicScene->setCollisionMode(m_collisionMode);

		int solverIterationCount = 60;
		m_physicScene->setSolverIterationCount(solverIterationCount);
		HUD->setSolverIterationCount(solverIterationCount);

		m_physicScene->setLinearSquaredSpeedThreshold(0.006f);
		m_physicScene->setAngularSquaredSpeedThreshold(0.0005f);

		//set camera initial position
		WORLD->deactivateCollisionPoint();

		XMFLOAT4 colors[5];
		colors[0] = XMFLOAT4(0.8f, 1, 1, 1);
		colors[1] = XMFLOAT4(0.93f, 0.68f, 1, 1);
		colors[2] = XMFLOAT4(1, 0.8f, 0.678f, 1);
		colors[3] = XMFLOAT4(0.89f, 0.71f, 0.75f, 1);
		colors[4] = XMFLOAT4(0.96f, 0.48f, 0.63f, 1);

		float groundHeight = 0;
		//ground
		{
			float width = 200;
			float height = 2;
			float depth = 200;

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createStaticActor(&act, actorId);

			act->setName("ground");
			act->setMass(100);
			act->setPosition(snVector4f(0, 0, 0, 1));
			//act->setIsKinematic(true);
			act->getPhysicMaterial().m_restitution = 1;
			act->getPhysicMaterial().m_friction = 0.8f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			//create the world entity
			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
			kinematicBox->setActor(act);

			groundHeight += height * 0.5f;
		}

		//back wall
		{
			float width = 200;
			float height = 200;
			float depth = 5;

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createStaticActor(&act, actorId);

			act->setName("back");
			act->setMass(100);
			act->setPosition(snVector4f(0, 101, -80, 1));
			//act->setIsKinematic(true);
			act->getPhysicMaterial().m_restitution = 1;
			act->getPhysicMaterial().m_friction = 1;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			//create world entity
			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
			kinematicBox->setActor(act);
		}

		const int MAX_ROW = 15;
		for (int row = MAX_ROW; row > 0; --row)
		{
			for (int i = 0; i < row; ++i)
			{
				float width = 5;
				float height = 5;
				float depth = 5;
				float space = 1.5;
				float xOffset = -10;
				snVector4f pos((MAX_ROW - row) * width * 0.5f + (width + space) * i + xOffset, groundHeight + height * 0.5f + height * (MAX_ROW - row), 0, 1);

				//create actor
				snActor* act = 0;
				int actorId = -1;
				m_physicScene->createActor(&act, actorId);

				string strRow = std::to_string(row);
				string strI = std::to_string(i);
				string name("base_" + strRow + "_" + strI);
				act->setName(name);
				//act->setName("base_" + std::to_string(row) + "_" + std::to_string(i));
				act->setMass(50);
				act->setPosition(pos);
				act->setIsKinematic(false);
				act->getPhysicMaterial().m_restitution = 0;
				act->getPhysicMaterial().m_friction = 1;

				//create collider
				snColliderBox* collider = 0;
				int colliderId = -1;
				act->createColliderBox(&collider, colliderId);

				collider->setSize(snVector4f(width, height, depth, 0));

				//initialize
				collider->initialize();
				act->initialize();

				EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), colors[(i + row) % 5]);
				box->setActor(act);
			}
		}
		return;
		{
			float width = 5;
			float height = 5;
			float depth = 5;
			snVector4f pos(10, 45, 50, 1);

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);
			act->setName("dynamic");
			act->setMass(500);
			act->setPosition(pos);
			act->setLinearVelocity(snVector4f(0, -50, -80, 0));
			act->getPhysicMaterial().m_restitution = 0;
			act->getPhysicMaterial().m_friction = 1;
			act->setIsKinematic(false);

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), colors[4]);
			box->setActor(act);
		}
	}

	void SceneManager::createScene3()
	{
		WORLD->initialize();

		WorldHUD* HUD = WORLD->createHUD();
		HUD->setSceneName(L"Scene 3 : Constraints");

		GRAPHICS->setClearScreenColor(Colors::DarkGray);

		snScene* m_physicScene = 0;
		int sceneId = -1;
		SUPERNOVA->createScene(&m_physicScene, sceneId);
		m_physicScene->setGravity(snVector4f(0, -9.81f * 4, 0, 0));
		m_physicScene->setCollisionMode(m_collisionMode);

		int solverIterationCount = 4;
		m_physicScene->setSolverIterationCount(solverIterationCount);
		HUD->setSolverIterationCount(solverIterationCount);

		m_physicScene->setLinearSquaredSpeedThreshold(0.006f);
		m_physicScene->setAngularSquaredSpeedThreshold(0.001f);

		//create the camera.
		XMVECTOR cameraPosition = XMVectorSet(0, 80, 150, 1);
		XMVECTOR cameraLookAt = XMVectorSet(10, 75, 0, 1);
		XMVECTOR cameraUp = XMVectorSet(0, 1, 0, 0);
		WORLD->createCamera(cameraPosition, cameraLookAt, cameraUp);

		//WORLD->deactivateCollisionPoint();
		WORLD->activateCollisionPoint();

		XMFLOAT4 colors[5];
		colors[0] = XMFLOAT4(0.8f, 1, 1, 1);
		colors[1] = XMFLOAT4(0.93f, 0.68f, 1, 1);
		colors[2] = XMFLOAT4(1, 0.8f, 0.678f, 1);
		colors[3] = XMFLOAT4(0.89f, 0.71f, 0.75f, 1);
		colors[4] = XMFLOAT4(0.96f, 0.48f, 0.63f, 1);

		float groundHeight = 0;
		//ground
		{
			float width = 200;
			float height = 2;
			float depth = 200;

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createStaticActor(&act, actorId);

			act->setName("ground");
			act->setMass(100);
			act->setPosition(snVector4f(0, 0, 0, 1));
			act->getPhysicMaterial().m_restitution = 1;
			act->getPhysicMaterial().m_friction = 0.8f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			//create the world entity
			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
			kinematicBox->setActor(act);

			groundHeight += height * 0.5f;
		}

		//back wall
		{
			float width = 200;
			float height = 200;
			float depth = 5;

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createStaticActor(&act, actorId);

			act->setName("back");
			act->setMass(100);
			act->setPosition(snVector4f(0, 101, -80, 1));
			//act->setIsKinematic(true);
			act->getPhysicMaterial().m_restitution = 1;
			act->getPhysicMaterial().m_friction = 1;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			//create world entity
			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
			kinematicBox->setActor(act);
		}
		float top = 100;
		snActor* previousActor = 0;
		{
			float width = 2;
			float height = 5;
			float depth = 2;
			snVector4f pos(10, top, 0, 1);

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);
			act->setName("d0");
			act->setMass(50);
			act->setPosition(pos);
			act->getPhysicMaterial().m_restitution = 0;
			act->getPhysicMaterial().m_friction = 1;
			act->setIsKinematic(false);
			act->setLinearDampingCoeff(20);
			act->setAngularDampingCoeff(20);
			previousActor = act;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), colors[4]);
			box->setActor(act);

			//create constraints
			snFixedConstraint* constraint = m_physicScene->createFixedConstraint(act, pos + snVector4f(0, 10, 0, 0), 10, 0.016f);
			WORLD->createFixedConstraint(constraint);
		}

		const int LENGTH = 7;
		const float LINK_LENGTH = 5;
		const float TWO_LINK_LENGTH = LINK_LENGTH * 2;
		for (int i = 0; i < LENGTH; ++i)
		{

			float width = 2;
			float height = 5;
			float depth = 2;
			top -= TWO_LINK_LENGTH;
			snVector4f pos(10, top, 0, 1);

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);
			act->setName("d1");
			act->setMass(50);
			act->setPosition(pos);
			act->getPhysicMaterial().m_restitution = 0;
			act->getPhysicMaterial().m_friction = 1;
			act->setIsKinematic(false);
			act->setAngularDampingCoeff(20);
			act->setLinearDampingCoeff(5);

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), colors[4]);
			box->setActor(act);

			//create p2p constraint
			snPointToPointConstraint* p2pc = m_physicScene->createPointToPointConstraint(previousActor, snVector4f(0, -LINK_LENGTH, 0, 1),
				act, snVector4f(0, LINK_LENGTH, 0, 1));
			WORLD->createPointToPointConstraint(p2pc);

			previousActor = act;
		}
	}

	void SceneManager::createScene4()
	{
		WORLD->initialize();

		WorldHUD* HUD = WORLD->createHUD();
		HUD->setSceneName(L"Scene 4 : Trash");

		GRAPHICS->setClearScreenColor(Colors::DarkGray);

		int sceneId = -1;
		snScene* m_physicScene = 0;
		SUPERNOVA->createScene(&m_physicScene, sceneId);
		m_physicScene->setCollisionMode(m_collisionMode);

		int solverIterationCount = 20;
		m_physicScene->setSolverIterationCount(solverIterationCount);
		HUD->setSolverIterationCount(solverIterationCount);

		m_physicScene->setBeta(0.2f);
		m_physicScene->setGravity(snVector4f(0, -9.81f * 0.5f, 0, 0));

		//create the camera.
		XMVECTOR cameraPosition = XMVectorSet(25, 30, 50, 1);
		XMVECTOR cameraLookAt = XMVectorSet(15, 7, 0, 1);
		XMVECTOR cameraUp = XMVectorSet(0, 1, 0, 0);
		WORLD->createCamera(cameraPosition, cameraLookAt, cameraUp);

		//WORLD->deactivateCollisionPoint();

		XMFLOAT4 color1(0.8f, 1, 1, 1);
		XMFLOAT4 color2(0.93f, 0.68f, 1, 1);
		XMFLOAT4 color3(1, 0.8f, 0.678f, 1);
		XMFLOAT4 color4(0.89f, 0.71f, 0.75f, 1);
		XMFLOAT4 color5(0.96f, 0.48f, 0.63f, 1);

		float groundHeight = 0;
		//ground
		{
			float width = 1000;
			float height = 2;
			float depth = 1000;

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);
			act->setName("ground");
			act->setMass(100);
			act->setPosition(snVector4f(0, 0, 0, 1));
			act->setIsKinematic(true);
			act->getPhysicMaterial().m_restitution = 1.f;
			act->getPhysicMaterial().m_friction = 0.f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);
			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
			kinematicBox->setActor(act);

			groundHeight += height * 0.5f;
		}

		//back wall
		//{
		//	float width = 200;
		//	float height = 200;
		//	float depth = 2;

		//	//create actor
		//	snActor* act = 0;
		//	int actorId = -1;
		//	m_physicScene->createActor(&act, actorId);

		//	act->setName("back");
		//	act->setMass(100);
		//	act->setPosition(snVector4f(0, 101, -80, 1));
		//	act->setIsKinematic(true);
		//	act->getPhysicMaterial().m_restitution = 1;
		//	act->getPhysicMaterial().m_friction = 1;

		//	//create collider
		//	snColliderBox* collider = 0;
		//	int colliderId = -1;
		//	act->createColliderBox(&collider, colliderId);

		//	collider->setSize(snVector4f(width, height, depth, 0));

		//	//initialize
		//	collider->initialize();
		//	act->initialize();

		//	//create world entity
		//	EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
		//	kinematicBox->setActor(act);
		//}

		//platform
		float platformHeight = 0;
		{
			float width = 7;
			float height = 1;
			float depth = 3;
			snVector4f pos(width * 0.5f, 20 + height * 0.5f + 0, 0, 1);

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);
			act->setName("platform");
			act->setMass(1);
			act->setPosition(pos);
			float angle = 3.14f * 0.25f;
			act->setOrientationQuaternion(snQuaternionFromEuler(angle, angle, 0));
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0.f;
			act->getPhysicMaterial().m_friction = 1.f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);
			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), color2);
			box->setActor(act);

			platformHeight = pos.getY() + height * 0.5f;
		}
		return;
		//dynamic
		{
			float width = 20;
			float height = 1;
			float depth = 3;
			snVector4f pos(width * 0.5f, 30 + height * 0.5f + 0, 0, 1);

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);
			act->setName("dymnamic");
			act->setMass(1);
			act->setPosition(pos);
			act->setOrientationQuaternion(snQuaternionFromEuler(0, 3.14f * 0.5f, 0));
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0.f;
			act->getPhysicMaterial().m_friction = 1.f;
			act->setAngularDampingCoeff(0.01f);

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);
			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), color4);
			box->setActor(act);

			//blockTwoHeight = pos.getY() + height * 0.5f;
		}
	}

	void SceneManager::createTower()
	{
		WORLD->initialize();
		WorldHUD* HUD = WORLD->createHUD();
		HUD->setSceneName(L"Scene 5 : Broken Tower");

		GRAPHICS->setClearScreenColor(Colors::DarkGray);
		int sceneId = -1;
		snScene* m_physicScene = 0;
		SUPERNOVA->createScene(&m_physicScene, sceneId);
		m_physicScene->setCollisionMode(m_collisionMode);

		int solverIterationCount = 45;
		m_physicScene->setSolverIterationCount(solverIterationCount);
		HUD->setSolverIterationCount(solverIterationCount);

		m_physicScene->setLinearSquaredSpeedThreshold(0.006f);
		m_physicScene->setAngularSquaredSpeedThreshold(0.001f);

		//create the camera.
		XMVECTOR cameraPosition = XMVectorSet(50, 50, 70, 1);
		XMVECTOR cameraLookAt = XMVectorSet(0, 7, 0, 1);
		XMVECTOR cameraUp = XMVectorSet(0, 1, 0, 0);
		WORLD->createCamera(cameraPosition, cameraLookAt, cameraUp);


		WORLD->deactivateCollisionPoint();


		float groundHeight = 0;
		//ground
		{
			float width = 2000;
			float height = 2;
			float depth = 2000;

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);

			act->setName("ground");
			act->setMass(100);
			act->setPosition(snVector4f(0, 0, 0, 1));
			act->setIsKinematic(true);
			act->getPhysicMaterial().m_restitution = 0;
			act->getPhysicMaterial().m_friction = 0.8f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			//create the world entity
			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
			kinematicBox->setActor(act);

			groundHeight += height * 0.5f;
		}

		//back wall
		{
			float width = 200;
			float height = 200;
			float depth = 2;

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);

			act->setName("back");
			act->setMass(100);
			act->setPosition(snVector4f(0, 101, -80, 1));
			act->setIsKinematic(true);
			act->getPhysicMaterial().m_restitution = 1;
			act->getPhysicMaterial().m_friction = 1;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			//create world entity
			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
			kinematicBox->setActor(act);
		}

		//tower
		snVector4f origin(0, groundHeight, 0, 1);

		int levelCount = 5;
		for (int i = 0; i < levelCount; ++i)
		{
			origin = createTowerLevel(m_physicScene, origin);
		}
		return;
		//projectile
		{
			float width = 2;
			float height = 2;
			float depth = 2;

			//create actor
			snActor* act = 0;
			int actorId = -1;
			m_physicScene->createActor(&act, actorId);

			act->setName("ground");
			act->setMass(500);
			act->setPosition(snVector4f(-0, 20, 50, 1));
			act->setLinearVelocity(snVector4f(15, 0, -50, 0));
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0;
			act->getPhysicMaterial().m_friction = 0.8f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			//create the world entity
			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth), XMFLOAT4(1, 0, 0, 1));
			kinematicBox->setActor(act);
		}
	}

	void SceneManager::createSceneFriction()
	{
		//initialize the world
		WORLD->initialize();

		WorldHUD* HUD = WORLD->createHUD();
		HUD->setSceneName(L"Scene 6 : Friction");

		GRAPHICS->setClearScreenColor(Colors::DarkGray);

		//create the physics scene
		int sceneId = -1;
		snScene* scene = 0;
		SUPERNOVA->createScene(&scene, sceneId);
		scene->setCollisionMode(m_collisionMode);

		int solverIterationCount = 4;
		scene->setSolverIterationCount(solverIterationCount);
		HUD->setSolverIterationCount(solverIterationCount);

		scene->setLinearSquaredSpeedThreshold(0.006f);
		scene->setAngularSquaredSpeedThreshold(0.001f);

		//create the camera.
		XMVECTOR cameraPosition = XMVectorSet(80, 50, 0, 1);
		XMVECTOR cameraLookAt = XMVectorSet(0, 7, 0, 1);
		XMVECTOR cameraUp = XMVectorSet(0, 1, 0, 0);
		WORLD->createCamera(cameraPosition, cameraLookAt, cameraUp);


		WORLD->activateCollisionPoint();

		float slopeAngle = 3.14f * 0.25f;
		//slope
		{
			float width = 50;
			float height = 2;
			float depth = 100;

			//create actor
			snActor* act = 0;
			int actorId = -1;
			scene->createActor(&act, actorId);

			act->setName("slope");
			act->setMass(100);
			act->setPosition(snVector4f(0, 0, 0, 1));
			act->setOrientationQuaternion(snQuaternionFromEuler(slopeAngle, 0, 0));

			act->setIsKinematic(true);
			act->getPhysicMaterial().m_restitution = 1.f;
			act->getPhysicMaterial().m_friction = 1.f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			//create the world entity
			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
			kinematicBox->setActor(act);
		}

		//ground
		{
			float width = 500;
			float height = 2;
			float depth = 500;

			//create actor
			snActor* act = 0;
			int actorId = -1;
			scene->createActor(&act, actorId);

			act->setName("ground");
			act->setMass(100);
			act->setPosition(snVector4f(0, -40, 0, 1));

			act->setIsKinematic(true);
			act->getPhysicMaterial().m_restitution = 0;
			act->getPhysicMaterial().m_friction = 0.8f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			collider->setSize(snVector4f(width, height, depth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			//create the world entity
			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
			kinematicBox->setActor(act);
		}

		XMFLOAT4 colors[5];
		colors[0] = XMFLOAT4(0.8f, 1, 1, 1);
		colors[1] = XMFLOAT4(0.93f, 0.68f, 1, 1);
		colors[2] = XMFLOAT4(1, 0.8f, 0.678f, 1);
		colors[3] = XMFLOAT4(0.89f, 0.71f, 0.75f, 1);
		colors[4] = XMFLOAT4(0.96f, 0.48f, 0.63f, 1);

		float height = 35;
		//full friction, no restitution
		{
			//create actor
			snActor* act = 0;
			int actorId = -1;
			scene->createActor(&act, actorId);
			act->setName("full friction");
			//act->setName("base_" + std::to_string(row) + "_" + std::to_string(i));
			act->setMass(50);
			act->setPosition(snVector4f(-10, height, -30, 1));
			act->setOrientationQuaternion(snQuaternionFromEuler(slopeAngle, 0, 0));
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = -1.f;
			act->getPhysicMaterial().m_friction = 1.f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			float cubeSize = 5;
			collider->setSize(snVector4f(cubeSize, cubeSize, cubeSize, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(cubeSize, cubeSize, cubeSize), colors[0]);
			box->setActor(act);
		}

		//no friction, no restitution
		{
			//create actor
			snActor* act = 0;
			int actorId = -1;
			scene->createActor(&act, actorId);
			act->setName("no friction");
			//act->setName("base_" + std::to_string(row) + "_" + std::to_string(i));
			act->setMass(50);
			act->setPosition(snVector4f(10, height, -30, 1));
			act->setOrientationQuaternion(snQuaternionFromEuler(slopeAngle, 0, 0));
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = -1.f;
			act->getPhysicMaterial().m_friction = -1.f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			float cubeSize = 5;
			collider->setSize(snVector4f(cubeSize, cubeSize, cubeSize, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(cubeSize, cubeSize, cubeSize), colors[2]);
			box->setActor(act);
		}

		//half friction, no restitution
		{
			//create actor
			snActor* act = 0;
			int actorId = -1;
			scene->createActor(&act, actorId);
			act->setName("half friction");
			act->setMass(50);
			act->setPosition(snVector4f(0, height, -30, 1));
			act->setOrientationQuaternion(snQuaternionFromEuler(slopeAngle, 0, 0));
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = -1.f;
			act->getPhysicMaterial().m_friction = 0.f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			float cubeSize = 5;
			collider->setSize(snVector4f(cubeSize, cubeSize, cubeSize, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(cubeSize, cubeSize, cubeSize), colors[1]);
			box->setActor(act);
		}
	}

	void SceneManager::setCollisionMode(snCollisionMode _collisionMode)
	{
		m_collisionMode = _collisionMode;
	}

	SceneManager::SceneManager() : m_collisionMode(snCollisionMode::snECollisionModeSweepAndPrune)
	{}

	SceneManager::~SceneManager()
	{}

	void SceneManager::clearScene() const
	{
		SUPERNOVA->deleteAllScenes();
		GRAPHICS->clear();
		WORLD->clearWorld();
	}

	snVector4f SceneManager::createTowerLevel(snScene* const _scene, const snVector4f& _origin) const
	{
		//colors
		const int colorCount = 5;
		XMFLOAT4 colors[colorCount];
		colors[0] = XMFLOAT4(0.8f, 1, 1, 1);
		colors[1] = XMFLOAT4(0.93f, 0.68f, 1, 1);
		colors[2] = XMFLOAT4(1, 0.8f, 0.678f, 1);
		colors[3] = XMFLOAT4(0.89f, 0.71f, 0.75f, 1);
		colors[4] = XMFLOAT4(0.96f, 0.48f, 0.63f, 1);

		float towerWidth = 20;
		float pillarHeight = 5;
		float pillarWidth = 2;
		float pillarDepth = 2;

		const int pillarCount = 4;
		snVector4f pillarPosition[pillarCount];

		float halfTowerWidth = towerWidth * 0.5f;
		float halfPillarHeight = pillarHeight * 0.5f;
		pillarPosition[0] = snVector4f(halfTowerWidth, halfPillarHeight, halfTowerWidth, 0);
		pillarPosition[1] = snVector4f(-halfTowerWidth, halfPillarHeight, halfTowerWidth, 0);
		pillarPosition[2] = snVector4f(-halfTowerWidth, halfPillarHeight, -halfTowerWidth, 0);
		pillarPosition[3] = snVector4f(halfTowerWidth, halfPillarHeight, -halfTowerWidth, 0);

		//create pillars
		for (int i = 0; i < pillarCount; ++i)
		{
			//create actor
			snActor* act = 0;
			int actorId = -1;
			_scene->createActor(&act, actorId);

			snVector4f pos = pillarPosition[i] + _origin;
			act->setName("pillar1");
			act->setMass(50);
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0;
			act->getPhysicMaterial().m_friction = 1;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			collider->setSize(snVector4f(pillarWidth, pillarHeight, pillarDepth, 0));

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(pillarWidth, pillarHeight, pillarDepth), colors[actorId % colorCount]);
			box->setActor(act);
		}

		float bedWidth = towerWidth + 2 * pillarWidth;
		float bedDepth = pillarDepth * 2;
		float bedHeight = 1;

		float firstBedHeight = pillarHeight + bedHeight * 0.5f;
		float secondBedHeight = firstBedHeight + bedHeight;

		snVector4f bedPosition[4];
		bedPosition[0] = snVector4f(0, firstBedHeight, halfTowerWidth, 0);
		bedPosition[1] = snVector4f(0, firstBedHeight, -halfTowerWidth, 0);
		bedPosition[2] = snVector4f(halfTowerWidth, secondBedHeight, 0, 0);
		bedPosition[3] = snVector4f(-halfTowerWidth, secondBedHeight, 0, 0);

		snVector4f bedSize[4];
		bedSize[0] = snVector4f(bedWidth, bedHeight, bedDepth, 0);
		bedSize[1] = snVector4f(bedWidth, bedHeight, bedDepth, 0);
		bedSize[2] = snVector4f(bedDepth, bedHeight, bedWidth, 0);
		bedSize[3] = snVector4f(bedDepth, bedHeight, bedWidth, 0);

		//create bed between pillars
		for (int i = 0; i < pillarCount; ++i)
		{
			//create actor
			snActor* act = 0;
			int actorId = -1;
			_scene->createActor(&act, actorId);

			snVector4f pos = bedPosition[i] + _origin;
			act->setName("bed");
			act->setMass(50);
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0;
			act->getPhysicMaterial().m_friction = 0.9f;

			//create collider
			snColliderBox* collider = 0;
			int colliderId = -1;
			act->createColliderBox(&collider, colliderId);

			collider->setSize(bedSize[i]);

			//initialize
			collider->initialize();
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(bedSize[i].getX(), bedSize[i].getY(), bedSize[i].getZ()), colors[actorId % colorCount]);
			box->setActor(act);
		}

		return _origin + snVector4f(0, pillarHeight + 2 * bedHeight, 0, 0);
	}

	
}