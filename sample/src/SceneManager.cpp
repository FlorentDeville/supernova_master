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
#include "snActorDynamic.h"
#include "snActorStatic.h"
#include "snColliderBox.h"
#include "snQuaternion.h"
#include "snDebugger.h"

#include "EntityBox.h"

#include "ComponentFollowPath.h"
#include "ComponentPathInterpolate.h"
#include "ComponentFloatingText.h"

#include "PathExplorer.h"

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
			createBasicTest();
			INPUT->keyUp(112);
		}
		else if (INPUT->isKeyDown(113))//F2
		{
			clearScene();
			createStacking();
			INPUT->keyUp(113);
		}
		else if (INPUT->isKeyDown(114))//F3
		{
			clearScene();
			createConstraints();
			INPUT->keyUp(114);
		}
		else if (INPUT->isKeyDown(115))//F4
		{
			clearScene();
			createSceneDamping();
			INPUT->keyUp(115);
		}
		else if (INPUT->isKeyDown(116))//F5
		{
			clearScene();
			createSceneFriction();
			INPUT->keyUp(116);
		}
		else if (INPUT->isKeyDown(117))//F6
		{
			clearScene();
			createSceneActorsType();
			INPUT->keyUp(117);
		}
		else if (INPUT->isKeyDown(118))//F7
		{
			clearScene();
			createSceneDomino();
			INPUT->keyUp(118);
		}
		else if (INPUT->isKeyDown(119))//F8
		{
			clearScene();
			createTower();
			INPUT->keyUp(119);
		}
	}

	void SceneManager::createBasicTest()
	{
		WORLD->initialize();
		WorldHUD* HUD = WORLD->createHUD();
		HUD->setSceneName(L"Scene 1 : Basic Test");

		GRAPHICS->setClearScreenColor(Colors::DarkGray);
		int sceneId = -1;

		//create the scene
		snScene* m_physicScene = 0;
		SUPERNOVA->createScene(&m_physicScene, sceneId);
		m_physicScene->setGravity(snVector4f(0, -9.81f * 2, 0, 0));
		m_physicScene->setCollisionMode(m_collisionMode);
		m_physicScene->setLinearSquaredSpeedThreshold(0.005f);
		int solverIterationCount = 5;
		m_physicScene->setSolverIterationCount(solverIterationCount);

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
		createGround(m_physicScene, 0.f, 0.f);
		
		//first block on the ground
		float blockOneHeight = 0;
		{
			float width = 10;
			float height = 7;
			float depth = 10;

			snVector4f pos(0, groundHeight + height * 0.5f, 0, 1);

			//create actor
			snActorDynamic* act = 0;
			int actorId = -1;
			m_physicScene->createActorDynamic(&act, actorId);
			_ASSERTE(_CrtCheckMemory());
			act->setName("base");
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0.f;
			act->getPhysicMaterial().m_friction = 0.25f;

			//create collider
			snColliderBox* collider = new snColliderBox();
			collider->setSize(snVector4f(width, height, depth, 0));
			act->addCollider(collider);
			act->updateMassAndInertia(200);
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), color1);
			_ASSERTE(_CrtCheckMemory());
			box->setActor(act);

			blockOneHeight = pos.getY() + height * 0.5f;
		}

		//platform
		float platformHeight = 0;
		{
			float width = 15;
			float height = 0.5;
			float depth = 3;
			snVector4f pos(width * 0.5f, blockOneHeight + height * 0.5f + 0, 0, 1);

			//create actor
			snActorDynamic* act = 0;
			int actorId = -1;
			m_physicScene->createActorDynamic(&act, actorId);
			act->setName("platform");
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0.f;
			act->getPhysicMaterial().m_friction = 1.f;

			//create collider
			snColliderBox* collider = new snColliderBox();
			collider->setSize(snVector4f(width, height, depth, 0));
			act->addCollider(collider);
			act->updateMassAndInertia(100);
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), color2);
			box->setActor(act);

			platformHeight = pos.getY() + height * 0.5f;
		}

		//second block
		float blockTwoHeight = 0;
		{
			float width = 3;
			float height = 3;
			float depth = 3;
			snVector4f pos(2, platformHeight + height * 0.5f, 0, 1);

			//create actor
			snActorDynamic* act = 0;
			int actorId = -1;
			m_physicScene->createActorDynamic(&act, actorId);
			act->setName("two");
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0.f;
			act->getPhysicMaterial().m_friction = 1.f;

			//create collider
			snColliderBox* collider = new snColliderBox();
			collider->setSize(snVector4f(width, height, depth, 0));
			act->addCollider(collider);
			act->updateMassAndInertia(100);
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), color3);
			box->setActor(act);

			blockTwoHeight = pos.getY() + height * 0.5f;
		}

		float blockThreeHeight = 0;
		{
			float width = 3;
			float height = 3;
			float depth = 3;
			snVector4f pos(2, blockTwoHeight + height * 0.5f, 0, 1);

			//create actor
			snActorDynamic* act = 0;
			int actorId = -1;
			m_physicScene->createActorDynamic(&act, actorId);
			act->setName("three");
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0.f;
			act->getPhysicMaterial().m_friction = 1.f;

			//create collider
			snColliderBox* collider = new snColliderBox();
			collider->setSize(snVector4f(width, height, depth, 0));
			act->addCollider(collider);

			act->updateMassAndInertia(100);
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), color5);
			box->setActor(act);

			blockThreeHeight = pos.getY() + height * 0.5f;
		}

		//dynamic
		{
			float width = 3;
			float height = 3;
			float depth = 3;
			snVector4f pos(11, platformHeight + height * 0.5f + 25, 0, 1);

			//create actor
			snActorDynamic* act = 0;
			int actorId = -1;
			m_physicScene->createActorDynamic(&act, actorId);
			act->setName("dymnamic");
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0.f;
			act->getPhysicMaterial().m_friction = 1.f;

			//create collider
			snColliderBox* collider = new snColliderBox();
			collider->setSize(snVector4f(width, height, depth, 0));
			act->addCollider(collider);

			act->updateMassAndInertia(500);
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), color4);
			box->setActor(act);

			blockTwoHeight = pos.getY() + height * 0.5f;
		}
	}

	void SceneManager::createStacking()
	{
		WORLD->initialize();

		WorldHUD* HUD = WORLD->createHUD();
		HUD->setSceneName(L"Scene 2 : Stacking");

		GRAPHICS->setClearScreenColor(Colors::DarkGray);

		//create the camera.
		XMVECTOR cameraPosition = XMVectorSet(100, 79, 140, 1);
		XMVECTOR cameraLookAt = XMVectorSet(35, 21, 0, 1);
		XMVECTOR cameraUp = XMVectorSet(0, 1, 0, 0);
		WORLD->createCamera(cameraPosition, cameraLookAt, cameraUp);

		int sceneId = -1;
		snScene* m_physicScene = 0;
		SUPERNOVA->createScene(&m_physicScene, sceneId);
		m_physicScene->setGravity(snVector4f(0, -9.81f * 2, 0, 0));
		m_physicScene->setCollisionMode(m_collisionMode);

		int solverIterationCount = 60;
		m_physicScene->setSolverIterationCount(solverIterationCount);

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
	
		//ground
		float groundHeight = 0;
		createGround(m_physicScene, 1, 0.8f);
		
		//back wall
		{
			float width = 200;
			float height = 200;
			float depth = 5;

			//create actor
			snActorStatic* act = 0;
			int actorId = -1;
			snVector4f position(0, 101, -80, 1);
			m_physicScene->createActorStatic(&act, actorId, position, snVector4f(0, 0, 0, 1));
			act->setName("back");
			act->getPhysicMaterial().m_restitution = 1;
			act->getPhysicMaterial().m_friction = 1;

			//create collider
			snColliderBox* collider = new snColliderBox();
			collider->setSize(snVector4f(width, height, depth, 0));
			act->addCollider(collider);

			//initialize
			act->initialize();

			//create world entity
			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
			kinematicBox->setActor(act);
		}

		const int MAX_ROW = 14;
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
				snActorDynamic* act = 0;
				int actorId = -1;
				m_physicScene->createActorDynamic(&act, actorId);

				string strRow = std::to_string(row);
				string strI = std::to_string(i);
				string name("base_" + strRow + "_" + strI);
				act->setName(name);
				act->setPosition(pos);
				act->setIsKinematic(false);
				act->getPhysicMaterial().m_restitution = 0;
				act->getPhysicMaterial().m_friction = 1;
				act->setSkinDepth(0.025f);

				//create collider
				snColliderBox* collider = new snColliderBox();
				collider->setSize(snVector4f(width, height, depth, 0));
				act->addCollider(collider);

				//initialize
				act->updateMassAndInertia(50);
				act->initialize();

				EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), colors[(i + row) % 5]);
				box->setActor(act);
			}
		}
	}

	void SceneManager::createConstraints()
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

		m_physicScene->setLinearSquaredSpeedThreshold(0.006f);
		m_physicScene->setAngularSquaredSpeedThreshold(0.001f);

		//create the camera.
		XMVECTOR cameraPosition = XMVectorSet(0, 80, 150, 1);
		XMVECTOR cameraLookAt = XMVectorSet(10, 75, 0, 1);
		XMVECTOR cameraUp = XMVectorSet(0, 1, 0, 0);
		WORLD->createCamera(cameraPosition, cameraLookAt, cameraUp);

		WORLD->activateCollisionPoint();

		XMFLOAT4 colors[5];
		colors[0] = XMFLOAT4(0.8f, 1, 1, 1);
		colors[1] = XMFLOAT4(0.93f, 0.68f, 1, 1);
		colors[2] = XMFLOAT4(1, 0.8f, 0.678f, 1);
		colors[3] = XMFLOAT4(0.89f, 0.71f, 0.75f, 1);
		colors[4] = XMFLOAT4(0.96f, 0.48f, 0.63f, 1);

		createGround(m_physicScene, 1, 1);

		//back wall
		{
			float width = 200;
			float height = 200;
			float depth = 5;

			//create actor
			snActorStatic* act = 0;
			int actorId = -1;
			m_physicScene->createActorStatic(&act, actorId, snVector4f(0, 101, -80, 1), snVector4f(0, 0, 0, 1));

			act->setName("back");
			act->getPhysicMaterial().m_restitution = 1;
			act->getPhysicMaterial().m_friction = 1;

			//create collider
			snColliderBox* collider = new snColliderBox();
			collider->setSize(snVector4f(width, height, depth, 0));
			act->addCollider(collider);

			//initialize
			act->initialize();

			//create world entity
			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
			kinematicBox->setActor(act);
		}

		float top = 100;
		snActorDynamic* previousActor = 0;
		{
			float width = 2;
			float height = 5;
			float depth = 2;
			snVector4f pos(10, top, 0, 1);

			//create actor
			snActorDynamic* act = 0;
			int actorId = -1;
			m_physicScene->createActorDynamic(&act, actorId);
			act->setName("d0");
			act->setPosition(pos);
			act->getPhysicMaterial().m_restitution = 0;
			act->getPhysicMaterial().m_friction = 1;
			act->setIsKinematic(false);
			previousActor = act;

			//create collider
			snColliderBox* collider = new snColliderBox();
			collider->setSize(snVector4f(width, height, depth, 0));
			act->addCollider(collider);

			//initialize
			act->updateMassAndInertia(50);
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), colors[4]);
			box->setActor(act);

			//create constraints
			snFixedConstraint* constraint = m_physicScene->createFixedConstraint(act, pos + snVector4f(0, 10, 0, 0), 10);
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
			snActorDynamic* act = 0;
			int actorId = -1;
			m_physicScene->createActorDynamic(&act, actorId);
			act->setName("d1");
			act->setPosition(pos);
			act->getPhysicMaterial().m_restitution = 0;
			act->getPhysicMaterial().m_friction = 1;
			act->setIsKinematic(false);
			act->setAngularDampingCoeff(0.1f);
			act->setLinearDampingCoeff(0.1f);

			//create collider
			snColliderBox* collider = new snColliderBox();
			collider->setSize(snVector4f(width, height, depth, 0));
			act->addCollider(collider);

			//initialize
			act->updateMassAndInertia(50);
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

	void SceneManager::createTower()
	{
		WORLD->initialize();

		WorldHUD* HUD = WORLD->createHUD();
		HUD->setSceneName(L"Scene : Broken Tower");
		GRAPHICS->setClearScreenColor(Colors::DarkGray);
		int sceneId = -1;
		snScene* m_physicScene = 0;
		SUPERNOVA->createScene(&m_physicScene, sceneId);
		m_physicScene->setCollisionMode(m_collisionMode);

		int solverIterationCount = 120;
		m_physicScene->setSolverIterationCount(solverIterationCount);
		//m_physicScene->setLinearSquaredSpeedThreshold(0.006f);
		m_physicScene->setAngularSquaredSpeedThreshold(0.f);
		m_physicScene->setContactConstraintBeta(0.005f);

		//create the camera.
		XMVECTOR 	cameraPosition = XMVectorSet(50, 50, 100, 1);
		XMVECTOR 	cameraLookAt = XMVectorSet(5, 7, 0, 1);
		XMVECTOR cameraUp = XMVectorSet(0, 1, 0, 0);
		WORLD->createCamera(cameraPosition, cameraLookAt, cameraUp);
		
		WORLD->activateCollisionPoint();
		
		float 	groundHeight = 0;
		
		createGround(m_physicScene, 0, 1);
		
		//back 	wall
		{
			float width = 200;
			float height = 200;
			float depth = 2;

			//create actor			
			snActorStatic* 	act = 0;
			
			int actorId = -1;
			
			m_physicScene->createActorStatic(&act, actorId, snVector4f(0, 101, -80, 1), snVector4f(0, 0, 0, 1));
			act->setName("back");
			act->getPhysicMaterial().m_restitution = 1;
			act->getPhysicMaterial().m_friction = 1;

			//create collider				
			snColliderBox* 	collider = new snColliderBox();
			collider->setSize(snVector4f(width, height, depth, 0));
			act->addCollider(collider);
				
			//initialize
			act->initialize();
			
			//create 	world 	entity	
			EntityBox* 	kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
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
			snActorStatic* act = 0;
			int actorId = -1;
			scene->createActorStatic(&act, actorId, snVector4f(0, 0, 0, 1), snQuaternionFromEuler(slopeAngle, 0, 0));
			act->setName("slope");
			act->getPhysicMaterial().m_restitution = 1.f;
			act->getPhysicMaterial().m_friction = 1.f;

			//create collider
			snColliderBox* collider = new snColliderBox();
			collider->setSize(snVector4f(width, height, depth, 0));
			act->addCollider(collider);

			//initialize
			act->initialize();

			//create the world entity
			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
			kinematicBox->setActor(act);
		}

		createGround(scene, 0, 0.8f);
		
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
			snActorDynamic* act = 0;
			int actorId = -1;
			scene->createActorDynamic(&act, actorId);
			act->setName("full friction");
			act->setPosition(snVector4f(-10, height, -30, 1));
			act->setOrientation(snQuaternionFromEuler(slopeAngle, 0, 0));
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = -1.f;
			act->getPhysicMaterial().m_friction = 1.f;

			//create collider
			snColliderBox* collider = new snColliderBox();
			float cubeSize = 5;
			collider->setSize(snVector4f(cubeSize, cubeSize, cubeSize, 0));
			act->addCollider(collider);

			//initialize
			act->updateMassAndInertia(50);
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(cubeSize, cubeSize, cubeSize), colors[0]);
			box->setActor(act);
		}

		//no friction, no restitution
		{
			//create actor
			snActorDynamic* act = 0;
			int actorId = -1;
			scene->createActorDynamic(&act, actorId);
			act->setName("no friction");
			act->setPosition(snVector4f(10, height, -30, 1));
			act->setOrientation(snQuaternionFromEuler(slopeAngle, 0, 0));
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = -1.f;
			act->getPhysicMaterial().m_friction = -1.f;

			//create collider
			snColliderBox* collider = new snColliderBox();
			float cubeSize = 5;
			collider->setSize(snVector4f(cubeSize, cubeSize, cubeSize, 0));
			act->addCollider(collider);

			//initialize
			act->updateMassAndInertia(50);
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(cubeSize, cubeSize, cubeSize), colors[2]);
			box->setActor(act);
		}

		//half friction, no restitution
		{
			//create actor
			snActorDynamic* act = 0;
			int actorId = -1;
			scene->createActorDynamic(&act, actorId);
			act->setName("half friction");
			act->setPosition(snVector4f(0, height, -30, 1));
			act->setOrientation(snQuaternionFromEuler(slopeAngle, 0, 0));
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = -1.f;
			act->getPhysicMaterial().m_friction = 0.f;

			//create collider
			snColliderBox* collider = new snColliderBox();
			float cubeSize = 5;
			collider->setSize(snVector4f(cubeSize, cubeSize, cubeSize, 0));
			act->addCollider(collider);

			//initialize
			act->updateMassAndInertia(50);
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(cubeSize, cubeSize, cubeSize), colors[1]);
			box->setActor(act);
		}
	}

	void SceneManager::createSceneDamping()
	{
		//initialize the world
		WORLD->initialize();

		WorldHUD* HUD = WORLD->createHUD();
		HUD->setSceneName(L"Scene 4 : Damping");

		GRAPHICS->setClearScreenColor(Colors::DarkGray);

		//create the physics scene
		int sceneId = -1;
		snScene* scene = 0;
		SUPERNOVA->createScene(&scene, sceneId);
		scene->setCollisionMode(m_collisionMode);

		int solverIterationCount = 4;
		scene->setSolverIterationCount(solverIterationCount);

		scene->setLinearSquaredSpeedThreshold(0.000001f);
		scene->setAngularSquaredSpeedThreshold(0.000001f);

		//create the camera.
		XMVECTOR cameraPosition = XMVectorSet(50, 90, 80, 1);
		XMVECTOR cameraLookAt = XMVectorSet(50, 60, 0, 1);
		XMVECTOR cameraUp = XMVectorSet(0, 1, 0, 0);
		WORLD->createCamera(cameraPosition, cameraLookAt, cameraUp);


		WORLD->activateCollisionPoint();

		createGround(scene, 0, 0.8f);

		XMFLOAT4 colors[5];
		colors[0] = XMFLOAT4(0.8f, 1, 1, 1);
		colors[1] = XMFLOAT4(0.93f, 0.68f, 1, 1);
		colors[2] = XMFLOAT4(1, 0.8f, 0.678f, 1);
		colors[3] = XMFLOAT4(0.89f, 0.71f, 0.75f, 1);
		colors[4] = XMFLOAT4(0.96f, 0.48f, 0.63f, 1);

		/////ANGULAR DAMPING////////////
		const int BOX_COUNT = 5;
		snVector4f origin(0, 70, 0, 1);
		for (int i = 0; i < BOX_COUNT; ++i)
		{
			float width = 5;
			float height = 5;
			float depth = 5;
			snVector4f pos = origin + snVector4f(20.f * i, 0, 0, 1);

			//create actor
			snActorDynamic* act = 0;
			int actorId = -1;
			scene->createActorDynamic(&act, actorId);
			act->setName("d0");
			act->setPosition(pos);
			act->getPhysicMaterial().m_restitution = 0;
			act->getPhysicMaterial().m_friction = 1;
			act->setIsKinematic(false);
			act->setAngularDampingCoeff(i*0.2f);
			const float v = 5;
			act->setAngularVelocity(snVector4f(0, v, 0, 0));

			//create collider
			snColliderBox* collider = new snColliderBox();
			collider->setSize(snVector4f(width, height, depth, 0));
			act->addCollider(collider);

			//initialize
			act->updateMassAndInertia(50);
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), colors[i]);
			box->setActor(act);

			//create floating text component
			ComponentFloatingText<snActorDynamic, float>* floatingText = new ComponentFloatingText<snActorDynamic, float>();
			floatingText->setAnchor(box);
			floatingText->setOffset(XMFLOAT2(-120, 50));
			floatingText->addItem(L"Angular Damping", act, &snActorDynamic::getAngularDampingCoeff);
			floatingText->addItem(L"Angular Speed", act, &snActorDynamic::computeAngularSpeed);
			box->addPostUpdateComponent(floatingText);

			//create constraints
			snFixedConstraint* constraint = scene->createFixedConstraint(act, pos + snVector4f(0, 10, 0, 0), 10);
			WORLD->createFixedConstraint(constraint);
		}

		/////////LINEAR DAMPING/////////////
		origin[1] = 50;
		for (int i = 0; i < BOX_COUNT; ++i)
		{
			float width = 5;
			float height = 5;
			float depth = 5;
			snVector4f pos = origin + snVector4f(20.f * i, 0, 0, 1);

			//create actor
			snActorDynamic* act = 0;
			int actorId = -1;
			scene->createActorDynamic(&act, actorId);
			act->setName("d0");
			act->setPosition(pos);
			act->getPhysicMaterial().m_restitution = 0;
			act->getPhysicMaterial().m_friction = 1;
			act->setIsKinematic(false);
			act->setLinearDampingCoeff(0.2f * i);
			const float v = 10;
			act->setLinearVelocity(snVector4f(v, 0, v, 0));
			
			//create collider
			snColliderBox* collider = new snColliderBox();
			collider->setSize(snVector4f(width, height, depth, 0));
			act->addCollider(collider);

			//initialize
			act->updateMassAndInertia(50);
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), colors[i]);
			box->setActor(act);

			//create floating text component
			ComponentFloatingText<snActorDynamic, float>* floatingText = new ComponentFloatingText<snActorDynamic, float>();
			floatingText->setAnchor(box);
			floatingText->setOffset(XMFLOAT2(-120, 50));
			floatingText->addItem(L"Linear Damping", act, &snActorDynamic::getLinearDampingCoeff);
			floatingText->addItem(L"Linear Speed", act, &snActorDynamic::computeLinearSpeed);
			box->addPostUpdateComponent(floatingText);

			//create constraints
			snFixedConstraint* constraint = scene->createFixedConstraint(act, pos + snVector4f(0, 10, 0, 0), 10);
			WORLD->createFixedConstraint(constraint);
		}
	}

	void SceneManager::createSceneActorsType()
	{
		//initialize the world
		WORLD->initialize();

		WorldHUD* HUD = WORLD->createHUD();
		HUD->setSceneName(L"Scene : Static, Dynamic, Kinematic");

		GRAPHICS->setClearScreenColor(Colors::DarkGray);

		//create the physics scene
		int sceneId = -1;
		snScene* scene = 0;
		SUPERNOVA->createScene(&scene, sceneId);
		scene->setCollisionMode(m_collisionMode);

		int solverIterationCount = 4;
		scene->setSolverIterationCount(solverIterationCount);

		scene->setLinearSquaredSpeedThreshold(0.000001f);
		scene->setAngularSquaredSpeedThreshold(0.000001f);

		//create the camera.
		XMVECTOR cameraPosition = XMVectorSet(0, 110, 50, 1);
		XMVECTOR cameraLookAt = XMVectorSet(0, 110, 0, 1);
		XMVECTOR cameraUp = XMVectorSet(0, 1, 0, 0);
		WORLD->createCamera(cameraPosition, cameraLookAt, cameraUp);


		WORLD->activateCollisionPoint();

		createGround(scene, 1, 1);

		//this is a kinematic platform
		{
			float width = 40;
			float height = 2;
			float depth = 40;

			//create the actor
			snActorDynamic* kin = 0;
			int actId = -1;
			scene->createActorDynamic(&kin, actId);
			kin->setIsKinematic(true);
			kin->setName("kinematic");
			kin->setPosition(snVector4f(-15, 100, 0, 1));
			kin->getPhysicMaterial().m_friction = 1.f;
			kin->getPhysicMaterial().m_restitution = 0.f;
			kin->setLinearDampingCoeff(0.f);

			//create collider
			snColliderBox* box = new snColliderBox();
			box->setSize(snVector4f(width, height, depth, 0));

			kin->addCollider(box);
			kin->initialize();

			//create entity
			EntityBox* entity = WORLD->createBox(XMFLOAT3(width, height, depth), m_colors[0]);
			entity->setActor(kin);

			//create path component
			ComponentFollowPath* path = new ComponentFollowPath(kin, true);
			float speed = 5.f;

			path->addWaypoint(snVector4f(-10, 100, 0, 1), speed);
			path->addWaypoint(snVector4f(10, 100, 0, 1), speed);

			entity->addPreUpdateComponent(path);

			//create text component
			ComponentFloatingText<EntityBox, int>* text = new ComponentFloatingText<EntityBox, int>();
			text->setAnchor(entity);
			text->addItem(L"KINEMATIC", 0, 0);
			text->setOffset(XMFLOAT2(-40, 10));
			entity->addPostUpdateComponent(text);

		}

		//this is a dynamic actor on the top of the kinematic actor
		{
			float width = 5;
			float height = 5;
			float depth = 5;

			//create the actor
			snActorDynamic* dyn = 0;
			int actId = -1;
			scene->createActorDynamic(&dyn, actId);
			dyn->setName("dynamic");
			dyn->setPosition(snVector4f(-2, 103.5, 0, 1));
			dyn->getPhysicMaterial().m_friction = 1.f;
			dyn->getPhysicMaterial().m_restitution = 0.f;

			//create collider
			snColliderBox* box = new snColliderBox();
			box->setSize(snVector4f(width, height, depth, 0));

			dyn->addCollider(box);
			dyn->updateMassAndInertia(100);
			dyn->initialize();

			//create entity
			EntityBox* entity = WORLD->createBox(XMFLOAT3(width, height, depth), m_colors[1]);
			entity->setActor(dyn);

			//create text component
			ComponentFloatingText<EntityBox, int>* text = new ComponentFloatingText<EntityBox, int>();
			text->setAnchor(entity);
			text->addItem(L"DYNAMIC", 0, 0);
			text->setOffset(XMFLOAT2(-40, 10));
			entity->addPostUpdateComponent(text);
		}

		//static right wall
		{
			float width = 1;
			float height = 45;
			float depth = 25;

			//create the actor
			snActorStatic* sta = 0;
			int actId = -1;
			scene->createActorStatic(&sta, actId, snVector4f(-15, 110, 0, 1), snVector4f(0, 0, 0, 1));
			sta->setName("static right wall");
			sta->getPhysicMaterial().m_friction = 1.f;
			sta->getPhysicMaterial().m_restitution = 1.f;

			//create collider
			snColliderBox* box = new snColliderBox();
			box->setSize(snVector4f(width, height, depth, 0));

			sta->addCollider(box);
			sta->initialize();

			//create entity
			EntityBox* entity = WORLD->createBox(XMFLOAT3(width, height, depth), m_colors[2]);
			entity->setActor(sta);

			//create text component
			ComponentFloatingText<EntityBox, int>* text = new ComponentFloatingText<EntityBox, int>();
			text->setAnchor(entity);
			text->addItem(L"STATIC", 0, 0);
			text->setOffset(XMFLOAT2(-40, 10));
			entity->addPostUpdateComponent(text);
		}

		//static left wall
		{
			float width = 1;
			float height = 45;
			float depth = 25;

			//create the actor
			snActorStatic* sta = 0;
			int actId = -1;
			scene->createActorStatic(&sta, actId, snVector4f(15, 110, 0, 1), snVector4f(0, 0, 0, 1));
			sta->setName("static left wall");
			sta->getPhysicMaterial().m_friction = 1.f;
			sta->getPhysicMaterial().m_restitution = 1.f;
			sta->addCollisionFlag(snCollisionFlag::CF_NO_CONTACT_RESPONSE);
			sta->addCollisionFlag(snCollisionFlag::CF_CONTACT_CALLBACK);
			sta->setOnCollisionCallback([](snIActor* const _a, snIActor* const _b)
			{
				string strAName = _a->getName();
				wstring _aName;
				_aName.assign(strAName.begin(), strAName.end());

				string strBName = _b->getName();
				wstring _bName;
				_bName.assign(strBName.begin(), strBName.end());
				wstring text = _aName + L" VS " + _bName;
				DEBUGGER->setWatchExpression(L"COLLISION CALLBACK", text);
			});

			//create collider
			snColliderBox* box = new snColliderBox();
			box->setSize(snVector4f(width, height, depth, 0));

			sta->addCollider(box);
			sta->initialize();

			//create entity
			EntityBox* entity = WORLD->createBox(XMFLOAT3(width, height, depth), m_colors[2]);
			entity->setActor(sta);

			//create text component
			ComponentFloatingText<EntityBox, int>* text = new ComponentFloatingText<EntityBox, int>();
			text->setAnchor(entity);
			text->addItem(L"STATIC", 0, 0);
			text->setOffset(XMFLOAT2(-40, 10));
			entity->addPostUpdateComponent(text);
		}
	}

	void SceneManager::createSceneDomino()
	{
		//initialize the world
		WORLD->initialize();

		WorldHUD* HUD = WORLD->createHUD();
		HUD->setSceneName(L"Scene : Domino");

		GRAPHICS->setClearScreenColor(Colors::DarkGray);

		//create the physics scene
		int sceneId = -1;
		snScene* scene = 0;
		SUPERNOVA->createScene(&scene, sceneId);
		scene->setCollisionMode(m_collisionMode);

		int solverIterationCount = 20;
		scene->setSolverIterationCount(solverIterationCount);
		scene->setContactConstraintBeta(0);
		scene->setLinearSquaredSpeedThreshold(0.000001f);
		scene->setAngularSquaredSpeedThreshold(0.000001f);
		scene->setGravity(snVector4f(0, -9.81f * 5, 0, 0));

		//create the camera.
		XMVECTOR cameraPosition = XMVectorSet(0, 100, -300, 1);
		XMVECTOR cameraLookAt = XMVectorSet(0, 0, 20, 1);
		XMVECTOR cameraUp = XMVectorSet(0, 1, 0, 0);
		WORLD->createCamera(cameraPosition, cameraLookAt, cameraUp);

		WORLD->deactivateCollisionPoint();

		createGround(scene, 1, 1);

		//create the path
		snVector4f dominoSize(5, 10, 1, 0);

		PathExplorer explorer(false);
		float distance = 4;
		float height = dominoSize[1] * 0.5f;
		explorer.addWaypoint(snVector4f(-20, height, 0, 1), distance);
		explorer.addWaypoint(snVector4f(-110, height, 0, 1), distance);
		explorer.addWaypoint(snVector4f(-10, height, -100, 1), distance);
		explorer.addWaypoint(snVector4f(-130, height, -100, 1), distance);

		explorer.addWaypoint(snVector4f(-130, height, 20, 1), distance);

		explorer.addWaypoint(snVector4f(110, height, 20, 1), distance);
		explorer.addWaypoint(snVector4f(110, height, -120, 1), distance);
		explorer.addWaypoint(snVector4f(0, height, 0, 1), distance);
		explorer.addWaypoint(snVector4f(0, height, -120, 1), distance);

		explorer.setCallback([](const snMatrix44f& _frenet)
		{
			//create actor
			snVector4f dominoSize(5, 10, 1, 0);
			snActorDynamic* actor = 0;
			int actorId = -1;
			SUPERNOVA->getScene(0)->createActorDynamic(&actor, actorId);
			actor->setPosition(_frenet[3]);

			//compute its orientation
			float cos = _frenet[0][0];
			float sin = _frenet[0][2];

			float theta = acosf(cos);
			if (sin < 0)
				theta = SN_PI * 2 - theta;

			snVector4f q = snQuaternionFromEuler(0, -theta, 0);

			actor->setOrientation(q);

			actor->getPhysicMaterial().m_friction = 0.1f;
			actor->getPhysicMaterial().m_restitution = 0.f;

			snColliderBox* collider = new snColliderBox();
			collider->setSize(dominoSize);

			actor->addCollider(collider);
			actor->updateMassAndInertia(2.f);
			actor->initialize();


			//ugly but it works
			const unsigned int COLOR_COUNT = 5;
			XMFLOAT4 colors[COLOR_COUNT];
			colors[0] = XMFLOAT4(0.8f, 1, 1, 1);
			colors[1] = XMFLOAT4(0.93f, 0.68f, 1, 1);
			colors[2] = XMFLOAT4(1, 0.8f, 0.678f, 1);
			colors[3] = XMFLOAT4(0.89f, 0.71f, 0.75f, 1);
			colors[4] = XMFLOAT4(0.96f, 0.48f, 0.63f, 1);

			//create entity
			EntityBox* box = WORLD->createBox(XMFLOAT3(dominoSize[0], dominoSize[1], dominoSize[2]), colors[actorId % COLOR_COUNT]);
			box->setActor(actor);
		});

		explorer.run();

		//create the hammer
		{
			snVector4f constraintOrigin(0, dominoSize[1] * 1.9f, -120, 1);
			snVector4f hammerOffset(0, dominoSize[1] * 0.9f, -dominoSize[1] * 0.9f, 1);
			snVector4f hammerPosition = snVector4f(0, dominoSize[1], -120, 1) + hammerOffset;
			float constraintDistance = (constraintOrigin - hammerPosition).norme();

			snActorDynamic* actor = 0;
			int actorId = -1;
			scene->createActorDynamic(&actor, actorId);
			actor->setPosition(hammerPosition);
			actor->setOrientation(snQuaternionFromEuler(0, 0, 0));
			actor->getPhysicMaterial().m_friction = 1;
			actor->getPhysicMaterial().m_restitution = 0.f;
			actor->setLinearDampingCoeff(0.5f);

			snColliderBox* collider = new snColliderBox();
			float size = 3;
			collider->setSize(snVector4f(size, size, size, 0));

			actor->addCollider(collider);
			actor->updateMassAndInertia(10);
			actor->initialize();

			//create entity
			EntityBox* box = WORLD->createBox(XMFLOAT3(size, size, size));
			box->setActor(actor);

			//create the fixed constraint
			snFixedConstraint* constraint = scene->createFixedConstraint(actor, constraintOrigin, constraintDistance);
			WORLD->createFixedConstraint(constraint);
		}

		//create kinematic component holding the hammer
		{
			snActorDynamic* actor = 0;
			int actorId = -1;
			snVector4f startPosition(0, dominoSize[1] + 7, -130, 1);
			snVector4f endPosition(0, dominoSize[1] + 7, -150, 1);
			scene->createActorDynamic(&actor, actorId);
			actor->setPosition(startPosition);
			actor->setOrientation(snQuaternionFromEuler(0, 0, 0));
			actor->getPhysicMaterial().m_friction = 1;
			actor->getPhysicMaterial().m_restitution = 0.f;
			actor->setIsKinematic(true);

			snColliderBox* collider = new snColliderBox();
			float width = 15;
			float height = 1;
			float depth = 10;
			collider->setSize(snVector4f(width, height, depth, 0));

			actor->addCollider(collider);
			actor->initialize();

			//create entity
			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), m_colors[4]);
			box->setActor(actor);
			box->setWireframe(false);

			//create component moving the entity
			//ComponentPathInterpolate* path = new ComponentPathInterpolate(actor, false);
			ComponentFollowPath* path = new ComponentFollowPath(actor, false);
			path->setIsActive(false);
			path->addWaypoint(startPosition, 2);
			path->addWaypoint(endPosition, 10);
			box->addPreUpdateComponent(path);
			m_dominoHammerBlockerPath = path;
		}

		//create trigger
		{
			snActorDynamic* actor = 0;
			int actorId = -1;
			scene->createActorDynamic(&actor, actorId);
			actor->setPosition(snVector4f(0, 100, -130, 1));
			actor->setOrientation(snQuaternionFromEuler(0, 0, 0));
			actor->getPhysicMaterial().m_friction = 1;
			actor->getPhysicMaterial().m_restitution = 0.f;

			//make it a kinematic trigger
			actor->setIsKinematic(true);
			actor->addCollisionFlag(snCollisionFlag::CF_NO_CONTACT_RESPONSE);
			actor->addCollisionFlag(snCollisionFlag::CF_CONTACT_CALLBACK);
			actor->setOnCollisionCallback([](snIActor * const _a, snIActor * const _b)
			{
				UNREFERENCED_PARAMETER(_a);
				UNREFERENCED_PARAMETER(_b);

				SCENEMGR->activateDominoSceneHammerBlocker();
			});

			snColliderBox* collider = new snColliderBox();
			float width = 30;
			float height = 30;
			float depth = 30;
			collider->setSize(snVector4f(width, height, depth, 0));

			actor->addCollider(collider);
			actor->initialize();

			//create entity
			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth));
			box->setActor(actor);
			box->setWireframe(true);

			ComponentFloatingText<EntityBox, float>* text = new ComponentFloatingText<EntityBox, float>();
			text->setAnchor(box);
			text->addItem(L"SHOOT ME!!!!", 0, 0);
			box->addPostUpdateComponent(text);
		}
	}

	void SceneManager::createGround(snScene* const _scene, float _restitution, float _friction)
	{
		float width = 500;
		float height = 2;
		float depth = 500;

		//create actor
		snActorStatic* act = 0;
		int actorId = -1;
		snVector4f position(0, -1, 0, 1);
		_scene->createActorStatic(&act, actorId, position, snVector4f(0, 0, 0, 1));
		act->setName("ground");
		act->getPhysicMaterial().m_restitution = _restitution;
		act->getPhysicMaterial().m_friction = _friction;

		//create collider
		snColliderBox* collider = new snColliderBox();
		collider->setSize(snVector4f(width, height, depth, 0));
		act->addCollider(collider);

		//initialize the actor
		act->initialize();

		//create the world entity
		EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
		kinematicBox->setActor(act);
	}

	void SceneManager::activateDominoSceneHammerBlocker()
	{
		m_dominoHammerBlockerPath->setIsActive(true);
	}

	void SceneManager::setCollisionMode(snCollisionMode _collisionMode)
	{
		m_collisionMode = _collisionMode;
	}

	SceneManager::SceneManager() : m_collisionMode(snCollisionMode::snECollisionModeSweepAndPrune)
	{
		m_colors[0] = XMFLOAT4(0.8f, 1, 1, 1);
		m_colors[1] = XMFLOAT4(0.93f, 0.68f, 1, 1);
		m_colors[2] = XMFLOAT4(1, 0.8f, 0.678f, 1);
		m_colors[3] = XMFLOAT4(0.89f, 0.71f, 0.75f, 1);
		m_colors[4] = XMFLOAT4(0.96f, 0.48f, 0.63f, 1);
	}

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
			snActorDynamic* act = 0;
			int actorId = -1;
			_scene->createActorDynamic(&act, actorId);

			snVector4f pos = pillarPosition[i] + _origin;
			act->setName("pillar1");
			
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0;
			act->getPhysicMaterial().m_friction = 1;

			//create collider
			snColliderBox* collider = new snColliderBox();
			collider->setSize(snVector4f(pillarWidth, pillarHeight, pillarDepth, 0));
			act->addCollider(collider);

			//initialize
			act->updateMassAndInertia(50);
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
			snActorDynamic* act = 0;
			int actorId = -1;
			_scene->createActorDynamic(&act, actorId);

			snVector4f pos = bedPosition[i] + _origin;
			act->setName("bed");
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0;
			act->getPhysicMaterial().m_friction = 1.f;

			//create collider
			snColliderBox* collider = new snColliderBox();
			collider->setSize(bedSize[i]);
			act->addCollider(collider);

			//initialize
			act->updateMassAndInertia(50);
			act->initialize();

			EntityBox* box = WORLD->createBox(XMFLOAT3(bedSize[i].getX(), bedSize[i].getY(), bedSize[i].getZ()), colors[actorId % colorCount]);
			box->setActor(act);
		}

		return _origin + snVector4f(0, pillarHeight + 2 * bedHeight, 0, 0);
	}

	
}