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

#include "GfxEntityHeightMap.h"

#include "snWorld.h"
#include "snScene.h"
#include "snRigidbody.h"
#include "snOBB.h"
#include "snSphere.h"
#include "snCapsule.h"

#include "snQuaternion.h"
#include "snDebugger.h"

#include "EntityBox.h"
#include "EntityCamera.h"
#include "EntityComposite.h"
#include "EntityStaticMesh.h"

#include "ComponentFollowPath.h"
#include "ComponentPathInterpolate.h"
#include "ComponentFloatingText.h"
#include "ComponentBackground.h"

#include "PathExplorer.h"
#include "TerrainCollider.h"
#include "snMath.h"

#include "TerrainData.h"
#include "TerrainLoader.h"
#include "TerrainDescription.h"
#include "TileId.h"
using namespace Devil::Terrain;

using namespace Supernova;
using namespace Supernova::Vector;

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
			//createSceneMonkeyBall();
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
		else if (INPUT->isKeyDown(120))//F9
		{
			clearScene();
			createSceneComposite();
			INPUT->keyUp(120);
		}
		else if (INPUT->isKeyDown('9'))//9
		{
			clearScene();
			createSceneGJK();
			INPUT->keyUp('9');
		}
		else if (INPUT->isKeyDown('8'))//9
		{
			clearScene();
			createSceneTerrain();
			INPUT->keyUp('8');
		}
	}

	void SceneManager::createBasicTest()
	{
		snhScene scene = createSandbox(L"Basic Test");
		scene->setSolverIterationCount(5);
		scene->setLinearSquaredSpeedThreshold(0.005f);
		scene->setGravity(snVec4Set(0, -9.81f * 2, 0, 0));
		WORLD->getCamera()->setPosition(snVec4Set(25, 30, 50, 1));
		WORLD->getCamera()->setLookAt(snVec4Set(15, 7, 0, 1));

		float groundHeight = 0;
		
		//first block on the ground
		float blockOneHeight = 0;
		{
			float width = 10;
			float height = 7;
			float depth = 10;

			snVec pos = snVec4Set(0, groundHeight + height * 0.5f, 0, 1);

			//create actor
			snhRigidbody act = SUPERNOVA->registerObject(new snRigidbody());
			scene->attachActor(act);
			act->setName("base");
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0.f;
			act->getPhysicMaterial().m_friction = 0.25f;

			//create collider
			snOBB* collider = new snOBB(snVec4Set(width, height, depth, 0) * 0.5f);
			SUPERNOVA->registerObject(collider);
			act->addCollider(collider);
			act->updateMassAndInertia(200);
			act->initializeDynamic();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), m_colors[1]);
			box->setActor(act.getPtr());

			blockOneHeight = snVec4GetY(pos) + height * 0.5f;
		}
	
		//platform
		float platformHeight = 0;
		{
			float width = 15;
			float height = 0.5;
			float depth = 3;
			snVec pos = snVec4Set(width * 0.5f, blockOneHeight + height * 0.5f + 0, 0, 1);

			//create actor
			snhRigidbody act = SUPERNOVA->registerObject(new snRigidbody());
			scene->attachActor(act);
			act->setName("platform");
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0.f;
			act->getPhysicMaterial().m_friction = 1.f;

			//create collider
			snOBB* collider = new snOBB(snVec4Set(width, height, depth, 0) * 0.5f);
			SUPERNOVA->registerObject(collider);
			act->addCollider(collider);
			act->updateMassAndInertia(100);
			act->initializeDynamic();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), m_colors[2]);
			box->setActor(act.getPtr());

			platformHeight = snVec4GetY(pos) + height * 0.5f;
		}
		
		//second block
		float blockTwoHeight = 0;
		{
			float width = 3;
			float height = 3;
			float depth = 3;
			snVec pos = snVec4Set(2, platformHeight + height * 0.5f, 0, 1);

			//create actor
			snhRigidbody act = SUPERNOVA->registerObject(new snRigidbody);
			scene->attachActor(act);
			act->setName("two");
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0.f;
			act->getPhysicMaterial().m_friction = 1.f;

			//create collider
			snOBB* collider = new snOBB(snVec4Set(width, height, depth, 0) * 0.5f);
			SUPERNOVA->registerObject(collider);
			act->addCollider(collider);
			act->updateMassAndInertia(100);
			act->initializeDynamic();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), m_colors[3]);
			box->setActor(act.getPtr());

			blockTwoHeight = snVec4GetY(pos) + height * 0.5f;
		}

		float blockThreeHeight = 0;
		{
			float width = 3;
			float height = 3;
			float depth = 3;
			snVec pos = snVec4Set(2, blockTwoHeight + height * 0.5f, 0, 1);

			//create actor
			snhRigidbody act = SUPERNOVA->registerObject(new snRigidbody);
			scene->attachActor(act);
			act->setName("three");
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0.f;
			act->getPhysicMaterial().m_friction = 1.f;

			//create collider
			snOBB* collider = new snOBB(snVec4Set(width, height, depth, 0) * 0.5f);
			SUPERNOVA->registerObject(collider);
			act->addCollider(collider);

			act->updateMassAndInertia(100);
			act->initializeDynamic();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), m_colors[4]);
			box->setActor(act.getPtr());

			blockThreeHeight = snVec4GetY(pos) + height * 0.5f;
		}
		
//float platformHeight = 10;
		//dynamic
		{
			float width = 3;
			float height = 3;
			float depth = 3;
			snVec pos = snVec4Set(11, platformHeight + height * 0.5f + 25, 0, 1);

			//create actor
			snhRigidbody act = SUPERNOVA->registerObject(new snRigidbody);
			scene->attachActor(act);
			act->setName("dynamic");
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0.f;
			act->getPhysicMaterial().m_friction = 1.f;

			//create collider
			snOBB* collider = new snOBB(snVec4Set(width, height, depth, 0) * 0.5f);
			SUPERNOVA->registerObject(collider);
			act->addCollider(collider);

			act->updateMassAndInertia(500);
			act->initializeDynamic();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), m_colors[0]);
			box->setActor(act.getPtr());

			blockTwoHeight = snVec4GetY(pos) + height * 0.5f;
		}
	}

	void SceneManager::createStacking()
	{
		snhScene scene = createSandbox(L"Stacking");
		scene->setSolverIterationCount(84);
		scene->setLinearSquaredSpeedThreshold(0.006f);
		scene->setAngularSquaredSpeedThreshold(0.0005f);
		scene->setGravity(snVec4Set(0, -9.81f * 2, 0, 0));
		WORLD->getCamera()->setPosition(snVec4Set(100, 79, 140, 1));
		WORLD->getCamera()->setLookAt(snVec4Set(35, 21, 0, 1));
	
		//ground
		float groundHeight = 0;
		
		//back wall
		{
			float width = 200;
			float height = 200;
			float depth = 5;

			//create actor
			snVec position = snVec4Set(0, 101, -80, 1);
			snRigidbody* rb = new snRigidbody();
			rb->setName("back");
			rb->getPhysicMaterial().m_restitution = 1;
			rb->getPhysicMaterial().m_friction = 1;
			
			
			//create collider
			snOBB* collider = new snOBB(snVec4Set(width, height, depth, 0) * 0.5f);
			rb->addCollider(collider);

			//initialize the rigidbody
			rb->initializeStatic(position, snVec4Set(0, 0, 0, 1));

			snhRigidbody act = SUPERNOVA->registerObject(rb);
			scene->attachActor(act);

			//create world entity
			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
			kinematicBox->setActor(act.getPtr());
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
				snVec pos = snVec4Set((MAX_ROW - row) * width * 0.5f + (width + space) * i + xOffset, groundHeight + height * 0.5f + height * (MAX_ROW - row), 0, 1);

				//create actor
				snhRigidbody act = SUPERNOVA->registerObject(new snRigidbody);
				scene->attachActor(act);

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
				snOBB* collider = new snOBB(snVec4Set(width, height, depth, 0) * 0.5);
				act->addCollider(collider);

				//initializeDynamic
				act->updateMassAndInertia(50);
				act->initializeDynamic();

				EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), m_colors[(i + row) % 5]);
				box->setActor(act.getPtr());
			}
		}
	}

	void SceneManager::createConstraints()
	{
		snhScene scene = createSandbox(L"Constraints");
		scene->setSolverIterationCount(4);
		scene->setLinearSquaredSpeedThreshold(0.006f);
		scene->setAngularSquaredSpeedThreshold(0.001f);
		scene->setGravity(snVec4Set(0, -9.81f * 4, 0, 0));
		WORLD->getCamera()->setPosition(snVec4Set(0, 80, 150, 1));
		WORLD->getCamera()->setLookAt(snVec4Set(10, 75, 0, 1));
		WORLD->activateCollisionPoint();

		//back wall
		{
			float width = 200;
			float height = 200;
			float depth = 5;

			//create actor
			snRigidbody* rb = new snRigidbody();
			rb->setName("back");
			rb->getPhysicMaterial().m_restitution = 1;
			rb->getPhysicMaterial().m_friction = 1;
			
			//create collider
			snOBB* collider = new snOBB(snVec4Set(width, height, depth, 0) * 0.5);
			rb->addCollider(collider);

			//initialize the rigidbody
			rb->initializeStatic(snVec4Set(0, 101, -80, 1), snVec4Set(0, 0, 0, 1));
			snhRigidbody act = SUPERNOVA->registerObject(rb);
			scene->attachActor(act);

			//create world entity
			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
			kinematicBox->setActor(act.getPtr());
		}

		float top = 100;
		snRigidbody* previousActor = 0;
		{
			float width = 2;
			float height = 5;
			float depth = 2;
			snVec pos = snVec4Set(10, top, 0, 1);

			//create actor
			snhRigidbody act = SUPERNOVA->registerObject(new snRigidbody);
			scene->attachActor(act);

			act->setName("d0");
			act->setPosition(pos);
			act->getPhysicMaterial().m_restitution = 0;
			act->getPhysicMaterial().m_friction = 1;
			act->setIsKinematic(false);
			previousActor = act.getPtr();

			//create collider
			snOBB* collider = new snOBB(snVec4Set(width, height, depth, 0) * 0.5);
			act->addCollider(collider);

			//initializeDynamic
			act->updateMassAndInertia(50);
			act->initializeDynamic();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), m_colors[4]);
			box->setActor(act.getPtr());

			//create constraints
			snFixedConstraint* constraint = scene->createFixedConstraint(act.getPtr(), pos + snVec4Set(0, 10, 0, 0), 10);
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
			snVec pos = snVec4Set(10, top, 0, 1);

			//create actor
			snhRigidbody act = SUPERNOVA->registerObject(new snRigidbody);
			scene->attachActor(act);

			act->setName("d1");
			act->setPosition(pos);
			act->getPhysicMaterial().m_restitution = 0;
			act->getPhysicMaterial().m_friction = 1;
			act->setIsKinematic(false);
			act->setAngularDampingCoeff(0.1f);
			act->setLinearDampingCoeff(0.1f);

			//create collider
			snOBB* collider = new snOBB(snVec4Set(width, height, depth, 0) * 0.5);
			act->addCollider(collider);

			//initializeDynamic
			act->updateMassAndInertia(50);
			act->initializeDynamic();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), m_colors[4]);
			box->setActor(act.getPtr());

			//create p2p constraint
			snPointToPointConstraint* p2pc = scene->createPointToPointConstraint(previousActor, snVec4Set(0, -LINK_LENGTH, 0, 1),
				act.getPtr(), snVec4Set(0, LINK_LENGTH, 0, 1));
			WORLD->createPointToPointConstraint(p2pc);

			previousActor = act.getPtr();
		}
	}

	void SceneManager::createTower()
	{
		snhScene scene = createSandbox(L"Tower");
		scene->setSolverIterationCount(120); //Wooooooooo that's a lot!!!!!!
		//scene->setContactConstraintBeta(0.005f);
		scene->setAngularSquaredSpeedThreshold(0.f);
		WORLD->getCamera()->setPosition(snVec4Set(50, 50, 100, 1));
		WORLD->getCamera()->setLookAt(snVec4Set(5, 7, 0, 1));
		WORLD->activateCollisionPoint();
		
		float groundHeight = 0;
		//back 	wall
		{
			float width = 200;
			float height = 200;
			float depth = 2;

			//create actor
			snRigidbody* rb = new snRigidbody();
			rb->setName("back");
			rb->getPhysicMaterial().m_restitution = 1;
			rb->getPhysicMaterial().m_friction = 1;

			//create collider				
			snOBB* 	collider = new snOBB(snVec4Set(width, height, depth, 0) * 0.5);
			rb->addCollider(collider);
				
			//initialize the rigidbody
			rb->initializeStatic(snVec4Set(0, 101, -80, 1), snVec4Set(0, 0, 0, 1));
			snhRigidbody act = SUPERNOVA->registerObject(rb);
			scene->attachActor(act);
			
			//create 	world 	entity	
			EntityBox* 	kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
			kinematicBox->setActor(act.getPtr());
			
		}

		//tower
		snVec origin = snVec4Set(0, groundHeight, 0, 1);
		int levelCount = 5;
		for (int i = 0; i < levelCount; ++i)		
		{
			origin = createTowerLevel(scene, origin);	
		}
	}

	void SceneManager::createSceneFriction()
	{
		snhScene scene = createSandbox(L"Friction");
		scene->setSolverIterationCount(4);
		WORLD->getCamera()->setPosition(snVec4Set(80, 50, 0, 1));
		WORLD->getCamera()->setLookAt(snVec4Set(0, 7, 0, 1));
		WORLD->activateCollisionPoint();

		float slopeAngle = 3.14f * 0.22f;
		//slope
		{
			float width = 50;
			float height = 2;
			float depth = 100;

			//create actor
			snRigidbody* rb = new snRigidbody();
			
			snhRigidbody act = SUPERNOVA->registerObject(rb);
			scene->attachActor(act);

			act->setName("slope");
			act->getPhysicMaterial().m_restitution = 1.f;
			act->getPhysicMaterial().m_friction = 1.f;
			
			//create collider
			snOBB* collider = new snOBB(snVec4Set(width, height, depth, 0) * 0.5);
			act->addCollider(collider);

			rb->initializeStatic(snVec4Set(0, 0, 0, 1), snQuaternionFromEuler(slopeAngle, 0, 0));

			//create the world entity
			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
			kinematicBox->setActor(act.getPtr());
		}

		createGround(scene, 0, 0.8f);
		
		XMFLOAT4 colors[5];
		colors[0] = XMFLOAT4(0.8f, 1, 1, 1);
		colors[1] = XMFLOAT4(0.93f, 0.68f, 1, 1);
		colors[2] = XMFLOAT4(1, 0.8f, 0.678f, 1);
		colors[3] = XMFLOAT4(0.89f, 0.71f, 0.75f, 1);
		colors[4] = XMFLOAT4(0.96f, 0.48f, 0.63f, 1);

		float height = 29.3f;
		//full friction, no restitution
		{
			//create actor
			snhRigidbody act = SUPERNOVA->registerObject(new snRigidbody);
			scene->attachActor(act);
			act->setName("full friction");
			act->setPosition(snVec4Set(-10, height, -30, 1));
			act->setOrientation(snQuaternionFromEuler(slopeAngle, 0, 0));
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = -1.f;
			act->getPhysicMaterial().m_friction = 1.f;

			//create collider
			float cubeSize = 5;
			snOBB* collider = new snOBB(snVec4Set(cubeSize, cubeSize, cubeSize, 0) * 0.5f);
			act->addCollider(collider);

			//initializeDynamic
			act->updateMassAndInertia(50);
			act->initializeDynamic();

			EntityBox* box = WORLD->createBox(XMFLOAT3(cubeSize, cubeSize, cubeSize), colors[0]);
			box->setActor(act.getPtr());
		}

		//no friction, no restitution
		{
			//create actor
			snhRigidbody act = SUPERNOVA->registerObject(new snRigidbody);
			scene->attachActor(act);

			act->setName("no friction");
			act->setPosition(snVec4Set(10, height, -30, 1));
			act->setOrientation(snQuaternionFromEuler(slopeAngle, 0, 0));
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = -1.f;
			act->getPhysicMaterial().m_friction = -1.f;

			//create collider
			float cubeSize = 5;
			snOBB* collider = new snOBB(snVec4Set(cubeSize, cubeSize, cubeSize, 0) * 0.5f);
			act->addCollider(collider);

			//initializeDynamic
			act->updateMassAndInertia(50);
			act->initializeDynamic();

			EntityBox* box = WORLD->createBox(XMFLOAT3(cubeSize, cubeSize, cubeSize), colors[2]);
			box->setActor(act.getPtr());
		}

		//half friction, no restitution
		{
			//create actor
			snhRigidbody act = SUPERNOVA->registerObject(new snRigidbody);
			scene->attachActor(act);

			act->setName("half friction");
			act->setPosition(snVec4Set(0, height, -30, 1));
			act->setOrientation(snQuaternionFromEuler(slopeAngle, 0, 0));
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = -1.f;
			act->getPhysicMaterial().m_friction = 0.f;

			//create collider
			float cubeSize = 5;
			snOBB* collider = new snOBB(snVec4Set(cubeSize, cubeSize, cubeSize, 0) * 0.5f);
			act->addCollider(collider);

			//initializeDynamic
			act->updateMassAndInertia(50);
			act->initializeDynamic();

			EntityBox* box = WORLD->createBox(XMFLOAT3(cubeSize, cubeSize, cubeSize), colors[1]);
			box->setActor(act.getPtr());
		}
	}

	void SceneManager::createSceneDamping()
	{
		snhScene scene = createSandbox(L"Damping");
		scene->setSolverIterationCount(4);
		WORLD->getCamera()->setPosition(snVec4Set(50, 90, 80, 1));
		WORLD->getCamera()->setLookAt(snVec4Set(50, 60, 0, 1));
		WORLD->activateCollisionPoint();

		/////ANGULAR DAMPING////////////
		const int BOX_COUNT = 5;
		snVec origin = snVec4Set(0, 70, 0, 1);
		for (int i = 0; i < BOX_COUNT; ++i)
		{
			float width = 5;
			float height = 5;
			float depth = 5;
			snVec pos = origin + snVec4Set(20.f * i, 0, 0, 0);

			//create actor
			snhRigidbody act = SUPERNOVA->registerObject(new snRigidbody);
			scene->attachActor(act);

			act->setName("d0");
			act->setPosition(pos);
			act->getPhysicMaterial().m_restitution = 0;
			act->getPhysicMaterial().m_friction = 1;
			act->setIsKinematic(false);
			act->setAngularDampingCoeff(i*0.2f);
			const float v = 5;
			act->setAngularVelocity(snVec4Set(0, v, 0, 0));

			//create collider
			snOBB* collider = new snOBB(snVec4Set(width, height, depth, 0) * 0.5f);
			act->addCollider(collider);

			//initializeDynamic
			act->updateMassAndInertia(50);
			act->initializeDynamic();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), m_colors[i]);
			box->setActor(act.getPtr());

			//create floating text component
			ComponentFloatingText<snRigidbody, float>* floatingText = new ComponentFloatingText<snRigidbody, float>();
			floatingText->setAnchor(box);
			floatingText->setOffset(XMFLOAT2(-120, 50));
			floatingText->addItem(L"Angular Damping", act.getPtr(), &snRigidbody::getAngularDampingCoeff);
			floatingText->addItem(L"Angular Speed", act.getPtr(), &snRigidbody::computeAngularSpeed);
			box->addPostUpdateComponent(floatingText);

			//create constraints
			snFixedConstraint* constraint = scene->createFixedConstraint(act.getPtr(), pos + snVec4Set(0, 10, 0, 0), 10);
			WORLD->createFixedConstraint(constraint);
		}

		/////////LINEAR DAMPING/////////////
		snVec4SetY(origin, 50);
		for (int i = 0; i < BOX_COUNT; ++i)
		{
			float width = 5;
			float height = 5;
			float depth = 5;
			snVec pos = origin + snVec4Set(20.f * i, 0, 0, 0);

			//create actor
			snhRigidbody act = SUPERNOVA->registerObject(new snRigidbody);
			scene->attachActor(act);

			act->setName("d0");
			act->setPosition(pos);
			act->getPhysicMaterial().m_restitution = 0;
			act->getPhysicMaterial().m_friction = 1;
			act->setIsKinematic(false);
			act->setLinearDampingCoeff(0.2f * i);
			const float v = 10;
			act->setLinearVelocity(snVec4Set(v, 0, v, 0));
			
			//create collider
			snOBB* collider = new snOBB(snVec4Set(width, height, depth, 0) * 0.5f);
			act->addCollider(collider);

			//initializeDynamic
			act->updateMassAndInertia(50);
			act->initializeDynamic();

			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), m_colors[i]);
			box->setActor(act.getPtr());

			//create floating text component
			ComponentFloatingText<snRigidbody, float>* floatingText = new ComponentFloatingText<snRigidbody, float>();
			floatingText->setAnchor(box);
			floatingText->setOffset(XMFLOAT2(-120, 50));
			floatingText->addItem(L"Linear Damping", act.getPtr(), &snRigidbody::getLinearDampingCoeff);
			floatingText->addItem(L"Linear Speed", act.getPtr(), &snRigidbody::computeLinearSpeed);
			box->addPostUpdateComponent(floatingText);

			//create constraints
			snFixedConstraint* constraint = scene->createFixedConstraint(act.getPtr(), pos + snVec4Set(0, 10, 0, 0), 10);
			WORLD->createFixedConstraint(constraint);
		}
	}

	void SceneManager::createSceneActorsType()
	{
		snhScene scene = createSandbox(L"Static, Dynamic, Kineatic");
		scene->setSolverIterationCount(4);

		WORLD->getCamera()->setPosition(snVec4Set(0, 110, 50, 1));
		WORLD->getCamera()->setLookAt(snVec4Set(0, 110, 0, 1));
		WORLD->activateCollisionPoint();

		//this is a kinematic platform
		{
			float width = 40;
			float height = 2;
			float depth = 40;

			//create the actor
			snhRigidbody kin = SUPERNOVA->registerObject(new snRigidbody);
			scene->attachActor(kin);
			kin->setIsKinematic(true);
			kin->setName("kinematic");
			kin->setPosition(snVec4Set(-15, 100, 0, 1));
			kin->getPhysicMaterial().m_friction = 1.f;
			kin->getPhysicMaterial().m_restitution = 0.f;
			kin->setLinearDampingCoeff(0.f);

			//create collider
			snOBB* box = new snOBB(snVec4Set(width, height, depth, 0) * 0.5f);
			kin->addCollider(box);
			kin->initializeDynamic();

			//create entity
			EntityBox* entity = WORLD->createBox(XMFLOAT3(width, height, depth), m_colors[0]);
			entity->setActor(kin.getPtr());

			//create path component
			ComponentFollowPath* path = new ComponentFollowPath(scene.getPtr(), kin.getPtr(), true);
			float speed = 5.f;

			path->addWaypoint(snVec4Set(-10, 100, 0, 1), speed);
			path->addWaypoint(snVec4Set(10, 100, 0, 1), speed);

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
			snhRigidbody dyn = SUPERNOVA->registerObject(new snRigidbody);
			scene->attachActor(dyn);
			dyn->setName("dynamic");
			dyn->setPosition(snVec4Set(-2, 103.5, 0, 1));
			dyn->getPhysicMaterial().m_friction = 1.f;
			dyn->getPhysicMaterial().m_restitution = 0.f;

			//create collider
			snOBB* box = new snOBB(snVec4Set(width, height, depth, 0) * 0.5f);
			dyn->addCollider(box);
			dyn->updateMassAndInertia(100);
			dyn->initializeDynamic();

			//create entity
			EntityBox* entity = WORLD->createBox(XMFLOAT3(width, height, depth), m_colors[1]);
			entity->setActor(dyn.getPtr());

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
			snRigidbody* rb = new snRigidbody();
			
			rb->setName("static right wall");
			rb->getPhysicMaterial().m_friction = 1.f;
			rb->getPhysicMaterial().m_restitution = 1.f;

			//create collider
			snOBB* box = new snOBB(snVec4Set(width, height, depth, 0) * 0.5f);
			rb->addCollider(box);
			rb->initializeDynamic();

			rb->initializeStatic(snVec4Set(-15, 110, 0, 1), snVec4Set(0, 0, 0, 1));
			snhRigidbody sta = SUPERNOVA->registerObject(rb);
			scene->attachActor(sta);

			//create entity
			EntityBox* entity = WORLD->createBox(XMFLOAT3(width, height, depth), m_colors[2]);
			entity->setActor(sta.getPtr());

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
			snRigidbody* rb = new snRigidbody();
			rb->setName("static left wall");
			rb->getPhysicMaterial().m_friction = 1.f;
			rb->getPhysicMaterial().m_restitution = 1.f;
			rb->addCollisionFlag(snCollisionFlag::CF_NO_CONTACT_RESPONSE);
			rb->addCollisionFlag(snCollisionFlag::CF_CONTACT_CALLBACK);
			rb->setOnCollisionCallback([](snRigidbody* const _a, snRigidbody* const _b)
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
			snOBB* box = new snOBB(snVec4Set(width, height, depth, 0) * 0.5f);
			rb->addCollider(box);
			rb->initializeDynamic();

			rb->initializeStatic(snVec4Set(15, 110, 0, 1), snVec4Set(0, 0, 0, 1));
			snhRigidbody sta = SUPERNOVA->registerObject(rb);
			scene->attachActor(sta);

			//create entity
			EntityBox* entity = WORLD->createBox(XMFLOAT3(width, height, depth), m_colors[2]);
			entity->setActor(sta.getPtr());

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
		snhScene scene = createSandbox(L"Domino");
		scene->setGravity(snVec4Set(0, -9.81f * 5, 0, 0));
		scene->setContactConstraintBeta(0.01f);
		scene->setSolverIterationCount(30);
		scene->setCollisionMode(snCollisionMode::snECollisionMode_ST_SweepAndPrune);

		WORLD->getCamera()->setPosition(snVec4Set(0, 100, -240, 1));
		WORLD->getCamera()->setLookAt(snVec4Set(0, 0, 20, 1));

		//create the path
		snVec dominoSize = snVec4Set(5, 10, 1, 0);

		PathExplorer explorer(false);
		float distance = 4;
		float height = snVec4GetY(dominoSize) * 0.5f;
		explorer.addWaypoint(snVec4Set(-20, height, 0, 1), distance);
		explorer.addWaypoint(snVec4Set(-110, height, 0, 1), distance);
		explorer.addWaypoint(snVec4Set(-10, height, -100, 1), distance);
		explorer.addWaypoint(snVec4Set(-130, height, -100, 1), distance);

		explorer.addWaypoint(snVec4Set(-130, height, 20, 1), distance);

		explorer.addWaypoint(snVec4Set(110, height, 20, 1), distance);
		explorer.addWaypoint(snVec4Set(110, height, -120, 1), distance);
		explorer.addWaypoint(snVec4Set(0, height, 0, 1), distance);
		explorer.addWaypoint(snVec4Set(0, height, -120, 1), distance);

		explorer.setCallback([](const snMatrix44f& _frenet)
		{
			//return;
			//create actor
			snVec dominoSize = snVec4Set(5, 10, 1, 0);
			snhRigidbody actor = SUPERNOVA->registerObject(new snRigidbody);
			WORLD->getPhysicsScene()->attachActor(actor);

			actor->setPosition(_frenet[3]);

			//compute its orientation
			float cos = snVec4GetX(_frenet[0]);
			float sin = snVec4GetZ(_frenet[0]);

			float theta = acosf(cos);
			if (sin < 0)
				theta = SN_PI * 2 - theta;

			snVec q = snQuaternionFromEuler(0, -theta, 0);

			actor->setOrientation(q);

			actor->getPhysicMaterial().m_friction = 0.1f;
			actor->getPhysicMaterial().m_restitution = 0.f;

			snOBB* collider = new snOBB(dominoSize * 0.5f);
			actor->addCollider(collider);
			actor->updateMassAndInertia(2.f);
			actor->initializeDynamic();


			//ugly but it works
			const unsigned int COLOR_COUNT = 5;
			XMFLOAT4 colors[COLOR_COUNT];
			colors[0] = XMFLOAT4(0.8f, 1, 1, 1);
			colors[1] = XMFLOAT4(0.93f, 0.68f, 1, 1);
			colors[2] = XMFLOAT4(1, 0.8f, 0.678f, 1);
			colors[3] = XMFLOAT4(0.89f, 0.71f, 0.75f, 1);
			colors[4] = XMFLOAT4(0.96f, 0.48f, 0.63f, 1);

			//create entity
			EntityBox* box = WORLD->createBox(XMFLOAT3(snVec4GetX(dominoSize), snVec4GetY(dominoSize), snVec4GetZ(dominoSize)), colors[actor.getId() % COLOR_COUNT]);
			box->setActor(actor.getPtr());
		});

		explorer.run();

		//create the hammer
		{
			snVec constraintOrigin = snVec4Set(0, snVec4GetY(dominoSize) * 1.9f, -120, 1);
			snVec hammerOffset = snVec4Set(0, snVec4GetY(dominoSize) * 0.9f, -snVec4GetY(dominoSize) * 0.9f, 1);
			snVec hammerPosition = snVec4Set(0, snVec4GetY(dominoSize), -120, 0) + hammerOffset;
			float constraintDistance = snVec3Norme(constraintOrigin - hammerPosition);

			snhRigidbody actor = SUPERNOVA->registerObject(new snRigidbody);
			scene->attachActor(actor);
			actor->setName("hammer");
			actor->setPosition(hammerPosition);
			actor->setOrientation(snQuaternionFromEuler(0, 0, 0));
			actor->getPhysicMaterial().m_friction = 1;
			actor->getPhysicMaterial().m_restitution = 0.f;
			actor->setLinearDampingCoeff(0.5f);

			float size = 3;
			snOBB* collider = new snOBB(snVec4Set(size, size, size, 0) * 0.5f);
			actor->addCollider(collider);
			actor->updateMassAndInertia(10);
			actor->initializeDynamic();

			//create entity
			EntityBox* box = WORLD->createBox(XMFLOAT3(size, size, size));
			box->setActor(actor.getPtr());

			//create the fixed constraint
			snFixedConstraint* constraint = scene->createFixedConstraint(actor.getPtr(), constraintOrigin, constraintDistance);
			WORLD->createFixedConstraint(constraint);
		}

		//create kinematic component holding the hammer
		{
			snVec startPosition = snVec4Set(0, snVec4GetY(dominoSize) + 7, -130, 1);
			snVec endPosition = snVec4Set(0, snVec4GetY(dominoSize) + 7, -150, 1);
			snhRigidbody actor = SUPERNOVA->registerObject(new snRigidbody());
			scene->attachActor(actor);
			actor->setName("hold");
			actor->setPosition(startPosition);
			actor->setOrientation(snQuaternionFromEuler(0, 0, 0));
			actor->getPhysicMaterial().m_friction = 1;
			actor->getPhysicMaterial().m_restitution = 0.f;
			actor->setIsKinematic(true);

			
			float width = 15;
			float height = 1;
			float depth = 10;
			snOBB* collider = new snOBB(snVec4Set(width, height, depth, 0) * 0.5f);

			actor->addCollider(collider);
			actor->initializeDynamic();

			//create entity
			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth), m_colors[4]);
			box->setActor(actor.getPtr());
			box->setWireframe(false);

			//create component moving the entity
			//ComponentPathInterpolate* path = new ComponentPathInterpolate(actor, false);
			ComponentFollowPath* path = new ComponentFollowPath(scene.getPtr(), actor.getPtr(), false);
			path->setIsActive(false);
			path->addWaypoint(startPosition, 2);
			path->addWaypoint(endPosition, 10);
			box->addPreUpdateComponent(path);
			m_dominoHammerBlockerPath = path;
		}

		//create trigger
		{
			snhRigidbody actor = SUPERNOVA->registerObject(new snRigidbody);
			scene->attachActor(actor);
			actor->setPosition(snVec4Set(0, 100, -130, 1));
			actor->setOrientation(snQuaternionFromEuler(0, 0, 0));
			actor->getPhysicMaterial().m_friction = 1;
			actor->getPhysicMaterial().m_restitution = 0.f;

			//make it a kinematic trigger
			actor->setIsKinematic(true);
			actor->addCollisionFlag(snCollisionFlag::CF_NO_CONTACT_RESPONSE);
			actor->addCollisionFlag(snCollisionFlag::CF_CONTACT_CALLBACK);
			actor->setOnCollisionCallback([](snRigidbody * const _a, snRigidbody * const _b)
			{
				UNREFERENCED_PARAMETER(_a);
				UNREFERENCED_PARAMETER(_b);

				SCENEMGR->activateDominoSceneHammerBlocker();
			});

			float width = 30;
			float height = 30;
			float depth = 30;
			snOBB* collider = new snOBB(snVec4Set(width, height, depth, 0) * 0.5f);

			actor->addCollider(collider);
			actor->initializeDynamic();

			//create entity
			EntityBox* box = WORLD->createBox(XMFLOAT3(width, height, depth));
			box->setActor(actor.getPtr());
			box->setWireframe(true);

			ComponentFloatingText<EntityBox, float>* text = new ComponentFloatingText<EntityBox, float>();
			text->setAnchor(box);
			text->addItem(L"SHOOT ME!!!!", 0, 0);
			box->addPostUpdateComponent(text);
		}
	}

	void SceneManager::createSceneComposite()
	{
		snhScene scene = createSandbox(L"Gears");
		scene->setGravity(snVec4Set(0, -9.81f * 5, 0, 0));
		scene->setContactConstraintBeta(0.05f);
		scene->setSolverIterationCount(30);
		scene->setCollisionMode(snCollisionMode::snECollisionMode_ST_SweepAndPrune);

		WORLD->getCamera()->setPosition(snVec4Set(10, 30, -120, 1));
		//WORLD->getCamera()->setPosition(snVec4Set(0, 10, -580, 1));
		WORLD->getCamera()->setLookAt(snVec4Set(10, 30, 0, 1));
		WORLD->activateCollisionPoint();

	
		float length = 30;
		float thickness = 2.8f;
		float depth = 5;
		float distance = length * 0.85f;

		snVec position = snVec4Set(0, 19, 0, 1);
		snVec orientation = snVec4Set(0, 0, 0, 1);
		EntityComposite* entity = createWheel(position, orientation, m_colors[1], length);	
		scene->createHingeConstraint(entity->getActor(), snVec4Set(0, 0, 1, 0), position);

		position = snVec4Set(-distance, 16, 0, 1);
		entity = createWheel(position, orientation, m_colors[4], length);
		scene->createHingeConstraint(entity->getActor(), snVec4Set(0, 0, 1, 0), position);

		position = snVec4Set(distance, 16, 0, 1);
		entity = createWheel(position, orientation, m_colors[2], length);
		scene->createHingeConstraint(entity->getActor(), snVec4Set(0, 0, 1, 0), position);

		position = position + snVec4Set(distance, 2, 0, 0);
		entity = createWheel(position, orientation, m_colors[0], length);
		scene->createHingeConstraint(entity->getActor(), snVec4Set(0, 0, 1, 0), position);

		position = position + snVec4Set(0, distance, 0, 0);

		snhRigidbody act = SUPERNOVA->registerObject(new snRigidbody);
		scene->attachActor(act);

		act->setName("composite");
		//act->setPosition(snVec4Set(0, 20, 0, 1));
		act->setPosition(position);
		act->setOrientation(snVec4Set(0, 0, 0.2f, 1));
		act->getPhysicMaterial().m_restitution = 0;
		act->setAngularDampingCoeff(0.f);
		act->setIsKinematic(true);
		act->setAngularVelocity(snVec4Set(0, 0, 1.f, 0));

		snMatrix44f localTranslation;
		localTranslation.createTranslation(snVec4Set(0, 0, 0, 1));

		const int PIN_COUNT = 6;
		float angle = SN_PI / PIN_COUNT;
		for (int pinId = 0; pinId < PIN_COUNT; ++pinId)
		{
			//create the pin
			snOBB* xBox = new snOBB(snVec4Set(length, thickness, depth, 0) * 0.5f);
			xBox->getTransform().setLocalEulerAngles(0, 0, angle * pinId);
			act->addCollider(xBox);

			//fill in the space between the pins
			snOBB* fill = new snOBB(snVec4Set(length * 0.7f, 4, depth, 0) * 0.5f);
			fill->getTransform().setLocalEulerAngles(0, 0, angle * (pinId + 0.5f));
			act->addCollider(fill);
		}

		act->initializeDynamic();

		WORLD->createComposite(act.getPtr(), m_colors[3]);

		////Create a sphere
		//int actSphereId = -1;
		//snRigidbody* actSphere = 0;
		//scene->createActorDynamic(&actSphere, actSphereId);
		//actSphere->setPosition(snVec4Set(0, 35, 0, 1));
		//actSphere->addCollisionFlag(snCollisionFlag::CF_CONTACT_CALLBACK);
		//actSphere->getPhysicMaterial().m_restitution = 0;
		//snColliderSphere* collider = new snColliderSphere(2);

		//actSphere->addCollider(collider);
		//actSphere->updateMassAndInertia(10);
		//actSphere->initializeDynamic();

		//actSphere->setOnCollisionCallback([](snIActor* _a1, snIActor* _a2)
		//{
		//	DEBUGGER->setWatchExpression(L"COLLISION", L"SPHERE");
		//});

		//WORLD->createComposite(actSphere, m_colors[2]);
	}

	void SceneManager::createSceneMonkeyBall()
	{
		//createSandbox(L"Monkey Ball");
		//initializeDynamic the world
		WORLD->initialize();

		WorldHUD* HUD = WORLD->createHUD();
		HUD->setSceneName(L"Scene : Monkey Ball");

		GRAPHICS->setClearScreenColor(DirectX::Colors::DarkGray);

		//create the physics scene
		snhScene scene = SUPERNOVA->registerObject(new snScene());
		WORLD->setPhysicsScene(scene);
		scene->setCollisionMode(m_collisionMode);

		//scene->setLinearSquaredSpeedThreshold(0.000001f);
		//scene->setAngularSquaredSpeedThreshold(0.000001f);
		scene->setLinearSquaredSpeedThreshold(0.00001f);
		scene->setAngularSquaredSpeedThreshold(0.00001f);

		//create the camera.
		EntityCamera* camera = WORLD->createCamera(snVec4Set(0, 100, -100, 1), snVec4Set(0, 0, 0, 1), snVec4Set(0, 1, 0, 0));
		camera->setCameraModeFollowTarget();

		WORLD->deactivateCollisionPoint();

		scene->setGravity(snVec4Set(0, -9.81f * 20, 0, 0));
		scene->setContactConstraintBeta(0.05f);
		scene->setSolverIterationCount(30);
		scene->setCollisionMode(snCollisionMode::snECollisionMode_ST_SweepAndPrune);
		scene->setContactConstraintBeta(0.3f);

		WORLD->getCamera()->setPosition(snVec4Set(10, 120, -180, 1));
		WORLD->getCamera()->setLookAt(snVec4Set(10, 30, 0, 1));

		snhRigidbody actEnvironment = SUPERNOVA->registerObject(new snRigidbody);
		scene->attachActor(actEnvironment);
		actEnvironment->setName("level");

		const float THICKNESS = 20;
		snOBB* levelOne = new snOBB(snVec4Set(200, THICKNESS, 200, 0) * 0.5f);
		actEnvironment->addCollider(levelOne);

		levelOne = new snOBB(snVec4Set(200, THICKNESS, 50, 0) * 0.5f);
		levelOne->getTransform().setLocalPosition(snVec4Set(200, 0, 0, 1));
		actEnvironment->addCollider(levelOne);

		levelOne = new snOBB(snVec4Set(50, THICKNESS, 200, 0) * 0.5f);
		levelOne->getTransform().setLocalPosition(snVec4Set(325, 0, 75, 1));
		actEnvironment->addCollider(levelOne);

		levelOne = new snOBB(snVec4Set(200, THICKNESS, 50, 0) * 0.5f);
		levelOne->getTransform().setLocalPosition(snVec4Set(450, 0, 150, 1));
		actEnvironment->addCollider(levelOne);

		levelOne = new snOBB(snVec4Set(50, THICKNESS, 200, 0) * 0.5f);
		levelOne->getTransform().setLocalPosition(snVec4Set(575, 0, 75, 1));
		actEnvironment->addCollider(levelOne);

		levelOne = new snOBB(snVec4Set(200, THICKNESS, 50, 0) * 0.5f);
		levelOne->getTransform().setLocalPosition(snVec4Set(700, 0, 0, 1));
		actEnvironment->addCollider(levelOne);

		levelOne = new snOBB(snVec4Set(200, THICKNESS, 200, 0) * 0.5f);
		levelOne->getTransform().setLocalPosition(snVec4Set(900, 0, 0, 1));
		actEnvironment->addCollider(levelOne);

		
		snVec initialPosition = snVec4Set(0, 50, 0, 1);
		snVec initialOrientation = snVec4Set(0, 0, 0, 1);
		actEnvironment->setPosition(initialPosition);
		actEnvironment->setIsKinematic(true);
		actEnvironment->getPhysicMaterial().m_restitution = 0;
		actEnvironment->getPhysicMaterial().m_friction = 1;
		actEnvironment->initializeDynamic();

		EntityComposite* entity = WORLD->createComposite(actEnvironment.getPtr(), XMFLOAT4(1, 1, 1, 1));
		entity->setTexture(GRAPHICS->getTexChecker());

		//Create ball
		snhRigidbody ball = SUPERNOVA->registerObject(new snRigidbody);
		scene->attachActor(ball);
		float sphereRadius = 10;
		snSphere* sphere = new snSphere(sphereRadius);
		ball->addCollider(sphere);
		ball->updateMassAndInertia(1);
		ball->setPosition(snVec4Set(-50, 60 + sphereRadius, 0, 1));
		ball->getPhysicMaterial().m_restitution = 0;
		ball->getPhysicMaterial().m_friction = 1;
		ball->setName("ball");
		ball->initializeDynamic();

		//WORLD->createComposite(ball, m_colors[3]);
		EntityComposite* monkeyBall = WORLD->createMonkeyBall(ball.getPtr(), m_colors[4]);
		WORLD->createSkybox(monkeyBall, 1000, XMFLOAT4(1, 1, 1, 1));

		ComponentBackground* cBack = new ComponentBackground(actEnvironment.getPtr(), ball.getPtr(), initialPosition, initialOrientation);
		entity->addPreUpdateComponent(cBack);

	}

	void SceneManager::createSceneGJK()
	{
		snhScene scene = createSandbox(L"GJK");
		scene->setContactConstraintBeta(0.005f);

		////create box reference
		//snRigidbody* actBox = 0;
		//int boxId = -1;
		//scene->createActorDynamic(&actBox, boxId);

		//snOBB* colBox = new snOBB(snVec4Set(5, 5, 5, 0));
		//actBox->setPosition(snVec4Set(-10, 15, 0, 1));
		////actBox->setOrientation(snVec4Set(0, 0, 0, 1));
		//actBox->setOrientation(snQuaternionFromEuler(0, 0, -3.14f * 0.15f));
		//actBox->addCollider(colBox);
		//actBox->updateMassAndInertia(10);
		//actBox->initializeDynamic();

		//EntityComposite* entityBox = WORLD->createComposite(actBox, m_colors[2]);
		//entityBox->setWireframe(true);

		//create cylinder

		const unsigned int SHAPE_COUNT = 10;
		snVec offsetPosition = snVec4Set(0, 20, 0, 0);
		float orientationOffset = 2 * 3.14f * (1.f / SHAPE_COUNT);
		for (unsigned int i = 0; i < SHAPE_COUNT; ++i)
		{
			snhRigidbody actCapsule = SUPERNOVA->registerObject(new snRigidbody);
			scene->attachActor(actCapsule);
			snCapsule* colCapsule = new snCapsule(10, 5);
			/*actCapsule->setPosition(snVec4Set(0, 15, 0, 1));
			actCapsule->setOrientation(snQuaternionFromEuler( 0, 0.f, 0));*/
			actCapsule->setPosition(snVec4Set(0, 15, 0, 1) + (float)i * offsetPosition);
			actCapsule->setOrientation(snQuaternionFromEuler(i * orientationOffset, 0.f, i * orientationOffset));
			//actCapsule->setLinearVelocity(snVec4Set(0, 15, 0, 0));
			//actCapsule->setAngularVelocity(snVec4Set(-5, 2, 0, 0));
			actCapsule->setAngularDampingCoeff(0.2f);
			actCapsule->addCollider(colCapsule);
			actCapsule->getPhysicMaterial().m_friction = 1.f;
			actCapsule->getPhysicMaterial().m_restitution = 0.f;
			//actCapsule->setIsKinematic(true);
			actCapsule->updateMassAndInertia(10);
			actCapsule->initializeDynamic();
			EntityComposite* comp = WORLD->createComposite(actCapsule.getPtr(), m_colors[i%4]);
			comp->setWireframe(true);
		}

		//{
		//	scene->createActorDynamic(&actCapsule, cylinderId);
		//	snCapsule* colCapsule = new snCapsule(10, 5);
		//	//snOBB* colCapsule = new snOBB(snVec4Set(5, 5, 5, 0));
		//	actCapsule->setPosition(snVec4Set(0.1, 35, 0, 1));
		//	actCapsule->setOrientation(snQuaternionFromEuler(0, 0, 3.14f * 0.5f));
		//	//actCapsule->setLinearVelocity(snVec4Set(0, -3, 0, 0));
		//	//actCapsule->setAngularVelocity(snVec4Set(0, 0, 5, 0));
		//	actCapsule->addCollider(colCapsule);
		//	actCapsule->getPhysicMaterial().m_friction = 1.f;
		//	actCapsule->getPhysicMaterial().m_restitution = 0.f;
		//	//actCapsule->setIsKinematic(true);
		//	actCapsule->updateMassAndInertia(10);
		//	actCapsule->initializeDynamic();
		//	EntityComposite* comp = WORLD->createComposite(actCapsule, m_colors[3]);
		//	comp->setWireframe(true);
		//}

		//set camera position
		WORLD->getCamera()->setPosition(snVec4Set(0, 80, -100, 1));
		WORLD->getCamera()->setLookAt(snVec4Set(0, 20, 0, 1));
		WORLD->activateCollisionPoint();
	}

	void SceneManager::createSceneTerrain()
	{
		//initializeDynamic the world
		WORLD->initialize();

		WorldHUD* HUD = WORLD->createHUD();
		HUD->setSceneName(L"Scene : Terrain");

		GRAPHICS->setClearScreenColor(DirectX::Colors::DarkGray);

		//create the physics scene
		snhScene scene = SUPERNOVA->registerObject(new snScene());
		WORLD->setPhysicsScene(scene);
		scene->setCollisionMode(m_collisionMode);

		scene->setLinearSquaredSpeedThreshold(0.000001f);
		scene->setAngularSquaredSpeedThreshold(0.000001f);
		snVec gravity = snVec4Set(0, -9.81f, 0, 0);
		scene->setGravity(gravity * 4);

		
		//snVec spawnPosition = snVec4Set(-61470, 800, -61470, 1);
		//snVec spawnPosition = snVec4Set(-61470, 80, -30750, 1);
		snVec spawnPosition = snVec4Set(0, 500, -400, 1);
		EntityCamera* camera = WORLD->createCamera(snVec4Set(0, 100, -100, 1), snVec4Set(0, 0, 0, 1), snVec4Set(0, 1, 0, 0));
		WORLD->getCamera()->setPosition(spawnPosition);
		WORLD->getCamera()->setLookAt(snVec4Set(0, 400, 0, 1));
		
		/*float size = 30;
		unsigned int tileResolution = 1;
		string bitmapFilename = "C:\\Users\\Florent\\Desktop\\terrain_slope_8bits_256_256.bmp";*/

		float size = 30;
		unsigned int tileResolution = 16;
		string bitmapFilename = "data\\terrain_8bits_4098_4098.bmp";


		/*IWorldEntity* entity = (IWorldEntity*)*/ WORLD->createTerrain(bitmapFilename, tileResolution, tileResolution, size, -200, 800, camera);
		//entity->setWireframe(true);

		//TerrainDescription desc;
		//if (!desc.initFromBitmap8(bitmapFilename, tileResolution, tileResolution, size, -200, 800))
		//	throw;

		//TileId tileToLoad;// = desc.getCurrentTile(spawnPosition);
		//tileToLoad.m_columnId = 0;
		//tileToLoad.m_rowId = 0;
		//TerrainLoader loader;
		//TerrainData data;
		//{
		//	if (!loader.loadTile(desc, tileToLoad, data))
		//		throw;

		//	snVec lowerLeftCorner = desc.computeTileLowerLeftCorner(tileToLoad);
		//	unsigned int gfxId = GRAPHICS->createHeightMap(lowerLeftCorner, size, desc.getQuadsPerTileRow(), desc.getQuadsPerTileColumn(), data.m_heights);
		//	IGfxEntity* gfx = GRAPHICS->getEntity(gfxId);
		//	EntityStaticMesh* entity = WORLD->createStaticMesh(gfx);
		//	entity->setWireframe(true);

		//	snAABB boundingVolume;
		//	boundingVolume.m_min = lowerLeftCorner;
		//	snVec4SetY(boundingVolume.m_min, data.m_min);

		//	boundingVolume.m_max = boundingVolume.m_min + snVec4Set(data.m_quadsPerRow * size, 0, data.m_quadsPerRow * size, 0);
		//	snVec4SetY(boundingVolume.m_max, data.m_max);

		//	//create the physic height map
		//	snRigidbody* snMap;
		//	int id = -1;
		//	scene->createActorStatic(&snMap, id, snVec4Set(0), snVec4Set(0));
		//	TerrainCollider* collider = new	TerrainCollider(boundingVolume.m_min, boundingVolume.m_max, size, data.m_quadsPerRow, data.m_quadsPerRow, data.m_heights);
		//	snMap->addCollider(collider);
		//	snMap->getPhysicMaterial().m_friction = 1;
		//	snMap->setName("terrain");
		//	snMap->initializeDynamic();
		//}

		const unsigned int SPHERE_COUNT = 20;
		float radius = 10;
		snVec offset = snVec4Set(0, 40, 0, 0);
		for (int i = 0; i < SPHERE_COUNT; ++i)
		{
			//create a dynamic sphere
			snhRigidbody sphere = SUPERNOVA->registerObject(new snRigidbody);
			scene->attachActor(sphere);
			snICollider* collider = 0;
			int mod = i % 3;
			if (mod == 0)
				collider = new snSphere(radius);
			else if (mod == 1)
				collider = new snOBB(snVec4Set(radius, radius, radius, 0));
			else
				collider = new snCapsule(radius * 2, radius);
			sphere->addCollider(collider);
			sphere->setPosition(snVec4Set(0, 500, -5, 1) + offset * (float)i);
			//sphere->setOrientation(snVec4Set(0, 0, 0, 1));
			sphere->setOrientation(snQuaternionFromEuler(0, 0.1f, 3.14f));
			//sphere->setAngularVelocity(snVec4Set(10, 0, 0, 0));
			sphere->updateMassAndInertia(10);
			sphere->setAngularDampingCoeff(0.1f);
			sphere->getPhysicMaterial().m_friction = 1;
			sphere->initializeDynamic();

			/*EntityComposite* entity =*/ WORLD->createComposite(sphere.getPtr(), m_colors[i % 4]);
			//entity->setWireframe(true);
		}

		//create the box launcher
		WORLD->createEntityBoxLauncher(1);

		WORLD->deactivateCollisionPoint();
	}

	void SceneManager::createGround(snhScene const _scene, float _restitution, float _friction)
	{
		float width = 500;
		float height = 2;
		float depth = 500;

		//create actor
		snVec position = snVec4Set(0, -1, 0, 1);
		snRigidbody* rb = new snRigidbody();
		
		snhRigidbody act = SUPERNOVA->registerObject(rb);
		_scene->attachActor(act);
		act->setName("ground");
		act->getPhysicMaterial().m_restitution = _restitution;
		act->getPhysicMaterial().m_friction = _friction;

		//create collider
		snOBB* collider = new snOBB(snVec4Set(width, height, depth, 0) * 0.5f);
		act->addCollider(collider);

		//initialize the actor
		act->initializeStatic(position, snVec4Set(0, 0, 0, 1));

		//create the world entity
		EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(width, height, depth));
		kinematicBox->setActor(act.getPtr());
	}

	void SceneManager::activateDominoSceneHammerBlocker()
	{
		m_dominoHammerBlockerPath->setIsActive(true);
	}

	void SceneManager::setCollisionMode(snCollisionMode _collisionMode)
	{
		m_collisionMode = _collisionMode;
	}

	SceneManager::SceneManager() : m_collisionMode(snCollisionMode::snECollisionMode_ST_SweepAndPrune)
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
		WORLD->getPhysicsScene().release();
		WORLD->clearWorld();
	}

	snVec SceneManager::createTowerLevel(snhScene const _scene, const snVec& _origin) const
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
		snVec pillarPosition[pillarCount];

		float halfTowerWidth = towerWidth * 0.5f;
		float halfPillarHeight = pillarHeight * 0.5f;
		pillarPosition[0] = snVec4Set(halfTowerWidth, halfPillarHeight, halfTowerWidth, 0);
		pillarPosition[1] = snVec4Set(-halfTowerWidth, halfPillarHeight, halfTowerWidth, 0);
		pillarPosition[2] = snVec4Set(-halfTowerWidth, halfPillarHeight, -halfTowerWidth, 0);
		pillarPosition[3] = snVec4Set(halfTowerWidth, halfPillarHeight, -halfTowerWidth, 0);

		//create pillars
		for (int i = 0; i < pillarCount; ++i)
		{
			//create actor
			snhRigidbody act = SUPERNOVA->registerObject(new snRigidbody);
			_scene->attachActor(act);

			snVec pos = pillarPosition[i] + _origin;
			act->setName("pillar1");
			
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0;
			act->getPhysicMaterial().m_friction = 1;

			//create collider
			snOBB* collider = new snOBB(snVec4Set(pillarWidth, pillarHeight, pillarDepth, 0) * 0.5f);
			act->addCollider(collider);

			//initializeDynamic
			act->updateMassAndInertia(50);
			act->initializeDynamic();

			EntityBox* box = WORLD->createBox(XMFLOAT3(pillarWidth, pillarHeight, pillarDepth), colors[act.getId() % colorCount]);
			box->setActor(act.getPtr());
		}

		float bedWidth = towerWidth + 2 * pillarWidth;
		float bedDepth = pillarDepth * 2;
		float bedHeight = 1;

		float firstBedHeight = pillarHeight + bedHeight * 0.5f;
		float secondBedHeight = firstBedHeight + bedHeight;

		snVec bedPosition[4];
		bedPosition[0] = snVec4Set(0, firstBedHeight, halfTowerWidth, 0);
		bedPosition[1] = snVec4Set(0, firstBedHeight, -halfTowerWidth, 0);
		bedPosition[2] = snVec4Set(halfTowerWidth, secondBedHeight, 0, 0);
		bedPosition[3] = snVec4Set(-halfTowerWidth, secondBedHeight, 0, 0);

		snVec bedSize[4];
		bedSize[0] = snVec4Set(bedWidth, bedHeight, bedDepth, 0);
		bedSize[1] = snVec4Set(bedWidth, bedHeight, bedDepth, 0);
		bedSize[2] = snVec4Set(bedDepth, bedHeight, bedWidth, 0);
		bedSize[3] = snVec4Set(bedDepth, bedHeight, bedWidth, 0);

		//create bed between pillars
		for (int i = 0; i < pillarCount; ++i)
		{
			//create actor
			snhRigidbody act = SUPERNOVA->registerObject(new snRigidbody);
			_scene->attachActor(act);

			snVec pos = bedPosition[i] + _origin;
			act->setName("bed");
			act->setPosition(pos);
			act->setIsKinematic(false);
			act->getPhysicMaterial().m_restitution = 0;
			act->getPhysicMaterial().m_friction = 1.f;

			//create collider
			snOBB* collider = new snOBB(bedSize[i] * 0.5f);
			act->addCollider(collider);

			//initializeDynamic
			act->updateMassAndInertia(50);
			act->initializeDynamic();

			EntityBox* box = WORLD->createBox(XMFLOAT3(snVec4GetX(bedSize[i]), snVec4GetY(bedSize[i]), snVec4GetZ(bedSize[i])), colors[act.getId() % colorCount]);
			box->setActor(act.getPtr());
		}

		return _origin + snVec4Set(0, pillarHeight + 2 * bedHeight, 0, 0);
	}

	snhScene SceneManager::createSandbox(const wstring& _sceneName)
	{
		//initializeDynamic the world
		WORLD->initialize();

		WorldHUD* HUD = WORLD->createHUD();
		HUD->setSceneName(L"Scene : " + _sceneName);

		GRAPHICS->setClearScreenColor(DirectX::Colors::DarkGray);

		//create the physics scene
		snhScene scene = SUPERNOVA->registerObject(new snScene());
		WORLD->setPhysicsScene(scene);
		scene->setCollisionMode(m_collisionMode);

		scene->setLinearSquaredSpeedThreshold(0.000001f);
		scene->setAngularSquaredSpeedThreshold(0.000001f);

		//create the camera.
		WORLD->createCamera(snVec4Set(0, 100, -100, 1), snVec4Set(0, 0, 0, 1), snVec4Set(0, 1, 0, 0));

		WORLD->deactivateCollisionPoint();

		//create the box launcher
		WORLD->createEntityBoxLauncher(1);

		createGround(scene, 1, 1);

		//create left wall 
		{
			snRigidbody* rb = new snRigidbody();
			
			//create collider
			snOBB* collider = new snOBB(snVec4Set(10, 500, 500, 0) * 0.5f);
			rb->addCollider(collider);
			rb->addCollisionFlag(snCollisionFlag::CF_NO_CONTACT_RESPONSE);
			rb->addCollisionFlag(snCollisionFlag::CF_CONTACT_CALLBACK);
			rb->setOnCollisionCallback([](snRigidbody* const _me, snRigidbody* const _other)
			{
				UNREFERENCED_PARAMETER(_me);
				IWorldEntity* entity = WORLD->getEntityFromActor(_other);
				if (entity != 0)
					entity->setIsActive(false);

				_other->setIsActive(false);
			});

			//initializethe rigidbody
			rb->initializeStatic(snVec4Set(250, 250, 0, 1), snVec4Set(0, 0, 0, 1));

			snhRigidbody stat = SUPERNOVA->registerObject(rb);
			scene->attachActor(stat);

			//create the world entity
			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(10, 500, 500));
			kinematicBox->setActor(stat.getPtr());
			kinematicBox->setWireframe(true);
		}

		//create right wall 
		{
			snRigidbody* rb = new snRigidbody();
				
			//create collider
			snOBB* collider = new snOBB(snVec4Set(10, 500, 500, 0) * 0.5f);
			rb->addCollider(collider);
			rb->addCollisionFlag(snCollisionFlag::CF_NO_CONTACT_RESPONSE);
			rb->addCollisionFlag(snCollisionFlag::CF_CONTACT_CALLBACK);
			rb->setOnCollisionCallback([](snRigidbody* const _me, snRigidbody* const _other)
			{
				UNREFERENCED_PARAMETER(_me);
				IWorldEntity* entity = WORLD->getEntityFromActor(_other);
				if (entity != 0)
					entity->setIsActive(false);

				_other->setIsActive(false);
			});

			//initialize the rigidbody
			rb->initializeStatic(snVec4Set(-250, 250, 0, 1), snVec4Set(0, 0, 0, 1));

			snhRigidbody stat = SUPERNOVA->registerObject(rb);
			scene->attachActor(stat);

			//create the world entity
			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(10, 500, 500));
			kinematicBox->setActor(stat.getPtr());
			kinematicBox->setWireframe(true);
		}

		//create front wall 
		{
			snRigidbody* rb = new snRigidbody();
			
			//create collider
			snOBB* collider = new snOBB(snVec4Set(500, 500, 10, 0) * 0.5f);
			rb->addCollider(collider);
			rb->addCollisionFlag(snCollisionFlag::CF_NO_CONTACT_RESPONSE);
			rb->addCollisionFlag(snCollisionFlag::CF_CONTACT_CALLBACK);
			rb->setOnCollisionCallback([](snRigidbody* const _me, snRigidbody* const _other)
			{
				UNREFERENCED_PARAMETER(_me);
				IWorldEntity* entity = WORLD->getEntityFromActor(_other);
				if (entity != 0)
					entity->setIsActive(false);

				_other->setIsActive(false);
			});

			//initialize the rigidbody
			rb->initializeStatic(snVec4Set(0, 250, 250, 1), snVec4Set(0, 0, 0, 1));
			snhRigidbody stat = SUPERNOVA->registerObject(rb);
			scene->attachActor(stat);

			//create the world entity
			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(500, 500, 10));
			kinematicBox->setActor(stat.getPtr());
			kinematicBox->setWireframe(true);
		}

		//create back wall 
		{
			snRigidbody* rb = new snRigidbody();
			
			//create collider
			snOBB* collider = new snOBB(snVec4Set(500, 500, 10, 0) * 0.5f);
			rb->addCollider(collider);
			rb->addCollisionFlag(snCollisionFlag::CF_NO_CONTACT_RESPONSE);
			rb->addCollisionFlag(snCollisionFlag::CF_CONTACT_CALLBACK);
			rb->setOnCollisionCallback([](snRigidbody* const _me, snRigidbody* const _other)
			{
				UNREFERENCED_PARAMETER(_me);
				IWorldEntity* entity = WORLD->getEntityFromActor(_other);
				if (entity != 0)
					entity->setIsActive(false);

				_other->setIsActive(false);
			});

			//initialize the rigidbody
			rb->initializeStatic(snVec4Set(0, 250, -250, 1), snVec4Set(0, 0, 0, 1));
			snhRigidbody stat = SUPERNOVA->registerObject(rb);
			scene->attachActor(stat);

			//create the world entity
			EntityBox* kinematicBox = WORLD->createBox(XMFLOAT3(500, 500, 10));
			kinematicBox->setActor(stat.getPtr());
			kinematicBox->setWireframe(true);
		}

		return scene;
	}

	EntityComposite* SceneManager::createWheel(const snVec& _position, const snVec& _orientation, const XMFLOAT4& _color, float _length)
	{
		const float DEPTH = 5;
		const float THICKNESS = 2.8f;

		snhScene scene = WORLD->getPhysicsScene();
		snhRigidbody act = SUPERNOVA->registerObject(new snRigidbody);
		scene->attachActor(act);

		act->setName("composite");
		act->setPosition(_position);
		act->setOrientation(_orientation);
		act->getPhysicMaterial().m_restitution = 0;
		act->setAngularDampingCoeff(0.5f);

		const int PIN_COUNT = 6;
		float angle = SN_PI / PIN_COUNT;
		for (int pinId = 0; pinId < PIN_COUNT; ++pinId)
		{
			snOBB* xBox = new snOBB(snVec4Set(_length, THICKNESS, DEPTH, 0) * 0.5f);
			xBox->getTransform().setLocalEulerAngles(0, 0, angle * pinId);
			act->addCollider(xBox);

			//fill in the space between the pins
			snOBB* fill = new snOBB(snVec4Set(_length * 0.7f, 4, DEPTH, 0) * 0.5f);
			fill->getTransform().setLocalEulerAngles(0, 0, angle * (pinId + 0.5f));
			act->addCollider(fill);
		}

		act->updateMassAndInertia(10);
		act->initializeDynamic();

		return WORLD->createComposite(act.getPtr(), _color);
	}
}