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
#include "World.h"

#include "EntitySphere.h"
#include "EntityBox.h"
#include "EntityCollisionPoint.h"
#include "EntityCamera.h"
#include "EntityFixedConstraint.h"
#include "EntityPointToPointConstraint.h"
#include "EntityBoxLauncher.h"
#include "EntityComposite.h"
#include "EntitySkybox.h"
#include "EntityStaticMesh.h"
#include "EntityTerrain.h"
using namespace Devil::Worlds::Entities;

#include "WorldHUD.h"
#include "Input.h"
#include "Graphics.h"
#include "IComponent.h"

#include "snIActor.h"
using Supernova::snIActor;

namespace Devil
{
	World* World::m_Instance = 0;

	World::World() :m_camera(0), m_monkeyBall(0), m_hud(0), m_collisionPoint(0), m_EntityList()
	{
	}


	World::~World()
	{
		for (std::vector<IWorldEntity*>::iterator i = m_EntityList.begin(); i != m_EntityList.end(); ++i)
		{
			if ((*i) != 0)
			{
				delete (*i);
				(*i) = 0;
			}
		}

		m_EntityList.clear();
	}

	World* World::getInstance()
	{
		if (m_Instance == 0)
			m_Instance = new World();

		return m_Instance;
	}

	void World::shutdown()
	{
		if (m_Instance == 0)
			return;

		delete m_Instance;
		m_Instance = 0;
	}

	bool World::initialize()
	{
		m_collisionPoint = createCollisionPoint(1.f);
		return true;
	}

	EntitySphere* World::createSphere(float _diameter, const XMVECTOR& _color)
	{
		EntitySphere* NewEntity = new EntitySphere();
		NewEntity->initialize(_diameter, _color);
		m_EntityList.push_back(NewEntity);
		return NewEntity;
	}

	EntityBox* World::createBox(const XMFLOAT3& _size)
	{
		EntityBox* NewEntity = new EntityBox();
		NewEntity->initialize(_size, XMFLOAT4(1, 1, 1, 1));
		m_EntityList.push_back(NewEntity);
		return NewEntity;
	}

	EntityBox* World::createBox(const XMFLOAT3& _size, const XMFLOAT4& _color)
	{
		EntityBox* NewEntity = new EntityBox();
		NewEntity->initialize(_size, _color);
		m_EntityList.push_back(NewEntity);
		return NewEntity;
	}

	EntityComposite* World::createComposite(snIActor* _actor, const XMFLOAT4& _color)
	{
		EntityComposite* newEntity = new EntityComposite(_actor, _color);
		m_EntityList.push_back(newEntity);
		return newEntity;
	}

	EntityCollisionPoint* World::createCollisionPoint(float _diameter)
	{
		EntityCollisionPoint* newEntity = new EntityCollisionPoint();
		newEntity->initialize(_diameter);
		m_EntityList.push_back(newEntity);
		return newEntity;
	}

	EntityCamera* World::createCamera(const XMVECTOR& _position, const XMVECTOR& _lookAt, const XMVECTOR& _up)
	{
		m_camera = new EntityCamera();
		m_camera->initialize(_position, _lookAt, _up);
		m_EntityList.insert(m_EntityList.cbegin(), m_camera);
		return m_camera;
	}

	EntityFixedConstraint* World::createFixedConstraint(const snFixedConstraint* _constraint)
	{
		EntityFixedConstraint* newEntity = new EntityFixedConstraint();
		newEntity->initialize(_constraint);
		m_EntityList.push_back(newEntity);
		return newEntity;
	}

	EntityPointToPointConstraint* World::createPointToPointConstraint(const snPointToPointConstraint* _constraint)
	{
		EntityPointToPointConstraint* newEntity = new EntityPointToPointConstraint();
		newEntity->initialize(_constraint);
		m_EntityList.push_back(newEntity);
		return newEntity;
	}

	WorldHUD* World::createHUD()
	{
		WorldHUD* newHUD = new WorldHUD();
		m_EntityList.push_back(newHUD);
		m_hud = newHUD;
		return newHUD;
	}

	EntityBoxLauncher* World::createEntityBoxLauncher(unsigned int _count)
	{
		EntityBoxLauncher* launcher = new EntityBoxLauncher();
		launcher->initialize(_count);
		m_EntityList.push_back(launcher);
		return launcher;
	}

	EntityComposite* World::createMonkeyBall(snIActor* _actor, const XMFLOAT4& _color)
	{
		m_monkeyBall = new EntityComposite(_actor, _color);
		m_EntityList.push_back(m_monkeyBall);
		return m_monkeyBall;
	}

	EntitySkybox* World::createSkybox(IWorldEntity* _target, float _size, const XMFLOAT4& _color)
	{
		EntitySkybox* skybox = new EntitySkybox(_target, _size, _color);
		m_EntityList.push_back(skybox);
		return skybox;
	}

	EntityStaticMesh* World::createStaticMesh(IGfxEntity* _gfx)
	{
		EntityStaticMesh* staticMesh = new EntityStaticMesh(_gfx);
		m_EntityList.push_back(staticMesh);
		return staticMesh;
	}

	EntityTerrain* World::createTerrain(const string& _filename, unsigned int _tilesPerRow, unsigned int _tilesPerColumn, float _quadSize,
		float _minScale, float _maxScale, IWorldEntity* _target)
	{
		EntityTerrain* terrain = new EntityTerrain();
		terrain->initialize(_filename, _tilesPerRow, _tilesPerColumn, _quadSize, _minScale, _maxScale, _target);
		m_EntityList.push_back(terrain);
		return terrain;
	}

	void World::clearWorld()
	{
		for (vector<IWorldEntity*>::iterator i = m_EntityList.begin(); i != m_EntityList.end(); ++i)
			delete *i;

		m_EntityList.clear();

		m_collisionPoint = 0;
		m_hud = 0;
	}

	void World::update(float _dt)
	{
		m_dt = _dt;

		if (INPUT->getMessage(Input::InputMessage::TOGGLE_COLLISION_POINTS))
		{
			toggleCollisionPointActivation();
		}
		if (INPUT->getMessage(Input::InputMessage::TOGGLE_RENDER_MODE))
		{
			toggleRenderMode();
		}

		//update world
		for (std::vector<IWorldEntity*>::iterator i = m_EntityList.begin(); i != m_EntityList.end(); ++i)
		{
			if ((*i)->getIsActive())
			{
				(*i)->preUpdateComponents(_dt);
				(*i)->update();
				(*i)->postUpdateComponents(_dt);
			}
			
		}
	}

	void World::render()
	{
		GRAPHICS->BeginRender();
		

		for (std::vector<IWorldEntity*>::iterator i = m_EntityList.begin(); i != m_EntityList.end(); ++i)
		{
			if ((*i)->getIsActive())
			{
				(*i)->preRenderComponents();
				(*i)->render();
				(*i)->postRenderComponents();
			}
		}

		GRAPHICS->spriteBeginRender();
		for (std::vector<IWorldEntity*>::iterator i = m_EntityList.begin(); i != m_EntityList.end(); ++i)
		{
			if ((*i)->getIsActive())
				(*i)->spriteRender();
		}
		GRAPHICS->spriteEndRender();

		GRAPHICS->EndRender();	
	}

	EntityCamera* World::getCamera() const
	{
		return m_camera;
	}

	EntityComposite* World::getMonkeyBall() const
	{
		return m_monkeyBall;
	}

	IWorldEntity* World::getEntityFromActor(snIActor* const _actor) const
	{
		for (std::vector<IWorldEntity*>::const_iterator i = m_EntityList.cbegin(); i != m_EntityList.cend(); ++i)
		{
			if ((*i)->getActor() == _actor)
				return (*i);
		}

		return 0;
	}

	void World::toggleCollisionPointActivation()
	{
		m_collisionPoint->setIsActive(!m_collisionPoint->getIsActive());
	}

	void World::activateCollisionPoint()
	{
		m_collisionPoint->setIsActive(true);
	}

	void World::deactivateCollisionPoint()
	{
		m_collisionPoint->setIsActive(false);
	}

	void World::toggleRenderMode()
	{
		m_wireframe = !m_wireframe;
		for (vector<IWorldEntity*>::iterator i = m_EntityList.begin(); i != m_EntityList.end(); ++i)
		{
			(*i)->setWireframe(m_wireframe);
		}
	}

	void World::setPhysicsFPS(int _physicsFPS) const
	{
		if (m_hud != 0)
			m_hud->setPhysicsFPS(_physicsFPS);
	}

	void World::setGraphicsFPS(int _graphicsFPS) const
	{
		if (m_hud != 0)
			m_hud->setGraphicsFPS(_graphicsFPS);
	}

	void World::setPhysicsScene(snhScene _hScene)
	{
		m_physicsScene = _hScene;
	}

	float World::getDeltaTime() const
	{
		return m_dt;
	}

	snhScene World::getPhysicsScene() const
	{
		return m_physicsScene;
	}

}