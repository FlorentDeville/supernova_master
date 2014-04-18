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
#include "EntityPlan.h"
#include "EntityCollisionPoint.h"
#include "EntityCamera.h"
#include "EntityFixedConstraint.h"
#include "EntityPointToPointConstraint.h"
#include "WorldHUD.h"
#include "Input.h"
#include "Graphics.h"
#include "IComponent.h"

#include "snActor.h"
using Supernova::snActor;

namespace Devil
{
	World* World::m_Instance = 0;

	World::World()
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

		for (std::vector<IComponent*>::iterator i = m_componentsList.begin(); i != m_componentsList.end(); ++i)
		{
			if ((*i) != 0)
			{
				delete (*i);
			}
		}
		m_componentsList.clear();
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

	EntitySphere* World::createSphere(float _diameter)
	{
		EntitySphere* NewEntity = new EntitySphere();
		NewEntity->initialize(_diameter);
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

	EntityPlan* World::createPlan(const XMFLOAT2& _size, const XMFLOAT4& _color)
	{
		EntityPlan* newEntity = new EntityPlan();
		newEntity->initialize(_size, _color);
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

	ComponentFloatingText<snActor, float>* World::createComponentFloatingText()
	{
		ComponentFloatingText<snActor, float>* newComponent = new ComponentFloatingText<snActor, float>();
		m_componentsList.push_back(newComponent);
		return newComponent;
	}

	void World::clearWorld()
	{
		for (vector<IWorldEntity*>::iterator i = m_EntityList.begin(); i != m_EntityList.end(); ++i)
			delete *i;

		for (vector<IComponent*>::iterator i = m_componentsList.begin(); i != m_componentsList.end(); ++i)
			delete *i;

		m_EntityList.clear();
		m_componentsList.clear();

		m_collisionPoint = 0;
		m_hud = 0;
	}

	void World::update()
	{
		//update world
		for (std::vector<IWorldEntity*>::iterator i = m_EntityList.begin(); i != m_EntityList.end(); ++i)
		{
			if ((*i)->getIsActive())
			{
				(*i)->update();
				(*i)->updateComponents();
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
				(*i)->render();
				(*i)->renderComponents();
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
}