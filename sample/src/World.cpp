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

	void World::clearWorld()
	{
		for (vector<IWorldEntity*>::iterator i = m_EntityList.begin(); i != m_EntityList.end(); ++i)
			delete *i;

		m_EntityList.clear();
		m_collisionPoint = 0;
		m_hud = 0;
	}

	void World::update()
	{
		//update world
		for (std::vector<IWorldEntity*>::iterator i = m_EntityList.begin(); i != m_EntityList.end(); ++i)
		{
			if ((*i)->getIsActive())
				(*i)->update();
		}
	}

	void World::render()
	{
		GRAPHICS->BeginRender();
		

		for (std::vector<IWorldEntity*>::iterator i = m_EntityList.begin(); i != m_EntityList.end(); ++i)
		{
			if ((*i)->getIsActive())
				(*i)->render();
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