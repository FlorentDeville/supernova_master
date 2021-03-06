#include "EntityCollisionPoint.h"
#include "World.h"

#include "Graphics.h"
#include "D3D.h"
#include "Camera.h"
#include "GfxEntitySphere.h"

#include "snScene.h"
#include "snWorld.h"

using namespace DirectX;

namespace Devil
{
	EntityCollisionPoint::EntityCollisionPoint() : IWorldEntity()
	{
	}


	EntityCollisionPoint::~EntityCollisionPoint()
	{
	}

	bool EntityCollisionPoint::initialize(float _diameter)
	{
		m_gfx = GRAPHICS->getSphere();
		m_diameter = _diameter;
		return true;
	}

	void EntityCollisionPoint::update()
	{
	}

	void EntityCollisionPoint::render()
	{
		XMMATRIX viewMatrix, projectionMatrix, transform;
		GRAPHICS->getCamera()->GetViewMatrix(viewMatrix);
		GRAPHICS->getDirectXWrapper()->getProjectionMatrix(projectionMatrix);

		const snVecVector& contacts = WORLD->getPhysicsScene()->getCollisionPoints();
		for (snVecVectorConstIterator i = contacts.cbegin(); i != contacts.cend(); ++i)
		{
			XMMATRIX transform = XMMatrixScaling(m_diameter, m_diameter, m_diameter) *  XMMatrixTranslationFromVector(*i);
			m_gfx->render(transform, viewMatrix, projectionMatrix, Colors::Red, m_texture, m_wireframe);
		}
	}

	void EntityCollisionPoint::addLocation(const snVec& _location)
	{
		m_locations.push_back(_location);
	}

	void EntityCollisionPoint::clearLocations()
	{
		m_locations.clear();
	}
}