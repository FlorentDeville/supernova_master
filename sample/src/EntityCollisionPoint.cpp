#include "EntityCollisionPoint.h"

#include "Graphics.h"
#include "D3D.h"
#include "Camera.h"
#include "GfxEntitySphere.h"

#include "snScene.h"
#include "snFactory.h"

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
		m_gfx = GRAPHICS->createSphere(_diameter, Colors::Red);
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

		const snContactPointVector& contacts = SUPERNOVA->getScene(0)->getContactsPoints();
		for (snContactPointVectorConstIterator i = contacts.cbegin(); i != contacts.cend(); ++i)
		{
			XMMATRIX transform = XMMatrixTranslationFromVector(i->m_point.m_vec);
			m_gfx->render(transform, viewMatrix, projectionMatrix);
		}
	}

	void EntityCollisionPoint::addLocation(const snVector4f& _location)
	{
		m_locations.push_back(_location);
	}

	void EntityCollisionPoint::clearLocations()
	{
		m_locations.clear();
	}
}