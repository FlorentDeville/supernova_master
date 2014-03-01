#include "EntitySphere.h"


#include "Graphics.h"
#include "D3D.h"
#include "Camera.h"
#include "GfxEntitySphere.h"

#include "snColliderSphere.h"

#include "snScene.h"

#include "snActor.h"
using namespace Supernova;

namespace Devil
{
	EntitySphere::EntitySphere() : IWorldEntity()
	{
	}


	EntitySphere::~EntitySphere()
	{
	}

	bool EntitySphere::initialize(float _diameter)
	{
		m_gfx = GRAPHICS->createSphere(_diameter);
		return true;
	}

	void EntitySphere::update()
	{
		snVector4f newPosition = m_actor->getPosition();
		m_position = XMVectorSet(newPosition.VEC4FX, newPosition.VEC4FY, newPosition.VEC4FZ, 1);
	}

	void EntitySphere::render()
	{
		XMMATRIX translation = XMMatrixTranslationFromVector(m_position);
		XMMATRIX orientation = XMMatrixRotationRollPitchYaw(m_orientation.x, m_orientation.y, m_orientation.z);
		XMMATRIX scaling = XMMatrixScaling(m_scale.x, m_scale.y, m_scale.z);

		XMMATRIX viewMatrix, projectionMatrix, transform;
		transform = scaling * orientation * translation;

		GRAPHICS->getCamera()->GetViewMatrix(viewMatrix);
		GRAPHICS->getDirectXWrapper()->getProjectionMatrix(projectionMatrix);
	
		m_gfx->render(transform, viewMatrix, projectionMatrix);
	}
}