#include "EntitySphere.h"


#include "Graphics.h"
#include "D3D.h"
#include "Camera.h"
#include "GfxEntitySphere.h"

#include "snSphere.h"

#include "snScene.h"

#include "snRigidbody.h"

using namespace DirectX;

namespace Devil
{
	EntitySphere::EntitySphere() : IWorldEntity()
	{
	}


	EntitySphere::~EntitySphere()
	{
	}

	bool EntitySphere::initialize(float _diameter, const XMVECTOR& _color)
	{
		m_diameter = _diameter;
		m_color = _color;
		m_gfx = GRAPHICS->getSphere();
		return true;
	}

	void EntitySphere::update()
	{
		m_position = m_actor->getPosition();
	}

	void EntitySphere::render()
	{
		XMMATRIX translation = XMMatrixTranslationFromVector(m_position);
		XMMATRIX orientation = XMMatrixRotationRollPitchYaw(m_orientation.x, m_orientation.y, m_orientation.z);
		XMMATRIX scaling = XMMatrixScaling(m_diameter, m_diameter, m_diameter);

		XMMATRIX viewMatrix, projectionMatrix, transform;
		transform = scaling * orientation * translation;

		GRAPHICS->getCamera()->GetViewMatrix(viewMatrix);
		GRAPHICS->getDirectXWrapper()->getProjectionMatrix(projectionMatrix);
	
		m_gfx->render(transform, viewMatrix, projectionMatrix, m_color, m_texture, m_wireframe);
	}
}