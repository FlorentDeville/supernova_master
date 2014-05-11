#include "EntityPlan.h"
#include "GfxEntityPlan.h"
#include "Graphics.h"
#include "Camera.h"
#include "D3D.h"

#include "snVec.inl"
#include "snIActor.h"

using namespace DirectX;

namespace Devil
{
	EntityPlan::EntityPlan()
	{
	}


	EntityPlan::~EntityPlan()
	{
	}

	bool EntityPlan::initialize(const XMFLOAT2& _size, const XMFLOAT4& _color)
	{
		m_gfx = GRAPHICS->createPlan(_size, _color);
		if (m_gfx)
			return true;

		return false;
	}

	void EntityPlan::update()
	{
		m_position = m_actor->getPosition();
	}

	void EntityPlan::render()
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