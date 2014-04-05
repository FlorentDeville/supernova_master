#include "EntityBox.h"

#include "Graphics.h"
#include "D3D.h"
#include "Camera.h"
#include "GfxEntityBox.h"

#include "snVector4f.h"
#include "snActor.h"

namespace Devil
{
	EntityBox::EntityBox() : m_gfx(0)
	{
	}


	EntityBox::~EntityBox()
	{
	}

	bool EntityBox::initialize(const XMFLOAT3& _size, const XMFLOAT4& _color)
	{
		m_size = _size;
		m_gfx = GRAPHICS->createBox(m_size, _color);
		return true;
	}

	void EntityBox::update()
	{
		snVector4f newPosition = m_actor->getPosition();
		m_position = XMVectorSet(newPosition.VEC4FX, newPosition.VEC4FY, newPosition.VEC4FZ, 1);
	}

	void EntityBox::render()
	{
		XMMATRIX translation = XMMatrixTranslationFromVector(m_position);
		XMMATRIX orientation;
		orientation.r[0] = m_actor->getOrientationMatrix().m_r[0].m_vec;
		orientation.r[1] = m_actor->getOrientationMatrix().m_r[1].m_vec;
		orientation.r[2] = m_actor->getOrientationMatrix().m_r[2].m_vec;
		orientation.r[3] = m_actor->getOrientationMatrix().m_r[3].m_vec;

		XMMATRIX scaling = XMMatrixScaling(m_scale.x, m_scale.y, m_scale.z);

		XMMATRIX viewMatrix, projectionMatrix, transform;
		transform = scaling * orientation * translation;

		GRAPHICS->getCamera()->GetViewMatrix(viewMatrix);
		GRAPHICS->getDirectXWrapper()->getProjectionMatrix(projectionMatrix);

		m_gfx->render(transform, viewMatrix, projectionMatrix);
	}
}