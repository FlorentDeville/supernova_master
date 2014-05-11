#include "EntityBox.h"

#include "Graphics.h"
#include "D3D.h"
#include "Camera.h"
#include "GfxEntityBox.h"

#include "snVec.inl"
#include "snIActor.h"

using namespace Supernova::Vector;
using namespace DirectX;

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
		/*snVector4f newPosition = m_actor->getPosition();
		m_position = XMVectorSet(newPosition.VEC4FX, newPosition.VEC4FY, newPosition.VEC4FZ, 1);*/
	}

	void EntityBox::render()
	{
		snVec newPosition = m_actor->getPosition();
		m_position = XMVectorSet(snVec4GetX(newPosition), snVec4GetY(newPosition), snVec4GetZ(newPosition), 1);

		XMMATRIX translation = XMMatrixTranslationFromVector(m_position);
		XMMATRIX orientation;
		orientation.r[0] = m_actor->getOrientationMatrix().m_r[0];
		orientation.r[1] = m_actor->getOrientationMatrix().m_r[1];
		orientation.r[2] = m_actor->getOrientationMatrix().m_r[2];
		orientation.r[3] = m_actor->getOrientationMatrix().m_r[3];

		XMMATRIX scaling = XMMatrixScaling(m_scale.x, m_scale.y, m_scale.z);

		XMMATRIX viewMatrix, projectionMatrix, transform;
		transform = scaling * orientation * translation;

		GRAPHICS->getCamera()->GetViewMatrix(viewMatrix);
		GRAPHICS->getDirectXWrapper()->getProjectionMatrix(projectionMatrix);

		if(m_wireframe)
			GRAPHICS->getDirectXWrapper()->turnOnWireframeMode();

		m_gfx->render(transform, viewMatrix, projectionMatrix);
		
		if(m_wireframe)
			GRAPHICS->getDirectXWrapper()->turnOnFillMode();
	}
}