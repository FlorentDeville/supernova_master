#include "EntityCamera.h"
#include "Graphics.h"
#include "Camera.h"

#include "World.h"
#include "Input.h"

#include "snVector4f.h"
namespace Devil
{

	EntityCamera::EntityCamera()
	{
	}


	EntityCamera::~EntityCamera()
	{
	}

	bool EntityCamera::initialize(const XMVECTOR& _position, const XMVECTOR& _lookAt, const XMVECTOR& _up)
	{
		m_gfxCamera = GRAPHICS->getCamera();
		m_position = _position;
		m_lookAt = _lookAt;
		m_up = _up;
		return true;
	}

	void EntityCamera::update()
	{
		//return;
		const float linearCameraSpeed = 0.5f;
		const float angularCameraSpeed = 1.5f;

		//forward vector
		XMVECTOR forward = m_lookAt - m_position;
		forward = XMVector3Normalize(forward);

		//left vector
		XMVECTOR left = XMVector3Cross(forward, m_up);

		//up vector
		XMVECTOR up = XMVector3Cross(left, forward);

		//move forward
		if (WORLD->getInput()->isKeyDown('Z'))
		{
			forward = forward * linearCameraSpeed;

			m_position = m_position + forward;
			m_lookAt = m_lookAt + forward;
		}
		else if (WORLD->getInput()->isKeyDown('S')) //move backward
		{
			forward = forward * linearCameraSpeed;

			m_position = m_position - forward;
			m_lookAt = m_lookAt - forward;
		}

		if (WORLD->getInput()->isKeyDown('Q')) //move to the left
		{
			left = left * linearCameraSpeed;
			m_position = m_position + left;
			m_lookAt = m_lookAt + left;
		}
		else if (WORLD->getInput()->isKeyDown('D')) //move to the right
		{
			left = left * linearCameraSpeed;
			m_position = m_position - left;
			m_lookAt = m_lookAt - left;
		}

		//mouse movement since last update
		XMFLOAT2 mouseOffset = WORLD->getInput()->getMouseDelta();

		if (mouseOffset.x < 0)
			m_lookAt = m_lookAt + left * angularCameraSpeed;
		else if (mouseOffset.x > 0)
			m_lookAt = m_lookAt - left * angularCameraSpeed;
		if (mouseOffset.y < 0)
			m_lookAt = m_lookAt + up * angularCameraSpeed;
		else if (mouseOffset.y > 0)
			m_lookAt = m_lookAt - up * angularCameraSpeed;
	}

	void EntityCamera::render()
	{
		m_gfxCamera->Render(snVector4f(m_position), snVector4f(m_lookAt), snVector4f(m_up));
	}

	void EntityCamera::setLookAt(const XMVECTOR& _lookAt)
	{
		m_lookAt = _lookAt;
	}

	void EntityCamera::setUp(const XMVECTOR& _up)
	{
		m_up = _up;
	}

	const XMVECTOR& EntityCamera::getLookAt() const
	{
		return m_lookAt;
	}
}