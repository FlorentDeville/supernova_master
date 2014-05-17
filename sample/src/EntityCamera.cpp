#include "EntityCamera.h"
#include "Graphics.h"
#include "Camera.h"

#include "World.h"
#include "Input.h"

#include "snVec.inl"

using namespace DirectX;

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
		const float linearCameraSpeed = 0.75f;
		const float angularCameraSpeed = 0.025f;

		//forward vector
		XMVECTOR forward = m_lookAt - m_position;
		forward = XMVector3Normalize(forward);

		//left vector
		XMVECTOR left = XMVector3Cross(forward, m_up);

		//up vector
		XMVECTOR up = XMVector3Cross(left, forward);

		
		const int WHEEL_SPEED = 4;
		if (INPUT->getMouseWheel() > 0) //move forward
		{
			forward = forward * linearCameraSpeed * WHEEL_SPEED;

			m_position = m_position + forward;
			m_lookAt = m_lookAt + forward;
			INPUT->setMouseWheel(0);
		} 
		else if(INPUT->getMouseWheel() < 0) //move backward
		{
			forward = forward * linearCameraSpeed * WHEEL_SPEED;

			m_position = m_position - forward;
			m_lookAt = m_lookAt - forward;
			INPUT->setMouseWheel(0);
		}

		if (INPUT->isKeyDown('Z'))//move up
		{
			XMVECTOR offset = up * linearCameraSpeed;
			m_position = m_position + offset;
			m_lookAt = m_lookAt + offset;
		}
		else if (INPUT->isKeyDown('S')) //move down
		{
			XMVECTOR offset = up * linearCameraSpeed;
			m_position = m_position - offset;
			m_lookAt = m_lookAt - offset;
		}

		if (INPUT->isKeyDown('Q')) //move to the left
		{
			left = left * linearCameraSpeed;
			m_position = m_position + left;
			m_lookAt = m_lookAt + left;
		}
		else if (INPUT->isKeyDown('D')) //move to the right
		{
			left = left * linearCameraSpeed;
			m_position = m_position - left;
			m_lookAt = m_lookAt - left;
		}

		//mouse movement since last update
		XMFLOAT2 mouseOffset = INPUT->getMouseDelta();

		
		if (mouseOffset.x < 0)
		{
			XMMATRIX rotation = XMMatrixRotationAxis(up, -angularCameraSpeed);
			forward = XMVector3Transform(forward, rotation);
			m_lookAt = m_position + forward;
		}		
		else if (mouseOffset.x > 0)
		{
			XMMATRIX rotation = XMMatrixRotationAxis(up, angularCameraSpeed);
			forward = XMVector3Transform(forward, rotation);
			m_lookAt = m_position + forward;
		}
		//left vector
		left = XMVector3Cross(forward, m_up);

		if (mouseOffset.y < 0)
		{
			XMMATRIX rotation = XMMatrixRotationAxis(left, angularCameraSpeed);
			forward = XMVector3Transform(forward, rotation);
			m_lookAt = m_position + forward;
		}
		else if (mouseOffset.y > 0)
		{
			XMMATRIX rotation = XMMatrixRotationAxis(left, -angularCameraSpeed);
			forward = XMVector3Transform(forward, rotation);
			m_lookAt = m_position + forward;
		}
		XMVectorSetW(m_lookAt, 1);
	}

	void EntityCamera::render()
	{
		m_gfxCamera->Render(m_position, m_lookAt, m_up);
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