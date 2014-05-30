#include "EntityCamera.h"
#include "Graphics.h"
#include "Camera.h"

#include "World.h"
#include "Input.h"

#include "snVec.inl"

using namespace DirectX;

#include "CameraState_FreeCamera.h"
#include "CameraState.h"

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

		//create the fsm
		CameraState_FreeCamera* stateFreeCamera = new CameraState_FreeCamera(this);
		m_fsmRunner.addState(CAMERA_STATE_FREE_CAMERA, stateFreeCamera);
		m_fsmRunner.setImmediateState(CAMERA_STATE_FREE_CAMERA);
		return true;
	}

	void EntityCamera::update()
	{
		m_fsmRunner.update();
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

	const XMVECTOR& EntityCamera::getUp() const
	{
		return m_up;
	}
}