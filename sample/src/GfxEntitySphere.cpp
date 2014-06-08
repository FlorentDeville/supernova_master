#include "GfxEntitySphere.h"

namespace Devil
{
	GfxEntitySphere::GfxEntitySphere()
	{
	}


	GfxEntitySphere::~GfxEntitySphere()
	{
	}

	bool GfxEntitySphere::initialize(ID3D11DeviceContext* _device)
	{
		m_primitive = GeometricPrimitive::CreateGeoSphere(_device, 1, 3U, false);
		return true;
	}

	void GfxEntitySphere::shutdown()
	{
	}

	void GfxEntitySphere::render(const XMMATRIX& _world, const XMMATRIX& _view, const XMMATRIX& _projection, const XMVECTOR& _color, bool _wireframe)
	{
		m_primitive->Draw(_world, _view, _projection, _color, nullptr, _wireframe);
	}
}