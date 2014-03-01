#include "GfxEntitySphere.h"

namespace Devil
{
	GfxEntitySphere::GfxEntitySphere()
	{
	}


	GfxEntitySphere::~GfxEntitySphere()
	{
	}

	bool GfxEntitySphere::initialize(ID3D11DeviceContext* _device, WCHAR* /*_texturePath*/, float _diameter, const XMVECTOR& _color)
	{
		m_primitive = GeometricPrimitive::CreateGeoSphere(_device, _diameter, 3U, false);
		m_color = _color;
		return true;
	}

	void GfxEntitySphere::shutdown()
	{
		m_primitive.release();
	}

	void GfxEntitySphere::render(const XMMATRIX& _world, const XMMATRIX& _view, const XMMATRIX& _projection)
	{
		m_primitive->Draw(_world, _view, _projection, m_color, nullptr, false);
	}
}