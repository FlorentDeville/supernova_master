#include "GfxEntitySphere.h"
#include "Texture.h"

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

	void GfxEntitySphere::render(const XMMATRIX& _world, const XMMATRIX& _view, const XMMATRIX& _projection, const XMVECTOR& _color, 
		const Texture* const _texture, bool _wireframe)
	{
		ID3D11ShaderResourceView* resource = 0;
		if (_texture != 0)
			resource = _texture->getTexture();

		m_primitive->Draw(_world, _view, _projection, _color, resource, _wireframe);
	}
}