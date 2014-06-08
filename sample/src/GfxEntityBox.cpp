#include "GfxEntityBox.h"
#include "Graphics.h"
#include "D3D.h"
#include "EffectProvider.h"
#include "Texture.h"

#include <VertexTypes.h>
using namespace DirectX;

namespace Devil
{
	GfxEntityBox::GfxEntityBox()
	{
	}

	GfxEntityBox::~GfxEntityBox()
	{
	}

	bool GfxEntityBox::initialize(ID3D11DeviceContext* _context)
	{
		m_primitive = GeometricPrimitive::CreateCube(_context, 1.f, false);
		return true;
	}

	void GfxEntityBox::shutdown()
	{}

	void GfxEntityBox::render(const XMMATRIX& _world, const XMMATRIX& _view, const XMMATRIX& _projection, const XMVECTOR& _color, 
		const Texture* const _texture, bool _wireframe)
	{
		ID3D11ShaderResourceView* resource = 0;
		if (_texture != 0)
			resource = _texture->getTexture();

		m_primitive->Draw(_world, _view, _projection, _color, resource, _wireframe);
		return;
	}
}