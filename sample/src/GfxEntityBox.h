#ifndef GFX_ENTITY_BOX_H
#define GFX_ENTITY_BOX_H

#include "IGfxEntity.h"
#include <memory>
#include <Effects.h>

using DirectX::XMFLOAT3;
using DirectX::XMFLOAT4;
using DirectX::XMMATRIX;

#include <GeometricPrimitive.h>
using DirectX::GeometricPrimitive;

namespace Devil
{
	class GfxEntityBox : public IGfxEntity
	{
	private:

		std::unique_ptr<GeometricPrimitive> m_primitive;

	public:
		GfxEntityBox();
		virtual ~GfxEntityBox();

		bool initialize(ID3D11DeviceContext* _context);
		void shutdown();
		void render(const XMMATRIX& _world, const XMMATRIX& _view, const XMMATRIX& _projection, const XMVECTOR& _color, 
			const Texture* const _texture, bool _wireframe);

	};

}
#endif