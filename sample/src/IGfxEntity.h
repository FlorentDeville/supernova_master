#ifndef I_GFX_ENTITY_H
#define I_GFX_ENTITY_H

#include <d3d11.h>

#include <DirectXMath.h>

using namespace DirectX;

namespace Devil
{
	class IGfxEntity
	{
	public:
		IGfxEntity(){};
		virtual ~IGfxEntity(){};

		virtual void shutdown() = 0;
		virtual void render(const XMMATRIX& _world, const XMMATRIX& _view, const XMMATRIX& _projection) = 0;
	};

}
#endif