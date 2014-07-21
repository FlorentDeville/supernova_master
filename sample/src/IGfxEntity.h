#ifndef I_GFX_ENTITY_H
#define I_GFX_ENTITY_H

//Disable warning : structure was padded due to __declspec(align())
#pragma warning( disable : 4324 )

#include <DirectXMath.h>

using DirectX::XMMATRIX;
using DirectX::XMVECTOR;

namespace Devil
{
	class Texture;

	class IGfxEntity
	{
	public:
		IGfxEntity(){};
		virtual ~IGfxEntity(){};

		virtual void shutdown() = 0;
		virtual void render(const XMMATRIX& _world, const XMMATRIX& _view, const XMMATRIX& _projection, const XMVECTOR& _color, 
			const Texture* const _texture, bool _wireframe) = 0;
	};

}
#endif