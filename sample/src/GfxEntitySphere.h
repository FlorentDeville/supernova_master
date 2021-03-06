#ifndef GFX_ENTITY_SPHERE_H
#define GFX_ENTITY_SPHERE_H

#include "IGfxEntity.h"
#include <memory>
#include <GeometricPrimitive.h>

using DirectX::XMVECTOR;
using DirectX::XMMATRIX;
using DirectX::GeometricPrimitive;

namespace Devil
{
	class GfxEntitySphere : public IGfxEntity
	{
	private:

		std::unique_ptr<GeometricPrimitive> m_primitive;
		XMVECTOR m_color;
	public:
		GfxEntitySphere();
		virtual ~GfxEntitySphere();

		bool initialize(ID3D11DeviceContext* _device);
		void shutdown();
		void render(const XMMATRIX& _world, const XMMATRIX& _view, const XMMATRIX& _projection, const XMVECTOR& _color, 
			const Texture* const _texture, bool _wireframe);

		void* operator new(size_t _count)
		{
			return _aligned_malloc(_count, 16);
		}

		void operator delete(void* _p)
		{
			_aligned_free(_p);
		}
	};

}
#endif