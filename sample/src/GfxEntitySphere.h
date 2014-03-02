#ifndef GFX_ENTITY_SPHERE_H
#define GFX_ENTITY_SPHERE_H

#include "IGfxEntity.h"
#include <memory>
#include <GeometricPrimitive.h>

using namespace DirectX;

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

		bool initialize(ID3D11DeviceContext*, WCHAR*, float, const XMVECTOR& _color = Colors::White);
		void shutdown();
		void render(const XMMATRIX& _world, const XMMATRIX& _view, const XMMATRIX& _projection);

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