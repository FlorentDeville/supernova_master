#ifndef ENTITY_SPHERE_H
#define ENTITY_SPHERE_H

#include "IWorldEntity.h"

namespace Devil
{
	class IGfxEntity;
	
	_declspec(align(16)) class EntitySphere : public IWorldEntity
	{
	private:
		float m_diameter;
		IGfxEntity* m_gfx;
		XMVECTOR m_color;

	public:
		EntitySphere();
		virtual ~EntitySphere();

		bool initialize(float _diameter, const XMVECTOR& _color);

		void update();
		void render();

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
