#ifndef ENTITY_BOX_H
#define ENTITY_BOX_H

#include "IWorldEntity.h"

namespace Devil
{
	class GfxEntityBox;

	class EntityBox: public IWorldEntity
	{
	private:
		XMFLOAT3 m_size;

		GfxEntityBox* m_gfx;

	public:
		EntityBox();
		virtual ~EntityBox();

		bool initialize(const XMFLOAT3&, const XMFLOAT4&);

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