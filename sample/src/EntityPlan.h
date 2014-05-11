#ifndef ENTITY_PLAN_H
#define ENTITY_PLAN_H

#include "IWorldEntity.h"

using DirectX::XMFLOAT2;
using DirectX::XMFLOAT4;

namespace Devil
{
	class GfxEntityPlan;

	class EntityPlan : public IWorldEntity
	{
	private:
		GfxEntityPlan* m_gfx;

	public:
		EntityPlan();
		virtual ~EntityPlan();

		bool initialize(const XMFLOAT2& _size, const XMFLOAT4& _color);

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
#endif //ENTITY_PLAN_H