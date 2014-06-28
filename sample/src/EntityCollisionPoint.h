#ifndef ENTITY_COLLISION_POINT_H
#define ENTITY_COLLISION_POINT_H

#include "IWorldEntity.h"

#include <vector>
using namespace std;
#include "snVec.h"

namespace Supernova
{
	class snScene;
}

namespace Devil
{
	class IGfxEntity;

	class EntityCollisionPoint : public IWorldEntity
	{
	private:
		IGfxEntity* m_gfx;
		vector<snVec> m_locations;
		float m_diameter;
	public:
		EntityCollisionPoint();
		virtual ~EntityCollisionPoint();

		bool initialize(float _diameter);

		void update();
		void render();

		void addLocation(const snVec& _location);
		void clearLocations();

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
#endif //ENTITY_COLLISION_POINT_H
