#ifndef ENTITY_CAMERA_H
#define ENTITY_CAMERA_H

#include "IWorldEntity.h"

namespace Devil
{
	class Camera;

	class EntityCamera : public IWorldEntity
	{
	private:
		//XMVECTOR m_position;
		XMVECTOR m_lookAt;
		XMVECTOR m_up;

		Camera* m_gfxCamera;

	public:
		EntityCamera();
		~EntityCamera();

		bool initialize(const XMVECTOR& _position, const XMVECTOR& _lookAt, const XMVECTOR& _up);

		void update();
		void render();

		void setLookAt(const XMVECTOR& _lookAt);
		void setUp(const XMVECTOR& _up);

		//Get the camera look at vector
		const XMVECTOR& getLookAt() const;

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