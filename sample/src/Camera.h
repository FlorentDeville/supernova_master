#ifndef CAMERA_H
#define CAMERA_H

#include <DirectXMath.h>

using namespace DirectX;

namespace Supernova
{
	class snVector4f;
}
using namespace Supernova;

namespace Devil
{
	class Camera
	{
	private:
		float m_positionX, m_positionY, m_positionZ;
		float m_rotationX, m_rotationY, m_rotationZ;
		XMMATRIX m_viewMatrix;

	public:
		Camera();
		virtual ~Camera();

		void SetPosition(float, float, float);
		void SetRotation(float, float, float);

		XMFLOAT3 GetPosition();
		XMFLOAT3 GetRotation();

		void Render();
		void Render(const snVector4f& _position, const snVector4f& _lookAt, const snVector4f& _up);
		void GetViewMatrix(XMMATRIX&);

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
