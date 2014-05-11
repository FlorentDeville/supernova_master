#ifndef CAMERA_H
#define CAMERA_H

//Forward declarations
namespace DirectX
{
	struct XMFLOAT3;
}
using DirectX::XMFLOAT3;

#include <DirectXMath.h>
using DirectX::XMMATRIX;

#include "snVec.inl"
using Supernova::snVec;

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
		void Render(const snVec& _position, const snVec& _lookAt, const snVec& _up);
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
