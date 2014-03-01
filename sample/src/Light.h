#ifndef LIGHT_H
#define LIGHT_H

#include <DirectXMath.h>
using namespace DirectX;

namespace Devil
{
	class Light
	{
	private:
		XMFLOAT4 m_diffuseColor;
		XMFLOAT3 m_direction;

	public:
		Light();
		~Light();

		void setDiffuseColor(float, float, float, float);
		void setDirection(float, float, float);

		XMFLOAT4 getDiffuseColor();
		XMFLOAT3 getDirection();	
	};

}

#endif