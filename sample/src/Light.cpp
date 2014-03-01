#include "Light.h"

namespace Devil
{
	Light::Light()
	{
	}

	Light::~Light()
	{
	}

	void Light::setDiffuseColor(float red, float green, float blue, float alpha)
	{
		m_diffuseColor = XMFLOAT4(red, green, blue, alpha);
		return;
	}


	void Light::setDirection(float x, float y, float z)
	{
		m_direction = XMFLOAT3(x, y, z);
		return;
	}


	XMFLOAT4 Light::getDiffuseColor()
	{
		return m_diffuseColor;
	}


	XMFLOAT3 Light::getDirection()
	{
		return m_direction;
	}

}