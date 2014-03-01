#ifndef COLLIDER_SHPERE_H
#define COLLIDER_SPHERE_H

#include "snICollider.h"

namespace Supernova
{
	class snVector4f;

	class snColliderSphere : public snICollider
	{
	private:
		/*Sphere radius.*/
		float m_radius;

	public:
		snColliderSphere();

		snColliderSphere(float);

		snColliderSphere(float, const snVector4f&);

		float getRadius()const;

		snVector4f getWorldOrigin()const;

		void setWorldTransform(const snMatrix44f& _transform);

		snVector4f getLocalCenterOfMass() const;
	};
}

#endif