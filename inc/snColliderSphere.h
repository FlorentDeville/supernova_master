#ifndef COLLIDER_SHPERE_H
#define COLLIDER_SPHERE_H

#include "snICollider.h"

namespace Supernova
{
	class snColliderSphere : public snICollider
	{
	private:
		/*Sphere radius.*/
		float m_radius;

	public:
		snColliderSphere();

		snColliderSphere(float);

		snColliderSphere(float, const snVec&);

		float getRadius()const;

		snVec getWorldOrigin()const;

		void setWorldTransform(const snMatrix44f& _transform);

		snVec getLocalCenterOfMass() const;

		void computeProjection(const snVec& _direction, float& _min, float& _max) const;
	};
}

#endif