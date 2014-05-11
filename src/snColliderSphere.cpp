#include "snColliderSphere.h"

namespace Supernova
{
	snColliderSphere::snColliderSphere() : snICollider(), m_radius(0)
	{
		m_typeOfCollider = snEColliderSphere;
	}

	snColliderSphere::snColliderSphere(float _radius) : snICollider(), m_radius(_radius)
	{
	}

	snColliderSphere::snColliderSphere(float _radius, const snVec& _origin) : snICollider(_origin), m_radius(_radius)
	{
	}

	float snColliderSphere::getRadius()const
	{
		return m_radius;
	}

	snVec snColliderSphere::getWorldOrigin()const
	{
		return m_worldOrigin;
	}

	void snColliderSphere::setWorldTransform(const snMatrix44f& _transform)
	{
		m_worldOrigin = _transform[3];
	}

	snVec snColliderSphere::getLocalCenterOfMass() const
	{
		return m_origin;
	}

	void snColliderSphere::computeProjection(const snVec& /*_direction*/, float& /*_min*/, float& /*_max*/) const
	{}
}