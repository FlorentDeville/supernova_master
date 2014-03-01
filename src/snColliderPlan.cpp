#include "snColliderPlan.h"

namespace Supernova
{
	snColliderPlan::snColliderPlan(float width, float height)
	{
		snVector4f corners[4];

		float halfWidth = width * 0.5f;
		float halfHeight = height * 0.5f;

		corners[0] = snVector4f(halfWidth, 0, halfHeight, 1);
		corners[1] = snVector4f(halfWidth, 0, -halfHeight, 1);
		corners[2] = snVector4f(-halfWidth, 0, -halfHeight, 1);
		corners[3] = snVector4f(-halfWidth, 0, halfHeight, 1);

		initialize(corners);
	}

	snColliderPlan::snColliderPlan(const snVector4f* _corners)
	{
		initialize(_corners);
	}


	snColliderPlan::~snColliderPlan()
	{
	}

	void snColliderPlan::setWorldTransform(const snMatrix44f& _transform)
	{
		for (int i = 0; i < 4; ++i)
		{
			m_worldCorners[i] = _transform * m_corners[i];
		}

		m_worldNormal = _transform * m_normal;
	}

	const snVector4f& snColliderPlan::getNormal()const
	{
		return m_normal;
	}

	const snVector4f* snColliderPlan::getCorners()const
	{
		return m_corners;
	}

	snVector4f snColliderPlan::getLocalCenterOfMass() const
	{
		return snVector4f(0, 0, 0, 1);
	}

	const snVector4f& snColliderPlan::getWorldNormal() const
	{
		return m_worldNormal;
	}

	const snVector4f* snColliderPlan::getWorldCorners() const
	{
		return m_worldCorners;
	}

	snVector4f snColliderPlan::getProjection(const snVector4f& _o) const
	{
		//Here is the idea:
		//P = O - Nl (1) and as P is in the plan it is solution of the plan equation P.N = -d (2)
		//Solving this system of equations to find l we end up with l=(d + N.O) / N.N (3)
		//By putting (3) in (1) we can calculate P.

		float l = getDistance(_o);

		//compute p
		return _o - (getWorldNormal() * l);

	}

	float snColliderPlan::getDistance(const snVector4f& _o) const
	{
		//compute -d in ax + by + cy = -d using the normal and a corner
		float minusD = getWorldNormal().dot(getWorldCorners()[0]);

		//compute l
		float l = (-minusD + getWorldNormal().dot(_o)) / getWorldNormal().squareNorme();

		return l;
	}

	void snColliderPlan::initialize(const snVector4f* _corners)
	{
		//m_TypeOfCollider = ColliderType::Collider_Plan;

		for (int i = 0; i < 4; ++i)
			m_corners[i] = _corners[i];

		//calculate edges vector
		snVector4f e1 = m_corners[1] - m_corners[0];
		snVector4f e2 = m_corners[1] - m_corners[2];

		e1.VEC4FW = 0;
		e2.VEC4FW = 0;

		//calculate normal
		m_normal = e2.cross(e1);
		m_normal.normalize();
		m_normal.VEC4FW = 0;

		//calculate aabb
		//m_aabb[0] = m_aabb[1] = m_corners[0];
	/*	for (int i = 1; i < 4; ++i)
		{
			if (m_aabb[0].VEC4FX > m_corners[i].VEC4FX)
				m_aabb[0].VEC4FX = m_corners[i].VEC4FX;
			if (m_aabb[0].VEC4FY > m_corners[i].VEC4FY)
				m_aabb[0].VEC4FY = m_corners[i].VEC4FY;
			if (m_aabb[0].VEC4FZ > m_corners[i].VEC4FZ)
				m_aabb[0].VEC4FZ = m_corners[i].VEC4FZ;

			if (m_aabb[1].VEC4FX < m_corners[i].VEC4FX)
				m_aabb[1].VEC4FX = m_corners[i].VEC4FX;
			if (m_aabb[1].VEC4FY < m_corners[i].VEC4FY)
				m_aabb[1].VEC4FY = m_corners[i].VEC4FY;
			if (m_aabb[1].VEC4FZ < m_corners[i].VEC4FZ)
				m_aabb[1].VEC4FZ = m_corners[i].VEC4FZ;
		}*/
	}
}