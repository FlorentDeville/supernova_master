#include "snColliderPlan.h"

using namespace Supernova::Vector;

namespace Supernova
{
	snColliderPlan::snColliderPlan(float width, float height)
	{
		m_typeOfCollider = snEColliderPlan;
		snVec corners[4];

		float halfWidth = width * 0.5f;
		float halfHeight = height * 0.5f;

		corners[0] = snVec4Set(halfWidth, 0, halfHeight, 1);
		corners[1] = snVec4Set(halfWidth, 0, -halfHeight, 1);
		corners[2] = snVec4Set(-halfWidth, 0, -halfHeight, 1);
		corners[3] = snVec4Set(-halfWidth, 0, halfHeight, 1);

		initialize(corners);
	}

	snColliderPlan::snColliderPlan(const snVec* _corners)
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
			m_worldCorners[i] = snMatrixTransform4(m_corners[i], _transform);
		}

		m_worldNormal = snMatrixTransform3(m_normal, _transform);
	}

	const snVec& snColliderPlan::getNormal()const
	{
		return m_normal;
	}

	const snVec* snColliderPlan::getCorners()const
	{
		return m_corners;
	}

	snVec snColliderPlan::getLocalCenterOfMass() const
	{
		return snVec4Set(0, 0, 0, 1);
	}

	const snVec& snColliderPlan::getWorldNormal() const
	{
		return m_worldNormal;
	}

	const snVec* snColliderPlan::getWorldCorners() const
	{
		return m_worldCorners;
	}

	snVec snColliderPlan::getProjection(const snVec& _o) const
	{
		//Here is the idea:
		//P = O - Nl (1) and as P is in the plan it is solution of the plan equation P.N = -d (2)
		//Solving this system of equations to find l we end up with l=(d + N.O) / N.N (3)
		//By putting (3) in (1) we can calculate P.

		float l = getDistance(_o);

		//compute p
		return _o - (getWorldNormal() * l);

	}

	float snColliderPlan::getDistance(const snVec& _o) const
	{
		//compute -d in ax + by + cy = -d using the normal and a corner
		float minusD = snVec3Dot(getWorldNormal(), getWorldCorners()[0]);

		//compute l
		float l = (-minusD + snVec3Dot(getWorldNormal(), _o)) / snVec3SquaredNorme(getWorldNormal());

		return l;
	}

	void snColliderPlan::initialize(const snVec* _corners)
	{
		//m_TypeOfCollider = ColliderType::Collider_Plan;

		for (int i = 0; i < 4; ++i)
			m_corners[i] = _corners[i];

		//calculate edges vector
		snVec e1 = m_corners[1] - m_corners[0];
		snVec e2 = m_corners[1] - m_corners[2];

		//calculate normal
		m_normal = snVec3Cross(e2, e1);
		snVec3Normalize(m_normal);
		snVec4SetW(m_normal, 0);

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