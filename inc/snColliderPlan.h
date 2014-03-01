#ifndef COLLIDER_PLAN_H
#define COLLIDER_PLAN_H

#include "snICollider.h"

namespace Supernova
{
	class snColliderPlan : public snICollider
	{
	private:
		//The four corners making the plan
		snVector4f m_corners[4];

		snVector4f m_worldCorners[4];

		//Plans normal.
		snVector4f m_normal;

		snVector4f m_worldNormal;

	public:
		snColliderPlan(float width, float height);
		snColliderPlan(const snVector4f* _corners);
		virtual ~snColliderPlan();

		void setWorldTransform(const snMatrix44f& _transform);

		const snVector4f& getNormal()const;

		const snVector4f* getCorners()const;

		snVector4f getLocalCenterOfMass() const;

		const snVector4f& getWorldNormal() const;

		const snVector4f* getWorldCorners() const;

		snVector4f getProjection(const snVector4f&) const;

		float getDistance(const snVector4f& _o) const;

	private:
		void initialize(const snVector4f* _corners);
	};
}

#endif //COLLIDER_PLAN_H