#ifndef COLLIDER_PLAN_H
#define COLLIDER_PLAN_H

#include "snICollider.h"

namespace Supernova
{
	class snColliderPlan : public snICollider
	{
	private:
		//The four corners making the plan
		snVec m_corners[4];

		snVec m_worldCorners[4];

		//Plans normal.
		snVec m_normal;

		snVec m_worldNormal;

	public:
		snColliderPlan(float width, float height);
		snColliderPlan(const snVec* _corners);
		virtual ~snColliderPlan();

		void setWorldTransform(const snMatrix44f& _transform);

		const snVec& getNormal()const;

		const snVec* getCorners()const;

		snVec getLocalCenterOfMass() const;

		const snVec& getWorldNormal() const;

		const snVec* getWorldCorners() const;

		snVec getProjection(const snVec&) const;

		float getDistance(const snVec& _o) const;

	private:
		void initialize(const snVec* _corners);
	};
}

#endif //COLLIDER_PLAN_H