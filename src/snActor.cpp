/****************************************************************************/
/*Copyright (c) 2014, Florent DEVILLE.                                      */
/*All rights reserved.                                                      */
/*                                                                          */
/*Redistribution and use in source and binary forms, with or without        */
/*modification, are permitted provided that the following conditions        */
/*are met:                                                                  */
/*                                                                          */
/* - Redistributions of source code must retain the above copyright         */
/*notice, this list of conditions and the following disclaimer.             */
/* - Redistributions in binary form must reproduce the above                */
/*copyright notice, this list of conditions and the following               */
/*disclaimer in the documentation and/or other materials provided           */
/*with the distribution.                                                    */
/* - The names of its contributors cannot be used to endorse or promote     */
/*products derived from this software without specific prior written        */
/*permission.                                                               */
/* - The source code cannot be used for commercial purposes without         */
/*its contributors' permission.                                             */
/*                                                                          */
/*THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       */
/*"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         */
/*LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         */
/*FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE            */
/*COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,       */
/*INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,      */
/*BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;          */
/*LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER          */
/*CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT        */
/*LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN         */
/*ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           */
/*POSSIBILITY OF SUCH DAMAGE.                                               */
/****************************************************************************/

#include "snActor.h"

#include "snICollider.h"
#include "snColliderBox.h"
#include "snColliderPlan.h"
#include "snColliderSphere.h"

#include <assert.h>

namespace Supernova
{
	snActor::snActor() : m_material(), m_linearDamping(0.5f), m_angularDamping(0.f)
	{
		m_w = snVector4f(0, 0, 0, 0);
		m_q = snVector4f(0, 0, 0, 1);
		m_R.identity();
		m_invR.identity();

		m_isKinematic = false;
		m_isStatic = false;
	}

	snActor::snActor(bool _isStatic) : m_material(), m_linearDamping(0.5f), m_angularDamping(0.f)
	{
		m_w = snVector4f(0, 0, 0, 0);
		m_q = snVector4f(0, 0, 0, 1);
		m_R.identity();
		m_invR.identity();

		m_isKinematic = false;
		m_isStatic = _isStatic;
	}

	snActor::~snActor()
	{
		for (vector<snICollider*>::iterator i = m_colliders.begin(); i != m_colliders.end(); ++i)
			delete *i;

		m_colliders.clear();
	}

	string snActor::getName() const
	{
		return m_name;
	}

	void snActor::setName(const string& _name)
	{
		m_name = _name;
	}

	float snActor::getMass() const
	{
		return m_mass;
	}

	void snActor::setMass(float _mass)
	{
		m_mass = _mass;
	}

	float snActor::getInvMass()const
	{
		if (m_isKinematic || m_isStatic)
			return 0;
		return m_massInv;
	}

	const snMatrix44f& snActor::getInertiaTensor() const
	{
		return m_inertiaTensor;
	}

	const snMatrix44f snActor::getInvInertiaTensor() const
	{
		if (m_isKinematic || m_isStatic)
			return snMatrix44f::m_zero;

		return m_invInertiaTensor;
	}

	void snActor::computeWInertiaTensor()
	{
		//WI = R * I * RT
		snMatrix44f RT;
		m_R.transpose(RT);

		snMatrix44f J = getInertiaTensor();
		snMatrix44f WJ;
		snMatrixMultiply3(m_R, J, WJ);
		snMatrixMultiply3(WJ, RT, m_worldInertiaTensor);
	}

	void snActor::computeWInvInertiaTensor()
	{
		snMatrix44f RT;
		m_R.transpose(RT);

		snMatrix44f InvJ = getInvInertiaTensor();
		snMatrix44f WInvJ;
		snMatrixMultiply3(m_R, InvJ, WInvJ);
		snMatrixMultiply3(WInvJ, RT, m_invWorldInertiaTensor);
	}

	const snMatrix44f& snActor::getWorldInertia()const
	{
		return m_worldInertiaTensor;
	}

	const snMatrix44f& snActor::getInvWorldInertia()const
	{
		return m_invWorldInertiaTensor;
	}

	bool snActor::getIsKinematic()const
	{
		return m_isKinematic;
	}

	void snActor::setIsKinematic(bool _isKinematic)
	{
		m_isKinematic = _isKinematic;
	}

	bool snActor::getIsStatic() const
	{
		return m_isStatic;
	}

	snVector4f snActor::getPosition()const
	{
		return m_x;
	}

	void snActor::setPosition(const snVector4f& _x)
	{
		m_x = _x;
	}

	snVector4f snActor::getLinearVelocity()const
	{
		return m_v;
	}

	void snActor::setLinearVelocity(const snVector4f& _v)
	{
		m_v = _v;
	}

	snVector4f snActor::getAngularVelocity() const
	{
		return m_w;
	}

	void snActor::setAngularVelocity(const snVector4f& _w)
	{
		m_w = _w;
	}

	snVector4f snActor::getOrientationQuaternion() const
	{
		return m_q;
	}

	void snActor::setOrientationQuaternion(const snVector4f& _q)
	{
		m_q = _q;
	}

	const snMatrix44f& snActor::getOrientationMatrix() const
	{
		return m_R;
	}

	void snActor::setOrientationMatrix(const snMatrix44f& _R)
	{
		m_R = _R;
		m_invR = m_R.inverse();
	}

	const snMatrix44f& snActor::getInverseOrientationMatrix() const
	{
		return m_invR;
	}

	void snActor::setLinearDamping(float _linearDamping)
	{
		m_linearDamping = _linearDamping;
	}

	void snActor::setAngularDamping(float _angularDamping)
	{
		m_angularDamping = _angularDamping;
	}

	float snActor::getLinearDamping() const
	{
		return m_linearDamping;
	}

	float snActor::getAngularDamping() const
	{
		return m_angularDamping;
	}

	void snActor::createColliderBox(snColliderBox** _box, int& _colliderId)
	{
		*_box = new snColliderBox();

		_colliderId = attachCollider(*_box);
	}

	void snActor::deleteCollider(unsigned int _colliderId)
	{
		if (_colliderId >= m_colliders.size())
			return;

		if (m_colliders[_colliderId] == 0)
			return;

		delete m_colliders[_colliderId];
		m_colliders[_colliderId] = 0;
	}

	int snActor::attachCollider(snICollider* _collider)
	{
		//try to add it to the vector
		for (unsigned int i = 0; i < m_colliders.size(); ++i)
		{
			if (m_colliders[i] != 0)
				continue;

			m_colliders[i] = _collider;
			return i;
		}

		//no existing spot found so push back
		int colliderId = m_colliders.size();
		m_colliders.push_back(_collider);
		return colliderId;
	}

	
	void snActor::removeCollider(unsigned int _colliderId)
	{
		if (m_colliders.size() <= _colliderId)
			return;

		m_colliders[_colliderId] = 0;
	}

	snICollider* snActor::getCollider(unsigned int _colliderId)
	{
		if (_colliderId >= m_colliders.size())
			return 0;

		return m_colliders[_colliderId];
	}

	std::vector<snICollider*>& snActor::getColliders()
	{
		return m_colliders;
	}

	bool snActor::initialize()
	{
		//calculate the mass inverse
		m_massInv = 1.f / m_mass;

		//calculate the inertia tensor
		computeInertiaTensor();

		updateCollider(m_x, m_R);

		//initialize state value
		return true;
	}

	bool snActor::updateCollider(const snVector4f& _x, const snMatrix44f& _R)
	{
		snMatrix44f translation;
		translation.createTranslation(_x);

		snMatrix44f transform;
		snMatrixMultiply4(_R, translation, transform);

		for (std::vector<snICollider*>::iterator i = m_colliders.begin(); i != m_colliders.end(); ++i)
			(*i)->setWorldTransform(transform);

		return true;
	}

	snPhysicMaterial& snActor::getPhysicMaterial()
	{
		return m_material;
	}

	void* snActor::operator new(size_t _count)
	{
		return _aligned_malloc(_count, SN_ALIGN_SIZE);
	}

	void snActor::operator delete(void* _p)
	{
		_aligned_free(_p);
	}

	void snActor::computeInertiaTensor()
	{
		/*switch (m_colliders[0]->getType())
		{
			
			case ICollider::ColliderType::Collider_Sphere:
				{
					ColliderSphere* sphere = static_cast<ColliderSphere*>(m_colliders[0]);
					float inertia = (2 * m_mass * sphere->getRadius() * sphere->getRadius()) / 5.f;
					m_inertiaTensor.r[0].m128_f32[XMVEC_ID_X] = inertia;
					m_inertiaTensor.r[0].m128_f32[XMVEC_ID_Y] = 0;
					m_inertiaTensor.r[0].m128_f32[XMVEC_ID_Z] = 0;
					m_inertiaTensor.r[0].m128_f32[XMVEC_ID_W] = 0;

					m_inertiaTensor.r[1].m128_f32[XMVEC_ID_X] = 0;
					m_inertiaTensor.r[1].m128_f32[XMVEC_ID_Y] = inertia;
					m_inertiaTensor.r[1].m128_f32[XMVEC_ID_Z] = 0;
					m_inertiaTensor.r[1].m128_f32[XMVEC_ID_W] = 0;

					m_inertiaTensor.r[2].m128_f32[XMVEC_ID_X] = 0;
					m_inertiaTensor.r[2].m128_f32[XMVEC_ID_Y] = 0;
					m_inertiaTensor.r[2].m128_f32[XMVEC_ID_Z] = inertia;
					m_inertiaTensor.r[2].m128_f32[XMVEC_ID_W] = 0;

					m_inertiaTensor.r[3].m128_f32[XMVEC_ID_X] = 0;
					m_inertiaTensor.r[3].m128_f32[XMVEC_ID_Y] = 0;
					m_inertiaTensor.r[3].m128_f32[XMVEC_ID_Z] = 0;
					m_inertiaTensor.r[3].m128_f32[XMVEC_ID_W] = 1;
				}
			break;

		default:
			assert(false);
		}*/

		//compute inertia of the collider
		for (vector<snICollider*>::const_iterator i = m_colliders.cbegin(); i != m_colliders.cend(); ++i)
		{
			snMatrix44f inertia;
			(*i)->computeLocalInertiaTensor(m_mass, inertia);

			//add the inertia of the collider to the global collider
			m_inertiaTensor = m_inertiaTensor + inertia;
		}

		//compute the inverse inertia tensor
		m_invInertiaTensor = m_inertiaTensor.inverse();

		computeWInertiaTensor();
		computeWInvInertiaTensor();
	}
}