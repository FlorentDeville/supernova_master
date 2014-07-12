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

#include "snCapsule.h"
#include "snAABB.h"
using namespace Supernova::Vector;

namespace Supernova
{
	snCapsule::snCapsule(float _length, float _radius)
	{
		m_typeOfCollider = snEColliderCapsule;
		m_radius = _radius;

		float halfLength = _length * 0.5f;
		m_localA = snVec4Set(0, -halfLength, 0, 1);
		m_localB = snVec4Set(0, halfLength, 0, 1);

		m_a = m_localA;
		m_b = m_localB;
	}

	snCapsule::snCapsule(const snVec& _a, const snVec& _b, float _radius)
	{
		m_typeOfCollider = snEColliderCapsule;
		m_localA = _a;
		m_localB = _b;
		m_a = _a;
		m_b = _b;
		m_radius = _radius;
	}

	snVec snCapsule::getFirstEndPoint() const
	{
		return m_a;
	}

	snVec snCapsule::getSecondEndPoint() const
	{
		return m_b;
	}

	float snCapsule::getRadius() const
	{
		return m_radius;
	}

	void snCapsule::initialize()
	{

	}

	void snCapsule::setTransform(const snMatrix44f& _transform)
	{
		m_a = snMatrixTransform4(m_localA, _transform);
		m_b = snMatrixTransform4(m_localB, _transform);
	}

	void snCapsule::computeLocalInertiaTensor(float _mass, snMatrix44f& _inertiaTensor) const
	{
		//let's not calculate the real inertia but approximate it with a box inertia tensor of dimension (radius, length, radius)
		float length = snVec3Norme(m_b - m_a);
		snVec size = snVec4Set(m_radius * 2, length, m_radius * 2, 0);
		snVec sqSize = size * size;

		float massOverTwelve = _mass / 12.f;
		_inertiaTensor.m_r[0] = snVec4Set(massOverTwelve * (snVec4GetY(sqSize) + snVec4GetZ(sqSize)), 0, 0, 0);
		_inertiaTensor.m_r[1] = snVec4Set(0, massOverTwelve * (snVec4GetX(sqSize) + snVec4GetZ(sqSize)), 0, 0);
		_inertiaTensor.m_r[2] = snVec4Set(0, 0, massOverTwelve * (snVec4GetX(sqSize) + snVec4GetY(sqSize)), 0);
		_inertiaTensor.m_r[3] = snVec4Set(0, 0, 0, 1);
	}

	void snCapsule::computeAABB(snAABB * const _boundingVolume) const
	{
		snVec extends = snVec4Set(m_radius, m_radius, m_radius, 0);

		snVec extendedA = m_a - extends;
		snVec extendedB = m_b - extends;
		_boundingVolume->m_min = snVec4GetMin(extendedA, extendedB);

		extendedA = m_a + extends;
		extendedB = m_b + extends;
		_boundingVolume->m_max = snVec4GetMax(extendedA, extendedB);
	}

	void snCapsule::getClosestFeature(const snVec& _n, snVec* const _polygon, unsigned int& _count, unsigned int& _featureId) const
	{
		snVec ab = m_b - m_a;
		
		//check if the feature is the first end
		float dot = snVec4GetX(snVec3Dot(ab, _n));

		const float EPSILON = 1e-2f;

		if (dot < -EPSILON) //first endpoint
		{
			_count = 1;
			_polygon[0] = m_a + _n * m_radius;
			_featureId = 0;
		}
		else if (dot > EPSILON) //second endpoint
		{
			_count = 1;
			_polygon[0] = m_b + _n * m_radius;
			_featureId = 1;
		}
		else // cylinder part
		{
			_count = 2;
			_polygon[0] = m_a + _n * m_radius;
			_polygon[1] = m_b + _n * m_radius;
			_featureId = 2;
		}
	}


	snVec snCapsule::support(const snVec& _direction, float& _distance) const
	{
		snVec dir = m_a - m_b;

		snVec s = m_b + (_direction * snVec3Dot(dir, _direction)) + (_direction * m_radius);
		_distance = snVec4GetX(snVec3Dot(_direction, s));

		return s;
	}

	snVec snCapsule::anyPoint() const
	{
		return m_a;
	}
}