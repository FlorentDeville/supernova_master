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

#include "snSphere.h"
#include "snAABB.h"
#include "snMatrix44f.h"
#include "snTransform.h"

using namespace Supernova::Vector;

namespace Supernova
{
	snSphere::snSphere(float _radius) : snICollider(), m_radius(_radius)
	{
		m_typeOfCollider = snEColliderSphere;
	}

	snVec snSphere::getCenter() const
	{
		return m_center;
	}

	float snSphere::getRadius()const
	{
		return m_radius;
	}

	void snSphere::setTransform(const snTransform& _transform)
	{
		m_center = _transform.getPosition();
	}

	void snSphere::initialize()
	{
		//nothing to do
	}

	void snSphere::computeLocalInertiaTensor(float _mass, snMatrix44f& _inertiaTensor) const
	{
		float inertia = 0.4f * _mass * m_radius * m_radius;
		_inertiaTensor.m_r[0] = snVec4Set(inertia, 0, 0, 0);
		_inertiaTensor.m_r[1] = snVec4Set(0, inertia, 0, 0);
		_inertiaTensor.m_r[2] = snVec4Set(0, 0, inertia, 0);
		_inertiaTensor.m_r[3] = snVec4Set(0, 0, 0, 0);
	}

	void snSphere::computeAABB(snAABB * const _boundingVolume) const
	{
		snVec vecRadius = snVec4Set(m_radius, m_radius, m_radius, 0);
		_boundingVolume->m_max = m_center + vecRadius;
		_boundingVolume->m_min = m_center - vecRadius;
	}
}