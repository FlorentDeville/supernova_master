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

#include "snTransform.h"
#include "snQuaternion.h"

using namespace Supernova::Vector;

namespace Supernova
{
	snTransform::snTransform()
	{
		m_position = snVec4Set(0, 0, 0, 1);
		m_orientation = snVec4Set(0, 0, 0, 1);
		m_scale = snVec4Set(1, 1, 1, 0);

		m_localToWorld.identity();
		m_dirty = false;
	}

	snTransform::snTransform(const snVec& _position)
	{
		m_position = _position;
		m_orientation = snVec4Set(0, 0, 0, 1);
		m_scale = snVec4Set(1, 1, 1, 0);
		m_dirty = true;
	}

	snTransform::snTransform(const snVec& _position, const snVec& _orientation)
	{
		m_position = _position;
		m_orientation = _orientation;
		m_scale = snVec4Set(1, 1, 1, 0);

		m_dirty = true;
	}

	snTransform::snTransform(const snVec& _position, const snVec& _orientation, const snVec& _scale)
	{
		m_position = _position;
		m_scale = _scale;
		m_orientation = _orientation;
		m_dirty = true;
	}

	snTransform::snTransform(const snVec& _position, const snVec& _orientation, const snVec& _scale, const snMatrix44f& _localToWorld)
	{
		m_position = _position;
		m_scale = _scale;
		m_orientation = _orientation;
		m_localToWorld = _localToWorld;
		m_dirty = false;
	}

	snTransform::~snTransform()
	{}

	snVec snTransform::getPosition() const
	{
		return m_position;
	}

	snVec snTransform::getOrientation() const
	{
		return m_orientation;
	}

	snVec snTransform::getScale() const
	{
		return m_scale;
	}

	snVec snTransform::getRight() const
	{
		const snMatrix44f& localToWorld = getLocalToWorld();
		return localToWorld.m_r[0];
	}

	snVec snTransform::getUp() const
	{
		const snMatrix44f& localToWorld = getLocalToWorld();
		return localToWorld.m_r[1];
	}

	snVec snTransform::getForward() const
	{
		const snMatrix44f& localToWorld = getLocalToWorld();
		return localToWorld.m_r[2];
	}

	const snMatrix44f& snTransform::getLocalToWorld() const
	{
		if (m_dirty) //recompute the local to world
		{
			snMatrix44f rot;
			rot.createRotationFromQuaternion(m_orientation);
			rot.m_r[3] = m_position;

			snMatrix44f scale;
			scale.createScale(m_scale);
			snMatrixMultiply4(scale, rot, m_localToWorld);

			m_dirty = false;
		}

		return m_localToWorld;
	}

	void snTransform::setPosition(const snVec& _position)
	{
		m_position = _position;
		m_dirty = true;
	}

	void snTransform::setOrientation(const snVec& _orientation)
	{
		m_orientation = _orientation;
		m_dirty = true;
	}

	void snTransform::setEulerAngles(const snVec& _eulerAngles)
	{
		m_orientation = snQuaternionFromEuler(_eulerAngles);
		m_dirty = true;
	}

	void snTransform::setScale(const snVec& _scale)
	{
		m_scale = _scale;
		m_dirty = true;
	}

	void snTransformMultiply(const snTransform& _first, const snTransform& _second, snTransform& result)
	{
		snMatrix44f resLocalToWorld;
		snMatrixMultiply4(_first.getLocalToWorld(), _second.getLocalToWorld(), resLocalToWorld);

		result.setPosition(resLocalToWorld.m_r[3]);
		snVec q = snQuaternionMultiply(_first.getOrientation(), _second.getOrientation());
		result.setOrientation(q);
		result.setScale(resLocalToWorld.getScale());
	}
}