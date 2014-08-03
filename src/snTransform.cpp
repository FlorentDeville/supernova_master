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
		m_localPosition = snVec4Set(0, 0, 0, 1);
		m_localOrientation = snVec4Set(0, 0, 0, 1);
		m_localScale = snVec4Set(1, 1, 1, 0);

		m_localToWorld.identity();
		m_localDirty = true;

		m_parent = 0;
		m_parentDirty = false;
	}

	snTransform::snTransform(const snVec& _localPosition)
	{
		m_localPosition = _localPosition;
		m_localOrientation = snVec4Set(0, 0, 0, 1);
		m_localScale = snVec4Set(1, 1, 1, 0);
		m_localDirty = true;

		m_parent = 0;
		m_parentDirty = false;
	}

	snTransform::snTransform(const snVec& _localPosition, const snVec& _localOrientation)
	{
		m_localPosition = _localPosition;
		m_localOrientation = _localOrientation;
		m_localScale = snVec4Set(1, 1, 1, 0);

		m_localDirty = true;

		m_parent = 0;
		m_parentDirty = false;
	}

	snTransform::snTransform(const snVec& _localPosition, const snVec& _localOrientation, const snVec& _localScale)
	{
		m_localPosition = _localPosition;
		m_localOrientation = _localOrientation;
		m_localScale = _localScale;
		m_localDirty = true;

		m_parent = 0;
		m_parentDirty = false;
	}

	/*snTransform::snTransform(const snVec& _position, const snVec& _orientation, const snVec& _scale, const snMatrix44f& _localToWorld)
	{
		m_position = _position;
		m_scale = _scale;
		m_orientation = _orientation;
		m_localToWorld = _localToWorld;
		m_dirty = false;
	}*/

	snTransform::~snTransform()
	{}

	snVec snTransform::getLocalPosition() const
	{
		return m_localPosition;
	}

	snVec snTransform::getLocalOrientation() const
	{
		return m_localOrientation;
	}

	snVec snTransform::getLocalScale() const
	{
		return m_localScale;
	}

	snVec snTransform::getPosition() const
	{
		computeTransform();
		return m_position;
	}

	snVec snTransform::getOrientation() const
	{
		computeTransform();
		return m_orientation;
	}

	snVec snTransform::getScale() const
	{
		computeTransform();
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
		computeTransform();
		if (m_parent == 0)
			return m_localToParent;
		else
			return m_localToWorld;
	}

	void snTransform::setLocalPosition(const snVec& _position)
	{
		m_localPosition = _position;
		m_localDirty = true;
		dirtyAllChildren();
	}

	void snTransform::setLocalOrientation(const snVec& _orientation)
	{
		m_localOrientation = _orientation;
		m_localDirty = true;
		dirtyAllChildren();
	}

	void snTransform::setLocalEulerAngles(const snVec& _eulerAngles)
	{
		m_localOrientation = snQuaternionFromEuler(_eulerAngles);
		m_localDirty = true;
		dirtyAllChildren();
	}

	void snTransform::setLocalEulerAngles(float _x, float _y, float _z)
	{
		m_localOrientation = snQuaternionFromEuler(_x, _y, _z);
		m_localDirty = true;
		dirtyAllChildren();
	}

	void snTransform::setLocalScale(const snVec& _scale)
	{
		m_localScale = _scale;
		m_localDirty = true;
		dirtyAllChildren();
	}

	void snTransform::setDirtyParent() const
	{
		m_parentDirty = true;
		dirtyAllChildren();
	}
	
	void snTransform::setParent(snTransform* const _parent)
	{
		m_parent = _parent;
		m_parentDirty = true;
	}

	void snTransform::addChild(snTransform* const _child)
	{
		m_children.push_back(_child);
		_child->setDirtyParent();
	}

	void snTransform::removeChild(snTransform* const _child)
	{
		for (vector<snTransform*>::const_iterator i = m_children.cbegin(); i != m_children.cend(); ++i)
		{
			if ((*i) == _child)
			{
				m_children.erase(i);
				_child->setDirtyParent();
				return;
			}
		}
	}

	void snTransform::computeTransform() const
	{
		//Nothing to recompute
		if (!m_parentDirty && !m_localDirty)
			return;

		if (m_localDirty) 
		{
			//compute the local to parent matrix.
			snMatrix44f rot;
			rot.createRotationFromQuaternion(m_localOrientation);
			rot.m_r[3] = m_localPosition;

			snMatrix44f scale;
			scale.createScale(m_localScale);
			snMatrixMultiply4(scale, rot, m_localToParent);
		}

		//Compute the global position, orientation and scale
		if (m_parent != 0)
		{
			m_position = m_parent->getPosition() + m_localPosition;
			snVec4SetW(m_position, 1);
			m_orientation = snQuaternionMultiply(m_localOrientation, m_parent->getOrientation());
			m_scale = m_localScale * m_parent->getScale();

			//Compute the local to world matrix
			snMatrixMultiply4(m_localToParent, m_parent->getLocalToWorld(), m_localToWorld);
			//snMatrixMultiply4(m_parent->getLocalToWorld(), m_localToParent, m_localToWorld);
		}
		else
		{
			m_position = m_localPosition;
			m_orientation = m_localOrientation;
			m_scale = m_localScale;
		}

		m_localDirty = false;
		m_parentDirty = false;	
	}

	void snTransform::dirtyAllChildren() const
	{
		for (vector<snTransform*>::const_iterator i = m_children.cbegin(); i != m_children.cend(); ++i)
			(*i)->setDirtyParent();
	}

	void snTransformAddLink(snTransform* const _parent, snTransform* const _child)
	{
		_parent->addChild(_child);
		_child->setParent(_parent);
	}

	void snTransformRemoveLink(snTransform* const _parent, snTransform* const _child)
	{
		_parent->removeChild(_child);
		_child->setParent(0);
	}

	//void snTransformMultiply(const snTransform& _first, const snTransform& _second, snTransform& result)
	//{
	//	snMatrix44f resLocalToWorld;
	//	snMatrixMultiply4(_first.getLocalToWorld(), _second.getLocalToWorld(), resLocalToWorld);

	//	//result.setPosition(resLocalToWorld.m_r[3]);
	//	snVec q = snQuaternionMultiply(_first.getOrientation(), _second.getOrientation());
	//	//result.setOrientation(q);
	//	//result.setScale(resLocalToWorld.getScale());
	//}


}