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

#ifndef SN_TRANSFORM_H
#define SN_TRANSFORM_H

#include "snMatrix44f.h"

#include <vector>
using std::vector;

namespace Supernova
{
	//Store the position, orientation and scale of an object and let you manipulate those information.
	class snTransform
	{
	private:
		//Position of the transform in world space
		mutable snVec m_position;

		//Orientation of the transform in world space as a quaternion.
		mutable snVec m_orientation;

		//Global scale of the transform.
		mutable snVec m_scale;

		//Position of the transform in the parent's space.
		snVec m_localPosition;

		//Orientation of the transform in the parent's space.
		snVec m_localOrientation;

		//Scale of the transform in the parent's space.
		snVec m_localScale;

		//Matrix to transform a point from local space into world space.
		mutable snMatrix44f m_localToWorld;

		//Matrix to transform a point from the local space into the parent space. 
		// This matrix is computed with only the local position, local orientation and local scale.
		mutable snMatrix44f m_localToParent;

		//Flag to indicate if any of the local position, local orientation and local scale has changed.
		mutable bool m_localDirty;

		//Flag to indicate that the parent transform has changed.
		mutable bool m_parentDirty;

		//List of children transform.
		vector<snTransform*> m_children;

		//Parent children. Null if this transform has no parent.
		snTransform* m_parent;

	public:
		//Default constructor initialized to the identity.
		snTransform();

		//Construction of a transform from a position.
		// _position : the position of the transform in world space.
		snTransform(const snVec& _localPosition);

		//Construction of a transform from a positon and orientation.
		// _position : the position of the transform in world space.
		// _orientation : orientation as a quaternion in world space.
		snTransform(const snVec& _localPosition, const snVec& _localOrientation);

		//Construction of a transform from a a position, orientation and scale.
		// _position : the position of the transform in world space.
		// _orientation : orientation as a quaternion in world space.
		// _scale : scale of the transform in world space.
		snTransform(const snVec& _localPosition, const snVec& _localOrientation, const snVec& _localScale);

		//Construction of a transform from a a position, orientation and scale.
		// _position : the position of the transform in world space.
		// _orientation : orientation as a quaternion in world space.
		// _scale : scale of the transform in world space.
		// _localToWorld : matrix to transform a point from local space to world space.
		//snTransform(const snVec& _localPosition, const snVec& _localOrientation, const snVec& _localScale, const snMatrix44f& _localToWorld);

		virtual ~snTransform();

		//Return the local position of the transform.
		snVec getLocalPosition() const;

		//Return the local orientation of the transform.
		snVec getLocalOrientation() const;

		//Return the local scale of the transform.
		snVec getLocalScale() const;

		//Return the position of the transform in world space.
		snVec getPosition() const;

		//Return the orientation as a quaternion of the transform in world space.
		snVec getOrientation() const;

		//Return the scale of the transform in world space.
		snVec getScale() const;

		//Return the right vector of the transform (x axis)
		snVec getRight() const;

		//Return the up vector of the transform (y axis)
		snVec getUp() const;

		//Return the forward vector of the transform (z axis)
		snVec getForward() const;

		//Return a matrix to transform a point from local space to world space.
		const snMatrix44f& getLocalToWorld() const;

		//Set the local position of the transform.
		void setLocalPosition(const snVec& _localPosition);

		//Set the local orientation of the transform using a quaternion.
		void setLocalOrientation(const snVec& _localOrientation);

		//Set the local orientation of the transform using euler angles.
		void setLocalEulerAngles(const snVec& _localEulerAngles);

		//Set the local orientation of the transform using euler angles.
		void setLocalEulerAngles(float _x, float _y, float _z);

		//Set the local scale of the transform.
		void setLocalScale(const snVec& _scale);

		//Set the dirty parent flag to true;
		void setDirtyParent() const;

		//Set the parent of this transform.
		void setParent(snTransform* const _parent);

		//Add a child to transform.
		void addChild(snTransform* const _child);

		//Remove a child from the transform.
		void removeChild(snTransform* const _child);

	private:

		//If the transform is dirty, recompute everything.
		void computeTransform() const;

		//Set all the children dirty
		void dirtyAllChildren() const;
	};

	//Create a link between a parent transform and a child transform.
	//_child will have _parent as a parent. _parent will have _child has a child transform.
	void snTransformAddLink(snTransform* const _parent, snTransform* const _child);

	//Remove a link between a parent transform and a child transform. 
	//The child will have no parent. The parent won't have _child has a child transform.
	void snTransformRemoveLink(snTransform* const _parent, snTransform* const _child);

	//void snTransformMultiply(const snTransform& _first, const snTransform& _second, snTransform& result);
}
#endif //ifndef SN_TRANSFORM_H