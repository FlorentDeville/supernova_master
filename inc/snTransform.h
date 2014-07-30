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

namespace Supernova
{
	//Store the position, orientation and scale of an object and let you manipulate those information.
	class snTransform
	{
	private:
		//Position of the transform in world space
		snVec m_position;

		//Orientation of the transform in world space as a quaternion.
		snVec m_orientation;

		//Global scale of the transform.
		snVec m_scale;

		//Matrix to transform a point from local space into world space.
		mutable snMatrix44f m_localToWorld;

		//Flag to indicate if the localToWorld matrix as to be computed again.
		mutable bool m_dirty;

	public:
		//Default constructor initialized to the identity.
		snTransform();

		//Construction of a transform from a position.
		// _position : the position of the transform in world space.
		snTransform(const snVec& _position);

		//Construction of a transform from a positon and orientation.
		// _position : the position of the transform in world space.
		// _orientation : orientation as a quaternion in world space.
		snTransform(const snVec& _position, const snVec& _orientation);

		//Construction of a transform from a a position, orientation and scale.
		// _position : the position of the transform in world space.
		// _orientation : orientation as a quaternion in world space.
		// _scale : scale of the transform in world space.
		snTransform(const snVec& _position, const snVec& _orientation, const snVec& _scale);

		//Construction of a transform from a a position, orientation and scale.
		// _position : the position of the transform in world space.
		// _orientation : orientation as a quaternion in world space.
		// _scale : scale of the transform in world space.
		// _localToWorld : matrix to transform a point from local space to world space.
		snTransform(const snVec& _position, const snVec& _orientation, const snVec& _scale, const snMatrix44f& _localToWorld);

		virtual ~snTransform();

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

		//Set the position of the transform.
		void setPosition(const snVec& _position);

		//Set the orientation of the transform using a quaternion.
		void setOrientation(const snVec& _orientation);

		//Set the orientation of the transform using euler angles.
		void setEulerAngles(const snVec& _eulerAngles);

		//Set the scale of the transform.
		void setScale(const snVec& _scale);
	};

	void snTransformMultiply(const snTransform& _first, const snTransform& _second, snTransform& result);
}
#endif //ifndef SN_TRANSFORM_H