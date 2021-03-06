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

#include "ComponentBackground.h"
#include "snRigidbody.h"
#include "snQuaternion.h"

#include "Input.h"

using namespace Supernova::Vector;

namespace Devil
{
	ComponentBackground::ComponentBackground(snRigidbody* _background, snRigidbody* _origin, const snVec& _initialTranslation,
		const snVec& _initialOrientation) : 
		m_background(_background), m_origin(_origin)
	{
		m_transform.createRotationFromQuaternion(snQuaternionFromEuler(snVec4GetX(_initialOrientation), snVec4GetY(_initialOrientation), snVec4GetZ(_initialOrientation)));
		m_transform.m_r[3] = _initialTranslation;

		m_forward = snVec4Set(1, 0, 0, 0);
		m_left = snVec4Set(0, 0, 1, 0);
		m_isActive = true;
	}

	ComponentBackground::~ComponentBackground(){}

	void ComponentBackground::update(float _dt)
	{
		const float ROTATION_SPEED = 0.7f;
		snMatrix44f backgroundNewRotation;
		
		float realRotation = ROTATION_SPEED * _dt;

		float amountUpAndDown = INPUT->getMessage(Input::InputMessage::MOVE_UP_AND_DOWN);
		float amountLeftandRight = INPUT->getMessage(Input::InputMessage::MOVE_SIDEWAY);
		if (amountUpAndDown != 0)
		{
			computeOriginFrame(m_origin, m_forward, m_left);
			backgroundNewRotation.createRotation(m_left, realRotation * amountUpAndDown);
		}
		/*else if (INPUT->isKeyDown('S'))
		{
			computeOriginFrame(m_origin, m_forward, m_left);
			backgroundNewRotation.createRotation(m_left, -realRotation);
		}*/
		else if (amountLeftandRight != 0)
		{
			computeOriginFrame(m_origin, m_forward, m_left);
			backgroundNewRotation.createRotation(m_forward, realRotation * amountLeftandRight);
		}
		/*else if (INPUT->isKeyDown('Q'))
		{
			computeOriginFrame(m_origin, m_forward, m_left);
			backgroundNewRotation.createRotation(m_forward, -realRotation);
		}*/
		else
			return;

		//Compute the transform matrix of the monkey ball and its inverse
		snMatrix44f monkeyBallTransform;
		monkeyBallTransform.createTranslation(m_origin->getPosition());
		snMatrix44f invMonkeyBallTransform = monkeyBallTransform.inverse();

		//Compute a transformation matrix to go from the monkey ball to the background original position
		snMatrix44f originToBackground;
		snMatrixMultiply4(m_transform, invMonkeyBallTransform, originToBackground);

		//Compute the new transform of the background
		// monkeyBallTransform x backgroundNewRotation x originToBackground
		snMatrix44f temp;
		snMatrixMultiply4(backgroundNewRotation, monkeyBallTransform, temp);
		snMatrixMultiply4(originToBackground, temp, m_transform);

		//extract rotation and quaternion
		snVec trans = snMatrixGetTranslation(m_transform);
		snVec rot = snQuaternionFromMatrix(m_transform);

		m_background->setKinematicTransform(trans, rot);
		

	}

	void ComponentBackground::render()
	{

	}

	void* ComponentBackground::operator new(size_t _count)
	{
		return _aligned_malloc(_count, SN_ALIGN_SIZE);
	}

	void ComponentBackground::operator delete(void* _p)
	{
		_aligned_free(_p);
	}

	void ComponentBackground::computeOriginFrame(const snRigidbody* _origin, snVec& _forward, snVec& _left)
	{
		snVec up = snVec4Set(0, 1, 0, 0);

		float speed = Supernova::Vector::snVec3Norme(_origin->getLinearVelocity());
		if (speed > 0.1f)
		{
			_forward = _origin->getLinearVelocity();
			Supernova::Vector::snVec3Normalize(_forward);
			_left = snVec3Cross(_forward, up);
		}
	}
}