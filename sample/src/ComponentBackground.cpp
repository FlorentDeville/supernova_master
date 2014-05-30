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
#include "snActorDynamic.h"
#include "snQuaternion.h"

#include "Input.h"

using namespace Supernova::Vector;

namespace Devil
{
	ComponentBackground::ComponentBackground(snActorDynamic* _background, snActorDynamic* _origin, const snVec& _initialTranslation,
		const snVec& _initialOrientation) : 
		m_background(_background), m_origin(_origin), m_orientation(_initialOrientation), m_translation(_initialTranslation)
	{}

	ComponentBackground::~ComponentBackground(){}

	void ComponentBackground::update(float _dt)
	{
		const float ROTATION_SPEED = 0.01f;
		if (INPUT->isKeyDown('M'))
		{
			snVec4SetZ(m_orientation, snVec4GetZ(m_orientation) + ROTATION_SPEED);
		}
		else if (INPUT->isKeyDown('L'))
		{
			snVec4SetZ(m_orientation, snVec4GetZ(m_orientation) - ROTATION_SPEED);
		}

		if (INPUT->isKeyDown('P'))
		{
			snVec4SetX(m_orientation, snVec4GetX(m_orientation) + ROTATION_SPEED);
		}
		else if (INPUT->isKeyDown('O'))
		{
			snVec4SetX(m_orientation, snVec4GetX(m_orientation) - ROTATION_SPEED);
		}

		//move the entity to its initial position
		snMatrix44f initialTranslation;
		initialTranslation.createTranslation(m_translation);

		//move the rotation axis into the position of the origin of the entity
		snMatrix44f backgroundTransform;
		snMatrixCreateTransform(m_background->getOrientationMatrix(), m_background->getPosition(), backgroundTransform);
		snMatrix44f backgroundInvTransform = backgroundTransform.inverse();

		snVec originInBackgroundFrame = snMatrixTransform4(m_origin->getPosition(), backgroundInvTransform);
		snMatrix44f originTransform, invOriginTransform;
		originTransform.createTranslation(originInBackgroundFrame);
		invOriginTransform.createTranslation(originInBackgroundFrame * snVec4Set(-1, -1, -1, 1));

		//compute the rotation
		snVec q = snQuaternionFromEuler(snVec4GetX(m_orientation), snVec4GetY(m_orientation), snVec4GetZ(m_orientation));
		snMatrix44f backgroundRotation;
		backgroundRotation.createRotationFromQuaternion(q);

		//to move to the final position : initialTranslation -> rotation -> backgroundInvTransform
		snMatrix44f temp, temp2, transform;
		snMatrixMultiply4(originTransform, initialTranslation, temp);
		snMatrixMultiply4(backgroundRotation, temp, temp2);
		snMatrixMultiply4(invOriginTransform, temp2, transform);

		//extract rotation and quaternion
		snVec trans = snMatrixGetTranslation(transform);
		snVec rot = snQuaternionFromMatrix(transform);

		m_background->setKinematicTransform(trans, rot);
		

	}

	void ComponentBackground::render()
	{

	}
}