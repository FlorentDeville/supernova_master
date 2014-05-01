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

#include "EntityFixedConstraint.h"
#include "GfxEntitySphere.h"
#include "GfxEntityBox.h"
#include "Graphics.h"
#include "D3D.h"
#include "Camera.h"

#include "snFixedConstraint.h"
#include "snMath.h"
#include "snIActor.h"
using namespace Supernova;

namespace Devil
{

	EntityFixedConstraint::EntityFixedConstraint() : IWorldEntity(), m_constraint(0)
	{}

	EntityFixedConstraint::~EntityFixedConstraint()
	{}

	bool EntityFixedConstraint::initialize(const snFixedConstraint* _constraint)
	{
		assert(_constraint != 0);

		m_constraint = _constraint;
		m_sphere = GRAPHICS->createSphere(1, Colors::Green);
		m_box = GRAPHICS->createBox(XMFLOAT3(1.f, m_constraint->getDistance(), 1.f), XMFLOAT4(0, 0.5f, 0, 1.f));
		return true;
	}

	void EntityFixedConstraint::update()
	{
	
	}

	void EntityFixedConstraint::render()
	{
		XMMATRIX viewMatrix, projectionMatrix;
		GRAPHICS->getCamera()->GetViewMatrix(viewMatrix);
		GRAPHICS->getDirectXWrapper()->getProjectionMatrix(projectionMatrix);

		//show the fixed point as a sphere
		XMMATRIX fixedPointTransform = XMMatrixTranslationFromVector(m_constraint->getFixedPosition().m_vec);
		m_sphere->render(fixedPointTransform, viewMatrix, projectionMatrix);

		//show the link between the fixed point and the actor
		snVector4f up, left, forward;
		up = m_constraint->getActor()->getPosition() - m_constraint->getFixedPosition();
		up.normalize();
		up[3] = 0;
		computeBasis(up, left, forward);
		XMMATRIX linkRotate;
		linkRotate.r[0] = left.m_vec;
		linkRotate.r[1] = up.m_vec;
		linkRotate.r[2] = -forward.m_vec;
		linkRotate.r[3] = _mm_set_ps(1, 0, 0, 0);

		XMMATRIX offset = XMMatrixTranslation(0, m_constraint->getDistance() * 0.5f, 0);

		XMMATRIX linkTransform = offset * linkRotate * fixedPointTransform;
		m_box->render(linkTransform, viewMatrix, projectionMatrix);
	}
}