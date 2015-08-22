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

#include "Graphics.h"
#include "D3D.h"
#include "Camera.h"
#include "GfxEntitySphere.h"
#include "GfxEntityBox.h"

#include "EntityPointToPointConstraint.h"

#include "snMath.h"
#include "snRigidbody.h"
#include "snPointToPointConstraint.h"

using Supernova::snPointToPointConstraint;

using namespace DirectX;

namespace Devil
{
	EntityPointToPointConstraint::EntityPointToPointConstraint() : IWorldEntity(), m_constraint(0)
	{
		m_box = 0;
		m_sphere = 0;
	}

	EntityPointToPointConstraint::~EntityPointToPointConstraint()
	{}

	bool EntityPointToPointConstraint::initialize(const snPointToPointConstraint* _constraint)
	{
		//save the constraint
		m_constraint = _constraint;

		//create the geometry for actor a and b of the constraint.
		m_sphere = GRAPHICS->getSphere();
		m_box = (GfxEntityBox*)GRAPHICS->getBox();

		return true;
	}

	void EntityPointToPointConstraint::update()
	{}

	void EntityPointToPointConstraint::render()
	{
		XMMATRIX viewMatrix, projectionMatrix;
		GRAPHICS->getCamera()->GetViewMatrix(viewMatrix);
		GRAPHICS->getDirectXWrapper()->getProjectionMatrix(projectionMatrix);

		snRigidbody const * const * actors = m_constraint->getActors();

		for (int i = 0; i < 2; ++i)
		{
			//show the fixed point as a sphere
			XMMATRIX fixedPointTransform = XMMatrixTranslationFromVector(m_constraint->getWPivot()[i]);
			m_sphere->render(fixedPointTransform, viewMatrix, projectionMatrix, Colors::Green, m_texture, m_wireframe);

			//show the link between the fixed point and the actor
			snVec up, left, forward;
			up = actors[i]->getPosition() - m_constraint->getWPivot()[i];
			up = XMVectorSetW(up, 0);

			Supernova::Vector::snVec3Normalize(up);
			computeBasis(up, left, forward);
			XMMATRIX linkRotate;
			linkRotate.r[0] = left;
			linkRotate.r[1] = up;
			linkRotate.r[2] = -forward;
			linkRotate.r[3] = _mm_set_ps(1, 0, 0, 0);

			float length = Supernova::Vector::snVec3Norme(m_constraint->getOffset()[i]);
			XMMATRIX offset = XMMatrixTranslation(0, length * 0.5f, 0);

			XMMATRIX scale = XMMatrixScaling(1, length, 1);
			XMMATRIX linkTransform = scale * offset * linkRotate * fixedPointTransform;
			m_box->render(linkTransform, viewMatrix, projectionMatrix, Colors::Green, m_texture, m_wireframe);
		}
	}
}