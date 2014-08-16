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
#include "EntityBox.h"

#include "Graphics.h"
#include "D3D.h"
#include "Camera.h"
#include "GfxEntityBox.h"

#include "snVec.h"
#include "snRigidbody.h"

using namespace Supernova::Vector;
using namespace DirectX;

namespace Devil
{
	EntityBox::EntityBox() : m_gfx(0)
	{
	}


	EntityBox::~EntityBox()
	{
	}

	bool EntityBox::initialize(const XMFLOAT3& _size, const XMFLOAT4& _color)
	{
		m_color = XMVectorSet(_color.x, _color.y, _color.z, _color.w);
		m_size = _size;
		m_gfx = GRAPHICS->getBox();
		return true;
	}

	void EntityBox::update()
	{
		m_position = m_actor->getTransform().getPosition();
	}

	void EntityBox::render()
	{
		XMMATRIX scaling = XMMatrixScaling(m_size.x, m_size.y, m_size.z);

		const snMatrix44f& physicTransform = m_actor->getTransform().getLocalToWorld();
		XMMATRIX dxPhysicTransform;
		dxPhysicTransform.r[0] = physicTransform[0];
		dxPhysicTransform.r[1] = physicTransform[1];
		dxPhysicTransform.r[2] = physicTransform[2];
		dxPhysicTransform.r[3] = physicTransform[3];
		XMMATRIX transform = scaling * dxPhysicTransform;
		
		XMMATRIX viewMatrix, projectionMatrix;
		GRAPHICS->getCamera()->GetViewMatrix(viewMatrix);
		GRAPHICS->getDirectXWrapper()->getProjectionMatrix(projectionMatrix);

		m_gfx->render(transform, viewMatrix, projectionMatrix, m_color, m_texture, m_wireframe);
	}
}