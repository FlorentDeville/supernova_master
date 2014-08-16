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

#include "EntitySkybox.h"

#include "Graphics.h"
#include "Camera.h"
#include "D3D.h"

#include "GfxEntityPlan.h"

#include <DirectXMath.h>
using namespace DirectX;

namespace Devil
{
	EntitySkybox::EntitySkybox(IWorldEntity* _target, float _size, const XMFLOAT4& _color) : m_target(_target), m_size(_size)
	{
		m_gfx = GRAPHICS->getBox();
		m_color = XMVectorSet(_color.x, _color.y, _color.z, _color.w);
		m_texture = GRAPHICS->getTexChecker();
	}

	EntitySkybox::~EntitySkybox(){}

	void EntitySkybox::update()
	{

	}
	
	void EntitySkybox::render()
	{
		//get view and projection matrices
		XMMATRIX viewMatrix, projectionMatrix, world;
		GRAPHICS->getCamera()->GetViewMatrix(viewMatrix);
		GRAPHICS->getDirectXWrapper()->getProjectionMatrix(projectionMatrix);

		XMVECTOR center = m_target->getPosition();
		XMMATRIX offset = XMMatrixTranslationFromVector(center);

		float halfSize = m_size * 0.5f;

		XMMATRIX translate = XMMatrixTranslation(0, -halfSize, 0);
		XMMATRIX rotation;
		XMMATRIX scale = XMMatrixScaling(m_size, 1, m_size);

		//bottom
		world = scale * translate * offset;
		m_gfx->render(world, viewMatrix, projectionMatrix, m_color, m_texture, m_wireframe);

		//top
		world = scale * XMMatrixTranslation(0, halfSize, 0) * offset;
		m_gfx->render(world, viewMatrix, projectionMatrix, m_color, m_texture, m_wireframe);

		//right
		rotation = scale * XMMatrixRotationRollPitchYaw(0, 0, 3.14f * 0.5f);
		world = translate * (rotation * offset);
		m_gfx->render(world, viewMatrix, projectionMatrix, m_color, m_texture, m_wireframe);

		//left
		rotation = scale * XMMatrixRotationRollPitchYaw(0, 0, -3.14f * 0.5f);
		world = translate * (rotation * offset);
		m_gfx->render(world, viewMatrix, projectionMatrix, m_color, m_texture, m_wireframe);

		//front
		rotation = scale * XMMatrixRotationRollPitchYaw(3.14f * 0.5f, 0, 0);
		world = translate * (rotation * offset);
		m_gfx->render(world, viewMatrix, projectionMatrix, m_color, m_texture, m_wireframe);

		//back
		rotation = scale * XMMatrixRotationRollPitchYaw(-3.14f * 0.5f, 0, 0);
		world = translate * (rotation * offset);
		m_gfx->render(world, viewMatrix, projectionMatrix, m_color, m_texture, m_wireframe);
	}
}